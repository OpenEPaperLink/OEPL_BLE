
# ATC BLE OEPL uploader by @ATC1441 (Aaron Christophel) 05.01.2025
# BLE Image uplaod POC including Zlib compression
# Make sure to install all missing packagage for python via pip install xxx

import asyncio
import struct
from bleak_retry_connector import BleakClientWithServiceCache
import zlib
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import datetime

def create_image_with_date(width, height, multi_color=False, compressed=False):
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    date_text = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        font = ImageFont.truetype("arial.ttf", size=min(width, height) // 10)
    except IOError:
        font = ImageFont.load_default()
    text_size = draw.textbbox((0, 0), date_text, font=font)
    text_xy = (
        (image.width - (text_size[2] - text_size[0])) // 2,
        (image.height - (text_size[3] - text_size[1])) // 2,
    )
    draw.text(text_xy, date_text, fill="black", font=font)
    if multi_color:
        draw.text((text_xy[0], text_xy[1] + 15), "Second Color", fill="red", font=font)
    pixel_array = np.array(image)
    height, width, _ = pixel_array.shape
    byte_data = []
    byte_data_red = []
    current_byte = 0
    current_byte_red = 0
    bit_position = 7
    for y in range(height):
        for x in range(width):
            r, g, b = pixel_array[y, x]
            luminance = 0.2126 * r + 0.7152 * g + 0.0722 * b
            if luminance > 128:
                current_byte |= (1 << bit_position)
            elif r > 170:
                current_byte_red |= (1 << bit_position)
            bit_position -= 1
            if bit_position < 0:
                byte_data.append(~current_byte & 0xFF)
                byte_data_red.append(current_byte_red)
                current_byte = 0
                current_byte_red = 0
                bit_position = 7
    if bit_position != 7:
        byte_data.append(~current_byte & 0xFF)
        byte_data_red.append(current_byte_red)
    bpp_array = bytearray(byte_data)
    if multi_color:
        bpp_array += bytearray(byte_data_red)
    if compressed:
        print("Doing compression")
        buffer = bytearray(6)
        buffer[0] = 6
        buffer[1] = width & 0xFF
        buffer[2] = (width >> 8) & 0xFF
        buffer[3] = height & 0xFF
        buffer[4] = (height >> 8) & 0xFF
        buffer[5] = 0x02 if multi_color else 0x01
        buffer += bpp_array
        the_compressor = zlib.compressobj(wbits=12)
        compressed_data = the_compressor.compress(buffer)
        compressed_data += the_compressor.flush()
        return 0x30, struct.pack('<I', len(buffer)) + compressed_data
    return 0x21 if multi_color else 0x20, bpp_array

shutdown = False
def stop():
    global shutdown
    shutdown = True
    print('Exiting')

class BLEHandler:
    def __init__(self):
        self.client = None
        self.device_name = None
        self.reconnect_tries = 0
        self.max_reconnect_tries = 5
        self.packets = []
        self.packet_index = 0
        self.img_array = b""
        self.img_array_len = 0

    async def connect(self, device_name=None):
        self.device_name = device_name
        try:
            self.client = BleakClientWithServiceCache(self.device_name, timeout=16.0)
            await self.client.connect()
            print(f"Connected to {self.device_name}")
            await self.client.start_notify("00001337-0000-1000-8000-00805f9b34fb", self.notification_handler)
        except Exception as e:
            print(f"Connection failed: {e}")
            await self.handle_reconnect()

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected")

    async def handle_reconnect(self):
        if self.reconnect_tries < self.max_reconnect_tries:
            self.reconnect_tries += 1
            print(f"Reconnect attempt {self.reconnect_tries}")
            await asyncio.sleep(1)
            await self.connect(self.device_name)
        else:
            print("Maximum reconnect attempts reached.")            

    async def notification_handler(self, sender, data):
        print(f"Notification from {sender}: {data.hex()}")
        await self.img_state_handle(data)

    async def send_command(self, cmd):
        try:
            await self.client.write_gatt_char("00001337-0000-1000-8000-00805f9b34fb", cmd, response=False)
            print(f"Sent command: {cmd.hex()}")
        except Exception as e:
            print(f"Write failed: {e}")
            await asyncio.sleep(0.5)
            await self.send_command(cmd)

    async def send_img(self, image_type, img_hex):
        if self.client:
            if self.client.is_connected:
                self.img_array = img_hex
                self.img_array_len = len(self.img_array)
                print(f"Sending image of size {self.img_array_len} bytes")
                data_info = self.create_data_info(255, self.crc32(self.img_array), self.img_array_len, image_type, 0, 0)
                await self.send_command(bytes.fromhex("0064") + data_info)            

    async def send_block_data(self, block_id):
        print("Building block id:", block_id)
        block_size = 4096
        block_start = block_id * block_size
        block_end = block_start + block_size
        block_data = self.img_array[block_start:block_end]
        crcBlock = sum(block_data[0:len(block_data)]) & 0xffff
        buffer = bytearray(4)
        buffer[0] = len(block_data) & 0xff
        buffer[1] = (len(block_data) >> 8) & 0xff
        buffer[2] = crcBlock & 0xff
        buffer[3] = (crcBlock >> 8) & 0xff
        block_data = buffer + block_data        
        print(f"Block data length: {len(block_data)} bytes")
        packet_count = (len(block_data) + 229) // 230
        self.packets = []
        for i in range(packet_count):
            start = i * 230
            end = start + 230
            slice_data = block_data[start:end]
            packet = self.create_block_part(block_id, i, slice_data)
            self.packets.append(packet)
        print(f"Total packets created: {len(self.packets)}")
        self.packet_index = 0
        await self.send_next_block_part()

    async def send_next_block_part(self):
        await self.send_command(bytes.fromhex("0065") + self.packets[self.packet_index])

    def create_block_part(self, block_id, part_id, data):
        max_data_size = 230
        data_length = len(data)
        if data_length > max_data_size:
            raise ValueError("Data length exceeds maximum allowed size for a packet.")
        buffer = bytearray(3 + max_data_size)
        buffer[1] = block_id & 0xFF
        buffer[2] = part_id & 0xFF
        buffer[3:3 + data_length] = data
        buffer[0] = sum(buffer[1:3 + data_length]) & 0xFF
        return buffer

    def create_data_info(self, checksum, data_ver, data_size, data_type, data_type_argument, next_check_in):
        return struct.pack(
            "<BQIBBH",
            checksum,
            data_ver,
            data_size,
            data_type,
            data_type_argument,
            next_check_in,
        )

    def crc32(self, input_data):
        table = []
        polynomial = 0xEDB88320
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ polynomial
                else:
                    crc >>= 1
            table.append(crc)
        crc = 0xFFFFFFFF
        for byte in input_data:
            table_index = (crc ^ byte) & 0xFF
            crc = (crc >> 8) ^ table[table_index]
        return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF
    
    async def img_state_handle(self, data):
        response_code = data[:2].hex().upper()
        if response_code == "00C6":
            print("Received block request")
            block_id = data[11]
            await self.send_block_data(block_id)
        elif response_code == "00C4":
            print("Block part acknowledged")
            await self.send_next_block_part()
        elif response_code == "00C5":
            print("Block part acknowledged")
            if self.packet_index > len(self.packets):
                print("Something went wrong, not so many packets available")
                await self.disconnect()
                stop()
            self.packet_index += 1
            await self.send_next_block_part()
        elif response_code == "00C7":
            print("Image will now be displayed")
            await self.disconnect()
            stop()
        elif response_code == "00C8":
            print("Image already displayed")
            await self.disconnect()
            stop()
        else:
            print(f"Unknown response: {response_code}")
            await self.disconnect()
            stop()

async def main():
    # Enter you screen size and MAC here
    dataType, pixel_array = create_image_with_date(184, 384, multi_color=True, compressed=True)
    print(f"DataType: {dataType:0x} DataLen: {len(pixel_array)}")
    ble_handler = BLEHandler()
    await ble_handler.connect("33:6E:71:15:24:41")
    if ble_handler.client.is_connected:
        await ble_handler.send_img(dataType, pixel_array)
        while not shutdown:
            # in this loop we wait for the image upload, a timeout would be nice here
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
