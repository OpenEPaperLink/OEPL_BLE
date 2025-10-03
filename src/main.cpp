#include <Arduino.h>
#include <bluefruit.h>
#include "main.h"
#include <SPI.h>
#include <uzlib.h>
#include <bb_epaper.h>

BBEPAPER epd(EP426_800x480);

#define Led_RED 26
#define Led_Green 30
#define Led_Blue 06
#define CS_PIN 44
#define DC_PIN 31
#define RESET_PIN 15
#define BUSY_PIN 29
#define CLK_PIN 45
#define MOSI_PIN 47
#define PowerPin 43

BLEDfu bledfu;

BLEService imageService("1337");
BLECharacteristic imageCharacteristic("1337",BLEWrite | BLENotify, 512);

ImageData currentImage = {0};
uint8_t currentBlockId = 0;
uint8_t currentPacketId = 0;
uint8_t expectedPackets = 0;
uint8_t receivedPackets = 0;

static unsigned long lastLog = 0;

void setup() {
    pinMode(Led_RED, OUTPUT);
    pinMode(Led_Green, OUTPUT);
    pinMode(Led_Blue, OUTPUT);
    digitalWrite(Led_RED, HIGH);
    digitalWrite(Led_Green, HIGH);
    digitalWrite(Led_Blue, HIGH);
    delay(100);
    digitalWrite(Led_RED, LOW);
    delay(100);
    digitalWrite(Led_RED, HIGH);
    digitalWrite(Led_Green, LOW);
    delay(100);
    digitalWrite(Led_Green, HIGH);
    digitalWrite(Led_Blue, LOW);
    delay(100);
    digitalWrite(Led_Blue, HIGH);
    delay(100);
    digitalWrite(Led_RED, LOW);
    digitalWrite(Led_Green, LOW);
    digitalWrite(Led_Blue, LOW);
    delay(100);
    digitalWrite(Led_RED, HIGH);
    digitalWrite(Led_Green, HIGH);
    digitalWrite(Led_Blue, HIGH);

    Serial.begin(115200);
    bool usb_connected = bitRead(NRF_POWER->USBREGSTATUS, 0);
    writeSerial("=== BLE OEPL Device Starting ===");

    writeSerial("Enabeling power management...");
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, HIGH);
    writeSerial("Initializing BLE...");
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.setTxPower(8);
    if (!Bluefruit.begin(1, 0))
    {
        writeSerial("ERROR: Failed to initialize BLE!");
        return;
    }
    bledfu.begin();
    writeSerial("BLE DFU initialized successfully");
    writeSerial("BLE initialized successfully");
    writeSerial("Setting up BLE service 0x1337...");
    imageService.begin();
    writeSerial("BLE service started");
    imageCharacteristic.setWriteCallback(imageDataWritten);
    writeSerial("BLE write callback set");
    imageCharacteristic.begin();
    writeSerial("BLE characteristic started");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    writeSerial("BLE callbacks registered");

    String deviceName = "OEPL" + getChipIdHex();
    Bluefruit.setName(deviceName.c_str());
    writeSerial("Device name set to: " + deviceName);

    writeSerial("Configuring power management...");
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    writeSerial("Power management configured");

    writeSerial("Setting up battery monitoring...");
    analogReference(AR_INTERNAL_2_4);
    analogReadResolution(ADC_RESOLUTION);
    pinMode(PIN_VBAT, INPUT);
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);
    writeSerial("Battery monitoring configured");
    writeSerial("Initializing display");
    initDisplay();
    writeSerial("Display initialized");

    writeSerial("=== Setup completed successfully ===");
    writeSerial("Configuring BLE advertising...");
    // Disable automatic UUID advertising
    Bluefruit.Advertising.clearData();
    // Now add only the fields you want
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    //Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addName();

//THIS IS A PLACEHOLDER FOR THE ACTUAL PAYLOAD
//carefull, the size is limited
uint8_t msd_payload[13];
uint16_t msd_cid = 0x1337;
memset(msd_payload, 0, sizeof(msd_payload));
memcpy(msd_payload, (uint8_t*)&msd_cid, sizeof(msd_cid));
msd_payload[2] = 0x02;
msd_payload[3] = 0x36;
msd_payload[4] = 0x00;
msd_payload[5] = 0x6C;
msd_payload[6] = 0x00;
msd_payload[7] = 0xC3;
msd_payload[8] = 0x01;
msd_payload[9] = 0xB9;
msd_payload[10] = 0x0B;
msd_payload[11] = 0x12;
msd_payload[12] = 0xA6;
Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, sizeof(msd_payload));


    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 1024);
    Bluefruit.Advertising.setFastTimeout(10);
    writeSerial("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
    writeSerial("BLE advertising started - waiting for connections...");
}

void pwrmgm(bool onoff){
    if(onoff){
        digitalWrite(PowerPin, HIGH);
        pinMode(RESET_PIN, OUTPUT);
        pinMode(CS_PIN, OUTPUT);
        pinMode(DC_PIN, OUTPUT);
        pinMode(CLK_PIN, OUTPUT);
        pinMode(MOSI_PIN, OUTPUT);
        delay(200);
    }
    else{
        SPI.end();
        pinMode(RESET_PIN, INPUT);
        pinMode(CS_PIN, INPUT);
        pinMode(DC_PIN, INPUT);
        pinMode(CLK_PIN, INPUT);
        pinMode(MOSI_PIN, INPUT);
        digitalWrite(PowerPin, LOW);
    }
}

void loop() {
    if (currentImage.ready) {
        writeSerial("Processing received image...");
        displayReceivedImage();
        currentImage.ready = false;
        if (currentImage.data) {
            free(currentImage.data);
            currentImage.data = nullptr;
            writeSerial("Image memory freed");
        }
    }
    writeSerial(String(millis() / 100));
    delay(2000);
}

void writeSerial(String message, bool newLine) {
        if (newLine == true)
        {
            Serial.println(message);
        }
        else
        {
            Serial.print(message);
        }
}

void initDisplay() {
    writeSerial("=== Initializing Display ===");
    pwrmgm(true);
    epd.initIO(DC_PIN, RESET_PIN, BUSY_PIN, CS_PIN, MOSI_PIN, CLK_PIN);
    epd.allocBuffer();
    epd.fillScreen(BBEP_WHITE);
    epd.setTextColor(BBEP_BLACK, BBEP_WHITE);
    epd.setFont(FONT_12x16);
    String chipId = getChipIdHex();
    writeSerial("Chip ID for display: " + chipId);
    epd.setCursor(100, 100); 
    epd.print("openepaperlink.org");
    epd.setCursor(100, 200); 
    epd.print("ID:" + chipId);
    epd.setCursor(100, 300); 
    epd.print("Ready");
    epd.writePlane();
    epd.refresh(REFRESH_FULL, true);
    pwrmgm(false);
}

void connect_callback(uint16_t conn_handle) {
    writeSerial("=== BLE CLIENT CONNECTED ===");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    writeSerial("=== BLE CLIENT DISCONNECTED ===");
    writeSerial("Disconnect reason: " + String(reason));
}

String getChipIdHex() {
    uint32_t id1 = NRF_FICR->DEVICEID[0];
    uint32_t id2 = NRF_FICR->DEVICEID[1]; 
    uint32_t last3Bytes = id2 & 0xFFFFFF;
    String hexId = String(last3Bytes, HEX);
    hexId.toUpperCase();
    while (hexId.length() < 6) {
        hexId = "0" + hexId;
    }
    writeSerial("Chip ID: " + String(id1, HEX) + String(id2, HEX));
    writeSerial("Using last 3 bytes: " + hexId);
    return hexId;
}

void imageDataWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    handleImageCommand(data, len);
}

void handleImageCommand(uint8_t* data, uint16_t len) {
    if (len < 2) {
        writeSerial("ERROR: Command too short (" + String(len) + " bytes)");
        return;
    }
    
    uint16_t command = (data[0] << 8) | data[1];  // Fixed byte order
    writeSerial("Processing command: 0x" + String(command, HEX));
    switch (command) {
        case 0x0011: // Read Dynamic Config command
            writeSerial("=== READ DYNAMIC CONFIG COMMAND (0x0011) ===");
            handleReadDynamicConfig();
            break;
        case 0x0064: // Image info command
            writeSerial("=== IMAGE INFO COMMAND (0x0064) ===");
            handleImageInfo(data + 2, len - 2);
            break;
        case 0x0065: // Block data command
            writeSerial("=== BLOCK DATA COMMAND (0x0065) ===");
            handleBlockData(data + 2, len - 2);
            break;
        case 0x0002: // Start sending packets for current block
            {
                writeSerial("=== START PACKETS COMMAND (0x0002) ===");
                writeSerial("Web tool ready to send packets for block " + String(currentImage.currentBlock));
            }
            break;
        case 0x0003: // Finalization command
            {
                writeSerial("=== FINALIZATION COMMAND (0x0003) ===");
                writeSerial("Image transfer finalized");
                if (currentImage.data && currentImage.received > 0) {
                    currentImage.ready = true;
                    writeSerial("Image marked as ready for display");
                }
                uint8_t ackResponse[] = {0x63, 0x00};
                sendResponse(ackResponse, sizeof(ackResponse));
            }
            break;
        case 0x0005: // Display info command
            writeSerial("=== DISPLAY INFO COMMAND (0x0005) ===");
            handleDisplayInfo();
            break;
        default:
            writeSerial("ERROR: Unknown command: 0x" + String(command, HEX));
            writeSerial("Expected: 0x0011 (read config), 0x0064 (image info), 0x0065 (block data), or 0x0003 (finalize)");
            break;
    }
}

void handleImageInfo(uint8_t* data, uint16_t len) {
    writeSerial("Parsing image info...");
    if (len < 16) {
        writeSerial("ERROR: Image info too short (" + String(len) + " bytes, need 16)");
        return;
    }
    uint8_t checksum = data[0];
    uint32_t dataSize = *(uint32_t*)(data + 9);
    uint16_t dataType = *(uint16_t*)(data + 13);
    uint16_t dataTypeArg = *(uint16_t*)(data + 15);
    uint16_t nextCheckIn = *(uint16_t*)(data + 17);
    writeSerial("Image info parsed:");
    writeSerial("  Checksum: 0x" + String(checksum, HEX));
    writeSerial("  Data size: " + String(dataSize) + " bytes");
    writeSerial("  Data type: 0x" + String(dataType, HEX));
    writeSerial("  Data type arg: 0x" + String(dataTypeArg, HEX));
    writeSerial("  Next check in: " + String(nextCheckIn));
    writeSerial("Allocating memory for image data...");
    if (currentImage.data) {
        writeSerial("Freeing previous image data");
        free(currentImage.data);
    }
    if (currentImage.blocksReceived) {
        writeSerial("Freeing previous block tracking arrays");
        free(currentImage.blocksReceived);
    }
    if (currentImage.blockBytesReceived) {
        free(currentImage.blockBytesReceived);
    }
    if (currentImage.blockPacketsReceived) {
        free(currentImage.blockPacketsReceived);
    }
    uint32_t bufferSize = dataSize + 1024; // Add 1KB buffer for safety
    currentImage.data = (uint8_t*)malloc(bufferSize);
    if (!currentImage.data) {
        writeSerial("ERROR: Failed to allocate " + String(bufferSize) + " bytes for image");
        return;
    }
    writeSerial("Memory allocated successfully: " + String(bufferSize) + " bytes (requested: " + String(dataSize) + " bytes)");
    currentImage.size = dataSize;
    currentImage.received = 0;
    currentImage.dataType = dataType;
    currentImage.isCompressed = (dataType == 0x30);
    currentImage.ready = false;
    currentImage.width = epd.width();
    currentImage.height = epd.height();
    writeSerial("Image state initialized:");
    writeSerial("  Size: " + String(currentImage.size) + " bytes");
    writeSerial("  Type: 0x" + String(currentImage.dataType, HEX));
    writeSerial("  Compressed: " + String(currentImage.isCompressed ? "Yes" : "No"));
    writeSerial("  Dimensions: " + String(currentImage.width) + "x" + String(currentImage.height));
    const uint32_t BLOCK_DATA_SIZE = 4096;
    currentImage.totalBlocks = (dataSize + BLOCK_DATA_SIZE - 1) / BLOCK_DATA_SIZE; // Ceiling division
    currentImage.currentBlock = 0;
    writeSerial("Calculated blocks needed: " + String(currentImage.totalBlocks) + " (data size: " + String(dataSize) + " bytes, block size: " + String(BLOCK_DATA_SIZE) + " bytes)");
    currentImage.blocksReceived = (bool*)malloc(currentImage.totalBlocks * sizeof(bool));
    currentImage.blockBytesReceived = (uint32_t*)malloc(currentImage.totalBlocks * sizeof(uint32_t));
    currentImage.blockPacketsReceived = (uint32_t*)malloc(currentImage.totalBlocks * sizeof(uint32_t));
    if (currentImage.blocksReceived && currentImage.blockBytesReceived && currentImage.blockPacketsReceived) {
        for (uint32_t i = 0; i < currentImage.totalBlocks; i++) {
            currentImage.blocksReceived[i] = false;
            currentImage.blockBytesReceived[i] = 0;
            currentImage.blockPacketsReceived[i] = 0;
        }
        writeSerial("Block tracking initialized for " + String(currentImage.totalBlocks) + " blocks");
    } else {
        writeSerial("ERROR: Failed to allocate block tracking arrays");
    }
    writeSerial("Sending block request for block 0...");
    uint8_t response[19] = {0x00, 0xC6}; // Command: 0x00C6
    response[2] = 0x00; // Checksum
    response[3] = 0x00; response[4] = 0x00; response[5] = 0x00; response[6] = 0x00;
    response[7] = 0x00; response[8] = 0x00; response[9] = 0x00; response[10] = 0x00;
    response[11] = 0; // Block ID 0
    response[12] = 0x00; // Type
    response[13] = 0xFF; response[14] = 0xFF; response[15] = 0xFF;
    response[16] = 0xFF; response[17] = 0xFF; response[18] = 0xFF;
    sendResponse(response, sizeof(response));
}

void handleBlockData(uint8_t* data, uint16_t len) {
    writeSerial("Processing block data...");
    if (len < 4) {
        writeSerial("ERROR: Block data too short (" + String(len) + " bytes, need 4)");
        return;
    }
    uint8_t checksum = data[0];
    uint8_t blockId = data[1];
    uint8_t packetId = data[2];
    uint8_t* blockData = data + 3;
    uint16_t dataLen = len - 3;
    writeSerial("  Data length: " + String(dataLen) + " bytes");
    writeSerial("Block data parsed:");
    writeSerial("  Block ID: " + String(blockId));
    writeSerial("  Packet ID: " + String(packetId));
    writeSerial("  Data length: " + String(dataLen) + " bytes");
    writeSerial("  Checksum: 0x" + String(checksum, HEX));
    uint32_t maxSize = currentImage.size + 1024;
    if (currentImage.data && currentImage.received + dataLen <= maxSize) {
        writeSerial("Copying data to image buffer...");
        static uint32_t lastBlockId = 0xFFFFFFFF;
        if (blockId != lastBlockId) {
            writeSerial("New block " + String(blockId) + " detected, skipping first 4 bytes");
            lastBlockId = blockId;
            if (dataLen > 4) {
                memcpy(currentImage.data + currentImage.received, blockData + 4, dataLen - 4);
                currentImage.received += (dataLen - 4);
            }
        } else {
            uint32_t expectedBytes = (blockId == currentImage.totalBlocks - 1) ? 
                (currentImage.size - blockId * 4096) : 4096;
            uint32_t remainingBytes = expectedBytes - (currentImage.received - blockId * 4096);
            if (remainingBytes < dataLen) {
                writeSerial("Last packet of block " + String(blockId) + ", removing " + String(dataLen - remainingBytes) + " bytes of padding");
                memcpy(currentImage.data + currentImage.received, blockData, remainingBytes);
                currentImage.received += remainingBytes;
            } else {
                memcpy(currentImage.data + currentImage.received, blockData, dataLen);
                currentImage.received += dataLen;
            }
        }
        float progress = (float)currentImage.received / (float)currentImage.size * 100.0;
        writeSerial("Progress: " + String(currentImage.received) + "/" + String(currentImage.size) + " bytes (" + String(progress, 1) + "%)");
        if (currentImage.blockBytesReceived && currentImage.blockPacketsReceived && blockId < currentImage.totalBlocks) {
            currentImage.blockBytesReceived[blockId] += dataLen;
            currentImage.blockPacketsReceived[blockId]++;
            writeSerial("Block " + String(blockId) + " progress: " + String(currentImage.blockBytesReceived[blockId]) + " bytes, " + String(currentImage.blockPacketsReceived[blockId]) + " packets");
        }
        bool currentBlockComplete = false;
        if (currentImage.blockBytesReceived && blockId < currentImage.totalBlocks) {
            const uint32_t BLOCK_DATA_SIZE = 4096;
            uint32_t expectedBytes = (blockId == currentImage.totalBlocks - 1) ? 
                (currentImage.size - blockId * BLOCK_DATA_SIZE) : BLOCK_DATA_SIZE;
            writeSerial("DEBUG: Block " + String(blockId) + " check - received: " + String(currentImage.blockBytesReceived[blockId]) + ", expected: " + String(expectedBytes));
            if (currentImage.blockBytesReceived[blockId] >= expectedBytes) {
                currentImage.blocksReceived[blockId] = true;
                currentBlockComplete = true;
                writeSerial("Block " + String(blockId) + " completed (" + String(currentImage.blockBytesReceived[blockId]) + "/" + String(expectedBytes) + " bytes)");
            } else if (currentImage.blockBytesReceived[blockId] > expectedBytes) {
                writeSerial("WARNING: Block " + String(blockId) + " exceeded expected size (" + String(currentImage.blockBytesReceived[blockId]) + "/" + String(expectedBytes) + " bytes)");
            }
        }
        bool allBlocksComplete = true;
        if (currentImage.blocksReceived) {
            for (uint32_t i = 0; i < currentImage.totalBlocks; i++) {
                if (!currentImage.blocksReceived[i]) {
                    allBlocksComplete = false;
                    break;
                }
            }
        }
        if (allBlocksComplete) {
            writeSerial("=== IMAGE TRANSFER COMPLETE ===");
            writeSerial("Total bytes received: " + String(currentImage.received));
            writeSerial("Expected bytes: " + String(currentImage.size));
            writeSerial("All " + String(currentImage.totalBlocks) + " blocks received");
            
            // Verify we have complete data
            if (currentImage.received >= currentImage.size) {
                currentImage.ready = true;
                writeSerial("Sending completion response (0x00C7)...");
                uint8_t response[] = {0x00, 0xC7};
                sendResponse(response, sizeof(response));
            } else {
                writeSerial("ERROR: Incomplete data - not marking as ready");
                writeSerial("Missing " + String(currentImage.size - currentImage.received) + " bytes");
            }
        } else if (currentBlockComplete) {
            currentImage.currentBlock++;
            if (currentImage.currentBlock < currentImage.totalBlocks) {
                writeSerial("Requesting next block: " + String(currentImage.currentBlock) + "/" + String(currentImage.totalBlocks));
                uint8_t response[19] = {0x00, 0xC6};
                response[2] = 0x00; // Checksum
                response[3] = 0x00; response[4] = 0x00; response[5] = 0x00; response[6] = 0x00;
                response[7] = 0x00; response[8] = 0x00; response[9] = 0x00; response[10] = 0x00;
                response[11] = currentImage.currentBlock; // Block ID
                response[12] = 0x00; // Type
                response[13] = 0xFF; response[14] = 0xFF; response[15] = 0xFF;
                response[16] = 0xFF; response[17] = 0xFF; response[18] = 0xFF;
                sendResponse(response, sizeof(response));
            } else {
                writeSerial("ERROR: Requested block " + String(currentImage.currentBlock) + " exceeds total blocks " + String(currentImage.totalBlocks));
                uint8_t response[] = {0x00, 0xC5};
                sendResponse(response, sizeof(response));
            }
        } else {
            writeSerial("Sending acknowledgment (0x00C5) - requesting next packet...");
            uint8_t response[] = {0x00, 0xC5};
            sendResponse(response, sizeof(response));
        }
    } else {
        writeSerial("ERROR: Invalid block data or buffer overflow");
        writeSerial("  Image data available: " + String(currentImage.data ? "Yes" : "No"));
        writeSerial("  Would exceed size: " + String(currentImage.received + dataLen) + " > " + String(maxSize));
        uint8_t response[] = {0x00, 0xC8};
        sendResponse(response, sizeof(response));
    }
}

void sendResponse(uint8_t* response, uint8_t len) {
    writeSerial("Sending BLE response:");
    writeSerial("  Length: " + String(len) + " bytes");
    writeSerial("  Command: 0x" + String(response[0], HEX) + String(response[1], HEX));
    imageCharacteristic.notify(response, len);
    writeSerial("Response sent successfully");
}

uint32_t calculateCRC32(uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    uint32_t polynomial = 0xEDB88320;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

void displayReceivedImage() {
    writeSerial("=== DISPLAYING RECEIVED IMAGE ===");
    if (!currentImage.data || currentImage.size == 0) {
        writeSerial("ERROR: No image data to display");
        return;
    }
    writeSerial("Image data available:");
    writeSerial("  Size: " + String(currentImage.size) + " bytes");
    writeSerial("  Type: 0x" + String(currentImage.dataType, HEX));
    writeSerial("  Compressed: " + String(currentImage.isCompressed ? "Yes" : "No"));
    writeSerial("  Dimensions: " + String(currentImage.width) + "x" + String(currentImage.height));
    writeSerial("  Received: " + String(currentImage.received) + " bytes");
    pwrmgm(true);
    epd.initIO(DC_PIN, RESET_PIN, BUSY_PIN, CS_PIN, MOSI_PIN, CLK_PIN);
    epd.allocBuffer();
    drawImageData();
    epd.writePlane();
    epd.refresh(REFRESH_FULL, true);
    pwrmgm(false); // Power down display after update is complete
    writeSerial("=== IMAGE DISPLAY COMPLETE ===");
}

bool decompressImageData(uint8_t** output, uint32_t* outputSize) {
    writeSerial("Starting uzlib decompression...");
    writeSerial("Input data size: " + String(currentImage.size) + " bytes");
    writeSerial("First 16 bytes: " + String(currentImage.data[0], HEX) + " " + String(currentImage.data[1], HEX) + " " + String(currentImage.data[2], HEX) + " " + String(currentImage.data[3], HEX) + " ...");
    
    if (!currentImage.data || currentImage.size == 0) {
        writeSerial("ERROR: No data to decompress");
        return false;
    }
    
    // Estimate decompressed size (typically 3-10x larger than compressed)
    uint32_t estimatedSize = currentImage.size * 5;
    uint32_t bufferSize = estimatedSize;
    uint8_t* buffer = (uint8_t*)malloc(bufferSize);
    
    if (!buffer) {
        writeSerial("ERROR: Failed to allocate memory for decompression buffer");
        return false;
    }
    
    // Initialize uzlib decompressor
    struct uzlib_uncomp decomp;
    memset(&decomp, 0, sizeof(decomp));
    
    // Set up input data
    decomp.source = currentImage.data;
    decomp.source_limit = currentImage.data + currentImage.size;
    decomp.source_read_cb = NULL; // We have all data in memory
    
    // Set up output buffer
    decomp.dest = buffer;
    decomp.dest_limit = buffer + bufferSize;
    
    // Python zlib.compressobj(wbits=12) creates raw DEFLATE, not gzip
    // Skip the first 4 bytes (length header from Python) and decompress raw DEFLATE
    if (currentImage.size < 4) {
        writeSerial("ERROR: Data too small for compressed format");
        free(buffer);
        return false;
    }
    
    // Skip the 4-byte length header and use raw DEFLATE
    uint32_t originalLength = *(uint32_t*)(currentImage.data);
    writeSerial("Original length from header: " + String(originalLength) + " bytes");
    writeSerial("Compressed data size: " + String(currentImage.size - 4) + " bytes");
    
    decomp.source = currentImage.data + 4;
    decomp.source_limit = currentImage.data + currentImage.size;
    writeSerial("Skipping 4-byte length header, decompressing " + String(currentImage.size - 4) + " bytes of raw DEFLATE");
    
    int ret = uzlib_uncompress(&decomp);
    
    if (ret != 0) {
        String errorMsg = "ERROR: uzlib decompression failed: " + String(ret);
        switch (ret) {
            case -1: errorMsg += " (UZLIB_DATA_ERROR - invalid data)"; break;
            case -2: errorMsg += " (UZLIB_CHUNK_ERROR - chunk error)"; break;
            case -3: errorMsg += " (UZLIB_MEM_ERROR - memory error)"; break;
            case -4: errorMsg += " (UZLIB_BUF_ERROR - buffer error)"; break;
            case -5: errorMsg += " (UZLIB_VERSION_ERROR - version error)"; break;
            default: errorMsg += " (unknown error)"; break;
        }
        writeSerial(errorMsg);
        free(buffer);
        return false;
    }
    
    uint32_t decompressedSize = decomp.dest - buffer;
    
    writeSerial("Decompression successful: " + String(currentImage.size) + " -> " + String(decompressedSize) + " bytes");
    
    // Allocate final output buffer with exact size
    *outputSize = decompressedSize;
    *output = (uint8_t*)malloc(*outputSize);
    if (!*output) {
        writeSerial("ERROR: Failed to allocate memory for final output");
        free(buffer);
        return false;
    }
    
    // Copy decompressed data to output
    memcpy(*output, buffer, *outputSize);
    free(buffer);
    
    writeSerial("uzlib decompression completed successfully");
    return true;
}

void drawImageData() {
    writeSerial("Drawing full image data...");
    writeSerial("Displaying all " + String(currentImage.size) + " bytes of image data");

    uint16_t displayWidth = epd.width();
    uint16_t displayHeight = epd.height();
    uint32_t expectedBytes = (displayWidth * displayHeight) / 8; 
    
    writeSerial("Expected bytes for 1-bit: " + String(expectedBytes));
    writeSerial("Actual received bytes: " + String(currentImage.size));
    writeSerial("Image type: 0x" + String(currentImage.dataType, HEX));
    
    //uncompressed image 1bpp
    if (currentImage.dataType == 0x20) {
        writeSerial("Processing 1-bit black/white image");
        // Process uncompressed image data
        uint32_t pixelIndex = 0;
        for (uint16_t y = 0; y < displayWidth; y++) {
            for (uint16_t x = 0; x < displayHeight; x++) {
                if (pixelIndex >= currentImage.size) break;
                uint8_t pixelByte = currentImage.data[pixelIndex];
                for (int bit = 7; bit >= 0; bit--) {
                    if (x >= displayHeight) break;
                    bool pixelValue = (pixelByte >> bit) & 0x01;
                    if (pixelValue) {
                       epd.drawPixel(displayWidth - y, x, BBEP_BLACK);
                    } else {
                        epd.drawPixel(displayWidth - y, x, BBEP_WHITE);
                    }
                    
                    x++;
                }
                x--; 
                pixelIndex++;
            }
        }
    }
    
    else if (currentImage.dataType == 0x30) {
        writeSerial("Processing compressed image data");
        
        // Dump compressed image buffer in hex
        writeSerial("=== COMPRESSED IMAGE BUFFER HEX DUMP ===");
        writeSerial("Total size: " + String(currentImage.size) + " bytes");
        writeSerial("Received: " + String(currentImage.received) + " bytes");
        
        // Dump first 64 bytes in detail
        writeSerial("First 64 bytes:");
        String hexLine = "";
        for (uint32_t i = 0; i < min((uint32_t)64, currentImage.received); i++) {
            if (currentImage.data[i] < 0x10) hexLine += "0";
            hexLine += String(currentImage.data[i], HEX) + " ";
            if ((i + 1) % 16 == 0) {
                writeSerial(hexLine);
                hexLine = "";
            }
        }
        if (hexLine.length() > 0) {
            writeSerial(hexLine);
        }
        
        // Dump length header (first 4 bytes)
        if (currentImage.received >= 4) {
            uint32_t originalLength = *(uint32_t*)(currentImage.data);
            writeSerial("Length header: " + String(originalLength) + " bytes (0x" + String(originalLength, HEX) + ")");
        }
        
        // Dump compressed data (after 4-byte header) in chunks
        if (currentImage.received > 4) {
            writeSerial("Compressed DEFLATE data (after header):");
            uint32_t compressedDataSize = currentImage.received - 4;
            uint32_t bytesToDump = min(compressedDataSize, (uint32_t)256); // Limit to first 256 bytes
            
            for (uint32_t i = 0; i < bytesToDump; i++) {
                if (i % 16 == 0) {
                    if (i > 0) writeSerial(hexLine);
                    hexLine = String(i + 4, HEX) + ": ";
                }
                if (currentImage.data[i + 4] < 0x10) hexLine += "0";
                hexLine += String(currentImage.data[i + 4], HEX) + " ";
            }
            if (hexLine.length() > 0) {
                writeSerial(hexLine);
            }
            if (compressedDataSize > 256) {
                writeSerial("... (truncated, showing first 256 bytes of " + String(compressedDataSize) + " total)");
            }
        }
        writeSerial("=== END HEX DUMP ===");
        
        // Decompress the data first
        uint8_t* decompressedData = nullptr;
        uint32_t decompressedSize = 0;
        
        if (decompressImageData(&decompressedData, &decompressedSize)) {
            writeSerial("Decompression successful, size: " + String(decompressedSize) + " bytes");
            
            // Parse the decompressed header
            if (decompressedSize >= 6) {
                uint16_t imgWidth = decompressedData[1] | (decompressedData[2] << 8);
                uint16_t imgHeight = decompressedData[3] | (decompressedData[4] << 8);
                uint8_t colorType = decompressedData[5];
                
                writeSerial("Image dimensions: " + String(imgWidth) + "x" + String(imgHeight));
                writeSerial("Color type: " + String(colorType));
                
                // Process the decompressed image data (skip 6-byte header)
                uint8_t* imageData = decompressedData + 6;
                uint32_t imageDataSize = decompressedSize - 6;
                
                uint32_t pixelIndex = 0;
                for (uint16_t y = 0; y < displayWidth; y++) {
                    for (uint16_t x = 0; x < displayHeight; x++) {
                        if (pixelIndex >= imageDataSize) break;
                        uint8_t pixelByte = imageData[pixelIndex];
                        for (int bit = 7; bit >= 0; bit--) {
                            if (x >= displayHeight) break;
                            bool pixelValue = (pixelByte >> bit) & 0x01;
                            if (pixelValue) {
                                epd.drawPixel(displayWidth - y, x, BBEP_BLACK);
                            } else {
                                epd.drawPixel(displayWidth - y, x, BBEP_WHITE);
                            }
                            
                            x++;
                        }
                        x--; // Adjust for the loop increment
                        pixelIndex++;
                    }
                }
            } else {
                writeSerial("ERROR: Decompressed data too small for header");
            }
            
            // Free decompressed data
            free(decompressedData);
        } else {
            writeSerial("ERROR: Failed to decompress image data");
        }
        writeSerial("=== DEBUG: FULL BUFFER DUMP ===");
        writeSerial("Total image size: " + String(currentImage.size) + " bytes");
        writeSerial("Total received: " + String(currentImage.received) + " bytes");
        writeSerial("Total blocks: " + String(currentImage.totalBlocks));
        writeSerial("NOTE: Each packet contains 232 bytes of actual data (4 bytes skipped at start of each block, padding removed from last packet)");
        for (uint32_t i = 0; i < currentImage.totalBlocks; i++) {
            writeSerial("Block " + String(i) + ": " + 
                       String(currentImage.blockBytesReceived[i]) + " bytes, " + 
                       String(currentImage.blockPacketsReceived[i]) + " packets, " +
                       (currentImage.blocksReceived[i] ? "COMPLETE" : "INCOMPLETE"));
        }
        writeSerial("First 64 bytes of buffer (hex):");
        String hexDump = "";
        for (uint32_t i = 0; i < min((uint32_t)4736, currentImage.received); i++) {
            if (currentImage.data[i] < 0x10) hexDump += "0";
            hexDump += String(currentImage.data[i], HEX) + " ";
            if ((i + 1) % 64 == 0) {
                writeSerial(hexDump);
                hexDump = "";
            }
        }
        if (hexDump.length() > 0) {
            writeSerial(hexDump);
        }
        for (uint32_t block = 0; block < currentImage.totalBlocks; block++) {
            uint32_t blockStart = block * 4096;
            uint32_t blockEnd = min(blockStart + 4096, currentImage.received);
            if (blockStart < currentImage.received) {
                writeSerial("Block " + String(block) + " boundary data (bytes " + String(blockStart) + "-" + String(blockEnd-1) + "):");
                String boundaryHex = "";
                for (uint32_t i = blockStart; i < min(blockStart + 32, blockEnd); i++) {
                    if (currentImage.data[i] < 0x10) boundaryHex += "0";
                    boundaryHex += String(currentImage.data[i], HEX) + " ";
                }
                writeSerial(boundaryHex);
            }
        }
        writeSerial("=== END DEBUG DUMP ===");
        uint32_t pixelIndex = 0;
        for (uint16_t y = 0; y < displayWidth; y++) {
            for (uint16_t x = 0; x < displayHeight; x++) {
                if (pixelIndex >= currentImage.size) break;
                uint8_t pixelByte = currentImage.data[pixelIndex];
                for (int bit = 7; bit >= 0; bit--) {
                    if (x >= displayHeight) break;
                    bool pixelValue = (pixelByte >> bit) & 0x01;
                    if (pixelValue) {
                       epd.drawPixel(displayWidth - y, x, BBEP_BLACK);
                    } else {
                        epd.drawPixel(displayWidth - y, x, BBEP_WHITE);
                    }
                    
                    x++;
                }
                x--;
                pixelIndex++;
            }
        }
    } else {
        writeSerial("ERROR: Unsupported image type: 0x" + String(currentImage.dataType, HEX));
        return;
    }
    writeSerial("Full image data drawn successfully");
    writeSerial("Processed " + String(currentImage.size) + " bytes of image data");
}

void handleReadDynamicConfig() {
    writeSerial("Handling Read Dynamic Config request...");
    uint8_t responseBuffer[94];
    uint16_t responseLen = 0;
    responseBuffer[responseLen++] = 0x00;
    responseBuffer[responseLen++] = 0xCD;
    uint16_t configLen = 0;
    buildDynamicConfigResponse(responseBuffer + 2, &configLen);
    responseLen += configLen;
    if (responseLen > 2) {
        writeSerial("Sending Dynamic Config response (" + String(responseLen) + " bytes)...");
        String hexDump = "Complete response (" + String(responseLen) + " bytes): ";
        for (uint16_t i = 0; i < responseLen; i++) {
            if (responseBuffer[i] < 0x10) hexDump += "0";
            hexDump += String(responseBuffer[i], HEX);
        }
        writeSerial(hexDump);
        sendResponse(responseBuffer, responseLen);
        writeSerial("Dynamic Config response sent successfully");
    } else {
        writeSerial("ERROR: Failed to build Dynamic Config response");
    }
}

void handleDisplayInfo() {
    writeSerial("Building Display Info response...");
    uint8_t response[33];
    uint16_t offset = 0;
    response[offset++] = 0x00;
    response[offset++] = 0x05;
    for (int i = 0; i < 31; i++) {
        response[offset++] = 0x00;
    }
    offset = 2;
    for (int i = 0; i < 18; i++) {
        response[offset++] = 0x00;
    }
    response[offset++] = 0x00; 
    response[offset++] = 0x00;
    response[offset++] = 0x00;
    response[offset++] = 0x00;
    // Offset 22-23: Width (uint16, little-endian)
    uint16_t width = epd.width();
    response[offset++] = width & 0xFF;
    response[offset++] = (width >> 8) & 0xFF;
    // Offset 24-25: Height (uint16, little-endian)  
    uint16_t height = epd.height();
    response[offset++] = height & 0xFF;
    response[offset++] = (height >> 8) & 0xFF;
    // Offset 26-29: Reserved
    for (int i = 0; i < 4; i++) {
        response[offset++] = 0x00;
    }
    // Offset 30: Color count (1=BW, 2=BWR/BWY, 3=BWRY)
    // Our display is monochrome (black/white only)
    response[offset++] = 0x01; // Monochrome
    
    writeSerial("Display Info - Width: " + String(width) + ", Height: " + String(height) + ", Colors: 1 (mono)");
    // Send the response
    sendResponse(response, sizeof(response));
    writeSerial("Display info response sent");
}

void buildDynamicConfigResponse(uint8_t* buffer, uint16_t* len) {
    writeSerial("Building Dynamic Config response...");
    uint16_t offset = 0;
    // Screen Type (2 bytes) - 1-color display (black/white only)
    buffer[offset++] = 0x20; // 1-color display type
    buffer[offset++] = 0x00;
    // Default Settings (41 bytes)
    // HW Type (2 bytes)
    buffer[offset++] = 0x36;
    buffer[offset++] = 0x00;
    // Screen Functions (2 bytes)
    buffer[offset++] = 0x01; // Basic display functions
    buffer[offset++] = 0x00;
    // W/H Inversed BLE (1 byte)
    buffer[offset++] = 0x00; // Not inversed
    // W/H Inversed (2 bytes)
    buffer[offset++] = 0x00; // Not inversed
    buffer[offset++] = 0x00;
    uint16_t screenHeight = epd.width();
    buffer[offset++] = screenHeight & 0xFF;
    buffer[offset++] = (screenHeight >> 8) & 0xFF;
    uint16_t screenWidth = epd.height();
    buffer[offset++] = screenWidth & 0xFF; 
    buffer[offset++] = (screenWidth >> 8) & 0xFF;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x01;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // EPD Pinout Enabled (4 bytes)
    buffer[offset++] = 0x01; // Enabled
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // LED Pinout Enabled (4 bytes)
    buffer[offset++] = 0x00; // Disabled
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // NFC Pinout Enabled (4 bytes)
    buffer[offset++] = 0x00; // Disabled
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // Flash Pinout Enabled (4 bytes)
    buffer[offset++] = 0x00; // Disabled
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // ADC Pinout (2 bytes)
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // UART Pinout (2 bytes)
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // EPD Pinout (26 bytes) - Your current pin configuration
    // Reset pin (D0)
    buffer[offset++] = 0x00; // D0 = 0
    buffer[offset++] = 0x00;
    // DC pin (D3)
    buffer[offset++] = 0x03; // D3 = 3
    buffer[offset++] = 0x00;
    // BUSY pin (D5)
    buffer[offset++] = 0x05; // D5 = 5
    buffer[offset++] = 0x00;
    // BUSY secondary pin (2 bytes) - not used
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // CS pin (D1)
    buffer[offset++] = 0x01; // D1 = 1
    buffer[offset++] = 0x00;
    // CS secondary pin (2 bytes) - not used
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // CLK pin (D8)
    buffer[offset++] = 0x08; // D8 = 8
    buffer[offset++] = 0x00;
    // MOSI pin (D10)
    buffer[offset++] = 0x0A; // D10 = 10
    buffer[offset++] = 0x00;
    // Enable pin (2 bytes) - not used
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // Enable1 pin (2 bytes) - not used
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // Enable Invert (1 byte)
    buffer[offset++] = 0x00; // Not inverted
    // Flash CS pin (2 bytes) - not used
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    // Pin Config Sleep (1 byte)
    buffer[offset++] = 0x00;
    // Pin Enable (1 byte)
    buffer[offset++] = 0x00;
    // Pin Enable Sleep (1 byte)
    buffer[offset++] = 0x00;
    // LED Pinout (7 bytes) - all disabled
    buffer[offset++] = 0x00; // R pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // G pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // B pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // Inverted
    // NFC Pinout (8 bytes) - all disabled
    buffer[offset++] = 0x00; // SDA pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // SCL pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // CS pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // IRQ pin
    buffer[offset++] = 0x00;
    // Flash Pinout (8 bytes) - all disabled
    buffer[offset++] = 0x00; // CS pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // CLK pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // MISO pin
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00; // MOSI pin
    buffer[offset++] = 0x00;
    *len = offset;
    writeSerial("Dynamic Config response built successfully (" + String(offset) + " bytes)");
    // Debug: Print first few bytes of config data
    String configDump = "Config data: ";
    for (uint16_t i = 0; i < min(offset, (uint16_t)16); i++) {
        if (buffer[i] < 0x10) configDump += "0";
        configDump += String(buffer[i], HEX);
    }
    if (offset > 16) configDump += "...";
    writeSerial(configDump);
    // Debug: Print complete config data
    String fullConfigDump = "Full config data (" + String(offset) + " bytes): ";
    for (uint16_t i = 0; i < offset; i++) {
        if (buffer[i] < 0x10) fullConfigDump += "0";
        fullConfigDump += String(buffer[i], HEX);
    }
    writeSerial(fullConfigDump);
}