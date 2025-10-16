#include "main.h"

void setup() {
    initio();
    Serial.begin(115200);
    bool usb_connected = bitRead(NRF_POWER->USBREGSTATUS, 0);
    writeSerial("=== BLE OEPL Device Starting ===");
    delay(2000);
    if (usb_connected) {writeSerial("USB connected");}
    writeSerial("Initializing BLE...");
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.setTxPower(8);
    Bluefruit.begin(1, 0);
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
    writeSerial("Initializing config storage...");
    if (initConfigStorage()) {
        writeSerial("Config storage initialized successfully");
    } else {
        writeSerial("Config storage initialization failed");
    }
    writeSerial("Loading global configuration...");
    if (loadGlobalConfig()) {
        writeSerial("Global configuration loaded successfully");
        printConfigSummary();
    } else {
       writeSerial("Global configuration load failed or no config found");
    }

    writeSerial("Initializing display");
    initDisplay();
    writeSerial("Display initialized");

    writeSerial("=== Setup completed successfully ===");
    writeSerial("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 200);
    Bluefruit.Advertising.setFastTimeout(10);
    writeSerial("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
    writeSerial("BLE advertising started - waiting for connections...");
}

void initio(){
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
    analogReference(AR_INTERNAL_2_4);
    analogReadResolution(ADC_RESOLUTION);
    pinMode(PIN_VBAT, INPUT);
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, LOW);
}

void updatemsdata(){
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
writeSerial("MSD data updated");
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
        cleanupImageMemory();
        writeSerial("Image processing complete");
    }
    delay(2000);
    writeSerial("Loop end: " + String(millis() / 100));
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

    writeSerial(String("DC_PIN: ") + String(DC_PIN));
    writeSerial(String("RESET_PIN: ") + String(RESET_PIN));
    writeSerial(String("BUSY_PIN: ") + String(BUSY_PIN));
    writeSerial(String("CS_PIN: ") + String(CS_PIN));
    writeSerial(String("MOSI_PIN: ") + String(MOSI_PIN));
    writeSerial(String("CLK_PIN: ") + String(CLK_PIN));

    writeSerial(String("Height: ") + String(epd.height()));
    writeSerial(String("Width: ") + String(epd.width()));

    epd.allocBuffer();
    //epd.allocBuffer(true);
    epd.fillScreen(BBEP_WHITE);
    //epd.fillScreen(BBEP_3COLOR);
    epd.setTextColor(BBEP_BLACK, BBEP_WHITE);
    epd.setFont(FONT_12x16);
    String chipId = getChipIdHex();
    //epd.drawSprite(epd_bitmap_logo, 152, 152, 19, 600, 22, BBEP_BLACK);
    writeSerial("Chip ID for display: " + chipId);
    //epd.setCursor(100, 100); 
    //epd.print("openepaperlink.org");
    //epd.setCursor(100, 200); 
    //epd.print("Name: OEPL"+ chipId);
    //epd.setCursor(100, 300); 
    //epd.print("Epaper driver by Larry Bank https://github.com/bitbank2");

    epd.setCursor(10, 10); 
    epd.print("Test");
    epd.writePlane();
    epd.refresh(REFRESH_FULL, false);
    delay(2000);
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
    if (currentImage.data || currentImage.blocksReceived || 
        currentImage.blockBytesReceived || currentImage.blockPacketsReceived) {
        writeSerial("Cleaning up image data due to disconnect...");
        cleanupImageMemory();
    }
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
    if (len < 2) {
        writeSerial("ERROR: Command too short (" + String(len) + " bytes)");
        return;
    }
    
    uint16_t command = (data[0] << 8) | data[1];  // Fixed byte order
    writeSerial("Processing command: 0x" + String(command, HEX));
    switch (command) {
        case 0x0040: // Read Config command
            writeSerial("=== READ CONFIG COMMAND (0x0040) ===");
            writeSerial("Command received at time: " + String(millis()));
            handleReadConfig();
            writeSerial("Returned from handleReadConfig");
            break;
        case 0x0041: // Write Config command
            writeSerial("=== WRITE CONFIG COMMAND (0x0041) ===");
            handleWriteConfig(data + 2, len - 2);
            break;
        case 0x0042: // Write Config Chunk command
            writeSerial("=== WRITE CONFIG CHUNK COMMAND (0x0042) ===");
            handleWriteConfigChunk(data + 2, len - 2);
            break;
        case 0x0011: // Read Dynamic Config command (legacy)
            writeSerial("=== READ DYNAMIC CONFIG COMMAND (0x0011) ===");
            handleReadDynamicConfig();
            break;
        case 0x0064: // Image info command
            writeSerial("=== IMAGE INFO COMMAND (0x0064) ===");
            handleImageInfo(data + 2, len - 2);
            break;
        case 0x0065: // Block data command
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
    
    writeSerial("Command processing completed successfully");
}

void handleImageInfo(uint8_t* data, uint16_t len) {
    writeSerial("=== HANDLING IMAGE INFO ===");
    writeSerial("Current image state before cleanup:");
    writeSerial("  data: " + String((uintptr_t)currentImage.data, HEX));
    writeSerial("  size: " + String(currentImage.size));
    writeSerial("  received: " + String(currentImage.received));
    writeSerial("  ready: " + String(currentImage.ready ? "true" : "false"));
    writeSerial("  dataType: 0x" + String(currentImage.dataType, HEX));
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
    writeSerial("Setting up image data buffer...");
    // Clean up any existing image data first
    cleanupImageMemory();
    // Check if image fits in global buffer
    if (dataSize > MAX_IMAGE_SIZE) {
        writeSerial("ERROR: Image too large (" + String(dataSize) + " bytes, max: " + String(MAX_IMAGE_SIZE) + " bytes)");
        return;
    }
    // Use global buffer
    currentImage.data = imageDataBuffer;
    writeSerial("Using global image buffer: " + String(dataSize) + " bytes");
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
    currentImage.currentBlock = 0xFFFFFFFF; // Initialize to invalid value to detect first block
    writeSerial("Calculated blocks needed: " + String(currentImage.totalBlocks) + " (data size: " + String(dataSize) + " bytes, block size: " + String(BLOCK_DATA_SIZE) + " bytes)");
    // Check if we have enough blocks in global array
    if (currentImage.totalBlocks > MAX_BLOCKS) {
        writeSerial("ERROR: Too many blocks (" + String(currentImage.totalBlocks) + ", max: " + String(MAX_BLOCKS) + ")");
        cleanupImageMemory();
        return;
    }
    // Use global block tracking arrays
    currentImage.blocksReceived = blocksReceived;
    currentImage.blockBytesReceived = blockBytesReceived;
    currentImage.blockPacketsReceived = blockPacketsReceived;
    // Initialize block tracking
    for (uint32_t i = 0; i < currentImage.totalBlocks; i++) {
        currentImage.blocksReceived[i] = false;
        currentImage.blockBytesReceived[i] = 0;
        currentImage.blockPacketsReceived[i] = 0;
    }
    writeSerial("Block tracking initialized for " + String(currentImage.totalBlocks) + " blocks");
    writeSerial("Sending block request for block 0...");
    uint8_t* response = bleResponseBuffer;
    response[0] = 0x00; response[1] = 0xC6; // Command: 0x00C6
    response[2] = 0x00; // Checksum
    response[3] = 0x00; response[4] = 0x00; response[5] = 0x00; response[6] = 0x00;
    response[7] = 0x00; response[8] = 0x00; response[9] = 0x00; response[10] = 0x00;
    response[11] = 0; // Block ID 0
    response[12] = 0x00; // Type
    response[13] = 0xFF; response[14] = 0xFF; response[15] = 0xFF;
    response[16] = 0xFF; response[17] = 0xFF; response[18] = 0xFF;
    sendResponse(response, 19);
}

void handleBlockData(uint8_t* data, uint16_t len) {
    if (len < 4) {
        writeSerial("ERROR: Block data too short (" + String(len) + " bytes, need 4)");
        return;
    }
    uint8_t blockId = data[1];
    uint8_t* blockData = data + 3;
    uint16_t dataLen = len - 3;
    uint32_t maxSize = currentImage.size + 1024;
    if (currentImage.data && currentImage.received + dataLen <= maxSize) {
        if (blockId != currentImage.currentBlock) {
            writeSerial("New block " + String(blockId) + " detected, skipping first 4 bytes");
            currentImage.currentBlock = blockId;
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
            uint32_t nextBlock = blockId + 1;
            if (nextBlock < currentImage.totalBlocks) {
                writeSerial("Requesting next block: " + String(nextBlock) + "/" + String(currentImage.totalBlocks));
                uint8_t* response = bleResponseBuffer;
                response[0] = 0x00; response[1] = 0xC6;
                response[2] = 0x00; // Checksum
                response[3] = 0x00; response[4] = 0x00; response[5] = 0x00; response[6] = 0x00;
                response[7] = 0x00; response[8] = 0x00; response[9] = 0x00; response[10] = 0x00;
                response[11] = nextBlock; // Block ID
                response[12] = 0x00; // Type
                response[13] = 0xFF; response[14] = 0xFF; response[15] = 0xFF;
                response[16] = 0xFF; response[17] = 0xFF; response[18] = 0xFF;
                sendResponse(response, 19);
            } else {
                writeSerial("ERROR: Requested block " + String(nextBlock) + " exceeds total blocks " + String(currentImage.totalBlocks));
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
    
    // Dump full command bytes
    String hexDump = "  Full command: ";
    for (int i = 0; i < len; i++) {
        if (i > 0) hexDump += " ";
        if (response[i] < 16) hexDump += "0";
        hexDump += String(response[i], HEX);
    }
    writeSerial(hexDump);
    
    imageCharacteristic.notify(response, len);
    delay(20);
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
        writeSerial("  currentImage.data: " + String((uintptr_t)currentImage.data, HEX));
        writeSerial("  currentImage.size: " + String(currentImage.size));
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
    drawImageData();
    epd.writePlane();
    epd.refresh(REFRESH_FULL, true);
    epd.sleep(DEEP_SLEEP);
    pwrmgm(false);
    writeSerial("=== IMAGE DISPLAY COMPLETE ===");
}

bool decompressImageData(uint8_t** output, uint32_t* outputSize) {
    writeSerial("Starting uzlib decompression...");
    // Read 4-byte header (expected uncompressed size)
    uint32_t originalLength = 0;
    memcpy(&originalLength, currentImage.data, sizeof(originalLength));
    if (originalLength == 0 || originalLength > 512000) {
        writeSerial("ERROR: Invalid decompressed size: " + String(originalLength));
        return false;
    }
    writeSerial("Header indicates decompressed size: " + String(originalLength) + " bytes");
    // Allocate final output buffer (for compatibility)
    // We'll still fill it in chunks.
    *output = (uint8_t*)malloc(originalLength);
    if (!*output) {
        writeSerial("ERROR: Failed to allocate memory for output buffer.");
        return false;
    }
    *outputSize = originalLength;
    const uint8_t* compressed = currentImage.data + 4;
    uint32_t compressedSize = currentImage.size - 4;
    writeSerial("Compressed data size: " + String(compressedSize) + " bytes");
    writeSerial("Total received data: " + String(currentImage.size) + " bytes");
    struct uzlib_uncomp d;
    memset(&d, 0, sizeof(d));
    d.source = compressed;
    d.source_limit = compressed + compressedSize;
    d.source_read_cb = NULL;
    uzlib_init();
    int hdr = uzlib_zlib_parse_header(&d);
    if (hdr < 0) {
        writeSerial("ERROR: Invalid zlib header: " + String(hdr));
        free(*output);
        *output = nullptr;
        return false;
    }
    uint16_t window = 0x100 << hdr;
    if (window > 32768) window = 32768;
    uint8_t* dict = (uint8_t*)malloc(window);
    if (!dict) {
        writeSerial("ERROR: Failed to allocate dictionary buffer.");
        free(*output);
        *output = nullptr;
        return false;
    }
    uzlib_uncompress_init(&d, dict, window);
    // streaming buffer (small chunk)
    uint8_t* chunk = (uint8_t*)malloc(DECOMP_CHUNK);
    if (!chunk) {
        writeSerial("ERROR: Failed to allocate chunk buffer.");
        free(dict);
        free(*output);
        *output = nullptr;
        return false;
    }
    uint32_t totalOut = 0;
    int res;
    do {
        d.dest_start = chunk;
        d.dest = chunk;
        d.dest_limit = chunk + DECOMP_CHUNK;
        res = uzlib_uncompress(&d);
        size_t bytesOut = d.dest - d.dest_start;
        // Copy chunk into final buffer (compatibility mode)
        if (bytesOut > 0) {
            // Ensure we don't exceed buffer bounds
            size_t bytesToCopy = min(bytesOut, (size_t)(originalLength - totalOut));
            if (bytesToCopy > 0) {
                memcpy(*output + totalOut, chunk, bytesToCopy);
                totalOut += bytesToCopy;
            }
        }
        if (res == TINF_DATA_ERROR) {
            writeSerial("ERROR: uzlib data error");
            break;
        }
    } while (res == TINF_OK && totalOut < originalLength);
    free(chunk);
    free(dict);
    if (res != TINF_DONE && res != TINF_OK) {
        writeSerial("ERROR: uzlib_uncompress() failed with code " + String(res));
        free(*output);
        *output = nullptr;
        return false;
    }
    writeSerial("Decompressed successfully: " + String(totalOut) + " bytes");
    writeSerial("Expected: " + String(originalLength) + " bytes");
    if (totalOut < originalLength) {
        writeSerial("WARNING: Decompressed size is " + String(originalLength - totalOut) + " bytes less than expected");
    }
    return true;
}

void drawImageData() {
    writeSerial("Drawing image data...");
    writeSerial("Image type: 0x" + String(currentImage.dataType, HEX));
    uint16_t displayWidth = epd.width();
    uint16_t displayHeight = epd.height();
    if (currentImage.dataType == 0x20) {
        writeSerial("Uncompressed image, drawing directly...");
        uint32_t pixelIndex = 0;
        for (uint16_t y = 0; y < displayWidth; y++) {
            for (uint16_t x = 0; x < displayHeight; x++) {
                if (pixelIndex >= currentImage.size) break;
                uint8_t pixelByte = currentImage.data[pixelIndex];
                for (int bit = 7; bit >= 0; bit--) {
                    if (x >= displayHeight) break;
                    bool pixelValue = (pixelByte >> bit) & 0x01;
                    epd.drawPixel(displayWidth - y, x, pixelValue ? BBEP_BLACK : BBEP_WHITE);
                    x++;
                }
                x--;
                pixelIndex++;
            }
        }
    }
    else if (currentImage.dataType == 0x30) {
        writeSerial("Compressed image detected, decompressing with chunked approach...");
        if (!decompressImageDataChunked()) {
            writeSerial("ERROR: Chunked decompression failed.");
            return;
        }
        writeSerial("Chunked decompression and drawing complete.");
    }
    else {
        writeSerial("ERROR: Unsupported image type.");
    }
    writeSerial("Image drawing complete.");
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

void cleanupImageMemory() {
    writeSerial("=== CLEANING UP IMAGE MEMORY ===");
    memset(imageDataBuffer, 0, MAX_IMAGE_SIZE);
    memset(blocksReceived, 0, MAX_BLOCKS * sizeof(bool));
    memset(blockBytesReceived, 0, MAX_BLOCKS * sizeof(uint32_t));
    memset(blockPacketsReceived, 0, MAX_BLOCKS * sizeof(uint32_t));
    memset(headerBuffer, 0, 6);
    memset(bleResponseBuffer, 0, 94);
    currentImage.data = nullptr;
    currentImage.blocksReceived = nullptr;
    currentImage.blockBytesReceived = nullptr;
    currentImage.blockPacketsReceived = nullptr;
    currentImage.size = 0;
    currentImage.received = 0;
    currentImage.dataType = 0;
    currentImage.isCompressed = false;
    currentImage.crc32 = 0;
    currentImage.width = 0;
    currentImage.height = 0;
    currentImage.totalBlocks = 0;
    currentImage.currentBlock = 0xFFFFFFFF;
    currentImage.ready = false;
    currentBlockId = 0;
    currentPacketId = 0;
    expectedPackets = 0;
    receivedPackets = 0;
    writeSerial("=== MEMORY CLEANUP COMPLETE ===");
}

bool decompressImageDataChunked() {
    writeSerial("Starting chunked decompression with direct drawing...");
    uint32_t originalLength = 0;
    memcpy(&originalLength, currentImage.data, sizeof(originalLength));
    if (originalLength == 0 || originalLength > MAX_DECOMPRESSED_SIZE) {
        writeSerial("ERROR: Invalid decompressed size: " + String(originalLength));
        return false;
    }
    writeSerial("Header indicates decompressed size: " + String(originalLength) + " bytes");
    const uint8_t* compressed = currentImage.data + 4;
    uint32_t compressedSize = currentImage.size - 4;
    writeSerial("Compressed data size: " + String(compressedSize) + " bytes");
    struct uzlib_uncomp d;
    memset(&d, 0, sizeof(d));
    d.source = compressed;
    d.source_limit = compressed + compressedSize;
    d.source_read_cb = NULL;
    uzlib_init();
    int hdr = uzlib_zlib_parse_header(&d);
    if (hdr < 0) {
        writeSerial("ERROR: Invalid zlib header: " + String(hdr));
        return false;
    }
    uint16_t window = 0x100 << hdr;
    if (window > MAX_DICT_SIZE) window = MAX_DICT_SIZE;
    uzlib_uncompress_init(&d, dictionaryBuffer, window);
    uint16_t displayWidth = epd.width();
    uint16_t displayHeight = epd.height();
    uint32_t totalDecompressed = 0;
    uint16_t imgWidth = 0, imgHeight = 0;
    uint8_t colorType = 0;
    bool headerParsed = false;
    uint32_t headerBytesReceived = 0;
    uint32_t pixelBytesProcessed = 0;
    writeSerial("Starting chunked decompression and direct drawing...");
    int res;
    do {
        d.dest_start = decompressionChunk;
        d.dest = decompressionChunk;
        d.dest_limit = decompressionChunk + DECOMP_CHUNK_SIZE;
        res = uzlib_uncompress(&d);
        size_t bytesOut = d.dest - d.dest_start;
        if (bytesOut > 0) {
            totalDecompressed += bytesOut;
            for (size_t i = 0; i < bytesOut; i++) {
                uint8_t byte = decompressionChunk[i];
                if (!headerParsed) {
                    if (headerBytesReceived < 6) {
                        headerBuffer[headerBytesReceived] = byte;
                        headerBytesReceived++;
                    }
                    if (headerBytesReceived >= 6) {
                        imgWidth = headerBuffer[1] | (headerBuffer[2] << 8);
                        imgHeight = headerBuffer[3] | (headerBuffer[4] << 8);
                        colorType = headerBuffer[5];
                        headerParsed = true;
                        writeSerial("Image dimensions: " + String(imgWidth) + "x" + String(imgHeight));
                        writeSerial("Color type: " + String(colorType));
                        uint16_t drawWidth = min(imgWidth, displayHeight);
                        uint16_t drawHeight = min(imgHeight, displayWidth);
                        writeSerial("Drawing bounds: " + String(drawWidth) + "x" + String(drawHeight));
                    }
                } else {
                    uint32_t pixelByteIndex = pixelBytesProcessed;
                    uint16_t y = pixelByteIndex / (imgWidth / 8);
                    uint16_t x = (pixelByteIndex % (imgWidth / 8)) * 8;
                    for (int bit = 7; bit >= 0; bit--) {
                        if (y < min(imgHeight, displayWidth) && x + (7-bit) < min(imgWidth, displayHeight)) {
                            bool pixelValue = (byte >> bit) & 0x01;
                            epd.drawPixel(displayWidth - y, x + (7-bit), pixelValue ? BBEP_BLACK : BBEP_WHITE);
                        }
                    }
                    pixelBytesProcessed++;
                }
            }
        }
        if (res == TINF_DATA_ERROR) {
            writeSerial("ERROR: uzlib data error");
            break;
        }
        
    } while (res == TINF_OK && totalDecompressed < originalLength);
    writeSerial("Chunked decompression complete: " + String(totalDecompressed) + " bytes");
    writeSerial("Expected: " + String(originalLength) + " bytes");
    writeSerial("Pixel bytes processed: " + String(pixelBytesProcessed));
    if (res != TINF_DONE && res != TINF_OK) {
        writeSerial("ERROR: uzlib_uncompress() failed with code " + String(res));
        return false;
    }
    return true;
}

bool initConfigStorage() {
    writeSerial("=== INIT CONFIG STORAGE DEBUG ===");
    
    // Mount the internal file system
    if (!InternalFS.begin()) {
        writeSerial("ERROR: Failed to mount internal file system");
        return false;
    }
    
    writeSerial("Internal file system mounted successfully");
    
    // Test file system operations
    writeSerial("Testing file system operations...");
    
    // Test creating a simple file
    File testFile = InternalFS.open("/test.txt", Adafruit_LittleFS_Namespace::FILE_O_WRITE);
    if (testFile) {
        writeSerial("Test file created successfully");
        size_t testWritten = testFile.write("Hello World", 11);
        writeSerial("Test bytes written: " + String(testWritten));
        testFile.close();
        
        // Test reading the file
        File readFile = InternalFS.open("/test.txt", Adafruit_LittleFS_Namespace::FILE_O_READ);
        if (readFile) {
            writeSerial("Test file read successfully");
            char buffer[20];
            size_t testRead = readFile.read((uint8_t*)buffer, 11);
            buffer[11] = '\0';
            writeSerial("Test bytes read: " + String(testRead) + " - Content: " + String(buffer));
            readFile.close();
    } else {
            writeSerial("ERROR: Failed to read test file");
        }
        
        // Clean up test file
        InternalFS.remove("/test.txt");
        writeSerial("Test file cleaned up");
            } else {
        writeSerial("ERROR: Failed to create test file");
        return false;
    }
    
    writeSerial("File system test completed successfully");
    return true;
}

bool saveConfig(uint8_t* configData, uint32_t len) {
    writeSerial("=== SAVE CONFIG DEBUG ===");
    writeSerial("Input data length: " + String(len) + " bytes");
    
    if (len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large (" + String(len) + " bytes)");
        return false;
    }
    
    // Create config storage structure
    static config_storage_t config;
    config.magic = 0xDEADBEEF;
    config.version = 1;
    config.data_len = len;
    config.crc = calculateConfigCRC(configData, len);
    memcpy(config.data, configData, len);
    
    writeSerial("Config structure created:");
    writeSerial("  Magic: 0x" + String(config.magic, HEX));
    writeSerial("  Version: " + String(config.version));
    writeSerial("  Data length: " + String(config.data_len));
    writeSerial("  CRC: 0x" + String(config.crc, HEX));
    
    // Calculate sizes
    size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    size_t totalSize = headerSize + len; // Header + actual data length
    size_t fullStructSize = sizeof(config_storage_t);
    
    writeSerial("Size calculations:");
    writeSerial("  Header size: " + String(headerSize) + " bytes");
    writeSerial("  Total size to write: " + String(totalSize) + " bytes");
    writeSerial("  Full struct size: " + String(fullStructSize) + " bytes");
    writeSerial("  MAX_CONFIG_SIZE: " + String(MAX_CONFIG_SIZE) + " bytes");
    
    // Try different file modes
    writeSerial("Opening file: " + String(CONFIG_FILE_PATH));
    
    // First try to remove existing file
    if (InternalFS.exists(CONFIG_FILE_PATH)) {
        writeSerial("Removing existing file...");
        InternalFS.remove(CONFIG_FILE_PATH);
    }
    
    // Try opening with write mode
    File file = InternalFS.open(CONFIG_FILE_PATH, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
    if (!file) {
        writeSerial("ERROR: Failed to open config file for writing with FILE_O_WRITE");
        // Try with create mode
        file = InternalFS.open(CONFIG_FILE_PATH, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
        if (!file) {
            writeSerial("ERROR: Failed to open config file for writing with CREATE|WRITE");
        return false;
        }
        writeSerial("File opened successfully with CREATE|WRITE mode");
    } else {
        writeSerial("File opened successfully with WRITE mode");
    }
    
    // Write config data (only the used portion of the structure)
    writeSerial("Writing " + String(totalSize) + " bytes to file...");
    size_t bytesWritten = file.write((uint8_t*)&config, totalSize);
    writeSerial("Bytes written: " + String(bytesWritten));
    
    // Check file size after write
    file.seek(0);
    size_t fileSize = file.size();
    writeSerial("File size after write: " + String(fileSize) + " bytes");
    
    file.close();
    writeSerial("File closed");
    
    if (bytesWritten != totalSize) {
        writeSerial("ERROR: Failed to write complete config data (expected " + String(totalSize) + ", wrote " + String(bytesWritten) + ")");
        return false;
    }
    
    writeSerial("Config saved successfully (" + String(len) + " bytes)");
    return true;
}

bool loadConfig(uint8_t* configData, uint32_t* len) {

    File file = InternalFS.open(CONFIG_FILE_PATH, Adafruit_LittleFS_Namespace::FILE_O_READ);
    if (!file) {
        writeSerial("No config file found");
        return false;
    }
    
    // Read config storage structure header first
    static config_storage_t config;
    static size_t bytesRead;
    static size_t headerSize = sizeof(config_storage_t) - MAX_CONFIG_SIZE; // Size without data array
    writeSerial("Reading config header...");
    writeSerial("  Header size: " + String(headerSize) + " bytes");
    
    bytesRead = file.read((uint8_t*)&config, headerSize);
    writeSerial("  Bytes read: " + String(bytesRead) + " bytes");
    
    if (bytesRead != headerSize) {
        writeSerial("ERROR: Failed to read config header (expected " + String(headerSize) + ", got " + String(bytesRead) + ")");
        file.close();
        return false;
    }
    
    // Validate magic number before reading data
    if (config.magic != 0xDEADBEEF) {
        writeSerial("ERROR: Invalid config magic number");
        file.close();
        return false;
    }
    
    // Check data length before reading
    if (config.data_len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Config data too large");
        file.close();
        return false;
    }
    
    // Read the actual data portion
    bytesRead = file.read(config.data, config.data_len);

    file.flush();
    file.close();
    
    if (bytesRead != config.data_len) {
        writeSerial("ERROR: Failed to read complete config data (expected " + String(config.data_len) + ", read " + String(bytesRead) + ")");
        return false;
    }
    
    // Validate CRC
    uint32_t calculatedCRC = calculateConfigCRC(config.data, config.data_len);
    if (config.crc != calculatedCRC) {
        writeSerial("ERROR: Config CRC mismatch");
        return false;
    }
    
    // Copy data to output buffer - byte by byte with debugging
    writeSerial("Starting individual byte copy...");
    writeSerial("  Source data length: " + String(config.data_len) + " bytes");
    writeSerial("  Destination buffer size: " + String(*len) + " bytes");
    
    for (uint32_t i = 0; i < config.data_len; i++) {
        if (i < *len) {  // Safety check to prevent buffer overflow
            configData[i] = config.data[i];
        } else {
            writeSerial("ERROR: Buffer overflow at byte " + String(i));
        return false;
        }
    }
    
    *len = config.data_len;
    
    writeSerial("Individual byte copy completed");
    writeSerial("  Final length: " + String(*len) + " bytes");
    writeSerial("Config loaded successfully (" + String(config.data_len) + " bytes) - from loadConfig function");
    return true;
}

uint32_t calculateConfigCRC(uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

void handleReadConfig() {
    
    uint8_t configData[MAX_CONFIG_SIZE];
    uint32_t configLen = MAX_CONFIG_SIZE;  // Initialize with max buffer size
    
    if (loadConfig(configData, &configLen)) {
        writeSerial("Sending config data in chunks...");
        
        // Send config data in chunks
        uint32_t remaining = configLen;
        uint32_t offset = 0;
        uint16_t chunkNumber = 0;
        const uint16_t maxChunks = 10; // Safety limit
        
        while (remaining > 0 && chunkNumber < maxChunks) {
            // Use global buffer to avoid stack allocation issues
            uint16_t responseLen = 0;
            
            // Add command response header
            configReadResponseBuffer[responseLen++] = 0x00; // Response type
            configReadResponseBuffer[responseLen++] = 0x40; // Command echo
            
            // Add chunk number (2 bytes, little endian)
            configReadResponseBuffer[responseLen++] = chunkNumber & 0xFF;
            configReadResponseBuffer[responseLen++] = (chunkNumber >> 8) & 0xFF;
            
            // Add total length (2 bytes, little endian) - only in first chunk
            if (chunkNumber == 0) {
                configReadResponseBuffer[responseLen++] = configLen & 0xFF;
                configReadResponseBuffer[responseLen++] = (configLen >> 8) & 0xFF;
            }
            
            // Calculate chunk size (use very small chunks to avoid crashes)
            uint16_t maxDataSize = 100 - responseLen; // Use 100 bytes max per chunk
            uint16_t chunkSize = min(remaining, (uint32_t)maxDataSize);
            
            // Safety check
            if (chunkSize == 0) {
                writeSerial("ERROR: Chunk size is 0, breaking");
                break;
            }
            
            // Add config data
            memcpy(configReadResponseBuffer + responseLen, configData + offset, chunkSize);
            responseLen += chunkSize;
            
            // Safety checks before sending
            if (responseLen > 100) {
                writeSerial("ERROR: Response too large (" + String(responseLen) + " bytes), skipping");
                break;
            }
            
            if (responseLen == 0) {
                writeSerial("ERROR: Empty response, skipping");
                break;
            }
            
            // Send chunk using sendResponse
            sendResponse(configReadResponseBuffer, responseLen);
            
            // Update counters
        offset += chunkSize;
            remaining -= chunkSize;
            chunkNumber++;
            
            writeSerial("Sent chunk " + String(chunkNumber) + " (" + String(chunkSize) + " bytes)");
            
            // Small delay after BLE notification
            delay(50);
        }
        
        if (chunkNumber >= maxChunks) {
            writeSerial("WARNING: Hit chunk limit, transmission may be incomplete");
        }
        
        writeSerial("Config read response sent (" + String(configLen) + " bytes) in " + String(chunkNumber) + " chunks");
    } else {
        // Send error response
        uint8_t errorResponse[] = {0xFF, 0x40, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config read failed - sent error response");
    }
    
    // Add some debugging to see if we can identify where the crash happens
    writeSerial("About to return from handleReadConfig");
    delay(100);
    writeSerial("handleReadConfig function completed successfully");
}

void handleWriteConfig(uint8_t* data, uint16_t len) {
    if (len == 0) {
        writeSerial("ERROR: No config data received");
        return;
    }
    
    // Check if this is a chunked write (either too large for single transmission or exactly 200+2 bytes indicating first chunk with size)
    if (len > 200 || (len >= 202 && len <= 202)) { // Start chunked write for large data or first chunk with size prefix
        writeSerial("Starting chunked write (received: " + String(len) + " bytes)");
        
        // Initialize chunked write state
        chunkedWriteState.active = true;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.expectedChunks = 0;
        chunkedWriteState.receivedChunks = 0;
        
        if (len >= 202) {
            // First chunk with total size prefix (2 bytes)
            chunkedWriteState.totalSize = data[0] | (data[1] << 8);
            chunkedWriteState.expectedChunks = (chunkedWriteState.totalSize + 200 - 1) / 200; // Round up division
            
            // Copy first chunk data (skip the 2-byte size prefix)
            uint16_t chunkDataSize = min(len - 2, (uint16_t)200);
            memcpy(chunkedWriteState.buffer, data + 2, chunkDataSize);
            chunkedWriteState.receivedSize = chunkDataSize;
            chunkedWriteState.receivedChunks = 1;
            
            writeSerial("First chunk received: " + String(chunkDataSize) + " bytes, total size: " + String(chunkedWriteState.totalSize) + " bytes, expected chunks: " + String(chunkedWriteState.expectedChunks));
        } else {
            // Large single transmission
            chunkedWriteState.totalSize = len;
            chunkedWriteState.expectedChunks = 1;
            
            uint16_t chunkSize = min(len, (uint16_t)200);
            memcpy(chunkedWriteState.buffer, data, chunkSize);
            chunkedWriteState.receivedSize = chunkSize;
            chunkedWriteState.receivedChunks = 1;
            
            writeSerial("Large single transmission: " + String(chunkSize) + " bytes");
        }
        
        // Send acknowledgment for first chunk
        uint8_t ackResponse[] = {0x00, 0x41, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    
    // Single transmission
    if (saveConfig(data, len)) {
        // Send success response
        uint8_t successResponse[] = {0x00, 0x41, 0x00, 0x00}; // Success, command, no data
        sendResponse(successResponse, sizeof(successResponse));
        writeSerial("Config write successful");
    } else {
        // Send error response
        uint8_t errorResponse[] = {0xFF, 0x41, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        writeSerial("Config write failed");
    }
}

void handleWriteConfigChunk(uint8_t* data, uint16_t len) {
    if (!chunkedWriteState.active) {
        writeSerial("ERROR: No chunked write in progress");
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    
    if (len == 0) {
        writeSerial("ERROR: No chunk data received");
        return;
    }
    
    // Safety check for chunk size
    if (len > 200) {
        writeSerial("ERROR: Chunk too large (" + String(len) + " bytes)");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    
    // Check if we have space for this chunk
    if (chunkedWriteState.receivedSize + len > MAX_CONFIG_SIZE) {
        writeSerial("ERROR: Chunk would exceed max config size");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00}; // Error, command, no data
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    
    // Safety check for too many chunks
    if (chunkedWriteState.receivedChunks >= 20) {
        writeSerial("ERROR: Too many chunks received");
        chunkedWriteState.active = false;
        uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    
    // Copy chunk data
    memcpy(chunkedWriteState.buffer + chunkedWriteState.receivedSize, data, len);
    chunkedWriteState.receivedSize += len;
    chunkedWriteState.receivedChunks++;
    
    writeSerial("Chunk " + String(chunkedWriteState.receivedChunks) + "/" + String(chunkedWriteState.expectedChunks) + " received (" + String(len) + " bytes)");
    
    // Check if we have all chunks
    if (chunkedWriteState.receivedChunks >= chunkedWriteState.expectedChunks) {
        writeSerial("All chunks received, saving config (" + String(chunkedWriteState.receivedSize) + " bytes)");
        
        if (saveConfig(chunkedWriteState.buffer, chunkedWriteState.receivedSize)) {
            // Send success response
            uint8_t successResponse[] = {0x00, 0x42, 0x00, 0x00}; // Success, command, no data
            sendResponse(successResponse, sizeof(successResponse));
            writeSerial("Chunked config write successful");
    } else {
            // Send error response
            uint8_t errorResponse[] = {0xFF, 0x42, 0x00, 0x00}; // Error, command, no data
            sendResponse(errorResponse, sizeof(errorResponse));
            writeSerial("Chunked config write failed");
        }
        
        // Reset chunked write state
        chunkedWriteState.active = false;
        chunkedWriteState.receivedSize = 0;
        chunkedWriteState.receivedChunks = 0;
    } else {
        // Send acknowledgment for chunk
        uint8_t ackResponse[] = {0x00, 0x42, 0x00, 0x00}; // Success, command, chunk received
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

bool loadGlobalConfig() {
    writeSerial("Loading global configuration...");
    
    // Initialize global config to zero
    memset(&globalConfig, 0, sizeof(globalConfig));

    writeSerial("Memset ");
    
    // Load config from storage
    static uint8_t configData[MAX_CONFIG_SIZE];
    static uint32_t configLen = MAX_CONFIG_SIZE; // Set to max size for loadConfig
    
    if (!loadConfig(configData, &configLen)) {
        writeSerial("No config found, using defaults");
        globalConfig.loaded = false;
        return false;
    }
    
    if (configLen < 3) {
        writeSerial("Config too short, using defaults");
        globalConfig.loaded = false;
        return false;
    }
    
    // Parse outer packet structure
    uint32_t offset = 0;
    
    // Read length (2 bytes, little endian)
    uint16_t packetLength = configData[offset] | (configData[offset + 1] << 8);
    offset += 2;
    
    // Read version (1 byte)
    globalConfig.version = configData[offset++];
    globalConfig.minor_version = 0; // Not stored in current format
    
    writeSerial("Config version: " + String(globalConfig.version));
    writeSerial("Config length: " + String(packetLength) + " bytes");
    writeSerial("Struct sizes - SystemConfig: " + String(sizeof(struct SystemConfig)) + 
               ", ManufacturerData: " + String(sizeof(struct ManufacturerData)) + 
               ", PowerOption: " + String(sizeof(struct PowerOption)) + 
               ", DisplayConfig: " + String(sizeof(struct DisplayConfig)));
    
    // Parse individual packets
    while (offset < configLen - 2) { // -2 for CRC
        if (offset + 2 > configLen - 2) break;
        
        uint8_t packetNumber = configData[offset++];
        uint8_t packetId = configData[offset++];
        
        writeSerial("Parsing packet " + String(packetNumber) + " with ID 0x" + String(packetId, HEX) + " at offset " + String(offset - 2));
        writeSerial("Raw bytes at offset " + String(offset - 2) + ": " + String(configData[offset-2], HEX) + " " + String(configData[offset-1], HEX) + " " + String(configData[offset], HEX) + " " + String(configData[offset+1], HEX) + " " + String(configData[offset+2], HEX) + " " + String(configData[offset+3], HEX) + " " + String(configData[offset+4], HEX) + " " + String(configData[offset+5], HEX));
        
        switch (packetId) {
            case 0x01: // system_config
                if (offset + sizeof(struct SystemConfig) <= configLen - 2) {
                    memcpy(&globalConfig.system_config, &configData[offset], sizeof(struct SystemConfig));
                    offset += sizeof(struct SystemConfig);
                    writeSerial("Loaded system_config");
                } else {
                    writeSerial("ERROR: Not enough data for system_config");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x02: // manufacturer_data
                if (offset + sizeof(struct ManufacturerData) <= configLen - 2) {
                    memcpy(&globalConfig.manufacturer_data, &configData[offset], sizeof(struct ManufacturerData));
                    offset += sizeof(struct ManufacturerData);
                    writeSerial("Loaded manufacturer_data");
                } else {
                    writeSerial("ERROR: Not enough data for manufacturer_data");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x04: // power_option
                if (offset + sizeof(struct PowerOption) <= configLen - 2) {
                    memcpy(&globalConfig.power_option, &configData[offset], sizeof(struct PowerOption));
                    offset += sizeof(struct PowerOption);
                    writeSerial("Loaded power_option, next offset will be: " + String(offset));
                } else {
                    writeSerial("ERROR: Not enough data for power_option");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x20: // display
                writeSerial("Processing display packet, current count: " + String(globalConfig.display_count));
                writeSerial("Remaining bytes: " + String(configLen - 2 - offset) + ", needed: " + String(sizeof(struct DisplayConfig)));
                if (globalConfig.display_count < 4 && offset + sizeof(struct DisplayConfig) <= configLen - 2) {
                    memcpy(&globalConfig.displays[globalConfig.display_count], &configData[offset], sizeof(struct DisplayConfig));
                    offset += sizeof(struct DisplayConfig);
                    globalConfig.display_count++;
                    writeSerial("Loaded display " + String(globalConfig.display_count - 1) + " (instance: " + String(globalConfig.displays[globalConfig.display_count - 1].instance_number) + ")");
                } else if (globalConfig.display_count >= 4) {
                    writeSerial("WARNING: Maximum display count reached, skipping");
                    offset += sizeof(struct DisplayConfig);
                } else {
                    writeSerial("ERROR: Not enough data for display");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x21: // led
                if (globalConfig.led_count < 4 && offset + sizeof(struct LedConfig) <= configLen - 2) {
                    memcpy(&globalConfig.leds[globalConfig.led_count], &configData[offset], sizeof(struct LedConfig));
                    offset += sizeof(struct LedConfig);
                    globalConfig.led_count++;
                    writeSerial("Loaded LED " + String(globalConfig.led_count - 1));
                } else if (globalConfig.led_count >= 4) {
                    writeSerial("WARNING: Maximum LED count reached, skipping");
                    offset += sizeof(struct LedConfig);
                } else {
                    writeSerial("ERROR: Not enough data for LED");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x23: // sensor_data
                if (globalConfig.sensor_count < 4 && offset + sizeof(struct SensorData) <= configLen - 2) {
                    memcpy(&globalConfig.sensors[globalConfig.sensor_count], &configData[offset], sizeof(struct SensorData));
                    offset += sizeof(struct SensorData);
                    globalConfig.sensor_count++;
                    writeSerial("Loaded sensor " + String(globalConfig.sensor_count - 1));
                } else if (globalConfig.sensor_count >= 4) {
                    writeSerial("WARNING: Maximum sensor count reached, skipping");
                    offset += sizeof(struct SensorData);
                } else {
                    writeSerial("ERROR: Not enough data for sensor");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x24: // data_bus
                if (globalConfig.data_bus_count < 4 && offset + sizeof(struct DataBus) <= configLen - 2) {
                    memcpy(&globalConfig.data_buses[globalConfig.data_bus_count], &configData[offset], sizeof(struct DataBus));
                    offset += sizeof(struct DataBus);
                    globalConfig.data_bus_count++;
                    writeSerial("Loaded data_bus " + String(globalConfig.data_bus_count - 1));
                } else if (globalConfig.data_bus_count >= 4) {
                    writeSerial("WARNING: Maximum data_bus count reached, skipping");
                    offset += sizeof(struct DataBus);
                } else {
                    writeSerial("ERROR: Not enough data for data_bus");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            case 0x25: // binary_inputs
                if (globalConfig.binary_input_count < 4 && offset + sizeof(struct BinaryInputs) <= configLen - 2) {
                    memcpy(&globalConfig.binary_inputs[globalConfig.binary_input_count], &configData[offset], sizeof(struct BinaryInputs));
                    offset += sizeof(struct BinaryInputs);
                    globalConfig.binary_input_count++;
                    writeSerial("Loaded binary_input " + String(globalConfig.binary_input_count - 1));
                } else if (globalConfig.binary_input_count >= 4) {
                    writeSerial("WARNING: Maximum binary_input count reached, skipping");
                    offset += sizeof(struct BinaryInputs);
                } else {
                    writeSerial("ERROR: Not enough data for binary_input");
                    globalConfig.loaded = false;
                    return false;
                }
                break;
                
            default:
                writeSerial("WARNING: Unknown packet ID 0x" + String(packetId, HEX) + ", skipping");
                // Skip unknown packet - we can't determine its size, so we'll stop parsing
                offset = configLen - 2; // Skip to CRC
                break;
        }
    }
    
    // Verify CRC
    if (offset < configLen - 2) {
        uint16_t crcGiven = configData[configLen - 2] | (configData[configLen - 1] << 8);
        uint16_t crcCalculated = calculateConfigCRC(configData, configLen - 2);
        
        if (crcGiven != crcCalculated) {
            writeSerial("WARNING: Config CRC mismatch (given: 0x" + String(crcGiven, HEX) + 
                       ", calculated: 0x" + String(crcCalculated, HEX) + ")");
        } else {
            writeSerial("Config CRC verified");
        }
    }
    
    globalConfig.loaded = true;
    writeSerial("Global config loaded successfully");
    writeSerial("Loaded: " + String(globalConfig.display_count) + " displays, " + 
               String(globalConfig.led_count) + " LEDs, " + 
               String(globalConfig.sensor_count) + " sensors, " + 
               String(globalConfig.data_bus_count) + " data buses, " + 
               String(globalConfig.binary_input_count) + " binary inputs");
    
    return true;
}

void printConfigSummary() {
    if (!globalConfig.loaded) {
        writeSerial("Config not loaded");
        return;
    }
    
    writeSerial("=== Configuration Summary ===");
    writeSerial("Version: " + String(globalConfig.version) + "." + String(globalConfig.minor_version));
    writeSerial("Loaded: " + String(globalConfig.loaded ? "Yes" : "No"));
    writeSerial("");
    
    // System Config
    writeSerial("--- System Configuration ---");
    writeSerial("IC Type: 0x" + String(globalConfig.system_config.ic_type, HEX));
    writeSerial("Communication Modes: 0x" + String(globalConfig.system_config.communication_modes, HEX));
    writeSerial("Device Flags: 0x" + String(globalConfig.system_config.device_flags, HEX));
    writeSerial("Power Pin: " + String(globalConfig.system_config.pwr_pin));
    writeSerial("");
    
    // Manufacturer Data
    writeSerial("--- Manufacturer Data ---");
    writeSerial("Manufacturer ID: 0x" + String(globalConfig.manufacturer_data.manufacturer_id, HEX));
    writeSerial("Board Type: " + String(globalConfig.manufacturer_data.board_type));
    writeSerial("Board Revision: " + String(globalConfig.manufacturer_data.board_revision));
    writeSerial("");
    
    // Power Option
    writeSerial("--- Power Configuration ---");
    writeSerial("Power Mode: " + String(globalConfig.power_option.power_mode));
    writeSerial("Battery Capacity: " + String(globalConfig.power_option.battery_capacity_mah[0]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[1]) + 
               " " + String(globalConfig.power_option.battery_capacity_mah[2]) + " mAh");
    writeSerial("Sleep Timeout: " + String(globalConfig.power_option.sleep_timeout_ms) + " ms");
    writeSerial("TX Power: " + String(globalConfig.power_option.tx_power));
    writeSerial("Sleep Flags: 0x" + String(globalConfig.power_option.sleep_flags, HEX));
    writeSerial("Battery Sense Pin: " + String(globalConfig.power_option.battery_sense_pin));
    writeSerial("Battery Sense Enable Pin: " + String(globalConfig.power_option.battery_sense_enable_pin));
    writeSerial("Battery Sense Flags: 0x" + String(globalConfig.power_option.battery_sense_flags, HEX));
    writeSerial("Capacity Estimator: " + String(globalConfig.power_option.capacity_estimator));
    writeSerial("Voltage Scaling Factor: " + String(globalConfig.power_option.voltage_scaling_factor));
    writeSerial("Deep Sleep Current: " + String(globalConfig.power_option.deep_sleep_current_ua) + " uA");
    writeSerial("");
    
    // Displays
    writeSerial("--- Display Configurations (" + String(globalConfig.display_count) + ") ---");
    for (int i = 0; i < globalConfig.display_count; i++) {
        writeSerial("Display " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.displays[i].instance_number));
        writeSerial("  Technology: 0x" + String(globalConfig.displays[i].display_technology, HEX));
        writeSerial("  Panel IC Type: 0x" + String(globalConfig.displays[i].panel_ic_type, HEX));
        writeSerial("  Resolution: " + String(globalConfig.displays[i].pixel_width) + "x" + String(globalConfig.displays[i].pixel_height));
        writeSerial("  Size: " + String(globalConfig.displays[i].active_width_mm) + "x" + String(globalConfig.displays[i].active_height_mm) + " mm");
        writeSerial("  OEPL Tag Type: 0x" + String(globalConfig.displays[i].oepl_tagtype, HEX));
        writeSerial("  Rotation: " + String(globalConfig.displays[i].rotation * 90) + " degrees");
        writeSerial("  Reset Pin: " + String(globalConfig.displays[i].reset_pin));
        writeSerial("  Busy Pin: " + String(globalConfig.displays[i].busy_pin));
        writeSerial("  DC Pin: " + String(globalConfig.displays[i].dc_pin));
        writeSerial("  CS Pin: " + String(globalConfig.displays[i].cs_pin));
        writeSerial("  Data Pin: " + String(globalConfig.displays[i].data_pin));
        writeSerial("  Partial Update: " + String(globalConfig.displays[i].partial_update_support ? "Yes" : "No"));
        writeSerial("  Color Scheme: 0x" + String(globalConfig.displays[i].color_scheme, HEX));
        writeSerial("  Transmission Modes: 0x" + String(globalConfig.displays[i].transmission_modes, HEX));
        writeSerial("");

        epd = BBEPAPER(globalConfig.displays[i].panel_ic_type);
        CS_PIN = globalConfig.displays[i].cs_pin;
        DC_PIN = globalConfig.displays[i].dc_pin;
        RESET_PIN = globalConfig.displays[i].reset_pin;
        BUSY_PIN = globalConfig.displays[i].busy_pin;
        CLK_PIN = globalConfig.displays[i].reserved_pin_1;
        MOSI_PIN = globalConfig.displays[i].data_pin;

    }
    
    // LEDs
    writeSerial("--- LED Configurations (" + String(globalConfig.led_count) + ") ---");
    for (int i = 0; i < globalConfig.led_count; i++) {
        writeSerial("LED " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.leds[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.leds[i].led_type, HEX));
        writeSerial("  Pins: R=" + String(globalConfig.leds[i].led_1_r) + 
                   " G=" + String(globalConfig.leds[i].led_2_g) + 
                   " B=" + String(globalConfig.leds[i].led_3_b) + 
                   " 4=" + String(globalConfig.leds[i].led_4));
        writeSerial("  Flags: 0x" + String(globalConfig.leds[i].led_flags, HEX));
        writeSerial("");
    }
    
    // Sensors
    writeSerial("--- Sensor Configurations (" + String(globalConfig.sensor_count) + ") ---");
    for (int i = 0; i < globalConfig.sensor_count; i++) {
        writeSerial("Sensor " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.sensors[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.sensors[i].sensor_type, HEX));
        writeSerial("  Bus ID: " + String(globalConfig.sensors[i].bus_id));
        writeSerial("");
    }
    
    // Data Buses
    writeSerial("--- Data Bus Configurations (" + String(globalConfig.data_bus_count) + ") ---");
    for (int i = 0; i < globalConfig.data_bus_count; i++) {
        writeSerial("Data Bus " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.data_buses[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.data_buses[i].bus_type, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.data_buses[i].pin_1) + 
                   " 2=" + String(globalConfig.data_buses[i].pin_2) + 
                   " 3=" + String(globalConfig.data_buses[i].pin_3) + 
                   " 4=" + String(globalConfig.data_buses[i].pin_4) + 
                   " 5=" + String(globalConfig.data_buses[i].pin_5) + 
                   " 6=" + String(globalConfig.data_buses[i].pin_6) + 
                   " 7=" + String(globalConfig.data_buses[i].pin_7));
        writeSerial("  Speed: " + String(globalConfig.data_buses[i].bus_speed_hz) + " Hz");
        writeSerial("  Flags: 0x" + String(globalConfig.data_buses[i].bus_flags, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.data_buses[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.data_buses[i].pulldowns, HEX));
        writeSerial("");
    }
    
    // Binary Inputs
    writeSerial("--- Binary Input Configurations (" + String(globalConfig.binary_input_count) + ") ---");
    for (int i = 0; i < globalConfig.binary_input_count; i++) {
        writeSerial("Binary Input " + String(i) + ":");
        writeSerial("  Instance: " + String(globalConfig.binary_inputs[i].instance_number));
        writeSerial("  Type: 0x" + String(globalConfig.binary_inputs[i].input_type, HEX));
        writeSerial("  Display As: 0x" + String(globalConfig.binary_inputs[i].display_as, HEX));
        writeSerial("  Pins: 1=" + String(globalConfig.binary_inputs[i].reserved_pin_1) + 
                   " 2=" + String(globalConfig.binary_inputs[i].reserved_pin_2) + 
                   " 3=" + String(globalConfig.binary_inputs[i].reserved_pin_3) + 
                   " 4=" + String(globalConfig.binary_inputs[i].reserved_pin_4) + 
                   " 5=" + String(globalConfig.binary_inputs[i].reserved_pin_5) + 
                   " 6=" + String(globalConfig.binary_inputs[i].reserved_pin_6) + 
                   " 7=" + String(globalConfig.binary_inputs[i].reserved_pin_7) + 
                   " 8=" + String(globalConfig.binary_inputs[i].reserved_pin_8));
        writeSerial("  Input Flags: 0x" + String(globalConfig.binary_inputs[i].input_flags, HEX));
        writeSerial("  Invert: 0x" + String(globalConfig.binary_inputs[i].invert, HEX));
        writeSerial("  Pullups: 0x" + String(globalConfig.binary_inputs[i].pullups, HEX));
        writeSerial("  Pulldowns: 0x" + String(globalConfig.binary_inputs[i].pulldowns, HEX));
        writeSerial("");
    }
    
    writeSerial("=============================");
}
