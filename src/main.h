#include <Arduino.h>
#include "structs.h"
#include <bluefruit.h>
#include "uzlib.h"
#include <bb_epaper.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define DECOMP_CHUNK 512
#define MAX_DECOMPRESSED_SIZE (512 * 1024)  // 512KB max decompressed size
#define DECOMP_CHUNK_SIZE 4096              // 4KB chunks for processing
#define MAX_DICT_SIZE 32768                 // Maximum dictionary size
#define MAX_IMAGE_SIZE (50 * 1024)          // 50KB max image data buffer
#define MAX_BLOCKS 16                       // Support up to 50KB images in 4KB blocks
#define CONFIG_FILE_PATH "/config.bin"      // Config file path in LittleFS
#define MAX_CONFIG_SIZE 4096                // 4KB max config size

BBEPAPER epd(EP426_800x480);

uint32_t Led_RED = 26;
uint32_t Led_Green = 30;
uint32_t Led_Blue = 06;
uint32_t CS_PIN = 44;
uint32_t DC_PIN = 31;
uint32_t RESET_PIN = 15;
uint32_t BUSY_PIN = 29;
uint32_t CLK_PIN = 45;
uint32_t MOSI_PIN = 47;
uint32_t PowerPin = 43;

BLEDfu bledfu;

BLEService imageService("1337");
BLECharacteristic imageCharacteristic("1337",BLEWrite | BLENotify, 512);

ImageData currentImage = {0};
uint8_t currentBlockId = 0;
uint8_t currentPacketId = 0;
uint8_t expectedPackets = 0;
uint8_t receivedPackets = 0;

uint8_t imageDataBuffer[MAX_IMAGE_SIZE];           // 50KB image data buffer
bool blocksReceived[MAX_BLOCKS];                   // Block tracking arrays
uint32_t blockBytesReceived[MAX_BLOCKS];
uint32_t blockPacketsReceived[MAX_BLOCKS];
uint8_t decompressionChunk[DECOMP_CHUNK_SIZE];     // 4KB decompression chunk
uint8_t dictionaryBuffer[MAX_DICT_SIZE];           // 32KB dictionary
uint8_t headerBuffer[6];                           // 6-byte image header
uint8_t bleResponseBuffer[94];                     // BLE response buffer

// Config storage structure
typedef struct {
    uint32_t magic;        // 0xDEADBEEF for validation
    uint32_t version;      // Config version
    uint32_t crc;          // CRC32 of config data
    uint32_t data_len;     // Actual data length
    uint8_t data[MAX_CONFIG_SIZE]; // Config data
} config_storage_t;

// LittleFS namespace
using namespace Adafruit_LittleFS_Namespace;

void pwrmgm(bool onoff);
void writeSerial(String message, bool newLine = true);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void initDisplay();
void hibernateDisplay();
void writeDisplayData();
String getChipIdHex();
void imageDataWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void handleImageInfo(uint8_t* data, uint16_t len);
void handleBlockData(uint8_t* data, uint16_t len);
void handleReadDynamicConfig();
void buildDynamicConfigResponse(uint8_t* buffer, uint16_t* len);
void handleDisplayInfo();
void sendResponse(uint8_t* response, uint8_t len);
uint32_t calculateCRC32(uint8_t* data, uint32_t len);
void displayReceivedImage();
void drawImageData();
bool decompressImageData(uint8_t** output, uint32_t* outputSize);
void drawLogo(int x, int y);
void busyCallback(const void*);
void initio();
void updatemsdata();
void cleanupImageMemory();
bool decompressImageDataChunked();

// Config storage functions
bool initConfigStorage();
bool saveConfig(uint8_t* configData, uint32_t len);
bool loadConfig(uint8_t* configData, uint32_t* len);
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);
void printConfigSummary();

// Chunked config write state
typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;

extern chunked_write_state_t chunkedWriteState;

chunked_write_state_t chunkedWriteState = {false, 0, 0, {0}, 0, 0};
struct GlobalConfig globalConfig = {0};
uint8_t configReadResponseBuffer[128];

// Global configuration instance
extern struct GlobalConfig globalConfig;

// Config loading function
bool loadGlobalConfig();

// Config access helper functions
bool isConfigLoaded();
struct SystemConfig* getSystemConfig();
struct ManufacturerData* getManufacturerData();
struct PowerOption* getPowerOption();
struct DisplayConfig* getDisplayConfig(uint8_t index);
uint8_t getDisplayCount();
struct LedConfig* getLedConfig(uint8_t index);
uint8_t getLedCount();
struct SensorData* getSensorData(uint8_t index);
uint8_t getSensorCount();
struct DataBus* getDataBus(uint8_t index);
uint8_t getDataBusCount();
struct BinaryInputs* getBinaryInput(uint8_t index);
uint8_t getBinaryInputCount();
void printConfigSummary();