#include <Arduino.h>
#include "structs.h"
#include "uzlib.h"
#include <bb_epaper.h>

#ifndef BUILD_VERSION
#define BUILD_VERSION "0.0"
#endif
#ifndef SHA
#define SHA ""
#endif

#define STRINGIFY(x) #x
#define XSTRINGIFY(x) STRINGIFY(x)
#define SHA_STRING XSTRINGIFY(SHA)

#ifdef TARGET_NRF
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#endif

#ifdef TARGET_ESP32
#include <LittleFS.h>
#endif

#include <Wire.h>

#define DECOMP_CHUNK 512
#define MAX_DECOMPRESSED_SIZE (512 * 1024)
#define DECOMP_CHUNK_SIZE 4096
#define MAX_DICT_SIZE 32768
#ifdef TARGET_LARGE_MEMORY
#define MAX_IMAGE_SIZE (100 * 1024)
#define MAX_COMPRESSED_SIZE (200 * 1024)
#else
#define MAX_IMAGE_SIZE (50 * 1024)
#define MAX_COMPRESSED_SIZE (100 * 1024)
#endif
#define MAX_BLOCKS 64
#define CONFIG_FILE_PATH "/config.bin"
#define MAX_CONFIG_SIZE 4096

#ifdef TARGET_NRF
#include <bluefruit.h>
extern BLEDfu bledfu;
extern BLEService imageService;
extern BLECharacteristic imageCharacteristic;
// Forward declaration for SoftDevice temperature API
extern "C" uint32_t sd_temp_get(int32_t *p_temp);
#endif

#ifdef TARGET_ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <esp_system.h>

extern BLEServer* pServer;
extern BLEService* pService;
extern BLECharacteristic* pTxCharacteristic;
extern BLECharacteristic* pRxCharacteristic;
extern BLEAdvertisementData* advertisementData;  // Pointer to global advertisementData object
#endif

BBEPAPER epd;

ImageData currentImage = {0};
uint8_t currentBlockId = 0;
uint8_t currentPacketId = 0;
uint8_t expectedPackets = 0;
uint8_t receivedPackets = 0;
uint8_t imageDataBuffer[MAX_IMAGE_SIZE];
bool blocksReceived[MAX_BLOCKS];
uint32_t blockBytesReceived[MAX_BLOCKS];
uint32_t blockPacketsReceived[MAX_BLOCKS];
uint8_t decompressionChunk[DECOMP_CHUNK_SIZE];
uint8_t dictionaryBuffer[MAX_DICT_SIZE];
uint8_t headerBuffer[6];
uint8_t bleResponseBuffer[94];
uint8_t mloopcounter = 0;

char wifiSsid[33] = {0};  // 32 bytes + null terminator
char wifiPassword[33] = {0};  // 32 bytes + null terminator
uint8_t wifiEncryptionType = 0;  // 0x00=none, 0x01=WEP, 0x02=WPA, 0x03=WPA2, 0x04=WPA3
bool wifiConfigured = false;  // True if WiFi config packet (0x26) was received and parsed

bool waitforrefresh(int timeout);
void pwrmgm(bool onoff);
void writeSerial(String message, bool newLine = true);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void initDisplay();
void hibernateDisplay();
void writeDisplayData();
String getChipIdHex();
#ifdef TARGET_NRF
void imageDataWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
#else
void imageDataWritten(void* conn_hdl, void* chr, uint8_t* data, uint16_t len);
#endif
void handleImageInfo(uint8_t* data, uint16_t len);
void handleBlockData(uint8_t* data, uint16_t len);
void handleReadDynamicConfig();
void buildDynamicConfigResponse(uint8_t* buffer, uint16_t* len);
void handleDisplayInfo();
void sendResponse(uint8_t* response, uint8_t len);
uint32_t calculateCRC32(uint8_t* data, uint32_t len);
void displayReceivedImage();
void drawImageData();
void drawLogo(int x, int y);
void busyCallback(const void*);
void initio();
void initDataBuses();
void scanI2CDevices();
void initSensors();
void initAXP2101(uint8_t busId);
void readAXP2101Data();
void updatemsdata();
void cleanupImageMemory();
bool decompressImageDataChunked();
void ble_init();
void full_config_init();
void formatConfigStorage();
bool initConfigStorage();
bool saveConfig(uint8_t* configData, uint32_t len);
bool loadConfig(uint8_t* configData, uint32_t* len);
uint32_t calculateConfigCRC(uint8_t* data, uint32_t len);
void handleReadConfig();
void handleWriteConfig(uint8_t* data, uint16_t len);
void handleWriteConfigChunk(uint8_t* data, uint16_t len);
void handleFirmwareVersion();
void printConfigSummary();
void reboot();
bool loadGlobalConfig();
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
int mapEpd(int id);
uint8_t getFirmwareMajor();
uint8_t getFirmwareMinor();
float readBatteryVoltage();  // Returns battery voltage in volts, or -1.0 if not configured
float readChipTemperature();  // Returns chip temperature in degrees Celsius

typedef struct {
    bool active;
    uint32_t totalSize;
    uint32_t receivedSize;
    uint8_t buffer[MAX_CONFIG_SIZE];
    uint32_t expectedChunks;
    uint32_t receivedChunks;
} chunked_write_state_t;

typedef struct {
    uint32_t magic;        
    uint32_t version;
    uint32_t crc;
    uint32_t data_len;
    uint8_t data[MAX_CONFIG_SIZE];
} config_storage_t;

extern chunked_write_state_t chunkedWriteState;
chunked_write_state_t chunkedWriteState = {false, 0, 0, {0}, 0, 0};
struct GlobalConfig globalConfig = {0};
uint8_t configReadResponseBuffer[128];

extern struct GlobalConfig globalConfig;

#define AXP2101_SLAVE_ADDRESS 0x34
#define AXP2101_REG_POWER_STATUS 0x00
#define AXP2101_REG_POWER_ON_STATUS 0x01
#define AXP2101_REG_POWER_OFF_STATUS 0x02
#define AXP2101_REG_DC_ONOFF_DVM_CTRL 0x80
#define AXP2101_REG_LDO_ONOFF_CTRL0 0x90
#define AXP2101_REG_DC_VOL0_CTRL 0x82  // DCDC1 voltage
#define AXP2101_REG_LDO_VOL2_CTRL 0x94  // ALDO3 voltage
#define AXP2101_REG_LDO_VOL3_CTRL 0x95  // ALDO4 voltage
#define AXP2101_REG_POWER_WAKEUP_CTL 0x26
#define AXP2101_REG_ADC_CHANNEL_CTRL 0x30
#define AXP2101_REG_ADC_DATA_BAT_VOL_H 0x34
#define AXP2101_REG_ADC_DATA_BAT_VOL_L 0x35
#define AXP2101_REG_ADC_DATA_VBUS_VOL_H 0x36
#define AXP2101_REG_ADC_DATA_VBUS_VOL_L 0x37
#define AXP2101_REG_ADC_DATA_SYS_VOL_H 0x38
#define AXP2101_REG_ADC_DATA_SYS_VOL_L 0x39
#define AXP2101_REG_BAT_PERCENT_DATA 0xA4
#define AXP2101_REG_PWRON_STATUS 0x20

#ifdef TARGET_NRF
BLEDfu bledfu;
BLEService imageService("2446");
BLECharacteristic imageCharacteristic("2446", BLEWrite | BLEWriteWithoutResponse | BLENotify, 512);
#endif

#ifdef TARGET_ESP32
// Define queue sizes and structures first
#define RESPONSE_QUEUE_SIZE 10
#define MAX_RESPONSE_SIZE 512
#define COMMAND_QUEUE_SIZE 5
#define MAX_COMMAND_SIZE 512

struct ResponseQueueItem {
    uint8_t data[MAX_RESPONSE_SIZE];
    uint16_t len;
    bool pending;
};

struct CommandQueueItem {
    uint8_t data[MAX_COMMAND_SIZE];
    uint16_t len;
    bool pending;
};

ResponseQueueItem responseQueue[RESPONSE_QUEUE_SIZE];
uint8_t responseQueueHead = 0;
uint8_t responseQueueTail = 0;

CommandQueueItem commandQueue[COMMAND_QUEUE_SIZE];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;

class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        writeSerial("=== BLE CLIENT CONNECTED (ESP32) ===");
        writeSerial("Client connected to ESP32 BLE server");
        delay(100);  // Give connection time to fully establish
        writeSerial("Number of connected clients: " + String(pServer->getConnectedCount()));
    }
    void onDisconnect(BLEServer* pServer) {
        writeSerial("=== BLE CLIENT DISCONNECTED (ESP32) ===");
        writeSerial("Client disconnected from ESP32 BLE server");
        writeSerial("Number of remaining clients: " + String(pServer->getConnectedCount()));
        writeSerial("Waiting before restarting advertising...");
        delay(500);
        if (pServer->getConnectedCount() == 0) {
            BLEDevice::startAdvertising();
            writeSerial("Advertising restarted");
        } else {
            writeSerial("Other clients still connected, not restarting advertising");
        }
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) {
        writeSerial("=== BLE WRITE RECEIVED (ESP32) ===");
        String value = pCharacteristic->getValue();
        writeSerial("Received data length: " + String(value.length()) + " bytes");
        if (value.length() > 0 && value.length() <= MAX_COMMAND_SIZE) {
            uint8_t* data = (uint8_t*)value.c_str();
            uint16_t len = value.length();
            // Log first few bytes
            String hexDump = "Data: ";
            for (int i = 0; i < len && i < 16; i++) {
                if (data[i] < 16) hexDump += "0";
                hexDump += String(data[i], HEX) + " ";
            }
            writeSerial(hexDump);
            uint8_t nextHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
            if (nextHead != commandQueueTail) {
                memcpy(commandQueue[commandQueueHead].data, data, len);
                commandQueue[commandQueueHead].len = len;
                commandQueue[commandQueueHead].pending = true;
                commandQueueHead = nextHead;
                writeSerial("ESP32: Command queued for processing");
            } else {
                writeSerial("ERROR: Command queue full, dropping command");
            }
        } else if (value.length() > MAX_COMMAND_SIZE) {
            writeSerial("WARNING: Command too large, dropping");
        } else {
            writeSerial("WARNING: Empty data received");
        }
    }
};

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
BLEAdvertisementData globalAdvertisementData;  // Global object, not pointer
BLEAdvertisementData* advertisementData = &globalAdvertisementData;  // Pointer to global object
MyBLECharacteristicCallbacks* charCallbacks = nullptr;
#endif
