#include <Arduino.h>
#include "structs.h"

#define EPD_RST   15
#define EPD_CS    44
#define EPD_BUSY  29
#define EPD_DC    31
#define EPD_SCK   45
#define EPD_MOSI  47
#define PowerPin 43

void pwrmgm(bool onoff);
void writeSerial(String message, bool newLine = true);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void initDisplay();
void hibernateDisplay();
void writeDisplayData();
String getChipIdHex();
void imageDataWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void handleImageCommand(uint8_t* data, uint16_t len);
void handleImageInfo(uint8_t* data, uint16_t len);
void handleBlockData(uint8_t* data, uint16_t len);
void handleReadDynamicConfig();
void buildDynamicConfigResponse(uint8_t* buffer, uint16_t* len);
void sendResponse(uint8_t* response, uint8_t len);
uint32_t calculateCRC32(uint8_t* data, uint32_t len);
void displayReceivedImage();
void drawImageData();
bool decompressImageData(uint8_t** output, uint32_t* outputSize);
void drawLogo(int x, int y);
void busyCallback(const void*);