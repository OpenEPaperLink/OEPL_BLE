#include <stdint.h>

// Image transfer state variables
struct ImageData {
    uint8_t* data;
    uint32_t size;
    uint32_t received;
    uint8_t dataType;
    bool isCompressed;
    uint32_t crc32;
    uint16_t width;
    uint16_t height;
    bool ready;
    uint32_t totalBlocks;
    uint32_t currentBlock;
    bool* blocksReceived;
    uint32_t* blockBytesReceived;  // Track bytes received per block
    uint32_t* blockPacketsReceived; // Track packets received per block
};

// Dynamic Config structures
struct DynamicConfig {
    // Screen Type (2 bytes)
    uint16_t screenType;
    
    // Default Settings (41 bytes)
    uint16_t hwType;
    uint16_t screenFunctions;
    uint8_t whInversedBle;
    uint16_t whInversed;
    uint16_t screenHeight;
    uint16_t screenWidth;
    uint16_t screenHOffset;
    uint16_t screenWOffset;
    uint16_t screenColors;
    uint16_t blackInvert;
    uint16_t secondColorInvert;
    uint32_t epdPinoutEnabled;
    uint32_t ledPinoutEnabled;
    uint32_t nfcPinoutEnabled;
    uint32_t flashPinoutEnabled;
    uint16_t adcPinout;
    uint16_t uartPinout;
    
    // EPD Pinout (26 bytes)
    uint16_t epdReset;
    uint16_t epdDc;
    uint16_t epdBusy;
    uint16_t epdBusyS;
    uint16_t epdCs;
    uint16_t epdCsS;
    uint16_t epdClk;
    uint16_t epdMosi;
    uint16_t epdEnable;
    uint16_t epdEnable1;
    uint8_t epdEnableInvert;
    uint16_t epdFlashCs;
    uint8_t epdPinConfigSleep;
    uint8_t epdPinEnable;
    uint8_t epdPinEnableSleep;
    
    // LED Pinout (7 bytes) - disabled
    uint16_t ledR;
    uint16_t ledG;
    uint16_t ledB;
    uint8_t ledInverted;
    
    // NFC Pinout (8 bytes) - disabled
    uint16_t nfcSda;
    uint16_t nfcScl;
    uint16_t nfcCs;
    uint16_t nfcIrq;
    
    // Flash Pinout (8 bytes) - disabled
    uint16_t flashCs;
    uint16_t flashClk;
    uint16_t flashMiso;
    uint16_t flashMosi;
};