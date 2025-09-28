

#define USER_SETUP_ID 506

#define SSD1677_DRIVER

#define EPAPER_ENABLE

#define TFT_WIDTH 800
#define TFT_HEIGHT 480

#define EPD_WIDTH TFT_WIDTH
#define EPD_HEIGHT TFT_HEIGHT

#define EPD_HORIZONTAL_MIRROR

#define TFT_SCLK EPD_SCK
#define TFT_MOSI EPD_MOSI
#define TFT_MISO -1
#define TFT_CS EPD_CS
#define TFT_DC EPD_DC
#define TFT_BUSY EPD_BUSY
#define TFT_RST EPD_RST

#define LOAD_GLCD  // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2 // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4 // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6 // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7 // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8 // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
// #define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

#define SPI_FREQUENCY 10000000
#define SPI_READ_FREQUENCY 4000000
#define ltoa itoa
