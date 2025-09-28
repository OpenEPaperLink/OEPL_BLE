#ifndef _SEEED_XIAO_NRF52840_SENSE_H_
#define _SEEED_XIAO_NRF52840_SENSE_H_

#define TARGET_SEEED_XIAO_NRF52840_SENSE

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
//#define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define PINS_COUNT           (64)
#define NUM_DIGITAL_PINS     (64)
#define NUM_ANALOG_INPUTS    (8) // A6 is used for battery, A7 is analog reference
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
#define PIN_LED              LED_RED
#define LED_PWR              (PINS_COUNT)
#define PIN_NEOPIXEL         (PINS_COUNT)
#define NEOPIXEL_NUM         0

#define LED_BUILTIN          PIN_LED

#define LED_RED              11
#define LED_GREEN            13
#define LED_BLUE             12

#define LED_STATE_ON         1         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          (PINS_COUNT)

//Digital PINs
#define D0 (2)
#define D1 (3)
#define D2 (28)
#define D3 (29)
#define D4 (4)
#define D5 (5)
#define D6 (43)
#define D7 (44)
#define D8 (45)
#define D9 (46)
#define D10 (47)

/*
 * Analog pins
 */
#define PIN_A0               (2)
#define PIN_A1               (3)
#define PIN_A2               (28)
#define PIN_A3               (29)
#define PIN_A4               (4)
#define PIN_A5               (5)
#define PIN_VBAT             (31)
#define VBAT_ENABLE          (14)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
#define ADC_RESOLUTION    12

// Other pins
#define PIN_NFC1           (9)
#define PIN_NFC2           (10)

//static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX       (9)
#define PIN_SERIAL1_TX       (10)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO         (46)
#define PIN_SPI_MOSI         (47)
#define PIN_SPI_SCK          (45)

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#define PIN_SPI1_MISO         (37)
#define PIN_SPI1_MOSI         (39)
#define PIN_SPI1_SCK          (35)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (4)
#define PIN_WIRE_SCL         (5)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// QSPI Pins
#define PIN_QSPI_SCK         (21)
#define PIN_QSPI_CS          (25)
#define PIN_QSPI_IO0         (20)
#define PIN_QSPI_IO1         (24)
#define PIN_QSPI_IO2         (22)
#define PIN_QSPI_IO3         (23)

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   P25Q16H
#define EXTERNAL_FLASH_USE_QSPI

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
