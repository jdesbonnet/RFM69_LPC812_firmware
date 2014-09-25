#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION "RFM69 0.2.7"

#define DEFAULT_NODE_ADDR 0x41

// Version of MCU used (LPC812, LPC810 supported)
#define LPC810
//#define LPC812

//
// Optional features are enabled by defining macro FEATURE_xxxxxx. As space is very
// limited on LPC810 (4KiB) not all features can be supported simultaneously.
//

// Remote register read/write
#define FEATURE_REMOTE_REG_READ
#define FEATURE_REMOTE_REG_WRITE

// Remote packet broadcast
//#define FEATURE_REMOTE_PKT_TX

// Support NMEA sentences for location
//#define FEATURE_NMEA_INPUT

#define FEATURE_SLEEP

// Support MCU deep-sleep (~10uA)
#define FEATURE_DEEPSLEEP

// Allow full UART command to be executed remotely
#define FEATURE_REMOTE_COMMAND

// Allow change of UART speed from default 9600
//#define FEATURE_UART_SPEED

// When sleeping trigger PIN interrupt on RXD line
//#define FEATURE_UART_INTERRUPT

// Read RFM69 temperature sensor
#define FEATURE_TEMPERATURE

// Remote MCU memory read/write and execute (+160bytes)
#define FEATURE_REMOTE_MEM_RWX

// MCU memory read/write/exec from UART API
#define FEATURE_UART_MEM_RWX

// Diagnostic LED (only available on LPC812)
#ifdef LPC812
#define FEATURE_LED
#define LED_PIN 14
#endif


//
// Pins used for SPI (note: pin numbers are are PIO0_x, *not* package pin numbers)
//
#ifdef LPC812
#define SCK_PIN 15
#define SS_PIN 9
#define MOSI_PIN 8
#define MISO_PIN 7
#define TIPBUCKET_PIN 17
#endif

#ifdef LPC810
#define MISO_PIN 3  // package pin 3
#define SS_PIN 5   // package pin 1
#define SCK_PIN 2 // package pin 4
#define MOSI_PIN 1 // package pin 5
#endif




// UART speed ('baud rate')
#define UART_BPS (9600)
//#define UART_BPS (115200)

#ifdef LPC812
// Enable ARM Cortex M SysTick timer
#define FEATURE_SYSTICK
// Experimental application to count rain tip bucket
//#define FEATURE_EVENT_COUNTER
#endif

#endif
