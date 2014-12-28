#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION "RFM69 0.3.3"

//#define DEFAULT_NODE_ADDR 0x42 // Garden
//#define DEFAULT_NODE_ADDR 0x43 // Master RX
//#define DEFAULT_NODE_ADDR 0x44 // 3rd
//#define DEFAULT_NODE_ADDR 0x45 // 4th
#define DEFAULT_NODE_ADDR 0x46 // 5th

#define DEFAULT_WATCHDOG_TIMEOUT 100000

// When in low power polling mode, timeout in 10ms units to trigger reset
#define DEFAULT_LINK_LOSS_TIMEOUT 0

// Version of MCU used (LPC812, LPC810 supported)
//#define LPC810
#define LPC812

// What PCB board or pin layout?
//#define BOARD_LPC812_V0 // Deadbug LPC812 made back in Sep 2014.
#define BOARD_LPC812_V1  // First rev of LPC812 RFM69/98 PCB (Nov 2014).


//
// Optional features are enabled by defining macro FEATURE_xxxxxx. As space is very
// limited on LPC810 (4KiB) not all features can be supported simultaneously.
//

// Remote register read/write
#define FEATURE_REMOTE_REG_READ
#define FEATURE_REMOTE_REG_WRITE





#define FEATURE_SLEEP

// Support MCU deep-sleep (~10uA)
#define FEATURE_DEEPSLEEP

// Allow full UART command to be executed remotely
#define FEATURE_REMOTE_COMMAND



// Read RFM69 temperature sensor
#define FEATURE_TEMPERATURE

// Remote MCU memory read/write and execute (+160bytes)
#define FEATURE_REMOTE_MEM_RWX

// MCU memory read/write/exec from UART API
#define FEATURE_UART_MEM_RWX

#ifdef LPC812
// Diagnostic LED (only available on LPC812)
#define FEATURE_LED
// Enable ARM Cortex M SysTick timer
#define FEATURE_SYSTICK
// Experimental application to count rain tip bucket
//#define FEATURE_EVENT_COUNTER
// DS18B20 one wire temperature sensor
//#define FEATURE_DS18B20
// Allow change of UART speed from default 9600
#define FEATURE_UART_SPEED
// When sleeping trigger PIN interrupt on RXD line
#define FEATURE_UART_INTERRUPT
// Support NMEA sentences for location
#define FEATURE_NMEA_INPUT
// Remote packet broadcast
#define FEATURE_REMOTE_PKT_TX
// Report unique serial number of MCU
#define FEATURE_MCU_UID
// Use WDT to reset MCU on loss of link
#define FEATURE_LINK_LOSS_RESET

// Use LPC8xx watchdog timer
//#define FEATURE_WATCHDOG_TIMER

#endif


//
// Pins used for SPI (note: pin numbers are are PIO0_x, *not* package pin numbers)
//
#ifdef LPC812

#ifdef BOARD_LPC812_V0
// The hand wired proto board
#define SCK_PIN 15
#define SS_PIN 9
#define MOSI_PIN 8
#define MISO_PIN 7
#define TIPBUCKET_PIN 17
#endif

#ifdef BOARD_LPC812_V1
// PCB v1
#define SCK_PIN 7
#define SS_PIN 1
#define MOSI_PIN 9
#define MISO_PIN 8
#define RESET_PIN 15
//#define DIO0_PIN 6
#define TIPBUCKET_PIN 16
#define LED_PIN 17
#define UART_RXD_PIN 0
#define DS18B20_PIN 14
#endif

// Enable reroute of RXD to alternative pin due to failure to fix via bug.
//#define BOARD_V1B_HACK
#ifdef BOARD_V1B_HACK
#define UART_RXD_PIN 11
#endif

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



#endif
