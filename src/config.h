#ifndef CONFIG_H_
#define CONFIG_H_

//#define VERSION "RFM69 0.6.1"
#define VERSION "RFM69/9x 0.7.1b"

// Experimental directive to load function in RAM to facilitate OTA update
#define RAM_FUNC __attribute__( ( long_call, section(".data.ramfunc") ) )
#define ALWAYS_INLINE __attribute__((always_inline))

// Radio and MCU in sleep state
#define MODE_ALL_OFF (0)

// MCU on, radio in SLEEP (currently not supported)
#define MODE_RADIO_OFF (1)

// Radio and MCU in sleep state with period ping and listen
#define MODE_LOW_POWER_POLL (2)

// Radio in RX, MCU polling radio.
#define MODE_AWAKE (3)



//#define DEFAULT_NODE_ADDR 0x42 // Garden
//#define DEFAULT_NODE_ADDR 0x43 // Master RX
//#define DEFAULT_NODE_ADDR 0x44 // 3rd
//#define DEFAULT_NODE_ADDR 0x45 // 4th
//#define DEFAULT_NODE_ADDR 0x46 // 5th

//#define DEFAULT_MODE MODE_RADIO_OFF
#define DEFAULT_MODE MODE_LOW_POWER_POLL
//#define DEFAULT_MODE MODE_AWAKE


#define DEFAULT_POLL_INTERVAL 30

// Time unit:
#define DEFAULT_WATCHDOG_TIMEOUT 100000

// When in low power polling mode, timeout in seconds units to trigger reset
#define DEFAULT_LINK_LOSS_TIMEOUT 7200

// Time to listen for response after transmitting a packet (in 10ms units)
#define DEFAULT_LISTEN_TIME_CS 50

// Enter low battery mode under this voltage (0.1V units)
#define DEFAULT_LOW_BATTERY_V (25)

// Minimum voltage require to operate radio (0.1V units)
#define DEFAULT_MIN_BATTERY_V (22)

#define DEFAULT_TX_POWER (15)  // RFM9x 0 - 15
#define DEFAULT_LORA_BW   (7)  // RFM9x LoRa BW 0 - 9
#define DEFAULT_LORA_CR   (4)  // RFM9x LoRa CodingRate 0 - 5
#define DEFAULT_LORA_SF  (12)  // RFM9x LoRa SpreadingFactor  6 - 12

#define WWDT_CLOCK_SPEED_HZ (2000)

// MCU used (only LPC812 supported, LPC824 later. LPC810 dropped due to lack of flash)
#define LPC812

// Radio module used
//#define RADIO_RFM69
#define RADIO_RFM9x

// What PCB board or pin layout?
//#define BOARD_LPC812_V0 // Deadbug LPC812 made back in Sep 2014.
//#define BOARD_LPC812_V1  // First rev of LPC812 RFM69/98 PCB (Nov 2014).
#define BOARD_LPC812_RFM98_V1  // First rev of LPC812 RFM69/98 PCB (Nov 2014).

// One board was populated without the trace cut: reroute RXD.
// Enable this in addition to BOARD_LPC812_V1
//#define BOARD_V1B_HACK


//
// Optional features are enabled by defining macro FEATURE_xxxxxx.
//


#define FEATURE_SLEEP

// Support MCU deep-sleep (~3uA)
#define FEATURE_DEEPSLEEP

// Remote MCU memory read/write and execute (+160bytes)
#define FEATURE_REMOTE_MEM_RWX

// MCU memory read/write/exec from UART API
#define FEATURE_UART_MEM_RWX

// Diagnostic LED
#define FEATURE_LED

#define FEATURE_DIO0
//#define FEATURE_DIO1

// Experimental application to count rain tip bucket
//#define FEATURE_EVENT_COUNTER

// DS18B20 one wire temperature sensor
#define FEATURE_DS18B20

// When sleeping trigger PIN interrupt on RXD line
#define FEATURE_UART_INTERRUPT

// Support NMEA sentences for location
#define FEATURE_NMEA_INPUT

// Remote packet broadcast
#define FEATURE_REMOTE_PKT_TX

// Use WDT to reset MCU on loss of link
#define FEATURE_LINK_LOSS_RESET

// Use LPC8xx watchdog timer. Note currently this timer is used
// for timing listening periods after sending status packet,
// so not an optional feature if low power mode required.
#define FEATURE_WATCHDOG_TIMER

// Measure battery V using comparator with Vref and Vcc through voltage ladder
#define FEATURE_VBAT

// GPS on second UART port.
//#define FEATURE_GPS_ON_USART1

// WS2812B RGB LED driver
//#define FEATURE_WS2812B

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
#define EVENT_COUNTER_PIN 17
#endif

#ifdef BOARD_LPC812_V1
// RFM69 PCB v1
#define SCK_PIN 7
#define SS_PIN 1
#define MOSI_PIN 9
#define MISO_PIN 8
#define RESET_PIN 15
#define DIO0_PIN 6
#define EVENT_COUNTER_PIN 16
#define LED_PIN 17
#define UART_RXD_PIN 0
#define DS18B20_PIN 14
#define WS2812B_PIN 14
#endif

#ifdef BOARD_LPC812_RFM98_V1
// RFM98 PCB v1
#define SCK_PIN 9
#define SS_PIN 1
#define MOSI_PIN 8
#define MISO_PIN 7
#define RESET_PIN 15
#define DIO0_PIN 6
#define DIO1_PIN 14
//#define EVENT_COUNTER_PIN 16
#define LED_PIN 17
#define UART_RXD_PIN 0
#define DS18B20_PIN 14
//#define WS2812B_PIN 14
#endif


// Enable reroute of RXD to alternative pin due to failure to fix via bug.
#ifdef BOARD_V1B_HACK
#define BOARD_V1B_HACK_MCU_ID 0x5034039
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
//#define UART_BPS (9600)
#define UART_BPS (115200)

// Max size of NMEA sentence: standard specifies 79 + '$' + CRLF
#define GPS_NMEA_SIZE 84

// Buffer size for RFM9x 128; for RFM96 66 bytes
#ifdef RADIO_RFM9x
#define RXTX_BUFFER_SIZE 128
#else
#define RXTX_BUFFER_SIZE 66
#endif

// Other common include files that all C files need
#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <stdint.h>
#include <string.h>
#include "rfm.h"
#include "printf.h"
#include "debug.h"
#include "spi.h"
#include "err.h"
#include "delay.h"
#include "battery.h"
#ifdef RADIO_RFM9x
#include "rfm98.h"
#else
#include "rfm69.h"
#endif


#endif // end CONFIG_H_
