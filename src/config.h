#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION "WRSC_RFM69_Controller 0.1.4"

// Version of MCU used (LPC812, LPC810 supported)
//#define LPC812

// LPC810 using all available pins for UART, SPI
#define LPC810
//#define LPC812


#ifdef LPC812
#define FEATURE_LED
#define LED_PIN 14
#endif


//
// Pins used for SPI
//
#ifdef LPC812
#define SCK_PIN 15
#define SS_PIN 9
#define MOSI_PIN 8
#define MISO_PIN 7
#endif

#ifdef LPC810
#define SCK_PIN 3  // package pin 3
#define SS_PIN 5   // package pin 1
#define MOSI_PIN 4 // package pin 4
#define MISO_PIN 2 // package pin 2
#endif




// UART speed ('baud rate')
#define UART_BPS (9600)
//#define UART_BPS (115200)


//#define USE_SYSTICK

#define FEATURE_SPI_FEEDBACK_TEST

#endif
