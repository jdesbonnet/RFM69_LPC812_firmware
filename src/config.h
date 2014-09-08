#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION "WRSC_RFM69_Controller 0.1.0"

// Version of MCU used (LPC812, LPC810 supported)
#define LPC812
//#define LPC810

// Pins used for SPI
#define SCK_PIN 15
#define SS_PIN 9
#define MOSI_PIN 8
#define MISO_PIN 7

// UART speed ('baud rate')
#define UART_BPS (115200)


//#define USE_SYSTICK


#endif
