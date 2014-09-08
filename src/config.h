#ifndef CONFIG_H_
#define CONFIG_H_

#define VERSION "WRSC_RFM69_Controller 0.1.0"

// Version of MCU used (LPC812, LPC810 supported)
#define LPC812
//#define LPC810


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
#define MOSI_PIN 2 // package pin 4
#define MISO_PIN 7 // package pin 2
#endif

#ifdef LPC810
void SwitchMatrix_Init()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0100UL;

    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;
}
#endif


// UART speed ('baud rate')
#define UART_BPS (115200)


//#define USE_SYSTICK


#endif
