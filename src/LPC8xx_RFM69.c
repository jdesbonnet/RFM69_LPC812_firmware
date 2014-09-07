/*
===============================================================================
 Name        : LPC8xx_RFM69.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include "lpc8xx_spi.h"
//#include "lpc8xx_uart.h"
#include "myuart.h"
#include "print_util.h"

#define LPC812

#ifdef LPC812
void SwitchMatrix_Init()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;

    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;
}
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

int main(void) {

	//while (1) ;

	SwitchMatrix_Init();

	GPIOInit();

/*
	GPIOSetDir(0,15, 1);
	GPIOSetDir(0,9, 1);
	GPIOSetDir(0,8, 1);
	GPIOSetDir(0,7, 1);

	while (1) {
		GPIOSetBitValue(0,15,0);
		GPIOSetBitValue(0,9,0);
		GPIOSetBitValue(0,8,0);
		GPIOSetBitValue(0,7,0);

		GPIOSetBitValue(0,9,1);
		GPIOSetBitValue(0,15,1);
		GPIOSetBitValue(0,8,1);
		GPIOSetBitValue(0,7,1);
	}
*/
	spi_init();

	rfm69_init();
	rfm69_config();

	MyUARTInit(LPC_USART0, 9600);
	//LPC_USART0->INTENSET = 0x01;	/* Enable UART interrupt */
	MyUARTSendStringZ (LPC_USART0, (uint8_t*)"WRSC2014 RFM69\r\n");

#ifdef LPC810
#define PIN 4
#endif
#ifdef LPC812
#define PIN 15
#endif

	// Set as output
	//GPIOSetDir(0,PIN,1);

	int i = 0;
	while (1) {
		//GPIOSetBitValue(0,PIN,1);
		rfm69_register_read(0x01);

		print_hex(LPC_USART0,i++);
		MyUARTSendStringZ (LPC_USART0, (uint8_t*)"test\r\n");

		//GPIOSetBitValue(0,PIN,0);
	}

}
