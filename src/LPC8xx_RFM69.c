/*
===============================================================================
 Name        : LPC8xx_RFM69.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#define LPC812
//#define LPC810

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include "lpc8xx_spi.h"
//#include "lpc8xx_uart.h"
#include "myuart.h"
#include "print_util.h"

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

	spi_init();

	rfm69_init();
	rfm69_config();

	MyUARTInit(LPC_USART0, 115200);

	int i;
	//for (i = 0; i < 10; i++) {
	//	MyUARTSendStringZ (LPC_USART0, (uint8_t*)"WRSC2014 RFM69\r\n");
	//}

	uint8_t *rxbuf = MyUARTGetBuf();

	uint8_t *args[8];
	int argc;

	while (1) {

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

			rxbuf = MyUARTGetBuf();

			// Echo command
			MyUARTSendStringZ(LPC_USART0,"[");
			MyUARTSendStringZ(LPC_USART0,rxbuf);
			MyUARTSendStringZ(LPC_USART0,"]\r\n");

			// Parse command line
			argc = 1;
			args[0] = rxbuf;
			while (*rxbuf != 0) {
				if (*rxbuf == ' ') {
					*rxbuf = 0;
					args[argc++] = rxbuf+1;
				}
				rxbuf++;
			}

			MyUARTSendStringZ(LPC_USART0,"argc=");
			MyUARTPrintDecimal(LPC_USART0, argc);
			MyUARTSendStringZ(LPC_USART0,"\r\n");
			for (i = 0; i < argc; i++) {
				MyUARTSendStringZ(LPC_USART0,"arg: ");
				MyUARTSendStringZ(LPC_USART0,args[i]);
				MyUARTSendStringZ(LPC_USART0,"\r\n");
			}

			MyUARTBufReset();


		}
		__WFI();

/*
		// Read register file in single mode
		for (i = 1; i < 0x6f; i++) {
			MyUARTSendStringZ(LPC_USART0,"reg[");
			print_hex8(LPC_USART0,i);
			MyUARTSendStringZ (LPC_USART0,"]=");
			print_hex8(LPC_USART0,rfm69_register_read(i));
			MyUARTSendStringZ (LPC_USART0,"\r\n");
		}
*/

	}

}
