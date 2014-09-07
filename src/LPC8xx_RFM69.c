/*
===============================================================================
 Name        : LPC8xx_RFM69.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#define VERSION "WRSC_RFM69_Controller 0.1.0"

#define LPC812
//#define LPC810

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>

#include "lpc8xx_spi.h"
//#include "lpc8xx_uart.h"
#include "myuart.h"
#include "print_util.h"
#include "rfm69.h"

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

	MyUARTInit(LPC_USART0, 115200);


	spi_init();

	// Configure hardware interface to radio modile
	rfm69_init();

	// Configure registers for this application
	rfm69_config();


	int i;

	uint8_t boat_addr=0x63;
	uint8_t current_lat[16], current_lon[16];
	uint8_t rssi;

	uint8_t *rxbuf;
	uint8_t *args[8];

	uint8_t frxbuf[66];
	uint8_t frame_len;

	int argc;

	rfm69_register_write(RFM69_OPMODE,
			RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX)
			);


	while (1) {


		//print_hex8(LPC_USART0,rfm69_register_read(RFM69_IRQFLAGS1));
		//print_hex8(LPC_USART0,rfm69_register_read(RFM69_IRQFLAGS2));
		//MyUARTSendStringZ(LPC_USART0,"\r\n");

		// Check for packet
		if (rfm69_payload_ready()) {
			MyUARTSendStringZ(LPC_USART0, "p ");
			frame_len = rfm69_frame_rx(frxbuf,66,&rssi);
			int i;
			for (i = 0; i < frame_len; i++) {
				print_hex8(LPC_USART0,frxbuf[i]);
			}
			MyUARTSendStringZ(LPC_USART0,"\r\n");
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

			rxbuf = MyUARTGetBuf();

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

			switch (*args[0]) {

			case 'C' :
			{
				rfm69_config();
				break;
			}
			case 'I' :
			{
				// Parse args[1]
				boat_addr = 0x66;
			}
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_lat,args[1],strlen(args[1])+1);
				memcpy(current_lon,args[2],strlen(args[2])+1);
				break;
			}
			case 'R' : {
				// Parameter is register address
				uint8_t *b;
				int regAddr = parse_dec(args[1],&b);
				print_hex8 (LPC_USART0, rfm69_register_read(regAddr));
				MyUARTSendStringZ(LPC_USART0,"\r\n");
				break;
			}
			case 'T' : {
				MyUARTSendStringZ(LPC_USART0,"T command ");
				MyUARTSendStringZ(LPC_USART0,current_lat);
				MyUARTSendStringZ(LPC_USART0," ");
				MyUARTSendStringZ(LPC_USART0,current_lon);
				MyUARTSendStringZ(LPC_USART0,"\r\n");

				uint8_t buf[] = {0x11, 0x23, 0x37, 0x55, 0x55, 0x55};
				rfm69_frame_tx (buf,6);
				break;
			}
			case 'V' : {
				MyUARTSendStringZ(LPC_USART0,VERSION);
				break;
			}
			case 'W' : {
				// Parameter is register address
				uint8_t *b;
				int regAddr = parse_dec(args[1],&b);
				int regValue = parse_dec(args[2],&b);
				rfm69_register_write(regAddr,regValue);
				MyUARTSendStringZ(LPC_USART0,"OK\r\n");
				break;
			}

			}

			MyUARTBufReset();


		}
		//__WFI();


	}

}
