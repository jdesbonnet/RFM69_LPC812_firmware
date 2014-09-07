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
//#define USE_SYSTICK

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
#include "cmd.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)
volatile uint32_t TimeTick = 0;


int8_t node_addr = 0x7e;
uint8_t promiscuous_mode = 0;
//uint8_t current_lat[16], current_lon[16];
uint8_t current_loc[32];

#ifdef LPC812



void loopDelay(uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}


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

	SwitchMatrix_Init();

	GPIOInit();

	MyUARTInit(LPC_USART0, 115200);


	spi_init();

	// Configure hardware interface to radio modile
	rfm69_init();

	// Configure registers for this application
	rfm69_config();


	int i;


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

		// Check for received packet
		if (rfm69_payload_ready()) {
			frame_len = rfm69_frame_rx(frxbuf,66,&rssi);

			/*
			int i;
			for (i = 0; i < frame_len; i++) {
				print_hex8(LPC_USART0,frxbuf[i]);
			}
			MyUARTSendStringZ(LPC_USART0,"\r\n");
			*/

			// First byte is node address
			uint8_t addr = frxbuf[0];
			if (addr == 0xff || addr == node_addr) {

				// This is for us!
				uint8_t msgType = frxbuf[1];
				switch (msgType) {
				// Message requesting position report
				case 0x23 :
				{
					MyUARTSendStringZ(LPC_USART0,"Sending loc: ");

					// report position
					int payload_len = strlen(current_loc) + 2;
					uint8_t payload[payload_len];
					payload[0] = node_addr;
					payload[1] = 'r';
					memcpy(payload+2,current_loc,payload_len-2);

					loopDelay(5000000);

					MyUARTSendStringZ(LPC_USART0,current_loc);
					MyUARTSendStringZ(LPC_USART0,"\r\n");

					rfm69_frame_tx(payload, payload_len);
				}
				// Remote register read
				case 'X' : {
					uint8_t base_addr = frxbuf[2];
					uint8_t read_len = frxbuf[3];
					uint8_t payload[read_len+3];
					payload[0] = node_addr;
					payload[1] = 'x';
					payload[2] = base_addr;
					for (i = base_addr; i < (base_addr+read_len); i++) {
						payload[i+3] = rfm69_register_read(i);
					}
					rfm69_frame_tx(payload, read_len+3);
				}
				// Remote register write
				case 'Y' : {
					uint8_t base_addr = frxbuf[2];
					uint8_t write_len = frxbuf[3];
					int i;
					for (i = base_addr; i < (base_addr+write_len); i++) {
						rfm69_register_write(i,frxbuf[i+4]);
					}
					uint8_t payload[2];
					payload[0] = node_addr;
					payload[1] = 'y';
					rfm69_frame_tx(payload, 2);
				}
				default: {

					MyUARTSendStringZ(LPC_USART0, "p ");

					int i;
					for (i = 0; i < frame_len; i++) {
						print_hex8(LPC_USART0,frxbuf[i]);
					}
					MyUARTSendStringZ(LPC_USART0,"\r\n");
				}
				}

			} else {
				MyUARTSendStringZ(LPC_USART0,"Ignoring packet from ");
				MyUARTPrintHex(LPC_USART0,addr);
				MyUARTSendStringZ(LPC_USART0,"\r\n");
			}
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
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_loc,args[1],strlen(args[1])+1);
				break;
			}
			case 'N' :
			{
				cmd_set_node_addr(argc, args);
				break;
			}
			case 'P' : {
				cmd_promiscuous_mode(argc, args);
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
			// Transmit arbitrary packet
			case 'T' : {
				cmd_packet_transmit(argc, args);
				// Back to RX mode
				rfm69_register_write(RFM69_OPMODE,
						RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX)
						);
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
