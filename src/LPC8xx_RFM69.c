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

#include <string.h>

#include "config.h"
#include "myuart.h"
#include "print_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"
#include "flags.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)
volatile uint32_t TimeTick = 0;

// Address of this node
int8_t node_addr = 0x7e;

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Various radio controller flags (done as one 32 bit register so as to
// reduce code size and SRAM requirements).
uint32_t flags = FLAG_RADIO_MODULE_ON;


void loopDelay(uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}

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
void SwitchMatrix_NoSpi_Init()
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


#ifdef LPC810
/**
 * TXD PIO0_0 (package pin 8)
 * RXD PIO0_1 (package pin 5)
 * package pins 1 - 4 as PIO0_5, PIO0_4, PIO0_3, PIO0_2
 *
 * Note: this configuration disables RESET and SWD.
 * To re-flash will need to access ISP
 * by holding PIO0_? low and cycling power.
 */
void SwitchMatrix_Spi_Init()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0100UL;

    /* Pin Assign 1 bit Configuration */
    LPC_SWM->PINENABLE0 = 0xffffffffUL;
}
#endif

#define IAP_LOCATION 0x1fff1ff1
typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP iap_entry = (IAP)0x1fff1ff1;

uint32_t get_mcu_serial_number () {
	unsigned long command[5];
	unsigned long result[4];
	command[0] = 58; // Read UID
	iap_entry (command, result);
	return (uint32_t)result[1];
}

void report_error (uint8_t cmd, int32_t code) {
	if (code<0) code = -code;
	MyUARTSendStringZ(LPC_USART0,"e ");
	MyUARTSendByte(LPC_USART0,cmd);
	MyUARTSendStringZ(LPC_USART0," ");
	MyUARTPrintHex(LPC_USART0,code);
	MyUARTSendCRLF(LPC_USART0);
}

int main(void) {

#ifdef LPC810
	SwitchMatrix_NoSpi_Init();
#endif
#ifdef LPC812
	SwitchMatrix_Init();
#endif

	GPIOInit();

	MyUARTInit(LPC_USART0, UART_BPS);


//#if !defined (LPC810_NOSPI)
	spi_init();
//#endif

	// Configure hardware interface to radio module
	rfm69_init();

	// Configure RFM69 registers for this application
	rfm69_config();


	int i;


	uint8_t rssi;

	uint8_t *rxbuf;
	uint8_t *args[8];

	uint8_t frxbuf[66];
	uint8_t frame_len;

	int argc;

	// Display firmware version on boot
	cmd_version(1,NULL);

	/*
	rfm69_register_write(RFM69_OPMODE,
			RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX)
			);
	*/

	// Optional Diagnostic LED. Configure pin for output and blink 3 times.
#ifdef FEATURE_LED
	GPIOSetDir(0,LED_PIN,1);
	for (i = 0; i < 3; i++)	{
		GPIOSetBitValue(0,LED_PIN,1);
		loopDelay(200000);
		GPIOSetBitValue(0,LED_PIN,0);
		loopDelay(200000);
	}
#endif


	while (1) {

		// Check for received packet
		if ( (flags&FLAG_RADIO_MODULE_ON) && rfm69_payload_ready()) {
			frame_len = rfm69_frame_rx(frxbuf,66,&rssi);

			// SPI error
			if (frame_len>0) {



			// All fames will have these:
			// First byte is to_node address
			// Second byte is from node_addess (0 = master controller, 0xff = broadcast)
			// Third is message type

			uint8_t to_addr = frxbuf[0];
			uint8_t from_addr = frxbuf[1];
			uint8_t msgType = frxbuf[2];

			// 0xff is broadcast
			if ( (flags&FLAG_PROMISCUOUS_MODE) || to_addr == 0xff || to_addr == node_addr) {

				// This is for us!

				switch (msgType) {

				// Message requesting position report. This will return the string
				// set by the UART 'L' command verbatim.
				case 'R' :
				{
					MyUARTSendStringZ(LPC_USART0,"i Sending loc to ");
					MyUARTSendStringZ(LPC_USART0," ");
					MyUARTPrintHex(LPC_USART0,node_addr);
					MyUARTSendStringZ(LPC_USART0," len=");
					int loc_len = strlen(current_loc);
					MyUARTPrintHex(LPC_USART0, loc_len);
					// report position
					int payload_len = loc_len + 3;
					uint8_t payload[payload_len];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'r';
					memcpy(payload+3,current_loc,loc_len);

					//loopDelay(5000000);

					MyUARTSendStringZ(LPC_USART0,current_loc);
					MyUARTSendCRLF(LPC_USART0);


					rfm69_frame_tx(payload, payload_len);
					break;
				}
				// Remote register read
				case 'X' : {
					uint8_t base_addr = frxbuf[3];
					uint8_t read_len = frxbuf[4];
					if (read_len>16) read_len = 16;
					uint8_t payload[read_len+4];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'x';
					payload[3] = base_addr;
					for (i = 0; i < read_len; i++) {
						payload[i+4] = rfm69_register_read(base_addr+i);
					}
					rfm69_frame_tx(payload, read_len+4);
					break;
				}

				// Remote register write
				case 'Y' : {
					uint8_t base_addr = frxbuf[3];
					uint8_t write_len = frame_len - 4;
					if (write_len > 16) write_len = 16;
					int i;
					for (i = 0; i < write_len; i++) {
						rfm69_register_write(base_addr+i,frxbuf[i+4]);
					}
					uint8_t payload[2];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'y';
					rfm69_frame_tx(payload, 3);
					break;
				}

				// If none of the above cases match, output packet to UART
				default: {

					MyUARTSendStringZ(LPC_USART0, "p ");

					print_hex8(LPC_USART0,frxbuf[0]);
					MyUARTSendStringZ(LPC_USART0, " ");
					print_hex8(LPC_USART0,frxbuf[1]);
					MyUARTSendStringZ(LPC_USART0, " ");

					int i;
					for (i = 2; i < frame_len; i++) {
						print_hex8(LPC_USART0,frxbuf[i]);
					}
					MyUARTSendStringZ (LPC_USART0," ");
					print_hex8(LPC_USART0, rssi);
					MyUARTSendCRLF(LPC_USART0);
				}
				}

			} else {
				MyUARTSendStringZ(LPC_USART0,"i Ignoring packet from ");
				MyUARTPrintHex(LPC_USART0,to_addr);
				MyUARTSendCRLF(LPC_USART0);

			}


			} // end frame len valid check
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

			MyUARTSendCRLF(LPC_USART0);


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

			// TODO: using an array of functions may be more space efficient than
			// switch statement.

			switch (*args[0]) {

			// Reset RFM69 with default configuration
			case 'C' :
			{
				rfm69_config();
				break;
			}

			// Read set various flags
			case 'F' : {
				cmd_flags(argc, args);
				break;
			}

			// Display MCU unique ID
			case 'I' : {
				MyUARTSendStringZ(LPC_USART0,"u ");
				MyUARTPrintHex(LPC_USART0,get_mcu_serial_number());
				MyUARTSendCRLF(LPC_USART0);
				break;
			}

			// Set current location
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_loc,args[1],strlen(args[1])+1);
				break;
			}

			// Set node address
			case 'N' :
			{
				cmd_set_node_addr(argc, args);
				break;
			}


			// Read RFM69 register
			case 'R' : {
				// Parameter is register address
				uint8_t *b;
				int regAddr = parse_dec(args[1],&b);
				MyUARTSendStringZ(LPC_USART0,"r ");
				print_hex8 (LPC_USART0, regAddr);
				MyUARTSendStringZ(LPC_USART0," ");
				print_hex8 (LPC_USART0, rfm69_register_read(regAddr));
				MyUARTSendCRLF(LPC_USART0);
				break;
			}



#ifdef LPC810
			case 'S' : {
				if (args[1][0]=='1') {
					// Note will disconnect SWD
					SwitchMatrix_Spi_Init();
					spi_init();
/*
					while (1) {
						GPIOSetBitValue(0,5,0);
						GPIOSetBitValue(0,5,1);
						GPIOSetBitValue(0,4,0);
						GPIOSetBitValue(0,4,1);
						GPIOSetBitValue(0,3,0);
						GPIOSetBitValue(0,3,1);
						GPIOSetBitValue(0,2,0);
						GPIOSetBitValue(0,2,1);
					}
*/
				} else {
					SwitchMatrix_NoSpi_Init();
				}
				break;
			}
#endif

			// Transmit arbitrary packet
			case 'T' : {
				int status = cmd_packet_transmit(argc, args);
				if ( status ) {
					report_error('T', status);
				}
				// Back to RX mode
				rfm69_register_write(RFM69_OPMODE,
						RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX)
						);
				break;
			}

#ifdef FEATURE_LED
			// Turn LED on/off
			case 'U' : {
				int i = parse_hex(args[1]);
				GPIOSetBitValue(0,LED_PIN,i);
				break;
			}
#endif

			// RFM69 controller (this) firmware
			case 'V' : {
				cmd_version(1,NULL);
				break;
			}

			// Write RFM69 register
			case 'W' : {
				// Parameter is register address
				//uint8_t *b;
				int regAddr = parse_hex(args[1]);
				int regValue = parse_hex(args[2]);
				rfm69_register_write(regAddr,regValue);
				break;
			}
			default : {
				report_error(*args[0], E_INVALID_CMD);
			}

			}

			MyUARTBufReset();


		}
		//__WFI();


	}

}
