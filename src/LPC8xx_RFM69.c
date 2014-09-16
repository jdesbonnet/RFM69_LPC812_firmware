/*
===============================================================================
 Name        : LPC8xx_RFM69.c
 Author      : Joe Desbonnet, jdesbonnet@gmail.com
 Version     :
 Copyright   : BSD licence. TODO: add licence to header.
 Description : TODO
===============================================================================

TODO:
For v0.2.0:
* Move to 4 byte header. Having an extra byte for special flags (like ack at MAC level).
* Also use struct: might enable greater code density.
*
* Change radio level remote register read/write with a remote command execution: ie take
* bytes from radio packet and inject into UART receive buffer and trigger a command, as if
* arriving from UART. To enable responses etc, have command to forward UART command responses
* and errors via radio to one or more nodes. This can then replace the following features:
* o Packet Forward (just exec a T command remotely)
* o Register Read/Write
* o Heartbeat interval set
*
*/


#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>

#include "config.h"
#include "myuart.h"
#include "sleep.h"
#include "print_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"
#include "flags.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)
//volatile uint32_t TimeTick = 0;

// Address of this node
int8_t node_addr = DEFAULT_NODE_ADDR;

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Various radio controller flags (done as one 32 bit register so as to
// reduce code size and SRAM requirements).
uint32_t flags =
		FLAG_RADIO_MODULE_ON
		| FLAG_HEARTBEAT_ENABLE;


#ifdef FEATURE_HEARTBEAT
uint32_t heartbeat_interval = 0x1FFFF;
#endif

void loopDelay(uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}

#ifdef LPC812
/**
 * UART RXD on SOIC package pin 19
 * UART TXD on SOIC package pin 5
 */
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
/**
 * On boot initialize LPC810 SwitchMatrix preserving SWD functionality. For SPI operation
 * we need to forfeit SWCLK, SWDIO and RESET functions due to lack of available pins.
 * Instead delay configuring these pins for SPI either by
 * switching pins to SPI by UART command or by delay so as to provide opportunity
 * to reprogram the device using SWD (else will have to use awkward ISP entry via
 * powercycling to reprogram the device).
 */

void SwitchMatrix_NoSpi_Init_old()
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
// reset disabled
void SwitchMatrix_NoSpi_Init()
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
    LPC_SWM->PINENABLE0 = 0xfffffff3UL;
}


/**
 *
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
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;

    /* Pin Assign 1 bit Configuration */
    LPC_SWM->PINENABLE0 = 0xffffffffUL;
}


#endif


#ifdef FEATURE_MCU_UID
/**
 * Retrieve MCU unique ID
 */
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
#endif

/**
 * Print error code 'code' while executing command 'cmd' to UART.
 */
void report_error (uint8_t cmd, int32_t code) {
	if (code<0) code = -code;
	MyUARTSendStringZ(LPC_USART0,"e ");
	MyUARTSendByte(LPC_USART0,cmd);
	MyUARTSendStringZ(LPC_USART0," ");
	MyUARTPrintHex(LPC_USART0,code);
	MyUARTSendCRLF(LPC_USART0);
}

#ifdef FEATURE_LED
/**
 * Blink diagnostic LED. Optional feature (edit config.h to define hardware configuration).
 */
void ledBlink () {
	int i;
	for (i = 0; i < 3; i++)	{
			GPIOSetBitValue(0,LED_PIN,1);
			loopDelay(200000);
			GPIOSetBitValue(0,LED_PIN,0);
			loopDelay(200000);
	}
}
#endif


int main(void) {

	/*
	 * LPC8xx features a SwitchMatrix which allows most functions to be mapped to most pins.
	 * This setups up the pins in a way that's convenient for our physical circuit layout.
	 * In the case of the LPC810, use of SPI (which necessitates loss of SWD functionality)
	 * is delayed to allow opportunity to reprogram device via SWD.
	 */

#ifdef LPC810
	SwitchMatrix_NoSpi_Init();
#elif LPC812
	SwitchMatrix_Init();
#endif

	//GPIOInit();
	MyUARTInit(LPC_USART0, UART_BPS);

	// Display firmware version on boot
	cmd_version(1,NULL);


#ifdef LPC810
	// Delay to allow debug probe to reflash
	loopDelay(20000000);

	// Won't be able to use debug probe from this point on (unless UART S 0 command used)
	SwitchMatrix_Spi_Init();

	// Hack:
	// Display firmware version again to indicate SPI is in operation
	cmd_version(1,NULL);
#endif


	spi_init();

	// Configure hardware interface to radio module
	rfm69_init();

	int i;

	uint8_t rssi;

	uint8_t *cmdbuf;
	uint8_t *args[8];

	// Radio frame receive buffer
	uint8_t frxbuf[66];
	uint8_t frame_len;

	// Acts as a crude clock
	uint32_t loop_counter = 0;
	uint32_t last_frame_time = 0;

	int argc;



	// Optional Diagnostic LED. Configure pin for output and blink 3 times.
#ifdef FEATURE_LED
	GPIOSetDir(0,LED_PIN,1);
	ledBlink();
#endif

	// Configure RFM69 registers for this application. I found that it was necessary
	// to delay a short period after powerup before configuring registers.
	loopDelay(200000);
	rfm69_config();

	// Main program loop
	while (1) {

		loop_counter++;

#ifdef FEATURE_SLEEP
		// If the radio is off then sleep until next interrupt (probably from UART)
		if ( ! (flags & FLAG_RADIO_MODULE_ON)) {

			// Experiment to allow wakup on async UART (temp switch RXD for SCLK
			// and config UART for SYNC slave)

			LPC_USART0->CFG |= (1<<11); // SYNCEN for wakeup
		    //LPC_SWM->PINASSIGN0 = 0xffffff04UL;
		    //LPC_SWM->PINASSIGN1 = 0xffffff00UL;
		    //LPC_SWM->PINENABLE0 = 0xffffffffUL;

			// Try GPIO on RXD instead
			LPC_SWM->PINASSIGN0 = 0xffffff04UL;
			LPC_SWM->PINENABLE0 = 0xffffffffUL;


			// Kinda works: but no UART during sleep making exiting sleep difficult
			prepareForPowerDown();
			LPC_WKT->COUNT = 10000;
			LPC_WKT->CTRL = 1;
			__WFI();
			loopDelay(20000);


			LPC_USART0->CFG &= ~(1<<11); // SYNCEN disable for normal UART
			SwitchMatrix_Spi_Init(); // Normal pin configuration again


			MyUARTSendStringZ(LPC_USART0,"Wake!\r\n");
			MyUARTInit(LPC_USART0, UART_BPS);
			loopDelay(200000); // Small window of time in which UART works
			//__WFI(); // regular sleep
		}
#endif

#ifdef FEATURE_HEARTBEAT
		// Send heartbeat signal every 10s or so
		if ( (flags&FLAG_RADIO_MODULE_ON) && (flags&FLAG_HEARTBEAT_ENABLE) && (loop_counter % heartbeat_interval) == 0) {
			uint8_t payload[3];
			payload[0] = 0xff;
			payload[1] = node_addr;
			payload[2] = 'h';
			rfm69_frame_tx(payload,3);
			MyUARTSendStringZ(LPC_USART0,"h\r\n");
		}
#endif

#ifdef FEATURE_LINK_LOSS_RESET
		if ( (loop_counter - last_frame_time) > 0x8FFFF) {
			report_error('$',E_LINK_LOSS_RESET);
			loopDelay(200000);
			NVIC_SystemReset();
		}
#endif

		// Check for received packet on RFM69
		if ( (flags&FLAG_RADIO_MODULE_ON) && rfm69_payload_ready()) {

			// Yes, frame ready to be read from FIFO
			frame_len = rfm69_frame_rx(frxbuf,66,&rssi);

			// TODO: tidy this
			// SPI error
			if (frame_len>0) {

				// Mark time of last incoming good frame
				last_frame_time = loop_counter;

				// All frames have a common header
				// 8 bit to address
				// 8 bit from address
				// 8 bit message type
				uint8_t to_addr = frxbuf[0];
				uint8_t from_addr = frxbuf[1];
				uint8_t msgType = frxbuf[2];

			// 0xff is the broadcast address
			if ( (flags&FLAG_PROMISCUOUS_MODE) || to_addr == 0xff || to_addr == node_addr) {

				// This frame is for us! Examine messageType field for appropriate action.

				switch (msgType) {

#ifdef FEATURE_REMOTE_PKT_TX
				// Experimental remote packet transmit / relay
				case 'B' : {
					int payload_len = frame_len - 3;
					uint8_t payload[payload_len];
					memcpy(payload,frxbuf+3,payload_len);
					rfm69_frame_tx(payload, payload_len);
					break;
				}
#endif

#ifdef FEATURE_HEARTBEAT
				case 'H' : {
					heartbeat_interval = frxbuf[3]<<16;
					MyUARTSendStringZ(LPC_USART0,"h ");
					MyUARTPrintHex(heartbeat_interval);
					MyUARTSendCRLF(LPC_USART0);
					break;
				}
#endif

				// Message requesting position report. This will return the string
				// set by the UART 'L' command verbatim.
				case 'R' :
				{
					int loc_len = strlen(current_loc);

#ifdef FEATURE_DEBUG
					MyUARTSendStringZ(LPC_USART0,"i Sending loc to ");
					MyUARTSendStringZ(LPC_USART0," ");
					MyUARTPrintHex(LPC_USART0,node_addr);
					MyUARTSendStringZ(LPC_USART0," len=");
					MyUARTPrintHex(LPC_USART0, loc_len);
#endif

					// report position
					int payload_len = loc_len + 3;
					uint8_t payload[payload_len];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'r';
					memcpy(payload+3,current_loc,loc_len);

#ifdef FEATURE_DEBUG
					MyUARTSendStringZ(LPC_USART0,current_loc);
					MyUARTSendCRLF(LPC_USART0);
#endif

					rfm69_frame_tx(payload, payload_len);
					break;
				}
#ifdef FEATURE_REMOTE_REG_READ
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
#endif

#ifdef FEATURE_REMOTE_REG_WRITE
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


#endif

#ifdef FEATURE_REMOTE_COMMAND
				case 'Z' : {
					// If there is an uncompleted UART command in buffer then
					// remote command takes priority and whatever is in the
					// command buffer is dropped. Output an error to indicate
					// this has happened.
					// TODO: should we disable UART interrupt for this block?
					// because incoming UART char between now and cmd parse could
					// cause corruption of cmd buffer.
					if (MyUARTGetBufIndex()>0) {
						MyUARTBufReset();
						report_error('Z',E_CMD_DROPPED);
					}
					MyUARTSendStringZ(LPC_USART0, "z ");
					int payload_len = frame_len - 3;
					memcpy(cmdbuf,frxbuf+3,payload_len);
					cmdbuf[payload_len] = 0; // zero terminate buffer
					MyUARTSendStringZ(LPC_USART0,cmdbuf);
					MyUARTSendCRLF(LPC_USART0);
					MyUARTSetBufFlags(UART_BUF_FLAG_EOL);
					break;
				}
#endif

#ifdef FEATURE_LED
				// Remote LED blink
				case 'U' : {
					uint8_t payload[2];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'u';
					rfm69_frame_tx(payload, 3);
					ledBlink();
					break;
				}
#endif


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

#ifdef FEATURE_DEBUG
				MyUARTSendStringZ(LPC_USART0,"i Ignoring packet from ");
				MyUARTPrintHex(LPC_USART0,to_addr);
				MyUARTSendCRLF(LPC_USART0);
#endif

			}


			} // end frame len valid check
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

			MyUARTSendCRLF(LPC_USART0);


			cmdbuf = MyUARTGetBuf();

			// Parse command line
			argc = 1;
			args[0] = cmdbuf;
			while (*cmdbuf != 0) {
				if (*cmdbuf == ' ') {
					*cmdbuf = 0;
					args[argc++] = cmdbuf+1;
				}
				cmdbuf++;
			}

			// TODO: using an array of functions may be more space efficient than
			// switch statement.

			switch (*args[0]) {

#ifdef FEATURE_UART_SPEED
			case 'B' : {
				cmd_set_uart_speed (argc, args);
				break;
			}
#endif

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
#ifdef FEATURE_MCU_UID
			case 'I' : {
				MyUARTSendStringZ(LPC_USART0,"u ");
				MyUARTPrintHex(LPC_USART0,get_mcu_serial_number());
				MyUARTSendCRLF(LPC_USART0);
				break;
			}
#endif

			// Set current location
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_loc,args[1],strlen(args[1])+1);
				break;
			}

#ifdef FEATURE_NMEA_INPUT
			// NMEA (only interested in $GPGLL)
			case '$' : {
				// +1 on len to include zero terminator
				if (strlen(args[0]) && args[0][4]=='L' && args[0][5]=='L') {
					memcpy(current_loc,args[0],strlen(args[0])+1);
				}
				break;
			}
#endif

#ifdef FEATURE_SLEEP
			case 'M' : {
				if (args[1][0]=='0') {
					flags &= ~FLAG_RADIO_MODULE_ON;
					rfm69_mode(RFM69_OPMODE_Mode_SLEEP);
					spi_off();
				} else if (args[1][0]=='1') {
					spi_init();
					flags |= FLAG_RADIO_MODULE_ON;
					rfm69_mode(RFM69_OPMODE_Mode_RX);
				}
			}
#endif

			// Set node address
			case 'N' :
			{
				cmd_set_node_addr(argc, args);
				break;
			}

			// Experimental reset
			case 'Q' : {
				loopDelay(200000);
				NVIC_SystemReset();
			}


			// Read RFM69 register
			case 'R' : {
				// Parameter is register address
				uint8_t *b;
				int regAddr = parse_hex(args[1],&b);
				MyUARTSendStringZ(LPC_USART0,"r ");
				print_hex8 (LPC_USART0, regAddr);
				MyUARTSendStringZ(LPC_USART0," ");
				print_hex8 (LPC_USART0, rfm69_register_read(regAddr));
				MyUARTSendCRLF(LPC_USART0);
				break;
			}



#ifdef LPC810
			// SPI pin initialize (delayed to keep SWD on bootup)
			case 'S' : {

				if (args[1][0]=='1') {
					// Note will disconnect SWD
					SwitchMatrix_Spi_Init();
					spi_init();
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
				/*
				rfm69_register_write(RFM69_OPMODE,
						RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX)
						);
				*/
				rfm69_mode(RFM69_OPMODE_Mode_RX);
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

			// Reset UART command line buffer ready for next command
			MyUARTBufReset();


		}

		// TODO: we could save some MCU power by delaying and entering sleep
		// state before polling RFM69 for reception of another packet. An
		// possible alternative approach is to configure RFM69 digital IO
		// pins to trigger interrupt on MCU on packet reception. However in the
		// case of LPC810 there are no available pins for this (unless we try
		// something clever by piggybacking this on another line. Probably not
		// worth the effort as this MCU current requirements are modest.
		//__WFI();


	}

}
