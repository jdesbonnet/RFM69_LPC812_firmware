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

#include "lpc8xx_pmu.h"


#define SYSTICK_DELAY		(SystemCoreClock/100)

// Address of this node
int8_t node_addr = DEFAULT_NODE_ADDR;

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Various radio controller flags (done as one 32 bit register so as to
// reduce code size and SRAM requirements).
uint32_t flags = MODE_LOW_POWER_POLL
		| (0x4<<8) // poll interval 500ms x 2^(3+1) = 8s
		;

void loopDelay(uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}
void wktDelay(uint32_t i) {
	LPC_WKT->COUNT = i;
	__WFI();
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
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

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
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

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
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

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
 * @param cmd  Command that generated the error
 * @param code Error code (ref err.h)
 */
void report_error (uint8_t cmd, int32_t code) {
	if (code<0) code = -code;
	MyUARTSendStringZ((uint8_t *)"e ");
	MyUARTSendByte(cmd);
	MyUARTSendStringZ((uint8_t *)" ");
	MyUARTPrintHex(code);
	MyUARTSendCRLF();
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


/**
 * New radio system (RFM69 module + MCU) operating modes:
 *
 * Mode 0 : Radio off, MCU in deepsleep. Can be woken by host. Current
 * drain ~60uA.
 *
 * Mode 1 : (reserved)
 *
 * Mode 2 : Radio in low power polling mode (poll, and listen, then sleep). MCU in
 * deepsleep during sleep phase. The poll packet replaces the heartbeat feature.
 *
 * Mode 3 : Radio on RX, listening for packets. MCU polling radio module continuously.
 *
 * Mode is stored in bits 3:0 of 'flags'.
 */
void setOpMode (uint32_t mode) {
	flags &= ~0xf;
	flags |= mode;
}

int main(void) {

	// Enable MCU subsystems needed in this application in one step. Saves bytes.
	// (it would be preferable to have in each subsystem init, but we're very
	// tight for space on LPC810).
    LPC_SYSCON->SYSAHBCLKCTRL |=
    		(1<<7)    // Switch Matrix (SWM)
    		| (1<<6)  // GPIO
    		| (1<<9)  // Wake Timer (WKT)
    		| (1<<14) // USART0
    		| (1<<17) // Watchdog timer
    		;



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
	MyUARTInit(UART_BPS);

	// Display firmware version on boot
	cmd_version(1,NULL);


#ifdef LPC810
	// Long delay (10-20seconds) to allow debug probe to reflash. Remove in production.
	// Tried using regular SLEEP mode for this, but seems debugger doesn't work in that mode.
	loopDelay(20000000);

	// Won't be able to use debug probe from this point on (unless UART S 0 command used)
	SwitchMatrix_Spi_Init();

	// Hack:
	// Display firmware version again to indicate SPI is in operation
	cmd_version(1,NULL);
#endif


	//
    // Watchdog configuration
	//

	// Let WDT run while in power down mode
	LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);

	// Setup watchdog oscillator frequency
    /* Freq = 0.5Mhz, div_sel is 0x1F, divided by 64. WDT_OSC should be 7.8125khz */
    LPC_SYSCON->WDTOSCCTRL = (0x1<<5)|0x1F;
    LPC_WWDT->TC = 0x100000;
    LPC_WWDT->MOD = (1<<0) // WDEN enable watchdog
    			| (1<<1); // WDRESET : enable watchdog to reset on timeout
    // Watchdog feed sequence
    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    /* Make sure feed sequence executed properly */
    //loopDelay(1000);


	spi_init();

	// Configure hardware interface to radio module
	rfm69_init();

	uint8_t rssi;

	uint8_t *cmdbuf;
	uint8_t *args[8];

	// Radio frame receive buffer
	uint8_t frxbuf[66];
	uint8_t frame_len;

	// Acts as a crude clock
//	uint32_t loop_counter = 0;

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


#ifdef FEATURE_TEMPERATURE

		if ((flags&0xf)==MODE_LOW_POWER_POLL) {
		// Must read temperature from STDBY or FS mode
		rfm69_mode(RFM69_OPMODE_Mode_STDBY);

		// Start temperature conversion
		rfm69_register_write(0x4E,0x8);

		// Should monitor register Temp1 bit 2 for transition to 0, but a dumb delay is more
		// space efficient (down to last few bytes of flash!)
		loopDelay(10000);

		// Hack: put temperature into unused register (AESKey1) for remote reading
		rfm69_register_write(0x3E,rfm69_register_read(0x4F));
		}

#endif

		if ( (flags&0xf) == MODE_AWAKE) {
			rfm69_mode(RFM69_OPMODE_Mode_RX);
		}

#ifdef FEATURE_DEEPSLEEP
		// Test for MODE_OFF or MODE_LOW_POWER_POLL
		if ( (flags&0x1) == 0) {

			// Set radio in SLEEP mode
			rfm69_mode(RFM69_OPMODE_Mode_SLEEP);

			// Setup power management registers so that WFI causes DEEPSLEEP
			prepareForPowerDown();

			// Writing into WKT counter automatically starts wakeup timer
			// Polling interval determined by bits 11:8 of flags.
			// Ts = 0.5 * 2 ^ flags[11:8]
			// is 500 ms x 2 to the power of this value (ie 0=500ms, 1=1s, 2=2s,3=4s,4=8s...)
			LPC_WKT->COUNT = 5000 << ((flags>>8)&0xf);

			// DeepSleep until WKT interrupt
			__WFI();

			// Allow time for clocks to stabilise after wake
			// TODO: can we use WKT and WFI?
			loopDelay(20000);

			// Indicator to host there is a short time window to issue command
			MyUARTSendStringZ("z\r\n");

			// Small window of time to allow host to exit sleep mode by issuing command.
			// Use WKT timer to wake from regular sleep mode (where UART works).
			SCB->SCR &= ~NVIC_LP_SLEEPDEEP;
		}
#else
		// If not using DEEPSLEEP, use regular SLEEP mode until next interrupt arrives
		if ( (flags&0xf) != MODE_AWAKE ) {
			__WFI();
		}
#endif

		// If in MODE_LOW_POWER_POLL send poll packet
		if ( (flags&0xf) == MODE_LOW_POWER_POLL) {
			uint8_t payload[3];
			payload[0] = 0xff;
			payload[1] = node_addr;
			payload[2] = 'z';
			rfm69_frame_tx(payload,3);

			// Allow time for response (120ms)
			// TODO: this is only long enough for a 4 or 5 bytes of payload.
			// Need to check for incoming signal and delay longer if transmission
			// in progress (or just delay longer.. which will affect battery drain).
			rfm69_mode(RFM69_OPMODE_Mode_RX);

			// Experimental: trig temperature measurement
			//rfm69_register_write(0x4E, 1<<3);

			LPC_WKT->COUNT = 2000;
			__WFI();
		}


		// Check for received packet on RFM69
		if ( ((flags&0xf)!=MODE_ALL_OFF) && rfm69_payload_ready()) {

			// Yes, frame ready to be read from FIFO
			frame_len = rfm69_frame_rx(frxbuf,66,&rssi);

			// TODO: tidy this
			// SPI error
			//if (frame_len>0) {

				// Feed watchdog
			    LPC_WWDT->FEED = 0xAA;
			    LPC_WWDT->FEED = 0x55;

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

				// Message requesting position report. This will return the string
				// set by the UART 'L' command verbatim.
				case 'R' :
				//case 'z' : // for testing only
				{
					int loc_len = strlen(current_loc);
					// report position
					int payload_len = loc_len + 3;
					uint8_t payload[payload_len];
					payload[0] = from_addr;
					payload[1] = node_addr;
					payload[2] = 'r';
					memcpy(payload+3,current_loc,loc_len);
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
					int i;
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
				case 'D' : {
					// If there is an uncompleted UART command in buffer then
					// remote command takes priority and whatever is in the
					// command buffer is dropped. Output an error to indicate
					// this has happened.
					// TODO: should we disable UART interrupt for this block?
					// because incoming UART char between now and cmd parse could
					// cause corruption of cmd buffer.
					if (MyUARTGetBufIndex()>0) {
						MyUARTBufReset();
						report_error('D',E_CMD_DROPPED);
					}
					MyUARTSendStringZ("d ");
					int payload_len = frame_len - 3;
					memcpy(cmdbuf,frxbuf+3,payload_len);
					cmdbuf[payload_len] = 0; // zero terminate buffer
					MyUARTSendStringZ(cmdbuf);
					MyUARTSendCRLF();
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

					MyUARTSendStringZ("p ");

					print_hex8(frxbuf[0]);
					MyUARTSendStringZ(" ");
					print_hex8(frxbuf[1]);
					MyUARTSendStringZ(" ");

					int i;
					for (i = 2; i < frame_len; i++) {
						print_hex8(frxbuf[i]);
					}
					MyUARTSendStringZ (" ");
					print_hex8(rssi);
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


			//} // end frame len valid check
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

#ifdef FEATURE_DEEPSLEEP
			// Any command will set mode to MODE_AWAKE if in MODE_ALL_OFF or MODE_LOW_POWER_POLL
			// TODO: will probably want to exclude remote commands
			setOpMode(MODE_AWAKE);
#endif

			MyUARTSendCRLF(LPC_USART0);

			cmdbuf = MyUARTGetBuf();

			// Split command line into parameters (separated by spaces)
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

			// Set radio system mode
			// TODO is this necessary now? Use F command instead.
			/*
			case 'M' : {
				flags &= ~0xf;
				if (args[1][0]=='0') {
					// no action
				} else if (args[1][0]=='2') {
					flags |= MODE_LOW_POWER_POLL;
				} else if (args[1][0]=='3') {
					flags |= MODE_AWAKE;
				}
				break;
			}
			*/

			// Set node address
			case 'N' :
			{
				cmd_set_node_addr(argc, args);
				break;
			}

			// Experimental reset
			case 'Q' : {
				NVIC_SystemReset();
				// no need for break
			}


			// Read RFM69 register
			case 'R' : {
				// Parameter is register address
				uint8_t *b;
				int regAddr = parse_hex(args[1],&b);
				MyUARTSendStringZ("r ");
				print_hex8 (regAddr);
				MyUARTSendStringZ(" ");
				print_hex8 (rfm69_register_read(regAddr));
				MyUARTSendCRLF(LPC_USART0);
				break;
			}



#ifdef LPC810
			// Allow re-enabling of SWD from UART API to facilitate reflashing
			case 'S' : {
				if (args[1][0]=='1') {
					// Note will disconnect SWD
					SwitchMatrix_Spi_Init();
					spi_init();
				} else {
					// Enable SWD pins
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


		} // end command switch block


	} // end main loop

}
