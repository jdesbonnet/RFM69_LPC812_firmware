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
#include "battery.h"
#include "switchmatrix.h"
#include "myuart.h"
#include "sleep.h"
#include "print_util.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"
#include "spi.h"
#include "led.h"
#include "flags.h"
#include "params.h"
#include "delay.h"
#include "eeprom.h"
#include "frame_buffer.h"

#include "lpc8xx_pmu.h"
#include "onewire.h"
#include "ds18b20.h"
#include "gps.h"
#include "iap_driver.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Coarse clock to keep track of time (for link loss etc) 1/100s intervals.
uint32_t last_frame_time;

params_union_type params_union;

// When in deepsleep or power down this lets us know which wake event occurred
typedef enum {
	NO_INTERRUPT = 0,
	WKT_INTERRUPT = 1,
	UART_INTERRUPT = 2,
	TIP_BUCKET_INTERRUPT = 3,
	PIEZO_SENSOR_INTERRUPT = 4,
	DIO0_INTERRUPT = 5
} interrupt_source_type;
volatile interrupt_source_type interrupt_source;

// Radio packet frame buffers
frame_buffer_type tx_buffer;
frame_buffer_type rx_buffer;

#ifdef FEATURE_TIP_BUCKET_COUNTER
	volatile uint32_t event_counter = 0;
	volatile uint32_t event_time = 0;
#endif


#ifdef FEATURE_SYSTICK
	volatile uint32_t systick_counter = 0;
	void SysTick_Handler(void) {
		systick_counter++; // every 10ms
		//coarse_clock++;
	}
#endif

/**
 * Print error code 'code' while executing command 'cmd' to UART.
 * @param cmd  Command that generated the error
 * @param code Error code (ref err.h)
 */
void report_error (uint8_t cmd, int32_t code) {
	if (code<0) code = -code;
	MyUARTSendStringZ("e ");
	MyUARTSendByte(cmd);
	MyUARTSendByte(' ');
	MyUARTPrintHex(code);
	MyUARTSendCRLF();
}


/**
 * Show system settings etc to UART
 */
void displayStatus () {
	// List optional features enabled
#ifdef FEATURE_TIP_BUCKET_COUNTER
	MyUARTSendStringZ ("; feature TIP_BUCKET_COUNTER\r\n");
#endif
#ifdef FEATURE_DS18B20
	MyUARTSendStringZ ("; feature DS18B20_TEMPERATURE\r\n");
#endif
#ifdef FEATURE_WS2812B
	MyUARTSendStringZ ("; feature WS2812B_LED\r\n");
#endif
#ifdef FEATURE_GPS_ON_USART1
	MyUARTSendStringZ ("; feature GPS\r\n");
#endif

	// Display parameters to UART
	//MyUARTSendStringZ ("; mode=");
	//MyUARTPrintHex(params_union.params.operating_mode);
	//MyUARTSendCRLF();
	tfp_printf ("; mode=%x\r\n",params_union.params.operating_mode);

	//MyUARTSendStringZ ("; node_addr=");
	//MyUARTPrintHex(params_union.params.node_addr);
	//MyUARTSendCRLF();
	tfp_printf ("; node_addr=%x\r\n",params_union.params.node_addr);

	//MyUARTSendStringZ ("; poll_interval=");
	//MyUARTPrintHex(params_union.params.poll_interval);
	//MyUARTSendCRLF();
	tfp_printf ("; poll_interval=%x\r\n",params_union.params.poll_interval);

	//MyUARTSendStringZ ("; listen_period_cs=");
	//MyUARTPrintHex(params_union.params.listen_period_cs);
	//MyUARTSendCRLF();
	tfp_printf ("; listen_period=%x\r\n",params_union.params.listen_period_cs);

	//MyUARTSendStringZ ("; link_loss_timeout_s=");
	//MyUARTPrintHex(params_union.params.link_loss_timeout_s);
	//MyUARTSendCRLF();
	tfp_printf ("; link_loss_timeout=%x\r\n",params_union.params.link_loss_timeout_s);

	//MyUARTSendStringZ ("; eeprom_addr=");
	//MyUARTPrintHex((uint32_t)eeprom_get_addr());
	//MyUARTSendCRLF();
	tfp_printf ("; eeprom_addr=%x\r\n",eeprom_get_addr());

#ifdef FEATURE_GPS_ON_USART1
	gps_report_status();
#endif


	uint32_t unique_id[4];
	iap_read_unique_id(unique_id);
	MyUARTSendStringZ("; mcu_unique_id= ");
	MyUARTPrintHex(unique_id[0]);
	MyUARTSendByte(' ');
	MyUARTPrintHex(unique_id[1]);
	MyUARTSendByte(' ');
	MyUARTPrintHex(unique_id[2]);
	MyUARTSendByte(' ');
	MyUARTPrintHex(unique_id[3]);
	MyUARTSendCRLF();


#ifdef FEATURE_WATCHDOG_TIMER
	MyUARTSendStringZ("; WatchDogTimer=");
	MyUARTPrintHex(LPC_WWDT->TV);
	MyUARTSendCRLF();
#endif
#ifdef FEATURE_TIP_BUCKET_COUNTER
	MyUARTSendStringZ("; TipBucketCounter=");
	MyUARTPrintHex(event_counter);
	MyUARTSendCRLF();
#endif

	MyUARTSendStringZ ("; supply_voltage_mV=");
	MyUARTPrintDecimal(readBattery());
	MyUARTSendCRLF();
}

// TODO: is volatile necessary?

static uint32_t last_gps_report_t = 0;


// To facilitate tfp_printf()
void myputc (void *p, char c) {
	MyUARTSendByte(c);
}

int main(void) {

	// Enable MCU subsystems needed in this application in one step. Saves bytes.
	// (it would be preferable to have in each subsystem init, but we're very
	// tight for space on LPC810).
    LPC_SYSCON->SYSAHBCLKCTRL |=
    		(1<<7)    // Switch Matrix (SWM)
    		| (1<<6)  // GPIO
    		| (1<<9)  // Wake Timer (WKT)
    		| (1<<17) // Watchdog timer (note: it may not be necessary to have on all the time)
    		;


    // Read MCU serial number
    uint32_t mcu_unique_id[4];
	iap_read_unique_id(&mcu_unique_id);

	// Read parameter block from eeprom
	eeprom_read(params_union.params_buffer);


#ifdef FEATURE_SYSTICK
    SysTick_Config( SYSTICK_DELAY );
#endif

	/*
	 * LPC8xx features a SwitchMatrix which allows most functions to be mapped to most pins.
	 * This setups up the pins in a way that's convenient for our physical circuit layout.
	 * In the case of the LPC810, use of SPI (which necessitates loss of SWD functionality)
	 * is delayed to allow opportunity to reprogram device via SWD.
	 */

#ifdef LPC812
#ifdef BOARD_V1B_HACK
    // Due to rev 1 PCB layout bug it was necessary to re-route RXD to non-default pin (PIO0_11).
    // This is not necessary on all boards as the PCB can be fixed with a little careful
    // knife work. This is one board which I forgot to make that fix. Expect to remove this
    // code after a while as it's just crud to facilitate one prototype board.
    if (mcu_unique_id[0] == BOARD_V1B_HACK_MCU_ID) {
    	SwitchMatrix_LPC812_PCB1b_Init();
    } else {
#ifdef FEATURE_GPS_ON_USART1
    	SwitchMatrix_GPS_UART1_Init();

#else
    	SwitchMatrix_Init();
#endif
    }

#else
	SwitchMatrix_Init();
	//SwitchMatrix_Acmp_Init();
#endif
#endif

	// Reset GPIO
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
	LPC_SYSCON->PRESETCTRL |= (0x1<<10);
	//lpc8xx_peripheral_reset(10);


	//
	// Initialize UART(s)
	//

	// API UART
	MyUARTxInit(LPC_USART0, UART_BPS);

#ifdef FEATURE_GPS_ON_USART1
	// Optional GPS module
	MyUARTxInit(LPC_USART1, UART_BPS);
#endif


	// Doc on this printf() library here:
	// http://www.sparetimelabs.com/tinyprintf/tinyprintf.php
	// GNU LGPL license
	init_printf(NULL,myputc);

	// Display firmware version on boot
	cmd_version(1,NULL);


#ifdef BOARD_V1B_HACK
	if (mcu_unique_id[0] == BOARD_V1B_HACK_MCU_ID) {
		MyUARTSendStringZ("; Board_V1B_Hack in effect (UART RXD on PIO0_11)\r\n");
	}
#endif

	// Auto assign node ID based on MCU ID
	// TODO: MCU unique ID is actually a 128bit value, but for convenience
	// incorrectly assuming that first 32bits are unique.
	// TODO: this is a temporary hack: move this configuration to a
	// structure and initalize in config.h
	if (params_union.params.node_addr == 0xFF) {
		switch (mcu_unique_id[0]) {
		case 0x5034039:
			params_union.params.node_addr = 0x42;
			break;
		case 0x18044043:
			params_union.params.node_addr = 0x43;
			break;
		case 0x05034043:
			params_union.params.node_addr = 0x44;
			break;
		case 0x1902E033:
			params_union.params.node_addr = 0x46;
			break;
		}
	}


#ifdef FEATURE_WATCHDOG_TIMER
	//
    // Watchdog configuration
	//

	// Power to WDT
	LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);

	// Setup watchdog oscillator frequency
    /* Freq = 0.5Mhz, div_sel is 0x1F, divided by 64. WDT_OSC should be 7.8125khz */
    LPC_SYSCON->WDTOSCCTRL = (0x1<<5)|0x1F;
    LPC_WWDT->TC = params_union.params.link_loss_timeout_s * 2000;
    LPC_WWDT->MOD = (1<<0) // WDEN enable watchdog
    			| (1<<1); // WDRESET : enable watchdog to reset on timeout
    // Watchdog feed sequence
    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    /* Make sure feed sequence executed properly */
    //loopDelay(1000);

    NVIC_EnableIRQ( (IRQn_Type) WDT_IRQn);

#endif


	spi_init();

	// Configure hardware interface to radio module
	rfm69_init();

	uint32_t regVal;

#ifdef RESET_PIN
	// If RFM reset line available, configure PIO pin for output and set low
	// (RFM resets are active high).
	regVal = LPC_GPIO_PORT->DIR0;
	regVal |= (1<<RESET_PIN);
	LPC_GPIO_PORT->DIR0 = regVal;
	// Force reset on boot
	LPC_GPIO_PORT->SET0=(1<<RESET_PIN);
	delay(20000);
	LPC_GPIO_PORT->CLR0=(1<<RESET_PIN);
#endif

#ifdef DIO0_PIN
	// If RFM DIO0 output line is available configure PIO pin for input. In RFM69 packet
	// mode DIO0 high signifies frame is ready to read from FIFO.
	regVal = LPC_GPIO_PORT->DIR0;
	regVal &= ~(1<<DIO0_PIN);
	LPC_GPIO_PORT->DIR0 = regVal;

	// Enable interrupt when DIO0 goes high.
	LPC_SYSCON->PINTSEL[3] = DIO0_PIN;
	LPC_PIN_INT->ISEL &= ~(0x1<<3);	/* Edge trigger */
	LPC_PIN_INT->IENR |= (0x1<<3);	/* Rising edge */
	NVIC_EnableIRQ((IRQn_Type)(PININT3_IRQn));
#endif

	// Absolute value of the RSSI in dBm, 0.5dB steps.
	// RSSI_dBm = -rssi/2
	uint8_t rssi;

	//uint8_t *cmdbuf;
	uint8_t *args[8];


	// Use to ID each sleep ping packet. No need to init (saves 4 bytes).
	uint32_t sleep_counter = 0;

	int argc;

	displayStatus();

	//tx_buffer.header.from_addr = DEFAULT_NODE_ADDR;
	tx_buffer.header.from_addr = params_union.params.node_addr;

#ifdef FEATURE_UART_INTERRUPT
	// Experimental wake on activity on UART RXD (RXD is normally shared with PIO0_0)
	LPC_SYSCON->PINTSEL[0] = UART_RXD_PIN; // PIO0_0 aka RXD
	LPC_PIN_INT->ISEL &= ~(0x1<<UART_RXD_PIN);	/* Edge trigger */
	LPC_PIN_INT->IENR |= (0x1<<UART_RXD_PIN);	/* Rising edge */
	NVIC_EnableIRQ((IRQn_Type)(PININT0_IRQn));
#endif


#ifdef FEATURE_TIP_BUCKET_COUNTER
	// Set TIPBUCKET_PIN as input
	LPC_GPIO_PORT->DIR0 &= ~(1<<TIPBUCKET_PIN);

	// Pulldown resistor on PIO0_16 (tip bucket)
	LPC_IOCON->PIO0_16=(0x1<<3);

	// Set interrupt on this pin.
	LPC_SYSCON->PINTSEL[1] = TIPBUCKET_PIN;
	LPC_PIN_INT->ISEL &= ~(0x1<<1);	/* Edge trigger */
	LPC_PIN_INT->IENR |= (0x1<<1);	/* Rising edge */
	NVIC_EnableIRQ((IRQn_Type)(PININT1_IRQn));

	// Experimental wake on comparator activity. Comparator output on PIO0_13
	LPC_SYSCON->PINTSEL[2] = 13;
	LPC_PIN_INT->ISEL &= ~(0x1<<2);	/* Edge trigger */
	LPC_PIN_INT->IENR |= (0x1<<2);	/* Rising edge */
	NVIC_EnableIRQ((IRQn_Type)(PININT2_IRQn));

	// No need for additional modifications to ACMP registers. Switch off clock.
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<19);
#endif

#ifdef FEATURE_DS18B20
	// Pullup resistor on DS18B20 data pin PIO0_14
	//LPC_IOCON->PIO0_14=(0x2<<3);
	//GPIOSetDir(0,DS18B20_PIN,1);
	ow_init(0,DS18B20_PIN);
#endif

	// Configure RFM69 registers for this application. I found that it was necessary
	// to delay a short period after powerup before configuring registers.
	delay(300000);

#ifdef FEATURE_LED
	// Optional Diagnostic LED. Configure pin for output and blink 3 times.
	//GPIOSetDir(0,LED_PIN,1);
	LPC_GPIO_PORT->DIR0 |= (1<<LED_PIN);
#endif

	//
	// Tests
	//

	// Bit indicating pass/fail of each test. Bit high indicates failure.
	// Bit 0 : RFM69 module
	// Bit 1 : DS18B20 temperature sensor

	uint32_t test_result = 0;


	// Test RFM69 radio module
	if (rfm69_test() != 0) {
		// Error communicating with RFM69: 4 blinks
		ledBlink(4);
		test_result |= 1<<0;
	}

#ifdef FEATURE_DS18B20
	if (ow_reset() == 0) {
		test_result |= 1<<1;
	}
#endif

	if (test_result == 0){
		// Normal: 2 blinks
		ledBlink(2);
	} else {
		ledBlink(2+test_result);
	}
	MyUARTSendStringZ("k ");
	MyUARTPrintHex(test_result);
	MyUARTSendCRLF();


	rfm69_config();

	//
	// Main program loop
	//
	while (1) {


#ifdef FEATURE_RFM69_TEMPERATURE
		if (params_union.params.operating_mode==MODE_LOW_POWER_POLL) {
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


#ifdef FEATURE_TIP_BUCKET_COUNTER
		if ( (event_time != 0) && (systick_counter - event_time > 10) ) {
			MyUARTSendStringZ("a ");
			MyUARTPrintHex(event_counter);
			MyUARTSendCRLF();
			MyUARTSendDrain();
			//event_counter = 0;
			event_time= 0;
		}
#endif

		// TODO: can we avoid calling rfm69_mode() on every iteration?
		// TODO: use macro to test flags
		if (params_union.params.operating_mode == MODE_AWAKE) {
			rfm69_mode(RFM69_OPMODE_Mode_RX);
		}

#ifdef FEATURE_DEEPSLEEP
		// Test for MODE_OFF or MODE_LOW_POWER_POLL (LSB==0 for those two modes)
		if ( params_union.params.operating_mode == MODE_LOW_POWER_POLL
				|| params_union.params.operating_mode == MODE_RADIO_OFF
				) {

			// Set radio in SLEEP mode
			rfm69_mode(RFM69_OPMODE_Mode_SLEEP);

			// Reset source of wake event
			interrupt_source = 0;

			// Setup power management registers so that WFI causes DEEPSLEEP
			prepareForPowerDown();

			// Writing into WKT counter automatically starts wakeup timer. WKT timer
			// is driven by 10kHz low power oscillator. However this is +/- 40%.
			// Finding 7.5kHz closer to the mark.
			uint32_t wakeup_time = 7500  * params_union.params.poll_interval;
			LPC_WKT->COUNT = wakeup_time ;

			// DeepSleep until WKT interrupt (or PIN interrupt)
			__WFI();

			// WKT in 100us increments, want sleep_clock in 10ms increments
			systick_counter += (wakeup_time - LPC_WKT->COUNT)/100;

			// Experimental: Re-set SPI pins after deepsleep/powerdown conditioning
			spi_init();

			// Allow time for clocks to stabilise after wake
			// TODO: can we use WKT and WFI?
			delay(20000);

			if (interrupt_source == UART_INTERRUPT) {
				//setOpMode(MODE_AWAKE);

				MyUARTSendStringZ("; UART interrupt, awake\r\n");
				params_union.params.operating_mode = MODE_AWAKE;

				// Probably crud in buffer : clear it.
				MyUARTBufReset();
			}

			// Indicator to host there is a short time window to issue command
			MyUARTSendStringZ("z\r\n");
			//MyUARTPrintDecimal(LPC_WWDT->TV);

			// Undo DeepSleep/PowerDown flag so that next WFI goes into regular sleep.
			SCB->SCR &= ~NVIC_LP_SLEEPDEEP;
		}
#else
		// If not using DEEPSLEEP, use regular SLEEP mode until next interrupt arrives
		if ( (flags&0xf) != MODE_AWAKE ) {
			__WFI();
		}
#endif


		// If in MODE_LOW_POWER_POLL send status packet
		if ( params_union.params.operating_mode == MODE_LOW_POWER_POLL) {

			tx_buffer.header.to_addr = 0xff; // broadcast
			tx_buffer.header.msg_type = 'z';
			tx_buffer.payload[0] = sleep_counter++;
#ifdef FEATURE_TIP_BUCKET_COUNTER
			tx_buffer.payload[1] = event_counter;
#else
			tx_buffer.payload[1] = 0xFF;
#endif
			tx_buffer.payload[2] = readBattery()/100;

#ifdef FEATURE_DS18B20

			// Pullup resistor on DS18B20 data pin PIO0_14
			LPC_IOCON->PIO0_14=(0x2<<3);

			ow_init(0,DS18B20_PIN);
			int32_t temperature = ds18b20_temperature_read();
			MyUARTSendStringZ("; temperature=");
			MyUARTPrintHex(temperature);
			MyUARTSendCRLF();
			tx_buffer.payload[3] = temperature>>8;
			tx_buffer.payload[4] = temperature&0xff;
#endif

#ifdef FEATURE_LED
			LPC_GPIO_PORT->PIN0 |= (1<<LED_PIN);
			rfm69_frame_tx(tx_buffer.buffer,8);
			LPC_GPIO_PORT->PIN0 &= ~(1<<LED_PIN);
#else
			rfm69_frame_tx(tx_buffer.buffer,8);
#endif





			// Allow time for response (120ms)
			// TODO: this is only long enough for a 4 or 5 bytes of payload.
			// Need to check for incoming signal and delay longer if transmission
			// in progress (or just delay longer.. which will affect battery drain).
			rfm69_mode(RFM69_OPMODE_Mode_RX);

			// Delay in SLEEP for 200ms to allow for reply radio frame
			// TODO: bug: we need to start this counter after the previous frame
			// has finished TX. For the moment, extend from 200ms to 800ms to
			// compensate for longer frame TX
			/*
			LPC_WKT->COUNT = 8000;
			interrupt_source = 0;
			do {
				__WFI();
			} while (interrupt_source != WKT_INTERRUPT);
			*/

			delayMilliseconds(params_union.params.listen_period_cs*10);
		}


		// IF we have access to RFM69 DIO0 use that to determine if frame ready to read,
		// else poll status register through SPI port.
#ifdef DIO0_PIN
#define IS_PAYLOAD_READY() (LPC_GPIO_PORT->PIN0&(1<<DIO0_PIN))
#else
#define IS_PAYLOAD_READY() rfm69_payload_ready()
#endif

		// Check for received packet on RFM69
		//if ( ((flags&0xf)!=MODE_ALL_OFF) && rfm69_payload_ready()) {
		//if ( ((flags&0xf)!=MODE_ALL_OFF) && IS_PAYLOAD_READY()  ) {
		// if ( IS_PAYLOAD_READY()  ) {
		if (rfm69_payload_ready()) {

			rssi = rfm69_register_read(RFM69_RSSIVALUE);

			// Yes, frame ready to be read from FIFO
#ifdef FEATURE_LED
			LPC_GPIO_PORT->PIN0 |= (1<<LED_PIN);
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,66);
			LPC_GPIO_PORT->PIN0 &= ~(1<<LED_PIN);
#else
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,66);
#endif

			last_frame_time = systick_counter;

			// TODO: tidy this
			// SPI error
			//if (frame_len>0) {

#ifdef FEATURE_WATCHDOG_TIMER
			// Feed watchdog whenever a packet is successfully received.
			LPC_WWDT->FEED = 0xAA;
			LPC_WWDT->FEED = 0x55;
#endif


			// 0xff is the broadcast address
			if ( rx_buffer.header.to_addr == 0xff
					|| rx_buffer.header.to_addr == tx_buffer.header.from_addr) {

				// This frame is for us! Examine messageType field for appropriate action.

				switch (rx_buffer.header.msg_type) {

#ifdef FEATURE_REMOTE_PKT_TX
				// Experimental remote packet transmit / relay
				case 'B' : {
					int payload_len = frame_len - 3;
					//uint8_t payload[payload_len];
					memcpy(tx_buffer.payload,rx_buffer.payload,payload_len);
					rfm69_frame_tx(tx_buffer.buffer, payload_len+3);
					break;
				}
#endif

				// Remote command execute
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

					tfp_printf ("; received remote cmd from %x\r\n",tx_buffer.header.from_addr);

					// Echo remote command to UART, copy remote command to UART buffer and
					// trigger UART command parsing.
					MyUARTSendStringZ("d ");
					int payload_len = frame_len - 3;
					uint8_t *uart_buf = MyUARTGetBuf();
					memcpy(uart_buf,rx_buffer.payload,payload_len);
					uart_buf[payload_len] = 0; // zero terminate buffer
					MyUARTSendStringZ((char *)uart_buf);
					MyUARTSendCRLF();
					MyUARTSetBufFlags(UART_BUF_FLAG_EOL);
					break;
				}

				// Start ping-pong test
				case 'P' :
				{
					MyUARTSendStringZ ("; received ping from ");
					MyUARTPrintHex(rx_buffer.header.from_addr);
					MyUARTSendCRLF();
					// Return ping-pong request to source
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'P'; // ping-pong message
					// payload[0] has a message counter: increment
					tx_buffer.payload[0] = rx_buffer.payload[0]+1;
					tx_buffer.payload[1] = rssi;
					rfm69_frame_tx(tx_buffer.buffer, 5);
					break;
				}

				// Position query request.
				// Message requesting position report. This will return the string
				// set by the GPS or the 'G' command
#ifdef FEATURE_GPS_ON_USART1
				case 'R' :
				{

					MyUARTSendStringZ ("; received node query request from ");
					MyUARTPrintHex(tx_buffer.header.from_addr);
					MyUARTSendCRLF();
					gps_send_status(rx_buffer.header.from_addr);
					break;
				}
#endif

				// Node status request
				case 'S' :
				{
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 's'; // node status response
					tx_buffer.payload[0] = readBattery()/100;
					tx_buffer.payload[1] = rssi;
					rfm69_frame_tx(tx_buffer.buffer, 5);
					break;
				}


				// Remote request to LED blink
				case 'U' : {
					//tx_buffer.header.to_addr = from_addr;
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'u';
					rfm69_frame_tx(tx_buffer.buffer, 3);
					ledBlink(3);
					break;
				}

				// Remote RFM69 register read (0x58)
				case 'X' : {
					uint8_t base_addr = rx_buffer.payload[0];
					uint8_t read_len = rx_buffer.payload[1];
					if (read_len>16) read_len = 16;
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'x';
					tx_buffer.payload[0] = base_addr;
					int i;
					for (i = 0; i < read_len; i++) {
						tx_buffer.payload[i+1] = rfm69_register_read(base_addr+i);
					}
					rfm69_frame_tx(tx_buffer.buffer, read_len+4);
					break;
				}

				// Remote RFM69 register write
				case 'Y' : {
					uint8_t base_addr = rx_buffer.payload[0];
					uint8_t write_len = frame_len - 4;
					if (write_len > 16) write_len = 16;
					int i;
					for (i = 0; i < write_len; i++) {
						rfm69_register_write(base_addr+i,rx_buffer.payload[i+1]);
					}
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'y';
					rfm69_frame_tx(tx_buffer.buffer, 3);
					break;
				}


				// Experimental write to MCU memory (0x3E)
				case '>' : {
					// At 32bit memory address at payload+0 write 32bit value at payload+4
					// Note: must be 32bit word aligned.
					uint32_t **mem_addr;
					mem_addr = (uint32_t **)rx_buffer.payload;
					uint32_t *mem_val;
					mem_val = (uint32_t *)(rx_buffer.payload+4);
					**mem_addr = *mem_val;
					/*
					int i;
					for (i = 0; i < (frame_len-sizeof(frame_header_type)-4)/4; i++) {
						**mem_addr = rx_buffer.payload[i+4];
					}
					*/
					break;
				}
				// Experimental read from MCU memory (0x3C)
				case '<' : {
					// Return 32bit value from memory at payload+0
					// Note address and result is LSB first (little endian)
					uint32_t **mem_addr;
					mem_addr = (uint32_t **)rx_buffer.payload;
					//uint8_t len = rx_buffer.payload[4];
					// First 4 bytes of return is base address
					*(uint32_t *)tx_buffer.payload = *mem_addr;
					*(uint32_t *)(tx_buffer.payload+4) = **mem_addr;
					/*
					int i;
					for (i = 0; i < len; i++) {
						tx_buffer.payload[i+4] = **mem_addr;
					}
					*/
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = '<'-32;
					rfm69_frame_tx(tx_buffer.buffer, sizeof(frame_header_type)+8);
					break;
				}
				// Experimental execute from memory (!?!)
				case 'E' : {
					// Execute function at memory location payload+0
					// Note on ARM Cortex M devices LSB always 1 for Thumb instruction set.
					uint32_t **mem_addr;
					mem_addr = rx_buffer.payload;
					typedef void (*E)(void);
					E e_entry = (E)*mem_addr;
					e_entry();
					break;
				}
				}


				// If none of the above cases match, output packet to UART
				{

					MyUARTSendStringZ("p ");

					print_hex8(rx_buffer.header.to_addr);
					MyUARTSendByte(' ');
					print_hex8(rx_buffer.header.from_addr);
					MyUARTSendByte(' ');


					int i;
					for (i = 2; i < frame_len; i++) {
						print_hex8(rx_buffer.buffer[i]);
					}
					MyUARTSendByte(' ');
					print_hex8(rssi);
					MyUARTSendCRLF();
				}

			} else {

#ifdef FEATURE_DEBUG
				MyUARTSendStringZ("; Ignoring packet from ");
				MyUARTPrintHex(to_addr);
				MyUARTSendCRLF();
#endif

			}


			//} // end frame len valid check
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {


#ifdef FEATURE_DEEPSLEEP
			// Any command will set mode to MODE_AWAKE if in MODE_ALL_OFF or MODE_LOW_POWER_POLL
			// TODO: will probably want to exclude remote commands
			//setOpMode(MODE_AWAKE);
			params_union.params.operating_mode = MODE_AWAKE;
#endif

			MyUARTSendCRLF();




			uint8_t *uart_buf = MyUARTGetBuf();

			// Split command line into parameters (separated by spaces)
			argc = 1;
			args[0] = uart_buf;
			while (*uart_buf != 0) {
				if (*uart_buf == ' ') {
					*uart_buf = 0;
					args[argc++] = uart_buf+1;
				}
				uart_buf++;
			}

			// TODO: using an array of functions may be more space efficient than
			// switch statement.

			switch (*args[0]) {

			case 'B' : {
				cmd_set_uart_speed (argc, args);
				break;
			}

			// Reset RFM69 with default configuration
			case 'C' :
			{
				rfm69_config();
				break;
			}

			// Transmit remote command
			case 'D' :
			{
				cmd_remote_cmd(argc, args);
				break;
			}

			// Echo GPS from UART1 to UART0
			case 'E' :
			{
				if (argc != 2) {
					report_error('E',E_INVALID_ARG);
					break;
				}
				params_union.params.gps_echo = parse_hex(args[1]);
			}

			// Transmit arbitrary packet
			case 'F' : {
				int status = cmd_node_query(argc, args);
				if ( status ) {
					report_error('F', status);
				}
				break;
			}

			// Set/override GPS position
#ifdef FEATURE_GPS_ON_USART1
			case 'G' : {
				cmd_gps (argc, args);
				break;
			}
#endif

			// Display MCU unique ID
			case 'I' : {
				MyUARTSendStringZ("i ");
				//uint32_t part_id;
				//iap_read_part_id(&part_id);
				MyUARTPrintHex(mcu_unique_id[0]);
				MyUARTSendCRLF();
				break;
			}

			// Set link loss reset timeout
			case 'J' : {
				if (argc == 1) {
					MyUARTSendStringZ("j ");
					MyUARTPrintHex(params_union.params.link_loss_timeout_s);
					MyUARTSendByte(' ');
					MyUARTPrintHex(LPC_WWDT->TV);
					MyUARTSendByte(' ');
					MyUARTPrintHex(LPC_WWDT->TC);
					MyUARTSendCRLF();
					break;
				}
				params_union.params.link_loss_timeout_s = parse_hex(args[1]);
				LPC_WWDT->TC = params_union.params.link_loss_timeout_s * 2000;
			    // Watchdog feed sequence
			    LPC_WWDT->FEED = 0xAA;
			    LPC_WWDT->FEED = 0x55;
				break;
			}



			// Set current location
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_loc,args[1],MyUARTGetStrLen(args[1])+1);
				break;
			}

			// NMEA (only interested in $GPGLL)
			case '$' : {
				// +1 on len to include zero terminator
				if (MyUARTGetStrLen(args[0]) && args[0][4]=='L' && args[0][5]=='L') {
					memcpy(current_loc,args[0],MyUARTGetStrLen(args[0])+1);
				}
				break;
			}

			// Set radio system operating mode
			// M (no params) : report current mode
			// M <mode> : set mode
			// M <mode> S : set mode and save in EEPROM (and RESET)
			case 'M' : {
				if (argc == 1) {
					MyUARTSendStringZ("m ");
					MyUARTPrintHex(params_union.params.operating_mode);
					MyUARTSendCRLF();
					break;
				}
				params_union.params.operating_mode = parse_hex(args[1]);
				if (argc == 3) {
					if (args[2][0]=='S') {
						MyUARTSendStringZ("; ModeSaveAndReset\r\n");
						eeprom_write(params_union.params_buffer);
						MyUARTSendDrain();
						NVIC_SystemReset();
					}
				}
				break;
			}


			// Set or report node address
			case 'N' :
			{
				//cmd_set_node_addr(argc, args);

				if (argc == 1) {
					MyUARTSendStringZ("n ");
					MyUARTPrintHex(params_union.params.node_addr);
					MyUARTSendCRLF();
					break;
				}
				if (argc != 2) {
					//return E_WRONG_ARGC;
				} else {
					params_union.params.node_addr = parse_hex(args[1]);
					tx_buffer.header.from_addr = params_union.params.node_addr ;
				}
				break;
			}

			// Set parameter <byte-index> <value>
			/*
			case 'P' : {
				int status = cmd_param (argc, args);
				if ( status ) {
					report_error('P', status);
				}
				break;
			}
			*/

			case 'Q' : {
				// Report reset for reason=1 (explicit reset command)
				MyUARTSendStringZ("q 1\r\n");
				MyUARTSendDrain();
				NVIC_SystemReset();
				// no need for break
			}


			// Read RFM69 register
			case 'R' : {
				// Parameter is register address
				int regAddr = parse_hex(args[1]);
				MyUARTSendStringZ("r ");
				print_hex8 (regAddr);
				MyUARTSendByte(' ');
				print_hex8 (rfm69_register_read(regAddr));
				MyUARTSendCRLF();
			}

			case 'S' :
			{
				eeprom_write(params_union.params_buffer);
				break;
			}

			// Transmit arbitrary packet
			case 'T' : {
				int status = cmd_packet_transmit(argc, args);
				if ( status ) {
					report_error('T', status);
				}
				break;
			}

			// Turn LED on/off
			case 'U' : {
				parse_hex(args[1]) == 0 ? ledOff() : ledOn();
				break;
			}

			// Report RFM69 controller (this) firmware
			case 'V' : {
				cmd_version(1,NULL);
				break;
			}

			// Write RFM69 register(s)
			// <base-addr> <val0> <val1>....
			case 'W' : {
				int regAddr = parse_hex(args[1]);
				int regValue;
				int i;
				for (i = 2; i < argc; i++) {
					regValue = parse_hex(args[i]);
					rfm69_register_write(regAddr++,regValue);
				}
				break;
			}

			// Read battery. Return in both x response message and also
			// human friendly value in decimal in info message
			case 'X' :  {
				int battery_vm = readBattery();
				MyUARTSendStringZ("; power_mV=");
				MyUARTPrintDecimal(battery_vm);
				MyUARTSendStringZ("\r\nx ");
				MyUARTPrintHex(battery_vm);
				MyUARTSendCRLF();
				break;
			}

			case 'Y' : {
				//uint64_t rom_addr = ds18b20_rom_read();
				//MyUARTPrintHex(rom_addr>>32);
				//MyUARTPrintHex(rom_addr);
				//MyUARTSendCRLF();

				MyUARTSendStringZ("y ");
				MyUARTPrintHex(ds18b20_temperature_read());
				MyUARTSendCRLF();
				break;
			}

			// Experimental write to memory. Write 8 bits at a time as writing more
			// might trigger a fault if not correctly aligned.
			// > 32bit-addr byte-value
			case '>' : {
				uint32_t *mem_addr;
				mem_addr = (uint32_t *)parse_hex(args[1]);
				*mem_addr = parse_hex(args[2]);
				break;
			}
			// Experimental read from memory
			// < 32bit-addr
			case '<' : {
				uint32_t *mem_addr;
				mem_addr = (uint32_t *)parse_hex(args[1]);
				MyUARTPrintHex(*mem_addr);
				MyUARTSendCRLF();
				break;
			}

			// Display system status
			case '?' : {
				displayStatus();
				break;
			}

			default : {
				report_error(*args[0], E_INVALID_CMD);
			}

			}

			// Reset UART command line buffer ready for next command
			MyUARTBufReset();


		} // end command switch block


#ifdef FEATURE_GPS_ON_USART1
		// Display GPS info if changed
		if ( (gps_get_last_position_t() != last_gps_report_t) && (params_union.params.gps_echo&0x2) ) {
			gps_report_status();

			// Send GPS position over radio
			if (params_union.params.gps_echo&(1<<2)) {
				gps_send_status(0xFF);
			}
			last_gps_report_t = gps_get_last_position_t();
		}
#endif


#ifdef FEATURE_LINK_LOSS_RESET
		if ( params_union.params.operating_mode == MODE_LOW_POWER_POLL) {
			uint32_t last_frame_age = systick_counter - last_frame_time;
			MyUARTSendStringZ ("; last_frame_age=");
			MyUARTPrintHex(last_frame_age);
			MyUARTSendStringZ ("\r\n");
			if (params_union.params.link_loss_timeout_s != 0
					&& (last_frame_age > params_union.params.link_loss_timeout_s*100)) {
				// Report MCU reset (reason=2 link loss timeout)
				MyUARTSendStringZ("q 2\r\n");
				MyUARTSendDrain();
				// Reset
				NVIC_SystemReset();
			}
		}
#endif

		// Reduce rate of polling RFM for frame by waiting for SysTick (or any) interrupt.
		// This additional sleep time may(?) help reduce power consumption (to be confirmed).
		__WFI();


	} // end main loop

}

#ifdef FEATURE_UART_INTERRUPT
/**
 * This interrupt is triggered on activity on UART RXD. It's used to facilitate immediate
 * wake of the MCU by the host by transmitting a char on the UART. The waking character
 * will be lost however. One solution to lost char is to prefix each command with one or
 * two <ESC> because <ESC> are ignored by UART API but will trigger this interrupt.
 */
void PININT0_IRQHandler (void) {
	// Clear interrupt
	LPC_PIN_INT->IST = 1<<0;
	//LPC_USART0->TXDATA='*';
	interrupt_source = UART_INTERRUPT;
}
#endif

void WDT_IRQHandler (void) {
	//MyUARTSendStringZ("q 2\r\n");
	//MyUARTSendDrain();
	//loopDelay(20000);
}

#ifdef FEATURE_TIP_BUCKET_COUNTER

/**
 * Interrupt generated by tip bucket
 */
void PININT1_IRQHandler (void) {

	// Clear interrupt
	LPC_PIN_INT->IST = 1<<1;


	// Printing event_counter in here results in very strange behavior.  Probably due to
	// using MyUART inside ISR.

	if (event_time == 0) {
		event_counter++;
		event_time = systick_counter;
		//LPC_USART0->TXDATA='B';
	}

	interrupt_source = TIP_BUCKET_INTERRUPT;
}

/**
 * Interrupt generated by comparator output.
 */
void PININT2_IRQHandler (void) {
	// Clear interrupt
	LPC_PIN_INT->IST = 1<<2;
	//LPC_USART0->TXDATA='X';
	interrupt_source = PIEZO_SENSOR_INTERRUPT;
}




void CMP_IRQHandler (void) {

	// Clear interrupt by writing 1 to CTRL bit 20 (EDGECLR), then 0.
	LPC_CMP->CTRL |= (1<<20);
	LPC_CMP->CTRL &= ~(1<<20);

	LPC_USART0->TXDATA='A';

	event_counter++;

	if (event_time == 0) {
		event_time = systick_counter;
	}
}
#endif

#ifdef DIO0_PIN
/**
 * Interrupt generated DIO0 (frame ready to download)
 */
void PININT3_IRQHandler (void) {
	// Clear interrupt
	LPC_PIN_INT->IST = 1<<3;
	interrupt_source = DIO0_INTERRUPT;
}
#endif

void WKT_IRQHandler(void)
{
	LPC_WKT->CTRL |= 0x02;			/* clear interrupt flag */
	interrupt_source = WKT_INTERRUPT;
}
