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


#include "config.h"
#include "switchmatrix.h"
#include "myuart.h"
#include "sleep.h"
#include "print_util.h"
#include "parse_util.h"
#include "cmd.h"
#include "led.h"
#include "flags.h"
#include "params.h"
#include "delay.h"
#include "eeprom.h"
#include "frame_buffer.h"
#include "mac.h"
#include "iap_driver.h"

#include "lpc8xx_pmu.h"
#include "onewire.h"
#include "ds18b20.h"
#include "ws2812b.h"
#include "gps.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Coarse clock to keep track of time (for link loss etc) 1/100s intervals.
static uint32_t last_frame_time;

params_union_type params_union;

// Use to ID each sleep update packet. No need to init (saves 4 bytes).
static uint32_t sleep_counter = 0;

// When in deepsleep or power down this lets us know which wake event occurred
typedef enum {
	NO_INTERRUPT = 0,
	WKT_INTERRUPT = 1,
	UART_INTERRUPT = 2,
	EVENT_COUNTER_INTERRUPT = 3,
	PIEZO_SENSOR_INTERRUPT = 4,
	DIO0_INTERRUPT = 5
} interrupt_source_type;
volatile interrupt_source_type interrupt_source;

// Radio packet frame buffers
frame_buffer_type tx_buffer;
frame_buffer_type rx_buffer;

static uint32_t last_gps_report_t = 0;
uint8_t wake_list[4] = {0,0,0,0};

#ifdef FEATURE_EVENT_COUNTER
	volatile uint32_t event_counter = 0;
	volatile uint32_t event_time = 0;
#endif


	volatile uint32_t systick_counter = 0;
	void SysTick_Handler(void) {
		systick_counter++; // every 10ms
	}

/**
 * Print error code 'code' while executing command 'cmd' to UART.
 * @param cmd  Command that generated the error
 * @param code Error code (ref err.h)
 */
void report_error (uint8_t cmd, int32_t code) {
	if (code<0) code = -code;
	tfp_printf("e %c %x\r\n",cmd,code);
}


/**
 * Show system settings etc to UART
 */
void displayStatus () {

#ifdef RADIO_RFM98
	tfp_printf("; radio=RFM98\r\n");
#endif
#ifdef RADIO_RFM69
	tfp_printf("; radio=RFM69\r\n");
#endif


#ifdef BOARD_LPC812_RFM98_V1
//	int ii;
//	for (ii = 0; ii < 20; ii++) {
//		tfp_printf("; reg[%d]=%x\r\n", ii, rfm_register_read(ii));
//	}
#endif

	tfp_printf ("; firmware=%s\r\n", VERSION);

	// List optional features enabled
#ifdef FEATURE_EVENT_COUNTER
	tfp_printf ("; feature EVENT_COUNTER\r\n");
#endif
#ifdef FEATURE_DS18B20
	tfp_printf ("; feature DS18B20\r\n");
#endif
#ifdef FEATURE_WS2812B
	tfp_printf ("; feature WS2812B\r\n");
#endif
#ifdef FEATURE_GPS_ON_USART1
	tfp_printf ("; feature GNSS\r\n");
#endif

	tfp_printf ("; mode=%x\r\n",params_union.params.operating_mode);
	tfp_printf ("; node_addr=%x\r\n",params_union.params.node_addr);
	tfp_printf ("; poll_interval=%x\r\n",params_union.params.poll_interval);
	tfp_printf ("; listen_period=%x\r\n",params_union.params.listen_period_cs);
	tfp_printf ("; link_loss_timeout=%x\r\n",params_union.params.link_loss_timeout_s);
	tfp_printf ("; eeprom_addr=%x\r\n",eeprom_get_addr());
	tfp_printf ("; systick_counter=%x\r\n", systick_counter);
#ifdef FEATURE_EVENT_COUNTER
	tfp_printf ("; event_counter=%x\r\n", event_counter);
#endif
	tfp_printf ("; ds18b20_mode=%x\r\n",params_union.params.ds18b20_mode);
	tfp_printf ("; low_battery_v=%d\r\n",params_union.params.low_battery_v);
	tfp_printf ("; min_battery_v=%d\r\n",params_union.params.min_battery_v);

#ifdef FEATURE_GPS_ON_USART1
	gps_report_status();
#endif


	uint32_t unique_id[4];
	iap_read_unique_id(unique_id);
	tfp_printf ("; mcu_unique_id=%x %x %x %x\r\n", unique_id[0], unique_id[1],
			unique_id[2], unique_id[3] );

#ifdef FEATURE_WATCHDOG_TIMER
	tfp_printf("; watchdog_timer=%x\r\n", LPC_WWDT->TV);
#endif

	tfp_printf("; supply_voltage_mV=%d\r\n",readBattery());


	int32_t t;

	//t = rfm69_temperature();
	//tfp_printf("; rfm69_temperature=%d\r\n", t);

#ifdef FEATURE_DS18B20
	t = (1000*ds18b20_temperature_read())/16;
	tfp_printf("; ds18b20_temperature_mC=%d\r\n", t);
#endif

	tfp_printf("; END\r\n");

}




// To facilitate tfp_printf()
void myputc (void *p, char c) {
	MyUARTSendByte(c);
}

/**
 * Save parameter/settings block to flash. Copy to RAM
 * before writing to flash.
 */
void eeprom_params_save (void) {
    uint8_t tmpbuf[64];
    memcpy(tmpbuf,params_union.params_buffer,64);
    eeprom_write(tmpbuf);
}


void transmit_status_packet() {


#ifdef RFM98
	// TODO:
	// This seems to be necessary after a module SLEEP. Why?
	//rfm98_lora_mode(RFM98_OPMODE_LoRa_RXCONTINUOUS);
	rfm98_lora_mode(RFM98_OPMODE_LoRa_FSRX);
	//delayMilliseconds(5);
#endif

	// Battery in 0.1V units
	uint8_t battery_v = readBattery_dV();

	if (battery_v >= params_union.params.min_battery_v) {
		tx_buffer.header.to_addr = 0;
		tx_buffer.header.msg_type = 'z';
		tx_buffer.payload[0] = sleep_counter++;
#ifdef FEATURE_EVENT_COUNTER
		tx_buffer.payload[1] = event_counter;
#endif
		tx_buffer.payload[2] = battery_v;

#ifdef FEATURE_DS18B20
		if (params_union.params.ds18b20_mode != 0) {
			// Pullup resistor on DS18B20 data pin PIO0_14
			LPC_IOCON->PIO0_14 = (0x2 << 3);
			ow_init(0, DS18B20_PIN);
			delayMilliseconds(20);
			int32_t temperature = ds18b20_temperature_read();
			tfp_printf("; ds18b20_temperature_mC=%d\r\n",
					(temperature * 1000) / 16);
			tx_buffer.payload[3] = temperature >> 8;
			tx_buffer.payload[4] = temperature & 0xff;
		}
#endif

		// Transmit 'z' periodic wake packet
		//rfm_config(); // why is this needed?
		ledOn();
		rfm_frame_tx(tx_buffer.buffer, 8);
		ledOff();

		//debug("temp=%d", rfm98_temperature());

	} else {
		tfp_printf("; bat too low to tx");
	}
}

/**
 * Note: this currently requires Watchdog timer operating.
 */
void listen_for_status_response () {

	int start_time = LPC_WWDT->TV;

	// Listen for a short period for a response
#ifdef RADIO_RFM9x
	// Set timeout register

	rfm98_lora_mode(RFM98_OPMODE_LoRa_CAD);
	rfm_wait_for_bit_high(RFM98_IRQFLAGS,RFM98_IRQFLAGS_CadDone);

	// Wait max 16 WWDT ticks for RSSI (indication of incoming packet)
	// and if detected wait a further 500 ticks for the packet to arrive.
	int end_time = start_time - 500;
	int cad=0;
	while (LPC_WWDT->TV > end_time) {

		rfm98_lora_mode(RFM98_OPMODE_LoRa_CAD);
		rfm_wait_for_bit_high(RFM98_IRQFLAGS,RFM98_IRQFLAGS_CadDone);
		rfm_register_write(RFM98_IRQFLAGS, RFM98_IRQFLAGS_CadDone);
		cad++;

		if (rfm_register_read(RFM98_IRQFLAGS) & RFM98_IRQFLAGS_CadDetected) {

			rfm98_lora_mode(RFM98_OPMODE_LoRa_RXCONTINUOUS);

			debug("CAD after %d WDT cycles, %d CAD ops", (start_time - LPC_WWDT->TV) , cad);

			// Wait for full frame
			end_time = start_time - 26000;
			while (LPC_WWDT->TV > end_time) {
				if (IS_PACKET_READY()) {
					debug("frame detected after %d WDT cycles", (start_time - LPC_WWDT->TV));
					break;
				}
			}

			// clear CAD IRQ
			rfm_register_write(RFM98_IRQFLAGS, RFM98_IRQFLAGS_CadDetected);

			break;

		}

	}

#else
	rfm69_mode(RFM69_OPMODE_Mode_RX);

	// Wait max 16 WWDT ticks for RSSI (indication of incoming packet)
	// and if detected wait a further 500 ticks for the packet to arrive.
	int end_time = start_time - 16;
	while (LPC_WWDT->TV > end_time) {
		if (IS_PACKET_READY()) { // ??
			end_time = start_time - 500;
			while (LPC_WWDT->TV > end_time) {
				if (IS_PACKET_READY()) {
					debug("frame detected after %d WDT cycles", (start_time - LPC_WWDT->TV));
					break;
				}
			}
			break;
		}
	}
#endif

}


int main(void) {

	// Enable clock to MCU subsystems needed in this application in one step. Saves bytes.
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


    SysTick_Config( SYSTICK_DELAY );

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
#endif
#endif




	// Reset GPIO
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
	LPC_SYSCON->PRESETCTRL |= (0x1<<10);

	// Experimental: turn off clock to other modules. Saves ~50uA.
	// Turn off clock to GPIO
	//LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<6);
	// Turn off clock to WKT
	//LPC_SY
	//SCON->SYSAHBCLKCTRL &= ~(1<<9);
	// Turn off clock to WDT
	//LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<17);

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
		case 0x18044043:
			params_union.params.node_addr = 0x40;
			break;
		case 0x1902E033:
			params_union.params.node_addr = 0x41;
			break;
		case 0x5034039: // V1B hack required
			params_union.params.node_addr = 0x42;
			break;
		case 0x05034043:
			params_union.params.node_addr = 0x43;
			break;
		case 0x5046049:
			params_union.params.node_addr = 0x44;
			break;
		case 0x5046043:
			params_union.params.node_addr = 0x45;
			break;


		case 0x19017037:
			params_union.params.node_addr = 0x51; // RFM98
			break;

		case 0x1902e034:
			params_union.params.node_addr = 0x52; // RFM98
			break;

		case 0x1900c037:
			params_union.params.node_addr = 0x53; // RFM98
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
	// FREQSEL (bits 8:5) = 0x1 : 0.6MHz  +/- 40%
	// DIVSEL (bits 4:0) = 0x1F : divide by 64
	// Watchdog timer: ~ 10kHz
    LPC_SYSCON->WDTOSCCTRL = (0x1<<5)
    						|0x1F;
    LPC_WWDT->TC = params_union.params.link_loss_timeout_s * WWDT_CLOCK_SPEED_HZ;
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

#ifdef DIO0_PIN
	// If RFM DIO0 output line is available configure PIO pin for input. In RFM69 packet
	// mode DIO0 high signifies frame is ready to read from FIFO.
	//LPC_GPIO_PORT->DIR0 &= ~(1<<DIO0_PIN);

	// Pulldown resistor on PIO0_6 (pin DIO0)
	// TODO: PIO0_6 IOCON hard coded
	LPC_IOCON->PIO0_6=(0x1<<3);

	//LPC_GPIO_PORT->DIR0 |= (1<<DIO0_PIN);
	//LPC_GPIO_PORT->CLR0 |= (1<<DIO0_PIN);

	// Enable interrupt when DIO0 goes high.
	//LPC_SYSCON->PINTSEL[3] = DIO0_PIN;
	//LPC_PIN_INT->ISEL &= ~(0x1<<3);	/* Edge trigger */
	//LPC_PIN_INT->IENR |= (0x1<<3);	/* Rising edge */
	//NVIC_EnableIRQ((IRQn_Type)(PININT3_IRQn));
#endif

#ifdef DIO1_PIN
	// Pulldown resistor on PIO0_14 (pin DIO1)
	// TODO: PIO0_14 IOCON hard coded
	LPC_IOCON->PIO0_14=(0x1<<3);
#endif


	// Absolute value of the RSSI in dBm, 0.5dB steps.
	// RSSI_dBm = -rssi/2
	// RFM9x: RSSI in dBm
	int rssi;
	uint8_t modem_state,prev_modem_state;

	// Used in UART commands
	uint8_t *args[8];




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


#ifdef FEATURE_EVENT_COUNTER
	// Set TIPBUCKET_PIN as input
	LPC_GPIO_PORT->DIR0 &= ~(1<<EVENT_COUNTER_PIN);

	// Pulldown resistor on PIO0_16 (tip bucket)
	LPC_IOCON->PIO0_16=(0x1<<3);

	// Set interrupt on this pin.
	LPC_SYSCON->PINTSEL[1] = EVENT_COUNTER_PIN;
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


	//
	// Test radio module
	//
#ifdef BOARD_LPC812_RFM98_V1
#else
	if (rfm69_test() != 0) {
		// Error communicating with RFM69: 4 blinks
		ledBlink(4);
		test_result |= 1<<0;
	}
#endif


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
	tfp_printf("k %x\r\n",test_result);

	rfm_config();

	//
	// Main program loop
	//
	while (1) {

#ifdef FEATURE_EVENT_COUNTER
		if ( (event_time != 0) && (systick_counter - event_time > 10) ) {
			tfp_printf("a %x\r\n",event_counter);
			MyUARTSendDrain();
			//event_counter = 0;
			event_time= 0;
		}
#endif

		// TODO: can we avoid calling rfm69_mode() on every iteration?
		// TODO: use macro to test flags
		if (params_union.params.operating_mode == MODE_AWAKE) {
#ifdef RADIO_RFM9x
			rfm98_lora_mode(RFM98_OPMODE_LoRa_RXCONTINUOUS);
#else
			rfm69_mode(RFM69_OPMODE_Mode_RX);
#endif
		}

#ifdef FEATURE_DEEPSLEEP
		// Test for MODE_OFF or MODE_LOW_POWER_POLL (LSB==0 for those two modes)
		if ( params_union.params.operating_mode == MODE_LOW_POWER_POLL) {

			tfp_printf("; z\r\n");

			uint8_t battery_v = readBattery_dV();

			// Set radio in SLEEP mode (sub microamp current use). Register retention.
#ifdef RADIO_RFM9x
			rfm98_lora_mode(RFM98_OPMODE_LoRa_SLEEP);
			//rfm_register_write(RFM98_OPMODE,0); // TODO experiment
#else
			rfm69_mode(RFM69_OPMODE_Mode_SLEEP);
#endif

			// Make sure 'z' message fully transmitted before sleeping
			MyUARTSendDrain();

			// Reset source of wake event
			interrupt_source = 0;

			sleep_set_pins_for_powerdown();

			// Setup power management registers so that WFI causes DEEPSLEEP
			prepareForPowerDown();

			// Writing into WKT counter automatically starts wakeup timer. WKT timer
			// is driven by 10kHz low power oscillator. However this is +/- 40%.
			// Finding 7.5kHz closer to the mark.
			uint32_t wakeup_time = 9000  * params_union.params.poll_interval;

			// If battery low then sleep for extended period
			if (battery_v <= params_union.params.low_battery_v) {;
				wakeup_time *= 8;
			}
			LPC_WKT->COUNT = wakeup_time ;

			// DeepSleep until WKT interrupt (or PIN interrupt)
			__WFI();



			// WKT in 100us increments, want sleep_clock in 10ms increments
			systick_counter += (wakeup_time - LPC_WKT->COUNT)/100;


			sleep_set_pins_for_wake();

			// Allow time for clocks to stabilise after wake
			// TODO: can we use WKT and WFI?
			//delay(20000);
			//delayMilliseconds() didn't work!
			//delayMilliseconds(10);

			if (interrupt_source == UART_INTERRUPT) {
				tfp_printf("; UART interrupt, awake\r\n");
				// Probably crud in UART rx buffer : clear it.
				MyUARTBufReset();
				if (params_union.params.operating_mode == MODE_LOW_POWER_POLL) {
					params_union.params.operating_mode = MODE_AWAKE;
				}
			}

			// Undo DeepSleep/PowerDown flag so that next WFI goes into regular sleep.
			SCB->SCR &= ~NVIC_LP_SLEEPDEEP;

			// Indicator to host there is a short time window to issue command
			tfp_printf("z\r\n");

		}
#else
		// If not using DEEPSLEEP, use regular SLEEP mode until next interrupt arrives
		if ( (flags&0xf) != MODE_AWAKE ) {
			__WFI();
		}
#endif


		// If in MODE_LOW_POWER_POLL send status packet and listen for response
		if ( params_union.params.operating_mode == MODE_LOW_POWER_POLL) {
#ifdef FEATURE_VBAT
			uint8_t battery_v = readBattery_dV();
			if (battery_v >= params_union.params.min_battery_v) {
				transmit_status_packet();
				listen_for_status_response();
			} else {
				tfp_printf("; battery too low to tx/rx\r\n");
				report_error((uint8_t)'z', E_BATTERY_V_TOO_LOW);
			}
#else
			transmit_status_packet();
			listen_for_status_response();
#endif
		}

#ifdef RADIO_RFM9x
		if (params_union.params.operating_mode != MODE_AWAKE) {
			modem_state = rfm_register_read(RFM98_MODEMSTAT);
			if (modem_state != prev_modem_state) {
				debug("MODEM %x",modem_state);
				prev_modem_state=modem_state;
			}
		}
#endif

		if ((params_union.params.operating_mode != MODE_RADIO_OFF) && IS_PACKET_READY()) {

#ifdef RADIO_RFM9x
			rssi = rfm98_last_packet_rssi();
#endif

			// Yes, frame ready to be read from FIFO
			ledOn();

#ifdef RADIO_RFM9x
			int frame_len = rfm98_frame_rx(rx_buffer.buffer,RXTX_BUFFER_SIZE);
#else
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,RXTX_BUFFER_SIZE);
#endif
			ledOff();

			last_frame_time = systick_counter;

			//
			// Experimental wake list. Node 0 reserved for empty slot (so cannot wake node 0).
			//
			{
				int ii;
				for (ii = 0; ii < 4; ii++) {
					if ( (wake_list[ii]!=0) && (rx_buffer.header.from_addr == wake_list[ii]) ) {
						// Wake with remote command "M 3" (mode 3 = wake with radio on)
						tx_buffer.header.to_addr = rx_buffer.header.from_addr;
						tx_buffer.header.msg_type = PKT_REMOTE_CMD;
						tx_buffer.payload[0]='M';
						tx_buffer.payload[1]=' ';
						tx_buffer.payload[2]='3';

						// Send it twice: CAD seems to prevent reception of first packet
						// TODO: not sure why that's the case. TODO: sent a NOP first (shorter).
						rfm_frame_tx(tx_buffer.buffer, 3+3);
						delayMilliseconds(10);
						rfm_frame_tx(tx_buffer.buffer, 3+3);
						wake_list[ii]=0;

						debug("wake sent to %x", rx_buffer.header.from_addr);

					}
				}
			}

#ifdef RADIO_RFM9x
			{
			int ii;
			tfp_printf("; FRAME: [ ");
			for (ii = 0; ii < frame_len; ii++) {
				tfp_printf(" %x", rx_buffer.buffer[ii]);
			}
			tfp_printf (" ] %d %d %d\r\n",
					rfm98_last_packet_rssi(),
					rfm98_last_packet_snr(),
					rfm98_last_packet_crc_ok()
					);
			}
#endif

#ifdef FEATURE_WATCHDOG_TIMER
			// Feed watchdog whenever a packet is successfully received.
			LPC_WWDT->FEED = 0xAA;
			LPC_WWDT->FEED = 0x55;
#endif


			// 0xff is the broadcast address
			if ( rx_buffer.header.to_addr == 0xff
					|| rx_buffer.header.to_addr == tx_buffer.header.from_addr) {

				// bit 7: ack request
				if (rx_buffer.header.msg_type & 0x80) {
					tx_buffer.header.msg_type = PKT_ACK;
					tx_buffer.payload[0] = rssi;
					rfm69_frame_tx(tx_buffer.buffer, 4);
				}


				// This frame is for us! Examine messageType field for appropriate action.

				switch (rx_buffer.header.msg_type & 0x7F) {

				case PKT_NOP:
					break;

				// Experimental remote packet transmit / relay
				case PKT_RELAY : {
					int payload_len = frame_len - 3;
					memcpy(tx_buffer.payload,rx_buffer.payload,payload_len);
					rfm69_frame_tx(tx_buffer.buffer, payload_len+3);
					break;
				}

				// Remote command execute
				case PKT_REMOTE_CMD : {
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

					tfp_printf ("; remote cmd from %x\r\n",tx_buffer.header.from_addr);

					// Echo remote command to UART, copy remote command to UART buffer and
					// trigger UART command parsing.
					//MyUARTSendStringZ("d ");
					int payload_len = frame_len - 3;
					uint8_t *uart_buf = MyUARTGetBuf();
					memcpy(uart_buf,rx_buffer.payload,payload_len);
					uart_buf[payload_len] = 0; // zero terminate buffer
					//MyUARTSendStringZ((char *)uart_buf);
					//MyUARTSendCRLF();
					tfp_printf ("d %s\r\n",uart_buf);
					MyUARTSetBufFlags(UART_BUF_FLAG_EOL);
					break;
				}

				case PKT_VERSION_REQUEST :
				{
					tfp_printf("; received version request from %x\r\n",rx_buffer.header.from_addr);
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = PKT_VERSION_RESPONSE;
					char *s = VERSION;
					int i=0;
					while (*s != 0) {
						tx_buffer.payload[i++] = *s++;
					}
					rfm69_frame_tx(tx_buffer.buffer, 3+i);
					break;
				}

				case PKT_MCU_ID_REQUEST:
				{
					tfp_printf("; received MCUID request from %x\r\n",rx_buffer.header.from_addr);
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = PKT_MCU_ID_RESPONSE;
					int i;
					uint32_t *p;
					for (i = 0; i < 4; i++) {
						p = &tx_buffer.payload[i*4];
						*p = mcu_unique_id[i];
					}
					rfm69_frame_tx(tx_buffer.buffer, 3+16);
					break;
				}

				// Start ping-pong test
				case PKT_START_PINGPONG :
				{
					tfp_printf("; received ping from %x\r\n",rx_buffer.header.from_addr);
					//MyUARTPrintHex(rx_buffer.header.from_addr);
					//MyUARTSendCRLF();
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
				case PKT_LOCATION_REPORT :
				{
					tfp_printf("; received node query request from %x\r\n",rx_buffer.header.from_addr);
					gps_send_status(rx_buffer.header.from_addr);
					break;
				}
#endif
				case PKT_STATUS_REPORT :
				{
					debug ("; received status request from %x\r\n",rx_buffer.header.from_addr);

					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = PKT_STATUS_RESPONSE; // node status response
					tx_buffer.payload[0] = readBattery_dV();
					tx_buffer.payload[1] = rssi;
#ifdef FEATURE_EVENT_COUNTER
					tx_buffer.payload[2] = ((event_counter>>8)&0xff);
					tx_buffer.payload[3] = (event_counter&0xff);
#endif

					int32_t temperature = ds18b20_temperature_read();
					tx_buffer.payload[4] = temperature>>8;
					tx_buffer.payload[5] = temperature&0xff;
					rfm_frame_tx(tx_buffer.buffer, 3+6);
					break;
				}


				// Remote request to LED blink
				case PKT_LED_BLINK : {
					//tx_buffer.header.to_addr = from_addr;
					//is there any need for a reply? Just use ACK mechanism instead?
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'u';
					rfm69_frame_tx(tx_buffer.buffer, 3);
					ledBlink(3);
					break;
				}

				// Remote RFM69 register read (0x58)
				case PKT_RADIO_REG_READ : {
					uint8_t base_addr = rx_buffer.payload[0];
					uint8_t read_len = rx_buffer.payload[1];
					if (read_len>16) read_len = 16;
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'x';
					tx_buffer.payload[0] = base_addr;
					int i;
					for (i = 0; i < read_len; i++) {
						tx_buffer.payload[i+1] = rfm_register_read(base_addr+i);
					}
					rfm_frame_tx(tx_buffer.buffer, read_len+4);
					break;
				}

				// Remote RFM69 register write
				case PKT_RADIO_REG_WRITE : {
					uint8_t base_addr = rx_buffer.payload[0];
					uint8_t write_len = frame_len - 4;
					if (write_len > 16) write_len = 16;
					int i;
					for (i = 0; i < write_len; i++) {
						rfm_register_write(base_addr+i,rx_buffer.payload[i+1]);
					}
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'y';
					rfm69_frame_tx(tx_buffer.buffer, 3);
					break;
				}


				// Experimental write to MCU memory (0x3E)
				case PKT_MEM_WRITE : {

					// Return 32bit value from memory at payload+0
					// Note address and result is LSB first (little endian)
					uint32_t **mem_addr;
					mem_addr = (uint32_t **)&rx_buffer.payload[0];

					int len = (frame_len - 3 - 4)/4;
					tfp_printf("; memory write request at %x, len=%d\r\n", *mem_addr,len);

					memcpy(*mem_addr, rx_buffer.payload+4, len*4);

					break;
				}

				// Experimental read from MCU memory (0x3C)
				case PKT_MEM_READ_REQUEST : {
					// Return 32bit value from memory at payload+0
					// Note address and result is LSB first (little endian)
					uint32_t **mem_addr;
					mem_addr = (uint32_t *)&rx_buffer.payload[0];
					tfp_printf("; memory read request at %x\r\n", *mem_addr);

					tfp_printf("; value of location %x = %x\r\n", *mem_addr, **mem_addr);
					// First 4 bytes of return is base address
					*(uint32_t *)tx_buffer.payload = *mem_addr;

					//*(uint32_t *)(tx_buffer.payload+4) = **mem_addr;

					int len = 1;
					if (frame_len == 8) {
						len = rx_buffer.payload[4];
						if (len > 8) len = 8;
					}

					memcpy(tx_buffer.payload+4, *mem_addr, len*4);

					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = PKT_MEM_READ_RESPONSE;
					rfm69_frame_tx(tx_buffer.buffer, sizeof(frame_header_type)+4+4*len);
					break;
				}

				// Experimental execute from memory (!?!)
				case PKT_EXEC_MEM : {
					// Execute function at memory location payload+0
					// Note on ARM Cortex M devices LSB always 1 for Thumb instruction set.
					uint32_t **mem_addr;
					mem_addr = rx_buffer.payload;
					typedef void (*E)(void);
					E e_entry = (E)*mem_addr;
					e_entry();
					break;
				}

				case PKG_OTA_ENTER_BOOTLOADER : {
					// Can never exit from here: only reset when complete
					ota_bootloader(params_union.params.node_addr);
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
			if (params_union.params.operating_mode == MODE_LOW_POWER_POLL) {
				params_union.params.operating_mode = MODE_AWAKE;
			}
#endif

			MyUARTSendCRLF();




			uint8_t *uart_buf = MyUARTGetBuf();

			// Split command line into parameters (separated by spaces)
			argc = 1;
			args[0] = uart_buf;
			uint8_t quote_state=0;
			while (*uart_buf != 0) {
				if (*uart_buf == '"') {
					// toggle quote_state
					quote_state = quote_state==0? 1:0;
				}
				if (*uart_buf == ' ' && quote_state == 0) {
					*uart_buf = 0;

					args[argc++] = uart_buf+1;
				}
				uart_buf++;
			}

			int ii;
			for (ii = 0; ii < argc; ii++) {
				debug("arg[%d]=%s",ii,args[ii]);
			}

			// TODO: using an array of functions may be more space efficient than
			// switch statement.

			switch (*args[0]) {

			// Enter bootloader mode
			case 'A' : {
				ota_bootloader(params_union.params.node_addr);
				break;
			}

			// Set API UART bitrate
			case 'B' : {
				cmd_set_uart_speed (argc, args);
				break;
			}

			// Reset RFM69 with default configuration
			case 'C' :
			{
				rfm_config();
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

			// Query node status
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

			case 'H' : {
				cmd_wake_node (argc, args);
				break;
			}

			// Display MCU unique ID
			case 'I' : {
				tfp_printf("i %x %x %x %x\r\n",mcu_unique_id[0],mcu_unique_id[1],
						mcu_unique_id[2], mcu_unique_id[3]);
				break;
			}

			// Set link loss reset timeout
			case 'J' : {
				if (argc == 1) {
					tfp_printf("j %x %x %x\r\n", params_union.params.link_loss_timeout_s,
							LPC_WWDT->TV, LPC_WWDT->TC);
					break;
				}
				params_union.params.link_loss_timeout_s = parse_hex(args[1]);
				LPC_WWDT->TC = params_union.params.link_loss_timeout_s * 2000;
			    // Watchdog feed sequence
			    LPC_WWDT->FEED = 0xAA;
			    LPC_WWDT->FEED = 0x55;
				break;
			}

			// Packet transmit test
			case 'K' : {
				cmd_tx_test(argc, args);
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
					tfp_printf("m %x\r\n",params_union.params.operating_mode);
					break;
				}
				params_union.params.operating_mode = parse_hex(args[1]);
				if (argc == 3) {
					if (args[2][0]=='S') {
						tfp_printf("; ModeSaveAndReset\r\n");
						eeprom_params_save();
						MyUARTSendDrain();
						NVIC_SystemReset();
					}
				}
				break;
			}


			// Set or report node address
			case 'N' :
			{
				if (argc == 1) {
					tfp_printf("n %x\r\n", params_union.params.node_addr);
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
			case 'P' : {

				if (argc == 2) {
					uint32_t param_index = parse_hex(args[1]);
					tfp_printf("%x\r\n",params_union.params_buffer[param_index]);
					break;
				}

				if (argc != 3) {
					return E_WRONG_ARGC;
				}

				uint32_t param_index = parse_hex(args[1]);
				uint32_t param_value = parse_hex(args[2]);
				params_union.params_buffer[param_index] = param_value;
				break;
			}

			// Reset MCU
			case 'Q' : {
				// Report reset for reason=1 (explicit reset command)
				tfp_printf("q 1\r\n");
				MyUARTSendDrain();
				NVIC_SystemReset();
				// no need for break
			}


			// Read RFM register
			case 'R' : {
				int regAddr = parse_hex(args[1]);
				tfp_printf("r %x %x\r\n",regAddr,rfm_register_read(regAddr));
				break;
			}

			// Save current settings in flash
			case 'S' :
			{
				eeprom_params_save();
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
					rfm_register_write(regAddr++,regValue);
				}
				break;
			}

			// Read battery. Return in both x response message and also
			// human friendly value in decimal in info message
			case 'X' :  {
				tfp_printf("; battery_mV=%d\r\n",readBattery());
				break;
			}

			case 'Y' : {
				tfp_printf("y %x\r\n", ds18b20_temperature_read());
				break;
			}

#ifdef FEATURE_WS2812B
			case 'Z' : {
				int rgb = parse_hex(args[1]);
				ws2812b_init();
				ws2812b_reset();
				ws2812b_bitbang(rgb);
				break;
			}
#endif

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
			//tfp_printf("; last_frame_age=%d\r\n",last_frame_age);
			MyUARTSendDrain();
			if (params_union.params.link_loss_timeout_s != 0
					&& (last_frame_age > params_union.params.link_loss_timeout_s*100)) {
				// Report MCU reset (reason=2 link loss timeout)
				tfp_printf("q 2\r\n");
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
RAM_FUNC
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

#ifdef FEATURE_EVENT_COUNTER

/**
 * Interrupt generated by event / tip bucket pin
 */
void PININT1_IRQHandler (void) {

	// Clear interrupt
	LPC_PIN_INT->IST = 1<<1;


	// Printing event_counter in here results in very strange behaviour.  Probably due to
	// using MyUART inside ISR.

	if (event_time == 0) {
		event_counter++;
		event_time = systick_counter;
	}

	interrupt_source = EVENT_COUNTER_INTERRUPT;
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
