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
#include "frame_buffer.h"

#include "lpc8xx_pmu.h"
#include "ds18b20.h"

#define SYSTICK_DELAY		(SystemCoreClock/100)

// Address of this node
//int8_t node_addr = DEFAULT_NODE_ADDR;

// Current location string specified by boat firmware. Format TBD.
uint8_t current_loc[32];

// Various radio controller flags (done as one 32 bit register so as to
// reduce code size and SRAM requirements).
// TODO: now that we've moved to LPC812, this sort of extreme optimization
// detracts from code readability. Consider a struct of params instead.
volatile uint32_t flags =
		MODE_LOW_POWER_POLL
		//MODE_AWAKE
		//| (0x4<<8) // poll interval 500ms x 2^(3+1) = 8s
		| (0x6<<8) // poll interval 500ms x 2^(6+1) = ?s
		;

// Coarse clock to keep track of time (for link loss etc) 1/100s intervals.
uint32_t last_frame_time;
uint32_t link_loss_timeout = DEFAULT_LINK_LOSS_TIMEOUT; // 10ms increments

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

frame_buffer_type tx_buffer;
frame_buffer_type rx_buffer;

//frame_send_buffer.header.from_addr = node_addr;


#ifdef FEATURE_EVENT_COUNTER
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

/**
 * UART RXD on SOIC package pin 19
 * UART TXD on SOIC package pin 5
 * ACMP_2 on SOIC package pin 12
 */
void SwitchMatrix_Acmp_Init()
{
    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;
    /* ACMP_O */
    LPC_SWM->PINASSIGN8 = 0xffff0dffUL;

    /* Pin Assign 1 bit Configuration */
    /* ACMP_I2 */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb1UL;


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
void SwitchMatrix_WithSWD_Init()
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
 * SwitchMatrix configuration to utilized RESET and SWD lines for SPI use
 * (current implementation uses bitbanging on GPIO, so configure these
 * pins for GPIO use. When this is called any debugging session in progress
 * will be terminated, and it will not be possible to reflash the device
 * unless SWD is restored (through UART command) or power cycle and reflash
 * during the ~10second delay on boot when SWD is still active. Alternatively
 * hold ISP entry pin (PIO0_0?) low and power cycle.
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

#ifdef BOARD_V1B_HACK
/**
 * Hack to facilitate PCB v1 bug where via on RXD at PIO0_0 (package pin 19)
 * line touches Vdd rendering it useless (always high). There is an easy work around using
 * a knife to separate the via from Vdd, but in one case the was not done before
 * the LPC812 package was soldered in pace. The via contact is under the LPC812, so
 * making the cut was no longer possible. Rather
 * than scrap the board, the RXD line from the board UART connecter was routed to
 * PIO0_11 instead (package pin 8). So in this case want to route LPC812 UART RXD
 * to there. AFAIK, it is not possible to program this board using ISP mode. SWD
 * must be used. I'm going to call this v1b of the board. JD 20141117.
 */
void SwitchMatrix_LPC812_PCB1b_Init()
{
	// UART0 TXD to PIO0_4 (pin package 5). This is location for ISP programming
	// UART0 RXD rerouted to PIO0_11 (pin package 11).
    LPC_SWM->PINASSIGN0 = 0xffff0b04UL;

    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;
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
	MyUARTSendByte(' ');
	MyUARTPrintHex(code);
	MyUARTSendCRLF();
}

/**
 * Use analog comparitor with internal reference to find approx battery voltage
 */
int readBattery () {

	//
	// Analog comparator configure
	//

	// Power to comparator. Use of comparator requires BOD. [Why?]
	LPC_SYSCON->PDRUNCFG &= ~( (0x1 << 15) | (0x1 << 3) );

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<19); // Analog comparator

	// Analog comparator reset
	LPC_SYSCON->PRESETCTRL &= ~(0x1 << 12);
	LPC_SYSCON->PRESETCTRL |= (0x1 << 12);


	// Measure Vdd relative to bandgap
	LPC_CMP->CTRL =  (0x1 << 3) // rising edge
			| (0x6 << 8)  // bandgap
			| (0x0 << 11) // - of cmp to voltage ladder
			;

	int k;
	for (k = 0; k <32; k++) {
		LPC_CMP->LAD = 1 | (k<<1);
		__WFI(); // allow to settle (15us on change, 30us on powerup
		if ( ! (LPC_CMP->CTRL & (1<<21))) {
			return 27900/k; // 900mV*31/k
		}
	}

	return 27900/32;
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
    		| (1<<17) // Watchdog timer (note: it may not be necessary to have on all the time)
    		;


#ifdef FEATURE_SYSTICK
    SysTick_Config( SYSTICK_DELAY );
#endif

	/*
	 * LPC8xx features a SwitchMatrix which allows most functions to be mapped to most pins.
	 * This setups up the pins in a way that's convenient for our physical circuit layout.
	 * In the case of the LPC810, use of SPI (which necessitates loss of SWD functionality)
	 * is delayed to allow opportunity to reprogram device via SWD.
	 */

#ifdef LPC810
	SwitchMatrix_WithSWD_Init();
#endif

#ifdef LPC812
#ifdef BOARD_V1B_HACK
	SwitchMatrix_LPC812_PCB1b_Init();
#else
	SwitchMatrix_Init();
	//SwitchMatrix_Acmp_Init();
#endif
#endif

	// Reset GPIO
	//LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
	//LPC_SYSCON->PRESETCTRL |= (0x1<<10);
	lpc8xx_peripheral_reset(10);

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


#ifdef FEATURE_WATCHDOG_TIMER
	//
    // Watchdog configuration
	//

	// Power to WDT
	LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);

	// Setup watchdog oscillator frequency
    /* Freq = 0.5Mhz, div_sel is 0x1F, divided by 64. WDT_OSC should be 7.8125khz */
    LPC_SYSCON->WDTOSCCTRL = (0x1<<5)|0x1F;
    LPC_WWDT->TC = DEFAULT_WATCHDOG_TIMEOUT;
    LPC_WWDT->MOD = (1<<0) // WDEN enable watchdog
    			| (1<<1); // WDRESET : enable watchdog to reset on timeout
    // Watchdog feed sequence
    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    /* Make sure feed sequence executed properly */
    //loopDelay(1000);
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
	loopDelay(20000);
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

	uint8_t rssi;

	//uint8_t *cmdbuf;
	uint8_t *args[8];

	// Radio frame receive buffer
	//uint8_t frxbuf[66];
	//uint8_t frame_len;

	// Use to ID each sleep ping packet. No need to init (saves 4 bytes).
	uint32_t sleep_counter = 0;

	int argc;

	tx_buffer.header.from_addr = DEFAULT_NODE_ADDR;

#ifdef FEATURE_UART_INTERRUPT
	// Experimental wake on activity on UART RXD (RXD is normally shared with PIO0_0)
	LPC_SYSCON->PINTSEL[0] = UART_RXD_PIN; // PIO0_0 aka RXD
	LPC_PIN_INT->ISEL &= ~(0x1<<UART_RXD_PIN);	/* Edge trigger */
	LPC_PIN_INT->IENR |= (0x1<<UART_RXD_PIN);	/* Rising edge */
	NVIC_EnableIRQ((IRQn_Type)(PININT0_IRQn));
#endif


#ifdef FEATURE_EVENT_COUNTER
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

	//MyUARTPrintDecimal(readBattery());
	//MyUARTSendCRLF();


/*
	LPC_CMP->CTRL =  (0x1 << 3) // rising edge
			| (0x2 << 8) // + of cmp to ACMP_input_2
			| (0x0 << 11) // - of cmp to voltage ladder
			;

	{int k;
	//for (k = 31; k >= 0; k--) {
	for (k = 0; k < 32; k++) {

		LPC_CMP->LAD = 1 | (k<<1);
		__WFI(); // allow to settle (15us on change, 30us on powerup)

		//if ( LPC_CMP->CTRL & (1<<21) ) {
		if ( ! (LPC_CMP->CTRL & (1<<21)) ) {

			MyUARTSendStringZ("L=");
			MyUARTPrintDecimal(k);
			MyUARTSendCRLF();
			MyUARTSendCRLF();
			break;
		}
	}
	}
	*/

	// No need for additional modifications to ACMP registers. Switch off clock.
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<19);

	//NVIC_EnableIRQ(CMP_IRQn);

#endif



#ifdef FEATURE_LED
	// Optional Diagnostic LED. Configure pin for output and blink 3 times.
	//GPIOSetDir(0,LED_PIN,1);
	LPC_GPIO_PORT->DIR0 |= (1<<LED_PIN);
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


#ifdef FEATURE_EVENT_COUNTER
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
		if ( (flags&0xf) == MODE_AWAKE) {
			rfm69_mode(RFM69_OPMODE_Mode_RX);
		}

#ifdef FEATURE_DEEPSLEEP
		// Test for MODE_OFF or MODE_LOW_POWER_POLL (LSB==0 for those two modes)
		if ( (flags&0x1) == 0) {

			// Set radio in SLEEP mode
			rfm69_mode(RFM69_OPMODE_Mode_SLEEP);

			// Reset source of wake event
			interrupt_source = 0;

			// Setup power management registers so that WFI causes DEEPSLEEP
			prepareForPowerDown();

			// Writing into WKT counter automatically starts wakeup timer
			// Polling interval determined by bits 11:8 of flags.
			// Ts = 0.5 * 2 ^ flags[11:8]
			// is 500 ms x 2 to the power of this value (ie 0=500ms, 1=1s, 2=2s,3=4s,4=8s...)
			uint32_t wakeup_time = 5000 << ((flags>>8)&0xf);
			LPC_WKT->COUNT = wakeup_time ;

			// DeepSleep until WKT interrupt (or PIN interrupt)
			__WFI();

			//MyUARTSendStringZ("sleep_clock += ");
			//MyUARTPrintDecimal(wakeup_time - LPC_WKT->COUNT);
			//MyUARTSendCRLF();
			// WKT in 100us increments, want sleep_clock in 10ms increments
			systick_counter += (wakeup_time - LPC_WKT->COUNT)/100;
			MyUARTSendStringZ("sleep_clock=");
			MyUARTPrintDecimal(systick_counter);
			MyUARTSendCRLF();

			// Experimental: Reassign UART to external pins
			//SwitchMatrix_Init();

			// Experimental: Re-set SPI pins after deepsleep/powerdown conditioning
			spi_init();

			// Allow time for clocks to stabilise after wake
			// TODO: can we use WKT and WFI?
			loopDelay(20000);

			if (interrupt_source == UART_INTERRUPT) {
				setOpMode(MODE_AWAKE);
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


		// If in MODE_LOW_POWER_POLL send poll packet
		if ( (flags&0xf) == MODE_LOW_POWER_POLL) {

			// Pullup resistor on PIO0_14
			//LPC_IOCON->PIO0_14=(0x2<<3);


			tx_buffer.header.to_addr = 0xff; // broadcast
			tx_buffer.header.msg_type = 'z';
			tx_buffer.payload[0] = sleep_counter++;
			tx_buffer.payload[1] = event_counter;
			tx_buffer.payload[2] = readBattery()/100;

#ifdef FEATURE_DS18B20
			ow_init(0,DS18B20_PIN);
			int32_t temperature = ds18b20_temperature_read();
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
			LPC_WKT->COUNT = 8000;
			interrupt_source = 0;
			do {
				__WFI();
			} while (interrupt_source != WKT_INTERRUPT);
			//loopDelay(150000);
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
		if ( ((flags&0xf)!=MODE_ALL_OFF) && IS_PAYLOAD_READY()  ) {

			// Yes, frame ready to be read from FIFO
#ifdef FEATURE_LED
			LPC_GPIO_PORT->PIN0 |= (1<<LED_PIN);
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,66,&rssi);
			LPC_GPIO_PORT->PIN0 &= ~(1<<LED_PIN);
#else
			int frame_len = rfm69_frame_rx(rx_buffer.buffer,66,&rssi);
#endif

			last_frame_time = systick_counter;

			// TODO: tidy this
			// SPI error
			//if (frame_len>0) {

#ifdef FEATURE_WATCHDOG_TIMER
			// Feed watchdog
			LPC_WWDT->FEED = 0xAA;
			LPC_WWDT->FEED = 0x55;

#endif


			// 0xff is the broadcast address
			if ( (flags&FLAG_PROMISCUOUS_MODE)
					|| rx_buffer.header.to_addr == 0xff
					|| rx_buffer.header.to_addr == tx_buffer.header.from_addr) {

				// This frame is for us! Examine messageType field for appropriate action.

				switch (rx_buffer.header.msg_type) {

#ifdef FEATURE_REMOTE_PKT_TX
				// Experimental remote packet transmit / relay
				case 'B' : {
					int payload_len = frame_len - 3;
					uint8_t payload[payload_len];
					memcpy(tx_buffer.payload,rx_buffer.payload,payload_len);
					rfm69_frame_tx(tx_buffer.buffer, payload_len+3);
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

					// Echo remote command to UART, copy remote command to UART buffer and
					// trigger UART command parsing.
					MyUARTSendStringZ("d ");
					int payload_len = frame_len - 3;
					uint8_t *uart_buf = MyUARTGetBuf();
					memcpy(uart_buf,rx_buffer.payload,payload_len);
					uart_buf[payload_len] = 0; // zero terminate buffer
					MyUARTSendStringZ(uart_buf);
					MyUARTSendCRLF();
					MyUARTSetBufFlags(UART_BUF_FLAG_EOL);
					break;
				}
#endif

				// Message requesting position report. This will return the string
				// set by the UART 'L' command verbatim.
				case 'R' :
				//case 'z' : // for testing only
				{
					int loc_len = MyUARTGetStrLen(current_loc);
					// report position
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'r';
					memcpy(tx_buffer.payload,current_loc,loc_len);
					tx_buffer.payload[loc_len] = rssi;
					rfm69_frame_tx(tx_buffer.buffer, loc_len+4);
					break;
				}

#ifdef FEATURE_LED
				// Remote LED blink
				case 'U' : {
					//tx_buffer.header.to_addr = from_addr;
					tx_buffer.header.to_addr = rx_buffer.header.from_addr;
					tx_buffer.header.msg_type = 'u';
					rfm69_frame_tx(tx_buffer.buffer, 3);
					ledBlink();
					break;
				}
#endif

#ifdef FEATURE_REMOTE_REG_READ
				// Remote register read
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
#endif

#ifdef FEATURE_REMOTE_REG_WRITE
				// Remote register write
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


#endif


#ifdef FEATURE_REMOTE_MEM_RWX
				// Experimental write to memory
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
				// Experimental read from memory
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
					// Execute function at memory loadion payload+0 (note LSB=1 for Thumb)
					uint32_t **mem_addr;
					mem_addr = rx_buffer.payload;
					typedef void (*E)(void);
					E e_entry = (E)*mem_addr;
					e_entry();
					break;
				}
#endif

				// If none of the above cases match, output packet to UART
				default: {

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
				}

			} else {

#ifdef FEATURE_DEBUG
				MyUARTSendStringZ("i Ignoring packet from ");
				MyUARTPrintHex(to_addr);
				MyUARTSendCRLF();
#endif

			}


			//} // end frame len valid check
		}

		if (MyUARTGetBufFlags() & UART_BUF_FLAG_EOL) {

			//MyUARTPrintHex(event_counter);
			//MyUARTSendCRLF();

#ifdef FEATURE_DEEPSLEEP
			// Any command will set mode to MODE_AWAKE if in MODE_ALL_OFF or MODE_LOW_POWER_POLL
			// TODO: will probably want to exclude remote commands
			setOpMode(MODE_AWAKE);
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
				MyUARTSendStringZ("u ");
				MyUARTPrintHex(get_mcu_serial_number());
				MyUARTSendCRLF();
				break;
			}
#endif

			// Set link loss reset
			case 'J' : {
				if (argc == 1) {
					MyUARTSendStringZ("j ");
					MyUARTPrintHex(link_loss_timeout);
					MyUARTSendCRLF();
					break;
				}
				link_loss_timeout = parse_hex(args[1]);
				break;
			}



			// Set current location
			case 'L' : {
				// +1 on len to include zero terminator
				memcpy(current_loc,args[1],MyUARTGetStrLen(args[1])+1);
				break;
			}

#ifdef FEATURE_NMEA_INPUT
			// NMEA (only interested in $GPGLL)
			case '$' : {
				// +1 on len to include zero terminator
				if (MyUARTGetStrLen(args[0]) && args[0][4]=='L' && args[0][5]=='L') {
					memcpy(current_loc,args[0],MyUARTGetStrLen(args[0])+1);
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
				uint8_t *b;
				int regAddr = parse_hex(args[1],&b);
				MyUARTSendStringZ("r ");
				print_hex8 (regAddr);
				MyUARTSendByte(' ');
				print_hex8 (rfm69_register_read(regAddr));
				MyUARTSendCRLF();
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
					SwitchMatrix_WithSWD_Init();
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

			case 'X' :  {
				MyUARTPrintDecimal(readBattery());
				MyUARTSendCRLF();
				break;
			}


#ifdef FEATURE_UART_MEM_RWX
				// Experimental write to memory. Write 8 bits at a time as writing more
				// might trigger a fault if not correctly aligned.
				// > 32bit-addr byte-value
				case '>' : {
					uint32_t *mem_addr;
					mem_addr = parse_hex(args[1]);
					*mem_addr = parse_hex(args[2]);
					break;
				}
				// Experimental read from memory
				// < 32bit-addr
				case '<' : {
					uint32_t *mem_addr;
					mem_addr = parse_hex(args[1]);
					MyUARTPrintHex(*mem_addr);
					MyUARTSendCRLF();
					break;
				}
#endif



			default : {
				report_error(*args[0], E_INVALID_CMD);
			}

			}

			// Reset UART command line buffer ready for next command
			MyUARTBufReset();


		} // end command switch block

#ifdef FEATURE_LINK_LOSS_RESET
		if ( (flags&0xf) == MODE_LOW_POWER_POLL) {
			if (link_loss_timeout!=0 && (systick_counter - last_frame_time > link_loss_timeout)) {
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


#ifdef FEATURE_EVENT_COUNTER

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
