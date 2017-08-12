/*
 * sleep.c
 *
 *  Created on: 1 Jul 2013
 *      Author: joe
 */


#include "config.h"
#include "lpc8xx_pmu.h"
#include "lpc8xx_util.h"
#include "sleep.h"

/**
 * Condition peripherals and external pins for optimal power consumption while in sleep/power-down.
 */
void sleep_set_pins_for_powerdown () {

	// Condition pins to minimize current use during sleep

	spi_deinit();

#ifdef FEATURE_LED
	// Setting LED pin into input with pull down R seems to eliminate the
	// last 60uA of unexplained current use.
	//LPC_GPIO_PORT->DIR0 &= ~(1<<LED_PIN);
	// Pulldown resistor on PIO0_17 (pin LED_PIN)
	// TODO: PIO0_17 IOCON hard coded
	//LPC_IOCON->PIO0_17=(0x1<<3);

	Chip_GPIO_SetPinDIR(LPC_GPIO_PORT, 0, LED_PIN, false);
	Chip_IOCON_PinSetMode(LPC_GPIO_PORT, LED_PIN,  PIN_MODE_PULLDN);
#endif

#ifdef FEATURE_WS2812B
	//LPC_GPIO_PORT->SET0 |= (1<<WS2812B_PIN);
	//LPC_GPIO_PORT->CLR0 |= (1<<WS2812B_PIN);
	LPC_GPIO_PORT->DIR0 &= ~(1<<WS2812B_PIN);
	LPC_IOCON->PIO0_14=(0x2<<3);
#endif
}

/**
 * Restore perhipherals and pins after wake from sleep/power-down
 */
void sleep_set_pins_for_wake () {

#ifdef FEATURE_LED
	//LPC_GPIO_PORT->DIR0 |= (1<<LED_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, LED_PIN);
#endif

	// Reinit SPI
	spi_init();
}

/**
 * Condition MCU (pin states etc) to minimize current consumption during
 * deep sleep modes.
 */
void prepareForPowerDown () {

	//
	// UM10601 §5.7.6.2, p52: Programming Power-down mode.
	// Power-down mode keeps processor state, registers and SRAM. Almost
	// everything else is off.
	// There are 6 steps to entering power-down state:
	//

	// Step 1: The PM (power mode) bits (bits 2:0) in the PCON (power control)
	// register must be set to 0x2 (power down mode)
	// Ref UM10601 §5.6.1, Table 44,  p46.
	// 0x1 Deep-sleep; 0x2 Power-down; 0x3 Deep power-down
	// LPC812 with RFM98 (fw 0.7.0) PCON=1 400uA; PCON=2 240uA;
	//LPC_PMU->PCON = 0x1;
	LPC_PMU->PCON = 0x2;


	  // Step 2: Select the power configuration in Power-down mode in the
	  // PDSLEEPCFG (Table 35) register
	  //LPC_SYSCON->PDSLEEPCFG = ~(1<<3); // WDT on during deep sleep/power down
	  //LPC_SYSCON->PDSLEEPCFG = ~((1<<6)|(1<<3)); // WDT+BOD on during deep sleep/power down


	  // Step 3: Select the power configuration after wake-up in the
	  // PDAWAKECFG (Table 36) register
	  LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;


	  // Step 4: If any of the available wake-up interrupts are used for wake-up,
	  // enable the interrupts in the interrupt wake-up registers
	  // (Table 33, Table 34) and in the NVIC.
	  NVIC_EnableIRQ(WKT_IRQn);


	  // Step 5: Write one to the SLEEPDEEP bit in the ARM Cortex-M0+
	  // SCR register (Table 41) UM10601 §5.3.1.1 p42
	  SCB->SCR |= NVIC_LP_SLEEPDEEP;


	  // STARTERP1: Start logic 1 interrupt wake-up enable register
	  // Bit 15: 1 = enable self wake-up timer interrupt wake-up
	  // Ref UM10601 §4.6.29
	  // hmm.. UART must be in synchronous slave mode:
	  // http://docs.lpcware.com/lpc800um/RegisterMaps/uart/c-ConfiguretheUSARTforwake-up.html
	  LPC_SYSCON->STARTERP1 = (1<<15);


#ifdef FEATURE_UART_INTERRUPT
	  LPC_SYSCON->STARTERP0 |= (1<<0); // PININT0 (UART RXD)
#endif

#ifdef FEATURE_EVENT_COUNTER
	  // Also PINTINT0, PININT1, PININT2
	  LPC_SYSCON->STARTERP0 |= (1<<1) // PININT1 (tip bucket)
							//| (1<<2) // PININT2 (comparator output)
							;
#endif

	  // DPDCTRL: Deep power-down control register
	  // UM10601 §5.6.3 p47
	  // Bit 0
	  // Bit 1 (WAKEPAD_DISABLE): 0=enable wakeup on PIO0_4.
	  // Bit 2 (LPOSCEN): 10kHz low power oscillator enable
	  // Bit 3 (LPOSCDPDEN): enable LPOSC in DPD mode.
	  LPC_PMU->DPDCTRL |= (0x1 << 2)
			  // | (1<<3)
			  ;


	  // Enable clock for WKT in System clock control register (SYSAHBCLKCTRL)
	  // Ref UM10601 §4.6.13, p26
	  // Bit 9: 1 = enable WKT
	  //LPC_SYSCON->SYSAHBCLKCTRL |= (0x1 << 9);

	  // Reset WKT by writing 0, and then 1 to bit 9 of Peripheral reset control register
	  // Ref UM10601, §4.6.2, p20.
	  // TODO: is this necessary?
	  //LPC_SYSCON->PRESETCTRL &= ~(0x1 << 9);
	  //LPC_SYSCON->PRESETCTRL |= (0x1 << 9);
	  lpc8xx_peripheral_reset(9);



	  // Use WTK clock source 1 (10kHz, low power, low accuracy).
	  // (clock source 0 is 750kHz, high accuracy, but unavailable in power-down
	  // mode)
	  // WKT Control Register
	  // UM10601 §13.6.1 p165
	  // Bit 0: CLKSEL 0=Divided IRC @750kHz, 1=LPOSC @10kHz (+/- 45%)
	  // Bit 1: ALARMFLAG 0=no timeout; 1=timeout.
	  LPC_WKT->CTRL |= 0x01; // 10kHz LPOSC
}
