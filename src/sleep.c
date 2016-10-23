/*
 * sleep.c
 *
 *  Created on: 1 Jul 2013
 *      Author: joe
 */

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "lpc8xx_pmu.h"
#include "lpc8xx_util.h"

#include "sleep.h"
#include "config.h"

/**
 * Condition perhiperals and external pins for optimal power consuption while in sleep/power-down.
 */
void sleep_condition_for_powerdown () {

	// Condition pins to minimize current use during sleep

//#ifdef LPC812
	// Experiment to disconnect UART to see if it reduces current during sleep
	//LPC_SWM->PINASSIGN0 = 0xffffffffUL;

	// Experiment (conducted w/o RFM69 radio connected): set SPI pins to output
	// SPI pins output, all 0 results in 380uA
	// SPI pins output, MISO=1, rest=0 328uA
	// SPI pins output, MISO=MOSI=1, rest=0 275uA
	// SPI pins output, MISO=MOSI=SCK=1, SS=0 222uA
	// SPI pins output, all=1 results in 169uA power down current

	// Experiment on LPC810 with RFM69HW connected:
	// 44uA current consumption with all SPI pins output and high during power down mode.

	// Set MISO (normally input) to output
	//LPC_GPIO_PORT->DIR0 |= 1<<MISO_PIN;

	// Unused pins
	//LPC_GPIO_PORT->DIR0 |= ( (1<<11) | (1<<10) | (1<<16) );


#ifdef DIO0_PIN
	//LPC_GPIO_PORT->DIR0 |= 1<<DIO0_PIN;
	//LPC_GPIO_PORT->CLR0 = (1<<DIO0_PIN);
#endif

	// Set all SPI pins high
	LPC_GPIO_PORT->SET0 = (1<<MISO_PIN) | (1<<MOSI_PIN) | (1<<SCK_PIN) | (1<<SS_PIN);


	// .. or low?
	//LPC_GPIO_PORT->CLR0 = (1<<MISO_PIN) | (1<<MOSI_PIN) | (1<<SCK_PIN) | (1<<SS_PIN);

	// All inputs - no difference still 240uA?
	//LPC_GPIO_PORT->DIR0 &= ~((1<<MISO_PIN) | (1<<MOSI_PIN) | (1<<SCK_PIN) | (1<<SS_PIN));

	// 320uA
	//LPC_GPIO_PORT->DIR0 |= 0xFFF;

	// Turn off clock to GPIO
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<6);
//#endif


	// Observation: disconnecting SWCLK during sleep reduces current draw from 200uA to 150uA.
	// Experimental disable SWD - na doesn't make a difference.

    /* Enable SWM clock */
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Bit 2: SWCLK;  Bit 3: SWDIO */
    //LPC_SWM->PINENABLE0 |= (1<<2) | (1<<3);

	// Turn off clock to SwitchMatrix
	//LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<7);

}

/**
 * Restore perhipherals and pins after wake from sleep/power-down
 */
void sleep_condition_after_wake () {

	// Turn on clock to GPIO
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

	// Experimental reenable SWD

    /* Enable SWM clock */
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Bit 2: SWCLK;  Bit 3: SWDIO */
    //LPC_SWM->PINENABLE0 &= ~ ((1<<2) | (1<<3));

	// Turn off clock to SwitchMatrix
	//LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<7);

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


