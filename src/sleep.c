/*
 * sleep.c
 *
 *  Created on: 1 Jul 2013
 *      Author: joe
 */

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */
#include "lpc8xx_pmu.h"

#include "sleep.h"

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
	// 0x1 Deep-sleep; 0x2 Power-down
	//LPC_PMU->PCON = 0x1; // Getting ~ 200uA in this mode
	LPC_PMU->PCON = 0x2; // Getting ~ 60uA in this mode



	  // Step 2: Select the power configuration in Power-down mode in the
	  // PDSLEEPCFG (Table 35) register
	  //LPC_SYSCON->PDSLEEPCFG = LPC_SYSCON->PDSLEEPCFG;


	  // Step 3: Select the power configuration after wake-up in the
	  // PDAWAKECFG (Table 36) register
	  LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;


	  // Step 4: If any of the available wake-up interrupts are used for wake-up,
	  // enable the interrupts in the interrupt wake-up registers
	  // (Table 33, Table 34) and in the NVIC.
	  // Needed?
	  //NVIC_ClearPendingIRQ(WKT_IRQn);
	  //NVIC_DisableIRQ(WKT_IRQn);
	  NVIC_EnableIRQ(WKT_IRQn);


	  // Step 5: Write one to the SLEEPDEEP bit in the ARM Cortex-M0+
	  // SCR register (Table 41) UM10601 §5.3.1.1 p42
	  SCB->SCR |= NVIC_LP_SLEEPDEEP;

	  // STARTERP1: Start logic 1 interrupt wake-up enable register
	  // Bit 15: 1 = enable self wake-up timer interrupt wake-up
	  // Bit 3: 1 = enable self wake-up on UART interrupt
	  // Ref UM10601 §4.6.29
	  // hmm.. UART must be in synchronous slave mode:
	  // http://docs.lpcware.com/lpc800um/RegisterMaps/uart/c-ConfiguretheUSARTforwake-up.html
	  LPC_SYSCON->STARTERP1 = (1<<15)  | (1<<3);

	  // DPDCTRL: Deep power-down control register
	  // UM10601 §5.6.3 p47
	  // Bit 0
	  // Bit 1 (WAKEPAD_DISABLE): 0=enable wakeup on PIO0_4.
	  // Bit 2 (LPOSCEN): 10kHz low power oscillator enable
	  // Bit 3 (LPOSCDPDEN): enable LPOSC in DPD mode.
	  LPC_PMU->DPDCTRL |= (0x1 << 2);


	  // Enable clock for WKT in System clock control register (SYSAHBCLKCTRL)
	  // Ref UM10601 §4.6.13, p26
	  // Bit 9: 1 = enable WKT
	  //LPC_SYSCON->SYSAHBCLKCTRL |= (0x1 << 9);

	  // Reset WKT by writing 0, and then 1 to bit 9 of Peripheral reset control register
	  // Ref UM10601, §4.6.2, p20.
	  // TODO: is this necessary?
	  LPC_SYSCON->PRESETCTRL &= ~(0x1 << 9);
	  LPC_SYSCON->PRESETCTRL |= (0x1 << 9);


	  // Use WTK clock source 1 (10kHz, low power, low accuracy).
	  // (clock source 0 is 750kHz, high accuracy, but unavailable in power-down
	  // mode)
	  // WKT Control Register
	  // UM10601 §13.6.1 p165
	  // Bit 0: CLKSEL 0=Divided IRC @750kHz, 1=LPOSC @10kHz (+/- 45%)
	  // Bit 1: ALARMFLAG 0=no timeout; 1=timeout.
	  LPC_WKT->CTRL |= 0x01; // 10kHz LPOSC
}

void WKT_IRQHandler(void)
{
  //if ( LPC_WKT->CTRL & 0x02 )
  //{
		LPC_WKT->CTRL |= 0x02;			/* clear interrupt flag */
  //}
}

