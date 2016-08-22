#include "LPC8xx.h"

/**
 * Use analog comparator with internal reference to find approx battery voltage. This
 * only works if powered directly from battery.
 *
 * @return Battery voltage in mV
 */
int readBattery () {

	//
	// Analog comparator configure
	//

	// Power to comparator. Use of comparator requires BOD. [Why?]
	LPC_SYSCON->PDRUNCFG &= ~( (0x1 << 15) | (0x1 << 3) );

	// Enable clock to analog comparator
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<19);

	// Analog comparator reset
	LPC_SYSCON->PRESETCTRL &= ~(0x1 << 12);
	LPC_SYSCON->PRESETCTRL |= (0x1 << 12);


	// Measure Vdd relative to 900mV bandgap reference
	LPC_CMP->CTRL =  (0x1 << 3) // rising edge
			| (0x6 << 8)  // + of cmp to 900mV bandgap reference
			| (0x0 << 11) // - of cmp to voltage ladder powered by Vdd
			;

	// Find k, voltage latter setting that causes comparator output to go low
	// At this point Vdd * k / 31 = 900mV
	// so Vdd (mV) = 900mV * 31 / k  =  27900mV / k;
	int k;
	for (k = 0; k <32; k++) {
		LPC_CMP->LAD = 1 | (k<<1);
		// allow time to settle (15us on change, 30us on powerup) by
		// waiting for next SYSTICK interrupt
		__WFI();
		if ( ! (LPC_CMP->CTRL & (1<<21))) {
			break;
		}
	}

	// Disable clock to analog comparator
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<19);

	// Power off comparator and BOD
	LPC_SYSCON->PDRUNCFG != ( (0x1 << 15) | (0x1 << 3) );

	// 900mV*31/k
	return 27900/k;
}
