
#include "chip.h"
#include "lpc8xx_util.h"

/**
 * Reset LPC8xx perhipheral. Moving this into a function saves a few bytes.
 */
void lpc8xx_peripheral_reset(int bitIndex) {
	//LPC_SYSCON->PRESETCTRL &= ~(0x1<<bitIndex);
	//LPC_SYSCON->PRESETCTRL |= (0x1<<bitIndex);
	Chip_SYSCTL_AssertPeriphReset(bitIndex);
	Chip_SYSCTL_DeassertPeriphReset(bitIndex);
}
