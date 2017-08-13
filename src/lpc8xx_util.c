
#include "chip.h"
#include "lpc8xx_util.h"

/**
 * Reset LPC8xx peripheral.
 */
void lpc8xx_peripheral_reset(CHIP_SYSCTL_PERIPH_RESET_T peripheral) {
	Chip_SYSCTL_AssertPeriphReset(peripheral);
	Chip_SYSCTL_DeassertPeriphReset(peripheral);
}
