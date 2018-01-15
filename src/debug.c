#include "config.h"

void debug_show_registers () {
	debug("PCON=0x%x", LPC_PMU->PCON);
	debug("PDAWAKECFG=0x%x", LPC_SYSCON->PDAWAKECFG);
	debug("PDRUNCFG=0x%x", LPC_SYSCON->PDRUNCFG);
	debug("SCR=0x%x", SCB->SCR);
	debug("DPDCTRL=0x%x",LPC_PMU->DPDCTRL);
	debug("DIR[0]=%x", LPC_GPIO_PORT->DIR[0]);

	int i;
	for (i = 0; i < 20; i++) {
		debug("PIO[%d] DIR=%d IOCON[%d]=0x%x",i, ((LPC_GPIO_PORT->DIR[0]&(1<<i))==0?0:1), LPC_IOCON->PIO0[i]);
	}
}
