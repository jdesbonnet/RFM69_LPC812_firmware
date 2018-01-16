#include "config.h"

static uint8_t iocon_map[] = {0x44,0x2c,0x18,0x14,
		0x10,0xc, 0x40,0x3c,
		0x38,0x34,0x20,0x1c,
		0x8, 0x4, 0x48, 0x28,
		0x24, 0x0,
};



void debug_show_registers () {
	debug("PCON=0x%x", LPC_PMU->PCON);
	debug("PDAWAKECFG=0x%x", LPC_SYSCON->PDAWAKECFG);
	debug("PDRUNCFG=0x%x", LPC_SYSCON->PDRUNCFG);
	debug("SCR=0x%x", SCB->SCR);
	debug("DPDCTRL=0x%x",LPC_PMU->DPDCTRL);
	debug("DIR[0]=%x", LPC_GPIO_PORT->DIR[0]);
	debug("IOCON PIO17=%x",LPC_IOCON->PIO0[IOCON_PIO17]);

	int i;
	uint32_t *iocon_ptr;

	for (i = 0; i < 18; i++) {
		iocon_ptr = 0x40044000 + iocon_map[i];
		debug("PIO[%d] DIR=%d IOCON=0x%x %x",i, ((LPC_GPIO_PORT->DIR[0]&(1<<i))==0?0:1), *iocon_ptr, iocon_ptr);
	}

	delayMicroseconds(20000);
}
