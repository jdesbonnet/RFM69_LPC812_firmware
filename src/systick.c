/**
 *
 */

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#ifdef X_FEATURE_SYSTICK
volatile uint32_t systick_counter = 0;

/* SysTick interrupt happens every 10 ms */
void SysTick_Handler(void)
{
	systick_counter++;
}

#endif
