#include "LPC8xx.h"
#include "lpc8xx_gpio.h"
#include "config.h"

/**
 * Initialize pin for output to WS2812B LED chain.
 */
void ws2812b_init () {
	// set direction output
	GPIOSetDir(0, WS2812B_PIN, 1);
}

/**
 * Write a color to the next LED in chain.
 *
 * @param color 0x00bbrrgg
 *
 */
//#define NO_OPTIMIZE __attribute__((optimize("O0")))
//#define NO_OPTIMIZE
void __attribute__((optimize("O2"))) ws2812b_bitbang (uint32_t color) {
	int i;
	for (i = 0; i < 24; i++) {
		if ((color & 1) == 0) {
			LPC_GPIO_PORT->SET0 = 1<<WS2812B_PIN;
			LPC_GPIO_PORT->CLR0 = 1<<WS2812B_PIN;
			__NOP();
		} else {
			LPC_GPIO_PORT->SET0 = 1<<WS2812B_PIN;
			__NOP();
			LPC_GPIO_PORT->CLR0 = 1<<WS2812B_PIN;
		}
		color >>= 1;
	}

}

void ws2812b_reset () {
	int j;
	for (j = 0; j < 192; j++) {
		__NOP();
	}
}
