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
void ws2812b_bitbang (uint32_t color) {
	int i;
	for (i = 0; i < 24; i++) {
		//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 1);
		GPIOSetBitValue(0, WS2812B_PIN, 1);
		if (color & 1) {
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
					//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 0);
			GPIOSetBitValue(0, WS2812B_PIN, 0);
		} else {
			__NOP();
			__NOP();
			//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, PIN_WS2812B, 0);
			GPIOSetBitValue(0, WS2812B_PIN, 0);
			__NOP();
			__NOP();
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
