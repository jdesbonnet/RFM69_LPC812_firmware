#include "config.h"
#include "delay.h"
#include "led.h"

/**
 * Blink diagnostic LED. Optional feature (edit config.h to define hardware configuration).
 */
void led_blink (int nblink) {
	int i;
	for (i = 0; i < nblink; i++)	{
			led_on();
			delay_milliseconds(200);
			led_off();
			delay_milliseconds(200);
	}
}

//__attribute__((always_inline))
void led_on() {
#ifdef FEATURE_LED
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, LED_PIN);
#endif
}

//__attribute__((always_inline))
void led_off() {
#ifdef FEATURE_LED
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, LED_PIN);
#endif
}

