#include "config.h"
#include "delay.h"
#include "led.h"

/**
 * Blink diagnostic LED. Optional feature (edit config.h to define hardware configuration).
 */
void ledBlink (int nblink) {
	int i;
	for (i = 0; i < nblink; i++)	{
			ledOn();
			delay_milliseconds(200);
			ledOff();
			delay_milliseconds(200);
	}
}

__attribute__((always_inline))
void ledOn() {
	//GPIOSetBitValue(0,LED_PIN,1);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, LED_PIN);
}

__attribute__((always_inline))
void ledOff() {
	//GPIOSetBitValue(0,LED_PIN,0);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, LED_PIN);
}

