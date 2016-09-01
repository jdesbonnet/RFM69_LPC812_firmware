#include "LPC8xx.h"
#include "config.h"
#include "lpc8xx_gpio.h"
#include "delay.h"
#include "led.h"

/**
 * Blink diagnostic LED. Optional feature (edit config.h to define hardware configuration).
 */
void ledBlink (int nblink) {
	int i;
	for (i = 0; i < nblink; i++)	{
			GPIOSetBitValue(0,LED_PIN,1);
			//loopDelay(200000);
			delayMilliseconds(200);
			GPIOSetBitValue(0,LED_PIN,0);
			//loopDelay(200000);
			delayMilliseconds(200);
	}
}

__attribute__((always_inline))
void ledOn() {
	GPIOSetBitValue(0,LED_PIN,1);
}

__attribute__((always_inline))
void ledOff() {
	GPIOSetBitValue(0,LED_PIN,0);
}

