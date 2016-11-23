/**
 * Bit-bang implementation of SPI protocol.
 */

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include "config.h"
#include "lpc8xx_gpio.h"

#define PORT 0


//#define SCK_HIGH() GPIOSetBitValue(PORT,SCK_PIN,1)
//#define SCK_LOW() GPIOSetBitValue(PORT,SCK_PIN,0)
//#define SS_HIGH() GPIOSetBitValue(PORT,SS_PIN,1)
//#define SS_LOW() GPIOSetBitValue(PORT,SS_PIN,0)
//#define MOSI_HIGH() GPIOSetBitValue(PORT,MOSI_PIN,1)
//#define MOSI_LOW() GPIOSetBitValue(PORT,MOSI_PIN,0)
//#define MISO_READ() GPIOGetPinValue(PORT,MISO_PIN)


#define SCK_HIGH() LPC_GPIO_PORT->SET0=(1<<SCK_PIN);
#define SCK_LOW() LPC_GPIO_PORT->CLR0=(1<<SCK_PIN);

#define SS_HIGH() LPC_GPIO_PORT->SET0=(1<<SS_PIN);
#define SS_LOW() LPC_GPIO_PORT->CLR0=(1<<SS_PIN);

#define MOSI_HIGH() LPC_GPIO_PORT->SET0=(1<<MOSI_PIN);
#define MOSI_LOW() LPC_GPIO_PORT->CLR0=(1<<MOSI_PIN);


#define MISO_READ() LPC_GPIO_PORT->PIN0&(1<<MISO_PIN)

/**
 * Initialize SPI using bigbang (without SSP0 peripheral).
 */
void spi_init () {

	//GPIOInit();
	/* Peripheral reset control to GPIO and GPIO INT, a "1" bring it out of reset. */
	//LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
	//LPC_SYSCON->PRESETCTRL |= (0x1<<10);

	//GPIOSetDir(PORT, SS_PIN, 1); // output
	//GPIOSetDir(PORT, SCK_PIN, 1);
	//GPIOSetDir(PORT, MOSI_PIN, 1);
	//GPIOSetDir(PORT, MISO_PIN, 0); // input

	// Note: more space efficient (by 4 bytes) to load into regVal for manipulation
	//LPC_GPIO_PORT->DIR0 |= (1<<SS_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN);
	//LPC_GPIO_PORT->DIR0 &= ~(1<<MISO_PIN);
	uint32_t regVal = LPC_GPIO_PORT->DIR0;
	regVal |= (1<<SS_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN);
	regVal &= ~(1<<MISO_PIN);
	LPC_GPIO_PORT->DIR0 = regVal;
}

void spi_deinit () {
	uint32_t regVal = LPC_GPIO_PORT->DIR0;
	regVal &=  ~((1<<SS_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN));
	LPC_GPIO_PORT->DIR0 = regVal;
}

/**
 * Disable SPI pins (for power saving)
 */
/*
void spi_off () {
	GPIOSetDir(PORT, SS_PIN, 0); // input
	GPIOSetDir(PORT, SCK_PIN, 0);
	GPIOSetDir(PORT, MOSI_PIN, 0);
}
*/

static void spi_delay(void) {

	int i = 0;
	for (i = 0; i < 16; i++) {
		__NOP();
	}

	__NOP();
	__NOP();
	//__NOP();
	//__NOP();
}

__attribute__((always_inline))
void spi_assert_ss () {
		SS_LOW();
}

__attribute__((always_inline))
void spi_deassert_ss () {
		SS_HIGH();
}

__attribute__((always_inline))
uint8_t spi_transfer_byte (uint8_t out) {

	int j=0x80;
	uint8_t in=0;

	while (j) {
			// Set MOSI
			if (out & j) {
				//LPC_GPIO0->DATA |= (1<<9);
				MOSI_HIGH();
			} else {
				//LPC_GPIO0->DATA &= ~(1<<9);
				MOSI_LOW();
			}

			j >>= 1;

			// Rising clock edge
			//LPC_GPIO0->DATA |= (1<<6);
			SCK_HIGH();

			spi_delay();

			// Read MISO
			in <<= 1;
			if (MISO_READ()) {
				in |= 1;
			}

			//LPC_GPIO0->DATA &= ~(1<<6);
			SCK_LOW();
	}

	return in;

}

