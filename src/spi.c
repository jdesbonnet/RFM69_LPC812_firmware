/**
 * Bit-bang implementation of SPI protocol.
 */

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include "lpc8xx_gpio.h"

#define PORT 0
#define SCK_PIN 15
#define SS_PIN 9
#define MOSI_PIN 8
#define MISO_PIN 7

#define SCK_HIGH() GPIOSetBitValue(PORT,SCK_PIN,1)
#define SCK_LOW() GPIOSetBitValue(PORT,SCK_PIN,0)
#define SS_HIGH() GPIOSetBitValue(PORT,SS_PIN,1)
#define SS_LOW() GPIOSetBitValue(PORT,SS_PIN,0)

#define MOSI_HIGH() __NOP()
#define MOSI_LOW() __NOP()
#define MISO_READ() 1

/**
 * Initialize SPI using bigbang (without SSP0 peripheral).
 */
void spi_init () {
	GPIOSetDir(PORT, SS_PIN, 1); // output
	GPIOSetDir(PORT, SCK_PIN, 1);
	GPIOSetDir(PORT, MOSI_PIN, 1);
	GPIOSetDir(PORT, MISO_PIN, 0); // input
}


static void spi_delay(void) {
	int i = 0;
	for (i = 0; i < 4; i++) {
		__NOP();
	}
}

void spi_assert_ss () {
		SS_LOW();
}

void spi_deassert_ss () {
		SS_HIGH();

}


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

