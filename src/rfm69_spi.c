/**
 * RFM69 library functions that need to be customized for a particular hardware
 * platform (mosty SPI interface).
 *
 * This is for LPC1114.
 */

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

//#include "lpc8xx_spi.h"
#include "spi.h"
#include "lpc8xx_gpio.h"

rfm69_init() {

#ifdef USE_SSP0
	// Initialize SPI peripheral
	/*SPI Init*/
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11);
	/* Peripheral reset control to SPI, a "1" bring it out of reset. */
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<0);
	LPC_SYSCON->PRESETCTRL |= (0x1<<0);
	LPC_SPI0->DIV = 19;
	SPI_Init(LPC_SPI0,
			  0x05, // DIV
			  CFG_MASTER,
			  DLY_PREDELAY(0x0)|DLY_POSTDELAY(0x0)|DLY_FRAMEDELAY(0x0)|DLY_INTERDELAY(0x0)
	);
#endif

	spi_init();


}

/**
 * Assert NSS line (bring low)
 */
void rfm69_nss_assert() {
	spi_assert_ss();
}

/**
 * Deassert NSS (slave select) line. Must be careful to ensure that SPI FIFO
 * has drained before doing this.
 */
void rfm69_nss_deassert() {
	spi_deassert_ss();
}

uint8_t rfm69_spi_transfer_byte(uint8_t b) {

#ifdef USE_SSP0
	uint8_t tx,rx;
	SPI_SendRcv(LPC_SPI0,
			0 /* assert SlaveSelect*/,
			&tx, &rx, 1);
	return rx;
#endif

	return spi_transfer_byte(b);
}
