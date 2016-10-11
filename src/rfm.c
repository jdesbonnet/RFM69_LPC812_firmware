/**
 * RFM69HW/RFM95/6/7/8 (HopeRF) common functions.
 *
 * Joe Desbonnet, jdesbonnet@gmail.com
 */

#include "config.h"

#include <stdint.h>
#include "rfm.h"
#include "spi.h"
#include "err.h"
#include "delay.h"

void rfm_init(void) {
	spi_init();
}

/**
 * Assert NSS line (bring low)
 */
RAM_FUNC
void rfm_nss_assert() {
	spi_assert_ss();
}

/**
 * Deassert NSS (slave select) line. Must be careful to ensure that SPI FIFO
 * has drained before doing this.
 */
RAM_FUNC
void rfm_nss_deassert() {
	spi_deassert_ss();
}

RAM_FUNC
uint8_t rfm_spi_transfer_byte(uint8_t b) {

#ifdef USE_SSP0
	uint8_t tx,rx;
	SPI_SendRcv(LPC_SPI0,
			0 /* assert SlaveSelect*/,
			&tx, &rx, 1);
	return rx;
#endif

	return spi_transfer_byte(b);
}

RAM_FUNC
uint8_t rfm_register_read (uint8_t reg_addr) {
	rfm_nss_assert();
	rfm_spi_transfer_byte(reg_addr);
	uint8_t reg_value = rfm_spi_transfer_byte(0xff);
	rfm_nss_deassert();
	return reg_value;
}

RAM_FUNC
void rfm_register_write (uint8_t reg_addr, uint8_t reg_value) {
	rfm_nss_assert();
	rfm_spi_transfer_byte (reg_addr | 0x80); // Set bit 7 to indicate write op
	rfm_spi_transfer_byte (reg_value);
	rfm_nss_deassert();
}
