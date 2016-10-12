#ifndef __RFM_H
#define __RFM_H

/**
 * Functions common to RFM69 and RFM95/6/7/8 modules
 */

void rfm_init();

void rfm_nss_assert();
void rfm_nss_deassert();

uint8_t rfm_spi_transfer_byte(uint8_t b);

uint8_t rfm_register_read (uint8_t reg_addr);
void rfm_register_write (uint8_t reg_addr, uint8_t reg_value);
int rfm_wait_for_bit_high (uint8_t reg_addr, uint8_t mask);

#endif /* end __RFM_H */
