#ifndef __RFM69_H
#define __RFM69_H

// Autogenerated RFM69 headers generated with ICRDL tool
#include "rfm69_rdl.h"

void rfm69_init(void);
void rfm69_config(void);
int rfm69_test(void);

int rfm69_mode(uint8_t mode);

int rfm69_frame_rx(uint8_t *buf, int maxlen);
void rfm69_frame_tx(uint8_t *buf, int len);

uint8_t rfm69_register_read (uint8_t reg_addr);
void rfm69_register_write (uint8_t reg_addr, uint8_t reg_value);

uint8_t rfm69_payload_ready();
uint8_t rfm69_is_pll_lock();
uint8_t rfm69_temperature(void);

// Interface with SPI bus
void rfm69_nss_assert(void);
void rfm69_nss_deassert(void);
uint8_t rfm69_spi_transfer_byte (uint8_t b);


// Debug functions
void rfm69_register_print(void);
void rfm69_config_print(void);
void rfm69_config_verify(void);
void rfm69_showirqreg (void);

#define IS_PACKET_READY(x) rfm69_payload_ready()

#endif /* end __RFM69_H */
