/**
 * RFM69HW (HopeRF) radio modem driver.
 *
 * Joe Desbonnet, jdesbonnet@gmail.com
 */

#include "config.h"
#include "rfm98_config.h"

extern const uint8_t RFM98_CONFIG[][2];

/**
 * Assert hardware reset line on RFM9x. Active low.
 */
void rfm98_hard_reset(void) {
#ifdef RESET_PIN
	// Configure RESET_PIN as output
	uint32_t regVal = LPC_GPIO_PORT->DIR0;
	regVal |= (1<<RESET_PIN);
	LPC_GPIO_PORT->DIR0 = regVal;

	LPC_GPIO_PORT->CLR0=(1<<RESET_PIN);
	delay(20000);
	LPC_GPIO_PORT->SET0=(1<<RESET_PIN);
#endif
}

/**
 * Configure RFM98 module.
 */
void rfm98_config() {

	rfm98_hard_reset();

	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRa_SLEEP);
	delayMilliseconds(10);
	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRaMode );
	delayMilliseconds(10);

	// Set max power

	rfm_register_write(RFM98_PACONFIG, RFM98_PACONFIG_PaSelect
			| RFM98_PACONFIG_MaxPower_VALUE(4)
			| RFM98_PACONFIG_OutputPower_VALUE(15)
			);

	//rfm_config(RFM98_CONFIG);
}

/**
 * Set operating mode. Use macro values RFM98_OPMODE_LoRa_xxxx as arg.
 */
int rfm98_lora_mode(uint8_t mode) {
	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRaMode | mode);
	return E_OK;
}

int rfm98_is_packet_ready() {
	//uint8_t regVal =  rfm_register_read(RFM98_IRQFLAGS);
	//debug("irq=%x",regVal);
	return rfm_register_read(RFM98_IRQFLAGS) & RFM98_IRQFLAGS_RxDone;
}

/**
 * Retrieve a frame. If successful returns length of frame. If not an error code (negative value).
 * Frame is returned in buf but will not exceed length maxlen. Should only be called when
 * a frame is ready to download.
 *
 * @return frame_length if successful, else a negative value error code
 * Error codes:
 * -2 : frame too long
 */
int rfm98_frame_rx(uint8_t *buf, int maxlen) {

	int i;

    uint8_t frame_length = rfm_register_read(RFM98_RXNBBYTES);

    //debug("frame_length=%d", frame_length);

    uint8_t start_buf_addr = rfm_register_read(RFM98_FIFORXCURRENT);

    //debug("start_buf_addr=%d", start_buf_addr);

    rfm_register_write(RFM98_FIFOADDRPTR, start_buf_addr);


    rfm_nss_assert();
    spi_transfer_byte(RFM98_FIFO);

    for (i = 0; i < frame_length; i++) {
    	if (i == maxlen) {
    		return E_PKT_TOO_LONG;
    	}
    	buf[i] = spi_transfer_byte(0);
    }
    rfm_nss_deassert();

    // Clear RxDone IRQ
    rfm_register_write(RFM98_IRQFLAGS,RFM98_IRQFLAGS_RxDone);
    //rfm_register_write(RFM98_IRQFLAGS,0xff);

    return frame_length;
}

/**
 * Transmit a frame.
 */
void rfm98_frame_tx(uint8_t *buf, int len) {

	// Turn off receiver before writing to FIFO
	rfm98_lora_mode(RFM98_OPMODE_LoRa_STDBY);


	// Default TX buffer is at FIFO 0x80
	rfm_register_write(RFM98_FIFOADDRPTR, 0x80);

	// Payload length
	rfm_register_write(0x22, len);

	// Write frame to FIFO
	rfm_nss_assert();

	rfm_spi_transfer_byte(RFM98_FIFO | 0x80);
	// packet length
	//rfm_spi_transfer_byte(len);

	int i;
	for (i = 0; i < len; i++) {
		rfm_spi_transfer_byte(buf[i]);
	}

	rfm_nss_deassert();

	// Power up TX
	rfm98_lora_mode(RFM98_OPMODE_LoRa_TX);

	// Wait for TxDone IRQ
	rfm_wait_for_bit_high(RFM98_IRQFLAGS, RFM98_IRQFLAGS_TxDone);

	// Clear TxDone IRQ
	rfm_register_write(RFM98_IRQFLAGS, RFM98_IRQFLAGS_TxDone);
}

/**
 * RSSI of last packet received. dBm.
 */
int rfm98_last_packet_rssi() {
	// RSSI
	return rfm_register_read(0x1a) - 137;
}


/**
 * SNR of last packet received. dB.
 */
int rfm98_last_packet_snr() {
	return (int)rfm_register_read(0x19);
}



