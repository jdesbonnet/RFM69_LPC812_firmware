/**
 * RFM69HW (HopeRF) radio modem driver.
 *
 * Joe Desbonnet, jdesbonnet@gmail.com
 */

#include "config.h"

#include <stdint.h>
#include "rfm.h"
#include "rfm98.h"
#include "spi.h"
#include "err.h"
#include "delay.h"


extern const uint8_t RFM98_CONFIG[][2];


/**
 * Configure RFM98 module.
 */
void rfm98_config() {
	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRa_SLEEP);
	delayMilliseconds(10);
	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRaMode );
	delayMilliseconds(10);
}

/**
 * Set operating mode. Use macro values RFM98_OPMODE_Mode_xxxx as arg.
 */
int rfm98_lora_mode(uint8_t mode) {
	rfm_register_write(RFM98_OPMODE, RFM98_OPMODE_LoRaMode | mode);

	// Wait until mode change is complete
	// IRQFLAGS1[7] ModeReady: Set to 0 when mode change, 1 when mode change complete
	return rfm_wait_for_bit_high(RFM98_IRQFLAGS1, RFM98_IRQFLAGS1_ModeReady);
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

    uint8_t frame_length;


    rfm_nss_assert();
    spi_transfer_byte(RFM98_FIFO);

	// Read frame length;
    frame_length = spi_transfer_byte(0);

    // Probably SPI bus problem
	if (frame_length == 0xff) {
		return E_SPI;
	}

    if (frame_length > 66) {
    	// error condition really
    	frame_length = 66;
    }

    for (i = 0; i < frame_length; i++) {
    	if (i == maxlen) {
    		return E_PKT_TOO_LONG;
    	}
    	buf[i] = spi_transfer_byte(0);
    }
    rfm_nss_deassert();

    return frame_length;
}

/**
 * Transmit a frame.
 */
void rfm98_frame_tx(uint8_t *buf, int len) {

	// Turn off receiver before writing to FIFO
	rfm98_lora_mode(RFM98_OPMODE_LoRa_STDBY);

	// Write frame to FIFO
	rfm_nss_assert();

	rfm_spi_transfer_byte(RFM98_FIFO | 0x80);

	// packet length
	rfm_spi_transfer_byte(len);

	int i;
	for (i = 0; i < len; i++) {
		rfm_spi_transfer_byte(buf[i]);
	}

	rfm_nss_deassert();

	// Power up TX
	rfm98_lora_mode(RFM98_OPMODE_LoRa_TX);

	rfm_wait_for_bit_high(RFM98_IRQFLAGS2, RFM98_IRQFLAGS2_PacketSent);
}
