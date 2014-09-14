/**
 * RFM69HW (HopeRF) radio module driver for WRSC2014.
 *
 * First (pre)release 4 Sep 2014. Please check back in a few days for an update.
 *
 * Joe Desbonnet, jdesbonnet@gmail.com
 */

#include <stdint.h>
#include "rfm69.h"
#include "err.h"


extern const uint8_t RFM69_CONFIG[][2];

/**
 * Configure RFM69 radio module for use. Assumes SPI interface is already configured.
 */
void rfm69_config() {
	int i;
	for (i = 0; RFM69_CONFIG[i][0] != 255; i++) {
	    rfm69_register_write(RFM69_CONFIG[i][0], RFM69_CONFIG[i][1]);
	}
}

/**
 * Set RFM69 operating mode. Use macro values RFM69_OPMODE_Mode_xxxx as arg.
 */
void rfm69_mode(uint8_t mode) {
	uint8_t regVal = rfm69_register_read(RFM69_OPMODE);
	regVal &= ~RFM69_OPMODE_Mode_MASK;
	regVal |= RFM69_OPMODE_Mode_VALUE(mode);
	rfm69_register_write(RFM69_OPMODE,regVal);

	// Wait until mode change is complete
	// IRQFLAGS1[7] ModeReady: Set to 0 when mode change, 1 when mode change complete
	while ( (rfm69_register_read(RFM69_IRQFLAGS1) & RFM69_IRQFLAGS1_ModeReady) == 0) ;

}

/**
 * Get RSSI
 */
uint8_t rfm69_rssi () {

	rfm69_register_write(RFM69_RSSICONFIG, RFM69_RSSICONFIG_RssiStart);

	// Wait for RSSI ready
	while ((rfm69_register_read(RFM69_RSSICONFIG) & RFM69_RSSICONFIG_RssiDone_MASK) == 0x00);
	return rfm69_register_read(RFM69_RSSIVALUE);
}

/**
 * Check if packet has been received and is ready to read from FIFO.
 * @return zero if no packet available, non-zero if packet available.
 */
uint8_t rfm69_payload_ready() {
	return rfm69_register_read(RFM69_IRQFLAGS2) & RFM69_IRQFLAGS2_PayloadReady_MASK;
}

/**
 * Retrieve a frame. If successful returns length of frame. If not an error code (negative value).
 * Frame is returned in buf but will not exceed length maxlen.
 *
 * @return frame_length if successful, else a negative value error code
 * Error codes:
 * -2 : frame too long
 */
int rfm69_frame_rx(uint8_t *buf, int maxlen, uint8_t *rssi) {

	int i;

	// Set mode RX
	// REG_OP_MODE §6.2, page 63
	// OP_MODE[4:2] Mode 0x4 = RX

	/*
	uint8_t regVal = rfm69_register_read(RFM69_OPMODE);
	regVal &= 0xE3;
	regVal |= 0x4 << 2;
	rfm69_register_write(RFM69_OPMODE,regVal);
	*/
	// TODO: this shouldn't be necessary
	rfm69_mode(RFM69_OPMODE_Mode_RX);


	// Wait for IRQFLAGS2[2] PayloadReady
	// TODO: implement timeout
	while ((rfm69_register_read(RFM69_IRQFLAGS2) & RFM69_IRQFLAGS2_PayloadReady_MASK) == 0) ;

    uint8_t frame_length;


    rfm69_nss_assert();
    spi_transfer_byte(RFM69_FIFO);

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
    rfm69_nss_deassert();

    // If pointer to rssi given, fetch it
    if (rssi != 0) {
    	*rssi = rfm69_rssi();
    }

    return frame_length;
}

/**
 * Transmit a frame.
 */
void rfm69_frame_tx(uint8_t *buf, int len) {

	// Turn off receiver before writing to FIFO
	// @register OPMODE
	// REG_OP_MODE §6.2, page 63
	/*
	uint8_t regVal = rfm69_register_read(RFM69_OPMODE);
	regVal |= RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_STDBY);
	rfm69_register_write(RFM69_OPMODE,regVal);
	*/
	rfm69_mode(RFM69_OPMODE_Mode_STDBY);



	// Write frame to FIFO
	rfm69_nss_assert();

	rfm69_spi_transfer_byte(RFM69_FIFO | 0x80);

	// packet length
	rfm69_spi_transfer_byte(len);

	int i;
	for (i = 0; i < len; i++) {
		rfm69_spi_transfer_byte(buf[i]);
	}

	rfm69_nss_deassert();

	// Power up TX
	// REG_OP_MODE §6.2, page 63
	/*
	regVal = rfm69_register_read(RFM69_OPMODE);
	regVal |= RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_TX);
	rfm69_register_write(RFM69_OPMODE,regVal);
	*/
	rfm69_mode(RFM69_OPMODE_Mode_TX);


	// REG_IRQFLAGS2 page 70
	// IRQFLAGS2[3] PacketSent 1 when complete packet sent. Cleared when existing TX mode.
	while ( (rfm69_register_read(RFM69_IRQFLAGS2) & RFM69_IRQFLAGS2_PacketSent_MASK) == 0x00){
		// TODO: implement timeout
	}

	// TODO: we shouldn't need a delay
	//delay (10000);

	// Back to receive mode
	// REG_OP_MODE §6.2, page 63
	// OP_MODE[4:2] Mode 0x1 = STDBY
	// OP_MODE[4:2] Mode 0x4 = RX
	/*
	regVal = rfm69_register_read(RFM69_OPMODE);
	regVal &= ~RFM69_OPMODE_Mode_MASK;
	regVal |= RFM69_OPMODE_Mode_VALUE(RFM69_OPMODE_Mode_RX);
	rfm69_register_write(RFM69_OPMODE,regVal);
	*/
	rfm69_mode(RFM69_OPMODE_Mode_RX);

	// Wait until STDBY mode ready
	// IRQFLAGS1[7] ModeReady: Set to 0 when mode change, 1 when mode change complete
	//while ( (rfm69_register_read(RFM69_IRQFLAGS1) & RFM69_IRQFLAGS1_ModeReady) == 0) ;


}

uint8_t rfm69_register_read (uint8_t reg_addr) {
	rfm69_nss_assert();
	rfm69_spi_transfer_byte(reg_addr);
	uint8_t reg_value = rfm69_spi_transfer_byte(0xff);
	rfm69_nss_deassert();
	return reg_value;
}

void rfm69_register_write (uint8_t reg_addr, uint8_t reg_value) {
	rfm69_nss_assert();
	rfm69_spi_transfer_byte (reg_addr | 0x80); // Set bit 7 to indicate write op
	rfm69_spi_transfer_byte (reg_value);
	rfm69_nss_deassert();
}
