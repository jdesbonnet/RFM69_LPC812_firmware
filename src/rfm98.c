/**
 * RFM69HW (HopeRF) radio module driver for WRSC2014.
 *
 * First (pre)release 4 Sep 2014. Please check back in a few days for an update.
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
	rfm_register_write(RFM98_OPMODE_Mode_SLEEP);
	delayMilliseconds(10);
	rfm_register_write(RFM98_OPMODE,RFM98_OPMODE_LoRaMode );
	delayMilliseconds(10);

}
