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
#include "rfm69.h"
#include "spi.h"
#include "err.h"
#include "delay.h"


extern const uint8_t RFM98_CONFIG[][2];


/**
 * Configure RFM69 radio module for use. Assumes SPI interface is already configured.
 */
void rfm98_config() {
	//int i;
	//for (i = 0; RFM69_CONFIG[i][0] != 255; i++) {
	//    rfm_register_write(RFM69_CONFIG[i][0], RFM69_CONFIG[i][1]);
	//}
}
