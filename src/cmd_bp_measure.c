#include "config.h"

/**
 * Command to make BP measurement.
 */
int cmd_bp_measure (int argc, uint8_t **argv) {
	bp_record_t bp;
	abpm_init();
	int bp_status = abpm_measure(&bp);
	if (bp_status != 0) {
		tfp_printf("; error listening for BP record\r\n");
		return E_TIMEOUT;
	}
	transmit_bp_packet(&bp);
	return E_OK;
}
