#include "config.h"
#include "myuart.h"
#include "cmd.h"
#include "err.h"


/**
 * Print RFMxx controller (this) firmware version
 * Args: none
 */
int cmd_version (int argc, uint8_t **argv) {
	tfp_printf("v %s\r\n",VERSION);
	return E_OK;
}
