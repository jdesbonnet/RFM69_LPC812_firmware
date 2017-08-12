#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include "config.h"
#include "params.h"
#include "myuart.h"
#include "cmd.h"
#include "err.h"


/**
 * Set parameter
 * Args: none
 */
int cmd_param (int argc, uint8_t **argv) {
/*
	if (argc == 2) {
		uint32_t param_index = parse_hex(argv[1]);
		tfp_printf("%x\r\n",params_union.params_buffer[param_index]);
		return E_OK;
	}

	if (argc != 3) {
		return E_WRONG_ARGC;
	}

	uint32_t param_index = parse_hex(argv[1]);
	uint32_t param_value = parse_hex(argv[2]);
	params_union.params_buffer[param_index] = param_value;
*/
	return E_OK;
}
