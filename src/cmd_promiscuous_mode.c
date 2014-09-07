#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>

#include "lpc8xx_spi.h"
//#include "lpc8xx_uart.h"
#include "myuart.h"
#include "print_util.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"

extern uint8_t promiscuous_mode;

/**
 * Enter/leave promiscuous mode
 * Args: <node-addr>
 */
int cmd_promiscuous_mode (int argc, uint8_t **argv) {

	if (argc != 2) {
		return E_WRONG_ARGC;
	}

	// Node address
	int i  = parse_hex(argv[1]);
	if (i == 0 || i == 1) {
		promiscuous_mode = i;
	} else {
		return E_INVALID_ARG;
	}

	return E_OK;
}
