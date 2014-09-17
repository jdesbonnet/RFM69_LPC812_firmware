#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>

#include "myuart.h"
#include "print_util.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"


extern uint8_t node_addr;

/**
 * Set node address
 * Args: <node-addr>
 */
int cmd_set_uart_speed (int argc, uint8_t **argv) {

	if (argc != 2) {
		return E_WRONG_ARGC;
	}

	uint32_t bps = parse_hex(argv[1]);

	MyUARTInit(bps);
}
