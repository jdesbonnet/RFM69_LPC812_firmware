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

extern uint8_t node_addr;

/**
 * Set node address
 * Args: <node-addr>
 */
int cmd_set_node_addr (int argc, uint8_t **argv) {

	if (argc != 2) {
		return E_WRONG_ARGC;
	}

	// Node address
	node_addr = parse_hex(argv[1]);

}
