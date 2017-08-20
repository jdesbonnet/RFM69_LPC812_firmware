#include "config.h"
#include "myuart.h"
#include "parse_util.h"

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

	MyUARTxInit(LPC_USART0, bps);

	return E_OK;
}
