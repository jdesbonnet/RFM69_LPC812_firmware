#include <string.h>

#include "parse_util.h"
#include "cmd.h"
#include "err.h"
#include "frame_buffer.h"

/**
 * Query/set node address
 * Args: <node-addr>
 */
extern frame_buffer_type tx_buffer;
int cmd_set_node_addr (int argc, uint8_t **argv) {

	if (argc == 1) {
		tfp_printf("n %x\r\n",tx_buffer.header.from_addr);
		return E_OK;
	}
	if (argc != 2) {
		return E_WRONG_ARGC;
	}
	tx_buffer.header.from_addr=parse_hex(argv[1]);
	return E_OK;
}
