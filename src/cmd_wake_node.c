#include "config.h"

#include "parse_util.h"
#include "cmd.h"

/**
 * Command to transmit multiple packets in a given mode
 * Args: <to-node-addr> <bw> <spread-factor> <error-coding-rate> <number-of-packets> <delay-ms>
 */
extern uint8_t wake_list[];
int cmd_wake_node (int argc, uint8_t **argv) {


	if (argc != 2) {
		debug("<node>");
		return E_WRONG_ARGC;
	}

	int node = parse_hex(argv[1]);

	int i;
	for (i = 0; i < 4; i++) {
		if (wake_list[i]==0) {
			wake_list[i] = node;
			debug ("node %x on wake list", node);
			return E_OK;
			break;
		}
	}
	debug ("no wake slots");

	return E_NO_WAKE_SLOTS;
}
