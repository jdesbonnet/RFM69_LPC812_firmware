#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>
#include <stdint.h>

#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"
#include "myuart.h"
#include "frame_buffer.h"

/**
 * Command to query node
 * Args: <to-node-addr>
 */
extern frame_buffer_type tx_buffer;
int cmd_node_query (int argc, uint8_t **argv) {

	if (argc < 2) {
		return E_WRONG_ARGC;
	}

	// To  address
	tx_buffer.header.to_addr = parse_hex(argv[1]);
	tx_buffer.buffer[2] = 'R';

	// Transmit frame
	rfm_frame_tx (tx_buffer.buffer,3);

	return E_OK;
}
