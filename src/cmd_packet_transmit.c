#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>
#include <stdint.h>

#include "myuart.h"
#include "print_util.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"

extern uint8_t node_addr;

/**
 * Command to transmit arbitrary packet.
 * Args: <to-node-addr> <packet-payload>
 */
int cmd_packet_transmit (int argc, uint8_t **argv) {

	if (argc != 3) {
		return E_WRONG_ARGC;
	}

	uint32_t payload_len = strlen(argv[2]) / 2;

	if (payload_len >= 66) {
		return E_PKT_TOO_LONG;
	}

	uint8_t buf[payload_len+2];

	// To  address
	buf[0] = parse_hex(argv[1]);

	// From address
	buf[1] = node_addr;

	int i;
	uint8_t tt[3];
	tt[3]=0;
	for (i = 0; i < payload_len; i++) {
		tt[0] = argv[2][i*2];
		tt[1] = argv[2][i*2+1];
		buf[i+2] = parse_hex(tt);
	}

	// Transmit frame
	rfm69_frame_tx (buf,payload_len+2);

	return E_OK;
}
