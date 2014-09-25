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

//extern uint8_t node_addr;

/**
 * Command to transmit arbitrary packet.
 * Args: <to-node-addr> <packet-payload>
 */
extern frame_buffer_type tx_buffer;
int cmd_packet_transmit (int argc, uint8_t **argv) {

	if (argc != 3) {
		return E_WRONG_ARGC;
	}


	// Packet payload in hex, so divide by two to get byte count
	uint32_t payload_len = MyUARTGetStrLen(argv[2]) / 2;

	if (payload_len >= 66) {
		return E_PKT_TOO_LONG;
	}

	// To  address
	tx_buffer.header.to_addr = parse_hex(argv[1]);

	int i;
	uint8_t tt[3];
	tt[3]=0;
	for (i = 0; i < payload_len; i++) {
		tt[0] = argv[2][i*2];
		tt[1] = argv[2][i*2+1];
		tx_buffer.buffer[i+2] = parse_hex(tt);
	}

	// Transmit frame
	rfm69_frame_tx (tx_buffer.buffer,payload_len+2);

	return E_OK;
}
