#include "config.h"
#include "parse_util.h"
#include "rfm.h"
#include "cmd.h"
#include "err.h"
#include "myuart.h"
#include "frame_buffer.h"

/**
 * Command to transmit arbitrary packet.
 * Args: <to-node-addr> <packet-payload>
 */
extern frame_buffer_type tx_buffer;
int cmd_packet_transmit (int argc, uint8_t **argv) {

	if (argc != 3) {
		return E_WRONG_ARGC;
	}

	// To  address
	tx_buffer.header.to_addr = parse_hex(argv[1]);


	uint8_t *payload = argv[2];

	int i=0;
	int payload_len;


	// Two formats supported T nn "remote cmd text" eg T 44 "M 2"
	// T nn payload-in-hex eg T 44 472053
	if (*payload=='"') {
		payload++;
		while (*payload != '"' && *payload != 0) {
			debug("payload=%c",*payload);
			tx_buffer.buffer[2+i++] = *(payload++);
			if (i > RXTX_BUFFER_SIZE) {
				return E_PKT_TOO_LONG;
			}
		}
		payload_len = i;
	} else {

		// Packet payload in hex, so divide by two to get byte count
		payload_len = MyUARTGetStrLen(argv[2]) / 2;

		if (payload_len >= RXTX_BUFFER_SIZE) {
			return E_PKT_TOO_LONG;
		}

		uint8_t tt[3];
		tt[2]=0;
		for (i = 0; i < payload_len; i++) {
			tt[0] = argv[2][i*2];
			tt[1] = argv[2][i*2+1];
			tx_buffer.buffer[i+2] = parse_hex(tt);
		}
	}

	//for (i = 0; i < payload_len; i++) {
		//debug("payload[%d]=%x",i,tx_buffer.buffer[i]);
	//}

	// Transmit frame
	rfm_frame_tx (tx_buffer.buffer,payload_len+2);

	return E_OK;
}
