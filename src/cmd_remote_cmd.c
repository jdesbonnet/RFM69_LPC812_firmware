#include "config.h"

#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"
#include "frame_buffer.h"
#include "myuart.h"

/**
 * Command to transmit arbitrary packet.
 * Args: <to-node-addr> <packet-payload>
 */
extern frame_buffer_type tx_buffer;
int cmd_remote_cmd (int argc, uint8_t **argv) {

	int i,j,k;
	int cmd_len = 0;

	//if (payload_len >= 66) {
	//	return E_PKT_TOO_LONG;
	//}

	// To  address
	tx_buffer.header.to_addr = parse_hex(argv[1]);

	tx_buffer.buffer[2]='D'; // remote command packet type

	k = 3;
	for (i = 2; i < argc; i++) {
		for (j = 0; j < MyUARTGetStrLen(argv[i]); j++) {
			tx_buffer.buffer[k++] = argv[i][j];
		}
		tx_buffer.buffer[k++] = ' ';
	}

	// Replace trailing space with null
	tx_buffer.buffer[k-1] = 0;

	cmd_len = MyUARTGetStrLen(tx_buffer.buffer+3);

	//
	// Transmit frame
	//

	led_on();

	rfm69_frame_tx (tx_buffer.buffer,cmd_len+3);

	led_off();

	return E_OK;
}
