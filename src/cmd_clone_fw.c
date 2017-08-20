#include "config.h"
#include "parse_util.h"
#include "rfm.h"
#include "cmd.h"
#include "err.h"
#include "myuart.h"
#include "frame_buffer.h"

/**
 * Command to clone firmware to remote node.
 * Args: <to-node-addr>
 */
extern frame_buffer_type tx_buffer;
int cmd_fw_clone (int argc, uint8_t **argv) {

#ifdef FEATURE_CLONE_FW
	if (argc != 2) {
		return E_WRONG_ARGC;
	}

	// To  address
	tx_buffer.header.to_addr = parse_hex(argv[1]);


	tx_buffer.payload[0] = PKT_OTA_ENTER_BOOTLOADER;
	rfm_frame_tx (tx_buffer.buffer, 3+1 );


	int page;
	for (page = 1; page < ((1<<14)/64); i++) {
		// Query page CRC
		tx.buffer.payload[0] = PKT_OTA_FLASH_CRC_REQUEST;
		rfm_frame_tx (tx_buffer.buffer, 3+1 );

		// Listen for response

		// If different send lower and upper page and iterate
		// If same increment page.w2e

	}
	// Transmit frame
	rfm_frame_tx (tx_buffer.buffer,payload_len+2);

#endif

	return E_OK;
}
