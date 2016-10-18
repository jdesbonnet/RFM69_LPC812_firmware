#include "config.h"

#include "parse_util.h"
#include "cmd.h"
#include "frame_buffer.h"
#include "delay.h"

//extern uint8_t node_addr;

/**
 * Command to transmit multiple packets in a given mode
 * Args: <to-node-addr> <bw> <spread-factor> <error-coding-rate> <number-of-packets> <delay-ms>
 */
extern frame_buffer_type tx_buffer;
int cmd_tx_test (int argc, uint8_t **argv) {

#ifdef RADIO_RFM9x

	if (argc != 8) {
		debug("<node> <bw> <sf> <ecr> <npacket> <delay> <reset>");
		return;
	}

	int node = parse_hex(argv[1]);
	int bw = parse_hex(argv[2]);
	int sf = parse_hex(argv[3]);
	int cr = parse_hex(argv[4]);
	int n = parse_hex(argv[5]);
	int delay = parse_hex(argv[6]);
	int reset = parse_hex(argv[7]);

	debug("bw=%d sf=%d cr=%d n=%d delay=%d reset=%d", bw,sf,cr,n,delay,reset);

	rfm_register_write(RFM98_MODEMCONFIG1,
			RFM98_MODEMCONFIG1_Bw_VALUE(bw)
			| RFM98_MODEMCONFIG1_CodingRate_VALUE(cr)
	);

	rfm_register_write(RFM98_MODEMCONFIG2,
			RFM98_MODEMCONFIG2_SpreadFactor_VALUE(sf)
	);

	int i;
	for (i = 0; i < n; i++) {
		tx_buffer.buffer[0] = (i>>8) & 0xff;
		tx_buffer.buffer[1] = i & 0xff;;

		debug ("transmitting test packet %d of %d",i,n);

		ledOn();
		rfm98_frame_tx(tx_buffer.buffer,14);
		ledOff();

		delayMilliseconds(delay);
	}
#else
	debug("not supported for RFM69");
#endif


	if (reset) {
		NVIC_SystemReset();
	}

	return E_OK;
}
