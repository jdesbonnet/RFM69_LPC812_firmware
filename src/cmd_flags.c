#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>

#include "myuart.h"
#include "print_util.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"

extern uint32_t flags;

/**
 * Enter/leave promiscuous mode
 * Args: <node-addr>
 */
int cmd_flags (int argc, uint8_t **argv) {

	if (argc != 2) {
		return E_WRONG_ARGC;
	}

	if (argv[1][0]=='?') {
		MyUARTSendStringZ(LPC_USART0,"f ");
		MyUARTPrintHex(LPC_USART0, flags);
		MyUARTSendCRLF(LPC_USART0);
		return;
	}

	// Node address
	flags  = parse_hex(argv[1]);

	return E_OK;
}
