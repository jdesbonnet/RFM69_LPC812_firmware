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
#include "err.h"

extern uint32_t flags;

/**
 * Set/get generic mode flags
 * Args: <node-addr>
 */
int cmd_flags (int argc, uint8_t **argv) {

	if (argc > 2) {
		return E_WRONG_ARGC;
	}

	if (argc==1) {
		MyUARTSendStringZ("f ");
		MyUARTPrintHex(flags);
		MyUARTSendCRLF();
		return;
	}

	// Node address
	flags  = parse_hex(argv[1]);

	return E_OK;
}
