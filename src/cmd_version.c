#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include "config.h"
#include "myuart.h"
#include "cmd.h"


/**
 * Print RFM69 controller (this) firmware version
 * Args: none
 */
int cmd_version (int argc, uint8_t **argv) {

	MyUARTSendStringZ(LPC_USART0,"v ");
	MyUARTSendStringZ(LPC_USART0,VERSION);
	MyUARTSendCRLF(LPC_USART0);
}
