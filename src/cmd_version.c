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

	MyUARTSendStringZ("v ");
	MyUARTSendStringZ(VERSION);
	MyUARTSendCRLF(LPC_USART0);
}
