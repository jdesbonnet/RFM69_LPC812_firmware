/*
 * myuart.c
 *
 *  Created on: 30 Jun 2013
 *      Author: joe
 */

#include <string.h>

#include "config.h"
#include "myuart.h"

static volatile uint8_t uart_rxbuf[UART_BUF_SIZE];
static volatile uint32_t uart_rxi=0;
static volatile uint32_t uart_buf_flags=0;



/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
**
*****************************************************************************/
void MyUARTxInit(LPC_USART_T *UARTx, uint32_t baudrate)
{


	//LPC_USART_TypeDef *UARTx = LPC_USART0;

	// Assign pins: use same assignment as serial bootloader
	/*
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, PIN_UART_TXD);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, PIN_UART_RXD);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
	*/


#define NEW_UART_INIT


#ifdef NEW_UART_INIT
	Chip_UART_Init(LPC_USART0);
	Chip_UART_ConfigData(LPC_USART0,
			UART_CFG_DATALEN_8
			| UART_CFG_PARITY_NONE
			| UART_CFG_STOPLEN_1);

	Chip_Clock_SetUSARTNBaseClockRate((UART_BAUD_RATE * 16), true);
	Chip_UART_SetBaud(LPC_USART0, UART_BAUD_RATE);
	Chip_UART_TXEnable(LPC_USART0);
	Chip_UART_Enable(LPC_USART0);
#endif

#ifdef OLD_UART_INIT
	uint32_t UARTSysClk;

	//UARTClock_Init( UARTx );
	LPC_SYSCON->UARTCLKDIV = 1;     /* divided by 1 */

	if (UARTx == LPC_USART0) {
		NVIC_DisableIRQ(UART0_IRQn);
	} else if (UARTx == LPC_USART1) {
		NVIC_DisableIRQ(UART1_IRQn);
	}

	/* Enable UART clock */
	if (UARTx == LPC_USART0) {
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14);
	} else if (UARTx == LPC_USART1) {
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);
	}

	/* Peripheral reset control to UART, a "1" bring it out of reset. */
	//LPC_SYSCON->PRESETCTRL &= ~(0x1<<3);
	//LPC_SYSCON->PRESETCTRL |= (0x1<<3);
	if (UARTx == LPC_USART0) {
		lpc8xx_peripheral_reset(RESET_USART0);
	} else if (UARTx == LPC_USART1) {
		lpc8xx_peripheral_reset(RESET_USART1);
	}

	UARTSysClk = SystemCoreClock/LPC_SYSCON->UARTCLKDIV;
	UARTx->CFG = UART_CFG_DATA_LENG_8|UART_CFG_PARITY_NONE|UART_CFG_STOP_BIT_1; /* 8 bits, no Parity, 1 Stop bit */
	UARTx->BRG = UARTSysClk/16/baudrate-1;	/* baud rate */
		/*
			Integer divider:
			BRG = UARTSysClk/(Baudrate * 16) - 1
			Frational divider:
			FRG = ((UARTSysClk / (Baudrate * 16 * (BRG + 1))) - 1) where FRG = (LPC_SYSCON->UARTFRDADD + 1) / (LPC_SYSCON->UARTFRDSUB + 1)
		*/
		/*	(1) The easist way is set SUB value to 256, -1 encoded, thus SUB register is 0xFF.
				(2) In ADD register value, depending on the value of UartSysClk, baudrate, BRG register value, and SUB register value, be careful
				about the order of multiplyer and divider and make sure any multiplyer doesn't exceed 32-bit boundary and any divider doesn't get
				down below one(integer 0).
				(3) ADD should be always less than SUB. */
	LPC_SYSCON->UARTFRGDIV = 0xFF;
	LPC_SYSCON->UARTFRGMULT = (((UARTSysClk / 16) * (LPC_SYSCON->UARTFRGDIV + 1)) / (baudrate * (UARTx->BRG + 1))) - (LPC_SYSCON->UARTFRGDIV + 1);

	UARTx->STAT = UART_STAT_CTS_DELTA | UART_STAT_DELTA_RXBRK;		/* Clear all status bits. */

#endif

	// Enable UART interrupt
	if (UARTx == LPC_USART0) {
		NVIC_EnableIRQ(UART0_IRQn);
	} else if (UARTx == LPC_USART1) {
		NVIC_EnableIRQ(UART1_IRQn);
	}

	/* Enable UART interrupt on receving byte */
	//UARTx->INTENSET = UART_STAT_RXRDY | UART_STAT_TXRDY | UART_STAT_DELTA_RXBRK;
	UARTx->INTENSET = UART_STAT_RXRDY;

	UARTx->CFG |= UART_CFG_UART_EN;

	return;
}

/*
void MyUARTInit(uint32_t baudrate)
{
	MyUARTxInit (LPC_USART0,baudrate);
	MyUARTxInit (LPC_USART1,baudrate);
}
*/

RAM_FUNC
void MyUARTSendByte (uint8_t v) {
	  // wait until data can be written to TXDATA
	  while ( ! (LPC_USART0->STAT & (1<<2)) );
	  LPC_USART0->TXDATA = v;
}

void MyUARTSendDrain () {
	// Wait for TXIDLE flag to be asserted
	while ( ! (LPC_USART0->STAT & (1<<3)) );
}

/**
 * Send zero-terminated string.
 */
RAM_FUNC
void MyUARTSendStringZ (char *buf) {
	while (*buf != 0) {
		MyUARTSendByte(*buf);
		buf++;
	}
}

RAM_FUNC
void MyUARTSendCRLF(void) {
	MyUARTSendByte('\r');
	MyUARTSendByte('\n');
}

void UART0_IRQHandler(void)
{

	uint32_t uart_status = LPC_USART0->STAT;

	// UM10601 §15.6.3, Table 162, p181. USART Status Register.
	// Bit 0 RXRDY: 1 = data is available to be read from RXDATA
	// Bit 2 TXRDY: 1 = data may be written to TXDATA
	if (uart_status & UART_STAT_RXRDY ) {

		uint8_t c = LPC_USART0->RXDATA;

		// If CR flag EOL
		if (c=='\r') {
			uart_buf_flags |= UART_BUF_FLAG_EOL;
			uart_rxbuf[uart_rxi]=0; // zero-terminate buffer
		} else if (c>31){
			// echo
			MyUARTSendByte(c);

			uart_rxbuf[uart_rxi] = c;
			uart_rxi++;
			if (uart_rxi == UART_BUF_SIZE) {
				MyUARTBufReset();
			}
		}

	} else if (uart_status & UART_STAT_TXRDY ){

		LPC_USART0->INTENCLR = 0x04;
	}

}

uint8_t* MyUARTGetBuf(void) {
	return (uint8_t*)uart_rxbuf;
}

uint32_t MyUARTGetBufIndex(void) {
	return (uint32_t)uart_rxi;
}

uint32_t MyUARTGetBufFlags(void) {
	return uart_buf_flags;
}

void MyUARTSetBufFlags(uint32_t flags) {
	uart_buf_flags = flags;
}

uint32_t MyUARTBufCopy(uint8_t *buf) {
	//memcpy(buf, (void*)uart_rxbuf, uart_rxi);
	int i;
	for (i = 0; i < uart_rxi; i++) {
		buf[i]=uart_rxbuf[i];
	}
	buf[i]=0; // Zero terminate
	return uart_rxi;
}

/**
 * Reset command buffer after command has been processed.
 */
void MyUARTBufReset() {
	uart_rxi=0;

	// Zero the buffer
	// The loop is more space efficient than using memset()
	//memset((void*)uart_rxbuf,0,UART_BUF_SIZE);
	int i;
	for (i = 0; i < UART_BUF_SIZE; i++) {
		uart_rxbuf[i]=0;
	}

	// Reset flags
	uart_buf_flags = 0;
}


void MyUARTPrintDecimal (int32_t i) {
	uint8_t buf[16];
	uint32_t j=0;

	if (i==0) {
		MyUARTSendByte('0');
		return;
	}

	if (i<0) {
		MyUARTSendByte('-');
		i *= -1;
	}
	while (i>0) {
		buf[j++] = '0' + i%10;
		i /= 10;
	}
	while (j>0) {
		MyUARTSendByte(buf[--j]);
	}
}

void MyUARTPrintHex (uint32_t v) {
	// 36 bytes long
	int i,h;
	for (i = 28; i >=0 ; i-=4) {
		h = (v>>i) & 0x0f;
		if (h<10) {
			MyUARTSendByte('0'+h);
		} else{
			MyUARTSendByte('A'+h-10);
		}
	}
}

RAM_FUNC
void MyUARTPrintHex8 (uint8_t v) {
	int i,h;
	for (i = 4; i >=0 ; i-=4) {
		h = (v>>i) & 0x0f;
		if (h<10) {
			MyUARTSendByte('0'+h);
		} else{
			MyUARTSendByte('A'+h-10);
		}
	}
}

#ifdef BLAH
int parse_dec(uint8_t *buf, uint8_t **end) {
	int v=0;
	while (isDigit(*buf)) {
		v *= 10;
		v += (*buf - '0');
		buf++;
	}
	*end = buf;
	return v;
}
#endif

#ifdef BLAH
void print_dec(uint8_t *buf, uint32_t v) {
	if (v==0) {
		*buf='0';
		*(buf+1)=0;
		return;
	}
	uint8_t *s = buf;
	while (v>0) {
		*s++ = '0' + (v%10);
		v /= 10;
	}
	*s=0;

	// reverse
	int len = s - buf;
	int i;
	uint8_t t;
	for (i = 0; i < len/2; i++) {
		s--;
		t = *s;
		*s = *buf;
		*buf = t;
		*buf++;
	}

}
#endif

/**
 * Return 1 if v is a decimal digit. Else return 0.
 */
int isDigit (uint8_t v) {
	return (v>='0'&&v<='9') ? 1:0;
}

/**
 * Return zero terminated string length
 */
int MyUARTGetStrLen (uint8_t *s) {
	int len=0;
	while (*s++) len++;
	return len;
}

