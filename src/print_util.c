#include <stdint.h>
#include "print_util.h"
#include "myuart.h"

void print_decimal (LPC_USART_TypeDef *UARTx, int32_t i) {
	uint8_t buf[16];
	uint32_t j = 0;

	if (i == 0) {
		MyUARTSendByte(UARTx,'0');
		return;
	}

	if (i < 0) {
		MyUARTSendByte(UARTx,'-');
		i *= -1;
	}
	while (i > 0) {
		buf[j++] = '0' + i % 10;
		i /= 10;
	}
	while (j > 0) {
		MyUARTSendByte(UARTx,buf[--j]);
	}
}

void print_hex(LPC_USART_TypeDef *UARTx, uint32_t v) {
	int i, h;
	for (i = 28; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte(UARTx,'0' + h);
		} else {
			MyUARTSendByte(UARTx,'A' + h - 10);
		}
	}
}

void print_hex16(LPC_USART_TypeDef *UARTx, uint16_t v) {
	int i, h;
	for (i = 12; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte(UARTx,'0' + h);
		} else {
			MyUARTSendByte(UARTx,'A' + h - 10);
		}
	}
}
void print_hex8(LPC_USART_TypeDef *UARTx, uint8_t v) {
	int i, h;
	for (i = 4; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte(UARTx,'0' + h);
		} else {
			MyUARTSendByte(UARTx,'A' + h - 10);
		}
	}
}

