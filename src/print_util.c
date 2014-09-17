#include <stdint.h>
#include "print_util.h"
#include "myuart.h"

void print_decimal (int32_t i) {
	uint8_t buf[16];
	uint32_t j = 0;

	if (i == 0) {
		MyUARTSendByte('0');
		return;
	}

	if (i < 0) {
		MyUARTSendByte('-');
		i *= -1;
	}
	while (i > 0) {
		buf[j++] = '0' + i % 10;
		i /= 10;
	}
	while (j > 0) {
		MyUARTSendByte(buf[--j]);
	}
}

void print_hex(uint32_t v) {
	int i, h;
	for (i = 28; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte('0' + h);
		} else {
			MyUARTSendByte('A' + h - 10);
		}
	}
}

void print_hex16(uint16_t v) {
	int i, h;
	for (i = 12; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte('0' + h);
		} else {
			MyUARTSendByte('A' + h - 10);
		}
	}
}
void print_hex8(uint8_t v) {
	int i, h;
	for (i = 4; i >= 0; i -= 4) {
		h = (v >> i) & 0x0f;
		if (h < 10) {
			MyUARTSendByte('0' + h);
		} else {
			MyUARTSendByte('A' + h - 10);
		}
	}
}

