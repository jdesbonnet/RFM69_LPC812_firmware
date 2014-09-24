#include <stdint.h>
#include <stdlib.h>
#include "parse_util.h"

/**
 * Return the numeric value of a hex digit or -1 if not a hex digit.
 */
static int is_hex_digit(uint8_t c) {
	if (c>='0' && c<='9') {
		return c - '0';
	}

	// This bit flip trick saves 12 bytes (converts upper to lowercase)
	c |= (1<<5);
	if (c>='a' && c<='f') {
		return c - 'a' + 10;
	}
	/*
	if (c>='A' && c<='F') {
		return c - 'A' + 10;
	}
	*/
	return -1;
}

uint32_t parse_dec_or_hex (uint8_t *buf) {
	if (buf[0]=='0' && buf[1]=='x') {
		return parse_hex(buf+2);
	} else {
		return atoi (buf);
	}
}

uint32_t parse_hex (uint8_t *buf) {
	int d=0,res=0;

	while ( (d=is_hex_digit(*buf++)) != -1 ) {
		res <<= 4;
		res |= d;
	}
	return res;
}

int32_t parse_dec(uint8_t *buf, uint8_t **end) {
	int v=0;
	while (isDigit(*buf)) {
		v *= 10;
		v += (*buf - '0');
		buf++;
	}
	*end = buf;
	return v;
}




