#ifndef __PRINT_UTIL_H
#define __PRINT_UTIL_H

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

void print_decimal (int32_t i);
void print_hex (uint32_t i);
void print_hex16 (uint16_t i);
void print_hex8 (uint8_t i);

#endif
