#ifndef __PRINT_UTIL_H
#define __PRINT_UTIL_H

#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

void print_decimal (LPC_USART_TypeDef *UARTx, int32_t i);
void print_hex (LPC_USART_TypeDef *UARTx, uint32_t i);
void print_hex16 (LPC_USART_TypeDef *UARTx, uint16_t i);
void print_hex8 (LPC_USART_TypeDef *UARTx, uint8_t i);

#endif
