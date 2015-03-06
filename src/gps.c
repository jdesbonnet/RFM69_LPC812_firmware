
#include <string.h>

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */

#include "config.h"
#include "myuart.h"

#include "params.h"
extern params_union_type params_union;

#ifdef FEATURE_GPS_ON_USART1
//volatile uint32_t nmea_line = 0;
volatile uint32_t gps_last_position_t;
volatile uint32_t nmea_buf_index = 0;
//volatile uint32_t nmea_flags = 0;
volatile uint8_t *nmea_words[24];
volatile int32_t nmea_word_count=0;
volatile uint8_t gps_latitude[12];
volatile uint8_t gps_longitude[12];
volatile uint8_t gps_time_of_day[12];
volatile uint8_t gps_hdop[8];
volatile uint8_t gps_fix[2];
volatile uint8_t nmea_buf[GPS_NMEA_SIZE];
#endif



#ifdef FEATURE_GPS_ON_USART1
/**
 * Second UART (USART1) interrupt handler to handle incoming NMEA sentences
 * from GPS receiver. (Optional feature).
 */
void UART1_IRQHandler(void)
{
	uint32_t uart_status = LPC_USART1->STAT;

	// UM10601 §15.6.3, Table 162, p181. USART Status Register.
	// Bit 0 RXRDY: 1 = data is available to be read from RXDATA
	// Bit 2 TXRDY: 1 = data may be written to TXDATA
	if (uart_status & UART_STAT_RXRDY ) {

		uint8_t c = LPC_USART1->RXDATA;

		// If CR flag EOL
		if (c=='\r') {
			//nmea_flags |= UART_BUF_FLAG_EOL;
			nmea_buf[nmea_buf_index]=0; // zero-terminate buffer
			//nmea_line++;
			nmea_buf_index = 0;

			nmea_word_count = 0;

			if (nmea_buf[3]=='G' && nmea_buf[4] == 'G' && nmea_buf[5] == 'A') {

				extern uint32_t systick_counter;
				gps_last_position_t = systick_counter;

				// Make copy from NMEA buffer to avoid data being clobbered in the background by incoming chars
				memcpy (gps_time_of_day, nmea_words[0],(nmea_words[1]-nmea_words[0])-1);
				memcpy (gps_latitude, nmea_words[1],(nmea_words[2]-nmea_words[1])-1);
				memcpy (gps_longitude, nmea_words[3],(nmea_words[4]-nmea_words[3])-1);
				memcpy (gps_fix, nmea_words[5], (nmea_words[6]-nmea_words[5])-1 );
				memcpy (gps_hdop, nmea_words[7], (nmea_words[8]-nmea_words[7])-1 );

				/*
				MyUARTSendStringZ("g ");
				MyUARTSendStringZ(&latitude);
				MyUARTSendStringZ(" ");
				MyUARTSendStringZ(&longitude);
				MyUARTSendCRLF();
				*/
			}

		} else if (c>31){
			nmea_buf[nmea_buf_index++] = c;

			if (nmea_buf_index == GPS_NMEA_SIZE) {
				nmea_buf[GPS_NMEA_SIZE-1]=0;
				nmea_buf_index = 0;
			}
			if (c == ',') {
				//nmea_buf[nmea_buf_index] = c;
				nmea_words[nmea_word_count++] = &nmea_buf[nmea_buf_index];
			} else {
				//nmea_buf[nmea_buf_index] = c;
			}

			//nmea_buf_index++;

		}

		if (params_union.params.gps_echo) {
			LPC_USART0->TXDATA = c;
		}

	} else if (uart_status & UART_STAT_TXRDY ){
		LPC_USART1->INTENCLR = 0x04;
	}
}
#endif
