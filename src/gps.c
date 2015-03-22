
#include <string.h>

#include "LPC8xx.h"			/* LPC8xx Peripheral Registers */

#include "config.h"
#include "myuart.h"

#include "params.h"
#include "frame_buffer.h"

#ifdef FEATURE_GPS_ON_USART1


extern params_union_type params_union;
extern uint32_t systick_counter;
extern frame_buffer_type tx_buffer;


//volatile uint32_t nmea_line = 0;
volatile uint32_t gps_last_position_t;
volatile uint32_t nmea_buf_index = 0;
//volatile uint32_t nmea_flags = 0;
volatile uint8_t nmea_fields[24];        // NMEA buffer field byte offset
volatile int32_t nmea_field_count=0;
volatile uint8_t gps_latitude[12];
volatile uint8_t gps_longitude[12];
volatile uint8_t gps_time_of_day[12];
volatile uint8_t gps_hdop[8];
volatile uint8_t gps_fix[2];
volatile uint8_t gps_heading[8];          // heading in degrees (0=North)
volatile uint8_t gps_speed[8];            // speed from NMEA sentence (in knots)
volatile uint8_t nmea_buf[GPS_NMEA_SIZE]; // buffer for incoming NMEA sentences
volatile uint32_t gps_top_of_second_t;    // systick time of top of last UTC second
static volatile uint32_t gps_dollar_t;    // systick time at which NMEA '$' received


uint32_t gps_get_last_position_t () {
	return gps_last_position_t;
}

void displayGPS () {
	tfp_printf ("g %d %s %s %s %s %s %s %s\r\n",
			//(systick_counter - gps_last_position_t),
			(systick_counter - gps_top_of_second_t),
			&gps_time_of_day, &gps_latitude, &gps_longitude,
			&gps_heading, &gps_speed,
			&gps_fix, &gps_hdop);
}

void sendGPSUpdate (uint8_t to_addr) {

	// report position
	tx_buffer.header.to_addr = to_addr;
	tx_buffer.header.msg_type = 'r';

	int n;
	strcpy (tx_buffer.payload,gps_time_of_day);
	n  = strlen(gps_time_of_day);
	tx_buffer.payload[n] = ' ';
	n++;
	strcpy (tx_buffer.payload+n,gps_latitude);
	n += strlen(gps_latitude);
	tx_buffer.payload[n] = ' ';
	n++;
	strcpy (tx_buffer.payload+n,gps_longitude);
	n += strlen(gps_longitude);

	//tx_buffer.payload[n] = rssi;
	LPC_GPIO_PORT->PIN0 |= (1<<LED_PIN);
	rfm69_frame_tx(tx_buffer.buffer, n+4);
	LPC_GPIO_PORT->PIN0 &= ~(1<<LED_PIN);
}


/**
 * Copy a data field from NMEA sentence to address dst. Like strcpy()
 * except terminating on comma ','.
 */
static void copy_nmea_field (uint8_t *dst, uint8_t *src) {
	while (*src != ',') {
		*dst = *src;
		dst++;
		src++;
	}
	// Terminate with 0
	*dst = 0;
}

/**
 * Get length of a NMEA sentence field
 */
static int nmea_field_len (int field_index) {
	return nmea_fields[field_index+1] - nmea_fields[field_index] - 1;
}

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

			nmea_field_count = 0;

			// $GPGGA NMEA sentence
			if (nmea_buf[3]=='G' && nmea_buf[4] == 'G' && nmea_buf[5] == 'A') {
				extern uint32_t systick_counter;
				gps_last_position_t = systick_counter;

				// Make copy from NMEA buffer to avoid data being clobbered in the background by incoming chars
				copy_nmea_field (gps_time_of_day, nmea_buf + nmea_fields[0]);

				// Prefix with negative sign if south of equator
				if (nmea_buf[nmea_fields[2]]=='S') {
					gps_latitude[0] = '-';
					copy_nmea_field (gps_latitude+1, nmea_buf + nmea_fields[1]);
				} else {
					copy_nmea_field (gps_latitude, nmea_buf + nmea_fields[1]);
				}

				// Prefix with negative sign of west of Greenwich meridian
				if (nmea_buf[nmea_fields[4]]=='W') {
					gps_longitude[0] = '-';
					copy_nmea_field (gps_longitude+1, nmea_buf + nmea_fields[3]);
				} else {
					copy_nmea_field (gps_longitude, nmea_buf + nmea_fields[3]);
				}

				copy_nmea_field (gps_fix, nmea_buf + nmea_fields[5]);
				copy_nmea_field (gps_hdop, nmea_buf + nmea_fields[7]);
			}

			// $GPRMC NMEA sentence
			if (nmea_buf[3]=='R' && nmea_buf[4] == 'M' && nmea_buf[5] == 'C') {

				// The "$" of $GPRMC sentence (the first in sequence) is measured by
				// oscilloscope to be about 55ms +/- 5ms after PPS signal. This allows
				// the systick timer to be tied to GPS UTC within +/- 5ms.
				gps_top_of_second_t = gps_dollar_t - 5;

				// If no heading substitute with "999"
				if ( nmea_field_len(6) == 0) {
					strcpy(gps_heading,"999");
				} else {
					copy_nmea_field (gps_heading, nmea_buf + nmea_fields[6]);
				}

				copy_nmea_field (gps_speed, nmea_buf + nmea_fields[7]);
			}

		} else if (c>31){

			if (c == '$') {
				gps_dollar_t = systick_counter;
			}

			if (nmea_buf_index == GPS_NMEA_SIZE) {
				nmea_buf[GPS_NMEA_SIZE-1]=0;
				nmea_buf_index = 0;
			}

			if (c == ',') {
				nmea_fields[nmea_field_count++] = nmea_buf_index+1;
			}



			nmea_buf[nmea_buf_index++] = c;
		}

		// If bit 0 if gps_echo param set then echo to UART0
		if (params_union.params.gps_echo&0x1) {
			LPC_USART0->TXDATA = c;
		}

	} else if (uart_status & UART_STAT_TXRDY ){
		LPC_USART1->INTENCLR = 0x04;
	}
}
#endif
