#ifdef __USE_CMSIS
#include "LPC8xx.h"
#endif

#include <cr_section_macros.h>

#include <string.h>
#include <stdint.h>

#include "config.h"
#include "parse_util.h"
#include "rfm69.h"
#include "cmd.h"
#include "err.h"


#ifdef FEATURE_GPS_ON_USART1


extern volatile uint32_t gps_last_position_t, gps_top_of_second_t;
extern volatile uint8_t gps_time_of_day[], gps_latitude[], gps_longitude[], gps_fix[], gps_hdop[];
extern volatile uint8_t gps_heading[], gps_speed[];
extern uint32_t systick_counter;

/**
 * Command to query/set GPS location, heading etc
 * Args: <to-node-addr>
 */
int cmd_gps (int argc, uint8_t **args) {

	if (argc == 1) {
		displayGPS();
		return;
	}
	if (argc != 6) {
		report_error('G',E_INVALID_ARG);
		return;
	}
	strcpy(gps_time_of_day, args[1]);
	strcpy(gps_latitude,args[2]);
	strcpy(gps_longitude,args[3]);
	strcpy(gps_heading,args[4]);
	strcpy(gps_speed,args[5]);

	gps_last_position_t = systick_counter;

	//sendGPSUpdate(0xff);
	displayGPS();
}
#endif
