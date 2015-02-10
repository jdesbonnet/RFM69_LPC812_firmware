#ifndef PARAMS_H_
#define PARAMS_H_

/**
 * The paramater block. Controls the behaviour of the system.
 */
typedef struct {
	uint8_t listen_period;
	uint8_t poll_interval;

	// To facilitate experimenting with the optimum low power state of pins
	// allow the GPIO pin directions and states to be configured here.
	uint16_t sleep_pin_dir;
	uint16_t sleep_pin_state;
} params_struct;

// Allow the parameter block to be accessed either as a structure or a sequence of bytes
typedef union {
	params_struct params;
	//uint8_t params_buffer[sizeof(params_struct)];
	uint8_t params_buffer[64];
	//uint64_t padx[8] = {0,0,0,0, 1,2,3,4};
} params_type;

#endif
