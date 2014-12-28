#ifndef PARAMS_H_
#define PARAMS_H_

typedef struct {
	uint8_t listen_period;
	uint8_t poll_interval;
	uint16_t sleep_pin_dir;
	uint16_t sleep_pin_state;
} params_struct;

typedef union {
	params_struct params;
	uint8_t params_buffer[sizeof(params_struct)];
} params_type;

#endif
