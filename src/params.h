#ifndef PARAMS_H_
#define PARAMS_H_

/**
 * The paramater block. Controls the behaviour of the system.
 *
 * Design note: must be no more than 64 bytes in length to fit in single page
 * of flash memory.
 *
 * Default values currently in eeprom.c (not ideal).
 */
typedef struct __attribute__((packed)) {
	uint8_t node_addr;
	uint8_t operating_mode;
	uint8_t listen_period_cs;
	uint8_t poll_interval;      // index 3: poll interval in seconds

	// index 4
	uint8_t gps_echo;
	uint8_t ds18b20_mode;
	uint16_t link_loss_timeout_s;  // index 6,7

	// index 8
	uint8_t min_battery_v;	// index 8: min voltage required for radio operation in 0.1V units
	uint8_t low_battery_v;	// index 9: min voltage for regular operation 0.1V units.

	uint8_t tx_power; // index 0xa: RFM9x 0 - 15
	uint8_t lora_bw;  // index 0xb: RFM9x LoRa BW 0 - 9
	uint8_t lora_cr;  // index 0xc: RFM9x LoRa CodingRate 0 - 5
	uint8_t lora_sf;  // index 0xd: RFM9x LoRa SpreadingFactor  6 - 12

	// To facilitate experimenting with the optimum low power state of pins
	// allow the GPIO pin directions and states to be configured here.
	uint16_t sleep_pin_dir;
	uint16_t sleep_pin_state;
} params_struct;

// Allow the parameter block to be accessed either as a structure or a sequence of bytes
// TODO: is there a better approach? Refering to params as param_union.param.blah is very
// verbose.
typedef union {
	params_struct params;
	//uint8_t params_buffer[sizeof(params_struct)];
	uint8_t params_buffer[64];
	//uint64_t padx[8] = {0,0,0,0, 1,2,3,4};
} params_union_type;

#endif
