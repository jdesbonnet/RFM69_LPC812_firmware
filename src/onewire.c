/**
 * A simple one wire big-bang library.
 *
 */


#include "config.h"

#include "onewire.h"
#include "crc8.h"

#include "delay.h"


static uint32_t ow_port;
static uint32_t ow_pin;

void ow_init(int port, int pin) {
	ow_port=port;
	ow_pin=pin;
}

void ow_low() {
	// set direction output
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, ow_port, ow_pin);
	// set low
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, ow_port, ow_pin);
}

void ow_high() {
	// set direction input (high Z) and let pull-up R bring high
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, ow_port, ow_pin);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ow_port, ow_pin);
}

int ow_read() {
	//GPIOSetDir(ow_port, ow_pin, 0);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ow_port, ow_pin);

	//return GPIOGetPinValue(ow_port, ow_pin);
	return Chip_GPIO_GetPinState(LPC_GPIO_PORT, ow_port, ow_pin);
}

/**
 * Issue a read slot and return the result. The result must be read within
 * 15µs of the
 *
 * @return 0 or 1
 */
int ow_bit_read () {

	// The read slot starts with the bus diven low.
	// We have 15µs from the falling edge read the bus.
	ow_low();
	//delay_microseconds(1); // Must be held low for at least 1µs
	delay_nop_loop(2);

	// Bring bus high again. And read within the 15µs time interval
	// (already a few µs used by by now...)
	ow_high();
	delay_microseconds(1);

	int b = ow_read();

	// Read slots must be a minimum of 60µs in duration with a minimum of 1µs
	// recovery time between slots. Rather than monitor bus to check for end
	// of slot, just delay for a period well exceeding the 60µs slot time.
	delay_microseconds(65);

	return b;
}

/**
 * Reset one wire bus.
 * @return -1 if successful.
 */
int ow_reset() {
	ow_low();
	delay_microseconds(480);
	ow_high();
	delay_microseconds(70);

	int detect = ow_read();
	ow_high();

	delay_microseconds(410);

	debug ("ow detect %d",detect);

	return ~detect;
}

void ow_bit_write (int b) {

	// Write slot duration min 60µs
	ow_low();
	if (b) {
		// having trouble getting this in the 1-15µs range. Need better delay mechanism.
		delay_nop_loop(2); // max 15µs, min 1µs (?)
		ow_high();
		delay_microseconds(60);
	} else {
		delay_microseconds(61);
		ow_high();
	}

	// Recovery time
	delay_microseconds(2);
}
void ow_byte_write (int data) {
	int i;

	// Send LSB first.

	for (i = 0; i < 8; i++) {
		ow_bit_write(data & 0x01);
		data >>= 1;
	}

}

int ow_byte_read () {
	int i, data = 0;
	for (i = 0; i < 8; i++) {
		data >>= 1;
		data |= ow_bit_read() ? 0x80 : 0x00;
	}
	return data;
}


uint64_t ow_uint64_read () {
	uint64_t data = 0;

	int i;
	for (i = 0; i < 8; i++) {
		data <<= 8;
		data |= ow_byte_read();
	}

	return data;
}

