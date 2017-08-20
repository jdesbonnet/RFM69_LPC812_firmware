/*
===============================================================================
 Name        : abpm.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "config.h"
#include "abpm.h"

extern volatile uint32_t systick_counter;
extern volatile wake_interrupt_source_t wake_interrupt_source;

// Variables global within scope of this file
static volatile int sda_pin_state=1;
static volatile int scl_pin_state=1;
static volatile int start_btn_interrupt;
static char i2c_buf[1024];
static volatile int i2c_buf_ptr=0;
static uint32_t last_clock = 0;

static void abpm_setup_pin_for_interrupt (int interrupt_pin, int interrupt_channel, int riseOrFallEdge) {

	/* Configure interrupt channel for the GPIO pin in SysCon block */
	Chip_SYSCTL_SetPinInterrupt(interrupt_channel, interrupt_pin);

	/* Configure GPIO pin as input pin */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, interrupt_pin);

	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(interrupt_channel));

	if (riseOrFallEdge == 0) {
		Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(interrupt_channel));
	} else {
		Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH(interrupt_channel));
	}
}


/**
 * Assert START button by pulling low and going into input (high Z)
 * state otherwise. Doing this due the 4.4V logic rail: not sure
 * if exposing a output high (normally at Vdd or 2.2V) to 4.4V is
 * good idea.
 * @state  1 = press button, 0 = normal button released
 */
void abpm_press_start_button (int state) {
	if (state == 1) {
		// Pin to 0V and switch to output
		Chip_GPIO_SetPinState(LPC_GPIO_PORT,0,PIN_BPM_START,0);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,0,PIN_BPM_START);
	} else {
		// Pin to input (high Z)
		Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT,0,PIN_BPM_START);
	}
}


/**
 * @brief	START/STOP button falling edge
 * @return	Nothing
 */
void PININT2_IRQHandler(void)
{
	//tfp_printf ("; START/STOP\r\n");
	wake_interrupt_source = START_BTN_INTERRUPT;
	start_btn_interrupt = 1;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
}

/**
 * @brief	SCL rising edge
 * @return	Nothing
 */
void PININT4_IRQHandler(void)
{
	scl_pin_state = 1;
	i2c_buf[i2c_buf_ptr++] = sda_pin_state ? '1':'0';
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
}

/**
 * @brief	SCL falling edge
 * @return	Nothing
 */
void PININT5_IRQHandler(void)
{
	last_clock = systick_counter;
	scl_pin_state = 0;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
}

/**
 * @brief	SDA rising edge
 * @return	Nothing
 */
void PININT6_IRQHandler(void)
{
	sda_pin_state = 1;

	// Stop condition
	if (scl_pin_state == 1) {
		i2c_buf[i2c_buf_ptr++] = 'P';
	}
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
}

/**
 * @brief	SDA falling edge
 * @return	Nothing
 */
void PININT7_IRQHandler(void)
{
	sda_pin_state = 0;

	// Start condition
	if (scl_pin_state  == 1) {
		i2c_buf[i2c_buf_ptr++] = 'S';
	}
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH7);
}

void abpm_init ()  {

	// START button in input/high-z normally. Note that this
	// pin is at 4.4V: using 5V tolerance feature. Button is
	// 'pressed' by pulling low.
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, PIN_BPM_START);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, PIN_BPM_SCL);
	Chip_IOCON_PinSetMode(LPC_IOCON,PIN_BPM_SCL,PIN_MODE_PULLUP);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, PIN_BPM_SDA);
	Chip_IOCON_PinSetMode(LPC_IOCON,PIN_BPM_SDA,PIN_MODE_PULLUP);

	// START button
	abpm_setup_pin_for_interrupt(PIN_BPM_START, 2, 0);

	// Configure I2C lines to trigger interrupts on both
	// rising and falling edges.
	abpm_setup_pin_for_interrupt(PIN_BPM_SCL, 4, 1);
	abpm_setup_pin_for_interrupt(PIN_BPM_SCL, 5, 0);
	abpm_setup_pin_for_interrupt(PIN_BPM_SDA, 6, 1);
	abpm_setup_pin_for_interrupt(PIN_BPM_SDA, 7, 0);




	// Enable interrupts in the NVIC
	NVIC_EnableIRQ(PININT2_IRQn);

	NVIC_EnableIRQ(PININT4_IRQn);
	NVIC_EnableIRQ(PININT5_IRQn);
	NVIC_EnableIRQ(PININT6_IRQn);
	NVIC_EnableIRQ(PININT7_IRQn);


    scl_pin_state = Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_BPM_SCL);
    sda_pin_state = Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_BPM_SDA);
}

/**
 * Stop a measurement in progress.
 */
void abpm_stop () {
	abpm_press_start_button(1);
	delayMilliseconds(500);
	abpm_press_start_button(0);
	delayMilliseconds(500);
}


int abpm_bus_snoop (bp_record_t *bp) {
	tfp_printf("; BPM I2C bus snoop...\r\n");

	bp->systolic_pressure = 0xff;
	bp->diastolic_pressure = 0xff;
	bp->heart_rate = 0xff;

	uint32_t start_time = systick_counter;

	int i = 0;

	// Bit in current byte counter, when 8 we have a byte, 9th bit is ack.
	int bit_count = 0;

	// Number of bytes in transaction. Reset on STOP (not START).
	int byte_count = 0;

	// Bit shift each incoming bit here
	uint8_t current_byte = 0;

	// Frame data (including addr). Reset on STOP (not START).
	uint8_t i2c_bytes[8];

	// Bytes written to EEPROM for BP reading
	uint8_t bp_record[12];
	int bp_record_ptr = 0;

	start_btn_interrupt = 0;

	while (systick_counter - start_time < 60*TICKRATE_HZ) {

		if (start_btn_interrupt == 1) {
			tfp_printf("; BP stopped\r\n");
			return -2;
		}


		//tfp_printf("tick=%d last=%d\r\n", systick_counter, last_clock);

		// TODO: this is a rather messy way of extracting data from snooped
		// I2C transaction.

		// One of two formats:
		// read operation: S A0 [ack] addr [ack] S A1 [nak] data-from-eeprom P
		// write operation S A0 [ack] addr [ack] [data-to-write] P
		if ((i2c_buf_ptr > 0) && ((systick_counter - last_clock) > 50)) {
			for (i = 0; i < i2c_buf_ptr; i++) {
				char c = i2c_buf[i];
				tfp_printf("%c", c);

				// START condition (S)
				if (c == 'S') {
					bit_count = 0;
				}

				if (c == '0' || c == '1') {
					current_byte <<= 1;
					current_byte |= (c == '1');
					bit_count++;
				}

				// Received full byte (add or data)
				if (bit_count == 8) {
					i2c_bytes[byte_count] = current_byte;
					tfp_printf(" [%x] ", current_byte);
				}

				// Ack bit
				if (bit_count == 9) {
					bit_count = 0;
					byte_count++;
					tfp_printf(" ");
				}

				// STOP condition (P)
				if (c == 'P') {

					tfp_printf("\r\n");

					// Write operations have 3 bytes of addr+data between S and P
					if (byte_count == 3) {
						bp_record[bp_record_ptr++] = i2c_bytes[2];
					}
					// Do we have full BP record?
					if (bp_record_ptr == 10) {
						int systolic = (bp_record[5] >> 4) * 100;
						systolic += (bp_record[6] >> 4) * 10;
						systolic += (bp_record[6] & 0xf);

						int diastolic = (bp_record[5] & 0xf) * 100;
						diastolic += (bp_record[7] >> 4) * 10;
						diastolic += (bp_record[7] & 0xf);

						int heart_rate = bp_record[8];

						tfp_printf("; BP %d %d %d\r\n", systolic, diastolic, heart_rate);
						bp->systolic_pressure = systolic;
						bp->diastolic_pressure = diastolic;
						bp->heart_rate = heart_rate;
						return 0;

					}

					byte_count = 0;
				}
			}
			i2c_buf_ptr = 0;
			bp_record_ptr = 0;

		} else {
			//tfp_printf("i2cptr=%d systick=%d last_clock=%d\r\n", i2c_buf, systick_counter, last_clock);
		}
	}
	return -1;
}

/**
 * Start a measurement
 *
 */

int abpm_measure (bp_record_t *bp) {
	// Press for 1 sec to wake, release for 1 sec
	// and press again to start measurement
	abpm_press_start_button(1);
	delayMilliseconds(500);
	abpm_press_start_button(0);
	delayMilliseconds(500);
	abpm_press_start_button(1);
	delayMilliseconds(500);
	abpm_press_start_button(0);
	delayMilliseconds(500);

	return abpm_bus_snoop(bp);
}

