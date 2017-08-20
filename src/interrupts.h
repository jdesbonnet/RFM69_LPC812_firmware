#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_
typedef enum {
	NO_INTERRUPT = 0,
	WKT_INTERRUPT = 1,
	UART_INTERRUPT = 2,
	EVENT_COUNTER_INTERRUPT = 3,
	PIEZO_SENSOR_INTERRUPT = 4,
	DIO0_INTERRUPT = 5,
	START_BTN_INTERRUPT = 6
} wake_interrupt_source_t;
#endif
