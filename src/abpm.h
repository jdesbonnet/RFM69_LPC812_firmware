#ifndef ABPM_H_
#define ABPM_H_

void abpm_init();
int abpm_measure();
void abpm_stop();
int abpm_bus_snoop();


typedef struct {
	uint8_t month;
	uint8_t day_of_month;
	uint8_t hour;
	uint8_t minute;

	uint8_t systolic_pressure;
	uint8_t diastolic_pressure;
	uint8_t heart_rate;
	uint8_t reserved;

} bp_record_t;


#endif
