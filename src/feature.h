#ifndef FEATURE_H_
#define FEATURE_H_

/**
 * Bit of feature_enable parameter
 */
typedef enum {
	F_DS18B20 = 0,
	F_ABPM = 1,
	F_RELAY = 2
} feature_t;

int is_feature_enabled (feature_t feature);
void set_feature_enable (feature_t feature, int state);
#endif
