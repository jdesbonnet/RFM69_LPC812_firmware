#ifndef FEATURE_H_
#define FEATURE_H_
typedef enum {
	F_DS18B20,
	F_ABPM
} feature_t;

int is_feature_enabled (feature_t feature);
void set_feature_enable (feature_t feature, int state);
#endif
