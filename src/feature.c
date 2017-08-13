#include "config.h"
#include "params.h"

extern params_union_type params_union;

/**
 * Return 1 if feature is enabled.
 */
int is_feature_enabled (feature_t feature) {
	return params_union.params.feature_enable & (1<<feature) ? 1 : 0;
}

/**
 * Enable/disable feature.
 */
void feature_enable (feature_t feature, int state) {
	if (state) {
		params_union.params.feature_enable  |= (1<<feature);
	} else {
		params_union.params.feature_enable  &= ~(1<<feature);
	}
}
