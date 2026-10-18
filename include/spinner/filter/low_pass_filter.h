/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SPINNER_FILTER_LOW_PASS_FILTER_H_
#define SPINNER_FILTER_LOW_PASS_FILTER_H_

#include <stdint.h>
#include <arm_math.h>

struct low_pass_filter_config {
	float cutoff_hz;
};

struct low_pass_filter {
	arm_biquad_casd_df1_inst_f32 biquad;
	float coefficients[5];
	float state[4];
};

int low_pass_filter_init(struct low_pass_filter *filter,
			 uint32_t sample_rate_hz,
			 const struct low_pass_filter_config *config);
float low_pass_filter_process(struct low_pass_filter *filter, float sample);

#endif /* SPINNER_FILTER_LOW_PASS_FILTER_H_ */
