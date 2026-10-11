/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SPINNER_FILTER_LOW_PASS_FILTER_H_
#define SPINNER_FILTER_LOW_PASS_FILTER_H_

#include <stdint.h>

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
#include <arm_math.h>
#endif

struct low_pass_filter_config {
	float cutoff_hz;
	uint16_t taps;
};

struct low_pass_filter {
#if defined(CONFIG_SPINNER_FILTER_LOW_PASS_BIQUAD)
	arm_biquad_casd_df1_inst_f32 biquad;
	float coefficients[5];
	float state[4];
#elif defined(CONFIG_SPINNER_FILTER_LOW_PASS_FIR)
	arm_fir_instance_f32 fir;
	float coefficients[CONFIG_SPINNER_FILTER_LOW_PASS_MAX_TAPS];
	float state[CONFIG_SPINNER_FILTER_LOW_PASS_MAX_TAPS];
#else
	uint8_t unused;
#endif
};

int low_pass_filter_init(struct low_pass_filter *filter,
			 uint32_t sampling_period_ms,
			 const struct low_pass_filter_config *config);
float low_pass_filter_process(struct low_pass_filter *filter, float sample);

#endif