/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <errno.h>

#include <spinner/filter/low_pass_filter.h>

#define LOW_PASS_FILTER_PI 3.14159265358979323846f

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS_BIQUAD)
int low_pass_filter_init(struct low_pass_filter *filter,
			 uint32_t sample_rate_hz,
			 const struct low_pass_filter_config *config)
{
	const float sample_rate = (float)sample_rate_hz;
	const float cutoff = config != NULL ? config->cutoff_hz : 0.0f;
	float omega;
	float alpha;
	float cos_omega;
	float a0;

	if (filter == NULL || config == NULL || sample_rate_hz == 0U ||
	    cutoff <= 0.0f || cutoff * 2.0f >= sample_rate) {
		return -EINVAL;
	}

	omega = 2.0f * LOW_PASS_FILTER_PI * cutoff / sample_rate;
	alpha = sinf(omega) / (2.0f * 0.7071067812f);
	cos_omega = cosf(omega);
	a0 = 1.0f + alpha;
	memset(filter, 0, sizeof(*filter));

	filter->coefficients[0] = ((1.0f - cos_omega) * 0.5f) / a0;
	filter->coefficients[1] = (1.0f - cos_omega) / a0;
	filter->coefficients[2] = filter->coefficients[0];
	/* CMSIS-DSP adds feedback terms; RBJ denominator terms use subtraction. */
	filter->coefficients[3] = (2.0f * cos_omega) / a0;
	filter->coefficients[4] = -(1.0f - alpha) / a0;
	arm_biquad_cascade_df1_init_f32(&filter->biquad, 1, filter->coefficients,
					filter->state);
	return 0;
}

float low_pass_filter_process(struct low_pass_filter *filter, float sample)
{
	float output;

	arm_biquad_cascade_df1_f32(&filter->biquad, &sample, &output, 1);
	return output;
}
#elif defined(CONFIG_SPINNER_FILTER_LOW_PASS_FIR)
int low_pass_filter_init(struct low_pass_filter *filter,
			 uint32_t sample_rate_hz,
			 const struct low_pass_filter_config *config)
{
	const float sample_rate = (float)sample_rate_hz;
	const float cutoff = config != NULL ? config->cutoff_hz : 0.0f;
	const size_t taps = config != NULL ? config->taps : 0U;
	float sum = 0.0f;

	if (filter == NULL || config == NULL || sample_rate_hz == 0U ||
	    cutoff <= 0.0f || cutoff * 2.0f >= sample_rate || taps < 3U ||
	    taps > CONFIG_SPINNER_FILTER_LOW_PASS_MAX_TAPS || (taps & 1U) == 0U) {
		return -EINVAL;
	}

	memset(filter, 0, sizeof(*filter));
	for (size_t i = 0; i < taps; i++) {
		const float centered = (float)i - (float)(taps - 1U) * 0.5f;
		const float sinc = centered == 0.0f
			? 2.0f * cutoff / sample_rate
			: sinf(2.0f * LOW_PASS_FILTER_PI * cutoff * centered / sample_rate) /
			  (LOW_PASS_FILTER_PI * centered);
		const float window = 0.54f - 0.46f * cosf(2.0f * LOW_PASS_FILTER_PI * i /
									(taps - 1U));

		filter->coefficients[i] = sinc * window;
		sum += filter->coefficients[i];
	}

	for (size_t i = 0; i < taps; i++) {
		filter->coefficients[i] /= sum;
	}

	arm_fir_init_f32(&filter->fir, (uint16_t)taps, filter->coefficients,
				 filter->state, 1);
	return 0;
}

float low_pass_filter_process(struct low_pass_filter *filter, float sample)
{
	float output;

	arm_fir_f32(&filter->fir, &sample, &output, 1);
	return output;
}
#else
int low_pass_filter_init(struct low_pass_filter *filter,
			 uint32_t sample_rate_hz,
			 const struct low_pass_filter_config *config)
{
	(void)filter;
	(void)sample_rate_hz;
	(void)config;
	return -ENOTSUP;
}

float low_pass_filter_process(struct low_pass_filter *filter, float sample)
{
	(void)filter;
	return sample;
}
#endif