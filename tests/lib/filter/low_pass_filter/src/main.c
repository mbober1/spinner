/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include <spinner/filter/low_pass_filter.h>

static const struct low_pass_filter_config filter_config = {
	.cutoff_hz = 10.0f,
	.taps = 15U,
};

ZTEST(low_pass_filter, test_invalid_configuration)
{
	struct low_pass_filter filter;
	struct low_pass_filter_config invalid = filter_config;

	zassert_equal(low_pass_filter_init(NULL, 10U, &filter_config), -EINVAL, NULL);
	zassert_equal(low_pass_filter_init(&filter, 0U, &filter_config), -EINVAL, NULL);

	invalid.cutoff_hz = 50.0f;
	zassert_equal(low_pass_filter_init(&filter, 10U, &invalid), -EINVAL, NULL);

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS_FIR)
	invalid = filter_config;
	invalid.taps = 2U;
	zassert_equal(low_pass_filter_init(&filter, 10U, &invalid), -EINVAL, NULL);
	invalid.taps = 16U;
	zassert_equal(low_pass_filter_init(&filter, 10U, &invalid), -EINVAL, NULL);
	invalid.taps = CONFIG_SPINNER_FILTER_LOW_PASS_MAX_TAPS + 1U;
	zassert_equal(low_pass_filter_init(&filter, 10U, &invalid), -EINVAL, NULL);
#endif
}

ZTEST(low_pass_filter, test_constant_input_converges)
{
	struct low_pass_filter filter;
	float output = 0.0f;

	zassert_equal(low_pass_filter_init(&filter, 10U, &filter_config), 0, NULL);
	for (size_t i = 0; i < 200U; i++) {
		output = low_pass_filter_process(&filter, 1.0f);
	}

	zassert_true(isfinite(output), NULL);
	zassert_true(fabsf(output - 1.0f) < 0.01f, NULL);
}

ZTEST(low_pass_filter, test_reinitialization_resets_state)
{
	struct low_pass_filter filter;
	float output;

	zassert_equal(low_pass_filter_init(&filter, 10U, &filter_config), 0, NULL);
	(void)low_pass_filter_process(&filter, 1.0f);
	zassert_equal(low_pass_filter_init(&filter, 10U, &filter_config), 0, NULL);
	output = low_pass_filter_process(&filter, 0.0f);

	zassert_true(fabsf(output) < 0.001f, NULL);
}

ZTEST_SUITE(low_pass_filter, NULL, NULL, NULL, NULL, NULL);