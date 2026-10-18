/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SPINNER_RTIO_FEEDBACK_H_
#define SPINNER_RTIO_FEEDBACK_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

#include <spinner/drivers/feedback.h>

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
#include <spinner/filter/low_pass_filter.h>
#endif

struct rtio_feedback_config {
	const struct device *sensor;
	const struct rtio_iodev *iodev;
	struct rtio *rtio;
	k_thread_stack_t *stack;
	size_t stack_size;
#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
	struct low_pass_filter_config speed_filter_config;
#endif
};

struct rtio_feedback_data {
	struct k_thread thread;
	struct k_spinlock snapshot_lock;
	const struct sensor_decoder_api *decoder;
	bool have_sample;
	uint64_t previous_timestamp_ns;
	float previous_angle_degrees;
	float position_degrees;
	float angle_degrees;
	float speed_rpm;
#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
	struct low_pass_filter speed_filter;
#endif
};

extern const struct feedback_driver_api rtio_feedback_api;

int rtio_feedback_init(const struct device *dev);

#define RTIO_FEEDBACK_DEFINE(inst)                                             \
	SENSOR_DT_STREAM_IODEV(rtio_feedback_iodev_##inst,                          \
			      DT_INST_PHANDLE(inst, sensor),                  \
			      SENSOR_STREAM_TRIGGER_PREP(SENSOR_TRIG_TIMER,                  \
							 SENSOR_STREAM_DATA_INCLUDE));                 \
	RTIO_DEFINE_WITH_MEMPOOL(rtio_feedback_rtio_##inst, 4, 4, 4, 32, 4);       \
	K_THREAD_STACK_DEFINE(rtio_feedback_stack_##inst, 1024);                    \
	static const struct rtio_feedback_config rtio_feedback_config_##inst = {    \
		.sensor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sensor)),              \
		.iodev = &rtio_feedback_iodev_##inst,                                \
		.rtio = &rtio_feedback_rtio_##inst,                                  \
		.stack = rtio_feedback_stack_##inst,                                  \
		.stack_size = K_THREAD_STACK_SIZEOF(rtio_feedback_stack_##inst),     \
	IF_ENABLED(CONFIG_SPINNER_FILTER_LOW_PASS_BIQUAD,                        \
		(.speed_filter_config.cutoff_hz = DT_INST_PROP(inst, filter_cutoff_hz),))      \
	};                                                   											\
	static struct rtio_feedback_data rtio_feedback_data_##inst;              \
	DEVICE_DT_INST_DEFINE(inst, rtio_feedback_init, NULL,                     \
			      &rtio_feedback_data_##inst, &rtio_feedback_config_##inst, \
			      POST_KERNEL, 91, &rtio_feedback_api)

#endif