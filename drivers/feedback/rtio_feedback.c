/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "rtio_feedback.h"

LOG_MODULE_REGISTER(rtio_feedback, CONFIG_SPINNER_FEEDBACK_LOG_LEVEL);

#define RTIO_FEEDBACK_FULL_TURN_DEGREES 360.0f

static float rtio_feedback_wrap_delta(float delta)
{
	if (delta > 180.0f) {
		return delta - RTIO_FEEDBACK_FULL_TURN_DEGREES;
	}
	if (delta < -180.0f) {
		return delta + RTIO_FEEDBACK_FULL_TURN_DEGREES;
	}

	return delta;
}

static float rtio_feedback_wrap_angle(float angle)
{
	angle = fmodf(angle, RTIO_FEEDBACK_FULL_TURN_DEGREES);
	if (angle < 0.0f) {
		angle += RTIO_FEEDBACK_FULL_TURN_DEGREES;
	}

	return angle;
}

static void rtio_feedback_process(const struct device *dev, int result, uint8_t *buffer)
{
	struct rtio_feedback_data *data = dev->data;
	struct sensor_q31_data decoded_data;
	struct sensor_decode_context context;
	uint64_t timestamp_ns;
	float angle_degrees;
	float delta_degrees;
	float speed_rpm = 0.0f;
	int decoded;

	if (result != 0 || buffer == NULL) {
		LOG_DBG("RTIO encoder read failed: %d", result);
		return;
	}

	context = (struct sensor_decode_context)SENSOR_DECODE_CONTEXT_INIT(
		data->decoder, buffer, SENSOR_CHAN_ROTATION, 0);
	decoded = sensor_decode(&context, &decoded_data, 1);
	if (decoded <= 0 || decoded_data.header.reading_count == 0U) {
		LOG_DBG("RTIO encoder decode failed: %d", decoded);
		return;
	}

	timestamp_ns = decoded_data.header.base_timestamp_ns;
	angle_degrees = ((float)decoded_data.readings[0].angle * (float)(1U << decoded_data.shift)) /
				(float)(1ULL << 31);

	k_spinlock_key_t key = k_spin_lock(&data->snapshot_lock);
	if (data->have_sample) {
		delta_degrees = rtio_feedback_wrap_delta(angle_degrees - data->previous_angle_degrees);
		if (timestamp_ns > data->previous_timestamp_ns) {
			float elapsed_seconds = (float)(timestamp_ns - data->previous_timestamp_ns) / 1.0e9f;
			speed_rpm = delta_degrees / elapsed_seconds * (60.0f / 360.0f);
		}
		data->position_degrees += delta_degrees;
	} else {
		data->position_degrees = angle_degrees;
		data->have_sample = true;
	}

	data->previous_angle_degrees = angle_degrees;
	data->previous_timestamp_ns = timestamp_ns;
	data->angle_degrees = rtio_feedback_wrap_angle(angle_degrees);

	k_spin_unlock(&data->snapshot_lock, key);

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
	speed_rpm = low_pass_filter_process(&data->speed_filter, speed_rpm);
#endif

	key = k_spin_lock(&data->snapshot_lock);
	data->speed_rpm = speed_rpm;
	k_spin_unlock(&data->snapshot_lock, key);
}

static void rtio_feedback_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	const struct rtio_feedback_config *config = dev->config;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (sensor_stream(config->iodev, config->rtio, NULL, NULL) < 0) {
		LOG_ERR("Failed to start sensor RTIO stream");
		return;
	}

	while (true) {
		struct rtio_cqe *cqe = rtio_cqe_consume_block(config->rtio);
		uint8_t *buffer = NULL;
		uint32_t buffer_len = 0;

		if (cqe->result == 0 &&
			rtio_cqe_get_mempool_buffer(config->rtio, cqe, &buffer, &buffer_len) == 0) {
			rtio_feedback_process(dev, cqe->result, buffer);
		}

		if (buffer != NULL) {
			rtio_release_buffer(config->rtio, buffer, buffer_len);
		}
		rtio_cqe_release(config->rtio, cqe);
	}
}

static float rtio_feedback_get_angle(const struct device *dev)
{
	struct rtio_feedback_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->snapshot_lock);
	float value = data->angle_degrees;
	k_spin_unlock(&data->snapshot_lock, key);

	return value;
}

static float rtio_feedback_get_speed(const struct device *dev)
{
	struct rtio_feedback_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->snapshot_lock);
	float value = data->speed_rpm;
	k_spin_unlock(&data->snapshot_lock, key);

	return value;
}

static float rtio_feedback_get_position(const struct device *dev)
{
	struct rtio_feedback_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->snapshot_lock);
	float value = data->position_degrees;
	k_spin_unlock(&data->snapshot_lock, key);

	return value;
}

const struct feedback_driver_api rtio_feedback_api = {
	.get_angle = rtio_feedback_get_angle,
	.get_speed = rtio_feedback_get_speed,
	.get_position = rtio_feedback_get_position,
};

int rtio_feedback_init(const struct device *dev)
{
	const struct rtio_feedback_config *config = dev->config;
	struct rtio_feedback_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->sensor)) {
		return -EINVAL;
	}

	ret = sensor_get_decoder(config->sensor, &data->decoder);
	if (ret < 0) {
		LOG_ERR("Encoder decoder unavailable: %d", ret);
		return ret;
	}

#if defined(CONFIG_SPINNER_FILTER_LOW_PASS)
	ret = low_pass_filter_init(&data->speed_filter, CONFIG_SPINNER_FEEDBACK_RTIO_SAMPLE_PERIOD_MS,
					   &config->speed_filter_config);
	if (ret < 0) {
		return ret;
	}
#endif

	k_thread_create(&data->thread, config->stack, config->stack_size,
				rtio_feedback_thread, (void *)dev, NULL, NULL,
				K_PRIO_PREEMPT(8), 0, K_NO_WAIT);

	return 0;
}