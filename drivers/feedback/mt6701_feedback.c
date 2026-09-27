/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT spinner_mt6701_feedback

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <spinner/drivers/feedback.h>

LOG_MODULE_REGISTER(mt6701_feedback, CONFIG_SPINNER_FEEDBACK_LOG_LEVEL);

#define MT6701_FEEDBACK_MILLI_SCALE 1000.0f

struct mt6701_feedback_config {
	const struct device *sensor;
	uint32_t sampling_period_ms;
	uint32_t pole_pairs;
	float phase_offset_degrees;
};

struct mt6701_feedback_data {
	const struct device *dev;
	struct k_work_delayable sample_work;
	atomic_t eangle_millidegrees;
	atomic_t speed_millirpm;
};

static int mt6701_feedback_sample(const struct device *dev)
{
	const struct mt6701_feedback_config *config = dev->config;
	struct mt6701_feedback_data *data = dev->data;
	struct sensor_value rotation;
	struct sensor_value speed;
	float electrical_angle;
	int ret;

	ret = sensor_sample_fetch(config->sensor);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(config->sensor, SENSOR_CHAN_ROTATION, &rotation);
	if (ret < 0) {
		return ret;
	}

	electrical_angle = fmodf(sensor_value_to_float(&rotation) * config->pole_pairs +
				 config->phase_offset_degrees,
				 360.0f);
	if (electrical_angle < 0.0f) {
		electrical_angle += 360.0f;
	}

	atomic_set(&data->eangle_millidegrees,
		   (atomic_val_t)(electrical_angle * MT6701_FEEDBACK_MILLI_SCALE));

	ret = sensor_channel_get(config->sensor, SENSOR_CHAN_RPM, &speed);
	if (ret == 0) {
		atomic_set(&data->speed_millirpm,
			   (atomic_val_t)(sensor_value_to_float(&speed) *
					  MT6701_FEEDBACK_MILLI_SCALE));
	}

	return 0;
}

static void mt6701_feedback_sample_work(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct mt6701_feedback_data *data =
		CONTAINER_OF(delayable, struct mt6701_feedback_data, sample_work);
	const struct mt6701_feedback_config *config = data->dev->config;
	int ret;

	ret = mt6701_feedback_sample(data->dev);
	if (ret < 0) {
		LOG_DBG("MT6701 sample failed (%d); retaining last angle", ret);
	}

	k_work_schedule(&data->sample_work, K_MSEC(config->sampling_period_ms));
}

static float mt6701_feedback_get_eangle(const struct device *dev)
{
	const struct mt6701_feedback_data *data = dev->data;

	return (float)atomic_get(&data->eangle_millidegrees) /
	       MT6701_FEEDBACK_MILLI_SCALE;
}

static float mt6701_feedback_get_speed(const struct device *dev)
{
	const struct mt6701_feedback_data *data = dev->data;

	return (float)atomic_get(&data->speed_millirpm) /
	       MT6701_FEEDBACK_MILLI_SCALE;
}

static const struct feedback_driver_api mt6701_feedback_api = {
	.get_eangle = mt6701_feedback_get_eangle,
	.get_speed = mt6701_feedback_get_speed,
};

static int mt6701_feedback_init(const struct device *dev)
{
	const struct mt6701_feedback_config *config = dev->config;
	struct mt6701_feedback_data *data = dev->data;
	int ret;

	if (config->pole_pairs == 0U || config->sampling_period_ms == 0U) {
		LOG_ERR("Pole-pair count and sampling period must be non-zero");
		return -EINVAL;
	}

	if (!device_is_ready(config->sensor)) {
		LOG_ERR("MT6701 sensor is not ready");
		return -ENODEV;
	}

	data->dev = dev;
	k_work_init_delayable(&data->sample_work, mt6701_feedback_sample_work);

	ret = mt6701_feedback_sample(dev);
	if (ret < 0) {
		LOG_ERR("Initial MT6701 sample failed (%d)", ret);
		return ret;
	}

	k_work_schedule(&data->sample_work, K_MSEC(config->sampling_period_ms));
	return 0;
}

#define MT6701_FEEDBACK_DEFINE(inst)                                            \
	static const struct mt6701_feedback_config mt6701_feedback_config_##inst = { \
		.sensor = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sensor)),                \
		.sampling_period_ms = DT_INST_PROP(inst, sampling_period_ms),          \
		.pole_pairs = DT_INST_PROP(inst, pole_pairs),                          \
		.phase_offset_degrees = DT_INST_PROP(inst, phase_offset_degrees),     \
	};                                                                          \
	static struct mt6701_feedback_data mt6701_feedback_data_##inst;             \
	DEVICE_DT_INST_DEFINE(inst, mt6701_feedback_init, NULL,                    \
			      &mt6701_feedback_data_##inst,                         \
			      &mt6701_feedback_config_##inst, POST_KERNEL,            \
			      91,                                                     \
			      &mt6701_feedback_api);

DT_INST_FOREACH_STATUS_OKAY(MT6701_FEEDBACK_DEFINE)