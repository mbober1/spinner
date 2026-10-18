/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT spinner_current_sense

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <spinner/drivers/current_sense.h>
#include <spinner/drivers/currsmp.h>
#include <spinner/filter/low_pass_filter.h>

LOG_MODULE_REGISTER(current_sense, CONFIG_SPINNER_CURRENT_SENSE_LOG_LEVEL);

/*******************************************************************************
 * Private
 ******************************************************************************/

struct current_sense_config {
	const struct device *currsmp;
	struct low_pass_filter_config filter_config;
	float amps_per_volt;
	k_thread_stack_t *stack;
	size_t stack_size;
};

struct current_sense_data {
	struct k_thread thread;
	struct k_spinlock lock;
	struct k_sem samples;
	struct currsmp_sample sample;
	volatile uint32_t overrun_count;
	current_sense_regulation_cb_t regulation_cb;
	void *regulation_ctx;
	struct low_pass_filter filter[3];
};

/**
 * @brief Sample ready callback, called from the ADC interrupt.
 *
 * Stores the latest raw sample and wakes the thread.
 */
static void current_sense_sample_isr(void *ctx)
{
	const struct device *dev = ctx;
	const struct current_sense_config *config = dev->config;
	struct current_sense_data *data;
	data = dev->data;

	if (k_sem_count_get(&data->samples) > 0) {
		data->overrun_count++;
	}

	currsmp_get_sample(config->currsmp, &data->sample);
	k_sem_give(&data->samples);
}

static void current_sense_volt_to_curr(const struct current_sense_config *config,
				       const struct currsmp_volt *volt,
				       struct currsmp_curr *curr)
{
	curr->i_a = volt->v_a * config->amps_per_volt;
	curr->i_b = volt->v_b * config->amps_per_volt;
	curr->i_c = volt->v_c * config->amps_per_volt;
}

static void current_sense_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct current_sense_config *config = dev->config;
	struct current_sense_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct currsmp_sample sample;
		struct currsmp_volt volt;
		struct currsmp_curr raw;
		struct currsmp_curr filtered;

		k_sem_take(&data->samples, K_FOREVER);
		sample = data->sample;

		/* counts -> volts -> amps */
		currsmp_convert(config->currsmp, &sample, &volt);
		current_sense_volt_to_curr(config, &volt, &raw);

		/* phase current filtering */
		filtered.i_a = low_pass_filter_process(&data->filter[0], raw.i_a);
		filtered.i_b = low_pass_filter_process(&data->filter[1], raw.i_b);
		filtered.i_c = low_pass_filter_process(&data->filter[2], raw.i_c);

		if (data->regulation_cb != NULL) {
			data->regulation_cb(data->regulation_ctx, &filtered);
		}
	}
}

/*******************************************************************************
 * API
 ******************************************************************************/

static void current_sense_drv_configure(const struct device *dev,
				       current_sense_regulation_cb_t regulation_cb,
				       void *ctx)
{
	struct current_sense_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->regulation_cb = regulation_cb;
	data->regulation_ctx = ctx;
	k_spin_unlock(&data->lock, key);
}

static void current_sense_drv_get_currents(const struct device *dev,
					   struct currsmp_curr *curr)
{
	const struct current_sense_config *config = dev->config;
	struct currsmp_sample sample;
	struct currsmp_volt volt;

	currsmp_get_sample(config->currsmp, &sample);
	currsmp_convert(config->currsmp, &sample, &volt);
	current_sense_volt_to_curr(config, &volt, curr);
}

static const struct current_sense_driver_api current_sense_driver_api = {
	.configure = current_sense_drv_configure,
	.get_currents = current_sense_drv_get_currents,
};

/*******************************************************************************
 * Initialization
 ******************************************************************************/

static int current_sense_init(const struct device *dev)
{
	const struct current_sense_config *config = dev->config;
	struct current_sense_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->currsmp)) {
		LOG_ERR("Current sampling device not ready");
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(data->filter); i++) {
		ret = low_pass_filter_init(&data->filter[i],
					   CONFIG_SPINNER_SVPWM_STM32_PWM_FREQ,
					   &config->filter_config);
		if (ret < 0) {
			LOG_ERR("Could not initialize current filter (%d)", ret);
			return ret;
		}
	}

	k_sem_init(&data->samples, 0, 1);

	k_thread_create(&data->thread, config->stack, config->stack_size,
    current_sense_thread, (void *)dev, NULL, NULL,
    CONFIG_SPINNER_CURRENT_SENSE_THREAD_PRIORITY, 0, K_NO_WAIT);
    
  currsmp_configure(config->currsmp, current_sense_sample_isr, (void *)dev);
	return 0;
}

#define CURRENT_SENSE_DEFINE(inst)                                                  \
	K_THREAD_STACK_DEFINE(current_sense_stack_##inst,                           \
			      CONFIG_SPINNER_CURRENT_SENSE_THREAD_STACK_SIZE);      \
	static const struct current_sense_config current_sense_config_##inst = {    \
		.currsmp = DEVICE_DT_GET(DT_INST_PHANDLE(inst, adc)),               \
		.filter_config = {                                                  \
			.cutoff_hz = DT_INST_PROP(inst, filter_cutoff_hz),          \
		},                                                                  \
		.amps_per_volt = 1000.0f / DT_INST_PROP(inst, csa_gain_mv_per_a),   \
		.stack = current_sense_stack_##inst,                                \
		.stack_size = K_THREAD_STACK_SIZEOF(current_sense_stack_##inst),    \
	};                                                                          \
	static struct current_sense_data current_sense_data_##inst;                 \
	DEVICE_DT_INST_DEFINE(inst, current_sense_init, NULL,                       \
			      &current_sense_data_##inst,                           \
			      &current_sense_config_##inst, POST_KERNEL,            \
			      CONFIG_SPINNER_CURRENT_SENSE_INIT_PRIORITY,           \
			      &current_sense_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CURRENT_SENSE_DEFINE)
