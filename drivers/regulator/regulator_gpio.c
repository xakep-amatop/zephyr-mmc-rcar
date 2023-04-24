/*
 * Copyright 2023 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT regulator_gpio

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(regulator_gpio, CONFIG_REGULATOR_LOG_LEVEL);

struct regulator_gpio_config {
	struct regulator_common_config common;
	struct gpio_dt_spec switch_volt;
	int initial_state;
	const int32_t *states;
	const uint8_t states_cnt;
};

struct regulator_gpio_data {
	struct regulator_common_data common;
	int32_t current_volt_uv;
};

static unsigned int regulator_gpio_count_voltages(const struct device *dev)
{
	const struct regulator_gpio_config *cfg = dev->config;

	return cfg->states_cnt / 2;
}

static int regulator_gpio_list_voltage(const struct device *dev, unsigned int idx,
				       int32_t *volt_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;

	if (idx >= cfg->states_cnt / 2) {
		LOG_ERR("%s: can't get list voltage for idx %u", dev->name, idx);
		return -EINVAL;
	}

	*volt_uv = cfg->states[idx * 2];
	return 0;
}

static int regulator_gpio_get_state_voltage(const struct device *dev, int state,
				       int32_t *volt_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;
	unsigned int state_idx = 0;

	for (; state_idx < cfg->states_cnt / 2; state_idx++) {
		if (cfg->states[state_idx * 2 + 1] == state) {
			*volt_uv = cfg->states[state_idx * 2];
			return 0;
		}
	}

	LOG_ERR("%s: can't get voltage for state %u", dev->name, state);
	return -EINVAL;
}

static int regulator_gpio_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;
	struct regulator_gpio_data *data = dev->data;
	unsigned int state = 0;
	int32_t mid = (min_uv + max_uv) / 2;
	int32_t min_diff = INT32_MAX;
	unsigned int best_state;
	int ret = 0;

	for (; state < cfg->states_cnt / 2; state++) {
		int32_t diff;

		if (!IN_RANGE(cfg->states[state * 2], min_uv, max_uv)) {
			continue;
		}

		diff = mid - cfg->states[state * 2];
		if (diff < 0) {
			diff = -diff;
		}

		if (diff < min_diff) {
			min_diff = diff;
			best_state = state;
		}
	}

	if (min_diff == INT32_MAX) {
		LOG_ERR("%s: can't find voltage is states", dev->name);
		return -EINVAL;
	}

	ret = gpio_pin_get_dt(&cfg->switch_volt);
	if (ret < 0) {
		LOG_ERR("%s: can't get pin state", dev->name);
		return ret;
	}

	if (ret != cfg->states[best_state * 2 + 1]) {
		ret = gpio_pin_set_dt(&cfg->switch_volt, cfg->states[best_state * 2 + 1]);
		if (ret < 0) {
			LOG_ERR("%s: can't set pin state", dev->name);
			return ret;
		}
	}

	data->current_volt_uv = cfg->states[best_state * 2];
	return 0;
}

static int regulator_gpio_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_gpio_data *data = dev->data;

	*volt_uv = data->current_volt_uv;
	return 0;
}

static const struct regulator_driver_api regulator_gpio_api = {
	.set_voltage = regulator_gpio_set_voltage,
	.get_voltage = regulator_gpio_get_voltage,
	.count_voltages = regulator_gpio_count_voltages,
	.list_voltage = regulator_gpio_list_voltage,
};

static int regulator_gpio_init(const struct device *dev)
{
	const struct regulator_gpio_config *cfg = dev->config;
	struct regulator_gpio_data *data = dev->data;
	int ret;

	regulator_common_data_init(dev);

	if (!device_is_ready(cfg->switch_volt.port)) {
		LOG_ERR("%s: gpio port: %s not ready", dev->name, cfg->switch_volt.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->switch_volt,
				    cfg->initial_state ? GPIO_OUTPUT_ACTIVE :
							 GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: can't configure pin", dev->name);
		return ret;
	}

	ret = regulator_gpio_get_state_voltage(dev, cfg->initial_state, &data->current_volt_uv);
	if (ret < 0) {
		LOG_ERR("%s: can't get list volatge for state %d", dev->name, cfg->initial_state);
		return ret;
	}

	ret = regulator_common_init(dev, true);
	if (ret < 0) {
		LOG_ERR("%s: can't init common part of regulator", dev->name);
		return ret;
	}

	return 0;
}

#define REGULATOR_GPIO_DEFINE(inst) \
	static struct regulator_gpio_data data##inst; \
	BUILD_ASSERT(!(DT_PROP_LEN(DT_DRV_INST(inst), states) & 0x1), \
		     "Number of regulator states should be even"); \
	static const struct regulator_gpio_config config##inst = { \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst), \
		.switch_volt = GPIO_DT_SPEC_INST_GET(inst, switch_voltage_gpios), \
		.initial_state = DT_PROP(DT_DRV_INST(inst), gpios_state), \
		.states = ((const int []) \
			      DT_PROP(DT_DRV_INST(inst), states)), \
		.states_cnt = DT_PROP_LEN(DT_DRV_INST(inst), states), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, regulator_gpio_init, NULL, &data##inst, \
			      &config##inst, POST_KERNEL, \
			      CONFIG_REGULATOR_GPIO_INIT_PRIORITY, \
			      &regulator_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_GPIO_DEFINE)
