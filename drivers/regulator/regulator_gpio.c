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

	const struct gpio_dt_spec *gpios;
	uint8_t num_gpios;
	int gpios_states;

	const int32_t *states;
	uint8_t states_cnt;

	const struct gpio_dt_spec enable;
	int32_t startup_delay_us;

};

struct regulator_gpio_data {
	struct regulator_common_data common;
	int32_t current_volt_uv;
};

static int regulator_gpio_get_state_voltage(const struct device *dev, uint32_t state,
					    int32_t *volt_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;

	for (unsigned int state_idx = 0; state_idx < cfg->states_cnt; state_idx++) {
		if (cfg->states[state_idx * 2 + 1] == state) {
			*volt_uv = cfg->states[state_idx * 2];
			return 0;
		}
	}

	LOG_ERR("%s: can't get voltage for state %u", dev->name, state);
	return -EINVAL;
}

static int regulator_gpio_apply_state(const struct device *dev, uint32_t state)
{
	const struct regulator_gpio_config *cfg = dev->config;

	for (unsigned int gpio_idx = 0; gpio_idx < cfg->num_gpios; gpio_idx++) {
		int ret;
		int new_state_of_gpio = (state >> gpio_idx) & 0x1;

		ret = gpio_pin_get_dt(&cfg->gpios[gpio_idx]);
		if (ret < 0) {
			LOG_ERR("%s: can't get pin state", dev->name);
			return ret;
		}

		if (ret != new_state_of_gpio) {
			ret = gpio_pin_set_dt(&cfg->gpios[gpio_idx], new_state_of_gpio);
			if (ret < 0) {
				LOG_ERR("%s: can't set pin state", dev->name);
				return ret;
			}
		}
	}

	return 0;
}

static int regulator_gpio_enable(const struct device *dev)
{
	const struct regulator_gpio_config *cfg = dev->config;
	int ret;

	if (cfg->enable.port == NULL) {
		return 0;
	}

	ret = gpio_pin_set_dt(&cfg->enable, 1);
	if (ret < 0) {
		LOG_ERR("%s: can't enable regulator!", dev->name);
		return ret;
	}

	if (cfg->startup_delay_us > 0U) {
		k_sleep(K_USEC(cfg->startup_delay_us));
	}

	return 0;
}

static int regulator_gpio_disable(const struct device *dev)
{
	const struct regulator_gpio_config *cfg = dev->config;

	if (cfg->enable.port == NULL) {
		return 0;
	}

	return gpio_pin_set_dt(&cfg->enable, 0);
}

static unsigned int regulator_gpio_count_voltages(const struct device *dev)
{
	const struct regulator_gpio_config *cfg = dev->config;

	return cfg->states_cnt;
}

static int regulator_gpio_list_voltage(const struct device *dev, unsigned int idx, int32_t *volt_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;

	if (idx >= cfg->states_cnt) {
		LOG_ERR("%s: can't get list voltage for idx %u", dev->name, idx);
		return -EINVAL;
	}

	*volt_uv = cfg->states[idx * 2];
	return 0;
}

static int regulator_gpio_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_gpio_config *cfg = dev->config;
	struct regulator_gpio_data *data = dev->data;
	int32_t best_voltage = INT32_MAX;
	unsigned int best_state;
	int ret = 0;

	/* choose minimum possible voltage in range provided by a caller */
	for (unsigned int state_idx = 0; state_idx < cfg->states_cnt; state_idx++) {
		if (!IN_RANGE(cfg->states[state_idx * 2], min_uv, max_uv) ||
		    cfg->states[state_idx * 2] >= best_voltage) {
			continue;
		}

		best_voltage = cfg->states[state_idx * 2];
		best_state = cfg->states[state_idx * 2 + 1];
	}

	if (best_voltage == INT32_MAX) {
		LOG_ERR("%s: can't find voltage is states", dev->name);
		return -EINVAL;
	}

	if (best_voltage == data->current_volt_uv) {
		return 0;
	}

	ret = regulator_gpio_apply_state(dev, best_state);
	if (ret) {
		return ret;
	}

	data->current_volt_uv = best_voltage;
	return 0;
}

static int regulator_gpio_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_gpio_data *data = dev->data;

	*volt_uv = data->current_volt_uv;
	return 0;
}

static const struct regulator_driver_api regulator_gpio_api = {
	.enable = regulator_gpio_enable,
	.disable = regulator_gpio_disable,
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

	LOG_ERR("INIT 0000 %d", cfg->num_gpios);

	for (unsigned int gpio_idx = 0; gpio_idx < cfg->num_gpios; gpio_idx++) {
		int ret;

		if (!gpio_is_ready_dt(&cfg->gpios[gpio_idx])) {
			LOG_ERR("%s: gpio pin: %s not ready", dev->name,
				cfg->gpios[gpio_idx].port ? cfg->gpios[gpio_idx].port->name
							  : "null");
			return -ENODEV;
		}

		LOG_ERR("GPIO[%u] %p %d", gpio_idx, cfg->gpios[gpio_idx].port,
			cfg->gpios[gpio_idx].pin);

		ret = gpio_pin_configure_dt(&cfg->gpios[gpio_idx], GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("%s: can't configure pin (%d) as output", dev->name,
				cfg->gpios[gpio_idx].pin);
			return ret;
		}
	}

	if (cfg->enable.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->enable)) {
			LOG_ERR("%s: gpio pin: %s not ready", dev->name, cfg->enable.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->enable, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
		if (ret < 0) {
			LOG_ERR("%s: can't configure enable pin (%d) as output", dev->name,
				cfg->enable.pin);
			return ret;
		}
	}

	/* if init_uv is set, regulator_common_init uses it instead of applying states here */
	if (1 /*cfg->common.init_uv != INT32_MIN*/) {
		ret = regulator_gpio_apply_state(dev, cfg->gpios_states);
		if (ret) {
			return ret;
		}

		LOG_ERR("HERE 0001");
		ret = regulator_gpio_get_state_voltage(dev, cfg->gpios_states,
						       &data->current_volt_uv);
		if (ret < 0) {
			LOG_ERR("%s: can't get list volatge for state %d", dev->name,
				cfg->gpios_states);
			return ret;
		}
	}

	ret = regulator_common_init(dev, false);
	if (ret < 0) {
		LOG_ERR("%s: can't init common part of regulator", dev->name);
		return ret;
	}

	return 0;
}

#define REG_GPIO_CONTEXT_GPIOS_SPEC_ELEM(_node_id, _prop, _idx)                                    \
	GPIO_DT_SPEC_GET_BY_IDX(_node_id, _prop, _idx),

#define REG_GPIO_CONTEXT_GPIOS_FOREACH_ELEM(_node_id)                                              \
	DT_FOREACH_PROP_ELEM(_node_id, gpios, REG_GPIO_CONTEXT_GPIOS_SPEC_ELEM)

#define REG_GPIO_CONTEXT_GPIOS_INITIALIZE(_node_id)                                                \
	.gpios = (const struct gpio_dt_spec[]){COND_CODE_1(                                        \
		DT_NODE_HAS_PROP(_node_id, gpios),                                                 \
		(REG_GPIO_CONTEXT_GPIOS_FOREACH_ELEM(_node_id)), ({0}))},                          \
	.num_gpios = DT_PROP_LEN_OR(_node_id, gpios, 0),

#define REGULATOR_GPIO_DEFINE(inst)                                                                \
	static struct regulator_gpio_data data##inst = {                                           \
		.current_volt_uv = INT32_MAX,                                                      \
	};                                                                                         \
	BUILD_ASSERT(!(DT_PROP_LEN(DT_DRV_INST(inst), states) & 0x1),                              \
		     "Number of regulator states should be even");                                 \
	static const struct regulator_gpio_config config##inst = {                                 \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		REG_GPIO_CONTEXT_GPIOS_INITIALIZE(DT_DRV_INST(inst))                               \
		.enable = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),                       \
		.gpios_states = DT_PROP(DT_DRV_INST(inst), gpios_states),                          \
		.states = ((const int[])DT_PROP(DT_DRV_INST(inst), states)),                       \
		.states_cnt = DT_PROP_LEN(DT_DRV_INST(inst), states) / 2,                          \
		.startup_delay_us = DT_INST_PROP_OR(inst, startup_delay_us, 0),                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, regulator_gpio_init, NULL, &data##inst, &config##inst,         \
			      POST_KERNEL, CONFIG_REGULATOR_GPIO_INIT_PRIORITY,                    \
			      &regulator_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_GPIO_DEFINE)
