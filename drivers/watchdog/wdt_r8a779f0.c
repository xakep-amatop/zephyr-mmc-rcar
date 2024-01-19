/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a779f0_wdt

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_r8a779f0, CONFIG_WDT_LOG_LEVEL);

#define RWTCNT_OFF        0x0 /* RCLK watchdog timer counter */
#define RWTCNT_MASK       0xffff
#define RWTCNT_WR_PATTERN 0x5a5a0000

#define RWTCSRA_OFF 0x4    /* RCLK watchdog timer control/status register A */
#define RWTCSRA_TME BIT(7) /* start stop timer */

#define RWTCSRB_OFF 0x8 /* RCLK watchdog timer control/status register B */

#define RWTCSR_MASK       0xff
#define RWTCSR_WR_PATTERN 0xa5a5a500

struct wdt_r8a779f0_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *cpg_dev;
	struct rcar_cpg_clk cpg_clk;
	void (*irq_cfg)(void);
};

struct wdt_r8a779f0_data {
	DEVICE_MMIO_RAM; /* Must be first */
	wdt_callback_t callback;
	bool timeout_valid;
	uint32_t tout_ms;
	uint32_t clk_freq;
	uint8_t cks;
};

/* todo: add support of extended mode cks1 */
static const uint16_t cks0_div[] = {1, 4, 16, 32, 64, 128, 1024, 4096};

static void wdt_r8a779f0_write_reg(const struct device *dev, mem_addr_t off, uint32_t val)
{
	if (off == RWTCNT_OFF) {
		val &= RWTCNT_MASK;
		val |= RWTCNT_WR_PATTERN;
	} else {
		val &= RWTCSR_MASK;
		val |= RWTCSR_WR_PATTERN;
	}
	sys_write32(DEVICE_MMIO_GET(dev) + off, val);
}

static int wdt_r8a779f0_feed(const struct device *dev, int channel_id)
{
	struct wdt_r8a779f0_data *data = dev->data;
	uint32_t cnt = 65536;

	cnt -= DIV_ROUND_UP((data->tout_ms * cks0_div[data->cks]), data->clk_freq) / 1000;
	wdt_r8a779f0_write_reg(dev, RWTCNT_OFF, cnt);
	return 0;
}

static int wdt_r8a779f0_setup(const struct device *dev, uint8_t options)
{
	struct wdt_r8a779f0_data *data = dev->data;
	uint8_t rwtcsra;

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_ERR("%s: watchdog doesn't support PAUSE_IN_SLEEP", dev->name);
		return -ENOTSUP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		LOG_ERR("%s: watchdog doesn't support PAUSE_HALTED_BY_DBG", dev->name);
		return -ENOTSUP;
	}

	if (!data->timeout_valid) {
		LOG_ERR("%s: no valid timeouts installed", dev->name);
		return -EINVAL;
	}

	wdt_r8a779f0_write_reg(dev, RWTCSRA_OFF, data->cks);
	wdt_r8a779f0_feed(dev, 0);

	rwtcsra = sys_read8(DEVICE_MMIO_GET(dev) + RWTCSRA_OFF);
	wdt_r8a779f0_write_reg(dev, RWTCSRA_OFF, rwtcsra | RWTCSRA_TME);

	return 0;
}

static int wdt_r8a779f0_disable(const struct device *dev)
{
	struct wdt_r8a779f0_data *data = dev->data;

	wdt_r8a779f0_write_reg(dev, RWTCSRA_OFF, data->cks);
	/* Delay 3 cycles before disabling module clock */
	k_usleep(DIV_ROUND_UP((3 * 1000000), data->clk_freq));

	data->timeout_valid = false;
	return 0;
}

static int wdt_r8a779f0_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct wdt_r8a779f0_data *data = dev->data;
	const struct wdt_r8a779f0_cfg *drv_cfg = dev->config;
	uint8_t cks0;

	if (data->timeout_valid) {
		LOG_ERR("%s: no more timeouts can be installed", dev->name);
		return -ENOMEM;
	}

	if (cfg->window.min > cfg->window.max) {
		LOG_ERR("%s: min above max timout inside window", dev->name);
		return -EINVAL;
	}

	/* search cks0 */
	for (cks0 = 0; cks0 < ARRAY_SIZE(cks0_div); cks0++) {
		uint32_t max_tout_ms = ((RWTCNT_MASK + 1) * cks0_div[cks0] * 1000) / data->clk_freq;

		if (cfg->window.max <= max_tout_ms) {
			data->tout_ms = cfg->window.max;
			data->cks = cks0;
			break;
		}

		if (cfg->window.min <= max_tout_ms) {
			data->tout_ms = cfg->window.max;
			data->cks = cks0;
			break;
		}
	}

	if (cks0 == ARRAY_SIZE(cks0_div)) {
		LOG_ERR("%s: requested timeout window isn't supported by devive", dev->name);
		return -EINVAL;
	}

	if (cfg->callback) {
		data->callback = cfg->callback;
		drv_cfg->irq_cfg();
	}

	data->timeout_valid = true;
	return 0;
}

static const struct wdt_driver_api wdt_r8a779f0_driver_api = {
	.setup = wdt_r8a779f0_setup,
	.disable = wdt_r8a779f0_disable,
	.install_timeout = wdt_r8a779f0_install_timeout,
	.feed = wdt_r8a779f0_feed,
};

static void wdt_r8a779f0_isr(const struct device *dev)
{
	struct wdt_r8a779f0_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, 0);
	}
}

static int wdt_r8a779f0_init(const struct device *dev)
{
	int ret;
	const struct wdt_r8a779f0_cfg *cfg = dev->config;
	struct wdt_r8a779f0_data *data = dev->data;

	if (!device_is_ready(cfg->cpg_dev)) {
		LOG_ERR("%s: error cpg_dev isn't ready", dev->name);
		return -EINVAL;
	}

	ret = clock_control_on(cfg->cpg_dev, (clock_control_subsys_t *)&cfg->cpg_clk);
	if (ret < 0) {
		LOG_ERR("%s: can't enable wdt clock", dev->name);
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	wdt_r8a779f0_write_reg(dev, RWTCSRA_OFF, 0);
	wdt_r8a779f0_write_reg(dev, RWTCSRB_OFF, 0);
	wdt_r8a779f0_write_reg(dev, RWTCNT_OFF, 0);

	data->clk_freq = 32768;
	data->timeout_valid = false;

	return 0;
}

#define R8A779F0_WDT_INIT(index)                                                                   \
                                                                                                   \
	static void wdt_r8a779f0_cfg_func_##index(void);                                           \
                                                                                                   \
	static const struct wdt_r8a779f0_cfg wdt_r8a779f0_cfg_##index = {                          \
		.cpg_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(index)),                              \
		.cpg_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(index, 0, module),                    \
		.cpg_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(index, 0, domain),                    \
		.irq_cfg = wdt_r8a779f0_cfg_func_##index,                                          \
	};                                                                                         \
                                                                                                   \
	static struct wdt_r8a779f0_data wdt_r8a779f0_data_##index;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &wdt_r8a779f0_init, NULL, &wdt_r8a779f0_data_##index,         \
			      &wdt_r8a779f0_cfg_##index, POST_KERNEL,                              \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &wdt_r8a779f0_driver_api);      \
                                                                                                   \
	static void wdt_r8a779f0_cfg_func_##index(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), wdt_r8a779f0_isr,   \
			    DEVICE_DT_INST_GET(index), DT_INST_IRQ(index, flags));                 \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

DT_INST_FOREACH_STATUS_OKAY(R8A779F0_WDT_INIT)
