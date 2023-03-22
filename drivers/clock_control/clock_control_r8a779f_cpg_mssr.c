/*
 * Copyright (c) 2023 EPAM Systems
 *
 * r8a779f Clock Pulse Generator / Module Standby and Software Reset
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a779f_cpg_mssr

#include <errno.h>
#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>
#include <zephyr/irq.h>
#include "clock_control_renesas_cpg_mssr.h"

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_rcar);

/*
 * TODO: add locks to this driver and parent driver, because it can run
 *       on multicore system, for now all boards ue only one CPU, so it
 *       isn't a big problem.
 */

struct r8a779f_cpg_mssr_cfg {
	mm_reg_t base_address;
};

int r8a779f_cpg_mssr_start_stop(const struct device *dev,
	clock_control_subsys_t sys, bool enable)
{
	const struct r8a779f_cpg_mssr_cfg *config = dev->config;
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	int ret = -EINVAL;

	/* TODO: add support for CPG_CORE */
	if (clk->domain != CPG_MOD) {
		return ret;
	}
	return rcar_cpg_mstp_clock_endisable(config->base_address,
			clk->module, enable);
}

static int r8a779f_cpg_mssr_start(const struct device *dev,
	clock_control_subsys_t sys)
{
	return r8a779f_cpg_mssr_start_stop(dev, sys, true);
}

static int r8a779f_cpg_mssr_stop(const struct device *dev,
	clock_control_subsys_t sys)
{
	return r8a779f_cpg_mssr_start_stop(dev, sys, false);
}

static int r8a779f_cpg_mssr_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

/* TODO: add at least get/set rate */
static const struct clock_control_driver_api r8a779f_cpg_mssr_api = {
	.on = r8a779f_cpg_mssr_start,
	.off = r8a779f_cpg_mssr_stop,
};

#define R8A779F_MSSR_INIT(inst)						\
	static struct r8a779f_cpg_mssr_cfg cpg_mssr##inst##_cfg = {	\
		.base_address = DT_INST_REG_ADDR(inst)			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      &r8a779f_cpg_mssr_init,			\
			      NULL,					\
			      NULL, &cpg_mssr##inst##_cfg,		\
			      PRE_KERNEL_1,				\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
			      &r8a779f_cpg_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(R8A779F_MSSR_INIT)
