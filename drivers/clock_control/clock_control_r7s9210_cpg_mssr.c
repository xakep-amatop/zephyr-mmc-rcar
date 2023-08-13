/*
 * Copyright (c) 2023 EPAM Systems
 *
 * r7s9210 Clock Pulse Generator/Module Standby and Software Reset
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_cpg_mssr

#include <errno.h>
#include <stdlib.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/r7s9210_cpg_mssr.h>
#include <zephyr/irq.h>
#include "clock_control_renesas_cpg_mssr.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(clock_control_renesas);

#define FRQCR_OFFSET         0x0
#define FRQCR_PFC_OFFSET     0x0
#define FRQCR_PFC_MASK       0x3
#define FRQCR_BFC_OFFSET     0x4
#define FRQCR_BFC_MASK       0x3
#define FRQCR_IFC_OFFSET     0x8
#define FRQCR_IFC_MASK       0x3

#define CKIOSEL_OFFSET       0xF0
#define CKIOSEL_MASK         0x3

#define SCLKSEL_OFFSET       0xF4
#define SCLKSEL_SPICR_OFFSET 0x0
#define SCLKSEL_SPICR_MASK   0x3
#define SCLKSEL_HYMCR_OFFSET 0x4
#define SCLKSEL_HYMCR_MASK   0x3
#define SCLKSEL_OCTCR_OFFSET 0x8
#define SCLKSEL_OCTCR_MASK   0x3

#define RZ_CPG_SET_REG_VAL(reg, val, off, mask) \
	reg |= ((reg) & ~((mask) <<(off))) | ((CONFIG_RZ_A_CLOCK_CONTROL_SELECT_##val) << (off))

struct r7s9210_cpg_mssr_config {
	DEVICE_MMIO_ROM; /* Must be first */
};

struct r7s9210_cpg_mssr_data {
	struct renesas_cpg_mssr_data cmn; /* Must be first */
};

#define RZ_CORE_CLK_INF(id, off, parrent_id) \
	RENESAS_CORE_CLK_INFO_ITEM(id, off, parrent_id, RENESAS_CPG_NONE)

#define RZ_FIXED_PLL_CLK_INFO_ITEM(id, in_frq)			\
	{							\
		.domain		= CPG_CORE,			\
		.module		= id,				\
		.offset		= RENESAS_CPG_NONE,		\
		.parent_id	= RENESAS_CPG_NONE,		\
		.in_freq	= in_frq,			\
		.out_freq	= RENESAS_CPG_NONE,		\
		.status		= CLOCK_CONTROL_STATUS_ON,	\
	}

#define RZ_PLL_IN_FREQ DT_INST_PROP_BY_PHANDLE(0, clocks, clock_frequency)

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table core_props[] = {
	/* PLL */
	RZ_FIXED_PLL_CLK_INFO_ITEM(R7S9210_CLK_PLL, RZ_PLL_IN_FREQ),
	/* CPU clock */
	RZ_CORE_CLK_INF(R7S9210_CLK_I, FRQCR_OFFSET, R7S9210_CLK_PLL),
	/* Image processing clock */
	RZ_CORE_CLK_INF(R7S9210_CLK_G, FRQCR_OFFSET, R7S9210_CLK_B),
	/* Internal bus clock */
	RZ_CORE_CLK_INF(R7S9210_CLK_B, FRQCR_OFFSET, R7S9210_CLK_PLL),
	/* Peripheral clock 1 */
	RZ_CORE_CLK_INF(R7S9210_CLK_P1, FRQCR_OFFSET, R7S9210_CLK_PLL),
	/* Peripheral clock 1C */
	RZ_CORE_CLK_INF(R7S9210_CLK_P1C, FRQCR_OFFSET, R7S9210_CLK_P1),
	/* Peripheral clock 0 */
	RZ_CORE_CLK_INF(R7S9210_CLK_P0, FRQCR_OFFSET, R7S9210_CLK_PLL),
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table mod_props[] = {
	/* ostm */
	RENESAS_MOD_CLK_INFO_ITEM(34,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(35,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(36,	R7S9210_CLK_P1C),

	/* scif */
	RENESAS_MOD_CLK_INFO_ITEM(43,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(44,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(45,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(46,	R7S9210_CLK_P1C),
	RENESAS_MOD_CLK_INFO_ITEM(47,	R7S9210_CLK_P1C),

	/* usb */
	RENESAS_MOD_CLK_INFO_ITEM(60,	R7S9210_CLK_B),
	RENESAS_MOD_CLK_INFO_ITEM(61,	R7S9210_CLK_B),

	/* ether */
	RENESAS_MOD_CLK_INFO_ITEM(64,	R7S9210_CLK_B),
	RENESAS_MOD_CLK_INFO_ITEM(65,	R7S9210_CLK_B),

	/* spibsc */
	RENESAS_MOD_CLK_INFO_ITEM(83,	R7S9210_CLK_P1),

	/* i2c */
	RENESAS_MOD_CLK_INFO_ITEM(84,	R7S9210_CLK_P1),
	RENESAS_MOD_CLK_INFO_ITEM(85,	R7S9210_CLK_P1),
	RENESAS_MOD_CLK_INFO_ITEM(86,	R7S9210_CLK_P1),
	RENESAS_MOD_CLK_INFO_ITEM(87,	R7S9210_CLK_P1),

	/* spi */
	RENESAS_MOD_CLK_INFO_ITEM(95,	R7S9210_CLK_P1),
	RENESAS_MOD_CLK_INFO_ITEM(96,	R7S9210_CLK_P1),
	RENESAS_MOD_CLK_INFO_ITEM(97,	R7S9210_CLK_P1),

	/* sdhi */
	RENESAS_MOD_CLK_INFO_ITEM(100,	R7S9210_CLK_B),
	RENESAS_MOD_CLK_INFO_ITEM(101,	R7S9210_CLK_B),
	RENESAS_MOD_CLK_INFO_ITEM(102,	R7S9210_CLK_B),
	RENESAS_MOD_CLK_INFO_ITEM(103,	R7S9210_CLK_B),
};

/* Standby Control Register */
static const uint16_t STBCR[] = {
	0x10, 0x14, 0x410, 0x414, 0x418, 0x41C, 0x420, 0x424, 0x428, 0x42C
};

static const uint16_t STBREQ[] = {
	0x20, 0x24, 0x28
};

static const uint16_t STBACK[] = {
	0x30, 0x34, 0x38
};

struct stbcr_to_stbreq {
	uint8_t stbcr_id;
	uint8_t stbreq_id;
} stbcr_to_stbreq_map[] = {
	{20,  15},   {51,  13},   {56,  10},   {60,  32},
	{60,  33},   {61,  30},   {61,  31},   {64,  26},
	{65,  26},   {66,  27},   {76,  20},   {76,  21},
	{77,  23},   {81,  25},   {90,  24},   {100, 11},
	{101, 11},   {102, 12},   {103, 12},   {104, 22},
};

static uint32_t r7s9210_get_div_helper(uint32_t reg_val, uint32_t module)
{
	uint32_t divider = RENESAS_CPG_NONE;

	switch (module) {
	case R7S9210_CLK_PLL:
		divider = 1;
		break;
	case R7S9210_CLK_I:
		divider = 1 << (((reg_val >> FRQCR_IFC_OFFSET) & FRQCR_IFC_MASK) + 1);
		break;
	case R7S9210_CLK_G:
		divider = 1;
		break;
	case R7S9210_CLK_B:
		divider = 1 << (((reg_val >> FRQCR_BFC_OFFSET) & FRQCR_BFC_MASK) + 2);
		if (divider == 4) {
			divider = RENESAS_CPG_NONE;
		} else if (module == R7S9210_CLK_G) {
			/* Gφ is always twice Bφ */
			divider >>= 1;
		}
		break;
	case R7S9210_CLK_P1:
		divider = 1 << (((reg_val >> FRQCR_BFC_OFFSET) & FRQCR_BFC_MASK) + 2);
		if (divider == 4 || divider == 8) {
			divider = RENESAS_CPG_NONE;
		}
	case R7S9210_CLK_P1C:
		divider = 1;
		break;
	case R7S9210_CLK_P0:
		divider = 32;
		break;
	default:
		break;
	}

	return divider;
}

static uint32_t r7s9210_get_mul_helper(uint32_t reg_val, uint32_t module)
{
	uint32_t multiplier = 1;

	switch (module) {
	case R7S9210_CLK_PLL:
		if (RZ_PLL_IN_FREQ > RENESAS_CPG_MHZ(12)) {
			multiplier = 44;
		} else {
			multiplier = 88;
		}
		break;
	case R7S9210_CLK_G:
		multiplier = 2;
		break;
	default:
		break;
	}

	return multiplier;
}

static int r7s9210_set_rate_helper(uint32_t module, uint32_t *divider, uint32_t *div_mask)
{
	int ret = -ENOTSUP;

	if (!is_power_of_two(*divider)) {
		return -EINVAL;
	}

	switch (module) {
	case R7S9210_CLK_I:
		switch (*divider) {
			case 2:
				*divider = 0;
			case 4:
				*divider = 1;
			case 8:
				*divider = 2;
			case 16:
				*divider = 3;
			default:
				return -EINVAL;
		}
		*divider <<= FRQCR_IFC_OFFSET;
		*div_mask = FRQCR_IFC_MASK << FRQCR_IFC_OFFSET;
		break;
	case R7S9210_CLK_B:
		switch (*divider) {
			case 8:
				*divider = 1;
			case 16:
				*divider = 2;
			case 32:
				*divider = 3;
			default:
				return -EINVAL;
		}
		*divider <<= FRQCR_BFC_OFFSET;
		*div_mask = FRQCR_BFC_MASK << FRQCR_BFC_OFFSET;
		break;
	case R7S9210_CLK_P1:
		switch (*divider) {
			case 16:
				*divider = 2;
			case 32:
				*divider = 3;
			default:
				return -EINVAL;
		}
		*divider <<= FRQCR_PFC_OFFSET;
		*div_mask = FRQCR_PFC_MASK << FRQCR_PFC_OFFSET;
		break;
	default:
		break;
	}

	return ret;
}

static int cmp_stbcr_to_stbreq_items(const void *key, const void *element)
{
	const struct stbcr_to_stbreq *e = element;
	uint8_t module = (uintptr_t)key;

	if (e->stbcr_id == module) {
		return 0;
	} else if (e->stbcr_id < module) {
		return 1;
	} else {
		return -1;
	}
}

static inline uint8_t rz_cpg_get_srbreq_bitmask(uint32_t module, uint32_t *off)
{
	struct stbcr_to_stbreq *item;
	uint8_t stbreq_bitmask = 0;

	item = bsearch((void *)module, &stbcr_to_stbreq_map, ARRAY_SIZE(stbcr_to_stbreq_map),
		       sizeof(*item), cmp_stbcr_to_stbreq_items);

	if (item) {
		int32_t idx;
		int32_t item_idx = ARRAY_INDEX(stbcr_to_stbreq_map, item);

		for (idx = item_idx; idx > -1; idx--) {
			if (stbcr_to_stbreq_map[idx].stbcr_id == module) {
				stbreq_bitmask |= BIT(stbcr_to_stbreq_map[idx].stbreq_id % 10);
			} else {
				break;
			}
		}

		for (idx = item_idx; idx < ARRAY_SIZE(stbcr_to_stbreq_map); idx++) {
			if (stbcr_to_stbreq_map[idx].stbcr_id == module) {
				stbreq_bitmask |= BIT(stbcr_to_stbreq_map[idx].stbreq_id % 10);
			} else {
				break;
			}
		}

		*off = item->stbreq_id / 10 - 1;
	}

	return stbreq_bitmask;
}

static inline void rz_cpg_change_reg_bits(mem_addr_t reg, uint8_t bitmask, bool enable)
{
	uint8_t reg_val;

	reg_val = sys_read8(reg);

	if (enable) {
		reg_val &= ~bitmask;
	} else {
		reg_val |= bitmask;
	}

	sys_write8(reg_val, reg);
	/* according to the documentation we should perform a dummy-read the same register */
	sys_read8(reg);
}

static int rz_cpg_mstp_clock_endisable(uint32_t base, uint32_t module, bool enable)
{
	uint32_t stbcr_reg_idx = module / 10 - 1;
	uint8_t bit = module % 10;
	uint8_t mstp_bitmask = BIT(bit);
	uint32_t req_reg_idx;
	uint8_t stbreq_bitmask;
	uint8_t reg_val = 0;

	__ASSERT((bit < 8) && reg_idx < ARRAY_SIZE(STBCR),
		 "Invalid mod number for cpg: %d", module);

	stbreq_bitmask = rz_cpg_get_srbreq_bitmask(module, &req_reg_idx);

	if (enable) {
		if (stbreq_bitmask != 0) {
			uint32_t wait_cnt = 10;

			rz_cpg_change_reg_bits(base + STBREQ[req_reg_idx], stbreq_bitmask, enable);
			while (wait_cnt--) {
				reg_val = sys_read8(base + STBACK[req_reg_idx]) & stbreq_bitmask;

				if (!reg_val) {
					break;
				}

				k_busy_wait(5);
			}
		}
		if (reg_val) {
			return -EIO;
		}
		rz_cpg_change_reg_bits(base + STBCR[stbcr_reg_idx], mstp_bitmask, enable);
	} else {
		rz_cpg_change_reg_bits(base + STBCR[stbcr_reg_idx], mstp_bitmask, enable);
		if (stbreq_bitmask != 0) {
			uint32_t wait_cnt = 10;

			rz_cpg_change_reg_bits(base + STBREQ[req_reg_idx], stbreq_bitmask, enable);
			while (wait_cnt--) {
				reg_val = sys_read8(base + STBACK[req_reg_idx]) & stbreq_bitmask;

				if (reg_val) {
					break;
				}

				k_busy_wait(5);
			}
		}
		if (!reg_val) {
			return -EIO;
		}
	}

	return 0;
}

static int r7s9210_cpg_mssr_start_stop(const struct device *dev, clock_control_subsys_t sys,
					      bool enable)
{
	struct renesas_cpg_clk *clk = (struct renesas_cpg_clk *)sys;
	int ret = -EINVAL;

	if (!dev || !sys) {
		return -EINVAL;
	}

	if (clk->domain == CPG_MOD) {
		struct r7s9210_cpg_mssr_data *data = dev->data;
		k_spinlock_key_t key;
		key = k_spin_lock(&data->cmn.lock);
		ret = rz_cpg_mstp_clock_endisable(DEVICE_MMIO_GET(dev), clk->module, enable);
		k_spin_unlock(&data->cmn.lock, key);
	} else if (clk->domain == CPG_CORE) {
		LOG_WRN("%s: enabling/disabling of core clocks aren't supported by this controller",
			dev->name);
		ret = -ENOTSUP;
	}

	return ret;
}

static int r7s9210_cpg_mssr_start(const struct device *dev, clock_control_subsys_t sys)
{
	return r7s9210_cpg_mssr_start_stop(dev, sys, true);
}

static int r7s9210_cpg_mssr_stop(const struct device *dev, clock_control_subsys_t sys)
{
	return r7s9210_cpg_mssr_start_stop(dev, sys, false);
}

static int r7s9210_cpg_mssr_init(const struct device *dev)
{
	uint16_t reg;

	renesas_cpg_build_clock_relationship(dev);
	renesas_cpg_update_all_in_out_freq(dev);

	reg = sys_read16(DEVICE_MMIO_GET(dev) + CKIOSEL_OFFSET);
	RZ_CPG_SET_REG_VAL(reg, CKIO, 0, CKIOSEL_MASK);
	if (reg != sys_read16(DEVICE_MMIO_GET(dev) + CKIOSEL_OFFSET)) {
		sys_write16(reg, DEVICE_MMIO_GET(dev) + CKIOSEL_OFFSET);
	}

	/* maybe better to add some property to flash memory nodes and use here */
	reg = sys_read16(DEVICE_MMIO_GET(dev) + SCLKSEL_OFFSET);
	RZ_CPG_SET_REG_VAL(reg, SPI_MULTI_IO_BUS, SCLKSEL_SPICR_OFFSET, SCLKSEL_SPICR_MASK);
	RZ_CPG_SET_REG_VAL(reg, HYPER_BUS,        SCLKSEL_HYMCR_OFFSET, SCLKSEL_HYMCR_MASK);
	RZ_CPG_SET_REG_VAL(reg, OCTA_MEMORY,      SCLKSEL_OCTCR_OFFSET, SCLKSEL_OCTCR_MASK);
	if (reg != sys_read16(DEVICE_MMIO_GET(dev) + SCLKSEL_OFFSET)) {
		sys_write16(reg, DEVICE_MMIO_GET(dev) + SCLKSEL_OFFSET);
	}

	return 0;
}

static const struct clock_control_driver_api r7s9210_cpg_mssr_api = {
	.on = r7s9210_cpg_mssr_start,
	.off = r7s9210_cpg_mssr_stop,
	.get_rate = renesas_cpg_get_rate,
	.set_rate = renesas_cpg_set_rate,
};

#define R7S9210_MSSR_INIT(inst)                                                                    \
	static struct r7s9210_cpg_mssr_config r7s9210_cpg_mssr##inst##_config = {                  \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
                                                                                                   \
	static struct r7s9210_cpg_mssr_data r7s9210_cpg_mssr##inst##_data = {                      \
		.cmn.clk_info_table[CPG_CORE] = core_props,                                        \
		.cmn.clk_info_table_size[CPG_CORE] = ARRAY_SIZE(core_props),                       \
		.cmn.clk_info_table[CPG_MOD] = mod_props,                                          \
		.cmn.clk_info_table_size[CPG_MOD] = ARRAY_SIZE(mod_props),                         \
		.cmn.get_div_helper = r7s9210_get_div_helper,                                      \
		.cmn.get_mul_helper = r7s9210_get_mul_helper,                                      \
		.cmn.set_rate_helper = r7s9210_set_rate_helper};                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, r7s9210_cpg_mssr_init, NULL, &r7s9210_cpg_mssr##inst##_data,   \
			      &r7s9210_cpg_mssr##inst##_config, PRE_KERNEL_1,                      \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &r7s9210_cpg_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(R7S9210_MSSR_INIT)
