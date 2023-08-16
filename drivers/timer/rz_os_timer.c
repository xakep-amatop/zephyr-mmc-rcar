/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>

#define DT_DRV_COMPAT renesas_ostm

DEVICE_MMIO_TOPLEVEL_STATIC(ostm_base, DT_DRV_INST(0));

#define OSTM_REG_ADDR(off) ((mm_reg_t)(DEVICE_MMIO_TOPLEVEL_GET(ostm_base) + (off)))

#define OSTM_CMP_OFFSET 0x0    /* compare register */
#define OSTM_CNT_OFFSET 0x4    /* counter register */

#define OSTM_TE_OFFSET  0x10   /* count enable status register */
#define OSTM_TE_ENABLE  BIT(0) /* timer enabled */

#define OSTM_TS_OFFSET  0x14   /* count start trigger register */
#define OSTM_TS_START   BIT(0) /* trigger start of the timer */

#define OSTM_TT_OFFSET  0x18   /* count stop trigger register */
#define OSTM_TT_STOP    BIT(0) /* trigger stop of the timer */

#define OSTM_CTL_OFFSET 0x20   /* control register */
/*
 * Bit 0 of CTL controls enabling/disabling of OSTMnTINT interrupt requests when counting starts
 *    0: Disables the interrupts when counting starts
 *    1: Enables the interrupts when counting starts
 */
#define OSTM_CTL_TRIG_IRQ_ON_START 1
/*
 * Bit 1 of CTL specifies the operating mode for the counter
 *    0: Interval timer mode
 *    1: Free-running comparison mode
 */
#define OSTM_CTL_INTERVAL 0
#define OSTM_CTL_FREERUN  2

extern int z_clock_hw_cycles_per_sec;

static struct k_spinlock lock;
static uint32_t cyc_per_tick;
static uint32_t ostm_last_cnt;
static uint32_t last_elapse_read_cnt;

static void ostm_irq_handler(const struct device *dev)
{
	uint32_t ticks_diff;

	ARG_UNUSED(dev);

	ticks_diff = sys_clock_cycle_get_32() - ostm_last_cnt;
	ticks_diff /= cyc_per_tick;

	ostm_last_cnt += ticks_diff * cyc_per_tick;
#if !defined(CONFIG_TICKLESS_KERNEL)
	sys_write32(ostm_last_cnt + cyc_per_tick, OSTM_REG_ADDR(OSTM_CMP_OFFSET));
#else
	last_elapse_read_cnt = 0;
	irq_disable(DT_INST_IRQN(0));
#endif

	/* Announce to the kernel */
	sys_clock_announce(ticks_diff);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
#if defined(CONFIG_TICKLESS_KERNEL)
	if (ticks == K_TICKS_FOREVER) {
		if (idle) {
			return;
		}
		ticks = INT32_MAX;
	}

	ticks = CLAMP(ticks, 0, (unsigned long)-1 / 2 / cyc_per_tick);
	ticks = CLAMP(ticks, 0, INT32_MAX / 2);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t next_cycle = ostm_last_cnt + (last_elapse_read_cnt + ticks) * cyc_per_tick;
	sys_write32(next_cycle, OSTM_REG_ADDR(OSTM_CMP_OFFSET));
	k_spin_unlock(&lock, key);

	irq_enable(DT_INST_IRQN(0));
#else  /* CONFIG_TICKLESS_KERNEL */
	ARG_UNUSED(ticks);
	ARG_UNUSED(idle);
#endif
}

void sys_clock_disable(void)
{
	if ((sys_read8(OSTM_REG_ADDR(OSTM_TE_OFFSET)) & OSTM_TE_ENABLE) != OSTM_TE_ENABLE) {
		return;
	}

	sys_write8(OSTM_TT_STOP, OSTM_REG_ADDR(OSTM_TT_OFFSET));
	while((sys_read8(OSTM_REG_ADDR(OSTM_TE_OFFSET)) & OSTM_TE_ENABLE) == OSTM_TE_ENABLE);
}

uint32_t sys_clock_elapsed(void)
{
	uint32_t ticks_diff;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	last_elapse_read_cnt = sys_clock_cycle_get_32();
	ticks_diff = last_elapse_read_cnt - ostm_last_cnt;
	return ticks_diff / cyc_per_tick;
}

uint32_t sys_clock_cycle_get_32(void)
{
	uint32_t ostm_cnt;
	k_spinlock_key_t key;

	key = k_spin_lock(&lock);
	ostm_cnt = sys_read32(OSTM_REG_ADDR(OSTM_CNT_OFFSET));
	k_spin_unlock(&lock, key);

	return ostm_cnt;
}

static int sys_clock_driver_init(void)
{
	int ret;
	const struct device *clk;

	struct renesas_cpg_clk mod_clk = {
		.module = DT_INST_CLOCKS_CELL(0, module),
		.domain = DT_INST_CLOCKS_CELL(0, domain),
	};

	clk = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));
	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&mod_clk);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&mod_clk,
				     &z_clock_hw_cycles_per_sec);
	if (ret < 0) {
		return ret;
	}

	cyc_per_tick  = sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	DEVICE_MMIO_TOPLEVEL_MAP(ostm_base, K_MEM_CACHE_NONE);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ostm_irq_handler,
		    NULL, DT_INST_IRQ(0, flags));

	/* restarting the timer will cause reset of CNT register in free-running mode */
	sys_clock_disable();

	ostm_last_cnt = 0;
	sys_write32(cyc_per_tick, OSTM_REG_ADDR(OSTM_CMP_OFFSET));
	sys_write8(OSTM_CTL_FREERUN, OSTM_REG_ADDR(OSTM_CTL_OFFSET));
	sys_write8(OSTM_TS_START, OSTM_REG_ADDR(OSTM_TS_OFFSET));

	irq_enable(DT_INST_IRQN(0));
	return 0;
}
SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
