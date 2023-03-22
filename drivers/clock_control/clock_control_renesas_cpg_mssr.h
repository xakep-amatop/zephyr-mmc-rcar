/*
 * Copyright (c) 2022 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_
#define ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_

#ifdef CONFIG_SOC_SERIES_RCAR_GEN3
/* Software Reset Clearing Register offsets */
#define SRSTCLR(i)      (0x940 + (i) * 4)

/* CPG write protect offset */
#define CPGWPR          0x900

/* Realtime Module Stop Control Register offsets */
static const uint16_t mstpcr[] = {
	0x110, 0x114, 0x118, 0x11c,
	0x120, 0x124, 0x128, 0x12c,
	0x980, 0x984, 0x988, 0x98c,
};

/* Software Reset Register offsets */
static const uint16_t srcr[] = {
	0x0A0, 0x0A8, 0x0B0, 0x0B8,
	0x0BC, 0x0C4, 0x1C8, 0x1CC,
	0x920, 0x924, 0x928, 0x92C,
};

/* CAN-FD Clock Frequency Control Register */
#define CANFDCKCR                 0x244

/* Clock stop bit */
#define CANFDCKCR_CKSTP           BIT(8)

/* CANFD Clock */
#define CANFDCKCR_PARENT_CLK_RATE 800000000
#define CANFDCKCR_DIVIDER_MASK    0x1FF

/* Peripherals Clocks */
#define S3D4_CLK_RATE             66600000	/* SCIF	*/
#define S0D12_CLK_RATE            66600000	/* PWM	*/
#elif defined(CONFIG_SOC_SERIES_RCAR_GEN4)
/* Software Reset Clearing Register offsets */
#define SRSTCLR(i) (0x2C80 + (i) * 4)

/* CPG write protect offset */
#define CPGWPR      0x0

/* Realtime Module Stop Control Register offsets */
static const uint16_t mstpcr[] = {
	0x2D00, 0x2D04, 0x2D08, 0x2D0C,
	0x2D10, 0x2D14, 0x2D18, 0x2D1C,
	0x2D20, 0x2D24, 0x2D28, 0x2D2C,
	0x2D30, 0x2D34, 0x2D38, 0x2D3C,
	0x2D40, 0x2D44, 0x2D48, 0x2D4C,
	0x2D50, 0x2D54, 0x2D58, 0x2D5C,
	0x2D60, 0x2D64, 0x2D68, 0x2D6C,
};

/* Software Reset Register offsets */
static const uint16_t srcr[] = {
	0x2C00, 0x2C04, 0x2C08, 0x2C0C,
	0x2C10, 0x2C14, 0x2C18, 0x2C1C,
	0x2C20, 0x2C24, 0x2C28, 0x2C2C,
	0x2C30, 0x2C34, 0x2C38, 0x2C3C,
	0x2C40, 0x2C44, 0x2C48, 0x2C4C,
	0x2C50, 0x2C54, 0x2C58, 0x2C5C,
	0x2C60, 0x2C64, 0x2C68, 0x2C6C,
};
#endif

void rcar_cpg_write(uint32_t base_address, uint32_t reg, uint32_t val);

int rcar_cpg_mstp_clock_endisable(uint32_t base_address, uint32_t module, bool enable);

#endif /* ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_ */
