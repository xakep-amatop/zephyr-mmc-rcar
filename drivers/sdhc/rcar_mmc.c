/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_mmc

#include <zephyr/devicetree.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>
#include <soc.h>

#include "rcar_mmc_registers.h"

/**
 * @note we don't need any locks here, because SDHC subsystem cares about it
 */

LOG_MODULE_REGISTER(rcar_mmc, CONFIG_LOG_DEFAULT_LEVEL);

/* SD-IF2 Clock Frequency Control Register offset */
#define SDIF2_CFCR_OFFSET 0x268

/* initial divider of SD clock register after soft reset */
#define MMC_INIT_SD_CLK_DIV_SHIFT 7

#define MMC_POLL_FLAGS_TIMEOUT_US 100000
#define MMC_POLL_FLAGS_ONE_CYCLE_TIMEOUT_US 1

/**
 * @brief Renesas MMC host controller driver data
 *
 */
struct mmc_rcar_data {
	struct sdhc_io host_io;
	struct sdhc_host_props props;

	uint8_t ver;
	/*
	 * in bytes, possible values are 2, 4 or 8
	 * TODO: add possibility of working with 2/4 bytes width
	 */
	uint8_t width_access_sd_buf0;
	uint8_t ddr_mode;
};

/**
 * @brief Renesas MMC host controller driver configuration
 */
struct mmc_rcar_cfg {
	struct rcar_cpg_clk cpg_clk;
	uint32_t reg_addr;
	const struct device *cpg_dev;
	const struct pinctrl_dev_config *pcfg;

	uint32_t max_frequency;
	uint8_t non_removable;
	uint8_t uhs_support;
	uint8_t mmc_hs200_1_8v;
	uint8_t mmc_hs400_1_8v;
	uint8_t bus_width;
	uint8_t mmc_sdr104_support;
};

/**
 * @brief Read 32-bit Renesas MMC register
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cfg MMC driver configuration, used for getting reg base address
 * @param reg register offset relative to the base address
 *
 * @retval register value
 */
static uint32_t rcar_mmc_read_reg32(const struct mmc_rcar_cfg *cfg,
	uint32_t reg)
{
	return sys_read32(cfg->reg_addr + reg);
}

/**
 * @brief Write 32-bit Renesas MMC register
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cfg MMC driver configuration, used for getting reg base address
 * @param reg register offset relative to the base address
 * @param val value that should be written to register
 *
 * @retval none
 */
static void rcar_mmc_write_reg32(const struct mmc_rcar_cfg *cfg,
	uint32_t reg, uint32_t val)
{
	sys_write32(val, cfg->reg_addr + reg);
}

/**
 * @brief Cleanup SD card interrupt flag register and mask their interrupts
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cfg MMC driver configuration, used for getting reg base address
 *
 * @retval none
 */
static inline void rcar_mmc_reset_and_mask_irqs(const struct mmc_rcar_cfg *cfg)
{
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1, 0);
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1_MASK, ~0);

	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO2, RCAR_MMC_INFO2_CLEAR);
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO2_MASK, ~0);
}

/**
 * @brief check if MMC is busy
 *
 * This check should generally be implemented as checking the controller
 * state. No MMC commands need to be sent.
 *
 * @param dev MMC device
 * @retval 0 card is not busy
 * @retval 1 card is busy
 * @retval -EINVAL: the dev pointer is NULL
 */
static int rcar_mmc_card_busy(const struct device *dev)
{
	const struct mmc_rcar_cfg *cfg;
	uint32_t reg;

	if (!dev || !dev->config) {
		return -EINVAL;
	}

	cfg = dev->config;
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO2);
	return (reg & RCAR_MMC_INFO2_CBSY) ? 1 : 0;
}

/**
 * @brief Check error flags inside INFO2 MMC register
 *
 * @note in/out parameters should be checked by a caller function
 *
 * @param cfg MMC driver configuration
 *
 * @retval 0 INFO2 register hasn't errors
 * @retval -ETIMEDOUT: timed out while tx/rx
 * @retval -EIO: I/O error
 * @retval -EILSEQ: communication out of sync
 */
static int rcar_mmc_check_errors(const struct mmc_rcar_cfg *cfg)
{
	uint32_t info2 = rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO2);

	if (info2 & (RCAR_MMC_INFO2_ERR_TO | RCAR_MMC_INFO2_ERR_RTO)) {
		LOG_ERR("timeout error 0x%08x", info2);
		return -ETIMEDOUT;
	}

	if (info2 & (RCAR_MMC_INFO2_ERR_END | RCAR_MMC_INFO2_ERR_CRC | RCAR_MMC_INFO2_ERR_IDX)) {
		LOG_ERR("communication out of sync 0x%08x", info2);
		return -EILSEQ;
	}

	if (info2 & (RCAR_MMC_INFO2_ERR_ILA | RCAR_MMC_INFO2_ERR_ILR |  RCAR_MMC_INFO2_ERR_ILW)) {
		LOG_ERR("illegal access 0x%08x", info2);
		return -EIO;
	}

	return 0;
}

/**
 * @brief Poll flag(s) in MMC register and check errors
 *
 * @note in/out parameters should be checked by a caller function
 *
 * @param cfg MMC driver configuration
 * @param reg register offset relative to the base address
 * @param flag polling flag(s)
 * @param state state of flag(s) when we should stop polling
 * @param check_errors call @ref rcar_mmc_check_errors function or not
 *
 * @retval 0 poll of flag(s) was successful
 * @retval -ETIMEDOUT: timed out while tx/rx
 * @retval -EIO: I/O error
 * @retval -EILSEQ: communication out of sync
 */
static int rcar_mmc_poll_reg_flags_check_err(const struct mmc_rcar_cfg *cfg,
	unsigned int reg, uint32_t flag, uint32_t state, bool check_errors)
{
	long wait = MMC_POLL_FLAGS_TIMEOUT_US;
	int ret;

	while ((rcar_mmc_read_reg32(cfg, reg) & flag) != state) {
		if (wait-- < 0) {
			LOG_ERR("timeout error during polling flag(s) 0x%08x in reg 0x%08x",
				flag, reg);
			return -ETIMEDOUT;
		}

		if (check_errors) {
			ret = rcar_mmc_check_errors(cfg);
			if (ret) {
				return ret;
			}
		}

		k_usleep(MMC_POLL_FLAGS_ONE_CYCLE_TIMEOUT_US);
	}

	return 0;
}

/**
 * @brief reset MMC controller state
 *
 * Used when the MMC has encountered an error. Resetting the MMC controller
 * should clear all errors on the MMC, but does not necessarily reset I/O
 * settings to boot (this can be done with @ref sdhc_set_io)
 *
 * @note during reset the clock input is disabled, also this call changes rate
 *
 * @param dev MMC controller device
 * @retval 0 reset succeeded
 * @retval -ETIMEDOUT: controller reset timed out
 * @retval -EINVAL: the dev pointer is NULL
 * @retval -EILSEQ: communication out of sync
 *
 * @todo add list of all affected registers to description of the function.
 */
static int rcar_mmc_reset(const struct device *dev)
{
	int ret = 0;
	uint32_t reg;
	const struct mmc_rcar_cfg *cfg;
	struct mmc_rcar_data *data;

	if (!dev || !dev->config) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = dev->data;

	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
				RCAR_MMC_INFO2_CBSY, 0, false);
	if (ret) {
		return -ETIMEDOUT;
	}

	/*
	 * soft reset of the host
	 */
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_SOFT_RST);
	reg &= ~RCAR_MMC_SOFT_RST_RSTX;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_SOFT_RST, reg);
	reg |= RCAR_MMC_SOFT_RST_RSTX;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_SOFT_RST, reg);

	rcar_mmc_reset_and_mask_irqs(cfg);

#ifdef CONFIG_RCAR_MMC_DMA_SUPPORT
#error "Add reset of DMAC"
#endif

	data->ddr_mode = 0;

	data->host_io.bus_width = SDHC_BUS_WIDTH4BIT;
	data->host_io.clock = data->props.f_max >> MMC_INIT_SD_CLK_DIV_SHIFT;
	data->host_io.timing = SDHC_TIMING_LEGACY;

	return 0;
}

/**
 * @brief SD Clock (SD_CLK) Output Control Enable
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cfg MMC driver configuration
 * @param enable
 *          false: SD_CLK output is disabled. The SD_CLK signal is fixed 0.
 *          true:  SD_CLK output is enabled.
 *
 * @retval 0 I/O was configured correctly
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_enable_clock(const struct mmc_rcar_cfg *cfg, bool enable)
{
	int ret;
	uint32_t mmc_clk_ctl = rcar_mmc_read_reg32(cfg, RCAR_MMC_CLKCTL);

	if (enable == true) {
		mmc_clk_ctl &= ~RCAR_MMC_CLKCTL_OFFEN;
		mmc_clk_ctl |= RCAR_MMC_CLKCTL_SCLKEN;
	} else {
		mmc_clk_ctl |= RCAR_MMC_CLKCTL_OFFEN;
		mmc_clk_ctl &= ~RCAR_MMC_CLKCTL_SCLKEN;
	}

	/*
	 * Do not change the values of these bits
	 * when the CBSY bit in SD_INFO2 is 1
	 */
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
				RCAR_MMC_INFO2_CBSY, 0, false);
	if (ret) {
		return -ETIMEDOUT;
	}
	rcar_mmc_write_reg32(cfg, RCAR_MMC_CLKCTL, mmc_clk_ctl);

	/* SD spec recommends at least 1 ms of delay */
	k_msleep(1);

	return 0;
}

/**
 * @brief Convert SDHC response to Renesas MMC response
 *
 * Function performs a conversion from SDHC response to Renesas MMC
 * CMD register response.
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param response_type SDHC response type without SPI flags
 *
 * @retval positiv number (partial configuration of CMD register) on
 *         success, negative errno code otherwise
 */
static int32_t rcar_mmc_convert_sd_to_mmc_resp(uint32_t response_type)
{
	uint32_t mmc_resp = 0U;

	switch (response_type) {
	case SD_RSP_TYPE_NONE:
		mmc_resp = RCAR_MMC_CMD_RSP_NONE;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		mmc_resp = RCAR_MMC_CMD_RSP_R1;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		mmc_resp = RCAR_MMC_CMD_RSP_R1B;
		break;
	case SD_RSP_TYPE_R2:
		mmc_resp = RCAR_MMC_CMD_RSP_R2;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		mmc_resp = RCAR_MMC_CMD_RSP_R3;
		break;
	default:
		LOG_ERR("unknown response type 0x%08x",
			response_type);
		return -EINVAL;
	}

	if ((int32_t)mmc_resp < 0) {
		LOG_WRN("%s: converted response shouldn't be negative", __func__);
	}

	return mmc_resp;
}

/**
 * @brief Convert response from Renesas MMC to SDHC
 *
 * Function writes a response to response array of @ref sdhc_command structure
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cfg MMC driver configuration
 * @param cmd MMC command
 * @param response_type SDHC response type without SPI flags
 *
 * @retval none
 */
static void rcar_mmc_extract_resp(const struct mmc_rcar_cfg *cfg,
	struct sdhc_command *cmd, uint32_t response_type)
{
	if (response_type == SD_RSP_TYPE_R2) {
		uint32_t rsp_127_104 = rcar_mmc_read_reg32(cfg, RCAR_MMC_RSP76);
		uint32_t rsp_103_72 = rcar_mmc_read_reg32(cfg, RCAR_MMC_RSP54);
		uint32_t rsp_71_40 = rcar_mmc_read_reg32(cfg, RCAR_MMC_RSP32);
		uint32_t rsp_39_8 = rcar_mmc_read_reg32(cfg, RCAR_MMC_RSP10);

		cmd->response[0] = (rsp_39_8 & 0xffffff) << 8;
		cmd->response[1] = ((rsp_71_40 & 0x00ffffff) << 8) |
				((rsp_39_8 & 0xff000000) >> 24);
		cmd->response[2] = ((rsp_103_72 & 0x00ffffff) << 8) |
				((rsp_71_40 & 0xff000000) >> 24);
		cmd->response[3] = ((rsp_127_104 & 0x00ffffff) << 8) |
				((rsp_103_72 & 0xff000000) >> 24);

		LOG_DBG("Response 2\n\t[0]: 0x%08x\n\t[1]: 0x%08x"
				"\n\t[2]: 0x%08x\n\t[3]: 0x%08x",
			cmd->response[0], cmd->response[1],
			cmd->response[2], cmd->response[3]);
	} else {
		cmd->response[0] = rcar_mmc_read_reg32(cfg, RCAR_MMC_RSP10);
		LOG_DBG("Response %u\n\t[0]: 0x%08x", response_type, cmd->response[0]);
	}
}

/**
 * @brief Configure CMD register for tx/rx data
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param cmd MMC command
 * @param data MMC data buffer for tx/rx
 *
 * @retval partial configuration of CMD register
 */
static uint32_t rcar_mmc_gen_data_cmd(struct sdhc_command *cmd,
	struct sdhc_data *data)
{
	uint32_t cmd_reg = RCAR_MMC_CMD_DATA;

	switch (cmd->opcode) {
	case MMC_SEND_EXT_CSD:
	case SD_READ_SINGLE_BLOCK:
		cmd_reg |= RCAR_MMC_CMD_RD;
		break;
	case SD_READ_MULTIPLE_BLOCK:
		cmd_reg |= RCAR_MMC_CMD_RD;
		cmd_reg |= RCAR_MMC_CMD_MULTI;
		break;
	case SD_WRITE_MULTIPLE_BLOCK:
		cmd_reg |= RCAR_MMC_CMD_MULTI;
		break;
	case SD_WRITE_SINGLE_BLOCK:
		/* fall through */
	default:
		break;
	}

	if (data->blocks > 1) {
		cmd_reg |= RCAR_MMC_CMD_MULTI;
	}

	return cmd_reg;
}

/**
 * @brief Transmit/Receive data to/from MMC
 *
 * Sends/Receives data to/from the MMC controller.
 *
 * @note in/out parameters should be checked by a caller function.
 *
 * @param dev MMC device
 * @param cfg MMC driver configuration
 * @param cmd MMC command
 * @param data MMC data buffer for tx/rx
 * @param is_read it is read or write operation
 *
 * @retval 0 tx/rx was successful
 * @retval -EINVAL: invalid block size
 * @retval -ETIMEDOUT: timed out while tx/rx
 * @retval -EIO: I/O error
 * @retval -EILSEQ: communication out of sync
 */
static int rcar_mmc_rx_tx_data(const struct device *dev,
	const struct mmc_rcar_cfg *cfg,
	struct sdhc_command *cmd,
	struct sdhc_data *data,
	bool is_read)
{
	struct mmc_rcar_data *dev_data = dev->data;
	uint32_t info1_reg;
	uint32_t block;
	int ret = 0;
	uint32_t info2_poll_flag = is_read ? RCAR_MMC_INFO2_BRE : RCAR_MMC_INFO2_BWE;

	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1_MASK,
			(uint32_t)~RCAR_MMC_INFO1_CMP);

	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO2_MASK,
			~(info2_poll_flag | RCAR_MMC_INFO2_ERRORS));

#ifdef CONFIG_RCAR_MMC_DMA_SUPPORT
#error "Add support of DMA request"
#else
	if ((data->block_size % dev_data->width_access_sd_buf0) ||
	    (data->block_size < dev_data->width_access_sd_buf0)) {
		/* TODO: fix me */
		LOG_ERR("%s: block size (%u) less or not align on SD BUF0 access width (%hhu)",
			dev->name, data->block_size, dev_data->width_access_sd_buf0);
		return -EINVAL;
	}

	for (block = 0; block < data->blocks; block++) {
		uint64_t *buf = (uint64_t *)data->data;
		uint32_t info2_reg;
		uint32_t w_off; /* word offset in a block */
		/* wait until the buffer is filled with data */
		ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
						info2_poll_flag, info2_poll_flag, true);
		if (ret) {
			return ret;
		}

		buf += (block * data->block_size) / sizeof(*buf);

		/* clear write/read buffer ready flag */
		info2_reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO2);
		info2_reg &= ~info2_poll_flag;
		rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO2, info2_reg);

		for (w_off = 0; (w_off * sizeof(*buf)) < data->block_size; w_off++) {
			if (is_read) {
				buf[w_off] = sys_read64(cfg->reg_addr + RCAR_MMC_BUF0);
			} else {
				sys_write64(buf[w_off], cfg->reg_addr + RCAR_MMC_BUF0);
			}
		}
	}
#endif
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO1,
					RCAR_MMC_INFO1_CMP, RCAR_MMC_INFO1_CMP, true);
	if (ret) {
		return ret;
	}

	/* clear access end flag  */
	info1_reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO1);
	info1_reg &= ~RCAR_MMC_INFO1_CMP;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1, info1_reg);

	return ret;
}

/**
 * @brief Send command to MMC
 *
 * Sends a command to the MMC controller.
 *
 * @param dev MMC device
 * @param cmd MMC command
 * @param data MMC data. Leave NULL to send SD command without data.
 *
 * @retval 0 command was sent successfully
 * @retval -ETIMEDOUT: command timed out while sending
 * @retval -ENOTSUP: host controller does not support command
 * @retval -EIO: I/O error
 * @retval -EILSEQ: communication out of sync
 */
static int rcar_mmc_request(const struct device *dev,
	struct sdhc_command *cmd,
	struct sdhc_data *data)
{
	int ret;
	const struct mmc_rcar_cfg *cfg;
	uint32_t reg;
	uint32_t response_type;
	bool is_read = true;

	if (!dev || !cmd || !dev->config) {
		return -EINVAL;
	}

	/* TODO: add retries mechanism, use retries counter from cmd for that */

	cfg = dev->config;
	response_type = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (rcar_mmc_card_busy(dev)) {
		LOG_ERR("%s: device busy", dev->name);
		return -EBUSY;
	}

	rcar_mmc_reset_and_mask_irqs(cfg);

	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1_MASK, (uint32_t)~RCAR_MMC_INFO1_RSP);
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO2_MASK, (uint32_t)~RCAR_MMC_INFO2_ERRORS);

	rcar_mmc_write_reg32(cfg, RCAR_MMC_ARG, cmd->arg);

	reg = cmd->opcode;

	if (data) {
		rcar_mmc_write_reg32(cfg, RCAR_MMC_SIZE, data->block_size);
		rcar_mmc_write_reg32(cfg, RCAR_MMC_SECCNT, data->blocks);
		reg |= rcar_mmc_gen_data_cmd(cmd, data);
		is_read = (reg & RCAR_MMC_CMD_RD) ? true : false;
	}

	ret = rcar_mmc_convert_sd_to_mmc_resp(response_type);
	if (ret < 0) {
		return -EINVAL;
	}

	reg |= ret;

	LOG_DBG("(SD_CMD=%08x, SD_ARG=%08x)", cmd->opcode, cmd->arg);
	rcar_mmc_write_reg32(cfg, RCAR_MMC_CMD, reg);

	/* wait until response end flag is set or errors occur */
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO1,
				   RCAR_MMC_INFO1_RSP, RCAR_MMC_INFO1_RSP, true);
	if (ret) {
		return ret;
	}

	/* clear response end flag */
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO1);
	reg &= ~RCAR_MMC_INFO1_RSP;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_INFO1, reg);

	rcar_mmc_extract_resp(cfg, cmd, response_type);

	if (response_type != SD_RSP_TYPE_NONE) {
		/* TODO: add cheching of RESP10 register here */
	}

	if (data) {
		ret = rcar_mmc_rx_tx_data(dev, cfg, cmd, data, is_read);
		if (ret) {
			return ret;
		}
	}

	/* wait until the SD bus (CMD, DAT) is free or errors occur */
	return rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
					RCAR_MMC_INFO2_SCLKDIVEN, RCAR_MMC_INFO2_SCLKDIVEN, true);
}

/**
 * @brief convert @ref sd_voltage to string
 *
 * @param voltage SD Voltage
 *
 * @retval pointer to string that represents voltage
 */
static inline const char * const
rcar_mmc_get_signal_voltage_str(enum sd_voltage voltage)
{
	static const char * const sig_vol_str[] = {
		[0] = "Unset",
		[SD_VOL_3_3_V] = "3.3V",
		[SD_VOL_3_0_V] = "3.0V",
		[SD_VOL_1_8_V] = "1.8V",
		[SD_VOL_1_2_V] = "1.2V",
	};

	if (voltage > -1 && voltage < ARRAY_SIZE(sig_vol_str)) {
		return sig_vol_str[voltage];
	} else {
		return "Unknown";
	}
}

/**
 * @brief convert @ref sdhc_timing_mode to string
 *
 * @param timing SDHC timing
 *
 * @retval pointer to string that represents timing
 */
static inline const char * const
rcar_mmc_get_timing_str(enum sdhc_timing_mode timing)
{
	static const char * const timing_str[] = {
		[0] = "Unset",
		[SDHC_TIMING_LEGACY] = "LEGACY",
		[SDHC_TIMING_HS] = "HS",
		[SDHC_TIMING_SDR12] = "SDR12",
		[SDHC_TIMING_SDR25] = "SDR25",
		[SDHC_TIMING_SDR50] = "SDR50",
		[SDHC_TIMING_SDR104] = "SDR104",
		[SDHC_TIMING_DDR50] = "DDR50",
		[SDHC_TIMING_DDR52] = "DDR52",
		[SDHC_TIMING_HS200] = "HS200",
		[SDHC_TIMING_HS400] = "HS400",
	};

	if (timing > -1 && timing < ARRAY_SIZE(timing_str)) {
		return timing_str[timing];
	} else {
		return "Unknown";
	}
}

/**
 * @brief change voltage of MMC
 *
 * @note in/out parameters should be checked by a caller function.
 * @note currently, this function do nothing, except some of checkings
 *
 * @param host_io old I/O properties
 * @param io new I/O properties
 *
 * @retval 0 I/O was configured correctly
 * @retval -ENOTSUP: controller does not support these I/O settings
 *
 * @todo configure pin voltage here, looks like MMC controller (GEN3)
 *       has access for both 1.8 and 3.3 voltages, but dataX/CMD pins
 *       work with 3.3V by default. According to documentation, we
 *       can configure it to work with 1.8V, look at POCCTRL0 reg.
 *       I belive that we need Renesas Regulator driver for that.
 */
static int rcar_mmc_change_voltage(struct sdhc_io *host_io,
	struct sdhc_io *ios)
{
	int ret = 0;

	/* Set host signal voltage */
	if (!ios->signal_voltage || ios->signal_voltage == host_io->signal_voltage) {
		return 0;
	}

	switch (ios->signal_voltage) {
	case SD_VOL_3_3_V:
		break;
	case SD_VOL_1_8_V:
		break;
	case SD_VOL_3_0_V:
	case SD_VOL_1_2_V:
		/* fall through */
	default:
		ret = -ENOTSUP;
		return ret;
	}

	host_io->signal_voltage = ios->signal_voltage;
	return ret;
}

/* note: for zero val function returns zero */
static inline uint32_t round_up_next_pwr_of_2(uint32_t val)
{
	val--;
	val |= val >> 1;
	val |= val >> 2;
	val |= val >> 4;
	val |= val >> 8;
	val |= val >> 16;
	return ++val;
}

/**
 * @brief configure clock divider on MMC controller
 *
 * @note In/out parameters should be checked by a caller function.
 * @note In the case of data transfer in HS400 mode (HS400 bit in
 *       SDIF_MODE = 1), do not set this width equal to 1.
 * @note In the case of writing of one-byte block, 8-bit width cannot
 *       be specified for the bus width. Change the bus width to 4 bits
 *       or 1 bit before writing one-byte block.
 *
 * @param dev MMC device
 * @param io I/O properties
 *
 * @retval 0 I/O was configured correctly
 * @retval -ENOTSUP: controller does not support these I/O settings
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_set_clk_rate(const struct device *dev,
	struct sdhc_io *ios)
{
	int ret = 0;
	uint32_t divisor;
	uint32_t mmc_clk_ctl;
	struct mmc_rcar_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
	const struct mmc_rcar_cfg *cfg = dev->config;

	if (host_io->clock == ios->clock) {
		return 0;
	}

	if (ios->clock == 0) {
		return 0;
	}

	if (ios->clock > data->props.f_max ||
	    ios->clock < data->props.f_min) {
		LOG_ERR("SDHC I/O: clock (%d) isn't in range %d - %d Hz",
			ios->clock, data->props.f_min, data->props.f_max);
		return -EINVAL;
	}

	/* TODO: get clock rate from clock dev instead */
	divisor = ceiling_fraction(data->props.f_max, ios->clock);

	/* Do not set divider to 0xff in DDR mode */
	if (data->ddr_mode && (divisor == 1)) {
		divisor = 2;
	}

	divisor = round_up_next_pwr_of_2(divisor);
	if (divisor == 0) {
		divisor = RCAR_MMC_CLKCTL_RCAR_DIV1;
	} else {
		divisor >>= 2;
	}

	/*
	 * Stop the clock before changing its rate
	 * to avoid a glitch signal
	 */
	ret = rcar_mmc_enable_clock(cfg, false);
	if (ret) {
		return ret;
	}

	mmc_clk_ctl = rcar_mmc_read_reg32(cfg, RCAR_MMC_CLKCTL);
	if ((mmc_clk_ctl & RCAR_MMC_CLKCTL_SCLKEN) &&
	    (mmc_clk_ctl & RCAR_MMC_CLKCTL_DIV_MASK) == divisor) {
		host_io->clock = ios->clock;
		return rcar_mmc_enable_clock(cfg, false);
	}

	/*
	 * Do not change the values of these bits
	 * when the CBSY bit in SD_INFO2 is 1
	 */
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
				RCAR_MMC_INFO2_CBSY, 0, false);
	if (ret) {
		return -ETIMEDOUT;
	}

	mmc_clk_ctl &= ~RCAR_MMC_CLKCTL_DIV_MASK;
	mmc_clk_ctl |= divisor;

	rcar_mmc_write_reg32(cfg, RCAR_MMC_CLKCTL, mmc_clk_ctl);
	ret = rcar_mmc_enable_clock(cfg, true);
	if (ret) {
		return ret;
	}

	host_io->clock = ios->clock;

	LOG_INF("%s: set clock rate to %d",
		dev->name, ios->clock);

	return 0;
}

/**
 * @brief set bus width of MMC
 *
 * @note In/out parameters should be checked by a caller function.
 * @note In the case of data transfer in HS400 mode (HS400 bit in
 *       SDIF_MODE = 1), do not set this width equal to 1.
 * @note In the case of writing of one-byte block, 8-bit width cannot
 *       be specified for the bus width. Change the bus width to 4 bits
 *       or 1 bit before writing one-byte block.
 *
 * @param dev MMC device
 * @param io I/O properties
 *
 * @retval 0 I/O was configured correctly
 * @retval -ENOTSUP: controller does not support these I/O settings
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_set_bus_width(const struct device *dev,
	struct sdhc_io *ios)
{
	int ret = 0;
	uint32_t mmc_option_reg;
	uint32_t reg_width;
	struct mmc_rcar_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
	const struct mmc_rcar_cfg *cfg = dev->config;

	/* Set bus width */
	if (host_io->bus_width == ios->bus_width) {
		return 0;
	}

	if (ios->bus_width) {
		return 0;
	}

	switch (ios->bus_width) {
	case SDHC_BUS_WIDTH1BIT:
		reg_width = RCAR_MMC_OPTION_WIDTH_1;
		break;
	case SDHC_BUS_WIDTH4BIT:
		if (data->props.host_caps.bus_4_bit_support) {
			reg_width = RCAR_MMC_OPTION_WIDTH_4;
		} else {
			LOG_ERR("SDHC I/O: 4-bits bus width isn't supported");
			return -ENOTSUP;
		}
		break;
	case SDHC_BUS_WIDTH8BIT:
		if (data->props.host_caps.bus_8_bit_support) {
			reg_width = RCAR_MMC_OPTION_WIDTH_8;
		} else {
			LOG_ERR("SDHC I/O: 8-bits bus width isn't supported");
			return -ENOTSUP;
		}
		break;
	default:
		return -ENOTSUP;
	}

	/*
	 * Do not change the values of these bits
	 * when the CBSY bit in SD_INFO2 is 1
	 */
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
				RCAR_MMC_INFO2_CBSY, 0, false);
	if (ret) {
		return -ETIMEDOUT;
	}

	mmc_option_reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_OPTION);
	mmc_option_reg &= ~RCAR_MMC_OPTION_WIDTH_MASK;
	mmc_option_reg |= reg_width;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_OPTION, mmc_option_reg);

	host_io->bus_width = ios->bus_width;
	return 0;
}

/**
 * @brief set DDR mode on MMC controller according to value inside
 * ddr_mode field from @ref mmc_rcar_data structure.
 *
 * @note In/out parameters should be checked by a caller function.
 *
 * @param dev MMC device
 *
 * @retval 0 set ddr flag successful
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_set_ddr_mode(const struct device *dev)
{
	int ret = 0;
	uint32_t if_mode_reg;
	struct mmc_rcar_data *data = dev->data;
	const struct mmc_rcar_cfg *cfg = dev->config;

	/*
	 * Do not change the values of these bits
	 * when the CBSY bit in SD_INFO2 is 1
	 */
	ret = rcar_mmc_poll_reg_flags_check_err(cfg, RCAR_MMC_INFO2,
				RCAR_MMC_INFO2_CBSY, 0, false);
	if (ret) {
		return -ETIMEDOUT;
	}

	if_mode_reg = rcar_mmc_read_reg32(dev->config, RCAR_MMC_IF_MODE);
	if (data->ddr_mode) {
		/* HS400 mode (DDR mode) */
		if_mode_reg |= RCAR_MMC_IF_MODE_DDR;
	} else {
		/* Normal mode (default, high speed, or SDR) */
		if_mode_reg &= ~RCAR_MMC_IF_MODE_DDR;
	}
	rcar_mmc_write_reg32(dev->config, RCAR_MMC_IF_MODE, if_mode_reg);

	return 0;
}

/**
 * @brief set timing property of MMC
 *
 * For now function only can enable DDR mode and call the function for
 * changing voltage. It is expectable that we change clock using another
 * I/O option.
 * @note In/out parameters should be checked by a caller function.
 *
 * @param dev MMC device
 * @param io I/O properties
 *
 * @retval 0 I/O was configured correctly
 * @retval -ENOTSUP: controller does not support these I/O settings
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_set_timings(const struct device *dev,
	struct sdhc_io *ios)
{
	int ret;
	struct mmc_rcar_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
	enum sd_voltage new_voltage = host_io->signal_voltage;

	if (host_io->timing == ios->timing) {
		return 0;
	}

	if (!host_io->timing) {
		return 0;
	}

	data->ddr_mode = 0;

	switch (ios->timing) {
	case SDHC_TIMING_LEGACY:
		break;
	case SDHC_TIMING_HS:
		if (!data->props.host_caps.high_spd_support) {
			LOG_ERR("SDHC I/O: HS timing isn't supported");
			return -ENOTSUP;
		}
		break;
	case SDHC_TIMING_SDR12:
	case SDHC_TIMING_SDR25:
		break;
	case SDHC_TIMING_SDR104:
		if (!data->props.host_caps.sdr104_support) {
			LOG_ERR("SDHC I/O: SDR104 timing isn't supported");
			return -ENOTSUP;
		}
		break;
	case SDHC_TIMING_HS400:
		if (!data->props.host_caps.hs400_support) {
			LOG_ERR("SDHC I/O: HS400 timing isn't supported");
			return -ENOTSUP;
		}
		new_voltage = SD_VOL_1_8_V;
		data->ddr_mode = 1;
		break;
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
		if (!data->props.host_caps.ddr50_support) {
			LOG_ERR("SDHC I/O: DDR50/DDR52 timing isn't supported");
			return -ENOTSUP;
		}
		data->ddr_mode = 1;
		break;
	case SDHC_TIMING_HS200:
		if (!data->props.host_caps.hs200_support) {
			LOG_ERR("SDHC I/O: HS200 timing isn't supported");
			return -ENOTSUP;
		}
		new_voltage = SD_VOL_1_8_V;
		break;
	default:
		return -ENOTSUP;
	}

	ios->signal_voltage = new_voltage;
	if (rcar_mmc_change_voltage(host_io, ios)) {
		return -ENOTSUP;
	}

	ret = rcar_mmc_set_ddr_mode(dev);
	if (ret) {
		return ret;
	}

	host_io->timing = ios->timing;
	return 0;
}

/**
 * @brief set I/O properties of MMC
 *
 * I/O properties should be reconfigured when the card has been sent a command
 * to change its own MMC settings. This function can also be used to toggle
 * power to the SD card.
 *
 * @param dev MMC device
 * @param io I/O properties
 *
 * @retval 0 I/O was configured correctly
 * @retval -ENOTSUP: controller does not support these I/O settings
 * @retval -EINVAL: some of pointers provided to the function are NULL
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	int ret = 0;
	const struct mmc_rcar_cfg *cfg;
	struct mmc_rcar_data *data;
	struct sdhc_io *host_io;

	if (!dev || !ios || !dev->data || !dev->config) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = dev->data;
	host_io = &data->host_io;

	LOG_DBG("SDHC I/O: bus width %d, clock %dHz, card power %s, "
		"timing %s, voltage %s",
		ios->bus_width,
		ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		rcar_mmc_get_timing_str(ios->timing),
		rcar_mmc_get_signal_voltage_str(ios->signal_voltage)
		);

	/* Set host clock */
	ret = rcar_mmc_set_clk_rate(dev, ios);
	if (ret) {
		LOG_ERR("SDHC I/O: can't change clock rate error %d old %d new %d",
			ret, host_io->clock, ios->clock);
		return ret;
	}

	/* Set card bus mode */
	if (ios->bus_mode && host_io->bus_mode != ios->bus_mode) {
		switch (ios->bus_mode) {
		case SDHC_BUSMODE_OPENDRAIN:
		case SDHC_BUSMODE_PUSHPULL:
			/*
			 * NOTE: do nothing for now, for most of scenarios it will be
			 *       enough to add appropriate properties to pinctrl inside dts
			 * TODO: make an investigation do we need to change it in runtime
			 *       or not and if answer yes add appropriate pinctrl api call
			 *       here
			 */
			break;
		default:
			LOG_ERR("SDHC I/O: not supported bus mode %d",
				ios->bus_mode);
			return -ENOTSUP;
		}
		host_io->bus_mode = ios->bus_mode;
	}

	/* Set card power */
	if (ios->power_mode && host_io->power_mode != ios->power_mode) {
		/*
		 * note: power is always on for eMMC cards,
		 *       so enable/disable clock instead
		 */
		switch (ios->power_mode) {
		case SDHC_POWER_ON:
			ret = rcar_mmc_enable_clock(cfg, true);
			break;
		case SDHC_POWER_OFF:
			ret = rcar_mmc_enable_clock(cfg, false);
			break;
		default:
			LOG_ERR("SDHC I/O: not supported power mode %d",
				ios->power_mode);
			return -ENOTSUP;
		}

		if (ret) {
			return ret;
		}
		host_io->power_mode = ios->power_mode;
	}

	ret = rcar_mmc_set_bus_width(dev, ios);
	if (ret) {
		LOG_ERR("SDHC I/O: can't change bus width error %d old %d new %d",
			ret, host_io->bus_width, ios->bus_width);
		return ret;
	}

	ret = rcar_mmc_set_timings(dev, ios);
	if (ret) {
		LOG_ERR("SDHC I/O: can't change timing error %d old %d new %d",
			ret, host_io->timing, ios->timing);
		return ret;
	}

	/* Set host driver type */
	if (ios->driver_type && ios->driver_type != host_io->driver_type) {
		switch (ios->driver_type) {
		case SD_DRIVER_TYPE_B:
		case SD_DRIVER_TYPE_A:
		case SD_DRIVER_TYPE_C:
		case SD_DRIVER_TYPE_D:
		default:
			LOG_ERR("SDHC I/O: not supported driver type %d",
				ios->driver_type);
			return -ENOTSUP;
		}
		host_io->driver_type = ios->driver_type;
	}

	ret = rcar_mmc_change_voltage(host_io, ios);
	if (ret) {
		LOG_ERR("SDHC I/O: can't change voltage! error %d old %d new %d",
			ret, host_io->signal_voltage, ios->signal_voltage);
		return ret;
	}

	return 0;
}

/**
 * @brief check for MMC card presence
 *
 * Checks if card is present on the bus.
 *
 * @param dev MMC device
 *
 * @retval 1 card is present
 * @retval 0 card is not present
 * @retval -EINVAL: some of pointers provided to the function are NULL
 */
static int rcar_mmc_get_card_present(const struct device *dev)
{
	const struct mmc_rcar_cfg *cfg;

	if (!dev || !dev->config) {
		return -EINVAL;
	}

	cfg = dev->config;
	if (cfg->non_removable) {
		return 1;
	}

	return !!(rcar_mmc_read_reg32(cfg, RCAR_MMC_INFO1) & RCAR_MMC_INFO1_CD);
}

/**
 * @brief run MMC tuning
 *
 * MMC cards require signal tuning for UHS modes SDR104, HS200 or HS400.
 * This function allows an application to request the SD host controller
 * to tune the card.
 *
 * @param dev MMC device
 *
 * @retval 0 tuning succeeded, card is ready for commands
 * @retval -ENOTSUP: controller does not support tuning
 * @retval -EINVAL: the dev pointer is NULL
 */
static int rcar_mmc_execute_tuning(const struct device *dev)
{
	if (!dev) {
		return -EINVAL;
	}

	LOG_ERR("%s: tuning isn't supported yet", dev->name);
	return -ENOTSUP;
}

/**
 * @brief Get MMC controller properties
 *
 * Gets host properties from the host controller. Host controller should
 * initialize all values in the @ref sdhc_host_props structure provided.
 *
 * @param dev Renesas MMC device
 * @param props property structure to be filled by MMC driver
 *
 * @retval 0 function succeeded.
 * @retval -EINVAL: some of pointers provided to the function are NULL
 */
static int rcar_mmc_get_host_props(const struct device *dev,
	struct sdhc_host_props *props)
{
	struct mmc_rcar_data *data;

	if (!props || !dev || !dev->data) {
		return -EINVAL;
	}

	data = dev->data;
	memcpy(props, &data->props, sizeof(*props));
	return 0;
}

static const struct sdhc_driver_api rcar_sdhc_api = {
	.card_busy = rcar_mmc_card_busy,
	.execute_tuning = rcar_mmc_execute_tuning,
	.get_card_present = rcar_mmc_get_card_present,
	.get_host_props = rcar_mmc_get_host_props,
	.request = rcar_mmc_request,
	.reset = rcar_mmc_reset,
	.set_io = rcar_mmc_set_io,
};

/*
 * TODO: share structure using header file or add API for get base
 *       address to CPG driver
 */
struct r8a7795_cpg_mssr_config {
	mm_reg_t base_address;
};

/**
 * @brief Start SD-IF2 clock at 200MHz
 *
 * @param cfg The Renesas MMC driver configuration
 *
 * @retval 0 on success, negative errno code otherwise
 */
static int rcar_mmc_init_start_clk(const struct mmc_rcar_cfg *cfg)
{
	unsigned int key;
	int ret = 0;
	const struct device *cpg_dev = cfg->cpg_dev;
	const struct r8a7795_cpg_mssr_config *clk_cfg = cpg_dev->config;

	key = irq_lock();

	/*
	 * TODO: fix me for Gen4
	 *
	 * RCAR Gen3 SD-IF2 clock frequency control register base + SDIF2_CFCR_OFFSET
	 * SDn = 200 MHz
	 */
	sys_write32(1, clk_cfg->base_address + SDIF2_CFCR_OFFSET);
	irq_unlock(key);

	ret = clock_control_on(cpg_dev, (clock_control_subsys_t *)&cfg->cpg_clk);
	return ret;
}

/**
 * @brief Initialize Host properties for working with SDHC and MMC stack
 *
 * @param dev The pointer to an MMC device structure
 *
 * @retval none
 */
static void rcar_mmc_init_host_props(const struct device *dev)
{
	struct mmc_rcar_data *data = dev->data;
	const struct mmc_rcar_cfg *cfg = dev->config;
	struct sdhc_host_props *props = &data->props;
	struct sdhc_host_caps *host_caps = &props->host_caps;

	memset(props, 0, sizeof(*props));

	/* Note: init only properties that are used for mmc/sdhc */

	props->f_max = cfg->max_frequency;
	/*
	 * note: actually, it's possible to get lower frequency
	 *       if we use divider from cpg too
	 */
	props->f_min = (cfg->max_frequency >> 9);

	props->power_delay = 100; /* ms */

	props->is_spi = 0;

	/* TODO: add support of 4/8 bits bus */
	host_caps->bus_8_bit_support = 0;
	host_caps->bus_4_bit_support = 0;

	/* TODO: add support to driver */
	host_caps->high_spd_support = 1;

	host_caps->vol_330_support = 1;
	host_caps->vol_300_support = 0;
	host_caps->vol_180_support = 1;

	host_caps->sdr50_support = 1;
	host_caps->sdr104_support = cfg->mmc_sdr104_support;
	host_caps->ddr50_support = 1;
	host_caps->uhs_2_support = cfg->uhs_support;
	host_caps->hs200_support = cfg->mmc_hs200_1_8v;
	host_caps->hs400_support = cfg->mmc_hs400_1_8v;
}

/**
 * @brief Reset sampling clock controller registers
 *
 * @param cfg The Renesas MMC driver configuration
 *
 * @retval 0 SCC reset successful
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_disable_scc(const struct mmc_rcar_cfg *cfg)
{
	int ret;
	uint32_t reg;

	/* just to be to be sure that the SD clock is disabled */
	ret = rcar_mmc_enable_clock(cfg, false);
	if (ret) {
		return ret;
	}

	/*
	 * Reset SCC registers, need to disable and enable clock
	 * before and after reset
	 */

	/* Disable SCC sampling clock */
	reg = rcar_mmc_read_reg32(cfg, RENESAS_SDHI_SCC_CKSEL);
	reg &= ~RENESAS_SDHI_SCC_CKSEL_DTSEL;
	rcar_mmc_write_reg32(cfg, RENESAS_SDHI_SCC_CKSEL, reg);

	/* disable hs400 mode & data output timing */
	reg = rcar_mmc_read_reg32(cfg, RENESAS_SDHI_SCC_TMPPORT2);
	reg &= ~(RENESAS_SDHI_SCC_TMPPORT2_HS400EN |
			 RENESAS_SDHI_SCC_TMPPORT2_HS400OSEL);
	rcar_mmc_write_reg32(cfg, RENESAS_SDHI_SCC_TMPPORT2, reg);

	ret = rcar_mmc_enable_clock(cfg, true);
	if (ret) {
		return ret;
	}

	/* disable SCC sampling clock position correction */
	reg = rcar_mmc_read_reg32(cfg, RENESAS_SDHI_SCC_RVSCNTL);
	reg &= ~RENESAS_SDHI_SCC_RVSCNTL_RVSEN;
	rcar_mmc_write_reg32(cfg, RENESAS_SDHI_SCC_RVSCNTL, reg);

	return 0;
}

/**
 * @brief Initialize and configure the Renesas MMC controller registers
 *
 * @param dev The pointer to an MMC device structure
 *
 * @retval 0 init MMC registers successful
 * @retval -ETIMEDOUT: card busy flag is set during long time
 */
static int rcar_mmc_init_controller_regs(const struct device *dev)
{
	int ret = 0;
	uint32_t reg;
	struct mmc_rcar_data *data = dev->data;
	const struct mmc_rcar_cfg *cfg = dev->config;
	struct sdhc_io ios = {0};

	rcar_mmc_reset(dev);

	/*  Disable SD clock (SD_CLK) output */
	ret = rcar_mmc_enable_clock(cfg, false);
	if (ret) {
		return ret;
	}

	/* set transfer data length to 0 */
	rcar_mmc_write_reg32(cfg, RCAR_MMC_SIZE, 0);

	/* disable the SD_BUF read/write DMA transfer */
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_EXTMODE);
	reg &= ~RCAR_MMC_EXTMODE_DMA_EN;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_EXTMODE, reg);
	/* mask DMA irqs and clear dma irq flags */
	rcar_mmc_reset_and_mask_irqs(cfg);
	/* set system address increment mode selector & 64-bit bus width */
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_DMA_MODE);
	reg |= RCAR_MMC_DMA_MODE_ADDR_INC | RCAR_MMC_DMA_MODE_WIDTH;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_DMA_MODE, reg);

	/* store version of of introductory IP */
	data->ver = rcar_mmc_read_reg32(cfg, RCAR_MMC_VERSION);
	data->ver &= RCAR_MMC_VERSION_IP;

	/*
	 * set bus width to 1
	 * timeout counter: SDCLK * 2^27
	 * card detect time counter: SDϕ * 2^24
	 */
	reg = rcar_mmc_read_reg32(cfg, RCAR_MMC_OPTION);
	reg |= RCAR_MMC_OPTION_WIDTH_MASK | 0xEE;
	rcar_mmc_write_reg32(cfg, RCAR_MMC_OPTION, reg);

	/* block count enable */
	rcar_mmc_write_reg32(cfg, RCAR_MMC_STOP, RCAR_MMC_STOP_SEC);
	/* number of transfer blocks */
	rcar_mmc_write_reg32(cfg, RCAR_MMC_SECCNT, 0);

	/*
	 * SD_BUF0 data swap disabled.
	 * Read/write access to SD_BUF0 can be performed with the 64-bit access.
	 *
	 * Note: when using the DMA, the bus width should be fixed at 64 bits.
	 */
	rcar_mmc_write_reg32(cfg, RCAR_MMC_HOST_MODE, 0);
	data->width_access_sd_buf0 = 8;

	/*
	 * disable sampling clock controller
	 * it is used for uhs/sdr104, hs200 and hs400
	 *
	 * TODO: add support of SCC tuning and UHS/HS200/HS400 modes
	 */
	ret = rcar_mmc_disable_scc(cfg);
	if (ret) {
		return ret;
	}

	/*
	 * configure divider inside MMC controller
	 * set maximum possible divider
	 */
	ios.clock = data->props.f_min;
	rcar_mmc_set_clk_rate(dev, &ios);

	return 0;
}

/**
 * @brief Initialize and configure the Renesas MMC driver
 *
 * @param dev The pointer to an MMC device structure
 *
 * @retval 0 on success, negative errno code otherwise
 */
static int rcar_mmc_init(const struct device *dev)
{
	int ret = 0;
	struct mmc_rcar_data *data = dev->data;
	const struct mmc_rcar_cfg *cfg = dev->config;

	if (unlikely(cfg == NULL)) {
		LOG_ERR("%s: there isn't config inside device structure", dev->name);
		return -EFAULT;
	}

	if (unlikely(dev->data == NULL)) {
		LOG_ERR("%s: there isn't data instance inside device structure",
			dev->name);
		return -EFAULT;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: error can't apply pinctrl state", dev->name);
		return ret;
	}

	if (!device_is_ready(cfg->cpg_dev)) {
		LOG_ERR("%s: error cpg_dev isn't ready", dev->name);
		return -ENODEV;
	}

	/*
	 * TODO: configure pin voltage here, looks like MMC controller (GEN3)
	 *       has access for both 1.8 and 3.3 voltages, but dataX/CMD pins
	 *       work with 3.3V by default. According to documentation, we
	 *       can configure it to work with 1.8V, look at POCCTRL0 reg.
	 *       I belive that we need Renesas Regulator driver for that.
	 *
	 * note: u-boot already configured this pin (dataX/cmd) as 1.8V IO.
	 */

	/*
	 * TODO: call clock_control_set_rate instead of the next lines here,
	 * but due to a migration of this driver to XenVM, maybe it isn't necessary
	 */
	ret = rcar_mmc_init_start_clk(cfg);
	if (ret < 0) {
		LOG_ERR("%s: error can't turn on the cpg", dev->name);
		return ret;
	}

	/* it's needed for SDHC */
	rcar_mmc_init_host_props(dev);

	ret = rcar_mmc_init_controller_regs(dev);
	if (ret) {
		return ret;
	}

	LOG_INF("%s: initialize driver, MMC version 0x%hhx",
		dev->name, data->ver);

	return 0;
}

#define MMC_RCAR_INIT(n) \
	static struct mmc_rcar_data mmc_rcar_data_##n; \
	PINCTRL_DT_INST_DEFINE(n);	\
	static const struct mmc_rcar_cfg mmc_rcar_cfg_##n = { \
		.reg_addr = DT_INST_REG_ADDR(n), \
		.cpg_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.cpg_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module), \
		.cpg_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
		.max_frequency = DT_INST_PROP(n, max_frequency), \
		.non_removable = DT_INST_PROP(n, non_removable), \
		.mmc_hs200_1_8v = DT_INST_PROP(n, mmc_hs200_1_8v), \
		.mmc_hs400_1_8v = DT_INST_PROP(n, mmc_hs400_1_8v), \
		.mmc_sdr104_support = DT_INST_PROP(n, mmc_sdr104_support), \
		.uhs_support = 1, \
	}; \
	DEVICE_DT_INST_DEFINE(n, \
				 rcar_mmc_init, \
				 NULL, \
				 &mmc_rcar_data_##n, \
				 &mmc_rcar_cfg_##n, \
				 POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, \
				 &rcar_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(MMC_RCAR_INIT)
