/*
 * evm.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

//#define DEBUG

#include <common.h>
#include <asm/cache.h>
#include <asm/omap_common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/arch/nand.h>
#include <asm/arch/clock.h>
#include <linux/mtd/nand.h>
#include <nand.h>
#include <net.h>
#include <miiphy.h>
#include <netdev.h>
#include <spi_flash.h>
#include "common_def.h"
//#include "pmic.h"
#include "tps65217.h"
#include <i2c.h>
#include <serial.h>
#include "myir_header.h"

DECLARE_GLOBAL_DATA_PTR;

/* UART Defines */
#define UART_SYSCFG_OFFSET	(0x54)
#define UART_SYSSTS_OFFSET	(0x58)

#define UART_RESET		(0x1 << 1)
#define UART_CLK_RUNNING_MASK	0x1
#define UART_SMART_IDLE_EN	(0x1 << 0x3)

/* Timer Defines */
#define TSICR_REG		0x54
#define TIOCP_CFG_REG		0x10
#define TCLR_REG		0x38

/* CPLD registers */
#define CFG_REG			0x10

/*
 * I2C Address of various board
 */
#define I2C_BASE_BOARD_ADDR	0x50
#define I2C_DAUGHTER_BOARD_ADDR 0x51
#define I2C_LCD_BOARD_ADDR	0x52

#define I2C_CPLD_ADDR		0x35

/* RGMII mode define */
#define RGMII_MODE_ENABLE	0xA
#define RMII_MODE_ENABLE	0x5
#define MII_MODE_ENABLE		0x0

#define NO_OF_MAC_ADDR          3
#define ETH_ALEN		6

/* DDR Base address */
#define DDR_CTRL_ADDR						0x44E10E04
#define DDR_CONTROL_BASE_ADDR				0x44E11404
#define DDR_CKE_CTRL_NORMAL					0x1

/* DDR Base address */
#define DDR_PHY_CMD_ADDR					0x44E12000
#define DDR_PHY_DATA_ADDR					0x44E120C8
#define DDR_PHY_CMD_ADDR2					0x47C0C800
#define DDR_PHY_DATA_ADDR2					0x47C0C8C8
#define DDR_DATA_REGS_NR					2

/* SDRAM_REF_CTRL */
#define EMIF_REG_INITREF_DIS_SHIFT			31
#define EMIF_REG_INITREF_DIS_MASK			(1 << 31)
#define EMIF_REG_SRT_SHIFT					29
#define EMIF_REG_SRT_MASK					(1 << 29)
#define EMIF_REG_ASR_SHIFT					28
#define EMIF_REG_ASR_MASK					(1 << 28)
#define EMIF_REG_PASR_SHIFT					24
#define EMIF_REG_PASR_MASK					(0x7 << 24)
#define EMIF_REG_REFRESH_RATE_SHIFT			0
#define EMIF_REG_REFRESH_RATE_MASK			(0xffff << 0)

/* Micron MT41K256M16HA-125E */
#define MT41K256M16HA125E_EMIF_READ_LATENCY	0x100007
#define MT41K256M16HA125E_EMIF_TIM1			0x0AAAD4DB
#define MT41K256M16HA125E_EMIF_TIM2			0x266B7FDA
#define MT41K256M16HA125E_EMIF_TIM3			0x501F867F
#define MT41K256M16HA125E_EMIF_SDCFG		0x61C05332
#define MT41K256M16HA125E_EMIF_SDREF		0xC30
#define MT41K256M16HA125E_ZQ_CFG			0x50074BE4
#define MT41K256M16HA125E_RATIO				0x80
#define MT41K256M16HA125E_INVERT_CLKOUT		0x0
#define MT41K256M16HA125E_RD_DQS			0x38
#define MT41K256M16HA125E_WR_DQS			0x44
#define MT41K256M16HA125E_PHY_WR_DATA		0x7D
#define MT41K256M16HA125E_PHY_FIFO_WE		0x94
#define MT41K256M16HA125E_IOCTRL_VALUE		0x18B

struct am335x_baseboard_id {
	unsigned int  magic;
	char name[8];
	char version[4];
	char serial[12];
	char config[32];
	char mac_addr[NO_OF_MAC_ADDR][ETH_ALEN];
};

static struct am335x_baseboard_id header;
extern void cpsw_eth_set_mac_addr(const u_int8_t *addr);
static unsigned char daughter_board_connected;
static volatile int board_id = BASE_BOARD;

/*
 * dram_init:
 * At this point we have initialized the i2c bus and can read the
 * EEPROM which will tell us what board and revision we are on.
 */
int dram_init(void)
{
	gd->ram_size = PHYS_DRAM_1_SIZE;

	return 0;
}

void dram_init_banksize (void)
{
	/* Fill up board info */
	gd->bd->bi_dram[0].start = PHYS_DRAM_1;
	gd->bd->bi_dram[0].size = PHYS_DRAM_1_SIZE;
}

#ifdef CONFIG_SPL_BUILD
static void Data_Macro_Config(int dataMacroNum)
{
	u32 BaseAddrOffset = 0x00;;

	if (dataMacroNum == 1)
		BaseAddrOffset = 0xA4;

	__raw_writel(((DDR3_RD_DQS<<30)|(DDR3_RD_DQS<<20)
			|(DDR3_RD_DQS<<10)|(DDR3_RD_DQS<<0)),
			(DATA0_RD_DQS_SLAVE_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_RD_DQS>>2,
			(DATA0_RD_DQS_SLAVE_RATIO_1 + BaseAddrOffset));
	__raw_writel(((DDR3_WR_DQS<<30)|(DDR3_WR_DQS<<20)
			|(DDR3_WR_DQS<<10)|(DDR3_WR_DQS<<0)),
			(DATA0_WR_DQS_SLAVE_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_WR_DQS>>2,
			(DATA0_WR_DQS_SLAVE_RATIO_1 + BaseAddrOffset));
	
//	__raw_writel(1, (DATA0_WRLVL_INIT_MODE_0 + BaseAddrOffset));
//	__raw_writel(1, (DATA0_GATELVL_INIT_MODE_0 + BaseAddrOffset));
	
	__raw_writel(((DDR3_PHY_WRLVL<<30)|(DDR3_PHY_WRLVL<<20)
			|(DDR3_PHY_WRLVL<<10)|(DDR3_PHY_WRLVL<<0)),
			(DATA0_WRLVL_INIT_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_PHY_WRLVL>>2,
			(DATA0_WRLVL_INIT_RATIO_1 + BaseAddrOffset));
	__raw_writel(((DDR3_PHY_GATELVL<<30)|(DDR3_PHY_GATELVL<<20)
			|(DDR3_PHY_GATELVL<<10)|(DDR3_PHY_GATELVL<<0)),
			(DATA0_GATELVL_INIT_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_PHY_GATELVL>>2,
			(DATA0_GATELVL_INIT_RATIO_1 + BaseAddrOffset));
	__raw_writel(((DDR3_PHY_FIFO_WE<<30)|(DDR3_PHY_FIFO_WE<<20)
			|(DDR3_PHY_FIFO_WE<<10)|(DDR3_PHY_FIFO_WE<<0)),
			(DATA0_FIFO_WE_SLAVE_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_PHY_FIFO_WE>>2,
			(DATA0_FIFO_WE_SLAVE_RATIO_1 + BaseAddrOffset));
	__raw_writel(((DDR3_PHY_WR_DATA<<30)|(DDR3_PHY_WR_DATA<<20)
			|(DDR3_PHY_WR_DATA<<10)|(DDR3_PHY_WR_DATA<<0)),
			(DATA0_WR_DATA_SLAVE_RATIO_0 + BaseAddrOffset));
	__raw_writel(DDR3_PHY_WR_DATA>>2,
			(DATA0_WR_DATA_SLAVE_RATIO_1 + BaseAddrOffset));
	__raw_writel(PHY_DLL_LOCK_DIFF,
			(DATA0_DLL_LOCK_DIFF_0 + BaseAddrOffset));
}

static void Cmd_Macro_Config(void)
{
	__raw_writel(DDR3_RATIO, CMD0_CTRL_SLAVE_RATIO_0);
	__raw_writel(CMD_FORCE, CMD0_CTRL_SLAVE_FORCE_0);
	__raw_writel(CMD_DELAY, CMD0_CTRL_SLAVE_DELAY_0);
	__raw_writel(DDR3_DLL_LOCK_DIFF, CMD0_DLL_LOCK_DIFF_0);
	__raw_writel(DDR3_INVERT_CLKOUT, CMD0_INVERT_CLKOUT_0);

	__raw_writel(DDR3_RATIO, CMD1_CTRL_SLAVE_RATIO_0);
	__raw_writel(CMD_FORCE, CMD1_CTRL_SLAVE_FORCE_0);
	__raw_writel(CMD_DELAY, CMD1_CTRL_SLAVE_DELAY_0);
	__raw_writel(DDR3_DLL_LOCK_DIFF, CMD1_DLL_LOCK_DIFF_0);
	__raw_writel(DDR3_INVERT_CLKOUT, CMD1_INVERT_CLKOUT_0);

	__raw_writel(DDR3_RATIO, CMD2_CTRL_SLAVE_RATIO_0);
	__raw_writel(CMD_FORCE, CMD2_CTRL_SLAVE_FORCE_0);
	__raw_writel(CMD_DELAY, CMD2_CTRL_SLAVE_DELAY_0);
	__raw_writel(DDR3_DLL_LOCK_DIFF, CMD2_DLL_LOCK_DIFF_0);
	__raw_writel(DDR3_INVERT_CLKOUT, CMD2_INVERT_CLKOUT_0);
}

/*************************** VTP configure, MYiR *************************/
/* VTP Base address */
#define VTP0_CTRL_ADDR			0x44E10E0C
#define VTP1_CTRL_ADDR			0x48140E10

/* VTP Registers */
struct vtp_reg {
	unsigned int vtp0ctrlreg;
};


static struct vtp_reg *vtpreg[2] = {
				(struct vtp_reg *)VTP0_CTRL_ADDR,
				(struct vtp_reg *)VTP1_CTRL_ADDR};

/**
 * Base address for EMIF instances
 */
static struct emif_reg_struct *emif_reg[2] = {
				(struct emif_reg_struct *)EMIF4_0_CFG_BASE,
				(struct emif_reg_struct *)EMIF4_1_CFG_BASE};
/**
 * Base addresses for DDR PHY cmd/data regs
 */
static struct ddr_cmd_regs *ddr_cmd_reg[2] = {
				(struct ddr_cmd_regs *)DDR_PHY_CMD_ADDR,
				(struct ddr_cmd_regs *)DDR_PHY_CMD_ADDR2};

static struct ddr_data_regs *ddr_data_reg[2] = {
				(struct ddr_data_regs *)DDR_PHY_DATA_ADDR,
				(struct ddr_data_regs *)DDR_PHY_DATA_ADDR2};	
				

static void config_vtp(int nr)
{
	writel(readl(&vtpreg[nr]->vtp0ctrlreg) | VTP_CTRL_ENABLE,
			&vtpreg[nr]->vtp0ctrlreg);
	writel(readl(&vtpreg[nr]->vtp0ctrlreg) & (~VTP_CTRL_START_EN),
			&vtpreg[nr]->vtp0ctrlreg);
	writel(readl(&vtpreg[nr]->vtp0ctrlreg) | VTP_CTRL_START_EN,
			&vtpreg[nr]->vtp0ctrlreg);

	/* Poll for READY */
	while ((readl(&vtpreg[nr]->vtp0ctrlreg) & VTP_CTRL_READY) !=
			VTP_CTRL_READY)
		;
}

/*********************** Add by MYiR ***********************************/

/**
 * This structure represents the DDR io control on AM33XX devices.
 */
struct ctrl_ioregs {
	unsigned int cm0ioctl;
	unsigned int cm1ioctl;
	unsigned int cm2ioctl;
	unsigned int dt0ioctl;
	unsigned int dt1ioctl;
	unsigned int dt2ioctrl;
	unsigned int dt3ioctrl;
	unsigned int emif_sdram_config_ext;
};


/**
 * Encapsulates DDR DATA registers.
 */
struct ddr_data {
	unsigned long datardsratio0;
	unsigned long datawdsratio0;
	unsigned long datawiratio0;
	unsigned long datagiratio0;
	unsigned long datafwsratio0;
	unsigned long datawrsratio0;
};

/**
 * Encapsulates DDR CMD control registers.
 */
struct cmd_control {
	unsigned long cmd0csratio;
	unsigned long cmd0csforce;
	unsigned long cmd0csdelay;
	unsigned long cmd0iclkout;
	unsigned long cmd1csratio;
	unsigned long cmd1csforce;
	unsigned long cmd1csdelay;
	unsigned long cmd1iclkout;
	unsigned long cmd2csratio;
	unsigned long cmd2csforce;
	unsigned long cmd2csdelay;
	unsigned long cmd2iclkout;
};

struct ddr_ctrl {
	unsigned int ddrioctrl;
	unsigned int resv1[325];
	unsigned int ddrckectrl;
};

struct ddr_cmd_regs {
	unsigned int resv0[7];
	unsigned int cm0csratio;	/* offset 0x01C */
	unsigned int resv1[3];
	unsigned int cm0iclkout;	/* offset 0x02C */
	unsigned int resv2[8];
	unsigned int cm1csratio;	/* offset 0x050 */
	unsigned int resv3[3];
	unsigned int cm1iclkout;	/* offset 0x060 */
	unsigned int resv4[8];
	unsigned int cm2csratio;	/* offset 0x084 */
	unsigned int resv5[3];
	unsigned int cm2iclkout;	/* offset 0x094 */
	unsigned int resv6[3];
};

struct ddr_data_regs {
	unsigned int dt0rdsratio0;	/* offset 0x0C8 */
	unsigned int resv1[4];
	unsigned int dt0wdsratio0;	/* offset 0x0DC */
	unsigned int resv2[4];
	unsigned int dt0wiratio0;	/* offset 0x0F0 */
	unsigned int resv3;
	unsigned int dt0wimode0;	/* offset 0x0F8 */
	unsigned int dt0giratio0;	/* offset 0x0FC */
	unsigned int resv4;
	unsigned int dt0gimode0;	/* offset 0x104 */
	unsigned int dt0fwsratio0;	/* offset 0x108 */
	unsigned int resv5[4];
	unsigned int dt0dqoffset;	/* offset 0x11C */
	unsigned int dt0wrsratio0;	/* offset 0x120 */
	unsigned int resv6[4];
	unsigned int dt0rdelays0;	/* offset 0x134 */
	unsigned int dt0dldiff0;	/* offset 0x138 */
	unsigned int resv7[12];
};

/*
 * Structure containing shadow of important registers in EMIF
 * The calculation function fills in this structure to be later used for
 * initialization and DVFS
 */
struct emif_regs {
	unsigned int freq;
	unsigned int sdram_config_init;
	unsigned int sdram_config;
	unsigned int sdram_config2;
	unsigned int ref_ctrl;
	unsigned int sdram_tim1;
	unsigned int sdram_tim2;
	unsigned int sdram_tim3;
	unsigned int read_idle_ctrl;
	unsigned int zq_config;
	unsigned int temp_alert_config;
	unsigned int emif_ddr_phy_ctlr_1_init;
	unsigned int emif_ddr_phy_ctlr_1;
	unsigned int emif_ddr_ext_phy_ctrl_1;
	unsigned int emif_ddr_ext_phy_ctrl_2;
	unsigned int emif_ddr_ext_phy_ctrl_3;
	unsigned int emif_ddr_ext_phy_ctrl_4;
	unsigned int emif_ddr_ext_phy_ctrl_5;
	unsigned int emif_rd_wr_lvl_rmp_win;
	unsigned int emif_rd_wr_lvl_rmp_ctl;
	unsigned int emif_rd_wr_lvl_ctl;
	unsigned int emif_rd_wr_exec_thresh;
	unsigned int emif_prio_class_serv_map;
	unsigned int emif_connect_id_serv_1_map;
	unsigned int emif_connect_id_serv_2_map;
	unsigned int emif_cos_config;
};

/**
 * This structure represents the DDR io control on AM33XX devices.
 */
struct ddr_cmdtctrl {
	unsigned int cm0ioctl;
	unsigned int cm1ioctl;
	unsigned int cm2ioctl;
	unsigned int resv2[12];
	unsigned int dt0ioctl;
	unsigned int dt1ioctl;
	unsigned int dt2ioctrl;
	unsigned int dt3ioctrl;
	unsigned int resv3[4];
	unsigned int emif_sdram_config_ext;
};

/* Reg mapping structure */
struct emif_reg_struct {
	u32 emif_mod_id_rev;
	u32 emif_status;
	u32 emif_sdram_config;
	u32 emif_lpddr2_nvm_config;
	u32 emif_sdram_ref_ctrl;
	u32 emif_sdram_ref_ctrl_shdw;
	u32 emif_sdram_tim_1;
	u32 emif_sdram_tim_1_shdw;
	u32 emif_sdram_tim_2;
	u32 emif_sdram_tim_2_shdw;
	u32 emif_sdram_tim_3;
	u32 emif_sdram_tim_3_shdw;
	u32 emif_lpddr2_nvm_tim;
	u32 emif_lpddr2_nvm_tim_shdw;
	u32 emif_pwr_mgmt_ctrl;
	u32 emif_pwr_mgmt_ctrl_shdw;
	u32 emif_lpddr2_mode_reg_data;
	u32 padding1[1];
	u32 emif_lpddr2_mode_reg_data_es2;
	u32 padding11[1];
	u32 emif_lpddr2_mode_reg_cfg;
	u32 emif_l3_config;
	u32 emif_l3_cfg_val_1;
	u32 emif_l3_cfg_val_2;
	u32 emif_iodft_tlgc;
	u32 padding2[7];
	u32 emif_perf_cnt_1;
	u32 emif_perf_cnt_2;
	u32 emif_perf_cnt_cfg;
	u32 emif_perf_cnt_sel;
	u32 emif_perf_cnt_tim;
	u32 padding3;
	u32 emif_read_idlectrl;
	u32 emif_read_idlectrl_shdw;
	u32 padding4;
	u32 emif_irqstatus_raw_sys;
	u32 emif_irqstatus_raw_ll;
	u32 emif_irqstatus_sys;
	u32 emif_irqstatus_ll;
	u32 emif_irqenable_set_sys;
	u32 emif_irqenable_set_ll;
	u32 emif_irqenable_clr_sys;
	u32 emif_irqenable_clr_ll;
	u32 padding5;
	u32 emif_zq_config;
	u32 emif_temp_alert_config;
	u32 emif_l3_err_log;
	u32 emif_rd_wr_lvl_rmp_win;
	u32 emif_rd_wr_lvl_rmp_ctl;
	u32 emif_rd_wr_lvl_ctl;
	u32 padding6[1];
	u32 emif_ddr_phy_ctrl_1;
	u32 emif_ddr_phy_ctrl_1_shdw;
	u32 emif_ddr_phy_ctrl_2;
	u32 padding7[4];
	u32 emif_prio_class_serv_map;
	u32 emif_connect_id_serv_1_map;
	u32 emif_connect_id_serv_2_map;
	u32 padding8[5];
	u32 emif_rd_wr_exec_thresh;
	u32 emif_cos_config;
	u32 padding9[6];
	u32 emif_ddr_phy_status[21];
	u32 padding10[27];
	u32 emif_ddr_ext_phy_ctrl_1;
	u32 emif_ddr_ext_phy_ctrl_1_shdw;
	u32 emif_ddr_ext_phy_ctrl_2;
	u32 emif_ddr_ext_phy_ctrl_2_shdw;
	u32 emif_ddr_ext_phy_ctrl_3;
	u32 emif_ddr_ext_phy_ctrl_3_shdw;
	u32 emif_ddr_ext_phy_ctrl_4;
	u32 emif_ddr_ext_phy_ctrl_4_shdw;
	u32 emif_ddr_ext_phy_ctrl_5;
	u32 emif_ddr_ext_phy_ctrl_5_shdw;
	u32 emif_ddr_ext_phy_ctrl_6;
	u32 emif_ddr_ext_phy_ctrl_6_shdw;
	u32 emif_ddr_ext_phy_ctrl_7;
	u32 emif_ddr_ext_phy_ctrl_7_shdw;
	u32 emif_ddr_ext_phy_ctrl_8;
	u32 emif_ddr_ext_phy_ctrl_8_shdw;
	u32 emif_ddr_ext_phy_ctrl_9;
	u32 emif_ddr_ext_phy_ctrl_9_shdw;
	u32 emif_ddr_ext_phy_ctrl_10;
	u32 emif_ddr_ext_phy_ctrl_10_shdw;
	u32 emif_ddr_ext_phy_ctrl_11;
	u32 emif_ddr_ext_phy_ctrl_11_shdw;
	u32 emif_ddr_ext_phy_ctrl_12;
	u32 emif_ddr_ext_phy_ctrl_12_shdw;
	u32 emif_ddr_ext_phy_ctrl_13;
	u32 emif_ddr_ext_phy_ctrl_13_shdw;
	u32 emif_ddr_ext_phy_ctrl_14;
	u32 emif_ddr_ext_phy_ctrl_14_shdw;
	u32 emif_ddr_ext_phy_ctrl_15;
	u32 emif_ddr_ext_phy_ctrl_15_shdw;
	u32 emif_ddr_ext_phy_ctrl_16;
	u32 emif_ddr_ext_phy_ctrl_16_shdw;
	u32 emif_ddr_ext_phy_ctrl_17;
	u32 emif_ddr_ext_phy_ctrl_17_shdw;
	u32 emif_ddr_ext_phy_ctrl_18;
	u32 emif_ddr_ext_phy_ctrl_18_shdw;
	u32 emif_ddr_ext_phy_ctrl_19;
	u32 emif_ddr_ext_phy_ctrl_19_shdw;
	u32 emif_ddr_ext_phy_ctrl_20;
	u32 emif_ddr_ext_phy_ctrl_20_shdw;
	u32 emif_ddr_ext_phy_ctrl_21;
	u32 emif_ddr_ext_phy_ctrl_21_shdw;
	u32 emif_ddr_ext_phy_ctrl_22;
	u32 emif_ddr_ext_phy_ctrl_22_shdw;
	u32 emif_ddr_ext_phy_ctrl_23;
	u32 emif_ddr_ext_phy_ctrl_23_shdw;
	u32 emif_ddr_ext_phy_ctrl_24;
	u32 emif_ddr_ext_phy_ctrl_24_shdw;
	u32 padding[22];
	u32 emif_ddr_fifo_misaligned_clear_1;
	u32 emif_ddr_fifo_misaligned_clear_2;
};

static struct ddr_ctrl *ddrctrl = (struct ddr_ctrl *)DDR_CTRL_ADDR;


const struct ctrl_ioregs ioregs_bonelt = {
	.cm0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm2ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
};

static const struct ddr_data ddr3_beagleblack_data = {
	.datardsratio0 = MT41K256M16HA125E_RD_DQS,
	.datawdsratio0 = MT41K256M16HA125E_WR_DQS,
	.datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
	.datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static const struct cmd_control ddr3_beagleblack_cmd_ctrl_data = {
	.cmd0csratio = MT41K256M16HA125E_RATIO,
	.cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd1csratio = MT41K256M16HA125E_RATIO,
	.cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd2csratio = MT41K256M16HA125E_RATIO,
	.cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};


static struct emif_regs ddr3_beagleblack_emif_reg_data = {
	.sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
	.ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
	.sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
	.sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
	.sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
	.zq_config = MT41K256M16HA125E_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};

/**
 * Base address for ddr io control instances
 */
static struct ddr_cmdtctrl *ioctrl_reg = {
			(struct ddr_cmdtctrl *)DDR_CONTROL_BASE_ADDR
};

/**
 * Configure DDR PHY
 */
static void config_ddr_phy(const struct emif_regs *regs, int nr)
{
	/*
	 * disable initialization and refreshes for now until we
	 * finish programming EMIF regs.
	 */
	setbits_le32(&emif_reg[nr]->emif_sdram_ref_ctrl, 
		EMIF_REG_INITREF_DIS_MASK);

	writel(regs->emif_ddr_phy_ctlr_1,
		&emif_reg[nr]->emif_ddr_phy_ctrl_1);
	writel(regs->emif_ddr_phy_ctlr_1,
		&emif_reg[nr]->emif_ddr_phy_ctrl_1_shdw);
}

/**
 * Configure DDR CMD control registers
 */
static void config_cmd_ctrl(const struct cmd_control *cmd, int nr)
{
	if (!cmd)
		return;

	writel(cmd->cmd0csratio, &ddr_cmd_reg[nr]->cm0csratio);
	writel(cmd->cmd0iclkout, &ddr_cmd_reg[nr]->cm0iclkout);

	writel(cmd->cmd1csratio, &ddr_cmd_reg[nr]->cm1csratio);
	writel(cmd->cmd1iclkout, &ddr_cmd_reg[nr]->cm1iclkout);

	writel(cmd->cmd2csratio, &ddr_cmd_reg[nr]->cm2csratio);
	writel(cmd->cmd2iclkout, &ddr_cmd_reg[nr]->cm2iclkout);
}

/**
 * Configure DDR DATA registers
 */
static void config_ddr_data(const struct ddr_data *data, int nr)
{
	int i;

	if (!data)
		return;

	for (i = 0; i < DDR_DATA_REGS_NR; i++) {
		writel(data->datardsratio0,
			&(ddr_data_reg[nr]+i)->dt0rdsratio0);
		writel(data->datawdsratio0,
			&(ddr_data_reg[nr]+i)->dt0wdsratio0);
		writel(data->datawiratio0,
			&(ddr_data_reg[nr]+i)->dt0wiratio0);
		writel(data->datagiratio0,
			&(ddr_data_reg[nr]+i)->dt0giratio0);
		writel(data->datafwsratio0,
			&(ddr_data_reg[nr]+i)->dt0fwsratio0);
		writel(data->datawrsratio0,
			&(ddr_data_reg[nr]+i)->dt0wrsratio0);
	}
}

static void config_io_ctrl(const struct ctrl_ioregs *ioregs)
{
	if (!ioregs)
		return;

	writel(ioregs->cm0ioctl, &ioctrl_reg->cm0ioctl);
	writel(ioregs->cm1ioctl, &ioctrl_reg->cm1ioctl);
	writel(ioregs->cm2ioctl, &ioctrl_reg->cm2ioctl);
	writel(ioregs->dt0ioctl, &ioctrl_reg->dt0ioctl);
	writel(ioregs->dt1ioctl, &ioctrl_reg->dt1ioctl);
#ifdef CONFIG_AM43XX
	writel(ioregs->dt2ioctrl, &ioctrl_reg->dt2ioctrl);
	writel(ioregs->dt3ioctrl, &ioctrl_reg->dt3ioctrl);
	writel(ioregs->emif_sdram_config_ext,
	       &ioctrl_reg->emif_sdram_config_ext);
#endif
}


/**
 * Set SDRAM timings
 */
static void set_sdram_timings(const struct emif_regs *regs, int nr)
{
	writel(regs->sdram_tim1, &emif_reg[nr]->emif_sdram_tim_1);
	writel(regs->sdram_tim1, &emif_reg[nr]->emif_sdram_tim_1_shdw);
	writel(regs->sdram_tim2, &emif_reg[nr]->emif_sdram_tim_2);
	writel(regs->sdram_tim2, &emif_reg[nr]->emif_sdram_tim_2_shdw);
	writel(regs->sdram_tim3, &emif_reg[nr]->emif_sdram_tim_3);
	writel(regs->sdram_tim3, &emif_reg[nr]->emif_sdram_tim_3_shdw);
}			

/* Control Status Register */
struct ctrl_stat {
	unsigned int resv1[16];
	unsigned int statusreg;		/* ofset 0x40 */
	unsigned int resv2[51];
	unsigned int secure_emif_sdram_config;	/* offset 0x0110 */
	unsigned int resv3[319];
	unsigned int dev_attr;
};

struct ctrl_stat *cstat = (struct ctrl_stat *)CTRL_BASE;

/**
 * Configure SDRAM
 */
static void config_sdram(const struct emif_regs *regs, int nr)
{
	if (regs->zq_config) {
		/*
		 * A value of 0x2800 for the REF CTRL will give us
		 * about 570us for a delay, which will be long enough
		 * to configure things.
		 */
		writel(0x2800, &emif_reg[nr]->emif_sdram_ref_ctrl);
		writel(regs->zq_config, &emif_reg[nr]->emif_zq_config);
		writel(regs->sdram_config, &cstat->secure_emif_sdram_config);
		writel(regs->sdram_config, &emif_reg[nr]->emif_sdram_config);
		writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl);
		writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl_shdw);
	}
	writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl);
	writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl_shdw);
	writel(regs->sdram_config, &emif_reg[nr]->emif_sdram_config);
}


void config_ddr(unsigned int pll, const struct ctrl_ioregs *ioregs,
		const struct ddr_data *data, const struct cmd_control *ctrl,
		const struct emif_regs *regs, int nr)
{
	//ddr_pll_config(pll);
//#ifndef CONFIG_TI816X
	config_vtp(nr);
//#endif
	config_cmd_ctrl(ctrl, nr);

	config_ddr_data(data, nr);
	
//#ifdef CONFIG_AM33XX
	config_io_ctrl(ioregs);

	/* Set CKE to be controlled by EMIF/DDR PHY */
	writel(DDR_CKE_CTRL_NORMAL, &ddrctrl->ddrckectrl);
//#endif

	/* Program EMIF instance */
	config_ddr_phy(regs, nr);
	set_sdram_timings(regs, nr);
	config_sdram(regs, nr);
}


static void config_am335x_ddr(void)
{

	config_ddr(400, &ioregs_bonelt,
		   &ddr3_beagleblack_data,
		   &ddr3_beagleblack_cmd_ctrl_data,
		   &ddr3_beagleblack_emif_reg_data, 0);
}

/*
* clock 
*/
/*
 * Encapsulating peripheral functional clocks
 * pll registers
 */
/* Encapsulating core pll registers */
struct cm_wkuppll {
	unsigned int wkclkstctrl;	/* offset 0x00 */
	unsigned int wkctrlclkctrl; /* offset 0x04 */
	unsigned int wkgpio0clkctrl;	/* offset 0x08 */
	unsigned int wkl4wkclkctrl; /* offset 0x0c */
	unsigned int timer0clkctrl; /* offset 0x10 */
	unsigned int resv2[3];
	unsigned int idlestdpllmpu; /* offset 0x20 */
	unsigned int resv3[2];
	unsigned int clkseldpllmpu; /* offset 0x2c */
	unsigned int resv4[1];
	unsigned int idlestdpllddr; /* offset 0x34 */
	unsigned int resv5[2];
	unsigned int clkseldpllddr; /* offset 0x40 */
	unsigned int resv6[4];
	unsigned int clkseldplldisp;	/* offset 0x54 */
	unsigned int resv7[1];
	unsigned int idlestdpllcore;	/* offset 0x5c */
	unsigned int resv8[2];
	unsigned int clkseldpllcore;	/* offset 0x68 */
	unsigned int resv9[1];
	unsigned int idlestdpllper; /* offset 0x70 */
	unsigned int resv10[2];
	unsigned int clkdcoldodpllper;	/* offset 0x7c */
	unsigned int divm4dpllcore; /* offset 0x80 */
	unsigned int divm5dpllcore; /* offset 0x84 */
	unsigned int clkmoddpllmpu; /* offset 0x88 */
	unsigned int clkmoddpllper; /* offset 0x8c */
	unsigned int clkmoddpllcore;	/* offset 0x90 */
	unsigned int clkmoddpllddr; /* offset 0x94 */
	unsigned int clkmoddplldisp;	/* offset 0x98 */
	unsigned int clkseldpllper; /* offset 0x9c */
	unsigned int divm2dpllddr;	/* offset 0xA0 */
	unsigned int divm2dplldisp; /* offset 0xA4 */
	unsigned int divm2dpllmpu;	/* offset 0xA8 */
	unsigned int divm2dpllper;	/* offset 0xAC */
	unsigned int resv11[1];
	unsigned int wkup_uart0ctrl;	/* offset 0xB4 */
	unsigned int wkup_i2c0ctrl; /* offset 0xB8 */
	unsigned int wkup_adctscctrl;	/* offset 0xBC */
	unsigned int resv12;
	unsigned int timer1clkctrl; /* offset 0xC4 */
	unsigned int resv13[4];
	unsigned int divm6dpllcore; /* offset 0xD8 */
};

/**
 * Encapsulating peripheral functional clocks
 * pll registers
 */
struct cm_perpll {
	unsigned int l4lsclkstctrl; /* offset 0x00 */
	unsigned int l3sclkstctrl;	/* offset 0x04 */
	unsigned int l4fwclkstctrl; /* offset 0x08 */
	unsigned int l3clkstctrl;	/* offset 0x0c */
	unsigned int resv1;
	unsigned int cpgmac0clkctrl;	/* offset 0x14 */
	unsigned int lcdclkctrl;	/* offset 0x18 */
	unsigned int usb0clkctrl;	/* offset 0x1C */
	unsigned int resv2;
	unsigned int tptc0clkctrl;	/* offset 0x24 */
	unsigned int emifclkctrl;	/* offset 0x28 */
	unsigned int ocmcramclkctrl;	/* offset 0x2c */
	unsigned int gpmcclkctrl;	/* offset 0x30 */
	unsigned int mcasp0clkctrl; /* offset 0x34 */
	unsigned int uart5clkctrl;	/* offset 0x38 */
	unsigned int mmc0clkctrl;	/* offset 0x3C */
	unsigned int elmclkctrl;	/* offset 0x40 */
	unsigned int i2c2clkctrl;	/* offset 0x44 */
	unsigned int i2c1clkctrl;	/* offset 0x48 */
	unsigned int spi0clkctrl;	/* offset 0x4C */
	unsigned int spi1clkctrl;	/* offset 0x50 */
	unsigned int resv3[3];
	unsigned int l4lsclkctrl;	/* offset 0x60 */
	unsigned int l4fwclkctrl;	/* offset 0x64 */
	unsigned int mcasp1clkctrl; /* offset 0x68 */
	unsigned int uart1clkctrl;	/* offset 0x6C */
	unsigned int uart2clkctrl;	/* offset 0x70 */
	unsigned int uart3clkctrl;	/* offset 0x74 */
	unsigned int uart4clkctrl;	/* offset 0x78 */
	unsigned int timer7clkctrl; /* offset 0x7C */
	unsigned int timer2clkctrl; /* offset 0x80 */
	unsigned int timer3clkctrl; /* offset 0x84 */
	unsigned int timer4clkctrl; /* offset 0x88 */
	unsigned int resv4[8];
	unsigned int gpio1clkctrl;	/* offset 0xAC */
	unsigned int gpio2clkctrl;	/* offset 0xB0 */
	unsigned int gpio3clkctrl;	/* offset 0xB4 */
	unsigned int resv5;
	unsigned int tpccclkctrl;	/* offset 0xBC */
	unsigned int dcan0clkctrl;	/* offset 0xC0 */
	unsigned int dcan1clkctrl;	/* offset 0xC4 */
	unsigned int resv6;
	unsigned int epwmss1clkctrl;	/* offset 0xCC */
	unsigned int emiffwclkctrl; /* offset 0xD0 */
	unsigned int epwmss0clkctrl;	/* offset 0xD4 */
	unsigned int epwmss2clkctrl;	/* offset 0xD8 */
	unsigned int l3instrclkctrl;	/* offset 0xDC */
	unsigned int l3clkctrl; 	/* Offset 0xE0 */
	unsigned int resv8[2];
	unsigned int timer5clkctrl; /* offset 0xEC */
	unsigned int timer6clkctrl; /* offset 0xF0 */
	unsigned int mmc1clkctrl;	/* offset 0xF4 */
	unsigned int mmc2clkctrl;	/* offset 0xF8 */
	unsigned int resv9[8];
	unsigned int l4hsclkstctrl; /* offset 0x11C */
	unsigned int l4hsclkctrl;	/* offset 0x120 */
	unsigned int resv10[8];
	unsigned int cpswclkstctrl; /* offset 0x144 */
	unsigned int lcdcclkstctrl; /* offset 0x148 */
};

/* Encapsulating Display pll registers */
struct cm_dpll {
	unsigned int resv1;
	unsigned int clktimer7clk;	/* offset 0x04 */
	unsigned int clktimer2clk;	/* offset 0x08 */
	unsigned int clktimer3clk;	/* offset 0x0C */
	unsigned int clktimer4clk;	/* offset 0x10 */
	unsigned int resv2;
	unsigned int clktimer5clk;	/* offset 0x18 */
	unsigned int clktimer6clk;	/* offset 0x1C */
	unsigned int resv3[2];
	unsigned int clktimer1clk;	/* offset 0x28 */
	unsigned int resv4[2];
	unsigned int clklcdcpixelclk;	/* offset 0x34 */
};


/* Control Module RTC registers */
struct cm_rtc {
	unsigned int rtcclkctrl;	/* offset 0x0 */
	unsigned int clkstctrl;		/* offset 0x4 */
};

#define CM_RTC				0x44E00800


struct cm_perpll *const cmper = (struct cm_perpll *)CM_PER;
struct cm_wkuppll *const cmwkup = (struct cm_wkuppll *)CM_WKUP;
struct cm_dpll *const cmdpll = (struct cm_dpll *)CM_DPLL;
struct cm_rtc *const cmrtc = (struct cm_rtc *)CM_RTC;


#define LDELAY 1000000

/*CM_<clock_domain>__CLKCTRL */
#define CD_CLKCTRL_CLKTRCTRL_SHIFT		0
#define CD_CLKCTRL_CLKTRCTRL_MASK		3

#define CD_CLKCTRL_CLKTRCTRL_NO_SLEEP		0
#define CD_CLKCTRL_CLKTRCTRL_SW_SLEEP		1
#define CD_CLKCTRL_CLKTRCTRL_SW_WKUP		2

/* CM_<clock_domain>_<module>_CLKCTRL */
#define MODULE_CLKCTRL_MODULEMODE_SHIFT		0
#define MODULE_CLKCTRL_MODULEMODE_MASK		3
#define MODULE_CLKCTRL_IDLEST_SHIFT		16
#define MODULE_CLKCTRL_IDLEST_MASK		(3 << 16)

#define MODULE_CLKCTRL_MODULEMODE_SW_DISABLE		0
#define MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN	2

#define MODULE_CLKCTRL_IDLEST_FULLY_FUNCTIONAL	0
#define MODULE_CLKCTRL_IDLEST_TRANSITIONING	1
#define MODULE_CLKCTRL_IDLEST_IDLE		2
#define MODULE_CLKCTRL_IDLEST_DISABLED		3

/* CM_CLKMODE_DPLL */
#define CM_CLKMODE_DPLL_SSC_EN_SHIFT		12
#define CM_CLKMODE_DPLL_SSC_EN_MASK		(1 << 12)
#define CM_CLKMODE_DPLL_REGM4XEN_SHIFT		11
#define CM_CLKMODE_DPLL_REGM4XEN_MASK		(1 << 11)
#define CM_CLKMODE_DPLL_LPMODE_EN_SHIFT		10
#define CM_CLKMODE_DPLL_LPMODE_EN_MASK		(1 << 10)
#define CM_CLKMODE_DPLL_RELOCK_RAMP_EN_SHIFT	9
#define CM_CLKMODE_DPLL_RELOCK_RAMP_EN_MASK	(1 << 9)
#define CM_CLKMODE_DPLL_DRIFTGUARD_EN_SHIFT	8
#define CM_CLKMODE_DPLL_DRIFTGUARD_EN_MASK	(1 << 8)
#define CM_CLKMODE_DPLL_RAMP_RATE_SHIFT		5
#define CM_CLKMODE_DPLL_RAMP_RATE_MASK		(0x7 << 5)
#define CM_CLKMODE_DPLL_EN_SHIFT		0
#define CM_CLKMODE_DPLL_EN_MASK			(0x7 << 0)

#define CM_CLKMODE_DPLL_DPLL_EN_SHIFT		0
#define CM_CLKMODE_DPLL_DPLL_EN_MASK		7

#define DPLL_EN_STOP			1
#define DPLL_EN_MN_BYPASS		4
#define DPLL_EN_LOW_POWER_BYPASS	5
#define DPLL_EN_LOCK			7

/* CM_IDLEST_DPLL fields */
#define ST_DPLL_CLK_MASK		1

/* CM_CLKSEL_DPLL */
#define CM_CLKSEL_DPLL_M_SHIFT			8
#define CM_CLKSEL_DPLL_M_MASK			(0x7FF << 8)
#define CM_CLKSEL_DPLL_N_SHIFT			0
#define CM_CLKSEL_DPLL_N_MASK			0x7F


static inline void wait_for_clk_enable(u32 *clkctrl_addr)
{
	u32 clkctrl, idlest = MODULE_CLKCTRL_IDLEST_DISABLED;
	u32 bound = LDELAY;

	while ((idlest == MODULE_CLKCTRL_IDLEST_DISABLED) ||
		(idlest == MODULE_CLKCTRL_IDLEST_TRANSITIONING)) {
		clkctrl = readl(clkctrl_addr);
		idlest = (clkctrl & MODULE_CLKCTRL_IDLEST_MASK) >>
			 MODULE_CLKCTRL_IDLEST_SHIFT;
		if (--bound == 0) {
			printf("Clock enable failed for 0x%p idlest 0x%x\n",
			       clkctrl_addr, clkctrl);
			return;
		}
	}
}


static inline void enable_clock_domain(u32 *const clkctrl_reg, u32 enable_mode)
{
	clrsetbits_le32(clkctrl_reg, CD_CLKCTRL_CLKTRCTRL_MASK,
			enable_mode << CD_CLKCTRL_CLKTRCTRL_SHIFT);
	debug("Enable clock domain - %p\n", clkctrl_reg);
}

static inline void enable_clock_module(u32 *const clkctrl_addr, u32 enable_mode,
				       u32 wait_for_enable)
{
	clrsetbits_le32(clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_MASK,
			enable_mode << MODULE_CLKCTRL_MODULEMODE_SHIFT);
	debug("Enable clock module - %p\n", clkctrl_addr);
	if (wait_for_enable)
		wait_for_clk_enable(clkctrl_addr);
}

static void do_enable_clocks(u32 *const *clk_domains,
		      u32 *const *clk_modules_explicit_en, u8 wait_for_enable)
{
	u32 i, max = 100;

	/* Put the clock domains in SW_WKUP mode */
	for (i = 0; (i < max) && clk_domains[i]; i++) {
		enable_clock_domain(clk_domains[i],
				    CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	}

	/* Clock modules that need to be put in SW_EXPLICIT_EN mode */
	for (i = 0; (i < max) && clk_modules_explicit_en[i]; i++) {
		enable_clock_module(clk_modules_explicit_en[i],
				    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN,
				    wait_for_enable);
	};
}


static void enable_basic_clocks(void)
{
	u32 *const clk_domains[] = {
		&cmper->l3clkstctrl,
		&cmper->l4fwclkstctrl,
		&cmper->l3sclkstctrl,
		&cmper->l4lsclkstctrl,
		&cmwkup->wkclkstctrl,
		&cmper->emiffwclkctrl,
		&cmrtc->clkstctrl,
		0
	};

	u32 *const clk_modules_explicit_en[] = {
		&cmper->l3clkctrl,
		&cmper->l4lsclkctrl,
		&cmper->l4fwclkctrl,
		&cmwkup->wkl4wkclkctrl,
		&cmper->l3instrclkctrl,
		&cmper->l4hsclkctrl,
		&cmwkup->wkgpio0clkctrl,
		&cmwkup->wkctrlclkctrl,
		&cmper->timer2clkctrl,
		&cmper->gpmcclkctrl,
		&cmper->elmclkctrl,
		&cmper->mmc0clkctrl,
		&cmper->mmc1clkctrl,
		&cmwkup->wkup_i2c0ctrl,
		&cmper->gpio1clkctrl,
		&cmper->gpio2clkctrl,
		&cmper->gpio3clkctrl,
		&cmper->i2c1clkctrl,
		&cmper->cpgmac0clkctrl,
		&cmper->spi0clkctrl,
		&cmrtc->rtcclkctrl,
		&cmper->usb0clkctrl,
		&cmper->emiffwclkctrl,
		&cmper->emifclkctrl,
		0
	};

	do_enable_clocks(clk_domains, clk_modules_explicit_en, 1);

	/* Select the Master osc 24 MHZ as Timer2 clock source */
	writel(0x1, &cmdpll->clktimer2clk);
}

struct dpll_params {
	u32 m;
	u32 n;
	s8 m2;
	s8 m3;
	s8 m4;
	s8 m5;
	s8 m6;
};

#define V_OSCK2			24000000  /* Clock output from T2 */
#define OSC2			(V_OSCK2/1000000)
#define MPUPLL_M_300	300

const struct dpll_params dpll_mpu = {
		MPUPLL_M_300, OSC2-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_core = {
		50, OSC2-1, -1, -1, 1, 1, 1};
const struct dpll_params dpll_per = {
		960, OSC2-1, 5, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_bone_black = {
		400, OSC2-1, 1, -1, -1, -1, -1};		

struct dpll_regs {
	u32 cm_clkmode_dpll;
	u32 cm_idlest_dpll;
	u32 cm_autoidle_dpll;
	u32 cm_clksel_dpll;
	u32 cm_div_m2_dpll;
	u32 cm_div_m3_dpll;
	u32 cm_div_m4_dpll;
	u32 cm_div_m5_dpll;
	u32 cm_div_m6_dpll;
};

const struct dpll_regs dpll_mpu_regs = {
	.cm_clkmode_dpll	= CM_WKUP + 0x88,
	.cm_idlest_dpll		= CM_WKUP + 0x20,
	.cm_clksel_dpll		= CM_WKUP + 0x2C,
	.cm_div_m2_dpll		= CM_WKUP + 0xA8,
};

const struct dpll_regs dpll_core_regs = {
	.cm_clkmode_dpll	= CM_WKUP + 0x90,
	.cm_idlest_dpll		= CM_WKUP + 0x5C,
	.cm_clksel_dpll		= CM_WKUP + 0x68,
	.cm_div_m4_dpll		= CM_WKUP + 0x80,
	.cm_div_m5_dpll		= CM_WKUP + 0x84,
	.cm_div_m6_dpll		= CM_WKUP + 0xD8,
};

const struct dpll_regs dpll_per_regs = {
	.cm_clkmode_dpll	= CM_WKUP + 0x8C,
	.cm_idlest_dpll		= CM_WKUP + 0x70,
	.cm_clksel_dpll		= CM_WKUP + 0x9C,
	.cm_div_m2_dpll		= CM_WKUP + 0xAC,
};


const struct dpll_regs dpll_ddr_regs = {
	.cm_clkmode_dpll	= CM_WKUP + 0x94,
	.cm_idlest_dpll		= CM_WKUP + 0x34,
	.cm_clksel_dpll		= CM_WKUP + 0x40,
	.cm_div_m2_dpll		= CM_WKUP + 0xA0,
};


static void setup_post_dividers(const struct dpll_regs *dpll_regs,
			 const struct dpll_params *params)
{
	/* Setup post-dividers */
	if (params->m2 >= 0)
		writel(params->m2, dpll_regs->cm_div_m2_dpll);
	if (params->m3 >= 0)
		writel(params->m3, dpll_regs->cm_div_m3_dpll);
	if (params->m4 >= 0)
		writel(params->m4, dpll_regs->cm_div_m4_dpll);
	if (params->m5 >= 0)
		writel(params->m5, dpll_regs->cm_div_m5_dpll);
	if (params->m6 >= 0)
		writel(params->m6, dpll_regs->cm_div_m6_dpll);
}

static inline void do_lock_dpll(const struct dpll_regs *dpll_regs)
{
	clrsetbits_le32(dpll_regs->cm_clkmode_dpll,
			CM_CLKMODE_DPLL_DPLL_EN_MASK,
			DPLL_EN_LOCK << CM_CLKMODE_DPLL_EN_SHIFT);
}

static inline void wait_for_lock(const struct dpll_regs *dpll_regs)
{
	if (!wait_on_value(ST_DPLL_CLK_MASK, ST_DPLL_CLK_MASK,
			   (void *)dpll_regs->cm_idlest_dpll, LDELAY)) {
		printf("DPLL locking failed for 0x%x\n",
		       dpll_regs->cm_clkmode_dpll);
		hang();
	}
}

static inline void do_bypass_dpll(const struct dpll_regs *dpll_regs)
{
	clrsetbits_le32(dpll_regs->cm_clkmode_dpll,
			CM_CLKMODE_DPLL_DPLL_EN_MASK,
			DPLL_EN_MN_BYPASS << CM_CLKMODE_DPLL_EN_SHIFT);
}

static inline void wait_for_bypass(const struct dpll_regs *dpll_regs)
{
	if (!wait_on_value(ST_DPLL_CLK_MASK, 0,
			   (void *)dpll_regs->cm_idlest_dpll, LDELAY)) {
		printf("Bypassing DPLL failed 0x%x\n",
		       dpll_regs->cm_clkmode_dpll);
	}
}

static void bypass_dpll(const struct dpll_regs *dpll_regs)
{
	do_bypass_dpll(dpll_regs);
	wait_for_bypass(dpll_regs);
}

void do_setup_dpll(const struct dpll_regs *dpll_regs,
		   const struct dpll_params *params)
{
	u32 temp;

	if (!params)
		return;

	temp = readl(dpll_regs->cm_clksel_dpll);

	bypass_dpll(dpll_regs);

	/* Set M & N */
	temp &= ~CM_CLKSEL_DPLL_M_MASK;
	temp |= (params->m << CM_CLKSEL_DPLL_M_SHIFT) & CM_CLKSEL_DPLL_M_MASK;

	temp &= ~CM_CLKSEL_DPLL_N_MASK;
	temp |= (params->n << CM_CLKSEL_DPLL_N_SHIFT) & CM_CLKSEL_DPLL_N_MASK;

	writel(temp, dpll_regs->cm_clksel_dpll);

	setup_post_dividers(dpll_regs, params);

	/* Wait till the DPLL locks */
	do_lock_dpll(dpll_regs);
	wait_for_lock(dpll_regs);
}


static void setup_dplls(void)
{
	const struct dpll_params *params;
	/*
	params = &dpll_core;
	do_setup_dpll(&dpll_core_regs, params);

	params = &dpll_mpu;
	do_setup_dpll(&dpll_mpu_regs, params);

	params = &dpll_per;
	do_setup_dpll(&dpll_per_regs, params);
	writel(0x300, &cmwkup->clkdcoldodpllper);
	*/
	
	params = &dpll_ddr_bone_black;
	do_setup_dpll(&dpll_ddr_regs, params);
}


void prcm_init2(void)
{
	enable_basic_clocks();
	setup_dplls();
}
			
static void config_emif_ddr3(void)
{
	u32 i;

	/*Program EMIF0 CFG Registers*/
	__raw_writel(EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1);
	__raw_writel(EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_1_SHADOW);
	__raw_writel(EMIF_READ_LATENCY, EMIF4_0_DDR_PHY_CTRL_2);
	__raw_writel(EMIF_TIM1, EMIF4_0_SDRAM_TIM_1);
	__raw_writel(EMIF_TIM1, EMIF4_0_SDRAM_TIM_1_SHADOW);
	__raw_writel(EMIF_TIM2, EMIF4_0_SDRAM_TIM_2);
	__raw_writel(EMIF_TIM2, EMIF4_0_SDRAM_TIM_2_SHADOW);
	__raw_writel(EMIF_TIM3, EMIF4_0_SDRAM_TIM_3);
	__raw_writel(EMIF_TIM3, EMIF4_0_SDRAM_TIM_3_SHADOW);

	__raw_writel(EMIF_SDCFG, EMIF4_0_SDRAM_CONFIG);
	__raw_writel(EMIF_SDCFG, EMIF4_0_SDRAM_CONFIG2);

	/* __raw_writel(EMIF_SDMGT, EMIF0_0_SDRAM_MGMT_CTRL);
	__raw_writel(EMIF_SDMGT, EMIF0_0_SDRAM_MGMT_CTRL_SHD); */
	__raw_writel(0x00004650, EMIF4_0_SDRAM_REF_CTRL);
	__raw_writel(0x00004650, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

	for (i = 0; i < 5000; i++) {

	}

	/* __raw_writel(EMIF_SDMGT, EMIF0_0_SDRAM_MGMT_CTRL);
	__raw_writel(EMIF_SDMGT, EMIF0_0_SDRAM_MGMT_CTRL_SHD); */
	__raw_writel(EMIF_SDREF, EMIF4_0_SDRAM_REF_CTRL);
	__raw_writel(EMIF_SDREF, EMIF4_0_SDRAM_REF_CTRL_SHADOW);

	__raw_writel(EMIF_SDCFG, EMIF4_0_SDRAM_CONFIG);
	__raw_writel(EMIF_SDCFG, EMIF4_0_SDRAM_CONFIG2);
}

static void init_timer(void)
{
	/* Reset the Timer */
	__raw_writel(0x2, (DM_TIMER2_BASE + TSICR_REG));

	/* Wait until the reset is done */
	while (__raw_readl(DM_TIMER2_BASE + TIOCP_CFG_REG) & 1);

	/* Start the Timer */
	__raw_writel(0x1, (DM_TIMER2_BASE + TCLR_REG));
}
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_BOARD_INIT)
/* MYIR , only applicable in PG2.x. */
unsigned int get_mpu_maxfreq(void)
{
#define CONTROL_MODULE_BASE 0x44E10000
#define EFUSE_SMA    		(CONTROL_MODULE_BASE + 0x7FC)
#define MPU_FREQ_MASK		0x1FFF
	#define	MPU_MAX_FREQ_300M	0x1FEF
	#define MPU_MAX_FREQ_600M	0x1FAF
	#define MPU_MAX_FREQ_720M	0x1F2F
	#define MPU_MAX_FREQ_800M	0x1E2F
	#define MPU_MAX_FREQ_1000M	0x1C2F
    #define MPU_MAX_FREQ_300M_ZCE   0x1FDF
    #define MPU_MAX_FREQ_600M_ZCE   0x1F9F

	unsigned int reg = __raw_readl(EFUSE_SMA);
	
	printf("EFUSE_SMA: 0x%08X, max freq reg: %#X\n", reg, reg&MPU_FREQ_MASK);

	switch (reg&MPU_FREQ_MASK) {
		case MPU_MAX_FREQ_300M:
		case MPU_MAX_FREQ_300M_ZCE:
			return MPUPLL_M_300;
		case MPU_MAX_FREQ_600M:
		case MPU_MAX_FREQ_600M_ZCE:
			return MPUPLL_M_600;
		case MPU_MAX_FREQ_720M:
			return MPUPLL_M_720;
		case MPU_MAX_FREQ_800M:
			return MPUPLL_M_800;
		case MPU_MAX_FREQ_1000M:
			return MPUPLL_M_1000;
		default:
			return MPUPLL_M_800;
	}
}

/* Added by MYIR for tps65217 */
int myir_pmic_init(void)
{
	/* BeagleBone PMIC Code */
	int usb_cur_lim;
	int mpu_vdd;
	unsigned int mpu_freq = get_mpu_maxfreq();
	
	printf("Set MPU freq to %d MHz\n", mpu_freq);
	switch (mpu_freq) {
		case MPUPLL_M_300:
		case MPUPLL_M_600:
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1125MV;
			break;
		case MPUPLL_M_720:
		case MPUPLL_M_800:
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1275MV;
			break;
		case MPUPLL_M_1000:
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1325MV;
			break;
		default:
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1275MV;
			break;
	}

	/* Configure the i2c0 pin mux */
	enable_i2c0_pin_mux();

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	
	if (i2c_probe(TPS65217_CHIP_PM))
		return;

	/*
	 * Increase USB current limit to 1300mA or 1800mA and set
	 * the MPU voltage controller as needed.
	 */
	usb_cur_lim = TPS65217_USB_INPUT_CUR_LIMIT_1300MA;
		
	if (tps65217_reg_write(TPS65217_PROT_LEVEL_NONE,
				   TPS65217_POWER_PATH,
				   usb_cur_lim,
				   TPS65217_USB_INPUT_CUR_LIMIT_MASK))
		puts("tps65217_reg_write failure\n");

	/* Set DCDC3 (CORE) voltage to 1.125V */
	if (tps65217_voltage_update(TPS65217_DEFDCDC3,
					TPS65217_DCDC_VOLT_SEL_1125MV)) {
		puts("tps65217_voltage_update failure\n");
		return;
	}

	/* Set DCDC2 (MPU) voltage */
	if (tps65217_voltage_update(TPS65217_DEFDCDC2, mpu_vdd)) {
		puts("tps65217_voltage_update failure\n");
		return;
	}

	/*
	 * Set LDO3 to 1.8V and LDO4 to 3.3V.
	 */
	if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
				   TPS65217_DEFLS1,
				   TPS65217_LDO_VOLTAGE_OUT_1_8,
				   TPS65217_LDO_MASK))
		puts("tps65217_reg_write failure\n");

	if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
				   TPS65217_DEFLS2,
				   TPS65217_LDO_VOLTAGE_OUT_3_3,
				   TPS65217_LDO_MASK))
		puts("tps65217_reg_write failure\n");
		
	mpu_pll_config(mpu_freq);
}

void spl_board_init(void)
{
/* Comment by Conway */
	myir_pmic_init();
}
#endif

/* 
 * Added by MYIR, turn off lcd backlight by setting GPIO0_2 output LOW.
 */
extern void enable_backlight_pin_mux(void);
extern void enable_wdt_pin_mux(void);
static void myir_init_backlight()
{
#define SOC_PRCM_REGS                        (0x44E00000)
#define SOC_CM_WKUP_REGS                     (SOC_PRCM_REGS + 0x400)
#define CM_WKUP_GPIO0_CLKCTRL   (0x8)
#define CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_WKUP_GPIO0_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK   (0x00040000u)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_WKUP_CONTROL_CLKCTRL   (0x4)
#define CM_WKUP_CONTROL_CLKCTRL_IDLEST   (0x00030000u)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK   (0x00000008u)
#define CM_WKUP_CM_L3_AON_CLKSTCTRL   (0x18)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_WKUP_L4WKUP_CLKCTRL   (0xc)
#define CM_WKUP_L4WKUP_CLKCTRL_IDLEST   (0x00030000u)
#define CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK   (0x00000004u)
#define CM_WKUP_CLKSTCTRL   (0x0)
#define CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK   (0x00000004u)
#define CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL   (0xcc)
#define CM_WKUP_GPIO0_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_WKUP_GPIO0_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_WKUP_GPIO0_CLKCTRL   (0x8)
#define CM_WKUP_GPIO0_CLKCTRL_IDLEST   (0x00030000u)
#define CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK   (0x00000100u)

#define GPIO0_BASE              0x44E07000
#define GPIO0_OE                (GPIO0_BASE + 0x134)
#define GPIO0_DATAOUT           (GPIO0_BASE + 0x13c)
#define GPIO0_SETDATAOUT        (GPIO0_BASE + 0x194)
#define GPIO0_CLEARDATAOUT      (GPIO0_BASE + 0x190)
#define BL_BIT                 (1 << 2)

    /* Writing to MODULEMODE field of CM_WKUP_GPIO0_CLKCTRL register. */
    __raw_writel(__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE, SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL);

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_MODULEMODE));

    /*
    ** Writing to OPTFCLKEN_GPIO0_GDBCLK field of CM_WKUP_GPIO0_CLKCTRL
    ** register.
    */
    __raw_writel(__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK, SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL);

    /* Waiting for OPTFCLKEN_GPIO0_GDBCLK field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));


    /* Writing to IDLEST field in CM_WKUP_GPIO0_CLKCTRL register. */
    while((CM_WKUP_GPIO0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_GPIO0_CLKCTRL_IDLEST_SHIFT) !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO0_GDBCLK field in CM_WKUP_GPIO0_CLKCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK !=
          (__raw_readl(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK));



	enable_backlight_pin_mux();
	__raw_writel(__raw_readl(GPIO0_OE) & ~BL_BIT, GPIO0_OE);
	__raw_writel(BL_BIT, GPIO0_CLEARDATAOUT);

}

/*
 * early system init of muxing and clocks.
 */
void s_init(void)
{
	/* Can be removed as A8 comes up with L2 enabled */
	l2_cache_enable();

	/* WDT1 is already running when the bootloader gets control
	 * Disable it to avoid "random" resets
	 */
	__raw_writel(0xAAAA, WDT_WSPR);
	while(__raw_readl(WDT_WWPS) != 0x0);
	__raw_writel(0x5555, WDT_WSPR);
	while(__raw_readl(WDT_WWPS) != 0x0);

	/* Added by MYIR, init LCD backlight(turn_off) */
	myir_init_backlight();
	enable_wdt_pin_mux();
	enable_e2pwp_pin_mux();

#ifdef CONFIG_SPL_BUILD
	/* Setup the PLLs and the clocks for the peripherals */
	pll_init();

	/* UART softreset */
	u32 regVal;
	u32 uart_base = DEFAULT_UART_BASE;

	enable_uart0_pin_mux();
	/* IA Motor Control Board has default console on UART3*/
	/* XXX: This is before we've probed / set board_id */
	if (board_id == IA_BOARD) {
		uart_base = UART3_BASE;
	}

	regVal = __raw_readl(uart_base + UART_SYSCFG_OFFSET);
	regVal |= UART_RESET;
	__raw_writel(regVal, (uart_base + UART_SYSCFG_OFFSET) );
	while ((__raw_readl(uart_base + UART_SYSSTS_OFFSET) &
			UART_CLK_RUNNING_MASK) != UART_CLK_RUNNING_MASK);

	/* Disable smart idle */
	regVal = __raw_readl((uart_base + UART_SYSCFG_OFFSET));
	regVal |= UART_SMART_IDLE_EN;
	__raw_writel(regVal, (uart_base + UART_SYSCFG_OFFSET));

	/* Initialize the Timer */
	init_timer();

	preloader_console_init();

	prcm_init2();
	config_am335x_ddr();

#endif
}

static unsigned char profile = PROFILE_0;

/*
 * Basic board specific setup
 */
#ifndef CONFIG_SPL_BUILD
int board_evm_init(void)
{
	/* mach type passed to kernel */
	if (board_id == IA_BOARD)
		gd->bd->bi_arch_number = MACH_TYPE_TIAM335IAEVM;
	else
		gd->bd->bi_arch_number = MACH_TYPE_TIAM335EVM;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_DRAM_1 + 0x100;

	return 0;
}
#endif

/*
 * LCD type -- Conway
 */
const unsigned char *lcd_type(void)
{
	static myir_header_t header;

	memset(&header, '\0', sizeof(myir_header_t));
	
	if (get_header(&header))
		return NULL;	
	return get_header_subtype(&header);
}
	
/*
 * LCD identify -- Conway
 */
void lcd_identify(void)
{
	char *env_optargs = getenv("optargs");
	char *tmp = "";
	if (!env_optargs)
		env_optargs = tmp;
	
	int  optargs_len = strlen(env_optargs);
	char *display = "board-am335xevm.display_mode=";
	int  display_len = strlen(display);
	char mode[14] = { '\0' };
	unsigned char *type = NULL;
	char *new_optargs = malloc(optargs_len + display_len + 16);
	if (!new_optargs) {
		printf("Alloc memory failed\n");
		return;
	}
	
	memset(new_optargs, '\0', optargs_len + display_len + 5);

	int idx = 0;
	int mode_idx = 0;
	while (idx < optargs_len) {
		if (env_optargs[idx] == ' ' || idx == 0) {
			if (env_optargs[idx] == ' ') 
				strncat(new_optargs, &env_optargs[idx++], 1);

			if ((optargs_len - idx) >= display_len) {
				if (strncmp(&env_optargs[idx], display, display_len) == 0) {
					idx += display_len;
					while (env_optargs[idx] != ' ' && env_optargs[idx] != '\0') {
						if (mode_idx + 1 > 14) {
							printf("The length of displaymode string is to long, should be smaller tha [14]\n");
							break;
						}
						mode[mode_idx++] = env_optargs[idx++];
					}
					continue;
				}
			}
		}
		strncat(new_optargs, &env_optargs[idx++], 1);
	}

	strncat(new_optargs, " ", 1);
	if (type = lcd_type()) {
		strncat(new_optargs, display, display_len);
		strncat(new_optargs, type, strlen(type));
	} else if (mode_idx > 0) {
		strncat(new_optargs, display, display_len);
		strncat(new_optargs, mode, mode_idx);
	}

	setenv("optargs", new_optargs);
	free(new_optargs);
}

#if 0
struct serial_device *default_serial_console(void)
{

	if (board_id != IA_BOARD) {
		return &eserial1_device;	/* UART0 */
	} else {
		return &eserial4_device;	/* UART3 */
	}
}
#endif

int board_init(void)
{
	
	/* Configure the i2c0 pin mux */
	enable_i2c0_pin_mux();

/* Modified by Conway */

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	
	printf("Did not find a recognized configuration, "
		"assuming General purpose EVM in Profile 0 with "
		"Daughter board\n");
	board_id = GP_BOARD;
	profile = 1;	/* profile 0 is internally considered as 1 */
	daughter_board_connected = 1;

	configure_evm_pin_mux(board_id, header.version, profile, daughter_board_connected);
	
#ifndef CONFIG_SPL_BUILD
	board_evm_init();
#endif
	gpmc_init();

	return 0;
}

int misc_init_r(void)
{
#ifdef DEBUG
	unsigned int cntr;
	unsigned char *valPtr;

	debug("EVM Configuration - ");
	debug("\tBoard id %x, profile %x, db %d\n", board_id, profile,
						daughter_board_connected);
	debug("Base Board EEPROM Data\n");
	valPtr = (unsigned char *)&header;
	for(cntr = 0; cntr < sizeof(header); cntr++) {
		if(cntr % 16 == 0)
			debug("\n0x%02x :", cntr);
		debug(" 0x%02x", (unsigned int)valPtr[cntr]);
	}
	debug("\n\n");

	debug("Board identification from EEPROM contents:\n");
	debug("\tBoard name   : %.8s\n", header.name);
	debug("\tBoard version: %.4s\n", header.version);
	debug("\tBoard serial : %.12s\n", header.serial);
	debug("\tBoard config : %.6s\n\n", header.config);
#endif
	
	lcd_identify();

#ifdef AUTO_UPDATESYS
        run_command("run updatesys", 0);
#endif
	return 0;
}

#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
	if (board_id == IA_BOARD) {
		/*
		* SPI bus number is switched to in case Industrial Automation
		* motor control EVM.
		*/
		setenv("spi_bus_no", "1");
		/* Change console to tty03 for IA Motor Control EVM */
		setenv("console", "ttyO3,115200n8");
	}

	return 0;
}
#endif

#ifdef CONFIG_DRIVER_TI_CPSW
/* TODO : Check for the board specific PHY */
static void evm_phy_init(char *name, int addr)
{
	unsigned short val;
	unsigned int cntr = 0;
	unsigned short phyid1, phyid2;
	int bone_pre_a3 = 0;

	if (board_id == BONE_BOARD && (!strncmp(header.version, "00A1", 4) ||
		    !strncmp(header.version, "00A2", 4)))
		bone_pre_a3 = 1;

	/*
	 * This is done as a workaround to support TLK110 rev1.0 PHYs.
	 * We can only perform these reads on these PHYs (currently
	 * only found on the IA EVM).
	 */
	if ((miiphy_read(name, addr, MII_PHYSID1, &phyid1) != 0) ||
			(miiphy_read(name, addr, MII_PHYSID2, &phyid2) != 0)) {
		printf("miiphy read id fail\n");
		return;
	}

	/* Enable Autonegotiation */
	if (miiphy_read(name, addr, MII_BMCR, &val) != 0) {
		printf("failed to read bmcr\n");
		return;
	}

	if (bone_pre_a3) {
		val &= ~(BMCR_FULLDPLX | BMCR_ANENABLE | BMCR_SPEED100);
		val |= BMCR_FULLDPLX;
	} else
		val |= BMCR_FULLDPLX | BMCR_ANENABLE | BMCR_SPEED100;

	if (miiphy_write(name, addr, MII_BMCR, val) != 0) {
		printf("failed to write bmcr\n");
		return;
	}

	miiphy_read(name, addr, MII_BMCR, &val);

	/*
	 * The 1.0 revisions of the GP board don't have functional
	 * gigabit ethernet so we need to disable advertising.
	 */
	if (board_id == GP_BOARD && !strncmp(header.version, "1.0", 3)) {
		miiphy_read(name, addr, MII_CTRL1000, &val);
		val &= ~PHY_1000BTCR_1000FD;
		val &= ~PHY_1000BTCR_1000HD;
		miiphy_write(name, addr, MII_CTRL1000, val);
		miiphy_read(name, addr, MII_CTRL1000, &val);
	}

	/* Setup general advertisement */
	if (miiphy_read(name, addr, MII_ADVERTISE, &val) != 0) {
		printf("failed to read anar\n");
		return;
	}

	if (bone_pre_a3)
		val |= (LPA_10HALF | LPA_10FULL);
	else
		val |= (LPA_10HALF | LPA_10FULL | LPA_100HALF | LPA_100FULL);

	if (miiphy_write(name, addr, MII_ADVERTISE, val) != 0) {
		printf("failed to write anar\n");
		return;
	}

	miiphy_read(name, addr, MII_ADVERTISE, &val);


#if 0 /* wo don't do the negotiation */
	/* Restart auto negotiation*/
	miiphy_read(name, addr, MII_BMCR, &val);
	val |= BMCR_ANRESTART;
	miiphy_write(name, addr, MII_BMCR, val);

	/*check AutoNegotiate complete - it can take upto 3 secs*/
	do {
		udelay(40000);
		cntr++;
		if (!miiphy_read(name, addr, MII_BMSR, &val)) {
			if (val & BMSR_ANEGCOMPLETE)
				break;
		}
	} while (cntr < 250);

	if (cntr >= 250)
		printf("Auto negotitation failed\n");
#endif

	return;
}

static void cpsw_control(int enabled)
{
	/* nothing for now */
	/* TODO : VTP was here before */
	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,	
		.sliver_reg_ofs	= 0xd80,
		.phy_id		= 4/*0*/,/* modified by MYIR */
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_id		= 6/*2*/,/* Modified by MYIR */
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= AM335X_CPSW_MDIO_BASE,
	.cpsw_base		= AM335X_CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.mac_control		= (1 << 5) /* MIIEN */,
	.control		= cpsw_control,
	.phy_init		= evm_phy_init,
	.gigabit_en		= 1,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

int board_eth_init(bd_t *bis)
{
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
	u_int32_t i;

	if (!eth_getenv_enetaddr("ethaddr", mac_addr)) {
		debug("<ethaddr> not set. Reading from E-fuse\n");
		/* try reading mac address from efuse */
		mac_lo = __raw_readl(MAC_ID0_LO);
		mac_hi = __raw_readl(MAC_ID0_HI);
		mac_addr[0] = mac_hi & 0xFF;
		mac_addr[1] = (mac_hi & 0xFF00) >> 8;
		mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
		mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
		mac_addr[4] = mac_lo & 0xFF;
		mac_addr[5] = (mac_lo & 0xFF00) >> 8;

		if (!is_valid_ether_addr(mac_addr)) {
			debug("Did not find a valid mac address in e-fuse. "
					"Trying the one present in EEPROM\n");

			for (i = 0; i < ETH_ALEN; i++)
				mac_addr[i] = header.mac_addr[0][i];
		}

		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
		else {
			printf("Caution: Using hardcoded mac address. "
				"Set <ethaddr> variable to overcome this.\n");
		}
	}
	
	/* set mii mode to rgmii in in device configure register */
	__raw_writel(RGMII_MODE_ENABLE, MAC_MII_SEL);
	

	cpsw_data.gigabit_en = 0;

	return cpsw_register(&cpsw_data);
}
#endif

#ifndef CONFIG_SPL_BUILD
#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0);
	return 0;
}
#endif

#ifdef CONFIG_NAND_TI81XX
/******************************************************************************
 * Command to switch between NAND HW and SW ecc
 *****************************************************************************/
extern void ti81xx_nand_switch_ecc(nand_ecc_modes_t hardware, int32_t mode);
static int do_switch_ecc(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	int type = 0;
	if (argc < 2)
		goto usage;

	if (strncmp(argv[1], "hw", 2) == 0) {
		if (argc == 3)
			type = simple_strtoul(argv[2], NULL, 10);
		ti81xx_nand_switch_ecc(NAND_ECC_HW, type);
	}
	else if (strncmp(argv[1], "sw", 2) == 0)
		ti81xx_nand_switch_ecc(NAND_ECC_SOFT, 0);
	else
		goto usage;

	return 0;

usage:
	printf("Usage: nandecc %s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(
	nandecc, 3, 1,	do_switch_ecc,
	"Switch NAND ECC calculation algorithm b/w hardware and software",
	"[sw|hw <hw_type>] \n"
	"   [sw|hw]- Switch b/w hardware(hw) & software(sw) ecc algorithm\n"
	"   hw_type- 0 for Hamming code\n"
	"            1 for bch4\n"
	"            2 for bch8\n"
	"            3 for bch16\n"
);

#endif /* CONFIG_NAND_TI81XX */
#endif /* CONFIG_SPL_BUILD */
