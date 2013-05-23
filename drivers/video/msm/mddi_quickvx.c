/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
/* For gpio control */
#include <linux/gpio.h>
/* For electrical power control */
#include <mach/vreg.h>
#include <linux/i2c.h>

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

/* WVGA Primary Display */
#define MDDI_QUICKVX_1_2		1
/* MDDI Manufacturer Code */
#define QUICKVX_MDDI_MFR_CODE	0xc583
/* MDDI Product Code */
#define QUICKVX_MDDI_PRD_CODE	0x5800

/* Register Address Maps */
/* MDDI Address Anti-fuse values for bits [31:22] */
#define QUICKVX_ADDR_31_22_AF	(0X000 << 22)

/* MDDI Address Maps */
/* VEE Block Address Base */
#define QUICKVX_VEE_BASE		(QUICKVX_ADDR_31_22_AF | 0x00000000)
/* SPI Block Address Base */
#define QUICKVX_SPI_BASE		(QUICKVX_ADDR_31_22_AF | 0x00010000)
/* Clock and Reset (CAR) Address Base */
#define QUICKVX_CAR_BASE		(QUICKVX_ADDR_31_22_AF | 0x00020000)
/* Register Control Block (RCB) Address Base */
#define QUICKVX_RCB_BASE		(QUICKVX_ADDR_31_22_AF | 0x00030000)
/* Cellular RAM Address Base */
#define QUICKVX_CELLRAM_BASE	(QUICKVX_ADDR_31_22_AF | 0x00100000)
/* FB through A2F Address Base */
#define QUICKVX_FB_A2F_BASE		(QUICKVX_ADDR_31_22_AF | 0x00200000)


/***************************************************
 * Common Registers in Register Control Block (RCB) Registers
 ***************************************************/
 /* CellRAM Configuration RCR Register */
#define QUICKVX_RCB_RCR_REG			(QUICKVX_RCB_BASE | 0x00000000)
/* Image Effect Register */
#define QUICKVX_RCB_IER_REG			(QUICKVX_RCB_BASE | 0x00000004)
/* Row Number Register */
#define QUICKVX_RCB_ROWNUM_REG		(QUICKVX_RCB_BASE | 0x00000008)
/* TCON Timing0 Register */
#define QUICKVX_RCB_TCON0_REG		(QUICKVX_RCB_BASE | 0x0000000C)
/* TCON Timing1 Register */
#define QUICKVX_RCB_TCON1_REG		(QUICKVX_RCB_BASE | 0x00000010)
/* TCON Timing2 Register */
#define QUICKVX_RCB_TCON2_REG		(QUICKVX_RCB_BASE | 0x00000014)
/* PWM Control Register */
#define QUICKVX_RCB_PWMC_REG		(QUICKVX_RCB_BASE | 0x00000018)
/* PWM Width Register */
#define QUICKVX_RCB_PWMW_REG		(QUICKVX_RCB_BASE | 0x0000001C)
/* VEE Configuration Register */
#define QUICKVX_RCB_VEECONF_REG		(QUICKVX_RCB_BASE | 0x00000020)
/* CellRAM Configuration BCR Register */
#define QUICKVX_RCB_CELLBCR_REG		(QUICKVX_RCB_BASE | 0x00000024)
/* CellRAM Configuration Control Register */
#define QUICKVX_RCB_CELLCC_REG		(QUICKVX_RCB_BASE | 0x00000028)
/* Use Case Register */
#define QUICKVX_RCB_USECASE_REG		(QUICKVX_RCB_BASE | 0x00000100)
/* Video Parameter Register */
#define QUICKVX_RCB_VPARM_REG		(QUICKVX_RCB_BASE | 0x00000104)
/* MDDI Client Wake-up Register */
#define QUICKVX_RCB_MCW_REG			(QUICKVX_RCB_BASE | 0x00000108)
/* Burst Length Register */
#define QUICKVX_RCB_BURSTLN_REG		(QUICKVX_RCB_BASE | 0x0000010C)
/* Display Attributes Register */
#define QUICKVX_RCB_DISPATTR_REG	(QUICKVX_RCB_BASE | 0x00000110)
/* Error Status Register */
#define QUICKVX_RCB_ERRSTAT_REG		(QUICKVX_RCB_BASE | 0x00000114)
/* Error Mask Register */
#define QUICKVX_RCB_ERRMSK_REG		(QUICKVX_RCB_BASE | 0x00000118)
/* MDDI ASSP FIFO Overflow Address Register */
#define QUICKVX_RCB_ASSPFOA_REG		(QUICKVX_RCB_BASE | 0x0000011C)
/* MDDI Fabric FIFO Overflow Address Register */
#define QUICKVX_RCB_FABFOA_REG		(QUICKVX_RCB_BASE | 0x00000120)
/* Incoming RGB FIFO Overflow Address Register */
#define QUICKVX_RCB_IRFOA_REG		(QUICKVX_RCB_BASE | 0x00000124)
/* SPI Overflow Address Register */
#define QUICKVX_RCB_SPIOA_REG		(QUICKVX_RCB_BASE | 0x00000128)
/* Ping Buffer Address Register */
#define QUICKVX_RCB_PINGBA_REG		(QUICKVX_RCB_BASE | 0x0000012C)
/* Pong Buffer Address Register */
#define QUICKVX_RCB_PONGBA_REG		(QUICKVX_RCB_BASE | 0x00000130)
/* Configuration Done Register */
#define QUICKVX_RCB_CONFDONE_REG	(QUICKVX_RCB_BASE | 0x00000134)
/* FIFO Flush Register */
#define QUICKVX_RCB_FFLUSH_REG		(QUICKVX_RCB_BASE | 0x00000138)


/***************************************************
 * SPI Block Registers
 ***************************************************/
/* SPI Rx0 Register */
#define QUICKVX_SPI_RX0_REG			(QUICKVX_SPI_BASE | 0x00000000)
/* SPI Rx1 Register */
#define QUICKVX_SPI_RX1_REG			(QUICKVX_SPI_BASE | 0x00000004)
/* SPI Rx2 Register */
#define QUICKVX_SPI_RX2_REG			(QUICKVX_SPI_BASE | 0x00000008)
/* SPI Rx3 Register */
#define QUICKVX_SPI_RX3_REG			(QUICKVX_SPI_BASE | 0x0000000C)
/* SPI Rx4 Register */
#define QUICKVX_SPI_RX4_REG			(QUICKVX_SPI_BASE | 0x00000010)
/* SPI Rx5 Register */
#define QUICKVX_SPI_RX5_REG			(QUICKVX_SPI_BASE | 0x00000014)
/* SPI Rx6 Register */
#define QUICKVX_SPI_RX6_REG			(QUICKVX_SPI_BASE | 0x00000018)
/* SPI Rx7 Register */
#define QUICKVX_SPI_RX7_REG			(QUICKVX_SPI_BASE | 0x0000001C)
/* SPI Tx0 Register */
#define QUICKVX_SPI_TX0_REG			(QUICKVX_SPI_BASE | 0x00000020)
/* SPI Tx1 Register */
#define QUICKVX_SPI_TX1_REG			(QUICKVX_SPI_BASE | 0x00000024)
/* SPI Tx2 Register */
#define QUICKVX_SPI_TX2_REG			(QUICKVX_SPI_BASE | 0x00000028)
/* SPI Tx3 Register */
#define QUICKVX_SPI_TX3_REG			(QUICKVX_SPI_BASE | 0x0000002C)
/* SPI Tx4 Register */
#define QUICKVX_SPI_TX4_REG			(QUICKVX_SPI_BASE | 0x00000030)
/* SPI Tx5 Register */
#define QUICKVX_SPI_TX5_REG			(QUICKVX_SPI_BASE | 0x00000034)
/* SPI Tx6 Register */
#define QUICKVX_SPI_TX6_REG			(QUICKVX_SPI_BASE | 0x00000038)
/* SPI Tx7 Register */
#define QUICKVX_SPI_TX7_REG			(QUICKVX_SPI_BASE | 0x0000003C)
/* SPI Control Register */
#define QUICKVX_SPI_CTRL_REG		(QUICKVX_SPI_BASE | 0x00000040)
/* SPI Transfer Length Register */
#define QUICKVX_SPI_TLEN_REG		(QUICKVX_SPI_BASE | 0x00000044)


/***************************************************
 * Clock and Reset (CAR) Block Registers
 ***************************************************/
/* ASSP Global Clock Enable Register */
#define QUICKVX_CAR_ASSP_GCE_REG	(QUICKVX_CAR_BASE | 0x00000000)
/* VLP Control1 Register */
#define QUICKVX_CAR_VLPCTRL1_REG	(QUICKVX_CAR_BASE | 0x00000004)
/* VLP Control2 Register */
#define QUICKVX_CAR_VLPCTRL2_REG	(QUICKVX_CAR_BASE | 0x00000008)
/* Clock Selection Register */
#define QUICKVX_CAR_CLKSEL_REG		(QUICKVX_CAR_BASE | 0x0000000C)
/* PLL Control Register */
#define QUICKVX_CAR_PLLCTRL_REG		(QUICKVX_CAR_BASE | 0x00000010)
/* PLL Clock Ratio Register */
#define QUICKVX_CAR_PLLCLKRATIO_REG	(QUICKVX_CAR_BASE | 0x00000014)


/***************************************************
 * VEE Block Registers
 ***************************************************/
/* VEE Control Register */
#define QUICKVX_VEE_VEECTRL_REG		(QUICKVX_VEE_BASE | 0x00000000)
/* Strength Register */
#define QUICKVX_VEE_STRENGTH_REG	(QUICKVX_VEE_BASE | 0x0000000C)
/* Variance Register */
#define QUICKVX_VEE_VARIANCE_REG	(QUICKVX_VEE_BASE | 0x00000010)
/* Slope Register */
#define QUICKVX_VEE_SLOPE_REG		(QUICKVX_VEE_BASE | 0x00000014)
/* Sharpen Control0 Register */
#define QUICKVX_VEE_SHRPCTRL0_REG	(QUICKVX_VEE_BASE | 0x0000001C)
/* Sharpen Control1 Register */
#define QUICKVX_VEE_SHRPCTRL1_REG	(QUICKVX_VEE_BASE | 0x00000020)
/* Upper Horizontal Positon Register */
#define QUICKVX_VEE_UHPOS_REG		(QUICKVX_VEE_BASE | 0x00000024)
/* Lower Horizontal Positon Register */
#define QUICKVX_VEE_LHPOS_REG		(QUICKVX_VEE_BASE | 0x00000028)
/* Upper Vertical Positon Register */
#define QUICKVX_VEE_UVPOS_REG		(QUICKVX_VEE_BASE | 0x0000002C)
/* Lower Vertical Positon Register */
#define QUICKVX_VEE_LVPOS_REG		(QUICKVX_VEE_BASE | 0x00000030)
/* Upper Frame Width Register */
#define QUICKVX_VEE_UFWDTH_REG		(QUICKVX_VEE_BASE | 0x00000034)
/* Lower Frame Width Register */
#define QUICKVX_VEE_LFWDTH_REG		(QUICKVX_VEE_BASE | 0x00000038)
/* Upper Frame Height Register */
#define QUICKVX_VEE_UFHGHT_REG		(QUICKVX_VEE_BASE | 0x0000003C)
/* Lower Frame Height Register */
#define QUICKVX_VEE_LFHGHT_REG		(QUICKVX_VEE_BASE | 0x00000040)
/* Control0 Register */
#define QUICKVX_VEE_CTRL0_REG		(QUICKVX_VEE_BASE | 0x00000044)
/* Control1 Register */
#define QUICKVX_VEE_CTRL1_REG		(QUICKVX_VEE_BASE | 0x00000048)
/* Video Enhancement Enable Register */
#define QUICKVX_VEE_VDOEEN_REG		(QUICKVX_VEE_BASE | 0x0000004C)
/* Black Level Register */
#define QUICKVX_VEE_BLCKLEV_REG		(QUICKVX_VEE_BASE | 0x00000050)
/* White Level Register */
#define QUICKVX_VEE_WHTLEV_REG		(QUICKVX_VEE_BASE | 0x00000054)
/* Amplification Limits Register */
#define QUICKVX_VEE_AMPLMTS_REG		(QUICKVX_VEE_BASE | 0x00000060)
/* Dithering Mode Register */
#define QUICKVX_VEE_DITHMOD_REG		(QUICKVX_VEE_BASE | 0x00000064)
/* Upper Look-up Data Register */
#define QUICKVX_VEE_ULUD_REG		(QUICKVX_VEE_BASE | 0x00000080)
/* Lower Look-up Data Register */
#define QUICKVX_VEE_LLUD_REG		(QUICKVX_VEE_BASE | 0x00000084)
/* Look-up Address Register */
#define QUICKVX_VEE_LUADDR_REG		(QUICKVX_VEE_BASE | 0x00000088)
/* Look-up Write Enable Register */
#define QUICKVX_VEE_LUWREN_REG		(QUICKVX_VEE_BASE | 0x0000008C)
/* VEE ID Register */
#define QUICKVX_VEE_VEEID_REG		(QUICKVX_VEE_BASE | 0x000003FC)
/* M_11 Register */
#define QUICKVX_VEE_M_11_REG		(QUICKVX_VEE_BASE | 0x000000C0)
/* M_12 Register */
#define QUICKVX_VEE_M_12_REG		(QUICKVX_VEE_BASE | 0x000000C4)
/* M_13 Register */
#define QUICKVX_VEE_M_13_REG		(QUICKVX_VEE_BASE | 0x000000C8)
/* M_21 Register */
#define QUICKVX_VEE_M_21_REG		(QUICKVX_VEE_BASE | 0x000000CC)
/* M_22 Register */
#define QUICKVX_VEE_M_22_REG		(QUICKVX_VEE_BASE | 0x000000D0)
/* M_23 Register */
#define QUICKVX_VEE_M_23_REG		(QUICKVX_VEE_BASE | 0x000000D4)
/* M_31 Register */
#define QUICKVX_VEE_M_31_REG		(QUICKVX_VEE_BASE | 0x000000D8)
/* M_32 Register */
#define QUICKVX_VEE_M_32_REG		(QUICKVX_VEE_BASE | 0x000000DC)
/* M_33 Register */
#define QUICKVX_VEE_M_33_REG		(QUICKVX_VEE_BASE | 0x000000E0)
/* R Offset Register */
#define QUICKVX_VEE_OFFSET_R_REG	(QUICKVX_VEE_BASE | 0x000000E8)
/* G Offset Register */
#define QUICKVX_VEE_OFFSET_G_REG	(QUICKVX_VEE_BASE | 0x000000EC)
/* B Offset Register */
#define QUICKVX_VEE_OFFSET_B_REG	(QUICKVX_VEE_BASE | 0x000000F0)

/* LCD Reset Register */
#define QUICKVX_FB_A2F_LCD_RESET_REG (QUICKVX_FB_A2F_BASE | 0x00000000)

/* Register bit defines */
/* PLL Lock bit in the PLL Control Register */
#define QUICKVX_PLL_LOCK_BIT		(1 << 7)

#define QL_SPI_CTRL_rSPISTart(x) (x)
#define QL_SPI_CTRL_rCPHA(x) (x << 1)
#define QL_SPI_CTRL_rCPOL(x) (x << 2)
#define QL_SPI_CTRL_rLSB(x) (x << 3)
#define QL_SPI_CTRL_rSLVSEL(x) (x << 4)
#define QL_SPI_CTRL_MASK_rTxDone (1 << 9)

#define QL_SPI_LCD_DEV_ID 0x1c
#define QL_SPI_LCD_RS(x) (x << 1)
#define QL_SPI_LCD_RW(x) (x)
#define QL_SPI_LCD_INDEX_START_BYTE ((QL_SPI_LCD_DEV_ID << 2) | \
	QL_SPI_LCD_RS(0) | QL_SPI_LCD_RW(0))
#define QL_SPI_LCD_CMD_START_BYTE ((QL_SPI_LCD_DEV_ID << 2) | \
	QL_SPI_LCD_RS(1) | QL_SPI_LCD_RW(0))
#define QL_SPI_CTRL_LCD_START (QL_SPI_CTRL_rSPISTart(1) | \
	QL_SPI_CTRL_rCPHA(1) | QL_SPI_CTRL_rCPOL(1) | \
	QL_SPI_CTRL_rLSB(0) | QL_SPI_CTRL_rSLVSEL(0))

#define MDDI_GAMMA_SET_MAX_NUM    23
#define MDDI_BRIGHT_LEVEL_MAX     27
#define MDDI_BRIGHT_LEVEL_DEFAULT 28
/*#define MDDI_BRIGHT_LEVEL_DEFAULT 39*/
/*#define MDDI_BRIGHT_LEVEL_MAX     25*/
/*#define MDDI_BRIGHT_LEVEL_DEFAULT 26*/

#define MDDI_RST_N   133
#define MDDI_VLP     17

struct i2c_client *i2c_quick_client = NULL;

/* Brightness Lv VEE ON */
#define BRIGHTNESS_USER_VAL_OFFSET 2
#define BRIGHTNESS_VEE_ON_LV  (14 + BRIGHTNESS_USER_VAL_OFFSET)

static uint32 mddi_quickvx_vee_brightness_tbl[] = {
    12 + BRIGHTNESS_USER_VAL_OFFSET,
    13 + BRIGHTNESS_USER_VAL_OFFSET,
    13 + BRIGHTNESS_USER_VAL_OFFSET,
    14 + BRIGHTNESS_USER_VAL_OFFSET,
    15 + BRIGHTNESS_USER_VAL_OFFSET,
    15 + BRIGHTNESS_USER_VAL_OFFSET,
    16 + BRIGHTNESS_USER_VAL_OFFSET,
    17 + BRIGHTNESS_USER_VAL_OFFSET,
    17 + BRIGHTNESS_USER_VAL_OFFSET,
    18 + BRIGHTNESS_USER_VAL_OFFSET,
    19 + BRIGHTNESS_USER_VAL_OFFSET,
    20 + BRIGHTNESS_USER_VAL_OFFSET,
};
static uint32 mddi_quickvx_vee_brightness_tbl_size
    = sizeof(mddi_quickvx_vee_brightness_tbl)/sizeof(uint32);

/* VEE Strength */
#define VEE_STRENGTH_BIT_FIELD(x) ( x << 3 )
static uint32 mddi_quickvx_vee_strength_tbl[] = {
    VEE_STRENGTH_BIT_FIELD(2),
    VEE_STRENGTH_BIT_FIELD(2),
    VEE_STRENGTH_BIT_FIELD(3),
    VEE_STRENGTH_BIT_FIELD(3),
    VEE_STRENGTH_BIT_FIELD(3),
    VEE_STRENGTH_BIT_FIELD(4),
    VEE_STRENGTH_BIT_FIELD(4),
    VEE_STRENGTH_BIT_FIELD(4),
    VEE_STRENGTH_BIT_FIELD(5),
    VEE_STRENGTH_BIT_FIELD(5),
    VEE_STRENGTH_BIT_FIELD(5),
    VEE_STRENGTH_BIT_FIELD(6),
};
static uint32 mddi_quickvx_vee_strength_tbl_size
    = sizeof(mddi_quickvx_vee_strength_tbl)/sizeof(uint32);

/* VEE Seq Table */
#define QUICKVX_REVISION_ID_REG           0x210000
#define QUICKVX_VEE_SWITCHING_REG         0x220000
#define QUICKVX_VEE_COMPENSATION_REG      0x220008
#define QUICKVX_HIBERNATION_CONTROL_REG   0x220010
#define QUICKVX_HIBERNATION_TRANS_D_REG   0x220014
#define QUICKVX_HIBERNATION_LINE_D_REG    0x220018
#define QUICKVX_VEE_MODE_WRITE_BUSY_BIT   0x00008000
#define QUICKVX_VEE_SWITCHING_ON          0x3
#define QUICKVX_VEE_SWITCHING_OFF         0x2
#define QUICKVX_VEE_CTRL_ON               0x1
#define QUICKVX_VEE_CTRL_OFF              0x0

#define QUICKVX_NEW_DEGINE_REV            0x27B
#define QUICKVX_02_DEGINE_REV            0x470
#define QUICKVX_VEE_SWITCHING_RETRY       100

#define MDDI_QUICKVX_VEE_SEQ_MAX 4
#define MDDI_QUICKVX_VEE_SEQ_OFFSET 3
#define MDDI_QUICKVX_VEE_STRENGTH_ONLY 1

#define VEE_SWITCHING                     0x00
#define VEE_STRENGTH                      0x01

/* VEE-Auto Brightness state */
typedef enum {
    OFF,
    ON,
}mddi_quickvx_ctrl_e_type;

typedef enum {
    MDDI_QUICKVX_VEE_DO_SEQ = 0,
    MDDI_QUICKVX_VEE_NON_SEQ,
}mddi_quickvx_vee_ctrl_e_type;

struct mddi_quickvx_vee_value_s_type{
    mddi_quickvx_ctrl_e_type on;
    uint32 vee_strength;
    uint32 vee_bl_lv;
};

static mddi_quickvx_ctrl_e_type mddi_quickvx_al_state = OFF;
static mddi_quickvx_ctrl_e_type mddi_quickvx_vee_state = OFF;
static uint32 mddi_quickvx_ic_rev = 0;

/* brightness default */
static uint32 mddi_blc_tableDE[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000149,
    0x0000013C, 0x00000129, 0x000001B7, 0x000001BF, 0x000001AD, 0x000001B5,
    0x000001BE, 0x000001AC, 0x000001C5, 0x000001CF, 0x000001C3, 0x00000100,
    0x00000178, 0x00000100, 0x0000016D, 0x00000100, 0x00000192
};

/* brightness Off */
static uint32 mddi_BLC_tableOff[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001C4, 0x000001C9, 0x000001BE, 0x000001C5,
    0x000001C9, 0x000001BC, 0x000001CE, 0x000001D7, 0x000001CD, 0x00000100,
    0x0000013F, 0x00000100, 0x00000138, 0x00000100, 0x0000014F
};
/* brightness Dim */
static uint32 mddi_BLC_tableDim[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001C3, 0x000001C8, 0x000001BC, 0x000001C3,
    0x000001C8, 0x000001BA, 0x000001CD, 0x000001D6, 0x000001CC, 0x00000100,
    0x00000145, 0x00000100, 0x0000013E, 0x00000100, 0x00000156
};

/* brightness 0 */
static uint32 mddi_blc_table00[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001C3, 0x000001C8, 0x000001BB, 0x000001C2,
    0x000001C7, 0x000001B9, 0x000001CC, 0x000001D5, 0x000001CB, 0x00000100,
    0x00000147, 0x00000100, 0x00000140, 0x00000100, 0x00000159
};

/* brightness 1 */
static uint32 mddi_blc_table01[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001C2, 0x000001C7, 0x000001B9, 0x000001C0,
    0x000001C6, 0x000001B6, 0x000001CB, 0x000001D4, 0x000001CA, 0x00000100,
    0x0000014F, 0x00000100, 0x00000148, 0x00000100, 0x00000162
};

/* brightness 2 */
static uint32 mddi_blc_table02[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001BF, 0x000001C5, 0x000001B5, 0x000001BC,
    0x000001C4, 0x000001B3, 0x000001C9, 0x000001D2, 0x000001C8, 0x00000100,
    0x0000015A, 0x00000100, 0x00000153, 0x00000100, 0x0000016F
};

/* brightness 3 */
static uint32 mddi_blc_table03[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001BE, 0x000001C4, 0x000001B3, 0x000001B9,
    0x000001C2, 0x000001B0, 0x000001C8, 0x000001D1, 0x000001C6, 0x00000100,
    0x00000166, 0x00000100, 0x0000015D, 0x00000100, 0x0000017C
};

/* brightness 4 */
static uint32 mddi_blc_table04[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001BD, 0x000001C4, 0x000001B4, 0x000001B8,
    0x000001C1, 0x000001AF, 0x000001C7, 0x000001D0, 0x000001C5, 0x00000100,
    0x0000016E, 0x00000100, 0x00000165, 0x00000100, 0x00000187
};

/* brightness 5 */
static uint32 mddi_blc_table05[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x0000014A,
    0x00000138, 0x00000129, 0x000001B7, 0x000001BF, 0x000001AD, 0x000001B6,
    0x000001BF, 0x000001AD, 0x000001C6, 0x000001D0, 0x000001C4, 0x00000100,
    0x00000173, 0x00000100, 0x00000169, 0x00000100, 0x0000018D
};

/* brightness 6 */
static uint32 mddi_blc_table06[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000149,
    0x0000013C, 0x00000129, 0x000001B7, 0x000001BF, 0x000001AD, 0x000001B5,
    0x000001BE, 0x000001AC, 0x000001C5, 0x000001CF, 0x000001C3, 0x00000100,
    0x00000178, 0x00000100, 0x0000016D, 0x00000100, 0x00000192
};

/* brightness 7 */
static uint32 mddi_blc_table07[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000148,
    0x0000013F, 0x00000129, 0x000001B6, 0x000001BF, 0x000001AC, 0x000001B5,
    0x000001BD, 0x000001AB, 0x000001C4, 0x000001CE, 0x000001C3, 0x00000100,
    0x0000017C, 0x00000100, 0x00000171, 0x00000100, 0x00000197
};

/* brightness 8 */
static uint32 mddi_blc_table08[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000147,
    0x00000142, 0x00000129, 0x000001B6, 0x000001BF, 0x000001AC, 0x000001B4,
    0x000001BD, 0x000001AB, 0x000001C3, 0x000001CD, 0x000001C2, 0x00000100,
    0x00000180, 0x00000100, 0x00000175, 0x00000100, 0x0000019C
};

/* brightness 9 */
static uint32 mddi_blc_table09[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000148,
    0x00000144, 0x00000129, 0x000001B5, 0x000001BE, 0x000001AB, 0x000001B3,
    0x000001BC, 0x000001AA, 0x000001C2, 0x000001CB, 0x000001C1, 0x00000100,
    0x00000184, 0x00000100, 0x00000179, 0x00000100, 0x000001A1
};

/* brightness 10 */
static uint32 mddi_blc_table10[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000149,
    0x00000145, 0x00000129, 0x000001B5, 0x000001BE, 0x000001AB, 0x000001B2,
    0x000001BB, 0x000001A9, 0x000001C1, 0x000001CA, 0x000001C0, 0x00000100,
    0x00000188, 0x00000100, 0x0000017D, 0x00000100, 0x000001A6
};

/* brightness 11 */
static uint32 mddi_blc_table11[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000146,
    0x00000146, 0x00000128, 0x000001B5, 0x000001BE, 0x000001AB, 0x000001B1,
    0x000001BA, 0x000001A8, 0x000001C0, 0x000001C9, 0x000001BF, 0x00000100,
    0x0000018C, 0x00000100, 0x00000180, 0x00000100, 0x000001AA
};

/* brightness 12 */
static uint32 mddi_blc_table12[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000144,
    0x00000147, 0x00000127, 0x000001B5, 0x000001BE, 0x000001AB, 0x000001B0,
    0x000001BA, 0x000001A7, 0x000001BF, 0x000001C9, 0x000001BE, 0x00000100,
    0x0000018F, 0x00000100, 0x00000183, 0x00000100, 0x000001AF
};

/* brightness 13 */
static uint32 mddi_blc_table13[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000144,
    0x00000149, 0x00000127, 0x000001B5, 0x000001BE, 0x000001AB, 0x000001B0,
    0x000001B9, 0x000001A6, 0x000001BE, 0x000001C8, 0x000001BD, 0x00000100,
    0x00000193, 0x00000100, 0x00000187, 0x00000100, 0x000001B4
};

/* brightness 14 */
static uint32 mddi_blc_table14[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000143,
    0x0000014A, 0x00000126, 0x000001B4, 0x000001BD, 0x000001AA, 0x000001AF,
    0x000001B9, 0x000001A6, 0x000001BD, 0x000001C7, 0x000001BC, 0x00000100,
    0x00000197, 0x00000100, 0x0000018A, 0x00000100, 0x000001B8
};

/* brightness 15 */
static uint32 mddi_blc_table15[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000143,
    0x0000014B, 0x00000126, 0x000001B4, 0x000001BD, 0x000001AA, 0x000001AF,
    0x000001B8, 0x000001A5, 0x000001BD, 0x000001C7, 0x000001BC, 0x00000100,
    0x0000019A, 0x00000100, 0x0000018D, 0x00000100, 0x000001BC
};

/* brightness 16 */
static uint32 mddi_blc_table16[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000141,
    0x0000014B, 0x00000123, 0x000001B4, 0x000001BD, 0x000001AA, 0x000001AE,
    0x000001B7, 0x000001A4, 0x000001BC, 0x000001C6, 0x000001BB, 0x00000100,
    0x0000019D, 0x00000100, 0x00000190, 0x00000100, 0x000001C0
};

/* brightness 17 */
static uint32 mddi_blc_table17[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000141,
    0x0000014C, 0x00000124, 0x000001B4, 0x000001BD, 0x000001A9, 0x000001AE,
    0x000001B7, 0x000001A4, 0x000001BB, 0x000001C5, 0x000001BA, 0x00000100,
    0x000001A0, 0x00000100, 0x00000193, 0x00000100, 0x000001C4
};

/* brightness 18 */
static uint32 mddi_blc_table18[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000142,
    0x0000014C, 0x00000124, 0x000001B3, 0x000001BC, 0x000001A9, 0x000001AE,
    0x000001B6, 0x000001A4, 0x000001BB, 0x000001C4, 0x000001BA, 0x00000100,
    0x000001A3, 0x00000100, 0x00000196, 0x00000100, 0x000001C8
};

/* brightness 19 */
static uint32 mddi_blc_table19[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000142,
    0x0000014D, 0x00000125, 0x000001B3, 0x000001BC, 0x000001A9, 0x000001AD,
    0x000001B6, 0x000001A3, 0x000001BA, 0x000001C3, 0x000001B9, 0x00000100,
    0x000001A6, 0x00000100, 0x00000199, 0x00000100, 0x000001CB
};

/* brightness 20 */
static uint32 mddi_blc_table20[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000143,
    0x0000014E, 0x00000126, 0x000001B2, 0x000001BC, 0x000001A8, 0x000001AD,
    0x000001B5, 0x000001A3, 0x000001BA, 0x000001C3, 0x000001B9, 0x00000100,
    0x000001A9, 0x00000100, 0x0000019B, 0x00000100, 0x000001CE
};

/* brightness 21 */
static uint32 mddi_blc_table21[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000143,
    0x0000014F, 0x00000125, 0x000001B2, 0x000001BB, 0x000001A8, 0x000001AD,
    0x000001B5, 0x000001A3, 0x000001B9, 0x000001C2, 0x000001B8, 0x00000100,
    0x000001AC, 0x00000100, 0x0000019D, 0x00000100, 0x000001D1
};

/* brightness 22 */
static uint32 mddi_blc_table22[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000143,
    0x00000150, 0x00000125, 0x000001B1, 0x000001BB, 0x000001A8, 0x000001AC,
    0x000001B5, 0x000001A2, 0x000001B8, 0x000001C1, 0x000001B7, 0x00000100,
    0x000001AF, 0x00000100, 0x000001A0, 0x00000100, 0x000001D4
};

/* brightness 23 */
static uint32 mddi_blc_table23[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000142,
    0x00000150, 0x00000124, 0x000001B1, 0x000001BB, 0x000001A7, 0x000001AC,
    0x000001B4, 0x000001A2, 0x000001B7, 0x000001C0, 0x000001B6, 0x00000100,
    0x000001B2, 0x00000100, 0x000001A2, 0x00000100, 0x000001D8
};

/* brightness 24 */
static uint32 mddi_blc_table24[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000142,
    0x00000151, 0x00000124, 0x000001B0, 0x000001BA, 0x000001A7, 0x000001AB,
    0x000001B4, 0x000001A1, 0x000001B6, 0x000001C0, 0x000001B5, 0x00000100,
    0x000001B5, 0x00000100, 0x000001A5, 0x00000100, 0x000001DC
};

/* brightness 25 */
static uint32 mddi_blc_table25[MDDI_GAMMA_SET_MAX_NUM] =
{
    0x000000FA, 0x00000102, 0x00000118, 0x00000108, 0x00000124, 0x00000142,
    0x00000151, 0x00000123, 0x000001B0, 0x000001BA, 0x000001A7, 0x000001AB,
    0x000001B4, 0x000001A1, 0x000001B6, 0x000001C0, 0x000001B5, 0x00000100,
    0x000001B8, 0x00000100, 0x000001A8, 0x00000100, 0x000001E0
};

static const uint32* mddi_blc_gamma_table[] =
{
    mddi_BLC_tableOff,
    mddi_BLC_tableDim,
    mddi_blc_table00,
    mddi_blc_table01,
    mddi_blc_table02,
    mddi_blc_table03,
    mddi_blc_table04,
    mddi_blc_table05,
    mddi_blc_table06,
    mddi_blc_table07,
    mddi_blc_table08,
    mddi_blc_table09,
    mddi_blc_table10,
    mddi_blc_table11,
    mddi_blc_table12,
    mddi_blc_table13,
    mddi_blc_table14,
    mddi_blc_table15,
    mddi_blc_table16,
    mddi_blc_table17,
    mddi_blc_table18,
    mddi_blc_table19,
    mddi_blc_table20,
    mddi_blc_table21,
    mddi_blc_table22,
    mddi_blc_table23,
    mddi_blc_table24,
    mddi_blc_table25,
    mddi_blc_tableDE
};

extern struct mddi_local_disp_state_type mddi_local_state;

boolean mddi_refresh_force_flg = FALSE;

extern struct semaphore disp_local_mutex;

extern int32 mddi_client_type;

#define MDDI_SKEW_CONTROL_MAX  5

static uint32 mddi_skew_buff[MDDI_SKEW_CONTROL_MAX][2] =
{
    { 0x00210008, 0x0000001F },  /* Skew Calibration Control */
    { 0x0021000C, 0x00000003 },  /* Skew DATA0               */
    { 0x00210010, 0x00000001 },  /* Skew DATA1               */
    { 0x00210014, 0x00000004 },  /* Skew STB                 */
    { 0x0021001C, 0x00000000 },  /* Skew CA/CCM              */
};

static void mddi_quickvx_lcd_set_backlight(struct msm_fb_data_type *mfd);
static void mddi_quickvx_vee_new_degine_seq(
    mddi_quickvx_ctrl_e_type ctrl,
    uint32 strength
);

static uint32 mddi_quickvx_vee_ctrl(
    uint32 bl_lv,
    mddi_quickvx_vee_ctrl_e_type ctrl
);

#define QUICKVX_WRITE_BUSY_BIT   0x00008000

static void mddi_quickvx_wait_block_switch( void )
{
    int i;
    int ret;

    uint32 reg_addr = QUICKVX_HIBERNATION_CONTROL_REG;
    uint32 rd_value = 0;

    for(i=0; i<100; i++)
    {
       ret = i2c_master_send( i2c_quick_client,
                              (char*)(&reg_addr),
                              sizeof(reg_addr) );

       DISP_LOCAL_LOG_EMERG("DISP: wait block switch wr:reg[0x%X]\n",(int)(reg_addr));

       ret = i2c_master_recv( i2c_quick_client,
                              (char*)(&rd_value),
                              sizeof(rd_value) );

       DISP_LOCAL_LOG_EMERG("DISP: wait block switcht rev rd:data[0x%X]\n",(int)(rd_value));

       if( (ret>0)
           && ((rd_value & QUICKVX_WRITE_BUSY_BIT)==0x00) )
       {
           break;
       }

       mddi_wait(1);
    }

    DISP_LOCAL_LOG_EMERG("DISP: wait for switch block\n");

    if(i >= 100)
    {
        DISP_LOCAL_LOG_EMERG("DISP: wait for test TimeOut!!\n");
        DISP_LOCAL_LOG_EMERG("DISP: wait for test I2C ret = %d\n",ret);
    }

    return;
}

static int mddi_quickvx_i2c_write(uint32 address, uint32 value)
{
    uint32 buff[2];
    int i2c_ret;
    int ret = 0;

    if( NULL == i2c_quick_client)
    {
        DISP_LOCAL_LOG_EMERG( "DISP: i2c NO register client!!\n");
        return 1;
    }

    buff[0] = address;
    buff[1] = value;

    DISP_LOCAL_LOG_EMERG( "DISP: i2c wr:reg[0x%X] data[0x%X]\n",
                          (int)address,
                          (int)value );

    i2c_ret = i2c_master_send( i2c_quick_client,
                               (char*)buff,
                               sizeof(buff));

    if( i2c_ret != sizeof(buff))
    {
        DISP_LOCAL_LOG_EMERG( "DISP: i2c err!! ret[0x%X]\n",i2c_ret);
        ret = 1;
    }

    return ret;
}

int ql_mddi_write(uint32 address, uint32 value)
{
	uint32 regval = 0;
	int ret = 0;

	ret = mddi_queue_register_write(address, value, TRUE, 0);

	if (!ret) {
		ret = mddi_queue_register_read(address, &regval, TRUE, 0);
		if (regval != value) {
			MDDI_MSG_DEBUG("\nMismatch: ql_mddi_write[0x%x]->0x%x "
				"r0x%x\n", address, value, regval);
		} else {
			MDDI_MSG_DEBUG("\nMatch: ql_mddi_write[0x%x]->0x%x "
				"r0x%x\n", address, value, regval);
		}
	}

	return ret;
}

int ql_mddi_read(uint32 address, uint32 *regval)
{
	int ret = 0;

	ret = mddi_queue_register_read(address, regval, TRUE, 0);
	MDDI_MSG_DEBUG("\nql_mddi_read[0x%x]=0x%x", address, *regval);

	return ret;
}

int ql_send_spi_cmd_to_lcd(uint32 index, uint32 cmd)
{
	int retry, ret;
	uint32 readval;

	MDDI_MSG_DEBUG("\n %s(): index 0x%x, cmd 0x%x", __func__, index, cmd);
	/* do the index phase */
	/* send 24 bits in the index phase */
	ql_mddi_write(QUICKVX_SPI_TLEN_REG, 23);

	/* send 24 bits in the index phase, starting at bit 23 of TX0 reg */
	ql_mddi_write(QUICKVX_SPI_TX0_REG,
		(QL_SPI_LCD_INDEX_START_BYTE << 16) | index);

	/* set start */
	ql_mddi_write(QUICKVX_SPI_CTRL_REG,  QL_SPI_CTRL_LCD_START);
	retry = 0;

	do {
		ret = ql_mddi_read(QUICKVX_SPI_CTRL_REG, &readval);

		if (ret || ++retry > 5) {
			MDDI_MSG_DEBUG("\n ql_send_spi_cmd_to_lcd: retry "
				"timeout at index phase, ret = %d", ret);
			return -EIO;
		}
		mddi_wait(1);
	} while ((readval & QL_SPI_CTRL_MASK_rTxDone) == 0);

	/* do the command phase */
	/* send 24 bits in the cmd phase */
	ql_mddi_write(QUICKVX_SPI_TLEN_REG, 23);

	/* send 24 bits in the cmd phase, starting at bit 23 of TX0 reg. */
	ql_mddi_write(QUICKVX_SPI_TX0_REG,
		(QL_SPI_LCD_CMD_START_BYTE << 16) | cmd);

	/* set start */
	ql_mddi_write(QUICKVX_SPI_CTRL_REG,  QL_SPI_CTRL_LCD_START);
	retry = 0;

	do {
		ret = ql_mddi_read(QUICKVX_SPI_CTRL_REG, &readval);

		if (ret || ++retry > 5) {
			MDDI_MSG_DEBUG("\n ql_send_spi_cmd_to_lcd: retry "
				"timeout at cmd phase, ret = %d", ret);
			return -EIO;
		}
		mddi_wait(1);
	} while ((readval & QL_SPI_CTRL_MASK_rTxDone) == 0);

	return 0;
}


int ql_send_spi_data_from_lcd(uint32 index, uint32 *value)
{
	int retry, ret;
	uint32 readval;

	MDDI_MSG_DEBUG("\n %s(): index 0x%x", __func__, index);
	/* do the index phase */
	/* send 24 bits in the index phase */
	ql_mddi_write(QUICKVX_SPI_TLEN_REG, 23);

	/* send 24 bits in the index phase, starting at bit 23 of TX0 reg */
	ql_mddi_write(QUICKVX_SPI_TX0_REG,
		(QL_SPI_LCD_INDEX_START_BYTE << 16) | index);

	/* set start */
	ql_mddi_write(QUICKVX_SPI_CTRL_REG,  QL_SPI_CTRL_LCD_START);
	retry = 0;

	do {
		ret = ql_mddi_read(QUICKVX_SPI_CTRL_REG, &readval);

		if (ret || ++retry > 5) {
			MDDI_MSG_DEBUG("\n ql_send_spi_cmd_to_lcd: retry "
				"timeout at index phase, ret = %d", ret);
			return -EIO;
		}
		mddi_wait(1);
	} while ((readval & QL_SPI_CTRL_MASK_rTxDone) == 0);

	/* do the command phase */
	/* send 8 bits  and read 24 bits in the cmd phase, so total 32 bits */
	ql_mddi_write(QUICKVX_SPI_TLEN_REG, 31);

	/* send 24 bits in the cmd phase, starting at bit 31 of TX0 reg */
	ql_mddi_write(QUICKVX_SPI_TX0_REG,
		((QL_SPI_LCD_CMD_START_BYTE << 16)) << 8);

	/* set start */
	ql_mddi_write(QUICKVX_SPI_CTRL_REG,  QL_SPI_CTRL_LCD_START);
	retry = 0;

	do {
		ret = ql_mddi_read(QUICKVX_SPI_CTRL_REG, &readval);

		if (ret || ++retry > 5) {
			MDDI_MSG_DEBUG("\n ql_send_spi_cmd_to_lcd: retry "
				"timeout at cmd phase, ret = %d", ret);
			return -EIO;
		}
		mddi_wait(1);
	} while ((readval & QL_SPI_CTRL_MASK_rTxDone) == 0);

	/* value will appear at lower 16 bits */
	ret = ql_mddi_read(QUICKVX_SPI_RX0_REG, value);

	if (!ret) {
		*value = *value & 0xffff;
		MDDI_MSG_DEBUG("\n QUICKVX_SPI_RX0_REG value = 0x%x", *value);
	} else
		MDDI_MSG_DEBUG("\n Read QUICKVX_SPI_RX0_REG Failed");

	return ret;
}

/*===========================================================================

FUNCTION  QL_SEND_SPI_DATA

===========================================================================*/
int ql_send_spi_data(uint32 data)
{
    int retry, ret;
    uint32 readval;

    MDDI_MSG_DEBUG("\n %s(): data 0x%x", __func__, data);

    retry = 0;

    ql_mddi_write(QUICKVX_SPI_TX0_REG,data);

    ql_mddi_write(QUICKVX_SPI_CTRL_REG,0x1);
    /* Polling */
    while(1)
    {
        ret = ql_mddi_read(QUICKVX_SPI_CTRL_REG, &readval);

        if ((0 == ret) && ((readval & QL_SPI_CTRL_MASK_rTxDone) != 0))
        {
            break;
        }

        if (ret || ++retry > 100) /* trial! 100 times */
        {
            MDDI_MSG_DEBUG("\n ql_send_spi_data: retry "
                "timeout at cmd phase, ret = %d", ret);
            return -EIO;
        }
        mddi_wait(1);
    }

    return ret;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_GET_REVISION

===========================================================================*/
static void mddi_quickvx_get_revision(void)
{
    uint32 reg_addr = QUICKVX_REVISION_ID_REG;
    uint32 rd_value = 0x00;
    int ret;

    if ( (NULL == i2c_quick_client) 
         || (mddi_quickvx_ic_rev) )
    {
        DISP_LOCAL_LOG_EMERG("DISP: Get rev:[0x%X]\n",(int)(mddi_quickvx_ic_rev));
        return;
    }

    ret = i2c_master_send( i2c_quick_client,
                           (char*)(&reg_addr),
                           sizeof(reg_addr) );

    DISP_LOCAL_LOG_EMERG("DISP: Get rev wr:reg[0x%X]\n",(int)(reg_addr));

    ret = i2c_master_recv( i2c_quick_client,
                           (char*)(&rd_value),
                           sizeof(rd_value) );

    DISP_LOCAL_LOG_EMERG("DISP: Get rev rd:data[0x%X]\n",(int)(rd_value));

    if( ret > 0 )
    {
        mddi_quickvx_ic_rev = rd_value;
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_WAIT_BLOCK_SWITCH

===========================================================================*/
static void mddi_quickvx_vee_wait_block_switch( uint8 kind )
{
    int i;
    int ret;

    uint32 reg_addr = QUICKVX_VEE_SWITCHING_REG;
    uint32 rd_value = 0;

    if( VEE_STRENGTH == kind )
    {
        reg_addr = QUICKVX_VEE_COMPENSATION_REG;
    }

    for(i=0; i<QUICKVX_VEE_SWITCHING_RETRY; i++)
    {
       ret = i2c_master_send( i2c_quick_client,
                              (char*)(&reg_addr),
                              sizeof(reg_addr) );

       DISP_LOCAL_LOG_EMERG("DISP: wait vee switch wr:reg[0x%X]\n",(int)(reg_addr));

       ret = i2c_master_recv( i2c_quick_client,
                              (char*)(&rd_value),
                              sizeof(rd_value) );

       DISP_LOCAL_LOG_EMERG("DISP: wait vee switcht rev rd:data[0x%X]\n",(int)(rd_value));

       if( (ret>0)
           && ((rd_value & QUICKVX_VEE_MODE_WRITE_BUSY_BIT)==0x00) )
       {
           break;
       }

       mddi_wait(1);
    }

    DISP_LOCAL_LOG_EMERG("DISP: wait for switch vee\n");

    if(i >= QUICKVX_VEE_SWITCHING_RETRY)
    {
        DISP_LOCAL_LOG_EMERG("DISP: wait for switch vee TimeOut!!\n");
        DISP_LOCAL_LOG_EMERG("DISP: wait for switch vee I2C ret = %d\n",ret);
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_NEW_DEGINE_SEQ

===========================================================================*/
static void mddi_quickvx_vee_new_degine_seq(
    mddi_quickvx_ctrl_e_type ctrl,
    uint32 strength
)
{
    uint32 buff[2];

    if ( NULL == i2c_quick_client )
    {
        return;
    }

    if( ON == ctrl )
    {
        DISP_LOCAL_LOG_EMERG("DISP: vee on (new degine)\n");
        DISP_LOCAL_LOG_EMERG("DISP: vee strength:[%d]\n",(int)(strength));

        /* Use Case Switching Register */
        buff[0] = QUICKVX_VEE_SWITCHING_REG;
        buff[1] = QUICKVX_VEE_SWITCHING_ON;

        i2c_master_send( i2c_quick_client,
                         (char*)buff,
                         sizeof(buff) );
        DISP_LOCAL_LOG_EMERG("DISP: I2C wr: reg[0x%X] data[0x%X]\n",
                (int)(buff[0]),(int)(buff[1]));

        /* wait */
        mddi_quickvx_vee_wait_block_switch(VEE_SWITCHING);

        /* VEE Compensation Register */
        buff[0] = QUICKVX_VEE_COMPENSATION_REG;
        buff[1] = strength;

        i2c_master_send( i2c_quick_client,
                         (char*)buff,
                         sizeof(buff) );
        DISP_LOCAL_LOG_EMERG("DISP: I2C wr: reg[0x%X] data[0x%X]\n",
                (int)(buff[0]),(int)(buff[1]));

        /* wait */
        mddi_quickvx_vee_wait_block_switch(VEE_STRENGTH);

    }
    else
    {/* OFF */
        DISP_LOCAL_LOG_EMERG("DISP: vee off (new degine)\n");

        /* VEE Compensation Register */
        buff[0] = QUICKVX_VEE_COMPENSATION_REG;
        buff[1] = 0;

        i2c_master_send( i2c_quick_client,
                         (char*)buff,
                         sizeof(buff) );
        DISP_LOCAL_LOG_EMERG("DISP: I2C wr: reg[0x%X] data[0x%X]\n",
                (int)(buff[0]),(int)(buff[1]));

        /* wait */
        mddi_quickvx_vee_wait_block_switch(VEE_STRENGTH);

        /* Use Case Switching Register */
        buff[0] = QUICKVX_VEE_SWITCHING_REG;
        buff[1] = QUICKVX_VEE_SWITCHING_OFF;

        i2c_master_send( i2c_quick_client,
                         (char*)buff,
                         sizeof(buff) );
        DISP_LOCAL_LOG_EMERG("DISP: I2C wr: reg[0x%X] data[0x%X]\n",
                (int)(buff[0]),(int)(buff[1]));

        /* wait */
        mddi_quickvx_vee_wait_block_switch(VEE_SWITCHING);
    }

    return;
}


/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_SEQ

===========================================================================*/
static void mddi_quickvx_vee_seq(
    mddi_quickvx_ctrl_e_type ctrl,
    uint32 strength,
    uint32 opt
)
{
    uint32 buff[2];

    if( mddi_quickvx_ic_rev < QUICKVX_02_DEGINE_REV )
    {
        return;
    }

    if( MDDI_QUICKVX_VEE_STRENGTH_ONLY == opt )
    {
        if(NULL != i2c_quick_client)
        {
            buff[0] = QUICKVX_VEE_COMPENSATION_REG;
            buff[1] = strength;

            i2c_master_send( i2c_quick_client,
                             (char*)buff,
                             sizeof(buff) );

            /* wait */
            mddi_quickvx_vee_wait_block_switch(VEE_STRENGTH);

            DISP_LOCAL_LOG_EMERG("DISP: vee strength:[%d]\n",(int)((buff[1])>>3));
        }
    }
    else
    {
        mddi_quickvx_vee_new_degine_seq( ctrl, strength );
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_NV_COPY

===========================================================================*/
static void mddi_quickvx_vee_nv_copy(
    uint8* vee_brightness,
    uint8* vee_strength
)
{
    int i;

    for(i=0; i<mddi_quickvx_vee_brightness_tbl_size; i++)
    {
        mddi_quickvx_vee_brightness_tbl[i]
            = (uint32)(vee_brightness[i] + BRIGHTNESS_USER_VAL_OFFSET);
    }

    for(i=0; i<mddi_quickvx_vee_strength_tbl_size; i++)
    {
        mddi_quickvx_vee_strength_tbl[i]
            = VEE_STRENGTH_BIT_FIELD((uint32)vee_strength[i]);
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_GET_INFO

===========================================================================*/
static void mddi_quickvx_vee_get_info(
    uint32 req_bl_lv,
    struct mddi_quickvx_vee_value_s_type* info
)
{
    uint8 index = 0;

    if( (req_bl_lv != MDDI_BRIGHT_LEVEL_DEFAULT)
        &&(req_bl_lv >= BRIGHTNESS_VEE_ON_LV) )
    {
        index = req_bl_lv - BRIGHTNESS_VEE_ON_LV;

        info->on = ON;
        info->vee_bl_lv = mddi_quickvx_vee_brightness_tbl[index];
        info->vee_strength = mddi_quickvx_vee_strength_tbl[index];

    }
    else
    {
        info->on = OFF;
        info->vee_bl_lv = req_bl_lv;
        info->vee_strength = 0;
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_AL_CTRL

===========================================================================*/
static void mddi_quickvx_set_al_mode( struct msm_fb_data_type *mfd, uint32 al_mode )
{
    struct mddi_quickvx_vee_value_s_type info;

    if( al_mode )
    {/* Auto Brightness => ON */
        DISP_LOCAL_LOG_EMERG("DISP: mddi_quickvx_set_al_mode ON\n");

        mddi_quickvx_al_state = ON;

        mddi_quickvx_vee_get_info( mddi_local_state.mddi_brightness_level,
                                   &info );

        if(ON == info.on)
        {/* VEE ON */
            mfd->bl_level = mddi_local_state.mddi_brightness_level;
            mddi_quickvx_lcd_set_backlight(mfd);
        }
    }
    else
    {/* Auto Brightness => OFF */
        DISP_LOCAL_LOG_EMERG("DISP: mddi_quickvx_set_al_mode OFF\n");

        (void)mddi_quickvx_vee_ctrl( 0, MDDI_QUICKVX_VEE_DO_SEQ );

        mddi_quickvx_al_state = OFF;
        mfd->bl_level = mddi_local_state.mddi_brightness_level;
        mddi_quickvx_lcd_set_backlight(mfd);
    }

    return;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_VEE_CTRL

===========================================================================*/
static uint32 mddi_quickvx_vee_ctrl(
    uint32 bl_lv,
    mddi_quickvx_vee_ctrl_e_type ctrl
)
{
    uint32 bl_lv_new;
    struct mddi_quickvx_vee_value_s_type info;

    DISP_LOCAL_LOG_EMERG("DISP: mddi_quickvx_vee_ctrl bl lv[%d]/ctrl[%d]\n",
             (int)bl_lv,(int)ctrl);

    if( ON == mddi_quickvx_al_state )
    {/* al on */
        mddi_quickvx_vee_get_info( bl_lv, &info );

        if( ON == info.on )
        {/* VEE ON */
            bl_lv_new = info.vee_bl_lv;

            if( (ctrl == MDDI_QUICKVX_VEE_DO_SEQ)
                 && (ON == mddi_quickvx_vee_state) )
            {/* change only sterngth */
                mddi_quickvx_vee_seq( ON,
                                      info.vee_strength,
                                      MDDI_QUICKVX_VEE_STRENGTH_ONLY );
            }
            else if( (ctrl == MDDI_QUICKVX_VEE_DO_SEQ)
                      && (OFF == mddi_quickvx_vee_state) )
            {
                mddi_quickvx_vee_seq( ON, info.vee_strength, 0 );
                mddi_quickvx_vee_state = ON;
            }
        }
        else
        {/* VEE OFF */
            bl_lv_new = bl_lv;

            if( (ctrl == MDDI_QUICKVX_VEE_DO_SEQ)
                 && (ON == mddi_quickvx_vee_state) )
            {
                mddi_quickvx_vee_seq( OFF, 0, 0 );
                mddi_quickvx_vee_state = OFF;
            }
        }
    }
    else
    {/* al off */
        bl_lv_new = bl_lv;
    }

    DISP_LOCAL_LOG_EMERG("DISP: mddi_quickvx_vee_ctrl new bl[%d]\n",(int)bl_lv_new);

    return bl_lv_new;
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_PANEL_ON

DESCRIPTION
  Display ON Sequence

DEPENDENCIES
  None

RETURN VALUE
  void

SIDE EFFECTS
  None

===========================================================================*/
void mddi_quickvx_panel_on( void )
{
    int32  i = 0;

    int32 level = 0;

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_panel_on S\n");
    
    /* Panel Condition Set  */
    ql_send_spi_data(0x000000F8);
    ql_send_spi_data(0x00000101);
    ql_send_spi_data(0x00000127);
    ql_send_spi_data(0x00000127);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x00000154);
    ql_send_spi_data(0x0000019F);
    ql_send_spi_data(0x00000163);
    ql_send_spi_data(0x00000186);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x0000010D);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);

    /* Display Condition Set */
    ql_send_spi_data(0x000000F2);
    ql_send_spi_data(0x00000102);
    ql_send_spi_data(0x00000103);
    ql_send_spi_data(0x0000011C);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x000000F7);
    ql_send_spi_data(0x00000103);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);

    if( mddi_local_state.mddi_brightness_level>MDDI_BRIGHT_LEVEL_MAX )
    {
        mddi_local_state.mddi_brightness_level = MDDI_BRIGHT_LEVEL_MAX;
    }

    level = mddi_quickvx_vee_ctrl(
                mddi_local_state.mddi_brightness_level,
                MDDI_QUICKVX_VEE_NON_SEQ );

    for ( i = 0; i < MDDI_GAMMA_SET_MAX_NUM; i++ )
    {
        ql_send_spi_data(
          mddi_blc_gamma_table[level][i] );
    }

    /* Gamma Set Update */
    ql_send_spi_data(0x000000FA);
    ql_send_spi_data(0x00000103); 

    /* Etc Correction Set */
    ql_send_spi_data(0x000000F6);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x0000018E);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x000000B3);
    ql_send_spi_data(0x0000016C);
    ql_send_spi_data(0x000000B5);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000B6);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x000000B7);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000B8);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x000000B9);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000BA);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);

    /* Stand-by Off Command */
    ql_send_spi_data(0x00000011);
    /* 120ms Wait */
    mddi_wait(120);

    /* Display On Command */
    ql_send_spi_data(0x00000029);

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_panel_on E\n");

}


/*===========================================================================

FUNCTION  MDDI_QUICKVX_PANEL_OFF

===========================================================================*/
void mddi_quickvx_panel_off( void )
{
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_panel_off S\n");

    /* Display Off */
    ql_mddi_write(QUICKVX_SPI_TLEN_REG, 0x00000008);
    ql_send_spi_data(0x00000028);
    /* 10ms Wait */
    mddi_wait(10);

    /* Sleep in */
    ql_mddi_write(QUICKVX_SPI_TLEN_REG, 0x00000008);
    ql_send_spi_data(0x00000010);
    /* 120ms Wait */
    mddi_wait(120);

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_panel_off E\n");
}

/* Global Variables */
static uint32 mddi_quickvx_rows_per_second;
static uint32 mddi_quickvx_usecs_per_refresh;
static uint32 mddi_quickvx_rows_per_refresh;

/*===========================================================================

FUNCTION  MDDI_QUICKVX_CONFIGRE_REGISTERS

===========================================================================*/
void mddi_quickvx_configure_registers(void)
{
    uint32 result = MDDI_LOCAL_CRC_OK;
    uint32 loop;
    uint32 count; 
    int32  retry_cnt;

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_configure_registers S\n");

    /* 100ms Wait */
    mddi_wait(100);
    for(loop = 0; loop < MDDI_LOCAL_CRC_ERR_CHECK_RETRY; loop++)
    {
        /* VLP Mode out (GPIO17 LO) */
        gpio_set_value(MDDI_VLP, 0);
        /* 1ms Wait */
        mddi_wait(1);
        /* VLP Mode out (GPIO17 HI) */
        gpio_set_value(MDDI_VLP, 1);
        /* 1ms Wait */
        mddi_wait(1);

        /* Hardware Reset (GPIO133 LO) */
        gpio_set_value(MDDI_RST_N, 0);
        /* 1ms Wait */
        mddi_wait(1);
        /* Hardware Reset Relese (GPIO133 HI) */
        gpio_set_value(MDDI_RST_N, 1);
        /* 10ms Wait */
        mddi_wait(10);

        if ( NULL != i2c_quick_client )
        {
            /* Skew Calibration Control */
            for ( count = 0; count < MDDI_SKEW_CONTROL_MAX; count++ )
            {
                i2c_master_send(i2c_quick_client,
                                (char*)mddi_skew_buff[count],
                                sizeof(uint32)*2);
            }

            /* 1ms Wait */
            mddi_wait(1);
            /* Hardware Reset (GPIO133 LO) */
            gpio_set_value(MDDI_RST_N, 0);
            /* 1ms Wait */
            mddi_wait(1);
            /* Hardware Reset Relese (GPIO133 HI) */
            gpio_set_value(MDDI_RST_N, 1);
            /* 10ms Wait */
            mddi_wait(10);
        }

        mddi_quickvx_get_revision();
        /* HIBERNATION CONTROL REGISTER */
        mddi_quickvx_i2c_write(QUICKVX_HIBERNATION_CONTROL_REG, 0x00000001);
        mddi_quickvx_wait_block_switch();

        mddi_set_hibernation_to_active();
        /* Hibernation Transfer Delay Register */
        mddi_quickvx_i2c_write(QUICKVX_HIBERNATION_TRANS_D_REG, 0x0000000F);
        /* Hibernation Line Delay Register */
        mddi_quickvx_i2c_write(QUICKVX_HIBERNATION_LINE_D_REG, 0x00000001);
        /* Global Clock Register */
        mddi_quickvx_i2c_write(QUICKVX_CAR_ASSP_GCE_REG, 0x000003EF);
        /* 1ms Wait */
        mddi_wait(1);
        /* GPIO17 LO */
        gpio_set_value(MDDI_VLP, 0);

        /* Clock Selection Register */
        ql_mddi_write(QUICKVX_CAR_CLKSEL_REG, 0x0000700A);
        /* PLL Control Register */
        ql_mddi_write(QUICKVX_CAR_PLLCTRL_REG, 0x00000008);
        /* PLL CLK Ratio Register */
        ql_mddi_write(QUICKVX_CAR_PLLCLKRATIO_REG, 0x000DF077);
        /* PWM Width Register */
        ql_mddi_write(QUICKVX_RCB_PWMW_REG, 0x0000FFFF);
        /* PWM Control Register */
        ql_mddi_write(QUICKVX_RCB_PWMC_REG, 0x00000001);
        /* Configuration Done Register */
        ql_mddi_write(QUICKVX_RCB_CONFDONE_REG, 0x00000000);

        /* VEE Config Register */
        ql_mddi_write(QUICKVX_RCB_VEECONF_REG, 0x00001FF8);
        /* Global Clock Register */
        ql_mddi_write(QUICKVX_CAR_ASSP_GCE_REG, 0x000003EF);
        /* TCON Timing0 Register */
        ql_mddi_write(QUICKVX_RCB_TCON0_REG, 0x031F01DF);
        /* TCON Timing1 Register */
        ql_mddi_write(QUICKVX_RCB_TCON1_REG, 0x01A001FB);
        /* TCON Timing2 Register */
        ql_mddi_write(QUICKVX_RCB_TCON2_REG, 0x00000021);
        /* Display Attributes Register */
        ql_mddi_write(QUICKVX_RCB_DISPATTR_REG, 0x00000000);
        /* Video parameter Register */
        ql_mddi_write(QUICKVX_RCB_VPARM_REG, 0x00000888);
        /* Image Effect Register */
        ql_mddi_write(QUICKVX_RCB_IER_REG, 0x00000000);
        /* CellRAM Bus Configuration Register */
        ql_mddi_write(QUICKVX_RCB_CELLBCR_REG, 0x8008746F);
        /* CellRAM Configuration RCR Register */
        ql_mddi_write(QUICKVX_RCB_RCR_REG, 0x80000010);
        /* CellRAM Configuration Control Register */
        ql_mddi_write(QUICKVX_RCB_CELLCC_REG, 0x800000A3);
        /* Configuration Done Register */
        ql_mddi_write(QUICKVX_RCB_CONFDONE_REG, 0x00000001);
        /* VEE Control Register 0 */
        ql_mddi_write(QUICKVX_VEE_VEECTRL_REG, 0x00000001);
        /* VEE Compensation Register */
        ql_mddi_write(QUICKVX_VEE_COMPENSATION_REG, 0x00000000);
        /* Video Enhancement Enable Register */
        ql_mddi_write(QUICKVX_VEE_VDOEEN_REG, 0x00070007);

        for ( retry_cnt=0; 
              retry_cnt<MDDI_LOCAL_CRC_CHECK_RETRY_LIMIT ;
              retry_cnt++ )
        {
            result = mddi_local_crc_error_check();
            if ( MDDI_LOCAL_CRC_NO_CHK != result )
            {
                break;
            }
            mddi_wait(1);
        }

        if(MDDI_LOCAL_CRC_OK == result)
        {
            break;
        }
        else
        {
            /* GPIO17 HI */
            gpio_set_value( MDDI_VLP, 1);
            /* 1ms Wait */
            mddi_wait(1);
            /* Global Clock Register */
            mddi_quickvx_i2c_write(QUICKVX_CAR_ASSP_GCE_REG, 0x000001EF);
            /* 1ms wait */
            mddi_wait(1);
        }
    }
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_configure_registers E\n");
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_PRIM_LCD_INIT

===========================================================================*/
void mddi_quickvx_prim_lcd_init( struct msm_fb_data_type *mfd )
{
    uint32 result = MDDI_LOCAL_CRC_OK;
    uint32 loop;
    int32  retry_cnt;
    struct mddi_quickvx_vee_value_s_type info;

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_prim_lcd_init S\n");

    mddi_quickvx_vee_state = OFF;

    mddi_quickvx_vee_get_info( mddi_local_state.mddi_brightness_level, &info );
    if( (ON == mddi_quickvx_al_state)
        &&(ON == info.on) )
    {
        mddi_quickvx_vee_seq( ON, info.vee_strength, 0 );
        mddi_quickvx_vee_state = ON;
    }
    else
    {
        mddi_quickvx_vee_seq( OFF, 0, 0 );
        mddi_quickvx_vee_state = OFF;
    }

    /* Control Register */
    ql_mddi_write(QUICKVX_FB_A2F_LCD_RESET_REG, 0x00000003);

    for(loop = 0; loop < MDDI_LOCAL_CRC_ERR_CHECK_RETRY; loop++)
    {

        /* 1ms Wait */
        mddi_wait(1);
        /* Control Register */
        ql_mddi_write(QUICKVX_FB_A2F_LCD_RESET_REG, 0x00000002);
        /* 1ms Wait */
        mddi_wait(1);
        /* Control Register */
        ql_mddi_write(QUICKVX_FB_A2F_LCD_RESET_REG, 0x00000003);
        /* 10ms Wait */
        mddi_wait(10);
        /* SPI Transfer Length Register */
        ql_mddi_write(QUICKVX_SPI_TLEN_REG, 0x00000008);
        /* ALL PIXELS OFF */
        ql_send_spi_data(0x00000022);
        /* Display ON Sequence */
        mddi_quickvx_panel_on();

        mddi_refresh_force_flg = TRUE;
        mdp_refresh_screen_at_once( mfd );
        mddi_refresh_force_flg = FALSE;

        /* Control Register */
        ql_mddi_write(QUICKVX_FB_A2F_LCD_RESET_REG, 0x00000001);

        for ( retry_cnt=0; 
              retry_cnt<MDDI_LOCAL_CRC_CHECK_RETRY_LIMIT ;
              retry_cnt++ )
        {
            result = mddi_local_crc_error_check();

            if ( MDDI_LOCAL_CRC_NO_CHK != result )
            {
                break;
            }
            mddi_wait(1);

        }

        if(MDDI_LOCAL_CRC_OK == result)
        {
            break;
        }
    }

    /* normal mode ON */
    ql_send_spi_data(0x00000013);

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_prim_lcd_init E\n");

}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_DISPLAY_OFF

===========================================================================*/
void mddi_quickvx_display_off( void )
{
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_display_off S\n");

    mddi_set_hibernation_to_active();

    /* All Pixel OFF */
    ql_send_spi_data(0x00000022);

    /* 40ms Wait test */
    mddi_wait(40);

    /* OLED Power OFF Sequence */
    mddi_quickvx_panel_off();

    /* Control Register */
    ql_mddi_write(QUICKVX_FB_A2F_LCD_RESET_REG, 0x00000003);

    /* 1ms Wait */
    mddi_wait(1);

    /* CellRAM Configuration RCR Register */
    ql_mddi_write(QUICKVX_RCB_RCR_REG, 0x80000000);

    mddi_set_auto_hibernation();

    /* 1ms Wait */
    mddi_wait(1);

    /* GPIO17 HI */
    gpio_set_value(MDDI_VLP, 1);
    /* 1ms Wait */
    mddi_wait(1);
    /* Global Clock Register */
    mddi_quickvx_i2c_write(QUICKVX_CAR_ASSP_GCE_REG, 0x000001EF);
    /* 1ms Wait */
    mddi_wait(1);

    /* Hardware Reset (GPIO133 LO) */
    gpio_set_value(MDDI_RST_N, 0);

    /* 1ms Wait */
    mddi_wait(1);

    /* VLP Mode out (GPIO17 LO) */
    gpio_set_value(MDDI_VLP, 0);

    /* 1ms Wait */
    mddi_wait(1);

    if ( FALSE != mddi_local_state.mddi_first_dispon_flg )
    {
        /* MDDI SYS CLK OFF */
        pmapp_display_clock_config(0);
    }

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_display_off E\n");

}

/* Function to Power On the Primary and Secondary LCD panels */
static int mddi_quickvx_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_on S\n");
	mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		MDDI_MSG_DEBUG("\n mddi_quickvx_lcd_on: Device not found!");
		return -ENODEV;
	}

	if (mfd->key != MFD_KEY) {
		MDDI_MSG_DEBUG("\n mddi_quickvx_lcd_on: Invalid MFD key!");
		return -EINVAL;
	}

	mddi_host_client_cnt_reset();

    down(&disp_local_mutex);

    /* MDDI SYS CLK ON */
    pmapp_display_clock_config(1);

    if ( FALSE == mddi_local_state.mddi_first_dispon_flg )
    {
        mddi_quickvx_display_off();
        mddi_local_state.disp_state = MDDI_LOCAL_DISPLAY_OFF;
        mfd->panel_info.lcd.rev = 2;
        mddi_client_type = 2;
        mddi_local_state.mddi_first_dispon_flg = TRUE;
    }

    /* MDDI initialize setting Sequence */
    mddi_quickvx_configure_registers();

    /* MDDI Display ON Sequence */
/*  mddi_quickvx_prim_lcd_init(); */
    mddi_quickvx_prim_lcd_init( mfd );

    /* Display ON maintain state */
    mddi_local_state.disp_state = MDDI_LOCAL_DISPLAY_ON;

    mddi_set_auto_hibernation();

    up(&disp_local_mutex);
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_on E\n");

	return 0;
}

/* Function to Power Off the Primary and Secondary LCD panels */
static int mddi_quickvx_lcd_off(struct platform_device *pdev)
{
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_off S\n");

    down(&disp_local_mutex);
    mddi_host_client_cnt_reset();

    /* MDDI Display OFF Sequence */
    mddi_quickvx_display_off();

    /* Display OFF maintain state */
    mddi_local_state.disp_state = MDDI_LOCAL_DISPLAY_OFF;

    up(&disp_local_mutex);
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_off E\n");

	return 0;
}

/* Function to set the Backlight brightness level */
static void mddi_quickvx_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
    uint32 level = 0;
    uint32 level_internal = 0;
    int32 i     = 0;
    uint32 result = MDDI_LOCAL_CRC_OK;
    uint32 loop;
    int32  retry_cnt;

	MDDI_MSG_DEBUG("%s(): ", __func__);

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_set_backlight S\n");
    down(&disp_local_mutex);

    mddi_set_hibernation_to_active();

    level = mfd->bl_level;

    if ( level <= MDDI_BRIGHT_LEVEL_MAX )
    {
        /* disp on state */
        if ( MDDI_LOCAL_DISPLAY_ON == mddi_local_state.disp_state )
        {

            DISP_LOCAL_LOG_EMERG("DISP Backlight Req Level:[%d]\n",level);

            level_internal = mddi_quickvx_vee_ctrl( level,
                                                    MDDI_QUICKVX_VEE_DO_SEQ );

            DISP_LOCAL_LOG_EMERG("DISP Backlight Set Level:[%d]\n",level_internal);

            for(loop = 0; loop < MDDI_LOCAL_CRC_ERR_CHECK_RETRY; loop++)
            {

                /* Display Condition Set */
                for ( i = 0; i < MDDI_GAMMA_SET_MAX_NUM; i++ )
                {
                    ql_send_spi_data( mddi_blc_gamma_table[level_internal][i] );
                }

                for ( retry_cnt=0; 
                      retry_cnt<MDDI_LOCAL_CRC_CHECK_RETRY_LIMIT ;
                      retry_cnt++ )
                {
                    result = mddi_local_crc_error_check();

                    if ( MDDI_LOCAL_CRC_NO_CHK != result )
                    {
                        break;
                    }
                    mddi_wait(1);
                }

                if(MDDI_LOCAL_CRC_OK == result)
                {
                    break;
                }
            }
            /* Gamma Set Update */
            ql_send_spi_data( 0x000000FA );
            ql_send_spi_data( 0x00000103 ); 
        }
        mddi_local_state.mddi_brightness_level = level;
    }

    mddi_set_auto_hibernation();

    up(&disp_local_mutex);
    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_lcd_set_backlight E\n");

}

#ifdef MSMFB_GAMMA_GET_NV
static void mddi_quickvx_bl_nv_copy( uint32* to_data, uint8* from_data )
{
    to_data[2]  = from_data[0]  | 0x0100;
    to_data[3]  = from_data[1]  | 0x0100;
    to_data[4]  = from_data[2]  | 0x0100;
    to_data[5]  = from_data[3]  | 0x0100;
    to_data[6]  = from_data[4]  | 0x0100;
    to_data[7]  = from_data[5]  | 0x0100;
    to_data[8]  = from_data[6]  | 0x0100;
    to_data[9]  = from_data[7]  | 0x0100;
    to_data[10] = from_data[8]  | 0x0100;
    to_data[11] = from_data[9]  | 0x0100;
    to_data[12] = from_data[10] | 0x0100;
    to_data[13] = from_data[11] | 0x0100;
    to_data[14] = from_data[12] | 0x0100;
    to_data[15] = from_data[13] | 0x0100;
    to_data[16] = from_data[14] | 0x0100;
    to_data[18] = from_data[15] | 0x0100;
    to_data[20] = from_data[16] | 0x0100;
    to_data[22] = from_data[17] | 0x0100;
}
#endif /* MSMFB_GAMMA_GET_NV */

static void mddi_quickvx_lcd_set_nv( struct fb_nv_data* nv_data )
{
#ifdef MSMFB_GAMMA_GET_NV
    mddi_quickvx_bl_nv_copy( mddi_BLC_tableOff, nv_data->brightness_off  );
    mddi_quickvx_bl_nv_copy( mddi_BLC_tableDim, nv_data->brightness_dim  );
    mddi_quickvx_vee_nv_copy( nv_data->vee_brightness,nv_data->vee_strength);
    mddi_quickvx_bl_nv_copy( mddi_blc_table00, nv_data->brightness_0  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table01, nv_data->brightness_1  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table02, nv_data->brightness_2  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table03, nv_data->brightness_3  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table04, nv_data->brightness_4  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table05, nv_data->brightness_5  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table06, nv_data->brightness_6  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table07, nv_data->brightness_7  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table08, nv_data->brightness_8  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table09, nv_data->brightness_9  );
    mddi_quickvx_bl_nv_copy( mddi_blc_table10, nv_data->brightness_10 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table11, nv_data->brightness_11 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table12, nv_data->brightness_12 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table13, nv_data->brightness_13 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table14, nv_data->brightness_14 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table15, nv_data->brightness_15 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table16, nv_data->brightness_16 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table17, nv_data->brightness_17 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table18, nv_data->brightness_18 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table19, nv_data->brightness_19 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table20, nv_data->brightness_20 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table21, nv_data->brightness_21 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table22, nv_data->brightness_22 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table23, nv_data->brightness_23 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table24, nv_data->brightness_24 );
    mddi_quickvx_bl_nv_copy( mddi_blc_table25, nv_data->brightness_25 );
#endif /* MSMFB_GAMMA_GET_NV */
}

/*===========================================================================

FUNCTION  MDDI_QUICKVX_REFRESH

DESCRIPTION
  Refresh Sequence

DEPENDENCIES
  None

RETURN VALUE
  void

SIDE EFFECTS
  None

===========================================================================*/
void mddi_quickvx_refresh( unsigned int cmd )
{
    int32  i = 0;
    int32 level = 0;

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_refresh S\n");

    if( cmd == MDDI_REFREH_SEQ_ALL )
    {
        /* Clock Selection Register */
        ql_mddi_write(QUICKVX_CAR_CLKSEL_REG, 0x00007000);

        /* Configuration Done Register */
        ql_mddi_write(QUICKVX_RCB_CONFDONE_REG, 0x00000000);

        /* TCON Timing0 Register */
        ql_mddi_write(QUICKVX_RCB_TCON0_REG, 0x031F01DF);
        /* TCON Timing1 Register */
        ql_mddi_write(QUICKVX_RCB_TCON1_REG, 0x01A001FB);
        /* TCON Timing2 Register */
        ql_mddi_write(QUICKVX_RCB_TCON2_REG, 0x00000021);

        /* Display Attributes Register */
        ql_mddi_write(QUICKVX_RCB_DISPATTR_REG, 0x00000000);

        /* Video parameter Register */
        ql_mddi_write(QUICKVX_RCB_VPARM_REG, 0x00000888);

        /* Image Effect Register */
        ql_mddi_write(QUICKVX_RCB_IER_REG, 0x00000000);

        /* Configuration Done Register */
        ql_mddi_write(QUICKVX_RCB_CONFDONE_REG, 0x00000001);

        mddi_quickvx_vee_state = OFF;
        (void)mddi_quickvx_vee_ctrl(
                  mddi_local_state.mddi_brightness_level,
                  MDDI_QUICKVX_VEE_DO_SEQ );

        /* SPI Transfer Length Register */
        ql_mddi_write(QUICKVX_SPI_TLEN_REG, 0x00000008);
    }

    /* Panel Condition Set  */
    ql_send_spi_data(0x000000F8);
    ql_send_spi_data(0x00000101);
    ql_send_spi_data(0x00000127);
    ql_send_spi_data(0x00000127);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x00000154);
    ql_send_spi_data(0x0000019F);
    ql_send_spi_data(0x00000163);
    ql_send_spi_data(0x00000186);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x0000010D);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);

    /* Display Condition Set */
    ql_send_spi_data(0x000000F2);
    ql_send_spi_data(0x00000102);
    ql_send_spi_data(0x00000103);
    ql_send_spi_data(0x0000011C);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x000000F7);
    ql_send_spi_data(0x00000103);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);

    /* Display Condition Set */
    level = mddi_quickvx_vee_ctrl(
                mddi_local_state.mddi_brightness_level,
                MDDI_QUICKVX_VEE_NON_SEQ );

    for ( i = 0; i < MDDI_GAMMA_SET_MAX_NUM; i++ )
    {
        ql_send_spi_data(
          mddi_blc_gamma_table[level][i] );
    }

    /* Gamma Set Update */
    ql_send_spi_data(0x000000FA);
    ql_send_spi_data(0x00000103);

    /* Etc Correction Set */
    ql_send_spi_data(0x000000F6);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x0000018E);
    ql_send_spi_data(0x00000107);
    ql_send_spi_data(0x000000B3);
    ql_send_spi_data(0x0000016C);
    ql_send_spi_data(0x000000B5);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000B6);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x000000B7);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000B8);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x000000B9);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000112);
    ql_send_spi_data(0x0000010C);
    ql_send_spi_data(0x0000010A);
    ql_send_spi_data(0x00000110);
    ql_send_spi_data(0x0000010E);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x00000113);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x0000012A);
    ql_send_spi_data(0x00000124);
    ql_send_spi_data(0x0000011F);
    ql_send_spi_data(0x0000011B);
    ql_send_spi_data(0x0000011A);
    ql_send_spi_data(0x00000117);
    ql_send_spi_data(0x0000012B);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000013A);
    ql_send_spi_data(0x00000134);
    ql_send_spi_data(0x00000130);
    ql_send_spi_data(0x0000012C);
    ql_send_spi_data(0x00000129);
    ql_send_spi_data(0x00000126);
    ql_send_spi_data(0x00000125);
    ql_send_spi_data(0x00000123);
    ql_send_spi_data(0x00000121);
    ql_send_spi_data(0x00000120);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x0000011E);
    ql_send_spi_data(0x000000BA);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000100);
    ql_send_spi_data(0x00000111);
    ql_send_spi_data(0x00000122);
    ql_send_spi_data(0x00000133);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000144);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000155);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);
    ql_send_spi_data(0x00000166);

    DISP_LOCAL_LOG_EMERG("DISP mddi_quickvx_refresh E\n");
}

/* Driver Probe function */
static int __devinit mddi_quickvx_lcd_probe(struct platform_device *pdev)
{
	MDDI_MSG_DEBUG("\n%s(): id is %d", __func__, pdev->id);
	msm_fb_add_device(pdev);
	return 0;
}

/* Driver data structure */
static struct platform_driver this_driver = {
	.probe  = mddi_quickvx_lcd_probe,
	.driver	= {
		.name	= "mddi_quickvx",
	},
};


/* Primary LCD panel data structure */
static struct msm_fb_panel_data mddi_quickvx_panel_data0 = {
	.on					= mddi_quickvx_lcd_on,
	.off				= mddi_quickvx_lcd_off,
	.set_backlight		= mddi_quickvx_lcd_set_backlight,
    .set_nv             = mddi_quickvx_lcd_set_nv,
    .refresh            = mddi_quickvx_refresh,
    .set_al_mode        = mddi_quickvx_set_al_mode,
};


/* Primary LCD panel device structure */
static struct platform_device this_device0 = {
	.name   = "mddi_quickvx",
	.id		= MDDI_QUICKVX_1_2,
	.dev	= {
		.platform_data = &mddi_quickvx_panel_data0,
	}
};

static int i2c_quickvx_probe0(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
  i2c_quick_client = client;
  return 0;

}

static struct i2c_device_id i2c_quickvx_idtable[] = {
  { "i2c_quickvx", 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, i2c_quickvx_idtable);

static struct i2c_driver i2c_quickvx_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name  = "i2c_quickvx",
  },
  .id_table = i2c_quickvx_idtable,
  .probe    = i2c_quickvx_probe0,
};

/* Module init - driver main entry point */
static int __init mddi_quickvx_lcd_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
/* 	u32 cid; */
/* 	MDDI_MSG_DEBUG("\n%s(): ", __func__); */

/* 	ret = msm_fb_detect_client("mddi_quickvx"); */

/* 	if (ret == -ENODEV)	{ */
		/* Device not found */
/* 		MDDI_MSG_DEBUG("\n mddi_quickvx_lcd_init: No device found!"); */
/* 		return 0; */
/* 	} */

/* 	if (ret) { */
/* 		cid = mddi_get_client_id(); */

/* 		MDDI_MSG_DEBUG("\n cid = 0x%x", cid); */
/* 		if (((cid >> 16) != QUICKVX_MDDI_MFR_CODE) || */
/* 			((cid & 0xFFFF) != QUICKVX_MDDI_PRD_CODE)) { */
 			/* MDDI Client ID not matching */
/* 			MDDI_MSG_DEBUG("\n mddi_quickvx_lcd_init: " */
/* 				"Client ID missmatch!"); */

/* 			return 0; */
/* 		} */
/* 		MDDI_MSG_DEBUG("\n mddi_quickvx_lcd_init: " */
/* 			"QuickVX LCD panel detected!"); */
/* 	} */
#endif /* CONFIG_FB_MSM_MDDI_AUTO_DETECT */

/*	mddi_quickvx_rows_per_refresh = 872; */
/*	mddi_quickvx_rows_per_second = 52364; */
/*	mddi_quickvx_usecs_per_refresh = 16574; */
    mddi_quickvx_rows_per_refresh = 831;

/*    mddi_quickvx_rows_per_second = 48750; */
/*    mddi_quickvx_usecs_per_refresh = 17046; */
    mddi_quickvx_rows_per_second = 53625;
    mddi_quickvx_usecs_per_refresh = 15497;

	ret = platform_driver_register(&this_driver);

	if (!ret) {
		pinfo = &mddi_quickvx_panel_data0.panel_info;
		pinfo->xres = 480;
/*		pinfo->yres = 864; */
        pinfo->yres = 800;

		MSM_FB_SINGLE_MODE_PANEL(pinfo);
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 24;
		pinfo->fb_num = 2;

		pinfo->clk_rate = 192000000;
		pinfo->clk_min = 192000000;
		pinfo->clk_max = 200000000;
		pinfo->lcd.rev = 1;
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.refx100 = (mddi_quickvx_rows_per_second \
			* 100)/mddi_quickvx_rows_per_refresh;

/*		pinfo->lcd.v_back_porch = 4; */
/*		pinfo->lcd.v_front_porch = 2; */
        pinfo->lcd.v_back_porch = 1;
        pinfo->lcd.v_front_porch = 28;

		pinfo->lcd.v_pulse_width = 2;
        pinfo->lcd.hw_vsync_mode = TRUE;

		pinfo->lcd.vsync_notifier_period = (1 * HZ);
		pinfo->bl_max = 10;
		pinfo->bl_min = 0;

		ret = platform_device_register(&this_device0);
		if (ret) {
			platform_driver_unregister(&this_driver);
			MDDI_MSG_DEBUG("mddi_quickvx_lcd_init: "
				"Primary device registration failed!\n");
		}

    ret = i2c_add_driver(&i2c_quickvx_driver);

	}

    mddi_local_state.mddi_brightness_level = MDDI_BRIGHT_LEVEL_DEFAULT;

	return ret;
}

module_init(mddi_quickvx_lcd_init);

