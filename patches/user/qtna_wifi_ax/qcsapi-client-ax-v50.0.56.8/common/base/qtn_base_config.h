/*
 * (C) Copyright 2010 - 2018 Quantenna Communications Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __QTN_BASE_CONFIG_H
#define __QTN_BASE_CONFIG_H

#include "qtn_base_current_platform.h"

/*
 * According to ASIC team feedback, the normal register mapping(a) should always be
 * the 1st choice as alias one (b) is not fully verified in the history.
 */
#define TOPAZ_ALIAS_MAP_SWITCH(a, b)	(a)

/* Fixed phy addresses */
#define PEARL_PHY0_ADDR			0
#define PEARL_PHY0_ADDR2		1
#define PEARL_PHY0_ADDR3		3
#define PEARL_PHY1_ADDR			4

/* Definition indicates that the platform is FPGA */
#if PEARL_FPGA_PLATFORM
	/* CLK speeds are in MHz and 1/10th the speed of actual ASIC */
	#define QTN_SERIAL_BAUD		38400
	#define QTN_APB_CLK		12500000

#ifdef PEARL_PLATFORM_C0
	#define QTN_AHB_CLK		32000000
	#define QTN_CPU_CLK		64000000
#else
	#define QTN_AHB_CLK		25000000
	#define QTN_CPU_CLK		50000000
#endif
#else
	#if defined(PEARL_VELOCE)
		#define QTN_SERIAL_BAUD	460800
	#else
		#define QTN_SERIAL_BAUD	115200
	#endif
	#define QTN_APB_CLK		125000000
	#define QTN_AHB_CLK		320000000
	#define QTN_CPU_CLK		640000000
#endif

/*
 * Re-use Ruby and Topaz defines to simplify the number of changes required
 * to compile new binaries for the current platform
 */
#define RUBY_SERIAL_BAUD		QTN_SERIAL_BAUD
#define TOPAZ_SERIAL_BAUD		QTN_SERIAL_BAUD
#define RUBY_FIXED_DEV_CLK		QTN_APB_CLK
#define RUBY_FIXED_CPU_CLK		QTN_CPU_CLK

#define CONFIG_SPIFLASH_DEV_CLK		QTN_AHB_CLK

#define DEFAULT_BOARD_ID		PLATFORM_DEFAULT_BOARD_ID

/* RGMII related defines */
#define CONFIG_ARCH_RGMII_DEFAULT	0x8F8F8F8F
#define CONFIG_ARCH_RGMII_BBIC5_EVK	0

/* EMAC related defines */

/* EMAC flags */
#define EMAC_NOT_IN_USE			(0)
#define EMAC_IN_USE			(BIT(0))
#define EMAC_PHY_NOT_IN_USE		(BIT(1))  // do not initialize/access phy mdio
#define EMAC_PHY_FORCE_10MB		(BIT(2))
#define EMAC_PHY_FORCE_100MB		(BIT(3))
#define EMAC_PHY_FORCE_1000MB		(BIT(4))
#define EMAC_PHY_FORCE_HDX		(BIT(5))
#define EMAC_PHY_RESET			(BIT(6)) // force PHY reset
#define EMAC_PHY_MII			(BIT(7)) // default is rgmii
#define EMAC_PHY_AR8236			(BIT(8))
#define	EMAC_MUST_INIT_RXAUI_PCS	(BIT(8))
#define EMAC_PHY_AR8327			(BIT(9))
#define EMAC_PHY_GPIO1_RESET		(BIT(10))
#define EMAC_PHY_GPIO13_RESET		(BIT(11))
#define EMAC_PHY_NO_COC			(BIT(12)) // do not adjust link speed for power savings
#define EMAC_PHY_MV88E6071		(BIT(13))
#define EMAC_PHY_FPGAA_ONLY		(BIT(15))
#define EMAC_PHY_FPGAB_ONLY		(BIT(16))
#define EMAC_PHY_RTL8363SB_P0		(BIT(18))
#define EMAC_PHY_RTL8363SB_P1		(BIT(19))
#define EMAC_BONDED			(BIT(20))
#define EMAC_PHY_RTL8365MB		(BIT(21))
#define EMAC_PHY_RTL8211DS		(BIT(22))
/* 10g emac interface parameter */
#define EMAC_SPEED_S_5G			(BIT(23))
#define EMAC_RXAUI			(BIT(24))
#define EMAC_SPEED_S_10G		(BIT(25))
#define EMAC_SPEED_S_2G5		(BIT(26))
#define EMAC_SGMII			(BIT(27))
#define EMAC_SGMII_AN			(BIT(28))
#define EMAC_PHY_CUSTOM			(BIT(31))

#define EMAC_PHY_AUTO_MASK		(EMAC_PHY_FORCE_10MB | EMAC_PHY_FORCE_100MB | EMAC_PHY_FORCE_1000MB | \
					EMAC_SPEED_S_10G | EMAC_SPEED_S_2G5)

#define EMAC_MV88E6071			(EMAC_IN_USE | EMAC_PHY_MII | EMAC_PHY_NOT_IN_USE |	\
						EMAC_PHY_NO_COC | EMAC_PHY_FORCE_100MB | EMAC_PHY_MV88E6071)
#define EMAC_SLOW_PHY			(EMAC_PHY_FORCE_10MB|EMAC_PHY_FORCE_100MB|EMAC_PHY_MII)

/* force phy addr scan */
#define EMAC_PHY_ADDR_SCAN		(32)	// scan bus for addr

/* Flash memory sizes */
#define FLASH_64MB			(64*1024*1024)
#define FLASH_32MB			(32*1024*1024)
#define FLASH_16MB			(16*1024*1024)
#define FLASH_8MB			(8*1024*1024)
#define FLASH_4MB			(4*1024*1024)
#define FLASH_2MB			(2*1024*1024)
#define FLASH_256KB			(256*1024)
#define FLASH_64KB			(64*1024)
#define DEFAULT_FLASH_SIZE		(FLASH_8MB)
#define FLASH_SIZE_JEDEC		(0)

/* DDR Type */
#define	DDR_32_MICRON		0
#define DDR_16_MICRON		1
#define DDR_16_ETRON		2
#define DDR_16_SAMSUNG		3
#define DDR_32_ETRON		4
#define DDR_32_SAMSUNG		5
#define DDR_16_HYNIX		6
#define DDR3_16_WINBOND		7
#define DDR3_32_WINBOND		8
#define DDR_NO_INIT		9
#define DEFAULT_DDR_CFG		(DDR3_16_WINBOND)

/* DDR Size */
#define DDR_256MB		(256*1024*1024)
#define DDR_128MB		(128*1024*1024)
#define DDR_64MB		(64*1024*1024)
#define DDR_46MB		(46*1024*1024)
#define DDR_32MB		(32*1024*1024)
#define DDR_AUTO		(0)
#define DEFAULT_DDR_SIZE	(DDR_256MB)

/* DDR Speed */
#define DDR3_1000MHz		1000
#define DDR3_854MHz		854
#define DDR3_800MHz		800
#define DDR3_640MHz		640
#define DDR3_500MHz		500
#define DDR3_400MHz		400
#define DDR3_320MHz		320
#define DDR_400			400
#define DDR_320			320
#define DDR_250			250
#define DDR_160			160
#define DEFAULT_DDR_SPEED	(DDR3_800MHz)

/* DDR ZQDIV defines */
#define DEFAULT_DDR_ZQDIV	(0x5b)
#define DEFAULT_DDR_MR1		(0xe)
#define DEFAULT_DDR_PARAM	(DEFAULT_DDR_ZQDIV | (DEFAULT_DDR_MR1 << 8))
#define DDR_ZQDIV_PARAM(x)	((x) & 0xff)
#define DDR_MR1_PARAM(x)	(((x) >> 8) & 0xffff)

/* SPI defines */
#define	SPI1_NOT_IN_USE		(0)
#define	SPI1_IN_USE		(BIT(0))

/* UART1 defines */
#define	UART1_NOT_IN_USE	0
#define	UART1_IN_USE		1

/* RFIC defines */
#define RFIC_REF_40		(40)
#define RFIC_REF_80		(80)

#define RFIC_NOT_IN_USE		0
#define RFIC_V4_IN_USE		4
#define RFIC_V6_IN_USE		6

/* PCIe defines */
#define PCIE_NOT_IN_USE				0
#define PCIE_IN_USE				(BIT(0))
#define PCIE_USE_PHY_LOOPBK			(BIT(1))
#define PCIE_RC_MODE				(BIT(2))
#define PCIE_GEN3				(BIT(3))
#define PCIE_LANE_NUMBER_X1			(BIT(4))
#define PCIE_CFG_TARGET_LINK_SPEED_VEC0		(BIT(6))
#define PCIE_CFG_TARGET_LINK_SPEED_VEC1		(BIT(7))
#define PCIE_CFG_TARGET_LINK_SPEED_VEC2		(BIT(8))
#define PCIE_CFG_TARGET_LINK_SPEED_MASK		(PCIE_CFG_TARGET_LINK_SPEED_VEC0 | \
						PCIE_CFG_TARGET_LINK_SPEED_VEC1 | \
						PCIE_CFG_TARGET_LINK_SPEED_VEC2)
#define PCIE_ENDPOINT				(PCIE_IN_USE | PCIE_USE_PHY_LOOPBK)
#define PCIE_ENDPOINT_GEN3			(PCIE_ENDPOINT | PCIE_GEN3)
#define PCIE_ENDPOINT_GEN3_X1			(PCIE_ENDPOINT_GEN3 | PCIE_LANE_NUMBER_X1)
#define PCIE_ROOTCOMPLEX			(PCIE_IN_USE | PCIE_RC_MODE | PCIE_USE_PHY_LOOPBK)

#define PCIE_CFG_2_TARGET_LINK_SPEED_S		(6)
#define PCIE_CFG_2_TARGET_LINK_SPEED(cfg)       (((cfg) & PCIE_CFG_TARGET_LINK_SPEED_MASK) >> \
						PCIE_CFG_2_TARGET_LINK_SPEED_S)
/* IO voltage control */
#define DEFAULT_IO_VOLTAGE_CTRL			(0x4)
#define DEFAULT_IO_VOLTAGE_CTRL_QSR5G		(0x5)
#define IO_VOLTAGE_CTRL_C0_EVK			(0xc)
#define IO_VOLTAGE_CTRL_U84P8			(0xaad)

#define IO_VOLT_CTRL_PCIE_VREG_BYPASS		(BIT(0))
#define EMACSS_SPARE0_MASK			(0xffe)

#if PEARL_FPGA_PLATFORM && (MAC_UNITS < 2)
/*
 * On PEARL(always), and on JADE(some builds), there is only one WMAC in FPGA builds.
 */
#define FPGA_SINGLE_WMAC	1
#else
#define FPGA_SINGLE_WMAC	0
#endif

#endif /* __QTN_BASE_CONFIG_H */
