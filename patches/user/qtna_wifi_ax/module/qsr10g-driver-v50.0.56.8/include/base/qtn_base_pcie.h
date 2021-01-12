/*
 * (C) Copyright 2015 - 2018 Quantenna Communications Inc.
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

#ifndef __QTN_BASE_PCIE_H
#define __QTN_BASE_PCIE_H

#define	PCIE_GEN2_BASE				(0xe9000000)
#define	PCIE_GEN3_BASE				(0xe7000000)

#define PCIE_HDP_CTRL(base)			((base) + 0x2c00)
#define PCIE_HDP_AXI_CTRL(base)			((base) + 0x2c04)
#define PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH	0xFFFF0000
#define PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH_S	16
#define PCIE_HDP_HOST_WR_DESC0(base)		((base) + 0x2c10)

#define PCIE_HDP_TX0_BASE_ADDR(base)		((base) + 0x2c60)/* TQE interface */
#define PCIE_HDP_TX1_BASE_ADDR(base)		((base) + 0x2c64)
#define PCIE_HDP_TX0_Q_CTRL(base)		((base) + 0x2c70)
#define PCIE_HDP_TX1_Q_CTRL(base)		((base) + 0x2c74)
#define PCIE_HDP_CFG0(base)			((base) + 0x2c80)
#define PCIE_HDP_CFG1(base)			((base) + 0x2c84)
#define PCIE_HDP_CFG2(base)			((base) + 0x2c88)
#define PCIE_HDP_CFG3(base)			((base) + 0x2c8c)
#define PCIE_HDP_CFG4(base)			((base) + 0x2c90)
#define PCIE_HDP_CFG5(base)			((base) + 0x2c94)
#define PCIE_INT(base)				((base) + 0x2cb0)
#define PCIE_HDP_HHBM_BUF_PTR(base)		((base) + 0x2d00)
#define PCIE_HDP_TX_HOST_Q_SZ_CTRL(base)	((base) + 0x2d2c)
#define PCIE_HDP_TX_HOST_Q_BASE_L(base)		((base) + 0x2d30)
#define PCIE_HDP_TX_HOST_Q_BASE_H(base)		((base) + 0x2d34)
#define PCIE_HDP_TX_HOST_Q_WR_PTR(base)		((base) + 0x2d38)

/* Host HBM pool registers */
#define PCIE_HHBM_CSR_REG(base)			((base) + 0x2e00)
#define PCIE_HHBM_Q_BASE_REG(base)		((base) + 0x2e04)
#define PCIE_HHBM_Q_LIMIT_REG(base)		((base) + 0x2e08)
#define PCIE_HHBM_Q_WR_REG(base)		((base) + 0x2e0c)
#define PCIE_HHBM_Q_RD_REG(base)		((base) + 0x2e10)
#define PCIE_HHBM_POOL_DATA_0_H(base)		((base) + 0x2e90)
#define PCIE_HHBM_CONFIG(base)			((base) + 0x2f9c)
#define PCIE_HHBM_POOL_REQ_0(base)		((base) + 0x2f10)
#define PCIE_HHBM_POOL_DATA_0(base)		((base) + 0x2f40)
#define PCIE_HHBM_WATERMARK_MASKED_INT(base)	((base) + 0x2f68)
#define PCIE_HHBM_WATERMARK_INT(base)		((base) + 0x2f6c)
#define PCIE_HHBM_POOL_WATERMARK(base)		((base) + 0x2f70)
#define PCIE_HHBM_POOL_OVERFLOW_CNT(base)	((base) + 0x2f90)
#define PCIE_HHBM_POOL_UNDERFLOW_CNT(base)	((base) + 0x2f94)

/* host HBM bit field definition */
#define HHBM_CONFIG_SOFT_RESET			(BIT(8))
#define HHBM_RD_REQ				(BIT(1))
#define HHBM_DONE				(BIT(31))

/* offsets for dual PCIE */
#define PCIE_PORT_LINK_CTL(base)		((base) + 0x0710)
#define PCIE_GEN2_CTL(base)			((base) + 0x080C)
#define PCIE_GEN3_OFF(base)			((base) + 0x0890)
#define PCIE_GEN3_EQ_CTRL_OFF(base)		((base) + 0x08a8)
#define PCIE_ATU_BASE_LOW(base)			((base) + 0x090C)
#define PCIE_ATU_BASE_HIGH(base)		((base) + 0x0910)
#define PCIE_ATU_BASE_LIMIT(base)		((base) + 0x0914)
#define PCIE_ATU_TGT_LOW(base)			((base) + 0x0918)
#define PCIE_ATU_TGT_HIGH(base)			((base) + 0x091C)
#define PCIE_ROM_MASK_ADDR(base)		((base) + 0x1030)

#define PCIE_ID(base)				((base) + 0x0000)
#define PCIE_CMD(base)				((base) + 0x0004)
#define PCIE_BAR(base, n)			((base) + 0x0010 + ((n) << 2))
#define PCIE_CAP_PTR(base)			((base) + 0x0034)
#define PCIE_MSI_CTRL(base)			((base) + 0x0050)
#define PCIE_MSI_ADDR_L(base)			((base) + 0x0054)
#define PCIE_MSI_MASK_BIT(base)			((base) + 0x0060)
#define PCIE_DEVCTLSTS(base)			((base) + 0x0078)

#define PCIE_CMDSTS(base)			((base) + 0x0004)
#define PCIE_LINK_STAT(base)			((base) + 0x80)
#define PCIE_LINK_CTL2(base)			((base) + 0xa0)
#define PCIE_LANE_EQ_CTRL1(base)		((base) + 0x154)
#define PCIE_CFG_SPACE_LIMIT(base)		((base) + 0x100)

/* PCIe link defines */
#define PEARL_PCIE_LINKUP			(0x7)

/* PCIe Lane defines */
#define PCIE_G2_LANE_X1				((BIT(0)) << 16)
#define PCIE_G2_LANE_X2				((BIT(0) | BIT(1)) << 16)

/* PCIe DLL link enable */
#define PCIE_DLL_LINK_EN			((BIT(0)) << 5)

#define PCIE_LINK_MODE(x)			(((x) >> 16) & 0x7)

/* PCIE_LINK_CTL2 */
#define PCIE_TARGET_LINK_SPEED_MASK		(0xf)

#define PCIE_BAR_MASK(base, n)			((base) + 0x1010 + ((n) << 2))
#define PCIE_MAX_BAR				(6)

#define PCIE_ATU_VIEW(base)			((base) + 0x0900)
#define PCIE_ATU_CTL1(base)			((base) + 0x0904)
#define PCIE_ATU_CTL2(base)			((base) + 0x0908)
#define PCIE_ATU_LBAR(base)			((base) + 0x090c)
#define PCIE_ATU_LAR(base)			((base) + 0x0914)
#define PCIE_ATU_LTAR(base)			((base) + 0x0918)
#define PCIE_ATU_UTAR(base)			((base) + 0x091c)

#define PCIE_MSI_ENABLE(base)			((base) + 0x0828)
#define PCIE_MSI_STATUS(base)			((base) + 0x0830)
#define PEARL_PCIE_MSI_REGION			(0xce000000)

#define PCIE_ATU_EN_REGION		(BIT(31))
#define PCIE_ATU_EN_MATCH		(BIT(30))

#define PCIE_CONFIG_REGION		(0xcf000000)

#define PCIE_MSI_REGION			(0xce000000)

#define PCIE_MAX_BAR			(6)

#define PCIE_LINK_DETECT_CTRL		(0xed006018)

#define PCIE0_PHY_CONFIG2		(0xed00601c)
#define PCIE0_PHY_VREG_BYPASS		(0x80000000)

#define QTN_CHIP_ID_MASK		(0xf0)
#define QTN_CHIP_ID_PEARL_B		(0x60)
#define QTN_CHIP_ID_PEARL_C		(0x70)

#endif /* __QTN_BASE_PCIE_H */
