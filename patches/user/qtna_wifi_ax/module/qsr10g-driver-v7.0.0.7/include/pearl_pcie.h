
/*
 * (C) Copyright 2015 Quantenna Communications Inc.
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
 */

#ifndef __PEARL_PCIE_H
#define __PEARL_PCIE_H

#include "base/qtn_base_pcie.h"

#define PCIE_HDP_HOST_WR_DESC0_H(base)		((base) + 0x2c14)
#define PCIE_HDP_HOST_WR_DESC1(base)		((base) + 0x2c18)
#define PCIE_HDP_HOST_WR_DESC1_H(base)		((base) + 0x2c1c)

#define PCIE_HDP_RX2_DESC_BASE_ADDR(base)	((base) + 0x2c20)
#define PCIE_HDP_RX2_DESC_BASE_ADDR_H(base)	((base) + 0x2c24)
#define PCIE_HDP_RX3_DESC_BASE_ADDR(base)	((base) + 0x2c28)
#define PCIE_HDP_RX3_DESC_BASE_ADDR_H(base)	((base) + 0x2c2c)

#define PCIE_HDP_RX_INT_CTRL(base)		((base) + 0x2c30)
#define PCIE_HDP_TX_INT_CTRL(base)		((base) + 0x2c34)
#define PCIE_HDP_INT_STATUS(base)		((base) + 0x2c38)
#define PCIE_HDP_INT_EN(base)			((base) + 0x2c3c)
#define PCIE_HDP_RX_DESC0_PTR(base)		((base) + 0x2c40)
#define PCIE_HDP_RX_DESC0_NOE(base)		((base) + 0x2c44)
#define PCIE_HDP_RX_DESC1_PTR(base)		((base) + 0x2c48)
#define PCIE_HDP_RX_DESC1_NOE(base)		((base) + 0x2c4c)
#define PCIE_HDP_RX_DESC2_PTR(base)		((base) + 0x2c50)
#define PCIE_HDP_RX_DESC2_NOE(base)		((base) + 0x2c54)
#define PCIE_HDP_RX_DESC3_PTR(base)		((base) + 0x2c58)
#define PCIE_HDP_RX_DESC3_NOE(base)		((base) + 0x2c5c)

#define PCIE_HDP_CFG6(base)			((base) + 0x2c98)
#define PCIE_HDP_CFG7(base)			((base) + 0x2c9c)
#define PCIE_HDP_CFG8(base)			((base) + 0x2ca0)
#define PCIE_HDP_CFG9(base)			((base) + 0x2ca4)
#define PCIE_HDP_CFG10(base)			((base) + 0x2ca8)
#define PCIE_HDP_CFG11(base)			((base) + 0x2cac)
#define PCIE_INT_MASK(base)			((base) + 0x2cb4)
#define PCIE_MSI_MASK(base)			((base) + 0x2cb8)
#define PCIE_MSI_PNDG(base)			((base) + 0x2cbc)
#define PCIE_PRI_CFG(base)			((base) + 0x2cc0)
#define PCIE_PHY_CR(base)			((base) + 0x2cc4)
#define PCIE_HDP_CTAG_CTRL(base)		((base) + 0x2cf4)
#define PCIE_HDP_HHBM_BUF_PTR(base)		((base) + 0x2d00)
#define PCIE_HDP_HHBM_BUF_PTR_H(base)		((base) + 0x2d04)
#define PCIE_HDP_HHBM_BUF_FIFO_NOE(base)	((base) + 0x2d04)
#define PCIE_HDP_RX0DMA_CNT(base)		((base) + 0x2d10)
#define PCIE_HDP_RX1DMA_CNT(base)		((base) + 0x2d14)
#define PCIE_HDP_RX2DMA_CNT(base)		((base) + 0x2d18)
#define PCIE_HDP_RX3DMA_CNT(base)		((base) + 0x2d1c)
#define PCIE_HDP_TX0DMA_CNT(base)		((base) + 0x2d20)
#define PCIE_HDP_TX1DMA_CNT(base)		((base) + 0x2d24)
#define PCIE_HDP_RXDMA_CTRL(base)		((base) + 0x2d28)
#define RXDMA_WPTR			(BIT(27))
#define RXDMA_FAST			(BIT(28))
#define RXDMA_NEW			(BIT(29))
#define RXDMA_INTERLEAVE		(BIT(30))

#define PCIE_HDP_TX_HOST_Q_SZ_CTRL(base)	((base) + 0x2d2c)
#define PCIE_HDP_TX_HOST_Q_BASE_L(base)		((base) + 0x2d30)
#define PCIE_HDP_TX_HOST_Q_BASE_H(base)		((base) + 0x2d34)
#define PCIE_HDP_TX_HOST_Q_WR_PTR(base)		((base) + 0x2d38)
#define PCIE_HDP_TX_HOST_Q_RD_PTR(base)		((base) + 0x2d3c)
#define PCIE_HDP_TX_HOST_Q_STS(base)		((base) + 0x2d40)

#define PCIE_HDP_RX2_DESC_Q_CTRL(base)		((base) + 0x2d80)
#define PCIE_HDP_RX2_DESC_Q_WR_PTR(base)	((base) + 0x2d84)
#define PCIE_HDP_RX2_DESC_FIFO_CTRL(base)	((base) + 0x2d88)
#define PCIE_HDP_RX2_DEV_PTR_ADDR(base)		((base) + 0x2dd8)
#define PCIE_HDP_RX2_DEV_PTR_ADDR_H(base)	((base) + 0x2ddc)

#define PCIE_HDP_RX3_DESC_Q_CTRL(base)		((base) + 0x2d90)
#define PCIE_HDP_RX3_DESC_Q_WR_PTR(base)	((base) + 0x2d94)
#define PCIE_HDP_RX3_DESC_FIFO_CTRL(base)	((base) + 0x2d98)
#define PCIE_HDP_RX3_DEV_PTR_ADDR(base)		((base) + 0x2fb4)
#define PCIE_HDP_RX3_DEV_PTR_ADDR_H(base)	((base) + 0x2fb8)

#define PCIE_HDP_TX0_DESC_Q_CTRL(base)		((base) + 0x2da0)
#define PCIE_HDP_TX0_DESC_Q_WR_PTR(base)	((base) + 0x2da4)
#define PCIE_HDP_TX0_DESC_BASE_ADDR(base)	((base) + 0x2dac)
#define PCIE_HDP_TX0_DESC_BASE_ADDR_H(base)	((base) + 0x2da8)
#define PCIE_HDP_TX_DMA_CTRL(base)		((base) + 0x2dcc)
#define TX_DMA_DISABLE			(BIT(0))
#define TX_HW_OWN_CLEAR			(BIT(1))
#define TX_CLEAR_ALL			(BIT(2))
#define TXDMA_NEW			(BIT(8))

#define PCIE_HDP_TX0_DESC_HQ_STS(base)		((base) + 0x2fbc)
#define PCIE_HDP_TX0_DESC_HQ_PTR(base)		((base) + 0x2df4)
#define PCIE_HDP_TX0_DEV_PTR_ADDR(base)		((base) + 0x2db0)
#define PCIE_HDP_TX0_DEV_PTR_ADDR_H(base)	((base) + 0x2db4)
#define PCIE_HDP_TX0_DESC_FIFO_CTRL(base)	((base) + 0x2db8)

#define PCIE_HDP_TX1_DESC_Q_CTRL(base)		((base) + 0x2fc0)
#define PCIE_HDP_TX1_DESC_Q_WR_PTR(base)	((base) + 0x2fc4)
#define PCIE_HDP_TX1_DESC_BASE_ADDR(base)	((base) + 0x2fc8)
#define PCIE_HDP_TX1_DESC_BASE_ADDR_H(base)	((base) + 0x2fcc)
#define PCIE_HDP_TX1_DESC_HQ_STS(base)		((base) + 0x2df8)
#define PCIE_HDP_TX1_DESC_HQ_PTR(base)		((base) + 0x2dfc)
#define PCIE_HDP_TX1_DEV_PTR_ADDR(base)		((base) + 0x2fd0)
#define PCIE_HDP_TX1_DEV_PTR_ADDR_H(base)	((base) + 0x2fd4)
#define PCIE_HDP_TX1_DESC_FIFO_CTRL(base)	((base) + 0x2fd8)

#define PCIE_HDP_AXI_MASTER_CTRL(base)		((base) + 0x2de0)
#define PCIE_HDP_AXI_EN_BURST32_READ		((1 << 3) | (1 << 7))
#define PCIE_HDP_AXI_EN_BURST32_WRITE		(1 << 11)
#define PCIE_HDP_AXI_BURST32_SIZE		(32 * 8)

/* Host HBM pool registers */
#define PCIE_HHBM_WATERMARK_MASKED_INT(base)	((base) + 0x2f68)
#define PCIE_HHBM_WATERMARK_INT(base)		((base) + 0x2f6c)
#define PCIE_HHBM_POOL_WATERMARK(base)		((base) + 0x2f70)
#define PCIE_HHBM_POOL_OVERFLOW_CNT(base)	((base) + 0x2f90)
#define PCIE_HHBM_POOL_UNDERFLOW_CNT(base)	((base) + 0x2f94)

#define PCIE_HDP_RX2_TIMOUT_THLD(base)		((base) + 0x2fe4)
#define PCIE_HDP_FRM_CNT_THLD(base)		((base) + 0x2fdc)
#define HDP_FRM_RX2_THLD(val, th)		(((val) & ~(0xff << 16)) | ((th & 0xff) << 16))
#define HDP_FRM_TX0_THLD(val, th)		(((val) & ~(0xff)) | ((th & 0xff)))
#define PCIE_HDP_TX0_TIMOUT_THLD(base)		((base) + 0x2fec)

/* PCIE_CFG1 defines */
#define PCIE_MSI_REQ				(BIT(2))

/* host HBM bit field definition */
#define HHBM_WR_REQ				(BIT(0))
#define HHBM_WATERMARK_LOW_INTR_STATS		(BIT(2))
#define HHBM_WATERMARK_LOW_MSK			(0xffff)
#define HHBM_WATERMARK(l, h)			(((l) & HHBM_WATERMARK_LOW_MSK) | (((h) & 0xffff) << 16))
#define HHBM_RDWR_PTR_MASK			(0x7fff)
#define HHBM_64BIT				(BIT(10))

/* offsets for dual PCIE */
#define PCIE_ATU_CTRL1(base)			((base) + 0x0904)
#define PCIE_ATU_CTRL2(base)			((base) + 0x0908)
#define PCIE_DMA_WR_ENABLE(base)		((base) + 0x097C)
#define PCIE_DMA_WR_CHWTLOW(base)		((base) + 0x0988)
#define PCIE_DMA_WR_CHWTHIG(base)		((base) + 0x098C)
#define PCIE_DMA_WR_INTSTS(base)		((base) + 0x09BC)
#define PCIE_DMA_WR_INTMASK(base)		((base) + 0x09C4)
#define PCIE_DMA_WR_INTCLER(base)		((base) + 0x09C8)
#define PCIE_DMA_WR_DONE_IMWR_ADDR_L(base)	((base) + 0x09D0)
#define PCIE_DMA_WR_DONE_IMWR_ADDR_H(base)	((base) + 0x09D4)
#define PCIE_DMA_WR_ABORT_IMWR_ADDR_L(base)	((base) + 0x09D8)
#define PCIE_DMA_WR_ABORT_IMWR_ADDR_H(base)	((base) + 0x09DC)
#define PCIE_DMA_WR_IMWR_DATA(base)		((base) + 0x09E0)
#define PCIE_DMA_WR_LL_ERR_EN(base)		((base) + 0x0A00)
#define PCIE_DMA_WR_DOORBELL(base)		((base) + 0x0980)
#define PCIE_DMA_RD_ENABLE(base)		((base) + 0x099C)
#define PCIE_DMA_RD_DOORBELL(base)		((base) + 0x09A0)
#define PCIE_DMA_RD_CHWTLOW(base)		((base) + 0x09A8)
#define PCIE_DMA_RD_CHWTHIG(base)		((base) + 0x09AC)
#define PCIE_DMA_RD_INTSTS(base)		((base) + 0x0A10)
#define PCIE_DMA_RD_INTMASK(base)		((base) + 0x0A18)
#define PCIE_DMA_RD_INTCLER(base)		((base) + 0x0A1C)
#define PCIE_DMA_RD_ERR_STS_L(base)		((base) + 0x0A24)
#define PCIE_DMA_RD_ERR_STS_H(base)		((base) + 0x0A28)
#define PCIE_DMA_RD_LL_ERR_EN(base)		((base) + 0x0A34)
#define PCIE_DMA_RD_DONE_IMWR_ADDR_L(base)	((base) + 0x0A3C)
#define PCIE_DMA_RD_DONE_IMWR_ADDR_H(base)	((base) + 0x0A40)
#define PCIE_DMA_RD_ABORT_IMWR_ADDR_L(base)	((base) + 0x0A44)
#define PCIE_DMA_RD_ABORT_IMWR_ADDR_H(base)	((base) + 0x0A48)
#define PCIE_DMA_RD_IMWR_DATA(base)		((base) + 0x0A4C)
#define PCIE_DMA_CHNL_CONTEXT(base)		((base) + 0x0A6C)
#define PCIE_DMA_CHNL_CNTRL(base)		((base) + 0x0A70)
#define PCIE_DMA_XFR_SIZE(base)			((base) + 0x0A78)
#define PCIE_DMA_SAR_LOW(base)			((base) + 0x0A7C)
#define PCIE_DMA_SAR_HIGH(base)			((base) + 0x0A80)
#define PCIE_DMA_DAR_LOW(base)			((base) + 0x0A84)
#define PCIE_DMA_DAR_HIGH(base)			((base) + 0x0A88)
#define PCIE_DMA_LLPTR_LOW(base)		((base) + 0x0A8C)
#define PCIE_DMA_LLPTR_HIGH(base)		((base) + 0x0A90)
#define PCIE_DMA_WRLL_ERR_ENB(base)		((base) + 0x0A00)
#define PCIE_DMA_RDLL_ERR_ENB(base)		((base) + 0x0A34)
#define PCIE_DMABD_CHNL_CNTRL(base)		((base) + 0x8000)
#define PCIE_DMABD_XFR_SIZE(base)		((base) + 0x8004)
#define PCIE_DMABD_SAR_LOW(base)		((base) + 0x8008)
#define PCIE_DMABD_SAR_HIGH(base)		((base) + 0x800c)
#define PCIE_DMABD_DAR_LOW(base)		((base) + 0x8010)
#define PCIE_DMABD_DAR_HIGH(base)		((base) + 0x8014)
#define PCIE_DMABD_LLPTR_LOW(base)		((base) + 0x8018)
#define PCIE_DMABD_LLPTR_HIGH(base)		((base) + 0x801c)
#define PCIE_WRDMA0_CHNL_CNTRL(base)		((base) + 0x8000)
#define PCIE_WRDMA0_XFR_SIZE(base)		((base) + 0x8004)
#define PCIE_WRDMA0_SAR_LOW(base)		((base) + 0x8008)
#define PCIE_WRDMA0_SAR_HIGH(base)		((base) + 0x800c)
#define PCIE_WRDMA0_DAR_LOW(base)		((base) + 0x8010)
#define PCIE_WRDMA0_DAR_HIGH(base)		((base) + 0x8014)
#define PCIE_WRDMA0_LLPTR_LOW(base)		((base) + 0x8018)
#define PCIE_WRDMA0_LLPTR_HIGH(base)		((base) + 0x801c)
#define PCIE_WRDMA1_CHNL_CNTRL(base)		((base) + 0x8020)
#define PCIE_WRDMA1_XFR_SIZE(base)		((base) + 0x8024)
#define PCIE_WRDMA1_SAR_LOW(base)		((base) + 0x8028)
#define PCIE_WRDMA1_SAR_HIGH(base)		((base) + 0x802c)
#define PCIE_WRDMA1_DAR_LOW(base)		((base) + 0x8030)
#define PCIE_WRDMA1_DAR_HIGH(base)		((base) + 0x8034)
#define PCIE_WRDMA1_LLPTR_LOW(base)		((base) + 0x8038)
#define PCIE_WRDMA1_LLPTR_HIGH(base)		((base) + 0x803c)
#define PCIE_RDDMA0_CHNL_CNTRL(base)		((base) + 0x8040)
#define PCIE_RDDMA0_XFR_SIZE(base)		((base) + 0x8044)
#define PCIE_RDDMA0_SAR_LOW(base)		((base) + 0x8048)
#define PCIE_RDDMA0_SAR_HIGH(base)		((base) + 0x804c)
#define PCIE_RDDMA0_DAR_LOW(base)		((base) + 0x8050)
#define PCIE_RDDMA0_DAR_HIGH(base)		((base) + 0x8054)
#define PCIE_RDDMA0_LLPTR_LOW(base)		((base) + 0x8058)
#define PCIE_RDDMA0_LLPTR_HIGH(base)		((base) + 0x805c)
#define PCIE_RDDMA1_CHNL_CNTRL(base)		((base) + 0x8060)
#define PCIE_RDDMA1_XFR_SIZE(base)		((base) + 0x8064)
#define PCIE_RDDMA1_SAR_LOW(base)		((base) + 0x8068)
#define PCIE_RDDMA1_SAR_HIGH(base)		((base) + 0x806c)
#define PCIE_RDDMA1_DAR_LOW(base)		((base) + 0x8070)
#define PCIE_RDDMA1_DAR_HIGH(base)		((base) + 0x8074)
#define PCIE_RDDMA1_LLPTR_LOW(base)		((base) + 0x8078)
#define PCIE_RDDMA1_LLPTR_HIGH(base)		((base) + 0x807c)

#define PCIE_MSI_LBAR(base)			((base) + 0x0054)
#define PCIE_MSI_ADDR_H(base)			((base) + 0x0058)
#define PCIE_MSI_DATA(base)			((base) + 0x005C)
#define PCIE_MSI_PEND_BIT(base)			((base) + 0x0064)
#define PCIE_DEVCAP(base)			((base) + 0x0074)
#define PCIE_ASPM_L1_CTRL(base)			((base) + 0x70c)
#define PCIE_ASPM_LINK_CTRL(base)		(PCIE_LINK_STAT)
#define PCIE_ASPM_L1_SUBSTATE_TIMING(base)	((base) + 0xB44)
#define PCIE_L1SUB_CTRL1(base)			((base) + 0x150)
#define PCIE_PMCSR(base)			((base) + 0x44)

/* AXI control defines */
#define PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH	0xFFFF0000
#define PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH_S	16

/* TX_DMA_CTRL defines */
#define PCIE_TX_DMA_DISABLE			(BIT(0))

/* PCIe link defines */
#define PEARL_PCIE_DATA_LINK			(BIT(0))
#define PEARL_PCIE_PHY_LINK			(BIT(1))
#define PEARL_PCIE_LINK_RST			(BIT(3))
#define PEARL_PCIE_FATAL_ERR			(BIT(5))
#define PEARL_PCIE_NONFATAL_ERR			(BIT(6))

#define PCIE_LINK_GEN1				(BIT(0))
#define PCIE_LINK_GEN2				(BIT(1))
#define PCIE_LINK_GEN3				(BIT(2))

#define MSI_EN					(BIT(0))
#define MSI_64_EN				(BIT(7))
#define PCIE_MSI_ADDR_OFFSET(a)			((a) & 0xFFFF)
#define PCIE_MSI_ADDR_ALIGN(a)			((a) & (~0xFFFF))

#define PCIE_ATU_UBAR(base)			((base) + 0x0910)

#define PCIE_MSI_ADDR_LOWER(base)		((base) + 0x0820)
#define PCIE_MSI_ADDR_UPPER(base)		((base) + 0x0824)
#define PCIE_MSI_MASK_RC(base)			((base) + 0x082c)
#define PEARL_PCIE_MSI_DATA			(0)
#define PCIE_MSI_GPIO(base)			((base) + 0x0888)

#define PCIE_HDP_HOST_QUEUE_FULL	(BIT(17))
#define USE_BAR_MATCH_MODE
#define PCIE_ATU_OB_REGION		(BIT(0))
#define PCIE_BASE_REGION		(0xb0000000)
#define PCIE_MEM_MAP_SIZE		(512*1024)

#define PCIE_OB_REG_REGION		(0xcf000000)

/* inbound mapping */
#define PCIE_IB_BAR0			(0x00000000)	/* ddr */
#define PCIE_IB_BAR0_CH			(0)
#define PCIE_IB_BAR3			(0xe0000000)	/* sys_reg */
#define PCIE_IB_BAR3_CH			(1)


/* outbound mapping */
#define PCIE_MEM_CH			(0)
#define PCIE_REG_CH			(1)
#define PCIE_MEM_REGION			(0xc0000000)
#define	PCIE_MEM_SIZE			(0x000fffff)
#define PCIE_MEM_TAR			(0x80000000)

/* size of config region */
#define PCIE_CFG_SIZE			(0x0000ffff)


#define PCIE_ATU_DIR_IB			(BIT(31))
#define PCIE_ATU_DIR_OB			(0)
#define PCIE_ATU_DIR_CFG		(2)
#define PCIE_ATU_DIR_MATCH_IB		(BIT(31) | BIT(30))


#define PCIE_DMA_WR_0			(0)
#define PCIE_DMA_WR_1			(1)
#define PCIE_DMA_RD_0			(2)
#define PCIE_DMA_RD_1			(3)

#define PCIE_DMA_CHNL_CNTRL_CB		(BIT(0))
#define PCIE_DMA_CHNL_CNTRL_TCB		(BIT(1))
#define PCIE_DMA_CHNL_CNTRL_LLP		(BIT(2))
#define PCIE_DMA_CHNL_CNTRL_LIE		(BIT(3))
#define PCIE_DMA_CHNL_CNTRL_RIE		(BIT(4))
#define PCIE_DMA_CHNL_CNTRL_CSS		(BIT(8))
#define PCIE_DMA_CHNL_CNTRL_LLE		(BIT(9))
#define PCIE_DMA_CHNL_CNTRL_TLP		(BIT(26))

#define PCIE_DMA_CHNL_CONTEXT_RD	(BIT(31))
#define PCIE_DMA_CHNL_CONTEXT_WR	(0)

/* PCIe HDP interrupt status definition */
#define PCIE_HDP_INT_EP_RXDMA		(BIT(0))
#define PCIE_HDP_INT_HBM_UF		(BIT(1))
#define PCIE_HDP_INT_RX_LEN_ERR		(BIT(2))
#define PCIE_HDP_INT_RX_HDR_LEN_ERR	(BIT(3))
#define PCIE_HDP_INT_RX_FIFO_EMPTY	(BIT(4))
#define PCIE_HDP_INT_EP_TXDMA		(BIT(12))
#define PCIE_HDP_INT_HHBM_UF		(BIT(13))
#define PCIE_HDP_INT_EP_TXEMPTY		(BIT(15))
#define PCIE_HDP_INT_IPC		(BIT(29))

/* PCIe interrupt status definition */
#define PCIE_INT_MSI			(BIT(24))
#define PCIE_INT_INTX			(BIT(23))

/* PCIe DESC control bit definition */
#define HDP_DESC_FETCH_EN		(BIT(31))

#define HDP_PCIE0_IPC_INT_OFF		(0x184)
#define HDP_PCIE0_IPC_INT_MSK_OFF	(0x188)
#define HDP_PCIE1_IPC_INT_OFF		(0x18c)
#define HDP_PCIE1_IPC_INT_MSK_OFF	(0x190)

#define IPC_SET_VALUE(x)		((x) | ((x) << 16))
#define IPC_CLR_VALUE(x)		((x) << 16)


#define IPC_BIT_EP_LINK			(BIT(0))
#define IPC_BIT_EP_IDLE			(BIT(1))
#define IPC_BIT_EP_REBOOT		(BIT(2))
/* Notify RC that EP has exits low power mode  */
#define IPC_BIT_EP_EXIT_IDLE		(BIT(3))

#define QTN_SYS_CTL_CPU_VEC_OFF		(0x4)
#define QTN_PCIE0_EN			(BIT(13))
#define QTN_PCIE1_EN			(BIT(16))

#define QTN_SYS_CTL_CSR_OFF		(0x14)

/* PCIe int control definition */
#define INT_MITIG_EN			(1 << 31)
#define INT_MITIG_NUM(n)		((n) & 0xff)
/*
 * timer input clock is 320MHz,
 * calculate timeout counter by 100us_per_packet in light traffic
 */
#define INT_MITIG_PKTN_TO_CNT(n)	(((n) * 320 * 100))
#define INT_MITIG_TIMEOUT_CNT_MIN	(500 * 320)
#define INT_MITIG_TIMEOUT_CNT_MAX	(0xfffff)
#define PCIE_HDP_DMA_PTR_SIZE		(4 * sizeof(uint64_t))
#endif /* __PEARL_PCIE_H */
