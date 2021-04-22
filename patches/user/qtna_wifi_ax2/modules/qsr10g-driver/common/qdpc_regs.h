/**
 * Copyright (c) 2015 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 **/

#ifndef __QDPC_REGS_H__
#define __QDPC_REGS_H__

#include <linux/bitops.h>
#include <qdpc_platform.h>

#define QDPC_SYSCTL_BAR		0
#define QDPC_SHMEM_BAR		2
#define QDPC_DMA_BAR		3

/*
 * NOTE: Below registers are at EP but accessed and written by RC
 * Make sure EP codes do not write them, otherwise we have race conditions
*/

/*
 * The register is one of registers of Endpoint. Root Complex uses it
 * to interrupt Endpoint to transmit packets.
 */
#define PEARL_IPC_OFFSET		(0x1C)
#define PEARL_LH_IPC_EP_REBOOT (7)

/* Used to deassert Legacy INTx */
#define TOPAZ_PCIE_CFG0_OFFSET		(0x6C)
#define TOPAZ_ASSERT_INTX		BIT(9)

/* This macro is used to set interrupt bit of register QDPC_EP_SYS_CTL_IPC4_INT */
#define TOPAZ_SET_INT(x)		((x) | ((x) << 16))

/* "DMA Write Done IMWr Address Low" register at EP side*/
#define TOPAZ_IMWR_DONE_ADDRLO_OFFSET	(0x700 + 0x2D0)
#define TOPAZ_IMWR_ABORT_ADDRLO_OFFSET	(0x700 + 0x2D8)

/* Power management control status register */
#define TOPAZ_PCI_PM_CTRL_OFFSET	(0x44)

#endif				//__QDPC_REGS_H__
