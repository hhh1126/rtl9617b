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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Header file which describes PCI Express Boot Data Area
 * Has to be used by both kernel and bootloader.
 */

#ifndef PEARL_PCIE_BDA_H
#define PEARL_PCIE_BDA_H

#include "base/qtn_base_pcie_bda.h"

#define QDPC_PCIE_BDA_VERSION_WITH_RC_VERSION 0x1005

/*
 * These pare of FW_SYNC is used to sync
 * packets of FW downloading
 */
#define QDPC_RC_FW_SYNC			(1 << 9)

#define PCIE_HHBM_LOW_WATERMARK_DFT	(QTN_BUFS_PCIE_TXQ0_RING + 32) /* 32 is safe margin */
#define QDPC_PCIE_FW_DLMASK		0xF
#define QDPC_FW_DLTIMEOUT		3000

#define PEARL_PCIE_DWBUFSZ      2048
#define PEARL_PCIE_CFG_DWBUFSZ	1600

#endif
