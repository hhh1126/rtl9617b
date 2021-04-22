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
 */

#ifndef __QTN_BASE_PCIE_BDA_H
#define __QTN_BASE_PCIE_BDA_H

#include "qtn_base_shm_ipc_defs.h"

#define QDPC_PCIE_BDA_VERSION		0x1005

/* Area mapped by via the BAR visible to the host */
#define RUBY_PCIE_BDA_ADDR		CONFIG_ARC_PCIE_BASE
#define RUBY_PCIE_BDA_SIZE		CONFIG_ARC_PCIE_SIZE

/*
 * bit map definition for ep status and flags
 * update by EP, read by RC
 */
#define QDPC_EP_HAS_UBOOT		(1 << 0)
#define QDPC_EP_HAS_FRIMWARE		(1 << 1)
#define QDPC_EP_REQ_UBOOT		(1 << 2)
#define QDPC_EP_REQ_FIRMWARE		(1 << 3)
#define QDPC_EP_ERROR_UBOOT		(1 << 4)
#define QDPC_EP_ERROR_FIRMWARE		(1 << 5)

#define QDPC_EP_FW_LOADRDY		(1 << 8)
#define QDPC_EP_FW_SYNC			(1 << 9)
#define QDPC_EP_FW_RETRY		(1 << 10)
#define QDPC_EP_FW_QLINK_DONE		(1 << 15)
#define QDPC_EP_FW_DONE			(1 << 16)
#define QDPC_EP_RX_READY		(1 << 17)
/* Used by linux Kernel only */
#define QDPC_EP_CFG_WAIT		(1 << 19)
#define QDPC_EP_CFG_DONE		(1 << 20)
#define QDPC_EP_CFG_FAILED		(1 << 21)

/*
 * bit map definition for ep status and flags
 * update by EP, read by RC
 */
#define QDPC_RC_PCIE_LINK		(1 << 0)
#define QDPC_RC_NET_LINK		(1 << 1)

#define QDPC_RC_FW_SOFTMAC		(1 << 6)
#define QDPC_RC_FW_QLINK		(1 << 7)
#define QDPC_RC_FW_LOADRDY		(1 << 8)
#define QDPC_RC_EP_CFG_NONE		(1 << 10) /* Used by linux Kernel only */
#define QDPC_RC_FW_TBM1			(1 << 11)
#define QDPC_RC_FW_TBM2			(1 << 12)

#define PEARL_PCIE_BOARDFLG		"PCIEQTN"

#define QDPC_PCI_ENDIAN_DETECT_DATA	0x12345678
#define QDPC_PCI_ENDIAN_REVERSE_DATA	0x78563412

#define PCIE_BDA_NAMELEN		32

#define PCIE_HHBM_MAX_SIZE		512

#define ENET_ADDR_LENGTH		6

#define RC_VERSION_LEN			32

typedef struct qdpc_pcie_bda {
	__le16	bda_len;			/* Size of BDA block */
	__le16	bda_version;			/* BDA version */
	__le32	bda_pci_endian;			/* Check pci memory endian format */
	__le32	bda_ep_state;			/* ep status and flags */
	__le32	bda_rc_state;
	__le32	bda_dma_mask;			/* Number of addressable DMA bits */
	__le32	bda_msi_addr;
	__le32	bda_flashsz;
	u8	bda_boardname[PCIE_BDA_NAMELEN];
	__le32	bda_rc_msi_enabled;
	__le32	bda_hhbm_list[PCIE_HHBM_MAX_SIZE];
	__le32	bda_dsbw_start_index;
	__le32	bda_dsbw_end_index;
	__le32	bda_dsbw_total_bytes;
	__le32	bda_rc_tx_bd_base;
	__le32	bda_rc_tx_bd_num;
	u8	bda_pcie_mac[ENET_ADDR_LENGTH];
	struct qtn_pcie_shm_region bda_shm_reg1	__attribute__ ((aligned (4096)));
	struct qtn_pcie_shm_region bda_shm_reg2	__attribute__ ((aligned (4096)));
	u8	bda_rc_version[RC_VERSION_LEN];
} __attribute__((__packed__)) qdpc_pcie_bda_t;

enum pearl_fw_loadtype {
	PEARL_FW_DBEGIN,
	PEARL_FW_DSUB,
	PEARL_FW_DEND,
	PEARL_FW_CTRL
};

struct pearl_pcie_fwhd {
	u8 boardflg[8];
	__le32 fwsize;
	__le32 seqnum;
	__le32 type;
	__le32 pktlen;
	__le32 crc;
} __attribute__((__packed__));

#endif /* __QTN_BASE_PCIE_BDA_H */
