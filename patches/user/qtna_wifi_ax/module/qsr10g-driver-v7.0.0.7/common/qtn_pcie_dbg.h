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
#ifndef __QTN_PCIE_DBG__H
#define __QTN_PCIE_DBG__H

#define VMAC_DEBUG_MODE
/* Tx dump flag */
#define DMP_FLG_TX_BD		(0x1 << ( 0))	/* vmac s 32 */
#define DMP_FLG_TX_SKB		(0x1 << ( 1))	/* vmac s 33 */
/* Rx dump flag */
#define DMP_FLG_RX_BD		(0x1 << (16))	/* vmac s 48 */
#define DMP_FLG_RX_SKB		(0x1 << (17))	/* vmac s 49 */
#define DMP_FLG_RX_INT		(0x1 << (18))	/* vmac s 50 */

#define VMAC_CTRL_FLAG_SIMU_PCI_RX	(0x1 << 20)
#define VMAC_CTRL_FLAG_SIMU_PCI_TX	(0x1 << 21)

#define SHOW_TX_BD		(16)
#define SHOW_RX_BD		(17)
#define SHOW_VMAC_STATS		(18)
#define SHOW_VMAC_MEM		(19)

enum vmac_dbg_cmd {
	VMAC_DBG_SET = 0,
	VMAC_DBG_GET,
	/* absolute address access, may be local address or PCIe mapped address */
	VMAC_DBG_READ,
	VMAC_DBG_WRITE,
	/* PCIe mapped sys control bar access */
	VMAC_SYS_READ,
	VMAC_SYS_WRITE,
	/* PCIe mapped HDP bar access */
	VMAC_HDP_READ,
	VMAC_HDP_WRITE,
	/* PCIe mapped memory bar access */
	VMAC_mMEM_READ,
	VMAC_mMEM_WRITE,

	VMAC_DBG_BW
};

#ifdef VMAC_DEBUG_MODE

#define dump_tx_bd(vmp) do { \
		if (unlikely((vmp)->dbg_flg & DMP_FLG_TX_BD)) { \
			txbd2str(vmp); \
		} \
	} while (0)

#define dump_tx_pkt(vmp, data, len) do { \
		if (unlikely(((vmp)->dbg_flg & DMP_FLG_TX_SKB))) \
			dump_pkt(data, len, "Tx"); \
	} while(0)

#define dump_rx_bd(vmp) do { \
		if (unlikely((vmp)->dbg_flg & DMP_FLG_RX_BD)) { \
			rxbd2str(vmp); \
		} \
	} while (0)

#define dump_rx_pkt(vmp, data, len) do { \
		if (unlikely((vmp)->dbg_flg & DMP_FLG_RX_SKB)) \
			dump_pkt(data, len, "Rx"); \
	} while(0)

#define dump_rx_int(vmp) do { \
		if (unlikely((vmp)->dbg_flg & DMP_FLG_RX_INT)) \
			dump_rx_interrupt(vmp); \
	} while (0)

#else
#define dump_tx_bd(vmp)
#define dump_tx_pkt(vmp, skb, len)
#define dump_rx_bd(vmp)
#define dump_rx_pkt(vmp, skb, len)
#define dump_rx_int(vmp)
#endif

#ifdef VMAC_DEBUG_MODE
void dump_pkt(unsigned char *data, int len, char *s);
void dump_rx_interrupt(struct vmac_priv *vmp);
int rxbd2str(struct vmac_priv *vmp);
int txbd2str_range(struct vmac_priv *vmp, uint16_t s, int num);
ssize_t vmac_dbg_show(struct file *fp, struct kobject *kobj,
	struct bin_attribute *bin_attr,	char *buf, loff_t pos, size_t size);
ssize_t vmac_dbg_store(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count);
int txbd2str_range(struct vmac_priv *vmp, uint16_t s, int num);
int txbd2str(struct vmac_priv *vmp);
#endif

#endif				/* __QTN_PCIE_DBG__H */
