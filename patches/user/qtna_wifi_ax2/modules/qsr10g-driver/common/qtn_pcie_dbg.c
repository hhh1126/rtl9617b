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

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <qdpc_platform.h>
#include "pearl_vnet.h"
#include "qdpc_config.h"
#include "qdpc_init.h"
#include "pearl_pcie.h"
#include "qtn_pcie_dbg.h"
#ifdef QTN_RC_ENABLE_HDP
#include "qtn/dmautil.h"
#endif
#define SKB_BUF_SIZE_DBG		(2048)
#define MAX_RUNTIME_DUMP_BD_NUM		(5)
#define VMAC_MAX_PRINT_BUF		(1 * 1024 * 1024 - 2 * sizeof(int))
#define dbg_str(dbg, fmt, ...) do { \
		((dbg)->wr) += sprintf((dbg)->buf + (dbg)->wr, fmt, ##__VA_ARGS__); \
	} while (0)

struct vmac_dbg_buf {
	int wr;
	int rd;
	char buf[VMAC_MAX_PRINT_BUF];
};

uintptr_t vmac_dbg_show_addr = 0;
int vmac_dbg_show_num = 1;

static void vmac_dbg_counts_clear(struct net_device *ndev);

static void *vmac_init_dbg_buf(struct vmac_priv *vmp)
{
	struct vmac_dbg_buf *dbg;

	if (!vmp->dbg_buf)
		vmp->dbg_buf = vmalloc(sizeof(struct vmac_dbg_buf));

	dbg = vmp->dbg_buf;
	if (dbg) {
		dbg->wr = 0;
		dbg->rd = 0;
	}
	return dbg;
}
void vmac_dbg_buf_print(struct vmac_priv *vmp)
{
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	if (!dbg)
		return;
	if (dbg->wr)
		printk("%s", dbg->buf);
	dbg->wr = dbg->rd = 0;
}

static unsigned long get_bar_base_addr(struct vmac_priv *vmp, enum vmac_dbg_cmd cmd)
{
	if (cmd == VMAC_SYS_READ || cmd == VMAC_SYS_WRITE)
		return (unsigned long)vmp->sysctl_bar.b_vaddr;
	else  if (cmd == VMAC_HDP_READ || cmd == VMAC_HDP_WRITE)
		return (unsigned long)vmp->dmareg_bar.b_vaddr;
	else  if (cmd == VMAC_mMEM_READ || cmd == VMAC_mMEM_WRITE)
		return (unsigned long)vmp->epmem_bar.b_vaddr;
	return 0;
}

static void vmac_write_mem(unsigned long addr, unsigned int value)
{
	vmac_writel(value, (void *)addr);
	vmac_dbg_show_addr = addr;
}

static int vmac_read_mem(struct vmac_priv *vmp)
{
	uintptr_t addr = vmac_dbg_show_addr;
	int i;
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;

	if (!addr)
		return 0;

	for (i = 0; i < vmac_dbg_show_num; i++) {
		if (i == 0 || !(addr % 32)) {
			if (i != 0)
				dbg_str(dbg, "\n");
			dbg_str(dbg, "%p:", (void *)addr);
		}
		dbg_str(dbg, " %08x", vmac_readl((void *)addr));
		addr += sizeof(uint32_t);
	}
	dbg_str(dbg, "\n");
	return (dbg->wr);
}

int txbd2str_range(struct vmac_priv *vmp, uint16_t s, int num)
{
	volatile struct vmac_tx_bd *tbdp;
	struct tx_buf_info_t *txinfo;
	int i;
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	char *iflag;

	dbg_str(dbg, "index dsc_add\t\taddress\t\tinfo_l\t\tskb\t\tpa\n");
	for (i = 0; i < num; i++) {
		tbdp = &vmp->tx_bd_base[s];
		txinfo = &vmp->tx_buf_info[s];
		iflag = (s == vmp->tx_bd_index) ? "Tx" : ((s == vmp->tx_done_index) ? "dn" : " ");
		dbg_str(dbg, "%3u %3s %p\t%p\t%08x\t%p\t%p\n",
			(uint32_t)s, iflag, tbdp, (void *)vmac_mergell(tbdp->buff_addr, tbdp->buff_addr_h),
			tbdp->buff_info, txinfo->skb, (void *)txinfo->pa);
		s = VMAC_TX_INDEX_INC(vmp, s, 1);
	}

	return dbg->wr;
}

int txbd2str(struct vmac_priv *vmp)
{
	uint16_t s;

	s = VMAC_TX_INDEX_INC(vmp, vmp->tx_bd_index, - (MAX_RUNTIME_DUMP_BD_NUM / 2));
	txbd2str_range(vmp, s, MAX_RUNTIME_DUMP_BD_NUM);
	vmac_dbg_buf_print(vmp);

	return 0;
}

static int rxbd2str_range(struct vmac_priv *vmp, uint16_t s, int num)
{
	int i;
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	volatile struct vmac_rx_bd *rbdp;
	struct rx_buf_info_t *rxinfo;
	char *iflag;

#ifdef QTN_RC_ENABLE_HDP
	dbg_str(dbg, "rxindx rbdaddr\t\tbuff\t\tinfo\t\tpa\n");
#else
	dbg_str(dbg, "rxindx rbdaddr\t\tbuff\t\tinfo\t\tpa\t\trx_skb\n");
#endif
	for (i = 0; i < num; i++) {
		rbdp = &vmp->rx_bd_base[s];
		rxinfo = &vmp->rx_buf_info[s];

		iflag = (s == vmp->rx_bd_index) ? "Rx" : " ";
#ifdef QTN_RC_ENABLE_HDP
		dbg_str(dbg, "%3u %3s %p\t%p\t%08x\t%p\n",
			(uint32_t)s, iflag, rbdp, (void *)vmac_mergell(rbdp->buff_addr, rbdp->buff_addr_h),
			rbdp->buff_info, (void *)rxinfo->pa);
#else
		dbg_str(dbg, "%3u %3s %p\t%p\t%08x\t%p\t%p\n",
			(uint32_t)s, iflag, rbdp, (void *)vmac_mergell(rbdp->buff_addr, rbdp->buff_addr_h),
			rbdp->buff_info, (void *)rxinfo->pa, rxinfo->skb);
#endif
		s = VMAC_RX_INDEX_INC(vmp, s, 1);
	}
	return dbg->wr;
}

int rxbd2str(struct vmac_priv *vmp)
{
	uint16_t s;

	s = VMAC_RX_INDEX_INC(vmp, vmp->rx_bd_index, -(MAX_RUNTIME_DUMP_BD_NUM / 2));
	rxbd2str_range(vmp, s, MAX_RUNTIME_DUMP_BD_NUM);
	vmac_dbg_buf_print(vmp);
	return 0;
}

uint32_t get_hhbm_avail(struct vmac_priv *vmp)
{
#define TOPAZ_HBM_MAX_POOL_COUNT	(1 << 15)
#define PEARL_HBM_CACHE_SIZE		(1 << 6)
	uint32_t wr_ptr;
	uint32_t rd_ptr;
	uint32_t max_pool_count;
	uint32_t w;
	uint32_t r;
	uint32_t n;

	wr_ptr = le32_to_cpu(readl(IO_ADDRESS(PCIE_HHBM_Q_WR_REG(vmp->pcie_reg_base))));
	rd_ptr = le32_to_cpu(readl(IO_ADDRESS(PCIE_HHBM_Q_RD_REG(vmp->pcie_reg_base))));
	max_pool_count = le32_to_cpu(readl(IO_ADDRESS(PCIE_HHBM_Q_LIMIT_REG(vmp->pcie_reg_base))))
				& HHBM_RDWR_PTR_MASK;

	w = wr_ptr & HHBM_RDWR_PTR_MASK;
	r = rd_ptr & HHBM_RDWR_PTR_MASK;
	n = (w >= r) ? w - r : w + max_pool_count - r;

	n += ((wr_ptr >> 16) - (rd_ptr >> 16)) % PEARL_HBM_CACHE_SIZE;
	return n;
}

static int vmaccnt2str(struct vmac_priv *vmp)
{
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	struct net_device *ndev = vmp->ndev;
	uint32_t tmp;

	dbg_str(dbg, "==host0: %s\n", netif_running(ndev) ? "running" : "down");
	dbg_str(dbg, "vmp state:\t0x%lx\n",  vmp->state);

	dbg_str(dbg, "\n== TX:\n");
	dbg_str(dbg, "tx_bd_index:\t%08x\n", vmp->tx_bd_index);
	dbg_str(dbg, "tx_done_index:\t%08x\n", vmp->tx_done_index);
	dbg_str(dbg, "tx_intr_cnt:\t%08x\n", vmp->tx_intr_cnt);
	dbg_str(dbg, "tx_pending_cnt:\t%08x\n", vmp->tx_pending_cnt);
	if (vmac_fast_hdp(vmp)) {
		dbg_str(dbg, "tx_dma_ptr_va:\t%04x\n", vmac_readw(vmp->tx_dma_ptr_va));
		tmp = vmac_readl(PCIE_HDP_RX2_DESC_Q_WR_PTR(vmp->pcie_reg_base));
		dbg_str(dbg, "hw_tx_idex: %04x\n", tmp & 0xffff);
		dbg_str(dbg, "hw_tx_done: %04x\n", tmp >> 16 & 0xffff);
		tmp = vmac_readl(PCIE_HDP_RX2DMA_CNT(vmp->pcie_reg_base));
	} else {
		tmp = vmac_readl(PCIE_HDP_RX0DMA_CNT(vmp->pcie_reg_base));
	}
	dbg_str(dbg, "reg_rxdma_cnt:\t%08x\n", tmp);
	dbg_str(dbg, "tx_packets:\t%08lx\n", ndev->stats.tx_packets);
	dbg_str(dbg, "tx_done_cnt:\t%08x\n", vmp->xmit_txdone_cnt + vmp->irq_txdone_cnt);
	dbg_str(dbg, "irq_txdone_cnt:\t%08x\n", vmp->irq_txdone_cnt);
	dbg_str(dbg, "tx_q_stop_cnt:\t%08x %s\n", vmp->tx_q_stop_cnt,
			netif_queue_stopped(ndev) ?  "stop" : "wake");
	dbg_str(dbg, "tx_bd_busy_cnt:\t%08x\n", vmp->tx_bd_busy_cnt);
	dbg_str(dbg, "tx_timeout_cnt:\t%08x\n", vmp->tx_timeout_cnt);
	dbg_str(dbg, "tx_free_err:\t%08x\n", vmp->tx_free_err);
	dbg_str(dbg, "tx_oversize_cnt:\t%08x\n", vmp->tx_oversize_cnt);

	dbg_str(dbg, "\n== RX:\n");
	dbg_str(dbg, "rx_intr_cnt:\t%08x\n", vmp->rx_intr_cnt);
	dbg_str(dbg, "rx_done_cnt:\t%08x\n", vmp->rx_done_cnt);
	dbg_str(dbg, "rx_napi_no_pkt:\t%08x\n", vmp->rx_napi_no_pkt);
	dbg_str(dbg, "rx_bd_index:\t%08x\n", vmp->rx_bd_index);
	dbg_str(dbg, "rx_packets:\t%08lx\n", ndev->stats.rx_packets);
	if (vmac_fast_hdp(vmp))
		dbg_str(dbg, "reg_txdma_cnt:\t%08x\n", vmac_readl(PCIE_HDP_TX0DMA_CNT(vmp->pcie_reg_base)));
	dbg_str(dbg, "rx_skb_alloc_failures:\t%x\n", vmp->rx_skb_alloc_failures);
	dbg_str(dbg, "intr_rx_no_dsc:\t%08x\n", vmp->intr_rx_no_dsc);
	dbg_str(dbg, "intr_rx_no_buf:\t%08x\n", vmp->intr_rx_no_buf);

	dbg_str(dbg, "\n== Interrupts:\n");
	dbg_str(dbg, "intr_cnt:\t%08x\n", vmp->intr_cnt);
	dbg_str(dbg, "pcie_int_en:\t%08x\n", vmp->pcie_int_en);

#ifdef QTN_SKB_RECYCLE_SUPPORT
	dbg_str(dbg, "skb_recycle_cnt:\t%08x\n", vmp->skb_recycle_cnt);
	dbg_str(dbg, "skb_recycle_failures:\t%08x\n", vmp->skb_recycle_failures);
#endif
#ifdef QTN_RC_ENABLE_HDP
	dbg_str(dbg, "fwt_loss_cnt:\t%08x\n", vmp->fwt_loss_cnt);
#endif
	return dbg->wr;
}

ssize_t vmac_dbg_show(struct file *fp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct vmac_priv *vmp = netdev_priv(ndev);
	int len;
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;

	if (!dbg || dbg->wr <= dbg->rd)
		return 0;

	len = scnprintf(buf, (int)size, "%s", &dbg->buf[dbg->rd]);
	dbg->rd += len;

	return len + 1;
}

static int vmac_dbg_get(struct net_device *ndev, int cmd)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	int count = 0;

	switch (cmd) {
	case SHOW_TX_BD:	/* show Tx BD */
		count = txbd2str_range(vmp, 0, vmp->tx_bd_num);
		break;
	case SHOW_RX_BD:	/* show Rx BD */
		count = rxbd2str_range(vmp, 0, vmp->rx_bd_num);
		break;
	case SHOW_VMAC_STATS:	/* show vmac interrupt statistic info */
		count = vmaccnt2str(vmp);
		break;
	default:
		dbg_str(dbg, "unsupported cmd=%d\n", cmd);
		break;
	}
	return count;
}

void vmac_dbg_set(struct net_device *ndev, int cmd)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;

	if (cmd < 16) {
		switch (cmd) {
		case 0:
			vmp->dbg_flg = 0;
			break;
		case 1:
			napi_schedule(&vmp->napi);
			dbg_str(dbg, "Napi scheduled\n");
			break;
		case 2:
			vmac_dbg_counts_clear(ndev);
			dbg_str(dbg, "clear counter, done\n");
			break;
		case 3:
			dbg_str(dbg, "hhbm avail: %d\n", get_hhbm_avail(vmp));
			break;
		default:
			dbg_str(dbg, "unsupported cmd\n");
			break;
		}
		return;
	} else if (cmd >= 32 && cmd < 64) { /* set runtime dump flag */
		vmp->dbg_flg |= (0x1 << (cmd - 32));
	}
	dbg_str(dbg, "vmp->dbg_flg=%x\n", vmp->dbg_flg);
}

static void pearl_start_dsbw_test(struct net_device *ndev, uint32_t num,
				  uint32_t len, uint32_t offset,
				  uint32_t pattern);

ssize_t vmac_dbg_store(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
#define	MAX_ARGS		(8)
#define MEM_DBG_CMD_SEP		" \n"
#define MEM_DBG_CMD_LEN		128
	struct device *dev = container_of(kobj, struct device, kobj);
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	char kbuf[MEM_DBG_CMD_LEN];
	char *pstr = kbuf;
	int i, ret;
	int argc = 0;
	unsigned long argv[MAX_ARGS] = { 0 };
	char *token;
	int cmd = 0;
	struct vmac_priv *const vmp = netdev_priv(ndev);
	struct vmac_dbg_buf *dbg;

	strlcpy(kbuf, buf, min((size_t)MEM_DBG_CMD_LEN, count));

	/* parse the parameter */
	token = strsep(&pstr, MEM_DBG_CMD_SEP);
	if (!strcmp(token, "s") || !strcmp(token, "set"))
		cmd = VMAC_DBG_SET;
	if (!strcmp(token, "g") || !strcmp(token, "get"))
		cmd = VMAC_DBG_GET;
	else if (!strcmp(token, "r") || !strcmp(token, "read"))
		cmd = VMAC_DBG_READ;
	else if (!strcmp(token, "sysr"))
		cmd = VMAC_SYS_READ;
	else if (!strcmp(token, "hdpr"))
		cmd = VMAC_HDP_READ;
	else if (!strcmp(token, "mmr"))
		cmd = VMAC_mMEM_READ;
	else if (!strcmp(token, "w") || !strcmp(token, "write"))
		cmd = VMAC_DBG_WRITE;
	else if (!strcmp(token, "sysw"))
		cmd = VMAC_SYS_WRITE;
	else if (!strcmp(token, "hdpw"))
		cmd = VMAC_HDP_WRITE;
	else if (!strcmp(token, "mmw"))
		cmd = VMAC_mMEM_WRITE;
	else if (!strcmp(token, "ds"))
		cmd = VMAC_DBG_BW;
	 else if (!strcmp(token, "firmware_download")) {
		printk("download firware...");
		ret = qdpc_bringup_fw(vmp);
		printk(KERN_CONT "%s\n", ret == 0 ? "OK" : "failed");
	}

	while ((token = strsep(&pstr, MEM_DBG_CMD_SEP)) && (token[0] != 0)) {
		unsigned int base = (cmd == VMAC_DBG_SET ||
					cmd == VMAC_DBG_GET) ? 10 : 16;
		argv[argc++] = simple_strtoul(token, NULL, base);
	}
	/* Prepare buffer before execute command */
	dbg = vmac_init_dbg_buf(vmp);
	if (!dbg) {
		printk("Fail to get debug buffer\n");
		return count;
	}
	/* execut the command */
	switch (cmd) {
	case VMAC_DBG_SET:
		for (i = 0; i < argc; i++)
			vmac_dbg_set(ndev, argv[i]);
		break;
	case VMAC_DBG_GET:
		for (i = 0; i < argc; i++) {
			vmac_dbg_get(ndev, argv[i]);
		}
		break;
	case VMAC_DBG_READ:
	case VMAC_SYS_READ:
	case VMAC_HDP_READ:
	case VMAC_mMEM_READ:
		if (argc > 0)
			vmac_dbg_show_addr = argv[0] + get_bar_base_addr(vmp, cmd);
		if (argc > 1)
			vmac_dbg_show_num = argv[1];
		vmac_read_mem(vmp);
		break;
	case VMAC_SYS_WRITE:
	case VMAC_HDP_WRITE:
	case VMAC_mMEM_WRITE:
		if (argc > 1) {
			vmac_write_mem(argv[0] + get_bar_base_addr(vmp, cmd),
				(unsigned int)argv[1]);
			dbg_str(dbg, "OK\n");
		} else {
			dbg_str(dbg, "Error, argc less than 2\n");
		}
		break;
	case VMAC_DBG_BW:
		if (argv[0])
			pearl_start_dsbw_test(ndev, argv[0], argv[1], argv[2],
					      argv[3]);
		break;
	default:
		break;
	}
	return count;
}

#ifdef VMAC_DEBUG_MODE
void dump_pkt(unsigned char *data, int len, char *s)
{
	int i;

	if (len > 128)
		len = 128;
	printk("%spkt start : %p len: %d>\n", s, data, len);
	for (i = 0; i < len;) {
		printk(KERN_CONT "%02x ", data[i]);
		if ((++i % 16) == 0)
			printk(KERN_CONT "\n");
	}
	printk(KERN_CONT "<%spkt end\n", s);
}

void dump_rx_interrupt(struct vmac_priv *vmp)
{
	printk("intr_cnt:\t%08x\n", vmp->intr_cnt);
}
#endif

static void fill_mem(uint8_t * dst, uint32_t len, uint32_t pattern)
{
	int inc = 0;

	if (pattern > 0xff) {
		inc = 1;
		pattern = 0;
	}

	while (len--) {
		*dst++ = pattern;
		if (inc)
			pattern++;
	}
}

static inline int vmac_wait_until_txdone(struct vmac_priv *vmp)
{
	uint16_t dma_index;
	unsigned long timeout = jiffies + HZ;

	do {
		if (vmac_fast_hdp(vmp)) {
			dma_index = vmac_readw(vmp->tx_dma_ptr_va);
		} else {
			dma_index = (uint16_t)vmac_readl(
				PCIE_HDP_RX0DMA_CNT(vmp->pcie_reg_base));
		}
		dma_index &= vmp->tx_bd_num_msk;
		if (time_after(jiffies, timeout))
			break;

	} while (dma_index != vmp->tx_bd_index);

	return (dma_index == vmp->tx_bd_index);
}

static void pearl_start_dsbw_test(struct net_device *ndev, uint32_t num,
				  uint32_t len, uint32_t offset,
				  uint32_t pattern)
{
	struct vmac_priv *const vmp = netdev_priv(ndev);
	struct sk_buff *skb;
	void **data_buf;
	int i;
	int irc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	struct timespec64 s_ts;
	struct timespec64 e_ts;
	struct timespec64 delta;
#else
	struct timespec s_ts;
	struct timespec e_ts;
	struct timespec delta;
#endif

	unsigned long flags;
	int success = 0;
	struct vmac_dbg_buf *dbg = vmp->dbg_buf;
	num = (num > vmp->tx_bd_num_msk) ? vmp->tx_bd_num_msk : num;

	data_buf =  kzalloc(sizeof(void *) * num, GFP_KERNEL);
	if (!data_buf) {
		printk("Fail to allocate buffer, exit\n");
		return;
	}
	/* prepare the test data */
	for (i = 0; i < num; i++) {
		skb = dev_alloc_skb(SKB_BUF_SIZE_DBG);
		if (!skb) {
			printk(KERN_ERR "%s: alloc_netdev failed\n",
			       __FUNCTION__);
			break;
		}
		data_buf[i] = skb;
		skb_reserve(skb, align_buf_dma_offset(skb->data));
		skb_reserve(skb, offset);
		fill_mem(skb->data, len, pattern);
		skb_put(skb, len);
	}

	local_irq_save(flags);

	netif_stop_queue(ndev);
	preempt_disable();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	ktime_get_ts64(&s_ts);
#else
	getnstimeofday(&s_ts);
#endif
	for (i = 0; i < num; i++) {
		irc = vmac_hard_start_xmit(data_buf[i], ndev);
		if (NETDEV_TX_OK != irc) {
			dev_kfree_skb(data_buf[i]);
		}
		success++;
	}
	irc = vmac_wait_until_txdone(vmp);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	ktime_get_ts64(&e_ts);
#else
	getnstimeofday(&e_ts);
#endif
	preempt_enable();
	netif_start_queue(ndev);
	local_irq_restore(flags);
	kfree(data_buf);

	if (irc) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
		delta = timespec64_sub(e_ts, s_ts);
#else
		delta = timespec_sub(e_ts, s_ts);
#endif
		dbg_str(dbg, "Time: %ldnS, success: %d, bandwidth: %ldMbps\n",
				delta.tv_nsec, success,
				(8 * NSEC_PER_USEC * len * success) / delta.tv_nsec);
	} else {
		dbg_str(dbg, "Fail to get test result\n");
	}

}

static void vmac_dbg_counts_clear(struct net_device *ndev)
{
	struct vmac_priv *const vmp = netdev_priv(ndev);

	vmp->tx_intr_cnt = 0;
	vmp->tx_pending_cnt = 0;
	vmp->tx_bd_busy_cnt = 0;
	vmp->tx_timeout_cnt = 0;
	vmp->tx_q_stop_cnt = 0;
	vmp->irq_txdone_cnt = 0;

	vmp->rx_intr_cnt = 0;
	vmp->rx_done_cnt = 0;
	vmp->rx_skb_alloc_failures = 0;
	vmp->intr_rx_no_dsc = 0;

	vmp->intr_cnt = 0;
}

