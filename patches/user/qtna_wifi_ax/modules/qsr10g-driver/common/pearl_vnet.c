/**
 * Copyright (c) 2015-2021 Quantenna Communications, Inc.
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

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/skbuff.h>
#include <linux/if_bridge.h>
#include <linux/sysfs.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/timex.h>

#ifdef QTN_RC_ENABLE_HDP
#include <linux/netdevice.h>
#include <linux/net/bridge/br_public.h>

#include <qtn/topaz_tqe.h>
#include <qtn/topaz_hbm_cpuif.h>
#include <qtn/topaz_fwt_db.h>
#include <topaz_pcie_tqe.h>
#include <qtn/topaz_hbm.h>
#endif

#include <qdpc_platform.h>

#include <asm/cache.h>		/* For cache line size definitions */
#include <asm/cacheflush.h>	/* For cache flushing functions */

#include <net/netlink.h>

#include "pearl_vnet.h"
#include "qdpc_config.h"
#include "qdpc_init.h"
#include "qdpc_debug.h"
#include "qdpc_regs.h"
#include "qdpc_version.h"
#include "pearl_pcie.h"
#include "qtn_pcie_dbg.h"

/* Temporarily switch back to the old way to work around the
 * BBIC5 A0 defect of HBM failing when it's empty. The defect will
 * be fixed in B0
 */

#define QTN_HOST_DRV_NAME	"qsr10g-pcie"

#define DRV_AUTHOR	"Quantenna Communications Inc."
#define DRV_DESC	"PCIe virtual Ethernet port driver"

#define VMAC_INT_ALL_BITS (0	\
	| PCIE_HDP_INT_EP_TXDMA		\
	| PCIE_HDP_INT_EP_TXEMPTY	\
	| PCIE_HDP_INT_HHBM_UF		\
	| PCIE_HDP_INT_EP_RXDMA		\
	| PCIE_HDP_INT_IPC		\
	)

#define VMAC_INT_NAPI_BITS (0	\
	| PCIE_HDP_INT_EP_TXDMA		\
	| PCIE_HDP_INT_EP_TXEMPTY	\
	| PCIE_HDP_INT_HHBM_UF		\
	)

#define VMAC_TX_PENDING_TH		(2)

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#undef __sram_text
#ifdef QTN_RC_ENABLE_HDP
#define __sram_text		__attribute__ ((__section__ (__sram_text_sect_name)))
#else
#define __sram_text
#endif

static int vmac_rx_poll(struct napi_struct *napi, int budget);
static int skb2rbd_attach(struct net_device *ndev, uint16_t i);
static irqreturn_t vmac_interrupt(int irq, void *dev_id);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static void vmac_tx_timeout(struct net_device *ndev, unsigned int txqueue);
#else
static void vmac_tx_timeout(struct net_device *ndev);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
static int vmac_get_link_ksettings(struct net_device *ndev, struct ethtool_link_ksettings *cmd);
static int vmac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *cmd);
#else
static int vmac_get_settings(struct net_device *ndev, struct ethtool_cmd *cmd);
static int vmac_set_settings(struct net_device *ndev, struct ethtool_cmd *cmd);
#endif
static void vmac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info);
static void free_tx_skbs(struct vmac_priv *vmp);
static void free_rx_skbs(struct vmac_priv *vmp);
static int alloc_and_init_rxbuffers(struct net_device *ndev);
static void bring_up_interface(struct net_device *ndev);
static void shut_down_interface(struct net_device *ndev);
static int vmac_open(struct net_device *ndev);
static int vmac_close(struct net_device *ndev);
static int vmac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);
static int vmac_change_mtu(struct net_device *netdev, int new_mtu);

#ifdef QTN_RC_ENABLE_HDP
#define HOST_RXDESC_WRAP	(0X8000)
#endif
#define VMAC_RX_BD_LEN		(sizeof(struct vmac_rx_bd))
#define VMAC_TX_BD_LEN		(sizeof(struct vmac_tx_bd))

#define QTN_GLOBAL_INIT_EMAC_TX_QUEUE_LEN 256

#ifndef QDPC_PLATFORM_IFPORT
#define QDPC_PLATFORM_IFPORT 0
#endif

#define VMAC_TX_TIMEOUT			(180 * HZ)
#define VMAC_TX_NUM_PER_INTR		(4)

struct vmac_cfg vmaccfg = {
	QDPC_RX_QUEUE_SIZE, QDPC_TX_QUEUE_SIZE, "host%d", NULL
};

static char *ethaddr = NULL;
module_param(ethaddr, charp, S_IRUGO);
MODULE_PARM_DESC(store, "ethaddr");

int napi_budget = 10;
unsigned int rxpkt_per_intr = 4;
unsigned int txpkt_per_intr = VMAC_TX_NUM_PER_INTR;
unsigned int tx_pending_th = VMAC_TX_PENDING_TH;
module_param(napi_budget, int, 0644);
module_param(rxpkt_per_intr, uint, 0644);
module_param(txpkt_per_intr, uint, 0644);
module_param(tx_pending_th, uint, 0644);

static BIN_ATTR(dbg, 0644, vmac_dbg_show, vmac_dbg_store, 0);	/* bin_attr_dbg */

static ssize_t vmac_pm_show(struct device *dev, struct device_attribute *attr,
			    char *buff)
{
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct vmac_priv *vmp = netdev_priv(ndev);
	int count = 0;

	count += sprintf(buff + count, "PCIE Device Power State : %s\n",
			 vmp->ep_pmstate == PCI_D3hot ? "D3" : "D0");

	return count;
}

static ssize_t vmac_pm_set(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t count)
{
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct vmac_priv *vmp = netdev_priv(ndev);
	uint8_t cmd;

	cmd = (uint8_t) simple_strtoul(buf, NULL, 10);

	if (cmd == 0) {
		qdpc_pcie_resume(vmp->pdev);
	} else if (cmd == 1) {
		pm_message_t state;
		state.event = 0;
		qdpc_pcie_suspend(vmp->pdev, state);
	}

	return count;
}

static DEVICE_ATTR(pmctrl, S_IWUSR | S_IRUSR, vmac_pm_show, vmac_pm_set);	/* dev_attr_pmctrl */

static struct attribute *vmac_device_attrs[] = {
	&dev_attr_pmctrl.attr,
	NULL,
};
static struct bin_attribute *vmac_bin_attrs[] = {
	&bin_attr_dbg,
	NULL,
};
static const struct attribute_group vmac_attr_group = {
	.attrs = vmac_device_attrs,
	.bin_attrs = vmac_bin_attrs,
};

static inline void vmac_set_hdp_base(struct vmac_priv *vmp)
{
	vmp->pcie_reg_base = QDPC_BAR_VADDR(vmp->dmareg_bar, 0);
}

static inline uint32_t pktnum_to_timeout_cnt(unsigned int pktnum)
{
	uint32_t cnt = INT_MITIG_PKTN_TO_CNT(pktnum);

	if (cnt < INT_MITIG_TIMEOUT_CNT_MIN)
		cnt = INT_MITIG_TIMEOUT_CNT_MIN;
	else if (cnt > INT_MITIG_TIMEOUT_CNT_MAX)
		cnt = INT_MITIG_TIMEOUT_CNT_MAX;
	return cnt;
}

static inline void set_tx_mitigation(struct vmac_priv *vmp,
		unsigned int pktnum)
{
	void *regbase = vmp->pcie_reg_base;
	uint32_t tm_cnt = pktnum_to_timeout_cnt(pktnum);

	if (vmac_fast_hdp(vmp)) {
		uint32_t tmp;
		tmp = vmac_readl(PCIE_HDP_FRM_CNT_THLD(regbase));
		tmp = HDP_FRM_RX2_THLD(tmp, pktnum);
		vmac_writel(tmp, PCIE_HDP_FRM_CNT_THLD(regbase));

		vmac_writel(tm_cnt, PCIE_HDP_RX2_TIMOUT_THLD(regbase));
		vmac_writel(INT_MITIG_EN, PCIE_HDP_RX_INT_CTRL(regbase));
	} else {
		vmac_writel(INT_MITIG_EN | INT_MITIG_NUM(pktnum) | (tm_cnt << 8),
			PCIE_HDP_RX_INT_CTRL(regbase));
	}
}

static inline void set_rx_mitigation(struct vmac_priv *vmp,
		unsigned int pktnum)
{
	void *regbase = vmp->pcie_reg_base;
	uint32_t tm_cnt = pktnum_to_timeout_cnt(pktnum);

	if (vmac_fast_hdp(vmp)) {
		uint32_t tmp;
		tmp = vmac_readl(PCIE_HDP_FRM_CNT_THLD(regbase));
		tmp = HDP_FRM_TX0_THLD(tmp, pktnum);
		vmac_writel(tmp, PCIE_HDP_FRM_CNT_THLD(regbase));

		vmac_writel(tm_cnt, PCIE_HDP_TX0_TIMOUT_THLD(regbase));
		vmac_writel(INT_MITIG_EN, PCIE_HDP_TX_INT_CTRL(regbase));
	} else {
		vmac_writel(INT_MITIG_EN | INT_MITIG_NUM(pktnum) | (tm_cnt << 8),
			PCIE_HDP_TX_INT_CTRL(regbase));
	}
}

static inline void vmac_set_intr_mask(struct vmac_priv *vmp, uint32_t msk)
{
	unsigned long flags;

	spin_lock_irqsave(&vmp->intr_lock, flags);
	vmp->pcie_int_en = msk;
	vmac_writel(msk, PCIE_HDP_INT_EN(vmp->pcie_reg_base));
	spin_unlock_irqrestore(&vmp->intr_lock, flags);
}

static void inline vmac_enable_intr(struct vmac_priv *vmp, uint32_t msk)
{
	unsigned long flags;
	uint32_t tmp;

	spin_lock_irqsave(&vmp->intr_lock, flags);
	tmp = vmp->pcie_int_en | msk;
	if (tmp != vmp->pcie_int_en) {
		vmac_writel(tmp, PCIE_HDP_INT_EN(vmp->pcie_reg_base));
		vmp->pcie_int_en = tmp;
	}
	spin_unlock_irqrestore(&vmp->intr_lock, flags);
}

static int alloc_bd_tbl(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	void *ucaddr;
	void *paddr;
	int len;	/* Length of allocated Transmitted & Received descriptor array */

	len = vmp->tx_bd_num * VMAC_TX_BD_LEN +
		vmp->rx_bd_num * VMAC_RX_BD_LEN +
		2 * PCIE_HDP_DMA_PTR_SIZE + sizeof(vmac_addr_t) +
		dma_get_cache_alignment();

	ucaddr = pci_alloc_consistent(vmp->pdev, len, (dma_addr_t *)&paddr);
	if (!ucaddr)
		return -1;

	memset((void *)ucaddr, 0, len);

	vmp->addr_uncache = ucaddr;
	vmp->uncache_len = len;

	/* Update pointers related with Tx descriptor table */
	vmp->tx_bd_base = (struct vmac_tx_bd *)ucaddr;
	vmp->tx_bd_phyb = (struct vmac_tx_bd *)paddr;
	qdpc_pcie_posted_write((uint32_t)(uintptr_t)paddr, &vmp->bda->bda_rc_tx_bd_base);
	printk(KERN_INFO
		"Tx Descriptor table: uncache virtual addr: %p paddr: %p\n",
	       vmp->tx_bd_base, paddr);

	/* Update pointers related with Rx descriptor table */
	ucaddr += vmp->tx_bd_num * VMAC_TX_BD_LEN;
	paddr += vmp->tx_bd_num * VMAC_TX_BD_LEN;

	vmp->rx_bd_base = (struct vmac_rx_bd *)ucaddr;
	vmp->rx_bd_phyb = (struct vmac_rx_bd *)paddr;

	printk(KERN_INFO
	       "Rx Descriptor table: uncache virtual addr: %p paddr: %p\n",
	       vmp->rx_bd_base, paddr);

	ucaddr += vmp->rx_bd_num * VMAC_RX_BD_LEN;
	paddr += vmp->rx_bd_num * VMAC_RX_BD_LEN;

	vmp->rx_dma_ptr_pa = paddr;
	vmp->rx_dma_ptr_va = ucaddr;
	ucaddr += PCIE_HDP_DMA_PTR_SIZE;
	paddr += PCIE_HDP_DMA_PTR_SIZE;

	vmp->tx_dma_ptr_pa = paddr;
	vmp->tx_dma_ptr_va = ucaddr;
	ucaddr += PCIE_HDP_DMA_PTR_SIZE;
	paddr += PCIE_HDP_DMA_PTR_SIZE;

	vmp->intr_status_pa = paddr;
	vmp->intr_status_va = ucaddr;
	ucaddr += sizeof(vmac_addr_t);
	if (ucaddr - vmp->addr_uncache > len) {
		panic("ERROR: PCIe shared memory is not enough!!!\n");
		return -1;
	}

	return 0;
}

static void free_bd_tbl(struct vmac_priv *vmp)
{
	pci_free_consistent(vmp->pdev, vmp->uncache_len,
			    (void *)vmp->addr_uncache,
			    (dma_addr_t)(uintptr_t)vmp->tx_bd_phyb);
}

static int alloc_skb_desc_array(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	void *addr;
	int len;

	len = vmp->tx_bd_num * sizeof(struct tx_buf_info_t) +
		vmp->rx_bd_num * sizeof(struct rx_buf_info_t);

	addr = kzalloc(len, VMAC_ALLOC_FLAG);
	if (!addr)
		return -1;
	vmp->tx_buf_info = addr;
	addr += vmp->tx_bd_num * sizeof(struct tx_buf_info_t);
	vmp->rx_buf_info = addr;

	return 0;
}

static void free_pkt_info(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);

	kfree(vmp->tx_buf_info);
}

#ifdef QTN_SKB_RECYCLE_SUPPORT
static inline struct sk_buff *__vmac_rx_skb_freelist_pop(struct vmac_priv *vmp)
{
	struct sk_buff *skb = __skb_dequeue(&vmp->rx_skb_freelist);

	return skb;
}

static inline int vmac_rx_skb_freelist_push(struct vmac_priv *vmp,
					    dma_addr_t buff_addr,
					    struct sk_buff *skb)
{
	unsigned long flag;

	if (skb_queue_len(&vmp->rx_skb_freelist) > QTN_RX_SKB_FREELIST_MAX_SIZE) {
		pci_unmap_single(vmp->pdev, buff_addr, skb->len,
				 (int)DMA_BIDIRECTIONAL);
		dev_kfree_skb(skb);
		vmp->vmac_skb_free++;
		return 0;
	}

	/* check for undersize skb; this should never happen, and indicates problems elsewhere */
	if (unlikely((skb_end_pointer(skb) - skb->head) < QTN_RX_BUF_MIN_SIZE)) {
		pci_unmap_single(vmp->pdev, buff_addr, skb->len,
				 (int)DMA_BIDIRECTIONAL);
		dev_kfree_skb(skb);
		vmp->vmac_skb_free++;
		vmp->skb_recycle_failures++;
		return -EINVAL;
	}

	skb->len = 0;
	skb->tail = skb->data = skb->head;
	skb_reserve(skb, NET_SKB_PAD);
	skb_reserve(skb, align_buf_dma_offset(skb->data));

	qtn_spin_lock_bh_save(&vmp->rx_skb_freelist_lock, &flag);
	__skb_queue_tail(&vmp->rx_skb_freelist, skb);
	qtn_spin_unlock_bh_restore(&vmp->rx_skb_freelist_lock, &flag);

	vmp->skb_recycle_cnt++;

	return 0;
}

static inline void __vmac_rx_skb_freelist_refill(struct vmac_priv *vmp)
{
	struct sk_buff *skb = NULL;
	int num =
	    vmp->rx_skb_freelist_fill_level -
	    skb_queue_len(&vmp->rx_skb_freelist);

	while (num > 0) {
		if (!(skb = VMAC_DEV_ALLOC_SKB(SKB_BUF_SIZE))) {
			vmp->rx_skb_alloc_failures++;
			break;
		}
		/* Move skb->data to a cache line boundary */
		skb_reserve(skb, align_buf_dma_offset(skb->data));
		pci_map_single(vmp->pdev, skb->data,
			       skb_end_pointer(skb) - skb->data,
			       (int)DMA_FROM_DEVICE);
		__skb_queue_tail(&vmp->rx_skb_freelist, skb);

		num--;
	}
}

static void vmac_rx_skb_freelist_purge(struct vmac_priv *vmp)
{
	unsigned long flag;

	qtn_spin_lock_bh_save(&vmp->rx_skb_freelist_lock, &flag);
	__skb_queue_purge(&vmp->rx_skb_freelist);
	qtn_spin_unlock_bh_restore(&vmp->rx_skb_freelist_lock, &flag);
}
#endif				/* QTN_SKB_RECYCLE_SUPPORT */

static inline bool check_netlink_magic(qdpc_cmd_hdr_t * cmd_hdr)
{
	return ((memcmp(cmd_hdr->dst_magic, QDPC_NETLINK_DST_MAGIC, ETH_ALEN) ==
		 0)
		&& (memcmp(cmd_hdr->src_magic, QDPC_NETLINK_SRC_MAGIC, ETH_ALEN)
		    == 0));
}

static void vmac_netlink_rx(struct net_device *ndev, void *buf, size_t len,
			    uint16_t rpc_type, uint32_t total_len)
{
	struct vmac_priv *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int pid = 0;
	int frag = (rpc_type & QDPC_RPC_TYPE_FRAG_MASK);

	rpc_type &= QDPC_RPC_TYPE_MASK;

	if (unlikely(total_len > VMAC_NL_BUF_SIZE)) {
		printk(KERN_INFO
		       "%s: total length %u exceeds buffer length %u\n",
		       __func__, total_len, VMAC_NL_BUF_SIZE);
		goto reset_nlbuf;
	}

	if (unlikely(priv->nl_len + len > total_len)) {
		printk(KERN_INFO "%s: frag length %u exceeds total length %u\n",
		       __func__, (unsigned int)(priv->nl_len + len), total_len);
		goto reset_nlbuf;
	}

	memcpy(priv->nl_buf + priv->nl_len, buf, len);
	priv->nl_len += len;

	if (frag)
		return;

	/* last fragment -- hand it to upper layer */
	buf = priv->nl_buf;
	len = priv->nl_len;

	skb = nlmsg_new(len, GFP_ATOMIC);
	if (skb == NULL) {
		if (printk_ratelimit())
			printk("WARNING: out of netlink SKBs\n");
		goto reset_nlbuf;
	}

	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, len, 0);;
	memcpy(nlmsg_data(nlh), buf, len);
	NETLINK_CB(skb).dst_group = 0;

	if (rpc_type == QDPC_RPC_TYPE_STRCALL)
		pid = priv->str_call_nl_pid;
	else if (rpc_type == QDPC_RPC_TYPE_LIBCALL)
		pid = priv->lib_call_nl_pid;

	if (unlikely(pid == 0)) {
		kfree_skb(skb);
		goto reset_nlbuf;
	}

	nlmsg_unicast(priv->nl_socket, skb, pid);

 reset_nlbuf:
	priv->nl_len = 0;
}

static int vmac_rx_is_netlink_pkt(void *pkt_header)
{
	struct ethhdr *eth = (struct ethhdr *)(pkt_header);

	return (ntohs(eth->h_proto) == QDPC_APP_NETLINK_TYPE);
}

#ifdef QTN_RC_ENABLE_HDP
void vmac_rx_netlink_pkt(struct net_device *ndev, dma_addr_t baddr, int len)
#else
void vmac_rx_netlink_pkt(struct net_device *ndev, struct sk_buff *skb)
#endif
{
	qdpc_cmd_hdr_t *cmd_hdr;
	unsigned char *data;
#ifdef QTN_RC_ENABLE_HDP
	data = (unsigned char *) bus_to_virt(baddr);
#else
	data = skb->data;
#endif

	/* Double Check if it's netlink packet */
	cmd_hdr = (qdpc_cmd_hdr_t *)data;
	if (check_netlink_magic(cmd_hdr)) {
		vmac_netlink_rx(ndev,
				data + sizeof(qdpc_cmd_hdr_t),
				ntohs(cmd_hdr->len),
				ntohs(cmd_hdr->rpc_type),
				ntohs(cmd_hdr->total_len));
	}

#ifdef QTN_RC_ENABLE_HDP
	inv_dcache_range((unsigned long)baddr, (unsigned long)baddr + len);
	topaz_hbm_put_buf((void *)baddr, TOPAZ_HBM_BUF_EMAC_RX_POOL);
#else
	dev_kfree_skb(skb);
#endif
}

#ifdef QTN_RC_ENABLE_HDP
static inline void vmac_rx_forward(struct vmac_priv *vmp, uint32_t bdata,
				   uint16_t len, const void *vdata)
{
	const struct ethhdr *eth;
	union tqe_cpuif_descr desc;
	fwt_db_entry *fwt_ent;
	struct sk_buff *skb;
	int push_count;
	uint16_t vlan_id = 0;
	/* FIXME-PEARL_VLAN - set vlan id in desc.data.vlan, desc.data.vlan_id, skb->hw_vlan_id? */

	eth = vdata;
	if (is_multicast_ether_addr(eth->h_dest)) {
		int8_t pool = topaz_hbm_payload_get_pool_bus((void *)bdata);
		union tqe_cpuif_descr *desc_p = &desc;

		memset(&desc, 0, sizeof(desc));
		TQE_DESCR_DATA_ITEM_SET(desc_p, buff_ptr_offset,
			topaz_hbm_payload_buff_ptr_offset_bus((void *)bdata, pool, NULL));
		desc.data.length = len;
		desc.data.in_port = TOPAZ_TQE_PCIE_REL_PORT;
		desc.data.pkt = (void *)bdata;

		push_count = tqe_rx_multicast(NULL, &desc);
		if (push_count > 0)
			return;
	} else {
		fwt_ent = vmac_get_tqe_ent(eth->h_source, eth->h_dest, vlan_id);
		if (likely(fwt_ent)) {
			topaz_pcie_tqe_xmit(fwt_ent, (void *)bdata, len);
			return;
		} else {
			vmp->fwt_loss_cnt++;
		}
	}

	skb =
	    topaz_hbm_attach_skb((void *)vdata, TOPAZ_HBM_BUF_EMAC_RX_POOL, 0);
	if (likely(skb)) {
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, vmp->ndev);
		skb->src_port = 0;
		netif_receive_skb(skb);
	} else {
		printk(KERN_ERR "Failed to attach skb\n");
		topaz_hbm_put_buf(topaz_hbm_payload_store_align_bus
				  ((void *)bdata, TOPAZ_HBM_BUF_EMAC_RX_POOL,
				   0), TOPAZ_HBM_BUF_EMAC_RX_POOL);
	}
}
#endif
 
#if defined(CONFIG_FC_QTNA_WIFI_AX)
extern int rtk_fc_fastfwd_netif_rx(struct sk_buff *skb);
#endif

static inline bool vmac_rxpkt_handup(struct net_device *ndev,
		struct rx_buf_info_t *rxinfo, int len)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
#ifdef QTN_RC_ENABLE_HDP
	uintptr_t baddr = rxinfo->pa;
	char *vdata = (char *)bus_to_virt(baddr);

	if (!baddr)
		return false;

	if (unlikely(vmac_rx_is_netlink_pkt(vdata))) {
		vmac_rx_netlink_pkt(ndev, baddr, len);
		return false;
	} else {
		dump_rx_pkt(vmp, vdata, len);
		vmac_rx_forward(vmp, baddr, len, (void *)vdata);
	}
#else
	struct sk_buff *skb = rxinfo->skb;
	if (!skb)
		return false;

	skb_put(skb, len);
	pci_unmap_single(vmp->pdev, rxinfo->pa,
		skb_end_pointer(skb) - skb->data,
#ifdef QTN_SKB_RECYCLE_SUPPORT
		(int)DMA_BIDIRECTIONAL
#else
		(int)DMA_FROM_DEVICE
#endif
		);

	if (unlikely(vmac_rx_is_netlink_pkt(skb->data))) {
		vmac_rx_netlink_pkt(ndev, skb);
		return false;
	}

	dump_rx_pkt(vmp, (char *)skb->data, (int)skb->len);
	skb->protocol = eth_type_trans(skb, ndev);
#if defined(CONFIG_FC_QTNA_WIFI_AX)
        rtk_fc_fastfwd_netif_rx(skb);
#else
	netif_receive_skb(skb);
#endif

#endif
	return true;
}

/*
 * HDP may fail to update the Rx Descriptor, possibly caused by unstable link
 * It causes the circular buffer ring stucked at current descriptor,
 * Which ultimately cause the HHBM buffer underflow and traffic stop
 * So, drop the packet and keep buffer ring going forward.
 */
static noinline bool vmac_skip_bad_rbd(struct vmac_priv *vmp,
				struct rx_buf_info_t *rxinfo)
{
	uint32_t wr_index;
	uint32_t rd_index;
	uint32_t tmp;
	void *regbase = vmp->pcie_reg_base;

	vmp->rx_buf_unflow = false;
	wr_index = vmac_readl(PCIE_HDP_TX_HOST_Q_WR_PTR(regbase));
	rd_index = vmac_readl(PCIE_HDP_TX_HOST_Q_RD_PTR(regbase));

	tmp = (wr_index - rd_index) & vmp->rx_bd_num_msk;
	if ((tmp == 0) && ((rd_index ^ wr_index) & VMAC_BD_WRAP))
		tmp = vmp->rx_bd_num_msk;

	if (tmp < VMAC_HHBM_UNFLOW_TH) {
#ifdef QTN_RC_ENABLE_HDP
		uintptr_t baddr = rxinfo->pa;
		inv_dcache_range((unsigned long)baddr, (unsigned long)baddr +
			TOPAZ_HBM_BUF_EMAC_RX_SIZE);
		topaz_hbm_put_buf((void *)baddr, TOPAZ_HBM_BUF_EMAC_RX_POOL);
#else
		dev_kfree_skb(rxinfo->skb);
#endif
		vmp->ndev->stats.rx_errors++;
		return true;
	}

	return false;
}

void inline vmac_rx_update_hw_index(struct vmac_priv *vmp, uint16_t new)
{
	uint16_t ihw;

	if (new == vmp->rx_bd_index)
		return;

	ihw = new | (vmp->rx_prehw_index & VMAC_BD_WRAP);
	if (ihw  < vmp->rx_prehw_index)
		ihw ^= VMAC_BD_WRAP;

	if (vmac_fast_hdp(vmp)) {
		vmac_writel(ihw | ((ihw ^ VMAC_BD_WRAP) << 16), PCIE_HDP_TX0_DESC_Q_WR_PTR(vmp->pcie_reg_base));
	} else {
		vmac_writel(ihw, PCIE_HDP_TX_HOST_Q_WR_PTR(vmp->pcie_reg_base));
	}

	vmp->rx_bd_index = new;
	vmp->rx_prehw_index = ihw;
}

static int __sram_text vmac_rx_poll(struct napi_struct *napi, int budget)
{
	struct vmac_priv *vmp = container_of(napi, struct vmac_priv, napi);
	struct net_device *ndev = vmp->ndev;
	int processed = 0;
	uint16_t i = vmp->rx_bd_index;
	volatile struct vmac_rx_bd *rbdp;
	struct rx_buf_info_t *rxinfo;
	uint32_t desc_info;
	int len;

	while (processed < budget) {
		rbdp = &vmp->rx_bd_base[i];
		rxinfo = &vmp->rx_buf_info[i];
		desc_info = vmac_readl(&rbdp->buff_info);
		len = VMAC_GET_LEN(desc_info);

		if (desc_info & VMAC_TXDONE_MSK) { /* normal case */
			if (true == vmac_rxpkt_handup(ndev, rxinfo, len)) {
				processed++;
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
			}
		}  else { /* no more packet */
			if (vmac_fast_hdp(vmp) || vmp->rx_buf_unflow == false ||
					vmac_skip_bad_rbd(vmp, rxinfo) == false) {
				vmp->rx_done_cnt++;
				break;
			}
		}
		dump_rx_bd(vmp);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
		ndev->last_rx = jiffies;
#endif
		/*
		 * We are done with the current buffer attached to this descriptor,
		 * so attach a new one.
		 */
		if (skb2rbd_attach(ndev, i) == 0) {
			i =  VMAC_RX_INDEX_INC(vmp, i, 1);
			rbdp = &vmp->rx_bd_base[i];
		} else {
			break;
		}
	}
	vmac_rx_update_hw_index(vmp, i);

	if (processed < budget) {
		if (processed == 0)
			vmp->rx_napi_no_pkt++;
		napi_complete(napi);
		vmac_enable_intr(vmp, VMAC_INT_NAPI_BITS);
	}

#ifdef QTN_SKB_RECYCLE_SUPPORT
	spin_lock(&vmp->rx_skb_freelist_lock);
	__vmac_rx_skb_freelist_refill(vmp);
	spin_unlock(&vmp->rx_skb_freelist_lock);
#endif
	return processed;
}

static int __sram_text skb2rbd_attach(struct net_device *ndev,
					uint16_t rx_bd_index)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	volatile struct vmac_rx_bd *rbdp;
	dma_addr_t buff_addr;

#ifdef QTN_RC_ENABLE_HDP
	buff_addr =
	    (uint32_t)topaz_hbm_get_payload_bus(TOPAZ_HBM_BUF_EMAC_RX_POOL);
#else /* !QTN_RC_ENABLE_HDP */
	struct sk_buff *skb = NULL;
#ifdef QTN_SKB_RECYCLE_SUPPORT
	spin_lock(&vmp->rx_skb_freelist_lock);
	if (unlikely(!(skb = __vmac_rx_skb_freelist_pop(vmp)))) {
		spin_unlock(&vmp->rx_skb_freelist_lock);
		vmp->rx_buf_info[rx_bd_index].skb = NULL; /* prevent old packet from passing the packet up */
		return -1;
	}
	spin_unlock(&vmp->rx_skb_freelist_lock);
	buff_addr = virt_to_bus(skb->data);
#else
	if (!(skb = VMAC_DEV_ALLOC_SKB(SKB_BUF_SIZE))) {
		vmp->rx_skb_alloc_failures++;
		vmp->rx_buf_info[rx_bd_index].skb = NULL;/* prevent old packet from passing the packet up */
		return -1;
	}
		/* Move skb->data to a cache line boundary */
	skb_reserve(skb, align_buf_dma_offset(skb->data) + 2);
		/* Invalidate cache and map virtual address to bus address. */
	buff_addr = pci_map_single(vmp->pdev, skb->data,
				   skb_end_pointer(skb) - skb->data,
				   (int)DMA_FROM_DEVICE);
#endif				/* QTN_SKB_RECYCLE_SUPPORT */
	skb->dev = ndev;
	vmp->rx_buf_info[rx_bd_index].skb = skb;
#endif /* end of QTN_RC_ENABLE_HDP */

	vmp->rx_buf_info[rx_bd_index].pa = buff_addr;

	rbdp = &vmp->rx_bd_base[rx_bd_index];

	if (vmac_fast_hdp(vmp)) {
		vmac_set_bd_paddr(buff_addr, rbdp);
	} else {
		/* PCIE_HDP_HHBM_BUF_PTR_H should be written
		 * in advance of low 32 bit to avoid race condition */
		vmac_writell_hi(buff_addr, PCIE_HDP_HHBM_BUF_PTR_H(vmp->pcie_reg_base));
		vmac_writel(buff_addr, PCIE_HDP_HHBM_BUF_PTR(vmp->pcie_reg_base));
	}
	rbdp->buff_info = 0;

	return 0;
}

static inline void vmac_try_wake_txqueue(struct net_device *ndev)
{
	if (unlikely(netif_queue_stopped(ndev)) &&
				likely(netif_running(ndev))) {
		struct vmac_priv *vmp = netdev_priv(ndev);
		if (((vmp->tx_bd_index - vmp->tx_done_index) & vmp->tx_bd_num_msk) <
				vmp->tx_wake_q_threshold)
			netif_wake_queue(ndev);
	}
}

/* Increase both sw tx_bd_index and hw index */
void inline vmac_tx_bd_index_inc(struct vmac_priv *vmp)
{
	unsigned long flags;
	void *regbase = vmp->pcie_reg_base;
	uint16_t i;

	spin_lock_irqsave(&vmp->tx_index_lock, flags);
	if (vmac_fast_hdp(vmp)) {
		uint32_t ihw;
		i = VMAC_TX_INDEX_INC(vmp, vmp->tx_bd_index, 1);
		ihw = i | (vmp->tx_prehw_index & VMAC_BD_WRAP);

		if (ihw < vmp->tx_prehw_index)
			ihw ^= VMAC_BD_WRAP;

		vmac_writel(ihw | (vmp->tx_done_index << 16) | vmp->tx_host_ack_wrap, PCIE_HDP_RX2_DESC_Q_WR_PTR(regbase));
		vmp->tx_prehw_index = ihw;
	} else {
		i = vmp->tx_bd_index;
		vmac_writell_hi(&vmp->tx_bd_phyb[i], PCIE_HDP_HOST_WR_DESC0_H(regbase));
		vmac_writel(&vmp->tx_bd_phyb[i], PCIE_HDP_HOST_WR_DESC0(regbase));
		i = VMAC_TX_INDEX_INC(vmp, i, 1);
	}
	vmp->tx_bd_index = i;
	spin_unlock_irqrestore(&vmp->tx_index_lock, flags);
}

/* Update the tx_done_index to new now and update the hw index if in irq*/
void inline vmac_tx_done_update(struct vmac_priv *vmp, uint16_t new)
{
	unsigned long flags;

	spin_lock_irqsave(&vmp->tx_index_lock, flags);

	if (vmac_fast_hdp(vmp) && in_irq()) {
		uint32_t ihw = (new << 16) | vmp->tx_host_ack_wrap | vmp->tx_prehw_index;
		vmac_writel(ihw, PCIE_HDP_RX2_DESC_Q_WR_PTR(vmp->pcie_reg_base));
	}
	vmp->tx_done_index = new;

	spin_unlock_irqrestore(&vmp->tx_index_lock, flags);
}


/*
 * Tear down the Tx BD after HDP RX DMA done, Free the skb to protocol stack
 */
static __attribute__ ((section(".sram.text")))
	void vmac_tx_teardown(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	volatile struct vmac_tx_bd *tbdp;
	uint16_t dma_done_num;
	uint16_t i;

	if (test_and_set_bit(__TXDONE_INPROGRESS, &vmp->state))
		return;

	i = vmp->tx_done_index;
	if (vmac_fast_hdp(vmp))
		dma_done_num = vmac_readw(vmp->tx_dma_ptr_va);
	else
		dma_done_num = (uint16_t)vmac_readl(PCIE_HDP_RX0DMA_CNT(vmp->pcie_reg_base));

	dma_done_num = (dma_done_num - i) & vmp->tx_bd_num_msk;

	while (dma_done_num-- != 0) {
		struct tx_buf_info_t *txinfo = &vmp->tx_buf_info[i];
#ifdef QTN_RC_ENABLE_HDP
		if (!txinfo->pa) {
#else
		struct sk_buff *skb;
		skb = txinfo->skb;
		if (!skb) {
#endif
			vmp->tx_free_err++;
			break;
		}
		tbdp = &vmp->tx_bd_base[i];

		ndev->stats.tx_packets++;

#ifdef QTN_RC_ENABLE_HDP
		if (txinfo->type == PKT_TQE) {
			topaz_hbm_put_buf((void *)txinfo->pa,
				TOPAZ_HBM_BUF_EMAC_RX_POOL);
		} else {
			pci_unmap_single(vmp->pdev, txinfo->pa,
				txinfo->len, (int)DMA_TO_DEVICE);
			dev_kfree_skb_irq(txinfo->skb);
		}
		ndev->stats.tx_bytes += txinfo->len;
		txinfo->pa = 0;
#else
		ndev->stats.tx_bytes += skb->len;
#ifdef QTN_SKB_RECYCLE_SUPPORT
		vmac_rx_skb_freelist_push(vmp, info->pa, skb);
#else
		pci_unmap_single(vmp->pdev, txinfo->pa,
				 skb->len, (int)DMA_TO_DEVICE);
		dev_kfree_skb_irq(skb);
#endif				/* QTN_SKB_RECYCLE_SUPPORT */
		txinfo->skb = NULL;
#endif				/* QTN_RC_ENABLE_HDP */

		if (in_irq())
			vmp->irq_txdone_cnt++;
		else
			vmp->xmit_txdone_cnt++;

		i = VMAC_TX_INDEX_INC(vmp, i, 1);
		if (i == 0)
			vmp->tx_host_ack_wrap ^= (VMAC_BD_WRAP << 16);
	}
	vmac_tx_done_update(vmp, i);
	clear_bit(__TXDONE_INPROGRESS, &vmp->state);
	/* Keep vmac_enable_intr out of txdone lock. */
	vmac_enable_intr(vmp, PCIE_HDP_INT_EP_RXDMA);
}


static dma_addr_t inline vmac_tx_skb_store(struct vmac_priv *vmp,
		struct tx_buf_info_t *txinfo, struct sk_buff *skb)
{
	uintptr_t baddr;

	txinfo->skb = skb;
	baddr = pci_map_single(vmp->pdev, skb->data, skb->len,
#if !defined(QTN_RC_ENABLE_HDP) && defined(QTN_SKB_RECYCLE_SUPPORT)
			(int)DMA_BIDIRECTIONAL
#else
 			(int)DMA_TO_DEVICE
#endif
			);
	txinfo->pa = baddr;
	return baddr;
}

void qdpc_pcie_notify_ep_new_packet(struct vmac_priv *priv)
{
	writel(TOPAZ_SET_INT(IPC_RC_NEW_PKT), priv->ep_ipc_reg);
}

#ifdef QTN_RC_ENABLE_HDP
static dma_addr_t inline vmp_hdp_pkt_store(struct vmac_priv *vmp,
		struct tx_buf_info_t *txinfo, void *pkt_handle,
		enum pkt_type pkt_type)
{
	uintptr_t baddr;
	int len;

	if (pkt_type == PKT_TQE) {
		union pearl_tqe_pcieif_descr *tqe_desc
		    = (union pearl_tqe_pcieif_descr *)pkt_handle;
		baddr = (dma_addr_t)tqe_desc->data.pkt;
		txinfo->pa = baddr;
		len = tqe_desc->data.length;
	} else {
		struct sk_buff *skb = pkt_handle;
		baddr = vmac_tx_skb_store(vmp, txinfo, skb);
		len = skb->len;
	}
	txinfo->type= pkt_type;
	txinfo->len = len;
	return baddr;
}

static int vmac_tx(void *pkt_handle, struct net_device *ndev, enum pkt_type pkt_type);
int vmac_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	return vmac_tx((void *)skb, ndev, PKT_SKB);
}

void vmac_hdp_tx(void *pkt_handle, struct net_device *ndev)
{
	(void)vmac_tx(pkt_handle, ndev, PKT_TQE);
}

static int __attribute__ ((section(".sram.text")))
    vmac_tx(void *pkt_handle, struct net_device *ndev, enum pkt_type pkt_type)
#else /* !QTN_RC_ENABLE_HDP */
int __attribute__ ((section(".sram.text")))
    vmac_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
#endif
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	uint16_t i;	/* tx index */
	volatile struct vmac_tx_bd *tbdp;	/* Tx BD pointer */
	struct tx_buf_info_t *txinfo;
	int len;
	uintptr_t baddr;
#ifdef	QTN_RC_ENABLE_HDP
	union pearl_tqe_pcieif_descr *tqe_desc = (union pearl_tqe_pcieif_descr *)pkt_handle;
	struct sk_buff *skb = (struct sk_buff *)pkt_handle;
#endif

#ifdef QTN_RC_ENABLE_HDP
	if (pkt_type == PKT_TQE)
		len = tqe_desc->data.length;
	else
		len = skb->len;
#else
	len = skb->len;
#endif

	if (len > vmp->dma_threshold) {
		vmp->tx_oversize_cnt++;
#ifdef QTN_RC_ENABLE_HDP
		if (pkt_type == PKT_TQE) {
			baddr = (dma_addr_t)tqe_desc->data.pkt;
			topaz_hbm_put_buf((void *)baddr, TOPAZ_HBM_BUF_EMAC_RX_POOL);
		} else
			dev_kfree_skb_any(skb);
#else
		dev_kfree_skb_any(skb);
#endif
		return NETDEV_TX_OK;
	}

	i = vmp->tx_bd_index;

	if (vmp->ep_pmstate == PCI_D3hot && vmp->ep_under_low_pwr == true) {
		qdpc_pcie_exit_low_power(vmp);
		qdpc_pcie_notify_ep_new_packet(vmp);
	}
	/*
	 * if the pending packet is more than VMAC_TX_PENDING_TH,
	 * check if the packets were transferred
	 */
	if ((vmp->tx_bd_num_msk & (i - vmp->tx_done_index)) > tx_pending_th)
		vmac_tx_teardown(ndev);
	/*
	 * can NOT increase tx_bd_index to be equal tx_done_index
	 * but tx_done_index can catch up with tx_bd_index (be equal)
	 */
	if (unlikely(VMAC_TX_INDEX_INC(vmp, i, 1) == vmp->tx_done_index)) {
		vmp->tx_bd_busy_cnt++;
		return NETDEV_TX_BUSY;
	}

#ifndef QTN_RC_ENABLE_HDP
	if (VMAC_TX_INDEX_INC(vmp, i, 2) == vmp->tx_done_index) {
		netif_stop_queue(ndev);
		vmp->tx_q_stop_cnt++;
	}
#endif

	tbdp = &vmp->tx_bd_base[i];
	txinfo = &vmp->tx_buf_info[i];

#ifdef QTN_RC_ENABLE_HDP
	baddr = vmp_hdp_pkt_store(vmp, txinfo, pkt_handle, pkt_type);
	len = txinfo->len;
#else
	baddr = vmac_tx_skb_store(vmp, txinfo, skb);
	len = skb->len;
#endif
	vmac_set_bd_paddr(baddr, tbdp);
	vmac_writel(len, &tbdp->buff_info);

	wmb();

	dump_tx_pkt(vmp, bus_to_virt(baddr), len);
	dump_tx_bd(vmp);

	vmac_tx_bd_index_inc(vmp);
	vmp->tx_pending_cnt++;

	return NETDEV_TX_OK;
}

static void vmac_ipc_init(struct vmac_priv *vmp)
{
	void *msk_reg;

	if (vmp->dev_info & QTN_DEV_PCIE1)
		msk_reg = QDPC_BAR_VADDR(vmp->sysctl_bar, HDP_PCIE1_IPC_INT_MSK_OFF);
	else
		msk_reg = QDPC_BAR_VADDR(vmp->sysctl_bar, HDP_PCIE0_IPC_INT_MSK_OFF);
	/* Enable all */
	vmac_writel(0xffff, msk_reg);
}

static void vmac_update_carrier_status(struct vmac_priv *vmp)
{
	__iomem qdpc_pcie_bda_t *bda = vmp->bda;
	 if (vmac_readl(&bda->bda_ep_state) & QDPC_EP_RX_READY)
		netif_carrier_on(vmp->ndev);
	else
		netif_carrier_off(vmp->ndev);
}

static void vmac_put_ep_low_pwr(struct work_struct *data)
{
	struct vmac_priv *vmp = container_of(data, struct vmac_priv, low_pwr_work);
	pm_message_t state;

	state.event = 0;
	qdpc_pcie_suspend(vmp->pdev, state);
}

static noinline void vmac_ipc_handler(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	void *ipc_reg;
	uint32_t ipc;

	if (vmp->dev_info & QTN_DEV_PCIE1)
		ipc_reg = QDPC_BAR_VADDR(vmp->sysctl_bar, HDP_PCIE1_IPC_INT_OFF);
	else
		ipc_reg = QDPC_BAR_VADDR(vmp->sysctl_bar, HDP_PCIE0_IPC_INT_OFF);

	ipc = vmac_readl(ipc_reg);
	if (!ipc)
		return;

	vmac_writel(IPC_CLR_VALUE(ipc), ipc_reg);
	vmac_writel(PCIE_HDP_INT_IPC, PCIE_HDP_INT_STATUS(vmp->pcie_reg_base));

	if (ipc & IPC_BIT_EP_IDLE)
		qdpc_pcie_enter_low_power(vmp);

	if (ipc & IPC_BIT_EP_EXIT_IDLE)
		qdpc_pcie_exit_low_power(vmp);

	if (ipc & IPC_BIT_EP_LINK)
		vmac_update_carrier_status(vmp);

	if (ipc & IPC_BIT_EP_REBOOT)
		handle_ep_rst_int(ndev);

	vmac_enable_intr(vmp, PCIE_HDP_INT_IPC);
}

static inline uint32_t vmac_get_intr_and_ack(struct vmac_priv *vmp)
{
	unsigned long flags;
	uint32_t intflag;
	void *regbase = vmp->pcie_reg_base;

	spin_lock_irqsave(&vmp->intr_lock, flags);
	/* mask all interrupt events, clear the status */
        vmac_writel(0, PCIE_HDP_INT_EN(regbase));
	intflag = vmac_readl(PCIE_HDP_INT_STATUS(regbase)) & vmp->pcie_int_en;
	vmac_writel(intflag, PCIE_HDP_INT_STATUS(regbase));

	/*
	 * Only disable the events that generate,
	 * will enable when the events are processed
	 */
	vmp->pcie_int_en &= ~intflag;
	vmac_writel(vmp->pcie_int_en, PCIE_HDP_INT_EN(regbase));
	spin_unlock_irqrestore(&vmp->intr_lock, flags);
	return intflag;
}

static irqreturn_t vmac_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct vmac_priv *vmp = netdev_priv(ndev);
	uint32_t intflag;

	vmp->intr_cnt++;

	if (!(intflag = vmac_get_intr_and_ack(vmp)))
		return IRQ_HANDLED;

	/* Deassert remote INTx message */
	if (!vmp->msi_enabled)
		qdpc_deassert_intx(vmp);

	if (unlikely(intflag & PCIE_HDP_INT_IPC))
		vmac_ipc_handler(ndev);

	if (intflag & PCIE_HDP_INT_EP_RXDMA) {
		vmp->tx_intr_cnt++;
		vmac_tx_teardown(ndev);
#ifndef QTN_RC_ENABLE_HDP
		vmac_try_wake_txqueue(ndev);
#endif
	}

	if (intflag & (VMAC_INT_NAPI_BITS)) {
		napi_schedule(&vmp->napi);

		if (intflag & PCIE_HDP_INT_EP_TXDMA)
			vmp->rx_intr_cnt++;

		if (unlikely(intflag & PCIE_HDP_INT_EP_TXEMPTY)) {
			vmp->intr_rx_no_dsc++;
			vmp->rx_buf_unflow = true;
		}

		if (unlikely(intflag & PCIE_HDP_INT_HHBM_UF)) {
			vmp->intr_rx_no_buf++;
			vmp->rx_buf_unflow = true;
		}
	}

	dump_rx_int(vmp);

	return IRQ_HANDLED;
}

/*
 * The Tx ring has been full longer than the watchdog timeout
 * value. The transmitter must be hung?
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static void vmac_tx_timeout(struct net_device *ndev, unsigned int txqueue)
#else
static void vmac_tx_timeout(struct net_device *ndev)
#endif
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	vmp->tx_timeout_cnt++;
	printk(KERN_ERR "%s: vmac_tx_timeout: ndev=%p\n", ndev->name, ndev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	netif_trans_update(ndev);
#else
	ndev->trans_start = jiffies;
#endif
	vmac_try_wake_txqueue(ndev);
}

/* ethtools support */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
static int vmac_get_link_ksettings(struct net_device *ndev, struct ethtool_link_ksettings *cmd)
{
	return -EINVAL;
}

static int vmac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *cmd)
{
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	return -EINVAL;
}
#else
static int vmac_get_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	return -EINVAL;
}

static int vmac_set_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	return -EINVAL;
}
#endif

static int vmac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	return -EINVAL;
}

static void vmac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct vmac_priv *vmp = netdev_priv(ndev);

	strcpy(info->driver, QTN_HOST_DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	info->fw_version[0] = '\0';
	sprintf(info->bus_info, "%s %d", QTN_HOST_DRV_NAME, vmp->mac_id);
	info->regdump_len = 0;
}

static const struct ethtool_ops vmac_ethtool_ops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	.get_link_ksettings = vmac_get_link_ksettings,
	.set_link_ksettings = vmac_set_link_ksettings,
#else
	.get_settings = vmac_get_settings,
	.set_settings = vmac_set_settings,
#endif
	.get_drvinfo = vmac_get_drvinfo,
	.get_link = ethtool_op_get_link,
};

static const struct net_device_ops vmac_device_ops = {
	.ndo_open = vmac_open,
	.ndo_stop = vmac_close,
	.ndo_start_xmit = vmac_hard_start_xmit,
	.ndo_change_mtu = vmac_change_mtu,
	.ndo_do_ioctl = vmac_ioctl,
	.ndo_tx_timeout = vmac_tx_timeout,
	.ndo_set_mac_address = eth_mac_addr,
};

void vmac_set_dma_threshold(struct vmac_priv *vmp, uint32_t dma_threshold)
{
	uint32_t temp;

	temp = readl(IO_ADDRESS(PCIE_HDP_AXI_CTRL(vmp->pcie_reg_base)));
	temp &= ~PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH;
	temp |= (dma_threshold << PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH_S) &
		PCIE_HDP_AXI_CTRL_DMA_LEN_THRESH;
	writel(temp, IO_ADDRESS(PCIE_HDP_AXI_CTRL(vmp->pcie_reg_base)));

	vmp->dma_threshold = dma_threshold;
}

/*
 * @ndev: network interface device structure
 * @new_mtu: new value for new MTU size
 */
static int vmac_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	uint32_t max_frame_size = new_mtu + ETH_HLEN + VLAN_HLEN;
	uint32_t buf_size = min(NPU_MAX_BUF_SIZE, SKB_BUF_SIZE);

	if (new_mtu  < ETH_ZLEN || max_frame_size > buf_size) {
		printk(KERN_ERR "set mtu %d rejected - must be between %d and %d\n",
				new_mtu, ETH_ZLEN, (buf_size - ETH_HLEN - VLAN_HLEN));
		return -EINVAL;
	}

	ndev->mtu = new_mtu;

	vmac_set_dma_threshold(vmp, max_frame_size);
	return 0;
}

struct net_device *vmac_alloc_ndev(void)
{
	struct net_device *ndev;

	/* Allocate device structure */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
	ndev =
	    alloc_netdev(sizeof(struct vmac_priv), vmaccfg.ifname, 0,
			 ether_setup);
#else
	ndev =
	    alloc_netdev(sizeof(struct vmac_priv), vmaccfg.ifname, ether_setup);
#endif
	if (!ndev)
		printk(KERN_ERR "%s: alloc_etherdev failed\n", vmaccfg.ifname);

	return ndev;
}

EXPORT_SYMBOL(vmac_alloc_ndev);

static void eth_parse_enetaddr(const char *addr, uint8_t * enetaddr)
{
	char *end;
	int i;

	for (i = 0; i < 6; ++i) {
		enetaddr[i] = addr ? simple_strtoul(addr, &end, 16) : 0;
		if (addr)
			addr = (*end) ? end + 1 : end;
	}
}


#if defined(VMAC_64BIT)
static void __init vmac_hhbm_64bit(struct vmac_priv *vmp)
{
	uint32_t tmp;

	tmp = vmac_readl(PCIE_HHBM_CONFIG(vmp->pcie_reg_base));
	tmp |= HHBM_CONFIG_SOFT_RESET;
	vmac_writel(tmp, PCIE_HHBM_CONFIG(vmp->pcie_reg_base));
	udelay(10);
	tmp &= ~(HHBM_CONFIG_SOFT_RESET);
	vmac_writel((tmp | HHBM_64BIT), PCIE_HHBM_CONFIG(vmp->pcie_reg_base));
}
#endif

static int vmac_hhbm_check(struct vmac_priv *vmp)
{
#define MAX_EP_FW_READY_CHECK 500
	int max_bufs;
	int max_try = MAX_EP_FW_READY_CHECK;

	if (vmac_fast_hdp(vmp)) {
		uint32_t tmp;

		tmp = vmac_readl(PCIE_HHBM_CONFIG(vmp->pcie_reg_base));
		tmp |= HHBM_CONFIG_SOFT_RESET;
		vmac_writel(tmp, PCIE_HHBM_CONFIG(vmp->pcie_reg_base));
		return 0;
	}

#if defined(VMAC_64BIT)
	vmac_hhbm_64bit(vmp);
#endif
	do {
		max_bufs = vmac_readl(PCIE_HHBM_Q_LIMIT_REG(vmp->pcie_reg_base));
		if (max_bufs < vmp->rx_bd_num) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ / 20);
		} else
			return 0;
	} while (max_try--);

	printk(KERN_ERR "EP is not ready after %d check\n",
	       MAX_EP_FW_READY_CHECK);
	return 1;
}
static int vmac_dev_identify(struct vmac_priv *vmp)
{
	void *reg = QDPC_BAR_VADDR(vmp->sysctl_bar, QTN_SYS_CTL_CPU_VEC_OFF);
	uint32_t tmp = vmac_readl(reg);

	/* Get PCIe port is used, pcie0 or pcie1 */
	if (tmp & QTN_PCIE1_EN)
		vmp->dev_info |= QTN_DEV_PCIE1;

	/* Get the device version, C0 or B0 */
	reg = QDPC_BAR_VADDR(vmp->sysctl_bar, QTN_SYS_CTL_CSR_OFF);
	tmp = vmac_readl(reg) & QTN_CHIP_ID_MASK;

	/*
	 * Since BBIC5 C0, a faster PCIe data-path is adopted.
	 */
	if (tmp >= QTN_CHIP_ID_PEARL_C) {
		vmp->dev_info |= QTN_DEV_FAST_DATA_PATH;
		return 0;
	}

	if (tmp == QTN_CHIP_ID_PEARL_B)
		return 0;

	printk(KERN_ERR "%s: unsupported device ID %x\n", QTN_HOST_DRV_NAME, tmp);
	return -EIO;
}

static void __init vmac_pcie_hdp_b0_init(struct vmac_priv *vmp)
{
	void *regbase = vmp->pcie_reg_base;
	vmac_writel(vmp->rx_bd_phyb, PCIE_HDP_TX_HOST_Q_BASE_L(regbase));
	vmac_writell_hi(vmp->rx_bd_phyb, PCIE_HDP_TX_HOST_Q_BASE_H(regbase));
	vmac_writel((vmp->rx_bd_num | (sizeof(struct vmac_rx_bd)) << 16),
	       PCIE_HDP_TX_HOST_Q_SZ_CTRL(regbase));
}

void vmac_hdp_enable(struct vmac_priv *vmp, bool en)
{
	if (vmac_fast_hdp(vmp)) {
		void *regbase = vmp->pcie_reg_base;
		vmac_bit_op(HDP_DESC_FETCH_EN, en, PCIE_HDP_RX2_DESC_Q_CTRL(regbase));
		vmac_bit_op(HDP_DESC_FETCH_EN, en, PCIE_HDP_TX0_DESC_Q_CTRL(regbase));
	}
}

static void vmac_pcie_hdp_c0_init(struct vmac_priv *vmp)
{
	struct pci_dev *pdev = vmp->pdev;
	void *regbase = vmp->pcie_reg_base;
	int mps = pcie_get_mps(pdev);
	int mrrs = pcie_get_readrq(pdev);
	uint32_t tmp;

	/*
	 * HDP write burst size < MAX_PAYLOAD_SIZE (MPS)
	 * HDP read burst size < MAX_READ_REQUEST_SIZE (MRRS)
	 */
	tmp = vmac_readl(PCIE_HDP_AXI_MASTER_CTRL(regbase));
	if (mrrs > PCIE_HDP_AXI_BURST32_SIZE)
		tmp |= PCIE_HDP_AXI_EN_BURST32_READ;
	else
		tmp &= ~PCIE_HDP_AXI_EN_BURST32_WRITE;
	if (mps > PCIE_HDP_AXI_BURST32_SIZE)
		tmp |= PCIE_HDP_AXI_EN_BURST32_WRITE;
	else
		tmp &= ~PCIE_HDP_AXI_EN_BURST32_WRITE;
	vmac_writel(tmp, PCIE_HDP_AXI_MASTER_CTRL(regbase));

	/* Tx initialization */
	vmac_writel(RXDMA_INTERLEAVE | RXDMA_NEW | RXDMA_WPTR, PCIE_HDP_RXDMA_CTRL(regbase));
	vmac_writel(TXDMA_NEW, PCIE_HDP_TX_DMA_CTRL(regbase));

	vmac_writel(vmp->tx_bd_phyb, PCIE_HDP_RX2_DESC_BASE_ADDR(regbase));
	vmac_writell_hi(vmp->tx_bd_phyb, PCIE_HDP_RX2_DESC_BASE_ADDR_H(regbase));
	vmac_writel((vmp->tx_bd_num | (sizeof(struct vmac_rx_bd)) << 16),
			PCIE_HDP_RX2_DESC_Q_CTRL(regbase));
	vmac_writel(vmp->tx_dma_ptr_pa, PCIE_HDP_RX2_DEV_PTR_ADDR(regbase));
	vmac_writell_hi(vmp->tx_dma_ptr_pa, PCIE_HDP_RX2_DEV_PTR_ADDR_H(regbase));
	vmac_writew(vmp->tx_bd_index, PCIE_HDP_RX2_DESC_Q_WR_PTR(regbase));

	/* Rx initialization */
	vmac_writel(vmp->rx_bd_phyb, PCIE_HDP_TX0_DESC_BASE_ADDR(regbase));
	vmac_writell_hi(vmp->rx_bd_phyb, PCIE_HDP_TX0_DESC_BASE_ADDR_H(regbase));
	vmac_writel((vmp->rx_bd_num | (sizeof(struct vmac_rx_bd)) << 16),
			PCIE_HDP_TX0_DESC_Q_CTRL(regbase));
	vmac_writel(vmp->rx_dma_ptr_pa, PCIE_HDP_TX0_DEV_PTR_ADDR(regbase));
	vmac_writell_hi(vmp->rx_dma_ptr_pa, PCIE_HDP_TX0_DEV_PTR_ADDR_H(regbase));
}

static void vmac_pcie_hdp_init(struct vmac_priv *vmp)
{
	if (vmac_fast_hdp(vmp))
		vmac_pcie_hdp_c0_init(vmp);
	else
		vmac_pcie_hdp_b0_init(vmp);
	vmp->rx_bd_index = vmp->rx_bd_num_msk;
	vmp->rx_prehw_index = vmp->rx_bd_index;
	vmac_rx_update_hw_index(vmp, 0);
}

int vmac_net_init(struct pci_dev *pdev)
{
	struct vmac_priv *vmp = NULL;
	struct net_device *ndev = NULL;
	int err = -ENOMEM;
	__iomem qdpc_pcie_bda_t *bda;

	printk(KERN_INFO "%s version %s %s\n", QTN_HOST_DRV_NAME, DRV_VERSION,
	       DRV_AUTHOR);
	printk(KERN_INFO "txpkt_per_intr %d, rxpkt_per_intr %d, tx_pending_th %d\n",
			txpkt_per_intr, rxpkt_per_intr, tx_pending_th);

	ndev = (struct net_device *)pci_get_drvdata(pdev);
	if (!ndev)
		goto vnet_init_err_0;

	ndev->netdev_ops = &vmac_device_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	netdev_set_default_ethtool_ops(ndev, &vmac_ethtool_ops);
#else
	SET_ETHTOOL_OPS(ndev, &vmac_ethtool_ops);
#endif

	/* Initialize private data */
	vmp = netdev_priv(ndev);
	vmp->pdev = pdev;
	vmp->ndev = ndev;
	vmac_set_hdp_base(vmp);

	err = vmac_dev_identify(vmp);
	if (err < 0)
		goto vnet_init_err_0;

	vmp->pcfg = &vmaccfg;
	if (!vmac_fast_hdp(vmp)) {
		if (vmp->pcfg->tx_bd_num > QDPC_TX_MAX_BD_SIZE_B0)
			vmp->pcfg->tx_bd_num = QDPC_TX_MAX_BD_SIZE_B0;
		if (vmp->pcfg->rx_bd_num > QDPC_RX_MAX_BD_SIZE_B0)
			vmp->pcfg->rx_bd_num = QDPC_RX_MAX_BD_SIZE_B0;
	}

	vmp->tx_bd_num = rounddown_pow_of_two(vmp->pcfg->tx_bd_num);
	vmp->tx_bd_num_msk =  vmp->tx_bd_num - 1;
	vmp->rx_bd_num = rounddown_pow_of_two(vmp->pcfg->rx_bd_num);
	vmp->rx_bd_num_msk = vmp->rx_bd_num - 1;
	spin_lock_init(&vmp->tx_index_lock);
	vmp->tx_wake_q_threshold = 2 * (uint16_t)txpkt_per_intr;

#ifdef QTN_SKB_RECYCLE_SUPPORT
	spin_lock_init(&vmp->rx_skb_freelist_lock);
	skb_queue_head_init(&vmp->rx_skb_freelist);
	vmp->rx_skb_freelist_fill_level = QTN_RX_SKB_FREELIST_FILL_SIZE;
	vmp->skb_recycle_cnt = 0;
	vmp->skb_recycle_failures = 0;
#endif

	if (!vmac_fast_hdp(vmp) && vmp->tx_bd_num > PCIE_HHBM_MAX_SIZE) {
		printk("Error: The length of TX BD array is too much\n");
		goto vnet_init_err_0;
	}

	vmp->ep_ipc_reg = QDPC_BAR_VADDR(vmp->sysctl_bar, PEARL_IPC_OFFSET);

	ndev->irq = pdev->irq;

	ndev->if_port = QDPC_PLATFORM_IFPORT;
#ifdef QTN_RC_ENABLE_HDP
	ndev->if_port =  tqe_pcie_port_get();
#endif

	ndev->watchdog_timeo = VMAC_TX_TIMEOUT;

	bda = vmp->bda;
	qdpc_pcie_posted_write(vmp->tx_bd_num, &bda->bda_rc_tx_bd_num);

	memcpy(ndev->dev_addr, bda->bda_pcie_mac, ndev->addr_len);

	if (ethaddr)
		eth_parse_enetaddr(ethaddr, ndev->dev_addr);

	if (!is_valid_ether_addr(ndev->dev_addr))
		random_ether_addr(ndev->dev_addr);

	if (vmac_hhbm_check(vmp))
		goto vnet_init_err_0;

	vmp->pcie_int_en = 0;
	spin_lock_init(&vmp->intr_lock);

	/* Allocate Tx & Rx SKB descriptor array */
	if (alloc_skb_desc_array(ndev))
		goto vnet_init_err_0;

	/* Allocate and initialise Tx & Rx descriptor array */
	if (alloc_bd_tbl(ndev))
		goto vnet_init_err_1;

#ifdef QTN_SKB_RECYCLE_SUPPORT
	__vmac_rx_skb_freelist_refill(vmp);
#endif

	if (alloc_and_init_rxbuffers(ndev))
		goto vnet_init_err_2;

	vmac_pcie_hdp_init(vmp);


	set_rx_mitigation(vmp, rxpkt_per_intr);
	set_tx_mitigation(vmp, txpkt_per_intr);

	/* Initialize NAPI */
	netif_napi_add(ndev, &vmp->napi, vmac_rx_poll, napi_budget);

	/* Register device */
	if ((err = register_netdev(ndev)) != 0) {
		printk(KERN_ERR "%s: Cannot register net device, error %d\n",
		       QTN_HOST_DRV_NAME, err);
		goto vnet_init_err_3;
	}
	printk(KERN_INFO "%s: Vmac Ethernet found\n", ndev->name);

	/* Add the device attributes */
	err = sysfs_create_group(&ndev->dev.kobj, &vmac_attr_group);
	if (err) {
		printk(KERN_ERR "Error creating sysfs files\n");
	}

	enable_ep_rst_detection(ndev);

	INIT_WORK(&vmp->low_pwr_work, vmac_put_ep_low_pwr);
	vmp->ep_under_low_pwr = false;

#ifdef QTN_SKB_RECYCLE_SUPPORT
	__vmac_rx_skb_freelist_refill(vmp);
#endif

#ifdef QTN_RC_ENABLE_HDP
	tqe_lhost_add_pcie_handler(&vmac_hdp_tx, ndev);
#endif
	return 0;

 vnet_init_err_3:
	free_rx_skbs(vmp);
 vnet_init_err_2:
#ifdef QTN_SKB_RECYCLE_SUPPORT
	vmac_rx_skb_freelist_purge(vmp);
#endif
	free_bd_tbl(vmp);
 vnet_init_err_1:
	free_pkt_info(ndev);
 vnet_init_err_0:
	return err;
}
EXPORT_SYMBOL(vmac_net_init);

static void free_rx_skbs(struct vmac_priv *vmp)
{
	/* All Ethernet activity should have ceased before calling
	 * this function
	 */
	uint16_t i;
	for (i = 0; i < vmp->rx_bd_num; i++) {
#ifdef QTN_RC_ENABLE_HDP
		uintptr_t baddr = vmp->rx_bd_base[i].buff_addr;
		if (baddr)
			topaz_hbm_put_buf((void *)baddr,
					  TOPAZ_HBM_BUF_EMAC_RX_POOL);
#else
		if (vmp->rx_buf_info[i].skb) {
			dev_kfree_skb(vmp->rx_buf_info[i].skb);
			vmp->rx_buf_info[i].skb = NULL;
		}
#endif
	}
}

static void free_tx_skbs(struct vmac_priv *vmp)
{
	/* All Ethernet activity should have ceased before calling
	 * this function
	 */
	uint16_t i;
	for (i = 0; i < vmp->tx_bd_num; i++) {
#ifdef QTN_RC_ENABLE_HDP
		uint32_t baddr = vmp->tx_bd_base[i].buff_addr;
		if (baddr) {
			topaz_hbm_put_buf((void *)baddr,
					  TOPAZ_HBM_BUF_EMAC_RX_POOL);
			vmp->tx_buf_info[i].pa = 0;
		}
#else
		if (vmp->tx_buf_info[i].skb) {
			dev_kfree_skb(vmp->tx_buf_info[i].skb);
			vmp->tx_buf_info[i].skb = 0;
		}
#endif
	}
}

static int alloc_and_init_rxbuffers(struct net_device *ndev)
{
	uint16_t i;
	struct vmac_priv *vmp = netdev_priv(ndev);

	/* Allocate rx buffers */
	for (i = 0; i < vmp->rx_bd_num; i++) {
		if (skb2rbd_attach(ndev, i)) {
			return -1;
		}
	}

	return 0;
}

extern int qdpc_unmap_iomem(struct vmac_priv *priv);
void vmac_clean(struct net_device *ndev)
{
	struct vmac_priv *vmp;

	if (!ndev)
		return;

	vmp = netdev_priv(ndev);

//	vmac_writel(0x80000000, PCIE_HDP_CTRL(regbase));

	sysfs_remove_group(&ndev->dev.kobj, &vmac_attr_group);

	unregister_netdev(ndev);

	free_rx_skbs(vmp);
	free_tx_skbs(vmp);
	free_pkt_info(ndev);
#ifdef QTN_SKB_RECYCLE_SUPPORT
	vmac_rx_skb_freelist_purge(vmp);
#endif

	disable_ep_rst_detection(ndev);

	netif_napi_del(&vmp->napi);

	free_bd_tbl(vmp);
}

static void bring_up_interface(struct net_device *ndev)
{
	/* Interface will be ready to send/receive data, but will need hooking
	 * up to the interrupts before anything will happen.
	 */
	struct vmac_priv *vmp = netdev_priv(ndev);
	vmac_set_intr_mask(vmp, VMAC_INT_ALL_BITS);
	vmac_ipc_init(vmp);
	vmac_hdp_enable(vmp, 1);
}

static void shut_down_interface(struct net_device *ndev)
{
	struct vmac_priv *vmp = netdev_priv(ndev);
	/* Close down MAC and DMA activity and clear all data. */
	vmac_hdp_enable(vmp, 0);
	vmac_set_intr_mask(vmp, 0);
}

static int vmac_open(struct net_device *ndev)
{
	int retval = 0;
	struct vmac_priv *vmp = netdev_priv(ndev);
	int irq_flags = 0;

	if (!vmp->msi_enabled)
		irq_flags = IRQF_SHARED;

	bring_up_interface(ndev);

	napi_enable(&vmp->napi);

	vmac_update_carrier_status(vmp);

	netif_wake_queue(ndev);

	/* Todo: request_irq here */
	retval = request_irq(ndev->irq, &vmac_interrupt, irq_flags, ndev->name, ndev);
	if (retval) {
		printk(KERN_ERR "%s: unable to get IRQ %d\n",
		       ndev->name, ndev->irq);
		goto err_out;
	}

	return 0;
 err_out:
	napi_disable(&vmp->napi);
	return retval;
}

/*
 * When Data Link Status of EP experiences a change from DL_Active to DL_Inactive,
 * RC will learn the event and probably record event in Root Error Status register
 * and Error Source Identification register. After RC records the event, it will
 * interrupt CPU to handle this event. However, we can't locate the related code
 * inside CPU. So following check is added to confirm whether EP experiences a reset.
 */
static inline int vmac_is_ep_reset(struct pci_dev *pdev)
{
	uint32_t val = 0;

	if (pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val))
		return 0;

	if (((val & QDPC_BAR_ADDR_MASK) == 0) || val == QDPC_BAR_NOT_INIT)
		return 1;
	else
		return 0;
}

static int vmac_close(struct net_device *ndev)
{
	struct vmac_priv *const vmp = netdev_priv(ndev);

	napi_disable(&vmp->napi);

	if (!vmac_is_ep_reset(vmp->pdev))
		shut_down_interface(ndev);

	netif_stop_queue(ndev);

	free_irq(ndev->irq, ndev);

	return 0;
}

