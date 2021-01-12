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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

#ifndef __DRIVERS_NET_PEARL_VNET_H
#define __DRIVERS_NET_PEARL_VNET_H	1

#define ETH_TX_TIMEOUT (100*HZ)
#define MULTICAST_FILTER_LIMIT 64

#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>

#include <qdpc_config.h>
#include <topaz_netcom.h>

#define PROC_NAME_SIZE		(32)
#define VMAC_BD_FULL		((uint32_t)0x80000000)
#define VMAC_TXDONE_MSK		((uint32_t)0x80000000)
#define VMAC_BD_MASK_LEN	((uint32_t)0x0000FFFF)

#define VMAC_GET_LEN(x)		((x) & 0xFFFF)
#define VMAC_SET_LEN(len)	((len) & 0xFFFF)

#define VMAC_ADD_MSK(msk, index, inc)		((msk) & ((index) + (inc)))
#define VMAC_TX_INDEX_INC(vmp, index, inc)	VMAC_ADD_MSK((vmp)->tx_bd_num_msk, (index), (inc))
#define VMAC_RX_INDEX_INC(vmp, index, inc)	VMAC_ADD_MSK((vmp)->rx_bd_num_msk, (index), (inc))
#define VMAC_BD_WRAP		(0x8000)

/*
 * Helper macros handling memory mapped area access
 */
#define VMAC_REG_TST(reg, val) ( *((volatile unsigned int *)(reg)) & (val) )
#define VMAC_REG_SET(reg, val) { volatile unsigned int *r = (unsigned int *)(reg); *r = (*r | (val)); }
#define VMAC_REG_CLR(reg, val) { volatile unsigned int *r = (unsigned int *)(reg); *r = (*r & ~(val)); }
#define VMAC_REG_WRITE(reg, val) { *(volatile unsigned int *)reg = (val); }
#define VMAC_REG_READ(reg) {*(volatile unsigned int *)(reg); }

#define QTN_RC_TX_BUDGET		(16)
#define QTN_RC_TX_TASKLET_BUDGET	(32)

#define QTN_RX_SKB_FREELIST_FILL_SIZE	(1024)
#define QTN_RX_SKB_FREELIST_MAX_SIZE	(8192)
#define QTN_RX_BUF_MIN_SIZE		(1536)

#define VMAC_NL_BUF_SIZE		USHRT_MAX

#define VMAC_HHBM_UNFLOW_TH		20

#if defined(BITS_PER_LONG) && (BITS_PER_LONG == 64)
#define VMAC_64BIT
#endif

#define VMAC_ALLOC_FLAG (GFP_KERNEL)
#define VMAC_DEV_ALLOC_SKB(x) dev_alloc_skb((x))

#define vmac_readl(addr)	(le32_to_cpu(readl(addr)))


#if defined(VMAC_64BIT)
typedef uint64_t vmac_addr_t;
#define vmac_writel(val, addr)	do { \
		writel(cpu_to_le32((uint32_t)(uintptr_t)val), (addr)); \
	} while (0)

#define vmac_writell_hi(val, addr)	do { \
		writel(cpu_to_le32((uint64_t)(uintptr_t)(val) >> 32), (addr)); \
	} while (0)
#define vmac_mergell(lo, hi)	((lo) | (((uint64_t)(hi)) << 32))
#else /* !VMAC_64BIT */
typedef uint32_t vmac_addr_t;
#define vmac_writel(val, addr)	do { \
		writel(cpu_to_le32(val), (addr)); \
	} while (0)
#define vmac_writell_hi(val, addr)	do {} while (0)
#define vmac_mergell(lo, hi)		(lo)
#endif /* VMAC_64BIT */

#define vmac_bit_op(msk, set, addr) do { \
		uint32_t tmp = vmac_readl(addr); \
		if (!!(set)) \
			tmp |= (msk); \
		else \
			tmp &= ~(msk); \
		vmac_writel(tmp, (addr)); \
		} while (0)

#define vmac_writew(val, addr) do{ \
		writel(cpu_to_le16((uint16_t)(uintptr_t)(val)), (addr)); \
	} while (0)

#define vmac_readw(addr) (le16_to_cpu(readw(addr)))

#define vmac_get_bd_paddr(bd)	vmac_mergell((vmac_readl((bd)->buff_addr)), \
					vmac_readl((bd)->buff_addr_h))

#define vmac_set_bd_paddr(paddr, bd)	do { \
			vmac_writel(paddr, &(bd)->buff_addr); \
			vmac_writell_hi(paddr, &(bd)->buff_addr_h); \
	} while (0)


typedef struct qdpc_bar {
	void *b_vaddr;		/* PCIe bar virtual address */
	dma_addr_t b_busaddr;	/* PCIe bar physical address */
	size_t b_len;		/* Bar resource length */
	uint32_t b_offset;	/* Offset from start of map */
	uint8_t b_index;	/* Bar Index */
} qdpc_bar_t;

#define QDPC_BAR_VADDR(bar, off) ((bar).b_vaddr +(off))

struct vmac_cfg {
	uint16_t rx_bd_num;
	uint16_t tx_bd_num;
	char ifname[PROC_NAME_SIZE];
	struct net_device *dev;
};

#if defined(QTN_RC_ENABLE_HDP)
enum pkt_type {
	PKT_SKB = 0,
	PKT_TQE
};
#endif

struct tx_buf_info_t {
	struct sk_buff *skb;
	uintptr_t pa; /* packet buffer physical address */
#if defined(QTN_RC_ENABLE_HDP)
	uint16_t len;
	uint8_t type;
	uint8_t rsv;
#else
#endif
};

struct rx_buf_info_t {
#if !defined(QTN_RC_ENABLE_HDP)
	struct sk_buff *skb;
#endif
	uintptr_t pa; /* packet buffer physical address */
};

enum {
	__TXDONE_INPROGRESS = 0,
};

struct vmac_priv {
	/* packet buffer having post to PCIe TxDMA */
	struct tx_buf_info_t *tx_buf_info;
	volatile struct vmac_tx_bd *tx_bd_base;	/* Tx buffer descriptor, uncached */
	volatile struct vmac_tx_bd *tx_bd_phyb;	/* Tx buffer descriptor physical address */
	uint16_t tx_bd_index;
	uint16_t tx_done_index;
	uint16_t tx_bd_num;
	uint16_t tx_bd_num_msk;
	uint16_t tx_prehw_index;	/* Index value that RC last programs HOST_WR ptr   */
	uint32_t tx_host_ack_wrap;	/* Wrap bit used by RC to acknoledge EP		   */
	uint32_t dma_threshold;		/* PCIe HDP DMA threshold for both TX and RX */
	spinlock_t tx_index_lock;	/* protect tx_bd_index, tx_done_index and hw index */

	void *pcie_reg_base;
#define QTN_DEV_FAST_DATA_PATH		(1 << 0)
#define QTN_DEV_PCIE1			(1 << 1)
	uint32_t dev_info;
	void *ep_ipc_reg;

	uint32_t tx_pending_cnt;
	uint32_t tx_bd_busy_cnt;	/* tx BD unavailable */
	uint32_t tx_q_stop_cnt;
	uint16_t tx_wake_q_threshold;
	uint32_t tx_free_err;
	uint32_t tx_oversize_cnt;

	uint32_t tx_intr_cnt;
	uint32_t tx_timeout_cnt;
	unsigned long state;

	/* packet buffer having post to PCIe RxDMA */
	struct rx_buf_info_t *rx_buf_info;
	volatile struct vmac_rx_bd *rx_bd_base;	/* Rx buffer descriptor, uncached  */
	volatile struct vmac_rx_bd *rx_bd_phyb;	/* rx buffer descriptor physical address */
	uint16_t rx_bd_num;
	uint16_t rx_bd_num_msk;
	uint16_t rx_bd_index;
	uint16_t rx_prehw_index;

	vmac_addr_t *rx_dma_ptr_pa;	/* Rx HDP DMA done index physical address */
	vmac_addr_t *rx_dma_ptr_va;	/* Rx HDP DMA done index uncached address */

	vmac_addr_t *tx_dma_ptr_pa;	/* Tx HDP DMA done index physical address */
	vmac_addr_t *tx_dma_ptr_va;	/* Tx HDP DMA done index uncached address */

	vmac_addr_t *intr_status_pa;	/* HDP interrupt status physical address */
	vmac_addr_t *intr_status_va;	/* HDP interrupt status uncached address */

	uint32_t rx_intr_cnt;
	uint32_t rx_done_cnt;
	uint32_t rx_napi_no_pkt;

	uint32_t rx_skb_alloc_failures;
	bool rx_buf_unflow;

#ifdef QTN_RC_ENABLE_HDP
	uint32_t fwt_loss_cnt;
#endif
	uint32_t irq_txdone_cnt;
	uint32_t xmit_txdone_cnt;

	/* Lock vmac_intr_lock is used to guarantee software variable pcie_int_en is
	consistent with hardware status when interrupt service routine is executed */
	spinlock_t intr_lock;
	uint32_t pcie_int_en;
	uint32_t intr_cnt;	/* msi/legacy interrupt counter */
	/* EP Tx exception when RC can't replenish descriptor in time */
	uint32_t intr_rx_no_dsc;
	uint32_t intr_rx_no_buf;

	struct sock *nl_socket;
	uint32_t str_call_nl_pid;
	uint32_t lib_call_nl_pid;
	struct napi_struct napi;

	uint32_t dbg_flg;
	void *dbg_buf;

	struct net_device *ndev;
	struct pci_dev *pdev;

	int mac_id;
	uint32_t ep_pciecfg0_val;	/* used to deassert Legacy irq from RC */

	/* The following members aren't related to datapath */
	struct vmac_cfg *pcfg;

	void *addr_uncache;
	uint32_t uncache_len;

	uint8_t msi_enabled;	/* PCIe MSI: 1 - Enabled, 0 - Disabled */

	qdpc_bar_t sysctl_bar;
	qdpc_bar_t epmem_bar;
	qdpc_bar_t dmareg_bar;

	uint32_t dma_imwr;

	/* io memory pointers */
	__iomem qdpc_pcie_bda_t *bda;

#ifdef QTN_SKB_RECYCLE_SUPPORT
	struct sk_buff_head rx_skb_freelist;
	spinlock_t rx_skb_freelist_lock;
	uint32_t rx_skb_freelist_fill_level;
	uint32_t skb_recycle_cnt;
	uint32_t skb_recycle_failures;
#endif

	uint32_t ep_pmstate;
	uint8_t *nl_buf;
	size_t nl_len;
	struct work_struct low_pwr_work;
	bool ep_under_low_pwr; /* Indicate whether EP is under low power mode */
};

#define QTN_DISABLE_SOFTIRQ		(0xABCD)

static inline void qtn_spin_lock_bh_save(spinlock_t * lock, unsigned long *flag)
{
	if (likely(irqs_disabled() || in_softirq())) {
		spin_lock(lock);
		*flag = 0;
	} else {
		spin_lock_bh(lock);
		*flag = QTN_DISABLE_SOFTIRQ;
	}
}

static inline void qtn_spin_unlock_bh_restore(spinlock_t * lock,
					      unsigned long *flag)
{
	if (unlikely(*flag == QTN_DISABLE_SOFTIRQ)) {
		*flag = 0;
		spin_unlock_bh(lock);
	} else {
		spin_unlock(lock);
	}
}

#ifndef QTN_RC_ENABLE_HDP
/* Alignment helper functions */
__always_inline static unsigned long align_up_off(unsigned long val,
						  unsigned long step)
{
	return (((val + (step - 1)) & (~(step - 1))) - val);
}

__always_inline static unsigned long align_down_off(unsigned long val,
						    unsigned long step)
{
	return ((val) & ((step) - 1));
}

__always_inline static unsigned long align_val_up(unsigned long val,
						  unsigned long step)
{
	return ((val + step - 1) & (~(step - 1)));
}

__always_inline static unsigned long align_val_down(unsigned long val,
						    unsigned long step)
{
	return (val & (~(step - 1)));
}

__always_inline static void *align_buf_dma(void *addr)
{
	return (void *)align_val_up((unsigned long)addr,
				    dma_get_cache_alignment());
}

__always_inline static unsigned long align_buf_dma_offset(void *addr)
{
	return (align_buf_dma(addr) - addr);
}

__always_inline static void *align_buf_cache(void *addr)
{
	return (void *)align_val_down((unsigned long)addr,
				      dma_get_cache_alignment());
}

__always_inline static unsigned long align_buf_cache_offset(void *addr)
{
	return (addr - align_buf_cache(addr));
}

__always_inline static unsigned long align_buf_cache_size(void *addr,
							  unsigned long size)
{
	return align_val_up(size + align_buf_cache_offset(addr),
			    dma_get_cache_alignment());
}
#endif

static inline bool vmac_fast_hdp(struct vmac_priv *vmp)
{
	return !!(vmp->dev_info & QTN_DEV_FAST_DATA_PATH);
}

extern struct net_device *vmac_alloc_ndev(void);
extern int vmac_net_init(struct pci_dev *pdev);
extern void vmac_clean(struct net_device *ndev);
extern int vmac_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev);
extern void vmac_set_dma_threshold(struct vmac_priv *vmp, uint32_t dma_threshold);

#define PCIE_REG_CFG_BASE		0x0
#define PCIE_LOGIC_PORT_CFG_BASE	(PCIE_REG_CFG_BASE + 0x700)
#define PCIE_DMA_WR_INTR_MASK		0x2c4

void vmac_hdp_enable(struct vmac_priv *vmp, bool en);
void vmac_pcie_edma_enable(struct vmac_priv *priv);
void qdpc_deassert_intx(struct vmac_priv *priv);
void qdpc_pcie_edma_enable(struct vmac_priv *priv);
int qdpc_pcie_suspend(struct pci_dev *pdev, pm_message_t state);
int qdpc_pcie_resume(struct pci_dev *pdev);
int qdpc_bringup_fw(struct vmac_priv *priv);
void qdpc_pcie_enter_low_power(struct vmac_priv *priv);
void qdpc_pcie_exit_low_power(struct vmac_priv *priv);

#endif /*  __DRIVERS_NET_PEARL_VNET_H */
