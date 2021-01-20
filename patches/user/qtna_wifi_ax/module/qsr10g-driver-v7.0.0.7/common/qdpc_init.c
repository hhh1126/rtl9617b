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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>

#include "qdpc_config.h"
#include "qdpc_debug.h"
#include "qdpc_init.h"
#include "qdpc_regs.h"
#include "qdpc_platform.h"
#include "pearl_vnet.h"
#include "qdpc_version.h"
#include "qtn_crc.h"

#define QDPC_TOPAZ_IMG		"topaz-linux.lzma.img"
#define QDPC_PEARL_IMG		"pearl-linux.lzma.img"
#define QDPC_JADE_IMG		"jade-linux.lzma.img"
#define QDPC_TOPAZ_UBOOT	"u-boot.bin"
#define MAX_IMG_NUM		2

#define EP_BOOT_FROM_FLASH 1
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"


#ifndef MEMORY_START_ADDRESS
#define MEMORY_START_ADDRESS virt_to_bus((void *)PAGE_OFFSET)
#endif

#define QDPC_EP_CFG_VERSION	1
#define QDPC_EP_CFG_NAME_LEN	64
static const int QDPC_EP_CFG_CHECK_PERIOD_MS = 500;
static unsigned int tlp_mps = 256;
module_param(tlp_mps, uint, 0644);
MODULE_PARM_DESC(tlp_mps, "Default PCIe Max_Payload_Size");

/* Quantenna PCIE vendor and device identifiers  */
static struct pci_device_id qdpc_pcie_ids[] = {
	{PCI_DEVICE(QDPC_VENDOR_ID, QDPC_DEVICE_ID),},
	{PCI_DEVICE(QDPC_VENDOR_ID, QDPC_DEVICE_ID_PEARL),},
	{PCI_DEVICE(QDPC_VENDOR_ID, QDPC_DEVICE_ID_JADE),},
	{0,}
};

MODULE_DEVICE_TABLE(pci, qdpc_pcie_ids);

static int qdpc_pcie_probe(struct pci_dev *pdev,
			   const struct pci_device_id *id);
static void qdpc_pcie_remove(struct pci_dev *pdev);
static void qdpc_nl_recv_msg(struct sk_buff *skb);
int qdpc_init_netdev(struct net_device **net_dev, struct pci_dev *pdev);

static bool is_ep_reset = false;
#ifndef PCIE_HOTPLUG_SUPPORTED
static int link_monitor(void *data);
static struct task_struct *link_monitor_thread = NULL;
#endif

#define QDPC_EP_CFG_DOWNLOAD_TIMEOUT         (60 * HZ)
static unsigned long qdpc_ep_cfg_push_jiffies;
static int qdpc_ep_cfg_push_monitor(void *data);
static struct task_struct *qdpc_push_cfg_thread;
static DECLARE_COMPLETION(qdpc_push_cfg_thread_exit);

char qdpc_pcie_driver_name[] = "qdpc_host";

static struct pci_driver qdpc_pcie_driver = {
	.name = qdpc_pcie_driver_name,
	.id_table = qdpc_pcie_ids,
	.probe = qdpc_pcie_probe,
	.remove = qdpc_pcie_remove,
#ifdef CONFIG_QTN_PM
	.suspend = qdpc_pcie_suspend,
	.resume = qdpc_pcie_resume,
#endif
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
struct netlink_kernel_cfg qdpc_netlink_cfg = {
	.groups = 0,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	.flags = 0,
#endif
	.input = qdpc_nl_recv_msg,
	.cb_mutex = NULL,
	.bind = NULL,
};
#endif

struct sock *qdpc_nl_sk = NULL;
int qdpc_clntPid = 0;

unsigned int (*qdpc_pci_readl) (void *addr) = qdpc_readl;
void (*qdpc_pci_writel) (unsigned int val, void *addr) = qdpc_writel;
unsigned short (*qdpc_pci_readw) (void *addr) = qdpc_readw;
void (*qdpc_pci_writew) (unsigned short val, void *addr) = qdpc_writew;
unsigned char (*qdpc_pci_readb) (void *addr) = qdpc_readb;
void (*qdpc_pci_writeb) (unsigned char val, void *addr) = qdpc_writeb;

static inline int qdpc_isbootstate(uint32_t * reg, uint32_t state)
{
	uint32_t s = qdpc_pci_readl(reg);
	return (s & state);
}

static inline void qdpc_setbootstate(uint32_t * reg, uint32_t state)
{
	uint32_t s = qdpc_pci_readl(reg);
	qdpc_pcie_posted_write(state | s, reg);
}

static inline void qdpc_clearbootstate(uint32_t * reg, uint32_t state)
{
	uint32_t s = qdpc_pci_readl(reg);
	qdpc_pcie_posted_write((s & ~state), reg);
}

static inline void qdpc_fillin_rc_version(uint8_t *reg, uint8_t *version){
	int i,n;

	n = strlen(version);
	if ((n == 0) || (n >=  RC_VERSION_LEN))
		return;
	for (i = 0; i <= n; i++)
		qdpc_pcie_posted_writeb(version[i], reg++);
}

static int qdpc_bootpoll(uint32_t * reg, uint32_t state)
{
	uint32_t timeout = 0;
	while ((qdpc_isbootstate(reg, state) == 0)) {
		udelay(1000);
		if (++timeout > QDPC_FW_DLTIMEOUT)
			return -1;
	}
	return 0;
}

static int qtn_ep_fw_send(struct pci_dev *pdev, uint32_t size, uint32_t max_pkt_len,
			  int cnt, const uint8_t * pcur, const uint8_t * fw)
{
	struct sk_buff *skb = NULL;
	struct net_device *ndev = (struct net_device *)pci_get_drvdata(pdev);
	struct pearl_pcie_fwhd *pfwhd;
	uint8_t *pdata;
	int hds = sizeof(struct pearl_pcie_fwhd);
	int dlen = 0;

#ifndef QTN_RC_ENABLE_HDP
	/* Need allocate extra MPS bytes for alignment later due to hardware limitation */
	skb = VMAC_DEV_ALLOC_SKB(max_pkt_len + tlp_mps);
#else
	skb = VMAC_DEV_ALLOC_SKB(max_pkt_len);
#endif
	if (!skb)
		return -1;
#ifndef QTN_RC_ENABLE_HDP
	/*align data to MPS boudary for hardware */
	skb_reserve(skb, align_up_off((unsigned long)skb->data, tlp_mps));
#endif
	skb->len = max_pkt_len;
	skb->dev = ndev;

	pfwhd = (struct pearl_pcie_fwhd *)skb->data;
	memcpy(pfwhd->boardflg, PEARL_PCIE_BOARDFLG,
	       strlen(PEARL_PCIE_BOARDFLG));
	pfwhd->fwsize = size;
	pfwhd->seqnum = cnt;
	if (cnt)
		pfwhd->type = PEARL_FW_DSUB;
	else
		pfwhd->type = PEARL_FW_DBEGIN;
	pdata = skb->data + hds;

	dlen = max_pkt_len - hds;
	if (pcur >= (fw + size - dlen)) {
		dlen = fw + size - pcur;
		pfwhd->type = PEARL_FW_DEND;
	}
	pfwhd->pktlen = dlen;
	memcpy(pdata, pcur, dlen);
	pfwhd->crc = qtn_crc_32(~0, pdata, dlen);
	vmac_hard_start_xmit(skb, ndev);
	return dlen;
}

static int qtn_ep_fw_load(struct pci_dev *pdev, const uint32_t max_pkt_len,
		const uint8_t *fw, uint32_t size, struct vmac_priv *priv, uint8_t is_cfg)
{
	__iomem qdpc_pcie_bda_t *bda = priv->bda;
	int dlen, cnt = 0;
	const uint8_t *pcur;
	int threshold = 0;
	int defdlen = max_pkt_len - sizeof(struct pearl_pcie_fwhd);
	int group = size / defdlen + !!(size % defdlen);
	struct net_device *ndev = (struct net_device *)pci_get_drvdata(pdev);
	struct vmac_priv *vmp;

	if (!fw)
		return -1;

	if (ndev == NULL)
		return -1;

	vmp = netdev_priv(ndev);
	vmac_set_dma_threshold(vmp, max_pkt_len);
	vmac_hdp_enable(vmp, true);

	pcur = fw;
	printk("FW: start tr fw address = 0x%p, size=%d\n", fw, size);

	while (cnt < group) {

		if (++threshold > 10000) {
			printk
			    ("FW: Retry too many times, failed to download EP FW\n");
			return -1;
		}

		/*
		 * The download retry algorithm is error-prone and inefficient.
		 * This delay eliminates retries during download, which is safer than
		 * reworking the retry algorithm.
		 */
		if (is_cfg)
			udelay(100);

		dlen = qtn_ep_fw_send(pdev, size, max_pkt_len, cnt, pcur, fw);
		if (dlen <= 0)
			continue;

		if (!((cnt + 1) & QDPC_PCIE_FW_DLMASK) || (cnt == (group - 1))) {
			qdpc_setbootstate(&bda->bda_rc_state, QDPC_RC_FW_SYNC);
			if (qdpc_bootpoll(&bda->bda_ep_state, QDPC_EP_FW_SYNC)) {
				printk
				    ("Failed to download firmware, EP SYNC timeout....\n");
				return -1;
			}
			qdpc_clearbootstate(&bda->bda_ep_state,
					    QDPC_EP_FW_SYNC);
			if (qdpc_isbootstate
			    (&bda->bda_ep_state, QDPC_EP_FW_RETRY)) {
				if (cnt == (group - 1)) {
					int last_round =
					    group & QDPC_PCIE_FW_DLMASK;
					cnt -= last_round;
					pcur -=
					    ((last_round - 1) * defdlen + dlen);
				} else {
					cnt -= QDPC_PCIE_FW_DLMASK;
					pcur -= QDPC_PCIE_FW_DLMASK * defdlen;
				}
				qdpc_clearbootstate(&bda->bda_ep_state,
						    QDPC_EP_FW_RETRY);
				continue;
			}
		}
		cnt++;
		pcur += dlen;
	};

	printk("FW: finished downloading, totally send %d pkts\n", cnt);
	return 0;
}

static int qdpc_firmware_load(struct pci_dev *pdev, struct vmac_priv *priv,
			      const char *name)
{
	int result = SUCCESS;
	const struct firmware *fw;
	__iomem qdpc_pcie_bda_t *bda = priv->bda;

	/* Request compressed firmware from user space */
	result = request_firmware(&fw, name, &pdev->dev);
	if (result != SUCCESS) {
		/*
		 * No firmware found in the firmware directory, skip firmware downloading process
		 * boot from flash directly on target
		 */
		printk("no firmware found skip fw downloading\n");
		return FAILURE;
	}

	result = qtn_ep_fw_load(pdev, PEARL_PCIE_DWBUFSZ,
			fw->data, fw->size, priv, 0);
	if (result != SUCCESS) {
		PRINT_ERROR("Failed to load firmware:%d\n", result);
	} else if (qdpc_bootpoll(&bda->bda_ep_state, QDPC_EP_FW_DONE)) {
		printk("FW downloading timeout\n");
		result = -1;
	} else {
		PRINT_INFO("Image downloaded....!\n");
	}
	release_firmware(fw);

	return result;
}

static void qdpc_pcie_dev_init(struct vmac_priv *priv, struct pci_dev *pdev,
			       struct net_device *ndev)
{
	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv->pdev = pdev;
	priv->ndev = ndev;
	pci_set_drvdata(pdev, ndev);
}

static void qdpc_tune_pcie_mps(struct pci_dev *pdev, int pos)
{
	struct pci_dev *parent = NULL;
	int ppos = 0;
	uint32_t dev_cap;
	uint16_t dev_ctl, pctl;
	unsigned int mps = tlp_mps;
#define BIT_TO_MPS(m) (1 << ((m) + 7))

	if (!pdev->bus)
		return;

	parent = pdev->bus->self;
	if (likely(parent)) {
		ppos = pci_find_capability(parent, PCI_CAP_ID_EXP);
		if (ppos) {
			pci_read_config_word(parent,
					     ppos + PCI_EXP_DEVCTL,
					     &pctl);
			pci_read_config_dword(pdev,
					      pos + PCI_EXP_DEVCAP,
					      &dev_cap);
			printk(KERN_INFO
			       "parent ctrl:%u, dev cap:%u, parent pos %d, dev pos %d\n",
			       BIT_TO_MPS((pctl &
					   PCI_EXP_DEVCTL_PAYLOAD) >>
					  5),
			       BIT_TO_MPS(dev_cap &
					  PCI_EXP_DEVCAP_PAYLOAD), ppos,
			       pos);
			mps =
			    min(BIT_TO_MPS
				((pctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5),
				BIT_TO_MPS(dev_cap &
					   PCI_EXP_DEVCAP_PAYLOAD));
		}
	}
	tlp_mps = mps;
	printk(KERN_INFO "Setting MPS to %u\n", mps);

	/*
	 * Set Max_Payload_Size
	 * Max_Payload_Size_in_effect = 1 << ( ( (dev_ctl >> 5) & 0x07) + 7);
	 */
	mps = ((ffs(mps) - 8) << 5);
	pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &dev_ctl);
	dev_ctl = ((dev_ctl & ~PCI_EXP_DEVCTL_PAYLOAD) | mps);
	pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, dev_ctl);
}

static struct net_device *g_ndev = NULL;
static int qdpc_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct vmac_priv *priv = NULL;
	struct net_device *ndev = NULL;
	int result = SUCCESS;
	int pos;

	/* Allocate device structure */
	if (!(ndev = vmac_alloc_ndev()))
		return -ENOMEM;

	g_ndev = ndev;
	priv = netdev_priv(ndev);
	qdpc_pcie_dev_init(priv, pdev, ndev);

	/* allocate netlink data buffer */
	priv->nl_buf = kmalloc(VMAC_NL_BUF_SIZE, GFP_KERNEL);
	if (!priv->nl_buf) {
		result = -ENOMEM;
		goto out;
	}

	/* Check if the device has PCI express capability */
	pos = pci_find_capability(pdev, PCI_CAP_ID_EXP);
	if (!pos) {
		PRINT_ERROR(KERN_ERR
			    "The device %x does not have PCI Express capability\n",
			    pdev->device);
		result = -ENOSYS;
		goto out;
	} else {
		PRINT_DBG(KERN_INFO
			  "The device %x has PCI Express capability\n",
			  pdev->device);
	}

	qdpc_tune_pcie_mps(pdev, pos);

	/*  Wake up the device if it is in suspended state and allocate IO,
	 *  memory regions and IRQ if not
	 */
	if (pci_enable_device(pdev)) {
		PRINT_ERROR(KERN_ERR
			    "Failed to initialize PCI device with device ID %x\n",
			    pdev->device);

		result = -EIO;
		goto out;
	} else {
		PRINT_DBG(KERN_INFO
			  "Initialized PCI device with device ID %x\n",
			  pdev->device);
	}

	qdpc_set_dma_mask(priv);
	/*
	 * Check if the PCI device can support DMA addressing properly.
	 * The mask gives the bits that the device can address
	 */
	pci_set_master(pdev);

	/* Initialize PCIE layer  */
	if ((result = qdpc_pcie_init_intr_and_mem(priv)) < 0) {
		PRINT_DBG("Interrupt & Memory Initialization failed \n");
		goto release_memory;
	}

	pci_save_state(pdev);

	/*Update bda version and add rc version to bda*/
	qdpc_pcie_posted_writew(QDPC_PCIE_BDA_VERSION, &priv->bda->bda_version);
	qdpc_fillin_rc_version((uint8_t *)&priv->bda->bda_rc_version, DRV_VERSION);

	if (! !(result = vmac_net_init(pdev))) {
		PRINT_DBG("Vmac netdev init fail\n");
		goto free_mem_interrupt;
	}
/* Create and start the thread to initiate the INIT Handshake*/
	qdpc_bringup_fw(priv);
	ndev->netdev_ops->ndo_change_mtu(ndev, ndev->mtu);
#ifdef QTN_RC_ENABLE_HDP
	qdpc_wps_button_init(ndev);
#endif

	/* Create netlink & register with kernel */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	priv->nl_socket = netlink_kernel_create(&init_net,
						QDPC_NETLINK_RPC_PCI_CLNT,
						&qdpc_netlink_cfg);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
	priv->nl_socket = netlink_kernel_create(&init_net,
						QDPC_NETLINK_RPC_PCI_CLNT,
						THIS_MODULE, &qdpc_netlink_cfg);
#else
	priv->nl_socket = netlink_kernel_create(&init_net,
						QDPC_NETLINK_RPC_PCI_CLNT, 0,
						qdpc_nl_recv_msg, NULL,
						THIS_MODULE);
#endif
	if (priv->nl_socket) {
		return SUCCESS;
	}

	PRINT_ERROR(KERN_ALERT "Error creating netlink socket.\n");
	result = FAILURE;

 free_mem_interrupt:
	qdpc_unmap_iomem(priv);
	qdpc_pcie_free_mem(pdev);
	qdpc_free_interrupt(pdev);

 release_memory:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
	/* Releasing the memory region if any error occured */
	pci_clear_master(pdev);
#endif

	pci_disable_device(pdev);

 out:
	kfree(priv->nl_buf);
	free_netdev(ndev);
	g_ndev = NULL;
	/* Any failure in probe, so it can directly return in remove */
	pci_set_drvdata(pdev, NULL);

	return result;
}

static void qdpc_pcie_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct vmac_priv *vmp;

	if (ndev == NULL)
		return;

	vmp = netdev_priv(ndev);

	if (vmp->nl_socket)
		netlink_kernel_release(vmp->nl_socket);

	if (vmp->dbg_buf)
		vfree(vmp->dbg_buf);

	kfree(vmp->nl_buf);

	vmac_clean(ndev);

	qdpc_free_interrupt(pdev);
	qdpc_pcie_free_mem(pdev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
	pci_clear_master(pdev);
#endif
	pci_disable_device(pdev);

	writel(TOPAZ_SET_INT(1 << PEARL_LH_IPC_EP_REBOOT),
		IO_ADDRESS(vmp->ep_ipc_reg));

	msleep(1000);
	pci_restore_state(pdev);

	qdpc_unmap_iomem(vmp);

	free_netdev(ndev);
	g_ndev = NULL;

	return;
}

static inline int qdpc_pcie_set_power_state(struct pci_dev *pdev,
					    pci_power_t state)
{
	uint16_t pmcsr;

	pci_read_config_word(pdev, TOPAZ_PCI_PM_CTRL_OFFSET, &pmcsr);

	switch (state) {
	case PCI_D0:
#ifdef QTN_RC_ENABLE_HDP
		if ((readl(IO_ADDRESS(RUBY_SYS_CTL_CSR)) & 0xff) == TOPAZ_BOARD_REVA2)
			pci_write_config_word(pdev, TOPAZ_PCI_PM_CTRL_OFFSET,
					      (pmcsr & ~PCI_PM_CTRL_STATE_MASK)
					      | (PCI_D0 |
						 PCI_PM_CTRL_PME_ENABLE));
		else
#endif
			pci_write_config_word(pdev, TOPAZ_PCI_PM_CTRL_OFFSET,
					      (pmcsr & ~PCI_PM_CTRL_STATE_MASK)
					      | PCI_D0);
		break;

	case PCI_D3hot:
		pci_write_config_word(pdev, TOPAZ_PCI_PM_CTRL_OFFSET,
				      (pmcsr & ~PCI_PM_CTRL_STATE_MASK) |
				      (PCI_D3hot | PCI_PM_CTRL_PME_ENABLE));
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

void qdpc_pcie_ep_tx_en(struct vmac_priv *priv, bool enable)
{
	uint32_t value;

	value = le32_to_cpu(readl(IO_ADDRESS(PCIE_HDP_TX_DMA_CTRL(priv->pcie_reg_base))));

	if (enable == true)
		value &= ~PCIE_TX_DMA_DISABLE;
	else
		value |= PCIE_TX_DMA_DISABLE;

	writel(cpu_to_le32(value),
		IO_ADDRESS(PCIE_HDP_TX_DMA_CTRL(priv->pcie_reg_base)));
}

void qdpc_pcie_enter_low_power(struct vmac_priv *priv)
{
	if (priv->ep_pmstate == PCI_D3hot)
		return;

	printk("%s PCIe enter power save (D3hot)\n", qdpc_pcie_driver_name);

	qdpc_pcie_ep_tx_en(priv, false);
	qdpc_pcie_set_power_state(priv->pdev, PCI_D3hot);
	priv->ep_pmstate = PCI_D3hot;
	priv->ep_under_low_pwr = true;
}

void qdpc_pcie_exit_low_power(struct vmac_priv *priv)
{
	if (priv->ep_pmstate == PCI_D0)
		return;

	printk("%s PCIe exit power save mode\n", qdpc_pcie_driver_name);

	priv->ep_pmstate = PCI_D0;
	qdpc_pcie_set_power_state(priv->pdev, PCI_D0);
	qdpc_pcie_ep_tx_en(priv, true);
	priv->ep_under_low_pwr = false;
}

int qdpc_pcie_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct vmac_priv *priv;

	if (ndev == NULL)
		return -EINVAL;

	priv = netdev_priv(ndev);
	if (priv->ep_pmstate == PCI_D3hot) {
		return 0;
	}

	printk("%s start power management suspend\n", qdpc_pcie_driver_name);

	/* Set ep not ready to drop packets in low power mode */
	netif_stop_queue(ndev);
	qdpc_pcie_ep_tx_en(priv, false);
	ndev->flags &= ~IFF_RUNNING;
	priv->ep_pmstate = PCI_D3hot;
	barrier();
	writel(TOPAZ_SET_INT(IPC_EP_PM_CTRL), IO_ADDRESS(priv->ep_ipc_reg));

	msleep(100);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	qdpc_pcie_set_power_state(pdev, PCI_D3hot);

	return 0;
}

int qdpc_pcie_resume(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct vmac_priv *priv;
	int ret;

	if (ndev == NULL)
		return -EINVAL;

	priv = netdev_priv(ndev);
	if (priv->ep_pmstate == PCI_D0) {
		return 0;
	}

	printk("%s start power management resume\n", qdpc_pcie_driver_name);

	ret = pci_enable_device(pdev);
	if (ret) {
		PRINT_ERROR("%s: pci_enable_device failed on resume\n",
			    __func__);
		return ret;
	}

	pci_restore_state(pdev);
	qdpc_pcie_set_power_state(pdev, PCI_D0);

#ifdef QTN_RC_ENABLE_HDP
	/* Version C board ignore the interrupt here. Because current PCIE
	 * link is in L1 in which EP's memory is not accessible.
	 * Instead C board's EP adds a timer to check the PCIe power state
	 * periodically, if power state change to D3hot, EP will do
	 * IPC_EP_PM_CTRL handler's job
	 */
	if ((readl(IO_ADDRESS(RUBY_SYS_CTL_CSR)) & 0xff) == TOPAZ_BOARD_REVA2) {
		msleep(5000);

		priv->ep_pmstate = PCI_D0;
		barrier();
	} else
#endif
	{
		priv->ep_pmstate = PCI_D0;
		barrier();
		writel(TOPAZ_SET_INT(IPC_EP_PM_CTRL),
			IO_ADDRESS(priv->ep_ipc_reg));

		msleep(5000);
	}

	/* Set ep_ready to resume tx traffic */
	ndev->flags |= IFF_RUNNING;
	qdpc_pcie_ep_tx_en(priv, true);
	netif_wake_queue(ndev);

	return 0;
}

static int __init qdpc_init_module(void)
{
	int ret;

	PRINT_DBG(KERN_INFO "Quantenna pcie driver initialization\n");

	if (qdpc_platform_init()) {
		PRINT_ERROR("Platform initilization failed \n");
		ret = FAILURE;
		return ret;
	}

	/*  Register the pci driver with device */
	if ((ret = pci_register_driver(&qdpc_pcie_driver)) < 0) {
		PRINT_ERROR("Could not register the driver to pci : %d\n", ret);
		ret = -ENODEV;
		return ret;
	}

	if (!g_ndev) {
		pci_unregister_driver(&qdpc_pcie_driver);
		qdpc_platform_exit();
		return -EPERM;
	}

#ifndef PCIE_HOTPLUG_SUPPORTED
	link_monitor_thread = kthread_run(link_monitor, NULL, "link_monitor");
#endif
	qdpc_push_cfg_thread = kthread_run(qdpc_ep_cfg_push_monitor,
			NULL, "qdpc_ep_cfg_monitor");
	if (IS_ERR(qdpc_push_cfg_thread)) {
		complete(&qdpc_push_cfg_thread_exit);
		qdpc_push_cfg_thread = NULL;
	}
	qdpc_ep_cfg_push_jiffies = jiffies;

	return ret;
}

static void __exit qdpc_exit_module(void)
{
	/* Release netlink */
	qdpc_platform_exit();

#ifndef PCIE_HOTPLUG_SUPPORTED
	kthread_stop(link_monitor_thread);
	link_monitor_thread = NULL;
#endif
	if (qdpc_push_cfg_thread) {
		kthread_stop(qdpc_push_cfg_thread);
		wait_for_completion(&qdpc_push_cfg_thread_exit);
	}

	/* Unregister the pci driver with the device */
	pci_unregister_driver(&qdpc_pcie_driver);

#ifdef QTN_RC_ENABLE_HDP
	qdpc_wps_button_exit();
#endif

	return;
}

static inline bool is_pcie_linkup(struct pci_dev *pdev)
{
	uint32_t cs = 0;

	pci_read_config_dword(pdev, QDPC_VENDOR_ID_OFFSET, &cs);
	if (cs == QDPC_LINK_UP) {
		msleep(10000);
		printk("%s: PCIe link up!\n", __func__);
		return true;
	}

	return false;
}

static inline void qdpc_pcie_print_config_space(struct pci_dev *pdev)
{
	int i = 0;
	uint32_t cs = 0;

	/* Read PCIe configuration space header */
	for (i = QDPC_VENDOR_ID_OFFSET; i <= QDPC_INT_LINE_OFFSET;
	     i += QDPC_ROW_INCR_OFFSET) {
		pci_read_config_dword(pdev, i, &cs);
		printk
		    ("%s: pdev:0x%p config_space offset:0x%02x value:0x%08x\n",
		     __func__, pdev, i, cs);
	}
	printk("\n");
}

static inline void qdpc_pcie_check_link(struct pci_dev *pdev,
					struct vmac_priv *priv)
{
	__iomem qdpc_pcie_bda_t *bda = priv->bda;
	uint32_t cs = 0;

	pci_read_config_dword(pdev, QDPC_VENDOR_ID_OFFSET, &cs);
	/* Endian value will be all 1s if link went down */
	if (readl(IO_ADDRESS(&bda->bda_pci_endian)) == QDPC_LINK_DOWN) {
		is_ep_reset = true;
		printk("Reset detected\n");
	}
}

static int qdpc_request_cfg_firmware(const struct firmware **pqtn_cfg,
		uint8_t *pcfgname, struct pci_dev *pdev)
{
	int result;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	result = request_firmware_direct(pqtn_cfg, pcfgname, &pdev->dev);
#else
	result = request_firmware(pqtn_cfg, pcfgname, &pdev->dev);
#endif
	return result;
}

static int qdpc_config_load(struct pci_dev *pdev, struct net_device *ndev)
{
	struct vmac_priv *priv = netdev_priv(ndev);
	uint8_t cfgname[QDPC_EP_CFG_NAME_LEN] = {0};
	__iomem qdpc_pcie_bda_t *bda = priv->bda;
	const struct firmware *qtn_cfg;
	int result;

	snprintf(cfgname, sizeof(cfgname) - 1, "qtn/qtn-%02X%02X%02X%02X%02X%02X-v%u.cfg",
			MAC2STR(ndev->dev_addr), QDPC_EP_CFG_VERSION);
	result = qdpc_request_cfg_firmware(&qtn_cfg, cfgname, pdev);
	if (result != SUCCESS) {
		PRINT_INFO("%s: EP config %s not found\n",
			__func__, cfgname);
		qdpc_setbootstate(&bda->bda_rc_state, QDPC_RC_EP_CFG_NONE);
		return 0;
	}
	PRINT_INFO("%s: Downloading EP config %s\n",
		__func__, cfgname);
	qdpc_clearbootstate(&bda->bda_rc_state, QDPC_RC_EP_CFG_NONE);

	result = qtn_ep_fw_load(pdev, PEARL_PCIE_CFG_DWBUFSZ,
			qtn_cfg->data, qtn_cfg->size, priv, 1);
	if (result != SUCCESS) {
		PRINT_ERROR("%s: Failed to download EP config %s - %d\n",
			__func__, cfgname, result);
	} else if (qdpc_isbootstate(&bda->bda_ep_state, QDPC_EP_CFG_FAILED)) {
		PRINT_ERROR("%s: Failed to download EP config %s\n",
			__func__, cfgname);
		result = -1;
	} else {
		PRINT_ERROR("%s: EP config %s downloaded\n",
			__func__, cfgname);
	}
	release_firmware(qtn_cfg);

	return result;
}

static int qdpc_ep_cfg_push_monitor(void *data)
{
	struct net_device *ndev = NULL;
	struct vmac_priv *priv = NULL;
	__iomem qdpc_pcie_bda_t *bda = NULL;
	struct pci_dev *pdev = NULL;

	set_current_state(TASK_RUNNING);

	while (!kthread_should_stop()) {
		__set_current_state(TASK_INTERRUPTIBLE);

		ndev = g_ndev;
		priv = netdev_priv(ndev);
		bda = priv->bda;
		pdev = priv->pdev;

		if (qdpc_isbootstate(&bda->bda_ep_state, QDPC_EP_CFG_WAIT)) {
			PRINT_INFO("%s: [%pM] Setting EP config ready...\n",
				__func__, ndev->dev_addr);
			if (qdpc_config_load(priv->pdev, ndev))
				PRINT_INFO("%s: Failed to download EP config - trying again...\n",
					__func__);
		} else if (qdpc_isbootstate(&bda->bda_ep_state,
					QDPC_EP_CFG_DONE | QDPC_EP_CFG_FAILED)) {
			qdpc_clearbootstate(&bda->bda_ep_state,
					QDPC_EP_CFG_DONE | QDPC_EP_CFG_FAILED);
			break;
		}

		if (time_after(jiffies,
					qdpc_ep_cfg_push_jiffies + QDPC_EP_CFG_DOWNLOAD_TIMEOUT)) {
			qdpc_ep_cfg_push_jiffies = jiffies;
			break;
		}

		if (!kthread_should_stop())
			schedule_timeout_interruptible(msecs_to_jiffies(QDPC_EP_CFG_CHECK_PERIOD_MS));
	}

	printk("%s: EP Download config timeout - quit\n", __func__);
	qdpc_push_cfg_thread = NULL;
	complete_and_exit(&qdpc_push_cfg_thread_exit, 0);
}

#ifndef PCIE_HOTPLUG_SUPPORTED
static int link_monitor(void *data)
{
	struct net_device *ndev = NULL;
	struct vmac_priv *priv = NULL;
	__iomem qdpc_pcie_bda_t *bda = NULL;
	struct pci_dev *pdev = NULL;
	uint32_t cs = 0;

	set_current_state(TASK_RUNNING);
	while (!kthread_should_stop()) {
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		set_current_state(TASK_RUNNING);

		ndev = g_ndev;
		priv = netdev_priv(ndev);
		bda = priv->bda;
		pdev = priv->pdev;

#ifdef QDPC_CS_DEBUG
		qdpc_pcie_print_config_space(pdev);
		msleep(5000);
#endif

		/* Check if reset to EP occurred */
		while (!pci_read_config_dword(pdev, QDPC_VENDOR_ID_OFFSET, &cs)) {

			if (kthread_should_stop())
				do_exit(0);

			qdpc_pcie_check_link(pdev, priv);
			if (is_ep_reset) {
				is_ep_reset = false;
				qdpc_pcie_remove(pdev);
				printk
				    ("%s: Attempting to recover from EP reset\n",
				     __func__);
				break;
			}
			msleep(500);
		}

		while (!is_pcie_linkup(pdev)) {
		}

#ifdef QDPC_CS_DEBUG
		qdpc_pcie_print_config_space(pdev);
#endif

		qdpc_pcie_probe(pdev, NULL);
	}
	do_exit(0);
}
#endif

int qdpc_bringup_fw(struct vmac_priv *priv)
{
	__iomem qdpc_pcie_bda_t *bda = priv->bda;
	uint16_t dev_id;
	char *fwname;

	if (qdpc_pci_readl(IO_ADDRESS(&bda->bda_ep_state)) &
					QDPC_EP_HAS_FRIMWARE) {
		printk("EP has image, stop loading firmware\n");
		return 0;
	}

	printk("Setting HOST ready...\n");
	qdpc_setbootstate(&bda->bda_rc_state, QDPC_RC_FW_LOADRDY);
	if (qdpc_bootpoll(&bda->bda_ep_state, QDPC_EP_FW_LOADRDY)) {
		printk("Failed to download firmware, EP RDY timeout....\n");
		return -ENXIO;
	}
	qdpc_clearbootstate(&bda->bda_ep_state, QDPC_EP_FW_LOADRDY);

	pci_read_config_word(priv->pdev, PCI_DEVICE_ID, &dev_id);
	if (dev_id == QDPC_DEVICE_ID_JADE)
		fwname = QDPC_JADE_IMG;
	else
		fwname = QDPC_PEARL_IMG;

	printk("Start download Firmware %s...\n", fwname);
	if (qdpc_firmware_load(priv->pdev, priv, fwname)) {
		printk("Failed to download firmware.\n");
		return -EIO;
	}

	return 0;
}

static void qdpc_nl_recv_msg(struct sk_buff *skb)
{
	struct vmac_priv *priv = netdev_priv(g_ndev);
	struct nlmsghdr *nlh = (struct nlmsghdr *)skb->data;
	struct sk_buff *skb2;
	unsigned int data_len;
	unsigned int offset;
	qdpc_cmd_hdr_t *cmd_hdr;
	uint16_t rpc_type;

	/* Parsing the netlink message */

	PRINT_DBG(KERN_INFO
		  "%s line %d Netlink received pid:%d, size:%d, type:%d\n",
		  __FUNCTION__, __LINE__, nlh->nlmsg_pid, nlh->nlmsg_len,
		  nlh->nlmsg_type);

	switch (nlh->nlmsg_type) {
	case QDPC_NL_TYPE_CLNT_STR_REG:
	case QDPC_NL_TYPE_CLNT_LIB_REG:
		if (nlh->nlmsg_type == QDPC_NL_TYPE_CLNT_STR_REG)
			priv->str_call_nl_pid = nlh->nlmsg_pid;
		else
			priv->lib_call_nl_pid = nlh->nlmsg_pid;
		return;
	case QDPC_NL_TYPE_CLNT_STR_REQ:
	case QDPC_NL_TYPE_CLNT_LIB_REQ:
		break;
	default:
		PRINT_DBG(KERN_INFO "%s line %d Netlink Invalid type %d\n",
			  __FUNCTION__, __LINE__, nlh->nlmsg_type);
		return;
	}

	/*
	 * make new skbs; Fragment if necessary.
	 * The original skb will be freed in netlink_unicast_kernel,
	 * we hold the new skbs until DMA transfer is done
	 */
	offset = sizeof(struct nlmsghdr);
	data_len = nlh->nlmsg_len;

	while (data_len > 0) {
		unsigned int len =
		    min_t(unsigned int, data_len, priv->ndev->mtu-sizeof(qdpc_cmd_hdr_t));
		unsigned int skb2_len = len + sizeof(qdpc_cmd_hdr_t);

		skb2 = VMAC_DEV_ALLOC_SKB(skb2_len);
		if (!skb2) {
			printk(KERN_INFO "%s: skb alloc failed\n", __func__);
			return;
		}

		data_len -= len;

		rpc_type = nlh->nlmsg_type & QDPC_RPC_TYPE_MASK;
		rpc_type |= (data_len > 0 ? QDPC_RPC_TYPE_FRAG : 0);

		cmd_hdr = (qdpc_cmd_hdr_t *) skb2->data;
		memcpy(cmd_hdr->dst_magic, QDPC_NETLINK_DST_MAGIC, ETH_ALEN);
		memcpy(cmd_hdr->src_magic, QDPC_NETLINK_SRC_MAGIC, ETH_ALEN);
		cmd_hdr->type = __constant_htons(QDPC_APP_NETLINK_TYPE);
		cmd_hdr->len = htons((uint16_t) len);
		cmd_hdr->rpc_type = htons(rpc_type);
		cmd_hdr->total_len = htons((uint16_t) (nlh->nlmsg_len));

		memcpy((uint8_t *) (cmd_hdr + 1), skb->data + offset, len);

		offset += len;

		skb_put(skb2, skb2_len);
		skb_reset_mac_header(skb2);
		skb_reset_network_header(skb2);
		skb2->protocol = __constant_htons(QDPC_APP_NETLINK_TYPE);
		skb2->dev = priv->ndev;

		dev_queue_xmit(skb2);
	}
}

module_init(qdpc_init_module);
module_exit(qdpc_exit_module);
