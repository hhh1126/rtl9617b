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

#ifndef __QDPC_INIT_H_
#define __QDPC_INIT_H_

#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include "pearl_vnet.h"

#define QDPC_MODULE_NAME                 "qdpc_ruby"
#define QDPC_DEV_NAME                    "qdpc_ruby"
#define QDPC_MODULE_VERSION              "1.0"

/* PCIe device information declarations */
#define  QDPC_VENDOR_ID                   0x1bb5
#define  QDPC_DEVICE_ID                   0x0008
#define  QDPC_DEVICE_ID_PEARL             0x0009
#define  QDPC_DEVICE_ID_JADE              0x000a
#define  QDPC_PCIE_NUM_BARS               6

/* PCIe Configuration Space Defines */
#define	QDPC_LINK_UP	((QDPC_DEVICE_ID << 16) | QDPC_VENDOR_ID)	/* Used to indicate CS is valid and link is up */
#define	QDPC_LINK_DOWN	0xffffffff	/* Used to indicate link went down */
#define QDPC_BAR_ADDR_MASK	0xfffff000
#define	QDPC_BAR_NOT_INIT	0xffffffff
#define	QDPC_VENDOR_ID_OFFSET	0x00
#define	QDPC_INT_LINE_OFFSET	0x3C
#define	QDPC_ROW_INCR_OFFSET	0x04
#undef	QDPC_CS_DEBUG

extern unsigned int (*qdpc_pci_readl) (void *addr);
extern void (*qdpc_pci_writel) (unsigned int val, void *addr);
extern unsigned short (*qdpc_pci_readw) (void *addr);
extern void (*qdpc_pci_writew) (unsigned short val, void *addr);
extern unsigned char (*qdpc_pci_readb) (void *addr);
extern void (*qdpc_pci_writeb) (unsigned char val, void *addr);
/*
 * End-point(EP) is little-Endian.
 * These two macros are used for host side outbound window memory access.
 * Outbound here is host side view-point. So memory accessed by these two macros
 * should be on EP side.
 * NOTE: On some platforms, outbound hardware swap(byte order swap) should be
 * enabled for outbound memory access correctly. If enabled, Endian translation
 * will be done by hardware, and software Endian translation should be disabled.
 * */
#ifdef OUTBOUND_HW_SWAP
#define le32_readl(x)           readl(x)
#define le32_writel(x, addr)    writel(x, addr)
#else
#define le32_readl(x)           le32_to_cpu(readl((x)))
#define le32_writel(x, addr)    writel(cpu_to_le32((x)), addr)
#endif

static inline unsigned int qdpc_readl(void *addr)
{
	return readl(addr);
}

static inline void qdpc_writel(unsigned int val, void *addr)
{
	writel(val, addr);
}

static inline unsigned int qdpc_le32_readl(void *addr)
{
	return le32_to_cpu(readl((addr)));
}

static inline void qdpc_le32_writel(unsigned int val, void *addr)
{
	writel(cpu_to_le32((val)), addr);
}

static inline void qdpc_pcie_posted_write(uint32_t val, __iomem void *basereg)
{
	qdpc_pci_writel(val, basereg);
	/* flush posted write */
	qdpc_pci_readl(basereg);
}

static inline unsigned short qdpc_readw(void *addr)
{
	return readw(addr);
}

static inline void qdpc_writew(unsigned short val, void *addr)
{
	writew(val, addr);
}

static inline void qdpc_pcie_posted_writew(uint16_t val, __iomem void *basereg)
{
	qdpc_pci_writew(cpu_to_le16(val), basereg);
	qdpc_pci_readw(basereg);
}

static inline unsigned char qdpc_readb(void *addr)
{
	return readb(addr);
}

static inline void qdpc_writeb(unsigned char val, void *addr)
{
	writeb(val,addr);
}

static inline void qdpc_pcie_posted_writeb(uint8_t val, __iomem void *basereg)
{
	qdpc_pci_writeb(val, basereg);
	qdpc_pci_readb(basereg);
}

/* Function prototypes */
int qdpc_pcie_init_intr_and_mem(struct vmac_priv *priv);
void qdpc_interrupt_target(struct vmac_priv *priv, uint32_t intr);
void qdpc_disable_irqs(struct vmac_priv *priv);
void qdpc_enable_irqs(struct vmac_priv *priv);
void qdpc_free_interrupt(struct pci_dev *pdev);
void qdpc_pcie_free_mem(struct pci_dev *pdev);
void qdpc_init_target_buffers(void *data);
int qdpc_send_packet(struct sk_buff *skb, struct net_device *ndev);
void *qdpc_map_pciemem(unsigned long busaddr, size_t len);
void qdpc_unmap_pciemem(unsigned long busaddr, void *vaddr, size_t len);
int qdpc_unmap_iomem(struct vmac_priv *priv);
int32_t qdpc_set_dma_mask(struct vmac_priv *priv);

#endif
