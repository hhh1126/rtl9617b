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

#ifndef __QDPC_PFDEP_H__
#define __QDPC_PFDEP_H__

#include <linux/version.h>

#include <pearl_vnet.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#define IOREMAP      ioremap_wc
#else				/* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) */
#define IOREMAP      ioremap
#endif

#ifndef IO_ADDRESS
#define IO_ADDRESS(x) ((void *)(x))
#endif

/* IO functions */
#ifndef readb
#define readb(addr) (*(volatile unsigned char *) (addr))
#endif

#ifndef readw
#define readw(addr) (*(volatile unsigned short *) (addr))
#endif

#ifndef readl
#define readl(addr) (*(volatile unsigned int *) (addr))
#endif

#ifndef writeb
#define writeb(b,addr) (*(volatile unsigned char *) (addr) = (b))
#endif

#ifndef writew
#define writew(b,addr) (*(volatile unsigned short *) (addr) = (b))
#endif

#ifndef writel
#define writel(b,addr) (*(volatile unsigned int *) (addr) = (b))
#endif

/* Bit number and mask of MSI in the interrupt mask and status register */
#define	QDPC_INTR_MSI_BIT		0
#define QDPC_INTR_MSI_MASK		(1 << QDPC_INTR_MSI_BIT)

/* Enable interrupt for detecting EP reset */
extern void enable_ep_rst_detection(struct net_device *ndev);
/* Disable interrupt for detecting EP reset */
extern void disable_ep_rst_detection(struct net_device *ndev);
/* Interrupt context for detecting EP reset */
extern void handle_ep_rst_int(struct net_device *ndev);

/* Allocated buffer size for a packet */
#define SKB_BUF_SIZE		2048

/* NPU Max BUF size */
#define NPU_MAX_BUF_SIZE	SKB_BUF_SIZE

/* NPU Max BUF size */
#define VMAC_NPU_MAX_BUF_SIZE	SKB_BUF_SIZE

/* NPU Max BUF size */
#define NPU_MAX_BUF_SIZE	SKB_BUF_SIZE

/* Transmit and Receive Queue Length */
#define QDPC_TX_QUEUE_SIZE	512
#define QDPC_RX_QUEUE_SIZE	512

#define QDPC_TX_MAX_BD_SIZE_B0	32
#if defined(VMAC_64BIT)
/*
 * HHBM buffer list is not enough for 64bit platfrom,
 * Reduce to 256 temporarily
 */
#define QDPC_RX_MAX_BD_SIZE_B0	256
#else
#define QDPC_RX_MAX_BD_SIZE_B0	512
#endif

/* Customer defined function	*/
#define qdpc_platform_init()                  0
#define qdpc_platform_exit()                  do { } while(0)

#define TX_QUEUE_FLOW_CONTROL

#endif				/* __QDPC_PFDEP_H__ */
