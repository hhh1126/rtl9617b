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

#ifndef __QTN_BASE_EARLY_FLASH_H
#define __QTN_BASE_EARLY_FLASH_H

/* Offset in MTD_PARTNAME_UBOOT_BIN partition */
#define BOOTCFG_EARLY_FLASH_CFG_START_OFFSET		(0x20)

#ifndef __ASSEMBLY__
struct early_flash_config {
	uint32_t	method;
	uint32_t	ipaddr;
	uint32_t	serverip;
	uint32_t	pcie_hw_cfg;	/* PHCW, PCIe Hardware Configuration Word in flash */
	/*
	 * Field pcie_hw_config is overloaded. A macro is used because QCSAPI RPC does not support unions.
	 * When uboot is copied into sram, it stores timestamp that PCIe is initialized
	 */
#define UBOOT_EF_PCIE_UPTIME	pcie_hw_cfg
	uint32_t	rsvd_for_br;	/* Reserved for branch instuction */
	uint8_t		built_time_utc_sec[11];
	uint8_t		uboot_type;
} __attribute__ ((packed));
#endif /* __ASSEMBLY__ */

#define RUBY_BOOT_METHOD_TRYLOOP	0
#define RUBY_BOOT_METHOD_TFTP		1
#define RUBY_BOOT_METHOD_BOOTP		2
#define RUBY_BOOT_METHOD_MAX		3

#define PCIE_HW_CFG_OFFSET	12	/* Used in assembly code, must be consistent with  struct early_flash_config */

/* definition for PCIe Hardware Configuration Word */
#define PHCW_EN_LDO		(1 << 0)
#define PHCW_PORT_SEL		(1 << 1)
#define PHCW_SKIP_EP_RX_DET	(1 << 2)
#define PHCW_LANE_NUM		(1 << 3)
#define PHCW_EP_MODE		(1 << 5)
#define PHCW_DEV_ID_MSK		(0xff << 16)
#define PHCW_DEV_ID_OFFSET	(16)
#define PHCW_MAGIC_NUM		(0xa5 << 24)
#define PHCW_MAGIC_MSK		(0xff << 24)

#endif /* __QTN_BASE_EARLY_FLASH_H */
