/*
 * (C) Copyright 2010 Quantenna Communications Inc.
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
 * Header file which describes Topaz platform.
 * Has to be used by both kernel and bootloader.
 */

#ifndef __TOPAZ_CONFIG_H
#define __TOPAZ_CONFIG_H

#ifndef __QTN_CONFIG_H_INCLUDED
#error "Include qtn_config.h instead"
#endif

#include "current_platform.h"
#include "base/qtn_base_config.h"

#if !TOPAZ_FPGA_PLATFORM
#undef TOPAZ_ICACHE_WORKAROUND
#endif

/*
 * FIXME
 * BBIC5-FPGA: Just for MuC bring-up. Unnecessary functions will be disabled.
 * Should be deleted after other modules(wmac & wlan and so on) are OK.
 */
/* A workaround for A0, need to undef it for B0 */

#define PEARL_WAR_CPU_DSP2

#define PEARL_RXSPUR_WAR
#define PEARL_SPCS
/* A workaround for NRX=8 in long range issue, we don't need it with B1 */
#define PEARL_NRX_DUTY_CYCLE
#define PEARL_RSSI_REPORT_WAR
#define PEARL_PPPC_WAR_WITH_PSD_SHAPING
#define PEARL_DAC_WAR
#define PEARL_PPPC_WAR
#define PEARL_SVD_WAR
#define PEARL_RXIQ_LOAD_WAR
#define PEARL_TB_SIGMA_WAR
#ifndef JADE_PLATFORM
#define PEARL_DCACHE_RTL_FIX
#endif

#define PEARL_RADAR_SHORT_PULSE

#define ENABLE_MAC_BB_RESET_DEBUG

#define RX_NEXT_PTR_0_WAR	1
#define RX_STATUS_RECALC_RXLEN	1

#ifndef C0_AS_B1_REPLACE
#define C0_AS_B1_REPLACE	0
#endif

#ifdef JADE_PLATFORM
#define JADE_USE_NATIVE_TX_STATUS_LAYOUT	1
#define JADE_USE_NATIVE_RX_STATUS_LAYOUT	1
#define JADE_USE_NATIVE_RX_VECTOR_LAYOUT	JADE_USE_NATIVE_RX_STATUS_LAYOUT
#define JADE_USE_NATIVE_FCS_LAYOUT		1
#define JADE_USE_TQE_COMPRESSED_DESC		1
#define JADE_WAR_USE_OLD_BM_TX_STATUS		0
#else
#define JADE_USE_NATIVE_TX_STATUS_LAYOUT	0
#define JADE_USE_NATIVE_RX_STATUS_LAYOUT	0
#define JADE_USE_NATIVE_RX_VECTOR_LAYOUT	0
#define JADE_USE_NATIVE_FCS_LAYOUT		0
#define JADE_USE_TQE_COMPRESSED_DESC		0
#define JADE_WAR_USE_OLD_BM_TX_STATUS		0
#endif /* JADE_PLATFORM */

#ifdef JADE_FPGA_PLATFORM
#define JADE_FPGA_WAR_LIMIT_RU_SIZE		1
#else
#define JADE_FPGA_WAR_LIMIT_RU_SIZE		0
#endif /* JADE_FPGA_PLATFORM */

#ifdef JADE_PLATFORM
#define AUC_UPLINK_SCHEDULER			1
#define AUC_ADMA				(0 && JADE_USE_TQE_COMPRESSED_DESC)
#define AUC_MTID_AGG_TX				1
#define AUC_HW_RETRY_BIT			JADE_USE_NATIVE_FCS_LAYOUT
#define AUC_HTP_ACK_AGG_TX			1
#define AUC_HW_SMARTRTR_TXOP			1
#else
#define AUC_UPLINK_SCHEDULER			0
#define AUC_ADMA				0
#define AUC_MTID_AGG_TX				0
#define AUC_HW_RETRY_BIT			0
#define AUC_HTP_ACK_AGG_TX			0
#define AUC_HW_SMARTRTR_TXOP			0
#endif

/*
 * On BBIC5, NC be shared by all WMACs, NCIDX is unique in the whole system(From 0 to 1023).
 *	unique(virtual) NCIDX == Phy-NCIDX.
 *
 * On BBIC6, NC is per uni-WMAC, the range is from 0 to 255.
 *	Up-Layer still use unique(virtual) NCIDX, keep same logic between BBIC5 and BBIC6.
 *	When accessing WMAC-registers, need to map unique(virtual) NCIDX to phy-NCIDX.
 *	When Rx-Done, need to convert phy-INDX in RxStatus to unique(virtual) NCIDX.
 */
#ifdef JADE_PLATFORM
#define QTN_NCIDX_REMAP				1
#else
#define QTN_NCIDX_REMAP				0
#endif

#ifdef PEARL_PLATFORM_C0
/*
 * C0_AS_B1_REPLACE is mutual exclusion with some new features in C0, such as 11ax.
 * To enable those new features, please set C0_AS_B1_REPLACE to 0.
 */
#if C0_AS_B1_REPLACE
	#define AMSDU_HW_DEAGG_8023	0
	#define PEARL_11AX_ENABLED	0
	#define HW_MCAST_ENGINE		0
	#define GCMP_CRYPT_EN		0
	#define PEARL_WMAC_RXP_ENABLED	0
	#define PEARL_TQEW_NEW_Q_AVAIL	0
	#define RTS_DYN_BW_SIGNALING	1
#else
#if SIGMA_TESTBED_SUPPORT
	#define AMSDU_HW_DEAGG_8023	0 /* in 3 address mode DL is better when HW deagg is off */
#else
	#define AMSDU_HW_DEAGG_8023	0
#endif
	#define HW_MCAST_ENGINE		0
	#define GCMP_CRYPT_EN		1
	/*
	 * TODO: possible HW issue on C0, new programming doesn't work in some corner case.
	 * Disable it for now
	 */
	#define PEARL_TQEW_NEW_Q_AVAIL	0
	#define PEARL_11AX_ENABLED	(PEARL_PLATFORM_5GAX | PEARL_PLATFORM_10GAX)
	#define RTS_DYN_BW_SIGNALING	1

#ifdef PEARL_VELOCE
	#define PEARL_WMAC_RXP_ENABLED	0
#else
	#define PEARL_WMAC_RXP_ENABLED	0
#endif /* PEARL_VELOCE */
	#define QTN_CQE_ENABLED
#endif /* C0_AS_B1_REPLACE */

#define PEARL_EVM_WAR
#define PEARL_EVM_CAL

#else /* NON-PEARL_PLATFORM_C0 */
#define AMSDU_HW_DEAGG_8023	0
#define PEARL_WMAC_RXP_ENABLED	0
#define PEARL_11AX_ENABLED	0
#define HW_MCAST_ENGINE		0
#define GCMP_CRYPT_EN		0
#define PEARL_TQEW_NEW_Q_AVAIL	0
#endif /* PEARL_PLATFORM_C0 */

#if PEARL_11AX_ENABLED
#define PEARL_11AX_RX_TRIGGER	1
#define HE_PERIODIC_TRIGGERTX   1
#define PEARL_11AX_NDP_ECO
#else
#define PEARL_11AX_RX_TRIGGER	0
#endif /* PEARL_11AX_ENABLED */

#ifdef JADE_PLATFORM
#define JADE_USE_HW_PSDU_LEN_CALC	(PEARL_11AX_ENABLED && 0)
#else
#define JADE_USE_HW_PSDU_LEN_CALC	0
#endif

#define PEARL_WMAC_RXP_PPCTL_OFFSET	160

#if defined(__linux__)
	#define RUBY_SYS_CTL_MMAP_REGVAL	(TOPAZ_SYS_CTL_UNIFIED_MAP | TOPAZ_SYS_CTL_ALIAS_MAP)
#else
	#define RUBY_SYS_CTL_MMAP_REGVAL	RUBY_SYS_CTL_LINUX_MAP(0x1)
#endif

/*
 * Setting UPF_SPD_FLAG gives a developer the option to set the
 * flag to match a UPF_ define from <linux>/include/linux/serial_core.h
 * or set the value to 0 to use the default baud rate setting DEFAULT_BAUD
 */
#define UPF_SPD_FLAG	0
#define DEFAULT_BAUD	TOPAZ_SERIAL_BAUD

#define CONFIG_RUBY_BROKEN_IPC_IRQS	0

#define RUBY_IPC_HI_IRQ(bit_num)	((bit_num) + 8)
#define RUBY_M2L_IPC_HI_IRQ(bit_num)	(bit_num)

#define PLATFORM_REG_SWITCH(reg1, reg2)	(reg2)

#define writel_topaz(a, b)		writel(a, b)

#define QTN_VLAN_LLC_ENCAP		1

#define DSP_ENABLE_STATS		1

/*
 * TACMAP Config:
 *	max-tid =  8: max-node-num can be 128/256/384/512/640/768/896/1024
 *	max-tid = 16: max-node-num can be 64/128/196/256/320/384/448/512
 */
#define QTN_TACMAP_NODE_MAX	1024
#define QTN_TACMAP_TID_MAX	8

#define QTN_MAX_NUM_STA_TOTAL		256

#ifndef MAC_UNITS
#define MAC_UNITS			AUC_CORES_NUM
#endif

#define QTN_MAX_NUM_STA_DEF_WMAC0	128
#define QTN_MAX_NUM_STA_DEF_WMAC1	0
#define QTN_MAX_NUM_STA_DEF_DUAL_WMAC1	64

#if MAC_UNITS == 1
#define MULTI_UNITS_ARRAY_INIT(x)		{x}
#define MULTI_UNITS_ARRAY_INIT_DIFF(x, y, z)	{x}
#define MULTI_UNITS_AUC_WMAC_MAP		{0}
#elif MAC_UNITS == 2
#define MULTI_UNITS_ARRAY_INIT(x)		{x, x}
#define MULTI_UNITS_ARRAY_INIT_DIFF(x, y, z)	{x, y}
#define MULTI_UNITS_AUC_WMAC_MAP		{0, 2}
#elif MAC_UNITS == 3
#define MULTI_UNITS_ARRAY_INIT(x)		{x, x, x}
#define MULTI_UNITS_ARRAY_INIT_DIFF(x, y, z)	{x, y, z}
#define MULTI_UNITS_AUC_WMAC_MAP		{0, 2, 1}
#else
#error "Wrong MAC_UNITS"
#endif

#if AUC_CORES_NUM == 1
#define MULTI_AUCS_ARRAY_INIT(x)		{x}
#define MULTI_AUCS_ARRAY_INIT_DIFF(x, y, z)	{x}
#elif AUC_CORES_NUM == 2
#define MULTI_AUCS_ARRAY_INIT(x)		{x, x}
#define MULTI_AUCS_ARRAY_INIT_DIFF(x, y, z)	{x, y}
#elif AUC_CORES_NUM == 3
#define MULTI_AUCS_ARRAY_INIT(x)		{x, x, x}
#define MULTI_AUCS_ARRAY_INIT_DIFF(x, y, z)	{x, y, z}
#else
#error "Wrong AUC_CORES_NUM"
#endif

#if AUC_TOTAL_CORES_NUM == 1
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x}
#elif AUC_TOTAL_CORES_NUM == 2
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x, x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x, y}
#elif AUC_TOTAL_CORES_NUM == 3
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x, x, x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x, y, z}
#elif AUC_TOTAL_CORES_NUM == 4
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x, x, x, x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x, y, z, a}
#elif AUC_TOTAL_CORES_NUM == 5
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x, x, x, x, x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x, y, z, a, b}
#elif AUC_TOTAL_CORES_NUM == 6
#define MULTI_TOTAL_AUCS_ARRAY_INIT(x)				{x, x, x, x, x, x}
#define MULTI_TOTAL_AUCS_ARRAY_INIT_DIFF(x, y, z, a, b, c)	{x, y, z, a, b, c}
#else
#error "Wrong AUC_TOTAL_CORES_NUM"
#endif

#define QTN_MAX_NUM_STA_DEF_WMAC2	\
	(QTN_MAX_NUM_STA_TOTAL - QTN_MAX_NUM_STA_DEF_WMAC0 - QTN_MAX_NUM_STA_DEF_WMAC1)
#define QTN_MAX_NUM_STA_DEF_DUAL_WMAC2	\
	(QTN_MAX_NUM_STA_TOTAL - QTN_MAX_NUM_STA_DEF_WMAC0 - QTN_MAX_NUM_STA_DEF_DUAL_WMAC1)
#define QTN_MAX_BSS_VAPS		16
#define QTN_MAX_WDS_VAPS		12
#define QTN_MAX_WEP			4

#define QTN_NODE_NUMBER		\
	(QTN_MAX_NUM_STA_TOTAL + QTN_MAX_VAPS * MAC_UNITS + QTN_MAX_WEP * MAC_UNITS)
#define QTN_TID_NUMBER			8

/*
 * The max capacity: NODE_NUMBER/TID_NUMBER shouldn't exceed them.
 */
#define QTN_NODE_MAX			1024
#define QTN_TID_MAX			8
#define QTN_MAX_VAPS			((QTN_MAX_BSS_VAPS) + (QTN_MAX_WDS_VAPS))

#if QTN_NODE_NUMBER > QTN_NODE_MAX
#error QTN_NODE_NUMBER exceeds HW capability
#endif

#if ((QTN_MAX_NUM_STA_DEF_WMAC0 % 8) || (QTN_MAX_NUM_STA_DEF_WMAC1 % 8) || \
	(QTN_MAX_NUM_STA_DEF_WMAC2 % 8) || (QTN_MAX_NUM_STA_DEF_DUAL_WMAC1 % 8) || \
	(QTN_MAX_NUM_STA_DEF_DUAL_WMAC2 % 8) || (QTN_NODE_NUMBER % 8))
#error Assoc limit values must be multiple of 8
#endif

#define QTN_BSS_COUNT_PER_GROUP		8

#define QTN_FORCE_CRASH	1

#define QTN_MONITOR_A2M_IPC	1

#endif /* #ifndef __TOPAZ_CONFIG_H */

