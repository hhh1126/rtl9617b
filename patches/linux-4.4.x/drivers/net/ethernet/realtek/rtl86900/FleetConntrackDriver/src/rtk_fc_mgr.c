/*
 * Copyright (C) 2019 Realtek Semiconductor Corp.
 * All Rights Reserved.
 *
 * This program is the proprietary software of Realtek Semiconductor
 * Corporation and/or its licensors, and only be used, duplicated,
 * modified or distributed under the authorized license from Realtek.
 *
 * ANY USE OF THE SOFTWARE OTHER THAN AS AUTHORIZED UNDER
 * THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
*/
#define COMPILE_RTK_L34_FC_MGR_MODULE 1

#include <linux/module.h>

#include <rtk_fc_mgr.h>
#include <rtk_fc_mgrTRx.h>
#include <rtk_fc_helper.h>
#include <rtk_fc_port.h>


uint8 MOD_PROBE_LOG=1;		// 1: enable console log; 0: disable console log


// NIC driver
extern int drv_nic_register_rxhook(int portmask,int priority,p2rfunc_t rx);
extern int drv_nic_unregister_rxhook(int portmask,int priority,p2rfunc_t rx);

#if defined(CONFIG_RTK_L34_G3_PLATFORM) && defined(CONFIG_RTK_NIC_TX_HOOK)
extern int nic_register_txhook(p2tfunc_t tx);
extern int nic_txhook_init(void);
extern int nic_txhook_exit(void);
#endif


__SRAM_FC_DATA rtk_fc_mgr_database_t fc_mgr_db;

#if !defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
/************************************************************************************************/
/*	Initialized Mapping Table for Wlan HWNAT acceleration.												*/
/*	1. The WLAN Dev_ID and Dev_Name should be one-to-one mapping, no conflict.							*/
/*	2. Ext Port Index starts from:																	*/
/*		2.1. IF 9607C series (ApolloPro)	: RTK_FC_MAC_EXT_PORT0 	(1)									*/
/*		2.2. IF 8277 series (G3)			: RTK_FC_MAC_EXT_CPU 	(0)									*/
/*	3. If any two wlan devices share same ext port, the performance is poor because DA lookup is necessary		*/
/************************************************************************************************/	
rtk_fc_wlan_initmap_t wlanInitMap[] = 
{
	// #WLAN DEV ID			#DEV NAME		#CPU PORT ID						#CPU PORT EXT ID
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)

#if !defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
	/*
	 * RTL9607C / RTL9603C / RTL9603CVD / RTL8198D
	 */
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN0_CLIENT_INTF, "wlan0-vxd",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_MESH_SUPPORT)
	{RTK_FC_WLAN0_MESH_INTF, "wlan-msh",	{RTK_FC_MAC_PORT_MASTERCPU_CORE0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_FC_RTL9607C_SERIES)
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
#endif

#if defined(CONFIG_RTK_REMOTE_ADSL)
	{RTK_FC_WLANx_ATM_VC0_INTF, "vc0", 		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC1_INTF, "vc1", 		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC2_INTF, "vc2", 		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC3_INTF, "vc3", 		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC4_INTF, "vc4",		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC5_INTF, "vc5",		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC6_INTF, "vc6",		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC7_INTF, "vc7",		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
#endif

#if defined(CONFIG_USB_RTL8152)
	{RTK_FC_WLANx_USB_INTF, "eth1",        		{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
#else
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN1_CLIENT_INTF, "wlan1-vxd",	{RTK_FC_MAC_PORT_MASTERCPU_CORE1, RTK_FC_MAC_EXT_PORT5}},
#endif
#endif
#if defined(CONFIG_GMAC2_USABLE)
	{RTK_FC_WLAN2_ROOT_INTF, "wlan2", 		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN2_VAP0_INTF, "wlan2-vap0", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN2_VAP1_INTF, "wlan2-vap1", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN2_VAP2_INTF, "wlan2-vap2", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN2_VAP3_INTF, "wlan2-vap3",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN2_VAP4_INTF, "wlan2-vap4",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP5_INTF, "wlan2-vap5",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP6_INTF, "wlan2-vap6",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN2_CLIENT_INTF, "wlan2-vxd",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#endif
#endif

#else //CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID
	/* 
	 * RTL9607C (03C) (03CVD), see rtk_fc_wlan_init() for wlan port configuration
	 */
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{0, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{0, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{0, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{0, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{0, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN0_CLIENT_INTF, "wlan0-vxd",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_MESH_SUPPORT)
	{RTK_FC_WLAN0_MESH_INTF, "wlan-msh",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_FC_RTL9607C_SERIES)
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{0, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{0, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{0, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{0, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{0, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif

#if defined(CONFIG_USB_RTL8152)
	{RTK_FC_WLANx_USB_INTF, "eth1",        		{0, RTK_FC_MAC_EXT_PORT5}},
#else
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN1_CLIENT_INTF, "wlan1-vxd",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#endif
#if defined(CONFIG_GMAC2_USABLE)
	{RTK_FC_WLAN2_ROOT_INTF, "wlan2", 		{0, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN2_VAP0_INTF, "wlan2-vap0", 	{0, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN2_VAP1_INTF, "wlan2-vap1", 	{0, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN2_VAP2_INTF, "wlan2-vap2", 	{0, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN2_VAP3_INTF, "wlan2-vap3",	{0, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN2_VAP4_INTF, "wlan2-vap4",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP5_INTF, "wlan2-vap5",	{0, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP6_INTF, "wlan2-vap6",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN2_CLIENT_INTF, "wlan2-vxd",	{0, RTK_FC_MAC_EXT_PORT5}},
#endif
#endif
#endif
	

#endif //!CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID
	
#elif defined (CONFIG_RTK_L34_G3_PLATFORM)
#if defined(CONFIG_FC_RTL8198F_SERIES)
	/*
	 * RTL8198F
	 */
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{RTK_FC_MAC_PORT_WLAN_CPU0, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_CLIENT_INTF, "wlan0-vxd",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{RTK_FC_MAC_PORT_WLAN_CPU2, RTK_FC_MAC_EXT_NONE}},
#endif
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_WLAN_CPU3, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_CLIENT_INTF, "wlan1-vxd", 	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{RTK_FC_MAC_PORT_WLAN_CPU5, RTK_FC_MAC_EXT_NONE}},
#endif
#else //!defined(CONFIG_FC_RTL8198F_SERIES)

	/*
	 * CA8277 / CA8277B / RTL9617B / RTL9607DA
	 */
#if defined(CONFIG_BAND_2G_ON_WLAN0)
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_WLAN_CPU0, RTK_FC_MAC_EXT_NONE}},
#else
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 		{RTK_FC_MAC_PORT_WLAN_CPU0, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#endif
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#endif
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#endif

#if defined(CONFIG_FC_QTNA_WIFI_AX)
	{RTK_FC_WLANx_PCIE_INTF, "host0",	{RTK_FC_MAC_PORT_WLAN_CPU1, RTK_FC_MAC_EXT_NONE}},
#endif
 
#endif //CONFIG_FC_RTL8198F_SERIES

#endif
};
#else //defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
#if defined(CONFIG_FC_RTL9607C_SERIES) && defined(CONFIG_GMAC2_USABLE)
/************************************************************************************************/
/*	Initialized Mapping Table for Wlan HWNAT acceleration (CONFIG_FC_WIFI_TRAP_HASH_SUPPORT is enabled).												*/
/*	1. The WLAN Dev_ID and Dev_Name should be one-to-one mapping, no conflict.							*/
/*	2. Ext Port Index starts from:																	*/
/*		2.1. IF 9607C series (ApolloPro)	: RTK_FC_MAC_EXT_PORT0 	(1)									*/
/*	3. If any two wlan devices share same ext port, the performance is poor because DA lookup is necessary		*/
/*	4. CPU Port ID:
		4.1. RTK_FC_MAC_PORT_MAINCPU: RTK_FC_MAC_PORT_MASTERCPU_CORE0 and RTK_FC_MAC_PORT_MASTERCPU_CORE1 are use to support trap hash
		4.2. RTK_FC_MAC_PORT_SLAVECPU: not support trap hash*/
/************************************************************************************************/
const rtk_fc_wlan_initmap_t wlanInitMap[] =
{
	// #WLAN DEV ID			#DEV NAME		#CPU PORT ID						#CPU PORT EXT ID
#if !defined(CONFIG_BAND_2G_ON_WLAN0)
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN0_CLIENT_INTF, "wlan0-vxd", {RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN1_CLIENT_INTF, "wlan1-vxd", {RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif

#else // define(CONFIG_BAND_2G_ON_WLAN0
	{RTK_FC_WLAN0_ROOT_INTF, "wlan0", 	 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN0_VAP0_INTF, "wlan0-vap0",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN0_VAP1_INTF, "wlan0-vap1",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN0_VAP2_INTF, "wlan0-vap2",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN0_VAP3_INTF, "wlan0-vap3",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN0_VAP4_INTF, "wlan0-vap4",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP5_INTF, "wlan0-vap5",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN0_VAP6_INTF, "wlan0-vap6",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN0_CLIENT_INTF, "wlan0-vxd", {RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
	{RTK_FC_WLAN1_ROOT_INTF, "wlan1", 		{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT0}},
	{RTK_FC_WLAN1_VAP0_INTF, "wlan1-vap0", 	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT1}},
	{RTK_FC_WLAN1_VAP1_INTF, "wlan1-vap1", 	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT2}},
	{RTK_FC_WLAN1_VAP2_INTF, "wlan1-vap2", 	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT3}},
	{RTK_FC_WLAN1_VAP3_INTF, "wlan1-vap3",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN1_VAP4_INTF, "wlan1-vap4",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP5_INTF, "wlan1-vap5",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN1_VAP6_INTF, "wlan1-vap6",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN1_CLIENT_INTF, "wlan1-vxd", {RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#endif

#if defined(CONFIG_RTL_MESH_SUPPORT)
	{RTK_FC_WLAN0_MESH_INTF, "wlan-msh",	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif

#if defined(CONFIG_RTK_REMOTE_ADSL)
	{RTK_FC_WLANx_ATM_VC0_INTF, "vc0",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC1_INTF, "vc1",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC2_INTF, "vc2",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC3_INTF, "vc3",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC4_INTF, "vc4",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC5_INTF, "vc5",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC6_INTF, "vc6",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLANx_ATM_VC7_INTF, "vc7",		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif

#if defined(CONFIG_USB_RTL8152)
	{RTK_FC_WLANx_USB_INTF, "eth1",        	{RTK_FC_MAC_PORT_MAINCPU, RTK_FC_MAC_EXT_PORT5}},
#endif
	{RTK_FC_WLAN2_ROOT_INTF, "wlan2", 		{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP0_INTF, "wlan2-vap0", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP1_INTF, "wlan2-vap1", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP2_INTF, "wlan2-vap2", 	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP3_INTF, "wlan2-vap3",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT4}},
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	{RTK_FC_WLAN2_VAP4_INTF, "wlan2-vap4",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP5_INTF, "wlan2-vap5",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
	{RTK_FC_WLAN2_VAP6_INTF, "wlan2-vap6",	{RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif
#if defined(CONFIG_RTL_REPEATER_MODE_SUPPORT)
	{RTK_FC_WLAN2_CLIENT_INTF, "wlan2-vxd", {RTK_FC_MAC_PORT_SLAVECPU, RTK_FC_MAC_EXT_PORT5}},
#endif

};
#else
#error "CONFIG_FC_WIFI_TRAP_HASH_SUPPORT support within CONFIG_RTK_L34_XPON_PLATFORM conf only"
#endif
#endif
const size_t wlanInitMap_size = sizeof(wlanInitMap) / sizeof(rtk_fc_wlan_initmap_t);

static int rtk_fc_mgr_spin_lock_init(void)
{
	spin_lock_init(&fc_mgr_db.lock_fc);
	spin_lock_init(&fc_mgr_db.lock_rtnetlinkTimer);
	spin_lock_init(&fc_mgr_db.lock_tracefilterShow);
	return 0;
}

int rtk_fc_mgr_global_spin_lock(void)
{
	fc_spin_lock(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_global_spin_unlock(void)
{
	fc_spin_unlock(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_global_spin_lock_bh(void)
{
	fc_spin_lock_bh(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_global_spin_unlock_bh(void)
{
	fc_spin_unlock_bh(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_global_spin_lock_bh_irq_protect(void)
{
	fc_spin_lock_bh_irq_protect(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_global_spin_unlock_bh_irq_protect(void)
{
	fc_spin_unlock_bh_irq_protect(&fc_mgr_db.lock_fc);
	return SUCCESS;
}

int rtk_fc_mgr_rtnetlink_timer_spin_lock_bh(void)
{
	fc_spin_lock_bh(&fc_mgr_db.lock_rtnetlinkTimer);
	return SUCCESS;
}

int rtk_fc_mgr_rtnetlink_timer_spin_unlock_bh(void)
{
	fc_spin_unlock_bh(&fc_mgr_db.lock_rtnetlinkTimer);
	return SUCCESS;
}

int rtk_fc_mgr_tracefilter_spin_lock_bh(void)
{
	fc_spin_lock_bh(&fc_mgr_db.lock_tracefilterShow);
	return SUCCESS;
}

int rtk_fc_mgr_tracefilter_spin_unlock_bh(void)
{
	fc_spin_unlock_bh(&fc_mgr_db.lock_tracefilterShow);
	return SUCCESS;
}

static int rtk_fc_mgr_nic_init(void)
{
	
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD) &&defined(CONFIG_FC_RTL9607C_SERIES)
	if(drv_nic_register_rxhook((int)RTK_FC_ALL_MAC_PORTMASK, RE8686_RXPRI_NPTV6_FF, rtk_fc_nicHook_rx_skb_NPTv6FastForward) < 0) {
		//WARNING("FleetConntrack driver was fail to reigster nic Rx funciton!");
	}
#endif
	/*register NIC rx handler*/
	if(drv_nic_register_rxhook((int)RTK_FC_ALL_MAC_PORTMASK, FC_NICRX_PRI, rtk_fc_skb_rx) < 0) {
		//WARNING("FleetConntrack driver was fail to reigster nic Rx funciton!");
	}

#if defined(CONFIG_RTK_L34_G3_PLATFORM) && defined(CONFIG_RTK_NIC_TX_HOOK)
	if(nic_register_txhook(rtk_fc_skb_tx) != SUCCESS) {
		//WARNING("FleetConntrack driver was fail to reigster nic Tx funciton!");
	}
#endif	
	
	return 0;
}

static int rtk_fc_mgr_nic_exit(void)
{
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD) &&defined(CONFIG_FC_RTL9607C_SERIES)
	/*unregister NIC rx handler*/
	if(drv_nic_unregister_rxhook((int)RTK_FC_ALL_MAC_PORTMASK, RE8686_RXPRI_NPTV6_FF, rtk_fc_nicHook_rx_skb_NPTv6FastForward)) {
		//WARNING("FleetConntrack driver was fail to unreigster nic Rx funciton!");
	}
#endif

	/*unregister NIC rx handler*/
	if(drv_nic_unregister_rxhook((int)RTK_FC_ALL_MAC_PORTMASK, FC_NICRX_PRI, rtk_fc_skb_rx)) {
		//WARNING("FleetConntrack driver was fail to unreigster nic Rx funciton!");
	}


#if defined(CONFIG_RTK_L34_G3_PLATFORM) && defined(CONFIG_RTK_NIC_TX_HOOK)
	if(nic_txhook_exit() != SUCCESS) {
		//WARNING("FleetConntrack driver was fail to unreigster nic Tx funciton!");
	}
#endif	

	return 0;
}

int rtk_fc_mgr_init(void)
{
	int percpu_all_size=0;
		
	memset(&fc_mgr_db, 0, sizeof(fc_mgr_db));
	//fc_mgr_db.debug_prk = 1;

	if(MOD_PROBE_LOG)
		FCMGR_ERR("module init");

#if defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
	{
		uint32 ChipId, Rev, Subtype;
		rtk_switch_version_get(&ChipId, &Rev, &Subtype);
		fc_mgr_db.chipId = ChipId;
		if(fc_mgr_db.chipId == RTL9607C_CHIP_ID) {
			fc_mgr_db.macport_pon = 5;
			fc_mgr_db.macport_scpu= 7;
			fc_mgr_db.macport_mcpu_0 = 9;
			fc_mgr_db.macport_mcpu_1 = 10;

			fc_mgr_db.mac10extport_0 = 6;
			fc_mgr_db.mac7extport_0 = 12;
		}else {
			fc_mgr_db.macport_pon = 4;
			fc_mgr_db.macport_scpu= 5;
			fc_mgr_db.macport_mcpu_0 = 5;
			fc_mgr_db.macport_mcpu_1 = 5;
			
			fc_mgr_db.mac10extport_0 = 0;
			fc_mgr_db.mac7extport_0 = 0;
		}
		if(MOD_PROBE_LOG)
			FCMGR_ERR("hybrid image detect 0x%x", fc_mgr_db.chipId);
	}
#endif
	
	/*
	 * Sequence: MGR_DB -> HELPER -> HOOK_DATAPATH
	 */

	rtk_fc_mgr_spin_lock_init(); //should be called before rtk_fc_helper_init()
	
	rtk_fc_trx_init();				// database for nic or wlan trx

	rtk_fc_helper_init();			// helper function for FC core module

	rtk_fc_mgr_nic_init();		// callback function for nic driver

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)

		percpu_all_size = ((int)sizeof(rtk_fc_nicrx_ipi_ctrl_t) + (int)sizeof(rtk_fc_nicrx_ipi_ctrl_t)
							+ (int)sizeof(rtk_fc_nicrx_ring_ctrl_t) + (int)sizeof(rtk_fc_nicrx_hiring_ctrl_t)) * NR_CPUS;


	
#endif

	if(MOD_PROBE_LOG) {
		printk("RTK FleetConntrack Driver - manager module init\n");
		printk(" - mem usage %d KB (db:%d nicrx_ipi:%d)\n", 
			((int)sizeof(fc_mgr_db) +  percpu_all_size) /1024, 
			(int)sizeof(fc_mgr_db),  percpu_all_size);
	}
	
#if 0
	printk(" - nicrx_ipi:%d\n", (int)sizeof(nicrx_ipi) * NR_CPUS);
	printk(" - nicrx_hi_ipi:%d\n", (int)sizeof(nicrx_hi_ipi) * NR_CPUS);
	printk(" - nicrx_ring:%d\n", (int)sizeof(nicrx_ring) * NR_CPUS);
	printk(" - nicrx_hiring:%d\n", (int)sizeof(nicrx_hiring) * NR_CPUS);
	/*
	// 9607C
	[    3.470000]  - mem usage 459 KB (db:434816 nicrx_ipi:35584)
	[    3.470000]  - nicrx_ipi:384
	[    3.480000]  - nicrx_hi_ipi:384
	[    3.480000]  - nicrx_ring:32768
	[    3.480000]  - nicrx_hiring:2048
	// 8277
	[   18.277573]  - mem usage 617 KB (db:602496 nicrx_ipi:29696)
	[   18.283131]  - nicrx_ipi:1024
	[   18.286081]  - nicrx_hi_ipi:1024
	[   18.289295]  - nicrx_ring:24576
	[   18.292428]  - nicrx_hiring:3072
	*/
#endif
	
	fc_mgr_db.debug_prk = 0;
	
	return 0;
}

void rtk_fc_mgr_exit(void)
{
	FCMGR_PRK("module exit");
	
	rtk_fc_helper_exit();
	
	rtk_fc_mgr_nic_exit();
	
}


module_init(rtk_fc_mgr_init);
module_exit(rtk_fc_mgr_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Realtek Semiconductor Corp.");
MODULE_DESCRIPTION("HWNAT - FleetConntrack Driver - Manager mdule");
