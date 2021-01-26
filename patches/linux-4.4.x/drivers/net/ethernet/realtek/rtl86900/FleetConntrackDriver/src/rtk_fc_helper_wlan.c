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

#include <dal/dal_mapper.h>
#include <dal/rtl9607c/dal_rtl9607c.h>
#include <linux/netdevice.h>

#include <rtk_fc_helper_wlan.h>
#include <rtk_fc_helper.h>
#include <rtk_fc_mgr.h>
#if defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
#include <rtk/trap.h>
#include <rtk/l2.h>
#endif

#if defined(CONFIG_GMAC2_USABLE)
#include <rtk/cpu.h>
#endif


extern rtk_fc_wlan_initmap_t wlanInitMap[];
extern const size_t wlanInitMap_size;

#if defined(CONFIG_RTL_FC_USB_INTF)
#include "rtk_fc_usb.c"
#endif

int rtk_fc_wlanDevidx2bandidx(rtk_fc_wlan_devidx_t wlanDevIdx)
{
	// software band index. 
	rtk_fc_wlan_id_t band = 0;

	if(wlanDevIdx >= RTK_FC_WLANX_END_INTF){
		FCMGR_ERR("wlanDevIdx %d is not support, please have a check\n", wlanDevIdx);
		return 0;
	}

	if(wlanDevIdx < RTK_FC_WLAN1_ROOT_INTF)
		band = RTK_FC_WLAN_ID_0;
	else if (wlanDevIdx < RTK_FC_WLAN2_ROOT_INTF)
		band = RTK_FC_WLAN_ID_1;
#if defined(CONFIG_GMAC2_USABLE)	
	else 
		band = RTK_FC_WLAN_ID_2;
#endif

	return band;
}

int rtk_fc_wlanDevIdx2port(rtk_fc_wlan_devidx_t wlanDevIdx, rtk_fc_mac_port_idx_t *macPort, rtk_fc_mac_ext_port_idx_t *macExtPort)
{
	if(wlanDevIdx<RTK_FC_WLANX_END_INTF)
	{
		if(fc_mgr_db.wlanDevMap[wlanDevIdx].valid) {

			*macPort = fc_mgr_db.wlanDevMap[wlanDevIdx].portmap.macPortIdx;
			*macExtPort = fc_mgr_db.wlanDevMap[wlanDevIdx].portmap.macExtPortIdx;
			
		}else
			return FAIL;
	
	}else
		return FAIL;
		
	return SUCCESS;
}

__IRAM_FC_NICTRX
int rtk_fc_wlanDevIdx2dev(rtk_fc_wlan_devidx_t wlan_dev_idx, struct net_device **pWifiDev)
{		
	*pWifiDev =NULL;
	
	if(wlan_dev_idx<RTK_FC_WLANX_END_INTF)
	{
		if(fc_mgr_db.wlanDevMap[wlan_dev_idx].valid && 
			fc_mgr_db.wlanDevMap[wlan_dev_idx].wlanDev && 
			netif_running(fc_mgr_db.wlanDevMap[wlan_dev_idx].wlanDev)) {

			*pWifiDev = fc_mgr_db.wlanDevMap[wlan_dev_idx].wlanDev;
		}
	
	}

	if(wlan_dev_idx < RTK_FC_WLANX_END_INTF &&  *pWifiDev==NULL){
		FCMGR_ERR("wlan dev id %d is not ready", wlan_dev_idx);
		return FAIL;
	}else
		return SUCCESS;
}

int rtk_fc_dev2wlanDevIdx(struct net_device *dev, rtk_fc_wlan_devidx_t *wlan_dev_idx)
{
	rtk_fc_wlan_devmap_t *pDevMap=NULL, *pNextDevMap=NULL;
	uint32_t hashidx = rtk_fc_wlan_ifidx_devlist_hash(dev);
	
	*wlan_dev_idx = RTK_FC_WLANX_NOT_FOUND;

	list_for_each_entry_safe(pDevMap, pNextDevMap, &fc_mgr_db.wlanIfidxDevHead[hashidx], ifidxDevList)
	{
		if(pDevMap && (pDevMap->wlanDev!=NULL) && (pDevMap->wlanDev == dev))
		{
			*wlan_dev_idx = pDevMap->wlandevidx;
		}
	}

	if(*wlan_dev_idx!=RTK_FC_WLANX_NOT_FOUND)
		return SUCCESS;
	else{
		return FAIL;
	}
}

int rtk_fc_devName2wlanDevIdx(char *devName, rtk_fc_wlan_devidx_t *wlanDevIdx, rtk_fc_wlan_devmask_t *wlanDevIdMask)
{
	rtk_fc_wlan_devidx_t i;

	*wlanDevIdx= RTK_FC_WLANX_NOT_FOUND;
	*wlanDevIdMask = 0LL;

	// control path, low performance search is fine.
	for(i = 0; i < RTK_FC_WLANX_END_INTF; i++)
	{
		if(!fc_mgr_db.wlanDevMap[i].valid)
			continue;

		if(!strncmp(fc_mgr_db.wlanDevMap[i].wlanDev->name, devName, IFNAMSIZ))
		{
			*wlanDevIdx = i;
			*wlanDevIdMask |= (1LL<<i);
		}
	}

	if(*wlanDevIdx!=RTK_FC_WLANX_NOT_FOUND)
		return SUCCESS;
	else
		return FAIL;
}



__IRAM_FC_NICTRX
int rtk_fc_port2wlanDevidx(rtk_fc_mac_port_idx_t macPort, rtk_fc_mac_ext_port_idx_t macExtPort, rtk_fc_wlan_devidx_t *wlanDevIdx, rtk_fc_wlan_devmask_t *wlanDevIdMask)
{
	rtk_fc_wlan_devmap_t *pDevMap=NULL, *pNextDevMap=NULL;
	uint32_t hashidx = rtk_fc_wlan_port_devlist_hash(macPort, macExtPort);

	*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;
	*wlanDevIdMask = 0LL;
	
	list_for_each_entry_safe(pDevMap, pNextDevMap, &fc_mgr_db.wlanPortDevHead[hashidx], portDevList)
	{
		if(pDevMap 
			&& (pDevMap->portmap.macPortIdx == macPort) 
#if !defined (CONFIG_RTK_L34_G3_PLATFORM)
			&& (pDevMap->portmap.macExtPortIdx == macExtPort)
#endif
			&& (pDevMap->wlandevidx < RTK_FC_WLANX_END_INTF))
		{

			*wlanDevIdMask |= (1LL<<(pDevMap->wlandevidx%63));
			
			if(!pDevMap->shareExtPort) {
				*wlanDevIdx = pDevMap->wlandevidx;
				break;
				
			}else {
				// N wlan to 1 port: could not decide correct wlandevid
				*wlanDevIdx = RTK_FC_WLANX_MULTI_INTF;
			}

		}
	}
	

	return SUCCESS;
}

int rtk_fc_cpuport2wlanDevidx(rtk_fc_mac_port_idx_t macPort, rtk_fc_wlan_devidx_t *wlanDevIdx, rtk_fc_wlan_devmask_t *wlanDevIdMask)
{
	rtk_fc_wlan_devidx_t i;
	bool found = FALSE, duplicate = FALSE;

	*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;
	*wlanDevIdMask = 0;

	// control path, low performance search is fine.
	for(i = 0; i < RTK_FC_WLANX_END_INTF; i++) 
	{
		if(!fc_mgr_db.wlanDevMap[i].valid)
			continue;
		
		if(fc_mgr_db.wlanDevMap[i].portmap.macPortIdx == macPort) {
			if(found)
				duplicate = TRUE;
			
			found = TRUE;
			*wlanDevIdx = i;
			*wlanDevIdMask |= (1LL<<i);
		}
	}

	if(duplicate)
		*wlanDevIdx = RTK_FC_WLANX_MULTI_INTF;

	return SUCCESS;
	
}



int rtk_fc_wlanDevMask2extMask(rtk_fc_wlan_devmask_t wlanDevIdMask, rtk_fc_ext_port_mask_t *extPortMask)
{
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
	int i, wlanDevStart, wlanDevEnd;
	rtk_fc_ext_port_list_t extpidx = 0;

	if(wlanDevIdMask==0)
		return FAIL;
	
	wlanDevStart  = __ffs64(wlanDevIdMask);
	wlanDevEnd = (63 - __builtin_clzll(wlanDevIdMask));

	*extPortMask = FC_EXTMASK_EMPTY;
	
	for(i=wlanDevStart; i<=wlanDevEnd; i++)
	{
		if((1LL<<i) & wlanDevIdMask) {

			extpidx = (fc_mgr_db.wlanDevMap[i].portmap.macExtPortIdx - RTK_FC_MAC_EXT_PORT0);	// offset 1

			if(fc_mgr_db.wlanDevMap[i].portmap.macPortIdx == RTK_FC_MAC_PORT_MAINCPU) {
				//extpidx += FC_EXT_MAC9_PORT0;
			}
#if defined(CONFIG_FC_RTL9607C_SERIES) || defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
			else if(fc_mgr_db.wlanDevMap[i].portmap.macPortIdx == RTK_FC_MAC_PORT_MASTERCPU_CORE1) {
				extpidx += FC_EXT_MAC10_PORT0;
			}else if(fc_mgr_db.wlanDevMap[i].portmap.macPortIdx == RTK_FC_MAC_PORT_SLAVECPU) {
				extpidx += FC_EXT_MAC7_PORT0;
			}
#endif
			else {
				// ERROR
			}

			*extPortMask |= (1<<extpidx);			
		}
	}

		
#elif defined (CONFIG_RTK_L34_G3_PLATFORM)

	*extPortMask = wlanDevIdMask;

#endif

	return SUCCESS;
}

// wifi register: tx func & port map
int rtk_fc_wlan_register(rtk_fc_wlan_devidx_t wlanDevIdx, struct net_device *dev)
{

#if defined(CONFIG_RTK_FC_WLAN_HWNAT_ACCELERATION)

	int i = 0, targetInitTblIdx = 0;

	if(dev==NULL) return FAIL;

	FCMGR_PRK("REGISTER wlan dev %s, bind to wlandevid %d\n", dev->name, wlanDevIdx);
	
	// two method to do wlan dev register
	//	#1. call from FC core netdev event, the wlandevid is implemented by string comparison if wlanDevIdx==RTK_FC_WLANX_NOT_FOUND.
	//	#2. call form other device driver, it is used for manual registration if wlanInitMap[i].manuallyReg = 1.
	
	for(i = 0; i < wlanInitMap_size; i++) {
		if((!strncmp(wlanInitMap[i].ifname, dev->name, IFNAMSIZ))) {

			if(wlanDevIdx == RTK_FC_WLANX_NOT_FOUND) {
				//method #1: auto registration
				if(wlanInitMap[i].manuallyReg == 1)
					return FAIL;
				else 
					wlanDevIdx = wlanInitMap[i].wlanDevIdx;
			}
			else if (wlanDevIdx != RTK_FC_WLANX_NOT_FOUND) {		
				// method #2: manual registration	
				if(wlanInitMap[i].manuallyReg == 0)
					return FAIL;

			} 
			
			targetInitTblIdx = i;
			
			break;
		}
	}

	if(wlanDevIdx < RTK_FC_WLANX_END_INTF) {

		struct net_device_ops *pOps=NULL;	
		uint32_t hashidx;
#if defined(CONFIG_FC_CA8277B_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
		if(wlanInitMap[targetInitTblIdx].portmap.macPortIdx == RTK_FC_MAC_PORT_WLAN_CPU4 ||
			wlanInitMap[targetInitTblIdx].portmap.macPortIdx == RTK_FC_MAC_PORT_WLAN_CPU5 )
		{
			return FAIL;
		}
#endif
		if(fc_mgr_db.wlanDevMap[wlanDevIdx].valid == FALSE) {
			// start registration

			pOps = &fc_mgr_db.wlanDevMap[wlanDevIdx].wlan_fc_ops;
			
			if((fc_mgr_db.wlanDevMap[wlanDevIdx].wlan_native_devops == NULL) && (pOps->ndo_start_xmit != rtk_fc_fastfwd_dev_xmit) ){
				
				FCMGR_PRK("REGISTER wlan dev %s, bind to wlandevid %d\n", dev->name, wlanDevIdx);
				
				// save native tx function and replace to fc wlan tx for egress learning							
				fc_mgr_db.wlanDevMap[wlanDevIdx].wlan_native_devops = dev->netdev_ops;	
				memcpy(pOps,dev->netdev_ops,sizeof(struct net_device_ops));
				pOps->ndo_start_xmit = rtk_fc_fastfwd_dev_xmit;
				dev->netdev_ops = pOps;
		
				fc_mgr_db.wlanDevMap[wlanDevIdx].valid = TRUE;
				fc_mgr_db.wlanDevMap[wlanDevIdx].band = rtk_fc_wlanDevidx2bandidx(wlanDevIdx);
				fc_mgr_db.wlanDevMap[wlanDevIdx].wlandevidx = wlanDevIdx;
				fc_mgr_db.wlanDevMap[wlanDevIdx].wlanDev = dev;

				// init wlan dev map

				// sync share port info.
				for(i = 0; i < RTK_FC_WLANX_END_INTF; i++) {
					if((fc_mgr_db.wlanDevMap[i].valid == TRUE) && 
						(fc_mgr_db.wlanDevMap[i].portmap.macPortIdx == wlanInitMap[targetInitTblIdx].portmap.macPortIdx)) {
						
						fc_mgr_db.wlanDevMap[i].shareCpuPort = TRUE;
						fc_mgr_db.wlanDevMap[wlanDevIdx].shareCpuPort = TRUE;

						if(fc_mgr_db.wlanDevMap[i].portmap.macExtPortIdx== wlanInitMap[targetInitTblIdx].portmap.macExtPortIdx) {
							fc_mgr_db.wlanDevMap[i].shareExtPort = TRUE;
							fc_mgr_db.wlanDevMap[wlanDevIdx].shareExtPort = TRUE;
						}
					}
				}
				
				memcpy(&fc_mgr_db.wlanDevMap[wlanDevIdx].portmap, &wlanInitMap[targetInitTblIdx].portmap, sizeof(rtk_fc_pmap_t));

				hashidx = rtk_fc_wlan_ifidx_devlist_hash(dev);
				fc_mgr_db.wlanDevMap[wlanDevIdx].devHashIdx = hashidx;
				list_add_tail(&fc_mgr_db.wlanDevMap[wlanDevIdx].ifidxDevList, &fc_mgr_db.wlanIfidxDevHead[hashidx]);

				hashidx = rtk_fc_wlan_port_devlist_hash(fc_mgr_db.wlanDevMap[wlanDevIdx].portmap.macPortIdx, fc_mgr_db.wlanDevMap[wlanDevIdx].portmap.macExtPortIdx);
				list_add_tail(&fc_mgr_db.wlanDevMap[wlanDevIdx].portDevList, &fc_mgr_db.wlanPortDevHead[hashidx]);			
			}
			
		}


	}
#else
	//Do nothing, just return success
#endif
	return SUCCESS;
}

int rtk_fc_wlan_unregister(struct net_device *dev)
{

#if defined(CONFIG_RTK_FC_WLAN_HWNAT_ACCELERATION)

	int i = 0, wlanDevIdx = -1;

	if(dev==NULL) return FAIL;
	
	FCMGR_PRK("UN-REGISTER wlan dev %s\n", dev->name);
	
	for(i = 0; i < RTK_FC_WLANX_END_INTF; i++) 
	{
		if(fc_mgr_db.wlanDevMap[i].valid == TRUE 
			&& fc_mgr_db.wlanDevMap[i].wlanDev == dev) 
		{
			wlanDevIdx = i;
			break;
		}
	}
	
	if(wlanDevIdx >= 0)
	{
		list_del(&fc_mgr_db.wlanDevMap[wlanDevIdx].ifidxDevList);
		list_del(&fc_mgr_db.wlanDevMap[wlanDevIdx].portDevList);
		
		dev->netdev_ops = fc_mgr_db.wlanDevMap[wlanDevIdx].wlan_native_devops;
		
		memset(&fc_mgr_db.wlanDevMap[wlanDevIdx], 0, sizeof(rtk_fc_wlan_devmap_t));
	}
	
#else
	//Do nothing, just return success
#endif
	return SUCCESS;
}

char *rtk_fc_wlan_devmap_devname_get(rtk_fc_wlan_devmap_t *devmap)
{
	return devmap->wlanDev->name;
}

int rtk_fc_wlan_devmap_macportidx_get(rtk_fc_wlan_devmap_t *devmap)
{
	return devmap->portmap.macPortIdx;
}

int rtk_fc_wlan_devmap_macextportidx_get(rtk_fc_wlan_devmap_t *devmap)
{
	return devmap->portmap.macExtPortIdx;
}

int rtk_fc_wlan_devmap_shareextport_get(rtk_fc_wlan_devmap_t *devmap)
{
	if(devmap->shareExtPort == TRUE) {
		return TRUE;
	} else {
		return FALSE;
	}
}

int rtk_fc_wlan_devmap_band_get(rtk_fc_wlan_devmap_t *devmap)
{
	return devmap->band;
}

int rtk_fc_wlan_devmap_get(rtk_fc_wlan_devidx_t wlanDevIdx, rtk_fc_wlan_devmap_t **devmap)
{
	if((wlanDevIdx < RTK_FC_WLANX_END_INTF) && (fc_mgr_db.wlanDevMap[wlanDevIdx].valid)) {

		*devmap = &fc_mgr_db.wlanDevMap[wlanDevIdx];	
		return SUCCESS;
	}else{
	
		*devmap = NULL;
		return FAIL;
	}
	
}

__IRAM_FC_NICTRX
rtk_fc_devtx_t rtk_fc_wlan_hard_start_xmit(rtk_fc_wlan_devidx_t wlan_dev_idx, struct sk_buff *skb, fc_rx_info_t *pRxInfo)
{
	rtk_fc_devtx_t wlanrc = RTK_FC_DEVTX_OK;
#if defined(CONFIG_SMP) && defined(CONFIG_RTK_L34_FC_IPI_WIFI_TX)
	rtk_fc_mgr_dispatch_mode_t *wlanTxDispatch;
#endif
		
	if(unlikely((wlan_dev_idx >= RTK_FC_WLANX_END_INTF) || (fc_mgr_db.wlanDevMap[wlan_dev_idx].valid == FALSE))) {
		FCMGR_ERR("ERROR: dev %s wlandevidx %d skb %p \n", skb->dev->name, wlan_dev_idx, skb); 
		dev_kfree_skb_any(skb);	
		return RTK_FC_DEVTX_ERROR;
	}

	skb->dev = fc_mgr_db.wlanDevMap[wlan_dev_idx].wlanDev;

	if(unlikely(!netif_running(skb->dev))){
		FCMGR_ERR("ERROR: dev %s wlandevidx %d skb %p dev not running\n", skb->dev->name, wlan_dev_idx, skb); 
		dev_kfree_skb_any(skb);	
		return RTK_FC_DEVTX_ERROR;
	}
	
	if(wlan_dev_idx >= RTK_FC_WLANx_ATM_VC0_INTF && wlan_dev_idx <= RTK_FC_WLANx_ATM_VC7_INTF) 
	{
		if(pRxInfo!=NULL) {
			/* fast forward handling */
			if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_QID].startBit != RTK_FC_MGR_RMK_UNDEF) {
				_rtk_fc_set_skbMark_vlaue(skb, FC_MGR_SKBMARK_QID, RXINFO_INT_PRI(pRxInfo));
				FCMGR_PRK("fast forward to devidx %d, skb->mark = 0x%x", wlan_dev_idx, skb->mark);
			}
		}
	}
	
#if defined(CONFIG_SMP) && defined(CONFIG_RTK_L34_FC_IPI_WIFI_TX)
	wlanTxDispatch = &fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_TX + fc_mgr_db.wlanDevMap[wlan_dev_idx].band];

	if(unlikely(wlanTxDispatch->mode == RTK_FC_DISPATCH_MODE_IPI)) {
		rtk_fc_wlantx_info_t wlanTxInfo;
		wlanTxInfo.skb = skb;
		wlanTxInfo.wlandevidx = wlan_dev_idx;
		
		// IPI tx enqueue
		rtk_fc_smp_wlan_tx_dispatch(&wlanTxInfo);

	}else 
#endif
	{
		// direct tx
		wlanrc = fc_mgr_db.wlanDevMap[wlan_dev_idx].wlan_native_devops->ndo_start_xmit(skb, skb->dev);
		if(unlikely(fc_mgr_db.smpStatistic)){
		
			if(RTK_FC_DEVTX_OK == wlanrc) {
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_WIFI_WLAN0_TX+fc_mgr_db.wlanDevMap[wlan_dev_idx].band].smp_static[smp_processor_id()]); 
			}else {
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_WIFI_WLAN0_TX_DROP+fc_mgr_db.wlanDevMap[wlan_dev_idx].band].smp_static[smp_processor_id()]); 
			}
		}
		
	}

	return wlanrc;
}

int rtk_fc_wlan_system_post_init(void)
{
	// mamager module system init should be later than core module init
#if (defined(CONFIG_RTK_L34_XPON_PLATFORM) &&  defined(CONFIG_GMAC2_USABLE)) || defined(CONFIG_RTK_CPU_PORT_FLOW_CONTROL)
	rtk_port_macAbility_t cpuAbility;
#endif

#if (defined(CONFIG_RTK_L34_XPON_PLATFORM) &&  defined(CONFIG_GMAC2_USABLE))
	memset(&cpuAbility,0,sizeof(cpuAbility));
	cpuAbility.duplex = PORT_FULL_DUPLEX;
	cpuAbility.linkStatus = PORT_LINKUP;
	cpuAbility.speed = PORT_SPEED_1000M;
	if(rtk_port_macForceAbility_set(RTK_FC_MAC_PORT_SLAVECPU,cpuAbility))FCMGR_ERR("Set slave CPU port fail!!!\n");
	if(rtk_port_macForceAbilityState_set(RTK_FC_MAC_PORT_SLAVECPU,ENABLED))FCMGR_ERR("Set slave CPU port fail!!!\n");
			
	rtk_cpu_tagAwareByPort_set(RTK_FC_MAC_PORT_SLAVECPU,ENABLED);
	rtk_cpu_trapInsertTagByPort_set(RTK_FC_MAC_PORT_SLAVECPU,ENABLED);
#endif

#if defined(CONFIG_RTK_L34_XPON_PLATFORM) && defined(CONFIG_FC_RTL9607C_SERIES) && defined(CONFIG_LAN_SDS1_FEATURE)
	// 9607C only: take switch port 7 as SDS interface, enable src port block feature. 
	{
		rtk_portmask_t srcPortBlocking;
		rtk_l2_srcPortEgrFilterMask_get(&srcPortBlocking);
		srcPortBlocking.bits[0] |= RTK_FC_MAC_PORT_SLAVECPU;
		rtk_l2_srcPortEgrFilterMask_set(&srcPortBlocking);
	}
#endif

#if defined(CONFIG_RTK_CPU_PORT_FLOW_CONTROL)
	/*Enable cpu port flow control*/
	if(rtk_port_macForceAbility_get(RTK_FC_MAC_PORT_MAINCPU,&cpuAbility)) {
		FCMGR_ERR("Get main CPU port flow control failed!!!\n");
	} else {
		cpuAbility.txFc=ENABLED;
		cpuAbility.rxFc=ENABLED;
		if(rtk_port_macForceAbility_set(RTK_FC_MAC_PORT_MAINCPU,cpuAbility))FCMGR_ERR("Get main CPU port flow control failed!!!\n");
	}

	if(rtk_port_macForceAbility_get(RTK_FC_MAC_PORT_LASTCPU,&cpuAbility)) {
		FCMGR_ERR("Get last CPU port flow control failed!!!\n");
	} else {
		cpuAbility.txFc=ENABLED;
		cpuAbility.rxFc=ENABLED;
		if(rtk_port_macForceAbility_set(RTK_FC_MAC_PORT_LASTCPU,cpuAbility))FCMGR_ERR("Set last CPU port flow control failed!!!\n");
	}

#if defined(CONFIG_GMAC2_USABLE)
	if(rtk_port_macForceAbility_get(RTK_FC_MAC_PORT_SLAVECPU,&cpuAbility)) {
		FCMGR_ERR("Get slave CPU port flow control failed!!!\n");
	} else {
		cpuAbility.txFc=ENABLED;
		cpuAbility.rxFc=ENABLED;
		if(rtk_port_macForceAbility_set(RTK_FC_MAC_PORT_LASTCPU,cpuAbility))FCMGR_ERR("Set slave CPU port flow control failed!!!\n");
	}
#endif
#endif

#if defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
	rtk_l2_illegalPortMoveAction_set(RTK_FC_MAC_PORT_MASTERCPU_CORE1, ACTION_FORWARD); // wifi RX packets may use port 9 or port 10 (Its MAC address will be learned in port 9)

	rtk_trap_cpuTrapHashState_set(ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_SPA, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_SMAC, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_DMAC, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_SIP_INNER, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_DIP_INNER, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_SPORT_INNER, ENABLED);
	rtk_trap_cpuTrapHashMask_set(TRAP_HASH_DPORT_INNER, ENABLED);

	rtk_trap_cpuTrapHashPort_set(0, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(1, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(2, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(3, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(4, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(5, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(6, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(7, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(8, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(9, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(10, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(11, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(12, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(13, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
	rtk_trap_cpuTrapHashPort_set(14, RTK_FC_MAC_PORT_MASTERCPU_CORE0);
	rtk_trap_cpuTrapHashPort_set(15, RTK_FC_MAC_PORT_MASTERCPU_CORE1);
#endif

	return SUCCESS;
}


int rtk_fc_wlan_init(void)
{
	int i = 0;

#if defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
	for(i = 0; i <wlanInitMap_size; i++) {
		
		if(fc_mgr_db.chipId == RTL9607C_CHIP_ID) {
			// 3 GMACs
			if (wlanInitMap[i].wlanDevIdx == RTK_FC_WLANx_USB_INTF)
				wlanInitMap[i].portmap.macPortIdx = 10;
			else if(wlanInitMap[i].wlanDevIdx < RTK_FC_WLAN1_ROOT_INTF)
				wlanInitMap[i].portmap.macPortIdx = 9;
			else if(wlanInitMap[i].wlanDevIdx < RTK_FC_WLAN2_ROOT_INTF)
				wlanInitMap[i].portmap.macPortIdx = 10;
			else
				wlanInitMap[i].portmap.macPortIdx = 7;
				
		}else{
			// 1 GMAC
			wlanInitMap[i].portmap.macPortIdx = 5;
		}
	}
#endif

	for(i=0;i<RTK_FC_WLAN_PORT_BUCKET_SIZE;i++)
		INIT_LIST_HEAD(&fc_mgr_db.wlanPortDevHead[i]);

	
	for(i=0;i<RTK_FC_WLANX_END_INTF;i++) {
		INIT_LIST_HEAD(&fc_mgr_db.wlanIfidxDevHead[i]);
		
		INIT_LIST_HEAD(&fc_mgr_db.wlanDevMap[i].ifidxDevList);
		INIT_LIST_HEAD(&fc_mgr_db.wlanDevMap[i].portDevList);
	}

	return SUCCESS;
}



int rtk_fc_dev_is_wlan_dev(struct net_device *dev, bool *wlandev)
{
	if((dev->priv_flags & IFF_DOMAIN_WLAN) != 0)
		*wlandev = TRUE;
	else if(!strncmp(dev->name, "wlan", 4))
		*wlandev = TRUE;
#if defined(CONFIG_RTK_REMOTE_ADSL)
	else if(!strncmp(dev->name, "vc", 2) && dev->name[3] == '\0')
		*wlandev = TRUE;
#endif
#if defined(CONFIG_RTL_FC_USB_INTF)
#if defined(CONFIG_RTL_FC_USB_AUTO_INTFNAME)
	else if(usb_idx && !strcmp(dev->name, wlanInitMap[usb_idx].ifname))
		*wlandev = TRUE;
#else
	else if(!strcmp(dev->name, "eth1"))
		*wlandev = TRUE;
#endif
#endif

#if defined(CONFIG_FC_QTNA_WIFI_AX)
	else if(!strncmp(dev->name, "host0", 5))
		*wlandev = TRUE;
#endif
 
	else
		*wlandev = FALSE;

	return SUCCESS;	
}



int rtk_fc_wlan_rtmbssid2devidx(rt_wlan_index_t wlanIdx, rt_wlan_mbssid_index_t mbssidIdx, rtk_fc_wlan_devidx_t *wlanDevIdx)
{
	*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;

#if defined(CONFIG_COMMON_RT_API) && defined(CONFIG_RTK_FC_WLAN_HWNAT_ACCELERATION)

	if(wlanIdx == RT_WLAN_DEVICE0_INDEX) {
		switch(mbssidIdx) {
			case RT_WLAN_MBSSID_ROOT_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_ROOT_INTF;
				break;
			case RT_WLAN_MBSSID_VAP0_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP0_INTF;
				break;
			case RT_WLAN_MBSSID_VAP1_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP1_INTF;
				break;
			case RT_WLAN_MBSSID_VAP2_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP2_INTF;
				break;
			case RT_WLAN_MBSSID_VAP3_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP3_INTF;
				break;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
			case RT_WLAN_MBSSID_VAP4_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP4_INTF;
				break;
			case RT_WLAN_MBSSID_VAP5_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP5_INTF;
				break;
			case RT_WLAN_MBSSID_VAP6_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_VAP6_INTF;
				break;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
			case RT_WLAN_MBSSID_MESH_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN0_MESH_INTF;
				break;
#endif
			default:
				*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;
				break; 
		}
	}else if(wlanIdx == RT_WLAN_DEVICE1_INDEX) {
		switch(mbssidIdx) {
			case RT_WLAN_MBSSID_ROOT_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_ROOT_INTF;
				break;
			case RT_WLAN_MBSSID_VAP0_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP0_INTF;
				break;
			case RT_WLAN_MBSSID_VAP1_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP1_INTF;
				break;
			case RT_WLAN_MBSSID_VAP2_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP2_INTF;
				break;
			case RT_WLAN_MBSSID_VAP3_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP3_INTF;
				break;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
			case RT_WLAN_MBSSID_VAP4_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP4_INTF;
				break;
			case RT_WLAN_MBSSID_VAP5_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP5_INTF;
				break;
			case RT_WLAN_MBSSID_VAP6_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_VAP6_INTF;
				break;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
			case RT_WLAN_MBSSID_MESH_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN1_MESH_INTF;
				break;
#endif
			default:
				*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;
				break; 
		}
	}else if(wlanIdx == RT_WLAN_DEVICE2_INDEX) {
		switch(mbssidIdx) {
			case RT_WLAN_MBSSID_ROOT_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_ROOT_INTF;
				break;
			case RT_WLAN_MBSSID_VAP0_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP0_INTF;
				break;
			case RT_WLAN_MBSSID_VAP1_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP1_INTF;
				break;
			case RT_WLAN_MBSSID_VAP2_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP2_INTF;
				break;
			case RT_WLAN_MBSSID_VAP3_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP3_INTF;
				break;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
			case RT_WLAN_MBSSID_VAP4_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP4_INTF;
				break;
			case RT_WLAN_MBSSID_VAP5_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP5_INTF;
				break;
			case RT_WLAN_MBSSID_VAP6_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_VAP6_INTF;
				break;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
			case RT_WLAN_MBSSID_MESH_INTF_INDEX:
				*wlanDevIdx = RTK_FC_WLAN2_MESH_INTF;
				break;
#endif
			default:
				*wlanDevIdx = RTK_FC_WLANX_NOT_FOUND;
				break; 
		}
	}

#endif // CONFIG_COMMON_RT_API

	return SUCCESS;
}

int rtk_fc_wlan_rtmbssidMask2devMask(rt_wlan_mbssid_mask_t *rtWlanMbssidMsk,rtk_fc_wlan_devmask_t *wlanDevIdMask)
{
#if defined(CONFIG_COMMON_RT_API) && defined(CONFIG_RTK_FC_WLAN_HWNAT_ACCELERATION)
	int i;
	rt_wlan_mbssid_index_t mbssid_index=0;
	rtk_fc_wlan_devidx_t wlanDevIdx;
	*wlanDevIdMask =0;

	for(i=0 ;i<RT_WLAN_DEVICE_MAX ;i++)
	{
		for(mbssid_index=0;mbssid_index<RT_WLAN_MBSSID_MAX;mbssid_index++)
		{
			if((1<<mbssid_index) & rtWlanMbssidMsk[i]) {
				rtk_fc_wlan_rtmbssid2devidx(i, mbssid_index, &wlanDevIdx);
				if(wlanDevIdx < RTK_FC_WLANX_END_INTF)
					*wlanDevIdMask |= (1LL<<wlanDevIdx);
				else {
					FCMGR_ERR("translate rt wlan %d ssidmsk 0x%x but ssid %d is failed to mapping to fc dev idx", i, rtWlanMbssidMsk[i], mbssid_index);
				}
			}
		}
	}
#endif // CONFIG_COMMON_RT_API

	return SUCCESS;
}

int rtk_fc_wlan_devMask2RtmbssidMask(rtk_fc_wlan_devmask_t *wlanDevIdMask, rt_wlan_mbssid_mask_t *rtWlanMbssidMsk)
{
#if defined(CONFIG_COMMON_RT_API) && defined(CONFIG_RTK_FC_WLAN_HWNAT_ACCELERATION)
	rtWlanMbssidMsk[0]=0x0;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_ROOT_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_ROOT_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP0_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP0_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP1_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP1_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP2_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP2_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP3_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP3_INTF_BIT;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP4_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP4_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP5_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP5_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_VAP6_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_VAP6_INTF_BIT;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN0_MESH_INTF)
		rtWlanMbssidMsk[0]|=RT_WLAN_MBSSID_MESH_INTF_BIT;
#endif

	rtWlanMbssidMsk[1]=0x0;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_ROOT_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_ROOT_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP0_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP0_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP1_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP1_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP2_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP2_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP3_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP3_INTF_BIT;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP4_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP4_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP5_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP5_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_VAP6_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_VAP6_INTF_BIT;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN1_MESH_INTF)
		rtWlanMbssidMsk[1]|=RT_WLAN_MBSSID_MESH_INTF_BIT;
#endif

	rtWlanMbssidMsk[2]=0x0;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_ROOT_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_ROOT_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP0_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP0_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP1_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP1_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP2_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP2_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP3_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP3_INTF_BIT;
#if defined(CONFIG_WLAN_MBSSID_NUM) && (CONFIG_WLAN_MBSSID_NUM==7)
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP4_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP4_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP5_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP5_INTF_BIT;
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_VAP6_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_VAP6_INTF_BIT;
#endif
#ifdef CONFIG_RTL_MESH_SUPPORT
	if(*wlanDevIdMask&1LL<<RTK_FC_WLAN2_MESH_INTF)
		rtWlanMbssidMsk[2]|=RT_WLAN_MBSSID_MESH_INTF_BIT;
#endif
#endif // CONFIG_COMMON_RT_API

	return SUCCESS;
}

EXPORT_SYMBOL(rtk_fc_wlan_register);
EXPORT_SYMBOL(rtk_fc_wlan_devMask2RtmbssidMask);
EXPORT_SYMBOL(rtk_fc_dev2wlanDevIdx);


