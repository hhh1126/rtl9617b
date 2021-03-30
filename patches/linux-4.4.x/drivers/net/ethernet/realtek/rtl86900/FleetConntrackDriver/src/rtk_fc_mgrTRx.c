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

#include <linux/smp.h>

#include <rtk_fc_mgr.h>
#include <rtk_fc_mgrTRx.h>
#include <rtk_fc_api.h>
#include <rtk_fc_helper_multicast.h>


#if 0 //defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
static uint32 _rtk_fc_wlan_rx_lookup_gmac_decision_by_hash(void)
{
	// round robin algorithm
	static uint32 gmac = 1;

	gmac = gmac?0:1;

	return gmac;
}
#endif
static void _rtk_fc_rx_final_process(fc_rx_info_t *pRxInfo, struct sk_buff *skb)
{
	rtk_fc_nic_rx_t ret;
	struct rt_skbuff rtskb;

	rtk_fc_converter_skb(skb, &rtskb);

	if((ret = rtk_fc_ingress_flowLearning(&rtskb, pRxInfo)) == RTK_FC_NIC_RX_STOP_SKBNOFREERE) {
		// shortcut done
	}else {
		switch(ret) {
			case RTK_FC_NIC_RX_CONTINUE:
			case RTK_FC_NIC_RX_NETIFRX_BY_FC:
#ifdef CONFIG_RTL_ETH_RECYCLED_SKB
				if(rtskb.skb->recyclable) {skb=recycle_skb_swap(skb);if(skb==NULL)return;}
				else if(skb->recyclable){skb=rtskb.skb;}//swap at rtk_fc_ingress_flowLearning, renew skb pointer!
#endif
				rtk_fc_skb_eth_type_trans(skb, skb->dev);
				rtk_fc_skb_netif_rx(skb);
				break;
			case RTK_FC_NIC_RX_STOP:
				dev_kfree_skb_any(skb);
				break;
			//case RTK_FC_NIC_RX_STOP_SKBNOFREERE:
				// done
				// break;
			case RTK_FC_NIC_RX_CON_NO_ETHTYPE_TRANS:
#ifdef CONFIG_RTL_ETH_RECYCLED_SKB
				if(rtskb.skb->recyclable) {skb=recycle_skb_swap(skb);if(skb==NULL)return;}
				else if(skb->recyclable){skb=rtskb.skb;}//swap at rtk_fc_ingress_flowLearning, renew skb pointer!
#endif
				rtk_fc_skb_netif_rx(skb);
				break;

			default:
				FCMGR_ERR("ingress learning ret = %d", ret);
				break;
		}
	}
}
// wifi rx hardware lookup
static int rtk_fc_wlan_rx_lookup(struct sk_buff *skb, unsigned int wlan_dev_idx)
{
	/*
	 * IPI disabled or IPI execute function
	 */
	int ret = SUCCESS;
	rtk_fc_pmap_t *pPmap = &fc_mgr_db.wlanDevMap[wlan_dev_idx].portmap;
	
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
	{
		fc_tx_info_t txInfo;
		fc_tx_info_t *ptxInfo = &txInfo;

		memset(&txInfo, 0, sizeof(txInfo));

		TXINFO_STAG_AWARE(ptxInfo)=1;
		TXINFO_EXTSPA(ptxInfo) = pPmap->macExtPortIdx;

		if(pPmap->macPortIdx == RTK_FC_MAC_PORT_MAINCPU) {
			//txInfo.tx_gmac_id = 0;
#if 0 //defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
			TXINFO_GMAC_ID(ptxInfo) = _rtk_fc_wlan_rx_lookup_gmac_decision_by_hash();
#endif
		}
#if defined(CONFIG_FC_RTL9607C_SERIES) || defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
		else if(pPmap->macPortIdx == RTK_FC_MAC_PORT_MASTERCPU_CORE1) {
			TXINFO_GMAC_ID(ptxInfo)	 = 1;
		}
		else if(pPmap->macPortIdx == RTK_FC_MAC_PORT_SLAVECPU) {
			TXINFO_GMAC_ID(ptxInfo)	 = 2;
		}
#endif
		// smp_nicTx_info.txInfo is ready.
		
#if defined(CONFIG_RTL8686NIC)
		ret = re8686_send_with_txInfo(skb, ptxInfo, (FC_NIC_TX_PRI_TO_RING >> (TXINFO_CPUTAG_PRI(ptxInfo)<<2)) & 0xf);
#endif
		
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+TXINFO_GMAC_ID(ptxInfo)].smp_static[smp_processor_id()]);
	}
#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
	{
	
		ca_ni_tx_config_t tx_config;
		ni_tx_core_config_t *pTx_core_config = &tx_config.core_config;

		memset(&tx_config, 0, sizeof(tx_config));
		
		if(fc_mgr_db.wlanDevMap[wlan_dev_idx].shareCpuPort) {
			pTx_core_config->bf.flow_id_set = TRUE;
			tx_config.flow_id = fc_mgr_db.wlanDevMap[wlan_dev_idx].wlandevidx;
		}
		tx_config.bypass_lso = TRUE;					// bypass dmalso and header_a if possible.

		FCMGR_PRK("wlan use cpu[0x%x] extport[%d] hwlookup flow_id %s [%d]", pPmap->macPortIdx, pPmap->macExtPortIdx, pTx_core_config->bf.flow_id_set?"set":"ignore", tx_config.flow_id);

		pTx_core_config->bf.ldpid = AAL_LPORT_L3_LAN;
		pTx_core_config->bf.lspid = pPmap->macPortIdx;
		pTx_core_config->bf.is_from_ca_tx = TRUE;
		pTx_core_config->bf.bypass_fwd_engine =  (pTx_core_config->bf.flow_id_set ? TRUE : FALSE);
		FCMGR_PRK("CA NI hwlookup to %s port[%d]", skb->dev?skb->dev->name:"NULL", AAL_LPORT_L3_LAN);

		
		//ca_ni_start_xmit_native(skb, skb->dev, &tx_config);
		ret = nic_egress_start_xmit(skb, skb->dev, &tx_config);
		
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC2_TX+RTK_FC_MACPORT_TO_GMAC(tx_config.core_config.bf.lspid)].smp_static[smp_processor_id()]);

	}
#endif

	return ret;
}


#if defined(CONFIG_SMP)

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)

__IRAM_FC_NICTRX
int rtk_fc_smp_nic_rx_dispatch(rtk_fc_smp_nicRx_private_t *smp_nicRx_info)
{
	int cpu = 0;
	rtk_fc_nicrx_ipi_ctrl_t *smpnicrx = NULL;
	rtk_fc_smp_nicRx_work_info_t *pWorkinfo=NULL;
	rtk_fc_smp_nicRx_work_t *pFirstOne=NULL;
	fc_rx_info_t *pRxInfo = &smp_nicRx_info->rxInfo;
	bool highpri = (RXINFO_INT_PRI(pRxInfo)>=FC_NIC_RX_PRI_TO_HI_QUEUE) ? TRUE : FALSE;
	int freeidx = 0;
	rtk_fc_smp_buffer_state_t bufstate; 

	//int i;
	//int cpu_index=0;
		
		
	if(fc_mgr_db.fc_rps_maps.len!=0)
	{
		if(fc_mgr_db.fc_rps_maps.mode == RTK_FC_RPS_DISPATCH_MODE_ORIGINAL)
		{
#if defined(CONFIG_RTK_L34_G3_PLATFORM)
		
			u32 hash=0;
		
			rtk_fc_skb_eth_type_trans(smp_nicRx_info->skb, smp_nicRx_info->skb->dev); //important, need skb->protocol before get skb->hash
		
			skb_reset_network_header(smp_nicRx_info->skb);

			hash = skb_get_hash(smp_nicRx_info->skb);
		
			cpu = fc_mgr_db.fc_rps_maps.cpus[reciprocal_scale(hash, fc_mgr_db.fc_rps_maps.len)];
		
			FCMGR_PRK("[FC RPS]Use cpu = %d, hash = %d\n", cpu, hash);
			/*
				In eth_type_trans(), skb->data's position will be start from iphdr, instead of from eth hdr.
				So we need to reset,
			*/
			
			smp_nicRx_info->skb->data -= (ETH_HLEN);
			smp_nicRx_info->skb->len += (ETH_HLEN);

			//cpu = rtk_fc_smp_nic_rx_rps_get_cpu(smp_nicRx_info->skb, &smp_nicRx_info->rxInfo);
#elif defined(CONFIG_RTK_L34_XPON_PLATFORM)
			if(likely(RXINFO_FBI(pRxInfo))){
		
				cpu =  fc_mgr_db.fc_rps_maps.cpus[( (RXINFO_FB_HASH(pRxInfo)) & 0xf)];
				//FCMGR_PRK("[FC RPS]Use cpu = %d, hash = %d\n", cpu, hash);
			}
			else
			{
				if(highpri)
					cpu = fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_HIGHPRI_NIC_RX].smp_id;	
				else
					cpu = fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].smp_id;
				FCMGR_PRK("Get Hash Failed! Use ipi default cpu.\n");
			}
#endif
		}
		else if(fc_mgr_db.fc_rps_maps.mode == RTK_FC_RPS_DISPATCH_MODE_PORT)
		{		
			//printk("RXINFO_SPA(pRxInfo) = %d\n",RXINFO_SPA(pRxInfo));
			if(RXINFO_SPA(pRxInfo)<RTK_FC_MAC_PORT_MAX)
				cpu = fc_mgr_db.fc_rps_maps.port_to_cpu[RXINFO_SPA(pRxInfo)];
			else
				cpu = 2;
		}

		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_RPS_CPU_DISTRIBUTE].smp_static[cpu]);
	}
	else {		
		if(highpri)
			cpu = fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_HIGHPRI_NIC_RX].smp_id;	
		else
			cpu = fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].smp_id;
	}

	if(highpri) {
		//cpu = 3;	// for test, to schedule another idle cpu
		//smpnicrx = &per_cpu(nicrx_hi_ipi, cpu);
		smpnicrx = fc_mgr_db.nicrx_hi_ipi[cpu];
	}else {
		//smpnicrx = &per_cpu(nicrx_ipi, cpu);
		smpnicrx = fc_mgr_db.nicrx_ipi[cpu];
	}
	
RESCHED:
	bufstate = atomic_read(&smpnicrx->bufstate);

	if((bufstate == SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME) || (bufstate == SMP_BUF_PRIV_AVAILABLE_GLOBAL_CONSUME)) {

		freeidx = atomic_read(&smpnicrx->priv_free_idx);
		pWorkinfo = &smpnicrx->priv_work[freeidx];
		
	}else {
		//SMP_BUF_PRIV_FULL_PRIV_CONSUME
		dev_kfree_skb_any(smp_nicRx_info->skb);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_IPI_TO_FC_RX_DROP].smp_static[smp_processor_id()]);
		goto DISPATCH;
	}
	

	if(bufstate == SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME) {

		if(unlikely(atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_SCHED) )
		{
		
			//FCMGR_ERR("change mode0 to mode1, cur free idx = %d", freeidx);
			//no free priv buf, change sate to use global buf
			//1 buffer state change
			atomic_set(&smpnicrx->bufstate, SMP_BUF_PRIV_FULL_PRIV_CONSUME);
			goto RESCHED;
		}
	}else{
	
		//SMP_BUF_PRIV_FULL_PRIV_CONSUME or SMP_BUF_PRIV_AVAILABLE_GLOBAL_CONSUME
		if(unlikely(atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_SCHED) )
		{
			//no free rx_works, lots of skb are waiting to process
			dev_kfree_skb_any(smp_nicRx_info->skb);
			if(unlikely(fc_mgr_db.smpStatistic))
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_IPI_TO_FC_RX_DROP].smp_static[smp_processor_id()]);
			goto DISPATCH;
		}
	}

	//FCMGR_PRK("buf state[%d] put pkt to %s queue freeidx[%d] target cpu[%d]", bufstate, highpri?"high":"low", freeidx, cpu);


	//Setup information
	memcpy(&(pWorkinfo->smp_nicRx_info), smp_nicRx_info, sizeof(pWorkinfo->smp_nicRx_info));
	
	freeidx += 1;

	if((bufstate == SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME) || (bufstate == SMP_BUF_PRIV_AVAILABLE_GLOBAL_CONSUME)) {
		
		freeidx &= (smpnicrx->priv_work_cnt-1);
		smp_mb(); 
		atomic_set(&smpnicrx->priv_free_idx, freeidx);
		atomic_set(&pWorkinfo->state, SMP_WORK_STATE_SCHED);
		
	}else{
		//SMP_BUF_PRIV_FULL_PRIV_CONSUME
		freeidx &= (MAX_FC_NIC_RX_QUEUE_SIZE-1);
		smp_mb(); 
		atomic_set(&fc_mgr_db.nicrx_free_idx, freeidx);
		atomic_set(&pWorkinfo->state, SMP_WORK_STATE_SCHED);
		
		/*
		 * add to target cpu input queue.
		 */
		list_add_tail(&pFirstOne->rxwork_list, &smpnicrx->input_q);
		smpnicrx->input_q_cnt++;
	}

	
DISPATCH:

	if(cpu_online(cpu) && atomic_dec_and_test(&smpnicrx->csd_available)) {
		
		//FCMGR_PRK("dispatch: buf state[%d] freeidx[%d] target cpu[%d]", bufstate, freeidx, cpu);
	
		if(smp_call_function_single_async(cpu, &smpnicrx->csd) != SUCCESS) {
			FCMGR_PRK("smp call function not ready");
		}

	}
	return RTK_FC_NIC_RX_STOP_SKBNOFREERE;
}

__IRAM_FC_SHORTCUT
void rtk_fc_smp_nic_rx_tasklet(void *info)
{
	rtk_fc_nicrx_ipi_ctrl_t *smpnicrx = info;
	tasklet_hi_schedule(&smpnicrx->tasklet);
}



__IRAM_FC_SHORTCUT
static void rtk_fc_smp_nic_rx_processone(rtk_fc_smp_nicRx_work_info_t *work)
{
	fc_rx_info_t *pRxInfo;
	struct sk_buff *skb;
	
	skb=work->smp_nicRx_info.skb;
	pRxInfo = &work->smp_nicRx_info.rxInfo;

	if(unlikely(fc_mgr_db.smpStatistic))
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_IPI_TO_FC_RX].smp_static[smp_processor_id()]);
	
	_rtk_fc_rx_final_process(pRxInfo, skb);
	
	return;
}

// call_single_data function
__IRAM_FC_SHORTCUT
void rtk_fc_smp_nic_rx_exec(unsigned long data)
{
	// de-queue
	rtk_fc_nicrx_ipi_ctrl_t *smpnicrx = (rtk_fc_nicrx_ipi_ctrl_t *)data;
	unsigned int cnt=MAX_FC_NIC_RX_QUEUE_SIZE<<1;
	rtk_fc_smp_nicRx_work_info_t *pWorkinfo=NULL;
	rtk_fc_smp_nicRx_work_t *pOldestOne = NULL;
	unsigned long int break_jiffies=jiffies+(CONFIG_HZ<<1);
	bool timeout = FALSE;
	rtk_fc_smp_buffer_state_t bufstate; 
	int scheduleidx;


	local_bh_disable();

	
RESCHED:	
	

	do{
		bufstate = atomic_read(&smpnicrx->bufstate);

		if((bufstate == SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME) || (bufstate == SMP_BUF_PRIV_FULL_PRIV_CONSUME)) {
			
			// process priv ring buffer first
			scheduleidx = atomic_read(&smpnicrx->priv_sched_idx);
			pWorkinfo = &smpnicrx->priv_work[scheduleidx];
			
			//FCMGR_PRK("exec: buf state[%d] scheduleidx[%d] work state %d", bufstate, scheduleidx, atomic_read(&pOldestOne->state));

			do{	
				if(unlikely((atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_FREE)))
				{				
					if((bufstate == SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME)) {
						//empty rx queue
						goto END;
					}else if (bufstate == SMP_BUF_PRIV_FULL_PRIV_CONSUME) {

						//FCMGR_ERR("change mode1 to mode2, cur schedidx=%d freeidx=%d", scheduleidx, atomic_read(&smpnicrx->priv_free_idx));
						atomic_set(&smpnicrx->bufstate, SMP_BUF_PRIV_AVAILABLE_GLOBAL_CONSUME);
						
						goto RESCHED;
					}
				}else{

					/*
					 * fc ingress process
					 */
					rtk_fc_smp_nic_rx_processone(pWorkinfo);

					--cnt;
					scheduleidx += 1;
					
					// get next one
					scheduleidx &= (smpnicrx->priv_work_cnt-1);		
					smp_mb(); 
					atomic_set(&smpnicrx->priv_sched_idx, scheduleidx);
					atomic_set(&pWorkinfo->state, SMP_WORK_STATE_FREE);
					smp_mb(); 
					pWorkinfo = &smpnicrx->priv_work[scheduleidx];

				
					if((cnt==0) || (time_is_before_jiffies(break_jiffies))) {
						timeout = TRUE;
					}				
				}
			}while(!timeout);

		}else {
			// SMP_BUF_PRIV_AVAILABLE_GLOBAL_CONSUME
			
			/*
			 * move input queue to process queue
			 */
			spin_lock_irq(&fc_mgr_db.nicrx_lock);
			
			if(likely(smpnicrx->input_q_cnt)) {
				list_splice_tail_init(&smpnicrx->input_q, &smpnicrx->process_q);
				smpnicrx->process_q_cnt += smpnicrx->input_q_cnt;
				smpnicrx->input_q_cnt = 0;
				
				spin_unlock_irq(&fc_mgr_db.nicrx_lock);
			}else { 
				// input queue is empty, this situation happens if high or normal priority packets occupy all shared buffer.
				spin_unlock_irq(&fc_mgr_db.nicrx_lock);
				//FCMGR_ERR("process list but there is no input queue exist!!!!");
				atomic_set(&smpnicrx->bufstate, SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME);
				goto RESCHED;
			}
			
			//FCMGR_ERR("change mode2 to mode0, cur queuing works = %d", smpnicrx->process_q_cnt);
			atomic_set(&smpnicrx->bufstate, SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME);

			// process all works			
			while( smpnicrx->process_q_cnt>0){
				
				pOldestOne=list_first_entry_or_null(&smpnicrx->process_q, rtk_fc_smp_nicRx_work_t, rxwork_list);
				smp_mb(); 
				
				if(pOldestOne == NULL) {
					FCMGR_ERR("process list but entry count %d is not matched!!!!", smpnicrx->process_q_cnt);
					--cnt;
					--smpnicrx->process_q_cnt; 
					continue;
				}
				
				pWorkinfo = &pOldestOne->workinfo;
				
				/*
				 * fc ingress process
				 */
				rtk_fc_smp_nic_rx_processone(pWorkinfo);
								
				list_del(&pOldestOne->rxwork_list);
				// state change must be the last step.
				atomic_set(&pWorkinfo->state, SMP_WORK_STATE_FREE);

				--cnt;
				--smpnicrx->process_q_cnt; 
					
				if((cnt==0) || (time_is_before_jiffies(break_jiffies))) {
					timeout = TRUE;
				}
			}
			
			
		}
		

	} while(!timeout);

END:
	
	if(timeout)
	{		
		rtk_fc_smp_nic_rx_tasklet(smpnicrx);
	}else {		
		atomic_set(&smpnicrx->csd_available, 1);// to allow smp_call_function
	}

	local_bh_enable();

	return;
}

// do init for each cpu, priv_work_cnt and *priv_work should be prepared.
static int rtk_fc_smp_nic_rx_percpu_init(rtk_fc_nicrx_ipi_ctrl_t *smpnicrx)
{
	int i;
	

	//FCMGR_ERR("init cpu[%d] ipi .....", cpu);
	INIT_LIST_HEAD(&smpnicrx->input_q);
	INIT_LIST_HEAD(&smpnicrx->process_q);
	atomic_set(&smpnicrx->csd_available, 1);
	
	smpnicrx->csd.func = rtk_fc_smp_nic_rx_tasklet;
	smpnicrx->csd.info = smpnicrx;
	tasklet_init(&smpnicrx->tasklet, rtk_fc_smp_nic_rx_exec, (unsigned long) smpnicrx);


	atomic_set(&smpnicrx->bufstate, SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME);
	atomic_set(&smpnicrx->priv_free_idx, 0);
	atomic_set(&smpnicrx->priv_sched_idx, 0);
	
	memset(&smpnicrx->priv_work[0], 0, sizeof(rtk_fc_smp_nicRx_work_info_t)*smpnicrx->priv_work_cnt);
	
	for(i=0; i<smpnicrx->priv_work_cnt; i++) {
		atomic_set(&smpnicrx->priv_work[i].state, SMP_WORK_STATE_FREE);
	}

	return SUCCESS;
}



#endif

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_TX)
__IRAM_FC_SHORTCUT
void rtk_fc_smp_nic_tx_tasklet_xps(void *info)
{
	rtk_fc_nictx_ipi_ctrl_t *smpnictx = info;
	tasklet_hi_schedule(&smpnictx->tasklet);
}

__IRAM_FC_NICTRX
void rtk_fc_smp_nic_tx_exec_xps(unsigned long data)
{
	// de-queue
	rtk_fc_nictx_ipi_ctrl_t *smp_data = (rtk_fc_nictx_ipi_ctrl_t *)data;
	rtk_fc_nictx_ipi_ctrl_t *smpnictx = NULL;
	int ret = SUCCESS;
	unsigned int cnt=MAX_FC_NIC_TX_PERCPU_RINGBUF_SIZE;
	rtk_fc_smp_nicTx_work_t *pWorkinfo=NULL;
	unsigned long int break_jiffies=jiffies+(CONFIG_HZ<<1);
	int scheduleidx = 0;
	int caller_cpu = 0;
	bool timeout = FALSE;

	caller_cpu = smp_data->caller_cpu_bit_mask;
	FCMGR_PRK("caller_cpu = %d now_cpu = %d\n",caller_cpu,smp_processor_id());
	if(caller_cpu <0 || caller_cpu>=NR_CPUS)
	{
		FCMGR_PRK("Caller cpu %d error!\n", caller_cpu);
		return;
	}
	//smpnictx = &per_cpu(nictx_ipi, caller_cpu);
	smpnictx = &fc_mgr_db.nictx_perCPU_ipi[caller_cpu];
	if(smpnictx==NULL)
	{
		FCMGR_PRK("NULL POINTER!\n");
		return;
	}
	else
	{
		scheduleidx = atomic_read(&smpnictx->priv_sched_idx);
		pWorkinfo = &smpnictx->priv_work[scheduleidx];
	}
	

	if(unlikely(atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_FREE))
	{
		//empty tx queue
		atomic_set(&smpnictx->csd_available, 1);// to allow smp_call_function
		return;
	}

	local_bh_disable();
	
	do{
#if defined(CONFIG_RTL8686NIC)
		{
			fc_tx_info_t *ptxInfo;
			ptxInfo = &(pWorkinfo->smp_nicTx_info.txInfo);

			ret = re8686_send_with_txInfo(pWorkinfo->smp_nicTx_info.skb, ptxInfo, (FC_NIC_TX_PRI_TO_RING >> (TXINFO_CPUTAG_PRI(ptxInfo)<<2)) & 0xf);
			if(unlikely(fc_mgr_db.smpStatistic))
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+TXINFO_GMAC_ID(ptxInfo)].smp_static[smp_processor_id()]);
		}
#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
		{
			ca_ni_tx_config_t tx_config;

			memset(&tx_config, 0, sizeof(tx_config));
			tx_config.flow_id = pWorkinfo->smp_nicTx_info.flow_id;
#if defined(CONFIG_FC_CA8277B_SERIES)		
			tx_config.dma_aft_l2fib_enable = pWorkinfo->smp_nicTx_info.lso_para0.bf.dma_aft_l2fib_enable;
			tx_config.dma_aft_l2fib_index = pWorkinfo->smp_nicTx_info.lso_para0.bf.dma_aft_l2fib_index;
#endif
			memcpy(&(tx_config.core_config), &(pWorkinfo->smp_nicTx_info.core_config), sizeof(tx_config.core_config));
			
			//ca_ni_start_xmit_native(pOldestOne->smp_nicTx_info.skb, pOldestOne->smp_nicTx_info.dev, &tx_config);
			ret = nic_egress_start_xmit(pWorkinfo->smp_nicTx_info.skb, pWorkinfo->smp_nicTx_info.skb->dev, &tx_config);
			//if(unlikely(fc_mgr_db.smpStatistic))
				//atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+(tx_config.core_config.bf.lspid-RTK_FC_MAC_PORT_CPU)].smp_static[smp_processor_id()]);
		}
#endif
		--cnt;
		scheduleidx += 1;
		scheduleidx &= (smpnictx->priv_work_cnt-1);
		smp_mb(); 
		atomic_set(&pWorkinfo->state, SMP_WORK_STATE_FREE);
		atomic_set(&smpnictx->priv_sched_idx, scheduleidx);
		pWorkinfo = &smpnictx->priv_work[scheduleidx];
		if((cnt==0) || (time_is_before_jiffies(break_jiffies))) {
			timeout = TRUE;
		}	

		if(unlikely(atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_FREE)) {
			pWorkinfo = NULL;
		}

	}while((ret==SUCCESS) && (pWorkinfo) && !timeout);


	local_bh_enable();
	
	if(pWorkinfo || scheduleidx!=atomic_read(&smpnictx->priv_free_idx))
		rtk_fc_smp_nic_tx_tasklet_xps(smpnictx);
	else
		atomic_set(&smpnictx->csd_available, 1);// to allow smp_call_function
	
	return;

}


__IRAM_FC_SHORTCUT
int rtk_fc_smp_nic_tx_dispatch(rtk_fc_smp_nicTx_private_t *smp_nicTx_info)
{
	rtk_fc_smp_nicTx_work_t *pFirstOne=NULL;
	int freeidx = 0;
	
	freeidx = atomic_read(&fc_mgr_db.nictx_free_idx);
	
	pFirstOne = &fc_mgr_db.nictx_work[freeidx];

	if(unlikely(atomic_read(&pFirstOne->state) == SMP_WORK_STATE_SCHED))
	{
		//no free rx_works, lots of skb are waiting to process
		dev_kfree_skb_any(smp_nicTx_info->skb);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_FC_IPI_TO_NIC_TX_DROP].smp_static[smp_processor_id()]);
		goto DISPATCH;
	}else{

		//Setup information
		memcpy(&pFirstOne->smp_nicTx_info, smp_nicTx_info, sizeof(pFirstOne->smp_nicTx_info));
		
		freeidx += 1;
		freeidx &= (MAX_FC_NIC_TX_QUEUE_SIZE-1);
		smp_mb(); 
		
		atomic_set(&fc_mgr_db.nictx_free_idx, freeidx);
		atomic_set(&pFirstOne->state, SMP_WORK_STATE_SCHED);
	}

DISPATCH:
	
	if(cpu_online(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_TX].smp_id) && atomic_dec_and_test(&fc_mgr_db.nictx_ipi.csd_available)) {

		if(smp_call_function_single_async(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_TX].smp_id, &fc_mgr_db.nictx_ipi.csd) != SUCCESS) {
			FCMGR_PRK("smp call function not ready");
		}
	}

	return SUCCESS;
}
__IRAM_FC_SHORTCUT
int rtk_fc_smp_nic_tx_dispatch_xps(rtk_fc_smp_nicTx_private_t *smp_nicTx_info)
{
	//rtk_fc_smp_nicTx_work_t *pFirstOne=NULL;
	rtk_fc_nictx_ipi_ctrl_t *smpnictx = NULL;
	rtk_fc_smp_nicTx_work_t *pWorkinfo=NULL;
	int freeidx = 0;
	int cpu = 0;

	cpu = fc_mgr_db.fc_xps_maps.cpu_map[smp_processor_id()];
	if(unlikely(fc_mgr_db.smpStatistic))
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_XPS_CPU_DISTRIBUTE].smp_static[cpu]);

	FCMGR_PRK("[FC][XPS]cpu %d try to use cpu %d to tx\n",smp_processor_id(),cpu );



	//smpnictx = &per_cpu(nictx_ipi, smp_processor_id());
	smpnictx = &fc_mgr_db.nictx_perCPU_ipi[smp_processor_id()];

	freeidx = atomic_read(&smpnictx->priv_free_idx);
	pWorkinfo = &smpnictx->priv_work[freeidx];

	smpnictx->caller_cpu_bit_mask= -1; // reset
	
	smpnictx->caller_cpu_bit_mask = smp_processor_id();
	
	//freeidx = atomic_read(&fc_mgr_db.nictx_free_idx);
	
	//pFirstOne = &fc_mgr_db.nictx_work[freeidx];

	if(unlikely(atomic_read(&pWorkinfo->state) == SMP_WORK_STATE_SCHED))
	{
		//no free rx_works, lots of skb are waiting to process
		dev_kfree_skb_any(smp_nicTx_info->skb);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_FC_IPI_TO_NIC_TX_DROP].smp_static[smp_processor_id()]);
		goto DISPATCH;
	}else{

		//Setup information
		memcpy(&pWorkinfo->smp_nicTx_info, smp_nicTx_info, sizeof(pWorkinfo->smp_nicTx_info));
		
		freeidx += 1;
		freeidx &= (smpnictx->priv_work_cnt-1);
		smp_mb(); 
		
		atomic_set(&smpnictx->priv_free_idx, freeidx);
		atomic_set(&pWorkinfo->state, SMP_WORK_STATE_SCHED);
	}

DISPATCH:
	
	if(cpu_online(cpu) && atomic_dec_and_test(&smpnictx->csd_available)) {

		if(smp_call_function_single_async(cpu, &smpnictx->csd) != SUCCESS) {
			FCMGR_PRK("smp call function not ready");
		}
	}

	return SUCCESS;
}

__IRAM_FC_NICTRX
void rtk_fc_smp_nic_tx_tasklet(void *info)
{
	tasklet_hi_schedule(&fc_mgr_db.nictx_ipi.tasklet);
}

// call_single_data function
__IRAM_FC_NICTRX
void rtk_fc_smp_nic_tx_exec(unsigned long data)
{
	// de-queue
	int ret = SUCCESS;
	unsigned int cnt=MAX_FC_NIC_TX_QUEUE_SIZE;
	rtk_fc_smp_nicTx_work_t *pOldestOne = NULL;
	unsigned long int break_jiffies=jiffies+(CONFIG_HZ<<1);
	int scheduleidx = 0;
	
	scheduleidx = atomic_read(&fc_mgr_db.nictx_sched_idx);

	pOldestOne = &fc_mgr_db.nictx_work[scheduleidx];

	if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE))
	{
		//empty tx queue
		atomic_set(&fc_mgr_db.nictx_ipi.csd_available, 1);// to allow smp_call_function
		return;
	}

	local_bh_disable();
	
	do{
#if defined(CONFIG_RTL8686NIC)
		{
			fc_tx_info_t *ptxInfo;
			ptxInfo = &(pOldestOne->smp_nicTx_info.txInfo);

			ret = re8686_send_with_txInfo(pOldestOne->smp_nicTx_info.skb, ptxInfo, (FC_NIC_TX_PRI_TO_RING >> (TXINFO_CPUTAG_PRI(ptxInfo)<<2)) & 0xf);
			if(unlikely(fc_mgr_db.smpStatistic))
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+TXINFO_GMAC_ID(ptxInfo)].smp_static[smp_processor_id()]);
		}
#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
		{
			ca_ni_tx_config_t tx_config;

			memset(&tx_config, 0, sizeof(tx_config));
			tx_config.flow_id = pOldestOne->smp_nicTx_info.flow_id;
#if defined(CONFIG_FC_CA8277B_SERIES)		
			tx_config.dma_aft_l2fib_enable = pOldestOne->smp_nicTx_info.lso_para0.bf.dma_aft_l2fib_enable;
			tx_config.dma_aft_l2fib_index = pOldestOne->smp_nicTx_info.lso_para0.bf.dma_aft_l2fib_index;
#endif
			memcpy(&(tx_config.core_config), &(pOldestOne->smp_nicTx_info.core_config), sizeof(tx_config.core_config));
			tx_config.lso_para0.wrd = pOldestOne->smp_nicTx_info.lso_para0.wrd;

			
			//ca_ni_start_xmit_native(pOldestOne->smp_nicTx_info.skb, pOldestOne->smp_nicTx_info.dev, &tx_config);
			ret = nic_egress_start_xmit(pOldestOne->smp_nicTx_info.skb, pOldestOne->smp_nicTx_info.skb->dev, &tx_config);
			if(unlikely(fc_mgr_db.smpStatistic))
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+(tx_config.core_config.bf.lspid-RTK_FC_MAC_PORT_CPU)].smp_static[smp_processor_id()]);
		}
#endif
		
		scheduleidx += 1;
		scheduleidx &= (MAX_FC_NIC_TX_QUEUE_SIZE-1);
		smp_mb(); 
		
		atomic_set(&pOldestOne->state, SMP_WORK_STATE_FREE);
		atomic_set(&fc_mgr_db.nictx_sched_idx, scheduleidx);
		pOldestOne = &fc_mgr_db.nictx_work[scheduleidx];

		if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE)) {
			pOldestOne = NULL;
		}

	}while((ret==SUCCESS) && (pOldestOne) && (cnt-->0) && (time_is_after_jiffies(break_jiffies)));


	local_bh_enable();
	
	if(pOldestOne || scheduleidx!=atomic_read(&fc_mgr_db.nictx_free_idx))
		rtk_fc_smp_nic_tx_tasklet(NULL);
	else
		atomic_set(&fc_mgr_db.nictx_ipi.csd_available, 1);// to allow smp_call_function
	
	return;

}



#endif

#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_RX)
int rtk_fc_smp_wlan_rx_dispatch(rtk_fc_wlanrx_info_t *pWlanRxInfo)
{
	unsigned int wlanRxSrcSmpId = smp_processor_id();
	unsigned int wlanband = fc_mgr_db.wlanDevMap[pWlanRxInfo->wlandevidx].band;
	rtk_fc_smp_wlanRx_work_t *pFirstOne=NULL;
	int freeidx = 0;
	
	local_bh_disable();
	
	freeidx = atomic_read(&fc_mgr_db.wlanrx_free_idx[wlanRxSrcSmpId]);
	
	pFirstOne = &fc_mgr_db.wlanrx_work[wlanRxSrcSmpId][freeidx];

	if(unlikely(atomic_read(&pFirstOne->state) == SMP_WORK_STATE_SCHED))
	{
		//no free rx_works, lots of skb are waiting to process
		dev_kfree_skb_any(pWlanRxInfo->skb);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_IPI_BAND0_HW_LOOKUP_DROP+wlanband].smp_static[smp_processor_id()]);
		goto DISPATCH;
	}else{

		//Setup information
		memcpy(&pFirstOne->smp_wlanRx_info, pWlanRxInfo, sizeof(pFirstOne->smp_wlanRx_info));
		
		freeidx += 1;
		freeidx &= (MAX_FC_WLAN_RX_QUEUE_SIZE-1);
		smp_mb(); 
		
		atomic_set(&fc_mgr_db.wlanrx_free_idx[wlanRxSrcSmpId], freeidx);
		atomic_set(&pFirstOne->state, SMP_WORK_STATE_SCHED);
	}

DISPATCH:
	
	if(cpu_online(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX+wlanband].smp_id) && 
		atomic_dec_and_test(&fc_mgr_db.wlanrx_ipi[wlanRxSrcSmpId].csd_available))
	{

		if(smp_call_function_single_async(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX+wlanband].smp_id, &fc_mgr_db.wlanrx_ipi[wlanRxSrcSmpId].csd) != SUCCESS) {
			FCMGR_PRK("smp call function not ready");
		}
	}
	
	local_bh_enable();

	return RTK_FC_NIC_RX_STOP_SKBNOFREERE;
}

void rtk_fc_smp_wlan_rx_tasklet(void *info)
{
	rtk_fc_smp_ipi_ctrl_t *smpctrl = info;
	tasklet_hi_schedule(&smpctrl->tasklet);
}

// call_single_data function
void rtk_fc_smp_wlan_rx_exec(unsigned long data)
{
	// de-queue
	int ret = SUCCESS;
	unsigned int wlanRxSrcSmpId = data;
	unsigned int cnt=MAX_FC_WLAN_RX_QUEUE_SIZE;
	rtk_fc_smp_wlanRx_work_t *pOldestOne = NULL;
	unsigned long int break_jiffies=jiffies+(CONFIG_HZ<<1);
	int scheduleidx = 0;
	
	
	scheduleidx = atomic_read(&fc_mgr_db.wlanrx_sched_idx[wlanRxSrcSmpId]);

	pOldestOne = &fc_mgr_db.wlanrx_work[wlanRxSrcSmpId][scheduleidx];

	if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE))
	{
		//empty
		atomic_set(&fc_mgr_db.wlanrx_ipi[wlanRxSrcSmpId].csd_available, 1);// to allow smp_call_function
		return;
	}

	local_bh_disable();
	
	do{

		ret = rtk_fc_wlan_rx_lookup(pOldestOne->smp_wlanRx_info.skb, pOldestOne->smp_wlanRx_info.wlandevidx);

		scheduleidx += 1;
		scheduleidx &= (MAX_FC_WLAN_RX_QUEUE_SIZE-1);
		smp_mb();

		atomic_set(&pOldestOne->state, SMP_WORK_STATE_FREE);
		atomic_set(&fc_mgr_db.wlanrx_sched_idx[wlanRxSrcSmpId], scheduleidx);
		pOldestOne = &fc_mgr_db.wlanrx_work[wlanRxSrcSmpId][scheduleidx];

		if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE)) {
			pOldestOne = NULL;
		}

		if(unlikely(fc_mgr_db.smpStatistic && (ret!=SUCCESS)))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_FC_IPI_TO_NIC_TX_DROP].smp_static[smp_processor_id()]);

	}while((ret==SUCCESS) && (pOldestOne) && (cnt-->0) && (time_is_after_jiffies(break_jiffies)));


	local_bh_enable();
	
	if(pOldestOne || scheduleidx!=atomic_read(&fc_mgr_db.wlanrx_free_idx[wlanRxSrcSmpId]))
		rtk_fc_smp_wlan_rx_tasklet(&fc_mgr_db.wlanrx_ipi[wlanRxSrcSmpId]);
	else
		atomic_set(&fc_mgr_db.wlanrx_ipi[wlanRxSrcSmpId].csd_available, 1);// to allow smp_call_function
	
	return;

}

#endif

#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_TX)
__IRAM_FC_NICTRX
int rtk_fc_smp_wlan_tx_dispatch(rtk_fc_wlantx_info_t *pWlanTxInfo)
{
	unsigned int wlanTxSrcSmpId = smp_processor_id();
	unsigned int wlanband = fc_mgr_db.wlanDevMap[pWlanTxInfo->wlandevidx].band;
	rtk_fc_smp_wlanTx_work_t *pFirstOne=NULL;
	int freeidx = 0;

	local_bh_disable();

	freeidx = atomic_read(&fc_mgr_db.wlantx_free_idx[wlanTxSrcSmpId]);
	
	pFirstOne = &fc_mgr_db.wlantx_work[wlanTxSrcSmpId][freeidx];

	if(unlikely(atomic_read(&pFirstOne->state) == SMP_WORK_STATE_SCHED))
	{
		//no free tx_works, lots of skb are waiting to process
		dev_kfree_skb_any(pWlanTxInfo->skb);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_IPI_TO_WIFI_BAND0_TX_DROP+wlanband].smp_static[smp_processor_id()]);
		goto DISPATCH;
	}else{

		//Setup information
		memcpy(&pFirstOne->smp_wlanTx_info, pWlanTxInfo, sizeof(pFirstOne->smp_wlanTx_info));

		freeidx += 1;
		freeidx &= (MAX_FC_WLAN_TX_QUEUE_SIZE-1);
		smp_mb();

		atomic_set(&fc_mgr_db.wlantx_free_idx[wlanTxSrcSmpId], freeidx);
		atomic_set(&pFirstOne->state, SMP_WORK_STATE_SCHED);
	}
	
DISPATCH:

	if(cpu_online(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_TX+wlanband].smp_id) && 
		atomic_dec_and_test(&fc_mgr_db.wlantx_ipi[wlanTxSrcSmpId].csd_available))
	{

		if(smp_call_function_single_async(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_TX+wlanband].smp_id, &fc_mgr_db.wlantx_ipi[wlanTxSrcSmpId].csd) != SUCCESS) {
			FCMGR_PRK("smp call function not ready");
		}
	}

	local_bh_enable();
	
	return RTK_FC_DEVTX_OK;
}

void rtk_fc_smp_wlan_tx_tasklet(void *info)
{
	rtk_fc_smp_ipi_ctrl_t *smpctrl = info;
	tasklet_hi_schedule(&smpctrl->tasklet);
}

// call_single_data function
void rtk_fc_smp_wlan_tx_exec(unsigned long data)
{
	// de-queue
	int ret = SUCCESS;
	unsigned int cnt=MAX_FC_WLAN_TX_QUEUE_SIZE;
	rtk_fc_smp_wlanTx_work_t *pOldestOne = NULL;
	unsigned long int break_jiffies=jiffies+(CONFIG_HZ<<1);
	unsigned int wlanTxSrcSmpId = data;
	int scheduleidx = 0;
	struct sk_buff *skb;
	rtk_fc_wlan_devidx_t wlan_dev_idx;
	unsigned int wlanband;

	scheduleidx = atomic_read(&fc_mgr_db.wlantx_sched_idx[wlanTxSrcSmpId]);

	pOldestOne = &fc_mgr_db.wlantx_work[wlanTxSrcSmpId][scheduleidx];

	if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE))
	{
		//empty
		atomic_set(&fc_mgr_db.wlantx_ipi[wlanTxSrcSmpId].csd_available, 1);// to allow smp_call_function
		return;
	}

	local_bh_disable();
	do{
		skb=pOldestOne->smp_wlanTx_info.skb;
		wlan_dev_idx = pOldestOne->smp_wlanTx_info.wlandevidx;
		wlanband = fc_mgr_db.wlanDevMap[wlan_dev_idx].band;

		//rtl8192cd_start_xmit(skb,dev);
		ret = fc_mgr_db.wlanDevMap[wlan_dev_idx].wlan_native_devops->ndo_start_xmit(pOldestOne->smp_wlanTx_info.skb, pOldestOne->smp_wlanTx_info.skb->dev);

		scheduleidx += 1;
		scheduleidx &= (MAX_FC_WLAN_TX_QUEUE_SIZE-1);
		smp_mb();

		atomic_set(&pOldestOne->state, SMP_WORK_STATE_FREE);
		atomic_set(&fc_mgr_db.wlantx_sched_idx[wlanTxSrcSmpId], scheduleidx);
		pOldestOne = &fc_mgr_db.wlantx_work[wlanTxSrcSmpId][scheduleidx];

		if(unlikely(atomic_read(&pOldestOne->state) == SMP_WORK_STATE_FREE)) {
			pOldestOne = NULL;
		}

		if(unlikely(fc_mgr_db.smpStatistic))
		{
			if(ret==RTK_FC_DEVTX_OK)
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_WIFI_BAND0_TX+wlanband].smp_static[smp_processor_id()]);
			else
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_WIFI_BAND0_TX_DROP+wlanband].smp_static[smp_processor_id()]);
		}

	}while((ret==RTK_FC_DEVTX_OK) && (pOldestOne) && (cnt-->0) && (time_is_after_jiffies(break_jiffies)));

	local_bh_enable();
	
	if(pOldestOne || scheduleidx!=atomic_read(&fc_mgr_db.wlantx_free_idx[wlanTxSrcSmpId]))
		rtk_fc_smp_wlan_tx_tasklet(&fc_mgr_db.wlantx_ipi[wlanband]);
	else
		atomic_set(&fc_mgr_db.wlantx_ipi[wlanTxSrcSmpId].csd_available, 1);// to allow smp_call_function

	return;
}
#endif


int rtk_fc_smp_trx_init(void)
{
	int i, cpu, wlanBufId;
	int nicrx_ipi_cpu, nictx_ipi_cpu, wifirx_ipi_cpu;
	int j;

	i = wlanBufId = cpu = 0;

#if defined(CONFIG_RTK_L34_XPON_PLATFORM)

#if CONFIG_NR_CPUS==2
	nicrx_ipi_cpu = -1;
	nictx_ipi_cpu = 0;
	wifirx_ipi_cpu = 0;
#else
	nicrx_ipi_cpu = 2;
	nictx_ipi_cpu = 0;
	wifirx_ipi_cpu = 0;
#endif

#elif defined(CONFIG_RTK_L34_G3_PLATFORM)

#if CONFIG_NR_CPUS==2
	nicrx_ipi_cpu = -1;
	nictx_ipi_cpu = 0;
	wifirx_ipi_cpu = 1;
#else
	nicrx_ipi_cpu = 2;
	nictx_ipi_cpu = 0;
	wifirx_ipi_cpu = -1;
#endif

#else

	nicrx_ipi_cpu = 0;
	nictx_ipi_cpu = 0;
	wifirx_ipi_cpu = 0;

#endif
	

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)
	/*
	 *	initialization for NIC(-to-FC) Rx dispatch
	 */

	for(i = 0 ; i < CONFIG_NR_CPUS; i++)
	{
		//FCMGR_ERR("init cpu[%d] ipi .....", cpu);
		fc_mgr_db.nicrx_ring[i] = kmalloc(sizeof(rtk_fc_nicrx_ring_ctrl_t), GFP_ATOMIC);
		fc_mgr_db.nicrx_ipi[i] = kmalloc(sizeof(rtk_fc_nicrx_ipi_ctrl_t), GFP_ATOMIC);

		memset(fc_mgr_db.nicrx_ring[i], 0, sizeof(rtk_fc_nicrx_ring_ctrl_t));
		memset(fc_mgr_db.nicrx_ipi[i], 0, sizeof(rtk_fc_nicrx_ipi_ctrl_t));
		
		fc_mgr_db.nicrx_ipi[i]->priv_work_cnt = MAX_FC_NIC_RX_PERCPU_RINGBUF_SIZE;
		fc_mgr_db.nicrx_ipi[i]->priv_work = &fc_mgr_db.nicrx_ring[i]->priv_work[0];
		
		rtk_fc_smp_nic_rx_percpu_init(fc_mgr_db.nicrx_ipi[i]);
	}
	
	for(i = 0 ; i < CONFIG_NR_CPUS; i++)
	{
		// high ring

		//FCMGR_ERR("init cpu[%d] ipi .....", cpu);
		fc_mgr_db.nicrx_hiring[i] = kmalloc(sizeof(rtk_fc_nicrx_hiring_ctrl_t), GFP_ATOMIC);
		fc_mgr_db.nicrx_hi_ipi[i] = kmalloc(sizeof(rtk_fc_nicrx_ipi_ctrl_t), GFP_ATOMIC);


		memset(fc_mgr_db.nicrx_hiring[i], 0, sizeof(rtk_fc_nicrx_hiring_ctrl_t));
		memset(fc_mgr_db.nicrx_hi_ipi[i], 0, sizeof(rtk_fc_nicrx_ipi_ctrl_t));

		fc_mgr_db.nicrx_hi_ipi[i]->priv_work_cnt = MAX_FC_NIC_RX_PERCPU_HIRINGBUF_SIZE;
		fc_mgr_db.nicrx_hi_ipi[i]->priv_work = &fc_mgr_db.nicrx_hiring[i]->priv_work[0];
		
		rtk_fc_smp_nic_rx_percpu_init(fc_mgr_db.nicrx_hi_ipi[i]);
	}


	// receive high priority packet to Linux directly.
	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_HIGHPRI_NIC_RX].mode = RTK_FC_DISPATCH_MODE_NONE;
		
	if(nicrx_ipi_cpu==-1)
	{
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].mode = RTK_FC_DISPATCH_MODE_NONE;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].smp_id = nicrx_ipi_cpu;
	}
	else
	{
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].mode = RTK_FC_DISPATCH_MODE_IPI;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].smp_id = nicrx_ipi_cpu;

	}
	

	atomic_set(&fc_mgr_db.nicrx_free_idx, 0);
	spin_lock_init(&fc_mgr_db.nicrx_lock);

	memset(&fc_mgr_db.nicrx_work[0], 0, sizeof(rtk_fc_smp_nicRx_work_t) * MAX_FC_NIC_RX_QUEUE_SIZE);
	for(i=0; i<MAX_FC_NIC_RX_QUEUE_SIZE; i++) {
		atomic_set(&fc_mgr_db.nicrx_work[i].workinfo.state, SMP_WORK_STATE_FREE);
		
	}
#endif	

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_TX)
	/*
	 *	initialization for (FC-to-)NIC Tx dispatch
	 */
	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_TX].mode = RTK_FC_DISPATCH_MODE_IPI;
	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_TX].smp_id = nictx_ipi_cpu;

	// interrupt irq handler
	fc_mgr_db.nictx_ipi.csd.func = rtk_fc_smp_nic_tx_tasklet;
	tasklet_init(&fc_mgr_db.nictx_ipi.tasklet, rtk_fc_smp_nic_tx_exec, (unsigned long) 0);
	atomic_set(&fc_mgr_db.nictx_ipi.csd_available, 1);
	atomic_set(&fc_mgr_db.nictx_free_idx, 0);
	atomic_set(&fc_mgr_db.nictx_sched_idx, 0);

	memset(&fc_mgr_db.nictx_work[0], 0, sizeof(fc_mgr_db.nictx_work[0]) * MAX_FC_NIC_TX_QUEUE_SIZE);
	for(i=0; i<MAX_FC_NIC_TX_QUEUE_SIZE; i++)
		atomic_set(&fc_mgr_db.nictx_work[i].state, SMP_WORK_STATE_FREE);

	/*
	 *	initialization for NIC(-to-FC) Rx dispatch
	 */

	for(i = 0 ; i < NR_CPUS ; i++)
	{
		fc_mgr_db.nictx_perCPU_ipi[i].priv_work_cnt = MAX_FC_NIC_TX_PERCPU_RINGBUF_SIZE;
		fc_mgr_db.nictx_perCPU_ipi[i].priv_work = &fc_mgr_db.nictx_perCPU_ring[i].priv_work[0];
		
			
		
		//FCMGR_ERR("init cpu[%d] ipi .....", cpu);
		INIT_LIST_HEAD(&fc_mgr_db.nictx_perCPU_ipi[i].input_q);
		INIT_LIST_HEAD(&fc_mgr_db.nictx_perCPU_ipi[i].process_q);
		atomic_set(&fc_mgr_db.nictx_perCPU_ipi[i].csd_available, 1);
		
		fc_mgr_db.nictx_perCPU_ipi[i].csd.func = rtk_fc_smp_nic_tx_tasklet_xps;
		fc_mgr_db.nictx_perCPU_ipi[i].csd.info = &fc_mgr_db.nictx_perCPU_ipi[i];
		tasklet_init(&fc_mgr_db.nictx_perCPU_ipi[i].tasklet, rtk_fc_smp_nic_tx_exec_xps, (unsigned long) &fc_mgr_db.nictx_perCPU_ipi[i]);
	
	
		atomic_set(&fc_mgr_db.nictx_perCPU_ipi[i].bufstate, SMP_BUF_PRIV_AVAILABLE_PRIV_CONSUME);
		atomic_set(&fc_mgr_db.nictx_perCPU_ipi[i].priv_free_idx, 0);
		atomic_set(&fc_mgr_db.nictx_perCPU_ipi[i].priv_sched_idx, 0);
		
		memset(&fc_mgr_db.nictx_perCPU_ipi[i].priv_work[0], 0, sizeof(rtk_fc_smp_nicTx_work_t)*fc_mgr_db.nictx_perCPU_ipi[i].priv_work_cnt);
		
		for(j=0; j<fc_mgr_db.nictx_perCPU_ipi[i].priv_work_cnt; j++) {
			atomic_set(&fc_mgr_db.nictx_perCPU_ipi[i].priv_work[j].state, SMP_WORK_STATE_FREE);
		}

	}
	
#endif


#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_RX)
	/*
	 *	initialization for WLAN Rx dispatch (nic hardware lookup)
	 */

	if(wifirx_ipi_cpu ==-1){
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX].mode = RTK_FC_DISPATCH_MODE_NONE;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN1_RX].mode = RTK_FC_DISPATCH_MODE_NONE;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN2_RX].mode = RTK_FC_DISPATCH_MODE_NONE;

	}else{
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX].mode = RTK_FC_DISPATCH_MODE_IPI;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX].smp_id = wifirx_ipi_cpu;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN1_RX].mode = RTK_FC_DISPATCH_MODE_IPI;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN1_RX].smp_id = wifirx_ipi_cpu;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN2_RX].mode = RTK_FC_DISPATCH_MODE_IPI;
		fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN2_RX].smp_id = wifirx_ipi_cpu;
	}
	for(wlanBufId = 0; wlanBufId < CONFIG_NR_CPUS; wlanBufId++) {
		
		// interrupt irq handler
		fc_mgr_db.wlanrx_ipi[wlanBufId].csd.func = rtk_fc_smp_wlan_rx_tasklet;
		fc_mgr_db.wlanrx_ipi[wlanBufId].csd.info = &fc_mgr_db.wlanrx_ipi[wlanBufId];
		tasklet_init(&fc_mgr_db.wlanrx_ipi[wlanBufId].tasklet, rtk_fc_smp_wlan_rx_exec, (unsigned long) wlanBufId);
		atomic_set(&fc_mgr_db.wlanrx_ipi[wlanBufId].csd_available, 1);
		atomic_set(&fc_mgr_db.wlanrx_free_idx[wlanBufId], 0);
		atomic_set(&fc_mgr_db.wlanrx_sched_idx[wlanBufId], 0);

		memset(&fc_mgr_db.wlanrx_work[wlanBufId][0], 0, sizeof(fc_mgr_db.wlanrx_work[wlanBufId][0]) * MAX_FC_WLAN_RX_QUEUE_SIZE);
		for(i=0; i<MAX_FC_WLAN_RX_QUEUE_SIZE; i++)
			atomic_set(&fc_mgr_db.wlanrx_work[wlanBufId][i].state, SMP_WORK_STATE_FREE);

	}
#endif

#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_TX)

	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_TX].mode = RTK_FC_DISPATCH_MODE_NONE;
	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN1_TX].mode = RTK_FC_DISPATCH_MODE_NONE;
	fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN2_TX].mode = RTK_FC_DISPATCH_MODE_NONE;


	for(wlanBufId  = 0; wlanBufId  < CONFIG_NR_CPUS; wlanBufId++) {

		fc_mgr_db.wlantx_ipi[wlanBufId].csd.func = rtk_fc_smp_wlan_tx_tasklet;
		fc_mgr_db.wlantx_ipi[wlanBufId].csd.info = &fc_mgr_db.wlantx_ipi[wlanBufId];
		tasklet_init(&fc_mgr_db.wlantx_ipi[wlanBufId].tasklet, rtk_fc_smp_wlan_tx_exec, (unsigned long) wlanBufId);
		atomic_set(&fc_mgr_db.wlantx_ipi[wlanBufId].csd_available, 1);
		atomic_set(&fc_mgr_db.wlantx_free_idx[wlanBufId], 0);
		atomic_set(&fc_mgr_db.wlantx_sched_idx[wlanBufId], 0);

		memset(&fc_mgr_db.wlantx_work[wlanBufId][0] , 0, sizeof(fc_mgr_db.wlantx_work[wlanBufId][0]) * MAX_FC_WLAN_TX_QUEUE_SIZE);
		for(i=0; i<MAX_FC_WLAN_TX_QUEUE_SIZE; i++)
			atomic_set(&fc_mgr_db.wlantx_work[wlanBufId][i].state, SMP_WORK_STATE_FREE);
	}

#endif

	fc_mgr_db.fc_rps_maps.mode = RTK_FC_RPS_DISPATCH_MODE_END;
	
	return SUCCESS;
}

#endif
int _rtk_fc_set_skbMark_vlaue(struct sk_buff *skb, rtk_fc_mgr_skbmarkTarget_t target, int32 value)
{
	int startBit, len, mark1or2;
	uint32 mark_value = 0;
			
	startBit = fc_mgr_db.mgr_skbmark[target].startBit;
	len 	 = fc_mgr_db.mgr_skbmark[target].len;
	mark1or2 = fc_mgr_db.mgr_skbmark[target].mark1or2;

	if(mark1or2 == 0) {
		mark_value = (skb->mark>>startBit)&((1<<len)-1);
		skb->mark &= ~(((1<<len)-1) << startBit);
		skb->mark |= (value << startBit);
	}
#if defined(CONFIG_RTK_SKB_MARK2)
	else if(mark1or2 == 1) {
		mark_value = (uint32)(skb->mark2>>startBit)&((1LL<<len)-1);
		
		skb->mark &= ~(((1LL<<len)-1) << (uint32)(skb->mark2>>startBit));
		skb->mark |= ((unsigned long long)value << startBit);
	}
#endif
	
	return SUCCESS;
}
int _rtk_fc_get_skbMark_vlaue(struct sk_buff *skb, rtk_fc_mgr_skbmarkTarget_t target)
{
	int startBit, len, mark1or2;
	uint32 mark_value = 0;
			
	startBit = fc_mgr_db.mgr_skbmark[target].startBit;
	len 	 = fc_mgr_db.mgr_skbmark[target].len;
	mark1or2 = fc_mgr_db.mgr_skbmark[target].mark1or2;

	if(mark1or2 == 0)
		mark_value = (skb->mark>>startBit)&((1<<len)-1);
#if defined(CONFIG_RTK_SKB_MARK2)
	else if(mark1or2 == 1)
		mark_value = (uint32)(skb->mark2>>startBit)&((1LL<<len)-1);
#endif
	
	return mark_value;
}

void _rtk_fc_set_streamId_to_skbMark(struct sk_buff *skb, fc_rx_info_t *pRxInfo)
{
#if !defined(CONFIG_FC_CAG3_SERIES) && !defined(CONFIG_FC_RTL8198F_SERIES)
	/*
	*  If ingressPort is PON port,
	*  then set rx_info's stream id to skb->mark from fc_db.skbmark[RTK_FC_SKBMARK_STREAMID_EN].startBit
	*											   to fc_db.skbmark[RTK_FC_SKBMARK_STREAMID].startBit+len
	*/
	u32 clear_streamId_bit=0;
#if defined(CONFIG_RTK_SKB_MARK2)
	u64 clear_streamId_bit_mark2=0;
#endif

	int streamId_startBit, streamId_len, enableBit_startBit;

	streamId_startBit  = fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].startBit;
	streamId_len       = fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].len;
	enableBit_startBit = fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID_EN].startBit;

	if(enableBit_startBit == RTK_FC_MGR_RMK_UNDEF)
		return;

	
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].mark1or2 == 0){
		uint32 ori_skbmark = skb->mark;
		// step 1: Create a mask to clear the stream id position in skb->mark, e.g. 1111  11 00	0000  0 111  1111  1111  1111  1111  , total 32bits.																			-> | stream id |<-
		clear_streamId_bit = 0xffffffff^((1 << (streamId_startBit + streamId_len)) - 1) ^ ((1 << (streamId_startBit)) - 1);

		// step 2: Clear the bits.
		skb->mark &= clear_streamId_bit;	

		// step 3: Set streamId bits
		skb->mark |= (RXINFO_PON_SID(pRxInfo)<<(streamId_startBit));
		FCMGR_PRK("set streamID %u to skb->mark = 0x%08X (original:0x%08X)", (RXINFO_PON_SID(pRxInfo)), skb->mark, ori_skbmark);
	}
	// step 4: Set Enable bit
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID_EN].mark1or2==0)
	{
		uint32 ori_skbmark = skb->mark;
		skb->mark |= 1 << enableBit_startBit; 			// If enable bit is set on mark1
		FCMGR_PRK("set streamID_en to skb->mark = 0x%08X (original:0x%08X)", skb->mark, ori_skbmark);
	}
		
#if defined(CONFIG_RTK_SKB_MARK2)
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].mark1or2 == 1){
		uint64 ori_skbmark = skb->mark2;
		// step 1: Create a mask to clear the stream id position in skb->mark
		clear_streamId_bit_mark2 = 0xffffffffffffffff^((1LL << (streamId_startBit + streamId_len)) - 1) ^ ((1LL << (streamId_startBit )) - 1);

		// step 2: Clear the bits.

		skb->mark2 &= clear_streamId_bit_mark2;

		// step 3: set stream id.
		skb->mark2 |= ((unsigned long long)(RXINFO_PON_SID(pRxInfo)))<<(streamId_startBit);

		FCMGR_PRK("set streamID %u to skb->mark2 = 0x%016llX (original:0x%016llX)", (RXINFO_PON_SID(pRxInfo)), skb->mark2, ori_skbmark);
	}
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID_EN].mark1or2==1)
	{
		uint64 ori_skbmark = skb->mark2;
		skb->mark2 |= 1LL << enableBit_startBit; 		// If enable bit is set on mark2
		FCMGR_PRK("set streamID_en to skb->mark2 = 0x%016llX (original:0x%016llX)", skb->mark2, ori_skbmark);
	}
#endif

#endif
	return;
}

#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
static int _rtk_fc_skipFcEgressFunc(struct sk_buff *skb, int dpmask)
{
	fc_tx_info_t	*ptxInfo, txInfo;
	uint32 egress_qid = 0, egress_streamIdEn = 0, egress_streamId = 0;
	uint8 ip_summed;

	if(unlikely(fc_mgr_db.smpStatistic)){
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_FROM_PS_RX_SKIP_FC].smp_static[smp_processor_id()]);
	}

	FCMGR_PRK("[TX] SKB[%p] tx to pmask %d ", skb, dpmask);

	/*skipFcEgressFunc:
	- not support NIC vlan offload function
	- not support sw host policing counting
	*/
		
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_QID].startBit != RTK_FC_MGR_RMK_UNDEF)
		egress_qid = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_QID);

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID_EN].startBit != RTK_FC_MGR_RMK_UNDEF)
		egress_streamIdEn = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_STREAMID_EN);

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].startBit != RTK_FC_MGR_RMK_UNDEF)
	{
		egress_streamId = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_STREAMID);
		if(atomic_read(&fc_mgr_db.epon_llid_format) == 1)
		{
			egress_streamId <<= 4;
			egress_streamId |= egress_qid;
			egress_streamId &= ((1 << 7) -1); // stream ID has 7 bits
		}
	}
	
	// nic tx
	ptxInfo = &txInfo;
	memset(ptxInfo, 0, sizeof(fc_tx_info_t));

	TXINFO_EOR(ptxInfo)			= 0;
	TXINFO_FS(ptxInfo) 			= 1;
	TXINFO_LS(ptxInfo) 			= 1;
	rtk_fc_skb_ip_summed_get(skb, &ip_summed);
	if(ip_summed!= CHECKSUM_NONE)
	{
		// CHECKSUM_NONE/Tx: The skb was already checksummed by the protocol, or a checksum is not required.
		TXINFO_IPCS(ptxInfo)			= 1;
		TXINFO_L4CS(ptxInfo)			= 1;
	}
	TXINFO_CRC(ptxInfo)			= 1;
	TXINFO_DATA_LEN(ptxInfo)	= skb->len;
	TXINFO_CPUTAG(ptxInfo) 		= 1;

	TXINFO_SVLAN_ACT(ptxInfo)   = 0;
	TXINFO_CVLAN_ACT(ptxInfo)   = 0;
			
	TXINFO_SVLAN_CFI(ptxInfo)		= 0;
	TXINFO_CVLAN_CFI(ptxInfo)		= 0;
	TXINFO_TX_PORTMSK(ptxInfo)      = dpmask;

	TXINFO_ASPRI(ptxInfo)		= 1;
	TXINFO_CPUTAG_PRI(ptxInfo) 	= egress_qid;
	TXINFO_KEEP(ptxInfo)		= 0;
	TXINFO_DISLRN(ptxInfo) 		= 1;
	if(dpmask & (1<<RTK_FC_MAC_PORT_PON)){
		TXINFO_CPUTAG_PSEL(ptxInfo)		= egress_streamIdEn;
		TXINFO_TX_DST_STREAM_ID(ptxInfo)= egress_streamId;
	}
	TXINFO_L34_KEEP(ptxInfo)		= 1;
	TXINFO_GMAC_ID(ptxInfo)		    = 0;
	TXINFO_EXTSPA(ptxInfo) 			= 0;
	TXINFO_TX_PPPOE_ACT(ptxInfo)    = 0;
	TXINFO_TX_PPPOE_IDX(ptxInfo)	= 0;

	{
#if defined(CONFIG_RTL8686NIC)
		re8686_send_with_txInfo(skb, ptxInfo, (FC_NIC_TX_PRI_TO_RING >> (TXINFO_CPUTAG_PRI(ptxInfo)<<2)) & 0xf);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+TXINFO_GMAC_ID(ptxInfo)].smp_static[smp_processor_id()]);
#endif		

	}
	return SUCCESS;
}

#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
static int _rtk_fc_skipFcEgressFunc(struct sk_buff *skb, int dport)
{
	uint32 egress_qid = 0, egress_streamIdEn = 0, egress_streamId = 0;
	ca_ni_tx_config_t ca_tx_config;
	
	if(unlikely(fc_mgr_db.smpStatistic)){
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_FROM_PS_RX_SKIP_FC].smp_static[smp_processor_id()]);
	}
	
	FCMGR_PRK("[TX] SKB[%p] tx to port %d ", skb, dport);

	/*skipFcEgressFunc:
	- not support NIC vlan offload function
	- not support sw host policing counting
	*/

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_QID].startBit != RTK_FC_MGR_RMK_UNDEF)
		egress_qid = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_QID);

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID_EN].startBit != RTK_FC_MGR_RMK_UNDEF)
		egress_streamIdEn = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_STREAMID_EN);

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_STREAMID].startBit != RTK_FC_MGR_RMK_UNDEF)
	{
		egress_streamId = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_STREAMID);
		if(atomic_read(&fc_mgr_db.epon_llid_format) == 1)
		{
			egress_streamId <<= 4;
			egress_streamId |= egress_qid;
			egress_streamId &= ((1 << 7) -1); // stream ID has 7 bits
		}
	}

	memset(&ca_tx_config, 0, sizeof(ca_tx_config));

#if defined(CONFIG_FC_CAG3_SERIES)
	if((dport == RTK_FC_MAC_PORT_PON) && (egress_streamIdEn == TRUE))
	{
		/* add stag for PON SID */

		uint16 svlanTCI = egress_streamId;
		int off = 0, insertOff = 0;
		u8 *pData = skb->data;
		struct vlan_hdr *pCvlan_hdr = NULL, *pSvlan_hdr = NULL;
		struct ethhdr *eth_hdr = NULL;
		int parsing_fail = FALSE;

		{
			//parsing CVLAN tag
			if(unlikely((skb == NULL) || (skb->data == NULL) || (skb->dev == NULL) ||
				((skb->len - skb->data_len) < ETH_HLEN)/*non linear skbuff len < ETH_HLEN*/))
				parsing_fail = TRUE;

			if(parsing_fail != TRUE)
			{
				off += (ETH_HLEN-2);

				// SVLAN
				if((*(u16*)(pData+off)) == htons(ETH_P_8021AD))
				{
					//already have Stag. Insert Stag fail, keep original
				}
				else
				{
					off+=VLAN_HLEN;

					// CVLAN
					if((*(u16*)(pData+off)) == htons(ETH_P_8021Q))
					{
						pCvlan_hdr = (struct vlan_hdr *)(pData+off+2);
						svlanTCI |= (ntohs(pCvlan_hdr->h_vlan_TCI) & VLAN_PRIO_MASK); //svlan pri = cvlan pri
					}

					insertOff += (ETH_HLEN - 2);
					if(rtk_fc_skb_skb_cow_head(skb, VLAN_HLEN) < 0)
					{
						//"skb head room is not enough. Insert Stag fail, keep original
					}
					else
					{
						rtk_fc_skb_skb_push(skb, VLAN_HLEN, &pData);
						memmove(pData, pData + VLAN_HLEN, insertOff);
						skb->mac_header -= VLAN_HLEN;

						eth_hdr = (struct ethhdr *)pData;
						eth_hdr->h_proto = htons(ETH_P_8021AD);

						pSvlan_hdr = (struct vlan_hdr *)(pData+insertOff+2);
						pSvlan_hdr->h_vlan_TCI = htons(svlanTCI);
					}
				}
			}
		}
	}

#elif !defined(CONFIG_FC_RTL8198F_SERIES)
	
	if((dport == RTK_FC_MAC_PORT_PON) && (egress_streamIdEn == TRUE)) { // PON SID
		// to pon	
		rtk_fc_coreDB_info_t coredb;
		
		coredb.data_sid.index = egress_streamId;
		rtk_fc_coreDBInfo_get(RTK_FC_COREDB_OPS_STREAMID, &coredb);

		if(!coredb.data_sid.entry.valid) {
			dev_kfree_skb_any(skb);
			FCMGR_ERR("invalid sid %d, drop packet.", egress_streamId);
			return SUCCESS;
		}

		ca_tx_config.core_config.bf.ldpid = coredb.data_sid.entry.ldpid;
		ca_tx_config.core_config.bf.txq_index = coredb.data_sid.entry.cos;	
		
		ca_tx_config.core_config.bf.flow_id_set = TRUE;
		ca_tx_config.flow_id = coredb.data_sid.entry.gemid;
		
		ca_tx_config.core_config.bf.lspid = RTK_FC_MAC_PORT_CPU1;	// for mirror function: if enabling pon port(0x7) egress mirror, the ldpid may be 0x20~0x2F
	}else 

#endif
	{
		ca_tx_config.core_config.bf.ldpid = dport;
		ca_tx_config.core_config.bf.txq_index = egress_qid;
		
		ca_tx_config.core_config.bf.lspid = RTK_FC_MAC_PORT_CPU;
	}

	ca_tx_config.core_config.bf.is_from_ca_tx =  TRUE;
	ca_tx_config.core_config.bf.bypass_fwd_engine =  TRUE;
	
	nic_egress_start_xmit(skb, skb->dev, &ca_tx_config);
	
	if(unlikely(fc_mgr_db.smpStatistic))
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+(ca_tx_config.core_config.bf.lspid-RTK_FC_MAC_PORT_CPU)].smp_static[smp_processor_id()]);

	return SUCCESS;
}
#endif

int rtk_fc_trx_init(void)
{
	
	rtk_fc_wlan_init();
	
#if defined(CONFIG_SMP)
	rtk_fc_smp_trx_init();
#endif

	return SUCCESS;

}

__IRAM_FC_NICTRX
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
int rtk_fc_skb_rx(struct re_private *cp, struct sk_buff *skb, struct rx_info *pOriRxInfo)
#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
int rtk_fc_skb_rx(struct napi_struct *napi,struct net_device *dev, struct sk_buff *skb, nic_hook_private_t *nh_priv)
#endif	
{
	fc_rx_info_t *pRxInfo, rxInfo;
	int if_skipFcIngressFunc = FALSE;
	bool highpri = FALSE;
	
#if defined(CONFIG_RTK_L34_G3_PLATFORM)
	HEADER_A_T *hdrA = nh_priv->hdr_a;
	HEADER_CPU_T *hdrCPU = nh_priv->hdr_cpu;
#endif

#if defined(CONFIG_RTK_SOC_RTL8198D)&&defined(CONFIG_RTK_DEV_AP)
	int trapToPs = 0;
#endif

	
	if(unlikely(fc_mgr_db.smpStatistic))
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC_RX].smp_static[smp_processor_id()]);

	pRxInfo = &rxInfo;
	
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)

	/*
	 * rxinfo translation
	 */ 
	pRxInfo->opts2.dw = pOriRxInfo->opts2.dw;
	pRxInfo->opts3.dw = pOriRxInfo->opts3.dw;
	
	// aditional sw fields in rxinfo
	RXINFO_GMAC(pRxInfo)= cp->gmac;

#if defined(CONFIG_FC_WIFI_TRAP_HASH_SUPPORT)
	if(RXINFO_SPA(pRxInfo) == RTK_FC_MAC_PORT_MASTERCPU_CORE1)
		RXINFO_SPA(pRxInfo) = RTK_FC_MAC_PORT_MAINCPU;
	if(RXINFO_GMAC(pRxInfo) == 1)
		RXINFO_GMAC(pRxInfo) = 0;
#endif

	if((RXINFO_REASON(pRxInfo)==CPU_REASON_L34_FWD) 
		|| (RXINFO_REASON(pRxInfo)==CPU_REASON_LUT)	// Hybrid mode support l2 forwarding but no flooding
	) {
#if defined(CONFIG_RTK_SOC_RTL8198D)
		if(RXINFO_REASON(pRxInfo)==CPU_REASON_L34_FWD) 
		{
			if((skb->data[0]&1)==1)
			{
				if(rtk_fc_check_user_group((unsigned char*)skb->data) == 1)
				{
					//trap to ps case : not goto wifi fastforward, so skb dev is required 
					{
						extern struct net_device* decideRxDevice(struct re_private *cp, struct rx_info *pOriRxInfo);
					
						struct net_device *rxdev=NULL;
						if((rxdev=decideRxDevice(cp, pOriRxInfo))!=NULL)
							skb->dev = rxdev;
					}
					trapToPs = 1;
				}
			}
		}
		if(trapToPs == 0)
#endif
		return rtk_fc_fastfwd_directXmit(skb, pRxInfo);
	}


	if(RXINFO_SPA(pRxInfo)==RTK_FC_MAC_PORT_PON)
		_rtk_fc_set_streamId_to_skbMark(skb, pRxInfo);
	
	
	// skb dev is not required if wifi fastforward
	if((RXINFO_REASON(pRxInfo)!=CPU_REASON_L34_FWD)){
		extern struct net_device* decideRxDevice(struct re_private *cp, struct rx_info *pOriRxInfo);

		struct net_device *rxdev=NULL;
		if((rxdev=decideRxDevice(cp, pOriRxInfo))!=NULL)
			skb->dev = rxdev;
	}

#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
	/*
	 * rxinfo translation
	 */ 	
	memset(pRxInfo, 0, sizeof(fc_rx_info_t));

	RXINFO_SPA(pRxInfo)		= hdrA->bits.lspid;
	RXINFO_INT_PRI(pRxInfo) = hdrA->bits.cos;
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
	if(hdrA->bits.lspid == 0x14 || hdrA->bits.lspid == 0x15 || hdrA->bits.lspid == 0x16 || hdrA->bits.lspid == 0x17)
	{
		if(printk_ratelimit())	
			printk(KERN_INFO "[VXLAN] VXLAN Strange packet from lspid %x, drop! \n", hdrA->bits.lspid);
		return RTK_FC_NIC_RX_STOP;
	}
	
#endif

	if(hdrCPU->mdata_raw.mdata_h&RXINFO_REF_VALID_BIT){
		if(hdrCPU->mdata_raw.mdata_h&RXINFO_REF_TRAP_RSN_BIT){
			switch((hdrCPU->mdata_raw.mdata_h&RXINFO_REF_TRAP_RSN_BIT)>>RXINFO_REF_TRAP_RSN_SHIFT_BIT){
				case RXINFO_REF_TRAP_RSN_ACL:
					RXINFO_REASON(pRxInfo) = CPU_REASON_ACL;
					break;
				case RXINFO_REF_TRAP_RSN_FLOWMISS:
					RXINFO_REASON(pRxInfo) = CPU_REASON_FLOWMISS;
					break;
				case RXINFO_REF_TRAP_RSN_UNKNOWN_DA:
					RXINFO_REASON(pRxInfo) = CPU_REASON_UNKNOWN_DA;
					break;
				default:
					break;
			}
		}else if(hdrCPU->mdata_raw.mdata_h&RXINFO_REF_ACL_RSN_BIT){
			RXINFO_REASON(pRxInfo) = CPU_REASON_ACL_NON_TRAP;
		}
		RXINFO_REFIDX(pRxInfo) = hdrCPU->mdata_raw.mdata_h&RXINFO_REF_VALID_BIT;
	}

	// TO WLAN - fast forward, store destination wlan dev ID into RXINFO_DST_EXTPIDX
	if((hdrA->bits.lspid == AAL_LPORT_MC) && (skb->data[0]&0x1) && (hdrA->bits.bits_32_63.pkt_info.pol_id != 0))
	{
		// MC: fwd by l2FE mce + arb table, pol_id stands for wifi dev id
		RXINFO_REASON(pRxInfo) = CPU_REASON_L34_FWD;
#if 0 // wifi FF: ldpid may be 0x10~0x0x13, and no need RXINFO_GMAC info
		RXINFO_GMAC(pRxInfo)= (hdrA->bits.ldpid - RTK_FC_MAC_PORT_WLAN_CPU0);			// to which mac (cpu) port
#endif
		RXINFO_DST_EXTPIDX(pRxInfo) = hdrA->bits.bits_32_63.pkt_info.pol_id; // to which wlan dev index
	}
	else if((hdrCPU->mdata_raw.mdata_l&0xffff) != 0)
	{
		// UC: fwd by L3FE mainhash, matadata stands for wifi flow id s
		RXINFO_REASON(pRxInfo) = CPU_REASON_L34_FWD;
#if 0 // wifi FF: ldpid may be 0x10~0x0x13, and no need RXINFO_GMAC info
		RXINFO_GMAC(pRxInfo)= (hdrA->bits.ldpid - RTK_FC_MAC_PORT_WLAN_CPU0);			// to which mac (cpu) port
#endif
		RXINFO_DST_EXTPIDX(pRxInfo) = (hdrCPU->mdata_raw.mdata_l&0xffff); // to which wlan dev index

	}
	
	if((1 << RXINFO_SPA(pRxInfo)) & RTK_FC_ALL_MAC_WLANCPU_PORTMASK){

		// FROM WLAN - hardware lookup miss or wlan-to-wlan

		RXINFO_CPU_EXTSPA(pRxInfo) = hdrA->bits.bits_32_63.pkt_info.pol_id;			// ext port -> rx_extportspa
	}

	if(RXINFO_REASON(pRxInfo)==CPU_REASON_L34_FWD){
		FCMGR_PRK("[WiFi FF] skb from dev: %s, spa: 0x%x", skb->dev?skb->dev->name:"NULL", RXINFO_SPA(pRxInfo));
		return rtk_fc_fastfwd_directXmit(skb, pRxInfo);
			
	}else if(RXINFO_SPA(pRxInfo) >= RTK_FC_MAC_PORT_WLAN_CPU0){
		FCMGR_PRK("[WiFI Rx] hw lookup miss!! spa: 0x%x wifi_flow_id: %d", RXINFO_SPA(pRxInfo), RXINFO_CPU_EXTSPA(pRxInfo));
		
	}

	else if(RXINFO_SPA(pRxInfo) == RTK_FC_MAC_PORT_PON){
		RXINFO_CPU_EXTSPA(pRxInfo) = hdrA->bits.bits_32_63.pkt_info.pol_id;
		_rtk_fc_set_streamId_to_skbMark(skb, pRxInfo);
	}
#endif


	{
		// SKIP FC
		if(unlikely(fc_mgr_db.hwnat_mode == RT_FLOW_HWNAT_MODE_DIS_ACC)|| unlikely(fc_mgr_db.hwnat_mode==RT_FLOW_HWNAT_MODE_TRAP_ALL))//Trap to ps mode
			if_skipFcIngressFunc = TRUE;
		else if(((1<<RXINFO_SPA(pRxInfo)) & RTK_FC_ALL_MAC_WLANCPU_PORTMASK) 
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
			&& (RXINFO_CPU_EXTSPA(pRxInfo)!=RTK_FC_MAC_EXT_CPU)
#endif
			)
			if_skipFcIngressFunc = FALSE; //wifi HW lookup miss packets. Wifi RX packets need FC driver to decide ingress  device, can not skip FC
		else {
			if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_SKIPFCFUNC].startBit != RTK_FC_MGR_RMK_UNDEF)
				if_skipFcIngressFunc = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_SKIPFCFUNC);
		}
		
		if(if_skipFcIngressFunc)
		{
			/*skip FC ingress: not support sw host policing counting*/
			if(unlikely(fc_mgr_db.smpStatistic)){
				atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC_RX_SKIP_FC].smp_static[smp_processor_id()]);
			}
			
			//((not wifi HW lookup miss packet and skipFcFunc is enabled) OR (disable hwnat_mode),
			FCMGR_PRK("[RX] SKB[%p] from port %d with len %d dev: %s -- continue rx to ps)\n", skb, RXINFO_SPA(pRxInfo), skb->len, skb->dev?skb->dev->name:"NULL");

			return RTK_FC_NIC_RX_CONTINUE;
		}
	}
	
	FCMGR_PRK("[RX] SKB[%p] len %d from dev: %s, spa: 0x%x\n", skb, skb->len, skb->dev?skb->dev->name:"NULL", RXINFO_SPA(pRxInfo));


	highpri = (RXINFO_INT_PRI(pRxInfo)>=FC_NIC_RX_PRI_TO_HI_QUEUE) ? TRUE : FALSE;

#if defined(CONFIG_SMP) && defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)
	if(	(highpri && fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_HIGHPRI_NIC_RX].mode==RTK_FC_DISPATCH_MODE_IPI) ||
		(!highpri && fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_RX].mode==RTK_FC_DISPATCH_MODE_IPI)) {
		rtk_fc_smp_nicRx_private_t smp_nicRx_info;
		smp_nicRx_info.skb = skb;
		memcpy(&(smp_nicRx_info.rxInfo), pRxInfo, sizeof(smp_nicRx_info.rxInfo));
		return rtk_fc_smp_nic_rx_dispatch(&smp_nicRx_info);
	}else
#endif
	{
		_rtk_fc_rx_final_process(pRxInfo, skb);
		return RTK_FC_NIC_RX_STOP_SKBNOFREERE;
	}
	
}

int rtk_fc_skb_tx(struct sk_buff *skb, struct net_device *dev)
{
	//int ret;
	int dpmask = 0;
	struct rt_skbuff rtskb;
	int if_skipFcEgressFunc = FALSE;
#if defined(CONFIG_RTK_L34_G3_PLATFORM)
	ca_eth_private_t *cep = netdev_priv(dev);
	int dport = cep->port_cfg.tx_ldpid;
#endif

	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_SKIPFCFUNC].startBit != RTK_FC_MGR_RMK_UNDEF)
		if_skipFcEgressFunc = _rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_SKIPFCFUNC);

	if(unlikely(fc_mgr_db.hwnat_mode==RT_FLOW_HWNAT_MODE_DIS_ACC) || unlikely(fc_mgr_db.hwnat_mode==RT_FLOW_HWNAT_MODE_TRAP_ALL) )
		if_skipFcEgressFunc = TRUE;
	
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)

	dpmask = ((struct re_dev_private*)dev->priv)->txPortMask;
	if(if_skipFcEgressFunc)
	{
		/*skipFcEgressFunc is TRUE*/
		_rtk_fc_skipFcEgressFunc(skb, dpmask);	
		return 0;
	}

#elif defined(CONFIG_RTK_L34_G3_PLATFORM)

	if((dport == AAL_LPORT_L3_LAN) || (dport == AAL_LPORT_L3_WAN))
	{
		// tx to eth0 or nas0 - dirTx to 0x18/0x19 is forbidden
		dev_kfree_skb_any(skb);
		//printk("dirtx to %d is forbidden! drop packet", dport);
		return 0;	//NETDEV_TX_OK
	}

	if(if_skipFcEgressFunc)
	{
		/*skipFcEgressFunc is TRUE*/
		_rtk_fc_skipFcEgressFunc(skb, dport);
		return 0;
	}
	
	dpmask = (1<<dport);

#endif

	rtk_fc_converter_skb(skb, &rtskb);

	FCMGR_PRK("[TX] SKB[%p] dev %s txpmask 0x%x ", skb, dev->name, dpmask);
		
	return rtk_fc_egress_flowLearning(&rtskb, dev, dpmask, RTK_FC_WLANX_NOT_FOUND);
}

__IRAM_FC_SHORTCUT
int rtk_fc_nic_tx(void *pNicTx_privateInfo_data)
{
	rtk_fc_smp_nicTx_private_t *pNicTx_privateInfo = (rtk_fc_smp_nicTx_private_t *)pNicTx_privateInfo_data;

#if defined(CONFIG_SMP) && defined(CONFIG_RTK_L34_FC_IPI_NIC_TX)
	if(fc_mgr_db.fc_xps_maps.valid == 1 && !(RTK_FC_FORCE_TX_NO_IPI(pNicTx_privateInfo)))
	{
		rtk_fc_smp_nic_tx_dispatch_xps(pNicTx_privateInfo);
	}
	else if((fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_NIC_TX].mode == RTK_FC_DISPATCH_MODE_IPI) && !(RTK_FC_FORCE_TX_NO_IPI(pNicTx_privateInfo)))
	{
		//nic tx dispatch mode (dirtx && ipi enabled)
		rtk_fc_smp_nic_tx_dispatch(pNicTx_privateInfo);
	}
	else
#endif
	{
		//nic tx (ipi disabled)
#if defined(CONFIG_RTL8686NIC)

		fc_tx_info_t *ptxInfo = &(pNicTx_privateInfo->txInfo);

		re8686_send_with_txInfo(pNicTx_privateInfo->skb, ptxInfo, (FC_NIC_TX_PRI_TO_RING >> (TXINFO_CPUTAG_PRI(ptxInfo)<<2)) & 0xf);
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+TXINFO_GMAC_ID(ptxInfo)].smp_static[smp_processor_id()]);

#elif defined(CONFIG_RTK_L34_G3_PLATFORM)

		ca_ni_tx_config_t tx_config;

		memset(&tx_config, 0, sizeof(tx_config));
		tx_config.flow_id = pNicTx_privateInfo->flow_id;
#if defined(CONFIG_FC_CA8277B_SERIES)		
		tx_config.dma_aft_l2fib_enable = pNicTx_privateInfo->lso_para0.bf.dma_aft_l2fib_enable;
		tx_config.dma_aft_l2fib_index = pNicTx_privateInfo->lso_para0.bf.dma_aft_l2fib_index;
#endif
		memcpy(&(tx_config.core_config), &(pNicTx_privateInfo->core_config), sizeof(tx_config.core_config));
		tx_config.lso_para0.wrd = pNicTx_privateInfo->lso_para0.wrd;

		
		//ca_ni_start_xmit_native(pNicTx_privateInfo->skb, pNicTx_privateInfo->dev, &tx_config);
		nic_egress_start_xmit(pNicTx_privateInfo->skb, pNicTx_privateInfo->skb->dev, &tx_config);
		
		if(unlikely(fc_mgr_db.smpStatistic))
			atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_GMAC0_TX+(tx_config.core_config.bf.lspid-RTK_FC_MAC_PORT_CPU)].smp_static[smp_processor_id()]);

#endif
	}
	return SUCCESS;
}

/*
 * The low level rx handling function of fastforward dev.
 * - the major task is doing hardware lookup if uc packet, ingress (mac) learning if mc/bc packet.
 * - user needs to take care skb-<data, data must start from ethernet header.
 * - user needs to handle return value.
 * - NOTICE: recommend using rtk_fc_fastfwd_netif_rx() and rtk_fc_fastfwd_napi_gro_receive() instead of this one.
*/
int fwdEngine_wifi_rx(struct sk_buff *skb)
{
	rtk_fc_wlan_devidx_t wlan_dev_idx = RTK_FC_WLANX_NOT_FOUND;

	if(unlikely(fc_mgr_db.smpStatistic)){
		atomic_inc(&fc_mgr_db.mgr_smp_statistic[FC_MGR_SMP_STATIC_WIFI_RX].smp_static[smp_processor_id()]);
		
		FCMGR_PRK("[RX] SKB[%p] len %d from dev: %s", skb, skb->len, skb->dev ? skb->dev->name : "NULL");
	}


	
	if(unlikely(fc_mgr_db.hwnat_mode == RT_FLOW_HWNAT_MODE_DIS_ACC) || unlikely(fc_mgr_db.hwnat_mode==RT_FLOW_HWNAT_MODE_TRAP_ALL) ){
		// direct rx to ps
		FCMGR_PRK("bypass - continue rx to ps ... dev: %s", ((skb) && (skb->dev)) ? skb->dev->name : "NULL");
		return RTK_FC_NIC_RX_CONTINUE;	//driver needs to do netif_rx()!!
	}
	
	if(fc_mgr_db.mgr_skbmark[FC_MGR_SKBMARK_SKIPFCFUNC].startBit != RTK_FC_MGR_RMK_UNDEF)
	{
		if(_rtk_fc_get_skbMark_vlaue(skb, FC_MGR_SKBMARK_SKIPFCFUNC))
		{
			FCMGR_PRK("skb_mark skipFcFunc is enabled - continue rx to ps ... dev: %s", ((skb) && (skb->dev)) ? skb->dev->name : "NULL");
			return RTK_FC_NIC_RX_CONTINUE;	//driver needs to do netif_rx()!!
		}	
	}

#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
	if(unlikely(fc_mgr_db.wifi_flow_crtl_info.wifi_flow_ctrl_auto_en)){
		if(!strncmp(skb->dev->name, "wlan0", 5)) //packet from wlan0
			fc_mgr_db.wifi_flow_crtl_info.wlan0_accumulate_bit += (((skb->len+4)&0x3fff)<<3);

		else if(!strncmp(skb->dev->name, "wlan1", 5)) //packet from wlan1
			fc_mgr_db.wifi_flow_crtl_info.wlan1_accumulate_bit += (((skb->len+4)&0x3fff)<<3);

	}
#endif

		
	rtk_fc_dev2wlanDevIdx(skb->dev, &wlan_dev_idx);
	
	if(unlikely(wlan_dev_idx >= RTK_FC_WLANX_END_INTF)) {	
		FCMGR_PRK("unknown wlan dev %s - return rx continue to wifi driver ...", ((skb) && (skb->dev)) ? skb->dev->name : "NULL");
		return RTK_FC_NIC_RX_CONTINUE;
	}

	//Use flow-based hw to acclerate unicast and calculate flow hash index when trap
	if(!(skb->data[0]&1)) {
		if(unlikely(fc_mgr_db.skipHwlookUp_stat.status))
		{
			if((fc_mgr_db.wlanDevMap[wlan_dev_idx].portmap.macPortIdx == fc_mgr_db.skipHwlookUp_stat.portInfo.macPortIdx) && (fc_mgr_db.wlanDevMap[wlan_dev_idx].portmap.macExtPortIdx == fc_mgr_db.skipHwlookUp_stat.portInfo.macExtPortIdx))
			{
				FCMGR_PRK("skip hw look up due to skipHwlookUp_stat is ON, port %d(%d)", fc_mgr_db.wlanDevMap[wlan_dev_idx].portmap.macPortIdx, fc_mgr_db.wlanDevMap[wlan_dev_idx].portmap.macExtPortIdx);
				goto FC_INGRESS_LRN;
			}
		}
		// supported wifi dev! do hw lookup
#if defined(CONFIG_SMP)
		if(unlikely(fc_mgr_db.smp_dispatch[RTK_FC_MGR_DISPATCH_WLAN0_RX+fc_mgr_db.wlanDevMap[wlan_dev_idx].band].mode 
			== RTK_FC_DISPATCH_MODE_NONE)) 
		{
			/*
			 * ipi disabled
			 */ 			
			rtk_fc_wlan_rx_lookup(skb, wlan_dev_idx);

		}
#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_RX)
		else
		{
			/*
			 * ipi enabled
			 */ 
			rtk_fc_wlanrx_info_t wlanrxinfo;

			wlanrxinfo.skb = skb;
			wlanrxinfo.wlandevidx = wlan_dev_idx;
			rtk_fc_smp_wlan_rx_dispatch(&wlanrxinfo);
			
		}
#endif

#else
		/*
		 * ipi disabled
		 */ 			
		rtk_fc_wlan_rx_lookup(skb, wlan_dev_idx);
#endif

		return RTK_FC_NIC_RX_STOP_SKBNOFREERE;
	}

FC_INGRESS_LRN:
	/*for the packets from wifi and no need to do HW look up (non-unicast),
	  go to FC ingress_flowLearning (for ingress LUT learning)*/
	{
		struct rt_skbuff rtskb;
		int ret;
		rtk_fc_converter_skb(skb, &rtskb);
		ret = rtk_fc_fastfwd_ingress_rcv(&rtskb);
		if(ret == RTK_FC_NIC_RX_CON_NO_ETHTYPE_TRANS)
			ret = RTK_FC_NIC_RX_CONTINUE; // for wifi RX without HW lookup packets, the ret should not be RTK_FC_NIC_RX_CON_NO_ETHTYPE_TRANS, for Foolproof here
		return ret;
	}

}

/*
 * FastForward Dev Rx Function
 *  - Instead of netif_rx(), fastforward dev call this FC API to process rx packets.
*/
int rtk_fc_fastfwd_netif_rx(struct sk_buff *skb)
{
	int ret = 0, status = NET_RX_DROP;

	skb_push(skb, 14);
	ret = fwdEngine_wifi_rx(skb);

	if(ret==RTK_FC_NIC_RX_CONTINUE)
	{
		skb_pull(skb, 14);
		status = netif_rx (skb);
	}
	else if(ret==RTK_FC_NIC_RX_STOP)
	{
		kfree_skb(skb);
		status = NET_RX_DROP;
	}
	else if(ret==RTK_FC_NIC_RX_STOP_SKBNOFREERE)
	{
		status = NET_RX_SUCCESS;
	}

	return status;
}

/*
 * FastForward Dev Rx Function
 *  - Instead of napi_gro_receive(), fastforward dev call this FC API to process rx packets.
*/
gro_result_t rtk_fc_fastfwd_napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	int ret = 0, status = GRO_DROP;

	skb_push(skb, 14);
	ret = fwdEngine_wifi_rx(skb);

	if(ret==RTK_FC_NIC_RX_CONTINUE)
	{
		skb_pull(skb, 14);
		status = napi_gro_receive(napi, skb);
	}
	else if(ret==RTK_FC_NIC_RX_STOP)
	{
		kfree_skb(skb);
		status = NET_RX_DROP;
	}
	else if(ret==RTK_FC_NIC_RX_STOP_SKBNOFREERE)
	{
		status = NET_RX_SUCCESS;
	}

	return status;
}

/*
 * FastForward Dev Tx Function
 *  - when fastfwd dev register to FC driver, dev tx function will be replaced by rtk_fc_fastfwd_dev_xmit().
*/
int rtk_fc_fastfwd_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rt_skbuff rtskb;

	rtk_fc_converter_skb(skb, &rtskb);

	return rtk_fc_fastfwd_egress_xmit(&rtskb, dev);
}


EXPORT_SYMBOL(rtk_fc_skb_rx);
EXPORT_SYMBOL(rtk_fc_skb_tx);

EXPORT_SYMBOL(fwdEngine_wifi_rx);
EXPORT_SYMBOL(rtk_fc_fastfwd_napi_gro_receive);
EXPORT_SYMBOL(rtk_fc_fastfwd_netif_rx);
EXPORT_SYMBOL(rtk_fc_fastfwd_dev_xmit);


