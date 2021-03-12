#ifndef __RTK_FC_MGRINIT__
#define __RTK_FC_MGRINIT__

#include <rtk_fc_helper.h>
#include <rtk_fc_mgrTRx.h>
#include <rtk_fc_helper_wlan.h>
#if defined(CONFIG_RTL8686NIC)
#include <re8686_rtl9607c.h>
#endif

#if defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
#include <rtk/switch.h>
#include <dal/rtl9607c/dal_rtl9607c_switch.h>
#endif

/****************************************/
/* 			User Configuration				*/
/****************************************/

/*
 * IPI private ring buffer size
 */
#if IS_BUILTIN(CONFIG_RTK_L34_FC_KERNEL_MODULE)
#define FC_NIC_RX_PERCPU_HIRINGBUF_OFFSET	5	// 5: 32
#define FC_NIC_RX_PERCPU_RINGBUF_OFFSET		9	// 9: 512
#else
#define FC_NIC_RX_PERCPU_HIRINGBUF_OFFSET	5	// 5: 32
#define FC_NIC_RX_PERCPU_RINGBUF_OFFSET		8	// 8: 256	/* max size limited by kernel if build module */
#endif

/*
 * IPI global queue size
 */
#define FC_NIC_RX_QUEUE_SIZE_OFFSET		9	// 9: 512
#define FC_NIC_TX_QUEUE_SIZE_OFFSET		10	// 10: 1024
#define FC_WLAN_RX_QUEUE_SIZE_OFFSET		12	// 11: 2048
#define FC_WLAN_TX_QUEUE_SIZE_OFFSET		12	// 11: 2048


/*
 * NIC Rx: dispatch priority packet to high priority buffer
 */
#define FC_NIC_RX_PRI_TO_HI_QUEUE			6	

/*
 * NIC Tx: dispatch priority packet to corresponding ring buffer
 *   bit[3:0] for internal priority 0, 
 *   bit[7:4] for internal priority 1, 
 *   ...
 *   bit[31:28] for internal priority 7
 */
#define FC_NIC_TX_PRI_TO_RING				0x22110000

/*****************************************/


// IPI queue size: must be power of 2

#define MAX_FC_NIC_RX_PERCPU_HIRINGBUF_SIZE	(1<<FC_NIC_RX_PERCPU_HIRINGBUF_OFFSET)
#define MAX_FC_NIC_RX_PERCPU_RINGBUF_SIZE 	(1<<FC_NIC_RX_PERCPU_RINGBUF_OFFSET)
#define MAX_FC_NIC_RX_QUEUE_SIZE 				(1<<FC_NIC_RX_QUEUE_SIZE_OFFSET)
#define MAX_FC_NIC_TX_QUEUE_SIZE 				(1<<FC_NIC_TX_QUEUE_SIZE_OFFSET) 

#define MAX_FC_NIC_TX_PERCPU_HIRINGBUF_SIZE	(1<<4)
#define MAX_FC_NIC_TX_PERCPU_RINGBUF_SIZE 	(1<<10)


#define MAX_FC_WLAN_RX_QUEUE_SIZE 			(1<<FC_WLAN_RX_QUEUE_SIZE_OFFSET) 
#define MAX_FC_WLAN_TX_QUEUE_SIZE 			(1<<FC_WLAN_TX_QUEUE_SIZE_OFFSET)
#if defined(FC_NIC_RX_PRI_TO_HI_QUEUE) && (FC_NIC_RX_PRI_TO_HI_QUEUE>7)
#ERROR_CONFIG
#endif

/*
 *Per cpu ring buffer
 */
typedef struct rtk_fc_nicrx_ring_ctrl_s
{
	rtk_fc_smp_nicRx_work_info_t priv_work[MAX_FC_NIC_RX_PERCPU_RINGBUF_SIZE];
}rtk_fc_nicrx_ring_ctrl_t;

typedef struct rtk_fc_nicrx_hiring_ctrl_s
{
	rtk_fc_smp_nicRx_work_info_t priv_work[MAX_FC_NIC_RX_PERCPU_HIRINGBUF_SIZE];
}rtk_fc_nicrx_hiring_ctrl_t;

typedef struct rtk_fc_nicrx_ipi_ctrl_s
{
	struct call_single_data csd ____cacheline_aligned_in_smp;
	atomic_t csd_available;

	struct tasklet_struct tasklet;
	
	struct list_head input_q;
	struct list_head process_q;
	
	uint16 input_q_cnt;
	uint16 process_q_cnt;

	atomic_t bufstate;		//rtk_fc_smp_buffer_state_t
	
	atomic_t priv_free_idx; 
	atomic_t priv_sched_idx; 

	uint32 priv_work_cnt;
	rtk_fc_smp_nicRx_work_info_t *priv_work;

}rtk_fc_nicrx_ipi_ctrl_t;

typedef struct rtk_fc_nictx_ring_ctrl_s
{
	rtk_fc_smp_nicTx_work_t priv_work[MAX_FC_NIC_TX_QUEUE_SIZE];
}rtk_fc_nictx_ring_ctrl_t;

typedef struct rtk_fc_nictx_hiring_ctrl_s
{
	rtk_fc_smp_nicTx_work_t priv_work[MAX_FC_NIC_TX_PERCPU_HIRINGBUF_SIZE];
}rtk_fc_nictx_hiring_ctrl_t;

typedef struct rtk_fc_nictx_ipi_ctrl_s
{
	struct call_single_data csd ____cacheline_aligned_in_smp;
	atomic_t csd_available;

	struct tasklet_struct tasklet;
	
	struct list_head input_q;
	struct list_head process_q;
	
	uint16 input_q_cnt;
	uint16 process_q_cnt;

	atomic_t bufstate;		//rtk_fc_smp_buffer_state_t
	
	atomic_t priv_free_idx; 
	atomic_t priv_sched_idx; 
	int caller_cpu_bit_mask;
	uint32 priv_work_cnt;
	rtk_fc_smp_nicTx_work_t *priv_work;

}rtk_fc_nictx_ipi_ctrl_t;



typedef struct rtk_fc_mgr_portmask_s
{
	rtk_fc_port_mask_t portmask;
}rtk_fc_mgr_portmask_t;

typedef struct rtk_fc_mgr_database_s
{
	char debug_prk;
	atomic_t epon_llid_format; //0: remarking LLID as SID;  1: remarking LLID merge qid as whole SID
	bool smpStatistic;
	void* mgr_null_pointer;
	rt_flow_hwnat_mode_t hwnat_mode;

#if defined(CONFIG_SMP)
	rtk_fc_mgr_dispatch_mode_t smp_dispatch[RTK_FC_MGR_DISPATCH_ARRAY_SIZE];
#if defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)

	spinlock_t nicrx_lock;			// global buffer
	atomic_t nicrx_free_idx; 
	atomic_t nicrx_sched_idx;
	rtk_fc_smp_nicRx_work_t nicrx_work[MAX_FC_NIC_RX_QUEUE_SIZE];
#endif

#if defined(CONFIG_RTK_L34_FC_IPI_NIC_TX)
	rtk_fc_smp_ipi_ctrl_t nictx_ipi;

	atomic_t nictx_free_idx;
	atomic_t nictx_sched_idx;	
	
	rtk_fc_smp_nicTx_work_t nictx_work[MAX_FC_NIC_TX_QUEUE_SIZE];

	rtk_fc_nictx_ring_ctrl_t nictx_perCPU_ring[CONFIG_NR_CPUS];
	rtk_fc_nictx_ipi_ctrl_t nictx_perCPU_ipi[CONFIG_NR_CPUS];
#endif

#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_RX)
	rtk_fc_smp_ipi_ctrl_t wlanrx_ipi[CONFIG_NR_CPUS];

	atomic_t wlanrx_free_idx[CONFIG_NR_CPUS];
	atomic_t wlanrx_sched_idx[CONFIG_NR_CPUS];
	
	rtk_fc_smp_wlanRx_work_t wlanrx_work[CONFIG_NR_CPUS][MAX_FC_WLAN_RX_QUEUE_SIZE];
#endif

#if defined(CONFIG_RTK_L34_FC_IPI_WIFI_TX)
	rtk_fc_smp_ipi_ctrl_t wlantx_ipi[RTK_FC_WLAN_ID_MAX];
	spinlock_t lock_wlantx[RTK_FC_WLAN_ID_MAX];

	struct list_head wlantx_free[RTK_FC_WLAN_ID_MAX];	// wifi free list
	struct list_head wlantx_queue[RTK_FC_WLAN_ID_MAX];	// wifi tx list

	rtk_fc_smp_wlanTx_work_t wlantx_work[RTK_FC_WLAN_ID_MAX][MAX_FC_WLAN_TX_QUEUE_SIZE];
#endif
	rtk_fc_mgr_xps_map_t fc_xps_maps;
	rtk_fc_mgr_rps_map_t fc_rps_maps;
	unsigned int fc_rps_cpu_bit_mask;
#endif

	rtk_fc_mgr_portmask_t wanPortMask;		// set by func: rtk_fc_wan_portmask_set

	spinlock_t lock_fc;
	spinlock_t lock_rtnetlinkTimer;
	spinlock_t lock_tracefilterShow;
	/*** HELPER API ***/
	rtk_fc_converter_api_t 	converterapi;
	rtk_fc_ps_api_t 		psapi;
	rtk_fc_mgr_api_t		mgrapi;
#if defined(CONFIG_FC_RTL8198F_SERIES)
	rtk_fc_83xx_qos_api_t 	rtk83xxqos;
#endif
	rtk_fc_multi_wan_api_t 	rtk_multi_wan;
	rtk_fc_vlan_api_t 		rtk_vlan;
	rtk_fc_wlan_api_t 		wlanapi;
	rtk_fc_igmpv3_api_t 	rtk_igmpv3;
#if defined(CONFIG_FC_RTL8198F_SERIES)
	rtk_fc_8367r_relay_mc_api_t rtk_8367r_relay_mc;
#endif
#if defined(CONFIG_RTK_SOC_RTL8198D)
	rtk_fc_trap_user_mcast_group_to_ps_api_t rtk_trap_user_mcast_group_to_ps;
#endif

	rtk_fc_rt_helper_api_t  rt_helper_api;
	rtk_fc_init_all_mac_portmask_t rkt_all_mac_portmask;
#if defined(CONFIG_RTK_SOC_RTL8198D) || defined(CONFIG_FC_RTL8198F_SERIES)
	rtk_fc_ipfrag_helper_t 	rtk_ipfrag;
	rtk_fc_tcp_helper_t rtk_fc_tcp_helper;
#endif
	
	/*** WLAN ACCELERATION ***/
	int wlan_first_ifidex;
	rtk_fc_wlan_devmap_t wlanDevMap[RTK_FC_WLANX_END_INTF];
	struct list_head wlanIfidxDevHead[RTK_FC_WLANX_END_INTF];
	struct list_head wlanPortDevHead[RTK_FC_WLAN_PORT_BUCKET_SIZE];

	rtk_fc_mgr_smp_static_t mgr_smp_statistic[FC_MGR_SMP_STATIC_TYPE_MAX];

#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
	rtk_fc_wifi_flow_crtl_info_t wifi_flow_crtl_info;
#endif

#if defined(CONFIG_RTK_FC_PER_SW_FLOW_MIB)
	rt_flow_op_sw_flow_mib_info_t sw_flow_mib[RTK_FC_TABLESIZE_HW_FLOW + RTK_FC_MAX_SHORTCUT_FLOW_SIZE]; // per flow mib
#endif
	rtk_fc_skbmark_t mgr_skbmark[FC_MGR_SKBMARK_END];
	rtk_fc_skipHwlookUp_stat_t skipHwlookUp_stat;	// for share ext port dev, if unknown SA packet do HW lookup directly, FC ingress will not be able to do device recovery. The packets will be drop.

#if defined(CONFIG_FC_RTL9607C_RTL9603CVD_HYBRID)
	uint32 chipId;
	uint8 macport_pon;
	uint8 macport_scpu;
	uint8 macport_mcpu_0;
	uint8 macport_mcpu_1;
	
	uint8 mac10extport_0;
	uint8 mac7extport_0;
#endif

	/*** OTHERS ***/
	char dbgprint_buf[256];
#if defined(CONFIG_RTK_L34_FC_IPI_NIC_RX)

	rtk_fc_nicrx_ring_ctrl_t *nicrx_ring[NR_CPUS];
	rtk_fc_nicrx_ipi_ctrl_t *nicrx_ipi[NR_CPUS];
	
	rtk_fc_nicrx_hiring_ctrl_t *nicrx_hiring[NR_CPUS];
	rtk_fc_nicrx_ipi_ctrl_t *nicrx_hi_ipi[NR_CPUS];

#endif

}rtk_fc_mgr_database_t;


extern rtk_fc_mgr_database_t fc_mgr_db;

#define FCMGR_MOD_NAME	"[FCMGR]"

#define FCMGR_PRK(comment, arg...) \
do {\
	if(unlikely(fc_mgr_db.debug_prk)) { \
		int mt_trace_i;\
		sprintf(fc_mgr_db.dbgprint_buf, comment,## arg);\
		for(mt_trace_i=1;mt_trace_i<256;mt_trace_i++) \
		{ \
		        if(fc_mgr_db.dbgprint_buf[mt_trace_i]==0) \
		        { \
		                if(fc_mgr_db.dbgprint_buf[mt_trace_i-1]=='\n') fc_mgr_db.dbgprint_buf[mt_trace_i-1]=' '; \
		                else break; \
		        } \
		} \
		if(printk_ratelimit()) \
			printk("\033[1;36;40m  "FCMGR_MOD_NAME" %s \033[1;30;40m@%s(%d)\033[0m\n",fc_mgr_db.dbgprint_buf,__FUNCTION__,__LINE__); \
	} \
} while(0)

#define FCMGR_ERR(comment, arg...) \
do {\
	int mt_trace_i;\
	sprintf(fc_mgr_db.dbgprint_buf, comment,## arg);\
	for(mt_trace_i=1;mt_trace_i<256;mt_trace_i++) \
	{ \
	        if(fc_mgr_db.dbgprint_buf[mt_trace_i]==0) \
	        { \
	                if(fc_mgr_db.dbgprint_buf[mt_trace_i-1]=='\n') fc_mgr_db.dbgprint_buf[mt_trace_i-1]=' '; \
	                else break; \
	        } \
	} \
	if(printk_ratelimit()) \
		printk(FCMGR_MOD_NAME" %s @%s(%d)\n",fc_mgr_db.dbgprint_buf,__FUNCTION__,__LINE__); \
} while(0)


/* =================== */
/*		EXPORT_SYMBOL		*/
/* =================== */


/* Function Name:
*	rtk_fc_skb_rx
* Description:
*	NI rx function
* Input:
*	skb		- sk buffer
* Output:
*	N/A
* Return:
*	0	   	- SUCCESS
*	others  	- error code
*/
#if defined(CONFIG_RTK_L34_XPON_PLATFORM)
int rtk_fc_skb_rx(struct re_private *cp, struct sk_buff *skb, struct rx_info *pOriRxInfo);
#elif defined(CONFIG_RTK_L34_G3_PLATFORM)
int rtk_fc_skb_rx(struct napi_struct *napi,struct net_device *dev, struct sk_buff *skb, nic_hook_private_t *nh_priv);
#endif

#if defined(CONFIG_FC_RTL9607C_SERIES)
int rtk_fc_nicHook_rx_skb_NPTv6FastForward(struct re_private *cp, struct sk_buff *skb, struct rx_info *pRxInfo);
#endif

/* Function Name:
*	rtk_fc_skb_tx
* Description:
*	NI tx function
* Input:
*	skb		- sk buffer
*	dev		- net device
* Output:
*	N/A
* Return:
*	0		- SUCCESS
*	others  	- error code
*/
int rtk_fc_skb_tx(struct sk_buff *skb, struct net_device *dev);

/* Function Name:
*	fwdEngine_wifi_rx
* Description:
*	wlan rx function
* Input:
*	skb		- sk buffer
* Output:
*	N/A
* Return:
*	0	   	- SUCCESS
*	others  	- error code
*/
int fwdEngine_wifi_rx(struct sk_buff *skb);


/* Function Name:
*	rtk_fc_fastfwd_netif_rx
* Description:
*	wlan rx function - recomment other dev driver call this one instead of fwdEngine_wifi_rx.
* Input:
*	skb		- sk buffer
* Output:
*	N/A
* Return:
*	0	   	- SUCCESS
*	others  	- error code
*/
int rtk_fc_fastfwd_netif_rx(struct sk_buff *skb);

/* Function Name:
*	rtk_fc_fastfwd_napi_gro_receive
* Description:
*	wlan rx function - recomment other dev driver call this one instead of fwdEngine_wifi_rx.
* Input:
*	napi		- napi struct
*	skb		- sk buffer
* Output:
*	N/A
* Return:
*	0	   	- SUCCESS
*	others  	- error code
*/
gro_result_t rtk_fc_fastfwd_napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb);
	
/* Function Name:
*	rtk_fc_fastfwd_dev_xmit
* Description:
*	wlan tx function
* Input:
*	skb		- sk buffer
*	dev		- net device
* Output:
*	N/A
* Return:
*	0	   	- SUCCESS
*	others  	- error code
*/
int rtk_fc_fastfwd_dev_xmit(struct sk_buff *skb, struct net_device *dev);

#endif
