/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2009 - 2018 Quantenna Communications, Inc.          **
**                                                                           **
**  File        : call_qcsapi.c                                              **
**  Description :                                                            **
**                                                                           **
*******************************************************************************
**                                                                           **
**  Redistribution and use in source and binary forms, with or without       **
**  modification, are permitted provided that the following conditions       **
**  are met:                                                                 **
**  1. Redistributions of source code must retain the above copyright        **
**     notice, this list of conditions and the following disclaimer.         **
**  2. Redistributions in binary form must reproduce the above copyright     **
**     notice, this list of conditions and the following disclaimer in the   **
**     documentation and/or other materials provided with the distribution.  **
**  3. The name of the author may not be used to endorse or promote products **
**     derived from this software without specific prior written permission. **
**                                                                           **
**  Alternatively, this software may be distributed under the terms of the   **
**  GNU General Public License ("GPL") version 2, or (at your option) any    **
**  later version as published by the Free Software Foundation.              **
**                                                                           **
**  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR       **
**  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES**
**  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  **
**  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,         **
**  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT **
**  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,**
**  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    **
**  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      **
**  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF **
**  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.        **
**                                                                           **
*******************************************************************************
EH0*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#include <net80211/ieee80211_qos.h>
#include <net80211/ieee80211_dfs_reentry.h>
#include <net80211/ieee80211_ioctl.h>
#include <net80211/_ieee80211.h>
#include <qtn/qtn_vlan_api.h>
#include <qtn/qtn_bs_comm.h>
#include <qtn/qtn_monitor.h>
#include <qtn/ieee80211_aacs.h>
#include "qcsapi.h"
#include "qcsapi_param.h"
#include <qtn/qtn_nis.h>
#include <qtn/qtn_sis.h>
#include <qtn/qtn_global.h>
#include <qtn_bits.h>
#include <qtn/qtnis.h>
#include "qcsapi_driver.h"
#include "call_qcsapi.h"
#include "qcsapi_sem.h"
#include "qcsapi_util.h"
#include "qcsapi_grabber.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#endif

#define CALL_QCSAPI(_fn) \
	static int (_fn)(const struct call_bundle *cb, struct qcsapi_output *print, \
		const char *interface, int argc, char *argv[])

#ifndef IS_MULTIPLE_BITS_SET
#define IS_MULTIPLE_BITS_SET(_x)	(((unsigned)(_x)) & (((unsigned)(_x)) - 1))
#endif

#define printf		Do_not_use_printf
#define fprintf		Do_not_use_fprintf

#define IP_ADDR_STR_LEN 16
#define BEACON_INTERVAL_WARNING_LOWER_LIMIT	24
#define BEACON_INTERVAL_WARNING_UPPER_LIMIT	100

static int verbose_flag;

const char *qcsapi_ieee80211_reason_str[] = IEEE80211_REASON_STR;

static struct qcsapi_param_name_ent _qcsapi_param_name_counter[] = {
	{qcsapi_total_bytes_sent,		"tx_bytes"},
	{qcsapi_total_bytes_received,		"rx_bytes"},
	{qcsapi_total_packets_sent,		"tx_packets"},
	{qcsapi_total_packets_received,		"rx_packets"},
	{qcsapi_discard_packets_sent,		"tx_discard"},
	{qcsapi_discard_packets_received,	"rx_discard"},
	{qcsapi_error_packets_sent,		"tx_errors"},
	{qcsapi_error_packets_received,		"rx_errors"},
	{qcsapi_fragment_frames_received,	"rx_fragment_pkts"},
	{qcsapi_vlan_frames_received,		"rx_vlan_pkts"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_counter, "Counter");

static struct qcsapi_param_name_ent _qcsapi_param_name_option[] = {
	{qcsapi_channel_refresh,		"channel_refresh"},
	{qcsapi_DFS,				"DFS"},
	{qcsapi_wmm,				"WiFi_MultiMedia"},
	{qcsapi_wmm,				"WMM"},
	{qcsapi_beacon_advertise,		"beacon_advertise"},
	{qcsapi_beacon_advertise,		"beacon"},
	{qcsapi_wifi_radio,			"radio"},
	{qcsapi_autorate_fallback,		"autorate_fallback"},
	{qcsapi_autorate_fallback,		"autorate"},
	{qcsapi_security,			"security"},
	{qcsapi_SSID_broadcast,			"broadcast_SSID"},
	{qcsapi_SSID_broadcast,			"SSID_broadcast"},
	{qcsapi_short_GI,			"shortGI"},
	{qcsapi_short_GI,			"short_GI"},
	{qcsapi_802_11h,			"802_11h"},
	{qcsapi_tpc_query,			"tpc_query"},
	{qcsapi_dfs_fast_channel_switch,	"dfs_fast_switch"},
	{qcsapi_dfs_avoid_dfs_scan,		"avoid_dfs_scan"},
	{qcsapi_uapsd,				"uapsd"},
	{qcsapi_sta_dfs,			"sta_dfs"},
	{qcsapi_specific_scan,			"specific_scan"},
	{qcsapi_GI_probing,			"GI_probing"},
	{qcsapi_GI_fixed,			"GI_fixed"},
	{qcsapi_stbc,				"stbc"},
	{qcsapi_beamforming,			"beamforming"},
	{qcsapi_short_slot,			"short_slot"},
	{qcsapi_short_preamble,			"short_preamble"},
	{qcsapi_rts_cts,			"rts_cts"},
	{qcsapi_40M_only,			"40M_bw_only"},
	{qcsapi_obss_coexist,			"obss_coexist"},
	{qcsapi_11g_protection,			"11g_protection"},
	{qcsapi_11n_protection,			"11n_protection"},
	{qcsapi_qlink,				"qlink"},
	{qcsapi_allow_11b,			"allow_11b"},
	{qcsapi_dyn_beacon_period,		"dyn_beacon_period"},
	{qcsapi_acs_obss_chk,			"acs_obss_chk"},
	{qcsapi_sta_dfs_strict,			"sta_dfs_strict"},
	{qcsapi_bw_resume,			"bw_resume"},
	{qcsapi_subband_radar,			"subband_radar"},
	{qcsapi_priority_repeater,		"priority_repeater"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_option, "Option");

static struct qcsapi_param_name_ent _qcsapi_param_name_board_param[] = {
	{qcsapi_hw_revision,			"hw_revision"},
	{qcsapi_hw_id,				"hw_id"},
	{qcsapi_hw_desc,			"hw_desc"},
	{qcsapi_rf_chipid,			"rf_chipid"},
	{qcsapi_bond_opt,			"bond_opt"},
	{qcsapi_vht,				"vht_status"},
	{qcsapi_bandwidth,			"bw_supported"},
	{qcsapi_spatial_stream,			"spatial_stream"},
	{qcsapi_interface_types,		"interface_types"},
	{qcsapi_rf_chip_verid,			"rf_chip_verid"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_board_param, "Board Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_rate_types[] = {
	{qcsapi_basic_rates,			"basic_rates"},
	{qcsapi_basic_rates,			"basic"},
	{qcsapi_operational_rates,		"operational_rates"},
	{qcsapi_operational_rates,		"operational"},
	{qcsapi_possible_rates,			"possible_rates"},
	{qcsapi_possible_rates,			"possible"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_rate_types, "Rate Type");

static struct qcsapi_param_name_ent _qcsapi_param_name_wifi_std[] = {
	{qcsapi_mimo_ht,			"ht"},
	{qcsapi_mimo_vht,			"vht"},
	{qcsapi_mimo_he,			"he"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_wifi_std, "WiFi Standard");

static struct qcsapi_param_name_ent _qcsapi_param_name_partition[] = {
	{qcsapi_image_linux_live,		"live"},
	{qcsapi_image_linux_safety,		"safety"},
	{qcsapi_image_uboot_live,		"uboot_live"},
	{qcsapi_image_uboot_safety,		"uboot_safety"},
	{qcsapi_image_uboot,			"uboot"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_partition, "Partition");

static struct qcsapi_param_name_ent _qcsapi_param_name_qos_queue[] = {
	{WME_AC_BE,				"BE"},
	{WME_AC_BK,				"BK"},
	{WME_AC_VI,				"VI"},
	{WME_AC_VO,				"VO"},
	{WME_AC_BE,				"besteffort"},
	{WME_AC_BK,				"background"},
	{WME_AC_VI,				"video"},
	{WME_AC_VO,				"voice"}
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_qos_queue, "QoS Queue");

static struct qcsapi_param_name_ent _qcsapi_param_name_vendor_fix[] = {
	{VENDOR_FIX_IDX_BRCM_DHCP,		"brcm_dhcp"},
	{VENDOR_FIX_IDX_BRCM_IGMP,		"brcm_igmp"},
	{VENDOR_FIX_IDX_BRCM_VHT,		"brcm_vht"},
	{VENDOR_FIX_IDX_CC_3SS,			"cc_3ss_snd"},
	{VENDOR_FIX_IDX_CC_4SS,			"cc_4ss_snd"},
	{VENDOR_FIX_IDX_CC_RFGAIN_PPPC,		"cc_rfgain_pppc"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_vendor_fix, "Vendor Fix");

static struct qcsapi_param_name_ent _qcsapi_param_name_qos_param[] = {
	{IEEE80211_WMMPARAMS_CWMIN,		"cwmin"},
	{IEEE80211_WMMPARAMS_CWMAX,		"cwmax"},
	{IEEE80211_WMMPARAMS_AIFS,		"aifs"},
	{IEEE80211_WMMPARAMS_TXOPLIMIT,		"tx_op"},
	{IEEE80211_WMMPARAMS_TXOPLIMIT,		"txoplimit"},
	{IEEE80211_WMMPARAMS_ACM,		"acm"},
	{IEEE80211_WMMPARAMS_NOACKPOLICY,	"noackpolicy"}
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_qos_param, "QoS Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_per_node_param[] = {
	{QCSAPI_LINK_QUALITY,			"link_quality"},
	{QCSAPI_RSSI_DBM,			"rssi_dbm"},
	{QCSAPI_BANDWIDTH,			"bw"},
	{QCSAPI_SNR,				"snr"},
	{QCSAPI_TX_PHY_RATE,			"tx_phy_rate"},
	{QCSAPI_RX_PHY_RATE,			"rx_phy_rate"},
	{QCSAPI_STAD_CCA,			"stand_cca_req"},
	{QCSAPI_RSSI,				"rssi"},
	{QCSAPI_PHY_NOISE,			"hw_noise"},
	{QCSAPI_SOC_MAC_ADDR,			"soc_macaddr"},
	{QCSAPI_SOC_IP_ADDR,			"soc_ipaddr"},
	{QCSAPI_NODE_MEAS_BASIC,		"basic"},
	{QCSAPI_NODE_MEAS_CCA,			"cca"},
	{QCSAPI_NODE_MEAS_RPI,			"rpi"},
	{QCSAPI_NODE_MEAS_CHAN_LOAD,		"channel_load"},
	{QCSAPI_NODE_MEAS_NOISE_HIS,		"noise_histogram"},
	{QCSAPI_NODE_MEAS_BEACON,		"beacon"},
	{QCSAPI_NODE_MEAS_FRAME,		"frame"},
	{QCSAPI_NODE_MEAS_TRAN_STREAM_CAT,	"tran_stream_cat"},
	{QCSAPI_NODE_MEAS_MULTICAST_DIAG,	"multicast_diag"},
	{QCSAPI_NODE_TPC_REP,			"tpc_report"},
	{QCSAPI_NODE_LINK_MEASURE,		"link_measure"},
	{QCSAPI_NODE_NEIGHBOR_REP,		"neighbor_report"},
	{QCSAPI_NODE_SGI_CAPS,			"sgi_caps"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_per_node_param, "Per-node Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_sys_status[] = {
	{qcsapi_sys_status_ethernet,		"Ethernet interface"},
	{qcsapi_sys_status_pcie_ep,		"PCIE EP driver"},
	{qcsapi_sys_status_pcie_rc,		"PCIE RC driver"},
	{qcsapi_sys_status_wifi,		"WiFi driver"},
	{qcsapi_sys_status_rpcd,		"Rpcd server"},
	{qcsapi_sys_status_cal_mode,		"Calstate mode"},
	{qcsapi_sys_status_completed,		"System boot up completely"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_sys_status, "System Status Module");

static struct qcsapi_param_name_ent _qcsapi_param_name_scs_param[] = {
	{SCS_SMPL_DWELL_TIME,			"scs_smpl_dwell_time"},
	{SCS_SAMPLE_INTV,			"scs_sample_intv"},
	{SCS_SAMPLE_TYPE,			"scs_sample_type"},
	{SCS_THRSHLD_SMPL_PKTNUM,		"scs_thrshld_smpl_pktnum"},
	{SCS_THRSHLD_SMPL_AIRTIME,		"scs_thrshld_smpl_airtime"},
	{SCS_THRSHLD_ATTEN_INC,			"scs_thrshld_atten_inc"},
	{SCS_THRSHLD_DFS_REENTRY,		"scs_thrshld_dfs_reentry"},
	{SCS_THRSHLD_DFS_REENTRY_MINRATE,	"scs_thrshld_dfs_reentry_minrate"},
	{SCS_THRSHLD_DFS_REENTRY_INTF,		"scs_thrshld_dfs_reentry_intf"},
	{SCS_THRSHLD_LOADED,			"scs_thrshld_loaded"},
	{SCS_THRSHLD_AGING_NOR,			"scs_thrshld_aging_nor"},
	{SCS_THRSHLD_AGING_DFSREENT,		"scs_thrshld_aging_dfsreent"},
	{SCS_ENABLE,				"scs_enable"},
	{SCS_DEBUG_ENABLE,			"scs_debug_enable"},
	{SCS_SMPL_ENABLE,			"scs_smpl_enable"},
	{SCS_SMPL_ENABLE_ALONE,			"scs_smpl_enable_alone"},
	{SCS_REPORT_ONLY,			"scs_report_only"},
	{SCS_CCA_IDLE_THRSHLD,			"scs_cca_idle_thrshld"},
	{SCS_CCA_INTF_HI_THRSHLD,		"scs_cca_intf_hi_thrshld"},
	{SCS_CCA_INTF_LO_THRSHLD,		"scs_cca_intf_lo_thrshld"},
	{SCS_CCA_INTF_RATIO,			"scs_cca_intf_ratio"},
	{SCS_CCA_INTF_DFS_MARGIN,		"scs_cca_intf_dfs_margin"},
	{SCS_PMBL_ERR_THRSHLD,			"scs_pmbl_err_thrshld"},
	{SCS_CCA_SAMPLE_DUR,			"scs_cca_sample_dur"},
	{SCS_CCA_INTF_SMTH_NOXP,		"scs_cca_intf_smth_fctr"},
	{SCS_CCA_INTF_SMTH_XPED,		"scs_cca_intf_smth_fctr"},
	{SCS_RSSI_SMTH_UP,			"scs_rssi_smth_fctr"},
	{SCS_RSSI_SMTH_DOWN,			"scs_rssi_smth_fctr"},
	{SCS_CHAN_MTRC_MRGN,			"scs_chan_mtrc_mrgn"},
	{SCS_ATTEN_ADJUST,			"scs_atten_adjust"},
	{SCS_ATTEN_SW_ENABLE,			"scs_atten_sw_enable"},
	{SCS_PMBL_ERR_SMTH_FCTR,		"scs_pmbl_err_smth_fctr"},
	{SCS_PMBL_ERR_RANGE,			"scs_pmbl_err_range"},
	{SCS_PMBL_ERR_MAPPED_INTF_RANGE,	"scs_pmbl_err_mapped_intf_range"},
	{SCS_SP_WF,				"scs_sp_wf"},
	{SCS_LP_WF,				"scs_lp_wf"},
	{SCS_PMP_RPT_CCA_SMTH_FCTR,		"scs_pmp_rpt_cca_smth_fctr"},
	{SCS_PMP_RX_TIME_SMTH_FCTR,		"scs_pmp_rx_time_smth_fctr"},
	{SCS_PMP_TX_TIME_SMTH_FCTR,		"scs_pmp_tx_time_smth_fctr"},
	{SCS_PMP_STATS_STABLE_PERCENT,		"scs_pmp_stats_stable_percent"},
	{SCS_PMP_STATS_STABLE_RANGE,		"scs_pmp_stats_stable_range"},
	{SCS_PMP_STATS_CLEAR_INTERVAL,		"scs_pmp_stats_clear_interval"},
	{SCS_AS_RX_TIME_SMTH_FCTR,		"scs_as_rx_time_smth_fctr"},
	{SCS_AS_TX_TIME_SMTH_FCTR,		"scs_as_tx_time_smth_fctr"},
	{SCS_CCA_IDLE_SMTH_FCTR,		"scs_cca_idle_smth_fctr"},
	{SCS_TX_TIME_COMPENSTATION_START,	"scs_tx_time_compensation"},
	{SCS_RX_TIME_COMPENSTATION_START,	"scs_rx_time_compensation"},
	{SCS_TDLS_TIME_COMPENSTATION_START,	"scs_tdls_time_compensation"},
	{SCS_LEAVE_DFS_CHAN_MTRC_MRGN,		"scs_leavedfs_chan_mtrc_mrgn"},
	{SCS_BURST_ENABLE,			"scs_burst_enable"},
	{SCS_BURST_WINDOW,			"scs_burst_window"},
	{SCS_BURST_THRESH,			"scs_burst_thresh"},
	{SCS_BURST_PAUSE_TIME,			"scs_burst_pause_time"},
	{SCS_BURST_FORCE_SWITCH,		"scs_burst_force_switch"},
	{SCS_NAC_MONITOR_MODE,			"scs_nac_monitor_mode"},
	{SCS_CHECK_BAND_MRGN,			"scs_check_band_mrgn"},
	{SCS_OUT_OF_BAND_MRGN,			"scs_out_of_band_mrgn"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_scs_param, "SCS Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_extender_param[] = {
	{qcsapi_extender_role,			"role"},
	{qcsapi_extender_mbs_best_rssi,		"mbs_best_rssi"},
	{qcsapi_extender_rbs_best_rssi,		"rbs_best_rssi"},
	{qcsapi_extender_mbs_wgt,		"mbs_wgt"},
	{qcsapi_extender_rbs_wgt,		"rbs_wgt"},
	{qcsapi_extender_roaming,		"roaming"},
	{qcsapi_extender_bgscan_interval,	"bgscan_interval"},
	{qcsapi_extender_verbose,		"verbose"},
	{qcsapi_extender_mbs_rssi_margin,	"mbs_rssi_margin"},
	{qcsapi_extender_short_retry_limit,	"short_retry"},
	{qcsapi_extender_long_retry_limit,	"long_retry"},
	{qcsapi_extender_scan_mbs_intvl,	"scan_mbs_interval"},
	{qcsapi_extender_scan_mbs_mode,		"scan_mbs_mode"},
	{qcsapi_extender_scan_mbs_expiry,	"scan_mbs_expiry"},
	{qcsapi_extender_fast_cac,		"fast_cac"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_extender_param, "Extender Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_wifi_param[] = {
	{qcsapi_wifi_param_dtim_period,		"dtim_period"},
	{qcsapi_wifi_param_csa_count,		"csa_count"},
	{qcsapi_wifi_param_csa_mode,		"csa_mode"},
	{qcsapi_wifi_param_rrm_nr,		"rrm_nr"},
	{qcsapi_wifi_param_tx_enable,		"tx_enable"},
	{qcsapi_wifi_param_nsm_timeout,		"nsm_timeout"},
	{qcsapi_wifi_param_inact_timeout,	"inact_timeout"},
	{qcsapi_wifi_param_wcac_cfg,		"wcac_cfg"},
	{qcsapi_wifi_param_cfg_4addr,		"cfg_4addr"},
	{qcsapi_wifi_param_repeater_bcn,	"repeater_bcn"},
	{qcsapi_wifi_param_single_rtsthreshold,	"single_rts"},
	{qcsapi_wifi_param_aggr_rtsthreshold,	"aggr_rts"},
	{qcsapi_wifi_param_max_bss_num,		"max_bss_num"},
	{qcsapi_wifi_param_wcac_duration,	"wcac_duration"},
	{qcsapi_wifi_param_wcac_wea_duration,	"wcac_wea_duration"},
	{qcsapi_wifi_param_multiap_backhaul_sta,	"multiap_backhaul_sta"},
	{qcsapi_wifi_param_multiap_backhaul_sta_profile,	"multiap_backhaul_sta_profile"},
	{qcsapi_wifi_param_multi_bssid,		"multi_bssid"},
	{qcsapi_wifi_param_aacs_verbose,	"aacs_verbose"},
	{qcsapi_wifi_param_aacs_excl_tbl,	"aacs_excl_tbl"},
	{qcsapi_wifi_param_aacs_startbw,	"aacs_start_bw"},
	{qcsapi_wifi_param_aacs_startchan,	"aacs_start_chan"},
	{qcsapi_wifi_param_aacs_curr_cfg,	"aacs_current_config"},
	{qcsapi_wifi_param_aacs_avg_dl_phyrate_wgt,	"aacs_dl_phyrate_avg_wgt"},
	{qcsapi_wifi_param_aacs_inc_spf,	"aacs_inc_spf"},
	{qcsapi_wifi_param_aacs_inc_lpf,	"aacs_inc_lpf"},
	{qcsapi_wifi_param_aacs_use_thres_tbl,	"aacs_use_thres_tbl"},
	{qcsapi_wifi_param_aacs_norm_bwctrl,	"aacs_bw_ctrl"},
	{qcsapi_wifi_param_aacs_norm_bwlimit,	"aacs_bw_limit"},
	{qcsapi_wifi_param_aacs_ics_bwctrl,	"aacs_ics_bw_ctrl"},
	{qcsapi_wifi_param_aacs_ics_bwlimit,	"aacs_ics_bw_limit"},
	{qcsapi_wifi_param_aacs_alt_bwctrl,	"aacs_alt_bw_ctrl"},
	{qcsapi_wifi_param_aacs_alt_bwlimit,	"aacs_alt_bw_limit"},
	{qcsapi_wifi_param_aacs_fastmode,	"aacs_fast_mode"},
	{qcsapi_wifi_param_aacs_sw_timebase_sec,	"aacs_sw_timebase_sec"},
	{qcsapi_wifi_param_aacs_sw_timefactor,	"aacs_sw_timefactor"},
	{qcsapi_wifi_param_aacs_sw_max_sec,	"aacs_sw_max_sec"},
	{qcsapi_wifi_param_aacs_sw_reset_sec,	"aacs_sw_reset_sec"},
	{qcsapi_wifi_param_aacs_min_scan_succrate,	"aacs_min_scan_succ_rate"},
	{qcsapi_wifi_param_aacs_dec_max_sec,	"aacs_dec_max_sec"},
	{qcsapi_wifi_param_aacs_apcnt_penalty,	"aacs_apcnt_penalty"},
	{qcsapi_wifi_param_aacs_use_vnode,	"aacs_use_vnode"},
	{qcsapi_wifi_param_aacs_sel_dfs,	"aacs_select_dfs"},
	{qcsapi_wifi_param_aacs_ics_dfs,	"aacs_ics_dfs"},
	{qcsapi_wifi_param_aacs_norm_dfs,	"aacs_norm_dfs"},
	{qcsapi_wifi_param_aacs_alt_dfs,	"aacs_alt_dfs"},
	{qcsapi_wifi_param_aacs_dfs_thres,	"aacs_dfs_thres"},
	{qcsapi_wifi_param_aacs_inc_all_chan,	"aacs_inc_all_chan"},
	{qcsapi_wifi_param_aacs_alt_inc_all_chan,	"aacs_alt_inc_all_chan"},
	{qcsapi_wifi_param_aacs_enable_alt,	"aacs_enable_alt"},
	{qcsapi_wifi_param_aacs_alt_list,	"aacs_alt_list"},
	{qcsapi_wifi_param_aacs_enable_sta_d,	"aacs_enable_sta_d"},
	{qcsapi_wifi_param_aacs_allow_sta_d,	"aacs_allow_sta_d"},
	{qcsapi_wifi_param_aacs_enable_obss,	"aacs_enable_obss"},
	{qcsapi_wifi_param_force_vhtopi_in_heop,	"force_vhtopi_in_heop"},
	{qcsapi_wifi_param_aacs_chan_fade_secs,	"aacs_chan_fade_sec"},
	{qcsapi_wifi_param_aacs_sta_check_type,	"aacs_sta_check_type"},
	{qcsapi_wifi_param_pps_max_bcast,	"pps_max_bcast"},
	{qcsapi_wifi_param_pps_max_ssdp,	"pps_max_ssdp"},
	{qcsapi_wifi_param_dbvc_level,		"dbvc_level"},
	{qcsapi_wifi_param_dbvc_dwell,		"dbvc_dwell"},
	{qcsapi_wifi_param_dbvc_same_ch_switch,	"dbvc_same_ch_sw"},
	{qcsapi_wifi_param_6g_min_rate,		"get_6g_min_rate"},
	{qcsapi_wifi_param_tx_power_cap,	"tx_power_cap"},
	{qcsapi_wifi_param_rfic_internal_temp,	"rfic_int_temp"},
	{qcsapi_wifi_param_aacs_enable_non_psc,	"aacs_enable_non_psc"},
	{qcsapi_wifi_param_bss_bw,		"bss_bw"},
	{qcsapi_wifi_param_hecap,		"hecap"},
	{qcsapi_wifi_param_twt,			"twt"},
	{qcsapi_wifi_param_aacs_init,		"aacs_init"},
	{qcsapi_wifi_param_port_isolate,	"port_isolate"},
	{qcsapi_wifi_param_nfr_level,		"nfr_level"},
	{qcsapi_wifi_param_nfr_dwell,		"nfr_dwell"},
	{qcsapi_wifi_param_nfr_same_ch_switch,	"nfr_same_ch_sw"},
	{qcsapi_wifi_param_eap_vlan_tag,	"eap_vlan_tag"},
	{qcsapi_wifi_param_bridge_mode,		"bridge_mode"},
	{qcsapi_wifi_param_sub_ov_wcac,		"dfs_subband_priority"},
	{qcsapi_wifi_param_aacs_flag_ctrl,      "aacs_flag_ctrl"},
	{qcsapi_wifi_param_dfs_mgmt_wcac_en,	"dfs_mgmt_wcac_en"},
	{qcsapi_wifi_param_dfs_mgmt_zcac_en,	"dfs_mgmt_zcac_en"},
	{qcsapi_wifi_param_dfs_mgmt_ocac_en,	"dfs_mgmt_ocac_en"},
	{qcsapi_wifi_param_reduce_rx_gain,	"reduce_rx_gain"},
	{qcsapi_wifi_param_aacs_mode,		"aacs_mode"},
	{qcsapi_wifi_param_rxchain_ctrl,	"rxchain_ctrl"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_wifi_param, "WiFi Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_eap[] = {
	{qcsapi_eap_param_key_timeout,		"key_timeout"},
	{qcsapi_eap_param_key_retries,		"key_retries"},
	{qcsapi_eap_param_id_req_timeout,	"id_req_timeout"},
	{qcsapi_eap_param_id_req_retries,	"id_req_retries"},
	{qcsapi_eap_param_req_timeout,		"req_timeout"},
	{qcsapi_eap_param_req_retries,		"req_retries"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_eap,	"EAP Parameter");

static struct qcsapi_param_name_ent _qcsapi_param_name_hw_module[] = {
	{qcsapi_hw_pm_signal,			"pm_signal"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_hw_module, "HW module");

static struct qcsapi_param_name_ent _qcsapi_param_name_ssid_fmt[] = {
	{qcsapi_ssid_fmt_str,			"str"},
	{qcsapi_ssid_fmt_hex_str,		"hexstr"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_ssid_fmt, "SSID Format");

static struct qcsapi_param_name_ent _qcsapi_param_name_phyrate[] = {
	{QTN_PHY_RATE_LEGACY_6M, "6M"},
	{QTN_PHY_RATE_LEGACY_9M, "9M"},
	{QTN_PHY_RATE_LEGACY_12M, "12M"},
	{QTN_PHY_RATE_LEGACY_18M, "18M"},
	{QTN_PHY_RATE_LEGACY_24M, "24M"},
	{QTN_PHY_RATE_LEGACY_36M, "36M"},
	{QTN_PHY_RATE_LEGACY_48M, "48M"},
	{QTN_PHY_RATE_LEGACY_54M, "54M"},
	{QTN_PHY_RATE_LEGACY_1M, "1M"},
	{QTN_PHY_RATE_LEGACY_2M, "2M"},
	{QTN_PHY_RATE_LEGACY_5_5M, "5.5M"},
	{QTN_PHY_RATE_LEGACY_11M, "11M"},
	{QTN_BCN_PHY_RATE_HE_MCS0, "HE:MCS0"},
	{QTN_BCN_PHY_RATE_HE_MCS1, "HE:MCS1"},
	{QTN_BCN_PHY_RATE_HE_MCS2, "HE:MCS2"},
	{QTN_BCN_PHY_RATE_HE_MCS3, "HE:MCS3"},
	{QTN_BCN_PHY_RATE_HE_MCS4, "HE:MCS4"},
	{QTN_BCN_PHY_RATE_HE_MCS5, "HE:MCS5"},
	{QTN_BCN_PHY_RATE_HE_MCS6, "HE:MCS6"},
	{QTN_BCN_PHY_RATE_HE_MCS7, "HE:MCS7"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_phyrate, "Beacon PHY Rate");

static struct qcsapi_param_name_ent _qcsapi_param_name_dpp_param[] = {
	{qcsapi_dpp_cmd_get_config,		"cfg_get"},
	{qcsapi_dpp_cmd_set_config,		"cfg_set"},
	{qcsapi_dpp_cmd_qr_code,		"dpp_qr_code"},
	{qcsapi_dpp_cmd_bootstrap_gen,		"dpp_bootstrap_gen"},
	{qcsapi_dpp_cmd_bootstrap_get_uri,	"dpp_bootstrap_get_uri"},
	{qcsapi_dpp_cmd_auth_init,		"dpp_auth_init"},
	{qcsapi_dpp_cmd_listen,			"dpp_listen"},
	{qcsapi_dpp_cmd_stop_listen,		"dpp_stop_listen"},
	{qcsapi_dpp_cmd_configurator_add,	"dpp_configurator_add"},
	{qcsapi_dpp_cmd_configurator_remove,	"dpp_configurator_remove"},
	{qcsapi_dpp_cmd_configurator_sign,	"dpp_configurator_sign"},
	{qcsapi_dpp_cmd_pkex_add,		"dpp_pkex_add"},
	{qcsapi_dpp_cmd_pkex_remove,		"dpp_pkex_remove"},
	{qcsapi_dpp_cmd_configurator_params,	"dpp_configurator_params"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_dpp_param, "DPP");

static struct qcsapi_param_name_ent _qcsapi_param_name_acs[] = {
	{qcsapi_acs_param_obss_chk,		"obss_chk"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_acs, "ACS");

static struct qcsapi_param_name_ent _qcsapi_param_name_qos_premier_rule[] = {
	{qcsapi_qos_premier_rule_none,		"none"},
	{qcsapi_qos_premier_rule_atf,		"atf"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_qos_premier_rule, "QoS Premier Rule");

static struct qcsapi_param_name_ent _qcsapi_param_name_qos_entity[] = {
	{qcsapi_qos_entity_node,		"node"},
	{qcsapi_qos_entity_vap,			"vap"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_qos_entity, "QoS Entity");

static struct qcsapi_param_name_ent _qcsapi_param_name_qos_class[] = {
	{qcsapi_qos_airtime_ctl,		"airtime"},
	{qcsapi_qos_tp_ctl,			"throughput"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_qos_class, "QoS Control Class");

static struct qcsapi_param_name_ent _qcsapi_param_name_zsdfs_param[] = {
	{qcsapi_zsdfs_enable,		"enable"},
	{qcsapi_zsdfs_chan_bw,		"chan_bw"},
};
QCSAPI_PARAM_NAME_TBL(qcsapi_param_name_zsdfs_param, "ZSDFS Param");

static const struct qcsapi_eth_info_result_s {
	qcsapi_eth_info_result result_type;
	const char *result_label;
	const char *result_bit_set;
	const char *result_bit_unset;
} qcsapi_eth_info_result_table[] = {
	{qcsapi_eth_info_connected,	"Connected",		"yes",		"no"},
	{qcsapi_eth_info_speed_unknown,	"Speed",		"unknown",	NULL},
	{qcsapi_eth_info_speed_10M,	"Speed",		"10Mb/s",	NULL},
	{qcsapi_eth_info_speed_100M,	"Speed",		"100Mb/s",	NULL},
	{qcsapi_eth_info_speed_1000M,	"Speed",		"1000Mb/s",	NULL},
	{qcsapi_eth_info_speed_2500M,	"Speed",		"2500Mb/s",	NULL},
	{qcsapi_eth_info_speed_10000M,	"Speed",		"10000Mb/s",	NULL},
	{qcsapi_eth_info_duplex_full,	"Duplex",		"full",		"half"},
	{qcsapi_eth_info_autoneg_on,	"Auto-negotiation",	NULL,		"disabled"},
	{qcsapi_eth_info_autoneg_success, "Auto-negotiation",	"completed",	"failed"},
};

static const struct {
	qcsapi_eth_info_type type;
	qcsapi_eth_info_type_mask mask;
} qcsapi_eth_info_type_mask_table[] = {
	{qcsapi_eth_info_link,		qcsapi_eth_info_link_mask},
	{qcsapi_eth_info_speed,		qcsapi_eth_info_speed_mask},
	{qcsapi_eth_info_duplex,	qcsapi_eth_info_duplex_mask},
	{qcsapi_eth_info_autoneg,	qcsapi_eth_info_autoneg_mask},
	{qcsapi_eth_info_all,		qcsapi_eth_info_all_mask},
};

static const char * const qcsapi_auth_algo_list[] = {
	"OPEN",
	"SHARED",
};

static const char * const qcsapi_auth_keyproto_list[] = {
	"NONE",
	"WPA",
	"WPA2",
};

static const char * const qcsapi_auth_keymgmt_list[] = {
	"NONE",
	"WPA-EAP",
	"WPA-PSK",
	"WEP",
};

static const char * const qcsapi_auth_cipher_list[] = {
	"WEP",
	"TKIP",
	"OCB",
	"CCMP",
	"CMAC",
	"CKIP",
};

static const char * const qcsapi_wifi_modes_strings[] = WLAN_WIFI_MODES_STRINGS;

static const char * const qcsapi_csw_reason_list[] = {
	[IEEE80211_CSW_REASON_UNKNOWN] = "UNKNOWN",
	[IEEE80211_CSW_REASON_SCS] = "SCS",
	[IEEE80211_CSW_REASON_DFS] = "DFS",
	[IEEE80211_CSW_REASON_MANUAL] = "MANUAL",
	[IEEE80211_CSW_REASON_CONFIG] = "CONFIG",
	[IEEE80211_CSW_REASON_SCAN] = "SCAN",
	[IEEE80211_CSW_REASON_OCAC] = "SDFS",
	[IEEE80211_CSW_REASON_CSA] = "CSA",
	[IEEE80211_CSW_REASON_TDLS_CS] = "TDLS",
	[IEEE80211_CSW_REASON_ZSDFS] = "ZSDFS",
};

/*
 * Node information set labels
 */
#define QTN_NIS_LABEL_LEN	50

/*
 * Node information set labels
 * This table must be kept in sync with Node Information Set enums (e.g. qtn_nis_s0_e).
 */
const struct nis_meta_data qtn_nis_label[][QTN_NIS_VAL_MAX] = {
	{ /* Set 0 */
	[QTN_NIS_S0_assoc_id] =		{QTN_NIS_VAL_UNSIGNED, "Association ID"},
	[QTN_NIS_S0_bw] =		{QTN_NIS_VAL_UNSIGNED, "Bandwidth"},
	[QTN_NIS_S0_tx_bytes] =		{QTN_NIS_VAL_UNSIGNED, "Tx bytes"},
	[QTN_NIS_S0_tx_packets] =	{QTN_NIS_VAL_UNSIGNED, "Tx packets"},
	[QTN_NIS_S0_tx_amsdu_msdus] =	{QTN_NIS_VAL_UNSIGNED, "Tx aggregated MSDUs"},
	[QTN_NIS_S0_tx_mpdus] =		{QTN_NIS_VAL_UNSIGNED, "Tx MPDUs"},
	[QTN_NIS_S0_tx_ppdus] =		{QTN_NIS_VAL_UNSIGNED, "Tx PPDUs"},
	[QTN_NIS_S0_tx_dropped] =	{QTN_NIS_VAL_UNSIGNED, "Tx discards"},
	[QTN_NIS_S0_tx_wifi_drop1] =	{QTN_NIS_VAL_UNSIGNED,
					"Packets failed to transmit on AC 1"},
	[QTN_NIS_S0_tx_wifi_drop2] =	{QTN_NIS_VAL_UNSIGNED,
					"Packets failed to transmit on AC 2"},
	[QTN_NIS_S0_tx_wifi_drop3] =	{QTN_NIS_VAL_UNSIGNED,
					"Packets failed to transmit on AC 3"},
	[QTN_NIS_S0_tx_wifi_drop4] =	{QTN_NIS_VAL_UNSIGNED,
					"Packets failed to transmit on AC 4"},
	[QTN_NIS_S0_tx_errors] =	{QTN_NIS_VAL_UNSIGNED, "Tx errors"},
	[QTN_NIS_S0_tx_ucast] =		{QTN_NIS_VAL_UNSIGNED, "Tx unicast"},
	[QTN_NIS_S0_tx_mcast] =		{QTN_NIS_VAL_UNSIGNED, "Tx multicast"},
	[QTN_NIS_S0_tx_mcast_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx bytes multicast"},
	[QTN_NIS_S0_tx_bcast] =		{QTN_NIS_VAL_UNSIGNED, "Tx broadcast"},
	[QTN_NIS_S0_tx_max_phy_rate] =	{QTN_NIS_VAL_UNSIGNED, "Tx max PHY rate (kbps)"},
	[QTN_NIS_S0_tx_max_nss] =	{QTN_NIS_VAL_UNSIGNED, "Tx max NSS"},
	[QTN_NIS_S0_tx_max_mcs] =	{QTN_NIS_VAL_UNSIGNED, "Tx max MCS"},
	[QTN_NIS_S0_tx_last_phy_rate] =	{QTN_NIS_VAL_UNSIGNED, "Tx last PHY rate (Mbps)"},
	[QTN_NIS_S0_tx_median_phyrate] = {QTN_NIS_VAL_UNSIGNED, "Tx median PHY rate (Mbps)"},
	[QTN_NIS_S0_tx_last_nss] =	{QTN_NIS_VAL_UNSIGNED, "Tx last NSS"},
	[QTN_NIS_S0_tx_last_mcs] =	{QTN_NIS_VAL_UNSIGNED, "Tx last MCS"},
	[QTN_NIS_S0_tx_allretries] =	{QTN_NIS_VAL_UNSIGNED, "Retrans pkts"},
	[QTN_NIS_S0_tx_exceed_retries] =	{QTN_NIS_VAL_UNSIGNED,
					"Retrans pkts exceeding retry limit"},
	[QTN_NIS_S0_tx_retries_succ] =	{QTN_NIS_VAL_UNSIGNED, "Successfully retrans pkts"},
	[QTN_NIS_S0_tx_mretries_succ] =	{QTN_NIS_VAL_UNSIGNED,
					"Successfully retrans pkts with multiple retries"},
	[QTN_NIS_S0_rx_flags] =		{QTN_NIS_VAL_UNSIGNED, "Rx flags"},
	[QTN_NIS_S0_tx_retries] =	{QTN_NIS_VAL_UNSIGNED, "Tx retries"},
	[QTN_NIS_S0_tx_bw] =		{QTN_NIS_VAL_UNSIGNED, "Tx bandwidth"},
	[QTN_NIS_S0_tx_pppc] =		{QTN_NIS_VAL_UNSIGNED, "Tx PPPC (dBm)"},
	[QTN_NIS_S0_tx_gi] =		{QTN_NIS_VAL_GI_TYPE, "Tx Guard Interval"},
	[QTN_NIS_S0_rx_bytes] =		{QTN_NIS_VAL_UNSIGNED, "Rx bytes"},
	[QTN_NIS_S0_rx_packets] =	{QTN_NIS_VAL_UNSIGNED, "Rx packets"},
	[QTN_NIS_S0_rx_amsdu_msdus] =	{QTN_NIS_VAL_UNSIGNED, "Rx aggregated MSDUs"},
	[QTN_NIS_S0_rx_mpdus] =		{QTN_NIS_VAL_UNSIGNED, "Rx MPDUs"},
	[QTN_NIS_S0_rx_ppdus] =		{QTN_NIS_VAL_UNSIGNED, "Rx PPDUs"},
	[QTN_NIS_S0_rx_dropped] =	{QTN_NIS_VAL_UNSIGNED, "Rx discards"},
	[QTN_NIS_S0_rx_errors] =	{QTN_NIS_VAL_UNSIGNED, "Rx errors"},
	[QTN_NIS_S0_rx_ucast] =		{QTN_NIS_VAL_UNSIGNED, "Rx unicast"},
	[QTN_NIS_S0_rx_mcast] =		{QTN_NIS_VAL_UNSIGNED, "Rx multicast"},
	[QTN_NIS_S0_rx_mcast_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx bytes multicast"},
	[QTN_NIS_S0_rx_bcast] =		{QTN_NIS_VAL_UNSIGNED, "Rx broadcast"},
	[QTN_NIS_S0_rx_unknown] =	{QTN_NIS_VAL_UNSIGNED, "Rx unknown data"},
	[QTN_NIS_S0_rx_max_phy_rate] =	{QTN_NIS_VAL_UNSIGNED, "Rx max PHY rate (kbps)"},
	[QTN_NIS_S0_rx_median_phyrate] = {QTN_NIS_VAL_UNSIGNED, "Rx median PHY rate (Mbps)"},
	[QTN_NIS_S0_rx_max_nss] =	{QTN_NIS_VAL_UNSIGNED, "Rx max NSS"},
	[QTN_NIS_S0_rx_max_mcs] =	{QTN_NIS_VAL_UNSIGNED, "Rx max MCS"},
	[QTN_NIS_S0_rx_last_phy_rate] =	{QTN_NIS_VAL_UNSIGNED, "Rx last PHY rate (Mbps)"},
	[QTN_NIS_S0_rx_last_nss] =	{QTN_NIS_VAL_UNSIGNED, "Rx last NSS"},
	[QTN_NIS_S0_rx_last_mcs] =	{QTN_NIS_VAL_UNSIGNED, "Rx last MCS"},
	[QTN_NIS_S0_rx_smthd_rssi] =	{QTN_NIS_VAL_UNSIGNED, "Rx smoothed RSSI (-ve)"},
	[QTN_NIS_S0_rx_flags] =		{QTN_NIS_VAL_UNSIGNED, "Rx flags"},
	[QTN_NIS_S0_rx_retries] =	{QTN_NIS_VAL_UNSIGNED, "Rx retries"},
	[QTN_NIS_S0_rx_bw] =		{QTN_NIS_VAL_UNSIGNED, "Rx bandwidth"},
	[QTN_NIS_S0_rx_last_rssi] =	{QTN_NIS_VAL_UNSIGNED, "Rx last RSSI (-ve)"},
	[QTN_NIS_S0_rx_last_rssi_tot] =	{QTN_NIS_VAL_UNSIGNED, "Rx last total RSSI (-ve)"},
	[QTN_NIS_S0_rx_smthd_rssi_tot] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx smoothed total RSSI (-ve)"},
	[QTN_NIS_S0_rx_gi] =		{QTN_NIS_VAL_GI_TYPE, "Rx Guard Interval"},
	[QTN_NIS_S0_wifi_mode] =	{QTN_NIS_VAL_WIFI_MODE, "WiFi mode"},
	},
	{ /* Set 1 */
	[QTN_NIS_S1_tx_tid0_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID0 bytes"},
	[QTN_NIS_S1_tx_tid1_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID1 bytes"},
	[QTN_NIS_S1_tx_tid2_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID2 bytes"},
	[QTN_NIS_S1_tx_tid3_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID3 bytes"},
	[QTN_NIS_S1_tx_tid4_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID4 bytes"},
	[QTN_NIS_S1_tx_tid5_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID5 bytes"},
	[QTN_NIS_S1_tx_tid6_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID6 bytes"},
	[QTN_NIS_S1_tx_tid7_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx TID7 bytes"},

	[QTN_NIS_S1_rx_tid0_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID0 bytes"},
	[QTN_NIS_S1_rx_tid1_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID1 bytes"},
	[QTN_NIS_S1_rx_tid2_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID2 bytes"},
	[QTN_NIS_S1_rx_tid3_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID3 bytes"},
	[QTN_NIS_S1_rx_tid4_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID4 bytes"},
	[QTN_NIS_S1_rx_tid5_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID5 bytes"},
	[QTN_NIS_S1_rx_tid6_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID6 bytes"},
	[QTN_NIS_S1_rx_tid7_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Rx TID7 bytes"},
	},
	{ /* Set 2 */
	[QTN_NIS_S2_mu_beamformer] =	{QTN_NIS_VAL_UNSIGNED, "MU beamformer capability"},
	[QTN_NIS_S2_mu_beamformee] =	{QTN_NIS_VAL_UNSIGNED, "MU beamformee capability"},
	[QTN_NIS_S2_su_beamformer] =	{QTN_NIS_VAL_UNSIGNED, "SU beamformer capability"},
	[QTN_NIS_S2_su_beamformee] =	{QTN_NIS_VAL_UNSIGNED, "SU beamformee capability"},
	[QTN_NIS_S2_tx_11n_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx 802.11n bytes"},
	[QTN_NIS_S2_tx_11ac_su_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx 802.11ac SU bytes"},
	[QTN_NIS_S2_tx_11ac_mu_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx 802.11ac MU bytes"},
	[QTN_NIS_S2_tx_11ax_su_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx 802.11ax SU bytes"},
	[QTN_NIS_S2_tx_11ax_mu_bytes] =	{QTN_NIS_VAL_UNSIGNED, "Tx 802.11ax MU bytes"},
	[QTN_NIS_S2_tx_11ax_ofdma_bytes] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx 802.11ax OFDMA SU bytes"},
	[QTN_NIS_S2_tx_su_nss] =	{QTN_NIS_VAL_UNSIGNED, "Tx SU beamforee NSS"},
	[QTN_NIS_S2_tx_su_is_orthsnd] =	{QTN_NIS_VAL_UNSIGNED, "Tx SU is orthogonal sounding"},
	[QTN_NIS_S2_tx_mu_nss] =	{QTN_NIS_VAL_UNSIGNED, "Tx MU beamformee NSS"},
	[QTN_NIS_S2_tx_mu_is_orthsnd] =	{QTN_NIS_VAL_UNSIGNED, "Tx MU is orthogonal sounding"},
	[QTN_NIS_S2_tx_mu_aid] =	{QTN_NIS_VAL_UNSIGNED, "Tx MU AID"},
	},
	{ /* Set 3 */
	[QTN_NIS_S3_tx_amsdu_subfrms_1] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 1 subframe"},
	[QTN_NIS_S3_tx_amsdu_subfrms_2_4] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 2 to 4 subframes"},
	[QTN_NIS_S3_tx_amsdu_subfrms_5_8] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 5 to 8 subframes"},
	[QTN_NIS_S3_tx_amsdu_subfrms_9_16] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 9 to 16 subframes"},
	[QTN_NIS_S3_tx_amsdu_subfrms_17_32] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 17 to 32 subframes"},
	[QTN_NIS_S3_tx_amsdu_subfrms_gt_32] =	{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs with 33+ subframes"},
	[QTN_NIS_S3_tx_amsdu_lt_2k] =		{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs smaller than 2kB"},
	[QTN_NIS_S3_tx_amsdu_2k_4k] =		{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs from 2kB to 4kB"},
	[QTN_NIS_S3_tx_amsdu_4k_8k] =		{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs from 4kB to 8kB"},
	[QTN_NIS_S3_tx_amsdu_gt_8k] =		{QTN_NIS_VAL_UNSIGNED,
						"Tx A-MSDUs larger than 8kB"},
	[QTN_NIS_S3_rx_amsdu_subfrms_1] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 1 subframe"},
	[QTN_NIS_S3_rx_amsdu_subfrms_2_4] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 2 to 4 subframes"},
	[QTN_NIS_S3_rx_amsdu_subfrms_5_8] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 5 to 8 subframes"},
	[QTN_NIS_S3_rx_amsdu_subfrms_9_16] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 9 to 16 subframes"},
	[QTN_NIS_S3_rx_amsdu_subfrms_17_32] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 17 to 32 subframes"},
	[QTN_NIS_S3_rx_amsdu_subfrms_gt_32] =	{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs with 33+ subframes"},
	[QTN_NIS_S3_rx_amsdu_lt_2k] =		{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs smaller than 2kB"},
	[QTN_NIS_S3_rx_amsdu_2k_4k] =		{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs from 2kB to 4kB"},
	[QTN_NIS_S3_rx_amsdu_4k_8k] =		{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs from 4kB to 8kB"},
	[QTN_NIS_S3_rx_amsdu_gt_8k] =		{QTN_NIS_VAL_UNSIGNED,
						"Rx A-MSDUs larger than 8kB"},
	},
	{ /* Set 4 */
	[QTN_NIS_S4_rssi_chain_1] =	{QTN_NIS_VAL_DBM, "RSSI chain 1"},
	[QTN_NIS_S4_rssi_chain_2] =	{QTN_NIS_VAL_DBM, "RSSI chain 2"},
	[QTN_NIS_S4_rssi_chain_3] =	{QTN_NIS_VAL_DBM, "RSSI chain 3"},
	[QTN_NIS_S4_rssi_chain_4] =	{QTN_NIS_VAL_DBM, "RSSI chain 4"},
	[QTN_NIS_S4_rssi_chain_5] =	{QTN_NIS_VAL_DBM, "RSSI chain 5"},
	[QTN_NIS_S4_rssi_chain_6] =	{QTN_NIS_VAL_DBM, "RSSI chain 6"},
	[QTN_NIS_S4_rssi_chain_7] =	{QTN_NIS_VAL_DBM, "RSSI chain 7"},
	[QTN_NIS_S4_rssi_chain_8] =	{QTN_NIS_VAL_DBM, "RSSI chain 8"},
	[QTN_NIS_S4_rssi_average] =	{QTN_NIS_VAL_DBM, "RSSI average"},
	[QTN_NIS_S4_rcpi_chain_1] =	{QTN_NIS_VAL_DBM, "RCPI chain 1"},
	[QTN_NIS_S4_rcpi_chain_2] =	{QTN_NIS_VAL_DBM, "RCPI chain 2"},
	[QTN_NIS_S4_rcpi_chain_3] =	{QTN_NIS_VAL_DBM, "RCPI chain 3"},
	[QTN_NIS_S4_rcpi_chain_4] =	{QTN_NIS_VAL_DBM, "RCPI chain 4"},
	[QTN_NIS_S4_rcpi_chain_5] =	{QTN_NIS_VAL_DBM, "RCPI chain 5"},
	[QTN_NIS_S4_rcpi_chain_6] =	{QTN_NIS_VAL_DBM, "RCPI chain 6"},
	[QTN_NIS_S4_rcpi_chain_7] =	{QTN_NIS_VAL_DBM, "RCPI chain 7"},
	[QTN_NIS_S4_rcpi_chain_8] =	{QTN_NIS_VAL_DBM, "RCPI chain 8"},
	[QTN_NIS_S4_rcpi_max] =		{QTN_NIS_VAL_DBM, "RCPI max"},
	[QTN_NIS_S4_evm_chain_1] =	{QTN_NIS_VAL_DBM, "EVM chain 1"},
	[QTN_NIS_S4_evm_chain_2] =	{QTN_NIS_VAL_DBM, "EVM chain 2"},
	[QTN_NIS_S4_evm_chain_3] =	{QTN_NIS_VAL_DBM, "EVM chain 3"},
	[QTN_NIS_S4_evm_chain_4] =	{QTN_NIS_VAL_DBM, "EVM chain 4"},
	[QTN_NIS_S4_evm_chain_5] =	{QTN_NIS_VAL_DBM, "EVM chain 5"},
	[QTN_NIS_S4_evm_chain_6] =	{QTN_NIS_VAL_DBM, "EVM chain 6"},
	[QTN_NIS_S4_evm_chain_7] =	{QTN_NIS_VAL_DBM, "EVM chain 7"},
	[QTN_NIS_S4_evm_chain_8] =	{QTN_NIS_VAL_DBM, "EVM chain 8"},
	[QTN_NIS_S4_evm_sum] =		{QTN_NIS_VAL_DBM, "EVM sum"},
	[QTN_NIS_S4_hw_noise_chain_1] =	{QTN_NIS_VAL_DBM, "HW noise chain 1"},
	[QTN_NIS_S4_hw_noise_chain_2] =	{QTN_NIS_VAL_DBM, "HW noise chain 2"},
	[QTN_NIS_S4_hw_noise_chain_3] =	{QTN_NIS_VAL_DBM, "HW noise chain 3"},
	[QTN_NIS_S4_hw_noise_chain_4] =	{QTN_NIS_VAL_DBM, "HW noise chain 4"},
	[QTN_NIS_S4_hw_noise_chain_5] =	{QTN_NIS_VAL_DBM, "HW noise chain 5"},
	[QTN_NIS_S4_hw_noise_chain_6] =	{QTN_NIS_VAL_DBM, "HW noise chain 6"},
	[QTN_NIS_S4_hw_noise_chain_7] =	{QTN_NIS_VAL_DBM, "HW noise chain 7"},
	[QTN_NIS_S4_hw_noise_chain_8] =	{QTN_NIS_VAL_DBM, "HW noise chain 8"},
	[QTN_NIS_S4_hw_noise_average] =	{QTN_NIS_VAL_DBM, "HW noise average"},
	},
	{ /* Set 5 */
	},
	{ /* Set 6 */
	[QTN_NIS_S6_timestamp_last_rx] =	{QTN_NIS_VAL_UNSIGNED, "Timestamp last Rx"},
	[QTN_NIS_S6_timestamp_last_tx] =	{QTN_NIS_VAL_UNSIGNED, "Timestamp last Tx"},
	[QTN_NIS_S6_average_tx_phyrate] =	{QTN_NIS_VAL_UNSIGNED, "Average Tx PHY rate"},
	[QTN_NIS_S6_average_rx_phyrate] =	{QTN_NIS_VAL_UNSIGNED, "Average Rx PHY rate"},
	[QTN_NIS_S6_average_rssi] =	{QTN_NIS_VAL_SIGNED, "Average RSSI"},
	[QTN_NIS_S6_pkts_per_sec] =	{QTN_NIS_VAL_UNSIGNED, "Packets per second"},
	[QTN_NIS_S6_tx_pkt_errors] =	{QTN_NIS_VAL_UNSIGNED, "Tx packet errors"},
	[QTN_NIS_S6_tx_airtime] =	{QTN_NIS_VAL_UNSIGNED, "Tx airtime (10ths of a percent)"},
	[QTN_NIS_S6_rx_airtime] =	{QTN_NIS_VAL_UNSIGNED, "Rx airtime (10ths of a percent)"},
	[QTN_NIS_S6_tx_last_rate] =	{QTN_NIS_VAL_UNSIGNED, "Last data Tx rate (kbps)"},
	[QTN_NIS_S6_rx_last_rate] =	{QTN_NIS_VAL_UNSIGNED, "Last data Rx rate (kbps)"},
	[QTN_NIS_S6_tx_retry_cnt] =	{QTN_NIS_VAL_UNSIGNED, "Retransmission count"},
	}
};

/**
 * Scan information set labels
 */
#define QTN_SIS_LABEL_LEN	35

/*
 * All-node information set labels
 * This table must be kept in sync with All-node Information Set enums (e.g. qtn_nis_all_s0_e).
 */
const struct nis_meta_data qtn_nis_all_label[][QTN_NIS_ALL_FIELD_MAX] = {
	{ /* Set 0 */
	[QTN_NIS_ALL_S0_rsn_caps] = {QTN_NIS_VAL_RSN_CAPS, "RSN capabilities"},
	[QTN_NIS_ALL_S0_rsn_ucastcipher] = {QTN_NIS_VAL_RSN_UCASTCIPHER, "RSN unicast ciphers"},
	[QTN_NIS_ALL_S0_rsn_mcastcipher] = {QTN_NIS_VAL_RSN_MCASTCIPHER, "RSN multicast ciphers"},
	[QTN_NIS_ALL_S0_rsn_keymgmt] = {QTN_NIS_VAL_RSN_KEYMGMT, "RSN key management"},
	}
};

/**
 * All-node scan information set labels
 * This table must be kept in sync with All-node Scan Information Set enums (e.g.,
 * qtn_sis_all_s0_e).
 */
const struct sis_meta_data qtn_sis_all_label[][QTN_SIS_DATA_ENTRY_MAX] = {
	{ /* Set 0 */
	[QTN_SIS_ALL_S0_ap_bandinfo] = {QTN_SIS_VAL_BAND_INFO, "Operating band"},
	[QTN_SIS_ALL_S0_ap_channel] = {QTN_SIS_VAL_SIGNED, "Channel"},
	[QTN_SIS_ALL_S0_ap_bw] = {QTN_SIS_VAL_SIGNED, "Bandwidth"},
	[QTN_SIS_ALL_S0_ap_rssi] = {QTN_SIS_VAL_SIGNED, "RSSI"},
	[QTN_SIS_ALL_S0_ap_protocol] = {QTN_SIS_VAL_SECURITY_PROTOCOL, "Protocol"},
	[QTN_SIS_ALL_S0_ap_flags] = {QTN_SIS_VAL_SGI_CAP, "SGI capability"},
	[QTN_SIS_ALL_S0_ap_encryption_modes] = {QTN_SIS_VAL_RSN_UCASTCIPHER, "Encryption modes"},
	[QTN_SIS_ALL_S0_ap_authentication_modes] = {QTN_SIS_VAL_RSN_KEYMGMT, "Authentication modes"},
	[QTN_SIS_ALL_S0_ap_best_data_rates] = {QTN_SIS_VAL_DATA_RATE, "Best data rate"},
	[QTN_SIS_ALL_S0_ap_wps] = {QTN_SIS_VAL_PRINT_SKIP, "WPS"},
	[QTN_SIS_ALL_S0_ap_80211_proto] = {QTN_SIS_VAL_UNSIGNED, "802.11 protocol"},
	[QTN_SIS_ALL_S0_ap_qhop_role] = {QTN_SIS_VAL_PRINT_SKIP, "QHOP role"},
	[QTN_SIS_ALL_S0_ap_noise] = {QTN_SIS_VAL_PRINT_SKIP, "Noise"},
	[QTN_SIS_ALL_S0_ap_opmode] = {QTN_SIS_VAL_UNSIGNED, "Operating mode"},
	[QTN_SIS_ALL_S0_ap_bintval] = {QTN_SIS_VAL_SIGNED, "Beacon interval"},
	[QTN_SIS_ALL_S0_ap_ht_secoffset] = {QTN_SIS_VAL_HT_SEC_OFFSET,
					"HT secondary channel offset"},
	[QTN_SIS_ALL_S0_ap_chan_center1] = {QTN_SIS_VAL_UNSIGNED, "Channel Center Freq Segment 0"},
	[QTN_SIS_ALL_S0_ap_chan_center2] = {QTN_SIS_VAL_UNSIGNED, "Channel Center Freq Segment 1"},
	[QTN_SIS_ALL_S0_ap_last_beacon] = {QTN_SIS_VAL_UNSIGNED, "Last seen"},
	[QTN_SIS_ALL_S0_ap_dtimeriod] = {QTN_SIS_VAL_PRINT_SKIP, "DTIM"},
	[QTN_SIS_ALL_S0_ap_11b_present] = {QTN_SIS_VAL_PRINT_SKIP, "802.11b present"},
	[QTN_SIS_ALL_S0_ap_bsscolor] = {QTN_SIS_VAL_UNSIGNED, "BSS Color"},
	[QTN_SIS_ALL_S0_ap_heop] = {QTN_SIS_VAL_UNSIGNED, "HE Operation"},
	}
};

#define QTN_SIS_ALL_LABEL_PRINTABLE(_set_id, _fld)	\
	(qtn_sis_all_label[_set_id][_fld].type != QTN_SIS_VAL_PRINT_SKIP)

static const struct {
	const char		*name;
	enum qcsapi_freq_band	index;
} qcsapi_freq_band_table[] = {
	{"default",	qcsapi_freq_band_default}, /* 2.4GHz or 5GHz */
	{"2.4GHz",	qcsapi_freq_band_2pt4_ghz},
	{"4GHz",	qcsapi_freq_band_4_ghz},
	{"5GHz",	qcsapi_freq_band_5_ghz},
	{"6GHz",	qcsapi_freq_band_6_ghz},
};

/**
 * Interface information set labels
 */
#define QTNIS_IF_LABEL_LEN	35

/**
 * Interface information set labels
 * This table must be kept in sync with Interface Information Set enums (e.g. qtnis_if_s0_e).
 */
const struct nis_meta_data qcsapi_qtnis_if_label[][QTNIS_IF_VAL_MAX] = {
	{ /* Set 0 */
	[QTNIS_S0_assoc_id] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_bw] = {QTN_NIS_VAL_UNSIGNED, ""},

	[QTNIS_S0_tx_bytes] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_packets] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_amsdu_msdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_mpdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_ppdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_sent_be] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_sent_bk] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_sent_vi] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_sent_vo] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_dropped] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_drop_be] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_drop_bk] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_drop_vi] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_wifi_drop_vo] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_errors] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_ucast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_mcast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_bcast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_max_phy_rate] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_max_nss] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_max_mcs] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_last_phy_rate] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_last_nss] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_last_mcs] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_flags] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_retries] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_tx_bw] = {QTN_NIS_VAL_UNSIGNED, ""},

	[QTNIS_S0_rx_bytes] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_packets] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_amsdu_msdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_mpdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_ppdus] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_dropped] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_errors] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_ucast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_mcast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_bcast] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_unknown] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_max_phy_rate] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_max_nss] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_max_mcs] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_last_phy_rate] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_last_nss] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_last_mcs] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_smthd_rssi] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_flags] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_retries] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_bw] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_last_rssi] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_last_rssi_tot] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S0_rx_smthd_rssi_tot] = {QTN_NIS_VAL_UNSIGNED, ""},

	[QTNIS_S0_tx_ucast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Unicast bytes sent" },
	[QTNIS_S0_tx_mcast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Multicast bytes sent" },
	[QTNIS_S0_tx_bcast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Broadcast bytes sent" },
	[QTNIS_S0_rx_ucast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Unicast bytes received" },
	[QTNIS_S0_rx_mcast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Multicast bytes received" },
	[QTNIS_S0_rx_bcast_bytes] = {QTN_NIS_VAL_UNSIGNED, "Broadcast bytes received" },
	},
	{ /* Set 1 */
	[QTNIS_S1_offset] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S1_duration] = {QTN_NIS_VAL_UNSIGNED, ""},
	[QTNIS_S1_channel] = {QTN_NIS_VAL_UNSIGNED, ""},

	[QTNIS_S1_basic] = {QTN_NIS_VAL_UNSIGNED, ""},
	},
	{ /* Set 2 */
	[QTNIS_S2_tx_ack_failures] = {QTN_NIS_VAL_UNSIGNED, "ACK failures"},
	[QTNIS_S2_rx_invalid_mac_header] = {QTN_NIS_VAL_UNSIGNED, "Rx with invalid MAC header"},
	[QTNIS_S2_rx_non_assoc_packets] = {QTN_NIS_VAL_UNSIGNED, "Rx non-assoc packets"},
	[QTNIS_S2_rx_plcp_errors] = {QTN_NIS_VAL_UNSIGNED, "Rx with PLCP errors"},
	[QTNIS_S2_rx_fcs_errors] = {QTN_NIS_VAL_UNSIGNED, "Rx with FCS errors"},
	[QTNIS_S2_tx_overflow] = {QTN_NIS_VAL_UNSIGNED, "Tx Overflow"},
	},
	{ /* Set 3 */
	[QTNIS_S3_radar_status] = {QTN_NIS_VAL_UNSIGNED, "Radar status"},
	[QTNIS_S3_radar_mode] = {QTN_NIS_VAL_UNSIGNED, "Radar mode"},
	[QTNIS_S3_radar_bw_1] = {QTN_NIS_VAL_UNSIGNED, "Radar block 1 bandwidth"},
	[QTNIS_S3_radar_detections_1] = {QTN_NIS_VAL_UNSIGNED, "Radar block 1 detections"},
	[QTNIS_S3_radar_bw_2] = {QTN_NIS_VAL_UNSIGNED, "Radar block 2 bandwidth"},
	[QTNIS_S3_radar_detections_2] = {QTN_NIS_VAL_UNSIGNED, "Radar block 2 detections"},
	},
	{ /* Set 4 Full Duplex Repeater (FDR) */
	[QTNIS_S4_fdr_mode] = {QTN_NIS_VAL_UNSIGNED, "FDR mode"},
	[QTNIS_S4_fdr_current_intf_status] = {QTN_NIS_VAL_FDR_STATUS, "Current interface status"},
	[QTNIS_S4_fdr_other_intf] = {QTN_NIS_VAL_IFNAME, "Other interface"},
	[QTNIS_S4_fdr_other_intf_status] = {QTN_NIS_VAL_FDR_STATUS, "Other interface status"},
	[QTNIS_S4_fdr_backup_sta] = {QTN_NIS_VAL_UNSIGNED, "Backup STA"},
	},
	{ /* Set 5 */
	[QTNIS_S5_dbvc_is_running] = {QTN_NIS_VAL_UNSIGNED, "Running"},
	[QTNIS_S5_dbvc_level] = {QTN_NIS_VAL_UNSIGNED, "DBVC level"},
	[QTNIS_S5_dbvc_dwell] = {QTN_NIS_VAL_UNSIGNED, "DBVC dwell time (msec)"},
	[QTNIS_S5_dbvc_downlink_chan] = {QTN_NIS_VAL_UNSIGNED, "Downlink channel"},
	[QTNIS_S5_dbvc_downlink_bw] = {QTN_NIS_VAL_SIGNED, "Downlink BW"},
	[QTNIS_S5_dbvc_uplink_chan] = {QTN_NIS_VAL_UNSIGNED, "Uplink channel"},
	[QTNIS_S5_dbvc_uplink_bw] = {QTN_NIS_VAL_UNSIGNED, "Uplink BW"},
	[QTNIS_S5_dbvc_is_on_downlink] = {QTN_NIS_VAL_UNSIGNED, "Current channel is on downlink"},
	},
	{ /* Set 6 */
	[QTNIS_S6_nfr_is_running] = {QTN_NIS_VAL_UNSIGNED, "Running"},
	[QTNIS_S6_nfr_level] = {QTN_NIS_VAL_UNSIGNED, "NFR level"},
	[QTNIS_S6_nfr_dwell] = {QTN_NIS_VAL_UNSIGNED, "NFR dwell time (msec)"},
	[QTNIS_S6_nfr_same_ch_switch] = {QTN_NIS_VAL_UNSIGNED, "NFR same chan switch"},
	[QTNIS_S6_nfr_downlink_intf] = {QTN_NIS_VAL_IFNAME, "Downlink interface"},
	[QTNIS_S6_nfr_downlink_chan] = {QTN_NIS_VAL_UNSIGNED, "Downlink channel"},
	[QTNIS_S6_nfr_downlink_bw] = {QTN_NIS_VAL_UNSIGNED, "Downlink BW"},
	[QTNIS_S6_nfr_downlink_connect] = {QTN_NIS_VAL_UNSIGNED, "Downlink connect"},
	[QTNIS_S6_nfr_uplink_intf] = {QTN_NIS_VAL_IFNAME, "Uplink interface"},
	[QTNIS_S6_nfr_uplink_chan] = {QTN_NIS_VAL_UNSIGNED, "Uplink channel"},
	[QTNIS_S6_nfr_uplink_bw] = {QTN_NIS_VAL_UNSIGNED, "Uplink BW"},
	[QTNIS_S6_nfr_uplink_connect] = {QTN_NIS_VAL_UNSIGNED, "Uplink connect"},
	[QTNIS_S6_nfr_uplink_duty_cycle] = {QTN_NIS_VAL_UNSIGNED, "Uplink duty cycle"},
	},
	{ /* Set 7 */
	[QTNIS_S7_cap_scan_auto_scan] = {QTN_NIS_VAL_UNSIGNED, "Auto scan"},
	[QTNIS_S7_cap0_scan_boot] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 only at boot"},
	[QTNIS_S7_cap0_scan_impact] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 scan impact"},
	[QTNIS_S7_cap0_scan_min_scan_intv] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 min scan intv"},
	[QTNIS_S7_cap0_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 reserved for future use"},
	[QTNIS_S7_cap0_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 reserved for future use"},
	[QTNIS_S7_cap0_reserved_2] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 reserved for future use"},
	[QTNIS_S7_cap1_scan_boot] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 only at boot"},
	[QTNIS_S7_cap1_scan_impact] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 scan impact"},
	[QTNIS_S7_cap1_scan_min_scan_intv] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 min scan intv"},
	[QTNIS_S7_cap1_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 reserved for future use"},
	[QTNIS_S7_cap1_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 reserved for future use"},
	[QTNIS_S7_cap1_reserved_2] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 reserved for future use"},
	[QTNIS_S7_cap2_scan_boot] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 only at boot"},
	[QTNIS_S7_cap2_scan_impact] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 scan impact"},
	[QTNIS_S7_cap2_scan_min_scan_intv] = {QTN_NIS_VAL_UNSIGNED, "Capability 2 min scan intv"},
	[QTNIS_S7_cap2_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 reserved for future use"},
	[QTNIS_S7_cap2_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 reserved for future use"},
	[QTNIS_S7_cap2_reserved_2] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 reserved for future use"},
	[QTNIS_S7_cap3_scan_boot] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 only at boot"},
	[QTNIS_S7_cap3_scan_impact] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 scan impact"},
	[QTNIS_S7_cap3_scan_min_scan_intv] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 min scan intv"},
	[QTNIS_S7_cap3_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 reserved for future use"},
	[QTNIS_S7_cap3_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 reserved for future use"},
	[QTNIS_S7_cap3_reserved_2] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 reserved for future use"},
	},
	{ /* Set 8 */
	[QTNIS_S8_cap0_cac_type] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 CAC type"},
	[QTNIS_S8_cap0_cac_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 CAC dur"},
	[QTNIS_S8_cap0_cac_dur_wea] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 CAC dur in wea chan"},
	[QTNIS_S8_cap0_cac_nop_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 non-occupancy dur"},
	[QTNIS_S8_cap0_cac_nop_dur_wea] = {QTN_NIS_VAL_UNSIGNED,
							"Cap 0 non-occupancy dur in wea chan"},
	[QTNIS_S8_cap0_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 reserved for future use"},
	[QTNIS_S8_cap0_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 0 reserved for future use"},
	[QTNIS_S8_cap1_cac_type] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 CAC type"},
	[QTNIS_S8_cap1_cac_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 CAC dur"},
	[QTNIS_S8_cap1_cac_dur_wea] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 CAC dur in wea chan"},
	[QTNIS_S8_cap1_cac_nop_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 non-occupancy dur"},
	[QTNIS_S8_cap1_cac_nop_dur_wea] = {QTN_NIS_VAL_UNSIGNED,
							"Cap 1 non-occupancy dur in wea chan"},
	[QTNIS_S8_cap1_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 reserved for future use"},
	[QTNIS_S8_cap1_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 1 reserved for future use"},
	[QTNIS_S8_cap2_cac_type] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 CAC type"},
	[QTNIS_S8_cap2_cac_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 CAC dur"},
	[QTNIS_S8_cap2_cac_dur_wea] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 CAC dur in wea chan"},
	[QTNIS_S8_cap2_cac_nop_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 non-occupy dur"},
	[QTNIS_S8_cap2_cac_nop_dur_wea] = {QTN_NIS_VAL_UNSIGNED,
							"Cap 2 non-occupancy dur in wea chan"},
	[QTNIS_S8_cap2_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 reserved for future use"},
	[QTNIS_S8_cap2_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 2 reserved for future use"},
	[QTNIS_S8_cap3_cac_type] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 CAC type"},
	[QTNIS_S8_cap3_cac_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 CAC dur"},
	[QTNIS_S8_cap3_cac_dur_wea] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 CAC dur in wea chan"},
	[QTNIS_S8_cap3_cac_nop_dur] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 non-occupancy dur"},
	[QTNIS_S8_cap3_cac_nop_dur_wea] = {QTN_NIS_VAL_UNSIGNED,
							"Cap 3 non-occupancy dur in wea chan"},
	[QTNIS_S8_cap3_reserved_0] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 reserved for future use"},
	[QTNIS_S8_cap3_reserved_1] = {QTN_NIS_VAL_UNSIGNED, "Cap 3 reserved for future use"},
	}
};

char *qcsapi_hs20_params[] = {
	"hs20_wan_metrics",
	"disable_dgaf",
	"hs20_operating_class",
	"osu_ssid",
	"osen",
	"hs20_deauth_req_timeout"
};

static const qcsapi_bsa_conf_table bsa_config[] = { QCSAPI_BSA_CONF_PARAMS };

static int local_atoi32(const char *name, const char *str, int32_t *p, qcsapi_output *print)
{
	char *endptr = NULL;

	errno = 0;

	*p = strtol(str, &endptr, 0);
	if (errno || str == endptr || *endptr != '\0') {
		print_err(print, "Invalid value %s for %s - must be an integer\n",
				str, name);
		return -EINVAL;
	}

	return 0;
}

static int local_atoi32_range(const char *name, const char *str, int32_t *p, qcsapi_output *print,
				int32_t min, int32_t max)
{
	int retval;

	retval = local_atoi32(name, str, p, print);
	if (retval < 0)
		return -EINVAL;

	if (*p < min || *p > max) {
		print_err(print, "Invalid value %s for %s - must be between %d and %d\n",
				str, name, min, max);
		return -EINVAL;
	}

	return 0;
}

static int local_atou32(const char *name, const char *str, uint32_t *p, qcsapi_output *print)
{
	if (qcsapi_str_to_uint32(str, p) < 0) {
		print_err(print, "Invalid value %s for %s - must be an unsigned integer\n",
				str, name);
		return -EINVAL;
	}

	return 0;
}

static int local_atou32_range(const char *name, const char *str, uint32_t *p, qcsapi_output *print,
				uint32_t min, uint32_t max)
{
	int retval;

	retval = local_atou32(name, str, p, print);
	if (retval < 0)
		return -EINVAL;

	if (*p < min || *p > max) {
		print_err(print, "Invalid value %s for %s - must be between %u and %u\n",
			str, name, min, max);
		return -EINVAL;
	}

	return 0;
}

static int local_atou16_range(const char *name, const char *str, uint16_t *p, qcsapi_output *print,
				uint16_t min, uint16_t max)
{
	uint32_t v;
	int retval;

	retval = local_atou32_range(name, str, &v, print, min, max);
	if (retval < 0)
		return -EINVAL;

	*p = (uint16_t) v;

	return 0;
}

static int local_atou16(const char *name, const char *str, uint16_t *p, qcsapi_output *print)
{
	uint32_t v;
	int retval;

	retval = local_atou32_range(name, str, &v, print, 0, 0xffff);
	if (retval < 0)
		return -EINVAL;

	*p = (uint16_t) v;

	return 0;
}

static int local_is_valid_null_string(const char *val)
{
	if (strcasecmp(val, "null") == 0)
		return 1;

	return 0;
}

static int local_atoi_bool(const char *name, const char *arg, int *val, qcsapi_output *print)
{
	return local_atoi32_range(name, arg, val, print, 0, 1);
}

static int local_atou_bool(const char *name, const char *arg, unsigned int *val,
					qcsapi_output *print)
{
	return local_atou32_range(name, arg, val, print, 0, 1);
}

/*
 * Convert "TRUE", "true", "YES", "yes" or "1" to integer value 1
 * Convert "FALSE", "false", "NO", "no" or "0" to integer value 0
 * For backward compatibility only. Use local_atou_bool() for new parameters.
 */
static int local_atoi_bool_legacy(const char *arg, unsigned int *val, qcsapi_output *print)
{
	if (qcsapi_str_to_uint32(arg, val) < 0) {
		if (strcasecmp(arg, "true") == 0 || strcasecmp(arg, "yes") == 0) {
			*val = 1;
		} else if (strcasecmp(arg, "false") == 0 || strcasecmp(arg, "no") == 0) {
			*val = 0;
		} else {
			print_err(print, "Invalid enable value %s\n", arg);
			return -EINVAL;
		}
		return 0;
	}

	return local_atou32_range("enable", arg, val, print, 0, 1);
}

static int local_parse_loca_remote_flag(qcsapi_output *print, const char *local_remote_str,
		int *p_local_remote_flag)
{
	int local_remote = QCSAPI_LOCAL_NODE;

	if (isdigit(local_remote_str[0])) {
		local_remote = atoi(local_remote_str);
	} else if (strcasecmp(local_remote_str, "remote") == 0) {
		local_remote = QCSAPI_REMOTE_NODE;
	} else if (strcasecmp(local_remote_str, "local") == 0) {
		local_remote = QCSAPI_LOCAL_NODE;
	} else {
		print_err(print, "Invalid value %s for local/remote flag\n", local_remote_str);
		return -1;
	}

	*p_local_remote_flag = local_remote;

	return 0;
}

static void call_qcsapi_param_name_list(qcsapi_output *print,
			const struct qcsapi_param_name_tbl *tbl)
{
	uint32_t i;

	for (i = 0; i < tbl->size; i++) {
		if (qcsapi_param_idx_is_defined(tbl, i))
			print_out(print, "  %s\n", qcsapi_param_idx2name(tbl, i));
	}
}

static int call_qcsapi_param_name2enum(qcsapi_output *print,
			const struct qcsapi_param_name_tbl *tbl, const char *name, uint32_t *idx)
{
	if (qcsapi_param_name2enum(tbl, name, idx) != 0) {
		print_err(print, "Invalid QCSAPI %s param %s\n", tbl->tbl_name, name);
		return -1;
	}

	return 0;
}

static int parse_generic_parameter_name(const call_bundle *cb, char *name, qcsapi_generic_param *p)
{
	qcsapi_output *print = cb->output;

	switch (cb->call_entry->generic_param_type) {
	case e_qcsapi_none:
		break;
	case e_qcsapi_option:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_option, name,
							&p->parameter_type.option);
	case e_qcsapi_counter:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_counter, name,
							&p->parameter_type.counter);
	case e_qcsapi_rates:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_rate_types, name,
							&p->parameter_type.typeof_rates);
	case e_qcsapi_modulation:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_wifi_std, name,
							&p->parameter_type.modulation);
	case e_qcsapi_index:
		if (local_atou32("index", name, &p->index, print) < 0)
			return -EINVAL;
		return 0;
	case e_qcsapi_LED:
		if (local_atou32_range("pin", name, &p->index, print, 0, QCSAPI_MAX_LED) < 0)
			return -EINVAL;
		return 0;
	case e_qcsapi_select_SSID:
	case e_qcsapi_SSID_index:
		/*
		 * APIs with generic parameter type of e_qcsapi_SSID_index expect both an SSID and
		 * an index. Get the SSID now. Get the index in the individual call_qcsapi routines.
		 */
		strncpy(p->parameter_type.SSID, name, sizeof(p->parameter_type.SSID) - 1);
		p->parameter_type.SSID[sizeof(p->parameter_type.SSID) - 1] = '\0';
		return 0;
	case e_qcsapi_file_path_config:
		if (strcasecmp("security", name) != 0) {
			print_err(print, "Invalid QCSAPI file path configuration %s\n", name);
			return -EINVAL;
		}
		p->index = qcsapi_security_configuration_path;
		return 0;
	case e_qcsapi_urepeater_params:
		break;
	case e_qcsapi_board_parameter:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_board_param, name,
							&p->parameter_type.board_param);
	case e_qcsapi_extender_params:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_extender_param, name,
							&p->parameter_type.type_of_extender);
	case e_qcsapi_wifi_parameter:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_wifi_param, name,
							&p->parameter_type.wifi_param_type);
	case e_qcsapi_bcn_phyrate:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_phyrate, name,
							&p->parameter_type.phyrate);
	case e_qcsapi_hw_module:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_hw_module, name,
							&p->parameter_type.hw_module);
	case e_qcsapi_eap_params:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_eap, name,
							&p->parameter_type.eap_param_type);
	case e_qcsapi_dpp_parameter:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_dpp_param, name,
							&p->parameter_type.dpp_param_type);
	case e_qcsapi_zsdfs_parameter:
		return call_qcsapi_param_name2enum(print, &qcsapi_param_name_zsdfs_param, name,
							&p->parameter_type.zsdfs_param);
	}

	print_err(print,
		"Internal error - unknown param type %d\n", cb->call_entry->generic_param_type);

	return -EINVAL;
}

static const char *wifi_mode_to_string(qcsapi_output *print, qcsapi_wifi_mode wifi_mode)
{
	switch (wifi_mode) {
	case qcsapi_access_point:
		return "Access point";
	case qcsapi_station:
		return "Station";
	case qcsapi_mode_not_defined:
		return "WiFi mode not defined";
	case qcsapi_nosuch_mode:
	default:
		return "Unknown WiFi mode\n";
	}
}

static qcsapi_wifi_mode string_to_wifi_mode(const char *str)
{
	if (strcasecmp(str, "ap") == 0)
		return qcsapi_access_point;
	else if (strcasecmp(str, "access_point") == 0)
		return qcsapi_access_point;
	else if (strcasecmp(str, "access point") == 0)
		return qcsapi_access_point;
	else if (strcasecmp(str, "sta") == 0)
		return qcsapi_station;
	else if (strcasecmp(str, "station") == 0)
		return qcsapi_station;
	else if (strcasecmp(str, "repeater") == 0)
		return qcsapi_repeater;
	else
		return qcsapi_nosuch_mode;
}

static void local_convert_and_save(void *output_list, int to_uint32, int elem_offset, int value)
{
	if (!to_uint32)
		((uint8_t *) output_list)[elem_offset] = value;
	else
		((uint32_t *) output_list)[elem_offset] = value;
}

static int local_string_to_list(qcsapi_output *print, void *input_str, void *output_list,
		unsigned int *number, int to_uint32)
{
	uint8_t list_number = 0;
	char *pcur = NULL, *pend = NULL;
	char buffer[256] = { 0 };
	char *input_end;
	int single_len = 0;

	if (!input_str || !output_list || !number)
		return -EINVAL;

	input_end = input_str + strnlen(input_str, 1024);
	pcur = input_str;
	do {
		pend = strchr(pcur, ',');
		if (pend) {
			single_len = pend - pcur;
			strncpy(buffer, pcur, single_len);
			buffer[single_len] = 0;
			pend++;
			local_convert_and_save(output_list, to_uint32, list_number++, atoi(buffer));
			pcur = pend;
		} else if (pcur) {
			local_convert_and_save(output_list, to_uint32, list_number++, atoi(pcur));
		}
	} while (pend && pend < input_end);

	*number = list_number;

	return 0;
}

static int local_string_to_u8_list(qcsapi_output *print, void *input_str, uint8_t *output_list,
				unsigned int *number)
{
		return local_string_to_list(print, input_str, output_list, number, 0);
}

static int local_string_to_u32_list(qcsapi_output *print, void *input_str, uint32_t *output_list,
				unsigned int *number)
{
		return local_string_to_list(print, input_str, output_list, number, 1);
}

static void local_dump_macaddr(qcsapi_output *print, qcsapi_mac_addr macaddr)
{
	print_out(print, MACSTR "\n", MAC2STR(macaddr));
}

static void local_dump_data_array(qcsapi_output *print, uint8_t *data, int size, int order,
		char delimiter)
{
	int i;

	if (data == NULL)
		return;

	i = 0;
	if (order == 10) {
		do {
			print_out(print, "%d%c", data[i], delimiter);
			i++;
		} while (i < (size - 1));
		print_out(print, "%d", data[i]);
	} else {
		do {
			print_out(print, "0x%x%c", data[i], delimiter);
			i++;
		} while (i < (size - 1));
		print_out(print, "0x%x", data[i]);
	}

	print_out(print, "\n");
}

static void dump_scs_param(qcsapi_output *print, qcsapi_scs_param_rpt *p_rpt)
{
#define MAX_SCS_PARAM_DESC 35
	uint32_t i;
	int loop;
	uint32_t str_len = 0;
	const char *name;
	uint32_t index;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_scs_param;

	for (i = 0; i < tbl->size; i++) {
		if (!qcsapi_param_idx_is_defined(tbl, i))
			continue;

		name = qcsapi_param_idx2name(tbl, i);
		index = qcsapi_param_idx2enum(tbl, i);

		str_len = min(strlen(name), strlen("scs_tdls_time_compensation"));
		if (!strncmp(name, "scs_tx_time_compensation", str_len) ||
				!strncmp(name, "scs_rx_time_compensation", str_len) ||
				!strncmp(name, "scs_tdls_time_compensation", str_len)) {
			print_out(print, "%-*s ", MAX_SCS_PARAM_DESC, name);
			loop = SCS_MAX_TXTIME_COMP_INDEX;
			do {
				print_out(print, "%u ", p_rpt[index++].scs_cfg_param);
				loop--;
			} while (loop);
			print_out(print, "\n");
		} else {
			if (p_rpt[index].scs_signed_param_flag == 0) {
				print_out(print, "%-*s %u\n", MAX_SCS_PARAM_DESC, name,
						p_rpt[index].scs_cfg_param);
			} else if (p_rpt[index].scs_signed_param_flag == 1) {
				print_out(print, "%-*s %d\n", MAX_SCS_PARAM_DESC, name,
						p_rpt[index].scs_cfg_param);
			} else {
				print_out(print, "invalid param flag!\n");
			}
		}
	}
}

static int report_qcsapi_error(const call_bundle *cb, const int retval)
{
	char error_msg[128] = { '\0' };
	qcsapi_output *print = cb->output;

	qcsapi_errno_get_message(retval, error_msg, sizeof(error_msg));
	print_out(print, "QCS API error %d: %s\n", 0 - retval, error_msg);

	return retval;
}

static int qcsapi_report_usage(const call_bundle *cb)
{
	qcsapi_output *print = cb->output;

	if (cb->call_entry->usage[0] == '\0')
		print_out(print, "Usage: call_qcsapi %s\n",
				cb->call_entry->cmdname);
	else
		print_out(print, "Usage: call_qcsapi %s %s\n",
			cb->call_entry->cmdname, cb->call_entry->usage);

	return -EINVAL;
}

static int qcsapi_report_complete(const call_bundle *cb, int retval)
{
	qcsapi_output *print = cb->output;

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

static int qcsapi_report_str_or_error(const call_bundle *cb, int retval, const char *str)
{
	qcsapi_output *print = cb->output;

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s", str);
	if (str[strlen(str) - 1] != '\n')
		print_out(print, "\n");

	return 0;
}

static int qcsapi_report_uint_or_error(const call_bundle *cb, int retval, unsigned int val)
{
	qcsapi_output *print = cb->output;

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%u\n", val);

	return 0;
}

static int qcsapi_report_int_or_error(const call_bundle *cb, int retval, int val)
{
	qcsapi_output *print = cb->output;

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d\n", val);

	return 0;
}

static void *local_malloc(const call_bundle *cb, int size)
{
	qcsapi_output *print = cb->output;
	void *buf;

	buf = calloc(1, size);
	if (!buf)
		print_err(print, "Failed to allocate %u bytes\n", size);

	return buf;
}

static int local_parse_macaddr(const call_bundle *cb, const char *macaddr_str,
				qcsapi_mac_addr macaddr)
{
	qcsapi_output *print = cb->output;
	int retval;

	retval = parse_mac_addr(macaddr_str, macaddr);
	if (retval < 0) {
		print_out(print, "Error parsing MAC address %s\n", macaddr_str);
		return -EINVAL;
	}

	return 0;
}

static const char *csw_reason_to_string(uint32_t reason_id)
{
	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_csw_reason_list) == IEEE80211_CSW_REASON_MAX);

	if (reason_id < ARRAY_SIZE(qcsapi_csw_reason_list))
		return qcsapi_csw_reason_list[reason_id];

	return qcsapi_csw_reason_list[IEEE80211_CSW_REASON_UNKNOWN];
}

int local_hexstr_to_uint32_range(const char *name, const char *str, uint32_t *result,
			qcsapi_output *print, uint32_t min, uint32_t max)
{
	int retval;

	retval = qcsapi_util_hexstr_to_uint32(str, result);
	if (retval < 0) {
		print_out(print, "Invalid %s hex string %s\n", name, str);
		return -EINVAL;
	}

	if (*result < min || *result > max) {
		print_out(print, "Invalid parameter %s - value must be between %x and %x\n",
										str, min, max);
		return -ERANGE;
	}

	return 0;
}

#define QCSAPI_MAX_ERR_MSG_BUF 256

CALL_QCSAPI(call_qcsapi_errno_get_message)
{
	int retval = 0;
	char buf[QCSAPI_MAX_ERR_MSG_BUF] = { 0 };
	int val;

	if (local_atoi32("errno", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_errno_get_message(val, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_store_ipaddr)
{
	int retval;
	uint32_t ipaddr;
	uint32_t netmask;
	int netmask_len;
	char *slash;

	slash = strstr(argv[0], "/");
	if (slash == NULL) {
		netmask = htonl(0xFFFFFF00);
	} else {
		*slash = '\0';
		netmask_len = atoi(slash + 1);
		if (netmask_len < 1 || netmask_len > 32) {
			print_err(print, "invalid network mask %s\n", slash + 1);
			return -EINVAL;
		}
		netmask = htonl(~((1 << (32 - netmask_len)) - 1));
	}

	if (inet_pton(AF_INET, argv[0], &ipaddr) != 1) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	if (ipaddr == 0) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_store_ipaddr(ipaddr, netmask);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_interface_enable)
{
	int retval;
	uint32_t enable = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_interface_enable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_interface_get_BSSID)
{
	qcsapi_mac_addr macaddr;
	int retval;

	retval = qcsapi_interface_get_BSSID(interface, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(print, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_interface_get_mac_addr)
{
	qcsapi_mac_addr macaddr;
	int retval;

	retval = qcsapi_interface_get_mac_addr(interface, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(print, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_interface_set_mac_addr)
{
	int retval;
	qcsapi_mac_addr macaddr;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_interface_set_mac_addr(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_interface_get_counter)
{
	qcsapi_unsigned_int val;
	int retval;
	qcsapi_counter_type type = cb->generic_param.parameter_type.counter;

	retval = qcsapi_interface_get_counter(interface, type, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_interface_get_counter64)
{
	uint64_t val;
	int retval;
	qcsapi_counter_type type = cb->generic_param.parameter_type.counter;

	retval = qcsapi_interface_get_counter64(interface, type, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%llu\n", val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_pm_get_counter)
{
	qcsapi_unsigned_int val;
	int retval;
	qcsapi_counter_type type = cb->generic_param.parameter_type.counter;
	const char *pm_interval = argv[0];

	retval = qcsapi_pm_get_counter(interface, type, pm_interval, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_pm_get_elapsed_time)
{
	qcsapi_unsigned_int val;
	int retval;
	const char *pm_interval = argv[0];

	retval = qcsapi_pm_get_elapsed_time(pm_interval, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_update_bootcfg_binfile)
{
	int retval;

	retval = qcsapi_update_bootcfg_binfile(argv[0], argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_flash_image_update)
{
	int retval;
	qcsapi_flash_partiton_type partition_type;
	const char *image_file_path;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_partition;

	image_file_path = argv[0];

	retval = call_qcsapi_param_name2enum(print, tbl, argv[1], &partition_type);
	if (retval < 0)
		return retval;

	retval = qcsapi_flash_image_update(image_file_path, partition_type);

	return qcsapi_report_complete(cb, retval);
}

#define GET_FIRMWARE_VERSION_MAX_LEN	40

CALL_QCSAPI(call_qcsapi_firmware_get_version)
{
	int retval;
	char buf[GET_FIRMWARE_VERSION_MAX_LEN];
	qcsapi_unsigned_int len = GET_FIRMWARE_VERSION_MAX_LEN;

	retval = qcsapi_firmware_get_version(buf, len);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_system_get_time_since_start)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_system_get_time_since_start(&val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_get_system_status)
{
	int retval;
	qcsapi_unsigned_int status;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_sys_status;
	uint32_t i;

	retval = qcsapi_get_system_status(&status);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%X\n", status);
	for (i = 0; i < tbl->size; i++) {
		if (!qcsapi_param_idx_is_defined(tbl, i))
			continue;
		print_out(print, "bit %-2d - %s\n",
			qcsapi_param_idx2enum(tbl, i),
			qcsapi_param_idx2name(tbl, i));
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_cpu_usage)
{
	int retval;
	string_256 buf = { 0 };

	retval = qcsapi_get_cpu_usage(buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_get_memory_usage)
{
	int retval;
	string_256 buf = { 0 };

	retval = qcsapi_get_memory_usage(buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_get_random_seed)
{
	int retval = 0;
	struct qcsapi_data_512bytes *buf;
	int i;

	buf = local_malloc(cb, sizeof(*buf));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_get_random_seed(buf);
	if (retval < 0) {
		report_qcsapi_error(cb, retval);
	} else {
		for (i = 0; i < sizeof(buf->data); i++)
			print_out(print, "%c", buf->data[i]);
	}

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_set_random_seed)
{
	int retval;
	struct qcsapi_data_512bytes *buf;
	qcsapi_unsigned_int entropy = 0;

	entropy = atoi(argv[1]);

	buf = local_malloc(cb, sizeof(*buf));
	if (!buf)
		return -ENOMEM;

	memcpy(buf->data, argv[0], min(sizeof(buf->data), strlen(argv[0])));

	retval = qcsapi_set_random_seed(buf, entropy);

	free(buf);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_led_get)
{
	int retval;
	uint8_t led = (uint8_t) (cb->generic_param.index);
	uint8_t led_value;

	retval = qcsapi_led_get(led, &led_value);

	return qcsapi_report_uint_or_error(cb, retval, led_value);
}

CALL_QCSAPI(call_qcsapi_led_set)
{
	int retval;
	uint8_t led = (uint8_t) (cb->generic_param.index);
	uint8_t val = (uint8_t) atoi(argv[0]);

	retval = qcsapi_led_set(led, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_led_pwm_enable)
{
	int retval = 0;
	uint8_t led_ident = (uint8_t) (cb->generic_param.index);
	uint32_t onoff = 0;
	uint32_t high_count = 0;
	uint32_t low_count = 0;

	if (sscanf(argv[0], "%u", &onoff) != 1)
		return qcsapi_report_usage(cb);

	if (onoff != 0) {
		if (argc < 3)
			return qcsapi_report_usage(cb);
		if (sscanf(argv[1], "%u", &high_count) != 1)
			return qcsapi_report_usage(cb);
		if (sscanf(argv[2], "%u", &low_count) != 1)
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_led_pwm_enable(led_ident, (uint8_t) onoff, high_count, low_count);

	return qcsapi_report_complete(cb, retval);

}

CALL_QCSAPI(call_qcsapi_led_brightness)
{
	int retval = 0;
	uint8_t led_ident = (uint8_t) (cb->generic_param.index);
	qcsapi_unsigned_int level = 0;

	if (sscanf(argv[0], "%u", &level) != 1)
		return qcsapi_report_usage(cb);

	retval = qcsapi_led_brightness(led_ident, level);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_gpio_get_config)
{
	int retval;
	uint8_t gpio = (uint8_t) (cb->generic_param.index);
	qcsapi_gpio_config val;

	retval = qcsapi_gpio_get_config(gpio, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_gpio_set_config)
{
	int retval;
	uint8_t gpio = (uint8_t) (cb->generic_param.index);
	qcsapi_gpio_config val = (qcsapi_gpio_config) atoi(argv[0]);

	retval = qcsapi_gpio_set_config(gpio, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_gpio_monitor_reset_device)
{

	print_err(print, "Not supported from call_qcsapi\n");

	return -EOPNOTSUPP;
}

CALL_QCSAPI(call_qcsapi_gpio_enable_wps_push_button)
{
	int retval;
	uint8_t use_interrupt_flag = 0;
	uint8_t wps_push_button = (uint8_t) (cb->generic_param.index);
	uint8_t active_logic = (uint8_t) atoi(argv[0]);

	if (argc > 1 && strcasecmp(argv[1], "intr") == 0)
		use_interrupt_flag = 1;

	retval = qcsapi_gpio_enable_wps_push_button(wps_push_button, active_logic,
			use_interrupt_flag);

	return qcsapi_report_complete(cb, retval);
}

static struct qcsapi_param_name_ent _qcsapi_error_msg_thermal[] = {
	{ QTN_THERMAL_RET_CODE_SUCCESS, "Success" },
	{ QTN_THERMAL_RET_CODE_INVALID_COMMAND, "Invalid command" },
	{ QTN_THERMAL_RET_CODE_INVALID_FLAGS, "Invalid flags" },
	{ QTN_THERMAL_RET_CODE_INVALID_RECORDS, "Invalid record" },
	{ QTN_THERMAL_RET_CODE_INVALID_INTRVL, "Invalid interval" },
	{ QTN_THERMAL_RET_CODE_INVALID_STAGE, "Invalid stage" },
	{ QTN_THERMAL_RET_CODE_INVALID_TEMPERATURE, "Invalid temperature" },
	{ QTN_THERMAL_RET_CODE_INVALID_RADIO_ID, "Invalid radio id" },
	{ QTN_THERMAL_RET_CODE_INVALID_TX_CHAINS, "Invalid tx chains" },
	{ QTN_THERMAL_RET_CODE_INVALID_TX_CHAINS_ON_RADIO, "Tx chains not supported on radio" },
	{ QTN_THERMAL_RET_CODE_INVALID_TX_PWR_BACKOFF, "Invalid tx power backoff" },
	{ QTN_THERMAL_RET_CODE_INVALID_STATUS, "Invalid status" },
	{ QTN_THERMAL_RET_CODE_RADIO_NOT_ENABLED, "Radio not enabled" },
	{ QTN_THERMAL_RET_CODE_NO_TEMP_SENSOR, "No temperature sensor" },
	{ QTN_THERMAL_RET_CODE_ALREADY_ENABLED, "Already enabled" },
	{ QTN_THERMAL_RET_CODE_NSM_ENABLED, "Cannot start when NSM is enabled" },
	{ QTN_THERMAL_RET_CODE_STAGE_NOT_CONFIGURED, "Stage not configured" },
	{ QTN_THERMAL_RET_CODE_OVERLAPPING_STAGE_TEMPS, "Stage temperatures overlap" },
	{ QTN_THERMAL_RET_CODE_INTERNAL, "Driver internal error" },
	{ QTN_THERMAL_RET_CODE_UNKNOWN, "Unknown error" }
};
QCSAPI_PARAM_NAME_TBL(qcsapi_error_msg_thermal, "Thermal error message");

#define QCSAPI_THERMAL_PRNT_FMT_RADIO_CFG \
			"  radio %d, stage %d, low temp %d C, " \
			"high temp %d C, tx chains %d, tx power backoff %d dB\n"

#define QCSAPI_THERMAL_PRNT_FMT_RADIO_STATUS \
			"  radio %d, temp sens %d, config %d, stage %d, rfic temp %d.%d C\n"

static int local_thermal_print_status(qcsapi_output *print,
		struct qtn_thermal_cfg_records *cfg_records)
{
	int idx;
	int cfg_count;

	if (cfg_records->num_records >= ARRAY_SIZE(cfg_records->record)) {
		print_out(print, "Unknown Status\n");
		return -EINVAL;
	}

	cfg_count = 0;
	print_out(print, "Current configuration:\n");
	for (idx = 0; idx < cfg_records->num_records; idx++) {
		if (cfg_records->record[idx].msg_type == QTN_THERMAL_MSG_TYPE_INTERVAL) {
			print_out(print, "  Polling interval: %d\n",
							cfg_records->record[idx].arg1);
			continue;
		}
		if (cfg_records->record[idx].msg_type == QTN_THERMAL_MSG_TYPE_RADIO_CONFIG) {
			print_out(print, QCSAPI_THERMAL_PRNT_FMT_RADIO_CFG,
				cfg_records->record[idx].radio_id, cfg_records->record[idx].arg1,
				cfg_records->record[idx].arg2, cfg_records->record[idx].arg3,
				cfg_records->record[idx].arg4, cfg_records->record[idx].arg5);
			cfg_count++;
			continue;
		}
	}
	if (cfg_count == 0)
		print_out(print, "  Radio not configured\n");

	print_out(print, "Current status:\n");
	print_out(print, "Enabled: %d\n", (cfg_records->ret_status ? 1 : 0));
	if (cfg_records->ret_status == 0)
		return 0;

	cfg_count = 0;
	for (idx = 0; idx < cfg_records->num_records; idx++) {
		if (cfg_records->record[idx].msg_type == QTN_THERMAL_MSG_TYPE_OPER_STATUS) {
			print_out(print, QCSAPI_THERMAL_PRNT_FMT_RADIO_STATUS,
				cfg_records->record[idx].radio_id, cfg_records->record[idx].arg5,
				cfg_records->record[idx].arg1, cfg_records->record[idx].arg2,
				cfg_records->record[idx].arg3, cfg_records->record[idx].arg4);
			cfg_count++;
			continue;
		}
	}
	if (cfg_count == 0)
		print_out(print, "  Status not available\n");

	return 0;
}

static int local_thermal_cmd_proc(const call_bundle *cb, qcsapi_output *print,
			struct qtn_thermal_cfg_param *param)
{
	struct qtn_thermal_cfg_records *cfg_records;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_error_msg_thermal;
	int retval;

	if (!QTN_THERMAL_MSG_TYPE_VALID(param->msg_type)) {
		print_err(print, "Invalid message type\n");
		return -EINVAL;
	}

	cfg_records = local_malloc(cb, sizeof(*cfg_records));
	if (!cfg_records)
		return -ENOMEM;

	memset(cfg_records, 0, sizeof(*cfg_records));
	cfg_records->flags = 0;
	cfg_records->num_records = 1;
	memcpy(&cfg_records->record[0], param, sizeof(cfg_records->record[0]));

	retval = qcsapi_thermal(cfg_records, sizeof(*cfg_records));
	if (retval < 0) {
		print_err(print, "qcsapi_thermal failed (error %d)\n", retval);
		goto exit;
	}
	if (cfg_records->ret_code != QTN_THERMAL_RET_CODE_SUCCESS) {
		print_err(print, "%s (code %d)\n",
			qcsapi_param_enum2name(tbl, cfg_records->ret_code), cfg_records->ret_code);
		retval = -qcsapi_configuration_error;
		goto exit;
	}
	if (param->msg_type == QTN_THERMAL_MSG_TYPE_GET_STATUS)
		retval = local_thermal_print_status(print, cfg_records);

exit:
	free(cfg_records);
	return retval;
}

CALL_QCSAPI(call_qcsapi_thermal)
{
	struct qtn_thermal_cfg_param param;
	int retval;

	if (argc < 1)
		goto out;

	memset(&param, 0, sizeof(param));

	if (!strcasecmp(argv[0], "init")) {
		param.msg_type = QTN_THERMAL_MSG_TYPE_INIT;
	} else if (!strcasecmp(argv[0], "cfg_interval")) {
		if (argc != 2)
			goto out;
		param.msg_type = QTN_THERMAL_MSG_TYPE_INTERVAL;

		if (local_atou32_range("interval", argv[1], &param.arg1, print,
				QTN_THERMAL_POLL_INTRVL_SEC_MIN,
				QTN_THERMAL_POLL_INTRVL_SEC_MAX) < 0)
			return -EINVAL;
	} else if (!strcasecmp(argv[0], "cfg_radio")) {
		if (argc != 7)
			goto out;
		param.msg_type = QTN_THERMAL_MSG_TYPE_RADIO_CONFIG;

		/* radio id */
		if (local_atou32_range("radio", argv[1], &param.radio_id, print,
				QTN_THERMAL_RADIO_ID_MIN, QTN_THERMAL_RADIO_ID_MAX) < 0)
			return -EINVAL;

		/* stage id */
		if (local_atou32_range("stage", argv[2], &param.arg1, print,
				QTN_THERMAL_STAGE_MIN, QTN_THERMAL_STAGE_MAX) < 0)
			return -EINVAL;

		/* low temp threshold */
		if (local_atou32_range("low threshold", argv[3], &param.arg2, print,
				QTN_THERMAL_TEMP_DEGC_MIN, QTN_THERMAL_TEMP_DEGC_MAX) < 0)
			return -EINVAL;

		/* high temp threshold */
		if (local_atou32_range("high threshold", argv[4], &param.arg3, print,
				QTN_THERMAL_TEMP_DEGC_MIN, QTN_THERMAL_TEMP_DEGC_MAX) < 0)
			return -EINVAL;

		/* number of tx chains */
		if (local_atou32_range("tx chains", argv[5], &param.arg4, print,
				QTN_THERMAL_TX_CHAINS_MIN, QTN_THERMAL_TX_CHAINS_MAX) < 0)
			return -EINVAL;

		/* tx power backoff */
		if (local_atou32_range("tx power backoff", argv[6], &param.arg5, print,
				QTN_THERMAL_TX_PWR_BACKOFF_DB_MIN,
				QTN_THERMAL_TX_PWR_BACKOFF_DB_MAX) < 0)
			return -EINVAL;
	} else if (!strcasecmp(argv[0], "start")) {
		param.msg_type = QTN_THERMAL_MSG_TYPE_START;
	} else if (!strcasecmp(argv[0], "stop")) {
		param.msg_type = QTN_THERMAL_MSG_TYPE_STOP;
	} else if (!strcasecmp(argv[0], "exit")) {
		param.msg_type = QTN_THERMAL_MSG_TYPE_EXIT;
	} else if (!strcasecmp(argv[0], "status")) {
		param.msg_type = QTN_THERMAL_MSG_TYPE_GET_STATUS;
	} else {
		goto out;
	}

	retval = local_thermal_cmd_proc(cb, print, &param);

	if (!strcasecmp(argv[0], "status"))
		if (retval == 0)
			return 0;

	return qcsapi_report_complete(cb, retval);
out:
	qcsapi_report_usage(cb);
	return -EINVAL;
}

CALL_QCSAPI(call_qcsapi_file_path_get_config)
{
	int retval;
	qcsapi_file_path_config cfg = (qcsapi_file_path_config) (cb->generic_param.index);
	char buf[80] = { 0 };

	retval = qcsapi_file_path_get_config(cfg, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_file_path_set_config)
{
	int retval;
	qcsapi_file_path_config cfg = (qcsapi_file_path_config) (cb->generic_param.index);

	retval = qcsapi_file_path_set_config(cfg, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_wifi_macaddr)
{
	qcsapi_mac_addr macaddr;
	int retval;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_wifi_macaddr(macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_create_restricted_bss)
{
	int retval = 0;
	qcsapi_mac_addr macaddr = { 0 };

	if (argc == 1) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_create_restricted_bss(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_create_bss)
{
	int retval = 0;
	qcsapi_mac_addr macaddr = { 0 };

	if (argc == 1) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_create_bss(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_bss)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_bss(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_primary_interface)
{
	int retval = 0;
	char ifname[IFNAMSIZ] = { 0 };
	qcsapi_unsigned_int radio;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_get_primary_interface(radio, ifname, sizeof(ifname));

	return qcsapi_report_str_or_error(cb, retval, ifname);
}

CALL_QCSAPI(call_qcsapi_wifi_get_interface_by_index)
{
	int retval = 0;
	char ifname[IFNAMSIZ] = { 0 };
	qcsapi_unsigned_int if_index = cb->generic_param.index;
	qcsapi_unsigned_int radio;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_get_interface_by_index(radio, if_index, ifname, sizeof(ifname));

	return qcsapi_report_str_or_error(cb, retval, ifname);
}

CALL_QCSAPI(call_qcsapi_wifi_get_interface_by_index_all)
{
	int retval = 0;
	char ifname[IFNAMSIZ] = { 0 };
	qcsapi_unsigned_int if_index = cb->generic_param.index;
	qcsapi_unsigned_int radio = 0;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_get_interface_by_index_all(radio, if_index, ifname,
							sizeof(ifname));

	return qcsapi_report_str_or_error(cb, retval, ifname);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mode)
{
	qcsapi_wifi_mode wifi_mode;
	int retval;

	retval = qcsapi_wifi_get_mode(interface, &wifi_mode);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", wifi_mode_to_string(print, wifi_mode));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_mode)
{
	int retval;
	qcsapi_wifi_mode new_wifi_mode;

	new_wifi_mode = string_to_wifi_mode(argv[0]);
	if (new_wifi_mode == qcsapi_nosuch_mode) {
		print_err(print, "Invalid WiFi mode %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_mode(interface, new_wifi_mode);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_phy_mode)
{
	int retval;
	string_64 buf = { 0 };

	retval = qcsapi_wifi_get_phy_mode(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_phy_mode)
{
	int retval;
	const char *mode = argv[0];

	retval = qcsapi_wifi_set_phy_mode(interface, mode);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_phy_mode_required)
{
	int retval;
	string_64 buf = { 0 };

	retval = qcsapi_wifi_get_phy_mode_required(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_phy_mode_required)
{
	int retval;

	retval = qcsapi_wifi_set_phy_mode_required(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_reload_in_mode)
{
	int retval;
	qcsapi_wifi_mode new_wifi_mode;

	new_wifi_mode = string_to_wifi_mode(argv[0]);

	if (new_wifi_mode == qcsapi_nosuch_mode) {
		print_err(print, "Invalid WiFi mode %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_reload_in_mode(interface, new_wifi_mode);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_radio_rfenable)
{
	qcsapi_unsigned_int enable;
	int retval;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_rfenable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_rfstatus)
{
	int retval;
	qcsapi_unsigned_int rfstatus = 0;

	retval = qcsapi_radio_rfstatus(interface, &rfstatus);

	return qcsapi_report_str_or_error(cb, retval, rfstatus ? "On" : "Off");
}

CALL_QCSAPI(call_qcsapi_wifi_startprod)
{
	int retval;

	retval = qcsapi_wifi_startprod();

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_supported_freq_bands)
{
	int retval;
	string_32 bands = { 0 };

	retval = qcsapi_wifi_get_supported_freq_bands(interface, bands);

	return qcsapi_report_str_or_error(cb, retval, bands);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bw)
{
	qcsapi_unsigned_int bw;
	int retval;

	retval = qcsapi_wifi_get_bw(interface, &bw);

	return qcsapi_report_uint_or_error(cb, retval, bw);
}

CALL_QCSAPI(call_qcsapi_wifi_set_bw)
{
	qcsapi_unsigned_int bw = (qcsapi_unsigned_int) atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_bw(interface, bw);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_BSSID)
{
	qcsapi_mac_addr macaddr;
	int retval;

	retval = qcsapi_wifi_get_BSSID(interface, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(print, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_config_BSSID)
{
	qcsapi_mac_addr macaddr;
	int retval;

	retval = qcsapi_wifi_get_config_BSSID(interface, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(print, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_ssid_get_bssid)
{
	qcsapi_mac_addr macaddr;
	int retval = 0;
	const char *SSID = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_wifi_ssid_get_bssid(interface, SSID, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(print, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_ssid_set_bssid)
{
	qcsapi_mac_addr macaddr;
	int retval = 0;
	const char *SSID = cb->generic_param.parameter_type.SSID;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_ssid_set_bssid(interface, SSID, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_SSID)
{
	qcsapi_SSID ssid;
	int retval;

	memset(ssid, 0, sizeof(ssid));

	retval = qcsapi_wifi_get_SSID(interface, ssid);

	return qcsapi_report_str_or_error(cb, retval, ssid);
}

CALL_QCSAPI(call_qcsapi_wifi_get_SSID2)
{
	qcsapi_SSID2 ssid;
	int retval;
	qcsapi_ssid_fmt fmt;

	memset(ssid, 0, sizeof(ssid));

	retval = qcsapi_wifi_get_SSID2(interface, ssid, &fmt);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, (fmt == qcsapi_ssid_fmt_str) ? "\"%s\"\n" : "%s\n", ssid);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_SSID)
{
	int retval;
	qcsapi_ssid_fmt fmt;
	char *new_ssid;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (argc == 2) {
		if (call_qcsapi_param_name2enum(print, tbl, argv[0], &fmt) < 0)
			return -EINVAL;
		new_ssid = argv[1];
		retval = qcsapi_wifi_set_SSID2(interface, new_ssid, fmt);
	} else {
		new_ssid = argv[0];
		retval = qcsapi_wifi_set_SSID(interface, new_ssid);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_channel)
{
	qcsapi_unsigned_int val;
	int retval;

	retval = qcsapi_wifi_get_channel(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan)
{
	qcsapi_unsigned_int band;
	qcsapi_unsigned_int channel;
	qcsapi_unsigned_int bandwidth;
	int retval;

	retval = qcsapi_wifi_get_chan(interface, &channel, &bandwidth, &band);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s %d %d\n",
		qcsapi_freq_band_table[band].name, channel, bandwidth);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_channel)
{
	qcsapi_unsigned_int channel = atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_channel(interface, channel);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_chan)
{
	qcsapi_unsigned_int channel;
	qcsapi_unsigned_int bw;
	qcsapi_unsigned_int band = qcsapi_freq_band_unknown;
	unsigned int iter;
	int retval;

	errno = 0;

	channel = strtol(argv[0], NULL, 10);
	if (errno)
		return -EINVAL;

	bw = strtol(argv[1], NULL, 10);
	if (errno)
		return -EINVAL;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_freq_band_table); iter++) {
		if (strcasecmp(qcsapi_freq_band_table[iter].name, argv[2]) == 0) {
			band = qcsapi_freq_band_table[iter].index;
			break;
		}
	}
	if (band == qcsapi_freq_band_unknown) {
		print_err(print, "Unknown frequency band\n");
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_chan(interface, channel, bw, band);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_channel_and_bw)
{
	struct qcsapi_data_32bytes chan_bw;
	int retval;

	memset(&chan_bw, 0, sizeof(struct qcsapi_data_32bytes));
	retval = qcsapi_wifi_get_channel_and_bw(interface, &chan_bw);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", chan_bw.data);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_channel_and_bw)
{
	qcsapi_unsigned_int chan;
	qcsapi_unsigned_int bw;
	int retval;

	errno = 0;
	chan = strtol(argv[0], NULL, 10);
	if (errno)
		return -EINVAL;
	bw = strtol(argv[1], NULL, 10);
	if (errno)
		return -EINVAL;

	retval = qcsapi_wifi_set_channel_and_bw(interface, chan, bw);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_wea_cac_en)
{
	qcsapi_unsigned_int enable;
	int retval;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_wea_cac_en(interface, enable);

	return qcsapi_report_complete(cb, retval);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_auto_channel)
{
	int retval;
	char channel_str[QCSAPI_SHORT_PARAM_VAL_LEN] = {0};
	int is_autochan = 0;

	retval = qcsapi_config_get_parameter(interface,
			"channel", channel_str, QCSAPI_SHORT_PARAM_VAL_LEN);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (strcmp(channel_str, "0") == 0)
		is_autochan = 1;
	print_out(print, "%s\n", is_autochan ? "enabled" : "disabled");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_auto_channel)
{
	int retval;
	qcsapi_unsigned_int channel;
	char channel_str[QCSAPI_SHORT_PARAM_VAL_LEN] = {0};
	char *param = argv[0];
	int is_autochan = 0;

	retval = qcsapi_config_get_parameter(interface,
			"channel", channel_str, QCSAPI_SHORT_PARAM_VAL_LEN);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (strcmp(channel_str, "0") == 0)
		is_autochan = 1;

	if (strncmp(param, "enable", strlen(param)) == 0) {
		if (!is_autochan) {
			retval = qcsapi_config_update_parameter(interface, "channel", "0");
			if (retval >= 0) {
				retval = qcsapi_wifi_set_channel(interface, 0);
			}
		}
	} else if (strncmp(param, "disable", strlen(param)) == 0) {
		if (is_autochan) {
			retval = qcsapi_wifi_get_channel(interface, &channel);
			if (retval >= 0) {
				sprintf(channel_str, "%u", channel);
				retval = qcsapi_config_update_parameter(interface,
						"channel", channel_str);
			}
		}
	} else {
		return qcsapi_report_usage(cb);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_chan_off)
{
	qcsapi_unsigned_int channel;
	int retval;
	uint32_t disable = 1;

	if (local_atou32_range("channel", argv[0], &channel, print,
				QCSAPI_MIN_CHANNEL, QCSAPI_MAX_CHANNEL) < 0)
		return -EINVAL;

	if (local_atou_bool("disable", argv[1], &disable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_chan_off(interface, channel, disable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan_pri_inactive)
{
	int retval;
	int i;
	struct ieee80211_inactive_chanlist list_channels;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_256bytes) >=
			sizeof(struct ieee80211_inactive_chanlist));

	memset(&list_channels, 0, sizeof(list_channels));

	retval = qcsapi_wifi_get_chan_pri_inactive(interface,
			(struct qcsapi_data_256bytes *)&list_channels);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 1; i < IEEE80211_CHAN_MAX; i++) {
		if (list_channels.channels[i] & CHAN_PRI_INACTIVE_CFG_USER_OVERRIDE) {
			print_out(print, "%d%s,", i,
				(list_channels.channels[i] & CHAN_PRI_INACTIVE_CFG_AUTOCHAN_ONLY)
					? "(auto)" : "");
		}
	}
	print_out(print, "\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_chan_pri_inactive)
{
	qcsapi_unsigned_int channel = atoi(argv[0]);
	int retval;
	qcsapi_unsigned_int inactive = 1;
	uint32_t flags = 0;

	if (argc >= 2)
		inactive = atoi(argv[1]);

	if (argc == 3) {
		if (!strcasecmp(argv[2], "autochan"))
			flags = QCSAPI_CHAN_PRI_INACTIVE_AUTOCHAN;
		else
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_chan_pri_inactive_ext(interface, channel, inactive, flags);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan_disabled)
{
	int retval;
	struct qcsapi_data_256bytes chan_list;
	uint8_t cnt = 0;
	int loop;

	retval = qcsapi_wifi_get_chan_disabled(interface, &chan_list, &cnt);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (cnt > 0) {
		print_out(print, "%d", chan_list.data[0]);
		for (loop = 1; loop < cnt; loop++)
			print_out(print, ",%d", chan_list.data[loop]);
		print_out(print, "\n");
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_standard)
{
	char ieee_standard[16];
	int retval;

	retval = qcsapi_wifi_get_IEEE_802_11_standard(interface, ieee_standard);

	return qcsapi_report_str_or_error(cb, retval, ieee_standard);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dtim)
{
	int retval;
	qcsapi_unsigned_int dtim;

	retval = qcsapi_wifi_get_dtim(interface, &dtim);

	return qcsapi_report_int_or_error(cb, retval, dtim);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dtim)
{
	qcsapi_unsigned_int dtim = atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_dtim(interface, dtim);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_assoc_limit)
{
	int retval;
	qcsapi_unsigned_int assoc_limit;

	retval = qcsapi_wifi_get_assoc_limit(interface, &assoc_limit);

	return qcsapi_report_int_or_error(cb, retval, assoc_limit);
}

CALL_QCSAPI(call_qcsapi_wifi_set_assoc_limit)
{
	qcsapi_unsigned_int assoc_limit = atoi(argv[0]);
	int retval;
	int i;

	for (i = 0; argv[0][i] != 0; i++) {
		if (isdigit(argv[0][i]) == 0) {
			print_err(print, "Invalid integer parameter %s\n", argv[0]);
			return -EINVAL;
		}
	}

	retval = qcsapi_wifi_set_assoc_limit(interface, assoc_limit);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bss_assoc_limit)
{
	int retval;
	qcsapi_unsigned_int assoc_limit;
	qcsapi_unsigned_int group;

	if (local_atou32("group", argv[0], &group, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_bss_assoc_limit(group, &assoc_limit);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "group assoc_limit %d\n", assoc_limit);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_bss_assoc_limit)
{
	qcsapi_unsigned_int limit;
	qcsapi_unsigned_int group;
	int retval;

	if (local_atou32("group", argv[0], &group, print) < 0)
		return -EINVAL;

	if (local_atou32("limit", argv[1], &limit, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_bss_assoc_limit(group, limit);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_SSID_group_id)
{
	qcsapi_unsigned_int group;
	int retval;

	if (local_atou32("group", argv[0], &group, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_SSID_group_id(interface, group);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_SSID_group_id)
{
	int retval;
	qcsapi_unsigned_int group;

	retval = qcsapi_wifi_get_SSID_group_id(interface, &group);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "group_id %d\n", group);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_SSID_assoc_reserve)
{
	qcsapi_unsigned_int group;
	qcsapi_unsigned_int value;
	int retval;

	if (local_atou32("group", argv[0], &group, print) < 0)
		return -EINVAL;

	if (local_atou32("value", argv[1], &value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_SSID_assoc_reserve(group, value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_SSID_assoc_reserve)
{
	qcsapi_unsigned_int group;
	qcsapi_unsigned_int value;
	int retval;

	if (local_atou32("group", argv[0], &group, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_SSID_assoc_reserve(group, &value);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "group assoc reserved value : %u\n", value);

	return 0;
}

CALL_QCSAPI(call_qcsapi_interface_get_status)
{
	char buf[16];
	int retval;

	retval = qcsapi_interface_get_status(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_interface_set_ip4)
{
	uint32_t if_param_val;
	uint32_t if_param_val_ne;
	int retval;
	char *if_param;

	if_param = argv[0];

	if (inet_pton(AF_INET, argv[1], &if_param_val) != 1) {
		print_err(print, "invalid IPv4 argument %s\n", argv[1]);
		return -EINVAL;
	}
	if_param_val_ne = htonl(if_param_val);

	retval = qcsapi_interface_set_ip4(interface, if_param, if_param_val_ne);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_interface_get_ip4)
{
	string_64 if_param_val;
	int retval;
	char *if_param = NULL;

	if (argc == 1)
		if_param = argv[0];

	retval = qcsapi_interface_get_ip4(interface, if_param, if_param_val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", if_param_val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_interface_set_mtu)
{
	int retval;
	uint32_t mtu;

	if (local_atou32("mtu", argv[0], &mtu, print) < 0)
		return -EINVAL;

	retval = qcsapi_interface_set_mtu(interface, mtu);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_interface_get_mtu)
{
	int retval;
	uint32_t mtu;

	retval = qcsapi_interface_get_mtu(interface, &mtu);

	return qcsapi_report_uint_or_error(cb, retval, mtu);
}

CALL_QCSAPI(call_qcsapi_wifi_get_list_channels)
{
	int retval;
	char *buf;

	buf = local_malloc(cb, sizeof(string_1024));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_list_channels(interface, buf);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

#define LOCAL_PRINT_CHARS_PER_CHANNEL	5

static int local_print_channel_list(const call_bundle *cb, const uint8_t *p_ch_list,
					const size_t size)
{
	qcsapi_output *print = cb->output;
	char *buf;
	int buf_size = size * NBBY * LOCAL_PRINT_CHARS_PER_CHANNEL;
	char *p_buf;
	int indx;
	int len;
	int used_len = 0;
	int free_len;
	int channels_2g = 0;
	int channels_5g = 0;
	int channels_6g = 0;
	int channels_total = 0;

	buf = local_malloc(cb, buf_size);
	if (!buf)
		return -ENOMEM;

	p_buf = buf;

	for (indx = 0; indx < (size * NBBY); indx++) {
		if (isset(p_ch_list, indx)) {
			free_len = buf_size - used_len;

			if (IC_IEEE_IS_CHAN_IN_2G(indx)) {
				if (channels_2g == 0) {
					len = snprintf(p_buf, free_len, "%s", "2.4GHz: ");
					p_buf += len;
					used_len += len;
				}
				channels_2g++;
				channels_total++;
			} else if (IC_IEEE_IS_CHAN_IN_5G(indx)) {
				if (channels_5g == 0) {
					if (channels_total)
						snprintf(p_buf - 1, free_len, "\n");
					len = snprintf(p_buf, free_len, "%s", "5GHz: ");
					p_buf += len;
					used_len += len;
				}
				channels_5g++;
				channels_total++;
			} else if (IC_IEEE_IS_CHAN_IN_6G(indx)) {
				if (channels_6g == 0) {
					if (channels_total)
						snprintf(p_buf - 1, free_len, "\n");
					len = snprintf(p_buf, free_len, "%s", "6GHz: ");
					p_buf += len;
					used_len += len;
				}
				channels_6g++;
				channels_total++;
			} else {
				len = snprintf(p_buf, free_len, "%s",
						"\nERROR: Unknown channel in the list ");
				p_buf += len;
				used_len += len;
				break;
			}

			free_len = buf_size - used_len;
			len = snprintf(p_buf, free_len, "%d,", (indx & IC_IEEE_CHANNEL_MASK));
			p_buf += len;
			used_len += len;
		}
	}

	buf[used_len ? used_len - 1 : 0] = '\0';

	print_out(print, "%s\n", buf);

	free(buf);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_s_radio_chan_off)
{
	int retval;
	struct qcsapi_data_256bytes chans;

	memset(&chans, 0, sizeof(chans));

	retval = qcsapi_wifi_get_chan_list(interface, &chans, qcsapi_chlist_flag_ocac_off);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	return local_print_channel_list(cb, chans.data, sizeof(chans));
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan_list)
{
	int retval;
	struct qcsapi_data_256bytes chan_list;
	uint32_t flags = 0;

	if (argc == 0) {
		flags |= qcsapi_chlist_flag_available;
	} else {
		if (!strcasecmp(argv[0], "available"))
			flags |= qcsapi_chlist_flag_available;
		else if (!strcasecmp(argv[0], "disabled"))
			flags |= qcsapi_chlist_flag_disabled;
		else if (!strcasecmp(argv[0], "scan"))
			flags |= qcsapi_chlist_flag_scan;
		else if (!strcasecmp(argv[0], "active"))
			flags |= qcsapi_chlist_flag_active;
		else if (!strcasecmp(argv[0], "ocac_off"))
			flags |= qcsapi_chlist_flag_ocac_off;
		else
			return qcsapi_report_usage(cb);
	}

	memset(&chan_list, 0, sizeof(chan_list));
	retval = qcsapi_wifi_get_chan_list(interface, &chan_list, flags);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	return local_print_channel_list(cb, chan_list.data, sizeof(chan_list));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_chan_list)
{
	struct qcsapi_data_256bytes chan_list;
	int retval;
	qcsapi_unsigned_int band = qcsapi_freq_band_unknown;
	uint32_t chan_list_flags = 0;
	uint32_t control_flag = 0;
	uint32_t len;
	uint32_t i;

	if (!strcasecmp(argv[0], "disabled"))
		chan_list_flags |= qcsapi_chlist_flag_disabled;
	else if (!strcasecmp(argv[0], "scan"))
		chan_list_flags |= qcsapi_chlist_flag_scan;
	else
		return qcsapi_report_usage(cb);

	for (i = 0; i < ARRAY_SIZE(qcsapi_freq_band_table); i++) {
		if (strcasecmp(qcsapi_freq_band_table[i].name, argv[1]) == 0) {
			band = qcsapi_freq_band_table[i].index;
			break;
		}
	}
	if (band == qcsapi_freq_band_unknown) {
		print_err(print, "Unknown frequency band\n");
		return -EINVAL;
	}

	memset(&chan_list, 0, sizeof(chan_list));
	retval = local_string_to_u8_list(print, argv[2], chan_list.data, &len);
	if (retval < 0) {
		print_err(print, "Invalid channel in list\n");
		return -EINVAL;
	}

	if (local_atou_bool("set", argv[3], &control_flag, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_chan_list(interface, &chan_list, len,
					chan_list_flags, band, control_flag);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan_usable)
{
	int retval;
	struct qcsapi_data_256bytes chan_list = { {0} };

	COMPILE_TIME_ASSERT(sizeof(chan_list) >= IEEE80211_CHAN_MAX / NBBY);

	retval = qcsapi_wifi_get_chan_list(interface, &chan_list, qcsapi_chlist_flag_active);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	return local_print_channel_list(cb, chan_list.data, sizeof(chan_list));

}

CALL_QCSAPI(call_qcsapi_wifi_get_supp_chans)
{
	int retval = 0;
	static string_1024 buf;
	qcsapi_mac_addr macaddr;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_supp_chans(interface, macaddr, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mode_switch)
{
	uint8_t wifi_mode;
	int retval;

	retval = qcsapi_wifi_get_mode_switch(&wifi_mode);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%x\n", wifi_mode);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_option)
{
	int wifi_option;
	int retval;
	qcsapi_option_type option = cb->generic_param.parameter_type.option;

	retval = qcsapi_wifi_get_option(interface, option, &wifi_option);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (wifi_option == 0)
		print_out(print, "FALSE\n");
	else
		print_out(print, "TRUE\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_dpp_parameter)
{
	int i;
	int j;
	int retval;
	char buf[QCSAPI_DPP_MAX_BUF_SIZE];
	struct qcsapi_dpp_set_parameters dpp_params;
	enum qcsapi_dpp_cmd_param_type cmd = cb->generic_param.parameter_type.dpp_param_type;

	memset(&dpp_params, 0, sizeof(dpp_params));

	if (cmd == qcsapi_dpp_cmd_get_config) {
		if (argc != 1)
			return qcsapi_report_usage(cb);
		strncpy(dpp_params.param[0].key, argv[0], sizeof(dpp_params.param[0].key) - 1);
	} else {
		if (argc < 2 || argc > (2 * ARRAY_SIZE(dpp_params.param)) || !!(argc % 2))
			return qcsapi_report_usage(cb);

		for (i = 0, j = 0; i < ARRAY_SIZE(dpp_params.param) && j < argc; i++, j += 2) {
			strncpy(dpp_params.param[i].key, argv[j],
				sizeof(dpp_params.param[i].key) - 1);
			strncpy(dpp_params.param[i].value, argv[j + 1],
				sizeof(dpp_params.param[i].value) - 1);
		}
	}

	retval = qcsapi_wifi_dpp_parameter(interface, cmd, &dpp_params, buf, sizeof(buf));

	/* Interpret successful set/dpp_command response */
	if (retval >= 0 && (strcasecmp(buf, "OK") == 0)) {
		if (verbose_flag >= 0)
			snprintf(buf, sizeof(buf), "%s", "complete");
		else
			buf[0] = '\0';
	}

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_parameter)
{
	int retval;
	int val;
	qcsapi_wifi_param_type type = cb->generic_param.parameter_type.wifi_param_type;

	retval = qcsapi_wifi_get_parameter(interface, type, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_parameter)
{
	int retval;
	int val;
	qcsapi_wifi_param_type type = cb->generic_param.parameter_type.wifi_param_type;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_wifi_param;
	const char *param_name = qcsapi_param_enum2name(tbl, type);

	if (local_atoi32(param_name, argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_parameter(interface, type, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_board_parameter)
{
	int retval = 0;
	qcsapi_board_parameter_type boardparam = cb->generic_param.parameter_type.board_param;
	string_64 buf;
	qcsapi_unsigned_int radio = 0;

	if (argc > 0) {
		if (local_atou32("radio", argv[0], &radio, print) < 0)
			return -EINVAL;
	}

	memset(buf, 0, sizeof(buf));
	retval = qcsapi_radio_get_board_parameter(radio, boardparam, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_noise)
{
	int retval;
	int noise;

	retval = qcsapi_wifi_get_noise(interface, &noise);

	return qcsapi_report_int_or_error(cb, retval, noise);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rssi_by_chain)
{
	int retval;
	int rssi;
	int rf_chain;

	if (local_atoi32("chain number", argv[0], &rf_chain, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_rssi_by_chain(interface, rf_chain, &rssi);

	return qcsapi_report_int_or_error(cb, retval, rssi);
}

CALL_QCSAPI(call_qcsapi_wifi_get_avg_snr)
{
	int retval;
	int buf;

	retval = qcsapi_wifi_get_avg_snr(interface, &buf);

	return qcsapi_report_int_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_option)
{
	int enable;
	int retval;
	qcsapi_option_type option = cb->generic_param.parameter_type.option;

	if (local_atoi_bool_legacy(argv[0], (unsigned int *)&enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_option(interface, option, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rates)
{
	int retval;
	qcsapi_rate_type rates_type = cb->generic_param.parameter_type.typeof_rates;
	char *buf;

	buf = local_malloc(cb, sizeof(string_2048));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_rates(interface, rates_type, buf);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_set_rates)
{
	int retval;
	struct qcsapi_data_256bytes current_rates;
	qcsapi_rate_type rates_type = cb->generic_param.parameter_type.typeof_rates;
	int rates_len = 0;
	int tmp_len = 0;
	int i;

	memset(&current_rates, 0, sizeof(current_rates));

	for (i = 0; i < argc; i++) {
		tmp_len = strlen(argv[i]) + 1;

		if (rates_len + tmp_len > sizeof(current_rates.data)) {
			print_err(print, "Invalid input rates - the Maximum length of rates list is %d\n",
				sizeof(current_rates.data));
			return -EINVAL;
		}

		memcpy(current_rates.data + rates_len, argv[i], tmp_len);
		rates_len += tmp_len;
	}

	retval = qcsapi_wifi_set_wifi_rates(interface, rates_type, &current_rates, argc);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_max_bitrate)
{
	int retval;
	char buf[QCSAPI_MAX_BITRATE_STR_MIN_LEN + 1] = { 0 };

	retval = qcsapi_get_max_bitrate(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_beacon_type)
{
	char buf[16];
	int retval;

	retval = qcsapi_wifi_get_beacon_type(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_beacon_type)
{
	int retval;

	retval = qcsapi_wifi_set_beacon_type(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_beacon_interval)
{
	uint32_t val = 0;
	int retval;

	retval = qcsapi_wifi_get_beacon_interval(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_beacon_interval)
{
	int retval;
	qcsapi_unsigned_int interval;

	if (local_atou32("interval", argv[0], &interval, print) < 0)
		return -EINVAL;
	if (interval > BEACON_INTERVAL_WARNING_LOWER_LIMIT
			&& interval < BEACON_INTERVAL_WARNING_UPPER_LIMIT) {
		print_out(print, "Beacon intervals less than %dms may reduce network performance\n",
			BEACON_INTERVAL_WARNING_UPPER_LIMIT);
	}

	retval = qcsapi_wifi_set_beacon_interval(interface, interval);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_list_regulatory_regions)
{
	string_2048 buf;
	int retval;

	retval = qcsapi_regulatory_get_list_regulatory_regions_ext(buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_regulatory_tx_power)
{
	int retval;
	uint32_t chan;
	const char *region = argv[1];
	int tx_power = 0;
	uint32_t bw = 0;

	if (local_atou32("channel", argv[0], &chan, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_bw(interface, &bw);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	retval = qcsapi_regulatory_get_regulatory_tx_power(interface,
					chan, region, &tx_power);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	/* bit15:8 of tx_power > 0, bw160M tx power using imbalance way */
	if (bw == qcsapi_bw_160MHz && (tx_power & 0xFF00)) {
		print_out(print, "bandwidth 160MHz using imbalanced regulatory Tx power\n");
		print_out(print, "lower 80MHz band: %d\n", tx_power & 0xFF);
		print_out(print, "higher 80MHz band: %d\n", (tx_power & 0xFF00) >> 8);
	} else {
		print_out(print, "%d\n", tx_power);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_configured_tx_power)
{
	uint32_t chan;
	const char *region = argv[1];
	uint32_t bw = 0;
	uint32_t bf_on = 0;
	uint32_t ss = 1;
	int retval;
	int tx_power = 0;

	if (local_atou32("channel", argv[0], &chan, print) < 0)
		return -EINVAL;

	if (argc < 3) {
		retval = qcsapi_wifi_get_bw(interface, &bw);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
	} else {
		if (local_atou32("bandwidth", argv[2], &bw, print) < 0)
			return -EINVAL;
	}

	if (argc == 4)
		return qcsapi_report_usage(cb);

	if (argc == 5) {
		if (local_atou32("bf", argv[3], &bf_on, print) < 0)
			return -EINVAL;
		if (local_atou32("ss", argv[4], &ss, print) < 0)
			return -EINVAL;
		ss = atoi(argv[4]);
	}

	retval = qcsapi_regulatory_get_configured_tx_power_ext(interface, chan, region, bw,
								bf_on, ss, &tx_power);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if ((tx_power >= 0) && ((tx_power >> 8) & 0xFF))
		print_out(print, "%d.%d\n", (tx_power & 0xFF),
				((tx_power >> 8) & 0xFF));
	else
		print_out(print, "%d\n", tx_power);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_regulatory_channel)
{

	int retval;
	uint32_t chan;
	const char *region;
	uint32_t offset = 0;

	if (local_atou32("channel", argv[0], &chan, print) < 0)
		return -EINVAL;

	region = argv[1];

	if (argc == 3) {
		if (local_atou32("offset", argv[2], &offset, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_regulatory_set_regulatory_channel(interface, chan, region, offset);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_regulatory_region)
{
	int retval;

	retval = qcsapi_regulatory_set_regulatory_region(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_restore_regulatory_tx_power)
{
	int retval;

	retval = qcsapi_regulatory_restore_regulatory_tx_power(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_regulatory_region)
{
	int retval;
	char buf[6];

	retval = qcsapi_wifi_get_regulatory_region(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_overwrite_country_code)
{
	int retval;
	const char *curr_name = argv[0];
	const char *new_name = argv[1];

	retval = qcsapi_regulatory_overwrite_country_code(interface, curr_name, new_name);
	if (retval == -qcsapi_configuration_error) {
		print_err(print, "Country code cannot be overridden on a provisional board\n");
		return retval;
	}
	if (retval == -qcsapi_region_not_supported) {
		print_err(print, "Current region is not %s\n", curr_name);
		return retval;
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_country_code)
{
	int retval;
	string_16 buf;

	retval = qcsapi_regulatory_get_country_code(interface, buf);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", buf);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_list_regulatory_channels)
{
	int retval = 0;
	const char *region = argv[0];
	qcsapi_unsigned_int bw = 0;
	char *ifname = NULL;
	char *buf;

	if (argc == 3)
		ifname = argv[2];

	if (argc == 1 || ((argc >= 2) && !strcmp(argv[1], "current"))) {
		retval = qcsapi_wifi_get_bw(ifname, &bw);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
	} else if (argc >= 2) {
		if (local_atou32("bandwidth", argv[1], &bw, print) < 0)
			return -EINVAL;
	}

	buf = local_malloc(cb, sizeof(string_2048));
	if (!buf)
		return -ENOMEM;

	if (ifname)
		retval = qcsapi_regulatory_get_list_regulatory_channels_if(ifname,
								region, bw, -1, buf);
	else
		retval = qcsapi_regulatory_get_list_regulatory_channels(region, bw, buf);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_list_regulatory_bands)
{
	char *ifname = NULL;
	int retval;
	const char *region = argv[0];
	char *buf;

	if (argc > 1)
		ifname = argv[1];

	buf = local_malloc(cb, sizeof(string_2048));
	if (!buf)
		return -ENOMEM;

	if (ifname)
		retval = qcsapi_regulatory_get_list_regulatory_bands_if(ifname, region, buf);
	else
		retval = qcsapi_regulatory_get_list_regulatory_bands(region, buf);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_regulatory_db_version)
{
	int retval;
	int retval1 = 0;
	int version = 0;
	int index = 0;
	char ch = 'v';
	int *p_qcsapi_retval = &retval;
	const char *format[2] = { "%c%d", "0%c%x" };

	if (argc > 0) {
		if (local_atoi32("index", argv[0], &index, print) < 0)
			return -EINVAL;
		ch = 'x';
	}

	do {
		*p_qcsapi_retval = qcsapi_regulatory_get_db_version(&version, index++);
		if (retval == -1 || retval1 < 0)
			break;

		print_out(print, format[argc > 0], ch, version);

		ch = '.';
		p_qcsapi_retval = &retval1;
	} while (argc == 0 && retval >= 0);

	if (retval == -1)
		print_out(print, "database not available");

	print_out(print, "\n");

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_set_regulatory_tx_power_cap)
{
	qcsapi_unsigned_int capped = atoi(argv[0]);
	int retval;

	retval = qcsapi_regulatory_apply_tx_power_cap(capped);

	return qcsapi_report_complete(cb, retval);
}

unsigned long local_reg_chan_txpower_nss_to_ssidx(unsigned long input)
{
	return input - 1;
}

unsigned long local_reg_chan_txpower_bw_to_bwidx(unsigned long input)
{
	unsigned long ret;

	switch (input) {
	case 20:
		ret = QCSAPI_PWR_BW_20M;
		break;
	case 40:
		ret = QCSAPI_PWR_BW_40M;
		break;
	case 80:
		ret = QCSAPI_PWR_BW_80M;
		break;
	case 160:
		ret = QCSAPI_PWR_BW_160M;
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static unsigned int regulatory_chan_txpower_input_parse(char *input_str,
		const unsigned int num_bits, unsigned long (*transform_func) (unsigned long input))
{
	char *cur;
	unsigned int bitmap = 0;
	unsigned long tmp_val;
	char *saved_p;

	if (!input_str)
		goto out;

	cur = strtok_r(input_str, ",", &saved_p);
	if (!cur)
		goto out;

	if (*cur == '*' && *(cur + 1) == '\0') {
		bitmap |= (1 << num_bits) - 1;
	} else {
		char *endptr = NULL;

		do {
			tmp_val = strtoul(cur, &endptr, 10);
			if (transform_func)
				tmp_val = transform_func(tmp_val);

			if (errno || *endptr != '\0' || (tmp_val >= num_bits)) {
				bitmap = 0;
				goto out;
			}

			bitmap |= (1 << tmp_val);
			cur = strtok_r(NULL, ",", &saved_p);
		} while (cur);
	}
out:
	return bitmap;
}

static int regulatory_chan_txpower_input_parse_bitmap(char *input, qcsapi_output *print,
		uint8_t *chan, unsigned int *ss_map, unsigned int *bf_map,
		unsigned int *fem_pri_map, unsigned int *bw_map)
{
	char *endptr;
	char *cur;

	cur = strtok(input, ":");
	if (!cur)
		goto error;

	errno = 0;
	*chan = strtoul(cur, &endptr, 10);
	if (errno || *endptr != '\0')
		goto error;

	cur = strtok(NULL, ":");
	if (!cur) {
		/* Specifying channel number only equals to specifying <chan:*:*:*:*> */
		*ss_map = (1 << QCSAPI_PWR_IDX_SS_NUM) - 1;
		*bf_map = (1 << QCSAPI_PWR_IDX_BF_NUM) - 1;
		*fem_pri_map = (1 << QCSAPI_PWR_IDX_FEM_PRIPOS_NUM) - 1;
		*bw_map = (1 << QCSAPI_PWR_BW_NUM) - 1;
		return 0;
	}

	*ss_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_IDX_SS_NUM, &local_reg_chan_txpower_nss_to_ssidx);
	if (!*ss_map) {
		print_err(print, "Bad NSS\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*bf_map = regulatory_chan_txpower_input_parse(cur, QCSAPI_PWR_IDX_BF_NUM, NULL);
	if (!*bf_map) {
		print_err(print, "Bad BF\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*fem_pri_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_IDX_FEM_PRIPOS_NUM, NULL);
	if (!*fem_pri_map) {
		print_err(print, "Bad FEM/PRI\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*bw_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_BW_NUM, &local_reg_chan_txpower_bw_to_bwidx);
	if (!*bw_map) {
		print_err(print, "Bad BW\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (cur != NULL && cur[0] != '\0')
		goto error;

	return 0;

error:
	return -1;
}

CALL_QCSAPI(call_qcsapi_reg_chan_txpower_set)
{
	int retval;
	int power;
	int power_decimal;
	char *cur;
	unsigned int fem_pri_map;
	unsigned int bf_map;
	unsigned int ss_map;
	unsigned int bw_map;
	unsigned int fem_pri;
	unsigned int bf;
	unsigned int ss;
	unsigned int bw;
	struct qcsapi_chan_tx_powers_with_decimal_info info;
	qcsapi_chan_powers *pwrs = (qcsapi_chan_powers *) &info.maxpwr;
	qcsapi_chan_powers *pwrs_decimal = (qcsapi_chan_powers *) &info.maxpwr_decimal;
	unsigned int arg_idx = 0;
	qcsapi_txpwr_value_type set_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;

	if (argc != 2 && argc != 4) {
		goto usage;
	}

	while (arg_idx < argc) {
		if (argv[arg_idx][0] != '-')
			break;

		if (!strcmp(argv[arg_idx], "-t")) {
			if (++arg_idx == argc) {
				goto usage;
			}

			if (!strcmp(argv[arg_idx], "dntx")) {
				set_type = QCSAPI_PWR_VALUE_TYPE_DNTX;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				goto usage;
			}
		} else {
			print_err(print, "Invalid option %s\n", argv[arg_idx]);
			goto usage;
		}

		++arg_idx;
	}

	if ((arg_idx + 2) != argc) {
		goto usage;
	}

	memset(&info, 0, sizeof(info));

	if (regulatory_chan_txpower_input_parse_bitmap(argv[arg_idx], print, &info.channel, &ss_map,
					&bf_map, &fem_pri_map, &bw_map))
		goto usage;

	cur = strtok(argv[++arg_idx], ",");

	for (ss = 0; ss < QCSAPI_PWR_IDX_SS_NUM; ++ss) {
		if (!(ss_map & (1 << ss)))
			continue;

		for (bf = 0; bf < QCSAPI_PWR_IDX_BF_NUM; ++bf) {
			if (!(bf_map & (1 << bf)))
				continue;

			for (fem_pri = 0; fem_pri < QCSAPI_PWR_IDX_FEM_PRIPOS_NUM; ++fem_pri) {
				if (!(fem_pri_map & (1 << fem_pri)))
					continue;

				for (bw = 0; bw < QCSAPI_PWR_BW_NUM; ++bw) {
					if (!(bw_map & (1 << bw)))
						continue;

					if (!cur) {
						print_err(print, "Not enough PWR values\n");
						goto usage;
					}

					if (strstr(cur, "."))
						sscanf(cur, "%d.%d", &power, &power_decimal);
					else {
						sscanf(cur, "%d", &power);
						power_decimal = 0;
					}

					if (power > INT8_MAX || power < INT8_MIN
							|| power_decimal > INT8_MAX
							|| power_decimal < 0)
						goto usage;

					cur = strtok(NULL, ",");

					(*pwrs)[fem_pri][bf][ss][bw] = (int8_t) power;
					(*pwrs_decimal)[fem_pri][bf][ss][bw] =
							(int8_t) power_decimal;
				}
			}
		}
	}

	if (cur) {
		print_err(print, "Too many PWR values\n");
		goto usage;
	}

	retval = qcsapi_regulatory_chan_txpower_with_decimal_set(interface, &info,
			set_type);

	return qcsapi_report_complete(cb, retval);

usage:
	qcsapi_report_usage(cb);
	return -EINVAL;
}

CALL_QCSAPI(call_qcsapi_reg_chan_txpower_path_get)
{
	char buf[80] = { 0 };
	uint32_t path_len;
	int retval;

	path_len = (uint32_t) sizeof(buf);

	retval = qcsapi_regulatory_chan_txpower_path_get(interface, buf, path_len);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

static inline void reg_chan_txpower_get_print_header(qcsapi_output *print, unsigned int print_map)
{
	unsigned int bw;

	print_out(print, "Ch:ss:bf:fp ");

	for (bw = QCSAPI_PWR_BW_160M; print_map; --bw) {
		if (!(print_map & (1 << bw)))
			continue;

		print_map &= ~(1 << bw);
		switch (bw) {
		case QCSAPI_PWR_BW_160M:
			print_out(print, "160M ");
			break;
		case QCSAPI_PWR_BW_80M:
			print_out(print, "80M  ");
			break;
		case QCSAPI_PWR_BW_40M:
			print_out(print, "40M  ");
			break;
		case QCSAPI_PWR_BW_20M:
			print_out(print, "20M");
			break;
		}
	}

	print_out(print, "\n");
}

CALL_QCSAPI(call_qcsapi_reg_chan_txpower_get)
{
	int retval;
	unsigned int arg_idx = 0;
	int print_hdr = 1;
	unsigned int fem_pri_map;
	unsigned int bf_map;
	unsigned int ss_map;
	unsigned int bw_map;
	unsigned int fem_pri;
	unsigned int bf;
	unsigned int ss;
	unsigned int bw;
	struct qcsapi_chan_tx_powers_with_decimal_info info;
	qcsapi_chan_powers *pwrs = (qcsapi_chan_powers *) &info.maxpwr;
	qcsapi_chan_powers *pwrs_decimal = (qcsapi_chan_powers *) &info.maxpwr_decimal;
	qcsapi_txpwr_value_type report_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;
	int dntx = 0;

	while (arg_idx < argc) {
		if (argv[arg_idx][0] != '-')
			break;

		if (!strcmp(argv[arg_idx], "-n")) {
			print_hdr = 0;
		} else if (!strcmp(argv[arg_idx], "-f")) {
			if (++arg_idx == argc)
				return qcsapi_report_usage(cb);
			if (!strcmp(argv[arg_idx], "active")) {
				report_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;
			} else if (!strcmp(argv[arg_idx], "configured")) {
				report_type = QCSAPI_PWR_VALUE_TYPE_CONFIGURED;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				return -EINVAL;
			}
		} else if (!strcmp(argv[arg_idx], "-t")) {
			if (++arg_idx == argc)
				return qcsapi_report_usage(cb);
			if (!strcmp(argv[arg_idx], "dntx")) {
				dntx = 1;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				return -EINVAL;
			}

		} else {
			print_err(print, "Invalid option %s\n", argv[arg_idx]);
				return -EINVAL;
		}

		++arg_idx;
	}

	if (dntx == 1) {
		if (report_type == QCSAPI_PWR_VALUE_TYPE_ACTIVE)
			report_type = QCSAPI_PWR_VALUE_TYPE_DNTX;
		else
			report_type = QCSAPI_PWR_VALUE_TYPE_DNTX_CONFIGURED;
	}

	if ((arg_idx + 1) != argc)
		return qcsapi_report_usage(cb);

	memset(&info, 0, sizeof(info));

	if (regulatory_chan_txpower_input_parse_bitmap(argv[arg_idx], print, &info.channel, &ss_map,
					&bf_map, &fem_pri_map, &bw_map))
		return qcsapi_report_usage(cb);

	retval = qcsapi_regulatory_chan_txpower_with_decimal_get(interface, &info, report_type);
	if (retval < 0) {
		report_qcsapi_error(cb, retval);
		return -EINVAL;
	}

	if (print_hdr)
		reg_chan_txpower_get_print_header(print, bw_map);

	for (fem_pri = 0; fem_pri < QCSAPI_PWR_IDX_FEM_PRIPOS_NUM; ++fem_pri) {
		if (!(fem_pri_map & (1 << fem_pri)))
			continue;

		for (bf = 0; bf < QCSAPI_PWR_IDX_BF_NUM; ++bf) {
			if (!(bf_map & (1 << bf)))
				continue;

			for (ss = 0; ss < QCSAPI_PWR_IDX_SS_NUM; ++ss) {
				if (!(ss_map & (1 << ss)))
					continue;

				print_out(print, "%3d:%d:%d:%d ",
						info.channel, ss + 1, bf, fem_pri);

				for (bw = QCSAPI_PWR_BW_160M; bw < QCSAPI_PWR_BW_NUM; --bw) {
					if (!(bw_map & (1 << bw)))
						continue;

					if ((*pwrs_decimal)[fem_pri][bf][ss][bw] == 0)
						print_out(print, "%4d ",
								(*pwrs)[fem_pri][bf][ss][bw]);
					else
						print_out(print, "%2d.%d ",
								(*pwrs)[fem_pri][bf][ss][bw],
								(*pwrs_decimal)[fem_pri][bf][ss]
								[bw]);
				}

				print_out(print, "\n");
			}
		}
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_power_selection)
{
	uint32_t val;
	int retval;

	retval = qcsapi_wifi_get_power_selection(&val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_power_selection)
{
	qcsapi_unsigned_int power_selection = atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_power_selection(power_selection);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_congestion_idx)
{
	int retval;
	int val;

	retval = qcsapi_wifi_get_congestion_index(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_supported_tx_power_levels)
{
	int retval;
	string_128 buf = "";

	retval = qcsapi_wifi_get_supported_tx_power_levels(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_current_tx_power_level)
{
	int retval;
	uint32_t val = 0;

	retval = qcsapi_wifi_get_current_tx_power_level(interface, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d\n", (int)val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_power_constraint)
{
	int retval;
	int val;

	if (local_atoi32("power constraint", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_power_constraint(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_power_constraint)
{
	qcsapi_unsigned_int val;
	int retval;

	retval = qcsapi_wifi_get_power_constraint(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_tpc_interval)
{
	int retval;
	int val;

	if (local_atoi32("interval", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_tpc_interval(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tpc_interval)
{
	qcsapi_unsigned_int val;
	int retval;

	retval = qcsapi_wifi_get_tpc_interval(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scan_chk_inv)
{
	int retval;
	int interval = 0;

	if (local_atoi32_range("interval", argv[0], &interval, print, 0, 24 * 60 * 60) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scan_chk_inv(interface, interval);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scan_chk_inv)
{
	int val = 0;
	int retval;

	retval = qcsapi_wifi_get_scan_chk_inv(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

static void local_display_assoc_records(qcsapi_output *print, const struct qcsapi_assoc_records *p)
{
	int iter;
	char mac_addr_string[24];

	for (iter = 0; iter < QCSAPI_ASSOC_MAX_RECORDS; iter++) {
		if (p->timestamp[iter] <= 0)
			return;
		snprintf(&mac_addr_string[0], sizeof(mac_addr_string), MACFILTERINGMACFMT,
				p->addr[iter][0],
				p->addr[iter][1],
				p->addr[iter][2],
				p->addr[iter][3],
				p->addr[iter][4],
				p->addr[iter][5]);
		print_out(print, "%s: %d\n", &mac_addr_string[0],
				(int)p->timestamp[iter]);
	}
}

CALL_QCSAPI(call_qcsapi_wifi_get_assoc_records)
{
	int retval;
	int reset_flag = 0;
	struct qcsapi_assoc_records assoc_records;
	struct qcsapi_assoc_records *p = &assoc_records;

	if (argc > 0) {
		if (!isdigit(argv[0][0])) {
			print_err(print, "Reset flag must be numeric\n");
			return -EINVAL;
		}

		reset_flag = atoi(argv[0]);
	}

	if (argc > 1 && strcmp(argv[1], "NULL") == 0)
		p = NULL;

	retval = qcsapi_wifi_get_assoc_records(interface, reset_flag, p);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);
	local_display_assoc_records(print, &assoc_records);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_list_dfs_channels)
{
	static string_1024 buf;
	int retval;
	const char *region = argv[0];
	int dfs_flag = 0;
	qcsapi_unsigned_int bw = 0;
	const char *ifname = NULL;

	if (local_atoi_bool("DFS flag", argv[1], &dfs_flag, print) < 0)
		return -EINVAL;

	if (argc >= 4)
		ifname = argv[3];

	if (argc == 2 || ((argc >= 3) && !strcmp(argv[2], "current"))) {
		retval = qcsapi_wifi_get_bw(ifname, &bw);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
	} else if (argc >= 3) {
		if (local_atou32("bandwidth", argv[2], &bw, print) < 0)
			return -EINVAL;
	}

	if (ifname)
		retval = qcsapi_regulatory_get_list_regulatory_channels_if(ifname,
				region, bw, dfs_flag, buf);
	else
		retval = qcsapi_regulatory_get_list_DFS_channels(region, dfs_flag, bw, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_is_channel_dfs)
{
	int retval;
	const char *region = NULL;
	int dfs_flag = 0;
	qcsapi_unsigned_int channel;
	const char *ifname = NULL;

	if (strcmp(argv[0], "NULL") != 0)
		region = argv[0];

	channel = (qcsapi_unsigned_int) atoi(argv[1]);

	if (argc == 3)
		ifname = argv[2];

	if (ifname)
		retval = qcsapi_regulatory_is_channel_DFS_if(ifname, region, channel, &dfs_flag);
	else
		retval = qcsapi_regulatory_is_channel_DFS(region, channel, &dfs_flag);

	return qcsapi_report_int_or_error(cb, retval, dfs_flag);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_alt_channel)
{
	qcsapi_unsigned_int val;
	int retval;

	retval = qcsapi_wifi_get_DFS_alt_channel(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_alt_channel)
{
	qcsapi_unsigned_int dfs_alt_chan = atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_DFS_alt_channel(interface, dfs_alt_chan);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_reentry)
{
	int retval;

	retval = qcsapi_wifi_start_dfs_reentry(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_radar_chain)
{
	qcsapi_unsigned_int radar_blockid = atoi(argv[0]);
	qcsapi_unsigned_int radar_chain_selection = atoi(argv[1]);
	int retval;

	retval = qcsapi_wifi_set_radar_chain(radar_blockid, radar_chain_selection);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_zsdfs_param)
{
	int retval;
	qcsapi_zsdfs_param param = cb->generic_param.parameter_type.zsdfs_param;
	int value1 = 0;
	int value2 = 0;

	if (local_atoi32("value1", argv[0], &value1, print) < 0)
		return -EINVAL;

	if (argc >= 2) {
		if (local_atoi32("value2", argv[1], &value2, print) < 0)
			return -EINVAL;
	} else if (param == qcsapi_zsdfs_chan_bw)
		return -EINVAL;

	retval = qcsapi_wifi_set_zsdfs_param(interface, param, value1, value2);

	return qcsapi_report_complete(cb, retval);

}

CALL_QCSAPI(call_qcsapi_wifi_get_zsdfs_param)
{
	int retval;
	qcsapi_zsdfs_param param = cb->generic_param.parameter_type.zsdfs_param;
	int value1;
	int value2;

	retval = qcsapi_wifi_get_zsdfs_param(interface, param, &value1, &value2);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (param == qcsapi_zsdfs_enable)
		print_out(print, "%d\n", value1);
	else
		print_out(print, "%d %d\n", value1, value2);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_cce_channels)
{
	int retval;
	qcsapi_unsigned_int prev_chan = 0;
	qcsapi_unsigned_int cur_chan = 0;
	qcsapi_unsigned_int *p_prev_chan = &prev_chan;
	qcsapi_unsigned_int *p_cur_chan = &cur_chan;

	if (argc >= 2) {
		if (strcmp(argv[1], "NULL") == 0) {
			p_cur_chan = NULL;
		}
	}

	if (argc >= 1) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_prev_chan = NULL;
		}
	}

	retval = qcsapi_wifi_get_scs_cce_channels(interface, p_prev_chan, p_cur_chan);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", (int)prev_chan, (int)cur_chan);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_cce_channels)
{
	int retval;
	qcsapi_unsigned_int prev_chan = 0;
	qcsapi_unsigned_int cur_chan = 0;
	qcsapi_unsigned_int *p_prev_chan = &prev_chan;
	qcsapi_unsigned_int *p_cur_chan = &cur_chan;

	if (argc >= 2) {
		if (strcmp(argv[1], "NULL") == 0) {
			p_cur_chan = NULL;
		}
	}

	if (argc >= 1) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_prev_chan = NULL;
		}
	}

	retval = qcsapi_wifi_get_dfs_cce_channels(interface, p_prev_chan, p_cur_chan);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", (int)prev_chan, (int)cur_chan);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_csw_records)
{
	int retval;
	int reset = 0;
	qcsapi_csw_record records;
	int i;
	int j = 0;

	if (argc == 1) {
		if (local_atoi_bool("reset", argv[0], &reset, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_get_csw_records(interface, reset, &records);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "channel switch history record count : %d\n", records.cnt);
	for (i = 0; i < records.cnt; i++) {
		j = (records.index + QCSAPI_CSW_MAX_RECORDS - i) % QCSAPI_CSW_MAX_RECORDS;
		print_out(print, "time=%u channel=%u reason=%s\n",
				records.timestamp[j], records.channel[j],
				csw_reason_to_string(records.reason[j]));
	}

	if (reset)
		print_out(print, "clear records complete\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_radar_status)
{
	int retval = 0;
	qcsapi_radar_status rdstatus;

	memset(&rdstatus, 0, sizeof(rdstatus));
	rdstatus.channel = atoi(argv[0]);

	retval = qcsapi_wifi_get_radar_status(interface, &rdstatus);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "channel %d:\nradar_status=%d\nradar_count=%d\n",
				rdstatus.channel, rdstatus.flags,
				rdstatus.ic_radardetected);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_WEP_encryption_level)
{
	string_64 buf;
	int retval;

	retval = qcsapi_wifi_get_WEP_encryption_level(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_WEP_key)
{
	string_64 buf = { 0 };
	int retval;
	qcsapi_unsigned_int index = 0;

	if (local_atou32("index", argv[0], &index, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_WEP_key(interface, index, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_WEP_key)
{
	int retval;
	qcsapi_unsigned_int index = 0;

	if (local_atou32("index", argv[0], &index, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_WEP_key(interface, index, argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_WEP_key_index)
{
	int retval;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_WEP_key_index(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_WEP_key_index)
{
	int retval;
	qcsapi_unsigned_int index = 0;

	if (local_atou32("index", argv[0], &index, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_WEP_key_index(interface, index);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_WEP_config)
{
	int retval;

	retval = qcsapi_wifi_remove_WEP_config(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_WPA_encryption_modes)
{
	char buf[36] = { 0 };
	int retval;

	retval = qcsapi_wifi_get_WPA_encryption_modes(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_WPA_encryption_modes)
{
	int retval;

	retval = qcsapi_wifi_set_WPA_encryption_modes(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_WPA_authentication_mode)
{
	char buf[36] = { 0 };
	int retval;

	retval = qcsapi_wifi_get_WPA_authentication_mode(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_WPA_authentication_mode)
{
	int retval;

	retval = qcsapi_wifi_set_WPA_authentication_mode(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

static int local_get_params(const call_bundle *cb, const char *SSID, int argc, char *argv[])
{
	int i;
	int retval = 0;
	qcsapi_output *print = cb->output;
	struct qcsapi_set_parameters get_params;

	if (argc < 1 || argc > ARRAY_SIZE(get_params.param))
		return qcsapi_report_usage(cb);

	memset(&get_params, 0, sizeof(get_params));

	for (i = 0; i < ARRAY_SIZE(get_params.param) && i < argc; i++)
		strncpy(get_params.param[i].key, argv[i], sizeof(get_params.param[i].key) - 1);

	retval = qcsapi_get_params(cb->interface, SSID, &get_params);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < ARRAY_SIZE(get_params.param); i++) {
		if (get_params.param[i].key[0] == 0)
			break;
		print_out(print, "%s: %s\n", get_params.param[i].key,
				get_params.param[i].value);
	}

	return 0;
}


CALL_QCSAPI(call_qcsapi_wifi_get_params)
{
	return local_get_params(cb, NULL, argc, argv);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_params)
{
	int i;
	int retval = 0;
	struct qcsapi_set_parameters remove_params;

	if (argc < 1 || argc > ARRAY_SIZE(remove_params.param))
		return qcsapi_report_usage(cb);

	memset(&remove_params, 0, sizeof(remove_params));

	for (i = 0; i < ARRAY_SIZE(remove_params.param) && i < argc; i++)
		strncpy(remove_params.param[i].key, argv[i],
				sizeof(remove_params.param[i].key) - 1);

	retval = qcsapi_remove_params(cb->interface, NULL, &remove_params);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_params)
{
	int i;
	int k;
	int retval = 0;
	struct qcsapi_set_parameters set_params;

	if (argc < 2 || argc > (2 * ARRAY_SIZE(set_params.param)) || !!(argc % 2))
		return qcsapi_report_usage(cb);

	memset(&set_params, 0, sizeof(set_params));

	for (i = 0, k = 0; i < ARRAY_SIZE(set_params.param) && k < argc; i++, k += 2) {
		strncpy(set_params.param[i].key, argv[k], sizeof(set_params.param[i].key) - 1);
		strncpy(set_params.param[i].value, argv[k + 1],
				sizeof(set_params.param[i].value) - 1);
	}

	retval = qcsapi_set_params(interface, NULL, &set_params);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_get_params)
{
	const char *ssid = cb->generic_param.parameter_type.SSID;

	return local_get_params(cb, ssid, argc, argv);
}

CALL_QCSAPI(call_qcsapi_SSID_set_params)
{
	int i;
	int k;
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;
	struct qcsapi_set_parameters set_params;

	if (argc < 2 || argc > (2 * ARRAY_SIZE(set_params.param)) || !!(argc % 2))
		return qcsapi_report_usage(cb);

	memset(&set_params, 0, sizeof(set_params));

	for (i = 0, k = 0; i < ARRAY_SIZE(set_params.param) && k < argc; i++, k += 2) {
		strncpy(set_params.param[i].key, argv[k], sizeof(set_params.param[i].key) - 1);
		strncpy(set_params.param[i].value, argv[k + 1],
				sizeof(set_params.param[i].value) - 1);
	}

	retval = qcsapi_set_params(interface, ssid, &set_params);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_interworking)
{
	char buf[2] = { 0 };
	int retval;

	retval = qcsapi_wifi_get_interworking(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_interworking)
{
	int retval;
	char *p_interworking = argv[0];

	retval = qcsapi_wifi_set_interworking(interface, p_interworking);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_80211u_params)
{
	string_256 buf = { 0 };
	int retval;

	retval = qcsapi_wifi_get_80211u_params(interface, argv[0], buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_80211u_params)
{
	int retval;
	char *param = argv[0];

	if (!strcmp(param, "ipaddr_type_availability")) {
		if (argc != 3)
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_80211u_params(interface, param, argv[1], argv[2]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_set_sec_agent)
{
	int retval;

	retval = qcsapi_security_set_sec_agent(argv[0], argv[1], 0);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_sec_agent_status)
{
	string_16 val;
	int retval;

	retval = qcsapi_security_get_sec_agent_status(val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_security_get_nai_realms)
{
	string_4096 buf;
	int retval;

	retval = qcsapi_security_get_nai_realms(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_add_nai_realm)
{
	int retval = 0;
	int encoding = 0;
	char *p_nai_realm = argv[1];
	char *p_eap_method = argv[2];

	if (local_atoi_bool("encoding", argv[0], &encoding, print) < 0)
		return -EINVAL;

	if (strcmp(argv[1], "NULL") == 0)
		p_nai_realm = NULL;

	if (strcmp(argv[2], "NULL") == 0)
		p_eap_method = NULL;

	retval = qcsapi_security_add_nai_realm(interface, encoding, p_nai_realm, p_eap_method);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_del_nai_realm)
{
	int retval;
	char *val = argv[0];

	retval = qcsapi_security_del_nai_realm(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_roaming_consortium)
{
	string_1024 buf;
	int retval;

	retval = qcsapi_security_get_roaming_consortium(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_add_roaming_consortium)
{
	int retval;
	char *val = argv[0];

	retval = qcsapi_security_add_roaming_consortium(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_del_roaming_consortium)
{
	int retval;
	char *val = argv[0];

	retval = qcsapi_security_del_roaming_consortium(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_venue_name)
{
	string_4096 buf;
	int retval;

	retval = qcsapi_security_get_venue_name(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_add_venue_name)
{
	int retval;
	char *lang_code = argv[0];
	char *venue_name = argv[1];

	retval = qcsapi_security_add_venue_name(interface, lang_code, venue_name);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_del_venue_name)
{
	int retval;
	char *lang_code = argv[0];
	char *venue_name = argv[1];

	retval = qcsapi_security_del_venue_name(interface, lang_code, venue_name);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_oper_friendly_name)
{
	string_4096 buf;
	int retval;

	retval = qcsapi_security_get_oper_friendly_name(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_add_oper_friendly_name)
{
	int retval;
	char *lang_code = argv[0];
	char *oper_friendly_name = argv[1];

	retval = qcsapi_security_add_oper_friendly_name(interface, lang_code, oper_friendly_name);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_del_oper_friendly_name)
{
	int retval;
	char *lang_code = argv[0];
	char *oper_friendly_name = argv[1];

	retval = qcsapi_security_del_oper_friendly_name(interface, lang_code, oper_friendly_name);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_hs20_conn_capab)
{
	string_4096 buf;
	int retval;

	retval = qcsapi_security_get_hs20_conn_capab(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_add_hs20_conn_capab)
{
	int retval;
	char *ip_proto = argv[0];
	char *port_num = argv[1];
	char *status = argv[2];

	retval = qcsapi_security_add_hs20_conn_capab(interface, ip_proto, port_num, status);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_del_hs20_conn_capab)
{
	int retval;
	char *ip_proto = argv[0];
	char *port_num = argv[1];
	char *status = argv[2];

	retval = qcsapi_security_del_hs20_conn_capab(interface, ip_proto, port_num, status);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_add_hs20_icon)
{
	int retval;
	qcsapi_unsigned_int width;
	qcsapi_unsigned_int height;

	if (local_atou32("width", argv[0], &width, print) < 0)
		return -EINVAL;

	if (local_atou32("height", argv[1], &height, print) < 0)
		return -EINVAL;

	retval = qcsapi_security_add_hs20_icon(interface, width, height,
			argv[2], argv[3], argv[4], argv[5]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_hs20_icon)
{
	int retval;
	string_1024 buf = { 0 };
	char *param = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL"))
		param = buf;

	retval = qcsapi_security_get_hs20_icon(interface, param);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_del_hs20_icon)
{
	int retval;

	retval = qcsapi_security_del_hs20_icon(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_add_osu_server_uri)
{
	int retval;

	retval = qcsapi_security_add_osu_server_uri(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_osu_server_uri)
{
	int retval;
	string_1024 buf = { 0 };
	char *param = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL"))
		param = buf;

	retval = qcsapi_security_get_osu_server_uri(interface, param);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_del_osu_server_uri)
{
	int retval;

	retval = qcsapi_security_del_osu_server_uri(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_add_osu_server_param)
{
	int retval;

	retval = qcsapi_security_add_osu_server_param(interface,
			argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_security_get_osu_server_param)
{
	int retval;
	string_1024 buf = { 0 };

	retval = qcsapi_security_get_osu_server_param(interface, argv[0], argv[1], buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_security_del_osu_server_param)
{
	int retval;
	const char *val = NULL;

	if (argc == 3)
		val = argv[2];

	retval = qcsapi_security_del_osu_server_param(interface, argv[0], argv[1], val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_hs20_status)
{
	string_32 val;
	int retval;

	retval = qcsapi_wifi_get_hs20_status(interface, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_hs20_status)
{
	int retval;

	retval = qcsapi_wifi_set_hs20_status(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_proxy_arp)
{
	int retval;
	uint32_t enable;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_proxy_arp(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_proxy_arp)
{
	char buf[2];
	int retval;

	retval = qcsapi_wifi_get_proxy_arp(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_l2_ext_filter)
{
	int retval;
	string_32 buf;

	retval = qcsapi_wifi_get_l2_ext_filter(interface, argv[0], buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_l2_ext_filter)
{
	int retval;
	char *param = argv[0];
	char *value = argv[1];

	retval = qcsapi_wifi_set_l2_ext_filter(interface, param, value);

	return qcsapi_report_complete(cb, retval);
}

static int local_check_hs20_param(qcsapi_output *print, char *name)
{
	unsigned int i;

	int hs20_param_count = ARRAY_SIZE(qcsapi_hs20_params);

	for (i = 0; i < hs20_param_count; i++) {
		if (strcmp(qcsapi_hs20_params[i], name) == 0)
			return 0;
	}

	print_out(print, "%s is not an hs20 parameter\n", name);

	return -EINVAL;
}

CALL_QCSAPI(call_qcsapi_wifi_get_hs20_params)
{
	string_64 buf;
	int retval;

	if (local_check_hs20_param(print, argv[0]))
		return -EINVAL;

	retval = qcsapi_wifi_get_hs20_params(interface, argv[0], buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_hs20_params)
{
	const char *usage = "call_qcsapi set_hs20_params <WiFi interface> %s %s\n";
	const char *metrics_cmd = "hs20_wan_metrics";
	const char *metrics_params =
		"<WAN info> <uplink speed> <downlink speed> <uplink load> <downlink load> <LMD>";
	const char *dgaf_cmd = "disable_dgaf";
	const char *dgaf_params = "{0 | 1}";

	int retval;

	if (local_check_hs20_param(print, argv[0]))
		return -EINVAL;

	if (!strcmp(argv[0], metrics_cmd)) {
		if (argc != 7) {
			print_out(print, usage, metrics_cmd, metrics_params);
			return -EINVAL;
		}
	}

	if (!strcmp(argv[0], dgaf_cmd)) {
		if (argc != 2) {
			print_out(print, usage, dgaf_cmd, dgaf_params);
			return -EINVAL;
		}
	}

	retval = qcsapi_wifi_set_hs20_params(interface, argv[0],
			argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_remove_11u_param)
{
	return qcsapi_report_complete(cb, qcsapi_remove_11u_param(interface, argv[0]));
}

CALL_QCSAPI(call_qcsapi_remove_hs20_param)
{
	int retval;

	if (local_check_hs20_param(print, argv[0]))
		return -EINVAL;

	retval = qcsapi_remove_hs20_param(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_IEEE11i_encryption_modes)
{
	char buf[36] = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_IEEE11i_encryption_modes(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_IEEE11i_encryption_modes)
{
	int retval;
	char *encryption_mode = argv[0];

	retval = qcsapi_wifi_set_IEEE11i_encryption_modes(interface, encryption_mode);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_IEEE11i_authentication_mode)
{
	char buf[36] = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_IEEE11i_authentication_mode(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_IEEE11i_authentication_mode)
{
	int retval;
	char *authentication_mode = argv[0];

	retval = qcsapi_wifi_set_IEEE11i_authentication_mode(interface, authentication_mode);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_michael_errcnt)
{
	int retval;
	uint32_t val;

	retval = qcsapi_wifi_get_michael_errcnt(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pre_shared_key)
{
	char buf[68] = { 0 };
	int retval;
	qcsapi_unsigned_int index = cb->generic_param.index;

	retval = qcsapi_wifi_get_pre_shared_key(interface, index, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pre_shared_key)
{
	int retval;
	qcsapi_unsigned_int index = cb->generic_param.index;

	retval = qcsapi_wifi_set_pre_shared_key(interface, index, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_psk_auth_failures)
{
	int retval;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_psk_auth_failures(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_key_passphrase)
{
	char buf[68] = { 0 };
	int retval;
	qcsapi_unsigned_int index = cb->generic_param.index;

	retval = qcsapi_wifi_get_key_passphrase(interface, index, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_key_passphrase)
{
	int retval;
	qcsapi_unsigned_int index = cb->generic_param.index;

	retval = qcsapi_wifi_set_key_passphrase(interface, index, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

static int local_get_key_interval(int (*p_key_get_hook)(const char *, unsigned int *),
					const call_bundle *cb, int argc, char *argv[])
{
	unsigned int val;
	int retval;

	retval = p_key_get_hook(cb->interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_group_key_interval)
{
	return local_get_key_interval(qcsapi_wifi_get_group_key_interval, cb, argc, argv);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pairwise_key_interval)
{
	return local_get_key_interval(qcsapi_wifi_get_pairwise_key_interval, cb, argc, argv);
}

static int local_set_key_interval(int (*p_key_set_hook) (const char *, unsigned int),
				const call_bundle *cb, int argc, char *argv[])
{
	char *p_group_key_interval = argv[0];
	int val;
	int retval;

	val = atoi(p_group_key_interval);

	retval = p_key_set_hook(cb->interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_group_key_interval)
{
	return local_set_key_interval(qcsapi_wifi_set_group_key_interval, cb, argc, argv);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pairwise_key_interval)
{
	return local_set_key_interval(qcsapi_wifi_set_pairwise_key_interval, cb, argc, argv);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pmf)
{
	int val = 0;
	int retval;

	retval = qcsapi_wifi_get_pmf(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pmf)
{
	int retval;
	qcsapi_unsigned_int pmf_cap = atoi(argv[0]);

	retval = qcsapi_wifi_set_pmf(interface, pmf_cap);

	if (retval == -qcsapi_daemon_socket_error)
		return 0;

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pairing_id)
{
	char buf[33];
	int retval;

	retval = qcsapi_wifi_get_pairing_id(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pairing_id)
{
	int retval;
	char *pairing_id = argv[0];

	retval = qcsapi_wifi_set_pairing_id(interface, pairing_id);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pairing_enable)
{
	char buf[2];
	int retval;

	retval = qcsapi_wifi_get_pairing_enable(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pairing_enable)
{
	int retval;
	char *pairing_enable = argv[0];

	retval = qcsapi_wifi_set_pairing_enable(interface, pairing_enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_txqos_sched_tbl)
{
	int retval;
	int val;

	if (local_atoi32("index", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_txqos_sched_tbl(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_txqos_sched_tbl)
{
	int val = 0;
	int retval;

	retval = qcsapi_wifi_get_txqos_sched_tbl(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_eth_phy_power_off)
{
	int retval;
	int on_off = atoi(argv[0]);

	retval = qcsapi_eth_phy_power_control(!!on_off, interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_aspm_l1)
{
	int retval;
	int enable = atoi(argv[0]);
	int latency = 0;

	if (enable && argc == 1)
		return qcsapi_report_usage(cb);

	if (enable)
		latency = atoi(argv[1]);

	retval = qcsapi_set_aspm_l1(enable, latency);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_l1)
{
	int retval;
	int enter = atoi(argv[0]);

	if (enter != 0 && enter != 1) {
		print_err(print, "parameter (%d) is not supported\n", enter);
		return -EINVAL;
	}

	retval = qcsapi_set_l1(enter);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mac_address_filtering)
{
	qcsapi_mac_address_filtering val = 0;
	int retval;

	retval = qcsapi_wifi_get_mac_address_filtering(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, (int)val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_mac_address_filtering)
{
	int retval;
	qcsapi_mac_address_filtering val = (qcsapi_mac_address_filtering) atoi(argv[0]);

	retval = qcsapi_wifi_set_mac_address_filtering(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_is_mac_address_authorized)
{
	qcsapi_mac_addr macaddr;
	int retval;
	int is_authorized = -1;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_is_mac_address_authorized(interface, macaddr, &is_authorized);

	return qcsapi_report_int_or_error(cb, retval, is_authorized);
}

#define QCSAPI_AUTH_MAC_ADDR_SIZE 126
CALL_QCSAPI(call_qcsapi_wifi_get_authorized_mac_addresses)
{
	int retval;
	char *buf = NULL;
	unsigned int size = QCSAPI_AUTH_MAC_ADDR_SIZE;

	if (argc > 0) {
		if (local_atou32_range("list size", argv[0], &size, print, 1,
						QCSAPI_MSG_BUFSIZE) < 0)
			return -EINVAL;
	}

	buf = local_malloc(cb, size);
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_authorized_mac_addresses(interface, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_denied_mac_addresses)
{
	int retval;
	char *buf = NULL;
	unsigned int size = QCSAPI_AUTH_MAC_ADDR_SIZE;

	if (argc > 0) {
		if (local_atou32_range("list size", argv[0], &size, print, 1,
						QCSAPI_MSG_BUFSIZE) < 0)
			return -EINVAL;
	}

	buf = local_malloc(cb, size);
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_denied_mac_addresses(interface, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_authorize_mac_address)
{
	qcsapi_mac_addr_list mac_addr_list;
	int retval = 0;
	int i = 0;

	for (i = 0; i < MIN(argc, MAC_ADDR_LIST_SIZE); i++) {
		if (local_parse_macaddr(cb, argv[i], &mac_addr_list[i * MAC_ADDR_SIZE]))
			return -EINVAL;
	}

	retval = qcsapi_wifi_authorize_mac_address_list(interface, i, mac_addr_list);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_deny_mac_address)
{
	qcsapi_mac_addr_list mac_addr_list;
	int retval = 0;
	int i = 0;

	for (i = 0; i < MIN(argc, MAC_ADDR_LIST_SIZE); i++) {
		if (local_parse_macaddr(cb, argv[i], &mac_addr_list[i * MAC_ADDR_SIZE]))
			return -EINVAL;
	}

	retval = qcsapi_wifi_deny_mac_address_list(interface, i, mac_addr_list);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_mac_address)
{
	qcsapi_mac_addr_list mac_addr_list;
	int retval = 0;
	int i = 0;

	for (i = 0; i < MIN(argc, MAC_ADDR_LIST_SIZE); i++) {
		if (local_parse_macaddr(cb, argv[i], &mac_addr_list[i * MAC_ADDR_SIZE]))
			return -EINVAL;
	}

	retval = qcsapi_wifi_remove_mac_address_list(interface, i, mac_addr_list);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_clear_mac_address_filters)
{
	int retval;

	retval = qcsapi_wifi_clear_mac_address_filters(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_mac_address_reserve)
{
	int retval;
	char *eth_type = "";

	if (argc == 4)
		eth_type = argv[3];

	retval = qcsapi_wifi_set_macaddr_reserve(interface, argv[0], argv[1], argv[2], eth_type);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mac_address_reserve)
{
	string_256 buf;
	int retval;

	retval = qcsapi_wifi_get_mac_address_reserve(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_clear_mac_address_reserve)
{
	int retval;

	retval = qcsapi_wifi_clear_mac_address_reserve(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mac_list)
{
	int retval = 0;
	char *buf;
	qcsapi_mac_list_type mac_list_type = 0;
	qcsapi_mac_addr start_macaddr = { 0 };
	int more_data = 1;
	int last_mac_idx;
	int last_mac_offset;

	if (!strcasecmp(argv[0], "denied"))
		mac_list_type = qcsapi_mac_list_type_denied;
	else if (!strcasecmp(argv[0], "authorized"))
		mac_list_type = qcsapi_mac_list_type_authorized;
	else
		return qcsapi_report_usage(cb);

	buf = local_malloc(cb, QCSAPI_MAC_LIST_SIZE_MAX);
	if (!buf)
		return -ENOMEM;

	while (retval >= 0 && more_data) {
		retval = qcsapi_wifi_get_mac_list(interface, mac_list_type, buf,
						QCSAPI_MAC_LIST_SIZE_MAX, start_macaddr);

		qcsapi_report_str_or_error(cb, retval, buf);

		last_mac_idx = QCSAPI_MAC_LIST_SIZE_MAX / (QCSAPI_MAX_ETHER_STRING + 1) - 1;
		last_mac_offset = last_mac_idx * (QCSAPI_MAX_ETHER_STRING + 1);

		if (buf[last_mac_offset] == '\0' || local_parse_macaddr(cb,
						&buf[last_mac_offset], start_macaddr) < 0)
			more_data = 0;
	}

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_backoff_fail_max)
{
	int retval;
	int backoff_fail_max = atoi(argv[0]);

	retval = qcsapi_wifi_backoff_fail_max(interface, backoff_fail_max);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_backoff_timeout)
{
	int retval;
	int backoff_timeout = atoi(argv[0]);

	retval = qcsapi_wifi_backoff_timeout(interface, backoff_timeout);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_registrar_report_button_press)
{
	int retval = qcsapi_wps_registrar_report_button_press(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_registrar_report_pin)
{
	int retval;

	retval = qcsapi_wps_registrar_report_pin(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_registrar_get_pp_devname)
{
	int retval;
	string_128 buf = "";
	int blacklist = 0;

	if (argc == 1 && strcmp(argv[0], "blacklist") == 0)
		blacklist = 1;

	retval = qcsapi_wps_registrar_get_pp_devname(interface, blacklist, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wps_registrar_set_pp_devname)
{
	int retval;
	char *p_pp_devname = NULL;
	uint32_t wps_pp_status;
	int update_blacklist = 0;

	if (argc == 1) {
		p_pp_devname = strcmp(argv[0], "NULL") == 0 ? NULL : argv[0];
	} else if (argc == 2 && strcmp(argv[0], "blacklist") == 0) {
		update_blacklist = 1;
		p_pp_devname = strcmp(argv[1], "NULL") == 0 ? NULL : argv[1];
	} else {
		qcsapi_report_usage(cb);
		return 0;
	}

	retval = qcsapi_wps_get_access_control(interface, &wps_pp_status);
	if (retval >= 0) {
		if (wps_pp_status == 0) {
			print_err(print,
				"Enable WPS Pairing Protection before setting device name list\n");
			return -EINVAL;
		}
	}

	if (retval >= 0)
		retval = qcsapi_wps_registrar_set_pp_devname(interface, update_blacklist,
				p_pp_devname);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_enrollee_report_button_press)
{
	int retval = 0;
	qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
	qcsapi_mac_addr macaddr;

	if (argc > 0) {
		if (strcasecmp(argv[0], "any") != 0) {
			if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
				return -EINVAL;
		}
	}

	retval = qcsapi_wps_enrollee_report_button_press(interface, local_bssid);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_enrollee_report_pin)
{
	int retval = 0;
	qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
	const char *wps_pin;

	if (argc > 1) {
		if (strcasecmp(argv[0], "any") != 0) {
			if (local_parse_macaddr(cb, argv[0], local_bssid) < 0)
				return -EINVAL;
		}
		wps_pin = argv[1];
	} else {
		wps_pin = argv[0];
	}

	retval = qcsapi_wps_enrollee_report_pin(interface, local_bssid, wps_pin);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_enrollee_generate_pin)
{
	int retval;
	qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
	char generated_pin[QCSAPI_WPS_MAX_PIN_LEN + 1];

	if (argc > 0) {
		if (strcasecmp(argv[0], "any") != 0) {
			if (local_parse_macaddr(cb, argv[0], local_bssid) < 0)
				return -EINVAL;
		}
	}

	retval = qcsapi_wps_enrollee_generate_pin(interface, local_bssid, generated_pin);

	return qcsapi_report_str_or_error(cb, retval, generated_pin);
}

CALL_QCSAPI(call_qcsapi_wps_get_ap_pin)
{
	int retval;
	char buf[QCSAPI_WPS_MAX_PIN_LEN + 1];
	int force = 0;

	if (argc == 1) {
		if (local_atoi_bool("force", argv[0], &force, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wps_get_ap_pin(interface, buf, force);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wps_set_ap_pin)
{
	int retval;
	char wps_pin[2 * QCSAPI_WPS_MAX_PIN_LEN] = { 0 };

	strncpy(wps_pin, argv[0], sizeof(wps_pin) - 1);

	retval = qcsapi_wps_set_ap_pin(interface, wps_pin);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_save_ap_pin)
{
	int retval;

	retval = qcsapi_wps_save_ap_pin(interface);
	if (retval == -qcsapi_parameter_not_found) {
		print_err(print, "No AP PIN exists - set or generate one\n");
		return retval;
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_enable_ap_pin)
{
	int retval;
	int enable;

	if (local_atoi_bool("enable", argv[0], &enable, print) < 0)
		return qcsapi_report_usage(cb);

	retval = qcsapi_wps_enable_ap_pin(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_get_sta_pin)
{
	int retval;
	char buf[QCSAPI_WPS_MAX_PIN_LEN + 1];

	retval = qcsapi_wps_get_sta_pin(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

#define WPS_GET_STATE_MAX_LEN	128

CALL_QCSAPI(call_qcsapi_wps_get_state)
{
	int retval;
	char buf[WPS_GET_STATE_MAX_LEN] = "";

	retval = qcsapi_wps_get_state(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

#define WPA_GET_STATUS_MAX_LEN		32

CALL_QCSAPI(call_qcsapi_wifi_get_wpa_status)
{
	int retval;
	qcsapi_mac_addr macaddr;
	char buf[WPA_GET_STATUS_MAX_LEN] = "";

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_wpa_status(interface, buf, argv[0], sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_auth_state)
{
	int retval;
	qcsapi_mac_addr macaddr;
	int val = 0;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_auth_state(interface, argv[0], &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_disconn_info)
{
	int retval;
	qcsapi_disconn_info info;

	memset(&info, 0, sizeof(info));
	retval = qcsapi_wifi_get_disconn_info(interface, &info);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "association\t%d\n"
			"disconnect\t%d\n"
			"sequence\t%d\n"
			"uptime\t%d\n", info.asso_sta_count, info.disconn_count,
			info.sequence, info.up_time);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_reset_disconn_info)
{
	int retval;
	qcsapi_disconn_info info;

	memset(&info, 0, sizeof(info));
	info.resetflag = 1;
	retval = qcsapi_wifi_get_disconn_info(interface, &info);

	return qcsapi_report_str_or_error(cb, retval, "Reset complete\n");
}

CALL_QCSAPI(call_qcsapi_wps_get_configured_state)
{
	int retval;
	char buf[WPS_GET_STATE_MAX_LEN] = "";

	retval = qcsapi_wps_get_configured_state(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wps_get_runtime_state)
{
	int retval;
	char buf[WPS_GET_STATE_MAX_LEN] = "";

	retval = qcsapi_wps_get_runtime_state(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wps_allow_pbc_overlap)
{
	int retval;
	int allow = !!atoi(argv[0]);

	retval = qcsapi_wps_allow_pbc_overlap(interface, allow);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_get_allow_pbc_overlap_status)
{
	int retval;
	int val = -1;

	retval = qcsapi_wps_get_allow_pbc_overlap_status(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wps_check_config)
{
	int retval;

	retval = qcsapi_wps_check_config(interface);

	return qcsapi_report_complete(cb, retval);
}

#define WPS_GET_CFG_MAX_LEN 100

static int local_wps_param_str_to_type(const char *param_name, qcsapi_wps_param_type *param_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qcsapi_wps_param_map_tbl); i++) {
		if (strcmp(param_name, qcsapi_wps_param_map_tbl[i].param_name) == 0) {
			*param_type = qcsapi_wps_param_map_tbl[i].param_type;
			return 0;
		}
	}

	return -EINVAL;
}

CALL_QCSAPI(call_qcsapi_wps_get_param)
{
	int retval;
	qcsapi_wps_param_type wps_cfg_str_id;
	char  *buf;

	if (local_wps_param_str_to_type(argv[0], &wps_cfg_str_id) < 0) {
		print_err(print, "Invalid WPS parameter name\n");
		return -EINVAL;
	}

	buf = local_malloc(cb, QCSAPI_MAX_PARAM_VAL_LEN);
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wps_get_param(interface, wps_cfg_str_id, buf, QCSAPI_MAX_PARAM_VAL_LEN - 1);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wps_set_param)
{
	int retval;
	qcsapi_wps_param_type wps_cfg_str_id;

	if (local_wps_param_str_to_type(argv[0], &wps_cfg_str_id) < 0) {
		print_err(print, "Invalid WPS parameter name\n");
		return -EINVAL;
	}

	retval = qcsapi_wps_set_param(interface, wps_cfg_str_id, argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_set_configured_state)
{
	uint8_t val;
	int retval;

	val = (uint8_t) atoi(argv[0]);

	retval = qcsapi_wps_set_configured_state(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dwell_times)
{
	int retval;
	unsigned int max_active_chan;
	unsigned int min_active_chan;
	unsigned int max_passive_chan;
	unsigned int min_passive_chan;

	max_active_chan = (unsigned int)atoi(argv[0]);
	min_active_chan = (unsigned int)atoi(argv[1]);
	max_passive_chan = (unsigned int)atoi(argv[2]);
	min_passive_chan = (unsigned int)atoi(argv[3]);

	retval = qcsapi_wifi_set_dwell_times(interface, max_active_chan,
				min_active_chan, max_passive_chan, min_passive_chan);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dwell_times)
{
	int retval;
	unsigned int max_active_chan = 0;
	unsigned int min_active_chan = 0;
	unsigned int max_passive_chan = 0;
	unsigned int min_passive_chan = 0;

	retval = qcsapi_wifi_get_dwell_times(interface, &max_active_chan,
			&min_active_chan, &max_passive_chan, &min_passive_chan);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d %d %d\n",
		max_active_chan, min_active_chan, max_passive_chan, min_passive_chan);

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_chan_avail_status_by_bw)
{
	int retval = 0;
	struct qcsapi_chan_avail_stats stats;
	int i;
	const char *str[] = QTN_CHAN_AVAIL_STATUS_TO_STR;
	int chan_status;

	memset(&stats, 0, sizeof(stats));
	if (argc >= 1) {
		if (local_atoi32("bandwidth", argv[0], &stats.bw, print) < 0)
			return -EINVAL;
	} else {
		stats.bw = 0;
	}

	retval = qcsapi_wifi_get_chan_avail_status_by_bw(interface,
				&stats);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "Channel	Status	Status_string for BW %dMHz\n", stats.bw);
	for (i = 0; i < ARRAY_SIZE(stats.chan_avail_status); i++) {
		chan_status = stats.chan_avail_status[i];
		if (chan_status)
			print_out(print, "%7d	%6d	%s\n", i,
						chan_status, str[chan_status]);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_bgscan_dwell_times)
{
	int retval;
	unsigned int active_chan;
	unsigned int passive_chan;

	active_chan = (unsigned int)atoi(argv[0]);
	passive_chan = (unsigned int)atoi(argv[1]);

	retval = qcsapi_wifi_set_bgscan_dwell_times(interface, active_chan, passive_chan);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bgscan_dwell_times)
{
	int retval;
	unsigned int active_chan = 0;
	unsigned int passive_chan = 0;

	retval = qcsapi_wifi_get_bgscan_dwell_times(interface, &active_chan, &passive_chan);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", active_chan, passive_chan);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_count_associations)
{
	uint32_t val;
	int retval;

	retval = qcsapi_wifi_get_count_associations(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_associated_device_mac_addr)
{
	qcsapi_mac_addr macaddr;
	int retval;
	qcsapi_unsigned_int dev_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_associated_device_mac_addr(interface, dev_idx, macaddr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_dump_macaddr(cb->output, macaddr);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_associated_device_ip_addr)
{
	unsigned int ip_addr = 0;
	char ip_str[IP_ADDR_STR_LEN + 1];
	int retval;
	qcsapi_unsigned_int dev_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_associated_device_ip_addr(interface, dev_idx, &ip_addr);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	inet_ntop(AF_INET, &ip_addr, ip_str, IP_ADDR_STR_LEN);
	print_out(print, "%s\n", ip_str);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_link_quality)
{
	uint32_t val;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_link_quality(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rssi_per_association)
{
	uint32_t val;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rssi_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rssi_in_dbm_per_association)
{
	int32_t val;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rssi_in_dbm_per_association(interface, assoc_idx, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association)
{
	int rssi = 0;
	int rssi_min = 0;
	int rssi_max = 0;
	uint32_t seq_num = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association(interface,
			assoc_idx, &rssi_min, &rssi_max, &rssi, &seq_num);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%u: rssi %d min %d max %d\n",
			seq_num, rssi, rssi_min, rssi_max);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_meas_rssi_in_dbm_per_association)
{
	int rssi = 0;
	uint32_t seq_num = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_meas_rssi_in_dbm_per_association(interface,
					assoc_idx, &rssi, &seq_num);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%u: rssi %d\n", seq_num, rssi);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_rssi_meas_period)
{
	int retval;
	int period;

	if (local_atoi32_range("period", argv[0], &period, print, 0, INT32_MAX) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_rssi_meas_period(interface, period);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rssi_meas_period)
{
	int retval;
	uint32_t val = 0;

	retval = qcsapi_wifi_get_rssi_meas_period(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_snr_per_association)
{
	int retval;
	int val = 0;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_snr_per_association(interface, assoc_idx, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_hw_noise_per_association)
{
	int retval;
	int val;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_hw_noise_per_association(interface, assoc_idx, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d.%d\n", val / 10, abs(val % 10));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_rx_bytes_per_association)
{
	uint64_t rx_bytes = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rx_bytes_per_association(interface, assoc_idx, &rx_bytes);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%llu\n", rx_bytes);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_bytes_per_association)
{
	uint64_t val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_bytes_per_association(interface, assoc_idx, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%llu\n", val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_rx_packets_per_association)
{
	uint32_t val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rx_packets_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_packets_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_packets_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_err_packets_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_err_packets_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_allretries_per_association)
{
	qcsapi_unsigned_int val;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_allretries_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_exceed_retry_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_exceed_retry_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_retried_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_retried_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_retried_percent_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_retried_percent_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bw_per_association)
{
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_bw_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_phy_rate_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_phy_rate_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rx_phy_rate_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rx_phy_rate_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_mcs_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_tx_mcs_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rx_mcs_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_rx_mcs_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_achievable_tx_phy_rate_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_achievable_tx_phy_rate_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_achievable_rx_phy_rate_per_association)
{
	qcsapi_unsigned_int val = 0;
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_achievable_rx_phy_rate_per_association(interface, assoc_idx, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_auth_enc_per_association)
{
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	qcsapi_unsigned_int val = 0;
	uint8_t *assoc = (uint8_t *) &val;

	retval = qcsapi_wifi_get_auth_enc_per_association(interface, assoc_idx, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (assoc[IEEE80211_AUTHDESCR_ALGO_POS] >= ARRAY_SIZE(qcsapi_auth_algo_list) ||
		assoc[IEEE80211_AUTHDESCR_KEYMGMT_POS] >= ARRAY_SIZE(qcsapi_auth_keymgmt_list) ||
		assoc[IEEE80211_AUTHDESCR_KEYPROTO_POS] >= ARRAY_SIZE(qcsapi_auth_keyproto_list) ||
		assoc[IEEE80211_AUTHDESCR_CIPHER_POS] >= ARRAY_SIZE(qcsapi_auth_cipher_list)) {

		print_err(print, "Unknown auth enc value \"%08X\"\n", val);
		return -EINVAL;
	}

	if (assoc[IEEE80211_AUTHDESCR_KEYPROTO_POS])
		print_out(print, "%s/%s with %s\n",
			qcsapi_auth_keyproto_list[assoc[IEEE80211_AUTHDESCR_KEYPROTO_POS]],
			qcsapi_auth_keymgmt_list[assoc[IEEE80211_AUTHDESCR_KEYMGMT_POS]],
			qcsapi_auth_cipher_list[assoc[IEEE80211_AUTHDESCR_CIPHER_POS]]);
	else
		print_out(print, "%s/%s\n",
			qcsapi_auth_algo_list[assoc[IEEE80211_AUTHDESCR_ALGO_POS]],
			qcsapi_auth_keymgmt_list[assoc[IEEE80211_AUTHDESCR_KEYMGMT_POS]]);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_vendor_per_association)
{
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_vendor_per_association(interface, assoc_idx, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	switch (val) {
	case PEER_VENDOR_QTN:
		print_out(print, "quantenna\n");
		break;
	case PEER_VENDOR_BRCM:
		print_out(print, "broadcom\n");
		break;
	case PEER_VENDOR_ATH:
		print_out(print, "atheros\n");
		break;
	case PEER_VENDOR_RLNK:
		print_out(print, "ralink\n");
		break;
	case PEER_VENDOR_RTK:
		print_out(print, "realtek\n");
		break;
	case PEER_VENDOR_INTEL:
		print_out(print, "intel\n");
		break;
	default:
		print_out(print, "unknown\n");
		break;
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_set_rf_chains)
{
	int retval;
	qcsapi_unsigned_int tx_5g = atoi(argv[0]);
	qcsapi_unsigned_int rx_5g = atoi(argv[1]);
	qcsapi_unsigned_int tx_2g = atoi(argv[2]);
	qcsapi_unsigned_int rx_2g = atoi(argv[3]);
	qcsapi_unsigned_int tx_5g_2 = 0;
	qcsapi_unsigned_int rx_5g_2 = 0;

	if (argc == 6) {
		tx_5g_2 = atoi(argv[4]);
		rx_5g_2 = atoi(argv[5]);
	}

	retval = qcsapi_set_rf_chains(tx_5g, rx_5g, tx_2g, rx_2g, tx_5g_2, rx_5g_2);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_rf_chains)
{
	int retval;
	qcsapi_unsigned_int tx_5g;
	qcsapi_unsigned_int rx_5g;
	qcsapi_unsigned_int tx_2g;
	qcsapi_unsigned_int rx_2g;
	qcsapi_unsigned_int tx_5g_2;
	qcsapi_unsigned_int rx_5g_2;

	retval = qcsapi_get_rf_chains(&tx_5g, &rx_5g, &tx_2g, &rx_2g, &tx_5g_2, &rx_5g_2);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "5G: %dx%d, 2.4G: %dx%d, 5G_2: %dx%d\n",
				tx_5g, rx_5g, tx_2g, rx_2g, tx_5g_2, rx_5g_2);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_chains)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_tx_chains(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rx_chains)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_rx_chains(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_eap_reauth_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("period", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_eap_reauth_period(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_eap_reauth_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_eap_reauth_period(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_eap_reauth_period)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_eap_reauth_period(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_eap_param)
{
	int retval = 0;
	qcsapi_eap_param_type type = cb->generic_param.parameter_type.eap_param_type;
	qcsapi_unsigned_int val;

	if (local_atou32("value", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_eap_param(interface, type, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_eap_param)
{
	int retval = 0;
	qcsapi_unsigned_int val;
	qcsapi_eap_param_type type = cb->generic_param.parameter_type.eap_param_type;

	retval = qcsapi_wifi_get_eap_param(interface, type, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_eap_param)
{
	int retval = 0;
	qcsapi_eap_param_type type = cb->generic_param.parameter_type.eap_param_type;

	retval = qcsapi_wifi_remove_eap_param(interface, type);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_radius_max_retries)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("retries", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_radius_max_retries(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_radius_max_retries)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_radius_max_retries(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_radius_max_retries)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_radius_max_retries(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_radius_num_failover)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("num", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_radius_num_failover(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_radius_num_failover)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_radius_num_failover(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_radius_num_failover)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_radius_num_failover(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_radius_timeout)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("timeout", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_radius_timeout(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_radius_timeout)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_radius_timeout(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_radius_timeout)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_radius_timeout(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pmk_cache_enable)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("enable", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_pmk_cache_enable(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pmk_cache_enable)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_pmk_cache_enable(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pmk_cache_lifetime)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("lifetime", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_pmk_cache_lifetime(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_pmk_cache_lifetime)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_pmk_cache_lifetime(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_pmk_cache_lifetime)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_pmk_cache_lifetime(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_max_auth_attempts)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("max attempts", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_max_auth_attempts(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_max_auth_attempts)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_max_auth_attempts(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_lockout_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("lockout period", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_lockout_period(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_lockout_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_lockout_period(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_lockout_period)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_lockout_period(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_id_request_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("period", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_id_request_period(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_id_request_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_id_request_period(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_id_request_period)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_id_request_period(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_auth_quiet_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	if (local_atou32("period", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_auth_quiet_period(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_auth_quiet_period)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_auth_quiet_period(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_remove_auth_quiet_period)
{
	int retval = 0;

	retval = qcsapi_wifi_remove_auth_quiet_period(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_max_mimo)
{
	int retval;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	string_16 buf;

	retval = qcsapi_wifi_get_max_mimo(interface, assoc_idx, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tput_caps)
{
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	int retval;
	struct ieee8011req_sta_tput_caps tput_caps;
	struct ieee80211_ie_vhtcap *ie_vhtcap;
	struct ieee80211_ie_htcap *ie_htcap;

	retval = qcsapi_wifi_get_tput_caps(interface, assoc_idx, &tput_caps);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	switch (tput_caps.mode) {
	case IEEE80211_WIFI_MODE_AX:
		print_out(print, "Mode: HE\n");
		/* TODO: Fall through for 5G only as AC is non-standard for 2.4G */
	case IEEE80211_WIFI_MODE_AC:
		if (tput_caps.mode != IEEE80211_WIFI_MODE_AX)
			print_out(print, "Mode: VHT\n");
		ie_vhtcap = (struct ieee80211_ie_vhtcap *)tput_caps.vhtcap_ie;
		print_out(print, "VHT Capabilities Info: ");
		local_dump_data_array(print, ie_vhtcap->vht_cap,
				sizeof(ie_vhtcap->vht_cap), 16, ' ');
		print_out(print, "Supported VHT MCS & NSS Set: ");
		local_dump_data_array(print, ie_vhtcap->vht_mcs_nss_set,
				sizeof(ie_vhtcap->vht_mcs_nss_set), 16, ' ');
		/* Fall through */
	case IEEE80211_WIFI_MODE_NA:
		/* Fall through */
	case IEEE80211_WIFI_MODE_NG:
		if (tput_caps.mode != IEEE80211_WIFI_MODE_AC &&
				tput_caps.mode != IEEE80211_WIFI_MODE_AX)
			print_out(print, "Mode: HT\n");
		ie_htcap = (struct ieee80211_ie_htcap *)tput_caps.htcap_ie;

		print_out(print, "HT Capabilities Info: ");
		local_dump_data_array(print, ie_htcap->hc_cap,
					sizeof(ie_htcap->hc_cap), 16, ' ');

		print_out(print, "A-MPDU Parameters: %02X\n", ie_htcap->hc_ampdu);

		print_out(print, "Supported MCS Set: ");
		local_dump_data_array(print, ie_htcap->hc_mcsset,
					sizeof(ie_htcap->hc_mcsset), 16, ' ');

		print_out(print, "HT Extended Capabilities: ");
		local_dump_data_array(print, ie_htcap->hc_extcap,
					sizeof(ie_htcap->hc_extcap), 16, ' ');

		print_out(print, "Transmit Beamforming Capabilities: ");
		local_dump_data_array(print, ie_htcap->hc_txbf,
					sizeof(ie_htcap->hc_txbf), 16, ' ');

		print_out(print, "ASEL Capabilities: %02X\n", ie_htcap->hc_antenna);
		break;
	default:
		print_out(print, "Mode: non HT\n");
		break;
	}

	return 0;
}

static const char *local_wifi_mode_str(uint32_t wifi_mode)
{
	if (wifi_mode >= ARRAY_SIZE(qcsapi_wifi_modes_strings))
		return IEEE80211_WIFI_MODE_NONE;

	return qcsapi_wifi_modes_strings[wifi_mode];
}

CALL_QCSAPI(call_qcsapi_wifi_get_connection_mode)
{
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_connection_mode(interface, assoc_idx, &val);

	return qcsapi_report_str_or_error(cb, retval, local_wifi_mode_str(val));
}

CALL_QCSAPI(call_qcsapi_wifi_get_node_counter)
{
	int retval;
	qcsapi_unsigned_int node_index = cb->generic_param.index;
	qcsapi_counter_type type = qcsapi_nosuch_counter;
	int local_remote = QCSAPI_LOCAL_NODE;
	uint64_t val = 0;

	if (call_qcsapi_param_name2enum(print, &qcsapi_param_name_counter, argv[0], &type) < 0)
		return -EINVAL;

	if (argc > 1) {
		if (local_parse_loca_remote_flag(print, argv[1], &local_remote) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_get_node_counter(interface, node_index, type, local_remote, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%llu\n", val);

	return 0;
}

static int local_meas_arg(char *pref, char *arg, int *val)
{
	int preflen = strlen(pref);

	if (strncmp(arg, pref, preflen) == 0 && strlen(arg) > preflen) {
		*val = atoi(arg + strlen(pref));
		return 0;
	}

	return -1;
}

static int local_meas_arg_mac(char *pref, char *arg, uint8_t *mac)
{
	int preflen = strlen(pref);
	int i;
	unsigned int macint[IEEE80211_ADDR_LEN];

	if (strncmp(arg, pref, preflen) == 0 && strlen(arg) > preflen) {
		if (sscanf(arg + strlen(pref), MACSTR, MAC2ADDR(macint)) != 6)
			return -1;
		for (i = 0; i < 6; i++)
			mac[i] = (uint8_t) macint[i];
		return 0;
	}

	return -1;
}

static int local_parse_measure_request_param(qcsapi_measure_request_param *param,
		qcsapi_output *print, qcsapi_per_assoc_param type, int argc, char *argv[])
{
	int i;
	int qualified;
	int val;
	int bad_format = 0;
	char *arg;

	qualified = 0;
	switch (type) {
	case QCSAPI_NODE_MEAS_BASIC:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->basic.channel = val;
			} else if (local_meas_arg("off=", arg, &val) == 0) {
				param->basic.offset = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->basic.duration = val;
				qualified++;
			} else {
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				bad_format = 1;
				break;
			}
		}
		if (!qualified || bad_format) {
			print_err(print,
				"Basic measurement param: <du=duration> [ch=chan] [off=offset]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_CCA:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->cca.channel = val;
			} else if (local_meas_arg("off=", arg, &val) == 0) {
				param->cca.offset = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->cca.duration = val;
				qualified++;
			} else {
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				bad_format = 1;
				break;
			}
		}
		if (!qualified || bad_format) {
			print_err(print,
				"CCA measurement param: <du=duration> [ch=chan] [off=offset]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_RPI:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->rpi.channel = val;
			} else if (local_meas_arg("off=", arg, &val) == 0) {
				param->rpi.offset = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->rpi.duration = val;
				qualified++;
			} else {
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				bad_format = 1;
			}
		}
		if (!qualified || bad_format) {
			print_err(print,
				"RPI measurement param: <du=duration> [ch=chan] [off=offset]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_CHAN_LOAD:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->chan_load.channel = val;
			} else if (local_meas_arg("op=", arg, &val) == 0) {
				param->chan_load.op_class = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->chan_load.duration = val;
				qualified++;
			} else {
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				bad_format = 1;
				break;
			}
		}
		if (!qualified || bad_format) {
			print_err(print,
				"Chan load param: <du=duration> [ch=chan] [op=op class]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_NOISE_HIS:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->noise_his.channel = val;
			} else if (local_meas_arg("op=", arg, &val) == 0) {
				param->noise_his.op_class = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->noise_his.duration = val;
				qualified++;
			} else {
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				bad_format = 1;
				break;
			}
		}
		if (!qualified || bad_format) {
			print_err(print,
				"Noise histogram param: <du=duration> [ch=chan] [op=op class]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_BEACON:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->beacon.channel = val;
			} else if (local_meas_arg("op=", arg, &val) == 0) {
				param->beacon.op_class = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->beacon.duration = val;
				qualified++;
			} else if (local_meas_arg("mode=", arg, &val) == 0) {
				param->beacon.mode = val;
			} else {
				bad_format = 1;
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				break;
			}
		}

		if (!qualified || bad_format) {
			print_err(print,
				"Bcn param: <du=duration> [ch=chan] [mode=mode] [op=op class]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_FRAME:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("ch=", arg, &val) == 0) {
				param->frame.channel = val;
			} else if (local_meas_arg("op=", arg, &val) == 0) {
				param->frame.op_class = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->frame.duration = val;
				qualified++;
			} else if (local_meas_arg("type=", arg, &val) == 0) {
				param->frame.type = val;
				qualified++;
			} else if (local_meas_arg_mac("mac=", arg, param->frame.mac_address) != 0) {
				bad_format = 1;
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				break;
			}
		}
		if ((qualified < 2) || bad_format) {
			print_err(print,
			"Frame param: <du=dur> <type=type> [ch=chan] [op=op class] [mac=mac]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_TRAN_STREAM_CAT:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("tid=", arg, &val) == 0) {
				param->tran_stream_cat.tid = val;
				qualified++;
			} else if (local_meas_arg("bin0=", arg, &val) == 0) {
				param->tran_stream_cat.bin0 = val;
			} else if (local_meas_arg("du=", arg, &val) == 0) {
				param->tran_stream_cat.duration = val;
				qualified++;
			} else if (local_meas_arg_mac("peer_sta=", arg,
							param->tran_stream_cat.peer_sta) != 0) {
				bad_format = 1;
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				break;
			}
		}
		if ((qualified < 2) || bad_format) {
			print_err(print,
			"Transmit strm param: <du=dur> <tid=TID> [peer_sta=mac] [bin0=range]\n");
			return -EINVAL;
		}
		break;
	case QCSAPI_NODE_MEAS_MULTICAST_DIAG:
		for (i = 0; i < argc; i++) {
			arg = argv[i];
			if (local_meas_arg("du=", arg, &val) == 0) {
				param->multicast_diag.duration = val;
				qualified++;
			} else if (local_meas_arg_mac("group_mac=", arg,
							param->multicast_diag.group_mac) == 0) {
				qualified++;
			} else {
				bad_format = 1;
				print_err(print, "Unknown parameter \"%s\"\n", arg);
				break;
			}
		}
		if ((qualified < 2) || bad_format) {
			print_err(print,
				"Mcast param: <du=duration> <group_mac=group mac address>\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_node_param)
{
	int retval;
	qcsapi_unsigned_int node_index = cb->generic_param.index;
	qcsapi_per_assoc_param param_type = QCSAPI_NO_SUCH_PER_ASSOC_PARAM;
	int local_remote = QCSAPI_LOCAL_NODE;
	string_128 input_param_str;
	qcsapi_measure_request_param *request_param;
	qcsapi_measure_report_result result;
	int *p_param_value;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_per_node_param;
	uint8_t i;
	qcsapi_mac_addr macaddr;

	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &param_type) < 0)
		return -EINVAL;

	if (argc > 1) {
		if (local_parse_loca_remote_flag(print, argv[1], &local_remote) < 0)
			return -EINVAL;
	}

	request_param = (qcsapi_measure_request_param *) input_param_str;
	if (argc >= 2) {
		argc -= 2;
		argv += 2;
		memset(request_param, 0, sizeof(*request_param));
		if (local_parse_measure_request_param(request_param, print, param_type,
							argc, argv))
			return -EINVAL;
	}

	memset(&result, 0, sizeof(result));
	retval = qcsapi_wifi_get_node_param(interface, node_index, param_type, local_remote,
						input_param_str, &result);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	p_param_value = result.common;
	switch (param_type) {
	case QCSAPI_SOC_MAC_ADDR:
			memcpy(macaddr, p_param_value, sizeof(qcsapi_mac_addr));
			local_dump_macaddr(print, macaddr);
			break;
	case QCSAPI_SOC_IP_ADDR:
		print_out(print, "%d.%d.%d.%d\n",
			((char *)p_param_value)[0], ((char *)p_param_value)[1],
			((char *)p_param_value)[2], ((char *)p_param_value)[3]);
		break;
	case QCSAPI_NODE_MEAS_RPI:
		local_dump_data_array(print, result.rpi, 8, 10, ' ');
		break;
	case QCSAPI_NODE_TPC_REP:
		print_out(print, "link margin = %d db\n", result.tpc.link_margin);
		print_out(print, "transmit power = %d dbm\n", result.tpc.tx_power);
		break;
	case QCSAPI_NODE_MEAS_NOISE_HIS:
		print_out(print, "anntenna id = %d\n", result.noise_histogram.antenna_id);
		print_out(print, "anpi = %d\n", (0 - result.noise_histogram.anpi));
		for (i = 0; i < 11; i++)
			print_out(print, "ipi%d:%d\n", i,
				result.noise_histogram.ipi[i]);
		break;
	case QCSAPI_NODE_MEAS_BEACON:
		print_out(print, "report frame info = %x\n", result.beacon.rep_frame_info);
		print_out(print, "rcpi = %d\n", result.beacon.rcpi);
		print_out(print, "rsni = %d\n", result.beacon.rsni);
		print_out(print, "mac address:");
		memcpy(macaddr, result.beacon.bssid, sizeof(qcsapi_mac_addr));
		local_dump_macaddr(print, macaddr);
		print_out(print, "antenna id = %d\n", result.beacon.antenna_id);
		print_out(print, "parent_tsf = %d\n", result.beacon.parent_tsf);
		break;
	case QCSAPI_NODE_MEAS_FRAME:
		if (result.frame.sub_ele_report == 0) {
			print_out(print, "no measurement result\n");
		} else {
			print_out(print, "TA address:");
			memcpy(macaddr, result.frame.ta, sizeof(qcsapi_mac_addr));
			local_dump_macaddr(print, macaddr);
			print_out(print, "BSSID:");
			memcpy(macaddr, result.frame.bssid, sizeof(qcsapi_mac_addr));
			local_dump_macaddr(print, macaddr);
			print_out(print, "phy_type = %d\n", result.frame.phy_type);
			print_out(print, "average RCPI = %d\n", result.frame.avg_rcpi);
			print_out(print, "last RSNI = %d\n", result.frame.last_rsni);
			print_out(print, "last RCPI = %d\n", result.frame.last_rcpi);
			print_out(print, "antenna id = %d\n", result.frame.antenna_id);
			print_out(print, "Frame count = %d\n", result.frame.frame_count);
		}
		break;
	case QCSAPI_NODE_MEAS_TRAN_STREAM_CAT:
		print_out(print, "reason = %d\n",
				result.tran_stream_cat.reason);
		print_out(print, "transmitted MSDU count = %d\n",
			result.tran_stream_cat.tran_msdu_cnt);
		print_out(print, "MSDU Discarded Count = %d\n",
			result.tran_stream_cat.msdu_discard_cnt);
		print_out(print, "MSDU Failed Count = %d\n",
			result.tran_stream_cat.msdu_fail_cnt);
		print_out(print, "MSDU Multiple retry Count = %d\n",
			result.tran_stream_cat.msdu_mul_retry_cnt);
		print_out(print, "MSDU Qos CF-Polls Lost Count = %d\n",
			result.tran_stream_cat.qos_lost_cnt);
		print_out(print, "Average Queue Delay = %d\n",
			result.tran_stream_cat.avg_queue_delay);
		print_out(print, "Average Transmit Delay = %d\n",
			result.tran_stream_cat.avg_tran_delay);
		print_out(print, "Bin0 range = %d\n",
				result.tran_stream_cat.bin0_range);
		for (i = 0; i < 6; i++)
			print_out(print, "Bin%d = %d\n", i,
				result.tran_stream_cat.bins[i]);
		break;
	case QCSAPI_NODE_MEAS_MULTICAST_DIAG:
		print_out(print, "reason = %d\n",
				result.multicast_diag.reason);
		print_out(print, "Multicast Received MSDU Count = %d\n",
				result.multicast_diag.mul_rec_msdu_cnt);
		print_out(print, "First Sequence Number = %d\n",
				result.multicast_diag.first_seq_num);
		print_out(print, "Last Sequence Number = %d\n",
				result.multicast_diag.last_seq_num);
		print_out(print, "Multicast Rate = %d\n",
				result.multicast_diag.mul_rate);
		break;
	case QCSAPI_NODE_LINK_MEASURE:
		print_out(print, "transmit power = %d\n",
				result.link_measure.tpc_report.tx_power);
		print_out(print, "link margin = %d\n",
				result.link_measure.tpc_report.link_margin);
		print_out(print, "receive antenna id = %d\n",
				result.link_measure.recv_antenna_id);
		print_out(print, "transmit antenna id = %d\n",
				result.link_measure.tran_antenna_id);
		print_out(print, "RCPI = %d\n", result.link_measure.rcpi);
		print_out(print, "RSNI = %d\n", result.link_measure.rsni);
		break;
	case QCSAPI_NODE_NEIGHBOR_REP:
		if (result.neighbor_report.item_num == 0) {
			print_out(print, "no neighbor report\n");
		} else {
			for (i = 0; i < result.neighbor_report.item_num; i++) {
				print_out(print, "bssid=");
				memcpy(macaddr, result. neighbor_report.items[i].bssid,
						sizeof(qcsapi_mac_addr));
				local_dump_macaddr(print, macaddr);
				print_out(print, "BSSID Info = 0x%x\n",
						result.neighbor_report.items[i].bssid_info);
				print_out(print, "operating class = %d\n",
						result.neighbor_report.items[i].operating_class);
				print_out(print, "channel = %d\n",
						result.neighbor_report.items[i].channel);
				print_out(print, "phy_type = %d\n",
						result.neighbor_report.items[i].phy_type);
			}
		}
		break;
	case QCSAPI_NODE_SGI_CAPS:
		/*
		 * The variable 'result.common[0]' returns the SGI Capability of the node.
		 * If 'result.common[0]' is 0, the station is not SGI capable. If 'result.common[0]'
		 * is non-zero, the station is SGI capable.
		 * The following bitmap represents SGI capabilities in different Bandwidths.
		 * - if bit 0 is set the station is SGI capable in 20MHz
		 * - if bit 1 is set the station is SGI capable in 40MHz
		 * - if bit 2 is set the station is SGI capable in 80MHz
		 * - if bit 3 is set the station is SGI capable in 160MHz
		 */
		print_out(print, "sgi_caps = 0x%x\n", result.common[0]);
		break;
	default:
		print_out(print, "%d\n", *p_param_value);
		break;
	}

	return 0;
}

#define CALL_QCSAPI_NODE_STATS_LABEL_LEN			20

#define CALL_QCSAPI_NODE_STATS_PRINT(_name, _type, _val)	\
	print_out(print, "%-*s%s%" _type "\n",			\
		CALL_QCSAPI_NODE_STATS_LABEL_LEN, _name, ": ", _val)

CALL_QCSAPI(call_qcsapi_wifi_get_node_stats)
{
	int retval;
	qcsapi_unsigned_int node_index = cb->generic_param.index;
	int local_remote = QCSAPI_LOCAL_NODE;
	struct qcsapi_node_stats ns;

	memset(&ns, 0, sizeof(ns));

	if (argc > 0) {
		if (local_parse_loca_remote_flag(print, argv[0], &local_remote) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_get_node_stats(interface, node_index, local_remote, &ns);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (ns.snr < 0)
		ns.snr = (ns.snr - QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
					QCSAPI_RSSI_OR_SNR_FACTOR;
	else
		ns.snr = (ns.snr + QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
					QCSAPI_RSSI_OR_SNR_FACTOR;
	ns.snr = (0 - ns.snr);

	if (ns.rssi < 0)
		ns.rssi = 0;
	else
		ns.rssi = (qcsapi_unsigned_int) (ns.rssi + QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
				QCSAPI_RSSI_OR_SNR_FACTOR;

	CALL_QCSAPI_NODE_STATS_PRINT("tx_bytes", "llu", ns.tx_bytes);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_pkts", "u", ns.tx_pkts);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_discard", "u", ns.tx_discard);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_be", "u", ns.tx_wifi_sent[WMM_AC_BE]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_bk", "u", ns.tx_wifi_sent[WMM_AC_BK]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_vi", "u", ns.tx_wifi_sent[WMM_AC_VI]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_vo", "u", ns.tx_wifi_sent[WMM_AC_VO]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_be", "u", ns.tx_wifi_drop[WMM_AC_BE]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_bk", "u", ns.tx_wifi_drop[WMM_AC_BK]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_vi", "u", ns.tx_wifi_drop[WMM_AC_VI]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_vo", "u", ns.tx_wifi_drop[WMM_AC_VO]);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_err", "u", ns.tx_err);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_unicast", "u", ns.tx_unicast);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_multicast", "u", ns.tx_multicast);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_broadcast", "u", ns.tx_broadcast);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_phy_rate", "u", ns.tx_phy_rate);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_mgmt", "u", ns.tx_mgmt);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_mcs_index", "u", ns.tx_mcs);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_nss", "u", ns.tx_nss);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_bw", "u", ns.tx_bw);
	CALL_QCSAPI_NODE_STATS_PRINT("tx_sgi", "u", ns.tx_sgi);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_bytes", "llu", ns.rx_bytes);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_pkts", "u", ns.rx_pkts);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_discard", "u", ns.rx_discard);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_err", "u", ns.rx_err);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_unicast", "u", ns.rx_unicast);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_multicast", "u", ns.rx_multicast);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_broadcast", "u", ns.rx_broadcast);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_phy_rate", "u", ns.rx_phy_rate);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_mgmt", "u", ns.rx_mgmt);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_ctrl", "u", ns.rx_ctrl);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_mcs_index", "u", ns.rx_mcs);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_nss", "u", ns.rx_nss);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_bw", "u", ns.rx_bw);
	CALL_QCSAPI_NODE_STATS_PRINT("rx_sgi", "u", ns.rx_sgi);
	print_out(print, "%-*s: %d.%d\n",
			CALL_QCSAPI_NODE_STATS_LABEL_LEN,
			"hw_noise", (ns.hw_noise / 10), abs(ns.hw_noise % 10));
	CALL_QCSAPI_NODE_STATS_PRINT("snr", "d", ns.snr);
	CALL_QCSAPI_NODE_STATS_PRINT("rssi", "d", ns.rssi);
	CALL_QCSAPI_NODE_STATS_PRINT("bw", "d", ns.bw);
	print_out(print, "%-*s: %02x:%02x:%02x:%02x:%02x:%02x\n",
			CALL_QCSAPI_NODE_STATS_LABEL_LEN,
			"mac_addr", MAC_ADDR_ARG(ns.mac_addr));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_node_list)
{
	int retval;
	struct qtn_nis_node_list node_list;
	int i;

	memset(&node_list, 0, sizeof(node_list));

	retval = qcsapi_wifi_get_node_list(interface, &node_list);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (node_list.cnt > ARRAY_SIZE(node_list.node)) {
		retval = -E2BIG;
		return report_qcsapi_error(cb, retval);
	}

	for (i = 0; i < node_list.cnt; i++)
		print_out(print, "%3u " MACSTR "\n",
				node_list.node[i].idx, MAC2STR(node_list.node[i].mac_addr));

	return 0;
}

static char *local_node_infoset_get_mfp_desc(uint32_t val)
{
	if (val & RSN_CAP_MFP_REQ)
		return "required";
	else if (val & RSN_CAP_MFP_CAP)
		return "capable";

	return "disabled";
}

static char *local_infoset_print_get_cipher_desc(qcsapi_output *print, uint32_t val,
			uint8_t is_scan)
{
	if (!is_scan) {
		switch (val) {
		case IEEE80211_CIPHER_WEP:
			return "WEP";
		case IEEE80211_CIPHER_TKIP:
			return "TKIP";
		case IEEE80211_CIPHER_AES_OCB:
			return "AES-OCB";
		case IEEE80211_CIPHER_AES_CCM:
			return "AES-CCM";
		case IEEE80211_CIPHER_AES_CMAC:
			return "AES-CMAC";
		case IEEE80211_CIPHER_CKIP:
			return "CKIP";
		case IEEE80211_CIPHER_AES_CCM_256:
			return "AES-CCM-256";
		case IEEE80211_CIPHER_AES_GCM:
			return "AES-GCM";
		case IEEE80211_CIPHER_AES_GCM_256:
			return "AES-GCM-256";
		}
	} else {
		switch (val) {
		case BIT(IEEE80211_CIPHER_WEP):
			return "WEP";
		case BIT(IEEE80211_CIPHER_TKIP):
			return "TKIP";
		case BIT(IEEE80211_CIPHER_AES_OCB):
			return "AES-OCB";
		case BIT(IEEE80211_CIPHER_AES_CCM):
			return "AES-CCM";
		case BIT(IEEE80211_CIPHER_AES_CMAC):
			return "AES-CMAC";
		case BIT(IEEE80211_CIPHER_CKIP):
			return "CKIP";
		case BIT(IEEE80211_CIPHER_AES_CCM_256):
			return "AES-CCM-256";
		case BIT(IEEE80211_CIPHER_AES_GCM):
			return "AES-GCM";
		case BIT(IEEE80211_CIPHER_AES_GCM_256):
			return "AES-GCM-256";
		case BIT(IEEE80211_CIPHER_NONE):
			return "none";
		}
	}

	return "unknown";
}

/*
 * These descriptive names are generally taken from hostapd config.c
 */
static char *local_infoset_get_keymgmt_type(uint32_t val, uint8_t is_scan)
{
	if (!is_scan) {
		switch (val) {
		case RSN_ASE_NONE:
			return "none";
		case RSN_ASE_8021X_UNSPEC:
			return "WPA-EAP";
		case RSN_ASE_8021X_PSK:
			return "WPA-PSK";
		case RSN_ASE_FT_8021X:
			return "FT-EAP";
		case RSN_ASE_FT_PSK:
			return "FT-PSK";
		case RSN_ASE_8021X_SHA256:
			return "WPA-EAP-SHA256";
		case RSN_ASE_8021X_PSK_SHA256:
			return "WPA-PSK-SHA256";
		case RSN_SAE:
			return "SAE";
		case RSN_OWE:
			return "OWE";
		case RSN_ASE_DPP:
			return "DPP";
		}
	} else {
		switch (val) {
		case WPA_KEY_MGMT_NONE:
			return "none";
		case WPA_KEY_MGMT_IEEE8021X:
			return "WPA-EAP";
		case WPA_KEY_MGMT_PSK:
			return "WPA-PSK";
		case WPA_KEY_MGMT_FT_IEEE8021X:
			return "FT-EAP";
		case WPA_KEY_MGMT_FT_PSK:
			return "FT-PSK";
		case WPA_KEY_MGMT_IEEE8021X_SHA256:
			return "WPA-EAP-SHA256";
		case WPA_KEY_MGMT_PSK_SHA256:
			return "WPA-PSK-SHA256";
		case WPA_KEY_MGMT_SAE:
			return "SAE";
		case WPA_KEY_MGMT_OWE:
			return "OWE";
		case WPA_KEY_MGMT_DPP:
			return "DPP";
		case (WPA_KEY_MGMT_SAE | WPA_KEY_MGMT_PSK):
			return "SAE WPA-PSK";
		}
	}

	return "unknown";
}

static char *local_node_infoset_print_type_gi(uint32_t val)
{
	if (val & QTN_NIS_HE_GI_ENABLED) {
		val &= QTN_NIS_HE_GI_MASK;
		if (val == IEEE80211_HE_GI_08US)
			return "0.8us";
		if  (val == IEEE80211_HE_GI_16US)
			return "1.6us";
		if  (val == IEEE80211_HE_GI_32US)
			return "3.2us";
	} else {
		if (val == 1)
			return "Short GI";
		if (val == 0)
			return "Long GI";
	}

	return  "unsupported GI";
}

static const char *local_node_infoset_print_fdr_status(uint32_t val)
{
	static const char * const fdr_status_str[] = {
		[IEEE80211_FDR_STATE_DISABLED] = "Disabled",
		[IEEE80211_FDR_STATE_ENABLED] = "Enabled",
		[IEEE80211_FDR_STATE_ACTIVE] = "Active",
		[IEEE80211_FDR_STATE_STANDBY] = "Standby",
	};

	COMPILE_TIME_ASSERT(ARRAY_SIZE(fdr_status_str) == IEEE80211_FDR_STATE_MAX);
	if (val < ARRAY_SIZE(fdr_status_str))
		return fdr_status_str[val];
	else
		return "Unknown";
}

static const char *local_node_infoset_print_ifname(uint32_t val, char *ifname, int len)
{
	uint32_t radio = val & 0xffff;
	uint32_t if_index = (val & 0xffff0000) >> 16;

	snprintf(ifname, len, "wifi%u_%u", radio, if_index);

	return ifname;
}

static void local_node_infoset_print_type(qcsapi_output *print,
					enum qtn_nis_val_type_s val_type, uint32_t val)
{
	char ifname[IFNAMSIZ] = { 0 };

	switch (val_type) {
	case QTN_NIS_VAL_UNSIGNED:
		print_out(print, "%u\n", val);
		break;
	case QTN_NIS_VAL_SIGNED:
		print_out(print, "%d\n", val);
		break;
	case QTN_NIS_VAL_RSN_CAPS:
		if (val & QTN_NIS_S4_RSN_DISABLED)
			print_out(print, "0x%04x disabled\n", val);
		else
			print_out(print, "0x%04x MFP:%s\n",
					val, local_node_infoset_get_mfp_desc(val));
		break;
	case QTN_NIS_VAL_RSN_UCASTCIPHER:
		print_out(print, "0x%04x %s\n",
			val, local_infoset_print_get_cipher_desc(print, val, 0));
		break;
	case QTN_NIS_VAL_RSN_MCASTCIPHER:
		print_out(print, "0x%04x %s\n",
			val, local_infoset_print_get_cipher_desc(print, val, 0));
		break;
	case QTN_NIS_VAL_RSN_KEYMGMT:
		print_out(print, "0x%04x %s\n", val, local_infoset_get_keymgmt_type(val, 0));
		break;
	case QTN_NIS_VAL_GI_TYPE:
		print_out(print, "%s\n", local_node_infoset_print_type_gi(val));
		break;
	case QTN_NIS_VAL_IFNAME:
		print_out(print, "%s\n",
			local_node_infoset_print_ifname(val, ifname, sizeof(ifname)));
		break;
	case QTN_NIS_VAL_FDR_STATUS:
		print_out(print, "%s\n", local_node_infoset_print_fdr_status(val));
		break;
	case QTN_NIS_VAL_WIFI_MODE:
		print_out(print, "%s\n", local_wifi_mode_str(val));
		break;
	case QTN_NIS_VAL_DBM:
		print_out(print, "%4d.%d\n", ((int)val) / 10, abs((int)val) % 10);
		break;
	}
}

static void local_node_infoset_ru_stats_print(qcsapi_output *print, struct qtn_nis_set *nis)
{
	uint8_t ru_int[QTN_HE_RU_SIZE_MAX] = {0};
	uint8_t ru_frac[QTN_HE_RU_SIZE_MAX] = {0};
	uint64_t total_pkts;
	uint32_t percent;
	int i;

	print_out(print, "%-17s %8s %8s %8s %8s %8s %8s %8s %8s\n", "MAC", "Idx",
			"2MHz", "5MHz", "10MHz", "20MHz", "40MHz", "80MHz", "160MHz");

	total_pkts = 0;
	for (i = 0; i < QTN_HE_RU_SIZE_MAX; i++)
		total_pkts += nis->val[i];

	if (total_pkts) {
		for (i = 0; i < QTN_HE_RU_SIZE_MAX; i++) {
			percent = nis->val[i] * 1000 / total_pkts;
			ru_int[i] = percent / 10;
			ru_frac[i] = percent % 10;
		}
	}

	print_out(print, MACSTR " %8u %6u.%1u %6u.%1u %6u.%1u %6u.%1u %6u.%1u %6u.%1u %6u.%1u\n",
			MAC2STR(nis->mac_addr), nis->node_index, ru_int[0], ru_frac[0],
			ru_int[1], ru_frac[1], ru_int[2], ru_frac[2],
			ru_int[3], ru_frac[3], ru_int[4], ru_frac[4],
			ru_int[5], ru_frac[5], ru_int[6], ru_frac[6]);
}

static int local_node_infoset_print(const call_bundle *cb, const uint16_t set_id,
					struct qtn_nis_set *nis)
{
	qcsapi_output *print = cb->output;
	int i;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qtn_nis_label) <= ARRAY_SIZE(nis->val));
	if (set_id == QTN_SET_ID_PER_RU_PKTS) {
		local_node_infoset_ru_stats_print(print, nis);
		return 0;
	}


	print_out(print, "%-*s: " MACSTR "\n",
			QTN_NIS_LABEL_LEN, "MAC address", MAC2STR(nis->mac_addr));
	print_out(print, "%-*s: %u\n", QTN_NIS_LABEL_LEN, "Node index", nis->node_index);

	for (i = 0; i < ARRAY_SIZE(nis->val); i++) {
		if (QTN_NIS_IS_SET(nis, i)) {
			print_out(print, "%-*s: ", QTN_NIS_LABEL_LEN,
						qtn_nis_label[set_id][i].label);
			local_node_infoset_print_type(print, qtn_nis_label[set_id][i].type,
						nis->val[i]);
		}
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_node_infoset)
{
	int retval;
	struct qtn_nis_set nis;
	qcsapi_mac_addr macaddr;
	uint32_t node_index = 0;
	uint16_t set_id;

	if (qcsapi_str_to_uint32(argv[0], &node_index) < 0 || node_index == 0) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;
	}

	if (!strcmp(argv[1], "rssi"))
		set_id = QTN_SET_ID_FDR_RSSI;
	else if (!strcmp(argv[1], "ru_stats"))
		set_id = QTN_SET_ID_PER_RU_PKTS;
	else if (local_atou16("set id", argv[1], &set_id, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_node_infoset(interface, node_index, macaddr, set_id, &nis);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_node_infoset_print(cb, set_id, &nis);

	return retval;
}

static int local_node_infoset_ru_stats_all_print(const call_bundle *cb, struct qtn_nis_all_set *nis)
{
	qcsapi_output *print = cb->output;
	uint8_t ru_int[QTN_HE_RU_SIZE_MAX] = {0};
	uint8_t ru_frac[QTN_HE_RU_SIZE_MAX] = {0};
	struct qtn_nis_all_node *node;
	uint16_t node_idx = 0;
	uint64_t total_pkts;
	uint32_t percent;
	int node_num;
	int i;

	print_out(print, "%-17s %8s %8s %8s %8s %8s %8s %8s %8s\n", "MAC", "Idx",
			"2MHz", "5MHz", "10MHz", "20MHz", "40MHz", "80MHz", "160MHz");

	for (node_num = 0; node_num < nis->node_cnt; node_num++) {
		node = &nis->node[node_num];
		node_idx = node->node_index;
		total_pkts = 0;
		for (i = 0; i < QTN_HE_RU_SIZE_MAX; i++) {
			total_pkts += node->val[i];
			ru_int[i] = 0;
			ru_frac[i] = 0;
		}

		if (total_pkts) {
			for (i = 0; i < QTN_HE_RU_SIZE_MAX; i++) {
				percent = node->val[i] * 1000 / total_pkts;
				ru_int[i] = percent / 10;
				ru_frac[i] = percent % 10;
			}
		}

		print_out(print, MACSTR " %8u %6u.%u %6u.%u %6u.%u %6u.%u %6u.%u %6u.%u %6u.%u\n",
				MAC2STR(node->mac_addr), node->node_index, ru_int[0], ru_frac[0],
				ru_int[1], ru_frac[1], ru_int[2], ru_frac[2],
				ru_int[3], ru_frac[3], ru_int[4], ru_frac[4],
				ru_int[5], ru_frac[5], ru_int[6], ru_frac[6]);
	}

	/* If the report contains max nodes, there may be more to come. */
	if (nis->node_cnt == ARRAY_SIZE(nis->node))
		return node_idx + 1;

	return 0;
}

static int local_node_infoset_all_print(const call_bundle *cb, const uint16_t set_id,
					struct qtn_nis_all_set *nis)
{
	qcsapi_output *print = cb->output;
	struct qtn_nis_all_node *node;
	uint16_t node_idx = 0;
	int node_num;
	int fld;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qtn_nis_all_label) <= ARRAY_SIZE(nis->node[0].val));

	for (node_num = 0; node_num < nis->node_cnt; node_num++) {
		node = &nis->node[node_num];
		node_idx = node->node_index;
		for (fld = 0; fld < ARRAY_SIZE(node->val); fld++) {
			if (QTN_NIS_IS_SET(node, fld)) {
				print_out(print, MACSTR " %4u %-*s: ",
						MAC2STR(node->mac_addr), node_idx,
						QTN_NIS_LABEL_LEN,
						qtn_nis_all_label[set_id][fld].label);
				local_node_infoset_print_type(print,
						qtn_nis_all_label[set_id][fld].type,
						node->val[fld]);
			}
		}
	}

	/* If the report contains max nodes, there may be more to come. */
	if (nis->node_cnt == ARRAY_SIZE(nis->node))
		return node_idx + 1;

	return 0;
}

static int local_node_infoset_all_print_all_nodes(const call_bundle *cb, const uint16_t set_id,
							struct qtn_nis_all_set *nis)
{
	int retval;
	int first_node = 0;

	do {
		retval = qcsapi_wifi_get_node_infoset_all(cb->interface, first_node,
				set_id, 0, nis);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		if (set_id == QTN_SET_ID_PER_RU_PKTS)
			first_node = local_node_infoset_ru_stats_all_print(cb, nis);
		else
			first_node = local_node_infoset_all_print(cb, set_id, nis);
	} while (first_node);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_node_infoset_all)
{
	struct qtn_nis_all_set nis;
	uint16_t set_id;

	if (!strcmp(argv[0], "ru_stats"))
		set_id = QTN_SET_ID_PER_RU_PKTS;
	else if (local_atou16("set id", argv[0], &set_id, print) < 0)
		return -EINVAL;

	return local_node_infoset_all_print_all_nodes(cb, set_id, &nis);
}

static void local_snprint_bitrate(char *buffer, int len, unsigned int bitrate)
{
	int i = 0;
	char ch[] = { 'k', 'M', 'G' };
	int remainder = 0;
	int val;

	for (i = 0; i < (ARRAY_SIZE(ch) - 1) && bitrate >= 1000; i++) {
		val = bitrate / 1000;
		remainder = (bitrate % 1000);
		bitrate = val;
	}

	if (remainder)
		snprintf(buffer, len, "%u.%1.1d %cb/s", bitrate, remainder, ch[i]);
	else
		snprintf(buffer, len, "%u %cb/s", bitrate, ch[i]);
}

static char *local_sis_infoset_print_type_data_rate(char *buf, int len, uint32_t val)
{
	local_snprint_bitrate(buf, len, val);

	return buf;
}

static char *local_sis_infoset_print_type_sgi_cap(char *buf, uint32_t val)
{
	int space_flag;

	buf[0] = '\0';
	space_flag = 0;
	val &= ((1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ) |
			(1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ) |
			(1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ) |
			(1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ));

	if (val & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ)) {
		strcat(buf, "20MHz");
		space_flag = 1;
	}

	if (val & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ)) {
		if (space_flag)
			strcat(buf, " ");
		strcat(buf, "40MHz");
		space_flag = 1;
	}

	if (val & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ)) {
		if (space_flag)
			strcat(buf, " ");
		strcat(buf, "80MHz");
		space_flag = 1;
	}

	if (val & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ)) {
		if (space_flag)
			strcat(buf, " ");
		strcat(buf, "160MHz");
	}

	if (buf[0] == '\0')
		strcat(buf, "None\n");

	return buf;
}

static char *local_sis_infoset_print_type_htsec_offst(uint32_t val)
{
	if (val == IEEE80211_HTINFO_EXTOFFSET_ABOVE)
		return "Above";

	if (val == IEEE80211_HTINFO_EXTOFFSET_BELOW)
		return "Below";

	return  "None";
}

static char *local_sis_infoset_print_type_band_info(uint32_t val)
{
	if (val == QTN_MUC_BAND_2_4_GHZ)
		return "2.4 GHZ";

	if (val == QTN_MUC_BAND_5_GHZ)
		return "5 GHZ";

	if (val == QTN_MUC_BAND_6_GHZ)
		return "6 GHZ";

	return  "None";
}

static char *local_sis_infoset_print_type_protocol_info(uint32_t val)
{
	switch (val) {
	case IEEE80211_PROTOCOL_WPA1:
		return "WPA1";
	case IEEE80211_PROTOCOL_WPA2:
		return "WPA2";
	case IEEE80211_PROTOCOL_WPA3:
		return "WPA3";
	}

	return "None";
}

static void local_sis_infoset_print_type(qcsapi_output *print, enum qtn_sis_val_type_s val_type,
						uint32_t val)
{
	int len = QTN_SIS_ALL_ENTRY_MAX;
	char buf[len];

	switch (val_type) {
	case QTN_SIS_VAL_UNSIGNED:
		print_out(print, "%u\n", val);
		break;
	case QTN_SIS_VAL_SIGNED:
		print_out(print, "%d\n", val);
		break;
	case QTN_SIS_VAL_HT_SEC_OFFSET:
		print_out(print, "%s\n", local_sis_infoset_print_type_htsec_offst(val));
		break;
	case QTN_SIS_VAL_SGI_CAP:
		print_out(print, "%s\n", local_sis_infoset_print_type_sgi_cap(buf, val));
		break;
	case QTN_SIS_VAL_DATA_RATE:
		print_out(print, "%s\n", local_sis_infoset_print_type_data_rate(buf, len, val));
		break;
	case QTN_SIS_VAL_BAND_INFO:
		print_out(print, "%s\n", local_sis_infoset_print_type_band_info(val));
		break;
	case QTN_SIS_VAL_RSN_UCASTCIPHER:
		print_out(print, "%s\n", local_infoset_print_get_cipher_desc(print, val, 1));
		break;
	case QTN_SIS_VAL_RSN_KEYMGMT:
		print_out(print, "%s\n", local_infoset_get_keymgmt_type(val, 1));
		break;
	case QTN_SIS_VAL_SECURITY_PROTOCOL:
		print_out(print, "%s\n", local_sis_infoset_print_type_protocol_info(val));
		break;
	case QTN_SIS_VAL_PRINT_SKIP:
		break;
	}
}

static int local_sis_infoset_all_print(const call_bundle *cb, const uint16_t set_id,
					struct qtn_sis_all_set *sis)
{
	qcsapi_output *print = cb->output;
	struct qtn_sis_all_ctrl *ctrl;
	struct qtn_sis_all_ap *ap;
	uint16_t ap_idx = 0;
	int ap_num;
	int fld;
	uint16_t ap_cnt;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qtn_sis_all_label) <= (ARRAY_SIZE(sis->ap[0].val)));

	ctrl = &sis->ctrl;
	ap_cnt = ctrl->ap_cnt - ctrl->first_ap_idx;

	for (ap_num = 0; ap_num < ap_cnt; ap_num++) {
		ap = &sis->ap[ap_num];
		ap_idx = ap->ap_idx;
		print_out(print, "AP: %d\n", ap_idx);
		print_out(print, "    %-*s: " MACSTR "\n", QTN_SIS_LABEL_LEN,
					"BSSID", MAC2STR(ap->bssid));
		print_out(print, "    %-*s: %s\n", QTN_SIS_LABEL_LEN, "SSID",
						(char *)(ap->ssid));

		for (fld = 0; fld < ARRAY_SIZE(ap->val); fld++) {
			if (QTN_SIS_IS_SET(ap, fld) && QTN_SIS_ALL_LABEL_PRINTABLE(set_id, fld)) {
				print_out(print, "    %-*s: ", QTN_SIS_LABEL_LEN,
						qtn_sis_all_label[set_id][fld].label);
				local_sis_infoset_print_type(print,
						qtn_sis_all_label[set_id][fld].type,
						ap->val[fld]);
			}
		}

	}

	return 0;
}

static int local_sis_infoset_all_print_all_nodes(const call_bundle *cb, const uint16_t set_id,
							struct qtn_sis_all_set *sis)
{
	qcsapi_output *print = cb->output;
	struct qtn_sis_all_ctrl *ctrl;
	int retval;
	uint16_t first_ap_idx = 0;
	uint32_t is_report_full;

	memset(sis, 0, sizeof(*sis));
	ctrl = &sis->ctrl;

	while (1) {
		retval = qcsapi_wifi_get_sis_infoset_all(cb->interface, set_id, first_ap_idx,
								0, sis);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);

		is_report_full = MS(ctrl->flags, QTN_SIS_ALL_SET_F_REPORT);

		local_sis_infoset_all_print(cb, set_id, sis);

		/* No more sub-reports to retrieve ? */
		if (is_report_full != QTN_SIS_ALL_SET_F_REPORT_FULL)
			break;

		first_ap_idx = ctrl->ap_cnt;
	}

	print_out(print, "Total scan entries: %d\n", ctrl->ap_cnt);

	return 0;
}


CALL_QCSAPI(call_qcsapi_wifi_get_sis_infoset_all)
{
	struct qtn_sis_all_set sis;
	uint16_t set_id;

	if (local_atou16_range("set id", argv[0], &set_id, print, 0,
					QTN_SIS_ALL_SET_ID_MAX - 1) < 0)
		return -EINVAL;

	return local_sis_infoset_all_print_all_nodes(cb, set_id, &sis);
}

static int local_if_infoset_print(const call_bundle *cb, const uint16_t set_id,
				struct qtnis_if_set *infoset)
{
	qcsapi_output *print = cb->output;
	int i;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_qtnis_if_label) < ARRAY_SIZE(infoset->val));

	for (i = 0; i < ARRAY_SIZE(infoset->val); i++) {
		if (QTNIS_IS_SET(infoset, i)) {
			print_out(print, "%-*s: ", QTN_NIS_LABEL_LEN,
				qcsapi_qtnis_if_label[set_id][i].label);
			local_node_infoset_print_type(print,
				qcsapi_qtnis_if_label[set_id][i].type, infoset->val[i]);
		}
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_if_infoset)
{
	int retval;
	struct qtnis_if_set infoset;
	uint16_t set_id;

	if (!strcmp(argv[0], "fdr_status")) {
		set_id = QTNIS_SET_ID_FDR_STATUS;
	} else if (!strcmp(argv[0], "dbvc_status")) {
		set_id = QTNIS_SET_ID_DBVC_STATUS;
	} else if (!strcmp(argv[0], "nfr_status")) {
		set_id = QTNIS_SET_ID_NFR_STATUS;
	} else if (!strcmp(argv[0], "scan_cap")) {
		set_id = QTNIS_SET_ID_SCAN_CAP;
	} else if (!strcmp(argv[0], "cac_cap")) {
		set_id = QTNIS_SET_ID_CAC_CAP;
	} else if (local_atou16("set id", argv[0], &set_id, print) < 0) {
		return -EINVAL;
	}

	retval = qcsapi_wifi_get_if_infoset(interface, set_id, &infoset);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_if_infoset_print(cb, set_id, &infoset);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_max_queued)
{
	int retval;
	uint32_t node_index = cb->generic_param.index;
	int local_remote = QCSAPI_LOCAL_NODE;
	int reset = 0;
	uint32_t val;

	if (argc > 0) {
		if (local_parse_loca_remote_flag(print, argv[0], &local_remote) < 0)
			return -EINVAL;
	}

	if (argc > 1) {
		if (local_atoi_bool("reset", argv[1], &reset, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_get_max_queued(interface, node_index, local_remote, reset, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_associate)
{
	int retval;
	qcsapi_ssid_fmt fmt;
	char *join_ssid;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (argc < 2) {
		join_ssid = argv[0];
		retval = qcsapi_wifi_associate(interface, join_ssid);
	} else {
		if (call_qcsapi_param_name2enum(print, tbl, argv[0], &fmt) < 0)
			return -EINVAL;
		join_ssid = argv[1];
		retval = qcsapi_wifi_associate2(interface, join_ssid, fmt, 0);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_disassociate)
{
	int retval;

	retval = qcsapi_wifi_disassociate(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_disassociate_sta)
{
	int retval;
	qcsapi_mac_addr macaddr = { 0 };

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_disassociate_sta(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_reassociate)
{
	int retval;

	retval = qcsapi_wifi_reassociate(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_update_bss_cfg)
{
	int retval = 0;
	qcsapi_wifi_mode mode;
	const char *ifname;
	const char *ssid;
	const char *param_name;
	const char *param_value;
	const char *param_type;
	int argc_index = 0;
	int api_version = 0;
	qcsapi_ssid_fmt fmt;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (!strcasecmp(argv[argc_index], "ap")) {
		argc_index++;
		ifname = argv[argc_index];
		argc_index++;
		ssid = NULL;
		mode = qcsapi_access_point;
	} else if (!strcasecmp(argv[argc_index], "sta")) {
		argc_index++;
		if (qcsapi_param_name2enum(tbl, argv[argc_index], &fmt) == 0) {
			api_version = 1;
			argc_index++;
		}
		ifname = interface;
		ssid = argv[argc_index];
		argc_index++;
		mode = qcsapi_station;
	} else {
		return report_qcsapi_error(cb, -qcsapi_invalid_wifi_mode);
	}

	if (argc >= argc_index + 2) {
		param_name = argv[argc_index];
		argc_index++;
		param_value = argv[argc_index];
		argc_index++;
		if (argc > argc_index)
			param_type = argv[argc_index];
		else
			param_type = NULL;
		if (api_version)
			retval = qcsapi_wifi_update_bss2_cfg(ifname, mode, ssid, fmt, param_name,
						param_value, param_type);
		else
			retval = qcsapi_wifi_update_bss_cfg(ifname, mode, ssid, param_name,
						param_value, param_type);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bss_cfg)
{
	int retval = 0;
	qcsapi_wifi_mode mode;
	const char *ifname;
	const char *ssid;
	const char *param_name;
	int argc_index = 0;
	int api_version = 0;
	qcsapi_ssid_fmt fmt = qcsapi_ssid_fmt_str;
	string_256 val = { '\0' };
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (!strcasecmp(argv[argc_index], "ap")) {
		argc_index++;
		ifname = argv[argc_index];
		argc_index++;
		ssid = NULL;
		mode = qcsapi_access_point;
	} else if (!strcasecmp(argv[argc_index], "sta")) {
		argc_index++;
		if (call_qcsapi_param_name2enum(print, tbl, argv[argc_index], &fmt) == 0) {
			api_version = 1;
			argc_index++;
		}
		ifname = interface;
		ssid = argv[argc_index];
		argc_index++;
		mode = qcsapi_station;
	} else {
		return report_qcsapi_error(cb, -qcsapi_invalid_wifi_mode);
	}

	if (argc != argc_index + 1)
		return qcsapi_report_usage(cb);

	param_name = argv[argc_index];
	if (api_version)
		retval = qcsapi_wifi_get_bss2_cfg(ifname, mode, ssid, fmt, param_name, val,
							sizeof(val));
	else
		retval = qcsapi_wifi_get_bss_cfg(ifname, mode, ssid, param_name, val, sizeof(val));

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_SSID_create_SSID)
{
	int retval;
	qcsapi_ssid_fmt fmt;
	char *new_ssid;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (argc < 2) {
		new_ssid = argv[0];
		retval = qcsapi_SSID_create_SSID(interface, new_ssid);
	} else {
		if (call_qcsapi_param_name2enum(print, tbl, argv[0], &fmt) < 0)
			return -EINVAL;
		new_ssid = argv[1];
		retval = qcsapi_SSID_create_SSID2(interface, new_ssid, fmt);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_remove_SSID)
{
	int retval;
	qcsapi_ssid_fmt fmt;
	char *del_SSID;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (argc < 2) {
		del_SSID = argv[0];
		retval = qcsapi_SSID_remove_SSID(interface, del_SSID);
	} else {
		if (call_qcsapi_param_name2enum(print, tbl, argv[0], &fmt) < 0) {
			return -EINVAL;
		} else {
			del_SSID = argv[1];
			retval = qcsapi_SSID_remove_SSID2(interface, del_SSID, fmt);
		}
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_verify_SSID)
{
	int retval;
	qcsapi_ssid_fmt fmt;
	char *existing_SSID;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (argc < 2) {
		existing_SSID = argv[0];
		retval = qcsapi_SSID_verify_SSID(interface, existing_SSID);
	} else {
		if (call_qcsapi_param_name2enum(print, tbl, argv[0], &fmt) < 0) {
			return -EINVAL;
		} else {
			existing_SSID = argv[1];
			retval = qcsapi_SSID_verify_SSID2(interface, existing_SSID, fmt);
		}
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_rename_SSID)
{
	qcsapi_ssid_fmt curr_fmt;
	qcsapi_ssid_fmt new_fmt;
	int argc_index;
	char *new_ssid;
	char *curr_SSID;
	int retval;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	argc_index = 0;
	curr_fmt = qcsapi_ssid_fmt_str;
	new_fmt = qcsapi_ssid_fmt_str;
	if (argc < 3) {
		curr_SSID = argv[argc_index++];
		new_ssid = argv[argc_index];
		retval = qcsapi_SSID_rename_SSID(interface, curr_SSID, new_ssid);
	} else {
		if (call_qcsapi_param_name2enum(print, tbl, argv[argc_index], &curr_fmt) == 0)
			argc_index++;
		curr_SSID = argv[argc_index];
		argc_index++;
		if (call_qcsapi_param_name2enum(print, tbl, argv[argc_index], &new_fmt) == 0) {
			argc_index++;
			if (argc < argc_index + 1)
				return qcsapi_report_usage(cb);
		}
		new_ssid = argv[argc_index];
		retval = qcsapi_SSID_rename_SSID2(interface, curr_SSID,
					curr_fmt, new_ssid, new_fmt);
	}

	return qcsapi_report_complete(cb, retval);
}

static int local_SSID_get_SSID_list(const call_bundle *cb, int argc, char *argv[],
		int api_version)
{
	qcsapi_output *print = cb->output;

	/*
	 * array_SSIDs has the space that receives the SSIDs from the API.
	 * Let this get as large as required, without affecting the integrity of the stack.
	 */
	static qcsapi_SSID2 array_ssids[QCSAPI_SSID_LIST_SIZE_MAX];

	int retval;
	unsigned int iter;
	qcsapi_unsigned_int sizeof_list = QCSAPI_SSID_LIST_SIZE_DEFAULT;
	char *list_ssids[QCSAPI_SSID_LIST_SIZE_MAX];
	struct qcsapi_data_32bytes list_fmt;

	if (argc > 0) {
		if (!isdigit(*argv[0]))
			return qcsapi_report_usage(cb);

		sizeof_list = atoi(argv[0]);

		if (sizeof_list > QCSAPI_SSID_LIST_SIZE_MAX)
			return qcsapi_report_usage(cb);
	}

	for (iter = 0; iter < sizeof_list; iter++) {
		list_ssids[iter] = array_ssids[iter];
		*list_ssids[iter] = '\0';
	}
	if (api_version == 0)
		retval = qcsapi_SSID_get_SSID_list(cb->interface, sizeof_list, list_ssids);
	else
		retval = qcsapi_SSID_get_SSID2_list(cb->interface, sizeof_list, list_ssids,
							&list_fmt);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (iter = 0; iter < sizeof_list; iter++) {
		if (list_ssids[iter] == NULL || *list_ssids[iter] == '\0')
			break;
		if (api_version == 0)
			print_out(print, "%s\n", list_ssids[iter]);
		else
			print_out(print, (list_fmt.data[iter] == qcsapi_ssid_fmt_str) ?
						"\"%s\"\n" : "%s\n", list_ssids[iter]);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_SSID_get_SSID_list)
{
	return local_SSID_get_SSID_list(cb, argc, argv, 0);
}

CALL_QCSAPI(call_qcsapi_SSID_get_SSID2_list)
{
	return local_SSID_get_SSID_list(cb, argc, argv, 1);
}

CALL_QCSAPI(call_qcsapi_SSID_get_protocol)
{
	string_16 buf;
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_get_protocol(interface, ssid, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_get_encryption_modes)
{
	char buf[36] = { 0 };
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_get_encryption_modes(interface, ssid, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_get_group_encryption)
{
	char buf[36] = { 0 };
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_get_group_encryption(interface, ssid, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_get_authentication_mode)
{
	char buf[36];
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_get_authentication_mode(interface, ssid, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_set_protocol)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_set_protocol(interface, ssid, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_set_encryption_modes)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_set_encryption_modes(interface, ssid, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_set_group_encryption)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_set_group_encryption(interface, ssid, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_set_authentication_mode)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_set_authentication_mode(interface, ssid, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_get_pre_shared_key)
{
	char buf[68] = { 0 };
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;
	qcsapi_unsigned_int index = (qcsapi_unsigned_int) atoi(argv[0]);

	retval = qcsapi_SSID_get_pre_shared_key(interface, ssid, index, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_get_key_passphrase)
{
	char buf[68] = { 0 };
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;
	qcsapi_unsigned_int index = (qcsapi_unsigned_int) atoi(argv[0]);

	retval = qcsapi_SSID_get_key_passphrase(interface, ssid, index, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_SSID_set_pre_shared_key)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;
	qcsapi_unsigned_int index = (qcsapi_unsigned_int) atoi(argv[0]);

	retval = qcsapi_SSID_set_pre_shared_key(interface, ssid, index, argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_add_radius_auth_server_cfg)
{
	int retval = 0;

	retval = qcsapi_wifi_add_radius_auth_server_cfg(interface, argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_del_radius_auth_server_cfg)
{
	int retval = 0;

	retval = qcsapi_wifi_del_radius_auth_server_cfg(interface, argv[0], argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_radius_auth_server_cfg)
{
	int retval = 0;
	string_1024 radius_auth_server_cfg;

	retval = qcsapi_wifi_get_radius_auth_server_cfg(interface,
			radius_auth_server_cfg);

	return qcsapi_report_str_or_error(cb, retval, radius_auth_server_cfg);
}

CALL_QCSAPI(call_qcsapi_wifi_add_radius_acct_server_cfg)
{
	int retval = 0;

	retval = qcsapi_wifi_add_radius_acct_server_cfg(interface,
			argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_del_radius_acct_server_cfg)
{
	int retval = 0;

	retval = qcsapi_wifi_del_radius_acct_server_cfg(interface, argv[0], argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_radius_acct_server_cfg)
{
	int retval = 0;
	string_1024 radius_server_cfg;

	retval = qcsapi_wifi_get_radius_acct_server_cfg(interface, radius_server_cfg);

	return qcsapi_report_str_or_error(cb, retval, radius_server_cfg);
}

CALL_QCSAPI(call_qcsapi_wifi_set_own_ip_addr)
{
	int retval;
	char *val = argv[0];

	retval = qcsapi_wifi_set_own_ip_addr(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_set_key_passphrase)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;
	qcsapi_unsigned_int index = (qcsapi_unsigned_int) atoi(argv[0]);
	char *val = argv[1];

	retval = qcsapi_SSID_set_key_passphrase(interface, ssid, index, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_get_pmf)
{
	int val = 0;
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	retval = qcsapi_SSID_get_pmf(interface, ssid, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_SSID_set_pmf)
{
	int retval;
	const char *ssid = cb->generic_param.parameter_type.SSID;

	qcsapi_unsigned_int val = atoi(argv[0]);

	retval = qcsapi_SSID_set_pmf(interface, ssid, val);
	if (retval == -qcsapi_daemon_socket_error)
		return 0;

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_SSID_get_wps_SSID)
{
	qcsapi_SSID buf = "";
	int retval;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		retval = qcsapi_SSID_get_wps_SSID(interface, NULL);
	else
		retval = qcsapi_SSID_get_wps_SSID(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

static int local_wifi_vlanid_valid(char *vlanid, int all)
{
	int vid;

	if (all && !strcasecmp(vlanid, "all"))
		return QVLAN_VID_ALL;

	vid = atoi(vlanid);
	if (vid >= 1 && vid < QVLAN_VID_MAX)
		return vid;
	else
		return -EFAULT;
}

static int local_wifi_vlan_parser(char *argv, int cmd)
{
	if (!strcasecmp(argv, "default"))
		cmd |= e_qcsapi_vlan_pvid;
	else if (!strcasecmp(argv, "tag"))
		cmd |= e_qcsapi_vlan_tag;
	else if (!strcasecmp(argv, "untag"))
		cmd |= e_qcsapi_vlan_untag;
	else if (!strcasecmp(argv, "delete"))
		cmd |= e_qcsapi_vlan_del;
	else if (!strcasecmp(argv, "vlan_prio"))
		cmd |= e_qcsapi_vlan_prio;
	else if (!strcasecmp(argv, "drop_ctagged"))
		cmd |= e_qcsapi_vlan_drop_ctagged;
	else
		cmd = 0;

	return cmd;
}

CALL_QCSAPI(call_qcsapi_wifi_vlan_config)
{
	int retval;
	qcsapi_vlan_cmd cmd = 0;
	int vlanid = 0;
	int index;

	if (argc < 2) {
		if (!strcasecmp(argv[0], "enable"))
			cmd = e_qcsapi_vlan_enable;
		else if (!strcasecmp(argv[0], "disable"))
			cmd = e_qcsapi_vlan_disable;
		else if (!strcasecmp(argv[0], "reset"))
			cmd = e_qcsapi_vlan_reset;
	} else if (!strcasecmp(argv[0], "bind")) {
		vlanid = local_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc == 2)
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
	} else if (!strcasecmp(argv[0], "unbind")) {
		vlanid = local_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc == 2)
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_del |
					e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
	} else if (!strcasecmp(argv[0], "dynamic")) {
		if (argc == 2) {
			if (atoi(argv[1]))
				cmd = e_qcsapi_vlan_dynamic;
			else
				cmd = e_qcsapi_vlan_undynamic;
		}
	} else if (!strcasecmp(argv[0], "access")) {
		vlanid = local_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc <= 3) {
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
			for (index = 2; index < argc; index++)
				cmd = local_wifi_vlan_parser(argv[index], cmd);
		}
	} else if (!strcasecmp(argv[0], "trunk") || !strcasecmp(argv[0], "hybrid")) {
		vlanid = local_wifi_vlanid_valid(argv[1], 1);
		if (vlanid >= 0 && argc <= 5) {
			cmd = e_qcsapi_vlan_trunk;
			for (index = 2; index < argc; index++)
				cmd = local_wifi_vlan_parser(argv[index], cmd);
			if ((vlanid == QVLAN_VID_ALL) && (cmd & e_qcsapi_vlan_pvid))
				cmd = 0;
		}
	} else if (!strcasecmp(argv[0], "drop_stag")) {
		if (argc > 2) {
			cmd = 0;
		} else {
			uint32_t enable;

			if (local_atou_bool("enable", argv[1], &enable, print) < 0)
				return -EINVAL;

			if (enable)
				cmd = e_qcsapi_vlan_drop_stag;
			else
				cmd = e_qcsapi_vlan_undrop_stag;
		}
	} else if (!strcasecmp(argv[0], "ethertype")) {
		if (argc == 2 || argc == 3) {
			uint32_t ethertype;

			if (argc == 2)
				cmd = e_qcsapi_vlan_ethertype_add;
			else if (argc == 3 && !strcasecmp(argv[2], "delete"))
				cmd = e_qcsapi_vlan_ethertype_del;
			else
				return -EINVAL;

			if (local_hexstr_to_uint32_range(argv[0], argv[1], &ethertype, print,
							ETHER_MAX_LEN, 0xffff) < 0) {
				return -EINVAL;
			}

			vlanid = ethertype;
		}
	}
	retval = qcsapi_wifi_vlan_config(interface, cmd, vlanid);
	if (retval == -qcsapi_param_value_invalid) {
		qcsapi_report_usage(cb);
		return -EINVAL;
	}

	return qcsapi_report_complete(cb, retval);
}

static void local_wifi_print_vlan_config(const call_bundle *cb, const char *ifname,
						struct qcsapi_data_2Kbytes *byte)
{
	qcsapi_output *print = cb->output;
	struct qtn_vlan_config *vcfg = (struct qtn_vlan_config *)byte;
	uint16_t vmode;
	uint16_t vid;
	uint16_t i;
	uint16_t j;
	uint32_t tagrx;
	uint8_t drop_stag;

	if (vcfg->vlan_cfg) {
		vmode = ((vcfg->vlan_cfg & QVLAN_MASK_MODE) >> QVLAN_SHIFT_MODE);
		vid = (vcfg->vlan_cfg & QVLAN_MASK_VID);
		drop_stag = vcfg->drop_stag;
	} else {
		print_out(print, "tagrx VLAN:");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			tagrx = qtn_vlan_get_tagrx(vcfg->u.tagrx_config, i);
			if (tagrx) {
				if ((j++ & 0xF) == 0)
					print_out(print, "\n\t");
				print_out(print, "%u-%u, ", i, tagrx);
			}
		}
		print_out(print, "\n");
		return;
	}

	switch (vmode) {
	case QVLAN_MODE_TRUNK:
		print_out(print, "%s, default VLAN %u\n", QVLAN_MODE_STR_TRUNK, vid);

		print_out(print, "    Member of VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");

		print_out(print, "    Untagged VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i) &&
					is_clr_a(vcfg->u.dev_config.tag_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");

		print_out(print, "    Use VLAN priority VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i) &&
					is_set_a(vcfg->u.dev_config.prio_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");
		break;
	case QVLAN_MODE_ACCESS:
		print_out(print, "%s, VLAN %u%s\n", QVLAN_MODE_STR_ACCESS, vid,
				is_set_a(vcfg->u.dev_config.prio_bitmap,
						vid) ? ", Use VLAN priority" : "");
		print_out(print, "    Drop C-tagged packets: %u\n",
					!!(vcfg->flags & QVLAN_DEV_F_DROPCTAGGED));
		break;
	case QVLAN_MODE_DYNAMIC:
		print_out(print, "%s\n", QVLAN_MODE_STR_DYNAMIC);
		break;
	default:
		print_out(print, "VLAN disabled\n");
		break;
	}

	print_out(print, "Drop S-tagged packets: %u\n", drop_stag);
	print_out(print, "Ethertype list: ");
	if (vcfg->vlan_ethertype_count == 0xffff) {
		print_out(print, "%s\n", "0xffff");
	} else if (vcfg->vlan_ethertype_count == 0) {
		print_out(print, "%s\n", "0");
	} else {
		for (i = 0; i < ARRAY_SIZE(vcfg->vlan_ethertype); i++) {
			if (vcfg->vlan_ethertype[i])
				print_out(print, "0x%04x,", vcfg->vlan_ethertype[i]);
		}
		print_out(print, "\n");
	}
}

CALL_QCSAPI(call_qcsapi_wifi_show_vlan_config)
{
	int retval = 0;
	struct qtn_vlan_config *vcfg;

	vcfg = local_malloc(cb, sizeof(struct qcsapi_data_2Kbytes));
	if (!vcfg)
		return -ENOMEM;

	if (argc == 1 && !strcmp(argv[0], "tagrx")) {
		retval = qcsapi_wifi_show_vlan_config(interface,
				(struct qcsapi_data_2Kbytes *)vcfg, argv[0]);
		qtn_vlan_config_ntohl(vcfg, 1);
	} else if (argc == 0) {
		retval = qcsapi_wifi_show_vlan_config(interface,
				(struct qcsapi_data_2Kbytes *)vcfg, NULL);
		qtn_vlan_config_ntohl(vcfg, 0);
	} else {
		retval = -EINVAL;
	}

	if (retval < 0) {
		report_qcsapi_error(cb, retval);
	} else {
		local_wifi_print_vlan_config(cb, interface, (struct qcsapi_data_2Kbytes *)vcfg);
		retval = 0;
	}

	free(vcfg);

	return retval;
}

CALL_QCSAPI(call_qcsapi_set_vlan_promisc)
{
	int retval;
	int enable = !!atoi(argv[0]);

	retval = qcsapi_wifi_set_vlan_promisc(enable);

	return qcsapi_report_complete(cb, retval);
}

static int local_set_multicast(const call_bundle *cb, int add, int argc, char *argv[])
{
	int retval;
	qcsapi_output *print = cb->output;
	uint32_t ipaddr;
	uint32_t ipaddr_ne;
	qcsapi_mac_addr macaddr = { 0 };

	/* FIXME subnets and IPv6 are not yet supported */

	if (inet_pton(AF_INET, argv[0], &ipaddr_ne) != 1) {
		/* FIXME support IPv6 */
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	ipaddr = ntohl(ipaddr_ne);

	if (!IN_MULTICAST(ipaddr)) {
		print_err(print, "invalid multicast IPv4 address " NIPQUAD_FMT "\n",
				NIPQUAD(ipaddr_ne));
		return -EINVAL;
	}

	if (local_parse_macaddr(cb, argv[1], macaddr) < 0)
		return -EINVAL;

	if (add)
		retval = qcsapi_wifi_add_multicast(ipaddr, macaddr);
	else
		retval = qcsapi_wifi_del_multicast(ipaddr, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_add_multicast)
{
	return local_set_multicast(cb, 1, argc, argv);
}

CALL_QCSAPI(call_qcsapi_del_multicast)
{
	return local_set_multicast(cb, 0, argc, argv);
}

#define QCSAPI_FWT_GET_MAX	4096

CALL_QCSAPI(call_qcsapi_get_multicast_list)
{
	int retval;
	char buf[QCSAPI_FWT_GET_MAX];

	retval = qcsapi_wifi_get_multicast_list(buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

static int local_set_ipff(const call_bundle *cb, int add, int argc, char *argv[])
{
	int retval;
	qcsapi_output *print = cb->output;
	uint32_t ipaddr;
	uint32_t ipaddr_ne;

	/* FIXME subnets and IPv6 are not yet supported */

	if (inet_pton(AF_INET, argv[0], &ipaddr_ne) != 1) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	ipaddr = ntohl(ipaddr_ne);

	if (!IN_MULTICAST(ipaddr)) {
		print_err(print, "invalid multicast IPv4 address " NIPQUAD_FMT "\n",
				NIPQUAD(ipaddr_ne));
		return -EINVAL;
	}

	if (add) {
		retval = qcsapi_wifi_add_ipff(ipaddr);
	} else {
		retval = qcsapi_wifi_del_ipff(ipaddr);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_add_ipff)
{
	return local_set_ipff(cb, 1, argc, argv);
}

CALL_QCSAPI(call_qcsapi_del_ipff)
{
	return local_set_ipff(cb, 0, argc, argv);
}

CALL_QCSAPI(call_qcsapi_get_ipff)
{
#define QCSAPI_IPFF_GET_MAX	256
	char buf[IP_ADDR_STR_LEN * QCSAPI_IPFF_GET_MAX];

	qcsapi_wifi_get_ipff(buf, sizeof(buf));

	print_out(print, "%s", buf);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_rts_threshold)
{
	int retval;
	qcsapi_unsigned_int rts_threshold;

	retval = qcsapi_wifi_get_rts_threshold(interface, &rts_threshold);
	if (retval >= 0) {
		print_out(print, "%d\n", rts_threshold);
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_set_rts_threshold)
{
	int retval;
	qcsapi_unsigned_int rts_threshold;
	int32_t arg;

	if (sscanf(argv[0], "%d", &arg) != 1) {
		print_err(print, "Error parsing '%s'\n", argv[0]);
		return -EINVAL;
	}

	if (arg < IEEE80211_RTS_MIN) {
		print_err(print, "Value should be non negative\n");
		return -EINVAL;
	}

	rts_threshold = arg;

	retval = qcsapi_wifi_set_rts_threshold(interface, rts_threshold);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_disable_wps)
{
	int retval;
	int disable_wps = atoi(argv[0]);

	retval = qcsapi_wifi_disable_wps(interface, disable_wps);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_start_cca)
{
	int retval;
	int channel;
	int duration;

	channel = atoi(argv[0]);
	duration = atoi(argv[1]);

	retval = qcsapi_wifi_start_cca(interface, channel, duration);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scan_chan_list)
{
	int retval;
	struct qcsapi_data_256bytes buf;
	uint32_t count = 0;
	int i;

	memset(&buf, 0, sizeof(buf));
	retval = qcsapi_wifi_get_scan_chan_list(interface, &buf, &count);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d channels in scan list: ", count);
	for (i = 0; i < count; i++)
		print_out(print, "%d%c",
			buf.data[i], (i < (count - 1)) ? ',' : '\n');

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_scan_chan_list)
{
	int retval;
	struct qcsapi_data_256bytes buf;
	struct qcsapi_data_256bytes *p_chan_list;
	uint32_t count = 0;

	if (strcmp(argv[0], "default") == 0) {
		p_chan_list = NULL;
		count = 0;
	} else {
		p_chan_list = &buf;
		memset(&buf, 0, sizeof(buf));
		retval = local_string_to_u8_list(print, argv[0], buf.data, &count);
		if (retval < 0) {
			print_err(print, "Invalid channel in list\n");
			return -EINVAL;
		}
	}

	retval = qcsapi_wifi_set_scan_chan_list(interface, p_chan_list, count);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_start_scan)
{
	int retval;
	int pick_flags = 0;
	uint32_t algorithm;
	uint32_t chan_set;

	if (argc > 0) {
		while (argc > 0) {
			if (!strcasecmp("reentry", argv[0]))
				pick_flags |= IEEE80211_PICK_REENTRY;
			else if (!strcasecmp("clearest", argv[0]))
				pick_flags |= IEEE80211_PICK_CLEAREST;
			else if (!strcasecmp("no_pick", argv[0]))
				pick_flags |= IEEE80211_PICK_NOPICK;
			else if (!strcasecmp("background", argv[0]))
				pick_flags |= IEEE80211_PICK_NOPICK_BG;
			else if (!strcasecmp("dfs", argv[0]))
				pick_flags |= IEEE80211_PICK_DFS;
			else if (!strcasecmp("non_dfs", argv[0]))
				pick_flags |= IEEE80211_PICK_NONDFS;
			else if (!strcasecmp("all", argv[0]))
				pick_flags |= IEEE80211_PICK_ALL;
			else if (!strcasecmp("flush", argv[0]))
				pick_flags |= IEEE80211_PICK_SCAN_FLUSH;
			else if (!strcasecmp("active", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_ACTIVE;
			else if (!strcasecmp("fast", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_FAST;
			else if (!strcasecmp("normal", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_NORMAL;
			else if (!strcasecmp("slow", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_SLOW;
			else if (!strcasecmp("auto", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_AUTO;
			else if (!strcasecmp("check", argv[0]))
				pick_flags |= IEEE80211_PICK_BG_CHECK;
			else
				return qcsapi_report_usage(cb);
			argc--;
			argv++;
		}
		if (!(pick_flags & IEEE80211_PICK_ALGORITHM_MASK)) {
			print_out(print, "Pick algorithm was not specified\n");
			return -EINVAL;
		}
	}

	algorithm = pick_flags & IEEE80211_PICK_ALGORITHM_MASK;
	chan_set = pick_flags & IEEE80211_PICK_DOMIAN_MASK;

	if (IS_MULTIPLE_BITS_SET(algorithm)) {
		print_out(print, "Only one pick algorithm can be specified\n");
		return -EINVAL;
	}

	if (chan_set) {
		if (IS_MULTIPLE_BITS_SET(chan_set)) {
			print_out(print, "Only one channel set can be specified\n");
			return -EINVAL;
		}
	} else {
		pick_flags |= IEEE80211_PICK_ALL;
	}

	if (pick_flags & IEEE80211_PICK_NOPICK_BG) {
		uint32_t dfs_mode = pick_flags & IEEE80211_PICK_BG_MODE_MASK;

		if (IS_MULTIPLE_BITS_SET(dfs_mode)) {
			print_out(print, "Specify only one background scan mode\n");
			return -EINVAL;
		}
	} else if (pick_flags & IEEE80211_PICK_BG_CHECK) {
		print_out(print, "Check flag is only for QTN background scan\n");
		return -EINVAL;
	}

	retval = qcsapi_wifi_start_scan_ext(interface, pick_flags);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_cancel_scan)
{
	int force = 0;
	int retval;

	if (argc == 1) {
		if (!strcasecmp("force", argv[0])) {
			force = 1;
		} else {
			return qcsapi_report_usage(cb);
		}
	}

	retval = qcsapi_wifi_cancel_scan(interface, force);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scan_status)
{
	int retval;
	int val = 0;

	retval = qcsapi_wifi_get_scan_status(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_cac_status)
{
	int retval;
	int val = 0;

	retval = qcsapi_wifi_get_cac_status(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_wait_scan_completes)
{
	int retval;
	time_t timeout;

	timeout = (time_t) atoi(argv[0]);

	retval = qcsapi_wifi_wait_scan_completes(interface, timeout);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_results_AP_scan)
{
	qcsapi_unsigned_int val = 0;
	int retval;

	retval = qcsapi_wifi_get_results_AP_scan(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_count_APs_scanned)
{
	uint32_t val = 0;
	int retval;

	retval = qcsapi_wifi_get_count_APs_scanned(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_properties_AP)
{
	int retval;
	qcsapi_ap_properties ap_properties;
	qcsapi_unsigned_int ap_index = cb->generic_param.index;
	char mac_addr_string[24];

	retval = qcsapi_wifi_get_properties_AP(interface, ap_index, &ap_properties);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	snprintf(&mac_addr_string[0], sizeof(mac_addr_string), MACFILTERINGMACFMT,
			ap_properties.ap_mac_addr[0], ap_properties.ap_mac_addr[1],
			ap_properties.ap_mac_addr[2], ap_properties.ap_mac_addr[3],
			ap_properties.ap_mac_addr[4], ap_properties.ap_mac_addr[5]);

	print_out(print,
		"\"%s\" %s %d %d %x %d %d %d %u %d %d %d %d %d %d %d %d %d %s %s\n",
		ap_properties.ap_name_SSID, &mac_addr_string[0], ap_properties.ap_channel,
		ap_properties.ap_RSSI, ap_properties.ap_flags, ap_properties.ap_protocol,
		ap_properties.ap_authentication_mode, ap_properties.ap_encryption_modes,
		ap_properties.ap_best_data_rate, ap_properties.ap_wps,
		ap_properties.ap_80211_proto, ap_properties.ap_qhop_role,
		ap_properties.ap_bw, ap_properties.ap_noise, ap_properties.ap_opmode,
		ap_properties.ap_bintval, ap_properties.ap_dtimperiod,
		ap_properties.ap_11b_present, ap_properties.ap_basic_rates,
		ap_properties.ap_support_rates);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_wps_ie_scanned_AP)
{
	int retval;
	struct qcsapi_ie_data ie_data;
	qcsapi_unsigned_int ap_index = cb->generic_param.index;
	char wps_ie_hex[QCSAPI_MAX_IE_INFOLEN * 2 + 1] = { 0 };
	int i;
	int pos;

	retval = qcsapi_wifi_get_wps_ie_scanned_AP(interface, ap_index, &ie_data);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	pos = 0;
	for (i = 0; i < ie_data.ie_len; i++) {
		snprintf(&wps_ie_hex[pos], sizeof(wps_ie_hex) - pos,
				"%02X", ie_data.ie_buf[i]);
		pos += 2;
	}

	print_out(print, "%d %s\n", ie_data.ie_len, wps_ie_hex);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_mcs_rate)
{
	int retval;
	char buf[16];

	retval = qcsapi_wifi_get_mcs_rate(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_mcs_rate)
{
	int retval;
	char *buf = argv[0];

	retval = qcsapi_wifi_set_mcs_rate(interface, buf);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_time_associated_per_association)
{
	int retval = 0;
	qcsapi_unsigned_int val = 0;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_time_associated_per_association(interface, assoc_idx, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_wds_add_peer)
{
	int retval = 0;
	qcsapi_mac_addr macaddr;
	qcsapi_unsigned_int encrypt = 0;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	if (argc > 1) {
		if (strcasecmp(argv[1], "encrypt") == 0)
			encrypt = 1;
		else
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wds_add_peer_encrypt(interface, macaddr, encrypt);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wds_remove_peer)
{
	int retval = 0;
	qcsapi_mac_addr macaddr;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wds_remove_peer(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wds_get_peer_address)
{
	int retval = 0;
	qcsapi_mac_addr peer_address;
	qcsapi_unsigned_int index = 0;
	char temp_peer_address_str[20];

	index = (qcsapi_unsigned_int) atoi(argv[0]);
	retval = qcsapi_wds_get_peer_address(interface, index, peer_address);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	snprintf(&temp_peer_address_str[0], sizeof(temp_peer_address_str),
			MACFILTERINGMACFMT,
			peer_address[0], peer_address[1], peer_address[2],
			peer_address[3], peer_address[4], peer_address[5]);
	print_out(print, "%s\n", temp_peer_address_str);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_wds_set_psk)
{
	int retval = 0;
	qcsapi_mac_addr peer_address;
	char *buf = NULL;

	if (local_parse_macaddr(cb, argv[0], peer_address) < 0)
		return -EINVAL;

	if (!local_is_valid_null_string(argv[1]))
		buf = argv[1];

	retval = qcsapi_wifi_wds_set_psk(interface, peer_address, buf);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wds_set_mode)
{
	int retval = 0;
	qcsapi_mac_addr peer_address;
	int rbs_mode;
	int rbs_mask;

	if (local_parse_macaddr(cb, argv[0], peer_address) < 0)
		return -EINVAL;

	if (strcasecmp(argv[1], "rbs") == 0) {
		rbs_mode = IEEE80211_QTN_WDS_RBS;
		rbs_mask = IEEE80211_QTN_WDS_MASK;
	} else if (strcasecmp(argv[1], "mbs") == 0) {
		rbs_mode = IEEE80211_QTN_WDS_MBS;
		rbs_mask = IEEE80211_QTN_WDS_MASK;
	} else if (strcasecmp(argv[1], "wds") == 0) {
		rbs_mode = IEEE80211_QTN_WDS_ONLY;
		rbs_mask = IEEE80211_QTN_WDS_MASK;
	} else if (strcasecmp(argv[1], "reset") == 0) {
		rbs_mode = 0;
		rbs_mask = IEEE80211_QTN_EXTDR_ALLMASK;
	} else {
		print_err(print, "Invalid WDS mode %s\n", argv[1]);
		return -EINVAL;
	}

	retval = qcsapi_wds_set_mode(interface, peer_address,
					ieee80211_extdr_combinate(rbs_mode, rbs_mask));

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wds_get_mode)
{
	int retval = 0;
	int rbs_mode;
	qcsapi_unsigned_int index = 0;
	const char *mode_str[] = { "mbs", "rbs", "none" };

	index = (qcsapi_unsigned_int) atoi(argv[0]);
	retval = qcsapi_wds_get_mode(interface, index, &rbs_mode);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "wds %s\n", mode_str[rbs_mode]);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_qos_get_param)
{
	int retval = 0;
	int queue = -1;
	int param = -1;
	int ap_bss_flag = 0;
	int val;
	const struct qcsapi_param_name_tbl *tbl;

	tbl = &qcsapi_param_name_qos_queue;
	if (isdigit(*argv[0])) {
		queue = atoi(argv[0]);
	} else if (call_qcsapi_param_name2enum(print, tbl, argv[0], (uint32_t *)&queue) < 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS queues:\n");
			call_qcsapi_param_name_list(print, tbl);
		}
		return -EINVAL;
	}

	tbl = &qcsapi_param_name_qos_param;
	if (isdigit(*argv[1])) {
		param = atoi(argv[1]);
	} else if (call_qcsapi_param_name2enum(print, tbl, argv[0], (uint32_t *)&param) < 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS parameters:\n");
			call_qcsapi_param_name_list(print, tbl);
		}
		return -EINVAL;
	}

	if (argc > 2)
		ap_bss_flag = atoi(argv[2]);

	retval = qcsapi_wifi_qos_get_param(interface, queue, param, ap_bss_flag, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_qos_set_param)
{
	int retval = 0;
	int queue = -1;
	int param = -1;
	int ap_bss_flag = 0;
	int param_value = -1;
	const struct qcsapi_param_name_tbl *tbl;

	tbl = &qcsapi_param_name_qos_queue;
	if (isdigit(*argv[0])) {
		queue = atoi(argv[0]);
	} else if (call_qcsapi_param_name2enum(print, tbl, argv[0], (uint32_t *)&queue) < 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS queues:\n");
			call_qcsapi_param_name_list(print, tbl);
		}
		return -EINVAL;
	}

	tbl = &qcsapi_param_name_qos_param;
	if (isdigit(*argv[1])) {
		param = atoi(argv[1]);
	} else if (call_qcsapi_param_name2enum(print, tbl, argv[0], (uint32_t *)&param) < 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS parameters:\n");
			call_qcsapi_param_name_list(print, tbl);
		}
		return -EINVAL;
	}

	if (isdigit(*argv[2])) {
		param_value = atoi(argv[2]);
	} else {
		print_err(print, "Invalid QoS param value %s\n", argv[2]);
		return -EINVAL;
	}

	if (argc > 3)
		ap_bss_flag = atoi(argv[3]);

	retval = qcsapi_wifi_qos_set_param(interface, queue, param, ap_bss_flag, param_value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_wmm_ac_map)
{
	int retval = 0;
	string_64 buf = { 0 };	/* Must be a string for the RPC generation Perl script */

	assert(sizeof(buf) >= QCSAPI_WIFI_AC_MAP_SIZE);

	retval = qcsapi_wifi_get_wmm_ac_map(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_wmm_ac_map)
{
	int retval = 0;
	int user_prio = -1;
	int ac_index = -1;

	if (isdigit(*argv[0])) {
		user_prio = atoi(argv[0]);
	} else {
		print_err(print, "Invalid user priority %s\n", argv[0]);
		return -EINVAL;
	}

	if (isdigit(*argv[1])) {
		ac_index = atoi(argv[1]);
	} else {
		print_err(print, "Invalid AC index %s\n", argv[1]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_wmm_ac_map(interface, user_prio, ac_index);

	return qcsapi_report_complete(cb, retval);
}

#define QCSAPI_BINARY_CONVERT_MASK	0x20

CALL_QCSAPI(call_qcsapi_wifi_get_dscp_ac_map)
{
	int i;
	int retval = 0;
	struct qcsapi_data_64bytes ac_mapping;
	const char *acstr[] = { "AC_BE", "AC_BK", "AC_VI", "AC_VO" };

	assert(sizeof(ac_mapping) >= IP_DSCP_NUM);

	memset(&ac_mapping, 0, sizeof(ac_mapping));
	retval = qcsapi_wifi_get_dscp_ac_map(interface, &ac_mapping);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);


	print_out(print, "DSCP            AC\n");
	for (i = 0; i < IP_DSCP_NUM; i++) {
		uint8_t mask = QCSAPI_BINARY_CONVERT_MASK;
		/* Print DSCP in binary format */
		while (mask) {
			print_out(print, "%d", i & mask ? 1 : 0);
			mask >>= 1;
		}
		print_out(print, "(0x%02x)    %s\n", i,
				acstr[(uint8_t) ac_mapping.data[i]]);
	}

	return 0;
}

/*
 * Convert given formatted dscp string into digital value
 * Two types of formatted dscp string are acceptable
 * eg,
 * TYPE I  -- 3,4,5,25,38
 * TYPE II -- 3-25
 */
static int local_convert_ipdscp_digital(const char *dscpstr, uint8_t *array, uint8_t *number)
{
	uint8_t ip_dscp_number = 0;
	char *pcur;
	char *p;
	char buffer[256] = { 0 };

	strncpy(buffer, dscpstr, (sizeof(buffer) - 1));
	pcur = buffer;
	do {
		p = strchr(pcur, '-');
		if (p) {
			uint8_t dscpstart;
			uint8_t dscpend;
			int i;

			*p = '\0';
			p++;
			if (!isdigit(*pcur) || !isdigit(*p))
				return -EINVAL;
			dscpstart = atoi(pcur);
			dscpend = atoi(p);

			if ((dscpstart > dscpend) || (dscpstart >= IP_DSCP_NUM)
					|| (dscpend >= IP_DSCP_NUM))
				return -EINVAL;
			ip_dscp_number = dscpend - dscpstart;
			for (i = 0; i <= ip_dscp_number; i++)
				array[i] = dscpstart + i;
			break;
		} else {
			if (ip_dscp_number > (IP_DSCP_NUM - 1))
				return -EINVAL;

			p = strchr(pcur, ',');
			if (p) {
				*p = '\0';
				p++;
				array[ip_dscp_number] = atoi(pcur);
				if (array[ip_dscp_number] >= IP_DSCP_NUM)
					return -EINVAL;
				pcur = p;
				ip_dscp_number++;
			} else {
				array[ip_dscp_number] = atoi(pcur);
				if (array[ip_dscp_number] >= IP_DSCP_NUM)
					return -EINVAL;
			}
		}
	} while (p);
	*number = ip_dscp_number + 1;

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_dscp_ac_map)
{
	int retval = 0;
	uint8_t listlen = 0;
	uint8_t ac = 0;
	struct qcsapi_data_64bytes ip_dscp_value;

	if (!isdigit(*argv[1])) {
		print_err(print, "Invalid AC value %s\n", argv[1]);
		return -EINVAL;
	} else {
		ac = atoi(argv[1]);
	}

	memset(&ip_dscp_value, 0, sizeof(ip_dscp_value));
	retval = local_convert_ipdscp_digital(argv[0], ip_dscp_value.data, &listlen);
	if (retval < 0) {
		print_err(print, "Invalid DSCP list\n");
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_dscp_ac_map(interface, &ip_dscp_value, listlen, ac);

	return qcsapi_report_complete(cb, retval);
}

static void qcsapi_print_dscp_table(qcsapi_output *print, const char *header,
					int convert_to_ac_str, struct qcsapi_data_64bytes *table)
{
	int dscp;
	const char *acstr[] = { "AC_BE", "AC_BK", "AC_VI", "AC_VO" };

	print_out(print, header);
	for (dscp = 0; dscp < IP_DSCP_NUM; dscp++) {
		uint8_t mask = QCSAPI_BINARY_CONVERT_MASK;
		/* Print DSCP in binary format */
		while (mask) {
			print_out(print, "%d", dscp & mask ? 1 : 0);
			mask >>= 1;
		}
		print_out(print, "(0x%02x)    ", dscp);
		if (convert_to_ac_str)
			print_out(print, "%s\n", acstr[table->data[dscp]]);
		else
			print_out(print, "%u\n", table->data[dscp]);
	}
}

CALL_QCSAPI(call_qcsapi_wifi_get_dscp_ac_table)
{
	uint32_t tsel;
	int retval = 0;
	struct qcsapi_data_64bytes dscp_table = { {0} };

	COMPILE_TIME_ASSERT(sizeof(dscp_table) >= IP_DSCP_NUM);

	if (local_atou32("table index", argv[0], &tsel, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_dscp_ac_table(tsel, &dscp_table);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	qcsapi_print_dscp_table(print, "DSCP            AC\n", 1, &dscp_table);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_dscp_ac_table)
{
	int retval = 0;
	uint8_t listlen = 0;
	uint32_t ac = 0;
	uint32_t tsel = 0;
	struct qcsapi_data_64bytes ip_dscp_value = { {0} };

	if (local_atou32("table index", argv[0], &tsel, print) < 0)
		return -EINVAL;

	retval = local_convert_ipdscp_digital(argv[1], ip_dscp_value.data, &listlen);
	if (retval < 0) {
		print_err(print, "Invalid DSCP list\n");
		return -EINVAL;
	}

	if (local_atou32("AC", argv[2], &ac, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dscp_ac_table(tsel, &ip_dscp_value, listlen, ac);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dscp_tid_table)
{
	uint32_t tsel;
	int retval = 0;
	struct qcsapi_data_64bytes dscp_table = { {0} };

	COMPILE_TIME_ASSERT(sizeof(dscp_table) >= IP_DSCP_NUM);

	if (local_atou32("table index", argv[0], &tsel, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_dscp_tid_table(tsel, &dscp_table);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	qcsapi_print_dscp_table(print, "DSCP            TID\n", 0, &dscp_table);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_dscp_tid_table)
{
	int retval = 0;
	uint8_t listlen = 0;
	uint32_t tid = 0;
	uint32_t tsel = 0;
	struct qcsapi_data_64bytes ip_dscp_value = { {0} };

	if (local_atou32("table index", argv[0], &tsel, print) < 0)
		return -EINVAL;

	retval = local_convert_ipdscp_digital(argv[1], ip_dscp_value.data, &listlen);
	if (retval < 0) {
		print_err(print, "Invalid DSCP list\n");
		return -EINVAL;
	}

	if (local_atou32("TID", argv[2], &tid, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dscp_tid_table(tsel, &ip_dscp_value, listlen, tid);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dscp_vap_link)
{
	uint8_t tsel;
	int retval = 0;

	retval = qcsapi_wifi_get_dscp_vap_link(interface, &tsel);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "VAP %s is using DSCP2UP mapping table index %d\n", interface, tsel);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_dscp_vap_link)
{
	int retval = 0;
	uint8_t tsel = 0;

	if (!isdigit(*argv[0])) {
		print_err(print, "Invalid DSCP2UP table index value %s\n", argv[0]);
		return -EINVAL;
	}

	tsel = atoi(argv[0]);

	retval = qcsapi_wifi_set_dscp_vap_link(interface, tsel);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_ac_agg_hold_time)
{
	int retval = 0;
	uint32_t ac = 0;
	uint32_t val = 0;

	if (local_atou32("AC", argv[0], &ac, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_ac_agg_hold_time(interface, ac, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ac_agg_hold_time)
{
	int retval = 0;
	uint32_t ac = 0;
	uint32_t val = 0;

	if (local_atou32("AC", argv[0], &ac, print) < 0)
		return -EINVAL;

	if (local_atou32("AC", argv[1], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ac_agg_hold_time(interface, ac, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_qos_map)
{
	int retval = 0;

	retval = qcsapi_wifi_set_qos_map(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_del_qos_map)
{
	int retval = 0;

	retval = qcsapi_wifi_del_qos_map(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_qos_map)
{
	int retval = 0;
	string_256 value_buf = { 0 };

	retval = qcsapi_wifi_get_qos_map(interface, value_buf);

	return qcsapi_report_str_or_error(cb, retval, value_buf);
}

CALL_QCSAPI(call_qcsapi_wifi_send_qos_map_conf)
{
	int retval = 0;
	qcsapi_mac_addr sta_macaddr = { 0 };

	if (local_parse_macaddr(cb, argv[0], sta_macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_send_qos_map_conf(interface, sta_macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dscp_tid_map)
{
	int retval = 0;
	struct qcsapi_data_64bytes dscp2tid = { {0} };
	int dscp;

	retval = qcsapi_wifi_get_dscp_tid_map(interface, &dscp2tid);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "DSCP: TID\n");
	for (dscp = 0; dscp < ARRAY_SIZE(dscp2tid.data); dscp++)
		print_out(print, "%4d: %u\n", dscp, dscp2tid.data[dscp]);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_priority)
{
	int retval = 0;
	uint8_t val;

	retval = qcsapi_wifi_get_priority(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_priority)
{
	int retval = 0;
	uint8_t val = 0;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
		if (val >= QTN_VAP_PRIORITY_NUM)
			return qcsapi_report_usage(cb);
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_priority(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_airfair)
{
	int retval = 0;
	uint8_t val;

	retval = qcsapi_wifi_get_airfair(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_airfair)
{
	int retval = 0;
	uint8_t val = 0;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
		if (val > 1)
			return qcsapi_report_usage(cb);
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_airfair(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_airquota)
{
	int retval = 0;
	uint32_t val;

	retval = qcsapi_wifi_get_airquota(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_airquota)
{
	int retval = 0;
	uint32_t val = 0;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
		if (val > QTN_QOS_AIRQUOTA_MAX)
			return qcsapi_report_usage(cb);
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_airquota(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_airquota_node)
{
	int retval = 0;
	string_256 buf = { 0 };

	retval = qcsapi_wifi_get_airquota_node(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_airquota_node)
{
	uint16_t val = 0;
	qcsapi_mac_addr macaddr;
	int retval = 0;

	if (local_is_valid_null_string(argv[0])) {
		memset(macaddr, 0, MAC_ADDR_SIZE);
		retval = qcsapi_wifi_set_airquota_node(interface, macaddr, 0);
	} else if (argc == 2) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;

		retval = validate_mac_addr_unicast(macaddr);
		if (retval < 0) {
			print_out(print, "Invalid unicast MAC address\n");
			return -EINVAL;
		}

		if (local_atou16("air quota", argv[1], &val, print) < 0)
			return -EINVAL;

		retval = qcsapi_wifi_set_airquota_node(interface, macaddr, val);
	} else {
		return qcsapi_report_usage(cb);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tpquota_node)
{
	int retval = 0;
	string_256 buf = { 0 };

	retval = qcsapi_wifi_get_tpquota_node(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_tpquota_node)
{
	uint32_t val = 0;
	qcsapi_mac_addr macaddr = { 0 };
	int retval = 0;

	if (strcasecmp("0", argv[0]) == 0) {
		retval = qcsapi_wifi_set_tpquota_node(interface, macaddr, 0);
	} else if (argc == 2) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;

		if (strcasecmp("0", argv[1]) == 0)
			val = 0;
		else if (local_atou32_range("quota", argv[1], &val, print,
					QTN_QOS_TPQUOTA_MIN_MB, QTN_QOS_TPQUOTA_MAX_MB) < 0)
			return -EINVAL;

		if (retval >= 0) {
			retval = qcsapi_wifi_set_tpquota_node(interface, macaddr, val);
		}
	} else {
		return qcsapi_report_usage(cb);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_tpquota_vap)
{
	int retval = 0;
	uint32_t val;

	retval = qcsapi_wifi_get_tpquota_vap(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_tpquota_vap)
{
	int retval = 0;
	uint32_t val = 0;

	if (local_atou32_range("quota", argv[0], &val, print, 0, QTN_QOS_TPQUOTA_MAX_MB) < 0)
		return -EINVAL;

	if (retval >= 0)
		retval = qcsapi_wifi_set_tpquota_vap(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_premier_list)
{
	qcsapi_mac_addr_list mac_addr_list;
	int retval = 0;
	int count = 0;
	int i;
	unsigned char *pmac;

	retval = qcsapi_wifi_get_premier_list(interface, &count, mac_addr_list);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < count; i++) {
		pmac = ((unsigned char *)mac_addr_list) + i * MAC_ADDR_SIZE;
		print_out(print, MACSTR "\n", MAC2STR(pmac));
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_premier_list)
{
	qcsapi_mac_addr_list mac_addr_list;
	int retval = 0;
	int i = 0;

	if (local_is_valid_null_string(argv[0])) {
		retval = qcsapi_wifi_set_premier_list(interface, 0, NULL);
	} else {
		for (i = 0; i < MIN(argc, MAC_ADDR_LIST_SIZE); i++) {
			if (local_parse_macaddr(cb, argv[i], &mac_addr_list[i * MAC_ADDR_SIZE]))
				return -EINVAL;
		}
		retval = qcsapi_wifi_set_premier_list(interface, i, mac_addr_list);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_premier_rule)
{
	int retval = 0;
	qcsapi_qos_premier_rule val;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_qos_premier_rule;

	retval = qcsapi_wifi_get_premier_rule(interface, &val);

	return qcsapi_report_str_or_error(cb, retval, qcsapi_param_enum2name(tbl, val));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_premier_rule)
{
	int retval = 0;
	qcsapi_qos_premier_rule val = 0;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_qos_premier_rule;

	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &val) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_premier_rule(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_qos_class)
{
	uint32_t class;
	uint32_t entity = 0;
	int retval = 0;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_qos_entity;

	retval = call_qcsapi_param_name2enum(print, tbl, argv[0], &entity);
	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &entity) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_qos_class(interface, entity, &class);

	tbl = &qcsapi_param_name_qos_class;
	return qcsapi_report_str_or_error(cb, retval, qcsapi_param_enum2name(tbl, class));
}

CALL_QCSAPI(call_qcsapi_wifi_set_qos_class)
{
	uint32_t class = 0;
	uint32_t entity = 0;
	int retval = 0;
	const struct qcsapi_param_name_tbl *tbl;

	tbl = &qcsapi_param_name_qos_entity;
	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &entity) < 0)
		return -EINVAL;

	tbl = &qcsapi_param_name_qos_class;
	if (call_qcsapi_param_name2enum(print, tbl, argv[1], &class) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_qos_class(interface, entity, class);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_config_get_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *buf;
	size_t size = QCSAPI_MAX_PARAM_VAL_LEN;

	buf = calloc(size, sizeof(char));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_config_get_parameter(interface, name, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_config_update_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *val = argv[1];

	retval = qcsapi_config_update_parameter(interface, name, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_config_get_ssid_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *buf;
	size_t size = QCSAPI_MAX_PARAM_VAL_LEN;

	buf = calloc(size, sizeof(char));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_config_get_ssid_parameter(interface, name, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_config_update_ssid_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *val = argv[1];

	retval = qcsapi_config_update_ssid_parameter(interface, name, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_bootcfg_get_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *buf;
	size_t size = QCSAPI_MAX_PARAM_VAL_LEN + 1;

	buf = calloc(size, sizeof(char));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_bootcfg_get_parameter(name, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_bootcfg_update_parameter)
{
	int retval = 0;
	char *name = argv[0];
	char *val = argv[1];

	retval = qcsapi_bootcfg_update_parameter(name, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_bootcfg_commit)
{
	int retval = 0;

	retval = qcsapi_bootcfg_commit();

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_service_control)
{
	int retval = 0;
	char *name = argv[0];
	char *action = argv[1];
	qcsapi_service_name serv_name;
	qcsapi_service_action serv_action;

	if (strcmp(name, "telnet") == 0)
		name = "inetd";

	retval = qcsapi_get_service_name_enum(name, &serv_name);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	retval = qcsapi_get_service_action_enum(action, &serv_action);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	retval = qcsapi_service_control(serv_name, serv_action);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wfa_cert)
{
	int retval = 0;
	uint16_t enable = 1;

	if (local_atou16("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wfa_cert_mode_enable(enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_scs_enable)
{
	int retval = 0;
	uint16_t enable = 1;

	if (local_atou16("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_scs_enable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_scs_set_version)
{
	int retval = 0;
	uint16_t version = 1;

	if (local_atou16("version", argv[0], &version, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_scs_set_version(interface, version);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_scs_switch_channel)
{
	int retval = 0;
	uint16_t pick_flags = 0;
	int check_margin = 0;

	if (argc > 0) {
		if (!strcasecmp("dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY;
		} else if (!strcasecmp("non_dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_NON_DFS_ONLY;
		} else if (!strcasecmp("all", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
		} else {
			return qcsapi_report_usage(cb);
		}

		if (argc == 2 && local_atoi_bool("check margin", argv[1], &check_margin, print) < 0)
			return -EINVAL;
		if (check_margin == 0)
			pick_flags |= IEEE80211_SCS_PICK_ANYWAY;
	} else {
		pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL | IEEE80211_SCS_PICK_ANYWAY;
	}

	retval = qcsapi_wifi_scs_switch_channel(interface, pick_flags);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_scs_pick_best_channel)
{
	int retval = 0;
	uint16_t pick_flags = IEEE80211_SCS_NOPICK |
			IEEE80211_SCS_PICK_ANYWAY | IEEE80211_SCS_PICK_ALLOW_CURRENT;
	int check_margin = 0;
	int chan = 0;

	if (argc > 0) {
		if (!strcasecmp("dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY;
		} else if (!strcasecmp("non_dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_NON_DFS_ONLY;
		} else if (!strcasecmp("all", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
		} else {
			return qcsapi_report_usage(cb);
		}

		if (argc == 2 && local_atoi_bool("check margin", argv[1], &check_margin, print) < 0)
			return -EINVAL;
		if (check_margin == 1)
			pick_flags &= ~(IEEE80211_SCS_PICK_ANYWAY |
					IEEE80211_SCS_PICK_ALLOW_CURRENT);
	} else {
		pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
	}

	retval = qcsapi_wifi_scs_pick_best_channel(interface, pick_flags, &chan);

	return qcsapi_report_int_or_error(cb, retval, chan);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_verbose)
{
	int retval = 0;
	uint16_t enable = 1;

	if (argc > 0)
		enable = (uint16_t) atoi(argv[0]);

	retval = qcsapi_wifi_set_scs_verbose(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_status)
{
	int retval = 0;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_scs_status(interface, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (val == 1)
		print_out(print, "Enabled (%d)\n", val);
	else if (val == 0)
		print_out(print, "Disabled (%d)\n", val);
	else
		print_out(print, "Unknown (%d)\n", val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_smpl_enable)
{
	int retval = 0;
	uint16_t enable = 1;

	if (argc > 0) {
		enable = (uint16_t) atoi(argv[0]);
	}

	retval = qcsapi_wifi_set_scs_smpl_enable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_active_chan_list)
{
	int retval = 0;
	struct qcsapi_scs_chan_list list;
	uint32_t ch_count;
	uint32_t enable;

	if (local_atou_bool("enable", argv[1], &enable, print) < 0)
		return qcsapi_report_usage(cb);

	memset(&list, 0, sizeof(list));
	retval = local_string_to_u8_list(print, argv[0], list.chan, &ch_count);
	if (retval < 0) {
		print_err(print, "Invalid channel in list\n");
		return -EINVAL;
	}

	list.num = ch_count;

	retval = qcsapi_wifi_set_scs_active_chan_list(interface, &list, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_active_chan_list)
{
	int retval = 0;
	struct qcsapi_scs_chan_list list;
	int i;

	memset(&list, 0, sizeof(list));

	retval = qcsapi_wifi_get_scs_active_chan_list(interface, &list);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "Number of SCS enabled channels: %u\n", list.num);
	for (i = 0; i < list.num; i++)
		print_out(print, "%d ", list.chan[i]);
	print_out(print, "\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_smpl_dwell_time)
{
	int retval = 0;
	uint16_t val = 0;

	if (local_atou16("duration", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_smpl_dwell_time(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_smpl_dwell_time)
{
	int retval;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_scs_smpl_dwell_time(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_sample_intv)
{
	int retval = 0;
	uint16_t val = 0;

	if (local_atou16("duration", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_sample_intv(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_sample_type)
{
	int retval = 0;
	uint16_t val = 0;

	if (local_atou16("type", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_sample_type(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_sample_intv)
{
	int retval;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_scs_sample_intv(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_intf_detect_intv)
{
	int retval = 0;
	uint16_t intv = 0;

	if (local_atou16("duration", argv[0], &intv, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_intf_detect_intv(interface, intv);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_thrshld)
{
	int retval = 0;
	char *thrshld_param_name = argv[0];
	uint16_t thrshld_value;

	if (local_atou16("threshold", argv[1], &thrshld_value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_thrshld(interface, thrshld_param_name, thrshld_value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_report_only)
{

	int retval = 0;
	uint16_t report_value = 0;

	report_value = atoi(argv[0]);

	retval = qcsapi_wifi_set_scs_report_only(interface, report_value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_report)
{
	int retval;
	int i;

	if (strcmp(argv[0], "current") == 0) {
		struct qcsapi_scs_currchan_rpt rpt;

		retval = qcsapi_wifi_get_scs_currchan_report(interface, &rpt);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		print_out(print, "SCS: chan %d try %u idle=%u busy %u intf %u ",
			rpt.chan, rpt.cca_try, rpt.cca_idle, rpt.cca_busy, rpt.cca_intf);
		print_out(print, "tx %u tx_ms %u rx_ms %u pmbl_cnt %u\n",
			rpt.cca_tx, rpt.tx_ms, rpt.rx_ms, rpt.pmbl);
	} else if (strcmp(argv[0], "all") == 0) {
		struct qcsapi_scs_ranking_rpt rpt;
		const char *str[] = QTN_CHAN_AVAIL_STATUS_TO_STR;

		retval = qcsapi_wifi_get_scs_stat_report(interface, &rpt);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		print_out(print, "SCS ranking report: chan number = %u\n", rpt.num);
		print_out(print, "chan dfs txpower  numbeacon cca_intf     metric    ");
		print_out(print, "pmbl_ap   pmbl_sta   age duration times status\n");
		for (i = 0; i < rpt.num; i++)
			print_out(print,
				"%4d %3d %7d %10u %8u %10d %10d %10d %5u %8u %5u %s\n",
				rpt.chan[i], rpt.dfs[i], rpt.txpwr[i], rpt.numbeacons[i],
				rpt.cca_intf[i], rpt.metric[i], rpt.pmbl_ap[i],
				rpt.pmbl_sta[i], rpt.metric_age[i], rpt.duration[i],
				rpt.times[i], str[rpt.chan_avail_status[i]]);
	} else if (strcmp(argv[0], "autochan") == 0) {
		struct qcsapi_autochan_rpt rpt;

		retval = qcsapi_wifi_get_autochan_report(interface, &rpt);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		print_out(print, "AP: initial auto channel ranking table: chan number = %u\n",
				rpt.num);
		print_out(print, "chan dfs txpower  numbeacon        cci        aci     metric\n");
		for (i = 0; i < rpt.num; i++) {
			print_out(print, "%4d %3d %7d %10u %10d %10d %10d\n",
				rpt.chan[i], rpt.dfs[i], rpt.txpwr[i],
				rpt.numbeacons[i], rpt.cci[i], rpt.aci[i], rpt.metric[i]);
		}
	} else if (strcmp(argv[0], "score") == 0) {
		struct qcsapi_scs_score_rpt rpt;

		retval = qcsapi_wifi_get_scs_score_report(interface, &rpt);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		print_out(print, "SCS score report: channel number = %u\n", rpt.num);
		print_out(print, "channel  score\n");
		for (i = 0; i < rpt.num; i++)
			print_out(print, "%4d  %5d\n", rpt.chan[i], rpt.score[i]);
	} else if (strcmp(argv[0], "interference") == 0) {
#define INTF_NUM		6
#define SCS_CCA_INTF_INVALID	0xFFFF
		struct qcsapi_scs_interference_rpt rpt;
		char cca_intf20[INTF_NUM];
		char cca_intf40[INTF_NUM];
		char cca_intf80[INTF_NUM];

		retval = qcsapi_wifi_get_scs_interference_report(interface, &rpt);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		print_out(print, "SCS ranking report: chan number = %u\n", rpt.num);
		print_out(print, "chan cca_intf_20 cca_intf_40 cca_intf_80\n");
		for (i = 0; i < rpt.num; i++) {
			snprintf(cca_intf20, INTF_NUM, "%u", rpt.cca_intf_20[i]);
			snprintf(cca_intf40, INTF_NUM, "%u", rpt.cca_intf_40[i]);
			snprintf(cca_intf80, INTF_NUM, "%u", rpt.cca_intf_80[i]);
			print_out(print, "%4d %11s %11s %11s\n",
				rpt.chan[i],
				rpt.cca_intf_20[i] == SCS_CCA_INTF_INVALID ? "-" : cca_intf20,
				rpt.cca_intf_40[i] == SCS_CCA_INTF_INVALID ? "-" : cca_intf40,
				rpt.cca_intf_80[i] == SCS_CCA_INTF_INVALID ? "-" : cca_intf80);
		}
	} else {
		return qcsapi_report_usage(cb);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_cca_intf_smth_fctr)
{
	int retval = 0;
	uint8_t fctr_noxp = 0;
	uint8_t fctr_xped = 0;

	fctr_noxp = atoi(argv[0]);
	fctr_xped = atoi(argv[1]);

	retval = qcsapi_wifi_set_scs_cca_intf_smth_fctr(interface, fctr_noxp, fctr_xped);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_stats)
{
	int retval = 0;
	uint16_t start = 1;

	if (argc > 0)
		start = (uint16_t) atoi(argv[0]);

	retval = qcsapi_wifi_set_scs_stats(interface, start);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_burst_enable)
{
	int retval = 0;
	uint16_t enable = 0;

	if (local_atou16("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_burst_enable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_burst_window)
{
	int retval = 0;
	uint16_t window = 0;

	if (local_atou16("window", argv[0], &window, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_burst_window(interface, window);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_burst_thresh)
{
	int retval = 0;
	uint16_t threshold = 0;

	if (local_atou16("threshold", argv[0], &threshold, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_burst_thresh(interface, threshold);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_burst_pause)
{
	int retval = 0;
	uint16_t pause_time = 0;

	if (local_atou16("pause time", argv[0], &pause_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_burst_pause(interface, pause_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_burst_switch)
{
	int retval = 0;
	uint16_t switch_flag = 0;

	if (local_atou16("enable", argv[0], &switch_flag, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_burst_switch(interface, switch_flag);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_chan_pool)
{
	int retval = 0;
	static string_1024 buf = { 0 };

	retval = qcsapi_wifi_get_scs_chan_pool(interface, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_chan_pool)
{
	int retval;
	uint32_t listlen = 0;
	struct qcsapi_data_256bytes buf = { { 0 } };

	retval = local_string_to_u8_list(print, argv[0], buf.data, &listlen);
	if (retval < 0) {
		print_err(print, "Invalid channel in list\n");
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_scs_chan_pool(interface, &buf, listlen);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_vendor_fix)
{
	int retval = 0;
	int param = -1;
	int val = -1;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_vendor_fix;

	if (call_qcsapi_param_name2enum(print, tbl, argv[0], (uint32_t *)&param) < 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Supported Vendor Fix parameters:\n");
			call_qcsapi_param_name_list(print, tbl);
		}
		return -EINVAL;
	}

	if (isdigit(*argv[1])) {
		val = atoi(argv[1]);
	} else {
		print_err(print, "Invalid Vendor Fix value %s\n", argv[1]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_vendor_fix(interface, param, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_chan_mtrc_mrgn)
{
	int retval = 0;
	uint8_t val = 0;

	val = atoi(argv[0]);

	retval = qcsapi_wifi_set_scs_chan_mtrc_mrgn(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_nac_monitor_mode)
{
	int retval = 0;
	uint32_t enable;
	uint32_t on_period = MONITOR_DEFAULT_ON_PERIOD * 100 / MONITOR_DEFAULT_CYCLE_PERIOD;
	uint32_t cycle_period = MONITOR_DEFAULT_CYCLE_PERIOD;

	if (argc != 3 && argc != 1)
		return qcsapi_report_usage(cb);

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	if (argc == 3) {
		if (local_atou32_range("on period", argv[1], &on_period, print,
					MONITOR_MIN_ON_PERIOD, MONITOR_MAX_ON_PERIOD) < 0)
			return -EINVAL;
		if (local_atou32_range("cycle period", argv[2], &cycle_period, print,
					MONITOR_MIN_CYCLE_PERIOD, MONITOR_MAX_CYCLE_PERIOD) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_set_scs_nac_monitor_mode(interface, enable, on_period, cycle_period);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_band_margin_check)
{
	int retval = 0;
	uint16_t enable;

	if (local_atou16("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_band_margin_check(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_band_margin)
{
	int retval = 0;
	uint16_t margin;

	if (local_atou16("margin", argv[0], &margin, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_band_margin(interface, margin);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_dfs_reentry_request)
{
	int retval = 0;
	qcsapi_unsigned_int status = 0;

	retval = qcsapi_wifi_get_scs_dfs_reentry_request(interface, &status);

	return qcsapi_report_int_or_error(cb, retval, status);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_cca_intf)
{

	int retval;
	qcsapi_unsigned_int chan = atoi(argv[0]);
	int cca_intf = 0;

	retval = qcsapi_wifi_get_scs_cca_intf(interface, chan, &cca_intf);

	return qcsapi_report_int_or_error(cb, retval, chan);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scs_param)
{
	int retval = 0;
	int len = 0;
	qcsapi_scs_param_rpt *p_rpt = NULL;
	struct qcsapi_data_1Kbytes *buf = NULL;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_1Kbytes) >= (sizeof(*p_rpt) * SCS_PARAM_MAX));

	len = sizeof(*buf);
	buf = local_malloc(cb, len);
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_scs_params(interface, buf, len);
	if (retval < 0)
		report_qcsapi_error(cb, retval);
	else
		dump_scs_param(print, (qcsapi_scs_param_rpt *) buf);

	free(buf);

	return retval;
}

static void display_acs_param_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "    call_qcsapi set_acs_param <interface> <acs_param> <value>\n");
	print_out(print, "Parameters:\n");
	call_qcsapi_param_name_list(print, &qcsapi_param_name_acs);
}

CALL_QCSAPI(call_qcsapi_set_acs_param)
{
	int retval = 0;
	uint32_t val = 0;
	qcsapi_acs_param_type param;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_acs;

	if (strcmp(argv[0], "--help") == 0) {
		display_acs_param_usage(print);
		return 0;
	}

	if (argc != 2) {
		display_acs_param_usage(print);
		return -EINVAL;
	}

	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &param) < 0)
		return -EINVAL;

	val = atoi(argv[1]);

	retval = qcsapi_set_acs_params(interface, param, val);
	if (retval == -qcsapi_param_value_invalid)
		print_out(print, "Invalid value - use --help for valid values\n");

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_start_ocac)
{
	int retval;
	uint16_t chan = 0;

	if (!strcasecmp("auto", argv[0])) {
		chan = 0;
	} else {
		if (local_atou16("channel", argv[0], &chan, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_start_ocac(interface, chan);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_stop_ocac)
{
	int retval;

	retval = qcsapi_wifi_stop_ocac(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_ocac_status)
{
	int retval = 0;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_get_ocac_status(interface, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (val == 1)
		print_out(print, "Enabled\n");
	else if (val == 0)
		print_out(print, "Disabled\n");
	else
		print_out(print, "Unknown (%u)\n", val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_ocac_dwell_time)
{
	int retval;
	uint16_t dwell_time = 0;

	if (local_atou16("dwell time", argv[0], &dwell_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ocac_dwell_time(interface, dwell_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ocac_duration)
{
	int retval;
	uint16_t duration = 0;

	if (local_atou16("duration", argv[0], &duration, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ocac_duration(interface, duration);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ocac_cac_time)
{
	int retval;
	uint16_t cac_time = 0;

	if (local_atou16("cac type", argv[0], &cac_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ocac_cac_time(interface, cac_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ocac_report_only)
{
	int retval;
	uint16_t value = 0;

	if (local_atou16("enable", argv[0], &value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ocac_report_only(interface, value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ocac_threshold)
{
	int retval = 0;
	char *thrshld_param_name = argv[0];
	uint16_t thrshld_value;

	if (local_atou16("threshold", argv[1], &thrshld_value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ocac_thrshld(interface, thrshld_param_name, thrshld_value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_start_dfs_s_radio)
{
	int retval;
	uint16_t chan = 0;
	uint16_t is_auto = 0;
	int16_t chan_idx = -1;

	if (!strcasecmp("auto", argv[0]))
		is_auto = 1;

	chan_idx = (is_auto && argc == 2) ? 1 : (!is_auto && argc == 1) ? 0 : -1;

	if ((chan_idx >= 0) && (local_atou16_range("channel", argv[chan_idx], &chan,
					print, QCSAPI_ANY_CHANNEL, QCSAPI_MAX_CHANNEL) < 0)) {
		if (is_auto == 0)
			return qcsapi_report_usage(cb);
		if (argc == 2)
			return qcsapi_report_usage(cb);
	}

	if (argc == 2) {
		if (is_auto)
			chan |= IEEE80211_OCAC_AUTO_WITH_FIRST_DFS_CHAN;
		else
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_start_dfs_s_radio(interface, chan);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_stop_dfs_s_radio)
{
	int retval;

	retval = qcsapi_wifi_stop_dfs_s_radio(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_s_radio_status)
{
	int retval = 0;
	qcsapi_unsigned_int status = 0;

	retval = qcsapi_wifi_get_dfs_s_radio_status(interface, &status);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (status == 1)
		print_out(print, "Enabled\n");
	else if (status == 0)
		print_out(print, "Disabled\n");
	else
		print_out(print, "Unknown (%u)\n", status);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_s_radio_availability)
{
	int retval;
	qcsapi_unsigned_int available = 0;

	retval = qcsapi_wifi_get_dfs_s_radio_availability(interface, &available);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (available == 1)
		print_out(print, "Available\n");
	else
		print_out(print, "Unavailable (reason %d)\n", retval);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_dwell_time)
{
	int retval;
	uint16_t dwell_time = 0;

	if (local_atou16("dwell time", argv[0], &dwell_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_dwell_time(interface, dwell_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_duration)
{
	int retval;
	uint16_t duration = 0;

	if (local_atou16("duration", argv[0], &duration, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_duration(interface, duration);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_cac_time)
{
	int retval;
	uint16_t cac_time = 0;

	if (local_atou16("cac time", argv[0], &cac_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_cac_time(interface, cac_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_report_only)
{
	int retval;
	uint16_t value = 0;

	if (local_atou16("enable", argv[0], &value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_report_only(interface, value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_wea_duration)
{
	int retval;
	uint32_t duration = 0;

	if (local_atou32("duration", argv[0], &duration, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_wea_duration(interface, duration);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_wea_cac_time)
{
	int retval;
	uint32_t cac_time = 0;

	if (local_atou32("cac time", argv[0], &cac_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_wea_cac_time(interface, cac_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time)
{
	int retval;
	uint16_t dwell_time = 0;

	if (local_atou16("dwell time", argv[0], &dwell_time, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_wea_dwell_time(interface, dwell_time);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_dfs_s_radio_threshold)
{
	int retval = 0;
	char *thrshld_param_name = argv[0];
	uint16_t thrshld_value;

	if (local_atou16("threshold", argv[1], &thrshld_value, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_dfs_s_radio_thrshld(interface, thrshld_param_name, thrshld_value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_ap_isolate)
{
	int retval;
	int enable;

	if (local_atoi_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_ap_isolate(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_ap_isolate)
{
	int retval;
	int enable = 0;

	retval = qcsapi_wifi_get_ap_isolate(interface, &enable);

	return qcsapi_report_int_or_error(cb, retval, enable);
}

CALL_QCSAPI(call_qcsapi_get_interface_stats)
{
	int retval;
	qcsapi_interface_stats stats;

	retval = qcsapi_get_interface_stats(interface, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (sizeof(long) == 8) {
		print_out(print, "tx_bytes:\t%llu\n"
				"tx_pkts:\t%u\n"
				"tx_discard:\t%u\n"
				"tx_err:\t\t%u\n"
				"tx_unicast:\t%u\n"
				"tx_multicast:\t%u\n"
				"tx_broadcast:\t%u\n"
				"rx_bytes:\t%llu\n"
				"rx_pkts:\t%u\n"
				"rx_discard:\t%u\n"
				"rx_err:\t\t%u\n"
				"rx_unicast:\t%u\n"
				"rx_multicast:\t%u\n"
				"rx_broadcast:\t%u\n"
				"rx_unknown:\t%u\n",
				stats.tx_bytes,
				stats.tx_pkts,
				stats.tx_discard,
				stats.tx_err,
				stats.tx_unicast,
				stats.tx_multicast,
				stats.tx_broadcast,
				stats.rx_bytes,
				stats.rx_pkts,
				stats.rx_discard,
				stats.rx_err,
				stats.rx_unicast,
				stats.rx_multicast,
				stats.rx_broadcast, stats.rx_unknown);
	} else {
		print_out(print, "tx_bytes:\t%llu\n"
				"tx_pkts:\t%u\n"
				"tx_discard:\t%u\n"
				"tx_err:\t\t%u\n"
				"tx_unicast:\t%u\n"
				"tx_multicast:\t%u\n"
				"tx_broadcast:\t%u\n"
				"rx_bytes:\t%llu\n"
				"rx_pkts:\t%u\n"
				"rx_discard:\t%u\n"
				"rx_err:\t\t%u\n"
				"rx_unicast:\t%u\n"
				"rx_multicast:\t%u\n"
				"rx_broadcast:\t%u\n"
				"rx_unknown:\t%u\n",
				stats.tx_bytes,
				stats.tx_pkts,
				stats.tx_discard,
				stats.tx_err,
				stats.tx_unicast,
				stats.tx_multicast,
				stats.tx_broadcast,
				stats.rx_bytes,
				stats.rx_pkts,
				stats.rx_discard,
				stats.rx_err,
				stats.rx_unicast,
				stats.rx_multicast,
				stats.rx_broadcast, stats.rx_unknown);

	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_vap_extstats)
{
	int retval;
	qcsapi_vap_extstats stats;

	retval = qcsapi_get_vap_extstats(interface, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "ssid_RetransCount:\t%u\n"
			"ssid_FailedRetransCount:\t%u\n"
			"ssid_RetryCount:\t%u\n"
			"ssid_MultipleRetryCount:\t%u\n"
			"ssid_AggregatedPacketCount:\t%u\n"
			"ssid_ACKFailureCount:\t%u\n",
			stats.tx_retrans,
			stats.tx_fretrans,
			stats.tx_retries,
			stats.tx_mretries, stats.tx_aggpkts, stats.tx_toutpkts);

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_phy_stats)
{
	int iter;
	int retval;
	qcsapi_phy_stats stats;

	retval = qcsapi_get_phy_stats(interface, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "tstamp=\t\t%u\n"
			"assoc=\t\t%u\n"
			"channel=\t%u\n"
			"attenuation=\t%u\n"
			"cca_total=\t%u\n"
			"cca_tx=\t\t%u\n"
			"cca_rx=\t\t%u\n"
			"cca_int=\t%u\n"
			"cca_idle\t%u\n"
			"rx_pkts=\t%u\n"
			"last_rx_pkt_timestamp=\t%u\n"
			"rx_gain=\t%u\n"
			"rx_cnt_crc=\t%u\n"
			"rx_noise=\t%5.1f\n"
			"tx_pkts=\t%u\n"
			"last_tx_pkt_timestamp=\t%u\n"
			"tx_defers=\t%d\n"
			"tx_touts=\t%u\n"
			"tx_retries=\t%u\n"
			"cnt_sp_fail=\t%u\n"
			"cnt_lp_fail=\t%u\n"
			"last_rx_mcs=\t%d\n"
			"last_tx_mcs=\t%d\n",
			stats.tstamp,
			stats.assoc,
			stats.channel,
			stats.atten,
			stats.cca_total,
			stats.cca_tx,
			stats.cca_rx,
			stats.cca_int,
			stats.cca_idle,
			stats.rx_pkts,
			stats.last_rx_pkt_timestamp,
			stats.rx_gain,
			stats.rx_cnt_crc,
			stats.rx_noise,
			stats.tx_pkts,
			stats.last_tx_pkt_timestamp,
			stats.tx_defers,
			stats.tx_touts,
			stats.tx_retries,
			stats.cnt_sp_fail,
			stats.cnt_lp_fail, stats.last_rx_mcs, stats.last_tx_mcs);
	print_out(print, "last_evm=\t%5.1f\n", stats.last_evm);
	for (iter = 0; iter < QCSAPI_QDRV_NUM_RF_STREAMS; iter++) {
		print_out(print, "last_evm_%d=\t%5.1f\n", iter,
				stats.last_evm_array[iter]);
	}

	print_out(print, "last_rcpi=\t%5.1f\n", stats.last_rcpi);

	print_out(print, "last_rssi=\t%5.1f\n", stats.last_rssi);
	for (iter = 0; iter < QCSAPI_QDRV_NUM_RF_STREAMS; iter++) {
		print_out(print, "last_rssi_%d=\t%5.1f\n", iter,
				stats.last_rssi_array[iter]);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_telnet_enable)
{
	uint32_t enable;
	int retval = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_telnet_enable(enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_set_access_control)
{
	int retval = 0;
	uint32_t pp_enable;
	char wps_state[32];

	char *val = argv[0];

	if (!strcmp(val, "1"))
		pp_enable = 1;
	else if (!strcmp(val, "0"))
		pp_enable = 0;
	else
		return qcsapi_report_usage(cb);

	retval = qcsapi_wps_get_configured_state(interface, wps_state, sizeof(wps_state));
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (strncmp(wps_state, "configured", sizeof(wps_state)) != 0) {
		print_err(print,
			"Enable WPS before setting up WPS access control\n");
		return -EINVAL;
	}

	retval = qcsapi_wps_set_access_control(interface, pp_enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_get_access_control)
{
	int retval = 0;
	uint32_t pp_enable;

	retval = qcsapi_wps_get_access_control(interface, &pp_enable);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", (pp_enable ? "1" : "0"));

	return 0;
}

CALL_QCSAPI(call_qcsapi_non_wps_set_pp_enable)
{
	int retval = 0;
	uint32_t pp_enable;

	char *val = argv[0];

	if (!strcmp(val, "1"))
		pp_enable = 1;
	else if (!strcmp(val, "0"))
		pp_enable = 0;
	else
		return qcsapi_report_usage(cb);

	retval = qcsapi_non_wps_set_pp_enable(interface, pp_enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_non_wps_get_pp_enable)
{
	int retval = 0;
	uint32_t pp_enable;

	retval = qcsapi_non_wps_get_pp_enable(interface, &pp_enable);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", (pp_enable ? "1" : "0"));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wps_cancel)
{
	int retval = 0;

	retval = qcsapi_wps_cancel(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_set_pbc_in_srcm)
{
	int retval = 0;
	uint16_t enable = 0;

	if (local_atou16("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wps_set_pbc_in_srcm(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_get_pbc_in_srcm)
{
	int retval = 0;
	uint32_t val = 0;

	retval = qcsapi_wps_get_pbc_in_srcm(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wps_set_timeout)
{
	int retval;
	int val = 0;

	if (local_atoi32("timeout", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wps_set_timeout(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_on_hidden_ssid)
{
	int retval = 0;
	int enable = 0;

	if (local_atoi_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wps_on_hidden_ssid(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_on_hidden_ssid_status)
{
	int retval = 0;
	char buf[64];

	retval = qcsapi_wps_on_hidden_ssid_status(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wps_upnp_enable)
{
	int retval = 0;
	int enable = 0;

	if (local_atoi_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wps_upnp_enable(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wps_upnp_status)
{
	int retval = 0;
	char buf[16] = { 0 };

	retval = qcsapi_wps_upnp_status(interface, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_registrar_set_default_pbc_bss)
{
	int retval = 0;

	retval = qcsapi_registrar_set_default_pbc_bss(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_registrar_get_default_pbc_bss)
{
	int retval = 0;
	char buf[16] = { 0 };

	retval = qcsapi_registrar_get_default_pbc_bss(buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_radio_registrar_set_default_pbc_bss)
{
	int retval = 0;
	qcsapi_unsigned_int radio = 0;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_registrar_set_default_pbc_bss(radio, argv[1]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_radio_registrar_get_default_pbc_bss)
{
	int retval = 0;
	char buf[16] = { 0 };
	qcsapi_unsigned_int radio = 0;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_registrar_get_default_pbc_bss(radio, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_reset_all_counters)
{
	qcsapi_unsigned_int node_index = cb->generic_param.index;
	int local_remote = QCSAPI_LOCAL_NODE;
	int retval = 0;

	if (argc > 0) {
		if (local_parse_loca_remote_flag(print, argv[0], &local_remote) < 0)
			return -EINVAL;
	}

	retval = qcsapi_reset_all_counters(interface, node_index, local_remote);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_test_traffic)
{
	int retval = 0;
	uint32_t period = 0;

	if ((argc == 2) && !strcasecmp("start", argv[0])) {
		if (local_atou32_range("period", argv[1], &period, print, 10, UINT32_MAX) < 0)
			return -EINVAL;
	} else if ((argc == 1) && (!strcasecmp("stop", argv[0]))) {
		period = 0;
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_test_traffic(interface, period);

	return qcsapi_report_complete(cb, retval);
}

static void call_qcsapi_pm_print_mode(qcsapi_output *print, const int level)
{
	switch (level) {
	case QCSAPI_PM_MODE_DISABLE:
		print_out(print, "off\n");
		break;
	case QCSAPI_PM_MODE_SUSPEND:
		print_out(print, "suspend\n");
		break;
	case QCSAPI_PM_MODE_IDLE:
		print_out(print, "idle\n");
		break;
	default:
		print_out(print, "auto\n");
		break;
	}
}

static int local_pm_str_to_level(qcsapi_output *print,
		const char *const level_str, int *level)
{
	if (strcmp(level_str, "off") == 0) {
		*level = QCSAPI_PM_MODE_DISABLE;
	} else if (strcmp(level_str, "on") == 0 || strcmp(level_str, "auto") == 0) {
		*level = QCSAPI_PM_MODE_AUTO;
	} else if (strcmp(level_str, "suspend") == 0) {
		*level = QCSAPI_PM_MODE_SUSPEND;
	} else if (strcmp(level_str, "idle") == 0) {
		*level = QCSAPI_PM_MODE_IDLE;
	} else {
		print_err(print, "%s: invalid parameter '%s'\n", __func__, level_str);
		return -EINVAL;
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_pm_get_set_mode)
{
	int level;
	int retval = 0;
	int demac = 0;

	if (argc > 0) {
		if (strcmp("dual_emac", argv[argc - 1]) == 0) {
			demac = 1;
			argc--;
		}
	}

	if (!argc) {
		if (demac)
			retval = qcsapi_pm_dual_emac_get_mode(&level);
		else
			retval = qcsapi_pm_get_mode(&level);
		if (retval >= 0 && verbose_flag >= 0)
			call_qcsapi_pm_print_mode(print, level);
	} else if (argc == 1) {
		retval = local_pm_str_to_level(print, argv[0], &level);
		if (retval >= 0) {
			if (demac)
				retval = qcsapi_pm_dual_emac_set_mode(level);
			else
				retval = qcsapi_pm_set_mode(level);
		}
	} else {
		retval = -EINVAL;
	}

	if (retval < 0)
		report_qcsapi_error(cb, retval);

	return retval;
}

CALL_QCSAPI(call_qcsapi_qpm_get_level)
{
	int qpm_level;
	int retval = 0;

	retval = qcsapi_get_qpm_level(&qpm_level);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d\n", qpm_level);

	return 0;
}

CALL_QCSAPI(call_qcsapi_restore_default_config)
{
	int flag = 0;
	char *argp;
	int retval = 0;
	int offset = 0;
	int i = 0;
	unsigned int radio = 0;

	while (argc > 0) {
		offset = QCSAPI_RESTORE_FG_MODE_OFFSET;
		argp = argv[i];
		/* global flags */
		if (strcmp(argp, "1") == 0 || strcmp(argp, "ip") == 0) {
			flag |= QCSAPI_RESTORE_FG_IP;
		} else if (strcmp(argp, "noreboot") == 0) {
			flag |= QCSAPI_RESTORE_FG_NOREBOOT;
		} else if (strcmp(argp, "fdr") == 0) {
			flag |= QCSAPI_RESTORE_FG_FDR;
		} else if (strcmp(argp, "nfr") == 0) {
			flag |= QCSAPI_RESTORE_FG_NFR;
		} else if (strcmp(argp, "dcdc") == 0) {
			flag |= QCSAPI_RESTORE_FG_DCDC;
		/* global modes */
		} else if (strcmp(argp, "ap") == 0) {
			flag |= (QCSAPI_RESTORE_FG_AP << offset);
		} else if (strcmp(argp, "sta") == 0) {
			flag |= (QCSAPI_RESTORE_FG_STA << offset);
		} else if (strcmp(argp, "repeater") == 0) {
			flag |= (QCSAPI_RESTORE_FG_REPEATER << offset);
		/* per-radio flags */
		} else if (strncmp(argp, "wifi", strlen("wifi")) == 0) {
			if (argc < 2) {
				retval = -EINVAL;
				break;
			}
			retval = qcsapi_get_radio_from_ifname(argp, &radio);
			if (retval < 0)
				break;
			offset += (radio + 1) * QCSAPI_RESTORE_FG_MODE_BITS;
			if (strcmp(argv[i + 1], "ap") == 0) {
				flag |= (QCSAPI_RESTORE_FG_AP << offset);
			} else if (strcmp(argv[i + 1], "sta") == 0) {
				flag |= (QCSAPI_RESTORE_FG_STA << offset);
			} else if (strcmp(argv[i + 1], "repeater") == 0) {
				flag |= (QCSAPI_RESTORE_FG_REPEATER << offset);
			} else {
				retval = -EINVAL;
				break;
			}
			argc--;
			i++;
		} else {
			retval = -EINVAL;
			break;
		}
		argc--;
		i++;
	}
	if (retval < 0)
		return qcsapi_report_usage(cb);

	retval = qcsapi_restore_default_config(flag);
	if (retval < 0)
		report_qcsapi_error(cb, retval);

	return retval;
}

CALL_QCSAPI(call_qcsapi_run_script)
{
	int i = 0;
	char *scriptname = NULL;
	char param[QCSAPI_CMD_BUFSIZE], *param_p;
	int len = 0;
	int space = sizeof(param) - 1;
	int retval;

	param_p = param;

	if (argc == 0)
		return qcsapi_report_usage(cb);

	scriptname = argv[0];

	for (i = 1; i < argc; i++) {
		if (strlen(argv[i]) + 1 < space) {
			len = sprintf(param_p, "%s ", argv[i]);
			param_p += len;
			space -= len;
		} else {
			print_err(print, "Parameter string is too long\n");
			return -EINVAL;
		}
	}

	*param_p = '\0';
	retval = qcsapi_wifi_run_script(scriptname, param);
	if (retval < 0) {
		report_qcsapi_error(cb, retval);
		if (retval == -qcsapi_not_authorized)
			return retval;
	}
#ifdef BUILDIN_TARGET_BOARD
	{
		char strbuf[QCSAPI_MSG_BUFSIZE] = { 0 };
		FILE *log_file;
		/* output the script message */
		errno = 0;
		log_file = fopen(QCSAPI_SCRIPT_LOG, "r");
		if (log_file != NULL) {
			while (fgets(strbuf, sizeof(strbuf), log_file))
				print_out(print, "%s", strbuf);
			fclose(log_file);
		} else {
			print_err(print, "Failed to open file %s\n", QCSAPI_SCRIPT_LOG);
			return -errno;
		}
	}
#endif
	return retval;
}

CALL_QCSAPI(call_qcsapi_get_temperature)
{
	int retval;
	struct qcsapi_int_array32 temps_in;
	struct qcsapi_int_array32 temps_out;

	temps_in.val[0] = QCSAPI_TEMPSENS_INFO_TYPE_DEFAULT;

	retval = qcsapi_get_temperature_info_ext(&temps_in, &temps_out);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (temps_out.val[QCSAPI_TEMP_INDEX_RFIC_EXT] != QDRV_TEMPSENS_INVALID) {
		print_out(print, "temperature_rfic_external = %3.1f\n",
				(float)temps_out.val[QCSAPI_TEMP_INDEX_RFIC_EXT] /
				QDRV_TEMPSENS_COEFF);
	}
	if (temps_out.val[QCSAPI_TEMP_INDEX_RFIC_INT] != QDRV_TEMPSENS_INVALID) {
		print_out(print, "temperature_rfic_internal = %3.1f\n",
				(float)temps_out.val[QCSAPI_TEMP_INDEX_RFIC_INT] /
				QDRV_TEMPSENS_COEFF10);
	}
	if (temps_out.val[QCSAPI_TEMP_INDEX_BBIC_INT] != QDRV_TEMPSENS_INVALID) {
		print_out(print, "temperature_bbic_internal = %3.1f\n",
				(float)temps_out.val[QCSAPI_TEMP_INDEX_BBIC_INT] /
				QDRV_TEMPSENS_COEFF10);
	}
	if (temps_out.val[QCSAPI_TEMP_INDEX_BBIC_EXT] != QDRV_TEMPSENS_INVALID) {
		print_out(print, "temperature_bbic_external = %3.1f\n",
				(float)temps_out.val[QCSAPI_TEMP_INDEX_BBIC_EXT] /
				QDRV_TEMPSENS_COEFF10);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_set_accept_oui_filter)
{
	qcsapi_mac_addr macaddr;
	qcsapi_mac_addr oui = { 0 };
	int retval;
	int action;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	action = atoi(argv[1]);

	memcpy(oui, macaddr, 3);
	retval = qcsapi_wifi_set_accept_oui_filter(interface, oui, action);

	return qcsapi_report_complete(cb, retval);
}

#define QCSAPI_OUI_LIST_SIZE 126

CALL_QCSAPI(call_qcsapi_get_accept_oui_filter)
{
	int retval;
	char *buf = NULL;
	unsigned int size = QCSAPI_OUI_LIST_SIZE;

	if (argc > 0) {
		if (local_atou32_range("size", argv[0], &size, print, 1, QCSAPI_MSG_BUFSIZE) < 0)
			return -EINVAL;
	}

	buf = local_malloc(cb, size);
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_get_accept_oui_filter(interface, buf, size);

	qcsapi_report_str_or_error(cb, retval, buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_set_vht)
{
	qcsapi_unsigned_int vht_status = (qcsapi_unsigned_int) atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_vht(interface, vht_status);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_he)
{
	int retval = 0;
	uint32_t enable = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_he(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_he)
{
	int retval = 0;
	qcsapi_unsigned_int he_status;

	retval = qcsapi_wifi_get_he(interface, &he_status);
	if (retval >= 0) {
		print_out(print, "%d\n", he_status);
	} else {
		report_qcsapi_error(cb, retval);
		retval = 1;
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_get_swfeat_list)
{
	int retval;
	string_4096 buf;
	unsigned int radio = 0;

	if (argc > 0) {
		if (local_atou32("radio", argv[0], &radio, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_radio_get_swfeat_list(radio, buf);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", buf);

	return 0;
}

/*
 * Pass-in epoch time (UTC secs) to convert to readable date string
 */
static void local_qcsapi_timestr(char *const buf, const size_t bufsize,
					const uint32_t utc_time_secs)
{
	const time_t epoch_seconds = utc_time_secs;
	struct tm tm_parsed;

	gmtime_r(&epoch_seconds, &tm_parsed);

	strftime(buf, bufsize, "%d %B %Y %H:%M:%S", &tm_parsed);
}

static char *uboot_type_to_str(char type)
{
	char *ptr;

	switch (type - '0') {
	case UBOOT_INFO_LARGE:
		ptr = "Large";
		break;
	case UBOOT_INFO_MINI:
		ptr = "Mini";
		break;
	case UBOOT_INFO_TINY:
		ptr = "Tiny";
		break;
	default:
		ptr = "Unknown";
	}

	return ptr;
}

/*
 * Primary userspace call_qcsapi handler to get u-boot information
 */
CALL_QCSAPI(call_qcsapi_get_uboot_info)
{
	struct early_flash_config ef_config;
	string_32 version_str;
	string_32 built_str = { 0 };
	uint32_t u_boot_time;
	int retval;
	int uboot_info;
	char *file;

	file = (argc > 1) ? argv[1] : NULL;

	retval = qcsapi_get_uboot_img_info(version_str, &ef_config, file);
	if (retval) {
		print_err(print, "Call to qcsapi_get_uboot_info failed retval=%d\n",
				retval);
		return -1;
	}

	errno = 0;
	u_boot_time = strtol((char *)ef_config.built_time_utc_sec, NULL, 10);
	if (errno) {
		print_err(print, "strtol(%s) failed, errno=-%d\n",
				(char *)ef_config.built_time_utc_sec, errno);
		return -errno;
	}

	/* Convert UTC seconds to readable date string */
	local_qcsapi_timestr(built_str, sizeof(built_str) - 1, u_boot_time);

	uboot_info = atoi(argv[0]);
	switch (uboot_info) {
	case UBOOT_INFO_VER:
		print_out(print, "Version: %s\n", version_str);
		break;
	case UBOOT_INFO_BUILT:
		print_out(print, "Built: %s\n", built_str);
		break;
	default:
	case UBOOT_INFO_TYPE:
	case UBOOT_INFO_ALL:
		if (uboot_info == UBOOT_INFO_ALL) {
			print_out(print, "Version: %s\nBuilt  : %s\n", version_str, built_str);
		}
		print_out(print, "Type   : U-boot (%s)\n", uboot_type_to_str(ef_config.uboot_type));
		break;
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_vht)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_vht(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_calcmd_set_test_mode)
{
	int retval;
	int channel;
	int antenna;
	int mcs;
	int bw;
	int pkt_size;
	int eleven_n;
	int primary_chan;

	channel = atoi(argv[0]);
	antenna = atoi(argv[1]);
	mcs = atoi(argv[2]);
	bw = atoi(argv[3]);
	pkt_size = atoi(argv[4]);
	eleven_n = atoi(argv[5]);
	primary_chan = atoi(argv[6]);

	retval = qcsapi_calcmd_set_test_mode(channel, antenna, mcs, bw, pkt_size, eleven_n,
			primary_chan);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_calcmd_show_test_packet)
{
	int retval;
	uint32_t txnum;
	uint32_t rxnum;
	uint32_t crc;


	retval = qcsapi_calcmd_show_test_packet(&txnum, &rxnum, &crc);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "tx_pkts# = \t%d\nrx_pkts# = \t%d\nCRC_err# = \t%d\n",
					txnum, rxnum, crc);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_calcmd_send_test_packet)
{
	int retval;
	int packet_num;

	packet_num = atoi(argv[0]);

	retval = qcsapi_calcmd_send_test_packet(packet_num);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_stop_test_packet)
{
	int retval;

	retval = qcsapi_calcmd_stop_test_packet();
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_send_dc_cw_signal)
{
	int retval;
	qcsapi_unsigned_int channel;

	channel = atoi(argv[0]);
	retval = qcsapi_calcmd_send_dc_cw_signal(channel);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_stop_dc_cw_signal)
{
	int retval;

	retval = qcsapi_calcmd_stop_dc_cw_signal();
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_test_mode_antenna_sel)
{
	int retval;
	qcsapi_unsigned_int antenna;

	retval = qcsapi_calcmd_get_test_mode_antenna_sel(&antenna);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", antenna);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_test_mode_mcs)
{
	int retval;
	qcsapi_unsigned_int mcs;

	retval = qcsapi_calcmd_get_test_mode_mcs(&mcs);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", mcs);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_test_mode_bw)
{
	int retval;
	qcsapi_unsigned_int bw;

	retval = qcsapi_calcmd_get_test_mode_bw(&bw);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", bw);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_tx_power)
{
	int retval;
	qcsapi_calcmd_tx_power_rsp tx_power;

	retval = qcsapi_calcmd_get_tx_power(&tx_power);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d.%ddBm %d.%ddBm %d.%ddBm %d.%ddBm\n",
					tx_power.value[0] / 10, tx_power.value[0] % 10,
					tx_power.value[1] / 10, tx_power.value[1] % 10,
					tx_power.value[2] / 10, tx_power.value[2] % 10,
					tx_power.value[3] / 10, tx_power.value[3] % 10);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_set_tx_power)
{
	int retval;
	qcsapi_unsigned_int tx_power;

	if (local_atou32("Tx power", argv[0], &tx_power, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_calcmd_set_tx_power(interface, tx_power);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_calcmd_get_real_time_txpower)
{
	int retval;
	qcsapi_calcmd_tx_power_rsp tx_power;
	int i;

	retval = qcsapi_calcmd_get_real_time_txpower(interface, &tx_power);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < QCSAPI_QDRV_NUM_RF_STREAMS; i++) {
		print_out(print, " %u.%udBm",
				tx_power.value[i] / 10, tx_power.value[i] % 10);
	}
	print_out(print, "\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_test_mode_rssi)
{
	int retval;
	qcsapi_calcmd_rssi_rsp rssi;

	retval = qcsapi_calcmd_get_test_mode_rssi(&rssi);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d.%d %d.%d %d.%d %d.%d\n",
					rssi.value[0] / 10, rssi.value[0] % 10,
					rssi.value[1] / 10, rssi.value[1] % 10,
					rssi.value[2] / 10, rssi.value[2] % 10,
					rssi.value[3] / 10, rssi.value[3] % 10);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_set_mac_filter)
{
	int retval = 0;
	int sec_enable;
	int q_num;
	qcsapi_mac_addr macaddr;

	q_num = atoi(argv[0]);
	sec_enable = atoi(argv[1]);

	if (local_parse_macaddr(cb, argv[2], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_calcmd_set_mac_filter(q_num, sec_enable, macaddr);

	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(cb, retval);
		retval = 1;
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_antenna_count)
{
	int retval;
	qcsapi_unsigned_int antenna_count;

	retval = qcsapi_calcmd_get_antenna_count(&antenna_count);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", antenna_count);
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_clear_counter)
{
	int retval;

	retval = qcsapi_calcmd_clear_counter();
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		return report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_calcmd_get_info)
{
	int retval;
	string_1024 val = { 0 };

	retval = qcsapi_calcmd_get_info(val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dfs_channels_status)
{
	int retval = 0;
	qcsapi_unsigned_int dfs_channels_status;

	retval = qcsapi_wifi_get_dfs_channels_status(interface, &dfs_channels_status);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", dfs_channels_status);
		}
	} else {
		report_qcsapi_error(cb, retval);
		retval = 1;
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_disable_dfs_channels)
{
	int retval = 0;
	int new_channel = 0;

	if (argc == 2)
		new_channel = atoi(argv[1]);

	retval = qcsapi_wifi_disable_dfs_channels(interface, atoi(argv[0]), new_channel);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_soc_macaddr)
{
	qcsapi_mac_addr macaddr;
	int retval = 0;

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_set_soc_mac_addr(interface, macaddr);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_carrier_id)
{
	int retval = 0;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_get_carrier_id(&val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_set_carrier_id)
{
	int retval = 0;
	uint32_t val;
	uint32_t update_uboot = 0;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
	} else {
		print_err(print, "Invalid carrier id value %s\n", argv[0]);
		return -EINVAL;
	}

	if (argc > 1) {
		if (isdigit(*argv[1])) {
			update_uboot = atoi(argv[1]);
		} else {
			print_err(print, "Invalid uboot update flag %s\n", argv[1]);
			return -EINVAL;
		}
	}

	retval = qcsapi_set_carrier_id(val, update_uboot);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_platform_id)
{
	int retval = 0;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_get_platform_id(&val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_spinor_jedecid)
{
	unsigned int val;
	int retval;

	retval = qcsapi_wifi_get_spinor_jedecid(interface, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "0x%08x\n", val);

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_custom_value)
{
	int retval;
	char *key;
	char val[QCSAPI_CUSTOM_VALUE_MAX_LEN] = { '\0' };

	key = argv[0];
	retval = qcsapi_get_custom_value(key, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_set_custom_value)
{
	int retval;
	char *key = argv[0];
	char *val = argv[1];

	retval = qcsapi_set_custom_value(key, val);

	return qcsapi_report_complete(cb, retval);
}

static void local_print_mlme_stats(struct qcsapi_output *print, qcsapi_mlme_stats *stats)
{
	print_out(print, "auth:\t\t%u\n"
			"auth_fails:\t%u\n"
			"assoc:\t\t%u\n"
			"assoc_fails:\t%u\n"
			"deauth:\t\t%u\n"
			"diassoc:\t%u\n",
			stats->auth, stats->auth_fails, stats->assoc,
			stats->assoc_fails, stats->deauth, stats->diassoc);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mlme_stats_per_mac)
{
	int retval = 0;
	qcsapi_mac_addr macaddr = { 0 };
	qcsapi_mlme_stats stats;

	if (argc == 1) {
		if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_get_mlme_stats_per_mac(macaddr, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_print_mlme_stats(print, &stats);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_mlme_stats_per_association)
{
	int retval = 0;
	qcsapi_mlme_stats stats;
	qcsapi_unsigned_int assoc_idx = cb->generic_param.index;

	retval = qcsapi_wifi_get_mlme_stats_per_association(interface, assoc_idx, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_print_mlme_stats(print, &stats);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_mlme_stats_macs_list)
{
	int retval = 0;
	qcsapi_mlme_stats_macs mac_list;
	qcsapi_mac_addr terminator_addr;
	int i;

	memset(&terminator_addr, 0xFF, sizeof(terminator_addr));

	retval = qcsapi_wifi_get_mlme_stats_macs_list(&mac_list);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < QCSAPI_MLME_STATS_MAX_MACS; ++i) {
		if (memcmp(mac_list.addr[i], terminator_addr, sizeof(qcsapi_mac_addr)) == 0)
			break;
		local_dump_macaddr(print, mac_list.addr[i]);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_nss_cap)
{
	qcsapi_mimo_type modulation;
	int retval;
	unsigned int val;

	modulation = cb->generic_param.parameter_type.modulation;
	retval = qcsapi_wifi_get_nss_cap(interface, modulation, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_nss_cap)
{
	qcsapi_mimo_type modulation = cb->generic_param.parameter_type.modulation;
	qcsapi_unsigned_int val = (qcsapi_unsigned_int) atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_nss_cap(interface, modulation, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_security_defer_mode)
{
	int retval;
	int val;

	retval = qcsapi_wifi_get_security_defer_mode(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_security_defer_mode)
{
	int val = (qcsapi_unsigned_int) atoi(argv[0]);
	int retval;

	retval = qcsapi_wifi_set_security_defer_mode(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_apply_security_config)
{
	int retval = 0;

	retval = qcsapi_wifi_apply_security_config(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_intra_bss_isolate)
{
	int retval = 0;
	qcsapi_unsigned_int enable;

	enable = (qcsapi_unsigned_int) atoi(argv[0]);
	if (enable > 1) {
		print_err(print, "bad parameter %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_intra_bss_isolate(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_intra_bss_isolate)
{
	int retval = 0;
	qcsapi_unsigned_int enable;

	retval = qcsapi_wifi_get_intra_bss_isolate(interface, &enable);

	return qcsapi_report_uint_or_error(cb, retval, enable);
}

CALL_QCSAPI(call_qcsapi_wifi_set_bss_isolate)
{
	int retval = 0;
	qcsapi_unsigned_int enable;

	enable = (qcsapi_unsigned_int) atoi(argv[0]);
	if (enable > 1) {
		print_err(print, "bad parameter %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_bss_isolate(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_bss_isolate)
{
	int retval = 0;
	qcsapi_unsigned_int enable;

	retval = qcsapi_wifi_get_bss_isolate(interface, &enable);

	return qcsapi_report_uint_or_error(cb, retval, enable);
}

CALL_QCSAPI(call_qcsapi_set_host_state)
{
	uint16_t val;
	int retval;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
	} else {
		return -EINVAL;
	}
	retval = qcsapi_set_host_state(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wowlan_get_host_state)
{
	int retval;
	uint16_t val;
	uint32_t len = sizeof(val);

	retval = qcsapi_wifi_wowlan_get_host_state(interface, &val, &len);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wowlan_match_type_set)
{
	uint16_t val;
	int retval;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
	} else {
		return -EINVAL;
	}
	retval = qcsapi_wowlan_set_match_type(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wowlan_get_match_type)
{
	int retval = 0;
	uint16_t val;
	qcsapi_unsigned_int len = sizeof(val);

	retval = qcsapi_wifi_wowlan_get_match_type(interface, &val, &len);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wowlan_l2_type_set)
{
	uint16_t ether_type;
	int retval;

	if (isdigit(*argv[0])) {
		ether_type = atoi(argv[0]);
	} else {
		return -EINVAL;
	}
	retval = qcsapi_wowlan_set_L2_type(interface, ether_type);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wowlan_get_l2_type)
{
	int retval = 0;
	uint16_t val;
	qcsapi_unsigned_int len = sizeof(val);

	retval = qcsapi_wifi_wowlan_get_l2_type(interface, &val, &len);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wowlan_udp_port_set)
{
	uint16_t val;
	int retval;

	if (isdigit(*argv[0])) {
		val = atoi(argv[0]);
	} else {
		return -EINVAL;
	}
	retval = qcsapi_wowlan_set_udp_port(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wowlan_get_udp_port)
{
	int retval = 0;
	uint16_t val;
	qcsapi_unsigned_int len = sizeof(val);

	retval = qcsapi_wifi_wowlan_get_udp_port(interface, &val, &len);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_wlmonitor_enable)
{
	int retval;
	uint32_t enable = 1;
	qcsapi_mac_addr macaddr;

	if (!isdigit(*argv[1]))
		return -EINVAL;

	enable = atoi(argv[1]);

	if (local_parse_macaddr(cb, argv[0], macaddr) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_wlmonitor_enable(interface, macaddr, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wlmonitor_rate_thres)
{
	int retval;
	uint32_t rate_thres;
	uint8_t op_severity;

	if (!isdigit(*argv[0]) || !isdigit(*argv[1]))
		return -EINVAL;

	op_severity = atoi(argv[0]);
	rate_thres = atoi(argv[1]);

	retval = qcsapi_wifi_wlmonitor_config_threshold(interface,
			WLAN_MONITOR_LINK_RATE, op_severity, 0, rate_thres);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_wlmonitor_period_thres)
{
	int retval;
	uint32_t period_thres;
	uint8_t op_severity;
	uint8_t period_unit;

	if (!isdigit(*argv[0]) || !isdigit(*argv[1]) || !isdigit(*argv[2]))
		return -EINVAL;

	op_severity = atoi(argv[0]);
	period_unit = atoi(argv[1]);
	period_thres = atoi(argv[2]);

	retval = qcsapi_wifi_wlmonitor_config_threshold(interface,
			WLAN_MONITOR_LINK_PERIOD, op_severity, period_unit, period_thres);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_vlan_loop_detect)
{
	int retval;
	uint32_t val = 1;

	if (!isdigit(*argv[0]))
		return qcsapi_report_usage(cb);

	val = atoi(argv[0]);

	retval = qcsapi_set_vlan_loop_detect(val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_vlan_loop_detect)
{
	int retval = -1;
	uint32_t val = 0;

	retval = qcsapi_get_vlan_loop_detect(&val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_add_app_ie)
{
	qcsapi_unsigned_int frametype = (qcsapi_unsigned_int) atoi(argv[0]);
	qcsapi_unsigned_int ieindex = (qcsapi_unsigned_int) atoi(argv[1]);
	char *app_ie = argv[2];
	int retval;

	retval = qcsapi_add_app_ie(interface, frametype, ieindex, app_ie);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_remove_app_ie)
{
	qcsapi_unsigned_int frametype = (qcsapi_unsigned_int) atoi(argv[0]);
	qcsapi_unsigned_int ieindex = (qcsapi_unsigned_int) atoi(argv[1]);
	int retval;

	retval = qcsapi_remove_app_ie(interface, frametype, ieindex);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_disable_11b)
{
	int retval;
	uint32_t enable;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_disable_11b(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_is_weather_channel)
{
	int retval = 0;
	uint16_t chan;

	if (local_atou16("channel", argv[0], &chan, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_is_weather_channel(interface, chan);

	return qcsapi_report_int_or_error(cb, retval, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_nac_mon_mode)
{
	int retval = 0;
	uint16_t period = MONITOR_DEFAULT_CYCLE_PERIOD;
	uint16_t percentage_on = MONITOR_DEFAULT_ON_PERIOD * 100 / MONITOR_DEFAULT_CYCLE_PERIOD;
	uint8_t enable;

	if (!strcmp(argv[0], "enable")) {
		enable = 1;
	} else if (!strcmp(argv[0], "disable")) {
		enable = 0;
	} else {
		return qcsapi_report_usage(cb);
	}

	if (enable) {
		if (argc >= 2) {
			if (local_atou16("period", argv[1], &period, print) < 0)
				return -EINVAL;
		}
		if (argc == 3) {
			if (local_atou16_range("percentage on", argv[2], &percentage_on,
							print, 0, 0xff) < 0)
				return -EINVAL;
		}
	} else {
		if (argc > 1)
			return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_set_nac_mon_mode(interface, enable, period,
			(uint8_t) percentage_on);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_nac_mon_mode)
{
	int retval = 0;
	int enable = 0;
	int percentage_on = 0;
	int period = 0;

	retval = qcsapi_wifi_get_nac_mon_mode(interface, &enable, &period,
			&percentage_on);
	if (retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "status: %s\n Duty Cycle time: %d\n Percentage on: %d\n",
					enable ? "enabled" : "disabled", period, percentage_on);
		}
	} else {
		report_qcsapi_error(cb, retval);
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_nac_stats)
{
	int retval;
	qcsapi_nac_stats_report *report = NULL;
	int i = 0;

	report = calloc(1, sizeof(qcsapi_nac_stats_report));
	if (!report)
		return -ENOMEM;

	retval = qcsapi_wifi_get_nac_stats(interface, report);

	if (retval < 0) {
		report_qcsapi_error(cb, retval);
	} else {
		print_out(print,
			"  MAC Address      RSSI(dB)  Timestamp  Channel  Packet Type\n");
		for (i = 0; i < report->num_valid_entries; i++) {
			print_out(print, MACSTR " %9d %10llu %8d   %-10s\n",
					MAC2STR(&report->stats[i].txmac[0]),
					report->stats[i].average_rssi,
					report->stats[i].timestamp,
					report->stats[i].channel,
					(report->stats[i].packet_type == 1) ? "Control" :
					((report->stats[i].packet_type ==
						2) ? "Data" : "Management"));
		}
	}

	free(report);

	return retval;
}

#define MAX_USER_DEFINED_MAGIC	256

static void local_str_to_hex(uint8_t *pb_dest, const char *pb_src, int nlen)
{
	char h1;
	char h2;
	uint8_t s1;
	uint8_t s2;
	int i;

	for (i = 0; i < nlen; i++) {
		h1 = pb_src[2 * i];
		h2 = pb_src[2 * i + 1];
		s1 = toupper(h1) - 0x30;
		if (s1 > 9)
			s1 -= 7;
		s2 = toupper(h2) - 0x30;
		if (s2 > 9)
			s2 -= 7;
		pb_dest[i] = s1 * 16 + s2;
	}
}

static int local_get_pattern_string(const char *arg, uint8_t *pattern)
{
	int loop = 0;
	int num = 0;
	int len = strnlen(arg, MAX_USER_DEFINED_MAGIC << 1);

	while (loop < len) {
		if (isxdigit(arg[loop]) && isxdigit(arg[loop + 1])) {
			local_str_to_hex(&pattern[num], &arg[loop], 1);
			num++;
			loop += 2;
		} else {
			loop++;
		}
	}
	return num;
}

CALL_QCSAPI(call_qcsapi_wowlan_set_magic_pattern)
{
	int retval;
	uint8_t pattern[MAX_USER_DEFINED_MAGIC];
	struct qcsapi_data_256bytes pattern_data;
	uint32_t len;
	uint32_t actual_string_len;

	memset(pattern, 0, MAX_USER_DEFINED_MAGIC);
	len = strlen(argv[0]);
	if (len > (MAX_USER_DEFINED_MAGIC << 1)) {
		print_err(print, "pattern should be 256 bytes in total length\n");
		return -EINVAL;
	}

	actual_string_len = local_get_pattern_string(argv[0], pattern);
	if (actual_string_len != (len >> 1)) {
		print_err(print, "there are unrecognized chars\n");
		return -EINVAL;
	}

	memset(&pattern_data, 0, sizeof(pattern_data));
	memcpy(pattern_data.data, pattern, actual_string_len);
	retval = qcsapi_wowlan_set_magic_pattern(interface, &pattern_data, actual_string_len);

	return qcsapi_report_complete(cb, retval);
}

static void dump_magic_pattern(qcsapi_output *print, struct qcsapi_data_256bytes *pattern,
				qcsapi_unsigned_int len)
{
	int i;

	for (i = 0; i < len; i++)
		print_out(print, "%02X", pattern->data[i]);
	print_out(print, "\n");
}

CALL_QCSAPI(call_qcsapi_wifi_wowlan_get_magic_pattern)
{
	int retval = 0;
	struct qcsapi_data_256bytes pattern;
	qcsapi_unsigned_int len = sizeof(pattern);

	memset(&pattern, 0, sizeof(pattern));
	retval = qcsapi_wifi_wowlan_get_magic_pattern(interface, &pattern, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	dump_magic_pattern(print, &pattern, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_bgscan_status)
{
	int retval = 0;
	int enable = 0;

	retval = qcsapi_wifi_get_bgscan_status(interface, &enable);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "Bgscan enable: %d\n", enable);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_enable_bgscan)
{
	int retval = 0;
	int enable = 0;

	if (isdigit(*argv[0])) {
		enable = atoi(argv[0]);
	} else {
		print_err(print, "Invalid parameter value %s\n", argv[0]);
	}

	retval = qcsapi_wifi_enable_bgscan(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

static void print_eth_info(qcsapi_eth_info_type type, qcsapi_eth_info_result val,
				qcsapi_output *print)
{
	int iter;
	int mask = 0;
	const struct qcsapi_eth_info_result_s *result;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_eth_info_type_mask_table); iter++) {
		if (qcsapi_eth_info_type_mask_table[iter].type == type) {
			mask = qcsapi_eth_info_type_mask_table[iter].mask;
			break;
		}
	}

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_eth_info_result_table); iter++) {
		result = &qcsapi_eth_info_result_table[iter];
		if (!(mask & result->result_type))
			continue;
		if (val & result->result_type) {
			if (result->result_bit_set)
				print_out(print, "%s: %s\n",
					result->result_label, result->result_bit_set);
		} else {
			if (result->result_bit_unset)
				print_out(print, "%s: %s\n",
					result->result_label, result->result_bit_unset);
		}
	}
}

#define QCSAPI_TX_AMSDU_BE_OFFSET 4
#define QCSAPI_TX_AMSDU_BK_OFFSET 5
#define QCSAPI_TX_AMSDU_VI_OFFSET 6
#define QCSAPI_TX_AMSDU_VO_OFFSET 7

CALL_QCSAPI(call_qcsapi_wifi_get_tx_amsdu)
{
	int enable, retval;
	uint8_t be_en;
	uint8_t bk_en;
	uint8_t vi_en;
	uint8_t vo_en;

	retval = qcsapi_wifi_get_tx_amsdu(interface, &enable);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (enable == 1) {
		be_en = 1;
		bk_en = 1;
		vi_en = 1;
		vo_en = 1;
	} else {
		be_en = (enable >> QCSAPI_TX_AMSDU_BE_OFFSET) & 1;
		bk_en = (enable >> QCSAPI_TX_AMSDU_BK_OFFSET) & 1;
		vi_en = (enable >> QCSAPI_TX_AMSDU_VI_OFFSET) & 1;
		vo_en = (enable >> QCSAPI_TX_AMSDU_VO_OFFSET) & 1;
	}
	print_out(print, "BE %d\n", be_en);
	print_out(print, "BK %d\n", bk_en);
	print_out(print, "VI %d\n", vi_en);
	print_out(print, "VO %d\n", vo_en);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_tx_amsdu)
{
	int retval;
	uint32_t enable;
	uint32_t be_en;
	uint32_t bk_en;
	uint32_t vi_en;
	uint32_t vo_en;

	if (argc == 4) {
		if (local_atou32("AC BE", argv[0], &be_en, print) < 0)
			return -EINVAL;
		if (local_atou32("AC BK", argv[1], &bk_en, print) < 0)
			return -EINVAL;
		if (local_atou32("AC VI", argv[2], &vi_en, print) < 0)
			return -EINVAL;
		if (local_atou32("AC VO", argv[3], &vo_en, print) < 0)
			return -EINVAL;
		retval = qcsapi_wifi_set_tx_amsdu_per_ac(interface, be_en, bk_en, vi_en, vo_en);
	} else if (argc == 1) {
		if (local_atou_bool("enable", argv[0], &enable, print) < 0)
			return -EINVAL;
		retval = qcsapi_wifi_set_tx_amsdu(interface, enable);
	} else {
		return qcsapi_report_usage(cb);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_beacon_phyrate)
{
	int beacon_phyrate;
	int retval;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_phyrate;

	retval = qcsapi_wifi_get_beacon_phyrate(interface, &beacon_phyrate);

	return qcsapi_report_str_or_error(cb, retval, qcsapi_param_enum2name(tbl, beacon_phyrate));
}

CALL_QCSAPI(call_qcsapi_wifi_set_beacon_phyrate)
{
	int retval;
	qcsapi_legacy_phyrate phyrate;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_phyrate;

	if (call_qcsapi_param_name2enum(print, tbl, argv[0], &phyrate) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_beacon_phyrate(interface, phyrate);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_beacon_power_backoff)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_beacon_power_backoff(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_beacon_power_backoff)
{
	int retval;
	qcsapi_unsigned_int backoff;

	if (local_atou32_range("backoff", argv[0], &backoff, print,
				IEEE80211_BEACON_POWER_BACKOFF_MIN,
				IEEE80211_BEACON_POWER_BACKOFF_MAX) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_beacon_power_backoff(interface, backoff);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mgmt_power_backoff)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_mgmt_power_backoff(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_mgmt_power_backoff)
{
	int retval;
	qcsapi_unsigned_int val;

	if (local_atou32_range("backoff", argv[0], &val, print,
			IEEE80211_MGMT_POWER_BACKOFF_MIN,
			IEEE80211_MGMT_POWER_BACKOFF_MAX) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_mgmt_power_backoff(interface, val);

	return qcsapi_report_complete(cb, retval);
}

static int local_check_dcs_parameter(char *str_s, char *str_t, char *str_v,
					uint16_t *p, qcsapi_output *print)
{
	if ((strcmp(str_s, str_t) == 0) && local_atou16(str_t, str_v, p, print) == 0)
		return -EINVAL;

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_start_dcs_scan)
{
	int retval = 0;
	int index = 0;
	uint32_t chan_num = 0;
	qcsapi_dcs_params dcs_params;
	uint16_t interval_sec = 0;

	memset(&dcs_params, 0, sizeof(qcsapi_dcs_params));
	if (argc >= 1) {
		for (index = 0; index < (argc - 1); index++) {
			if ((strcmp(argv[index], "interval") == 0)
					&& local_atou16_range("interval", argv[index + 1],
						&dcs_params.scan_interval, print, 0, 0xFF) == 0)
				index++;
			else if ((strcmp(argv[index], "interval_sec") == 0)
					&& local_atou16_range("interval sec", argv[index + 1],
						&interval_sec, print, 0, 0xFF) == 0)
				index++;
			else if (local_check_dcs_parameter(argv[index], "duration", argv[index + 1],
							&dcs_params.scan_duration, print))
				index++;
			else if (local_check_dcs_parameter(argv[index], "dwell", argv[index + 1],
							&dcs_params.dwell_time, print))
				index++;
			else if (local_check_dcs_parameter(argv[index], "spacing", argv[index + 1],
							&dcs_params.spacing, print))
				index++;
			else if ((strcmp(argv[index], "chanlist") == 0)
					&& (local_string_to_u8_list(print, argv[index + 1],
						dcs_params.chan_list, &chan_num) == 0))
				index++;
			else
				break;
		}

		if (index < argc)
			qcsapi_report_usage(cb);
	}

	dcs_params.scan_interval = QCSAPI_DCS_INTERVAL_PACK(dcs_params.scan_interval, interval_sec);
	retval = qcsapi_wifi_start_dcs_scan(interface, &dcs_params);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_stop_dcs_scan)
{
	int retval = 0;

	retval = qcsapi_wifi_stop_dcs_scan(interface);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_dcs_scan_params)
{
	int retval;
	int index = 0;
	qcsapi_dcs_params dcs_params;

	memset(&dcs_params, 0, sizeof(qcsapi_dcs_params));

	retval = qcsapi_wifi_get_dcs_scan_params(interface, &dcs_params);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print,
	"DCS:%s interval:%u(min)%u(sec) duration:%u(msec) dwell time:%u(msec) spacing:%u(msec)\n",
			dcs_params.dcs_status ? "started" : "stopped",
			QCSAPI_DCS_INTERVAL_GET_MINUTES(dcs_params.scan_interval),
			QCSAPI_DCS_INTERVAL_GET_SECONDS(dcs_params.scan_interval),
			dcs_params.scan_duration, dcs_params.dwell_time,
			dcs_params.spacing);
	print_out(print, "channel list:");
	for (index = 0; index < ARRAY_SIZE(dcs_params.chan_list); index++) {
		if (dcs_params.chan_list[index] == 0)
			break;
		print_out(print, "%u ", dcs_params.chan_list[index]);
	}
	print_out(print, "\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_extender_params)
{
	int retval = 0;
	qcsapi_extender_type type = cb->generic_param.parameter_type.type_of_extender;
	int val = 0;

	switch (type) {
	case qcsapi_extender_role:
		if (strcasecmp(argv[0], "mbs") == 0) {
			val = IEEE80211_EXTENDER_ROLE_MBS;
		} else if (strcasecmp(argv[0], "rbs") == 0) {
			val = IEEE80211_EXTENDER_ROLE_RBS;
		} else if (strcasecmp(argv[0], "none-ap") == 0) {
			val = IEEE80211_EXTENDER_ROLE_NONE_AP;
		} else if (strcasecmp(argv[0], "none") == 0) {
			val = IEEE80211_EXTENDER_ROLE_NONE;
		} else {
			print_err(print, "invalid role [%s]\n", argv[0]);
			return -EINVAL;
		}
		break;
	case qcsapi_extender_mbs_best_rssi:
	case qcsapi_extender_rbs_best_rssi:
	case qcsapi_extender_mbs_wgt:
	case qcsapi_extender_rbs_wgt:
	case qcsapi_extender_verbose:
	case qcsapi_extender_roaming:
	case qcsapi_extender_bgscan_interval:
	case qcsapi_extender_mbs_rssi_margin:
	case qcsapi_extender_short_retry_limit:
	case qcsapi_extender_long_retry_limit:
	case qcsapi_extender_scan_mbs_intvl:
	case qcsapi_extender_scan_mbs_expiry:
	case qcsapi_extender_scan_mbs_mode:
	case qcsapi_extender_fast_cac:
		if (local_atoi32("chan", argv[0], &val, print) < 0)
			return -EINVAL;
		break;
	default:
		print_err(print, "Unsupported value %d\n", type);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_extender_params(interface, type, val);

	return qcsapi_report_complete(cb, retval);
}

static void local_print_extender_param(qcsapi_extender_type type, int val, qcsapi_output *print)
{
	char *role;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_extender_param;

	if (type == qcsapi_extender_role) {
		switch (val) {
		case IEEE80211_EXTENDER_ROLE_NONE:
			role = "NONE";
			break;
		case IEEE80211_EXTENDER_ROLE_NONE_AP:
			role = "NONE-AP";
			break;
		case IEEE80211_EXTENDER_ROLE_MBS:
			role = "MBS";
			break;
		case IEEE80211_EXTENDER_ROLE_RBS:
			role = "RBS";
			break;
		default:
			role = "unknown";
			break;
		}
		print_out(print, "%s: %s\n", qcsapi_param_enum2name(tbl, type), role);
		return;
	}
	print_out(print, "%s: %d\n", qcsapi_param_enum2name(tbl, type), val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_extender_params)
{
	int retval;
	qcsapi_extender_type type;
	int val = 0;
	uint32_t iter;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_extender_param;

	for (iter = 0; iter < tbl->size; iter++) {
		if (!qcsapi_param_idx_is_defined(tbl, iter))
			continue;
		type = qcsapi_param_idx2enum(tbl, iter);
		retval = qcsapi_wifi_get_extender_params(interface, type, &val);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
		local_print_extender_param(type, val, print);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_is_startprod_done)
{
	int retval;
	int val = 0;

	retval = qcsapi_is_startprod_done(&val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_disassoc_reason)
{
	int retval;

	qcsapi_unsigned_int disassoc_reason;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_ieee80211_reason_str) == IEEE80211_REASON_MAX);

	retval = qcsapi_wifi_get_disassoc_reason(interface, &disassoc_reason);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (disassoc_reason < ARRAY_SIZE(qcsapi_ieee80211_reason_str))
		print_out(print, "Disassoc Reason Code - %u: %s\n", disassoc_reason,
				qcsapi_ieee80211_reason_str[disassoc_reason]);
	else
		print_out(print, "Reserved Code [%d]\n", disassoc_reason);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_scan_buf_max_size)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	val = (qcsapi_unsigned_int) atoi(argv[0]);

	retval = qcsapi_wifi_set_scan_buf_max_size(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scan_buf_max_size)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_scan_buf_max_size(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scan_table_max_len)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	val = (qcsapi_unsigned_int) atoi(argv[0]);
	retval = qcsapi_wifi_set_scan_table_max_len(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_scan_table_max_len)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_scan_table_max_len(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_enable_mu)
{
	int retval = 0;
	uint32_t enable = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_enable_mu(interface, enable);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	retval = qcsapi_config_update_parameter(interface, "mu", argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_enable_mu)
{
	int retval = 0;
	qcsapi_unsigned_int enable;

	retval = qcsapi_wifi_get_enable_mu(interface, &enable);

	return qcsapi_report_uint_or_error(cb, retval, enable);
}

CALL_QCSAPI(call_qcsapi_wifi_set_mu_use_eq)
{
	int retval = 0;
	uint32_t enable;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_mu_use_eq(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_mu_use_eq)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_mu_use_eq(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

#define QCSAPI_MU_GROUPS_PRINT_BUF_SIZE		2048

CALL_QCSAPI(call_qcsapi_wifi_get_mu_groups)
{
	int retval = 0;
	char buf[QCSAPI_MU_GROUPS_PRINT_BUF_SIZE];

	retval = qcsapi_wifi_get_mu_groups(interface, &buf[0], sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_optim_stats)
{
	int retval;
	uint32_t enable;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_optim_stats(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_emac_switch)
{
	int retval;
	char buf[2048] = { 0 };

	retval = qcsapi_get_emac_switch(buf);
	buf[sizeof(buf) - 1] = '\0';

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_set_emac_switch)
{
	int retval = 0;
	uint32_t enable = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_set_emac_switch(enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_sys_time)
{
	int retval;
	uint32_t secs;

	if (local_atou32("seconds since epoch", argv[0], &secs, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_sys_time(secs);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_sys_time)
{
	int retval;
	uint32_t val;

	retval = qcsapi_wifi_get_sys_time(&val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_get_eth_info)
{
	qcsapi_eth_info_type eth_info_type = qcsapi_eth_nosuch_type;
	qcsapi_eth_info_result eth_info_result = qcsapi_eth_info_unknown;
	int retval;

	if (argc == 0) {
		for (eth_info_type = qcsapi_eth_info_start;
				eth_info_type < qcsapi_eth_info_all; eth_info_type++) {
			retval = qcsapi_get_eth_info(interface, eth_info_type);
			if (retval < 0)
				return report_qcsapi_error(cb, retval);
			eth_info_result |= (qcsapi_eth_info_result) retval;
		}
		print_eth_info(eth_info_type, eth_info_result, print);
		return 0;
	}

	if (!strcmp("link", argv[0])) {
		eth_info_type = qcsapi_eth_info_link;
	} else if (!strcmp("speed", argv[0])) {
		eth_info_type = qcsapi_eth_info_speed;
	} else if (!strcmp("duplex", argv[0])) {
		eth_info_type = qcsapi_eth_info_duplex;
	} else if (!strcmp("autoneg", argv[0])) {
		eth_info_type = qcsapi_eth_info_autoneg;
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_get_eth_info(interface, eth_info_type);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_eth_info(eth_info_type, retval, print);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_ap_interface_name)
{
	int retval;

	retval = qcsapi_wifi_set_ap_interface_name(argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_ap_interface_name)
{
	int retval = 0;
	char buf[IFNAMSIZ] = { 0 };
	qcsapi_unsigned_int radio = 0;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_get_ap_interface_name(radio, buf);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_verify_repeater_mode)
{
	int retval = 0;
	qcsapi_unsigned_int radio = 0;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_verify_repeater_mode(radio);

	return qcsapi_report_int_or_error(cb, retval, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_block_bss)
{
	int retval;
	qcsapi_unsigned_int flag;

	if (isdigit(*argv[0])) {
		flag = atoi(argv[0]);
	} else {
		print_err(print, "Invalid %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_block_bss(interface, flag);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_txba_disable)
{
	int retval = 0;
	uint32_t val;

	if (local_atou_bool("disable", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_txba_disable(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_txba_disable)
{
	int retval;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_txba_disable(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_rxba_decline)
{
	int retval = 0;
	uint32_t val;

	if (local_atou_bool("decline", argv[0], &val, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_rxba_decline(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_rxba_decline)
{
	int retval = 0;
	qcsapi_unsigned_int val;

	retval = qcsapi_wifi_get_rxba_decline(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_get_sec_chan)
{
	int chan;
	int val;
	int retval;

	if (local_atoi32_range("chan", argv[0], &chan, print, 0, QCSAPI_MAX_CHANNEL) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_get_sec_chan(interface, chan, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_sec_chan)
{
	int chan, offset, retval;

	chan = atoi(argv[0]);
	offset = atoi(argv[1]);
	if (chan > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %d\n", chan);
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_sec_chan(interface, chan, offset);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_vap_default_state)
{
	int retval;
	uint32_t radio;
	int32_t enable;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	if (local_atoi_bool("enable", argv[1], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_wifi_set_vap_default_state(radio, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_vap_default_state)
{
	int retval;
	uint32_t radio;
	int val;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_radio_wifi_get_vap_default_state(radio, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

static void dump_tx_airtime_per_node(qcsapi_output *print, uint32_t idx,
					qcsapi_node_tx_airtime *nta)
{
	print_out(print, "%-17s %8s %8s %20s\n", "MAC", "Idx", "airtime", "airtime_accumulative");
	print_out(print, MACSTR " %8d %8d %20d\n", MAC2STR(nta->addr), idx,
			nta->tx_airtime, nta->tx_airtime_accum);
}

static void dump_tx_airtime_buffer(qcsapi_output *print, const struct qcsapi_data_3Kbytes *buffer)
{
	uint32_t *p_airtime_accum;
	uint32_t *p_airtime;
	uint16_t nr_nodes;
	uint16_t *p_idx;
	uint8_t *p_mac;
	char *pos;
	uint16_t fat;
	int i;

	pos = (char *)buffer->data;

	nr_nodes = *(uint16_t *) pos;
	pos += sizeof(uint16_t);

	fat = *(uint16_t *) pos;
	pos += sizeof(uint16_t);

	print_out(print, "Free airtime: %8u\n", fat);
	print_out(print, "%-17s %8s %8s %20s\n", "MAC", "Idx", "airtime", "airtime_accumulative");

	for (i = 0; i < nr_nodes; i++) {
		p_idx = (uint16_t *) pos;
		p_mac = (uint8_t *) (pos + sizeof(*p_idx));
		p_airtime = (uint32_t *) (p_mac + sizeof(qcsapi_mac_addr));
		p_airtime_accum = p_airtime + 1;

		print_out(print, MACSTR " %8u %8u %20u\n", MAC2STR(p_mac), *p_idx,
				*p_airtime, *p_airtime_accum);

		pos += sizeof(*p_idx) + sizeof(qcsapi_mac_addr) + sizeof(*p_airtime) +
				sizeof(*p_airtime_accum);
	}
}

CALL_QCSAPI(call_qcsapi_wifi_get_tx_airtime)
{
	qcsapi_node_tx_airtime nta;
	struct qcsapi_data_3Kbytes *buf;
	int retval;
	uint32_t idx = 0;
	int for_all = 0;
	int control = 0;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_3Kbytes) >= sizeof(qcsapi_node_tx_airtime));

	if (!strcmp(argv[0], "all")) {
		for_all = 1;
	} else {
		if (local_atou32("node index", argv[0], &idx, print) < 0)
			return -EINVAL;
	}

	if (argc == 2) {
		if (!strcmp(argv[1], "start"))
			control = qcsapi_accum_airtime_start;
		else if (!strcmp(argv[1], "stop"))
			control = qcsapi_accum_airtime_stop;
		else
			return qcsapi_report_usage(cb);
	}

	buf = local_malloc(cb, sizeof(struct qcsapi_data_3Kbytes));
	if (!buf)
		return -ENOMEM;

	if (control) {
		if (for_all)
			retval = qcsapi_wifi_tx_airtime_accum_control(interface, control);
		else
			retval = qcsapi_wifi_node_tx_airtime_accum_control(interface, idx, control);
	} else {
		if (for_all)
			retval = qcsapi_wifi_get_tx_airtime(interface, buf);
		else
			retval = qcsapi_wifi_node_get_tx_airtime(interface, idx, &nta);
	}

	if (retval >= 0) {
		if (control) {
			print_out(print, "complete\n");
		} else {
			if (for_all)
				dump_tx_airtime_buffer(print, buf);
			else
				dump_tx_airtime_per_node(print, idx, &nta);
		}
	} else {
		report_qcsapi_error(cb, retval);
	}

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_set_cs_thrshld_range)
{
	int retval = 0;
	int min = 0;
	int max = 0;

	min = atoi(argv[0]);
	max = atoi(argv[1]);

	if (min > max) {
		print_out(print, "max must be greater than min\n");
		return -EINVAL;
	}

	retval = qcsapi_wifi_set_cs_thrshld_range(interface, min, max);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_cs_thrshld_range)
{
	int retval;
	int min;
	int max;

	retval = qcsapi_wifi_get_cs_thrshld_range(interface, &min, &max);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", min, max);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_cs_thrshld_inuse)
{
	int retval = 0;

	retval = qcsapi_wifi_set_cs_thrshld_inuse(interface, atoi(argv[0]));

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_cs_thrshld_inuse)
{
	int retval;
	int val;

	retval = qcsapi_wifi_get_cs_thrshld_inuse(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);

}

CALL_QCSAPI(call_qcsapi_qwe_command)
{
	int retval;
	char *command = argv[0];
	char *param1 = (argc >= 2) ? argv[1] : NULL;
	char *param2 = (argc >= 3) ? argv[2] : NULL;
	char *param3 = (argc >= 4) ? argv[3] : NULL;
	char buf[1024];

	retval = qcsapi_qwe_command(command, param1, param2, param3, buf, sizeof(buf));

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_get_core_dump2)
{
	int retval = 0;
	struct qcsapi_data_4Kbytes *buf = NULL;
	unsigned int bytes_copied;
	unsigned int bytes_written;
	unsigned int start_offset;

	buf = calloc(1, sizeof(*buf));
	if (!buf) {
		print_err(print, "Could not allocate %lu bytes of memory\n", sizeof(*buf));
		retval = -ENOMEM;
		goto out;
	}

	start_offset = 0;

	while (1) {
		retval = qcsapi_get_core_dump2(buf, sizeof(*buf), start_offset, &bytes_copied);
		if (retval < 0) {
			report_qcsapi_error(cb, retval);
			goto out;
		}
		retval = 0;

		if (!bytes_copied)
			break;

		bytes_written = write(STDOUT_FILENO, buf->data, bytes_copied);
		if ((bytes_written == -1) || (bytes_written != bytes_copied)) {
			retval = -errno;
			goto out;
		}

		start_offset += bytes_copied;
	}

out:
	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_gather_info)
{
	int retval;
	struct qcsapi_data_3Kbytes *buf = NULL;
	unsigned int offset = 0;
	unsigned int bytes_copied = 0;
	unsigned int bytes_remain = 0;
	unsigned int bytes_remain_last;
	int dump_finished;
	int pid = 0;

	retval = qcsapi_gather_info_start(&pid);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	buf = local_malloc(cb, sizeof(struct qcsapi_data_3Kbytes));
	if (!buf)
		return -ENOMEM;

	dump_finished = 0;
	bytes_remain_last = (unsigned int)(-1);	/*initialize with the max value */
	while ((dump_finished == 0) || (bytes_remain > 0)) {
		if (dump_finished == 0) {
			/* the gathering process not finished yet, so don't hurry */
			usleep(500000);
			retval = qcsapi_gather_info_check_cmd_status(pid, &dump_finished);
			if (retval < 0) {
				report_qcsapi_error(cb, retval);
				goto out;
			}
		}

		memset(buf, 0, sizeof(struct qcsapi_data_3Kbytes));
		bytes_copied = 0;
		bytes_remain = 0;
		retval = qcsapi_read_gathered_info(buf, offset, &bytes_copied,
				&bytes_remain);
		if (retval < 0) {
			report_qcsapi_error(cb, retval);
			goto out;
		}
		offset += bytes_copied;

		if (write(STDOUT_FILENO, buf, bytes_copied) == -1) {
			retval = -errno;
			print_err(print, "write buffer: %s\n", strerror(errno));
			break;
		}

		/* make sure the loop will end after dump finished */
		if (dump_finished) {
			if ((bytes_copied == 0) || (bytes_remain >= bytes_remain_last)) {
				break;
			}
			bytes_remain_last = bytes_remain;
		}
	}

out:
	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_get_sysrpt)
{
	struct qcsapi_data_3Kbytes *buf;
	char *report_type;
	unsigned int offset = 0;
	unsigned int bytes_copied;
	unsigned int bytes_remain;
	uint32_t flags = 0;
	uint32_t id = getpid();
	int retval;

	if (argc == 2) {
		if (strcmp(argv[1], "force") == 0)
			flags |= QCSAPI_GET_SYSRPT_FLAG_FORCE;
		else
			return qcsapi_report_usage(cb);
	}

	report_type = argv[0];

	buf = calloc(1, sizeof(*buf));
	if (!buf) {
		print_err(print, "memory alloc failed\n");
		return -EINVAL;
	}

	while (1) {
		retval = qcsapi_get_sysrpt(report_type, NULL, id, flags, buf,
						offset, &bytes_copied, &bytes_remain);
		if (retval < 0 || !bytes_copied)
			break;
		offset += bytes_copied;
		print_out(print, (char *)buf);
		if (!bytes_remain)
			break;
	}

	free(buf);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_client_mac_list)
{
	int retval;
	int index = (argc >= 1) ? atoi(argv[0]) : 0;
	struct qcsapi_mac_list *mlist = NULL;
	int i;
	int j;

	mlist = calloc(1, sizeof(struct qcsapi_mac_list));

	retval = qcsapi_get_client_mac_list(interface, index, mlist);
	if (retval < 0) {
		report_qcsapi_error(cb, retval);
	} else {
		if (mlist->flags & 0x2)
			print_out(print, "Node supports 4 address\n");
		if (mlist->flags & 0x1)
			print_out(print, "Results are truncated to Max[%d]\n",
					QCSAPI_MAX_MACS_IN_LIST);
		for (i = 0, j = 0; i < mlist->num_entries; i++, j += 6)
			print_out(print, "\t" MACSTR "\n", MAC2STR(&mlist->macaddr[j]));
	}

	free(mlist);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_sample_all_clients)
{
	int retval;
	uint8_t val = 0;

	retval = qcsapi_wifi_sample_all_clients(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

#define MAX_ASSOC_STA 2007

CALL_QCSAPI(call_qcsapi_wifi_get_per_assoc_data)
{
	int retval = 0;
	int num_entry;
	int offset;
	struct qcsapi_sample_assoc_data data[1];
	char ip_str[IP_ADDR_STR_LEN + 1];
	int i;
	int j;

	num_entry = atoi(argv[0]);
	if ((num_entry < 1) || (num_entry > MAX_ASSOC_STA)) {
		retval = -EINVAL;
		goto out;
	}

	offset = atoi(argv[1]);
	if (num_entry <= offset) {
		retval = -EINVAL;
		goto out;
	}

	for (i = offset; i < num_entry; i++) {
		/* We've to get the data for each STAs to WAR the RPC buffer issue*/
		retval = qcsapi_wifi_get_per_assoc_data(interface, data, i + 1, i);
		if (retval < 0)
			goto out;
		print_out(print, "Assoc ID: %u\nMacaddr: " MACSTR "\nTx: %u\nRx: %u\n"
				"Tx_rate(Max): %u\nRx_rate(Max): %u\n"
				"Mode: %s\nBw: %u\nAssoc_time: %usec\n",
				data[0].assoc_id,
				MAC2STR(data[0].mac_addr),
				data[0].tx_stream,
				data[0].rx_stream,
				data[0].achievable_tx_phy_rate,
				data[0].achievable_rx_phy_rate,
				qcsapi_wifi_modes_strings[data[0].protocol],
				data[0].bw, data[0].time_associated);
		print_out(print, "Rx_bytes: %llu\nTx_bytes: %llu\nRx_pkts: %u\n"
				"Tx_pkts: %u\nRx_errors: %u\nTx_errors: %u\n"
				"Rx_dropped %u\nTx_dropped: %u\nRx_ucast: %u\n"
				"Tx_ucast: %u\nRx_mcast: %u\nTx_mcast: %u\n"
				"Rx_bcast: %u\nTx_bcast: %u\nLink_quality: %u\n",
				data[0].rx_bytes,
				data[0].tx_bytes,
				data[0].rx_packets,
				data[0].tx_packets,
				data[0].rx_errors,
				data[0].tx_errors,
				data[0].rx_dropped,
				data[0].tx_dropped,
				data[0].rx_ucast,
				data[0].tx_ucast,
				data[0].rx_mcast,
				data[0].tx_mcast,
				data[0].rx_bcast, data[0].tx_bcast, data[0].link_quality);
		print_out(print, "tx_wifi_drop: %u %u %u %u\n",
				data[0].tx_wifi_drop[WMM_AC_BE],
				data[0].tx_wifi_drop[WMM_AC_BK],
				data[0].tx_wifi_drop[WMM_AC_VI],
				data[0].tx_wifi_drop[WMM_AC_VO]);

		print_out(print, "\nRSSI\t RCPI\t EVM\t HW_NOISE\n");
		for (j = 0; j < QCSAPI_NUM_ANT; j++) {
			if (j == (QCSAPI_NUM_ANT - 1))
				print_out(print, "\n(AVG)\t(Max)\t(Sum)\t(Avg)\n");

			print_out(print, "%4d.%d\t",
					((int)(data[0].last_rssi_dbm[j])) / 10,
					abs((int)data[0].last_rssi_dbm[j]) % 10);

			print_out(print, "%4d.%d\t",
					((int)(data[0].last_rcpi_dbm[j])) / 10,
					abs((int)data[0].last_rcpi_dbm[j]) % 10);
			print_out(print, "%4d.%d\t",
					((int)(data[0].last_evm_dbm[j])) / 10,
					abs(((int)data[0].last_evm_dbm[j])) % 10);

			print_out(print, "%4d.%d\n",
					((int)(data[0].last_evm_dbm[j])) / 10,
					abs(((int)data[0].last_evm_dbm[j])) % 10);
		}

		switch (data[0].vendor) {
		case PEER_VENDOR_QTN:
			print_out(print, "vendor: quantenna\n");
			break;
		case PEER_VENDOR_BRCM:
			print_out(print, "vendor: broadcom\n");
			break;
		case PEER_VENDOR_ATH:
			print_out(print, "vendor: atheros\n");
			break;
		case PEER_VENDOR_RLNK:
			print_out(print, "vendor: ralink\n");
			break;
		case PEER_VENDOR_RTK:
			print_out(print, "vendor: realtek\n");
			break;
		case PEER_VENDOR_INTEL:
			print_out(print, "vendor: intel\n");
			break;
		default:
			print_out(print, "vendor: unknown\n");
		}

		inet_ntop(AF_INET, &data[0].ip_addr, ip_str, IP_ADDR_STR_LEN);
		print_out(print, "Ipaddr: %s\n\n", ip_str);
		memset(data, 0, sizeof(struct qcsapi_sample_assoc_data));
	}

out:
	if (retval < 0)
		report_qcsapi_error(cb, retval);

	return retval;
}

CALL_QCSAPI(call_qcsapi_get_wifi_ready)
{
	int retval;
	qcsapi_unsigned_int val = 0;

	retval = qcsapi_wifi_is_iface_ready(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_get_cca_stats)
{
	int retval;
	qcsapi_cca_stats stats;

	retval = qcsapi_get_cca_stats(interface, &stats);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "cca_occupy=\t%u\n"
			"cca_intf=\t%u\n"
			"cca_trfc=\t%u\n"
			"cca_tx=\t\t%u\n"
			"cca_rx=\t\t%u\n",
		stats.cca_occupy, stats.cca_intf, stats.cca_trfc,
		stats.cca_tx, stats.cca_rx);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_vapdebug)
{
	int retval = 0;
	int bitmap = 0;

	if (sscanf(argv[0], "%i", &bitmap) != 1)
		return qcsapi_report_usage(cb);

	retval = qcsapi_wifi_set_vapdebug(interface, bitmap);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_vapdebug)
{
	int retval = 0;
	int bitmap = 0;

	retval = qcsapi_wifi_get_vapdebug(interface, &bitmap);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%#x\n", bitmap);

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_igmp_snooping_state)
{
	int retval = 0;
	uint32_t enable = 0;

	retval = qcsapi_get_igmp_snooping_state(interface, &enable);

	return qcsapi_report_uint_or_error(cb, retval, enable);
}

CALL_QCSAPI(call_qcsapi_set_igmp_snooping_state)
{
	int retval = 0;
	uint32_t enable = 0;

	if (local_atou_bool("enable", argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_set_igmp_snooping_state(interface, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_br_get_groups)
{
	int retval = 0;
	string_2048 *buf = NULL;

	buf = calloc(sizeof(char), sizeof(string_2048));
	if (buf == NULL) {
		print_err(print, "alloc memory failed\n");
		return -EINVAL;
	}

	retval = qcsapi_br_get_groups(*buf);

	qcsapi_report_str_or_error(cb, retval, *buf);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_br_get_interfaces)
{
	int retval = 0;
	string_2048 *buf = NULL;

	buf = calloc(sizeof(char), sizeof(string_2048));
	if (buf == NULL) {
		print_err(print, "alloc memory failed\n");
		return -EINVAL;
	}

	retval = qcsapi_br_get_interfaces(interface, *buf);

	qcsapi_report_str_or_error(cb, retval, *buf);

	free(buf);

	return retval;
}

static int local_name_to_bsa_param_enum(char *lookup_name, qcsapi_bsa_conf_param *p_bsa_param)
{
	unsigned int iter;

	for (iter = 0; bsa_config[iter].bsa_name != NULL; iter++) {
		if (strcasecmp(bsa_config[iter].bsa_name, lookup_name) == 0) {
			*p_bsa_param = bsa_config[iter].bsa_enum;
			return -EINVAL;
		}
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_bsa_get_parameter)
{
	int retval = 0;
	char store_bsa_param[QCSAPI_MSG_BUFSIZE] = { 0 };
	qcsapi_bsa_conf_param bsa_param = -1;

	if (strcmp(argv[0], "--help") == 0)
		return qcsapi_report_usage(cb);

	if (local_name_to_bsa_param_enum(argv[0], &bsa_param) == 0
					|| bsa_param == qcsapi_bsa_reload_new_config) {
		print_err(print, "Invalid parameter %s\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_bsa_get_parameter(bsa_param, store_bsa_param);
	if (retval != E_PARAMETER_FOUND) {
		if (retval == E_PARAMETER_NOT_FOUND)
			retval = -qcsapi_parameter_not_found;
		if (retval == E_PARAMETER_INVALID)
			retval = -qcsapi_param_value_invalid;
		report_qcsapi_error(cb, retval);
		return retval;
	}

	print_out(print, "%s", store_bsa_param);

	return 0;
}

static void display_bsa_param_usage(qcsapi_output *print)
{
	print_out(print, "NOTE: The value ranges below are experimental and subject to change\n");
	print_out(print, "Usage:\n");
	print_out(print,
		"    call_qcsapi set_bsa_param_ext <mobility_domain> <bsa_param> <value>\n");
	print_out(print, "    call_qcsapi get_bsa_param_ext <mobility_domain> <bsa_param>\n");
	print_out(print, "\n");
	print_out(print, "Parameter\t\t\tValue\t\t\t# Notes\n");
	print_out(print,
		"    GLOBAL Parameters---------------------------------- "
			"# global parameters not limited to mobility domain\n"
		"    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* enable */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* house_keeping_interval */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* stale_entry_interval */
		"    %-27s {%d - %d}\t\t# house_keeping_interval\n" /* clear_dual_band_interval */
		"    %-27s <mac addr> {0 | 1}\t# 0 (delete) or 1 (add)\n" /* whitelist_sta_entry */
		/* deauth_for_steer_btm_capa_sta */
		"    %-27s {0 | 1}\t\t# 0 (no deauth) or 1 (deauth)\n"
		/* use_fix_bw_for_5g_phyrate_estimation */
		"    %-27s {0 | 1}\t# 0 (support bw) or 1 (20M bw)\n"
		"    %-27s {%d | %d}# dBm\n"  /* rssi_threshold_for_5g_phyrate_estimation */
		"    %-27s {0 - %d}\t\t\t#\n"	/* bsa_debug_level */
		"    %-27s {0 - 1}\t\t\t#\n"	/* bsa_debug_qevt */
		"    Per Mobile domain Parameters----------------------- "
			"# ssid parameters for each mobility domain\n"
		"    %-27s {0 - 1}\t\t\t#\n"	/* ssid_pair_status */
		/* bs_ssid */
		"    %-27s <ssid> {str | hexstr}\t# max 32 | 64 chars\n"
		"    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* allow_5g_11ac_only */
		"    %-27s {%d - %d}\t\t\t#\n"	/* non_steerable_count */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* blacklist_timeout_val */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* inactive_assoc_interval */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* inactive_runtime_interval */
		"    %-27s {%d - %d}\t\t# packets/second\n"	/* client_inactivity_threshold */
		"    %-27s {%d - %d}\t\t# seconds\n"	/* runtime_decision_interval */
		"    %-27s {%d - %d}\t\t#\n"	/* no_steering_interval */
		"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_loaded_fat */
		"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_unloaded_fat */
		"    %-27s {%d - %d}\t\t#\n"	/* run_5g_loaded_fat */
		"    %-27s {%d - %d}\t\t#\n"	/* run_5g_unloaded_fat */
		"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_loaded_min_phyrate */
		"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_min_phyrate */
		"    %-27s {%d - %d}\t\t#\n"	/* run_5g_loaded_min_phyrate */
		"    %-27s {%d - %d}\t\t#\n"	/* run_5g_unloaded_min_phyrate */
		"    %-27s {%d - %d}\t\t# dBm\n"	/* assoc_5g_loaded_min_rssi */
		"    %-27s {%d - %d}\t\t# dBm\n"	/* assoc_5g_min_rssi */
		"    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* ssid_check */
		/* bsa_reload_new_config */
		"    %-27s\t\t\t\t# Trigger BSA App to reload new config\n"
		"    %-27s\t\t\t\t# restart BSA module\n"	/* bsa_module_restart */
		"\n"
		"Note: FAT (Free Air Time) values are in tenths of a percent. E.g. 450 is 45%%\n",
		bsa_config[qcsapi_enable].bsa_name,
		bsa_config[qcsapi_house_keeping_interval].bsa_name,
			BSA_HOUSE_KEEP_MIN, BSA_HOUSE_KEEP_MAX,
		bsa_config[qcsapi_stale_entry_interval].bsa_name,
			BSA_STALE_ENTRY_INTERVAL_MIN, BSA_STALE_ENTRY_INTERVAL_MAX,
		bsa_config[qcsapi_clear_dual_band_interval].bsa_name,
			BSA_CLEAR_DUAL_BAND_MIN, BSA_CLEAR_DUAL_BAND_MAX,
		bsa_config[qcsapi_whitelist_sta_entry].bsa_name,
		bsa_config[qcsapi_deauth_for_steer_btm_capa_sta].bsa_name,
		bsa_config[qcsapi_use_fix_bw_for_5g_phyrate_estimation].bsa_name,
		bsa_config[qcsapi_rssi_threshold_for_5g_phyrate_estimation].bsa_name,
			BSA_RSSI_TH_FOR_5G_PHYRATE_ESTIMATION_MIN,
			BSA_RSSI_TH_FOR_5G_PHYRATE_ESTIMATION_MAX,
		bsa_config[qcsapi_bsa_debug_level].bsa_name, BSA_DEBUG_LEVEL_MAX,
		bsa_config[qcsapi_bsa_debug_qevt].bsa_name,
		bsa_config[qcsapi_ssid_pair_status].bsa_name,
		bsa_config[qcsapi_bs_ssid].bsa_name,
		bsa_config[qcsapi_allow_5g_11ac_only].bsa_name,
		bsa_config[qcsapi_non_steerable_count].bsa_name,
			BSA_NON_STEERABLE_CNT_MIN, BSA_NON_STEERABLE_CNT_MAX,
		bsa_config[qcsapi_blacklist_timeout_val].bsa_name,
			BSA_BLACKLIST_TIMEOUT_MIN, BSA_BLACKLIST_TIMEOUT_MAX,
		bsa_config[qcsapi_inactive_assoc_interval].bsa_name,
			BSA_INACTV_ASSOC_INTERVAL_MIN, BSA_INACTV_ASSOC_INTERVAL_MAX,
		bsa_config[qcsapi_inactive_runtime_interval].bsa_name,
			BSA_INACTV_RUNTIME_INTERVAL_MIN, BSA_INACTV_RUNTIME_INTERVAL_MAX,
		bsa_config[qcsapi_client_inactivity_threshold].bsa_name,
			BSA_CLI_INACTV_THRSHLD_MIN, BSA_CLI_INACTV_THRSHLD_MAX,
		bsa_config[qcsapi_runtime_decision_interval].bsa_name,
			BSA_RUNTIME_DECISION_INTV_MIN, BSA_RUNTIME_DECISION_INTV_MAX,
		bsa_config[qcsapi_no_steering_interval].bsa_name,
			BSA_NO_STEERING_INTV_MIN, BSA_NO_STEERING_INTV_MAX,
		bsa_config[qcsapi_assoc_5g_loaded_fat].bsa_name,
			BSA_ASSOC_5G_LOAD_FAT_MIN, BSA_ASSOC_5G_LOAD_FAT_MAX,
		bsa_config[qcsapi_assoc_5g_unloaded_fat].bsa_name,
			BSA_ASSOC_5G_UNLOAD_FAT_MIN, BSA_ASSOC_5G_UNLOAD_FAT_MAX,
		bsa_config[qcsapi_run_5g_loaded_fat].bsa_name,
			BSA_RUN_5G_LOAD_FAT_MIN, BSA_RUN_5G_LOAD_FAT_MAX,
		bsa_config[qcsapi_run_5g_unloaded_fat].bsa_name,
			BSA_RUN_5G_UNLOAD_FAT_MIN, BSA_RUN_5G_UNLOAD_FAT_MAX,
		bsa_config[qcsapi_assoc_5g_loaded_min_phyrate].bsa_name,
			BSA_ASSOC_5G_LOAD_PHYRATE_MIN, BSA_ASSOC_5G_LOAD_PHYRATE_MAX,
		bsa_config[qcsapi_assoc_5g_min_phyrate].bsa_name,
			BSA_ASSOC_5G_PHYRATE_MIN, BSA_ASSOC_5G_PHYRATE_MAX,
		bsa_config[qcsapi_run_5g_loaded_min_phyrate].bsa_name,
			BSA_RUN_5G_LOAD_PHYRATE_MIN, BSA_RUN_5G_LOAD_PHYRATE_MAX,
		bsa_config[qcsapi_run_5g_unloaded_min_phyrate].bsa_name,
			BSA_RUN_5G_UNLOAD_PHYRATE_MIN, BSA_RUN_5G_UNLOAD_PHYRATE_MAX,
		bsa_config[qcsapi_assoc_5g_loaded_min_rssi].bsa_name,
			BSA_ASSOC_5G_LOAD_RSSI_MIN, BSA_ASSOC_5G_LOAD_RSSI_MAX,
		bsa_config[qcsapi_assoc_5g_min_rssi].bsa_name,
			BSA_ASSOC_5G_RSSI_MIN, BSA_ASSOC_5G_RSSI_MAX,
		bsa_config[qcsapi_ssid_check].bsa_name,
		bsa_config[qcsapi_bsa_reload_new_config].bsa_name,
		bsa_config[qcsapi_bsa_module_restart].bsa_name);
}

CALL_QCSAPI(call_qcsapi_bsa_set_parameter)
{
	int retval = 0;
	const char *param_value1 = NULL, *param_value2 = NULL;
	qcsapi_bsa_conf_param bsa_param = -1;

	if (strcmp(argv[0], "--help") == 0) {
		display_bsa_param_usage(print);
		return 0;
	}

	if (argc == 1) {
		if (strcmp(argv[0], bsa_config[qcsapi_bsa_reload_new_config].bsa_name) == 0) {
			retval = qcsapi_bsa_set_parameter(qcsapi_bsa_reload_new_config, NULL, NULL);
			return qcsapi_report_complete(cb, retval);
		}
		return qcsapi_report_usage(cb);
	}

	if (local_name_to_bsa_param_enum(argv[0], &bsa_param) == 0) {
		print_err(print, "Invalid parameter %s\n", argv[0]);
		return -EINVAL;
	}

	param_value1 = argv[1];
	if (argc > 2)
		param_value2 = argv[2];

	retval = qcsapi_bsa_set_param(bsa_param, param_value1, param_value2);
	if (retval == -qcsapi_config_update_failed) {
		print_err(print, "BSA app is active - operation not allowed\n");
		return retval;
	}
	if (retval == -qcsapi_param_value_invalid) {
		print_err(print, "Invalid value - use --help for valid values\n");
		return retval;
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_bsa_get_parameter_ext)
{
	int retval = 0;
	qcsapi_bsa_conf_param bsa_param = -1;
	uint32_t bsa_mb_domain;

	if (strcmp(argv[0], "--help") == 0) {
		display_bsa_param_usage(print);
		return 0;
	}

	if (argc != 2)
		return qcsapi_report_usage(cb);

	if (local_atou32_range("mobility domain", argv[0], &bsa_mb_domain, print,
				0, BSA_PAIRS_NUM_MAX) < 0)
		return -EINVAL;

	if (local_name_to_bsa_param_enum(argv[1], &bsa_param) == 0) {
		print_err(print, "Invalid BSA parameter %s\n", argv[1]);
		return -EINVAL;
	}

	if ((!bsa_mb_domain && bsa_param <= qcsapi_bsa_global_param_end) ||
			(bsa_mb_domain && bsa_param <= qcsapi_mb_domain_parameter_end
					&& bsa_param > qcsapi_bsa_global_param_end)) {
		char store_bsa_param[QCSAPI_MSG_BUFSIZE] = { 0 };
		char *p_store = &store_bsa_param[0];

		retval = qcsapi_bsa_get_parameter_ext(bsa_mb_domain, bsa_param, p_store);
		if (retval == E_PARAMETER_FOUND) {
			print_out(print, "%s", p_store);
		} else {
			if (retval == E_PARAMETER_NOT_FOUND)
				retval = -qcsapi_parameter_not_found;
			if (retval == E_PARAMETER_INVALID)
				retval = -qcsapi_param_value_invalid;
			return report_qcsapi_error(cb, retval);
		}
	} else {
		print_out(print, "Invalid BSA parameter %s\n", argv[1]);
		return -EINVAL;
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_bsa_set_parameter_ext)
{
	int retval = 0;
	char *param_value1 = NULL;
	char *param_value2 = NULL;
	uint32_t bsa_mb_domain = 0;
	int cmd_err = 1;
	qcsapi_ssid_fmt fmt = qcsapi_ssid_fmt_str;
	qcsapi_bsa_conf_param bsa_param;
	const struct qcsapi_param_name_tbl *tbl = &qcsapi_param_name_ssid_fmt;

	if (strcmp(argv[0], "--help") == 0) {
		display_bsa_param_usage(print);
		return 0;
	}

	if (argc < 2)
		return qcsapi_report_usage(cb);

	if (local_atou32_range("mobility domain", argv[0], &bsa_mb_domain, print,
				0, BSA_MB_DOMAIN_ALL) < 0)
		return -EINVAL;

	if (local_name_to_bsa_param_enum(argv[1], &bsa_param) == 0) {
		print_err(print, "Invalid BSA parameter %s\n", argv[1]);
		return -EINVAL;
	}

	if (argc > 2)
		param_value1 = argv[2];

	if (argc > 3)
		param_value2 = argv[3];

	if (!bsa_mb_domain) {
		if ((bsa_param <= qcsapi_bsa_global_param_end) && (argc >= 3)) {
			cmd_err = 0;
		} else if ((bsa_param <= qcsapi_bsa_action_parameter_end)
				&& (bsa_param > qcsapi_mb_domain_parameter_end) && (argc == 2)) {
			cmd_err = 0;
		}
	} else if ((bsa_mb_domain <= BSA_PAIRS_NUM_MAX) || (bsa_mb_domain == BSA_MB_DOMAIN_ALL)) {
		if ((bsa_param > qcsapi_bsa_global_param_end)
				&& (bsa_param <= qcsapi_mb_domain_parameter_end) && (argc >= 3)) {
			cmd_err = 0;
		}
	}

	if (argc > 3 && bsa_param == qcsapi_bs_ssid && cmd_err == 0) {
		if (call_qcsapi_param_name2enum(print, tbl, argv[3], &fmt) == 0) {
			sprintf(param_value2, "%d", fmt);
		} else {
			cmd_err = 1;
		}
	}

	if (cmd_err) {
		print_out(print, "Invalid BSA param value\n");
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_bsa_set_param_ext(bsa_mb_domain, bsa_param, param_value1,
			param_value2);
	if (retval == -qcsapi_config_update_failed)
		print_out(print, "BSA App is active - operation not allowed\n");
	else if (retval == -qcsapi_param_value_invalid)
		print_err(print, "Invalid value - use --help for valid values\n");

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_radio_pwr_save)
{
	int retval = 0;
	uint32_t radio;
	uint32_t force;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	if (local_atou_bool("force", argv[1], &force, print) < 0)
		return -EINVAL;

	retval = qcsapi_set_radio_pwr_save(radio, force);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_put_radio_under_reset)
{
	int retval = 0;
	uint32_t radio;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	retval = qcsapi_put_radio_under_reset(radio);

	return qcsapi_report_complete(cb, retval);
}

#define BSA_INVALID_BAND		2
static int print_sta_cnt;

static void dump_bsa_sta_table(qcsapi_output *print, const struct qcsapi_data_3Kbytes *buffer,
				int sta_cnt, int is_assoc)
{
	int i;
	char *pos;
	struct bsa_sta_entry *psta;
	char int2str_buf[20][16];

	pos = (char *)buffer->data;

	for (i = 0; i < sta_cnt; i++) {
		psta = (struct bsa_sta_entry *)pos;
		pos += sizeof(struct bsa_sta_entry);

		if (is_assoc && psta->curr_band == BSA_INVALID_BAND)
			continue;

		if ((print_sta_cnt % 20) == 0) {
			print_out(print, "\n"
					"MAC address       Assoc 2G+5G    Not    |"
					"   LastSteeringAttempt   |"
					" SteerStats |"
					"         5G_CapabilitiesAndAssocStats         |"
					"   2G_CapabilitiesAndAssocStats\n");
			print_out(print, "                              Steerable |"
					" Target  Mode  time  phy |"
					"  OK   Fail |"
					" RSSI    CAP VHT 11v MU max_phy phy_RX phy_TX |"
					" RSSI    CAP max_phy phy_RX phy_TX\n");
		}
#define OP_YORN(con)			((con) ? "Y" : "N")
#define OP_INT2STR(str, val, format)	(val ? ((sprintf(str, format, val) > 0) ? str : "-") : "-")
#define OP_BAND_STR(band)		((band == 0) ? "2G" : ((band == 1) ? "5G" : "-"))
#define OP_STEER_MODE_STR(mode)		((mode == 1) ? "assoc" : ((mode == 2) ? "runtime" : "-"))

		print_out(print, MACSTR " %5s %5s %9s |"
				" %3s %7s %6s %4s |"
				" %4d %4d  |"
				" %4s %6s %3s %3s %2s %7s %6s %6s |"
				" %4s %6s %7s %6s %6s\n",
				MAC2STR(psta->sta_mac), OP_BAND_STR(psta->curr_band),
				OP_YORN(psta->dual_band), OP_YORN(psta->not_steerable),
				OP_BAND_STR(psta->target_band),
				OP_STEER_MODE_STR(psta->steering_mode), OP_INT2STR(int2str_buf[0],
						(uint32_t) psta->ts_steering_decision, "%u"),
				OP_INT2STR(int2str_buf[1], psta->steer_phyrate, "%u"),
				psta->steer_counter, psta->failed_steer_total,
				OP_INT2STR(int2str_buf[2], psta->info_5g.last_rssi, "%d"),
				OP_INT2STR(int2str_buf[3], psta->info_5g.sta_capab, "%#x"),
				OP_YORN(psta->info_5g.phy_mode_supported),
				OP_YORN(psta->info_5g.btm_11v_support),
				OP_YORN(psta->info_5g.mumimo_support), OP_INT2STR(int2str_buf[4],
						psta->info_5g.supported_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[5], psta->info_5g.avg_tx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[6], psta->info_5g.avg_rx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[7], psta->info_2g.last_rssi, "%d"),
				OP_INT2STR(int2str_buf[8], psta->info_2g.sta_capab, "%#x"),
				OP_INT2STR(int2str_buf[9], psta->info_2g.supported_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[10], psta->info_2g.avg_tx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[11], psta->info_2g.avg_rx_phy_rate, "%u"));
		print_sta_cnt++;
	}
}

CALL_QCSAPI(call_qcsapi_bsa_get_sta_table)
{
	int retval = 0;
	struct qcsapi_data_3Kbytes *buf = NULL;
	unsigned int sta_cnt = 0;
	unsigned int total_sta_cnt = 0;
	unsigned int remain_sta_cnt = 0;
	unsigned short session_id = 0;
	int sta_assoc = 1;

	if (argc == 1) {
		if (strcasecmp(argv[0], "all") == 0)
			sta_assoc = 0;
		else if (strcasecmp(argv[0], "assoc") == 0)
			sta_assoc = 1;
		else
			return qcsapi_report_usage(cb);
	}

	buf = local_malloc(cb, sizeof(struct qcsapi_data_3Kbytes));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_bsa_start_dump_sta_table(&session_id, &total_sta_cnt);
	if (retval >= 0) {
		if (total_sta_cnt == 0) {
			print_out(print, "BSA station table is empty\n");
		} else {
			print_sta_cnt = 0;
			while (remain_sta_cnt < total_sta_cnt) {
				retval = qcsapi_bsa_get_sta_table_item(session_id, buf,
						&sta_cnt);
				if (retval < 0 || sta_cnt == 0)
					break;
				dump_bsa_sta_table(print, buf, sta_cnt, sta_assoc);
				remain_sta_cnt += sta_cnt;
			}
			if (!print_sta_cnt)
				print_out(print, "BSA %s client table is empty\n",
						sta_assoc ? "assoc" : "");
		}
		retval = qcsapi_bsa_stop_dump_sta_table(session_id);
	}

	if (retval < 0)
		report_qcsapi_error(cb, retval);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_qdrv_set_hw_module_state)
{
	qcsapi_unsigned_int enable = 0;
	int retval = 0;
	qcsapi_hw_module hw_module = cb->generic_param.parameter_type.hw_module;

	if (local_atoi_bool_legacy(argv[0], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_qdrv_set_hw_module_state(hw_module, enable);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_qdrv_get_hw_module_state)
{
	qcsapi_unsigned_int enable = 0;
	int retval = 0;
	qcsapi_hw_module hw_module = cb->generic_param.parameter_type.hw_module;

	retval = qcsapi_qdrv_get_hw_module_state(hw_module, &enable);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (enable == 0)
		print_out(print, "FALSE\n");
	else
		print_out(print, "TRUE\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_ieee80211r)
{
	int retval = 0;

	retval = qcsapi_wifi_set_ieee80211r_str(interface, argv[0]);

	if (retval < 0) {
		report_qcsapi_error(cb, retval);
		if (retval == -qcsapi_option_not_supported)
			print_err(print,
				"802.11r only supported with WPA2-PSK and WPA2-EAP modes\n");
		return retval;
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_ieee80211r)
{
	string_16 val = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_ieee80211r(interface, val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", ((atoi(val) == 1) ? "enabled" : "disabled"));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_set_11r_mobility_domain)
{
	int retval = 0;

	retval = qcsapi_wifi_set_ieee80211r_mobility_domain_str(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_mobility_domain)
{
	string_16 val = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_ieee80211r_mobility_domain(interface, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

static int qcsapi_report_complete_check_11r_retval(const call_bundle *cb, int retval)
{
	qcsapi_output *print = cb->output;

	if (retval == -qcsapi_option_not_supported)
		print_out(print, "Configuration only supported when 802.11r is enabled\n");

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_11r_nas_id)
{
	int retval = 0;

	retval = qcsapi_wifi_set_ieee80211r_nas_id(interface, argv[0]);

	return qcsapi_report_complete_check_11r_retval(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_nas_id)
{
	string_64 val = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_ieee80211r_nas_id_64(interface, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_11r_ft_over_ds)
{
	int retval = 0;

	retval = qcsapi_wifi_set_ieee80211r_ft_over_ds_str(interface, argv[0]);

	return qcsapi_report_complete_check_11r_retval(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_ft_over_ds)
{
	string_16 val = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_ieee80211r_ft_over_ds(interface, val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", ((atoi(val) == 1) ? "enabled" : "disabled"));

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_add_11r_neighbour)
{
	int retval = 0;

	retval = qcsapi_wifi_add_11r_neighbour_str(interface, argv[0], argv[1], argv[2], argv[3]);

	return qcsapi_report_complete_check_11r_retval(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_del_11r_neighbour)
{
	int retval = 0;

	retval = qcsapi_wifi_del_11r_neighbour_str(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_neighbour)
{

	int retval = 0;
	string_4096 buf = { 0 };
	int buflen = sizeof(string_4096);

	retval = qcsapi_wifi_get_11r_neighbour(interface, buf, buflen);

	return qcsapi_report_str_or_error(cb, retval, buf);
}

CALL_QCSAPI(call_qcsapi_wifi_set_11r_r1_key_holder)
{
	int retval;

	retval = qcsapi_wifi_set_11r_r1_key_holder_str(interface, argv[0]);

	return qcsapi_report_complete_check_11r_retval(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_r1_key_holder)
{
	int retval = 0;
	string_16 val = { 0 };

	retval = qcsapi_wifi_get_11r_r1_key_holder(interface, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_11r_r0_key_lifetime)
{
	int retval = 0;

	retval = qcsapi_wifi_set_11r_r0_key_lifetime(interface, argv[0]);

	return qcsapi_report_complete_check_11r_retval(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_11r_r0_key_lifetime)
{
	string_128 val = { 0 };
	int retval = 0;

	retval = qcsapi_wifi_get_11r_r0_key_lifetime(interface, val);

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn)
{

	int retval = 0;
	uint32_t val = 0;

	if (local_atou32_range("margin", argv[0], &val, print,
				0, IEEE80211_SCS_CHAN_MTRC_MRGN_MAX) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_set_max_boot_cac_duration)
{
	int retval;
	int duration;

	if (local_atoi32_range("duration", argv[0], &duration, print, -1,
						MAX_BOOT_CAC_DURATION) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_max_boot_cac_duration(interface, duration);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_icac_status)
{
	int retval;
	int status = 0;

	retval = qcsapi_wifi_get_icac_status(interface, &status);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%s\n", status ? "Active" : "Inactive");

	return 0;
}

CALL_QCSAPI(call_qcsapi_get_reboot_cause)
{
	qcsapi_unsigned_int val = 0;
	int retval = 0;

	retval = qcsapi_system_get_debug_value(QCSAPI_REBOOT_CAUSE, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "Reboot Cause - %u\n", val);

	return 0;
}

#define QCSAPI_AACS_VEC_BUFLEN 256
#define QCSAPI_AACS_VEC_MAXPR 250

static int local_aacs_print_vector(const call_bundle *cb, int32_t *pbuf, uint32_t len)
{
	qcsapi_output *print = cb->output;

	int i = 0;
	int j = 0;
	char tmpbuf[QCSAPI_AACS_VEC_BUFLEN];

	for (i = 0; i < len && j < QCSAPI_AACS_VEC_MAXPR; i++)
		j += sprintf((tmpbuf + j), "%d ", *(pbuf + i));
	print_out(print, "%s\n", tmpbuf);

	return 0;
}

static int local_aacs_load_vector(const call_bundle *cb, int argc, char *argv[],
					int32_t *pbuf, uint32_t *len, uint32_t offset)
{
	int i;

	/* skip command */
	if (*len < (argc - offset))
		return -EINVAL;

	*len = argc - offset;

	for (i = offset; i < argc; i++)
		pbuf[i-offset] = (uint32_t) atoi(argv[i]);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_thres_min_tbl_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_thres_min_tbl_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_thres_min_tbl_get)
{
	int retval = 0;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	retval = qcsapi_wifi_aacs_thres_min_tbl_get(interface, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_aacs_print_vector(cb, buf.val, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_thres_max_tbl_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_thres_max_tbl_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_thres_max_tbl_get)
{
	int retval = 0;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	retval = qcsapi_wifi_aacs_thres_max_tbl_get(interface, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_aacs_print_vector(cb, buf.val, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_rssi_tbl_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_vnode_rssi_tbl_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_rssi_tbl_get)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));

	retval = qcsapi_wifi_aacs_vnode_rssi_tbl_get(interface, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_aacs_print_vector(cb, buf.val, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_wgt_tbl_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_VNODETBL_SIZE;

	if (argc != AACS_VNODETBL_SIZE) {
		print_err(print, "Must provide %d weights.\n", AACS_VNODETBL_SIZE);
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_vnode_wgt_tbl_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_wgt_tbl_get)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	retval = qcsapi_wifi_aacs_vnode_wgt_tbl_get(interface, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_aacs_print_vector(cb, buf.val, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_vnode_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_vnode_get)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len;
	int i;

	for (i = 0; i < AACS_VNODETBL_SIZE; i++) {
		len = AACS_VNODEQRY_SIZE;
		memset(buf.val, 0, sizeof(buf.val));
		buf.val[0] = i;
		retval = qcsapi_wifi_aacs_vnode_get(interface, &buf, &len);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);

		print_out(print, "%d: rssi=%d weight=%d index=%d\n",
				i,
				buf.val[AACS_VNODEQRY_OFF_RSSI],
				buf.val[AACS_VNODEQRY_OFF_WGT],
				buf.val[AACS_VNODEQRY_OFF_IDX]);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 0);

	retval = qcsapi_wifi_aacs_dfs_thres_adj_tbl_set(interface, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get)
{
	int retval = 0;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	retval = qcsapi_wifi_aacs_dfs_thres_adj_tbl_get(interface, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	local_aacs_print_vector(cb, buf.val, len);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_excl_ch_set)
{
	int retval = 0;
	uint16_t int1 = AACS_UINT16_MAX;
	uint16_t int2 = AACS_UINT16_MAX;

	if (local_atou16("channel", argv[0], &int1, print) < 0)
		return -EINVAL;

	if (argc > 1) {
		if (local_atou16("channel end", argv[1], &int2, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_aacs_excl_ch_set(interface, int1, int2);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_excl_ch_get)
{
	int retval = 0;
	qcsapi_unsigned_int int1 = 0;
	qcsapi_unsigned_int int2 = 0;

	retval = qcsapi_wifi_aacs_excl_ch_get(interface, &int1, &int2);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", int1, int2);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_alt_excl_ch_set)
{
	int retval = 0;
	uint16_t int1 = AACS_UINT16_MAX;
	uint16_t int2 = AACS_UINT16_MAX;

	if (local_atou16("channel", argv[0], &int1, print) < 0)
		return -EINVAL;

	if (argc > 1) {
		if (local_atou16("channel end", argv[1], &int2, print) < 0)
			return -EINVAL;
	}

	retval = qcsapi_wifi_aacs_alt_excl_ch_set(interface, int1, int2);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_alt_excl_ch_get)
{
	int retval = 0;
	qcsapi_unsigned_int int1 = 0;
	qcsapi_unsigned_int int2 = 0;

	retval = qcsapi_wifi_aacs_alt_excl_ch_get(interface, &int1, &int2);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "%d %d\n", int1, int2);

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_sel_ch_set)
{
	int retval = -1;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_SELCH_SIZE + 1;
	uint32_t op;
	uint32_t cmd_len;

	if (argc > len)
		return qcsapi_report_usage(cb);

	cmd_len = strlen(argv[0]);
	if (!strncmp(argv[0], AACS_CH_OP_ADD_STR, strlen(AACS_CH_OP_ADD_STR)) &&
			cmd_len == strlen(AACS_CH_OP_ADD_STR))
		op = AACS_CH_OP_ADD;
	else if (!strncmp(argv[0], AACS_CH_OP_DEL_STR, strlen(AACS_CH_OP_DEL_STR)) &&
			cmd_len == strlen(AACS_CH_OP_DEL_STR))
		op = AACS_CH_OP_DEL;
	else if (!strncmp(argv[0], AACS_CH_OP_ADD_STR_ALT, strlen(AACS_CH_OP_ADD_STR_ALT)) &&
			cmd_len == strlen(AACS_CH_OP_ADD_STR_ALT))
		op = AACS_CH_OP_ADD_ALT;
	else if (!strncmp(argv[0], AACS_CH_OP_DEL_STR_ALT, strlen(AACS_CH_OP_DEL_STR_ALT)) &&
			cmd_len == strlen(AACS_CH_OP_DEL_STR_ALT))
		op = AACS_CH_OP_DEL_ALT;
	else
		return qcsapi_report_usage(cb);

	memset(buf.val, 0, sizeof(buf.val));
	local_aacs_load_vector(cb, argc, argv, buf.val, &len, 1);

	retval = qcsapi_wifi_aacs_sel_ch_set(interface, op, &buf, len);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_aacs_sel_ch_get)
{
	int retval = 0;
	struct qcsapi_int_array32 buf;
	uint32_t len = AACS_INCL_CH_LIST_LEN;
	int i;
	int j;
	int num_ch = 0;
	int op;

	if (!strncmp(argv[0], AACS_CH_OP_GET_STR, strlen(AACS_CH_OP_GET_STR)) &&
				strlen(argv[0]) == strlen(AACS_CH_OP_GET_STR))
		op = AACS_CH_OP_ADD;
	else if (!strncmp(argv[0], AACS_CH_OP_GET_STR_ALT, strlen(AACS_CH_OP_GET_STR_ALT)) &&
				strlen(argv[0]) == strlen(AACS_CH_OP_GET_STR_ALT))
		op = AACS_CH_OP_ADD_ALT;
	else
		return qcsapi_report_usage(cb);

	retval = qcsapi_wifi_aacs_sel_ch_get(interface, op, &buf, &len);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < len; i++) {
		for (j = 0; j < AACS_UINT32_BIT_SIZE; j++) {
			if (buf.val[i] & (1 << j)) {
				print_out(print, "%d ", i * AACS_UINT32_BIT_SIZE + j);
				num_ch++;
			}
		}
	}
	if (!num_ch)
		print_out(print, "No channel selected");
	print_out(print, "\n");

	return 0;
}

static const char *protocol_name_table[] = {
	"None",
	"WPA",
	"WPA2",
	"WPA WPA2",
	"WPA3"
};

static const char *encryption_name_table[] = {
	"None",
	"TKIP",
	"CCMP",
	"TKIP CCMP"
};

static const char *authentication_name_table[] = {
	"None",
	"PSK",
	"EAP",
	"SAE",
	"OWE",
	"DPP"
};

static const char *get_name_by_value(const int qcsapi_value, const char **qcsapi_lookup_table,
		const size_t lookup_table_size)
{
	const char *retaddr = NULL;

	if (qcsapi_value >= 0 && qcsapi_value < (int)lookup_table_size)
		retaddr = qcsapi_lookup_table[qcsapi_value];

	return retaddr;
}

static void local_show_ap_properties(qcsapi_output *print, const qcsapi_unsigned_int index_ap,
					const qcsapi_ap_properties *p_ap_properties)
{
	char mac_addr_string[24];
	char buffer[32];

	print_out(print, "AP %d:\n", index_ap);
	print_out(print, "\tSSID: %s\n", p_ap_properties->ap_name_SSID);
	sprintf(&mac_addr_string[0], MACFILTERINGMACFMT,
			p_ap_properties->ap_mac_addr[0],
			p_ap_properties->ap_mac_addr[1],
			p_ap_properties->ap_mac_addr[2],
			p_ap_properties->ap_mac_addr[3],
			p_ap_properties->ap_mac_addr[4], p_ap_properties->ap_mac_addr[5]);
	print_out(print, "\tMAC address: %s\n", &mac_addr_string[0]);
	print_out(print, "\tChannel: %d\n", p_ap_properties->ap_channel);
	print_out(print, "\tBandwidth: %d\n", p_ap_properties->ap_bw);
	print_out(print, "\tRSSI: %d\n", p_ap_properties->ap_RSSI);
	print_out(print, "\tHT secondary offset: %s\n",
			p_ap_properties->ap_ht_secoffset == IEEE80211_HTINFO_EXTOFFSET_ABOVE ?
				"Above" :
			p_ap_properties->ap_ht_secoffset == IEEE80211_HTINFO_EXTOFFSET_BELOW ?
				"Below" : "None");
	print_out(print, "\tcenter channel 1: %d\n", p_ap_properties->ap_chan_center1);
	print_out(print, "\tcenter channel 2: %d\n", p_ap_properties->ap_chan_center2);
	print_out(print, "\tLast seen: %u\n", p_ap_properties->ap_last_beacon);
	local_snprint_bitrate(buffer, sizeof(buffer), p_ap_properties->ap_best_data_rate);
	print_out(print, "\tBest Data Rate: %s\n", buffer);

	print_out(print, "\tSGI capability:");
	if (p_ap_properties->ap_flags &
			((1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ) | (1 <<
							QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ) | (1
							<< QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ) |
					(1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ))) {
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ))
			print_out(print, " 20MHz");
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ))
			print_out(print, " 40MHz");
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ))
			print_out(print, " 80MHz");
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ))
			print_out(print, " 160MHz");
		print_out(print, "\n");
	} else
		print_out(print, " None\n");

	if ((p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SEC_ENABLE)) != 0) {
		const char *value_name = NULL;

		print_out(print, "\tsecurity enabled\n");

		value_name = get_name_by_value(p_ap_properties->ap_protocol,
				protocol_name_table, ARRAY_SIZE(protocol_name_table));
		if (value_name == NULL)
			value_name = "(unknown)";
		print_out(print, "\tprotocol: %s\n", value_name);

		if (verbose_flag > 0) {
			value_name = get_name_by_value(p_ap_properties->ap_authentication_mode,
					authentication_name_table,
					ARRAY_SIZE(authentication_name_table));
			if (value_name == NULL)
				value_name = "(unknown)";
			print_out(print, "\tauthentication mode: %s\n", value_name);

			value_name = get_name_by_value(p_ap_properties->ap_encryption_modes,
					encryption_name_table, ARRAY_SIZE(encryption_name_table)
					);
			if (value_name == NULL)
				value_name = "(unknown)";
			print_out(print, "\tencryption modes: %s\n", value_name);
		}
	} else {
		print_out(print, "\tsecurity disabled\n");
	}

	print_out(print, "\n");
}

CALL_QCSAPI(call_qcsapi_wifi_show_access_points)
{
	int retval;
	uint32_t ap_count;
	uint32_t offchan = 0;
	qcsapi_unsigned_int i;
	qcsapi_ap_properties ap_properties;

	if (local_atou_bool("off chan", argv[0], &offchan, print) < 0)
		return -EINVAL;

	if (offchan)
		retval = qcsapi_wifi_get_results_AP_scan_by_scs(interface, &ap_count);
	else
		retval = qcsapi_wifi_get_results_AP_scan(interface, &ap_count);

	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	for (i = 0; i < ap_count && retval >= 0; i++) {
		if (offchan)
			retval = qcsapi_wifi_get_properties_AP_by_scs(interface, i, &ap_properties);
		else
			retval = qcsapi_wifi_get_properties_AP(interface, i, &ap_properties);
		if (retval >= 0)
			local_show_ap_properties(print, i + 1, &ap_properties);
	}

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_pta)
{
	uint32_t val = 0;
	int retval = 0;

	retval = qcsapi_wifi_get_pta(interface, &val);

	return qcsapi_report_int_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_pta_op_mode)
{
	int retval = 0;

	retval = qcsapi_wifi_set_pta_op_mode(interface, strtoul(argv[0], NULL, 16));

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_reg_chan_txpower_backoff_set)
{
	int retval;
	uint8_t channel;
	uint8_t is_percentage;
	uint8_t backoff;

	channel = atoi(argv[0]);
	is_percentage = !!atoi(argv[1]);
	backoff = atoi(argv[2]);

	if (channel > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %u\n", channel);
		return -EINVAL;
	}
	if (is_percentage && (backoff >= 100)) {
		print_err(print, "Invalid backoff %d\n", backoff);
		return -EINVAL;
	}

	retval = qcsapi_reg_chan_txpower_backoff_set(interface, channel, is_percentage, backoff);

	return qcsapi_report_complete(cb, retval);

}

CALL_QCSAPI(call_qcsapi_reg_chan_txpower_backoff_get)
{
	int retval;
	uint8_t chan;
	uint8_t is_percentage;
	uint8_t backoff;

	chan = atoi(argv[0]);
	if (chan > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %s\n", chan);
		return -EINVAL;
	}

	retval = qcsapi_reg_chan_txpower_backoff_get(interface, chan, &is_percentage, &backoff);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (backoff > 0)
		print_out(print, "%s: %d\n", is_percentage ? "percentage" : "absolute", backoff);
	else
		print_out(print, "backoff off\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_grab_config)
{
	int ret = 0;
	const char *output_path;
	FILE *output_stream = NULL;
	size_t bytes_written = 0;

	output_path = argv[0];
	output_stream = fopen(output_path, "w");
	if (output_stream == NULL) {
		print_err(print, "Failed to open %s: %s\n", output_path, strerror(errno));
		ret = -EFAULT;
		goto out;
	}

	print_err(print, "Grabbing config...\n");
	ret = qcsapi_grabber_write_config_blob(output_stream, QCSAPI_GRABBER_PARAM_ALL,
			&bytes_written);
	if (ret) {
		print_err(print, "Can not write blob to %s\n", output_path);
		goto out;
	}

out:
	if (output_stream)
		fclose(output_stream);

	return qcsapi_report_complete(cb, ret);
}

CALL_QCSAPI(call_qcsapi_wifi_repeater_mode_cfg)
{
	int retval;
	qcsapi_unsigned_int radio = 0;
	unsigned int enable;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	if (local_atou_bool("enable", argv[1], &enable, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_repeater_mode_cfg(radio, enable);

	return qcsapi_report_complete(cb, retval);

}

struct urepeater_params {
	const char *str;
	qcsapi_urepeater_type type;
	uint16_t is_get;
	uint16_t is_set;
};

static const struct urepeater_params urep_params[] = {
	{"max_level", qcsapi_urepeater_max_level, 1, 1},
	{"curr_level", qcsapi_urepeater_curr_level, 1, 0}
};

CALL_QCSAPI(call_qcsapi_wifi_set_urepeater_params)
{
	int retval = 0;
	qcsapi_urepeater_type type = qcsapi_urepeater_none;
	uint32_t value;
	int i;

	for (i = 0; i < ARRAY_SIZE(urep_params); i++) {
		if (strcmp(argv[0], urep_params[i].str) == 0) {
			if (!urep_params[i].is_set)
				break;

			type = urep_params[i].type;
			break;
		}
	}

	if (type == qcsapi_urepeater_none) {
		print_out(print, "Invalid parameter type \"%s\"\n", argv[0]);
		return -EINVAL;
	}

	if (local_atou32_range("value", argv[1], &value, print,
				REPEATER_MIN_LEVEL, REPEATER_MAX_LEVEL) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_urepeater_params(type, (int)value);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_get_urepeater_params)
{
	int retval = -1;
	qcsapi_urepeater_type type = qcsapi_urepeater_none;
	int value;
	int i;

	for (i = 0; i < ARRAY_SIZE(urep_params); i++) {
		if (strcmp(argv[0], urep_params[i].str) == 0) {
			if (!urep_params[i].is_get)
				break;

			type = urep_params[i].type;
			break;
		}
	}

	if (type == qcsapi_urepeater_none) {
		print_out(print, "Invalid parameter type \"%s\"\n", argv[0]);
		return -EINVAL;
	}

	retval = qcsapi_wifi_get_urepeater_params(type, &value);

	if (verbose_flag >= 0) {
		if (retval >= 0) {
			print_out(print, "%d\n", value);
		} else {
			report_qcsapi_error(cb, retval);
		}
	}

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_get_legacy_bbic)
{
	qcsapi_unsigned_int val = 0;
	int retval;

	retval = qcsapi_wifi_get_legacy_bbic(interface, &val);

	return qcsapi_report_uint_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_wifi_set_legacy_bbic)
{
	int retval;

	qcsapi_unsigned_int val = (qcsapi_unsigned_int) atoi(argv[0]);

	retval = qcsapi_wifi_set_legacy_bbic(interface, val);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_set_6g_min_rate)
{
	int retval;
	uint32_t nss;
	uint32_t mcs;

	if (local_atou32_range("nss", argv[0], &nss, print, 0, IEEE80211_HE_NSS3) < 0)
		return -EINVAL;

	if (local_atou32_range("mcs", argv[1], &mcs, print, 0,
					IEEE80211_AX_MCS_6G_MIN_RATE_MAX) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_set_6g_min_rate(interface, nss, mcs);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_freq2chan_and_band)
{
	int retval;
	uint32_t freq;
	uint32_t band;
	uint32_t chan;

	if (local_atou32("freq", argv[0], &freq, print) < 0)
		return -EINVAL;

	retval = qcsapi_wifi_freq2chan_and_band(freq, &band, &chan);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (band < ARRAY_SIZE(qcsapi_freq_band_table))
		print_out(print, "Frequency band: %s\n",
				qcsapi_freq_band_table[band].name);
	print_out(print, "Channel: %u\n", chan);

	return 0;
}

CALL_QCSAPI(call_qcsapi_set_3addr_br_config)
{
	int retval;
	int enable;
	qcsapi_unsigned_int radio;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	if (strcmp(argv[1], "dhcp_chaddr") == 0) {
		if (local_atoi_bool("enable", argv[2], &enable, print) < 0)
			return -EINVAL;
		retval = qcsapi_radio_set_3addr_br_config(radio,
						e_qcsapi_3addr_br_dhcp_chaddr, enable);
	} else {
		return qcsapi_report_usage(cb);
	}

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_get_3addr_br_config)
{
	int retval;
	uint32_t val;
	qcsapi_unsigned_int radio;

	if (local_atou32("radio", argv[0], &radio, print) < 0)
		return -EINVAL;

	if (strcmp(argv[1], "dhcp_chaddr") != 0)
		return -EINVAL;

	retval = qcsapi_radio_get_3addr_br_config(radio,
						e_qcsapi_3addr_br_dhcp_chaddr, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (val)
		print_out(print, "Enabled\n");
	else
		print_out(print, "Disabled\n");

	return 0;
}

#define QCSAPI_CONFIG_NAME_LEN	128
#define QCSAPI_CONFIG_FILE_PATH "/lib/firmware/qtn/"

static void local_macaddr_remove_colons(char *mac)
{
	int i;
	int j = 0;

	for (i = 0; mac[i] != '\0'; i++) {
		if (mac[i] != ':') {
			if (i != j)
				mac[j] = mac[i];
			j++;
		}
	}
	mac[j] = '\0';
}

static int local_get_qtn_ep_cfg_file_name(FILE *f, char *output_dir,
		char *output_name, unsigned int name_len)
{
	char last_line[64];
	char *macaddr = NULL;
	char *version = NULL;
	uint32_t parsed_ver = 0;
	uint64_t pos;
	int ret;
	uint64_t end;

	if (!f)
		return -EINVAL;

	if (fseek(f, 0, SEEK_END))
		return -EINVAL;

	/*
	 * The config file is a tar file followed by a newline character and
	 * metadata, which contains (for version 1) the following fields, separated by spaces.
	 * - config file version
	 * - EP MAC address
	 * - Tar file checksum
	 * Refer to the save_config script for the full format.
	 */
	end = ftell(f);
	pos = end;
	while (pos) {
		if (!fseek(f, --pos, SEEK_SET)) {
			if (fgetc(f) == '\n') {
				if (pos < (end - 1))
					break;
			}
		} else {
			return -EINVAL;
		}
	}

	/* first 2 values are in "version macaddr" format */
	if (fgets(last_line, sizeof(last_line), f) == NULL)
		return -EFAULT;
	last_line[sizeof(last_line) - 1] = '\0';

	version = strtok(last_line, " ");
	macaddr = strtok(NULL, " ");
	if (!strlen(version) || !strlen(macaddr))
		return -EFAULT;

	local_macaddr_remove_colons(macaddr);

	parsed_ver = strtol(version, NULL, 10);
	if (!parsed_ver)
		return -EFAULT;

	ret = snprintf(output_name, name_len,
			"%sqtn-%s-v%u.cfg",
			output_dir[0] ? output_dir : QCSAPI_CONFIG_FILE_PATH,
			macaddr, parsed_ver);

	if (ret >= name_len)
		return -qcsapi_buffer_overflow;

	return ret;
}

CALL_QCSAPI(call_qcsapi_save_config)
{
	int ret = 0;
	const int buflen = QCSAPI_CONFIG_NAME_LEN;
	char output_name[buflen];
	char output_dir[buflen];
	char output_name_tmp[buflen];
	FILE *output_fp = NULL;
	unsigned int bytes_remain = 0;
	unsigned int bytes_copied = 0;
	unsigned int offset = 0;
	struct qcsapi_data_3Kbytes *buf = NULL;
	int suc = 0;
	int output_dir_len = 0;
	int copy_len = 0;

	if (access(QCSAPI_SYSTEM_STATUS_FILE, F_OK) == 0) {
		print_err(print, "%s must be run from the host system\n",
			cb->call_entry->cmdname);
		return -EINVAL;
	}

	if (getuid() != 0) {
		print_err(print, "Only root can do that\n");
		return -EINVAL;
	}

	memset(output_name, 0, buflen);
	memset(output_dir, 0, buflen);
	memset(output_name_tmp, 0, buflen);
	if (argv[0]) {
		output_dir_len = snprintf(output_dir, buflen, "%s", argv[0]);
		if (output_dir_len >= (buflen - 2)) {
			print_err(print, "Output directory name is too long\n");
			return -EINVAL;
		}
		if (output_dir[output_dir_len - 1] != '/') {
			output_dir[output_dir_len] = '/';
			output_dir_len++;
		}
	}

	if (output_dir_len) {
		copy_len = snprintf(output_name_tmp, buflen, "%sqtn_ep_cfg.tmp", output_dir);
		if (copy_len >= buflen) {
			print_err(print, "Output filename is too long\n");
			return -EINVAL;
		}
	} else {
		copy_len = snprintf(output_name_tmp, buflen,
				QCSAPI_CONFIG_FILE_PATH "qtn_ep_cfg.tmp");
		if (copy_len >= buflen) {
			print_err(print, "Output filename is too long\n");
			return -EINVAL;
		}
		ret = mkdir(QCSAPI_CONFIG_FILE_PATH, 0755);
		if ((ret != 0) && (errno != EEXIST)) {
			print_err(print, "Make %s directory failed\n",
					QCSAPI_CONFIG_FILE_PATH);
			return -EINVAL;
		}
	}

	output_fp = fopen(output_name_tmp, "wb+");
	if (output_fp == NULL)
		return -EFAULT;

	buf = local_malloc(cb, sizeof(*buf));
	if (!buf)
		goto out;

	ret = -EFAULT;
	do {
		memset(buf, 0, sizeof(*buf));
		bytes_copied = 0;
		bytes_remain = 0;
		ret = qcsapi_save_config(buf, offset, &bytes_copied, &bytes_remain);
		if (ret < 0)
			goto out;

		offset += bytes_copied;

		if (fwrite(buf, 1, bytes_copied, output_fp) != bytes_copied)
			break;

		if (bytes_remain == 0) {
			suc = 1;
			ret = 0;
		}

	} while (bytes_remain > 0);

out:
	free(buf);

	if (suc) {
		ret = local_get_qtn_ep_cfg_file_name(output_fp, output_dir,
				output_name, sizeof(output_name));
		fclose(output_fp);
		if ((ret <= 0) || (output_name[0] == '\0')) {
			print_err(print, "Cannot get saved file name\n");
			return -EINVAL;
		}

		ret = rename(output_name_tmp, output_name);
	} else {
		if (output_fp)
			fclose(output_fp);
		unlink(output_name_tmp);
	}

	if (ret == 0)
		print_out(print, "Config saved to %s\n", output_name);

	return qcsapi_report_complete(cb, ret);
}

CALL_QCSAPI(call_qcsapi_get_config_status)
{
	int retval = 0;
	char val[QCSAPI_MSG_BUFSIZE] = {0};

	retval = qcsapi_get_config_status(val, sizeof(val));

	return qcsapi_report_str_or_error(cb, retval, val);
}

CALL_QCSAPI(call_qcsapi_gpio_config)
{
	int retval;
	gpio_cmd cmd = GPIO_CMD_INVALID;
	uint32_t pin;
	uint32_t value;
	uint32_t val;

	if (argc == 2) {
		if (!strcasecmp(argv[0], "read"))
			cmd = GPIO_CMD_READ;
		else if (!strcasecmp(argv[0], "disconnect"))
			cmd = GPIO_CMD_DISCONNECT;
	} else if (argc == 3) {
		if (!strcasecmp(argv[0], "mode"))
			cmd = GPIO_CMD_MODE;
		else if (!strcasecmp(argv[0], "write"))
			cmd = GPIO_CMD_WRITE;
		else if (!strcasecmp(argv[0], "connect"))
			cmd = GPIO_CMD_CONNECT;
	}

	if (cmd == GPIO_CMD_INVALID)
		return qcsapi_report_usage(cb);

	if (local_atou32("pin", argv[1], &pin, print) < 0)
		return -EINVAL;

	if (argc == 3 && local_atou32("value", argv[2], &value, print) < 0)
		return -EINVAL;

	retval = qcsapi_gpio(cmd, pin, value, &val);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	if (cmd == GPIO_CMD_READ)
		print_out(print, "%u\n", val);
	else if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_multi_psk_info_append)
{
	int retval;

	retval = qcsapi_wifi_multi_psk_info_append(interface, argv[0]);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_multi_psk_info_read)
{
	int retval;
	struct qcsapi_data_4Kbytes *buf;

	buf = local_malloc(cb, sizeof(*buf));
	if (!buf)
		return -ENOMEM;

	retval = qcsapi_wifi_multi_psk_info_read(interface, (char *)buf->data);
	if (retval < 0)
		report_qcsapi_error(cb, retval);
	else
		print_out(print, "%s\n", buf->data);

	free(buf);

	return retval;
}

CALL_QCSAPI(call_qcsapi_wifi_multi_psk_info_replace)
{
	int retval;
	const char *buf = NULL;

	if (argc == 1)
		buf = argv[0];

	retval = qcsapi_wifi_multi_psk_info_replace(interface, buf);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_start_phy_scan)
{
	int retval = 0;
	int i = 0;
	string_1024 tmp = {0};
	uint16_t bw = qcsapi_bw_20MHz;
	uint32_t freqs_num = 0;
	struct qcsapi_int_array64 freqs;
	uint32_t *p_freqs = (uint32_t *) freqs.val;
	uint32_t max_freq = min(IEEE80211_MAX_DUAL_BAND_CHANNELS, ARRAY_SIZE(freqs.val));

	for (i = 0; i < (argc - 1); i++) {
		if ((strcmp(argv[i], "bw") == 0) &&
				!local_atou16_range("bw", argv[i + 1], &bw, print, 0, 0xFF))
			i++;
		else if (strcmp(argv[i], "freqs") == 0) {
			strncpy(tmp, argv[i + 1], sizeof(tmp) - 1);
			if (!tmp[0])
				break;
			local_string_to_u32_list(print, tmp, p_freqs, &freqs_num);
			if (freqs_num >= max_freq)
				freqs_num = max_freq;
			i++;
		} else {
			break;
		}
	}

	if (i < argc)
		return qcsapi_report_usage(cb);

	retval = qcsapi_wifi_start_phy_scan(interface, bw, &freqs, freqs_num);

	return qcsapi_report_complete(cb, retval);
}

CALL_QCSAPI(call_qcsapi_wifi_xcac_set)
{
	int retval = 0;
	struct qcsapi_xcac_op_req cac_req;
	qcsapi_unsigned_int status = 0;

	memset(&cac_req, 0, sizeof(cac_req));

	if (strcasecmp(argv[0], "start") == 0) {
		if (argc != 5)
			return qcsapi_report_usage(cb);

		cac_req.command = QCSAPI_XCAC_CMD_START;

		retval = local_atou16_range("channel", argv[1], &cac_req.channel, print,
					QCSAPI_MIN_CHANNEL_5G, QCSAPI_MAX_CHANNEL);
		if (retval < 0)
			return retval;

		retval = local_atou16_range("bw", argv[2], &cac_req.bw, print,
					qcsapi_bw_20MHz, qcsapi_bw_160MHz);
		if (retval < 0)
			return retval;

		retval = local_atou16_range("method", argv[3], &cac_req.method, print,
					QCSAPI_XCAC_METHOD_MIN, QCSAPI_XCAC_METHOD_MAX);
		if (retval < 0)
			return retval;

		retval = local_atou16_range("action", argv[4], &cac_req.action, print,
					QCSAPI_XCAC_ACTION_MIN, QCSAPI_XCAC_ACTION_MAX);
		if (retval < 0)
			return retval;

	} else if (strcasecmp(argv[0], "stop") == 0) {
		cac_req.command = QCSAPI_XCAC_CMD_STOP;
	} else {
		return qcsapi_report_usage(cb);
	}

	retval = qcsapi_wifi_xcac_set(interface, &cac_req, &status);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "Status: 0x%x\n", status);
	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_xcac_get)
{
	int retval = 0;
	struct qcsapi_xcac_op_req cac_req;
	struct qcsapi_xcac_get_result cac_result;

	memset(&cac_req, 0, sizeof(cac_req));
	memset(&cac_result, 0, sizeof(cac_result));

	if (strcasecmp(argv[0], "status") != 0)
		return qcsapi_report_usage(cb);
	cac_req.command = QCSAPI_XCAC_CMD_GET_STATUS;

	retval = qcsapi_wifi_xcac_get(interface, &cac_req, &cac_result);
	if (retval < 0)
		return report_qcsapi_error(cb, retval);

	print_out(print, "status %d, channel %d, bw %d, channel status 0x%x\n",
			cac_result.status, cac_result.channel,
			cac_result.bw, cac_result.chan_status);
	return 0;
}

static void local_output_phy_info(qcsapi_output *print, struct qcsapi_chan_phy_stats *phy_stats)
{
	print_out(print, "chan_no=%d, flag=%d, bw=%d, ",
			phy_stats->chan_no, phy_stats->flag, phy_stats->bandwidth);
	print_out(print, "busy_20=%d, busy_40=%d, busy_80=%d, ",
			phy_stats->busy_20, phy_stats->busy_40, phy_stats->busy_80);
	print_out(print, "tx_20=%d, rx_20=%d, rx_others_20=%d, ",
			phy_stats->tx_20, phy_stats->rx_20, phy_stats->rx_others_20);
	print_out(print, "scan_typ=%s, scan_age=%d, noise_20=%d, aggr_scan_duration=%d\n",
			phy_stats->scan_type ==
					QCSAPI_PHY_STATS_SCAN_TYPE_ACTIVE ?
					"active" : "passive",
			phy_stats->scan_age, phy_stats->noise_20,
			phy_stats->aggr_scan_duration);
}

static int local_chan_list_bitmap_to_u8_list(qcsapi_output *print, const uint8_t *input_bitmap,
					const size_t size, uint8_t *output_list, uint32_t *number)
{
	int i;
	uint32_t j;

	if (!input_bitmap || !output_list)
		return -1;

	for (i = 0, j = 0; i < (size * NBBY); i++) {
		if (isset(input_bitmap, i))
			output_list[j++] = i;
	}

	if (number)
		*number = j;

	return 0;
}

CALL_QCSAPI(call_qcsapi_wifi_get_chan_phy_info)
{
	int retval = 0;
	int i = 0;
	int j = 0;
	int count = 0;
	int getall = 0;
	int getsaved = 0;
	int should_cont = 0;
	int len_left = 0;
	unsigned int total_no = 0;
	char *p_end;
	struct qcsapi_data_256bytes chan_list;
	struct qcsapi_data_256bytes get_chan_list;
	struct qcsapi_chan_phy_info *buf;
	struct qcsapi_chan_phy_stats *phy_stats;
	struct qcsapi_chan_phy_stats *p_offset;
	const char *ifname = interface;
	int buflen = sizeof(struct qcsapi_data_1Kbytes);

	if (strcmp(argv[j], "-saved") == 0) {
		if (argc <= 1)
			return qcsapi_report_usage(cb);

		getsaved = 1;
		++j;
	}

	memset(&chan_list, 0, sizeof(chan_list));

	if (strcmp(argv[j], "all") == 0) {
		getall = 1;
		memset(&get_chan_list, 0, sizeof(get_chan_list));
		retval = qcsapi_wifi_get_chan_list(ifname, &get_chan_list, qcsapi_chlist_flag_scan);
		if (retval < 0)
			return -EFAULT;
		retval = local_chan_list_bitmap_to_u8_list(print, get_chan_list.data,
						sizeof(get_chan_list), chan_list.data, &total_no);
		if (retval < 0)
			return -EFAULT;
	} else {
		retval = local_string_to_u8_list(print, argv[j], chan_list.data, &total_no);
		if (retval < 0)
			return report_qcsapi_error(cb, retval);
	}

	buf = local_malloc(cb, buflen);
	if (!buf)
		return -ENOMEM;

	do {
		memset(buf, 0, buflen);

		if (getsaved)
			buf->flag |= QCSAPI_CHAN_PHY_STATS_GET_SAVED;

		p_end = (char *) buf + buflen;
		p_offset = buf->phy_stats + 1;

		i = 0;
		do {
			buf->phy_stats[i].chan_no = chan_list.data[count];
			i++;
			count++;
			p_offset++;
		} while ((char *) p_offset <= p_end && count < total_no);

		buf->num = i;

		retval = qcsapi_wifi_get_chan_phy_info(ifname,
					(struct qcsapi_data_1Kbytes *) buf);
		if (retval < 0) {
			free(buf);
			return report_qcsapi_error(cb, retval);
		}

		len_left = sizeof(struct qcsapi_data_1Kbytes) -
					sizeof(struct qcsapi_chan_phy_info);
		for (i = 0; i < buf->num; i++) {
			if (len_left < sizeof(struct qcsapi_chan_phy_stats))
				break;
			phy_stats = &buf->phy_stats[i];
			if (phy_stats->flag & QCSAPI_CHAN_PHY_STATS_READY)
				local_output_phy_info(print, phy_stats);
			else if (!getall)
				print_out(print, "Stats of chan %d not ready\n",
								phy_stats->chan_no);

			len_left -= sizeof(struct qcsapi_chan_phy_stats);
		}

		should_cont = count < total_no;
	} while (should_cont);

	free(buf);
	return 0;
}

/* end of programs to call individual QCSAPIs */

const char qcsapi_usage_vlan_config[] = "\n"
	"    wifi0_0 enable\n"
	"    wifi0_0 disable\n"
	"    <interface> reset\n"
	"    <interface> access <VLAN ID> [delete] [vlan_prio] [drop_ctagged]\n"
	"    <interface> trunk <VLAN ID> [ {tag | untag} ] [default] [delete] [vlan_prio]\n"
	"    <interface> dynamic {0 | 1}\n"
	"    wifi0_0 drop_stag {0 | 1}\n"
	"    <interface> ethertype <Ethertype> [delete]\n";

const char qcsapi_usage_dpp_config[] = "\n"
	"    <WiFi interface> cfg_get <param1>\n"
	"    <WiFi interface> cfg_set <param1> <value1>\n"
	"    <WiFi interface> <dpp_command> <param1> <value1>... [<param8> <value8>]";

const char qcsapi_usage_gpio[] = "\n"
	"    mode <pin> <mode>\n"
	"    write <pin> <value>\n"
	"    read <pin>\n"
	"    connect <pin> <value>\n"
	"    disconnect <pin>";

const char qcsapi_usage_start_dcs_scan[] =
	"<WiFi interface> [interval <interval>] [interval_sec <secs>]\n"
	"    [duration <msecs>] [dwell <msecs>] [spacing <msecs>] [chanlist <chanlist>]";

const char qcsapi_usage_restore_default_config[] = "\n"
	"    [ {fdr | nfr | dcdc} ] [ {ap | sta | repeater} ]\n"
	"    [ <WiFi interface> {ap | sta | repeater} ]... [ip] [noreboot]";

const char qcsapi_usage_thermal[] = "\n"
	"    init\n"
	"    cfg_interval <interval>\n"
	"    cfg_radio <radio> <stage id> <low temp> <high temp> <tx chains> <tx power backoff>\n"
	"    start\n"
	"    stop\n"
	"    status\n"
	"    exit";

static const struct call_table_entry_s call_table_entry[] = {
	{
	.fn = call_qcsapi_errno_get_message,
	.cmdname = "get_error_message",
	.usage = "<errno>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_store_ipaddr,
	.cmdname = "store_ipaddr",
	.usage = "<ipaddr/netmask>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_enable,
	.cmdname = "enable_interface",
	.usage = "<network interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_BSSID,
	.cmdname = "interface_bssid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_mac_addr,
	.cmdname = "get_mac_addr",
	.cmdname_alt = "get_macaddr",
	.usage = "<network device>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_set_mac_addr,
	.cmdname = "set_mac_addr",
	.cmdname_alt = "set_macaddr",
	.usage = "<network device> <mac address>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_counter,
	.cmdname = "get_counter",
	.usage = "<network device> <counter name>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_counter,
	},
	{
	.fn = call_qcsapi_interface_get_counter64,
	.cmdname = "get_counter64",
	.usage = "<network device> <counter name>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_counter,
	},
	{
	.fn = call_qcsapi_pm_get_counter,
	.cmdname = "get_pm_counter",
	.usage = "<network device> <counter name> <PM interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_counter,
	},
	{
	.fn = call_qcsapi_pm_get_elapsed_time,
	.cmdname = "get_pm_elapsed_time",
	.usage = "<PM interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_flash_image_update,
	.cmdname = "flash_image_update",
	.usage = "<image file path> {live | safety | uboot}",
	.args = {2, 2},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_firmware_get_version,
	.cmdname = "get_firmware_version",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_system_get_time_since_start,
	.cmdname = "get_time_since_start",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_system_status,
	.cmdname = "get_sys_status",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_cpu_usage,
	.cmdname = "get_cpu_usage",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_memory_usage,
	.cmdname = "get_memory_usage",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_random_seed,
	.cmdname = "get_random_seed",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_random_seed,
	.cmdname = "set_random_seed",
	.usage = "<random_string> <entropy_count>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_led_get,
	.cmdname = "get_led",
	.usage = "<LED/GPIO pin number>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_led_set,
	.cmdname = "set_led",
	.usage = "<LED/GPIO pin number> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_led_pwm_enable,
	.cmdname = "set_led_pwm",
	.usage = "<LED/GPIO pin number> {0 | 1 <high_count> <low_count>}",
	.args = {1, 3},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_led_brightness,
	.cmdname = "set_led_brightness",
	.usage = "<LED/GPIO pin number> <level>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_gpio_get_config,
	.cmdname = "get_gpio_config",
	.usage = "<GPIO pin number>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_gpio_set_config,
	.cmdname = "set_gpio_config",
	.usage = "<GPIO pin number> <configuration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_gpio_monitor_reset_device,
	.cmdname = "monitor_reset_device",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_gpio_enable_wps_push_button,
	.cmdname = "enable_wps_push_button",
	.usage = "<GPIO pin> {0 | 1} [intr]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_LED,
	},
	{
	.fn = call_qcsapi_thermal,
	.cmdname = "thermal",
	.usage = qcsapi_usage_thermal,
	.args = {1, 7},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_file_path_get_config,
	.cmdname = "get_file_path",
	.usage = "security",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_file_path_config,
	},
	{
	.fn = call_qcsapi_file_path_set_config,
	.cmdname = "set_file_path",
	.usage = "security <new location>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_file_path_config,
	},
	{
	.fn = call_qcsapi_wifi_set_wifi_macaddr,
	.cmdname = "set_wifi_macaddr",
	.cmdname_alt = "set_wifi_mac_addr",
	.usage = "<new MAC addr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_create_restricted_bss,
	.cmdname = "wifi_create_restricted_bss",
	.usage = "<WiFi interface> [<mac_addr>]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_create_bss,
	.cmdname = "wifi_create_bss",
	.usage = "<WiFi interface> [<mac_addr>]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_bss,
	.cmdname = "wifi_remove_bss",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_primary_interface,
	.cmdname = "get_primary_interface",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_interface_by_index,
	.cmdname = "get_interface_by_index",
	.usage = "<index> <radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_interface_by_index_all,
	.cmdname = "get_interface_by_index_all",
	.usage = "<index> <radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_mode,
	.cmdname = "get_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mode,
	.cmdname = "set_mode",
	.usage = "<WiFi interface> {ap | sta}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_phy_mode,
	.cmdname = "get_phy_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_phy_mode,
	.cmdname = "set_phy_mode",
	.usage = "<WiFi interface> <Mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_phy_mode_required,
	.cmdname = "get_phy_mode_required",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_phy_mode_required,
	.cmdname = "set_phy_mode_required",
	.usage = "<WiFi interface> {none | 11n | 11ac | 11ax}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_reload_in_mode,
	.cmdname = "reload_in_mode",
	.cmdname_alt = "reload",
	.usage = "<WiFi interface> {ap | sta | repeater}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_radio_rfenable,
	.cmdname = "rfenable",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_service_control,
	.cmdname = "service_control",
	.usage = "<service_name> {start|stop|enable|disable}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wfa_cert,
	.cmdname = "wfa_cert",
	.usage = "{0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_rfstatus,
	.cmdname = "rfstatus",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_startprod,
	.cmdname = "startprod",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_supported_freq_bands,
	.cmdname = "get_supported_freq_bands",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bw,
	.cmdname = "get_bw",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_bw,
	.cmdname = "set_bw",
	.usage = "<WiFi interface> <bw>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_BSSID,
	.cmdname = "get_bssid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_config_BSSID,
	.cmdname = "get_config_bssid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_ssid_get_bssid,
	.cmdname = "get_ssid_bssid",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_wifi_ssid_set_bssid,
	.cmdname = "set_ssid_bssid",
	.usage = "<WiFi interface> <SSID> <BSSID MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_wifi_get_SSID,
	.cmdname = "get_ssid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_SSID2,
	.cmdname = "get_ssid2",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_SSID,
	.cmdname = "set_ssid",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_channel,
	.cmdname = "get_channel",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan,
	.cmdname = "get_chan",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_channel,
	.cmdname = "set_channel",
	.usage = "<WiFi interface> <new_channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_chan,
	.cmdname = "set_chan",
	.usage = "<WiFi interface> <new_channel> <new_bw> <new_band>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_channel_and_bw,
	.cmdname = "set_chan_and_bw",
	.usage = "<WiFi interface> <new_channel> <new_bw>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_channel_and_bw,
	.cmdname = "get_chan_and_bw",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_wea_cac_en,
	.cmdname = "set_wea_cac_en",
	.usage = "<WiFi interface> <en_value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_auto_channel,
	.cmdname = "get_auto_channel",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_auto_channel,
	.cmdname = "set_auto_channel",
	.usage = "<WiFi interface> {enable | disable}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_standard,
	.cmdname = "get_standard",
	.cmdname_alt = "get_802.11",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dtim,
	.cmdname = "get_dtim",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dtim,
	.cmdname = "set_dtim",
	.usage = "<WiFi interface> <DTIM interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_assoc_limit,
	.cmdname = "get_dev_assoc_limit",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_assoc_limit,
	.cmdname = "set_dev_assoc_limit",
	.usage = "<WiFi interface> <limit>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bss_assoc_limit,
	.cmdname = "get_bss_assoc_limit",
	.usage = "<group>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_bss_assoc_limit,
	.cmdname = "set_bss_assoc_limit",
	.usage = "<group> <limit>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_SSID_group_id,
	.cmdname = "set_ssid_group_id",
	.usage = "<WiFi interface> <group>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_SSID_group_id,
	.cmdname = "get_ssid_group_id",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_SSID_assoc_reserve,
	.cmdname = "set_ssid_assoc_reserve",
	.usage = "<group> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_SSID_assoc_reserve,
	.cmdname = "get_ssid_assoc_reserve",
	.usage = "<group>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_status,
	.cmdname = "get_status",
	.usage = "<network interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_set_ip4,
	.cmdname = "set_ip",
	.usage = "<network interface> {ipaddr <IP address> | netmask <netmask>}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_ip4,
	.cmdname = "get_ip",
	.usage = "<network interface> [{ipaddr | netmask}]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_set_mtu,
	.cmdname = "set_mtu",
	.usage = "<network interface> <mtu>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_interface_get_mtu,
	.cmdname = "get_mtu",
	.usage = "<network interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_list_channels,
	.cmdname = "get_channel_list",
	.cmdname_alt = "get_list_of_channels",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan_list,
	.cmdname = "get_chan_list",
	.usage = "<WiFi interface> [ {available | disabled | scan | active | ocac_off} ]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_chan_list,
	.cmdname = "set_chan_list",
	.usage = "<WiFi interface> {disabled | scan} <freq band> <channel list> {0 | 1}",
	.args = {4, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_supp_chans,
	.cmdname = "get_supp_chan",
	.usage = "<WiFi interface> <MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mode_switch,
	.cmdname = "get_mode_switch",
	.cmdname_alt = "get_wifi_mode_switch",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_noise,
	.cmdname = "get_noise",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_rssi_by_chain,
	.cmdname = "get_rssi_by_chain",
	.usage = "<WiFi interface> <RF chain number>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_avg_snr,
	.cmdname = "get_avg_snr",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_option,
	.cmdname = "get_option",
	.usage = "<WiFi interface> <option>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_option,
	},
	{
	.fn = call_qcsapi_wifi_set_option,
	.cmdname = "set_option",
	.usage = "<WiFi interface> <option> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_option,
	},
	{
	.fn = call_qcsapi_wifi_set_parameter,
	.cmdname = "set_wifi_param",
	.usage = "<WiFi interface> <parameter> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_wifi_parameter,
	},
	{
	.fn = call_qcsapi_wifi_get_parameter,
	.cmdname = "get_wifi_param",
	.usage = "<WiFi interface> <parameter>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_wifi_parameter,
	},
	{
	.fn = call_qcsapi_wifi_get_rates,
	.cmdname = "get_rates",
	.usage = "<WiFi interface> {basic | operational | possible}",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_rates,
	},
	{
	.fn = call_qcsapi_wifi_set_rates,
	.cmdname = "set_rates",
	.usage = "<WiFi interface> {basic | operational} <rate> [<rate>...]",
	.args = {1, -1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_rates,
	},
	{
	.fn = call_qcsapi_wifi_get_max_bitrate,
	.cmdname = "get_max_bitrate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_beacon_type,
	.cmdname = "get_beacon",
	.cmdname_alt = "get_beacon_type",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_beacon_type,
	.cmdname = "set_beacon",
	.cmdname_alt = "set_beacon_type",
	.usage = "<WiFi interface> <beacon type>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_beacon_interval,
	.cmdname = "get_beacon_interval",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_beacon_interval,
	.cmdname = "set_beacon_interval",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_list_regulatory_regions,
	.cmdname = "get_list_regulatory_regions",
	.cmdname_alt = "get_regulatory_regions",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_regulatory_tx_power,
	.cmdname = "get_regulatory_tx_power",
	.usage = "<WiFi interface> <channel> <region>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_configured_tx_power,
	.cmdname = "get_configured_tx_power",
	.usage = "<WiFi interface> <channel> <reg region> [<bandwidth>] [<bf> <ss>]",
	.args = {2, 5},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_regulatory_channel,
	.cmdname = "set_regulatory_channel",
	.usage = "<WiFi interface> <channel> <region> [<offset>]",
	.args = {2, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_regulatory_region,
	.cmdname = "set_regulatory_region",
	.usage = "<WiFi interface> <region>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_regulatory_region,
	.cmdname = "get_regulatory_region",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_overwrite_country_code,
	.cmdname = "overwrite_country_code",
	.usage = "<WiFi interface> <current country code> <new country code>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_country_code,
	.cmdname = "get_country_code",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_list_regulatory_channels,
	.cmdname = "get_list_regulatory_channels",
	.usage = "<region> [<bandwidth>] [<ifname>]",
	.args = {1, 3},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_list_regulatory_bands,
	.cmdname = "get_list_regulatory_bands",
	.usage = "<region> [<ifname>]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_regulatory_db_version,
	.cmdname = "get_regulatory_db_version",
	.usage = "[<index>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_regulatory_tx_power_cap,
	.cmdname = "apply_regulatory_cap",
	.usage = "<interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_restore_regulatory_tx_power,
	.cmdname = "restore_regulatory_tx_power",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_chan_pri_inactive,
	.cmdname = "set_chan_pri_inactive",
	.usage = "<WiFi interface> <channel> [<inactive>] [<options>]",
	.args = {1, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan_pri_inactive,
	.cmdname = "get_chan_pri_inactive",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan_disabled,
	.cmdname = "get_chan_disabled",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan_usable,
	.cmdname = "get_chan_usable",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reg_chan_txpower_get,
	.cmdname = "get_chan_power_table",
	.cmdname_alt = "reg_chan_txpower_get",
	.usage = "<WiFi interface> [-n] [-f <type>] [-t <dntx>] <channel:nss:bf:fem_pri:bw>",
	.args = {1, 6},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reg_chan_txpower_set,
	.cmdname = "reg_chan_txpower_set",
	.usage = "<ifname> [-t <dntx>] <chan:nss:bf:fem_pri:bw> <power_list>",
	.args = {2, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reg_chan_txpower_path_get,
	.cmdname = "reg_chan_txpower_path_get",
	.usage = "<ifname>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_power_selection,
	.cmdname = "get_power_selection",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_power_selection,
	.cmdname = "set_power_selection",
	.usage = "<power_selection>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_congestion_idx,
	.cmdname = "get_congest_idx",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_supported_tx_power_levels,
	.cmdname = "get_supported_tx_power",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_current_tx_power_level,
	.cmdname = "get_current_tx_power",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_power_constraint,
	.cmdname = "set_power_constraint",
	.usage = "<WiFi interface> <power constraint>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_power_constraint,
	.cmdname = "get_power_constraint",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_tpc_interval,
	.cmdname = "set_tpc_query_interval",
	.usage = "<WiFi interface> <interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tpc_interval,
	.cmdname = "get_tpc_query_interval",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_assoc_records,
	.cmdname = "get_assoc_records",
	.usage = "<WiFi interface> [<reset>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_list_dfs_channels,
	.cmdname = "get_list_dfs_channels",
	.usage = "<regulatory region> {0 | 1} [<bw>] [<ifname>]",
	.args = {2, 4},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_is_channel_dfs,
	.cmdname = "is_channel_dfs",
	.usage = "<regulatory region> <channel> [<ifname>]",
	.args = {2, 3},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_alt_channel,
	.cmdname = "get_dfs_alt_channel",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_alt_channel,
	.cmdname = "set_dfs_alt_channel",
	.usage = "<WiFi interface> <alternative channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_reentry,
	.cmdname = "start_dfsreentry",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_radar_chain,
	.cmdname = "set_radar_chain",
	.usage = "<radar_blockid> <radar_chain_selection>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_cce_channels,
	.cmdname = "get_scs_cce_channels",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_cce_channels,
	.cmdname = "get_dfs_cce_channels",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_csw_records,
	.cmdname = "get_csw_records",
	.usage = "<WiFi interface> [reset]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radar_status,
	.cmdname = "get_radar_status",
	.usage = "<WiFi interface> <DFS-Channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_WEP_encryption_level,
	.cmdname = "get_wep_encryption_level",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_WEP_key,
	.cmdname = "get_wep_key",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_WEP_key,
	.cmdname = "set_wep_key",
	.usage = "<WiFi interface> <index> <key>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_WEP_key_index,
	.cmdname = "get_wep_key_index",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_WEP_key_index,
	.cmdname = "set_wep_key_index",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_WEP_config,
	.cmdname = "remove_wep_config",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_WPA_encryption_modes,
	.cmdname = "get_wpa_encryption_modes",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_WPA_encryption_modes,
	.cmdname = "set_wpa_encryption_modes",
	.usage = "<WiFi interface> <encryption modes>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_WPA_authentication_mode,
	.cmdname = "get_wpa_authentication_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_WPA_authentication_mode,
	.cmdname = "set_wpa_authentication_mode",
	.usage = "<WiFi interface> <authentication modes>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_params,
	.cmdname = "get_params",
	.usage = "<WiFi interface> <param1> [<param2>... <param8>]",
	.args = {1, 8},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_params,
	.cmdname = "set_params",
	.usage = "<WiFi interface> <param1> <value1> [<param2> <value2>]... [<param8> <value8>]",
	.args = {2, 16},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_params,
	.cmdname = "remove_params",
	.usage = "<WiFi interface> <param1> [<param2>... <param8>]",
	.args = {1, 8},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_interworking,
	.cmdname = "get_interworking",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_interworking,
	.cmdname = "set_interworking",
	.usage = "<WiFi interface> <interworking>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_80211u_params,
	.cmdname = "get_80211u_params",
	.usage = "<WiFi interface> <80211u_param>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_80211u_params,
	.cmdname = "set_80211u_params",
	.usage = "<WiFi interface> <param> <value1> [<value2>]",
	.args = {2, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_set_sec_agent,
	.cmdname = "sec_agent",
	.usage = "{enable | disable | start} <key>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_sec_agent_status,
	.cmdname = "get_sec_agent_status",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_nai_realms,
	.cmdname = "get_nai_realms",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_nai_realm,
	.cmdname = "add_nai_realm",
	.usage = "<WiFi interface> <encoding> <NAI realm> <EAP methods>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_nai_realm,
	.cmdname = "del_nai_realm",
	.usage = "<WiFi interface> <Nai realm>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_roaming_consortium,
	.cmdname = "add_roaming_consortium",
	.usage = "<WiFi interface> <OI>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_roaming_consortium,
	.cmdname = "del_roaming_consortium",
	.usage = "<WiFi interface> <OI>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_roaming_consortium,
	.cmdname = "get_roaming_consortium",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_venue_name,
	.cmdname = "get_venue_name",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_venue_name,
	.cmdname = "add_venue_name",
	.usage = "<WiFi interface> <lang_code> <name>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_venue_name,
	.cmdname = "del_venue_name",
	.usage = "<WiFi interface> <lang_code> <name>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_oper_friendly_name,
	.cmdname = "get_oper_friendly_name",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_oper_friendly_name,
	.cmdname = "add_oper_friendly_name",
	.usage = "<WiFi interface> <lang_code> <name>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_oper_friendly_name,
	.cmdname = "del_oper_friendly_name",
	.usage = "<WiFi interface> <lang_code> <name>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_hs20_conn_capab,
	.cmdname = "get_hs20_conn_capab",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_hs20_conn_capab,
	.cmdname = "add_hs20_conn_capab",
	.usage = "<WiFi interface> <ip_proto> <port_num> <status>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_hs20_conn_capab,
	.cmdname = "del_hs20_conn_capab",
	.usage = "<WiFi interface> <ip_proto> <port_num> <status>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_hs20_icon,
	.cmdname = "add_hs20_icon",
	.usage = "<WiFi interface> <width> <height> <lang_code> <type> <name> <file_path>",
	.args = {6, 6},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_hs20_icon,
	.cmdname = "get_hs20_icon",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_hs20_icon,
	.cmdname = "del_hs20_icon",
	.usage = "<WiFi interface> <name>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_osu_server_uri,
	.cmdname = "add_osu_server_uri",
	.usage = "<WiFi interface> <osu_server_uri>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_osu_server_uri,
	.cmdname = "get_osu_server_uri",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_osu_server_uri,
	.cmdname = "del_osu_server_uri",
	.usage = "<WiFi interface> <osu_server_uri>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_add_osu_server_param,
	.cmdname = "add_osu_server_param",
	.usage = "<WiFi interface> <osu_server_uri> <param> <value>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_get_osu_server_param,
	.cmdname = "get_osu_server_param",
	.usage = "<WiFi interface> <uri> <param>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_security_del_osu_server_param,
	.cmdname = "del_osu_server_param",
	.usage = "<WiFi interface> <osu_server_uri> <param> [<value>]",
	.args = {2, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_hs20_status,
	.cmdname = "get_hs20_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_hs20_status,
	.cmdname = "set_hs20_status",
	.usage = "<WiFi interface> <hs>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_hs20_params,
	.cmdname = "get_hs20_params",
	.usage = "<WiFi interface> <hs_param>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_hs20_params,
	.cmdname = "set_hs20_params",
	.usage = "<WiFi interface> <hs_param> <val1> [<val2>... <val6>]",
	.args = {2, 7},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_remove_11u_param,
	.cmdname = "remove_11u_param",
	.usage = "<WiFi interface> <param>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_remove_hs20_param,
	.cmdname = "remove_hs20_param",
	.usage = "<WiFi interface> <hs_param>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_proxy_arp,
	.cmdname = "set_proxy_arp",
	.usage = "<WiFi interface> <proxy_arp>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_proxy_arp,
	.cmdname = "get_proxy_arp",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_l2_ext_filter,
	.cmdname = "get_l2_ext_filter",
	.usage = "<WiFi interface> <param>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_l2_ext_filter,
	.cmdname = "set_l2_ext_filter",
	.usage = "<WiFi interface> <param> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_IEEE11i_encryption_modes,
	.cmdname = "get_ieee11i_encryption_modes",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_IEEE11i_encryption_modes,
	.cmdname = "set_ieee11i_encryption_modes",
	.usage = "<WiFi interface> <encryption_mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_IEEE11i_authentication_mode,
	.cmdname = "get_ieee11i_authentication_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_IEEE11i_authentication_mode,
	.cmdname = "set_ieee11i_authentication_mode",
	.usage = "<WiFi interface> <authentication_mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_michael_errcnt,
	.cmdname = "get_michael_errcnt",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pre_shared_key,
	.cmdname = "get_pre_shared_key",
	.cmdname_alt = "get_psk",
	.usage = "<WiFi interface> <key index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_set_pre_shared_key,
	.cmdname = "set_pre_shared_key",
	.cmdname_alt = "set_psk",
	.usage = "<WiFi interface> <key index> <preshared key>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_add_radius_auth_server_cfg,
	.cmdname = "add_radius_auth_server_cfg",
	.usage = "<WiFi interface> <ipaddr> <port> <sh_key>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_del_radius_auth_server_cfg,
	.cmdname = "del_radius_auth_server_cfg",
	.usage = "<WiFi interface> <ipaddr> <port>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radius_auth_server_cfg,
	.cmdname = "get_radius_auth_server_cfg",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_add_radius_acct_server_cfg,
	.cmdname = "add_radius_acct_server_cfg",
	.usage = "<WiFi interface> <ipaddr> <port> <sh_key>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_del_radius_acct_server_cfg,
	.cmdname = "del_radius_acct_server_cfg",
	.usage = "<WiFi interface> <ipaddr> <port>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radius_acct_server_cfg,
	.cmdname = "get_radius_acct_server_cfg",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_own_ip_addr,
	.cmdname = "set_own_ip_addr",
	.usage = "<WiFi interface> <own ip addr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_psk_auth_failures,
	.cmdname = "get_psk_auth_failures",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_key_passphrase,
	.cmdname = "get_passphrase",
	.cmdname_alt = "get_key_passphrase",
	.usage = "<WiFi interface> 0",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_set_key_passphrase,
	.cmdname = "set_passphrase",
	.cmdname_alt = "set_key_passphrase",
	.usage = "<WiFi interface> 0 <passphrase>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_group_key_interval,
	.cmdname = "get_group_key_interval",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_group_key_interval,
	.cmdname = "set_group_key_interval",
	.usage = "<WiFi interface> <group key interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pairwise_key_interval,
	.cmdname = "get_pairwise_key_interval",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pairwise_key_interval,
	.cmdname = "set_pairwise_key_interval",
	.usage = "<WiFi interface> <pairwise key interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pmf,
	.cmdname = "get_pmf",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pmf,
	.cmdname = "set_pmf",
	.usage = "<WiFi interface> <pmf_cap>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_count_associations,
	.cmdname = "get_count_assoc",
	.cmdname_alt = "get_association_count",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_associated_device_mac_addr,
	.cmdname = "get_associated_device_mac_addr",
	.cmdname_alt = "get_station_mac_addr",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_associated_device_ip_addr,
	.cmdname = "get_associated_device_ip_addr",
	.cmdname_alt = "get_station_ip_addr",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_link_quality,
	.cmdname = "get_link_quality",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rssi_per_association,
	.cmdname = "get_rssi",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_hw_noise_per_association,
	.cmdname = "get_hw_noise",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rssi_in_dbm_per_association,
	.cmdname = "get_rssi_dbm",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_meas_rssi_in_dbm_per_association,
	.cmdname = "get_meas_rssi_dbm",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association,
	.cmdname = "get_meas_rssi_minmax",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_set_rssi_meas_period,
	.cmdname = "set_rssi_meas_period",
	.usage = "<WiFi interface> <period>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_rssi_meas_period,
	.cmdname = "get_rssi_meas_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_snr_per_association,
	.cmdname = "get_snr",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rx_bytes_per_association,
	.cmdname = "get_rx_bytes",
	.cmdname_alt = "get_assoc_rx_bytes",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_bytes_per_association,
	.cmdname = "get_tx_bytes",
	.cmdname_alt = "get_assoc_tx_bytes",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rx_packets_per_association,
	.cmdname = "get_rx_packets",
	.cmdname_alt = "get_assoc_rx_packets",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_packets_per_association,
	.cmdname = "get_tx_packets",
	.cmdname_alt = "get_assoc_tx_packets",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_err_packets_per_association,
	.cmdname = "get_tx_err_packets",
	.cmdname_alt = "get_assoc_tx_err_packets",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_allretries_per_association,
	.cmdname = "get_tx_allretries",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_exceed_retry_per_association,
	.cmdname = "get_tx_exceed_retry",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_retried_per_association,
	.cmdname = "get_tx_retried",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_retried_percent_per_association,
	.cmdname = "get_tx_retried_percent",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_bw_per_association,
	.cmdname = "get_assoc_bw",
	.usage = "<WiFi interface> <index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_phy_rate_per_association,
	.cmdname = "get_tx_phy_rate",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rx_phy_rate_per_association,
	.cmdname = "get_rx_phy_rate",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_mcs_per_association,
	.cmdname = "get_tx_mcs",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_rx_mcs_per_association,
	.cmdname = "get_rx_mcs",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_achievable_tx_phy_rate_per_association,
	.cmdname = "get_achievable_tx_phy_rate",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_achievable_rx_phy_rate_per_association,
	.cmdname = "get_achievable_rx_phy_rate",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_auth_enc_per_association,
	.cmdname = "get_auth_enc_per_assoc",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_tput_caps,
	.cmdname = "get_tput_caps",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_connection_mode,
	.cmdname = "get_connection_mode",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_vendor_per_association,
	.cmdname = "get_vendor",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_max_mimo,
	.cmdname = "get_max_mimo",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_node_counter,
	.cmdname = "get_node_counter",
	.usage = "<WiFi interface> <node index> <counter type> [<remote>]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_node_param,
	.cmdname = "get_node_param",
	.usage = "<WiFi interface> <node index> <param name> [<remote> [<param>...]]",
	.args = {1, -1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_node_stats,
	.cmdname = "get_node_stats",
	.usage = "<WiFi interface> <node index> [<remote>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_node_list,
	.cmdname = "get_node_list",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_node_infoset,
	.cmdname = "get_node_infoset",
	.usage = "<WiFi interface> <MAC addr> <set id>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_node_infoset_all,
	.cmdname = "get_node_infoset_all",
	.usage = "<WiFi interface> <set id>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_sis_infoset_all,
	.cmdname = "get_scan_infoset_all",
	.usage = "<WiFi interface> <set id>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_if_infoset,
	.cmdname = "get_if_infoset",
	.usage = "<WiFi interface> <set id>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_max_queued,
	.cmdname = "get_max_queued",
	.usage = "<WiFi interface> <node index> [<remote> [<reset>]]",
	.args = {0, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_disassociate,
	.cmdname = "disassociate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_disassociate_sta,
	.cmdname = "disassociate_sta",
	.usage = "<WiFi interface> <MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_reassociate,
	.cmdname = "reassociate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_associate,
	.cmdname = "associate",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mac_address_filtering,
	.cmdname = "get_macaddr_filter",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mac_address_filtering,
	.cmdname = "set_macaddr_filter",
	.usage = "<WiFi interface> {0 | 1 | 2}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_is_mac_address_authorized,
	.cmdname = "is_macaddr_authorized",
	.cmdname_alt = "is_mac_addr_authorized",
	.usage = "<WiFi interface> <MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_authorized_mac_addresses,
	.cmdname = "get_authorized_macaddr",
	.cmdname_alt = "get_authorized_mac_addr",
	.usage = "<WiFi interface> [<list size>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_denied_mac_addresses,
	.cmdname = "get_denied_macaddr",
	.cmdname_alt = "get_blocked_macaddr",
	.usage = "<WiFi interface> [<list size>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_authorize_mac_address,
	.cmdname = "authorize_macaddr",
	.cmdname_alt = "authorize_mac_addr",
	.usage = "<WiFi interface> <MAC address1> [... <MAC address8>]",
	.args = {1, 8},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_deny_mac_address,
	.cmdname = "deny_macaddr",
	.cmdname_alt = "block_macaddr",
	.usage = "<WiFi interface> <MAC address1> [... <MAC address8>]",
	.args = {1, 8},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_mac_address,
	.cmdname = "remove_macaddr",
	.cmdname_alt = "remove_mac_addr",
	.usage = "<WiFi interface> <MAC address1> [... <MAC address8>]",
	.args = {1, 8},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_clear_mac_address_filters,
	.cmdname = "clear_mac_filters",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mac_address_reserve,
	.cmdname = "set_macaddr_reserve",
	.usage = "<WiFi interface> <addr> <mask> <block direction> [<allowed ethernet type>]",
	.args = {3, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mac_address_reserve,
	.cmdname = "get_macaddr_reserve",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_clear_mac_address_reserve,
	.cmdname = "clear_macaddr_reserve",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mac_list,
	.cmdname = "get_mac_list",
	.usage = "<WiFi interface>  {authorized | denied}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_backoff_fail_max,
	.cmdname = "backoff_fail_max",
	.usage = "<WiFi interface> <max failure count>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_backoff_timeout,
	.cmdname = "backoff_timeout",
	.usage = "<WiFi interface> <timeout>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_wpa_status,
	.cmdname = "get_wpa_status",
	.usage = "<WiFi interface> <MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_auth_state,
	.cmdname = "get_auth_state",
	.usage = "<WiFi interface> < MAC address>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_disconn_info,
	.cmdname = "get_disconn_info",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_reset_disconn_info,
	.cmdname = "reset_disconn_info",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pairing_id,
	.cmdname = "get_pairing_id",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pairing_id,
	.cmdname = "set_pairing_id",
	.usage = "<WiFi interface> <pairing ID>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pairing_enable,
	.cmdname = "get_pairing_enable",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pairing_enable,
	.cmdname = "set_pairing_enable",
	.usage = "<WiFi interface> <pairing enable flag>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_txqos_sched_tbl,
	.cmdname = "set_txqos_sched_tbl",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_txqos_sched_tbl,
	.cmdname = "get_txqos_sched_tbl",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_registrar_report_button_press,
	.cmdname = "registrar_report_pbc",
	.cmdname_alt = "registrar_report_button_press",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_registrar_report_pin,
	.cmdname = "registrar_report_pin",
	.usage = "<WiFi interface> <PIN>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_registrar_get_pp_devname,
	.cmdname = "registrar_get_pp_devname",
	.usage = "<WiFi interface> [ blacklist ]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_registrar_set_pp_devname,
	.cmdname = "registrar_set_pp_devname",
	.usage = "<WiFi interface> [<blacklist>] <list of device IDs>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_enrollee_report_button_press,
	.cmdname = "enrollee_report_pbc",
	.cmdname_alt = "enrollee_report_button_press",
	.usage = "<WiFi interface> [{any | <BSSID>}]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_enrollee_report_pin,
	.cmdname = "enrollee_report_pin",
	.usage = "<WiFi interface> {[any | <BSSID> ]} <PIN>",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_enrollee_generate_pin,
	.cmdname = "enrollee_generate_pin",
	.usage = "<WiFi interface> {[any | <BSSID> ]}",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_ap_pin,
	.cmdname = "get_wps_ap_pin",
	.usage = "<WiFi interface> [<force>]",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_ap_pin,
	.cmdname = "set_wps_ap_pin",
	.usage = "<WiFi interface> <wps_pin>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_save_ap_pin,
	.cmdname = "save_wps_ap_pin",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_enable_ap_pin,
	.cmdname = "enable_wps_ap_pin",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_sta_pin,
	.cmdname = "get_wps_sta_pin",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_state,
	.cmdname = "get_wps_state",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_configured_state,
	.cmdname = "get_wps_configured_state",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_configured_state,
	.cmdname = "set_wps_configured_state",
	.usage = "<WiFi interface> {0 | 1 | 2}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_runtime_state,
	.cmdname = "get_wps_runtime_state",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_allow_pbc_overlap_status,
	.cmdname = "get_allow_pbc_overlap_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_allow_pbc_overlap,
	.cmdname = "allow_pbc_overlap",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_check_config,
	.cmdname = "check_wps_config",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_param,
	.cmdname = "get_wps_param",
	.usage = "<WiFi interface> <WPS parameter name>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_param,
	.cmdname = "set_wps_param",
	.usage = "<WiFi interface> <WPS parameter name> <WPS parameter value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_access_control,
	.cmdname = "set_wps_access_control",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_access_control,
	.cmdname = "get_wps_access_control",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_non_wps_set_pp_enable,
	.cmdname = "set_non_wps_pp_enable",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_non_wps_get_pp_enable,
	.cmdname = "get_non_wps_pp_enable",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_cancel,
	.cmdname = "wps_cancel",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_pbc_in_srcm,
	.cmdname = "set_wps_pbc_in_srcm",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_get_pbc_in_srcm,
	.cmdname = "get_wps_pbc_in_srcm",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_set_timeout,
	.cmdname = "wps_set_timeout",
	.usage = "<WiFi interface> <timeout value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_on_hidden_ssid,
	.cmdname = "wps_on_hidden_ssid",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_on_hidden_ssid_status,
	.cmdname = "wps_on_hidden_ssid_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_upnp_enable,
	.cmdname = "wps_upnp_enable",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wps_upnp_status,
	.cmdname = "wps_upnp_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_registrar_set_default_pbc_bss,
	.cmdname = "registrar_set_default_pbc_bss",
	.usage = "<WiFi interface> | null",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_registrar_get_default_pbc_bss,
	.cmdname = "registrar_get_default_pbc_bss",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_radio_registrar_set_default_pbc_bss,
	.cmdname = "radio_registrar_set_default_pbc_bss",
	.usage = "<radio> {<WiFi interface> | null}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_radio_registrar_get_default_pbc_bss,
	.cmdname = "radio_registrar_get_default_pbc_bss",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dwell_times,
	.cmdname = "set_dwell_times",
	.usage = "<WiFi interface> <max_active> <min_active> <max_passive> <min_passive>",
	.args = {4, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dwell_times,
	.cmdname = "get_dwell_times",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_bgscan_dwell_times,
	.cmdname = "set_bgscan_dwell_times",
	.usage = "<WiFi interface> <active> <passive>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bgscan_dwell_times,
	.cmdname = "get_bgscan_dwell_times",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scan_chan_list,
	.cmdname = "get_scan_chan_list",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scan_chan_list,
	.cmdname = "set_scan_chan_list",
	.usage = "<WiFi interface> default | <channel list>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_start_scan,
	.cmdname = "start_scan",
	.usage = "<WiFi interface> [<algorithm> [<channels>] [<control flag>...]]",
	.args = {0, -1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_cancel_scan,
	.cmdname = "cancel_scan",
	.usage = "<WiFi interface> [force]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scan_status,
	.cmdname = "get_scanstatus",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_cac_status,
	.cmdname = "get_cacstatus",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wait_scan_completes,
	.cmdname = "wait_scan_completes",
	.usage = "<WiFi interface> <timeout>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scan_chk_inv,
	.cmdname = "set_scan_chk_inv",
	.usage = "<WiFi interface> <interval>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scan_chk_inv,
	.cmdname = "get_scan_chk_inv",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_update_bss_cfg,
	.cmdname = "update_bss_cfg",
	.usage = "<WiFi interface> {ap <interface> | sta [<fmt>] <ssid>} <param> <value> [<type>]",
	.args = {4, 6},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_create_SSID,
	.cmdname = "create_ssid",
	.cmdname_alt = "ssid_create_ssid",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_remove_SSID,
	.cmdname = "remove_ssid",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_verify_SSID,
	.cmdname = "verify_ssid",
	.cmdname_alt = "ssid_verify_ssid",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID>",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_rename_SSID,
	.cmdname = "rename_ssid",
	.cmdname_alt = "ssid_rename_ssid",
	.usage = "<WiFi interface> [{str | hexstr}] <SSID> [{str | hexstr}] <new SSID>",
	.args = {2, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_get_SSID_list,
	.cmdname = "get_ssid_list",
	.usage = "<WiFi interface> [<max SSIDs>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_get_SSID2_list,
	.cmdname = "get_ssid2_list",
	.usage = "<WiFi interface> [<max SSIDs>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_get_protocol,
	.cmdname = "ssid_get_proto",
	.cmdname_alt = "get_ssid_proto",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_protocol,
	.cmdname = "ssid_set_proto",
	.cmdname_alt = "set_ssid_proto",
	.usage = "<WiFi interface> <SSID> <protocol type>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_get_encryption_modes,
	.cmdname = "ssid_get_encryption_modes",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_encryption_modes,
	.cmdname = "ssid_set_encryption_modes",
	.usage = "<WiFi interface> <SSID> <encryption modes>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_get_group_encryption,
	.cmdname = "ssid_get_group_encryption",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_group_encryption,
	.cmdname = "ssid_set_group_encryption",
	.usage = "<WiFi interface> <SSID> <encryption_mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_get_authentication_mode,
	.cmdname = "ssid_get_authentication_mode",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_authentication_mode,
	.cmdname = "ssid_set_authentication_mode",
	.usage = "<WiFi interface> <SSID> <authentication mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_get_pre_shared_key,
	.cmdname = "ssid_get_pre_shared_key",
	.usage = "<WiFi interface> <SSID> <key index>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_SSID_index,
	},
	{
	.fn = call_qcsapi_SSID_set_pre_shared_key,
	.cmdname = "ssid_set_pre_shared_key",
	.usage = "<WiFi interface> <SSID> <key index> <preshared key>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_SSID_index,
	},
	{
	.fn = call_qcsapi_SSID_get_key_passphrase,
	.cmdname = "ssid_get_passphrase",
	.cmdname_alt = "ssid_get_key_passphrase",
	.usage = "<WiFi interface> <SSID> 0",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_SSID_index,
	},
	{
	.fn = call_qcsapi_SSID_set_key_passphrase,
	.cmdname = "ssid_set_passphrase",
	.cmdname_alt = "ssid_set_key_passphrase",
	.usage = "<WiFi interface> <SSID> 0 <new passphrase>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_SSID_index,
	},
	{
	.fn = call_qcsapi_SSID_get_pmf,
	.cmdname = "ssid_get_pmf",
	.usage = "<WiFi interface> <SSID>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_pmf,
	.cmdname = "ssid_set_pmf",
	.usage = "<WiFi interface> <SSID> {0 | 1 | 2}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_get_wps_SSID,
	.cmdname = "ssid_get_wps_ssid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_SSID_get_params,
	.cmdname = "ssid_get_params",
	.usage = "<WiFi interface> <ssid> [<param1> <param2>... <param8>]",
	.args = {0, 7},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_SSID_set_params,
	.cmdname = "ssid_set_params",
	.usage = "<WiFi interface> <ssid> <param1> <val1> [<param2> <val2>]... [<param8> <val8>]",
	.args = {0, 16},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_select_SSID,
	},
	{
	.fn = call_qcsapi_wifi_vlan_config,
	.cmdname = "vlan_config",
	.usage = qcsapi_usage_vlan_config,
	.args = {1, 9},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_show_vlan_config,
	.cmdname = "show_vlan_config",
	.usage = "<ifname> [tagrx]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_start_cca,
	.cmdname = "start_cca",
	.usage = "<WiFi interface> <channel> <duration>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_disable_wps,
	.cmdname = "disable_wps",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_results_AP_scan,
	.cmdname = "get_results_ap_scan",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_count_APs_scanned,
	.cmdname = "get_count_aps_scanned",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_properties_AP,
	.cmdname = "get_properties_ap",
	.usage = "<WiFi interface> <index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_wps_ie_scanned_AP,
	.cmdname = "get_wps_ie_scanned_ap",
	.usage = "<WiFi interface> <index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_time_associated_per_association,
	.cmdname = "get_time_associated",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_wds_add_peer,
	.cmdname = "wds_add_peer",
	.usage = "<WiFi interface> <BSSID of peer AP> [encrypt]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wds_remove_peer,
	.cmdname = "wds_remove_peer",
	.usage = "<WiFi interface> <BSSID of peer AP>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wds_get_peer_address,
	.cmdname = "wds_get_peer_address",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wds_set_psk,
	.cmdname = "wds_set_psk",
	.usage = "<WiFi interface> <BSSID of peer AP> {<WDS PSK> | NULL}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wds_set_mode,
	.cmdname = "wds_set_mode",
	.usage = "<WiFi interface> <BSSID of peer AP> {rbs | mbs | wds | reset}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wds_get_mode,
	.cmdname = "wds_get_mode",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_qos_get_param,
	.cmdname = "get_qos_param",
	.usage = "<WiFi interface> {0|1|2|3} {1|2|3|4|6} [ {0|1} ]",
	.args = {2, 3},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_qos_set_param,
	.cmdname = "set_qos_param",
	.usage = "<WiFi interface> {0|1|2|3} {1|2|3|4|6} {value} [ {0|1} ]",
	.args = {3, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_wmm_ac_map,
	.cmdname = "get_wmm_ac_map",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_wmm_ac_map,
	.cmdname = "set_wmm_ac_map",
	.usage = "<WiFi interface> {0|1|2|3|4|5|6|7} {0|1|2|3}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dscp_ac_map,
	.cmdname = "get_dscp_ac_map",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dscp_ac_map,
	.cmdname = "set_dscp_ac_map",
	.usage = "<WiFi interface> <0-64>[,0-64]... <0-3>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dscp_ac_table,
	.cmdname = "get_dscp_ac_table",
	.usage = "<0-7>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dscp_ac_table,
	.cmdname = "set_dscp_ac_table",
	.usage = "<0-7> <0-64>[0-64]... <0-3>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dscp_tid_table,
	.cmdname = "get_dscp_tid_table",
	.usage = "<0-7>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dscp_tid_table,
	.cmdname = "set_dscp_tid_table",
	.usage = "<0-7> <0-64>[0-64]... <0-7>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dscp_vap_link,
	.cmdname = "get_dscp_vap_link",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dscp_vap_link,
	.cmdname = "set_dscp_vap_link",
	.usage = "<WiFi interface> <DSCP2UP table index>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_ac_agg_hold_time,
	.cmdname = "get_ac_agg_hold_time",
	.usage = "<WiFi interface> <ac>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ac_agg_hold_time,
	.cmdname = "set_ac_agg_hold_time",
	.usage = "<WiFi interface> <ac> <agg_hold_time>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_qos_map,
	.cmdname = "set_qos_map",
	.usage = "<WiFi interface> <qos_map_set>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_del_qos_map,
	.cmdname = "del_qos_map",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_qos_map,
	.cmdname = "get_qos_map",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_send_qos_map_conf,
	.cmdname = "send_qos_map_conf",
	.usage = "<WiFi interface> <STA MAC addr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dscp_tid_map,
	.cmdname = "get_dscp_tid_map",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_priority,
	.cmdname = "get_priority",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_priority,
	.cmdname = "set_priority",
	.usage = "<WiFi interface> <priority>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_airfair,
	.cmdname = "get_airfair",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_airfair,
	.cmdname = "set_airfair",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_airquota,
	.cmdname = "get_airquota",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_airquota,
	.cmdname = "set_airquota",
	.usage = "<WiFi interface> <airquota>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_airquota_node,
	.cmdname = "get_airquota_node",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_airquota_node,
	.cmdname = "set_airquota_node",
	.usage = "<WiFi interface> {<MAC address> | NULL} [ <airquota> ]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_premier_list,
	.cmdname = "get_premier_list",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_premier_list,
	.cmdname = "set_premier_list",
	.usage = "<WiFi interface> {null | <MAC1> [<MAC2> ... <MAC8>]}",
	.args = {1, 8},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_premier_rule,
	.cmdname = "get_premier_rule",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_premier_rule,
	.cmdname = "set_premier_rule",
	.usage = "<WiFi interface> <rule>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_qos_class,
	.cmdname = "get_qos_class",
	.usage = "<WiFi interface> {vap | node}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_qos_class,
	.cmdname = "set_qos_class",
	.usage = "<WiFi interface> {vap | node} <QOS class>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tpquota_node,
	.cmdname = "get_tpquota_node",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_tpquota_node,
	.cmdname = "set_tpquota_node",
	.usage = "<WiFi interface> {<MAC address> [<tpquota>] | 0}",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_tpquota_vap,
	.cmdname = "set_tpquota_vap",
	.usage = "<WiFi interface> <tpquota>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tpquota_vap,
	.cmdname = "get_tpquota_vap",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_config_get_parameter,
	.cmdname = "get_config_param",
	.cmdname_alt = "get_persistent_param",
	.usage = "<WiFi interface> <param_name>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_config_update_parameter,
	.cmdname = "update_config_param",
	.cmdname_alt = "update_persistent_param",
	.usage = "<WiFi interface> <param_name> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bootcfg_get_parameter,
	.cmdname = "get_bootcfg_param",
	.usage = "<param_name>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bootcfg_update_parameter,
	.cmdname = "update_bootcfg_param",
	.usage = "<param_name> <param_value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bootcfg_commit,
	.cmdname = "commit_bootcfg",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mcs_rate,
	.cmdname = "get_mcs_rate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mcs_rate,
	.cmdname = "set_mcs_rate",
	.usage = "<WiFi interface> <MCS index string>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_config_get_ssid_parameter,
	.cmdname = "get_persistent_ssid_param",
	.usage = "<WiFi interface> <param_name>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_config_update_ssid_parameter,
	.cmdname = "update_persistent_ssid_param",
	.usage = "<WiFi interface> <param_name> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_scs_enable,
	.cmdname = "enable_scs",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_scs_set_version,
	.cmdname = "scs_set_version",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_scs_switch_channel,
	.cmdname = "scs_switch_chan",
	.usage = "<WiFi interface> [<pick flags> [<check margin>]]",
	.args = {0, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_scs_pick_best_channel,
	.cmdname = "scs_pick_chan",
	.usage = "<WiFi interface> [<pick flags> [<check margin>]]",
	.args = {0, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_verbose,
	.cmdname = "set_scs_verbose",
	.usage = "<WiFi interface> [{0 | 1}]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_status,
	.cmdname = "get_scs_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_smpl_enable,
	.cmdname = "set_scs_smpl_enable",
	.usage = "<WiFi interface> [{0 | 1}]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_active_chan_list,
	.cmdname = "set_scs_active_chan_list",
	.usage = "<WiFi interface> <Channel list> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_active_chan_list,
	.cmdname = "get_scs_active_chan_list",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_smpl_dwell_time,
	.cmdname = "set_scs_smpl_dwell_time",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_smpl_dwell_time,
	.cmdname = "get_scs_smpl_dwell_time",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_sample_intv,
	.cmdname = "set_scs_smpl_intv",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_sample_intv,
	.cmdname = "get_scs_smpl_intv",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_sample_type,
	.cmdname = "set_scs_smpl_type",
	.usage = "<WiFi interface> <type>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_intf_detect_intv,
	.cmdname = "set_scs_intf_detect_intv",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_thrshld,
	.cmdname = "set_scs_thrshld",
	.usage = "<WiFi interface> <threshold_name> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_report_only,
	.cmdname = "set_scs_report_only",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_report,
	.cmdname = "get_scs_report",
	.usage = "<WiFi interface> {all | interference | score | current | autochan}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_cca_intf_smth_fctr,
	.cmdname = "set_scs_cca_intf_smth_fctr",
	.usage = "<WiFi interface> <smth_fctr_noxp> <smth_fctr_xped>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_chan_mtrc_mrgn,
	.cmdname = "set_scs_chan_mtrc_mrgn",
	.usage = "<WiFi interface> <chan_mtrc_mrgn>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_nac_monitor_mode,
	.cmdname = "set_scs_nac_monitor_mode",
	.usage = "<WiFi interface> {0 | 1 [<on period> <cycle period>]}",
	.args = {1, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_band_margin_check,
	.cmdname = "set_scs_band_margin_check",
	.usage = "<WiFi interface> <enable>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_band_margin,
	.cmdname = "set_scs_band_margin",
	.usage = "<WiFi interface> <margin>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_dfs_reentry_request,
	.cmdname = "get_scs_dfs_reentry_request",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_cca_intf,
	.cmdname = "get_scs_cca_intf",
	.usage = "<WiFi interface> <channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_param,
	.cmdname = "get_scs_params",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_stats,
	.cmdname = "set_scs_stats",
	.usage = "<WiFi interface> [{0 | 1}]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_burst_enable,
	.cmdname = "set_scs_burst_enable",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_burst_window,
	.cmdname = "set_scs_burst_window",
	.usage = "<WiFi interface> <window>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_burst_thresh,
	.cmdname = "set_scs_burst_thresh",
	.usage = "<WiFi interface> <threshold>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_burst_pause,
	.cmdname = "set_scs_burst_pause_time",
	.usage = "<WiFi interface> <pause_time>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_burst_switch,
	.cmdname = "set_scs_burst_force_switch",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scs_chan_pool,
	.cmdname = "get_scs_chan_pool",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_chan_pool,
	.cmdname = "set_scs_chan_pool",
	.usage = "<WiFi interface> <channel list>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_acs_param,
	.cmdname = "set_acs_param",
	.usage = "<WiFi interface> {<param> <value> | --help}",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_start_ocac,
	.cmdname = "start_ocac",
	.usage = "<WiFi interface> {auto | <DFS_channel>}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_stop_ocac,
	.cmdname = "stop_ocac",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_ocac_status,
	.cmdname = "get_ocac_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ocac_threshold,
	.cmdname = "set_ocac_thrshld",
	.usage = "<WiFi interface> <threshold_name> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ocac_dwell_time,
	.cmdname = "set_ocac_dwell_time",
	.usage = "<WiFi interface> <dwelltime>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ocac_duration,
	.cmdname = "set_ocac_duration",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ocac_cac_time,
	.cmdname = "set_ocac_cac_time",
	.usage = "<WiFi interface> <cac_time>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ocac_report_only,
	.cmdname = "set_ocac_report_only",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_start_dfs_s_radio,
	.cmdname = "start_dfs_s_radio",
	.usage = "<WiFi interface> {auto [<first DFS chan>] | <DFS channel>}",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_stop_dfs_s_radio,
	.cmdname = "stop_dfs_s_radio",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_s_radio_status,
	.cmdname = "get_dfs_s_radio_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_s_radio_availability,
	.cmdname = "get_dfs_s_radio_availability",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_threshold,
	.cmdname = "set_dfs_s_radio_thrshld",
	.usage = "<WiFi interface> <threshold_name> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_dwell_time,
	.cmdname = "set_dfs_s_radio_dwell_time",
	.usage = "<WiFi interface> <dwelltime>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_duration,
	.cmdname = "set_dfs_s_radio_duration",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_cac_time,
	.cmdname = "set_dfs_s_radio_cac_time",
	.usage = "<WiFi interface> <cac_time>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_report_only,
	.cmdname = "set_dfs_s_radio_report_only",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_wea_duration,
	.cmdname = "set_dfs_s_radio_wea_duration",
	.usage = "<WiFi interface> <duration>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_wea_cac_time,
	.cmdname = "set_dfs_s_radio_wea_cac_time",
	.usage = "<WiFi interface> <cac_time>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time,
	.cmdname = "set_dfs_s_radio_wea_dwell_time",
	.usage = "<WiFi interface> <dwelltime>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_dfs_s_radio_chan_off,
	.cmdname = "set_dfs_s_radio_chan_off",
	.usage = "<WiFi interface> <channel> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_s_radio_chan_off,
	.cmdname = "get_dfs_s_radio_chan_off",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_vendor_fix,
	.cmdname = "set_vendor_fix",
	.usage = "<WiFi interface> <param> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_rts_threshold,
	.cmdname = "get_rts_threshold",
	.usage = "<interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_rts_threshold,
	.cmdname = "set_rts_threshold",
	.usage = "<interface> <rts_threshold>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_soc_macaddr,
	.cmdname = "set_soc_macaddr",
	.usage = "<WiFi interface> <soc mac addr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_interface_stats,
	.cmdname = "get_interface_stats",
	.usage = "<interface name>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_vap_extstats,
	.cmdname = "get_vap_extstats",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_phy_stats,
	.cmdname = "get_phy_stats",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ap_isolate,
	.cmdname = "set_ap_isolate",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_ap_isolate,
	.cmdname = "get_ap_isolate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_pm_get_set_mode,
	.cmdname = "pm",
	.usage = "[dual_emac] [ {off | on | auto | suspend | idle} ]",
	.args = {0, 2},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_qpm_get_level,
	.cmdname = "qpm_level",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reset_all_counters,
	.cmdname = "reset_all_stats",
	.usage = "<WiFi interface> <node index> [<remote>]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_eth_phy_power_off,
	.cmdname = "eth_phy_power_off",
	.usage = "<network interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_aspm_l1,
	.cmdname = "set_aspm_l1",
	.usage = "{0 | 1} <latency>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_l1,
	.cmdname = "set_l1",
	.usage = "{0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_telnet_enable,
	.cmdname = "enable_telnet",
	.usage = "<value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_restore_default_config,
	.cmdname = "restore_default_config",
	.usage = qcsapi_usage_restore_default_config,
	.args = {0, -1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_run_script,
	.cmdname = "run_script",
	.usage = "<scriptname> [<param>...]",
	.args = {1, -1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_test_traffic,
	.cmdname = "test_traffic",
	.usage = "<WiFi interface> {start [<period>] | stop}",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_temperature,
	.cmdname = "get_temperature",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_accept_oui_filter,
	.cmdname = "set_accept_oui_filter",
	.usage = "<WiFi interface> <OUI> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_accept_oui_filter,
	.cmdname = "get_accept_oui_filter",
	.usage = "<WiFi interface> [<size>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_swfeat_list,
	.cmdname = "get_swfeat_list",
	.usage = "[<radio>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_vht,
	.cmdname = "set_vht",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_vht,
	.cmdname = "get_vht",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_he,
	.cmdname = "set_he",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_he,
	.cmdname = "get_he",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_set_test_mode,
	.cmdname = "set_test_mode",
	.usage = "calcmd <channel> <antenna> <MCS> <BW> <Packet size> <11n> <beamforming>",
	.args = {7, 7},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_show_test_packet,
	.cmdname = "show_test_packet",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_send_test_packet,
	.cmdname = "send_test_packet",
	.usage = "calcmd <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_stop_test_packet,
	.cmdname = "stop_test_packet",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_send_dc_cw_signal,
	.cmdname = "send_dc_cw_signal",
	.usage = "calcmd <channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_stop_dc_cw_signal,
	.cmdname = "stop_dc_cw_signal",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_test_mode_antenna_sel,
	.cmdname = "get_test_mode_antenna_sel",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_test_mode_mcs,
	.cmdname = "get_test_mode_mcs",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_test_mode_bw,
	.cmdname = "get_test_mode_bw",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_tx_power,
	.cmdname = "get_test_mode_tx_power",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_set_tx_power,
	.cmdname = "set_test_mode_tx_power",
	.usage = "<ifname> <tx_power>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_real_time_txpower,
	.cmdname = "get_real_time_txpower",
	.usage = "<ifname>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_test_mode_rssi,
	.cmdname = "get_test_mode_rssi",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_set_mac_filter,
	.cmdname = "calcmd_set_mac_filter",
	.usage = "calcmd <q_num> <sec_enable> <mac_addr>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_antenna_count,
	.cmdname = "get_test_mode_antenna_count",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_clear_counter,
	.cmdname = "calcmd_clear_counter",
	.usage = "calcmd",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_calcmd_get_info,
	.cmdname = "get_info",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_disable_dfs_channels,
	.cmdname = "disable_dfs_channels",
	.usage = "<WiFi interface> {0 | 1} [<new channel>]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dfs_channels_status,
	.cmdname = "get_dfs_channels_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_vlan_promisc,
	.cmdname = "enable_vlan_promisc",
	.usage = "<enable>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_add_multicast,
	.cmdname = "add_multicast",
	.usage = "<IP address> <MAC address>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_del_multicast,
	.cmdname = "del_multicast",
	.usage = "<IP address> <MAC address>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_multicast_list,
	.cmdname = "get_multicast_list",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_add_ipff,
	.cmdname = "add_ipff",
	.usage = "<ipaddr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_del_ipff,
	.cmdname = "del_ipff",
	.usage = "<ipaddr>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_ipff,
	.cmdname = "get_ipff",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_carrier_id,
	.cmdname = "get_carrier_id",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_carrier_id,
	.cmdname = "set_carrier_id",
	.usage = "<carrier_id> [<update_uboot>]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_platform_id,
	.cmdname = "get_platform_id",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_spinor_jedecid,
	.cmdname = "get_spinor_jedecid",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_custom_value,
	.cmdname = "get_custom_value",
	.usage = "<key>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_custom_value,
	.cmdname = "set_custom_value",
	.usage = "<key> <value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mlme_stats_per_mac,
	.cmdname = "get_mlme_stats_per_mac",
	.usage = "[<mac_addr>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mlme_stats_per_association,
	.cmdname = "get_mlme_stats_per_association",
	.usage = "<WiFi interface> <association index>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_index,
	},
	{
	.fn = call_qcsapi_wifi_get_mlme_stats_macs_list,
	.cmdname = "get_mlme_stats_macs_list",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_nss_cap,
	.cmdname = "get_nss_cap",
	.usage = "<WiFi interface> {ht | vht | he}",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_modulation,
	},
	{
	.fn = call_qcsapi_wifi_set_nss_cap,
	.cmdname = "set_nss_cap",
	.usage = "<WiFi interface> {ht | vht | he} <nss>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_modulation,
	},
	{
	.fn = call_qcsapi_wifi_get_security_defer_mode,
	.cmdname = "get_security_defer_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_security_defer_mode,
	.cmdname = "set_security_defer_mode",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_apply_security_config,
	.cmdname = "apply_security_config",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_board_parameter,
	.cmdname = "get_board_param",
	.cmdname_alt = "get_board_parameter",
	.usage = "<board parameter> [<radio>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_board_parameter,
	},
	{
	.fn = call_qcsapi_wifi_set_intra_bss_isolate,
	.cmdname = "set_intra_bss_isolate",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_intra_bss_isolate,
	.cmdname = "get_intra_bss_isolate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_bss_isolate,
	.cmdname = "set_bss_isolate",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bss_isolate,
	.cmdname = "get_bss_isolate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_host_state,
	.cmdname = "wowlan_host_state",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wowlan_match_type_set,
	.cmdname = "wowlan_match_type",
	.usage = "<WiFi interface> {0 | 1 | 2}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wowlan_l2_type_set,
	.cmdname = "wowlan_L2_type",
	.usage = "<WiFi interface> <ether type>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wowlan_udp_port_set,
	.cmdname = "wowlan_udp_port",
	.usage = "<WiFi interface> <udp dest port>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wowlan_set_magic_pattern,
	.cmdname = "wowlan_pattern",
	.usage = "<WiFi interface> <pattern>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wowlan_get_host_state,
	.cmdname = "wowlan_get_host_state",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wowlan_get_match_type,
	.cmdname = "wowlan_get_match_type",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wowlan_get_l2_type,
	.cmdname = "wowlan_get_L2_type",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wowlan_get_udp_port,
	.cmdname = "wowlan_get_udp_port",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wowlan_get_magic_pattern,
	.cmdname = "wowlan_get_pattern",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_extender_params,
	.cmdname = "set_extender_params",
	.usage = "<WiFi interface> <parameter type> <parameter value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_extender_params,
	},
	{
	.fn = call_qcsapi_wifi_get_extender_params,
	.cmdname = "get_extender_params",
	.cmdname_alt = "get_extender_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_enable_bgscan,
	.cmdname = "enable_bgscan",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bgscan_status,
	.cmdname = "get_bgscan_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_uboot_info,
	.cmdname = "get_uboot_info",
	.usage = "<uboot info type> (0 - ver, 1 - built, 2 - type, 3 - all), [<uboot image name>]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_disassoc_reason,
	.cmdname = "disassoc_reason",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_is_startprod_done,
	.cmdname = "is_startprod_done",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_amsdu,
	.cmdname = "get_tx_amsdu",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_tx_amsdu,
	.cmdname = "set_tx_amsdu",
	.usage = "<WiFi interface> { 0 | 1 | <AC BE> <AC BK> <AC VI> <AC VO> }",
	.args = {1, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scan_buf_max_size,
	.cmdname = "set_scan_buf_max_size",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scan_buf_max_size,
	.cmdname = "get_scan_buf_max_size",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scan_table_max_len,
	.cmdname = "set_scan_table_max_len",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_scan_table_max_len,
	.cmdname = "get_scan_table_max_len",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_enable_mu,
	.cmdname = "set_enable_mu",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_enable_mu,
	.cmdname = "get_enable_mu",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mu_use_eq,
	.cmdname = "set_mu_use_eq",
	.usage = "<WiFi interface> <enable>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mu_use_eq,
	.cmdname = "get_mu_use_eq",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mu_groups,
	.cmdname = "get_mu_groups",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_emac_switch,
	.cmdname = "set_emac_switch",
	.usage = "{0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_emac_switch,
	.cmdname = "get_emac_switch",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_verify_repeater_mode,
	.cmdname = "verify_repeater_mode",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ap_interface_name,
	.cmdname = "set_ap_interface_name",
	.usage = "<interface name>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_ap_interface_name,
	.cmdname = "get_ap_interface_name",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_optim_stats,
	.cmdname = "set_optim_stats",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_sys_time,
	.cmdname = "set_sys_time",
	.usage = "<seconds since epoch>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_sys_time,
	.cmdname = "get_sys_time",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_eth_info,
	.cmdname = "get_eth_info",
	.usage = "<Ethernet interface> [{link | speed | duplex | autoneg}]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_block_bss,
	.cmdname = "block_bss",
	.usage = "<WiFi interface> <flag>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_txba_disable,
	.cmdname = "txba_disable",
	.usage = "<WiFi interface> <disable>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_txba_disable,
	.cmdname = "get_txba_disable",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_rxba_decline,
	.cmdname = "rxba_decline",
	.usage = "<WiFi interface> <decline>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_rxba_decline,
	.cmdname = "get_rxba_decline",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_sec_chan,
	.cmdname = "get_sec_chan",
	.usage = "<WiFi interface> <chan>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_sec_chan,
	.cmdname = "set_sec_chan",
	.usage = "<WiFi interface> <chan> <offset>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_vap_default_state,
	.cmdname = "set_vap_default_state",
	.usage = "<radio> <enable>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_vap_default_state,
	.cmdname = "get_vap_default_state",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_airtime,
	.cmdname = "get_tx_airtime",
	.usage = "<interface name> {<node index> | all} [{start | stop}]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_qwe_command,
	.cmdname = "qwe",
	.usage = "<command> [<param1>] [<param2>] [<param3>]",
	.args = {1, 4},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_core_dump2,
	.cmdname = "get_core_dump",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_gather_info,
	.cmdname = "gather_info",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_sysrpt,
	.cmdname = "get_sysrpt",
	.usage = "<report type> [force]",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_client_mac_list,
	.cmdname = "get_client_mac_list",
	.usage = "<WiFi interface> <index>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_sample_all_clients,
	.cmdname = "sample_all_clients",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_per_assoc_data,
	.cmdname = "get_sampled_assoc_data",
	.usage = "<WiFi interface> <num entry> <offset>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_wifi_ready,
	.cmdname = "is_wifi_ready",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_cca_stats,
	.cmdname = "get_cca_stats",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_rf_chains,
	.cmdname = "set_rf_chains",
	.usage = "<Tx 5GHz> <Rx 5GHz> <Tx 2.4GHz> <Rx 2.4GHz> [ <Tx 5GHz 2> ] [ <Rx 5GHz 2> ]",
	.args = {4, 6},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_rf_chains,
	.cmdname = "get_rf_chains",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_tx_chains,
	.cmdname = "get_tx_chains",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_rx_chains,
	.cmdname = "get_rx_chains",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_vapdebug,
	.cmdname = "set_vapdebug",
	.usage = "<WiFi interface> <bitmap>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_vapdebug,
	.cmdname = "get_vapdebug",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_eap_reauth_period,
	.cmdname = "set_eap_reauth_period",
	.usage = "<WiFi interface> <eap_reauth_period>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_eap_reauth_period,
	.cmdname = "get_eap_reauth_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_eap_reauth_period,
	.cmdname = "remove_eap_reauth_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_eap_param,
	.cmdname = "set_eap_param",
	.usage = "<WiFi interface> <EAP param name> <EAP param value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_eap_params,
	},
	{
	.fn = call_qcsapi_wifi_get_eap_param,
	.cmdname = "get_eap_param",
	.usage = "<WiFi interface> <EAP param name>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_eap_params,
	},
	{
	.fn = call_qcsapi_wifi_remove_eap_param,
	.cmdname = "remove_eap_param",
	.usage = "<WiFi interface> <EAP param name>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_eap_params,
	},
	{
	.fn = call_qcsapi_wifi_set_cs_thrshld_range,
	.cmdname = "set_cs_thrshld_range",
	.usage = "<WiFi interface> <min> <max>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_cs_thrshld_range,
	.cmdname = "get_cs_thrshld_range",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_cs_thrshld_inuse,
	.cmdname = "set_cs_thrshld_inuse",
	.usage = "<WiFi interface> <inuse>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_cs_thrshld_inuse,
	.cmdname = "get_cs_thrshld_inuse",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_igmp_snooping_state,
	.cmdname = "get_igmp_snooping_state",
	.usage = "<bridge interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_igmp_snooping_state,
	.cmdname = "set_igmp_snooping_state",
	.usage = "<bridge interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_br_get_groups,
	.cmdname = "get_br_groups",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_br_get_interfaces,
	.cmdname = "get_br_interfaces",
	.usage = "<bridge interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bsa_get_parameter,
	.cmdname = "get_bsa_param",
	.usage = "<param>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bsa_set_parameter,
	.cmdname = "set_bsa_param",
	.usage = "[{<param> <value1> [<value2>] | --help}]",
	.args = {1, 3},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bsa_get_parameter_ext,
	.cmdname = "get_bsa_param_ext",
	.usage = "{<mobility domain> <param> | --help}",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bsa_set_parameter_ext,
	.cmdname = "set_bsa_param_ext",
	.usage = "<mobility domain> {<param> [<value1> [<value2>]] | --help}",
	.args = {1, 4},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_radius_max_retries,
	.cmdname = "set_radius_max_retries",
	.usage = "<WiFi interface> <max_retries>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radius_max_retries,
	.cmdname = "get_radius_max_retries",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_radius_max_retries,
	.cmdname = "remove_radius_max_retries",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_radius_num_failover,
	.cmdname = "set_radius_num_failover",
	.usage = "<WiFi interface> <failover_num>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radius_num_failover,
	.cmdname = "get_radius_num_failover",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_radius_num_failover,
	.cmdname = "remove_radius_num_failover",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_radius_timeout,
	.cmdname = "set_radius_timeout",
	.usage = "<WiFi interface> <timeout>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_radius_timeout,
	.cmdname = "get_radius_timeout",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_radius_timeout,
	.cmdname = "remove_radius_timeout",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pmk_cache_enable,
	.cmdname = "set_pmk_cache_enable",
	.usage = "<WiFi interface> <enable_val>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pmk_cache_enable,
	.cmdname = "get_pmk_cache_enable",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pmk_cache_lifetime,
	.cmdname = "set_pmk_cache_lifetime",
	.usage = "<WiFi interface> <lifetime>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pmk_cache_lifetime,
	.cmdname = "get_pmk_cache_lifetime",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_pmk_cache_lifetime,
	.cmdname = "remove_pmk_cache_lifetime",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_max_auth_attempts,
	.cmdname = "set_max_auth_attempts",
	.usage = "<WiFi interface> <max_attempts>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_max_auth_attempts,
	.cmdname = "get_max_auth_attempts",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_lockout_period,
	.cmdname = "set_lockout_period",
	.usage = "<WiFi interface> <period>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_lockout_period,
	.cmdname = "get_lockout_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_lockout_period,
	.cmdname = "remove_lockout_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_id_request_period,
	.cmdname = "set_id_request_period",
	.usage = "<WiFi interface> <period>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_id_request_period,
	.cmdname = "get_id_request_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_id_request_period,
	.cmdname = "remove_id_request_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_auth_quiet_period,
	.cmdname = "set_auth_quiet_period",
	.usage = "<WiFi interface> <period>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_auth_quiet_period,
	.cmdname = "get_auth_quiet_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_remove_auth_quiet_period,
	.cmdname = "remove_auth_quiet_period",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_set_api_without_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wlmonitor_enable,
	.cmdname = "enable_link_monitor",
	.usage = "<WiFi interface> <mac address> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wlmonitor_rate_thres,
	.cmdname = "set_link_monitor_rate_thres",
	.usage = "<WiFi interface> <severity> <thres>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_wlmonitor_period_thres,
	.cmdname = "set_link_monitor_duration",
	.usage = "<WiFi interface> <severity> <unit> <thres>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_beacon_phyrate,
	.cmdname = "get_beacon_phyrate",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_beacon_phyrate,
	.cmdname = "set_beacon_phyrate",
	.usage = "<WiFi interface> <PHY rate>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_beacon_power_backoff,
	.cmdname = "get_beacon_power_backoff",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_beacon_power_backoff,
	.cmdname = "set_beacon_power_backoff",
	.usage = "<WiFi interface> <backoff>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_mgmt_power_backoff,
	.cmdname = "get_mgmt_power_backoff",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_mgmt_power_backoff,
	.cmdname = "set_mgmt_power_backoff",
	.usage = "<WiFi interface> <backoff>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_qdrv_set_hw_module_state,
	.cmdname = "set_hw_module_state",
	.usage = "<WiFi interface> <module> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_hw_module,
	},
	{
	.fn = call_qcsapi_qdrv_get_hw_module_state,
	.cmdname = "get_hw_module_state",
	.usage = "<WiFi interface> <module>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_hw_module,
	},
	{
	.fn = call_qcsapi_wifi_start_dcs_scan,
	.cmdname = "start_dcs_scan",
	.usage = qcsapi_usage_start_dcs_scan,
	.args = {0, -1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_stop_dcs_scan,
	.cmdname = "stop_dcs_scan",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_dcs_scan_params,
	.cmdname = "get_dcs_scan_params",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_vlan_loop_detect,
	.cmdname = "set_vlan_loop_detect",
	.usage = "<type>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_vlan_loop_detect,
	.cmdname = "get_vlan_loop_detect",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_bsa_get_sta_table,
	.cmdname = "get_bsa_sta_table",
	.usage = "[{all | assoc}]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api_without_ifname_parameter,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_add_app_ie,
	.cmdname = "add_app_ie",
	.usage = "<WiFi interface> <frametype> <index> <ie>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_remove_app_ie,
	.cmdname = "remove_app_ie",
	.usage = "<WiFi interface> <frametype> <index>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_disable_11b,
	.cmdname = "disable_11b",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_nac_mon_mode,
	.cmdname = "get_nac_mon_mode",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_nac_mon_mode,
	.cmdname = "set_nac_mon_mode",
	.usage = "<WiFi interface> {disable | enable [ <period> [ <percentage on> ]]}",
	.args = {1, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_nac_stats,
	.cmdname = "get_nac_stats",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_update_bootcfg_binfile,
	.cmdname = "update_bootcfg_binfile",
	.usage = "<dst> <path_src>",
	.args = {2, 2},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_ieee80211r,
	.cmdname = "set_ieee80211r",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_ieee80211r,
	.cmdname = "get_ieee80211r",
	.usage = "<wifi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_11r_mobility_domain,
	.cmdname = "set_11r_mobility_domain",
	.usage = "<wifi interface> <mobility_domain_id>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_mobility_domain,
	.cmdname = "get_11r_mobility_domain",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_11r_nas_id,
	.cmdname = "set_11r_nas_id",
	.usage = "<WiFi interface> <nas_identifier>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_nas_id,
	.cmdname = "get_11r_nas_id",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_11r_ft_over_ds,
	.cmdname = "set_11r_ft_over_ds",
	.usage = "<wifi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_ft_over_ds,
	.cmdname = "get_11r_ft_over_ds",
	.usage = "<wifi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_add_11r_neighbour,
	.cmdname = "add_11r_neighbour",
	.usage = "<wifi interface> <mac address> <NAS Identifier> <128-bit hex key> <R1KH-ID>",
	.args = {4, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_del_11r_neighbour,
	.cmdname = "del_11r_neighbour",
	.usage = "<wifi interface> <mac address>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_neighbour,
	.cmdname = "get_11r_neighbour",
	.usage = "<wifi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_11r_r1_key_holder,
	.cmdname = "set_11r_r1_key_holder",
	.usage = "<wifi interface> <r1_key_holder>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_r1_key_holder,
	.cmdname = "get_11r_r1_key_holder",
	.usage = "<wifi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_11r_r0_key_lifetime,
	.cmdname = "set_11r_r0_key_lifetime",
	.usage = "<wifi interface> <r0_key_lifetime>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_11r_r0_key_lifetime,
	.cmdname = "get_11r_r0_key_lifetime",
	.usage = "<wifi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_max_boot_cac_duration,
	.cmdname = "set_max_boot_cac_duration",
	.usage = "<WiFi interface> <value>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_icac_status,
	.cmdname = "get_icac_status",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn,
	.cmdname = "set_scs_leavedfs_chan_mtrc_mrgn",
	.usage = "<WiFi interface> <leavedfs_chan_mtrc_mrgn>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_is_weather_channel,
	.cmdname = "is_weather_channel",
	.usage = "<WiFi interface> <channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_reboot_cause,
	.cmdname = "get_reboot_cause",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_radio_pwr_save,
	.cmdname = "set_radio_pwr_save",
	.usage = "<radio> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_put_radio_under_reset,
	.cmdname = "put_radio_under_reset",
	.usage = "<radio>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_pta_op_mode,
	.cmdname = "set_pta",
	.usage = "<WiFi interface> <pta_mode>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_pta,
	.cmdname = "get_pta",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_show_access_points,
	.cmdname = "show_access_points",
	.usage = "<WiFi interface> {0 | 1}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reg_chan_txpower_backoff_set,
	.cmdname = "set_chan_txpower_backoff",
	.usage = "<WiFi interface> <channel> <is_percentage> <backoff>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_reg_chan_txpower_backoff_get,
	.cmdname = "get_chan_txpower_backoff",
	.usage = "<WiFi interface> <channel>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_grab_config,
	.cmdname = "grab_config",
	.usage = "<output file>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_repeater_mode_cfg,
	.cmdname = "repeater_mode_cfg",
	.usage = "<radio> {0 | 1}",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_urepeater_params,
	.cmdname = "set_urepeater_params",
	.usage = "<parameter type> <parameter value>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_urepeater_params,
	.cmdname = "get_urepeater_params",
	.usage = "<parameter type>",
	.args = {1, 1},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_save_config,
	.cmdname = "save_config",
	.usage = "[<output dir>]",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_config_status,
	.cmdname = "get_config_status",
	.usage = "",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_dpp_parameter,
	.cmdname = "dpp_param",
	.usage = qcsapi_usage_dpp_config,
	.args = {0, 16},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_dpp_parameter,
	},
	{
	.fn = call_qcsapi_wifi_aacs_thres_min_tbl_set,
	.cmdname = "aacs_thres_min_tbl",
	.usage = "<WiFi interface> <v0> <v1> <v2> <v3> <v4> <v5>",
	.args = {6, 6},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_thres_min_tbl_get,
	.cmdname = "aacs_get_thres_min_tbl",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_thres_max_tbl_set,
	.cmdname = "aacs_thres_max_tbl",
	.usage = "<WiFi interface> <v0> <v1> <v2> <v3> <v4> <v5>",
	.args = {6, 6},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_thres_max_tbl_get,
	.cmdname = "aacs_get_thres_max_tbl",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_rssi_tbl_set,
	.cmdname = "aacs_vnode_rssi_tbl",
	.usage = "<WiFi interface> <v0> <v1> <v2> <v3>",
	.args = {4, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_rssi_tbl_get,
	.cmdname = "aacs_get_vnode_rssi_table",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_wgt_tbl_set,
	.cmdname = "aacs_vnode_wgt_tbl",
	.usage = "<WiFi interface> <v0> <v1> <v2> <v3>",
	.args = {4, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_wgt_tbl_get,
	.cmdname = "aacs_get_vnode_wgt_tbl",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_set,
	.cmdname = "aacs_vnode",
	.usage = "<WiFi interface> <rssi> <weight> <index>",
	.args = {3, 3},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_vnode_get,
	.cmdname = "aacs_get_vnodes",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set,
	.cmdname = "aacs_dfs_thres_adj_tbl",
	.usage = "<WiFi interface> <v0> <v1> <v2> <v3> <v4> <v5>",
	.args = {6, 6},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get,
	.cmdname = "aacs_get_dfs_thres_adj_tbl",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_excl_ch_set,
	.cmdname = "aacs_excl_chan",
	.usage = "<WiFi interface> <chan start> [<chan end>]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_excl_ch_get,
	.cmdname = "aacs_get_excl_chan",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_alt_excl_ch_set,
	.cmdname = "aacs_alt_excl_chan",
	.usage = "<WiFi interface> <chan start> [<chan end>]",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_alt_excl_ch_get,
	.cmdname = "aacs_get_alt_excl_chan",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_sel_ch_set,
	.cmdname = "aacs_set_sel_chan",
	.usage = "<WiFi interface> {add | del | alt_add | alt_del} <ch1> [<ch2>... <ch10>]",
	.args = {2, 11},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_aacs_sel_ch_get,
	.cmdname = "aacs_get_sel_chan",
	.usage = "<WiFi interface> {get | alt_get}",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_legacy_bbic,
	.cmdname = "set_legacy_bbic",
	.usage = "<WiFi interface> <val>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_legacy_bbic,
	.cmdname = "get_legacy_bbic",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_bss_cfg,
	.cmdname = "get_bss_cfg",
	.usage = "<WiFi interface> {ap <WiFi interface> | sta [<SSID fmt>] <ssid>} <param_name>",
	.args = {3, 4},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_6g_min_rate,
	.cmdname = "set_6g_min_rate",
	.usage = "<WiFi interface> <NSS> <MCS>",
	.args = {2, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_set_3addr_br_config,
	.cmdname = "set_3addr_br_config",
	.usage = "<radio> dhcp_chaddr { 0 | 1 }",
	.args = {3, 3},
	.call_type = e_qcsapi_set_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_3addr_br_config,
	.cmdname = "get_3addr_br_config",
	.usage = "<radio> dhcp_chaddr",
	.args = {2, 2},
	.call_type = e_qcsapi_get_system_value,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_gpio_config,
	.cmdname = "gpio",
	.usage = qcsapi_usage_gpio,
	.args = {2, 3},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_freq2chan_and_band,
	.cmdname = "freq2chan_and_band",
	.usage = "<freq>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api_without_ifname,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_multi_psk_info_append,
	.cmdname = "multi_psk_info_append",
	.usage = "<WiFi interface> <string>",
	.args = {1, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_multi_psk_info_read,
	.cmdname = "multi_psk_info_read",
	.usage = "<WiFi interface>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_multi_psk_info_replace,
	.cmdname = "multi_psk_info_replace",
	.usage = "<WiFi interface> [<string>]",
	.args = {0, 1},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_start_phy_scan,
	.cmdname = "start_phy_scan",
	.usage = "<WiFi interface> [ bw <bw> ] [ freqs <freqs> ]",
	.args = {0, 4},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_xcac_set,
	.cmdname = "xcac_set",
	.usage = "<WiFi interface> {start <chan> <bw> <method> <action> | stop}",
	.args = {1, 5},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_xcac_get,
	.cmdname = "xcac_get",
	.usage = "<WiFi interface> status",
	.args = {1, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_get_chan_phy_info,
	.cmdname = "get_chan_phy_info",
	.usage = "<WiFi interface> [-saved] {<channel list> | all}",
	.args = {1, 2},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_get_chan_avail_status_by_bw,
	.cmdname = "get_chan_avail_stats_by_bw",
	.usage = "<WiFi interface> {<bw>}",
	.args = {0, 1},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_none,
	},
	{
	.fn = call_qcsapi_wifi_set_zsdfs_param,
	.cmdname = "set_zsdfs_param",
	.usage = "<WiFi interface> <parameter name> <value1> {<value2>}",
	.args = {1, 2},
	.call_type = e_qcsapi_set_api,
	.generic_param_type = e_qcsapi_zsdfs_parameter,
	},
	{
	.fn = call_qcsapi_wifi_get_zsdfs_param,
	.cmdname = "get_zsdfs_param",
	.usage = "<WiFi interface> <parameter name>",
	.args = {0, 0},
	.call_type = e_qcsapi_get_api,
	.generic_param_type = e_qcsapi_zsdfs_parameter,
	},
};

static const struct call_table_entry_s *call_qcsapi_find_entry_point(char *name)
{
	int i;
	static const struct call_table_entry_s *entry;

	for (i = 0; i < ARRAY_SIZE(call_table_entry); i++) {
		entry = &call_table_entry[i];
		if (strcasecmp(entry->cmdname, name) == 0)
			return entry;
		if (entry->cmdname_alt && strcasecmp(entry->cmdname_alt, name) == 0)
			return entry;
	}

	return NULL;
}

static void call_qcsapi_list_entry_point_names(qcsapi_output *print)
{
	unsigned int iter;

	print_out(print, "API entry point names\n");

	for (iter = 0; iter < ARRAY_SIZE(call_table_entry); iter++)
		print_out(print, "  %s\n", call_table_entry[iter].cmdname);
}

static void call_qcsapi_print_cmd(call_bundle *cb, int argc, char *argv[])
{
	qcsapi_output *print = cb->output;
	const struct call_table_entry_s *call_entry = cb->call_entry;
	int i;

	print_out(print, "Call QCSAPI: %s",
			call_entry->cmdname);

	if (call_entry->call_type == e_qcsapi_get_api ||
			call_entry->call_type == e_qcsapi_set_api ||
			call_entry->call_type == e_qcsapi_set_api_without_parameter) {
		print_out(print, " %s", cb->interface);
	}

	for (i = 0; i < argc; i++)
		print_out(print, " %s", argv[i]);

	print_out(print, "\n");
}

static void call_qcsapi_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "  call_qcsapi [<option> ...] <entry point> [<param> ...]\n");
	print_out(print, "  call_qcsapi -h [<type>]\n");
	print_out(print, "Options:\n");
	print_out(print, "  -?            Display this help\n");
	print_out(print, "  -v            Verbose mode\n");
	print_out(print, "  -q            Quiet mode\n");
	print_out(print, "  -n <iter>     Repeat command <iter> times\n");
	print_out(print, "  -d <secs>     Delay <secs> seconds between iterations\n");
	print_out(print, "  -u            Disable locking - for internal use only\n");
	print_out(print, "  -h [<type>]   Show more specific help - use '?' for types\n");
	print_out(print, "                Default is to list the available entry points\n");
}

static int local_check_args(call_bundle *cb, int argc, int expected_argc)
{
	qcsapi_output *print = cb->output;
	const struct call_table_entry_s *call_entry = cb->call_entry;

	if (argc < call_entry->args.min ||
			(call_entry->args.max >= 0 && argc > call_entry->args.max)) {
		if (verbose_flag > 0)
			print_err(print, "Args: min %d max %d entered %d\n",
				call_entry->args.min + expected_argc,
				call_entry->args.max + expected_argc,
				argc + expected_argc);
		qcsapi_report_usage(cb);
		return -EINVAL;
	}

	return 0;
}

static int call_qcsapi(call_bundle *cb, qcsapi_output *print, int argc, char *argv[])
{
	int retval = 0;
	int expected_argc = 0;
	const struct call_table_entry_s *call_entry = NULL;

	if (argc < 1) {
		call_qcsapi_usage(print);
		return -EINVAL;
	}

	call_entry = call_qcsapi_find_entry_point(argv[0]);
	if (!call_entry) {
		print_out(print, "QCSAPI entry point %s not found\n", argv[0]);
		return -EINVAL;
	}
	cb->call_entry = call_entry;
	argc--;
	argv++;

	if (call_entry->call_type == e_qcsapi_get_api ||
			call_entry->call_type == e_qcsapi_set_api ||
			call_entry->call_type == e_qcsapi_set_api_without_parameter) {
		expected_argc++;
		if (argc > 0)
			cb->interface = argv[0];
		/* Skip over the interface name */
		argc--;
		argv++;
	}

	if (verbose_flag > 0)
		call_qcsapi_print_cmd(cb, argc, argv);

	if (call_entry->generic_param_type != e_qcsapi_none) {
		expected_argc++;
		if (argc > 0) {
			if (parse_generic_parameter_name(cb, argv[0], &cb->generic_param) < 0)
				return -EINVAL;
		}
		/* Skip over the generic parameter */
		argc--;
		argv++;
	}

	if (local_check_args(cb, argc, expected_argc) < 0)
		return -EINVAL;

	while (cb->init_cnt-- > 0) {
		retval = qcsapi_init();
		if (retval < 0)
			return retval;
	}

	while (cb->call_cnt-- > 0) {
		retval = call_entry->fn(cb, cb->output, cb->interface, argc, argv);
		if (retval < 0)
			return retval;
		if (cb->call_cnt && cb->delay_time > 0)
			sleep(cb->delay_time);
	}

	return retval;
}

static void process_option_help(call_bundle *cb, qcsapi_output *print, int argc, char **argv, int i)
{
	char *help_option;
	const char *usage =
		"call_qcsapi -h <type>\n"
		"<type> is\n"
		"  entry_points     list all entry points (default)\n"
		"  <entry point>    show usage for a specific entry point\n"
		"  options          list list params for get_option, set_option\n"
		"  counters         list params for get_counter, get_node_counter, get_pm_counter\n"
		"  get_node_params  list params for get_node_param\n"
		"  board_params     list params for get_board_param\n"
		"  wifi_params      list params for get_wifi_param, set_wifi_param\n";

	i++;
	if (i >= argc) {
		call_qcsapi_list_entry_point_names(print);
		return;
	}

	help_option = argv[i];

	if (strcasecmp(help_option, "entry_points") == 0) {
		call_qcsapi_list_entry_point_names(print);
	} else if (strcasecmp(help_option, "options") == 0) {
		call_qcsapi_param_name_list(print, &qcsapi_param_name_option);
	} else if (strcasecmp(help_option, "counters") == 0) {
		call_qcsapi_param_name_list(print, &qcsapi_param_name_counter);
	} else if (strcasecmp(help_option, "get_node_params") == 0) {
		call_qcsapi_param_name_list(print, &qcsapi_param_name_per_node_param);
	} else if (strcasecmp(help_option, "board_params") == 0) {
		call_qcsapi_param_name_list(print, &qcsapi_param_name_board_param);
	} else if (strcasecmp(help_option, "wifi_params") == 0) {
		call_qcsapi_param_name_list(print, &qcsapi_param_name_wifi_param);
	} else {
		cb->call_entry = call_qcsapi_find_entry_point(help_option);
		if (!cb->call_entry) {
			print_out(print, usage);
			return;
		}
		qcsapi_report_usage(cb);
	}
}

static int process_option(call_bundle *cb, qcsapi_output *print, int argc, char **argv, int *i)
{
	char *option_arg = argv[*i];
	char option;
	unsigned int len = strlen(option_arg);
	unsigned int j = 1;
	uint32_t k;
	int min_value;
	char *local_addr;

	if (len == 1) {
		print_err(print, "Invalid option %s\n", option_arg);
		return -EINVAL;
	}

	option = option_arg[1];

	switch (option) {
	case 'v':
		while (option_arg[j] == 'v') {
			verbose_flag++;
			j++;
		}
		break;
	case 'q':
		verbose_flag = -1;
		break;
	case 'n':
	case 'd':
	case 'i':
		/* These options require an integer argument */
		if (len > 2) {
			local_addr = option_arg + 2;
		} else {
			(*i)++;
			if (*i >= argc) {
				print_err(print, "Missing value for %s\n", option_arg);
				return -EINVAL;
			}
			local_addr = argv[*i];
		}
		if (option == 'i')
			min_value = 0;
		else
			min_value = 1;
		if (local_atou32("value", local_addr, &k, print) < 0 || k < min_value)
			return -EINVAL;
		if (option == 'n')
			cb->call_cnt = k;
		else if (option == 'i')
			cb->init_cnt = k;
		else
			cb->delay_time = k;
		break;
	case 'u':
		qcsapi_sem_disable();
		break;
	case 'h':
		process_option_help(cb, print, argc, argv, *i);
		return -EAGAIN;	/* stop processing */
	default:
		call_qcsapi_usage(print);
		return -EAGAIN;	/* stop processing */
	}

	return 0;
}

static int process_options(call_bundle *cb, qcsapi_output *print, int argc, char **argv)
{
	int retval = 0;
	int i = 0;

	while (i < argc && *(argv[i]) == '-') {
		retval = process_option(cb, print, argc, argv, &i);
		if (retval < 0)
			return retval;
		i++;
	}

	return i;
}

/*
 * Returns 0 on success or 1 on failure.
 */
int qcsapi_main(qcsapi_output *print, int argc, char **argv)
{
	int retval;
	int ival;
	call_bundle cb;

	if (argc <= 1) {
		call_qcsapi_usage(print);
		return 1;
	}

	memset(&cb, 0, sizeof(cb));
	cb.output = print;
	cb.init_cnt = 1;
	cb.call_cnt = 1;

	argc--;
	argv++;

	ival = process_options(&cb, print, argc, argv);
	if (ival < 0) {
		if (ival == -EAGAIN)	/* Help command invoked */
			return 0;
		return 1;
	}

	argc = argc - ival;
	argv += ival;

	retval = call_qcsapi(&cb, print, argc, argv);
	if (retval < 0) {
		if (verbose_flag > 0)
			print_err(print, "Call QCSAPI failed with retval %d\n", retval);
		return 1;
	}

	return 0;
}
