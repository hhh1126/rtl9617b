// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019 Quantenna Communications Inc
 * All Rights Reserved

 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IEEE80211_AACS_SYS_H_
#define _IEEE80211_AACS_SYS_H_

#ifndef BIT
#define BIT(pos) (1 << (pos))
#endif

#define AACSLOG_CRIT				0
#define AACSLOG_WARNING				1
#define AACSLOG_NOTICE				2
#define AACSLOG_INFO				3
#define AACSLOG_VERBOSE				4
#define AACSLOG_EXTRA_VERBOSE			5
#define AACSLOG_LEVEL_MAX			5
#define AACSDBG_SET(x) aacs->aacs_params.debug_verbosity = (x)
#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
#ifdef AACS_UNIT_TEST
#define _AACSDBG(_prefix, id, _level, _fmt, ...)            do {               \
		if (aacs->aacs_params.debug_verbosity >= (_level)) {  \
			DBGFN(_prefix _fmt, id, ##__VA_ARGS__);     \
		}                                               \
	} while (0)
#else
#define _AACSDBG(_prefix, id, _level, _fmt, ...)            do {               \
		if (aacs->aacs_params.debug_verbosity >= (_level)) {  \
			DBGFN(KERN_CONT _prefix _fmt, id, ##__VA_ARGS__);     \
		}                                               \
	} while (0)
#endif
#define AACSDBG(_level, _fmt, ...) \
	_AACSDBG("AACS[%s]: ", aacs->aacs_identifier_str, _level, _fmt, ##__VA_ARGS__)
#define AACSDBG_NO_PREFIX(_level, _fmt, ...)	_AACSDBG("%s", "", _level, _fmt, ##__VA_ARGS__)
#define AACSDBG_ONCE(_fmt, ...) \
	DBGFN_ONCE("AACS[%s]: " _fmt, aacs->aacs_identifier_str, ##__VA_ARGS__)
#define AACSDBG_START(x) do { \
		int __tmpval_verbosity = aacs->aacs_params.debug_verbosity; \
		aacs->aacs_params.debug_verbosity = (x)
#define AACSDBG_STOP() \
		aacs->aacs_params_ctx.aacs_params.debug_verbosity = __tmpval_verbosity; \
	} while (0)
#endif

#define IEEE80211_AACS_BW_MAX 4
#define IEEE80211_AACS_HYSTERESIS_MAX 32
#define IEEE80211_AACS_INVALID_TX_POWER 0xFFFF
#define IEEE80211_AACS_BW_20 0
#define IEEE80211_AACS_BW_40 1
#define IEEE80211_AACS_BW_80 2
#define IEEE80211_AACS_BW_160 3

#define AACS_EXCLUDE_M					0x1
#define AACS_EXCLUDE_CHBW_S				0
#define AACS_EXCLUDE_BWLIMIT_S				1
#define AACS_EXCLUDE_NOT_USER_SEL_S			2
#define AACS_EXCLUDE_DISALLOW_PRI_S			3
#define AACS_EXCLUDE_DFS_S				4
#define AACS_EXCLUDE_CHANNEL_S				5
#define AACS_EXCLUDE_AVAIL_S				6
#define AACS_EXCLUDE_NOT_ACTIVE_S			16
#define AACS_EXCLUDE_PRIMARY_INACTIVE_S			17
#define AACS_EXCLUDE_INTERNAL_EXCLUDE_S			18
#define AACS_EXCLUDE_NOT_AVAILABLE_S			19
#define AACS_EXCLUDE_WEATHER_NOT_ALLOWED_S		20
#define AACS_EXCLUDE_HT_FORTY_NOT_ALLOWED_S		21
#define AACS_EXCLUDE_CURRENT_CHANNEL_NOT_ALLOWED_S	22
#define AACS_EXCLUDE_RADAR_DETECTED_S			23
#define AACS_EXCLUDE_5G_OBSS_NOT_CONFORM		24
#define AACS_EXCLUDE_OVERLAPPING_CHANNEL_S		25
#define AACS_EXCLUDE_CLIENT_INTFED			26
#define AACS_EXCLUDE_6G_NON_PSC				27
#define AACS_EXCLUDE_MAX				32

#define AACS_UINT16_MIN				0
#define AACS_UINT16_MAX				65535

#define AACS_UINT8_BIT_SIZE			8
#define AACS_UINT16_BIT_SIZE			16
#define AACS_UINT32_BIT_SIZE			32
#define AACS_UINT32_FULLMASK			0xFFFFFFFF

#define AACS_TYPE_VALUE				0
#define AACS_TYPE_BITMAP			1

#define AACS_INCL_CH_LIST_LEN			8

#define IEEE80211_AACS_CHANNEL_STATUS_AVAILABLE (0x2)
#define IEEE80211_AACS_CHANNEL_STATUS_NOT_AVAILABLE_RADAR_DETECTED (0x4)
#define IEEE80211_AACS_WCAC_L160M_U80M_END_CH 64

/* number of 5MHz bw in a 20MHz channel */
#define AACS_CHAN_BASE_BW 4

/* number of additional 5MHz bw for 2.4G legacy */
#define AACS_CHAN_24G_BW 1

/* num of 20MHz channels in BW */
#define IEEE80211_AACS_MAX_CHAN_IN_BW(bw) ((bw)/AACS_HT20)

#define AACS_VNODES_MAX 4
#define IEEE80211_AACS_STA_ALLOWED_DL_DEFICIENCY 80

enum aacs_command_type {
	AACS_DISABLE = 0,
	AACS_DISABLE_ALLOCED, /* Needed to pick a channel even though it's disabled */
	AACS_ENABLE_BACKGROUND,
	AACS_ENABLE_DECISION,
};

enum aacs_bw {
	AACS_BW_20M	= 0,
	AACS_BW_40M,
	AACS_BW_80M,
	AACS_BW_160M,
	AACS_BW_MMAX
};

enum aacs_alt_category {
	AACS_ALT_RDR_FULL = 0,
	AACS_ALT_RDR_U80 = 1,
	AACS_ALT_RDR_L80 = 2,
	AACS_ALT_RDR_MAX
};

enum {
	AACS_HT20 = 20,
	AACS_HT40 = 40,
	AACS_HT80 = 80,
	AACS_HT160 = 160,
};

enum {
	AACS_CHBW_CHG_ALT = 1,
	AACS_CHBW_CHG_CORE = 2
};

struct aacs_chan_bw {
	void *priv;
	struct ieee80211_aacs_ctx *aacs;
	/* Pointer to chan_bw where actually off-channel scan is performed */
	struct aacs_chan_bw *aacs_scan_chan_bw;
	int32_t		aacs_index;
	uint16_t	aacs_chan_num;
	uint8_t		aacs_bw;
	uint8_t		reserved1;
	uint32_t	aacs_cfreq; /* Center freq (MHz) */
	uint16_t	aacs_cchan; /* Center channel */
	uint16_t	aacs_sec80ll;
	uint16_t	aacs_lowest_chan_num;

	uint32_t	aacs_cca_try_cnt;
	uint32_t	aacs_cca_pri_cnt;
	uint32_t	aacs_cca_busy_cnt[AACS_BW_160M + 1];
	uint32_t	aacs_sp_errs;
	uint32_t	aacs_lp_errs;
	int32_t		aacs_hw_noise;

	/* For averaging */
	uint32_t	aacs_tot_cca_try_cnt;
	uint32_t	aacs_tot_cca_pri_cnt;
	uint32_t	aacs_tot_cca_busy_cnt[AACS_BW_160M + 1];
	uint32_t	aacs_tot_sp_errs;
	uint32_t	aacs_tot_lp_errs;
	int32_t		aacs_tot_hw_noise;
	int32_t		aacs_tot_ctr;

	uint32_t	aacs_beacon_recvd;
	uint32_t	aacs_beacon_recvd_band;
	uint8_t		aacs_is_ht_forty_allowed; /* Valid for 2.4G only */
	uint8_t		aacs_5g_conforms_obss; /* Valid for 5G only */

	int32_t		aacs_metric;
	int32_t		aacs_metric_buffer[IEEE80211_AACS_HYSTERESIS_MAX];
	int32_t		aacs_metric_buffer_idx;
	uint8_t		aacs_is_selectable;
	uint32_t	aacs_is_excluded;
	uint8_t		aacs_is_internal_excluded;
	uint8_t		aacs_is_overlapping;
	uint8_t		aacs_is_current_channel;
	uint8_t		aacs_alt_class;
	uint8_t		aacs_is_dfs;
	uint8_t		aacs_is_cac_required;
	uint8_t		aacs_is_radar_detected;
	uint32_t	aacs_alt_is_excluded;
	uint8_t		aacs_is_psc;

	int32_t		aacs_fat;
	int32_t		aacs_sp_lp_percnt;
	int32_t		aacs_tot_rate;
	uint32_t	aacs_scan_msecs;
	int32_t		aacs_better_ctr;
	int32_t		aacs_betterness_average;
	int32_t		aacs_betterness;

	int32_t		aacs_deficient_sta_cnt;

	uint8_t		aacs_is_new_scan;

	uint32_t	aacs_scan_history;
	uint32_t	aacs_scan_cnt;
	uint8_t		aacs_hyst_cnt;
	uint32_t	aacs_last_intf_client;
};

/*
 * Current channel bw struct that extends channel bw structure
 */
struct aacs_chan_bw_curr {
	struct aacs_chan_bw	base;
	uint32_t		aacs_tx_usecs;
	uint32_t		aacs_rx_usecs;
	uint32_t		aacs_try_usecs;
	uint32_t		aacs_cca_interf_cnt;
	int32_t			aacs_chan_decision_metric_est;
	int32_t			aacs_chan_decision_metric_act;
	int32_t			aacs_chan_dfs_decision_metric_est;
	int32_t			aacs_chan_dfs_decision_metric_act;
	int32_t			aacs_is_set;
};

struct ieee80211_aacs_channel_debug {
	int32_t aacs_dl_rate;
	int32_t aacs_ul_rate;
	int32_t aacs_raw_dl_rate;
	int32_t aacs_raw_ul_rate;
	int32_t aacs_est_dl_snr;
	int32_t aacs_noise_floor;
	int32_t aacs_sta_tx_power;
	int32_t aacs_est_ul_snr;
	int32_t aacs_ap_tx_power;
};

#define	AACS_ADDR_LEN 6

enum ieee80211_aacs_wifi_mode {
	AACS_WIFI_MODE_HT = 0,
	AACS_WIFI_MODE_VHT = 1,
	AACS_WIFI_MODE_HE = 2,
};

struct ieee80211_aacs_node {
	void		*priv;
	int8_t		macaddr[AACS_ADDR_LEN];
	int32_t		per;
	int32_t		rx_nss;
	int32_t		tx_nss;
	int32_t		max_node_bw_index;
	int32_t		samp_tx_mcs;
	int32_t		samp_tx_nss;
	int32_t		samp_tx_bw_index;
	int32_t		samp_rx_mcs;
	int32_t		samp_rx_nss;
	int32_t		samp_rx_bw_index;
	uint32_t	tx_phyrate;
	uint32_t	rx_phyrate;
	int32_t		last_rssi;
	uint8_t		is_virtual;
	uint8_t		wifi_mode;

	int32_t		path_loss;
	int32_t		path_loss1; /* Debug */
	int32_t		path_loss2; /* Debug */
	int32_t		sustained_snr;
	int32_t		weight; /* Percentage */
	uint8_t		is_alive;
	/*
	 * If this is not NULL, this has the same order and same size with
	 * struct ieee80211_aacs_ctx->aacs_chan_bw_list
	 */
	struct ieee80211_aacs_channel_debug *channel_list;
};

#define AACS_NCIDX_MAX 256
#define AACS_NODE_MAX (AACS_NCIDX_MAX + 4)

#define AACS_EXCL_CHAN_SIZE 2
#define IEEE80211_AACS_DECISION_FACTOR_SIZE 6

#define	AACS_CHECK_TYPE_STA_BITMASK	(1)
#define	AACS_CHECK_TYPE_RBS_BITMASK	(2)
#define	AACS_CHECK_TYPE_RPT_BITMASK	(4)

struct ieee80211_aacs {
	int16_t			aacs_enable;
	int16_t			debug_verbosity;
	int16_t			exclusion_table_raw;
	int16_t			start_bw;
	int16_t			start_chan;
	int16_t			dl_phy_rate_avg_weight;
	int16_t			dl_tput_avg_weight;
	int16_t			inclusion_spf;
	int16_t			inclusion_lpf;
	int16_t			threshold_table_enable;
	int16_t			bw_limit_control;
	int16_t			bw_limit;
	int16_t			decision_is_fast_mode;
	int16_t			aacs_switch_base_secs;
	int16_t			aacs_switch_factor;
	int16_t			aacs_switch_max_secs;
	int16_t			aacs_switch_reset_secs;
	int16_t			aacs_scan_success_rate_min;
	int16_t			aacs_decision_max_secs;
	int16_t			aacs_obss_check_enable;
	int16_t			apcnt_penalty;
	int16_t			aacs_sta_deficiency_enabled;
	int16_t			aacs_24g_exclude_overlapping_enable;
	int16_t			init_acs_assumed_sta_enable;
	uint16_t		aacs_sta_check_type_bitmask;
	int32_t			aacs_sta_allowed_dl_deficiency;
	uint32_t		aacs_chan_fade_msecs;
	struct aacs_vnode {
		int32_t		rssi;
		int32_t		weight;
		int32_t		index;
	}			init_acs_vnode[AACS_VNODES_MAX];
	struct ieee80211_aacs_decision_factor {
		int metric_min;
		int metric_max;
		int decision_factor_max;
		int decision_factor_min;
		int dfs_factor;
	} aacs_decision_factor_list[IEEE80211_AACS_DECISION_FACTOR_SIZE];
	int16_t			dfs_enable;
	int16_t			dfs_select_dfs_chan_at_init_enable;
	int16_t			dfs_select_dfs_chan_at_normal_enable;
	int16_t			dfs_select_unavail_chan_at_alt_enable;
	int16_t			enable_dfs_dec;

	struct aacs_chan_bw	*dfs_alt_chbw[AACS_ALT_RDR_MAX];

	int16_t			init_acs_enable;
	int16_t			init_acs_bw_limit_control;
	int16_t			init_acs_bw_limit;
	int32_t			excl_channel[AACS_EXCL_CHAN_SIZE];
	int32_t			sel_ch_list[AACS_INCL_CH_LIST_LEN];
	int16_t			alt_dfs_enable;
	int16_t			alt_bw_limit_control;
	int16_t			alt_bw_limit;
	int32_t			alt_excl_channel[AACS_EXCL_CHAN_SIZE];
	int32_t			alt_sel_ch_list[AACS_INCL_CH_LIST_LEN];
	int16_t			alt_prev_channel;
	int16_t			alt_seamless;
	int16_t			alt_dest_ieee_chan;
	int16_t			alt_dest_bw;
	int16_t			dfs_alt_cleared;
	uint16_t		chbw_chg_op;
	int16_t			alt_enable_160;
	uint16_t		enable_6g_non_psc;
	uint32_t		aacs_flags;
};

struct ieee80211_aacs_info {
	uint32_t	bw_sel;
	uint32_t	cca_idle;
	uint32_t	cca_curr_bw_busy;
	uint32_t	cca_busy_cnt[AACS_BW_160M + 1];
	uint32_t	cca_tx;
	uint32_t	cca_intf;
	uint32_t	cca_try;
	uint32_t	cca_try_cnt;
	uint32_t	cca_pri;
	uint32_t	cca_sec20;
	uint32_t	cca_sec40;
	uint32_t	bcn_rcvd;
	uint32_t	crc_err;
	uint32_t	lpre_err;
	uint32_t	spre_err;
	int32_t		hw_noise;
};

struct ieee80211_aacs_oc_info {
	uint32_t	off_channel;
	uint32_t	off_chan_bw_sel;
	uint32_t	off_chan_cca_curr_bw_busy;
	uint32_t	off_chan_cca_busy_cnt[AACS_BW_160M + 1];
	uint32_t	off_chan_cca_sample;
	uint32_t	off_chan_cca_sample_cnt;
	uint32_t	off_chan_cca_try;
	uint32_t	off_chan_cca_try_cnt;
	uint32_t	off_chan_beacon_recvd;
	uint32_t	off_chan_crc_errs;
	uint32_t	off_chan_sp_errs;
	uint32_t	off_chan_lp_errs;
	uint32_t	off_chan_cca_pri;
	uint32_t	off_chan_cca_pri_cnt;
	uint32_t	off_chan_cca_sec;
	uint32_t	off_chan_cca_sec40;
	int32_t		off_chan_hw_noise;
};

#define AACS_SCS_MAX_OC_INFO 32
struct ieee80211_aacs_cca_info {
	uint32_t	oc_info_count;
	struct ieee80211_aacs_oc_info oc_info[AACS_SCS_MAX_OC_INFO];
	uint32_t	bw_sel;
	uint32_t	cca_try;
	uint32_t	cca_try_cnt;
	uint32_t	cca_curr_bw_busy;
	uint32_t	cca_busy_cnt[AACS_BW_160M + 1];
	uint32_t	cca_sample_cnt;
	uint32_t	cca_idle;
	uint32_t	cca_tx;
	uint32_t	cca_interference;
	uint32_t	cca_interference_cnt;
	uint32_t	cca_pri;
	uint32_t	cca_pri_cnt;
	uint32_t	cca_sec20;
	uint32_t	cca_sec40;
	uint32_t	cca_pri_intf;
	uint32_t	cca_sec20_intf;
	uint32_t	cca_sec40_intf;
	uint32_t	beacon_recvd;
	uint32_t	tx_usecs;
	uint32_t	rx_usecs;
	int32_t		hw_noise;
};

/* This should be in sync with IEEE80211_CHAN_MAX_EXT*/
#define IEEE80211_AACS_CHAN_MAX		511


struct ieee80211_aacs_ctx {
	void *priv;
	struct aacs_chan_bw *aacs_chan_bw_list;
	struct aacs_chan_bw **aacs_chan_bw_sorted;
	struct aacs_chan_bw **aacs_chan_bw_alwayson_sorted;
	int32_t aacs_chan_bw_list_size;
	int32_t aacs_number_of_scan_channels;
	struct ieee80211_aacs_node *aacs_node_p_list[AACS_NODE_MAX];

	struct aacs_chan_bw_curr aacs_curr_chan_bw;
	struct aacs_chan_bw *aacs_last_scan_chan_bw;

	/* For faster access */
	uint16_t aacs_bw_offset[IEEE80211_AACS_BW_MAX];
	uint16_t aacs_chan_hash_map[IEEE80211_AACS_BW_MAX][IEEE80211_AACS_CHAN_MAX];

	/* Debug */
	uint8_t aacs_node_debug_enabled;
	uint8_t aacs_node_debug_cnt;
	char	aacs_identifier_str[16];

	int32_t aacs_switch_base_secs;
	int32_t aacs_switch_secs;
	int32_t aacs_switch_max_secs;
	int32_t aacs_switch_reset_secs;
	int32_t aacs_switch_factor;
	int32_t aacs_scan_success_rate_min; /* Percentage */
	int32_t aacs_scan_cnt_max;
	int32_t aacs_scan_cnt_min;
	int32_t aacs_decision_max_secs;
	int32_t aacs_spf_lpf_damping; /* Percentage */
	int32_t aacs_metric_period; /* Seconds */
	int32_t aacs_smthing_coeff;
	int32_t aacs_fat_smthing_coeff;
	int32_t aacs_rate_smthing_coeff;
	int32_t aacs_max_iot_tx_power;
	int32_t aacs_tx_power_add;
	int32_t aacs_antenna_gain;
	int32_t aacs_spf_duration; /* us */
	int32_t aacs_lpf_duration; /* us */
	int32_t aacs_nominal_per; /* Percentage */

	int32_t aacs_max_tx_power_all_channels;
	uint8_t aacs_is_ics; /* Initial Channel Selection is being done */
	uint8_t aacs_is_fast_mode;
	uint8_t aacs_has_real_stations;
	int32_t aacs_node_cnt;
	uint8_t aacs_is_nodes_changed;
	uint8_t aacs_is_waiting_for_channel_change;
	uint8_t aacs_reset_switch_time_latch;
	uint8_t aacs_wifi_mode;
	uint32_t aacs_channel_switch_type;
	uint32_t aacs_check_channel_counter;
	uint16_t aacs_sta_intf_settle_time;

	void	*aacs_decision_work;
	void	*aacs_stats_work;
	uint32_t last_channel_change_msecs;

	struct ieee80211_aacs aacs_params; /* AACS related params */
	uint8_t	aacs_mode;
	uint32_t aacs_override_bw;
	uint8_t aacs_init_channel_bootup;
	uint8_t aacs_config_complete;
};

/* for sub-commands that require vector I/O */
enum aacs_vcmd {
	AACS_VCMD_MIN = 0,
	AACS_VCMD_M_DEC_MIN,
	AACS_VCMD_M_DEC_MAX,
	AACS_VCMD_DFS_DEC,
	AACS_VCMD_VNODES,
	AACS_VCMD_STA_RSSI_TBL,
	AACS_VCMD_STA_WEIGHT_TBL,
	AACS_VCMD_EXCL_CHANNEL,
	AACS_VCMD_ALT_EXCL_CHANNEL,
	AACS_VCMD_SEL_CHANNEL,
	AACS_VCMD_DEL_CHANNEL,
	AACS_VCMD_SEL_ALT_CHANNEL,
	AACS_VCMD_DEL_ALT_CHANNEL,
	AACS_VCMD_TEST_UINT8,
	AACS_VCMD_TEST_UINT16,
	AACS_VCMD_TEST_UINT32,
	AACS_VCMD_MAX
};

enum {
	AACS_PICK_CHANNEL_ICAC,
	AACS_PICK_CHANNEL_ICS,
	AACS_PICK_CHANNEL_SET_CHANNEL
};

enum {
	AACS_FLAG_MIN			= 0,
	AACS_FLAG_RESET			= 0,
	AACS_FLAG_SWITCH_IFF_NO_STA	= 1,
	AACS_FLAG_DBG_REDUCED		= 24, /* Debug print only selectable channels */
	AACS_FLAG_MAX			= 32,
};

int ieee80211_aacs_start(struct ieee80211_aacs_ctx *aacs, enum aacs_command_type command);
void ieee80211_aacs_update_channels(struct ieee80211_aacs_ctx *aacs);
void ieee80211_aacs_update_scan_result(struct ieee80211_aacs_ctx *aacs, int chan_num, int success);
struct aacs_chan_bw *ieee80211_aacs_get_chan_bw(struct ieee80211_aacs_ctx *aacs, int chan, int bw);
void ieee80211_aacs_feed_initial_cca_data(struct ieee80211_aacs_ctx *aacs,
		struct ieee80211_aacs_info *aacs_info, int aacs_info_size);
struct aacs_chan_bw *ieee80211_aacs_pick_channel(struct ieee80211_aacs_ctx *aacs, int pick_type);

int ieee80211_aacs_param_print_exclusion_table(struct ieee80211_aacs_ctx *aacs, int raw);
int ieee80211_aacs_set_bw_control(struct ieee80211_aacs_ctx *aacs);
int ieee80211_aacs_param_exclude_overlapped(struct ieee80211_aacs_ctx *aacs);
int ieee80211_aacs_classify_alternates(struct ieee80211_aacs_ctx *aacs);
int ieee80211_aacs_pick_alternates(struct ieee80211_aacs_ctx *aacs);
void ieee80211_aacs_reinstall_virtual_node(struct ieee80211_aacs_ctx *aacs);
void ieee80211_aacs_alternates_list(struct ieee80211_aacs_ctx *aacs, int category_sorted);
void ieee80211_aacs_set_factor_table(struct ieee80211_aacs_ctx *aacs, uint32_t cmd,
		uint32_t *buf, uint32_t len);
void ieee80211_aacs_get_factor_table(struct ieee80211_aacs_ctx *aacs, uint32_t cmd,
		uint32_t *buf, uint32_t *len); /* AACS: Always-On DFS */
void ieee80211_aacs_update_virtual_node(struct ieee80211_aacs_ctx *aacs,
		uint32_t *v_int, int length);
int ieee80211_aacs_bw_index_to_bw(struct ieee80211_aacs_ctx *aacs, int bw_index);
int ieee80211_aacs_bw_to_bw_index(struct ieee80211_aacs_ctx *aacs, int bw);
int ieee80211_aacs_elapsed_time_since_scan(struct ieee80211_aacs_ctx *aacs,
		struct aacs_chan_bw *chan_bw);
int ieee80211_aacs_param_set_channel_select(struct ieee80211_aacs_ctx *aacs,
		uint32_t *vlist, uint32_t len, uint32_t op);
int ieee80211_aacs_param_get_channel_select(struct ieee80211_aacs_ctx *aacs,
		uint32_t *vlist, uint32_t *len, uint32_t op);

#if defined(CONFIG_DFS_MGMT_SUPPORT)
void ieee80211_aacs_decision_work_for_dfs_mgmt(struct ieee80211_aacs_ctx *aacs);
void ieee80211_aacs_dfs_mgmt_feedback_handler(struct ieee80211_aacs_ctx *aacs, int method);
#endif
#endif /* _IEEE80211_AACS_SYS_H_ */
