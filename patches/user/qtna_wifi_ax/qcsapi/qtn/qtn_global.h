/*
 * Copyright (c) 2011-2012 Quantenna Communications, Inc.
 * All rights reserved.
 */

#ifndef _QTN_GLOBAL_H
#define _QTN_GLOBAL_H

#define QTN_6G_FREQ_ALLOC_DRAFT_7		1

/* channel freq band information */
#define QTN_2G_FIRST_OPERATING_CHAN		1
#define QTN_2G_LAST_OPERATING_CHAN		14
#define QTN_4G_FIRST_OPERATING_CHAN		183
#define QTN_4G_LAST_OPERATING_CHAN		196
#define QTN_5G_FIRST_OPERATING_CHAN		36
#define QTN_5G_LAST_UNII1_OPERATING_CHAN	48
#define QTN_5G_LAST_UNII2_OPERATING_CHAN	140
#define QTN_5G_LAST_OPERATING_CHAN		169
#define QTN_5G_LAST_OPERATING_CHAN_OF_LOWBAND	64
#define QTN_5G_FIRST_OPERATING_CHAN_OF_HIGHBAND	100

#define QTN_6G_FIRST_OPERATING_CHAN		1
#if QTN_6G_FREQ_ALLOC_DRAFT_7
#define QTN_6G_LAST_OPERATING_CHAN		233
#else
#define QTN_6G_LAST_OPERATING_CHAN		253
#endif

#define QTN_FREQ_OFFSET_PER_CHAN	5

/* All frequencies are in MHz */
#define QTN_FREQ_2GHZ_FIRST		2412	/* Chan 1 */
#define QTN_FREQ_2GHZ_LAST		2484	/* Chan 14 */
#define QTN_FREQ_4GHZ_FIRST		4920	/* Chan 184 */
#define QTN_FREQ_5GHZ_BASE		5000
#define QTN_FREQ_5GHZ_FIRST		5180	/* Chan 36 */
#define QTN_FREQ_5GHZ_LAST \
	(QTN_FREQ_5GHZ_BASE + (QTN_5G_LAST_OPERATING_CHAN * QTN_FREQ_OFFSET_PER_CHAN))

#if QTN_6G_FREQ_ALLOC_DRAFT_7
#define QTN_FREQ_6GHZ_BAND_START	5950  /* Starting freq of 6GHz band supported in driver */
#define QTN_FREQ_6GHZ_CHAN_2		5935  /* Will not be supported in driver */
#define QTN_FREQ_6GHZ_FROM_CHAN_NO(_ch) \
		(((_ch) == 2) ? QTN_FREQ_6GHZ_CHAN_2 : \
		(QTN_FREQ_6GHZ_BAND_START + ((_ch) * QTN_FREQ_OFFSET_PER_CHAN)))
#define QTN_CHAN_NO_FROM_FREQ_6G(_freq) \
		(((_freq) == QTN_FREQ_6GHZ_CHAN_2) ? 2 : \
		(((_freq) - QTN_FREQ_6GHZ_BAND_START) / QTN_FREQ_OFFSET_PER_CHAN))
#else
#define QTN_FREQ_6GHZ_BAND_START	5940  /* Starting freq of 6GHz band supported in driver */
#define QTN_FREQ_6GHZ_FROM_CHAN_NO(_ch) \
		(QTN_FREQ_6GHZ_BAND_START + ((_ch) * QTN_FREQ_OFFSET_PER_CHAN))
#define QTN_CHAN_NO_FROM_FREQ_6G(_freq) \
		(((_freq) - QTN_FREQ_6GHZ_BAND_START) / QTN_FREQ_OFFSET_PER_CHAN)
#endif
#define QTN_FREQ_6GHZ_FIRST_CHAN	QTN_FREQ_6GHZ_FROM_CHAN_NO(QTN_6G_FIRST_OPERATING_CHAN)
#define QTN_FREQ_6GHZ_LAST_CHAN		QTN_FREQ_6GHZ_FROM_CHAN_NO(QTN_6G_LAST_OPERATING_CHAN)
#define QTN_FREQ_6GHZ_BAND_END		(QTN_FREQ_6GHZ_LAST_CHAN + QTN_FREQ_OFFSET_PER_CHAN)

#define QTN_FREQ_IS_IN_2G_BAND(_freq)	\
	(((_freq) >= QTN_FREQ_2GHZ_FIRST) && ((_freq) <= QTN_FREQ_2GHZ_LAST))
#define QTN_FREQ_IS_IN_4G_BAND(_freq)	\
	(((_freq) >= QTN_FREQ_4GHZ_FIRST) && ((_freq) < QTN_FREQ_5GHZ_FIRST))
#define QTN_FREQ_IS_IN_5G_BAND(_freq)	\
	(((_freq) >= QTN_FREQ_5GHZ_FIRST) && ((_freq) <= QTN_FREQ_5GHZ_LAST))
#define QTN_FREQ_IS_IN_6G_BAND(_freq)	\
	(((_freq) >= QTN_FREQ_6GHZ_FIRST_CHAN) && ((_freq) <= QTN_FREQ_6GHZ_LAST_CHAN))

#define QTN_IS_2G_OPER_CHAN(ch_num)	\
	 (((ch_num) >= QTN_2G_FIRST_OPERATING_CHAN) && \
	  ((ch_num) <= QTN_2G_LAST_OPERATING_CHAN))

#define QTN_IS_4G_OPER_CHAN(ch_num)	\
	 (((ch_num) >= QTN_4G_FIRST_OPERATING_CHAN) && \
	  ((ch_num) <= QTN_4G_LAST_OPERATING_CHAN))

#define QTN_IS_5G_OPER_CHAN(ch_num)	\
	 (((ch_num) >= QTN_5G_FIRST_OPERATING_CHAN) && \
	  ((ch_num) <= QTN_5G_LAST_OPERATING_CHAN))

#define QTN_IS_6G_OPER_CHAN(ch_num)	\
	 (((ch_num) >= QTN_6G_FIRST_OPERATING_CHAN) && \
	  ((ch_num) <= QTN_6G_LAST_OPERATING_CHAN))

#define IEEE80211_FDR_LOWBAND(ch) \
	((IC_IEEE_IS_CHAN_IN_2G_OR_5G(ch)) && \
	 ((ch) >= QTN_5G_FIRST_OPERATING_CHAN) && \
	 ((ch) <= QTN_5G_LAST_OPERATING_CHAN_OF_LOWBAND))
#define IEEE80211_FDR_HIGHBAND(ch) \
	((IC_IEEE_IS_CHAN_IN_2G_OR_5G(ch)) && \
	 ((ch) >= QTN_5G_FIRST_OPERATING_CHAN_OF_HIGHBAND) && \
	 ((ch) <= QTN_5G_LAST_OPERATING_CHAN))

/* Retry counts */
#define QTN_INVD_LEGACY_RETRY_PER_RATE	0xfffffff
#define QTN_DEFAULT_LEGACY_RETRY_PER_RATE 1
#define QTN_PROBE_RES_MAX_RETRY_COUNT	4
#define QTN_TABLE0_RETRY_CNT		14 /* Must be 14 or less so RTS fail doesn't overflow */

#define QTN_GI_SELECT_ENABLE		6	/* Ref: enum qtn_gi_adapt_setting */
#define QTN_PPPC_SELECT_ENABLE		1	/* Ref: enum qtn_pppc_adapt_setting */
#define QTN_PPPC_SS_ITER		8
#define QTN_PPPC_SEL_LO_PWR_DEFAULT	0

#define QTN_GLOBAL_INIT_DEF_MATRIX		1

#define QTN_AC_BE_INHERIT_VO_NO_STA		4
#define QTN_AC_BE_INHERIT_VI_NO_STA		3
#define QTN_AC_BE_INHERIT_VO			2
#define QTN_AC_BE_INHERIT_VI			1
#define QTN_AC_BE_INHERIT_DISABLE		0
#define QTN_AC_BE_INHERIT_Q2Q_ENABLE		1
#define QTN_AC_BE_INHERIT_Q2Q_DISABLE		0
#define QTN_GLOBAL_MU_ENABLE			1
#define QTN_GLOBAL_MU_DISABLE			0
#define QTN_GLOBAL_MU_INITIAL_STATE		QTN_GLOBAL_MU_DISABLE
#define QTN_GLOBAL_MU_QMAT_BYPASS_MODE_INITIAL_STATE 0
#define QTN_GLOBAL_MU_MAX_NODES_IN_MIXED_GRSEL	3 /* 0 means "disable mixed group selection" */
#define QTN_GLOBAL_MU_OS_ENABLED                1

/*
 * The MuC TX Queuing algorithm is selected by setting
 * g_tx_queuing_alg. Values are:
 * 0 = Round robin
 * 1 = Equal airtime
 * 2 = Greedy best
 * 3 = Round robin (filling to make a power of 4)
 * x >= 4: algorithm chosen by (val % 4), with airtime
 * debugs enabled, printed every (val) seconds.
 */
#define QTN_TX_QUEUING_ALG_ROUND_ROBIN		0
#define QTN_TX_QUEUING_ALG_EQUAL_AIRTIME	1
#define QTN_TX_QUEUING_ALG_GREEDY_BEST		2
#define QTN_TX_QUEUING_ALGS			4
#if QTN_TX_QUEUING_ALGS & (QTN_TX_QUEUING_ALGS - 1)
	#error QTN_TX_QUEUING_ALGS should be a power of 2
#endif
#define QTN_GLOBAL_INIT_TX_QUEUING_ALG		QTN_TX_QUEUING_ALG_ROUND_ROBIN

#define QTN_TX_AIRTIME_XMIT_BUMP_USECS		100

#define QTN_TX_BUF_RETURN_MIN			100
/* must be greater than the above to prevent stalling */
#define QDRV_TX_LOW_RATE_TOKENS_MAX		QTN_TX_BUF_RETURN_MIN + 28

#define QTN_GLOBAL_RATE_NSS_MIN		IEEE80211_HT_NSS1
#define QTN_GLOBAL_RATE_NSS_MAX		IEEE80211_HT_NSS4
#define QTN_RX_REORDER_BUF_TIMEOUT_US	200000
#define QTN_TX_MSDU_EXPIRY		0	/* allow MSDUs to time out? */
#define QTN_TX_AGGREGATION		1	/* allow aggregation? */
#define QTN_CALSTATE_CALIB		1
#define QTN_CALSTATE_PROD		3
#define QTN_CALSTATE_DEFAULT		QTN_CALSTATE_PROD
#define QTN_CALSTATE_IS_PROD()		(likely(g_qtn_params.g_calstate == QTN_CALSTATE_PROD))
#define QTN_CALSTATE_VPD_LOG		0
#define QTN_CALSTATE_VPD_LINEAR		1
#define QTN_CALSTATE_MIN_TX_POWER	7
#define QTN_CALSTATE_MAX_TX_POWER	23
#define QTN_EMI_POWER_SWITCH_ENABLE	1

#define QTN_NSS_CAP_TO_IE(_nss_cap)	MIN((_nss_cap) - 1, QTN_GLOBAL_RATE_NSS_MAX - 1)

#define QTN_TX_AMSDU_DISABLED		0
#define QTN_TX_AMSDU_ADAPTIVE		1
#define QTN_TX_AMSDU_FIXED		0xff

#define QTN_FLAG_ACT_FRAME_RTS_CTS		0x00000001
#define QTN_FLAG_ACT_FRAME_NO_LDPC		0x00000002
#define QTN_FLAG_MCS_UEQM_DISABLE		0x00000004
#define QTN_FLAG_AUC_TX				0x00000008
#define QTN_FLAG_RA_BW_SWITCHING_ENABLE_11N	0x00000010
#define QTN_FLAG_RA_BW_SWITCHING_ENABLE_11AC	0x00000020
#define QTN_FLAG_RA_BW_SWITCHING_ENABLE_11AX	0x00000040

#if PEARL_FPGA_PLATFORM
/* The FPGA BB BP module doesn't have UEQM rate mapping except part of 1024QAM rates (MCS10,11) */
#define QTN_GLOBAL_MUC_FLAGS_DEFAULT		QTN_FLAG_RA_BW_SWITCHING_ENABLE_11N | \
						QTN_FLAG_RA_BW_SWITCHING_ENABLE_11AC | QTN_FLAG_MCS_UEQM_DISABLE
#else
#define QTN_GLOBAL_MUC_FLAGS_DEFAULT		QTN_FLAG_RA_BW_SWITCHING_ENABLE_11N | \
						QTN_FLAG_RA_BW_SWITCHING_ENABLE_11AC | \
						QTN_FLAG_RA_BW_SWITCHING_ENABLE_11AX
#endif

#define QTN_NDPA_IN_HT_VHT_FORMAT	0
#define QTN_NDPA_IN_LEGACY_FORMAT	1

#define QTN_DBG_MODE_SEND_PWR_MGT		0x00000001
#define QTN_DBG_MODE_ACCEPT_PWR_MGT		0x00000002
#define QTN_DBG_MODE_TX_PKT_LOSS		0x00000004
#define QTN_DBG_MODE_DELBA_ON_TX_PKT_LOSS	0x00000008
#define QTN_DBG_MODE_CCA_FORCE			0x00000010
#define QTN_DBG_MODE_INJECT_INV_NDP		0x00000020
#define QTN_DBG_MODE_CBF_DUMP			0x00000040
#define QTN_DBG_MODE_SSB_MODE			0x00000080

#define QTN_DBG_FD_CHECK_PERIODIC	0x00000001
#define QTN_DBG_FD_DUMP_OLD		0x00000002
#define QTN_DBG_FD_CHECK_ONESHOT	0x00000004
#define QTN_DBG_FD_DUMP_BCN_FAIL	0x00000008
#define QTN_DBG_FD_DUMP_VERBOSE	0x00000010 /* + top byte is the FD to dump */
#define QTN_DBG_DUMP_SC		0x00000020
#define QTN_DBG_DUMP_AGEQ		0x00000040
#define QTN_DBG_FD_FLAG_MASK		0x0000FFFF

#define QTN_HW_UPDATE_NDPA_DUR  0x0
#define	QTN_TXBF_TX_CNT_DEF_THRSHLD 2

#define QTN_RX_BAR_SYNC_DISABLE	0
#define QTN_RX_BAR_SYNC_QTN	1
#define QTN_RX_BAR_SYNC_ALL	2
/* copy from macfw/qtn/if_qtnvar.h */
#define QN_AIRTIME_ACCUM_START       (1 << 0)
#define QN_AIRTIME_ACCUM_STOP        (1 << 1)
#define QN_AIRTIME_ALL_NODES_CONTROL (1 << 24)

struct qtn_wowlan_params {
	uint16_t	host_state;
	uint16_t	wowlan_match;
	uint16_t	l2_ether_type;
	uint16_t	l3_udp_port;
};

#define QTN_ETSI_COT_PCLASS_4	4
#define QTN_ETSI_COT_PCLASS_3	3
#define QTN_ETSI_COT_LEGACY	1

#define ETSI_MAXCOT_P4		2000
#define ETSI_MAXCOT_P3		4000
#define ETSI_MAXCOT_P1_2	6000
#define MAXCOT_MAX_OHEAD	277	/* Headroom for RTS-SIFS-CTS-SIFS-SIFS-ACK */

#if (defined(MUC_BUILD) || defined(SYSTEM_BUILD))

#define QTN_RX_GAIN_MIN_THRESHOLD		16
#define QTN_RX_GAIN_MAX_THRESHOLD		44
#define QTN_RX_GAIN_TIMER_INTV			1000 /* msecs */

/* counter for delay in RFIC6/RFIC8 500Mhz */
#define QTN_RF_DELAY_MICRO_S			500
#define QTN_RF_DELAY_MILI_S			QTN_RF_DELAY_MICRO_S * 1000

#define QTN_CFG_SWCCA_FOR_BW_EN		1
#define QTN_CFG_SWCCA_FOR_BW_DIS	0

#define SIZE_D1(x)	(sizeof(x)/sizeof(x[0]))
#define SIZE_D2(x)	(sizeof(x[0])/sizeof(x[0][0]))

struct qtn_gain_settings {
	uint8_t	gain_flags;
/* Enable SW workaround for short range association */
#define QTN_AUTO_PWR_ADJUST_EN		0x1
/* Hardware supports automatic RX gain */
#define QTN_SHORTRANGE_SCANCNT_HW	0x2
	uint32_t	gain_cumulative; /* Cumulative gain for all rx pkts */
	uint32_t	gain_num_pkts;	 /* Number of pkts for which cumulative gain was considered */
	uint32_t	gain_timer;
	uint32_t	gain_min_thresh;
	uint32_t	gain_max_thresh;
	uint32_t	gain_timer_intv;
	uint32_t	gain_low_txpow;
	int		ext_lna_gain;
	int		ext_lna_bypass_gain;
};

#ifndef QTN_CCA_SCS_DEBUG
#define QTN_CCA_SCS_DEBUG	1
#endif

#define QTN_SCS_MAX_OC_STATS	32
#define QTN_BGSCAN_MAX_OC_STATS	6
/* off channel params */
struct qtn_scs_oc_stats {
	uint32_t	oc_chan_band;
	uint32_t	oc_bw_sel;
	uint32_t	oc_crc_cnt;
	uint32_t	oc_lp_cnt;
	uint32_t	oc_sp_cnt;
	uint32_t	oc_cca_pri;
	uint32_t	oc_cca_sec;
	uint32_t	oc_cca_sec40;
	uint32_t	oc_cca_curr_bw_busy;
	uint32_t	oc_cca_smpl;
	uint32_t	oc_cca_try;
	uint32_t	oc_bcn_recvd;
	int32_t		oc_hw_noise;
	uint32_t	oc_cca_busy[QTN_BW_160M + 1];
};

struct qtn_cca_counts {
	uint32_t	cca_pri_cnt;
	uint32_t	cca_sec_cnt;
	uint32_t	cca_sec40_cnt;
	uint32_t	cca_curr_bw_busy_cnt;
	uint32_t	cca_busy_cnt[QTN_BW_160M + 1];
	uint32_t	cca_sample_cnt;
	uint32_t	cca_try_cnt;
	uint32_t	cca_csw_cnt;
	uint32_t	cca_off_pri_cnt;
	uint32_t	cca_off_sec_cnt;
	uint32_t	cca_off_sec40_cnt;
	uint32_t	cca_off_curr_bw_busy_cnt;
	uint32_t	cca_off_busy_cnt[QTN_BW_160M + 1];
	uint32_t	cca_off_sample_cnt;
	uint32_t	cca_off_res_cnt;
	uint32_t	cca_off_try_cnt;
	uint32_t	cca_meas_cnt;
};

struct qtn_scs_params {
	uint32_t	cca_pri_cnt;
	uint32_t	cca_sec_cnt;
	uint32_t	cca_sec40_cnt;
	uint32_t	cca_curr_bw_busy_cnt;
	uint32_t	cca_busy_cnt[QTN_BW_160M + 1];
	uint32_t	cca_sample_cnt;
	uint32_t	cca_try_cnt;
	uint32_t	cca_csw_cnt;
	uint32_t	cca_off_res_cnt;
	uint32_t	cca_off_try_cnt;
	uint32_t	cca_meas_cnt;
	uint32_t	tx_usecs;
	uint32_t	rx_usecs;
	uint32_t	rx_usecs_80;
	uint32_t	rx_usecs_40;
	uint32_t	rx_usecs_20;
	uint32_t	bcn_recvd;
	uint32_t	oc_stats_index;
	struct qtn_scs_oc_stats oc_stats[QTN_SCS_MAX_OC_STATS];
};

struct qtn_bgscan_params {
	uint32_t	oc_stats_index;
	struct qtn_scs_oc_stats oc_stats[QTN_BGSCAN_MAX_OC_STATS];
};

#define QTN_AUTO_CCA_SHORT_PREAMBLE_THRESHOLD	15000	/* If higher than this value, increase the CCA threshold */
#define QTN_AUTO_CCA_INTF_THRESHOLD		250	/* If higher than this value, increase the CCA threshold */

#define QTN_AUTO_CCA_THRESHOLD_MAX		0x10000	/* The max cca threshold we can set */


struct qtn_auto_cca_params {
#define QTN_AUTO_CCA_FLAGS_DISABLE			0x0
#define QTN_AUTO_CCA_FLAGS_ENABLE			0x1
#define QTN_AUTO_CCA_FLAGS_DEBUG			0x2
#define QTN_AUTO_CCA_FLAGS_SAMPLE_ONLY			0x4
	uint32_t	flags;

	uint32_t	spre_threshold;
	uint32_t	cca_intf_threshold;
	uint32_t	cca_threshold_max;
};

#define QTN_ETSI_COT_PCLASS_4	4
#define QTN_ETSI_COT_PCLASS_3	3
#define QTN_ETSI_COT_LEGACY	1

#define ETSI_MAXCOT_P4		2000
#define ETSI_MAXCOT_P3		4000
#define ETSI_MAXCOT_P1_2	6000
#define MAXCOT_MAX_OHEAD	277	/* Headroom for RTS-SIFS-CTS-SIFS-SIFS-ACK */

/*
 * Fixme.
 * For BBIC5 + RFIC6, not sure the proximity issue is still there.
 * Need to confirm with SYS/RF guys later.
 * Disable it now, and some code(if_qtn_auto_cca.c) need to be modified to support BBIC5
 */
#define QTN_AUTO_CCA_PARARMS_DEFAULT		\
	{ QTN_AUTO_CCA_FLAGS_DISABLE, QTN_AUTO_CCA_SHORT_PREAMBLE_THRESHOLD, \
		QTN_AUTO_CCA_INTF_THRESHOLD, QTN_AUTO_CCA_THRESHOLD_MAX}
#define QTN_HR_INTERVAL (3*HZ)
#define QTN_HR_COUNT_MAX (10)

struct qtn_global_param {
	uint32_t	g_legacy_retry_count;
	uint32_t	g_dbg_check_flags;
	uint32_t	g_dbg_stop_flags;
	uint32_t	g_dbg_mode_flags;
	uint32_t	g_rx_agg_timeout;
	uint32_t	g_muc_flags;
	uint32_t	g_probe_res_retries;
	uint8_t		g_slow_eth_war;
	uint8_t		g_tx_msdu_expiry;
	uint8_t		g_tx_aggregation;
	uint32_t	g_iot_tweaks;
	uint8_t		g_calstate;
	uint32_t	g_ack_policy;
	uint32_t	g_dbg_fd_flags;
	uint32_t	g_qtn_disassoc_fd_threshold;
	uint32_t	g_qtn_qn_fd_threshold;
	int32_t         g_2_tx_chains_mimo_mode;
	uint8_t		g_calstate_tx_power;
	uint8_t		g_min_tx_power;
	uint8_t		g_max_tx_power;
	uint8_t		g_emi_power_switch_enable;
	uint8_t		g_dyn_agg_timeout;
	uint8_t		g_tx_amsdu;
	uint8_t         g_11g_erp;
	uint8_t		g_single_agg_queuing;
	uint32_t	g_tx_restrict;
	uint32_t	g_tx_restrict_fd_limit;
	uint32_t	g_tx_restrict_rate;	/* Max packets per second in Tx restrict mode */
	uint32_t	g_tx_restrict_attempts;
	uint32_t        g_aggr_rts_threshold;        /* Aggregate RTS threshold */
	uint8_t		g_tx_queuing_alg;
	uint32_t        g_carrier_id;
	uint8_t		g_rx_accelerate;
	uint8_t		g_rx_accel_lu_sa;
	uint16_t	g_tx_maxmpdu;
	uint8_t		g_tx_ac_inheritance;	/* promote AC_BE traffic to vo/vi */
	uint8_t		g_tx_ac_q2q_inheritance;/* promote AC_BE traffic to vo/vi */
	uint8_t		g_tx_1ss_amsdu_supp;	/* enable-disable 1ss AMSDU support - Non-qtn clients */
	uint32_t        g_vht_ndpa_dur;         /* manual update VHT NDPA duration, if it is 0, then HW auto update */
	uint32_t        g_txbf_pkt_cnt;         /* Tx operation count threshold to a TxBF station */
	struct qtn_auto_cca_params	g_auto_cca_params;
	struct qtn_wowlan_params wowlan_params;
	uint8_t		g_rx_optim;
	uint8_t		g_airfair;
	uint8_t		g_cca_fixed;
	uint8_t		g_ndpa_legacy_format;	/* Configure HT-VHT / Legacy frame format for NDP announcements */
	uint8_t		g_inst_1ss_def_mat_en;		/* enable default 1ss matrix feature */
	uint8_t		g_rate_train_dbg;
	uint8_t		g_rx_optim_pkt_stats;
	uint8_t		g_mrc_enable;
	uint8_t		g_beaconing_scheme;
	uint32_t	g_hreset_interval;	/* unit is jiffies, interval less than it is treated as a constant hard-reset */
	uint32_t	g_hreset_reboot_threshold;	/* number of constant hard-reset bigger than it will cause a reboot */
	uint32_t	g_cfg_swcca_for_bw;	/* Allow SWCCA configuration to follow system BW */
	uint8_t		g_cot_tweaks;		/* Channel Occupancy Time tweaks */
	uint8_t		g_no_cal;		/* no calibration file */
	uint32_t        g_rts_threshold;        /* RTS threshold */
	uint32_t	g_rx_bar_sync;		/* sync rx reorder window on receiving BAR */
	uint32_t	g_tcp_ack_compress_thres[AUC_CORES_NUM];
	uint32_t	g_fdr_mode;
	char		*g_last_field;		/* Add all new fields before this one */
};

/* Please keep this structure in sync with qtn_global_param */
#define G_PARAMS_INIT	{			\
	QTN_INVD_LEGACY_RETRY_PER_RATE,		\
	0,					\
	0,					\
	0,					\
	QTN_RX_REORDER_BUF_TIMEOUT_US,		\
	QTN_GLOBAL_MUC_FLAGS_DEFAULT,		\
	QTN_PROBE_RES_MAX_RETRY_COUNT,		\
	0,					\
	QTN_TX_MSDU_EXPIRY,			\
	QTN_TX_AGGREGATION,			\
	QTN_IOT_DEFAULT_TWEAK,			\
	QTN_CALSTATE_DEFAULT,			\
	1,					\
	0,					\
	50,					\
	64,					\
	1,					\
	QTN_CALSTATE_VPD_LOG,			\
	QTN_CALSTATE_MIN_TX_POWER,		\
	QTN_CALSTATE_MAX_TX_POWER,		\
	QTN_EMI_POWER_SWITCH_ENABLE,		\
	0,					\
	QTN_TX_AMSDU_ADAPTIVE,			\
	0,					\
	0,					\
	1,					\
	IEEE80211_NODE_TX_RESTRICT_LIMIT,	\
	IEEE80211_TX_RESTRICT_RATE,		\
	IEEE80211_NODE_TX_RESTRICT_RETRY,	\
	IEEE80211_RTS_THRESH_OFF,		\
	QTN_GLOBAL_INIT_TX_QUEUING_ALG,		\
	0,					\
	1,					\
	1,					\
	IEEE80211_VHTCAP_MAX_MPDU_11454,	\
	QTN_AC_BE_INHERIT_VO,			\
	QTN_AC_BE_INHERIT_Q2Q_ENABLE,		\
	QTN_TX_AMSDU_DISABLED,			\
	QTN_HW_UPDATE_NDPA_DUR,			\
	QTN_TXBF_TX_CNT_DEF_THRSHLD,		\
	QTN_AUTO_CCA_PARARMS_DEFAULT,		\
	{0, 0, 0x0842, 0xffff},			\
	0,					\
	QTN_AUC_AIRFAIR_DFT,			\
	0,					\
	QTN_NDPA_IN_LEGACY_FORMAT,		\
	1,					\
	0,					\
	0,					\
	1,					\
	0,					\
	QTN_HR_INTERVAL,			\
	QTN_HR_COUNT_MAX,			\
	QTN_CFG_SWCCA_FOR_BW_EN,		\
	0,					\
	0,					\
	IEEE80211_RTS_THRESH_OFF,		\
	QTN_RX_BAR_SYNC_ALL,			\
	MULTI_AUCS_ARRAY_INIT_DIFF(			\
		TCP_ACK_COMPRESS_IDLENESS_THRES,	\
		TCP_ACK_COMPRESS_IDLENESS_THRES,	\
		TCP_ACK_COMPRESS_IDLENESS_THRES),	\
	0,					\
	"end"					\
}

struct qtn_global_vopt_param {
	uint8_t		g_tx_ac_inheritance;
	uint8_t		g_ht_20m_resp_rate;
	uint32_t	g_tcp_ack_compress_thres[AUC_CORES_NUM];
};

#define G_PARAMS_VOPT_INIT	{		\
	QTN_AC_BE_INHERIT_VO,			\
	0,					\
	MULTI_AUCS_ARRAY_INIT_DIFF(		\
		TCP_ACK_COMPRESS_IDLENESS_THRES,	\
		TCP_ACK_COMPRESS_IDLENESS_THRES,	\
		TCP_ACK_COMPRESS_IDLENESS_THRES),	\
}

extern struct qtn_global_param g_qtn_params;
extern struct qtn_scs_params g_qtn_scs_params[];
extern volatile __uncached__ struct qtn_gain_settings g_gain;
extern struct qtn_cca_counts g_cca_counts[];
extern struct qtn_cca_stats g_qtn_cca_stats[];
extern uint32_t g_qtn_rxtime_usecs[];
extern uint32_t g_qtn_txtime_usecs[];
extern uint32_t g_qtn_txtime_noack_usecs[];
extern uint32_t g_qtn_rxtime_usecs_sta[];
extern uint32_t g_qtn_txtime_usecs_sta[];
extern uint32_t g_rf_xmit_status[];
extern int vlan_enabled_bus;
extern uint8_t vlan_drop_stag_bus;
#ifdef QTN_CCA_SCS_DEBUG
extern uint32_t g_qtn_txtime_pmbl_usecs[];
extern uint32_t g_qtn_rxtime_pmbl_usecs[];
extern uint32_t g_qtn_txtime_dens_usecs[];
extern uint32_t g_qtn_rxtime_dens_usecs[];
extern uint32_t g_qtn_txtime_data_usecs[];
extern uint32_t g_qtn_rxtime_data_usecs[];
extern uint32_t g_qtn_txpkt[];
extern uint32_t g_qtn_rxpkt[];
extern uint32_t g_qtn_txbyte[];
extern uint32_t g_qtn_rxbyte[];
#endif

#endif	/* defined(MUC_BUILD) */

/*
 * SKBs on the power save queue are tagged with an age and timed out.  We reuse the
 * hardware checksum field in the mbuf packet header to store this data.
 */
#define skb_age csum_offset

#define M_AGE_SET(skb,v)	(skb->skb_age = v)
#define M_AGE_GET(skb)		(skb->skb_age)
#define M_AGE_SUB(skb,adj)	(skb->skb_age -= adj)

#define QTN_MUC_BAND_2_4_GHZ	0
#define QTN_MUC_BAND_5_GHZ	1
#define QTN_MUC_BAND_6_GHZ	2
#define QTN_MUC_TOTAL_BANDS	3
#define QTN_MUC_BAND_UNKNOWN	0xFF

#define RFIC_PROJID_MASK       0x000000e0
#define RFIC_VERID_MASK                0x0000001c
#define RFIC_CHIPID_MASK       0x00000003
#define RFIC_ALL_MASK		0xfffffff0

#define RFIC_PROJID_AND_VER(projid, ver)       (((projid) << 5) | ((ver) << 2))

/* RFIC6 chip version */
#define RFIC6_PROJ_ID  3
#define RFIC6_VER_A    0
#define RFIC6_VER_B    2
#define RFIC6_VER_C    3
#define RFIC6_VER_D    4
#define RFIC6_VER_E    5
#define RFIC6_VER_MAX  7

#define RFIC6_PROJID_AND_VER_A RFIC_PROJID_AND_VER(RFIC6_PROJ_ID, RFIC6_VER_A)
#define RFIC6_PROJID_AND_VER_B RFIC_PROJID_AND_VER(RFIC6_PROJ_ID, RFIC6_VER_B)
#define RFIC6_PROJID_AND_VER_C RFIC_PROJID_AND_VER(RFIC6_PROJ_ID, RFIC6_VER_C)
#define RFIC6_PROJID_AND_VER_D RFIC_PROJID_AND_VER(RFIC6_PROJ_ID, RFIC6_VER_D)
#define RFIC6_PROJID_AND_VER_E RFIC_PROJID_AND_VER(RFIC6_PROJ_ID, RFIC6_VER_E)

/* RFIC622 is 2 chain RFIC6 variant, but has own IDs */
#define RFIC622_PROJ_ID                5
#define RFIC622_VER_A          0
#define RFIC622_VER_MAX                7

#define RFIC622_PROJID_AND_VER_A       RFIC_PROJID_AND_VER(RFIC622_PROJ_ID, RFIC622_VER_A)

/* driver should treat RFIC6 vs RFIC622 identically - do explicit
 * comparisons in rare cases when they need to be distinguished
 */

#define IS_RFIC6_ID(x)	(((x) == RFIC6_PROJ_ID) ||  ((x) == RFIC622_PROJ_ID))

#define IS_RFIC6X_VER(x, y, a, b) ((((x) == RFIC6_PROJ_ID) && ((y) == (a))) || \
					(((x) == RFIC622_PROJ_ID) && ((y) == (b))))

#define IS_RFIC6X_VER_GE(x, y, a, b) ((((x) == RFIC6_PROJ_ID) && ((y) >= (a))) || \
					(((x) == RFIC622_PROJ_ID) && ((y) >= (b))))

#define IS_RFIC6X_VER_LE(x, y, a, b) ((((x) == RFIC6_PROJ_ID) && ((y) <= (a))) || \
					(((x) == RFIC622_PROJ_ID) && ((y) <= (b))))

/* RFIC8 chip version */
#define RFIC8_PROJ_ID_1		4
#define RFIC842_PROJ_ID		6
#define RFIC842_VER_A		0
#define RFIC842_VER_B		1
#define RFIC842_VER_C		2
#define RFIC842_VER_MAX		7
#define RFIC8_PROJ_ID_2		7
#define RFIC8_VER_A		0
#define RFIC8_VER_B		1
#define RFIC8_VER_C		2
#define RFIC8_VER_D		3
#define RFIC8_VER_MAX		7

/* RFIC Reg 6 values for RFIC8 Rev B */
#define RFIC8_REV_B0	(0x84)		/* Proj/Ver/ChipID 4/1/0 */
#define RFIC8_REV_B1	(0x87)		/* Proj/Ver/ChipID 4/1/3 */
#define RFIC8_REV_B2	(0xE2)		/* Proj/Ver/ChipID 7/0/2 */
#define RFIC8_REV_C0	(0x88)		/* Proj/Ver/ChipID 4/2/0 */
#define RFIC8_REV_C0_V2	(0xE3)		/* Proj/Ver/ChipID 7/0/3 */
#define RFIC8_REV_C1	(0x89)		/* Proj/Ver/ChipID 4/2/1 */
#define RFIC8_REV_C1_V2	(0xE4)		/* Proj/Ver/ChipID 7/1/0 */
#define RFIC8_REV_C1_V3	(0xE8)		/* Proj/Ver/ChipID 7/2/0 */
#define RFIC8_REV_C2	(0x8A)		/* Proj/Ver/ChipID 4/2/2 */
#define RFIC8_REV_C2_V2	(0xE5)		/* Proj/Ver/ChipID 7/1/1 */
#define RFIC8_REV_C3	(0x8B)		/* Proj/Ver/ChipID 4/2/3 */
#define RFIC8_REV_D0	(0x8C)		/* Proj/Ver/ChipID 4/3/0 */
#define RFIC842_REV_A0	(0xD0)		/* Proj/Ver/ChipID 6/4/0 */
#define RFIC842_REV_B0	(0xD4)		/* Proj/Ver/ChipID 6/5/0 */
#define RFIC842_REV_C0	(0xDC)		/* Proj/Ver/ChipID 6/7/0 */

#define IS_RFIC8_ID(x)	(((x) == RFIC8_PROJ_ID_1) || ((x) == RFIC8_PROJ_ID_2))
#define IS_RFIC842_ID(x)	((x) == RFIC842_PROJ_ID)
#define IS_RFIC8_VER_C(x, y) ((IS_RFIC8_ID(x)) && ((y) > RFIC8_VER_B))

#define RFIC_EX_ID_FORMAT_MASK	(0x800000)	/* ID Format bit in Chip ID register */
#define RFIC_EX_ID_SHIFT	(23)
#define RFIC_EX_VARIANT_MASK	(0x70000)
#define RFIC_EX_VARIANT_SHIFT	(16)
#define RFIC_EX_PROJ_ID_MASK	(0x3F00)
#define RFIC_EX_PROJ_ID_SHIFT	(8)
#define RFIC_EX_REV_ID_MASK	(0xF0)
#define RFIC_EX_REV_ID_SHIFT	(4)
#define RFIC_EX_METAL_MASK	(0xF)
#define RFIC_EX_METAL_SHIFT	(0)
#define RFIC_EX_PROJ_ID_FLAG	(0x40)		/* | with extended ID for unique 8-bit value */

#define	IS_RFIC_EX_ID_MODE(x)	((x) & RFIC_EX_ID_FORMAT_MASK)

#define GET_RFIC_EX_VARIANT(x)	(((x) & RFIC_EX_VARIANT_MASK) >> RFIC_EX_VARIANT_SHIFT)
#define GET_RFIC_EX_PROJ_ID(x)	(((x) & RFIC_EX_PROJ_ID_MASK) >> RFIC_EX_PROJ_ID_SHIFT)
#define GET_RFIC_EX_REV_ID(x)	(((x) & RFIC_EX_REV_ID_MASK) >> RFIC_EX_REV_ID_SHIFT)
#define GET_RFIC_EX_METAL(x)	(((x) & RFIC_EX_METAL_MASK) >> RFIC_EX_METAL_SHIFT)

#define RFIC10_PROJ_ID_BASE	(0x1)		/* project_id field value for RFIC10 */
#define RFIC10_PROJ_ID	(RFIC_EX_PROJ_ID_FLAG | RFIC10_PROJ_ID_BASE)	/* 0x41 */
#define RFIC10_VER_A	(0xA)
#define RFIC10_VER_B	(0xB)
#define RFIC10_VER_C	(0xC)
#define RFIC10_VER_D	(0xD)
#define RFIC10_VER_MAX	(0xF)

/* RFIC Reg 6 values for RFIC10 */
#define RFIC10_REV_A0_V1	(0x8101A0)	/* Extended/Variant/Project/Rev/Metal 1/1/1/0xA/0 */
#define RFIC10_REV_A0_V2	(0x8201A0)	/* Extended/Variant/Project/Rev/Metal 1/2/1/0xA/0 */
#define RFIC10_REV_A0_V3	(0x8301A0)	/* Extended/Variant/Project/Rev/Metal 1/3/1/0xA/0 */
#define RFIC10_REV_A0_V4	(0x8401A0)	/* Extended/Variant/Project/Rev/Metal 1/4/1/0xA/0 */
#define RFIC10_REV_B0_V1	(0x8101B0)	/* Extended/Variant/Project/Rev/Metal 1/1/1/0xB/0 */

#define IS_RFIC10_ID(x)	((x) == RFIC10_PROJ_ID)

enum qtn_oper_mode {
	OPER_MODE_LEG	= 0,
	OPER_MODE_11N	= 1,
	OPER_MODE_11AC	= 2,
	OPER_MODE_11AX	= 3,
	OPER_MODE_NUM	= 4,
};

#define QTN_IEEE_BAND_MASK		0x00FF0000
#define QTN_IEEE_BAND_MASK_S		16
#define QTN_IEEE_CHAN_MASK		0x0000FFFF
#define QTN_IEEE_CHAN_MASK_S		0

#define	QTN_GET_CHAN_NUM(x)	(((x) & QTN_IEEE_CHAN_MASK) >> QTN_IEEE_CHAN_MASK_S)
#define	QTN_GET_FREQ_BAND(x)	(((x) & QTN_IEEE_BAND_MASK) >> QTN_IEEE_BAND_MASK_S)

#define QTN_RSSI_DBM_NEGINF (-1000)    /* -100.0 dBm */
/**
 * @addtogroup SystemAPIs
 * @{
 */

/**
 * \brief Enumeration to represent GPIO mode API.
 *
 * \sa qcsapi_gpio
 */
typedef enum {
	/**
	 * GPIO config mode
	 */
	GPIO_CMD_MODE = 1,

	/**
	 * Write GPIO PIN
	 */
	GPIO_CMD_WRITE = 2,

	/**
	 * Read GPIO PIN value
	 */
	GPIO_CMD_READ = 3,

	/**
	 * Connect GPIO PIN (feature dependent)
	 */
	GPIO_CMD_CONNECT = 4,

	/**
	 * Disconnect GPIO PIN (feature dependent)
	 */
	GPIO_CMD_DISCONNECT = 5,
	GPIO_CMD_MAX = GPIO_CMD_DISCONNECT,
	GPIO_CMD_INVALID = 0xffffffff
} gpio_cmd;

/** @} */

#endif	/* _QTN_GLOBAL_MUC_H */

