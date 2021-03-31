/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2006-2018 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: ieee80211_ioctl.h 1856 2006-12-14 01:38:00Z scottr $
 */
#ifndef _NET80211_IEEE80211_IOCTL_H_
#define _NET80211_IEEE80211_IOCTL_H_

/*
 * IEEE 802.11 ioctls.
 */
#include "net80211/_ieee80211.h"
#include "net80211/ieee80211.h"
#include "net80211/ieee80211_qos.h"
#include "net80211/ieee80211_crypto.h"
#include <qtn_power_table.h>

#pragma pack(4)

#define BSA_EVT_IFNAME_SIZE		19
/*
 * Per-channel flags to differentiate chan_pri_inactive configuration
 * between regulatory db and user configuration.
 * By default, system uses static regulatory db configs.
 * However driver shall always honour dynamic user coniguration.
 * In this way, user configuration will override regulatory db configs.
 */
enum {
	CHAN_PRI_INACTIVE_CFG_DATABASE = 0x1,
	CHAN_PRI_INACTIVE_CFG_USER_OVERRIDE = 0x2,
	CHAN_PRI_INACTIVE_CFG_AUTOCHAN_ONLY = 0x4,
};

/*
 * Per/node (station) statistics available when operating as an AP.
 */
struct ieee80211_nodestats {
	uint32_t ns_rx_data;		/* rx data frames */
	uint32_t ns_rx_mpdus;		/* rx mpdu frames */
	uint32_t ns_rx_ppdus;		/* rx ppdu frames */
	uint32_t ns_rx_msdus;		/* rx msdu frames from a-msdus */
	uint32_t ns_rx_mgmt;		/* rx management frames */
	uint32_t ns_rx_ctrl;		/* rx control frames */
	uint32_t ns_rx_11ax_trigger;	/* rx 11ax Trigger frame */
	uint32_t ns_rx_ucast;		/* rx unicast frames */
	uint32_t ns_rx_mcast;		/* rx multicast frames */
	uint64_t ns_rx_mcast_bytes;	/* rx multicast data count (bytes) */
	uint32_t ns_rx_bcast;		/* rx broadcast frames */
	uint64_t ns_rx_bytes;		/* rx data count (bytes) */
	uint64_t ns_rx_beacons;		/* rx beacon frames */
	uint32_t ns_rx_proberesp;	/* rx probe response frames */

	uint32_t ns_rx_dup;		/* rx discard because it's a dup */
	uint32_t ns_rx_retries;		/* rx retries */
	uint32_t ns_rx_ctrls;		/* rx ctrl frame */
	uint32_t ns_rx_noprivacy;	/* rx w/ wep but privacy off */
	uint32_t ns_rx_wepfail;		/* rx wep processing failed */
	uint32_t ns_rx_demicfail;	/* rx demic failed */
	uint32_t ns_rx_decap;		/* rx decapsulation failed */
	uint32_t ns_rx_defrag;		/* rx defragmentation failed */
	uint32_t ns_rx_disassoc;	/* rx disassociation */
	uint32_t ns_rx_deauth;		/* rx deauthentication */
	uint32_t ns_rx_decryptcrc;	/* rx decrypt failed on crc */
	uint32_t ns_rx_unauth;		/* rx on unauthorized port */
	uint32_t ns_rx_unencrypted;	/* rx unecrypted w/ privacy */
	uint32_t ns_rx_tid_bytes[WME_NUM_TID];

	uint32_t ns_tx_data;		/* tx data frames */
	uint32_t ns_tx_ppdus;		/* tx ppdu frames */
	uint32_t ns_tx_mpdus;		/* tx mpdu frames */
	uint32_t ns_tx_msdus;		/* tx msdu frames from a-msdus */
	uint32_t ns_tx_mgmt;		/* tx management frames */
	uint32_t ns_tx_ucast;		/* tx unicast frames */
	uint32_t ns_tx_mcast;		/* tx multicast frames */
	uint64_t ns_tx_mcast_bytes;	/* tx multicast data count (bytes) */
	uint32_t ns_tx_bcast;		/* tx broadcast frames */
	uint64_t ns_tx_bytes;		/* tx data count (bytes) */
	uint32_t ns_tx_probereq;	/* tx probe request frames */
	uint32_t ns_tx_uapsd;		/* tx on uapsd queue */
	uint32_t ns_tx_tid_bytes[WME_NUM_TID];

	uint32_t ns_tx_novlantag;	/* tx discard due to no tag */
	uint32_t ns_tx_vlanmismatch;	/* tx discard due to of bad tag */
	uint32_t ns_tx_unauth;		/* rx on unauthorized port */

	uint32_t ns_tx_eosplost;	/* uapsd EOSP retried out */

	uint32_t ns_ps_discard;		/* ps discard due to of age */

	uint32_t ns_uapsd_triggers;	/* uapsd triggers */

	/* MIB-related state */
	uint32_t ns_tx_assoc;		/* [re]associations */
	uint32_t ns_tx_assoc_fail;	/* [re]association failures */
	uint32_t ns_tx_auth;		/* [re]authentications */
	uint32_t ns_tx_auth_fail;	/* [re]authentication failures*/
	uint32_t ns_tx_deauth;		/* deauthentications */
	uint32_t ns_tx_deauth_code;	/* last deauth reason */
	uint32_t ns_tx_disassoc;	/* disassociations */
	uint32_t ns_tx_disassoc_code;	/* last disassociation reason */
	uint32_t ns_psq_drops;		/* power save queue drops */
	uint32_t ns_rx_action;         /* rx action */
	uint32_t ns_tx_action;
	/*
	 * Next few fields track the corresponding entry in struct net_device_stats,
	 * but here for each associated node
	 */
	uint32_t ns_rx_errors;
	uint32_t ns_tx_errors;
	uint32_t ns_rx_dropped;
	uint32_t ns_tx_dropped;
	uint32_t ns_tx_hwretry;
	uint32_t ns_tx_swretry;
	uint32_t ns_tx_restrict_enter;
	uint32_t ns_tx_restrict_exit;
	uint32_t ns_tx_restrict_enter_prev;
	uint32_t ns_tx_restrict_exit_prev;
	uint32_t ns_tx_subf_mode;
	/*
	 * The number of dropped data packets failed to transmit through
	 * wireless media for each traffic category(TC).
	 */
	uint32_t ns_tx_wifi_drop[WME_AC_NUM];
	uint32_t ns_tx_wifi_drop_xattempts[WME_AC_NUM];
	uint32_t ns_tx_wifi_sent[WME_AC_NUM];
	uint32_t ns_tx_allretries;
	uint32_t ns_tx_exceed_retry;
	uint32_t ns_tx_retried_succ;
	uint32_t ns_tx_mretried_succ;
	uint32_t ns_tx_retried;
	uint32_t ns_tx_retried_pct;

	uint32_t ns_tx_airtime;
	uint32_t ns_tx_airtime_user;

	uint32_t ns_ap_isolation_dropped;
	uint32_t ns_rx_fragment_pkts;
	uint32_t ns_rx_vlan_pkts;

#define QTN_STAT_BKT_MAX                        20
        uint32_t ns_stat_bkt[QTN_STAT_BKT_MAX];
};

/*
 * Summary statistics.
 */
struct ieee80211_stats {
	uint32_t is_rx_badversion;	/* rx frame with bad version */
	uint32_t is_rx_tooshort;	/* rx frame too short */
	uint32_t is_rx_tooshort_cnt;	/* rx frame too short accumulated */
	uint32_t is_rx_wrongbss;	/* rx from wrong bssid */
	uint32_t is_rx_otherbss;	/* rx from another bssid */
	uint32_t is_rx_dup;		/* rx discard due to it's a dup */
	uint32_t is_rx_wrongdir;	/* rx w/ wrong direction */
	uint32_t is_rx_mcastecho;	/* rx discard due to of mcast echo */
	uint32_t is_rx_notassoc;	/* rx discard due to sta !assoc */
	uint32_t is_rx_noprivacy;	/* rx w/ wep but privacy off */
	uint32_t is_rx_unencrypted;	/* rx w/o wep and privacy on */
	uint32_t is_rx_wepfail;		/* rx wep processing failed */
	uint32_t is_rx_decap;		/* rx decapsulation failed */
	uint32_t is_rx_mgtdiscard;	/* rx discard mgt frames */
	uint32_t is_rx_ctl;		/* rx discard ctrl frames */
	uint32_t is_rx_rstoobig;	/* rx rate set truncated */
	uint32_t is_rx_elem_missing;	/* rx required element missing*/
	uint32_t is_rx_elem_toobig;	/* rx element too big */
	uint32_t is_rx_elem_toosmall;	/* rx element too small */
	uint32_t is_rx_elem_unknown;	/* rx element unknown */
	uint32_t is_rx_badchan;	/* rx frame w/ invalid chan */
	uint32_t is_rx_chanmismatch;	/* rx frame chan mismatch */
	uint32_t is_rx_nodealloc;	/* rx frame dropped */
	uint32_t is_rx_ssidmismatch;	/* rx frame ssid mismatch  */
	uint32_t is_rx_auth_unsupported;/* rx w/ unsupported auth alg */
	uint32_t is_rx_auth_fail;	/* rx sta auth failure */
	uint32_t is_rx_auth_countermeasures;/* rx auth discard due to CM */
	uint32_t is_rx_assoc_bss;	/* rx assoc from wrong bssid */
#if defined(CONFIG_QTN_BSA_SUPPORT)
	uint32_t is_rx_assoc_bsa;	/* rx assoc from STA blacklisted by BSA */
#endif
	uint32_t is_rx_assoc_notauth;	/* rx assoc w/o auth */
	uint32_t is_rx_assoc_capmismatch;/* rx assoc w/ cap mismatch */
	uint32_t is_rx_assoc_norate;	/* rx assoc w/ no rate match */
	uint32_t is_rx_assoc_badwpaie;	/* rx assoc w/ bad WPA IE */
	uint32_t is_rx_deauth;		/* rx deauthentication */
	uint32_t is_rx_disassoc;	/* rx disassociation */
	uint32_t is_rx_action;         /* rx action mgt */
	uint32_t is_rx_badsubtype;	/* rx frame w/ unknown subtype*/
	uint32_t is_rx_nobuf;		/* rx failed for lack of buf */
	uint32_t is_rx_decryptcrc;	/* rx decrypt failed on crc */
	uint32_t is_rx_bad_auth;	/* rx bad auth request */
	uint32_t is_rx_unauth;		/* rx on unauthorized port */
	uint32_t is_rx_badkeyid;	/* rx w/ incorrect keyid */
	uint32_t is_rx_ccmpreplay;	/* rx seq# violation (CCMP) */
	uint32_t is_rx_ccmpformat;	/* rx format bad (CCMP) */
	uint32_t is_rx_ccmpmic;		/* rx MIC check failed (CCMP) */
	uint32_t is_rx_tkipreplay;	/* rx seq# violation (TKIP) */
	uint32_t is_rx_tkipformat;	/* rx format bad (TKIP) */
	uint32_t is_rx_tkipmic;		/* rx MIC check failed (TKIP) */
	uint32_t is_rx_tkipicv;		/* rx ICV check failed (TKIP) */
	uint32_t is_rx_badcipher;	/* rx failed due to of key type */
	uint32_t is_rx_nocipherctx;	/* rx failed due to key !setup */
	uint32_t is_rx_acl;		/* rx discard due to of acl policy */
	uint32_t is_tx_nobuf;		/* tx failed for lack of buf */
	uint32_t is_tx_nonode;		/* tx failed for no node */
	uint32_t is_tx_unknownmgt;	/* tx of unknown mgt frame */
	uint32_t is_tx_badcipher;	/* tx failed due to of key type */
	uint32_t is_tx_nodefkey;	/* tx failed due to no defkey */
	uint32_t is_tx_noheadroom;	/* tx failed due to no space */
	uint32_t is_tx_unauth;		/* tx on unauthorized port */
	uint32_t is_scan_active;	/* active scans started */
	uint32_t is_scan_passive;	/* passive scans started */
	uint32_t is_node_timeout;	/* nodes timed out inactivity */
	uint32_t is_crypto_nomem;	/* no memory for crypto ctx */
	uint32_t is_crypto_tkip;	/* tkip crypto done in s/w */
	uint32_t is_crypto_tkipenmic;	/* tkip en-MIC done in s/w */
	uint32_t is_crypto_tkipdemic;	/* tkip de-MIC done in s/w */
	uint32_t is_crypto_tkipcm;	/* tkip counter measures */
	uint32_t is_crypto_ccmp;	/* ccmp crypto done in s/w */
	uint32_t is_crypto_wep;		/* wep crypto done in s/w */
	uint32_t is_crypto_setkey_cipher;/* cipher rejected key */
	uint32_t is_crypto_setkey_nokey;/* no key index for setkey */
	uint32_t is_crypto_delkey;	/* driver key delete failed */
	uint32_t is_crypto_badcipher;	/* unknown cipher */
	uint32_t is_crypto_nocipher;	/* cipher not available */
	uint32_t is_crypto_attachfail;	/* cipher attach failed */
	uint32_t is_crypto_swfallback;	/* cipher fallback to s/w */
	uint32_t is_crypto_keyfail;	/* driver key alloc failed */
	uint32_t is_crypto_enmicfail;	/* en-MIC failed */
	uint32_t is_ibss_capmismatch;	/* merge failed-cap mismatch */
	uint32_t is_ps_unassoc;	/* ps-poll for unassoc. sta */
	uint32_t is_ps_badaid;		/* ps-poll w/ incorrect aid */
	uint32_t is_ps_qempty;		/* ps-poll w/ nothing to send */
	uint32_t is_rx_assoc_nohtcap;	/* HT capabilities mismatch */
	uint32_t is_rx_assoc_tkiphtreject; /* rx assoc requesting TKIP and HT capabilities */
	uint32_t is_rx_assoc_toomany;	/* reach assoc limit */
	uint32_t is_rx_ps_unauth;	/* ps-poll for un-authenticated STA */
	uint32_t is_rx_corrupt_vht_bfrpt;
	uint32_t is_rx_wrong_type;
	uint32_t is_he_no_supported;	/* HE is not supported */
} __packed;

#define QTN_PARAM_WME_AC_M		0x00000003
#define QTN_PARAM_WME_AC_S		30
#define QTN_PARAM_TX_AGG_HOLD_TIME_M	0x3FFFFFFF
#define QTN_PARAM_MAX_TX_AGG_HOLD_TIME	0x3FFFFFFF

/*
 * Max size of optional information elements.  We artificially
 * constrain this; it's limited only by the max frame size (and
 * the max parameter size of the wireless extensions).
 */
#define	IEEE80211_MAX_OPT_IE	256
#define	IEEE80211_MAX_GEN_IE	64

/* Get request for driver capabilities */
struct ieee80211req_drv_cap {
#define QTN_DRV_CAP_FLAG_SUPPORT_WPS	BIT(0)
	uint32_t cap_flags;

	u_int8_t ext_cap_ie[IEEE80211_EXTCAP_IE_LEN + 2];
};

/*
 * WPA/RSN get/set key request.  Specify the key/cipher
 * type and whether the key is to be used for sending and/or
 * receiving.  The key index should be set only when working
 * with global keys (use IEEE80211_KEYIX_NONE for ``no index'').
 * Otherwise a unicast/pairwise key is specified by the bssid
 * (on a station) or mac address (on an ap).  They key length
 * must include any MIC key data; otherwise it should be no
 more than IEEE80211_KEYBUF_SIZE.
 */
struct ieee80211req_key {
	uint8_t ik_type;		/* key/cipher type */
	uint8_t ik_pad;
	uint8_t ik_keyix;		/* key index */
	uint8_t ik_keylen;		/* key length in bytes */
	uint8_t ik_flags;
/* NB: IEEE80211_KEY_XMIT and IEEE80211_KEY_RECV defined elsewhere */
#define	IEEE80211_KEY_DEFAULT	0x80	/* default xmit key */
	uint8_t ik_macaddr[IEEE80211_ADDR_LEN];
	uint16_t ik_vlan;
	uint64_t ik_keyrsc;		/* key receive sequence counter */
	uint64_t ik_keytsc;		/* key transmit sequence counter */
	uint8_t ik_keydata[IEEE80211_KEYBUF_SIZE+IEEE80211_MICBUF_SIZE];
};

/*
 * Delete a key either by index or address.  Set the index
 * to IEEE80211_KEYIX_NONE when deleting a unicast key.
 */
struct ieee80211req_del_key {
	uint8_t idk_keyix;		/* key index */
	uint8_t idk_macaddr[IEEE80211_ADDR_LEN];
};

/*
 * MLME state manipulation request.  IEEE80211_MLME_ASSOC
 * only makes sense when operating as a station.  The other
 * requests can be used when operating as a station or an
 * ap (to effect a station).
 */
struct ieee80211req_mlme {
	uint8_t im_op;			/* operation to perform */
#define	IEEE80211_MLME_ASSOC		1	/* associate station */
#define	IEEE80211_MLME_DISASSOC		2	/* disassociate station */
#define	IEEE80211_MLME_DEAUTH		3	/* deauthenticate station */
#define	IEEE80211_MLME_AUTHORIZE	4	/* authorize station */
#define	IEEE80211_MLME_UNAUTHORIZE	5	/* unauthorize station */
#define IEEE80211_MLME_CLEAR_STATS	6	/* clear station statistic */
#define IEEE80211_MLME_AUTH             8       /* authenticate */
	uint8_t im_ssid_len;		/* length of optional ssid */
	uint16_t im_reason;		/* 802.11 reason code */
#define IEEE80211_MLME_REASON_FLAG_BSA	0x0800	/* unlikely to clash with future MLME codes */
	uint8_t im_macaddr[IEEE80211_ADDR_LEN];
	uint8_t im_ssid[IEEE80211_NWID_LEN];
	uint32_t im_timeout;
	uint8_t	im_optie[IEEE80211_MAX_OPT_IE];
	uint16_t im_optie_len;
	uint32_t im_param1;
};

struct ieee80211req_brcm {
	uint8_t ib_op;				/* operation to perform */
#define IEEE80211REQ_BRCM_INFO        0       /* BRCM client information */
#define IEEE80211REQ_BRCM_PKT         1       /* BRCM pkt from ap to client */
	uint8_t ib_macaddr[IEEE80211_ADDR_LEN];
	int ib_rssi;
	uint32_t ib_rxglitch;
	uint8_t *ib_pkt;
	int32_t ib_pkt_len;
};

#define QTN_CHAN_AVAIL_STATUS_TO_STR	{"", "Non-Available", "Available",\
					"", "Not-Available-Radar-Detected", "",\
					"", "", "Not-Available-CAC-Required"}

struct ieee80211req_scs_chan_list {
	uint8_t iscl_num;
	uint8_t iscl_flag;
	uint8_t iscl_chans[IEEE80211_CHAN_MAX];
};

#define IEEE80211REQ_SCS_REPORT_CHAN_NUM    32

struct ieee80211req_scs_currchan_rpt {
	uint8_t iscr_curchan;
	uint16_t iscr_cca_try;
	uint16_t iscr_cca_idle;
	uint16_t iscr_cca_busy;
	uint16_t iscr_cca_intf;
	uint16_t iscr_cca_tx;
	uint16_t iscr_tx_ms;
	uint16_t iscr_rx_ms;
	uint32_t iscr_pmbl;
};

struct ieee80211req_scs_ranking_rpt_chan {
	uint8_t isrc_chan;
	uint8_t isrc_dfs;
	uint8_t isrc_txpwr;
	int32_t isrc_metric;
	uint32_t isrc_metric_age;
	/* scs part */
	uint16_t isrc_cca_intf;
	uint32_t isrc_pmbl_ap;
	uint32_t isrc_pmbl_sta;
	/* initial channel selection part */
	unsigned int isrc_numbeacons;
	int isrc_cci;
	int isrc_aci;
	/* channel usage */
	uint32_t isrc_duration;
	uint32_t isrc_times;
	uint8_t isrc_chan_avail_status;
};

struct ieee80211req_scs_ranking_rpt {
	uint8_t isr_num;
	struct ieee80211req_scs_ranking_rpt_chan isr_chans[IEEE80211REQ_SCS_REPORT_CHAN_NUM];
};

struct ieee80211req_scs_interference_rpt_chan {
	uint8_t isrc_chan;
	uint16_t isrc_cca_intf_20;
	uint16_t isrc_cca_intf_40;
	uint16_t isrc_cca_intf_80;
};
struct ieee80211req_scs_interference_rpt {
	uint8_t isr_num;
	struct ieee80211req_scs_interference_rpt_chan isr_chans[IEEE80211REQ_SCS_REPORT_CHAN_NUM];
};

struct ieee80211req_scs_score_rpt_chan {
	uint8_t isrc_chan;
	uint8_t isrc_score;
};
struct ieee80211req_scs_score_rpt {
	uint8_t isr_num;
	struct ieee80211req_scs_score_rpt_chan isr_chans[IEEE80211REQ_SCS_REPORT_CHAN_NUM];
};

#define SCS_MAX_TXTIME_COMP_INDEX	8
#define SCS_MAX_RXTIME_COMP_INDEX	8
#define SCS_MAX_TDLSTIME_COMP_INDEX	8
/*
 * Restrictions:
 *   this structure must be kept in sync with ieee80211_scs
 */
enum qscs_cfg_param_e {
	SCS_SMPL_DWELL_TIME = 0,
	SCS_SAMPLE_INTV,
	SCS_THRSHLD_SMPL_PKTNUM,
	SCS_THRSHLD_SMPL_AIRTIME,
	SCS_THRSHLD_ATTEN_INC,
	SCS_THRSHLD_DFS_REENTRY,
	SCS_THRSHLD_DFS_REENTRY_MINRATE,
	SCS_THRSHLD_DFS_REENTRY_INTF,
	SCS_THRSHLD_LOADED,
	SCS_THRSHLD_AGING_NOR,
	SCS_THRSHLD_AGING_DFSREENT,
	SCS_ENABLE,
	SCS_DEBUG_ENABLE,
	SCS_SMPL_ENABLE,
	SCS_SMPL_ENABLE_ALONE,
	SCS_REPORT_ONLY,
	SCS_CCA_IDLE_THRSHLD,
	SCS_CCA_INTF_HI_THRSHLD,
	SCS_CCA_INTF_LO_THRSHLD,
	SCS_CCA_INTF_RATIO,
	SCS_CCA_INTF_DFS_MARGIN,
	SCS_PMBL_ERR_THRSHLD,
	SCS_CCA_SAMPLE_DUR,
	SCS_CCA_INTF_SMTH_NOXP,
	SCS_CCA_INTF_SMTH_XPED,
	SCS_RSSI_SMTH_UP,
	SCS_RSSI_SMTH_DOWN,
	SCS_CHAN_MTRC_MRGN,
	SCS_ATTEN_ADJUST,
	SCS_ATTEN_SW_ENABLE,
	SCS_PMBL_ERR_SMTH_FCTR,
	SCS_PMBL_ERR_RANGE,
	SCS_PMBL_ERR_MAPPED_INTF_RANGE,
	SCS_SP_WF,
	SCS_LP_WF,
	SCS_PMP_RPT_CCA_SMTH_FCTR,
	SCS_PMP_RX_TIME_SMTH_FCTR,
	SCS_PMP_TX_TIME_SMTH_FCTR,
	SCS_PMP_STATS_STABLE_PERCENT,
	SCS_PMP_STATS_STABLE_RANGE,
	SCS_PMP_STATS_CLEAR_INTERVAL,
	SCS_AS_RX_TIME_SMTH_FCTR,
	SCS_AS_TX_TIME_SMTH_FCTR,
	SCS_CCA_IDLE_SMTH_FCTR,
	SCS_TX_TIME_COMPENSTATION_START,
	SCS_TX_TIME_COMPENSTATION_END = SCS_TX_TIME_COMPENSTATION_START+SCS_MAX_TXTIME_COMP_INDEX-1,
	SCS_RX_TIME_COMPENSTATION_START,
	SCS_RX_TIME_COMPENSTATION_END = SCS_RX_TIME_COMPENSTATION_START+SCS_MAX_RXTIME_COMP_INDEX-1,
	SCS_TDLS_TIME_COMPENSTATION_START,
	SCS_TDLS_TIME_COMPENSTATION_END = SCS_TDLS_TIME_COMPENSTATION_START+SCS_MAX_TDLSTIME_COMP_INDEX-1,
	SCS_LEAVE_DFS_CHAN_MTRC_MRGN,
	SCS_SAMPLE_TYPE,
	SCS_BURST_ENABLE,
	SCS_BURST_WINDOW,
	SCS_BURST_THRESH,
	SCS_BURST_PAUSE_TIME,
	SCS_BURST_FORCE_SWITCH,
	SCS_NAC_MONITOR_MODE,
	SCS_CHECK_BAND_MRGN,
	SCS_OUT_OF_BAND_MRGN,
	SCS_PARAM_MAX,
};

struct ieee80211req_scs_param_rpt {
	uint32_t cfg_param;
	uint32_t signed_param_flag;
};

struct ieee80211req_scs {
	uint32_t is_op;
#define IEEE80211REQ_SCS_ID_UNKNOWN               0
#define IEEE80211REQ_SCS_FLAG_GET                 0x80000000
#define IEEE80211REQ_SCS_GET_CURRCHAN_RPT         (IEEE80211REQ_SCS_FLAG_GET | 1)
#define IEEE80211REQ_SCS_GET_INIT_RANKING_RPT     (IEEE80211REQ_SCS_FLAG_GET | 2)
#define IEEE80211REQ_SCS_GET_RANKING_RPT          (IEEE80211REQ_SCS_FLAG_GET | 3)
#define IEEE80211REQ_SCS_GET_PARAM_RPT            (IEEE80211REQ_SCS_FLAG_GET | 4)
#define IEEE80211REQ_SCS_GET_SCORE_RPT            (IEEE80211REQ_SCS_FLAG_GET | 5)
#define IEEE80211REQ_SCS_GET_INTERFERENCE_RPT     (IEEE80211REQ_SCS_FLAG_GET | 6)
#define IEEE80211REQ_SCS_GET_CHAN_POOL            (IEEE80211REQ_SCS_FLAG_GET | 7)
#define IEEE80211REQ_SCS_SET_CHAN_POOL            (1)
#define IEEE80211REQ_SCS_GET_ACTIVE_CHAN_LIST     (IEEE80211REQ_SCS_FLAG_GET | 8)

#define IEEE80211REQ_SCS_FLAG_SET                 0x40000000
#define IEEE80211REQ_SCS_SET_ACTIVE_CHAN_LIST     (IEEE80211REQ_SCS_FLAG_SET | 1)

	uint32_t *is_status;                  /* SCS specific reason for ioctl failure */
#define IEEE80211REQ_SCS_RESULT_OK                    0
#define IEEE80211REQ_SCS_RESULT_SYSCALL_ERR           1
#define IEEE80211REQ_SCS_RESULT_SCS_DISABLED          2
#define IEEE80211REQ_SCS_RESULT_NO_VAP_RUNNING        3
#define IEEE80211REQ_SCS_RESULT_NOT_EVALUATED         4        /* channel ranking not evaluated */
#define IEEE80211REQ_SCS_RESULT_TMP_UNAVAILABLE       5        /* when channel switch or param change */
#define IEEE80211REQ_SCS_RESULT_APMODE_ONLY           6
#define IEEE80211REQ_SCS_RESULT_AUTOCHAN_DISABLED     7
#define IEEE80211REQ_SCS_RESULT_RESOURCE_NOT_AVAILABLE     8
#define IEEE80211REQ_SCS_RESULT_CH_NOT_AVAILABLE      9
	uint8_t *is_data;
	int32_t is_data_len;
};

struct ieeee80211_dscp2ac {
	uint8_t dscp[IP_DSCP_NUM];
	uint8_t list_len;
	uint8_t ac;
};
/*
 * MAC ACL operations.
 */
enum {
	IEEE80211_MACCMD_POLICY_OPEN	= 0,	/* set policy: no ACL's */
	IEEE80211_MACCMD_POLICY_ALLOW	= 1,	/* set policy: allow traffic */
	IEEE80211_MACCMD_POLICY_DENY	= 2,	/* set policy: deny traffic */
	IEEE80211_MACCMD_FLUSH		= 3,	/* flush ACL database */
	IEEE80211_MACCMD_DETACH		= 4,	/* detach ACL policy */
};

/*
 * Set the active channel list.  Note this list is
 * intersected with the available channel list in
 * calculating the set of channels actually used in
 * scanning.
 */
struct ieee80211req_chanlist {
	uint8_t ic_channels[IEEE80211_CHAN_BYTES_EXT];
};

/*
 * Get the active channel list info.
 */
struct ieee80211req_chaninfo {
	uint32_t ic_nchans;
	uint16_t ic_ieee[IEEE80211_CHAN_MAX];       /* IEEE channel number */
	uint16_t ic_freq[IEEE80211_CHAN_MAX];       /* setting in Mhz */
	uint32_t ic_flags[IEEE80211_CHAN_MAX];      /* state flags */
};

/*
 * Set the active channel list for 20Mhz, 40Mhz and 80Mhz
 */
struct ieee80211_active_chanlist {
	u_int8_t bw;
	u_int8_t channels[IEEE80211_CHAN_BYTES_EXT];
};

/*
 * Set or Get the inactive channel list
 */
struct ieee80211_inactive_chanlist {
	u_int8_t channels[IEEE80211_CHAN_MAX];
};

enum ieee80211_local_chan_bw
{
	QTN_BW_BIT_20M	= 0x1,
	QTN_BW_BIT_40M	= 0x2,
	QTN_BW_BIT_80M	= 0x4,
	QTN_BW_BIT_160M	= 0x8,
	QTN_BW_BIT_ALL	= 0xF,
};

/*
 * Set or get the disabled channel list
 */
struct ieeee80211_disabled_chanlist {
	uint16_t chan[IEEE80211_CHAN_MAX_EXT];
	uint32_t list_len;
	uint8_t flag;	/*0: disable 1: enable*/
	uint8_t dir;	/*0: set 1: get*/
};

enum ieee80211_chan_control_dir
{
	SET_CHAN_DISABLED = 0,
	GET_CHAN_DISABLED = 1,
};
/*
 * NOTE: This enum should be in sync with 'enum qcsapi_chlist_flags'
 */
enum ieee80211_chlist_flags {
	/**
	 * List of available channels.
	 */
	IEEE80211_CHLIST_FLAG_AVAILABLE = 0x00000001,
	/**
	 * List of disabled channels.
	 */
	IEEE80211_CHLIST_FLAG_DISABLED	= 0x00000002,
	/**
	 * List of scan channels.
	 */
	IEEE80211_CHLIST_FLAG_SCAN	= 0x00000004,
	/**
	 * List of active channels.
	 */
	IEEE80211_CHLIST_FLAG_ACTIVE	= 0x00000008,
	/**
	 * List of OCAC off-channels.
	 */
	IEEE80211_CHLIST_FLAG_OCAC_OFF	= 0x00000010,
};

enum ieee80211_chlist_operation {
	IEEE80211_CHLIST_OPER_CLEAR = 0,
	IEEE80211_CHLIST_OPER_SET   = 1,
};

/*
 * Used for operation on a list of channels
 */
struct ieeee80211_chanlist_data {
	uint16_t chan[IEEE80211_CHAN_MAX_EXT];
	uint32_t list_len;
	uint32_t flags;		/* enum ieee80211_chlist_flags */
	uint32_t oper;		/* enum ieee80211_chlist_operation */
};

/*
 * Retrieve the WPA/RSN information element for an associated station.
 */
struct ieee80211req_wpaie {
	uint8_t	wpa_macaddr[IEEE80211_ADDR_LEN];
	uint8_t	wpa_ie[IEEE80211_MAX_OPT_IE];
	uint8_t	rsn_ie[IEEE80211_MAX_OPT_IE];
	uint8_t	osen_ie[IEEE80211_MAX_OPT_IE];
	uint8_t	wps_ie[IEEE80211_MAX_OPT_IE];
	uint8_t	qtn_pairing_ie[IEEE80211_MAX_OPT_IE];
	uint8_t	mdie[IEEE80211_MAX_OPT_IE];
	uint8_t	ftie[IEEE80211_MAX_OPT_IE];
	uint8_t	owe_dh[IEEE80211_MAX_GEN_IE];
	uint8_t	rsnxe[IEEE80211_MAX_GEN_IE];
#define QTN_PAIRING_IE_EXIST 1
#define QTN_PAIRING_IE_ABSENT 0
	uint8_t	has_pairing_ie;		/* Indicates whether Pairing IE exists in assoc req/resp */
};

/*
 * Retrieve per-node statistics.
 */
struct ieee80211req_sta_stats {
	union {
		/* NB: explicitly force 64-bit alignment */
		uint8_t macaddr[IEEE80211_ADDR_LEN];
		uint64_t pad;
	} is_u;
	struct ieee80211_nodestats is_stats;
};
/*
 * Retrieve STA Statistics(Radio measurement) information element for an associated station.
 */
struct ieee80211req_qtn_rmt_sta_stats {
	int status;
	struct ieee80211_ie_qtn_rm_sta_all rmt_sta_stats;
	struct ieee80211_ie_rm_sta_grp221	rmt_sta_stats_grp221;
};

struct ieee80211req_qtn_rmt_sta_stats_setpara {
	uint32_t flags;
	uint8_t macaddr[IEEE80211_ADDR_LEN];
};

struct ieee80211req_node_meas {
	uint8_t mac_addr[6];

	uint8_t type;
#define IOCTL_MEAS_TYPE_BASIC		0x0
#define IOCTL_MEAS_TYPE_CCA		0x1
#define IOCTL_MEAS_TYPE_RPI		0x2
#define IOCTL_MEAS_TYPE_CHAN_LOAD	0x3
#define IOCTL_MEAS_TYPE_NOISE_HIS	0x4
#define IOCTL_MEAS_TYPE_BEACON		0x5
#define IOCTL_MEAS_TYPE_FRAME		0x6
#define IOCTL_MEAS_TYPE_CAT		0x7
#define IOCTL_MEAS_TYPE_MUL_DIAG	0x8
#define IOCTL_MEAS_TYPE_LINK		0x9
#define IOCTL_MEAS_TYPE_NEIGHBOR	0xA

	struct _ioctl_basic {
		uint16_t start_offset_ms;
		uint16_t duration_ms;
		uint8_t channel;
	} ioctl_basic;
	struct _ioctl_cca {
		uint16_t start_offset_ms;
		uint16_t duration_ms;
		uint8_t channel;
	} ioctl_cca;
	struct _ioctl_rpi {
		uint16_t start_offset_ms;
		uint16_t duration_ms;
		uint8_t channel;
	} ioctl_rpi;
	struct _ioctl_chan_load {
		uint16_t duration_ms;
		uint8_t channel;
	} ioctl_chan_load;
	struct _ioctl_noise_his {
		uint16_t duration_ms;
		uint8_t channel;
	} ioctl_noise_his;
	struct _ioctl_beacon {
		uint8_t op_class;
		uint8_t channel;
		uint16_t duration_ms;
		uint8_t mode;
		uint8_t bssid[IEEE80211_ADDR_LEN];
	} ioctl_beacon;
	struct _ioctl_frame {
		uint8_t op_class;
		uint8_t channel;
		uint16_t duration_ms;
		uint8_t type;
		uint8_t mac_address[IEEE80211_ADDR_LEN];
	} ioctl_frame;
	struct _ioctl_tran_stream_cat {
		uint16_t duration_ms;
		uint8_t peer_sta[IEEE80211_ADDR_LEN];
		uint8_t tid;
		uint8_t bin0;
	} ioctl_tran_stream_cat;
	struct _ioctl_multicast_diag {
		uint16_t duration_ms;
		uint8_t group_mac[IEEE80211_ADDR_LEN];
	} ioctl_multicast_diag;
};

struct ieee80211req_node_tpc {
	uint8_t	mac_addr[6];
};

struct ieee80211req_node_info {
	uint8_t	req_type;
#define IOCTL_REQ_MEASUREMENT	0x0
#define IOCTL_REQ_TPC		0x1
	union {
		struct ieee80211req_node_meas	req_node_meas;
		struct ieee80211req_node_tpc	req_node_tpc;
	} u_req_info;
};

struct ieee80211_ioctl_neighbor_report_item {
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint32_t bssid_info;
	uint8_t operating_class;
	uint8_t channel;
	uint8_t phy_type;
};
#define IEEE80211_MAX_NEIGHBOR_REPORT_ITEM 3

struct ieee80211rep_node_meas_result {
	uint8_t	status;
#define IOCTL_MEAS_STATUS_SUCC		0
#define IOCTL_MEAS_STATUS_TIMEOUT	1
#define IOCTL_MEAS_STATUS_NODELEAVE	2
#define IOCTL_MEAS_STATUS_STOP		3

	uint8_t report_mode;
#define IOCTL_MEAS_REP_OK	(0)
#define IOCTL_MEAS_REP_LATE	(1 << 0)
#define IOCTL_MEAS_REP_INCAP	(1 << 1)
#define IOCTL_MEAS_REP_REFUSE	(1 << 2)
#define IOCTL_MEAS_REP_MASK	(0x07)

	union {
		uint8_t	basic;
		uint8_t	cca;
		uint8_t	rpi[8];
		uint8_t chan_load;
		struct {
			uint8_t antenna_id;
			uint8_t anpi;
			uint8_t ipi[11];
		} noise_his;
		struct {
			uint8_t reported_frame_info;
			uint8_t rcpi;
			uint8_t rsni;
			uint8_t bssid[IEEE80211_ADDR_LEN];
			uint8_t antenna_id;
			uint32_t parent_tsf;
		} beacon;
		struct {
			uint32_t sub_ele_report;
			uint8_t ta[IEEE80211_ADDR_LEN];
			uint8_t bssid[IEEE80211_ADDR_LEN];
			uint8_t phy_type;
			uint8_t avg_rcpi;
			uint8_t last_rsni;
			uint8_t last_rcpi;
			uint8_t antenna_id;
			uint16_t frame_count;
		} frame;
		struct {
			uint8_t reason;
			uint32_t tran_msdu_cnt;
			uint32_t msdu_discard_cnt;
			uint32_t msdu_fail_cnt;
			uint32_t msdu_mul_retry_cnt;
			uint32_t qos_lost_cnt;
			uint32_t avg_queue_delay;
			uint32_t avg_tran_delay;
			uint8_t bin0_range;
			uint32_t bins[6];
		} tran_stream_cat;
		struct {
			uint8_t reason;
			uint32_t mul_rec_msdu_cnt;
			uint16_t first_seq_num;
			uint16_t last_seq_num;
			uint16_t mul_rate;
		} multicast_diag;
		struct {
			struct {
				int8_t tx_power;
				int8_t link_margin;
			} tpc_report;
			uint8_t recv_antenna_id;
			uint8_t tran_antenna_id;
			uint8_t rcpi;
			uint8_t rsni;
		} link_measure;
		struct {
			uint8_t item_num;
			struct ieee80211_ioctl_neighbor_report_item item[IEEE80211_MAX_NEIGHBOR_REPORT_ITEM];
		} neighbor_report;
	} u_data;
};

struct ieee80211rep_node_tpc_result {
	uint8_t status;
	int8_t	tx_power;
	int8_t	link_margin;
};

union ieee80211rep_node_info {
	struct ieee80211rep_node_meas_result	meas_result;
	struct ieee80211rep_node_tpc_result	tpc_result;
};

typedef struct assoc_info_report {
	uint64_t	ai_rx_bytes;
	uint64_t	ai_tx_bytes;
	uint32_t	ai_rx_packets;
	uint32_t	ai_tx_packets;
	uint32_t	ai_rx_errors;
	uint32_t	ai_tx_errors;
	uint32_t	ai_rx_dropped;
	uint32_t	ai_tx_dropped;
	uint32_t	ai_tx_wifi_drop[WME_AC_NUM];
	uint32_t	ai_tx_wifi_sent[WME_AC_NUM];
	uint32_t	ai_tx_ucast;
	uint32_t	ai_rx_ucast;
	uint32_t	ai_rx_mgmt;
	uint32_t	ai_rx_ctl;
	uint32_t	ai_tx_mgmt;
	uint32_t	ai_tx_mcast;
	uint32_t	ai_rx_mcast;
	uint32_t	ai_tx_bcast;
	uint32_t	ai_rx_bcast;
	uint32_t	ai_tx_failed;
	uint32_t	ai_tx_allretries;	/*all retransmissions*/
	uint32_t	ai_tx_exceed_retry;	/*packets exceed retry limit*/
	uint32_t	ai_tx_retried;		/*packets that retried, multiple retries count as one*/
	uint32_t	ai_tx_retried_percent;	/*percentage of packets that retried*/
	uint32_t	ai_time_associated;	/*Unit: seconds*/
	uint16_t	ai_assoc_id;
	uint16_t	ai_link_quality;
	uint16_t	ai_tx_phy_rate;
	uint16_t	ai_rx_phy_rate;
	uint32_t	ai_achievable_tx_phy_rate;
	uint32_t	ai_achievable_rx_phy_rate;
	u_int32_t	ai_rx_fragment_pkts;
	u_int32_t	ai_rx_vlan_pkts;
	uint8_t		ai_mac_addr[IEEE80211_ADDR_LEN];
	int32_t		ai_rssi;
	int32_t		ai_smthd_rssi;
	int32_t		ai_snr;
	int32_t		ai_max_queued;
	uint8_t         ai_bw;
	uint8_t		ai_tx_mcs;
	uint8_t		ai_rx_mcs;
	uint8_t		ai_auth;
	char		ai_ifname[IFNAMSIZ];
	uint32_t	ai_ip_addr;
	int32_t		ai_hw_noise;
	uint32_t	ai_is_qtn_node;
	int		ai_meas_rssi_dbm;
	int		ai_meas_rssi_dbm_min;
	int		ai_meas_rssi_dbm_max;
	uint32_t	ai_meas_rssi_seq;
	uint32_t	ai_time_idle;	/* Time since last data/ack pkt rx'd from sta (in secs) */
	uint8_t         ai_nss;         /* Number of spatial streams that the client supporting */
	uint8_t		ai_phy_mode;	/* Actual PHY mode of the client (ieee80211_wifi_mode) */
	uint16_t	ai_tx_ba_state;	/* Bitmap, TX Block ack state, a bit per TID. LSB is TID 0 */
	uint16_t	ai_rx_ba_state;	/* Bitmap, RX Block ack state, a bit per TID. LSB is TID 0 */
	u_int16_t       ai_ht_caps_info;
	u_int32_t       ai_vht_caps_info;
	uint8_t		ai_tx_bw;
	uint8_t		ai_rx_bw;
	uint8_t		ai_tx_nss;
	uint8_t		ai_rx_nss;
	uint8_t		ai_tx_sgi;
	uint8_t		ai_rx_sgi;
} assoc_info_report;

/*
 * Station information block; the mac address is used
 * to retrieve other data like stats, unicast key, etc.
 */
struct ieee80211req_sta_info {
	uint16_t isi_len;		/* length (mult of 4) */
	uint16_t isi_freq;		/* MHz */
	uint16_t isi_flags;		/* channel flags */
	uint16_t isi_state;		/* state flags */
	uint8_t isi_authmode;		/* authentication algorithm */
	uint8_t isi_rssi;
	uint16_t isi_capinfo;		/* capabilities */
	uint8_t isi_athflags;		/* Atheros capabilities */
	uint8_t isi_erp;		/* ERP element */
	uint8_t isi_macaddr[IEEE80211_ADDR_LEN];
	uint8_t isi_nrates;		/* negotiated rates */
	uint8_t isi_rates[IEEE80211_RATE_MAXSIZE];
	uint8_t isi_txrate;		/* index to isi_rates[] */
	uint16_t isi_ie_len;		/* IE length */
	uint16_t isi_associd;		/* assoc response */
	uint16_t isi_txpower;		/* current tx power */
	uint16_t isi_vlan;		/* vlan tag */
	uint16_t isi_txseqs[17];	/* seq to be transmitted */
	uint16_t isi_rxseqs[17];	/* seq previous for qos frames*/
	uint16_t isi_inact;		/* inactivity timer */
	uint8_t isi_uapsd;		/* UAPSD queues */
	uint8_t isi_opmode;		/* sta operating mode */
	uint16_t isi_htcap;		/* HT capabilities */

	/* XXX frag state? */
	/* variable length IE data */
};

enum {
	IEEE80211_STA_OPMODE_NORMAL,
	IEEE80211_STA_OPMODE_XR
};

/*
 * Retrieve per-station information; to retrieve all
 * specify a mac address of ff:ff:ff:ff:ff:ff.
 */
struct ieee80211req_sta_req {
	union {
		/* NB: explicitly force 64-bit alignment */
		uint8_t macaddr[IEEE80211_ADDR_LEN];
		uint64_t pad;
	} is_u;
	struct ieee80211req_sta_info info[1];	/* variable length */
};

/*
 * Get/set per-station tx power cap.
 */
struct ieee80211req_sta_txpow {
	uint8_t	it_macaddr[IEEE80211_ADDR_LEN];
	uint8_t	it_txpow;
};

/*
 * WME parameters are set and return using i_val and i_len.
 * i_val holds the value itself.  i_len specifies the AC
 * and, as appropriate, then high bit specifies whether the
 * operation is to be applied to the BSS or ourself.
 */
#define	IEEE80211_WMEPARAM_SELF	0x0000		/* parameter applies to self */
#define	IEEE80211_WMEPARAM_BSS	0x8000		/* parameter applies to BSS */
#define	IEEE80211_WMEPARAM_VAL	0x7fff		/* parameter value */

/*
 * Scan result data returned for IEEE80211_IOC_SCAN_RESULTS.
 */
struct ieee80211req_scan_result {
	uint16_t isr_len;		/* length (mult of 4) */
	uint16_t isr_freq;		/* MHz */
	uint16_t isr_flags;		/* channel flags */
	uint8_t isr_noise;
	uint8_t isr_rssi;
	uint8_t isr_intval;		/* beacon interval */
	uint16_t isr_capinfo;		/* capabilities */
	uint8_t isr_erp;		/* ERP element */
	uint8_t isr_bssid[IEEE80211_ADDR_LEN];
	uint8_t isr_nrates;
	uint8_t isr_rates[IEEE80211_RATE_MAXSIZE];
	uint8_t isr_ssid_len;		/* SSID length */
	uint8_t isr_ie_len;		/* IE length */
	uint8_t isr_pad[5];
	/* variable length SSID followed by IE data */
};

#define IEEE80211_MAX_ASSOC_HISTORY	32

struct ieee80211_assoc_history {
	uint8_t  ah_macaddr_table[IEEE80211_MAX_ASSOC_HISTORY][IEEE80211_ADDR_LEN];
	uint32_t ah_timestamp[IEEE80211_MAX_ASSOC_HISTORY];
};

/*
 * Channel switch history record.
 */
#define CSW_MAX_RECORDS_MAX 32
struct ieee80211req_csw_record {
	uint32_t cnt;
	int32_t index;
	uint32_t channel[CSW_MAX_RECORDS_MAX];
	uint32_t timestamp[CSW_MAX_RECORDS_MAX];
	uint32_t reason[CSW_MAX_RECORDS_MAX];
};

struct ieee80211req_radar_status {
	uint32_t channel;
	uint32_t flags;
	uint32_t ic_radardetected;
};

struct ieee80211req_chan_avail_status {
	int bw;
	uint8_t ic_chan_avail_status[IEEE80211_CHAN_MAX_EXT];
};

struct ieee80211req_disconn_info {
	uint32_t asso_sta_count;
	uint32_t disconn_count;
	uint32_t sequence;
	uint32_t up_time;
	uint32_t resetflag;
};

#define AP_SCAN_MAX_NUM_RATES 64
/* for qcsapi_get_results_AP_scan */
struct ieee80211_general_ap_scan_result {
	int32_t num_bitrates;
	int32_t bitrates[AP_SCAN_MAX_NUM_RATES];
	int32_t num_ap_results;
};

struct ieee80211req_aacs_ctrl {
	uint32_t cmd;
	uint32_t length;
	union content_u {
		uint8_t v_byte[256];
		uint16_t v_short[128];
		uint32_t v_int[64];
	} data;
};

/* Bit definitions for 'ap_flags' in ieee80211_per_ap_scan_result */
#define IEEE80211_AP_FLAG_BIT_SEC_ENABLE	0 /* security enabled or not */
#define IEEE80211_AP_FLAG_BIT_PROTO_11N		1 /* 11n capable */
#define IEEE80211_AP_FLAG_BIT_PROTO_11AC	2 /* 11ac capable */
#define IEEE80211_AP_FLAG_BIT_PROTO_11AX	3 /* 11ax capable */

#define IEEE80211_AP_HEOP_BSSCOLOR_ENABLED	0

struct ieee80211_per_ap_scan_result {
	int8_t		ap_addr_mac[IEEE80211_ADDR_LEN];
	int8_t		ap_name_ssid[32 + 1];
	int32_t		ap_channel_ieee;
	int32_t		ap_max_bw;
	int32_t		ap_rssi;
	int32_t		ap_noise;
	int32_t		ap_flags;
	int32_t		ap_htcap;
	int32_t		ap_vhtcap;
	uint32_t	ap_heop;
	uint8_t		ap_bsscolor;
	int16_t		ap_capinfo;
	int16_t		ap_bintval;
	uint8_t		ap_dtimperiod;
	int8_t		ap_nonerp_present;
	int8_t		ap_qhop_role;
	uint8_t		ap_ht_secoffset;
	uint8_t		ap_chan_center1;
	uint8_t		ap_chan_center2;
	uint32_t	ap_bestrate;
	uint32_t	ap_basicrates_num;
	uint32_t	ap_basicrates[AP_SCAN_MAX_NUM_RATES];	/*in 0.5Mbps*/
	uint32_t	ap_suprates_num;
	uint32_t	ap_suprates[AP_SCAN_MAX_NUM_RATES];	/*in 0.5Mbps*/
	int32_t		ap_num_genies;
	uint32_t	ap_last_beacon;
	int8_t		ap_ie_buf[0];	/* just to remind there might be WPA/RSN/WSC IEs right behind*/
};

#define IEEE80211_ACS_OBSS_CHK_MIN	0
#define IEEE80211_ACS_OBSS_CHK_MAX	1
#define IEEE80211_ACS_OBSS_CHK_EN	1

#define MAX_MACS_SIZE	1200 /* 200 macs */
/* Report results of get mac address of clients behind associated node */
struct ieee80211_mac_list {
	/**
	 * flags indicating
	 * bit 0 set means addresses are behind 4 addr node
	 * bit 1 set means results are truncated to fit to buffer
	 */
	uint32_t flags;
	/**
	 * num entries in the macaddr list below
	 */
	uint32_t num_entries;
	/**
	 * buffer to store mac addresses
	 */
	uint8_t macaddr[MAX_MACS_SIZE];
};

/*
 * IEEE80211_CONFIG_BW_TXPOWER_* - shift and mask values for Tx power configure request.
 *
 * IEEE80211_CONFIG_BW_TXPOWER_CHAN_{S,M} - encode IEEE channel number.
 * IEEE80211_CONFIG_BW_TXPOWER_FEM_PRI_{S,M} - encode FEM index for 5GHz channel and primary
 *	channel position for 2.4GHz channel.
 * IEEE80211_CONFIG_BW_TXPOWER_BF_{S,M} - encode BeamForming on/off cases.
 * IEEE80211_CONFIG_BW_TXPOWER_SS_{S,M} - encode number of Spatial Streams.
 * IEEE80211_CONFIG_BW_TXPOWER_BW_{S,M} - encode bandwidth index, one of QTN_BW_*.
 * IEEE80211_CONFIG_BW_TXPOWER_PWR_{S,M} - Tx power value.
 */
#define IEEE80211_CONFIG_BW_TXPOWER_CHAN_S		24
#define IEEE80211_CONFIG_BW_TXPOWER_CHAN_M		0xFF
#define IEEE80211_CONFIG_BW_TXPOWER_FEM_PRI_S		21
#define IEEE80211_CONFIG_BW_TXPOWER_FEM_PRI_M		0x1
#define IEEE80211_CONFIG_BW_TXPOWER_BF_S		20
#define IEEE80211_CONFIG_BW_TXPOWER_BF_M		0x1
#define IEEE80211_CONFIG_BW_TXPOWER_SS_S		16
#define IEEE80211_CONFIG_BW_TXPOWER_SS_M		0xF
#define IEEE80211_CONFIG_BW_TXPOWER_BW_S		8
#define IEEE80211_CONFIG_BW_TXPOWER_BW_M		0xF
#define IEEE80211_CONFIG_BW_TXPOWER_PWR_S		0
#define IEEE80211_CONFIG_BW_TXPOWER_PWR_M		0xFF

#define NAC_MAX_STATIONS 128
/* non associated clients information */
struct nac_info_entry {
	uint64_t	nac_timestamp;   /* time stamp of last packet received */
	uint64_t	nac_age;   /* age of last packet received */
	int8_t		nac_avg_rssi; /*average rssi in dBm */
	uint8_t		nac_channel;  /* channel on which last seen */
	uint8_t		nac_packet_type; /* packet type last transmitted */
	uint8_t		nac_txmac[IEEE80211_ADDR_LEN]; /* mac address */
};
struct ieee80211_nac_stats_report {
	uint8_t	nac_entries; /* number of entries filled, upto NAC_MAX_STATIONS */
	struct nac_info_entry nac_stats[NAC_MAX_STATIONS];
};

#define QTN_FREQ_RANGE_MAX_NUM	128

struct	ieee80211_freq
{
	int32_t m;		/* Mantissa */
	int16_t e;		/* Exponent */
	uint8_t i;		/* List index (when in range struct) */
	uint8_t flags;		/* Flags (fixed/auto) */
};

struct ieee80211_freq_ranges {
	uint8_t num_freq;
	struct ieee80211_freq freq[QTN_FREQ_RANGE_MAX_NUM];
};

struct ieee80211_tgen_params {
	int32_t	spi_num;	/* always present - SPI */
	uint32_t freq_div5;	/* (frequency / 5) Hz - used to set and get */
	uint32_t chains;	/* chain mask for enables */
	int32_t enable;		/* gain enable */
	int32_t agc;		/* agc gain */
	int32_t rdiv;		/* rdiv gain */
};

#define	IEEE80211_SECCHAN_OFFSET_SCA	0
#define	IEEE80211_SECCHAN_OFFSET_SCB	1
#define	IEEE80211_SECCHAN_OFFSET_AUTO	2
#define	IEEE80211_SECCHAN_OFFSET_AUTO_RESCAN	3
#define IEEE80211_SECCHAN_OFFSET_MIN	IEEE80211_SECCHAN_OFFSET_SCA
#define IEEE80211_SECCHAN_OFFSET_MAX	IEEE80211_SECCHAN_OFFSET_AUTO_RESCAN

#define IEEE80211_MAX_TID_ACTIVE_THRESH 255
#define IEEE80211_MAX_NODE_INACT_THRESH 255

#ifdef __FreeBSD__
/*
 * FreeBSD-style ioctls.
 */
/* the first member must be matched with struct ifreq */
struct ieee80211req {
	char i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	uint16_t i_type;	/* req type */
	int16_t	i_val;		/* Index or simple value */
	int16_t	i_len;		/* Index or simple value */
	void *i_data;		/* Extra data */
};
#define	SIOCS80211		 _IOW('i', 234, struct ieee80211req)
#define	SIOCG80211		_IOWR('i', 235, struct ieee80211req)
#define	SIOCG80211STATS		_IOWR('i', 236, struct ifreq)
#define	SIOC80211IFCREATE	_IOWR('i', 237, struct ifreq)
#define	SIOC80211IFDESTROY	 _IOW('i', 238, struct ifreq)

#define IEEE80211_IOC_SSID		1
#define IEEE80211_IOC_NUMSSIDS		2
#define IEEE80211_IOC_WEP		3
#define	IEEE80211_WEP_NOSUP		-1
#define	IEEE80211_WEP_OFF		0
#define	IEEE80211_WEP_ON		1
#define	IEEE80211_WEP_MIXED		2
#define IEEE80211_IOC_WEPKEY		4
#define IEEE80211_IOC_NUMWEPKEYS	5
#define IEEE80211_IOC_WEPTXKEY		6
#define IEEE80211_IOC_AUTHMODE		7
#define IEEE80211_IOC_STATIONNAME	8
#define IEEE80211_IOC_CHANNEL		9
#define IEEE80211_IOC_POWERSAVE		10
#define	IEEE80211_POWERSAVE_NOSUP	-1
#define	IEEE80211_POWERSAVE_OFF		0
#define	IEEE80211_POWERSAVE_CAM		1
#define	IEEE80211_POWERSAVE_PSP		2
#define	IEEE80211_POWERSAVE_PSP_CAM	3
#define	IEEE80211_POWERSAVE_ON		IEEE80211_POWERSAVE_CAM
#define IEEE80211_IOC_POWERSAVESLEEP	11
#define	IEEE80211_IOC_RTSTHRESHOLD	12
#define IEEE80211_IOC_PROTMODE		13
#define	IEEE80211_PROTMODE_OFF		0
#define	IEEE80211_PROTMODE_CTS		1
#define	IEEE80211_PROTMODE_RTSCTS	2
#define	IEEE80211_IOC_TXPOWER		14	/* global tx power limit */
#define	IEEE80211_IOC_BSSID		15
#define	IEEE80211_IOC_ROAMING		16	/* roaming mode */
#define	IEEE80211_IOC_PRIVACY		17	/* privacy invoked */
#define	IEEE80211_IOC_DROPUNENCRYPTED	18	/* discard unencrypted frames */
#define	IEEE80211_IOC_WPAKEY		19
#define	IEEE80211_IOC_DELKEY		20
#define	IEEE80211_IOC_MLME		21
#define	IEEE80211_IOC_OPTIE		22	/* optional info. element */
#define	IEEE80211_IOC_SCAN_REQ		23
#define	IEEE80211_IOC_SCAN_RESULTS	24
#define	IEEE80211_IOC_COUNTERMEASURES	25	/* WPA/TKIP countermeasures */
#define	IEEE80211_IOC_WPA		26	/* WPA mode (0,1,2) */
#define	IEEE80211_IOC_CHANLIST		27	/* channel list */
#define	IEEE80211_IOC_WME		28	/* WME mode (on, off) */
#define	IEEE80211_IOC_HIDESSID		29	/* hide SSID mode (on, off) */
#define IEEE80211_IOC_APBRIDGE		30	/* AP inter-sta bridging */
#define	IEEE80211_IOC_MCASTCIPHER	31	/* multicast/default cipher */
#define	IEEE80211_IOC_MCASTKEYLEN	32	/* multicast key length */
#define	IEEE80211_IOC_UCASTCIPHERS	33	/* unicast cipher suites */
#define	IEEE80211_IOC_UCASTCIPHER	34	/* unicast cipher */
#define	IEEE80211_IOC_UCASTKEYLEN	35	/* unicast key length */
#define	IEEE80211_IOC_DRIVER_CAPS	36	/* driver capabilities */
#define	IEEE80211_IOC_KEYMGTALGS	37	/* key management algorithms */
#define	IEEE80211_IOC_RSNCAPS		38	/* RSN capabilities */
#define	IEEE80211_IOC_WPAIE		39	/* WPA information element */
#define	IEEE80211_IOC_STA_STATS		40	/* per-station statistics */
#define	IEEE80211_IOC_MACCMD		41	/* MAC ACL operation */
#define	IEEE80211_IOC_TXPOWMAX		43	/* max tx power for channel */
#define	IEEE80211_IOC_STA_TXPOW		44	/* per-station tx power limit */
#define	IEEE80211_IOC_STA_INFO		45	/* station/neighbor info */
#define	IEEE80211_IOC_WME_CWMIN		46	/* WME: ECWmin */
#define	IEEE80211_IOC_WME_CWMAX		47	/* WME: ECWmax */
#define	IEEE80211_IOC_WME_AIFS		48	/* WME: AIFSN */
#define	IEEE80211_IOC_WME_TXOPLIMIT	49	/* WME: txops limit */
#define	IEEE80211_IOC_WME_ACM		50	/* WME: ACM (bss only) */
#define	IEEE80211_IOC_WME_ACKPOLICY	51	/* WME: ACK policy (!bss only)*/
#define	IEEE80211_IOC_DTIM_PERIOD	52	/* DTIM period (beacons) */
#define	IEEE80211_IOC_BEACON_INTERVAL	53	/* beacon interval (ms) */
#define	IEEE80211_IOC_ADDMAC		54	/* add sta to MAC ACL table */
#define	IEEE80211_IOC_DELMAC		55	/* del sta from MAC ACL table */
#define	IEEE80211_IOC_FF		56	/* ATH fast frames (on, off) */
#define	IEEE80211_IOC_TURBOP		57	/* ATH turbo' (on, off) */
#define	IEEE80211_IOC_APPIEBUF		58	/* IE in the management frame */
#define	IEEE80211_IOC_FILTERFRAME	59	/* management frame filter */

/*
 * Scan result data returned for IEEE80211_IOC_SCAN_RESULTS.
 */
struct ieee80211req_scan_result {
	uint16_t isr_len;		/* length (mult of 4) */
	uint16_t isr_freq;		/* MHz */
	uint16_t isr_flags;		/* channel flags */
	uint8_t isr_noise;
	uint8_t isr_rssi;
	uint8_t isr_intval;		/* beacon interval */
	uint16_t isr_capinfo;		/* capabilities */
	uint8_t isr_erp;		/* ERP element */
	uint8_t isr_bssid[IEEE80211_ADDR_LEN];
	uint8_t isr_nrates;
	uint8_t isr_rates[IEEE80211_RATE_MAXSIZE];
	uint8_t isr_ssid_len;		/* SSID length */
	uint8_t isr_ie_len;		/* IE length */
	uint8_t isr_pad[5];
	/* variable length SSID followed by IE data */
};

#endif /* __FreeBSD__ */

#if defined(__linux__) || defined(MUC_BUILD) || defined(DSP_BUILD)
/*
 * Wireless Extensions API, private ioctl interfaces.
 *
 * NB: Even-numbered ioctl numbers have set semantics and are privileged!
 *     (regardless of the incorrect comment in wireless.h!)
 */
#ifdef __KERNEL__
#include <linux/if.h>
#endif
#define	IEEE80211_IOCTL_SETPARAM	(SIOCIWFIRSTPRIV+0)
#define	IEEE80211_IOCTL_GETPARAM	(SIOCIWFIRSTPRIV+1)
#define	IEEE80211_IOCTL_SETMODE		(SIOCIWFIRSTPRIV+2)
#define	IEEE80211_IOCTL_GETMODE		(SIOCIWFIRSTPRIV+3)
#define	IEEE80211_IOCTL_SETWMMPARAMS	(SIOCIWFIRSTPRIV+4)
#define	IEEE80211_IOCTL_GETWMMPARAMS	(SIOCIWFIRSTPRIV+5)
#define	IEEE80211_IOCTL_SETCHANLIST	(SIOCIWFIRSTPRIV+6)
#define	IEEE80211_IOCTL_GETCHANLIST	(SIOCIWFIRSTPRIV+7)
#define	IEEE80211_IOCTL_CHANSWITCH	(SIOCIWFIRSTPRIV+8)
#define	IEEE80211_IOCTL_GET_APPIEBUF	(SIOCIWFIRSTPRIV+9)
#define	IEEE80211_IOCTL_SET_APPIEBUF	(SIOCIWFIRSTPRIV+10)
#define	IEEE80211_IOCTL_FILTERFRAME	(SIOCIWFIRSTPRIV+12)
#define	IEEE80211_IOCTL_GETCHANINFO	(SIOCIWFIRSTPRIV+13)
#define	IEEE80211_IOCTL_SETOPTIE	(SIOCIWFIRSTPRIV+14)
#define	IEEE80211_IOCTL_GETOPTIE	(SIOCIWFIRSTPRIV+15)
#define	IEEE80211_IOCTL_SETMLME		(SIOCIWFIRSTPRIV+16)
#define	IEEE80211_IOCTL_RADAR		(SIOCIWFIRSTPRIV+17)
#define	IEEE80211_IOCTL_SETKEY		(SIOCIWFIRSTPRIV+18)
#define	IEEE80211_IOCTL_POSTEVENT	(SIOCIWFIRSTPRIV+19)
#define	IEEE80211_IOCTL_DELKEY		(SIOCIWFIRSTPRIV+20)
#define	IEEE80211_IOCTL_TXEAPOL		(SIOCIWFIRSTPRIV+21)
#define	IEEE80211_IOCTL_ADDMAC		(SIOCIWFIRSTPRIV+22)
#define	IEEE80211_IOCTL_STARTCCA	(SIOCIWFIRSTPRIV+23)
#define	IEEE80211_IOCTL_DELMAC		(SIOCIWFIRSTPRIV+24)
#define IEEE80211_IOCTL_GETSTASTATISTIC	(SIOCIWFIRSTPRIV+25)
#define	IEEE80211_IOCTL_WDSADDMAC	(SIOCIWFIRSTPRIV+26)
#define IEEE80211_IOCTL_TRANSMIT_HETRIG (SIOCIWFIRSTPRIV+27)
#define	IEEE80211_IOCTL_WDSDELMAC	(SIOCIWFIRSTPRIV+28)
#define IEEE80211_IOCTL_GETBLOCK	(SIOCIWFIRSTPRIV+29)
#define	IEEE80211_IOCTL_KICKMAC		(SIOCIWFIRSTPRIV+30)
#define	IEEE80211_IOCTL_DFSACTSCAN	(SIOCIWFIRSTPRIV+31)

#define IEEE80211_AMPDU_MIN_DENSITY	0
#define IEEE80211_AMPDU_MAX_DENSITY	7

#define IEEE80211_CCE_PREV_CHAN_SHIFT	8

enum {
	IEEE80211_PARAM_TURBO = 1,			/* turbo mode */
	IEEE80211_PARAM_MODE = 2,			/* 11ac mode enabled */
	IEEE80211_PARAM_AUTHMODE = 3,			/* authentication mode */
	IEEE80211_PARAM_PROTMODE = 4,			/* 802.11g protection */
	IEEE80211_PARAM_MCASTCIPHER = 5,		/* multicast/default cipher */
	IEEE80211_PARAM_MCASTKEYLEN = 6,		/* multicast key length */
	IEEE80211_PARAM_UCASTCIPHERS = 7,		/* unicast cipher suites */
	IEEE80211_PARAM_UCASTCIPHER = 8,		/* unicast cipher */
	IEEE80211_PARAM_UCASTKEYLEN = 9,		/* unicast key length */
	IEEE80211_PARAM_WPA = 10,			/* WPA mode (0,1,2) */
	IEEE80211_PARAM_ROAMING = 12,			/* roaming mode */
	IEEE80211_PARAM_PRIVACY = 13,			/* privacy invoked */
	IEEE80211_PARAM_COUNTERMEASURES = 14,		/* WPA/TKIP countermeasures */
	IEEE80211_PARAM_DROPUNENCRYPTED = 15,		/* discard unencrypted frames */
	IEEE80211_PARAM_DRIVER_CAPS = 16,		/* driver capabilities */
	IEEE80211_PARAM_MACCMD = 17,			/* MAC ACL operation */
	IEEE80211_PARAM_WMM = 18,			/* WMM mode (on, off) */
	IEEE80211_PARAM_HIDESSID = 19,			/* hide SSID mode (on, off) */
	IEEE80211_PARAM_APBRIDGE = 20,			/* AP inter-sta bridging */
	IEEE80211_PARAM_KEYMGTALGS = 21,		/* key management algorithms */
	IEEE80211_PARAM_RSNCAPS = 22,			/* RSN capabilities */
	IEEE80211_PARAM_INACT = 23,			/* station inactivity timeout */
	IEEE80211_PARAM_INACT_AUTH = 24,		/* station auth inact timeout */
	IEEE80211_PARAM_INACT_INIT = 25,		/* station init inact timeout */
	IEEE80211_PARAM_ABOLT = 26,			/* Atheros Adv. Capabilities */
	IEEE80211_PARAM_DTIM_PERIOD = 28,		/* DTIM period (beacons) */
	IEEE80211_PARAM_BEACON_INTERVAL = 29,		/* beacon interval (ms) */
	IEEE80211_PARAM_DOTH = 30,			/* 11.h is on/off */
	IEEE80211_PARAM_PWRCONSTRAINT = 31,		/* Current chan pwr constraint */
	IEEE80211_PARAM_GENREASSOC = 32,		/* Generate a reassociation request */
	IEEE80211_PARAM_COMPRESSION = 33,		/* compression */
	IEEE80211_PARAM_FF = 34,			/* fast frames support */
	IEEE80211_PARAM_XR = 35,			/* XR support */
	IEEE80211_PARAM_BURST = 36,			/* burst mode */
	IEEE80211_PARAM_PUREG = 37,			/* pure 11g (no 11b stations) */
	IEEE80211_PARAM_REPEATER = 38,			/* simultaneous AP and STA mode */
	IEEE80211_PARAM_WDS = 39,			/* Enable 4 address processing */
	IEEE80211_PARAM_BGSCAN = 40,			/* bg scanning (on, off) */
	IEEE80211_PARAM_BGSCAN_IDLE = 41,		/* bg scan idle threshold */
	IEEE80211_PARAM_BGSCAN_INTERVAL = 42,		/* bg scan interval */
	IEEE80211_PARAM_MCAST_RATE = 43,		/* Multicast Tx Rate */
	IEEE80211_PARAM_COVERAGE_CLASS = 44,		/* coverage class */
	IEEE80211_PARAM_COUNTRY_IE = 45,		/* enable country IE */
	IEEE80211_PARAM_SCANVALID = 46,			/* scan cache valid threshold */
	IEEE80211_PARAM_ROAM_RSSI_11A = 47,		/* rssi threshold in 11a */
	IEEE80211_PARAM_ROAM_RSSI_11B = 48,		/* rssi threshold in 11b */
	IEEE80211_PARAM_ROAM_RSSI_11G = 49,		/* rssi threshold in 11g */
	IEEE80211_PARAM_ROAM_RATE_11A = 50,		/* tx rate threshold in 11a */
	IEEE80211_PARAM_ROAM_RATE_11B = 51,		/* tx rate threshold in 11b */
	IEEE80211_PARAM_ROAM_RATE_11G = 52,		/* tx rate threshold in 11g */
	IEEE80211_PARAM_UAPSDINFO = 53,			/* value for qos info field */
	IEEE80211_PARAM_SLEEP = 54,			/* force sleep/wake */
	IEEE80211_PARAM_QOSNULL = 55,			/* force sleep/wake */
	IEEE80211_PARAM_PSPOLL = 56,			/* force ps-poll generation (sta only) */
	IEEE80211_PARAM_EOSPDROP = 57,			/* force uapsd EOSP drop (ap only) */
	IEEE80211_PARAM_MARKDFS = 58,			/* mark a dfs interfernce chan when found */
	IEEE80211_PARAM_REGCLASS = 59,			/* enable regclass ids in country IE */
	IEEE80211_PARAM_DROPUNENC_EAPOL = 60,		/* drop unencrypted eapol frames */
	IEEE80211_PARAM_SHPREAMBLE = 61,		/* Short Preamble */
	IEEE80211_PARAM_FIXED_TX_RATE = 62,		/* Set fixed TX rate */
	IEEE80211_PARAM_MIMOMODE = 63,			/* Select antenna to use */
	IEEE80211_PARAM_AGGREGATION = 64,		/* Enable aggregation */
	IEEE80211_PARAM_RETRY_COUNT = 65,		/* Retry AMPDU packets (0-15) per rate */
	IEEE80211_PARAM_VAP_DBG = 66,			/* Set the VAP debug verbosity . */
	IEEE80211_PARAM_VCO_CALIB = 67,			/* Set VCO calibration */
	IEEE80211_PARAM_STD_BF_QMAT_USE = 68,		/* Enable use of Tx BF matrices */
	IEEE80211_PARAM_RADIO_BW = 69,			/* Select radio BW */
	IEEE80211_PARAM_RG = 70,			/* Let software fill in duration update */
	IEEE80211_PARAM_BW_SEL_MUC = 71,		/* Let software fill in duration update */
	IEEE80211_PARAM_ACK_POLICY = 72,		/* 1 for ACK, zero for no ACK */
	IEEE80211_PARAM_LEGACY_MODE = 73,		/* 1 for legacy, zero for HT*/
	IEEE80211_PARAM_MAX_AMPDU_SUBFRM = 74,		/* Max aggregation subframes */
	IEEE80211_PARAM_ADD_WDS_MAC = 75,		/* Add MAC address for WDS peer */
	IEEE80211_PARAM_DEL_WDS_MAC = 76,		/* Delete MAC address for WDS peer */
	IEEE80211_PARAM_TXBF_CTRL = 77,			/* Control TX beamforming */
	IEEE80211_PARAM_STD_BF_SOUND = 78,		/* Enable Tx BF sounding */
	IEEE80211_PARAM_BSSID = 79,			/* Set BSSID */
	IEEE80211_PARAM_HTBA_SEQ_CTRL = 80,		/* Control HT Block ACK */
	IEEE80211_PARAM_HTBA_SIZE_CTRL = 81,		/* Control HT Block ACK */
	IEEE80211_PARAM_HTBA_TIME_CTRL = 82,		/* Control HT Block ACK */
	IEEE80211_PARAM_HT_ADDBA = 83,			/* ADDBA control */
	IEEE80211_PARAM_HT_DELBA = 84,			/* DELBA control */
	IEEE80211_PARAM_CHANNEL_NOSCAN = 85,		/* Disable the scanning for fixed chans */
	IEEE80211_PARAM_PROFILE_MUC = 86,		/* Control MuC profiling */
	IEEE80211_PARAM_MUC_PHY_STATS = 87,		/* Control MuC phy stats */
	IEEE80211_PARAM_MUC_SET_PARTNUM = 88,		/* set muc part num for cal */
	IEEE80211_PARAM_ENABLE_GAIN_ADAPT = 89,		/* turn on the anlg gain tuning */
	IEEE80211_PARAM_GET_RFCHIP_ID = 90,		/* Get RF chip frequency id */
	IEEE80211_PARAM_GET_RFCHIP_VERID = 91,		/* Get RF chip version id */
	IEEE80211_PARAM_ADD_WDS_MAC_DOWN = 92,		/* Add MAC address for WDS downlink peer */
	IEEE80211_PARAM_SHORT_GI = 93,			/* Enable SGI */
	IEEE80211_PARAM_LINK_LOSS = 94,			/* Enable Link Loss feature */
	IEEE80211_PARAM_BCN_MISS_THR = 95,		/* Set beacon miss threshold */
	IEEE80211_PARAM_FORCE_SMPS = 96,		/* Force SMPS mode (STA) */
	IEEE80211_PARAM_FORCEMICERROR = 97,
			/* Force MIC error - loopback MUC to QDRV to normal TKIP MIC error path */
	IEEE80211_PARAM_ENABLECOUNTERMEASURES = 98,	/* Enable countermeasures */
	IEEE80211_PARAM_IMPLICITBA = 99,		/* Set the implicit BA flags in the QIE */
	IEEE80211_PARAM_CLIENT_REMOVE = 100,		/* Remove clients but DON'T deauth them */
	IEEE80211_PARAM_SHOWMEM = 101,			/* Show summary view for malloc/free */
	IEEE80211_PARAM_SCANSTATUS = 102,		/* Get scanning state */
	IEEE80211_PARAM_GLOBAL_BA_CONTROL = 103,	/* Set the global BA flags */
	IEEE80211_PARAM_NO_SSID_ASSOC = 104,		/* Enable associations without SSIDs */
	IEEE80211_PARAM_FIXED_SGI = 105,		/* Choose node based SGI or fixed SGI */
	IEEE80211_PARAM_CONFIG_TXPOWER = 106,		/* set TX pwr for band (start/stop chan) */
	IEEE80211_PARAM_SKB_LIST_MAX = 107,		/* set max len of skb list */
	IEEE80211_PARAM_VAP_STATS = 108,		/* Show VAP stats */
	IEEE80211_PARAM_RATE_CTRL_FLAGS = 109,		/* Set flags to tweak rate control algo */
	IEEE80211_PARAM_LDPC = 110,			/* Enabling/disabling LDPC */
	IEEE80211_PARAM_DFS_FAST_SWITCH = 111,		/* switch to non-DFS chan on radar detect */
	IEEE80211_PARAM_11N_40_ONLY_MODE = 112,		/* Support for 11n 40MHZ only mode */
	IEEE80211_PARAM_AMPDU_DENSITY = 113,		/* AMPDU DENSITY CONTROL */
	IEEE80211_PARAM_SCAN_NO_DFS = 114,		/* On radar detect, avoid DFS chans */
	IEEE80211_PARAM_REGULATORY_REGION = 115,	/* set the regulatory region */
	IEEE80211_PARAM_CONFIG_BB_INTR_DO_SRESET = 116,	/* enable sw reset for BB interrupt */
	IEEE80211_PARAM_CONFIG_MAC_INTR_DO_SRESET = 117,/* enable sw reset for MAC interrupt */
	IEEE80211_PARAM_CONFIG_WDG_DO_SRESET = 118,	/* enable sw reset triggered by watchdog */
	IEEE80211_PARAM_TRIGGER_RESET = 119,		/* trigger reset for MAC/BB */
	IEEE80211_PARAM_INJECT_INVALID_FCS = 120,	/* inject bad FCS to induce tx hang */
	IEEE80211_PARAM_CONFIG_WDG_SENSITIVITY = 121,	/* higher value means less sensitive */
	IEEE80211_PARAM_SAMPLE_RATE = 122,		/* Set data sampling rate */
	IEEE80211_PARAM_MCS_CAP = 123,			/* Configure an MCS cap rate - for debug */
	IEEE80211_PARAM_MAX_MGMT_FRAMES = 124,		/* Max number of mgmt frames not complete */
	IEEE80211_PARAM_MCS_ODD_EVEN = 125,		/* Set RA algo to odd or even MCSs */
	IEEE80211_PARAM_BLACKLIST_GET = 126,		/* List blacklisted stations. */
	IEEE80211_PARAM_BA_MAX_WIN_SIZE = 128,		/* Max BA window size on TX and RX */
	IEEE80211_PARAM_RESTRICTED_MODE = 129,		/* Enable restricted mode */
	IEEE80211_PARAM_BB_MAC_RESET_MSGS = 130,	/* Enable BB amd MAC reset msgs */
	IEEE80211_PARAM_PHY_STATS_MODE = 131,		/* Mode for get_phy_stats */
	IEEE80211_PARAM_BB_MAC_RESET_DONE_WAIT = 132,	/* max secs for tx or rx before reset */
	IEEE80211_PARAM_MIN_DWELL_TIME_ACTIVE = 133,	/* min dwell time for an active chan */
	IEEE80211_PARAM_MIN_DWELL_TIME_PASSIVE = 134,	/* min dwell time for a passive chan */
	IEEE80211_PARAM_MAX_DWELL_TIME_ACTIVE = 135,	/* max dwell time for an active chan */
	IEEE80211_PARAM_MAX_DWELL_TIME_PASSIVE = 136,	/* max dwell time for a passive chan */
	IEEE80211_PARAM_TX_AGG_TIMEOUT = 137,		/* Configure timeout for TX aggregation */
	IEEE80211_PARAM_LEGACY_RETRY_LIMIT = 138,	/* Retry for non-AMPDU packets per rate */
	IEEE80211_PARAM_TRAINING_COUNT = 139,		/* Training count for rate retry algo */
	IEEE80211_PARAM_DYNAMIC_AC = 140,		/* Enable dynamic 1 bit auto corr. algo */
	IEEE80211_PARAM_DUMP_TRIGGER = 141,		/* Request immediate dump */
	IEEE80211_PARAM_DUMP_TCM_FD = 142,		/* Dump TCM frame descriptors */
	IEEE80211_PARAM_RXCSR_ERR_ALLOW = 143,		/* allow passing error packets to MuC */
	IEEE80211_PARAM_STOP_FLAGS = 144,		/* Set dbg halt on error conditions flags */
	IEEE80211_PARAM_CHECK_FLAGS = 145,		/* Set additional runtime checks flags */
	IEEE80211_PARAM_RX_CTRL_FILTER = 146,		/* Set the control packet filter on hal. */
	IEEE80211_PARAM_SCS = 147,			/* ACI/CCI Detection and Mitigation*/
	IEEE80211_PARAM_ALT_CHAN = 148,			/* set jump chan if radar detected */
	IEEE80211_PARAM_QTN_BCM_WAR = 149,	/* WAR for BCM receiver not accepting last aggr */
	IEEE80211_PARAM_GI_SELECT = 150,		/* Enable dynamic GI selection */
	IEEE80211_PARAM_RADAR_NONOCCUPY_PERIOD = 151,	/* Specify radar non-occupancy period */
	IEEE80211_PARAM_RADAR_NONOCCUPY_ACT_SCAN = 152,	/* non-occupancy expire scan/no-action */
	IEEE80211_PARAM_MC_LEGACY_RATE = 153,		/* Legacy multicast rate table */
	IEEE80211_PARAM_LDPC_ALLOW_NON_QTN = 154,	/* Allow non QTN nodes to use LDPC */
	IEEE80211_PARAM_IGMP_HYBRID = 155,		/* IGMP hybrid mode for Swisscom */
	IEEE80211_PARAM_BCST_4 = 156,		/* Reliable (4 addr encap) bcast to all clients */
	IEEE80211_PARAM_AP_FWD_LNCB = 157,	/* AP fwd LNCB packets from STA to other STAs */
	IEEE80211_PARAM_PPPC_SELECT = 158,		/* Per packet power control */
	IEEE80211_PARAM_TEST_LNCB = 159,		/* Test LNCB code - leaks, drops etc. */
	IEEE80211_PARAM_STBC = 160,			/* Enable STBC */
	IEEE80211_PARAM_RTS_CTS = 161,			/* Enable RTS-CTS */
	IEEE80211_PARAM_GET_DFS_CCE = 162,		/* Get most recent DFS chan change event */
	IEEE80211_PARAM_GET_SCS_CCE = 163,	/* Get most recent SCS (ACI/CCI) chan chg event */
	IEEE80211_PARAM_GET_CH_INUSE = 164,	/* Enable printing of chan in Use at end of scan */
	IEEE80211_PARAM_RX_AGG_TIMEOUT = 165,		/* RX aggregation timeout value (ms) */
	IEEE80211_PARAM_FORCE_MUC_HALT = 166,		/* Force MUC halt debug code. */
	IEEE80211_PARAM_FORCE_ENABLE_TRIGGERS = 167,	/* Enable trace triggers */
	IEEE80211_PARAM_FORCE_MUC_TRACE = 168,		/* MuC trace force without halt */
	IEEE80211_PARAM_BK_BITMAP_MODE = 169,		/* back bit map mode set */
	IEEE80211_PARAM_PROBE_RES_DEAUTH_RETRIES = 170,	/* Set probe response and deauth retries */
	IEEE80211_PARAM_MUC_FLAGS = 171,		/* MuC flags */
	IEEE80211_PARAM_HT_NSS_CAP = 172,		/* Set max spatial streams for HT mode */
	IEEE80211_PARAM_ASSOC_LIMIT = 173,		/* STA assoc limit */
	IEEE80211_PARAM_PWR_ADJUST_SCANCNT = 174,/* Enable pwr adjust if nearby sta don't assoc */
	IEEE80211_PARAM_PWR_ADJUST = 175,	/* ioctl to adjust rx gain */
	IEEE80211_PARAM_PWR_ADJUST_AUTO = 176,	/* Enable auto RX gain adjust when associated */
	IEEE80211_PARAM_UNKNOWN_DEST_ARP = 177,	/* Send ARP requests for unknown destinations */
	IEEE80211_PARAM_UNKNOWN_DEST_FWD = 178,	/* Send unknown dest pkt to all bridge STAs */
	IEEE80211_PARAM_DBG_MODE_FLAGS = 179,	/* set/clear debug mode flags */
	IEEE80211_PARAM_ASSOC_HISTORY = 180,	/* record of associated nodes by MAC address */
	IEEE80211_PARAM_CSW_RECORD = 181,	/* get chan switch record data */
	IEEE80211_PARAM_RESTRICT_RTS = 182,	/* HW xretry failures before switch to RTS mode */
	IEEE80211_PARAM_RESTRICT_LIMIT = 183,	/* RTS xretry failures before start restrict mode */
	IEEE80211_PARAM_AP_ISOLATE = 184,	/* set ap isolation mode */
	IEEE80211_PARAM_IOT_TWEAKS = 185,		/* mask to switch on / off IOT tweaks */
	IEEE80211_PARAM_BSS_ASSOC_LIMIT = 188,		/* STA assoc limit for a VAP */
	IEEE80211_PARAM_CCA_PRI = 191,			/* Primary CCA threshold */
	IEEE80211_PARAM_CCA_SEC = 192,			/* Secondary CCA threshold */
	IEEE80211_PARAM_DYN_AGG_TIMEOUT = 193,	/* try to prevent unnec. agg wait before snd */
	IEEE80211_PARAM_HW_BONDING = 194,		/* HW bonding option */
	IEEE80211_PARAM_PS_CMD = 195,			/* set probe select for matrices */
	IEEE80211_PARAM_PWR_SAVE = 196,			/* Power save parameter ctrl */
	IEEE80211_PARAM_DBG_FD = 197,			/* Debug FD alloc/free */
	IEEE80211_PARAM_DISCONN_CNT = 198,		/* get count of disconnection event */
	IEEE80211_PARAM_FAST_REASSOC = 199,		/* Do a fast reassociation */
	IEEE80211_PARAM_SIFS_TIMING = 200,		/* SIFS timing */
	IEEE80211_PARAM_TEST_TRAFFIC = 201,		/* Test Traffic start|stop control */
	IEEE80211_PARAM_TX_AMSDU = 202,		/* Enable AMSDU and/or Adaptive AMSDU to Qclients */
	IEEE80211_PARAM_SCS_DFS_REENTRY_REQUEST = 203,	/* DFS re-entry request from SCS */
	IEEE80211_PARAM_QCAT_STATE = 204,		/* QCAT state information */
	IEEE80211_PARAM_RALG_DBG = 205,			/* SU Rate Adaptation debugging */
	IEEE80211_PARAM_QTN_BGSCAN_DWELL_TIME_ACTIVE = 207,/* bgscan dwell time for active chan */
	IEEE80211_PARAM_QTN_BGSCAN_DWELL_TIME_PASSIVE = 208,/* bgscan dwell time for passive chan */
	IEEE80211_PARAM_QTN_BGSCAN_DEBUG = 209,		/* background scan debugging */
	IEEE80211_PARAM_CONFIG_REGULATORY_TXPOWER = 210,/* set reg TX pwr for a band */
	IEEE80211_PARAM_SINGLE_AGG_QUEUING = 211,/* Queue 1 AMPDU at a time until sw retries done */
	IEEE80211_PARAM_CSA_FLAG = 212,			/* Chan switch announcement flag */
	IEEE80211_PARAM_BR_IP_ADDR = 213,		/* Set bridge IP address */
	IEEE80211_PARAM_REMAP_QOS = 214,		/* Configure qos remap feature */
	IEEE80211_PARAM_DEF_MATRIX = 215,		/* Use default expansion matrices */
	IEEE80211_PARAM_SCS_CCA_INTF = 216,		/* CCA interference for a chan */
	IEEE80211_PARAM_CONFIG_TPC_INTERVAL = 217,	/* periodic TPC request interval */
	IEEE80211_PARAM_TPC_QUERY = 218,		/* Enable TPC request periodically */
	IEEE80211_PARAM_TPC = 219,			/* Enable TPC feature */
	IEEE80211_PARAM_CACSTATUS = 220,		/* Get CAC status */
	IEEE80211_PARAM_RTSTHRESHOLD = 221,		/* Get/Set RTS Threshold */
	IEEE80211_PARAM_BA_THROT = 222,			/* Manual BA throttling */
	IEEE80211_PARAM_TX_QUEUING_ALG = 223,		/* MuC TX queuing algorithm */
	IEEE80211_PARAM_BEACON_ALLOW = 224,		/* Enable beacon rx in STA mode */
	IEEE80211_PARAM_1BIT_PKT_DETECT = 225,		/* enable 1bit pkt detection */
	IEEE80211_PARAM_WME_THROT = 226,		/* Manual WME throttling */
	IEEE80211_PARAM_FIXED_11AC_TX_RATE = 228,	/* Set 11AC mcs */
	IEEE80211_PARAM_GENPCAP = 229,			/* WMAC tx/rx pcap ring buffer */
	IEEE80211_PARAM_CCA_DEBUG = 230,		/* Debug of CCA */
	IEEE80211_PARAM_STA_DFS = 231,			/* Enable or disable station DFS */
	IEEE80211_PARAM_OCAC = 232,			/* Off-chan CAC */
	IEEE80211_PARAM_CCA_STATS_PERIOD = 233,		/* the period for updating CCA stats */
	IEEE80211_PARAM_AUC_TX = 234,			/* Control HW datapath transmission */
	IEEE80211_PARAM_RADAR_BW = 235,			/* Set radar filter mode */
	IEEE80211_PARAM_TDLS_DISC_INT = 236,		/* Set TDLS discovery interval */
	IEEE80211_PARAM_TDLS_PATH_SEL_WEIGHT = 237,	/* Weight for path selection algorithm */
	IEEE80211_PARAM_DAC_DBG = 238,			/* dynamic ac debug */
	IEEE80211_PARAM_CARRIER_ID = 239,		/* Get/Set carrier ID */
	IEEE80211_PARAM_DEACTIVE_CHAN_PRI = 241,	/* Deactive primary chan */
	IEEE80211_PARAM_RESTRICT_RATE = 242,		/* Max pkts per sec in Tx restrict mode */
	IEEE80211_PARAM_AUC_RX_DBG = 243,		/* AuC rx debug command */
	IEEE80211_PARAM_RX_ACCELERATE = 244,		/* Enable Topaz MuC rx accelerate */
	IEEE80211_PARAM_RX_ACCEL_LOOKUP_SA = 245,	/* Enable lookup SA in FWT for rx accel. */
	IEEE80211_PARAM_TX_MAXMPDU = 246,		/* Set Max MPDU size to be supported */
	/* 247 is obsolete - do not reuse */
	IEEE80211_PARAM_SPECIFIC_SCAN = 249,		/* Just perform specific SSID scan */
	/* 250 is obsolete - do not reuse */
	IEEE80211_PARAM_TRAINING_START = 251,		/* restart rate training to a node */
	IEEE80211_PARAM_AUC_TX_DBG = 252,		/* AuC tx debug command */
	IEEE80211_PARAM_AC_INHERITANCE = 253,
					/* promote AC_BE to use aggresive medium contention */
	IEEE80211_PARAM_NODE_OPMODE = 254,	/* Set bw and NSS used for a node */
	IEEE80211_PARAM_TACMAP = 255,		/* Config TID AC and prio at TAC_MAP, debug only */
	IEEE80211_PARAM_VAP_PRI = 256,		/* Config VAP prio, for TID priority at TAC_MAP */
	IEEE80211_PARAM_AUC_QOS_SCH = 257,	/* Tune QoS scheduling in AuC */
	IEEE80211_PARAM_TXBF_IOT = 258,		/* turn on/off TxBF IOT to non QTN node */
	IEEE80211_PARAM_CONGEST_IDX = 259,	/* Current chan congestion index */
	IEEE80211_PARAM_SPEC_COUNTRY_CODE = 260,/* Set country code for EU region */
	IEEE80211_PARAM_AC_Q2Q_INHERITANCE = 261,
				/* promote AC_BE to use aggresive medium contention - Q2Q case */
	IEEE80211_PARAM_1SS_AMSDU_SUPPORT = 262,/* Enable AMSDU support for 1SS devies */
	IEEE80211_PARAM_VAP_PRI_WME = 263,	/* Auto adjust WME bss param based on VAP prio */
	IEEE80211_PARAM_MICHAEL_ERR_CNT = 264,	/* total number of TKIP MIC errors */
	IEEE80211_PARAM_DUMP_CONFIG_TXPOWER = 265,/* Dump configured txpower for all chans */
	IEEE80211_PARAM_EMI_POWER_SWITCHING = 266,/* Enable EMI power switching */
	IEEE80211_PARAM_CONFIG_BW_TXPOWER = 267,/* Configure the TX powers different bandwidths */
	IEEE80211_PARAM_SCAN_CANCEL = 268,	/* Cancel any ongoing scanning */
	IEEE80211_PARAM_VHT_NSS_CAP = 269,	/* Set max spatial streams for VHT mode */
	IEEE80211_PARAM_FIXED_BW = 270,		/* Config fixed tx bw without changing BSS BW */
	IEEE80211_PARAM_SFS = 271,		/* Smart Feature Select commands */
	IEEE80211_PARAM_CONFIG_PMF = 274,		/* Enable 802.11w / PMF */
	IEEE80211_PARAM_AUTO_CCA_ENABLE = 275,		/* Enable auto-cca-threshold feature */
	IEEE80211_PARAM_AUTO_CCA_PARAMS = 276,		/* Configure the threshold parameter */
	IEEE80211_PARAM_AUTO_CCA_DEBUG = 277,		/* Configure the auto-cca debug flag */
	IEEE80211_PARAM_INTRA_BSS_ISOLATE = 278,	/* Intra BSS isolation */
	IEEE80211_PARAM_BSS_ISOLATE = 279,		/* BSS isolation */
	IEEE80211_PARAM_BF_RX_STS = 280,		/* Set max BF sounding receive STS */
	IEEE80211_PARAM_WOWLAN = 281,			/* Wake on Wireless LAN */
	IEEE80211_PARAM_WDS_MODE = 282,			/* WDS mode */
	IEEE80211_PARAM_EXTENDER_ROLE = 283,		/* EXTENDER Device role */
	IEEE80211_PARAM_EXTENDER_MBS_BEST_RSSI = 284,	/* MBS best rssi threshold */
	IEEE80211_PARAM_EXTENDER_RBS_BEST_RSSI = 285,	/* RBS best rssi threshold */
	IEEE80211_PARAM_EXTENDER_MBS_WGT = 286,		/* MBS RSSI weight */
	IEEE80211_PARAM_EXTENDER_RBS_WGT = 287,		/* RBS RSSI weight */
	IEEE80211_PARAM_AIRFAIR = 288,			/* Set airtime fairness configuration */
	/* 289 is obsolete - do not reuse */
	IEEE80211_PARAM_RX_AMSDU_ENABLE = 290,		/* RX AMSDU: disable/enable/dynamic */
	IEEE80211_PARAM_DISASSOC_REASON = 291,		/* Get Disassoc reason */
	IEEE80211_PARAM_TX_QOS_SCHED = 292,		/* TX QoS hold-time table */
	IEEE80211_PARAM_RX_AMSDU_THRESHOLD_CCA = 293,	/* Dynamic Rx A-MSDU CCA threshld AMSDU */
	IEEE80211_PARAM_RX_AMSDU_THRESHOLD_PMBL = 294,	/* Dynamic Rx A-MSDU pmble error threshld */
	IEEE80211_PARAM_RX_AMSDU_PMBL_WF_SP = 295,	/* Weight factor of short preamble error */
	IEEE80211_PARAM_RX_AMSDU_PMBL_WF_LP = 296,	/* Weight factor of long preamble error */
	IEEE80211_PARAM_PEER_RTS_MODE = 297,		/* Mode setting for peer RTS */
	IEEE80211_PARAM_DYN_WMM = 298,			/* Dynamic WMM enable */
	IEEE80211_PARAM_BA_SETUP_ENABLE = 299,		/* enable BA according RSSI threshld */
	IEEE80211_PARAM_AGGRESSIVE_AGG = 300,		/* Compound aggressive agg params */
	/* 301 Baseband param is deprecated */
	IEEE80211_PARAM_VAP_TX_AMSDU = 302,		/* Enable A-MSDU for VAP */
	IEEE80211_PARAM_PC_OVERRIDE = 303,		/* RSSI based Power-contraint override */
	IEEE80211_PARAM_NDPA_DUR = 304,			/* Set VHT NDPA duration field */
	IEEE80211_PARAM_TXBF_PKT_CNT = 305,	/* Pkt cnt per txbf interval for sounding to node */
	IEEE80211_PARAM_MAX_AGG_SIZE = 306,	/* Maximum AMPDU size in bytes */
	IEEE80211_PARAM_TQEW_DESCR_LIMIT = 307,	/* Set/Get tqew descriptor limit */
	IEEE80211_PARAM_SCAN_TBL_LEN_MAX = 308,	/* Scan table max length */
	IEEE80211_PARAM_CS_THRESHOLD = 309,	/* Carrier Sense threshold */
	IEEE80211_PARAM_NODEREF_DBG = 327,	/* show history of node reference debug info */
	IEEE80211_PARAM_SWFEAT_DISABLE = 329,	/* disable an optional software feature */
	IEEE80211_PARAM_11N_AMSDU_CTRL = 330,	/* ctrl TX AMSDU of IP ctrl packets for 11N STAs */
	IEEE80211_PARAM_CCA_FIXED = 331,	/* Set CCA threshold */
	IEEE80211_PARAM_CCA_SEC40 = 332,	/* Set CCA threshold */
	IEEE80211_PARAM_CS_THRESHOLD_DBM = 333,	/* Set threshold in dBm */
	IEEE80211_PARAM_EXTENDER_VERBOSE = 334,	/* EXTENDER Debug Level */
	IEEE80211_PARAM_FLUSH_SCAN_ENTRY = 335,	/* Flush scan entry */
	IEEE80211_PARAM_SCAN_OPCHAN = 336,	/* Scan operating chan periodically in STA mode */
	IEEE80211_PARAM_DUMP_PPPC_TX_SCALE_BASES = 337,	/* Dump the current PPPC tx scale bases */
	IEEE80211_PARAM_VHT_OPMODE_BW = 338,		/* Set peer transmitter's BW */
	IEEE80211_PARAM_HS2 = 339,			/* Enable HS2.0 */
	IEEE80211_PARAM_DGAF_CONTROL = 340,		/* Downstream Group-Addressed Forwarding */
	IEEE80211_PARAM_PROXY_ARP = 341,		/* Proxy ARP */
	IEEE80211_PARAM_GLOBAL_FIXED_TX_SCALE_INDEX = 342,/* Set global fixed tx scale index */
	IEEE80211_PARAM_RATE_TRAIN_DBG = 343,		/* Rate training */
	IEEE80211_PARAM_NDPA_LEGACY_FORMAT = 344,	/* Configure PHY format for NDPA frame */
	IEEE80211_PARAM_QTN_HAL_PM_CORRUPT_DEBUG = 345,	/* dbg qtn packet memory corruption */
	IEEE80211_PARAM_UPDATE_MU_GRP = 346,		/* Update MU group/position */
	IEEE80211_PARAM_FIXED_MU_TX_RATE = 347,		/* Set 11AC MU fixed mcs */
	IEEE80211_PARAM_MU_DEBUG_LEVEL = 348,		/* Set 11AC MU debug level */
	IEEE80211_PARAM_MU_ENABLE = 349,		/* Enable MU transmission */
	IEEE80211_PARAM_INST_MU_GRP_QMAT = 350,		/* Install qmat for mu group */
	IEEE80211_PARAM_DELE_MU_GRP_QMAT = 351,		/* Delete qmat in mu group */
	IEEE80211_PARAM_GET_MU_GRP = 352,		/* Retrieve MU group and Q matrix info */
	IEEE80211_PARAM_EN_MU_GRP_QMAT = 353,		/* Enable qmat in mu group */
	IEEE80211_PARAM_MU_DEBUG_FLAG = 354,		/* Set or clear MU debug flag */
	IEEE80211_PARAM_DSP_DEBUG_LEVEL = 355,		/* DSP debug verbocity level */
	IEEE80211_PARAM_DSP_DEBUG_FLAG = 356,		/* Set DSP debug flag */
	IEEE80211_PARAM_SET_CRC_ERR = 357,		/* Pass CRC error pkt memory */
	IEEE80211_PARAM_MU_SWITCH_USR_POS = 358,/* Switch MU user_pos to dbg MU interference */
	IEEE80211_PARAM_SET_PREC_SND_PERIOD = 360,	/* Sets precoding sounding period */
	IEEE80211_PARAM_INST_1SS_DEF_MAT_ENABLE = 361,	/* Enable install 1ss default matrix */
	IEEE80211_PARAM_INST_1SS_DEF_MAT_THRESHOLD = 362,/* Set threshold for 1ss dflt matrix */
	IEEE80211_PARAM_SCAN_RESULTS_CHECK_INV = 363,	/* interval to check scan results */
	IEEE80211_PARAM_TDLS_OVER_QHOP_ENABLE = 364,	/* Enable TDLS over qhop */
	IEEE80211_PARAM_DSP_PRECODING_ALGORITHM = 365,	/* set precoding algo 1-projection 2-BD */
	IEEE80211_PARAM_DSP_RANKING_ALGORITHM = 366,	/* set ranking algo 1-projection or 2-BD */
	IEEE80211_PARAM_DIS_MU_GRP_QMAT = 367,		/* Disable QMat for MU group */
	IEEE80211_PARAM_GET_MU_GRP_QMAT = 368,		/* Get QMat status */
	IEEE80211_PARAM_MU_USE_EQ = 369,		/* Equalizer status */
	IEEE80211_PARAM_INITIATE_TXPOWER_TABLE = 370,	/* Init TX pwr tbl for single value band */
	IEEE80211_PARAM_L2_EXT_FILTER = 371,		/* External L2 Filter */
	IEEE80211_PARAM_L2_EXT_FILTER_PORT = 372,	/* External L2 Filter port */
	IEEE80211_PARAM_MU_AIRTIME_PADDING = 373,	/* Airtime padding for MU/SU Tx decision */
	IEEE80211_PARAM_MU_AMSDU_SIZE = 374,		/* Set Fixed MU AMSDU size */
	IEEE80211_PARAM_SDFS = 375,			/* Seamless DFS, same as PARAM_OCAC */
	IEEE80211_PARAM_DSP_MU_RANK_CRITERIA = 376,	/* select mu ranking criteria */
	IEEE80211_PARAM_SET_VERIWAVE_POWER = 377,	/* Set Fixed tx pwr for veriwave clients */
	IEEE80211_PARAM_ENABLE_RX_OPTIM_STATS = 378,	/* Enable RX optim stats */
	IEEE80211_PARAM_SET_UNICAST_QUEUE_NUM = 379,	/* Set Max congest queue num for unicast */
	IEEE80211_PARAM_MRC_ENABLE = 380,		/* Set Mgmt Frame Rate Control feature */
	IEEE80211_PARAM_VCO_LOCK_DETECT_MODE = 381,	/* Enable lock detect functionality */
	IEEE80211_PARAM_OBSS_EXEMPT_REQ = 382,		/* OBSS scan exemption request*/
	IEEE80211_PARAM_OBSS_TRIGG_SCAN_INT = 383,	/* OBSS scan exemption request*/
	IEEE80211_PARAM_BW_2_4GHZ = 385,		/* Bandwidth in 2.4ghz band */
	IEEE80211_PARAM_ALLOW_VHT_TKIP = 386,	/* allow VHT with only TKIP for WFA testbed */
	IEEE80211_PARAM_AUTO_CS_ENABLE = 387,	/* Enable auto-cs-threshold feature */
	IEEE80211_PARAM_AUTO_CS_PARAMS = 388,	/* Configure the threshold parameter */
	IEEE80211_PARAM_QTN_BGSCAN_DURATION_ONESHOT = 389,
						/* bgscan duration for oneshot mode */
	IEEE80211_PARAM_QTN_BGSCAN_DURATION_MULTI_SLOTS_FAST = 390,
						/* bgscan duration for multiple slot fast mode */
	IEEE80211_PARAM_QTN_BGSCAN_DURATION_MULTI_SLOTS_NORMAL = 391,
						/* bgscan duration for multiple slot normal mode */
	IEEE80211_PARAM_QTN_BGSCAN_DURATION_MULTI_SLOTS_SLOW = 392,
						/* bgscan duration for multiple slots slow mode */
	IEEE80211_PARAM_QTN_BGSCAN_THRSHLD_MULTI_SLOTS_FAST = 393,
					/* bgscan fat threshold for multiple slots fast mode */
	IEEE80211_PARAM_QTN_BGSCAN_THRSHLD_MULTI_SLOTS_NORMAL = 394,
					/* bgscan fat threshold for multiple slots normal mode */
	IEEE80211_PARAM_QTN_BLOCK_BSS = 395,		/* Block assoc requests for specified BSS */
	IEEE80211_PARAM_VHT_2_4GHZ = 396,		/* Qtn 2.4G band VHT support */
	IEEE80211_PARAM_PHY_MODE = 397,			/* Hardware phy mode */
	IEEE80211_PARAM_BEACONING_SCHEME = 398,	/* map btwn 8 VAPs and 4 HW event queues for bcn */
	IEEE80211_PARAM_STA_BMPS = 399,			/* enable STA BMPS */
	IEEE80211_PARAM_40MHZ_INTOLERANT = 400,		/* 20/40 coexistence - 40 MHz intolerant */
	IEEE80211_PARAM_DISABLE_TX_BA = 402,		/* enable TX Block Ack establishment */
	IEEE80211_PARAM_DECLINE_RX_BA = 403,		/* permit RX Block Ack establishment */
	IEEE80211_PARAM_ENABLE_11AX = 404,		/* Enable 802.11ax */
	IEEE80211_PARAM_TX_AIRTIME_CONTROL = 405,	/* start or stop tx airtime accumulaton */
	IEEE80211_PARAM_OBSS_COEXIST = 406,		/* Enable or disable OBSS scan */
	IEEE80211_PARAM_SHORT_SLOT = 407,		/* short slot */
	IEEE80211_PARAM_SET_RTS_BW_DYN = 408,		/* set RTS bw signal bw and dynamic flag */
	IEEE80211_PARAM_SET_CTS_BW = 409,/* force CTS BW by setting secondary 20/40 chan CCA busy */
	IEEE80211_PARAM_VHT_MCS_CAP = 410,/* Set MCS capability for VHT mode, for WFA testbed */
	IEEE80211_PARAM_VHT_OPMODE_NOTIF = 411,	/* Override OpMode notif IE for WFA testbed */
	IEEE80211_PARAM_FIRST_STA_IN_MU_SOUNDING = 412,	/* select first STA for mu sounding */
	IEEE80211_PARAM_USE_NON_HT_DUPLICATE_MU = 413,
			/* Allow non-HT duplicate for MU NDPA and Report_Poll using BW signal TA */
	IEEE80211_PARAM_OSEN = 414,			/* OSEN */
	IEEE80211_PARAM_BG_PROTECT = 415,		/* 802.11g protection */
	IEEE80211_PARAM_11N_PROTECT = 417,		/* 802.11n protection */
	IEEE80211_PARAM_SET_MU_RANK_TOLERANCE = 418,	/* MU rank tolerance */
	IEEE80211_PARAM_MU_ALLOW_NON_QTN = 419,		/* Allow non QTN nodes to use MU */
	IEEE80211_PARAM_MU_NDPA_BW_SIGNALING_SUPPORT = 420,/* Recv NDPA with bw signalling TA */
	IEEE80211_PARAM_RESTRICT_WLAN_IP = 421,	/* Block IP pkts from wifi to bridge interfaces */
	IEEE80211_PARAM_MC_TO_UC = 422,		/* Convert L2 multicast to unicast */
	IEEE80211_PARAM_ENABLE_BC_IOT_WAR = 423,/* allow STS to 4 in beacon when disabled */
	IEEE80211_PARAM_HOSTAP_STARTED = 424,		/* hostapd state */
	IEEE80211_PARAM_WPA_STARTED = 425,		/* wpa_supplicant state */
	IEEE80211_PARAM_DRIVER_ASSOC = 426,		/* driver alone or wpa_supplicant */
	IEEE80211_PARAM_ENABLE_11B_WAR = 427,		/* driver alone or wpa_supplicant */
	IEEE80211_PARAM_PROFILE_LHOST = 428,		/* Control host driver profiling */
	IEEE80211_PARAM_PROFILE_AUC = 429,		/* Control AuC firmware profiling */
	IEEE80211_PARAM_RSSI_MEAS_PERIOD = 430,		/* Set the period for measuring RSSI */
	IEEE80211_PARAM_AIRQUOTA = 431,			/* Set airtime quota configuration */
	IEEE80211_PARAM_SHORT_RETRY_LIMIT = 432,/* Set shrt retry limits for frame <= RTS thrshld */
	IEEE80211_PARAM_LONG_RETRY_LIMIT = 433,	/* Set long retry limits for frame > RTS thrshld */
	IEEE80211_PARAM_STD_BF_FEEDBACK = 434,		/* Enable Tx BF feedback */
	IEEE80211_PARAM_SUBFRM_PER_AMSDU_LIMIT = 435,	/* Set sumfrm per amsdu limit */
	IEEE80211_PARAM_MU_QMAT_BYPASS_MODE_EN = 436,	/* Enable MU Q matrices bypass mode */
	IEEE80211_PARAM_1024QAM = 437,			/* Enable 1024 QAM MCS */
	IEEE80211_PARAM_ALLOW_11B = 438,		/* Allow 11b client to connect */
	IEEE80211_PARAM_AUTOCHAN_DBG_LEVEL = 439,	/* set/get debug level of chan selection */
	IEEE80211_PARAM_QOS_PREMIER = 440,		/* set qos premier value for node */
	IEEE80211_PARAM_QOS_PREMIER_RULE = 441,		/* set qos premier rule */
	IEEE80211_PARAM_TXBF_QCACHE_DUMP = 442,		/* Dump the QCache table */
	IEEE80211_PARAM_TXBF_DDR_DBG = 443,		/* Enable DDR and QCACHE debugs */
	IEEE80211_PARAM_PPPC_TXML_BYPASS = 444,		/* Enable baseband PPPC function */
	IEEE80211_PARAM_80211V_BTM = 445,		/* 80211v(WNM) - BSS transition mgmt */
	IEEE80211_PARAM_WNM_SEND_TEST_BTM_REQ = 446,	/* 80211v- bss transition request */
	IEEE80211_PARAM_1024QAM_DYNC_LIMIT = 447,	/* Enable 4SS 1024QAM selection on 160M */
	IEEE80211_PARAM_PPPC_WAR_CMD = 448,		/* PPPC WAR */
	IEEE80211_PARAM_OFFLOAD_CFG = 449,		/* Set hbm offloading to Lhost switch */
	IEEE80211_PARAM_BSS_GROUP_ID = 450,		/* Assign VAP (SSID) a logical group id */
	IEEE80211_PARAM_BSS_ASSOC_RESERVE = 451,	/* Reserve assoc for specified group */
	IEEE80211_PARAM_MU_STATIC_GROUPING = 452,/* Enable 1+1+1+1 or 1+1+1 MU static grouping */
	IEEE80211_PARAM_ORTH_SOUND = 453,		/* Enable Orthoginal Sounding */
	IEEE80211_PARAM_CONFIG_MODE = 454,		/* start/end hostapd config */
	IEEE80211_PARAM_MU_ORTH_SOUND = 455,		/* Enable MU Orthoginal Sounding */
	IEEE80211_PARAM_RXSPUR_WAR = 456,		/* Enable RX supr per chan workaround */
	IEEE80211_PARAM_DELTS = 457,			/* Delete TSpec */
	IEEE80211_PARAM_WMM_AUC_AIFSN = 458,		/* Change AIFSN at AUC */
	IEEE80211_PARAM_DISABLE_11B = 459,		/* Disable 11B */
	IEEE80211_PARAM_MU_4P4_CABLED_MODE = 460,	/* MU 4+4 cabled setup mode control */
	IEEE80211_PARAM_NO_CAL_REBOOT = 461,		/* Disable reboot if no calibration file */
	IEEE80211_PARAM_WEATHERCHAN_CAC_ALLOWED = 462,	/* Whether weather chan CAC is allowed */
	IEEE80211_PARAM_BEACON_PHYRATE = 463,		/* Change beacon phyrate */
	IEEE80211_PARAM_BSS_BW = 464,			/* Set/get BSS operating bandwidth */
	IEEE80211_PARAM_BEACON_POWER_BACKOFF = 465,	/* Change the backoff of beacon txpower */
	IEEE80211_PARAM_MU_SEND_MU_GRP_MGMT = 466,	/* Send VHT MU grp id mgmt frame to node */
	IEEE80211_PARAM_TXBF_DBG = 467,			/* Transmit BF debugs */
	IEEE80211_PARAM_VOPT = 468,			/* Enable V optimization */
	IEEE80211_PARAM_AIRQUOTA_NODE = 469,		/* Set node airtime quota configuration */
	IEEE80211_PARAM_QTN_CAL_RESET = 470,		/* Cal reset command */
	IEEE80211_PARAM_QTN_CAL_CONFIG = 471,		/* Cal configuration command */
	IEEE80211_PARAM_QTN_CAL_TIMER = 472,		/* Cal timer trigger command */
	IEEE80211_PARAM_HR_TRIGGER_INTERVAL = 473,	/* Set constant hw reset interval */
	IEEE80211_PARAM_HR_RESET_THRESHOLD = 474,	/* Set EP reboot hard-reset threshold */
	IEEE80211_PARAM_RFIC_TEMPERATURE = 475,		/* Get the temperature of RFIC */
	IEEE80211_PARAM_NRX_DUTY_CYCLE = 476,		/* Set parameters for NRX=8/4 duty cycle */
	IEEE80211_PARAM_DTQEW = 477,			/* Dynamic TQEW */
	IEEE80211_PARAM_MU_FIXED_SS = 478,		/* Set fixed spatial strms per MU pos */
	IEEE80211_PARAM_MU_DUMP_GRCANDS = 479,		/* Dump full list of MU group candidates */
	IEEE80211_PARAM_VHT_OPMODE_ACTION = 480,	/* Send opmode action frm for APUT debug */
	IEEE80211_PARAM_BEACON_WDG_COUNT = 481,		/* Set Beacon-watchdog-count */
	IEEE80211_PARAM_TRIGGER_BURST_BB_RESET = 482,	/* Trigger burst reset for MAC/BB */
	IEEE80211_PARAM_MAX_DEVICE_BW = 483,		/* Set/get the max supported bandwidth */
	IEEE80211_PARAM_BW_AUTO_SELECT = 484,		/* Enable bandwidth automatic selection */
	IEEE80211_PARAM_PLATFORM_ID = 485,		/* Get Platform ID */
	IEEE80211_PARAM_DETECT_TCP = 486,		/* set/get TCP detection in AuC */
	IEEE80211_PARAM_SET_LEGACY_RATE_BASIC = 488,	/* set legacy rates as basic rates */
	IEEE80211_PARAM_CFG_SWCCA_FOR_BW = 489,		/* Configure SWCCA as per system BW */
	IEEE80211_PARAM_FORCE_CRASH = 490,		/* Force a crash on a specified CPU */
	IEEE80211_PARAM_MU_MIX_GSEL_MAX_NODES = 491,	/* Set max nodes for mixed grp selection */
	IEEE80211_PARAM_SET_VCO_LOCK_VERBOSE = 492,	/* Set VCO Lock debugging verbosity */
	IEEE80211_PARAM_EN_AUC_DECAP = 493,		/* Enable the RX decap at AUC side */
	IEEE80211_PARAM_NAC_MONITOR_MODE = 494,		/* Non assoc clients monitoring mode */
	IEEE80211_PARAM_TIMESTAMP = 495,		/* Get MuC timestamp(jiffies) */
	IEEE80211_PARAM_NOTCH_FILTER = 496,		/* Set/get notch filter */
	IEEE80211_PARAM_1024QAM_IOT = 497,		/* Enable 1024 qam for IOT */
	IEEE80211_PARAM_DYNAMIC_NTX = 498,		/* Set dynamic NTX mode */
	IEEE80211_PARAM_BF_GAIN_THRESH = 499,		/* Set BF gain threshold */
	IEEE80211_PARAM_MU_DURATION_ALIGNMENT = 500,	/* set min/max for MU duration alignment */
	IEEE80211_PARAM_MU_DYNAMIC_GROUPING = 501,	/* Configure MU dynamic grouping */
	IEEE80211_PARAM_DSP_TASK_STATS_DUMP = 502,	/* Dump DSP task execution time stats */
	IEEE80211_PARAM_NODE_RATE_STATS = 503,		/* Enable rate stats for node */
	IEEE80211_PARAM_MGMT_POWER_BACKOFF = 504,	/* Change backoff of mgmt frame txpower */
	IEEE80211_PARAM_VAP_DBG_EXT = 506,		/* Set/get VAP extended debug verbosity */
	IEEE80211_PARAM_MU_RTS_CTS = 507,		/* set MU RTS/CTS usage */
	IEEE80211_PARAM_VAP_ACL_LIST = 508,		/* Set VAP 802.11 ACL list */
	IEEE80211_PARAM_CSD_BYPASS = 509,	/* Modify CSD bypass bit in Tx_mimo_ctrl register */
	IEEE80211_PARAM_MAX_BOOT_CAC_DURATION = 510,	/* Max boot CAC duration in seconds */
	IEEE80211_PARAM_GET_REG_DOMAIN_IS_EU = 511,/* Check if reg region falls under EU domain */
	IEEE80211_PARAM_GET_CHAN_AVAILABILITY_STATUS = 512,/* chan availability status */
	IEEE80211_PARAM_CBF_NG = 513,		/* decimation (grouping aka NG), used for CBF */
	IEEE80211_PARAM_CSA_STATUS = 514,		/* CSA status */
	IEEE80211_PARAM_STOP_ICAC = 516,		/* Stop ICAC */
	IEEE80211_PARAM_SET_MAC_NSM_MODE = 517,		/* Set NSM Mode flags for MAC */
	IEEE80211_PARAM_GET_MAC_NSM_MODE = 518,		/* Set NSM Mode flags for MAC */
	IEEE80211_PARAM_EN_AUC_PROCESS = 519,		/* Enable whole RX process at AUC side */
	IEEE80211_PARAM_MU_FIXED_BW = 520,		/* Set fixed bw for MU sounding */
	IEEE80211_PARAM_MU_8STS_SND = 521,		/* Enable 8STS MU sounding */
	IEEE80211_PARAM_MU_TARGET_PWR = 522,		/* Set target tx power for MU data */
	IEEE80211_PARAM_MU_SEND_GRP_ID = 523,	/* send grpid mgt frm just after grpsel sounding */
	IEEE80211_PARAM_SECBUFOFLOW_RX_RECOV = 524,	/* Enable RX recovery on sec buf overflow */
	IEEE80211_PARAM_DYN_BEACON_PERIOD = 525,	/* Enable dynamic beacon interval */
	IEEE80211_PARAM_LIB_HWA_STAT_CLEAR = 526,	/* Clear stats of lib-hwa performance */
	IEEE80211_PARAM_LIB_HWA_STAT_START = 527,	/* Start recording stats of lib-hwa perf */
	IEEE80211_PARAM_LIB_HWA_STAT_STOP = 528,	/* Stop recording stats of lib-hwa perf */
	IEEE80211_PARAM_LIB_HWA_STAT_PRINT = 529,	/* Print current stats of lib-hwa perf */
	IEEE80211_PARAM_CSA_CNT = 530,			/* Set/get CSA count */
	IEEE80211_PARAM_XATTEMPT_DISSOLVE = 531,	/* Enable / disable xattempt dissolve */
	IEEE80211_PARAM_MU_TRACKING_CFG = 532,	/* change MU tracking config limit and threshold */
	IEEE80211_PARAM_PUT_RADIO_UNDER_RESET = 533,	/* Put WMAC/AFE/RFIC under reset */
	IEEE80211_PARAM_CSA_MODE = 534,			/* set/get CSA mode */
	IEEE80211_PARAM_ACS_OBSS_CHK = 535,		/* Enable ACS OBSS secondary chan check */
	IEEE80211_PARAM_MU_GRP_MODE_FLAGS = 536,	/* set/clr mu grouping mode flags */
	IEEE80211_PARAM_STA_DFS_STRICT_MODE = 537,	/* STA DFS - strict mode operation */
	IEEE80211_PARAM_STA_DFS_STRICT_MEASUREMENT_IN_CAC = 538,
				/* STA DFS - Send Measurement report if radar found during CAC */
	IEEE80211_PARAM_STA_DFS_STRICT_TX_CHAN_CLOSE_TIME = 539,
				/* STA DFS - Configure chan tx close time when radar detected */
	IEEE80211_PARAM_DFS_CSA_CNT = 540,		/* set CSA count */
	IEEE80211_PARAM_DFS_CHANS_AVAILABLE_FOR_DFS_REENTRY = 541,/* Check if a DFS chan is avail */
	IEEE80211_PARAM_IS_WEATHER_CHANNEL = 542,	/* check if weather chan */
	IEEE80211_PARAM_MIN_CAC_PERIOD = 543,	/* Get Min CAC period for ICAC sanity checks */
	IEEE80211_PARAM_CUR_CHAN_CHECK_REQUIRED = 544,	/* Check if current chan check required */
	IEEE80211_PARAM_ICAC_STATUS = 545,		/* Get ICAC status */
	IEEE80211_PARAM_TX_ENABLE = 546,		/* Enable transmission */
	IEEE80211_PARAM_SVD_RESET_WAR = 547,		/* SVD reset after each transmission */
	IEEE80211_PARAM_IGNORE_ICAC_SELECTION = 548,	/* Ignore ICAC selection */
	IEEE80211_PARAM_RBS_MBS_ALLOW_TX_FRMS_IN_CAC = 549,
							/* Allow QHOP report Tx while RBS in CAC */
	IEEE80211_PARAM_RBS_DFS_TX_CHAN_CLOSE_TIME = 550,
						/* Config chan tx close time when radar detected */
	IEEE80211_PARAM_PHASE_SHIFT = 551,		/* Set per packet phase shift */
	IEEE80211_PARAM_AQM = 552,			/* Control Active Queue Mgmt settings */
	IEEE80211_PARAM_80211K_NEIGH_REPORT = 553,	/* 802.11k - neighbor report */
	IEEE80211_PARAM_MU_RALG_DBG = 554,		/* MU Rate Adaptation debugging */
	IEEE80211_PARAM_QTN_BGSCAN_DURATION_OBSS = 555,	/* Max duration per chan for OBSS scan */
	IEEE80211_PARAM_MU_NDP_PWR = 556,		/* set tx power for MU NDP */
	IEEE80211_PARAM_USE_RXP_PPCTL = 557,		/* Use ppctl table generated by WMAC RXP */
	IEEE80211_PARAM_FIXED_11AX_TX_RATE = 558,	/* Set 11AX mcs */
	IEEE80211_PARAM_TRANSMIT_HE_TRIG = 559,		/* Transmit a basic trigger frame */
	IEEE80211_PARAM_FIXED_HE_GI_LTF = 560,		/* Fixed 11AX GI and LTF */
	IEEE80211_PARAM_MU_EDCA_AIFSN = 561,	/* Set MU EDCA AC param record AIFSN */
	IEEE80211_PARAM_MU_EDCA_ECWMIN = 562,	/* Set MU EDCA AC param record ECWMIN */
	IEEE80211_PARAM_MU_EDCA_ECWMAX = 563,	/* Set MU EDCA AC param record ECWMAX */
	IEEE80211_PARAM_MU_EDCA_TIMER = 564,	/* Set MU EDCA AC param record MU EDCA TIMER */
	IEEE80211_PARAM_BSSCOLOR = 565,			/* HE BSS Color */
	IEEE80211_PARAM_PPE_THRESHOLD = 566,		/* HE Cap - PPE Threshold Info */
	IEEE80211_PARAM_CH_TXPOWER_BACKOFF = 567,	/* Change backoff txpower for a chan */
	IEEE80211_PARAM_CHANGE_CHAN_DEBUG = 568,	/* Change chan debugging */
	IEEE80211_PARAM_DSP_PRINT_TASK = 569,		/* Enable print DSP tasks */
	IEEE80211_PARAM_MU_GRP_DEBUG_FLAGS = 570,	/* set/clr MuC MU grouping debug flags */
	IEEE80211_PARAM_MU_QMAT_MODE_2U = 571,		/* set 2 users qmat modes for MU-MIMO */
	IEEE80211_PARAM_MU_QMAT_MODE_3U = 572,		/* set 3 users qmat modes for MU-MIMO */
	IEEE80211_PARAM_MU_QMAT_MODE_4U = 573,		/* set 4 users qmat modes for MU-MIMO */
	IEEE80211_PARAM_MU_AIFS_WAR_DATA_THRESH = 574,
						/* Queued data threshold to enable MU AIFS WAR */
	IEEE80211_PARAM_SET_PTA = 575,			/* Set PTA */
	IEEE80211_PARAM_AUC_TX_AGG_DBG_CMD = 576,	/* Aggregation debugging */
	IEEE80211_PARAM_DSP_CALC_NUM_FTONES = 578,	/* Enable calculation num of failed tones */
	IEEE80211_PARAM_DSP_NUM_FTONES_CLR = 579,	/* clear statistics of failed tones */
	IEEE80211_PARAM_MAC_NSM_TIMEOUT = 580,		/* NSM timer value for MAC */
	IEEE80211_PARAM_COUNTRY_STR_ENV = 581,		/* Set country string 3rd byte */
	IEEE80211_PARAM_CCA_SEC80 = 582,		/* Secondary CCA-80 threshold */
	IEEE80211_PARAM_SET_CHAN_OCAC_OFF = 583,	/* Enable OCAC for a chan */
	IEEE80211_PARAM_PHYMODE_REQUIRED = 584,	/* phymode cap (11n, 11ac, 11ax) of peer STA */
	IEEE80211_PARAM_PROFILE_DSP = 585,		/* Control DSP firmware profiling */
	IEEE80211_PARAM_QTN_BGSCAN_BEACON_CHECK = 586,	/* beacon conflict check for QTN bg scan */
	IEEE80211_PARAM_BGSCAN_TURNOFF_RF = 587,	/* Turn off RF on DFS chan in BGscan */
	IEEE80211_PARAM_EXTENDER_SCAN_MBS_INTVL = 588,	/* Interval of RBS scan for MBS in secs */
	IEEE80211_PARAM_EXTENDER_SCAN_MBS_EXPIRY = 589,	/* Expiry of MBS scanned by RBS */
	IEEE80211_PARAM_EXTENDER_SCAN_MBS_MODE = 590,	/* Mode of RBS scans for MBS: normal/bg */
	IEEE80211_PARAM_EXTENDER_FAST_CAC = 591,	/* End CAC when bcn frame received */
	IEEE80211_PARAM_BTM_TERM_DELAY = 592,		/* BTM Terminate delay in sec */
	IEEE80211_PARAM_BTM_BSS_DUR = 593,		/* BTM BSS duration in mins */
	IEEE80211_PARAM_MU_SND_TASK_QUEUE = 595,	/* MU sounding task queue */
	IEEE80211_PARAM_MU_GROUP_TTL = 597,		/* MU group time to live */
	IEEE80211_PARAM_SU_SND_TASK_QUEUE = 598,	/* SU sounding task queue */
	IEEE80211_PARAM_SET_SU_PREC_SND_PERIOD = 599,	/* SU sounding period */
	IEEE80211_PARAM_REPEATER_MAX_LEVEL = 600,	/* Maximum repeaters in a cascaded chain */
	IEEE80211_PARAM_REPEATER_CURR_LEVEL = 601,	/* Current level in a cascaded rptr chain */
	IEEE80211_PARAM_STD_BF_USE_MU_FEEDBACK = 602,	/* Use MU CBFs to compute SU precoding */
	IEEE80211_PARAM_FIXED_HE_TPE = 603,		/* Fixed 11AX packet extension */
	IEEE80211_PARAM_SEND_MU_NDP = 604,		/* Send MU sounding once */
	IEEE80211_PARAM_PROFILE_AXI_BUS = 605,		/* Set AXI bus profile */
	IEEE80211_PARAM_COT_TWEAKS = 606,		/* Chan occupancy time tweaks */
	IEEE80211_PARAM_CFG_4ADDR = 607,		/* use 4-addr headers */
	IEEE80211_PARAM_NO_QTNIE = 608,			/* Don't send QTN IE (for testing) */
	IEEE80211_PARAM_WCAC_CFG = 609,			/* Enable WCAC configuration */
	IEEE80211_PARAM_WCAC_TIME = 610,		/* WCAC time value in seconds for U160M */
	IEEE80211_PARAM_WCAC_STAGE = 611,		/* WCAC stages */
	IEEE80211_PARAM_WCAC_DBG_LEVEL = 612,		/* WCAC debug level */
	IEEE80211_PARAM_REPEATER_BCN = 613,		/* control repeater's AP beacon transmit */
	IEEE80211_PARAM_RX_MAXMPDU = 614,		/* Set Max RX MPDU size to be supported */
	IEEE80211_PARAM_11AX_24G_VHT = 615,		/* VHT support for 2.4GHz 11ax */
	IEEE80211_PARAM_RESP_TX_RATE = 616,	/* Set fixed TX rate for ACK, BA, CTS frames */
	IEEE80211_PARAM_SNDDIM = 617,		/* Set num of sounding dimensions advertised */
	IEEE80211_PARAM_TXOP_RTS_THRESHOLD = 618,/* Set TXOP Duration RTS Threshold in HE Op elem */
	IEEE80211_PARAM_CFG_SWCCA2 = 619,		/* Set SWCCA */
	IEEE80211_PARAM_ACTIVE_TID_THRESH = 620,/* tid activity threshold limit for AC elevation */
	IEEE80211_PARAM_TXOP_RTS_PEER_TH = 621,	/* TXOP RTS Threshold to trigger peer RTS */
	IEEE80211_PARAM_HE_SOUND_FEEDBACK_TYPE = 623,	/* Set HE sounding feedback type */
	IEEE80211_PARAM_HE_NDP_GI_LTF = 624,		/* Set HE NDP GI and LTF */
	IEEE80211_PARAM_ENABLE_Q2Q_HE_SOUNDING = 625,	/* Enable Q2Q 11ax sounding */
	IEEE80211_PARAM_MAX_MSDU_IN_AMSDU = 626,	/* Maximum number of MSDU in AMSDU */
	IEEE80211_PARAM_RESP_SIFS = 627,		/* set 2.4g BA SIFS in us */
	IEEE80211_PARAM_NODE_INACT_THRESH = 628,	/* set/get node inactivity threshold */
	IEEE80211_PARAM_HE_MCS_CAP = 629,		/* Set MCS capability for HE mode */
	IEEE80211_PARAM_HE_NSS_CAP = 630,		/* Set max spatial streams for HE mode */
	IEEE80211_PARAM_HE_TXBF_BIT_CTL = 631,		/* Contrl txbf bit in SIG-A*/
	IEEE80211_PARAM_TRIG_PAD_DURATION = 632,	/* HE Trigger Frame MAC Padding Duration */
	IEEE80211_PARAM_ADAPGAIN_ENABLE = 633,		/* Enable adaptive gain feature */
	IEEE80211_PARAM_ADAPGAIN_PARAMS = 634,		/* Conf adaptive gain control parameters */
	IEEE80211_PARAM_GRCAND_DEBUG_LEVEL = 635,	/* MU grouping debug verbosity */
	IEEE80211_PARAM_AGGR_RTSTHRESHOLD = 637,	/* aggregate frames RTS threshold */
	IEEE80211_PARAM_MOBILITY_DOMAIN = 638,		/* Mobility domain */
	IEEE80211_PARAM_FT_OVER_DS = 639,		/* FT over DS - 802.11r */
	IEEE80211_PARAM_11AX_256BA_TX_RX = 640,		/* allow 256bit BA in RX and TX ADDBA */
	IEEE80211_PARAM_PRECODED_SND = 641,		/* Precoded sounding */
	IEEE80211_PARAM_MU_2P2_CABLED_MODE = 642,	/* 2.4G MU 2+2 cabled setup mode control */
	IEEE80211_PARAM_PIVOTAL_SND = 643,		/* 3SS SU BF sounding mode */
	IEEE80211_PARAM_EXTENDER_MBS_RSSI_MARGIN = 644,	/* MBS RSSI margin */
	IEEE80211_PARAM_SUBBAND_RADAR = 645,		/* Enable subband radar */
	IEEE80211_PARAM_SMARTRTR_CFG = 646,		/* smart retry configuration */
	IEEE80211_PARAM_SR_PARAMS = 647,		/* HE Spatial Reuse Parameter Set */
	IEEE80211_PARAM_SRP_SR_SUPPORT = 648,		/* HE phy Cap - SRP-based SR Support */
	IEEE80211_PARAM_SRP_RESPONDER = 649,		/* HE mac Cap - SRP Responder */
	IEEE80211_PARAM_HE_OPMODE_QOSNULL = 650,/* Send opmode qos null frame for APUT dbg */
	IEEE80211_PARAM_RX_BAR_SYNC = 651,	/* sync rx reorder window on receiving BAR */
	IEEE80211_PARAM_40_INTOLERANT = 652,	/* disable to ignore 40MHz-intolerance */
	IEEE80211_PARAM_SUBBAND_OVER_WCAC = 653,/* Subband decision has higher priority to WCAC */
	IEEE80211_PARAM_AUC_PER_EMU = 654,	/* Emulate PER on AuC */
	IEEE80211_PARAM_TCP_RTS_THRESHOLD = 655,/* Threshold measured in airtime microseconds */
	IEEE80211_PARAM_WFA_MODE = 656,		/* Enable WFA mode */
	IEEE80211_PARAM_BSSCOLOR_COLLISION_PARAM = 657,	/* bsscolor collision param */
	IEEE80211_PARAM_MU_HE_SND_MODE = 658,	/* sounding mode for HE MU */
	IEEE80211_PARAM_RTS_DYN_BW_SIG_MODE = 660,/* Set the mode of RTS dynamic BW signaling */
	IEEE80211_PARAM_HE_ER_NODE = 661,	/* HE-ER-PPDU setting */
	IEEE80211_PARAM_WCAC_TIME_L160M = 662,	/* WCAC time value in seconds for L160M */
	IEEE80211_PARAM_NODE_RATE_OPER_MODE = 663,	/* Node rate operating mode */
	IEEE80211_PARAM_TCP_ACK_COMPRESS_CFG = 664,	/* TCP Ack compress configuration */
	IEEE80211_PARAM_SET_CHAN_BW = 665,	/* config channel and bw */
	IEEE80211_PARAM_MAX_DPDUR = 666,	/* Get the max data PPDU duration target */
	IEEE80211_PARAM_11B_TX_RATE = 667,	/* The manual setting value of 11b TX rate */
	IEEE80211_PARAM_SWBA_CFG = 669,
	IEEE80211_PARAM_EVM_WAR_CMD = 670,	/* EVM WAR */
	IEEE80211_PARAM_MU_USE_SU_FB_PRD = 671,	/* MU will use SU sounding one time per period */
	IEEE80211_PARAM_TX_TCM_RD_HANG_WAR = 672,	/* TX TCM read hang WAR (C0 only) */
	IEEE80211_PARAM_OFDMA_DEF_QMATRIX = 674,	/* Use default Qmatrices for OFDMA */
	IEEE80211_PARAM_ZSDFS = 675,		/* Set and Get Zero Second DFS setting */
	IEEE80211_PARAM_BWSIG = 676,		/* RTS/CTS BW configuration */
	IEEE80211_PARAM_START_OCS_NAC_MON = 677,/* Start ocs nac monitor on specific off-channel */
	IEEE80211_PARAM_MULTI_BSSID = 678,	/* 802.11v multiple BSSID */
	IEEE80211_PARAM_CFG_4ADDR_NODE = 679,	/* use 4-addr headers for a specific node */
	IEEE80211_PARAM_VI_AMSDU_4K = 681,	/* Limit AC VI AMSDUs to 4K */
	IEEE80211_PARAM_VI_RTS = 682,		/* Force RTS on for AC VI */
	IEEE80211_PARAM_HE_MU_RTS_CFG = 683,	/* Enable HE MU RTS usage */
	IEEE80211_PARAM_MU_PPPC_INIT_PWR = 684,	/* Initial power setting for MU PPPC in dBm */
	IEEE80211_PARAM_SYNC_CONFIG = 685,	/* Master device sync BSS config with slave devs */
	IEEE80211_PARAM_AACS_VERBOSITY = 686,		/* AACS: debug level */
	IEEE80211_PARAM_AACS_EXCLUSION_TABLE = 687,	/* AACS: get exclusion table */
	IEEE80211_PARAM_AACS_START_BW = 688,		/* AACS: starting BW */
	IEEE80211_PARAM_AACS_DL_PHYRATE_AVG_WEIGHT = 689,	/* AACS: DL PHY avg rate weight */
	IEEE80211_PARAM_AACS_PERMETRIC_INCLUSION_SPF = 691,	/* AACS: includes SPF */
	IEEE80211_PARAM_AACS_PERMETRIC_INCLUSION_LPF = 692,	/* AACS: includes LPF */
	IEEE80211_PARAM_AACS_USE_THRESHOLD_TABLE = 693,	/* AACS: use threshold tbl */
	IEEE80211_PARAM_AACS_NORM_BW_CTRL = 694,	/* AACS: norm bw ctrl */
	IEEE80211_PARAM_AACS_NORM_BW_LIMIT = 695,	/* AACS: norm bw limit */
	IEEE80211_PARAM_AACS_ICS_BW_CTRL = 696,		/* AACS: ics bw ctrl */
	IEEE80211_PARAM_AACS_ICS_BW_LIMIT = 697,	/* AACS: ics bw limit */
	IEEE80211_PARAM_AACS_ALT_BW_CTRL = 698,		/* AACS: alt bw ctrl */
	IEEE80211_PARAM_AACS_ALT_BW_LIMIT = 699,	/* AACS: alt bw limit */
	IEEE80211_PARAM_AACS_FASTMODE = 700,		/* AACS: fast decision mode */
	IEEE80211_PARAM_AACS_SW_TIMEBASE_SEC = 701,	/* AACS: switching timebase */
	IEEE80211_PARAM_AACS_SW_FACTOR = 702,		/* AACS: switching factor */
	IEEE80211_PARAM_AACS_SW_MAX_SEC = 703,		/* AACS: switching max interval */
	IEEE80211_PARAM_AACS_SW_RESET_SEC = 704,	/* AACS: switching reset interval */
	IEEE80211_PARAM_AACS_MIN_SCAN_SUCCRATE = 705,	/* AACS: min scan success rate */
	IEEE80211_PARAM_AACS_DEC_MAX_SEC = 706,		/* AACS: decision max interval */
	IEEE80211_PARAM_AACS_APCNT_PENALTY = 708,	/* AACS: config APcnt penalty */
	IEEE80211_PARAM_AACS_USE_VNODES = 709,	/* AACS: use virtual nodes */
	IEEE80211_PARAM_AACS_DFS_ENABLE = 710,	/* AACS: allow uncleared chan (TBD) */
	IEEE80211_PARAM_AACS_DFS_ICS,		/* AACS: allow uncleared chan at ICS */
	IEEE80211_PARAM_AACS_DFS_NORM,		/* AACS: allow uncleared chan at norm op */
	IEEE80211_PARAM_AACS_DFS_ALT,		/* AACS: allow uncleared chan at Always-on DFS */
	IEEE80211_PARAM_AACS_DFS_THRES,		/* AACS: allow DFS threshold table */
	IEEE80211_PARAM_AACS_INCL_ALLCHAN,	/* AACS: include all channels */
	IEEE80211_PARAM_AACS_ALT_ENABLE,	/* AACS: enable Always-on DFS */
	IEEE80211_PARAM_AACS_SW_CHAN_FADE_SEC,	/* AACS: The channel fading seconds */
	IEEE80211_PARAM_AACS_STA_CHECK_BITMASK,	/* AACS: The client checking type bitmask */
	IEEE80211_PARAM_AACS_ALT_LIST,		/* AACS: fetch Always-on DFS candidate list */
	IEEE80211_PARAM_MAX_AMPDU_SUBFRM_PER_AC,/* Max subframes for a-MPDU per AC */
	IEEE80211_PARAM_ENABLE_QNODE_BBF,	/* Enable BBF with Quantenna node */
	IEEE80211_PARAM_BF_FEEDBACK_STATIC_BW_MCS,/* Set static BW, MCS for bf reports */
	IEEE80211_PARAM_SU_FIXED_SS,		/* Fix the SS number in SU */
	IEEE80211_PARAM_AACS_ENABLE_STA_D,	/* AACS: enables sta deficiency */
	IEEE80211_PARAM_AACS_ALLOW_STA_D,	/* AACS: allow level of sta deficiency */
	IEEE80211_PARAM_BW_RESUME,		/* 40MHz BW resumption in 2.4GHz (Default on) */
	IEEE80211_PARAM_AACS_ENABLE_OBSS,	/* AACS: enables OBSS */
	IEEE80211_PARAM_ERX_HE_TRIG_HANDL,	/* HE trigger frames handling using early RX */
	IEEE80211_PARAM_AACS_START_CHAN,	/* AACS: convey bw from wireless_conf.txt */
	IEEE80211_PARAM_RXFILTER_BCAST_ALLOW,	/* Enable broadcast packet RX in MAC */
	IEEE80211_PARAM_TX_ACK_FAILURE_COUNT,	/* expected ACKs that were never received */
	IEEE80211_PARAM_RX_PKT_NON_ASSOC,	/* packets Rx from non-associated stations */
	IEEE80211_PARAM_RX_FCS_ERR_CNT,		/* FCS of the MAC frame was in error */
	IEEE80211_PARAM_RX_PLCP_ERR_CNT,	/* parity check of the PLCP header failed */
	IEEE80211_PARAM_GET_SU_SND_INFO,	/* Get SU sounding info */
	IEEE80211_PARAM_OFDMA_NANO_RA,		/* Config OFDMA NANO RA */
	IEEE80211_PARAM_OFDMA_RALG_DBG,		/* OFDMA Rate Adaptation debugging */
	IEEE80211_PARAM_UL_OFDMA_RALG_FLG,	/* UL OFDMA RA flag setting */
	IEEE80211_PARAM_UL_OFDMA_FIXED_RATE,	/* set UL OFDMA fixed rate */
	IEEE80211_PARAM_PRIORITY_REPEATER,	/* use higher WMM prio for repeater's AP iface */
	IEEE80211_PARAM_DUMP_TX_DONE_TIMESTAMP,	/* Dump Tx done timestamp */
	IEEE80211_PARAM_PPS_MAX,		/* Max RX pkts/sec for specific pkt types */
	IEEE80211_PARAM_DEF_3SS_PHSHFT,		/* 3SS phase shifted matrix sampling feature */
	IEEE80211_PARAM_BSS_LOAD_ENABLE,	/* Enable BSS Load IE regardless of RM settings */
	IEEE80211_PARAM_SUBF_SAMPLE,		/* SU BF mode selection feature */
	IEEE80211_PARAM_HE_MULTI_TID_AGG_RX,	/* HE Multi-TID Aggregation Rx Support */
	IEEE80211_PARAM_HE_MULTI_TID_AGG_TX,	/* HE Multi-TID Aggregation Tx Support */
	IEEE80211_PARAM_AUC_TX_SCHED_DBG,	/* AuC TX scheduler debug */
	IEEE80211_PARAM_AUC_TX_SCHED_CFG,	/* AuC TX scheduler commands */
	IEEE80211_PARAM_GET_NDP_BOOST,		/* Get NDP power boost value */
	IEEE80211_PARAM_SET_NDP_BOOST,		/* Set NDP power boost value */
	IEEE80211_PARAM_HE_ACK_ENABLED_RX,	/* Ack-Enabled Aggregation Rx Support */
	IEEE80211_PARAM_HE_ACK_ENABLED_TX,	/* Ack-Enabled Aggregation Tx Support */
	IEEE80211_PARAM_A2M_IPC_CTRL,		/* Initiate/control stress testing for A2M and M2A IPC */
	IEEE80211_PARAM_HE_TX_CFG, /* HE related TX config. Mostly for debug. */
	IEEE80211_SET_SU_MCS_FOR_MU_SND, /* Set SU MCS from which we should start MU sounding */
	IEEE80211_PARAM_DBVC_OFFCHAN_LEVEL, /* DBVC off channel level */
	IEEE80211_PARAM_HE_PPAD_DURATION,	/* HE Nominal Packet Padding Duration */
	IEEE80211_PARAM_LOG_RXSTATUS,		/* Log RxStatus for debugging purpose */
	IEEE80211_PARAM_DBVC_DWELL, /* DBVC off channel dwell time for repeater STA */
	IEEE80211_PARAM_HE_TXTIME_HWCALC,	/* Enable PSDU txtime HW calculation */
	IEEE80211_PARAM_RUBY_DISABLE_AMSDU,	/* Disable / enable AMSDU for peer BBIC3 device */
	IEEE80211_PARAM_IGNORE_CMIC_ERROR,	/* Ignore CMIC error for peer BBIC3 device */
	IEEE80211_PARAM_3SS_SND,	/* Enable / disable OS/alt OS for 3SS clients */
	IEEE80211_PARAM_4SS_SND,	/* Enable / disable OS/alt OS for 4SS clients */
	IEEE80211_PARAM_TRIAL_SND,	/* Enable / disable trial sounding mode */
	IEEE80211_PARAM_ARSSI_ENABLE,		/* Enable analog RSSI feature */
	IEEE80211_PARAM_ARSSI_PARAMS,		/* Conf analog RSSI control parameters */
	IEEE80211_PARAM_NODE_BW_CAP,		/* Node RX operating bandwidth */
	IEEE80211_PARAM_STRICT_IE_LEN,		/* Strict checking of IE lengths */
	IEEE80211_PARAM_SND_BACKOFF_CFG,	/* Configure sounding packets backoff */
	IEEE80211_PARAM_OFDMA_TXBF_QMATRIX,	/* OFDMA form composite Qmatrix from SU TxBF */
	IEEE80211_PARAM_SET_6G_MIN_RATE,	/* Set 6G BSS Min rate */
	IEEE80211_PARAM_GET_6G_MIN_RATE,	/* Get 6G BSS Min rate */
	IEEE80211_PARAM_BSA_PROBE_EVENT_TUNNEL, /* Enable/disable a netlink for probe event */
	IEEE80211_PARAM_NSM_REDUCE_CPU_FREQ,	/* Enable/Disable reducing CPU frequency */
	IEEE80211_PARAM_CONFIG_TX_CHAINS,		/* Tx chain configuration params */
	IEEE80211_PARAM_HE_SR_CONFIG,	/* enable/disable/configure Spatial Reuse */
	IEEE80211_PARAM_UL_SCHED_CFG,		/* UL scheduler configuration */
	IEEE80211_PARAM_RFIC_INTERNAL_TEMPERATURE,	/* per-modem RFIC internal temperature */
	IEEE80211_PARAM_HE_TB_CBF_RATE,		/* Set fixed HE TB CBF rate */
	IEEE80211_PARAM_HE_MU_ACK_TYPE, /* ACK type in MU: MU BAR, basic trigger, BAR */
	IEEE80211_PARAM_VHT_RA_INIT_MCS_SET,	/* Initial RA MCS set for 11ac */
	IEEE80211_PARAM_HE_RA_INIT_MCS_SET,	/* Initial RA MCS set for 11ax */
	IEEE80211_PARAM_RA_QUICK_ADAPT_MIN_PKT,	/* Min packet count for quick adapt mode in SU RA */
	IEEE80211_PARAM_AACS_ENABLE_6G_NON_PSC,	/* AACS: Enable non-PSC channels in 6GHz */
	IEEE80211_PARAM_TX_AGG_TWEAK,		/* TX aggregation tweak parameters*/
	IEEE80211_PARAM_DBVC_SAME_CH_SWITCH,	/* DBVC can switch on same chan */
	IEEE80211_PARAM_QOS_CLASS,		/* Configure QoS control class */
	IEEE80211_PARAM_QOS_TP,			/* Configure QoS throughput control */
	IEEE80211_PARAM_FDR_CONFIG,		/* FDR config */
	IEEE80211_PARAM_FDR_CHANGE_PORT,	/* FDR change bridge and FWT interface */
	IEEE80211_PARAM_FDR_CFG_SYNC_STATUS,	/* FDR config sync status */
	IEEE80211_PARAM_BEACON_POWER_BACKOFF_APPLIED,
	IEEE80211_PARAM_MGMT_POWER_BACKOFF_APPLIED,
	IEEE80211_PARAM_AMSDU_PARTLY_DEAGG_NUM, /* partly de-agg the AMSDU by setting number */
	IEEE80211_PARAM_PN_VALIDATION,		/* Enable/disable PN validation */
	IEEE80211_PARAM_SND_SCHED,		/* Set sounding scheduler params */
	IEEE80211_PARAM_HECAP,			/* Toggle HE capability on VAP basis */
	IEEE80211_PARAM_NODE_RATE_STATS_RESET,	/* Reset node rate stats counters */
	IEEE80211_PARAM_DUMP_VCACHE,		/* V-Cache debug dump */
	IEEE80211_PARAM_MAX_MU_GRP_COUNT,	/* Set number of allowed groups */
	IEEE80211_PARAM_MAX_MU_USR_COUNT,       /* Set number of allowed users for grouping */
	IEEE80211_PARAM_MU_GROUPING_COEF,       /* Set MU grouping algorithm coeficients */
	IEEE80211_PARAM_MU_GROUPING_PERMISSIONS,/* Set MU grouping algorithm permissions */
	IEEE80211_PARAM_MU_GROUPING_SET_PARAM,  /* Set MU grouping algorithm params */
	IEEE80211_PARAM_AACS_INIT,		/* AACS: init to default config */
	IEEE80211_PARAM_PORT_ISOLATE,		/* Configure port isolate. */
	IEEE80211_PARAM_AGGRESS_RPTR,		/* Enable aggressive repeater */
	IEEE80211_PARAM_SET_RFIC6_LUT,		/* Program BB/RF LUT to MainLUT or ACI optm LUT */
	IEEE80211_PARAM_NFR_DBVC_LEVEL, /* No Filter Repeater switch frequency based on DBVC*/
	IEEE80211_PARAM_NFR_DBVC_DWELL, /* No Filter Repeater STA dwell time in msec */
	IEEE80211_PARAM_NFR_SAME_CH_SWITCH, /* No Filter Repeater switching mode */
	IEEE80211_PARAM_DBVC_DBG, /* DBVC debug flags config, shared with NFR */
	IEEE80211_PARAM_HE_TRS_SUPPORT,		/* Enable RX/TX of TRS */
	IEEE80211_PARAM_DEF_PE_DURATION,	/* Default PE duration in HE Operation element*/
	IEEE80211_PARAM_PKTGEN,			/* Config/start/stop packet generation */
	IEEE80211_PARAM_FIXED_TXCHAINS,	/* Transmit becaons and ctrl frames from fixed antset */
	IEEE80211_PARAM_SU_QMAT_SPACE_LIMIT,	/* Limit for SU TXBF QMAT memory allocations */
	IEEE80211_PARAM_RELIABLE_PS_NOTIFICATION,	/* Reliable PS state notification */
	IEEE80211_PARAM_6GHZ_NON_PSC_CHANNELS,	/* Enable/Disable non-PSC channels in 6GHz */
	IEEE80211_PARAM_CBF_CODEBOOK,		/* Set VHT/HE CBF Codebook (CB) setting */
	IEEE80211_PARAM_TAG_EAPOL,	/* Enable/disable EAPOL VLAN tagging */
	IEEE80211_PARAM_DSP_SET_MU_PREC_TYPE, /* Change DSP MU  precoding type */
	IEEE80211_PARAM_DSP_SET_MU_PREC_ALGO, /* Change DSP MU precoding algo */
	IEEE80211_PARAM_DSP_SET_V_SMOOTH, /* Change DSP V-smoothing state */
	IEEE80211_PARAM_GET_DFS_REGION,	/* Get DFS region ID */
	IEEE80211_PARAM_DSP_SET_V_CORR, /* Change DSP V-correlation feature state */
	IEEE80211_PARAM_TPE_TX_PWR_UNIT,	/* TPE Max Tx power Unit Interpretation */
	IEEE80211_PARAM_KEEP_SU_VMAT, /* Keep SU VMats after precoding computation */
	IEEE80211_PARAM_DSP_SET_SU_PREC, /* Change DSP SU precoding calculation mode for debug */
	IEEE80211_PARAM_DSP_SET_MU_PREC, /* Change DSP MU precoding calculation mode for debug */
	IEEE80211_PARAM_NODE_SND_FLAGS, /* Per node sounding debug flags */
	IEEE80211_PARAM_SET_MOBILITY_DOMAIN_IE, /* Enable MD IE for beacon/probe response */
	IEEE80211_PARAM_SSB_MODE, /* Get/Set BB/RF BW larger than BSS BW. SSB mode improves EVM */
	IEEE80211_PARAM_DFS_DOWNGRADE_BW, /* Enable/Disable bw downgrade when radar detected */
	IEEE80211_PARAM_MEM_TRACE_PRINT, /* Print memory tracer table */
	IEEE80211_PARAM_TGEN_FREQUENCY,	/* set/get TGEN frequency */
	IEEE80211_PARAM_TGEN_CHAINS,	/* set TGEN chain enables */
	IEEE80211_PARAM_TGEN_AGC_POWER,	/* set TGEN AGC enable, AGC & RDIV power */
	IEEE80211_PARAM_TGEN_DISABLE,	/* disable TGEN */
	IEEE80211_PARAM_AACS_FLAG_CTRL,	/* AACS: ctrl over AACS flags */
	IEEE80211_PARAM_BBIC_INTERNAL_TEMPERATURE, /* BBIC6 SOC internaltemperature */
	IEEE80211_PARAM_DFSMGMT_XCAC_EN, /* DFS MGMT xCAC enable/disable */
	IEEE80211_PARAM_DSP_SET_WDG_ASSERT, /* MuC-DSP watchdog assertion configuration */
	IEEE80211_PARAM_REASSOC_ALL, /* Reassociate all VAPs */
	IEEE80211_PARAM_REDUCE_RX_GAIN,	/* RX gain mode (2.4GHz only) */
	IEEE80211_PARAM_BB_CS_THR_DEBUG, /* Debugging of Carrier Sense Threshold */
	IEEE80211_PARAM_MU_BFRP_AIFSN, /* Set AIFSN value for MU BF report poll */
	IEEE80211_PARAM_RFIC_PHASE,	/* set/get RFIC phase for 8x8 operation */
	IEEE80211_PARAM_QDMA_WAR, /* Set QDMA WAR mode */
	IEEE80211_PARAM_MAC_BB_RESET_QDMA_WAR, /* Enable MAC/BB reset WAR for BBIC6 - QDMA NDP */
	IEEE80211_PARAM_FIXED_GROUP_MU_RATE, /* Set current fixed MU rate as per-group fixed MU rate */
	IEEE80211_PARAM_REPEATER_REJECT_RBS, /* Do not associate with RBS in repeater mode */
	IEEE80211_PARAM_DYN_RXCHAIN_CTRL, /* Enable or disable RX chain control feature */
	IEEE80211_PARAM_SNAP_HDR_CHECK, /* Check the AMSDU subframe's SA field is SNAP header */
	IEEE80211_PARAM_ARP_PAE_AC, /* Set AC for ARP and PAE payload */
	IEEE80211_PARAM_FORCE_VHTOPI_IN_HEOP, /* Forcefully embed VHTOPI element in HEOP IE */
}; /* ### Do not hard-code integer constants for new enums. ### */

enum IEEE80211_BSSCLR_COLLISION_PARAM {
	BSSCOLOR_COLLISION_PERIOD			= 0,
	BSSCOLOR_COLLISION_DETECT_ENABLE		= 1,
	BSSCOLOR_COLLISION_OBSS_USED_BITMAP_ADD		= 2,
	BSSCOLOR_COLLISION_OBSS_USED_BITMAP_DEL		= 3,
	BSSCOLOR_COLLISION_DUMP_OBSS_USED_BITMAP	= 4,
	BSSCOLOR_COLLISION_REPORT			= 5,
	BSSCOLOR_COLLISION_REPORT_ACTION		= 6,
};

/* bitfields for IEEE80211_PARAM_RESP_TX_RATE command */
#define IEEE80211_PARAM_RESP_TX_RATE_INDEX	0x7f
#define IEEE80211_PARAM_RESP_TX_RATE_INDEX_S	0
#define IEEE80211_PARAM_RESP_TX_RATE_FIXED	(1u << IEEE80211_PARAM_RESP_TX_RATE_FIXED_S)
#define IEEE80211_PARAM_RESP_TX_RATE_FIXED_S	7
#define IEEE80211_PARAM_RESP_TX_RATE_BW		(0x0f << IEEE80211_PARAM_RESP_TX_RATE_BW_S)
#define IEEE80211_PARAM_RESP_TX_RATE_BW_S	8
#define IEEE80211_PARAM_RESP_TX_RATE_FORMAT	(0x0f << IEEE80211_PARAM_RESP_TX_RATE_FORMAT_S)
#define IEEE80211_PARAM_RESP_TX_RATE_FORMAT_S	12

#define IEEE80211_PARAM_11AX_256BA_RX	(1u << IEEE80211_PARAM_11AX_256BA_RX_S)
#define IEEE80211_PARAM_11AX_256BA_RX_S	0
#define IEEE80211_PARAM_11AX_256BA_TX	(1u << IEEE80211_PARAM_11AX_256BA_TX_S)
#define IEEE80211_PARAM_11AX_256BA_TX_S	1

#define IEEE80211_PARAM_EXTRA_ARG_MASK          0xFFFF0000
#define IEEE80211_PARAM_EXTRA_ARG_MASK_S        16
#define IEEE80211_PARAM_EXTRA_ARG(value)        MS((value), IEEE80211_PARAM_EXTRA_ARG_MASK)
#define IEEE80211_PARAM_SHORT_VALUE_MASK        0x0000FFFF
#define IEEE80211_PARAM_SHORT_VALUE_MASK_S      0
#define IEEE80211_PARAM_SHORT_VALUE(value)      MS((value), IEEE80211_PARAM_SHORT_VALUE_MASK)

#define IEEE80211_OCAC_AUTO_WITH_FIRST_DFS_CHAN 0x8000

#define	SIOCG80211STATS			(SIOCDEVPRIVATE+2)
/* NB: require in+out parameters so cannot use wireless extensions, yech */
#define	IEEE80211_IOCTL_GETKEY		(SIOCDEVPRIVATE+3)
#define	IEEE80211_IOCTL_GETWPAIE	(SIOCDEVPRIVATE+4)
#define	IEEE80211_IOCTL_STA_STATS	(SIOCDEVPRIVATE+5)
#define	IEEE80211_IOCTL_STA_INFO	(SIOCDEVPRIVATE+6)
#define	SIOC80211IFCREATE		(SIOCDEVPRIVATE+7)
#define	SIOC80211IFDESTROY		(SIOCDEVPRIVATE+8)
#define	IEEE80211_IOCTL_SCAN_RESULTS	(SIOCDEVPRIVATE+9)
#define SIOCR80211STATS                 (SIOCDEVPRIVATE+0xA) /* This define always has to sync up with SIOCRDEVSTATS in /linux/sockios.h */
#define IEEE80211_IOCTL_GET_ASSOC_TBL	(SIOCDEVPRIVATE+0xB)
#define IEEE80211_IOCTL_GET_RATES	(SIOCDEVPRIVATE+0xC)
#define IEEE80211_IOCTL_SET_RATES	(SIOCDEVPRIVATE+0xD)
#define IEEE80211_IOCTL_EXT		(SIOCDEVPRIVATE+0xF) /* This command is used to support sub-ioctls */

/*
 * ioctl command IEEE80211_IOCTL_EXT is used to support sub-ioctls.
 * The following lists the sub-ioctl numbers
 *
 */
#define SIOCDEV_SUBIO_BASE		(0)
#define SIOCDEV_SUBIO_RST_QUEUE		(SIOCDEV_SUBIO_BASE + 1)
#define SIOCDEV_SUBIO_RADAR_STATUS	(SIOCDEV_SUBIO_BASE + 2)
#define SIOCDEV_SUBIO_GET_PHY_STATS	(SIOCDEV_SUBIO_BASE + 3)
#define SIOCDEV_SUBIO_DISCONN_INFO	(SIOCDEV_SUBIO_BASE + 4)
#define SIOCDEV_SUBIO_SET_BRCM_IOCTL	(SIOCDEV_SUBIO_BASE + 5)
#define SIOCDEV_SUBIO_SCS	        (SIOCDEV_SUBIO_BASE + 6)
#define SIOCDEV_SUBIO_SET_SOC_ADDR_IOCTL	(SIOCDEV_SUBIO_BASE + 7) /* Command to set the SOC addr of the STB to VAP for recording */
#define SIOCDEV_SUBIO_SET_TDLS_OPER	(SIOCDEV_SUBIO_BASE + 8)	/* Set TDLS Operation */
#define SIOCDEV_SUBIO_WAIT_SCAN_TIMEOUT	(SIOCDEV_SUBIO_BASE + 9)
#define SIOCDEV_SUBIO_AP_SCAN_RESULTS	(SIOCDEV_SUBIO_BASE + 10)
#define SIOCDEV_SUBIO_GET_11H_11K_NODE_INFO	(SIOCDEV_SUBIO_BASE + 11)
#define SIOCDEV_SUBIO_GET_DSCP2AC_MAP	(SIOCDEV_SUBIO_BASE + 12)
#define SIOCDEV_SUBIO_SET_DSCP2AC_MAP	(SIOCDEV_SUBIO_BASE + 13)
#define SIOCDEV_SUBIO_SET_MARK_DFS_CHAN	(SIOCDEV_SUBIO_BASE + 14)
#define SIOCDEV_SUBIO_WOWLAN		(SIOCDEV_SUBIO_BASE + 15)
#define SIOCDEV_SUBIO_GET_STA_AUTH	(SIOCDEV_SUBIO_BASE + 16)
#define SIOCDEV_SUBIO_GET_STA_VENDOR	(SIOCDEV_SUBIO_BASE + 17)
#define SIOCDEV_SUBIO_GET_STA_TPUT_CAPS	(SIOCDEV_SUBIO_BASE + 18)
#define SIOCDEV_SUBIO_GET_SWFEAT_MAP	(SIOCDEV_SUBIO_BASE + 19)
#define SIOCDEV_SUBIO_DI_DFS_CHANNELS	(SIOCDEV_SUBIO_BASE + 20) /* Deactive DFS channels */
#define SIOCDEV_SUBIO_SET_ACTIVE_CHANNEL_LIST (SIOCDEV_SUBIO_BASE + 21)
#define SIOCDEV_SUBIO_PRINT_SWFEAT_MAP	(SIOCDEV_SUBIO_BASE + 22)
#define SIOCDEV_SUBIO_SEND_ACTION_FRAME (SIOCDEV_SUBIO_BASE + 23)
#define SIOCDEV_SUBIO_GET_DRIVER_CAPABILITY (SIOCDEV_SUBIO_BASE + 24)
#define SIOCDEV_SUBIO_SET_AP_INFO	(SIOCDEV_SUBIO_BASE + 25)
#define SIOCDEV_SUBIO_GET_LINK_QUALITY_MAX	(SIOCDEV_SUBIO_BASE + 26)
#define SIOCDEV_SUBIO_SET_CHANNEL_POWER_TABLE	(SIOCDEV_SUBIO_BASE + 27)
#define SIOCDEV_SUBIO_SET_WEATHER_CHAN	(SIOCDEV_SUBIO_BASE + 28)
#define SIOCDEV_SUBIO_GET_CHANNEL_POWER_TABLE	(SIOCDEV_SUBIO_BASE + 29)
#define SIOCDEV_SUBIO_SETGET_CHAN_DISABLED	(SIOCDEV_SUBIO_BASE + 30)
#define SIOCDEV_SUBIO_SET_SEC_CHAN		(SIOCDEV_SUBIO_BASE + 31)
#define SIOCDEV_SUBIO_GET_SEC_CHAN		(SIOCDEV_SUBIO_BASE + 32)
#define SIOCDEV_SUBIO_SET_DSCP2TID_MAP		(SIOCDEV_SUBIO_BASE + 33)
#define SIOCDEV_SUBIO_GET_DSCP2TID_MAP		(SIOCDEV_SUBIO_BASE + 34)
#define SIOCDEV_SUBIO_GET_TX_AIRTIME		(SIOCDEV_SUBIO_BASE + 35)
#define SIOCDEV_SUBIO_GET_CHAN_PRI_INACT	(SIOCDEV_SUBIO_BASE + 36)
#define SIOCDEV_SUBIO_GET_SUPP_CHAN		(SIOCDEV_SUBIO_BASE + 37)
#define SIOCDEV_SUBIO_GET_CLIENT_MACS		(SIOCDEV_SUBIO_BASE + 38)
#define SIOCDEV_SUBIO_SAMPLE_ALL_DATA		(SIOCDEV_SUBIO_BASE + 39)
#define SIOCDEV_SUBIO_GET_ASSOC_DATA		(SIOCDEV_SUBIO_BASE + 40)
#define SIOCDEV_SUBIO_GET_VAP_EXTSTATS		(SIOCDEV_SUBIO_BASE + 41)
#define SIOCDEV_SUBIO_GET_CCA_STATS		(SIOCDEV_SUBIO_BASE + 42)
#define SIOCDEV_SUBIO_GET_DFS_CHANNELS_STATUS	(SIOCDEV_SUBIO_BASE + 43)
#define SIOCDEV_SUBIO_SET_CS_ARGS		(SIOCDEV_SUBIO_BASE + 44)
#define SIOCDEV_SUBIO_GET_CS_ARGS		(SIOCDEV_SUBIO_BASE + 45)
#define SIOCDEV_SUBIO_SET_DSCP_VAP_LINK		(SIOCDEV_SUBIO_BASE + 46)
#define SIOCDEV_SUBIO_GET_DSCP_VAP_LINK		(SIOCDEV_SUBIO_BASE + 47)
#define SIOCDEV_SUBIO_SET_BSA_STATUS		(SIOCDEV_SUBIO_BASE + 48)
#define SIOCDEV_SUBIO_GET_BSA_INTF_INFO		(SIOCDEV_SUBIO_BASE + 49)
#define SIOCDEV_SUBIO_SET_BSA_MAC_FILTER_POLICY (SIOCDEV_SUBIO_BASE + 50)
#define SIOCDEV_SUBIO_UPDATE_MACFILTER_LIST	(SIOCDEV_SUBIO_BASE + 51)
#define SIOCDEV_SUBIO_WLAN_LINK_MONITOR		(SIOCDEV_SUBIO_BASE + 52)
#define SIOCDEV_SUBIO_PREMIER			(SIOCDEV_SUBIO_BASE + 53)
#define SIOCDEV_SUBIO_GET_BSA_FAT_INFO		(SIOCDEV_SUBIO_BASE + 54)
#define SIOCDEV_SUBIO_BSA_START_FAT_MON		(SIOCDEV_SUBIO_BASE + 55)
#define SIOCDEV_SUBIO_GET_BSA_STA_STATS		(SIOCDEV_SUBIO_BASE + 56)
#define SIOCDEV_SUBIO_SEND_BTM_REQ_FRM		(SIOCDEV_SUBIO_BASE + 57)
#define SIOCDEV_SUBIO_GET_STA_EXT_CAP_IE	(SIOCDEV_SUBIO_BASE + 58)
#define SIOCDEV_SUBIO_QOS_QUOTA			(SIOCDEV_SUBIO_BASE + 59)
#define SIOCDEV_SUBIO_DCS			(SIOCDEV_SUBIO_BASE + 60)
#define SIOCDEV_SUBIO_GET_NAC_STATS		(SIOCDEV_SUBIO_BASE + 61)
#define SIOCDEV_SUBIO_SET_CHANNEL_POWER_TABLE_DNTX	(SIOCDEV_SUBIO_BASE + 62)
#define SIOCDEV_SUBIO_GET_CHANNEL_POWER_TABLE_DNTX	(SIOCDEV_SUBIO_BASE + 63)
#define SIOCDEV_SUBIO_GET_NODE_LIST		(SIOCDEV_SUBIO_BASE + 64)
#define SIOCDEV_SUBIO_GET_NODE_INFOSET		(SIOCDEV_SUBIO_BASE + 65)
#define SIOCDEV_SUBIO_GET_BSA_ASSOC_STA_STATS	(SIOCDEV_SUBIO_BASE + 66)
#define SIOCDEV_SUBIO_ERW_ENTRY			(SIOCDEV_SUBIO_BASE + 67)
#define SIOCDEV_SUBIO_GET_SCAN_CHAN_LIST	(SIOCDEV_SUBIO_BASE + 68)
#define SIOCDEV_SUBIO_SET_SCAN_CHAN_LIST	(SIOCDEV_SUBIO_BASE + 69)
#define SIOCDEV_SUBIO_SET_MFR			(SIOCDEV_SUBIO_BASE + 70)
#define SIOCDEV_SUBIO_GET_OCAC_OFF_CHANLIST	(SIOCDEV_SUBIO_BASE + 71)
#define SIOCDEV_SUBIO_SET_EXTCAP_IE		(SIOCDEV_SUBIO_BASE + 72)
#define SIOCDEV_SUBIO_GET_FREQ_RANGE		(SIOCDEV_SUBIO_BASE + 73)
#define SIOCDEV_SUBIO_GET_NODE_INFOSET_ALL	(SIOCDEV_SUBIO_BASE + 74)
#define SIOCDEV_SUBIO_SET_APP_FRAMES		(SIOCDEV_SUBIO_BASE + 75)
#define SIOCDEV_SUBIO_SET_FT_ADD_NODE		(SIOCDEV_SUBIO_BASE + 76)
#define SIOCDEV_SUBIO_GET_CHAN_AVAIL		(SIOCDEV_SUBIO_BASE + 77)
#define SIOCDEV_SUBIO_GET_OPCLASS_INFO		(SIOCDEV_SUBIO_BASE + 78)
#define SIOCDEV_SUBIO_SET_AACS_PARAM		(SIOCDEV_SUBIO_BASE + 79)
#define SIOCDEV_SUBIO_GET_AACS_PARAM		(SIOCDEV_SUBIO_BASE + 80)
#define SIOCDEV_SUBIO_SET_REMAIN_ON_CHAN	(SIOCDEV_SUBIO_BASE + 81)
#define SIOCDEV_SUBIO_SET_CANCEL_REMAIN_ON_CHAN	(SIOCDEV_SUBIO_BASE + 82)
#define SIOCDEV_SUBIO_GET_IF_INFOSET		(SIOCDEV_SUBIO_BASE + 83)
#define SIOCDEV_SUBIO_GET_SPDIA_STATS		(SIOCDEV_SUBIO_BASE + 84)
#define SIOCDEV_SUBIO_SET_AUTH			(SIOCDEV_SUBIO_BASE + 85)
#define SIOCDEV_SUBIO_SET_ASSOC_RESP            (SIOCDEV_SUBIO_BASE + 86)
#define SIOCDEV_SUBIO_SET_REASSOC_RESP          (SIOCDEV_SUBIO_BASE + 87)
#define SIOCDEV_SUBIO_GET_CHAN_ACTIVE		(SIOCDEV_SUBIO_BASE + 88)
#define SIOCDEV_SUBIO_GET_SIS_INFOSET_ALL	(SIOCDEV_SUBIO_BASE + 89)
#define SIOCDEV_SUBIO_MFR_TX_MGMT_FRM		(SIOCDEV_SUBIO_BASE + 90)
#define SIOCDEV_SUBIO_GET_DFS_MGMT_INFO		(SIOCDEV_SUBIO_BASE + 91)
#define SIOCDEV_SUBIO_DFS_MGMT_CMD		(SIOCDEV_SUBIO_BASE + 92)
#define SIOCDEV_SUBIO_SET_CHAN_LIST		(SIOCDEV_SUBIO_BASE + 93)
#define SIOCDEV_SUBIO_GET_STA_VER_FLAGS		(SIOCDEV_SUBIO_BASE + 94)
#define SIOCDEV_SUBIO_QRPE_TRIGGER_SCAN		(SIOCDEV_SUBIO_BASE + 95)
#define SIOCDEV_SUBIO_SET_SAE_PWE		(SIOCDEV_SUBIO_BASE + 96)
#define SIOCDEV_SUBIO_QRPE_REQ_XCAC		(SIOCDEV_SUBIO_BASE + 97)
#define SIOCDEV_SUBIO_GET_CHAN_PHY_INFO		(SIOCDEV_SUBIO_BASE + 98)
#define SIOCDEV_SUBIO_GET_CHAN_AVAIL_STAT	(SIOCDEV_SUBIO_BASE + 99)
#define SIOCDEV_SUBIO_THERMAL_CMD		(SIOCDEV_SUBIO_BASE + 100)

struct ieee8011req_sta_ver_flags {
	uint8_t		macaddr[IEEE80211_ADDR_LEN];
	uint32_t	ver_flags;
};

enum IEEE80211_DFS_MGMT_SUB_CMD {
	IEEE80211_DFS_MGMT_IC_INFO = 0,
	IEEE80211_DFS_MGMT_DFS_REGION,
	IEEE80211_DFS_MGMT_OFFCHAN_CCA_INFO,
	IEEE80211_DFS_MGMT_CURCHAN_CCA_INFO,
	IEEE80211_DFS_MGMT_XCAC_INFO,
	IEEE80211_DFS_MGMT_ASSOC_INFO,
	IEEE80211_DFS_MGMT_CHAN_BW_INFO,
	IEEE80211_DFS_MGMT_CHAN_LIST_INFO,
};

struct ieee80211_dfs_mgmt_dfs_region {
	uint32_t dfs_code;
};

struct ieee80211_dfs_mgmt_ic_info {
	enum ieee80211_opmode opmode;
	uint8_t rx_chains;
	uint8_t dcdc_mode;
	uint8_t wea_cac_allowed;
	uint8_t zcac_swfeat_support;
};

struct ieee80211_dfs_mgmt_offchan_cca_info {
	uint16_t ch_ieee;
	uint16_t cca_intf;
};

struct ieee80211_dfs_mgmt_curchan_cca_info {
	uint32_t cca_intf;
	uint32_t cca_trfc;
};

struct ieee80211_dfs_mgmt_xcac_info {
	uint8_t running_method;
	uint8_t stage;
	uint16_t clearing_ch;
};

struct ieee80211_dfs_mgmt_assoc_info {
	uint8_t assoc_or_not;
	uint8_t long_range_client_existed;
};

struct ieee80211_dfs_mgmt_chan_bw_info {
#define IEEE80211_DFS_MGMT_NOSCAN	0
#define IEEE80211_DFS_MGMT_SCAN		1
#define IEEE80211_DFS_MGMT_BGSCAN	2
	uint32_t cur_chan_ieee;
	uint32_t cur_bw;
	uint8_t scan_type;
};

struct dfs_mgmt_channel_t {
	uint16_t chan_ieee;
	uint16_t cca_intf;
	/*
	 * bit 1:0 : indicate channel avialalble state
	 * bit 4   : indicate channel DFS flag per 80MHz
	 * bit 5   : indicate weather channel flag per 80MHz
	 * bit 6   : indicate channel primary inactive flag per 80MHz
	 */
	uint8_t status;
};

struct ieee80211_dfs_mgmt_chan_list_info {
	uint8_t chan_num;
	struct dfs_mgmt_channel_t chan_list[24];
};

struct ieee80211_dfs_mgmt_info {
	uint32_t cmd;
	uint32_t length;
	union content {
		struct ieee80211_dfs_mgmt_ic_info ic_info;
		struct ieee80211_dfs_mgmt_dfs_region dfs_region;
		struct ieee80211_dfs_mgmt_offchan_cca_info offchan_cca_info;
		struct ieee80211_dfs_mgmt_curchan_cca_info curchan_cca_info;
		struct ieee80211_dfs_mgmt_xcac_info xcac_info;
		struct ieee80211_dfs_mgmt_assoc_info assoc_info;
		struct ieee80211_dfs_mgmt_chan_bw_info chan_bw_info;
		struct ieee80211_dfs_mgmt_chan_list_info chan_list_info;
	} data;
};

enum IEEE80211_DFS_MGMT_CMD_FROM_APP {
	IEEE80211_DFS_MGMT_START_SCAN = 0,
	IEEE80211_DFS_MGMT_STATE_SYNC = 1,
	IEEE80211_DFS_MGMT_XCAC_SEL_RESULT = 2,
};

struct ieee80211_dfs_mgmt_scan_req_cmd {
	uint16_t scan_ch_ieee;
	uint16_t scan_bw;
};

struct ieee80211_dfs_mgmt_state_sync_cmd {
	int state;
};

struct ieee80211_dfs_mgmt_xcac_sel_result_cmd {
	uint16_t xcac_method;
	uint16_t new_oper_chan;
	uint16_t bw;
};

struct ieee80211_dfs_mgmt_cmd {
	uint32_t subcmd;
	uint32_t length;
	union subcmd_content {
		struct ieee80211_dfs_mgmt_scan_req_cmd scan_req_cmd;
		struct ieee80211_dfs_mgmt_state_sync_cmd state_sync;
		struct ieee80211_dfs_mgmt_xcac_sel_result_cmd xcac_sel_result;
	} data;
};

/* These values must be kept in sync with the wifi_sr script */
enum ieee80211_sr_param {
	IEEE80211_SR_PARAM_SRP_DISALLOW				= 0,
	IEEE80211_SR_PARAM_NON_SRG_OBSS_PD_DISALLOW		= 1,
	IEEE80211_SR_PARAM_NON_SRG_OBSS_PD_MAX_OFFSET		= 2,
	IEEE80211_SR_PARAM_HE_SIGA_SR_VAL15_ALLOW		= 3,
	IEEE80211_SR_PARAM_SRG_OBSS_PD_MIN_OFFSET		= 4,
	IEEE80211_SR_PARAM_SRG_OBSS_PD_MAX_OFFSET		= 5,
	IEEE80211_SR_PARAM_SRG_BSSCOLOR_BITMAP_ADD		= 6,
	IEEE80211_SR_PARAM_SRG_BSSCOLOR_BITMAP_DEL		= 7,
	IEEE80211_SR_PARAM_SRG_BSSCOLOR_BITMAP_GET_LOW		= 8,
	IEEE80211_SR_PARAM_SRG_BSSCOLOR_BITMAP_GET_HIGH		= 9,
	IEEE80211_SR_PARAM_SRG_PART_BSSID_BITMAP_ADD		= 10,
	IEEE80211_SR_PARAM_SRG_PART_BSSID_BITMAP_DEL		= 11,
	IEEE80211_SR_PARAM_SRG_PART_BSSID_BITMAP_GET_LOW	= 12,
	IEEE80211_SR_PARAM_SRG_PART_BSSID_BITMAP_GET_HIGH	= 13,
};

enum L2_EXT_FILTER_PORT {
	L2_EXT_FILTER_EMAC_0_PORT = 0,
	L2_EXT_FILTER_EMAC_1_PORT = 1,
	L2_EXT_FILTER_PCIE_PORT = 2
};

struct ieee80211_clone_params {
	char icp_name[IFNAMSIZ];		/* device name */
	uint16_t icp_opmode;			/* operating mode */
	uint16_t icp_flags;			/* see below */
#define	IEEE80211_CLONE_BSSID	0x0001		/* allocate unique mac/bssid */
#define	IEEE80211_NO_STABEACONS	0x0002		/* Do not setup the station beacon timers */
};

enum power_table_sel {
	PWR_TABLE_SEL_BOOTCFG_ONLY = 0,	/* Search for power table in bootcfg only */
	PWR_TABLE_SEL_BOOTCFG_PRIOR,	/* Search for power table in bootcfg at first, if not find, then search /etc/ */
	PWR_TABLE_SEL_IMAGE_PRIOR,	/* Search for power table in /etc/ at first, if not find, then search bootcfg */
	PWR_TABLE_SEL_IMAGE_ONLY,	/* Search for power table in /etc/ only */
	PWR_TABLE_SEL_IMAGE_RCFG_PRIOR,	/* Search for power table in rf cfg  path first, if not found then search /etc/ */
	PWR_TABLE_SEL_IMAGE_RCFG_ONLY,	/* Search for power table in rf cfg  path only */
	PWR_TABLE_SEL_MAX = PWR_TABLE_SEL_IMAGE_RCFG_ONLY,
};

/* APPIEBUF related definitions */
/* Management frame type to which application IE is added */
enum {
	IEEE80211_APPIE_FRAME_BEACON		= 0,
	IEEE80211_APPIE_FRAME_PROBE_REQ		= 1,
	IEEE80211_APPIE_FRAME_PROBE_RESP	= 2,
	IEEE80211_APPIE_FRAME_ASSOC_REQ		= 3,
	IEEE80211_APPIE_FRAME_ASSOC_RESP	= 4,
	IEEE80211_APPIE_FRAME_TDLS_ACT		= 5,
	IEEE80211_APPIE_FRAME_TOT		= 6
};

/* the beaconing schemes - the mapping between 8 VAPs and 4 HW TX queues for beacon */
enum {
	/*
	 * Scheme 0 - default
	 * VAP0/VAP4 - HW queue0
	 * VAP1/VAP5 - HW queue1
	 * VAP2/VAP6 - HW queue2
	 * VAP3/VAP7 - HW queue3
	 */
	QTN_BEACONING_SCHEME_0 = 0,
	/*
	 * Scheme 1:
	 * VAP0/VAP1 - HW queue0
	 * VAP2/VAP3 - HW queue1
	 * VAP4/VAP5 - HW queue2
	 * VAP6/VAP7 - HW queue3
	 */
	QTN_BEACONING_SCHEME_1 = 1
};

/*
 * This enum must be kept in sync with tdls_operation_string.
 * enum ieee80211_tdls_operation - values for tdls_oper callbacks
 * @IEEE80211_TDLS_DISCOVERY_REQ: Send a TDLS discovery request
 * @IEEE80211_TDLS_SETUP: Setup TDLS link
 * @IEEE80211_TDLS_TEARDOWN: Teardown a TDLS link which is already established
 * @IEEE80211_TDLS_ENABLE_LINK: Enable TDLS link
 * @IEEE80211_TDLS_DISABLE_LINK: Disable TDLS link
 * @IEEE80211_TDLS_ENABLE: Enable TDLS function
 * @IEEE80211_TDLS_DISABLE: Disable TDLS function
 * @IEEE80211_TDLS_PTI_REQ: Send a TDLS Peer Traffic Indication Frame
 */
enum ieee80211_tdls_operation {
	IEEE80211_TDLS_DISCOVERY_REQ	= 0,
	IEEE80211_TDLS_SETUP			= 1,
	IEEE80211_TDLS_TEARDOWN			= 2,
	IEEE80211_TDLS_ENABLE_LINK		= 3,
	IEEE80211_TDLS_DISABLE_LINK		= 4,
	IEEE80211_TDLS_ENABLE			= 5,
	IEEE80211_TDLS_DISABLE			= 6,
	IEEE80211_TDLS_PTI_REQ			= 7,
	IEEE80211_TDLS_SWITCH_CHAN		= 8,
};

struct ieee80211_tdls_oper_data {
	uint8_t	dest_mac[IEEE80211_ADDR_LEN];
	uint8_t	oper;
} __packed;

struct ieee80211_tdls_action_data {
	uint8_t	dest_mac[IEEE80211_ADDR_LEN];	/* Destination address of tdls action */
	uint8_t	action;		/* TDLS action frame type */
	uint16_t status;	/* Statu code */
	uint8_t	dtoken;		/* Dialog token */
	uint32_t ie_buflen;	/* Subsequent IEs length*/
	uint8_t	ie_buf[0];	/* Subsequent IEs */
} __packed;

struct ieee80211req_getset_appiebuf {
	uint32_t app_frmtype;	/* management frame type for which buffer is added */
	uint32_t app_buflen;	/* application-supplied buffer length */
#define F_QTN_IEEE80211_PAIRING_IE	0x1
#define F_QTN_IEEE80211_WPSIE_APPEXT	0x2
#define F_QTN_IEEE80211_RPE_APPIE	0x4
	uint8_t	flags;		/* flags here is used to check whether QTN pairing IE exists */
	uint8_t	app_buf[0];	/* application-supplied IE(s) */
};

/* Action frame payload */
struct action_frame_payload {
	u_int16_t	length;                 /* action frame payload length */
	u_int8_t	data[0];                /* action frame payload data */
} __packed;

/* Structure used to send action frame from hostapd */
struct app_action_frame_buf {
	u_int8_t	cat;			/* action frame category */
	u_int8_t	action;			/* action frame action */
	u_int8_t	dst_mac_addr[IEEE80211_ADDR_LEN];
	struct action_frame_payload frm_payload;
}__packed;

struct app_ie {
	u_int8_t id;
	u_int16_t len;
	union {
		struct {
			u_int8_t interworking;
			u_int8_t an_type;
			u_int8_t hessid[IEEE80211_ADDR_LEN];
		}__packed interw;
	}u;
}__packed;

struct qtn_cca_args
{
	uint32_t cca_channel;
	uint32_t duration;
};

/* Flags ORed by application to set filter for receiving management frames */
enum {
	IEEE80211_FILTER_TYPE_BEACON		= 1<<0,
	IEEE80211_FILTER_TYPE_PROBE_REQ		= 1<<1,
	IEEE80211_FILTER_TYPE_PROBE_RESP	= 1<<2,
	IEEE80211_FILTER_TYPE_ASSOC_REQ		= 1<<3,
	IEEE80211_FILTER_TYPE_ASSOC_RESP	= 1<<4,
	IEEE80211_FILTER_TYPE_AUTH		= 1<<5,
	IEEE80211_FILTER_TYPE_DEAUTH		= 1<<6,
	IEEE80211_FILTER_TYPE_DISASSOC		= 1<<7,
	IEEE80211_FILTER_TYPE_ACTION		= 1<<8,
	IEEE80211_FILTER_TYPE_ALL		= 0x1FF	/* used to check the valid filter bits */
};

struct ieee80211req_set_filter {
	uint32_t app_filterype;		/* management frame filter type */
};

/*AXI bus profile*/
#define QTN_AXI_BUS_PROF_CMD	0x0000000F
#define QTN_AXI_BUS_PROF_CMD_S	0
#define QTN_AXI_BUS_PROF_SUB_ID	0xF0000000
#define QTN_AXI_BUS_PROF_SUB_ID_S 28
#define QTN_AXI_BUS_PROF_SRC	0x0F000000
#define QTN_AXI_BUS_PROF_SRC_S	24
#define QTN_AXI_BUS_PROF_DST	0x00F00000
#define QTN_AXI_BUS_PROF_DST_S	20
#define QTN_AXI_BUS_PROF_STOP	0
#define QTN_AXI_BUS_PROF_START	1
#define QTN_AXI_BUS_PROF_DUMP	2
#define QTN_PROF_VALUE_MASK	0xF00000FF

/* Tx Restrict */
#define IEEE80211_TX_RESTRICT_RTS_MIN		2
#define IEEE80211_TX_RESTRICT_RTS_DEF		6
#define IEEE80211_TX_RESTRICT_LIMIT_MIN		2
#define IEEE80211_TX_RESTRICT_LIMIT_DEF		12
#define IEEE80211_TX_RESTRICT_RATE		5

/* Beacon txpower backoff */
#define IEEE80211_BEACON_POWER_BACKOFF_MIN	0	/* dB */
#define IEEE80211_BEACON_POWER_BACKOFF_MAX	20	/* dB */

/* Management frame txpower backoff */
#define IEEE80211_MGMT_POWER_BACKOFF_MIN	0	/* dB */
#define IEEE80211_MGMT_POWER_BACKOFF_MAX	20	/* dB */

#define IEEE80211_QTN_BGSCAN_DWELLTIME_ACTIVE_MAX	50 /* milliseconds */
#define IEEE80211_QTN_BGSCAN_DWELLTIME_ACTIVE_MIN	8 /* milliseconds */
#define IEEE80211_QTN_BGSCAN_DWELLTIME_PASSIVE_MAX	50 /* milliseconds */
#define IEEE80211_QTN_BGSCAN_DWELLTIME_PASSIVE_MIN	8 /* milliseconds */

/* Compatibility fix bitmap for various vendor peer */
#define VENDOR_FIX_BRCM_DHCP			0x01
#define VENDOR_FIX_BRCM_REPLACE_IGMP_SRCMAC	0x02
#define VENDOR_FIX_BRCM_REPLACE_IP_SRCMAC	0x04
#define VENDOR_FIX_BRCM_DROP_STA_IGMPQUERY	0x08
#define VENDOR_FIX_BRCM_AP_GEN_IGMPQUERY	0x10
#define VENDOR_FIX_BRCM_VHT			0x20

enum vendor_fix_idx {
	VENDOR_FIX_IDX_BRCM_DHCP = 1,
	VENDOR_FIX_IDX_BRCM_IGMP = 2,
	VENDOR_FIX_IDX_BRCM_REPLACE_IP_SRCMAC = 3,
	VENDOR_FIX_IDX_BRCM_DROP_STA_IGMPQUERY = 4,
	VENDOR_FIX_IDX_BRCM_AP_GEN_IGMPQUERY = 5,
	VENDOR_FIX_IDX_CC_3SS = 6,
	VENDOR_FIX_IDX_CC_4SS = 7,
	VENDOR_FIX_IDX_CC_RFGAIN_PPPC = 8,
	VENDOR_FIX_IDX_BRCM_VHT = 9,
	VENDOR_FIX_IDX_MAX,
};

struct ieee80211req_wowlan {
	uint32_t is_op;
	uint8_t *is_data;
	int32_t is_data_len;
};

/*
 * BBIC5: WLAN link monitoring and report
 */
enum ieee80211_wlmonitor_type {
	WLAN_MONITOR_LINK_RATE,
	WLAN_MONITOR_LINK_PERIOD,
	WLAN_MONITOR_ENABLE,
	WLAN_MONITOR_TYPE_MAX
};
enum ieee80211_wlmonitor_severity {
	WLAN_MONITOR_ERROR,
	WLAN_MONITOR_WARNING,
	WLAN_MONITOR_SEVERITY_MAX
};
enum ieee80211_wlmonitor_period_unit {
	WLAN_MONITOR_HOUR,
	WLAN_MONITOR_SECOND,
	WLAN_MONITOR_UNIT_MAX
};

struct ieee80211req_wlan_link_monitor {
	uint8_t		op_type;	/* 0: link rate; 1: period; 2: enable */
	uint8_t		op_severity;	/* 0: error; 1: warning */
	uint8_t		op_unit;	/* 0: hour; 1: second*/
	union {
		uint8_t macaddr[IEEE80211_ADDR_LEN];
		uint32_t threshold;
	} op_value;
};

#define IEEE80211REQ_PREMIER_FLAG_GET                 0x80000000
enum ieee80211_premier_op {
	/* set part start */
	IEEE80211_PREMIER_LIST_SET,

	/* get part start */
	IEEE80211_PREMIER_LIST_GET = IEEE80211REQ_PREMIER_FLAG_GET,
};

#define IEEE80211_PREMIER_NODE_MAX	8
struct ieee80211_premier_list {
	uint8_t		ipl_num;
	uint8_t		ipl_macs[IEEE80211_PREMIER_NODE_MAX][IEEE80211_ADDR_LEN];
};
struct ieee80211req_premier {
	uint32_t ipl_op;
	uint8_t *ipl_data;
	int32_t ipl_data_len;
};

#define IEEE80211_NODE_AIRQUOTA_MAX	10
enum ieee80211_qos_quota_op {
	IEEE80211_QUOTA_NODE_GET = 1,
	IEEE80211_QUOTA_NODE_SET,
	IEEE80211_QOS_CLASS_SET,
	IEEE80211_QOS_CLASS_GET,
	IEEE80211_QUOTA_VAP_SET,
	IEEE80211_QUOTA_VAP_GET,
};

struct ieee80211_qos_quota_node {
	uint32_t	quota;
	uint8_t		mac[IEEE80211_ADDR_LEN];
	uint8_t		class;
};

struct ieee80211_qos_quota_node_list {
	uint8_t				ipl_num;
	struct ieee80211_qos_quota_node	ipl_node[IEEE80211_NODE_AIRQUOTA_MAX];
};

struct ieee80211req_qos_quota {
	uint32_t ipl_op;
	uint8_t *ipl_data;
	int32_t ipl_data_len;
};

struct ieee80211_qos_entity_class {
	uint32_t entity;
	uint32_t class;
};

struct ieee80211_qos_quota_vap {
	uint32_t quota;
	uint32_t class;
};

#define IEEE80211_AUTHDESCR_KEYMGMT_NONE		0x00
#define IEEE80211_AUTHDESCR_KEYMGMT_EAP			0x01
#define IEEE80211_AUTHDESCR_KEYMGMT_PSK			0x02
#define IEEE80211_AUTHDESCR_KEYMGMT_WEP			0x03

#define IEEE80211_AUTHDESCR_KEYPROTO_NONE		0x00
#define IEEE80211_AUTHDESCR_KEYPROTO_WPA		0x01
#define IEEE80211_AUTHDESCR_KEYPROTO_RSN		0x02

#define IEEE80211_AUTHDESCR_ALGO_POS			0x00
#define IEEE80211_AUTHDESCR_KEYMGMT_POS			0x01
#define IEEE80211_AUTHDESCR_KEYPROTO_POS		0x02
#define IEEE80211_AUTHDESCR_CIPHER_POS			0x03


struct ieee80211req_auth_description {
	uint8_t macaddr[IEEE80211_ADDR_LEN];
	uint32_t description;
};

enum ieee80211_extender_role {
	IEEE80211_EXTENDER_ROLE_NONE = 0x00,
	IEEE80211_EXTENDER_ROLE_MBS = 0x01,
	IEEE80211_EXTENDER_ROLE_RBS = 0x02,
	IEEE80211_EXTENDER_ROLE_NONE_AP = 0x03,
	IEEE80211_EXTENDER_ROLE_MIN = IEEE80211_EXTENDER_ROLE_NONE,
	IEEE80211_EXTENDER_ROLE_MAX = IEEE80211_EXTENDER_ROLE_NONE_AP
};

#define WDS_EXT_RECEIVED_MBS_IE		0
#define WDS_EXT_RECEIVED_RBS_IE		1
#define WDS_EXT_LINK_STATUS_UPDATE	2
#define WDS_EXT_RBS_OUT_OF_BRR		3
#define WDS_EXT_RBS_SET_CHANNEL		4
#define WDS_EXT_CLEANUP_WDS_LINK	5
#define WDS_EXT_STA_UPDATE_EXT_INFO	6

#define IEEE80211_MAX_EXT_EVENT_DATA_LEN	512

#define IEEE80211_EXTENDER_MIN_RSSI	0
#define IEEE80211_EXTENDER_MAX_RSSI	70
#define	IEEE80211_EXTENDER_MIN_WGT	0
#define	IEEE80211_EXTENDER_MAX_WGT	10
#define	IEEE80211_EXTENDER_MIN_VERBOSE	0
#define	IEEE80211_EXTENDER_MAX_VERBOSE	2
#define IEEE80211_EXTENDER_MIN_INTERVAL	30
#define IEEE80211_EXTENDER_MAX_INTERVAL	300
#define IEEE80211_EXTENDER_DISABLED	0
#define IEEE80211_EXTENDER_ENABLED	1
#define IEEE80211_EXTENDER_MIN_MARGIN	0
#define IEEE80211_EXTENDER_MAX_MARGIN	12
#define IEEE80211_EXTENDER_MIN_SHORT_RETRY_LIMIT	0	/* Disable */
#define IEEE80211_EXTENDER_MAX_SHORT_RETRY_LIMIT	14	/* FIXME: Use QTN_TABLE0_RETRY_CNT */
#define IEEE80211_EXTENDER_MIN_LONG_RETRY_LIMIT		IEEE80211_EXTENDER_MIN_SHORT_RETRY_LIMIT
#define IEEE80211_EXTENDER_MAX_LONG_RETRY_LIMIT		IEEE80211_EXTENDER_MAX_SHORT_RETRY_LIMIT
#define IEEE80211_EXTENDER_MIN_EXPIRY	3
#define IEEE80211_EXTENDER_MAX_EXPIRY	200
#define IEEE80211_EXTENDER_MIN_SCAN_MODE	0
#define IEEE80211_EXTENDER_MAX_SCAN_MODE	1

/**
 * Structure contains data of Association reject event.
 * @name will always be "ASSOCREJECT"
 * @reason_code association reject reason code.
 * @bssid bssid
 */
struct qtn_assoc_reject_event_data {
	char name[12];
	uint16_t reason_code;
	uint8_t bssid[IEEE80211_ADDR_LEN];
} __packed;

/**
 * Structure contains data of deauth event.
 * @name will always be "DEAUTH"
 * @reason_code deauthentication reason code.
 * @bssid bssid
 */
struct qtn_deauth_event_data {
	char name[12];
	uint16_t reason_code;
	uint8_t addr[IEEE80211_ADDR_LEN];
} __packed;

/**
 * Structure contains data of wds extender event.
 * @name will always be "QTN-WDS-EXT"
 * @cmd message type.
 * @mac specify wds peer mac address
 * @link_status specify the wds link state.
 * @ie_len when the message contains an wds extender IE, ie_len is larger than 0.
 */
struct qtn_wds_ext_event_data {
	char name[12];
	uint8_t cmd;
	uint8_t mac[IEEE80211_ADDR_LEN];
	uint8_t extender_role;
	uint8_t link_status;
	uint8_t channel;
	uint8_t bandwidth;
	uint8_t ssid[IEEE80211_NWID_LEN + 1];
	uint8_t ie_len;
	uint8_t wds_extender_ie[0];
}__packed;

/**
 * Structure contains data of Remain on channel event.
 * @name will always be "REMAINONCHAN"
 * @frequency RF frequency to remain on.
 * @duration duration in milliseconds for remain on
 * @cancel_flag: flag to indicate cancel remain on channel
 */
struct qtn_remain_on_chan_event_data {
	char name[16];
	unsigned int frequency;
	unsigned int duration;
	unsigned int cancel_flag;
} __packed;

#define QTN_SEND_ACTION_SUCCESS	0x00
#define QTN_SEND_ACTION_NO_ACK	0x01
#define QTN_SEND_ACTION_FAILED	0x02

/**
 * Structure contains data of TX status event.
 * @name will always be "TXSTATUS"
 * @tx_status success/no-ack
 * @payload_len length of action frame payload
 * @hdr	ieee80211 frame header
 * @payload payload buffer contents
 */
struct qtn_tx_status_event_data {
	char name[9];
	uint8_t tx_status;
	int payload_len;
	struct ieee80211_frame hdr;
	uint8_t payload[0];
} __packed;

struct qtn_exp_cca_stats {
	/* Percentage of air time the channel occupied by activity of own radio and other radios */
	uint32_t	cca_fat;
	/* Percentage of air time which is occupied by other APs and STAs except the local AP/STA and associated STAs/AP */
	uint32_t	cca_intf;
	/* Percentage of air time which is occupied by the local AP/STA and the associated STAs/AP */
	uint32_t	cca_trfc;
	/* Percentage of air time which is occupied by the local AP/STA in trasmission */
	uint32_t	cca_tx;
	/* Percentage of air time which is occupied by the local AP/STA in reception */
	uint32_t	cca_rx;
};

struct ieee80211_scan_freqs {
	uint32_t num;
	uint32_t freqs[0];
} __packed;

struct ieee80211req_cs_args {
#define IEEE80211_CS_THRSHLD_LEVEL		0x01
#define IEEE80211_CS_THRSHLD_INUSE		0x02
	uint32_t arg_type;
	union {
		/* Carrier Sense Threshold level supported by the radio */
		struct {
			int min;
			int max;
		} cs_thrshld_level;
		/* The RSSI signal level at which CS/CCA detects a busy condition.
		* This attribute enables APs to increase minimum sensitivity
		* to avoid detecting busy condition from multiple/weak Wi-Fi sources in dense Wi-Fi
		*/
		int cs_thrshld_inuse;
	} args;
};

#define IEEE80211_DCS_CHAN_LIST_NUM		32
#define IEEE80211_DCS_CMD_START			1
#define IEEE80211_DCS_CMD_STOP			0
#define IEEE80211_DCS_CMD_GET_PARAMS		2
#define IEEE80211_DCS_INTERVAL_MIN		0
#define IEEE80211_DCS_INTERVAL_MAX		60
#define IEEE80211_DCS_DURATION_MIN		50
#define IEEE80211_DCS_DURATION_MAX		20000
#define IEEE80211_DCS_DWELL_TIME_MIN		10
#define IEEE80211_DCS_DWELL_TIME_MAX		50
#define IEEE80211_DCS_SPACING_MIN		100
#define IEEE80211_DCS_SPACING_MAX		500

struct ieee80211_dcs_params {
	uint8_t		dcs_cmd;
	uint16_t	scan_interval;	/* in seconds */
	uint16_t	scan_duration;
	uint16_t	dwell_time;
	uint16_t	spacing;
	uint16_t	chan_list[IEEE80211_DCS_CHAN_LIST_NUM];
};

#define IEEE80211_DCS_RECORD_AP_MAX_NUM		64

struct ieee80211_dcs_ap_info {
	char	ap_bssid[IEEE80211_ADDR_LEN];
	int32_t	ap_bw;
	int32_t	ap_rssi;
};

struct ieee80211_dcs_scan_record_per_ch {
	int32_t				version;
	int32_t				channel_number;
	int32_t				channel_noise;
	int32_t				channel_radar_noise;
	int32_t				channel_non_80211_noise;
	int32_t				channel_utilization;
	int32_t				channel_txpower;
	struct ieee80211_dcs_ap_info	ap_list[IEEE80211_DCS_RECORD_AP_MAX_NUM];
	int32_t				channel_ap_count;
};

struct ieee80211_wps_app_ext {
	uint8_t oui[3];
	uint16_t len;
	uint8_t payload[0];
} __packed;

struct ieee80211_mfr_cmd {
	uint8_t subtype;
#define IEEE80211_MFR_FLAG_BYPASS	0x01
#define IEEE80211_MFR_FLAG_SKB_COPY	0x02
#define IEEE80211_MFR_FLAG_RECV		0x04
#define IEEE80211_MFR_FLAG_XMIT		0x08
#define IEEE80211_MFR_FLAG_ADD		0x40
#define IEEE80211_MFR_FLAG_DEL_ALL	0x80
	uint8_t flags;
	uint8_t resv1;
	uint8_t match_len;
	uint8_t match[0]; /* format: action_category + action_code */
} __packed;

struct ieee80211_remain_chan_info {
	unsigned int frequency;
	unsigned int duration;
} __packed;

#if QTN_FORCE_CRASH
#define IEEE80211_FORCE_CRASH_CPU_MASK	0xC0DE0000

enum ieee80211_force_crash_e {
	IEEE80211_FORCE_CRASH_LHOST = 1,
	IEEE80211_FORCE_CRASH_MUC,
	IEEE80211_FORCE_CRASH_AUC0,
	IEEE80211_FORCE_CRASH_AUC1,
	IEEE80211_FORCE_CRASH_AUC2,
	IEEE80211_FORCE_CRASH_AUC3,
	IEEE80211_FORCE_CRASH_AUC4,
	IEEE80211_FORCE_CRASH_AUC5,
	IEEE80211_FORCE_CRASH_DSP0,
	IEEE80211_FORCE_CRASH_DSP1,
	IEEE80211_FORCE_CRASH_DSP2,
	IEEE80211_FORCE_CRASH_DSP0_ISR,
};
#endif

/*
 * The required phy mode capability of peer STA allowed for association,
 * if a peer STA doesn't support the required phy mode capability, it will
 * not be allowed for association.
 */
#define IEEE80211_PHYMODE_REQUIRED_STR	{"none", "11n", "11ac", "11ax"}
enum ieee80211_phymode_required {
	IEEE80211_PHYMODE_REQUIRE_NONE	= 0,
	IEEE80211_PHYMODE_REQUIRE_11N	= 1,
	IEEE80211_PHYMODE_REQUIRE_11AC	= 2,
	IEEE80211_PHYMODE_REQUIRE_11AX	= 3,
};

/* struct for sending mgmt frame from APP */
struct app_mgmt_frame_buf {
	uint8_t	subtype;
	uint8_t	dst_mac_addr[IEEE80211_ADDR_LEN];
	uint8_t flags;
	struct action_frame_payload frm_payload;
} __packed;

struct ieee80211_mfr_tx_mgmt_frame_buf {
	uint8_t		tx_chan;	/* Tx frame channel */
	uint8_t		tx_bw;		/* Tx frame bandwidth */
	uint32_t	dwell_time_ms;	/* Tx chan dwell time (ms) */
#define	IEEE80211_MFR_NORMAL_ACT 0x01	/* Support SIOCDEV_SUBIO_SEND_ACTION_FRAME for QRPE send */
#define	IEEE80211_MFR_FOR_DPP_AUTH 0x02	/* need to use CTS-to-self to silence client */
#define	IEEE80211_MFR_IS_ACTION 0x04	/* IS an action frame, only support action for now */
	uint32_t	tx_flags;
	union {
		struct app_action_frame_buf action_frm;
		struct app_mgmt_frame_buf mgmt_frm;
	} u_frm;
};

#endif /* __linux__ */

#pragma pack()

#endif /* _NET80211_IEEE80211_IOCTL_H_ */
