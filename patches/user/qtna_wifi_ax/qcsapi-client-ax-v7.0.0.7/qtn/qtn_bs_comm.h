/*
 * Copyright (c) 2012 Quantenna Communications, Inc.
 */

/*
 * Define the common data struct and macro for band steering feature
 */

#ifndef _QTN_BS_COMM_H
#define _QTN_BS_COMM_H

#include <net/if.h>

#define QEVT_SYSTEM_EVENT_SOCKET_PATH	"/tmp/qevt_system_event_socket"
#define QCSAPI_BSA_SOCKET_PATH		"/tmp/qcsapi_bsa_socket"
#define BSA_QCSAPI_SOCKET_PATH		"/tmp/bsa_qcsapi_socket"
#ifndef ETH_ALEN
#define ETH_ALEN			6
#endif

enum QCSAPI_BS_CMD {
	QCSAPI_START_DUMP_STA_TBL	= 0x0001,
	QCSAPI_GET_STA_INFO		= 0x0002,
	QCSAPI_STOP_DUMP_STA_TBL	= 0x0003,
};

enum QCSAPI_BS_STATUS {
	QCSAPI_OK			= 0x0000,
	QCSAPI_ESESSION_INVAL		= 0x0001,
	QCSAPI_ENOMEM			= 0x0002,
};

struct qcsapi_bs_req_hdr {
	u_int16_t session_id;
	u_int16_t cmd_id;
};

struct qcsapi_bs_resp_hdr {
	u_int16_t status;
	u_int16_t session_id;
};

struct qcsapi_bs_sta_tbl_req {
	u_int16_t session_id;
	u_int16_t cmd_id;
	u_int16_t req_cnt;
	u_int16_t flag;
	u_int32_t len; /* payload length */
	char payload[0];
}__attribute__((packed));

struct qcsapi_bs_sta_tbl_resp {
	u_int16_t status;
	u_int16_t session_id;
	u_int16_t total_sta_cnt;
	u_int16_t remain_sta_cnt;
	u_int16_t sta_cnt;
	u_int32_t len;	/* payload length */
	char payload[0];
}__attribute__((packed));

struct bsa_sta_bspecific_info {
	int32_t last_rssi;
	u_int16_t sta_capab;
	u_int16_t avg_tx_phy_rate;
	u_int16_t avg_rx_phy_rate;
	u_int16_t rx_ss_info;
	u_int32_t supported_phy_rate;
	u_int64_t ts_probe;
	u_int64_t ts_last_phy_rate;
	u_int32_t data_pkt_count;
	u_int32_t avg_airtime;
	u_int8_t blacklist;
	u_int8_t btm_11v_support;
	u_int8_t phy_mode_supported; /* VHT support ? */
	u_int8_t mumimo_support;
	u_int8_t bw_support; /* 20/40/80/160 */
	u_int8_t btm_resp_pend; /* set after sending BTM req frame and waiting for resp */
}__attribute__((packed));

struct bsa_sta_entry {
	u_int8_t sta_mac[ETH_ALEN];
	u_int8_t cfg_not_steerable;
	u_int8_t steering_mode; /* assoc, run, BTM */
	u_int8_t steer_reason; /* reason of determining last steering decision */
	u_int8_t not_steerable;
	u_int8_t steer_counter; /* number of times STA is steered successfully */
	u_int8_t failed_steer_attempt; /* number of failed steering attempts */
	u_int8_t prev_bssid[ETH_ALEN];
	u_int8_t bssid[ETH_ALEN]; /* Currently connected BSSID */
	u_int64_t ts_last_seen;
	u_int64_t ts_steering_decision;
	u_int64_t ts_deauthentication; /* time stamp for deauth command sending
 */
	u_int64_t ts_disconnection; /* time stamp for the event disconnection */
	u_int64_t conn_ts; /* connection timestamp(sec) */
	u_int64_t ts_not_steerable; /* time stamp for becoming not steerable */
	struct bsa_sta_bspecific_info info_2g;
	struct bsa_sta_bspecific_info info_5g;
	u_int8_t prev_band;
	u_int8_t curr_band;
	u_int8_t target_band;
	u_int8_t steer_pend;
	u_int8_t dual_band;
	u_int8_t failed_steer_total;
	char ifname[IFNAMSIZ]; /* Interface name to which STA is associated */
	u_int32_t steer_phyrate; /* Phy rate that determined last steering decision */
	u_int64_t ts_last_2gto5g_steering;
	u_int64_t ts_last_5gto2g_steering;
	u_int8_t btm_retry_num;
	u_int8_t added_bsa_blacklist;
	u_int8_t send_deauth;
}__attribute__((packed));

#endif // #ifndef _QTN_BS_COMM_H
