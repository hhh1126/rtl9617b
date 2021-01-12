/**
  Copyright (c) 2017 Quantenna Communications Inc
  All Rights Reserved

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

/*
 * Node Information Sets
 */
#ifndef _QTN_NIS_H_
#define _QTN_NIS_H_

#define QTN_NIS_SET_NOT_ENABLED(_nis, _set_id, _name, _val)
#define QTN_NIS_SET(_nis, _set_id, _name, _val)	\
	do {	\
		KASSERT(_set_id == _nis->set_id, ("must use initial set"));	\
		_nis->val[QTN_NIS_S ## _set_id ## _ ## _name] = (_val);		\
		_nis->bitmap |= (1ULL << QTN_NIS_S ## _set_id ## _ ## _name);	\
	} while (0)

#define QTN_NIS_ALL_SET(_nis, _set_id, _idx, _name, _val)	\
	do {	\
		KASSERT(_set_id == _nis->set_id, ("must use initial set"));			\
		_nis->node[_idx].val[QTN_NIS_ALL_S ## _set_id ## _ ## _name] = (_val);		\
		_nis->node[_idx].bitmap |= (1ULL << QTN_NIS_ALL_S ## _set_id ## _ ## _name);	\
	} while (0)

#define QTN_NIS_IS_SET(_nis, _i)	(!!(_nis->bitmap & (1ULL << _i)))

enum qtn_nis_val_type_s {
	QTN_NIS_VAL_UNSIGNED = 0,
	QTN_NIS_VAL_SIGNED,
	QTN_NIS_VAL_RSN_CAPS,
	QTN_NIS_VAL_RSN_UCASTCIPHER,
	QTN_NIS_VAL_RSN_MCASTCIPHER,
	QTN_NIS_VAL_RSN_KEYMGMT,
};

struct nis_meta_data {
	enum qtn_nis_val_type_s type;
	char *label;
};

/**@addtogroup PerNodeAPIs
 *@{*/

/** Max number of nodes in Node List */
#define QTN_NIS_NODE_LIST_SIZE	1024

/** Number of elements in an Node Information Set */
#define QTN_NIS_VAL_MAX		64

/** Number of defined Node Information Sets */
#define QTN_NIS_SET_ID_MAX	4

/**
 * Identifying information for a node.
 */
struct qtn_nis_node_list_entry {
	/**
	 * Node index
	 */
	uint16_t	idx;

	/**
	 * MAC address
	 */
	uint8_t		mac_addr[MAC_ADDR_LEN];
};

/**
 * Identifying information for all nodes in a VAP.
 */
struct qtn_nis_node_list {
	/**
	 * Total number of entries in the list.
	 */
	uint32_t cnt;

	/**
	 * Node information
	 */
	struct qtn_nis_node_list_entry node[QTN_NIS_NODE_LIST_SIZE];
};

/**
 * Generic structure to hold an array of integer values for a node.
 */
struct qtn_nis_set {
	/**
	 * Node information set ID
	 */
	uint16_t	set_id;

	/**
	 * MAC address
	 */
	uint8_t		mac_addr[MAC_ADDR_LEN];

	/**
	 * Node index
	 */
	uint16_t	node_index;

	/**
	 * Miscellaneous flags
	 */
	uint32_t	flags;

	/**
	 * Bitmap of fields that have been set
	 */
	uint64_t	bitmap;

	/**
	 * Returned data
	 */
	uint64_t	val[QTN_NIS_VAL_MAX];
};

/*
 * NOTE: For backwards compatibility, the contents of these sets should not be changed.
 * Keep sets in sync with qtn_nis_label.
 * New fields *MUST* only be added to the end.
 */

/**
 * Node information set 0
 */
enum qtn_nis_s0_e {
	QTN_NIS_S0_assoc_id,
	QTN_NIS_S0_bw,
	QTN_NIS_S0_tx_bytes,
	QTN_NIS_S0_tx_packets,
	QTN_NIS_S0_tx_amsdu_msdus,
	QTN_NIS_S0_tx_mpdus,
	QTN_NIS_S0_tx_ppdus,
	QTN_NIS_S0_tx_dropped,
	QTN_NIS_S0_tx_wifi_drop1,
	QTN_NIS_S0_tx_wifi_drop2,
	QTN_NIS_S0_tx_wifi_drop3,
	QTN_NIS_S0_tx_wifi_drop4,
	QTN_NIS_S0_tx_errors,
	QTN_NIS_S0_tx_ucast,
	QTN_NIS_S0_tx_mcast,
	QTN_NIS_S0_tx_bcast,
	QTN_NIS_S0_tx_max_phy_rate,
	QTN_NIS_S0_tx_max_nss,
	QTN_NIS_S0_tx_max_mcs,
	QTN_NIS_S0_tx_last_phy_rate,
	QTN_NIS_S0_tx_last_nss,
	QTN_NIS_S0_tx_last_mcs,
	QTN_NIS_S0_tx_flags,
	QTN_NIS_S0_tx_retries,
	QTN_NIS_S0_rx_bytes,
	QTN_NIS_S0_rx_packets,
	QTN_NIS_S0_rx_amsdu_msdus,
	QTN_NIS_S0_rx_mpdus,
	QTN_NIS_S0_rx_ppdus,
	QTN_NIS_S0_rx_dropped,
	QTN_NIS_S0_rx_errors,
	QTN_NIS_S0_rx_ucast,
	QTN_NIS_S0_rx_mcast,
	QTN_NIS_S0_rx_bcast,
	QTN_NIS_S0_rx_unknown,
	QTN_NIS_S0_rx_max_phy_rate,
	QTN_NIS_S0_rx_max_nss,
	QTN_NIS_S0_rx_max_mcs,
	QTN_NIS_S0_rx_last_phy_rate,
	QTN_NIS_S0_rx_last_nss,
	QTN_NIS_S0_rx_last_mcs,
	QTN_NIS_S0_rx_smthd_rssi,
	QTN_NIS_S0_rx_flags,
	QTN_NIS_S0_rx_retries,
	QTN_NIS_S0_tx_bw,
	QTN_NIS_S0_rx_bw,
	QTN_NIS_S0_rx_last_rssi,
	QTN_NIS_S0_rx_last_rssi_tot,
	QTN_NIS_S0_rx_smthd_rssi_tot,
	QTN_NIS_S0_tx_median_phyrate,
	QTN_NIS_S0_rx_median_phyrate,
	QTN_NIS_S0_tx_pppc,
	/* Only add new fields directly above this line. */
	QTN_NIS_S0_MAX
};

/**
 * Node information set 1
 *
 * Per-TID counters
 */
enum qtn_nis_s1_e {
	QTN_NIS_S1_tx_tid0_bytes,
	QTN_NIS_S1_tx_tid1_bytes,
	QTN_NIS_S1_tx_tid2_bytes,
	QTN_NIS_S1_tx_tid3_bytes,
	QTN_NIS_S1_tx_tid4_bytes,
	QTN_NIS_S1_tx_tid5_bytes,
	QTN_NIS_S1_tx_tid6_bytes,
	QTN_NIS_S1_tx_tid7_bytes,
	QTN_NIS_S1_rx_tid0_bytes,
	QTN_NIS_S1_rx_tid1_bytes,
	QTN_NIS_S1_rx_tid2_bytes,
	QTN_NIS_S1_rx_tid3_bytes,
	QTN_NIS_S1_rx_tid4_bytes,
	QTN_NIS_S1_rx_tid5_bytes,
	QTN_NIS_S1_rx_tid6_bytes,
	QTN_NIS_S1_rx_tid7_bytes,
	/* Only add new fields directly above this line. */
	QTN_NIS_S1_MAX
};

/**
 * Node information set 2
 *
 * BF, SU, MU and OFDMA stats
 */
enum qtn_nis_s2_e {
	QTN_NIS_S2_mu_beamformer,
	QTN_NIS_S2_mu_beamformee,
	QTN_NIS_S2_su_beamformer,
	QTN_NIS_S2_su_beamformee,
	QTN_NIS_S2_tx_11n_bytes,
	QTN_NIS_S2_tx_11ac_su_bytes,
	QTN_NIS_S2_tx_11ac_mu_bytes,
	QTN_NIS_S2_tx_11ax_su_bytes,
	QTN_NIS_S2_tx_11ax_mu_bytes,
	QTN_NIS_S2_tx_11ax_ofdma_bytes,
	QTN_NIS_S2_tx_su_nss,
	QTN_NIS_S2_tx_su_is_orthsnd,
	QTN_NIS_S2_tx_mu_nss,
	QTN_NIS_S2_tx_mu_is_orthsnd,
	QTN_NIS_S2_tx_mu_aid,
	/* Only add new fields directly above this line. */
	QTN_NIS_S2_MAX
};

/**
 * Node information set 3
 *
 * Aggregation counters
 */
enum qtn_nis_s3_e {
	QTN_NIS_S3_tx_amsdu_subfrms_1,
	QTN_NIS_S3_tx_amsdu_subfrms_2_4,
	QTN_NIS_S3_tx_amsdu_subfrms_5_8,
	QTN_NIS_S3_tx_amsdu_subfrms_9_16,
	QTN_NIS_S3_tx_amsdu_subfrms_17_32,
	QTN_NIS_S3_tx_amsdu_subfrms_gt_32,
	QTN_NIS_S3_tx_amsdu_lt_2k,
	QTN_NIS_S3_tx_amsdu_2k_4k,
	QTN_NIS_S3_tx_amsdu_4k_8k,
	QTN_NIS_S3_tx_amsdu_gt_8k,
	QTN_NIS_S3_rx_amsdu_subfrms_1,
	QTN_NIS_S3_rx_amsdu_subfrms_2_4,
	QTN_NIS_S3_rx_amsdu_subfrms_5_8,
	QTN_NIS_S3_rx_amsdu_subfrms_9_16,
	QTN_NIS_S3_rx_amsdu_subfrms_17_32,
	QTN_NIS_S3_rx_amsdu_subfrms_gt_32,
	QTN_NIS_S3_rx_amsdu_lt_2k,
	QTN_NIS_S3_rx_amsdu_2k_4k,
	QTN_NIS_S3_rx_amsdu_4k_8k,
	QTN_NIS_S3_rx_amsdu_gt_8k,
	/* Only add new fields directly above this line. */
	QTN_NIS_S3_MAX
};

/**
 * RSN is not enabled for this station.
 */
#define QTN_NIS_S4_RSN_DISABLED		BIT(31)

/** Number of node entries in an All-node Information Set */
#define QTN_NIS_ALL_ENTRY_MAX	64

/** Number of fields in an All-node Information Set */
#define QTN_NIS_ALL_FIELD_MAX	4

/** Number of defined All-node Information Sets */
#define QTN_NIS_ALL_SET_ID_MAX	1

/**
 * Generic structure to hold up to four integer values for a node.
 */
struct qtn_nis_all_node {
	/**
	 * MAC address
	 */
	uint8_t		mac_addr[MAC_ADDR_LEN];

	/**
	 * Node index
	 */
	uint16_t	node_index;

	/**
	 * Bitmap of fields that have been set
	 */
	uint16_t	bitmap;

	/**
	 * Unused
	 */
	uint16_t	flags;

	/**
	 * Returned data
	 */
	uint32_t	val[QTN_NIS_ALL_FIELD_MAX];
};

/**
 * All-node Information Set entry.
 */
struct qtn_nis_all_set {
	/**
	 * Node information set ID
	 */
	uint16_t	set_id;

	/**
	 * Node index to start retrieving from
	 */
	uint16_t	first_node_index;

	/**
	 * Miscellaneous flags
	 */
	uint32_t	flags;

	/**
	 * Number of nodes in report
	 */
	uint16_t	node_cnt;

	/**
	 * Node information
	 */
	struct qtn_nis_all_node node[QTN_NIS_ALL_ENTRY_MAX];
};

/*
 * NOTE: For backwards compatibility, the contents of these sets should not be changed.
 * Keep sets in sync with qtn_nis_all_label.
 * New fields *MUST* only be added to the end.
 */

/**
 * All-node information set 0
 *
 * RSN Capabilities
 */
enum qtn_nis_all_s0_e {
	/**
	 * RSN capability bitmap
	 * Bit 6 - MFP optional
	 * Bit 7 - MFP required
	 * Bit 31 - RSN disabled
	 */
	QTN_NIS_ALL_S0_rsn_caps,
	/**
	 * RSN Unicast cipher
	 */
	QTN_NIS_ALL_S0_rsn_ucastcipher,
	/**
	 * RSN Multicast cipher
	 */
	QTN_NIS_ALL_S0_rsn_mcastcipher,
	/**
	 * RSN key management
	 */
	QTN_NIS_ALL_S0_rsn_keymgmt,

	/* Only add new fields here. */
	QTN_NIS_ALL_S0_MAX
};

/**@}*/

#endif /* _QTN_NIS_H_ */
