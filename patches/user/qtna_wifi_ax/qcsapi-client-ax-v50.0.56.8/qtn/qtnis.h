/**
 * Copyright (c) 2018-2019 Quantenna Communications Inc
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 **/

/*
 * Quantenna Information Sets
 */
#ifndef _QTNIS_H_
#define _QTNIS_H_

#ifndef IFNAMSIZ
#define IFNAMSIZ	16
#endif /* IFNAMSIZ */

#define QTNIS_SET_NOT_ENABLED(_qtnis, _set_id, _name, _val)
#ifdef __KERNEL__
#define QTNIS_SET(_qtnis, _set_id, _name, _val)	\
	do {	\
		KASSERT(_set_id == _qtnis->set_id, ("must use initial set"));	\
		_qtnis->val[QTNIS_S ## _set_id ## _ ## _name] = (_val);		\
		_qtnis->bitmap |= (1ULL << QTNIS_S ## _set_id ## _ ## _name);	\
	} while (0)
#else
#define QTNIS_SET(_qtnis, _set_id, _name, _val)	\
	do {	\
		_qtnis->val[QTNIS_S ## _set_id ## _ ## _name] = (_val);		\
		_qtnis->bitmap |= (1ULL << QTNIS_S ## _set_id ## _ ## _name);	\
	} while (0)
#endif
#define QTNIS_IS_SET(_qtnis, _i)	(!!(_qtnis->bitmap & (1ULL << _i)))

/**
 * @addtogroup InfosetAPIs
 * @{
 */

/**
 * Infoset field types.
 */
enum qtnis_val_type_s {
	/**
	 * Unsigned integer
	 */
	QTNIS_VAL_UNSIGNED = 0,

	/**
	 * Signed integer
	 */
	QTNIS_VAL_SIGNED,

	/**
	 * Miscellaneous flags
	 */
	QTNIS_VAL_FLAG,

	/**
	 * MAC address
	 */
	QTNIS_VAL_MACADDR,

	/**
	 * Index
	 */
	QTNIS_VAL_INDEX
};

/**
 * Infoset metadata
 */
struct qtnis_meta_data {
	enum qtnis_val_type_s type;
	char *label;
};

/** Number of elements in an Interface Information Set */
#define QTNIS_IF_VAL_MAX		64

/** Number of defined Interface Information Sets */
#define QTNIS_SET_ID_MAX		4

/**
 * Generic structure to hold an array of integer values for an interface.
 */
struct qtnis_if_set {
	/**
	 * Interface information set ID
	 */
	uint16_t	set_id;

	/**
	 * Interface name
	 */
	int8_t		ifname[IFNAMSIZ];

	/**
	 * Interface MAC address
	 */
	uint8_t		mac_addr[MAC_ADDR_LEN];

	/**
	 * Miscellaneous flags
	 */
	uint64_t	flags;

	/**
	 * Bitmap of fields that have been set
	 */
	uint64_t	bitmap;

	/**
	 * Returned data
	 */
	uint64_t	val[QTNIS_IF_VAL_MAX];
};

/*
 * NOTE: For backwards compatibility, the contents of these sets should not be changed.
 * Keep sets in sync with qcsapi_qtnis_if_label.
 * New fields *MUST* only be added to the end.
 */

/**
 * Interface information set 0
 */

enum qtnis_if_s0_e {
	QTNIS_S0_assoc_id,
	QTNIS_S0_bw,

	QTNIS_S0_tx_bytes,
	QTNIS_S0_tx_packets,
	QTNIS_S0_tx_amsdu_msdus,
	QTNIS_S0_tx_mpdus,
	QTNIS_S0_tx_ppdus,
	QTNIS_S0_tx_wifi_sent_be,
	QTNIS_S0_tx_wifi_sent_bk,
	QTNIS_S0_tx_wifi_sent_vi,
	QTNIS_S0_tx_wifi_sent_vo,
	QTNIS_S0_tx_dropped,
	QTNIS_S0_tx_wifi_drop_be,
	QTNIS_S0_tx_wifi_drop_bk,
	QTNIS_S0_tx_wifi_drop_vi,
	QTNIS_S0_tx_wifi_drop_vo,
	QTNIS_S0_tx_errors,
	QTNIS_S0_tx_ucast,
	QTNIS_S0_tx_mcast,
	QTNIS_S0_tx_bcast,
	QTNIS_S0_tx_max_phy_rate,
	QTNIS_S0_tx_max_nss,
	QTNIS_S0_tx_max_mcs,
	QTNIS_S0_tx_last_phy_rate,
	QTNIS_S0_tx_last_nss,
	QTNIS_S0_tx_last_mcs,
	QTNIS_S0_tx_flags,
	QTNIS_S0_tx_retries,
	QTNIS_S0_tx_bw,

	QTNIS_S0_rx_bytes,
	QTNIS_S0_rx_packets,
	QTNIS_S0_rx_amsdu_msdus,
	QTNIS_S0_rx_mpdus,
	QTNIS_S0_rx_ppdus,
	QTNIS_S0_rx_dropped,
	QTNIS_S0_rx_errors,
	QTNIS_S0_rx_ucast,
	QTNIS_S0_rx_mcast,
	QTNIS_S0_rx_bcast,
	QTNIS_S0_rx_unknown,
	QTNIS_S0_rx_max_phy_rate,
	QTNIS_S0_rx_max_nss,
	QTNIS_S0_rx_max_mcs,
	QTNIS_S0_rx_last_phy_rate,
	QTNIS_S0_rx_last_nss,
	QTNIS_S0_rx_last_mcs,
	QTNIS_S0_rx_smthd_rssi,
	QTNIS_S0_rx_flags,
	QTNIS_S0_rx_retries,
	QTNIS_S0_rx_bw,
	QTNIS_S0_rx_last_rssi,
	QTNIS_S0_rx_last_rssi_tot,
	QTNIS_S0_rx_smthd_rssi_tot,

	/* Only add new fields here. */
	QTNIS_S0_MAX
};

/**
 * Interface information set 1
 */
enum qtnis_11h_11k_basic_e {
	/**
	 * basic request set
	 */
	QTNIS_S1_offset,
	QTNIS_S1_duration,
	QTNIS_S1_channel,

	/**
	 * basic result set
	 */
	QTNIS_S1_basic,

	/* Only add new fields here. */
	QTNIS_S1_MAX
};

/**
 * Interface information set 2
 */
enum qtnis_if_infoset_e {
	/**
	 * The number of expected ACKs that were never received.
	 */
	QTNIS_S2_tx_ack_failures,
	/**
	 * The number of received packets with an invalid mac header.
	 */
	QTNIS_S2_rx_invalid_mac_header,
	/**
	 * The number of received packets from non-associated stations.
	 */
	QTNIS_S2_rx_non_assoc_packets,
	/**
	 * The number of frames for which the parity check of the PLCP header failed.
	 */
	QTNIS_S2_rx_plcp_errors,
	/**
	 * The number of frames for which the Frame Check Sequence (FCS) was in error.
	 */
	QTNIS_S2_rx_fcs_errors,

	/* Only add new fields here. */
	QTNIS_S2_MAX
};

/**
 * Interface information set 3 - radar
 */
enum qtnis_if_infoset_radar {
	/**
	 * Radio detection status - 0 (disabled) or 1 (enabled)
	 */
	QTNIS_S3_radar_status,
	/**
	 * Radio detection mode - 0 (disabled), 1 (single) or 2 (dual mode)
	 */
	QTNIS_S3_radar_mode,
	/**
	 * The bandwidth used for radar block 1
	 */
	QTNIS_S3_radar_bw_1,
	/**
	 * The number of radar detections in radar block 1
	 */
	QTNIS_S3_radar_detections_1,
	/**
	 * The bandwidth used for radar block 2
	 */
	QTNIS_S3_radar_bw_2,
	/**
	 * The number of radar detections in radar block 2
	 */
	QTNIS_S3_radar_detections_2,

	/* Only add new fields here. */
	QTNIS_S3_MAX
};

/**@}*/

#endif /* _QTNIS_H_ */
