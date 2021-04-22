/*
 * Copyright (c) 2019 Quantenna Communications, Inc.
 * All rights reserved.
 */

/*
 * Scan Information Sets
 */
#ifndef _QTN_SIS_H_
#define _QTN_SIS_H_

#define QTN_SIS_SET_NOT_ENABLED(_sis, _set_id, _name, _val)

#define QTN_SIS_SET(_sis, _set_id, _idx, _name, _val)		\
	do {	\
		KASSERT(_set_id == _sis->ctrl.set_id, ("must use initial set"));		\
		_sis->ap[_idx].val[QTN_SIS_ALL_S ## _set_id ## _ ## _name] = (_val);	\
		_sis->ap[_idx].bitmap |= (1ULL << QTN_SIS_ALL_S ## _set_id ## _ ## _name);\
	} while (0)

#define QTN_SIS_IS_SET(_node, _i)	(!!(_node->bitmap & (1ULL << _i)))

enum qtn_sis_val_type_s {
	QTN_SIS_VAL_UNSIGNED = 0,
	QTN_SIS_VAL_SIGNED,
	QTN_SIS_VAL_HT_SEC_OFFSET,
	QTN_SIS_VAL_SGI_CAP,
	QTN_SIS_VAL_DATA_RATE,
	QTN_SIS_VAL_BAND_INFO,
	QTN_SIS_VAL_PRINT_SKIP,
	QTN_SIS_VAL_RSN_UCASTCIPHER,
	QTN_SIS_VAL_RSN_KEYMGMT,
	QTN_SIS_VAL_SECURITY_PROTOCOL,
};

struct sis_meta_data {
	enum qtn_sis_val_type_s type;
	char *label;
};

/**
 * @addtogroup ScanAPIs
 * @{
 */

/** Number of AP entries in a Scan Information Set */
#define QTN_SIS_ALL_ENTRY_MAX		24	/* Must not be changed */

/** Number of entries in a Scan Information Set */
#define QTN_SIS_DATA_ENTRY_MAX		23	/* Must not be changed */

/** Number of defined Scan Information Sets */
#define QTN_SIS_ALL_SET_ID_MAX		1

#ifndef IW_ESSID_MAX_SIZE
#define IW_ESSID_MAX_SIZE		32
#endif

/**
 * Generic structure to hold values for a scan node.
 */
struct qtn_sis_all_ap {
	/**
	 * MAC address
	 */
	uint8_t	bssid[MAC_ADDR_LEN];

	/**
	 * SSID
	 */
	uint8_t	ssid[IW_ESSID_MAX_SIZE];

	/**
	 * Scan result index
	 */
	uint16_t	ap_idx;

	/**
	 * Bitmap of single data fields that have been set
	 */
	uint64_t	bitmap;

	/**
	 * Unused
	 */
	uint16_t	flags;

	/**
	 * Returned single data
	 */
	uint32_t	val[QTN_SIS_DATA_ENTRY_MAX];
};

/**
 * Scan Information Set control entry.
 */
struct qtn_sis_all_ctrl {
	/**
	 * Node information set ID
	 */
	uint16_t	set_id;

	/**
	 * Node index to start retrieving from
	 */
	uint16_t	first_ap_idx;

	/**
	 * Current node index count
	 */
	uint16_t	current_ap_idx;

	/**
	 * Miscellaneous flags
	 */
	uint32_t	flags;

	/**
	 * Number of APs in report
	 */
	uint16_t	ap_cnt;
};

/**
 * All-node Scan Information Set.
 */
struct qtn_sis_all_set {
	struct qtn_sis_all_ctrl ctrl;
	/**
	 * Node information
	 */
	struct qtn_sis_all_ap ap[QTN_SIS_ALL_ENTRY_MAX];
};

#define QTN_SIS_ALL_SET_F_REPORT		0x00000001
#define QTN_SIS_ALL_SET_F_REPORT_S		0
#define QTN_SIS_ALL_SET_F_REPORT_FULL		0x00000001

/*
 * NOTE: For backwards compatibility, the contents of these sets should not be changed.
 * Keep sets in sync with qtn_sis_all_label.
 * New fields *MUST* only be added to the end.
 */

/**
 * All-node scan information set 0
 *
 * AP scan information
 */
enum qtn_sis_all_s0_e {
	QTN_SIS_ALL_S0_ap_flags,
	QTN_SIS_ALL_S0_ap_channel,
	QTN_SIS_ALL_S0_ap_bw,
	QTN_SIS_ALL_S0_ap_rssi,
	QTN_SIS_ALL_S0_ap_protocol,
	QTN_SIS_ALL_S0_ap_encryption_modes,
	QTN_SIS_ALL_S0_ap_authentication_modes,
	QTN_SIS_ALL_S0_ap_best_data_rates,
	QTN_SIS_ALL_S0_ap_wps,
	QTN_SIS_ALL_S0_ap_80211_proto,
	QTN_SIS_ALL_S0_ap_qhop_role,
	QTN_SIS_ALL_S0_ap_noise,
	QTN_SIS_ALL_S0_ap_opmode,
	QTN_SIS_ALL_S0_ap_bintval,
	QTN_SIS_ALL_S0_ap_ht_secoffset,
	QTN_SIS_ALL_S0_ap_chan_center1,
	QTN_SIS_ALL_S0_ap_chan_center2,
	QTN_SIS_ALL_S0_ap_last_beacon,
	QTN_SIS_ALL_S0_ap_dtimeriod,
	QTN_SIS_ALL_S0_ap_11b_present,
	QTN_SIS_ALL_S0_ap_bsscolor,
	QTN_SIS_ALL_S0_ap_heop,
	QTN_SIS_ALL_S0_ap_bandinfo,

	/* Only add new fields here. */
	QTN_SIS_ALL_S0_MAX
};

/** @} */

#endif /* _QTN_NIS_H_ */
