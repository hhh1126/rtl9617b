/*
 * qtn_power_table.h
 *
 * The enums, data structures and function prototypes to parse the power table
 *
 * Copyright (c) 2020 Quantenna Communications, Inc.
 */

#ifndef _QTN_POWER_TABLE_H_
#define _QTN_POWER_TABLE_H_

#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
#include <linux/types.h>
#else
#include <types.h>
#endif

#define QTN_PWR_TBL_TX_CHAINS_1		1
#define QTN_PWR_TBL_TX_CHAINS_2		2
#define QTN_PWR_TBL_TX_CHAINS_3		3
#define QTN_PWR_TBL_TX_CHAINS_4		4
#define QTN_PWR_TBL_TX_CHAINS_5		5
#define QTN_PWR_TBL_TX_CHAINS_6		6
#define QTN_PWR_TBL_TX_CHAINS_7		7
#define QTN_PWR_TBL_TX_CHAINS_8		8
#define QTN_PWR_TBL_TX_CHAINS_MAX_2G	QTN_PWR_TBL_TX_CHAINS_4
#define QTN_PWR_TBL_TX_CHAINS_MAX_5G_6G	QTN_PWR_TBL_TX_CHAINS_8

#define QTN_PWR_VALUE_INVALID		(-1)

/* Index for the number of Tx chains */
enum ieee80211_pwr_idx_tx_chains {
	PWR_IDX_TX_CHAINS_8 = 0,
	PWR_IDX_TX_CHAINS_7 = 1,
	PWR_IDX_TX_CHAINS_6 = 2,
	PWR_IDX_TX_CHAINS_5 = 3,
	PWR_IDX_TX_CHAINS_4 = 4,
	PWR_IDX_TX_CHAINS_3 = 5,
	PWR_IDX_TX_CHAINS_2 = 6,
	PWR_IDX_TX_CHAINS_1 = 7,
	PWR_IDX_TX_CHAINS_MAX = 8
};

/*
 * enum ieee80211_pwr_idx_fem_prichan - union of per-FEM and per-Primary channel
 * position power indexes.
 *
 * Per-FEM power values are used for 5GHz band channels.
 * Power values for different primary channel position are used for 2.4GHz channels.
 */
enum ieee80211_pwr_idx_fem_prichan {
	PWR_IDX_FEM0_PRI_LOW = 0,
	PWR_IDX_FEM1_PRI_UPPER = 1,
	PWR_IDX_FEM_PRICHAN_MAX = 2
};

/* Index for beamforming support */
enum ieee80211_pwr_idx_bf {
	PWR_IDX_BF_OFF = 0,
	PWR_IDX_BF_ON = 1,
	PWR_IDX_BF_MAX = 2
};

/* Index for number of spatial streams */
enum ieee80211_pwr_idx_nss {
	PWR_IDX_1SS = 0,
	PWR_IDX_2SS = 1,
	PWR_IDX_3SS = 2,
	PWR_IDX_4SS = 3,
	PWR_IDX_SS_NUM_2G = 4,
	PWR_IDX_5SS = 4,
	PWR_IDX_6SS = 5,
	PWR_IDX_7SS = 6,
	PWR_IDX_8SS = 7,
	PWR_IDX_SS_MAX = 8
};

/* Index for supported bandwidths */
enum ieee80211_pwr_idx_bw {
	PWR_IDX_20M = 0,
	PWR_IDX_40M = 1,
	PWR_IDX_80M = 2,
	PWR_IDX_160M = 3,
	PWR_IDX_BW_MAX = 4
};

/* Represent integer or decimal part of power value */
enum ieee80211_pwr_value_type {
	PWR_VALUE_TYPE_INT = 0,
	PWR_VALUE_TYPE_DECIMAL = 1,
	PWR_VALUE_TYPE_MAX = 2
};

/* Index for the power table types. Need to see if this could be removed later */
enum ieee80211_pwr_table_type {
	PWR_TABLE_TYPE_ACTIVE = 0,
	PWR_TABLE_TYPE_DNTX = 1,
	PWR_TABLE_TYPE_MAX = 2
};

#define QTN_PWR_TBL_PWR_VALUE_VALID(_val)	(((_val) == QTN_PWR_VALUE_INVALID) ? 0 : 1)
#define QTN_PWR_TBL_CHECK_LIMITS(_val, _high)	(((_val) < (_high)) ? 1 : 0)

#define QTN_PWR_TBL_IDX_TX_CHAINS_VALID(_idx) \
			QTN_PWR_TBL_CHECK_LIMITS((_idx), PWR_IDX_TX_CHAINS_MAX)
#define QTN_PWR_TBL_IDX_FEM_PRI_VALID(_idx) \
			QTN_PWR_TBL_CHECK_LIMITS((_idx), PWR_IDX_FEM_PRICHAN_MAX)
#define QTN_PWR_TBL_IDX_BF_VALID(_idx) \
			QTN_PWR_TBL_CHECK_LIMITS((_idx), PWR_IDX_BF_MAX)
#define QTN_PWR_TBL_IDX_NSS_VALID(_idx) \
			QTN_PWR_TBL_CHECK_LIMITS((_idx), PWR_IDX_SS_MAX)
#define QTN_PWR_TBL_IDX_BW_VALID(_idx) \
			QTN_PWR_TBL_CHECK_LIMITS((_idx), PWR_IDX_BW_MAX)

#define QTN_PWR_TBL_TYPE_VALUE_VALID(_type) \
			QTN_PWR_TBL_CHECK_LIMITS((_type), PWR_VALUE_TYPE_MAX)
#define QTN_PWR_TBL_TYPE_TABLE_VALID(_type) \
			QTN_PWR_TBL_CHECK_LIMITS((_type), PWR_TABLE_TYPE_MAX)

struct qtn_shared_ch_power_table {
	/* Integer part of the power value */
	int8_t pwr_int[PWR_IDX_TX_CHAINS_MAX][PWR_IDX_FEM_PRICHAN_MAX][PWR_IDX_BF_MAX]
					[PWR_IDX_SS_MAX][PWR_IDX_BW_MAX];
	/* Decimal part of the power value */
	int8_t pwr_decimal[PWR_IDX_TX_CHAINS_MAX][PWR_IDX_FEM_PRICHAN_MAX][PWR_IDX_BF_MAX]
					[PWR_IDX_SS_MAX][PWR_IDX_BW_MAX];
};

/*
 * IOCTL data for per-channel, per-chain, max Tx power table
 *
 * @chan_ieee IEEE number of the channel this data describes.
 * @maxpwr Array of max Tx power values for every possible operational mode for the channel.
 */
struct ieee80211_chan_power_table {
	uint8_t chan_ieee;
	uint8_t idx_chain;	/* Chain index. Valid range 0-7 */
	int8_t maxpwr[PWR_IDX_FEM_PRICHAN_MAX][PWR_IDX_BF_MAX][PWR_IDX_SS_MAX][PWR_IDX_BW_MAX];
	int8_t maxpwr_decimal[PWR_IDX_FEM_PRICHAN_MAX][PWR_IDX_BF_MAX][PWR_IDX_SS_MAX]
							[PWR_IDX_BW_MAX];
};

enum ieee80211_pwr_idx_bw
qtn_pwr_tbl_get_idx_bw(int bw);
enum ieee80211_pwr_idx_tx_chains
qtn_pwr_tbl_get_idx_chain(uint8_t chains);
int8_t
qtn_pwr_tbl_get_pwr(struct qtn_shared_ch_power_table *sh_ptable,
	enum ieee80211_pwr_idx_tx_chains chains, enum ieee80211_pwr_idx_fem_prichan fem_pri,
	enum ieee80211_pwr_idx_bf bf, enum ieee80211_pwr_idx_nss nss, enum ieee80211_pwr_idx_bw bw,
	enum ieee80211_pwr_value_type pwr_value_type);
#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
int
qtn_pwr_tbl_idxes_valid_for_chan(uint16_t ic_ieee, uint32_t ic_flags,
	enum ieee80211_pwr_idx_fem_prichan pri, enum ieee80211_pwr_idx_nss ss,
	enum ieee80211_pwr_idx_bw bw);
int8_t
qtn_pwr_tbl_get_pwr_fem_max(uint16_t ic_ieee, uint32_t ic_flags,
	struct qtn_shared_ch_power_table *sh_ptable, enum ieee80211_pwr_idx_bw bw);
int
qtn_pwr_tbl_get_chan_pwr_per_chain(struct qtn_shared_ch_power_table *sh_ptable,
	struct ieee80211_chan_power_table *ptable, uint8_t idx_chain);
int8_t
qtn_pwr_tbl_set_pwr(struct qtn_shared_ch_power_table *sh_ptable,
	enum ieee80211_pwr_idx_tx_chains chains, enum ieee80211_pwr_idx_fem_prichan fem_pri,
	enum ieee80211_pwr_idx_bf bf, enum ieee80211_pwr_idx_nss nss, enum ieee80211_pwr_idx_bw bw,
	enum ieee80211_pwr_value_type pwr_value_type, int8_t pwr_value);
int
qtn_pwr_tbl_set_chan_pwr_per_chain(uint16_t ic_ieee, uint32_t ic_flags,
	struct qtn_shared_ch_power_table *sh_ptable,
	struct ieee80211_chan_power_table *ptable, uint8_t idx_chain, int8_t pwr_value);
int
qtn_pwr_tbl_set_chan_pwr_all_fixed(uint16_t ic_ieee, uint32_t ic_flags,
	struct qtn_shared_ch_power_table *sh_ptable, int8_t pwr_value);
void
qtn_pwr_tbl_init(struct qtn_shared_ch_power_table *sh_ptable);
#endif /* #if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD) */
#endif /*_QTN_POWER_TABLE_H_ */
