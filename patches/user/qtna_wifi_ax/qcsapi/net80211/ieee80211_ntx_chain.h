/*
 * Copyright (c) 2019 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * Common NTX (Number of TX) chain mode definitions
 */
#ifndef _IEEE80211_NTX_CHAIN_H
#define _IEEE80211_NTX_CHAIN_H

/* NTX chain bitmaps */
#define QTN_NTX_CHAIN_MASK_4_TX		0xF
#define QTN_NTX_CHAIN_MASK_2_TX		0x3
#define QTN_NTX_CHAIN_MASK_1_TX		0x1
#define QTN_NTX_CHAIN_MASK_ALL_TX	0x0

#define QTN_NTX_PARAM_CHAINS		0xFF		/* No of Tx chains */
#define QTN_NTX_PARAM_CHAINS_S		0
#define QTN_NTX_PARAM_TX_PWR_BACKOFF	(0xFF << 8)	/* Tx power back-off to be applied */
#define QTN_NTX_PARAM_TX_PWR_BACKOFF_S	8

#define IS_NTX_ACTIVE(_x)		(MS((_x), QTN_NTX_PARAM_CHAINS) != 0)

#endif /* #ifndef _IEEE80211_NTX_CHAIN_H */
