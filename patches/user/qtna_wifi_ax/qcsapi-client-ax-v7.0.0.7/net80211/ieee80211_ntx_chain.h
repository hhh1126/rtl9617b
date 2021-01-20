/*
 * Copyright (c) 2019 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * Common NTX (Number of TX) chain mode definitions
 */
#ifndef _IEEE80211_NTX_CHAIN_H
#define _IEEE80211_NTX_CHAIN_H

/*
 *  NTX functional block flags - if set, that block is (or may be) put in NTX mode
 */

/* NTX chain bitmaps */
#define QTN_NTX_CHAIN_MASK_4_TX 0xF
#define QTN_NTX_CHAIN_MASK_2_TX 0x3
#define QTN_NTX_CHAIN_MASK_1_TX 0x1

/* add new bits as needed here */
#define IEEE80211_NTX_DEBUG_FLAG	0x0100		/* NTX debugging */
#define IEEE80211_NTX_MODE_MASK		0xFF		/* functional flags */

/* for portability, please use these macros rather than explicit bit comparisons to check status */

#define IS_NTX_FLAG_ACTIVE(mode, flag)	(((mode) & (flag)) != 0)
#define IS_NTX_ACTIVE(x)		(((x) & IEEE80211_NTX_MODE_MASK) != 0)

#endif
