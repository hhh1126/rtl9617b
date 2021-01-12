/*
 * Copyright (c) 2017 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * Common NSM mode definitions
 */
#ifndef _IEEE80211_NSM_MODE_H
#define _IEEE80211_NSM_MODE_H

/*
 *  NSM functional block flags - if set, that block is (or may be) put in NSM mode
 */
#define IEEE80211_NSM_ACTIVE_RFIC	0x0001		/* RFIC for this RF may be put in NSM mode */
#define IEEE80211_NSM_ACTIVE_AFE	0x0002		/* AFE (and FEM controls) for this MAC "" */
#define IEEE80211_NSM_ACTIVE_MAC	0x0004		/* MAC for this RF "" */
#define IEEE80211_NSM_ACTIVE_BB		0x0008		/* BB for this RF "" */
#define IEEE80211_NSM_ACTIVE_2CHAIN	0x0080		/* 2 chains active for this RF "" */

/* add new bits as needed here */

#define IEEE80211_NSM_MODE_MASK		0xFF		/* mask may be extended if more blocks need separate flags */

#define IEEE80211_NSM_DEBUG_FLAG	0x0100		/* NSM debugging - flag can be set even if NSM not active */

/* for portability, please use these macros rather than explicit bit comparisons to check status */

#define IS_NSM_BLOCK_ACTIVE(mode, flag)	(((mode) & (flag) & IEEE80211_NSM_MODE_MASK) != 0)
#define IS_NSM_FLAG_ACTIVE(mode, flag)	(((mode) & (flag)) != 0)
#define IS_NSM_ACTIVE(x)		(((x) & IEEE80211_NSM_MODE_MASK) != 0)


/* NSM chain bitmaps */
#define QTN_NSM_MAX_BB_CHAIN_MASK_2x2 0x3
#define QTN_NSM_MAX_BB_CHAIN_MASK_1x1 0x1

#endif
