/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
 * $Id: _ieee80211.h 2749 2007-10-16 08:58:14Z kelmo $
 */
#ifndef _NET80211__IEEE80211_H_
#define _NET80211__IEEE80211_H_

#include <compat.h>
#ifdef __KERNEL__
#include <linux/in6.h>
#endif

#ifndef BIT
#define BIT(x) (1U << (x))
#endif

#define IEEE80211_AX_DRAFT_VER		40
#define IEEE80211_AX_IS_DRAFT_20	(IEEE80211_AX_DRAFT_VER == 20)
#define IEEE80211_AX_IS_DRAFT_40	(IEEE80211_AX_DRAFT_VER >= 40)

enum ieee80211_phytype {
	IEEE80211_T_DS,			/* direct sequence spread spectrum */
	IEEE80211_T_FH,			/* frequency hopping */
	IEEE80211_T_OFDM,		/* frequency division multiplexing */
	IEEE80211_T_TURBO,		/* high rate OFDM, aka turbo mode */
	IEEE80211_T_HT,			/* HT - full GI */
	IEEE80211_T_VHT,
	IEEE80211_T_HE,
	IEEE80211_T_MAX
};
#define	IEEE80211_T_CCK	IEEE80211_T_DS	/* more common nomenclature */

/* Always use bit-map for representing a band in 'enum ieee80211_phy_band' */
enum ieee80211_phy_band {
	IEEE80211_BAND_UNKNOWN	= 0,
	IEEE80211_2_4Ghz	= BIT(0),
	IEEE80211_5Ghz		= BIT(1),
	IEEE80211_6Ghz		= BIT(2),
};

enum {
	BW_INVALID = 0,
	BW_HT20 = 20,
	BW_HT25_IN_OPCLASS = 25,
	BW_HT40 = 40,
	BW_HT80 = 80,
	BW_HT160 = 160,
};

#define IS_VALID_BW(bw)		(((bw) == BW_HT20 || (bw) == BW_HT40 || (bw) == BW_HT80 || (bw) == BW_HT160))

#define QTN_BW_TO_IDX(_bw)	(((_bw) == BW_HT160) ? QTN_BW_160M :	\
				(((_bw) == BW_HT80) ? QTN_BW_80M :	\
				(((_bw) == BW_HT40) ? QTN_BW_40M : QTN_BW_20M)))

#define QTN_BW_IDX_TO_BW(_bw)	(((_bw) == QTN_BW_160M) ? BW_HT160 :	\
		(((_bw) == QTN_BW_80M) ? BW_HT80 :	\
		(((_bw) == QTN_BW_40M) ?  BW_HT40 : BW_HT20)))

#define IC_UNIT_5G_6G_1			0
#define IC_UNIT_5G_6G_2			1
#define IC_UNIT_FDR_PRIMARY		IC_UNIT_5G_6G_1
#define IC_UNIT_FDR_SECONDARY		IC_UNIT_5G_6G_2
#define IC_UNIT_2PT4G			2
#define IC_UNIT_IS_2PT4G(unit)		((unit) == IC_UNIT_2PT4G)
#define IC_UNIT_IS_5G_6G_1(unit)	((unit) == IC_UNIT_5G_6G_1)
#define IC_UNIT_IS_5G_6G_2(unit)	((unit) == IC_UNIT_5G_6G_2)
#define IC_UNIT_IS_5G_6G(unit)		((unit) <= IC_UNIT_5G_6G_2)

/*
 * XXX not really a mode; there are really multiple PHY's
 * Please update ieee80211_chanflags when the definition of
 * ieee80211_phymode changed
 */
enum ieee80211_phymode {
	IEEE80211_MODE_AUTO	= 0,	/* autoselect */
	IEEE80211_MODE_11A	= 1,	/* 5GHz, OFDM */
	IEEE80211_MODE_11B	= 2,	/* 2GHz, CCK */
	IEEE80211_MODE_11G	= 3,	/* 2GHz, OFDM */
	IEEE80211_MODE_FH	= 4,	/* 2GHz, GFSK */
	IEEE80211_MODE_TURBO_A	= 5,	/* 5GHz, OFDM, 2x clock dynamic turbo */
	IEEE80211_MODE_TURBO_G	= 6,	/* 2GHz, OFDM, 2x clock  dynamic turbo*/
	IEEE80211_MODE_11NA		= 7,	/* 5GHz, HT20 */
	IEEE80211_MODE_HT_FIRST = IEEE80211_MODE_11NA,
	IEEE80211_MODE_11NG		= 8,	/* 2GHz, HT20 */
	IEEE80211_MODE_11NG_HT40PM	= 9,	/* 2GHz HT40 */
	IEEE80211_MODE_11AC_NG_VHT20PM	= 10,	/* 2G VHT20 */
	IEEE80211_MODE_11AC_NG_VHT40PM	= 11,	/* 2G VHT40 */
	IEEE80211_MODE_11NA_HT40PM	= 12,	/* 5GHz HT40 */
	IEEE80211_MODE_11AC_VHT20PM	= 13,	/* 5GHz VHT20 */
	IEEE80211_MODE_VHT_FIRST = IEEE80211_MODE_11AC_VHT20PM,
	IEEE80211_MODE_11AC_VHT40PM	= 14,	/* 5GHz VHT40 */
	IEEE80211_MODE_11AC_VHT80PM	= 15,	/* 5GHz VHT80 */
	IEEE80211_MODE_11AC_VHT160PM	= 16,	/* 5GHz VHT160 */
	IEEE80211_MODE_11AX_2G_HE20PM	= 17,	/* 2GHz HE20 */
	IEEE80211_MODE_HE_FIRST = IEEE80211_MODE_11AX_2G_HE20PM,
	IEEE80211_MODE_11AX_2G_HE40PM	= 18,	/* 2GHz HE40 */
	IEEE80211_MODE_11AX_5G_6G_HE20PM	= 19,	/* 5GHz/6GHz HE20 */
	IEEE80211_MODE_11AX_5G_6G_HE40PM	= 20,	/* 5GHz/6GHz HE40 */
	IEEE80211_MODE_11AX_5G_6G_HE80PM	= 21,	/* 5GHz/6GHz HE80 */
	IEEE80211_MODE_11AX_5G_6G_HE160PM	= 22,	/* 5GHz/6GHz HE160 */
	IEEE80211_MODE_MAX		= 23,	/* Always keep this last */
};

struct ieee80211_phymode_info {
	char name[16];
	uint8_t	type;
	uint8_t bands;
	uint8_t bw;
};

extern struct ieee80211_phymode_info qtn_phymode_info[];
#define IEEE80211_PHYMODE_NAME(_m)	(qtn_phymode_info[_m].name)
#define IEEE80211_PHYMODE_TYPE(_m)	(qtn_phymode_info[_m].type)
#define IEEE80211_PHYMODE_BANDS(_m)	(qtn_phymode_info[_m].bands)
#define IEEE80211_PHYMODE_BW(_m)	(qtn_phymode_info[_m].bw)
#define IEEE80211_PHYMODE_BW_IDX(_m)	QTN_BW_TO_IDX(IEEE80211_PHYMODE_BW(_m))
#define IEEE80211_PHYMODE_IS_LEGACY(_m)	(IEEE80211_PHYMODE_TYPE(_m) < IEEE80211_T_HT)
#define IEEE80211_PHYMODE_IS_HT(_m)	(IEEE80211_PHYMODE_TYPE(_m) == IEEE80211_T_HT)
#define IEEE80211_PHYMODE_IS_VHT(_m)	(IEEE80211_PHYMODE_TYPE(_m) == IEEE80211_T_VHT)
#define IEEE80211_PHYMODE_IS_HE(_m)	(IEEE80211_PHYMODE_TYPE(_m) == IEEE80211_T_HE)
#define IEEE80211_PHYMODE_IS_VALID_IN_24G(_m)	(IEEE80211_PHYMODE_BANDS(_m) & IEEE80211_2_4Ghz)
#define IEEE80211_PHYMODE_IS_VALID_IN_5G(_m)	(IEEE80211_PHYMODE_BANDS(_m) & IEEE80211_5Ghz)
#define IEEE80211_PHYMODE_IS_VALID_IN_6G(_m)	(IEEE80211_PHYMODE_BANDS(_m) & IEEE80211_6Ghz)

#define IEEE80211_PHYMODE_IS_BW_20(_m)		(IEEE80211_PHYMODE_BW(_m) == BW_HT20)
#define IEEE80211_PHYMODE_IS_BW_40(_m)		(IEEE80211_PHYMODE_BW(_m) == BW_HT40)
#define IEEE80211_PHYMODE_IS_BW_80(_m)		(IEEE80211_PHYMODE_BW(_m) == BW_HT80)
#define IEEE80211_PHYMODE_IS_BW_160(_m)		(IEEE80211_PHYMODE_BW(_m) == BW_HT160)

#define IEEE80211_PHYMODE_INFO	\
{	\
	/* IEEE80211_MODE_AUTO */	\
	{ "auto",	IEEE80211_T_MAX,	IEEE80211_BAND_UNKNOWN,	BW_INVALID	},	\
	/* IEEE80211_MODE_11A */	\
	{ "11a",	IEEE80211_T_OFDM,	IEEE80211_5Ghz,		BW_HT20		},	\
	/* IEEE80211_MODE_11B */	\
	{ "11b",	IEEE80211_T_CCK,	IEEE80211_2_4Ghz,	BW_HT20		},	\
	/* IEEE80211_MODE_11G */	\
	{ "11g",	IEEE80211_T_OFDM,	IEEE80211_2_4Ghz,	BW_HT20		},	\
	/* IEEE80211_MODE_FH */	\
	{ "FH",		IEEE80211_T_FH,		IEEE80211_2_4Ghz,	BW_INVALID	},	\
	/* IEEE80211_MODE_TURBO_A */	\
	{ "turboA",	IEEE80211_T_OFDM,	IEEE80211_5Ghz,		BW_INVALID	},	\
	/* IEEE80211_MODE_TURBO_G */	\
	{ "turboG",	IEEE80211_T_OFDM,	IEEE80211_2_4Ghz,	BW_INVALID	},	\
	/* IEEE80211_MODE_11NA */	\
	{ "11na",	IEEE80211_T_HT,		IEEE80211_5Ghz,		BW_HT20		},	\
	/* IEEE80211_MODE_11NG */	\
	{ "11ng",	IEEE80211_T_HT,		IEEE80211_2_4Ghz,	BW_HT20		},	\
	/* IEEE80211_MODE_11NG_HT40PM */	\
	{ "11ng40",	IEEE80211_T_HT,		IEEE80211_2_4Ghz,	BW_HT40		},	\
	/* IEEE80211_MODE_11AC_NG_VHT20PM */	\
	{ "11acng",	IEEE80211_T_VHT,	IEEE80211_2_4Ghz,	BW_HT20		},	\
	/* IEEE80211_MODE_11AC_NG_VHT40PM */	\
	{ "11acng40",	IEEE80211_T_VHT,	IEEE80211_2_4Ghz,	BW_HT40		},	\
	/* IEEE80211_MODE_11NA_HT40PM */	\
	{ "11na40",	IEEE80211_T_HT,		IEEE80211_5Ghz,		BW_HT40		},	\
	/* IEEE80211_MODE_11AC_VHT20PM */	\
	{ "11ac20",	IEEE80211_T_VHT,	IEEE80211_5Ghz,		BW_HT20		},	\
	/* IEEE80211_MODE_11AC_VHT40PM */	\
	{ "11ac40",	IEEE80211_T_VHT,	IEEE80211_5Ghz,		BW_HT40		},	\
	/* IEEE80211_MODE_11AC_VHT80PM */	\
	{ "11ac80",	IEEE80211_T_VHT,	IEEE80211_5Ghz,		BW_HT80		},	\
	/* IEEE80211_MODE_11AC_VHT160PM */	\
	{ "11ac160",	IEEE80211_T_VHT,	IEEE80211_5Ghz,		BW_HT160	},	\
	/* IEEE80211_MODE_11AX_2G_HE20PM */	\
	{ "11axng",	IEEE80211_T_HE,		IEEE80211_2_4Ghz,	BW_HT20		},	\
	/* IEEE80211_MODE_11AX_2G_HE40PM */	\
	{ "11axng40",	IEEE80211_T_HE,		IEEE80211_2_4Ghz,	BW_HT40		},	\
	/* IEEE80211_MODE_11AX_5G_6G_HE20PM */	\
	{ "11ax20",	IEEE80211_T_HE,		(IEEE80211_5Ghz | IEEE80211_6Ghz),	BW_HT20}, \
	/* IEEE80211_MODE_11AX_5G_6G_HE40PM */	\
	{ "11ax40",	IEEE80211_T_HE,		(IEEE80211_5Ghz | IEEE80211_6Ghz),	BW_HT40}, \
	/* IEEE80211_MODE_11AX_5G_6G_HE80PM */	\
	{ "11ax80",	IEEE80211_T_HE,		(IEEE80211_5Ghz | IEEE80211_6Ghz),	BW_HT80}, \
	/* IEEE80211_MODE_11AX_5G_6G_HE160PM */	\
	{ "11ax160",	IEEE80211_T_HE,		(IEEE80211_5Ghz | IEEE80211_6Ghz),	BW_HT160},\
}

static __inline__
enum ieee80211_phymode ieee80211_find_phymode(uint8_t type, uint8_t band, uint8_t bw)
{
	int i;
	enum ieee80211_phymode phymode;

	for (i = IEEE80211_MODE_AUTO; i < IEEE80211_MODE_MAX; i++) {
		phymode = (enum ieee80211_phymode) i;
		if ((IEEE80211_PHYMODE_TYPE(phymode) == type) &&
				(IEEE80211_PHYMODE_BANDS(phymode) & band) &&
				(IEEE80211_PHYMODE_BW(phymode) == bw))
			return phymode;
	}

	return IEEE80211_MODE_AUTO;
}

static __inline__
void ieee80211_phymode_info_update_6g_vht(void)
{
	int phymode;

	for (phymode = IEEE80211_MODE_AUTO; phymode < IEEE80211_MODE_MAX; phymode++) {
		if (IEEE80211_PHYMODE_IS_VHT(phymode) && IEEE80211_PHYMODE_IS_VALID_IN_5G(phymode))
			qtn_phymode_info[phymode].bands |= IEEE80211_6Ghz;
	}
}

#define IEEE80211_IS_11B(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11B)

#define IEEE80211_IS_11G(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11G)

#define IEEE80211_IS_11NG_20(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11NG)

#define IEEE80211_IS_11NG_40(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11NG_HT40PM)

#define IS_IEEE80211_11NG(_c) \
	(IEEE80211_IS_11NG_20(_c) || IEEE80211_IS_11NG_40(_c))

#define IEEE80211_IS_11A(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11A)

#define IEEE80211_IS_11NA_20(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11NA)

#define IEEE80211_IS_11NA_40(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11NA_HT40PM)

#define IS_IEEE80211_11NA(_c) \
	(IEEE80211_IS_11NA_20(_c) || IEEE80211_IS_11NA_40(_c))

/* HT */
#define IEEE80211_IS_HT_2G_20(_c) \
	(IEEE80211_IS_11NG_20(_c) || IEEE80211_IS_VHT_2G_20(_c) || IEEE80211_IS_HE_2G_20(_c))

#define IEEE80211_IS_HT_2G_40(_c) \
	(IEEE80211_IS_11NG_40(_c) || IEEE80211_IS_VHT_2G_40(_c) || IEEE80211_IS_HE_2G_40(_c))

#define IEEE80211_IS_HT_5G_20(_c) \
	(IEEE80211_IS_11NA_20(_c) || IEEE80211_IS_VHT_5G_20(_c) || IEEE80211_IS_HE_5G_20(_c))

#define IEEE80211_IS_HT_5G_40(_c) \
	(IEEE80211_IS_11NA_40(_c) || IEEE80211_IS_VHT_5G_40(_c) || IEEE80211_IS_HE_5G_40(_c))

#define IS_IEEE80211_HT_2G_ENABLED(_c) \
	(IEEE80211_IS_HT_2G_40(_c) || IEEE80211_IS_HT_2G_20(_c))

#define IS_IEEE80211_HT_5G_ENABLED(_c) \
	(IEEE80211_IS_HT_5G_40(_c) || IEEE80211_IS_HT_5G_20(_c) || \
	 IS_IEEE80211_VHT_5G_ENABLED(_c) || IS_IEEE80211_HE_5G_ENABLED(_c))

#define IS_IEEE80211_HT_ENABLED(_c) \
	(!IEEE80211_IS_6G_HE_ONLY(_c) && \
	(IS_IEEE80211_HT_5G_ENABLED(_c) || IS_IEEE80211_HT_2G_ENABLED(_c)))

/* VHT */
#define IEEE80211_IS_VHT_2G_20(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_NG_VHT20PM) || \
	 (IEEE80211_IS_HE_2G_20(_c) && ((_c)->ic_flags_ext & IEEE80211_FEXT_11AX_24G_VHT)))

#define IEEE80211_IS_VHT_2G_40(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_NG_VHT40PM) || \
	 (IEEE80211_IS_HE_2G_40(_c) && ((_c)->ic_flags_ext & IEEE80211_FEXT_11AX_24G_VHT)))

#define IEEE80211_IS_VHT_5G_20(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_VHT20PM) || IEEE80211_IS_HE_5G_20(_c))

#define IEEE80211_IS_VHT_5G_40(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_VHT40PM) || IEEE80211_IS_HE_5G_40(_c))

#define IEEE80211_IS_VHT_5G_80(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_VHT80PM) || IEEE80211_IS_HE_5G_80(_c))

#define IEEE80211_IS_VHT_5G_160(_c) \
	(((_c)->ic_phymode == IEEE80211_MODE_11AC_VHT160PM) || IEEE80211_IS_HE_5G_160(_c))

#define IS_IEEE80211_VHT_2G_ENABLED(_c) \
	(IEEE80211_IS_VHT_2G_40(_c) || IEEE80211_IS_VHT_2G_20(_c))

#define IS_IEEE80211_VHT_5G_ENABLED(_c) \
	(IEEE80211_IS_VHT_5G_160(_c) || IEEE80211_IS_VHT_5G_80(_c) || \
	 IEEE80211_IS_VHT_5G_40(_c) || IEEE80211_IS_VHT_5G_20(_c))

#define IS_IEEE80211_VHT_ENABLED(_c) \
	(!IEEE80211_IS_6G_HE_ONLY(_c) && \
	(IS_IEEE80211_VHT_5G_ENABLED(_c) || IS_IEEE80211_VHT_2G_ENABLED(_c)))

/* HE */
#define IEEE80211_IS_HE_2G_20(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_2G_HE20PM)

#define IEEE80211_IS_HE_2G_40(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_2G_HE40PM)

#define IEEE80211_IS_HE_5G_20(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE20PM)

#define IEEE80211_IS_HE_5G_40(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE40PM)

#define IEEE80211_IS_HE_5G_80(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE80PM)

#define IEEE80211_IS_HE_5G_160(_c) \
	((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE160PM)

#define IEEE80211_IS_6G_HE_ONLY(_c) \
	(!!((_c)->ic_flags_ext2 & IEEE80211_FEXT2_6G_HE_ONLY))

#define IS_IEEE80211_HE_2G_ENABLED(_c) \
	(IEEE80211_IS_HE_2G_40(_c) || IEEE80211_IS_HE_2G_20(_c))

#define IS_IEEE80211_HE_5G_ENABLED(_c) \
	(((_c)->ic_phy_oper_band == IEEE80211_5Ghz) && \
		(IEEE80211_IS_HE_5G_160(_c) || IEEE80211_IS_HE_5G_80(_c) || \
		 IEEE80211_IS_HE_5G_40(_c) || IEEE80211_IS_HE_5G_20(_c)))

#define IS_IEEE80211_HE_6G_ENABLED(_c) \
	(((_c)->ic_phy_oper_band == IEEE80211_6Ghz) && \
		(((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE20PM) || \
		 ((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE40PM) || \
		 ((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE80PM) || \
		 ((_c)->ic_phymode == IEEE80211_MODE_11AX_5G_6G_HE160PM)))

#define IS_IEEE80211_HE_ENABLED(_c) \
	(IS_IEEE80211_HE_6G_ENABLED(_c) || IS_IEEE80211_HE_5G_ENABLED(_c) || \
	 IS_IEEE80211_HE_2G_ENABLED(_c))

#define IEEE80211_IS_VHT_6G_ENABLED(_c) \
	(!!((_c)->ic_flags_ext2 & IEEE80211_FEXT2_6G_ENABLE_VHT))

/* Band */
#define IS_IEEE80211_24G_BAND(_c) \
	(IEEE80211_PHYMODE_IS_VALID_IN_24G((_c)->ic_phymode) && \
	 ((_c)->ic_phy_oper_band == IEEE80211_2_4Ghz))

#define IS_IEEE80211_5G_BAND(_c) \
	(IEEE80211_PHYMODE_IS_VALID_IN_5G((_c)->ic_phymode) && \
	 ((_c)->ic_phy_oper_band == IEEE80211_5Ghz))

#define IS_IEEE80211_6G_BAND(_c) \
	(IEEE80211_PHYMODE_IS_VALID_IN_6G((_c)->ic_phymode) && \
	 ((_c)->ic_phy_oper_band == IEEE80211_6Ghz))

#define IEEE80211_DPDUR_UNIT		50
#define IEEE80211_DPDUR_MAX_NONE	0	/* no direct constraint on the maximum duration */
#define IEEE80211_DPDUR_MAX_HT			(10000/(IEEE80211_DPDUR_UNIT))	/* us */
#define IEEE80211_DPDUR_MAX_VHT			(5484/(IEEE80211_DPDUR_UNIT))	/* us */
#define IEEE80211_DPDUR_MAX_HT_EU_RED_VO	(2000/(IEEE80211_DPDUR_UNIT))	/* us */
#define IEEE80211_DPDUR_MAX_HT_EU_RED_VI	(4000/(IEEE80211_DPDUR_UNIT))	/* us */
#define IEEE80211_DPDUR_MAX_HT_EU_RED_BE	(6000/(IEEE80211_DPDUR_UNIT))	/* us */
#define IEEE80211_DPDUR_MAX_HT_EU_RED_BK	(6000/(IEEE80211_DPDUR_UNIT))	/* us */

#define IEEE80211_DPDUR_VO	0xff
#define IEEE80211_DPDUR_VO_S	0
#define IEEE80211_DPDUR_VI	0xff00
#define IEEE80211_DPDUR_VI_S	8
#define IEEE80211_DPDUR_BE	0xff0000
#define IEEE80211_DPDUR_BE_S	16
#define IEEE80211_DPDUR_BK	0xff000000
#define IEEE80211_DPDUR_BK_S	24

#define IEEE80211_DPDUR_MERGE_4AC_MAX_DUR(vo, vi, be, bk) \
		(SM(IEEE80211_DPDUR_MAX_ ## vo, IEEE80211_DPDUR_VO) | \
		SM(IEEE80211_DPDUR_MAX_ ## vi, IEEE80211_DPDUR_VI) | \
		SM(IEEE80211_DPDUR_MAX_ ## be, IEEE80211_DPDUR_BE) | \
		SM(IEEE80211_DPDUR_MAX_ ## bk, IEEE80211_DPDUR_BK))
#define IEEE80211_DPDUR_MERGE_4AC_MAX_DUR_SAME(dur)	\
		IEEE80211_DPDUR_MERGE_4AC_MAX_DUR(dur, dur, dur, dur)

enum ieee80211_opmode {
	IEEE80211_M_STA		= 1,	/* infrastructure station */
	IEEE80211_M_IBSS	= 0,	/* IBSS (adhoc) station */
	IEEE80211_M_AHDEMO	= 3,	/* Old lucent compatible adhoc demo */
	IEEE80211_M_HOSTAP	= 6,	/* Software Access Point */
	IEEE80211_M_MONITOR	= 8,	/* Monitor mode */
	IEEE80211_M_WDS		= 2	/* WDS link */
};

/*
 * 802.11n
 */

enum ieee80211_11n_htmode {
	IEEE80211_11N_HTAUTO	=  0,
	IEEE80211_11N_HT20	=  1,
	IEEE80211_11N_HT40PLUS	=  2,
	IEEE80211_11N_HT40MINUS	=  3
};

enum ieee80211_cwm_mode {
	IEEE80211_CWM_MODE20,
	IEEE80211_CWM_MODE2040,
	IEEE80211_CWM_MODE40,
	IEEE80211_CWM_MODEMAX

};

enum ieee80211_cwm_extprotspacing {
	IEEE80211_CWM_EXTPROTSPACING20,
	IEEE80211_CWM_EXTPROTSPACING25,
	IEEE80211_CWM_EXTPROTSPACINGMAX
};

enum ieee80211_cwm_width {
	IEEE80211_CWM_WIDTH20,
	IEEE80211_CWM_WIDTH40,
	IEEE80211_CWM_WIDTH80,
	IEEE80211_CWM_WIDTH160,	/* or 80+80 Mhz */
};

enum ieee80211_cwm_extprotmode {
	IEEE80211_CWM_EXTPROTNONE,	/* no protection */
	IEEE80211_CWM_EXTPROTCTSONLY,	/* CTS to self */
	IEEE80211_CWM_EXTPROTRTSCTS,	/* RTS-CTS */
	IEEE80211_CWM_EXTPROTMAX
};

/* CWM (Channel Width Management) Information */
struct ieee80211_cwm {

	/* Configuration */
	enum ieee80211_cwm_mode		cw_mode;		/* CWM mode */
	int8_t				cw_extoffset;		/* CWM Extension Channel Offset */
	enum ieee80211_cwm_extprotmode	cw_extprotmode;		/* CWM Extension Channel Protection Mode */
	enum ieee80211_cwm_extprotspacing cw_extprotspacing;	/* CWM Extension Channel Protection Spacing */

	/* State */
	 enum ieee80211_cwm_width	cw_width;		/* CWM channel width */
};


enum ieee80211_fixed_rate_status {
	DISABLED = 0,
	ENABLED = 1,	/* ieee rate */
};

/* Holds the fixed rate information for each VAP */
#define IEEE80211_MAX_FIXED_RATES		4
struct ieee80211_fixed_rate {
	enum ieee80211_fixed_rate_status	status;
	u_int8_t				rate[IEEE80211_MAX_FIXED_RATES];
	u_int32_t				retries;
	u_int32_t				mode;
};

/*
 * 802.11g protection mode.
 */
enum ieee80211_protmode {
	IEEE80211_PROT_NONE	= 0,	/* no protection */
	IEEE80211_PROT_CTSONLY	= 1,	/* CTS to self */
	IEEE80211_PROT_RTSCTS	= 2,	/* RTS-CTS */
};

/*
 * Authentication mode.
 */
enum ieee80211_authmode {
	IEEE80211_AUTH_NONE	= 0,
	IEEE80211_AUTH_OPEN	= 1,	/* open */
	IEEE80211_AUTH_SHARED	= 2,	/* shared-key */
	IEEE80211_AUTH_8021X	= 3,	/* 802.1x */
	IEEE80211_AUTH_AUTO	= 4,	/* auto-select/accept */
	/* NB: these are used only for ioctls */
	IEEE80211_AUTH_WPA	= 5,	/* WPA/RSN w/ 802.1x/PSK */
	IEEE80211_AUTH_SAE	= 6,	/* SAE */
	IEEE80211_AUTH_MAX	= 7,	/* Should be the last */
};

/*
 * Roaming mode is effectively who controls the operation
 * of the 802.11 state machine when operating as a station.
 * State transitions are controlled either by the driver
 * (typically when management frames are processed by the
 * hardware/firmware), the host (auto/normal operation of
 * the 802.11 layer), or explicitly through ioctl requests
 * when applications like wpa_supplicant want control.
 */
enum ieee80211_roamingmode {
	IEEE80211_ROAMING_DEVICE= 0,	/* driver/hardware control */
	IEEE80211_ROAMING_AUTO	= 1,	/* 802.11 layer control */
	IEEE80211_ROAMING_MANUAL= 2,	/* application control */
};

/*
 * Scanning mode controls station scanning work; this is
 * used only when roaming mode permits the host to select
 * the bss to join/channel to use.
 */
enum ieee80211_scanmode {
	IEEE80211_SCAN_DEVICE	= 0,	/* driver/hardware control */
	IEEE80211_SCAN_BEST	= 1,	/* 802.11 layer selects best */
	IEEE80211_SCAN_FIRST	= 2,	/* take first suitable candidate */
};

/* ba state describes the block ack state. block ack should only be sent if
 * ba state is set to established
 */
enum ieee80211_ba_state	{
	IEEE80211_BA_NOT_ESTABLISHED = 0,
	IEEE80211_BA_ESTABLISHED = 1,
	IEEE80211_BA_REQUESTED = 2,
	IEEE80211_BA_FAILED = 5,
	IEEE80211_BA_BLOCKED = 6,
};

#define IEEE80211_BA_IS_COMPLETE(_state) (\
	(_state) == IEEE80211_BA_ESTABLISHED || \
	(_state) == IEEE80211_BA_BLOCKED || \
	(_state) == IEEE80211_BA_FAILED) \

/* ba type describes the block acknowledgement type */
enum ieee80211_ba_type {
	IEEE80211_BA_DELAYED = 0,
	IEEE80211_BA_IMMEDIATE = 1,
};

#define IEEE80211_NORMAL_CAC	0x1
#define IEEE80211_OCAC		0x2
#define IEEE80211_ICAC		0x4

#define IEEE80211_OPER_CLASS_MAX	256
#define	IEEE80211_OPER_CLASS_BYTES	32
#define	IEEE80211_OPER_CLASS_BYTES_24G	8

#define IEEE80211_OC_BEHAV_DFS_50_100		0x0001
#define IEEE80211_OC_BEHAV_NOMADIC		0x0002
#define IEEE80211_OC_BEHAV_LICEN_EXEP		0x0004
#define IEEE80211_OC_BEHAV_CCA_ED		0x0008
#define IEEE80211_OC_BEHAV_ITS_NONMOB		0x0010
#define IEEE80211_OC_BEHAV_ITS_MOBILE		0x0020
#define IEEE80211_OC_BEHAV_CHAN_LOWER		0x0040
#define IEEE80211_OC_BEHAV_CHAN_UPPER		0x0080
#define IEEE80211_OC_BEHAV_80PLUS		0x0100
#define IEEE80211_OC_BEHAV_EIRP_TXPOWENV	0x0200

#define IEEE80211_OBSS_CHAN_PRI20		0x01
#define IEEE80211_OBSS_CHAN_SEC20		0x02
#define IEEE80211_OBSS_CHAN_PRI40		0x04
#define IEEE80211_OBSS_CHAN_SEC40		0x08

#define IEEE80211_OBSS_CHAN_MASK		0x0F

#define IEEE80211_IS_OBSS_CHAN_SECONDARY(_c) \
	(((_c) & IEEE80211_OBSS_CHAN_SEC20) == IEEE80211_OBSS_CHAN_SEC20)

#define PWR_BW_IDX_TO_QTN_BW(pwr_bw)	(QTN_BW_20M + (pwr_bw) - PWR_IDX_20M)
#define QTN_BW_TO_PWR_BW_IDX(qtn_bw)	(PWR_IDX_20M + (qtn_bw) - QTN_BW_20M)

#define BW_TO_PWR_BW_IDX(_bw)	QTN_BW_TO_PWR_BW_IDX(QTN_BW_TO_IDX(_bw))


enum ieee80211_dynamic_ntx_idx {
	DNTX_IDX_BASIS = 0,
	DNTX_IDX_OPTION,	/* for NTX=4 */
	DNTX_IDX_MAX
};

/* MFP capabilities (ieee80211w) */
enum ieee80211_mfp_capabilities {
	IEEE80211_MFP_NO_PROTECT = 0,
	IEEE80211_MFP_PROTECT_CAPABLE = 2,
	IEEE80211_MFP_PROTECT_REQUIRE = 3
};

/*
 * Channels are specified by frequency and attributes.
 */
enum ieee80211_bw_support_regpower {
	IEEE80211_MAXREGPOWER_STD = 0,
	IEEE80211_MAXREGPOWER_160MHZ = 1,
	IEEE80211_MAXREGPOWER_TBL_MAX = 2
};

struct ieee80211_channel {
	/*
	 * The params 'ic_ieee', 'cchan_40', 'cchan_80' and 'cchan_160'
	 * uses the following bit definitions to represent band and channel number
	 *    Bits 15-9: 0 (reserved)
	 *    Bit  8: band.  0 --> 2.4GHz/5GHz, 1 --> 6GHz
	 *    Bits 7-0: channel number
	 */
#define IC_IEEE_CHANNEL_MASK		0x00FF
#define IC_IEEE_BAND_MASK		0x0100
#define IC_IEEE_BAND_MASK_S		8
#define IC_IEEE_BAND_CHANNEL_MASK	(IC_IEEE_BAND_MASK | IC_IEEE_CHANNEL_MASK)
#define IC_IEEE_BAND_2G_OR_5G		0
#define IC_IEEE_BAND_6G			1

#define IC_IEEE_GET_CHAN_NUM(ic_ieee)		((ic_ieee) & IC_IEEE_CHANNEL_MASK)
#define IC_IEEE_GET_BAND(ic_ieee)		(MS((ic_ieee), IC_IEEE_BAND_MASK))
#define IC_IEEE_GET_CHAN_AND_BAND(ic_ieee)	((ic_ieee) & IC_IEEE_BAND_CHANNEL_MASK)
#define IC_IEEE_SET_VALID_CHAN_RANGE(ic_ieee)	((ic_ieee) & IC_IEEE_BAND_CHANNEL_MASK)
#define IC_IEEE_GET_IEEE_CH_NUMBER(_band, _chan) \
		((SM((_band), IC_IEEE_BAND_MASK)) | ((_chan) & IC_IEEE_CHANNEL_MASK))
#define IC_IEEE_IS_CHAN_IN_2G_OR_5G(ic_ieee) \
		(((IC_IEEE_GET_BAND(ic_ieee)) == IC_IEEE_BAND_2G_OR_5G) ? 1 : 0)
#define IC_IEEE_IS_CHAN_IN_2G(ic_ieee) \
		((IC_IEEE_IS_CHAN_IN_2G_OR_5G(ic_ieee)) && (QTN_IS_2G_OPER_CHAN(ic_ieee)))
#define IC_IEEE_IS_CHAN_IN_5G(ic_ieee) \
		((IC_IEEE_IS_CHAN_IN_2G_OR_5G(ic_ieee)) && (QTN_IS_5G_OPER_CHAN(ic_ieee)))
#define IC_IEEE_IS_CHAN_IN_6G(ic_ieee) \
		(((IC_IEEE_GET_BAND(ic_ieee)) == IC_IEEE_BAND_6G) ? 1 : 0)
	uint16_t ic_ieee;	/* IEEE channel number */
	uint16_t cchan_40;
	uint16_t cchan_80;
	uint16_t cchan_160;
	uint16_t ic_freq;	/* setting in Mhz */
	/* maximum regulatory tx power in dBm based on bandwidth */
	int8_t ic_maxregpower[IEEE80211_MAXREGPOWER_TBL_MAX];
	int8_t ic_maxpower;	/* maximum tx power in dBm for the current bandwidth with beam-forming off */
	int8_t ic_minpower;	/* minimum tx power in dBm */
	int8_t ic_maxpower_normal;	/* backup max tx power for short-range workaround */
	int8_t ic_minpower_normal;	/* backup min tx power for short-range workaround */
	struct qtn_shared_ch_power_table	*power_table;
	uint32_t ic_flags;	/* one of IEEE80211_CHAN_* */
	uint32_t ic_ext_flags;
	uint32_t ic_radardetected; /* number that radar signal has been detected on this channel */
	uint16_t txpower_backoff; /* txpower backoff */
};

#define IEEE80211_UNKNOWN_FREQ		0
#define IEEE80211_2GBAND_START_FREQ	2407
#define IEEE80211_4GBAND_START_FREQ	4000
#define IEEE80211_5GBAND_START_FREQ	5000
#define IEEE80211_6GBAND_START_FREQ	QTN_FREQ_6GHZ_BAND_START
#define IEEE80211_6GBAND_END_FREQ	QTN_FREQ_6GHZ_BAND_END
#define IEEE80211_CHAN_14_CENTER_FREQ	2484

#define IEEE80211_FREQ_RANGE_MIN_24G		(2412 - 10)
#define IEEE80211_FREQ_RANGE_MAX_24G		(2484 + 10)
#define IEEE80211_FREQ_RANGE_MIN_5G		(5150 - 10)
#define IEEE80211_FREQ_RANGE_MAX_5G		(5850 + 10)
#define IEEE80211_FREQ_RANGE_MIN_6G		(QTN_FREQ_6GHZ_BAND_START)
#define IEEE80211_FREQ_RANGE_MAX_6G		(QTN_FREQ_6GHZ_BAND_END)
#define IEEE80211_FREQ_RANGE_MIN_5G_OR_6G	(IEEE80211_FREQ_RANGE_MIN_5G)
#define IEEE80211_FREQ_RANGE_MAX_5G_OR_6G	(QTN_FREQ_6GHZ_BAND_END)

#define IEEE80211_CHAN_TO_FREQ_6G(_c)	(QTN_FREQ_6GHZ_FROM_CHAN_NO(_c))
#define IEEE80211_FREQ_TO_CHAN_6G(_f)	(QTN_CHAN_NO_FROM_FREQ_6G(_f))

#define	IEEE80211_CHAN_MAX		255
#define	IEEE80211_CHAN_MAX_EXT		511
#define	IEEE80211_CHAN_BYTES		32	/* howmany(IEEE80211_CHAN_MAX, NBBY) */
#define	IEEE80211_CHAN_BYTES_EXT	64	/* howmany(IEEE80211_CHAN_MAX_EXT, NBBY) */

#define	IEEE80211_CHAN_ANY	0xffff	/* token for ``any channel'' */
#define	IEEE80211_CHAN_ANYC	((struct ieee80211_channel *) IEEE80211_CHAN_ANY)

#define IEEE80211_SUBCHANNELS_OF_20MHZ	1
#define IEEE80211_SUBCHANNELS_OF_40MHZ	2
#define IEEE80211_SUBCHANNELS_OF_80MHZ	4
#define IEEE80211_SUBCHANNELS_OF_160MHZ	8

#define IEEE80211_MIN_DUAL_EXT_CHAN_24G		5
#define IEEE80211_MAX_DUAL_EXT_CHAN_24G		9
#define IEEE80211_MAX_DUAL_EXT_CHAN_24G_US	7

#define	IEEE80211_RADAR_11HCOUNT		1
#define IEEE80211_DEFAULT_CHANCHANGE_TBTT_COUNT	10
#define	IEEE80211_RADAR_TEST_MUTE_CHAN	36	/* Move to channel 36 for mute test */

/* bits 0-3 are for private use by drivers */
/* channel attributes */
#define IEEE80211_CHAN_TURBO			0x00000010  /* Turbo channel */
#define IEEE80211_CHAN_CCK			0x00000020  /* CCK channel */
#define IEEE80211_CHAN_OFDM			0x00000040  /* OFDM channel */
#define IEEE80211_CHAN_2GHZ			0x00000080  /* 2 GHz spectrum channel. */
#define IEEE80211_CHAN_5GHZ			0x00000100  /* 5 GHz spectrum channel */
#define IEEE80211_CHAN_PASSIVE			0x00000200  /* Only passive scan allowed */
#define IEEE80211_CHAN_DYN			0x00000400  /* Dynamic CCK-OFDM channel */
#define IEEE80211_CHAN_GFSK			0x00000800  /* GFSK channel (FHSS PHY) */
#define IEEE80211_CHAN_RADAR			0x00001000  /* Status: Radar found on channel */
#define IEEE80211_CHAN_STURBO			0x00002000  /* 11a static turbo channel only */
#define IEEE80211_CHAN_HALF			0x00004000  /* Half rate channel */
#define IEEE80211_CHAN_QUARTER			0x00008000  /* Quarter rate channel */
#define IEEE80211_CHAN_BW20			0x00010000  /* BW 20 channel */
#define IEEE80211_CHAN_BW40_U			0x00020000  /* BW 40 with ext ch above */
#define IEEE80211_CHAN_BW40_D			0x00040000  /* BW 40 with ext ch below */
#define IEEE80211_CHAN_BW40			0x00080000  /* BW 40 channel */
#define IEEE80211_CHAN_DFS			0x00100000  /* Config: DFS-required channel */
#define IEEE80211_CHAN_DFS_CAC_DONE		0x00200000  /* Status: CAC completed */
#define IEEE80211_CHAN_BW80			0x00400000  /* BW 80 channel */
#define IEEE80211_CHAN_DFS_OCAC_DONE		0x00800000  /* Status: Off-channel CAC completed */
#define IEEE80211_CHAN_DFS_CAC_IN_PROGRESS	0x01000000  /* Status: Valid CAC is in progress */
#define IEEE80211_CHAN_WEATHER			0x02000000  /* weather channel */
#define IEEE80211_CHAN_WEATHER_40M		0x04000000  /* weather ch exist within 40Mhz */
#define IEEE80211_CHAN_WEATHER_80M		0x08000000  /* weather ch exist within 80Mhz */
#define IEEE80211_CHAN_WEATHER_160M		0x10000000  /* weather ch exist within 160Mhz */
#define IEEE80211_CHAN_BW160			0x20000000  /* BW 160 channel */
#define IEEE80211_CHAN_AC_NG			0x40000000  /* 11ac on 2.4G */
#define IEEE80211_CHAN_6GHZ			0x80000000  /* 6GHz spectrum channel */

#define IEEE80211_DEFAULT_2_4_GHZ_CHANNEL	1
#define IEEE80211_DEFAULT_5_GHZ_CHANNEL		36
#define IEEE80211_DEFAULT_6_GHZ_CHANNEL		5

#define IEEE80211_MAX_2_4_GHZ_CHANNELS		14
#define IEEE80211_MAX_5_GHZ_CHANNELS		30
#define IEEE80211_MAX_6_GHZ_CHANNELS		59
#define IEEE80211_MAX_DUAL_BAND_CHANNELS	(IEEE80211_MAX_2_4_GHZ_CHANNELS + \
						 IEEE80211_MAX_5_GHZ_CHANNELS)
#define IEEE80211_MAX_TRI_BAND_CHANNELS		(IEEE80211_MAX_2_4_GHZ_CHANNELS + \
						 IEEE80211_MAX_5_GHZ_CHANNELS + \
						 IEEE80211_MAX_6_GHZ_CHANNELS)

enum {
	CHIPID_2_4_GHZ		= 0,
	CHIPID_5_GHZ		= 1,
	CHIPID_5GHZ_6GHZ	= 2,	/* Supporting 5GHz & 6GHz */
	CHIPID_INVALID		= 0xFF
};

/* below are channel ext attributes(ic_ext_flags) */
/* 80MHZ flags, please don't change the relative position of them */
#define IEEE80211_CHAN_BW80_LL		0x00000001
#define IEEE80211_CHAN_BW80_LU		0x00000002
#define IEEE80211_CHAN_BW80_UL		0x00000004
#define IEEE80211_CHAN_BW80_UU		0x00000008
/* 160MHZ flags, please don't change the relative position of them */
#define IEEE80211_CHAN_BW160_LLL	0x00000010
#define IEEE80211_CHAN_BW160_LLU	0x00000020
#define IEEE80211_CHAN_BW160_LUL	0x00000040
#define IEEE80211_CHAN_BW160_LUU	0x00000080
#define IEEE80211_CHAN_BW160_ULL	0x00000100
#define IEEE80211_CHAN_BW160_ULU	0x00000200
#define IEEE80211_CHAN_BW160_UUL	0x00000400
#define IEEE80211_CHAN_BW160_UUU	0x00000800

#define IEEE80211_CHAN_DFS_40M		0x00002000	/* Configuration: DFS channel exist within 40Mhz */
#define IEEE80211_CHAN_DFS_80M		0x00004000	/* Configuration: DFS channel exist within 80Mhz */
#define IEEE80211_CHAN_DFS_160M		0x00008000	/* Configuration: DFS channel exist within 160Mhz */

#define IEEE80211_CHAN_FLAG_EXT_PSC		0x00010000	/* 6G PSC channel */

#define IEEE80211_CHAN_FLAG_EXT_BAND_ISM	0x00020000  /* 2.4G: ch 1-14 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_1	0x00040000  /* 5G: ch 34-48 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_2A	0x00080000  /* 5G: ch 52-64 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_2C	0x00100000  /* 5G: ch 100-144 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_3	0x00200000  /* 5G: ch 149-169 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_JAPAN	0x00400000  /* 5G: ch 184-196 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_5	0x00800000  /* 6G: ch 1-97 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_6	0x01000000  /* 6G: ch 101-113 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_7	0x02000000  /* 6G: ch 117-185 */
#define IEEE80211_CHAN_FLAG_EXT_BAND_UNII_8	0x04000000  /* 6G: ch 189-237 */


/* 11ax has similar channel structure with 11ac, so there's no need to
 * define another set of channel flags for HE40U,HE40D,HE80LL,HE80LU,...
 * so let's seperate channel position and channel capabilities in the
 * future, and old macros are just linked to the new macros to avoid
 * conflicts while merging code
 */
#define IEEE80211_CHAN_BW40_MASK	(IEEE80211_CHAN_BW40_U | \
					 IEEE80211_CHAN_BW40_D)

#define IEEE80211_CHAN_BW80_MASK	(IEEE80211_CHAN_BW80_LL | \
					 IEEE80211_CHAN_BW80_LU | \
					 IEEE80211_CHAN_BW80_UL | \
					 IEEE80211_CHAN_BW80_UU)

#define IEEE80211_CHAN_BW160_MASK	(IEEE80211_CHAN_BW160_LLL | \
					 IEEE80211_CHAN_BW160_LLU | \
					 IEEE80211_CHAN_BW160_LUL | \
					 IEEE80211_CHAN_BW160_LUU | \
					 IEEE80211_CHAN_BW160_ULL | \
					 IEEE80211_CHAN_BW160_ULU | \
					 IEEE80211_CHAN_BW160_UUL | \
					 IEEE80211_CHAN_BW160_UUU)

#define IEEE80211_IS_CHAN_BW40(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_BW40_MASK) != 0)
#define IEEE80211_IS_CHAN_BW80(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW80_MASK) != 0)
#define IEEE80211_IS_CHAN_BW160(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW160_MASK) != 0)

#define	IEEE80211_IS_CHAN_BW40PLUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_BW40_U) == IEEE80211_CHAN_BW40_U)
#define	IEEE80211_IS_CHAN_BW40MINUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_BW40_D) == IEEE80211_CHAN_BW40_D)
#define IEEE80211_IS_CHAN_BW80_EDGEPLUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW80_LL) == IEEE80211_CHAN_BW80_LL)
#define IEEE80211_IS_CHAN_BW80_CNTRPLUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW80_LU) == IEEE80211_CHAN_BW80_LU)
#define IEEE80211_IS_CHAN_BW80_CNTRMINUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW80_UL) == IEEE80211_CHAN_BW80_UL)
#define IEEE80211_IS_CHAN_BW80_EDGEMINUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_BW80_UU) == IEEE80211_CHAN_BW80_UU)

#define IEEE80211_CHAN_HT40U		IEEE80211_CHAN_BW40_U	/* HT 40 with ext channel above */
#define IEEE80211_CHAN_HT40D		IEEE80211_CHAN_BW40_D	/* HT 40 with ext channel below */
#define IEEE80211_CHAN_VHT40U		IEEE80211_CHAN_BW40_U	/* VHT 40 with ext channel above */
#define IEEE80211_CHAN_VHT40D		IEEE80211_CHAN_BW40_D	/* VHT 40 with ext channel below */
#define IEEE80211_CHAN_VHT80_LL		IEEE80211_CHAN_BW80_LL
#define IEEE80211_CHAN_VHT80_LU		IEEE80211_CHAN_BW80_LU
#define IEEE80211_CHAN_VHT80_UL		IEEE80211_CHAN_BW80_UL
#define IEEE80211_CHAN_VHT80_UU		IEEE80211_CHAN_BW80_UU
#define IEEE80211_CHAN_VHT80_MASK	IEEE80211_CHAN_BW80_MASK
#define IEEE80211_CHAN_VHT160_LLL	IEEE80211_CHAN_BW160_LLL
#define IEEE80211_CHAN_VHT160_LLU	IEEE80211_CHAN_BW160_LLU
#define IEEE80211_CHAN_VHT160_LUL	IEEE80211_CHAN_BW160_LUL
#define IEEE80211_CHAN_VHT160_LUU	IEEE80211_CHAN_BW160_LUU
#define IEEE80211_CHAN_VHT160_ULL	IEEE80211_CHAN_BW160_ULL
#define IEEE80211_CHAN_VHT160_ULU	IEEE80211_CHAN_BW160_ULU
#define IEEE80211_CHAN_VHT160_UUL	IEEE80211_CHAN_BW160_UUL
#define IEEE80211_CHAN_VHT160_UUU	IEEE80211_CHAN_BW160_UUU
#define IEEE80211_CHAN_VHT160_MASK	IEEE80211_CHAN_BW160_MASK

#define IEEE80211_CHAN_VHT160_80L_MASK	(IEEE80211_CHAN_VHT160_LLL | \
					IEEE80211_CHAN_VHT160_LLU | \
					IEEE80211_CHAN_VHT160_LUL | \
					IEEE80211_CHAN_VHT160_LUU)

#define IEEE80211_CHAN_VHT160_80U_MASK	(IEEE80211_CHAN_VHT160_ULL | \
					IEEE80211_CHAN_VHT160_ULU | \
					IEEE80211_CHAN_VHT160_UUL | \
					IEEE80211_CHAN_VHT160_UUU)

enum {
	QTN_VHT80_SEC40L_CHAN_INDEX = 0,
	QTN_VHT80_SEC40U_CHAN_INDEX = 1,
	QTN_VHT80_SEC40_CHANS_TOTAL = 2,
	QTN_VHT80_CHANS_TOTAL = 4
};

enum {
	QTN_VHT160_SEC80LL_CHAN_INDEX = 0,
	QTN_VHT160_SEC80LU_CHAN_INDEX = 1,
	QTN_VHT160_SEC80UL_CHAN_INDEX = 2,
	QTN_VHT160_SEC80UU_CHAN_INDEX = 3,
	QTN_VHT160_SEC80_CHANS_TOTAL = 4,
	QTN_VHT160_CHANS_TOTAL = 8
};

/*
 * Useful combinations of channel characteristics.
 */
#define	IEEE80211_CHAN_FHSS \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_GFSK)
#define	IEEE80211_CHAN_A \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM)
#define	IEEE80211_CHAN_B \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_CCK)
#define	IEEE80211_CHAN_PUREG \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM)
#define	IEEE80211_CHAN_G \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_DYN)
#define IEEE80211_CHAN_108A \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_TURBO)
#define	IEEE80211_CHAN_108G \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_TURBO)
#define	IEEE80211_CHAN_ST \
	(IEEE80211_CHAN_108A | IEEE80211_CHAN_STURBO)
#define	IEEE80211_CHAN_11N \
	(IEEE80211_CHAN_BW20)
#define	IEEE80211_CHAN_11NG \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20)
#define	IEEE80211_CHAN_11NA \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20)
#define	IEEE80211_CHAN_11NG_HT40U \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_HT40U)
#define	IEEE80211_CHAN_11NG_HT40D \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_HT40D)
#define	IEEE80211_CHAN_11NA_HT40U \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_HT40U)
#define	IEEE80211_CHAN_11NA_HT40D \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_HT40D)
#define	IEEE80211_CHAN_11NG_HT40 \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_BW40)
#define	IEEE80211_CHAN_11NA_HT40 \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_BW40)

#define IEEE80211_CHAN_11AC_NG			(IEEE80211_CHAN_11NG | IEEE80211_CHAN_AC_NG)
#define IEEE80211_CHAN_11AC_NG_HT40		(IEEE80211_CHAN_11NG_HT40 | IEEE80211_CHAN_AC_NG)

#define IEEE80211_CHAN_11AC \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20)
#define IEEE80211_CHAN_11AC_VHT40 \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_BW40)
#define IEEE80211_CHAN_11AC_VHT80 \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_BW40 | IEEE80211_CHAN_BW80)
#define IEEE80211_CHAN_11AC_VHT160 \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_BW40 | IEEE80211_CHAN_BW80 | IEEE80211_CHAN_BW160)

#define IEEE80211_CHAN_11AX \
	(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20)
#define IEEE80211_CHAN_11AX_HE40 \
	(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | IEEE80211_CHAN_BW40)
#define IEEE80211_CHAN_11AX_HE80 \
	(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | IEEE80211_CHAN_BW40 | \
	 IEEE80211_CHAN_BW80)
#define IEEE80211_CHAN_11AX_HE160 \
	(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_BW20 | IEEE80211_CHAN_BW40 | \
	 IEEE80211_CHAN_BW80 | IEEE80211_CHAN_BW160)

#define IEEE80211_CHAN_11AX_2G			(IEEE80211_CHAN_11AC_NG)
#define IEEE80211_CHAN_11AX_2G_HE40		(IEEE80211_CHAN_11AC_NG_HT40)

#define	IEEE80211_CHAN_ALL \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_BW20 | \
	 IEEE80211_CHAN_HT40U | IEEE80211_CHAN_HT40D | IEEE80211_CHAN_BW40| \
	 IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_DYN| \
	 IEEE80211_CHAN_BW80 | \
	 IEEE80211_CHAN_BW160 | IEEE80211_CHAN_AC_NG)

#define	IEEE80211_CHAN_ALLTURBO \
	(IEEE80211_CHAN_ALL | IEEE80211_CHAN_TURBO | IEEE80211_CHAN_STURBO)

#define IEEE80211_CHAN_ANYN \
	(IEEE80211_CHAN_BW20 | IEEE80211_CHAN_HT40U | IEEE80211_CHAN_HT40D | \
		IEEE80211_CHAN_BW40)

#define	IEEE80211_CHAN_HT40_DUAL_EXT \
	(IEEE80211_CHAN_HT40U | IEEE80211_CHAN_HT40D)

#define	IEEE80211_IS_CHAN_CACDONE(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_DFS_CAC_DONE) != 0)
#define	IEEE80211_IS_CHAN_CAC_IN_PROGRESS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_DFS_CAC_IN_PROGRESS) != 0)
#define	IEEE80211_IS_CHAN_RADAR(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_RADAR) != 0)

#define IEEE80211_IS_CHAN_FHSS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_FHSS) == IEEE80211_CHAN_FHSS)
#define	IEEE80211_IS_CHAN_A(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_A) == IEEE80211_CHAN_A)
#define	IEEE80211_IS_CHAN_B(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_B) == IEEE80211_CHAN_B)
#define	IEEE80211_IS_CHAN_PUREG(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_PUREG) == IEEE80211_CHAN_PUREG)
#define	IEEE80211_IS_CHAN_G(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_G) == IEEE80211_CHAN_G)
#define	IEEE80211_IS_CHAN_ANYG(_c) \
	(IEEE80211_IS_CHAN_PUREG(_c) || IEEE80211_IS_CHAN_G(_c))
#define	IEEE80211_IS_CHAN_ST(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_ST) == IEEE80211_CHAN_ST)
#define	IEEE80211_IS_CHAN_108A(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_108A) == IEEE80211_CHAN_108A)
#define	IEEE80211_IS_CHAN_108G(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_108G) == IEEE80211_CHAN_108G)

#define	IEEE80211_IS_CHAN_2GHZ(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_2GHZ) != 0)
#define	IEEE80211_IS_CHAN_5GHZ(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_5GHZ) != 0)
#define IEEE80211_IS_CHAN_6GHZ(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_6GHZ) != 0)
#define	IEEE80211_IS_CHAN_OFDM(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_OFDM) != 0)
#define	IEEE80211_IS_CHAN_CCK(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_CCK) != 0)
#define	IEEE80211_IS_CHAN_GFSK(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_GFSK) != 0)
#define	IEEE80211_IS_CHAN_TURBO(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_TURBO) != 0)
#define	IEEE80211_IS_CHAN_STURBO(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_STURBO) != 0)
#define	IEEE80211_IS_CHAN_DTURBO(_c) \
	(((_c)->ic_flags & \
	(IEEE80211_CHAN_TURBO | IEEE80211_CHAN_STURBO)) == IEEE80211_CHAN_TURBO)
#define	IEEE80211_IS_CHAN_HALF(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_HALF) != 0)
#define	IEEE80211_IS_CHAN_QUARTER(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_QUARTER) != 0)

#define	IEEE80211_IS_CHAN_11N(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11N) == IEEE80211_CHAN_11N)
#define	IEEE80211_IS_CHAN_11NG(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11NG) == IEEE80211_CHAN_11NG)
#define	IEEE80211_IS_CHAN_11NA(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11NA) == IEEE80211_CHAN_11NA)
#define	IEEE80211_IS_CHAN_HT40PLUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_HT40U) == IEEE80211_CHAN_HT40U)
#define	IEEE80211_IS_CHAN_HT40MINUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_HT40D) == IEEE80211_CHAN_HT40D)
#define	IEEE80211_IS_CHAN_HT40(_c) \
	(IEEE80211_IS_CHAN_HT40PLUS((_c)) || IEEE80211_IS_CHAN_HT40MINUS((_c)))
#define IEEE80211_IS_CHAN_11NG_HT40(_c) \
	(IEEE80211_IS_CHAN_11NG((_c)) && IEEE80211_IS_CHAN_HT40((_c)))
#define	IEEE80211_IS_CHAN_11NG_HT40PLUS(_c) \
	(IEEE80211_IS_CHAN_11NG((_c)) && IEEE80211_IS_CHAN_HT40PLUS((_c)))
#define	IEEE80211_IS_CHAN_11NG_HT40MINUS(_c) \
	(IEEE80211_IS_CHAN_11NG((_c)) && IEEE80211_IS_CHAN_HT40MINUS((_c)))
#define	IEEE80211_IS_CHAN_11NA_HT40(_c) \
	(IEEE80211_IS_CHAN_11NA((_c)) && IEEE80211_IS_CHAN_HT40((_c)))
#define	IEEE80211_IS_CHAN_11NA_HT40PLUS(_c) \
	(IEEE80211_IS_CHAN_11NA((_c)) && IEEE80211_IS_CHAN_HT40PLUS((_c)))
#define	IEEE80211_IS_CHAN_11NA_HT40MINUS(_c) \
	(IEEE80211_IS_CHAN_11NA((_c)) && IEEE80211_IS_CHAN_HT40MINUS((_c)))
#define IEEE80211_IS_CHAN_ANYN(_c) 1
	//(((_c)->ic_flags & IEEE80211_CHAN_ANYN))

#define IEEE80211_IS_CHAN_11AC(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11AC) == IEEE80211_CHAN_11AC)
#define	IEEE80211_IS_CHAN_11AC_NG(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11AC_NG) == IEEE80211_CHAN_11AC_NG)
#define	IEEE80211_IS_CHAN_11AC_NG_HT40(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11AC_NG_HT40) == IEEE80211_CHAN_11AC_NG_HT40)
#define	IEEE80211_IS_CHAN_VHT40PLUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_VHT40U) == IEEE80211_CHAN_VHT40U)
#define	IEEE80211_IS_CHAN_VHT40MINUS(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_VHT40D) == IEEE80211_CHAN_VHT40D)
#define IEEE80211_IS_CHAN_VHT40(_c) \
	(IEEE80211_IS_CHAN_VHT40PLUS(_c) || IEEE80211_IS_CHAN_VHT40MINUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT40PLUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT40PLUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT40MINUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT40MINUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT40(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT40(_c))

#define IEEE80211_IS_CHAN_VHT80_EDGEPLUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_VHT80_LL) == IEEE80211_CHAN_VHT80_LL)
#define IEEE80211_IS_CHAN_VHT80_CNTRPLUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_VHT80_LU) == IEEE80211_CHAN_VHT80_LU)
#define IEEE80211_IS_CHAN_VHT80_CNTRMINUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_VHT80_UL) == IEEE80211_CHAN_VHT80_UL)
#define IEEE80211_IS_CHAN_VHT80_EDGEMINUS(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_VHT80_UU) == IEEE80211_CHAN_VHT80_UU)
#define IEEE80211_IS_CHAN_VHT80(_c) \
	(IEEE80211_IS_CHAN_VHT80_EDGEPLUS(_c) || IEEE80211_IS_CHAN_VHT80_EDGEMINUS(_c) || \
	 IEEE80211_IS_CHAN_VHT80_CNTRPLUS(_c) || IEEE80211_IS_CHAN_VHT80_CNTRMINUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT80_EDGEPLUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT80_EDGEPLUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT80_CNTRPLUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT80_CNTRPLUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT80_CNTRMINUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT80_CNTRMINUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT80_EDGEMINUS(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT80_EDGEMINUS(_c))
#define IEEE80211_IS_CHAN_11AC_VHT80(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT80(_c))

#define IEEE80211_IS_CHAN_VHT160(_c) \
	(((_c)->ic_ext_flags & IEEE80211_CHAN_VHT160_MASK) != 0)
#define IEEE80211_IS_CHAN_11AC_VHT160(_c) \
	(IEEE80211_IS_CHAN_11AC(_c) && IEEE80211_IS_CHAN_VHT160(_c))

#define IEEE80211_IS_CHAN_5GHZ_OR_6GHZ(_c) \
	((_c)->ic_flags & (IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_6GHZ))

#define IEEE80211_IS_CHAN_11AX(_c) \
	((((_c)->ic_flags & IEEE80211_CHAN_11AX) == IEEE80211_CHAN_11AX) && \
	 IEEE80211_IS_CHAN_5GHZ_OR_6GHZ(_c))
#define IEEE80211_IS_CHAN_11AX_HE40(_c) \
	((((_c)->ic_flags & IEEE80211_CHAN_11AX_HE40) == IEEE80211_CHAN_11AX_HE40) &&\
	 IEEE80211_IS_CHAN_BW40(_c) && IEEE80211_IS_CHAN_5GHZ_OR_6GHZ(_c))
#define IEEE80211_IS_CHAN_11AX_HE80(_c) \
	((((_c)->ic_flags & IEEE80211_CHAN_11AX_HE80) == IEEE80211_CHAN_11AX_HE80) && \
	 IEEE80211_IS_CHAN_BW80(_c) && IEEE80211_IS_CHAN_5GHZ_OR_6GHZ(_c))
#define IEEE80211_IS_CHAN_11AX_HE160(_c) \
	((((_c)->ic_flags & IEEE80211_CHAN_11AX_HE160) == IEEE80211_CHAN_11AX_HE160) && \
	 IEEE80211_IS_CHAN_BW160(_c) && IEEE80211_IS_CHAN_5GHZ_OR_6GHZ(_c))

#define IEEE80211_IS_CHAN_11AX_2G(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_11AX_2G) == IEEE80211_CHAN_11AX_2G)
#define IEEE80211_IS_CHAN_11AX_2G_HE40(_c) \
	((((_c)->ic_flags & IEEE80211_CHAN_11AX_2G_HE40) == IEEE80211_CHAN_11AX_2G_HE40) \
	 && IEEE80211_IS_CHAN_BW40(_c))

/* Other macros */
#define	IEEE80211_IS_RADAR_DETECT_SUPPORTED(_c) \
	(((_c)->ic_flags_ext2 & IEEE80211_FEXT2_RADAR_IS_SUPPORTED) != 0)
#define	IEEE80211_IS_RADAR_DETECT_CONFIGURED(_c) \
	(((_c)->ic_flags_ext2 & IEEE80211_FEXT2_RADAR_REGION_SET) != 0)

/* ni_chan encoding for FH phy */
#define	IEEE80211_FH_CHANMOD		80
#define	IEEE80211_FH_CHAN(set,pat)	(((set) - 1) * IEEE80211_FH_CHANMOD + (pat))
#define	IEEE80211_FH_CHANSET(chan)	((chan) / IEEE80211_FH_CHANMOD + 1)
#define	IEEE80211_FH_CHANPAT(chan)	((chan) % IEEE80211_FH_CHANMOD)

#define IEEE80211_HTCAP_TXBF_CAP_LEN	4

/* Peer RTS config */
#define IEEE80211_PEER_RTS_OFF		0
#define IEEE80211_PEER_RTS_PMP		1
#define IEEE80211_PEER_RTS_DYN		2
#define IEEE80211_PEER_RTS_MAX		2
#define IEEE80211_PEER_RTS_DEFAULT	IEEE80211_PEER_RTS_DYN

/* Dynamic WMM */
#define IEEE80211_DYN_WMM_CMD_ENABLE			0x00000001
#define IEEE80211_DYN_WMM_CMD_CFG			0x000000F0
#define IEEE80211_DYN_WMM_CMD_CFG_S			4
#define IEEE80211_DYN_WMM_CMD_AC			0xFF000000
#define IEEE80211_DYN_WMM_CMD_AC_S			24
#define IEEE80211_DYN_WMM_CMD_DELTA			0x00FF0000
#define IEEE80211_DYN_WMM_CMD_DELTA_S			16
#define IEEE80211_DYN_WMM_CMD_LIMIT			0x0000FF00
#define IEEE80211_DYN_WMM_CMD_LIMIT_S			8

#define IEEE80211_DYN_WMM_CMD_LOCAL_AIFSN		1
#define IEEE80211_DYN_WMM_CMD_LOCAL_CWMIN		2
#define IEEE80211_DYN_WMM_CMD_LOCAL_CWMAX		3
#define IEEE80211_DYN_WMM_CMD_BSS_AIFSN			4
#define IEEE80211_DYN_WMM_CMD_BSS_CWMIN			5
#define IEEE80211_DYN_WMM_CMD_BSS_CWMAX			6
#define IEEE80211_DYN_WMM_CMD_DUMP			0xF

#define IEEE80211_DYN_WMM_OFF			0
#define IEEE80211_DYN_WMM_ON			1

#define IEEE80211_DYN_WMM_DEFAULT		IEEE80211_DYN_WMM_ON

/*
 * The following values are applied when dynamic WMM is activated. The idea is to apply deltas
 * to the existing values, and also limit the lowest or highest value that is used in each case.
 * This approach was taken so if the default WMM values are used in future, dynamic WMM would just
 * work relative to these new values. Each access category can have different deltas and limits.
 */
#define IEEE80211_DYN_WMM_LOCAL_AIFS_DELTA	{-2, -2, -2, -2}
#define IEEE80211_DYN_WMM_LOCAL_CWMIN_DELTA	{-2, -2, -2, -2}
#define IEEE80211_DYN_WMM_LOCAL_CWMAX_DELTA	{-3, -3, -3, -3}
#define IEEE80211_DYN_WMM_LOCAL_AIFS_MIN	{3, 3, 2, 1}
#define IEEE80211_DYN_WMM_LOCAL_CWMIN_MIN	{2, 4, 2, 2}
#define IEEE80211_DYN_WMM_LOCAL_CWMAX_MIN	{4, 6, 3, 3}
#define IEEE80211_DYN_WMM_BSS_AIFS_DELTA	{2, 2, 2, 2}
#define IEEE80211_DYN_WMM_BSS_CWMIN_DELTA	{2, 2, 2, 2}
#define IEEE80211_DYN_WMM_BSS_CWMAX_DELTA	{3, 3, 3, 3}
#define IEEE80211_DYN_WMM_BSS_AIFS_MAX		{5, 5, 4, 3}
#define IEEE80211_DYN_WMM_BSS_CWMIN_MAX		{4, 6, 4, 3}
#define IEEE80211_DYN_WMM_BSS_CWMAX_MAX		{7, 8, 6, 5}

#define IEEE80211_WMM_MAX_LIMIT			15

/*
 * 802.11 rate set.
 */
#define	IEEE80211_RATE_SIZE	8		/* 802.11 standard */
#define	IEEE80211_RATE_SIZE_11B	4		/* no of 11b rates */
#define	IEEE80211_AG_RATE_MAXSIZE	12		/* Non 11n Rates */

#define IEEE80211_BSSMEMBER_SELECTOR_VHT_PHY	126
#define IEEE80211_BSSMEMBER_SELECTOR_HT_PHY	127
#define IEEE80211_BSSMEMBER_SELECTOR_SAE_H2E_ONLY	123

#define	IEEE80211_RATE_MAXSIZE		30		/* max rates we'll handle */
#define	IEEE80211_HT_RATE_MAXSIZE	77		/* Total number of 802.11n rates */
#define	IEEE80211_HT_RATE_SIZE		128
#define IEEE80211_SANITISE_RATESIZE(_rsz) \
	((_rsz > IEEE80211_RATE_MAXSIZE) ? IEEE80211_RATE_MAXSIZE : _rsz)


/* For legacy hardware - leaving it as is for now */

#define IEEE80211_RATE_MCS	0x8000
#define IEEE80211_RATE_MCS_VAL	0x7FFF

//#define IEEE80211_RATE_IDX_ENTRY(val, idx) (val&(0xff<<(8*idx))>>(idx*8))
#define IEEE80211_RATE_IDX_ENTRY(val, idx) (((val&(0xff<<(idx*8)))>>(idx*8)))

/*
 * 11n A-MPDU & A-MSDU limits . XXX
 */
#define IEEE80211_AMPDU_LIMIT_MIN   (1 * 1024)
#define IEEE80211_AMPDU_LIMIT_MAX   (64 * 1024 - 1)
#define IEEE80211_AMSDU_LIMIT_MAX   4096


/*
 * 11ac MPDU size limit
 */
#define IEEE80211_MPDU_VHT_1K		1500
#define IEEE80211_MPDU_VHT_4K		3895
#define IEEE80211_MPDU_VHT_6K		6064
#define IEEE80211_MPDU_VHT_7_5K		7500
#define IEEE80211_MPDU_VHT_8K		7991
#define IEEE80211_MPDU_VHT_11K		11454
#define IEEE80211_MPDU_ENCAP_OVERHEAD_MAX	64 /* enough for mpdu header 36 + crypto 20 + fcs 4 */

/*
 * 11n and 11ac AMSDU sizes
 */
#define IEEE80211_AMSDU_NONE		0
#define IEEE80211_AMSDU_HT_4K		3839
#define IEEE80211_AMSDU_HT_8K		7935
#define IEEE80211_AMSDU_VHT_1K		(IEEE80211_MPDU_VHT_1K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_VHT_4K		(IEEE80211_MPDU_VHT_4K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_VHT_6K		(IEEE80211_MPDU_VHT_6K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_VHT_7_5K	(IEEE80211_MPDU_VHT_7_5K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_VHT_8K		(IEEE80211_MPDU_VHT_8K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_VHT_11K		(IEEE80211_MPDU_VHT_11K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)

/*
 * P802.11ax_D4.0: 11ax spec - 6G 11ax AMSDU sizes
 */
#define IEEE80211_AMSDU_6G_HE_4K	(IEEE80211_MPDU_VHT_4K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_6G_HE_8K	(IEEE80211_MPDU_VHT_8K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
#define IEEE80211_AMSDU_6G_HE_11K	(IEEE80211_MPDU_VHT_11K - IEEE80211_MPDU_ENCAP_OVERHEAD_MAX)
/*
 * 11n MCS set limits
 */
#define IEEE80211_HT_MAXMCS_SET				10
#define IEEE80211_HT_MAXMCS_SET_SUPPORTED		10
#define IEEE80211_HT_MAXMCS_BASICSET_SUPPORTED		2
#define IEEE80211_MCS_PER_STREAM			8
/*
 * B0-B2: MCS index, B3-B6: MCS set index, B8: BASIC RATE
 */
#define IEEE80211_HT_BASIC_RATE		0x80
#define IEEE80211_HT_MCS_MASK		0x07
#define IEEE80211_HT_MCS_SET_MASK	0x78
#define IEEE80211_HT_RATE_TABLE_IDX_MASK	0x7F

#define IEEE80211_HT_MCS_VALUE(_v) \
		((_v) & IEEE80211_HT_MCS_MASK)

#define IEEE80211_HT_MCS_SET_IDX(_v) \
		(((_v) & IEEE80211_HT_MCS_SET_MASK) >> 3)

#define IEEE80211_HT_IS_BASIC_RATE(_v) \
		(((_v) & IEEE80211_HT_BASIC_RATE) == IEEE80211_HT_BASIC_RATE)

/* index number in the set will be (_i - 1) if (_i != 0) */
#define IEEE80211_HT_MCS_IDX(_m,_i) \
		{ \
			u_int8_t temp = (_m); \
			_i = 0; \
			while (temp) \
			{ \
				temp = temp >> 1; \
				_i++; \
			} \
			if(_i) \
				_i--; \
			else \
				_i = 0xFF; \
		}

/* rate table index = (MCS set index << 3) + MCS index */
#define IEEE80211_HT_RATE_TABLE_IDX(_s,_i) \
		(((_s) << 3) + (_i))

/* Supported rate label */
#define IEEE80211_RATE_11MBPS	22

#if 0
/* integer portion of HT rates */
u_int16_t ht_rate_table_20MHz_400[] = {
							7,
							14,
							21,
							28,
							43,
							57,
							65,
							72,
							14,
							28,
							43,
							57,
							86,
							115,
							130,
							144
						};

u_int16_t ht_rate_table_20MHz_800[] = {
							6,
							13,
							19,
							26,
							39,
							52,
							58,
							65,
							13,
							26,
							39,
							52,
							78,
							104,
							117,
							130
						};

u_int16_t ht_rate_table_40MHz_400[] = {
							15,
							30,
							45,
							60,
							90,
							120,
							135,
							150,
							30,
							60,
							90,
							120,
							180,
							240,
							270,
							300
						};

u_int16_t ht_rate_table_40MHz_800[] = {
							13,
							27,
							40,
							54,
							81,
							108,
							121,
							135,
							27,
							54,
							81,
							108,
							162,
							216,
							243,
							270
						};
#endif

/* MCS mapping for VHT-MCS and HE-MCS */
#define IEEE80211_MCSMAP_NSS_MAX		8
#define IEEE80211_MCSMAP_MCS_MASK		0x3
#define IEEE80211_MCSMAP_MCS_NA			0x3	/* Spatial stream not supported */
#define IEEE80211_MCSMAP_ALL_DISABLE		0xffff
#define IEEE80211_MCSMAP_MANDATORY		0xfffc	/* support MCS 0-7 for 1SS */
#define IEEE80211_MCSMAP_GET_MCS(_mcs_map, _nss)		\
	(((_mcs_map) >> (((_nss) - 1) * 2)) & IEEE80211_MCSMAP_MCS_MASK)
#define IEEE80211_MCSMAP_SET_MCS(_mcs_map, _nss, _mcs) {	\
	(_mcs_map) &= ~(IEEE80211_MCSMAP_MCS_MASK << (((_nss) - 1) * 2)); \
	(_mcs_map) |= (((_mcs) & IEEE80211_MCSMAP_MCS_MASK) << (((_nss) - 1) * 2));}
#define IEEE80211_MCSMAP_NSS_IS_SUPPORTED(_mcs_map, _nss)	\
	(IEEE80211_MCSMAP_GET_MCS(_mcs_map, _nss) != IEEE80211_MCSMAP_MCS_NA)

static __inline__
uint8_t ieee80211_mcs_map_max_nss(uint16_t mcs_map)
{
	int nss;

	for (nss = IEEE80211_MCSMAP_NSS_MAX; nss > 0; nss--) {
		if (IEEE80211_MCSMAP_NSS_IS_SUPPORTED(mcs_map, nss))
			return nss;
	}

	return 1;
}

struct ieee80211_rateset {
	u_int8_t		rs_legacy_nrates; /* Number of legacy rates */
	u_int8_t		rs_nrates; /* Total rates = Legacy + 11n */
	u_int8_t		rs_rates[IEEE80211_RATE_MAXSIZE];
};

struct ieee80211_ht_rateset {
	u_int8_t		rs_legacy_nrates; /* Number of legacy rates */
	u_int8_t		rs_nrates; /* Total rates = Legacy + 11n */
	u_int8_t		rs_rates[IEEE80211_HT_RATE_MAXSIZE];
};

struct ieee80211_roam {
	int8_t			rssi11a;	/* rssi thresh for 11a bss */
	int8_t			rssi11b;	/* for 11g sta in 11b bss */
	int8_t			rssi11bOnly;	/* for 11b sta */
	u_int8_t		pad1;
	u_int8_t		rate11a;	/* rate thresh for 11a bss */
	u_int8_t		rate11b;	/* for 11g sta in 11b bss */
	u_int8_t		rate11bOnly;	/* for 11b sta */
	u_int8_t		pad2;
};
struct ieee80211_htcap {
	u_int16_t		cap;		/* HT capabilities */
	u_int8_t		numtxspstr; /* Number of Tx spatial streams */
	u_int8_t		numrxstbcstr;	/* Number of Rx stbc streams */
	u_int8_t		pwrsave;	/* HT power save mode */
	u_int8_t		mpduspacing;	/* MPDU density */
	u_int16_t		maxmsdu;	/* Max MSDU size */
	u_int16_t		maxampdu;	/* maximum rx A-MPDU factor */
	u_int8_t		mcsset[IEEE80211_HT_MAXMCS_SET_SUPPORTED]; /* HT MCS set */
	u_int16_t		maxdatarate;	/* HT max data rate */
	u_int16_t		extcap;		/* HT extended capability */
	u_int8_t		mcsparams;	/* HT MCS params */
	u_int8_t		hc_txbf[IEEE80211_HTCAP_TXBF_CAP_LEN];	/* HT transmit beamforming capabilities */
} __packed;

struct ieee80211_htinfo {
	u_int8_t		ctrlchannel;	/* control channel */
	u_int8_t		byte1;		/* ht ie byte 1 */
	u_int8_t		byte2;		/* ht ie byte 2 */
	u_int8_t		byte3;		/* ht ie byte 3 */
	u_int8_t		byte4;		/* ht ie byte 4 */
	u_int8_t		byte5;		/* ht ie byte 5 */
	u_int8_t		sigranularity;	/* signal granularity */
	u_int8_t		choffset;	/* external channel offset */
	u_int8_t		opmode;		/* operational mode */
	u_int8_t		basicmcsset[IEEE80211_HT_MAXMCS_BASICSET_SUPPORTED]; /* basic MCS set */
} __packed;

/* VHT capabilities MIB */

/* Maximum MPDU Length B0-1 */
enum ieee80211_vht_maxmpdu {
	IEEE80211_VHTCAP_MAX_MPDU_3895 = 0,
	IEEE80211_VHTCAP_MAX_MPDU_7991,
	IEEE80211_VHTCAP_MAX_MPDU_11454,
	IEEE80211_VHTCAP_MAX_MPDU_RESERVED,
};

/* Supported Channel Width Set B2-3 */
enum ieee80211_vhtcap_chanwidth {
	IEEE80211_VHTCAP_CW_80M_ONLY = 0,
	IEEE80211_VHTCAP_CW_160M,
	IEEE80211_VHTCAP_CW_160_AND_80P80M,
	IEEE80211_VHTCAP_CW_RESERVED,
};

/* RX STBC B8-10 */
enum ieee80211_vht_rxstbc {
	IEEE80211_VHTCAP_RX_STBC_NA = 0,
	IEEE80211_VHTCAP_RX_STBC_UPTO_1,
	IEEE80211_VHTCAP_RX_STBC_UPTO_2,
	IEEE80211_VHTCAP_RX_STBC_UPTO_3,
	IEEE80211_VHTCAP_RX_STBC_UPTO_4,
};

/* RX STS B13-15 */
enum ieee80211_vht_rxsts {
	IEEE80211_VHTCAP_RX_STS_1 = 0,
	IEEE80211_VHTCAP_RX_STS_2,
	IEEE80211_VHTCAP_RX_STS_3,
	IEEE80211_VHTCAP_RX_STS_4,
	IEEE80211_VHTCAP_RX_STS_5,
	IEEE80211_VHTCAP_RX_STS_6,
	IEEE80211_VHTCAP_RX_STS_7,
	IEEE80211_VHTCAP_RX_STS_8,
	IEEE80211_VHTCAP_RX_STS_INVALID = 0xff
};

/* SOUNDING DIM B16-18 */
enum ieee80211_vht_numsnd {
	IEEE80211_VHTCAP_SNDDIM_1 = 0,
	IEEE80211_VHTCAP_SNDDIM_2,
	IEEE80211_VHTCAP_SNDDIM_3,
	IEEE80211_VHTCAP_SNDDIM_4,
	IEEE80211_VHTCAP_SNDDIM_5,
	IEEE80211_VHTCAP_SNDDIM_6,
	IEEE80211_VHTCAP_SNDDIM_7,
	IEEE80211_VHTCAP_SNDDIM_8
};

/* Maximum A-MPDU Length exponent B23-25 */
/* 2^(13 + Max A-MPDU) -1 */
enum ieee80211_vht_maxampduexp {
	IEEE80211_VHTCAP_MAX_A_MPDU_8191,		/* (2^13) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_16383,	/* (2^14) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_32767,	/* (2^15) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_65535,	/* (2^16) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_131071,	/* (2^17) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_262143,	/* (2^18) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_524287,	/* (2^19) -1 */
	IEEE80211_VHTCAP_MAX_A_MPDU_1048575,	/* (2^20) -1 */
};

/* VHT link Adaptation capable B26-27 */
enum ieee80211_vht_lnkadptcap {
	IEEE80211_VHTCAP_LNKADAPTCAP_NO_FEEDBACK,
	IEEE80211_VHTCAP_LNKADAPTCAP_RESERVED,
	IEEE80211_VHTCAP_LNKADAPTCAP_UNSOLICITED,
	IEEE80211_VHTCAP_LNKADAPTCAP_BOTH,
};

/* VHT MCS supported */
enum ieee80211_vht_mcs_supported {
	IEEE80211_VHT_MCS_0_7,
	IEEE80211_VHT_MCS_0_8,
	IEEE80211_VHT_MCS_0_9,
	IEEE80211_VHT_MCS_NA = IEEE80211_MCSMAP_MCS_NA,
};

/* NSS */
enum ieee80211_nss {
	IEEE80211_NSS1 = 1,
	IEEE80211_NSS2,
	IEEE80211_NSS3,
	IEEE80211_NSS4,
	IEEE80211_NSS5,
	IEEE80211_NSS6,
	IEEE80211_NSS7,
	IEEE80211_NSS8,
};

/* VHT NSS */
enum ieee80211_vht_nss {
	IEEE80211_VHT_NSS1 = IEEE80211_NSS1,
	IEEE80211_VHT_NSS2 = IEEE80211_NSS2,
	IEEE80211_VHT_NSS3 = IEEE80211_NSS3,
	IEEE80211_VHT_NSS4 = IEEE80211_NSS4,
	IEEE80211_VHT_NSS5 = IEEE80211_NSS5,
	IEEE80211_VHT_NSS6 = IEEE80211_NSS6,
	IEEE80211_VHT_NSS7 = IEEE80211_NSS7,
	IEEE80211_VHT_NSS8 = IEEE80211_NSS8,
};

#define IEEE80211_VHT_NSS_MAX_MCS_IDX(mcs_map, nss)		\
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_VHT_MCS_0_9) ? 9 : \
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_VHT_MCS_0_8) ? 8 : \
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_VHT_MCS_0_7) ? 7 : 0)))

struct ieee80211_vhtcap {
	u_int32_t			raw_cap_flags;
	u_int32_t			cap_flags;
	u_int32_t			maxmpdu;
	u_int32_t			chanwidth;
	u_int32_t			rxstbc;
	u_int8_t			bfstscap;
	u_int8_t			numsounding;
	u_int32_t			maxampduexp;
	u_int32_t			lnkadptcap;
	u_int16_t			rxmcsmap;
	u_int16_t			rxlgimaxrate;
	u_int16_t			txmcsmap;
	u_int16_t			txlgimaxrate;
	u_int8_t			bfstscap_save;
} __packed;

/* VHT capability macros */
#define VHT_SUPPORTS_MCS0_9_FOR_4SS_BIT	0x0080
#define VHT_SUPPORTS_MCS0_8_FOR_4SS_BIT	0x0040
#define VHT_SUPPORTS_MCS0_9_FOR_3SS_BIT	0x0020
#define VHT_SUPPORTS_MCS0_8_FOR_3SS_BIT	0x0010
#define VHT_SUPPORTS_MCS0_9_FOR_2SS_BIT	0x0008
#define VHT_SUPPORTS_MCS0_8_FOR_2SS_BIT	0x0004
#define VHT_SUPPORTS_MCS0_9_FOR_1SS_BIT	0x0002
#define VHT_SUPPORTS_MCS0_8_FOR_1SS_BIT	0x0001

#define IEEE80211_HE_VHT_HAS_4SS(rxmcsmap) \
	!((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_4SS_BIT) && \
	(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_4SS_BIT))

#define IEEE80211_HE_VHT_HAS_3SS(rxmcsmap) \
	!((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_3SS_BIT) && \
	(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_3SS_BIT))

#define IEEE80211_HE_VHT_HAS_2SS(rxmcsmap) \
	!((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_2SS_BIT) && \
	(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_2SS_BIT))

#define IEEE80211_HE_VHT_HAS_1SS(rxmcsmap) \
	!((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_1SS_BIT) && \
	(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_1SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_8_FOR_4SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_4SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_4SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_9_FOR_4SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_4SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_4SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_8_FOR_3SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_3SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_3SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_9_FOR_3SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_3SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_3SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_8_FOR_2SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_2SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_2SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_9_FOR_2SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_2SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_2SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_8_FOR_1SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_1SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_1SS_BIT))

#define IEEE80211_VHT_SUPPORTS_MCS0_9_FOR_1SS(rxmcsmap)	\
	((rxmcsmap & VHT_SUPPORTS_MCS0_9_FOR_1SS_BIT) && \
	!(rxmcsmap & VHT_SUPPORTS_MCS0_8_FOR_1SS_BIT))

/* VHT Operation element */
/* VHT Operation Information subfields */
enum ieee80211_vhtop_chanwidth {
	IEEE80211_VHTOP_CHAN_WIDTH_20_40MHZ = 0,
	IEEE80211_VHTOP_CHAN_WIDTH_80MHZ = 1,
	IEEE80211_VHTOP_CHAN_WIDTH_160MHZ = 2,	/* deprecated */
	IEEE80211_VHTOP_CHAN_WIDTH_80PLUS80MHZ = 3, /* deprecated */
	IEEE80211_VHTOP_CHAN_WIDTH_80_160_80PLUS80MHZ = IEEE80211_VHTOP_CHAN_WIDTH_80MHZ
};

#define IEEE80211_VHT_MAXMCS_SET_SUPPORTED	10

struct ieee80211_vhtop_info {
	u_int8_t			chanwidth;
	u_int8_t			centerfreq0;
	u_int8_t			centerfreq1;
} __packed;

struct ieee80211_vhtop {
	u_int32_t			chanwidth;
	u_int8_t			centerfreq0;
	u_int8_t			centerfreq1;
	u_int16_t			basicvhtmcsnssset;
} __packed;

/* VHT Operating Mode Notification element */
#define IEEE80211_VHT_OPMODE_NOTIF_DEFAULT	0xFFFF

/* 6 GHz Operation Information element - BSS Channel Width Options */
enum ieee80211_6gop_chanwidth {
	IEEE80211_6GOP_CHANWIDTH_20MHZ = 0,
	IEEE80211_6GOP_CHANWIDTH_40MHZ = 1,
	IEEE80211_6GOP_CHANWIDTH_80MHZ = 2,
	IEEE80211_6GOP_CHANWIDTH_160_80PLUS80MHZ = 3,
};

/* Max number of MU mimo groups */
#define IEEE80211_MU_MIMO_GRP_NUM_MAX	64

/* Max number of MU ofdma groups */
#define IEEE80211_MU_OFDMA_GRP_NUM_MAX	64

/* Max number of nodes in a MU group as per 802.11ac */
#define IEEE80211_MU_GRP_NODES_MAX	4

/* GROUP IDs which are used for SU PPDU as per IEEE P802.11ac/D5.0,
 * chapter 9.17a Group ID and partial AID in VHT PPDUs  */
#define IEEE80211_SU_GROUP_ID_0		0u
#define IEEE80211_SU_GROUP_ID_63	63u

#define IEEE80211_MAC_ADDRESS_GROUP_BIT 0x01

/* VHT MU membership & user position arrays */
struct ieee80211_vht_mu_grp {
#define IEEE80211_VHT_GRP_1ST_BIT_OFFSET	1
#define IEEE80211_VHT_GRP_MAX_BIT_OFFSET	62
#define IEEE80211_VHT_GRP_MEMBERSHIP_ARRAY_SIZE	(IEEE80211_MU_MIMO_GRP_NUM_MAX/(sizeof(u_int8_t)*8))
#define IEEE80211_VHT_USR_POS_ARRAY_SIZE	((IEEE80211_MU_GRP_NODES_MAX >> 1)*	\
							IEEE80211_MU_MIMO_GRP_NUM_MAX/(sizeof(u_int8_t)*8))
	u_int8_t member[IEEE80211_VHT_GRP_MEMBERSHIP_ARRAY_SIZE];
	u_int8_t pos[IEEE80211_VHT_USR_POS_ARRAY_SIZE];
} __packed;

/* Maximum number of nodes currently supported for MU group */
#define QTN_MU_MIN_NODES_PER_GROUP	1
#define QTN_MU_NODES_PER_GROUP		IEEE80211_MU_GRP_NODES_MAX
#define QTN_MU_QMAT_MAX_SLOTS		10
#define QTN_MU_MAX_GROUP_CNT		QTN_MU_QMAT_MAX_SLOTS

struct ieee80211_mu_groups_update {
	uint16_t ncidx[QTN_MU_NODES_PER_GROUP * QTN_MU_MAX_GROUP_CNT];
	struct ieee80211_vht_mu_grp grps[QTN_MU_NODES_PER_GROUP * QTN_MU_MAX_GROUP_CNT];
} __packed;

struct ieee80211_assoc_resp_data {
	u_int16_t reject_code;			/* reject code */
	u_int8_t macfilter_matched_type;
	void *params;
};

#define QTN_MULTI_TID_MAX_DATA_TIDS	2 /* maximum number of data TID in one AMPDU */
#define QTN_MULTI_TID_TX_MAX_SIZE	(QTN_MULTI_TID_MAX_DATA_TIDS + 1) /* data + action frame */

#define IEEE80211_HECAP_PHY_CHAN_WIDTH_2P4G_40MHZ	BIT(0)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_40P80MHZ	BIT(1)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ	BIT(2)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_80P80MHZ	BIT(3)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_2P4G_RU242	BIT(4)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_RU242		BIT(5)
#define IEEE80211_HECAP_PHY_CHAN_WIDTH_MASK		\
	(IEEE80211_HECAP_PHY_CHAN_WIDTH_2P4G_40MHZ |	\
		IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_40P80MHZ | \
		IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ | \
		IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_80P80MHZ)

#define IEEE80211_HE_HAS_2G_40M(hecap)		\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_2P4G_40MHZ))
#define IEEE80211_HE_HAS_5G_40P80M(hecap)		\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_40P80MHZ))
#define IEEE80211_HE_HAS_5G_160M(hecap)		\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ))
#define IEEE80211_HE_HAS_5G_80P80M(hecap)	\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_80P80MHZ))
#define IEEE80211_HE_HAS_2G_RU242(hecap)	\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_2P4G_RU242))
#define IEEE80211_HE_HAS_5G_RU242(hecap)	\
	(!!((hecap)->phycaps.cw_set & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_RU242))

#define IEEE80211_HECAP_PHY_MAX_BW_2G(hecap)	\
	 (IEEE80211_HE_HAS_2G_40M(hecap) ? BW_HT40 : BW_HT20)
#define IEEE80211_HECAP_PHY_MAX_BW_5G(hecap)	\
	((IEEE80211_HE_HAS_5G_80P80M(hecap) || IEEE80211_HE_HAS_5G_160M(hecap)) ? BW_HT160 : \
		(IEEE80211_HE_HAS_5G_40P80M(hecap) ? BW_HT80 : BW_HT20))
#define IEEE80211_HECAP_PHY_MAX_BW(hecap, is_5g)	\
	((is_5g) ? IEEE80211_HECAP_PHY_MAX_BW_5G(hecap) : IEEE80211_HECAP_PHY_MAX_BW_2G(hecap))
#define IEEE80211_HECAP_PHY_MAX_QTN_BW_IDX(hecap, is_5g)	\
	QTN_BW_TO_IDX(IEEE80211_HECAP_PHY_MAX_BW(hecap, is_5g))

enum {
	QTN_MCSSET_11AX_LE80	= 0,
	QTN_MCSSET_11AX_160	= 1,
	QTN_MCSSET_11AX_80P80	= 2,
	QTN_MCSSET_NUM		= 3,
};
#define QTN_MCSSET_IDX_11AX(_bw)	(((_bw) <= QTN_BW_80M) ? QTN_MCSSET_11AX_LE80 : \
			(((_bw) == QTN_BW_160M) ? QTN_MCSSET_11AX_160 : QTN_MCSSET_11AX_80P80))

/* HE NSS */
enum ieee80211_he_nss {
	IEEE80211_HE_NSS1 = IEEE80211_NSS1,
	IEEE80211_HE_NSS2 = IEEE80211_NSS2,
	IEEE80211_HE_NSS3 = IEEE80211_NSS3,
	IEEE80211_HE_NSS4 = IEEE80211_NSS4,
	IEEE80211_HE_NSS5 = IEEE80211_NSS5,
	IEEE80211_HE_NSS6 = IEEE80211_NSS6,
	IEEE80211_HE_NSS7 = IEEE80211_NSS7,
	IEEE80211_HE_NSS8 = IEEE80211_NSS8,
};
#define IEEE80211_HE_NSS_MAX	4

enum ieee80211_he_mcs_supported {
	IEEE80211_HE_MCS_0_7,
	IEEE80211_HE_MCS_0_9,
	IEEE80211_HE_MCS_0_11,
	IEEE80211_HE_MCS_NA = IEEE80211_MCSMAP_MCS_NA,
};

#define IEEE80211_HE_MCS_MAX			11
#define IEEE80211_HE_MAXMCS_SET_SUPPORTED	8

#define IEEE80211_HE_CAP_MAX_RX_NSS(hecap, mcsset_idx)	\
	ieee80211_mcs_map_max_nss((hecap)->he_mcs_map[mcsset_idx].rx_mcs_map)

#define IEEE80211_HE_NSS_MAX_MCS_IDX(mcs_map, nss)		\
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_HE_MCS_0_11) ? 11 : \
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_HE_MCS_0_9) ? 9 : \
	((IEEE80211_MCSMAP_GET_MCS(mcs_map, nss) == IEEE80211_HE_MCS_0_7) ? 7 : 0)))

#define QTN_HE_MAX_MCS_IDX_TO_VALUE(max_he_mcs)		\
	((max_he_mcs == IEEE80211_HE_MCS_0_11) ? 11 : \
	(max_he_mcs == IEEE80211_HE_MCS_0_9) ? 9 : \
	(max_he_mcs == IEEE80211_HE_MCS_0_7) ? 7 : 0)

#define IEEE80211_HE_HAS_LDPC(hecap)		((hecap)->phycaps.ldpc)
#define IEEE80211_HE_HAS_STBC_RX_LE80(hecap)	((hecap)->phycaps.stbc_rx_le80)
/* 11AX TODO: redefine below macro after phycap parsing is complete */
#define IEEE80211_HE_HAS_STBC_RX_GT80(hecap)	(0)

enum ieee80211_he_trig_pad {
	IEEE80211_HE_TRIG_PAD_DUR_NONE = 0,
	IEEE80211_HE_TRIG_PAD_DUR_8USEC,
	IEEE80211_HE_TRIG_PAD_DUR_16USEC,
	IEEE80211_HE_TRIG_PAD_DUR_RESERVE,
};

enum {
	IEEE80211_HE_DEFALT_PE_DUR_US_PER_UNIT = 4,
	IEEE80211_HE_DEFALT_PE_DUR_NONE = 0,
	IEEE80211_HE_DEFALT_PE_DUR_4USEC = 4 / IEEE80211_HE_DEFALT_PE_DUR_US_PER_UNIT,
	IEEE80211_HE_DEFALT_PE_DUR_8USEC = 8 / IEEE80211_HE_DEFALT_PE_DUR_US_PER_UNIT,
	IEEE80211_HE_DEFALT_PE_DUR_12USEC = 12 / IEEE80211_HE_DEFALT_PE_DUR_US_PER_UNIT,
	IEEE80211_HE_DEFALT_PE_DUR_16USEC = 16 / IEEE80211_HE_DEFALT_PE_DUR_US_PER_UNIT,
};

/* Maximum A-MPDU Length Exponent B27-B28*/
enum ieee80211_he_maxampduexp {
	IEEE80211_HECAP_MAX_A_MPDU_0,	/* (2^(13 + VHT exp)) - 1 or (2^(13 + HT exp)) - 1 */
	IEEE80211_HECAP_MAX_A_MPDU_1,	/* (2^21) - 1 or (2^17) - 1 */
	IEEE80211_HECAP_MAX_A_MPDU_2,	/* (2^22) - 1 or (2^18) - 1 */
	IEEE80211_HECAP_MAX_A_MPDU_RESERVE,
};

/* Nominal Packet Padding */
enum ieee80211_he_ppad {
	IEEE80211_HE_PPAD_DUR_0USEC = 0,
	IEEE80211_HE_PPAD_DUR_8USEC,
	IEEE80211_HE_PPAD_DUR_16USEC,
	IEEE80211_HE_PPAD_DUR_RESERVE,
};

/* PPE Thresholds field - RU allocation index */
enum ieee80211_ppet_ru_idx {
	IEEE80211_PPET_RU_242	= 0,
	IEEE80211_PPET_RU_484	= 1,
	IEEE80211_PPET_RU_996	= 2,
	IEEE80211_PPET_RU_2x996	= 3,
};
#define IEEE80211_PPET_RU_IDX_MIN	IEEE80211_PPET_RU_242
#define IEEE80211_PPET_RU_IDX_MAX	IEEE80211_PPET_RU_2x996
#define IEEE80211_PPET_RU_COUNT		(IEEE80211_PPET_RU_IDX_MAX + 1)

/* PPE Thresholds field - Constellation index */
enum ieee80211_ppet_constell_idx {
	IEEE80211_PPET_CONSTELL_BPSK	= 0,	/* MCS0 */
	IEEE80211_PPET_CONSTELL_QPSK	= 1,	/* MCS1, MCS2 */
	IEEE80211_PPET_CONSTELL_16QAM	= 2,	/* MCS3, MCS4 */
	IEEE80211_PPET_CONSTELL_64QAM	= 3,	/* MCS5, MCS6, MCS7 */
	IEEE80211_PPET_CONSTELL_256QAM	= 4,	/* MCS8, MCS9 */
	IEEE80211_PPET_CONSTELL_1024QAM	= 5,	/* MCS10, MCS11 */
	IEEE80211_PPET_CONSTELL_RESERVE	= 6,
	IEEE80211_PPET_CONSTELL_NONE	= 7,
};
#define IEEE80211_PPET16_CONSTELL		0x7
#define IEEE80211_PPET16_CONSTELL_S		0
#define IEEE80211_PPET8_CONSTELL		0x38
#define IEEE80211_PPET8_CONSTELL_S		3
#define IEEE80211_PPET_CONSTELL_PAIR_MASK	0x3f
#define IEEE80211_PPET_CONSTELL_PAIR_SIZE	6
#define IEEE80211_PPET_CONSTELL_PAIR(_ppet16, _ppet8) \
	(SM((_ppet16), IEEE80211_PPET16_CONSTELL) | SM((_ppet8), IEEE80211_PPET8_CONSTELL))
#define IEEE80211_PPET_CONSTELL_PAIR_NONE \
	IEEE80211_PPET_CONSTELL_PAIR(IEEE80211_PPET_CONSTELL_NONE, IEEE80211_PPET_CONSTELL_NONE)

#define IEEE80211_PPET_CONSTELL_PAIR_ALL_0US	IEEE80211_PPET_CONSTELL_PAIR_NONE
#define IEEE80211_PPET_CONSTELL_PAIR_ALL_16US \
	IEEE80211_PPET_CONSTELL_PAIR(IEEE80211_PPET_CONSTELL_BPSK, IEEE80211_PPET_CONSTELL_NONE)

#define IEEE80211_PPET_MCS_NONE			15
#define IEEE80211_PPET16_MCS			0xf
#define IEEE80211_PPET16_MCS_S			0
#define IEEE80211_PPET8_MCS			0xf0
#define IEEE80211_PPET8_MCS_S			4
#define IEEE80211_PPET_MCS_PAIR_MASK		0xff
#define IEEE80211_PPET_MCS_PAIR_SIZE		8
#define IEEE80211_PPET_MCS_PAIR(_ppet16, _ppet8) \
	(SM((_ppet16), IEEE80211_PPET16_MCS) | SM((_ppet8), IEEE80211_PPET8_MCS))
#define IEEE80211_PPET_MCS_PAIR_NONE \
	IEEE80211_PPET_MCS_PAIR(IEEE80211_PPET_MCS_NONE, IEEE80211_PPET_MCS_NONE)

#define IEEE80211_PPET_CONSTELL_TO_MCS_MAP	\
	{ 0, 1, 3, 5, 8, 10, IEEE80211_PPET_MCS_NONE, IEEE80211_PPET_MCS_NONE}

/*
 * |<--NSS-->|<--RU Index Bitmask-->|<--PPE Thresholds Info-->|<--PPE Pad-->|
 * |  3 Bit  |        4 Bit         |       (variable)        |  0 ~ 7 Bit  |
 *
 * The PPE Thresholds Info field contains 6 * (NSS + 1) bits for
 * every bit in the RU Index Bitmask subfield that is nonzero
 */
#define IEEE80211_IE_HECAP_PPET_INFO_SIZE(_n, _m) \
	((((_m) & 0x1) + (((_m) & 0x2) >> 1) + (((_m) & 0x4) >> 2) + (((_m) & 0x8) >> 3)) \
		* IEEE80211_PPET_CONSTELL_PAIR_SIZE * ((_n) + 1))
#define IEEE80211_IE_HECAP_PPET_INFO_OFFSET (3 + 4)
#define IEEE80211_IE_HECAP_PPET_SIZE_OPT(_nss, _ru_idx_mask) \
	(((IEEE80211_IE_HECAP_PPET_INFO_OFFSET + \
		IEEE80211_IE_HECAP_PPET_INFO_SIZE(_nss, _ru_idx_mask)) + 7) >> 3)
#define IEEE80211_IE_HECAP_PPET_SIZE_MAX \
	IEEE80211_IE_HECAP_PPET_SIZE_OPT(IEEE80211_HE_NSS8 - 1, 0xf)

enum ieee80211_vht_he_rxsts {
	IEEE80211_VHT_HE_RX_STS_1 = IEEE80211_VHTCAP_RX_STS_1,
	IEEE80211_VHT_HE_RX_STS_2 = IEEE80211_VHTCAP_RX_STS_2,
	IEEE80211_VHT_HE_RX_STS_3 = IEEE80211_VHTCAP_RX_STS_3,
	IEEE80211_VHT_HE_RX_STS_4 = IEEE80211_VHTCAP_RX_STS_4,
	IEEE80211_VHT_HE_RX_STS_5 = IEEE80211_VHTCAP_RX_STS_5,
	IEEE80211_VHT_HE_RX_STS_6 = IEEE80211_VHTCAP_RX_STS_6,
	IEEE80211_VHT_HE_RX_STS_7 = IEEE80211_VHTCAP_RX_STS_7,
	IEEE80211_VHT_HE_RX_STS_8 = IEEE80211_VHTCAP_RX_STS_8,
	IEEE80211_VHT_HE_RX_STS_INVALID = IEEE80211_VHTCAP_RX_STS_INVALID,
};

static __inline__
uint8_t ieee80211_vht_he_rxsts_to_nss_count(enum ieee80211_vht_he_rxsts rxsts)
{
	return rxsts + 1;
}

/* 802.11ax HE Capabilities */
struct ieee80211_he_mac_cap {
	uint8_t htc_he;
	uint8_t twt_requester;
	uint8_t twt_responder;
	uint8_t frag_support;
	uint8_t trigger_pad_dur;
	uint8_t multi_tid_agg_rx;
	uint8_t multi_tid_agg_tx;
	uint8_t ack_enabled_agg_rx;
	uint8_t ack_enabled_agg_tx;
	uint8_t ack_enabled_amsdu_rx;
	uint8_t trs_support;
	uint8_t bsr_support;
	uint8_t bcast_twt_support;
	uint8_t om_ctl_support;
	uint8_t ofdma_ra_support;
	uint8_t max_ampdu_exp;
	uint8_t amsdu_frag;
	uint8_t flex_twt_support;
	uint8_t qtp_support;
	uint8_t bqr_support;
	uint8_t srp_responder;
	uint8_t sst_support;
};

struct ieee80211_he_phy_cap {
	uint8_t cw_set;
	uint8_t dev_class;
	uint8_t ldpc;
	uint8_t su_ppdu_1ltf_sgi;
	uint8_t ndp_4ltf_lgi;
	uint8_t stbc_tx_le80;
	uint8_t stbc_rx_le80;
	uint8_t rx_mu_ppdu;
	uint8_t su_beamformer;
	uint8_t su_beamformee;
	uint8_t mu_beamformer;
	/* No MU bfmee cap in 11ax standard, just for internal use */
	uint8_t mu_beamformee;
	uint8_t bf_sts_le80;
	uint8_t bf_sts_gt80;
	uint8_t num_snd_dim_le80;
	uint8_t num_snd_dim_gt80;
	uint8_t ng_16_su_fb;
	uint8_t ng_16_mu_fb;
	uint8_t cb_4_2_su_fb;
	uint8_t cb_7_5_mu_fb;
	uint8_t trig_su_bf_fb;
	uint8_t trig_mu_bf_part_fb;
	uint8_t trig_cqi_fb;
	uint8_t partial_bw_ext_range;
	uint8_t partial_bw_dl_mu;
	uint8_t ppe_thres_present;
	uint8_t srp_sr_support;
	uint8_t ppdu_4ltf_sgi;
	uint8_t he_20_in_40mhz_ppdu;
	uint8_t tx_1024qam_lt242;
	uint8_t rx_1024qam_lt242;
	uint8_t	nom_packet_pad;
};

struct ieee80211_he_tx_rx_mcs_nss {
	uint16_t rx_mcs_map;
	uint16_t tx_mcs_map;
};

struct ieee80211_hecap {
	struct ieee80211_he_mac_cap maccaps;
	struct ieee80211_he_phy_cap phycaps;
	struct ieee80211_he_tx_rx_mcs_nss he_mcs_map[QTN_MCSSET_NUM];
	uint8_t he_ppet_map[IEEE80211_HE_NSS_MAX][IEEE80211_PPET_RU_COUNT];
	uint8_t he_ppet_ie[IEEE80211_IE_HECAP_PPET_SIZE_MAX];
};

#define IEEE80211_IE_HEOP_RTS_THRESHOLD_DISABLED	1023

struct ieee80211_heop_params {
	uint8_t def_pe_duration;
	uint8_t twt_reqired;
	uint16_t txop_rts_thres;
	uint8_t vhtop_present;
	uint8_t has_6g_info;
} __packed;


struct ieee80211_heop_bsscolor {
	uint8_t bsscolor;
	uint8_t partial_bsscolor;
	uint8_t bsscolor_disabled;
} __packed;

struct ieee80211_6gop_info {
	uint8_t primary_chan;
	uint8_t	chan_width;
	uint8_t	dup_bcn;
	uint8_t center_freq0;
	uint8_t center_freq1;
	uint8_t	min_rate;
} __packed;

/* 802.11ax HE Operation */
struct ieee80211_heop {
	struct ieee80211_heop_params	params;
	struct ieee80211_heop_bsscolor	bsscolor_info;
	uint16_t			basic_he_mcs_nss;
	uint8_t				max_bssid_indicator;
	struct ieee80211_6gop_info	he_6gop_info;
} __packed;

#define IEEE80211_SR_OBSS_PD_MIN	(-82)
#define IEEE80211_SR_OBSS_PD_MAX	(-62)
#define IEEE80211_SR_OBSS_PD_OFFSET_MAX	(IEEE80211_SR_OBSS_PD_MAX - IEEE80211_SR_OBSS_PD_MIN)
struct ieee80211_sr_params {
	uint8_t srp_disallow;
	uint8_t non_srg_obss_pd_disallow;
	uint8_t non_srg_obss_pd_max_offset;
	uint8_t he_siga_sr_val15_allow;
	uint8_t srg_obss_pd_min_offset;
	uint8_t srg_obss_pd_max_offset;
	union {
		/* forcing to 64-bit alignment */
		uint8_t srg_bsscolor_bitmap[8];
		uint64_t srg_bsscolor;
	};
	union {
		/* forcing to 64-bit alignment */
		uint8_t srg_part_bssid_bitmap[8];
		uint64_t srg_part_bssid;
	};
};

struct ieee80211_action_data {
	u_int8_t cat;				/* category identifier */
	u_int8_t action;			/* action identifier */
	void *params;
};

struct ba_action_req {
	u_int8_t		tid;		/* TID */
	u_int16_t		seq;		/* sequence number of first frame to be block acked */
	u_int8_t		frag;       /* fragment number of first frame to be block acked */
	enum ieee80211_ba_type type;/* block ack type */
	u_int16_t		buff_size;	/* suggested re-order buffer size */
	u_int16_t		timeout;	/* block ack timeout if no transfer */
};

struct ba_action_resp {
	u_int8_t		tid;		/* TID */
	u_int16_t		seq;		/* sequence number of first frame to be block acked */
	u_int8_t		frag;		/* fragment number of first frame to be block acked */
	enum ieee80211_ba_type type;/* block ack type */
	u_int16_t		buff_size;	/* actual re-order buffer size */
	u_int16_t		reason;		/* block ack negotiation status */
	u_int16_t		timeout;	/* negotiated block ack timeout if no transfer */
};

struct ba_action_del {
	u_int8_t		tid;		/* TID */
	u_int16_t		reason;		/* block ack termination reason */
	u_int8_t		initiator;	/* initiator/ recipient of block ack negotiation */
};

struct ht_action_nc_beamforming {
	u_int16_t		num_sts;		/* number of space time streams, Nc */
	u_int16_t		num_txchains;	/* number of transmit chains, Nr */
	u_int8_t		snr[2];			/* SNR for received space time streams */
	u_int16_t		size_matrices;	/* size of beamforming matrices in bytes */
	u_int8_t		*matrices;		/* pointer to beamforming matrices */
	u_int8_t		bw_mode;		/* bwmode = 0 for 20Mhz and 1 for 40 M */

};

struct ht_action_channelswitch {
	u_int8_t		ch_width;		/* switched channel width */
};

struct ht_action_sm_powersave {
	u_int8_t		sm_power_mode;		/* new power mode */
	u_int8_t		sm_power_enabled;	/* power save enabled */
};

struct ht_action_antennasel {
	u_int8_t		antenna_sel;	/* antenna selection: bit number corresponds
									to antenna number */
};

struct ht_action_mimo_ctrl {
	u_int8_t		num_columns;	/* Nc in received beamforming matrices */
	u_int8_t		num_rows;		/* Nr in received beamforming matrices */
	u_int8_t		chan_width;	/* Channel Width 0=20, 1 =40 */
	u_int8_t		num_grouping;	/* Ng in received beamforming matrices */
	u_int8_t		num_coeffsize;	/* Nb in received beamforming matrices */
	u_int8_t		snr[2];			/* SNR as seen by sender of action frame */
	u_int32_t		matrices[1024];		/* pointer to beamforming matrices,
										contents must be copied */
};

typedef void (*ppq_callback_success)(void *ctx);
typedef void (*ppq_callback_fail)(void *ctx, int32_t reason);

struct ieee80211_meas_request_ctrl {
	u_int8_t meas_type;
	unsigned long expire;
	ppq_callback_success fn_success;
	ppq_callback_fail fn_fail;
	union {
		struct _req_basic {
			u_int64_t start_tsf;
			u_int16_t duration_ms;
			u_int8_t channel;
		} basic;
		struct _req_cca {
			u_int64_t start_tsf;
			u_int16_t duration_ms;
			u_int8_t channel;
		} cca;
		struct _req_rpi {
			u_int64_t start_tsf;
			u_int16_t duration_ms;
			u_int8_t channel;
		} rpi;
		struct _req_sta_stats {
			void *sub_item;
			u_int16_t duration_tu;
			u_int8_t group_id;
		} sta_stats;
		struct _req_qtn_cca {
			u_int16_t duration_tu;
		} qtn_cca;
		struct _req_chan_load {
			u_int8_t channel;
			u_int16_t duration_ms;
		} chan_load;
		struct _req_noise_his {
			u_int8_t channel;
			u_int16_t duration_ms;
		} noise_his;
		struct _req_beacon {
			u_int8_t op_class;
			u_int8_t channel;
			u_int8_t duration_ms;
			u_int8_t mode;
			u_int8_t bssid[6];
			/* optional ssid sub elment */
			uint8_t *ssid;
			uint8_t ssid_len;
		} beacon;
		struct _req_frame {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t duration_ms;
			u_int8_t type;
			u_int8_t mac_address[6];
		} frame;
		struct _req_tran_stream_cat {
			u_int16_t duration_ms;
			u_int8_t peer_sta[6];
			u_int8_t tid;
			u_int8_t bin0;
		} tran_stream_cat;
		struct _req_multicast_diag {
			u_int16_t duration_ms;
			u_int8_t group_mac[6];
		} multicast_diag;
	} u;
};

struct ieee80211_meas_report_ctrl {
	u_int8_t meas_type;
	u_int8_t report_mode;
	u_int8_t token;		/* dialog token */
	u_int8_t meas_token;	/* measurement token */
	u_int8_t autonomous;	/* 1: autonomous report */
	union {
		struct _rep_basic {
			u_int8_t channel;
			u_int8_t basic_report;
			u_int16_t duration_tu;
			u_int64_t start_tsf;
		} basic;
		struct _rep_cca {
			u_int8_t channel;
			u_int8_t cca_report;
			u_int16_t duration_tu;
			u_int64_t start_tsf;
		} cca;
		struct _rep_rpi {
			u_int64_t start_tsf;
			u_int16_t duration_tu;
			u_int8_t channel;
			u_int8_t rpi_report[8];
		} rpi;
		struct _rep_sta_stats {
			void *sub_item;
			u_int16_t duration_tu;
			u_int8_t group_id;
		} sta_stats;
		struct _rep_qtn_cca {
			u_int16_t type;
			/* CCA radio measurement report field */
			u_int64_t start_tsf;
			u_int16_t duration_ms;
			u_int8_t channel;
			u_int8_t qtn_cca_report;
			union {
				struct _rep_qtn_cca_info {
					u_int16_t others_time;
					u_int32_t sp_fail;
					u_int32_t lp_fail;
				} qtn_cca_info;
				struct _rep_qtn_fat_info {
					u_int16_t free_airtime;
				} qtn_fat_info;
				struct _rep_qtn_dfs_info {
					u_int16_t dfs_enabled;
					u_int8_t max_txpower;
				} qtn_dfs_info;
			} u;
			u_int8_t *extra_ie;
			u_int16_t extra_ie_len;
		} qtn_cca;
		struct _rep_chan_load {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t duration_tu;
			u_int8_t channel_load;
		} chan_load;
		struct _rep_noise_his {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t duration_tu;
			u_int8_t antenna_id;
			u_int8_t anpi;
			u_int8_t ipi[11];
		} noise_his;
		struct _rep_beacon {
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t duration_tu;
			u_int8_t reported_frame_info;
			u_int8_t rcpi;
			u_int8_t rsni;
			u_int8_t bssid[6];
			u_int8_t antenna_id;
			u_int8_t parent_tsf[4];
		} beacon;
		struct _rep_frame {
			void *sub_item;
			u_int8_t op_class;
			u_int8_t channel;
			u_int16_t duration_tu;
		} frame;
		struct _rep_tran_stream_cat {
			u_int16_t duration_tu;
			u_int8_t peer_sta[6];
			u_int8_t tid;
			u_int8_t reason;
			u_int32_t tran_msdu_cnt;
			u_int32_t msdu_discard_cnt;
			u_int32_t msdu_fail_cnt;
			u_int32_t msdu_mul_retry_cnt;
			u_int32_t qos_lost_cnt;
			u_int32_t avg_queue_delay;
			u_int32_t avg_tran_delay;
			u_int8_t bin0_range;
			u_int32_t bins[6];
		} tran_stream_cat;
		struct _rep_multicast_diag {
			u_int16_t duration_tu;
			u_int8_t group_mac[6];
			u_int8_t reason;
			u_int32_t mul_rec_msdu_cnt;
			u_int16_t first_seq_num;
			u_int16_t last_seq_num;
			u_int16_t mul_rate;
		} multicast_diag;
	} u;
};

struct stastats_subele_vendor {
	u_int32_t flags;
	u_int8_t sequence;
};

struct frame_report_subele_frame_count {
	u_int8_t ta[6];
	u_int8_t bssid[6];
	u_int8_t phy_type;
	u_int8_t avg_rcpi;
	u_int8_t last_rsni;
	u_int8_t last_rcpi;
	u_int8_t antenna_id;
	u_int16_t frame_count;
};

/* TPC actions */
struct ieee80211_action_tpc_request {
	unsigned long expire;
	ppq_callback_success fn_success;
	ppq_callback_fail fn_fail;
};

struct ieee80211_action_tpc_report {
	uint8_t		rx_token;
	int8_t		tx_power;
	int8_t		link_margin;
};

struct ppq_request_param {
	unsigned long expire;
	ppq_callback_success fn_success;
	ppq_callback_fail fn_fail;
};

struct ieee80211_link_measure_request {
	struct ppq_request_param ppq;
};

struct ieee80211_link_measure_report {
	uint8_t token;
	struct ieee80211_action_tpc_report tpc_report;
	uint8_t recv_antenna_id;
	uint8_t tran_antenna_id;
	uint8_t rcpi;
	uint8_t rsni;
};

struct ieee80211_neighbor_report_request {
	struct ppq_request_param ppq;
};

struct ieee80211_neighbor_report_request_item {
	uint8_t bssid[6];
	uint32_t bssid_info;
	uint8_t operating_class;
	uint8_t channel;
	uint8_t phy_type;
};

struct ieee80211_neighbor_report_response {
	uint8_t token;
	uint8_t bss_num;
	struct ieee80211_neighbor_report_request_item *neighbor_report_ptr[32];
};

struct ieee80211_add_del_ts {
	uint8_t token;
	uint8_t status;
	const struct ieee80211_wme_tspec *tspec;
};

#define IEEE80211_MAXIMUM_TIMESTAMP_DIFF_NC_BF	1000000

#define	IEEE80211_TXPOWER_MAX	100	/* .5 dBm units */
#define	IEEE80211_TXPOWER_MIN	0	/* kill radio */

#define	IEEE80211_DTIM_MAX	15	/* max DTIM period */
#define	IEEE80211_DTIM_MIN	1	/* min DTIM period */
#define	IEEE80211_DTIM_DEFAULT	3	/* default DTIM period */
#define	IEEE80211_CSA_MAX	50	/* max CSA count */
#define	IEEE80211_CSA_MIN	1	/* min CSA count */
enum ieee80211_csa_mode {
	IEEE80211_CSA_MODE_DEF = 0,		/* csa default mode, don't send deauth */
	IEEE80211_CSA_MODE_BCAST = 1,		/* Send broadcast deauth */
	IEEE80211_CSA_MODE_PER_CLIENT = 2,	/* Send unicast deauth per client */
	IEEE80211_CSA_MODE_MAX = IEEE80211_CSA_MODE_PER_CLIENT
};
#define IEEE80211_RRM_NR_MIN	0
#define IEEE80211_RRM_NR_MAX	1
#define IEEE80211_INACT_TIMEOUT_MIN 10
#define IEEE80211_INACT_TIMEOUT_MAX 86400

#define	IEEE80211_BINTVAL_MAX	5000	/* max beacon interval (TU's) */

#define	IEEE80211_BINTVAL_WFA_MIN	5

#ifdef PEARL_VELOCE
#define	IEEE80211_BINTVAL_MIN	10	/* min beacon interval (TU's) */
#else
#define	IEEE80211_BINTVAL_MIN	25	/* min beacon interval (TU's) */
#endif
#define	IEEE80211_BINTVAL_DEFAULT 100	/* default beacon interval (TU's) */
#define IEEE80211_BINTVAL_SCALE_DEFAULT	1	/* default beacon scale value */
#define IEEE80211_BINTVAL_SCALE_MBSS	2	/* MBSS beacon scale value */
#define IEEE80211_MBSS_THROLD		2	/* MBSS threshold */

#define IEEE80211_SCAN_TBL_LEN_MAX_DFLT	2000

#define IEEE80211_BWSTR_20	"20"
#define IEEE80211_BWSTR_40	"40"
#define IEEE80211_BWSTR_80	"80"
#define IEEE80211_BWSTR_160	"160"
#define IEEE80211_BWSTR_80P80	"80+80"

/* traffic ID, used to uniquely identify a TS with TA and RA */
#define	IEEE80211_TSPEC_TID		0xF
#define	IEEE80211_TSPEC_TID_S		0
#define	IEEE80211_TSPEC_NODE		0x3FF0
#define	IEEE80211_TSPEC_NODE_S		4
#define	IEEE80211_TSPEC_DIR		0xC000
#define	IEEE80211_TSPEC_DIR_S		14
#define	IEEE80211_TSPEC_PSB		0x10000
#define	IEEE80211_TSPEC_PSB_S		16
#define	IEEE80211_TSPEC_UP		0xE0000
#define	IEEE80211_TSPEC_UP_S		17
#define	IEEE80211_TSPEC_CMD		0x100000
#define	IEEE80211_TSPEC_CMD_S		20

#define IEEE80211_TSPEC_ADDTS		0
#define IEEE80211_TSPEC_DELTS		1
#define IEEE80211_TSPEC_PSB_LEGACY	0
#define IEEE80211_TSPEC_PSB_UAPSD	1
#define IEEE80211_TSPEC_DIR_UPLINK	0
#define IEEE80211_TSPEC_DIR_DOWNLINK	1
#define IEEE80211_TSPEC_DIR_BI		3

#define IEEE80211_NSM_MIN_TIMEOUT	60
#define IEEE80211_NSM_MAX_TIMEOUT	3600

#define IEEE80211_WCAC_TIME_MIN		0
#define IEEE80211_WCAC_TIME_MAX		86400

#define IEEE80211_QTN_FREQ_MULTIPLE	100000
#endif /* _NET80211__IEEE80211_H_ */
