/*
 * Copyright (c) 2013 Quantenna Communications, Inc.
 */

#ifndef _QTN_WMM_AC_H
#define _QTN_WMM_AC_H

#define WMM_AC_BE	0
#define WMM_AC_BK	1
#define WMM_AC_VI	2
#define WMM_AC_VO	3
#define WMM_AC_NUM	4
#define QTN_AC_MGMT	WMM_AC_VO
#define WMM_AC_INVALID	WMM_AC_NUM

#define QTN_AC_ORDER	{ WMM_AC_VO, WMM_AC_VI, WMM_AC_BE, WMM_AC_BK }

#define QTN_TID_BE	0
#define QTN_TID_BK	1
#define QTN_TID_2	2
#define QTN_TID_NS	3	/* nerver suspend TID for OCS/CSA or other frame transmit */
#define QTN_TID_WLAN	4	/* 802.11 encap'ed data from wlan driver */
#define QTN_TID_VI	5
#define QTN_TID_VO	6
#define QTN_TID_MGMT	7
#define QTN_TID_IS_80211(tid)	((tid == QTN_TID_MGMT) || (tid == QTN_TID_WLAN) || (tid == QTN_TID_NS))
#define QTN_80211_TID_NUM	3

#define QTN_TID_ORDER	{ \
	QTN_TID_MGMT,	\
	QTN_TID_WLAN,	\
	QTN_TID_NS,	\
	QTN_TID_VO,	\
	QTN_TID_VI,	\
	QTN_TID_BE,	\
	QTN_TID_BK,	\
	QTN_TID_2	\
}

#define QTN_TID_ORDER_DATA { \
	QTN_TID_VO,	\
	QTN_TID_VI,	\
	QTN_TID_BE,	\
	QTN_TID_BK	\
}
#define QTN_TID_ORDER_DATA_BITMAP	(BIT(QTN_TID_VO) |			\
						BIT(QTN_TID_VI) |		\
						BIT(QTN_TID_BE) |		\
						BIT(QTN_TID_BK))

#define QTN_TID_ORDER_POLL { \
	QTN_TID_VO,	\
	QTN_TID_VI,	\
	QTN_TID_BE,	\
	QTN_TID_BK,	\
	QTN_TID_2,	\
	QTN_TID_NS,	\
	QTN_TID_WLAN,	\
	QTN_TID_MGMT	\
}

#define WMM_AC_TO_TID(_ac) (			\
	(_ac == WMM_AC_VO) ? QTN_TID_VO :	\
	(_ac == WMM_AC_VI) ? QTN_TID_VI :	\
	(_ac == WMM_AC_BK) ? QTN_TID_BK :	\
	QTN_TID_BE)

#define TID_TO_WMM_AC(_tid) (		\
	(_tid == QTN_TID_BK)	? WMM_AC_BK :	\
	(_tid == QTN_TID_2)	? WMM_AC_BK :	\
	(_tid == QTN_TID_VI)	? WMM_AC_VI :	\
	(_tid == QTN_TID_VO)	? WMM_AC_VO :	\
	(_tid == QTN_TID_NS)	? QTN_AC_MGMT :	\
	(_tid == QTN_TID_WLAN)	? QTN_AC_MGMT :	\
	(_tid == QTN_TID_MGMT)	? QTN_AC_MGMT :	\
	WMM_AC_BE)

/* From "Table 10-1—UP-to-AC mappings, IEEE Std 802.11-2016", then map to local TID */
#define TID_TO_UL_TID(_tid) (		\
	(_tid == 0)	?  QTN_TID_BE :	\
	(_tid == 1)	?  QTN_TID_BK :	\
	(_tid == 2)	?  QTN_TID_BK :	\
	(_tid == 3)	?  QTN_TID_BE :	\
	(_tid == 4)	?  QTN_TID_VI :	\
	(_tid == 5)	?  QTN_TID_VI :	\
	(_tid == 6)	?  QTN_TID_VO :	\
	(_tid == 7)	?  QTN_TID_VO :	\
	QTN_TID_BE)
#define TID_TO_UL_TID_NUM		2
#define TID_TO_UL_TID_IDX(_tid)		\
	(((tid == 0) || (tid == 1) || (tid == 5) || (tid == 6)) ? 0 : 1)

#define QTN_TID_COLLAPSE(_tid)	WMM_AC_TO_TID(TID_TO_WMM_AC(_tid))

#define AC_TO_QTN_QNUM(_ac)		\
	(((_ac) == WME_AC_BE) ? 1 :	\
	 ((_ac) == WME_AC_BK) ? 0 :	\
	  (_ac))

#define QTN_TID_MAP_UNUSED(_tid) (_tid)

#endif	/* _QTN_WMM_AC_H */
