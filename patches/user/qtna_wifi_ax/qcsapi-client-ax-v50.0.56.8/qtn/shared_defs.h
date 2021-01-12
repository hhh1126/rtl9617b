/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2010 Quantenna Communications Inc                   **
**                            All Rights Reserved                            **
**                                                                           **
**  Author      : Quantenna Communications Inc                               **
**  File        : shared_params.h                                            **
**  Description :                                                            **
**                                                                           **
*******************************************************************************
**                                                                           **
**  Redistribution and use in source and binary forms, with or without       **
**  modification, are permitted provided that the following conditions       **
**  are met:                                                                 **
**  1. Redistributions of source code must retain the above copyright        **
**     notice, this list of conditions and the following disclaimer.         **
**  2. Redistributions in binary form must reproduce the above copyright     **
**     notice, this list of conditions and the following disclaimer in the   **
**     documentation and/or other materials provided with the distribution.  **
**  3. The name of the author may not be used to endorse or promote products **
**     derived from this software without specific prior written permission. **
**                                                                           **
**  Alternatively, this software may be distributed under the terms of the   **
**  GNU General Public License ("GPL") version 2, or (at your option) any    **
**  later version as published by the Free Software Foundation.              **
**                                                                           **
**  In the case this software is distributed under the GPL license,          **
**  you should have received a copy of the GNU General Public License        **
**  along with this software; if not, write to the Free Software             **
**  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA  **
**                                                                           **
**  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR       **
**  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES**
**  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  **
**  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,         **
**  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT **
**  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,**
**  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    **
**  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      **
**  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF **
**  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.        **
**                                                                           **
*******************************************************************************
EH1*/

#ifndef _SHARED_DEFS_H_
#define _SHARED_DEFS_H_

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L)
#include <assert.h>
#endif

#include "../common/base/qtn_base_shared_defs.h"
#include "../common/qtn_config.h"

#define QTN_SWITCH_CHANNEL_TIME_AVG_2G	1500	/* microseconds */
#define QTN_SWITCH_CHANNEL_TIME_AVG	3500	/* microseconds */
#define QTN_SWITCH_CHANNEL_BGSCAN_TIME_AVG	4700	/* microseconds */

#define IEEE80211_MAX_NAV	32767

/* SCS (ACI/CCI Detection and Mitigation) APIs */
enum qtn_vap_scs_cmds {
	IEEE80211_SCS_SET_ENABLE = 1,
	IEEE80211_SCS_SET_DEBUG_ENABLE,
	IEEE80211_SCS_SET_SAMPLE_ENABLE,
	IEEE80211_SCS_SET_SAMPLE_DWELL_TIME,
	IEEE80211_SCS_SET_SAMPLE_INTERVAL,
	IEEE80211_SCS_SET_THRSHLD_SMPL_PKTNUM,
	IEEE80211_SCS_SET_THRSHLD_PRI_CCA,
	IEEE80211_SCS_SET_THRSHLD_SEC_CCA,
	IEEE80211_SCS_SET_THRSHLD_SMPL_AIRTIME,
	IEEE80211_SCS_SET_WF_CCA,
	IEEE80211_SCS_SET_WF_RSSI,
	IEEE80211_SCS_SET_WF_CRC_ERR,
	IEEE80211_SCS_SET_WF_LPRE,
	IEEE80211_SCS_SET_WF_SPRE,
	IEEE80211_SCS_SET_WF_RETRIES,
	IEEE80211_SCS_SET_WF_DFS,
	IEEE80211_SCS_SET_WF_MAX_TX_PWR,
	IEEE80211_SCS_SET_REPORT_ONLY,
	IEEE80211_SCS_SET_CCA_INTF_RATIO,
	IEEE80211_SCS_SET_CCA_IDLE_THRSHLD,
	IEEE80211_SCS_SET_CCA_INTF_LO_THR,
	IEEE80211_SCS_SET_CCA_INTF_HI_THR,
	IEEE80211_SCS_SET_CCA_SMPL_DUR,
	IEEE80211_SCS_GET_REPORT,
	IEEE80211_SCS_GET_INTERNAL_STATS,
	IEEE80211_SCS_SET_CCA_INTF_SMTH_FCTR,
	IEEE80211_SCS_RESET_RANKING_TABLE,
	IEEE80211_SCS_SET_CHAN_MTRC_MRGN,
	IEEE80211_SCS_SET_RSSI_SMTH_FCTR,
	IEEE80211_SCS_SET_ATTEN_ADJUST,
	IEEE80211_SCS_SET_ATTEN_SWITCH_ENABLE,
	IEEE80211_SCS_SET_THRSHLD_ATTEN_INC,
	IEEE80211_SCS_SET_THRSHLD_DFS_REENTRY,
	IEEE80211_SCS_SET_THRSHLD_DFS_REENTRY_MINRATE,
	IEEE80211_SCS_SET_PMBL_ERR_SMTH_FCTR,
	IEEE80211_SCS_SET_PMBL_ERR_RANGE,
	IEEE80211_SCS_SET_PMBL_ERR_MAPPED_INTF_RANGE,
	IEEE80211_SCS_SET_THRSHLD_LOAD,
	IEEE80211_SCS_SET_PMBL_ERR_WF,
	IEEE80211_SCS_SET_THRSHLD_AGING_NOR,
	IEEE80211_SCS_SET_THRSHLD_AGING_DFSREENT,
	IEEE80211_SCS_SET_THRSHLD_DFS_REENTRY_INTF,
	IEEE80211_SCS_SET_PMP_RPT_CCA_SMTH_FCTR,
	IEEE80211_SCS_SET_PMP_RX_TIME_SMTH_FCTR,
	IEEE80211_SCS_SET_PMP_TX_TIME_SMTH_FCTR,
	IEEE80211_SCS_SET_PMP_STATS_STABLE_PERCENT,
	IEEE80211_SCS_SET_PMP_STATS_STABLE_RANGE,
	IEEE80211_SCS_SET_PMP_STATS_CLEAR_INTERVAL,
	IEEE80211_SCS_SET_PMP_TXTIME_COMPENSATION,
	IEEE80211_SCS_SET_PMP_RXTIME_COMPENSATION,
	IEEE80211_SCS_SET_PMP_TDLSTIME_COMPENSATION,
	IEEE80211_SCS_SET_SWITCH_CHANNEL_MANUALLY,
	IEEE80211_SCS_SET_AS_RX_TIME_SMTH_FCTR,
	IEEE80211_SCS_SET_AS_TX_TIME_SMTH_FCTR,
	IEEE80211_SCS_SET_STATS_START,
	IEEE80211_SCS_SET_CCA_IDLE_SMTH_FCTR,
	IEEE80211_SCS_SET_PMBL_ERR_THRSHLD,
	IEEE80211_SCS_SET_CCA_INTF_DFS_MARGIN,
	IEEE80211_SCS_SET_HW_NOISE_SMTH_FCTR,
	IEEE80211_SCS_SET_LEAVE_DFS_CHAN_MTRC_MRGN,
	IEEE80211_SCS_SET_SAMPLE_TYPE,
	IEEE80211_SCS_SET_BURST_ENABLE,
	IEEE80211_SCS_SET_BURST_WINDOW,
	IEEE80211_SCS_SET_BURST_THRESH,
	IEEE80211_SCS_SET_BURST_PAUSE,
	IEEE80211_SCS_SET_BURST_SWITCH,
	IEEE80211_SCS_SET_NAC_MONITOR_MODE,
	IEEE80211_SCS_SET_BAND_MRGN_CHECK,
	IEEE80211_SCS_SET_OUT_OF_BAND_MTRC_MRGN,
	IEEE80211_SCS_SET_VERSION,
	IEEE80211_SCS_API_MAX,
};

#define IEEE80211_SCS_STATE_INIT			0
#define IEEE80211_SCS_STATE_RESET			1
#define IEEE80211_SCS_STATE_CHANNEL_SWITCHING		2
#define IEEE80211_SCS_STATE_MEASUREMENT_CHANGE_CLEAN	3    /* param change */
#define IEEE80211_SCS_STATE_PERIOD_CLEAN		4
#define IEEE80211_SCS_STANDARD				0
#define IEEE80211_SCS_ADVANCED				1

#define IEEE80211_SCS_COMPARE_INIT_TIMER	5
#define IEEE80211_SCS_COMPARE_TIMER_INTVAL	2
#define IEEE80211_CCA_SAMPLE_DUR		IEEE80211_SCS_COMPARE_TIMER_INTVAL /* seconds */
#define IEEE80211_SCS_CHAN_CURRENT		0
#define IEEE80211_SCS_CHAN_ALL			0xFF
#define IEEE80211_SCS_THRSHLD_MAX		100	/* metric */
#define IEEE80211_SCS_THRSHLD_MIN		1	/* metric */
#define IEEE80211_SCS_SMPL_DWELL_TIME_MAX	24	/* milliseconds, limited by max NAV reservation */
#define IEEE80211_SCS_SMPL_DWELL_TIME_MIN	5	/* milliseconds */
#define IEEE80211_SCS_SMPL_DWELL_TIME_DEFAULT	11	/* milliseconds */
#define IEEE80211_SCS_SMPL_INTV_MAX		3600	/* seconds */
#define IEEE80211_SCS_SMPL_INTV_MIN		1	/* seconds */
#define IEEE80211_SCS_SMPL_INTV_DEFAULT		3	/* seconds */
#define IEEE80211_SCS_THRSHLD_SMPL_PKTNUM_DEFAULT	16	/* packet number */
#define IEEE80211_SCS_THRSHLD_SMPL_PKTNUM_MAX	1000	/* packet number */
#define IEEE80211_SCS_THRSHLD_SMPL_PKTNUM_MIN	1	/* packet number */
#define IEEE80211_SCS_THRSHLD_SMPL_AIRTIME_DEFAULT	200	/* ms */
#define IEEE80211_SCS_THRSHLD_SMPL_AIRTIME_MAX	1000	/* ms */
#define IEEE80211_SCS_THRSHLD_SMPL_AIRTIME_MIN	1	/* ms */
#define IEEE80211_SCS_THRSHLD_PMBL_ERR_MAX	10000	/* count */
#define IEEE80211_SCS_THRSHLD_PMBL_ERR_MIN	1	/* count */

/*
 * Packet rate threshold is determined by how many packets we can hold in buffer without drop
 * during off-channel period. It is limited by:
 * - sw queue length of each node/tid
 * - global resource shared by all node/tid, such as tqew descriptors and msdu headers.
 * Current value doesn't apply to the scenario when tqew descriptors are already used up by large
 * number of stations.
 */
#define IEEE80211_SCS_THRSHLD_SMPL_TX_PKTRATE	(1024 - 128)	/* margin = 128 + hw ring size */
#define IEEE80211_SCS_THRSHLD_SMPL_RX_PKTRATE	IEEE80211_SCS_THRSHLD_SMPL_TX_PKTRATE /* assume qtn peer */
#define IEEE80211_SCS_THRSHLD_ATTEN_INC_DFT	5	/* db */
#define IEEE80211_SCS_THRSHLD_ATTEN_INC_MIN     0       /* db */
#define IEEE80211_SCS_THRSHLD_ATTEN_INC_MAX     20      /* db */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_DFT	60	/* seconds */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MIN   0       /* seconds */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MAX   0xffff  /* seconds */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_INTF_MIN   0
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_INTF_MAX   100
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_INTF_DFT   40
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MINRATE_UNIT	100	/* kbps */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MINRATE_DFT	5	/* unit: 100kbps */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MINRATE_MIN   0       /* unit: 100kbps */
#define IEEE80211_SCS_THRSHLD_DFS_REENTRY_MINRATE_MAX   0xffff  /* unit: 100kbps */
#define IEEE80211_SCS_THRSHLD_AGING_MIN         0
#define IEEE80211_SCS_THRSHLD_AGING_MAX         0xFFFF
#define IEEE80211_SCS_THRSHLD_AGING_NOR_DFT     (60 * 6)
#define IEEE80211_SCS_THRSHLD_AGING_DFSREENT_DFT  5
#define IEEE80211_SCS_CCA_DUR_MAX		10	/* seconds */
#define IEEE80211_SCS_CCA_DUR_MIN		2	/* seconds */
#define IEEE80211_SCS_CCA_INTF_SCALE		1000	/* milliseconds */
#define IEEE80211_SCS_SENDING_QOSNULL_TIME_AVG	1000	/* microseconds */
#define IEEE80211_SCS_SMPL_TIME_MARGIN		2000	/* microseconds */
#define IEEE80211_SCS_SMPL_TIME_OFFSET_SEND_QOSNULL	2000	/* microseconds */
#define IEEE80211_SCS_SMPL_TIME_SENDING_ALL_BEACONS	25000	/* microseconds, the time duration for transmitting all beacons */
#define IEEE80211_CCA_INTF_SMTH_FCTR_NOXP_DFT	75
#define IEEE80211_CCA_INTF_SMTH_FCTR_XPED_DFT	90
#define IEEE80211_CCA_INTF_SMTH_FCTR_MIN	0
#define IEEE80211_CCA_INTF_SMTH_FCTR_MAX	100
#define IEEE80211_SCS_CHAN_MARGIN_MAX		100
#define IEEE80211_SCS_CHAN_MTRC_MRGN_MAX	IEEE80211_SCS_CHAN_MARGIN_MAX
#define IEEE80211_SCS_CHAN_MTRC_MRGN_DFT	15
#define IEEE80211_SCS_LEAVE_DFS_CHAN_MTRC_MRGN_DFT	25
#define IEEE80211_SCS_OUT_OF_BAND_MTRC_MRGN_MAX	100
#define IEEE80211_SCS_OUT_OF_BAND_MRGN_DFT	50
#define IEEE80211_SCS_RSSI_SMTH_FCTR_UP_DFT	75
#define IEEE80211_SCS_RSSI_SMTH_FCTR_DOWN_DFT	25
#define IEEE80211_SCS_RSSI_SMTH_FCTR_MAX	100
#define IEEE80211_SCS_ATTEN_ADJUST_MIN		-20
#define IEEE80211_SCS_ATTEN_ADJUST_MAX		20
#define IEEE80211_SCS_ATTEN_ADJUST_DFT		5
#define IEEE80211_SCS_BRCM_RXGLITCH_THRSHLD_SCALE_DFT    40
#define IEEE80211_SCS_PMBL_ERR_SMTH_FCTR_MIN    0
#define IEEE80211_SCS_PMBL_ERR_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_PMBL_ERR_SMTH_FCTR_DFT    66
#define IEEE80211_SCS_CCA_IDLE_SMTH_FCTR_MIN    0
#define IEEE80211_SCS_CCA_IDLE_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_CCA_IDLE_SMTH_FCTR_DFT    50
#define IEEE80211_SCS_PMP_RPT_CCA_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_PMP_RPT_CCA_SMTH_FCTR_DFT    66
#define IEEE80211_SCS_PMP_RX_TIME_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_PMP_RX_TIME_SMTH_FCTR_DFT    66
#define IEEE80211_SCS_PMP_TX_TIME_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_PMP_TX_TIME_SMTH_FCTR_DFT    66
#define IEEE80211_SCS_PMP_STATS_STABLE_PERCENT_MAX  100
#define IEEE80211_SCS_PMP_STATS_STABLE_PERCENT_DFT  30
#define IEEE80211_SCS_PMP_STATS_STABLE_RANGE_MAX    1000
#define IEEE80211_SCS_PMP_STATS_STABLE_RANGE_DFT    50
#define IEEE80211_SCS_PMP_STATS_CLEAR_INTERVAL_MAX  3600 /* seconds */
#define IEEE80211_SCS_PMP_STATS_CLEAR_INTERVAL_DFT  60 /* seconds */
#define IEEE80211_SCS_AS_RX_TIME_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_AS_RX_TIME_SMTH_FCTR_DFT    50
#define IEEE80211_SCS_AS_TX_TIME_SMTH_FCTR_MAX    100
#define IEEE80211_SCS_AS_TX_TIME_SMTH_FCTR_DFT    50
#define IEEE80211_SCS_HW_NOISE_SMTH_FCTR_DFT	50
#define IEEE80211_SCS_HW_NOISE_SMTH_FCTR_MIN	0
#define IEEE80211_SCS_HW_NOISE_SMTH_FCTR_MAX	100

#define IEEE80211_SCS_SMTH_RBS_TIME			80

#define IEEE80211_SCS_PMBL_ERR_RANGE_MIN        1000
#define IEEE80211_SCS_PMBL_ERR_RANGE_MAX        0xFFFF
#define IEEE80211_SCS_PMBL_ERR_RANGE_DFT        5000
#define IEEE80211_SCS_PMBL_ERR_MAPPED_INTF_RANGE_MIN  0
#define IEEE80211_SCS_PMBL_ERR_MAPPED_INTF_RANGE_MAX  100
#define IEEE80211_SCS_PMBL_ERR_MAPPED_INTF_RANGE_DFT  40
#define IEEE80211_SCS_PMBL_ERR_WF_MIN           0
#define IEEE80211_SCS_PMBL_ERR_WF_MAX           100
#define IEEE80211_SCS_PMBL_SHORT_WF_DFT         0
#define IEEE80211_SCS_PMBL_LONG_WF_DFT          100
#define IEEE80211_SCS_THRSHLD_LOADED_MIN        0
#define IEEE80211_SCS_THRSHLD_LOADED_MAX        1000
#define IEEE80211_SCS_THRSHLD_LOADED_DFT        20

#define IEEE80211_SCS_CHAN_POWER_CUTPOINT       15
#define IEEE80211_SCS_NORMALIZE(_v, _duration)       (((_v) < (0xFFFFFFFF / IEEE80211_SCS_CCA_INTF_SCALE)) ?  \
							((_v) * IEEE80211_SCS_CCA_INTF_SCALE / (_duration)) : \
							((_v) / (_duration) * IEEE80211_SCS_CCA_INTF_SCALE))

#define IEEE80211_SCS_SMOOTH(_old, _new, _fctr)	(((_old) * (_fctr) + (_new) * (100 - (_fctr))) / 100)

#define IEEE80211_SCS_MARGIN_TO_METRIC(margin)	((int32_t)IEEE80211_SCS_NORMALIZE(margin, IEEE80211_SCS_CHAN_MARGIN_MAX))
#define IEEE80211_SCS_OFFCHAN_WHOLE_DUR(_dwell_us, cc_time)	((_dwell_us) +					\
								(2 * (cc_time)) +				\
								IEEE80211_SCS_SENDING_QOSNULL_TIME_AVG +	\
								IEEE80211_SCS_SMPL_TIME_MARGIN)

#define IEEE80211_SCS_VALUE_S			0
#define IEEE80211_SCS_VALUE_M			0xffff
#define IEEE80211_SCS_WF_VALUE_M		0xff
#define IEEE80211_SCS_COMMAND_S			16
#define IEEE80211_SCS_COMMAND_M			0xffff

#define IEEE80211_SCS_NA_CC			0x0
#define IEEE80211_SCS_STA_CCA_REQ_CC		0x1
#define IEEE80211_SCS_SELF_CCA_CC               0x2
#define IEEE80211_SCS_ATTEN_INC_CC		0x4
#define IEEE80211_SCS_BRCM_STA_TRIGGER_CC	0x8
#define IEEE80211_SCS_CCA_INTF_CC               (IEEE80211_SCS_STA_CCA_REQ_CC | IEEE80211_SCS_SELF_CCA_CC)
#define IEEE80211_SCS_INTF_CC                   (IEEE80211_SCS_CCA_INTF_CC | IEEE80211_SCS_BRCM_STA_TRIGGER_CC)

#define IEEE80211_SCS_BURST_ENABLE_MIN		0
#define IEEE80211_SCS_BURST_ENABLE_MAX		1
#define	IEEE80211_SCS_BURST_ENABLE_DEFAULT	0
#define IEEE80211_SCS_BURST_WINDOW_MIN		(1)	/* minutes */
#define IEEE80211_SCS_BURST_WINDOW_MAX		(300)	/* minutes */
#define	IEEE80211_SCS_BURST_WINDOW_DEFAULT	(180)	/* minutes */
#define IEEE80211_SCS_BURST_THRESH_MIN		2
#define IEEE80211_SCS_BURST_THRESH_MAX		100
#define	IEEE80211_SCS_BURST_THRESH_DEFAULT	3
#define IEEE80211_SCS_BURST_PAUSE_MIN		(30)	/* minutes */
#define IEEE80211_SCS_BURST_PAUSE_MAX		(600)	/* minutes */
#define	IEEE80211_SCS_BURST_PAUSE_DEFAULT	(60)	/* minutes */
#define IEEE80211_SCS_BURST_SWITCH_MIN		0
#define IEEE80211_SCS_BURST_SWITCH_MAX		1
#define	IEEE80211_SCS_BURST_SWITCH_DEFAULT	0
#define IEEE80211_SCS_UNSTABLE_INTF			0x00000001
#define IEEE80211_SCS_UNSTABLE_INTF_OUTDATED		0x00000002
#define IEEE80211_SCS_UNSTABLE_INTF_INVALID		0x00000004
#define IEEE80211_SCS_UNSTABLE_IDLE			0x00000008
#define IEEE80211_SCS_UNSTABLE_IDLE_OUTDATED		0x00000010
#define IEEE80211_SCS_UNSTABLE_OTHERSTIME		0x00000020
#define IEEE80211_SCS_UNSTABLE_OTHERSTIME_OUTDATED	0x00000040
#define IEEE80211_SCS_UNSTABLE_TDLS_TX			0x00000080
#define IEEE80211_SCS_UNSTABLE_TDLS_RX			0x00000100
#define IEEE80211_SCS_UNSTABLE_TDLS_OUTDATED		0x00000200

#define	IEEE80211_REMAIN_CHAN_MIN_RSV_PERD	2

enum ieee80211_scs_update_mode {
	IEEE80211_SCS_OFFCHAN,		/* off-channel, use smoothing and omit current channel */
	IEEE80211_SCS_COCHAN,		/* co-channel mode */
	IEEE80211_SCS_INIT_SCAN,	/* like off-channel but include current channel */
};

#define SCSLOG_CRIT                             0
#define SCSLOG_NOTICE                           1
#define SCSLOG_INFO                             2
#define SCSLOG_VERBOSE                          3
#define SCSLOG_EXTRA_VERBOSE                    4
#define SCSLOG_LEVEL_MAX                        4
#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
#define _SCSDBG(_prefix, _level, _fmt, ...)            do {               \
		if (ic->ic_scs.scs_debug_enable >= (_level)) {  \
			DBGFN(_prefix _fmt, ##__VA_ARGS__);     \
		}                                               \
	} while (0)
#define SCSDBG(_level, _fmt, ...)		_SCSDBG("SCS: ", _level, _fmt, ##__VA_ARGS__)
#define SCSDBG_NO_PREFIX(_level, _fmt, ...)	_SCSDBG("", _level, _fmt, ##__VA_ARGS__)
#endif

#define RADAR_BLOCK_COUNT	2

/* OCAC (Off-channel CAC) APIs */
enum qtn_ocac_cmds {
	IEEE80211_OCAC_SET_ENABLE = 1,
	IEEE80211_OCAC_SET_DISABLE,
	IEEE80211_OCAC_SET_DEBUG_LEVEL,
	IEEE80211_OCAC_SET_DWELL_TIME,
	IEEE80211_OCAC_SET_DURATION,
	IEEE80211_OCAC_SET_THRESHOLD_FAT,
	IEEE80211_OCAC_SET_DUMP_COUNTS,
	IEEE80211_OCAC_SET_CAC_TIME,
	IEEE80211_OCAC_SET_THRESHOLD_TRAFFIC,
	IEEE80211_OCAC_SET_TIMER_INTERVAL,
	IEEE80211_OCAC_SET_DUMP_TSFLOG,
	IEEE80211_OCAC_SET_DUMP_CFG,
	IEEE80211_OCAC_SET_TRAFFIC_CONTROL,
	IEEE80211_OCAC_SET_THRESHOLD_CCA_INTF,
	IEEE80211_OCAC_SET_REPORT_ONLY,
	IEEE80211_OCAC_SET_DUMP_CCA_COUNTS,
	IEEE80211_OCAC_SET_OFFSET_TXHALT,
	IEEE80211_OCAC_SET_OFFSET_OFFCHAN,
	IEEE80211_OCAC_SET_THRESHOLD_FAT_DEC,
	IEEE80211_OCAC_SET_TIMER_EXPIRE_INIT,
	IEEE80211_OCAC_SET_SECURE_DWELL_TIME,
	IEEE80211_OCAC_SET_BEACON_INTERVAL,
	IEEE80211_OCAC_SET_WEATHER_DURATION,
	IEEE80211_OCAC_SET_WEATHER_CAC_TIME,
	IEEE80211_OCAC_SET_WEATHER_DWELL_TIME,
	IEEE80211_OCAC_SET_ENABLE_AUTO_DFS,
	IEEE80211_OCAC_SET_SUSPEND_TIME,
	IEEE80211_OCAC_SET_THRESHOLD_VIDEO_FRAMES,
	IEEE80211_OCAC_SET_MAX
};

enum qtn_ocac_get_cmds {
	IEEE80211_OCAC_GET_STATUS = 1,
	IEEE80211_OCAC_GET_AVAILABILITY,
};

#define IEEE80211_OCAC_CLEAN_STATS_STOP		0
#define IEEE80211_OCAC_CLEAN_STATS_START	1
#define IEEE80211_OCAC_CLEAN_STATS_RESET	2


#define IEEE80211_OCAC_DWELL_TIME_MIN		5	/* milliseconds */
#define IEEE80211_OCAC_DWELL_TIME_MAX		200	/* milliseconds */
#define IEEE80211_OCAC_DWELL_TIME_DEFAULT	40	/* milliseconds */
#define IEEE80211_OCAC_WEA_DWELL_TIME_DEFAULT	46	/* milliseconds */

#define IEEE80211_OCAC_SECURE_DWELL_TIME_MIN		5	/* milliseconds */
#define IEEE80211_OCAC_SECURE_DWELL_TIME_MAX		23	/* milliseconds */
#define IEEE80211_OCAC_SECURE_DWELL_TIME_DEFAULT	23	/* milliseconds */

#define IEEE80211_OCAC_DURATION_MIN		1	/* seconds */
#define IEEE80211_OCAC_DURATION_MAX		64800	/* seconds */
#define IEEE80211_OCAC_DURATION_DEFAULT		720	/* seconds */

#define IEEE80211_OCAC_CAC_TIME_MIN		1	/* seconds */
#define IEEE80211_OCAC_CAC_TIME_MAX		64800	/* seconds */
#define IEEE80211_OCAC_CAC_TIME_DEFAULT		240	/* seconds */

#define IEEE80211_OCAC_WEA_DURATION_MIN		60	/* seconds */
#define IEEE80211_OCAC_WEA_DURATION_MAX		86400	/* seconds */
#define IEEE80211_OCAC_WEA_DURATION_DEFAULT	11520	/* seconds */

#define IEEE80211_OCAC_WEA_CAC_TIME_MIN		1	/* seconds */
#define IEEE80211_OCAC_WEA_CAC_TIME_MAX		86400	/* seconds */
#define IEEE80211_OCAC_WEA_CAC_TIME_DEFAULT	4329	/* seconds */

#define IEEE80211_OCAC_THRESHOLD_FAT_MIN	1	/* percent */
#define IEEE80211_OCAC_THRESHOLD_FAT_MAX	100	/* percent */
#define IEEE80211_OCAC_THRESHOLD_FAT_DEFAULT	90	/* percent */

#define IEEE80211_OCAC_THRESHOLD_TRAFFIC_MIN		1	/* percent */
#define IEEE80211_OCAC_THRESHOLD_TRAFFIC_MAX		100	/* percent */
#define IEEE80211_OCAC_THRESHOLD_TRAFFIC_DEFAULT	30	/* percent */

#define IEEE80211_OCAC_OFFSET_TXHALT_MIN		2	/* milliseconds */
#define IEEE80211_OCAC_OFFSET_TXHALT_MAX		80	/* milliseconds */
#define IEEE80211_OCAC_OFFSET_TXHALT_DEFAULT		10	/* milliseconds */

#define IEEE80211_OCAC_OFFSET_OFFCHAN_MIN		2	/* milliseconds */
#define IEEE80211_OCAC_OFFSET_OFFCHAN_MAX		80	/* milliseconds */
#define IEEE80211_OCAC_OFFSET_OFFCHAN_DEFAULT		5	/* milliseconds */

#define IEEE80211_OCAC_TRAFFIC_CTRL_DEFAULT		1	/* on */

#define IEEE80211_OCAC_THRESHOLD_CCA_INTF_MIN		1	/* percent */
#define IEEE80211_OCAC_THRESHOLD_CCA_INTF_MAX		100	/* percent */
#define IEEE80211_OCAC_THRESHOLD_CCA_INTF_DEFAULT	20	/* percent */

#define IEEE80211_OCAC_THRESHOLD_FAT_DEC_MIN		1	/* percent */
#define IEEE80211_OCAC_THRESHOLD_FAT_DEC_MAX		100	/* percent */
#define IEEE80211_OCAC_THRESHOLD_FAT_DEC_DEFAULT	10	/* percent */

#define IEEE80211_OCAC_TIMER_INTERVAL_MIN		1	/* seconds */
#define IEEE80211_OCAC_TIMER_INTERVAL_MAX		100	/* seconds */
#define IEEE80211_OCAC_TIMER_INTERVAL_DEFAULT		2	/* seconds */

#define IEEE80211_OCAC_BEACON_INTERVAL_MIN		100	/* TUs */
#define IEEE80211_OCAC_BEACON_INTERVAL_MAX		1000	/* TUs */
#define IEEE80211_OCAC_BEACON_INTERVAL_DEFAULT		100	/* TUs */

#define IEEE80211_OCAC_TIMER_EXPIRE_INIT_MIN		1	/* seconds */
#define IEEE80211_OCAC_TIMER_EXPIRE_INIT_MAX		65000	/* seconds */
#define IEEE80211_OCAC_TIMER_EXPIRE_INIT_DEFAULT	2	/* seconds */

#define IEEE80211_OCAC_SUSPEND_TIME_MIN			1	/* seconds */
#define IEEE80211_OCAC_SUSPEND_TIME_MAX			65000	/* seconds */
#define IEEE80211_OCAC_SUSPEND_TIME_DEFAULT		2	/* seconds */

#define IEEE80211_OCAC_THRESHOLD_VIDEO_FRAMES_MIN	1	/* frames */
#define IEEE80211_OCAC_THRESHOLD_VIDEO_FRAMES_MAX	65000	/* frames */
#define IEEE80211_OCAC_THRESHOLD_VIDEO_FRAMES_DEFAULT	100	/* frames */

#define	IEEE80211_OBSS_PASSIVE_DWELL_DEFAULT		20
#define	IEEE80211_OBSS_ACTIVE_DWELL_DEFAULT		10
#define	IEEE80211_OBSS_TRIGGER_INTERVAL_DEFAULT		200
#define	IEEE80211_OBSS_PASSIVE_TOTAL_DEFAULT		200
#define	IEEE80211_OBSS_ACTIVE_TOTAL_DEFAULT		20
#define	IEEE80211_OBSS_CHANNEL_WIDTH_DELAY_DEFAULT	5
#define	IEEE80211_OBSS_ACTIVITY_THRESHOLD_DEFAULT	25

#define IEEE80211_OCAC_VALUE_S			0
#define IEEE80211_OCAC_VALUE_M			0xffff
#define IEEE80211_OCAC_COMMAND_S		16
#define IEEE80211_OCAC_COMMAND_M		0xffff
#define IEEE80211_OCAC_COMPRESS_VALUE_F		0x8000
#define IEEE80211_OCAC_COMPRESS_VALUE_M		0x7fff

#define IEEE80211_OCAC_TIME_MARGIN		2000	/* microseconds */

#define OCACLOG_CRIT				0
#define OCACLOG_WARNING				1
#define OCACLOG_NOTICE				2
#define OCACLOG_INFO				3
#define OCACLOG_VERBOSE				4
#define OCACLOG_LEVEL_MAX			4
#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
#define OCACDBG(_level, _fmt, ...)            do {               \
		if (ic->ic_ocac.ocac_cfg.ocac_debug_level >= (_level)) {  \
			DBGFN("DFS_s_radio: " _fmt, ##__VA_ARGS__);     \
		}                                               \
        } while (0)
#endif

enum qtn_bwsig_cmds {
	IEEE80211_BWSIG_PROBE_ENABLE = 1,
	IEEE80211_BWSIG_PROBE_STATE_SWITCH = 2,
	IEEE80211_BWSIG_DEBUG_DUMP = 3,
	IEEE80211_BWSIG_DEBUG_LEVEL = 4,
	IEEE80211_BWSIG_PROBE_SEND_INTERVAL = 16,
	IEEE80211_BWSIG_PROBE_BURST,
	IEEE80211_BWSIG_PROBE_DURATION,
	IEEE80211_BWSIG_PROBE_MCS,
	IEEE80211_BWSIG_PROBE_TXPOWER,
	IEEE80211_BWSIG_PROBE_REENTRY_MIN,
	IEEE80211_BWSIG_PROBE_REENTRY_MAX,
	IEEE80211_BWSIG_PROBE_EN_RATE_MRGN,
	IEEE80211_BWSIG_PROBE_EN_RSSI_THRESHLD,
	IEEE80211_BWSIG_PROBE_EN_MCS_THRESHLD,
	IEEE80211_BWSIG_PROBE_EN_CHK_LIMIT,
	IEEE80211_BWSIG_PROBE_DIS_REENTRY_LIMIT,
	IEEE80211_BWSIG_EN_RATE_MRGN,
	IEEE80211_BWSIG_EN_DUTY_THRESHLD,
	IEEE80211_BWSIG_EN_DUTY_LOW_THRESHLD,
	IEEE80211_BWSIG_DIS_MIN_LIMIT,
	IEEE80211_BWSIG_DIS_RATE_MRGN,
	IEEE80211_BWSIG_DIS_CHK_LIMIT,
	IEEE80211_BWSIG_DIS_LIMIT,
	IEEE80211_BWSIG_SET_MAX
};

#define WCACLOG_LEVEL_MIN			0
#define WCACLOG_CRIT				1
#define WCACLOG_WARNING				2
#define WCACLOG_NOTICE				3
#define WCACLOG_INFO				4
#define WCACLOG_VERBOSE				5
#define WCACLOG_LEVEL_MAX			WCACLOG_VERBOSE
#if !defined(MUC_BUILD) && !defined(DSP_BUILD) && !defined(AUC_BUILD)
#define WCACDBG(_level, _fmt, ...)            do {               \
		if (ic->wcac_param.wcac_debug_level >= (_level)) {  \
			DBGFN("WCAC: " _fmt, ##__VA_ARGS__);     \
		}                                               \
	} while (0)
#endif

/* Zero Second DFS Feature */
enum qtn_zsdfs_set_cmds {
	IEEE80211_ZSDFS_SET_ENABLE = 1,
	IEEE80211_ZSDFS_SET_CHAN_BW,
	IEEE80211_ZSDFS_SET_CAC_TIME,
	IEEE80211_ZSDFS_SET_WEA_CAC_TIME,
	IEEE80211_ZSDFS_SET_DBG_LEVEL,
	IEEE80211_ZSDFS_SET_OFFSET_TXHALT,
	IEEE80211_ZSDFS_SET_MUC_DBG,
	IEEE80211_ZSDFS_SET_OFFSET_HIJACK,
	IEEE80211_ZSDFS_SET_MAX
};

enum qtn_zsdfs_get_cmds {
	IEEE80211_ZSDFS_GET_ACTIVE_STATUS = 1,
	IEEE80211_ZSDFS_GET_CHAN_BW,
	IEEE80211_ZSDFS_GET_CAC_TIME,
	IEEE80211_ZSDFS_GET_WEA_CAC_TIME,
	IEEE80211_ZSDFS_GET_DBG_LEVEL,
	IEEE80211_ZSDFS_GET_OFFSET_TXHALT,
	IEEE80211_ZSDFS_GET_MUC_DBG,
	IEEE80211_ZSDFS_GET_OFFSET_HIJACK,
	IEEE80211_ZSDFS_GET_MAX
};

#define IEEE80211_BGSCAN_TIME_MARGIN		2000	/* microseconds */

#define QTN_M2A_EVENT_TYPE_DTIM		1
#define	QTN_M2A_PS_EVENT_PM_ENABLE	2		/* enable power management */
#define	QTN_M2A_PS_EVENT_PM_DISABLE	3		/* disable power management */
#define	QTN_M2A_PS_EVENT_PS_POLL	4		/* ps poll */
#define	QTN_M2A_EVENT_TYPE_UAPSD_SP	5		/* U-APSD SP */
#define QTN_M2A_EVENT_PTID_FLAG_SET     6               /* Set per-TID flag(muc) */
#define QTN_M2A_EVENT_TYPE_TXBA_DISABLE	7		/* per VAP TX BA est control */

/* Common definitions for flags used to indicate ieee80211_node's states */
#define	IEEE80211_NODE_AUTH		0x00000001	/* authorized for data */
#define	IEEE80211_NODE_QOS		0x00000002	/* QoS enabled */
#define	IEEE80211_NODE_ERP		0x00000004	/* ERP enabled */
#define	IEEE80211_NODE_HT		0x00000008	/* HT enabled */
/* NB: this must have the same value as IEEE80211_FC1_PWR_MGT */
#define	IEEE80211_NODE_PWR_MGT		0x00000010	/* power save mode enabled */
#define	IEEE80211_NODE_PS_DELIVERING	0x00000040	/* STA out of PS, getting delivery */
#define	IEEE80211_NODE_PS_POLL		0x00000080	/* power save ps poll mode */
#define	IEEE80211_NODE_AREF		0x00000020	/* authentication ref held */
#define IEEE80211_NODE_2_TX_CHAINS      0x00000400	/* this node needs to use 2 TX chain only, for IOT purpose */
#define IEEE80211_NODE_11B		0x00000800	/* 11b only client */
#define IEEE80211_NODE_BRCM_S8		0x00000200	/* BRCM Galaxy device */
#define IEEE80211_NODE_UAPSD		0x00001000
#define IEEE80211_NODE_WDS_PEER		0x00002000	/* this node is the wds peer in a wds vap */
#define IEEE80211_NODE_VHT		0x00004000	/* VHT enabled */
#define IEEE80211_NODE_TPC		0x00008000	/* indicate tpc capability */
#define IEEE80211_NODE_HE		0x00010000	/* HE enabled */
#define IEEE80211_NODE_TWT		0x00020000	/* TWT Enabled */
#define IEEE80211_NODE_TWT_PWR_MGT	0x00040000	/* TWT Announced Pwr Mgmt */
#define IEEE80211_NODE_TWT_AWAKE	0x00080000	/* TWT Awake */
#define IEEE80211_NODE_HE_ER		0x00100000	/* HE_ER enabled */
#define IEEE80211_NODE_TWT_FLEX		0x00200000	/* FLEXIBLE TWT enabled */
#define IEEE80211_NODE_SST		0x00400000	/* SST enabled */
#define IEEE80211_NODE_DBVC_IS_ASLEEP	0x00800000	/* client is in PS when DBVC applies PS */
#define IEEE80211_NODE_DBVC		0x01000000	/* client is in DBVC mode */
#define IEEE80211_NODE_DBVC_OFFCHAN	0x02000000	/* client temporarily left current working channel */
#define IEEE80211_NODE_6G_HE		0x04000000	/* In 6GHz band, HE only */
#define IEEE80211_NODE_HECAP_VHTSTA	0x08000000	/* HE STA, connected in VHTopmode */

/* same define as IEEE80211_NODE_PS_DELIVERING,
 * IEEE80211_NODE_PS_DELIVERING  not used in AuC,
 * safe to override descr_ni_flags in AuC
 * do not use this define in MuC/driver
 */
#define IEEE80211_AUC_NODE_PSPOLL_PKT_QUEUED	0x0040

/* Common definitions for ext_flags */
#define IEEE80211_NODE_TDLS_PTI_REQ	0x0001	/* Should sending PTI request to peer */
#define IEEE80211_NODE_TDLS_PTI_PENDING	0x0002	/* PTI request xmit to peer but not responsed */
#define IEEE80211_NODE_UAPSD_SP_IN_PROGRESS	0x0004	/* U-APSD SP in progress */
#define IEEE80211_NODE_TDLS_PTI_RESP	0x0008	/* PTI response frame received */
#define IEEE80211_NODE_EXT_PWR_MGT	0x0010	/* Applicable only for STA. If set VAP is in PSM state */
#define	IEEE80211_NODE_TDLS_MASK	0x000B	/* Mask for TDLS bits */
#define IEEE80211_NODE_LNCB_4ADDR	0x0080	/* 4-addr encap of LNCB packets (multicast) */
#define IEEE80211_NODE_IN_LNCB_LIST	0x0100
#define IEEE80211_NODE_IN_BR_LIST	0x0200

#define QTN_VAP_PRIORITY_RESERVED	2	/* reserve the low values for internal use */
#define QTN_VAP_PRIORITY_NUM		4
#define QTN_VAP_PRIORITY_MGMT		(QTN_VAP_PRIORITY_RESERVED + QTN_VAP_PRIORITY_NUM)
#define QTN_TACMAP_HW_PRI_NUM		8	/* hw limitation for 128 node mode */
#define QTN_TACMAP_PRI_PER_VAP		8	/* for maximum 8 TIDs */
#define QTN_TACMAP_SW_PRI_BASE		64	/* values below this are used for "bad apple" nodes */

/* Pearl QoS */
#define QTN_QOS_AIRQUOTA_MIN	0
#define QTN_QOS_AIRQUOTA_MAX	1000
#define QTN_QOS_AIRQUOTA_DFT	QTN_QOS_AIRQUOTA_MAX
#define QTN_QOS_AIRFAIR_DFT	0
#define QTN_QOS_AIRQUOTA_CASCADE(_x, _y)	((_x) * (_y) / QTN_QOS_AIRQUOTA_MAX)

enum {
	QTN_QOS_FAMILY_PREMIER_RULE_FREE = 0,
	QTN_QOS_FAMILY_PREMIER_RULE_ATF,
	QTN_QOS_FAMILY_PREMIER_RULE_NUM,
};

/* Quantenna specific flags (ni_qtn_flags), do not modify in Auc */
#define QTN_IS_BCM_NODE			0x0000001
#define QTN_IS_INTEL_5100_NODE		0x0000002
#define QTN_IS_INTEL_5300_NODE		0x0000004
#define QTN_IS_CLIENT_COLLIDES_MUCH	0x0000008 /* client has excessive number of bidirectional traffic collisions */
#define QTN_AC_BE_INHERITANCE_UPTO_VO	0x0000020
#define QTN_AC_BE_INHERITANCE_UPTO_VI	0x0000040
#define QTN_IS_INTEL_NODE		0x0000080
#define QTN_IS_REALTEK_NODE		0x0000100
#define	QTN_NODE_TX_RESTRICTED		0x0000200 /* restricted tx enabled */
#define	QTN_NODE_TX_RESTRICT_RTS	0x0000400 /* use RTS to confirm node is lost */
#define QTN_IS_NO_RXAMSDU_NO_BF_NODE	0x0000800
#define QTN_NODE_RXAMSDU_SUPPORT	0x0001000 /* node support TX amsdu */
#define QTN_NODE_11N_TXAMSDU_OFF	0x0002000
#define	QTN_NODE_TXOP_RESTRICTED	0x0004000
#define	QTN_NODE_TXOP_DISABLE		0x0008000
#define	QTN_NODE_FORCE_STD_SND		0x0010000
#define	QTN_NODE_IS_LEAVING		0x0020000
#define QTN_NODE_AMSDU_SCHEDULER_ID	0x00C0000
#define QTN_NODE_AMSDU_SCHEDULER_ID_S	18
#define	QTN_NODE_IS_MBO			0x0100000
#define QTN_IS_QTN_REPEATER		0x0200000
#define QTN_IS_APPLE_NODE		0x0400000
#define QTN_NODE_BF_FEEDBACK_OFF	0x0800000 /* disable CBF response for this node */
#define QTN_IS_RUBY_ENCRYPTED		0x1000000

/*
 * Bits that can be updated again by Lhost after association creation. Explicit definition helps
 * avoid overwriting bits maintained by MuC itself.
 */
#define QTN_FLAGS_UPDATABLE_BITS	(QTN_IS_INTEL_NODE | QTN_NODE_FORCE_STD_SND)

#define QTN_NODE_RXACCEL_LU_SA_ON	0x0000001 /* node support source address look up */

/* QTN bandwidth definition - make sure this is up-to-date with regards to txbf_common.h */
enum qtn_bw {
	QTN_BW_20M	= 0,
	QTN_BW_40M,
	QTN_BW_80M,
	QTN_BW_160M,

	QTN_BW_MAX	= QTN_BW_160M,
	QTN_BW_INVALID	= -1
};

#define QTN_BW_TO_TXBF_BW(qtn_bw)	(1 << qtn_bw)
#define QTN_BW_MAX_VAL 3

#define QTN_TX_CHAIN_MAX		8
#define QTN_NUM_RF_TX_GAIN		36
#define QTN_RF_BACKOFF_GRANULARITY	3

#define RTS_BWSIG_BW_MAX	(QTN_BW_160M + 1)

#define BWSIG_HW_BW_IDX_TO_QTN_BW(bw_idx)	(QTN_BW_160M - (bw_idx))
#define BWSIG_QTN_BW_TO_HW_BW_IDX(qtn_bw)	(QTN_BW_160M - (qtn_bw))
#define QTN_MAILBOX_INVALID	0xffffffff	/* Invalid value to indicate mailbox is disabled */

/* WoWLAN APIs */
enum qtn_vap_wowlan_cmds {
	IEEE80211_WOWLAN_HOST_POWER_SAVE = 1,
	IEEE80211_WOWLAN_MATCH_TYPE,
	IEEE80211_WOWLAN_L2_ETHER_TYPE,
	IEEE80211_WOWLAN_L3_UDP_PORT,
	IEEE80211_WOWLAN_MAGIC_PATTERN,
	IEEE80211_WOWLAN_MAGIC_PATTERN_GET,
	IEEE80211_WOWLAN_SET_MAX
};
/*
 * Definitions relating to individual fields from phy_stats,
 * shared between the Q driver and the APIs.
 */

/*
 * Error Sum needs to be reported together with the corresponding Number of
 * Symbols; getting them in separate operations would introduce a race condition
 * where the Error Sum and the Number of Symbols came from different
 * PHY stat blocks.
 */

#define QTN_PHY_AVG_ERROR_SUM_NSYM_NAME			"avg_error_sum_nsym"

#define QTN_PHY_EVM_MANTISSA_SHIFT		5
#define QTN_PHY_EVM_EXPONENT_MASK		0x1f

enum qtn_phy_stat_field {
	QTN_PHY_NOSUCH_FIELD = -1,
	QTN_PHY_AVG_ERROR_SUM_NSYM_FIELD,
};

#define QTN_M2A_TX_SCALE_BITS	4
#define QTN_M2A_TX_SCALE_MASK	((1 << QTN_M2A_TX_SCALE_BITS) - 1)

/* only for little endian */
#if defined(AUC_BUILD)
#define U64_LOW32(_v)		((uint32_t)(_v))
#define U64_HIGH32(_v)		((uint32_t)((_v) >> 32))
#else
#define U64_LOW32(_v)		(((uint32_t*)&(_v))[0])
#define U64_HIGH32(_v)		(((uint32_t*)&(_v))[1])
#endif

#define U64_COMPARE_GE(_a, _b)	((U64_HIGH32(_a) > U64_HIGH32(_b)) ||	\
				((U64_HIGH32(_a) == U64_HIGH32(_b)) && (U64_LOW32(_a) >= U64_LOW32(_b))))

#define U64_COMPARE_GT(_a, _b)	((U64_HIGH32(_a) > U64_HIGH32(_b)) ||	\
				((U64_HIGH32(_a) == U64_HIGH32(_b)) && (U64_LOW32(_a) > U64_LOW32(_b))))

#define U64_COMPARE_LE(_a, _b)	((U64_HIGH32(_a) < U64_HIGH32(_b)) ||	\
				((U64_HIGH32(_a) == U64_HIGH32(_b)) && (U64_LOW32(_a) <= U64_LOW32(_b))))

#define U64_COMPARE_LT(_a, _b)	((U64_HIGH32(_a) < U64_HIGH32(_b)) ||	\
				((U64_HIGH32(_a) == U64_HIGH32(_b)) && (U64_LOW32(_a) < U64_LOW32(_b))))

#define MAC_ADDR_LEN		6
#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
#define MACSTRL "%02x:%02x:%02x:%02x:%02x:%02x"	/* for MuC and Auc which don't support "X" */
#endif

#define ASSERT_CONCAT_(__a, __b) __a##__b
#define ASSERT_CONCAT(__a, __b) ASSERT_CONCAT_(__a, __b)

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && defined(static_assert))
#define STATIC_ASSERT(__expr,__message) static_assert(__expr, __message)
#elif defined(__COUNTER__)
#define STATIC_ASSERT(__expr,__message) \
	enum { ASSERT_CONCAT(__static_assert_, __COUNTER__) = 1/(int)(!!(__expr)) }
#else
/*
 * This can't be used twice on the same line so ensure if using in headers
 * that the headers are not included twice (by wrapping in #ifndef...#endif)
 * Note it doesn't cause an issue when used on same line of separate modules
 * compiled with gcc -combine -fwhole-program.
 */
#define STATIC_ASSERT(__expr,__message) \
	enum { ASSERT_CONCAT(__assert_line_, __LINE__) = 1/(int)(!!(__expr)) }
#endif

#define COMPILE_TIME_ASSERT(__expr) STATIC_ASSERT((__expr), "assertion failed")
#define COMPILE_TIME_ASSERT_TYPE_ALIGNMENT(__type, __alignment) \
		STATIC_ASSERT(sizeof(__type) % (__alignment) == 0, \
			      "type " #__type " size is not " #__alignment " byte aligned")

#define IS_POW2(X) ((X) && !((X) & ((X) - 1)))

#define QTN_IS_11B_PHYRATE_IDX(_v)		\
	(((_v) == QTN_PHY_RATE_LEGACY_1M) ||	\
	 ((_v) == QTN_PHY_RATE_LEGACY_2M) ||	\
	 ((_v) == QTN_PHY_RATE_LEGACY_5_5M) ||	\
	 ((_v) == QTN_PHY_RATE_LEGACY_11M))

/**@addtogroup DFSAPIs
 *@{*/
/**
 * Reason for channel change
 */
enum ieee80211_csw_reason {
	/**
	 * Reason is unknown
	 */
	IEEE80211_CSW_REASON_UNKNOWN,
	/**
	 * Smart channel selection
	 */
	IEEE80211_CSW_REASON_SCS,
	/**
	 * Radar detection
	 */
	IEEE80211_CSW_REASON_DFS,
	/**
	 * Channel set by user
	 */
	IEEE80211_CSW_REASON_MANUAL,
	/**
	 * Configuration change
	 */
	IEEE80211_CSW_REASON_CONFIG,
	/**
	 * Scan initiated by user
	 */
	IEEE80211_CSW_REASON_SCAN,
	/**
	 * Off-channel CAC
	 */
	IEEE80211_CSW_REASON_OCAC,
	/**
	 * Channel switch announcement
	 */
	IEEE80211_CSW_REASON_CSA,
	/**
	 * TDLS Channel switch announcement
	 */
	IEEE80211_CSW_REASON_TDLS_CS,
	/**
	 * Number of values
	 */
	IEEE80211_CSW_REASON_MAX
};

/**
 * \brief Enumeration to represent 802.11 legacy PHY rates
 */
typedef enum {
	/**
	 * 6M
	 */
	QTN_PHY_RATE_LEGACY_6M = 0,
	/**
	 * 9M
	 */
	QTN_PHY_RATE_LEGACY_9M,
	/**
	 * 12M
	 */
	QTN_PHY_RATE_LEGACY_12M,
	/**
	 * 18M
	 */
	QTN_PHY_RATE_LEGACY_18M,
	/**
	 * 24M
	 */
	QTN_PHY_RATE_LEGACY_24M,
	/**
	 * 36M
	 */
	QTN_PHY_RATE_LEGACY_36M,
	/**
	 * 48M
	 */
	QTN_PHY_RATE_LEGACY_48M,
	/**
	 * 54M
	 */
	QTN_PHY_RATE_LEGACY_54M,
	/**
	 * 1M
	 */
	QTN_PHY_RATE_LEGACY_1M,
	/**
	 * 2M
	 */
	QTN_PHY_RATE_LEGACY_2M,
	/**
	 * 5.5M
	 */
	QTN_PHY_RATE_LEGACY_5_5M,
	/**
	 * 11M
	 */
	QTN_PHY_RATE_LEGACY_11M,
	/**
	 * Placeholder - invalid
	 */
	QTN_PHY_RATE_LEGACY_INVALID,
} qcsapi_legacy_phyrate;
/**@}*/

/*
 * Reasons for channel switches that are not recorded and therefore
 * should not be listed in QCSAPI documentation
 */
enum ieee80211_csw_reason_private {
	IEEE80211_CSW_REASON_SAMPLING = IEEE80211_CSW_REASON_MAX,
	IEEE80211_CSW_REASON_OCAC_RUN,
	IEEE80211_CSW_REASON_BGSCAN,
	IEEE80211_CSW_REASON_DBVC,
};

/* Keep this in sync with ieee80211_wireless_swfeat_desc */
enum swfeat {
	SWFEAT_ID_MODE_AP,
	SWFEAT_ID_MODE_STA,
	SWFEAT_ID_MODE_REPEATER,
	SWFEAT_ID_PCIE_RC,
	SWFEAT_ID_VHT,
	SWFEAT_ID_2X2,
	SWFEAT_ID_2X4,
	SWFEAT_ID_3X3,
	SWFEAT_ID_4X4,
	SWFEAT_ID_5X5,
	SWFEAT_ID_6X6,
	SWFEAT_ID_5X8,
	SWFEAT_ID_8X8,
	SWFEAT_ID_24G_2X2,
	SWFEAT_ID_24G_4X4,
	SWFEAT_ID_HS20,
	SWFEAT_ID_WPA2_ENT,
	SWFEAT_ID_MESH,
	SWFEAT_ID_TDLS,
	SWFEAT_ID_OCAC,
	SWFEAT_ID_QHOP,
	SWFEAT_ID_QSV,
	SWFEAT_ID_QSV_NEIGH,
	SWFEAT_ID_MU_MIMO,
	SWFEAT_ID_DUAL_CHAN_VIRT,
	SWFEAT_ID_DUAL_CHAN,
	SWFEAT_ID_DUAL_BAND_VIRT,
	SWFEAT_ID_DUAL_BAND,
	SWFEAT_ID_QTM_PRIO,
	SWFEAT_ID_QTM,
	SWFEAT_ID_SPEC_ANALYZER,
	SWFEAT_ID_HE,
	SWFEAT_ID_BWSIG,
	SWFEAT_ID_ZS_DFS,
	SWFEAT_ID_MAX
};

#define SWFEAT_MAP_SIZE (SWFEAT_ID_MAX / 8 + 1)

/* Used to scale temperature measurements */
#define QDRV_TEMPSENS_COEFF	100000
#define QDRV_TEMPSENS_COEFF10	(10 * QDRV_TEMPSENS_COEFF)
#define QDRV_TEMPSENS_INVALID	(-274 * QDRV_TEMPSENS_COEFF10)

#define QDRV_TEMPSENS_ID_RFIC_EXT 1
#define QDRV_TEMPSENS_ID_BBIC_EXT 2
#define QDRV_PMIC0_ID_EXT	8
#define QDRV_PMIC1_ID_EXT	9

/* Define the max software retry number for aggregation and none-aggregation frames */
#define	QTN_TX_SW_ATTEMPTS_AGG_MAX 8
#define QTN_TX_SW_ATTEMPTS_NOAGG_MAX 1

#define QTN_MAX_TX_RATES		4
#define QTN_TX_SW_ATTEMPTS_AGG_MAX_BWSIG	(2)
#define QTN_TX_SW_RETRY_ENTRIES_MAX_BWSIG	(QTN_TX_SW_ATTEMPTS_AGG_MAX_BWSIG << 2)
#define QTN_TX_SW_RETRY_ENTRY_NUM		MAX((QTN_TX_SW_ATTEMPTS_AGG_MAX), (QTN_TX_SW_RETRY_ENTRIES_MAX_BWSIG))
#define QTN_SW_RETRY_RATES	(QTN_TX_SW_ATTEMPTS_AGG_MAX - 1)

/* Aligns the supplied size to the specified power_of_two */
#define QTN_ALIGN_TO(size_to_align, power_of_two) \
	(((size_to_align) + (power_of_two) - 1) & ~((power_of_two) - 1))

#define FIELD_ARRAY_SIZE(t, a)	(sizeof((((t*)0)->a))/sizeof(((((t*)0)->a))[0]))

#define QTN_HE_RU_INDEX_MIN	0
#define QTN_HE_RU_INDEX_MAX	36

#define QTN_HE_MAX_SIGB_MCS	5

#define QTN_HE_GI_LTF_FIXED_EN		0x10
#define QTN_HE_GI_LTF_FIXED_TYPE	0xF
#define QTN_HE_GI_LTF_FIXED_MASK	(QTN_HE_GI_LTF_FIXED_EN | QTN_HE_GI_LTF_FIXED_TYPE)

#define QTN_HE_TPE_FIXED_EN		0x10
#define QTN_HE_TPE_FIXED_VAL		0x3
#define QTN_HE_TPE_FIXED_MASK		(QTN_HE_TPE_FIXED_EN | QTN_HE_TPE_FIXED_VAL)

enum {
	QTN_HE_GI_LTF_TYPE_0 = 0,	/* 0.8us GI with 1x HE-LTF for (ER) SU PPDU; or 1.6us GI with 1x HE-LTF for TB PPDU */
	QTN_HE_GI_LTF_TYPE_1,		/* 0.8us GI with 2x HE-LTF */
	QTN_HE_GI_LTF_TYPE_2,		/* 1.6us GI with 2x HE-LTF */
	QTN_HE_GI_LTF_TYPE_3,		/* 3.2us GI with 4x HE-LTF */
	QTN_HE_GI_LTF_TYPE_MAX
};

enum {
	QTN_HE_NDP_GI_LTF_TYPE_0 = 0,	/* 0.8us GI with 2x HE-LTF */
	QTN_HE_NDP_GI_LTF_TYPE_1,	/* 1.6us GI with 2x HE-LTF */
	QTN_HE_NDP_GI_LTF_TYPE_2,	/* 3.2us GI with 4x HE-LTF */
	QTN_HE_NDP_GI_LTF_TYPE_MAX
};

enum {
	QTN_HE_TPE_0US = 0,
	QTN_HE_TPE_8US,
	QTN_HE_TPE_16US,
	QTN_HE_TPE_MAX
};

enum qtn_he_ru_size {
	QTN_HE_RU_SIZE_26 = 0,
	QTN_HE_RU_SIZE_52,
	QTN_HE_RU_SIZE_106,
	QTN_HE_RU_SIZE_242,
	QTN_HE_RU_SIZE_484,
	QTN_HE_RU_SIZE_996,
	QTN_HE_RU_SIZE_996x2,
	QTN_HE_RU_SIZE_MAX
};

enum qtn_ofdma_user_type {
	OFDMA_USER_TYPE_SU_NODE = 0,
	OFDMA_USER_TYPE_MU_GROUP	/* MU-MIMO group */
};
#define QTN_OFDMA_USER_ID_ALL		0xFF

typedef enum {
	QTN_OFDMA_CMD_ADD_GRP = 0,
	QTN_OFDMA_CMD_DEL_GRP,
	QTN_OFDMA_CMD_ACT_GRP,
	QTN_OFDMA_CMD_DUMP_GRP, /* Group commands must be defined before this line */
	QTN_OFDMA_CMD_ADD_USR,
	QTN_OFDMA_CMD_DEL_USR,
	QTN_OFDMA_CMD_SET_GI_LTF,
	QTN_OFDMA_CMD_SET_SIGB_MCS,
	QTN_OFDMA_CMD_SET_MCS,
	QTN_OFDMA_CMD_SET_UPLINK_PERIOD,
	QTN_OFDMA_CMD_SET_UPLINK_MCS,
	QTN_OFDMA_CMD_SET_LSIG_LENGTH,
	QTN_OFDMA_CMD_SET_UPLINK_GI_LTF,
	QTN_OFDMA_CMD_SET_MUMIMO_LTF_MODE,
	QTN_OFDMA_CMD_SET_LDPC_EXTRA_SYMB,
	QTN_OFDMA_CMD_SET_PKT_EXT,
	QTN_OFDMA_CMD_SET_SPATIAL_REUSE,
	QTN_OFDMA_CMD_SET_AP_TX_POWER,
	QTN_OFDMA_CMD_SET_TARGET_RSSI,
	QTN_OFDMA_CMD_SET_UL_LDPC,
	QTN_OFDMA_CMD_SET_UL_TXTIME,
	QTN_OFDMA_CMD_SET_UL_SCHEDULER_TXTIME,
	QTN_OFDMA_CMD_SET_UL_SCHEDULER_BURST_SIZE,
	QTN_OFDMA_CMD_SET_UL_CAROUSEL_SCHEDULER,
	QTN_OFDMA_CMD_SET_UL_MMSF,
	QTN_OFDMA_CMD_SET_DBG_LVL,
	QTN_OFDMA_CMD_SET_DBG_FLG,
	QTN_OFDMA_CMD_SET_BSRP_PERIOD,
	QTN_OFDMA_CMD_SET_GROUPING_MODE,
	QTN_OFDMA_CMD_SET_GROUP_SIZE,
	QTN_OFDMA_CMD_SET_GROUP_RU_SIZE,
	QTN_OFDMA_CMD_SET_ACTIVE_GROUP_LIMIT,
	QTN_OFDMA_CMD_SET_SU_COMPARE_SWITCH,
	QTN_OFDMA_CMD_SET_GRP_FLAG,
	QTN_OFDMA_CMD_SET_TX_SCALE_ALLOWANCE,
	QTN_OFDMA_CMD_GET_GRP_NUM_USERS,
	QTN_OFDMA_CMD_SET_SU_COMP_COEFF,
	QTN_OFDMA_CMD_SET_OFDMA_COMP_COEFF,
	QTN_OFDMA_CMD_SET_GRP_BITMAP_SWITCH,
	QTN_OFDMA_CMD_SET_OVERHEAD_COEFF,
	QTN_OFDMA_CMD_SET_SAMPLE_COUNT,
	QTN_OFDMA_CMD_SET_PEAK_TP_COEFF,
	QTN_OFDMA_CMD_SET_AGG_SIZE_COEFF,
	QTN_OFDMA_CMD_SET_RU_BLACKLIST_THRESH,
	QTN_OFDMA_CMD_DUMP_USERS,
	QTN_OFDMA_CMD_SET_INTF_THRESH,
	QTN_OFDMA_CMD_SET_RU_BLACKLIST,
	QTN_OFDMA_CMD_SET_OFDMA_SU_SHARE,
} qtn_ofdma_cmd;

typedef enum {
	OFDMA_GROUP_CMD = 0,
	OFDMA_USER_CMD,
	OFDMA_UL_CMD,
	OFDMA_MISC_CMD,
	OFDMA_GET_CMD,
} qtn_ofdma_cmd_class;

#define QTN_OFDMA_GROUP_MAX		IEEE80211_MU_OFDMA_GRP_NUM_MAX
#define QTN_OFDMA_GROUP_START		IEEE80211_MU_MIMO_GRP_NUM_MAX
#define QTN_OFDMA_GROUP_END		(QTN_OFDMA_GROUP_START + QTN_OFDMA_GROUP_MAX - 1)
#define QTN_OFDMA_GROUP_ID_VALID(_id)	(((_id) >= QTN_OFDMA_GROUP_START) && ((_id) <= QTN_OFDMA_GROUP_END))
#define QTN_OFDMA_GROUP_ID_ALL		0xFF

#define QTN_MU_MIMO_GROUP_ID_VALID(id)	(((id) >= IEEE80211_VHT_GRP_1ST_BIT_OFFSET) && \
							((id) <= IEEE80211_VHT_GRP_MAX_BIT_OFFSET))

#define QTN_OFDMA_FIXED_RATE_ENABLE	0x80
#define QTN_OFDMA_FIXED_RATE_MASK	0x7F
#define QTN_OFDMA_FIXED_RATE_CANCEL	0xFF
#define QTN_OFDMA_FIXED_RATE_VALUE(_r)	(((_r) == QTN_OFDMA_FIXED_RATE_CANCEL) ? 0 : \
	 (((_r) & QTN_OFDMA_FIXED_RATE_MASK) | QTN_OFDMA_FIXED_RATE_ENABLE))

#define QTN_TWT_BCAST_FLOWS_MAX			8
#define QTN_TWT_BCAST_NODES_PER_FLOW_MAX	4
#define QTN_TWT_FLOWS_PER_NODE_MAX		8
#define IEEE80211_TWT_EXPONENT_MAX		(BIT(5) - 1)
#define IEEE80211_TWT_MANTISSA_MAX		(BIT(16) - 1)
#define IEEE80211_TWT_DURATION_MAX		(BIT(8) - 1)

#ifdef PEARL_SPCS
#define QTN_SPCS_CHAN_LIST_5G			{40, 56, 104, 120, 136, 149, 153}
#define QTN_SPCS_CHAN_LIST_5G_B1		{40, 56, 104, 120, 136, 153}
#define QTN_SPCS_SPUR_CHAN_WITH_NF_ON		153
#define QTN_SPCS_SGMII_SPUR_CHAN_LIST		{100, 124, 149}
#define QTN_SPCS_CHAN_LIST_2PT4G_C0		{13}
#define QTN_SPCS_CHAN_LIST_5G_C0_BW20		{153}
#define QTN_SPCS_SGMII_SPUR_CHAN_LIST_BW20	{124, 149}
#endif

#define AUC_TX_DETECT_TCP_ENABLED	1
#define AUC_TX_DETECT_TCP_RTS_THRESHOLD_US			4000
#define AUC_TX_DETECT_TCP_RTS_COLLIDES_MUCH_THRESHOLD_US	400
#define AUC_TX_DETECT_TCP_RTS_THRESHOLD_PARAMS(thresh, collides_much_thresh) \
	((((uint32_t)(thresh)) << 16) | \
	(((uint32_t)(collides_much_thresh)) & 0xFFFF))
#define AUC_TX_DETECT_TCP_RTS_GET_THRESHOLD_US(params) \
	(((uint32_t)(params)) >> 16)
#define AUC_TX_DETECT_TCP_RTS_GET_COLLIDES_MUCH_THRESHOLD_US(params) \
	(((uint32_t)(params)) & 0xFFFF)

#define QTN_AUC_NORMAL_DENSITY_AMSDU_SCHEDULER 1
#define QTN_AUC_HIGH_DENSITY_AMSDU_SCHEDULER 2

enum qtn_ntx_mode {
	QTN_NTX_MODE_DEFAULT = 0,	/* default static ntx which depend on the RF config */
	QTN_NTX_MODE_DYNAMIC,		/* dynamic ntx=8 or 4 */
//	QTN_NTX_MODE_FOUR,		/* static ntx=4, unsupport it now */
	QTN_NTX_MODE_MAX
};

enum qtn_device_5g_mode {
	BB_MODE_REGULAR = 0,
	BB_MODE_80P80 = 1,
	BB_MODE_DCDC = 2,
	BB_MODE_DUAL80 = 3,
	BB_MODE_HIJACK = 4,
	BB_MODE_TOTAL
};

enum qtn_txbf_snd_mode {
	TXBF_SND_DEFAULT = 0,
	TXBF_SND_OS = 1,
	TXBF_SND_ALT_OS = 2,
	TXBF_SND_MAX
};

#define QTN_5G_MODE_IS_DUAL(_mode)	\
	((_mode == BB_MODE_DCDC) || (_mode == BB_MODE_DUAL80))

/* TWT Power Save */
#define QTN_TWT_AWAKE		0
#define QTN_TWT_SLEEP		1
#define QTN_TWT_AWAKE_ANNOUNCED	2

#define QTN_OFDMA_USERS_MAX	4			/* the maximum number of users in one OFDMA group */
#define QTN_OFDMA_2G_USERS_MAX	2			/* the maximum number of users in one 2.4G OFDMA group */
#define QTN_OFDMA_5G_USERS_MAX	QTN_OFDMA_USERS_MAX	/* the maximum number of users in one 5G OFDMA group */
#define QTN_UL_OFDMA_USERS_MAX	1	/* the maximum number of users in one uplink OFDMA group.
					 * BBIC5 C0 supports only one user in UL.
					 */

/* HE SU SIG-A BF bit control*/
#define QTN_HE_SIG_TXBF_ENABLE_PRECODE_APPLIED 0
#define QTN_HE_SIG_TXBF_ENABLE_CBF_RECEIVED 1
#define QTN_HE_SIG_TXBF_ENABLE_ALWAYS 2
#define QTN_HE_SIG_TXBF_DISABLE_ALWAYS 3
#define QTN_HE_SIG_TXBF_CTL_MAX 4

#define QTN_OFDMA_USERS_MAX_UNIT(_unit)	\
	((_unit == 2) ? QTN_OFDMA_2G_USERS_MAX : QTN_OFDMA_5G_USERS_MAX)

enum qtn_rts_dyn_bw_sig_mode {
	QTN_RTS_DYN_BW_SIG_DIS = 0,
	QTN_RTS_DYN_BW_SIG_FORCE = 1,
	QTN_RTS_DYN_BW_SIG_AUTO = 2,
#if RTS_DYN_BW_SIGNALING
	QTN_RTS_DYN_BW_SIG_MODE_MAX = 3,
#else
	QTN_RTS_DYN_BW_SIG_MODE_MAX = 1
#endif
};

enum qtn_wmac_timing {
	WMAC_TIMING_RX_DELAY_DSSS = 0,
	WMAC_TIMING_RX_DELAY_CCK,
	WMAC_TIMING_RX_DELAY_OFDM,
	WMAC_TIMING_RX_DELAY_11N,
	WMAC_TIMING_RX_DELAY_11AC,
	WMAC_TIMING_RX_DELAY_11AX,
	WMAC_TIMING_SEC_CCA_DELAY,
	WMAC_TIMING_SEC80_CCA_DELAY,
	WMAC_TIMING_TX_PHY_LATENCY,
	WMAC_TIMING_DELAY_MAX = WMAC_TIMING_TX_PHY_LATENCY,
	WMAC_TIMING_OFDM_SIFS,
	WMAC_TIMING_HT_SIFS,
	WMAC_TIMING_SLOT_TIME,
	WMAC_TIMING_NUM
};

/* Define CPU idle level below which disable feature */
#define TCP_ACK_COMPRESS_IDLENESS_THRES		(255 / 8)
#define TCP_ACK_COMPRESS_IDLENESS_THRES_VW	(256)

#define QTN_SMOOTHING_ALG(_smoothed, _rt, _smoothing_pow)	\
	(((_smoothed >= 1 ? _smoothed : 1) * (BIT(_smoothing_pow) - 1) + _rt) >> _smoothing_pow)

#define IEEE80211_AACS_VALUE_S			0
#define IEEE80211_AACS_VALUE_M			0xffff
#define IEEE80211_AACS_WF_VALUE_M		0xff
#define IEEE80211_AACS_COMMAND_S		16
#define IEEE80211_AACS_COMMAND_M		0xffff

#define IEEE80211_AACS_STATE_INIT		0
#define IEEE80211_AACS_STATE_RESET		1
#define IEEE80211_AACS_STATE_MAX		1

#define AACS_BW_MIN				0
#define AACS_BW_MAX				0xF
#define AACS_BW_M				0xF
#define AACS_BW_S				12
#define AACS_BW_20_M				0x01
#define AACS_BW_40_M				0x02
#define AACS_BW_80_M				0x04
#define AACS_BW_160_M				0x08

#define AACS_CH_MIN				0
#define AACS_CH_MAX				255
#define AACS_CH_M				0xFFF

#define AACS_SELCH_SIZE				10

enum {
	AACS_CH_OP_ADD = 0,
	AACS_CH_OP_ADD_ALT = 1,
	AACS_CH_OP_DEL = 2,
	AACS_CH_OP_DEL_ALT = 3,
	AACS_CH_OP_GET = 4,
	AACS_CH_OP_GET_ALT = 5,
	AACS_CH_OP_MAX
};

#define AACS_CH_OP_ADD_STR			"add"
#define AACS_CH_OP_ADD_STR_ALT			"alt_add"
#define AACS_CH_OP_DEL_STR			"del"
#define AACS_CH_OP_DEL_STR_ALT			"alt_del"
#define AACS_CH_OP_GET_STR			"get"
#define AACS_CH_OP_GET_STR_ALT			"alt_get"

#define AACS_THTBL_SIZE				6
#define AACS_APCNTTBL_SIZE			4
#define AACS_EXCL_CHAN_SIZE			2
#define AACS_EXCL_CHAN_OFF_START		0
#define AACS_EXCL_CHAN_OFF_END			1
#define AACS_VNODETBL_SIZE			4
#define AACS_VNODEQRY_SIZE			3
#define AACS_VNODEQRY_OFF_RSSI			0
#define AACS_VNODEQRY_OFF_WGT			1
#define AACS_VNODEQRY_OFF_IDX			2
#define AACS_VNODEWGT_INIT			25
#define AACS_VNODEWGT_REQD			100

#define AACS_BWCTRL_MIN				0
#define AACS_BWCTRL_MAX				7
#define AACS_BWLIMIT_MIN			0
#define AACS_BWLIMIT_20M			BW_HT20
#define AACS_BWLIMIT_40M			BW_HT40
#define AACS_BWLIMIT_80M			BW_HT80
#define AACS_BWLIMIT_160M			BW_HT160
#define AACS_BWLIMIT_MAX			AACS_BWLIMIT_160M
#define AACS_TIME_LLIMIT			1
#define AACS_TIME_ULIMIT			65535
#define AACS_TIME_HOURINSEC			3600
#define AACS_TIME_FACTORL			1
#define AACS_TIME_FACTORU			100
#define AACS_PCT_MIN				0
#define AACS_PCT_MAX				100
#define AACS_CTRL_DISABLE			0
#define AACS_CTRL_ENABLE			1
#define AACS_ALT_LIST_MIN			0
#define AACS_ALT_LIST_MAX			0x3F
#define AACS_EXCLTBL_NORM			0
#define AACS_EXCLTBL_RAW			1

#define AACS_NO_IOCTL				0
#define AACS_FIXED_INPUT			0

#define QTN_SST_BW_PRI_20_1	1
#define QTN_SST_BW_PRI_20_2	2
#define QTN_SST_BW_PRI_20_3	3
#define QTN_SST_BW_PRI_20_4	4
#define QTN_SST_BW_SEC_20_1	5
#define QTN_SST_BW_SEC_20_2	6
#define QTN_SST_BW_SEC_20_3	7
#define QTN_SST_BW_SEC_20_4	8
#define QTN_SST_BW_PRI_80	9
#define QTN_SST_BW_SEC_80	10
#define QTN_SST_BW_PRI_160	11
#define QTN_SST_BW_COUNT	(QTN_SST_BW_PRI_160 + 1)

#endif /* _SHARED_DEFS_H_ */
