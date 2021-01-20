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
 */
#ifndef _NET80211_IEEE80211_H_
#define _NET80211_IEEE80211_H_
#include <compat.h>
#include "net80211/_ieee80211.h"
#include "net80211/ieee80211_qos.h"
#include "net80211/ieee80211_dfs_reentry.h"
#include "net80211/ieee80211_nsm.h"
#include "net80211/ieee80211_ntx_chain.h"

/*
 * 802.11 protocol definitions.
 */

#define	IEEE80211_AMPDU_SUBFRM_MAX_LOG2	6
#define	IEEE80211_AMPDU_SUBFRM_MAX	(1 << IEEE80211_AMPDU_SUBFRM_MAX_LOG2)
#define	IEEE80211_HE_AMPDU_SUBFRM_MAX	256

#define	IEEE80211_ADDR_LEN	6		/* size of 802.11 address */
/* is 802.11 address multicast/broadcast? */
#define	IEEE80211_IS_MULTICAST(_a)	(*(_a) & 0x01)
/* is 802.11 address locally administered */
#define	IEEE80211_IS_LOCAL(_a)		(*(_a) & 0x02)

#define IEEE80211_RSSI_VALID(_rssi)           (((_rssi) < -1) && ((_rssi) > -1200))

#define IEEE80211_BCAST_SSID	1
#define IEEE80211_BCAST_RA	2

#define IEEE80211_SEC_TO_USEC(x)	((x) * 1000 * 1000)
#define IEEE80211_MS_TO_USEC(x)		((x) * 1000)
#define IEEE80211_USEC_TO_MS(x)		((x) / 1000)

#define	IEEE80211_MS_TO_TU(x)		(((x) * 1000) / 1024)
#define	IEEE80211_TU_TO_MS(x)		(((x) * 1024) / 1000)
#define	IEEE80211_TU_TO_USEC(x)		((x) * 1024)
#define	IEEE80211_USEC_TO_TU(x)		((x) / 1024)
#define IEEE80211_MS_TO_JIFFIES(x)	((x) * HZ / 1000)
#define IEEE80211_JIFFIES_TO_MS(x)	((x) * 1000 / HZ)
#define IEEE80211_US_TO_JIFFIES(x)	IEEE80211_MS_TO_JIFFIES((x) / 1000)
#define	IEEE80211_TU_TO_JIFFIES(x)	((IEEE80211_TU_TO_MS(x) * HZ) / 1000)

/* IEEE 802.11 PLCP header */
struct ieee80211_plcp_hdr {
	uint16_t	i_sfd;
	uint8_t	i_signal;
	uint8_t	i_service;
	uint16_t	i_length;
	uint16_t	i_crc;
} __packed;

#define IEEE80211_PLCP_SFD	0xF3A0
#define IEEE80211_PLCP_SERVICE  0x00

/* Dot11Msg reason fields - directly taken from the Reason field in the 802.11 spec(s). */
#define IEEE80211_REASON_STR { \
	"No disassoc reason reported", \
	"Unspecified reason", \
	"Previous authentication no longer valid", \
	"Deauthenticated because sending STA is leaving (or has left) IBSS or ESS", \
	"Disassociated due to inactivity", \
	"Disassociated because AP is unable to handle all currently associated STAs", \
	"Class 2 frame received from nonauthenticated STA", \
	"Class 3 frame received from nonassociated STA", \
	"Disassociated because sending STA is leaving (or has left) BSS", \
	"STA requesting (re)association is not authenticated with responding STA", \
	"Disassociated because the information in the Power Capability element is unacceptable", \
	"Disassociated because the information in the Supported Channels element is unacceptable", \
	"Reserved", \
	"Invalid information element", \
	"Message integrity code (MIC) failure", \
	"4-Way Handshake timeout", \
	"Group Key Handshake timeout", \
	"Information element in 4-Way Handshake different from (Re)Association Request/Probe Response/Beacon frame", \
	"Invalid group cipher", \
	"Invalid pairwise cipher", \
	"Invalid AKMP", \
	"Unsupported RSN information element version", \
	"Invalid RSN information element capabilities", \
	"IEEE 802.1X authentication failed", \
	"Cipher suite rejected because of the security policy", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"TS deleted because QoS AP lacks sufficient bandwidth due to change in BSS or operational mode", \
	"Disassociated for unspecified, QoS-related reason", \
	"Disassociated because QoS AP lacks sufficient bandwidth for this QoS STA", \
	"Disassociated because excessive number of frames need to be acknowledged, but are not acknowledged", \
	"Disassociated because STA is transmitting outside the limits of its TXOPs", \
	"Requested from peer STA as the STA is leaving hte BSS (or resetting)", \
	"Requested from peer STA as it does not want to use the mechanism", \
	"Requested from peer STA as the STA received frames using the mechanism for which a setup is required", \
	"Requested from peer STA due to timeout", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Reserved", \
	"Peer STA does not support the requested cipher suite", \
	"Disassociated because authorized access limit reached", \
	"Disassociated due to external service requirements", \
	"Invalid FT Action frame count", \
	"Invalid pairwise master key identifier (PMKI)", \
	"Invalid MDE", \
	"Invalid FTE", \
	"SME cancels the mesh peering instance with the reason other than reaching the maximum number of peer mesh STAs", \
	"The mesh STA has reached the supported maximum number of peer mesh STAs", \
	"The received information violates the Mesh Configuration policy configured in the mesh	STA profile", \
	"The mesh STA has received a Mesh Peering Close message requesting to close the mesh peering", \
	"The mesh STA has re-sent dot11MeshMaxRetries Mesh Peering Open messages, without receiving a Mesh Peering Confirm message", \
	"The confirmTimer for the mesh peering instance times out", \
	"The mesh STA fails to unwrap the GTK or the values in the wrapped contents do not match", \
	"The mesh STA receives inconsistent information about the mesh parameters between Mesh Peering Management frames", \
	"The mesh STA fails the authenticated mesh peering exchange because due to failure in selecting" \
		"either the pairwise ciphersuite or group ciphersuite", \
	"The mesh STA does not have proxy information for this external destination", \
	"The mesh STA does not have forwarding information for this destination", \
	"The mesh STA determines that the link to the next hop of an active path in its forwarding information is no longer usable", \
	"The Deauthentication frame was sent because the MAC address of the STA already exists in the mesh BSS." \
		"See 11.3.3 (Additional mechanisms for an AP collocated with a mesh STA)", \
	"The mesh STA performs channel switch to meet regulatory requirements", \
	"The mesh STA performs channel switch with unspecified reason", \
	}

/*
 * generic definitions for IEEE 802.11 frames
 */
struct ieee80211_frame {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_qosframe {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_qos[2];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_htframe {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_addr1[IEEE80211_ADDR_LEN];
	uint8_t	i_addr2[IEEE80211_ADDR_LEN];
	uint8_t	i_addr3[IEEE80211_ADDR_LEN];
	uint8_t	i_seq[2];
	uint8_t	i_ht[4];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_qoscntl {
	uint8_t i_qos[2];
};

struct ieee80211_ht_qosframe {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_addr1[IEEE80211_ADDR_LEN];
	uint8_t	i_addr2[IEEE80211_ADDR_LEN];
	uint8_t	i_addr3[IEEE80211_ADDR_LEN];
	uint8_t	i_seq[2];
	uint8_t	i_qos[2];
	uint8_t	i_ht[4];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_htcntl {
	uint8_t	i_ht[4];
};

struct ieee80211_frame_addr4 {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_addr4[IEEE80211_ADDR_LEN];
} __packed;


struct ieee80211_qosframe_addr4 {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_addr4[IEEE80211_ADDR_LEN];
	uint8_t i_qos[2];
} __packed;

#define IEEE80211_HT_CAPABLE		1
#define IEEE80211_NON_HT_CAPABLE	0

struct ieee80211_htframe_addr4 {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_addr1[IEEE80211_ADDR_LEN];
	uint8_t	i_addr2[IEEE80211_ADDR_LEN];
	uint8_t	i_addr3[IEEE80211_ADDR_LEN];
	uint8_t	i_seq[2];
	uint8_t	i_addr4[IEEE80211_ADDR_LEN];
	uint8_t	i_ht[4];
} __packed;

struct ieee80211_ht_qosframe_addr4 {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_addr1[IEEE80211_ADDR_LEN];
	uint8_t	i_addr2[IEEE80211_ADDR_LEN];
	uint8_t	i_addr3[IEEE80211_ADDR_LEN];
	uint8_t	i_seq[2];
	uint8_t	i_addr4[IEEE80211_ADDR_LEN];
	uint8_t	i_qos[2];
	uint8_t	i_ht[4];
} __packed;

#define IEEE80211_IS_4ADDRESS(__wh)	\
		(((__wh)->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS)

struct ieee80211_ctlframe_addr2 {
	uint8_t i_fc[2];
	__le16 i_aidordur; /* AID or duration */
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
} __packed;

#define IEEE80211_VHT_NDPA_STA_INFO_LEN	2
struct ieee80211_vht_su_ndpa {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_diagtoken;
	uint8_t i_sta1_info[IEEE80211_VHT_NDPA_STA_INFO_LEN];
} __packed;

/* Bits 0-11 */
/* VHT NDPA STA INFO AID12 */
static __inline__ void ieee80211_vht_ndpa_sta_info_set_aid12(
	uint8_t sta_info[IEEE80211_VHT_NDPA_STA_INFO_LEN], uint16_t associd)
{
	sta_info[0] = associd & 0xFF;
	sta_info[1] &= ~0x0F;
	sta_info[1] |= (associd >> 8) & 0x0F;
}

/* Bit 12 */
/* VHT NDPA STA INFO Feedback type */
static __inline__ void ieee80211_vht_ndpa_sta_info_set_fbtype(
	uint8_t sta_info[IEEE80211_VHT_NDPA_STA_INFO_LEN], uint8_t fbtype)
{
	sta_info[1] &= ~0x10;
	sta_info[1] |= (fbtype & 0x01) << 4;
}

/* Bits 13-15 */
/* VHT NDPA STA INFO NC */
static __inline__ void ieee80211_vht_ndpa_sta_info_set_nc(
	uint8_t sta_info[IEEE80211_VHT_NDPA_STA_INFO_LEN], uint8_t nc)
{
	sta_info[1] &= ~0xE0;
	sta_info[1] |= (nc & 0x07) << 5;
}

#define IEEE80211_HE_NDPA_STA_INFO_LEN	4
struct ieee80211_he_su_ndpa {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_diagtoken;
#if defined(PREAMBLE_PUNCTURE_SUPPORT)
	uint8_t i_aid2047_info[IEEE80211_HE_NDPA_STA_INFO_LEN];
#endif
	uint8_t i_sta1_info[IEEE80211_HE_NDPA_STA_INFO_LEN];
} __packed;

/* Bits 0-10 */
/* HE NDPA STA INFO AID11 */
static __inline__ void ieee80211_he_ndpa_sta_info_set_aid11(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint16_t associd)
{
	sta_info[0] = associd & 0xFF;
	sta_info[1] &= ~0x07;
	sta_info[1] |= (associd >> 8) & 0x07;
}

/* Bits 11-18 */
/* HE NDPA STA INFO DISALLOW SUBCHANNEL BITMAP */
static __inline__ void ieee80211_he_ndpa_aid2047_info_set_disallow_sub_ch_map(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t bitmap)
{
	sta_info[1] &= ~0xF8;
	sta_info[1] |= (bitmap & 0x1F) << 3;
	sta_info[2] &= ~0x07;
	sta_info[2] |= (bitmap >> 5) & 0x07;
}

/* Bits 11-17 */
/* HE NDPA STA INFO Partial BW RU START */
static __inline__ void ieee80211_he_ndpa_sta_info_set_ru_start(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint16_t ru)
{
	sta_info[1] &= ~0xF8;
	sta_info[1] |= (ru & 0x1F) << 3;
	sta_info[2] &= ~0x03;
	sta_info[2] |= (ru >> 5) & 0x03;
}

/* Bits 18-24 */
/* HE NDPA STA INFO Partial BW RU END */
static __inline__ void ieee80211_he_ndpa_sta_info_set_ru_end(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint16_t ru)
{
	sta_info[2] &= ~0xFC;
	sta_info[2] |= (ru & 0x3F) << 2;
	sta_info[3] &= ~0x01;
	sta_info[3] |= (ru >> 6) & 0x01;
}

/* Bit 25 */
/* HE NDPA STA INFO Feedback type */
static __inline__ void ieee80211_he_ndpa_sta_info_set_fbtype(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t fbtype)
{
	sta_info[3] &= ~0x02;
	sta_info[3] |= (fbtype & 0x01) << 1;
}

/* Bit 26 */
/* HE NDPA STA INFO NG */
static __inline__ void ieee80211_he_ndpa_sta_info_set_ng(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t ng)
{
	sta_info[3] &= ~0x04;
	sta_info[3] |= (ng & 0x01) << 2;
}

/* Bit 27 */
/* HE NDPA STA INFO Disambiguation */
static __inline__ void ieee80211_he_ndpa_sta_info_set_disambiguation(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t d)
{
	sta_info[3] &= ~0x08;
	sta_info[3] |= (d & 0x01) << 3;
}

/* Bit 28 */
/* HE NDPA STA INFO Codebook size */
static __inline__ void ieee80211_he_ndpa_sta_info_set_cb_sz(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t cb)
{
	sta_info[3] &= ~0x10;
	sta_info[3] |= (cb & 0x01) << 4;
}

/* Bits 29-31 */
/* HE NDPA STA INFO NC */
static __inline__ void ieee80211_he_ndpa_sta_info_set_nc(
	uint8_t sta_info[IEEE80211_HE_NDPA_STA_INFO_LEN], uint8_t nc)
{
	sta_info[3] &= ~0xE0;
	sta_info[3] |= (nc & 0x07) << 5;
}

static __inline__ uint32_t ieee80211_bar_get_tid_info(const uint8_t i_bar_ctl[])
{
	return i_bar_ctl[1] >> 4;
}

static __inline__ uint32_t ieee80211_bar_get_type(const uint8_t i_bar_ctl[])
{
	return (i_bar_ctl[0] >> 1) & 0x0f;
}

static __inline__ uint32_t ieee80211_ba_star_seq_ctrl_get_ssn(const uint8_t i_back_seq[])
{
	return (i_back_seq[0] | (i_back_seq[1] << 8)) >> 4;
}

static __inline__ uint32_t ieee80211_ba_star_seq_ctrl_get_fn(const uint8_t i_back_seq[])
{
	return i_back_seq[0] & 0x0f;
}

struct ieee80211_bar_frame {
        uint8_t i_fc[2];
        uint8_t i_dur[2];
        uint8_t i_addr1[IEEE80211_ADDR_LEN];
        uint8_t i_addr2[IEEE80211_ADDR_LEN];
        uint8_t i_bar_ctl[2];
        __le16   i_back_seq;
} __packed;

struct ieee80211_he_vht_mu_ndpa {
	uint8_t i_fc[2];
	__le16 i_dur;
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_diagtoken;
	uint8_t data[0];
} __packed;

struct ieee80211_vht_mu_rpt_poll {
	uint8_t i_fc[2];
	__le16 i_dur;
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_fbseg_map;
} __packed;

struct ieee80211_vht_mu_grp_mgmt {
	uint8_t i_fc[2];
	__le16 i_dur;
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	__le16 i_seq;
	uint8_t category;
	uint8_t vht_act;
	struct ieee80211_vht_mu_grp mu_grp;
} __packed;

struct ieee80211_auth {
	uint16_t auth_alg;
	uint16_t auth_transaction;
	uint16_t status_code;
	/* possibly followed by Challenge text */
	uint8_t variable[0];
} __packed;

struct ieee80211_assoc {
	uint16_t assoc_cap_info;
	uint16_t assoc_status_code;
	uint16_t assoc_aid;
	/* information elements */
	uint8_t variable[0];
} __packed;

#define	IEEE80211_FC0_VERSION_MASK		0x03
#define	IEEE80211_FC0_VERSION_SHIFT		0
#define	IEEE80211_FC0_VERSION_0			0x00
#define	IEEE80211_FC0_TYPE_MASK			0x0c
#define	IEEE80211_FC0_TYPE_SHIFT		2
#define	IEEE80211_FC0_TYPE_MGT			0x00
#define	IEEE80211_FC0_TYPE_CTL			0x04
#define	IEEE80211_FC0_TYPE_DATA			0x08

#define	IEEE80211_FC0_SUBTYPE_MASK		0xf0
#define	IEEE80211_FC0_SUBTYPE_SHIFT		4
/* for TYPE_MGT */
#define	IEEE80211_FC0_SUBTYPE_ASSOC_REQ		0x00
#define	IEEE80211_FC0_SUBTYPE_ASSOC_RESP	0x10
#define	IEEE80211_FC0_SUBTYPE_REASSOC_REQ	0x20
#define	IEEE80211_FC0_SUBTYPE_REASSOC_RESP	0x30
#define	IEEE80211_FC0_SUBTYPE_PROBE_REQ		0x40
#define	IEEE80211_FC0_SUBTYPE_PROBE_RESP	0x50
#define	IEEE80211_FC0_SUBTYPE_BEACON		0x80
#define	IEEE80211_FC0_SUBTYPE_ATIM		0x90
#define	IEEE80211_FC0_SUBTYPE_DISASSOC		0xa0
#define	IEEE80211_FC0_SUBTYPE_AUTH		0xb0
#define	IEEE80211_FC0_SUBTYPE_DEAUTH		0xc0
#define IEEE80211_FC0_SUBTYPE_ACTION		0xd0
#define IEEE80211_FC0_SUBTYPE_ACTION_NOACK	0xe0
/* for TYPE_CTL */
#define	IEEE80211_FC0_SUBTYPE_VHT_RPT_POLL      0x40
#define	IEEE80211_FC0_SUBTYPE_VHT_HE_NDPA	0x50
#define	IEEE80211_FC0_SUBTYPE_BAR		0x80
#define	IEEE80211_FC0_SUBTYPE_BA		0x90
#define	IEEE80211_FC0_SUBTYPE_PS_POLL		0xa0
#define	IEEE80211_FC0_SUBTYPE_RTS		0xb0
#define	IEEE80211_FC0_SUBTYPE_CTS		0xc0
#define	IEEE80211_FC0_SUBTYPE_ACK		0xd0
#define	IEEE80211_FC0_SUBTYPE_CF_END		0xe0
#define	IEEE80211_FC0_SUBTYPE_CF_END_ACK	0xf0
#define	IEEE80211_FC0_SUBTYPE_HETRIGGER		0x20
/* for TYPE_DATA (bit combination) */
#define	IEEE80211_FC0_SUBTYPE_DATA		0x00
#define	IEEE80211_FC0_SUBTYPE_CF_ACK		0x10
#define	IEEE80211_FC0_SUBTYPE_CF_POLL		0x20
#define	IEEE80211_FC0_SUBTYPE_CF_ACPL		0x30
#define	IEEE80211_FC0_SUBTYPE_NODATA		0x40
#define	IEEE80211_FC0_SUBTYPE_CFACK		0x50
#define	IEEE80211_FC0_SUBTYPE_CFPOLL		0x60
#define	IEEE80211_FC0_SUBTYPE_CF_ACK_CF_ACK	0x70
#define	IEEE80211_FC0_SUBTYPE_QOS		0x80
#define	IEEE80211_FC0_SUBTYPE_QOS_NULL		0xc0

#define	IEEE80211_FC1_DIR_MASK			0x03
#define	IEEE80211_FC1_DIR_NODS			0x00	/* STA->STA */
#define	IEEE80211_FC1_DIR_TODS			0x01	/* STA->AP  */
#define	IEEE80211_FC1_DIR_FROMDS		0x02	/* AP ->STA */
#define	IEEE80211_FC1_DIR_DSTODS		0x03	/* AP ->AP  */

#define	IEEE80211_FC1_MORE_FRAG			0x04
#define	IEEE80211_FC1_RETRY			0x08
#define	IEEE80211_FC1_PWR_MGT			0x10
#define	IEEE80211_FC1_MORE_DATA			0x20
#define	IEEE80211_FC1_PROT			0x40
#define	IEEE80211_FC1_WEP			0x40
#define	IEEE80211_FC1_ORDER			0x80

static __inline__ const uint8_t *ieee80211_qos_frame_get_i_ht(const void *qos_frame)
{
	const struct ieee80211_ht_qosframe_addr4 *wh_4 = qos_frame;
	const struct ieee80211_ht_qosframe *wh_3 = qos_frame;

	if (IEEE80211_IS_4ADDRESS(wh_4))
		return wh_4->i_ht;

	return wh_3->i_ht;
}

#define IEEE80211_QOS_CTRL_GET_SF(i_qos) \
	((i_qos)[1] >> 6)

#define IEEE80211_QOS_CTRL_GET_UV(i_qos) \
		((i_qos)[1] & 0x3f)

#define IEEE80211_QOS_CTRL_SF_UNSPECIFIED	3
#define IEEE80211_QOS_CTRL_UV_UNSPECIFIED	63

#define IEEE80211_QOS_CTRL_QUEUE_SIZE_MAX	2147328

#define	IEEE80211_HT0_VARIANT_HE		0x03

#define	IEEE80211_HE_A_CTRL_ID_TRS		0x00
#define	IEEE80211_HE_A_CTRL_ID_OM		0x01
#define	IEEE80211_HE_A_CTRL_ID_HLA		0x02
#define	IEEE80211_HE_A_CTRL_ID_BSR		0x03
#define	IEEE80211_HE_A_CTRL_ID_UPH		0x04
#define	IEEE80211_HE_A_CTRL_ID_BQR		0x05
#define	IEEE80211_HE_A_CTRL_ID_CAS		0x06
#define	IEEE80211_HE_A_CTRL_ID_ONES		0x0F
#define	IEEE80211_HE_A_CTRL_ONES_VAL		0xffffffff

/*
 * Based on "Figure 9-19b Control subfield format" and "Table 9-22a Control ID subfield values"
 * from IEEE P802.11ax/D6.0
 */
#define IEEE80211_HE_A_CTRL_ID_LENGTH_INITIALIZER \
	26 + 4, 12 + 4, 26 + 4, 26 + 4, 8 + 4, 10 + 4, 8 + 4

#define IEEE80211_HE_A_CTRL_GET_VARIANT(i_ht) \
	((i_ht)[0] & 0x03)

#define IEEE80211_HE_A_CTRL_GET_ID(i_ht) \
	(((i_ht)[0] >> 2) & 0x0f)

#define IEEE80211_HE_A_CTRL_BSR_GET_ACI_BITMAP(i_ht) \
	(((i_ht)[0] >> 6) | (((i_ht)[1] & 0x03) << 2))

#define IEEE80211_HE_A_CTRL_BSR_GET_DELTA_TID(i_ht) \
	(((i_ht)[1] >> 2) & 0x03)

#define IEEE80211_HE_A_CTRL_BSR_GET_ACI_HIGH(i_ht) \
	(((i_ht)[1] >> 4) & 0x03)

#define IEEE80211_HE_A_CTRL_BSR_GET_SCALING_FACTOR(i_ht) \
	(((i_ht)[1] >> 6) & 0x03)

#define IEEE80211_HE_A_CTRL_BSR_GET_QUEUE_SIZE_HIGH(i_ht) \
	((i_ht)[2])

#define IEEE80211_HE_A_CTRL_BSR_GET_QUEUE_SIZE_ALL(i_ht) \
	((i_ht)[3])

#define IEEE80211_HE_A_CTRL_BSR_QUEUE_SIZE_UNSPECIFIED \
	255

#define IEEE80211_HE_A_CTRL_BSR_QUEUE_SIZE_MAX \
	8323072

#define IEEE80211_HE_A_CTRL_OM_GET_RX_NSS(i_ht) \
	((((i_ht)[0] >> 6) & 0x03) | (((i_ht)[1] & 0x01) << 2))

#define IEEE80211_HE_A_CTRL_OM_GET_CHANNEL_WIDTH(i_ht) \
	(((i_ht)[1] >> 1) & 0x03)

#define IEEE80211_HE_A_CTRL_OM_GET_UL_MU_DISABLE(i_ht) \
	(((i_ht)[1] >> 3) & 0x01)

#define IEEE80211_HE_A_CTRL_OM_GET_TX_NSTS(i_ht) \
	(((i_ht)[1] >> 4) & 0x07)

#define IEEE80211_HE_A_CTRL_OM_GET_ER_SU_DISABLE(i_ht) \
	(((i_ht)[1] >> 7) & 0x01)

#define IEEE80211_HE_A_CTRL_OM_GET_DL_MU_MIMO_RESOUND(i_ht) \
	(((i_ht)[2] >> 0) & 0x01)

#define IEEE80211_HE_A_CTRL_OM_GET_UL_MU_DATA_DISABLE(i_ht) \
	(((i_ht)[2] >> 1) & 0x01)

#define IEEE80211_HE_A_CTRL_OM_RX_NSS			(0x000001c0)
#define IEEE80211_HE_A_CTRL_OM_RX_NSS_S			(6 + 0)
#define IEEE80211_HE_A_CTRL_OM_CHANNEL_WIDTH		(0x00000600)
#define IEEE80211_HE_A_CTRL_OM_CHANNEL_WIDTH_S		(6 + 3)
#define IEEE80211_HE_A_CTRL_OM_UL_MU_DISABLE		(0x0000800)
#define IEEE80211_HE_A_CTRL_OM_UL_MU_DISABLE_S		(6 + 5)
#define IEEE80211_HE_A_CTRL_OM_TX_NSTS			(0x00007000)
#define IEEE80211_HE_A_CTRL_OM_TX_NSTS_S		(6 + 6)
#define IEEE80211_HE_A_CTRL_OM_ER_SU_DISABLE		(0x0008000)
#define IEEE80211_HE_A_CTRL_OM_ER_SU_DISABLE_S		(6 + 9)
#define IEEE80211_HE_A_CTRL_OM_DL_MUMIMO_RESND		(0x0010000)
#define IEEE80211_HE_A_CTRL_OM_DL_MUMIMO_RESND_S	(6 + 10)
#define IEEE80211_HE_A_CTRL_OM_UL_MU_DATA_DISABLE	(0x0020000)
#define IEEE80211_HE_A_CTRL_OM_UL_MU_DATA_DISABLE_S	(6 + 11)

#define IEEE80211_HE_A_CTRL_OM_BASIC_TRIGGER_ALLOWED(ctrl)	(!((ctrl) & \
	(IEEE80211_HE_A_CTRL_OM_UL_MU_DATA_DISABLE | IEEE80211_HE_A_CTRL_OM_UL_MU_DISABLE)))
#define IEEE80211_HE_A_CTRL_OM_OTHER_TRIGGER_ALLOWED(ctrl)	\
	(!((ctrl) & IEEE80211_HE_A_CTRL_OM_UL_MU_DISABLE))

#define	IEEE80211_SEQ_FRAG_MASK			0x000f
#define	IEEE80211_SEQ_FRAG_SHIFT		0
#define	IEEE80211_SEQ_SEQ_MASK			0xfff0
#define	IEEE80211_SEQ_SEQ_SHIFT			4
#define IEEE80211_SEQ_RANGE			4096
#define IEEE80211_SEQ_ORDERLAG			64

#define IEEE80211_NDPA_TOKEN			0xFC
#define IEEE80211_NDPA_TOKEN_S			2
#define IEEE80211_NDPA_HE_MASK			0x02
#define IEEE80211_NDPA_RSRV_MASK		0x01
#define IEEE80211_NDPA_RSRV_SHIFT		0

#define IEEE80211_SEQ_ADD(seq, offset) \
	(((seq) + (offset)) & (IEEE80211_SEQ_RANGE - 1))
#define IEEE80211_SEQ_SUB(seq, offset)					\
	(((seq) + IEEE80211_SEQ_RANGE - (offset)) & (IEEE80211_SEQ_RANGE - 1))
#define IEEE80211_SEQ_DIFF(seq_front, seq_back)				\
	(((seq_front) + IEEE80211_SEQ_RANGE - (seq_back)) & (IEEE80211_SEQ_RANGE - 1))
#define IEEE80211_SEQ_INORDER_LAG(seq_front, seq_back, seq_lag)		\
	(IEEE80211_SEQ_DIFF((seq_front), (seq_back)) < (seq_lag))
#define IEEE80211_SEQ_INORDER(seq_front, seq_back)			\
	IEEE80211_SEQ_INORDER_LAG((seq_front), (seq_back), IEEE80211_SEQ_ORDERLAG)
#define	IEEE80211_SEQ_EQ(a,b)	((a) == (b))

#define	IEEE80211_NWID_LEN			32

#define	IEEE80211_QOS_TXOP			0x00ff
/* bit 8 is reserved */
#define	IEEE80211_QOS_ACKPOLICY			0x60
#define	IEEE80211_QOS_ACKPOLICY_S		5
#define	IEEE80211_QOS_EOSP			0x10
#define	IEEE80211_QOS_EOSP_S			4
#define	IEEE80211_QOS_TID			0x0f
#define IEEE80211_QOS_A_MSDU_PRESENT		0x80

/* IEEE80211_QOS_ACKPOLICY field */
#define IEEE80211_QOS_NORMAL_ACK_POLICY		(0x00 << IEEE80211_QOS_ACKPOLICY_S)
#define IEEE80211_QOS_NO_ACK_POLICY		(0x01 << IEEE80211_QOS_ACKPOLICY_S)
#define IEEE80211_QOS_HTP_ACK_POLICY		(0x02 << IEEE80211_QOS_ACKPOLICY_S)
#define IEEE80211_QOS_BLOCK_ACK_POLICY		(0x03 << IEEE80211_QOS_ACKPOLICY_S)

#define IEEE80211_QOS_QUEUE_SIZE_UNKNOWN	255

/* bit 1 is reserved */
#define IEEE80211_HTC0_TRQ			0x02
#define IEEE80211_HTC0_MAI_MASK			0x3C
#define IEEE80211_HTC0_MAI_SHIFT		2
#define IEEE80211_HTC0_MFSI_LOW_MASK	0xC0
#define IEEE80211_HTC0_MFSI_LOW_SHIFT	6

#define IEEE80211_HTC1_MFSI_HIGH		0x01
#define IEEE80211_HTC1_MFB_ASEL_MASK	0xFE
#define IEEE80211_HTC1_MFB_ASEL_SHIFT	1

#define IEEE80211_HTC2_CALIB_POS_MASK	0x03
#define IEEE80211_HTC2_CALIB_POS_SHIFT	0
#define IEEE80211_HTC2_CALIB_SEQ_MASK	0x0C
#define IEEE80211_HTC2_CALIB_SEQ_SHIFT	2
/* bits 4-5 are reserved */
#define IEEE80211_HTC2_CSI_STEER_MASK	0xC0
#define IEEE80211_HTC2_CSI_STEER_SHIFT	6

#define IEEE80211_HTC3_NDP_ANNOUNCE		0x01
/* bits 1-5 are reserved */
#define IEEE80211_HTC3_AC_CONSTRAINT	0x40
#define IEEE80211_HTC3_MORE_PPDU_RDG	0x80

#define IEEE80211_CHAN_SPACE		5
#define IEEE80211_SEC_CHAN_OFFSET	4
#define IEEE80211_40M_CENT_FREQ_OFFSET	2
#define IEEE80211_OBSS_AFFECT_CHAN_SPAN 5

#define IEEE80211_ETH_P_PAE		0x888E

/*
 * Country/Region Codes from MS WINNLS.H
 * Numbering from ISO 3166
 * XXX belongs elsewhere
 *
 * First 2 entries taken from ieee80211.c ...
 */
enum CountryCode {
	CTRY_DEBUG	= 0x1ff,/* debug, =511 radix 10 */
	CTRY_DEFAULT	= 0,	/* default or not defined */
	CTRY_AFGHANISTAN	= 4,	/* Afghanistan */
	CTRY_ALAND_ISLANDS	= 248,	/* Åland Islands */
	CTRY_ALBANIA	= 8,	/* Albania */
	CTRY_ALGERIA	= 12,	/* Algeria */
	CTRY_AMERICAN_SAMOA	= 16,	/* American Samoa */
	CTRY_ANDORRA	= 20,	/* Andorra */
	CTRY_ANGOLA	= 24,	/* Angola */
	CTRY_ANGUILLA	= 660,	/* Anguilla */
	CTRY_ANTARTICA	= 10,	/* Antarctica */
	CTRY_ANTIGUA	= 28,	/* Antigua and Barbuda */
	CTRY_ARGENTINA	= 32,	/* Argentina */
	CTRY_ARMENIA	= 51,	/* Armenia */
	CTRY_ARUBA	= 533,	/* Aruba */
	CTRY_AUSTRALIA	= 36,	/* Australia */
	CTRY_AUSTRIA	= 40,	/* Austria */
	CTRY_AZERBAIJAN	= 31,	/* Azerbaijan */
	CTRY_BAHAMAS	= 44,	/* Bahamas (the) */
	CTRY_BAHRAIN	= 48,	/* Bahrain */
	CTRY_BANGLADESH	= 50,	/* Bangladesh */
	CTRY_BARBADOS	= 52,	/* Barbados */
	CTRY_BELARUS	= 112,	/* Belarus */
	CTRY_BELGIUM	= 56,	/* Belgium */
	CTRY_BELIZE	= 84,	/* Belize */
	CTRY_BENIN	= 204,	/* Benin */
	CTRY_BERMUDA	= 60,	/* Bermuda */
	CTRY_BHUTAN	= 64,	/* Bhutan */
	CTRY_BOLIVIA	= 68,	/* Bolivia (Plurinational State of) */
	CTRY_BONAIRE_SINT_EUSTATIUS_AND_SABA	= 535,	/* Bonaire, Sint Eustatius and Saba */
	CTRY_BOSNIA_AND_HERZEGOWINA	= 70,	/* Bosnia and Herzegovina */
	CTRY_BOTSWANA	= 72,	/* Botswana */
	CTRY_BOUVET_ISLAND	= 74,	/* Bouvet Island */
	CTRY_BRAZIL	= 76,	/* Brazil */
	CTRY_BRITISH_INDIAN_OCEAN_TERRITORY	= 86,	/* British Indian Ocean Territory (the) */
	CTRY_BRUNEI_DARUSSALAM	= 96,	/* Brunei Darussalam */
	CTRY_BULGARIA	= 100,	/* Bulgaria */
	CTRY_BURKINA_FASO	= 854,	/* Burkina Faso */
	CTRY_BURUNDI	= 108,	/* Burundi */
	CTRY_CAMBODIA	= 116,	/* Cambodia */
	CTRY_CAMEROON	= 120,	/* Cameroon */
	CTRY_CANADA	= 124,	/* Canada */
	CTRY_CAPE_VERDE	= 132,	/* Cabo Verde */
	CTRY_CAYMAN_ISLANDS	= 136,	/* Cayman Islands (the) */
	CTRY_CENTRAL_AFRICAN_REPUBLIC	= 140,	/* Central African Republic (the) */
	CTRY_CHAD	= 148,	/* Chad */
	CTRY_CHILE	= 152,	/* Chile */
	CTRY_CHINA	= 156,	/* China */
	CTRY_CHRISTMAS_ISLAND	= 162,	/* Christmas Island */
	CTRY_COCOS_ISLANDS	= 166,	/* Cocos (Keeling) Islands (the) */
	CTRY_COLOMBIA	= 170,	/* Colombia */
	CTRY_COMOROS	= 174,	/* Comoros (the) */
	CTRY_CONGO	= 178,	/* Congo (the) */
	CTRY_CONGO_DR	= 180,	/* Congo (the Democratic Republic of the) */
	CTRY_COOK_ISLANDS	= 184,	/* Cook Islands (the) */
	CTRY_COSTA_RICA	= 188,	/* Costa Rica */
	CTRY_COTE_DIVOIRE	= 384,	/* Côte d'Ivoire */
	CTRY_CROATIA	= 191,	/* Croatia */
	CTRY_CUBA	= 192,	/* Cuba */
	CTRY_CURACAO	= 531,	/* Curaçao */
	CTRY_CYPRUS	= 196,	/* Cyprus */
	CTRY_CZECH	= 203,	/* Czechia */
	CTRY_DENMARK	= 208,	/* Denmark */
	CTRY_DJIBOUTI	= 262,	/* Djibouti */
	CTRY_DOMINICA	= 212,	/* Dominica */
	CTRY_DOMINICAN_REPUBLIC	= 214,	/* Dominican Republic (the) */
	CTRY_ECUADOR	= 218,	/* Ecuador */
	CTRY_EGYPT	= 818,	/* Egypt */
	CTRY_EL_SALVADOR	= 222,	/* El Salvador */
	CTRY_EQUATORIAL_GUINEA	= 226,	/* Equatorial Guinea */
	CTRY_ERITREA	= 232,	/* Eritrea */
	CTRY_ESTONIA	= 233,	/* Estonia */
	CTRY_ESWATINI	= 748,	/* Eswatini */
	CTRY_ETHIOPIA	= 231,	/* Ethiopia */
	CTRY_EUROPE	= 200,	/* European Union */
	CTRY_FAEROE_ISLANDS	= 234,	/* Faroe Islands (the) */
	CTRY_FALKLAND_ISLANDS	= 238,	/* Falkland Islands (the) [Malvinas] */
	CTRY_FIJI	= 242,	/* Fiji */
	CTRY_FINLAND	= 246,	/* Finland */
	CTRY_FRANCE	= 250,	/* France */
	CTRY_FRANCE2	= 255,	/* France2 */
	CTRY_FRENCH_GUIANA	= 254,	/* French Guiana */
	CTRY_FRENCH_POLYNESIA	= 258,	/* French Polynesia */
	CTRY_FRENCH_SOUTHERN_TERRITORIES	= 260,	/* French Southern Territories (the) */
	CTRY_GABON	= 266,	/* Gabon */
	CTRY_GAMBIA	= 270,	/* Gambia (the) */
	CTRY_GEORGIA	= 268,	/* Georgia */
	CTRY_GERMANY	= 276,	/* Germany */
	CTRY_GHANA	= 288,	/* Ghana */
	CTRY_GIBRALTAR	= 292,	/* Gibraltar */
	CTRY_GREECE	= 300,	/* Greece */
	CTRY_GREENLAND	= 304,	/* Greenland */
	CTRY_GRENADA	= 308,	/* Grenada */
	CTRY_GUADELOUPE	= 312,	/* Guadeloupe */
	CTRY_GUAM	= 316,	/* Guam */
	CTRY_GUATEMALA	= 320,	/* Guatemala */
	CTRY_GUERNSEY	= 831,	/* Guernsey */
	CTRY_GUINEA	= 324,	/* Guinea */
	CTRY_GUINEA_BISSAU	= 624,	/* Guinea-Bissau */
	CTRY_GUYANA	= 328,	/* Guyana */
	CTRY_HAITI	= 332,	/* Haiti */
	CTRY_HEARD_ISLAND_AND_MCDONALD_ISLANDS	= 334,	/* Heard Island and McDonald Islands */
	CTRY_HOLY_SEE	= 336,	/* Holy See (the) */
	CTRY_HONDURAS	= 340,	/* Honduras */
	CTRY_HONG_KONG	= 344,	/* Hong Kong */
	CTRY_HUNGARY	= 348,	/* Hungary */
	CTRY_ICELAND	= 352,	/* Iceland */
	CTRY_INDIA	= 356,	/* India */
	CTRY_INDONESIA	= 360,	/* Indonesia */
	CTRY_IRAN	= 364,	/* Iran (Islamic Republic of) */
	CTRY_IRAQ	= 368,	/* Iraq */
	CTRY_IRELAND	= 372,	/* Ireland */
	CTRY_ISLE_OF_MAN	= 833,	/* Isle of Man */
	CTRY_ISRAEL	= 376,	/* Israel */
	CTRY_ITALY	= 380,	/* Italy */
	CTRY_JAMAICA	= 388,	/* Jamaica */
	CTRY_JAPAN	= 392,	/* Japan */
	CTRY_JAPAN1	= 393,	/* Japan (JP1) */
	CTRY_JAPAN2	= 394,	/* Japan (JP0) */
	CTRY_JAPAN3	= 395,	/* Japan (JP1-1) */
	CTRY_JAPAN4	= 396,	/* Japan (JE1) */
	CTRY_JAPAN5	= 397,	/* Japan (JE2) */
	CTRY_JAPAN6	= 399,	/* Japan (JP6) */
	CTRY_JAPAN7	= 900,	/* Japan */
	CTRY_JAPAN8	= 901,	/* Japan */
	CTRY_JAPAN9	= 902,	/* Japan */
	CTRY_JAPAN10	= 903,	/* Japan */
	CTRY_JAPAN11	= 904,	/* Japan */
	CTRY_JAPAN12	= 905,	/* Japan */
	CTRY_JAPAN13	= 906,	/* Japan */
	CTRY_JAPAN14	= 907,	/* Japan */
	CTRY_JAPAN15	= 908,	/* Japan */
	CTRY_JAPAN16	= 909,	/* Japan */
	CTRY_JAPAN17	= 910,	/* Japan */
	CTRY_JAPAN18	= 911,	/* Japan */
	CTRY_JAPAN19	= 912,	/* Japan */
	CTRY_JAPAN20	= 913,	/* Japan */
	CTRY_JAPAN21	= 914,	/* Japan */
	CTRY_JAPAN22	= 915,	/* Japan */
	CTRY_JAPAN23	= 916,	/* Japan */
	CTRY_JAPAN24	= 917,	/* Japan */
	CTRY_JAPAN25	= 918,	/* Japan */
	CTRY_JAPAN26	= 919,	/* Japan */
	CTRY_JAPAN27	= 920,	/* Japan */
	CTRY_JAPAN28	= 921,	/* Japan */
	CTRY_JAPAN29	= 922,	/* Japan */
	CTRY_JAPAN30	= 923,	/* Japan */
	CTRY_JAPAN31	= 924,	/* Japan */
	CTRY_JAPAN32	= 925,	/* Japan */
	CTRY_JAPAN33	= 926,	/* Japan */
	CTRY_JAPAN34	= 927,	/* Japan */
	CTRY_JAPAN35	= 928,	/* Japan */
	CTRY_JAPAN36	= 929,	/* Japan */
	CTRY_JAPAN37	= 930,	/* Japan */
	CTRY_JAPAN38	= 931,	/* Japan */
	CTRY_JAPAN39	= 932,	/* Japan */
	CTRY_JAPAN40	= 933,	/* Japan */
	CTRY_JAPAN41	= 934,	/* Japan */
	CTRY_JAPAN42	= 935,	/* Japan */
	CTRY_JAPAN43	= 936,	/* Japan */
	CTRY_JAPAN44	= 937,	/* Japan */
	CTRY_JAPAN45	= 938,	/* Japan */
	CTRY_JAPAN46	= 939,	/* JapanA */
	CTRY_JAPAN47	= 940,	/* Japan */
	CTRY_JAPAN48	= 941,	/* Japan */
	CTRY_JERSEY	= 832,	/* Jersey */
	CTRY_JORDAN	= 400,	/* Jordan */
	CTRY_KAZAKHSTAN	= 398,	/* Kazakhstan */
	CTRY_KENYA	= 404,	/* Kenya */
	CTRY_KIRIBATI	= 296,	/* Kiribati */
	CTRY_KOREA_NORTH	= 408,	/* Korea (the Democratic People's Republic of) */
	CTRY_KOREA_ROC	= 410,	/* Korea (the Republic of) */
	CTRY_KOREA_ROC2	= 411,	/* South Korea */
	CTRY_KUWAIT	= 414,	/* Kuwait */
	CTRY_KYRGYZSTAN	= 417,	/* Kyrgyzstan */
	CTRY_LAO	= 418,	/* Lao People's Democratic Republic (the) */
	CTRY_LATVIA	= 428,	/* Latvia */
	CTRY_LEBANON	= 422,	/* Lebanon */
	CTRY_LESOTHO	= 426,	/* Lesotho */
	CTRY_LIBERIA	= 430,	/* Liberia */
	CTRY_LIBYA	= 434,	/* Libya */
	CTRY_LIECHTENSTEIN	= 438,	/* Liechtenstein */
	CTRY_LITHUANIA	= 440,	/* Lithuania */
	CTRY_LUXEMBOURG	= 442,	/* Luxembourg */
	CTRY_MACAU	= 446,	/* Macao */
	CTRY_MACEDONIA	= 807,	/* Macedonia (the former Yugoslav Republic of) */
	CTRY_MADAGASCAR	= 450,	/* Madagascar */
	CTRY_MALAWI	= 454,	/* Malawi */
	CTRY_MALAYSIA	= 458,	/* Malaysia */
	CTRY_MALDIVES	= 462,	/* Maldives */
	CTRY_MALI	= 466,	/* Mali */
	CTRY_MALTA	= 470,	/* Malta */
	CTRY_MARSHALL_ISLANDS	= 584,	/* Marshall Islands (the) */
	CTRY_MARTINIQUE	= 474,	/* Martinique */
	CTRY_MAURITANIA	= 478,	/* Mauritania */
	CTRY_MAURITIUS	= 480,	/* Mauritius */
	CTRY_MAYOTTE	= 175,	/* Mayotte */
	CTRY_MEXICO	= 484,	/* Mexico */
	CTRY_MICRONESIA	= 583,	/* Micronesia (Federated States of) */
	CTRY_MOLDOVA	= 498,	/* Moldova (the Republic of) */
	CTRY_MONACO	= 492,	/* Monaco */
	CTRY_MONGOLIA	= 496,	/* Mongolia */
	CTRY_MONTENEGRO	= 499,	/* Montenegro */
	CTRY_MONTSERRAT	= 500,	/* Montserrat */
	CTRY_MOROCCO	= 504,	/* Morocco */
	CTRY_MOZAMBIQUE	= 508,	/* Mozambique */
	CTRY_MYANMAR	= 104,	/* Myanmar */
	CTRY_NAMIBIA	= 516,	/* Namibia */
	CTRY_NAURU	= 520,	/* Nauru */
	CTRY_NEPAL	= 524,	/* Nepal */
	CTRY_NETHERLANDS	= 528,	/* Netherlands (the) */
	CTRY_NEW_CALEDONIA	= 540,	/* New Caledonia */
	CTRY_NEW_ZEALAND	= 554,	/* New Zealand */
	CTRY_NICARAGUA	= 558,	/* Nicaragua */
	CTRY_NIGER	= 562,	/* Niger (the) */
	CTRY_NIGERIA	= 566,	/* Nigeria */
	CTRY_NIUE	= 570,	/* Niue */
	CTRY_NORFOLK_ISLAND	= 574,	/* Norfolk Island */
	CTRY_NORTHERN_MARIANA_ISLANDS	= 580,	/* Northern Mariana Islands (the) */
	CTRY_NORWAY	= 578,	/* Norway */
	CTRY_OMAN	= 512,	/* Oman */
	CTRY_PAKISTAN	= 586,	/* Pakistan */
	CTRY_PALAU	= 585,	/* Palau */
	CTRY_PALESTINE	= 275,	/* Palestine, State of */
	CTRY_PANAMA	= 591,	/* Panama */
	CTRY_PAPUA_NEW_GUINEA	= 598,	/* Papua New Guinea */
	CTRY_PARAGUAY	= 600,	/* Paraguay */
	CTRY_PERU	= 604,	/* Peru */
	CTRY_PHILIPPINES	= 608,	/* Philippines (the) */
	CTRY_PITCAIRN	= 612,	/* Pitcairn */
	CTRY_POLAND	= 616,	/* Poland */
	CTRY_PORTUGAL	= 620,	/* Portugal */
	CTRY_PUERTO_RICO	= 630,	/* Puerto Rico */
	CTRY_QATAR	= 634,	/* Qatar */
	CTRY_REUNION	= 638,	/* Réunion */
	CTRY_ROMANIA	= 642,	/* Romania */
	CTRY_RUSSIA	= 643,	/* Russian Federation (the) */
	CTRY_RWANDA	= 646,	/* Rwanda */
	CTRY_SAINT_BARTHELEMY	= 652,	/* Saint Barthélemy */
	CTRY_SAINT_HELENA_ASCENSION_AND_TRISTAN_DA_CUNHA	= 654,
	CTRY_SAINT_KITTS_AND_NEVIS	= 659,	/* Saint Kitts and Nevis */
	CTRY_SAINT_LUCIA	= 662,	/* Saint Lucia */
	CTRY_SAINT_MARTIN	= 663,	/* Saint Martin (French part) */
	CTRY_SAINT_PIERRE_AND_MIQUELON	= 666,	/* Saint Pierre and Miquelon */
	CTRY_SAINT_VINCENT_AND_THE_GRENADINES	= 670,	/* Saint Vincent and the Grenadines */
	CTRY_SAMOA	= 882,	/* Samoa */
	CTRY_SAN_MARINO	= 674,	/* San Marino */
	CTRY_SAO_TOME_AND_PRINCIPE	= 678,	/* Sao Tome and Principe */
	CTRY_SAUDI_ARABIA	= 682,	/* Saudi Arabia */
	CTRY_SENEGAL	= 686,	/* Senegal */
	CTRY_SERBIA	= 688,	/* Serbia */
	CTRY_SEYCHELLES	= 690,	/* Seychelles */
	CTRY_SIERRA_LEONE	= 694,	/* Sierra Leone */
	CTRY_SINGAPORE	= 702,	/* Singapore */
	CTRY_SINT_MAARTEN	= 534,	/* Sint Maarten (Dutch part) */
	CTRY_SLOVAKIA	= 703,	/* Slovakia */
	CTRY_SLOVENIA	= 705,	/* Slovenia */
	CTRY_SOLOMON_ISLANDS	= 90,	/* Solomon Islands */
	CTRY_SOMALIA	= 706,	/* Somalia */
	CTRY_SOUTH_AFRICA	= 710,	/* South Africa */
	CTRY_SOUTH_GEORGIA_AND_THE_SOUTH_SANDWICH_ISLANDS	= 239,
	CTRY_SOUTH_SUDAN	= 728,	/* South Sudan */
	CTRY_SPAIN	= 724,	/* Spain */
	CTRY_SRILANKA	= 144,	/* Sri Lanka */
	CTRY_SUDAN	= 729,	/* Sudan (the) */
	CTRY_SURINAME	= 740,	/* Suriname */
	CTRY_SVALBARD_AND_JAN_MAYEN	= 744,	/* Svalbard and Jan Mayen */
	CTRY_SWEDEN	= 752,	/* Sweden */
	CTRY_SWITZERLAND	= 756,	/* Switzerland */
	CTRY_SYRIA	= 760,	/* Syrian Arab Republic */
	CTRY_TAIWAN	= 158,	/* Taiwan (Province of China) */
	CTRY_TAJIKISTAN	= 762,	/* Tajikistan */
	CTRY_TANZANIA	= 834,	/* Tanzania, United Republic of */
	CTRY_THAILAND	= 764,	/* Thailand */
	CTRY_TIMOR_LESTE	= 626,	/* Timor-Leste */
	CTRY_TOGO	= 768,	/* Togo */
	CTRY_TOKELAU	= 772,	/* Tokelau */
	CTRY_TONGA	= 776,	/* Tonga */
	CTRY_TRINIDAD_Y_TOBAGO	= 780,	/* Trinidad and Tobago */
	CTRY_TUNISIA	= 788,	/* Tunisia */
	CTRY_TURKEY	= 792,	/* Turkey */
	CTRY_TURKMENISTAN	= 795,	/* Turkmenistan */
	CTRY_TURKS_AND_CAICOS_ISLANDS	= 796,	/* Turks and Caicos Islands (the) */
	CTRY_TUVALU	= 798,	/* Tuvalu */
	CTRY_UAE	= 784,	/* United Arab Emirates (the) */
	CTRY_UGANDA	= 800,	/* Uganda */
	CTRY_UKRAINE	= 804,	/* Ukraine */
	CTRY_UNITED_KINGDOM	= 826,
	CTRY_UNITED_STATES	= 840,	/* United States of America (the) */
	CTRY_UNITED_STATES_FCC49	= 842,	/* United States (Public Safety) */
	CTRY_UNITED_STATES_MINOR_OUTLYING_ISLANDS	= 581,
	CTRY_URUGUAY	= 858,	/* Uruguay */
	CTRY_UZBEKISTAN	= 860,	/* Uzbekistan */
	CTRY_VANUATU	= 548,	/* Vanuatu */
	CTRY_VENEZUELA	= 862,	/* Venezuela (Bolivarian Republic of) */
	CTRY_VIET_NAM	= 704,	/* Viet Nam */
	CTRY_VIRGIN_ISLANDS_BRITISH	= 92,	/* Virgin Islands (British) */
	CTRY_VIRGIN_ISLANDS_US	= 850,	/* Virgin Islands (U.S.) */
	CTRY_WALLIS_AND_FUTUNA	= 876,	/* Wallis and Futuna */
	CTRY_WESTERN_SAHARA	= 732,	/* Western Sahara (Provisional name) */
	CTRY_YEMEN	= 887,	/* Yemen */
	CTRY_ZAMBIA	= 894,	/* Zambia */
	CTRY_ZIMBABWE	= 716	/* Zimbabwe */
};


#define IS_IN_EU(cc)   ( (cc) == CTRY_EUROPE || (cc) == CTRY_AUSTRIA || (cc) == CTRY_BULGARIA || \
			(cc) == CTRY_BELGIUM || (cc) == CTRY_CZECH || (cc) == CTRY_DENMARK || \
			(cc) == CTRY_ESTONIA || (cc) == CTRY_FINLAND || (cc) == CTRY_FRANCE || \
			(cc) == CTRY_FRANCE2 || (cc) == CTRY_GERMANY || (cc) == CTRY_GREECE || \
			(cc) == CTRY_HUNGARY || (cc) == CTRY_IRELAND || (cc) == CTRY_ITALY || \
			(cc) == CTRY_LATVIA || (cc) == CTRY_LITHUANIA || (cc) == CTRY_LUXEMBOURG || \
			(cc) == CTRY_NETHERLANDS || (cc) == CTRY_POLAND || (cc) == CTRY_PORTUGAL || \
			(cc) == CTRY_ROMANIA || (cc) == CTRY_SLOVAKIA || (cc) == CTRY_SLOVENIA || \
			(cc) == CTRY_SPAIN )


#define IEEE80211_IE_HDRLEN 2
#define IEEE80211_IE_TOTLEN(_ie) (IEEE80211_IE_HDRLEN + (_ie)[1])
#define IEEE80211_IE_LEN(_ie) ((_ie)[1])

/*
 * Generic information element
 */
struct ieee80211_ie {
	uint8_t id;
	uint8_t len;
	uint8_t info[0];
} __packed;

struct ieee80211_ie_ext {
	uint8_t id;
	uint8_t len;
	uint8_t ext_id;
	uint8_t info[0];
} __packed;

struct ieee80211_rsn_xe {
	uint8_t id;
	uint8_t len;
	uint8_t info[1];
} __packed;
/*
 * Country information element.
 */
#define IEEE80211_COUNTRY_MAX_TRIPLETS (83)
struct ieee80211_ie_country {
	uint8_t country_id;
	uint8_t country_len;
	uint8_t country_str[3];
	uint8_t country_triplet[IEEE80211_COUNTRY_MAX_TRIPLETS * 3];
} __packed;

#define MAX_BSSID_INDICATOR	QTN_MAX_BSS_VAPS
struct ieee80211_non_trans_cap {
	uint8_t id;
	uint8_t len;
	uint8_t cap[2];
} __packed;

struct ieee80211_non_trans_dtim {
	uint8_t id;
	uint8_t len;
	uint8_t bssid_index;
	uint8_t dtim_period;
	uint8_t dtim_count;
} __packed;

/* Nontransmitted BSSID profile includes below IEs at least:
 * Nontransmitted BSSID Capability
 * SSID(IEEE80211_NWID_LEN)
 * Nontransmitted BSSID index
 */
#define MAX_BSSID_OPTI_SUB_IE_LEN	43
struct ieee80211_ie_multi_bssid_sub {
	uint8_t sub_id;
	uint8_t sub_len;
	uint8_t sub_info[MAX_BSSID_OPTI_SUB_IE_LEN];
} __packed;

struct ieee80211_ie_multi_bssid {
	uint8_t mBSSID_id;
	uint8_t mBSSID_len;
	uint8_t max_mBSSID_ind;
	struct ieee80211_ie_multi_bssid_sub sub_ie;
} __packed;

/*
 * Channel Switch Announcement information element.
 */
struct ieee80211_ie_csa {
	uint8_t csa_id;	/* IEEE80211_ELEMID_CHANSWITCHANN */
	uint8_t csa_len;	/* == 3 */
	uint8_t csa_mode;	/* Channel Switch Mode: 1 == stop transmission until CS */
	uint8_t csa_chan;	/* New Channel Number */
	uint8_t csa_count;	/* TBTTs until Channel Switch happens */
} __packed;

/* for Spectrum Management Actions. Table 20e in 802.11h $7.4.1 */
#define IEEE80211_ACTION_S_MEASUREMENT_REQUEST 0
#define IEEE80211_ACTION_S_MEASUREMENT_REPORT  1
#define IEEE80211_ACTION_S_TPC_REQUEST         2
#define IEEE80211_ACTION_S_TPC_REPORT          3
#define IEEE80211_ACTION_S_CHANSWITCHANN       4

/* for csa_mode. It must be either 0 or 1. 1 means that the receiver shall stop
 * sending until CS. 0 imposes no requirement. See 7.3.2.20 */
#define IEEE80211_CSA_CAN_STOP_TX       0
#define IEEE80211_CSA_MUST_STOP_TX      1

/* minimal Channel Switch Count in the initial announcement */
#define IEEE80211_CSA_PROTECTION_PERIOD 3

/* maximum allowed deviance of measurement of intervals between CSA in Beacons */
#define IEEE80211_CSA_SANITY_THRESHOLD 100

/* Quantenna CSA tsf ie, to complement an 802.11h CSA ie. More timing precision */
struct ieee80211_ie_qtn_csa_tsf {
	uint8_t id;	/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;   /* length in bytes */
	uint8_t qtn_ie_oui[3];		/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t qtn_ie_type;		/* IE type */
	uint64_t tsf;			/* TSF at which channel change happens. */
} __packed;

/* Quantenna SCS IE */
#define QTN_SCS_IE_TYPE_STA_INTF_RPT		0x1
#define QTN_SCS_IE_TYPE_STA_DFS_RPT		0x2
#define QTN_SCS_IE_TYPE_STA_FAT_RPT		0x3
#define QTN_SCS_IE_TYPE_STA_TRFC_RPT		0x4
struct ieee80211_ie_qtn_scs {
	uint8_t id;			/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;                    /* length in bytes */
	uint8_t qtn_ie_oui[3];		/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t qtn_ie_type;		/* IE type */
	uint8_t scs_ie_type;            /* for future expansion and backward compatibility */
	/* following depends on scs_ie_type */
	union {
		struct {
			uint32_t sp_fail;		/* short preamble failure in last second */
			uint32_t lp_fail;		/* long preamble failure in last second */
			uint16_t others_time;		/* rx + tx time for all nodes */
		} cca_info;
		struct {
			uint16_t free_airtime;		/* free air time */
		} fat_info;
		struct {
			uint16_t cca_tx;		/* cca_tx */
			uint16_t cca_rx;		/* cca_rx */
			uint16_t cca_intf;		/* cca_intf */
			uint16_t cca_idle;		/* cca_idle */
		} trfc_info;
		struct {
			uint16_t dfs_enabled;		/* whether station's DFS feature enabled */
			uint8_t max_txpower;		/* station's tx power */
		} dfs_info;
	} u;
	/* Warning: using this variable length field would cause backward compatibility issue
	 * in future if want to add new fields */
	uint16_t extra_ie_len;		/* extra ie len */
	uint8_t extra_ie[0];		/* tdls stats */
}__packed;
#define QTN_SCS_IE_LEN_MIN			7    /* till scs ie type */
#define QTN_SCS_IE_STA_INTF_RPT_LEN_MIN		(QTN_SCS_IE_LEN_MIN + 8)
#define QTN_SCS_IE_STA_DFS_RPT_LEN_MIN		(QTN_SCS_IE_LEN_MIN + 3)
#define QTN_SCS_IE_STA_FAT_RPT_LEN_MIN		(QTN_SCS_IE_LEN_MIN + 2)
#define QTN_SCS_IE_STA_TRFC_RPT_LEN_MIN		(QTN_SCS_IE_LEN_MIN + 8)

#define IEEE80211_IS_ALL_SET(__flags__, __msb__)	\
	(((__flags__) & ((1 << ((__msb__)+1)) - 1)) == ((1 << ((__msb__)+1)) - 1))

/* does frame have QoS sequence control data */
#define	IEEE80211_QOS_HAS_SEQ(wh) \
	(((wh)->i_fc[0] & \
	  (IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_QOS)) == \
	  (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS))


#if defined(__LITTLE_ENDIAN_BITFIELD)
struct ieee80211_ie_qos_info {
	uint8_t	qi_update_cnt	: 4,
		qi_q_ack	: 1,
		qi_queue_req	: 1,
		qi_txop_req	: 1,
		qi_more_data_ack: 1;
} __packed;

struct ieee80211_ie_mu_edca_ac_param {
	uint8_t	acp_aifsn	: 4,
		acp_acm		: 1,
		acp_aci		: 2,
		reserved	: 1;
	uint8_t	acp_ecwmin	: 4,
		acp_ecwmax	: 4;
	uint8_t acp_mu_edca_timer;
} __packed;

#elif defined(__BIG_ENDIAN_BITFIELD)
struct ieee80211_ie_qos_info {
	uint8_t	qi_more_data_ack: 1;
		qi_txop_req	: 1,
		qi_queue_req	: 1,
		qi_q_ack	: 1,
		qi_update_cnt	: 4,
} __packed;

struct ieee80211_ie_mu_edca_ac_param {
	uint8_t	reserved	: 1;
		acp_aci		: 2,
		acp_acm		: 1,
		acp_aifsn	: 4,
	uint8_t	acp_ecwmax	: 4,
		acp_ecwmin	: 4;
	uint8_t acp_mu_edca_timer;

} __packed;

#endif

#if defined(__LITTLE_ENDIAN_BITFIELD) || defined(__BIG_ENDIAN_BITFIELD)
struct ieee80211_ie_mu_edca {
	struct ieee80211_ie_ext			header;
	struct ieee80211_ie_qos_info		mu_edca_qos_info;
	struct ieee80211_ie_mu_edca_ac_param	mu_edca_ac[WME_AC_NUM];
} __packed;
#endif

struct ieee80211_qos_info {
	uint8_t qi_update_cnt;
	uint8_t qi_queue_req;
};

struct ieee80211_mu_edca_ac_param {
	uint8_t acp_aifsn;
	uint8_t acp_ecwmin;
	uint8_t acp_ecwmax;
	uint8_t acp_mu_edca_timer;
};

struct ieee80211_mu_edca {
	struct ieee80211_qos_info		mu_edca_qos_info;
	struct ieee80211_mu_edca_ac_param	mu_edca_ac[WME_AC_NUM];
};

#define IEEE80211_MU_EDCA_TIMER_DEFAULT		255	/* in 8 TUs */
#define IEEE80211_MU_EDCA_TIMER_TU_PER_UNIT	8

static __inline__ uint32_t ieee80211_mu_edca_timer_to_tu(uint32_t mu_edca_timer)
{
	return mu_edca_timer * IEEE80211_MU_EDCA_TIMER_TU_PER_UNIT;
}

static __inline__ uint32_t ieee80211_mu_edca_timer_to_us(uint32_t mu_edca_timer)
{
	return IEEE80211_TU_TO_USEC(ieee80211_mu_edca_timer_to_tu(mu_edca_timer));
}

static __inline__ uint32_t ieee80211_mu_edca_timer_from_us(uint32_t us)
{
	return IEEE80211_USEC_TO_TU(us) / IEEE80211_MU_EDCA_TIMER_TU_PER_UNIT;
}

/*
 * for WMM Actions. Table 8 in WFA WMM Spec
 */
#define IEEE80211_ACTION_WMM_ADDTS_REQ		0
#define IEEE80211_ACTION_WMM_ADDTS_RESP		1
#define IEEE80211_ACTION_WMM_DELTS		2

/*
 * WME/802.11e information element.
 */
struct ieee80211_ie_wme {
	uint8_t wme_id;		/* IEEE80211_ELEMID_VENDOR */
	uint8_t wme_len;	/* length in bytes */
	uint8_t wme_oui[3];	/* 0x00, 0x50, 0xf2 */
	uint8_t wme_type;	/* OUI type */
	uint8_t wme_subtype;	/* OUI subtype */
	uint8_t wme_version;	/* spec revision */
	uint8_t wme_info;	/* QoS info */
} __packed;

/*
 * WME/802.11e ADD/DEL TS Action Frame
 */
enum ieee80211_ts_action_status {
	IEEE80211_TS_ACTION_S_ACCEPTED		= 0,
	IEEE80211_TS_ACTION_S_INVALID		= 1,
	IEEE80211_TS_ACTION_S_REFUSED		= 3,
};

struct ieee80211_action_add_del_ts {
	uint8_t ia_category;
	uint8_t ia_action;
	uint8_t token;
	uint8_t status;
	uint8_t data[0];
}__packed;

/*
 * WME/802.11e Tspec Element
 */
#define WME_TSPEC_TS_INFO_DIRECTION_UP		0
#define WME_TSPEC_TS_INFO_DIRECTION_DOWN	1
#define WME_TSPEC_TS_INFO_DIRECTION_BI		3

#define WME_TSPEC_TS_NOM_MSDU_SIZE_MASK		0x7FFF
#define WME_TSPEC_TS_NOM_MSDU_FIX_MASK		0x8000
#define WME_TSPEC_TS_BW_SURPLUS_INTEGER		0xE000
#define WME_TSPEC_TS_BW_SURPLUS_INTEGER_S	13
#define WME_TSPEC_TS_BW_SURPLUS_DECIMAL_MASK	0x1FFF
#define WME_TSPEC_TS_MEDIUM_TIME_UNIT_US	32
#define WME_TSPEC_TS_MEDIUM_TIME_FULLNESS	(1000000 / 32)

struct ieee80211_wme_tspec_ts_info {
	uint8_t ts_info_type		: 1,
		ts_info_tid		: 4,
		ts_info_direction	: 2,
		ts_info_rsvd0		: 1;
	uint8_t ts_info_rsvd1		: 2,
		ts_info_psb		: 1,
		ts_info_up		: 3,
		ts_info_rsvd2		: 2;
	uint8_t ts_info_rsvd3;
} __packed;

struct ieee80211_wme_tspec {
	uint8_t ts_id;
	uint8_t ts_len;
	uint8_t ts_oui[3];
	uint8_t ts_oui_type;
	uint8_t ts_oui_subtype;
	uint8_t ts_version;
	struct ieee80211_wme_tspec_ts_info ts_tsinfo;
	uint8_t ts_nom_msdu[2];
	uint8_t ts_max_msdu[2];
	uint8_t ts_min_svc[4];
	uint8_t ts_max_svc[4];
	uint8_t ts_inactv_intv[4];
	uint8_t ts_susp_intv[4];
	uint8_t ts_start_svc[4];
	uint8_t ts_min_rate[4];
	uint8_t ts_mean_rate[4];
	uint8_t ts_peak_rate[4];
	uint8_t ts_max_burst[4];
	uint8_t ts_delay[4];
	uint8_t ts_min_phy[4];
	uint8_t ts_surplus[2];
	uint8_t ts_medium_time[2];
} __packed;

/*
 * WME AC parameter field
 */

struct ieee80211_wme_acparams {
	uint8_t acp_aci_aifsn;
	uint8_t acp_logcwminmax;
	uint16_t acp_txop;
} __packed;

#define IEEE80211_WME_PARAM_LEN	24

#define WME_NUM_TID		8

#define WME_NUM_AC		4	/* 4 AC categories */
#define WME_TID_UNKNOWN		(-1)
#define WME_TID_NONQOS		(-2)
#define WME_TID_VALID(_tid)	(((_tid) >= 0) && ((_tid) < WME_NUM_TID))
#define WME_TID_VALID_U(_tid)	((_tid) < WME_NUM_TID)

#define WME_PARAM_ACI		0x60	/* Mask for ACI field */
#define WME_PARAM_ACI_S		5	/* Shift for ACI field */
#define WME_PARAM_ACM		0x10	/* Mask for ACM bit */
#define WME_PARAM_ACM_S		4	/* Shift for ACM bit */
#define WME_PARAM_AIFSN		0x0f	/* Mask for aifsn field */
#define WME_PARAM_AIFSN_S	0	/* Shift for aifsn field */
#define WME_PARAM_LOGCWMIN	0x0f	/* Mask for CwMin field (in log) */
#define WME_PARAM_LOGCWMIN_S	0	/* Shift for CwMin field */
#define WME_PARAM_LOGCWMAX	0xf0	/* Mask for CwMax field (in log) */
#define WME_PARAM_LOGCWMAX_S	4	/* Shift for CwMax field */

/* EDCA tx oplimit in 32us units can be programmed in 12 bits */
#define WME_TXOP_MAX		IEEE80211_TXOP_TO_US(0xFFF)

#define WME_AC_TO_TID(_ac) (       \
	((_ac) == WME_AC_VO) ? 6 : \
	((_ac) == WME_AC_VI) ? 5 : \
	((_ac) == WME_AC_BK) ? 1 : \
	0)

#define TID_TO_WME_AC(_tid)				\
	((((_tid) == 0) || ((_tid) == 3)) ? WME_AC_BE :	\
	 ((_tid) < 3) ? WME_AC_BK :	\
	 ((_tid) < 6) ? WME_AC_VI :	\
	 WME_AC_VO)

/*
 * WME Parameter Element
 */
struct ieee80211_wme_param {
	uint8_t param_id;
	uint8_t param_len;
	uint8_t param_oui[3];
	uint8_t param_oui_type;
	uint8_t param_oui_sybtype;
	uint8_t param_version;
	uint8_t param_qosInfo;
	uint8_t param_reserved;
	struct ieee80211_wme_acparams	params_acParams[WME_NUM_AC];
} __packed;

/*
 * WME U-APSD qos info field defines
 */
#define WME_CAPINFO_UAPSD_EN			0x00000080
#define WME_CAPINFO_UAPSD_VO			0x00000001
#define WME_CAPINFO_UAPSD_VI			0x00000002
#define WME_CAPINFO_UAPSD_BK			0x00000004
#define WME_CAPINFO_UAPSD_BE			0x00000008
#define WME_CAPINFO_UAPSD_ACFLAGS_SHIFT		0
#define WME_CAPINFO_UAPSD_ACFLAGS_MASK		0xF
#define WME_CAPINFO_UAPSD_MAXSP_SHIFT		5
#define WME_CAPINFO_UAPSD_MAXSP_MASK		0x3
#define WME_CAPINFO_IE_OFFSET			8
#define WME_UAPSD_MAXSP(_qosinfo) (((_qosinfo) >> WME_CAPINFO_UAPSD_MAXSP_SHIFT) & WME_CAPINFO_UAPSD_MAXSP_MASK)
#define WME_UAPSD_AC_ENABLED(_ac, _qosinfo) ( (1<<(3 - (_ac))) &   \
		(((_qosinfo) >> WME_CAPINFO_UAPSD_ACFLAGS_SHIFT) & WME_CAPINFO_UAPSD_ACFLAGS_MASK) )

#define IEEE80211_EXTCAP_IE_LEN 11
struct ieee80211_extcap_param {
	u_int8_t param_id;
	u_int8_t param_len;
	u_int8_t ext_cap[IEEE80211_EXTCAP_IE_LEN];
} __packed;


#define IEEE8021P_PRIORITY_NUM			8
#define IEEE80211_DSCP_MAX_EXCEPTIONS		21
#define IP_DSCP_NUM				64

/* Extended capability IE table 9-135 at sepc 802.11-2016 */
#define IEEE80211_EXTCAP_20_40_COEXIST		0
#define IEEE80211_EXTCAP_EVENT			7
#define IEEE80211_EXTCAP_DIAG			8
#define IEEE80211_EXTCAP_MULTI_DIAG		9
#define IEEE80211_EXTCAP_LOC_TRACK		10
#define IEEE80211_EXTCAP_COLL_INTF_RPT		13
#define IEEE80211_EXTCAP_CIVIC_LOC		14
#define IEEE80211_EXTCAP_GOE_LOC		15
#define IEEE80211_EXTCAP_BTM			19
#define IEEE80211_EXTCAP_QOS_TRF_CAP		20
#define IEEE80211_EXTCAP_AC_STA_CNT		21
#define IEEE80211_EXTCAP_MULTI_BSSID		22
#define IEEE80211_EXTCAP_UTC_TSF_OFF		27
#define IEEE80211_EXTCAP_INTERWORKING		31
#define IEEE80211_EXTCAP_QOS_MAP		32
#define IEEE80211_EXTCAP_ID_LOC			44
#define IEEE80211_EXTCAP_WNM_NOTIF		46
#define IEEE80211_EXTCAP_OPMODE_NOTIF		62
#define IEEE80211_EXTCAP_MAX_MSDU_IN_AMSDU_L	63
#define IEEE80211_EXTCAP_MAX_MSDU_IN_AMSDU_H	64
#define IEEE80211_EXTCAP_FUT_CHAN_GUIDE		74
#define IEEE80211_EXTCAP_TWT_REQUESTER		77
#define IEEE80211_EXTCAP_TWT_RESPONDER		78
#define IEEE80211_EXTCAP_OBSS_NARROW_RU_TOL	79
#define IEEE80211_EXTCAP_COMPL_NON_TXBSSID	80

#define IEEE80211_EXTCAP_TO_BIT(_val)	(1 << ((_val) % NBBY))
#define IEEE80211_EXTCAP_TO_BYTE(_val)	((_val) / NBBY)

#define IEEE80211_EXTCAP_MAX_MSDU_IN_AMSDU_MASK         0x0180
#define IEEE80211_EXTCAP_MAX_MSDU_IN_AMSDU_MASK_S       7

/*
 * 20/40 MHZ BSS coexistence information element.
 */
struct ieee80211_20_40_coex_param {
	u_int8_t param_id;
	u_int8_t param_len;
	u_int8_t coex_param;
} __packed;

#define WLAN_20_40_BSS_COEX_INFO_REQ            BIT(0)
#define WLAN_20_40_BSS_COEX_40MHZ_INTOL         BIT(1)
#define WLAN_20_40_BSS_COEX_20MHZ_WIDTH_REQ     BIT(2)
#define WLAN_20_40_BSS_COEX_OBSS_EXEMPT_REQ     BIT(3)
#define WLAN_20_40_BSS_COEX_OBSS_EXEMPT_GRNT    BIT(4)

/*
 * 20/40 MHZ BSS intolerant channel report information element.
 */
struct ieee80211_20_40_in_ch_rep {
	u_int8_t param_id;
	u_int8_t param_len;
	u_int8_t reg;
	u_int8_t chan[0];
} __packed;

/*
 * Overlapping BSS Scan Parameter information element.
 */
struct ieee80211_obss_scan_ie {
	u_int8_t param_id;
	u_int8_t param_len;
	u_int16_t obss_passive_dwell;
	u_int16_t obss_active_dwell;
	u_int16_t obss_trigger_interval;
	u_int16_t obss_passive_total;
	u_int16_t obss_active_total;
	u_int16_t obss_channel_width_delay;
	u_int16_t obss_activity_threshold;
} __packed;

/*
 * Atheros Advanced Capability information element.
 */
struct ieee80211_ie_athAdvCap {
	uint8_t athAdvCap_id;		/* IEEE80211_ELEMID_VENDOR */
	uint8_t athAdvCap_len;		/* length in bytes */
	uint8_t athAdvCap_oui[3];	/* 0x00, 0x03, 0x7f */
	uint8_t athAdvCap_type;		/* OUI type */
	uint8_t athAdvCap_subtype;	/* OUI subtype */
	uint8_t athAdvCap_version;	/* spec revision */
	uint8_t athAdvCap_capability;	/* Capability info */
	uint8_t athAdvCap_defKeyIndex[2];
} __packed;

/*
 * Atheros XR information element.
 */
struct ieee80211_xr_param {
	uint8_t param_id;
	uint8_t param_len;
	uint8_t param_oui[3];
	uint8_t param_oui_type;
	uint8_t param_oui_sybtype;
	uint8_t param_version;
	uint8_t param_Info;
	uint8_t param_base_bssid[IEEE80211_ADDR_LEN];
	uint8_t param_xr_bssid[IEEE80211_ADDR_LEN];
	uint8_t param_xr_beacon_interval[2];
	uint8_t param_base_ath_capability;
	uint8_t param_xr_ath_capability;
} __packed;

/* Atheros capabilities */
#define IEEE80211_ATHC_TURBOP	0x0001		/* Turbo Prime */
#define IEEE80211_ATHC_COMP	0x0002		/* Compression */
#define IEEE80211_ATHC_FF	0x0004		/* Fast Frames */
#define IEEE80211_ATHC_XR	0x0008		/* Xtended Range support */
#define IEEE80211_ATHC_AR	0x0010		/* Advanced Radar support */
#define IEEE80211_ATHC_BURST	0x0020		/* Bursting - not negotiated */
#define IEEE80211_ATHC_WME	0x0040		/* CWMin tuning */
#define IEEE80211_ATHC_BOOST	0x0080		/* Boost */

/*
 * Quantenna Flags information element.
 * Fields up to qtn_ie_implicit_ba_tid are backwards-compatible with Envy images.
 */
struct ieee80211_ie_qtn {
	uint8_t qtn_ie_id;		/* IEEE80211_ELEMID_VENDOR */
	uint8_t qtn_ie_len;		/* length in bytes */
	uint8_t qtn_ie_oui[3];		/* QTN_OUI - 0x00, 0x26, 0x86 */
	uint8_t qtn_ie_type;		/* IE type */
	uint8_t qtn_ie_flags;		/* See below */

	/* V2 fields */
	uint8_t qtn_ie_implicit_ba_tid;/* Implicit block ACKs, set up directly after assoc */
	uint8_t qtn_ie_my_flags;	/* See below */

	/* V3 fields */
	/* Implicit block ACK with variable size - overrides v2 implicit BA field. */
	uint8_t qtn_ie_implicit_ba_tid_h;
	uint8_t qtn_ie_implicit_ba_size; /* Size of implicit BA >> 2 */

	/* V4 fields */
	uint8_t qtn_ie_vsp_version;	/* VSP version */

	/* V5 fields */
	uint32_t qtn_ie_ver_sw;
	uint16_t qtn_ie_ver_hw;
	uint16_t qtn_ie_ver_platform_id;
	uint32_t qtn_ie_ver_timestamp;
	/* Features advertised for QHop, even when not yet enabled */
#define QTN_IE_VER_FLAG_SW_WPA3_SAE	0x0001
#define QTN_IE_VER_FLAG_SW_WPA3_OWE	0x0002
#define QTN_IE_VER_FLAG_SW_WPA3_DPP	0x0004
	uint32_t qtn_ie_ver_flags;
	uint32_t qtn_ie_rate_train;

	/* V6 fields */
#define QTN_1024QAM_FLAGS_ENABLE	(1UL << 30)
	uint32_t qtn_ie_1024qam_flags; /* Bit[11:0] : MCS 251 ~ 262 . Bit[29:12] : Reserved*/
} __packed;

#define QTN_PAIRING_TLV_HASH_LEN 32
/*
 * QTN Pairing TLV element.
 *  Format:
 *   Type(1byte)   |   len(2bytes)    |  SHA-256 hash(32bytes)
 *      0x1        |        35        |     SHA-256 hash material of pairing
 */
struct ieee80211_ie_qtn_pairing_tlv {
	uint8_t qtn_pairing_tlv_type;
	uint16_t qtn_pairing_tlv_len;
	uint8_t qtn_pairing_tlv_hash[QTN_PAIRING_TLV_HASH_LEN];
} __packed;

/*
 * QTN Pairing IE
 *  Format:
 *  IE ID(1byte)    |     IE len(1byte)    |     IE OUI(3bytes)    | IE content(pairing)
 *     0xdd         |       38             |        00 26 86       |     Pairing TLV
 *
 */
struct ieee80211_ie_qtn_pairing {
	uint8_t qtn_pairing_ie_id;
	uint8_t qtn_pairing_ie_len;
	uint8_t qtn_pairing_ie_oui[3];
	struct ieee80211_ie_qtn_pairing_tlv qtn_pairing_tlv;
} __packed;

#define IEEE80211_QTN_IE_BA_SIZE_SH 2

enum ieee80211_vsp_version {
	IEEE80211_QTN_VSP_V_NONE,
	IEEE80211_QTN_VSP_V1,
};

#define IEEE80211_QTN_VSP_VERSION	IEEE80211_QTN_VSP_V_NONE

#define IEEE80211_QTN_TYPE_ENVY_LEGACY(qtnie) \
	((qtnie)->qtn_ie_len <= (&(qtnie)->qtn_ie_my_flags - &(qtnie)->qtn_ie_oui[0]))
#define IEEE80211_QTN_TYPE_ENVY(qtnie) \
	((IEEE80211_QTN_TYPE_ENVY_LEGACY(qtnie)) || \
	 ((qtnie)->qtn_ie_my_flags & IEEE80211_QTN_ENVY))

#define IEEE80211_QTN_FLAGS_ENVY	(IEEE80211_QTN_BRIDGEMODE | IEEE80211_QTN_BF_VER1)
#define IEEE80211_QTN_FLAGS_ENVY_DFLT	IEEE80211_QTN_BF_VER1
#define IEEE80211_QTN_CAPS_DFLT		IEEE80211_QTN_BF_VER2 | IEEE80211_QTN_BF_VER3 | \
					IEEE80211_QTN_BF_VER4 | IEEE80211_QTN_TX_AMSDU
/*
 * These flags are used in the following two fields.
 * - qtn_ie_flags contains the sender's settings, except in an association response, where
 *   it contains confirmation of the settings received from the peer station.  These flags
 *   must remain backwards-compatible with Envy images.
 * - qtn_ie_my_flags always contains the sender's settings.  It is not sent by Envy systems.
 */
#define IEEE80211_QTN_BRIDGEMODE	0x01		/* Use 4-addr headers */
#define IEEE80211_QTN_BF_VER1		0x02		/* Envy beamforming */
#define IEEE80211_QTN_BF_VER2		0x04		/* Ruby 2 stream beamforming */
#define IEEE80211_QTN_LNCB		0x08		/* Multicast packets in the local network
							 * control block are 4 address encapsulated.
							 */
#define IEEE80211_QTN_BF_VER3		0x10		/* Ruby 4 stream non-standard beamforming */
#define IEEE80211_QTN_ENVY		0x20		/* Envy with 'my flags' field in the IE. */
#define IEEE80211_QTN_BF_VER4		0x40		/* 4 strm standard bf with tone grouping */
#define IEEE80211_QTN_TX_AMSDU		0x80		/* Ruby TX AMSDU */

#define IEEE80211_QTN_IE_GE_V2(_qtnie)	((_qtnie->qtn_ie_len + IEEE80211_IE_HDRLEN) >	\
					offsetof(struct ieee80211_ie_qtn, qtn_ie_my_flags))
#define IEEE80211_QTN_IE_GE_V3(_qtnie)	((_qtnie->qtn_ie_len + IEEE80211_IE_HDRLEN) >	\
					offsetof(struct ieee80211_ie_qtn, qtn_ie_implicit_ba_size))
#define IEEE80211_QTN_IE_GE_V4(_qtnie)	((_qtnie->qtn_ie_len + IEEE80211_IE_HDRLEN) >	\
					offsetof(struct ieee80211_ie_qtn, qtn_ie_vsp_version))
#define IEEE80211_QTN_IE_GE_V5(_qtnie)	((_qtnie->qtn_ie_len + IEEE80211_IE_HDRLEN) >	\
					offsetof(struct ieee80211_ie_qtn, qtn_ie_rate_train))

#define IEEE80211_QTN_IE_GE_V6(_qtnie)	((_qtnie->qtn_ie_len + IEEE80211_IE_HDRLEN) >	\
					offsetof(struct ieee80211_ie_qtn, qtn_ie_1024qam_flags))

/*
 * Management Notification Frame
 */
struct ieee80211_mnf {
	uint8_t mnf_category;
	uint8_t mnf_action;
	uint8_t mnf_dialog;
	uint8_t mnf_status;
} __packed;
#define	MNF_SETUP_REQ	0
#define	MNF_SETUP_RESP	1
#define	MNF_TEARDOWN	2

/*
 * Management Action Frames
 */

/* generic frame format */
struct ieee80211_action {
	uint8_t	ia_category;
	uint8_t	ia_action;
} __packed;

/* FILS Discovery info field */
struct ieee80211_action_fils_disc_info {
	uint16_t frame_ctrl;
	uint64_t time_stamp;
	uint16_t beacon_interval;
	uint8_t ssid[IEEE80211_NWID_LEN];
	/*
	 * uint8_t length;
	 * uint16_t fd_caps;
	 * uint8_t oper_class;
	 * uint8_t pri_chan;
	 * uint8_t ap_cfg_seq_no;
	 * uint8_t access_net_opt;
	 * uint8_t fd_rsn_info[5];
	 * uint8_t chan_cntr_freq_seg1;
	 * uint8_t mobility_domain[3];
	 */
} __packed;

#define IEEE80211_FILS_DISC_IE_FRM_CTRL_SSID		0x1F	/* SSID length */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_SSID_S		0
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_CAP		0x1	/* Cap presence indicator */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_CAP_S		5
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_S_SSID		0x1	/* Short SSID Indicator */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_S_SSID_S	6
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_AP_CSN		0x1	/* AP-CSN presence */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_AP_CSN_S	7
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_ANO		0x1	/* ANO presenece */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_ANO_S		8
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_CFREQ_SEG1	0x1	/* Chan cntr freq seg1 presence*/
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_CFREQ_SEG1_S	9
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_PRI_CHAN	0x1	/* Primary chan presence */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_PRI_CHAN_S	10
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_RSN		0x1	/* RSN info presence */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_RSN_S		11
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_LEN		0x1	/* Length presence */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_LEN_S		12
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_MD		0x1	/* Mobility domain presence */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_MD_S		13
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_RES		0x3	/* Reserved */
#define IEEE80211_FILS_DISC_IE_FRM_CTRL_RES_S		14

#define IEEE80211_ACT_SIZE			(sizeof(struct ieee80211_action))
#define IEEE80211_ACT_SIZE_FILS_DISC	\
			(sizeof(struct ieee80211_frame) + \
			 sizeof(struct ieee80211_action) + \
			 sizeof(struct ieee80211_action_fils_disc_info) + \
			 sizeof(struct ieee80211_ie_tpe))

/* categories */
#define IEEE80211_ACTION_CAT_SPEC_MGMT		0	/* Spectrum MGMT */
#define IEEE80211_ACTION_CAT_QOS		1	/* qos */
#define IEEE80211_ACTION_CAT_DLS		2	/* dls */
#define IEEE80211_ACTION_CAT_BA			3	/* block ack */
#define IEEE80211_ACTION_CAT_PUBLIC		4	/* Public */
#define IEEE80211_ACTION_CAT_RM			5	/* Radio measurement */
#define IEEE80211_ACTION_CAT_FBSS		6	/* Fast BSS */
#define IEEE80211_ACTION_CAT_HT			7	/* HT */
#define IEEE80211_ACTION_CAT_SA_QUERY		8	/* SA Query */
#define IEEE80211_ACTION_CAT_PROT_DUAL_PA	9	/* Protected Dual of Public Action */
#define IEEE80211_ACTION_CAT_WNM		10	/* WNM */
#define IEEE80211_ACTION_CAT_UNPROT_WNM		11	/* Unprotected WNM */
#define IEEE80211_ACTION_CAT_TDLS		12	/* TDLS */
#define IEEE80211_ACTION_CAT_MESH		13	/* Mesh */
#define IEEE80211_ACTION_CAT_MULTIHOP		14	/* Multihop */
#define IEEE80211_ACTION_CAT_SELF_PROT		15	/* self protected */
#define IEEE80211_ACTION_CAT_WFA                17      /* reserved to be used by WFA */

#define IEEE80211_ACTION_CAT_VHT		21	/* VHT */
#define IEEE80211_ACTION_CAT_S1G		22	/* 802.11ah-2016: 9.6.25 S1G Action frame */
#define IEEE80211_ACTION_CAT_HE			30	/* HE */
#define IEEE80211_ACTION_CAT_HE_PROT		31	/* Protected HE */
#define IEEE80211_ACTION_CAT_VEND_PROT		126	/* Protected Vendor specific Action frame */
#define IEEE80211_ACTION_CAT_VENDOR		0x7F	/* Vendor specific Action frame */

/* Public Action Frames (7.4.7.1) */
#define IEEE80211_ACTION_PUB_20_40_COEX         0   /* 20/40 coex */
#define IEEE80211_ACTION_PUB_VENDOR_SPEC	9   /* Vendor specific Public action frame */
#define IEEE80211_ACTION_PUB_GAS_IREQ		10  /* GAS Service Initial Request */
#define IEEE80211_ACTION_PUB_GAS_IRESP		11  /* GAS Service Initial Response */
#define IEEE80211_ACTION_PUB_GAS_CREQ		12  /* GAS Comeback Request */
#define IEEE80211_ACTION_PUB_GAS_CRESP		13  /* GAS Comeback Response */
#define IEEE80211_ACTION_PUB_TDLS_DISC_RESP	14  /* TDLS Discovery Response */
#define IEEE80211_ACTION_PUB_FTM_REQ		32  /* FTM Request */
#define IEEE80211_ACTION_PUB_FTM		33  /* FTM */
#define IEEE80211_ACTION_PUB_FILS_DISC		34  /* FILS Discovery */

static __inline__ int ieee80211_action_is_a_gas(const struct ieee80211_action *ia)
{
	return (ia->ia_category == IEEE80211_ACTION_CAT_PUBLIC) &&
		(ia->ia_action >= IEEE80211_ACTION_PUB_GAS_IREQ) &&
		(ia->ia_action <= IEEE80211_ACTION_PUB_GAS_CRESP);
}

/* TDLS Action Frame details (7.4.11) */
#define IEEE80211_ACTION_TDLS_SETUP_REQ        0   /* Setup Request */
#define IEEE80211_ACTION_TDLS_SETUP_RESP       1   /* Setup Response */
#define IEEE80211_ACTION_TDLS_SETUP_CONFIRM    2   /* Setup Confirm */
#define IEEE80211_ACTION_TDLS_TEARDOWN         3   /* Teardown */
#define IEEE80211_ACTION_TDLS_PTI              4   /* Peer Traffic Indication */
#define IEEE80211_ACTION_TDLS_CS_REQ           5   /* Channel Switch Request */
#define IEEE80211_ACTION_TDLS_CS_RESP          6   /* Channel Switch Response */
#define IEEE80211_ACTION_TDLS_PEER_PSM_REQ     7   /* Peer PSM Request */
#define IEEE80211_ACTION_TDLS_PEER_PSM_RESP    8   /* Peer PSM Response */
#define IEEE80211_ACTION_TDLS_PEER_TRAF_RESP   9   /* Peer Traffic Response */
#define IEEE80211_ACTION_TDLS_DISC_REQ         10  /* Discovery Request */

/* FTM Parameters IE */
#define IEEE80211_TOA_LEN	6
struct ieee80211_toa {
	uint8_t time[IEEE80211_TOA_LEN];
} __packed;

#define IEEE80211_TOD_LEN	6
struct ieee80211_tod {
	uint8_t time[IEEE80211_TOD_LEN];
} __packed;


#define IEEE80211_IE_FTM_PARAM_LEN	9
struct ieee80211_ie_ftm_param {
	uint8_t id; /* IEEE80211_ELEMID_FTM_PARAM */
	uint8_t len; /* Length in bytes */

	uint8_t status;
	uint8_t burst_dur_num;
	uint8_t min_delta;
	uint16_t partial_tsf_timer;
	uint8_t flags;
	uint8_t format_bw;
	uint16_t burst_period;
} __packed;

/* FTM specific Public Action Frames */
struct ieee80211_action_ftm_req {
	struct ieee80211_action header;

	uint8_t trigger;
} __packed;

struct ieee80211_action_ftm {
	struct ieee80211_action header;

	uint8_t diag_tk;
	uint8_t diag_tk_follow;

	struct ieee80211_tod tod;
	struct ieee80211_toa toa;
	uint16_t tod_err;
	uint16_t toa_err;
} __packed;

/* TWT specific Action Frames (802.11ah-2016: Table 9-421b — Unprotected S1G Action field values) */
#define IEEE80211_ACTION_S1G_TWT_SETUP		6	/* TWT Setup */
#define IEEE80211_ACTION_S1G_TWT_TDOWN		7	/* TWT Teardown */
#define IEEE80211_ACTION_S1G_TWT_INFO		11	/* TWT Information */

#define IEEE80211_TWT_INFO_NEXTTWT_LEN_64	3
#define IEEE80211_TWT_INFO_NEXTTWT_LEN_48	2
#define IEEE80211_TWT_INFO_NEXTTWT_LEN_32	1

#define IEEE80211_TWT_INFO_NEXTTWT_LEN_32_MASK	0xFFFFFFFF00000000
#define IEEE80211_TWT_INFO_NEXTTWT_LEN_48_MASK	0xFFFFFFFFFF000000

/* Expected length of HE non-broadcast TWT IE (excluding and ie header) */
#define IEEE80211_TWT_IE_LEN			(sizeof(struct ieee80211_ie_twt) - 2)

#define IEEE80211_TWT_BCAST_SETUP_LEN_MAX	\
			(IEEE80211_ACT_SIZE + sizeof(struct ieee80211_ie_bcast_twt) + \
			QTN_TWT_BCAST_FLOWS_MAX * sizeof(struct ieee80211_bcast_twt_ie_params))

#define IEEE80211_TWT_IND_SETUP_LEN_MAX		\
			(IEEE80211_ACT_SIZE + sizeof(struct ieee80211_ie_twt))

#define IEEE80211_TWT_SETUP_LEN_MAX		\
			(MAX(IEEE80211_TWT_IND_SETUP_LEN_MAX, IEEE80211_TWT_BCAST_SETUP_LEN_MAX))

#define IEEE80211_TWT_WAKE_DUR_UNIT_US	256	/* Conversion unit for TWT Wake duration in us */

struct ieee80211_twt_info {
	uint8_t params;
	__le64 next_twt;
} __packed;

/* Individual TWT IE */
struct ieee80211_ie_twt {
	uint8_t id;			/* Element ID */
	uint8_t len;			/* Length in bytes */
	uint8_t control;		/* TWT IE Control Field */
	__le16 req_type;		/* Request Type Field */
	__le64 twt_tsf;			/* Target Wake Time TSF */
	uint8_t nm_wake_dur;		/* Nominal Minimum TWT Wake Duration */
	__le16 wake_int_mantissa;	/* TWT Wake Interval Mantissa */
	uint8_t twt_channel;
} __packed;

struct ieee80211_bcast_twt_ie_params {
	__le16 req_type;		/* Request Type Field */
	__le16 twt_tsf;			/* Broadcast Target Wake Time TSF */
	uint8_t nm_wake_dur;		/* Nominal Minimum TWT Wake Duration */
	__le16 wake_int_mantissa;	/* TWT Wake Interval Mantissa */
	__le16 bcast_twt_info;	/* Broadcast TWT Info Field */
} __packed;

/* Broadcast TWT IE */
struct ieee80211_ie_bcast_twt {
	uint8_t id;						/* Element ID */
	uint8_t len;						/* Length in bytes */
	uint8_t control;					/* TWT IE Control Field */
	struct ieee80211_bcast_twt_ie_params bcast_params[0];	/* Broadcast parameters per set */
} __packed;

struct ieee80211_action_twt_setup {
	struct ieee80211_action header;
	uint8_t token;
	struct ieee80211_ie_twt twt_ie;
} __packed;

struct ieee80211_action_twt_tdown {
	struct ieee80211_action header;
	uint8_t param;
} __packed;

struct ieee80211_action_twt_info {
	struct ieee80211_action header;
	struct ieee80211_twt_info twt_info;	/* TWT Information Field */
} __packed;

enum ieee80211_twt_info_cmd {
	IEEE80211_TWT_INFO_CMD_SUSPEND = 0,
	IEEE80211_TWT_INFO_CMD_RESUME = 1,
};

enum ieee80211_twt_command_is_req {
	IEEE80211_TWT_CMD_IS_RESPONSE = 0,
	IEEE80211_TWT_CMD_IS_REQUEST = 1,
};

/* Table 9-262k - TWT Setup Command field values */
enum ieee80211_twt_setup_cmd {
	IEEE80211_TWT_SETUP_CMD_REQUEST = 0,
	IEEE80211_TWT_SETUP_CMD_SUGGEST,
	IEEE80211_TWT_SETUP_CMD_DEMAND,
	IEEE80211_TWT_SETUP_CMD_GROUPING,
	IEEE80211_TWT_SETUP_CMD_ACCEPT,
	IEEE80211_TWT_SETUP_CMD_ALTERNATE,
	IEEE80211_TWT_SETUP_CMD_DICTATE,
	IEEE80211_TWT_SETUP_CMD_REJECT,
};

/* Table 9-262k1 - TWT Flow Identifier field for a broadcast TWT element */
enum ieee80211_twt_bcast_flow_id {
	IEEE80211_TWT_BCAST_FLOW_ID_ALL = 0,
	IEEE80211_TWT_BCAST_FLOW_ID_LIMITED_1,
	IEEE80211_TWT_BCAST_FLOW_ID_LIMITED_2,
	IEEE80211_TWT_BCAST_FLOW_ID_ALL_TIM,
};

/* IEEE802.11ax/D3.0 Table 9-262j1 - Negotiation Type Interpretation */
enum ieee80211_twt_nego_type {
	IEEE80211_TWT_NEGO_TYPE_INDIV_TWT = 0,
	IEEE80211_TWT_NEGO_TYPE_WAKE_TBTT,
	IEEE80211_TWT_NEGO_TYPE_BC_TWT_BCFRM,
	IEEE80211_TWT_NEGO_TYPE_BC_TWT_UCFRM,
};

/* TWT IE Control Field Bit Values */
#define IEEE80211_TWT_CTRL_PM_MODE		0x02
#define IEEE80211_TWT_CTRL_NEGO_TYPE		0x0c

/* TWT IE Request Type Bitfield Definitions */
#define IEEE80211_TWT_REQ_IS_REQ		0x0001
#define IEEE80211_TWT_REQ_CMD			0x000E
#define IEEE80211_TWT_REQ_TRIG			0x0010
#define IEEE80211_TWT_REQ_IMPLICIT		0x0020 /* Last parameter set for Broadcast TWT IE */
#define IEEE80211_TWT_REQ_FLOW_TYPE		0x0040
#define IEEE80211_TWT_REQ_FLOW_ID		0x0380
#define IEEE80211_TWT_REQ_WAKE_INT_EXP		0x7C00
#define IEEE80211_TWT_REQ_PROT			0x8000

/* TWT Information Field Definitions */
#define IEEE80211_TWT_INFO_FLOW_ID		0x07
#define IEEE80211_TWT_INFO_RESP_REQ		0x08
#define IEEE80211_TWT_INFO_NEXT_TWT_REQ		0x10
#define IEEE80211_TWT_INFO_NEXT_TWT_SIZE	0x60
#define IEEE80211_TWT_INFO_ALL_TWT		0x80

/* Broadcast TWT Info Subfield Definitions */
#define IEEE80211_TWT_BCAST_PERSIST_VAL_ALL	255
#define IEEE80211_BCAST_TWT_PERSIST_EXP		0x07
#define IEEE80211_BCAST_TWT_ID			0xF8
#define IEEE80211_BCAST_TWT_PERSIST_MAN		0xFF00

/* TWT Teardown Field definitions */
#define IEEE80211_TWT_TDOWN_NEGO_TYPE		0x60
#define IEEE80211_TWT_TDOWN_FLOW_ID		0x07
#define IEEE80211_TWT_TDOWN_BC_TWT_ID		0x1F

/* TWT IE Macros */
/* Control Field B2-B3: Negotiation Type */
#define IEEE80211_TWT_IE_CTRL_GET_NEGO_TYPE(twt_ie) \
	MS_OP((twt_ie)->control, IEEE80211_TWT_CTRL_NEGO_TYPE)

/* Request Type Field B0: Request */
#define IEEE80211_TWT_IE_REQ_GET_REQ(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_IS_REQ)
/* Request Type Field B1-3: Setup Command */
#define IEEE80211_TWT_IE_REQ_GET_CMD(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_CMD)
/* Request Type Field B4: Trigger */
#define IEEE80211_TWT_IE_REQ_GET_TRIG(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_TRIG)
/* Request Type Field B5: Implicit / Last parameter set for Broadcast TWT */
#define IEEE80211_TWT_IE_REQ_GET_IMPLICIT(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_IMPLICIT)
/* Request Type Field B6: Flow Type */
#define IEEE80211_TWT_IE_REQ_GET_FTYPE(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_FLOW_TYPE)
/* Request Type Field B7-9: Flow ID */
#define IEEE80211_TWT_IE_REQ_GET_FLOW_ID(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_FLOW_ID)
/* Request Type Field B10-14: Wake Interval Exponent */
#define IEEE80211_TWT_IE_REQ_GET_WAKE_INT_EXP(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_WAKE_INT_EXP)
/* Request Type Field B15: Protection */
#define IEEE80211_TWT_IE_REQ_GET_PROT(twt_ie) \
	MS_OP(le16toh(get_unaligned(&(twt_ie)->req_type)), IEEE80211_TWT_REQ_PROT)

/* Broadcast TWT IE Specific Macros*/
#define IEEE80211_BCAST_TWT_GET_PERSIST_EXP(twt_ie) \
	MS_OP((twt_ie)->bcast_twt_info, IEEE80211_BCAST_TWT_PERSIST_EXP)
#define IEEE80211_BCAST_TWT_GET_ID(twt_ie) \
	MS_OP((twt_ie)->bcast_twt_info, IEEE80211_BCAST_TWT_ID)
#define IEEE80211_BCAST_TWT_GET_PERSIST_MAN(twt_ie) \
	MS_OP((twt_ie)->bcast_twt_info, IEEE80211_BCAST_TWT_PERSIST_MAN)

/* TWT Information Field Macros */
#define IEEE80211_TWT_INFO_GET_FLOW_ID(twt_info) \
	MS_OP((twt_info)->params, IEEE80211_TWT_INFO_FLOW_ID)
#define IEEE80211_TWT_INFO_GET_RESP_REQ(twt_info) \
	MS_OP((twt_info)->params, IEEE80211_TWT_INFO_RESP_REQ)
#define IEEE80211_TWT_INFO_GET_NEXT_TWT_REQ(twt_info) \
	MS_OP((twt_info)->params, IEEE80211_TWT_INFO_NEXT_TWT_REQ)
#define IEEE80211_TWT_INFO_GET_NEXT_TWT_SIZE(twt_info) \
	MS_OP((twt_info)->params, IEEE80211_TWT_INFO_NEXT_TWT_SIZE)
#define IEEE80211_TWT_INFO_GET_ALL_TWT(twt_info) \
	MS_OP((twt_info)->params, IEEE80211_TWT_INFO_ALL_TWT)

/* TWT Teardown Field Macros */
#define IEEE80211_TWT_TDOWN_GET_NEGO_TYPE(twt_tdown) \
	MS_OP((twt_tdown), IEEE80211_TWT_TDOWN_NEGO_TYPE)
#define IEEE80211_TWT_TDOWN_GET_FLOW_ID(twt_tdown) \
	MS_OP((twt_tdown), IEEE80211_TWT_TDOWN_FLOW_ID)
#define IEEE80211_TWT_TDOWN_GET_BC_TWT_ID(twt_tdown) \
	MS_OP((twt_tdown), IEEE80211_TWT_TDOWN_BC_TWT_ID)

struct ieee80211_ie_power_capability {
	uint8_t	id;
	uint8_t	len;
	uint8_t	min_txpwr;
	uint8_t	max_txpwr;
} __packed;

struct ieee80211_ie_tpc_report {
	uint8_t	id;
	uint8_t	len;
	uint8_t tran_power;
	uint8_t link_margin;
} __packed;

#define IEEE80211_CCA_REQMODE_PARALLEL	(1 << 0)
#define IEEE80211_CCA_REQMODE_ENABLE	(1 << 1)
#define IEEE80211_CCA_REQMODE_REQUEST	(1 << 2)
#define IEEE80211_CCA_REQMODE_REPORT	(1 << 3)
#define IEEE80211_CCA_REQMODE_DURA_MAN	(1 << 4)

#define IEEE80211_CCA_REPMODE_LATE	(1 << 0)
#define IEEE80211_CCA_REPMODE_INCAP	(1 << 1)
#define IEEE80211_CCA_REPMODE_REFUSE	(1 << 2)

/* Spectrum Management */
#define IEEE80211_CCA_MEASTYPE_BASIC	0x00	/* Basic Request */
#define IEEE80211_CCA_MEASTYPE_CCA	0x01	/* Clear Channel Assessment Request */
#define IEEE80211_CCA_MEASTYPE_RPI	0x02	/* Receiver Power Indicator (RPI) histogram Request */
/* Radio Measurement */
#define IEEE80211_RM_MEASTYPE_CH_LOAD	0x03	/* Channel Load Request */
#define IEEE80211_RM_MEASTYPE_NOISE	0x04	/* Noise histogram Request */
#define IEEE80211_RM_MEASTYPE_BEACON	0x05	/* Beacon Request */
#define IEEE80211_RM_MEASTYPE_FRAME	0x06	/* Frame Request */
#define IEEE80211_RM_MEASTYPE_STA	0x07	/* STA statistics Request */
#define IEEE80211_RM_MEASTYPE_LCI	0x08	/* LCI Request */
#define IEEE80211_RM_MEASTYPE_CATEGORY	0x09	/* Transmit stream/Category Request */
#define IEEE80211_RM_MEASTYPE_MUL_DIAG	0x0A	/* Multicast diagnostics request */
#define IEEE80211_RM_MEASTYPE_LOC_CIVIC	0x0B	/* Location Civic request */
#define IEEE80211_RM_MEASTYPE_LOC_ID	0x0C	/* Location Identifier request */
#define IEEE80211_RM_MEASTYPE_QTN_CCA	0xFE	/* QTN CCA extension */
#define IEEE80211_RM_MEASTYPE_PAUSE	0xFF	/* Measurement Pause Request */

/* for Radio Measurement Actions. Table 7-57a in 802.11k $7.4.6 */
#define IEEE80211_ACTION_R_MEASUREMENT_REQUEST	0
#define IEEE80211_ACTION_R_MEASUREMENT_REPORT	1
#define IEEE80211_ACTION_R_LINKMEASURE_REQUEST	2
#define IEEE80211_ACTION_R_LINKMEASURE_REPORT	3
#define IEEE80211_ACTION_R_NEIGHBOR_REQUEST	4
#define IEEE80211_ACTION_R_NEIGHBOR_REPORT	5

struct ieee80211_action_sm_measurement_header {
	uint8_t	ia_category;
	uint8_t	ia_action;
	uint8_t	am_token;
	uint8_t	am_data[0];
} __packed;

/* RM Enabled Capabilities Element */
struct ieee80211_ie_bss_load {
        uint8_t id;
        uint8_t len;
        uint8_t version;
        uint8_t assoc_count;
        uint8_t chan_util;
        uint8_t admin_capacity[2];
} __packed;


/* RM measurement capabiltiy bits */
/* byte 0 */
#define IEEE80211_RM_LINK_REPORT_CAP            0x01
#define IEEE80211_RM_NEIGH_REPORT_CAP           0x02
#define IEEE80211_RM_BEACON_PASSIVE_REPORT_CAP  0x10
#define IEEE80211_RM_BEACON_ACTIVE_REPORT_CAP   0x20
#define IEEE80211_RM_BEACON_TABLE_REPORT_CAP    0x40
/* RM Enabled Capabilities Element */
struct ieee80211_ie_rm_enabled_cap {
        uint8_t id;
        uint8_t len;
        uint8_t cap[5];
} __packed;

/* RM - radio measurement request */
struct ieee80211_action_radio_measure_request {
	struct ieee80211_action	am_header;
	uint8_t	am_token;
	uint16_t	am_rep_num;
	uint8_t	am_data[0];
} __packed;

/* RM - radio measurement report */
struct ieee80211_action_radio_measure_report {
	struct ieee80211_action	am_header;
	uint8_t	am_token;
	uint8_t	am_data[0];
} __packed;

/*
 * 802.11h measurement request/report element
 * 802.11k measurement request/report element
 * common part
 */
struct ieee80211_ie_measure_comm {
	uint8_t id;		/* IEEE80211_ELEMID_MEASREQ = 38 */
	uint8_t len;		/* 14 for known types */
	uint8_t token;	/* Non-zero number for diff. measurement reqs. */
	uint8_t mode;	/* bits: 1 enable, 2 req, 3 report, 0,4-7 reserved */
	uint8_t type;	/* basic = 0, cca = 1, rpi histogram = 2 */
	uint8_t data[0];	/* variable format according to meas_type */
} __packed;

struct ieee80211_ie_measreq {
	uint8_t chan_num;	/* channel number */
	uint64_t start_tsf;	/* starting time in tsf */
	uint16_t duration_tu;	/* measurement duration in TU */
} __packed;

/*
 * 802.11k measurement request element of sta statistics
 * for PM module collect sta statistics
 * See 802.11k 2003 7.3.2.21.8
 */
struct ieee80211_ie_measreq_sta_stat {
	uint8_t peer_mac[IEEE80211_ADDR_LEN];	/* Peer Mac Address */
	uint16_t random_interval;	/* randomization interval */
	uint16_t duration_tu;	/* measurement duration in TU */
	uint8_t group_id;		/*	group identity	*/
	uint8_t data[0];	/*	Optional sub-elements in variable length	*/
} __packed;

struct ieee80211_ie_measreq_chan_load {
	uint8_t operating_class;
	uint8_t channel_num;
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measreq_noise_his {
	uint8_t operating_class;
	uint8_t channel_num;
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measreq_beacon {
	uint8_t operating_class;
	uint8_t channel_num;
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t measure_mode;
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measreq_frame {
	uint8_t operating_class;
	uint8_t channel_num;
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t frame_request_type;
#define FRAME_COUNT_REPORT	1

	uint8_t mac_addr[IEEE80211_ADDR_LEN];
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measreq_trans_stream_cat {
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t peer_sta_addr[IEEE80211_ADDR_LEN];
	uint8_t tid;
	uint8_t bin0_range;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measreq_multicast_diag {
	uint16_t random_interval_tu;
	uint16_t duration_tu;
	uint8_t group_mac_addr[IEEE80211_ADDR_LEN];
	uint8_t data[0];
} __packed;

struct ieee80211_subie_multicast_triggered_reporting {
	uint8_t sub_id;
	uint8_t len;
	uint8_t condition;
	uint8_t inactivity_timeout;
	uint8_t reactivation_delay;
} __packed;

struct ieee80211_action_rm_link_measure_request {
	struct ieee80211_action	at_header;
	uint8_t token;
	uint8_t tran_power_used;
	uint8_t max_tran_power;
	uint8_t data[0];
} __packed;

struct ieee80211_action_rm_neighbor_report_request {
	struct ieee80211_action	at_header;
	uint8_t token;
	uint8_t data[0];
} __packed;

/*
 * 802.11h measurement report element
 * see 8.4.2.24 IEEE 802.11-2012
 */
struct ieee80211_ie_measrep_basic {
	uint8_t	chan_num;	/* channel number */
	uint64_t start_tsf;	/* starting time in tsf */
	uint16_t duration_tu;	/* measurement duration in TU */
	uint8_t	basic_report;	/* basic report data */
} __packed;

struct ieee80211_ie_measrep_cca {
	uint8_t	chan_num;	/* channel number */
	uint64_t start_tsf;	/* starting time in tsf */
	uint16_t duration_tu;	/* measurement duration in TU */
	uint8_t	cca_report;	/* cca report data */
#define IEEE80211_MEASURE_BASIC_REPORT_BSS		(1 << 0)
#define IEEE80211_MEASURE_BASIC_REPORT_OFDM_PRE		(1 << 1)
#define IEEE80211_MEASURE_BASIC_REPORT_UNDEF		(1 << 2)
#define IEEE80211_MEASURE_BASIC_REPORT_RADAR		(1 << 3)
#define IEEE80211_MEASURE_BASIC_REPORT_UMMEASURE	(1 << 4)
} __packed;

struct ieee80211_ie_measrep_rpi {
	uint8_t	chan_num;	/* channel number */
	uint64_t start_tsf;	/* starting time in tsf */
	uint16_t duration_tu;	/* measurement duration in TU */
	uint8_t	rpi_report[8];	/* rpi report data */
} __packed;

/*
 * 802.11k measurement report element of sta statistics
 * for PM module collect sta statistics
 * See 802.11k 2003 7.3.2.22.8
 */
struct ieee80211_ie_measrep_sta_stat {
	uint16_t duration_tu;	/* measurement duration in TU */
	uint8_t group_id;		/*	group identity	*/
	uint8_t data[0];	/*	Optional sub-elements in variable length	*/
} __packed;

#define IEEE80211_RM_MEAS_SUBTYPE_LEN_MIN	2

/* Quantenna RM group ie, to complement an 802.11k group ie. Node statistics & parameters */
struct ieee80211_ie_qtn_rm_measure_sta {
	uint8_t id;	/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;   /* length in bytes */
	uint8_t qtn_ie_oui[3];		/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t seq;	/* sequence */
	uint8_t type;				/* Which group (special or all) contains in the data. */
	uint8_t data[0];
} __packed;

struct ieee80211_ie_qtn_rm_txstats {
	uint64_t tx_bytes;
	uint32_t tx_pkts;
	uint32_t tx_discard;
	/**
	 * The number of dropped data packets failed to transmit through
	 * wireless media for each traffic category(TC).
	 */
	uint32_t tx_wifi_drop[WME_AC_NUM];
	uint32_t tx_err;
	uint32_t tx_ucast;		/* unicast */
	uint32_t tx_mcast;		/* multicast */
	uint32_t tx_bcast;		/* broadcast */
} __packed;

struct ieee80211_ie_qtn_rm_rxstats {
	uint64_t rx_bytes;
	uint32_t rx_pkts;
	uint32_t rx_discard;
	uint32_t rx_err;
	uint32_t rx_ucast;		/* unicast */
	uint32_t rx_mcast;		/* multicast */
	uint32_t rx_bcast;		/* broadcast */
} __packed;

struct ieee80211_ie_qtn_rm_sta_all {
	struct ieee80211_ie_qtn_rm_txstats tx_stats;
	struct ieee80211_ie_qtn_rm_rxstats rx_stats;
	u_int32_t max_queued;
	u_int32_t link_quality;
	u_int32_t rssi_dbm;
	u_int32_t bandwidth;
	u_int32_t snr;
	u_int32_t tx_phy_rate;
	u_int32_t rx_phy_rate;
	u_int32_t cca;	/* Reserved for cca */
	u_int32_t br_ip;
	u_int32_t rssi;
	u_int32_t hw_noise;
	u_int8_t soc_macaddr[IEEE80211_ADDR_LEN];
	u_int32_t soc_ipaddr;
} __packed;

/*
 * Statistics Group data format for STB
 */
struct ieee80211_ie_rm_sta_grp221 {
	uint8_t soc_macaddr[IEEE80211_ADDR_LEN];
	uint8_t rssi;
	uint8_t phy_noise;
} __packed;

/* dot11Counters Group */
struct ieee80211_rm_sta_stats_group0 {
	uint32_t dot11TransmittedFragmentCount;
	uint32_t dot11MulticastTransmittedFrameCount;
	uint32_t dot11FailedCount;
	uint32_t dot11ReceivedFragmentCount;
	uint32_t dot11MulticastReceivedFrameCount;
	uint32_t dot11FCSErrorCount;
	uint32_t dot11TransmittedFrameCount;
} __packed;

/* dot11MACStatistics Group */
struct ieee80211_rm_sta_stats_group1 {
	uint32_t dot11RetryCount;
	uint32_t dot11MultipleRetryCount;
	uint32_t dot11FrameDuplicateCount;
	uint32_t dot11RTSSuccessCount;
	uint32_t dot11RTSFailureCount;
	uint32_t dot11ACKFailureCount;
} __packed;

/* dot11QosCounters Group for UP0-UP7 */
struct ieee80211_rm_sta_stats_group2to9 {
	uint32_t dot11QosTransmittedFragmentCount;
	uint32_t dot11QosFailedCount;
	uint32_t dot11QosRetryCount;
	uint32_t dot11QosMultipleRetryCount;
	uint32_t dot11QosFrameDuplicateCount;
	uint32_t dot11QosRTSSuccessCount;
	uint32_t dot11QosRTSFailureCount;
	uint32_t dot11QosACKFailureCount;
	uint32_t dot11QosReceivedFragmentCount;
	uint32_t dot11QosTransmittedFrameCount;
	uint32_t dot11QosDiscardedFrameCount;
	uint32_t dot11QosMPDUsReceivedCount;
	uint32_t dot11QosRetriesReceivedCount;
} __packed;

/* dot11BSSAverageAccessDelay Group (only available at an AP) */
struct ieee80211_rm_sta_stats_group10 {
	uint32_t dot11STAStatisticsAPAverageAccessDelay;
	uint32_t dot11STAStatisticsAverageAccessDelayBestEffort;
	uint32_t dot11STAStatisticsAverageAccessDelayBackGround;
	uint32_t dot11STAStatisticsAverageAccessDelayVideo;
	uint32_t dot11STAStatisticsAverageAccessDelayVoice;
	uint32_t dot11STAStatisticsStationCount;
	uint32_t dot11STAStatisticsChannelUtilization;
} __packed;

struct ieee80211_rm_sta_stats_group11 {
	uint32_t dot11TransmittedAMSDUCount;
	uint32_t dot11FailedAMSDUCount;
	uint32_t dot11RetryAMSDUCount;
	uint32_t dot11MultipleRetryAMSDUCount;
	uint32_t dot11TransmittedOctetsInAMSDUCount;
	uint32_t dot11AMSDUAckFailureCounnt;
	uint32_t dot11ReceivedAMSDUCount;
	uint32_t dot11ReceivedOctetsInAMSDUCount;
} __packed;

struct ieee80211_rm_sta_stats_group12 {
	uint32_t dot11TransmittedAMPDUCount;
	uint32_t dot11TransmittedMPDUsInAMPDUCount;
	uint64_t dot11TransmittedOctetsInAMPDUCount;
	uint32_t dot11AMPDUReceivedCount;
	uint32_t dot11MPDUInReceivedAMPDUCount;
	uint64_t dot11ReceivedOctetsInAMPDUCount;
	uint32_t dot11AMPDUDelimiterCRCErrorCount;
} __packed;

struct ieee80211_rm_sta_stats_group13 {
	uint32_t dot11ImplicitBARFailureCount;
	uint32_t dot11ExplicitBARFailureCount;
	uint32_t dot11ChannelWidthSwitchCount;
	uint32_t dot11TwentyMHzFrameTransmittedCount;
	uint32_t dot11FortyMHzFrameTransmittedCount;
	uint32_t dot11TwentyMHzFrameReceivedCount;
	uint32_t dot11FortyMHzFrameReceivedCount;
	uint32_t dot11PSMPUTTGrantDuration;
	uint32_t dot11PSMPUTTUsedDuration;
} __packed;

struct ieee80211_rm_sta_stats_group14 {
	uint32_t dot11GrantedRDGUsedCount;
	uint32_t dot11GrantedRDGUnusedCount;
	uint32_t dot11TransmittedFramesInGrantedRDGCount;
	uint64_t dot11TransmittedOctetsInGrantedRDGCount;
	uint32_t dot11DualCTSSuccessCount;
	uint32_t dot11DualCTSFailureCount;
	uint32_t dot11RTSLSIGSuccessCount;
	uint32_t dot11RTSLSIGFailureCount;
} __packed;

struct ieee80211_rm_sta_stats_group15 {
	uint32_t dot11BeamformingFrameCount;
	uint32_t dot11STBCCTSSuccessCount;
	uint32_t dot11STBCCTSFailureCount;
	uint32_t dot11nonSTBCCTSSuccessCount;
	uint32_t dot11nonSTBCCTSFailureCount;
} __packed;

struct ieee80211_rm_sta_stats_group16 {
	uint32_t dot11RSNAStatsCMACICVErrors;
	uint32_t dot11RSNAStatsCMACReplays;
	uint32_t dot11RSNAStatsRobustMgmtCCMPReplays;
	uint32_t dot11RSNAStatsTKIPICVErrors;
	uint32_t dot11RSNAStatsTKIPReplays;
	uint32_t dot11RSNAStatsCCMPDecryptErrors;
	uint32_t dot11RSNAStatsCCMPReplays;
} __packed;

/*
 * STA Statistics QTN specific
 */
enum RadioMeasureQTNElementID {
	RM_QTN_TX_STATS			=	0,
	RM_QTN_RX_STATS			=	1,
	RM_QTN_MAX_QUEUED		=	2,
	RM_QTN_LINK_QUALITY		=	3,
	RM_QTN_RSSI_DBM			=	4,
	RM_QTN_BANDWIDTH		=	5,
	RM_QTN_SNR			=	6,
	RM_QTN_TX_PHY_RATE		=	7,
	RM_QTN_RX_PHY_RATE		=	8,
	RM_QTN_CCA			=	9,
	RM_QTN_BR_IP			=	10,
	RM_QTN_RSSI			=	11,
	RM_QTN_HW_NOISE			=	12,
	RM_QTN_SOC_MACADDR		=	13,
	RM_QTN_SOC_IPADDR		=	14,
	RM_QTN_MAX			=	RM_QTN_SOC_IPADDR,
	RM_QTN_UNKNOWN			=	15,
	RM_QTN_CTRL_START		=	16,
	RM_QTN_RESET_CNTS		=	16,
	RM_QTN_RESET_QUEUED		=	17,
	RM_QTN_CTRL_END			=	17,
};
#define RM_QTN_MEASURE_MASK ((1 << (RM_QTN_CTRL_END + 1)) - 1)

/*
 * STA Statistic for Group221 specific
 */
enum RadioMeasureGrp221ElementID {
	RM_GRP221_RSSI			=	(RM_QTN_CTRL_END + 1),
	RM_GRP221_PHY_NOISE		=	(RM_QTN_CTRL_END + 2),
	RM_GRP221_SOC_MAC		=	(RM_QTN_CTRL_END + 3),
};

#define IEEE80211_MAX_ELEM_ID 255
extern uint8_t ieee80211_max_ie_len[IEEE80211_MAX_ELEM_ID + 1];
extern uint8_t ieee80211_extensible_ie_len[IEEE80211_MAX_ELEM_ID + 1];
extern const uint8_t ieee80211_meas_sta_qtn_report_subtype_len[RM_QTN_CTRL_END + 1];
extern struct ieee80211_conf_s ieee80211_conf;

/* Standard CCA Flag to used */
#define RM_STANDARD_CCA 0x1009
#define IEEE80211_11K_CCA_INTF_SCALE 255
/*
 * CCA radio measurement report field
 */
struct cca_rm_rep_data {
	uint8_t ch_num;
	uint8_t tm_start[8];
	uint8_t m_duration[2];
	uint8_t busy_frac;
} __packed;

/* CCA report IE*/
struct ieee80211_ie_rm_measure_cca_rep {
	uint8_t id;
	uint8_t len;
	uint8_t rm_token;
	uint8_t rm_rep_mode;
	uint8_t rm_rep_type;
	struct cca_rm_rep_data rep_data;
	struct ieee80211_ie_qtn_scs scs_data;
} __packed;

struct ieee80211_ie_measrep_chan_load {
	uint8_t operating_class;
	uint8_t channel_num;
	uint8_t start_time[8];
	uint16_t duration_tu;
	uint8_t channel_load;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measrep_noise_his {
	uint8_t operating_class;
	uint8_t channel_num;
	uint8_t start_time[8];
	uint16_t duration_tu;
	uint8_t antenna_id;
	uint8_t anpi;
	uint8_t ipi[11];
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measrep_beacon {
	uint8_t operating_class;
	uint8_t channel_num;
	uint8_t start_time[8];
	uint16_t duration_tu;
	uint8_t reported_frame_info;
	uint8_t rcpi;
	uint8_t rsni;
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint8_t antenna_id;
	uint8_t parent_tsf[4];
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measrep_frame {
	uint8_t operating_class;
	uint8_t channel_num;
	uint8_t start_time[8];
	uint16_t duration_tu;
	uint8_t data[0];
} __packed;

#define IEEE80211_FRAME_REPORT_SUBELE_FRAME_COUNT_REPORT	1

struct ieee80211_subie_section_frame_entry {
	uint8_t id;
	uint8_t len;
	uint8_t transmit_address[IEEE80211_ADDR_LEN];
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint8_t phy_type;
	uint8_t avg_rcpi;
	uint8_t last_rsni;
	uint8_t last_rcpi;
	uint8_t anntenna_id;
	uint16_t frame_cnt;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_measrep_trans_stream_cat {
	uint8_t start_time[8];
	uint16_t duration_tu;
	uint8_t peer_sta_address[IEEE80211_ADDR_LEN];
	uint8_t tid;
	uint8_t reason;
	uint32_t tran_msdu_cnt;
	uint32_t msdu_discarded_cnt;
	uint32_t msdu_failed_cnt;
	uint32_t msdu_mul_retry_cnt;
	uint32_t qos_cf_lost_cnt;
	uint32_t avg_queue_delay;
	uint32_t avg_trans_delay;
	uint8_t bin0_range;
	uint32_t bin0;
	uint32_t bin1;
	uint32_t bin2;
	uint32_t bin3;
	uint32_t bin4;
	uint32_t bin5;
} __packed;

struct ieee80211_ie_measrep_multicast_diag {
	uint8_t measure_time[8];
	uint16_t duration_tu;
	uint8_t group_mac_addr[IEEE80211_ADDR_LEN];
	uint8_t reason;
	uint32_t mul_rx_msdu_cnt;
	uint16_t first_seq_num;
	uint16_t last_seq_num;
	uint16_t mul_rate;
} __packed;

struct ieee80211_action_rm_link_measure_report {
	struct ieee80211_action at_header;
	uint8_t token;
	struct ieee80211_ie_tpc_report tpc_report;
	uint8_t recv_antenna_id;
	uint8_t tran_antenna_id;
	uint8_t rcpi;
	uint8_t rsni;
	uint8_t data[0];
} __packed;

struct ieee80211_action_rm_neighbor_report_response {
	struct ieee80211_action at_header;
	uint8_t token;
	uint8_t data[0];
} __packed;

struct ieee80211_ie_neighbor_report {
	uint8_t id;
	uint8_t len;
	uint8_t bssid[IEEE80211_ADDR_LEN];
	uint32_t bssid_info;
#define BSSID_INFO_AP_NOT_REACHABLE		(1 << 0)
#define BSSID_INFO_AP_UNKNOWN			(2 << 0)
#define BSSID_INFO_AP_REACHABLE			(3 << 0)
#define BSSID_INFO_SECURITY_COPY		(1 << 2)
#define BSSID_INFO_KEY_SCOPE_COPY		(1 << 3)
#define BSSID_INFO_CAP_SPECTRUM_MANAGEMENT	(1 << 4)
#define BSSID_INFO_CAP_QOS			(1 << 5)
#define BSSID_INFO_CAP_APSD			(1 << 6)
#define BSSID_INFO_CAP_RADIO_MEASUREMENT	(1 << 7)
#define BSSID_INFO_CAP_DELAYED_BA		(1 << 8)
#define BSSID_INFO_CAP_IMMEDIATE_BA		(1 << 9)
#define BSSID_INFO_MOBILITY_DOMAIN		(1 << 10)
#define BSSID_INFO_HIGH_THROUGHPUT		(1 << 11)
#define BSSID_INFO_VERY_HIGH_THROUGHPUT         (1 << 12)
#define BSSID_INFO_COLOCATED_AP			(1 << 16)
#define BSSID_INFO_20_TU_PROBE_RESP_ACT		(1 << 17)
	uint8_t operating_class;
	uint8_t channel;
	uint8_t phy_type;
	uint8_t data[0];
} __packed;

/* HT actions */
#define IEEE80211_ACTION_HT_TXCHWIDTH		0	/* recommended transmission channel width */
#define IEEE80211_ACTION_HT_MIMOPWRSAVE		1	/* MIMO power save */
#define IEEE80211_ACTION_HT_NCBEAMFORMING	5	/* HT non compressed beamforming report */
#define IEEE80211_ACTION_HT_CBEAMFORMING	6	/* HT compressed beamforming report */

/* VHT actions */
#define IEEE80211_ACTION_VHT_CBEAMFORMING	0	/* VHT compressed beamforming report */
#define IEEE80211_ACTION_VHT_MU_GRP_ID          1       /* VHT MU GRP ID mgmt */
#define IEEE80211_ACTION_VHT_OPMODE_NOTIFICATION	2	/* VHT Operating mode Notification */

/* HE actions */
#define IEEE80211_ACTION_HE_CBEAMFORMING_CQI	0	/* HE compressed beamforming and CQI report */
#define IEEE80211_ACTION_HE_QUIET_TIME_PERIOD	1       /* HE quiet timer period */
#define IEEE80211_ACTION_HE_OPS			2	/* HE OPS */

/* HT - recommended transmission channel width */
struct ieee80211_action_ht_txchwidth {
	struct ieee80211_action		at_header;
	u_int8_t			at_chwidth;
} __packed;

#define IEEE80211_A_HT_TXCHWIDTH_20	0
#define IEEE80211_A_HT_TXCHWIDTH_2040	1


/* HT - MIMO Power Save */
struct ieee80211_action_ht_mimopowersave {
	struct ieee80211_action		am_header;
	uint8_t				am_enable_mode;
} __packed;

/* HT - Non compressed beam forming */

struct ht_mimo_ctrl {
	uint16_t			am_mimoctrl;
	uint32_t			am_timestamp;
} __packed;

#define IEEE80211_HT_MIMO_CTRL_NC_M			0x0003
#define IEEE80211_HT_MIMO_CTRL_NC_S			0
#define IEEE80211_HT_MIMO_CTRL_NR_M			0x000C
#define IEEE80211_HT_MIMO_CTRL_NR_S			2
#define IEEE80211_HT_MIMO_CTRL_CH_WIDTH_20	0x0000
#define IEEE80211_HT_MIMO_CTRL_CH_WIDTH_40	0x0010
#define IEEE80211_HT_MIMO_CTRL_NG_M			0x0060
#define IEEE80211_HT_MIMO_CTRL_NG_S			5
#define IEEE80211_HT_MIMO_CTRL_NB_M		0x0180
#define IEEE80211_HT_MIMO_CTRL_NB_S		7
#define IEEE80211_HT_MIMO_CTRL_CODEBOOK_M	0x0600
#define IEEE80211_HT_MIMO_CTRL_CODEBOOK_S	9
#define IEEE80211_HT_MIMO_CTRL_SEG_M		0x3800
#define IEEE80211_HT_MIMO_CTRL_SEG_S		11


enum {
	IEEE80211_HT_MIMO_CTRL_NC_1 = 0,
	IEEE80211_HT_MIMO_CTRL_NC_2,
	IEEE80211_HT_MIMO_CTRL_NC_3,
	IEEE80211_HT_MIMO_CTRL_NC_4,
};

enum {
	IEEE80211_HT_MIMO_CTRL_NR_1 = 0,
	IEEE80211_HT_MIMO_CTRL_NR_2,
	IEEE80211_HT_MIMO_CTRL_NR_3,
	IEEE80211_HT_MIMO_CTRL_NR_4,
};

enum {
	IEEE80211_HT_MIMO_CTRL_NG_NONE = 0,
	IEEE80211_HT_MIMO_CTRL_NG_2,
	IEEE80211_HT_MIMO_CTRL_NG_4,
	IEEE80211_HT_MIMO_CTRL_NG_RESERVED,
};

enum {
	IEEE80211_HT_MIMO_CTRL_NB_4 = 0,
	IEEE80211_HT_MIMO_CTRL_NB_2,
	IEEE80211_HT_MIMO_CTRL_NB_6,
	IEEE80211_HT_MIMO_CTRL_NB_8,
};

struct ieee80211_action_ht_bf {
	struct ieee80211_action	am_header;
	struct ht_mimo_ctrl	am_mimo_ctrl;
	uint8_t			am_bf_report[0]; /* start of beamforming report */
} __packed;

/* VHT - Tx Beamforming */
struct vht_mimo_ctrl {
	uint8_t		am_mimoctrl[3];
} __packed;

/* VHT - Operating mode notification */
struct ieee80211_action_vht_opmode_notification {
	struct ieee80211_action		am_header;
	u_int8_t			am_opmode;
} __packed;

#define IEEE80211_VHT_OPMODE_CHWIDTH		0x03
#define IEEE80211_VHT_OPMODE_CHWIDTH_S		0
#define IEEE80211_VHT_OPMODE_160MH_FLAG		0x04
#define IEEE80211_VHT_OPMODE_160MH_FLAG_S	2
#define IEEE80211_VHT_OPMODE_RXNSS		0x70
#define IEEE80211_VHT_OPMODE_RXNSS_S		4
#define IEEE80211_VHT_OPMODE_RXNSS_TYPE		0x80
#define IEEE80211_VHT_OPMODE_RXNSS_TYPE_S	7

#define IEEE80211_VHT_MIMO_CTRL_NC_M		0x000007
#define IEEE80211_VHT_MIMO_CTRL_NC_S		0
#define IEEE80211_VHT_MIMO_CTRL_NR_M		0x000038
#define IEEE80211_VHT_MIMO_CTRL_NR_S		3
#define IEEE80211_VHT_MIMO_CTRL_CH_BW_M		0x0000C0
#define IEEE80211_VHT_MIMO_CTRL_CH_BW_S		6
#define IEEE80211_VHT_MIMO_CTRL_CH_WIDTH_20	0x000000
#define IEEE80211_VHT_MIMO_CTRL_CH_WIDTH_40	0x000040
#define IEEE80211_VHT_MIMO_CTRL_CH_WIDTH_80	0x000080
#define IEEE80211_VHT_MIMO_CTRL_CH_WIDTH_160	0x0000C0
#define IEEE80211_VHT_MIMO_CTRL_NG_M		0x000300
#define IEEE80211_VHT_MIMO_CTRL_NG_S		8
#define IEEE80211_VHT_MIMO_CTRL_CODEBOOK_M	0x000400
#define IEEE80211_VHT_MIMO_CTRL_CODEBOOK_S	10
#define IEEE80211_VHT_MIMO_CTRL_FBTYPE_M	0x000800
#define IEEE80211_VHT_MIMO_CTRL_FBTYPE_S	11
#define IEEE80211_VHT_MIMO_CTRL_R_FB_M          0x007000
#define IEEE80211_VHT_MIMO_CTRL_R_FB_S          12
#define IEEE80211_VHT_MIMO_CTRL_FIRSTFB_M       0x008000
#define IEEE80211_VHT_MIMO_CTRL_FIRSTFB_S       15
#define IEEE80211_VHT_MIMO_CTRL_DTOKEN_M        0xFC0000
#define IEEE80211_VHT_MIMO_CTRL_DTOKEN_S        18

/* Block Ack actions */
#define IEEE80211_ACTION_BA_ADDBA_REQ		0	/* Add block ack request */
#define IEEE80211_ACTION_BA_ADDBA_RESP		1	/* Add block ack response */
#define IEEE80211_ACTION_BA_DELBA			2	/* delete block ack */

/* BA - Add block ack request */
struct ieee80211_action_ba_addba_req {
	struct ieee80211_action		am_header;
	uint8_t	am_dlg;
	uint16_t	am_ba_params;
	uint16_t	am_ba_to;
	uint16_t	am_ba_seq;
} __packed;

#define IEEE80211_A_BA_AMSDU_SUPPORTED		0x0001
#define IEEE80211_A_BA_IMMEDIATE			0x0002
#define IEEE80211_A_BA_DELAYED				0x0000
#define IEEE80211_A_BA_TID_M				0x003C
#define IEEE80211_A_BA_TID_S				2
#define IEEE80211_A_BA_BUFF_SIZE_M			0xFFC0
#define IEEE80211_A_BA_BUFF_SIZE_S			6
#define IEEE80211_A_BA_FRAG_M				0x000F
#define IEEE80211_A_BA_FRAG_S				0
#define IEEE80211_A_BA_SEQ_M				0xFFF0
#define IEEE80211_A_BA_SEQ_S				4
#define IEEE80211_IOT_INTEL_AGG_MAX_FRAMES_NUM          16

/* BA - Add block ack response */
struct ieee80211_action_ba_addba_resp {
	struct ieee80211_action		am_header;
	uint8_t	am_dlg;
	__le16		am_status;
	__le16		am_ba_params;
	__le16		am_ba_to;
} __packed;

/* BA - delete block ack request */
struct ieee80211_action_ba_delba {
	struct ieee80211_action		am_header;
	__le16		am_delba_params;
	__le16		am_reason;
}__packed;

#define IEEE80211_A_BA_INITIATOR			0x0800
#define IEEE80211_A_BA_INITIATOR_S			11
#define IEEE80211_A_BA_DELBA_TID			0xF000
#define IEEE80211_A_BA_DELBA_TID_S			12

/* Move to a .config file later. */
#define CONFIG_QHOP 1

#ifdef CONFIG_QHOP
#define QDRV_ACTION_TYPE_QHOP        0x19
#define QDRV_ACTION_QHOP_DFS_REPORT  0x1
#define QDRV_ACTION_QHOP_SCS_REPORT  0x2

struct qdrv_vendor_action_header {
	uint8_t category;
	uint8_t oui[3];
	uint8_t type;
	uint8_t action;
} __packed;

struct qdrv_vendor_action_qhop_dfs_data {
	uint8_t cur_chan;
} __packed;

#endif

/*
 * 802.11w / PMF SA Query Action Frame
 */
#define IEEE80211_ACTION_W_SA_QUERY_REQ		0
#define IEEE80211_ACTION_W_SA_QUERY_RESP	1

struct ieee80211_action_sa_query {
	struct ieee80211_action		at_header;
	u_int16_t			at_tid;
} __packed;

/*
 * Protected HE Action frame
 */
#define IEEE80211_ACTION_HE_PROT_BSSCOLOR	0

/*
 * Control frames.
 */

struct ieee80211_frame_ctrl {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_min {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

/* 80211.v WNM */
#define	IEEE80211_WNM_BSS_TM_CAP 19

/* IEEE 802.11v - WNM Action field values */
#define IEEE80211_WNM_EVENT_REQ				0
#define	IEEE80211_WNM_EVENT_REPORT			1
#define	IEEE80211_WNM_DIAGNOSTIC_REQ			2
#define	IEEE80211_WNM_DIAGNOSTIC_REPORT			3
#define	IEEE80211_WNM_LOCATION_CFG_REQ			4
#define	IEEE80211_WNM_LOCATION_CFG_RESP			5
#define	IEEE80211_WNM_BSS_TRANS_MGMT_QUERY		6
#define	IEEE80211_WNM_BSS_TRANS_MGMT_REQ		7
#define	IEEE80211_WNM_BSS_TRANS_MGMT_RESP		8
#define	IEEE80211_WNM_FMS_REQ				9
#define	IEEE80211_WNM_FMS_RESP				10
#define	IEEE80211_WNM_COLLOCATED_INTERFERENCE_REQ	11
#define	IEEE80211_WNM_COLLOCATED_INTERFERENCE_REPORT	12
#define	IEEE80211_WNM_TFS_REQ				13
#define	IEEE80211_WNM_TFS_RESP				14
#define	IEEE80211_WNM_TFS_NOTIFY			15
#define	IEEE80211_WNM_SLEEP_MODE_REQ			16
#define	IEEE80211_WNM_SLEEP_MODE_RESP			17
#define	IEEE80211_WNM_TIM_BROADCAST_REQ			18
#define	IEEE80211_WNM_TIM_BROADCAST_RESP		19
#define	IEEE80211_WNM_QOS_TRAFFIC_CAPAB_UPDATE		20
#define	IEEE80211_WNM_CHANNEL_USAGE_REQ			21
#define	IEEE80211_WNM_CHANNEL_USAGE_RESP		22
#define	IEEE80211_WNM_DMS_REQ				23
#define	IEEE80211_WNM_DMS_RESP				24
#define	IEEE80211_WNM_TIMING_MEASUREMENT_REQ		25
#define	IEEE80211_WNM_NOTIFICATION_REQ			26
#define	IEE8E0211_WNM_NOTIFICATION_RESP			27

/* IEEE 802.11v - BSS Transition Management Request - Request Mode */
#define BTM_REQ_PREF_CAND_LIST_INCLUDED		BIT(0)
#define BTM_REQ_ABRIDGED			BIT(1)
#define BTM_REQ_DISASSOC_IMMINENT		BIT(2)
#define BTM_REQ_BSS_TERMINATION_INCLUDED	BIT(3)
#define BTM_REQ_ESS_DISASSOC_IMMINENT		BIT(4)

/* IEEE Std 802.11-2012 - Table 8-253 */
/* BTM response status codes */
#define	BTM_RSP_ACCEPT					0
#define	BTM_RSP_REJECT_UNSPECIFIED			1
#define	BTM_RSP_REJECT_INSUFFICIENT_BEACON		2
#define	BTM_RSP_REJECT_INSUFFICIENT_CAPABITY		3
#define	BTM_RSP_REJECT_UNDESIRED			4
#define	BTM_RSP_REJECT_DELAY_REQUEST			5
#define	BTM_RSP_REJECT_STA_CANDIDATE_LIST_PROVIDED	6
#define	BTM_RSP_REJECT_NO_SUITABLE_CANDIDATES		7
#define	BTM_RSP_REJECT_LEAVING_ESS			8

/* Neighbor report and BTM subelements */
#define WNM_NEIGHBOR_TSF				1
#define WNM_NEIGHBOR_CONDENSED_COUNTRY_STRING		2
#define WNM_NEIGHBOR_BTM_CANDIDATE_PREFERENCE		3
#define WNM_NEIGHBOR_BTM_TERMINATION_DURATION		4
#define WNM_NEIGHBOR_BEARING				5
#define WNM_NEIGHBOR_MEASUREMENT_PILOT			66
#define WNM_NEIGHBOR_RRM_ENABLED_CAPABILITIES		70
#define WNM_NEIGHBOR_MULTIPLE_BSSID			71

/* Event report event type */
#define WNM_EVENT_REPORT_BSSCOLOR_COLLISION		4
#define WNM_EVENT_REPORT_BSSCOLOR_IN_USE		5

#define WNM_BSSCOLOR_COLLISION_EVENT_REPORT_LEN		11
#define WNM_EVENT_REPORT_AUTO_DIATOKEN			0
#define IEEE80211_IE_EVENT_REPORT_SUCCESS		0
#define IEEE80211_IE_EVENT_REPORT_AUTO_TOKEN		0

struct ieee80211_ie_event_report {
	uint8_t		id;		/* IEEE80211_ELEMID_EVENT_REPORT */
	uint8_t		len;		/* length in bytes */
	uint8_t		token;		/* event token */
	uint8_t		type;		/* event_type */
	uint8_t		status;		/* event report status */
	uint8_t		data[0];	/* Optional Fields */
}__packed;

struct ieee80211_event_report_paramset {
	uint8_t dialog_token;
	struct ieee80211_ie_event_report report[0];
}__packed;

struct ieee80211_action_event_report {
	struct ieee80211_action report_header;
	struct ieee80211_event_report_paramset report_param;
}__packed;

/* BSS Termination duration */
struct ieee80211_ie_btm_bss_termdur {
	uint8_t    subelem_id;
	uint8_t    length;
	uint8_t    bss_term_tsf[8];
	uint16_t   duration;
}__packed;

struct ieee80211_btm_query_paramset {
	uint8_t dialog_token;
	uint8_t reason;
	/* Optional BSS Transition Candidate list (neighbor report) */
	uint8_t data[0];
}__packed;

struct ieee80211_action_btm_query {
	struct ieee80211_action btm_header;
	struct ieee80211_btm_query_paramset btm_query_param;
}__packed;

struct ieee80211_btm_req_paramset {
	uint8_t		dialog_token;
	uint8_t		request_mode;
	uint16_t	disassoc_timer;
	uint8_t		validity_interval;
	uint8_t		info[0];
	/* Optional BSS termination duration */
	/* Optional session information URL */
	/* Optional BSS Transition Candidate list (neighbor report) */
}__packed;

struct ieee80211_action_btm_req {
	struct ieee80211_action			btm_header;
	struct ieee80211_btm_req_paramset	btm_req_param;
}__packed;

struct ieee80211_btm_rsp_paramset {
	uint8_t	dialog_token;
	uint8_t status_code;
	uint8_t bss_term_delay;
	/* Optional Target BSSID */
	/* Optional BSS Transition Candidate list (neighbor report) */
	uint8_t data[0];
}__packed;

struct ieee80211_action_btm_rsp {
	struct ieee80211_action			btm_header;
	struct ieee80211_btm_rsp_paramset	btm_rsp_param;
}__packed;

#define IEEE80211_MAX_IDLE_PERIOD_DEF 0x0014
struct ieee80211_max_idle_ie {
	uint16_t max_idle_period;
	uint8_t protected_keep_alive;
} __packed;

/* ieee80211r  related*/
/*
 * mobility domain information element.
 */
struct ieee80211_md_ie {
	uint8_t		md_id;		/* IEEE80211_ELEMID_MOBILITY_DOMAIN */
	uint8_t		md_len;		/* length in bytes */
	uint16_t	md_info;	/* mobility domain id */
	uint8_t		md_cap;		/* capability */
}__packed;

#define IEEE80211_MDIE_LEN	3

/*
 * BAR frame format
 */
#define IEEE80211_BAR_CTL_TID		0xF000      /* tid mask             */
#define IEEE80211_BAR_CTL_TID_S         12      /* tid shift            */
#define IEEE80211_BAR_CTL_NOACK		0x0001      /* no-ack policy        */
#define IEEE80211_BAR_CTL_COMBA		0x0004      /* compressed block-ack */
#define IEEE80211_BAR_CTL_MULTIBA	0x0006		/* Multi TID Block Ack */
#define IEEE80211_BAR_INFO_FRAG_M	0x000F		/* fragment mask */
#define IEEE80211_BAR_CTL_FRAG_S	0			/* fragment shift */
#define IEEE80211_BAR_CTL_SEQ		0xFFF0		/* sequence number mask */
#define IEEE80211_BAR_CTL_SEQ_S		4			/* sequence number shift */


struct ieee80211_frame_bar {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_ra[IEEE80211_ADDR_LEN];
	uint8_t	i_ta[IEEE80211_ADDR_LEN];
	uint16_t	i_ctl;
	uint8_t	i_info[0];						/* variable length */
	/* FCS */
} __packed;

struct ieee80211_frame_bar_info_simple {
	uint16_t	i_seq;
} __packed;

struct ieee80211_frame_bar_info_tid {
	uint16_t	i_tid;
	uint16_t	i_seq;
} __packed;

#define IEEE80211_BAR_HDR_LEN		16
#define IEEE80211_BAR_COMPRESSED_LEN	(sizeof(struct ieee80211_frame_bar) + \
						sizeof(struct ieee80211_frame_bar_info_simple))

/*
 * BA frame format
 */

#define IEEE80211_BA_TYPE_COMPRESSED	2
#define IEEE80211_BA_TYPE_MULTI_STA	11

struct ieee80211_frame_ba {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_ra[IEEE80211_ADDR_LEN];
	uint8_t	i_ta[IEEE80211_ADDR_LEN];
	uint16_t	i_ctl;
	uint8_t	i_info[0];						/* variable length */
	/* FCS */
} __packed;

struct ieee80211_frame_ba_simple {
	uint16_t	i_seq;
	uint8_t	i_bm[128];
} __packed;

struct ieee80211_frame_ba_comp {
	uint16_t	i_seq;
	uint8_t	i_bm[8];
} __packed;

struct ieee80211_frame_ba256_comp {
	uint16_t i_seq;
	uint8_t i_bm[32];
} __packed;

struct ieee80211_frame_ba_aid_tid_tuple {
	uint16_t aid_tid_info;
	uint16_t ba_start_seq_ctrl;
	uint8_t ba_bm[8];
} __packed;

struct ieee80211_frame_ba256_aid_tid_tuple {
	uint16_t aid_tid_info;
	uint16_t ba_start_seq_ctrl;
	uint8_t ba_bm[32];
} __packed;

struct ieee80211_frame_ba_tid {
	uint16_t	i_tid;
	uint16_t	i_seq;
	uint8_t	i_bm[8];
} __packed;

struct ieee80211_frame_rts {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[IEEE80211_ADDR_LEN];
	uint8_t i_ta[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_cts {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_ack {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_pspoll {
	uint8_t i_fc[2];
	uint8_t i_aid[2];
	uint8_t i_bssid[IEEE80211_ADDR_LEN];
	uint8_t i_ta[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_cfend {		/* NB: also CF-End+CF-Ack */
	uint8_t i_fc[2];
	uint8_t i_dur[2];	/* should be zero */
	uint8_t i_ra[IEEE80211_ADDR_LEN];
	uint8_t i_bssid[IEEE80211_ADDR_LEN];
	/* FCS */
} __packed;

struct ieee80211_frame_cw {
	uint8_t	i_fc[2];
	uint8_t	i_dur[2];
	uint8_t	i_ra[IEEE80211_ADDR_LEN];
	uint8_t	i_cfc[2]; /* carried frame control */
	/* variable control frame */
	/* FCS */
} __packed;

/* 802.11 Management over Ethernet Payload Types (Annex U.1) */
#define IEEE80211_SNAP_TYPE_REMOTE             1   /* Remote request/response */
#define IEEE80211_SNAP_TYPE_TDLS               2   /* TDLS */

#define IEEE80211_FCS_LEN		4
#define IEEE80211_ENCR_HDR_AES_LEN	16

/*
 * BEACON management packets
 *
 *	octet timestamp[8]
 *	octet beacon interval[2]
 *	octet capability information[2]
 *	information element
 *		octet elemid
 *		octet length
 *		octet information[length]
 */
#define	IEEE80211_BEACON_PROBE_RESP_FIXED_PARAMS_LEN	12

typedef uint8_t *ieee80211_mgt_beacon_t;

#define	IEEE80211_BEACON_INTERVAL(beacon) \
	((beacon)[8] | ((beacon)[9] << 8))
#define	IEEE80211_BEACON_CAPABILITY(beacon) \
	((beacon)[10] | ((beacon)[11] << 8))

#define	IEEE80211_CAPINFO_ESS			0x0001
#define	IEEE80211_CAPINFO_IBSS			0x0002
#define	IEEE80211_CAPINFO_CF_POLLABLE		0x0004
#define	IEEE80211_CAPINFO_CF_POLLREQ		0x0008
#define	IEEE80211_CAPINFO_PRIVACY		0x0010
#define	IEEE80211_CAPINFO_SHORT_PREAMBLE	0x0020
#define	IEEE80211_CAPINFO_PBCC			0x0040
#define	IEEE80211_CAPINFO_CHNL_AGILITY		0x0080
#define IEEE80211_CAPINFO_SPECTRUM_MGMT		0x0100
#define IEEE80211_CAPINFO_WME                   0x0200
#define IEEE80211_CAPINFO_SHORT_SLOTTIME        0x0400
#define IEEE80211_CAPINFO_APSD                  0x0800
#define IEEE80211_CAPINFO_RM                    0x1000
#define	IEEE80211_CAPINFO_DSSSOFDM		0x2000
#define IEEE80211_CAPINFO_DELAYED_BA            0x4000
#define IEEE80211_CAPINFO_IMMEDIATE_BA          0x8000

/* Extended Capabilities element (8.4.2.29) - bits 0 to 31 */
#define IEEE80211_EXTCAP1_TDLS_UAPSD		0x10000000UL	/* TDLS peer U-APSD buf STA support */
#define IEEE80211_EXTCAP1_TDLS_PSM		0x20000000UL	/* Peer PSM Support */
#define IEEE80211_EXTCAP1_TDLS_CS		0x40000000UL	/* channel switching */

/* Extended Capabilities element (8.4.2.29) - bits 32 to 63 */
#define IEEE80211_EXTCAP2_TDLS			0x00000020UL	/* TDLS supported */
#define IEEE80211_EXTCAP2_TDLS_PROHIB		0x00000040UL	/* TDLS prohibited */
#define IEEE80211_EXTCAP2_TDLS_CS_PROHIB	0x00000080UL	/* TDLS channel switch prohibited */

#define IEEE80211_EXTCAP_LENGTH	10	/* Extended capabilities element length */

/* SSID IE */
struct ieee80211_ie_ssid {
	uint8_t id;			/* IEEE80211_ELEMID_SSID */
	uint8_t len;			/* length in bytes */
	uint8_t ssid[0];		/* ssid */
} __packed;

/*
 * 802.11i/WPA information element (maximally sized).
 */
struct ieee80211_ie_wpa {
	uint8_t wpa_id;			/* IEEE80211_ELEMID_VENDOR */
	uint8_t wpa_len;		/* length in bytes */
	uint8_t wpa_oui[3];		/* 0x00, 0x50, 0xf2 */
	uint8_t wpa_type;		/* OUI type */
	uint16_t wpa_version;		/* spec revision */
	uint32_t wpa_mcipher[1];	/* multicast/group key cipher */
	uint16_t wpa_uciphercnt;	/* # pairwise key ciphers */
	uint32_t wpa_uciphers[8];	/* ciphers */
	uint16_t wpa_authselcnt;	/* authentication selector cnt*/
	uint32_t wpa_authsels[8];	/* selectors */
	uint16_t wpa_caps;		/* 802.11i capabilities */
	uint16_t wpa_pmkidcnt;		/* 802.11i pmkid count */
	uint16_t wpa_pmkids[8];		/* 802.11i pmkids */
} __packed;

/* Extender Role IE */
struct ieee80211_ie_qtn_extender {
	uint8_t id;		/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;		/* 5 */
	uint8_t qtn_ie_oui[3];	/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t qtn_ie_type;	/* QTN_OUI_EXTENDER_ROLE */
	uint8_t role;		/* extender device role */
} __packed;

/* Extender Role IE */
struct ieee80211_qtn_ext_role {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;				/* 5 */
	uint8_t qtn_ie_oui[3];			/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t qtn_ie_type;			/* QTN_OUI_EXTENDER_ROLE */
	uint8_t role;				/* extender device role: MBS, RBS, NONE */
} __packed;

#define QTN_MAX_RBS_NUM		8
struct ieee80211_qtn_ext_bssid {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;				/* 59 */
	uint8_t qtn_ie_oui[3];			/* QTN_OUI - 0x00, 0x26, 0x86*/
	uint8_t qtn_ie_type;			/* QTN_OUI_EXTENDER_BSSID */
	uint8_t mbs_bssid[IEEE80211_ADDR_LEN];	/* BSSID of mbs */
	uint8_t rbs_num;
	uint8_t rbs_bssid[QTN_MAX_RBS_NUM][IEEE80211_ADDR_LEN]; /* BSSID of rbs */
} __packed;

struct ieee80211_ie_qtn_ocac_state {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
#define OCAC_STATE_IE_LEN	6
	uint8_t len;				/* 6 - QTN_OCAC_STATE_IE_LEN */
	uint8_t qtn_ie_oui[3];			/* QTN_OUI - 0x00, 0x26, 0x86 */
	uint8_t qtn_ie_type;			/* IE type - QTN_OUI_OCAC_STATE */
#define OCAC_STATE_NONE		0
#define OCAC_STATE_BACKOFF	1
#define OCAC_STATE_ONGOING	2
	uint8_t state;
	uint8_t param;				/* Use of params depends on the current stage
						 * OCAC_STATE_NONE   : Not used (set as 0)
						 * OCAC_STATE_BACKOFF: Backoff before OCAC would start (in beacon count)
						 * OCAC_STATE_ONGOING: Not used (set as 0)
						 */
} __packed;

struct ieee80211_ie_qtn_repeater {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;				/* 14 */
	uint8_t qtn_ie_oui[3];			/* QTN_OUI - 0x00, 0x26, 0x86 */
	uint8_t qtn_ie_type;			/* IE type - QTN_OUI_REPEATER_CASCADE */
	uint8_t mode;
	uint8_t intf_id;
	uint8_t level;
	uint8_t max_level;
	uint8_t stamac[6];
} __packed;
#define QTN_REPEATER_CASCADE_IE_LEN	(sizeof(struct ieee80211_ie_qtn_repeater) - 2)

/*
 * 802.11n AMPDU delimiters and frame structure
 */

/* XXX - Endianness?  */
struct ieee80211_ampdu_delim {
	uint8_t	dl_mpdulen[2];		/* only 12 bits */
	uint8_t	dl_crc;
	uint8_t	dl_uniquepat;
} __packed;

#define IEEE80211_AMPDU_DLPAT		0x4E	/* ASCII for char 'N' */
#define	IEEE80211_AMPDU_PADMAX		3

/*
 * 802.11n HT Capability IE
 */
struct ieee80211_ie_htcap {
	uint8_t	hc_id;			/* element ID */
	uint8_t	hc_len;			/* length in bytes */
	uint8_t	hc_cap[2];			/* HT capabilities */
	uint8_t	hc_ampdu;		/* A-MPDU parameters */
	uint8_t	hc_mcsset[16];		/* supported MCS set */
	uint8_t	hc_extcap[2];		/* extended HT capabilities */
	uint8_t	hc_txbf[4];		/* txbf capabilities */
	uint8_t	hc_antenna;		/* antenna capabilities */
} __packed;


/* HT capability flags */
#define	IEEE80211_HTCAP_C_LDPCCODING		0x0001
#define	IEEE80211_HTCAP_C_CHWIDTH40		0x0002
#define	IEEE80211_HTCAP_C_GREENFIELD		0x0010
#define IEEE80211_HTCAP_C_SHORTGI20		0x0020
#define IEEE80211_HTCAP_C_SHORTGI40		0x0040
#define IEEE80211_HTCAP_C_TXSTBC		0x0080
#define IEEE80211_HTCAP_C_RXSTBC		0x0100
#define IEEE80211_HTCAP_C_DELAYEDBLKACK		0x0400
#define IEEE80211_HTCAP_C_MAXAMSDUSIZE_8K	0x0800  /* 1 = 8K, 0 = 3839 bytes */
#define IEEE80211_HTCAP_C_MAXAMSDUSIZE_4K	0x0000
#define IEEE80211_HTCAP_C_DSSSCCK40		0x1000
#define IEEE80211_HTCAP_C_PSMP			0x2000
#define IEEE80211_HTCAP_C_40_INTOLERANT		0x4000
#define IEEE80211_HTCAP_C_LSIGTXOPPROT		0x8000

/* STBC defines */
#define IEEE80211_MAX_TX_STBC_SS		2

/* MCS set flags */
#define IEEE80211_HTCAP_MCS_TX_SET_DEFINED	0x01
#define IEEE80211_HTCAP_MCS_TX_RX_SET_NEQ	0x02
#define IEEE80211_HTCAP_MCS_TX_UNEQ_MOD		0x10

/* Maximum MSDU sizes */
#define	IEEE80211_MSDU_SIZE_7935			7935
#define	IEEE80211_MSDU_SIZE_3839			3839


#define IEEE80211_HT_MCS_SET_BPSK_CR_HALF		0x01
#define IEEE80211_HT_MCS_SET_QPSK_CR_HALF		0x02
#define IEEE80211_HT_MCS_SET_QPSK_CR_THREEFORTH	0x04
#define IEEE80211_HT_MCS_SET_16QAM_CR_HALF		0x08
#define IEEE80211_HT_MCS_SET_16QAM_CR_THREEFORTH	0x10
#define IEEE80211_HT_MCS_SET_64QAM_CR_TWOTHIRD	0x20
#define IEEE80211_HT_MCS_SET_64QAM_CR_THREEFORTH	0x40
#define IEEE80211_HT_MCS_SET_64QAM_CR_FIVESIXTH	0x80

/* Extended capabilities flags */
#define IEEE80211_HTCAP_E_PCO				0x0001
#define IEEE80211_HTCAP_E_PLUS_HTC			0x0400
#define IEEE80211_HTCAP_E_RD_RESPONSE		0x0800

/* Tx Beamforming flags */
#define IEEE80211_HTCAP_B_IMP_TXBF_RX		0x00000001
#define IEEE80211_HTCAP_B_STAG_SOUNDING_RX	0x00000002
#define IEEE80211_HTCAP_B_STAG_SOUNDING_TX	0x00000004
#define IEEE80211_HTCAP_B_NDP_RX			0x00000008
#define IEEE80211_HTCAP_B_NDP_TX			0x00000010
#define IEEE80211_HTCAP_B_IMP_TXBF_TX		0x00000020
#define IEEE80211_HTCAP_B_EXP_CSI_TXBF		0x00000100
#define IEEE80211_HTCAP_B_EXP_NCOMP_STEER	0x00000200
#define IEEE80211_HTCAP_B_EXP_COMP_STEER	0x00000400

/* Antenna selection flags */
#define IEEE80211_HTCAP_A_ASEL_CAPABLE		0x01
#define IEEE80211_HTCAP_A_EXP_CSI_FB_ASEL	0x02
#define IEEE80211_HTCAP_A_ANT_IND_FB_ASEL	0x04
#define IEEE80211_HTCAP_A_EXP_CSI_FB		0x08
#define IEEE80211_HTCAP_A_ANT_IND_FB		0x10
#define IEEE80211_HTCAP_A_RX_ASEL			0x20
#define IEEE80211_HTCAP_A_TX_SOUNDING_PPDU	0x40

enum {
	IEEE80211_MCS_TYPE_N = 0,
	IEEE80211_MCS_TYPE_AC,
	IEEE80211_MCS_TYPE_AX,
};

/* MCS related defines */
#define IEEE80211_11AC_MCS_VAL_ERR		-1
#define IEEE80211_HT_EQUAL_MCS_START		0
#define IEEE80211_HT_EQUAL_MCS_2SS_MAX		15
#define IEEE80211_HT_EQUAL_MCS_3SS_MAX		23
#define IEEE80211_EQUAL_MCS_32			32
#define IEEE80211_UNEQUAL_MCS_START		33
#define IEEE80211_HT_UNEQUAL_MCS_2SS_MAX	38
#define IEEE80211_HT_UNEQUAL_MCS_3SS_MAX	52
#define IEEE80211_UNEQUAL_MCS_MAX		76
#define IEEE80211_UNEQUAL_MCS_BIT		0x100
#define IEEE80211_N_MCS_VAL_MASK		0xFF
#define IEEE80211_AC_MCS_MASK			0xFF
#define IEEE80211_AC_MCS_SHIFT			8
#define IEEE80211_AC_MCS_VAL_MASK		0x0F
#define IEEE80211_AC_MCS_NSS_MASK		0xF0
#define IEEE80211_11AC_MCS_NSS_SHIFT		4
#define IEEE80211_AX_MCS_MASK			0xFF
#define IEEE80211_AX_MCS_VAL_MASK		0x0F
#define IEEE80211_AX_MCS_NSS_MASK		0xF0
#define IEEE80211_11AX_MCS_NSS_SHIFT		4
#define IEEE80211_AC_MCS_WITH_NSS_INFO		100
#define IEEE80211_AX_MCS_WITH_NSS_INFO		1000

/* 0xA/0xB/0xC is for 1024QAM */
#define QTN_1024_QAM_MCS_NUM			3
#define IEEE80211_AC_MCS_MAX			(10 + QTN_1024_QAM_MCS_NUM)
#define IEEE80211_AC_MCS_NSS_MAX		4
#define IEEE80211_AC_MCS_TBL_MAX		13

#define IEEE80211_AX_MCS_MAX			12
#define IEEE80211_AX_MCS_NSS_MAX		IEEE80211_HE_NSS_MAX

#define IEEE80211_AX_MCS_6G_MIN_RATE_MAX	3
#define IEEE80211_AX_6G_MIN_RATE_LIMIT		11 /* MCS=3, NSS=3, 0.8us GI, 26 tones */

/* B0-1 maximum rx A-MPDU factor 2^(13+Max Rx A-MPDU Factor) - 1 */
enum {
	IEEE80211_HTCAP_MAXRXAMPDU_8191,	/* (2 ^ 13) - 1*/
	IEEE80211_HTCAP_MAXRXAMPDU_16383,   /* (2 ^ 14) - 1 */
	IEEE80211_HTCAP_MAXRXAMPDU_32767,   /* (2 ^ 15) - 1*/
	IEEE80211_HTCAP_MAXRXAMPDU_65535,   /* (2 ^ 16) - 1*/
};

/* B2-4 MPDU spacing (usec) */
enum {
	IEEE80211_HTCAP_MPDUSPACING_NA,		/* No time restriction */
	IEEE80211_HTCAP_MPDUSPACING_0_25,   /* 1/4 usec */
	IEEE80211_HTCAP_MPDUSPACING_0_5,    /* 1/2 usec */
	IEEE80211_HTCAP_MPDUSPACING_1,      /* 1 usec */
	IEEE80211_HTCAP_MPDUSPACING_2,      /* 2 usec */
	IEEE80211_HTCAP_MPDUSPACING_4,      /* 4 usec */
	IEEE80211_HTCAP_MPDUSPACING_8,      /* 8 usec */
	IEEE80211_HTCAP_MPDUSPACING_16,     /* 16 usec */
};

/*
 * Rx MCS set
 * # Supported rates IE is a 10 octet bitmap - also see mcs_stream_map[]
 * Octet: 0        1        2        3        4 UEQM1  5 UEQM2  6 UEQM3  7 UEQM4  8 UEQM5  9 UEQM6
 * NSS:   11111111 22222222 33333333 44444444 02222223 33333333 33333444 44444444 44444444 44444...
 * MCS:   0        8        16       24       32       40       48       56       64       72  76
 */
enum {
	IEEE80211_HT_MCSSET_20_40_NSS1,		/* CBW = 20/40 MHz, Nss = 1, Nes = 1, EQM/ No EQM */
	IEEE80211_HT_MCSSET_20_40_NSS2,		/* CBW = 20/40 MHz, Nss = 2, Nes = 1, EQM */
	IEEE80211_HT_MCSSET_20_40_NSS3,		/* CBW = 20/40 MHz, Nss = 3, Nes = 1, EQM */
	IEEE80211_HT_MCSSET_20_40_NSS4,		/* CBW = 20/40 MHz, Nss = 4, Nes = 1, EQM */
	IEEE80211_HT_MCSSET_20_40_UEQM1,	/* MCS 32 and UEQM MCSs 33 - 39 */
	IEEE80211_HT_MCSSET_20_40_UEQM2,	/* UEQM MCSs 40 - 47 */
	IEEE80211_HT_MCSSET_20_40_UEQM3,        /* UEQM MCSs 48 - 55 */
	IEEE80211_HT_MCSSET_20_40_UEQM4,        /* UEQM MCSs 56 - 63 */
	IEEE80211_HT_MCSSET_20_40_UEQM5,        /* UEQM MCSs 64 - 71 */
	IEEE80211_HT_MCSSET_20_40_UEQM6,        /* UEQM MCSs 72 - 76 plus 3 reserved bits */
};

#define IEEE80211_HT_MCSSET_20_40_UEQM1_2SS	0x7E

#define IEEE80211_HT_MCSSET_20_40_UEQM1_3SS	0x80
#define IEEE80211_HT_MCSSET_20_40_UEQM2_3SS	0xFF
#define IEEE80211_HT_MCSSET_20_40_UEQM3_3SS	0x1F

#define IEEE80211_HT_MCSSET_20_40_UEQM3_4SS	0xE0
#define IEEE80211_HT_MCSSET_20_40_UEQM4_4SS	0xFF
#define IEEE80211_HT_MCSSET_20_40_UEQM5_4SS	0xFF
#define IEEE80211_HT_MCSSET_20_40_UEQM6_4SS	0x1F

#define IEEE80211_HT_HAS_1SS_ONLY(mcsset) \
		(mcsset[IEEE80211_HT_MCSSET_20_40_NSS1] && \
			!mcsset[IEEE80211_HT_MCSSET_20_40_NSS2])

#define IEEE80211_HT_HAS_2SS_UEQM_MCS(mcsset) \
		(mcsset[IEEE80211_HT_MCSSET_20_40_UEQM1] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM1_2SS)

#define IEEE80211_HT_HAS_3SS_UEQM_MCS(mcsset) \
		((mcsset[IEEE80211_HT_MCSSET_20_40_UEQM1] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM1_3SS) ||	\
		 (mcsset[IEEE80211_HT_MCSSET_20_40_UEQM2] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM2_3SS) ||	\
		 (mcsset[IEEE80211_HT_MCSSET_20_40_UEQM3] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM3_3SS))

#define IEEE80211_HT_HAS_4SS_UEQM_MCS(mcsset) \
		((mcsset[IEEE80211_HT_MCSSET_20_40_UEQM3] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM3_4SS) ||	\
		 (mcsset[IEEE80211_HT_MCSSET_20_40_UEQM4] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM4_4SS) ||	\
		 (mcsset[IEEE80211_HT_MCSSET_20_40_UEQM5] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM5_4SS) ||	\
		 (mcsset[IEEE80211_HT_MCSSET_20_40_UEQM6] &	\
			IEEE80211_HT_MCSSET_20_40_UEQM6_4SS))

#define IEEE80211_HT_IS_1SS_NODE(mcsset) \
		((mcsset[IEEE80211_HT_MCSSET_20_40_NSS1] != 0) && \
		(mcsset[IEEE80211_HT_MCSSET_20_40_NSS2] == 0))

#define IEEE80211_HT_IS_2SS_NODE(mcsset) \
		((mcsset[IEEE80211_HT_MCSSET_20_40_NSS2] != 0) && \
		(mcsset[IEEE80211_HT_MCSSET_20_40_NSS3] == 0))

#define IEEE80211_HT_IS_3SS_NODE(mcsset) \
		((mcsset[IEEE80211_HT_MCSSET_20_40_NSS3] != 0) && \
		(mcsset[IEEE80211_HT_MCSSET_20_40_NSS4] == 0))

#define IEEE80211_HT_IS_4SS_NODE(mcsset) \
		(mcsset[IEEE80211_HT_MCSSET_20_40_NSS4] != 0)

/* B2-3 Maximum Tx spatial streams */
enum {
	IEEE80211_HTCAP_MCS_ONE_TX_SS,		/* One spatial stream */
	IEEE80211_HTCAP_MCS_TWO_TX_SS,		/* Two spatial streams */
	IEEE80211_HTCAP_MCS_THREE_TX_SS,	/* Three spatial streams */
	IEEE80211_HTCAP_MCS_FOUR_TX_SS		/* Four spatial streams */
};

/* B2-3 power save mode */
enum {
	IEEE80211_HTCAP_C_MIMOPWRSAVE_STATIC = 0,	/* No MIMO (static mode) */
	IEEE80211_HTCAP_C_MIMOPWRSAVE_DYNAMIC,		/* Precede MIMO with RTS */
	IEEE80211_HTCAP_C_MIMOPWRSAVE_NA,		/* Not applicable        */
	IEEE80211_HTCAP_C_MIMOPWRSAVE_NONE		/* No limitation on MIMO (SM power save disabled) */
};

/* B8-9 Rx STBC Mode */
enum {
	IEEE80211_HTCAP_C_RXSTBC_NONE,			/* No STBC SS */
	IEEE80211_HTCAP_C_RXSTBC_ONE_SS,		/* One STBC SS */
	IEEE80211_HTCAP_C_RXSTBC_TWO_SS,		/* Two STBC SS */
	IEEE80211_HTCAP_C_RXSTBC_THREE_SS		/* Three STBC SS */
};

/* B1-2 PCO transition time */
enum {
	IEEE80211_HTCAP_E_PCO_NONE,				/* No transition */
	IEEE80211_HTCAP_E_PCO_FOUR_HUNDRED_US,	/* 400 us */
	IEEE80211_HTCAP_E_PCO_ONE_HALF_MS,		/* 1.5 ms */
	IEEE80211_HTCAP_E_PCO_FIVE_MS			/* 5 ms */
};

/* B8-9 MCS feedback */
enum {
	IEEE80211_HTCAP_E_MCS_FB_NONE,			/* No feedback */
	IEEE80211_HTCAP_E_MCS_FB_NA,			/* Reserved */
	IEEE80211_HTCAP_E_MCS_FB_UNSOLICITED,	/* Unsolicited feedback only*/
	IEEE80211_HTCAP_E_MCS_FB_SOLICITED		/* Solicited and unsolicited feedback */
};

/* B6-7 Calibration */
enum {
	IEEE80211_HTCAP_B_CALIBRATION_NONE,			/* No support */
	IEEE80211_HTCAP_B_CALIBRATION_RESP_ONLY,	/* Response only */
	IEEE80211_HTCAP_B_CALIBRATION_NA,			/* Reserved */
	IEEE80211_HTCAP_B_CALIBRATION_REQ_RESP		/* Request and response */
};

/* B11-12 explicit CSI TxBF feedback, B13-14 explicit non compressed TxBF,
 * B15-16 explicit compressed TxBF
 */
enum {
	IEEE80211_HTCAP_B_CAPABLE_NONE,			/* No support */
	IEEE80211_HTCAP_B_CAPABLE_DELAYED,		/* delayed response only */
	IEEE80211_HTCAP_B_CAPABLE_IMMEDIATE,	/* immediate response only */
	IEEE80211_HTCAP_B_CAPABLE_BOTH			/* both delayed and immediate response */
};

/* B17-18 Grouping */
enum {
	IEEE80211_HTCAP_B_GROUPING_NONE,		/* No support */
	IEEE80211_HTCAP_B_GROUPING_ONE_TWO,		/* groups 1 and 2 */
	IEEE80211_HTCAP_B_GROUPING_ONE_FOUR,	/* groups 1 and 4 */
	IEEE80211_HTCAP_B_GROUPING_ONE_TWO_FOUR	/* groups 1, 2 and 4 */
};

/* B19-20 CSI number of beamforming antennas, B21-22 non compressed number of beamforming
 * antennas, B23-24 compressed number of beamforming antennas
 */
enum {
	IEEE80211_HTCAP_B_ANTENNAS_ONE,		/* Single antenna sounding */
	IEEE80211_HTCAP_B_ANTENNAS_TWO,		/* 2 antenna sounding */
	IEEE80211_HTCAP_B_ANTENNAS_THREE,	/* 3 antenna sounding */
	IEEE80211_HTCAP_B_ANTENNAS_FOUR		/* 4 antenna sounding */
};

/* B25-26 CSI Max number of beamformer rows */
enum {
	IEEE80211_HTCAP_B_CSI_ONE_ROW,
	IEEE80211_HTCAP_B_CSI_TWO_ROWS,
	IEEE80211_HTCAP_B_CSI_THREE_ROWS,
	IEEE80211_HTCAP_B_CSI_FOUR_ROWS
};

/* B27-28 channel estimation capability */
enum {
	IEEE80211_HTCAP_B_ST_STREAM_ONE,	/* one space time stream */
	IEEE80211_HTCAP_B_ST_STREAM_TWO,	/* two space time streams */
	IEEE80211_HTCAP_B_ST_STREAM_THREE,	/* three space time streams */
	IEEE80211_HTCAP_B_ST_STREAM_FOUR	/* four space time streams */
};

/* HT NSS */
enum ieee80211_ht_nss {
	IEEE80211_HT_NSS1 = 1,
	IEEE80211_HT_NSS2 = 2,
	IEEE80211_HT_NSS3 = 3,
	IEEE80211_HT_NSS4 = 4
};

/* HT capability macros */

/* get macros */
/* A-MPDU spacing  B2-B4 */
#define IEEE80211_HTCAP_MIN_AMPDU_SPACING(htcap) \
	(((htcap)->hc_ampdu & 0x1c) >> 2)
/* max RX A-MPDU length  B0-B1 */
#define IEEE80211_HTCAP_MAX_AMPDU_LEN(htcap) \
	(((htcap)->hc_ampdu & 0x03))
/* highest supported data rate, B0-B7 in set 10, B0-B1 in set 11 */
#define IEEE80211_HTCAP_HIGHEST_DATA_RATE(htcap) \
	(((htcap)->hc_mcsset[10]) | (((htcap)->hc_mcsset[11] & 0x3) << 8))
/* MCS parameters (all bits)*/
#define IEEE80211_HTCAP_MCS_PARAMS(htcap) \
	((htcap)->hc_mcsset[12] & 0x1F)
/* MCS maximum spatial streams, B2-B3 in set 12 */
#define IEEE80211_HTCAP_MCS_STREAMS(htcap) \
	(((htcap)->hc_mcsset[12] & 0xC) >> 2)
/* MCS set value (all bits) */
#define IEEE80211_HTCAP_MCS_VALUE(htcap,_set) \
	((htcap)->hc_mcsset[_set])
/* HT capabilities (all bits) */
#define IEEE80211_HTCAP_CAPABILITIES(htcap) \
	(((htcap)->hc_cap[0]) | ((htcap)->hc_cap[1] << 8))
/* B3-4 power save mode */
#define IEEE80211_HTCAP_PWRSAVE_MODE(htcap) \
	(((htcap)->hc_cap[0] & 0x0C) >> 2)
/* B8-9 Rx STBC MODE */
#define IEEE80211_HTCAP_RX_STBC_MODE(htcap) \
	((htcap)->hc_cap[1] & 0x3)
/* HT extended capabilities (all bits) */
#define IEEE80211_HTCAP_EXT_CAPABILITIES(htcap) \
	((htcap)->hc_extcap)
/* B1-2 PCO transition time */
#define IEEE80211_HTCAP_PCO_TRANSITION(htcap) \
	(((htcap)->hc_extcap & 0x6) >> 1)
/* B8-9 MCS feedback type */
#define IEEE80211_HTCAP_MCS_FEEDBACK_TYPE(htcap) \
	(((htcap)->hc_extcap & 0x300) >> 8)
/* HT TxBeamForming (bits 0-13) */
#define IEEE80211_HTCAP_TXBF_CAPABILITIES(htcap) \
	((htcap)->hc_txbf[0] | ((htcap)->hc_txbf[1] << 8))
/* HT TxBeamForming (bits 14-31) */
#define IEEE80211_HTCAP_TXBF_CAPABILITIES_EXTN(htcap) \
	((htcap)->hc_txbf[2] | ((htcap)->hc_txbf[3] << 8))
/* B6-7 Calibration */
#define IEEE80211_HTCAP_CALIBRATION(htcap) \
	(((htcap)->hc_txbf[0] & 0xC0) >> 6)
/* B11-12 explicit CSI TxBF feedback*/
#define IEEE80211_HTCAP_EXP_CSI_TXBF(htcap) \
	(((htcap)->hc_txbf[1] & 0x18) >> 3)
/* B13-14 explicit non compressed TxBF */
#define IEEE80211_HTCAP_EXP_NCOMP_TXBF(htcap) \
	(((htcap)->hc_txbf[1] & 0x60) >> 5)
/* B15-16 explicit compressed TxBF */
#define IEEE80211_HTCAP_EXP_COMP_TXBF(htcap) \
	((((htcap)->hc_txbf[1] & 0x80) >> 7) | (((htcap)->hc_txbf[2] & 0x01) << 1))
/* B17-18 Grouping */
#define IEEE80211_HTCAP_GROUPING(htcap) \
	(((htcap)->hc_txbf[2] & 0x6) >> 1)
/* B19-20 CSI number of beamforming antennas */
#define IEEE80211_HTCAP_CSI_NUM_BF(htcap) \
	(((htcap)->hc_txbf[2] & 0x18) >> 3)
/* B21-22 non compressed number of beamforming antennas */
#define IEEE80211_HTCAP_NCOM_NUM_BF(htcap) \
	(((htcap)->hc_txbf[2] & 0x60) >> 5)
/* B23-24 compressed number of beamforming antennas */
#define IEEE80211_HTCAP_COMP_NUM_BF(htcap) \
	((((htcap)->hc_txbf[2] & 0x80) >> 7) | (((htcap)->hc_txbf[3] & 0x01) << 1))
/* B25-26 CSI Max number of beamformer rows */
#define IEEE80211_HTCAP_CSI_BF_ROWS(htcap) \
	(((htcap)->hc_txbf[3] & 0x6) >> 1)
/* B27-28 channel estimation capability */
#define IEEE80211_HTCAP_CHAN_EST(htcap) \
	(((htcap)->hc_txbf[3] & 0x18) >> 3)

/* set macros */
/* A-MPDU spacing  B2-B4 */
#define IEEE80211_HTCAP_SET_AMPDU_SPACING(htcap,_d) \
	((htcap)->hc_ampdu = (((htcap)->hc_ampdu & ~0x1c)  | ((_d) << 2)))
/* max RX A-MPDU length  B0-B1 */
#define IEEE80211_HTCAP_SET_AMPDU_LEN(htcap,_f)	\
	((htcap)->hc_ampdu = (((htcap)->hc_ampdu & ~0x03)  | (_f)))
/* highest supported data rate, B0-B7 in set 10, B0-B1 in set 11) */
#define IEEE80211_HTCAP_SET_HIGHEST_DATA_RATE(htcap,_r) \
	((htcap)->hc_mcsset[10] = ((_r) & 0xFF)); \
	((htcap)->hc_mcsset[11] = ((_r) & 0x3FF) >> 8)
/* MCS set parameters (all bits) */
#define IEEE80211_HTCAP_SET_MCS_PARAMS(htcap,_p) \
	((htcap)->hc_mcsset[12] = (_p & 0x1F))
/* MCS maximum spatial streams, B2-B3 in set 12 */
#define IEEE80211_HTCAP_SET_MCS_STREAMS(htcap,_s) \
	((htcap)->hc_mcsset[12] = ((htcap)->hc_mcsset[12] & ~0xC)| (_s << 2))
/* MCS set value (all bits) */
#define IEEE80211_HTCAP_SET_MCS_VALUE(htcap,_set,_value) \
	((htcap)->hc_mcsset[_set] = (_value & 0xFF))
/* HT capabilities (all bits) */
#define IEEE80211_HTCAP_SET_CAPABILITIES(htcap,_cap) \
	(htcap)->hc_cap[0] = (_cap & 0x00FF); \
	(htcap)->hc_cap[1] = ((_cap & 0xFF00) >> 8)
/* B2-B3 power save mode */
#define IEEE80211_HTCAP_SET_PWRSAVE_MODE(htcap,_m) \
	((htcap)->hc_cap[0] = (((htcap)->hc_cap[0] & ~0xC) | ((_m) << 2)))
/* B8-9 Rx STBC MODE */
#define IEEE80211_HTCAP_SET_RX_STBC_MODE(htcap,_m) \
	((htcap)->hc_cap[1] = (((htcap)->hc_cap[1] & ~0x3) | (_m) ))
/* HT extended capabilities (all bits) */
#define IEEE80211_HTCAP_SET_EXT_CAPABILITIES(htcap,_cap) \
	((htcap)->hc_extcap = (_cap & 0xFFFF))
/* B1-2 PCO transition time */
#define IEEE80211_HTCAP_SET_PCO_TRANSITION(htcap,_t) \
	((htcap)->hc_extcap = (((htcap)->hc_extcap & ~0x6) | ((_t) << 1)))
/* B8-9 MCS feedback type */
#define IEEE80211_HTCAP_SET_MCS_FEEDBACK_TYPE(htcap,_t) \
	((htcap)->hc_extcap = (((htcap)->hc_extcap & ~0x300) | ((_t) << 8)))
/* HT TxBeamForming (all bits ) */
#define IEEE80211_HTCAP_SET_TXBF_CAPABILITIES(htcap,_cap) \
	(htcap)->hc_txbf[0] = ((_cap) & 0x00FF); \
	(htcap)->hc_txbf[1] = (((_cap) & 0xFF00) >> 8)
/* B6-7 Calibration */
#define IEEE80211_HTCAP_SET_CALIBRATION(htcap,_t) \
	((htcap)->hc_txbf[0] = (((htcap)->hc_txbf[0] & ~0xC0) | ((_t) << 6)))
/* B11-12 explicit CSI TxBF feedback*/
#define IEEE80211_HTCAP_SET_EXP_CSI_TXBF(htcap,_t) \
	((htcap)->hc_txbf[1] = (((htcap)->hc_txbf[1] & ~0x18) | ((_t) << 3)))
/* B13-14 explicit non compressed TxBF */
#define IEEE80211_HTCAP_SET_EXP_NCOMP_TXBF(htcap,_t) \
	((htcap)->hc_txbf[1] = (((htcap)->hc_txbf[1] & ~0x60) | ((_t) << 5)))
/* B15-16 explicit compressed TxBF */
#define IEEE80211_HTCAP_SET_EXP_COMP_TXBF(htcap,_t) \
	(htcap)->hc_txbf[1] = (((htcap)->hc_txbf[1] & ~0x80) | ((((_t) & 0x01) << 7))); \
	(htcap)->hc_txbf[2] = (((htcap)->hc_txbf[2] & ~0x01) | ((_t) >> 1))
/* B17-18 Grouping */
#define IEEE80211_HTCAP_SET_GROUPING(htcap,_t) \
	((htcap)->hc_txbf[2] = (((htcap)->hc_txbf[2] & ~0x6) | ((_t) << 1)))
/* B19-20 CSI number of beamforming antennas */
#define IEEE80211_HTCAP_SET_CSI_NUM_BF(htcap,_t) \
	((htcap)->hc_txbf[2] = (((htcap)->hc_txbf[2] & ~0x18) | ((_t) << 3)))
/* B21-22 non compressed number of beamforming antennas */
#define IEEE80211_HTCAP_SET_NCOMP_NUM_BF(htcap,_t) \
	((htcap)->hc_txbf[2] = (((htcap)->hc_txbf[2] & ~0x60) | ((_t) << 5)))
/* B23-24 compressed number of beamforming antennas */
#define IEEE80211_HTCAP_SET_COMP_NUM_BF(htcap,_t) \
	(htcap)->hc_txbf[2] = (((htcap)->hc_txbf[2] & ~0x80) | (((_t) & 0x01) << 7)); \
	(htcap)->hc_txbf[3] = (((htcap)->hc_txbf[3] & ~0x01) | ((_t) >> 1))
/* B25-26 CSI Max number of beamformer rows */
#define IEEE80211_HTCAP_SET_CSI_BF_ROWS(htcap,_t) \
	((htcap)->hc_txbf[3] = (((htcap)->hc_txbf[3] & ~0x6) | ((_t) << 1)))
/* B27-28 channel estimation capability */
#define IEEE80211_HTCAP_SET_CHAN_EST(htcap,_t) \
	((htcap)->hc_txbf[3] = (((htcap)->hc_txbf[3] & ~0x18) | ((_t) << 3)))

#define IEEE80211_HTCAP_TXBFEE_MASK_BYTE0	(0xCB)
#define IEEE80211_HTCAP_TXBFEE_MASK_BYTE1	(0xF8)
#define IEEE80211_HTCAP_TXBFEE_MASK_BYTE2	(0xFF)
#define IEEE80211_HTCAP_TXBFEE_MASK_BYTE3	(0x1F)

#define IEEE80211_HTCAP_TXBFER_MASK_BYTE0	(0x34)
#define IEEE80211_HTCAP_TXBFER_MASK_BYTE1	(0x07)
#define IEEE80211_HTCAP_TXBFER_MASK_BYTE2	(0x00)
#define IEEE80211_HTCAP_TXBFER_MASK_BYTE3	(0x00)

/* Clear TX beamforee fields in HTCAP IE*/
#define IEEE80211_HTCAP_CLEAR_BFORMEE(hc_txbf) \
	hc_txbf[0] &= ((~IEEE80211_HTCAP_TXBFEE_MASK_BYTE0) & 0xff); \
	hc_txbf[1] &= ((~IEEE80211_HTCAP_TXBFEE_MASK_BYTE1) & 0xff); \
	hc_txbf[2] &= ((~IEEE80211_HTCAP_TXBFEE_MASK_BYTE2) & 0xff); \
	hc_txbf[3] &= ((~IEEE80211_HTCAP_TXBFEE_MASK_BYTE3) & 0xff);

/* Clear TX beamforer fields in HTCAP IE*/
#define IEEE80211_HTCAP_CLEAR_BFORMER(hc_txbf) \
	hc_txbf[0] &= ((~IEEE80211_HTCAP_TXBFER_MASK_BYTE0) & 0xff); \
	hc_txbf[1] &= ((~IEEE80211_HTCAP_TXBFER_MASK_BYTE1) & 0xff); \
	hc_txbf[2] &= ((~IEEE80211_HTCAP_TXBFER_MASK_BYTE2) & 0xff); \
	hc_txbf[3] &= ((~IEEE80211_HTCAP_TXBFER_MASK_BYTE3) & 0xff);

/*Restore TX beamforee fields in HTCAP IE*/
#define IEEE80211_HTCAP_COPY_BFORMEE(hc_txbf, txbf) \
	hc_txbf[0] |= (txbf[0] & IEEE80211_HTCAP_TXBFEE_MASK_BYTE0); \
	hc_txbf[1] |= (txbf[1] & IEEE80211_HTCAP_TXBFEE_MASK_BYTE1); \
	hc_txbf[2] |= (txbf[2] & IEEE80211_HTCAP_TXBFEE_MASK_BYTE2); \
	hc_txbf[3] |= (txbf[3] & IEEE80211_HTCAP_TXBFEE_MASK_BYTE3);

/*Restore TX beamforee fields in HTCAP IE*/
#define IEEE80211_HTCAP_COPY_BFORMER(hc_txbf, txbf) \
	hc_txbf[0] |= (txbf[0] & IEEE80211_HTCAP_TXBFER_MASK_BYTE0); \
	hc_txbf[1] |= (txbf[1] & IEEE80211_HTCAP_TXBFER_MASK_BYTE1); \
	hc_txbf[2] |= (txbf[2] & IEEE80211_HTCAP_TXBFER_MASK_BYTE2); \
	hc_txbf[3] |= (txbf[3] & IEEE80211_HTCAP_TXBFER_MASK_BYTE3);

/*
 * 802.11n HT Information IE
 */
struct ieee80211_ie_htinfo {
	uint8_t	hi_id;			/* element ID */
	uint8_t	hi_len;			/* length in bytes */
	uint8_t	hi_ctrlchannel;	/* control channel */
	uint8_t	hi_byte1;		/* ht ie byte 1 */
	uint8_t	hi_byte2;		/* ht ie byte 2 */
	uint8_t	hi_byte3;		/* ht ie byte 3 */
	uint8_t	hi_byte4;		/* ht ie byte 4 */
	uint8_t	hi_byte5;		/* ht ie byte 5 */
	uint8_t	hi_basicmcsset[16];	/* basic MCS set */
} __packed;

#define	IEEE80211_HTINFO_CHOFF_SCN			0
#define	IEEE80211_HTINFO_CHOFF_SCA			1
#define	IEEE80211_HTINFO_CHOFF_SCB			3

#define IEEE80211_HTINFO_B1_SEC_CHAN_OFFSET		0x03
#define IEEE80211_HTINFO_B1_REC_TXCHWIDTH_40		0x04
#define IEEE80211_HTINFO_B1_RIFS_MODE			0x08
#define IEEE80211_HTINFO_B1_CONTROLLED_ACCESS		0x10
#define IEEE80211_HTINFO_B2_NON_GF_PRESENT		0x04
#define IEEE80211_HTINFO_B2_OBSS_PROT			0x10
#define IEEE80211_HTINFO_B4_DUAL_BEACON			0x40
#define IEEE80211_HTINFO_B4_DUAL_CTS			0x80
#define IEEE80211_HTINFO_B5_STBC_BEACON			0x01
#define IEEE80211_HTINFO_B5_LSIGTXOPPROT		0x02
#define IEEE80211_HTINFO_B5_PCO_ACTIVE			0x04
#define IEEE80211_HTINFO_B5_40MHZPHASE			0x08

/* get macros */
/* control channel (all bits) */
#define IEEE80211_HTINFO_PRIMARY_CHANNEL(htie) \
	(htie->hi_ctrlchannel)
/* byte 1 (all bits) */
#define IEEE80211_HTINFO_BYTE_ONE(htie) \
	(htie->hi_byte1)
/* byte 2 (all bits) */
#define IEEE80211_HTINFO_BYTE_TWO(htie) \
	(htie->hi_byte2)
/* byte 3 (all bits) */
#define IEEE80211_HTINFO_BYTE_THREE(htie) \
	(htie->hi_byte3)
/* byte 4 (all bits) */
#define IEEE80211_HTINFO_BYTE_FOUR(htie) \
	(htie->hi_byte4)
/* byte 5 (all bits) */
#define IEEE80211_HTINFO_BYTE_FIVE(htie) \
	(htie->hi_byte5)
/* B5-B7, byte 1 */
#define IEEE80211_HTINFO_B1_SIGRANULARITY(htie) \
	(((htie)->hi_byte1 & 0xe0) >> 5)
/* B0-B1, byte 1 */
#define IEEE80211_HTINFO_B1_EXT_CHOFFSET(htie) \
	(((htie)->hi_byte1 & 0x3))
/* B0-B1, byte 2 */
#define IEEE80211_HTINFO_B2_OP_MODE(htie) \
	(((htie)->hi_byte2 & 0x3))
/* MCS set value (all bits) */
#define IEEE80211_HTINFO_BASIC_MCS_VALUE(htie,_set) \
	((htie)->hi_basicmcsset[_set])

/* set macros */
/* control channel (all bits) */
#define IEEE80211_HTINFO_SET_PRIMARY_CHANNEL(htie,_c) \
	(htie->hi_ctrlchannel = _c)
/* byte 1 (all bits) */
#define IEEE80211_HTINFO_SET_BYTE_ONE(htie,_b) \
	(htie->hi_byte1 = _b)
/* byte 2 (all bits) */
#define IEEE80211_HTINFO_SET_BYTE_TWO(htie,_b) \
	(htie->hi_byte2 = _b)
/* byte 3 (all bits) */
#define IEEE80211_HTINFO_SET_BYTE_THREE(htie,_b) \
	(htie->hi_byte3 = _b)
/* byte 4 (all bits) */
#define IEEE80211_HTINFO_SET_BYTE_FOUR(htie,_b) \
	(htie->hi_byte4 = _b)
/* byte 5 (all bits) */
#define IEEE80211_HTINFO_SET_BYTE_FIVE(htie,_b) \
	(htie->hi_byte5 = _b)
/* B5-B7, byte 1 */
#define IEEE80211_HTINFO_B1_SET_SIGRANULARITY(htie,_g)			\
	((htie)->hi_byte1 = (((htie)->hi_byte1 & ~0xe0)  |((_g) << 5) ))
/* B0-B1, byte 1 */
#define IEEE80211_HTINFO_B1_SET_EXT_CHOFFSET(htie,_off)					\
	((htie)->hi_byte1 = (((htie)->hi_byte1 & ~0x03)  |(_off)))
/* B0-B1, byte 2 */
#define IEEE80211_HTINFO_B2_SET_OP_MODE(htie,_m)									\
	((htie)->hi_byte2 = (((htie)->hi_byte2 & ~0x3) | ((_m) )))
/* Basic MCS set value (all bits) */
#define IEEE80211_HTINFO_SET_BASIC_MCS_VALUE(htie,_set,_value) \
	((htie)->hi_basicmcsset[_set] = (_value & 0xFF))


/* optional subelement IDs for Multiple BSSID */
enum {
	IEEE80211_MBSSID_NON_TRANS_PROFILE	 = 0,
	IEEE80211_MBSSID_RESERVE_1		 = 1,
	IEEE80211_MBSSID_VENDOR			= 221,
	IEEE80211_MBSSID_RESERVE_2		= 222
};
/* extension channel offset (2 bit signed number) */
enum {
	IEEE80211_HTINFO_EXTOFFSET_NA	 = 0,	/* 0  no extension channel is present */
	IEEE80211_HTINFO_EXTOFFSET_ABOVE = 1,   /* +1 extension channel above control channel */
	IEEE80211_HTINFO_EXTOFFSET_UNDEF = 2,   /* -2 undefined */
	IEEE80211_HTINFO_EXTOFFSET_BELOW = 3	/* -1 extension channel below control channel*/
};

/* operating mode */
enum {
	IEEE80211_HTINFO_OPMODE_NO_PROT,			/* no protection */
	IEEE80211_HTINFO_OPMODE_HT_PROT_NON_MEM,	/* protection required (Legacy device present in other BSS) */
	IEEE80211_HTINFO_OPMODE_HT_PROT_20_ONLY,	/* protection required ( One 20 MHZ only HT device is present in 20/40 BSS) */
	IEEE80211_HTINFO_OPMODE_HT_PROT_MIXED,		/* protection required (Legacy device is present in this BSS) */
};

/* signal granularity */
enum {
	IEEE80211_HTINFO_SIGRANULARITY_5,	/* 5 ms */
	IEEE80211_HTINFO_SIGRANULARITY_10,	/* 10 ms */
	IEEE80211_HTINFO_SIGRANULARITY_15,	/* 15 ms */
	IEEE80211_HTINFO_SIGRANULARITY_20,	/* 20 ms */
	IEEE80211_HTINFO_SIGRANULARITY_25,	/* 25 ms */
	IEEE80211_HTINFO_SIGRANULARITY_30,	/* 30 ms */
	IEEE80211_HTINFO_SIGRANULARITY_35,	/* 35 ms */
	IEEE80211_HTINFO_SIGRANULARITY_40,	/* 40 ms */
};

/*
 * Management information element payloads.
 */
enum {
	IEEE80211_ELEMID_SSID		= 0,
	IEEE80211_ELEMID_RATES		= 1,
	IEEE80211_ELEMID_FHPARMS	= 2,
	IEEE80211_ELEMID_DSPARMS	= 3,
	IEEE80211_ELEMID_CFPARMS	= 4,
	IEEE80211_ELEMID_TIM		= 5,
	IEEE80211_ELEMID_IBSSPARMS	= 6,
	IEEE80211_ELEMID_COUNTRY	= 7,
	IEEE80211_ELEMID_REQINFO	= 10,
	IEEE80211_ELEMID_BSS_LOAD	= 11,
	IEEE80211_ELEMID_EDCA		= 12,
	IEEE80211_ELEMID_CHALLENGE	= 16,
	/* 17-31 reserved for challenge text extension */
	IEEE80211_ELEMID_PWRCNSTR	= 32,
	IEEE80211_ELEMID_PWRCAP		= 33,
	IEEE80211_ELEMID_TPCREQ		= 34,
	IEEE80211_ELEMID_TPCREP		= 35,
	IEEE80211_ELEMID_SUPPCHAN	= 36,
	IEEE80211_ELEMID_CHANSWITCHANN	= 37,
	IEEE80211_ELEMID_MEASREQ	= 38,
	IEEE80211_ELEMID_MEASREP	= 39,
	IEEE80211_ELEMID_QUIET		= 40,
	IEEE80211_ELEMID_IBSSDFS	= 41,
	IEEE80211_ELEMID_ERP		= 42,
	IEEE80211_ELEMID_HTCAP		= 45,
	IEEE80211_ELEMID_QOSCAP		= 46,
	IEEE80211_ELEMID_RSN		= 48,
	IEEE80211_ELEMID_XRATES		= 50,
	IEEE80211_ELEMID_AP_CHAN_REP	= 51,
	IEEE80211_ELEMID_NEIGHBOR_REP	= 52,
	IEEE80211_ELEMID_MOBILITY_DOMAIN = 54,
	IEEE80211_ELEMID_FTIE		= 55,
	IEEE80211_ELEMID_TIMEOUT_INT	= 56,
	IEEE80211_ELEMID_DSE_REG_LOCT	= 58,
	IEEE80211_ELEMID_REG_CLASSES	= 59,
	IEEE80211_ELEMID_HTINFO		= 61,
	IEEE80211_ELEMID_SEC_CHAN_OFF	= 62,	/* Secondary Channel Offset */
	IEEE80211_ELEMID_AVG_DELAY	= 63,	/* BSS Average Access Delay */
	IEEE80211_ELEMID_AVAIL_CAP	= 67,	/* BSS Available Admission Capacity */
	IEEE80211_ELEMID_AC_DELAY	= 68,	/* BSS AC Access Delay */
	IEEE80211_ELEMID_TIME_ADVER	= 69,
	IEEE80211_ELEMID_RM_ENABLED	= 70,
	IEEE80211_ELEMID_MULTI_BSSID	= 71,
	IEEE80211_ELEMID_20_40_BSS_COEX = 72,	/* 20/40 BSS Coexistence */
	IEEE80211_ELEMID_20_40_IT_CH_REP	= 73,  /* 20/40 BSS Intolerant channel report */
	IEEE80211_ELEMID_OBSS_SCAN	= 74,   /* Overlapping BSS scan parameter  */
	IEEE80211_ELEMID_EVENT_REPORT	= 79,
	IEEE80211_ELEMID_NON_TRANS_BSSID_CAP	= 83,
	IEEE80211_ELEMID_SSID_LIST	= 84,
	IEEE80211_ELEMID_MULTI_BSSID_INDEX	= 85,
	IEEE80211_ELEMID_MAX_IDLE	= 90,
	IEEE80211_ELEMID_TDLS_LINK_ID	= 101, /* TDLS Link Identifier */
	IEEE80211_ELEMID_TDLS_WKUP_SCHED	= 102, /* TDLS Wakeup Schedule */
	IEEE80211_ELEMID_TDLS_CS_TIMING	= 104, /* TDLS Channel Switch Timing */
	IEEE80211_ELEMID_TDLS_PTI_CTRL	= 105, /* TDLS PTI Control */
	IEEE80211_ELEMID_TDLS_PU_BUF_STAT	= 106, /* TDLS PU Buffer Status */
	IEEE80211_ELEMID_INTERWORKING	= 107,
	IEEE80211_ELEMID_ADVERT_PROTO = 108,
	IEEE80211_ELEMID_ROAM_CONSORTIUM = 111,
	IEEE80211_ELEMID_ALERT_IDENTIFIER = 112,
	IEEE80211_ELEMID_MESH_CS_PARAM	= 118,
	IEEE80211_ELEMID_EXTCAP		= 127,
	/* 128-129 proprietary elements used by Agere chipsets */
	IEEE80211_ELEMID_AGERE1		= 128,
	IEEE80211_ELEMID_AGERE2		= 129,
	IEEE80211_ELEMID_TPC		= 150,
	IEEE80211_ELEMID_CCKM		= 156,
	IEEE80211_ELEMID_MULTI_BAND	= 158,
	/* 191-199 Table 8-54-Element IDs in Std 802.11ac-2013 */
	IEEE80211_ELEMID_VHTCAP		= 191,
	IEEE80211_ELEMID_VHTOP		= 192,
	IEEE80211_ELEMID_EXTBSSLOAD	= 193,
	IEEE80211_ELEMID_WBWCHANSWITCH	= 194,
	IEEE80211_ELEMID_TPE		= 195,
	IEEE80211_ELEMID_CHANSWITCHWRP	= 196,
	IEEE80211_ELEMID_AID		= 197,
	IEEE80211_ELEMID_QUIETCHAN	= 198,
	IEEE80211_ELEMID_OPMOD_NOTIF	= 199,
	IEEE80211_ELEMID_RED_NEIGH_RPT	= 201,	/* Reduced Neighbor Report */
	IEEE80211_ELEMID_LOCATION	= 204,
	IEEE80211_ELEMID_FTM_PARAM	= 206,
	IEEE80211_ELEMID_TWT		= 216,
	/* Vendor Specific */
	IEEE80211_ELEMID_VENDOR		= 221,	/* vendor private */
	/* 222-254 reserved*/
	IEEE80211_ELEMID_RSNX		= 244,  /* RSNXE */
	IEEE80211_ELEMID_EXT		= 255,	/* Elements using the Element ID Extension field */
};

/*
 * Element ID Extension, from Table 9-77 of IEEE 802.11-2016
 */
enum {
	IEEE80211_ELEMID_EXT_FTM_SYNC = 8,	/* FTM Syncrhonization Info */
	IEEE80211_ELEMID_EXT_EST_SVC_PARAM	= 11,	/* Estimated Service Parameters */
	IEEE80211_ELEMID_EXT_CHAN_GUIDE	= 14,	/* Future Channel Guidance */
	IEEE80211_ELEMID_EXT_DH_PARAM = 32,
	IEEE80211_ELEMID_EXT_HECAP = 35,
	IEEE80211_ELEMID_EXT_HEOP = 36,
	IEEE80211_ELEMID_EXT_UORA = 37,
	IEEE80211_ELEMID_EXT_MU_EDCA = 38,
	IEEE80211_ELEMID_EXT_SR_PARAM = 39,
	IEEE80211_ELEMID_EXT_NDP_FB_REPORT = 41,
	IEEE80211_ELEMID_EXT_BSSCOLOR_CHANGE = 42,
	IEEE80211_ELEMID_EXT_QUIET_TIME_PERIOD = 43,
	IEEE80211_ELEMID_EXT_ESS_REPORT = 45,
	IEEE80211_ELEMID_EXT_HE_6G_BAND_CAP = 59,
	IEEE80211_ELEMID_EXT_UL_MU_PWR = 60,
};

#define IEEE80211_IE_EXT_IS_HECAP(_ie) \
	(((_ie)[1] >= IEEE80211_IE_HECAP_BODY_MIN) \
		&& ((_ie)[1] <= IEEE80211_IE_HECAP_BODY_MAX) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_HECAP))
#define IEEE80211_IE_EXT_IS_HEOP(_ie) \
	(((_ie)[1] >= IEEE80211_IE_HEOP_BODY_MIN) \
		&& ((_ie)[1] <= IEEE80211_IE_HEOP_BODY_MAX) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_HEOP))
#define IEEE80211_IE_EXT_IS_MU_EDCA(_ie) \
	(((_ie)[1] == (sizeof(struct ieee80211_ie_mu_edca) - 2)) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_MU_EDCA))
#define IEEE80211_IE_EXT_IS_SR_PARAM(_ie) \
	(((_ie)[1] >= IEEE80211_IE_SR_PARAM_BODY_MIN) \
		&& ((_ie)[1] <= IEEE80211_IE_SR_PARAM_BODY_MAX) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_SR_PARAM))
#define IEEE80211_IE_EXT_IS_BSSCOLOR_CHANGE(_ie) \
	(((_ie)[1] == (sizeof(struct ieee80211_ie_bsscolor_change) - 2)) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_BSSCOLOR_CHANGE))
#define IEEE80211_IE_EXT_IS_HE_6G_BAND_CAP(_ie) \
	((_ie)[1] == (sizeof(struct ieee80211_ie_he_6g_band_cap) - IEEE80211_IE_HDRLEN) \
		&& ((_ie)[2] == IEEE80211_ELEMID_EXT_HE_6G_BAND_CAP))
#define IEEE80211_IE_EXT_IS_UL_MU_PWR(_ie) \
	((_ie)[2] == IEEE80211_ELEMID_EXT_UL_MU_PWR)
#define IEEE80211_IE_EXT_IS_DH_PARAM(_ie) \
	((_ie)[2] == IEEE80211_ELEMID_EXT_DH_PARAM)

#define IEEE80211_IE_VENDOR_IS_WME(_ie) \
	(((_ie)[1] == (sizeof(struct ieee80211_wme_param) - 2)) \
		&& ((_ie)[2] == WME_OUI_BYTE0) \
		&& ((_ie)[3] == WME_OUI_BYTE1) \
		&& ((_ie)[4] == WME_OUI_BYTE2) \
		&& ((_ie)[5] == WME_OUI_TYPE))

#define IEEE80211_IE_IS_BSSCOLOR_CHANGE(_ie)	\
	(((_ie)[0] == IEEE80211_ELEMID_EXT) && \
		IEEE80211_IE_EXT_IS_BSSCOLOR_CHANGE(_ie))
#define IEEE80211_IE_IS_MU_EDCA(_ie)	\
	(((_ie)[0] == IEEE80211_ELEMID_EXT) && \
		IEEE80211_IE_EXT_IS_MU_EDCA(_ie))

#define IEEE80211_2040BSSCOEX_INFO_REQ	0x01
#define IEEE80211_2040BSSCOEX_40_intol	0x02
#define IEEE80211_2040BSSCOEX_20_REQ	0x04
#define IEEE80211_2040BSSCOEX_SCAN_EXEP_REQ	0x08
#define IEEE80211_2040BSSCOEX_SCAN_EXEP_GRA	0x10

#define IEEE80211_CHANSWITCHANN_BYTES 5
#define QTN_CHANSWITCHANN_TSF_BYTES 10
#define IEEE80211_CSA_LEN	7
#define IEEE80211_CSA_TSF_LEN	(IEEE80211_CSA_LEN + 10)
#define IEEE80211_SEC_CHAN_OFF_IE_LEN 3
#define IEEE80211_WBAND_CHANSWITCH_IE_LEN 5
#define IEEE80211_NCW_ACT_LEN   3      /* Notify Channel Width Action size */
#define IEEE80211_MU_GRP_ID_ACT_LEN 26 /* MU grp id mgmt action size */

#define IEEE80211_NODE_IDX_UNMAP(x)	(BR_SUBPORT_UNMAP(x))
#define IEEE80211_NODE_IDX_MAP(x)	(BR_SUBPORT_MAP(x))
#define IEEE80211_NODE_IDX_VALID(x)	((x) & 0x8000)
#define IEEE80211_NODE_IDX_INVALID(x)	(!IEEE80211_NODE_IDX_VALID(x))

/*
 * The 802.11 spec says at most 2007 stations may be
 * associated at once.  For most AP's this is way more
 * than is feasible so we use a default of 128.  This
 * number may be overridden by the driver and/or by
 * user configuration.
 */
#define	IEEE80211_AID_MAX		2007
#define	IEEE80211_AID_DEF		IEEE80211_AID_MAX

#define	IEEE80211_AID(b)	((b) &~ 0xc000)

struct ieee80211_tim_ie_full {
	uint8_t	tim_ie;			/* IEEE80211_ELEMID_TIM */
	uint8_t	tim_len;
	uint8_t	tim_count;		/* DTIM count */
	uint8_t	tim_period;		/* DTIM period */
	uint8_t	tim_bitctl;		/* bitmap control */
	uint8_t	tim_bitmap[IEEE80211_AID_DEF / NBBY];		/* variable-length bitmap */
} __packed;

struct ieee80211_ie_sec_chan_off {
	uint8_t	sco_id;			/* IEEE80211_ELEMID_SEC_CHAN_OFF */
	uint8_t	sco_len;
	uint8_t	sco_off;		/* offset */
} __packed;

struct ieee80211_country_ie {
	uint8_t	ie;			/* IEEE80211_ELEMID_COUNTRY */
	uint8_t	len;
	uint8_t	cc[3];			/* ISO CC+(I)ndoor/(O)utdoor */
	struct {
		uint8_t schan;			/* starting channel */
		uint8_t nchan;			/* number channels */
		uint8_t maxtxpwr;		/* tx power cap */
	} __packed band[4];			/* up to 4 sub bands */
} __packed;

#define IEEE80211_CHALLENGE_LEN		128

#define IEEE80211_SUPPCHAN_LEN		52

#define	IEEE80211_RATE_BASIC		0x80
#define	IEEE80211_RATE_VAL			0x7f
#define IEEE80211_BSS_MEMBERSHIP_SELECTOR	0x7F

#define IEEE80211_RATE_MAXSIZE_11B	4

/* The value in ieee80211_rate_legacy is twice as original value,
 * to keep the number after the decimal point.
 */
enum ieee80211_rate_legacy {
	IEEE80211_RATE_1MB = 0x02,
	IEEE80211_RATE_2MB = 0x04,
	IEEE80211_RATE_5_5MB = 0x0B,
	IEEE80211_RATE_11MB = 0x16,
	IEEE80211_RATE_6MB = 0x0C,
	IEEE80211_RATE_9MB = 0x12,
	IEEE80211_RATE_12MB = 0x18,
	IEEE80211_RATE_18MB = 0x24,
	IEEE80211_RATE_24MB = 0x30,
	IEEE80211_RATE_36MB = 0x48,
	IEEE80211_RATE_48MB = 0x60,
	IEEE80211_RATE_54MB = 0x6C,
};

#define IEEE80211_RATE_MBPS_TO_KBPS		1000	/* B stands for bit here */

/* EPR information element flags */
#define	IEEE80211_ERP_NON_ERP_PRESENT	0x01
#define	IEEE80211_ERP_USE_PROTECTION	0x02
#define	IEEE80211_ERP_LONG_PREAMBLE	0x04

/* Atheros private advanced capabilities info */
#define	ATHEROS_CAP_TURBO_PRIME		0x01
#define	ATHEROS_CAP_COMPRESSION		0x02
#define	ATHEROS_CAP_FAST_FRAME		0x04
/* bits 3-6 reserved */
#define	ATHEROS_CAP_BOOST		0x80

#define IEEE80211_OUI_LEN	3

#define	ATH_OUI			0x7f0300	/* Atheros OUI */
#define	QCOM_OUI		0xf0fd8c	/* Qualcom OUI */
#define	ATH_OUI_TYPE		0x01
#define	ATH_OUI_SUBTYPE		0x01
#define ATH_OUI_VERSION		0x00
#define	ATH_OUI_TYPE_XR		0x03
#define	ATH_OUI_VER_XR		0x01

#define	QTN_OUI			0x862600	/* Quantenna OUI */
#define	QTN_OUI_CFG		0x01
#define QTN_OUI_PAIRING		0x02		/* Pairing Protection */
#define	QTN_OUI_VSP_CTRL	0x03		/* VSP configuration */
#define QTN_OUI_TDLS_BRMACS	0x04		/* Obsoleted */
#define QTN_OUI_TDLS		0x05		/* Obsoleted */
#define	QTN_OUI_RM_SPCIAL	0x10		/* Radio measurement special group */
#define	QTN_OUI_RM_ALL		0x11		/* Radio measurement all group */
#define QTN_OUI_SCS             0x12            /* SCS status report and control */
#define QTN_OUI_QWME            0x13            /* WME IE between QSTA */
#define QTN_OUI_EXTENDER_ROLE	0x14		/* WDS Extender Role */
#define QTN_OUI_EXTENDER_BSSID	0x15		/* Extender BSSID */
#define QTN_OUI_OCAC_STATE	0x17		/* APs OCAC state - NONE, BACKOFF or ONGOING */
#define QTN_OUI_REPEATER_CASCADE	0x18		/* U-Repeater cascade level */
#define QTN_OUI_DBG_STATE	0x19		/* Debug state */

#define QTN_OUI_EXTENDER_ROLE_NONE	0x00	/* NONE Role */
#define QTN_OUI_EXTENDER_ROLE_MBS	0x01	/* MBS Role */
#define QTN_OUI_EXTENDER_ROLE_RBS	0x02	/* RBS Role */

#define QTN_QWME_IE_VERSION	1

#define	WPA_OUI			0xf25000
#ifndef WPA_OUI_TYPE
#define	WPA_OUI_TYPE		0x01
#endif
#define	WSC_OUI_TYPE		0x04
#define	WPA_VERSION		1		/* current supported version */

#define	WPA_CSE_NULL		0x00
#define	WPA_CSE_WEP40		0x01
#define	WPA_CSE_TKIP		0x02
#define	WPA_CSE_CCMP		0x04
#define	WPA_CSE_WEP104		0x05
#define	RSN_CSE_GROUP_NOT_ALLOW 0x07  /* Group addressed traffic not allowed */

#define	WPA_ASE_NONE		0x00
#define	WPA_ASE_8021X_UNSPEC	0x01
#define	WPA_ASE_8021X_PSK	0x02
#define	IEEE80211_RSN_ASE_TPK	0x07  /* TDLS TPK Handshake */

#define	RSN_OUI			0xac0f00
#define	RSN_VERSION		1		/* current supported version */

#define WFA_OUI			0x9A6F50
#define WFA_TYPE_OSEN		0x12
#define WFA_MBO_OCE		0x16
#define WFA_AKM_TYPE_OSEN	0x1
#define WFA_AKM_TYPE_DPP	0x2
#define WFA_TYPE_OWE_TRANS	0x1C
#define WFA_TYPE_MAP		0x1B

#define	BCM_OUI			0x4C9000	/* Apple Products */
#define	BCM_OUI_TYPE		0x01
#define BCM_OUI_VHT_TYPE	0x0804
#define BCM_OUI_VHT_2_TYPE	0x1804		/* R8500, AC5300 */

#define BCM_OUI_VHT_TYPE2	0x0704
#define BCM_OUI_VHT_2_TYPE2	0x1704

#define BCM_OUI_UNKNOWN_TYPE	0x0003

#define	BCM_OUI_2		0x181000	/* iPad */
#define	BCM_OUI_2_TYPE		0x02

enum {
	IEEE80211_PROTOCOL_WPA1 = 1,
	IEEE80211_PROTOCOL_WPA2 = 2,
	IEEE80211_PROTOCOL_WPA3 = 3,
};

#define	RSN_CSE_NULL		0x00
#define	RSN_CSE_WEP40		0x01
#define	RSN_CSE_TKIP		0x02
#define	RSN_CSE_WRAP		0x03
#define	RSN_CSE_CCMP		0x04
#define	RSN_CSE_WEP104		0x05
#define	RSN_CSE_BIP		0x06
#define	RSN_CSE_GCMP		0x08
#define	RSN_CSE_GCMP_256	0x09
#define	RSN_CSE_CCMP_256	0x0A

#define	RSN_ASE_NONE		0x00
#define	RSN_ASE_8021X_UNSPEC	0x01
#define	RSN_ASE_8021X_PSK	0x02
#define RSN_ASE_FT_8021X	0x03
#define RSN_ASE_FT_PSK		0x04
#define	RSN_ASE_8021X_SHA256	0x05
#define	RSN_ASE_8021X_PSK_SHA256 0x06
#define	RSN_SAE			0x08
#define	RSN_OWE			0x12
#define	RSN_ASE_DPP		0x20

#define	RSN_CAP_PREAUTH		0x01
#define	RSN_CAP_MFP_REQ		0x0040
#define	RSN_CAP_MFP_CAP		0x0080
#define	RSN_CAP_SPP_CAP		0x0400
#define	RSN_CAP_SPP_REQ		0x0800

#define RSNX_CAP_SAE_H2E	0x20

#define RSN_IS_MFP(_rsn_caps) (((_rsn_caps) & RSN_CAP_MFP_REQ) || ((_rsn_caps) & RSN_CAP_MFP_CAP))

#define	WME_QOSINFO_COUNT	0x0f  /* Mask for Param Set Count field */
#define	WME_OUI_BYTE0		0x00
#define	WME_OUI_BYTE1		0x50
#define	WME_OUI_BYTE2		0xf2
#define	WME_OUI_TYPE		0x02
#define	WME_OUI_BYTES		WME_OUI_BYTE0, WME_OUI_BYTE1, WME_OUI_BYTE2
#define	WME_OUI			((WME_OUI_BYTE2 << 16) | (WME_OUI_BYTE1 << 8) | WME_OUI_BYTE0)
#define	WME_INFO_OUI_SUBTYPE	0x00
#define	WME_PARAM_OUI_SUBTYPE	0x01
#define	WME_TSPEC_OUI_SUBTYPE	0x02
#define	WME_VERSION		1
#define	WME_UAPSD_MASK		0x0f

#define RLNK_OUI		0x430C00	/* Ralink OUI */

#define RTK_OUI			0x4ce000	/* Realtek OUI */
#define EDIMAX_OUI		0x021f80	/* Edimax OUI */
#define MTEK_OUI		0xe70c00	/* Mediatek OUI */
#define APPLE_OUI		0xF21700

#define PEER_VENDOR_NONE	0x00
#define PEER_VENDOR_QTN		0x01
#define PEER_VENDOR_BRCM	0x02
#define PEER_VENDOR_ATH		0x04
#define PEER_VENDOR_RLNK	0x08
#define PEER_VENDOR_RTK		0x10
#define PEER_VENDOR_INTEL	0x20
#define PEER_VENDOR_MEDIATEK	0x40

/*
 * 802.11ac VHT Capabilities element
 */
struct ieee80211_ie_vhtcap {
	u_int8_t	vht_id;			/* element ID */
	u_int8_t	vht_len;		/* length in bytes */
	u_int8_t	vht_cap[4];		/* VHT capabilities info */
	u_int8_t	vht_mcs_nss_set[8];	/* supported MSC and NSS set */
} __packed;

/* VHT capabilities flags */
#define IEEE80211_VHTCAP_C_CHWIDTH			0x0000000C
#define IEEE80211_VHTCAP_C_RX_LDPC			0x00000010
#define IEEE80211_VHTCAP_C_SHORT_GI_80			0x00000020
#define IEEE80211_VHTCAP_C_SHORT_GI_160			0x00000040
#define IEEE80211_VHTCAP_C_TX_STBC			0x00000080
#define IEEE80211_VHTCAP_C_RX_STBC_1SS			0x00000100
#define IEEE80211_VHTCAP_C_RX_STBC_2SS			0x00000200
#define IEEE80211_VHTCAP_C_RX_STBC_4SS			0x00000400
#define IEEE80211_VHTCAP_C_SU_BEAM_FORMER_CAP		0x00000800
#define IEEE80211_VHTCAP_C_SU_BEAM_FORMEE_CAP		0x00001000
#define IEEE80211_VHTCAP_C_MU_BEAM_FORMER_CAP		0x00080000
#define IEEE80211_VHTCAP_C_MU_BEAM_FORMEE_CAP		0x00100000
#define IEEE80211_VHTCAP_C_VHT_TXOP_PS			0x00200000
#define IEEE80211_VHTCAP_C_PLUS_HTC_MINUS_VHT_CAP	0x00400000
#define IEEE80211_VHTCAP_C_RX_ATN_PATTERN_CONSISTNCY	0x10000000
#define IEEE80211_VHTCAP_C_TX_ATN_PATTERN_CONSISTNCY	0x20000000

#define IEEE80211_VHTCAP_C_MU_BEAM_FORMXX_CAP_MASK	(IEEE80211_VHTCAP_C_MU_BEAM_FORMER_CAP | \
							 IEEE80211_VHTCAP_C_MU_BEAM_FORMEE_CAP)

/* Channel mode for MCS to rate conversion table */
enum {
	IEEE80211_MCS2RATE_CHAN_MODE_80MHZ = 0,
	IEEE80211_MCS2RATE_CHAN_MODE_160MHZ,
	IEEE80211_MCS2RATE_CHAN_MODE_40MHZ,
	IEEE80211_MCS2RATE_CHAN_MODE_20MHZ,
	IEEE80211_MCS2RATE_CHAN_MODE_UNKNOWN
};

/* VHT capability macro */
/* get macros */
/* VHT capabilities (all bits) */
#define IEEE80211_VHTCAP_GET_CAPFLAGS(vhtcap) \
	(u_int32_t)((vhtcap)->vht_cap[0] | \
	((vhtcap)->vht_cap[1] << 8) | \
	((vhtcap)->vht_cap[2] << 16) | \
	((vhtcap)->vht_cap[3] << 24))

/* B0-1 Max. MPDU Length */
#define IEEE80211_VHTCAP_GET_MAXMPDU(vhtcap) \
	(enum ieee80211_vht_maxmpdu)((vhtcap)->vht_cap[0] & 0x03)

/* B2-3 Supported channel width */
#define IEEE80211_VHTCAP_GET_CHANWIDTH(vhtcap) \
	(enum ieee80211_vhtcap_chanwidth)(((vhtcap)->vht_cap[0] & 0x0C) >> 2)

/* B4 RX LDPC support */
#define IEEE80211_VHTCAP_GET_RXLDPC(vhtcap) \
	(((vhtcap)->vht_cap[0] & 0x10) >> 4)

/* B5 Short GI for 80MHz support */
#define IEEE80211_VHTCAP_GET_SGI_80MHZ(vhtcap) \
	(((vhtcap)->vht_cap[0] & 0x20) >> 5)

/* B6 Short GI for 160MHz support */
#define IEEE80211_VHTCAP_GET_SGI_160MHZ(vhtcap) \
	(((vhtcap)->vht_cap[0] & 0x40) >> 6)

/* B7 TX STBC */
#define IEEE80211_VHTCAP_GET_TXSTBC(vhtcap) \
	(((vhtcap)->vht_cap[0] & 0x80) >> 7)

/* B8-10 RX STBC */
#define IEEE80211_VHTCAP_GET_RXSTBC(vhtcap) \
	(enum ieee80211_vht_rxstbc)((vhtcap)->vht_cap[1] & 0x07)

/* B11 SU Beam-former */
#define IEEE80211_VHTCAP_GET_SU_BEAMFORMER(vhtcap) \
	(((vhtcap)->vht_cap[1] & 0x08) >> 3)

/* B12 SU Beam-formee */
#define IEEE80211_VHTCAP_GET_SU_BEAMFORMEE(vhtcap) \
	(((vhtcap)->vht_cap[1] & 0x10) >> 4)

/* B13-15 Beamformee STS capability */
#define IEEE80211_VHTCAP_GET_BFSTSCAP(vhtcap) \
	(u_int8_t)(((vhtcap)->vht_cap[1] & 0xE0) >> 5)

/* B16-18 Number of sounding Dimensions */
#define IEEE80211_VHTCAP_GET_NUMSOUND(vhtcap) \
	(u_int8_t)((vhtcap)->vht_cap[2] & 0x07)

/* B19 MU Beam-formee VHT capability */
#define IEEE80211_VHTCAP_GET_MU_BEAMFORMER(vhtcap) \
	(((vhtcap)->vht_cap[2] & 0x08) >> 3)

/* B20 MU Beam-former VHT capability */
#define IEEE80211_VHTCAP_GET_MU_BEAMFORMEE(vhtcap) \
	(((vhtcap)->vht_cap[2] & 0x10) >> 4)

/* B22 VHT variant HT control field */
#define IEEE80211_VHTCAP_GET_HTC_VHT(vhtcap) \
	(((vhtcap)->vht_cap[2] & 0x40) >> 6)

/* B23-25 Max. A-MPDU Length Exponent */
#define IEEE80211_VHTCAP_GET_MAXAMPDUEXP(vhtcap) \
	(enum ieee80211_vht_maxampduexp)((((vhtcap)->vht_cap[2] & 0x80) >> 7) | \
	(((vhtcap)->vht_cap[3] & 0x03) << 1))

/* B26-27 VHT Link Adaptation capable */
#define IEEE80211_VHTCAP_GET_LNKADPTCAP(vhtcap) \
	(enum ieee80211_vht_lnkadptcap)(((vhtcap)->vht_cap[3] & 0x0C) >> 2)

/* B28 Rx Antenna pattern consistency */
#define IEEE80211_VHTCAP_GET_RXANTPAT(vhtcap) \
	(((vhtcap)->vht_cap[3] & 0x10) >> 4)

/* B29 Tx Antenna pattern consistency */
#define IEEE80211_VHTCAP_GET_TXANTPAT(vhtcap) \
	(((vhtcap)->vht_cap[3] & 0x20) >> 5)

/* B0-B15 RX VHT-MCS MAP for Spatial streams 1-8 */
#define IEEE80211_VHTCAP_GET_RX_MCS_NSS(vhtcap) \
	(((vhtcap)->vht_mcs_nss_set[1] << 8) | \
	((vhtcap)->vht_mcs_nss_set[0]))

/* B32-B47 TX VHT-MCS MAP for Spatial streams 1-8 */
#define IEEE80211_VHTCAP_GET_TX_MCS_NSS(vhtcap) \
	(((vhtcap)->vht_mcs_nss_set[5] << 8) | \
	((vhtcap)->vht_mcs_nss_set[4]))

/* B16-B28 RX Highest supported Long GI data rates */
#define IEEE80211_VHTCAP_GET_RX_LGIMAXRATE(vhtcap) \
	(u_int16_t)(((vhtcap)->vht_mcs_nss_set[2]) | \
	((vhtcap)->vht_mcs_nss_set[3] << 8))

/* B48-B60 TX Highest supported Long GI data rates */
#define IEEE80211_VHTCAP_GET_TX_LGIMAXRATE(vhtcap) \
	(u_int16_t)(((vhtcap)->vht_mcs_nss_set[6]) | \
	((vhtcap)->vht_mcs_nss_set[7] << 8))

/* set macros */
/* VHT capabilities (all bits) */
#define IEEE80211_VHTCAP_SET_CAPFLAGS(vhtcap, _cap) \
	(vhtcap)->vht_cap[0] = ((_cap) & 0x000000FF); \
	(vhtcap)->vht_cap[1] = (((_cap) & 0x0000FF00) >> 8); \
	(vhtcap)->vht_cap[2] = (((_cap) & 0x00FF0000) >> 16); \
	(vhtcap)->vht_cap[3] = (((_cap) & 0xFF000000) >> 24)

/* B0-1 Max. MPDU Length */
#define IEEE80211_VHTCAP_SET_MAXMPDU(vhtcap, _m) \
	(vhtcap)->vht_cap[0] = (((vhtcap)->vht_cap[0] & ~0x03) | ((_m) & 0x03))

/* B2-3 Supported channel width */
#define IEEE80211_VHTCAP_SET_CHANWIDTH(vhtcap, _m) \
	(vhtcap)->vht_cap[0] = (((vhtcap)->vht_cap[0] & ~0x0C) | ((_m) & 0x03) << 2)

/* B8-10 RX STBC */
#define IEEE80211_VHTCAP_SET_RXSTBC(vhtcap, _m) \
	(vhtcap)->vht_cap[1] = (((vhtcap)->vht_cap[1] & ~0x07) | ((_m) & 0x07))

/* B13-15 Beamformee STS capability */
#define IEEE80211_VHTCAP_SET_BFSTSCAP(vhtcap, _m) \
	(vhtcap)->vht_cap[1] = (((vhtcap)->vht_cap[1] & ~0xE0) | ((_m) & 0x07) << 5)

/* B16-18 Number of sounding Dimensions */
#define IEEE80211_VHTCAP_SET_NUMSOUND(vhtcap, _m) \
	(vhtcap)->vht_cap[2] = (((vhtcap)->vht_cap[2] & ~0x07) | ((_m) & 0x07))

/* B23-25 Max. A-MPDU Length Exponent */
#define IEEE80211_VHTCAP_SET_MAXAMPDUEXP(vhtcap, _m) \
	(vhtcap)->vht_cap[2] = (((vhtcap)->vht_cap[2] & ~0x80) | ((_m) & 0x01) << 7); \
	(vhtcap)->vht_cap[3] = (((vhtcap)->vht_cap[3] & ~0x03) | ((_m) & 0x06) >> 1)

/* B26-27 VHT Link Adaptation capable */
#define IEEE80211_VHTCAP_SET_LNKADPTCAP(vhtcap, _m) \
	(vhtcap)->vht_cap[3] = (((vhtcap)->vht_cap[3] & ~0x0C) | ((_m) & 0x03) << 2)

/* B0-B15 RX VHT-MCS MAP for Spatial streams 1-8 */
#define IEEE80211_VHTCAP_SET_RX_MCS_NSS(vhtcap, _m) \
	(vhtcap)->vht_mcs_nss_set[1] = (((_m) & 0xFF00) >> 8); \
	(vhtcap)->vht_mcs_nss_set[0] = ((_m) & 0x00FF)

/* B16-B28 RX Highest supported Long GI data rates */
#define IEEE80211_VHTCAP_SET_RX_LGIMAXRATE(vhtcap, _m) \
	(vhtcap)->vht_mcs_nss_set[2] = ((_m) & 0x00FF); \
	(vhtcap)->vht_mcs_nss_set[3] = (((_m) & 0x1F00) >> 8)

/* B32-B47 TX VHT-MCS MAP for Spatial streams 1-8 */
#define IEEE80211_VHTCAP_SET_TX_MCS_NSS(vhtcap, _m) \
	(vhtcap)->vht_mcs_nss_set[5] = (((_m) & 0xFF00) >> 8); \
	(vhtcap)->vht_mcs_nss_set[4] = ((_m) & 0x00FF)

/* B48-B60 TX Highest supported Long GI data rates */
#define IEEE80211_VHTCAP_SET_TX_LGIMAXRATE(vhtcap, _m) \
	(vhtcap)->vht_mcs_nss_set[6] = ((_m) & 0x00FF); \
	(vhtcap)->vht_mcs_nss_set[7] = (((_m) & 0x1F00) >> 8)

/* VHT capabilities options */
/* Defined in _ieee80211.h file */
/*
 * 802.11ac VHT Operation element
 */
struct ieee80211_ie_vhtop {
	u_int8_t	vhtop_id;		/* element ID */
	u_int8_t	vhtop_len;		/* length in bytes */
	u_int8_t	vhtop_info[3];		/* VHT Operation info */
	u_int8_t	vhtop_bvhtmcs[2];	/* basic VHT MSC and NSS set */
} __packed;

/* VHT Operation Information */
/* Channel width Octet 1 */
#define IEEE80211_VHTOP_SET_CHANWIDTH(vhtop, _m) \
	(vhtop)->vhtop_info[0] = (_m)

/* Channel Center Frequency Segment 0 */
#define IEEE80211_VHTOP_SET_CENTERFREQ0(vhtop, _m) \
	(vhtop)->vhtop_info[1] = (_m)

/* Channel Center Frequency Segment 1 */
#define IEEE80211_VHTOP_SET_CENTERFREQ1(vhtop, _m) \
	(vhtop)->vhtop_info[2] = (_m)

/* Basic VHT-MCS and NSS Set  */
#define IEEE80211_VHTOP_SET_BASIC_MCS_NSS(vhtop, _m) \
	(vhtop)->vhtop_bvhtmcs[1] = ((_m) & 0xFF00) >> 8; \
	(vhtop)->vhtop_bvhtmcs[0] = ((_m) & 0x00FF)

/* Get macros */
/* Channel width Octet 1 */
#define IEEE80211_VHTOP_GET_CHANWIDTH(vhtop) \
	(vhtop)->vhtop_info[0]

/* Channel Center Frequency Segment 0 */
#define IEEE80211_VHTOP_GET_CENTERFREQ0(vhtop) \
	(vhtop)->vhtop_info[1]

/* Channel Center Frequency Segment 1 */
#define IEEE80211_VHTOP_GET_CENTERFREQ1(vhtop) \
	(vhtop)->vhtop_info[2]

/* Basic VHT-MCS and NSS Set  */
#define IEEE80211_VHTOP_GET_BASIC_MCS_NSS(vhtop) \
	(((vhtop)->vhtop_bvhtmcs[1] << 8) | \
	((vhtop)->vhtop_bvhtmcs[0]))

#define IEEE80211_CONVERT_VHTOPCW_TO_VHTCAPCW(_cw) \
	((_cw) - 1)

/*
 * 802.11ac VHT Operating mode notification element
 */
struct ieee80211_ie_vhtop_notif {
	uint8_t	id;
	uint8_t	len;
	uint8_t	vhtop_notif_mode;
} __packed;

/*
 * 802.11ac Extended BSS Load element
 */
struct ieee80211_ie_ebssload {
	u_int8_t	ebl_id;			/* element ID */
	u_int8_t	ebl_len;		/* length in bytes */
	u_int8_t	ebl_mumimo_cnt[2];	/* MU-MIMO Capable station count */
	u_int8_t	ebl_ss_underuse;	/* Spatial Stream Underutilization */
	u_int8_t	ebl_20mhz_use;		/* Observable Secondary 20Mhz use */
	u_int8_t	ebl_40mhz_use;		/* Observable Secondary 40Mhz use */
	u_int8_t	ebl_80mhz_use;		/* Observable Secondary 80Mhz use */
} __packed;


/*
 * 802.11ac Wide Bandwidth Channel Switch element
 */
struct ieee80211_ie_wbchansw {
	u_int8_t	wbcs_id;		/* element ID */
	u_int8_t	wbcs_len;		/* length in bytes */
	u_int8_t	wbcs_newchanw;		/* New Channel Width */
	u_int8_t	wbcs_newchancf0;	/* New Channel Center Freq 0 */
	u_int8_t	wbcs_newchancf1;	/* New Channel Center Freq 1 */
} __packed;


/*
 * Transmit Power Envelope (TPE) element
 */
#define IEEE80211_TPE_INFO_PWR_COUNT	0x07
#define IEEE80211_TPE_INFO_PWR_COUNT_S	0
#define IEEE80211_TPE_INFO_PWR_UNIT	0x38
#define IEEE80211_TPE_INFO_PWR_UNIT_S	3
#define IEEE80211_TPE_INFO_PWR_CAT	0xC0
#define IEEE80211_TPE_INFO_PWR_CAT_S	6

enum {
	IEEE80211_TPE_INFO_PWR_UNIT_LOCAL_EIRP = 0,
	IEEE80211_TPE_INFO_PWR_UNIT_LOCAL_EIRP_PSD,
	IEEE80211_TPE_INFO_PWR_UNIT_REG_CLIENT_EIRP,
	IEEE80211_TPE_INFO_PWR_UNIT_REG_CLIENT_EIRP_PSD
};

enum {
	IEEE80211_TPE_INFO_PWR_CAT_DEFAULT = 0,
	IEEE80211_TPE_INFO_PWR_CAT_SUBORDINATE_DEVICE
};

#define IEEE80211_TPE_MAX_TX_PWR_FIELDS		8

struct ieee80211_ie_tpe {
	u_int8_t	tpe_id;			/* element ID */
	u_int8_t	tpe_len;		/* length in byte */
	u_int8_t	tpe_txpwr_info;		/* tx power info */
	u_int8_t	tpe_txpwr_field[IEEE80211_TPE_MAX_TX_PWR_FIELDS];
} __packed;

/*
 * 802.11ac Channel Switch Wrapper element
 */
struct ieee80211_ie_chsw_wrapper {
	u_int8_t			chsw_id;		/* element ID */
	u_int8_t			chsw_len;		/* length in byte */
} __packed;

/*
 * 802.11ac AID element
 */
struct ieee80211_ie_aid {
	u_int8_t	aid_id;		/* element ID */
	u_int8_t	aid_len;	/* length in byte */
	u_int16_t	aid;		/* aid */
} __packed;

/*
 * 802.11ac Quiet Channel element
 */
struct ieee80211_ie_quietchan {
	u_int8_t	qc_id;		/* element ID */
	u_int8_t	qc_len;		/* length in byte */
	u_int8_t	qc_qmode;	/* AP Quite Mode */
	u_int8_t	qc_qcnt;	/* AP Quite Count */
	u_int8_t	qc_qperiod;	/* AP Quite Period */
	u_int8_t	qc_qduration;	/* AP Quite Duration */
	u_int8_t	qc_qoffset;	/* AP Quite Offset */
} __packed;


/*
 * 802.11ac Operating Mode Notification element
 */
struct ieee80211_ie_opmodenotice {
	u_int8_t	omn_id;		/* element ID */
	u_int8_t	omn_len;	/* length in byte */
	u_int8_t	opn_opmode;	/* Op Mode */
} __packed;

enum {
	IEEE80211_TIMEOUT_REASSOC_DEADLINE		= 1,
	IEEE80211_TIMEOUT_KEY_LIFETIME			= 2,
	IEEE80211_TIMEOUT_ASSOC_COMEBACK		= 3,
};

#define IEEE80211_W_ASSOC_COMEBACK_TO		1000
#define IEEE80211_W_SA_QUERY_RETRY_MAX_TO	1000 /* in TUs */
#define IEEE80211_W_SA_QUERY_RETRY_TO		201  /* in TUs */

/*
 * 802.11w timeout information IE
 */
struct ieee80211_timout_int_ie {
	u_int8_t	timout_int_ie;			/* IEEE80211_ELEMID_TIMEOUT_INT */
	u_int8_t	timout_int_len;
	u_int8_t	timout_int_type;		/* Timeout Interval Type */
	u_int32_t	timout_int_value;		/* in tus */
} __packed;

struct ieee80211_ie_brcm_vht {
	uint8_t id;
	uint8_t len;
	uint8_t brcm_vht_oui[3];
	uint16_t brcm_vht_type;
	uint8_t vht_ies[0];
}__packed;

/*
 * 802.11ax HE Capabilities element
 */
struct ieee80211_ie_hecap {
	uint8_t he_id;			/* Element ID */
	uint8_t he_len;			/* Length in bytes */
	uint8_t he_id_ext;		/* Element ID extension */
#if IEEE80211_AX_IS_DRAFT_20
	uint8_t he_mac_cap[5];		/* HE MAC capabilities info */
	uint8_t he_phy_cap[9];		/* HE PHY capabilities info */
#else
	uint8_t he_mac_cap[6];		/* HE MAC capabilities info */
	uint8_t he_phy_cap[11];		/* HE PHY capabilities info */
#endif
	uint8_t he_mcs_map_le80[4];	/* TX RX HE-MCS Map <=80MHz */
//	uint8_t he_mcs_map_160[4];	/* TX RX HE-MCS Map 160MHz(optional) */
//	uint8_t he_mcs_map_80p80[4];	/* TX RX HE-MCS Map 80+80MHz(optional) */
//	uint8_t he_ppe_thres[0];	/* PPE Thresholds(optional)(variable) */
} __packed;

#define IEEE80211_HECAP_PHY_GET_CW_SET(_hecap)	\
	IEEE80211_FLD_GET((_hecap)->he_phy_cap, IEEE80211_HECAP_PHY_CW_SET)
#define IEEE80211_HECAP_PHY_GET_PPE_THRES_PRESENT(_hecap)	\
	IEEE80211_FLD_GET((_hecap)->he_phy_cap, IEEE80211_HECAP_PHY_PPE_THRES_PRESENT)
#define IEEE80211_HECAP_PPET_GET_NSTS(_hecap) \
	IEEE80211_FLD_GET(IEEE80211_IE_HECAP_PPET(_hecap), IEEE80211_HECAP_PPET_NSTS)
#define IEEE80211_HECAP_PPET_GET_RU_IDX_BITMASK(_hecap) \
	IEEE80211_FLD_GET(IEEE80211_IE_HECAP_PPET(_hecap), IEEE80211_HECAP_PPET_RU_IDX_BITMASK)

#define IEEE80211_IE_HECAP_HE_MCS_160MHZ_SIZE	4
#define IEEE80211_IE_HECAP_HE_MCS_160MHZ_OFFSET	sizeof(struct ieee80211_ie_hecap)
#define IEEE80211_IE_HECAP_HE_MCS_160MHZ(_hecap)  \
	((uint8_t *)(_hecap) + IEEE80211_IE_HECAP_HE_MCS_160MHZ_OFFSET)

#define IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_SIZE	4
#define IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_OFFSET(_160mhz)  \
	(IEEE80211_IE_HECAP_HE_MCS_160MHZ_OFFSET +  \
		((_160mhz) ? IEEE80211_IE_HECAP_HE_MCS_160MHZ_SIZE : 0))
#define IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_OPT(_hecap, _160mhz)  \
	((uint8_t *)(_hecap) + IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_OFFSET(_160mhz))
#define IEEE80211_IE_HECAP_HE_MCS_80P80MHZ(_hecap)  \
	IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_OPT(_hecap,  \
		IEEE80211_HECAP_PHY_GET_CW_SET(_hecap) & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ)

#define IEEE80211_IE_HECAP_PPET_SIZE(_hecap) \
	IEEE80211_IE_HECAP_PPET_SIZE_OPT(IEEE80211_HECAP_PPET_GET_NSTS(_hecap), \
		IEEE80211_HECAP_PPET_GET_RU_IDX_BITMASK(_hecap))
#define IEEE80211_IE_HECAP_PPET_OFFSET(_160mhz, _80p80mhz)  \
	(IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_OFFSET(_160mhz) +  \
		((_80p80mhz) ? IEEE80211_IE_HECAP_HE_MCS_80P80MHZ_SIZE : 0))
#define IEEE80211_IE_HECAP_PPET_OPT(_hecap, _160mhz, _80p80mhz)  \
	((uint8_t *)(_hecap) + IEEE80211_IE_HECAP_PPET_OFFSET(_160mhz, _80p80mhz))
#define IEEE80211_IE_HECAP_PPET(_hecap)  \
	IEEE80211_IE_HECAP_PPET_OPT(_hecap,  \
		IEEE80211_HECAP_PHY_GET_CW_SET(_hecap) & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ,  \
		IEEE80211_HECAP_PHY_GET_CW_SET(_hecap) & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_80P80MHZ)

#define IEEE80211_IE_HECAP_SIZE_OPT(_160mhz, _80p80mhz, _ppe, _nss, _ru_idx_mask) \
	(IEEE80211_IE_HECAP_PPET_OFFSET(_160mhz, _80p80mhz) + \
		((_ppe) ? IEEE80211_IE_HECAP_PPET_SIZE_OPT(_nss, _ru_idx_mask) : 0))
#define IEEE80211_IE_HECAP_SIZE(_hecap) IEEE80211_IE_HECAP_SIZE_OPT( \
		IEEE80211_HECAP_PHY_GET_CW_SET(_hecap) & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_160MHZ, \
		IEEE80211_HECAP_PHY_GET_CW_SET(_hecap) & IEEE80211_HECAP_PHY_CHAN_WIDTH_5G_80P80MHZ, \
		IEEE80211_HECAP_PHY_GET_PPE_THRES_PRESENT(_hecap), \
		IEEE80211_HECAP_PPET_GET_NSTS(_hecap), \
		IEEE80211_HECAP_PPET_GET_RU_IDX_BITMASK(_hecap))

#define IEEE80211_IE_HECAP_SIZE_MIN sizeof(struct ieee80211_ie_hecap)
#define IEEE80211_IE_HECAP_SIZE_MAX IEEE80211_IE_HECAP_SIZE_OPT(1, 1, 1, IEEE80211_HE_NSS8 - 1, 0xf)
#define IEEE80211_IE_HECAP_BODY_MIN (IEEE80211_IE_HECAP_SIZE_MIN - IEEE80211_IE_HDRLEN)
#define IEEE80211_IE_HECAP_BODY_MAX (IEEE80211_IE_HECAP_SIZE_MAX - IEEE80211_IE_HDRLEN)

/* Draft P802.11ax_D3.0 - 9.4.2.237.2 HE MAC Capabilities Information field */
	/* field name */				/* field offset, bits */
#define IEEE80211_HECAP_MAC_HTC_HE			0, 1	/* B0 +HTC-HE Support */
#define IEEE80211_HECAP_MAC_TWT_REQUESTER		1, 1	/* B1 TWT Requester Support */
#define IEEE80211_HECAP_MAC_TWT_RESPONDER		2, 1	/* B2 TWT Responder Support */
#define IEEE80211_HECAP_MAC_FRAG_SUPPORT		3, 2	/* B3-4 Fragmentation Support */
#define IEEE80211_HECAP_MAC_MAX_FRAG_MSDU_EXP		5, 3	/* B5-7 Maximum Number Of Fragmented MSDUs/AMSDUs Exponent */
#define IEEE80211_HECAP_MAC_MIN_FRAG_SIZE		8, 2	/* B8-9 Minimum Fragment Size */
#define IEEE80211_HECAP_MAC_TRIGGER_PAD_DUR		10, 2	/* B10-11 Trigger Frame MAC Padding Duration */
#define IEEE80211_HECAP_MAC_MULTI_TID_AGG_RX		12, 3	/* B12-14 Multi-TID Aggregation Rx Support */
#define IEEE80211_HECAP_MAC_HE_LINK_ADAPT		15, 2	/* B15-16 HE Link Adaptation */
#define IEEE80211_HECAP_MAC_ALL_ACK			17, 1	/* B17 All Ack Support */
#define IEEE80211_HECAP_MAC_TRS_SUPPORT			18, 1	/* B18 TRS Support */
#define IEEE80211_HECAP_MAC_BSR_SUPPORT			19, 1	/* B19 BSR Support */
#define IEEE80211_HECAP_MAC_BCAST_TWT_SUPPORT		20, 1	/* B20 Broadcast TWT Support */
#define IEEE80211_HECAP_MAC_32BIT_BA_SUPPORT		21, 1	/* B21 32-bit BA Bitmap Support */
#define IEEE80211_HECAP_MAC_MU_CASCAD_SUPPORT		22, 1	/* B22 MU Cascading Support */
#define IEEE80211_HECAP_MAC_ACK_EN_AGG_SUPPORT		23, 1	/* B23 Ack-Enabled Aggregation Support */
#if IEEE80211_AX_IS_DRAFT_20
#define IEEE80211_HECAP_MAC_GRP_ADDR_MULTI_STA_BA	24, 1	/* B24 Group Addressed Multi-STA Block-Ack In DL MU Support */
#else
								/* B24 Reserved */
#endif
#define IEEE80211_HECAP_MAC_OM_CTL_SUPPORT		25, 1	/* B25 OM Control Support */
#define IEEE80211_HECAP_MAC_OFDMA_RA_SUPPORT		26, 1	/* B26 OFDMA RA Support */
#define IEEE80211_HECAP_MAC_MAX_AMPDU_EXP		27, 2	/* B27-28 Maximum A-MPDU Length Exponent */
#define IEEE80211_HECAP_MAC_AMSDU_FRAG_SUPPORT		29, 1	/* B29 A-MSDU Fragmentation Support */
#define IEEE80211_HECAP_MAC_FLEX_TWT_SUPPORT		30, 1	/* B30 Flexible TWT Schedule Support */
#define IEEE80211_HECAP_MAC_RX_CTRL_FRM			31, 1	/* B31 Rx Control Frame to MultiBSS */
#define IEEE80211_HECAP_MAC_BSRP_BQRP_AMPDU_AGG		32, 1	/* B32 BSRP BQRP AMPDU Aggregation */
#define IEEE80211_HECAP_MAC_QTP_SUPPORT			33, 1	/* B33 QTP Support */
#define IEEE80211_HECAP_MAC_BQR_SUPPORT			34, 1	/* B34 BQR Support */
#define IEEE80211_HECAP_MAC_SRP_RESPONDER		35, 1	/* B35 SRP Responder */
#define IEEE80211_HECAP_MAC_NDP_FB_REPORT		36, 1	/* B36 NDP Feedback Report Support */
#define IEEE80211_HECAP_MAC_OPS_SUPPORT			37, 1	/* B37 OPS Support */
#define IEEE80211_HECAP_MAC_AMSDU_IN_AMPDU		38, 1	/* B38 A-MSDU In A-MPDU Support */
#if !IEEE80211_AX_IS_DRAFT_20
#define IEEE80211_HECAP_MAC_MULTI_TID_AGG_TX		39, 3	/* B39-41 Multi-TID Aggregation Tx Support */
#define IEEE80211_HECAP_MAC_SUBCHAN_SEL_TX		42, 1	/* B42 HE Subchannel Selective Transmission Support */
#define IEEE80211_HECAP_MAC_UL_2x996_TONE_RU		43, 1	/* B43 UL 2x996-tone RU Support */
#define IEEE80211_HECAP_MAC_OM_CTL_UL_MU_DIS_RX		44, 1	/* B44 OM Control UL MU Data Disable RX Support */
#endif

/* Draft P802.11ax_D4.0 Additions - 9.4.2.242.2 HE MAC Capabilities Information field */
#define IEEE80211_HECAP_MAC_DYN_SMPS			45, 1	/* B45 HE Dynamic SM Power Save */
#define IEEE80211_HECAP_MAC_PUNC_SND			46, 1	/* B46 Punctured Sounding Support */
#define IEEE80211_HECAP_MAC_HT_VHT_TRIG_RX		47, 1	/* B47 HT/VHT Trigger Frame RX */

/* Draft P802.11ax_D3.0 - 9.4.2.237.3 HE PHY Capabilities Information field */
	/* field name */				/* field offset, bits */
#if IEEE80211_AX_IS_DRAFT_20
#define IEEE80211_HECAP_PHY_DUAL_BAND			0, 1	/* B0 Dual Band Support */
#else
								/* B0 Reserved */
#endif
#define IEEE80211_HECAP_PHY_CW_SET			1, 7	/* B1-7 Channel Width Set */
#define IEEE80211_HECAP_PHY_PUNC_PRMBL_RX		8, 4	/* B8-11 Punctured Preamble Rx */
#define IEEE80211_HECAP_PHY_DEV_CLASS			12, 1	/* B12 Device Class */
#define IEEE80211_HECAP_PHY_LDPC			13, 1	/* B13 LDPC Coding in Payload */
#define IEEE80211_HECAP_PHY_SU_PPDU_1LTF_SGI		14, 1	/* B14 HE SU PPDUs with 1x HE-LTF And 0.8 us GI */
#define IEEE80211_HECAP_PHY_MIDAMBLE_MAX_NSTS		15, 2	/* B15-16 Midamble Tx/Rx Max NSTS */
#define IEEE80211_HECAP_PHY_NDP_4LTF_LGI		17, 1	/* B17 NDP With 4x HE-LTF And 3.2 us GI */
#define IEEE80211_HECAP_PHY_STBC_TX_LE80		18, 1	/* B18 STBC TX <= 80 MHz */
#define IEEE80211_HECAP_PHY_STBC_RX_LE80		19, 1	/* B19 STBC RX <= 80 MHz */
#define IEEE80211_HECAP_PHY_DOPPLER_TX			20, 1	/* B20 Doppler Tx */
#define IEEE80211_HECAP_PHY_DOPPLER_RX			21, 1	/* B21 Doppler Rx */
#define IEEE80211_HECAP_PHY_FULL_BW_UL_MUMIMO		22, 1	/* B22 Full Bandwidth UL MU-MIMO */
#define IEEE80211_HECAP_PHY_PART_BW_UL_MUMIMO		23, 1	/* B23 Partial Bandwidth UL MU-MIMO */
#define IEEE80211_HECAP_PHY_DCM_MAX_CSTELL_TX		24, 2	/* B24-25 DCM Max Constellation Tx */
#define IEEE80211_HECAP_PHY_DCM_MAX_NSS_TX		26, 1	/* B26 DCM Max NSS Tx */
#define IEEE80211_HECAP_PHY_DCM_MAX_CSTELL_RX		27, 2	/* B27-28 DCM Max Constellation Rx */
#define IEEE80211_HECAP_PHY_DCM_MAX_NSS_RX		29, 1	/* B29 DCM Max NSS Rx */
#define IEEE80211_HECAP_PHY_RX_MU_PPDU			30, 1	/* B30 Rx HE MU PPDU From Non-AP STA */
#define IEEE80211_HECAP_PHY_SU_BFER			31, 1	/* B31 SU Beamformer */
#define IEEE80211_HECAP_PHY_SU_BFEE			32, 1	/* B32 SU Beamformee */
#define IEEE80211_HECAP_PHY_MU_BFER			33, 1	/* B33 MU Beamformer */
#define IEEE80211_HECAP_PHY_BFEE_STS_LE80		34, 3	/* B34-36 Beamformee STS <= 80 MHz */
#define IEEE80211_HECAP_PHY_BFEE_STS_GT80		37, 3	/* B37-39 Beamformee STS > 80 MHz */
#define IEEE80211_HECAP_PHY_SND_DIM_LE80		40, 3	/* B40-42 Number of Sounding Dimensions for <= 80Mhz */
#define IEEE80211_HECAP_PHY_SND_DIM_GT80		43, 3	/* B43-45 Number of Sounding Dimensions for > 80Mhz */
#define IEEE80211_HECAP_PHY_NG_16_SU_FB			46, 1	/* B46 Ng = 16 SU Feedback */
#define IEEE80211_HECAP_PHY_NG_16_MU_FB			47, 1	/* B47 Ng = 16 MU Feedback */
#define IEEE80211_HECAP_PHY_CB_SZ_4_2_SU_FB		48, 1	/* B48 Codebook Size (phi, psi) = {4, 2} SU Feedback */
#define IEEE80211_HECAP_PHY_CB_SZ_7_5_MU_FB		49, 1	/* B49 Codebook Size (phi, psi) = {7, 5} MU Feedback */
#define IEEE80211_HECAP_PHY_TRIG_SU_BF_FB		50, 1	/* B50 Triggered SU Beamforming Feedback */
#define IEEE80211_HECAP_PHY_TRIG_MU_BF_PART_FB		51, 1	/* B51 Triggered MU Beamforming Partial BW Feedback */
#define IEEE80211_HECAP_PHY_TRIG_CQI_FB			52, 1	/* B52 Triggered CQI Feedback */
#define IEEE80211_HECAP_PHY_PART_BW_ER			53, 1	/* B53 Partial Bandwidth Extended Range */
#define IEEE80211_HECAP_PHY_PART_BW_DL_MUMIMO		54, 1	/* B54 Partial Bandwidth DL MU-MIMO */
#define IEEE80211_HECAP_PHY_PPE_THRES_PRESENT		55, 1	/* B55 PPE Threshold Present */
#define IEEE80211_HECAP_PHY_SRP_SR_SUPPORT		56, 1	/* B56 SRP-based SR Support */
#define IEEE80211_HECAP_PHY_PWR_BOOST_FACTOR		57, 1	/* B57 Power Boost Factor Support */
#define IEEE80211_HECAP_PHY_PPDU_4LTF_SGI		58, 1	/* B58 HE SU PPDU And HE MU PPDU With 4x HE-LTF And 0.8 us GI */
#define IEEE80211_HECAP_PHY_MAX_NC			59, 3	/* B59-61 Max Nc */
#define IEEE80211_HECAP_PHY_STBC_TX_GT80		62, 1	/* B62 STBC TX > 80 MHz */
#define IEEE80211_HECAP_PHY_STBC_RX_GT80		63, 1	/* B63 STBC RX > 80 MHz */
#define IEEE80211_HECAP_PHY_ER_SU_PPDU_4LTF_SGI		64, 1	/* B64 HE ER SU PPDU With 4x HE-LTF And 0.8 us GI */
#define IEEE80211_HECAP_PHY_20M_IN_40M_PPDU_24G		65, 1	/* B65 20 MHz In 40 MHz HE PPDU In 2.4 GHz Band */
#define IEEE80211_HECAP_PHY_20M_IN_160M_PPDU		66, 1	/* B66 20 MHz In 160/80+80 MHz HE PPDU */
#define IEEE80211_HECAP_PHY_80M_IN_160M_PPDU		67, 1	/* B67 80 MHz In 160/80+80 MHz HE PPDU */
#define IEEE80211_HECAP_PHY_ER_SU_PPDU_1LTF_SGI		68, 1	/* B68 HE ER SU PPDU With 1x HE-LTF And 0.8 us GI */
#define IEEE80211_HECAP_PHY_MIDAMBLE_2_1LTF		69, 1	/* B69 Midamble Tx/Rx 2x And 1x HE-LTF */
#if !IEEE80211_AX_IS_DRAFT_20
#define IEEE80211_HECAP_PHY_DCM_MAX_BW			70, 2	/* B70-B71 DCM Max BW */
#define IEEE80211_HECAP_PHY_LT16_SIGB_SYMBOL		72, 1	/* B72 Longer Than 16 HE SIG-B OFDM Symbols Support */
#define IEEE80211_HECAP_PHY_NON_TRIG_CQI_FB		73, 1	/* B73 NonTriggered CQI Feedback */
#define IEEE80211_HECAP_PHY_1024QAM_LT242RU_TX		74, 1	/* B74 Tx 1024-QAM < 242-tone RU Support */
#define IEEE80211_HECAP_PHY_1024QAM_LT242RU_RX		75, 1	/* B75 Rx 1024-QAM < 242-tone RU Support */
#define IEEE80211_HECAP_PHY_RX_FULL_BW_SU_COMP_SIGB	76, 1	/* B76 Rx Full BW SU Using HE MU PPDU With Compressed SIGB */
#define IEEE80211_HECAP_PHY_RX_FULL_BW_SU_NONCOMP_SIGB	77, 1	/* B77 Rx Full BW SU Using HE MU PPDU With NonCompressed SIGB */
#endif /* !IEEE80211_AX_IS_DRAFT_20 */
#if IEEE80211_AX_IS_DRAFT_40
#define IEEE80211_HECAP_PHY_NOM_PPAD			78, 2	/* B78-79 Nominal Packet Padding */
#endif

/* Tx Rx HE-MCS NSS Support Macros */
#define IEEE80211_HECAP_HE_MCS_NSS_GET(_mcs_nss_map) \
	(((uint8_t *)(_mcs_nss_map))[0] | (((uint8_t *)(_mcs_nss_map))[1] << 8))
#define IEEE80211_HECAP_HE_MCS_NSS_GET_RX_MAP(_tx_rx_mcs_nss) \
	IEEE80211_HECAP_HE_MCS_NSS_GET(_tx_rx_mcs_nss)
#define IEEE80211_HECAP_HE_MCS_NSS_GET_TX_MAP(_tx_rx_mcs_nss) \
	IEEE80211_HECAP_HE_MCS_NSS_GET((uint8_t *)(_tx_rx_mcs_nss) + 2)

#define IEEE80211_HECAP_HE_MCS_NSS_SET(_mcs_nss_map, _m) \
	{((uint8_t *)(_mcs_nss_map))[0] = (_m) & 0xff; \
	 ((uint8_t *)(_mcs_nss_map))[1] = ((_m) >> 8) & 0xff;}
#define IEEE80211_HECAP_HE_MCS_NSS_SET_RX_MAP(_tx_rx_mcs_nss, _m) \
	IEEE80211_HECAP_HE_MCS_NSS_SET(_tx_rx_mcs_nss, _m)
#define IEEE80211_HECAP_HE_MCS_NSS_SET_TX_MAP(_tx_rx_mcs_nss, _m) \
	IEEE80211_HECAP_HE_MCS_NSS_SET((uint8_t *)(_tx_rx_mcs_nss) + 2, _m)

#define IEEE80211_HECAP_HE_MCS_NSS_SET_RX_TX(_tx_rx_mcs_nss, _src) \
	{IEEE80211_HECAP_HE_MCS_NSS_SET_RX_MAP(_tx_rx_mcs_nss, (_src)->rx_mcs_map); \
	 IEEE80211_HECAP_HE_MCS_NSS_SET_TX_MAP(_tx_rx_mcs_nss, (_src)->tx_mcs_map);}
#define IEEE80211_HECAP_HE_MCS_NSS_SET_LE80MHZ(_hecap, _src) \
	IEEE80211_HECAP_HE_MCS_NSS_SET_RX_TX((_hecap)->he_mcs_map_le80, _src)
#define IEEE80211_HECAP_HE_MCS_NSS_SET_160MHZ(_hecap, _src) \
	IEEE80211_HECAP_HE_MCS_NSS_SET_RX_TX(IEEE80211_IE_HECAP_HE_MCS_160MHZ(_hecap), _src)
#define IEEE80211_HECAP_HE_MCS_NSS_SET_80P80MHZ(_hecap, _src) \
	IEEE80211_HECAP_HE_MCS_NSS_SET_RX_TX(IEEE80211_IE_HECAP_HE_MCS_80P80MHZ(_hecap), _src)

/* Draft P802.11ax_D3.0 - 9.4.2.237.5 PPE Thresholds field */
	/* field name */				/* field offset, bits */
#define IEEE80211_HECAP_PPET_NSTS			0, 3	/* B0-B2 NSTS */
#define IEEE80211_HECAP_PPET_RU_IDX_BITMASK		3, 4	/* B3-B6 RU Index Bitmask */

#define IEEE80211_HECAP_MERGE_MACCAPS(_field, _name, _ie) \
	(rmaccaps)->_field = (IEEE80211_FLD_GET((_ie)->he_mac_cap, IEEE80211_HECAP_MAC_##_name) & (lmaccaps)->_field)
#define IEEE80211_HECAP_MERGE_PHYCAPS(_field, _name, _ie) \
	(rphycaps)->_field = (IEEE80211_FLD_GET((_ie)->he_phy_cap, IEEE80211_HECAP_PHY_##_name) & (lphycaps)->_field)
/*
 * 802.11ax HE Operation Element
 */
struct ieee80211_ie_heop {
	uint8_t heop_id;		/* Element ID */
	uint8_t heop_len;		/* length in bytes */
	uint8_t heop_id_ext;		/* Element ID Extension */
#if IEEE80211_AX_IS_DRAFT_20
	uint8_t heop_params[4];		/* HE Operation Parameters */
#else
	uint8_t heop_params[3];		/* HE Operation Parameters */
	uint8_t heop_bsscolor_info[1];	/* BSS Color Information */
#endif
	uint8_t heop_basic_mcs_nss[2];	/* Basic HE MCS and NSS Set */
//	uint8_t vhtop_info[0/3];	/* VHT Operation Information(optional) */
//	uint8_t heop_maxbssid[0/1];	/* Max Co-Located BSSID Indicator(optional) */
//	uint8_t heop_6gop_info[0/5];	/* 6 GHz Operation Information(optional) */
} __packed;

struct ieee80211_ie_6gop_info {
	uint8_t primary_chan;
	uint8_t control;
	uint8_t center_freq0;
	uint8_t center_freq1;
	uint8_t min_rate;
} __packed;

/* Draft P802.11ax_D6.0 - 9.4.2.248 "6 GHz Operation Information" field */
	/* field name */			/* field offset, bits */
#define IEEE80211_HEOP_6GOP_CW			0, 2	/* B0-B1 channel width */
#define IEEE80211_HEOP_6GOP_DB			2, 1	/* B2 Duplicate beacon */
#define IEEE80211_HEOP_6GOP_RI			3, 3	/* B3-B5 Regulatory info */

#define IEEE80211_HEOP_6GOP_RI_INDOOR_AP	0  /* Indoor Access Point */
#define IEEE80211_HEOP_6GOP_RI_STD_POWER_AP	1  /* Standard Power Access Point */

#define IEEE80211_HEOP_GET_VHTOP_PRESENT(_heop)	\
	IEEE80211_FLD_GET((_heop)->heop_params, IEEE80211_HEOP_PARAM_VHTOP_PRESENT)
#define IEEE80211_HEOP_GET_COLOCATED_BSS(_heop)	\
	IEEE80211_FLD_GET((_heop)->heop_params, IEEE80211_HEOP_PARAM_COLOCATED_BSS)
#define IEEE80211_HEOP_GET_6GOP_PRESENT(_heop)	\
	IEEE80211_FLD_GET((_heop)->heop_params, IEEE80211_HEOP_PARAM_6GOP_PRESENT)

#define IEEE80211_IE_HEOP_VHTOP_INFO_SIZE	3
#define IEEE80211_IE_HEOP_VHTOP_INFO_OFFSET	sizeof(struct ieee80211_ie_heop)
#define IEEE80211_IE_HEOP_VHTOP_INFO(_heop) \
	((struct ieee80211_vhtop_info *)((uint8_t *)(_heop) + IEEE80211_IE_HEOP_VHTOP_INFO_OFFSET))

#define IEEE80211_IE_HEOP_MAXBSSID_SIZE		1
#define IEEE80211_IE_HEOP_MAXBSSID_OFFSET(_vhtop_info) \
	(IEEE80211_IE_HEOP_VHTOP_INFO_OFFSET + \
		((_vhtop_info) ? IEEE80211_IE_HEOP_VHTOP_INFO_SIZE : 0))
#define IEEE80211_IE_HEOP_MAXBSSID_OPT(_heop, _vhtop_info) \
	((uint8_t *)(_heop) + IEEE80211_IE_HEOP_MAXBSSID_OFFSET(_vhtop_info))
#define IEEE80211_IE_HEOP_MAXBSSID(_heop) IEEE80211_IE_HEOP_MAXBSSID_OPT(_heop, \
		IEEE80211_HEOP_GET_VHTOP_PRESENT(_heop))

#define IEEE80211_IE_HEOP_6GOP_INFO_SIZE	sizeof(struct ieee80211_ie_6gop_info)
#define IEEE80211_IE_HEOP_6GOP_INFO_OFFSET(_vhtop_info, _maxbssid) \
	(IEEE80211_IE_HEOP_MAXBSSID_OFFSET(_vhtop_info) + \
		((_maxbssid) ? IEEE80211_IE_HEOP_MAXBSSID_SIZE : 0))
#define IEEE80211_IE_HEOP_6G_OPT(_heop, _vhtop_info, _maxbssid) \
	((struct ieee80211_ie_6gop_info *)((uint8_t *)(_heop) + \
					IEEE80211_IE_HEOP_6GOP_INFO_OFFSET(_vhtop_info, _maxbssid)))
#define IEEE80211_IE_HEOP_6GOP_INFO(_heop) IEEE80211_IE_HEOP_6G_OPT(_heop, \
		IEEE80211_HEOP_GET_VHTOP_PRESENT(_heop), IEEE80211_HEOP_GET_COLOCATED_BSS(_heop))

#define IEEE80211_IE_HEOP_SIZE_OPT(_vhtop_info, _colocated_bss, _6g_info) \
	(IEEE80211_IE_HEOP_6GOP_INFO_OFFSET(_vhtop_info, _colocated_bss) + \
		((_6g_info) ? IEEE80211_IE_HEOP_6GOP_INFO_SIZE : 0))
#define IEEE80211_IE_HEOP_SIZE(_heop) \
	IEEE80211_IE_HEOP_SIZE_OPT(IEEE80211_HEOP_GET_VHTOP_PRESENT(_heop), \
		IEEE80211_HEOP_GET_COLOCATED_BSS(_heop), IEEE80211_HEOP_GET_6GOP_PRESENT(_heop))

#define IEEE80211_IE_HEOP_SIZE_MIN sizeof(struct ieee80211_ie_heop)
#define IEEE80211_IE_HEOP_SIZE_MAX IEEE80211_IE_HEOP_SIZE_OPT(1, 1, 1)
#define IEEE80211_IE_HEOP_BODY_MIN (IEEE80211_IE_HEOP_SIZE_MIN - IEEE80211_IE_HDRLEN)
#define IEEE80211_IE_HEOP_BODY_MAX (IEEE80211_IE_HEOP_SIZE_MAX - IEEE80211_IE_HDRLEN)

/* Draft P802.11ax_D3.0 - 9.4.2.238 HE Operation element */
	/* field name */				/* field offset, bits */
#if IEEE80211_AX_IS_DRAFT_20
#define IEEE80211_HEOP_BSSCOLOR				0, 6	/* B0-5 BSS Color */
#define IEEE80211_HEOP_PARAM_DEF_PE_DUR			6, 3	/* B6-8 Default PE Duration */
#define IEEE80211_HEOP_PARAM_TWT_REQUIRED		9, 1	/* B9 TWT Required */
#define IEEE80211_HEOP_PARAM_TXOP_DUR_RTS_THRES		10, 10	/* B10-19 TXOP Duration RTS Threshold */
#define IEEE80211_HEOP_BSSCOLOR_PARTIAL			20, 1	/* B20 Partial BSS Color */
#define IEEE80211_HEOP_PARAM_VHTOP_PRESENT		21, 1	/* B21 VHT Operation Information Present */
#define IEEE80211_HEOP_PARAM_COLOCATED_BSS		28, 1	/* B28 Multiple BSSID AP */
#define IEEE80211_HEOP_BSSCOLOR_DISABLE			30, 1	/* B30 BSS Color Disabled */
#else
#define IEEE80211_HEOP_PARAM_DEF_PE_DUR			0, 3	/* B0-2 Default PE Duration */
#define IEEE80211_HEOP_PARAM_TWT_REQUIRED		3, 1	/* B3 TWT Required */
#define IEEE80211_HEOP_PARAM_TXOP_DUR_RTS_THRES		4, 10	/* B4-13 TXOP Duration RTS Threshold */
#define IEEE80211_HEOP_PARAM_VHTOP_PRESENT		14, 1	/* B14 VHT Operation Information Present */
#define IEEE80211_HEOP_PARAM_COLOCATED_BSS		15, 1	/* B15 Co-Located BSS */
#define IEEE80211_HEOP_PARAM_ER_SU_DISABLE		16, 1	/* B16 ER SU Disable */
#define IEEE80211_HEOP_PARAM_6GOP_PRESENT		17, 1	/* B17 6G Operation Info Present */

#define IEEE80211_HEOP_BSSCOLOR				0, 6	/* B0-5 BSS Color */
#define IEEE80211_HEOP_BSSCOLOR_PARTIAL			6, 1	/* B6 Partial BSS Color */
#define IEEE80211_HEOP_BSSCOLOR_DISABLE			7, 1	/* B7 BSS Color Disabled */
#endif

#define IEEE80211_BSSCOLOR_TBTT_COUNT_DEFAULT	10
#define IEEE80211_BSSCOLOR_MASK			0x3F
#define IEEE80211_BSSCOLOR_USED_BITMAP_MASK		0xFFFFFFFFFFFFFFFE
#define IEEE80211_BSSCOLOR_USED_BITMAP_ALL_SET(bitmap) \
	((*(uint64_t *)bitmap) & IEEE80211_BSSCOLOR_USED_BITMAP_MASK) == IEEE80211_BSSCOLOR_USED_BITMAP_MASK
#define IEEE80211_BSSCOLOR_COLLISION_PERIOD_MAX		50
#define IEEE80211_BSSCOLOR_COLLISION_PERIOD_MIN		10
#define IEEE80211_BSSCOLOR_AP_COLLISION_PERIOD_DEFAULT	IEEE80211_BSSCOLOR_COLLISION_PERIOD_MAX
#define IEEE80211_BSSCOLOR_STA_COLLISION_PERIOD_DEFAULT	IEEE80211_BSSCOLOR_COLLISION_PERIOD_MIN
#define IEEE80211_BSSCOLOR_SCAN_DURATION		200

/* Get a random non-zero BSS_COLOR */
#define IEEE80211_BSSCOLOR_RANDOM(_rndbuf)	\
	((_rndbuf) % IEEE80211_BSSCOLOR_MASK + 1)
#define IEEE80211_BSSCOLOR_IS_VALID(_bsscolor)	\
	(((_bsscolor) > 0) && ((_bsscolor) <= IEEE80211_BSSCOLOR_MASK))

/* Draft P802.11ax_D3.0 - 9.4.2.241 Spatial Reuse Parameter Set element */
struct ieee80211_ie_sr_params {
	struct ieee80211_ie_ext header;
	uint8_t sr_ctrl;	/* SR Control */
	/* NonSRG OBSS PD Max Offset (0 or 1) */
	/* SRG OBSS PD Min Offset (0 or 1) */
	/* SRG OBSS PD Max Offset (0 or 1) */
	/* SRG BSS Color Bitmap (0 or 8) */
	/* SRG Partial BSSID Bitmap (0 or 8) */
} __packed;
#define IEEE80211_SR_CTRL_SRP_DISALLOW			0x01
#define IEEE80211_SR_CTRL_SRP_DISALLOW_S		0
#define IEEE80211_SR_CTRL_NON_SRG_OBSS_PD_DISALLOW	0x02
#define IEEE80211_SR_CTRL_NON_SRG_OBSS_PD_DISALLOW_S	1
#define IEEE80211_SR_CTRL_NON_SRG_OFFSET_PRESENT	0x04
#define IEEE80211_SR_CTRL_NON_SRG_OFFSET_PRESENT_S	2
#define IEEE80211_SR_CTRL_SRG_INFO_PRESENT		0x08
#define IEEE80211_SR_CTRL_SRG_INFO_PRESENT_S		3
#define IEEE80211_SR_CTRL_HE_SIGA_SR_VAL15_ALLOW	0x10
#define IEEE80211_SR_CTRL_HE_SIGA_SR_VAL15_ALLOW_S	4

#define IEEE80211_IE_SR_PARAM_SIZE(_non_srg, _srg_info)	\
	(sizeof(struct ieee80211_ie_sr_params) + ((_non_srg) ? 1 : 0) + ((_srg_info) ? 18 : 0))
#define IEEE80211_IE_SR_PARAM_SIZE_MIN	IEEE80211_IE_SR_PARAM_SIZE(0, 0)
#define IEEE80211_IE_SR_PARAM_SIZE_MAX	IEEE80211_IE_SR_PARAM_SIZE(1, 1)
#define IEEE80211_IE_SR_PARAM_BODY_MIN	(IEEE80211_IE_SR_PARAM_SIZE_MIN - IEEE80211_IE_HDRLEN)
#define IEEE80211_IE_SR_PARAM_BODY_MAX	(IEEE80211_IE_SR_PARAM_SIZE_MAX - IEEE80211_IE_HDRLEN)

/* Draft P802.11ax_D3.0 - 9.4.2.243 BSS Color Change Announcement element */
struct ieee80211_ie_bsscolor_change {
	struct ieee80211_ie_ext header;
	uint8_t color_switch_countdown;
	uint8_t new_color_info;
} __packed;

/* Draft P802.11ax_D4.0 - 9.4.2.256 UL MU Power Capabilities element */
#define IEEE80211_NUM_UL_MU_PWRS	(IEEE80211_AX_MCS_MAX-1)
struct ieee80211_ie_ul_mu_pwr {
	struct ieee80211_ie_ext header;
	uint8_t rel_max_pwr[IEEE80211_NUM_UL_MU_PWRS];
} __packed;
#define IEEE80211_UL_MU_PWR_IE_SIZE \
	(sizeof(struct ieee80211_ie_ul_mu_pwr) - IEEE80211_IE_HDRLEN)

/* STA-ID list for HE MU PPDU */
#define HE_MU_NOT_MULTI_BSSID_ASSOC_MULTI_STA_ID	(0)
#define HE_MU_UNASSOC_MULTI_STA_ID			(2045)
#define HE_MU_MULTI_BSSID_ASSOC_MULTI_STA_ID		(2047)

#define IEEE80211_HETRIGGER_DEFAULT_LENGTH		29
#define IEEE80211_HETRIGGER_USERINFO_LENGTH		5
#define IEEE80211_HETRIGGER_CMNINFO_LENGTH		8

/* Trigger Type Variant User Information length */
#define IEEE80211_HETRIGGER_BASIC_USRINFO_LENGTH	1
#define IEEE80211_HETRIGGER_BASIC_CMNINFO_LENGTH	0
#define IEEE80211_HETRIGGER_BFRP_USRINFO_LENGTH		1
#define IEEE80211_HETRIGGER_BFRP_CMNINFO_LENGTH		0
#define IEEE80211_HETRIGGER_MUBAR_USRINFO_LENGTH	2 /* Also contains a variable part */
#define IEEE80211_HETRIGGER_MUBAR_CMNINFO_LENGTH	0
#define IEEE80211_HETRIGGER_MURTS_USRINFO_LENGTH	0
#define IEEE80211_HETRIGGER_MURTS_CMNINFO_LENGTH	0
#define IEEE80211_HETRIGGER_BSRP_USRINFO_LENGTH		0
#define IEEE80211_HETRIGGER_BSRP_CMNINFO_LENGTH		0
#define IEEE80211_HETRIGGER_GCRMUBAR_USRINFO_LENGTH	0
#define IEEE80211_HETRIGGER_GCRMUBAR_CMNINFO_LENGTH	4
#define IEEE80211_HETRIGGER_BQRP_USRINFO_LENGTH		0
#define IEEE80211_HETRIGGER_BQRP_CMNINFO_LENGTH		0
#define IEEE80211_HETRIGGER_NDPFB_USRINFO_LENGTH	5
#define IEEE80211_HETRIGGER_NDPFB_CMNINFO_LENGTH	0


/* Bit 0 - 3 */
/* HE Trigger Common Info field */
#define IEEE80211_HETRIGGER_CMNINFO_GET_TYPE(i_cmninfo) \
	((i_cmninfo)[0] & 0x0f)

#define IEEE80211_HETRIGGER_CMNINFO_SET_TYPE(hetrig, _m) \
		(hetrig)->i_cmninfo[0] = (((hetrig)->i_cmninfo[0] & ~0x0F) | ((_m) & 0x000F))

/* Bit 4 - 15 */
/* HE Trigger Common Info Length */
#define IEEE80211_HETRIGGER_CMNINFO_GET_LENGTH(i_cmninfo) \
	(i_cmninfo[0] >> 4 | (i_cmninfo[1] << 4))

#define IEEE80211_HETRIGGER_CMNINFO_SET_LENGTH(hetrig, _m) \
		(hetrig)->i_cmninfo[1] = ((((_m) & 0x0FF0) >> 4)); \
		(hetrig)->i_cmninfo[0] = (((hetrig)->i_cmninfo[0] & ~0xF0) | (((_m) & 0x0F) << 4))

/* Bit 16 */
/* HE Trigger Common Info Cascade Indication */
#define IEEE80211_HETRIGGER_CMNINFO_GET_CSCD_IND(i_cmninfo) \
	((i_cmninfo)[2] & 0x01)

#define IEEE80211_HETRIGGER_CMNINFO_SET_CSCD_IND(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x01) | ((_m) & 0x0001))

/* Bit 17 */
/* HE Trigger Common Info CS required */
#define IEEE80211_HETRIGGER_CMNINFO_GET_CSCD_REQ(i_cmninfo) \
	(((i_cmninfo)[2] & 0x02) >> 1)

#define IEEE80211_HETRIGGER_CMNINFO_SET_CSCD_REQ(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x02) | (((_m) & 0x0001) << 1))

/* Bit 18 - 19 */
/* HE Trigger Common Info Bandwidth */
#define IEEE80211_HETRIGGER_CMNINFO_GET_BW(i_cmninfo) \
	(((i_cmninfo)[2] >> 2) & 0x03)

#define IEEE80211_HETRIGGER_CMNINFO_SET_BW(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x0C) | (((_m) & 0x0003) << 2))

/* Bit 20 - 21 */
/* HE Trigger Common Info GI and LTF Type */
#define IEEE80211_HETRIGGER_CMNINFO_GET_GI_LTF(i_cmninfo) \
	(((i_cmninfo)[2] >> 4) & 0x03)

#define IEEE80211_HETRIGGER_CMNINFO_SET_GI_LTF(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x30) | (((_m) & 0x0003) << 4))

/* Bit 22 */
/* HE Trigger Common Info MU MIMO LTF mode */
#define IEEE80211_HETRIGGER_CMNINFO_GET_MUMIMO_LTF(i_cmninfo) \
	(((i_cmninfo)[2] >> 6) & 0x01)

#define IEEE80211_HETRIGGER_CMNINFO_SET_MUMIMO_LTF(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x40) | (((_m) & 0x0001) << 6))

/* Bit 23 - 25 */
/* HE Trigger Common Info number of HE-LTF symbols */
#define IEEE80211_HETRIGGER_CMNINFO_GET_LTF_SYM(i_cmninfo) \
	((((i_cmninfo)[2] >> 7) & 0x01) | (((i_cmninfo)[3] & 0x3) << 1))

#define IEEE80211_HETRIGGER_CMNINFO_GET_MID_PERIODICITY(i_cmninfo) \
	(IEEE80211_HETRIGGER_CMNINFO_GET_LTF_SYM(i_cmninfo) >> 2)

#define IEEE80211_HETRIGGER_CMNINFO_SET_LTF_SYM(hetrig, _m) \
		(hetrig)->i_cmninfo[2] = (((hetrig)->i_cmninfo[2] & ~0x80) | (((_m) & 0x0001) << 7)); \
		(hetrig)->i_cmninfo[3] = ((hetrig)->i_cmninfo[3] & ~0x03) | (((_m) & 0x0006) >> 1)

/* Bit 26 */
/* HE Trigger Common Info STBC */
#define IEEE80211_HETRIGGER_CMNINFO_GET_STBC(i_cmninfo) \
	(((i_cmninfo)[3] >> 2) & 0x01)

#define IEEE80211_HETRIGGER_CMNINFO_SET_STBC(hetrig, _m) \
		(hetrig)->i_cmninfo[3] = (((hetrig)->i_cmninfo[3] & ~0x04) | (((_m) & 0x0001) << 2))

/* Bit 27 */
/* HE Trigger Common Info LDPC Extra Symbol Segment */
#define IEEE80211_HETRIGGER_CMNINFO_GET_LDPC(i_cmninfo) \
	(((i_cmninfo)[3] >> 3) & 0x01)

#define IEEE80211_HETRIGGER_CMNINFO_SET_LDPC(hetrig, _m) \
		(hetrig)->i_cmninfo[3] = (((hetrig)->i_cmninfo[3] & ~0x08) | (((_m) & 0x0001) << 3))

/* Bit 28 - Bit 33 */
/* HE Trigger Ap Tx Power */
#define IEEE80211_HETRIGGER_CMNINFO_GET_APTX_POWER(i_cmninfo) \
	((((i_cmninfo)[3] >> 4) & 0x0f) | (((i_cmninfo)[4] & 0x03) << 4))

#define IEEE80211_HETRIGGER_CMNINFO_SET_APTX_POWER(hetrig, _m) \
		(hetrig)->i_cmninfo[3] = (((hetrig)->i_cmninfo[3] & ~0xF0) | (((_m) & 0x000F) << 4)); \
		(hetrig)->i_cmninfo[4] = (((hetrig)->i_cmninfo[4] & ~0x03) | (((_m) & 0x0030) >> 4))
#define IEEE80211_HETRIGGER_CMNINFO_MIN_APTX_POWER	(-20)
#define IEEE80211_HETRIGGER_CMNINFO_MAX_APTX_POWER	(40)
#define IEEE80211_HETRIGGER_CMNINFO_MAX_APTX_POWER_VAL	(60)

/* Bit 34 - Bit 36 */
/* HE Trigger Packet Extension */
#define IEEE80211_HETRIGGER_CMNINFO_GET_PKT_EXT(i_cmninfo) \
	(((i_cmninfo)[4] >> 2) & 0x07)

#define IEEE80211_HETRIGGER_CMNINFO_GET_PE_DISAMB(i_cmninfo) \
		((IEEE80211_HETRIGGER_CMNINFO_GET_PKT_EXT(i_cmninfo) >> 2) & 0x01)
#define IEEE80211_HETRIGGER_CMNINFO_GET_PRE_FEC_PAD(i_cmninfo) \
			(IEEE80211_HETRIGGER_CMNINFO_GET_PKT_EXT(i_cmninfo) & 0x03)

#define IEEE80211_HETRIGGER_CMNINFO_SET_PE_DISAMB(hetrig, _m) \
	((hetrig)->i_cmninfo[4] = (((hetrig)->i_cmninfo[4] & ~0x10) | (((_m) & 0x0001) << 4)))

#define IEEE80211_HETRIGGER_CMNINFO_SET_PKT_EXT(hetrig, _m) \
		(hetrig)->i_cmninfo[4] = (((hetrig)->i_cmninfo[4] & ~0x1C) | (((_m) & 0x0007) << 2));

/* Bit 37 - Bit 52 */
/* HE Trigger Spatial Reuse */
#define IEEE80211_HETRIGGER_CMNINFO_GET_SPATIAL_REUSE(i_cmninfo) \
	((((i_cmninfo)[4] >> 5) & 0x07) | ((i_cmninfo)[5] << 3) | (((i_cmninfo)[6] & 0x1f) << 11))

#define IEEE80211_HETRIGGER_CMNINFO_SET_SPATIAL_REUSE(hetrig, _m) \
		(hetrig)->i_cmninfo[4] = (((hetrig)->i_cmninfo[4] & ~0xE0) | (((_m) & 0x0007) << 5)); \
		(hetrig)->i_cmninfo[5] = ((((_m) & 0x07F8) >> 3)); \
		(hetrig)->i_cmninfo[6] = (((hetrig)->i_cmninfo[6] & ~0x1F) | (((_m) & 0xF800) >> 11))

/* Bit 53 */
/* HE Trigger Doppler */
#define IEEE80211_HETRIGGER_CMNINFO_GET_DOPPLER(i_cmninfo) \
	(((i_cmninfo)[6] >> 5) & 0x01)

#define IEEE80211_HETRIGGER_CMNINFO_SET_DOPPLER(hetrig, _m) \
		(hetrig)->i_cmninfo[6] = (((hetrig)->i_cmninfo[6] & ~0x20) | ((_m) & 0x0001) << 5)

/* Bit 54 - Bit 62*/
/* HE-SIG-A Reserved */
#define IEEE80211_HETRIGGER_CMNINFO_GET_HE_SIGA_RES(i_cmninfo) \
	((((i_cmninfo)[6] >> 6) & 0x03) | (((i_cmninfo)[7] & 0x7f) << 2))

#define IEEE80211_HETRIGGER_CMNINFO_SET_HE_SIGA_RES(hetrig, _m) \
	do { \
		(hetrig)->i_cmninfo[6] = ((hetrig)->i_cmninfo[6] & ~0xC0) | (((_m) & 0x03) << 6); \
		(hetrig)->i_cmninfo[7] = ((hetrig)->i_cmninfo[7] & ~0x7F) | (((_m) >> 2) & 0x7f);\
	} while (0)

/* HE Trigger type subfield encoding */
#define HE_TRIGGER_TYPE_BASIC			0
#define HE_TRIGGER_TYPE_BFRP			1
#define HE_TRIGGER_TYPE_MU_BAR			2
#define HE_TRIGGER_TYPE_MU_RTS			3
#define HE_TRIGGER_TYPE_BSRP			4
#define HE_TRIGGER_TYPE_GCR_MUBAR		5
#define HE_TRIGGER_TYPE_BQRP			6
#define HE_TRIGGER_TYPE_NDP_REPORT_POLL		7

/* HE Trigger BW Subfield Encoding */
#define HE_TRIGGER_CMNINFO_BW_20		0
#define HE_TRIGGER_CMNINFO_BW_40		1
#define HE_TRIGGER_CMNINFO_BW_80		2
#define HE_TRIGGER_CMNINFO_BW_160		3

/* HE Trigger GI and LTF subfield encoding */
#define HE_TRIGGER_CMNINFO_1LTF_SGI		0
#define HE_TRIGGER_CMNINFO_2LTF_SGI		1
#define HE_TRIGGER_CMNINFO_4LTF_LGI		2
#define HE_TRIGGER_CMNINFO_LTF_SGI_RESERVED	3

/* HE LTf symbols */
#define ONE_HE_LTF_SYMB				0

/* HE trigger min processing Time */
#define HE_MINTRIG_PROCTIME_8			8  /* 8 usec */
#define HE_MINTRIG_PROCTIME_16			16 /* 16 usec */

/* HE trigger padding values */
#define HE_TRIG_PAD_0				0
#define HE_TRIG_PAD_1				1
#define HE_TRIG_PAD_2				2

/* HE Trigger padding field value */
#define HE_TRIG_PAD_VALUE			0xff
#define HE_TRIG_MIN_PADLEN			2 /* in bytes */

/* HE Trigger MU-MIMO LTF Mode subfield encoding */
#define HE_TRIGGER_CMNINFO_MUMIMO_LTF_SS_PILOT	0
#define HE_TRIGGER_CMNINFO_MUMIMO_LTF_MASK_LTF	1

#define IEEE80211_HETRIGGER_USERINFO_CONT_RU_AID0 0
#define IEEE80211_HETRIGGER_USERINFO_CONT_RU_AID1 2045
#define IEEE80211_HETRIGGER_USERINFO_UNASSIG_RU_AID 2046
#define IEEE80211_HETRIGGER_USERINFO_PADDING_AID 4095
/* User Info fields */

/* Bit 0 - Bit 11 */
/* He Trigger User Info AID12 */
#define IEEE80211_HETRIGGER_USERINFO_GET_AID(userinfo) \
	((userinfo)[0] | (((userinfo)[1] & 0x0f) << 8))

#define IEEE80211_HETRIGGER_USERINFO_SET_AID(hetrig, _m) \
		hetrig[0] = (_m) & 0x00FF; \
		hetrig[1] = ((hetrig[1] & ~0x0F) | (((_m) & 0x0F00) >> 8))

/* Bit 12 - Bit 19 */
/* He Trigger UserInfo RU Allocation */
#define IEEE80211_HETRIGGER_USERINFO_GET_RU_ALLOC(i_userinfo) \
	(((i_userinfo[1] >> 4) & 0x0f) | ((i_userinfo[2] & 0x0f) << 4))

#define IEEE80211_HETRIGGER_USERINFO_SET_RU_ALLOC(hetrig, _m) \
		hetrig[1] = ((hetrig[1] & ~0xF0) | (((_m) & 0x000F) << 4)); \
		hetrig[2] = ((hetrig[2] & ~0x0F) | (((_m) & 0x00F0) >> 4))

/* Bit 20 */
/* He Trigger user Info Coding Type */
#define IEEE80211_HETRIGGER_USERINFO_GET_CODING_TYPE(i_userinfo) \
	(((i_userinfo)[2] >> 4) & 0x01)

#define IEEE80211_HETRIGGER_USERINFO_SET_CODING_TYPE(hetrig, _m) \
		hetrig[2] = ((hetrig[2] & ~0x10) | (((_m) & 0x0001) << 4))

/* Bit 21 - Bit 24 */
/* He Trigger user Info MCS */
#define IEEE80211_HETRIGGER_USERINFO_GET_MCS(i_userinfo) \
	((((i_userinfo)[2] >> 5) & 0x07) | (((i_userinfo)[3] & 0x01) << 3))

#define IEEE80211_HETRIGGER_USERINFO_SET_MCS(hetrig, _m) \
		hetrig[2] = ((hetrig[2] & ~0xE0) | (((_m) & 0x0007) << 5)); \
		hetrig[3] = ((hetrig[3] & ~0x01) | (((_m) & 0x0008) >> 3))

/* Bit 25 */
/* He Trigger user Info DCM */
#define IEEE80211_HETRIGGER_USERINFO_GET_DCM(i_userinfo) \
	(((i_userinfo)[3] >> 1) & 0x01)

#define IEEE80211_HETRIGGER_USERINFO_SET_DCM(hetrig, _m) \
		hetrig[3] = ((hetrig[3] & ~0x02) | (((_m) & 0x0001) << 1))

/* Bit 26 - Bit 31 */
/* He Trigger SS Allocation */
#define IEEE80211_HETRIGGER_USERINFO_GET_SS_ALLOC(i_userinfo) \
	(((i_userinfo)[3] >> 2) & 0x3f)
#define IEEE80211_HETRIGGER_USERINFO_GET_START_SS(i_userinfo) \
	(IEEE80211_HETRIGGER_USERINFO_GET_SS_ALLOC(i_userinfo) & 0x07)
#define IEEE80211_HETRIGGER_USERINFO_GET_NSS(i_userinfo) \
		(IEEE80211_HETRIGGER_USERINFO_GET_SS_ALLOC(i_userinfo) >> 3)

#define IEEE80211_HETRIGGER_USERINFO_SET_SS_ALLOC(hetrig, _m) \
		hetrig[3] = ((hetrig[3] & ~0xFC) | (((_m) & 0x003F) << 2))
#define IEEE80211_HETRIGGER_USERINFO_MAKE_SS_ALLOC(start_ss, num_ss) \
		(((start_ss) & 0x07) | (((num_ss) & 0x07) << 3))

/* Bit 32 - Bit 38 */
/* He Trigger user Info Target RSSI */
#define IEEE80211_HETRIGGER_USERINFO_GET_TARGET_RSSI(i_userinfo) \
	((i_userinfo)[4] & 0x7f)

#define IEEE80211_HETRIGGER_USERINFO_SET_TARGET_RSSI(hetrig, _m) \
		hetrig[4] = ((hetrig[4] & ~0x7F) | ((_m) & 0x007F ))

/* Trigger type based common Info */
/* Basic trigger variant cmn info */
/* Bit0 - Bit1 */
#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_GET_MPDUSPACING(i_userinfo) \
	((i_userinfo)[5] & 0x03)

#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_SET_MPDUSPACING(basictrig, _m) \
		basictrig[0] = ((basictrig[0] & ~0x03) | ((_m) & 0x3))
/* Bit2 - Bit4 */
#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_GET_AGGLIMIT(i_userinfo) \
	(((i_userinfo)[5] >> 2) & 0x07)

#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_SET_AGGLIMIT(basictrig, _m) \
		basictrig[0] = ((basictrig[0] & ~0x1C) | (((_m) & 0x7) << 2))
/* Bit6 - Bit7 */
#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_GET_PREFAC(i_userinfo) \
	(((i_userinfo)[5] >> 6) & 0x03)
#define IEEE80211_HETRIGGER_BASIC_VARUSERINFO_SET_PREFAC(basictrig, _m) \
		basictrig[0] = ((basictrig[0] & ~0xC0) | (((_m) & 0x3) << 6))

/* HE Trigger Frame */
struct ieee80211_frame_he_trigger {
	u_int8_t i_fc[2];
	u_int8_t i_dur[2];
	u_int8_t i_ra[6];
	u_int8_t i_ta[6];
	u_int8_t i_cmninfo[8];
	//u_int8_t i_varcmninfo[0];		/* variable trigger dependent common info */
	//u_int8_t i_userinfo[5];
	//u_int8_t i_varuserinfo[0];		/* Variable trigger dependent user info */
	/*FCS*/
} __packed;

struct ieee80211_frame_he_trigger_common_info {
	uint8_t i_cmninfo[8];
} __packed;

struct ieee80211_frame_he_trigger_basic {
	uint8_t i_cmninfo[8];
	uint8_t i_userinfo[6];
} __packed;

struct ieee80211_frame_he_trigger_mu_cbar {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[6];
	uint8_t i_ta[6];
	uint8_t i_cmninfo[8];
	uint8_t i_userinfo[5];
	uint8_t i_bar_ctl[2];
	__le16 i_back_seq;
} __packed;

struct ieee80211_frame_he_trigger_mu_rts {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[6];
	uint8_t i_ta[6];
	uint8_t i_cmninfo[8];
	uint8_t i_userinfo[5];
} __packed;

struct ieee80211_frame_he_trigger_bsrp {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_ra[6];
	uint8_t i_ta[6];
	uint8_t i_cmninfo[8];
	uint8_t i_userinfo[5];
} __packed;

#define IEEE80211_HETRIGGER_BFRP_FRAME_MAX_LEN	(4 * 1024)

#define IEEE80211_HE_A_CTRL_TRS_VARIANT_S	0
#define IEEE80211_HE_A_CTRL_TRS_VARIANT		(0x03 << IEEE80211_HE_A_CTRL_TRS_VARIANT_S)
#define IEEE80211_HE_A_CTRL_TRS_ID_S		2
#define IEEE80211_HE_A_CTRL_TRS_ID		(0x0f << IEEE80211_HE_A_CTRL_TRS_ID_S)
#define IEEE80211_HE_A_CTRL_TRS_DATA_SYMB_S	6
#define IEEE80211_HE_A_CTRL_TRS_DATA_SYMB	(0x1f << IEEE80211_HE_A_CTRL_TRS_DATA_SYMB_S)
#define IEEE80211_HE_A_CTRL_TRS_RU_ALLOC_S	11
#define IEEE80211_HE_A_CTRL_TRS_RU_ALLOC	(0xff << IEEE80211_HE_A_CTRL_TRS_RU_ALLOC_S)
#define IEEE80211_HE_A_CTRL_TRS_TX_POW_S	19
#define IEEE80211_HE_A_CTRL_TRS_TX_POW		(0x1f << IEEE80211_HE_A_CTRL_TRS_TX_POW_S)
#define IEEE80211_HE_A_CTRL_TRS_TARGET_RSSI_S	24
#define IEEE80211_HE_A_CTRL_TRS_TARGET_RSSI	(0x1f << IEEE80211_HE_A_CTRL_TRS_TARGET_RSSI_S)
#define IEEE80211_HE_A_CTRL_TRS_MCS_S		29
#define IEEE80211_HE_A_CTRL_TRS_MCS		(0x03 << IEEE80211_HE_A_CTRL_TRS_MCS_S)

/* Draft P802.11ax_D6.0 - 9.4.2.261 HE 6G band capabilities element */

	/* field name */			/* field offset, bits */
#define IEEE80211_HE_6G_BAND_CAP_MPDUSPACE_FD		0, 3	/* B0-2 MPDU spacing */
#define IEEE80211_HE_6G_BAND_CAP_MAXAMPDU_EXP_FD	3, 3	/* B3-5 Max A-MPDU Len exp */
#define IEEE80211_HE_6G_BAND_CAP_MAXMPDU_LEN_FD		6, 2	/* B6-7 Max MPDU Len */
#define IEEE80211_HE_6G_BAND_CAP_SMPS_FD		9, 2	/* B9-10 SMPS */
#define IEEE80211_HE_6G_BAND_CAP_RD_RESP_FD		11, 1	/* B11 RD Responder */
#define IEEE80211_HE_6G_BAND_CAP_RX_ANT_PATT_FD		12, 1	/* B12 Rx Ant Pattern Consistency */
#define IEEE80211_HE_6G_BAND_CAP_TX_ANT_PATT_FD		13, 1	/* B13 Tx Ant Pattern Consistency */

/*
 * 802.11ax HE 6G Band Capabilities element
 */
struct ieee80211_ie_he_6g_band_cap {
	uint8_t	he_6g_band_cap_id;	/* element ID */
	uint8_t	he_6g_band_cap_len;	/* length in bytes */
	uint8_t	he_6g_band_cap_ext_id;	/* Element ID Extension */
	uint8_t	he_6g_band_cap_info[2];	/* Capabilities Information */
} __packed;

/* B0-2 MPDU spacing (usec) */
enum {
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_NA,	/* No time restriction */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_0_25,	/* 1/4 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_0_5,	/* 1/2 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_1,	/* 1 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_2,	/* 2 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_4,	/* 4 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_8,	/* 8 usec */
	IEEE80211_HE_6G_BAND_CAP_MPDUSPACING_16	/* 16 usec */
};

/* B3-5 Maximum A-MPDU Length exponent */
/* 2^(13 + Max A-MPDU) -1 */
enum {
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_8191,	/* (2^13) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_16383,	/* (2^14) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_32767,	/* (2^15) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_65535,	/* (2^16) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_131071,	/* (2^17) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_262143,	/* (2^18) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_524287,	/* (2^19) -1 */
	IEEE80211_HE_6G_BAND_CAP_MAX_A_MPDU_1048575	/* (2^20) -1 */
};

/* B6-7 Maximum MPDU Length */
enum {
	IEEE80211_HE_6G_BAND_CAP_MAX_MPDU_3895,
	IEEE80211_HE_6G_BAND_CAP_MAX_MPDU_7991,
	IEEE80211_HE_6G_BAND_CAP_MAX_MPDU_11454,
	IEEE80211_HE_6G_BAND_CAP_MAX_MPDU_RESERVED
};

/* B8 - Reserved */

/* B9-10 SM power save mode */
enum {
	IEEE80211_HE_6G_BAND_CAP_SMPS_STATIC,	/* Static mode	*/
	IEEE80211_HE_6G_BAND_CAP_SMPS_DYNAMIC,	/* Dynamic mode */
	IEEE80211_HE_6G_BAND_CAP_SMPS_NA,	/* reserve	*/
	IEEE80211_HE_6G_BAND_CAP_SMPS_NONE	/* disabled	*/
};

/* B11 RD Responder. 0 - Not supported, 1 - Supported */
#define IEEE80211_HE_6G_BAND_CAP_RD_RESP		0
/* B12 Rx Antenna Pattern consistency.0 - can change anytime, 1 - Consistent */
#define IEEE80211_HE_6G_BAND_CAP_RX_ANT_PAT_CONS	1
/* B13 Tx Antenna Pattern consistency.0 - can change anytime, 1 - Consistent */
#define IEEE80211_HE_6G_BAND_CAP_TX_ANT_PAT_CONS	1

/* 802.11ax 6G HE 6G Band Capabilities */
struct ieee80211_he_6g_band_cap {
	uint8_t mpduspacing;
	uint8_t maxampduexp;
	uint8_t maxmpdulen;
	uint8_t smps;
	uint8_t rdresp;
	uint8_t rxantcons;
	uint8_t txantcons;
	uint8_t rsvd;
} __packed;


/* Draft P802.11ax_D6.0 - 9.4.2.170 Reduced Neighbor Report element */

/* TBTT Information Header subfield */
#define IEEE80211_RNR_TBTT_HDR_INFO_FIELD_TYPE	0x00
#define IEEE80211_RNR_TBTT_HDR_FILT_AP		0x04

#define IEEE80211_RNR_TBTT_HDR_INFO_LEN		13

#define IEEE80211_RNR_TBTT_HDR_INFO_CNT		0xF0	/* Mask for Count Value */
#define IEEE80211_RNR_TBTT_HDR_INFO_CNT_S	4	/* Shift for Count Value */
#define IEEE80211_RNR_TBTT_HDR_INFO_LEN_MASK	0xFF

#define IEEE80211_RNR_NEIGH_AP_OFFSET		0xFF

/* BSS Parameters subfield */
#define IEEE80211_RNR_BSSPARAM_SAMESSID		0x02
#define IEEE80211_RNR_BSSPARAM_MBSSID		0x04
#define IEEE80211_RNR_BSSPARAM_TXBSSID		0x08
#define IEEE80211_RNR_BSSPARAM_20TUPROBE_ACT	0x20
#define IEEE80211_RNR_BSSPARAM_COLOCATED	0x40

#define IEEE80211_RNR_PSD_20MHz_CONSTRAINT	13

#define IEEE80211_RNR_MIN_LEN	4
struct tbtt_info_set {
	uint8_t	tbtt_offset;		/* Neighbor AP TBTT Offset subfield */
	uint8_t	bssid[6];		/* BSSID subfield */
	uint8_t	shortssid[4];		/* Short-ssid subfield */
	uint8_t	bss_pmtr;		/* BSS Parameters subfield */
	uint8_t psd_20mhz;		/* 20 MHz PSD subfield (in dBM/MHz) */
} __packed;

struct rnr_neigh_ap_info {
	uint8_t tbtt_hdr[2];			/* TBTT Information Header */
	uint8_t	op_class;			/* Operating Class */
	uint8_t	channel;			/* Channel Number */
	struct	tbtt_info_set tbtt_info_sets[0];	/* Variable, TBTT Information Set per VAP */
} __packed;

struct ieee80211_ie_rnr {
	uint8_t id;		/* Element ID */
	uint8_t len;		/* length in bytes */
	struct	rnr_neigh_ap_info rnr_neigh_ap_info_ele[0];	/* Neighbor AP Info fields */
} __packed;

#define IEEE80211_CALC_RNR_INFO_SIZE(_vap_index)	(sizeof(struct rnr_neigh_ap_info) + \
		(sizeof(struct tbtt_info_set) * (_vap_index)))

/*
 * Add the Quantenna OUI to a frame
 */
uint8_t ieee80211_oui_add_qtn(uint8_t *oui);

/*
 * AUTH management packets
 *
 *	octet algo[2]
 *	octet seq[2]
 *	octet status[2]
 *	octet chal.id
 *	octet chal.length
 *	octet chal.text[253]
 */

typedef uint8_t *ieee80211_mgt_auth_t;

#define	IEEE80211_AUTH_ALGORITHM(auth) \
	((auth)[0] | ((auth)[1] << 8))
#define	IEEE80211_AUTH_TRANSACTION(auth) \
	((auth)[2] | ((auth)[3] << 8))
#define	IEEE80211_AUTH_STATUS(auth) \
	((auth)[4] | ((auth)[5] << 8))

#define	IEEE80211_AUTH_ALG_OPEN		0x0000
#define	IEEE80211_AUTH_ALG_SHARED	0x0001
#define	IEEE80211_AUTH_ALG_FT		0x0002
#define	IEEE80211_AUTH_ALG_SAE		0x0003
#define	IEEE80211_AUTH_ALG_LEAP		0x0080

enum {
	IEEE80211_AUTH_OPEN_REQUEST		= 1,
	IEEE80211_AUTH_OPEN_RESPONSE		= 2,
};

enum {
	IEEE80211_AUTH_SHARED_REQUEST		= 1,
	IEEE80211_AUTH_SHARED_CHALLENGE		= 2,
	IEEE80211_AUTH_SHARED_RESPONSE		= 3,
	IEEE80211_AUTH_SHARED_PASS		= 4,
};

/*
 * Reason codes
 *
 * Unlisted codes are reserved
 */

enum {
	IEEE80211_REASON_DISASSOCIATED			= 0,
	IEEE80211_REASON_UNSPECIFIED			= 1,
	IEEE80211_REASON_AUTH_EXPIRE			= 2,
	IEEE80211_REASON_AUTH_LEAVE			= 3,
	IEEE80211_REASON_ASSOC_EXPIRE			= 4,
	IEEE80211_REASON_ASSOC_TOOMANY			= 5,
	IEEE80211_REASON_NOT_AUTHED			= 6,
	IEEE80211_REASON_NOT_ASSOCED			= 7,
	IEEE80211_REASON_ASSOC_LEAVE			= 8,
	IEEE80211_REASON_ASSOC_NOT_AUTHED		= 9,
	IEEE80211_REASON_DISASSOC_BAD_POWER		= 10,
	IEEE80211_REASON_DISASSOC_BAD_SUPP_CHAN		= 11,
	IEEE80211_REASON_BSS_TRANSITION_DISASSOC	= 12,
	IEEE80211_REASON_IE_INVALID			= 13,
	IEEE80211_REASON_MIC_FAILURE			= 14,
	IEEE80211_REASON_4WAY_HANDSHAKE_TIMEOUT		= 15,
	IEEE80211_REASON_GROUP_KEY_HANDSHAKE_TIMEOUT	= 16,
	IEEE80211_REASON_IE_DIFFERENT			= 17,
	IEEE80211_REASON_INVALID_GROUP_CIPHER		= 18,
	IEEE80211_REASON_INVALID_PAIRWISE_CIPHER	= 19,
	IEEE80211_REASON_INVALID_AKMP			= 20,
	IEEE80211_REASON_UNSUPP_RSN_VERSION		= 21,
	IEEE80211_REASON_INVALID_RSN_IE_CAP		= 22,
	IEEE80211_REASON_IEEE8021X_FAILED		= 23,
	IEEE80211_REASON_CIPHER_SUITE_REJECTED		= 24,
	IEEE80211_REASON_TDLS_UNREACH			= 25, /* TDLS teardown due to peer unreachable */
	IEEE80211_REASON_TDLS_UNSPEC			= 26, /* TDLS teardown for unspecified reason */
	IEEE80211_REASON_RESERVED2			= 27,
	IEEE80211_REASON_RESERVED3			= 28,
	IEEE80211_REASON_RESERVED4			= 29,
	IEEE80211_REASON_NOT_AUTHORIZED_THIS_LOCATION	= 30,
	IEEE80211_REASON_TS_DELETED			= 31,
	IEEE80211_REASON_DISASSOC_UNSPECIFIED_QOS	= 32,
	IEEE80211_REASON_DISASSOC_QOS_AP_NO_BANDWIDTH	= 33,
	IEEE80211_REASON_DISASSOC_LOW_ACK		= 34,
	IEEE80211_REASON_DISASSOC_STA_EXCEED_TXOP	= 35,
	IEEE80211_REASON_STA_LEAVE_BSS			= 36,
	IEEE80211_REASON_STA_NOT_USE			= 37,
	IEEE80211_REASON_STA_REQUIRE_SETUP		= 38,
	IEEE80211_REASON_STA_TIMEOUT			= 39,
	IEEE80211_REASON_RESERVED6			= 40,
	IEEE80211_REASON_RESERVED7			= 41,
	IEEE80211_REASON_RESERVED8			= 42,
	IEEE80211_REASON_RESERVED9			= 43,
	IEEE80211_REASON_RESERVED10			= 44,
	IEEE80211_REASON_STA_CIPHER_NOT_SUPP		= 45,
	IEEE80211_REASON_AUTH_ACCESS_LIM		= 46,
	IEEE80211_REASON_EXT_SVC_REQ			= 47,
	IEEE80211_REASON_INVALID_FT_ACTION_FRAME_COUNT	= 48,
	IEEE80211_REASON_INVALID_PMKI			= 49,
	IEEE80211_REASON_MDE				= 50,
	IEEE80211_REASON_FTE				= 51,
	IEEE80211_REASON_SME_CANCELS_MESH_PEERING	= 52,
	IEEE80211_REASON_MESH_MAX_PEER			= 53,
	IEEE80211_REASON_MESH_CONFIG_POLICY_VIOLATION	= 54,
	IEEE80211_REASON_MESH_PEERING_CLOSE_MSG		= 55,
	IEEE80211_REASON_RESENT_DOT11_MESH_MAX_RETRIES	= 56,
	IEEE80211_REASON_CONFIRM_TIMER_TIMEOUT		= 57,
	IEEE80211_REASON_FAILS_TO_UNWRAP_GTK		= 58,
	IEEE80211_REASON_RECEIVES_INCONSISTANT_INFO	= 59,
	IEEE80211_REASON_AUTH_MESH_PEER_EXCH_FAIL	= 60,
	IEEE80211_REASON_DOES_NOT_HAVE_PROXY_INFO	= 61,
	IEEE80211_REASON_DOES_NOT_HAVE_FORWARDING_INFO	= 62,
	IEEE80211_REASON_NEXT_HOP_LINK_USE		= 63,
	IEEE80211_REASON_MAC_ADDR_EXIST_IN_MESH		= 64,
	IEEE80211_REASON_MESH_STA_PERFORMES_CS		= 65,
	IEEE80211_REASON_MESH_CS_UNSPEC			= 66,
	IEEE80211_REASON_MAX,
};

/*
 * Status codes
 */
enum {
	IEEE80211_STATUS_SUCCESS		= 0,
	IEEE80211_STATUS_UNSPECIFIED		= 1,
	IEEE80211_STATUS_TDLS_WKUP_REJ_ALT	= 2,  /* Wakeup sched rejected/alternative */
	IEEE80211_STATUS_TDLS_WKUP_REJ		= 3,  /* Wakeup sched rejected */
	IEEE80211_STATUS_SEC_DIS		= 5,  /* Security disabled */
	IEEE80211_STATUS_LIFETIME_NOTOK		= 6,  /* Unacceptable lifetime */
	IEEE80211_STATUS_BSS_INVALID		= 7,  /* Not in same BSS */
	IEEE80211_STATUS_CAPINFO		= 10,
	IEEE80211_STATUS_NOT_ASSOCED		= 11,
	IEEE80211_STATUS_OTHER			= 12,
	IEEE80211_STATUS_ALG			= 13,
	IEEE80211_STATUS_SEQUENCE		= 14,
	IEEE80211_STATUS_CHALLENGE		= 15,
	IEEE80211_STATUS_TIMEOUT		= 16,
	IEEE80211_STATUS_TOOMANY		= 17,
	IEEE80211_STATUS_BASIC_RATE		= 18,
	IEEE80211_STATUS_SP_REQUIRED		= 19,
	IEEE80211_STATUS_PBCC_REQUIRED		= 20,
	IEEE80211_STATUS_CA_REQUIRED		= 21,
	IEEE80211_STATUS_TOO_MANY_STATIONS	= 22,
	IEEE80211_STATUS_RATES			= 23,
	IEEE80211_STATUS_SHORTSLOT_REQUIRED	= 25,
	IEEE80211_STATUS_DSSSOFDM_REQUIRED	= 26,
	IEEE80211_STATUS_HT_FEATURE		= 27,
	IEEE80211_STATUS_PMF_REJECT_RETRY	= 30,
	IEEE80211_STATUS_PMF_VIOLATION		= 31,
	IEEE80211_STATUS_PEER_MECHANISM_REJECT	= 37,
	IEEE80211_STATUS_TDLS_RSNIE_INVALID	= 72, /* Invalid contents of RSNIE */
	IEEE80211_STATUS_SUGGESTED_BSS_TRANS	= 82,

	/* Quantenna */
	IEEE80211_STATUS_DENIED			= 100,
	IEEE80211_STATUS_DENIED_HE_NOT_SUPPORTED = 124,
};
#define IEEE80211_REASON_CODE_LEN	2

#define	IEEE80211_WEP_KEYLEN		5	/* 40bit */
#define	IEEE80211_WEP_IVLEN		3	/* 24bit */
#define	IEEE80211_WEP_KIDLEN		1	/* 1 octet */
#define	IEEE80211_WEP_CRCLEN		4	/* CRC-32 */
#define	IEEE80211_WEP_NKID		4	/* number of key ids */

/*
 * 802.11i defines an extended IV for use with non-WEP ciphers.
 * When the EXTIV bit is set in the key id byte an additional
 * 4 bytes immediately follow the IV for TKIP.  For CCMP the
 * EXTIV bit is likewise set but the 8 bytes represent the
 * CCMP header rather than IV+extended-IV.
 */
#define	IEEE80211_WEP_EXTIV		0x20
#define	IEEE80211_WEP_EXTIVLEN		4	/* extended IV length */
#define	IEEE80211_WEP_CCMPLEN		8	/* CCMP header */
#define	IEEE80211_WEP_MICLEN		8	/* trailing MIC */
#define	IEEE80211_WEP_ICVLEN		4	/* ICV */
#define	IEEE80211_WEP_GCMP_HDRLEN	8	/* GCMP header */
#define	IEEE80211_WEP_GCMP_MICLEN	16	/* trailing MIC */
#define	IEEE80211_WEP_CCMP_256_HDRLEN	8	/* CCMP256 header */
#define	IEEE80211_WEP_CCMP_256_MICLEN	16	/* trailing MIC */
#define	IEEE80211_WEP_GCMP_256_HDRLEN	8	/* GCMP256 header */
#define	IEEE80211_WEP_GCMP_256_MICLEN	16	/* trailing MIC */

#define	IEEE80211_CRC_LEN		4
#define IEEE80211_MAX_IE_LEN		257

/*
 * Maximum acceptable MTU is:
 *	IEEE80211_MAX_LEN - WEP overhead - CRC -
 *		QoS overhead - RSN/WPA overhead
 * Min is arbitrarily chosen > IEEE80211_MIN_LEN.  The default
 * mtu is Ethernet-compatible; it's set by ether_ifattach.
 */
#define	IEEE80211_MTU_MAX		3500
#define	IEEE80211_MTU_MSDU_SIZE		1500
#define	IEEE80211_MTU_MIN		32

#define	IEEE80211_MAX_LEN		(2300 + IEEE80211_CRC_LEN + \
	(IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN))
#define	IEEE80211_ACK_LEN \
	(sizeof(struct ieee80211_frame_ack) + IEEE80211_CRC_LEN)
#define	IEEE80211_MIN_LEN \
	(sizeof(struct ieee80211_frame_min) + IEEE80211_CRC_LEN)

/*
 * RTS frame length parameters.  The default is specified in
 * the 802.11 spec.  The max may be wrong for jumbo frames.
 */
#define	IEEE80211_RTS_DEFAULT		512
#define	IEEE80211_RTS_MIN		0
#define	IEEE80211_RTS_MAX		1048576
#define	IEEE80211_RTS_THRESH_OFF	(IEEE80211_RTS_MAX + 1)

enum ieee80211_pps_type {
	IEEE80211_PPS_TYPE_BCAST = 0,
	IEEE80211_PPS_TYPE_SSDP	= 1,
	IEEE80211_PPS_TYPE_MAX
};

struct ieee80211_pps_info {
	uint16_t	pps_ctr;
	uint16_t	drop_ctr;
	unsigned long	pps_start_time;
};

#define IEEE80211_PPS_MAX_BCAST_MIN	0
#define IEEE80211_PPS_MAX_BCAST_MAX	65535
#define IEEE80211_PPS_MAX_SSDP_MIN	0
#define IEEE80211_PPS_MAX_SSDP_MAX	65535

/*
 * Regulatory extension identifier for country IE.
 */
#define IEEE80211_REG_EXT_ID		201

/*
 * IEEE 802.11 timer synchronization function (TSF) timestamp length
 */
#define IEEE80211_TSF_LEN		8
/*
 * 802.11n defines
 */
#define IEEE80211_11N_BAWLEN		64
#define IEEE80211_11N_QLENLIM		(64*1024)

#define IEEE80211_11N_SEQINORDER_BAW(seq_front, seq_back)       \
        IEEE80211_SEQ_INORDER_LAG((seq_front), (seq_back), IEEE80211_11N_BAWLEN)

struct ieee80211_wmm_params {
	uint8_t wmm_acm;		/* ACM parameter */
	uint8_t wmm_aifsn;		/* AIFSN parameters */
	uint8_t wmm_logcwmin;		/* cwmin in exponential form */
	uint8_t wmm_logcwmax;		/* cwmax in exponential form */
	uint16_t wmm_txopLimit;		/* txopLimit */
	uint8_t wmm_noackPolicy;	/* No-Ack Policy: 0=ack, 1=no-ack */
};

#define IEEE80211_DEFAULT_BA_WINSIZE	64	/* use for explicit BA establishing, size and throughput is moderate */
#define IEEE80211_DEFAULT_BA_WINSIZE_H	256	/* use for implicit BA or explicit BA with 11ax clients,
						 * large size to support large aggregates and high throughput
						 */
#define IEEE80211_MAX_BA_WINSIZE	0x3FF

#define IEEE80211_PUT_INT16(_frm, _val)		do {		\
	put_unaligned(cpu_to_le16(_val), (__le16 *)frm);	\
	frm += 2;						\
} while (0)

#define IEEE80211_PUT_INT32(_frm, _val)		do {		\
	put_unaligned(cpu_to_le32(_val), (__le32 *)frm);	\
	frm += 4;						\
} while (0)

#define IEEE80211_PUT_INT64(_frm, _val)		do {		\
	put_unaligned(cpu_to_le64(_val), (__le64 *)frm);	\
	frm += 8;						\
} while (0)

/*value assignment for the little-endian*/
#define	ADDINT16LE(frm, v) do {			\
	frm[0] = (v) & 0xff;				\
	frm[1] = ((v) >> 8) & 0xff;			\
	frm += 2;							\
} while (0)
/* 32 bits to 32 bits */
#define	ADDINT32LE(frm, v) do {			\
	frm[0] = (v) & 0xff;				\
	frm[1] = ((v) >> 8) & 0xff;			\
	frm[2] = ((v) >> 16) & 0xff;		\
	frm[3] = ((v) >> 24) & 0xff;		\
	frm += 4;							\
} while (0)

/* 32 bits to 64 bits */
#define ADDINT32TO64LE(frm, v) do {		\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v >> 8) & 0xff;		\
	frm[2] = (v >> 16) & 0xff;		\
	frm[3] = (v >> 24) & 0xff;		\
	frm[4] = 0;				\
	frm[5] = 0;				\
	frm[6] = 0;				\
	frm[7] = 0;				\
	frm += 8;				\
} while (0)

/* 16 bits to 32 bits */
#define ADDINT16TO32LE(frm, v) do {		\
	frm[0] = (v) & 0xff;			\
	frm[1] = (v >> 8) & 0xff;		\
	frm[2] = 0;				\
	frm[3] = 0;				\
	frm += 4;				\
} while (0)

/* 8 bits to 32 bits */
#define ADDINT8TO32LE(frm, v) do {		\
	frm[0] = (v) & 0xff;			\
	frm[1] = 0;				\
	frm[2] = 0;				\
	frm[3] = 0;				\
	frm += 4;				\
} while (0)

/* value assignment */
/* 16 bits to 16 bits */
#define	ADDINT16(frm, v) do {			\
	frm[1] = (v) & 0xff;				\
	frm[0] = ((v) >> 8) & 0xff;			\
	frm += 2;							\
} while (0)
/* 32 bits to 32 bits */
#define	ADDINT32(frm, v) do {			\
	frm[3] = (v) & 0xff;				\
	frm[2] = ((v) >> 8) & 0xff;			\
	frm[1] = ((v) >> 16) & 0xff;		\
	frm[0] = ((v) >> 24) & 0xff;		\
	frm += 4;							\
} while (0)
/* 8 bits to 32 bits */
#define	ADDINT8TO32(frm, v) do {			\
	frm[3] = (v) & 0xff;				\
	frm[2] = 0;					\
	frm[1] = 0;					\
	frm[0] = 0;					\
	frm += 4;							\
} while (0)
/* 16 bits to 32 bits */
#define	ADDINT16TO32(frm, v) do {			\
	frm[3] = (v) & 0xff;				\
	frm[2] = ((v) >> 8) & 0xff;			\
	frm[1] = 0;					\
	frm[0] = 0;					\
	frm += 4;							\
} while (0)
/* 32 bits to 64 bits */
#define	ADDINT32TO64(frm, v) do {			\
	frm[7] = (v) & 0xff;				\
	frm[6] = ((v) >> 8) & 0xff;			\
	frm[5] = ((v) >> 16) & 0xff;		\
	frm[4] = ((v) >> 24) & 0xff;		\
	frm[3] = 0;							\
	frm[2] = 0;							\
	frm[1] = 0;							\
	frm[0] = 0;							\
	frm += 8;							\
} while (0)

#define	RDUINT16(frm, v) do {				\
	(v) = (frm[1] & 0xFF) +				\
	((frm[0] << 8) & 0xFF00);			\
} while (0)

#define	RDUINT32(frm, v) do {				\
	(v) = (frm[3] & 0xFF) +				\
		((frm[2] << 8) & 0xFF00) +		\
		((frm[1] << 16) & 0xFF0000) +		\
		((frm[0] << 24) & 0xFF000000);		\
} while (0)

#define	RDUINT16LE(frm, v) do {				\
	(v) = (frm[0] & 0xFF) +				\
		((frm[1] << 8) & 0xFF00);		\
} while (0)

#define	RDUINT32LE(frm, v) do {				\
	(v) = (frm[0] & 0xFF) +				\
		((frm[1] << 8) & 0xFF00) +		\
		((frm[2] << 16) & 0xFF0000) +		\
		((frm[3] << 24) & 0xFF000000);		\
} while (0)

#define	WRUINT16(frm, v) do {				\
	frm[1] = (v) & 0xff;				\
	frm[0] = ((v) >> 8) & 0xff;			\
} while (0)

#define	WRUINT16LE(frm, v) do {				\
	frm[0] = (v) & 0xff;				\
	frm[1] = ((v) >> 8) & 0xff;			\
} while (0)

#ifndef DSP_BUILD
static __inline__ int ieee80211_is_bcst(const void *p)
{
	const uint16_t *p16 = p;

	return (p16[0] == 0xFFFF) && (p16[1] == 0xFFFF) && (p16[2] == 0xFFFF);
}

/*
 * IEEE802.11w spec - Table 8-38 and section 11.1.7
 */
static __inline__ int ieee80211_mgmt_is_robust(const struct ieee80211_frame *wh) {

	int is_robust_mgmt = 0;
	const uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

	switch (subtype){
		case IEEE80211_FC0_SUBTYPE_DEAUTH:
		case IEEE80211_FC0_SUBTYPE_DISASSOC:
			is_robust_mgmt = 1;
			break;
		case IEEE80211_FC0_SUBTYPE_ACTION:
		{
			struct ieee80211_action *ia;
			ia = (struct ieee80211_action *) (void*)&wh[1];

			switch (ia->ia_category) {
				case IEEE80211_ACTION_CAT_SPEC_MGMT:
				case IEEE80211_ACTION_CAT_QOS:
				case IEEE80211_ACTION_CAT_DLS:
				case IEEE80211_ACTION_CAT_BA:
				case IEEE80211_ACTION_CAT_RM:
				case IEEE80211_ACTION_CAT_FBSS:
				case IEEE80211_ACTION_CAT_SA_QUERY:
				case IEEE80211_ACTION_CAT_PROT_DUAL_PA:
				case IEEE80211_ACTION_CAT_WNM:
				case IEEE80211_ACTION_CAT_MESH:
				case IEEE80211_ACTION_CAT_MULTIHOP:
				case IEEE80211_ACTION_CAT_VEND_PROT:
					is_robust_mgmt = 1;
					break;
				default:
					is_robust_mgmt = 0;
					break;
			}
			break;
		}
		default:
			break;

	}

	return is_robust_mgmt;
}
#endif

#define BITS_MASK(_bits) ((1 << (_bits)) - 1)
#define IEEE80211_GET_BITS(_m, _offset, _bits)		\
	(((_m) >> (_offset)) & BITS_MASK(_bits))
#define IEEE80211_SET_BITS(_m, _offset, _bits, _value)	\
	(_m) = (((_m) & ~(BITS_MASK(_bits) << (_offset))) | (((_value) & BITS_MASK(_bits)) << (_offset)))

#define IEEE80211_FLD_SET(_buf, _field, _val)	ieee80211_buf_set_bits(_buf, _field, _val)
#define IEEE80211_FLD_GET(_buf, _field)		ieee80211_buf_get_bits(_buf, _field)

static __inline__ void
ieee80211_buf_set_bits(uint8_t *buf, uint32_t bits_offset, uint8_t bits_count, uint32_t value)
{
	uint8_t bits_done;

	buf += bits_offset / NBBY;
	bits_offset %= NBBY;

	IEEE80211_SET_BITS(buf[0], bits_offset, bits_count, value);
	bits_done = NBBY - bits_offset;

	while (bits_done < bits_count) {
		buf++;
		IEEE80211_SET_BITS(buf[0], 0, bits_count - bits_done, value >> bits_done);
		bits_done += NBBY;
	}
}

static __inline__ uint32_t
ieee80211_buf_get_bits(uint8_t *buf, uint32_t bits_offset, uint8_t bits_count)
{
	uint32_t value;
	uint8_t bits_done;

	buf += bits_offset / NBBY;
	bits_offset %= NBBY;

	value = IEEE80211_GET_BITS(buf[0], bits_offset, bits_count);
	bits_done = NBBY - bits_offset;

	while (bits_done < bits_count) {
		buf++;
		value |= IEEE80211_GET_BITS(buf[0], 0, bits_count - bits_done) << bits_done;
		bits_done += NBBY;
	}

	return value;
}

static __inline__ int qtn_action_frame_check_category(const struct ieee80211_frame *const wh,
						uint8_t category, uint8_t action)
{
	const uint8_t type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	const uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
	const struct ieee80211_action *act_hdr = (void *)(wh + 1);

	if (type == IEEE80211_FC0_TYPE_MGT &&
			subtype == IEEE80211_FC0_SUBTYPE_ACTION &&
			act_hdr->ia_category == category &&
			act_hdr->ia_action == action) {
		return 1;
	}

	return 0;
}

static __inline__ int qtn_is_csa_action_frame(const struct ieee80211_frame *const wh)
{
	return qtn_action_frame_check_category(wh, IEEE80211_ACTION_CAT_SPEC_MGMT,
						IEEE80211_ACTION_S_CHANSWITCHANN);
}

static __inline__ int qtn_is_bsscolor_change_frame(const struct ieee80211_frame *const wh)
{
	return qtn_action_frame_check_category(wh, IEEE80211_ACTION_CAT_HE_PROT,
						IEEE80211_ACTION_HE_PROT_BSSCOLOR);
}

/*
 * HW calculated duration/ID value isn't enough to cover for NDP/NDPA and BF Report Poll exchanges
 * Not associated or hidden nodes can cause collision or failure which can cause
 * tx-hang issues. A fixed value is used to prevent such conditions
 */
#define QTN_FIXED_NDPA_DURATION	0xb0

/*
 * Calculate TXOP Field(B0-B6) for HE SIG-A2 according to the duration (11axD6.0: 26.11.5)
 * B0: 0 for scale 8, 1 for scale 128
 * B1-B6: Scaled value of duration information for NAV setting and protection of the TXOP
 */
#define HE_TXOP_DURATION_MAX	8448
static __inline__ uint32_t qtn_he_calc_txop(uint32_t duration)
{
	duration = MIN(duration, HE_TXOP_DURATION_MAX);
	if (duration < 512)
		return (duration >> 3) << 1;
	else
		return (((duration - 512) >> 7) << 1) | 1;
}

/* QTN Extender */
#define	IEEE80211_QTN_WDS_MASK		0x0003
#define	IEEE80211_QTN_EXTDR_ALLMASK	0xFFFF
#define	IEEE80211_QTN_EXTDR_MASK_SHIFT	16

#define	IEEE80211_QTN_WDS_ONLY		0x0000	/* 0 = Plain WDS; No WDS Extender */
#define	IEEE80211_QTN_WDS_MBS		0x0001	/* 1 = MBS-Master Base Station */
#define	IEEE80211_QTN_WDS_RBS		0x0002	/* 2 = RBS-Repeater/Remote Base Station */

static __inline__ uint32_t ieee80211_extdr_combinate(uint16_t flags, uint16_t mask)
{
	return (mask << IEEE80211_QTN_EXTDR_MASK_SHIFT) | flags;
}

#define IEEE80211_N_RATE_PREFIX 0x7F000000
#define IEEE80211_AC_RATE_PREFIX 0x7E000000
#define IEEE80211_AX_RATE_PREFIX 0x7D000000
#define IEEE80211_RATE_PREFIX_MASK 0xFF000000

#define IEEE80211U_PARAM_IPV4ADDRTYPE_MIN	0
#define IEEE80211U_PARAM_IPV4ADDRTYPE_MAX	7
#define IEEE80211U_PARAM_IPV6ADDRTYPE_MIN	0
#define IEEE80211U_PARAM_IPV6ADDRTYPE_MAX	2
#define IEEE80211U_PARAM_IP_STATUS_MAX		2
/* MU MIMO */
#define IEEE80211_MU_GRP_VALID(_grp)		\
	(((_grp) > 0) && ((_grp) < (IEEE80211_VHT_GRP_MAX_BIT_OFFSET+1)))

#define IEEE80211_MU_POS_VALID(_pos) ((_pos) < 4)

#define IEEE80211_MU_DEL_GRP(mu_grp, _grp) do {		\
	clrbit((mu_grp).member, (_grp)); \
} while (0)

#define IEEE80211_MU_ADD_GRP(mu_grp, _grp, _pos) do {	\
	setbit((mu_grp).member, (_grp)); \
	(mu_grp).pos[(_grp) >> 2] &= ~((0x03 << (((_grp) & 0x3) << 1))); \
	(mu_grp).pos[(_grp) >> 2] |= (((_pos) << (((_grp) & 0x3) << 1))); \
} while (0)

#define IEEE80211_MU_IS_GRP_MBR(mu_grp, _grp)	\
	isset((mu_grp).member, (_grp))

#define IEEE80211_MU_GRP_POS(mu_grp, _grp)	\
	(((mu_grp).pos[(_grp) >> 2] >> (((_grp) & 0x3) << 1)) & 0x3)

#define	IEEE80211_VAP_STATE_DISABLED			0
#define	IEEE80211_VAP_STATE_ENABLED			1

#define IEEE80211_MIN_BSS_GROUP	1 /* 0 - Default internal group. 1 - 31 User configurable group id */
#define IEEE80211_MAX_BSS_GROUP	32

#define IEEE80211_MAX_ACL_BLACKLIST_PER_VAP 120 /* Max number blacklist per VAP */

#define ETH_P_80211_RAW         0x0019

enum ieee80211_country_env {
	IEEE80211_COUNTRY_ENV_MBO = 0x04, /* Wi-Fi Agile Multiband Technical Specitication 1.0 on 2.3.4.*/
	IEEE80211_COUNTRY_ENV_ANY = 0x20,
	IEEE80211_COUNTRY_ENV_OUTDOOR = 0x4f,
	IEEE80211_COUNTRY_ENV_INDOOR = 0x49,
};

struct ieee80211_vendor_hdr {
	uint8_t        vendor_ie_id;		/* IEEE80211_ELEMID_VENDOR */
	uint8_t        vendor_ie_len;
	uint8_t        vendor_ie_oui[3];
	uint8_t        vendor_ie_type;
} __packed;

struct ieee80211_wfa_owe_trans {
	struct ieee80211_vendor_hdr vendor_hdr;
	uint8_t trans_bssid[IEEE80211_ADDR_LEN];
	uint8_t trans_ssid_len;
	uint8_t trans_ssid[0];
} __packed;

/* MBO information element */
#define IEEE80211_MBO_TRANS_CODE	0x6
struct ieee80211_mbo_oce_trans {
	struct ieee80211_vendor_hdr	mbo_hdr;
	uint8_t				mbo_attr_id;	/* IEEE80211_MBO_TRANS_CODE */
	uint8_t				mbo_attr_len;
#define IEEE80211_MBO_REASON_UNSPEC	0
	uint8_t				reason_code;	/* See WPA MBO spec 1.0 table 18 */
} __packed;

enum {
	IEEE80211_HE_GI_08US = 0,     /* 0.8 us */
	IEEE80211_HE_GI_16US,         /* 1.6 us */
	IEEE80211_HE_GI_32US,         /* 3.2 us */
};

/* For Unsolicited probe response frame */
#define IEEE80211_UPROBE_RESP_MIN_US		IEEE80211_TU_TO_USEC(10)
#define IEEE80211_UPROBE_RESP_MAX_US		IEEE80211_TU_TO_USEC(80)
#define IEEE80211_UPROBE_RESP_DELTA_WITH_BCN_US	IEEE80211_MS_TO_USEC(2)

/* Key management Capabilities */
#define WPA_KEY_MGMT_IEEE8021X BIT(0)
#define WPA_KEY_MGMT_PSK BIT(1)
#define WPA_KEY_MGMT_NONE BIT(2)
#define WPA_KEY_MGMT_IEEE8021X_NO_WPA BIT(3)
#define WPA_KEY_MGMT_WPA_NONE BIT(4)
#define WPA_KEY_MGMT_FT_IEEE8021X BIT(5)
#define WPA_KEY_MGMT_FT_PSK BIT(6)
#define WPA_KEY_MGMT_IEEE8021X_SHA256 BIT(7)
#define WPA_KEY_MGMT_PSK_SHA256 BIT(8)
#define WPA_KEY_MGMT_WPS BIT(9)
#define WPA_KEY_MGMT_SAE BIT(10)
#define WPA_KEY_MGMT_FT_SAE BIT(11)
#define WPA_KEY_MGMT_WAPI_PSK BIT(12)
#define WPA_KEY_MGMT_WAPI_CERT BIT(13)
#define WPA_KEY_MGMT_CCKM BIT(14)
#define WPA_KEY_MGMT_OSEN BIT(15)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B BIT(16)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B_192 BIT(17)
#define WPA_KEY_MGMT_FILS_SHA256 BIT(18)
#define WPA_KEY_MGMT_FILS_SHA384 BIT(19)
#define WPA_KEY_MGMT_FT_FILS_SHA256 BIT(20)
#define WPA_KEY_MGMT_FT_FILS_SHA384 BIT(21)
#define WPA_KEY_MGMT_OWE BIT(22)
#define WPA_KEY_MGMT_DPP BIT(23)

/* Epigram Vendor Specific IE */
struct ieee80211_ie_epigram_iotwar {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;				/* 6 */
	uint8_t bcm_ie_oui[3];			/* BCM_OUI - 0x00, 0x90, 0x4c */
	uint8_t bcm_ie_type;			/* 0x03 */
	uint8_t data1;				/* 0x00 */
	uint8_t data2;				/* 0x01 */
} __packed;

struct ieee80211_ie_epigram_vht_24g {
	uint8_t id;                             /* IEEE80211_ELEMID_VENDOR */
	uint8_t len;
	uint8_t bcm_ie_oui[3];                  /* BCM_OUI - 0x00, 0x90, 0x4c */
	uint8_t bcm_ie_type;                    /* 0x04 */
	uint8_t bcm_ie_subtype;                 /* 0x08 or 0x18 */
	struct ieee80211_ie_vhtcap	vhtcap;
	struct ieee80211_ie_vhtop	vhtop;
} __packed;

struct ieee80211_ie_epigram_vht_5g {
	uint8_t id;				/* IEEE80211_ELEMID_VENDOR */
	uint8_t len;				/* 5 */
	uint8_t bcm_ie_oui[3];			/* BCM_OUI - 0x00, 0x90, 0x4c */
	uint8_t bcm_ie_type;			/* 0x04 */
	uint8_t bcm_ie_subtype;			/* 0x00 or 0x10 */
} __packed;

#define IEEE80211_EPIGRAM_IE_MAX_SIZE	(sizeof(struct ieee80211_ie_epigram_vht_24g))


enum ieee80211_dbvc_level {
	IEEE80211_DBVC_LEVEL_0 = 0,     /* disable */
	IEEE80211_DBVC_LEVEL_1,		/* switch offchan once per 1 bcn intvl*/
	IEEE80211_DBVC_LEVEL_MAX,	/* switch offchan twice per 1 bcn intvl */
};
#define IEEE80211_DBVC_DWELL_MIN	10	/* msecs */
#define IEEE80211_DBVC_DWELL_MAX	27	/* msecs */

/**
 * @addtogroup WiFiAPIs
 * @{
 */

/**
 * \brief NFR switch level
 * \sa qcsapi_wifi_param_nfr_level
 */
enum ieee80211_nfr_dbvc_level {
	IEEE80211_NFR_DBVC_LEVEL_0 = 0,		/** disable */
	IEEE80211_NFR_DBVC_LEVEL_1,		/** switch once per 100ms */
	IEEE80211_NFR_DBVC_LEVEL_2,		/** switch twice per 100ms */
	IEEE80211_NFR_DBVC_LEVEL_3,		/** switch thrice per 100ms */
	IEEE80211_NFR_DBVC_LEVEL_MAX,		/** fixed duty cycle 50% */
};

#define IEEE80211_NFR_DBVC_DWELL_MIN	9	/* Minimum dwell time for NFR in msecs */
#define IEEE80211_NFR_DBVC_DWELL_MAX	31	/* Maximum dwell time for NFR in msecs */
#define IEEE80211_NFR_DBVC_DWELL_DEF	24	/* Default dwell time for NFR in msecs */
/** @} */

enum ieee80211_fdr_status {
	IEEE80211_FDR_STATE_DISABLED = 0,
	IEEE80211_FDR_STATE_ENABLED,
	IEEE80211_FDR_STATE_ACTIVE,
	IEEE80211_FDR_STATE_STANDBY,
	IEEE80211_FDR_STATE_MAX,
};

enum ieee80211_fdr_config_mode {
	IEEE80211_FDR_MODE_NONE = 0,
	IEEE80211_FDR_MODE_SIMPLE,
	IEEE80211_FDR_MODE_SEAMLESS,
	IEEE80211_FDR_MODE_MAX = IEEE80211_FDR_MODE_SEAMLESS
};

/* GPIO PIN config for DBVC */
enum ieee80211_dbvc_antenna_switch {
	/*
	 * disable Antenna config by GPIO (enforced when DBVC is disabled)
	 */
	IEEE80211_DBVC_ANT_SWITCH_DISABLE = 0,
	/*
	 * GPIO fixed low (default when DBVC is enabled)
	 */
	IEEE80211_DBVC_FIXED_LOW,
	/*
	 * GPIO fixed high
	 */
	IEEE80211_DBVC_FIXED_HIGH,
	/*
	 * GPIO is low while in AP mode and high while in STA mode
	 */
	IEEE80211_DBVC_AP_LOW_STA_HIGH,
	/*
	 * GPIO is high while in AP mode and low while in STA mode
	 */
	IEEE80211_DBVC_AP_HIGH_STA_LOW,
	IEEE80211_DBVC_ANT_SWITCH_MAX = IEEE80211_DBVC_AP_HIGH_STA_LOW
};

enum ieee80211_port_isolate_group {
	IEEE80211_PORT_GROUP_NONE = 0,
	IEEE80211_PORT_GROUP_SGMII,
	IEEE80211_PORT_GROUP_RGMII,
	IEEE80211_PORT_GROUP_MAX = IEEE80211_PORT_GROUP_RGMII,
};

#endif /* _NET80211_IEEE80211_H_ */
