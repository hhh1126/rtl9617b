/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2009 - 2018 Quantenna Communications Inc            **
**                                                                           **
**  File        : qcsapi.h                                                   **
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

/**
 * The main QCSAPI header file containing function prototypes, data types etc.
 */

#ifndef _QCSAPI_H
#define _QCSAPI_H

#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <net/ethernet.h>

#include "qcsapi_common.h"
#include <qtn/shared_defs.h>
#include <qtn/wlan_ioctl.h>
#include <qtn/qtn_wmm_ac.h>
#include <qtn/qtn_nis.h>
#include <qtn/qtnis.h>
#include <net80211/_ieee80211.h>

#include <common/ruby_pm.h>
#include <common/base/qtn_base_early_flash.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BIT
#define BIT(x) (1L << (x))
#endif

/*
 * The following #defines come from the TR-098 standard.
 */
#ifndef IEEE80211_CHAN_MAX
#define IEEE80211_CHAN_MAX		255
#endif

#ifndef IW_ESSID_MAX_SIZE
#define IW_ESSID_MAX_SIZE		32
#endif

#define IW_ESSID_STR_MAX_SIZE		(IW_ESSID_MAX_SIZE * 2 + 1)

/* Begin QCSAPI definitions */

#define QCSAPI_WPS_SHORT_PIN_LEN	4
#define QCSAPI_WPS_MAX_PIN_LEN		8

#define QCSAPI_MAX_PARAM_NAME_LEN	24
#define QCSAPI_MAX_PARAM_VAL_LEN	600	/* for long values parameters - use malloc */
#define QCSAPI_SHORT_PARAM_VAL_LEN	128	/* suitable for most config parameters */

#define QCSAPI_MIN_LENGTH_REGULATORY_REGION	10
#define QCSAPI_MAX_BITRATE_STR_MIN_LEN		5

#define QCSAPI_QDRV_NUM_RF_STREAMS		8
#define QCSAPI_QDRV_NUM_RF_STREAMS_PLUS1	9

#define QCSAPI_RPC_CALLBACK_UNSET		0xFFFFFFFF

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#endif

#define REGULATORY_DB_BIN_NOT_SUPPORT		-255

#define GET_PAIRING_ID			0
#define SET_PAIRING_ID			1

#define QCSAPI_SCRIPT_LOG		"/tmp/run_script.log"
#define QCSAPI_CMD_BUFSIZE		256
#define QCSAPI_MSG_BUFSIZE		1024

#define QCSAPI_SYSTEM_STATUS_FILE	"/tmp/qtn_sys_status"

#define QCSAPI_PRIMARY_WIFI_IFNAME	"wifi0"

#define QCSAPI_CUSTOM_DIR		"/etc/custom/"
#define QCSAPI_FILESYSTEM_SP		".."
#define QCSAPI_CUSTOM_VALUE_MAX_LEN	128

#define QCSAPI_RESTORE_FG_IP		(1 << 0)
#define QCSAPI_RESTORE_FG_NOREBOOT	(1 << 1)
#define QCSAPI_RESTORE_FG_AP		1
#define QCSAPI_RESTORE_FG_STA		2
#define QCSAPI_RESTORE_FG_SEC_DAEMON	0x00000010
#define QCSAPI_RESTORE_FG_MODE_OFFSET	8
#define QCSAPI_RESTORE_FG_MODE_BITS	4
#define QCSAPI_RESTORE_FG_MODE_MASK	(0xF)

#define LOCAL_GET_PER_SSID_CONFIG_SCRIPT		"/scripts/get_per_ssid_config"
#define LOCAL_UPDATE_PER_SSID_CONFIG_SCRIPT		"/scripts/update_per_ssid_config"
#define QCSAPI_SEC_CMD					"/scripts/sec_cmd"
#define QCSAPI_SEC_AGENT_KEY_LEN_MAX			64

/* 3GPP Cellular Network */
#define IEEE80211U_MCC_LEN		3	/* MCC - Mobile Country Code */
#define IEEE80211U_MNC_LEN_MAX		3	/* MNC - Mobile Network Code */
#define IEEE80211U_MNC_LEN_MIN		2
#define IEEE80211U_3GPP_CELL_NET_MAX	40

#define HS20_MAX_NAI_REALM_LEN			255
#define HS20_MAX_ROAMING_CONSORTIUM_LEN		30	/* 30 characters or 15 octet hexadecimal */
#define HS20_MIN_ROAMING_CONSORTIUM_LEN		6	/* 6 characters or 3 octet hexadecimal */
#define ISO639_LANG_CODE_LEN_MAX		3
#define IEEE80211U_VENUE_NAME_LEN_MAX		252
#define HS20_OPER_FRIENDLY_NAME_LEN_MAX		252
#define QCSAPI_QTM_CFG_INVALID	(-1)

#define QCSAPI_RSSI_OR_SNR_FACTOR	10
#define QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE	5
#define QCSAPI_RSSI_ROUND(_rssi) (((_rssi) < 0) ? \
	(((_rssi) - QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) / QCSAPI_RSSI_OR_SNR_FACTOR) : \
	(((_rssi) + QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) / QCSAPI_RSSI_OR_SNR_FACTOR))

#define QCSAPI_BRCTL_IGMP_SNOOPING	"brctl igmpsnoop"
#define QCSAPI_IGMP_SNOOPING_FILE_PATH	"/sys/class/net/"
#define QCSAPI_IGMP_SNOOPING_STATE	"/bridge/igmp_snoop_state"
#define QCSAPI_IGMP_SNOOPING_DISABLE	0
#define QCSAPI_IGMP_SNOOPING_ENABLE	1

#define QCSAPI_RATESET_STRING_LEN	256

#define BSA_CONF_FILE			"/mnt/jffs2/bsa_conf.txt"
#define TEMP_BSA_CONF_FILE		"/mnt/jffs2/bsa_tmp_conf.txt"

#define BSA_MB_DOMAIN			"mobility_domain"
#define BSA_PAIRS_NUM_MAX		8
#define BSA_MB_DOMAIN_ALL		0xFFFF

#define QTN_CHECK_AP_MODE	0x1
#define QTN_CHECK_PRIMARY	0x2
#define QTN_CHECK_STA_MODE	0x4

#define MAX_BOOT_CAC_DURATION	2048

/**
 * @addtogroup APIOverview
 * @{
 */

/**
 * \fn int call_qcsapi_example(const char *ifname, int example_input,
 *				int *example_output);
 *
 * \brief A brief description of the function call is provided inline with
 * the function listing for each section.
 *
 * In the Function Documentation section, for each API call, there is a
 * detailed definition of the function call, with extra information, side
 * effects, pre-requisites etc. This appears in the section which documents
 * the individual API call.
 *
 * After the detailed documentation the list of input/output parameters is
 * given.
 *
 * \param ifname Details of the parameter 'ifname'
 * \param example_input Details of the input parameter 'example_input'
 * \param example_output Details of the output parameter 'example_output'.
 * Output parameters are generally seen as pointers to variables.
 *
 * After the parameter list, the return values are documented.
 *
 * \return Details of the return value (generally \>= 0 indicates success,
 * \< 0 indicates failure). See the section @ref mysection4_1_4 "QCSAPI return values"
 * for details of the different return values.
 *
 * In addition to this, there may be extra documentation and references to
 * other function calls.
 *
 * \note Something noteworthy about the API may be documented in one of these
 * blocks.
 *
 * \warning Something that the reader <b>must</b> read and take into account
 * may be documented in one of these blocks.
 *
 * \callqcsapi
 *
 * This is where the command line <c>call_qcsapi</c> interface is detailed.
 * Input parameters, and expected output will be given.
 *
 * \note Not all QCSAPI C API calls have equivalent <c>call_qcsapi</c> command
 * line calls.
 */

/* NOTE: present to show up example documentation for QCSAPI doco. */
extern int call_qcsapi_example(const char *ifname, int example_input,
				int *example_output);
/** @} */

/**
 * @addtogroup DetailedDataTypes
 * @{
 */

/*
 * GPIO / LED PIN numbers
 *
 * LEDs are a subset of GPIO PINs
 */

/**
 * \brief This enumeration represents an abstract LED value.
 *
 * This enumeration represents an abstract LED value.
 */
enum qcsapi_led
{
	qcsapi_AGPIO1_LED = 1,
	qcsapi_AGPIO2_LED = 2,
	qcsapi_AGPIO3_LED = 3,
	qcsapi_AGPIO4_LED = 4,
	qcsapi_AGPIO5_LED = 5,
	qcsapi_AGPIO7_LED = 7,
	qcsapi_AGPIO11_LED = 11,
	qcsapi_AGPIO12_LED = 12,
	qcsapi_AGPIO27_LED = 27,
	qcsapi_nosuch_GPIO = 255,

	QCSAPI_MAX_LED = 31
};

/**
 * \brief This enumeration represents a set of security and authentication
 * modes.
 *
 * This enumeration represents a set of security and authentication modes.
 *
 * The security mode consists of an authentication method (eg, WPA, WPA2,
 * EAP, etc.) and an encryption method (eg, WEP, TKIP, CCMP). These are
 * represented in this enumeration.
 *
 * See @ref APSTADualFunctions "Authentication protocols and encrypyion" for
 * details of the difference between authentication and encryption.
 */
enum qcsapi_auth_crypto
{
	/**
	 * This value represents WPA v1 authentication mode.
	 */
	qcsapi_protocol_WPA_mask = 1,
	/**
	 * This value represents WPA v2 authentication mode.
	 */
	qcsapi_protocol_11i_mask = 2,
	/**
	 * This value represents WPA3 authentication mode.
	 */
	qcsapi_protocol_WPA3_mask = 4,
	/**
	 * This value represents preshared key authentication.
	 */
	qcsapi_ap_PSK_authentication = 1,
	/**
	 * This value represents EAP authentication.
	 */
	qcsapi_ap_EAP_authentication = 2,
	/**
	 * This value represents SAE authentication.
	 */
	qcsapi_ap_SAE_authentication = 3,
	/**
	 * This value represents OWE authentication.
	 */
	qcsapi_ap_OWE_authentication = 4,
	/**
	 * This value represents DPP authentication.
	 */
	qcsapi_ap_DPP_authentication = 5,
	/**
	 * Thie value represents use of the TKIP cipher for encryption.
	 */
	qcsapi_ap_TKIP_encryption_mask = 0x01,
	/**
	 * Thie value represents use of the CCMP cipher for encryption.
	 */
	qcsapi_ap_CCMP_encryption_mask = 0x02,
};

/**
 * \brief This enumeration is used to represent GPIO state.
 *
 * This enumeration is used to represent GPIO state.
 */
typedef enum
{
	/**
	 * This value indicates that the GPIO isn't available for some reason.
	 */
	qcsapi_gpio_not_available = 0,
	/**
	 * Thie value indicates that the GPIO is set to input only mode.
	 */
	qcsapi_gpio_input_only,
	/**
	 * Thie value indicates that the GPIO is set to output only.
	 */
	qcsapi_gpio_output,
	/**
	 * This is the invalid value - representing that a GPIO is not present
	 * on the platform.
	 */
	qcsapi_nosuch_gpio_config = -1
} qcsapi_gpio_config;

/**
 * \brief This enumeration is used to abstract configuration file paths.
 *
 * This enumeration is used to abstract configuration file paths.
 */
typedef enum
{
	/**
	 * This value is used to represent the security config file path.
	 */
	qcsapi_security_configuration_path = 0,
	/**
	 * Placeholder - invalid value.
	 */
	qcsapi_nosuch_file_path = -1
} qcsapi_file_path_config;

/**
 * \brief The operational WiFi mode of a device.
 *
 * The operational WiFi mode of a device.
 */
typedef enum {
	/** The WiFi mode has not yet been configured. */
	qcsapi_mode_not_defined = 1,

	/** The device is operating as an AP. */
	qcsapi_access_point,

	/** The device is operating as a STA. */
	qcsapi_station,

	/** The device is operating in Wireless Distribution Mode (WDS), or bridged mode. */
	qcsapi_wds,

	/**
	 * The device is operating in repeater mode. The primary interface is in STA mode and
	 * secondary interfaces are in AP mode.
	 */
	qcsapi_repeater,

	/** The device is operating in ad-hoc mode (not supported). */
	qcsapi_adhoc,

	/** Invalid mode. */
	qcsapi_nosuch_mode = 0
} qcsapi_wifi_mode;

/**
 * \brief Enumeration to represent rate sets.
 *
 * Enumeration to represent different rate sets as used in the system.
 */
typedef enum {
	/**
	 * The set of basic rates which must be supported by all clients.
	 */
	qcsapi_basic_rates = 1,
	/**
	 * The set of actual rates in use.
	 */
	qcsapi_operational_rates,
	/**
	 * The set of all supported rates on the device.
	 */
	qcsapi_possible_rates,
	/**
	 * Placeholder - invalid.
	 */
	qcsapi_nosuch_rate = 0
} qcsapi_rate_type;

/**
 * \brief Enumeration to represent 802.11 standards
 */
typedef enum {
	/**
	 * 802.11n
	 */
	qcsapi_mimo_ht = 1,
	/**
	 * 802.11ac
	 */
	qcsapi_mimo_vht,
	/**
	 * 802.11ax
	 */
	qcsapi_mimo_he,
	/**
	 * Placeholder - invalid.
	 */
	qcsapi_nosuch_standard = 0
} qcsapi_mimo_type;

/**
 * \brief Enumeration used to represent different interface counters.
 *
 * \sa qcsapi_interface_get_counter
 * \sa qcsapi_interface_get_counter64
 * \sa qcsapi_pm_get_counter
 * \sa qcsapi_wifi_get_node_counter
 */
typedef enum {
	qcsapi_nosuch_counter = 0,
	QCSAPI_NOSUCH_COUNTER = qcsapi_nosuch_counter,
	qcsapi_total_bytes_sent = 1,
	QCSAPI_TOTAL_BYTES_SENT = qcsapi_total_bytes_sent,
	qcsapi_total_bytes_received,
	QCSAPI_TOTAL_BYTES_RECEIVED = qcsapi_total_bytes_received,
	qcsapi_total_packets_sent,
	QCSAPI_TOTAL_PACKETS_SENT = qcsapi_total_packets_sent,
	qcsapi_total_packets_received,
	QCSAPI_TOTAL_PACKETS_RECEIVED = qcsapi_total_packets_received,
	qcsapi_discard_packets_sent,
	QCSAPI_DISCARD_PACKETS_SENT = qcsapi_discard_packets_sent,
	qcsapi_discard_packets_received,
	QCSAPI_DISCARD_PACKETS_RECEIVED = qcsapi_discard_packets_received,
	qcsapi_error_packets_sent,
	QCSAPI_ERROR_PACKETS_SENT = qcsapi_error_packets_sent,
	qcsapi_error_packets_received,
	QCSAPI_ERROR_PACKETS_RECEIVED = qcsapi_error_packets_received,
	qcsapi_fragment_frames_received,
	QCSAPI_FRAGMENT_FRAMES_RECEIVED = qcsapi_fragment_frames_received,
	qcsapi_vlan_frames_received,
	QCSAPI_VLAN_FRAMES_RECEIVED = qcsapi_vlan_frames_received,
} qcsapi_counter_type;

/**
 * \brief Enumeration for parameters as read in via qcsapi_wifi_get_node_param.
 *
 * Enumeration for parameters as read in via qcsapi_wifi_get_node_param.
 * Quoted strings are the corresponding parameter names to be used from call_qcsapi.
 *
 * \sa qcsapi_wifi_get_node_param
 */
typedef enum {
	QCSAPI_NO_SUCH_PER_ASSOC_PARAM = 0,

	/** Link quality "link_quality" */
	QCSAPI_LINK_QUALITY = 1,

	/** RSSI in dBm "rssi_dbm" */
	QCSAPI_RSSI_DBM,

	/** Bandwidth "bw" */
	QCSAPI_BANDWIDTH,

	/** Signal-to-noise ratio "snr" */
	QCSAPI_SNR,

	/** TX PHY rate "tx_phy_rate" */
	QCSAPI_TX_PHY_RATE,

	/** RX PHY rate "rx_phy_rate" */
	QCSAPI_RX_PHY_RATE,

	/** Standard CCA request "stand_cca_req" */
	QCSAPI_STAD_CCA,

	/** Per association hardware noise "pa_hw_noise" */
	QCSAPI_HW_NOISE,

	/** Bridge IP address "br_ip"*/
	QCSAPI_STA_IP,

	/** RSSI "rssi" */
	QCSAPI_RSSI,

	/** Hardware noise "hw_noise" */
	QCSAPI_PHY_NOISE,

	/** MAC address "soc_macaddr" */
	QCSAPI_SOC_MAC_ADDR,

	/** IP address "soc_ipaddr" */
	QCSAPI_SOC_IP_ADDR,

	/** Basic measurement "basic" */
	QCSAPI_NODE_MEAS_BASIC,

	/** CCA measurement "cca" */
	QCSAPI_NODE_MEAS_CCA,

	/** RPI measurement "rpi" */
	QCSAPI_NODE_MEAS_RPI,

	/** Channel load measurement "channel_load" */
	QCSAPI_NODE_MEAS_CHAN_LOAD,

	/** Noise histogram measurement "noise_histogram" */
	QCSAPI_NODE_MEAS_NOISE_HIS,

	/** Beacon measurement "beacon" */
	QCSAPI_NODE_MEAS_BEACON,

	/** Frame measurement "frame" */
	QCSAPI_NODE_MEAS_FRAME,

	/** Transmit stream category measurement "tran_stream_cat" */
	QCSAPI_NODE_MEAS_TRAN_STREAM_CAT,

	/** Multicast diagnostic measurement "multicast_diag" */
	QCSAPI_NODE_MEAS_MULTICAST_DIAG,

	/** Transmit Power Control (TPC) report "tpc_report" */
	QCSAPI_NODE_TPC_REP,

	/** Link measurement "link_measure" */
	QCSAPI_NODE_LINK_MEASURE,

	/** Neighbor report "neighbor_report" */
	QCSAPI_NODE_NEIGHBOR_REP,

	/** Short GI Capability "sgi_caps" */
	QCSAPI_NODE_SGI_CAPS,
} qcsapi_per_assoc_param;

/**
 * \brief This enumeration represents the throttling rule for the QoS premier class.
 *
 * This enumeration represents the throttling rule for the QoS premier family.
 */
typedef enum {
	/**
	 * No throttling rule.
	 */
	qcsapi_qos_premier_rule_none = 0,
	/**
	 * Throttling according to airtime fairness.
	 */
	qcsapi_qos_premier_rule_atf = 1,
	/**
	 * Invalid rule. Placeholder.
	 */
	qcsapi_qos_premier_rule_num,
	qcsapi_qos_premier_nosuch_rule = qcsapi_qos_premier_rule_num
} qcsapi_qos_premier_rule;

#define QCSAPI_LOCAL_NODE	0
#define QCSAPI_REMOTE_NODE	1

/* This enum lists booleans (yes / no) */
#define QCSAPI_TRUE	1
#define QCSAPI_FALSE	0

/* Node SGI Caps bit definitions */
/**
 * \brief Bit offsets in the \ref sgi_caps field, for Short Guard Interval (SGI) capabilities
 *
 * Bit offsets in the \ref sgi_caps field, for Short Guard Interval (SGI) capabilities
 */
enum qcsapi_node_sgi_caps_bits {
	/** SGI capable in 20MHz */
	QCSAPI_NODE_SGI_CAPS_BIT_FOR_20MHZ = 0,

	/** SGI capable in 40MHz */
	QCSAPI_NODE_SGI_CAPS_BIT_FOR_40MHZ = 1,

	/** SGI capable in 80MHz */
	QCSAPI_NODE_SGI_CAPS_BIT_FOR_80MHZ = 2,

	/** SGI capable in 160MHz */
	QCSAPI_NODE_SGI_CAPS_BIT_FOR_160MHZ = 3
};

/**
 * \brief Bit offsets in the \ref ap_flags field, for various AP capabilities.
 *
 * Bit offsets in the \ref ap_flags field, for various AP capabilities.
 */
enum qcsapi_ap_flag_bits {
	/** Security is enabled */
	QCSAPI_AP_FLAG_BIT_SEC_ENABLE = 0,

	/** SGI capability in 20MHz */
	QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ = 1,

	/** SGI capability in 40MHz */
	QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ = 2,

	/** SGI capability in 80MHz */
	QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ = 3,

	/** SGI capability in 1600MHz */
	QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ = 4
};

/**
 * \brief Bit offsets in the \ref ap_wps field, for WPS capabilities.
 *
 * Bit offsets in the \ref ap_wps field, for WPS capabilities.
 */
enum qcsapi_wps_flag_bits {
	/** WPS is enabled */
	QCSAPI_WPS_FLAG_BIT_ENABLE = 0,

	/** WPS selected registrar is set */
	QCSAPI_WPS_FLAG_BIT_REGISTRAR = 1,

	/** WPS push button supported */
	QCSAPI_WPS_FLAG_BIT_PUSH_BUTTON = 2,

	/** WPS push button active */
	QCSAPI_WPS_FLAG_BIT_PUSH_BUTTON_ACTIVE = 3,
};

/**
 * \brief Enumeration used in the option set/get API.
 *
 * \sa qcsapi_wifi_get_option
 * \sa qcsapi_wifi_set_option
 */
typedef enum {
	qcsapi_channel_refresh = 1,		/* 2.4 GHz only */
	qcsapi_DFS,				/* 5 GHz only */
	qcsapi_wmm,				/* wireless multimedia extensions */
	qcsapi_mac_address_control,
	qcsapi_beacon_advertise,
	qcsapi_wifi_radio,
	qcsapi_autorate_fallback,
	qcsapi_security,
	qcsapi_SSID_broadcast,
	qcsapi_802_11d,
	qcsapi_wireless_isolation,
	qcsapi_short_GI,
	qcsapi_802_11h,
	qcsapi_dfs_fast_channel_switch,
	qcsapi_dfs_avoid_dfs_scan,
	qcsapi_uapsd,
	qcsapi_tpc_query,
	qcsapi_sta_dfs,
	qcsapi_specific_scan,
	qcsapi_GI_probing,
	qcsapi_GI_fixed,
	qcsapi_stbc,
	qcsapi_beamforming,
	qcsapi_short_slot,
	qcsapi_short_preamble,
	qcsapi_rts_cts,
	qcsapi_40M_only,
	qcsapi_obss_coexist,
	qcsapi_11g_protection,
	qcsapi_11n_protection,
	qcsapi_qlink,
	qcsapi_allow_11b,
	qcsapi_dyn_beacon_period,
	qcsapi_acs_obss_chk,
	qcsapi_sta_dfs_strict,
	qcsapi_sync_config,
	/**
	 * Enable (1) or disable (0) automatic change from 20MHz bandwidth to 40MHz
	 * after 40 MHz intolerant stations leave the BSS. Default is 1.
	 */
	qcsapi_bw_resume,
	qcsapi_subband_radar = 100,
	qcsapi_priority_repeater,
	qcsapi_nosuch_option = 0
} qcsapi_option_type;

/**
 * \brief Enumeration used in the board parameter get API.
 *
 * \sa qcsapi_get_board_parameter
 */
typedef enum {
	qcsapi_hw_revision = 1,
	qcsapi_hw_id,
	qcsapi_hw_desc,
	qcsapi_rf_chipid,
	qcsapi_bond_opt,
	qcsapi_vht,
	qcsapi_bandwidth,
	qcsapi_spatial_stream,
	qcsapi_interface_types,
	qcsapi_rf_chip_verid,
	qcsapi_name,
	qcsapi_nosuch_parameter = 0
} qcsapi_board_parameter_type;

/**
 * \brief Enumeration used to find the service index
 *
 * \sa qcsapi_service_name
 */
typedef enum {
	QCSAPI_SERVICE_MAUI = 0,
	QCSAPI_SERVICE_TELNET = 1,
	QCSAPI_SERVICE_DHCP_CLIENT = 2,
	QCSAPI_SERVICE_HTTPD = 3,
	QCSAPI_SERVICE_MONITOR_TEMPERATURE = 4,
	QCSAPI_SERVICE_BSA = 5,
	QCSAPI_NOSUCH_SERVICE = -1,
} qcsapi_service_name;

/**
 * \brief Enumeration used to map start_index in /etc/init.d/
 *
 * \sa qcsapi_service_start_index
 */
typedef enum {
	qcsapi_service_maui_start_index = 90,
	qcsapi_service_inetd_start_index = 42,
	qcsapi_service_dhclient_start_index = 91,
	qcsapi_service_httpd_start_index = 92,
	qcsapi_service_monitor_temp_start_index = 70,
	qcsapi_service_bsa_start_index = 81,
	qcsapi_service_no_such_index = -1,
} qcsapi_service_start_index;

/**
 * \brief Enumeration used to find the service action
 *
 * \sa qcsapi_service_action
 */
typedef enum {
	QCSAPI_SERVICE_START = 0,
	QCSAPI_SERVICE_STOP = 1,
	QCSAPI_SERVICE_ENABLE = 2,
	QCSAPI_SERVICE_DISABLE = 3,
	QCSAPI_NOSUCH_ACTION = -1,
} qcsapi_service_action;

/**
 * Maximum number of bandwidths + 1
 */
#define MAX_NUM_OF_BANDWIDTHS	5

/**
 * \brief This enumeration represents the bandwidth in use on the device.
 *
 * This enumeration represents the bandwidth in use on the device.
 */
typedef enum
{
	/**
	 * The default bandwidth of the device
	 */
	qcsapi_bw_default = 0,
	/**
	 * The device is operating in 20MHz mode.
	 */
	qcsapi_bw_20MHz = 20,
	/**
	 * The device is operating in 40MHz mode.
	 */
	qcsapi_bw_40MHz = 40,
	/**
	 * The device is operating in 80MHz mode.
	 */
	qcsapi_bw_80MHz = 80,
	/**
	 * The device is operating in 160MHz mode.
	 */
	qcsapi_bw_160MHz = 160,
	/**
	 * Placeholder - unknown bandwidth (indicates error).
	 */
	qcsapi_nosuch_bw
} qcsapi_bw;

/* Number of per-channel power values in power table for 5GHz table format */
#define QCSAPI_POWER_TOTAL		16

/**
 * \brief List of per-channel Tx power value types for 5GHz power table.
 *
 * This enumeration represents the indices of Tx power values types used for FEM0 only.
 * Refer to qcsapi_pwr_idx* enumerations for full list of all possible power table
 * indexes.
 *
 * \sa qcsapi_wifi_get_chan_power_table
 * \sa qcsapi_wifi_set_chan_power_table
 *
 */
typedef enum
{
	/**
	 * The power index for beamforming off and 1 spatial stream.
	 */
	QCSAPI_POWER_INDEX_BFOFF_1SS = 0,
	/**
	 * The power index for beamforming off and 2 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_2SS,
	/**
	 * The power index for beamforming off and 3 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_3SS,
	/**
	 * The power index for beamforming off and 4 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_4SS,
	/**
	 * The power index for beamforming off and 5 spatial stream.
	 */
	QCSAPI_POWER_INDEX_BFOFF_5SS,
	/**
	 * The power index for beamforming off and 6 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_6SS,
	/**
	 * The power index for beamforming off and 7 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_7SS,
	/**
	 * The power index for beamforming off and 8 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFOFF_8SS,
	/**
	 * The power index for beamforming on and 1 spatial stream.
	 */
	QCSAPI_POWER_INDEX_BFON_1SS,
	/**
	 * The power index for beamforming on and 2 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_2SS,
	/**
	 * The power index for beamforming on and 3 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_3SS,
	/**
	 * The power index for beamforming on and 4 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_4SS,
	/**
	 * The power index for beamforming on and 5 spatial stream.
	 */
	QCSAPI_POWER_INDEX_BFON_5SS,
	/**
	 * The power index for beamforming on and 6 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_6SS,
	/**
	 * The power index for beamforming on and 7 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_7SS,
	/**
	 * The power index for beamforming on and 8 spatial streams.
	 */
	QCSAPI_POWER_INDEX_BFON_8SS,
} qcsapi_power_indices;

/**
 * \brief Union of per-FEM and per-Primary channel position power indexes.
 *
 * Per-FEM power values are used for 5GHz band channels.
 * Power values for different primary channel position are used for 2.4GHz channels. 2.4GHz
 * channels operate using FEM0 only.
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_regulatory_chan_txpower_set
 * \sa struct qcsapi_chan_tx_powers_info
 *
 */
typedef enum
{
	QCSAPI_PWR_IDX_FEM0_PRIPOS_LOW = 0,
	QCSAPI_PWR_IDX_FEM1_PRIPOS_UPPER = 1,
	QCSAPI_PWR_IDX_FEM_PRIPOS_NUM = 2
} qcsapi_pwr_idx_fem_pripos;

/**
 * \brief Power indices for Beam Forming ON/OFF cases.
 *
 * Power indexes for different beamforming states.
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_regulatory_chan_txpower_set
 * \sa struct qcsapi_chan_tx_powers_info
 *
 */
typedef enum
{
	QCSAPI_PWR_IDX_BFON = 0,
	QCSAPI_PWR_IDX_BFOFF = 1,
	QCSAPI_PWR_IDX_BF_NUM = 2
} qcsapi_pwr_idx_bf;

/**
 * \brief Power indexes for different number of Spatial Streams.
 *
 * Power indexes used in configurations with different number of Spatial Streams.
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_regulatory_chan_txpower_set
 * \sa struct qcsapi_chan_tx_powers_info
 *
 */
typedef enum
{
	QCSAPI_PWR_IDX_1SS = 0,
	QCSAPI_PWR_IDX_2SS = 1,
	QCSAPI_PWR_IDX_3SS = 2,
	QCSAPI_PWR_IDX_4SS = 3,
	QCSAPI_PWR_IDX_5SS = 4,
	QCSAPI_PWR_IDX_6SS = 5,
	QCSAPI_PWR_IDX_7SS = 6,
	QCSAPI_PWR_IDX_8SS = 7,
	QCSAPI_PWR_IDX_SS_NUM = 8
} qcsapi_pwr_idx_ss;

/**
 * \brief Power indexes for bandwidth configurations.
 *
 * Power indexes used in configurations with specified channel bandwidth.
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_regulatory_chan_txpower_set
 * \sa struct qcsapi_chan_tx_powers_info
 *
 */
typedef enum
{
	QCSAPI_PWR_BW_20M = 0,
	QCSAPI_PWR_BW_40M = 1,
	QCSAPI_PWR_BW_80M = 2,
	QCSAPI_PWR_BW_160M = 3,
	QCSAPI_PWR_BW_NUM = 4,
} qcsapi_pwr_idx_bw;

/**
 * \brief Possible Tx power value types.
 *
 * List of possible types of Tx power values
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa struct qcsapi_chan_tx_powers_info
 *
 */
typedef enum {
	/**
	 * Tx power value currently used by device. It can be changed
	 * at runtime but must be within regulatory and device limits.
	 */
	QCSAPI_PWR_VALUE_TYPE_ACTIVE,
	/**
	 * Tx power value configured for use when device boots up, if not
	 * changed by user later. The value ensures regulatory compliance and compliance with HW
	 * capabilities of the device. It can not be changed.
	 */
	QCSAPI_PWR_VALUE_TYPE_CONFIGURED,
	/**
	 * Tx power value currently used for dynamic number of TX chains. This option can be used
	 * only when RF is 8x8.
	 */
	QCSAPI_PWR_VALUE_TYPE_DNTX,
	/**
	 * Tx power value configured for dynamic number of TX chains. This option can be used only
	 * when RF is 8x8.
	 */
	QCSAPI_PWR_VALUE_TYPE_DNTX_CONFIGURED,
} qcsapi_txpwr_value_type;

/**
 * \brief This enumeration represents the 802.11w / PMF capability of the VAP.
 *
 * This enumeration represents the 802.11w / PMF capability of the VAP.
 */
typedef enum
{
	/**
	 * The PMF capability is disbled.
	 */
	qcsapi_pmf_disabled = 0,
	/**
	 * The PMF capability is optional.
	 */
	qcsapi_pmf_optional = 1,
	/**
	 * The PMF capability is required.
	 */
	qcsapi_pmf_required = 2

} qcsapi_pmf;

/**
 * \brief This enumeration represents the mode in use on the device.
 *
 * This enumeration represents the bandwidth in use on the device.
 *
 * This enumeration is used to set the correct bandwidth.
 */

typedef enum
{
	qcsapi_11nac_disable = 0,
	qcsapi_11nac_enable = 1
} qcsapi_11nac_stat;

/**
 * \brief This enumeration represents the type of channel list.
 *
 * This enumeration is used to represent the type of channel list used in various APIs.
 */
enum qcsapi_chlist_flags {
	/**
	 * List of available channels.
	 */
	qcsapi_chlist_flag_available	= 0x00000001,
	/**
	 * List of disabled channels.
	 */
	qcsapi_chlist_flag_disabled	= 0x00000002,
	/**
	 * List of scan channels.
	 */
	qcsapi_chlist_flag_scan		= 0x00000004,
	/**
	 * List of active channels.
	 */
	qcsapi_chlist_flag_active	= 0x00000008,
	/**
	 * List of OCAC off-channels.
	 */
	qcsapi_chlist_flag_ocac_off	= 0x00000010,
};

/**
 * \brief This enumeration represents frequency bands.
 *
 * This enumeration is used to represent band specific information in various APIs
 */
enum qcsapi_freq_band {
	qcsapi_freq_band_default	= 0, /* 2.4GHz or 5GHz */
	qcsapi_freq_band_2pt4_ghz	= 1,
	qcsapi_freq_band_4_ghz		= 2,
	qcsapi_freq_band_5_ghz		= 3,
	qcsapi_freq_band_6_ghz		= 4,
	qcsapi_freq_band_unknown	= 0xFF
};

/**
 * \brief This enumeration represents the preferred band to use on the device.
 *
 * This enumeration is used to set the preferred bandwidth.
 */
typedef enum
{
	qcsapi_band_2_4ghz = 0,
	qcsapi_band_5ghz = 1,
	qcsapi_nosuch_band = 2
}qcsapi_pref_band;

/**
 * \brief This enumeration represents the current RF Chip ID.
 *
 * This enumeration is used to get the current RF Chip ID.
 */
typedef enum
{
	qcsapi_rf_chipid_2_4ghz		= CHIPID_2_4_GHZ,
	qcsapi_rf_chipid_5ghz		= CHIPID_5_GHZ,
	qcsapi_rf_chipid_dual		= CHIPID_DUAL,
	qcsapi_rf_chipid_5ghz_6ghz	= CHIPID_5GHZ_6GHZ,
	qcsapi_nosuch_chipid		= CHIPID_INVALID
}qcsapi_rf_chip_id;

/**
 * \brief This enumeration represents the current Phy Mode.
 *
 * This enumeration is used to get the current RF Phy Mode.
 */
typedef enum
{
	qcsapi_phy_mode_11b,
	qcsapi_phy_mode_11g,
	qcsapi_phy_mode_11ng,
	qcsapi_phy_mode_11ng40,
	qcsapi_phy_mode_11na,
	qcsapi_phy_mode_11na40,
	qcsapi_phy_mode_11nonly,
	qcsapi_phy_mode_11nonly40,
	qcsapi_phy_mode_11acng,
	qcsapi_phy_mode_11acng40,

	qcsapi_phy_mode_11a,
	qcsapi_phy_mode_11ac20,
	qcsapi_phy_mode_11ac40,
	qcsapi_phy_mode_11ac80,
	qcsapi_phy_mode_11ac80Edgeplus,
	qcsapi_phy_mode_11ac80Cntrplus,
	qcsapi_phy_mode_11ac80Cntrminus,
	qcsapi_phy_mode_11ac80Edgeminus,
	qcsapi_phy_mode_11aconly,
	qcsapi_phy_mode_11aconly40,
	qcsapi_phy_mode_11aconly80,
	qcsapi_phy_mode_11ac160,
	qcsapi_phy_mode_11naconly,
	qcsapi_phy_mode_11naconly40,
	qcsapi_phy_mode_11naconly80,
	qcsapi_phy_mode_11nonly2g,
	qcsapi_phy_mode_11nonly402g,

	qcsapi_phy_mode_11axng,
	qcsapi_phy_mode_11axng40,
	qcsapi_phy_mode_11ax20,
	qcsapi_phy_mode_11ax40,
	qcsapi_phy_mode_11ax80,
	qcsapi_phy_mode_11ax80Edgeplus,
	qcsapi_phy_mode_11ax80Cntrplus,
	qcsapi_phy_mode_11ax80Cntrminus,
	qcsapi_phy_mode_11ax80Edgeminus,
	qcsapi_phy_mode_11ax160,

	qcsapi_phy_mode_invalid
} qcsapi_phy_mode;

/**
 * \brief This enumeration represents the vco lock detect mode status.
 *
 * This enumeration is used to set/get vco lock detect mode stats.
 */

typedef enum
{
	qcsapi_vco_lock_detect_mode_disable = 0,
	qcsapi_vco_lock_detect_mode_enable = 1
} qcsapi_vco_lock_detect_mode_stat;

/**
 * \brief This enumeration represents the state of the MAC address filtering.
 *
 * This enumeration represents the state of the MAC address filtering.
 *
 * MAC address filtering can be inclusive, exclusive or disabled.
 */
typedef enum
{
	/**
	 * MAC address filtering is fully disabled.
	 */
	qcsapi_disable_mac_address_filtering = 0,
	/**
	 * MAC address inclusive filtering - allow all packets unless explicitly
	 * denied in the filter list.
	 */
	qcsapi_accept_mac_address_unless_denied,
	/**
	 * MAC address exclusive filtering - deny all packets unless explicitly
	 * allowed in the filter list.
	 */
	qcsapi_deny_mac_address_unless_authorized,
	/**
	 * Placeholder - indicates an error.
	 */
	qcsapi_nosuch_mac_address_filtering = -1
} qcsapi_mac_address_filtering;

/**
 * \brief Enumeration to represent AP isolation status.
 *
 */
typedef enum
{
	/**
	 * AP isolation is disabled.
	 * Frames between associated stations in the BSS are passed.
	 */
	qcsapi_ap_isolate_disabled = 0,
	/**
	 * AP isolation is enabled.
	 * Frames between associated stations in the BSS are blocked.
	 */
	qcsapi_ap_isolate_enabled = 1,
	/**
	 * Placeholder - unused.
	 */
	qcsapi_ap_isolation_end
} qcsapi_ap_isolate_type;

/**
 * \brief This enumeration represents the partitions supported for firmware upgrade.
 *
 * This enumeration represents the partitions supported for firmware upgrade.
 *
 * The two partitions used for firmware are the live and safety images. Ideally, the
 * safety image is never touched, and is always present to allow the system to recover
 * to a known good (factory) setting.
 */
typedef enum
{
	/**
	 * This represents the live image partition - the partition that should be
	 * upgraded.
	 */
	qcsapi_image_linux_live = 0,
	qcsapi_live_image = qcsapi_image_linux_live,
	/**
	 * This represents the safety image partition - this should not be touched.
	 */
	qcsapi_image_linux_safety,
	qcsapi_safety_image = qcsapi_image_linux_safety,
	/**
	 * This represents the live uboot image partition - the partition that should be
	 * upgraded.
	 */
	qcsapi_image_uboot_live,
	/**
	 * This represents the safety uboot image partition - this should not be touched.
	 */
	qcsapi_image_uboot_safety,
	/**
	 * This represents the only u-boot image partition
	 * - upgrade it when it is really necessary
	 */
	qcsapi_image_uboot,
	qcsapi_image_max,
	/**
	 * Placeholder - indicates an error.
	 */
	qcsapi_nosuch_partition = -1
} qcsapi_flash_partiton_type;

/**
 * \brief Enumeration to represent WPS parameters as used by the qcsapi_wps_get_param API.
 *
 * \sa qcsapi_wps_get_param
 */
typedef enum
{
	/**
	 * The WPS device UUID.
	 */
	qcsapi_wps_uuid = 0,
	/**
	 * The OS version the WPS device is running.
	 */
	qcsapi_wps_os_version,
	/**
	 * The device name of the WPS device.
	 */
	qcsapi_wps_device_name,
	/**
	 * The supported configuration methods (eg, PBC, PIN) of the WPS device.
	 */
	qcsapi_wps_config_methods,
	/**
	 * Whether the AP setup is locked or able to be reconfigured by an external
	 * registrar.
	 */
	qcsapi_wps_ap_setup_locked,
	/**
	 * wps vendor for specific action in WPS process
	 */
	qcsapi_wps_vendor_spec,
	/**
	 * The label pin of the ap which is configured in the hostapd.conf
	 */
	qcsapi_wps_ap_pin,
	/**
	 * flag to force broadcast uuid
	 */
	qcsapi_wps_force_broadcast_uuid,
	/**
	 * decide the action after ap pin failure occur
	 */
	qcsapi_wps_ap_pin_fail_method,
	/**
	 * max retry count of ap pin fail in auto_lockdown mode
	 */
	qcsapi_wps_auto_lockdown_max_retry,
	/**
	 * last successful WPS client
	 */
	qcsapi_wps_last_successful_client,
	/**
	 * last successful WPS client device name
	 */
	qcsapi_wps_last_successful_client_devname,
	/**
	 * current ap pin fail number
	 */
	qcsapi_wps_auto_lockdown_fail_num,
	/**
	 * current wps serial number
	 */
	qcsapi_wps_serial_number,
	/**
	 * current wps manufacturer name
	 */
	qcsapi_wps_manufacturer,
	/**
	 * current wps model name
	 */
	qcsapi_wps_model_name,
	/**
	 * current wps model number
	 */
	qcsapi_wps_model_number,
	/**
	 * current wps "pbc in m1" setting (0 or 1)
	 */
	qcsapi_wps_pbc_in_m1,
	/**
	 * The RF band of third party
	 */
	qcsapi_wps_third_party_band,
	/**
	 * The WPS UPnP interface
	 *
	 * \note If the configured interface is not in an up state when the security
	 * daemon starts (e.g. a VLAN interface, which is created after start-up), the
	 * default interface (br0) will be used. The UPnP can be reset to the required
	 * interface after it has been brought up.
	 *
	 */
	qcsapi_upnp_iface,
	/**
	 * Last configuration error of WPS registrar
	 */
	qcsapi_wps_last_config_error,
	/**
	 * Number of entries for WPS registrar
	 */
	qcsapi_wps_registrar_number,
	/**
	 * Number of estabalished WPS registrar
	 */
	qcsapi_wps_registrar_established,
	/**
	 * Placeholder - unused.
	 */
	qcsapi_wps_param_end
} qcsapi_wps_param_type;

/**
 * \brief Enumeration to represent VLAN configuration command as used by the qcsapi_wifi_vlan_config
 * API.
 *
 * \sa qcsapi_wifi_vlan_config
 */
typedef enum {
	/**
	 * Set a VLAN ID for an interface
	 */
	e_qcsapi_vlan_add = 0x00000001,
	/**
	 * Clear a VLAN ID for an interface
	 */
	e_qcsapi_vlan_del = 0x00000002,
	/**
	 * Set the default VLAN ID for an interface
	 */
	e_qcsapi_vlan_pvid = 0x00000004,
	/**
	 * Set the VLAN mode for an interface
	 */
	e_qcsapi_vlan_mode = 0x00000008,
	/**
	 * Set an interface to VLAN Access mode
	 */
	e_qcsapi_vlan_access = 0x00000010,
	/**
	 * Set an interface to VLAN Trunk mode
	 */
	e_qcsapi_vlan_trunk = 0x00000020,
	/**
	 * Set an interface to VLAN Hybrid mode (deprecated - use Trunk mode)
	 */
	e_qcsapi_vlan_hybrid = 0x00000040,
	/**
	 * Set a VLAN tag for a Trunk interface
	 */
	e_qcsapi_vlan_tag = 0x00000100,
	/**
	 * Clear a VLAN tag for a Trunk interface
	 */
	e_qcsapi_vlan_untag = 0x00000200,
	/**
	 * Enable 802.1X dynamic VLAN for an interface
	 */
	e_qcsapi_vlan_dynamic = 0x00000400,
	/**
	 * Disable 802.1X dynamic VLAN for an interface
	 */
	e_qcsapi_vlan_undynamic = 0x00000800,
	/**
	 * Globally enable VLAN functionality
	 */
	e_qcsapi_vlan_enable = 0x00001000,
	/**
	 * Globally disable VLAN functionality
	 */
	e_qcsapi_vlan_disable = 0x00002000,
	/**
	 * Set VLAN configuration for an interface to default values
	 */
	e_qcsapi_vlan_reset = 0x00004000,
	/**
	 * Set to use VLAN priority
	 */
	e_qcsapi_vlan_prio = 0x00008000,
} qcsapi_vlan_cmd;

/**
 * \anchor QCSAPI_GET_SYSTEM_STATUS
 * \brief Bit number to represent system status value
 *
 * \sa qcsapi_get_system_status
 */
typedef enum {
	/**
	 * 1 means ethernet interface is up.
	 * 0 means ethernet interface is down.
	 */
	qcsapi_sys_status_ethernet = 0,
	/**
	 * 1 means pcie module for EP is loaded correctly.
	 * 0 means pcie module for EP is failed to load.
	 */
	qcsapi_sys_status_pcie_ep = 1,
	/**
	 * 1 means pcie module for RC is loaded correctly.
	 * 0 means pcie module for RC is failed to load.
	 */
	qcsapi_sys_status_pcie_rc = 2,
	/**
	 * 1 means wifi module is loaded correctly.
	 * 0 means wifi module is failed to load.
	 */
	qcsapi_sys_status_wifi = 3,
	/**
	 * 1 means Rpcd is ready.
	 * 0 Rpcd is failed to start.
	 */
	qcsapi_sys_status_rpcd = 4,
	/**
	 * 1 means device works in calstate=3 mode.
	 * 0 means device works in calstate=0 mode.
	 */
	qcsapi_sys_status_cal_mode = 30,
	/**
	 * 1 means system boot up completely.
	 * This bit <b>DOES NOT</b> means system can work correctly.
	 * It only indicates that system gets into a stage.
	 */
	qcsapi_sys_status_completed = 31,
} qcsapi_system_status;

/**
 * \brief Enumeration to represent ehternet DSCP operation command
 * as used by the qcsapi_eth_dscp_map API.
 *
 * \sa qcsapi_eth_dscp_map
 */
typedef enum {
	/**
	 * Setting DSCP Priority for all levels in EMAC
	 */
	qcsapi_eth_dscp_fill = 1,

	/**
	 * Setting DSCP Priority for particulat levels in EMAC
	 */
	qcsapi_eth_dscp_poke = 2,

	/**
	 * Getting DSCP Priority for all levels in EMAC
	 */
	qcsapi_eth_dscp_dump = 3,

} qcsapi_eth_dscp_oper;

/**
 * \brief Enumeration to represent EMAC switch connectivity
 *
 * \sa qcsapi_set_emac_switch
 */
typedef enum {
	/**
	 * switch functionality is enabled
	 */
	qcsapi_emac_switch_enable = 0,

	/**
	 * switch functionality is disabled
	 */
	qcsapi_emac_switch_disable = 1,

} qcsapi_emac_switch;

/**
 * \brief Enumeration to control airtime accumulation
 *
 * \sa qcsapi_wifi_node_tx_airtime_accum_control
 * \sa qcsapi_wifi_tx_airtime_accum_control
 */
typedef enum {
	/**
	 * start airtime accumulation
	 */
	qcsapi_accum_airtime_start = 1 << 0,

	/**
	 * stop airtime accumulation
	 */
	qcsapi_accum_airtime_stop = 1 << 1,

}qcsapi_airtime_control;

/**
 * \brief bsa configurable supported parameters
 *
 * \sa qcsapi_bsa_set_parameter
 * \sa qcsapi_bsa_get_parameter
 */

typedef enum {
	/**
	 * 1 means bsa is enabled.
	 * 0 means bsa is disabled.
	 */
	qcsapi_enable = 0,
	/**
	 * Periodicity at which stale entries are removed from the BSA station table.
	 */
	qcsapi_house_keeping_interval,
	/**
	 * Interval for which if STA is not actively considered as stale entry.
	 * Used in housekeeping.h
	 */
	qcsapi_stale_entry_interval,
	/**
	 * MAC address of station to be whitelisted and never considered for steering.
	 */
	qcsapi_whitelist_sta_entry,
	/**
	 * Increase BSA and BPE logging levels
	 */
	qcsapi_bsa_debug_level,
	/**
	 * Control the BSA debug log will send to qevt_server
	 * 0: Disable, 1: Enable
	 */
	qcsapi_bsa_debug_qevt,
	qcsapi_bsa_global_param_end = qcsapi_bsa_debug_qevt,
	/**
	 * Mobile domain identifier, VAPs of 5G and 2.4G with same identifier will be paired
	 */
	qcsapi_mobility_domain,
	/**
	 *Status of SSID pair, 0 = Inactive, 1 = Active
	 */
	qcsapi_ssid_pair_status,
	/**
	 * SSID for which Band steering is required.
	 * ssid and the valid range [1-32]
	 */
	qcsapi_bs_ssid,
	/**
	 * To allow 11ac ONLY STA on 5G radio
	 */
	qcsapi_allow_5g_11ac_only,
	/**
	 * Number of steering attempts before
	 * STA is classified as non-steerable.
	 */
	qcsapi_non_steerable_count,
	/**
	 * Timeout value for blacklisting a
	 * STA on a band
	 */
	qcsapi_blacklist_timeout_val,
	/**
	 * Duration for which phy rate is valid
	 * after a STA de-authenticated
	 */
	qcsapi_inactive_assoc_interval,
	/**
	 * Duration for which STA needs to idle
	 * before considering inactive. Client
	 * that show no activity. Considered
	 * while steering.
	 */
	qcsapi_inactive_runtime_interval,
	/**
	 * Number of tx & rx packets per second below which client is considered
	 * inactive. Objective is to not kick out clients that are currently in the
	 * middle of high activity (ex: video transmission) but are good candidates
	 * for steering based on other parameters.
	 */
	qcsapi_client_inactivity_threshold,
	/**
	 * Periodicity of the BS run-time decisions and metric average interval
	 */
	qcsapi_runtime_decision_interval,
	/* Interval of no steering for same band steering direction, which is N,
	 * the real interval is N * runtime_decision_interval.
	 */
	qcsapi_no_steering_interval,
	/**
	 * FAT below which "loaded" threshold for all parameters are applied.
	 * Used when steering 5GHz to 2.4GHz
	 */
	qcsapi_assoc_5g_loaded_fat,
	/**
	 * FAT above which steering from 2.4Ghz to 5Ghz is considered with
	 * "unloaded" thresholds
	 */
	qcsapi_assoc_5g_unloaded_fat,
	/**
	 * For run time steering, FAT below which "loaded" threshold for all
	 * parameters are applied. Used when steering to 2.4GHz
	 */
	qcsapi_run_5g_loaded_fat,
	/**
	 * For run time steering, FAT above which "unloaded" threshold for all
	 * parameters are applied. Used when steering 5GHz to 2.4GHz
	 */
	qcsapi_run_5g_unloaded_fat,
	/**
	 * Min phy rate required to a STA to be admitted on 5G band when 5GHz
	 * is loaded.
	 */
	qcsapi_assoc_5g_loaded_min_phyrate,
	/**
	 * Min phy rate required to a STA to be admitted on 5G band when 5GHz
	 * is unloaded.
	 */
	qcsapi_assoc_5g_min_phyrate,
	/**
	 * Min phy rate required to a STA to be kept in 5Ghz band when 5GHz loaded.
	 */
	qcsapi_run_5g_loaded_min_phyrate,
	/**
	 * Min phy rate of a STA above which STA is moved to 5Ghz band, either at
	 * assoc time or at run time when 5G is unloaded
	 */
	qcsapi_run_5g_unloaded_min_phyrate,
	/**
	 * Minimum RSSI of STA Probe/Association Requests to be admitted on 5Ghz
	 * when 5GHz is loaded
	 */
	qcsapi_assoc_5g_loaded_min_rssi,
	/**
	 * Minimum RSSI of STA Probe/Association Requests to be admitted on 5Ghz
	 * when 5GHz is unloaded.
	 */
	qcsapi_assoc_5g_min_rssi,
	/**
	 * If set validates the SSID configured on both interface and
	 * bsa_conf file(bs_ssid)
	 */
	qcsapi_ssid_check,
	qcsapi_mb_domain_parameter_end = qcsapi_ssid_check,
	/**
	 * Sends a signal to the BSA App to reload the configurations from the
	 * bsa_conf.txt file
	 */
	qcsapi_bsa_reload_new_config,
	/**
	 * restrart bsa module
	 */
	qcsapi_bsa_module_restart,
	/**
	 * When STA is to be set to not steerable after steered failed, if the
	 * earlier probe request on 2.4G and 5G was received before
	 * (clear_probe_req_interval*house_keeping_interval), it will be
	 * cleared, also dual_band flag will be cleared too.
	 * Clearing will be disabled when clear_dual_band_interval == 0.
	 */
	qcsapi_clear_dual_band_interval,
	/**
	 * Stop deauth BTM capable clients when steering bands with BSA, if configure
	 * deauth_for_steer_btm_capa_sta=0.
	 */
	qcsapi_deauth_for_steer_btm_capa_sta,
	/**
	 * Use fix bw 20M to estimate 5G phyrate if the 2.4G rssi below when steering
	 * from 2.4G to 5G in run time.
	 */
	qcsapi_use_fix_bw_for_5g_phyrate_estimation,
	/**
	 * rssi threshold on 2.4G, decide the bw for 5G phyrate estimation.
	 */
	qcsapi_rssi_threshold_for_5g_phyrate_estimation,
	qcsapi_bsa_action_parameter_end = qcsapi_rssi_threshold_for_5g_phyrate_estimation,

	qcsapi_bsa_param_list_end,

}qcsapi_bsa_conf_param;

/**
 * \brief This enumeration represents the hardware module
 *
 * \sa qcsapi_hw_module
 */

typedef enum {
	/**
	 * Interrupt signal for triggering power management
	 */
	qcsapi_hw_pm_signal = 0,
	qcsapi_hw_no_module = -1,
}qcsapi_hw_module;

/**
 * Debug type
 */
typedef enum {
	QCSAPI_REBOOT_CAUSE = 0,
	QCSAPI_DEBUG_TYPE_MAX,
} qcsapi_debug_type;

/*
 * NOTE: Please note that the value ranges suggested below are experimental
 * and subject to change after further testing.
 */
#define BSA_NON_STEERABLE_CNT_MIN 1
#define BSA_NON_STEERABLE_CNT_MAX 3

#define BSA_BLACKLIST_TIMEOUT_MIN 10 /* seconds */
#define BSA_BLACKLIST_TIMEOUT_MAX 30

#define BSA_HOUSE_KEEP_MIN 600 /* seconds */
#define BSA_HOUSE_KEEP_MAX 1800

#define BSA_STALE_ENTRY_INTERVAL_MIN 300 /* seconds */
#define BSA_STALE_ENTRY_INTERVAL_MAX 600

#define BSA_INACTV_ASSOC_INTERVAL_MIN 0 /* seconds */
#define BSA_INACTV_ASSOC_INTERVAL_MAX 600

#define BSA_INACTV_RUNTIME_INTERVAL_MIN 30 /* seconds */
#define BSA_INACTV_RUNTIME_INTERVAL_MAX 120

#define BSA_CLI_INACTV_THRSHLD_MIN 0 /* packets */
#define BSA_CLI_INACTV_THRSHLD_MAX 50

#define BSA_RUNTIME_DECISION_INTV_MIN 600 /* seconds */
#define BSA_RUNTIME_DECISION_INTV_MAX 1800

#define BSA_NO_STEERING_INTV_MIN 1
#define BSA_NO_STEERING_INTV_MAX 10

#define BSA_ASSOC_5G_LOAD_FAT_MIN 0
#define BSA_ASSOC_5G_LOAD_FAT_MAX 1000

#define BSA_ASSOC_5G_UNLOAD_FAT_MIN 0
#define BSA_ASSOC_5G_UNLOAD_FAT_MAX 1000

#define BSA_RUN_5G_LOAD_FAT_MIN 0
#define BSA_RUN_5G_LOAD_FAT_MAX 1000

#define BSA_RUN_5G_UNLOAD_FAT_MIN 0
#define BSA_RUN_5G_UNLOAD_FAT_MAX 1000

#define BSA_ASSOC_5G_LOAD_PHYRATE_MIN 30 /* mbps */
#define BSA_ASSOC_5G_LOAD_PHYRATE_MAX 2600

#define BSA_ASSOC_5G_PHYRATE_MIN 30 /* mbps */
#define BSA_ASSOC_5G_PHYRATE_MAX 2600

#define BSA_RUN_5G_LOAD_PHYRATE_MIN 30 /* mbps */
#define BSA_RUN_5G_LOAD_PHYRATE_MAX 2600

#define BSA_RUN_5G_UNLOAD_PHYRATE_MIN 30 /* mbps */
#define BSA_RUN_5G_UNLOAD_PHYRATE_MAX 2600

#define BSA_ASSOC_5G_LOAD_RSSI_MIN (-100) /* dBm */
#define BSA_ASSOC_5G_LOAD_RSSI_MAX (-55)

#define BSA_ASSOC_5G_RSSI_MIN (-100) /* dBm */
#define BSA_ASSOC_5G_RSSI_MAX (-55)

#define BSA_CLEAR_DUAL_BAND_MIN 0
#define BSA_CLEAR_DUAL_BAND_MAX 10

#define BSA_RSSI_TH_FOR_5G_PHYRATE_ESTIMATION_MIN (-100)
#define BSA_RSSI_TH_FOR_5G_PHYRATE_ESTIMATION_MAX (-65)
#define BSA_RSSI_TH_FOR_5G_PHYRATE_ESTIMATION_DEF (-80)

#define BSA_DEBUG_LEVEL_MAX	3

#define QCSAPI_BSA_CONF_PARAM(_name)	{qcsapi_ ## _name, #_name}

#define QCSAPI_11R_R0_KEY_MIN_LIFETIME	60

typedef struct qcsapi_bsa_conf_table
{
	qcsapi_bsa_conf_param	bsa_enum;
	const char		*bsa_name;
} qcsapi_bsa_conf_table;

#define QCSAPI_BSA_CONF_PARAMS					\
	QCSAPI_BSA_CONF_PARAM(enable),				\
	QCSAPI_BSA_CONF_PARAM(house_keeping_interval),		\
	QCSAPI_BSA_CONF_PARAM(stale_entry_interval),		\
	QCSAPI_BSA_CONF_PARAM(whitelist_sta_entry),		\
	QCSAPI_BSA_CONF_PARAM(bsa_debug_level),			\
	QCSAPI_BSA_CONF_PARAM(bsa_debug_qevt),			\
	QCSAPI_BSA_CONF_PARAM(mobility_domain),			\
	QCSAPI_BSA_CONF_PARAM(ssid_pair_status),		\
	QCSAPI_BSA_CONF_PARAM(bs_ssid),				\
	QCSAPI_BSA_CONF_PARAM(allow_5g_11ac_only),		\
	QCSAPI_BSA_CONF_PARAM(non_steerable_count),		\
	QCSAPI_BSA_CONF_PARAM(blacklist_timeout_val),		\
	QCSAPI_BSA_CONF_PARAM(inactive_assoc_interval),		\
	QCSAPI_BSA_CONF_PARAM(inactive_runtime_interval),	\
	QCSAPI_BSA_CONF_PARAM(client_inactivity_threshold),	\
	QCSAPI_BSA_CONF_PARAM(runtime_decision_interval),	\
	QCSAPI_BSA_CONF_PARAM(no_steering_interval),	\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_loaded_fat),		\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_unloaded_fat),		\
	QCSAPI_BSA_CONF_PARAM(run_5g_loaded_fat),		\
	QCSAPI_BSA_CONF_PARAM(run_5g_unloaded_fat),		\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_loaded_min_phyrate),	\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_min_phyrate),		\
	QCSAPI_BSA_CONF_PARAM(run_5g_loaded_min_phyrate),	\
	QCSAPI_BSA_CONF_PARAM(run_5g_unloaded_min_phyrate),	\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_loaded_min_rssi),	\
	QCSAPI_BSA_CONF_PARAM(assoc_5g_min_rssi),		\
	QCSAPI_BSA_CONF_PARAM(ssid_check),			\
	QCSAPI_BSA_CONF_PARAM(bsa_reload_new_config),		\
	QCSAPI_BSA_CONF_PARAM(bsa_module_restart),		\
	QCSAPI_BSA_CONF_PARAM(clear_dual_band_interval),	\
	QCSAPI_BSA_CONF_PARAM(deauth_for_steer_btm_capa_sta),	\
	QCSAPI_BSA_CONF_PARAM(use_fix_bw_for_5g_phyrate_estimation),	\
	QCSAPI_BSA_CONF_PARAM(rssi_threshold_for_5g_phyrate_estimation),	\
	{ qcsapi_bsa_param_list_end, NULL },			\

typedef enum {
	/**
	 * Control the acs check OBSS secondary channel
	 * 0: Disable, 1: Enable
	 */
	qcsapi_acs_param_obss_chk = 0,
	qcsapi_acs_param_invalid
}qcsapi_acs_param_type;

/**
 * \brief Enumeration used in the WiFi parameter set/get API.
 *
 * \sa qcsapi_wifi_get_parameter
 * \sa qcsapi_wifi_set_parameter
 */
typedef enum {
	/**
	 * Invalid parameter
	 */
	qcsapi_wifi_nosuch_parameter = 0,

	/**
	 * DTIM period
	 */
	qcsapi_wifi_param_dtim_period = 1,

	/**
	 * CSA (Channel Switch Announcement) count
	 */
	qcsapi_wifi_param_csa_count = 2,

	/**
	 * CSA (Channel Switch Announcement) mode
	 */
	qcsapi_wifi_param_csa_mode = 3,

	/**
	 * RRM (Radio Resource Management) Neighbour Reports,
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_rrm_nr = 4,

	/**
	 * Enable radio transmission
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_tx_enable = 5,

	/**
	 * NSM idle timer value
	 */
	qcsapi_wifi_param_nsm_timeout = 6,

	/**
	 * Timeout peroid for an inactive peer station, in seconds.
	 */
	qcsapi_wifi_param_inact_timeout = 7,

	/**
	 * Wideband CAC configuration
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_wcac_cfg = 8,

	/**
	 * Configure use of 802.11 4-address headers.
	 * * 0 - disable
	 * * 1 - enable for data frames other than A-MSDUs
	 * * 2 - enable for all data frames
	 *
	 * In STA mode, all frames are transmitted according to the above setting.
	 *
	 * In AP mode, this mode applies only to stations from which at least one 802.11 frame
	 *	has been received with a 4-address header.
	 *
	 * 4-address headers are always used to Quantenna devices, regardless of this setting.
	 */
	qcsapi_wifi_param_cfg_4addr = 9,

	/**
	 * Repeater beacon configuration
	 * 0 (disable) AP VAPs will transmit beacons even when the uplink is not established
	 * 1 (enable) AP VAPs will only transmit beacons when the uplink is established
	 */
	qcsapi_wifi_param_repeater_bcn = 10,

	/**
	 * Singleton RTS threshold configuration
	 */
	qcsapi_wifi_param_single_rtsthreshold = 11,

	/**
	 * Aggregate RTS threshold configuration
	 */
	qcsapi_wifi_param_aggr_rtsthreshold = 12,

	/**
	 * Max number of BSSes
	 */
	qcsapi_wifi_param_max_bss_num = 13,

	/**
	 * WCAC duration time for low 160M band
	 */
	qcsapi_wifi_param_wcac_duration = 14,

	/**
	 * WCAC weather channel duration time for up 160M band
	 */
	qcsapi_wifi_param_wcac_wea_duration = 15,

	/**
	 * Configure Multiap backhaul station mode
	 * * 0 - disable
	 * * 1 - enable
	 */
	qcsapi_wifi_param_multiap_backhaul_sta = 16,
	/**
	 * IEEE802.11v Multiple BSSID support
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_multi_bssid = 17,

	/**
	 * AACS verbosity
	 * * 0 (none) - 5 (maximum)
	 */
	qcsapi_wifi_param_aacs_verbose = 18,

	/**
	 * AACS exclusion table
	 */
	qcsapi_wifi_param_aacs_excl_tbl = 19,

	/**
	 * AACS start bw
	 * 0, 20, 40, 80, 160 (MHz)
	 */
	qcsapi_wifi_param_aacs_startbw = 20,

	/**
	 * AACS current config
	 */
	qcsapi_wifi_param_aacs_curr_cfg = 21,

	/**
	 * AACS DL PHY rate average weight
	 * 0 - 100, 100 = use only DL PHY rate
	 */
	qcsapi_wifi_param_aacs_avg_dl_phyrate_wgt = 22,

	/**
	 * AACS use SPF in PER metric
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_inc_spf,

	/**
	 * AACS use LPF in PER metric
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_inc_lpf,

	/**
	 * AACS use threshold table
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_use_thres_tbl,

	/**
	 * AACS normal BW control
	 * 0 - 7 (see AACS Confluence page)
	 */
	qcsapi_wifi_param_aacs_norm_bwctrl,

	/**
	 * AACS normal BW limit
	 * 20, 40, 80, or 160 (MHz)
	 */
	qcsapi_wifi_param_aacs_norm_bwlimit,

	/**
	 * AACS ics BW control
	 * 0 - 7 (see AACS Confluence page)
	 */
	qcsapi_wifi_param_aacs_ics_bwctrl,

	/**
	 * AACS ics BW limit
	 * 20, 40, 80, or 160 (MHz)
	 */
	qcsapi_wifi_param_aacs_ics_bwlimit,

	/**
	 * AACS always-on DFS BW control
	 * 0 - 7 (see AACS Confluence page)
	 */
	qcsapi_wifi_param_aacs_alt_bwctrl,

	/**
	 * AACS always-on DFS BW limit
	 * 20, 40, 80, or 160 (MHz)
	 */
	qcsapi_wifi_param_aacs_alt_bwlimit,

	/**
	 * AACS select fast decision mode
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_fastmode,

	/**
	 * AACS switch timebase
	 * time (in seconds)
	 */
	qcsapi_wifi_param_aacs_sw_timebase_sec,

	/**
	 * AACS switch time factor
	 * factor (1 or greater)
	 */
	qcsapi_wifi_param_aacs_sw_timefactor,

	/**
	 * AACS switch max time
	 * time (in seconds)
	 */
	qcsapi_wifi_param_aacs_sw_max_sec,

	/**
	 * AACS switch reset time
	 * time (in seconds)
	 */
	qcsapi_wifi_param_aacs_sw_reset_sec,

	/**
	 * AACS minimum scan success rate
	 * 0 - 100 (min pct of success scans)
	 */
	qcsapi_wifi_param_aacs_min_scan_succrate,

	/**
	 * AACS decision max time
	 * time (in seconds)
	 */
	qcsapi_wifi_param_aacs_dec_max_sec,

	/**
	 * AACS apcnt penalty
	 * 0 - TBD (sets AP count penalty)
	 */
	qcsapi_wifi_param_aacs_apcnt_penalty,

	/**
	 * AACS enable virtual nodes
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_use_vnode,

	/**
	 * AACS to allow considering DFS channels
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_sel_dfs,

	/**
	 * AACS allow select DFS channel during ICS
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_ics_dfs,

	/**
	 * AACS allow select DFS channel during normal op
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_norm_dfs,

	/**
	 * AACS allow selct DFS channel in Always-on DFS
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_alt_dfs,

	/**
	 * AACS enable DFS threshold table
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_dfs_thres,

	/**
	 * AACS includes all channels
	 */
	qcsapi_wifi_param_aacs_inc_all_chan,

	/**
	 * AACS includes all channels for Always-on DFS
	 */
	qcsapi_wifi_param_aacs_alt_inc_all_chan,

	/**
	 * AACS enable Always-on DFS
	 * 0 (disable) or 1 (enable)
	 */
	qcsapi_wifi_param_aacs_enable_alt,

	/**
	 * AACS lists Always-on DFS channel list
	 * 0 (radar on full bw), 1 (radar on U80), 2 (radar on L80)
	 */
	qcsapi_wifi_param_aacs_alt_list,

	/**
	 * AACS enables/disables STA deficiency
	 * 0 (disabled) or 1 (enabled)
	 */
	qcsapi_wifi_param_aacs_enable_sta_d,

	/**
	 * AACS allowed STA deficiency
	 * 0 (any) or 100 (req'd)
	 */
	qcsapi_wifi_param_aacs_allow_sta_d,

	/**
	 * AACS allowed OBSS
	 * 0 (disabled) or 1 (enabled)
	 */
	qcsapi_wifi_param_aacs_enable_obss,

	/** AACS start channel
	 * 0-255, must be valid channel number for the phy mode
	 */
	qcsapi_wifi_param_aacs_startchan,

	/**
	 * Max forwarded broadcast frames per second.
	 * If the number of broadcast frames received on all wired interfaces or on a vap
	 * interface reaches this number within one second, all subsequent packets will be dropped
	 * for the remainder of that second.
	 * This setting is global and applies to all vaps on all radios as well as to the wired
	 * interfaces.
	 * Valid values are from IEEE80211_PPS_MAX_BCAST_MIN to IEEE80211_PPS_MAX_BCAST_MAX.
	 * The default value is 0 (no limit).
	 */
	qcsapi_wifi_param_pps_max_bcast,

	/**
	 * Max forwarded SSDP frames per second.
	 * If the number of SSDP frames received on all wired interfaces or on a vap
	 * interface reaches this number within one second, all subsequent packets will be dropped
	 * for the remainder of that second.
	 * This setting is global and applies to all vaps on all radios as well as to the wired
	 * interfaces.
	 * Valid values are from IEEE80211_PPS_MAX_SSDP_MIN to IEEE80211_PPS_MAX_SSDP_MAX.
	 * The default value is 0 (no limit).
	 */
	qcsapi_wifi_param_pps_max_ssdp,

	/**
	 * Dual band virtual concurrent (DBVC)
	 * Configure DBVC level for switching channel frequency.
	 * 0 -- disable DBVC, default
	 * 1 -- switch to repeater STA channel once per beacon interval
	 * 2 -- switch to repeater STA channel twice per beacon interval
	 *
	 * Note: this parameter is not supported when SCS or NAC monitor is enabled
	 */
	qcsapi_wifi_param_dbvc_level,

	/**
	 * Dual band virtual concurrent (DBVC)
	 * Configure DBVC dwell time on repeater STA channel.
	 */
	qcsapi_wifi_param_dbvc_dwell,

	qcsapi_wifi_param_max
} qcsapi_wifi_param_type;

#define QCSAPI_WIFI_PARAM_DISABLE	0
#define QCSAPI_WIFI_PARAM_ENABLE	1

typedef enum {
	qcsapi_wifi_param_twt = 0,
	qcsapi_wifi_qdrv_param_max
} qcsapi_wifi_qdrv_param_type;

/**
 * \brief Enumeration used in the U-repeater parameter set/get API.
 *
 * \sa qcsapi_wifi_set_urepeater_param
 * \sa qcsapi_wifi_get_urepeater_param
 */
typedef enum {
	/**
	 * Invalid parameter
	 */
	qcsapi_urepeater_none = 0,
	/**
	 * Maximum hop level parameter
	 */
	qcsapi_urepeater_max_level = 1,
	/**
	 * Current hop level parameter
	 */
	qcsapi_urepeater_curr_level= 2,
} qcsapi_urepeater_type;
#define REPEATER_MAX_LEVEL			255
#define REPEATER_MIN_LEVEL			1

/**
 * \brief Enumeration of the temperature information call types. Currently only one type defined.
 *
 * \sa qcsapi_get_temperature_info_ext
 */
enum {
	/**
	 * Get default temperatures.
	 */
	QCSAPI_TEMPSENS_INFO_TYPE_DEFAULT = 0
} qcsapi_temperature_info_type;

/**
 * \brief Enumeration of the temperature array index values.
 *
 * \sa qcsapi_get_temperature_info_ext
 */
enum {
	/**
	 * External temperature of the RFIC chip.
	 */
	QCSAPI_TEMP_INDEX_RFIC_EXT = 0,
	/**
	 * Internal temperature of the RFIC chip.
	 */
	QCSAPI_TEMP_INDEX_RFIC_INT = 1,
	/**
	 * Internal temperature of the BBIC chip.
	 */
	QCSAPI_TEMP_INDEX_BBIC_INT = 2,
	/**
	 * External temperature of the BBIC chip.
	 */
	QCSAPI_TEMP_INDEX_BBIC_EXT = 3,
	/**
	 * Bounds check value for temperature type index.
	 */
	QCSAPI_TEMP_INDEX_LIMIT
} qcsapi_temperature_index;

/**
 * Basic signed int array definition for internal consistency.
 */
typedef int qcsapi_int_a32[32];
/**
 * Basic unsigned int definition for internal consistency.
 */
typedef	uint32_t	qcsapi_unsigned_int;
/**
 * Basic unsigned 64-bit int definition for internal consistency.
 */
typedef	uint64_t	qcsapi_unsigned_int64;

#define MACFILTERINGMACFMT	"%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDR_SIZE		ETHER_ADDR_LEN
#define MAC_ADDR_STRING_LENGTH	18
#define MAC_ADDR_LIST_SIZE	8

/* set multicast bit in 802.11 address */
#define	MAC_ADDR_SET_MULTICAST(_a)	(*(_a) |= (1 << 0))
/* set locally administered bit in 802.11 address */
#define	MAC_ADDR_SET_LOCAL(_a)		(*(_a) |= (1 << 1))

/**
 * \brief Convenience definition to represent a 6 byte MAC address.
 *
 * Convenience definition to represent a 6 byte MAC address.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a MAC address.
 */
typedef uint8_t		qcsapi_mac_addr[ MAC_ADDR_SIZE ];
typedef uint8_t		qcsapi_mac_addr_list[ MAC_ADDR_LIST_SIZE * MAC_ADDR_SIZE ];

#define QCSAPI_SSID_MAXLEN	(IW_ESSID_MAX_SIZE + 1)
#define QCSAPI_SSID_MAXNUM	32
#define QCSAPI_STATUS_MAXLEN	12
#define QCSAPI_SSID_MAX_RECORDS (6)

/**
 * \brief Convenience definition to represent a 32 byte array.
 *
 * Convenience definition to represent a 32 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 32 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_32bytes {
	uint8_t data[32];
};

/**
 * \brief Convenience definition to represent a 64 byte array.
 *
 * Convenience definition to represent a 64 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 64 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_64bytes {
	uint8_t data[64];
};

/**
 * \brief Convenience definition to represent a 128 unsigned byte array.
 *
 * Convenience definition to represent a 128 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 128 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_128bytes {
	uint8_t data[128];
};

/**
 * \brief Convenience definition to represent a 256 unsigned byte array.
 *
 * Convenience definition to represent a 256 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 256 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_256bytes {
	uint8_t data[256];
};

/**
 * \brief Convenience definition to represent a 512 unsigned byte array.
 *
 * Convenience definition to represent a 512 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 512 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_512bytes {
	uint8_t data[512];
};

/**
 * \brief Convenience definition to represent a 1024 unsigned byte array.
 *
 * Convenience definition to represent a 1024 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 1024 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_1Kbytes {
	uint8_t data[1024];
};

/**
 * \brief Convenience definition to represent a 2048 unsigned byte array.
 *
 * Convenience definition to represent a 2048 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 2048 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_2Kbytes {
	uint8_t data[2048];
};

/**
 * \brief Convenience definition to represent a 3072 unsigned byte array.
 *
 * Convenience definition to represent a 3072 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 3072 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_3Kbytes {
	uint8_t data[3072];
};

/**
 * \brief Convenience definition to represent a 4096 unsigned byte array.
 *
 * Convenience definition to represent a 4096 byte array.
 *
 * \note This type should not be considered a string as embedded NULL bytes
 * are allowed as part of a 4096 byte array.
 *
 * \note This type is not RPC endian-safe and should not be used to
 * pass integer values.
 */
struct qcsapi_data_4Kbytes {
	uint8_t data[4096];
};


/**
 * \brief Convenience definition to represent a 32 integer array.
 *
 * Convenience definition to represent a 32 integer array.
 *
 * \note This type is RPC endian-safe and can be used to
 * pass structures that are not pure byte sequences.
 */
struct qcsapi_int_array32 {
	int32_t val[32];
};

/**
 * \brief Convenience definition to represent a 128 integer array.
 *
 * Convenience definition to represent a 128 integer array.
 *
 * \note This type is RPC endian-safe and can be used to
 * pass structures that are not pure byte sequences.
 */
struct qcsapi_int_array128 {
	int32_t val[128];
};

/**
 * \brief Convenience definition to represent a 256 integer array.
 *
 * Convenience definition to represent a 256 integer array.
 *
 * \note This type is RPC endian-safe and can be used to
 * pass structures that are not pure byte sequences.
 */
struct qcsapi_int_array256 {
	int32_t val[256];
};

/**
 * \brief Convenience definition to represent a 768 integer array.
 *
 * Convenience definition to represent a 768 integer array.
 *
 * \note This type is RPC endian-safe and can be used to
 * pass structures that are not pure byte sequences.
 */
struct qcsapi_int_array768 {
	int32_t val[768];
};

/**
 * \brief Convenience definition to represent a 1024 integer array.
 *
 * Convenience definition to represent a 1024 integer array.
 *
 * \note This type is RPC endian-safe and can be used to
 * pass structures that are not pure byte sequences.
 */
struct qcsapi_int_array1024 {
	int32_t val[1024];
};


/**
 * \anchor SSID_RULES
 * \brief Convenience definition for a string large enough for a single SSID.
 *
 * Convenience definition for a string large enough for a single SSID.
 *
 * This typedef has enough room for a single SSID plus the NULL terminating
 * character.
 *
 * The content within the SSID must be a string with between 1 and 32 characters.
 * Control characters (^C, ^M, etc.) are not permitted in API calls using this type.
 */
typedef char			qcsapi_SSID[ QCSAPI_SSID_MAXLEN ];

/**
 * \anchor SSID2_RULES
 * \brief Convenience definition for a string large enough for a single SSID.
 *
 * Convenience definition for a string large enough for a single SSID.
 *
 * This typedef has enough room for a single SSID plus the NULL terminating
 * character.
 *
 * The variable contain either a string of up to 32 printable charactes, or
 * a hex dump of up to 64 hexadecimal characters, plus a NULL terminationg
 * character. The hexdump format can only be used with the various "SSID2" APIs.
 */
typedef char qcsapi_SSID2[IW_ESSID_STR_MAX_SIZE];

/**
 * \brief The enum is used to specify the format of an SSID parameter.
 */
typedef enum {
	/**
	 * The SSID parameter is a printable string.
	 */
	qcsapi_ssid_fmt_str = 0,
	/**
	 * The SSID parameter is a hexdump string.
	 */
	qcsapi_ssid_fmt_hex_str = 1,
	/**
	 * Any type of SSID parameter is acceptable.
	 */
	qcsapi_ssid_fmt_any = 2,
} qcsapi_ssid_fmt;

#define QCSAPI_MCS_RATE_MAXLEN	8


/**
 * \brief Type used to contain an MCS definition.
 *
 * QCSAPI MCS rate maximum length is distinct from MaxBitRate in TR-98.
 * TR-98 provides for 4 characters to represent the bit rate in MBPS.
 * QCSAPI MCS rate stores MCS rate specs - e.g. MCS0, MCS6, MCS76, etc.
 * Provide a bit more space for future expansion.
 * As with all QCSAPI maximum length definitions, space for the NULL ('\\0') is included.
 * So only QCSAPI_MCS_RATE_MAXLEN - 1 (7) non-NULL chars are available.
 */
typedef char	qcsapi_mcs_rate[ QCSAPI_MCS_RATE_MAXLEN ];

/**
 * \brief Array of available ECC groups initialized by \ref QCSAPI_ECC_GROUPS_SAE_LIST.
 */
extern qcsapi_unsigned_int qcsapi_ecc_groups_list_sae[];

/**
 * \brief Array of available ECC groups initialized by \ref QCSAPI_ECC_GROUPS_OWE_LIST.
 */
extern qcsapi_unsigned_int qcsapi_ecc_groups_list_owe[];

/**
 * List of the valid values for ECC groups for SAE (WPA3 Security Considerations).
 *
 * https://www.iana.org/assignments/ipsec-registry/ipsec-registry.xml#ipsec-registry-10
 */
#define QCSAPI_ECC_GROUPS_LIST_SAE {19, 20, 21}

/**
 * List of the valid values for ECC groups for OWE.
 *
 * https://www.iana.org/assignments/ipsec-registry/ipsec-registry.xml#ipsec-registry-10
 */
#define QCSAPI_ECC_GROUPS_LIST_OWE {19, 20, 21, 22, 23, 24, 25, 26}

#define QCSAPI_ECC_GROUPS_LIST_SAE_SIZE ARRAY_SIZE(qcsapi_ecc_groups_list_sae)
#define QCSAPI_ECC_GROUPS_LIST_OWE_SIZE ARRAY_SIZE(qcsapi_ecc_groups_list_owe)

/*
 * Leave extra char in these string-with-maximum-length data types for NULL termination.
 * The standard suggests that a string(64) can hold upto 64 non-NULL chars
 *
 * Also, the standard includes string(63); this is represented as string_64,
 * with the corresponding QCSAPI enforcing the limit of 63 chars.
 *
 * Finally, all QCSAPIs that work with strings of defined maximum length are
 * required to enforce string length limits, and cannot rely on the special
 * string-with-maximum-length data type to enforce the corresponding limit.
 */

/**
 * \brief Convenience definition for a string of size 16.
 *
 * Convenience definition for a string of size 16.
 *
 * This type can contain a string of maximum size 16 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_16[ 17 ];
/**
 * \brief Convenience definition for a string of size 32.
 *
 * Convenience definition for a string of size 32.
 *
 * This type can contain a string of maximum size 32 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_32[ 33 ];
/**
 * \brief Convenience definition for a string of size 64.
 *
 * Convenience definition for a string of size 64.
 *
 * This type can contain a string of maximum size 64 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_64[ 65 ];
/**
 * \brief Convenience definition for a string of size 128.
 *
 * Convenience definition for a string of size 128.
 *
 * This type can contain a string of maximum size 128 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_128[ 129 ];
/**
 * \brief Convenience definition for a string of size 256.
 *
 * Convenience definition for a string of size 256.
 *
 * This type can contain a string of maximum size 256 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_256[ 257 ];
/**
 * \brief Convenience definition for a string of size 512.
 *
 * Convenience definition for a string of size 512.
 *
 * This type can contain a string of maximum size 512 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_512[ 513 ];
/**
 * \brief Convenience definition for a string of size 1024.
 *
 * Convenience definition for a string of size 1024.
 *
 * This type can contain a string of maximum size 1024 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_1024[ 1025 ];
/**
 * \brief Convenience definition for a string of size 2048.
 *
 * Convenience definition for a string of size 2048.
 *
 * This type can contain a string of maximum size 2048 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_2048[ 2049 ];
/**
 * \brief Convenience definition for a string of size 4096.
 *
 * Convenience definition for a string of size 4096.
 *
 * This type can contain a string of maximum size 4096 bytes, plus the
 * NULL terminating character.
 */
typedef char	string_4096[ 4097 ];

/**
 * \brief Bit positions in the \link ap_80211_proto \endlink field for supported IEEE 802.11
 * protocols.
 *
 * Bit positions in the \link ap_80211_proto \endlink field for supported IEEE 802.11 protocols.
 */
enum qcsapi_ap_80211_proto_flags {
	/** IEEE 802.11b */
	IEEE80211_PROTO_11B = 0x00000001,

	/** IEEE 802.11g */
	IEEE80211_PROTO_11G = 0x00000002,

	/** IEEE 802.11a */
	IEEE80211_PROTO_11A = 0x00000004,

	/** IEEE 802.11n */
	IEEE80211_PROTO_11N = 0x00000008,

	/** IEEE 802.11ac */
	IEEE80211_PROTO_11AC = 0x00000010,

	/** IEEE 802.11ax */
	IEEE80211_PROTO_11AX = 0x00000020
};

#define IEEE80211_WMM_AC_BE	0	/* best effort */
#define IEEE80211_WMM_AC_BK	1	/* background */
#define IEEE80211_WMM_AC_VI	2	/* video */
#define IEEE80211_WMM_AC_VO	3	/* voice */

#define IEEE8021P_PRIORITY_ID0	0
#define IEEE8021P_PRIORITY_ID1	1
#define IEEE8021P_PRIORITY_ID2	2
#define IEEE8021P_PRIORITY_ID3	3
#define IEEE8021P_PRIORITY_ID4	4
#define IEEE8021P_PRIORITY_ID5	5
#define IEEE8021P_PRIORITY_ID6	6
#define IEEE8021P_PRIORITY_ID7	7

#define IEEE8021P_PRIORITY_NUM	8

#define IEEE80211_QOS_MAP_DSCP_EXCEPT_MAX	21
#define IEEE80211_QOS_MAP_MAX	255

#define QCSAPI_WIFI_AC_MAP_SIZE	(64)

#define IP_DSCP_NUM		64

/**
 * \struct qcsapi_channel_power_table
 *
 * \brief Structure to contain the power table for a single channel on devices with up to 4
 * transmit chains.
 *
 * This structure is used as an input or return parameter in the channel power table APIs.
 * It is filled in as as a power level (in dBm) for each combination of channel bandwidth,
 * spatial streams number and beamforming configuration. A total of
 * 24 power levels must be configured.
 *
 * For example, the following code snippet shows an initialisation of the structure and
 * the corresponding channel/bandwidth/SS power levels.
 *
 * @code
 * qcsapi_channel_power_table channel_36;
 * memset(&channel_36, 0, sizeof(channel_36));
 * channel_36.channel = 36;
 * channel_36.power_20M[QCSAPI_POWER_INDEX_BFOFF_1SS] = 19;
 * channel_36.power_20M[QCSAPI_POWER_INDEX_BFOFF_2SS] = 19;
 * ...
 * channel_36.power_40M[QCSAPI_POWER_INDEX_BFON_3SS] = 17;
 * channel_36.power_40M[QCSAPI_POWER_INDEX_BFON_4SS] = 17;
 * ...
 * channel_36.power_80M[QCSAPI_POWER_INDEX_BFON_3SS] = 15;
 * channel_36.power_80M[QCSAPI_POWER_INDEX_BFON_4SS] = 15;
 * @endcode
 *
 * \sa qcsapi_wifi_get_chan_power_table
 * \sa qcsapi_wifi_set_chan_power_table
 * \sa qcsapi_power_indices
 */
typedef struct qcsapi_channel_power_table
{
	/**
	 * The channel number.
	 */
	uint8_t		channel;
	/**
	 * The power for 20Mhz bandwidth. For the index, please see the definition for
	 * "QCSAPI_POWER_INDEX..."
	 */
	int		power_20M[QCSAPI_POWER_TOTAL];
	/**
	 * The power for 40Mhz bandwidth. For the index, please see the definition for
	 * "QCSAPI_POWER_INDEX..."
	 */
	int		power_40M[QCSAPI_POWER_TOTAL];
	/**
	 * The power for 80Mhz bandwidth. For the index, please see the definition for
	 * "QCSAPI_POWER_INDEX..."
	 */
	int		power_80M[QCSAPI_POWER_TOTAL];
	/**
	 * The power for 160Mhz bandwidth. For the index, please see the definition for
	 * "QCSAPI_POWER_INDEX..."
	 */
	int		power_160M[QCSAPI_POWER_TOTAL];
} qcsapi_channel_power_table;

/**
 * \struct qcsapi_chan_tx_powers_info
 *
 * \brief Information on maximum Tx power values for specified channel.
 *
 * The structure represents information on maximum Tx power values for the specified
 * channel for all possible configurations.
 * Power values are stored in 4-dimensional array of type qcsapi_chan_powers.
 *
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_regulatory_chan_txpower_set
 */

/*
 * The following number must be equal to
 * QCSAPI_PWR_IDX_FEM_PRIPOS_NUM * QCSAPI_PWR_IDX_BF_NUM * QCSAPI_PWR_IDX_SS_NUM * QCSAPI_PWR_BW_NUM
 *
 * It is defined separately because automatic compile-time marshalling framework requires all
 * function arguments to be of a priory known size.
 */
#define QCSAPI_CHAN_NUM_PWR_VALUES	128

struct qcsapi_chan_tx_powers_info {
	uint8_t channel;
	int8_t maxpwr[QCSAPI_CHAN_NUM_PWR_VALUES];
};

/**
 * \struct qcsapi_chan_tx_powers_with_decimal_info
 *
 * \brief Information on maximum Tx power values for specified channel.
 *
 * The structure represents information on maximum Tx power values for the specified
 * channel for all possible configurations.
 * Power values are stored in 4-dimensional array of type qcsapi_chan_powers.
 * Integer part and decimal part are stored separately.
 *
 * \sa qcsapi_regulatory_chan_txpower_with_decimal_get
 * \sa qcsapi_regulatory_chan_txpower_with_decimal_set
 */

struct qcsapi_chan_tx_powers_with_decimal_info {
	uint8_t channel;
	int8_t maxpwr[QCSAPI_CHAN_NUM_PWR_VALUES];
	int8_t maxpwr_decimal[QCSAPI_CHAN_NUM_PWR_VALUES];
};

typedef int8_t qcsapi_chan_powers[QCSAPI_PWR_IDX_FEM_PRIPOS_NUM][QCSAPI_PWR_IDX_BF_NUM]
					[QCSAPI_PWR_IDX_SS_NUM][QCSAPI_PWR_BW_NUM];

/**
 * \brief This structure represents the properties of an AP (BSS).
 *
 * This structure represents the properties of an AP (BSS).
 *
 * \sa qcsapi_wifi_get_properties_AP
 */
typedef struct qcsapi_ap_properties
{
	/** Service Set ID (SSID) */
	qcsapi_SSID		ap_name_SSID;

	/** Basic Service Set ID (BSSID) */
	qcsapi_mac_addr		ap_mac_addr;

	/** Flags as defined by \ref enum qcsapi_ap_flag_bits, in expanded hexadecimal */
	qcsapi_unsigned_int	ap_flags;

	/** Operating channel number */
	int			ap_channel;

	/** Maximum supported bandwidth */
	int			ap_bw;

	/** Received Signal Strength Indicator (RSSI), in the range [0 - 68] */
	int			ap_RSSI;

	/** Bitmap of accepted security protocol bit fields
	 * 0 (none), 1 (WPA), 2 (WPA2), 3 (WPA/WPA2), 4 (WPA3)
	 */
	int			ap_protocol;

	/** Bitmap of accepted cipher suites - 1 (TKIP), 2 (CCMP) */
	int			ap_encryption_modes;

	/** Bitmap of accepted key management algorithms
	 * 0 (none), 1 (PSK), 2 (EAP), 3 (SAE), 4 (OWE), 5 (DPP)
	 */
	int			ap_authentication_mode;

	/** The fastest data rate at which the BSS is capable of transmitting, in bps */
	uint32_t		ap_best_data_rate;

	/** WPS capability */
	int			ap_wps;

	/**
	 * Supported IEEE 802.11 protocols (e.g a, b, g, n, ac, ax), as defined by
	 * \ref qcsapi_ap_80211_proto_flags.
	 */
	int			ap_80211_proto;

	/** QHop role - 0 (none), 1 (MBS) or 2 (RBS) */
	int			ap_qhop_role;

	/** Noise, in dBm */
	int			ap_noise;

	/** Operating mode */
	qcsapi_wifi_mode	ap_opmode;

	/** Beacon interval, in TUs (1,024 usecs). */
	int			ap_bintval;

	/** HT secondary channel offset - 0 (none), 1 (above), 2 (undefined) or 3 (below) */
	int			ap_ht_secoffset;

	/**
	 * VHT/HT center channel index 1 in 20MHz, 40MHz, 80MHz, 160MHz channels.
	 * Center channel in 80+80MHz channels.
	 * Center channel of segment 1.
	 */
	int			ap_chan_center1;

	/**
	 * VHT center channel index 2 in 20MHz, 40MHz, 80MHz, 160MHz channels.
	 * Center channel in 80+80MHz channels.
	 * Center channel of segment 2.
	 */
	int			ap_chan_center2;

	/** Time at which the AP was last seen, in seconds since boot-up */
	uint32_t		ap_last_beacon;

	/** Delivery Traffic Indication Map (DTIM) period */
	int			ap_dtimperiod;

	/** Flag to indicate the presence of a non-ERP station */
	int			ap_11b_present;

	/** Basic rate set, as a comma-separated list of rates in Mbps */
	uint8_t			ap_basic_rates[QCSAPI_RATESET_STRING_LEN];

	/** Supported rate set, as a comma-separated list of rates in Mbps */
	uint8_t			ap_support_rates[QCSAPI_RATESET_STRING_LEN];
} qcsapi_ap_properties;

/**
 * \brief Structure to contain per node statistics.
 *
 * This structure is used as a return parameter in the per-node association APIs
 * associated with statistics gathering.
 *
 * \sa qcsapi_wifi_get_node_stats
 */
typedef struct qcsapi_node_stats
{
	/* TX path */
	/**
	 * The number of transmitted bytes to the node.
	 */
	uint64_t	tx_bytes;
	/**
	 * The number of transmitted packets to the node.
	 */
	uint32_t	tx_pkts;
	/**
	 * The number of transmit discards to the node.
	 */
	uint32_t	tx_discard;
	/**
	 * The number of data packets transmitted through
	 * * wireless media for each Access Classes(AC).
	 */
	uint32_t	tx_wifi_sent[WMM_AC_NUM];
	/**
	 * The number of dropped data packets failed to transmit through
	 * wireless media for each Access Classes(AC)
	 */
	uint32_t	tx_wifi_drop[WMM_AC_NUM];
	/**
	 * The number of transmit errors to the node.
	 */
	uint32_t	tx_err;
	/**
	 * The number of transmitted unicast packets to the node.
	 */
	uint32_t	tx_unicast;
	/**
	 * The number of transmitted multicast packets to the node.
	 */
	uint32_t	tx_multicast;
	/**
	 * The number of transmitted broadcast packets to the node.
	 */
	uint32_t	tx_broadcast;
	/**
	 * TX PHY rate in megabits per second (Mbps).
	 */
	uint32_t	tx_phy_rate;

	/**
	 * The number of transmitted management frames to the node.
	 */
	uint32_t	tx_mgmt;

	/* RX path */
	/**
	 * The number of received bytes from the node.
	 */
	uint64_t	rx_bytes;
	/**
	 * The number of received packets from the node.
	 */
	uint32_t	rx_pkts;
	/**
	 * The numbder of received packets discarded from the node.
	 */
	uint32_t	rx_discard;
	/**
	 * The number of received packets in error from the node.
	 */
	uint32_t	rx_err;
	/**
	 * The number of received unicast packets from the node.
	 */
	uint32_t	rx_unicast;
	/**
	 * The number of received multicast packets from the node.
	 */
	uint32_t	rx_multicast;
	/**
	 * The number of received broadcast packets form the node.
	 */
	uint32_t	rx_broadcast;
	/**
	 * The number of received unknown packets from the node.
	 */
	uint32_t	rx_unknown;
	/**
	 * RX PHY rate in megabits per second (MBPS).
	 */
	uint32_t	rx_phy_rate;
	/**
	 * The number of received management frames from the node.
	 */
	uint32_t	rx_mgmt;
	/**
	 * The number of received control from from the node.
	 */
	uint32_t	rx_ctrl;
	/**
	 * The MAC address of the node.
	 */
	qcsapi_mac_addr mac_addr;
	/**
	 * The hw noise of the node.
	 */
	int32_t		hw_noise;
	/**
	 * The snr of the node.
	 */
	int32_t		snr;
	/**
	 * The rssi of the node.
	 */
	int32_t		rssi;
	/**
	 * The bandwidth of the node.
	 */
	int32_t		bw;
	/**
	 * Last Tx MCS index.
	 */
	uint8_t		tx_mcs;
	/**
	 * Last Rx MCS index.
	 */
	uint8_t		rx_mcs;
	/**
	 * Number of spatial streams used in last transmission.
	 */
	uint8_t		tx_nss;
	/**
	 * Number of spatial streams received in last reception.
	 */
	uint8_t		rx_nss;
	/**
	 * Bandwidth used in last transmission.
	 */
	uint8_t		tx_bw;
	/**
	 * Bandwidth used in last reception.
	 */
	uint8_t		rx_bw;
	/**
	 * Was short guard interval used in last transmission?
	 */
	uint8_t		tx_sgi;
	/**
	 * Was short guard interval used in last reception?
	 */
	uint8_t		rx_sgi;
} qcsapi_node_stats;

/**
 * \brief Structure to contain per interface statistics.
 *
 * This structure is used as a return parameter in the per-interface APIs
 * associated with statistics gathering.
 *
 * \sa qcsapi_get_vap_extstats
 */
typedef struct _qcsapi_vap_extstats
{
	/**
	 * The number of retransmitted packets on the interface.
	 */
	uint32_t tx_retrans;
	/**
	 * The number of failed retransmitted packets on the interface.
	 */
	uint32_t tx_fretrans;
	/**
	 * The number of sucessfully retransmitted pkts on the interface.
	 */
	uint32_t tx_retries;
	/**
	 * The number of sucessfully retransmitted pkts on the interface
	 * with more than once retry.
	 */
	uint32_t tx_mretries;
	/**
	 * The number of transmitted aggregated pkts on the interface.
	 */
	uint32_t tx_aggpkts;
	/**
	 * The number of transmitted pkts never get an ack on the interface.
	 */
	uint32_t tx_toutpkts;
} qcsapi_vap_extstats;

/**
 * \brief Structure to contain per interface statistics.
 *
 * This structure is used as a return parameter in the per-interface APIs
 * associated with statistics gathering.
 *
 * \sa qcsapi_get_interface_stats
 */
typedef struct _qcsapi_interface_stats
{
	/* TX Path */
	/**
	 * The number of transmitted bytes on the interface.
	 */
	uint64_t tx_bytes;
	/**
	 * The number of transmitted packets on the interface.
	 */
	uint32_t tx_pkts;
	/**
	 * The number of discarded transmit packets on the interface.
	 */
	uint32_t tx_discard;
	/**
	 * The number of transmit errors on the interface.
	 */
	uint32_t tx_err;
	/**
	 * The number of transmitted unicast packets on the interface.
	 */
	uint32_t tx_unicast;
	/**
	 * The number of transmitted multicast packets on the interface.
	 */
	uint32_t tx_multicast;
	/**
	 * The number of transmitted broadcast packets on the interface.
	 */
	uint32_t tx_broadcast;

	/* RX Path */
	/**
	 * The number of received bytes on the interface.
	 */
	uint64_t rx_bytes;
	/**
	 * The number of received packets on the interface.
	 */
	uint32_t rx_pkts;
	/**
	 * The number of received packets discarded on the interface.
	 */
	uint32_t rx_discard;
	/**
	 * The number of received packets in error on the interface.
	 */
	uint32_t rx_err;
	/**
	 * The number of received unicast packets on the interface.
	 */
	uint32_t rx_unicast;
	/**
	 * The number of received multicast packets on the interface.
	 */
	uint32_t rx_multicast;
	/**
	 * The number of received broadcast packets on the interface.
	 */
	uint32_t rx_broadcast;
	/**
	 * The number of received unknown packets on the interface.
	 */
	uint32_t rx_unknown;
} qcsapi_interface_stats;

/**
 * \brief Structure containing PHY statistics.
 *
 * This structure is used as a return parameter in the per-interface APIs
 * associated with PHY statistics gathering.
 *
 * \sa qcsapi_get_phy_stats
 */
typedef struct _qcsapi_phy_stats
{
	/**
	 * The timestamp in seconds since system boot up
	 */
	uint32_t	tstamp;
	/**
	 * Associated Station count or if Station is associated
	 */
	uint32_t	assoc;
	/**
	 * Current active channel
	 */
	uint32_t	channel;
	/**
	 * Attenuation
	 */
	uint32_t	atten;
	/**
	 * Total CCA
	 */
	uint32_t	cca_total;
	/**
	 * Transmit CCA
	 */
	uint32_t	cca_tx;
	/**
	 * Receive CCA
	 */
	uint32_t	cca_rx;
	/**
	 * CCA interference
	 */
	uint32_t	cca_int;
	/**
	 * CCA Idle
	 */
	uint32_t	cca_idle;
	/**
	 * Received packets counter
	 */
	uint32_t	rx_pkts;
	/**
	 * Receive gain in dBm
	 */
	uint32_t	rx_gain;
	/**
	 * Received packet counter with frame check error
	 */
	uint32_t	rx_cnt_crc;
	/**
	 * Received noise level in dBm
	 */
	float		rx_noise;
	/**
	 * Transmitted packets counter
	 */
	uint32_t	tx_pkts;
	/**
	 * Deferred packet counter in transmission
	 */
	uint32_t	tx_defers;
	/**
	 * Time-out counter for transimitted packets
	 */
	uint32_t	tx_touts;
	/**
	 * Retried packets counter in transmission
	 */
	uint32_t	tx_retries;
	/**
	 * Counter of short preamble errors
	 */
	uint32_t	cnt_sp_fail;
	/**
	 * Counter of long preamble errors
	 */
	uint32_t	cnt_lp_fail;
	/**
	 * MCS index for last received packet
	 */
	uint32_t	last_rx_mcs;
	/**
	 * MCS index for last transimtted packet
	 */
	uint32_t	last_tx_mcs;
	/**
	 * Received signal strength indicator in dBm
	 */
	float		last_rssi;
	/**
	 * Per Chain RSSI
	 */
	float		last_rssi_array[QCSAPI_QDRV_NUM_RF_STREAMS];
	/**
	 * Received channel power level in dBm
	 */
	float		last_rcpi;
	/**
	 * Error vector magnitude measured in dBm
	 */
	float		last_evm;
	/**
	 * Per Chain EVM
	 */
	float		last_evm_array[QCSAPI_QDRV_NUM_RF_STREAMS];
	/**
	 * Timestamp for last transmitted packet
	 */
	uint32_t	last_tx_pkt_timestamp;
	/**
	 * Timestamp for last received packet
	 */
	uint32_t	last_rx_pkt_timestamp;
} qcsapi_phy_stats;

/**
 * \brief Structure containing per client mlme statistics.
 *
 * This structure is used as a return parameter in the mlme statistics
 * request functions.
 *
 * \sa qcsapi_wifi_get_mlme_stats_per_association
 * \sa qcsapi_wifi_get_mlme_stats_per_mac
 */
typedef struct _qcsapi_mlme_stats
{
	unsigned int auth;
	unsigned int auth_fails;
	unsigned int assoc;
	unsigned int assoc_fails;
	unsigned int deauth;
	unsigned int diassoc;
} qcsapi_mlme_stats;

/**
 * \brief Structure containing the list of macs.
 *
 * This structure is used as a return parameter in
 * mlme statistics macs request function
 *
 * \sa qcsapi_wifi_get_mlme_stats_macs_list
 */
#define QCSAPI_MLME_STATS_MAX_MACS	128
typedef struct _qcsapi_mlme_stats_macs {
	/**
	 * MAC addresses existing in mlme stats
	 */
	qcsapi_mac_addr addr[QCSAPI_MLME_STATS_MAX_MACS];
} qcsapi_mlme_stats_macs;

/**
 * Used with API 'qcsapi_wifi_start_cca'
 */
struct qcsapi_cca_info
{
	/**
	 * Channel to switch to for off channel CCA measurements
	 */
	int	cca_channel;

	/**
	 * Duration to stay on the channel being measured, in milliseconds
	 */
	int	cca_duration;
};

/**
 * \brief Structure containing a list of channels for various SCS APIs.
 *
 * This structure is used in SCS API to represent a list of channels.
 *
 * \sa qcsapi_wifi_set_scs_active_chan_list
 * \sa qcsapi_wifi_get_scs_active_chan_list
 */
typedef struct qcsapi_scs_chan_list {
	/**
	 * Number of elements in the following array
	 */
	uint8_t num;
	/**
	 * Channel numbers
	 */
	uint8_t chan[IEEE80211_CHAN_MAX];
} qcsapi_scs_chan_list;

/**
 * \brief Structure containing SCS report for current channel.
 *
 * This structure is used as a return parameter in the SCS API to return
 * report for the current channel.
 *
 * \sa qcsapi_wifi_get_scs_currchan_report
 */
typedef struct qcsapi_scs_currchan_rpt {
	/**
	 * Current channel number
	 */
	uint8_t chan;
	/**
	 * Total try count for cca sampling
	 */
	uint16_t cca_try;
	/**
	 * CCA idle count
	 */
	uint16_t cca_idle;
	/**
	 * CCA busy count
	 */
	uint16_t cca_busy;
	/**
	 * CCA interference count
	 */
	uint16_t cca_intf;
	/**
	 * CCA transmitting count
	 */
	uint16_t cca_tx;
	/**
	 * Transmiting time in ms
	 */
	uint16_t tx_ms;
	/**
	 * Receiving time in ms
	 */
	uint16_t rx_ms;
	/**
	 * Preamble error count
	 */
	uint32_t pmbl;
} qcsapi_scs_currchan_rpt;

#define QCSAPI_SCS_REPORT_CHAN_NUM	32

/**
 * \brief Structure containing SCS report for all channels.
 *
 * This structure is used as a return parameter in the SCS API to return
 * report for the all channels.
 *
 * The attributes for a certain channel use the same index into each
 * attribute array.
 *
 * \sa qcsapi_wifi_get_scs_stat_report
 */
typedef struct qcsapi_scs_ranking_rpt {
	/**
	 * Valid record number in the following attribute arrays
	 */
	uint8_t num;
	/**
	 * Channel numbers
	 */
	uint8_t chan[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Whether channel is DFS channel or not
	 */
	uint8_t dfs[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Txpower
	 */
	uint8_t txpwr[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Beacon count
	 */
	uint16_t numbeacons[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Ranking metric
	 */
	int32_t metric[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Ranking metric age
	 */
	uint32_t metric_age[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * CCA interference
	 */
	uint16_t cca_intf[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Preamble error detected by AP
	 */
	uint32_t pmbl_ap[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Maximum preamble error detected by STAs
	 */
	uint32_t pmbl_sta[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Amount of time the channel was used in seconds
	 */
	uint32_t duration[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Number of times the channel was used
	 */
	uint32_t times[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Channel availability status
	 */
	uint8_t chan_avail_status[QCSAPI_SCS_REPORT_CHAN_NUM];
} qcsapi_scs_ranking_rpt;

/**
 * \brief Structure containing SCS interference report for all channels.
 *
 * This structure is used as a return parameter in the SCS API to return
 * interference report for all channels.
 *
 * The attributes for a certain channel use the same index into each
 * attribute array.
 *
 * \sa qcsapi_wifi_get_scs_interference_report
 */
typedef struct qcsapi_scs_interference_rpt {
	/**
	 * Valid record number in the following attribute arrays
	 */
	uint8_t num;
	/**
	 * Channel numbers
	 */
	uint8_t chan[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * CCA interference of 20MHz channel
	 */
	uint16_t cca_intf_20[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * CCA interference of 40MHz channel
	 */
	uint16_t cca_intf_40[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * CCA interference of 80MHz channel
	 */
	uint16_t cca_intf_80[QCSAPI_SCS_REPORT_CHAN_NUM];
} qcsapi_scs_interference_rpt;

/**
 * \brief Structure containing the scores of all channels.
 *
 * This structure is used as a return parameter in the SCS API to return
 * the scores of the all channels.
 *
 * The attributes for a certain channel use the same index into each
 * attribute array.
 *
 * \sa qcsapi_wifi_get_scs_score_report
 */
typedef struct qcsapi_scs_score_rpt {
	/*
	 * Valid record number in the following attribute arrays
	 */
	uint8_t num;
	/*
	 * Channel numbers
	 */
	uint8_t chan[QCSAPI_SCS_REPORT_CHAN_NUM];
	/*
	 * The channel score (0 - 100, 100 means the best)
	 */
	uint8_t score[QCSAPI_SCS_REPORT_CHAN_NUM];
} qcsapi_scs_score_rpt;

/**
 * \brief Structure containing auto channel report for initial channel selection.
 *
 * This structure is used as a return parameter in the Auto Channel API to return
 * report for initial channel selection.
 *
 * The attributes for a certain channel use the same index into each attribute array.
 *
 * \sa qcsapi_wifi_get_autochan_report
 */
typedef struct qcsapi_autochan_rpt {
	/**
	 * Valid record number in the following attribute arrays
	 */
	uint8_t num;
	/**
	 * Channel number
	 */
	uint8_t chan[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Whether channel is DFS channel or not
	 */
	uint8_t dfs[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Txpower
	 */
	uint8_t txpwr[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Ranking metric
	 */
	int32_t metric[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Number of beacons detected
	 */
	uint32_t numbeacons[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Co-channel interference index
	 */
	uint32_t cci[QCSAPI_SCS_REPORT_CHAN_NUM];
	/**
	 * Adjacent Channel interference index
	 */
	uint32_t aci[QCSAPI_SCS_REPORT_CHAN_NUM];
} qcsapi_autochan_rpt;

/**
 * This structure is the same as 'struct ieee80211req_scs_param_rpt', but (re)defined for
 * convenience
 */
typedef struct qcsapi_scs_param_rpt {
	uint32_t scs_cfg_param;
	uint32_t scs_signed_param_flag;
} qcsapi_scs_param_rpt;

/**
 * Used with API 'qcsapi_wifi_get_assoc_records'
 */
#define QCSAPI_ASSOC_MAX_RECORDS	32
typedef struct qcsapi_assoc_records {
	/**
	 * MAC addresses of remote nodes that have associated
	 */
	qcsapi_mac_addr addr[QCSAPI_ASSOC_MAX_RECORDS];
	/**
	 * Time stamp of the most recent association by the corresponding remote node
	 */
	uint32_t timestamp[QCSAPI_ASSOC_MAX_RECORDS];
} qcsapi_assoc_records;


/**
 * Used with API 'qcsapi_wifi_node_get_tx_airtime'
 */
typedef struct _qcsapi_node_tx_airtime {
	/**
	 * MAC address of the node
	 */
	qcsapi_mac_addr addr;

	/**
	 * current instantaneous airtime
	 */
	uint32_t tx_airtime;

	/**
	 * cumulative airtime during the period from start to stop
	 */
	uint32_t tx_airtime_accum;
}qcsapi_node_tx_airtime;

#define QCSAPI_DCS_CHAN_LIST_NUM	32
#define QCSAPI_DCS_INTERVAL_MIN_MASK	0xff
#define QCSAPI_DCS_INTERVAL_SEC_MASK	0xff00
#define QCSAPI_DCS_INTERVAL_SEC_SHIFT	8
#define QCSAPI_DCS_INTERVAL_PACK(_minutes, _seconds) \
	((((_seconds) << QCSAPI_DCS_INTERVAL_SEC_SHIFT) & QCSAPI_DCS_INTERVAL_SEC_MASK) \
		| ((_minutes) & QCSAPI_DCS_INTERVAL_MIN_MASK))
#define QCSAPI_DCS_INTERVAL_GET_MINUTES(_i) \
	((_i) & QCSAPI_DCS_INTERVAL_MIN_MASK)
#define QCSAPI_DCS_INTERVAL_GET_SECONDS(_i) \
	(((_i) & QCSAPI_DCS_INTERVAL_SEC_MASK) >> QCSAPI_DCS_INTERVAL_SEC_SHIFT)
#define QCSAPI_DCS_INTERVAL_TO_SECONDS(_i) \
	(QCSAPI_DCS_INTERVAL_GET_MINUTES(_i) * 60 + QCSAPI_DCS_INTERVAL_GET_SECONDS(_i))
#define QCSAPI_DCS_INTERVAL_FROM_SECONDS(_sec) \
	QCSAPI_DCS_INTERVAL_PACK((_sec) / 60, (_sec) % 60)
/**
 * \struct qcsapi_dcs_params
 *
 * \brief Structure to contain the parameters for DCS (Dynamic Channel Selection).
 *
 * This structure is used as an input or return parameter in the DCS APIs.
 * Including the DCS status (started/stopped), scan interval,
 * scan duration, dwell time, spacing time, and channel list.
 *
 * \sa qcsapi_wifi_start_dcs_scan
 * \sa qcsapi_wifi_stop_dcs_scan
 * \sa qcsapi_wifi_get_dcs_scan_params
 */
typedef struct qcsapi_dcs_params {
	/**
	 * Feature status, only meaningful when getting parameters
	 */
	uint8_t dcs_status;
	/**
	 * Scan timer interval.
	 * This parameter was designed in minutes with valid range from 1 min to 60 min,
	 * but in order to support configuring scan interval to less than 1 min with
	 * backward-compatibility, we only use lower 8 bits for the scan interval in minutes,
	 * and use the higher 8 bits as extra scan interval in seconds, that is:
	 * total scan_interval in seconds = (scan_interval & 0xff) * 60 + (scan_interval >> 8)
	 */
	uint16_t scan_interval;
	/**
	 * Duration of each scan in milliseconds
	 */
	uint16_t scan_duration;
	/**
	 * Dwell time on each off-channel in milliseconds
	 */
	uint16_t dwell_time;
	/**
	 * Spacing time on each channel in milliseconds
	 */
	uint16_t spacing;
	/**
	 * List of channel to be scanned
	 */
	uint8_t chan_list[QCSAPI_DCS_CHAN_LIST_NUM];
} qcsapi_dcs_params;
/** @} */


/* Channel 0 means channel auto */
#define QCSAPI_ANY_CHANNEL	0
#define QCSAPI_MIN_CHANNEL	1
#define QCSAPI_MIN_CHANNEL_5G	36
#define QCSAPI_MAX_CHANNEL	IEEE80211_CHAN_MAX

#define RESTORE_DEFAULT_CONFIG	"/scripts/restore_default_config"

#ifndef BRIDGE_DEVICE
#define BRIDGE_DEVICE	"br0"
#endif

#define QCSAPI_CSW_MAX_RECORDS 32

/**
 * Channel switch history record
 */
typedef struct _qcsapi_csw_record {
	/**
	 * Entry number. Maximum value is QCSAPI_CSW_MAX_RECORDS.
	 */
	uint32_t cnt;
	/**
	 * Index of the latest channel change.
	 */
	int32_t index;
	/**
	 * Channel number which the device switch to. If the
	 * value is 0, it means the record is invalid.
	 */
	uint32_t channel[QCSAPI_CSW_MAX_RECORDS];
	/**
	 * Time when the channel change happens.
	 */
	uint32_t timestamp[QCSAPI_CSW_MAX_RECORDS];
	/**
	 * Reason for channel change. Possible values are enumerated by
	 * \link ieee80211_csw_reason \endlink
	 */
	uint32_t reason[QCSAPI_CSW_MAX_RECORDS];
} qcsapi_csw_record;

/**
 * Data should be set to the dscp to ac mapping
 */
typedef struct _qcsapi_dscp2ac_data {
	/**
	 * dscp value to be mapped
	 */
	uint8_t ip_dscp_list[64];
	/**
	 * Length of DSCP list
	 */
	uint8_t list_len;
	/**
	 * WME Access Class
	 */
	uint8_t ac;
} qcsapi_dscp2ac_data;


struct qcsapi_chan_disabled_data {
	uint8_t chan[QCSAPI_MAX_CHANNEL];
	uint32_t list_len;
	uint8_t flag;	/*0: disable 1: enable*/
	uint8_t dir;	/*0: set 1: get*/
};

/**
 * Each channel's Radar status and detected history records
 */
typedef struct _qcsapi_radar_status {
	/**
	 * Which channel to be queried. It must be DFS channel.
	 */
	uint32_t channel;
	/**
	 * If This API returns without error, it indicates the whether the channel is in non-occupy
	 * list currently.
	 */
	uint32_t flags;
	/**
	 * This records times radar signal is detected on this channel.
	 */
	uint32_t ic_radardetected;
}qcsapi_radar_status;

/**
 * Connection and Disconnecttion count information
 */
typedef struct _qcsapi_disconn_info {
	/**
	 * This indicates number of stations connect to this device.
	 */
	uint32_t asso_sta_count;
	/**
	 * Count of disconnect event.
	 */
	uint32_t disconn_count;
	/**
	 * Sequence to query disconnect count.
	 */
	uint32_t sequence;
	/**
	 * Time elapses since device boot up.
	 */
	uint32_t up_time;
	/**
	 * If resetflag is set to TRUE, member disconn_count and sequence will be set to 0.
	 */
	uint32_t resetflag;
}qcsapi_disconn_info;

/**
 * Retrieve values of tx_power on all antennas for calcmd
 */
typedef struct _qcsapi_calcmd_tx_power_rsp {
	uint32_t value[QCSAPI_QDRV_NUM_RF_STREAMS];
}qcsapi_calcmd_tx_power_rsp;

/**
 * Retrieve values of rssi on all antennas for calcmd
 */
typedef struct _qcsapi_calcmd_rssi_rsp{
	int32_t value[QCSAPI_QDRV_NUM_RF_STREAMS];
}qcsapi_calcmd_rssi_rsp;

typedef enum {
	qcsapi_extender_role = 1,
	qcsapi_extender_mbs_best_rssi,
	qcsapi_extender_rbs_best_rssi,
	qcsapi_extender_mbs_wgt,
	qcsapi_extender_rbs_wgt,
	qcsapi_extender_verbose,
	qcsapi_extender_roaming,
	qcsapi_extender_bgscan_interval,
	qcsapi_extender_mbs_rssi_margin,
	qcsapi_extender_short_retry_limit,
	qcsapi_extender_long_retry_limit,
	qcsapi_extender_scan_mbs_intvl,
	qcsapi_extender_scan_mbs_expiry,
	qcsapi_extender_scan_mbs_mode,
	qcsapi_extender_fast_cac,
	qcsapi_extender_nosuch_param = 0
} qcsapi_extender_type;

typedef enum {
	qcsapi_eth_info_connected	= 0x00000001,
	qcsapi_eth_info_speed_unknown	= 0x00000002,
	qcsapi_eth_info_speed_10M	= 0x00000004,
	qcsapi_eth_info_speed_100M	= 0x00000008,
	qcsapi_eth_info_speed_1000M	= 0x00000010,
	qcsapi_eth_info_speed_10000M	= 0x00000020,
	qcsapi_eth_info_duplex_full	= 0x00000040,
	qcsapi_eth_info_autoneg_on	= 0x00000080,
	qcsapi_eth_info_autoneg_success	= 0x00000100,
	qcsapi_eth_info_speed_2500M	= 0x00000200,
	qcsapi_eth_info_unknown		= 0
} qcsapi_eth_info_result;

typedef enum {
	qcsapi_eth_info_link_mask	= qcsapi_eth_info_connected,
	qcsapi_eth_info_speed_mask	= qcsapi_eth_info_speed_10M \
					| qcsapi_eth_info_speed_100M \
					| qcsapi_eth_info_speed_2500M \
					| qcsapi_eth_info_speed_1000M \
					| qcsapi_eth_info_speed_10000M \
					| qcsapi_eth_info_speed_unknown,
	qcsapi_eth_info_duplex_mask	= qcsapi_eth_info_duplex_full,
	qcsapi_eth_info_autoneg_mask	= qcsapi_eth_info_autoneg_on \
					| qcsapi_eth_info_autoneg_success,
	qcsapi_eth_info_all_mask	= qcsapi_eth_info_link_mask \
					| qcsapi_eth_info_speed_mask \
					| qcsapi_eth_info_duplex_mask \
					| qcsapi_eth_info_autoneg_mask
} qcsapi_eth_info_type_mask;

typedef enum {
	qcsapi_eth_info_start = 1,
	qcsapi_eth_info_link = qcsapi_eth_info_start,
	qcsapi_eth_info_speed,
	qcsapi_eth_info_duplex,
	qcsapi_eth_info_autoneg,
	qcsapi_eth_info_all,
	qcsapi_eth_nosuch_type = 0
} qcsapi_eth_info_type;

typedef enum {
	qcsapi_interface_status_error,
	qcsapi_interface_status_disabled,
	qcsapi_interface_status_up,
	qcsapi_interface_status_running,
} qcsapi_interface_status_code;

/**
 * Basic measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_basic_s {
	/**
	 * offset to start measurement, based on microsecond
	 */
	uint16_t offset;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * channel to execute the measurement, based on IEEEE channel number
	 */
	uint8_t channel;
};

/**
 * CCA measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_cca_s {
	/**
	 * offset to start measurement, based on microsecond
	 */
	uint16_t offset;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * channel to execute the measurement, based on IEEEE channel number
	 */
	uint8_t channel;
};

/**
 * RPI measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_rpi_s {
	/**
	 * offset to start measurement, based on microsecond
	 */
	uint16_t offset;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * channel to execute the measurement, based on IEEEE channel number
	 */
	uint8_t channel;
};

/**
 * Channel Load measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_chan_load_s {
	/**
	 * operating class, with channel and region to decide which frequency to execute
	 */
	uint8_t op_class;
	/**
	 * IEEE channel number, with operating class and region to decide which frequency to execute
	 */
	uint8_t channel;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
};

/**
 * Noise histogram measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_noise_his_s {
	/**
	 * operating class, with channel and region to decide which frequency to execute
	 */
	uint8_t op_class;
	/**
	 * IEEE channel number, with operating class and region to decide which frequency to execute
	 */
	uint8_t channel;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
};

/**
 * Beacon measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_beacon_s {
	/**
	 * operating class, with channel and region to decide which frequency to execute
	 */
	uint8_t op_class;
	/**
	 * IEEE channel number, with operating class and region to decide which frequency to execute
	 */
	uint8_t channel;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * beacon measurement mode, 0 passive, 1 active, 2 table
	 */
	uint8_t mode;
	/**
	 * specified bssid for beacon measurement
	 */
	uint8_t bssid[6];
};

/**
 * Frame measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_frame_s {
	/**
	 * operating class, with channel and region to decide which frequency to execute
	 */
	uint8_t op_class;
	/**
	 * IEEE channel number, with operating class and region to decide which frequency to execute
	 */
	uint8_t channel;
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * frame type, currently only frame count report(1) is supported
	 */
	uint8_t type;
	/**
	 * specified mac address for frame measurement
	 */
	uint8_t mac_address[6];
};

/**
 * Transmit stream/category measurement parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_tran_steam_cat_s {
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * specified mac_address for measurement
	 */
	uint8_t peer_sta[6];
	/**
	 * traffic ID
	 */
	uint8_t tid;
	/**
	 * bin 0
	 */
	uint8_t bin0;
};

/**
 * Multicast diagnostics report parameter
 *
 * \sa _qcsapi_measure_request_param
 */
struct qcsapi_measure_multicast_diag_s {
	/**
	 * duration to do the measurement, based on microsecond
	 */
	uint16_t duration;
	/**
	 * specified group mac_address for measurement
	 */
	uint8_t group_mac[6];
};

/**
 * Request parameter union for 11h and 11k measurement
 *
 * \sa qcsapi_wifi_get_node_param
 */
typedef union _qcsapi_measure_request_param {
	/**
	 * basic measurement parameter
	 */
	struct qcsapi_measure_basic_s basic;
	/**
	 * CCA measurement parameter
	 */
	struct qcsapi_measure_cca_s cca;
	/**
	 * RPI measurement parameter
	 */
	struct qcsapi_measure_rpi_s rpi;
	/**
	 * Channel Load measurement parameter
	 */
	struct qcsapi_measure_chan_load_s chan_load;
	/**
	 * Noise histogram measurement parameter
	 */
	struct qcsapi_measure_noise_his_s noise_his;
	/**
	 * Beacon measurement parameter
	 */
	struct qcsapi_measure_beacon_s beacon;
	/**
	 * Frame measurement parameter
	 */
	struct qcsapi_measure_frame_s frame;
	/**
	 * transmit stream/category measurement
	 */
	struct qcsapi_measure_tran_steam_cat_s tran_stream_cat;
	/**
	 * multicast diagnostics report
	 */
	struct qcsapi_measure_multicast_diag_s multicast_diag;
} qcsapi_measure_request_param;


/**
 * Neighbor report item
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_neighbor_item_s {
	uint8_t bssid[6];
	uint32_t bssid_info;
	uint8_t operating_class;
	uint8_t channel;
	uint8_t phy_type;
};

/**
 * Transmit power control report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_tpc_s {
	int8_t link_margin;
	int8_t tx_power;
};

/**
 * Noise histogram measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_noise_histogram_s {
	uint8_t antenna_id;
	uint8_t anpi;
	uint8_t ipi[11];
} ;

/**
 * Beacon measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_beacon_s {
	uint8_t rep_frame_info;
	uint8_t rcpi;
	uint8_t rsni;
	uint8_t bssid[6];
	uint8_t antenna_id;
	uint32_t parent_tsf;
};

/**
 * Frame measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_frame_s {
	uint32_t sub_ele_report;
	uint8_t ta[6];
	uint8_t bssid[6];
	uint8_t phy_type;
	uint8_t avg_rcpi;
	uint8_t last_rsni;
	uint8_t last_rcpi;
	uint8_t antenna_id;
	uint16_t frame_count;
};

/**
 * Transmit stream/category report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_tran_stream_cat_s {
	uint8_t reason;
	uint32_t tran_msdu_cnt;
	uint32_t msdu_discard_cnt;
	uint32_t msdu_fail_cnt;
	uint32_t msdu_mul_retry_cnt;
	uint32_t qos_lost_cnt;
	uint32_t avg_queue_delay;
	uint32_t avg_tran_delay;
	uint8_t bin0_range;
	uint32_t bins[6];
};

/**
 * Multicast diagnostics report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_multicast_diag_s {
	uint8_t reason;
	uint32_t mul_rec_msdu_cnt;
	uint16_t first_seq_num;
	uint16_t last_seq_num;
	uint16_t mul_rate;
};

/**
 * TPC measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_tpc_report_s {
	int8_t tx_power;
	int8_t link_margin;
};

/**
 * Link measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_link_measure_s {
	struct qcsapi_measure_rpt_tpc_report_s tpc_report;
	uint8_t recv_antenna_id;
	uint8_t tran_antenna_id;
	uint8_t rcpi;
	uint8_t rsni;
};

/**
 * Neighbor measurement report
 *
 * \sa _qcsapi_measure_report_result
 */
struct qcsapi_measure_rpt_neighbor_report_s {
	uint8_t item_num;
	struct qcsapi_measure_neighbor_item_s items[3];
};

/**
 * Report results for 11h and 11k measurement
 *
 * \sa qcsapi_wifi_get_node_param
 */
typedef union _qcsapi_measure_report_result {
	/**
	 * Common place to store results if no specified
	 */
	int common[16];
	/**
	 * Transmit power control report
	 */
	struct qcsapi_measure_rpt_tpc_s tpc;
	/**
	 * Basic measurement report
	 */
	uint8_t basic;
	/**
	 * CCA measurement report
	 */
	uint8_t cca;
	/**
	 * RPI measurement report
	 */
	uint8_t rpi[8];
	/**
	 * Channel Load measurement report
	 */
	uint8_t channel_load;
	/**
	 * Noise histogram measurement report
	 */
	struct qcsapi_measure_rpt_noise_histogram_s noise_histogram;
	/**
	 * Beacon measurement report
	 */
	struct qcsapi_measure_rpt_beacon_s beacon;
	/**
	 * Frame measurement report
	 */
	struct qcsapi_measure_rpt_frame_s frame;
	/**
	 * Transmit stream/category report
	 */
	struct qcsapi_measure_rpt_tran_stream_cat_s tran_stream_cat;
	/**
	 * Multicast diagnostics report
	 */
	struct qcsapi_measure_rpt_multicast_diag_s multicast_diag;
	/**
	 * Link measurement
	 */
	struct qcsapi_measure_rpt_link_measure_s link_measure;
	/**
	 * Neighbor report
	 */
	struct qcsapi_measure_rpt_neighbor_report_s neighbor_report;
} qcsapi_measure_report_result;

#define QCSAPI_MAX_MACS_IN_LIST		200
#define QCSAPI_MAX_MACS_SIZE	1200 /* 200 macs */
/**
 * Report results of get mac address of clients behind associated node
 *
 * \sa qcsapi_get_mac_client_list
 */
struct qcsapi_mac_list {
	/**
	 * flags indicating
	 * bit 0 set means addresses are behind 4 addr node
	 * bit 1 means results are truncated to fit to buffer
	 */
	uint32_t flags;
	/**
	 * num entries in the macaddr list below
	 */
	uint32_t num_entries;
	/**
	 * buffer to store mac addresses
	 */
	uint8_t macaddr[QCSAPI_MAX_MACS_SIZE];
};

#define QCSAPI_NUM_ANT QCSAPI_QDRV_NUM_RF_STREAMS_PLUS1
struct qcsapi_sample_assoc_data {
	qcsapi_mac_addr mac_addr;
	uint8_t assoc_id;
	uint8_t bw;
	uint8_t tx_stream;
	uint8_t rx_stream;
	uint32_t time_associated;	/*Unit: seconds*/
	uint32_t achievable_tx_phy_rate;
	uint32_t achievable_rx_phy_rate;
	uint32_t rx_packets;
	uint32_t tx_packets;
	uint32_t rx_errors;
	uint32_t tx_errors;
	uint32_t rx_dropped;
	uint32_t tx_dropped;
	uint32_t tx_wifi_drop[WMM_AC_NUM];
	uint32_t rx_ucast;
	uint32_t tx_ucast;
	uint32_t rx_mcast;
	uint32_t tx_mcast;
	uint32_t rx_bcast;
	uint32_t tx_bcast;
	uint16_t link_quality;
	uint32_t ip_addr;
	uint64_t rx_bytes;
	uint64_t tx_bytes;
	uint32_t last_rssi_dbm[QCSAPI_NUM_ANT];
	uint32_t last_rcpi_dbm[QCSAPI_NUM_ANT];
	uint32_t last_evm_dbm[QCSAPI_NUM_ANT];
	uint32_t last_hw_noise[QCSAPI_NUM_ANT];
	uint8_t protocol;
	uint8_t vendor;
}__packed;

/**
 * \brief Structure containing CCA statistics.
 *
 * This structure is used as a return parameter in the per-interface APIs
 * associated with CCA statistics gathering.
 *
 * \sa qcsapi_get_cca_stats
 */
typedef struct _qcsapi_cca_stats
{
	/**
	 * Percentage of channel occupied by actvities
	 */
	uint32_t	cca_occupy;
	/**
	 * Percentage of air time which is occupied by other APs and STAs except the local AP/STA
	 * and associated STAs/AP
	 */
	uint32_t	cca_intf;
	/**
	 * Percentage of the air time which is occupied by the local AP/STA and the associated
	 * STAs/AP
	 */
	uint32_t	cca_trfc;
	/**
	 * Percentage of air time which is occupied by the local AP/STA in transmission
	 */
	uint32_t	cca_tx;
	/**
	 * Percentage of air time which is occupied by the local AP/STA in reception
	 */
	uint32_t	cca_rx;
} qcsapi_cca_stats;


#define QCSAPI_MAX_IE_INFOLEN	255

/**
 * \brief Structure containing data of IE, the number of octets in the Information field and buffer
 * with Information.
 *
 * This structure is used as a parameter in qcsapi_wifi_get_wps_ie_scanned_AP
 *
 * \sa qcsapi_wifi_get_wps_ie_scanned_AP
 */
struct qcsapi_ie_data {
	uint8_t ie_len;
	uint8_t ie_buf[QCSAPI_MAX_IE_INFOLEN];
};

/**
 * \brief Packet type
 *
 * \sa qcsapi_packet_type
 */
typedef enum {
	/* management packet */
	qcsapi_packet_type_mgmt = 0,
	/* control packet */
	qcsapi_packet_type_control = 1,
	/* data packet */
	qcsapi_packet_type_data = 2,
} qcsapi_packet_type;

#define QCSAPI_MAX_NAC_STATS	128
/**
 * Non associated clients stat
 *
 * \sa qcsapi_nac_stats_entry
 */
typedef struct {
	/* average RSSI for all received chains */
	int8_t			average_rssi;
	/* last received channel */
	uint8_t			channel;
	/* last type of packet received */
	qcsapi_packet_type	packet_type;
	/* time stamp of last packet received */
	uint64_t		timestamp;
	/* mac address of non associated client */
	qcsapi_mac_addr		txmac;
} qcsapi_nac_stats_entry;

/**
 * Non associated clients stats report
 *
 * \sa qcsapi_nac_stats_report
 */
typedef struct {
	/* number of valid entries in the report */
	uint8_t num_valid_entries;
	/* non associated client stats list */
	qcsapi_nac_stats_entry stats[QCSAPI_MAX_NAC_STATS];
} qcsapi_nac_stats_report;

/**
 * \brief Enumeration to represent configuration grabber blob section type value.
 *
 * \sa qcsapi_grab_config
 * \sa struct qcsapi_grab_info_cmn
 */
typedef enum {
	QCSAPI_GRAB_INFO_TYPE_QCSAPI = 0,
	QCSAPI_GRAB_INFO_TYPE_FILE,
	QCSAPI_GRAB_INFO_TYPE_MEM
} qcsapi_grab_info_type;

/**
 * \brief Enumeration to represent configuration grabber blob section status value.
 *
 * \sa qcsapi_grab_config
 * \sa struct qcsapi_grab_info_cmn
 */
typedef enum {
	QCSAPI_GRAB_INFO_STATUS_OK = 0,
	QCSAPI_GRAB_INFO_STATUS_TRUNCATED,
	QCSAPI_GRAB_INFO_STATUS_WARNING,
	QCSAPI_GRAB_INFO_STATUS_ERROR
} qcsapi_grab_info_status;

/**
 * Configuration grabber blob magic.
 *
 * \sa struct qcsapi_grab_info_hdr
 */
#define QCSAPI_GRAB_INFO_MAGIC	{'Q', 'T', 'N', 'A', 0x56, 0x24, 0x72, 0x73}

/**
 * Configuration grabber blob version.
 *
 * \sa struct qcsapi_grab_info_hdr
 */
#define QCSAPI_GRAB_INFO_VERSION	1

/**
 * Configuration grabber blob header.
 *
 * \sa qcsapi_grab_config
 */
struct qcsapi_grab_info_hdr {
	/* configuration grabber blob magic */
	uint8_t magic[8];
	/* configuration grabber blob version */
	uint32_t version;
	/* blob header length */
	uint32_t header_len;
	/* offset of file md5sum, zero if not used */
	uint64_t hash_offset;
	/* blob creation time in seconds from epoch */
	uint64_t creation_time;
} __packed;

/**
 * Configuration grabber common section header.
 *
 * \sa qcsapi_grab_config
 */
struct qcsapi_grab_info_cmn {
	/* section type qcsapi_grab_info_type */
	uint16_t type;
	/* section status qcsapi_grab_info_status */
	uint16_t status;
	/* section header length */
	uint32_t header_len;
	/* section content length */
	uint64_t len;
} __packed;

/**
 * Configuration grabber QCSAPI section header.
 *
 * \sa qcsapi_grab_config
 */
struct qcsapi_grab_info_qcsapi {
	/* common section header */
	struct qcsapi_grab_info_cmn cmn;
	/* length of call qcsapi argument string */
	uint16_t qcsapi_str_len;
} __packed;

/**
 * Configuration grabber file section header.
 *
 * \sa qcsapi_grab_config
 */
struct qcsapi_grab_info_file {
	/* common section header */
	struct qcsapi_grab_info_cmn cmn;
	/* file path string len with NULL byte included */
	uint16_t fpath_len;
} __packed;

/**
 * Configuration grabber memory section header.
 *
 * \sa qcsapi_grab_config
 */
struct qcsapi_grab_info_mem {
	/* common section header */
	struct qcsapi_grab_info_cmn cmn;
	/* memory address */
	uint32_t addr;
	/* words count to read */
	uint32_t count;
} __packed;

/**
 * Flag for grabbing section header only.
 *
 * \sa qcsapi_grab_config
 */
#define QCSAPI_GRAB_ONLY_INFO_FL	0x0001

/** @} */

/**
 * @addtogroup BootConfigAPIs
 * @{
 */

/**
 * \brief Retrieve a parameter from bootcfg environment
 *
 * Read a u-boot environment parameter from the bootcfg driver, which manages persistent u-boot
 * environment variables.
 *
 * \param param_name Name of parameter being requested
 * \param param_value Result storage for the value of the parameter
 * \param max_param_len size of the buffer passed in <i>param_value</i>
 *
 * \return \zero_or_negative
 * \return -ENODATA if the parameter is not found
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bootcfg_param \<param_name\></c>
 *
 * Output will be the value of the requested environment variable on success, or an error message on
 * failure.
 */
extern int qcsapi_bootcfg_get_parameter(const char *param_name,
		char *param_value,
		const size_t max_param_len);
/**
 * \brief Persist a parameter in bootcfg environment flash
 *
 * Write a u-boot environment parameter to the bootcfg driver. Bootcfg driver will handle writing
 * the new parameter to persistent storage.
 *
 * \param param_name Name of parameter being set
 * \param param_value Value of parameter to be set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_bootcfg_param \<param_name\> \<param_value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_bootcfg_update_parameter(const char *param_name, const char *param_value);
/**
 * \brief Update uboot binary variable with regular file in the file system
 *
 * Read a regular binary file from file system into temporary buffer as input data, then create a
 * binary variable in the uboot environment with data if the dst variable doesn't exist,
 * or update the existing binary variable.
 *
 * \param dst uboot bootcfg file name in uboot environment
 * \param path_src source file name in regular file system
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_bootcfg_binfile \<dst\> \<path_src\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_update_bootcfg_binfile(const char *dst, const char *path_src);

/**
 * \brief Sync bootcfg updates to flash
 *
 * This function can be called after making changes to bootcfg with
 * <c>qcsapi_bootcfg_update_parameter</c>, to ensure that all pending updates
 * have been committed to flash.
 *
 * \note This call will block until the flash has been written back. Generally
 * this call will complete immediately with interactive use of call_qcsapi,
 * and is used during production for ensuring scripts complete write of the
 * bootcfg parameters prior to calling board reboot.
 *
 * \return 0 when all pending updates have been committed to flash
 * \return \negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi commit_bootcfg</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_bootcfg_commit(void);

/** @} */

/**
 * @addtogroup ServicesAPIs
 * @{
 */

extern int qcsapi_telnet_enable(const qcsapi_unsigned_int onoff);

/**
 * \brief Used to find service enum
 *
 * This is internally used in service_control
 */
extern int qcsapi_get_service_name_enum(const char *lookup_service, qcsapi_service_name *serv_name);

/**
 * \brief Used to find service action enum
 *
 * This is internally used in service control
 */
extern int qcsapi_get_service_action_enum(const char *lookup_action,
						qcsapi_service_action *serv_action);

/**
 * \brief Start, stop, enable or disable the services.
 *
 * Turn service on or off.
 *
 * \param service
 * \param action start/stop/enable/disable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi service_control \<service_name\> \<start/stop/enable/disable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_service_control(qcsapi_service_name service, qcsapi_service_action action);

/**
 * \brief Enable and disable features that are not needed for WFA testing
 *
 * Turn WFA certification mode on or off.
 *
 * \param enable a value turn the WFA certification mode on/off
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wfa_cert {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wfa_cert_mode_enable(uint16_t enable);

/** @} */

/**
 * @addtogroup SCSAPIs
 * @{
 */

/**
 * \brief Returns the channels during the last channel change event.
 *
 * Retrieve the previous channel and the current channel during the
 * last channel change event.
 *
 * \param ifname \wifi0
 * \param p_prev_channel the channel before the channel change.
 * \param p_cur_channel the channel after the channel change.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_cce_channels \<WiFi interface\></c>
 *
 * Unless an error occurs, the previous and current channel will be displayed.
 */
extern int qcsapi_wifi_get_scs_cce_channels(const char *ifname, qcsapi_unsigned_int *p_prev_channel,
					qcsapi_unsigned_int *p_cur_channel);

/**
 * \brief Turn SCS feature on/off.
 *
 * Turn the SCS feature on or off.
 *
 * \param ifname \wifi0
 * \param enable_val a value turn the feature on/off
 *
 * \return \zero_or_negative
 *
 * \note This API is not supported when DBVC is enabled
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_scs \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_scs_enable(const char *ifname, uint16_t enable_val);

/**
 * \brief Set SCS version.
 *
 * Select version of SCS module. Version 0 corresponds to the legacy SCS.
 * Version 1 is the new AACS channel and bandwidth selection mechanism.
 *
 * \param ifname \wifi0
 * \param version 0 or 1
 *
 * \return 0 on success or negative values on error
 *
 * \callqcsapi
 *
 * <c>call_qcsapi scs_set_version \<WiFi interface\> {0 | 1}</c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 */
extern int qcsapi_wifi_scs_set_version(const char *ifname, uint16_t ver);

/**
 * \brief Trigger SCS switch channel manually.
 *
 * Trigger SCS switch channel manually.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param pick_flags flag used for channel selection
 * Flags used as channel set are defined as below and they are mutually-exclusive:
 * @code
 * IEEE80211_SCS_PICK_DFS_ONLY			0x0001
 * IEEE80211_SCS_PICK_NON_DFS_ONLY		0x0002
 * IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY	0x0004
 * IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL	0x0008
 * IEEE80211_SCS_PICK_ANYWAY			0x0010
 * IEEE80211_SCS_NOPICK				0x8000
 * @endcode
 * The header file <c>net80211/ieee80211_dfs_reentry.h</c> including this macros comes
 * with the package libqcsapi_client_src.zip.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi scs_switch_chan \<WiFi interface\> \<pick flags\> \<check margin\></c>
 *
 * Where <c>\<pick flags\></c> should be "dfs", "non_dfs" or "all".
 * This parameter indicates that what kind of channels to be selected.
 * With using "dfs", It will pick channel from available dfs channels.
 * With using "non-dfs", it will pick channel from available non-dfs channels.
 * "all" is default which means it will pick channel from all available channels.
 *
 * Where <c>\<check margin\></c> should be "0" or "1" which means whether to pick channel
 * with checking the margin or not.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_scs_switch_channel(const char *ifname, uint16_t pick_flags);

/**
 * \brief Get the best channel by SCS channel ranking algorithm.
 *
 * Get the best channel by SCS channel ranking algorithm but not to change channel.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param pick_flags flag used for channel selection
 * Flags used as channel set are defined as below and they are mutually-exclusive:
 * @code
 * IEEE80211_SCS_PICK_DFS_ONLY			0x0001
 * IEEE80211_SCS_PICK_NON_DFS_ONLY		0x0002
 * IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY	0x0004
 * IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL	0x0008
 * IEEE80211_SCS_PICK_ANYWAY			0x0010
 * IEEE80211_SCS_NOPICK				0x8000
 * @endcode
 * The header file <c>net80211/ieee80211_dfs_reentry.h</c> including this macros comes
 * with the package libqcsapi_client_src.zip.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi scs_pick_chan \<WiFi interface\> \<pick flags\> \<check margin\></c>
 *
 * Where <c>\<pick flags\></c> should be "dfs", "non_dfs" or "all".
 * This parameter indicates that what kind of channels to be selected.
 * With using "dfs", It will pick channel from available dfs channels.
 * With using "non-dfs", it will pick channel from available non-dfs channels.
 * "all" is default which means it will pick channel from all available channels.
 *
 * Where <c>\<check margin\></c> should be "0" or "1" which means whether to pick channel
 * with checking the margin or not.
 *
 * Unless an error occurs, the output will be the channel number</c>.
 */
extern int qcsapi_wifi_scs_pick_best_channel(const char *ifname, uint16_t pick_flags, int *channel);

/**
 * \internal
 * \brief Turn SCS feature's verbose information output on/off.
 *
 * Turn the SCS feature's verbose information output on or off.
 *
 * \param ifname \wifi0
 * \param enable_val a value turn the verbose information output on/off
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_verbose \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_verbose(const char *ifname, uint16_t enable_val);

/**
 * \brief Get the current enabled state of SCS feature.
 *
 * Return the current enabled status of the SCS feature. It could be either
 * enabled or disabled.
 *
 * \param ifname \wifi0
 * \param p_scs_status value that contains if SCS is enabled or disabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_status \<WiFi interface\></c>
 *
 * The output will be the word "Disabled" or "Enabled" unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_status(const char *ifname, qcsapi_unsigned_int *p_scs_status);

/**
 * \internal
 * \brief Turn SCS feature's channel sampling on/off.
 *
 * Turn the SCS feature's channel sampling on or off.
 *
 * \param ifname \wifi0
 * \param enable_val a value turn the channel sampling on/off
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_smpl_enable \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_smpl_enable(const char *ifname, uint16_t enable_val);

/**
 * \brief Enable or disable SCS channels
 *
 * Enable or disable channels for Smart Channel Selection (SCS)
 *
 * \param ifname \wifi0only
 * \param ch_list buffer containing a list of channel numbers
 * \param enable_flag \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_active_chan_list \<WiFi interface\> \<Channel list\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * Example:
 * @code
 * call_qcsapi set_scs_active_chan_list wifi0_0 108,112,116 0
 * @endcode
 */
int qcsapi_wifi_set_scs_active_chan_list(const char *ifname,
		const struct qcsapi_scs_chan_list *ch_list, uint8_t enable_flag);

/**
 * \brief Get list of SCS-enabled channels
 *
 * Get the list of channels on which SCS is currently enabled
 *
 * \param ifname \wifi0only
 * \param ch_list \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_active_chan_list \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will contain the number of SCS channels
 * followed by a space-separated list SCS channels.
 */
int qcsapi_wifi_get_scs_active_chan_list(const char *ifname, struct qcsapi_scs_chan_list *ch_list);

/**
 * \internal
 * \brief Set the duration for sampling the busyness of a channel.
 *
 * API sets the dwell time for the scs feature ie. the duration in
 * which the busyness of the channel is sampled.
 * Unit is in milliseconds.
 *
 * \param ifname \wifi0
 * \param scs_sample_time Time during which busyness is sampled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_smpl_dwell_time \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_smpl_dwell_time(const char *ifname, uint16_t scs_sample_time);

/**
 * \internal
 * \brief Set the interval between two samples for SCS feature.
 *
 * API sets the sample interval for the SCS feature. This duration indicates
 * the duration to wait after which the next off-channel sampling session starts.
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param scs_sample_intv Time from the previous sample to the next sample.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_smpl_intv \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_sample_intv(const char *ifname, uint16_t scs_sample_intv);

/**
 * \internal
 * \brief Set the off channel sampling type for SCS feature.
 *
 * Set the sample type for the SCS feature.
 *
 * \param ifname \wifi0
 * \param scs_sample_type Sample type for SCS off channel sampling.
 * - QTN_OFF_CHAN_FLAG_MULTI_SLOTS_FAST - schedule twice every beacon interval, total 6
 *				 times per channel
 * - QTN_OFF_CHAN_FLAG_MULTI_SLOTS_NORMAL - schedule once every beacon interval, total 6
 *				times per channel
 * - QTN_OFF_CHAN_FLAG_MULTI_SLOTS_SLOW - schedule once every two beacon intervals, total
 *				6 times per channel
 * - QTN_OFF_CHAN_FLAG_ONESHOT - schedule once only per channel
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_smpl_type \<WiFi interface\> \<type\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_sample_type(const char *ifname, uint16_t scs_sample_type);

/**
 * \internal
 * \brief Set the interval between two interference detection for SCS feature.
 *
 * API sets the interference detection interval for the SCS feature. This duration indicates
 * the duration to wait after which the next interference detection session starts.
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param scs_intf_detect_intv Time from the previous interference detection to the next
 *	interference detection.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_intf_detect_intv \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_intf_detect_intv(const char *ifname, uint16_t scs_intf_detect_intv);

/**
 * \internal
 * \brief Set the threshold values for SCS feature.
 *
 * API sets the threshold for various parameters that control the
 * SCS feature. Threshold affects the sensitivity of the feature.
 *
 * \param ifname \wifi0
 * \param scs_param_name The threshold by name which is to be set
 * \param scs_threshold The value of the threshold to be set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_threshold \<WiFi interface\> \<threshold_name\> \<value\></c>
 *
 * \param threshold_name is one of "smpl_pktnum", "smpl_airtime", "intf_low", "intf_high",
 * "intf_ratio", "dfs_margin", "cca_idle", "pmbl_err", "atten_inc", "dfs_reentry",
 * "dfs_reentry_minrate". The unit of "dfs_reentry_minrate" is 100kbps.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_thrshld(const char *ifname, const char *scs_param_name,
					uint16_t scs_threshold);

/**
 * \internal
 * \brief Set if SCS feature should only report.
 *
 * Control whether SCS should change the channel upon making a decison, or just report it.
 *
 * \param ifname \wifi0
 * \param scs_report_only value that indicates if SCS feature should act
 * on thresholds or only report.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_report_only \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_report_only(const char *ifname, uint16_t scs_report_only);

/**
 * \brief Get the channel evaluation result for SCS feature.
 *
 * \brief Get the channel evaluation result for SCS feature.
 * Statistics such as channel, channel metric are returned.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param scs_rpt return the channel evaluation result.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_report \<WiFi interface\> all</c>
 *
 * The output will be stats containing channel, channel metric unless an error occurs.
 * The follwing parameters will be reported for each channel
 * - <c>chan</c> - The channel number
 * - <c>dfs</c> - Whether the channel is a DFS channel or not
 * - <c>txpower</c> - The transmit power
 * - <c>numbeacon</c> - The beacon count
 * - <c>cca_intf</c> - The CCA interference parameter
 * - <c>metric</c> - The SCS ranking metric
 * - <c>pmbl_ap</c> - The preamble errors detected by the AP
 * - <c>pmbl_sta</c> - The preamble errors detected by stations
 * - <c>age</c> - The age of channel metric in seconds
 * - <c>duration</c> - Number of seconds the channel was in use
 * - <c>times</c> - Number of times the channel was used
 * - <c>status</c> - The channel availability status
 */
extern int qcsapi_wifi_get_scs_stat_report(const char *ifname,
					struct qcsapi_scs_ranking_rpt *scs_rpt);

/**
 * \brief Get the channel evaluation result of interference.
 *
 * Reports the evaluation result for all channels of the interference.
 * Statistics such as channel, channel inteference are returned.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param scs_rpt return the channel interference evaluation result.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_report \<WiFi interface\> interference</c>
 *
 * The output will be stats containing channel, interferences unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_interference_report(const char *ifname,
					struct qcsapi_scs_interference_rpt *scs_rpt);

/**
 * \brief Get the channel evaluation with scoring way for SCS feature.
 *
 * Report the scores of all channels for the SCS feature.
 * Statistics such as channel, score are returned.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param scs_rpt return the channel score result.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_report \<WiFi interface\> score</c>
 *
 * The output will be stats containing channel, score unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_score_report(const char *ifname,
					struct qcsapi_scs_score_rpt *scs_rpt);

/**
 * \brief Get current channel's stats for SCS feature.
 *
 * Reports the statistics for the current channel for the SCS feature.
 * Statistics such as channel, cca interference are returned.
 *
 * \param ifname \wifi0
 * \param scs_currchan_rpt return the current channel's stats.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_report \<WiFi interface\> current</c>
 *
 * The output will be stats containing channel, cca interference unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_currchan_report(const char *ifname,
					struct qcsapi_scs_currchan_rpt *scs_currchan_rpt);

/**
 * \internal
 * \brief Start/Stop SCS stats task.
 *
 * Start/Stop the SCS stats task.
 *
 * \param ifname \wifi0
 * \param start a value for start/stop indication
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_stats \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_stats(const char *ifname, uint16_t start);

/**
 * \brief Get the initial auto channel evaluation result.
 *
 * Report the initial channel evalution result for Auto Channel feature.
 * Statistics such as channel, channel metric are returned.
 *
 * \param ifname \wifi0
 * \param autochan_rpt the initial channel evaluation result.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_report \<WiFi interface\> autochan</c>
 *
 * The output will be stats containing channel, channel metric unless an error occurs.
 */
extern int qcsapi_wifi_get_autochan_report(const char *ifname,
					struct qcsapi_autochan_rpt *autochan_rpt);

/**
 * \internal
 * \brief Set smoothing factor for SCS CCA interference measurement.
 *
 * Control the degree SCS smooths sequential cca interference measurement.
 *
 * \param ifname \wifi0
 * \param scs_cca_intf_smth_fctr_noxp value that indicates the smoothing factor
 * for channels once used as working channel.
 * \param scs_cca_intf_smth_fctr_xped value that indicates the smoothing factor
 * for channels never used as working channel.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_cca_intf_smth_fctr \<WiFi interface\> \<smth_fctr_noxp\>
 *	<smth_fctr_xped\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_cca_intf_smth_fctr(const char *ifname, uint8_t smth_fctr_noxp,
							uint8_t smth_fctr_xped);

/**
 * \internal
 * \brief Set channel metric margin for SCS channel ranking.
 *
 * Control the channel metric margin SCS used for channel ranking.
 *
 * \param ifname \wifi0
 * \param chan_mtrc_mrgn value that indicates the channel metric margin.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_chan_mtrc_mrgn \<WiFi interface\> \<chan_mtrc_mrgn\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_chan_mtrc_mrgn(const char *ifname, uint8_t chan_mtrc_mrgn);

/**
 * \brief Configure the monitor mode feature both on operational channels and off channels.
 *
 * Configure the monitor mode feature both on operational channels and off channels.
 *
 * When enabled, monitor mode is activated when SCS switches to an off-channel and deactivated
 * when switching back to the operational channel. While on the operational channel, monitor
 * mode is activated in a duty cycle.
 *
 * \note This API is for configuring both off-channel and on-channel monitor mode. To
 * configure only on-channel monitor mode, use \ref qcsapi_wifi_set_nac_mon_mode.
 *
 * \note This API is not supported when DBVC is enabled
 *
 * \param ifname \wifi0only
 * \param on_period on period as a pecentage between 1 and 99 (default 1%)
 * \param cycle_period cycle period in milliseconds between 100 and 5000 (default 1000)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_nac_monitor_mode \<WiFi interface\> {0 | 1} [on period] [cycle period]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_nac_monitor_mode(const char *ifname, uint32_t enable,
					uint32_t on_period, uint32_t cycle_period);

/**
 * \brief Get the specified channel's CCA interference level.
 *
 * Get the CCA interference level for a particular channel.
 * The interference value will be a integer value from -1 to 1000. -1 means no
 * CCA interference data is available, and other values represent CCA interference levels.
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the CCA interference is returned.
 * \param p_cca_intf return parameter to contain the CCA interference.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_cca_intf \<WiFi interface\> \<channel\></c>
 *
 * Unless an error occurs, the output will be the CCA interference status.
 */
extern int qcsapi_wifi_get_scs_cca_intf(const char *ifname,
			const qcsapi_unsigned_int the_channel, int *p_cca_intf);

/**
 * \brief Get the configured SCS parameters.
 *
 * Controls whether to check the band metric margin SCS used for channel ranking.
 *
 * \param ifname \wifi0only
 * \param enable \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_band_margin_check \<WiFi interface\> \<enable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_band_margin_check(const char *ifname, uint8_t enable);

/**
 * \internal
 * \brief Configure band metric margin for SCS channel ranking.
 *
 * Configure the band metric margin SCS used for channel ranking.
 *
 * \param ifname \wifi0only
 * \param margin band switch metric margin
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_band_margin_check \<WiFi interface\> <margin></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_band_margin(const char *ifname, uint8_t margin);

/**
 * \brief Get the configured SCS parameters.
 *
 * Get the configured SCS parameters.
 *
 * \replaced_by \ref qcsapi_wifi_get_scs_params.
 *
 * \param ifname \wifi0
 * \param p_scs_param_rpt return parameter to contain the SCS parameters
 * \param param_num list of parameter numbers to be returned
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_params \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will include current SCS parameters.
 */
int qcsapi_wifi_get_scs_param_report(const char *ifname,
			struct qcsapi_scs_param_rpt *p_scs_param_rpt, uint32_t param_num);

/**
 * \brief Get the configured SCS parameters.
 *
 * Gets the configured SCS parameters.
 *
 * \param ifname \wifi0
 * \param p_scs_param_rpt return parameter to contain the SCS parameters.
 * \param rpt_buf_len the length of p_scs_param_rpt buffer.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_params \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will include current SCS parameters.
 */
extern int qcsapi_wifi_get_scs_params(const char *ifname,
			struct qcsapi_data_1Kbytes *p_scs_param_rpt, uint32_t rpt_buf_len);
/**
 * \brief Get the list of SCS channels.
 *
 * Get the list of SCS channels with each value separated by commas.
 *
 * Use <c>qcsapi_wifi_get_scs_chan_pool</c> to get the list of SCS channels
 *
 * \param ifname \wifi0
 * \param list_of_channels buffer to contain the list of comma-separated channels
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_chan_pool \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of WiFi channels per the 802.11 standard.
 */
extern int qcsapi_wifi_get_scs_chan_pool(const char *ifname, string_1024 list_of_channels);

/**
 * \internal
 * \brief Get the current state of SCS DFS Re-entry request.
 *
 * Return the current status of the SCS DFS Re-entry request. It could be either
 * 0(not requested) or 1(requested).
 *
 * \param ifname \wifi0
 * \param p_scs_dfs_reentry_request value that contains the request level.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_dfs_reentry_request \<WiFi interface\></c>
 *
 * The output will be an integer representing the request level unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_dfs_reentry_request(const char *ifname,
					qcsapi_unsigned_int *p_scs_dfs_reentry_request);

/**
 * \internal
 * \brief Set the enable flag.
 *
 * API sets the enable flag to enable/disable burst channel switching
 *
 * \param ifname \wifi0only
 * \param enable_flag The value of the enable flag
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_burst_enable \<WiFi interface\> \<1 or 0\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_burst_enable(const char *ifname, uint16_t enable_flag);

/**
 * \internal
 * \brief Set the sliding window of time in minutes.
 *
 * API sets the sliding window of time used for life time of the burst channel switching event
 *
 * \param ifname \wifi0only
 * \param window The value of the sliding window, unit: minute
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_burst_window \<WiFi interface\> \<window\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_burst_window(const char *ifname, uint16_t window);

/**
 * \internal
 * \brief Set the burst channel switching threshold.
 *
 * API sets the burst channel switching threshold during sliding window
 *
 * \param ifname \wifi0only
 * \param threshold The value of the channel switching threshold
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_burst_thresh \<WiFi interface\> \<threshold\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_burst_thresh(const char *ifname, uint16_t threshold);

/**
 * \internal
 * \brief Set the pause time in minutes.
 *
 * API sets the time to pause when burstly channenl switching is above threshold, unit: minute
 *
 * \param ifname \wifi0only
 * \param pause_time The value of the pause time, unit: minute
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_burst_pause_time \<WiFi interface\> \<pause_time\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_burst_pause(const char *ifname, uint16_t pause_time);

/**
 * \internal
 * \brief Set the switch flag.
 *
 * API sets the switch flag to enable/disable channel switching after pausing specified time
 *
 * \param ifname \wifi0only
 * \param switch_flag The value of the switch flag
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_burst_force_switch \<WiFi interface\> \<1 or 0\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_burst_switch(const char *ifname, uint16_t switch_flag);

/**
 * \internal
 * \brief Get the duration for sampling the busyness of a channel.
 *
 * API Gets the dwell time for the scs feature ie. the duration in
 * which the busyness of the channel is sampled.
 * Unit is in milliseconds.
 *
 * \param ifname \wifi0
 * \param scs_sample_time buffer to store time during which busyness is sampled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_smpl_dwell_time \<WiFi interface\></c>
 *
 * The output will be an integer representing the sample duration unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_smpl_dwell_time(const char *ifname,
					qcsapi_unsigned_int *scs_sample_time);

/**
 * \internal
 * \brief Get the interval between two samples for SCS feature.
 *
 * API gets the sample interval for the SCS feature. This duration indicates
 * the duration to wait after which the next off-channel sampling session starts.
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param scs_sample_intv Buffer to store time from the previous sample to the next sample.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scs_smpl_intv \<WiFi interface\></c>
 *
 * The output will be an integer representing the sample interval unless an error occurs.
 */
extern int qcsapi_wifi_get_scs_sample_intv(const char *ifname,
					qcsapi_unsigned_int *scs_sample_intv);

/**
 * \brief Set the list of SCS channels.
 *
 * Set the list of SCS channels and one byte containing one channel
 *
 * Use <c>qcsapi_wifi_set_scs_chan_pool</c> to set the list of SCS channels
 *
 * \param ifname \wifi0
 * \param chans buffer to contain the list of channels and one byte containing one channel
 * \param cnt counter of channels
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_chan_pool \<WiFi interface\> \<channel list\></c>
 *
 * Unless an error occurs, the output will be the list of WiFi channels per the 802.11 standard.
 */
extern int qcsapi_wifi_set_scs_chan_pool(const char *ifname,
				const struct qcsapi_data_256bytes *chans, const uint32_t cnt);

/**
 * \brief Set the ACS parameter.
 *
 * Set the ACS parameter.
 *
 * \param ifname \wifi0
 * \param acs_param the acs parameters.See the definition of the enum \ref qcsapi_acs_param_type.
 * \param acs_param_value pointer to the parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_acs_param \<wifi_interface\> \<acs_param\> \<acs_param_value\></c>
 *
 * @code
 * call_qcsapi set_acs_param 0 bss_chk 1
 * @endcode
 *
 * \call_qcsapi_string_complete
 *
 */
extern int qcsapi_set_acs_params(const char *ifname, const qcsapi_acs_param_type acs_param,
					const qcsapi_unsigned_int acs_param_value);

/** @} */

/**
 * @addtogroup DFSAPIs
 * @{
 */

/**
 * \brief Start off-channel CAC.
 *
 * Start off-channel CAC on a DFS channel.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_start_dfs_s_radio.
 *
 * \param ifname \wifi0
 * \param channel specifies the DFS channel for CAC. 0 is to select DFS channel automatically.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_ocac \<WiFi interface\> {auto | \<DFS_channel\>}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_start_ocac(const char *ifname, uint16_t channel);

/**
 * \brief Stop off-channel CAC.
 *
 * Stop off-channel CAC.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_stop_dfs_s_radio.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi stop_ocac \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_stop_ocac(const char *ifname);

/**
 * \brief Get the current state of off-channel CAC.
 *
 * Get the current state of off-channel CAC.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_dfs_s_radio_status.
 *
 * \param ifname \wifi0
 * \param status value that contains if OCAC is enabled or disabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ocac_status \<WiFi interface\></c>
 *
 * The output will be the word "Disabled" or "Enabled" unless an error occurs.
 */
extern int qcsapi_wifi_get_ocac_status(const char *ifname, qcsapi_unsigned_int *status);

/**
 * \internal
 * \brief Set the dwell time on off-channel for off-channel CAC.
 *
 * API sets the dwell time for the off-channel CAC feature, ie. the duration on
 * off channel within a beacon interval.
 * Unit is in milliseconds.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_dfs_s_radio_dwell_time.
 *
 * \param ifname \wifi0
 * \param dwell_time Dwell time on off-channel in a beacon interval.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ocac_dwell_time \<WiFi interface\> \<dwelltime\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ocac_dwell_time(const char *ifname, uint16_t dwell_time);

/**
 * \internal
 * \brief Set the duration during which off-channel CAC is running for a DFS channel.
 *
 * API sets the duration during which the off-channel CAC is running for a specified
 * DFS channel
 * Unit is in seconds.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_dfs_s_radio_duration.
 *
 * \param ifname \wifi0
 * \param duration Duration for a specified DFS channel to run off-channel CAC.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ocac_duration \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ocac_duration(const char *ifname, uint16_t duration);

/**
 * \internal
 * \brief Set the total time on off channel for a DFS channel.
 *
 * API sets the total time on off channel for a specified DFS channel
 * Unit is in seconds.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_dfs_s_radio_cac_time.
 *
 * \param ifname \wifi0
 * \param cac_time total time on the specified DFS channel.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ocac_cac_time \<WiFi interface\> \<cac_time\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ocac_cac_time(const char *ifname, uint16_t cac_time);

/**
 * \internal
 * \brief Set the off-channel CAC report only mode.
 *
 * API sets the off-channel CAC as report only mode, that means, don't switch channel
 * after off-channel CAC is completed if report only mode is set.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_dfs_s_radio_report_only.
 *
 * \param ifname \wifi0
 * \param enable 0 - disable report only mode, otherwise enable it.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ocac_report_only \<WiFi interface\> \<1 or 0\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ocac_report_only(const char *ifname, uint16_t enable);

/**
 * \internal
 * \brief Set the threshold values for OCAC feature.
 *
 * API sets the threshold for various parameters that control the off-channel CAC
 * feature. Threshold affects the sensitivity of the feature.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_dfs_s_radio_thrshld.
 *
 * \param ifname \wifi0
 * \param param_name The threshold by name which is to be set
 * \param threshold The value of the threshold to be set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ocac_thrshld \<WiFi interface\> \<threshold_name\> <value\></c>
 * threshold name is one of "fat", "traffic" and "cca_intf".
 * "fat" means the free air time, and the threshold value is the percentage for the free air time.
 * off-channel CAC can run when the current FAT is larger than this threshold.
 * "traffic" is the traffic of local BSS, and the threshold value is the percentage for the local
 * traffic time against the measurement time. off-channel CAC can run when the local traffic is
 * less than the threshold value.
 * "cca_intf" means the cca interference on off channel. AP can switch to the DFS channel after off
 * channel CAC only when no radar is detected on this DFS channel and the cca interference on DFS
 * channel is less than the threshold, which is the percentage for the interference traffic time
 * against the measurement time.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ocac_thrshld(const char *ifname, const char *param_name,
					uint16_t threshold);

/**
 * \brief Start DFS seamless entry.
 *
 * This API is used to start DFS seamless entry on a DFS channel.
 *
 * \param ifname \wifi0
 * \param channel specifies the DFS channel. 0 is to select DFS channel automatically.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_dfs_s_radio \<WiFi interface\>
 *		{ {auto | \<test_DFS_channel\>} | auto \<first_DFS_channel\>}</c>
 *
 * \param auto the best DFS channel is selected for initiating SDFS
 * \param test_DFS_channel the channel on which SDFS will be initiated
 * \param first_DFS_channel initiates the SDFS Auto DFS feature, where the driver will try to
 *	initiate SDFS on this channel, then fall back to auto mode, where changing to a
 *	non_DFS_channel restarts SDFS
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_start_dfs_s_radio(const char *ifname, uint16_t channel);

/**
 * \brief Stop DFS seamless entry.
 *
 * This API is used to stop DFS seamless entry.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi stop_dfs_s_radio \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_stop_dfs_s_radio(const char *ifname);

/**
 * \brief Get the current status of DFS seamless entry.
 *
 * This API return the current status of whether DFS seamless entry is started or not.
 *
 * \param ifname \wifi0
 * \param status value that contains if DFS seamless entry is enabled or not.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dfs_s_radio_status \<WiFi interface\></c>
 *
 * The output will be the word "Disabled" or "Enabled" unless an error occurs.
 */
extern int qcsapi_wifi_get_dfs_s_radio_status(const char *ifname, qcsapi_unsigned_int *status);

/**
 * \brief Get the current availability of DFS seamless entry.
 *
 * This API return the status of whether DFS seamless entry is available or not with the
 * current configuration.
 *
 * \param ifname \wifi0
 * \param available value that contains if DFS seamless entry is available or unavailable.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dfs_s_radio_availability \<WiFi interface\></c>
 *
 * The output will be the word "Available" or "Unavailable" unless an error occurs.
 */
extern int qcsapi_wifi_get_dfs_s_radio_availability(const char *ifname,
					qcsapi_unsigned_int *available);

/**
 * \internal
 * \brief Set the dwell time on off-channel for DFS seamless entry.
 *
 * API sets the dwell time for the DFS seamless entry feature, ie. the duration on
 * off channel within a beacon interval.
 * Unit is in milliseconds.
 *
 * \param ifname \wifi0
 * \param dwell_time Dwell time on off-channel in a beacon interval.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_dwell_time \<WiFi interface\> \<dwelltime\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_dwell_time(const char *ifname, uint16_t dwell_time);

/**
 * \internal
 * \brief Set the duration during which DFS seamless entry is running for a DFS channel.
 *
 * API sets the duration during which the DFS seamless entry is running for a specified
 * DFS channel
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param duration Duration for a specified DFS channel to run DFS seamless entry.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_duration \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_duration(const char *ifname, uint16_t duration);

/**
 * \internal
 * \brief Set the duration during which DFS seamless entry is running for a weather channel.
 *
 * API sets the duration during which the DFS seamless entry is running for a specified
 * weather channel which is a DFS channel.
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param duration Duration for a specified DFS channel to run DFS seamless entry.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_wea_duration \<WiFi interface\> \<duration\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_wea_duration(const char *ifname, uint32_t duration);

/**
 * \internal
 * \brief Set the total time on off channel for a DFS channel.
 *
 * API sets the total time on off channel for a specified DFS channel
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param cac_time total time on the specified DFS channel.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_cac_time \<WiFi interface\> \<cac_time\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_cac_time(const char *ifname, uint16_t cac_time);

/**
 * \internal
 * \brief Set the total time on off channel for a weather channel.
 *
 * API sets the total time on off channel for a specified weather channel which is a DFS channel.
 * Unit is in seconds.
 *
 * \param ifname \wifi0
 * \param cac_time total time on the specified DFS channel.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_wea_cac_time \<WiFi interface\> \<cac_time\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_wea_cac_time(const char *ifname, uint32_t cac_time);

/**
 * \internal
 * \brief Set the dwell time on off-channel for a weather channel.
 *
 * API sets the dwell time for a specified weather channel, ie. the duration on
 * off channel within a beacon interval.
 * Unit is in milliseconds.
 *
 * \param ifname \wifi0
 * \param dwell_time Dwell time on the specified weather channel in a beacon interval.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_wea_dwell_time \<WiFi interface\> \<dwelltime\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_wea_dwell_time(const char *ifname, uint16_t dwell_time);

/**
 * \internal
 * \brief Set the DFS seamless entry report only mode.
 *
 * API sets the DFS seamless entry as report only mode, that means, don't switch channel
 * after DFS seamless entry is completed if report only mode is set.
 *
 * \param ifname \wifi0
 * \param enable 0 - disable report only mode, otherwise enable it.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_report_only \<WiFi interface\> \<1 or 0\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_report_only(const char *ifname, uint16_t enable);

/**
 * \internal
 * \brief Set the threshold values for DFS seamless entry.
 *
 * API sets the threshold for various parameters that control the DFS seamless entry.
 * Threshold affects the sensitivity of the feature.
 *
 * \param ifname \wifi0
 * \param param_name The threshold by name which is to be set
 * \param threshold The value of the threshold to be set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_thrshld \<WiFi interface\> \<threshold_name\> <value\></c>
 * threshold name is one of "fat", "traffic" and "cca_intf".
 * "fat" means the free air time, and the threshold value is the percentage for the free air time.
 * DFS seamless entry can run when the current FAT is larger than this threshold.
 * "traffic" is the traffic of local BSS, and the threshold value is the percentage for the local
 * traffic time against the measurement time. DFS seamless entry can run when the local traffic is
 * less than the threshold value.
 * "cca_intf" means the cca interference on off channel. AP can switch to the DFS channel after DFS
 * seamless entry only when no radar is detected on this DFS channel and the cca interference on DFS
 * channel is less than the threshold, which is the percentage for the interference traffic time
 * against the measurement time.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_thrshld(const char *ifname, const char *param_name,
						uint16_t threshold);

/** @} */

/**
 * @addtogroup APIInitializationAPIs
 * @{
 */

/**
 * \brief Call to initialise the runtime for QCSAPI usage.
 *
 * This function is required to be called prior to any other QCSAPI function.
 *
 * To prevent concurrency issues, a multi-threaded application should call this function
 * prior to creating additional threads.
 *
 * \return \zero_or_negative
 *
 * \note This function does not have a <c>call_qcsapi</c> equivalent call due to its nature.
 */
extern int qcsapi_init(void);

/**
 * \brief Disconnect the program from the calling terminal.
 *
 * Often a process needs to run in background.	Such a process is described as a daemon process.
 * This API will force the process into background mode and disconnect from the controlling terminal
 * or console.
 * When such a process is run from the command line the effect of calling this API is immediately
 * apparent, as the command line prompt will appear and another command can then be entered.
 *
 * Use this API for any process that starts as the WiFi device boots, and is expected to remain
 * active until the device is halted or rebooted (eg, a long-running daemon process).
 *
 * \return \zero_or_negative
 *
 * \note This function does not have a <c>call_qcsapi</c> equivalent call due to its nature.
 */
extern int qcsapi_console_disconnect(void);

/** @} */

/**
 * @addtogroup SystemAPIs
 * @{
 */

/**
 * \brief Initialise the device
 *
 * Manually initialise the device after initial boot, which includes task such as
 * - bring up interfaces
 * - start wireless drivers
 * - start security daemons
 * - start user-space applications
 *
 * Initialisation may be performed automatically or via this API, depending on the device
 * configuration. This API can be used:
 * - for stateless mode, where there is no persistent storage on the device and config must be
 *	installed prior to initialisation
 * - when the security agent is enabled, where config files must be decrypted prior to
 *	initialisation
 * - when autostart is disabled for any othe reason
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi startprod</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_startprod(void);

/**
 * \brief Get the initialisation status of the device
 *
 * Get the initialisation status of the device
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_startprod_done</c>
 *
 * Unless an error occurs, the output will be "1" if start-up is complete, or "0" if start-up is not
 * yet complete.
 */
extern int qcsapi_is_startprod_done(int *p_status);

/**
 * \brief Get the time since the system started up.
 *
 * This function is used to determine system uptime.
 *
 * \param p_elapsed_time return parameter to store the system uptime (number of seconds since boot).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_time_since_start</c>
 *
 * The output will be the time in seconds since the system started up, or an error string.
 */
extern int qcsapi_system_get_time_since_start(qcsapi_unsigned_int *p_elapsed_time);


/**
 * \brief Get system status.
 *
 * This function is used to get system status.
 *
 * \param p_status return parameter to store the system status.
 * It is in bit-mask format, each bit represents different status.
 * Possiable values are defined here @ref QCSAPI_GET_SYSTEM_STATUS
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_sys_status</c>
 */
extern int qcsapi_get_system_status(qcsapi_unsigned_int *p_status);

/**
 * \brief Get pseudorandom data from /dev/urandom.
 *
 * Get pseudorandom data from /dev/urandom. It can be used to store random seed across reboots.
 *
 * \param random_buf 512 bytes buffer to store random data
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_random_seed</c>
 *
 * The output will be 512 bytes of random data from /dev/urandom similar to <c>cat /dev/urandom</c>
 */
extern int qcsapi_get_random_seed(struct qcsapi_data_512bytes *random_buf);

/**
 * \brief Feed Linux PRNG with new seed.
 *
 * Add random buffer to Linux PRNG entropy pool as well as update entropy count with an estimation
 * of added entropy.
 *
 * \param random_buf 512 bytes buffer of random data
 * \param entropy added entropy estimation
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_random_seed \<random_string\> \<entropy_count\></c>
 *
 * \call_qcsapi_string_complete
 *
 * \note The command line interface is not suitable for setting binary data A better way is to read
 * the random data from a file.
 */
extern int qcsapi_set_random_seed(const struct qcsapi_data_512bytes *random_buf,
					const qcsapi_unsigned_int entropy);

/**
 * \brief Get the carrier ID
 *
 * Get the configured carrier ID.
 *
 * \param p_carrier_id
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_carrier_id</c>
 *
 * Unless an error occurs, the output will be the carrier ID.
 */
extern int qcsapi_get_carrier_id(qcsapi_unsigned_int *p_carrier_id);

/**
 * \brief Set the carrier ID
 *
 * \brief Set the carrier ID.
 *
 * \param carrier_id
 * \param update_uboot 1 write the carrier ID back to uboot env, else 0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_carrier_id \<carrier_id\> [\<update_uboot\>]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_set_carrier_id(uint32_t carrier_id, uint32_t update_uboot);

/**
 * \brief get platform ID
 *
 * This API call is used to retrieve current platform ID.
 *
 * \param p_platform_id
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_platform_id</c>
 *
 * Unless an error occurs, the output will be the platform ID.
 */
extern int qcsapi_get_platform_id(qcsapi_unsigned_int *p_platform_id);

/**
 * \brief get spinor jedecid.
 *
 * This API get the spinor flash jedec id.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be the jedec id.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_spinor_jedecid \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_spinor_jedecid(const char * ifname, unsigned int * p_jedecid);

/**
 * \brief get bb param.
 *
 * This API get the bb param.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be the status of lock detect function enabled .
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bb_param\<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_bb_param(const char * ifname, unsigned int * p_jedecid);
/**
 * \brief set bb param.
 *
 * This API set the bb param.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be <c>1</c> (enabled) or <c>0</c> (disabled)
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bb_param \<WiFi interface\></c>
 */
extern int qcsapi_wifi_set_bb_param(const char * ifname, const qcsapi_unsigned_int p_jedecid);

/**
 * \brief enable rx optim packet stats.
 *
 * This API enable rx optim packet stats.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be <c>1</c> (enabled) or <c>0</c> (disabled)
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_optim_stats \<WiFi interface\></c>
 */
extern int qcsapi_wifi_set_optim_stats(const char * ifname, const qcsapi_unsigned_int p_jedecid);

/**
 * \brief set system time
 *
 * This API sets system time.
 *
 * \param timestamp seconds since the epoch, 1970-01-01 00:00:00 +0000 (UTC)
 *
 * \return \positive_or_negative
 *
 * \call_qcsapi_string_complete
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_sys_time \<seconds since epoch\></c>
 */
extern int qcsapi_wifi_set_sys_time(const uint32_t timestamp);

/**
 * \brief get system time
 *
 * This API gets system time.
 *
 * \param timestamp buffer for returned timestamp
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be the current system time in seconds since the Epoch,
 * 1970-01-01 00:00:00 +0000 (UTC).
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_sys_time</c>
 */
extern int qcsapi_wifi_get_sys_time(uint32_t *timestamp);

/**
 * @brief Set the MAC address of the device.
 *
 * Set the MAC address of the device.
 *
 * \param ifname the interface
 * \param soc_mac_addr new MAC address
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_soc_macaddr \<WIFI interface\> \<soc mac addr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_set_soc_mac_addr(const char *ifname, const qcsapi_mac_addr soc_mac_addr);

/**
 * @brief Get a custom value.
 *
 * This API returns the value of a custom key contained in a file named
 * <c>/etc/custom/<i>key</i></c>.
 *
 * \note The filename must not have the substring '..' as any part or the filename
 * will be rejected as invalid.
 *
 * \param custom_key The custom key name
 * \param custom_value A buffer in which to store the returned value, which must be at least 129
 * bytes. The value will be truncated if longer than 128 characters.
 *
 * \return 0 on success
 * \return -qcsapi_configuration_error if the custom key has not been defined
 * \return -qcsapi_configuration_error if the key includes the string ".."
 * \return -qcsapi_configuration_error if the custom key value is an empty string
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_custom_value \<key\></c>
 *
 * Unless an error occurs, the output will be the custom key value.
 */
extern int qcsapi_get_custom_value(const char *custom_key, string_128 custom_value);

/**
 * @brief Set a custom value.
 *
 * This API sets the value of a custom key contained in a file named <c>/etc/custom/<i>key</i></c>.
 *
 * \note The filename must not have the substring '..' as any part or the filename
 * will be rejected as invalid.
 *
 * \param custom_key The custom key name
 * \param custom_value A pointer to string to put into custom key file, which must be at most 128
 * bytes. An empty string can be used to delete existing key file.
 *
 * \return \zero_or_negative
 * \return -qcsapi_configuration_error if the custom key has not been defined.
 * \return -qcsapi_configuration_error if the key includes the string "..".
 * \return -qcsapi_configuration_error if the custom key value is an empty string
 * and the key file doesn't exist.
 * \return -qcsapi_configuration_error if the custom key value is longer than 128 bytes.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_custom_value \<key\> \<value\></c>
 */
extern int qcsapi_set_custom_value(const char *custom_key, const char *custom_value);

/**
 * \brief Get default BSS state
 *
 * Get the default state (enabled or disabled) of a BSS.
 *
 * \replaced_by \ref qcsapi_radio_wifi_get_vap_default_state.
 *
 * \param enable buffer to return the default state. 1: enabled, 0: disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vap_default_state</c>
 *
 * Unless an error occurs, the output will be the default VAP state.
 */
extern int qcsapi_wifi_get_vap_default_state(int *enable);

/**
 * \brief Get default BSS state
 *
 * Get the default state (enabled or disabled) of a BSS.
 *
 * \param radio_id \radio_id
 * \param enable buffer to return the default state. 1: enabled, 0: disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vap_default_state \<radio\></c>
 *
 * Unless an error occurs, the output will be the default VAP state.
 */
extern int qcsapi_radio_wifi_get_vap_default_state(qcsapi_unsigned_int radio_id, int *enable);

/**
 * \brief Set default BSS state
 *
 * Set the default state (enabled or disabled) of a BSS.
 *
 * \replaced_by \ref qcsapi_radio_wifi_set_vap_default_state.
 *
 * \param enable the default BSS state - 1: enabled, 0: disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vap_default_state \<enable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_vap_default_state(const int enable);

/**
 * \brief Set default BSS state
 *
 * Set the default state (enabled or disabled) of a BSS.
 *
 * \param enable the default BSS state - 1: enabled, 0: disabled
 * \param radio_id \radio_id
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vap_default_state \<radio\> \<enable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_radio_wifi_set_vap_default_state(qcsapi_unsigned_int radio_id, const int enable);

/**
 * @brief set the hardware module state
 *
 * Set the hardware module state: on or off
 *
 * \param module specify which hardware module to be controlled
 * \param flag specify state of hardware module
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_hw_module_state \<WiFi interface\> \<module\> {0 | 1}</c>
 *
 * \sa qcsapi_hw_module
 */
extern int qcsapi_qdrv_set_hw_module_state(qcsapi_hw_module module, qcsapi_unsigned_int flag);

/**
 * @brief get the hardware module state
 *
 * get the specified hardware module state: on or off
 *
 * \param module specify which hardware module
 * \param flag pointer to buffer holding state of hardware module
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hw_module_state \<WiFi interface\> \<module\></c>
 *
 * \sa qcsapi_hw_module
 */
extern int qcsapi_qdrv_get_hw_module_state(qcsapi_hw_module module, qcsapi_unsigned_int *p_flag);

/**
 * \brief Restore the default configuration files.
 *
 * This API call restores most configuration files to their default configuration.
 *
 * The default behaviour is as follows.
 * - some configuration files, including wireless_conf.txt, admin.conf and
 *	security daemons are restored to the default version for the current image
 * - most other configuration files are deleted
 * - the WiFi mode for each radio is left unchanged
 * - the device's IP address is left unchanged
 * - the device is rebooted after configuration files have been restored
 *
 * \param flag An optional bitmap parameter which can be used to modify the default behaviour. It
 * may contain any combination of the following macros.
 * - QCSAPI_RESTORE_FG_IP - restore the IP address
 * - QCSAPI_RESTORE_FG_NOREBOOT - do not reboot
 * - QCSAPI_RESTORE_FG_AP - restore to AP mode
 * - QCSAPI_RESTORE_FG_STA - restore to Station mode
 * - QCSAPI_RESTORE_FG_MODE_OFFSET - radio offset, where 0 represents radio 0, etc.
 * - QCSAPI_RESTORE_FG_MODE_BITS - mode bitwidth
 * - QCSAPI_RESTORE_FG_MODE_MASK - mode mask
 *
 * \note Refer to the call_qcsapi implementation for an example of how to apply these macros.
 *
 * \return 0 This call always succeeds.
 *
 * \warning The no-reboot option is not guaranteed to fully restore the factory default environment
 * and processes. Rebooting is recommended.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi restore_default_config [ {ap | sta | \<WiFi interface\> {ap | sta} ...} ]
 *	[ip] [noreboot]</c>
 *
 * \param ap restore to AP mode
 * \param sta restore to Station mode
 * \param <WiFi interface> restore only the specified primary interface
 * \param ip restore the IP address
 * \param noreboot do not reboot
 *
 * \warning This call will replace all customised configuration with default settings.
 */
extern int qcsapi_restore_default_config(int flag);

/** @} */

/**
 * @addtogroup ParameterConfAPIs
 * @{
 */

/**
 * \brief Get a persistent configuration parameter
 *
 * Get a persistent configuration parameter.
 *
 * \param ifname the name of the interface for which configuration parameter value is queried, or
 *	<c>global</c> for system-wide parameters.
 * \param param_name the parameter to be retrieved
 * \param param_value a pointer to the buffer for storing the returned value
 * \param max_param_len the size of the buffer
 *
 * \return \zero_or_negative
 * \return <c>-qcsapi_parameter_not_found</c> if the parameter was not found.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_config_param \<WiFi interface\> \<param_name\></c>
 *
 * or
 *
 * <c>call_qcsapi get_persistent_param \<WiFi interface\> \<parameter name\></c>
 *
 * The output will be the parameter value unless an error occurs or the parameter is not found.
 */
extern int qcsapi_config_get_parameter(const char *ifname, const char *param_name,
					char *param_value, const size_t max_param_len);

/**
 * \brief Create or update the value for a persistent configuration parameter
 *
 * Update the value for a persistent configuration parameter. The parameter is added if it does not
 * already exist.
 *
 * \param ifname the name of the interface for which configuration parameter is set, or
 *	<c>global</c> for system-wide parameters.
 * \param param_name the parameter to be created or updated, from any of the following tables.
 * * supported_radio_parameters_tbl
 * * supported_global_parameters_tbl
 * * supported_parameters_tbl
 * \param param_value a pointer to a buffer containing the null-terminated parameter value string
 *
 * \return \zero_or_negative
 *
 * \note The device must be rebooted for these changes to take effect.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_config_param \<WiFi interface\> \<param_name\> \<value\></c>
 *
 * or
 *
 * <c>call_qcsapi update_persistent_param \<WiFi interface\> \<param_name\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 *
 * <b>Examples</b>
 *
 * To set the WiFi mode to AP, enter:
 *
 * <c>call_qcsapi update_persistent_param wifi0_0 mode ap</c>
 *
 * To set the initial WiFi channel to 36 (only effective on an AP), enter:
 *
 * <c>call_qcsapi update_persistent_param wifi0_0 channel 36</c>
 *
 * To disable automatic wireless interfaces bringup on device startup, enter:
 *
 * <c>call_qcsapi update_persistent_param global autostart</c>
 *
 */
extern int qcsapi_config_update_parameter(const char *ifname, const char *param_name,
						const char *param_value);
/** @} */

/**
 * @addtogroup ParameterConfAPIs
 * @{
 */

/**
 * \brief Get an MBSS persistent configuration parameter
 *
 * Get a MBSS persistent configuration parameter.
 *
 * \param ifname \wifi0
 * \param param_name the parameter to be retrieved
 * \param param_value a pointer to the buffer for storing the returned value
 * \param max_param_len the size of the buffer
 *
 * \return \zero_or_negative
 * \return <c>-qcsapi_parameter_not_found</c> if the parameter was not found.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_persistent_ssid_param \<WiFi interface\> \<param_name\></c>
 *
 * The output will be the parameter value unless an error occurs or the parameter is not found.
 */
extern int qcsapi_config_get_ssid_parameter(const char *ifname, const char *param_name,
					char *param_value, const size_t max_param_len);
/**
 * \brief Update an MBSS persistent config parameter
 *
 * Set a persistent configuration parameter. The parameter is added if it does not already exist.
 *
 * All real work for this API is done in the update_per_ssid_config script, as explained above.
 *
 * Unlike most APIs, this one will spawn a subprocess.	If for any reason the process
 * table is full, this API will fail with unpredictable results.
 *
 * \param ifname \wifi0
 * \param param_name the parameter to be created or updated
 * \param param_value a pointer to a buffer containing the null-terminated parameter value
 * string
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_persistenti_ssid_param \<WiFi interface\> \<param_name\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 *
 * <b>Examples</b>
 *
 * <c>call_qcsapi update_persistent_ssid_param wifi0_0 priority 3</c>
 *
 * /note The device must be rebooted for these changes to take effect.
 */
extern int qcsapi_config_update_ssid_parameter(const char *ifname, const char *param_name,
						const char *param_value);
/** @} */

/**
 * @addtogroup FilePathAPIs
 * @{
 */

/**
 * \brief Get the configuration file path.
 *
 * Reports the location associated with the configuration file path for the security files.
 *
 * \param e_file_path should be the symbolic value <c>qcsapi_security_configuration_path</c>
 * \param file_path return parameter to contain the configuration file path to the security files.
 * \param path_size the size of the file_path parameter above.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_file_path security</c>
 *
 * The command should be entered exactly as shown above. The output is the current configured path
 * to the security files.
 */
extern int qcsapi_file_path_get_config(const qcsapi_file_path_config e_file_path,
					char *file_path, qcsapi_unsigned_int path_size);

/**
 * @brief Update the location associated with the configuration file path.
 *
 * Updates the location associated with the file path configuration.
 *
 * \param e_file_path Use the symbolic value <c>qcsapi_security_configuration_path</c>.
 * \param new_path A NULL terminated string for the new path to set.
 *
 * \return \zero_or_negative
 *
 * \note The presence or validity of the file path is NOT checked.
 *
 * \note This API is only available in calibration mode (see @ref mysection4_1_5 "Production mode vs
 * calibration mode").
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_file_path security \<new location\></c>
 *
 * As this is a set API, the output from call_qcsapi is complete, unless an error occurs.
 *
 * \warning <b>Power should not be turned off to the WiFi device when calling the set file path
 * config API or immediately afterwards. Failure to follow this restriction can cause the flash
 * memory on the board to get corrupted. If power needs to be turned off to the WiFi device when
 * working with this API, enter the "halt" command first and wait for the device to shut down.
 *
 * \warning This API should only be called when initially configuring the board.</b>
 */
extern int qcsapi_file_path_set_config(const qcsapi_file_path_config e_file_path,
						const char *new_path);

/** @} */

/**
 * @addtogroup NetInterfaceAPIs
 * @{
 */

/**
 * \brief Stores IP address in a persistent storage.
 *
 * Stores IP address of the primary bridge interface in a file system. It will be used on reboot
 * or on stateless initialization after startprod invocation.
 *
 * \param ipaddr IPv4 address
 * \param netmask Network mask
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi store_ipaddr ipaddr/netmask</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_store_ipaddr(qcsapi_unsigned_int ipaddr, qcsapi_unsigned_int netmask);

/**
 * @brief Enable or disable an interface.
 *
 * Enable (or disable) an interface. Use <c>qcsapi_interface_get_status</c> to establish
 * whether an interface is enabled or disabled.
 *
 * \param ifname \wifi0
 * \param enable_flag if 0, disable the interface, else enable the interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_interface \<network interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * Examples:
 *
 * To enable the ethernet interface eth1_0:
 *
 * <c>call_qcsapi enable_interface eth1_0 1</c>
 *
 * To disable the WiFi interface wifi0_0:
 *
 * <c>call_qcsapi enable_interface wifi0_0 0</c>
 *
 * \sa qcsapi_interface_get_status
 */
extern int qcsapi_interface_enable(const char *ifname, const int enable_flag);

/**
 * @brief Get an interface status.
 *
 * Determine whether an interface is enabled (up) or disabled (down).
 *
 * \param ifname \wifi0
 * \param interface_status return parameter to indicate the interface status.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_status \<network interface\></c>
 *
 * Output is one of the words <c>up</c>, <c>disabled</c> or <c>error</c>; or an error message.
 *
 * Examples:
 *
 * To report the ethernet interface eth1_0, enter:
 *
 * <c>call_qcsapi get_status eth1_0</c>
 *
 * To report the WiFi interface status of wifi0_0, enter:
 *
 * <c>call_qcsapi get_status wifi0_0</c>
 */
extern int qcsapi_interface_get_status(const char *ifname, char *interface_status);

/**
 * @brief Set the IP address or netmask for an interface.
 *
 * Set the IP address or netmask for an interface.
 *
 * \param ifname
 * \param if_param "ipaddr" to set the IP address, or "netmask" to set the netmask
 * \param if_param_val IP address or netmask
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ip \<network interface\> {ipaddr \<IP address\> | netmask \<netmask\>}</c>
 *
 * \call_qcsapi_string_complete
 *
 * To Set the IP address of br0 interface, enter:
 *
 * <c>call_qcsapi set_ip br0 ipaddr ip_val</c>
 */
extern int qcsapi_interface_set_ip4(const char *ifname, const char *if_param,
					uint32_t if_param_val);

/**
 * @brief Get an interface IP Address and netmask.
 *
 * Determine IP address and netmask of the interface.
 *
 * \param ifname
 * \param if_param "ipaddr" to set the IP address, or "netmask" to set the netmask
 * \param if_param_val \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ip \<network interface\> {ipaddr | netmask}</c>
 *
 * Output is IP address or netmask of the interface, or an error message.
 *
 * To get netmask of br0 interface, enter:
 *
 * <c>call_qcsapi get_ip br0 netmask</c>
 *
 * To get the IP address and netmask of bridge interface br0, enter:
 *
 * <c>call_qcsapi get_ip br0</c>
 */
extern int qcsapi_interface_get_ip4(const char *ifname, const char *if_param,
					string_64 if_param_val);

/**
 * @brief Get statistics from an interface.
 *
 * This API call returns statistics for a given interface, and given counter.
 *
 * Per the TR-098 standard, all counters are represented as 32-bit unsigned integers.
 *
 * \note Be aware that rollover is quite possible, especially with counters reporting the number of
 *	bytes transferred.
 *
 * \param ifname \wifi0
 * \param qcsapi_counter one of the enumerations from the typedef struct qcsapi_counter_type (see
 *	the following table).
 * \param p_counter_value return parameter to contain the counter value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_counter \<network device\> \<counter name\></c>
 *
 * Valid counter names are listed in the following table:
 *
 * <TABLE>
 * <TR> <TD>call_qcsapi parameter name</TD>
 *	<TD>enum value</TD>
 *	<TD>Description</TD></TR>
 * <TR> <TD>tx_bytes</TD>
 *	<TD>qcsapi_total_bytes_sent</TD>
 *	<TD>Bytes transmitted</TD></TR>
 * <TR> <TD>rx_bytes</TD>
 *	<TD>qcsapi_total_bytes_received</TD>
 *	<TD>Bytes received</TD></TR>
 * <TR> <TD>tx_packets</TD>
 *	<TD>qcsapi_total_packets_sent</TD>
 *	<TD>Packets transmitted</TD></TR>
 * <TR> <TD>rx_packets</TD>
 *	<TD>qcsapi_total_packets_received</TD>
 *	<TD>Packets received</TD></TR>
 * <TR> <TD>tx_discard</TD>
 *	<TD>qcsapi_discard_packets_sent</TD>
 *	<TD>Packets discarded on transmit</TD></TR>
 * <TR> <TD>rx_discard</TD>
 *	<TD>qcsapi_discard_packets_received</TD>
 *	<TD>Packets discarded on receive</TD></TR>
 * <TR> <TD>tx_errors</TD>
 *	<TD>qcsapi_error_packets_sent</TD>
 *	<TD>Packets in error on transmit</TD></TR>
 * <TR> <TD>rx_errors</TD>
 *	<TD>qcsapi_error_packets_received</TD>
 *	<TD>Packets in error on receive</TD></TR>
 * </TABLE>
 *
 * \sa typedef enum qcsapi_counter_type
 */
extern int qcsapi_interface_get_counter(const char *ifname, qcsapi_counter_type qcsapi_counter,
					qcsapi_unsigned_int *p_counter_value);

/**
 * @brief Get statistics from an interface.
 *
 * Return statistics for a given interface and counter. This is the same as
 * \ref <c>qcsapi_interface_get_counter</c>, except that it returns a 64 bit value.
 *
 * \sa qcsapi_interface_get_counter
 */
extern int qcsapi_interface_get_counter64(const char *ifname, qcsapi_counter_type qcsapi_counter,
					uint64_t *p_counter_value);

/**
 * @brief Get the MAC address of an interface.
 *
 * Get the MAC address of an interface.
 *
 * \param ifname \wifi0
 * \param current_mac_addr \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mac_addr \<network device\></c>
 *
 * Output is the \mac_format.
 */
extern int qcsapi_interface_get_mac_addr(const char *ifname, qcsapi_mac_addr current_mac_addr);

/**
 * @brief Set the MAC address of an interface.
 *
 * Set the MAC address of the interface.
 *
 * \note This API needs a system reboot for the MAC address to take effect.
 *
 * \param ifname A wired interface or \wifi0only
 * \param interface_mac_addr \mac_format
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mac_addr \<network device\> \<mac address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_interface_set_mac_addr(const char *ifname,
					const qcsapi_mac_addr interface_mac_addr);

/**
 * \brief Get the Value of a Performance Monitoring Counter.
 *
 * Selected counters are available as Performance Monitor (PM) counters. A PM counter is tied to a
 * PM interval, 15 minutes or 24 hours. The PM counter records the change in the counter since the
 * start of the current PM interval. The Get PM Interval Elapsed Time API reports how much time has
 * elapsed in the current PM interval.
 *
 * \param ifname \wifi0
 * \param qcsapi_counter one of the enumerations from the typedef struct qcsapi_counter_type
 * \param pm_interval the PM interval, either "15_min" or "24_hr"
 * \param p_counter_value return parameter to contain the counter value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pm_counter \<network device\> \<counter name\> \<PM interval\></c>
 */

extern int qcsapi_pm_get_counter(const char *ifname, qcsapi_counter_type qcsapi_counter,
				const char *pm_interval, qcsapi_unsigned_int *p_counter_value);

/**
 * @brief Set the MTU for an interface.
 *
 * Set the MTU for an interface.
 *
 * \param ifname interface name
 * \param mtu MTU to be set. Valid values are dependent on the interface type and platform.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * \note The size of packets transmitted from the device is limited to the lowest MTU of all
 * interfaces in the same bridge group.
 *
 * <c>call_qcsapi set_mtu \<network interface\> \<mtu\></c>
 */
extern int qcsapi_interface_set_mtu(const char *ifname, const uint32_t mtu);

/**
 * @brief Get the MTU for an interface.
 *
 * Get the MTU for an interface.
 *
 * \param ifname interface name
 * \param mtu \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mtu \<network interface\></c>
 *
 * Output is the MTU.
 */
extern int qcsapi_interface_get_mtu(const char *ifname, uint32_t *mtu);

/** @} */

/**
 * @addtogroup PowerAPIs
 * @{
 */

/**
 * \brief enable/disable L1 state for the ASPM of PCIE.
 *
 * when enabling the L1 state, the latency time can be set with parameter(0 ~ 6) for L1 entry.
 * 0 - 1us
 * 1 - 2us
 * 2 - 4us
 * 3 - 8us
 * 4 - 16us
 * 5 - 32us
 * 6 - 64us
 *
 * \param enable 0/1
 * \param latency - time for L1 entry
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_aspm_l1 \<enable\> \<latency\></c>
 */
extern int qcsapi_set_aspm_l1(int enable, int latency);

/**
 * \brief enter/exit L1 state of PCIE link.
 *
 * \param enter 0/1
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_l1 \<enter\></c>
 */
extern int qcsapi_set_l1(int enter);

/** @} */

/**
 * @addtogroup NetInterfaceAPIs
 * @{
 */

/**
 * \brief Get the Elapsed Time in the current PM Interval
 *
 * Returns the amount of time in seconds that has elapsed since the start of the referenced
 * PM interval. PM Intervals last either 15 minutes (900 seconds) or 24 hours (86400 seconds).
 *
 * \param pm_interval the PM interval, either "15_min" or "24_hr"
 * \param p_elapsed_time return parameter to contain the elapsed time in seconds.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pm_elapsed_time \<PM interval\></c>
 */

extern int qcsapi_pm_get_elapsed_time(const char *pm_interval, qcsapi_unsigned_int *p_elapsed_time);

/**
 * @brief power on or power off the ethernet PHY.
 *
 * Power off / on eth PHY. Use <c>qcsapi_eth_phy_power_control</c> to establish
 * power on or off the eth PHY.
 *
 * \param on_off if 1, power off the PHY, else power on the PHY.
 * \param interface the interface name of the eth device
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi eth_phy_power_off \<network interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * Examples:
 *
 * To power off the PHY of interface eth1_0:
 *
 * <c>call_qcsapi eth_phy_power_off eth1_0 1</c>
 */
extern int qcsapi_eth_phy_power_control(int on_off, const char *interface);

/**
 * @brief Get EMAC switch connectivity.
 *
 * This function determines if the EMACs on the board have switch functionality enabled. That is,
 * whether traffic can be switched between devices connected directly to each port directly.
 *
 * \param buf return value of the EMAC switch connectivity.
 * 1 - switch functionality is enabled; 0 - switch functionality is disabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_emac_switch</c>
 *
 * Unless an error occurs, the output will be the value of communications between EMAC 0 and EMAC 1
 */
extern int qcsapi_get_emac_switch(char *buf);

/**
 * @brief Set EMAC switch connectivity.
 *
 * This function sets EMAC switch connectivity.
 *
 * \param value if 0, enabling emac switch functionality, else disabling emac switch functionality.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_emac_switch {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * Examples:
 *
 * To disable emac switch functionality:
 *
 * <c>call_qcsapi set_emac_switch 1</c>
 */
extern int qcsapi_set_emac_switch(qcsapi_emac_switch value);

/**
 * \brief Prioritizing Traffic in EMAC 0 and EMAC 1 using DSCP Commands
 *
 * To set and get DSCP priority values in EMAC0 and EMAC1
 *
 * \param eth_type type of the ethernet device. It should be emac0 or emac1
 * \param level the dscp level, level range should not be negative and less than 64
 * \param value the dscp priority value, value range should not be negative and less than 16
 * \param buf a pointer to the buffer for storing the returned value
 * \param size buf variable size
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi eth_dscp_map \<fill/poke/dump\> \<eth_type\> [level] [value]</c>
 */
extern int qcsapi_eth_dscp_map(qcsapi_eth_dscp_oper oper,
					const char *eth_type,
					const char *level,
					const char *value,
					char * buf,
					const unsigned int size);

/**
 * \brief get Ethernet interface information
 *
 * This API gets Ethernet interface information, including link/speed/duplex/auto-negotiation.
 *
 * \param ifname Ehternet interface name - e.g. eth1_0
 * \param eth_info_type "link", "speed", "duplex" or "autoneg"
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_eth_info \<Ethernet interface\> {link | speed | duplex | autoneg}</c>
 */
extern int qcsapi_get_eth_info(const char * ifname, const qcsapi_eth_info_type eth_info_type);

/**
 * \brief Get IGMP snooping state
 *
 * Get the IGMP snooping state.
 *
 * \param ifname bridge interface (e.g.br0)
 * \param igmp_snooping_state buffer to receive the returned state 1:enable 0:disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_igmp_snooping_state \<bridge interface\> \<igmp_snooping_state\></c>
 */
extern int qcsapi_get_igmp_snooping_state(const char *ifname, uint32_t *igmp_snooping_state);

/**
 * \brief Set IGMP snoop state
 *
 * Set IGMP snooping state. IGMP snooping is enabled by default, but can be disabled
 * if it is to be handled by an external process.
 *
 * \param ifname bridge interface (e.g. br0)
 * \param igmp_snooping_state 1 to enable or 0 to disable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_igmp_snooping_state \<bridge interface\> {0 | 1}</c>
 */
extern int qcsapi_set_igmp_snooping_state(const char *ifname, uint32_t igmp_snooping_state);

/**
 * \brief Get bridge group names
 *
 * Get a list of bridge group names.
 *
 * \param list string buffer to hold the returned string, which is a list of
 * bridge names separated by spaces
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_br_groups</c>
 */
extern int qcsapi_br_get_groups(string_2048 list);

/**
 * \brief Get interface names of a bridge
 *
 * Get a list of interface names of the given bridge.
 *
 * \param brname bridge interface (e.g. br0)
 * \param list string buffer to hold the returned string, which is a list of
 * interface names separated by spaces
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_br_interface \<bridge interface\></c>
 */
extern int qcsapi_br_get_interfaces(const char *brname, string_2048 list);

/** @} */

/**
 * @addtogroup WiFiAPIs
 * @{
 */

/**
 * @brief Get the WiFi mode of the interface.
 *
 * Determine what mode the WiFi interface is operating in, access point or station.
 *
 * \param ifname \wifi0
 * \param p_wifi_mode return parameter to contain the operational mode of the interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mode \<WiFi interface\></c>
 *
 * Output is either <c>Access Point</c> or <c>Station</c>, unless an error occurs.
 */
extern int qcsapi_wifi_get_mode(const char *ifname, qcsapi_wifi_mode *p_wifi_mode);

/**
 * @brief Set the WiFi mode of the interface.
 *
 * Sets the mode for the WiFi interface.
 *
 * \note As a side effect of this API call, a new network interface may be created.
 *
 * The API will fail if the referenced interface already exists.
 *
 * \param ifname \wifi0
 * \param new_wifi_mode the wifi mode to set the interface to.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mode \<WiFi interface\> {ap | sta}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_mode(const char *ifname, const qcsapi_wifi_mode new_wifi_mode);

/**
 * @brief Get the current phy mode of the WLAN interface.
 *
 * Determine what phy mode the WLAN interface is operating in.
 *
 * \param ifname \wifi0
 * \param p_wifi_phy_mode return parameter to contain the current phy mode.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_phy_mode \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be one of the values listed in the 'get_phy_mode value'
 * column of the table in \link qcsapi_wifi_set_phy_mode \endlink
 */
extern int qcsapi_wifi_get_phy_mode(const char *ifname, char *p_wifi_phy_mode);

/**
 * @brief Sets the phy mode of the WLAN interface.
 *
 * Sets the phy mode for the WiFi interface.
 *
 * \param ifname \wifi0
 * \param new_phy_mode phy mode - one of the strings from the Mode column of the table below
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_phy_mode \<WiFi interface\> \<Mode\></c>
 *
 * \call_qcsapi_string_complete
 *
 * <table>
 * <tr>
 *	<td>Mode</td>
 *	<td>phy mode</td>
 *	<td>Bandwidth</td>
 *	<td>Primary channel</td>
 *	<td>Band</td>
 *	<td>get_phy_mode value</td>
 * </tr>
 * <tr>
 *	<td>11ac80Edge+</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>lowest 20MHz channel</td>
 *	<td>5GHz</td>
 *	<td>11ac80Edge+</td>
 * </tr>
 * <tr>
 *	<td>11ac80Cntr+</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>second 20MHz channel</td>
 *	<td>5GHz</td>
 *	<td>11ac80Cntr+</td>
 * </tr>
 * <tr>
 *	<td>11ac80Cntr-</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>third 20MHz channel</td>
 *	<td>5GHz</td>
 *	<td>11ac80Cntr-</td>
 * </tr>
 * <tr>
 *	<td>11ac80Edge-</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>highest 20MHz channel</td>
 *	<td>5GHz</td>
 *	<td>11ac80Edge-</td>
 * </tr>
 * <tr>
 *	<td>11ac80</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ac80Edge+, 11ac80Cntr+, 11ac80Cntr-, or 11ac80Edge- depending on the primary chan</td>
 * </tr>
 * <tr>
 *	<td>11acOnly80</td>
 *	<td>802.11ac</td>
 *	<td>80MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11acOnly80</td>
 * </tr>
 * <tr>
 *	<td>11na40</td>
 *	<td>802.11na</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11na40</td>
 * </tr>
 * <tr>
 *	<td>11ac40</td>
 *	<td>802.11ac</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ac40</td>
 * </tr>
 * <tr>
 *	<td>11nOnly40</td>
 *	<td>802.11n only</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11nOnly40</td>
 * </tr>
 * <tr>
 *	<td>11acOnly40</td>
 *	<td>802.11ac only</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11acOnly40</td>
 * </tr>
 * <tr>
 *	<td>11a</td>
 *	<td>802.11a</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11a</td>
 * </tr>
 * <tr>
 *	<td>11na</td>
 *	<td>802.11na</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11na20</td>
 * </tr>
 * <tr>
 *	<td>11ac</td>
 *	<td>802.11ac</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ac20</td>
 * </tr>
 * <tr>
 *	<td>11nOnly</td>
 *	<td>802.11n only</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11nOnly20</td>
 * </tr>
 * <tr>
 *	<td>11acOnly</td>
 *	<td>802.11ac only</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11acOnly20</td>
 * </tr>
 * <tr>
 *	<td>11nacOnly</td>
 *	<td>802.11n/ac only</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11nacOnly20</td>
 * </tr>
 * <tr>
 *	<td>11nacOnly40</td>
 *	<td>802.11n/ac only</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11nacOnly40</td>
 * </tr>
 * <tr>
 *	<td>11nacOnly80</td>
 *	<td>802.11n/ac only</td>
 *	<td>80MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11nacOnly80</td>
 * </tr>
 * <tr>
 *	<td>11ac160</td>
 *	<td>802.11ac</td>
 *	<td>160MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ac160</td>
 * </tr>
 * <tr>
 *	<td>11b</td>
 *	<td>802.11b</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11b</td>
 * </tr>
 * <tr>
 *	<td>11g</td>
 *	<td>802.11g</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11g</td>
 * </tr>
 * <tr>
 *	<td>11ng</td>
 *	<td>802.11ng</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11ng</td>
 * </tr>
 * <tr>
 *	<td>11ng40</td>
 *	<td>802.11ng</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11ng40</td>
 * </tr>
 * <tr>
 *	<td>11nOnly2g</td>
 *	<td>802.11n only</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11nOnly202g</td>
 * </tr>
 * <tr>
 *	<td>11nOnly402g</td>
 *	<td>802.11n only</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11nOnly402g</td>
 * </tr>
 * <tr>
 *	<td>11ax</td>
 *	<td>802.11ax</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ax20</td>
 * </tr>
 * <tr>
 *	<td>11ax40</td>
 *	<td>802.11ax</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ax40</td>
 * </tr>
 * <tr>
 *	<td>11ax80</td>
 *	<td>802.11ax</td>
 *	<td>80MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ax80</td>
 * </tr>
 * <tr>
 *	<td>11ax160</td>
 *	<td>802.11ax</td>
 *	<td>160MHz</td>
 *	<td>current channel</td>
 *	<td>5GHz</td>
 *	<td>11ax160</td>
 * </tr>
 * <tr>
 *	<td>11axng</td>
 *	<td>802.11ax</td>
 *	<td>20MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11axng</td>
 * </tr>
 * <tr>
 *	<td>11axng40</td>
 *	<td>802.11ax</td>
 *	<td>40MHz</td>
 *	<td>current channel</td>
 *	<td>2.4GHz</td>
 *	<td>11axng40</td>
 * </tr>
 * </table>
 *
 * <b>Examples</b>
 *
 * <c>call_qcsapi set_phy_mode wifi0_0 11ac80Edge+</c>
 *
 * - When mode is <c>11ac80Edge+</c> and the PHY is on the 80MHz channel from channel 36 to 48,
 *	the primary channel will be the lowest channel (36).
 * - When mode is <c>11ac80Cntr+</c> and the PHY is on the 80MHz channel from channel 36 to 48,
 *	the primary channel will be the second channel (40).
 * - When mode is <c>11ac80Cntr-</c> and the PHY is on the 80MHz channel from channel 36 to 48,
 *	the primary channel will be the third channel (44).
 * - When mode is <c>11ac80Edge-</c> and the PHY is on the 80MHz channel from channel 36 to 48,
 *	the primary channel will be the highest channel (48).
 *
 * \note New APIs are added to get/set the required phymode capability of peer STA allowed for
 * association, it will be more flexible to use two separate APIs to set phymode and the
 * required phymode during assocation to achieve all possible combinations, so the old phymodes
 * that contain "Only" (such as "11nOnly", "11nOnly40", "11acOnly", ...) are not recommended
 * any more.
 *
 * For example:
 *
 * @code
 * call_qcsapi set_phy_mode wifi0_0 11nacOnly80
 * @endcode
 *
 * is equivalent to:
 *
 * @code
 * call_qcsapi set_phy_mode wifi0_0 11ac80
 * call_qcsapi set_phy_mode_required wifi0_0 11n
 * @endcode
 *
 * \sa qcsapi_wifi_set_phy_mode_required
 */
extern int qcsapi_wifi_set_phy_mode(const char *ifname, const char *new_phy_mode);

/**
 * @brief Reload the interface to change its mode (AP->STA or STA->AP).
 *
 * This function will delete the interface and other WDS vaps that may exist, then recreate the
 * interface in the requested mode, and perform all necessary initialization.
 *
 * This API is used for switching between AP and STA modes without rebooting.
 *
 * \param ifname \wifi0only
 * \param new_wifi_mode the new wifi mode to set the interface to.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reload_in_mode \<WiFi interface\> {ap | sta}</c>
 *
 * When used in NPU-based hostapd, it should be used like below to switch to ap:
 *
 * <c>qcsapi_pcie reload_in_mode \<WiFi interface\> \<ap\> \<global ctrl iface path and name\>
 *	\<hostapd conf file\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_reload_in_mode(const char *ifname, const qcsapi_wifi_mode new_wifi_mode);

/**
 * \brief Enable or disable a radio interface.
 *
 * Enable or disable a radio interface.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_rfenable.
 *
 * \param onoff \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \sa qcsapi_radio_rfenable
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rfenable {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_rfenable(const qcsapi_unsigned_int onoff);

/**
 * \brief Enable or disable a radio interface.
 *
 * Enable or disable a radio interface.
 *
 * \param ifname \wifi_all
 * \param onoff \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rfenable {\<WiFi interface\> | all} {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_radio_rfenable(const char *ifname, const qcsapi_unsigned_int onoff);

/**
 * \brief Get the current status
 *
 * This API call returns the current status of the specified device radio.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_rfstatus.
 *
 * \param rfstatus a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \sa qcsapi_radio_rfstatus
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rfstatus \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current status of the RadioY specified in wifiY_X.
 */
extern int qcsapi_wifi_rfstatus(qcsapi_unsigned_int *rfstatus);

/**
 * \brief Get the current status
 *
 * This API call returns the current status of the specified device radio.
 *
 * \param ifname \wifi0only
 * \param rfstatus a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rfstatus \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current status of the RadioY specified in wifiY_X.
 */
extern int qcsapi_radio_rfstatus(const char *ifname, qcsapi_unsigned_int *rfstatus);

/**
 * @brief Get the WiFi interface bandwidth (20MHz or 40MHz).
 *
 * Return the configured bandwidth, as specified in the 802.11n standard, either 20 MHz or 40 MHz.
 *
 * Returned value is in MHz, either 20 40, or 80.
 *
 * \param ifname \wifi0only
 * \param p_bw return parameter to contain the bandwidth the interface is working in (20, 40 or 80)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bw \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current bandwidth configuration, either 20, 40 or
 * 80.
 *
 * \sa qcsapi_wifi_set_bw
 * \sa qcsapi_wifi_set_phy_mode
 */
extern int qcsapi_wifi_get_bw(const char *ifname, qcsapi_unsigned_int *p_bw);

/**
 * \brief Get the channel list which OCAC disabled
 *
 * Get channels that have OCAC disabled
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_chan_list.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param buffer databuf to get the channels that are OCAC disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dfs_s_radio_chan_off \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of channels that are OCAC disabled
 */
extern int qcsapi_wifi_get_dfs_s_radio_chan_off(const char *ifname,
						struct qcsapi_data_32bytes *buffer);

/**
 * \brief Enable or disable OCAC for a DFS channel
 *
 * Enable or disable OCAC for a DFS channel
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param channel the channel which is OCAC disabled or enabled and must be a DFS one
 * \param disable 0 (enable OCAC) or 1 (disable OCAC) and must be either 0 or 1
 *
 * \return \zero_or_negative
 * \callqcsapi
 *
 * <c>call_qcsapi set_dfs_s_radio_chan_off \<WiFi interface\> \<channel\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dfs_s_radio_chan_off(const char *ifname,
				const qcsapi_unsigned_int channel, const uint32_t disable);

/**
 * @brief Set the WiFi interface bandwidth.
 *
 * Set the bandwidth during system startup, before calling the Enable Interface API for the WiFi
 * device.
 *
 * If the bandwidth is set to 20MHz, any associations will have a bandwidth limited to 20 MHz.
 *
 * If the bandwidth is set to 40MHz, an association will default to a bandwidth of 40 MHz (if the
 * peer supports this).
 *
 * If the bandwidth is set to 80MHz, an association will default to a bandwidth of 80 MHz (if the
 * peer supports this).
 *
 * \param ifname \wifi0only
 * \param bw the bandwith to set the device to.
 *
 * \note 80MHz bandwidth is only supported in 802.11ac mode. 802.11ac mode is set using
 *	qcsapi_wifi_set_phy_mode
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bw \<WiFi interface\> {80 | 40 | 20}</c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_wifi_set_phy_mode
 */
extern int qcsapi_wifi_set_bw(const char *ifname, const qcsapi_unsigned_int bw);

/**
 * @brief Get the WiFi 2.4Ghz interface bandwidth (20MHz or 40MHz).
 *
 * Return the configured bandwidth, as specified in the 802.11b/g/ng standard, either 20 MHz or 40
 * MHz.
 *
 * Returned value is in MHz, either 20 or 40.
 *
 * \param ifname \wifi0only
 * \param p_bw return parameter to contain the bandwidth the interface is working in (20 or 40)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_24g_bw \<WiFi interface\></c>
 *
 * \note This API can only be used on a dual band radio in Station mode.
 *
 * Unless an error occurs, the output will be the current bandwidth configuration, either 20 or 40.
 */
extern int qcsapi_wifi_get_24g_bw(const char *ifname, qcsapi_unsigned_int *p_bw);

/**
 * @brief Set the WiFi interface bandwidth.
 *
 * Use this API to set the bandwidth during system startup, before calling the Enable Interface API
 * for the WiFi device.
 *
 * Bandwidth can be either 40MHz or 20MHz.
 *
 * If the bandwidth is set to 20MHz, any associations will have a bandwidth limited to 20 MHz.
 * If the bandwidth is set to 40MHz, an association will default to a bandwidth of 40 MHz (if the
 * peer supports this).
 *
 * \param ifname \wifi0only
 * \param bw the bandwith to set the device to.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_24g_bw \<WiFi interface\> {40 | 20}</c>
 *
 * \note This API can only be used on a dual band radio in Station mode.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_24g_bw(const char *ifname, const qcsapi_unsigned_int bw);

/**
 * \brief set vht
 *
 * This API call is used to set vht mode to enable 802.11ac operation.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_phy_mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vht \<WiFi interface\> {0 | 1}</c>
 *
 * \sa qcsapi_wifi_set_phy_mode
 */
extern int qcsapi_wifi_set_vht(const char *ifname, const qcsapi_unsigned_int the_vht);

/**
 * \brief get vht
 *
 * This API call is used to get vht mode.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_phy_mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be <c>1</c> (enabled) or <c>0</c> (disabled)
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vht \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_vht(const char *ifname, qcsapi_unsigned_int *vht);

/**
 * \brief Enable or disable WiFi 802.11ax (HE) operation.
 *
 * Enable or disable WiFi 802.11ax (HE) operation.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_phy_mode.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * <c>call_qcsapi set_he \<WiFi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_he(const char *ifname, const qcsapi_unsigned_int enable);

/**
 * \brief Get WiFi 802.11ax (HE) operational status.
 *
 * Get WiFi 802.11ax HE (High Efficiency) operational status.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_phy_mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be <c>1</c> (enabled) or <c>0</c> (disabled)
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_he \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_he(const char *ifname, qcsapi_unsigned_int *enable);


/**
 * \brief Get the current operating channel.
 *
 * Get the current operating channel number.
 *
 * \note \deprecated_and_replaced_with \ref qcsapi_wifi_get_chan.
 *
 * \param ifname \wifi0
 * \param p_current_channel a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 * return -ERANGE, numeric value out of range.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_channel \<WiFi interface\></c>
 *
 * The output will be the channel number unless an error occurs.
 */
extern int qcsapi_wifi_get_channel(const char *ifname, qcsapi_unsigned_int *p_current_channel);

/**
 * \brief Get the current operating channel, bandwidth and the frequency band.
 *
 * Get the current operating channel, bandwidth and the frequency band.
 *
 * \param ifname \wifi0
 * \param p_current_channel \valuebuf
 * \param p_current_bw \valuebuf
 * \param p_current_band \valuebuf
 *
 * \return \zero_or_negative
 * return -ERANGE, numeric value out of range.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current operating channel, bandwidth
 * and the frequency band.
 */
extern int qcsapi_wifi_get_chan(const char *ifname,
			qcsapi_unsigned_int *p_current_channel,
			qcsapi_unsigned_int *p_current_bw,
			qcsapi_unsigned_int *p_current_band);

/**
 * \brief Set the current operating channel.
 *
 * Set the current operating channel.
 *
 * \note \deprecated_and_replaced_with \ref qcsapi_wifi_set_chan.
 *
 * \note The channel can only be set when the interface is up.
 *
 * \param ifname \wifi0
 * \param new_channel the new channel to be used
 *
 * \return \zero_or_negative
 * return -ERANGE, numeric value out of range.
 *
 * The new channel must be between 1 and 255, and is further restricted by the 802.11
 * standard.
 *
 * This is an engineering API, since it does not account for regulatory requirements. Use
 * of this API can cause the system to violate regulatory requirements. Use of the Set
 * Regulatory Channel API is recommended.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_channel \<WiFi interface\> \<new_channel\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_channel(const char *ifname, const qcsapi_unsigned_int new_channel);

/**
 * \brief Set the current operating channel, bandwidth and the frequency band.
 *
 * Set the current operating channel, bandwidth and the frequency band.
 *
 * \note The channel can only be set when the interface is up.
 *
 * \param ifname \wifi0
 * \param new_channel the new channel to be used.  This should be between 1 and 255,
 *	and is further restricted by the 802.11 standard.
 * \param new_bw the new bandwidth - must be 20/40/80/160, and is further restricted by
 *	the 802.11 standard.
 * \param new_band the new frequency band to be used. The valid values are defined
 *	in \ref qcsapi_freq_band.
 *
 * \return \zero_or_negative
 *
 * \note This is an engineering API, since it does not account for regulatory requirements. Use
 * of this API can cause the system to violate regulatory requirements. Use of the Set
 * Regulatory Channel API is recommended.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan \<WiFi interface\> \<new_channel\> \<new_bw\> \<new_band\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_chan(const char *ifname,
			const qcsapi_unsigned_int new_channel,
			const qcsapi_unsigned_int new_bw,
			const qcsapi_unsigned_int new_band);

/**
 * \brief Get the current operating channel and bandwidth.
 *
 * Get the current operating channel number and bandwidth.
 *
 * \param ifname \wifi0only
 * \param p_chan_bw a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_and_bw \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the chan and bandwidth <c>complete</c>.
 */
extern int qcsapi_wifi_get_channel_and_bw(const char *ifname,
			struct qcsapi_data_32bytes *p_chan_bw);

/**
 * \brief Set the current operating channel and bandwidth.
 *
 * Set the current operating channel and bandwidth.
 *
 * \note The channel can only be set when the interface is up.
 *
 * \param ifname \wifi0only
 * \param new_channel the new channel - must be between 1 and 255, and is further restricted by the
 *	802.11 standard.
 * \param new_bw the new bandwidth - must be 20/40/80/160, and is further restricted by the 802.11
 *	standard.
 *
 * \return \zero_or_negative
 *
 * \warning \bad_regulatory
 * Use of the Set Regulatory Channel API is recommended.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_and_bw \<WiFi interface\> \<new_channel\> \<new_bw\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 */
extern int qcsapi_wifi_set_channel_and_bw(const char *ifname, const qcsapi_unsigned_int new_channel,
			const qcsapi_unsigned_int new_bw);

/**
 * \brief Set weather channel CAC availability.
 *
 * This API is called to set weather channel CAC availability.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param en_value \disable_or_enable.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wea_cac_en \<WiFi interface\> \<en_value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_wea_cac_en(const char *ifname, const qcsapi_unsigned_int en_value);

/**
 * \brief Get the inactive channel list.
 *
 * \param ifname \wifi0only
 * \param buffer buffer to retrieve inactive channel list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_pri_inactive \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a comma-separated list of channels, with "(auto)"
 * after each auto channel.
 */
extern int qcsapi_wifi_get_chan_pri_inactive(const char *ifname,
				struct qcsapi_data_256bytes *buffer);

/**
 * \brief Set the channel inactive flag of primary channel.
 *
 * Specify whether the channel can be used as primary channel or not.
 *
 * \param ifname \wifi0only
 * \param channel the channel to change the inactive flag
 * \param inactive the flag whether it can be used as primary channel or not
 *
 * \return \zero_or_negative
 *
 * The channel must be between 1 and 255, and is further restricted by the 802.11
 * standard.
 *
 * The inactive flag must be either 0 or 1.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_pri_inactive \<WiFi interface\> \<channel\> \<inactive\></c>
 *
 * If inactive flag is not specified, default is 1.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_chan_pri_inactive(const char *ifname, const qcsapi_unsigned_int channel,
						const qcsapi_unsigned_int inactive);


#define QCSAPI_CHAN_PRI_INACTIVE_AUTOCHAN	0x1

/**
 * \brief Set the channel inactive flag of primary channel.
 *
 * Specify whether the channel can be used as primary channel or not, and also pass control flags to
 * kernel.
 *
 * \param ifname \wifi0only
 * \param channel the channel to change the inactive flag
 * \param inactive the flag whether it can be used as primary channel or not
 * \param option_flags the control flags .
 *
 * \return \zero_or_negative
 *
 * The channel must be between 1 and 255, and is further restricted by the 802.11
 * standard.
 *
 * The inactive flag must be either 0 or 1.
 *
 * The option_flags is defined as below:
 *	0x1 - auto channel selection only. If it is set, the primary channel inactive flag is valid
 *	for auto channel selection only, it is invalid for manual channel configuration. If it is
 *	not set, the inactive flag is valid for both auto and manual channel configuration.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_pri_inactive \<WiFi interface\> \<channel\> \<inactive\> \<option\></c>
 *
 * If inactive flag is not specified, default is 1.
 * The option can be "autochan", and it is valid only when inactive flag is 1.
 * The inactive flag must be present if option is present.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_chan_pri_inactive_ext(const char *ifname,
				const qcsapi_unsigned_int channel,
				const qcsapi_unsigned_int inactive, const uint32_t option_flags);

/**
 * \brief Set the channel disabled/enabled.
 *
 * Specify the channel can be used or not, this API can only be used during start-up period.
 *
 * \param ifname \wifi0only
 * \param chans point to array of channels.
 * \param cnt the number of channels
 * \param flag indication of enabling or disabling channels, 0: enable, 1: disable
 *
 * \return \zero_or_negative
 *
 * The chans must be between -255 and 255, and the absolute value is further restricted by the
 * 802.11 standard.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_disabled \<WiFi interface\> \<channel\> ...</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_chan_control(const char *ifname, const struct qcsapi_data_256bytes *chans,
					const uint32_t cnt, const uint8_t flag);

/**
 * \brief Get the disabled channel list.
 *
 * List the channels which are disabled and can not be used for primary channel.
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_chan_list.
 *
 * \param ifname \wifi0only
 * \param p_chans a pointer to the buffer for storing array of channels.
 * \param p_cnt a pointer to the buffer for storing the number of disabled channels
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_disabled \<WiFi interface\> \<channel\> ...</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_get_chan_disabled(const char *ifname, struct qcsapi_data_256bytes *p_chans,
						uint8_t *p_cnt);

/**
 * \brief Get the usable channel list.
 *
 * List the channels which are usable.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_chan_list.
 *
 * \param ifname \wifi0only
 * \param p_chans a pointer to the buffer for storing bitmap array of channels.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_usable \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the result of all usable channels.
 */
extern int qcsapi_wifi_get_chan_usable(const char *ifname, struct qcsapi_data_32bytes *p_chans);

/**
 * \brief Get supported frequency bands.
 *
 * List the supported frequency bands for a given interface.
 *
 * \param ifname \wifi0
 * \param p_chans a pointer to the buffer for storing array of channels.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_supported_freq_bands \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_get_supported_freq_bands(const char *ifname, string_32 p_bands);

/**
 * \brief Get the beacon interval
 *
 * Get the beacon interval.
 *
 * \param ifname \wifi0
 * \param p_current_intval a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_beacon_interval \<WiFi interface\></c>
 *
 * The output will be the beacon interval in TUs (1,024 usecs) unless an error occurs.
 */
extern int qcsapi_wifi_get_beacon_interval(const char *ifname,
				qcsapi_unsigned_int *p_current_intval);

/**
 * \brief Set the beacon interval
 *
 * Set the beacon interval.
 *
 * \param ifname \wifi0
 * \param new_intval interval in TUs (1,024 usecs).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_beacon_interval \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_beacon_interval(const char *ifname,
				const qcsapi_unsigned_int new_intval);

/**
 * \brief Get the DTIM interval
 *
 * Get the frequency (in number of beacons) at which the DTIM (Delivery Traffic
 * Information Message) Information Element is added to beacons.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_parameter dtim_period.
 *
 * \param ifname \wifi0
 * \param p_dtim a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dtim \<WiFi interface\></c>
 *
 * The output will be the DTIM interval in milliseconds unless an error occurs.
 */
extern int qcsapi_wifi_get_dtim(const char *ifname, qcsapi_unsigned_int *p_dtim);

/**
 * \brief Set the DTIM interval
 *
 * Set the frequency (in number of beacons) at which the DTIM (Delivery Traffic
 * Information Message) Information Element is added to beacons.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_parameter dtim_period.
 *
 * \param ifname \wifi0
 * \param new_dtim the new DTIM value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dtim \<WiFi interface\> \<DTIM interval\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dtim(const char *ifname, const qcsapi_unsigned_int new_dtim);

/**
 * \brief Get association limit
 *
 * This API retrieves the current association limit for the primary interface.
 * The association limit is the number of stations that may be simultaneously
 * associated to this AP.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param p_assoc_limit Address of variable for result retrieval
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dev_assoc_limit wifi0_0</c>
 *
 * The output will be the overall device association limit, or an error
 * message in case of failure.
 *
 * \sa qcsapi_wifi_get_bss_assoc_limit
 */
extern int qcsapi_wifi_get_assoc_limit(const char *ifname, qcsapi_unsigned_int *p_assoc_limit);

/**
 * \brief Get VAP logical group's association limit
 *
 * This API retrieves the current VAP logical group association limit.
 * The association limit is the maximum number of stations that can be associated to
 * this VAP even if the device limit is not reached.
 *
 * \note See \link qcsapi_wifi_set_SSID_group_id \endlink for the definition of a logical group.
 *
 * \param group group number, between 1 and 31
 * \param p_assoc_limit Address of variable for return result
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bss_assoc_limit \<group\></c>
 *
 * The output will be the VAP's logical group association limit of the device, or an error
 * message in case of failure.
 *
 * \sa qcsapi_wifi_get_assoc_limit
 */
extern int qcsapi_wifi_get_bss_assoc_limit(qcsapi_unsigned_int group,
						qcsapi_unsigned_int *p_assoc_limit);

/**
 * \brief Set association limit
 *
 * This API sets the current association limit across all BSSes in the system.
 *
 * \note When the limit is changed, all the group assoc parameters configured through
 * call_qcsapi set_bss_assoc_limit and call_qcsapi set_SSID_assoc_reserve commands will also
 * be reset to new dev limit and 0 respectively. All devices will be disassociated
 * and forced to reassociate.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param new_assoc_limit New association limit
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dev_assoc_limit wifi0_0 \<limit\></c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_wifi_set_bss_assoc_limit
 */
extern int qcsapi_wifi_set_assoc_limit(const char *ifname,
				const qcsapi_unsigned_int new_assoc_limit);

/**
 * \brief Set the maximum number of associations allowed per logical group of VAPs.
 *
 * This API sets the maximum number of associations allowed in a logical group.
 *
 * \note When the limit is changed, all devices associated to the group will be
 *	disassociated and forced to reassociate.
 *
 * \note \aponly
 *
 * \note See \link qcsapi_wifi_set_SSID_group_id \endlink for the definition of a logical group.
 *
 * \param group group number, between 1 and 31
 * \param limit maximum number of devices that are allowed to associate across
 *  all WiFi interfaces in the logical group
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bss_assoc_limit \<group\> \<limit\></c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_wifi_set_assoc_limit
 */
extern int qcsapi_wifi_set_bss_assoc_limit(const qcsapi_unsigned_int group,
							const qcsapi_unsigned_int limit);

/**
 * \brief Assign VAP (SSID) to a logical group
 *
 * This API assigns an SSID to a logical group.
 *
 * \note \aponly
 *
 * \note Configuring this parameter, disassociates all associated devices
 * and will force to reassociate.
 *
 * \note A logical group consists of a set of one or more VAPs across different radios or
 *  on one radio.
 *
 * \param ifname \wifi0
 * \param group group number, between 1 and 31
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_SSID_group_id \<WiFi interface\> \<group\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_SSID_group_id(const char *ifname, const qcsapi_unsigned_int group);

/**
 * \brief Get VAP's (SSID) logical group id
 *
 * This API returns the SSID's logical group id.
 *
 * \note \aponly
 *
 * \note See \link qcsapi_wifi_set_SSID_group_id \endlink for the definition of a logical group.
 *
 * \param ifname \wifi0
 * \param p_group Address of variable for result retrieval
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID_group_id \<WiFi interface\></c>
 *
 * The output will be the VAP's logical group id, or an error message in case of failure.
 */
extern int qcsapi_wifi_get_SSID_group_id(const char *ifname, qcsapi_unsigned_int *p_group);

/**
 * \brief Reserve associations for a logical group
 *
 * This API reserves associations for a group.
 *
 * \note This setting is only supported when all VAPs in the logical group are on the same
 * radio. If the group contains VAPs from more than one radio, this setting will be ignored.
 *
 * \note Configuring this parameter, disassociates all associated devices
 * and will force to reassociate.
 *
 * \note See \link qcsapi_wifi_set_SSID_group_id \endlink for the definition of a logical group.
 *
 * \param group group number, between 1 and 31
 * \param value number of reserved associations for a group
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_SSID_assoc_reserve \<group\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_SSID_assoc_reserve(const qcsapi_unsigned_int group,
							const qcsapi_unsigned_int value);
/**
 * \brief Get number of associations reserved for a logical group
 *
 * This API gets the number of associations reserved for the group.
 *
 * \note \aponly
 *
 * \note See \link qcsapi_wifi_set_SSID_group_id \endlink for the definition of a logical group.
 *
 * \param group group number, between 1 and 31
 * \param p_value address of variable for return result
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID_assoc_reserve \<group\></c>
 *
 * The output will be the VAPs logical group reserved association value, or an error
 * message in case of failure.
 */
extern int qcsapi_wifi_get_SSID_assoc_reserve(qcsapi_unsigned_int group,
						qcsapi_unsigned_int *p_value);

/**
 * \brief Get current BSSID
 *
 * This API retrieves the current BSSID (basic service set identification)
 *
 * \param ifname \wifi0
 * \param current_BSSID memory in which to store the current BSSID. On an unassociated station, or
 *	if the interface is disabled, this field will be zeroed.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_BSSID \<WiFi interface\></c>
 *
 * The output will be the BSSID MAC address.
 */
extern int qcsapi_wifi_get_BSSID(const char *ifname, qcsapi_mac_addr current_BSSID);

/**
 * \brief Get configured BSSID
 *
 * Retrieves BSSID stored in configuration file
 *
 * \param ifname \wifi0
 * \param config_BSSID memory in which to store the configured BSSID. If there is no BSSID
 * configured for the interface memory will be filled with zeros.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_config_BSSID \<WiFi interface\></c>
 *
 * The output will be a printout of the BSSID MAC address on success, or print an error message on
 * failure.
 */
extern int qcsapi_wifi_get_config_BSSID(const char *ifname, qcsapi_mac_addr config_BSSID);

/**
 * \brief Get BSSID for a SSID
 *
 * Retrieve the BSSID per SSID stored in the configuration file (if it exists)
 *
 * \param ifname \wifi0
 * \param BSSID memory in which to store the retrieved BSSID.
 * If there is no BSSID configured for the SSID, return result will be error 1012
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ssid_BSSID \<WiFi interface\> \<SSID\></c>

 * The output will be a printout of the BSSID MAC address on success, or print an error message on
 * failure.
 */
extern int qcsapi_wifi_ssid_get_bssid(const char *ifname, const qcsapi_SSID ssid_str,
		qcsapi_mac_addr bssid);

/**
 * \brief Set/Unset configured BSSID for a SSID
 *
 * Add/remove BSSID per SSID stored in configuration file (for a station). This optional
 * configuration will restrict to the specified BSSID address.
 *
 * \param ifname \wifi0
 * \param SSID_str is the SSID to be configured
 * \param BSSID memory in which to store the required BSSID. If the BSSID parameter
 * is filled with all 0xFF (ie MAC address ff:ff:ff:ff:ff:ff), then any existing configured BSSID
 * is removed for the specified SSID.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ssid_BSSID \<WiFi interface\> \<SSID\> \<BSSID MAC address\></c>
 */
extern int qcsapi_wifi_ssid_set_bssid(const char *ifname, const qcsapi_SSID ssid_str,
		const qcsapi_mac_addr bssid);

/**
 * @brief Get the WiFi interface SSID.
 *
 * Get the current SSID of the interface.
 *
 * \param ifname \wifi0
 * \param SSID_str return parameter to contain the SSID.
 *
 * \return \zero_or_negative
 *
 * \note On a STA with security enabled and is not in an association, this API will return the
 *	0-length string in the SSID_str parameter.
 * \note If the SSID was set by using the <c>hexstr</c> option, the output maybe report not
 * supported error. Use get_SSID2 if this option has been used.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current SSID.
 * On a STA, if security is enabled and is not in association, the output will be a blank line.
 */
extern int qcsapi_wifi_get_SSID(const char *ifname, qcsapi_SSID SSID_str);

/**
 * @brief Set the WiFi interface SSID.
 *
 * Set the current SSID for AP operation on the given interface.
 *
 * \note \aponly
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \warning Any preexisting associations will be dropped, and those stations will be required to
 *	reassociate, using the new SSID.
 *
 * Use the WiFi Associate API (qcsapi_wifi_associate) on a STA to set the SSID.
 *
 * The SSID must be a string with between 1 and 32 characters. Control characters (^C, ^M, etc.) are
 *	not permitted.
 *
 * \param ifname \wifi0
 * \param SSID_str the new SSID to set on the interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_SSID \<WiFi interface\> \<SSID\></c>
 *
 * \call_qcsapi_string_complete
 *
 * <b>Examples</b>
 *
 * To set SSID to Quantenna, enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "Quantenna"</c>
 *
 * To set SSID to 'Quantenna', enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "'Quantenna'"</c>
 *
 * To set SSID to "Quantenna", enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "\"Quantenna\""</c>
 *
 * \sa qcsapi_wifi_associate
 */
extern int qcsapi_wifi_set_SSID(const char *ifname, const qcsapi_SSID SSID_str);

/**
 * @brief Get the WiFi interface SSID.
 *
 * Get the current SSID of the interface.
 *
 * \param ifname \wifi0
 * \param SSID_str return parameter to contain the SSID.
 * \param fmt buffer to contain the returned format type of the SSID
 *
 * \return \zero_or_negative if the command succeeded and SSID_str contains the SSID.
 * \return A negative value if an error occurred.  See @ref mysection4_1_4 "QCSAPI Return Values"
 * for error codes and messages.
 *
 * \note For a station that is not associated, this API will return the zero-length string.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID2 \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current SSID.
 * * If the SSID was set as a printable string, it is printed in double quotes.
 * * If the SSID was set as a hex string, it is printed in hexdump format.
 *
 * For a station that is not associated, the output will be a null string.
 * \sa qcsapi_wifi_get_SSID
 */
extern int qcsapi_wifi_get_SSID2(const char *ifname, qcsapi_SSID2 SSID_str,
		qcsapi_ssid_fmt *fmt);

/**
 * @brief Set the WiFi interface SSID.
 *
 * Set the current SSID for AP operation on the given interface.
 *
 * \note \aponly
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \warning Any preexisting associations will be dropped, and those stations will be required
 * to reassociate, using the new SSID.
 *
 * \note Use \ref qcsapi_wifi_associate to set the SSID on a station.
 *
 * \param ifname \wifi0
 * \param SSID_str the new SSID for the interface. The allowed SSID formats are described
 * in \ref SSID2_RULES.
 * \param fmt format type of the SSID
 *
 * \return \zero_or_negative if the command succeeded and the SSID is updated.
 * \return A negative value if an error occurred.  See @ref mysection4_1_4 "QCSAPI Return Values"
 * for error codes and messages.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_SSID \<WiFi interface\> [{str | hexstr}] \<SSID\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 *
 * <b>Examples</b>
 *
 * To set SSID to Quantenna, enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "Quantenna"</c>
 *
 * To set SSID to 'Quantenna', enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "'Quantenna'"</c>
 *
 * To set SSID to "Quantenna", enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 "\"Quantenna\""</c>
 *
 * To set SSID to "\x0102\x0304\x0506", enter:
 *
 * <c>call_qcsapi set_ssid wifi0_0 hexstr "010203040506"</c>
 *
 * \sa qcsapi_wifi_set_SSID
 */
extern int qcsapi_wifi_set_SSID2(const char *ifname, const qcsapi_SSID2 SSID_str,
		qcsapi_ssid_fmt fmt);

/**
 * @brief Get the IEEE 802.11 PHY mode being used on the interface.
 *
 * Get the current IEEE 802.11 standard(s) that the WiFi device supports. Expected return values
 * (all are character strings) follow:
 *
 * <TABLE>
 * <TR> <TD>Value</TD>
 *	<TD>Interpretation</TD> </TR>
 * <TR> <TD>a-only</TD>
 *	<TD>Only 802.11a support present (5 GHz with up to 54 Mbps throughput)</TD> </TR>
 * <TR> <TD>b-only</TD>
 *	<TD>Only 802.11b support present (2.4 GHz with up to 11 Mbps throughput)</TD> </TR>
 * <TR> <TD>g-only</TD>
 *	<TD>Only 802.11g support present (2.4 GHz with up to 54 Mbps throughput)</TD> </TR>
 * <TR> <TD>a|n</TD>
 *	<TD>802.11n with 802.11a available for backwards compatibility in 5 GHz</TD> </TR>
 * <TR> <TD>g|n</TD>
 *	<TD>802.11n with 802.11g available for backwards compatibility in 2.4 GHz</TD> </TR>
 * <TR> <TD>a|n|ac</TD>
 *	<TD>802.11ac with 802.11n and 802.11a available for backwards compatibility in 5 GHz</TD>
 *	</TR>
 * <TR> <TD>b|g|n|ac</TD>
 *	<TD>802.11ac with 802.11n, 802.11g and 802.11b in 2.4GHz </TD> </TR>
 * <TR> <TD>a|n|ac|ax</TD>
 *	<TD>802.11ax with 802.11n, 802.11a and 802.11ac available for backwards compatibility in
 *	5 GHz </TD> </TR>
 * <TR> <TD>b|g|n|ac|ax</TD>
 *	<TD>802.11ax with 802.11n, 802.11g, 802.11b and 802.11ac available for backwards
 *	compatibility in 2.4 GHz </TD> </TR>
 * </TABLE>
 *
 * \note The passed in string MUST be at least 7 bytes to contain the maximum length string per the
 * preceding table.
 *
 * \param ifname \wifi0only
 * \param IEEE_802_11_standard return parameter to contain the string with the PHY mode.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_standard \<WiFi interface\></c>
 *
 * <c>call_qcsapi get_802.11 \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_IEEE_802_11_standard(const char *ifname, char *IEEE_802_11_standard);

/**
 * @brief Get the list of available channels for an interface.
 *
 * Get the list of available channels with each value separated by commas.
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_chan_list.
 *
 * \warning \bad_regulatory
 * Use the get list regulatory channels API (qcsapi_wifi_get_list_regulatory_channels) to get the
 * list of valid channels by regulatory authority.
 *
 * \param ifname \wifi0
 * \param list_of_channels return parameter to contain the list of channels, comma separated.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_channel_list \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of WiFi channels per the 802.11 standard.
 */
extern int qcsapi_wifi_get_list_channels(const char *ifname, string_1024 list_of_channels);

/**
 * @brief Get the list of supported channels for a STA.
 *
 * Get the list of supported channels with each value separated by commas.
 *
 * \note \aponly
 *
 * \warning \bad_regulatory
 * Use <c>qcsapi_wifi_get_supp_chans</c> to get the list of supported channels for a STA.
 *
 * \param ifname \wifi0only
 * \param mac_addr the MAC address of an associated station
 * \param list_of_channels buffer to contain the list of comma-separated channels
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_supp_chan \<WiFi interface\> \<MAC address\></c>
 *
 * Unless an error occurs, the output will be the list of WiFi channels per the 802.11 standard.
 */
extern int qcsapi_wifi_get_supp_chans(const char *ifname, qcsapi_mac_addr mac_addr,
		string_1024 list_of_channels);

/**
 * \brief Get WiFi mode switch setting
 *
 * This API retrieves the current WiFi mode switch setting.
 *
 * \note It is required that the relevant GPIOs attached to the switch (12 and 13 for 3way switch
 * and only 12 for 2way switch) be configured as inputs for this API to work.
 *
 * \param p_wifi_mode_switch_setting pointer to result memory. On success, the result memory will
 * have 0, 1 or 2 written to it depending on the position of the switch. If the GPIOs are not
 * configured as inputs, value is undefined. 0 value means AP, 1 means STA and 2 means AUTO mode.
 * On a 2-way switch only AP and STA modes are existing.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mode_switch</c>
 *
 * On success, call_qcsapi will print the setting as an integer (1, 2, or 3) depending on the switch
 * setting, or 0 if the relevant GPIOs are not configured as inputs. On failure, an error message
 * will be written to stdout.
 */
extern int qcsapi_wifi_get_mode_switch(uint8_t *p_wifi_mode_switch_setting);


/**
 * @brief Force a disassociate on the STA.
 *
 * This API call ends the current association and puts the device into an idle state,
 * so that it does not send out probe requests or attempt to associate with any AP.
 *
 * \note This API only applies for a STA.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disassociate \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_disassociate(const char *ifname);

/**
 * @brief Force a disassociate on the AP. Station to disassociate identified by MAC address
 *
 * This API call ends association with remote station. It is a wrapper around hostapd's
 * "disassociate" function.
 *
 * \note This API only applies for an AP.
 *
 * \param ifname \wifi0
 * \param mac the MAC address of an associated station
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disassociate_sta \<WiFi interface\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_disassociate_sta(const char *ifname, qcsapi_mac_addr mac);

/**
 * @brief Force reassociation
 *
 * On AP reassociation of all clients is forced by disassociating them without deauthenticating.
 * For STA it forces reassociation without going through scan.
 * It is equivalent to calling command <c>iwpriv \<WiFi interface\> cl_remove 1</c>.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reassociate \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_reassociate(const char *ifname);

/**
 * @brief Get information of disconnection count and time since device boot up.
 *
 * This API is called to get disconnection count. Each time successfully invoke this API,
 * the sequence of disconn_info will increase 1, by which the caller can determine whether module
 * has reset. Note that disconnection count only increases when the peer is authorized and
 * disassocated later.
 *
 * In STA mode, the disconn_count of disconn_info indicates how many times STA is disconnected
 * from AP since module boot up, the asso_sta_count is 1 if it connects to AP or 0 if not.
 * In AP mode, the disconn_count records times that AP disconnects STA which has been associated
 * with it, the asso_sta_count means the number of STAs connect to AP.
 *
 * This API also can reset sequence, and asso_sta_count by setting member resetflag of disconn_info
 * with 1.
 *
 * \param ifname \wifi0only
 * \param disconn_info where to save the data.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_disconn_info \<WiFi interface\></c>
 * <c>call_qcsapi reset_disconn_info \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_disconn_info(const char *ifname, qcsapi_disconn_info *disconn_info);

/**
 * \brief Enable/Disable WPS
 *
 * Available on APs only. This API alters hostapd wps_state.
 *
 * \param ifname \wifi0
 * \param disable_wps 0 = set wps_state to configured. 1 = set wps_state to disabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disable_wps \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_disable_wps(const char *ifname, int disable_wps);

/**
 * @brief Associate a STA to a network.
 *
 * Cause the STA to attempt to associate with the selected SSID.
 *
 * The SSID must be present and configured on the STA.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param join_ssid the SSID for the STA to join.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi associate \<WiFi interface\> \<SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_associate(const char *ifname, const qcsapi_SSID join_ssid);

/**
 * @brief Associate a STA to a network.
 *
 * Cause the STA to attempt to associate with the selected SSID.
 *
 * The SSID must be present and configured on the STA.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param join_ssid the SSID for the STA to join.
 * \param fmt format type of the SSID
 * \param flags unused.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi associate \<WiFi interface\> [{str | hexstr}] \<SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_associate2(const char *ifname, const qcsapi_SSID2 join_ssid,
		qcsapi_ssid_fmt fmt, uint32_t flags);

/**
 * \brief Trigger CCA (Clear Channel Assessment) measurement
 *
 * This API causes a CCA measurement to be scheduled 1 second after invocation, on the requested
 * channel, for a requested number of milliseconds.
 *
 * \param ifname \wifi0
 * \param channel Channel to switch to during CCA measurement
 * \param duration Time in milliseconds to spend on the channel
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_cca \<WiFi interface\> <i>channel</i> <i>duration</i></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_start_cca(const char *ifname, int channel, int duration);

/**
 * @brief Get the noise figure for the current channel.
 *
 * This API reports the noise on the current channel in dBm.
 *
 * Since the noise is expected to be much less than 1 milliwatt (0 dBm), the value should be much
 * less than 0.
 *
 * \param ifname \wifi0only
 * \param p_noise return parameter to contain the noise figure read from the interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_noise \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current noise in dBm, most likely a negative
 * number.
 */
extern int qcsapi_wifi_get_noise(const char *ifname, int *p_noise);

/**
 * @brief Get the RSSI measurement per RF chain.
 *
 * This API reports the RSSI of the selected RF chain in dBm.
 *
 * \param ifname \wifi0only
 * \param rf_chain the RF chain to get the RSSI of (between 0 and 3).
 * \param p_rssi return parameter to contain the RSSI reading of the interface/RF chain pair.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rssi_by_chain \<WiFi interface\> \<RF Chain number\></c>
 *
 * Unless an error occurs, the output will be the RSSI of the selected RF chain in dBm.
 */
extern int qcsapi_wifi_get_rssi_by_chain(const char *ifname, int rf_chain, int *p_rssi);

/**
 * \brief Get the average SNR of the interface.
 *
 * \param ifname \wifi0only
 * \param p_snr return parameter to contain the average SNR of the interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_avg_snr \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the average SNR of the primary WiFi interface.
 */
extern int qcsapi_wifi_get_avg_snr(const char *ifname, int *p_snr);

/**
 * @brief Get the primary wireless interface.
 *
 * This API will return the name of the primary WiFi interface.
 *
 * The primary interface is usually the first AP-mode interface on radio created on system start up,
 * and is the only WiFi interface that allows configuration of the underlying properties of the
 * radio: channel, TX power, bandwidth, etc.
 *
 * The primary interface is thus distinct from any additional AP-mode virtual interfaces created as
 * part of the MBSSID feature, which do not allow these radio properties to be set.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_get_primary_interface.
 *
 * \param ifname \wifi0
 * \param maxlen the size of the buffer pointed to by ifname.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_primary_interface</c>
 *
 * Output should be the name of the primary interface unless an error occurs.
 */
extern int qcsapi_get_primary_interface(char *ifname, size_t maxlen);

/**
 * @brief Get the primary wireless interface for radio.
 *
 * This API will return the name of the primary WiFi interface.
 *
 * The primary interface is usually the first AP-mode interface on radio created on system start up,
 * and is the only WiFi interface that allows configuration of the underlying properties of the
 * radio: channel, TX power, bandwidth, etc.
 *
 * The primary interface is thus distinct from any additional AP-mode virtual interfaces created as
 * part of the MBSSID feature, which do not allow these radio properties to be set.
 *
 * \param radio_id \radio_id
 * \param ifname \wifi0
 * \param maxlen the size of the buffer pointed to by ifname.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_primary_interface \<radio_id\></c>
 *
 * Output should be the name of the primary interface unless an error occurs.
 */
extern int qcsapi_radio_get_primary_interface(const qcsapi_unsigned_int radio_id,
						char *ifname, size_t maxlen);

/**
 * @brief Get a wireless interface by index.
 *
 * This API will return the name of the WiFi interface that corresponds to the input parameters.
 *
 * For if_index = 0, this API will return the name of the primary interface for first radio device
 * in the system.
 *
 * For if_index \> 0, the API will return the corresponding secondary interface for first radio
 * device.
 *
 * If if_index exceeds the number of interfaces - 1, this API will return a range error indicating
 * the index parameter is too large.
 *
 * No holes in the list of entries will be present; starting at 0, for each consecutive value of
 * index, either a unique interface name is returned or the API returns range error.
 *
 * If for a particular value of index the API returns range error, the API will return the same
 * range error for all larger values of index.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_get_interface_by_index.
 *
 * \param if_index the interface index to get the name of.
 * \param ifname \wifi0
 * \param maxlen the size of the buffer pointed to by ifname.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interface_by_index \<index\></c>
 *
 * Output should be the name of the interface on first radio that corresponds to the index value.
 */
extern int qcsapi_get_interface_by_index(unsigned int if_index, char *ifname, size_t maxlen);

/**
 * @brief Get a wireless interface by index.
 *
 * This API will return the name of the WiFi interface that corresponds to the input parameters.
 *
 * For if_index = 0, this API will return the name of the primary interface.
 *
 * For if_index \> 0, the API will return the corresponding secondary interface.
 *
 * If if_index exceeds the number of interfaces - 1, this API will return a range error indicating
 * the index parameter is too large.
 *
 * No holes in the list of entries will be present; starting at 0, for each consecutive value of
 * index, either a unique interface name is returned or the API returns range error.
 *
 * If for a particular value of index the API returns range error, the API will return the same
 * range error for all larger values of index.
 *
 * \param if_index the interface index to get the name of.
 * \param radio_id \radio_id
 * \param ifname \wifi0
 * \param maxlen the size of the buffer pointed to by ifname.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interface_by_index \<index\> \<radio_id\></c>
 *
 * Output should be the name of the interface that corresponds to the index value of specified
 * radio.
 */
extern int qcsapi_radio_get_interface_by_index(const qcsapi_unsigned_int radio_id,
				unsigned int if_index, char *ifname, size_t maxlen);

/*
 * @brief Get a wireless interface by index.
 *
 * Return the name of the WiFi interface that corresponds to the input parameters.
 *
 * For if_index = 0, this API will return the name of the primary interface of the specified radio.
 *
 * For if_index \> 0, the API will return the corresponding secondary interface of the specified
 * radio.
 *
 * No holes in the list of entries will be present; starting at 0, for each consecutive
 * value of index, either a unique interface name is returned or the API returns -EINVAL for
 * range error.
 *
 * \param if_index the interface index to get the name of.
 * \param ifname \wifi0only
 * \param maxlen the size of the buffer pointed to by ifname.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interface_by_index_all \<index\> \<radio_id\></c>
 *
 * Output is the name of the interface
 */
extern int qcsapi_radio_get_interface_by_index_all(const qcsapi_unsigned_int radio_id,
	unsigned int if_index, char *ifname, size_t maxlen);

/**
 * \brief Set the primary WiFi interface MAC address.
 *
 * This API allows the primary WiFi interface MAC address to be set.
 *
 * \param new_mac_addr the new MAC address for the primary WiFi interface.
 *
 * \return \zero_or_negative
 *
 * \warning This API can ONLY be called after the QDRV is started, but before the WiFi mode is
 * selected.
 *
 * \warning This API cannot be used to change the WiFi MAC address dynamically - it can be called
 * only once, after the QDRV is started. To change the WiFi MAC address again, a reboot is required.
 *
 * \warning This API does NOT save the set WiFi MAC address across reboots. Additional logic is
 * required to save/restore the MAC address across reboots.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wifi_macaddr \<new MAC addr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_wifi_macaddr(const qcsapi_mac_addr new_mac_addr);

/**
 * @brief Get the BSSID.
 *
 * Return the Basic Service Set ID (BSSID).
 *
 * For an AP, this is the MAC address of the WiFi interface. For a STA, it is the MAC address of the
 * AP it is associated with. If the STA is not in association, the BSSID will be the address
 * <c>00:00:00:00:00:00</c>
 *
 * \param ifname \wifi0
 * \param current_BSSID return parameter to contain the BSSID.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_BSSID \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current BSSID, expressed in the standard MAC
 * address format.
 *
 * On a station not in association, the value will be <c>00:00:00:00:00:00</c>
 */
extern int qcsapi_interface_get_BSSID(const char *ifname, qcsapi_mac_addr current_BSSID);

/**
 * @brief Get the list of supported rates on the given interface.
 *
 * This API will list the supported rates (as a string), with each value in megabits per second,
 * separated by commas.
 *
 * These rates represent what the device is capable of; the return value is NOT affected by the
 * current rate setting.
 *
 * \param ifname \wifi0
 * \param rate_type set this to qcsapi_possible_rates.
 * \param supported_rates return parameter to contain the comma separated rate list.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rates \<WiFi interface\> possible_rates</c>
 *
 * Unless an error occurs, the output will be the set of possible rates supported by the interface.
 */
extern int qcsapi_wifi_get_rates(const char *ifname, qcsapi_rate_type rate_type,
					string_2048 supported_rates);

/**
 * \brief Set rates from an MBPS list (unsupported)
 *
 * Set rates from an MBPS list (unsupported)
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_wifi_rates.
 */
extern int qcsapi_wifi_set_rates(const char *ifname, qcsapi_rate_type rate_type,
					const string_256 current_rates, int num_rates);

/**
 * \brief Get the maximum downstream operational bit rate.
 *
 * \param ifname \wifi0
 * \param max_bitrate \valuebuf
 * \param max_str_len The size of the max_bitrate buffer.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_max_bitrate \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the maximum operational bit rate, in Mbps.
 */
extern int qcsapi_get_max_bitrate(const char *ifname, char *max_bitrate, const int max_str_len);

/**
 * \brief Set the maximum upstream and downstream bit rate available to this connection in Mbps.
 *
 * This API can only input "auto" currently.
 *
 * \param ifname \wifi0
 * \param max_bitrate String value of the max_bitrate, currently only "auto" can be set.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_max_bitrate \<WiFi interface\> \<max_bitrate\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_set_max_bitrate(const char *ifname, const char *max_bitrate);

/**
 * \brief Get a Quality of Service (QOS) Parameter
 *
 *
 * Returns the value of a Quality of Service Parameter, based on the selected queue.
 *
 * \param ifname \wifi0
 * \param the_queue which queue to use. Value ranges from 0 to 3, see section
 *	@ref mygroup5_4 "Quality of Service (QoS) extensions" for queue index to symbolic name
 *	mappings.
 * \param the_param which parameter to report.	Value ranges from 1 to 6, refer to sections
 *	@ref mysection5_4_1 "ECWMin" through @ref mysection5_4_6 "AckPolicy" for description and
 *	limitations.
 * \param ap_bss_flag set to "0" or "1" to report either egress (self) or ingress (BSS) QoS
 *	parameter respectively. Refer to section @ref mygroup5_4 "Quality of Service (QoS)
 *	extensions" for difference.
 * \param p_value address of the location to receive the value of the QOS parameter.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_qos_param \<WiFi interface\> {0 | 1 | 2 | 3} {1 | 2 | 3 | 4 | 6}
 *	[ {0 | 1} ]</c>
 */
extern int qcsapi_wifi_qos_get_param(const char *ifname, int the_queue, int the_param,
					int ap_bss_flag, int *p_value);

/**
 * \brief Set a Quality of Service (QOS) Parameter
 *
 * \param ifname \wifi0
 * \param the_queue which queue to use. Value ranges from 0 to 3, see section
 *	@ref mygroup5_4 "Quality of Service (QoS) extensions" for queue index to symbolic name
 *	mappings.
 * \param the_param which parameter to report. Value ranges from 1 to 6, refer to sections
 *	@ref mysection5_4_1 "ECWMin" through @ref mysection5_4_6 "AckPolicy" for description and
 *	limitations.
 * \param ap_bss_flag "0" or "1" to set either egress (self) or ingress (BSS) QoS parameter
 *	respectively. Refer to section @ref mygroup5_4 "Quality of Service (QoS) extensions" for
 *	difference.
 * \param value the value of the QOS parameter to set.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_qos_param \<WiFi interface\> {0 | 1 | 2 | 3} {1 | 2 | 3 | 4 | 6}
 *	{value} [ {0 | 1} ]</c>
 */
extern int qcsapi_wifi_qos_set_param(const char *ifname, int the_queue, int the_param,
					int ap_bss_flag, int value);

/**
 * \brief Get the mapping table from TOS/DSCP or IEEE802.1p priority to WMM AC index
 *
 * Get the current mapping table from TOS/DSCP or IEEE802.1p priority to WMM AC index.
 *
 * \param ifname \wifi0
 * \param mapping_table Return value to contain the mapping table for priorities. Must be
 * a memory area large enough to contain 64 byte values
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wmm_ac_map \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_wmm_ac_map(const char *ifname, string_64 mapping_table);

/**
 * \brief Set one mapping table item from TOS/DSCP or IEEE802.1p priority to WMM AC index
 *
 * \param ifname \wifi0
 * \param user_prio Which user priority to be set. Value ranges from 0 to 7.
 * \param ac_index The AC index to map to the input user_prio. Valid values are 0 to 3,
 * see @ref mygroup5_4 "Quality of Service (QoS) extensions" for correspondence between AC index and
 * AC symbolic name.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wmm_ac_map \<WiFi interface\> {0 | 1 | 2 | 3 | 4 | 5 | 6 | 7}
 *	{0 | 1 | 2 | 3}</c>
 */
extern int qcsapi_wifi_set_wmm_ac_map(const char *ifname, int user_prio, int ac_index);

/**
 * \brief Get the mapping table from IP DSCP to IEEE802.1p priority
 *
 * Returns the current mapping table from IP DSCP to IEEE802.1p user priority
 *
 * \param ifname \wifi0
 * \param mapping_table Return value to contain the mapping table for IEEE802.1p user priorities.
 * Must be a memory area large enough to contain 64 byte values
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_8021p_map \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_dscp_8021p_map(const char *ifname, string_64 mapping_table);

/**
 * \brief Get IP DSCP to WMM AC mapping table entries
 *
 * Returns the current mapping table from IP DSCP to WME AC user priority
 *
 * \param ifname \wifi0
 * \param mapping_table Return value to contain the mapping table for WME AC user priorities.
 * Must be a memory area large enough to contain 64 byte values.
 *
 * Return parameter to contain the mapping table for WME AC user priorities
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_ac_map \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_dscp_ac_map(const char *ifname,
				struct qcsapi_data_64bytes *mapping_table);

/**
 * \brief Set one mapping table item from IP DSCP to IEEE802.1p user priority
 *
 * \param ifname \wifi0
 * \param ip_dscp_list List of IP DSCP values to be set. Value ranges from 0 to 64 and
 * 64 is a special use to revert mapping table to default. If the IP DSCP needed to be set
 * is 40 and 46 the format should be 40,46.
 * \param dot1p_up the 802.1p UP to be mapped to the input IP DSCP. Value ranges from 0~7.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dscp_8021p_map \<WiFi interface\> \<0-64\>[,1-64]... \<0-7\></c>
 */
extern int qcsapi_wifi_set_dscp_8021p_map(const char *ifname, const char *ip_dscp_list,
					uint8_t dot1p_up);

/**
 * \brief Set one mapping table item from IP DSCP to WMM AC priority.
 *
 * Set one mapping table item from IP DSCP to WMM AC priority.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_qos_map \endlink
 *
 * \param ifname \wifi0
 * \param dscp_list List of IP DSCP value(s) to be set. Value ranges from 0 to 63.
 * If more than one IP DSCP mapping is specified, the values must be separated by commas.
 * \param dcsp_list_len The number of IP DSCP values in the dscp_list argument.
 * \param ac the WME AC value that will be mapped to the input IP DSCP. Value ranges from 0 to 3,
 * see @ref mygroup5_4 "Quality of Service (QoS) extensions" for correspondence between AC index and
 * AC symbolic name.
 *
 * \return \zero_or_negative
 *
 * \note Any DSCP mappings not explicitly set will map to AC_BE.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dscp_ac_map \<WiFi interface\> \<0-64\>[,0-64]... \<0-3\></c>
 *
 * As an example, to map the following:
 *
 * CS0,CS1,CS4,CS7 (map to AC_BE);
 *
 * AF11,CS2, AF21,CS3,CS6 (map to AC_VI); and
 *
 * CS5, EF (map to AC_VO)
 *
 * the following set of call_qcsapi calls are made:
 *
 * @code
 * call_qcsapi set_dscp_ac_map wifi0_0 0,8,32,56 0
 * call_qcsapi set_dscp_ac_map wifi0_0 10,16,18,24,48 2
 * call_qcsapi set_dscp_ac_map wifi0_0 40,46 3
 * @endcode
 */
extern int qcsapi_wifi_set_dscp_ac_map(const char *ifname,
			const struct qcsapi_data_64bytes *dscp_list,
			uint8_t dscp_list_len, uint8_t ac);

/**
 * \brief Get aggregation hold time for an Access Class
 *
 * Get aggregation hold time for an Access Class.
 *
 * \param ifname \wifi0
 * \param ac WME Access Class - 0 (best effort), 1 (background), 2 (video) or 3 (voice)
 * \param p_agg_hold_time \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ac_agg_hold_time \<WiFi interface\> \<ac\></c>
 *
 * The output will be the aggregation hold time in us unless an error occurs.
 */
extern int qcsapi_wifi_get_ac_agg_hold_time(const char *ifname, uint32_t ac,
				uint32_t *p_agg_hold_time);

/**
 * \brief Set aggregation hold time for an Access Class
 *
 * Set aggregation hold time for an Access Class.
 *
 * \param ifname \wifi0
 * \param ac WME Access Class - 0 (best effort), 1 (background), 2 (video) or 3 (voice)
 * \param agg_hold_time aggregation hold time (0 - 1073741823). This value is in microseconds.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ac_agg_hold_time \<WiFi interface\> \<ac\> \<agg_hold_time\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_ac_agg_hold_time(const char *ifname, uint32_t ac,
					uint32_t agg_hold_time);

/**
 * \brief Set QoS Map Set
 *
 * Configure QoS Map Set information element. Refer to IEEE802.11-2012, 8.4.2.97 for further
 * description. This also configures DSCP to TID mapping to be used by AP.
 *
 * QoS Map Set string have the following format:
 * @code
 * [<DSCP Exceptions[DSCP,UP]>,] <UP 0 range[low,high]>,...<UP 7 range[low,high]>
 * @endcode
 *
 * There can be up to 21 optional DSCP Exceptions which are pairs of DSCP Value
 * (0..63 or 255) and User Priority (0..7). This is followed by eight DSCP Range
 * descriptions with DSCP Low Value and DSCP High Value pairs (0..63 or 255) for
 * each UP starting from 0. If both low and high value are set to 255, the
 * corresponding UP is not used.
 *
 * \param ifname \wifi0
 * \param qos_map_str QoS Map Set string
 *
 * \note This API supersedes qcsapi_wifi_set_dscp_ac_map that configures the same
 * DSCP to TID mapping. If TID cannot be used it will be replaced by the one mapped to the same AC.
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_qos_map \<WiFi interface\> \<qos_map_set\></c>
 *
 * Example:
 * @code
 * call_qcsapi set_qos_map wifi0_0 53,2,22,6,8,15,0,7,255,255,16,31,32,39,255,255,40,47,255,255
 * @endcode
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_qos_map(const char *ifname, const char *qos_map_str);

/**
 * \brief Remove QoS Map Set
 *
 * Remove QoS Map Set parameter.
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_qos_map \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_qos_map(const char *ifname);

/**
 * \brief Get QoS Map Set
 *
 * Get the value of QoS Map Set. Format is described in \link qcsapi_wifi_set_qos_map \endlink.
 *
 * \param ifname \wifi0
 * \param value pointer to buffer to store returned value
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_qos_map \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of QoS Map Set</c>.
 */
extern int qcsapi_wifi_get_qos_map(const char *ifname, string_256 value);

/**
 * \brief Send QoS Map Configure action frame to a station
 *
 * Send QoS Map Configure action frame to an associated station specified by MAC address.
 *
 * \param ifname \wifi0
 * \param sta_mac_addr STA MAC address
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi send_qos_map_conf \<WiFi interface\> \<STA MAC addr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_send_qos_map_conf(const char *ifname, const qcsapi_mac_addr sta_mac_addr);

/**
 * \internal
 * \brief Get DSCP to TID mapping
 *
 * Get the value of DSCP to TID mapping. The returned buffer will contain a 64-byte array
 * of 8-bit unsigned integers, which are the TID values for each DSCP from 0 to 63.
 *
 * \param ifname \wifi0only
 * \param dscp2tid_ptr pointer to buffer to store returned value
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_tid_map \<WiFi interface\></c>
 *
 * Unless an error occurs the output will be the header:
 * DSCP: TID
 * followed by corresponding \<DSCP\>: \<TID\> values on separate lines.
 */
extern int qcsapi_wifi_get_dscp_tid_map(const char *ifname,
		struct qcsapi_data_64bytes *dscp2tid_ptr);

/**
 * \brief Get the interface priority
 *
 * Get the priority for the given WiFi interface. The priority is used to differentiate traffic
 * between different SSIDs. Traffic in SSIDs with higher priority takes precedence over traffic in
 * SSIDs with lower priority.
 *
 * \param ifname \wifi0
 * \param p_priority a pointer to the buffer for storing the returned value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_priority \<WiFi interface\></c>
 *
 * The output will be the priority unless an error occurs.
 */
extern int qcsapi_wifi_get_priority(const char *ifname, uint8_t *p_priority);

/**
 * \brief Set the interface priority
 *
 * Set the priority for the given WiFi interface. The priority is used to differentiate traffic
 * between different SSIDs. Traffic in SSIDs with higher priority takes precedence over traffic in
 * SSIDs with lower priority.
 *
 * \param ifname \wifi0
 * \param priority interface priority. Value ranges from 0 to 3.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_priority \<WiFi interface\> \<priority\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_priority(const char *ifname, uint8_t priority);

/**
 * \brief Get the airtime fairness status
 *
 * Get the airtime fairness status for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param p_airfair a pointer to the buffer for storing the returned value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_airfair \<WiFi interface\></c>
 *
 * The output will be the status of airtime fairness unless an error occurs. 0 means diabled,
 * and 1 means enabled.
 */
extern int qcsapi_wifi_get_airfair(const char *ifname, uint8_t *p_airfair);

/**
 * \brief Set airtime fairness
 *
 * Set the airtime fairness for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param airfair interface airfair status. Value is either 0(disable) or 1(enable).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_airfair \<WiFi interface\> \<airfair\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_airfair(const char *ifname, uint8_t airfair);

/**
 * \brief Get the airtime quota
 *
 * Get the airtime quota for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param p_airquota a pointer to the buffer for storing the returned value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_airquota \<WiFi interface\></c>
 *
 * The output will be the airtime quota unless an error occurs. Airtime quota ranges from 0 to 1000
 * with unit of 1 thousandth. 1000 means unlimited.
 */
extern int qcsapi_wifi_get_airquota(const char *ifname, uint32_t *p_airquota);

/**
 * \brief Set airtime quota
 *
 * Set the airtime quota for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param airquota interface quota. Value ranges from 0 to 1000 with unit of 1 thousandth. 1000
 * means unlimited.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_airquota \<WiFi interface\> \<airquota\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_airquota(const char *ifname, uint32_t airquota);

/**
 * @brief Get premier family MAC address list.
 *
 * Report registered referenced MAC addresses (up to 8) which are in premier family.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param pnum a pointer to the buffer for storing the returned the number of MAC address
 * \param address_list the MAC addresses buffer
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_premier_list \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the MAC addresses in the list.
 */
extern int qcsapi_wifi_get_premier_list(const char *ifname, int *pnum,
					qcsapi_mac_addr_list address_list);

/**
 * @brief Set premier family MAC address list.
 *
 * Register referenced MAC addresses (up to 8) as in premier family.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param num number of MAC address
 * \param address_list the MAC addresses
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_premier_list \<WiFi interface\>
 *	\<MAC address\> [MAC address1] ... [MAC address7]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_premier_list(const char *ifname, const int num,
					const qcsapi_mac_addr_list address_list);


/**
 * \brief Get premier family throttling rule
 *
 * Get premier family throttling rule for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param p_rule a pointer to the buffer for storing the returned value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_premier_rule \<WiFi interface\></c>
 *
 * The output will be the premier value unless an error occurs.
 */
extern int qcsapi_wifi_get_premier_rule(const char *ifname, qcsapi_qos_premier_rule *p_rule);

/**
 * \brief Set premier family throttling rule
 *
 * Set the premier family throttling rule for the given WiFi interface.
 *
 * \param ifname \wifi0
 * \param rule interface premier family throttling rule.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_premier_rule \<WiFi interface\> \<rule\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_premier_rule(const char *ifname, qcsapi_qos_premier_rule rule);

/**
 * \brief Get the transmit power for the current bandwidth with beamforming off.
 *
 * This API returns the transmit power for the specified channel, which is the
 * maximum allowed power used in the case of beamforming off and 1 spatial stream
 * for current bandwidth.
 * The TX powers are reported in dBm.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is returned.
 * \param p_tx_power return parameter to contain the transmit power.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_power \<WiFi interface\> \<channel\></c>
 *
 * Unless an error occurs, the output will be the transmit power.
 */
extern int qcsapi_wifi_get_tx_power(const char *ifname,
					const qcsapi_unsigned_int the_channel,
					int *p_tx_power);

/**
 * \brief Set the transmit power for the current bandwidth.
 *
 * This API call sets the transmit power for a particular channel.
 * The TX power is in dBm unit.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_set.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is set.
 * \param tx_power tx power to be set.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_tx_power \<WiFi interface\> \<channel\> \<tx_power\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_tx_power(const char *ifname,
					const qcsapi_unsigned_int the_channel,
					const int tx_power);

/**
 * \brief Get the transmit powers for 20/40/80MHz bandwidths with beamforming off.
 *
 * This API returns the transmit powers for the specified channel, which is the maximum
 * allowed powers used in the case of beamforming off and 1 spatial stream.
 * The TX powers are reported in dBm.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is returned.
 * \param p_power_20M return parameter to contain the transmit power for 20MHz bandwidth.
 * \param p_power_40M return parameter to contain the transmit power for 40MHz bandwidth.
 * \param p_power_80M return parameter to contain the transmit power for 80MHz bandwidth.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bw_power \<WiFi interface\> \<channel\></c>
 *
 * Unless an error occurs, the output will be the transmit powers.
 */
extern int qcsapi_wifi_get_bw_power(const char *ifname, const qcsapi_unsigned_int the_channel,
					int *p_power_20M, int *p_power_40M, int *p_power_80M);

/**
 * \brief Set the transmit power for 20/40/80MHz bandwidths.
 *
 * This API call sets the transmit powers for a particular channel.
 * For 5G channels, power is set on FEM0, while for 2.4GHz channel (which use single FEM), power
 * is set for currently configured primary channel position.
 * The TX powers are in dBm unit.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_set.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is set.
 * \param power_20M tx power for 20MHz bandwidth to be set.
 * \param power_40M tx power for 40MHz bandwidth to be set.
 * \param power_80M tx power for 80MHz bandwidth to be set.
 *
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bw_power \<WiFi interface\> \<channel\>
 *	\<power_20M\> \<power_40M\> \<power_80M\></c>
 *
 * \call_qcsapi_string_complete.
 */
extern int qcsapi_wifi_set_bw_power(const char *ifname, const qcsapi_unsigned_int the_channel,
			const int power_20M, const int power_40M, const int power_80M);

/**
 * \brief Get the transmit powers for 20/40/80MHz bandwidths with beamforming on.
 *
 * This API returns the transmit powers for the specified channel, which is the maximum
 * allowed powers used in the case of beamforming is on.
 * The TX powers are reported in dBm.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is returned.
 * \param number_ss the number of spatial streams.
 * \param p_power_20M return parameter to contain the transmit power for 20MHz bandwidth.
 * \param p_power_40M return parameter to contain the transmit power for 40MHz bandwidth.
 * \param p_power_80M return parameter to contain the transmit power for 80MHz bandwidth.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bf_power \<WiFi interface\> \<channel\> \<number_ss\></c>
 *
 * Unless an error occurs, the output will be the transmit powers.
 */
extern int qcsapi_wifi_get_bf_power(const char *ifname, const qcsapi_unsigned_int the_channel,
					const qcsapi_unsigned_int number_ss,
					int *p_power_20M, int *p_power_40M, int *p_power_80M);

/**
 * \brief Set the transmit power for 20/40/80MHz bandwidths.
 *
 * This API call sets the transmit powers for a particular channel and number of spatial
 * streams.
 * For 5G channels, power is set on FEM0, while for 2.4GHz channel (which use single FEM), power
 * is set for currently configured primary channel position.
 * TX powers are in dBm units.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_set.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is set.
 * \param number_ss the number of spatial streams.
 * \param power_20M tx power for 20MHz bandwidth to be set.
 * \param power_40M tx power for 40MHz bandwidth to be set.
 * \param power_80M tx power for 80MHz bandwidth to be set.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bf_power \<WiFi interface\> \<channel\> \<number_ss\>
 *	\<power_20M\> \<power_40M\> \<power_80M\></c>
 *
 * \call_qcsapi_string_complete.
 */
extern int qcsapi_wifi_set_bf_power(const char *ifname, const qcsapi_unsigned_int the_channel,
			const qcsapi_unsigned_int number_ss,
			const int power_20M, const int power_40M, const int power_80M);

/**
 * \brief Get the transmit powers for 20/40/80MHz bandwidths.
 *
 * This API returns the transmit powers for a specified channel, which is the maximum
 * allowed powers used in the specified case.
 * The TX powers are reported in dBm.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is returned.
 * \param bf_on beamforming is either on or off. 1 for beamforming on and 0 for beamforming off.
 * \param number_ss the number of spatial streams.
 * \param p_power_20M return parameter to contains the transmit power for 20MHz bandwidth.
 * \param p_power_40M return parameter to contains the transmit power for 40MHz bandwidth.
 * \param p_power_80M return parameter to contains the transmit power for 80MHz bandwidth.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_power_ext \<WiFi interface\> \<channel\> \<bf_on(1/0)\> \<number_ss\></c>
 *
 * Unless an error occurs, the output will be the transmit powers.
 */
extern int qcsapi_wifi_get_tx_power_ext(const char *ifname, const qcsapi_unsigned_int the_channel,
					const qcsapi_unsigned_int bf_on,
					const qcsapi_unsigned_int number_ss,
					int *p_power_20M, int *p_power_40M, int *p_power_80M);

/**
 * \brief Set the transmit power for 20/40/80MHz bandwidths.
 *
 * This API call sets the transmit powers for a particular channel, number of Spatial Streams
 * and Beamforming ON/OFF status.
 * For 5G channels, power is set on FEM0, while for 2.4GHz channel (which use single FEM), power
 * is set for currently configured primary channel position.
 * The TX powers are in dBm unit.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_set.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param the_channel the channel for which the tx power is set.
 * \param bf_on beamforming is either on or off. 1 for beamforming on and 0 for beamforming off.
 * \param number_ss the number of spatial streams.
 * \param power_20M tx power for 20MHz bandwidth to be set.
 * \param power_40M tx power for 40MHz bandwidth to be set.
 * \param power_80M tx power for 80MHz bandwidth to be set.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_tx_power_ext \<WiFi interface\> \<channel\> \<bf_on(1/0)\> \<number_ss\>
 *	\<power_20M\> \<power_40M\> \<power_80M\></c>
 *
 * \call_qcsapi_string_complete.
 */
extern int qcsapi_wifi_set_tx_power_ext(const char *ifname, const qcsapi_unsigned_int the_channel,
			const qcsapi_unsigned_int bf_on, const qcsapi_unsigned_int number_ss,
			const int power_20M, const int power_40M, const int power_80M);

/**
 * \brief Get all the transmit powers for a single channel on device with 4 Tx chains.
 *
 * This API returns all the transmit powers for a specified channel.
 * The TX powers are reported in dBm.
 *
 * See the documentation for \ref qcsapi_channel_power_table for full
 * details of the return structure.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \param ifname \wifi0only
 * \param chan_power_table the pointer to a data structure which is used to store the return powers.
 * the variable "channel" of this structure must be initiated before calling this API.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_power_table \<WiFi interface\> \<channel\></c>
 *
 * Unless an error occurs, the output will be the power table of this channel.
 *
 * \sa qcsapi_channel_power_table
 */
extern int qcsapi_wifi_get_chan_power_table(const char *ifname,
		qcsapi_channel_power_table *chan_power_table);

/**
 * \brief Set all the transmit powers for a single channel.
 *
 * This API sets all the transmit powers for a particular channel.
 * The TX powers are in dBm unit.
 *
 * See the documentation for \ref qcsapi_channel_power_table for full
 * details of how to fill out the channel power table.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_set.
 *
 * \note \nonpersistent
 *
 * \param ifname \wifi0only
 * \param chan_power_table the pointer to a data structure which has the channel number and powers.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_power_table \<WiFi interface\> \<channel\> \<max_power\> \<backoff_20M\>
 *	\<backoff_40M\> \<backoff_80M\> \<backoff_160M\></c>
 *
 * <c>max_power</c> is the maximum power of this channel's 24 powers.
 *
 * <c>backoff_20M</c> is a 32 bits unsigned value, and every 4 bits indicate the backoff from the
 * <c>max_power</c> for a BF/SS case.
 *
 * The least significant 4 bits are for BF off 1SS case, and the most significant 4 bits are for BF
 * on 4SS case.
 *
 * For the sequence, see the enumeration definition for qcsapi_power_indices
 * (\ref qcsapi_power_indices). For example, max_power 23 and backoff_20M 0x54324321 give the powers
 * as below:
 *
 * - the power for 20Mhz bfoff 1ss: 23 - 1 = 22dBm
 * - the power for 20Mhz bfoff 2ss: 23 - 2 = 21dBm
 * - the power for 20Mhz bfoff 3ss: 23 - 3 = 20dBm
 * - the power for 20Mhz bfoff 4ss: 23 - 4 = 19dBm
 * - the power for 20Mhz bfon 1ss: 23 - 2 = 21dBm
 * - the power for 20Mhz bfon 2ss: 23 - 3 = 20dBm
 * - the power for 20Mhz bfon 3ss: 23 - 4 = 19dBm
 * - the power for 20Mhz bfon 4ss: 23 - 5 = 18dBm
 *
 * <c>backoff_40M</c>, <c>backoff_80M</c> <c>backoff_160M</c> use the same format as
 * <c>backoff_20M</c>, and they are the backoff for 40Mhz, 80Mhz and 160Mhz respectively.
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_channel_power_table
 */
extern int qcsapi_wifi_set_chan_power_table(const char *ifname,
					qcsapi_channel_power_table *chan_power_table);

/**
 * \brief Get the current mode for selecting power table.
 *
 * Get the current mode for selecting power table.
 *
 * \param p_power_selection a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_power_selection</c>
 *
 * The output will be the mode for selecting power table number unless an error occurs.
 */
extern int qcsapi_wifi_get_power_selection(qcsapi_unsigned_int *p_power_selection);

/**
 * \brief Set the mode for selecting power table.
 *
 * Set the mode for selecting power table.
 *
 * \param power_selection the mode to be used
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_power_selection \<power_selection\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_power_selection(const qcsapi_unsigned_int power_selection);

/**
 * \brief Get the configured beacon TX power backoff
 *
 * Get the configured beacon TX power backoff.
 *
 * \param ifname \wifi0
 * \param backoff TX power backoff in dB.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_beacon_power_backoff \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_beacon_power_backoff(const char *ifname, qcsapi_unsigned_int *backoff);

/**
 * \brief Set the backoff for the beacon TX power
 *
 * Set the backoff for the beacon TX power
 *
 * \param ifname \wifi0
 * \param backoff the backoff in dB. The maximum backoff is 20, and the backoff had better be
 * a even number for 2dB step size, otherwise the actual TX power backoff may be (backoff - 1).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_beacon_power_backoff \<WiFi interface\> \<backoff\></c>
 */
extern int qcsapi_wifi_set_beacon_power_backoff(const char *ifname, qcsapi_unsigned_int backoff);

/**
 * \brief Get the configured management frame TX power backoff
 *
 * Get the configured management frame TX power backoff.
 *
 * \param ifname \wifi0
 * \param backoff TX power backoff in dB.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mgmt_power_backoff \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_mgmt_power_backoff(const char *ifname, qcsapi_unsigned_int *backoff);

/**
 * \brief Set the backoff for the management frame TX power
 *
 * Set the backoff for the management frame TX power
 *
 * \param ifname \wifi0
 * \param backoff the backoff in dB. The maximum backoff is 20, and the backoff had better be
 * a even number for 2dB step size, otherwise the actual TX power backoff may be (backoff - 1).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mgmt_power_backoff \<WiFi interface\> \<backoff\></c>
 */
extern int qcsapi_wifi_set_mgmt_power_backoff(const char *ifname, qcsapi_unsigned_int backoff);

/**
 * \brief Get Carrier/Interference.
 *
 * This API is used to get Carrier/Interference (db).
 *
 * \param ifname \wifi0
 * \param ci return the Carrier/Interference in db unit.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_carrier_db \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the
 * Carrier/Interference in db unit.
 */
extern int qcsapi_wifi_get_carrier_interference(const char *ifname,
			int *ci);

/**
 * \brief Get congestion index.
 *
 * This API is used to get current congestion index.
 *
 * \param ifname \wifi0
 * \param ci return value to contain the congestion index.
 * The congestion index is in the range 0 - 10.
 *
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_congest_idx \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the congestion index.
 */
extern int qcsapi_wifi_get_congestion_index(const char *ifname,
			int *ci);

/**
 * \brief Get the Supported TX Power Levels (as a list of percentages of the maximum allowed)
 *
 * This API reports the supported transmit power levels on the current operating channel
 * as a list of percentages of the maximum allowed by the regulatory authority.
 * A regulatory region must have been configured with the Set Regulatory Region API.
 *
 * \param ifname \wifi0only
 * \param available_percentages address of a string to recve the list of available percentages.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_supported_tx_power \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of percentages of the maximum allowed
 * by the regulatory authority.
 */
extern int qcsapi_wifi_get_supported_tx_power_levels(const char *ifname,
						string_128 available_percentages);
/**
 * \brief Get the Current TX Power Level (as a percentage of the maximum allowed)
 *
 * This API reports the transmit power on the current operating channel as a percentage of
 * the maximum allowed by the regulatory authority. A regulatory region must have
 * been configured with the Set Regulatory Region API.
 *
 * \param ifname \wifi0only
 * \param p_current_percentage return parameter to contains the percentage
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_current_tx_power \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the percentage of the maximum allowed
 * by the regulatory authority.
 */
extern int qcsapi_wifi_get_current_tx_power_level(const char *ifname,
						uint32_t *p_current_percentage);

/**
 * \brief Set power constraint of current channel
 *
 * This API sets the power constraint on the current channel. Power constraint will be
 * filled in power constraint element of beacon and probe response when spectrum management enabled.
 *
 * \param ifname \wifi0only
 * \param pwr_constraint power constraint to set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_power_constraint \<WiFi interface\> \<power constraint\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_power_constraint(const char *ifname, uint32_t pwr_constraint);

/**
 * \brief Get power constraint of current channel
 *
 * This API returns the power constraint on the current channel.
 *
 * \param ifname \wifi0only
 * \param p_pwr_constraint return parameter to contains power constraint
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_power_constraint \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the power constraint on the current channel.
 */
extern int qcsapi_wifi_get_power_constraint(const char *ifname, uint32_t *p_pwr_constraint);

/**
 * \brief Set the TPC interval if 802.11h is enabled
 *
 * Set the TPC interval if 802.11h is enabled and the periodical method is used.
 *
 * \param ifname \wifi0only
 * \param tpc_interval interval for tpc request to set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_tpc_interval \<WiFi interface\> \<tpc_interval\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_tpc_interval(const char *ifname, int32_t tpc_interval);

/**
 * \brief Get the TPC interval if 802.11h is enabled
 *
 * Get the TPC interval if 802.11h is enabled and the periodical method is used.
 *
 * \note This API is available in AP/STA/WDS modes.
 *
 * \param ifname \wifi0only
 * \param tpc_interval interval for tpc request to set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tpc_interval \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_get_tpc_interval(const char *ifname, uint32_t *p_tpc_interval);

/**
 * \brief Get the record of nodes that have associated with the device.
 *
 * This API reports all remote STAs that have associated with the local AP.
 * The MAC address of the WiFi interface is reported together with the last time
 * that STA associated. The STA must pass the 4-way handshake to be included;
 * that is, its security credentials must be correct. Only one entry will be
 * present for a particular STA, based on its MAC address.
 *
 * \param ifname \wifi0only
 * \param reset set this to 1 to clear the association records.
 * The current set of association records will be returned.
 * \param records address of the data structure to receive the association records.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_assoc_records \<WiFi interface\></c>
 *
 * Unless an error occur, the output will be a list of all remote STAs that have
 * associated with the device, together with the time they last associated.
 *
 */
extern int qcsapi_wifi_get_assoc_records(const char *ifname, int reset,
						qcsapi_assoc_records *records);

/**
 * @brief Get the global AP isolation setting
 *
 * Get the current global AP isolation setting
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param p_ap_isolate return parameter to contain the current global AP isolation setting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ap_isolate \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be 0, 1, representing the qcsapi_ap_isolate_type values.
 */
extern int qcsapi_wifi_get_ap_isolate(const char *ifname, int *p_ap_isolate);

/**
 * @brief Set the global AP isolation for all WiFi interfaces
 *
 * Enable or disable AP isolation global control (applies across all BSSes).
 *
 * When AP isolation is enabled, packets are not bridged between stations in the same BSS.
 * When AP isolation is disabled, packets can be bridged between stations in the same BSS.
 * AP isolation is disabled by default.

 * \note \aponly
 * \note When enabled, this API disables AP isolation on all BSSes, regardless of the
 * individual interface configuration set using <c>set_intra_bss_isolate</c>
 *
 * \param ifname \wifi0only
 * \param new_ap_isolate the new AP isolation setting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ap_isolate \<WiFi interface\> {0 | 1}</c>
 *
 * \sa qcsapi_ap_isolate_type
 * \sa qcsapi_wifi_set_intra_bss_isolate
 * \sa qcsapi_wifi_set_bss_isolate
 */
extern int qcsapi_wifi_set_ap_isolate(const char *ifname, const int new_ap_isolate);

/**
 * @brief Get the current intra-BSS isolation setting.
 *
 * This API returns the current intra-BSS isolation setting for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_ap_isolate return parameter to contain the the current intra-BSS isolation setting.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_intra_bss_isolate \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current intra-BSS isolation setting for the given
 * interface.
 */
extern int qcsapi_wifi_get_intra_bss_isolate(const char *ifname, qcsapi_unsigned_int *p_ap_isolate);

/**
 * @brief Enable or disable intra-BSS isolation per BSS.
 *
 * This API configures intra-BSS isolation. When enabled for a BSS, packets will not be forwarded
 * from one station associated on the BSS to any other stations associated on the same BSS.
 *
 * \note \aponly
 * \note This setting is overridden by the global qcsapi_wifi_set_ap_isolate setting, if enabled.
 *
 * \param ifname \wifi0
 * \param new_ap_isolate the new intra-BSS isolation setting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_intra_bss_isolate \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_ap_isolate_type
 * \sa qcsapi_wifi_set_bss_isolate
 * \sa qcsapi_wifi_set_ap_isolate
 */
extern int qcsapi_wifi_set_intra_bss_isolate(const char *ifname,
					const qcsapi_unsigned_int new_ap_isolate);

/**
 * @brief Get the current inter-BSS isolation setting.
 *
 * This API returns the current inter-BSS isolation setting for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_ap_isolate return parameter to contain the the current BSS isolation setting.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bss_isolate \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current BSS isolation setting.
 */
extern int qcsapi_wifi_get_bss_isolate(const char *ifname, qcsapi_unsigned_int *p_ap_isolate);

/**
 * @brief Enable or disable inter-BSS isolation.
 *
 * This API configures inter-BSS isolation. When enabled for a BSS, packets will not be forwarded
 * from a station to any station associated on a different BSS.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param new_ap_isolate the new BSS isolation setting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bss_isolate \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_ap_isolate_type
 * \sa qcsapi_wifi_set_ap_isolate
 * \sa qcsapi_wifi_set_intra_bss_isolate
 */
extern int qcsapi_wifi_set_bss_isolate(const char *ifname,
					const qcsapi_unsigned_int new_ap_isolate);

/**
 * \brief Disable DFS channels
 *
 * Configures the list of channels permitted during operation
 *
 * \param ifname \wifi0
 * \param disable_dfs disable dfs channels
 * \param channel channel to switch after dfs channels are disabled
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disable_dfs_channels wifi0_0 {0 | 1} [new channel]</c>
 *
 */
extern int qcsapi_wifi_disable_dfs_channels(const char *ifname, int disable_dfs, int channel);

/**
 * \brief Get the current DFS channels status.
 *
 * This API call returns the current status of the specified device radio.
 *
 * \param ifname wifiY_X: indicate will return status of RadioY
 * \param p_current_status: a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dfs_channels_status \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be 1(enabled) or 0(disabled).
 */
extern int qcsapi_wifi_get_dfs_channels_status(const char *ifname,
					qcsapi_unsigned_int *p_current_status);

/**
 * \brief Get WiFi ready state
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_is_iface_ready.
 */
extern int qcsapi_wifi_is_ready(qcsapi_unsigned_int *p_value);

/**
 * \brief Get WiFi ready state
 *
 * This API call is used to retrieve the WiFi ready state.
 *
 * \param ifname \wifi0only
 * \param p_value the WiFi state, 1 is ready, 0 is not ready
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_wifi_ready \<iface\></c>
 *
 * Unless an error occurs, the output will be 1 or 0.
 */
extern int qcsapi_wifi_is_iface_ready(const char *ifname, qcsapi_unsigned_int *p_value);

/**
 * \brief Get node airtime quota
 *
 * This API call is used to get the node airtime quota table for the VAP.
 *
 * \param ifname \wifi0
 * \param p_buffer a pointer to the buffer for storing the returned data
 * \param length buffer length
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_airquota_node \<WiFi interface\></c>
 *
 * The output will be a string contains MAC address and airtime quota pairs unless an error occurs.
 * Airquota (0 - 1000) - 0 means no quota and 1000 means unlimited quota
 */
extern int qcsapi_wifi_get_airquota_node(const char *ifname, char *p_buffer, const uint32_t length);

/**
 * \brief Set node airtime quota
 *
 * Set the airtime quota for the specified node.
 *
 * \param ifname \wifi0
 * \param mac_addr the node's MAC address
 * \param airquota node quota (0 - 1000) - 0 means no quota and 1000 means unlimited quota
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_airquota_node \<WiFi interface\> {\<MAC address\> | NULL} [ \<airquota\> ]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_airquota_node(const char *ifname, const qcsapi_mac_addr mac_addr,
					const uint32_t airquota);

/**
 * \brief Get TX QoS hold-time table value
 *
 * Get the TX Qos hold-time table value.
 *
 * \param ifname \wifi0
 * \param txqos_sched \valuebuf
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_txqos_sched_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, output will be the returned integer value.
 */
extern int qcsapi_wifi_get_txqos_sched_tbl(const char *ifname, int *txqos_sched);

/**
 * \brief Set TX QoS hold-time table value
 *
 * Set the TX Qos hold-time table value.
 *
 * \param ifname \wifi0
 * \param index contains the access catagory value to be set. Value ranges from 0 to 15.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_txqos_sched_tbl \<WiFi interface\> \<index\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_txqos_sched_tbl(const char *ifname, const int index);

/**
 * \brief Set PTA operation mode
 *
 * This function is used to set PTA operation mode
 *
 * \param ifname \wifi0
 * \param pta_mode 0 - disable PTA 10000 - mode 0 10001 - mode 1 10002 - mode 2
 *
 * \note This API can only be used on a 2.4GHz radio interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pta wifi2_0 \<pta_mode\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pta_op_mode(const char *ifname, qcsapi_unsigned_int pta_mode);

/**
 * \brief Get PTA configuration
 *
 * Get the Packet Traffic Arbitration (PTA) value.
 *
 * \param ifname \wifi0only
 * \param pta \valuebuf
 *
 * \note This API can only be used on a 2.4GHz radio interface.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pta \<WiFi interface\></c>
 *
 * Unless an error occurs, output will be \disable_or_enable.
 */
extern int qcsapi_wifi_get_pta(const char *ifname, uint32_t *pta);

/** @} */

/**
 * @addtogroup MBSSIDAPIs
 * @{
 */

/**
 * @brief Create a new restricted BSS.
 *
 * Creates a new MBSSID AP-mode virtual interface with a set of default security parameters.
 * This new virtual interface isn't added into back-end bridge, just for test.
 *
 * After calling this API function the host AP security daemon configuration is updated, reloaded
 * and the new BSS is made active. Subsequent security and general interface APIs can then be called
 * on the new virtual interface.
 *
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \param ifname \wifi0
 * \param mac_addr mac address of the created vap. If set to 0, driver will generate one mac address
 *	based on primary interface mac address.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wifi_create_restricted_bss \<WiFi interface\> [mac_addr]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_create_restricted_bss(const char *ifname, const qcsapi_mac_addr mac_addr);

/**
 * @brief Create a new BSS.
 *
 * Creates a new MBSSID AP-mode virtual interface with a set of default security parameters.
 *
 * After calling this API function the host AP security daemon configuration is updated, reloaded
 * and the new BSS is made active. Subsequent security and general interface APIs can then be called
 * on the new virtual interface.
 *
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \param ifname \wifi0
 * not be present in the hostapd configuration file.</b>
 * \param mac_addr mac address of the created vap. If set to 0, driver will generate one mac address
 *	based on primary interface mac address.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wifi_create_bss \<WiFi interface\> [mac_addr]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_create_bss(const char *ifname, const qcsapi_mac_addr mac_addr);

/**
 * @brief Remove a BSS.
 *
 * Removes an existing MBSSID AP-mode virtual interface with interface name ifname.
 *
 * The API will return an error if the named interface does not exist.
 *
 * After calling this function, the host AP security daemon configuration is modified, reloaded and
 * the interface named is no longer active.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wifi_remove_bss \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_bss(const char *ifname);

/** @} */

/**
 * @addtogroup WDSAPIs
 * @{
 */

/**
 * @brief Add a WDS peer
 *
 * Add a WDS peer.
 *
 * \note This API allows unencrypted data to be transmitted across the WDS link as soon as it has
 * been established. If the link will be encrypted, use \link qcsapi_wds_add_peer_encrypt \endlink
 * instead.
 *
 * \param ifname \wifi0only
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_add_peer \<WiFi interface\> \<BSSID of peer AP\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wds_add_peer(const char *ifname, const qcsapi_mac_addr peer_address);

/**
 * @brief Add a WDS peer with an encrypted link
 *
 * Add a WDS peer, and block traffic over the link until the link has been encrypted.
 *
 * \param ifname \wifi0only
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 * \param encryption 0 if the link will not be encrypted, or 1 if the link will be encrypted. If set
 *	to 1, no traffic is forwarded over the link until encryption has been enabled via \link
 *	qcsapi_wifi_wds_set_psk \endlink. If set to 0, traffic can flow over the link even if
 *	the link is not encrypted. This mode does not prevent the link from being encrypted.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_add_peer \<WiFi interface\> \<BSSID of peer AP\> [ encrypt ]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wds_add_peer_encrypt(const char *ifname, const qcsapi_mac_addr peer_address,
					const qcsapi_unsigned_int encryption);

/**
 * @brief Remove a WDS peer.
 *
 * Remove a WDS peer.
 *
 * \param ifname \wifi0only
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_remove_peer \<WiFi interface\> \<BSSID of peer AP\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wds_remove_peer(const char *ifname, const qcsapi_mac_addr peer_address);

/**
 * @brief Get a WDS peer by index
 *
 * Get a WDS peer by index.
 *
 * This API is typically used to construct a list of the configured WDS peers.
 *
 * \param ifname \wifi0only
 * \param index the index to get the WDS peer address of.
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_get_peer_address \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be the BSSID for the peer, as selected by its index.
 */
extern int qcsapi_wds_get_peer_address(const char *ifname, const int index,
					qcsapi_mac_addr peer_address);

/**
 * @brief Configure encryption for a WDS link
 *
 * Configure encryption for a WDS link.
 *
 * WDS encryption is similar to WPA-NONE encryption which is often used for ad-hoc connections.
 * There is no key exchange protocol. Frames are encrypted with AES using a 256-bit preshared key,
 * which must be set to the same value on both peers.
 *
 * \param ifname \wifi0only
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 * \param pre_shared_key a 256 bit key, encoded as hexadecimal ASCII characters (0-9, a-f), or an
 * empty string to delete the encryption key.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_set_psk \<WiFi interface\> \<BSSID of peer AP\> {\<WDS PSK\> | NULL}</c>
 *
 * WDS PSK is a string of exactly 64 hexadecimal characters, or "NULL" to delete the encryption key.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_wds_set_psk(const char *ifname, const qcsapi_mac_addr peer_address,
					const string_64 pre_shared_key);

/**
 * @brief Configure encryption for a WDS link
 *
 * Configure encryption for a WDS link.
 *
 * \note This API is identical to \link qcsapi_wifi_wds_set_psk \endlink, and is provided for
 * backwards compatibility.
 */
extern int qcsapi_wds_set_psk(const char *ifname, const qcsapi_mac_addr peer_address,
					const string_64 pre_shared_key);

/**
 * @brief Set the WDS mode for a WDS peer link
 *
 * Set the WDS mode for a WDS peer link. The device can play a WDS role of either MBS (Main Base
 * Station) or RBS (Remote Base Station).
 *
 * \param ifname \wifi0only
 * \param peer_address \mac_format - the MAC address of the primary WiFi interface on the peer AP.
 * \param mode non-zero value for rbs mode and zero for mbs mode.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_set_mode \<WiFi interface\> \<BSSID of peer AP\> {rbs | mbs}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wds_set_mode(const char *ifname, const qcsapi_mac_addr peer_address,
					const int mode);

/**
 * @brief Get WDS peer mode by index
 *
 * @brief Get WDS peer mode by index.
 *
 * \param ifname \wifi0only
 * \param index the node index of a WDS peer device
 * \param mode \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wds_get_mode \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be the mode for the WDS peer.
 */
extern int qcsapi_wds_get_mode(const char *ifname, const int index, int *mode);

/**
 * @brief Set Extender device parameter
 *
 * \param ifname \wifi0
 * \param type Extender parameter type
 * \param param_value Extender parameter value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_extender_params \<WiFi interface\> \<parameter type\> \<parameter value\></c>
 *
 * where <c>WiFi interface</c> is the primary interface, <c>parameter type</c> is one of
 * <c>role</c>, <c>mbs_best_rssi</c>, <c>rbs_best_rssi</c>, <c>mbs_wgt</c>, <c>rbs_wgt</c>,
 * <c>roaming</c>, <c>mbs_rssi_margin</c>, <c>bgscan_interval</c>, <c>verbose</c>, <c>fast_cac</c>.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_extender_params(const char *ifname, const qcsapi_extender_type type,
					const int param_value);

/**
 * @brief Get all Extender device parameters
 *
 * \param ifname \wifi0only
 * \param type Extender parameter type
 * \param p_value Extender parameter value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_extender_status \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the Extender related parameter value.
 */

extern int qcsapi_wifi_get_extender_params(const char *ifname, const qcsapi_extender_type type,
					int *p_value);

/** @} */

/**
 * @addtogroup SecurityAPIs
 * @{
 */

/**
 * @brief Get the security protocol from the beacon.
 *
 * Get the current beacon type. Only applicable for an AP; for a STA, use the SSID Get Protocol API
 * (qcsapi_SSID_get_protocol). On success, returned string will be one of those as documented in the
 * <b>authentication protocol</b> table in \ref CommonSecurityDefinitions.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_current_beacon the protocol as returned by the API.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_beacon \<WiFi interface\></c>
 *
 * Unless an error occurs, the response will be one of Basic, 11i, WPA or WPAand11i with the
 * interpretation of each listed in the table in the <b>authentication protocol</b> table in \ref
 * CommonSecurityDefinitions.
 *
 * \sa qcsapi_SSID_get_protocol
 */
extern int qcsapi_wifi_get_beacon_type(const char *ifname, char *p_current_beacon);

/**
 * @brief Set the security protocol in the beacon.
 *
 * Set the current beacon type.
 *
 * This API only applies for an AP; for a Station, use the SSID Set Protocol API
 * (qcsapi_SSID_set_protocol).
 *
 * The value for the new beacon must be one of the expected values listed in the section on the
 * corresponding get API. Value must match exactly including upper vs. lower case letters.
 *
 * \note \aponly
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \param ifname \wifi0
 * \param p_new_beacon the new security protocol to set in the beacon.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_beacon \<WiFi interface\> \<beacon type\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Beacon type needs to be one of the values as per the <b>authentication protocol</b> table in
 * \ref CommonSecurityDefinitions.
 *
 * \sa qcsapi_SSID_set_protocol
 */
extern int qcsapi_wifi_set_beacon_type(const char *ifname, const char *p_new_beacon);

/**
 * \brief Structure containing DPP parameter and value
 */
struct qcsapi_data_pair_string_dpp {
	string_32 key;
	string_256 value;
};

/**
 * \brief Enumeration for the parameters used by the API qcsapi_wifi_dpp_parameter.
 *
 * Enumeration for the parameters used by the API qcsapi_wifi_dpp_parameter.
 * Quoted strings are the corresponding parameter names to be used from call_qcsapi.
 */
enum qcsapi_dpp_exchange_cfg_param {
	/** DPP authentication initiator/respondor "init" */
	QCSAPI_PARAM_DPP_AUTH_INIT = 0,

	/** DPP provisioning role configurator/enrollee "role" */
	QCSAPI_PARAM_DPP_PROV_ROLE,

	/** DPP bootstrapping method "type" */
	QCSAPI_PARAM_DPP_BOOTSTRAP_METHOD,

	/** DPP bootstrapping curve "curve" */
	QCSAPI_PARAM_DPP_BOOTSTRAP_CURVE,

	/** DPP PKEX code "code" */
	QCSAPI_PARAM_DPP_PKEX_CODE,

	/** DPP PKEX identifier "identifier" */
	QCSAPI_PARAM_DPP_PKEX_ID,

	/** DPP listen frequency "freq" */
	QCSAPI_PARAM_DPP_LISTEN_FREQ,

	/** DPP peer mac address "mac" */
	QCSAPI_PARAM_DPP_PEER_MAC,

	/** DPP local bootstrap "local" */
	QCSAPI_PARAM_DPP_LOCAL_BOOTSTRAP,

	/** DPP peer bootstrap "peer" */
	QCSAPI_PARAM_DPP_PEER_BOOTSTRAP,

	/** DPP configurator "configurator" */
	QCSAPI_PARAM_DPP_CONFIGURATOR,

	/** DPP peer URI "uri" */
	QCSAPI_PARAM_DPP_URI,

	/** DPP BSS config-type "conf" */
	QCSAPI_PARAM_DPP_CONF,

	/** DPP BSS SSID "ssid" */
	QCSAPI_PARAM_DPP_SSID,

	/** DPP BSS passphrase "pass" */
	QCSAPI_PARAM_DPP_PASS,

	/** DPP BSS group "group" */
	QCSAPI_PARAM_DPP_GROUP,
};

/**
 * \brief Parameters that can be configured using the API qcsapi_wifi_dpp_parameter.
 */
static const struct {
	enum qcsapi_dpp_exchange_cfg_param param;
	char *name;
} qcsapi_dpp_exchange_cfg_param_table[] = {
	{QCSAPI_PARAM_DPP_AUTH_INIT,		"init"},
	{QCSAPI_PARAM_DPP_PROV_ROLE,		"role"},
	{QCSAPI_PARAM_DPP_BOOTSTRAP_METHOD,	"type"},
	{QCSAPI_PARAM_DPP_BOOTSTRAP_CURVE,	"curve"},
	{QCSAPI_PARAM_DPP_PKEX_CODE,		"code"},
	{QCSAPI_PARAM_DPP_PKEX_ID,		"identifier"},
	{QCSAPI_PARAM_DPP_LISTEN_FREQ,		"freq"},
	{QCSAPI_PARAM_DPP_PEER_MAC,		"mac"},
	{QCSAPI_PARAM_DPP_LOCAL_BOOTSTRAP,	"local"},
	{QCSAPI_PARAM_DPP_PEER_BOOTSTRAP,	"peer"},
	{QCSAPI_PARAM_DPP_CONFIGURATOR,		"configurator"},
	{QCSAPI_PARAM_DPP_URI,			"uri"},
	{QCSAPI_PARAM_DPP_CONF,			"conf"},
	{QCSAPI_PARAM_DPP_SSID,			"ssid"},
	{QCSAPI_PARAM_DPP_PASS,			"pass"},
	{QCSAPI_PARAM_DPP_GROUP,		"group"},
};

/**
 * \brief Enumeration for the BSS config parameters used by the API qcsapi_wifi_dpp_parameter.
 *
 * Enumeration for the BSS config parameters used by the API qcsapi_wifi_dpp_parameter.
 * Quoted strings are the corresponding parameter names to be used from call_qcsapi.
 */
enum qcsapi_dpp_exchange_bss_param {
	/** DPP BSS conf type "conf" */
	QCSAPI_PARAM_DPP_BSS_CONF,
	/** DPP BSS conf SSID "ssid" */
	QCSAPI_PARAM_DPP_BSS_SSID,
	/** DPP BSS conf passphrase "pass" */
	QCSAPI_PARAM_DPP_BSS_PASS,
	/** DPP BSS conf type "group" */
	QCSAPI_PARAM_DPP_BSS_GROUP,
};

/**
 * \brief DPP BSS profile parameters that can be configured using the API dpp_set_param.
 */
static const struct {
	enum qcsapi_dpp_exchange_bss_param param;
	char *name;
} qcsapi_dpp_exchange_bss_param_table[] = {
	{QCSAPI_PARAM_DPP_BSS_CONF,		"conf"},
	{QCSAPI_PARAM_DPP_BSS_SSID,		"ssid"},
	{QCSAPI_PARAM_DPP_BSS_PASS,		"pass"},
	{QCSAPI_PARAM_DPP_BSS_GROUP,		"group"},
};

#define QCSAPI_SET_PARAMS_ARG_MAX	8

/**
 * \brief Structure containing the parameters for the API dpp_set_param.
 */
struct qcsapi_dpp_set_parameters {
	struct qcsapi_data_pair_string_dpp param[QCSAPI_SET_PARAMS_ARG_MAX];
};

/**
 * @brief Set/Get DPP parameters.
 *
 * Set/get values for DPP exchange parameters
 *
 * \param ifname \wifi0
 * \param dpp_params pointer to the structure \link qcsapi_dpp_set_parameters \endlink.
 * \param databuf buffer to store returned data
 * \param max_len buffer length
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi dpp_param \<WiFi interface\> cfg_get \<param1\>
 *
 * <c>call_qcsapi dpp_param \<WiFi interface\> cfg_set \<param1\> \<value1\>
 *
 * <c>call_qcsapi dpp_param \<WiFi interface\> \<DPP command\> \<param1\> \<value1\>
 *	[\<param2> <value2\>]... [\<param8\> \<value8\>]
 *
 * DPP command names and corresponding enum values are shown in \ref qcsapi_dpp_param_table
 * Parameter names and corresponding enum values are shown in \ref qcsapi_dpp_exchange_cfg_param.
 *
 * Unless an error occurs, the output will be the response string.
 * For cfg_set or for dpp_commands which perform a 'set' operation the output will be "complete".
 * For cfg_get or for dpp_commands which perform a 'get' operation the output will be
 * the parameter value.
 */
extern int qcsapi_wifi_dpp_parameter(const char *ifname, int cmd,
					struct qcsapi_dpp_set_parameters *dpp_params,
					char *databuf, int max_len);

/* WEP */

/**
 * \brief Get the default WEP key index for the given interface.
 *
 * Get the default WEP key index for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_key_index \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_WEP_key_index \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current default WEP key index.
 *
 */
extern int qcsapi_wifi_get_WEP_key_index(const char *ifname, qcsapi_unsigned_int *p_key_index);

/**
 * \brief Set the default WEP key index for the given interface.
 *
 * Set the default WEP key index for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_index default WEP key index
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_WEP_key_index \<WiFi interface\> \<index\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_WEP_key_index(const char *ifname, const qcsapi_unsigned_int key_index);

/**
 * \brief API is not supported
 *
 * \return -EOPNOTSUPP.
 */
extern int qcsapi_wifi_get_WEP_key_passphrase(const char *ifname, string_64 current_passphrase);

/**
 * \brief API is not supported
 *
 * \return -EOPNOTSUPP.
 */
extern int qcsapi_wifi_set_WEP_key_passphrase(const char *ifname, const string_64 new_passphrase);

/**
 * \brief Retrieves current encryption level and supported encryption levels.
 *
 * qcsapi_wifi_get_WEP_encryption_level return current encryption level describing
 * current encryption state and available encrytion options
 * for example '<c>Disabled,40-bit,104-bit,128-bit</c>'
 *
 * \param ifname \wifi0
 * \param current_encryption_level String to store encryption level data
 *
 * \return \positive_or_negative
 * \return -EFAULT if current_encryption_level is NULL
 * \return -EMSGSIZE or negative number if underlying wireless_extensions API indicated an error
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_WEP_encryption_level \<WiFi interface\> \<current_encryption_level\></c>
 *
 * Unless an error occurs, the output will be a string describing the encryption level.
 * E.g. <c>Disabled,40-bit,104-bit,128-bit</c>
 */
extern int qcsapi_wifi_get_WEP_encryption_level(const char *ifname,
					string_64 current_encryption_level);

/**
 * \brief API is not supported
 *
 * \return -EOPNOTSUPP.
 *
 */
extern int qcsapi_wifi_get_basic_encryption_modes(const char *ifname, string_32 encryption_modes);

/**
 * \brief API is not supported
 *
 * \return -EOPNOTSUPP.
 */
extern int qcsapi_wifi_set_basic_encryption_modes(const char *ifname,
					const string_32 encryption_modes);

/**
 * \brief API is not supported
 *
 * \return -EOPNOTSUPP.
 */
extern int qcsapi_wifi_get_basic_authentication_mode(const char *ifname,
					string_32 authentication_mode);

/**
 * \brief This API is not supported.
 *
 * This API is not supported.
 *
 * \return -EOPNOTSUPP.
 */
extern int qcsapi_wifi_set_basic_authentication_mode(const char *ifname,
					const string_32 authentication_mode);

/**
 * \brief Get WEP key for the given interface.
 *
 * Get WEP key for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_index WEP key index
 * \param current_passphrase \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_WEP_key \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be the WEP key.
 *
 */
extern int qcsapi_wifi_get_WEP_key(const char *ifname, qcsapi_unsigned_int key_index,
					string_64 current_passphrase);

/**
 * \brief Set WEP key for the given interface.
 *
 * Set WEP key for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_index WEP key index
 * \param new_passphrase passphrase - must be 5 or 13 ASCII characters, or 10 or 26 hexadecimal
 *	digits.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_WEP_key \<WiFi interface\> \<index\> \<key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_WEP_key(const char *ifname, qcsapi_unsigned_int key_index,
					const string_64 new_passphrase);

/**
 * \brief Remove WEP configuration for the given interface.
 *
 * Remove WEP configuration for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_WEP_config \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_WEP_config(const char *ifname);

/* WPA */

/**
 * @brief Get the security <b>encryption</b> mode(s) configured.
 *
 * Get the current WPA/11i encryption protocol(s) in use.
 * Applies to AP only. For a STA, use qcsapi_SSID_get_encryption_modes.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param encryption_modes a string containing a value per the <b>encryption</b> definitions table
 * in the \ref CommonSecurityDefinitions section.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_WPA_encryption_modes \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be one of the strings listed in the <b>encryption</b>
 * definitions table in the \ref CommonSecurityDefinitions section.
 *
 * \sa qcsapi_SSID_get_encryption_modes
 */
extern int qcsapi_wifi_get_WPA_encryption_modes(const char *ifname, string_32 encryption_modes);

/**
 * @brief Set the security <b>encryption</b> mode(s).
 *
 * Set the current security encryption mode(s). Applies to AP only. For a STA, use
 * qcsapi_SSID_set_encryption_modes. Value is required to be one of the expected values from the
 * corresponding get operation. Value must match exactly including upper vs. lower case letters.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param encryption_modes a string containing a value per the <b>encryption</b> definitions table
 * in the \ref CommonSecurityDefinitions section.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_WPA_encryption_modes \<WiFi interface\> \<encryption mode(s)\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Encryptions mode(s) needs to be one of the values per the <b>encryption</b> definitions table in
 * the \ref CommonSecurityDefinitions section.
 *
 * \sa qcsapi_SSID_set_encryption_modes
 */
extern int qcsapi_wifi_set_WPA_encryption_modes(const char *ifname,
					const string_32 encryption_modes);

/**
 * @brief Get the security <b>authentication</b> mode configured.
 *
 * Get the current security authentication mode in use.
 * Applies to AP only. For a STA, use qcsapi_SSID_get_authentication_mode
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param authentication_mode a string containing a value per the <b>authentication types</b> table
 * in the \ref CommonSecurityDefinitions section.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_WPA_authentication_mode \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be one of the strings listed in the
 * <b>authentication type</b> table in the \ref CommonSecurityDefinitions section.
 *
 * \sa qcsapi_SSID_get_authentication_mode
 */
extern int qcsapi_wifi_get_WPA_authentication_mode(const char *ifname,
					string_32 authentication_mode);


/**
 * @brief Set the security <b>authentication</b> mode.
 *
 * Set the current security authentication mode. Applies to AP only. For a STA, use
 * \ref qcsapi_SSID_set_authentication_modes. Value is required to be one of the expected values
 * from the corresponding get operation. Value must match exactly including upper vs. lower case
 * letters.
 *
 * \note \aponly
 *
 * Steps to enable EAP Authentication:
 * Set the EAP Authentication mode and the EAP Server Parameters.
 *
 * Command to set EAPAuthentication:
 *	\li call_qcsapi set_WPA_authentication_modes \<device\> EAP Authentication
 *
 * Command to set Encryption:
 *	\li call_qcsapi set_WPA_encryption_modes $device \<encryption\>
 *
 * Command to configure RADIUS authentication servers:
 *	\li call_qcsapi add_radius_auth_server_cfg \<device\> \<ipaddr\> \<port\> \<sharedkey\>
 *	\li call_qcsapi del_radius_auth_server_cfg \<device\> \<ipaddr\> \<port\>
 *	\li call_qcsapi get_radius_auth_server_cfg \<device\>
 *
 * \param ifname \wifi0
 * \param authentication_mode a string containing a value per the <b>authentication type</b> table
 * in the \ref CommonSecurityDefinitions section.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_WPA_authentication_modes \<WiFi interface\> \<authentication mode(s)\></c>
 *
 * \call_qcsapi_string_complete
 *
 * The authentication mode needs to be one of the values per the <b>authentication type</b>
 * definitions table in the \ref CommonSecurityDefinitions section.
 */
extern int qcsapi_wifi_set_WPA_authentication_mode(const char *ifname,
					const string_32 authentication_mode);

#define QCSAPI_DPP_MAX_CMD_LEN		400
#define QCSAPI_DPP_MAX_BUF_SIZE		512

/**
 * \brief QCSAPI DPP commands.
 *
 */
enum qcsapi_dpp_config_cmd {
	/**
	 * Generate DPP bootstrap
	 */
	QCSAPI_DPP_BOOTSTRAP_GEN,
	/**
	 * Get DPP URI for a given peer id
	 */
	QCSAPI_DPP_GET_URI,
	/**
	 * Configure remain on channel for DPP listen
	 */
	QCSAPI_DPP_CONFIG_REMAIN_CHAN,
	/**
	 * Trigger DPP authentication
	 */
	QCSAPI_DPP_AUTH_INIT,
	/**
	 * Self configure a DPP device
	 */
	QCSAPI_DPP_SELF_CONFIGURE,
	/**
	 * Add DPP PKEX bootstrapping parameters
	 */
	QCSAPI_DPP_PKEX_ADD,
	/**
	 * Add a DPP configurator
	 */
	QCSAPI_DPP_ADD_CONFIGURATOR,
	/**
	 * Set QR code a DPP peer
	 */
	QCSAPI_DPP_SET_QRCODE,
	/**
	 * Set configurator parameters
	 */
	QCSAPI_DPP_SET_CONFIGURATOR_PARAMS,
	/**
	 * Set/Get DPP device configuration
	 */
	QCSAPI_DPP_DEV_CONFIGURATION
};

/**
 * \brief DPP bootstrapping methods.
 *
 */
enum qcsapi_dpp_bootstrap_method {
	/**
	 * DPP bootstrapping method QRCODE
	 */
	QCSAPI_DPP_BOOTSTRAP_QRCODE,
	/**
	 * DPP bootstrapping method public key exchange PKEX
	 */
	QCSAPI_DPP_BOOTSTRAP_PKEX,
	/**
	 * DPP invalid bootstrapping method
	 */
	QSAPI_DPP_BOOTSTRAP_INVALID,
};

/**
 * \brief DPP device roles.
 *
 */
enum qcsapi_dpp_role {
	/**
	 * DPP device role Enrollee
	 */
	QCSAPI_DPP_ROLE_ENROLLEE,
	/**
	 * DPP device role Configurator
	 */
	QCSAPI_DPP_ROLE_CONFIGURATOR,
	/**
	 * DPP device role both (Enrollee/Configurator)
	 */
	QCSAPI_DPP_ROLE_BOTH,
	/**
	 * Invalid DPP device role
	 */
	QCSAPI_DPP_ROLE_INVALID,
};

/**
 * \brief DPP Crypto curves
 *
 */
enum qcsapi_dpp_crypto_curve {
	/**
	 * DPP ECC curve P-256
	 */
	QCSAPI_DPP_CRYPTO_P256,
	/**
	 * DPP ECC curve BP-256
	 */
	QCSAPI_DPP_CRYPTO_BP256,
	/**
	 * DPP ECC curve BP-384
	 */
	QCSAPI_DPP_CRYPTO_BP384,
	/**
	 * DPP ECC curve BP-512
	 */
	QCSAPI_DPP_CRYPTO_BP512,
};

/**
 * \brief Legacy Quantenna BBIC compatibility options.
 *
 * Legacy Quantenna BBIC compatibility options.
 *
 */
enum qcsapi_legacy_bbic_opt {
	/**
	 * 0 - disable compatibility options (the default)
	 */
	QCSAPI_LEGACY_BBIC_OPT_DISABLED,
	/**
	 * 1 - disable A-MSDU to BBIC3 peer devices
	 */
	QCSAPI_LEGACY_BBIC_OPT_AMSDU_ENABLED,
	/**
	 * 2 - ignore CMIC errors from BBIC3 peer devices
	 */
	QCSAPI_LEGACY_BBIC_OPT_CMIC_ENABLED,
	/**
	 * 3 - enable both above options
	 */
	QCSAPI_LEGACY_BBIC_OPT_BOTH_ENABLED,
};

/**
 * \brief DPP BSS (Basic Service Set) configuration structure.
 *
 */
struct qcsapi_dpp_bss_config {
	/**
	 * configuration role
	 */
	string_16 conf_role;
	/**
	 * group-id of for DPP BSS configuration
	 */
	string_32 group_id;
	/**
	 * SSID
	 */
	string_64 ssid;
	/**
	 * PSK/Passphrase
	 */
	string_128 psk;
};

/**
 * \brief DPP configuration parameters.
 *
 */
struct qcsapi_dpp_cfg {
	/**
	 * ECC curve ID being used for bootstrap public key generation
	 */
	uint16_t curve;
	/**
	 * Bootstrapping method
	 */
	uint16_t method;
	/**
	 * DPP peer ID
	 */
	uint16_t peer_id;
	/**
	 * DPP remain on channel configuration enable listen
	 */
	uint16_t listen_enable;
	/**
	 * DPP remain on channel configuration listen frequency
	 */
	uint16_t listen_freq;
	/**
	 * DPP remain on channel configuration listen duration
	 */
	uint16_t listen_dur;
	/**
	 * DPP configuration provisioning role
	 */
	uint16_t prov_role;
	/**
	 * DPP authentication role(initiator/responder)
	 */
	uint16_t auth_init;
	/**
	 * DPP configurator ID
	 */
	uint16_t configurator_id;
	/**
	 * DPP peer bootstrap ID
	 */
	uint16_t peer_bootstrap;
	/**
	 * DPP local bootstrap ID
	 */
	uint16_t local_bootstrap;
	/**
	 * DPP PKEX identifier
	 */
	string_64 pkex_id;
	/**
	 * DPP PKEX secret code
	 */
	string_64 pkex_code;
	/**
	 * DPP PKEX QR code string
	 */
	string_64 qrcode_str;
	/**
	 * DPP peer QR code URI
	 */
	string_512 peer_uri;
	/**
	 * MAC address of device being bootstrapped
	 */
	qcsapi_mac_addr mac_addr;
	/**
	 * Operating class, with channel and region to decide which frequency to execute
	 */
	uint8_t op_class;
	/**
	 * IEEE channel number, with operating class and region to decide which frequency to execute
	 */
	uint8_t channel;
};

/**
 * \brief Enumeration to represent DPP config parameters as used by qcsapi_wifi_dpp_parameter API.
 *
 * \sa qcsapi_wifi_dpp_parameter
 */
enum qcsapi_dpp_cfg_param_type {
	/**
	 * Invalid parameter
	 */
	qcsapi_dpp_cfg_nosuch_parameter = 0,
	/**
	 * DPP Authentication role: 0-Responder, 1-Initiator.
	 */
	qcsapi_dpp_cfg_auth_role,
	/**
	 * DPP Provisioning role: 0-Enrollee, 1-Configurator.
	 */
	qcsapi_dpp_cfg_prov_role,
	/**
	 * DPP bootstrapping method: 0-QRCODE, 1-PKEX.
	 */
	qcsapi_dpp_cfg_bootstrap_method,
	/**
	 * DPP Bootstrapping ECC Curve: 0-P-256.
	 */
	qcsapi_dpp_cfg_signing_curve_bootstrap,
	/**
	 * DPP C-Sign key ECC Curve: 0-P-256.
	 */
	qcsapi_dpp_cfg_signing_curve_csign,
	/**
	 * DPP Netrole: 0-station, 1-AP.
	 */
	qcsapi_dpp_cfg_netrole,
	/**
	 * DPP profile policy: 0-do not use, 1-do not update, 2-update.
	 */
	qcsapi_dpp_cfg_seccfg_processing,
	/**
	 * DPP Authentication direction: 0-Single, 1-Mutual.
	 */
	qcsapi_dpp_cfg_auth_direction,
	/**
	 * DPP configurator params string.
	 */
	qcsapi_dpp_cfg_configurator_params,
};

/**
 * \brief Enumeration to represent DPP commands as used by the qcsapi_wifi_dpp_parameter API.
 *
 * \sa qcsapi_wifi_dpp_parameter
 */
enum qcsapi_dpp_cmd_param_type {
	/**
	 * DPP command: get device configuration param
	 */
	qcsapi_dpp_cmd_get_config,
	/**
	 * DPP command: set device configuration param
	 */
	qcsapi_dpp_cmd_set_config,
	/**
	 * DPP command: Set QR code from peer device
	 */
	qcsapi_dpp_cmd_qr_code,
	/**
	 * DPP command: Generate bootstrap
	 */
	qcsapi_dpp_cmd_bootstrap_gen,
	/**
	 * DPP command: Get bootstrap URI
	 */
	qcsapi_dpp_cmd_bootstrap_get_uri,
	/**
	 * DPP command: Initiate DPP authentication
	 */
	qcsapi_dpp_cmd_auth_init,
	/**
	 * DPP command: Start listen channel operation
	 */
	qcsapi_dpp_cmd_listen,
	/**
	 * DPP command: Cancel ongoing listen channel operation
	 */
	qcsapi_dpp_cmd_stop_listen,
	/**
	 * DPP command: Add DPP configurator
	 */
	qcsapi_dpp_cmd_configurator_add,
	/**
	 * DPP command: Remove DPP configurator
	 */
	qcsapi_dpp_cmd_configurator_remove,
	/**
	 * DPP command: Remove DPP configurator
	 */
	qcsapi_dpp_cmd_configurator_sign,
	/**
	 * DPP command: Self-sign configurator
	 */
	qcsapi_dpp_cmd_pkex_add,
	/**
	 * DPP command: Add PKEX code
	 */
	qcsapi_dpp_cmd_pkex_remove,
	/**
	 * DPP command: Remove PKEX code
	 */
	qcsapi_dpp_cmd_configurator_params,
	/**
	 * DPP command: Set configurator parameters
	 */
	qcsapi_dpp_cmd_nosuch_parameter,
};


/**
 * \brief Commands that can be configured using the API qcsapi_wifi_dpp_parameter.
 *
 */
static const struct {
	enum qcsapi_dpp_cmd_param_type param_type;
	const char *param_name;
} qcsapi_dpp_param_table[] = {
	{qcsapi_dpp_cmd_get_config, "cfg_get"},
	{qcsapi_dpp_cmd_set_config, "cfg_set"},
	{qcsapi_dpp_cmd_qr_code, "dpp_qr_code"},
	{qcsapi_dpp_cmd_bootstrap_gen, "dpp_bootstrap_gen"},
	{qcsapi_dpp_cmd_bootstrap_get_uri, "dpp_bootstrap_get_uri"},
	{qcsapi_dpp_cmd_auth_init, "dpp_auth_init"},
	{qcsapi_dpp_cmd_listen, "dpp_listen"},
	{qcsapi_dpp_cmd_stop_listen, "dpp_stop_listen"},
	{qcsapi_dpp_cmd_configurator_add, "dpp_configurator_add"},
	{qcsapi_dpp_cmd_configurator_remove, "dpp_configurator_remove"},
	{qcsapi_dpp_cmd_configurator_sign, "dpp_configurator_sign"},
	{qcsapi_dpp_cmd_pkex_add, "dpp_pkex_add"},
	{qcsapi_dpp_cmd_pkex_remove, "dpp_pkex_remove"},
	{qcsapi_dpp_cmd_configurator_params, "dpp_configurator_params"},
	{qcsapi_dpp_cmd_nosuch_parameter,	NULL},
};

/**
 * \brief Configure WPA3 DPP (Device Provisioning Protocol).
 *
 *  Configure WPA3 DPP (Device Provisioning Protocol)
 *
 * \param ifname \wifi0
 * \param dpp_cmd DPP command ID
 * \param dpp_cfg Command-specific DPP config parameters
 * \param bss_cfg BSS configuration when applicable; NULL otherwise
 * \param wpa_resp Response string
 * \param max_len Length of response buffer
 * \return \zero_or_negative
 * \callqcsapi
 *
 *  A call_qcsapi is not provided for this API
 */
extern int qcsapi_dpp_configure_param(const char *ifname, int dpp_cmd,
					const struct qcsapi_dpp_cfg *dpp_cfg,
					const struct qcsapi_dpp_bss_config *bss_cfg,
					char *wpa_resp, int max_len);

/**
 * Enumeration for the parameters used by the API \ref qcsapi_set_params.
 * Quoted strings are the corresponding parameter names to be used from call_qcsapi.
 */
enum qcsapi_param {
	/** SAE groups "sae_groups" (see \ref QCSAPI_ECC_GROUPS_SAE_LIST) */
	QCSAPI_PARAM_SAE_GROUPS = 0,

	/**
	 * SAE synchronization "sae_sync"
	 * (Maximum number of SAE synchronization errors)
	 */
	QCSAPI_PARAM_SAE_SYNC,

	/** SAE requires MFP "sae_require_mfp" */
	QCSAPI_PARAM_SAE_REQUIRE_MFP,

	/** SAE anti-clogging threshold "sae_anti_clogging_threshold" */
	QCSAPI_PARAM_SAE_ANTI_CLOGGING_THRESHOLD,

	/** SAE max auth attempts "sae_max_auth_attempts" */
	QCSAPI_PARAM_SAE_MAX_AUTH_ATTEMPTS,

	/** SAE auth lockout period "sae_auth_lockout_period" */
	QCSAPI_PARAM_SAE_AUTH_LOCKOUT_PERIOD,

	/** SAE auth monitor period "sae_auth_monitor_period" */
	QCSAPI_PARAM_SAE_AUTH_MONITOR_PERIOD,

	/** OWE group "owe_group" (see \ref QCSAPI_ECC_GROUPS_OWE_LIST) */
	QCSAPI_PARAM_OWE_GROUP,

	/** OWE groups "owe_groups" (see \ref QCSAPI_ECC_GROUPS_OWE_LIST) */
	QCSAPI_PARAM_OWE_GROUPS,

	/**
	 * OWE transition BSSID "owe_transition_bssid"
	 * (Pointer to the matching open/OWE BSS for OWE transition mode)
	 */
	QCSAPI_PARAM_OWE_TRANSITION_BSSID,

	/** OWE transition SSID "owe_transition_ssid" (SSID2 format \ref SSID2_RULES) */
	QCSAPI_PARAM_OWE_TRANSITION_SSID,

	/**
	 * OWE transition interface name "owe_transition_ifname"
	 * (Reference to a matching open/OWE BSS for OWE transition mode)
	 */
	QCSAPI_PARAM_OWE_TRANSITION_IFNAME,

	/** Security authentication algorithm "auth_alg" */
	QCSAPI_PARAM_SECURITY_AUTH_ALG,
};

/**
 * \brief Parameters that can be configured using the API set_params.
 */
static const struct {
	enum qcsapi_param	param;
	char			*name;
} qcsapi_param_table[] = {
	/* SAE */
	{QCSAPI_PARAM_SAE_GROUPS,		"sae_groups"},
	{QCSAPI_PARAM_SAE_SYNC,			"sae_sync"},
	{QCSAPI_PARAM_SAE_REQUIRE_MFP,		"sae_require_mfp"},
	{QCSAPI_PARAM_SAE_ANTI_CLOGGING_THRESHOLD,	"sae_anti_clogging_threshold"},
	{QCSAPI_PARAM_SAE_MAX_AUTH_ATTEMPTS,	"sae_max_auth_attempts"},
	{QCSAPI_PARAM_SAE_AUTH_LOCKOUT_PERIOD,	"sae_auth_lockout_period"},
	{QCSAPI_PARAM_SAE_AUTH_MONITOR_PERIOD,	"sae_auth_monitor_period"},
	/* OWE */
	{QCSAPI_PARAM_OWE_GROUP,		"owe_group"},
	{QCSAPI_PARAM_OWE_GROUPS,		"owe_groups"},
	{QCSAPI_PARAM_OWE_TRANSITION_BSSID,	"owe_transition_bssid"},
	{QCSAPI_PARAM_OWE_TRANSITION_SSID,	"owe_transition_ssid"},
	{QCSAPI_PARAM_OWE_TRANSITION_IFNAME,	"owe_transition_ifname"},
	/* Other Security Params */
	{QCSAPI_PARAM_SECURITY_AUTH_ALG,	"auth_alg"},
};

/**
 * \brief Structure containing parameter and value
 */
struct qcsapi_data_pair_string_128bytes {
	string_64 key;
	string_64 value;
};

/**
 * \brief Structure containing the parameters for the API set_params.
 */
struct qcsapi_set_parameters {
	struct qcsapi_data_pair_string_128bytes param[QCSAPI_SET_PARAMS_ARG_MAX];
};

/**
 * @brief Set parameters.
 *
 * Set values for multiple parameters
 *
 * \param ifname \wifi0
 * \param SSID_substr the ssid of network block in station mode or NULL in AP mode
 * \param set_params pointer to the structure \link qcsapi_set_parameters \endlink.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_params \<WiFi interface\> \<param1\> \<value1\>
 *	[\<param2> <value2\>]... [\<param8\> \<value8\>]
 *
 * <c>call_qcsapi SSID_set_params \<WiFi interface\> \<ssid\> \<param1\>
 *	\<value1\> [\<param2> <value2\>]... [\<param8\> \<value8\>]
 *
 * Parameter names and corresponding enum values are shown in \ref qcsapi_param.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_set_params(const char *ifname,
		const qcsapi_SSID SSID_substr,
		const struct qcsapi_set_parameters *set_params);

/**
 * @brief Get parameters.
 *
 * Get values for multiple parameters
 *
 * \param ifname \wifi0
 * \param SSID_substr the ssid of network block in station mode or NULL in AP mode
 * \param get_params pointer to the structure \link qcsapi_set_parameters \endlink.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_params \<WiFi interface\> \<param1\> [\<param2\>... \<param8\>]
 *
 * <c>call_qcsapi SSID_get_params \<WiFi interface\> \<ssid\> [\<param1\> \<param2\>... \<param8\>]
 *
 * Parameter names and corresponding enum values are shown in \ref qcsapi_param.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_get_params(const char *ifname, const qcsapi_SSID SSID_substr,
			struct qcsapi_set_parameters *get_params);

/**
 * \brief Get the 802.11u Interworking status
 *
 * Get the 802.11u Interworking status.
 *
 * \param ifname \wifi0
 * \param interworking a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interworking \<WiFi interface\></c>
 *
 * The output will be the interworking status unless an error occurs.
 */
extern int qcsapi_wifi_get_interworking(const char *ifname, string_32 interworking);

/**
 * \brief Set the 802.11u Interworking status
 *
 * Set the 802.11u Interworking status.
 *
 * \param ifname \wifi0
 * \param interworking 0(Disable), 1(Enable)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_interworking \<WiFi interface\> \<interworking\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_interworking(const char *ifname, const string_32 interworking);

/**
 * \brief Get an 802.11u parameter
 *
 * Get the value for the specified 802.11u parameter, as specified by
 * the qcsapi_80211u_params parameter, with the value returned
 * in the address specified by p_buffer.
 *
 * \param ifname \wifi0
 * \param u_param is 802.11u parameter to get the value of
 * \param p_buffer return parameter to contain the value of the 802.11u parameter
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_80211u_params \<WiFi interface\> \<u_param\></c>
 *
 * The output will be the 802.11u parameter unless an error occurs.
 */
extern int qcsapi_wifi_get_80211u_params(const char *ifname,
						const string_32 u_param,
						string_256 p_buffer);

/**
 * \brief Set an 802.11u parameter
 *
 * Set the value for a specified 802.11u parameter.
 *
 * \param ifname \wifi0
 * \param param is the 802.11u parameter to set
 * \param value1 is the first value for the parameter
 * \param value2 is the second value for the parameter, or NULL if the parameter has only one value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * Valid parameters and their corresponding values shown in the following table.
 * The exact format of the values for each parameter are described in the
 * hostapd.conf man page.
 *
 * <TABLE>
 * <TR> <TD>Parameter</TD>		<TD>value1</TD>			<TD>value2</TD></TR>
 * <TR> <TD>internet</TD>		<TD>0 or 1</TD>			<TD>-</TD></TR>
 * <TR> <TD>access_network_type</TD>	<TD>type</TD>			<TD>-</TD></TR>
 * <TR> <TD>network_auth_type</TD>	<TD>indicator value</TD>	<TD>-</TD></TR>
 * <TR> <TD>hessid</TD>			<TD>MAC address</TD>		<TD>-</TD></TR>
 * <TR> <TD>ipaddr_type_availability</TD><TD>IPv4 type</TD>		<TD>IPv6 type</TD></TR>
 * <TR> <TD>domain_name</TD>		<TD>domain name</TD>		<TD>-</TD></TR>
 * <TR> <TD>anqp_3gpp_cell_net</TD>	<TD>MCC1,MNC1;MCC2,MNC2;...</TD><TD>-</TD></TR>
 * <TR> <TD>venue_group</TD>		<TD>venue group</TD>		<TD>-</TD></TR>
 * <TR> <TD>venue_type</TD>		<TD>venue type</TD>		<TD>-</TD></TR>
 * <TR> <TD>gas_comeback_delay</TD>	<TD>GAS comeback delay in TUs</TD>	<TD>-</TD></TR>
 * <TR> <TD>gas_frag_limit</TD>		<TD>GAS fragment limit in bytes</TD>	<TD>-</TD></TR>
 * </TABLE>
 *
 * \note Max anqp_3gpp_cell_net count is IEEE80211U_3GPP_CELL_NET_MAX
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_80211u_params \<WiFi interface\> \<param\> \<value1\> \<value2\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_80211u_params(const char *ifname, const string_32 param,
				const string_256 value1, const string_32 value2);

/**
 * \brief Get 802.11 NAI Realms
 *
 * Get a list of the configured 802.11u NAI Realms.
 *
 * \param ifname \wifi0
 *
 * \param p_value is pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note ifname must be the primary interface.
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_nai_realms \<WiFi interface\></c>
 *
 * The output will be a list of NAI Realms unless an error occurs.
 */
extern int qcsapi_security_get_nai_realms(const char *ifname, string_4096 p_value);

/**
 * \brief Add or update an 802.11 NAI Realm
 *
 * Add or update an 802.11u NAI Realm.
 *
 * \param ifname \wifi0
 * \param encoding accepts value 0 or 1
 * \param nai_realm
 * \param eap_method
 *
 * \note If the NAI Realm already exists, it will be updated.
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_nai_realm \<WiFi interface\> \<encoding\> \<NAI realm\> \<EAP methods\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_nai_realm(const char *ifname,
						const int encoding,
						const char *nai_realm,
						const char *eap_method);

/**
 * \brief Delete an 802.11u NAI Realm
 *
 * Delete an existing 802.11u NAI Realm.
 *
 * \param ifname \wifi0
 * \param p_value nai_realm to be deleted
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_nai_realm \<WiFi interface\> \<Nai realm\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_nai_realm(const char *ifname, const char *nai_realm);

/**
 * \brief Get 802.11u Roaming Consortia
 *
 * Get the list of configured 802.11 Roaming Consortia.
 *
 * \param ifname \wifi0
 * \param p_value is pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_roaming_consortium \<WiFi interface\></c>
 *
 * The output will be the roaming consortium value unless an error occurs.
 */
extern int qcsapi_security_get_roaming_consortium(const char *ifname, string_1024 p_value);

/**
 * \brief Add the 802.11u roaming_consortium
 *
 * Add an 802.11u Roaming Consortium.
 *
 * \param ifname \wifi0
 * \param p_value the Roaming Consortium OI, which is a 3 to 15 octet hexadecimal string
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_roaming_consortium \<WiFi interface\> \<OI\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_roaming_consortium(const char *ifname, const char *p_value);

/**
 * \brief Delete a 802.11u Roaming Consortium.
 *
 * Delete an existing 802.11 Roaming Consortium.
 *
 * \param ifname \wifi0
 * \param p_value roaming_consortium to be deleted
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_roaming_consortium \<WiFi interface\> \<OI\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_roaming_consortium(const char *ifname, const char *p_value);

/**
 * \brief Get 802.11u Venue names
 *
 * Get the list of configured 802.11 Venue names.
 *
 * \param ifname \wifi0
 * \param p_value is pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_venue_name \<WiFi interface\></c>
 *
 * The output will be the list of venue names unless an error occurs.
 */
extern int qcsapi_security_get_venue_name(const char *ifname, string_4096 p_value);

/**
 * \brief Add the 802.11u venue name
 *
 * Add an 802.11u venue name.
 *
 * \param ifname \wifi0
 * \param lang_code 2 or 3 character ISO-639 language code. E.g. "eng" for English
 * \param name venue name (Max IEEE80211U_VENUE_NAME_LEN_MAX characters)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_venue_name \<WiFi interface\> \<lang_code\> \<name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_venue_name(const char *ifname, const char *lang_code,
					const char *venue_name);

/**
 * \brief Delete the 802.11u venue name
 *
 * Delete an 802.11u venue name.
 *
 * \param ifname \wifi0
 * \param lang_code 2 or 3 character ISO-639 language code. E.g. "eng" for English
 * \param name venue name (Max IEEE80211U_VENUE_NAME_LEN_MAX characters)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_venue_name \<WiFi interface\> \<lang_code\> \<name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_venue_name(const char *ifname, const char *lang_code,
					const char *venue_name);

/**
 * \brief Get Hotspot 2.0 opererator friendly names
 *
 * Get the list of configured Hotspot 2.0 opererator friendly names.
 *
 * \param ifname \wifi0
 * \param p_value is pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_oper_friendly_name \<WiFi interface\></c>
 *
 * The output will be the list of Hotspot 2.0 opererator friendly names unless an error occurs.
 */
extern int qcsapi_security_get_oper_friendly_name(const char *ifname, string_4096 p_value);

/**
 * \brief Add Hotspot 2.0 opererator friendly name
 *
 * Add an Hotspot 2.0 opererator friendly name.
 *
 * \param ifname \wifi0
 * \param lang_code 2 or 3 character ISO-639 language code. E.g. "eng" for English
 * \param name Hotspot 2.0 opererator friendly name (Max HS20_OPER_FRIENDLY_NAME_LEN_MAX characters)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_oper_friendly_name \<WiFi interface\> \<lang_code\> \<name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_oper_friendly_name(const char *ifname, const char *lang_code,
							const char *oper_friendly_name);

/**
 * \brief Delete Hotspot 2.0 opererator friendly name
 *
 * Delete an Hotspot 2.0 opererator friendly name.
 *
 * \param ifname \wifi0
 * \param lang_code 2 or 3 character ISO-639 language code. E.g. "eng" for English
 * \param name Hotspot 2.0 opererator friendly name (Max HS20_OPER_FRIENDLY_NAME_LEN_MAX characters)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_oper_friendly_name \<WiFi interface\> \<lang_code\> \<name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_oper_friendly_name(const char *ifname, const char *lang_code,
							const char *oper_friendly_name);

/**
 * \brief Get Hotspot 2.0 connection capability
 *
 * Get the list of configured Hotspot 2.0 connection capability.
 *
 * \param ifname \wifi0
 * \param p_value is pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hs20_conn_capab \<WiFi interface\></c>
 *
 * The output will be the list of Hotspot 2.0 connection capability unless an error occurs.
 */
extern int qcsapi_security_get_hs20_conn_capab(const char *ifname, string_4096 p_value);

/**
 * \brief Add Hotspot 2.0 connection capability
 *
 * Add an Hotspot 2.0 connection capability.
 *
 * \param ifname \wifi0
 * \param ip_proto is IP Protocol from 0 to IPPROTO_MAX
 * \param port_num is Port Number from 0 to USHRT_MAX
 * \param status is status of selected IP Protocol and Port Number.
 *	0 (Closed), 1 (Open), or 2 (Unknown)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_hs20_conn_capab \<WiFi interface\> \<ip_proto\> \<port_num\> \<status\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_hs20_conn_capab(const char *ifname, const char *ip_proto,
				const char *port_num, const char *status);

/**
 * \brief Delete Hotspot 2.0 connection capability
 *
 * Delete an Hotspot 2.0 connection capability.
 *
 * \param ifname \wifi0
 * \param ip_proto is IP Protocol from 0 to IPPROTO_MAX
 * \param port_num is Port Number from 0 to USHRT_MAX
 * \param status is status of selected IP Protocol and Port Number.
 *	0 (Closed), 1 (Open), or 2 (Unknown)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_hs20_conn_capab \<WiFi interface\> \<ip_proto\> \<port_num\> \<status\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_hs20_conn_capab(const char *ifname, const char *ip_proto,
					const char *port_num, const char *status);
/**
 * \brief Get the Hotspot 2.0 parameter status
 *
 * Get the Hotspot 2.0 parameter status.
 *
 * \param ifname \wifi0
 * \param p_hs20 a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hs20_status \<WiFi interface\></c>
 *
 * The output will be the hs status unless an error occurs.
 */
extern int qcsapi_wifi_get_hs20_status(const char *ifname, string_32 p_hs20);

/**
 * \brief Enable or Disable Hotspot 2.0
 *
 * Enable or Disable Hotspot 2.0.
 *
 * \param ifname \wifi0
 * \param hs20_val either 0(Disable) or 1(Enable)
 *
 * \return \zero_or_negative
 *
 * \note If Hotspot 2.0 is enabled then WPS will be disabled.
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_hs20_status \<WiFi interface\> \<hs\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_hs20_status(const char *ifname, const string_32 hs20_val);

/**
 * \brief Get the Proxy ARP parameter status
 *
 * Get the current Proxy ARP status.
 *
 * \param ifname \wifi0
 * \param p_proxy_arp a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_proxy_arp \<WiFi interface\></c>
 *
 * The output will be the Proxy ARP status unless an error occurs.
 */
extern int qcsapi_wifi_get_proxy_arp(const char *ifname, string_32 p_proxy_arp);

/**
 * \brief Set the Proxy ARP parameter
 *
 * Set a Proxy ARP parameter.
 *
 * \param ifname \wifi0
 * \param proxy_arp_val 0-Disable, 1-Enable
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_proxy_arp \<WiFi interface\> \<proxy_arp\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_proxy_arp(const char *ifname, const string_32 proxy_arp_val);

/**
 * \brief Get the L2 external filter parameters
 *
 * Get the current L2 external filter parameters. Refer to
 * \link qcsapi_wifi_set_l2_ext_filter \endlink for a list of valid parameter names.
 *
 * \param ifname \wifi0
 * \param param parameter to get value of
 * \param value a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_l2_ext_filter \<WiFi interface\> \<param\></c>
 *
 * The output will be the L2 external filter status unless an error occurs.
 */
extern int qcsapi_wifi_get_l2_ext_filter(const char *ifname, const string_32 param,
						string_32 value);

/**
 * \brief Set the L2 external filter parameters
 *
 * Set the L2 external filter parameters.
 *
 * \param ifname \wifi0
 * \param param parameter to set value of
 * \param value value to be set for the parameter
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * Valid parameters and their corresponding values are shown in the following table.
 *
 * <TABLE>
 * <TR> <TD>Parameter</TD>	<TD>Value</TD>
 * <TR> <TD>status</TD>		<TD>0 (Disable) or 1 (Enable)</TD>
 * <TR> <TD>port</TD>		<TD>port name, one of emac0, emac1 or pcie</TD>
 * <TR> <TD>interface</TD>	<TD>network interface name</TD>
 * </TABLE>
 *
 * \note The \<port\> and \<interface\> parameters have the same effect, but \<port\> only accepts
 * internal interface names and is deprecated.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_l2_ext_filter \<WiFi interface\> \<param\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_l2_ext_filter(const char *ifname, const string_32 param,
						const string_32 value);

/**
 * \brief Get a Hotspot 2.0 parameter value
 *
 * Get the value for the specified Hotspot 2.0 parameter. Refer to
 * \link qcsapi_wifi_set_hs20_params \endlink for a list of valid parameter names.
 *
 * \param ifname \wifi0
 * \param hs_param Hotspot 2.0 parameter to get the value of
 * \param p_buffer buffer pointer to contain the value of the requested parameter
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hs20_params \<WiFi interface\> \<hs_param\></c>
 *
 * The output will be the hs parameter unless an error occurs.
 */
extern int qcsapi_wifi_get_hs20_params(const char *ifname,
			const string_32 hs_param, string_32 p_buffer);

/**
 * \brief Set a Hotspot 2.0 parameter value
 *
 * Set a value for the specified Hotspot 2.0 parameter.
 *
 * \param ifname \wifi0
 * \param hs_param is hs parameter to set
 * \param value1-value6 values to be set for the parameter (value2 - value6 may be NULL)
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * Valid parameters and their corresponding values are shown in the following table.
 * The exact format of the values for each parameter are described in the
 * hostapd.conf man page.
 *
 * <small>
 * <TABLE>
 * <TR>
 *	<TD>Parameter</TD>
 *	<TD>value1</TD>
 *	<TD>value2</TD>
 *	<TD>value3</TD>
 *	<TD>value4</TD>
 *	<TD>value5</TD>
 *	<TD>value6</TD>
 * </TR>
 * <TR>
 *	<TD>hs20_wan_metrics</TD>
 *	<TD>WAN Info</TD>
 *	<TD>Downlink Speed</TD>
 *	<TD>Uplink Speed</TD>
 *	<TD>Downlink Load</TD>
 *	<TD>Uplink Load</TD>
 *	<TD>Load Measurement</TD>
 * </TR>
 * <TR>
 *	<TD>disable_dgaf</TD>
 *	<TD>0 or 1</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 * </TR>
 * <TR>
 *	<TD>hs20_operating_class</TD>
 *	<TD>Single Band 2.4 GHz</TD>
 *	<TD>Single Band 5 GHz</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 * </TR>
 * <TR>
 *	<TD>osu_ssid</TD>
 *	<TD>SSID used for all OSU connections</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 * </TR>
 * <TR>
 *	<TD>osen</TD>
 *	<TD>1 (enable) or 0 (disable)</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 * </TR>
 * <TR>
 *	<TD>hs20_deauth_req_timeout</TD>
 *	<TD>Deauthentication request timeout in seconds</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 *	<TD>-</TD>
 * </TR>
 * </TABLE>
 * </small>
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_hs20_params \<WiFi interface\> \<hs_param\> \<value1\> \<value2\> \<value3\>
 *	\<value4\> \<value5\> \<value6\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_hs20_params(const char *ifname, const string_32 hs_param,
					const string_64 value1, const string_64 value2,
					const string_64 value3, const string_64 value4,
					const string_64 value5, const string_64 value6);

/**
 * \brief Remove the 802.11u parameter
 *
 * Remove the value for the specified 802.11u parameter from hostapd.conf file,
 * as specified by the qcsapi_11u_params parameter.
 *
 * \param ifname \wifi0
 * \param param 802.11u parameter to be removed
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_11u_param \<WiFi interface\> \<param\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_remove_11u_param(const char *ifname, const string_64 param);

/**
 * \brief Remove a Hotspot 2.0 parameter
 *
 * Remove the specified Hotspot 2.0 parameter. Refer to \link qcsapi_wifi_set_hs20_params \endlink
 * for a list of valid parameter names.
 *
 * \param ifname \wifi0
 * \param hs_param Hotspot 2.0 parameter to be removed
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_hs20_param \<WiFi interface\> \<hs_param\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_remove_hs20_param(const char *ifname, const string_64 hs_param);

/**
 * \brief Add a Hotspot 2.0 icon description
 *
 * Add a description for a Hotspot 2.0 icon.
 *
 * \param ifname \wifi0
 * \param icon_width icon width in pixels
 * \param icon_height icon height in pixels
 * \param lang_code 2 or 3 character ISO-639 language code
 * \param icon_type icon type e.g. "png"
 * \param icon_name name for the icon
 * \param file_path path to the image file; must be an existing file
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \note There can be multiple icon descriptions, but the names must be unique.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_hs20_icon \<WiFi interface\> \<icon_width\> \<icon_height\> \<lang_code\>
 *	\<icon_type\> \<icon_name\> \<file_path\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_hs20_icon(const char *ifname, const qcsapi_unsigned_int icon_width,
					const qcsapi_unsigned_int icon_height,
					const char *lang_code, const char *icon_type, const char
					*icon_name, const char *file_path);

/**
 * \brief Get all Hotspot 2.0 icon descriptions
 *
 * Get all configured Hotspot 2.0 icon descriptions. Each description has the following format.
 *
 * <c>\<Icon Width\>:\<Icon Height\>:\<Language Code\>:\<Icon Type\>:\<Icon Name\>:\<File Path\></c>
 *
 * \param ifname \wifi0
 * \param value pointer to buffer to contain returned icon descriptions
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hs20_icon \<WiFi interface\></c>
 *
 * The output will be the descriptions for Hotspot 2.0 icons, unless an error occurs.
 */
extern int qcsapi_security_get_hs20_icon(const char *ifname, string_1024 value);

/**
 * \brief Delete a Hotspot 2.0 icon description
 *
 * Delete the description for Hotspot 2.0 icon specified by name.
 *
 * \param ifname \wifi0
 * \param icon_name name of the icon to be deleted
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_hs20_icon \<WiFi interface\> \<icon_name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_hs20_icon(const char *ifname, const string_1024 icon_name);

/**
 * \brief Get all OSU servers URIs
 *
 * Get all configured Online Sign Up provider URIs.
 * Each returned URI string is separated by a newline character.
 *
 * \param ifname \wifi0
 * \param value pointer to buffer to contain the returned strings
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_osu_server_uri \<WiFi interface\></c>
 *
 * The output will be the URIs for all OSU providers, unless an error occurs.
 */
extern int qcsapi_security_get_osu_server_uri(const char *ifname,
						string_1024 value);

/**
 * \brief Delete OSU Provider
 *
 * Delete an Online Sign Up provider server identified by a specified URI.
 * All parameters for the specified provider are also deleted.
 *
 * \param ifname \wifi0
 * \param osu_server_uri OSU provider URI
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_osu_server_uri \<WiFi interface\> \<osu_server_uri\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_osu_server_uri(const char *ifname,
							const string_256 osu_server_uri);

/**
 * \brief Add an OSU Provider server parameter
 *
 * Add a parameter for an Online Sign Up provider identified by a specified URI.
 * Valid parameters are described in the following table.
 *
 * <table>
 * <tr>
 *	<td>Parameter</td>
 *	<td>Description</td>
 *	<td>Multiple Entries</td>
 * </tr>
 * <tr>
 *	<td>osu_friendly_name</td>
 *	<td>
 *	Friendly name for OSU provider in the following format:
 *	<c>\<Lang Code\>:\<Friendly Name\></c>
 *	</td>
 *	<td>Yes</td>
 * </tr>
 * <tr>
 *	<td>osu_nai</td>
 *	<td>Network Access Identifier</td>
 *	<td>No</td>
 * </tr>
 * <tr>
 *	<td>osu_method_list</td>
 *	<td>
 *	List of OSU methods separated by spaces. Valid values are:
 *	0 - OMA DM, 1 - SOAP XML SPP
 *	</td>
 *	<td>No</td>
 * </tr>
 * <tr>
 *	<td>osu_icon</td>
 *	<td>Name of the configured Hotspot 2.0 icon</td>
 *	<td>Yes</td>
 * </tr>
 * <tr>
 *	<td>osu_service_desc</td>
 *	<td>
 *	OSU services description in the following format:
 *	<c>\<Lang Code\>:\<Description\></c>
 *	</td>
 *	<td>Yes</td>
 * </tr>
 * </table>
 *
 * \param ifname \wifi0
 * \param osu_server_uri OSU provider URI
 * \param param name of the parameter to be set
 * \param value parameter value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_osu_server_param \<WiFi interface\> \<osu_server_uri\> \<param\>
 *	\<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_osu_server_param(const char *ifname,
						const string_256 osu_server_uri,
						const string_256 param,
						const string_256 value);

/**
 * \brief Get values for OSU server parameter
 *
 * Get all values for the specified parameter for an Online Sign Up provider.
 * Value strings separated by new lines are placed in the returned buffer.
 * Parameters are the same as for \link qcsapi_security_add_osu_server_param \endlink.
 *
 * \param ifname \wifi0
 * \param osu_server_uri URI of the OSU provider
 * \param param parameter name
 * \param value pointer to buffer to contain returned strings
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_osu_server_param \<WiFi interface\> \<param\></c>
 *
 * The output will be the values for OSU provider parameter, unless an error occurs.
 */
extern int qcsapi_security_get_osu_server_param(const char *ifname,
						const string_256 osu_server_uri,
						const string_256 param,
						string_1024 value);

/**
 * \brief Delete an OSU Provider parameter
 *
 * Delete an Online Sign Up provider parameter with a specific value.
 *
 * \param ifname \wifi0
 * \param osu_server_uri OSU provider URI
 * \param param parameter name
 * \param value parameter value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_osu_server_param \<WiFi interface\> \<osu_server_uri\> \<param\>
 *	\<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_del_osu_server_param(const char *ifname,
						const string_256 osu_server_uri,
						const string_256 param,
						const string_256 value);

/**
 * \brief Add OSU Provider server URI
 *
 * Add an Online Sign Up provider server URI. Each URI starts a new OSU provider description that
 * might contain parameters added by \link qcsapi_security_add_osu_server_param \endlink.
 *
 * \param ifname \wifi0
 * \param osu_server_uri OSU provider URI
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \note \hotspot_api
 *
 * \note There can be multiple OSU provider servers, but each must have a unique URI.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_osu_server_uri \<WiFi interface\> \<osu_server_uri\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_add_osu_server_uri(const char *ifname,
						const string_256 osu_server_uri);

/**
 * \brief see qcsapi_wifi_get_WPA_encryption_modes
 *
 * \sa qcsapi_wifi_get_WPA_encryption_modes
 */
extern int qcsapi_wifi_get_IEEE11i_encryption_modes(const char *ifname, string_32 encryption_modes);

/**
 * \brief see qcsapi_wifi_set_WPA_encryption_modes
 *
 * \sa qcsapi_wifi_set_WPA_encryption_modes
 */
extern int qcsapi_wifi_set_IEEE11i_encryption_modes(const char *ifname,
					const string_32 encryption_modes);

/**
 * \brief see qcsapi_wifi_get_WPA_authentication_mode
 *
 * \sa qcsapi_wifi_get_WPA_authentication_mode
 */
extern int qcsapi_wifi_get_IEEE11i_authentication_mode(const char *ifname,
					string_32 authentication_mode);

/**
 * \brief see qcsapi_wifi_set_WPA_authentication_mode
 *
 * \sa qcsapi_wifi_set_WPA_authentication_mode
 */
extern int qcsapi_wifi_set_IEEE11i_authentication_mode(const char *ifname,
					const string_32 authentication_mode);

/**
 * @brief Get TKIP MIC errors count.
 *
 * The total number of times the Michael integrity check has failed. This is an accumulated value of
 * number of times MIC check failed starting from the beginning of device operation. Used for
 * information purposes, it is not used directly for triggering Michael countermeasures event.
 * Relevant only to WPA and 802.11i.
 *
 * \param ifname \wifi0
 * \param errcount a pointer to memory where MIC error count value should be placed
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_michael_errcnt \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the total number of Michael integrity check errors on
 * specified interface.
 */
extern int qcsapi_wifi_get_michael_errcnt(const char *ifname, uint32_t *errcount);

/**
 * \brief Get the preshared key
 *
 * Get the WPA or RSN preshared key for an SSID.
 *
 * \param ifname \wifi0only
 * \param key_index reserved - set to zero
 * \param pre_shared_key a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pre_shared_key \<WiFi interface\> \<key index\></c>
 *
 * <c>call_qcsapi get_PSK \<WiFi interface\> \<key index\></c>
 *
 * The output will be the preshared key unless an error occurs.
 */
extern int qcsapi_wifi_get_pre_shared_key(const char *ifname, const qcsapi_unsigned_int key_index,
					string_64 pre_shared_key);

/**
 * \brief Set the preshared key
 *
 * Set the WPA or RSN preshared key for an SSID.
 *
 * \param ifname \wifi0only
 * \param key_index reserved - set to zero
 * \param pre_shared_key a 64 hex digit PSK
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pre_shared_key \<WiFi interface\> \<key index\> \<preshared key\></c>
 *
 * <c>call_qcsapi set_PSK \<WiFi interface\> \<key index\> \<preshared key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pre_shared_key(const char *ifname, const qcsapi_unsigned_int key_index,
					const string_64 pre_shared_key);

/**
 * \brief Add RADIUS authentication server
 *
 * Add RADIUS authentication server configuration
 *
 * \param ifname \wifi0
 * \param radius_auth_server_ipaddr - IP address of the RADIUS server
 * \param radius_auth_server_port - Port of the RADIUS server
 * \param radius_auth_server_sh_key - Shared secret key of the RADIUS server
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_radius_auth_server_cfg \<WiFi interface\> \<ipaddr\> \<port\> \<sh_key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_radius_auth_server_cfg(const char *ifname,
			const char *radius_auth_server_ipaddr, const char *radius_auth_server_port,
			const char *radius_auth_server_sh_key);

/**
 * \brief Remove RADIUS authentication server
 *
 * Remove RADIUS authentication server configuration
 *
 * \param ifname \wifi0
 * \param radius_auth_server_ipaddr - IP address of RADIUS server
 * \param radius_auth_server_port - Port of the RADIUS server
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_radius_auth_server_cfg \<WiFi interface\> \<ipaddr\> \<port\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_radius_auth_server_cfg(const char *ifname,
							const char *radius_auth_server_ipaddr,
							const char *constp_radius_port);

/**
 * \brief Get RADIUS authentication servers
 *
 * Get RADIUS authentication servers configuration
 *
 * \param ifname \wifi0
 * \param radius_auth_server_cfg - reads the RADIUS server configuraion
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radius_auth_server_cfg \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of the RADIUS servers.
 */
extern int qcsapi_wifi_get_radius_auth_server_cfg(const char *ifname,
							string_1024 radius_auth_server_cfg);

/**
 * \brief Set the EAP own ip address of the AP
 *
 * Set the EAP own ip address of the AP.
 *
 * \param ifname \wifi0only
 * \param own_ip_addr
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_own_ip_addr \<WiFi interface\> \<own ip addr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_own_ip_addr(const char *ifname, const string_16 own_ip_addr);

/**
 * @brief Get the WiFi passphrase for the given interface.
 *
 * Returns the current WPA/11i passphrase. Applies to AP only. For a STA, use
 * \ref qcsapi_SSID_get_key_passphrase.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_index - reserved, set to 0.
 * \param passphrase a string to store the passphrase.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_key_passphrase \<WiFi interface\> 0</c>
 *
 * <c>call_qcsapi get_passphrase \<WiFi interface\> 0</c>
 *
 * Unless an error occurs, the output will be the current passphrase.
 *
 * The final '0' in the command line represents the key index.
 */
extern int qcsapi_wifi_get_key_passphrase(const char *ifname, const qcsapi_unsigned_int key_index,
					string_64 passphrase);

/**
 * @brief Set the WiFi passphrase (ASCII) for the given interface.
 *
 * Sets the WPA/11i ASCII passphrase. Applies to AP only. For a STA, use
 * \ref qcsapi_SSID_set_key_passphrase.
 *
 * By the WPA standard, the passphrase is required to have between 8 and 63 ASCII characters.
 *
 * \note \aponly
 * \note \defer_mode_supported
 * \sa set_security_defer_mode
 *
 * \param ifname \wifi0
 * \param key_index - reserved, set to 0.
 * \param the NULL terminated passphrase string, 8 - 63 ASCII characters (NULL termination not
 *	included in the count)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_key_passphrase \<WiFi interface\> 0</c>
 *
 * <c>call_qcsapi set_passphrase \<WiFi interface\> 0</c>
 *
 * \call_qcsapi_string_complete
 *
 * The final '0' in the command line represents the key index.
 *
 * \note The Linux shell processes the passphase parameter. Selected characters are interpreted by
 * the shell, including the dollar sign ($), the backslash (\) and the backquote (`). We recommend
 * putting the new passphrase in quotes and/or using the backslash character to "escape" characters
 * that could be processed by the shell.
 *
 * \sa qcsapi_SSID_get_key_passphrase
 */
extern int qcsapi_wifi_set_key_passphrase(const char *ifname, const qcsapi_unsigned_int key_index,
					const string_64 passphrase);

/**
 * \brief Get the group key rotation interval.
 *
 * Get the group key interval.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_key_interval a pointer an integer to contain the group key rotation interval in seconds
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_group_key_interval \<WiFi interface\></c>
 *
 * The output will be the group key interval unless an error occurs.
 *
 */
extern int qcsapi_wifi_get_group_key_interval(const char *ifname, unsigned int *p_key_interval);

/**
 * \brief Get the pairwise key rotation interval.
 *
 * Get the pairwise key rotation interval. This interval is used to timeout and cause a new PTK to
 * be generated.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_key_interval a pointer an integer to contain the pairwise key rotation interval in
 *	seconds.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pairwise_key_interval \<WiFi interface\></c>
 *
 * The output will be the pairwise key interval unless an error occurs.
 *
 */
extern int qcsapi_wifi_get_pairwise_key_interval(const char *ifname, unsigned int *p_key_interval);

/**
 * \brief Set the group key rotation interval.
 *
 * Set the group key interval.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_interval the group key rotation interval in seconds. Set to 0 for no key rotation.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_group_key_interval \<WiFi interface\> \<group key interval\></c>
 *
 * \call_qcsapi_string_complete
 *
 */
extern int qcsapi_wifi_set_group_key_interval(const char *ifname, const unsigned int key_interval);

/**
 * \brief Set the pairwise key rotation interval.
 *
 * Set the pairwise key rotation interval. This interval is used to timeout and cause a new PTK to
 * be generated.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param key_interval The pairwise key rotation interval in seconds. Set to 0 for no key rotation.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pairwise_key_interval \<WiFi interface\> \<pairwise key interval\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pairwise_key_interval(const char *ifname,
					const unsigned int key_interval);

/**
 * @brief Get the 802.11w capability for the given interface.
 *
 * Returns the current 802.11w pmf capability. Applies to AP.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param passphrase an int rt.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pmf\<WiFi interface\> 0</c>
 *
 * <c>call_qcsapi get_pmf \<WiFi interface\> 0</c>
 *
 * Unless an error occurs, the output will be the current pmf capability.
 *
 * \sa qcsapi_SSID_get_pmf
 */
extern int qcsapi_wifi_get_pmf(const char *ifname, int *p_pmf_cap);

/**
 * @brief Set the 802.11w (PMF) capability for the given interface.
 *
 * Set the 802.11w (PMF) capability for the given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param pmf_cap PMF capability
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pmf \<WiFi interface\> 0</c>
 *
 * <c>call_qcsapi set_pmf \<WiFi interface\> 0</c>
 *
 * \call_qcsapi_string_complete
 *
 * The final '0' in the command line represents the key index.
 *
 * \note The Linux shell processes the pmf parameter
 *
 * \sa qcsapi_SSID_set_pmf
 */
extern int qcsapi_wifi_set_pmf(const char *ifname, int pmf_cap);


/**
 * @brief Get the the WPA status for the given interface.
 *
 * Returns the current WPA status. Only applies to AP.
 *
 * Possible WPA status are:
 * For AP
 * \li <c>WPA_HANDSHAKING</c> - WPA handshaking started.
 * \li <c>NO_WPA_HANDSHAKING</c> - WPA handshaking not started.
 * \li <c>WPA_SUCCESS</c> - WPA handshaking is successful.
 *
 * \param ifname \wifi0
 * \param wpa_status return parameter for storing the informative wpa_status string.
 * \param mac_addr the mac_addr of the station that is connecting or connected to the AP.
 * \param max_len the length of the wpa_status string passed in.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wpa_status \<WiFi interface\> \<Mac address\></c>
 *
 * Unless an error occurs, the output will be the current WPA handshaking status for the AP
 */
extern int qcsapi_wifi_get_wpa_status(const char *ifname,
		char *wpa_status,
		const char *mac_addr,
		const qcsapi_unsigned_int max_len);

/**
 * @brief Get the total number of PSK authentication failures.
 *
 * This API returns the total number of PSK authentication failures from
 * the AP and associated stations.
 *
 * \param ifname \wifi0
 * \param count return parameter to contain the count of PSK authentication failures.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_psk_auth_failures \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the count of PSK authentication failures.
 */
extern int qcsapi_wifi_get_psk_auth_failures(const char *ifname, qcsapi_unsigned_int *count);

/**
 * @brief Get the the authenticated state of the specific station according to the mac_addr
 * for the given interface.
 *
 * Returns the authenticated state(0/1). Only applies to AP.
 *
 * Possible authenticated state are:
 * For AP
 * \li <c>1</c> - the station is authorized.
 * \li <c>0</c> - the station is not authorized.
 *
 * \param ifname \wifi0
 * \param mac_addr the mac_addr of the station.
 * \param auth_state the state value to return .
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_auth_state \<WiFi interface\> \< Mac address\></c>
 *
 * Unless an error occurs, the output will be the authorized state for the station
 */
extern int qcsapi_wifi_get_auth_state(const char *ifname,
			const char *mac_addr,
			int *auth_state);

/**
 * \brief Set security defer mode
 *
 * Enable or disable 'security defer mode'. When enabled, each configuration command is applied to
 * the relevant configuration file but is not sent to the driver, which can significantly reduce
 * boot time, especially when many VAPs are configured.
 *
 * The most effective way to use this mode is by configuring the device not to start the drivers at
 * boot time, by using the following (one-time only) command.
 *
 * @code
 * call_qcsapi update_persistent_param global autostart 0
 * @endcode
 *
 * Configuration commands can then be applied after each boot in the following sequence.
 *
 * 1. If the security agent has been enabled, start it
 * @code
 * call_qcsapi sec_agent start <key>
 * @endcode
 *
 * 2. Enable defer mode
 * @code
 * call_qcsapi set_security_defer_mode wifi0_0 1
 * call_qcsapi set_security_defer_mode wifi2_0 1
 * @endcode
 *
 * 3. Configure BSSes - e.g.:
 * @code
 * call_qcsapi set_SSID wifi0_0 qtn-wifi0_0
 * call_qcsapi set_passphrase wifi0_0 0 qtn01234
 * call_qcsapi set_option wifi0_0 SSID_broadcast 1
 * call_qcsapi set_beacon wifi0_0 11i
 *
 * call_qcsapi set_SSID wifi2_0 qtn-wifi2_0
 * call_qcsapi set_passphrase wifi2_0 0 qtn01234
 * call_qcsapi set_option wifi2_0 SSID_broadcast 1
 * call_qcsapi set_beacon wifi2_0 11i
 *
 * call_qcsapi wifi_create_bss wifi0_1 00:11:22:33:44:51
 * call_qcsapi set_SSID wifi0_1 qtn-wifi0_1
 * call_qcsapi set_passphrase wifi0_1 0 qtn01234
 * call_qcsapi set_option wifi0_1 SSID_broadcast 0
 * call_qcsapi set_beacon wifi0_1 11i
 * @endcode
 *
 * 4. Disable defer mode
 * @code
 * call_qcsapi set_security_defer_mode wifi0_0 0
 * call_qcsapi set_security_defer_mode wifi2_0 0
 * @endcode
 *
 * 5. Start the drivers
 * @code
 * call_qcsapi startprod
 * @endcode
 *
 * \note The following commands are supported when security defer mode is enabled.
 *
 * @code
 * qcsapi_wifi_set_SSID
 * qcsapi_wifi_create_restricted_bss
 * qcsapi_wifi_create_bss
 * qcsapi_wifi_set_beacon_type
 * qcsapi_wifi_set_key_passphrase
 * qcsapi_wifi_set_option SSID_broadcast
 * @endcode
 *
 * \note \aponly
 *
 * \sa	qcsapi_wifi_startprod

 * \param ifname \wifi0only
 * \param defer \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * call_qcsapi set_security_defer_mode \<WiFi interface\> {0 | 1}</c>
 */
int qcsapi_wifi_set_security_defer_mode(const char *ifname, int defer);

/**
 * \brief Get security defer mode
 *
 * Get security defer mode
 *
 * \param ifname \wifi0only
 * \param defer \valuebuf
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_security_defer_mode \<WiFi interface\></c>
 */
int qcsapi_wifi_get_security_defer_mode(const char *ifname, int *defer);

/**
 * \brief Apply security configuration
 *
 * Apply stored security configuration to the running system.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi apply_security_config \<WiFi interface\></c>
 */
extern int qcsapi_wifi_apply_security_config(const char *ifname);

/**
 * \brief Add RADIUS accounting server
 *
 * Add RADIUS accounting server configuration
 *
 * \param ifname \wifi0
 * \param radius_server_addr - IP address of the RADIUS server
 * \param radius_server_port - Port of the RADIUS server
 * \param radius_server_shared_secret - Shared secret key of the RADIUS server
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_radius_acct_server_cfg \<WiFi interface\> \<ipaddr\> \<port\> \<sh_key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_radius_acct_server_cfg(const char *ifname,
			const char *radius_server_addr, const char *radius_server_port,
			const char *radius_server_shared_secret);

/**
 * \brief Remove RADIUS accounting server
 *
 * Remove RADIUS accounting server configuration
 *
 * \param ifname \wifi0
 * \param radius_server_addr - IP address of RADIUS server
 * \param radius_server_port - Port of the RADIUS server
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_radius_acct_server_cfg \<WiFi interface\> \<ipaddr\> \<port\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_radius_acct_server_cfg(const char *ifname,
			const char *radius_server_addr, const char *radius_server_port);

/**
 * \brief Get RADIUS accounting servers
 *
 * Get RADIUS accounting servers configuration
 *
 * \param ifname \wifi0
 * \param radius_server_cfg - buffer for returned value
 *
 * \return \zero_or_negative
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radius_acct_server_cfg \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of the RADIUS servers.
 */
extern int qcsapi_wifi_get_radius_acct_server_cfg(const char *ifname,
							string_1024 radius_server_cfg);

/**
 * \brief Set EAP reauthentication period
 *
 * Set EAP reauthentication period in seconds (the default is 3600 seconds).
 *
 * \param ifname \wifi0
 * \param eap_reauth_period - EAP reauthentication period in seconds, or 0 to disable
 * reauthentication.
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_eap_reauth_period \<WiFi interface\> \<eap_reauth_period\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_eap_reauth_period(const char *ifname,
						const qcsapi_unsigned_int eap_reauth_period);

/**
 * \brief Get EAP reauthentication period
 *
 * Get EAP reauthentication period in seconds.
 *
 * \param ifname \wifi0
 * \param eap_reauth_period - return buffer for EAP reauthentication period
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_eap_reauth_period \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of EAP reauthentication period in seconds.
 */
extern int qcsapi_wifi_get_eap_reauth_period(const char *ifname,
						qcsapi_unsigned_int *eap_reauth_period);

/**
 * \brief Remove EAP reauthentication period
 *
 * Remove EAP reauthentication configuration and use default - 3600 seconds.
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_eap_reauth_period \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_eap_reauth_period(const char *ifname);

/**
 * \brief Set maximum retries for RADIUS server access
 *
 * Set maximum retries for sending requests to a RADIUS server. Default - 10 retries.
 *
 * \param ifname \wifi0
 * \param max_retries - maximum number of retries
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_radius_max_retries \<WiFi interface\> \<max_retries\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_radius_max_retries(const char *ifname,
							const qcsapi_unsigned_int max_retries);
/**
 * \brief Get maximum retries for RADIUS server access
 *
 * Get maximum retries for sending requests to a RADIUS server
 *
 * \param ifname \wifi0
 * \param max_retries - return buffer for maximum retries
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radius_max_retries \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of maximum retries.
 */
extern int qcsapi_wifi_get_radius_max_retries(const char *ifname,
							qcsapi_unsigned_int *max_retries);

/**
 * \brief Remove maximum retries for RADIUS server access
 *
 * Remove maximum retries for sending requests to a RADIUS server and use default - 10 retries
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_radius_max_retries \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_radius_max_retries(const char *ifname);

/**
 * \brief Set RADIUS failover retries number
 *
 * Set number of failed retries of RADIUS server access before switching to the next backup server
 *
 * \param ifname \wifi0
 * \param failover_num - maximum of retries before switch to backup (0 disables backup servers)
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_radius_num_failover \<WiFi interface\> \<failover_num\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_radius_num_failover(const char *ifname,
							const qcsapi_unsigned_int num_failover);

/**
 * \brief Get RADIUS failover retries number
 *
 * Get number of failed retries of RADIUS server access before switching to the next backup server
 *
 * \param ifname \wifi0
 * \param num_failover - return buffer for failover retries number
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radius_num_failover \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of failover retries number.
 */
extern int qcsapi_wifi_get_radius_num_failover(const char *ifname,
							qcsapi_unsigned_int *num_failover);

/**
 * \brief Remove RADIUS failover retries number
 *
 * Remove number of failed retries of RADIUS server access before switching to the next backup
 * server and use default (4 attempts)
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_radius_num_failover \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_radius_num_failover(const char *ifname);

/**
 * \brief Set RADIUS request timeout
 *
 * Set timeout of requests to RADIUS server
 *
 * \param ifname \wifi0
 * \param timeout - timeout in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_radius_timeout \<WiFi interface\> \<timeout\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_radius_timeout(const char *ifname,
							const qcsapi_unsigned_int timeout);

/**
 * \brief Get RADIUS request timeout
 *
 * Get timeout of requests to RADIUS server
 *
 * \param ifname \wifi0
 * \param timeout - return buffer for timeout
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radius_timeout \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of timeout.
 */
extern int qcsapi_wifi_get_radius_timeout(const char *ifname,
							qcsapi_unsigned_int *timeout);

/**
 * \brief Remove RADIUS request timeout
 *
 * Remove timeout of requests to RADIUS server
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_radius_timeout \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_radius_timeout(const char *ifname);

/**
 * \brief Enable PMK caching
 *
 * Enable PMK caching.
 *
 * \param ifname \wifi0
 * \param enable_val - 1 - enable caching, 0 - disable
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pmk_cache_enable \<WiFi interface\> \<enable_val\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pmk_cache_enable(const char *ifname,
							const qcsapi_unsigned_int enable_val);

/**
 * \brief Get PMK caching enable setting status
 *
 * Get enable PMK caching setting
 *
 * \param ifname \wifi0
 * \param enable_val - return buffer for enable flag
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pmk_cache_enable \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the setting.
 */
extern int qcsapi_wifi_get_pmk_cache_enable(const char *ifname,
							qcsapi_unsigned_int *enable_val);

/**
 * \brief Set PMK cache lifetime
 *
 * Set the period after successful RADIUS (pre)authentication when PMK resides in AP cache.
 * During this period STA can (re)authenticate without RADIUS seever access.
 * Session-timeout attribute in Access-Accept packet (if set) overrides this setting.
 *
 * \param ifname \wifi0
 * \param lifetime - caching period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pmk_cache_lifetime \<WiFi interface\> \<lifetime\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pmk_cache_lifetime(const char *ifname,
							const qcsapi_unsigned_int lifetime);
/**
 * \brief Get PMK cache lifetime
 *
 * Get the period after successful RADIUS (pre)authentication when PMK resides in AP cache.
 *
 * \param ifname \wifi0
 * \param lifetime - return buffer for lifetime
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pmk_cache_lifetime \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of lifetime.
 */
extern int qcsapi_wifi_get_pmk_cache_lifetime(const char *ifname,
							qcsapi_unsigned_int *lifetime);

/**
 * \brief Remove PMK cache lifetime setting
 *
 * Remove PMK cache lifetime setting
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_pmk_cache_lifetime \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_pmk_cache_lifetime(const char *ifname);

/**
 * \brief Set maximum authentication attempts
 *
 * Set the number of failed authentication attempts before blacklisting the STA.
 * Stations in the blacklist are not allowed to authenticate.
 * Setting this parameter to 0 or removing this parameter disables blacklisting.
 *
 * \param ifname \wifi0
 * \param max_attempts - number of failed attempts
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_max_auth_attempts \<WiFi interface\> \<max_attempts\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_max_auth_attempts(const char *ifname,
							const qcsapi_unsigned_int max_attempts);

/**
 * \brief Get maximum authentication attempts
 *
 * Get the number of failed authentication attempts before blacklisting the STA.
 * Stations in the blacklist are not allowed no authenticate.
 *
 * \param ifname \wifi0
 * \param max_attempts - return buffer for allowed number of attempts
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_max_auth_attempts \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of allowed attempts.
 */
extern int qcsapi_wifi_get_max_auth_attempts(const char *ifname,
							qcsapi_unsigned_int *max_attempts);

/**
 * \brief Set blacklisting period
 *
 * Set period while the station remains in the blacklist after expiring failed
 * authentication attempt limit.
 * Stations in the blacklist are not allowed no authenticate.
 *
 * \param ifname \wifi0
 * \param period - blacklisting period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_set_lockout_period \<WiFi interface\> \<period\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_lockout_period(const char *ifname,
							const qcsapi_unsigned_int period);

/**
 * \brief Get blacklisting period
 *
 * Get period while the station remains in the blacklist after expiring failed
 * authentication attempt limit.
 *
 * \param ifname \wifi0
 * \param period - return buffer for period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_lockout_period \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value period.
 */
extern int qcsapi_wifi_get_lockout_period(const char *ifname,
							qcsapi_unsigned_int *period);

/**
 * \brief Remove blacklisting period setting
 *
 * Remove period while the station remains in the blacklist after expiring failed
 * authentication attempt limit and use the default value (60 seconds)
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_lockout_period(\<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_lockout_period(const char *ifname);

/**
 * \brief Set Request/Identity period
 *
 * Set period between sending EAP Request/Identity packets.
 *
 * \param ifname \wifi0
 * \param period - request period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_id_request_period \<WiFi interface\> \<period\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_id_request_period(const char *ifname,
							const qcsapi_unsigned_int max_retries);

/**
 * \brief Get Request/Identity period
 *
 * Get period between sending EAP Request/Identity packets
 *
 * \param ifname \wifi0
 * \param period - return buffer for period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_lockout_period \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of period.
 */
extern int qcsapi_wifi_get_id_request_period(const char *ifname,
							qcsapi_unsigned_int *period);

/**
 * \brief Remove Request/Identity period setting
 *
 * Remove period between sending EAP Request/Identity packets and use the default value
 * (calculates dynamically)
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_id_request_period(\<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_id_request_period(const char *ifname);

/**
 * \brief Set EAPOL quiet period
 *
 * Set period after failed authentication before starting next retry. During this period
 * the PAE state machine ignores and discards all EAPOL packets, so as to discourage
 * brute force attacks.
 * Setting this parameter to a nonzero value may affect some EAP methods, like
 * EAP-FAST with anonymous provisioning, and some devices using WPS that are not able
 * to deauthenticate themselves.
 *
 * \param ifname \wifi0
 * \param period - quiet period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_auth_quiet_period \<WiFi interface\> \<period\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_auth_quiet_period(const char *ifname,
							const qcsapi_unsigned_int period);

/**
 * \brief EAPOL quiet period
 *
 * Get period after failed authentication before starting next retry
 *
 * \param ifname \wifi0
 * \param period - return buffer for period in seconds
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_auth_quiet_period \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of period.
 */
extern int qcsapi_wifi_get_auth_quiet_period(const char *ifname,
							qcsapi_unsigned_int *period);

/**
 * \brief Remove EAPOL quiet period
 *
 * Remove period after failed authentication before starting next retry and use the
 * default value (0).
 *
 * \param ifname \wifi0
 *
 * \note \aponly
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_auth_quiet_period(\<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_auth_quiet_period(const char *ifname);

/**
 * \brief Perform a Security Agent action
 *
 * Perform a Security Agent action.
 *
 * \param action enable, disable, start
 * \param key a 1-64 byte encryption key (only the first 16 bytes are used)
 * \param flags reserved for future use
 *
 * \return \zero_or_negative
 *
 * \note Refer to the @ref SecurityAgent section for a full explanation of this API.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi sec_agent {enable | disable | start} \<key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_security_set_sec_agent(const char *action, const char *key,
						const uint32_t flags);

/**
 * \brief Get Security Agent status
 *
 * Get Security Agent status.
 *
 * \param status \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_sec_agent_status</c>
 *
 * Unless an error occurs, the output will be <c>disabled</c>, <c>enabled</c> or <c>running</c>.
 */
extern int qcsapi_security_get_sec_agent_status(string_16 status);

/** @} */

/**
 * @addtogroup MACFilterAPIs
 * @{
 */

/**
 * @brief Set MAC address filtering for the given interface.
 *
 * Set the current MAC address filtering, based on the input parameters
 *
 * If the MAC address filtering was configured as Disabled
 * (<c>qcsapi_disable_mac_address_filtering</c>), calling the API to deny access to a MAC address
 * will change the configuration to Accept unless Denied
 * (<c>qcsapi_accept_mac_address_unless_denied</c>), so the referenced MAC address will be blocked
 * from associating.
 *
 * Note that MAC address filtering is disabled by default.
 *
 * \param ifname \wifi0
 * \param new_mac_address_filtering the new MAC address filtering mode to enable.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_macaddr_filter \<WiFi interface\> {0 | 1 | 2}</c>
 *
 * Final argument configures the MAC address filtering:
 *
 * \li 0 to disable MAC address filtering,
 * \li 1 to accept an association unless the MAC address has been blocked,
 * \li 2 to block associations unless the MAC address has been authorized.
 *
 * These values match those in the enumerated data type qcsapi_mac_address_filtering.
 *
 * \call_qcsapi_string_complete
 *
 * \note If the MAC address filtering is set to Accept Unless Blocked, and MAC address filtering is
 * turned off, the list of blocked MAC addresses will be lost.
 *
 * \sa qcsapi_mac_address_filtering
 */
extern int qcsapi_wifi_set_mac_address_filtering(const char *ifname,
			const qcsapi_mac_address_filtering new_mac_address_filtering);

/**
 * @brief Get MAC Address Filtering
 *
 * Get the current MAC address filtering. Returned value will matches one of those in the enumerated
 * data type.
 *
 * This is the dual function of qcsapi_wifi_set_mac_address_filtering.
 *
 * \param ifname \wifi0
 * \param current_mac_address_filtering return parameter to contain the MAC address filtering mode
 * currently enabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_macaddr_filter \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be 0, 1 or 2, representing the enumerators in the
 * enumeration qcsapi_mac_address_filtering.
 */
extern int qcsapi_wifi_get_mac_address_filtering(const char *ifname,
			qcsapi_mac_address_filtering *current_mac_address_filtering);

/**
 * @brief Authorize a MAC address for MAC address filtering.
 *
 * Authorize the referenced MAC address against the MAC address filtering function.
 *
 * \param ifname \wifi0
 * \param address_to_authorize the MAC address of the device to authorize.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi authorize_macaddr \<WiFi interface\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_authorize_mac_address(const char *ifname,
					const qcsapi_mac_addr address_to_authorize);

/**
 * @brief Authorize set of MAC addresses for MAC address filtering.
 *
 * Authorize the referenced MAC addresses (up to 8) against the MAC address filtering function.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param num number of MAC address
 * \param address_list_to_authorize the MAC addresses of the device to authorize.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi authorize_macaddr \<WiFi interface\> \<MAC address0\>
 *	[MAC address1] ... [MAC address7]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_authorize_mac_address_list(const char *ifname, const int num,
					const qcsapi_mac_addr_list address_list_to_authorize);


/**
 * @brief Block MAC addresses using the MAC address filtering feature.
 *
 * Block the referenced MAC address. If the MAC address filtering was configured as disabled,
 * calling this API will change the configuration to accept unless denied.
 *
 * \param ifname \wifi0
 * \param address_to_deny the MAC address to deny.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi deny_macaddr \<WiFi interface\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_deny_mac_address(const char *ifname, const qcsapi_mac_addr address_to_deny);

/**
 * @brief Block MAC addresses using the MAC address filtering feature.
 *
 * Block the referenced MAC addresses (up to 8). If the MAC address filtering was configured as
 * disabled, calling this API will change the configuration to accept unless denied.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param num number of MAC address
 * \param address_list_to_deny the MAC addresses to deny.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi deny_macaddr \<WiFi interface\> \<MAC address\>
 *	[MAC address1] ... [MAC address7]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_deny_mac_address_list(const char *ifname, const int num,
					const qcsapi_mac_addr_list address_list_to_deny);

/**
 * @brief Remove MAC address from the MAC address filtering list.
 *
 * Remove the referenced MAC address.
 *
 * \param ifname \wifi0
 * \param address_to_remove the MAC address to remove.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_macaddr \<WiFi interface\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_mac_address(const char *ifname,
					const qcsapi_mac_addr address_to_remove);

/**
 * @brief Remove MAC address from the MAC address filtering list.
 *
 * Remove the referenced MAC addresses (up to 8).
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param num number of MAC address
 * \param address_list_to_remove the MAC address to remove.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_macaddr \<WiFi interface\>
 *	\<MAC address\> [MAC address1] ... [MAC address7]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_remove_mac_address_list(const char *ifname, const int num,
					const qcsapi_mac_addr_list address_list_to_remove);

/**
 * @brief Check whether a MAC address is authorized.
 *
 * Reports whether a STA with the referenced MAC address is authorized, that is, MAC address
 * filtering will allow the STA to associate.
 *
 * \param ifname \wifi0
 * \param address_to_verify the MAC address to check for authorization.
 * \param p_mac_address_authorized return parameter to indicate authorized (1) or
 *	not authorized (0).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_macaddr_authorized \<WiFi interface\> \<MAC address\></c>
 *
 * Unless an error occurs, the output is either 1 (MAC address can associate) or 0 (MAC address will
 * be blocked from associating).
 */
extern int qcsapi_wifi_is_mac_address_authorized(const char *ifname,
			const qcsapi_mac_addr address_to_verify,
			int *p_mac_address_authorized);

/**
 * @brief Get a list of authorized MAC addresses
 *
 * Get a list of authorized MAC addresses.
 * MAC address filtering must have been configured to Deny unless Authorized
 * <c>(qcsapi_deny_mac_address_unless_authorized)</c>.
 *
 * MAC addresses will be returned in <c>list_mac_addresses</c> up to the size of the parameter, as
 * expressed in <c>sizeof_list</c>, in the standard format for MAC addresses, separated by commas.
 *
 * \param ifname \wifi0
 * \param list_mac_addresses return parameter to contain the list of comma delimited MAC addresses
 * \param sizeof_list the size of the input list_mac_addresses buffer.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_authorized_macaddr \<WiFi interface\> \<size of string\></c>
 *
 * Unless an error occurs, output will be the list of authorized MAC addresses, separated by commas.
 * MAC address filtering must be set to deny unless authorized (2);
 * use <c>call_qcsapi get_macaddr_filter</c> to verify this.
 *
 * Final parameter is the size of the string to receive the list of authorized MAC addresses.
 */
extern int qcsapi_wifi_get_authorized_mac_addresses(const char *ifname, char *list_mac_addresses,
					const unsigned int sizeof_list);

/**
 * @brief Get a list of denied MAC addresses.
 *
 * Get a list of denied or blocked MAC addresses.
 * MAC address filtering must have been configured to accept unless denied
 * (<c>qcsapi_accept_mac_address_unless_denied</c>). MAC addresses will be returned in
 * <c>list_mac_addresses</c> up to the size of the passed in buffer, as expressed in
 * <c>sizeof_list</c>, in the standard format for MAC addresses, separated by commas.
 *
 * \param ifname \wifi0
 * \param list_mac_addresses return parameter to contain the list of comma delimited MAC addresses
 * \param sizeof_list the size of the input list_mac_addresses buffer.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_denied_macaddr \<WiFi interface\> \<size of string\></c>
 *
 * Unless an error occurs, output will be the list of denied MAC addresses, separated by commas.
 * MAC address filtering must be set to accept unless denied (1); use <c>call_qcsapi
 * get_macaddr_filter</c> to verify this. Final parameter is the size of the string to receive the
 * list of denied MAC addresses.
 */
extern int qcsapi_wifi_get_denied_mac_addresses(const char *ifname, char *list_mac_addresses,
					const unsigned int sizeof_list);

/**
 * @brief API to set OUI to filter list.
 *
 * This function can be called to set OUI into filter list.
 *
 * \param ifname \wifi0
 * \param oui Organizationally unique identifier string in full MAC address format.
 * \param flag 1 to insert OUI and 0 to remove OUI to/from white list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_accept_oui_filter \<WiFi interface\> \<OUI\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_accept_oui_filter(const char *ifname, const qcsapi_mac_addr oui,
					int flag);

/**
 * @brief API to get of OUI filter list.
 *
 * This function can be called to get OUI filter list.
 *
 * \param ifname \wifi0
 * \param oui_list Where to receive oui in string format.
 * \param sizeof_list Specifies the size of string that prepare for the list return.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_accept_oui_filter \<WiFi interface\> [size]</c>
 *
 * Unless an error occurs, the output will be string that contains MAC address separated by comma.
 */
extern int qcsapi_wifi_get_accept_oui_filter(const char *ifname, char *oui_list,
					const unsigned int sizeof_list);

/**
 * @brief Clear the MAC address lists.
 *
 * This function can be called to clear any accept or deny lists created using the MAC address
 * filtering APIs.
 *
 * After this is called, the hostapd.deny and hostapd.accept files will be reset to default - any
 * existing MAC addresses in these files will be cleared.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi clear_mac_filters \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_clear_mac_address_filters(const char *ifname);

/** @} */

/**
 * @addtogroup MACReserveAPIs
 * @{
 */

/**
 * @brief Set MAC address reservation
 *
 * Prevent selected MAC addresses from being used by WiFi stations or back-end devices.
 *
 * This feature can be used to ensure that MAC addresses of core networking devices
 * cannot be hijacked by WiFi stations or by devices connected to WiFi stations.
 * This feature also can be used to block traffic from selected MAC address.
 *
 * \note \aponly
 *
 * \param ifname \wifi0only
 * \param addr \mac_format
 * \param mask MAC address mask in the same format as a MAC address
 * \param dir Block direction for the reserved mac address, which is one of ingress, egress and
 * bidirection
 * \param eth_type Allowed Ethernet protocol type in expanded hexadecimal notation, or an empty
 * string to block all types. E.g use '0800' to allow IP packets.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_macaddr_reserve \<WiFi interface\> \<addr\> \<mask\> \<block direction\>
 *	[\<allowed ethernet type\>]</c>
 *
 * \call_qcsapi_string_complete
 *
 * \note A maximum of 6 MAC addresses and/or MAC address ranges may be reserved.
 */
extern int qcsapi_wifi_set_macaddr_reserve(const char *ifname, const char *addr,
		const char *mask, const char *dir, const char *eth_type);

/**
 * @brief Get MAC address reservation
 *
 * Get the list of reserved MAC addresses.
 *
 * \param ifname \wifi0
 * \param buf pointer to a buffer for storing the returned list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_macaddr_reserve \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a list of reserved MAC addresses and masks.
 */
extern int qcsapi_wifi_get_mac_address_reserve(const char *ifname, string_256 buf);

/**
 * @brief Clear MAC address reservation
 *
 * Delete all MAC address reservation configuration.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi clear_macaddr_reserve \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_clear_mac_address_reserve(const char *ifname);

/** @} */

/**
 * @addtogroup OptionsAPIs
 * @{
 */

/* generic */

/**
 * @brief Get a WiFi option
 *
 * Get the value for an option, as specified by the qcsapi_option_type parameter,
 * with the value returned in the address specified by p_current_option.
 *
 * \note Value will be either 0 (disabled, false) or 1 (enabled, true). If the option (feature)
 * is not supported, the API will return <c>-qcsapi_option_not_supported</c>.
 *
 * \note Two options, short_gi and stbc, are global and can only be configured on the primary
 * interface.
 *
 * \param ifname \wifi0
 * \param qcsapi_option the option for which to get the value.
 * \param p_current_option \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_option \<WiFi interface\> \<option\></c>
 *
 * \sa qcsapi_option_type
 */
extern int qcsapi_wifi_get_option(const char *ifname, qcsapi_option_type qcsapi_option,
					int *p_current_option);

/**
 * @brief Set a WiFi option
 *
 * Set the value for an option, as specified by the qcsapi_option_type parameter,
 * with a value of 0 disabling the option or setting it to false and a value of 1 enabling the
 * option or setting it to true. A non-zero value will be interpreted as 1.
 *
 * \note Not all options can be set. If the feature is not supported, the API
 * returns <c>-qcsapi_option_not_supported</c>. For some options having fixed value,
 * the API will return <c>-EOPNOTSUPP</c>.
 * \note \defer_mode_supported Only the SSID_broadcast option can be set in this mode.
 * \sa set_security_defer_mode
 *
 * \note Two options short_gi and stbc are global and can only be configured on a primary interface.
 *
 * \note Enabling qlink option is equivalent to setting it to auto mode (network iface
 * autoselection). Use \link qcsapi_config_update_parameter \endlink to set qlink option to a
 * specific network interface name.
 *
 * \param ifname \wifi0
 * \param qcsapi_option the option to set the value of.
 * \param new_option the new option value..
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_option \<WiFi interface\> \<option\> {0 | 1}</c>
 *
 * For the qcsapi_option_type 'sta_dfs', after enable/disable sta_dfs, the power table must also be
 * updated by using the following command.
 *
 * <c>call_qcsapi restore_regulatory_tx_power wifi0_0</c>
 *
 * \sa qcsapi_option_type.
 */
extern int qcsapi_wifi_set_option(const char *ifname, qcsapi_option_type qcsapi_option,
					int new_option);

/**
 * @brief Get a board related parameter
 *
 * Get the value for the specified board parameter, as specified by
 * the qcsapi_board_parameter_type parameter, with the value returned
 * in the address specified by p_buffer. For parameters that vary across physical radio devices
 * in system, first radio device is queried.
 *
 * If the parameter (feature) is not supported, the API will
 * return <c>-qcsapi_board_parameter_not_supported</c>.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_get_board_parameter.
 *
 * \param board_param the board parameter to get the value of.
 * \param p_buffer return parameter to contain the value of the board parameter.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_board_parameter \<board parameter\></c>
 *
 * \sa qcsapi_board_parameter_type
 */
extern int qcsapi_get_board_parameter(qcsapi_board_parameter_type board_param, string_64 p_buffer);

/**
 * @brief Get a board related parameter
 *
 * Get the value for the specified board parameter, as specified by
 * the qcsapi_board_parameter_type parameter and radio device ID, with the value returned
 * in the address specified by p_buffer.
 * Radio device ID is only used for board parameters relevant to the radio
 * and ignored for other board parameters.
 *
 * If the parameter (feature) is not supported, the API will
 * return <c>-qcsapi_board_parameter_not_supported</c>.
 *
 * \param radio_id \radio_id
 * \param board_param the board parameter to get the value of.
 * \param p_buffer return parameter to contain the value of the board parameter.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_board_parameter \<board parameter\> \<radio ID\></c>
 *
 * \sa qcsapi_board_parameter_type
 */
extern int qcsapi_radio_get_board_parameter(const qcsapi_unsigned_int radio_id,
		qcsapi_board_parameter_type board_param, string_64 p_buffer);

/**
 * @brief Get the feature list
 *
 * Get a list of features supported on the primary radio device.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_get_swfeat_list.
 *
 * \param p_buffer buffer to return the supported features list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_swfeat_list</c>
 *
 */
extern int qcsapi_get_swfeat_list(string_4096 p_buffer);

/**
 * @brief Get a WiFi parameter
 *
 * Get the value for a parameter with the value returned in the address specified by p_value.
 * Parameters supported can be showed by QCSAPI help option.
 *
 * <c>call_qcsapi -h wifi_parameters</c>
 *
 * \param ifname \wifi0
 * \param parameter_name the parameter to get the value of.
 * \param p_value return the value of the parameter.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wifi_param \<WiFi interface\> \<parameter\></c>
 *
 * \sa qcsapi_wifi_param_type
 */
extern int qcsapi_wifi_get_parameter(const char *ifname, qcsapi_wifi_param_type type, int *p_value);

/**
 * @brief Set a WiFi parameter
 *
 * Set the value for an parameter.
 * Parameters supported can be showed by QCSAPI help option.
 *
 * <c>call_qcsapi -h wifi_parameters</c>
 *
 * \param ifname \wifi0
 * \param parameter_name the parameter to set the value of.
 * \param value the value to be set for the parameter.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wifi_param \<WiFi interface\> \<parameter\> \<value\></c>
 *
 * \sa qcsapi_wifi_param_type
 */
extern int qcsapi_wifi_set_parameter(const char *ifname, qcsapi_wifi_param_type type, int value);

/**
 * @brief Get the feature list for a radio
 *
 * Get a list of supported features.
 *
 * \param radio_id \radio_id
 * \param p_buffer buffer to return the supported features list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_swfeat_list \<radio\></c>
 *
 */
extern int qcsapi_radio_get_swfeat_list(const qcsapi_unsigned_int radio_id, string_4096 p_buffer);

/** @} */

/**
 * @addtogroup SSIDAPIs
 * @{
 */

/**
 * @brief Create a new SSID.
 *
 * \param ifname \wifi0
 * \param new_SSID the new SSID to add.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi create_SSID \<WiFi interface\> \<new SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_create_SSID(const char *ifname, const qcsapi_SSID new_SSID);

/**
 * @brief Create a new SSID.
 *
 * \param ifname \wifi0
 * \param new_SSID the new SSID to add. The allowed SSID formats are described
 * in \ref SSID2_RULES.
 * \param fmt format type of the SSID
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi create_SSID \<WiFi interface\> [{str | hexstr}] \<new SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_create_SSID2(const char *ifname, const qcsapi_SSID2 new_SSID,
		qcsapi_ssid_fmt fmt);

/**
 * @brief Remove an existing SSID.
 *
 * \param ifname the interface to remove the SSID from.
 * \param del_SSID the existing SSID to remove.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_SSID \<WiFi interface\> \<del SSID\></c>
 *
 * \call_qcsapi_string_complete
 */

extern int qcsapi_SSID_remove_SSID(const char *ifname, const qcsapi_SSID del_SSID);

/**
 * @brief Remove an existing SSID.
 *
 * \param ifname the interface to remove the SSID from.
 * \param del_SSID the existing SSID to remove.
 * \param fmt format type of the SSID
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_SSID \<WiFi interface\> [{str | hexstr}] \<del SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_remove_SSID2(const char *ifname, const qcsapi_SSID2 del_SSID,
		qcsapi_ssid_fmt fmt);

/**
 * @brief Verify an SSID is present
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to check.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi verify_SSID \<WiFi interface\> \<current SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_verify_SSID(const char *ifname, const qcsapi_SSID current_SSID);

/**
 * @brief Verify an SSID is present
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to check
 * \param fmt format type of the SSID
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi verify_SSID \<WiFi interface\> [{str | hexstr}] \<current SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_verify_SSID2(const char *ifname, const qcsapi_SSID2 current_SSID,
		qcsapi_ssid_fmt fmt);

/**
 * @brief Rename an existing SSID
 *
 * Renames an SSID, as identified by current_SSID.
 * If the original Service Set ID is not present, this API returns an error.
 *
 * Both new_SSID and current_SSID must be strings with between 1 and 32 characters, as outlined in
 * \ref SSID_RULES.
 *
 * \param ifname \wifi0
 * \param current_SSID a currently defined SSID on this interface.
 * \param new_SSID the new SSID value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rename_SSID \<WiFi interface\> \<current SSID\> \<new SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_rename_SSID(const char *ifname, const qcsapi_SSID current_SSID,
					const qcsapi_SSID new_SSID);

/**
 * @brief Rename an existing SSID
 *
 * \param ifname \wifi0
 * \param current_SSID a currently defined SSID on this interface.
 * \param current_fmt the format of the current_SSID.
 * \param new_SSID the new SSID value.
 * \param new_fmt the format of the new_SSID.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rename_SSID \<WiFi interface\> [{str | hexstr}] \<current SSID\>
 * [{str | hexstr}] \<new SSID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_rename_SSID2(const char *ifname, const qcsapi_SSID2 current_SSID,
		qcsapi_ssid_fmt curr_fmt, const qcsapi_SSID2 new_SSID,
		qcsapi_ssid_fmt new_fmt);

/**
 * \brief Get the list of SSIDs
 *
 * Get the list of configured SSIDs.
 *
 * \param ifname \wifi0
 * \param arrayc the maximum number of SSIDs to return
 * \param list_SSID a pointer to the buffer for storing the returned values
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID_list \<WiFi interface\> [max SSIDs]</c>
 *
 * The output will be the SSID list unless an error occurs. An additional optional parameter selects
 * the number of SSIDs to be displayed. The default count is 2; the maximum count is 10.
 */
extern int qcsapi_SSID_get_SSID_list(const char *ifname, const unsigned int arrayc,
					char **list_SSID);

/**
 * \brief Get the list of SSIDs
 *
 * \param ifname \wifi0
 * \param arrayc the maximum number of SSID names to return.
 * \param list_SSID a pointer to the buffer for storing the returned values.
 * \param list_fmt buffer to contain the returned format type of the SSID
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_SSID2_list \<WiFi interface\> [max SSIDs]</c>
 *
 * The output will be the SSID list unless an error occurs. An additional optional parameter selects
 * the number of SSID names to be displayed. The default count is 2, the maximum count is 10.
 * * If the SSID was set as a printable string, it is printed in double quotes.
 * * If the SSID was set as a hex string, it is printed in hexdump format.
 */
extern int qcsapi_SSID_get_SSID2_list(const char *ifname, const unsigned int arrayc,
			char **list_SSID, struct qcsapi_data_32bytes *list_fmt);

/**
 * @brief Set the authentication protocol for an SSID
 *
 * Set the security authentication protocol (WPA or 11i or both) for an SSID.
 * Valid values for new_protocol are WPA, 11i and WPAand11i.
 *
 * This API is the SSID/STA equivalent of the AP only set beacon API.
 *
 * Basic will not be accepted for the new protocol. To disable security for an SSID,
 * use the SSID set authentication mode API (qcsapi_SSID_set_authentication_mode) with an
 * authentication mode of NONE.
 *
 * \param ifname \wifi0
 * \param current_SSID an previously defined SSID to apply the protocol to.
 * \param new_protocol the new protocol, as a string. See the <b>authentication protocol</b> table
 * in \ref CommonSecurityDefinitions for valid values.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_proto \<WiFi interface\> \<SSID\> \<protocol type\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Protocol type needs to be one as listed in the <b>authentication protocol</b> table in
 * \ref CommonSecurityDefinitions.
 *
 * \sa qcsapi_SSID_set_authentication_mode
 * \sa qcsapi_wifi_set_beacon_type
 */
extern int qcsapi_SSID_set_protocol(const char *ifname, const qcsapi_SSID current_SSID,
					const char *new_protocol);

/**
 * \brief Get the security protocol for an SSID.
 *
 * Get the security protocol (WPA or 11i or both) for an SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to check against.
 * \param current_protocol set to one of the values in the <b>authentication protocol</b> table in
 *	\ref CommonSecurityDefinitions
 *
 * \return \zero_or_negative
 *
 * This API is the SSID/STA equivalent of the get beacon API.
 *
 * \note This API should not be used to determine whether security is enabled for a particular SSID.
 * Use the SSID Get Authentication Mode API to determine if security is enabled for the SSID.
 * If the returned value is None, then security is disabled for the targeted SSID.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_proto \<WiFi interface\> \<SSID\></c>
 *
 * Unless an error occurs, the response will be one of the values from the
 * <b>authentication protocol</b> table in \ref CommonSecurityDefinitions.
 *
 * \sa qcsapi_SSID_get_authentication_mode
 * \sa qcsapi_wifi_get_beacon_type
 */
extern int qcsapi_SSID_get_protocol(const char *ifname, const qcsapi_SSID current_SSID,
					string_16 current_protocol);


/**
 * @brief Get the encryption modes for an SSID
 *
 * Get available encryption modes for an SSID.
 *
 * This API is called to determing the encryption modes supported on the given SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to read the encryption modes from.
 * \param encryption_modes a comma delimited set of strings, one for each encryption mode. The
 * values in this string are from the <b>encryption</b> type table in
 * \ref CommonSecurityDefinitions.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_encryption_modes \<WiFi interface\> \<SSID\></c>
 *
 * Unless an error occurs, the output will be one of the strings one of the values from the
 * <b>encryption</b> type table in \ref CommonSecurityDefinitions.
 *
 * \sa qcsapi_SSID_set_encryption_modes
 */
extern int qcsapi_SSID_get_encryption_modes(const char *ifname, const qcsapi_SSID current_SSID,
			string_32 encryption_modes);

/**
 * @brief Set the encryption mode for an SSID.
 *
 * Configure available encryption modes for an SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to set the encryption mode against.
 * \param encryption_modes a value as per the <b>encryption</b> type table in
 *	\ref CommonSecurityDefinitions.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_encryption_modes \<WiFi interface\> \<SSID\>
 *	\<encryption mode(s)\></c>
 *
 * where \<encryption mode(s)\> is one of the modes listed in the <b>encryption</b> type table in
 * \ref CommonSecurityDefinitions.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_set_encryption_modes(const char *ifname, const qcsapi_SSID current_SSID,
			const string_32 encryption_modes);

/**
 * \brief Get the group encryption cipher.
 *
 * Get the group encryption cipher for an SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID for which to retrieve the group encryption cipher
 * \param encryption_mode a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * This command is only supported on Stations.
 *
 * <c>call_qcsapi SSID_get_group_encryption \<WiFi interface\> \<SSID\></c>
 *
 * The output will be the encryption cipher unless an error occurs.
 */
extern int qcsapi_SSID_get_group_encryption(const char *ifname, const qcsapi_SSID current_SSID,
			string_32 encryption_mode);

/**
 * \brief Set the group encryption cipher.
 *
 * Set the group encryption cipher for an SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID for which to set the group encryption cipher
 * \param encryption_mode the cipher to be applied
 *
 * \return \zero_or_negative
 *
 * This command is only supported on Stations.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_group_encryption \<WiFi interface\> \<SSID\> \<encryption_mode\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_set_group_encryption(const char *ifname,
			const qcsapi_SSID current_SSID, const string_32 encryption_mode);

/**
 * @brief Get the authentication mode for an SSID.
 *
 * Get the current configured authentication mode for an SSID.
 *
 * This API is the SSID/STA version of WPA get authentication mode API
 * (qcsapi_wifi_get_WPA_authentication_mode). See that section for a description of possible
 * authentication modes returned by this API.
 * If security is disabled for the referenced SSID, this API will return <c>NONE</c>.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to get the authentication modes from.
 * \param authentication_mode return parameter to store the authentication type.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_authentication_mode \<WiFi interface\> \<SSID\></c>
 *
 * Unless an error occurs, the response will be of <c>PSKAuthentication, EAPAuthentication</c> or
 * <c>NONE</c>. That is, one of the values outlined in the <b>authentication type</b> table in
 * \ref CommonSecurityDefinitions.
 *
 * A response of NONE implies security is disabled for the referenced SSID.
 *
 * \sa qcsapi_wifi_get_WPA_authentication_mode
 */
extern int qcsapi_SSID_get_authentication_mode(const char *ifname,
			const qcsapi_SSID current_SSID, string_32 authentication_mode);

/**
 * @brief Set the authentication mode for an SSID.
 *
 * Set the authentication mode for an SSID.
 *
 * This API is the SSID/STA version of WPA set authentication mode API
 * (qcsapi_wifi_set_WPA_authentication_mode);
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to get the authentication modes from.
 * \param authentication_mode the authentication mode to use. One of the values from the
 *	<b>authentication type</b> table in \ref CommonSecurityDefinitions.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_authentication_mode \<WiFi interface\> \<SSID\>
 *	\<authentication mode\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Valid values for the authentication mode parameter are outlined in the <b>authentication type</b>
 * table in \ref CommonSecurityDefinitions.
 *
 * To disable authentication on the SSID, pass the value <c>NONE</c> to the authentication mode
 * parameter.
 *
 * \sa qcsapi_wifi_set_WPA_authentication_mode
 */
extern int qcsapi_SSID_set_authentication_mode(const char *ifname,
			const qcsapi_SSID current_SSID, const string_32 authentication_mode);

/**
 * \brief Get the preshared key
 *
 * Get the WPA or RSN preshared key for an SSID.
 *
 * \note \staonly
 *
 * \param ifname \wifi0only
 * \param current_SSID the SSID for which to retrieve the preshared key
 * \param key_index reserved - set to zero
 * \param pre_shared_key a pointer to the buffer for storing the returned value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_pre_shared_key \<WiFi interface\> \<SSID\> \<key index\></c>
 *
 * The output will be the preshared key unless an error occurs.
 */

extern int qcsapi_SSID_get_pre_shared_key(const char *ifname, const qcsapi_SSID current_SSID,
			const qcsapi_unsigned_int key_index, string_64 pre_shared_key);

/**
 * \brief Set the preshared key
 *
 * Set the WPA or RSN preshared key for an SSID.
 *
 * \note \staonly
 *
 * \param ifname \wifi0only
 * \param current_SSID the SSID to set the WPA PSK on
 * \param key_index reserved - set to zero
 * \param pre_shared_key a 64 hex digit PSK
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_pre_shared_key \<WiFi interface\> \<SSID\> \<key index\>
 *	\<preshared key\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_set_pre_shared_key(const char *ifname, const qcsapi_SSID current_SSID,
			const qcsapi_unsigned_int key_index, const string_64 pre_shared_key);

/**
 * @brief Get the passphrase for an SSID.
 *
 * Get the passphrase for an SSID.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to get the passphrase for.
 * \param key_index reserved - set to zero
 * \param passphrase return parameter to contain the NULL terminated passphrase.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_key_passphrase \<WiFi interface\> \<SSID\> 0</c>
 *
 * <c>call_qcsapi SSID_get_passphrase \<WiFi interface\> \<SSID\> 0</c>
 *
 * Unless an error occurs, the output will be the passphrase configured for the SSID.
 */
extern int qcsapi_SSID_get_key_passphrase(const char *ifname, const qcsapi_SSID current_SSID,
			const qcsapi_unsigned_int key_index, string_64 passphrase);

/**
 * @brief Set the passphrase (ASCII) for an SSID.
 *
 * Set the ASCII passphrase for a Service Set on a STA
 *
 * By the WPA standard, the passphrase is required to have between 8 and 63 ASCII characters.
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID to set the passphrase on.
 * \param key_index reserved - set to zero
 * \param the NULL terminated passphrase string, 8 - 63 ASCII characters (NULL termination not
 *	included in the count)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_key_passphrase \<WiFi interface\> \<SSID\> 0 \<new passphrase\></c>
 *
 * <c>call_qcsapi SSID_set_passphrase \<WiFi interface\> \<SSID\> 0 \<new passphrase\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_SSID_set_key_passphrase(const char *ifname, const qcsapi_SSID current_SSID,
			const qcsapi_unsigned_int key_index, const string_64 passphrase);

/**
 * @brief Configure a security parameter
 *
 * Configure a security parameter.
 *
 * \note The parameter will not take effect until the daemon is reconfigured.
 * \note Not all parameters are validated - make sure they are valid.
 *
 * \param ifname interface on which to perform the action
 * \param mode AP or STA
 * \param bss_id SSID to which the param is to be applied (STA mode only)
 * \param param_name parameter name
 * \param param_value parameter value
 * \n For parameters other than "ssid", the value of "NULL" can be used to delete the parameter.
 * \n Parameter of "ssid" will be deleted if the value is not identical to param ssid
 * \param param_type parameter attribute, quotes around the string or the bare strting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_bss_cfg \<WiFi interface\>
 *	{ap \<WiFi interface\> | sta \<ssid\>} \<param_name\> \<param_value\> [\<param_type\>]</c>
 *
 * \call_qcsapi_string_complete
 *
 * Example of adding or updating an access point parameter:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 ap wifi0_0 ignore_broadcast_ssid 1</c>
 *
 * Example of deleting an access point parameter:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 ap wifi0_0 ignore_broadcast_ssid NULL</c>
 *
 * Example of adding or updating a station parameter in the network with ssid as string "1234":
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 sta 1234 psk password 1</c>
 *
 * Example of deleting a station parameter in the network with ssid as string "1234":
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 sta 1234 psk NULL 1</c>
 */
extern int qcsapi_wifi_update_bss_cfg(const char *ifname,
		const qcsapi_wifi_mode wifi_mode,
		const char *bss_id, const char *param_name,
		const char *param_value, const char *param_type);

/**
 * @brief Configure a security parameter
 *
 * Configure a security parameter.
 *
 * \note The parameter will not take effect until the daemon is reconfigured.
 * \note Not all parameters are validated - make sure they are valid.
 *
 * \param ifname the interface to perform the action on (AP mode only)
 * \param mode ap or sta
 * \param bss_id SSID to which the param is to be applied (STA mode only)
 * \param fmt format type of the SSID (STA mode only)
 * \param param_name parameter name
 * \param param_value parameter value
 * \n For parameters other than "ssid", the value of "NULL" can be used to delete the parameter.
 * \n Parameter of "ssid" will be deleted if the value is not identical to param ssid
 * \param param_type parameter attribute, quotes around the string or the bare string
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi update_bss_cfg \<WiFi interface\>
 *	{ap \<WiFi interface\> | sta [\<SSID fmt\>] \<ssid\>} \<param_name\>
 *	\<param_value\> [<param_type>]</c>
 *
 * \call_qcsapi_string_complete
 *
 * Example of adding or updating an access point parameter:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 ap wifi0_0 ignore_broadcast_ssid 1</c>
 *
 * Example of deleting an access point parameter:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 ap wifi0_0 ignore_broadcast_ssid NULL</c>
 *
 * Example of adding or updating a station parameter in the network with ssid as hex string 1234:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 sta hexstr 1234 psk password 1</c>
 *
 * Example of deleting a station parameter in the network with ssid as hex string 1234:
 *
 * <c>call_qcsapi update_bss_cfg wifi0_0 sta hexstr 1234 psk NULL 1</c>
 */
extern int qcsapi_wifi_update_bss2_cfg(const char *ifname,
		const qcsapi_wifi_mode wifi_mode,
		const char *bss_id, qcsapi_ssid_fmt fmt, const char *param_name,
		const char *param_value, const char *param_type);

/**
 * @brief Get a security parameter
 *
 * Get a security parameter.
 *
 * \note This API directly reads the security daemon configuration file.
 *
 * \param ifname \wifi0only
 * \param wifi_mode ap or sta
 * \param bss_id SSID to which the param is to be applied (STA mode only)
 * \param param_name parameter name
 * \param param_value \valuebuf
 * \param value_len length of the buffer
 *
 * \return \zero_or_negative
 *
 * \sa qcsapi_wifi_get_bss2_cfg
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bss_cfg \<WiFi interface\>
 *	{ ap \<WiFi interface\> | sta \<SSID\> } \<param_name\></c>.
 *
 * Unless an error occurs, the output will be the parameter value.
 *
 * Example for getting a parameter in AP mode:
 *
 * <c>call_qcsapi get_bss_cfg wifi0_0 ap wifi0_2 wpa_key_mgmt
 *
 * Example for getting a parameter in STA mode:
 *
 * <c>call_qcsapi get_bss_cfg wifi0_0 sta Quantenna-wifi0_0 proto
 */
extern int qcsapi_wifi_get_bss_cfg(const char *ifname, const qcsapi_wifi_mode wifi_mode,
		const char *bss_id, const char *param_name, char *param_value, int value_len);

/**
 * @brief Get a security parameter
 *
 * Get a security parameter.
 *
 * \note This API directly reads the security daemon configuration file.
 *
 * \param ifname \wifi0only
 * \param wifi_mode ap or sta
 * \param bss_id SSID to which the param is to be applied (STA mode only)
 * \param fmt format type of the SSID (STA mode only)
 * \param param_name parameter name
 * \param param_value \valuebuf
 * \param value_len length of the buffer
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bss_cfg \<WiFi interface\>
 *	{ ap \<WiFi interface\> | sta [{ str | hexstr }] \<SSID\> } \<param_name\></c>.
 *
 * Unless an error occurs, the output will be the parameter value.
 *
 * Example for getting a parameter in AP mode:
 *
 * <c>call_qcsapi get_bss_cfg wifi0_0 ap wifi0_2 wpa_key_mgmt
 *
 * Example for getting a parameter in STA mode:
 *
 * <c>call_qcsapi get_bss_cfg wifi0_0 sta Quantenna-wifi0_0 str proto
 */
extern int qcsapi_wifi_get_bss2_cfg(const char *ifname, const qcsapi_wifi_mode wifi_mode,
		const char *bss_id, qcsapi_ssid_fmt fmt, const char *param_name,
		char *param_value, int value_len);

/**
 * @brief Get the 802.11w capability for the given interface.
 *
 * Returns the current 802.11w / PMF capability.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID on which to get the PMF capability
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_pmf \<WiFi interface\> \<SSID\></c>
 *
 * Unless an error occurs, the output will be the current PMF capability.
 *
 * \sa qcsapi_SSID_get_pmf
 */
extern int qcsapi_SSID_get_pmf(const char *ifname, const qcsapi_SSID current_SSID, int *p_pmf_cap);

/**
 * @brief Set the 802.11w / PMF capability for the given interface.
 *
 * Sets the 802.11w / PMF capability.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param current_SSID the SSID on which to set the PMF capability
 * \param pmf_cap.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_set_pmf \<WiFi interface\> \<SSID\> {0 | 1 | 2}</c>
 *
 * \call_qcsapi_string_complete
 *
 * The final '0' in the command line represents the key index.
 *
 * \note The Linux shell processes the PMF parameter
 *
 * \sa qcsapi_SSID_set_pmf
 */

extern int qcsapi_SSID_set_pmf(const char *ifname, const qcsapi_SSID SSID_str, int pmf_cap);

/**
 * \brief Get the SSID associated with the WPS session.
 *
 * This API returns the SSID as configured via WPS. This network block
 * is marked using a 'flags' parameter to indicate that the SSID was
 * configured via WPS.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param wps_SSID the return value to contain the SSID.
 *
 * \return \positive_or_negative
 * \return -qcsapi_configuration_error if the SSID is not one configured via WPS
 *
 * \callqcsapi
 *
 * <c>call_qcsapi SSID_get_WPS_SSID \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string containing the SSID, as obtained via WPS.
 */
extern int qcsapi_SSID_get_wps_SSID(const char *ifname, qcsapi_SSID wps_SSID);

/** @} */

/**
 * @addtogroup VLANAPIs
 * @{
 */

/**
 * \brief VLAN configuration
 *
 * Configure VLANs.
 *
 * \param ifname the interface to perform the action on
 * \param cmd VLAN command \ref qcsapi_vlan_cmd
 * \param vlanid VLAN identifier, 1 - 4095
 *
 * \return \zero_or_negative
 *
 * \note The <c>enable</c> and <c>disable</c> commands are global, so the <c>interface</c>
 * parameter is ignored. VLAN config cannot be applied to interfaces until VLANs have been enabled
 * by using the <c>enable</c> command.
 *
 * \callqcsapi
 *
 * @code
 * call_qcsapi vlan_config wifi0_0 enable
 * call_qcsapi vlan_config wifi0_0 disable
 * call_qcsapi vlan_config <interface> reset
 * call_qcsapi vlan_config <interface> access <VLAN ID> [delete] [vlan_prio]
 * call_qcsapi vlan_config <interface> trunk <VLAN ID> {tag | untag}
 *						[default] [delete] [vlan_prio]
 * call_qcsapi vlan_config <interface> dynamic {0 | 1}
 * @endcode
 *
 * The following commands are deprecated. The <c>access</c> keyword provides the same functionality.
 *
 * @code
 * call_qcsapi vlan_config <interface> bind <VLAN ID>
 * call_qcsapi vlan_config <interface> unbind <VLAN ID>
 * @endcode
 *
 * Examples:
 *
 * @code
 * call_qcsapi vlan_config wifi0_0 enable
 * call_qcsapi vlan_config wifi0_0 disable
 * call_qcsapi vlan_config wifi2_0 reset
 * call_qcsapi vlan_config wifi2_3 access 10
 * call_qcsapi vlan_config wifi2_3 access 10 delete
 * call_qcsapi vlan_config wifi2_3 access 10 vlan_prio
 * call_qcsapi vlan_config wifi2_0 trunk 10
 * call_qcsapi vlan_config wifi2_0 trunk 10 default
 * call_qcsapi vlan_config wifi2_0 trunk 10 delete
 * call_qcsapi vlan_config wifi2_0 trunk 10 vlan_prio
 * call_qcsapi vlan_config wifi2_0 trunk 10 default untag
 * call_qcsapi vlan_config wifi2_0 dynamic 1
 * call_qcsapi vlan_config wifi2_0 dynamic 0
 * @endcode
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_vlan_config(const char *ifname, qcsapi_vlan_cmd cmd, uint32_t vlanid);

/**
 * \brief Get VLAN configuration
 *
 * Get VLAN configuration for an interface.
 *
 * \param ifname the interface to perform the action on
 * \param vcfg the structure to retrieve the VLAN configuration
 * \param flag to imply what kind of configuration we want
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi show_vlan_config</c>
 *
 * Unless an error occurs, the output will be a table of VLAN configuration
 */
extern int qcsapi_wifi_show_vlan_config(const char *ifname, struct qcsapi_data_2Kbytes *vcfg,
					const char *flag);

/**
 * \brief Enable and disable VLAN promiscuous mode.
 *
 * If VLAN promiscuous mode is enabled, all VLAN tagged packets will be sent to the Linux protocol
 * stack.
 *
 * \note \aponly
 *
 * \param enable set to 1 to enable VLAN promiscuous mode, otherwise set to 0.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_vlan_promisc \<enable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_vlan_promisc(int enable);

/**
 * \brief Configure VLAN loop checking.
 *
 * Generate an event when packets for the same MAC address
 * are detected on different VLANs.
 *
 * \param type
 * - 0 - disable (no loop checking)
 * - 1 - enable (check MAC address)
 * - 3 - enable (check MAC address and port)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vlan_loop_detect \<type\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_set_vlan_loop_detect(const uint32_t type);

/**
 * \brief Get configuration of VLAN loop checking.
 *
 * Get VLAN loop checking configuration.
 *
 * \param type \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vlan_loop_detect</c>
 *
 * Unless an error occurs, the output will be the current configuration.
 */
extern int qcsapi_get_vlan_loop_detect(uint32_t *type);

/** @} */

/**
 * @addtogroup WPSAPIs
 * @{
 */

/**
 * @brief WPS registrar report button press
 *
 * This API starts a WPS session on the Registrar (AP) by pressing the (virtual) WPS Push Button.
 *
 * Under the WPS standard, a WPS session can be started by pressing a virtual button; i.e. by
 * entering a command.
 *
 * A side effect of this API call is that a WPS session will be started.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_report_button_press \<WiFi interface\></c>
 *
 * <c>call_qcsapi registrar_report_pbc \<WiFi interface\></c>
 *
 * This API has 2 scripting interface synonyms.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_registrar_report_button_press(const char *ifname);

/**
 * @brief Report a PIN event on the registrar.
 *
 * This API starts a WPS session on the registrar (AP) by reporting a PIN event.
 *
 * Under the WPS standard, a WPS session can be started by entering a PIN.
 *
 * The PIN is a sequence of either 4 or 8 digits. If the proposed PIN has a length different from 4
 * or 8 characters, or if any of the characters are not digits, the API will return an error code of
 * Invalid Value (-EINVAL).
 *
 * \note The 8 digit PIN (which has a checksum) does not check the validity of the checksum digit.
 *
 * \param ifname \wifi0
 * \param wps_pin the NULL terminated PIN - either 4 or 8 decimal numbers.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_report_pin \<WiFi interface\> \<PIN\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_registrar_report_pin(const char *ifname, const char *wps_pin);

/**
 * \brief Get the WPS Access Control list
 *
 * Get the WPS Access Control list.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param pp_devname comma-separated list of device IDs allowed or denied to receive credentials via
 *	WPS
 *
 * \return \zero_or_negative
 *
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_get_pp_devname \[blacklist\] \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string of allowed Device IDs.
 */
extern int qcsapi_wps_registrar_get_pp_devname(const char *ifname, int blacklist,
					string_128 pp_devname);

/**
 * @brief Set WPS Access Control Device ID List
 *
 * Set the list of Device IDs that are allowed or denied to receive WPS credentials from the AP.
 *
 * \note \aponly
 *
 * The Device IDs are a comma separated list 1 to 256 characters in length with commas as delimiters
 *
 * \param ifname \wifi0
 * \param update_blacklist flag to indicate whether update white-list or black-list
 * \param pp_devname comma-separated list of device IDs allowed or denied to receive credentials via
 *	WPS.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_set_pp_devname \<WiFi interface\> \[blacklist\]
 *	\<list of device IDs\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_registrar_set_pp_devname(const char *ifname, int update_blacklist,
					const string_256 pp_devname);

/**
 * @brief Report a WPS PBC event on the enrollee.
 *
 * This API starts a WPS session on the Enrollee (STA) by pressing the (virtual) WPS Push Button.
 *
 * Under the WPS standard, a WPS session can be started by pressing a virtual button; i.e. by
 * entering a command.
 *
 * The bssid parameter is present for future expansion and should be set to all 0s (zeros).
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enrollee_report_button_press \<WiFi interface\></c>
 * <c>call_qcsapi enrollee_report_pbc \<WiFi interface\></c>
 *
 * This API has 2 scripting interface synonyms. The bssid parameter is not required and will default
 * to all zeros.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_enrollee_report_button_press(const char *ifname, const qcsapi_mac_addr bssid);

/**
 * @brief Report a PIN event on the enrollee.
 *
 * This API starts a WPS session on the enrollee (STA) by reporting a PIN event.
 *
 * Under the WPS standard, a WPS session can be started by entering a PIN.
 *
 * The PIN is a sequence of either 4 or 8 digits. If the proposed PIN has a length different from 4
 * or 8 characters, or if any of the characters are not digits, the API will return an error code of
 * Invalid Value (-EINVAL).
 *
 * \note The 8 digit PIN (which has a checksum) does not check the validity of the checksum digit.
 *
 * \param ifname \wifi0
 * \param bssid the BSSID to report the PIN evens for.
 * \param wps_pin the NULL terminated PIN - either 4 or 8 decimal numbers.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enrollee_report_pin \<WiFi interface\> \<PIN\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_enrollee_report_pin(const char *ifname, const qcsapi_mac_addr bssid,
					const char *wps_pin);

/**
 * @brief Generate a WPS PIN on the enrollee.
 *
 * This API starts a WPS session on the enrollee (STA) by generating a PIN and then reporting
 * that newly generated PIN to any suitably configured and available registrars. The generated PIN
 * will have 8 digits.
 *
 * \param ifname \wifi0
 * \param bssid reserved - set to all zeros.
 * \param wps_pin return parameter to contain the WPS PIN (8 digits, so the string should be at
 *	least 9 bytes long).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enrollee_generate_pin \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string of 8 digits - the newly generated PIN
 *
 * The bssid parameter is not required and will default to all zeros.
 */
extern int qcsapi_wps_enrollee_generate_pin(const char *ifname, const qcsapi_mac_addr bssid,
					char *wps_pin);

/**
 * \brief Get the AP PIN used for WPS PIN operation.
 *
 * This API call is used to get the AP PIN associated with the WPS PIN
 * function. The PIN is either 4 digits or 8 digits long.
 *
 * \note \aponly
 *
 * \param ifname each interface in MBSS
 * \param wps_pin return parameter for containing the NULL terminated string.
 * \param force_regenerate whether to force the AP daemon (hostapd) to regenerate
 * the WPS PIN - a random PIN - for this call.
 *
 * \note a side effect of this call is if there is no WPS PIN currently set for
 * the device, a random PIN will be generated.
 *
 * \return \positive_or_negative
 * If the call succeeded, the wps_pin parameter will be filled with the NULL terminated string
 * containing the PIN.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_ap_pin \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string of 8 digits - the PIN on the AP.
 */
extern int qcsapi_wps_get_ap_pin(const char *ifname, char *wps_pin, int force_regenerate);

/**
 * \brief set the AP PIN used for WPS PIN operation .
 *
 * This API call is used to set the AP PIN associated with the WPS PIN
 * function. The PIN is either 4 digits or 8 digits long.
 *
 * \note \aponly
 *
 * \param ifname each interface in MBSS
 * \param wps_pin return parameter for containing the NULL terminated string.
 *
 * \return \positive_or_negative
 * \callqcsapi
 *
 * <c>call_qcsapi set_wps_ap_pin \<WiFi interface\> \<wps_pin\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_ap_pin(const char *ifname, const char *wps_pin);

/**
 * \brief save ap PIN to configure file
 *
 * This API call is used to save PIN to configure file
 *
 * \note \aponly
 *
 * \param ifname each interface in MBSS
 *
 * \return \positive_or_negative
 * \callqcsapi
 *
 * <c>call_qcsapi save_wps_ap_pin \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_save_ap_pin(const char *ifname);

/**
 * \brief enable/disable ap pin function
 *
 * This API call is used to enable/disable external registrar
 * configure this AP when wps state is not configured
 *
 * \note \aponly
 *
 * \param ifname each interface in MBSS
 * \param enable
 *
 * \return \positive_or_negative
 * \callqcsapi
 *
 * <c>call_qcsapi enable_wps_ap_pin \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_enable_ap_pin(const char *ifname, int enable);

/**
 * @brief Generate a new PIN randomly
 *
 * This API is used to generate a new PIN randomly on a STA.
 * This API won't start WPS session.
 *
 * \param ifname \wifi0
 * \param wps_pin return parameter for containing the NULL terminated string.
 *
 * \return \positive_or_negative
 * If the call succeeded, the wps_pin parameter will be filled with the NULL terminated string
 * containing the PIN.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_sta_pin \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string of 8 digits.
 */
extern int qcsapi_wps_get_sta_pin(const char *ifname, char *wps_pin);

/**
 * @brief Get the state of the current WPS session.
 *
 * Get the current WPS state, reported as a string in the format "%d (%s)", that is,
 * an integer followed by a short descriptive string in parentheses.
 * This API works for either an enrollee or a registrar; or stated differently, it works on both an
 * AP and a STA.
 *
 * \note WPS state is not interface-specific. The same state will be reported for all radios and
 * interfaces.
 *
 * Possible WPS states are:
 *
 * \li <c>0(WPS_INITIAL)</c> - Initial WPS state.
 * \li <c>1(WPS_START)</c> - WPS transaction has started.
 * \li <c>2(WPS_SUCCESS)</c> - WPS transaction succeeded and the device is in association with its
 *	partner.
 * \li <c>3(WPS_ERROR)</c> - WPS transaction ended with an error.
 * \li <c>4(WPS_TIMEOUT)</c> - WPS transaction timed out.
 * \li <c>5(WPS_OVERLAP)</c> - WPS overlap is detected.
 * \li <c>6(WPS_M2_SEND)</c> - WPS is sending M2 frame.
 * \li <c>7(WPS_M8_SEND)</c> - WPS is sending M8 frame.
 * \li <c>8(WPS_STA_CANCEL)</c> - WPS is canceled by STA.
 * \li <c>9(WPS_STA_PIN_ERR)</c> - WPS fail for wrong pin from STA.
 * \li <c>10(WPS_AP_PIN_SUC)</c> - WPS AP pin success.
 * \li <c>11(WPS_AP_PIN_ERR)</c> - WPS AP pin fail.
 *
 * \param ifname \wifi0
 * \param wps_state return parameter for storing the informative WPS state string.
 * \param max_len the length of the wps_state string passed in.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_state \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be one of the WPS State strings listed in the description
 * of the API itself.
 */
extern int qcsapi_wps_get_state(const char *ifname, char *wps_state,
					const qcsapi_unsigned_int max_len);

/**
 * \brief Get the WPS configured state for the given interface.
 *
 * This API call is used to find the WPS configured state - either configured or
 * not configured.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param wps_state return parameter to store the WPS state (configured or
 * not configured).
 * \param max_len the size of the input buffer (wps_state).
 *
 * \return \positive_or_negative
 * If the call succeeded, the wps_state parameter will be filled with the NULL terminated string
 * 'configured' or 'not configured'.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_configured_state \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string <c>configured</c> or <c>not configured</c>.
 */
extern int qcsapi_wps_get_configured_state(const char *ifname, char *wps_state,
					const qcsapi_unsigned_int max_len);


/**
 * \brief Get the WPS runtime state for the given interface.
 *
 * This API call is used to find the WPS runtime state, disabled, not configured or
 * configuired
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param state return parameter to store the WPS state (disabled, configured or
 * not configured).
 * \param max_len the size of the input buffer (state).
 *
 * \return \positive_or_negative
 * If the call succeeded, the wps_state parameter will be filled with the NULL terminated string
 * 'disable', 'configured' or 'not configured'.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_runtime_state \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string <c>disabled</c>,
 *	<c>configured</c> or <c>not configured</c>.
 */
extern int qcsapi_wps_get_runtime_state(const char *ifname, char *state, int max_len);

/**
 * \brief Set the WPS configured state for the given interface.
 *
 * This API call is used to set the WPS state to configured or unconfigured.
 *
 * \note \aponly
 * \note If Hotspot 2.0 is enabled then WPS configuration will not take any effect.
 *
 * \param ifname \wifi0
 * \param state either 0 (disabled), 1 (not configured) or 2 (configured).
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wps_configured_state \<WiFi interface\> {0 | 1 | 2}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_configured_state(const char *ifname, const qcsapi_unsigned_int state);

/**
 * @brief Check WPS configuration
 *
 * Check that all required WPS fields are configured. Any WPS
 * parameters not configured for the primary BSS will be added.
 *
 * \note This API normally runs automatically at boot time, but can be run at any time.
 *
 * \param ifname \wifi0only
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi check_wps_config \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_check_config(const char *ifname);

/**
 * @brief Get a WPS parameter
 *
 * This API returns the value of a WPS Parameter.
 *
 * \param ifname \wifi0
 * \param wps_type the WPS parameter. See the definition of the enum <c>qcsapi_wps_param_type</c>.
 * \param wps_str Address of the string to receive the parameter's value.
 * \param max_len Maximum number of characters that can be written to the parameter <c>wps_str</c>
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_param \<WiFi interface\> \<WPS parameter name\></c>
 *
 * <c>WPS parameter name</c> is a string containing one of the enum names from \ref
 * qcsapi_wps_param_type.
 *
 * \note this API can be used both on AP and STA, except for parameter name is
 * <c>ap_setup_locked</c>, <c>third_party_band</c> and <c>force_broadcast_uuid</c>.
 *
 * Unless an error occurs, the output will be the value of the selected WPS parameter.
 *
 * \note last_config_error, registrar_number, and registrar_established are not supported currently.
 */
extern int qcsapi_wps_get_param(const char *ifname, qcsapi_wps_param_type wps_type, char *wps_str,
					const qcsapi_unsigned_int max_len);

/**
 * @brief set wps walk time value from 120s to 600s
 *
 * This API call is used to set the wps walk time
 *
 * \note this API can be called both on an AP device or a STA device.
 *
 * \param ifname \wifi0
 * \param value walk time value
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_set_timeout \<WiFi interface\> \<timeout value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_timeout(const char *ifname, const int value);

/**
 * @brief set wps_on_hidden_ssid enabled or disabled
 *
 * This API call is used to enable or disable the feature wps_on_hidden_ssid
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_on_hidden_ssid \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_on_hidden_ssid(const char *ifname, const int value);

/**
 * @brief get wps_on_hidden_ssid status
 *
 * This API call is used to check status of wps_on_hidden_ssid
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return "on", "off" or "FAIL"
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_on_hidden_ssid_status \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_on_hidden_ssid_status(const char *ifname, char *state, int max_len);

/**
 * @brief enable or disable wps upnp module
 *
 * This API call is used to enable or disable wps upnp module
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_upnp_enable \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_upnp_enable(const char *ifname, const int value);

/**
 * @brief get upnp status
 *
 * This API call is used to get upnp status
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param reply \valuebuf
 * \param reply_len reply buffer length
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_upnp_status \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_upnp_status(const char *ifname, char *reply, int reply_len);

/**
 * @brief Allow or forbid the detection of WPS PBC overlap
 *
 * This API call is used to allow/forbid the detection of PBC overlap.
 *
 * \note this API can be called both on an AP device or a STA device.
 *
 * \param ifname \wifi0
 * \param allow 1 (allow) or 0 (forbid)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi allow_pbc_overlap \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_allow_pbc_overlap(const char *ifname, const qcsapi_unsigned_int allow);
/**
 * @brief get status if PBC overlap is allowed on AP or STA.
 *
 * This API returns the status if PBC overlap is allowed on AP or STA.
 *
 * \note this API can be called both on an AP device or a STA device.
 *
 * \param ifname \wifi0
 * \param status the return value if allow PBC overlap.
 *
 * \return \positive_or_negative
 * If the call succeeded, the status of allowing PBC overlap (0/1) will be returned.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_allow_pbc_overlap_status \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string '<c>1</c>' or '<c>0</c>'
 */
extern int qcsapi_wps_get_allow_pbc_overlap_status(const char *ifname, int *status);

/** \brief Enable/Disable the WPS Pair Protection for the given interface.
 *
 * This API call is used to Enable/Disable WPS Pair Protection.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param ctrl_state 0 (disabled) or 1 (enabled)
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wps_access_control \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_access_control(const char *ifname, uint32_t ctrl_state);

/** \brief Get the WPS Pair Protection state for the given interface.
 *
 * This API call is used to get the WPS Pair Protection state - either enabled or disabled.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param ctrl_state \valuebuf
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_access_control \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string '<c>1</c>' or '<c>0</c>'.
 */
extern int qcsapi_wps_get_access_control(const char *ifname, uint32_t *ctrl_state);

/**
 * @brief Set a WPS parameter
 *
 * This API is called to set a WPS Parameter.
 *
 * \note \aponly
 *
 * \param ifname \wifi_all
 * \param wps_type the WPS parameter. See the definition of the enum <c>qcsapi_wps_param_type</c>.
 * \param param_value Address of the string to set the parameter's value.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wps_param \<WiFi interface\> \<WPS parameter name\>
 *	\<WPS parameter value\></c>
 *
 * <c>WPS parameter name</c> is a string containing one of the enum names from \ref
 * qcsapi_wps_param_type.
 *
 * Parameter <c>ap_setup_locked</c> can only be set or reset when the WPS parameter
 * <c>ap_pin_fail_method</c> is set to auto_lockdown.
 *
 * The parameter value of uuid has format xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx. For example:
 *
 * <c>cdcb13e6-baa5-5f43-807f-4b4c28223a68</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_param(const char *ifname, const qcsapi_wps_param_type param_type,
					const char *param_value);

/**
 * @brief Cancel the ongoing wps procedure if any.
 *
 * This API equivalent to "wpa_cli wps_cancel". It will cancel ongoing wps procedure,
 * and do nothing if there are no wps procedure undergoing.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wps_cancel \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_cancel(const char *ifname);

/**
 * @brief Add/remove PBC methods in SRCM.
 *
 * This API is used to add or remove PBC methods in SRCM (selected registrar config methods)
 * attribute in WSC IE.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param enabled 1 to add and 0 to remove PBC methods in SRCM attribute in WSC IE.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_wps_pbc_in_srcm \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wps_set_pbc_in_srcm(const char *ifname, const qcsapi_unsigned_int enabled);

/**
 * @brief Get currently setting of PBC methods in SRCM attribute.
 *
 * This API is used to get currently setting of PBC methods in SRCM attribute.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_enabled Where to store the result return.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_pbc_in_srcm \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string <c>0</c> or <c>1</c>.
 */

extern int qcsapi_wps_get_pbc_in_srcm(const char *ifname, qcsapi_unsigned_int *p_enabled);

/**
 * @brief Set the default BSS for WPS Push button
 *
 * Set the default BSS for WPS Push button if there's more than one BSS such like
 * MBSS mode default BSS for WPS PBC is primary interface after power-up.
 *
 * \note \aponly
 *
 * \param ifname \wifi0 Use "null" to remove the default setting.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_set_default_pbc_bss \<WiFi interface\> | null</c>
 *
 * \call_qcsapi_string_complete
 */

extern	int qcsapi_registrar_set_default_pbc_bss(const char *ifname);

/**
 * @brief Get the default BSS for WPS Push Button
 *
 * Get the default BSS for WPS Push button.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_registrar_get_default_pbc_bss.
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_get_default_pbc_bss</c>
 *
 * Unless an error occurs, the output will be an interface name or "null" if default PBC WPS is
 * not configured.
 */

extern int qcsapi_registrar_get_default_pbc_bss(char *default_bss, int len);

/**
 * @brief get default bss on specified radio device for WPS Push button
 *
 * This API is used to get default bss on specified radio for WPS Push button
 * default bss for WPS PBC is primary interface on specified radio after powered up
 *
 * \note \aponly
 *
 * \param radio_id \radio_id
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi registrar_get_default_pbc_bss \<radio\></c>
 *
 * Unless an error occurs, the output will be an interface name or "null" if default PBC WPS is
 * not configured.
 */

extern int qcsapi_radio_registrar_get_default_pbc_bss(qcsapi_unsigned_int radio_id,
		char *default_bss, int len);

/** @} */

/**
 * @addtogroup LEDGPIOAPIs
 * @{
 */

/**
 * @brief Set GPIO configuration
 *
 * \warning <b>This API can potentially damage the chip, please treat it with respect and read
 * through the following documentation before using the API.</b>
 *
 * Configures a GPIO pin for input (1), input/output (2), or disables further use of the GPIO pin
 * (0), as specified by the new GPIO config parameter (see \ref qcsapi_gpio_config).
 *
 * GPIO pin values run from 0 to 31.
 *
 * \note This API is only available in calibration mode (see @ref mysection4_1_5 "Production mode vs
 * calibration mode").
 *
 * \param gpio_pin the GPIO to change.
 * \param new_gpio_config the new state of the PIN.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_gpio_config \<GPIO pin number\> \<configuration\></c>
 *
 * where \<GPIO pin number\> is a GPIO pin number, an integer in the range 0 to 31, and
 * \<configuration\> is either 0, 1 or 2.
 *
 * See above for the meaning of 0, 1 and 2 as a GPIO pin configuration.
 *
 * \call_qcsapi_string_complete
 *
 * \warning <b>Power should not be turned off to the WiFi device when calling the set GPIO config
 * API or immediately afterwards. Failure to follow this restriction can cause the flash memory on
 * the board to become corrupted. If power needs to be turned off to the WiFi device when working
 * with this API, enter the <c>halt</c> command first and wait for the device to shut down.
 * This API should only be called when initially configuring the board.</b>
 *
 * \warning <b>Be aware that configuring a GPIO pin for output that either not present or wired for
 * input can leave the board or chip open to being damaged should a set API attempt to change the
 * GPIO pin setting to a state not supported by the hardware.</b>
 *
 *
 * \sa qcsapi_gpio_config
 */
extern int qcsapi_gpio_set_config(const uint8_t gpio_pin, const qcsapi_gpio_config new_gpio_config);

/**
 * @brief Get GPIO configuration
 *
 * Get the current configuration of a GPIO pin, either input (1), output (2), or disabled (0).
 *
 * GPIO pin values are the same as in the set GPIO config API.
 *
 * \param gpio_pin the GPIO to read.
 * \param p_gpio_config return parameter to store the state of the PIN.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_gpio_config \<GPIO pin number\></c>
 *
 * where \<GPIO pin number\> is a GPIO pin number, an integer in the range 0 to 31.
 */
extern int qcsapi_gpio_get_config(const uint8_t gpio_pin, qcsapi_gpio_config *p_gpio_config);

/**
 * @brief Get LED state.
 *
 * Get the current level for an LED/GPIO pin, either HIGH (1) or LOW (0).
 *
 * \note The GPIO pin must have been previously configured for input or output thru
 * qcsapi_gpio_set_config.
 *
 * \param led_ident the GPIO pin number.
 * \param p_led_setting
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_LED \<LED / GPIO pin number\></c>
 *
 * where \<LED / GPIO pin number\> is an LED / GPIO pin number, an integer in the range 0 to 31.
 *
 * Unless an error occurs, the output will be either 0 (LOW) or 1 (HIGH).
 */
extern int qcsapi_led_get(const uint8_t led_ident, uint8_t *p_led_setting);

/**
 * @brief Set LED state.
 *
 * Set the current level for an LED/GPIO pin, either HIGH (1) or LOW (0).
 *
 * The LED identity is the GPIO pin number.
 *
 * \note The GPIO pin must have been previously configured for output thru the set GPIO config API
 * (qcsapi_gpio_set_config).
 *
 * \warning <b>Be aware that configuring an incorrect GPIO pin for input/output and then setting the
 * level for that invalid GPIO pin can damage the board. Consult the documentation or schematics for
 * the board for details on the GPIO pin configuration.</b>
 *
 * \param led_ident the GPIO corresponding to the LED to change.
 * \param new_led_setting the new state of the LED.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_LED \<LED / GPIO pin number\> {0 | 1}</c>
 *
 * where <LED / GPIO pin number> is an LED / GPIO pin number, an integer in the range 0 to 31.
 *
 * \call_qcsapi_string_complete
 *
 * \note Most GPIO pins connect to an LED or other item of hardware that software controls directly
 * using the get and set LED APIs. However, the WPS Push Button and the Reset Device Push Button
 * require additional programming,
 * since the end-user can press these push buttons to start a WPS association process or reset the
 * WiFi device. The software thus needs to be "armed" to respond to these events.
 * Because the way the system is expect to respond to a WPS push button press is quite different
 * from the way it should respond to a Reset Device button press, separate APIs are provided for
 * each.
 */
extern int qcsapi_led_set(const uint8_t led_ident, const uint8_t new_led_setting);

/**
 * @brief Enable pulse wide modulation for LED GPIO pin.
 *
 * Enable pulse wide modulation for LED GPIO pin to control LED brightness.
 *
 * The LED identity is the GPIO pin number.
 *
 * \param led_ident the GPIO corresponding to the LED to change.
 * \param onoff 1 to enable PWM, 0 - to disable.
 * \param high_count 'on' duration in each cycle, integer in range 1 - 256
 * \param low_count 'off' duration in each cycle, integer in range 1 - 256
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_LED_PWM \<LED / GPIO pin number\> {0 | 1} \<high_count\> \<low_count\></c>
 *
 * where <LED / GPIO pin number> is an LED / GPIO pin number, an integer in the range 0 to 31,
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_led_pwm_enable(const uint8_t led_ident, const uint8_t onoff,
			const qcsapi_unsigned_int high_count, const qcsapi_unsigned_int low_count);

/**
 * @brief Set LED brightness level
 *
 * Set LED brightness level. Level can be beetween 1 and 10 where 10 is a maximum brightness and 1
 * is a lowest level before turning off the LED. The LED identity is the GPIO pin number.
 *
 * \param led_ident the GPIO corresponding to the LED to change.
 * \param level brightness level in range 1 - 10
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_LED_brightness \<LED / GPIO pin number\> \<level\></c>
 *
 * where <LED / GPIO pin number> is an LED / GPIO pin number, an integer in the range 0 to 31,
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_led_brightness(const uint8_t led_ident, const qcsapi_unsigned_int level);

/**
 * This typedef is used to force a function prototype for reset button function callback.
 *
 * The reset_device_pin passes in the GPIO pin being monitored, and the current_level is either 1 or
 * 0.
 *
 * \sa qcsapi_gpio_monitor_reset_device
 */
typedef void	(*reset_device_callback)(uint8_t reset_device_pin, uint8_t current_level);

/**
 * @brief Monitor the reset device GPIO pin.
 *
 * This API lets an application identify the GPIO pin connected to the reset device push button,
 * and then monitors this push button.
 *
 * \param reset_device_pin the GPIO pin that is connected to the push button. This pin must be
 * configured for input.
 * \param active_logic identifies whether the active state of the pin is high or low, and should be
 * 1 or 0 respectively.
 * \param blocking_flag specifies whether the API should block the process until the button is
 * pressed. Currently this must be set to 1 - ie the API only supports blocking operation.
 * \param respond_reset_device is the address of a callback entry point, with signature as per the
 * \ref reset_device_callback.
 *
 * When called, this API (after completing error checking) periodically checks the state of
 * reset_device_pin. When this pin goes active, as specified by active_logic, it calls the callback
 * entry point identified by reset_device_callback. Notice the entry point is responsible for
 * handling any response to pressing the reset device push button.
 *
 * A sample requirement for how this API is used is:
 *
 * \li If the Reset Device Push Button is pressed for between 1 second and 5 seconds, the WiFi
 *	device reboots.
 * \li If the Reset Device Push Button is pressed for more than 5 seconds, the factory default
 *	settings are restored and the device then reboots.
 *
 * Again, the reset device callback, programming not part of the QCSAPI,
 * is responsible for handling the response to pressing this push button.
 *
 * \note This API cannot be called from within <c>call_qcsapi</c>
 */
extern int qcsapi_gpio_monitor_reset_device(const uint8_t reset_device_pin,
					const uint8_t active_logic, const int blocking_flag,
					reset_device_callback respond_reset_device);

/**
 * @brief Enable the WPS GPIO push button.
 *
 * This API enables the WPS push button.
 *
 * Unlike the reset device push button, the expected response when the WPS push button is pressed is
 * predefined. For this reason no callback programming is required.
 *
 * \param wps_push_button the GPIO used for WPS push button operation.
 * \param active_logic identifies whether the active state of the pin is high or low, and should be
 * 1 or 0 respectively. \param use_interrupt_flag if set to 0, selects polling operation, if 1,
 * selects interrupt operation. If interrupt mode is selected, the active logic must be 1.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_wps_push_button \<GPIO pin\> {0 | 1}</c>
 *
 * where \<GPIO pin\> is the number of the GPIO pin that controls the WPS push button.
 * The parameter that follows selects active logic, either LOW (0) or HIGH (1).
 *
 * To enable the WPS push button in interrupt mode, enter:
 * <c>call_qcsapi enable_wps_push_button \<GPIO pin\> 0 intr</c>
 */
extern int qcsapi_gpio_enable_wps_push_button(const uint8_t wps_push_button,
					const uint8_t active_logic,
					const uint8_t use_interrupt_flag);
/** @} */

/**
 * @addtogroup PerAssocAPIs
 * @{
 */

/**
 * @brief Get the number of STAs associated.
 *
 * Gets the number of stations currently associated with the access point.
 * As associations are dynamic, this count can change at any time.
 *
 * \note This API is used on both AP and STA. On a STA, it is used to
 * indicate whether it is associated with an AP.
 *
 * \param ifname \wifi0
 * \param p_association_count return parameter to store the count of STAs associated with the AP.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_count_assoc \<WiFi interface\></c>
 *
 * <c>call_qcsapi get_count_associations \<WiFi interface\></c>
 *
 * <c>call_qcsapi get_association_count \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the count of stations currently associated to this AP.
 *
 * \sa qcsapi_wifi_get_BSSID
 */
extern int qcsapi_wifi_get_count_associations(const char *ifname,
					qcsapi_unsigned_int *p_association_count);

/**
 * @brief Get the associated device MAC addresses.
 *
 * Gets the MAC address of a device (STA or WDS peer) currently associated with the AP.
 * Second parameter selects the association index, with a range from 0 to the association count - 1.
 * An index out of range causes the API to fail with the error set to Out of Range (-ERANGE).
 * Use qcsapi_wifi_get_count_associations to determine the current association count.
 *
 * As associations are dynamic, the count of associations can change at any time.
 * An application should never assume that a value previously returned from
 * qcsapi_wifi_get_count_associations remains valid. Applications should always be prepared for a
 * return value of -ERANGE from this API, even if it just verified the number of current
 * associations.
 *
 * This API only works on an AP; for a STA, use the get BSSID API (\ref qcsapi_wifi_get_BSSID) to
 * get the MAC address of the associated AP.
 *
 * \note \aponly
 *
 * \param ifname \wifi_wds
 * \param device_index the index of the device MAC address to return.
 * \param device_mac_addr the MAC address of the device at index 'device_index'.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_associated_device_mac_addr \<WiFi interface\> \<association index\></c>
 *
 * The output will be the MAC address of the associated device, or an error message.
 *
 * \sa qcsapi_wifi_get_BSSID
 */
extern int qcsapi_wifi_get_associated_device_mac_addr(const char *ifname,
						const qcsapi_unsigned_int device_index,
						qcsapi_mac_addr device_mac_addr);

/**
 * @brief Get the associated device IP addresses.
 *
 * Get the IP address of a device (STA or WDS peer) currently associated with the AP.
 *
 * \note \aponly
 *
 * Second parameter selects the association index, with a range from 0 to the association count - 1.
 * An index out of range causes the API to fail with the error set to Out of Range (-ERANGE).
 * Use qcsapi_wifi_get_count_associations to determine the current association count.
 * As associations are dynamic, the count of associations can change at any time.
 * An application should never assume that a value previously returned from
 * qcsapi_wifi_get_count_associations remains valid. Applications should always be prepared for a
 * return value of -ERANGE from this API, even if it just verified the number of current
 * associations.
 *
 * \param ifname \wifi_wds
 * \param device_index association index of STA, or 0 for a WDS peer
 * \param ip_addr the IP address of the device at index 'device_index'.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_associated_device_ip_addr \<WiFi interface\> \<association index\></c>
 *
 * The output will be the IP address of the associated device, or an error message.
 */
extern int qcsapi_wifi_get_associated_device_ip_addr(const char *ifname,
				const qcsapi_unsigned_int device_index, unsigned int *ip_addr);

/**
 * @brief Get the link quality per association index.
 *
 * Returns the link quality as the current TX PHY rate in megabits per second (MBPS).
 *
 * \note The device must have the autorate fallback option enabled.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA link quality to be
 * read. On the STA, this should be 0.
 * \param p_link_quality the link quality for the given index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_link_quality \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current link quality for the association, in terms of the current TX PHY
 * rate in Mbps.
 */
extern int qcsapi_wifi_get_link_quality(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_link_quality);

/**
 * @brief Get maximum link quality for all stations
 *
 * Returns link quality as the current TX PHY rate in megabits per second (Mbps).
 * The function is similar to <c>qcsapi_wifi_get_link_quality</c> but returns maximum link quality
 * for all current associations.
 *
 * \param ifname \wifi0
 * \param p_max_quality the maximum of the link quality for all current associations.
 *
 * \return \zero_or_negative
 *
 */
extern int qcsapi_wifi_get_link_quality_max(const char *ifname, qcsapi_unsigned_int *p_max_quality);

/**
 * @brief Get RX bytes per association index.
 *
 * Returns the current number of bytes received on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RX bytes is to be
 *	read. On the STA, this should be 0.
 * \param p_rx_bytes return parameter to contain the number of bytes received on this association
 *	index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rx_bytes \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of bytes received on that association.
 */
extern int qcsapi_wifi_get_rx_bytes_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, u_int64_t *p_rx_bytes);

/**
 * @brief Get TX bytes per association index.
 *
 * Returns the current number of bytes transmitted on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA TX bytes is to be
 *	read. On the STA, this should be 0.
 * \param p_tx_bytes return parameter to contain the number of bytes transmitted on this association
 *	index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_bytes \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of bytes transmitted on that association.
 */
extern int qcsapi_wifi_get_tx_bytes_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, u_int64_t *p_tx_bytes);

/**
 * @brief Get RX Packets by association.
 *
 * Returns the current number of packets received on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA, this
 *	should be 0.
 * \param p_rx_packets return parameter to contain the number of packets received on this
 *	association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rx_packets \<WiFi interface\> \<association index\></c>
 *
 * <c>call_qcsapi get_assoc_rx_packets \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of packets received on that association.
 */
extern int qcsapi_wifi_get_rx_packets_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_rx_packets);

/**
 * @brief Get TX Packets by association.
 *
 * Returns the current number of packets transmitted on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA, this
 *	should be 0.
 * \param p_tx_packets return parameter to contain the number of packets transmitted on this
 *	association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_packets \<WiFi interface\> \<association index\></c>
 *
 * <c>call_qcsapi get_assoc_tx_packets \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of packets transmitted on that association.
 */
extern int qcsapi_wifi_get_tx_packets_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_packets);

/**
 * @brief Get TX Packets Errors by association.
 *
 * Returns the current number of packets that failed to be transmitted on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA, this
 *	should be 0.
 * \param p_tx_err_packets return parameter to contain the number of packets which failed
 *	transmission on this association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_err_packets \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of packets that failed to be transmitted on the
 * association.
 */
extern int qcsapi_wifi_get_tx_err_packets_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_err_packets);

/**
 * @brief Get RSSI per association index.
 *
 * Returns the current Received Signal Strength Indication (RSSI) in the range [0, 68].
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RSSID is to be read.
 *	On the STA, this should be 0.
 * \param p_rssi return parameter to contain the RSSI on this association index, in the range
 *	[0 - 68].
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rssi \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current RSSI for the association in the range [0 - 68].
 */
extern int qcsapi_wifi_get_rssi_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_rssi);

/**
 * @brief Get RSSI in dBm per association index.
 *
 * Returns the current Received Signal Strength Indication (RSSI) in dBm.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read. On the
 *	STA, this should be 0.
 * \param p_rssi return parameter to contain the RSSI on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rssi_dbm \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current RSSI in dBm for the association.
 */
extern int qcsapi_wifi_get_rssi_in_dbm_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, int *p_rssi);

/**
 * @brief Get the measuring RSSI in dBm per association index.
 *
 * Returns the current measuring Received Signal Strength Indication (RSSI) in dBm if measurement
 * period is set.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read. On the
 *	STA, this should be 0.
 * \param p_rssi return parameter to contain the measuring RSSI on this association index.
 * \param p_seq_num return parameter to contain the sequence number for this returned measuring
 *	RSSI.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_meas_rssi_dbm \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current measuring RSSI in dBm for the association and the sequence number
 * for this RSSI. Before calling this call_qcsapi, you need to set the measurement period with
 * call_qcsapi set_rssi_meas_period
 */
extern int qcsapi_wifi_get_meas_rssi_in_dbm_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, int *p_rssi,
			uint32_t *p_seq_num);

/**
 * \brief Set the period for RSSI measurement
 *
 * This API sets the period for measuring the Received Signal Strength Indication (RSSI)
 *
 * \param ifname \wifi0only
 * \param period unit: second. 0 is to disable RSSI measurement
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_rssi_meas_period wifi0_0 \<period\></c>
 *
 * \call_qcsapi_string_complete
 *
 */
extern int qcsapi_wifi_set_rssi_meas_period(const char *ifname, const qcsapi_unsigned_int period);

/**
 * \brief get the period for RSSI measurement
 *
 * This API gets the period for measuring the Received Signal Strength Indication (RSSI)
 *
 * \param ifname \wifi0only
 * \param p_period return parameter to contain the period for RSSI measurement
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rssi_meas_period wifi0_0</c>
 *
 * The output will be the current period for measuring the RSSI.
 */
extern int qcsapi_wifi_get_rssi_meas_period(const char *ifname, qcsapi_unsigned_int *p_period);

/**
 * \brief Get the associated peer bandwidth (20 vs 40MHz).
 *
 * This API call is used to determine the bandwidth used by the peer STA.
 * The bandwidth is 20 or 40, representing 20MHz or 40MHz.
 *
 * \param ifname \wifi0
 * \param association_index the entry into the association table.
 * \param p_bw return parameter for storing the return value (either
 * 20 or 40)
 *
 * \return >= 0, success, < 0 error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_assoc_bw \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be one of the strings
 * '<c>20</c>' or '<c>40</c>'.
 */
extern int qcsapi_wifi_get_bw_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_bw);

/**
 * @brief Get TX PHY rate by association index.
 *
 * Returns the current TX PHY rate in megabits per second (MBPS)
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_tx_phy_rate return parameter to receive the TX PHY rate on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_phy_rate \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the current TX PHY rate in MBPS.
 */
extern int qcsapi_wifi_get_tx_phy_rate_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_phy_rate);

/**
 * @brief Get RX PHY rate by association index.
 *
 * Returns the current RX PHY rate in megabits per second (MBPS)
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_rx_phy_rate return parameter to receive the RX PHY rate on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rx_phy_rate \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the current RX PHY rate in MBPS.
 */
extern int qcsapi_wifi_get_rx_phy_rate_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_rx_phy_rate);

/**
 * @brief Get TX MCS by association index.
 *
 * Returns the current TX MCS
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_mcs return parameter to receive the TX MCS on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_mcs \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the current TX MCS.
 */
extern int qcsapi_wifi_get_tx_mcs_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_mcs);

/**
 * @brief Get RX MCS by association index.
 *
 * Returns the current RX MCS
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_mcs return parameter to receive the RX MCS on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rx_mcs \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the current RX MCS.
 */
extern int qcsapi_wifi_get_rx_mcs_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_mcs);

/**
 * @brief Get Achievable TX PHY rate by association index.
 *
 * Returns the achievable TX PHY rate in kilobits per second (KBPS)
 *
 * \note The units for this API are kilobits per second. The reported achievable TX PHY rate
 * typically ranges between 54000 and 1733300.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_achievable_tx_phy_rate return parameter to receive the achievable RX PHY rate on this
 *	association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_achievable_tx_phy_rate \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the achievable TX PHY rate in KBPS.
 */
extern int qcsapi_wifi_get_achievable_tx_phy_rate_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_achievable_tx_phy_rate);

/**
 * @brief Get Achievable RX PHY rate by association index.
 *
 * Returns the achievable RX PHY rate in kilobits per second (KBPS)
 *
 * \note The units for this API are kilobits per second. The reported achievable RX PHY rate
 * typically ranges between 54000 and 1733300.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RSSID is to be read.
 *	On the STA, this should be 0.
 * \param p_achievable_rx_phy_rate return parameter to receive the achievable RX PHY rate on this
 *	association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_achievable_rx_phy_rate \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the achievable RX PHY rate in KBPS.
 */
extern int qcsapi_wifi_get_achievable_rx_phy_rate_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_achievable_rx_phy_rate);


/**
 * @brief Get authentication description by association index.
 *
 * Returns the auth algorithm, key management, key protocol and cipher.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA authentication
 *	detail is to be read.
 * \param p_auth_enc return parameter to receive the authentication details. Information is passed
 *	in packed format:
 * Bits 0 - 7: Auth algorithm (0x00 - OPEN, 0x01 - SHARED)
 * Bits 8 - 15: Key management (0x00 - NONE, 0x01 - EAP, 0x02 - PSK, 0x03 - WEP)
 * Bits 16 - 23: Key protocol (0x00 - NONE, 0x01 - WPA, 0x02 - WPA2)
 * Bits 24 - 31: Cipher (0x01 - TKIP, 0x03 - CCMP)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_auth_enc_per_assoc \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the authenification details.
 */
extern int qcsapi_wifi_get_auth_enc_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_auth_enc);

/**
 * \internal
 * \brief Get HT, VHT and HE capabilities by association index.
 *
 * Returns the contents of the HT, VHT and HE information elements for an associated station.
 *
 * \param ifname \wifi0
 * \param association_index the association index of an associated station
 * \param tput_caps buffer to receive the returned data
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tput_caps \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the content of the station's HT, VHT and HE
 * information elements in hex format. LSB to MSB order will be used for output.
 */
extern int qcsapi_wifi_get_tput_caps(const char *ifname,
			const qcsapi_unsigned_int association_index,
			struct ieee8011req_sta_tput_caps *tput_caps);

/**
 * \internal
 * \brief Get connection mode by association index.
 *
 * Returns the connection mode for an associated station.
 *
 * \param ifname \wifi0
 * \param association_index the association index of an associated station
 * \param connection_mode buffer to receive the current mode, which is a value from the
 * <c>ieee80211_wifi_modes</c> enum
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_connection_mode \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be a WiFi mode from the <c>qcsapi_wifi_modes_strings</c>
 * array. E.g. 'a', 'b', 'g', 'na', 'ng' or 'ac'.
 */
extern int qcsapi_wifi_get_connection_mode(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *connection_mode);

/**
 * @brief Get vendor by association index.
 *
 * Returns vendor name of the peer device (if known) for the given association index.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RSSID is to be read.
 *	On the STA, this should be 0.
 * \param p_vendor return vendor name for this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vendor \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the vendor of the client.
 */
extern int qcsapi_wifi_get_vendor_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_vendor);

/**
 * \brief Set the RF TX/RX chains for both 5G and 2.4G
 *
 * This API sets the number of RF TX/RX chains
 *
 * \param tx_chains_5g the number of 5G TX chains.
 * \param rx_chains_5g the number of 5G RX chains.
 * \param tx_chains_2g the number of 2.4G TX chains.
 * \param rx_chains_2g the number of 2.4G RX chains.
 * \param tx_chains_5g_2 the number of 5G_2 TX chains. Used in DCDC mode only.
 *	Must be set to 0 since DCDC mode is not supported.
 * \param rx_chains_5g_2 the number of 5G_2 RX chains. Used in DCDC mode only.
 *	Must be set to 0 since DCDC mode is not supported.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_rf_chains \<tx_chains_5G\> \<rx_chains_5G\> \<tx_chains_2.4G\>
 *	\<rx_chains_2.4G\> [ \<tx_chains_5G_2\> ] [ \<rx_chains_5G_2\> ]</c>
 *
 * \call_qcsapi_string_complete
 *
 * \note System rebooting is required to make the configuration take effect.
 */
extern int qcsapi_set_rf_chains(qcsapi_unsigned_int tx_chains_5g, qcsapi_unsigned_int rx_chains_5g,
			qcsapi_unsigned_int tx_chains_2g, qcsapi_unsigned_int rx_chains_2g,
			qcsapi_unsigned_int tx_chains_5g_2, qcsapi_unsigned_int rx_chains_5g_2);

/**
 * \brief get the RF TX/RX chains for both 5G and 2.4G
 *
 * This API gets the number of RF TX/RX chains
 *
 * \param p_tx_chains_5g return parameter to contain the number of 5G TX chains
 * \param p_rx_chains_5g return parameter to contain the number of 5G RX chains
 * \param p_tx_chains_2g return parameter to contain the number of 2.4G TX chains
 * \param p_rx_chains_2g return parameter to contain the number of 2.4G RX chains
 * \param p_tx_chains_5g_2 return parameter to contain the number of 5G_2 TX chains.
 *	Valid for DCDC mode only.
 * \param p_rx_chains_5g_2 return parameter to contain the number of 5G_2 RX chains.
 *	Valid for DCDC mode only.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rf_chains</c>
 *
 * The output will be the RF TX/RX chains
 */
extern int qcsapi_get_rf_chains(qcsapi_unsigned_int *p_tx_chains_5g,
		qcsapi_unsigned_int *p_rx_chains_5g, qcsapi_unsigned_int *p_tx_chains_2g,
		qcsapi_unsigned_int *p_rx_chains_2g, qcsapi_unsigned_int *p_tx_chains_5g_2,
		qcsapi_unsigned_int *p_rx_chains_5g_2);

/**
 * \brief Set the tx chains
 *
 * This API sets the number of tx chains.
 *
 * \note \deprecate_and_replace \ref qcsapi_set_rf_chains.
 *
 * \param ifname \wifi0only
 * \param tx_chains the number of tx chains.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_tx_chains wifi0_0 \<tx_chains\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_tx_chains(const char *ifname, const qcsapi_unsigned_int tx_chains);

/**
 * \brief get the tx chains
 *
 * This API gets the number of tx chains
 *
 * \param ifname \wifi0only
 * \param p_tx_chains return parameter to contain the number of tx chains
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_chains wifi0_0</c>
 *
 * The output will be the tx chains
 */
extern int qcsapi_wifi_get_tx_chains(const char *ifname, qcsapi_unsigned_int *p_tx_chains);

/**
 * \brief Set the rx chains
 *
 * This API sets the number of rx chains
 *
 * \note \deprecate_and_replace \ref qcsapi_set_rf_chains.
 *
 * \param ifname
 * \param rx_chains the number of rx chains.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_rx_chains wifi0_0 \<rx_chains\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_rx_chains(const char *ifname, const qcsapi_unsigned_int rx_chains);

/**
 * \brief get the rx chains
 *
 * This API gets the number of rx chains
 *
 * \param ifname
 * \param p_rx_chains return parameter to contain the number of rx chains
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rx_chains wifi0_0</c>
 *
 * The output will be the rx chains
 */
extern int qcsapi_wifi_get_rx_chains(const char *ifname, qcsapi_unsigned_int *p_rx_chains);

/**
 * \internal
 * \brief Get max MIMO streams by association index
 *
 * Returns the maximum number of receive and transmit spatial streams allowed to/from an associated
 * station.
 *
 * \param ifname \wifi0
 * \param association_index the association index of an associated station
 * \param p_max_mimo buffer to receive the max MIMO streams
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_max_mimo \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be a string representing max MIMO streams supported by
 * the station in the form "Rx:A Tx:B" where A and B are integers. The output will be "unknown" if
 * the number of streams cannot be determined.
 */
extern int qcsapi_wifi_get_max_mimo(const char *ifname,
		const qcsapi_unsigned_int association_index, string_16 p_max_mimo);


/**
 * @brief Get Signal to Noise Ratio (SNR) by association index.
 *
 * Returns the current SNR for the given association index.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RSSID is to be read.
 *	On the STA, this should be 0.
 * \param p_snr return parameter to receive the SNR on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_snr \<WiFi interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the current SNR.
 */
extern int qcsapi_wifi_get_snr_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, int *p_snr);

/**
 * @brief Get the associated device time per association index.
 *
 * Returns the time in seconds a STA has been associated with an AP.
 * This API can be applied to both station and AP mode.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param time_associated return parameter to contain the time associated on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_time_associated \<WiFi interface\> 0</c>
 *
 * The output will be the time in seconds that the STA has been associated with an AP.
 *
 * For the AP, the final parameter is
 *
 * <c>call_qcsapi get_time_associated \<WiFi interface\> \<association index\></c>
 *
 * The output will be the time in seconds that the STA at association index has been associated with
 * an AP.
 */
extern int qcsapi_wifi_get_time_associated_per_association(const char *ifname,
						const qcsapi_unsigned_int association_index,
						qcsapi_unsigned_int *time_associated);
/**
 * @brief Get a Parameter for a Node
 *
 * Returns the value of the selected parameter for the selected node.
 *
 * \param ifname \wifi0
 * \param node_index \assoc_index
 * \param param_type the parameter type.
 * \param local_remote_flag use local flag to get local parameters, use remote flag to get
 *	parameters from remote associated STA; set to <c>QCSAPI_LOCAL_NODE</c> or
 *	<c>QCSAPI_REMOTE_NODE</c>
 * \param input_param_str address to related request parameters, actual request structure
 *	information please refer to qcsapi_measure_request_param
 * \param report_result address to receive all kinds of results
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_param \<WiFi interface\> \<node index\> \<parameter name\>
 *	\<local remote flag\> \<parameters indicated by type\></c>
 *
 * \param parameter-name parameter names and corresponding enum values
 * are shown in \ref qcsapi_per_assoc_param.
 *
 * The output will be the value for the selected parameter. RSSI_DBM will be in dBm; SNR and RSSI
 * will be in dB.
 */
extern int qcsapi_wifi_get_node_param(const char *ifname, const uint32_t node_index,
			qcsapi_per_assoc_param param_type, int local_remote_flag,
			const string_128 input_param_str,
			qcsapi_measure_report_result *report_result);
/**
 * @brief Get a Counter for a Node
 *
 * Returns the value of the selected counter for the selected node.
 *
 * \param ifname \wifi0
 * \param node_index \assoc_index
 * \param counter_type the counter to select. See \ref <c>qcsapi_counter_type</c>.
 * \param local_remote_flag use local flag to get local counters, use remote flag to get counters
 *	from remote associated STA; set to <c>QCSAPI_LOCAL_NODE</c> or <c>QCSAPI_REMOTE_NODE</c>
 * \param p_value address to receive the value of the parameter. It must address a 64-bit quantity.
 *
 * \note Not all per-node counters are 64 bits wide. Some will roll over when the maximum 32-bit
 * value is reached.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_counter \<WiFi interface\> \<node index\> \<counter type\>
 *	\<local remote flag\></c>
 *
 * where <c>counter name</c> is as defined in the section on the Get Counter API.
 *
 * The output will be the value for the selected counter.
 */
extern int qcsapi_wifi_get_node_counter(const char *ifname, const uint32_t node_index,
			qcsapi_counter_type counter_type, int local_remote_flag, uint64_t *p_value);

/**
 * @brief Get the Statistics (data structure of counters) for a Node
 *
 * Returns a data structure populated with statistics (counters) for a node.
 *
 * \param ifname \wifi0
 * \param node_index \assoc_index
 * \param local_remote_flag use local flag to get local statistics, use remote flag to get
 *	statistics from remote associated STA; set to <c>QCSAPI_LOCAL_NODE</c> or
 *	<c>QCSAPI_REMOTE_NODE</c>
 * \param p_stats address of a <c>struct qcsapi_node_stats</c>
 *
 * \note See the definition of the Node Stats data struct for details on what
 * counters this API will return.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_stats \<WiFi interface\> \<node index\> \<local remote flag\></c>
 *
 * The output will be the contents of the Node Stats data struct, one value per line.
 * The name of the field is also displayed.
 */
extern int qcsapi_wifi_get_node_stats(const char *ifname, const uint32_t node_index,
			int local_remote_flag, struct qcsapi_node_stats *p_stats);

/**
 * @brief Get the Maximum Number of Packets That Were Queued for the Selected Node
 *
 * Returns the maxiumum number of packets that were queued for the selected node.
 *
 * \param ifname \wifi0
 * \param node_index selects the node to report.
 * \param local_remote_flag use local flag to get local max queued packets, use remote flag to get
 *	max queues packets from remote associated STA; set to <c>QCSAPI_LOCAL_NODE</c> or
 *	<c>QCSAPI_REMOTE_NODE</c>
 * \param reset_flag whether to reset the statistic on read. "1" to reset and "0" not to reset.
 * \param max_queued address of a 32-bit unsigned integer to receive the value
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_max_queued \<WiFi interface\> \<node index\> \<local remote flag\>
 *	\<reset flag\></c>
 *
 * The output will be the maximum number of packets that were queued for the selected
 * node index.
 */
extern int qcsapi_wifi_get_max_queued(const char *ifname, const uint32_t node_index,
			int local_remote_flag, int reset_flag, uint32_t *max_queued);

/**
 * @brief Get HW Noise per association index.
 *
 * Returns the current HW noise.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA RSSID is to be read.
 *	On the STA, this should be 0.
 * \param p_hw_noise return parameter to contain the hw_noise on this association index.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_hw_noise \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current HW noise for the association in the range.
 */
extern int qcsapi_wifi_get_hw_noise_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, int *p_hw_noise);

/**
 * @brief Get a MLME statistics record.
 *
 * This API returns the mlme statistics record for specified mac address.
 *
 * \param client_mac_addr the mac addr of the client. 00:00:00:00:00:00 should be used
 * to get mlme stats for clients who were not associated with an AP.
 * \param stats address of a <c>struct qcsapi_mlme_stats</c>
 *
 * \return \zero_or_negative
 * \return -qcsapi_mlme_stats_not_supported if statistics facility is not suported.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mlme_stats_per_mac \<mac_addr\></c>
 *
 * Unless an error occurs, the output will be the mlme statistics for specified mac.
 */
extern int qcsapi_wifi_get_mlme_stats_per_mac(const qcsapi_mac_addr client_mac_addr,
					qcsapi_mlme_stats *stats);

/**
 * @brief Get a MLME statistics record.
 *
 * This API returns the mlme statistics record for specified association index.
 *
 * \param ifname the interface to perform the action on.
 * \param association_index the association index to get mlme statistics about
 * \param stats address of a <c>struct qcsapi_mlme_stats</c>
 *
 * \note \aponly
 *
 * \return \zero_or_negative
 * \return -qcsapi_mlme_stats_not_supported if statistics facility is not suported.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mlme_stats_per_association \<WIFI interface\> \<association index\></c>
 *
 * Unless an error occurs, the output will be the mlme statistics for specified association index.
 */
extern int qcsapi_wifi_get_mlme_stats_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, qcsapi_mlme_stats *stats);

/**
 * @brief Get a list of macs addresses.
 *
 * This API returns the list of macs currently existing in mlme statistic factory.
 *
 * \param macs_list address of a <c>struct qcsapi_mlme_stats_macs</c>
 *
 * \return \zero_or_negative
 * \return -qcsapi_mlme_stats_not_supported if statistics facility is not suported.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mlme_stats_macs_list</c>
 *
 * Unless an error occurs, the output will be the list of the mac addresses existing in mlme
 * statistics table.
 */
extern int qcsapi_wifi_get_mlme_stats_macs_list(qcsapi_mlme_stats_macs *macs_list);

/**
 * @brief Get the minimum, maximum and average measuring RSSI in dBm per association index.
 *
 * Returns the current minimum, maximum and average measuring Received Signal Strength Indication
 * (RSSI) in dBm if measurement period is set.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA to be read.
 *	On the STA, this should be 0.
 * \param p_rssi_min return parameter to contain the minimum measuring RSSI on this association
 *	index.
 * \param p_rssi_max return parameter to contain the maximum measuring RSSI on this association
 *	index.
 * \param p_rssi return parameter to contain the average measuring RSSI on this association index.
 * \param p_seq_num return parameter to contain the sequence number for this returned measuring
 *	RSSI.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_meas_rssi_minmax \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current minimum, maximum and average measuring RSSI in dBm for the
 * association and the sequence number for this RSSI. Before calling this call_qcsapi, you need to
 * set the measurement period with call_qcsapi set_rssi_meas_period
 *
 * \sa qcsapi_wifi_get_meas_rssi_in_dbm_per_association
 */
extern int qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index, int *p_rssi_min,
			int *p_rssi_max, int *p_rssi, uint32_t *p_seq_num);

/**
 * @brief Get Count of All Retransmissions by association.
 *
 * Returns total count of all retransmissions on the association.
 * Multiple re-transmissions of the same packet counted multiple times.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA,
 *	this should be 0.
 * \param p_tx_allretries return parameter to contain the count of all retransmissions on this
 * association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_allretries \<WiFi interface\> \<association index\></c>
 *
 * The output will be the count of all retransmissions on the association.
 */
extern int qcsapi_wifi_get_tx_allretries_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_allretries);

/**
 * @brief Get TX Packets exceeded retry limit by association.
 *
 * Returns the current number of MSDU frames that exceeded retry limit on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA,
 *	this should be 0.
 * \param p_tx_exceed_retry return parameter to contain the number of packets that exceeded retry
 * limit on this association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_exceed_retry \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of packets that exceeded retry limit the association.
 */
extern int qcsapi_wifi_get_tx_exceed_retry_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_exceed_retry);

/**
 * @brief Get TX Packets retransmitted by association.
 *
 * Returns the current number of MSDU frames that retransmitted on the association.
 * Multiple re-transmissions of the same packet count as one.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA,
 *	this should be 0.
 * \param get_tx_retried return parameter to contain the number of packets that retransmitted on
 *	this association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_retried \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current number of packets that retransmitted the association.
 */
extern int qcsapi_wifi_get_tx_retried_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_retried);

/**
 * @brief Get the Percentage of TX Packets Retransmitted by association.
 *
 * Returns the current percentage of packets that had to be retransmitted on the association.
 *
 * The count is set to 0 at the start of the association.
 *
 * \param ifname \wifi0
 * \param association_index On the AP this is the association index of the STA. On the STA,
 *	this should be 0.
 * \param p_tx_retried_percent return parameter to contain the percentage of packets that had to be
 *	retransmitted on this association.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_retried_percent \<WiFi interface\> \<association index\></c>
 *
 * The output will be the current percentage of packets that had to be retransmitted on the
 * association.
 */
extern int qcsapi_wifi_get_tx_retried_percent_per_association(const char *ifname,
			const qcsapi_unsigned_int association_index,
			qcsapi_unsigned_int *p_tx_retried_percent);

/** @} */

/**
 * @addtogroup PerNodeAPIs
 * @{
 */

/**
 * @brief Get a list of nodes.
 *
 * Get the MAC address and indiex of each node in a VAP.
 *
 * \param ifname \wifi0
 * \param node_list buffer to store the returned list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_list \<WiFi interface\></c>
 *
 * The output will be a list in the following format:
 *
 * @code
 * <node_index> <MAC addr>
 * @endcode
 */
int qcsapi_wifi_get_node_list(const char *ifname, struct qtn_nis_node_list *node_list);

/**
 * @brief Get selected information for a node
 *
 * Get selected information for a node.
 *
 * \param ifname \wifi0
 * \param node_index Node index, or 0 if using the mac_addr field
 * \param mac_addr \mac_format - ignored if node_index is not zero
 * \param set_id a predefined set number, from 1 to QTN_NIS_SET_ID_MAX
 * \param nis \databuf
 *
 * \return \zero_or_negative
 *
 * \note The median PHY rate is calculated from the last 16 unicast data packets. In the initial
 * connection phase, the returned value may not reflect the median of all packets if the packet
 * number is less than 16.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_infoset \<WiFi interface\> \<MAC addr\> \<set id\></c>
 *
 * Output is the contents of the info set in the following format, one line per field.
 * @code
 * <field label>                      : <value>
 * <field label>                      : <value>
 * ...
 * @endcode
 */
int qcsapi_wifi_get_node_infoset(const char *ifname, const uint16_t node_index,
				const qcsapi_mac_addr mac_addr,
				const qcsapi_unsigned_int set_id,
				struct qtn_nis_set *nis);

/**
 * @brief Get selected information for all nodes
 *
 * Get selected information for all nodes.
 *
 * \param ifname \wifi0
 * \param first_node_index Node index from which to start retrieving
 * \param set_id a predefined set number, from 1 to QTN_NIS_ALL_SET_ID_MAX
 * \param flags set-specific flags (not currently used)
 * \param nis \databuf
 *
 * \return \zero_or_negative
 *
 * \note There may be more associated nodes than can be contained in a single report.
 * The first_node_index parameter should initially be set to 0. If a report contains
 * QTN_NIS_ALL_ENTRY_MAX node entries, call the API again, setting first_node_index to one greater
 * than the node index in the last entry of the previous response, until a report is returned that
 * does not contain the maximum number of node entries. Refer to the
 * call_qcsapi_wifi_get_node_infoset_all() function for an example.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_node_infoset_all \<WiFi interface\> \<set id\> \<flags\></c>
 *
 * Output is the contents of the info set in the following format, one line per field
 * in the specified infoset for each associated station.
 * @code
 * <MAC address> <node index> <field label>                       : <value>
 * <MAC address> <node index> <field label>                       : <value>
 * ...
 * @endcode
 */
int qcsapi_wifi_get_node_infoset_all(const char *ifname,
	const uint16_t first_node_index, const qcsapi_unsigned_int set_id,
	const uint32_t flags, struct qtn_nis_all_set *nis);

/** @} */

/**
 * @addtogroup RegulatoryAPIs
 * @{
 */

/*Transmit power by regulatory authority*/
/**
 * \brief Get the list of WiFi regulatory regions.
 *
 * Use this API to get a list of the regulatory regions supported by the current firmware.
 * String will contain a list of regions, each separated by a comma.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_list_regulatory_regions.
 *
 * \param list_regulatory_regions the string where the results are returned.
 *
 * \return -EFAULT on error, or 0.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_regulatory_regions \<WiFi interface\></c>
 *
 * The output will be the list of regulatory regions that the firmware supports.
 * Some listed regions may be synonyms, e.g. "Europe" and "eu" are synonyms as are "USA" and "us".
 */
extern int qcsapi_wifi_get_list_regulatory_regions(string_256 list_regulatory_regions);

/*Transmit power by regulatory authority*/
/**
 * \brief Get the list of WiFi regulatory regions from regulatory database.
 *
 * Use this API to get a list of the regulatory regions supported by the current firmware.
 * String will contain a list of regions, each separated by a comma.
 *
 * \param list_regulatory_regions the string where the results are returned.
 *
 * \return -EFAULT on error, or 0.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_regulatory_regions \<WiFi interface\></c>
 *
 * The output will be the list of regulatory regions that the firmware supports.
 * Some listed regions may be synonyms, e.g. "Europe" and "eu" are synonyms as are "USA" and "us".
 */
extern int qcsapi_regulatory_get_list_regulatory_regions(string_256 list_regulatory_regions);

/**
 * \brief Get the List of Regulatory Channels.
 *
 * Use this API to get the list of channels authorized for use in the indicated regulatory region.
 * Bandwidth parameter should be either 20 or 40. Valid channels are returned
 * in the <c>list_of_channels</c> parameter as a list of numeric values separated by commas.
 * This API is provided as a reference and a convenience; its use is not required to insure
 * regulatory compliance.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_list_regulatory_channels.
 *
 * \param region_by_name the regulatory region for which the channel list is expected.
 * \param bw the bandwidth that is currently used. 40Mhz or 20Mhz.
 * \param list_of_channels the list of channels returned.
 *
 * \return -EFAULT, -EINVAL, -EOPNOTSUPP or other negative values on
 * error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_channels \<region\> {20 | 40}</c>
 *
 * <c>call_qcsapi get_list_regulatory_channels \<region\> {20 | 40}</c>
 *
 * where <c>\<regulatory region\></c> should be one of the regions listed in by the get list
 * regulatory regions API / command. Final parameter is the bandwidth and is optional.
 * If not present, the system will use the current configured bandwidth, defaulting to 40 if that
 * cannot be established. Output is the list of channels valid for that region separated by commas.
 *
 * Example:
 *
 * <c>call_qcsapi get_list_regulatory_channels eu</c>
 */
extern int qcsapi_wifi_get_list_regulatory_channels(const char *region_by_name,
						const qcsapi_unsigned_int bw,
						string_1024 list_of_channels);
/**
 * \brief Get the List of Regulatory Channels from regulatory database for all radios.
 *
 * Use this API to get the list of channels authorized for use in the indicated regulatory region
 * at least on one radio in the system.
 * Bandwidth parameter should be 20, 40 or 80.	Valid channels are returned
 * in the <c>list_of_channels</c> parameter as a list of numeric values separated by commas.
 * This API is provided as a reference and a convenience; its use is not required to insure
 * regulatory compliance.
 *
 * \param region_by_name the regulatory region for which the channel list is expected.
 * \param bw the bandwidth that is currently used. 80MHz, 40Mhz or 20Mhz.
 * \param list_of_channels the list of channels returned.
 *
 * \return -EFAULT, -EINVAL, -EOPNOTSUPP or other negative values on
 * error, or 0 on success.
 *
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_channels \<region\> {20 | 40 | 80}</c>
 *
 * <c>call_qcsapi get_list_regulatory_channels \<region\> {20 | 40 | 80}</c>
 *
 * where <c>\<regulatory region\></c> should be one of the regions listed in by the get list
 * regulatory regions API / command. Final parameter is the bandwidth and is optional.
 * If not present, the system will use the current configured bandwidth, defaulting to 80 if that
 * cannot be established. Output is the list of channels valid for that region separated by commas.
 *
 * Example:
 *
 * <c>call_qcsapi get_list_regulatory_channels eu</c>
 */
extern int qcsapi_regulatory_get_list_regulatory_channels(const char *region_by_name,
				const qcsapi_unsigned_int bw, string_1024 list_of_channels);

/**
 * \brief Get the List of Regulatory bands of all radios from regulatory database.
 *
 * Use this API to get the list of bands authorized for use at least on one physical radio in the
 * system when operating in indicated regulatory region.
 * Valid band indexes are returned in the <c>list_of_bands</c> parameter as a list of numeric
 * values separated by commas.
 * This API is provided as a reference and a convenience; its use is not required to insure
 * regulatory compliance.
 *
 * \param region_by_name the regulatory region for which the channel list is expected.
 * \param list_of_bands the list of bands returned.
 *
 * \return -EFAULT, -EINVAL, -EOPNOTSUPP or other negative values on
 * error, or 0 on success.
 *
 * \note To retrieve a list of bands authorized for use on specific radio only
 * use \link qcsapi_regulatory_get_list_regulatory_bands_if \endlink API.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_regulatory_bands \<regulatory region\></c>
 *
 * where <c>\<regulatory region\></c> should be one of the regions listed in by the get list
 * regulatory regions API / command.
 * Output is the list of bands valid for that region separated by commas.
 *
 * Example:
 * <c>call_qcsapi get_list_regulatory_bands eu</c>
 */
extern int qcsapi_regulatory_get_list_regulatory_bands(const char *region_by_name,
					string_128 list_of_bands);

/**
 * \brief Get the Regulatory Transmit Power.
 *
 * This API call gets the transmit power in a regulatory region for a particular
 * channel, the transmit power is per chain power based on current number of tx chains in use.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_regulatory_tx_power.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is returned.
 * \param p_tx_power the result which contains the transmit power.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_tx_power \<WiFi interface\> \<channel\> \<region\></c>
 *
 * Unless an error occurs, the output will be the channel number.
 */
extern int qcsapi_wifi_get_regulatory_tx_power(const char *ifname,
						const qcsapi_unsigned_int the_channel,
						const char *region_by_name, int *p_tx_power);

/**
 * \brief Get the Regulatory Transmit Power from regulatory database.
 *
 * This API call gets the transmit power in a regulatory region for a particular
 * channel.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is returned.
 * \param p_tx_power the result which contains the transmit power.
 * 160MHz bandwidth may use imbalanced Tx powers for the low/high 80M band, in which case, bits 8-15
 * will contain Tx power for the high 80M band.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_tx_power \<WiFi interface\> \<channel\> \<region\></c>
 *
 * Unless an error occurs, the output will be the tx power. For bw 160M imbalance tx power setting
 * in regulatory db, it will output 2 tx powers.
 */
extern int qcsapi_regulatory_get_regulatory_tx_power(const char *ifname,
		const qcsapi_unsigned_int the_channel, const char *region_by_name, int *p_tx_power);

/**
 * \brief Get WiFi Configured TX power.
 *
 * This API call gets the configured transmit power in a regulatory region
 * for a particular channel.
 *
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_configured_tx_power_ext.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is returned.
 * \param bw the bandwidth that is currently used. 40Mhz or 20Mhz.
 * \param p_tx_power the result which contains the transmit power.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_configured_tx_power \<WiFi interface\> \<channel\> <region\></c>
 *
 * Unless an error occurs, the output will be the channel number.
 *
 * Examples:
 *
 * @code
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 eu 40
 *	19
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 eu 20
 *	19
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 64 eu 40
 *	15
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 64 eu 20
 *	15
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 188 eu 20
 *	QCSAPI error 22: Invalid argument
 * @endcode
 *
 * Note: Numeric TX power results are just examples. Actual TX Power values may differ from what is
 * shown above.
 */
extern int qcsapi_wifi_get_configured_tx_power(const char *ifname,
				const qcsapi_unsigned_int the_channel,
				const char *region_by_name, const qcsapi_unsigned_int bw,
				int *p_tx_power);
/**
 * \brief Get WiFi Configured TX power from regulatory database.
 *
 * This API call gets the configured transmit power in a regulatory region
 * for a particular channel, for one spatial stream and beamforming off.
 * Please use qcsapi_regulatory_get_configured_tx_power_ext() to obtain
 * maximum allowed TX power taking into consideration beamforming and number of spatial streams.
 *
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_configured_tx_power_ext.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is returned.
 * \param bw the bandwidth that is currently used. 80Mhz, 40Mhz or 20Mhz.
 * \param p_tx_power the result which contains the transmit power.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_configured_tx_power \<interface\> \<channel\> \<region\> \<bandwidth\></c>
 *
 * Unless an error occurs, the output will be the channel number.
 *
 * Examples:
 *
 * @code
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 eu 40
 *	19
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 eu 20
 *	19
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 64 eu 40
 *	15
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 64 eu 20
 *	15
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 188 eu 20
 *	QCSAPI error 22: Invalid argument
 * @endcode
 */
extern int qcsapi_regulatory_get_configured_tx_power(const char *ifname,
							const qcsapi_unsigned_int the_channel,
							const char *region_by_name,
							const qcsapi_unsigned_int bw,
							int *p_tx_power);

/**
 * \brief Get WiFi Configured TX power from regulatory database.
 *
 * This API call gets the configured transmit power in a regulatory region
 * for a particular channel and number of spatial streams.
 * For a channel in 5GHz band, power value for FEM0 is returned.
 * For a channel in 2.4GHz band, power value for currently configured primary channel
 * position is returned.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region.
 * \param the_bw the bandwidth that is currently used. 80Mhz, 40Mhz or 20Mhz.
 * \param bf_on beamforming is either on or off. 1 for beamforming on and 0 for beamforming off.
 * \param number_ss the number of spatial streams.
 * \param p_tx_power the result which contains the transmit power. 0-7bits represent integer part,
 *	8-15bits represent decimal part.
 *
 * \return -EFAULT or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_configured_tx_power \<interface\> \<channel\> \<region\> \<bandwidth\>
 *	\<bf_on\> \<num_ss\></c>
 *
 * Unless an error occurs, the output will be the channel number.
 *
 * Examples:
 *
 * @code
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 us 80 1 4
 *	15
 *	quantenna # call_qcsapi get_configured_tx_power wifi0_0 100 us 20 0 2
 *	17
 * @endcode
 *
 * Note: Numeric TX power results are just examples. Actual TX Power values may differ from what is
 * shown above.
 */
extern int qcsapi_regulatory_get_configured_tx_power_ext(const char *ifname,
			const qcsapi_unsigned_int the_channel, const char *region_by_name,
			const qcsapi_bw the_bw, const qcsapi_unsigned_int bf_on,
			const qcsapi_unsigned_int number_ss, int *p_tx_power);

/**
 * \brief Set the Regulatory Region.
 *
 * This API call sets the regulatory region on a given interface.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_set_regulatory_region.
 *
 * \param ifname \wifi0
 * \param region_by_name the regulatory region.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_regulatory_region \<WiFi interface\> \<region\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_regulatory_region(const char *ifname, const char *region_by_name);

/**
 * \brief Set the Regulatory Region supported by regulatory database.
 *
 * This API call sets the regulatory region on a given interface.
 *
 * \param ifname \wifi0
 * \param region_by_name the regulatory region.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_regulatory_region \<WiFi interface\> \<region\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_regulatory_set_regulatory_region(const char *ifname, const char *region_by_name);
/**
 * \brief restore TX power by regulatory database
 *
 * This API call restore TX power by regulatory database
 *
 * \param ifname \wifi0
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi restore_regulatory_tx_power \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_regulatory_restore_regulatory_tx_power(const char *ifname);

/**
 * \brief Get the current Regulatory Region.
 *
 * This API call gets the current regulatory region on a given interface.
 *
 * \param ifname \wifi0
 * \param region_by_name the regulatory region that is currently configured.
 *
 * \return -EFAULT or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_region \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the '<c>\<region\></c>'.
 */
extern int qcsapi_wifi_get_regulatory_region(const char *ifname, char *region_by_name);

/**
 * \brief Overwrite country code, mainly set speicfic country string
 * in country IE when EU region is used.
 *
 * This API call sets secific country code for EU region on a given interface.
 *
 * \param ifname \wifi0
 * \param curr_country_name the current country name.
 * \param new_country_name the specific country name.
 *
 * \return -EFAULT, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi overwrite_country_code \<WiFi interface\> \<curr_country_name\>
 *	\<new_country_name\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_regulatory_overwrite_country_code(const char *ifname,
	const char *curr_country_name, const char *new_country_name);
/**
 * \brief Set WiFi Regulatory Channel.
 *
 * This API call sets the transmit power adjusting the offset on a given channel
 * on a given region(should be current region) for the passed in interface.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_set_regulatory_channel.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is modified
 * \param tx_power_offset the offset in integer from the currently configured tx power.
 *
 * \return -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_regulatory_channel \<WiFi interface\> \<channel\> \<region\> \<offset\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_regulatory_channel(const char *ifname,
					const qcsapi_unsigned_int the_channel,
					const char *region_by_name,
					const qcsapi_unsigned_int tx_power_offset);

/**
 * \brief Set WiFi Regulatory Channel supported by regulatory database.
 *
 * This API call sets the transmit power adjusting the offset on a given channel
 * on a given region(should be current region) for the passed in interface.
 *
 * \param ifname \wifi0
 * \param the_channel the channel for which the tx power is returned.
 * \param region_by_name the regulatory region for which the tx power is modified
 * \param tx_power_offset the offset in integer from the currently configured tx power.
 *
 * \return -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_regulatory_channel \<WiFi interface\> \<channel\> \<region\> \<offset\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_regulatory_set_regulatory_channel(const char *ifname,
					const qcsapi_unsigned_int the_channel,
					const char *region_by_name,
					const qcsapi_unsigned_int tx_power_offset);

/**
 * \brief Get regulatory database version number from regulatory database.
 *
 * This API call gets the regulatory database version number
 *
 * \param p_version pointer to save version number
 * \param index - which version number will be retrieved
 *
 * \return -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_db_version [index]</c>
 *
 * Unless an error occurs, the output will be the version number.
 */
extern int qcsapi_regulatory_get_db_version(int *p_version, const int index);

/**
 * \brief set if tx power is capped by regulatory database.
 *
 * This API call set TX power capped by regulatory database
 *
 * \param capped - zero for no capped by databse, non-zero for capped by database
 *
 * \return -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi apply_regulatory_cap \<interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_regulatory_apply_tx_power_cap(int capped);

/**
 * \brief Get a list of channels supported in specified region on a specified radio.
 *
 * Use this API to get the list of channels of specified bandwidth authorized for use in the
 * indicated regulatory region on a given radio.
 * Bandwidth parameter should be 20, 40 or 80.
 * Interface parameter is used to determine which band underlying physical radio is operating in.
 * Valid channels are returned in the <c>list_of_channels</c> parameter as a list of numeric
 * values separated by commas.
 * This API is provided as a reference and a convenience; its use is not required to insure
 * regulatory compliance.
 *
 * \param ifname \wifi0only
 * \param region_name the regulatory region for which the channel list is queried.
 * \param bw the bandwidth of the channels in the expected list, MHz - 80, 40 or 20.
 * \param restriction set to 1 to get a list of DFS affected channels, set to 0 to get the
 * complement list of channels, -1 for no additional restrictions.
 * \param list_of_channels pointer to a buffer where resulting channel list should be stored.
 *
 * \return -EFAULT, -EINVAL, -EOPNOTSUPP or other negative values on error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_regulatory_channels \<region\> [bandwidth] [ifname]</c>
 *
 * <c>call_qcsapi get_list_regulatory_channels \<region\> [bandwidth] [ifname]</c>
 *
 * where
 * <c>\<region\></c> - one of the regions listed in by the
 * \link qcsapi_regulatory_get_list_regulatory_regions \endlink.
 * <c>[bandwidth]</c> - bandwidth of channels in the list, in MHz. Can have values of 20, 40, 80
 * or "current" for using the bandwidth value which specified radio is currently using.
 * If not, specified, defaults to "current".
 * <c>[ifname]</c> - interface name for which the channel list is queried. If not specified,
 * defaults to "wifi0_0".
 *
 * Output is the list of channels of specified width authorized for use in indicated region in a
 * band which specified interface is operating in. Channels in the resulting list are separated
 * by commas. This call_qcsapi interface does not provide a way to specify additional restrictions
 * parameter, use \link qcsapi_regulatory_get_list_DFS_channels \endlink.
 *
 * Example:
 *
 * <c>call_qcsapi get_list_regulatory_channels eu 30 wifi2_0</c>
 */
extern int qcsapi_regulatory_get_list_regulatory_channels_if(const char *ifname,
				const char *region_name, const qcsapi_unsigned_int bw,
				const int restriction, string_1024 list_of_channels);

/**
 * \brief Get a list of bands authorized for use on a given radio from regulatory database.
 *
 * Use this API to get the list of bands authorized for use on indicated radio for the indicated
 * regulatory region. Valid bands are returned in the <c>list_of_bands</c> parameter as a list of
 * numeric values separated by commas. This API is provided as a reference and a convenience; its
 * use is not required to insure regulatory compliance.
 *
 * \param ifname \wifi0only
 * \param region_by_name the regulatory region for which the channel list is expected.
 * \param list_of_bands the list of bands returned.
 *
 * \return -EFAULT, -EINVAL, -EOPNOTSUPP or other negative values on
 * error, or 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_regulatory_bands \<regulatory region\> [ifname]</c>
 *
 * where <c>\<regulatory region\></c> is one of the regions listed in the get list
 * regulatory regions API.
 * <c>[ifname]</c> an interface name for which authorized band list is queried. If not specified,
 * all interfaces are queried.
 *
 * Output is the list of bands valid for that region on a given interface, separated by commas.
 *
 * Example:
 *
 * <c>call_qcsapi get_list_regulatory_bands eu wifi2_0</c>
 */
extern int qcsapi_regulatory_get_list_regulatory_bands_if(const char *ifname,
					const char *region_by_name, string_128 list_of_bands);

/**
 * \brief Get active transmit powers for the specified channel.
 *
 * This API returns all transmit powers for the specified channel currently used by device.
 * The TX powers are reported in dBm.
 *
 * See \ref qcsapi_chan_tx_powers_info for full details of the returned structure.
 *
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_get.
 *
 * \param ifname \wifi0only
 * \param dpwrt the pointer to a data structure which is used to store the return powers.
 * The variable "channel" of this structure must be initiated before calling this API.
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 *
 * \sa qcsapi_chan_tx_powers_info
 * \sa qcsapi_regulatory_chan_txpower_get
 */
extern int qcsapi_regulatory_get_chan_power_table(const char *ifname,
		struct qcsapi_chan_tx_powers_info *dpwrt);

/**
 * \brief Get transmit power values for a channel.
 *
 * Get transmit power values for a channel.
 *
 * Four types of power values can be queried.
 * * "active" Tx power values that are currently in use by the device
 * * "configured" Tx power that is used as default at boot-up. Configured
 *	Tx power sets regulatory and HW device limits for maximum Tx power.
 * * "dntx active" Tx power that is used for dynamic number of TX chains only when RF is 8x8.
 * * "dntx configured" Tx power that is useed as default at boot-up for dynamic number of TX chains
 *
 * The TX power values reported are in dBm.
 *
 * See \ref qcsapi_chan_tx_powers_info for the format of the structure describing Tx power values.
 *
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_with_decimal_get.
 *
 * \param ifname \wifi0only
 * \param info \databuf
 * \param report_type type of Tx power values to report, see \ref qcsapi_txpwr_value_type
 * for details.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reg_chan_txpower_get \<ifname\> [-n] [-f \<type\>]
 *	[-t \<dntx\>]\<chan:nss:bf:fem_pri:bw\></c>
 *
 * \param ifname interface name for which Tx powers are queried
 * \param -n do not print header (default is to print header)
 * \param -f Tx power values report format { active | configured } (default active)
 * \param -t Tx power values report type (dntx only)
 * \param chan:nss:bf:fem_pri:bw specifies which Tx power values to retrieve specifically.
 * Each subparameter (separated by symbol ":") can consist of a single value, a list of values
 * separated by symbol "," (no spaces), or special wildcard symbol "*" to choose all possible
 * values for this subparameter. Specifying channel number only equals to specifying
 * <chan:*:*:*:*>.
 * \param chan channel number
 * \param nss number of Spatial Streams, 1 to 8
 * \param bf '0' for Beamforming OFF case, '1' for Beamforming ON case
 * \param fem_pri meaning depends on which band does specified channel belong to. For
 * 2.4GHz channel, parameter specifies primary channel position: "0" for lower position and
 * "1" for higher position. For 5GHz channel, parameter specifies power value for which FEM
 * to update: "0" for FEM0, "1" for FEM1.
 * \param bw bandwidth in MHz, 20, 40, 80 or 160
 *
 * Unless an error occurs, the output will be the requested power table entry or entries.
 *
 * Examples:
 *
 * Get active Tx power value for channel 36, FEM0, Beamforming OFF, 2 Spatial Streams
 * and 20MHz bandwidth.
 * @code
 * #call_qcsapi reg_chan_txpower_get wifi0_0 36:2:0:0:20
 * @endcode
 *
 * Get active Tx power values for channel 100, FEM0, Beamforming ON, 1 Spatial Streams
 * and BW 20MHz, BW 80MHz, do not print header
 * @code
 * #call_qcsapi reg_chan_txpower_get wifi0_0 -n 100:1:1:0:20,80
 * @endcode
 *
 * Get configured Tx power values for channel 100, FEM0, Beamforming ON, 1 Spatial Streams
 * and BW 20-160 MHz.
 * @code
 * #call_qcsapi reg_chan_txpower_get wifi0_0 -f configured 100:1:1:0:20,40,80,160
 * #call_qcsapi reg_chan_txpower_get wifi0_0 -f configured 100:1:1:0:*
 * @endcode
 *
 * \sa qcsapi_chan_tx_powers_info
 * \sa qcsapi_regulatory_chan_txpower_set
 * \sa qcsapi_txpwr_value_type
 */
extern int qcsapi_regulatory_chan_txpower_get(const char *ifname,
	struct qcsapi_chan_tx_powers_info *dpwrt, qcsapi_txpwr_value_type report_type);

/**
 * \brief Selectively set active transmit powers for a single channel.
 *
 * Selectively set active transmit powers for the specified channel.
 *
 * Note that values configured with this API are NOT persistent and will be lost on device reboot.
 * The TX power values specified are in dBm.
 *
 * See the documentation for \ref qcsapi_chan_tx_powers_info for full
 * details of the format of the structure describing Tx power values.
 *
 * \note \deprecate_and_replace \ref qcsapi_regulatory_chan_txpower_with_decimal_set.
 *
 * \param ifname \wifi0only
 * \param info the pointer to a data structure containing information on all Tx power
 * values for the specified channel to be set as currently active. If some specific Tx power
 * value is set to "0", then active value used by device will not be updated - this allows
 * to update active Tx power values selectively.
 * \param set_type type of Tx power values to set, see \ref qcsapi_txpwr_value_type for details.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reg_chan_txpower_set \<ifname\> [-t \<dntx\>] \<chan:nss:bf:fem_pri:bw\>
 *	\<power_list\></c>
 *
 * \param ifname interface name for which Tx powers are set
 * \param -t Tx power values type (dntx only)
 * \param chan:nss:bf:fem_pri:bw specifies which Tx power values to update specifically.
 * Each subparameter (separated by symbol ":") can consist of a single value, a list of values
 * separated by symbol "," (no spaces), or special wildcard symbol "*" to choose all possible
 * values for this subparameter. Specifying channel number only equals to specifying
 * <chan:*:*:*:*>.
 * Note that power_list parameter must contain enough Tx power values separated with "," to
 * fill all Tx powers that were specified.
 * \param chan channel number
 * \param nss number of Spatial Streams, 1 to 8
 * \param bf '0' for Beamforming OFF case, '1' for Beamforming ON case
 * \param fem_pri meaning depends on which band does specified channel belong to. For
 * 2.4GHz channel, parameter specifies primary channel position: "0" for lower position and
 * "1" for higher position. For 5GHz channel, parameter specifies power value for which FEM
 * to update: "0" for FEM0, "1" for FEM1
 * \param bw bandwidth in MHz, 20, 40, 80 or 160
 * \param power_list list of power values, with values separated by ",". List must
 * have enough values to fill all Tx powers that were requsted to be updated.
 *
 * \call_qcsapi_string_complete
 *
 * Examples:
 *
 * Set active Tx power value as 17 for channel 36, FEM0, Beamforming OFF, 2 Spatial Streams
 * and 20MHz bandwidth.
 * @code
 * #call_qcsapi reg_chan_txpower_set wifi0_0 36:2:0:0:20 17
 * @endcode
 *
 * Set active Tx power values for channel 100, FEM0, Beamforming ON, 1 Spatial Streams
 * and BW 20MHz as 20, BW 80MHz as 19.
 * @code
 * #call_qcsapi reg_chan_txpower_set wifi0_0 100:1:1:0:20,80 20,19
 * @endcode
 *
 * Set active Tx power values for channel 100, FEM0, Beamforming ON, 1 Spatial Streams
 * and BW 20MHz as 20, BW 40MHz as 19, BW 80MHz as 18 and BW 160MHz as 17.
 * @code
 * #call_qcsapi reg_chan_txpower_set wifi0_0 100:1:1:0:20,40,80,160 20,19,18,17
 * #call_qcsapi reg_chan_txpower_set wifi0_0 100:1:1:0:* 20,19,18,17
 * @endcode
 *
 *
 * \sa qcsapi_chan_tx_powers_info
 * \sa qcsapi_regulatory_chan_txpower_get
 * \sa qcsapi_txpwr_value_type
 */
extern int qcsapi_regulatory_chan_txpower_set(const char *ifname,
	const struct qcsapi_chan_tx_powers_info *info, qcsapi_txpwr_value_type set_type);

/**
 * \brief Get current optional Tx power path
 *
 * Gets current file directory path - this may be the default path set at initialization time or
 * an alternative path set by application code.
 *
 * \param ifname interface name (not used)
 * \param file_path buffer to return null-terminated string with current path for txpower files
 * \param max_length buffer length
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reg_chan_txpower_path_get \<ifname>\ \<file_path\> \<max_length>\</c>
 *
 * \call_qcsapi_string_complete
 *
 * Examples:
 *
 * Get active Tx power path
 *
 * @code
 * call_qcsapi reg_chan_txpower_path_get wifi0_0
 * @endcode
 */
extern int qcsapi_regulatory_chan_txpower_path_get(const char *ifname, char *file_path,
						uint32_t max_len);

/**
 * \brief set the specified channel's TX power backoff.
 *
 * set transmit power backoff for the specified channel.
 *
 * Note that values configured with this API are NOT persistent and will be lost on device reboot.
 * The TX power backoff values specified are in dBm or percentage.
 *
 * \param ifname \wifi0
 * \param channel channel number
 * \param is_percentage the given backoff format
 * \param backoff backoff value
 * backoff values for the specified channel to be set.
 * If is_percentage is 0, the backoff is absolute value, else the backoff is the percentage
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_chan_txpower_backoff \<ifname\> \<channel\> \<is_percentage\> \<backoff\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_reg_chan_txpower_backoff_set(const char *ifname, uint8_t channel,
		uint8_t is_percentage, uint8_t backoff);

/**
 * \brief get the specified channel's TX power backoff.
 *
 * get transmit power backoff for the specified channel.
 *
 * \param ifname \wifi0
 * \param channel channel number
 * \param is_percentage indicates the backoff is absolute or percentage
 * \param backoff backoff value: absolute value or percentage
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_txpower_backoff \<ifname\> \<channel\></c>
 *
 * \return -EINVAL or other negative values on error, or 0 on success.
 */
extern int qcsapi_reg_chan_txpower_backoff_get(const char *ifname, uint8_t channel,
					uint8_t *is_percentage, uint8_t *backoff);

/**
 * \brief Get transmit power values for a channel.
 *
 * Get transmit power values for a channel.
 *
 * Four types of power values can be queried.
 * * "active" Tx power values that are currently in use by the device
 * * "configured" Tx power that is used as default at boot-up. Configured
 *	Tx power sets regulatory and HW device limits for maximum Tx power.
 * * "dntx active" Tx power that is used for dynamic number of TX chains only when RF is 8x8.
 * * "dntx configured" Tx power that is useed as default at boot-up for dynamic number of TX chains
 *
 * The TX power values reported are in dBm.
 *
 * See the documentation for \ref qcsapi_chan_tx_powers_with_decimal_info for full
 * details of the format of the structure describing Tx power values.
 *
 * \param ifname \wifi0only
 * \param info \databuf
 * \param report_type type of Tx power values to report, see \ref qcsapi_txpwr_value_type
 * for details.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reg_chan_txpower_get \<ifname\> [-n] [-f \<type\>]
 *	[-t \<dntx\>]\<chan:nss:bf:fem_pri:bw\></c>
 *
 * \param ifname interface name for which Tx powers are queried
 * \param -n do not print header (default is to print header)
 * \param -f Tx power values report format { active | configured } (default active)
 * \param -t Tx power values report type (dntx only)
 * \param chan:nss:bf:fem_pri:bw specifies which Tx power values to retrieve specifically.
 * Each subparameter (separated by symbol ":") can consist of a single value, a list of values
 * separated by symbol "," (no spaces), or special wildcard symbol "*" to choose all possible
 * values for this subparameter. Specifying channel number only equals to specifying
 * <chan:*:*:*:*>.
 * \param chan channel number
 * \param nss number of Spatial Streams, 1 to 8
 * \param bf '0' for Beamforming OFF case, '1' for Beamforming ON case
 * \param fem_pri meaning depends on which band does specified channel belong to. For
 * 2.4GHz channel, parameter specifies primary channel position: "0" for lower position and
 * "1" for higher position. For 5GHz channel, parameter specifies power value for which FEM
 * to update: "0" for FEM0, "1" for FEM1.
 * \param bw bandwidth in MHz, 20, 40, 80 or 160
 *
 * Unless an error occurs, the output will be the requested power table entry or entries.
 *
 * \sa qcsapi_chan_tx_powers_with_decimal_info
 * \sa qcsapi_regulatory_chan_txpower_with_decimal_set
 * \sa qcsapi_txpwr_value_type
 */
extern int qcsapi_regulatory_chan_txpower_with_decimal_get(const char *ifname,
	struct qcsapi_chan_tx_powers_with_decimal_info *dpwrt, qcsapi_txpwr_value_type report_type);

/**
 * \brief Selectively set active transmit powers for a single channel.
 *
 * Selectively set active transmit powers for the specified channel.
 *
 * Note that values configured with this API are NOT persistent and will be lost on device reboot.
 * The TX power values specified are in dBm.
 *
 * See the documentation for \ref qcsapi_chan_tx_powers_with_decimal_info for full
 * details of the format of the structure describing Tx power values.
 *
 * \param ifname \wifi0only
 * \param info the pointer to a data structure containing information on all Tx power
 * values for the specified channel to be set as currently active. If some specific Tx power
 * value is set to "0", then active value used by device will not be updated - this allows
 * to update active Tx power values selectively.
 * \param set_type type of Tx power values to set, see \ref qcsapi_txpwr_value_type for details.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reg_chan_txpower_set \<ifname\> [-t \<dntx\>] \<chan:nss:bf:fem_pri:bw\>
 *	\<power_list\></c>
 *
 * \param ifname interface name for which Tx powers are set
 * \param -t Tx power values type (dntx only)
 * \param chan:nss:bf:fem_pri:bw specifies which Tx power values to update specifically.
 * Each subparameter (separated by symbol ":") can consist of a single value, a list of values
 * separated by symbol "," (no spaces), or special wildcard symbol "*" to choose all possible
 * values for this subparameter. Specifying channel number only equals to specifying
 * <chan:*:*:*:*>.
 * Note that power_list parameter must contain enough Tx power values separated with "," to
 * fill all Tx powers that were specified. Tx power value can be integer only or with one decimal.
 * \param chan channel number
 * \param nss number of Spatial Streams, 1 to 8
 * \param bf '0' for Beamforming OFF case, '1' for Beamforming ON case
 * \param fem_pri meaning depends on which band does specified channel belong to. For
 * 2.4GHz channel, parameter specifies primary channel position: "0" for lower position and
 * "1" for higher position. For 5GHz channel, parameter specifies power value for which FEM
 * to update: "0" for FEM0, "1" for FEM1
 * \param bw bandwidth in MHz, 20, 40, 80 or 160
 * \param power_list list of power values, with values separated by ",". List must
 * have enough values to fill all Tx powers that were requsted to be updated.
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_chan_tx_powers_info
 * \sa qcsapi_regulatory_chan_txpower_with_decimal_get
 * \sa qcsapi_txpwr_value_type
 */
extern int qcsapi_regulatory_chan_txpower_with_decimal_set(const char *ifname,
			const struct qcsapi_chan_tx_powers_with_decimal_info *info,
			qcsapi_txpwr_value_type report_type);

/** @} */

/**
 * @addtogroup DFSAPIs
 * @{
 */

/**
 * @brief Get the list of DFS channels
 *
 * Use this API to get a list of all channels that require following the DFS protocols,
 * or alternately a list of channels that do not require the DFS protocols.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_get_list_DFS_channels.
 *
 * \param region_by_name the region to return. Has the same interpretation as with the regulatory
 *	authority APIs.
 * \param DFS_flag set to 1 to get a list of DFS affected channels, set to 0 to get the complement
 *	list of channels.
 * \param bw the bandwidth in use - either 20 or 40 to represent 20MHz and 40MHz respectively.
 * \param list_of_channels return parameter to contain the comma delimited list of channels.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_DFS_channels \<regulatory region\> {0 | 1} {20 | 40}</c>
 *
 * where \<regulatory region\> should be one of the regions listed in by the get list regulatory
 * regions API/command.
 *
 * Choices for the other two parameters are as shown above.
 *
 * Unless an error occurs, the output will be a list of channels, each value separated by a comma.
 *
 * <b>Examples:</b>
 *
 * To get the list of 40 MHz channels that require following the DFS protocols for Europe, enter:
 *
 * <c>call_qcsapi get_list_DFS_channels eu 1 40</c>
 *
 * To get the list of 20 MHz channels that do not require DFS protocols for the US, enter:
 *
 * <c>call_qcsapi get_list_DFS_channels us 0 20</c>
 */
extern int qcsapi_wifi_get_list_DFS_channels(const char *region_by_name, const int DFS_flag,
					const qcsapi_unsigned_int bw, string_1024 list_of_channels);

/**
 * @brief Get the list of DFS channels for all radios in system
 *
 * Use this API to get a list of all channels authorized for use on all radios in system that
 * require following the DFS protocols, or alternately a list of channels that do not require
 * the DFS protocols.
 *
 * \param region_by_name the region to return. Has the same interpretation as with the regulatory
 *	authority APIs.
 * \param DFS_flag set to 1 to get a list of DFS affected channels, set to 0 to get the complement
 *	list of channels.
 * \param bw the bandwidth in use - either 20, 40 or 80 to represent 20, 40 and 80MHz respectively.
 * \param list_of_channels return parameter to contain the comma delimited list of channels.
 *
 * \return \zero_or_negative
 *
 * \note To retrieve a list of DFS channels in a given regulatory region on specific
 * interface only use \link qcsapi_regulatory_get_list_regulatory_channels_if \endlink.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_list_DFS_channels \<regulatory region\> {0 | 1} {20 | 40 | 80}</c>
 *
 * where \<regulatory region\> should be one of the regions listed in by the get list regulatory
 * regions API/command.
 *
 * Choices for the other two parameters are as shown above.
 *
 * Unless an error occurs, the output will be a list of channels, each value separated by a comma.
 *
 * <b>Examples:</b>
 *
 * To get the list of 80 MHz channels that require following the DFS protocols for Europe, enter:
 *
 * <c>call_qcsapi get_list_DFS_channels eu 1 80</c>
 *
 * To get the list of 40 MHz channels that require following the DFS protocols for Europe, enter:
 *
 * <c>call_qcsapi get_list_DFS_channels eu 1 40</c>
 *
 * To get the list of 20 MHz channels that do not require DFS protocols for the US, enter:
 *
 * <c>call_qcsapi get_list_DFS_channels us 0 20</c>
 */
extern int qcsapi_regulatory_get_list_DFS_channels(const char *region_by_name,
					const int DFS_flag, const qcsapi_unsigned_int bw,
					string_1024 list_of_channels);

/**
 * @brief Is the given channel a DFS channel.
 *
 * Determine whether a particular channel is subject to the DFS protocols.
 *
 * \warning \bad_regulatory
 * \note \deprecate_and_replace \ref qcsapi_regulatory_is_channel_DFS.
 *
 * \param region_by_name the region to return. Has the same interpretation as with the regulatory
 *	authority APIs.
 * \param the_channel unsigned integer from 0 to 255. The channel must be valid for the referenced
 *	regulatory region.
 * \param p_channel_is_DFS return value which is set to 1 if the channel is affected by DFS, set to
 *	0 otherwise.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_channel_DFS \<regulatory region\> \<channel\></c>
 *
 * where \<regulatory region\> should be one of the regions listed in by the get list regulatory
 * regions API / command and \<channel\> is an unsigned integer.
 *
 * Unless an error occurs, the output will be either 0 or 1 depending on
 * whether DFS protocols are required for the referenced channel.
 */
extern int qcsapi_wifi_is_channel_DFS(const char *region_by_name,
				const qcsapi_unsigned_int the_channel, int *p_channel_is_DFS);

/**
 * @brief Is the given channel a DFS channel on any of the radio devices.
 *
 * Use this API to determine whether a particular channel is subject to the DFS protocols when used
 * on any of the radio devices in system.
 *
 * \param region_by_name the region to return. Has the same interpretation as with the regulatory
 *	authority APIs.
 * \param the_channel unsigned integer from 0 to 255. The channel must be valid for the referenced
 *	regulatory region.
 * \param p_channel_is_DFS return value which is set to 1 if the channel is affected by DFS, set
 *	to 0 otherwise.
 *
 * \return \zero_or_negative
 *
 * \note To setermine if specified channel is subject to DFS rules on specific radio only
 * use \link qcsapi_regulatory_is_channel_DFS_if \endlink API.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_channel_DFS \<regulatory region\> \<channel\></c>
 *
 * where \<regulatory region\> should be one of the regions listed in by the get list regulatory
 * regions API / command and \<channel\> is an unsigned integer.
 *
 * Unless an error occurs, the output will be either 0 or 1 depending on
 * whether DFS protocols are required for the referenced channel.
 */
extern int qcsapi_regulatory_is_channel_DFS(const char *region_by_name,
				const qcsapi_unsigned_int the_channel, int *p_channel_is_DFS);
/**
 * @brief Get previous and current channels from the most recent DFS channel change event
 *
 * This API returns the channel switched from and to as a result of the most recent DFS channel
 * change event.
 *
 * \param ifname \wifi0
 * \param p_prev_channel result memory pointer for the channel switched from
 * \param p_cur_channel result memory pointer for the channel switched to
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dfs_cce_channels \<WiFi interface\></c>
 *
 * The output will be the previous channel number then the current channel number, unless an error
 * occurs. If no DFS channel change has occurred, both numbers will be zero.
 */
extern int qcsapi_wifi_get_dfs_cce_channels(const char *ifname, qcsapi_unsigned_int *p_prev_channel,
					qcsapi_unsigned_int *p_cur_channel);

/**
 * \brief Get the alternative DFS channel to be used in case of radar
 * detection.
 *
 * Get the alternative DFS channel that will be switched to if radar is detected in the current
 * channel. This is known as a 'fast switch', to allow quickly changing to another high power
 * channel without having to do slow scans through all the channels.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_dfs_alt_chan return parameter for the alternative DFS channel.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_DFS_alt_channel \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be a string containing the DFS alternative channel, or
 * <c>0</c> if no alternative channel is specified.
 *
 * \sa qcsapi_wifi_set_DFS_alt_channel
 */
extern int qcsapi_wifi_get_DFS_alt_channel(const char *ifname, qcsapi_unsigned_int *p_dfs_alt_chan);

/**
 * \brief Set the alternative DFS channel to be used in case of radar
 * detection.
 **
 * This API call is used to set the alternative DFS channel that will be
 * switched over to in case radar is detected in the current channel.
 * This is known as a 'fast switch', to allow quickly changing to another
 * high power channel without having to do slow scans through all the
 * channels.
 *
 * \param ifname \wifi0
 * \param dfs_alt_chan the alternative DFS channel to set.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_DFS_alt_channel \<WiFi interface\> \<alternative channel\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Error can occur if the alternative channel being set is the same as the
 * current channel on the device.
 *
 * \sa qcsapi_wifi_get_DFS_alt_channel
 */
extern int qcsapi_wifi_set_DFS_alt_channel(const char *ifname,
					const qcsapi_unsigned_int dfs_alt_chan);

/**
 * \brief Start a channel scan and select a best DFS channel for usage.
 *
 * This API is used to trigger a channel scan and once the scan is done, the AP will switch to a
 * DFS channel based on channel ranking algorithm.
 *
 * \param ifname \wifi0
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_dfsreentry \<WiFi interface\></c>
 * \call_qcsapi_string_complete
 *
 * Error can occur if all DFS channels have been in non-occupy list.
 */
extern int qcsapi_wifi_start_dfs_reentry(const char *ifname);

/**
 * \brief Set the radar block id and antenna id to be used in case of radar
 * detection.
 **
 * This API call is used to set radar chain when doing radar dectection.
 *
 * \param radar_blockid the radar dectection block id to select. "0" to select 1st DFS radar block
 * id, "1" to select 2nd DFS radar block id. \param radar_chain_selection the radar antenna id to
 * set.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_radar_chain \<radar_blockid\> \<radar_chain_selection\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Error can occur if the alternative channel being set is the same as the
 * current channel on the device.
 */
extern int qcsapi_wifi_set_radar_chain(const qcsapi_unsigned_int radar_blockid,
					const qcsapi_unsigned_int radar_chain_selection);

/**
 * \brief Start a channel scan and select channel based on given rules.
 *
 * This API provides a way to scan channel and select channel by different rules.
 *
 * \param ifname \wifi0
 * \param scan_flag flags to control the channel selection algorithm, the set of channel to be
 * scanned and the scanning algorithm to be used
 *
 * Flags for setting the channel selection algorithm. These flags are mutually exclusive.
 * @code
 *	IEEE80211_PICK_CLEAREST		0x0100
 *	IEEE80211_PICK_REENTRY		0x0200
 *	IEEE80211_PICK_NOPICK		0x0400
 *	IEEE80211_PICK_NOPICK_BG	0x0800
 * @endcode
 *
 * Flags for setting the channels to be scanned. These flags are mutually exclusive.
 * @code
 *	IEEE80211_PICK_ALL		0x0001
 *	IEEE80211_PICK_DFS		0x0002
 *	IEEE80211_PICK_NONDFS		0x0004
 * @endcode
 *
 * Flags for controlling the scanning algorithm. Flags containing the string "BG" are mutually
 * exclusive.
 * @code
 *	IEEE80211_PICK_SCAN_FLUSH			0x0008
 *	IEEE80211_PICK_BG_ACTIVE			0x0010
 *	IEEE80211_PICK_BG_MULTI_SLOTS_FAST		0x0020
 *	IEEE80211_PICK_BG_MULTI_SLOTS_NORMAL		0x0040
 *	IEEE80211_PICK_BG_MULTI_SLOTS_SLOW		0x0080
 *	IEEE80211_PICK_BG_CHECK				0x8000
 *	IEEE80211_PICK_BG_MULTI_SLOTS_AUTO		0x4000
 * @endcode
 *
 * <c>scan_flag</c> may be any combination of channel set and algorithm.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_scan wifi0_0 \<algorithm\> \<select_channel\> \<control_flag\></c>
 *
 * \param algorithm set the channel selection algorithm
 * \li "reentry" - start dfs-reentry function.
 * \li "clearest" - pick the clearest channel (default flag).
 * \li "no_pick" - only perform channel scan.
 * \li "background" - scan channel in the background and no_pick.
 *
 * \param select_channel indicate that what kind of channel to be selected
 * \li "dfs" - pick channel from available dfs channels.
 * \li "non-dfs" - pick channel from available non-dfs channels.
 * \li "all" - pick channel from all available channels (default flag).
 *
 * \param control_flags indicate the required behavior for scanning activity,
 * "active", "fast", "normal", "slow" and "check" work for "background" algorithm only.
 * \li "flush" - flush previous scanning result before the new channel scan.
 * \li "active" - scan with probe requests.
 * \li "fast" - scan in a faster speed. with probe requests on non-DFS channels
 * and without on DFS channels.
 * \li "slow" - scan in a slower speed. with probe requests on non-DFS channels
 * and without on DFS channels.
 * \li "normal" - scan in a speed between them above.  with probe requests on non-DFS channels
 * and without on DFS channels.
 * \li "auto" - choose one from fast/slow/normal automatically.
 * \li "check" - enable checking for conflicts with the AP's normal beacon broadcasting in order
 * to guarantee consistent beacon broadcasting. It is normally not necessary and slightly decreases
 * the time available for scanning.
 *
 * Examples:
 * @code
 * <c>call_qcsapi start_scan wifi0_0</c>
 * <c>call_qcsapi start_scan wifi0_0 reentry dfs</c>
 * <c>call_qcsapi start_scan wifi0_0 clearest non_dfs flush</c>
 * <c>call_qcsapi start_scan wifi0_0 background all fast</c>
 * <c>call_qcsapi start_scan wifi0_0 background all slow check</c>
 * @endcode
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_start_scan_ext(const char *ifname, const int scan_flag);

/**
 * \brief Get channel switch history records
 *
 * This API reports back the channel change history up to a maximum of 32 records. This API can also
 * reset the records. As a get function, it needs <c>struct qcsapi_csw_record</c> as a buffer to
 * receive return data.
 *
 * \param ifname \wifi0
 * \param reset indicate whether to reset the records. "1" to reset records, and "0" to get records.
 * \param record where to store the records.
 *
 * \return \positive_or_negative
 *
 * \note This API does not work on a STA.
 *
 * \callqcsapi
 *
 * @code
 * <c>call_qcsapi get_csw_records \<WiFi interface\></c>
 * <c>call_qcsapi get_csw_records \<WiFi interface\> 1</c>
 * @endcode
 *
 * The output from the first command is the channel change record count, followed by a list of
 * channel change records. A channel change record includes time from start-up, channel that was
 * selected and a reason for channel change. Reasons are enumerated by \ref ieee80211_csw_reason.
 * Mappings to printed strings are defined by the array <c>qcsapi_csw_reason_list</c>.
 *
 * The output from the second command is the channel change history
 * followed by the string <c>"clear records complete"</c>.
 *
 * @code
 * call_qcsapi get_csw_records wifi0_0 1
 * channel switch history record count : 3
 * time=1234 channel=123 reason=SCS
 * time=11 channel=36 reason=CONFIG
 * time=7 channel=40 reason=CONFIG
 * clear records complete
 * @endcode
 */
extern int qcsapi_wifi_get_csw_records(const char *ifname, int reset, qcsapi_csw_record *record);

/**
 * \brief Get channel radar status and history of detected records
 *
 * This API is used to query the status of a DFS channel; whether it is in non-occupy list, and how
 * many times the radar signal has be detected on this channel. This data can be used to analyse the
 * local environment for radar usage.
 *
 * \param ifname \wifi0
 * \param rdstatus when used as input,it contain channel to query, when used as return value, it
 *	stores the radar channel status.
 *
 * \return \positive_or_negative
 * \note The channel passed in the rdstatus structure must be DFS affected, based on the
 *	regulatory region.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_radar_status \<WiFi interface\> \<DFS-Channel\></c>
 *
 * Unless an error occurs, the output will show the radar status of <c>DFS-Channel</c>.
 *
 * Example:
 *
 * @code
 * #call_qcsapi get_radar_status wifi0_0 100
 * channel 100:
 * radar_status=0
 * radar_count=1
 * @endcode
 */
extern int qcsapi_wifi_get_radar_status(const char *ifname, qcsapi_radar_status *rdstatus);

/**
 * @brief Get CAC status.
 *
 * This API is used to get CAC status on AP. Applications can use this API to poll CAC status
 * and ensure CAC process is completed.
 *
 * \param ifname \wifi0
 * \param cacstatus return the currently CAC status, 1 for CAC is running and 0 for no CAC is
 *	running.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_cacstatus \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the 1 or 0.
 */
extern int qcsapi_wifi_get_cac_status(const char *ifname, int *cacstatus);

/**
 * \internal
 * \brief Set channel metric margin for SCS channel ranking to switch from DFS-channel to
 *	Non-DFS-channel
 *
 * This API controls the channel metric margin SCS used for channel ranking to switch from
 * DFS-channel to Non-DFS-channel
 *
 * \param ifname \wifi0
 * \param leavedfs_chan_mtrc_mrgn value that indicates the channel metric margin.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scs_leavedfs_chan_mtrc_mrgn \<WiFi interface\> \<leavedfs_chan_mtrc_mrgn\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn(const char *ifname,
					uint8_t leavedfs_chan_mtrc_mrgn);

/**
 * @brief Set the maximum boot time CAC duration in seconds.
 *
 * This API call is used to configure the max Call Admission Control duration to be used at boot
 * time.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param max_boot_cac_duration Maximum boot time CAC duration in seconds.
 *				The valid range	is [(-1) - MAX_BOOT_CAC_DURATION].
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_max_boot_cac_duration \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_set_max_boot_cac_duration(const char *ifname, const int max_boot_cac_duration);

/**
 * \brief Get ICAC status.
 *
 * Get the ICAC (Initial Channel Availability Check) status. This API can be used
 * to check if the ICAC process has completed.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param status \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_icac_status \<WiFi interface\></c>
 *
 * The output will be the word "Active" or "Inactive" unless an error occurs.
 */
extern int qcsapi_wifi_get_icac_status(const char *ifname, int *status);

/**
 * \internal
 * @brief Check if the channel provided is a weather one.
 *
 * This API call is used to check if the channel provided is a weather one.
 *
 * \param ifname \wifi0
 * \param channel the channel needs to check
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_weather_channel \<WiFi interface\> \<channel\></c>
 *
 * Unless an error occurs, the output will be the string <c>1</c> or <c>0</c>.
 */
extern int qcsapi_wifi_is_weather_channel(const char *ifname, const uint16_t channel);

/**
 * @brief Check if channel requires DFS procedure on a given radio interface.
 *
 * Use this API to determine whether a particular channel is subject to the DFS protocols on a
 * given radio device.
 *
 * \param ifname \wifi0only
 * \param region_by_name regulatory region. Has the same interpretation as with the regulatory
 *	authority APIs.
 * \param the_channel unsigned integer from 0 to 255. The channel must be valid for the referenced
 *	regulatory region.
 * \param p_channel_is_DFS return value which is set to 1 if the channel is affected by DFS, set
 *	to 0 otherwise.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi is_channel_DFS \<regulatory region\> \<channel\> [ifname]</c>
 *
 * Where
 *
 * - \<regulatory region\> should be one of the regions listed in by the get list regulatory
 *	regions API / command.
 * - \<channel\> is an unsigned integer.
 * - \<ifname\> radio interface which is supposed to be using specified channel. If not
 *	specified, all interfaces are queried.
 *
 * Unless an error occurs, the output will be either 0 or 1 depending on
 * whether DFS protocols are required for the referenced channel.
 */
extern int qcsapi_regulatory_is_channel_DFS_if(const char *ifname, const char *region_by_name,
				const qcsapi_unsigned_int the_channel, int *p_channel_is_DFS);

/** @} */

/**
 * @addtogroup ScanAPIs
 * @{
 */

/**
 * @brief Get the results of an AP scan.
 *
 * This API gets the results of the most recent AP scan and caches them in memory for future
 * reference.
 *
 * \param ifname \wifi0
 * \param p_count_APs return parameter to contain the count of the number of AP scan results.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_results_AP_scan \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the number of APs found in the last scan.
 */
extern int qcsapi_wifi_get_results_AP_scan(const char *ifname, qcsapi_unsigned_int *p_count_APs);

/**
 * @brief Get the results of an AP scan performed by SCS off channel sampling.
 *
 * Get the results of AP scan performed by SCS off channel sampling and cache them in memory for
 * future reference.
 *
 * \param ifname \wifi0
 * \param p_count_APs return parameter to contain the count of the number of AP scan results.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi show_access_points \<WiFi interface\> {0 | 1}</c>
 *
 * Unless an error occurs, the output will be the summary of APs found in the last scan.
 */
extern int qcsapi_wifi_get_results_AP_scan_by_scs(const char *ifname,
					qcsapi_unsigned_int *p_count_APs);

/**
 * \brief Get a count of the number of APs scanned.
 *
 * This API call is used to get the count of APs that have been scanned in the
 * most recent channel scan.
 *
 * \param ifname \wifi0
 * \param p_count_APs return parameter to contain the count of APs that have
 * been scanned in the most recent scan.
 *
 * \return \positive_or_negative
 * If the call succeeded, p_count_APs contains a count of the number of APs scanned in the previous
 * scan.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_count_APs_scanned \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the number of APs that were
 * scanned in the previous channel scan.
 */
extern int qcsapi_wifi_get_count_APs_scanned(const char *ifname, qcsapi_unsigned_int *p_count_APs);

/**
 * @brief Get properties of the APs detected in a scan
 *
 * Get properties of the APs detected in a scan.
 *
 * \param ifname \wifi0
 * \param index_AP the index of an AP in the scan list, starting from 0
 * \param p_ap_properties \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_properties_AP \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be the properties of the referenced AP. Each of the
 * following fields is printed, separated by a space. Refer to qcsapi_ap_properties for the
 * field descriptions.
 *
 * Example output from call_qcsapi:
 *
 * @code
 * quantenna # call_qcsapi get_properties_AP wifi0_0 0
 * "qtest-wifi0_0" 00:26:86:f0:83:b9 100 55 f 2 1 2 1734000000 0 28 0 80 0 2 200 3 0
 *	6,162.5,325,487.5,650,975,1300,1462.5,1625
 *	6,9,12,18,24,36,48,54,15,30,45,60,90,120,135,150,30,60,90,120,180,240,270,304
 * @endcode
 *
 * The fields, in order, are:
 *
 * * "qtest-wifi0_0" - Service Set ID (SSID)
 * * 00:26:86:f0:83:b9 - BSSID
 * * 100 - Channel number
 * * 55 - Received Signal Strength Indicator (RSSI)
 * * f - Flags (0b1111 = security enabled (bit 0), SGI capable in 20 MHz (bit 1),
 *	40 MHz (bit 2) and 80 MHz (bit 3))
 * * 2 - Protocols (WPA2)
 * * 1 - Accepted cipher suites (CCMP)
 * * 2 - Encryption modes (WPA2)
 * * 1734000000 - best data rate (1,734 Mbps)
 * * 0 - WPS capability (no WPS)
 * * 28 - supported IEEE 802.11 protocols (0b11100 = 802.11a (bit 2), 802.11n (bit 3),
 *	802.11ac (bit 4))
 * * 0 - QHop role (none)
 * * 80 - Bandwidth
 * * 0 - Noise, in dBm
 * * 2 - Operating mode (AP)
 * * 200 - Beacon interval (TUs)
 * * 3 - Delivery Traffic Indication Map (DTIM) period
 * * 0 - Flag to indicate the presence of a non-ERP station
 * * 6,162.5,325,487.5,650,975,1300,1462.5,1625 - Basic rate set, in Mbps
 * * 6,9,12,18,24,36,48,54,15,30,45,60,90,120,135,150,30,60,90,120,180,240,270,304 -
 *	Supported rate set, in Mbps
 *
 * @code
 * quantenna # call_qcsapi get_properties_AP wifi0_0 4
 * QCS API error 34: Parameter value out of range
 * @endcode
 *
 * When the index becomes too large, the call_qcsapi command will fail as shown above.
 * In this example, only 4 APs were found in the scan.	Since the index is numbered starting at 0,
 * valid index values are 0, 1, 2 and 3.
 *
 * The report is generated from cached AP scan results. To ensure scan results are up to date, the
 * application should call \ref qcsapi_wifi_get_results_AP_scan first.
 *
 * \sa qcsapi_ap_properties
 */
extern int qcsapi_wifi_get_properties_AP(const char *ifname, const qcsapi_unsigned_int index_AP,
					qcsapi_ap_properties *p_ap_properties);

/**
 * @brief Get AP WPS IE per scan result.
 *
 * This API reports on the WPS IE of an AP, with the AP indentified by index.
 * The index is numbered starting at 0.
 *
 * If the cache of AP scan results is not present, this API will call the Get Results AP Scan to
 * update the cache.
 *
 * \param ifname \wifi0
 * \param index_AP the index to get the result from.
 * \param ie_data return parameter for storing the AP WPS IE scan data.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_wps_ie_scanned_AP \<WiFi interface\> \<index\></c>
 *
 * Unless an error occurs, the output will be the number of octets in the Information field of WPS
 * IE and HEX-encoded string that represents the Information field.
 *
 * Example output from call_qcsapi:
 *
 * @code
 * quantenna # call_qcsapi get_wps_ie_scanned_AP wifi0_0 0
 * 31 DD1D0050F204104A0001101044000102103C0001021049000600372A000120
 * @endcode
 *
 * \note The Get Properties API uses the results it finds in the in-memory cache.
 * To ensure the results from the latest AP scan are used,
 * an application should always call the Get Results AP Scan API first.
 *
 * \sa qcsapi_ie_data
 */
extern int qcsapi_wifi_get_wps_ie_scanned_AP(const char *ifname,
			const qcsapi_unsigned_int index_AP, struct qcsapi_ie_data *ie_data);


/**
 * \brief Set scan results check interval, unit is second
 *
 * This API sets the scan results check interval
 *
 * \param ifname \wifi0only
 * \param scan_chk_inv interval for scan results availability check
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scan_chk_inv \<WiFi interface\> \<scan_chk_inv\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scan_chk_inv(const char *ifname,
						int scan_chk_inv);

/**
 * \brief Get scan results check interval, unit is second
 *
 * This API gets the scan results check interval
 *
 * \note This API is available on AP/STA mode.
 *
 * \param ifname \wifi0only
 * \param p pointer to interval for scan results check
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scan_chk_inv \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_get_scan_chk_inv(const char *ifname,
						int *p);

/**
 * \brief Set the maximum scan buffer size for returned scan results
 *
 * Configure the maximum buffer size for scan results. If the result list exceeds this size
 *	it is sorted according to the following rules prior to truncation.
 * - matched SSID
 * - WPS active
 * - WPA/RSN security
 * - High RSSI
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param max_buf_size	max buffer size vlaue
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scan_buf_max_size \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scan_buf_max_size(const char *ifname, const unsigned int max_buf_size);

/**
 * \brief Get the maximum scan buffer size for returned scan results
 *
 * This API call is used to retrieve the maximum scan buffer size
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param max_buf_size return value to store scan buffer max size
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scan_buf_max_size \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the max scan buffer size.
 */
extern int qcsapi_wifi_get_scan_buf_max_size(const char *ifname, unsigned int *max_buf_size);

/**
 * \brief Set the maximum number of returned scan results
 *
 * This API call is used to set the maximum number of returned scan results
 * If the result list exceeds this number it is sorted according to
 *	the following rules prior to truncation.
 * - matched SSID
 * - WPS active
 * - WPA/RSN security
 * - High RSSI
 *
 * \param ifname \wifi0
 * \param max_table_len scan table max length
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scan_table_max_len \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_scan_table_max_len(const char *ifname, const unsigned int max_table_len);

/**
 * \brief Get the maximum number of returned scan results
 *
 * This API call is used to get the maximum number of returned scan results
 *
 * \param ifname \wifi0
 * \param max_table_len return value to store scan table max length
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scan_table_max_len \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the scan table max length
 */
extern int qcsapi_wifi_get_scan_table_max_len(const char *ifname, unsigned int *max_table_len);

/*APIs relating to scanning channels.*/
/**
 * \brief Set dwell times
 *
 * This API sets minimum and maximum active and passive channel dwell times used when scanning.
 *
 * \param ifname \wifi0
 * \param max_dwell_time_active_chan Maximum dwell time for active scans
 * \param min_dwell_time_active_chan Minimum dwell time for active scans
 * \param max_dwell_time_passive_chan Maximum dwell time for passive scans
 * \param min_dwell_time_passive_chan Maximum dwell time for passive scans
 *
 * All units are milliseconds.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dwell_times \<WiFi interface\> <i>max_active</i> <i>min_active</i>
 *	<i>max_passive</i> <i>min_passive</i></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_dwell_times(const char *ifname,
			const unsigned int max_dwell_time_active_chan,
			const unsigned int min_dwell_time_active_chan,
			const unsigned int max_dwell_time_passive_chan,
			const unsigned int min_dwell_time_passive_chan);

/**
 * \brief Get dwell times
 *
 * This API retrieves dwell times from the WLAN driver.
 *
 * \param ifname \wifi0
 * \param p_max_dwell_time_active_chan Result memory for maximum dwell time for active scans
 * \param p_min_dwell_time_active_chan Result memory for minimum dwell time for active scans
 * \param p_max_dwell_time_passive_chan Result memory for maximum dwell time for passive scans
 * \param p_min_dwell_time_passive_chan Result memory for maximum dwell time for passive scans
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dwell_times \<WiFi interface\></c>
 *
 * call_qcsapi will print dwell times in argument order to stdout on success, or print an error
 * message to stdout on failure.
 */
extern int qcsapi_wifi_get_dwell_times(const char *ifname,
			unsigned int *p_max_dwell_time_active_chan,
			unsigned int *p_min_dwell_time_active_chan,
			unsigned int *p_max_dwell_time_passive_chan,
			unsigned int *p_min_dwell_time_passive_chan);

/**
 * \brief Set bgscan dwell times
 *
 * This API sets active and passive channel dwell times used when background scanning.
 *
 * \param ifname \wifi0
 * \param dwell_time_active_chan dwell time for active scans
 * \param dwell_time_passive_chan dwell time for passive scans
 *
 * All units are milliseconds.
 *
 * \note bgscan dwell times should be less than regular scan dwell times.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bgscan_dwell_times \<WiFi interface\> <i>active</i> <i>passive</i></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_bgscan_dwell_times(const char *ifname,
			const unsigned int dwell_time_active_chan,
			const unsigned int dwell_time_passive_chan);

/**
 * \brief Get bgscan dwell times
 *
 * This API retrieves background scan dwell times from the WLAN driver.
 *
 * \param ifname \wifi0
 * \param p_dwell_time_active_chan Result memory for dwell time for active scans
 * \param p_dwell_time_passive_chan Result memory for dwell time for passive scans
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bgscan_dwell_times \<WiFi interface\></c>
 *
 * call_qcsapi will print dwell times in argument order to stdout on success, or print an error
 * message to stdout on failure.
 */
extern int qcsapi_wifi_get_bgscan_dwell_times(const char *ifname,
			unsigned int *p_dwell_time_active_chan,
			unsigned int *p_dwell_time_passive_chan);

/**
 * @brief Start a scan on the given wireless interface.
 *
 * This API causes the STA to scan available WiFi channels for beacons and associate with an Access
 * Point whose SSID is configured correctly.
 *
 * \param ifname \wifi0only
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_scan \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_start_scan(const char *ifname);

/**
 * @brief Cancel an ongoing scan on the given wireless interface.
 *
 * This API will cancel any ongoing WiFi channels scanning performed by the STA. It will do nothing
 * if no scanning is currently running.
 *
 * \param ifname \wifi0only
 * \param force Set to 0 to trigger scanning cancellation as soon as possible and return
 *	immediately.
 * Set to 1 to cancel scan immediately and then return.
 *
 * \return 0 if the command succeeded - scan cancellation is triggered (force=0), or scan is
 *	cancelled (force=1), or no scan was in progress.
 * \return \negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi cancel_scan \<WiFi interface\> \<force\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_cancel_scan(const char *ifname, int force);

/**
 * @brief Get scan status.
 *
 * This API is used to get the foreground scan status. Applications can use this API to poll scan
 * status and ensure scan is completed.
 *
 * \note This API only works to get the status of foreground (explicitly triggered) scans.
 * It may return unexpected values if used for background scans. When background scans are enabled,
 * instead use \ref qcsapi_wifi_wait_scan_completes.
 *
 * \param ifname \wifi0only
 * \param scanstatus return the currently scan status, 1 if a foreground scan is in progress,
 * and 0 if no foreground scan is in progress.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scanstatus \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value 1 if a foreground scan is in progress,
 * or 0 if no foreground scan is in progress.
 *
 * \sa qcsapi_wifi_wait_scan_completes
 */
extern int qcsapi_wifi_get_scan_status(const char *ifname, int *scanstatus);

/**
 * @brief Enable background scan
 *
 * \param ifname \wifi0only
 * \param enable Enable parameter, 1 means enable else 0 means disable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_bgscan \<WiFi interface\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_enable_bgscan(const char *ifname, const int enable);

/**
 * @brief get background scan status
 *
 * \param ifname \wifi0only
 * \param enable background scan enable status
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bgscan_status \<WiFi interface\></c>
 *
 * where <c>WiFi interface</c> is the primary interface,
 * Unless an error occurs, the output will be the Extender related parameter value.
 */
extern int qcsapi_wifi_get_bgscan_status(const char *ifname, int *enable);

/**
 * @brief Wait until the currently running scan has completed.
 *
 * Block the calling thread until the previously triggered scan completes.
 *
 * \param ifname \wifi0
 * \param timeout how long to wait for the scan to complete.
 *
 * If the scan has not completed in the specified timeout interval, the API will return an error
 * reporting timeout.
 *
 * This API is targeted for the STA but will also work on an AP.
 *
 * If no scan is in progress, the API will block the calling process until the timeout expires.
 *
 * \note To check whether scan is completed in RPC case, use non-block polling
 *	API <c>qcsapi_wifi_get_scan_status()</c> instead of this block API.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wait_scan_completes \<WiFi interface\> \<timeout\></c>
 *
 * \call_qcsapi_string_complete
 *
 * A timeout will be reported as <c>QCSAPI error 62: Timer expired.</c>
 */
extern int qcsapi_wifi_wait_scan_completes(const char *ifname, time_t timeout);

/**
 * \brief Start a periodic 'Dynamic Channel Selection (DCS)' scan.
 *
 * Start a periodic 'Dynamic Channel Selection (DCS)' scan.
 *
 * \param ifname \wifi0
 * \param dcs_params Refer to the structure definition for details. Any field containing a 0
 * will not be set.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi start_dcs_scan \<WiFi interface\> [ interval \<interval\> ]
 *	[ interval_sec \<interval_sec\> ] [ duration \<duration\> ] [ dwell \<dwell\> ]
 *	[ spacing \<spacing\> ][ chanlist \<chanlist\> ]</c>
 *
 * \call_qcsapi_string_complete
 */

extern int qcsapi_wifi_start_dcs_scan(const char *ifname, const qcsapi_dcs_params *dcs_params);

/**
 * \brief Stop 'Dynamic Channel Selection (DCS)' scan.
 *
 * Stop 'Dynamic Channel Selection (DCS)' scan.
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi stop_dcs_scan \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_stop_dcs_scan(const char *ifname);

/**
 * \brief Get 'Dynamic Channel Selection (DCS)' scan parameters.
 *
 * Get 'Dynamic Channel Selection (DCS)' scan parameters.
 *
 * \param ifname \wifi0
 * \param dcs_params Refer to the structure definition for details.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dcs_scan_stats \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current DCS scan parameters.
 */
extern int qcsapi_wifi_get_dcs_scan_params(const char *ifname, qcsapi_dcs_params *dcs_params);

/** @} */

/**
 * @addtogroup SecurityMisAPIs
 * @{
 */

/**
 * @brief Configure the retry backoff failure maximum count.
 *
 * Sets the number of times an association attempt can fail before backing off.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param fail_max the maximum number of failures permitted. The parameter can range from 2 to 20.
 *	The default failure value is 3.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi backoff_fail_max \<WiFi interface\> \<max failure count\></c>
 *
 * Example:
 *
 * <c>call_qcsapi backoff_fail_max wifi0_0 3</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_backoff_fail_max(const char *ifname, const int fail_max);

/**
 * @brief Configure retry backoff timeout.
 *
 * Configures the time to wait in seconds after backing off before attempting to associate again.
 *
 * \note \staonly
 *
 * \param ifname \wifi0
 * \param timeout the timeout between backoff and attempting a reconnection. Range is between 10
 *	and 300 seconds. Default value is 60 seconds.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi backoff_timeout \<WiFi interface\> \<timeout\></c>
 *
 * Example:
 *
 * <c>call_qcsapi backoff_timeout wifi0_0 60</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_backoff_timeout(const char *ifname, const int timeout);

/** @} */

/**
 * @addtogroup EngineeringAPIs
 * @{
 */

/**
 * @brief Get the current MCS rate.
 *
 * Get the current MCS rate.
 *
 * For an 11n rate, the value is a string in the format "MCSn" or "MCSnn", where n or nn is an
 * integer from 0 to 76, excluding 32.
 *
 * For an 11ac rate, the value is a string in the format \<MCSx0y\>, where x means NSS (number of
 * spatial streams) and y means MCS index.
 *
 * For an 11ax rate, the value is a string in the format \<MCSx0yy\>, where x means NSS (number of
 * spatial streams) and y means MCS index.
 *
 * If the autorate fallback option has been selected, this API will return Configuration Error.
 *
 * This API only returns an actual MCS rate if the set MCS rate API has been called to select a
 * particular MCS rate.
 *
 * \param ifname \wifi0
 * \param current_mcs_rate return parameter for storing the current MCS rate.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mcs_rate \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be an MCS index string, e.g. for 11n,
 * <c>MCS0, MCS8, MCS76</c>, for 11ac, <c>MCS100, MCS307</c> etc, or
 * <c>Configuration Error</c> if the auto rate fallback option has been selected.
 *
 * This command can return incorrect results if the rate has never been configured.
 */
extern int qcsapi_wifi_get_mcs_rate(const char *ifname,
		qcsapi_mcs_rate current_mcs_rate);

/**
 * @brief Set the MCS rate to a fixed value.
 *
 * Set the MCS rate to a fixed value.
 *
 * For an 11n rate, the value is a string in the format "MCSn" or "MCSnn", where n or nn is an
 * integer from 0 to 76, excluding 32. Leading zeros cannot be used and the string \<MCS01\>
 * cannot be used.
 *
 * For an 11ac rate, the value is a string in the format \<MCSx0y\>, where x means NSS (number of
 * spatial streams) and y means MCS index. Possible values are 100~109, 200~209, 300~309, 400~409.
 *
 * For an 11ax rate, the value is a string in the format \<MCSx0yy\>, where x means NSS (number of
 * spatial streams) and y means MCS index. Possible values are 1000~1011, 2000~2011, 3000~3011,
 * 4000~4011.
 *
 * This API cannot be used to configure auto rate fallback;
 * use the Set Option API with qcsapi_autorate_fallback as the option to select auto rate fallback.
 *
 * \note To set an 802.11n MCS on a VHT capable device, you must first set the bandwidth to 20MHz
 *	or 40MHz.
 * \sa qcsapi_wifi_set_bw
 *
 * \note This API should only be used to evaluate the performance of a particular MCS (modulation
 * and coding) index. Using it in a production application (i.e. with the end-user) can result in
 * unexpectedly poor performance, either lower than expected transfer rates or a failure to
 * associate. Use of the auto rate fallback option is strongly recommended.
 *
 * If option autorate fallback is enabled, this API will disable it as a side effect.
 *
 * \param ifname \wifi0only
 * \param new_mcs_rate the new MCS rate to use (fixed rate).
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mcs_rate \<WiFi interface\> \<MCS index string\></c>
 *
 * where \<MCS index string\> is an MCS index string.
 *
 * See the description of the API itself for the expected format of the MCS index string.
 *
 * \call_qcsapi_string_complete
 *
 * \note This command cannot be used to configure the auto rate fallback option;
 * use call_qcsapi set_option with autorate as the option for that purpose.
 */
extern int qcsapi_wifi_set_mcs_rate(const char *ifname, const qcsapi_mcs_rate new_mcs_rate);

/**
 * @brief Set pairing ID for pairing protection
 *
 * Set pairing ID for use of pairing protection
 *
 * The pairing ID is a 32 characters' string
 *
 * \param ifname \wifi0
 * \param pairing_id a 32 characters' string used for pairing protection.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pairing_id \<WiFi interface\> \<pairing ID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pairing_id(const char *ifname, const char *pairing_id);

/**
 * @brief Get pairing ID for pairing protection
 *
 * Get pairing ID which is for use of pairing protection
 *
 * The pairing ID is a 32 characters' string
 *
 * \param ifname \wifi0
 * \param pairing_id a 32 characters' string used for pairing protection.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pairing_id \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string of pairing ID.
 */
extern int qcsapi_wifi_get_pairing_id(const char *ifname, char *pairing_id);

/**
 * @brief Enable or disable pairing protection
 *
 * Enable or disable pairing protection
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param enable Enabling mode of the pairing protection.
 *	0 (disable),
 *	1 (enable and accept association when pairing ID matches), or
 *	2 (enable and deny association when pairing ID matches)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_pairing_enable \<WiFi interface\> \<pairing enable flag\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_pairing_enable(const char *ifname, const char *enable);

/**
 * @brief Get pairing protection enabled state
 *
 * Get pairing protection enabled state
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param enable a string used for enabling pairing protection.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pairing_enable \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string of enable flag
 */
extern int qcsapi_wifi_get_pairing_enable(const char *ifname, char *enable);

/**
 * @brief Enable or disable non-WPS pairing protection
 *
 * or disable non-WPS pairing protection
 *
 * \param ifname \wifi0
 * \param ctrl_state \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_non_wps_pp_enable \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_non_wps_set_pp_enable(const char *ifname, uint32_t ctrl_state);

/**
 * @brief Get non-WPS pairing protection enabled state
 *
 * Get non_WPS pairing enable flag which is for enabling pairing protection
 *
 * \param ifname \wifi0
 * \param ctrl_state a string used for getting the non WPS pairing protection status.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_non_wps_pp_enable \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the string of enable flag
 */
extern int qcsapi_non_wps_get_pp_enable(const char *ifname, uint32_t *ctrl_state);

/**
 * @brief Enable various fixes for compatibility issues with other vendor chipsets.
 *
 * Enable various fixes for compatibility issues with other vendor chipsets.
 *
 * \param ifname \wifi0
 * \param fix_param Vendor fix parameter
 * \param value \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vendor_fix \<WiFi interface\> \<fix-param\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_vendor_fix(const char *ifname, int fix_param, int value);

/**
 * @brief Convert a numeric error code to a descriptive string
 *
 * Given a numeric error code, convert to a human readable string. This wills for conventional
 * negative errno values, as well as QCSAPI negative error values (<= -1000).
 *
 * \param qcsapi_retval a negative error value to find the associated string of
 * \param error_msg memory for result storage
 * \param msglen length of <i>error_msg</i> buffer in bytes, including the null terminator
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_error_message \<errno\></c>
 *
 * where \<errno\> is a negative error value
 *
 * Output will be the requested error message, or the relevant error message if an error occurs.
 */
extern int qcsapi_errno_get_message(const int qcsapi_retval, char *error_msg, unsigned int msglen);

/**
 * \brief get vco lock detect status start/stop.
 *
 * This API get the vco lock detect function is enabled or disabled.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be the status of lock detect function enabled .
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vco_lock_detect_status \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_vco_lock_detect_mode(const char * ifname, unsigned int * p_jedecid);
/**
 * \brief set vco lock detect start/stop.
 *
 * This API set the vco lock detect function start/stop.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the output will be <c>1</c> (enabled) or <c>0</c> (disabled)
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vco_lock_detect_mode \<WiFi interface\></c>
 */
extern int qcsapi_wifi_set_vco_lock_detect_mode(const char * ifname, unsigned int * p_jedecid);

/**
 * @brief Set allowed rates
 *
 * This API will allowed to set rates on the given interface, with each value in megabits per
 * second.
 *
 * \param ifname \wifi0
 * \param rate_type set this to qcsapi_possible_rates.
 * \param current_rates the rates want to set, with each value in megabits per second.
 * \param num_rates number of rate in current_rates.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_rates \<WiFi interface\> possible_rates current_rates</c>
 * \note The each value of current_rates should be separated by space.
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_wifi_rates(const char *ifname, qcsapi_rate_type rate_type,
			const struct qcsapi_data_256bytes *current_rates, int num_rates);

/** @} */

/**
 * @addtogroup InfosetAPIs
 * @{
 */

/**
 * @brief Get selected information for an interface
 *
 * Get selected information for an interface
 *
 * \param ifname \wifi0
 * \param set_id a predefined set number, from 1 to QTNIS_IF_SET_ID_MAX
 * \param infoset \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_if_infoset \<WiFi interface\> \<set id\></c>
 *
 * Output is the contents of the info set in the following format, one line per field.
 * @code
 * <field label>                       : <value>
 * <field label>                       : <value>
 * ...
 * @endcode
 */
extern int qcsapi_wifi_get_if_infoset(const char *ifname, const qcsapi_unsigned_int set_id,
					struct qtnis_if_set *infoset);
/** @} */

/*
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * \brief Get statistics data of an interface.
 *
 * This API call is used to get interface statistics data.
 *
 * \param ifname \wifi0
 * \param stats return parameter to contain the statistics data of the interface being queried.
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the statistics data retrieved from the device for the
 * interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interface_stats \<interface name\></c>
 *
 * Unless an error occurs, the output will be the statistics data of the interface.
 */
extern int qcsapi_get_interface_stats(const char *ifname, qcsapi_interface_stats *stats);

/** @} */

/**
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * \brief Get statistics data of an interface.
 *
 * Get statistics data of an interface.
 *
 * \param ifname \wifi0
 * \param stats return parameter to contain the ext statistics data of the interface being queried.
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the statistics data retrieved from the device for the
 * interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_interface_stats \<interface name\></c>
 *
 * Unless an error occurs, the output will be the statistics data of the interface.
 */
extern int qcsapi_get_vap_extstats(const char *ifname, qcsapi_vap_extstats *stats);

/** @} */

/**
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * \brief Get latest PHY statistics data of an interface
 *
 * This API call is used to get latest PHY statistics data.
 *
 * \param ifname \wifi0
 * \param stats return parameter to contain the statistics data of the interface being queried.
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the PHY statistics data retrieved from the device for the
 * interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_phy_stats \<interface name\></c>
 *
 * Unless an error occurs, the output will be the PHY statistics data of the interface.
 */
extern int qcsapi_get_phy_stats(const char *ifname, qcsapi_phy_stats *stats);

/** @} */

/**
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * \brief Reset statistics data of an interface.
 *
 * This API call is used to reset interface statistics data.
 *
 * \param ifname \wifi0
 * \param node_index selects the node to operate, it's valid in AP mode when local_remote_flag is
 *	set to <c>QCSAPI_REMOTE_NODE</c>.
 * \param local_remote_flag use local flag to reset local all counters, use remote flag to reset all
 *	counters on the remote associated STA; set to <c>QCSAPI_LOCAL_NODE</c> or
 *	<c>QCSAPI_REMOTE_NODE</c>
 *
 * \return \positive_or_negative
 * If the call succeeded,the statistics data of the interface will be cleared.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi reset_all_stats \<WiFi interface\> \<node_index\>
 *	\<QCSAPI_LOCAL_NODE/QCSAPI_REMOTE_NODE\></c>
 *
 */
extern int qcsapi_reset_all_counters(const char *ifname, const uint32_t node_index,
					int local_remote_flag);

/** @} */

/**
 * @addtogroup FirmwareAPIs
 * @{
 */

/* U-boot information related defines */
#define UBOOT_INFO_VER			0
#define UBOOT_INFO_BUILT		1
#define UBOOT_INFO_TYPE			2
#define UBOOT_INFO_ALL			3
#define UBOOT_INFO_LARGE		0
#define UBOOT_INFO_MINI			1
#define UBOOT_INFO_TINY			2
#define MTD_DEV_BLOCK0			"/dev/mtdblock0"
#define MTD_UBOOT_VER_OFFSET		11
#define MTD_UBOOT_OTHER_INFO_OFFSET	32

/**
 * @brief Get u-boot information.
 *
 * Get u-boot information.
 *
 * \param uboot_info (0 - ver, 1 - built, 2 - type, 3 - all)
 * type can be U-boot (Mini) or U-boot (Large)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_uboot_info \<uboot info type\></c>
 *
 * Examples
 * @code
 * call_qcsapi get_uboot_info 0
 * Version: v36.7.0.2
 *
 * call_qcsapi get_uboot_info 1
 * Built: 05 June 2014 06:30:07
 *
 * call_qcsapi get_uboot_info 2
 * Type: U-boot (Mini)
 *
 * call_qcsapi get_uboot_info 3
 * Version: v36.7.0.2
 * Built  : 05 June 2014 06:30:07
 * Type   : U-boot (Mini)
 * @endcode
 */
extern int qcsapi_get_uboot_info(string_32 uboot_version, struct early_flash_config *ef_config);

/**
 * @brief Get the version of the firmware running on the device.
 *
 * This API reports the version of the firmware running on the device.
 *
 * \param firmware_version return parameter string to contain the firmware version.
 * \param version_size the size of the buffer firmware_version.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_firmware_version</c>
 *
 * Unless an error occurs, the output will be the version of the firmware currently running on the
 * device.
 */
extern int qcsapi_firmware_get_version(char *firmware_version,
					const qcsapi_unsigned_int version_size);

/**
 * @brief Update an image partition with the requested image.
 *
 * This API updates either the live or safety partition with a new image. The image is checked to be
 * for the appropriate architecture and checksummed before writing to flash.
 *
 * \param image_file path to the new firmware image in the filesystem
 * \param partition_to_upgrade either the live or safety partition
 *
 * \callqcsapi
 *
 * <c>call_qcsapi flash_image_update \<path\> {live | safety | uboot}</c>
 *
 * \call_qcsapi_string_complete
 * \note when used via RPC, timeout is 60 seconds
 */
extern int qcsapi_flash_image_update(const char *image_file,
					qcsapi_flash_partiton_type partition_to_upgrade);

/**
 * @brief Copy an image file from RC to EP
 *
 * This API transfers either kernel or bootloader image file from RC to /tmp directory of EP
 *
 * \param image_file path to the new firmware image in the filesystem
 * \param image_flags any additional flags, for future extensions
 *
 * \callqcsapi
 *
 * <c>call_qcsapi flash_image_update \<path\> \<flags\></c>
 *
 * \call_qcsapi_string_complete
 * Default binding interface for the server is pcie0.
 * This can be overwritten in /mnt/jffs2/qfts.conf file.
 */
extern int qcsapi_send_file(const char *image_file_path, const int image_flags);

/**
 * @brief Get u-boot image information.
 *
 * Get u-boot image information.
 *
 * \param uboot_info (0 - ver, 1 - built, 2 - type, 3 - all)
 * type can be U-boot (Mini) or U-boot (Large)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_uboot_info \<uboot info type\> \<uboot image name\></c>
 *
 * Examples
 * @code
 * call_qcsapi get_uboot_info 0 /etc/firmware/u-boot.bin
 * Version: v36.7.0.2
 *
 * call_qcsapi get_uboot_info 1 /etc/firmware/u-boot.bin
 * Built: 05 June 2014 06:30:07
 *
 * call_qcsapi get_uboot_info 2 /etc/firmware/u-boot.bin
 * Type: U-boot (Mini)
 *
 * call_qcsapi get_uboot_info 3 /etc/firmware/u-boot.bin
 * Version: v36.7.0.2
 * Built  : 05 June 2014 06:30:07
 * Type   : U-boot (Mini)
 * @endcode
 */
extern int qcsapi_get_uboot_img_info(string_32 uboot_version, struct early_flash_config *ef_config,
					const char *file);

/** @} */

/**
 * @addtogroup PowerAPIs
 * @{
 */

/**
 * @brief Set power save setting
 *
 * Set the current power save setting
 *
 * \param mode New power save setting. Valid values are:
 *	<li><i>QCSAPI_PM_MODE_DISABLE</i> - Disable all power saving features</li>
 *	<li><i>QCSAPI_PM_MODE_AUTO</i> - Automatic; power saving will adjust itself based on
 *		associated stations, traffic levels etc </li>
 *	<li><i>QCSAPI_PM_MODE_IDLE</i> - Force to idle state
 *	<li><i>QCSAPI_PM_MODE_SUSPEND</i> - Suspend all operations
 *
 * \return \zero_or_negative
 *
 * \note The 'on' and 'auto' keywords are synonymous.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi pm {off | on | auto | idle | suspend}</c>
 */
extern int qcsapi_pm_set_mode(int mode);

/**
 * @brief Set power save setting for Dual Ethernet
 *
 * Set the current power save setting for Ethernet ports, applied when two Ethernet ports are used
 *
 * \param mode new power save setting. Valid values are:
 *	<li><i>QCSAPI_PM_MODE_DISABLE</i> - Disable all power saving features</li>
 *	<li><i>QCSAPI_PM_MODE_AUTO</i> - Automatic; power saving will adjust itself based
 *		on associated stations, traffic levels etc </li>
 *
 * \return \zero_or_negative
 * \note The 'on' and 'auto' keywords are synonymous.
 *
 * \note Dual Ethernet power saving is only activated when two Ethernet interfaces are up.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi pm dual_emac [ {off | on | auto | suspend | idle} ]</c>
 *
 */
extern int qcsapi_pm_dual_emac_set_mode(int mode);

/**
 * @brief Get power save setting
 *
 * Get the current power save setting. This is related to the SoC power saving, not 802.11 power
 * saving.
 *
 * \param mode pointer to where the current power save setting value should be stored.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi pm</c>
 *
 * Unless an error occurs, the output will be the current value of power save setting from a list
 * {off | auto | idle | suspend}.
 */
extern int qcsapi_pm_get_mode(int *mode);

/**
 * @brief Get power save setting for Daul Ethernet
 *
 * Get the current power save setting. This is related to Dual Ethernet power saving.
 *
 * \param mode pointer to where the current power save setting value should be stored.
 *
 * \return \zero_or_negative
 *
 * \note Dual Ethernet power saving is only activated when two Ethernet interfaces are up.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi pm dual_emac</c>
 *
 * Unless an error occurs, the output will be the current value of power save setting from a list
 * {off | auto | idle | suspend}.
 */
extern int qcsapi_pm_dual_emac_get_mode(int *mode);

/**
 * @brief Get QPM level
 *
 * Get the current QPM level. This is related to the SoC power saving.
 *
 * \param qpm_level to where the current qpm level value should be stored
 *
 * Unless an error occurs, the output will be a number in the range 0 to 6.
 * \return 0 - power save disabled
 * \return 1 to 5 - the current power save level
 * \return 6 - power saving is suspended
 * \return \negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi qpm_level</c>
 */
extern int qcsapi_get_qpm_level(int *qpm_level);

#define QCSAPI_PM_MODE_DISABLE	BOARD_PM_LEVEL_FORCE_NO
#define QCSAPI_PM_MODE_AUTO	-1
#define QCSAPI_PM_MODE_IDLE	BOARD_PM_LEVEL_IDLE
#define QCSAPI_PM_MODE_SUSPEND	BOARD_PM_LEVEL_SUSPEND

/**
 * \brief set host state
 *
 * This API call is used for host CPU to inform radio module its state
 *
 * \param ifname \wifi0
 * \param host_state either '0 (exit power save) or '1' (enter power save)
 * \note This API works across all WiFi interfaces.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_host_state \<WiFi interface\> {0 | 1}</c>
 */
extern int qcsapi_set_host_state(const char *ifname, const uint32_t host_state);

/** @} */

/**
 * @addtogroup invoke_script
 * @{
 */

/**
 * \brief API of scripts for EMI testing and RF testing
 *
 * This function is used to call a script on the board. This API should be used when device
 * is configured to calstate=1 mode. The following scripts are supported.
 *
 * <b>set_test_mode</b>
 *
 * This script is used to configure the packet type.
 *
 * <c>set_test_mode \<Channel\> \<Antenna\> \<MCS Level\> \<BW\> \<Size\> \<11n signal\> \<BF\></c>
 *
 * \param Channel channel number of the center frequency
 * \param Antenna 127 - 4 chains on, 113 - chain 1 on, 116 - chain3 on, 120 - chain 4 on
 * \param MCS level MCS# of packet transmitted
 * \param BW 20 or 40 in MHz units
 * \param Size packet size in 100bytes units, smaller than 40 and greater than 0
 * \param 11n signal 2 - 11ac, 1 - 11n, 0 - 11a
 * \param BF 0 default.
 *
 * <b>send_test_packet</b>
 *
 * Start to transmit packet. Please note that before calling this script, test mode should be set by
 * script <c>set_test_mode</c>.
 *
 * <c>send_test_packet \<number\></c>
 *
 * \param number How many (number * 1000) packets will be sent
 *
 * <b>stop_test_packet</b>
 *
 * Stop sending packet.
 *
 * <b>set_tx_pow x</b>
 *
 * Set the packet output power to xdBm where x can vary depending on the front end device.
 *
 * <b>send_cw_signal</b>
 *
 * Generate CW tone for different channels for frequency offset measurement.
 *
 * <c>send_cw_signal \<channel\> \<chain\> \<CW pattern\> \<tone\> \<sideband\></c>
 *
 * \param channel channel number like 36, 40, 44, ...
 * \param chain the value are 2 separate numbers.
 *	- 0 0 - chain1
 *	- 0 2 - chain2
 *	- 1 0 - chain3
 *	- 1 2 - chain4
 * CW pattern
 *	- 0 - 625KHz with 0 dBFS power
 *	- 1 - 625KHz with -3 dBFS power
 *	- 2 - 1MHz with 0 dBFS power
 *	- 3 - 1MHz with -3dBFS power
 *
 * <b>stop_cw_signal</b>
 *
 * Stop the CW tone.
 *
 * <b>send_cw_signal_4chain</b>
 *
 * Generate CW tone for different channels and send signal using all four chains, or stop the CW
 * tone.
 *
 * <c>send_cw_signal_4chain {start \<channel\> | stop}</c>
 *
 * <b>show_test_packet</b>
 *
 * Show information about test packet.
 *
 * \param scriptname the name of the script file
 * \param param parameters used by the script
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi run_script \<scriptname\> \<parameters\></c>
 *
 * The output will be silent on success, or an error message on failure.
 *
 * Example:
 *
 * <c>call_qcsapi run_script set_test_mode 36 127 15 20 40 1 0</c>
 */
extern int qcsapi_wifi_run_script(const char *scriptname, const char *param);

/** @} */

/**
 * @addtogroup WiFiAPIs
 * @{
 */

/**
 * @brief Start/Stop test traffic.
 *
 * Start/Stop test traffic (null or qos null packets) on specified WiFi interface.
 *
 * \param ifname \wifi0
 * \param period Period interval for sending test traffic in milliseconds. 0 means disable.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi test_traffic \<start | stop\> \<period\></c>
 */
extern int qcsapi_wifi_test_traffic(const char *ifname, uint32_t period);

/**
 * \brief Add a multicast entry
 *
 * Add a static multicast entry to the forwarding table.
 *
 * \param ipaddr the IPv4 or IPv6 multicast address to be added to the table
 * \param mac the MAC address of an associated station or wired interface
 *
 * \return \zero_or_negative
 *
 * \note IGMP snooping should be disabled when using this command.
 *
 * \note The MAC address must be present in the forwarding table. Using the MAC address of an
 * associated station rather than a downstream endpoint is recommended because a downstream
 * endpoint is only added to the forwarding table when traffic is received from it.
 *
 * \note If a station disassociates, all multicast entries for the station are deleted.
 *
 * \note Entries added by using this command are not aged out of the table and must be deleted by
 * using <c>qcsapi_wifi_del_multicast</c>.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_multicast \<IP address\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_multicast(qcsapi_unsigned_int ipaddr, qcsapi_mac_addr mac);

/**
 * \brief Remove a multicast entry
 *
 * Remove a multicast entry from the forwarding table.
 *
 * \param ipaddr the IPv4 or IPv6 multicast address to be added to the table
 * \param mac the MAC address of a known endpoint
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_multicast \<IP address\> \<MAC address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_multicast(qcsapi_unsigned_int ipaddr, qcsapi_mac_addr mac);

/**
 * \brief Display multicast entries
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_multicast_list</c>
 *
 * Unless an error occurs, the output is a list of multicast IP addresses and MAC addresses. If
 * flood-forwarding is configured for a multicast IP address, the word 'flood' is printed in place
 * of the MAC address list. E.g.
 * @code
 * 225.1.2.4 00:26:86:f0:32:d5 00:1b:21:71:78:e6 00:26:86:5c:16:7e
 * 239.0.0.1 flood
 * 225.1.2.3 00:26:86:5c:16:7e
 * ...
 * @endcode
 */
extern int qcsapi_wifi_get_multicast_list(char *buf, int buflen);

/**
 * \brief Add a multicast IP address to the flood-forwarding table
 *
 * Add a multicast IP address to the flood-forwarding table.
 * Packets matching these addresses will be flood-forwarded to every interface and every associated
 * station.
 *
 * \note SSDP (239.255.255.250) and LNCB (224.0.0.0/24) packets are always flood-forwarded and do
 * not need to be added to this table.
 *
 * \param ipaddr the multicast IPv4 address to be added to the table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_ipff \<ipaddr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_ipff(qcsapi_unsigned_int ipaddr);

/**
 * \brief Remove a multicast IP address from the flood-forwarding table
 *
 * Remove a multicast IP address from the flood-forwarding table.
 *
 * \param ipaddr the multicast IPv4 address to be removed from the table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_ipff \<ipaddr\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_ipff(qcsapi_unsigned_int ipaddr);

/**
 * \brief Display the contents of the flood-forwarding table
 *
 * Display the contents of the flood-forwarding table.
 *
 * \note SSDP (239.255.255.250) and LNCB (224.0.0.0/24) packets are always flood-forwarded, even
 * if not added to this table.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ipff</c>
 *
 * Unless an error occurs, the output is a list of configured multicast IP addresses,
 * separated by newline characters.
 */
extern int qcsapi_wifi_get_ipff(char *buf, int buflen);

/**
 * \brief Get RTS threshold
 *
 * Get RTS threshold
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_parameter <c>aggr_rts</c> and
 *	<c>single_rts</c>.
 *
 * \param ifname \wifi0
 * \param rts_threshold Output parameter to contain the value of RTS threshold
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rts_threshold \<interface\></c>
 *
 * Unless an error occurs, outputs RTS threshold configured for interface
 */
extern int qcsapi_wifi_get_rts_threshold(const char *ifname, qcsapi_unsigned_int *rts_threshold);

/**
 * \brief Set RTS threshold
 *
 * Set RTS threshold
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_set_parameter <c>aggr_rts</c> and
 *	<c>single_rts</c>.
 *
 * \note Value of RTS threshold should be in the range 0 - 1048577;
 * 0 - enables RTS/CTS for every frame, 1048577 or more - disables RTS threshold
 *
 * \param ifname \wifi0
 * \param rts_threshold New value of RTS threshold
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_rts_threshold \<interface\> \<rts_threshold\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_rts_threshold(const char *ifname, qcsapi_unsigned_int rts_threshold);

/**
 * \brief set nss cap
 *
 * This API call is used to set the maximum number of spatial streams for a given interface
 *
 * \param ifname \wifi0
 * \param modulation the 802.11 MIMO modulation
 * \param nss number of spatial streams
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_nss_cap \<WiFi interface\> {ht | vht | he} \<nss\></c>
 */
extern int qcsapi_wifi_set_nss_cap(const char *ifname, const qcsapi_mimo_type modulation,
					const qcsapi_unsigned_int nss);

/**
 * \brief get nss cap
 *
 * This API call is used to get the maximum number of spatial streams for a given interface
 *
 * \param ifname \wifi0
 * \param modulation the 802.11 MIMO modulation
 * \param nss \valuebuf
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_nss_cap \<WiFi interface\> {ht | vht | he}</c>
 */
extern int qcsapi_wifi_get_nss_cap(const char *ifname, const qcsapi_mimo_type modulation,
					qcsapi_unsigned_int *nss);

/**
 * \brief Get A-MSDU status for a VAP for all Access Classes
 *
 * Get A-MSDU status for a VAP for all Access Classes.
 *
 * \param ifname \wifi0
 * \param enable \valuebuf
 *
 * From devices running more recent images, bits 4 to 7 of the value are set if A-MSDU is enabled
 * for each Access Class, as follows.
 * * bit 4 Best Effort
 * * bit 5 Background
 * * bit 6 Video
 * * bit 7 Voice
 *
 * From devices running older images, bit 0 is set if A-MSDU is enabled for all Access Classes, or
 * unset if A-MSDU is disabled.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_amsdu \<WiFi interface\><c>
 *
 * Unless an error occurs, output is in the following format.
 *
 * @code
 * BE {0 | 1}
 * BK {0 | 1}
 * VI {0 | 1}
 * VO {0 | 1}
 * @endcode
 */
extern int qcsapi_wifi_get_tx_amsdu(const char *ifname, int *enable);

/**
 * \brief Enable/Disable A-MSDU for a VAP for all Access Classes
 *
 * Enable/Disable A-MSDU for a VAP for all Access Classes.
 *
 * \param ifname \wifi0
 * \param enable 0 to disable A-MSDU, 1 to enable it
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * \note \ref qcsapi_wifi_set_tx_amsdu_per_ac can be used to enable/disable TX A-MSDU
 * separately for each Access Class.
 *
 * <c>call_qcsapi set_tx_amsdu \<WiFi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_tx_amsdu(const char *ifname, int enable);

/**
 * \brief Enable/Disable A-MSDU for a VAP per Access Class
 *
 * Enable/Disable A-MSDU for a VAP per Access Class.
 *
 * \param ifname \wifi0
 * \param be_en \enable_or_disable A-MSDU for the Best Effort Access Class
 * \param bk_en \enable_or_disable A-MSDU for the Background Access Class
 * \param vi_en \enable_or_disable A-MSDU for the Video Access Class
 * \param vo_en \enable_or_disable A-MSDU for the Voice Access Class
 *
 * \note For devices running an older image that does not support this API, a non-zero value for
 * any Access Class will cause A-MSDUs to be enabled for all Access Classes.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_tx_amsdu \<WiFi interface\> \<AC BE\> \<AC BK\> \<AC VI\> \<AC VO\></c>
 */
extern int qcsapi_wifi_set_tx_amsdu_per_ac(const char *ifname, uint8_t be_en, uint8_t bk_en,
			uint8_t vi_en, uint8_t vo_en);

/**
 * \brief get disassoc reason code
 *
 * This API call is used to get disassoc reason.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disassoc_reason \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_disassoc_reason(const char *ifname, qcsapi_unsigned_int *reason);

/**
 * \brief Block or unblock all association request for one specified BSS.
 *
 * Block one BSS.
 *
 * \param ifname wifix
 * \param flag user configuration for the specified BSS. 1: Block 0 Unblock
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi block_bss \<WiFi interface\> \<flag\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_block_bss(const char *ifname, const qcsapi_unsigned_int flag);

/**
 * \brief Check if device is in repeater mode
 *
 * Check if first radio device in system is in a repeater mode.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_verify_repeater_mode.
 *
 * \return 1 if in repeater mode
 * \return 0 if not in repeater mode
 * \return a negative value on error
 *
 * \callqcsapi
 *
 * <c>call_qcsapi verify_repeater_mode</c>
 */
extern int qcsapi_wifi_verify_repeater_mode(void);

/**
 * \brief Check if device is in repeater mode
 *
 * Check if specified radio device is in a repeater mode.
 *
 * \param radio_id \radio_id
 *
 * \return 1 if in repeater mode
 * \return 0 if not in repeater mode
 * \return a negative value on error
 *
 * \callqcsapi
 *
 * <c>call_qcsapi verify_repeater_mode \<radio_id\></c>
 */
extern int qcsapi_radio_verify_repeater_mode(const qcsapi_unsigned_int radio_id);

/**
 * \internal
 * \brief Configure the AP interface name
 *
 * Configure the AP interface name for AP mode or primary AP interface name for repeater mode.
 *
 * \note The new name does not take effect until configuration is reloaded.
 *
 * \param ifname the new interface name for AP mode
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ap_interface_name <interface name></c>
 */
extern int qcsapi_wifi_set_ap_interface_name(const char *ifname);

/**
 * \brief Get the AP interface name for first radio device
 *
 * This API gets interface name for AP mode or primary AP interface name for repeater mode on first
 * radio device in system.
 *
 * \note \deprecate_and_replace \ref qcsapi_radio_get_ap_interface_name.
 *
 * \param ifname the AP interface name returned
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ap_interface_name</c>
 */
extern int qcsapi_wifi_get_ap_interface_name(char *ifname);

/**
 * \brief Get the AP interface name
 *
 * Get the interface name for AP mode or primary AP interface name for repeater mode.
 *
 * \param radio_id \radio_id
 * \param ifname the AP interface name returned
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ap_interface_name \<radio\></c>
 */
extern int qcsapi_radio_get_ap_interface_name(const qcsapi_unsigned_int radio_id, char *ifname);

/**
 * \brief set preferred band 2.4/5G
 *
 * This API call is used to set preferred band between 2.4ghz or 5ghz.
 *
 * \param ifname \wifi0
 * \param pref_band The preferred band, from one of the values defined in @ref qcsapi_pref_band
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * \note This API can only be used on a dual band radio in Station mode.
 *
 * <c>call_qcsapi set_pref_band \<WiFi interface\> {2.4ghz | 5ghz}</c>
 */
extern int qcsapi_wifi_set_pref_band(const char *ifname, const qcsapi_unsigned_int pref_band);

/**
 * \brief get preferred band 2.4/5G
 *
 * This API call is used to get preferred band set between 2.4ghz or 5ghz.
 *
 * \param ifname \wifi0
 *
 * \return >= 0/1 on success, < 0 on error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_pref_band \<WiFi interface\></c>
 *
 * \note This API can only be used on a dual band radio in Station mode.
 *
 * Unless an error occurs, the output will be the string <c>2.4ghz</c> or <c>5ghz</c>.
 */
extern int qcsapi_wifi_get_pref_band(const char *ifname, qcsapi_unsigned_int *pref_band);

/**
 * \brief Set TX BA disable configuration for an SSID.
 *
 * \param ifname \wifi0
 * \param disable or enable TX BA
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi txba_disable \<WiFi interface\> \<disable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_txba_disable(const char *ifname, const qcsapi_unsigned_int value);

/**
 * \brief Get TX BA disable state for an SSID.
 *
 * \param ifname \wifi0
 * \param *enable Pointer to return buffer for storing the TX BA disable state
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_txba_disable \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the TXBA disable status for the specific SSID.
 */
extern int qcsapi_wifi_get_txba_disable(const char *ifname, qcsapi_unsigned_int *value);

/**
 * \brief Set RX BA decline configuration for an SSID.
 *
 * \param ifname \wifi0
 * \param decline or permit RX BA
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi rxba_decline \<WiFi interface\> \<decline\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_rxba_decline(const char *ifname, const qcsapi_unsigned_int value);

/**
 * \brief Get RX BA decline state for an SSID.
 *
 * \param ifname \wifi0
 * \param *enable Pointer to return buffer for storing the RX BA decline state
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_rxba_decline \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the RXBA decline status for the specific SSID.
 */
extern int qcsapi_wifi_get_rxba_decline(const char *ifname, qcsapi_unsigned_int *value);

/**
 * \brief Set Tx burst state for a BSS.
 *
 * \note This API will not currently set the TX burst for the specified BSS. It is a stub only.
 *
 * \param ifname \wifi0
 * \param enable Enable or disable Tx burst
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_txburst \<WiFi interface\> \<enable\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_txburst(const char *ifname, const qcsapi_unsigned_int enable);

/**
 * \brief Get Tx burst state for a BSS.
 *
 * \note This API will always return 1 currently. It is a stub only.
 *
 * \param ifname \wifi0
 * \param *enable Pointer to return buffer for storing the txburst state
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_txburst \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the tx burst status for the specified BSS.
 */
extern int qcsapi_wifi_get_txburst(const char *ifname, qcsapi_unsigned_int *enable);

/**
 * \brief get secondary channel of the assigned channel
 *
 * \param ifname \wifi0
 * \param chan assigned channel
 * \param p_sec_chan returned secondary channel
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_sec_chan \<WiFi interface\> \<chan\><c>
 */
extern int qcsapi_wifi_get_sec_chan(const char *ifname, int chan, int *p_sec_chan);

/**
 * \brief set secondary channel of the assigned channel
 *
 * \param ifname \wifi0
 * \param chan assigned channel
 * \param offset secondary channel offset, 0 means above and 1 means below
 * 2 means auto select secondary channel offset use previous scan result;
 * 3 means trigger a channel scan and then auto select secondary channel offset(it will take 7-8
 * seconds).
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_sec_chan \<WiFi interface\> \<chan\> \<offset\></c>
 */
extern int qcsapi_wifi_set_sec_chan(const char *ifname, int chan, int offset);

/**
 * \brief Control per-node TX airtime accumulation start and stop
 *
 * This API configures per-node TX airtime accumulation start and stop
 *
 * \param ifname interface name
 * \param node_index node index
 * \param control start or stop accumulation

 * \return \zero_or_negative

 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_airtime \<interface name\> {\<node_index\> | all} {start | stop}</c>
 */
extern int qcsapi_wifi_node_tx_airtime_accum_control(const char *ifname, const uint32_t node_index,
							qcsapi_airtime_control control);

/**
 * \brief Control TX airtime accumulation start and stop
 *
 * This API configures TX airtime accumulation start and stop for all nodes
 *
 * \param ifname interface name
 * \param control start or stop accumulation
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_airtime \<interface name\> \<node_index | all\> [start | stop]</c>
 */
extern int qcsapi_wifi_tx_airtime_accum_control(const char *ifname,
						qcsapi_airtime_control control);

/**
 * \brief Get per-node TX airtime stats
 *
 * This API gets per-node TX airtime statistics
 *
 * \param ifname interface name
 * \param node_index node index
 * \param node_tx_airtime current airtime and cumulative airtime
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_airtime \<interface name\> \<node_index | all\> [start | stop]</c>
 */
extern int qcsapi_wifi_node_get_tx_airtime(const char *ifname, const uint32_t node_index,
						qcsapi_node_tx_airtime *node_tx_airtime);

/**
 * \brief Get TX airtime stats
 *
 * This API gets TX airtime statistics for all nodes associated
 *
 * \param ifname interface name
 * \param buffer A pointer to the buffer for storing the returned value.
 *	The buffer is composed of fixed 2-byte nr_assoc_nodes, 2-byte free airtime
 *	and variable data, where variable data is nr_assoc_nodes * [2-byte node_idx,
 *	6-byte node_mac, 4-byte airtime and 4-byte airtime_accum]
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_tx_airtime \<interface name\> \<node_index | all\> [start | stop]</c>
 */
extern int qcsapi_wifi_get_tx_airtime(const char *ifname, struct qcsapi_data_3Kbytes *buffer);

/**
 * @brief this API is used to get nodes or clients macs behind asssociated QTN node.
 *
 * Get a list of mac addresses of nodes behind associated station.
 *
 * \param ifname \wifi0
 * \param index index of the associated node.
 * \param mac_address_list buffer to store the returned macaddress.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_client_mac_list \<WiFi interface\> \<index\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_get_client_mac_list(const char *ifname, int index,
					struct qcsapi_mac_list *mac_address_list);

/**
 * @brief this API is used to sample all client datas
 *
 * To sample all conected station records
 *
 * \param ifname wireless interface name
 * \param sta_count pointer for number of clients
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi sample_all_clients \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_sample_all_clients(const char *ifname, uint8_t *sta_count);

/**
 * @brief this API is used to get sampled data
 *
 * To get the record of conected station
 *
 * \param ifname wireless interface name
 * \param ptr client records pointer
 * \param num_entry station count
 * \param offset station offset
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_sampled_assoc_data \<WiFi interface\> \<num entry\> \<offset\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_get_per_assoc_data(const char *ifname,
				struct qcsapi_sample_assoc_data *ptr,
				const int num_entry, const int offset);

/**
 * \brief Get IP DSCP to WMM AC mapping table entries
 *
 * Returns the current mapping table from IP DSCP to WME AC user priority
 *
 * \param tsel table index
 * \param mapping_table Return value to contain the mapping table for WME AC user priorities.
 * Must be a memory area large enough to contain 64 byte values.
 *
 * \return \zero_or_negative
 *
 * Return parameter to contain the mapping table for WME AC user priorities
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_ac_table \<0-7\></c>
 */
extern int qcsapi_wifi_get_dscp_ac_table(int tsel, struct qcsapi_data_64bytes *mapping_table);

/**
 * \brief Set one mapping table item from IP DSCP to WMM AC priority.
 *
 * \param tsel Table Index
 * \param dscp_list List of IP DSCP value(s) to be set. Value ranges from 0 to 63.
 * If more than one IP DSCP mapping is specified, the values must be separated by commas.
 * \param dcsp_list_len The number of IP DSCP values in the dscp_list argument.
 * \param ac the WME AC value that will be mapped to the input IP DSCP. Value ranges from 0 to 3,
 *	see @ref mygroup5_4 "Quality of Service (QoS) extensions" for correspondence between AC
 *	index and AC symbolic name.
 * \return \zero_or_negative
 *
 * \note Any DSCP mappings not explicitly set will map to AC_BE.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dscp_ac_table \<0-7\> \<0-64\>[,0-64]... \<0-3\></c>
 *
 * As an example, to map the following:
 *
 * CS0,CS1,CS4,CS7 (map to AC_BE);
 *
 * AF11,CS2, AF21,CS3,CS6 (map to AC_VI); and
 *
 * CS5, EF (map to AC_VO) for table 1
 *
 * the following set of call_qcsapi calls are made:
 *
 * @code
 * call_qcsapi set_dscp_ac_table 1 0,8,32,56 0
 * call_qcsapi set_dscp_ac_table 1 10,16,18,24,48 2
 * call_qcsapi set_dscp_ac_table 1 40,46 3
 * @endcode
 */
extern int qcsapi_wifi_set_dscp_ac_table(int tsel, const struct qcsapi_data_64bytes *dscp_list,
		uint8_t dscp_list_len, uint8_t ac);

/**
 * \brief Get IP DSCP to TID mapping table entries
 *
 * Returns the current mapping table from IP DSCP to TID.
 *
 * \param tsel table index
 * \param mapping_table Return value to contain the DSCP to TID mapping table.
 * Must be a memory area large enough to contain 64 byte values.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_tid_table \<0-7\></c>
 */
extern int qcsapi_wifi_get_dscp_tid_table(int tsel, struct qcsapi_data_64bytes *mapping_table);

/**
 * \brief Set one mapping table item from IP DSCP to TID
 *
 * \param tsel Table Index
 * \param dscp_list List of IP DSCP value(s) to be set. Value ranges from 0 to 63.
 * If more than one IP DSCP mapping is specified, the values must be separated by commas.
 * \param dcsp_list_len The number of IP DSCP values in the dscp_list argument.
 * \param tid the TID value that will be mapped to the input IP DSCP. Value ranges from 0 to 7.
 *
 * \return \zero_or_negative
 *
 * \note The tables configured by this API are the same as configured by
 * \link qcsapi_wifi_set_dscp_ac_table \endlink but the latter selects optimal TID
 * for a specific AC and therefore it is recommended to use \ref qcsapi_wifi_set_dscp_ac_table.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dscp_tid_table \<0-7\> \<0-64\>[,0-64]... \<0-7\></c>
 *
 */
extern int qcsapi_wifi_set_dscp_tid_table(int tsel, const struct qcsapi_data_64bytes *dscp_list,
		uint8_t dscp_list_len, uint8_t tid);

/**
 * \brief Get VAP's IP DSCP to UP mapping table index
 *
 * Returns the current DSCP2UP mapping table index for VAP
 *
 * \param tsel table index
 *
 * \return \zero_or_negative
 *
 * Return parameter to contain the mapping table index for VAP
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_dscp_vap_link \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_dscp_vap_link(const char *ifname, uint8_t *tsel);

/**
 * \brief Set VAP's mapping table index for IP DSCP to WMM.
 *
 * \param tsel Table Index
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_dscp_ac_table \<WiFi interface\> \<0-7\></c>
 */
extern int qcsapi_wifi_set_dscp_vap_link(const char *ifname, uint8_t tsel);

/**
 * \brief Enable/disable IEEE802.11r support
 *
 * This API call is used to enable/disable the "ieee80211r" hostapd configuration
 * for a given interface. This setting enables or disables FT protocol per BSS
 * (only on WPA2-PSK and WPA2-enterprise modes).
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ieee80211r \<WiFi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_ieee80211r(const char *ifname, char *value);

/**
 * \brief Enable/disable IEEE802.11r support
 *
 * This API call is used to enable/disable the "ieee80211r" hostapd configuration
 * for a given interface. This setting enables or disables FT protocol per BSS
 * (only on WPA2-PSK and WPA2-enterprise modes).
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_ieee80211r \<wifi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_ieee80211r_str(const char *ifname, const char *value);

/**
 * \brief Get ieee80211r configuration value from hostapd.conf
 *
 * This API call is used to get the "ieee80211r" value from the
 * current hostapd configurations for a given interface.
 * This value indicates whether FT protocol is enabled/disabled per BSS.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_value is the value read from the current hostapd configuration file
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_ieee80211r \<wifi interface\></c>
 */
extern int qcsapi_wifi_get_ieee80211r(const char *ifname, string_16 p_value);

/**
 * \brief Set Mobility Domain identifier [IEEE802.11r]
 *
 * This API is used to set the 2-byte Mobility Domain id (cluster of APs participating in FT
 * roaming) in the current hostapd configuration file.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value 2-byte Mobility Domain identifier to be set in the hostapd.conf file
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_mobility_domain \<wifi interface\> \<mobility_domain_id\></c>
 */
extern int qcsapi_wifi_set_ieee80211r_mobility_domain(const char *ifname, char *value);

/**
 * \brief Set Mobility Domain identifier [IEEE 802.11r]
 *
 * Set the 2-byte Mobility Domain ID (cluster of APs participating in FT roaming)
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value 2-byte Mobility Domain identifier, as a hex string
 *
 * \return = 0 on success, < 0 on error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_mobility_domain \<WiFi interface\> \<mobility_domain_id\></c>
 */
extern int qcsapi_wifi_set_ieee80211r_mobility_domain_str(const char *ifname,
	const string_16 value);

/**
 * \brief Get Mobility Domain identifier [IEEE 802.11r]
 *
 * Get the Mobility Domain identifier.
 * The value is the configured Mobility Domain identifier.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_value \valuebuf
 *
 * \return = 0 on success, < 0 on error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_mobility_domain \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_ieee80211r_mobility_domain(const char *ifname, string_16 p_value);

/**
 * \brief Set NAS identifier [IEEE802.11r]
 *
 * This API is used to set the "nas_identifier" ID in the current hostapd configuration file.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value NAS identifier to be set in the hostapd.conf file
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_nas_id \<WiFi interface\> \<nas_identifier\></c>
 */
extern int qcsapi_wifi_set_ieee80211r_nas_id(const char *ifname, char *value);

/**
 * \brief Set NAS identifier [IEEE802.11r]
 *
 * This API is used to set the "nas_identifier" ID in the current hostapd configuration file.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value NAS identifier to be set in the hostapd.conf file
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_nas_id \<wifi interface\> \<nas_identifier\></c>
 */
extern int qcsapi_wifi_set_ieee80211r_nas_id_str(const char *ifname, const char *value);

/**
 * \brief Get NAS identifier [IEEE802.11r]
 *
 * This API call is used to get the "nas_identifier" value from the
 * current hostapd configurations for a given interface.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_ieee80211r_nas_id_64.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_value is the value read from the current hostapd configuration file
 *
 * \return -qcsapi_deprecated
 *
 * \sa qcsapi_wifi_get_ieee80211r_nas_id_64
 */
extern int qcsapi_wifi_get_ieee80211r_nas_id(const char *ifname, string_16 p_value);

/**
 * \brief Get NAS identifier [IEEE802.11r]
 *
 * This API call is used to get the "nas_identifier" value from the
 * current hostapd configurations for a given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_value is the value read from the current hostapd configuration file
 * and the maximum length is 48 bytes.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_nas_id \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_ieee80211r_nas_id_64(const char *ifname, string_64 p_value);


/**
 * \brief Enable/disable support for FT over DS [IEEE802.11r]
 *
 * This API is used to enable/disable support for FT over DS in the current hostapd
 * configuration file.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_ft_over_ds \<wifi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_ieee80211r_ft_over_ds(const char *ifname, char *value);

/**
 * \brief Enable/disable support for FT over DS [IEEE802.11r]
 *
 * This API is used to enable/disable support for FT over DS in the current hostapd
 * configuration file.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \disable_or_enable
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_ft_over_ds \<wifi interface\> {0 | 1}</c>
 */
extern int qcsapi_wifi_set_ieee80211r_ft_over_ds_str(const char *ifname, const char *value);

/**
 * \brief Get FT over DS status [IEEE802.11r]
 *
 * This API call is used to get the "ft_over_ds" value from the
 * current hostapd configurations for a given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param p_value is the value read from the current hostapd configuration file
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_ft_over_ds \<wifi interface\></c>
 */
extern int qcsapi_wifi_get_ieee80211r_ft_over_ds(const char *ifname, string_16 p_value);

/**
 * \brief Add IEEE8802.11r neighbour [IEEE802.11r]
 *
 * This API call is used to add the 11r neighbour details in the hostapd.conf file.
 * The API expects four input parameters and adds an r0kh entry and an r1kh entry in the
 * hostapd.conf in the format below:
 *
 * @code
 * r0kh=<MAC> <NAS-Identifier> <128-bit-key>
 * r1kh=<MAC> <R1KH-ID> <128-bit-key>
 * @endcode
 *
 * \param ifname \wifi0
 * \param mac MAC address
 * \param nas_id NAS Identifier
 * \param key 128-bit key as hex string
 * \param r1kh_id destination MAC address
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_11r_neighbour \<wifi interface\> \<mac address\> \<NAS Identifier\>
 *	\<128-bit key as hex string\> \<R1KH-ID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_11r_neighbour(const char *ifname, char *mac,
				char *nas_id, char *key, char *r1kh_id);

/**
 * \brief Add IEEE8802.11r neighbour [IEEE802.11r]
 *
 * This API call is used to add the 11r neighbour details in the hostapd.conf file.
 * The API expects four input parameters and adds an r0kh entry and an r1kh entry in the
 * hostapd.conf in the format below:
 *
 * @code
 * r0kh=<MAC> <NAS-Identifier> <128-bit-key>
 * r1kh=<MAC> <R1KH-ID> <128-bit-key>
 * @endcode
 *
 * \param ifname \wifi0
 * \param mac MAC address
 * \param nas_id NAS Identifier
 * \param key 128-bit key as hex string
 * \param r1kh_id destination MAC address
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_11r_neighbour \<wifi interface\> \<mac address\> \<NAS Identifier\>
 *	\<128-bit key as hex string\> \<R1KH-ID\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_add_11r_neighbour_str(const char *ifname, const char *mac,
	const char *nas_id, const char *key, const char *r1kh_id);

/**
 * \brief Delete IEEE8802.11r neighbour [IEEE802.11r]
 *
 * This API call is used to delete an 11r neighbour (r0kh, r1kh entry) in the hostapd.conf file
 * using the mac address specified.
 *
 * \param ifname \wifi0
 * \param mac MAC address to identify the r0kh, r1kh entry to be deleted
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_11r_neighbour \<wifi interface\> \<mac address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_11r_neighbour(const char *ifname, char *mac);

/**
 * \brief Delete IEEE8802.11r neighbour [IEEE802.11r]
 *
 * This API call is used to delete an 11r neighbour (r0kh, r1kh entry) in the hostapd.conf file
 * using the mac address specified.
 *
 * \param ifname \wifi0
 * \param mac MAC address to identify the r0kh, r1kh entry to be deleted
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi del_11r_neighbour \<wifi interface\> \<mac address\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_del_11r_neighbour_str(const char *ifname, const char *mac);

/**
 * \brief get IEEE802.11r neighbour information [IEEE802.11r]
 *
 * This API call is used to get the list of 11r neighbours (r0kh, r1kh) from the hostapd.conf file.
 *
 * \param ifname \wifi0
 * \param buf to be filled with the list of 11r neighbours
 * \param buflen length of buf passed
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_neighbour \<wifi interface\></c>
 */
extern int qcsapi_wifi_get_11r_neighbour(const char *ifname, char *buf, int buflen);

/**
 * \brief set R1 key holder ID [IEEE802.11r]
 *
 * This API call is used to set the R1 key holder ID value in the hostapd.conf file
 * in the below format:
 * \n r1_key_holder=<6-octet identifier as hex string>
 *
 * \param ifname \wifi0
 * \param value will be used to set the r1_key_holder ID.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_r1_key_holder \<wifi interface\> \<r1_key_holder\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_11r_r1_key_holder(const char *ifname, char *value);

/**
 * \brief set R1 key holder ID [IEEE802.11r]
 *
 * This API call is used to set the R1 key holder ID value in the hostapd.conf file
 * in the below format:
 * \n r1_key_holder=<6-octet identifier as hex string>
 *
 * \param ifname \wifi0
 * \param value will be used to set the r1_key_holder ID.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_r1_key_holder \<wifi interface\> \<r1_key_holder\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_11r_r1_key_holder_str(const char *ifname, const char *value);

/**
 * \brief get R1 key holder ID [IEEE802.11r]
 *
 * This API call is used to read the r1_key_holder ID value from the hostapd.conf file.
 *
 * \param ifname \wifi0
 * \param p_value to be filled with the R1 key holder value read from the hostapd.conf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_r1_key_holder \<wifi interface\></c>
 */
extern int qcsapi_wifi_get_11r_r1_key_holder(const char *ifname, string_16 p_value);

/**
 * \brief Set the default lifetime of the IEEE 802.11r PMK-R0
 *
 * Set the default lifetime of the IEEE 80211.r PMK-R0.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \param value the PMK-R0 lifetime in seconds; between 60 and 4294967295 (default is 1209600
 *	seconds)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_11r_r0_key_lifetime \<wifi interface\> \<r0_key_lifetime\</c>
 */
extern int qcsapi_wifi_set_11r_r0_key_lifetime(const char *ifname, const char *value);

/**
 * \brief Get R0 KEy lifetime [IEEE802.11r]
 *
 * This API call is used to get the "r0_key_lifetime" value from the
 * current hostapd configurations for a given interface.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param value \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_11r_r0_key_lifetime \<wifi interface\></c>
 */
extern int qcsapi_wifi_get_11r_r0_key_lifetime(const char *ifname, string_64 p_value);

/**
 * @brief Get various type of channel list supported on an interface.
 *
 * Get various type of channel lists supported in all frequency bands on an interface
 *
 * \param ifname \wifi0only
 * \param chan_list \valuebuf
 * \param flags represents the type of channel list. See \ref qcsapi_chlist_flags.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_chan_list \<WiFi interface\> [ {available | disabled | scan} ]</c>
 *
 * \note If the API is called without any argument, output will be the 'available' channel list.
 *
 * Unless an error occurs, the output will be the list of WiFi channels per the 802.11 standard.
 */
extern int qcsapi_wifi_get_chan_list(const char *ifname, struct qcsapi_data_256bytes *chan_list,
		const uint32_t flags);

/**
 * \brief Get the scan channel list.
 *
 * Get the list of target channels when doing scan.
 * \note \deprecate_and_replace \ref qcsapi_wifi_get_chan_list.
 *
 * \param ifname \wifi0only
 * \param chanlist buffer used to get the list of channel numbers, one byte contains one channel
 * \param p_count buffer used to get the number of channels in scan channel list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_scan_chan_list \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the list of WiFi channels numbers.
 *
 * \sa qcsapi_wifi_set_scan_chan_list
 */
extern int qcsapi_wifi_get_scan_chan_list(const char *ifname,
				struct qcsapi_data_256bytes *chanlist, uint32_t *p_count);

/**
 * \brief Set scan channel list.
 *
 * Set the list of target channels when doing scan, scanner will scan the channels contained in
 * this scan channel list if channel is supported and allowed to be scanned.
 * \note The scan channel list won't take effect for DCS scan on AP mode.
 * \note The scan channel list will be restored to all available channels when parameter chanlist
 * is NULL and parameter count is 0.
 *
 * \param ifname \wifi0only
 * \param chanlist buffer to contain the list of channel numbers, one byte contains one channel
 * \param count number of channels in the list
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_scan_chan_list \<WiFi interface\> default | \<channel list\></c>
 *
 * \call_qcsapi_string_complete
 *
 * \sa qcsapi_wifi_get_scan_chan_list
 * \sa qcsapi_wifi_start_dcs_scan
 */
extern int qcsapi_wifi_set_scan_chan_list(const char *ifname,
			const struct qcsapi_data_256bytes *chanlist, const uint32_t count);

/**
 * \brief Get the required phymode capability allowed for association.
 *
 * Get the required phymode capability of peer STA that is allowed for association
 *
 * \param ifname \wifi0
 * \param phymode_required parameter to contain the phymode required for the interface
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_phy_mode_required \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 *
 */
extern int qcsapi_wifi_get_phy_mode_required(const char *ifname, char *phymode_required);

/**
 * \brief Set the required phymode capability allowed for association.
 *
 * Set the required phymode capability of peer STA that is allowed for association, if a peer STA
 * doesn't support the phymode as required, then it's not allowed for association.
 *
 * \param ifname \wifi0
 * \param phymode_required a string that contains the required phymode: "none", "11n", "11ac" or
 *	"11ax"
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_phy_mode_required \<WiFi interface\> {none | 11n | 11ac | 11ax}</c>
 *
 * \call_qcsapi_string_complete
 *
 */
extern int qcsapi_wifi_set_phy_mode_required(const char *ifname, const char *phymode_required);

/** @} */

/**
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * @brief Get RFIC temperature
 *
 * Get RFIC temperature
 *
 * \note \deprecate_and_replace \ref qcsapi_get_temperature_info_ext.
 *
 * \param temp_exter Buffer to contain the returned RFIC external temperature.
 * \param temp_inter Buffer to contain the returned RFIC internal temperature.
 * \param temp_bbic Buffer to contain the returned BBIC temperature.
 *
 * \return \zero_or_negative
 *
 * \note The return value is a fixed point number, and the actual internal/external temperature
 *	value (in degrees Celsius) can be obtained using the following code snippet.
 *
 * @code
 * float rfic_temperature_ext, rfic_temperature_int, bbic_temperature;
 * rfic_temperature_ext = temp_exter / 100000.0f;
 * rfic_temperature_int = temp_inter / 1000000.0f;
 * bbic_temperature = temp_bbic / 1000000.0f;
 * @endcode
 */
extern int qcsapi_get_temperature_info(int *temp_exter, int *temp_inter, int *temp_bbic);

/**
 * @brief Get RFIC and BBIC temperatures
 *
 * Get RFIC and BBIC temperatures
 *
 * \param tempsens_in Pointer to a struct qcsapi_array_int32 which specifies the type of
 *	information to be returned and any further qualifying values. The value type(s) are
 *	defined in \ref qcsapi_temperature_info_type, and should be set in tempsens_in.val[0].
 * \param tempsens_out Pointer to a struct qcsapi_array_int32 to hold return temperatures.
 *
 * \return \zero_or_negative
 *
 * \note The return values in the array are indexed by a qcsapi_temperature_index value. Not all
 * temperatures are available on all boards, the value returned for unavailable temperatures is
 * QDRV_TEMPSENS_INVALID.
 *
 *
 * \note The return value is a fixed point number, and the actual internal/external temperature
 * value (in degrees Celsius) can be obtained using the following code snippet.
 * @code
 * float rfic_temperature_ext, rfic_temperature_int, bbic_temperature_int, bbic_temperature_ext;
 * rfic_temperature_ext = temps.arr[QCSAPI_TEMP_INDEX_RFIC_EXT] / 100000.0f;
 * rfic_temperature_int = temps.arr[QCSAPI_TEMP_INDEX_RFIC_INT] / 1000000.0f;
 * bbic_temperature_int = temps.arr[QCSAPI_TEMP_INDEX_BBIC_INT] / 1000000.0f;
 * bbic_temperature_ext = temps.arr[QCSAPI_TEMP_INDEX_BBIC_EXT] / 1000000.0f;
 * @endcode
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_temperature</c>
 *
 */
extern int qcsapi_get_temperature_info_ext(const struct qcsapi_int_array32 *tempsens_in,
						struct qcsapi_int_array32 *tempsens_out);

/** @} */

/**
 * @addtogroup CalcmdAPI
 * @{
 */

/**
 * \brief QCSAPI for calcmd SET_TEST_MODE
 *
 * \param ifname \wifi0
 * \param value value to set. The param belonging to each value is predefined.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_test_mode calcmd \<value:channel\> \<value:antenna\> \<value:MCS\>
 *	\<value:BW\> \<value:Packet size\> \<value:11N\> \<value:beamforming\></c>
 *
 * Output is silent with a return code of zero if successful, and an error message on failure.
 *
 * Example for channel 36, 4 antenna, MCS 15, HT40, 4Kbytes, 11N Signal, bf off.
 *
 * <c>call_qcsapi set_test_mode calcmd 36 127 15 40 40 1 0</c>
 */
extern int qcsapi_calcmd_set_test_mode(qcsapi_unsigned_int channel, qcsapi_unsigned_int antenna,
			qcsapi_unsigned_int mcs, qcsapi_unsigned_int bw,
			qcsapi_unsigned_int pkt_size, qcsapi_unsigned_int eleven_n,
			qcsapi_unsigned_int bf);

/**
 * \brief Show TX packet number, RX packet number and CRC number
 *
 * This API call is used to show test mode packet statistics.
 *
 * \param ifname \wifi0
 * \param stats return parameter to contain the statistics of TX, RX, CRC packet number.
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the packet number statistics data retrieved from the device
 * for the interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi show_test_packet calcmd</c>
 *
 * Unless an error occurs, the output will be the PHY statistics data of the interface.
 */

extern int qcsapi_calcmd_show_test_packet(qcsapi_unsigned_int *tx_packet_num,
		qcsapi_unsigned_int *rx_packet_num, qcsapi_unsigned_int *crc_packet_num);

/**
 * \brief Start sending OFDM Packet
 *
 * This API call is used to send OFDM packet.
 *
 * \param ifname \wifi0
 * \param value the number of packet to transmit, 1 is 1000 packets. If number is set, it will
 *	automatically stop, if 0, transmit infinitely
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the packet number statistics data retrieved from the device
 * for the interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi send_test_packet calcmd \<value\></c>
 *
 * Output is silent with a return code of zero if successful, and an error message on failure.
 */

extern int qcsapi_calcmd_send_test_packet(qcsapi_unsigned_int to_transmit_packet_num);

/**
 * \brief Stop transmitting OFDM Packet
 *
 * This API call is used to stop transmitting OFDM packet in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi stop_test_packet calcmd</c>
 *
 * Output is silent with a return code of zero if successful, and an error message on failure.
 */

extern int qcsapi_calcmd_stop_test_packet(void);

/**
 * \brief send CW signal at center frequency for Frequency offset measurement
 *
 * This API call is used to send continuous signal in test mode.
 *
 * \param ifname \wifi0
 *
 * \param channel to perform the action on. (calcmd)
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi send_dc_cw_signal calcmd \<channel\></c>
 *
 * Output is silent with a return code of zero if successful, and an error message on failure.
 */

extern int qcsapi_calcmd_send_dc_cw_signal(qcsapi_unsigned_int channel);

/**
 * \brief stop transmitting CW signal
 *
 * This API call is used to stop Continuous signal in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi stop_dc_cw_signal calcmd</c>
 *
 * Output is silent with a return code of zero if successful, and an error message on failure.
 */

extern int qcsapi_calcmd_stop_dc_cw_signal(void);

/**
 * \brief get antenna selection
 *
 * This API call is used to retrieve antenna configuration in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_test_mode_antenna_sel calcmd</c>
 *
 * Output is antenna number with bit mask type. e.g) 1010 means Ant4 & Ant2 is enabled.
 */

extern int qcsapi_calcmd_get_test_mode_antenna_sel(qcsapi_unsigned_int *antenna_bit_mask);

/**
 * \brief get mcs config
 *
 * This API call is used to retrieve MCS configuration in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_test_mode_mcs calcmd</c>
 *
 * Output is MCS configuration.
 */

extern int qcsapi_calcmd_get_test_mode_mcs(qcsapi_unsigned_int *test_mode_mcs);


/**
 * \brief get bandwidth config
 *
 * This API call is used to retrieve bandwidth configuration in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_test_mode_bw calcmd</c>
 *
 * Output is Bandwidth configuration.
 */

extern int qcsapi_calcmd_get_test_mode_bw(qcsapi_unsigned_int *test_mode_bw);


/**
 * \brief get tx power value
 *
 * This API call is used to retrieve current TX power.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_test_mode_tx_power calcmd</c>
 *
 * Output is TX power value.
 */
extern int qcsapi_calcmd_get_tx_power(qcsapi_calcmd_tx_power_rsp *tx_power);

/**
 * \brief Set Tx power for the specified interface.
 *
 * This API call is used to set TX power for the specified interface.
 *
 * \replaced_by \ref qcsapi_radio_calcmd_set_tx_power.
 *
 * \param tx_power Tx power value to set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_test_mode_tx_power \<ifname\> \<tx_power\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_calcmd_set_tx_power(qcsapi_unsigned_int tx_power);

/**
 * \brief Set Tx power for the specified interface.
 *
 * This API call is used to set TX power for the specified interface.
 *
 * \param ifname \wifi0only
 * \param tx_power Tx power value to set
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_test_mode_tx_power \<ifname\> \<tx_power\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_radio_calcmd_set_tx_power(const char *ifname, qcsapi_unsigned_int tx_power);

/**
 * \brief get RSSI value
 *
 * This API call is used to retrieve RSSI value in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_test_mode_rssi calcmd</c>
 *
 * Output is RSSI value.
 */

extern int qcsapi_calcmd_get_test_mode_rssi(qcsapi_calcmd_rssi_rsp *test_mode_rssi);


/**
 * \brief set mac filter
 *
 * This API call is used to set mac filter.
 *
 * \param q_num the contention queue number
 *
 * \param sec_enable the security if enable or not
 *
 * \param mac_addr the mac address which the device is used to filter the packet
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi calcmd_set_mac_filter calcmd \<q_num\> \<sec_enable\> \<mac_addr\></c>
 *
 * \call_qcsapi_string_complete
 *
 * Example:
 *
 * <c>call_qcsapi calcmd_set_mac_filter calcmd 0 2 00:11:22:33:44:55</c>
 */

extern int qcsapi_calcmd_set_mac_filter(int q_num, int sec_enable, const qcsapi_mac_addr mac_addr);


/**
 * \brief get number of antenna
 *
 * This API call is used to retrieve auntenna counter in test mode.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_antenna_counter calcmd</c>
 *
 * Output is number of antenna.
 */

extern int qcsapi_calcmd_get_antenna_count(qcsapi_unsigned_int *antenna_count);


/**
 * \brief clear tx/rx counter
 *
 * This API call is used to clear counter of tx/rx packets.
 *
 * \param ifname \wifi0
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi calcmd_clear_counter calcmd</c>
 */

extern int qcsapi_calcmd_clear_counter(void);

/**
 * \brief get firmware info
 *
 * This API call is used to retrieve firmware information.
 *
 * \param output_info
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_info wifi0_0</c>
 *
 * Output is firmware info.
 */
extern int qcsapi_calcmd_get_info(string_1024 output_info);

/**
 * \brief Get real time transmit power values on all chains.
 *
 * Get real time transmit power values on all chains.
 *
 * \param ifname \wifi0
 * \param tx_power \databuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_real_time_txpower \<ifname\></c>
 *
 * Unless an error occurs, the output will be transmit power values.
 * \note The output may not be reliable sometimes. For example, no any traffic being sent.
 */
extern int qcsapi_calcmd_get_real_time_txpower(const char *ifname,
					qcsapi_calcmd_tx_power_rsp *tx_power);

/** @} */

/**
 * @addtogroup WOWLAN_group
 * @{
 */

/**
 * \brief set WOWLAN match type
 *
 * This API call is used to set which type to use to match WOWLAN packet
 *
 * \param ifname \wifi0
 * \param wowlan_match '1' (L2 ether type) or '2' (L3 UDP), default is 0 match either of them
 * \note This API works across all WiFi interfaces.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_match_type \<WiFi interface\> {0 | 1 | 2}</c>
 */
extern int qcsapi_wowlan_set_match_type(const char *ifname, const uint32_t wowlan_match);

/**
 * \brief set WOWLAN L2 ether type
 *
 * This API call is used to set the ehter type value when using L2 to do matching
 *
 * \param ifname \wifi0
 * \param ether_type ether type value. 0x0842 will be used by default.
 * \note This API works across all WiFi interfaces.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_L2_type \<WiFi interface\> \<ether type\></c>
 */
extern int qcsapi_wowlan_set_L2_type(const char *ifname, const uint32_t ether_type);

/**
 * \brief set WOWLAN L3 UDP destination port
 *
 * This API call is used to set UDP destination port when using UDP to do matching
 *
 * \param ifname \wifi0
 * \param udp_port UDP destination port value. By default 7 or 9 will be used
 * \note This API works across all WiFi interfaces.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_udp_port \<WiFi interface\> \<udp dest port\></c>
 */
extern int qcsapi_wowlan_set_udp_port(const char *ifname, const uint32_t udp_port);

/**
 * \brief set user self-defined WOWLAN match pattern
 *
 * This API call is used to set user self-defined pattern.
 *
 * \param ifname \wifi0
 * \param pattern pattern array, 256 bytes in total length.
 * Default format is 6 bytes of all 255 (FF FF FF FF FF FF in hexadecimal),
 * followed by sixteen repetitions of BSSID or host CPU's MAC address
 * \param len length of pattern
 * \note This API works across all WiFi interfaces.
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_pattern \<WiFi interface\> \<pattern\> \<len\></c>
 */
extern int qcsapi_wowlan_set_magic_pattern(const char *ifname, struct qcsapi_data_256bytes *pattern,
					uint32_t len);

/**
 * \brief Get host CPU's power save state.
 *
 * This API is used to get host CPU's power save state.
 *
 * \param ifname \wifi0
 * \param p_value Buffer contains state of host CPU, 1: host in power save state, 0 : not.
 * \param len Buffer contains the length of the return parameter value.
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_get_host_state \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the host state.
 */
extern int qcsapi_wifi_wowlan_get_host_state(const char *ifname, uint16_t *p_value, uint32_t *len);
/**
 * \brief Get WOWLAN match type.
 * This API is used to get which match type is used for current WOWLAN filtering.
 *
 * \param ifname \wifi0
 * \param p_value Buffer contains match type value, 0: default, 1: L2 ether type
 *	matching, 2: UDP port matching.
 * \param len Buffer contains the length of the return parameter value.
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_get_match_type \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the match type.
 */
extern int qcsapi_wifi_wowlan_get_match_type(const char *ifname, uint16_t *p_value, uint32_t *len);
/**
 * \brief Get WOWLAN ether type value.
 *
 * This API is used to ether type value used for current WOWLAN filtering.
 *
 * \param ifname \wifi0
 * \param p_value Buffer contains ether type value.
 * \param len Buffer contains the length of the return parameter value.
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_get_L2_type \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the ether type.
 */
extern int qcsapi_wifi_wowlan_get_l2_type(const char *ifname, uint16_t *p_value, uint32_t *len);
/**
 * \brief Get WOWLAN UDP destination port.
 *
 * This API is used to get UDP destination port value used for current WOWLAN filtering.
 *
 * \param ifname \wifi0
 * \param p_value Buffer contains udp port value.
 * \param len Buffer contains the length of the return parameter value.
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_get_udp_port \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the udp port.
 */
extern int qcsapi_wifi_wowlan_get_udp_port(const char *ifname, uint16_t *p_value, uint32_t *len);
/**
 * \brief Get WOWLAN magci pattern.
 *
 * This API is used to get magic pattern used for current WOWLAN filtering.
 *
 * \param ifname \wifi0
 * \param p_value Buffer contains magic pattern in the format of "010203998877".
 * \param len Buffer contains the length of the return parameter value.
 * \return negative value on error, 0 on success.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi wowlan_get_pattern \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the value of the
 * magic pattern and its length.
 */
extern int qcsapi_wifi_wowlan_get_magic_pattern(const char *ifname,
					struct qcsapi_data_256bytes *p_value, uint32_t *len);

/** @} */

/**
 * @addtogroup MU_group
 * @{
 */

/**
 * \brief Enable/disable MU-MIMO functionality on AP
 *
 * This API call is used to enable/disable MU-MIMO functionality on AP
 *
 * \note This API only applies for an AP.
 *
 * \param ifname \wifi0
 * \param mu_enable 1 to enable MU-MIMO, 0 to disable it
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_enable_mu \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_enable_mu(const char *ifname, const unsigned int mu_enable);

/**
 * \brief Get the status of MU-MIMO, if it is enabled or not
 *
 * This API call is used to get the the status of MU-MIMO
 *
 * \param ifname \wifi0
 * \param mu_enable return value storing a flag showing if MU-MIMO is enabled or not
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_enable_mu \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the MU-MIMO enable flag
 */
extern int qcsapi_wifi_get_enable_mu(const char *ifname, unsigned int * mu_enable);

/**
 * \brief Enable/disable MU-MIMO precoding matrix for the group on AP
 *
 * This API call is used to enable/disable MU-MIMO precoding matrix for the group
 *
 * \param ifname \wifi0
 * \param grp MU-MIMO group ID, the valid range is [1-62]
 * \param prec_enable 1 to enable MU-MIMO precoding matrix for the group, 0 to disable it
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mu_use_precode \<WiFi interface\> \<group\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_mu_use_precode(const char *ifname, const unsigned int grp,
		const unsigned int prec_enable);

/**
 * \brief Get the status of MU-MIMO precoding matric for the group, if it is enabled or not
 *
 * This API call is used to get the the status of MU-MIMO precoding matrix
 *
 * \note This API only applies for an AP.
 *
 * \param ifname \wifi0
 * \param grp MU-MIMO group ID, the valid range is [1-62]
 * \param prec_enable return value storing a flag showing if MU-MIMO precoding matrix is
 * enabled or not
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mu_use_precode \<WiFi interface\> \<group\></c>
 *
 * Unless an error occurs, the output will be the MU-MIMO enable flag
 */
extern int qcsapi_wifi_get_mu_use_precode(const char *ifname, const unsigned int grp,
	unsigned int * prec_enable);

/**
 * \brief Enable/disable MU equalizer on STA
 *
 * This API call is used to enable/disable MU-MIMO equalizer on STA
 *
 * \note This API only applies for an STA.
 *
 * \param ifname \wifi0
 * \param eq_enable 1 to enable equalizer, 0 to disable it
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_mu_use_eq \<WiFi interface\> \<value\></c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_mu_use_eq(const char *ifname, const unsigned int eq_enable);

/**
 * \brief Get the status of MU equalizer, if it is enabled or not
 *
 * This API call is used to get the the status of MU equalizer
 *
 * \note This API only applies for an STA.
 *
 * \param ifname \wifi0
 * \param mu_enable return value storing a flag showing if MU equalizer is enabled or not
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mu_use_eq \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the MU equalizer enable flag
 */
extern int qcsapi_wifi_get_mu_use_eq(const char *ifname, unsigned int * meq_enable);

/**
 * \brief Get information about MU-MIMO groups formed
 *
 * This API call is used to get information about MU-MIMO groups
 *
 * \param ifname \wifi0
 * \param buf pointer to a buffer where the resulted information is placed to
 * \param size size of the buffer
 *
 * \return the number of bytes returned in the buf, or a negative value on error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_mu_groups \<WiFi interface\></c>
 *
 * Unless an error occurs, the output on AP will be MU groups information in the following form
 *GRP ID: 1 update cnt 304 Enabled
 * Rank: 44468
 * AID0: 0x0001 AID1: 0x0004
 * IDX0: 5 IDX1: 8
 * u0_1ss_u1_1ss: 0x0
 * u0_2ss_u1_1ss: 0x21
 * u0_3ss_u1_1ss: 0x52
 * u0_1ss_u1_2ss: 0x93
 * u0_1ss_u1_3ss: 0xc4
 * u0_2ss_u1_2ss: 0x105
.* The same table is repeated for each existing groups group
 * For the STA the output will be
 * AP GRP ID: 1 update cnt 0
 * User pos = 0 with AID = 0x0004
 * The same table is reapeated for every group the STA belongs to
 */
extern int qcsapi_wifi_get_mu_groups(const char *ifname, char * buf, const unsigned int size);

/** @} */

/**
 * @addtogroup QWEAPIs
 * @{
 */

/**
 * @brief Invoke QWE commands.
 *
 * QWE is a Quantenna Wireless Extension Platform Abstract Layer which is introduced to control 3rd
 * party 2.4GHz. The purpose of QWE is to provide a unified interface of 2.4G interface to upper
 * layer such as Web GUI.
 *
 * QWE provides two different commands "qweconfig" and "qweaction",
 * "qweconfig" will get/set 2.4G configurations without taking effect immediately.
 * "qweaction" will act some operations immediately such as take effect of 2.4G configuration, WPS
 * PBC and get all kinds of status.
 *
 * This API make QWE to be compatible with QCSAPI, by this API 3rd party 2.4G can be controlled by
 * QCSAPI RPC remotely in case QV860 is designed as a module instead of video bridge.
 *
 * \param command The QWE command to perform, it can be "qweconfig" or "qweaction" or "help".
 * \param param1 Command-dependent parameter for the QWE command.
 * \param param2 Command-dependent parameter for the QWE command.
 * \param param3 Command-dependent parameter for the QWE command.
 * \param output Address of the buffer to receive the output of the command - recommended to
 *	be 1024 bytes.
 * \param max_len Maximum number of characters that can be written to the parameter <c>output</c>
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi qwe qweconfig \<command\> [\<param\>] [\<value\>]</c>
 * <c>call_qcsapi qwe qweaction \<device\> \<command\> [\<argument\>]</c>
 * <c>call_qcsapi qwe help</c>
 */
extern int qcsapi_qwe_command(const char *command, const char *param1, const char *param2,
				const char *param3, char *output, const unsigned int max_len);

/** @} */

/**
 * @addtogroup ANDROID_group
 * @{
 */

/**
 * @brief this API is used to get scan IEs from the scan result.
 *
 * Currently this API is only used for Android support, user should not call this API explicitly.
 * And there is no call_qcsapi interface is provided for this API, only C interface is available.
 * Android wireless driver need the scan result IE for further process.
 * This API will save and send IE buf. And The IE buf will be zipped before sending to host for
 * processing.
 *
 * \note Please consult with Quantenna for the usage of this API.
 *
 * \param ifname \wifi0
 * \param buf buffer pointer for tranferring the IE.
 * \param block_id the block ID in the IE buf, each block is 1K Bytes.
 * \param ulength the IE buffer size readed from the IE, it should be less than 1K for the last
 *	block.
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the buf will contains the IE buf block</c>.
 *
 */
int qcsapi_wifi_get_scan_IEs(const char *ifname, struct qcsapi_data_1Kbytes *buf,
				uint32_t block_id, uint32_t *ulength);

/** @} */

/**
 * @addtogroup EngineeringAPIs
 * @{
 */

/**
 * @brief This API is used to get the size of the core dump that is stored across reboot.
 *
 * \param core_dump_size Size of the core dump (in bytes)
 *
 * \return \positive_or_negative
 *
 * Unless an error occurs, the buf will contains the size of the core dump</c>.
 *
 * \sa qcsapi_get_core_dump2
 */

int qcsapi_get_core_dump_size(uint32_t *core_dump_size);

/**
 * @brief Get the contents of a core dump.
 *
 * Get the contents of a core dump.
 *
 * \note \deprecate_and_replace \ref qcsapi_get_core_dump2.
 *
 * \param buf \databuf
 * \param bytes_to_copy Maximum number of bytes to be copied from the internal buffer.
 *			'buf' must be at least this big.
 * \param start_offset Offset into the internal buffer from where to start copying.
 * \param bytes_copied Buffer to contain the number of bytes that were copied.
 *		If 'bytes_copied' < 'bytes_to_copy', there is no more data in the internal
 *		buffer or an error has occurred.
 *
 * \return -qcsapi_deprecated
 *
 * \sa qcsapi_get_core_dump_size
 * \sa qcsapi_get_core_dump2
 */
extern int qcsapi_get_core_dump(string_4096 buf, uint32_t bytes_to_copy,
					uint32_t start_offset, uint32_t *bytes_copied);

/**
 * @brief Get the contents of a core dump.
 *
 * Get the contents of a core dump.
 *
 * \param buf \databuf
 * \param bytes_to_copy Maximum number of bytes to be copied from the internal buffer.
 *			'buf' must be at least this big.
 * \param start_offset Offset into the internal buffer from where to start copying.
 * \param bytes_copied Buffer to contain the number of bytes that were copied.
 *			If 'bytes_copied' < 'bytes_to_copy', there is no more data in the internal
 *			buffer or an error has occurred.
 *
 * \return \zero_or_negative
 *
 * Unless an error occurs, buf will contains the requested dump.
 *
 * \note The core dump is only available for retrieval via this API after the initial reboot
 * following a crash.
 *
 * \note The client should first invoke qcsapi_get_core_dump_size(...) to get the size of the core
 * dump. The dump can then be retrieved in one shot (bytes_to_copy = core_dump_size, start_offset =
 * 0) or in multiple chunks (bytes_to_copy = "chunk size", start_offset = "offset into the core
 * dump" by invoking qcsapi_get_core_dump2(...).
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_core_dump</c>
 *
 * Output is the core dump unless an error occurs.
 *
 * \note Since the core dump could contain non-ASCII characters, it is better to redirect
 * the output of this command to a file
 *
 * \sa qcsapi_get_core_dump_size
 */
extern int qcsapi_get_core_dump2(struct qcsapi_data_4Kbytes *buf, uint32_t bytes_to_copy,
					uint32_t start_offset, uint32_t *bytes_copied);

/**
 * @brief Set debug message mask bitmap for vap interface in wlan driver
 *
 * This API is used to set debug message mask bitmap for vap interface in wlan driver,
 * definitions for each bit of the bitmap are as below:
 * <small><table>
 * <tr><td> 0x80000000 </td><td> 11n mode debug </td></tr>
 * <tr><td> 0x40000000 </td><td> IFF_DEBUG equivalent </td></tr>
 * <tr><td> 0x20000000 </td><td> IFF_LINK2 equivalent </td></tr>
 * <tr><td> 0x10000000 </td><td> crypto work </td></tr>
 * <tr><td> 0x08000000 </td><td> input handling </td></tr>
 * <tr><td> 0x04000000 </td><td> rate set handling </td></tr>
 * <tr><td> 0x02000000 </td><td> element id parsing </td></tr>
 * <tr><td> 0x01000000 </td><td> node handling </td></tr>
 * <tr><td> 0x00800000 </td><td> association handling </td></tr>
 * <tr><td> 0x00400000 </td><td> authentication handling </td></tr>
 * <tr><td> 0x00200000 </td><td> scanning </td></tr>
 * <tr><td> 0x00100000 </td><td> output handling </td></tr>
 * <tr><td> 0x00080000 </td><td> state machine </td></tr>
 * <tr><td> 0x00040000 </td><td> power save handling </td></tr>
 * <tr><td> 0x00020000 </td><td> 802.1X authenticator </td></tr>
 * <tr><td> 0x00010000 </td><td> BSS node handling </td></tr>
 * <tr><td> 0x00008000 </td><td> 802.1X radius client </td></tr>
 * <tr><td> 0x00004000 </td><td> dump 802.1X radius packets </td></tr>
 * <tr><td> 0x00002000 </td><td> dump 802.1X keys </td></tr>
 * <tr><td> 0x00001000 </td><td> WPA/RSN protocol </td></tr>
 * <tr><td> 0x00000800 </td><td> ACL handling </td></tr>
 * <tr><td> 0x00000400 </td><td> WME protocol </td></tr>
 * <tr><td> 0x00000200 </td><td> 11ax mode debug-HE </td></tr>
 * <tr><td> 0x00000100 </td><td> 11.h </td></tr>
 * <tr><td> 0x00000080 </td><td> inactivity handling </td></tr>
 * <tr><td> 0x00000040 </td><td> sta-mode roaming </td></tr>
 * <tr><td> 0x00000020 </td><td> action management frames </td></tr>
 * <tr><td> 0x00000010 </td><td> transmit power control </td></tr>
 * <tr><td> 0x00000008 </td><td> VSP </td></tr>
 * <tr><td> 0x00000004 </td><td> 11ac mode debug-VHT </td></tr>
 * <tr><td> 0x00000001 </td><td> Extender: QHOP or Repeater </td></tr>
 * </table></small>
 *
 * \param ifname \wifi0
 * \param bitmap Debug message mask bitmap
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_vapdebug \<WiFi interface\> \<bitmap\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_set_vapdebug(const char *ifname, int bitmap);

/**
 * @brief Get debug message mask bitmap for vap interface in wlan driver
 *
 * \param ifname \wifi0
 * \param p_bitmap a pointer to the buffer for storing the returned value.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_vapdebug \<WiFi interface\></c>
 *
 * The output will be the vap debug bitmap unless an error occurs.
 * \sa qcsapi_wifi_set_vapdebug
 */
int qcsapi_wifi_get_vapdebug(const char *ifname, int *p_bitmap);

/**
 * @brief Do gather information and get the pid of the gathering process
 *
 * This API will start a background child process to gather comprehensive information,
 * and get the pid of the gathering process because it may take tens of seconds
 * to gather all information.
 *
 * \param p_pid a pointer to the buffer for the the pid of gathering process;
 * pass NULL if you don't care about the status of the gathering process
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi gather_info</c>
 *
 * The output will be the output of the gathered information
 */
int qcsapi_gather_info_start(int *p_pid);

/**
 * @brief Check whether pid is runnning or terminated
 *
 * \param pid the process id to check
 * \param cmd_done a pointer to the buffer for whether the process is
 * terminated.
 *
 * \return \zero_or_negative
 *
 * \sa qcsapi_gather_info_start
 */
int qcsapi_gather_info_check_cmd_status(int pid, int *cmd_done);

/**
 * @brief Read from gathered information
 *
 * The API will read from the gathered information with a start offset to the
 * internal buffer, and get the count of bytes copied and count of remaining bytes.
 *
 * \param buf a pointer to 3Kbytes buffer used to read from gathered information
 * \param offset start offset to internal gathered information buffer
 * \param bytes_copied a pointer to buffer to get the count of bytes copied
 * from internal buffer into buffer pointed by buf, pass NULL if don't care
 * \param bytes_remain a pointer to buffer to get the count of bytes remaining
 * in internal buffer, pass NULL if don't care
 *
 * \return \zero_or_negative
 *
 * \sa qcsapi_gather_info_start
 */
int qcsapi_read_gathered_info(struct qcsapi_data_3Kbytes *buf, unsigned int offset,
		unsigned int *bytes_copied, unsigned int *bytes_remain);

/** @} */

/**
 * @addtogroup StatisticsAPIs
 * @{
 */

/**
 * \brief Get latest CCA statistics data of an interface
 *
 * This API call is used to get latest CCA statistics data.
 *
 * \param ifname \wifi0
 * \param stats return parameter to contain the statistics data of the interface being queried.
 *
 * \return \positive_or_negative
 * If the call succeeded, stats contains the CCA statistics data retrieved from the device for the
 * interface.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_cca_stats \<interface name\></c>
 *
 * Unless an error occurs, the output will be the CCA statistics data of the interface.
 */
extern int qcsapi_get_cca_stats(const char *ifname, qcsapi_cca_stats *stats);

/** @} */

/**
 * @addtogroup commonAPIs
 * @{
 */

/**
 * \internal
 * \brief this API is used to get radio ID corresponded with interface name
 *
 * To get the radio ID of interface name
 *
 * \param ifname wireless interface name
 * \param radio_id pointer for radio ID
 *
 * \return \positive_or_negative
 *
 * \callqcsapi
 *
 * <c>no entry for the qcsapi</c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_get_radio_from_ifname(const char *ifname, unsigned int *radio_id);

/** @} */

/**
 * @addtogroup CSAPIs
 * @{
 */

/**
 * \brief Set the range of carrier sense threshold level supported by the radio
 *
 * Set range(minimum and maximum) of carrier sense threshold level supported by the radio
 *
 * \param ifname \wifi0
 * \param min minimum carrier sense threshold level
 * \param max maximum carrier sense threshold level
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_cs_thrshld_range \<WiFi interface\> \<min\> \<max\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_set_cs_thrshld_range(const char *ifname, int min, int max);

/**
 * \brief Grab EP configuration.
 *
 * Get content of the information parameter (memory or file) from EP.
 *
 * \param param_num Number of information parameter, same as line number in
 * EP's qtn_grabber.conf file
 * \param offset Offset into the file/memory from where start to grabbing.
 * \param flags Miscellaneous flags. Now just one flag supported:
 * QCSAPI_GRAB_ONLY_INFO_FL - grab only info header for passed parameter.
 * \param buf \databuf
 * \param bytes_copied Number of bytes copied to buffer
 *
 * \return \zero_or_negative
 *
 * Output is information parameter section header or content unless error occurs.
 *
 */
extern int qcsapi_grab_config(uint32_t param_num, uint32_t offset, uint32_t flags,
				struct qcsapi_data_1Kbytes *buf, uint32_t *bytes_copied);


/**
 * \brief Get the range of carrier sense threshold level supported by the radio
 *
 * Get range(minimum and maximum) of carrier sense threshold level supported by the radio
 *
 * \param ifname \wifi0
 * \param min buffer for minimum carrier sense threshold level
 * \param max buffer for maximum carrier sense threshold level
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_cs_thrshld_range \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_get_cs_thrshld_range(const char *ifname, int *min, int *max);

/**
 * \brief Set the RSSI signal level at which CS/CCA detects a busy condition.
 *
 * The RSSI signal level at which CS/CCA detects a busy condition.
 * This attribute enables APs to increase minimum sensitivity to avoid detecting busy condition
 * from multiple/weak Wi-Fi sources in dense Wi-Fi environments.
 *
 * \param ifname \wifi0
 * \param inuse the RSSI signal level
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_cs_thrshld_inuse \<WiFi interface\> \<inuse\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_set_cs_thrshld_inuse(const char *ifname, int inuse);

/**
 *
 * \brief Get the RSSI signal level at which CS/CCA detects a busy condition.
 *
 * The RSSI signal level at which CS/CCA detects a busy condition.
 * This attribute enables APs to increase minimum sensitivity to avoid detecting busy condition
 * from multiple/weak Wi-Fi sources in dense Wi-Fi environments.
 *
 * \param ifname \wifi0
 * \param inuse buffer for the RSSI signal level
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_cs_thrshld_inuse \<WiFi interface\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_get_cs_thrshld_inuse(const char *ifname, int *inuse);

/**
 *
 * \brief Get a BSA parameter.
 *
 * Get BSA parameter retrieves information from the BSA conf file.
 *
 * \note \deprecate_and_replace \ref qcsapi_bsa_get_parameter_ext.
 *
 * \param bsa_param the bsa parameters. See \ref qcsapi_bsa_conf_param.

 * \param store_info pointer to the bsa parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bsa_parameter \<bsa_param\></c>
 *
 * Unless an error occurs, the output will be the bsa parameter value.
 *
 * \note <c>\<bsa_param\></c> is a parameter string derived by removing the '<c>qcsapi_</c>' prefix
 * from a \link qcsapi_bsa_conf_param \endlink enum string. E.g.

 * @code
 * call_qcsapi get_bsa_parameter 2g_band_interface
 * @endcode
 */
int qcsapi_bsa_get_parameter(qcsapi_bsa_conf_param bsa_param, char *store_info);

/**
 * \brief Set a BSA parameter.
 *
 * Set a BSA parameter.
 *
 * \note \deprecate_and_replace \ref qcsapi_bsa_set_param_ext.
 *
 * \param bsa_param the bsa parameters. See the definition of \ref qcsapi_bsa_conf_param.
 * \param bsa_param_value1 pointer to the parameter value.
 * \param bsa_param_value2 pointer to the parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bsa_param \<bsa_param\> \<bsa_param_value1\> \<bsa_param_value2\></c>
 *
 * \note <c>\<bsa_param\></c> is a parameter string derived by removing the '<c>qcsapi_</c>' prefix
 * from a \link qcsapi_bsa_conf_param \endlink enum string. E.g.
 * @code
 * call_qcsapi set_bsa_parameter bs_ssid Quantenna
 * call_qcsapi set_bsa_parameter blacklist_timeout_val 10
 * @endcode
 *
 * \call_qcsapi_string_complete
 *
 */
int qcsapi_bsa_set_param(qcsapi_bsa_conf_param bsa_param, const char *bsa_param_value1,
					const char *bsa_param_value2);

/**
 *
 * \brief Set a BSA parameter.
 *
 * Set a BSA parameter.
 *
 * \note \deprecate_and_replace \ref qcsapi_bsa_set_param_ext.
 *
 * \param bsa_param the bsa parameters. See the definition of the enum <c>qcsapi_bsa_conf_param</c>.
 * \param bsa_param_value1 pointer to the parameter value.
 * \param bsa_param_value2 pointer to the parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bsa_param \<bsa_param\> \<bsa_param_value1\> \<bsa_param_value2\></c>
 *
 * Where <c>bsa_param</c> is one of
 * - <c>enable</c>
 * - <c>bs_ssid</c>
 * - <c>2g_band_interface</c>
 * - <c>5g_band_interface</c>
 * - <c>allow_5g_11ac_only</c>
 * - <c>non_steerable_count</c>
 * - <c>blacklist_timeout_val</c>
 * - <c>house_keeping_interval</c>
 * - <c>stale_entry_interval</c>
 * - <c>whitelist_sta_entry</c>
 * - <c>inactive_assoc_interval</c>
 * - <c>inactive_runtime_interval</c>
 * - <c>client_inactivity_threshold</c>
 * - <c>runtime_decision_interval</c>
 * - <c>assoc_5g_loaded_fat</c>
 * - <c>assoc_5g_unloaded_fat</c>
 * - <c>run_5g_loaded_fat</c>
 * - <c>run_5g_unloaded_fat</c>
 * - <c>assoc_5g_loaded_min_phyrate</c>
 * - <c>assoc_5g_min_phyrate</c>
 * - <c>run_5g_loaded_min_phyrate</c>
 * - <c>run_5g_unloaded_min_phyrate</c>
 * - <c>assoc_5g_loaded_min_rssi</c>
 * - <c>assoc_5g_min_rssi</c>
 * - <c>ssid_check</c>
 * - <c>debug_level</c>
 * - <c>reload_configuration</c>
 * - <c>clear_dual_band_interval</c>
 * - <c>deauth_for_steer_btm_capa_sta</c>
 * - <c>use_fix_bw_for_5g_phyrate_estimation</c>
 * - <c>rssi_threshold_for_5g_phyrate_estimation</c>
 *
 * <b>Examples</b>
 *
 * To set SSID to Quantenna, enter:
 *
 * <c>call_qcsapi set_bsa_param 0 Quantenna</c>
 *
 * To set RSSI threshold, enter:
 *
 * <c>call_qcsapi set_bsa_param 4 -64</c>
 *
 * \return \zero_or_negative
 */
int qcsapi_bsa_set_parameter(qcsapi_bsa_conf_param bsa_param, char *bsa_param_value1,
					char *bsa_param_value2);

/**
 * \brief Turn wlan link rate monitor feature on/off for the given MAC address.
 *
 * \param ifname \wifi0
 * \param mac_addr a MAC address to be monitored
 * \param enable a value turn the feature on/off
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi enable_link_monitor \<WiFi interface\> \<mac address\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */

int qcsapi_wifi_wlmonitor_enable(const char *ifname, const qcsapi_mac_addr mac_addr,
					uint32_t enable);

/**
 * \brief Configure threshold for WLAN link monitor below which a link report is generated.
 *
 * \param ifname \wifi0
 * \param type Two types, PHY rate threshold(0) and monitor period threshold(1)
 * \param severity Two levels, 0 is error and 1 is warning
 * \param period_unit Two units, 0 is hour and 1 is second
 * \param thres_value PHY rate threshold in unit of Mbps
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_link_monitor_rate_thres \<WiFi interface\> \<severity\> \<thres\></c>
 * <c>call_qcsapi set_link_monitor_duration \<WiFi interface\> \<severity\> \<unit\> \<thres\></c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_wlmonitor_config_threshold(const char *ifname, uint8_t type, uint8_t severity,
					uint8_t period_unint, uint32_t thres_value);

/**
 * \brief Display the list of supported BSA parameters
 *
 * Display the list of supported BSA parameters.
 *
 * \note \deprecate_and_replace \ref qcsapi_wifi_display_bsa_help.
 *
 * \param buf Buffer to hold the list of bsa_params
 * \param qcsapi_cmd Calling function (set_bsa_param/get_bsa_param)
 *
 * \return void
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_display_bsa_usage_help(char *buf, char *qcsapi_cmd);

/**
 * \brief Display the list of supported BSA parameters
 *
 * Display the list of supported BSA parameters.
 *
 * \param buf Buffer to hold the list of bsa_params
 * \param buf_len Length of buffer passed
 * \param qcsapi_cmd Calling function (set_bsa_param/get_bsa_param)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * @code
 * call_qcsapi get_bsa_parameter --help
 * @endcode
 *
 * Unless an error occurs, the output will be the list of supported BSA parameters.
 */
extern int qcsapi_wifi_display_bsa_help(char *buf, int buf_len, char *qcsapi_cmd);

/**
 * \brief Get the configured beacon PHY rate
 *
 * Get the configured beacon PHY rate.
 *
 * \note \aponly
 *
 * \param ifname \wifi_all
 * \param phyrate PHY rate for beacons in Mbps
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_beacon_phyrate \<WiFi interface\></c>
 */
extern int qcsapi_wifi_get_beacon_phyrate(const char *ifname, int *phyrate);

/**
 * \brief Set a fixed PHY rate for beacons
 *
 * Set a fixed PHY rate for beacons.
 *
 * \note \aponly
 *
 * \param ifname \wifi_all
 * \param phyrate PHY rate for beacons, a value from QTN_PHY_RATE_LEGACY_6M to
 * QTN_PHY_RATE_LEGACY_11M (qcsapi_legacy_phyrate)
 *
 * \note Only legacy (OFDM and ERP) rates can be specified.
 *
 * \note Only OFDM rates can be specified for radios that do not support 2.4GHz frequencies.
 *
 * \note If an ERP rate is configured, it will be ignored if 802.11b rates are disabled or
 * if the radio is on a 5GHz channel.
 *
 * \return 0 on success or the PHY rate configured is the same as the current beacon PHY rate,
 * or a negative value on error.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_beacon_phyrate \<WiFi interface\> <PHY rate in Mbps, eg 6M, 9M, ...></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 *
 * \sa qcsapi_legacy_phyrate
 */
extern int qcsapi_wifi_set_beacon_phyrate(const char *ifname, const qcsapi_legacy_phyrate phyrate);

/**
 *
 * \brief Issue the command to start dumping all of STAs' information from BSA station table.
 *
 * Issue command to tell BSA to prepare the information of all stations stored in station table and
 * will get the station information later.
 *
 * \param session_id holding the session ID which will be used as the session identifier during
 *	whole process of dumping station table
 * \param sta_count holding the total station count in the station table.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi bsa_get_sta_table</c>
 *
 * Unless an error occurs, the output will be the BSA STA information</c>.
 */
extern int qcsapi_bsa_start_dump_sta_table(uint16_t *session_id, uint32_t *sta_count);

/**
 *
 * \brief Get the all of STAs' information from BSA station table.
 *
 * Get information of stations stored in station table from the BSA app.
 *
 * \note Due to the buffer size limit, this function will be invoked several times until
 * information has been fetched for all stations.
 *
 * \param session_id specify the session ID identified the session of dumping station table
 * \param buf pointer to 3K bytes buffer holding the station information list.
 * \param sta_count holding the station count listed in buf.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi bsa_get_sta_table</c>
 *
 * Unless an error occurs, the output will be the BSA STA information</c>.
 */
extern int qcsapi_bsa_get_sta_table_item(uint16_t session_id, struct qcsapi_data_3Kbytes *buf,
					uint32_t *sta_count);

/**
 *
 * \brief Issue the command to stop dump all of STAs' information from BSA station table.
 *
 * Issue BSA to release the resource prepared before
 *
 * \param session_id specify the session ID
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi bsa_get_sta_table</c>
 *
 * Unless an error occurs, the output will be the BSA STA information</c>.
 */
extern int qcsapi_bsa_stop_dump_sta_table(uint16_t session_id);

/**
 * @brief Add an application specific IE.
 *
 * Set an application/customer specific IE, to be included in every transmission of the specified
 * frame type.
 *
 * \param ifname \wifi0
 * \param frametype 0 - beacon, 1 - probe req, 2 - probe resp, 3 - assoc req, 4 - assoc resp
 * \param index Index of IE to be added - currently must be '1'
 * \param ie Information Element in expanded hexadecimal notation with the following format:<br>
 * <c>\<element ID\> \<length\> \<content\></c><br>
 * Vendor specific (VS) content (when <c>element ID</c> = <c>0xDD</c>) is of the form:<br>
 * <c> \<OUI\> \<VS type\> \<VS data\></c>
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi add_app_ie \<WiFi interface\> \<frametype\> \<index\> \<ie\></c>
 *
 * <b>Example</b>
 *
 * <c>call_qcsapi add_app_ie wifi0_0 0 1 DD080026861911223344</c>
 *
 * The above command can be used to add an application-specific IE with the following
 * properties:
 * <ul>
 *  <li>Element ID = <c>0xDD</c> (vendor-specific IE)</li>
 *  <li>Length = <c>0x08</c> (8 bytes)</li>
 *  <li>Content = <c>0x0026861911223344</c>, where vendor-specific
 *  content is provided as:</li>
 *   <ul>
 *    <li>OUI = <c>0x002686</c> (Quantenna Communications, Inc.)</li>
 *    <li>VS type = <c>0x19</c> (Debug state)</li>
 *    <li>VS data = <c>0x11223344</c> (4 bytes of data)</li>
 *   </ul>
 * </ul>
 *
 * <b>Notes</b>
 * <ul>
 *  <li>The \ref OCAC feature inserts state and parameter values at octets 6 and 7 of
 *      the vendor-specific IE (Element ID 0xDD and VS type 0x17). It is therefore
 *      recommended that this combination is not used.</li>
 *  <li>Additionally, the following element IDs could produce side effects and
 * should not be used:
 *   <ul>
 *    <li>0x05 (IEEE80211_ELEMID_TIM)</li>
 *    <li>0x0B (IEEE80211_ELEMID_BSS_LOAD)</li>
 *    <li>0x23 (IEEE80211_ELEMID_TPCREP)</li>
 *    <li>0x25 (IEEE80211_ELEMID_CHANSWITCHANN)</li>
 *    <li>0x47 (IEEE80211_ELEMID_MULTI_BSSID)</li>
 *    <li>0xD8 (IEEE80211_ELEMID_TWT)</li>
 *   </ul>
 *  </li>
 * </ul>
 */
extern int qcsapi_add_app_ie(const char *ifname, const qcsapi_unsigned_int frametype,
				const qcsapi_unsigned_int index, const char *ie);

/**
 * @brief Remove application specific IE.
 *
 * Remove a previously added application/customer specific IE for the specified frame type.
 *
 * \param ifname \wifi0
 * \param frametype 0 - beacon, 1 - probe req, 2 - probe resp, 3 - assoc req, 4 - assoc resp
 * \param index Index of IE to be removed - currently must be '1'
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi remove_app_ie \<WiFi interface\> \<frametype\> \<index\></c>
 *
 */
extern int qcsapi_remove_app_ie(const char *ifname, const qcsapi_unsigned_int frametype,
							const qcsapi_unsigned_int index);

/**
 * @brief Disable 802.11b mode on a WiFi interface.
 *
 * Disable 802.11b mode on a WiFi interface.
 *
 * \param ifname \wifi0
 * \param value 1 (disable 802.11b) or 0 (enable 802.11b)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi disable_11b \<WiFi interface\> {0 | 1}</c>
 *
 */
extern int qcsapi_wifi_disable_11b(const char *ifname, const qcsapi_unsigned_int value);

/**
 *
 * \brief Get a BSA parameter
 *
 * Get a BSA parameter.
 *
 * \param mobility_domain the mobility domain identifier.</c>.
 * \param bsa_param the bsa parameters. See \ref <c>qcsapi_bsa_conf_param</c>.
 * \param store_info pointer to the bsa parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_bsa_parameter_ext \<mobility_domain\> \<bsa_param\></c>
 *
 * Where <c>bsa_param</c> is one of <c>enable</c>, <c>house_keeping_interval</c>,
 * <c>stale_entry_interval</c>, <c>whitelist_sta_entry</c>, <c>bsa_debug_level</c>,
 * <c>bsa_debug_qevt</c>, <c>sid_pair_status</c>, <c>bs_ssid</c>, <c>allow_5g_11ac_only</c>,
 * <c>non_steerable_count</c>, <c>blacklist_timeout_val</c>, <c>inactive_assoc_interval</c>,
 * <c>inactive_runtime_interval</c>, <c>client_inactivity_threshold</c>,
 * <c>runtime_decision_interval</c>, <c>assoc_5g_loaded_fat</c>, <c>assoc_5g_unloaded_fat</c>,
 * <c>run_5g_loaded_fat</c>, <c>run_5g_unloaded_fat</c>, <c>assoc_5g_loaded_min_phyrate</c>,
 * <c>assoc_5g_min_phyrate</c>, <c>run_5g_loaded_min_phyrate</c>,
 * <c>run_5g_unloaded_min_phyrate</c>, <c>assoc_5g_loaded_min_rssi</c>, <c>assoc_5g_min_rssi</c>,
 * <c>ssid_check</c>, <c>clear_dual_band_interval</c>,  <c>deauth_for_steer_btm_capa_sta</c>,
 * <c>use_fix_bw_for_5g_phyrate_estimation</c>, <c>rssi_threshold_for_5g_phyrate_estimation</c>,
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_bsa_get_parameter_ext(const int bsa_mb_domain, qcsapi_bsa_conf_param bsa_param,
					char *store_info);

/**
 *
 * \brief Set a BSA parameter.
 *
 * Set a BSA parameter.
 *
 * \note \deprecate_and_replace \ref qcsapi_bsa_get_parameter_ext.
 *
 * \param mobility_domain the Mobile domain identifier.
 * \param bsa_param the bsa parameters. See the definition of the enum <c>qcsapi_bsa_conf_param</c>.
 * \param bsa_param_value1 pointer to the parameter value.
 * \param bsa_param_value2 pointer to the parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bsa_param_ext \<mobility_domain\> \<bsa_param\> \<bsa_param_value1\>
 *	\<bsa_param_value2\></c>
 *
 * where <c>bsa_param</c> is one of <c>enable</c>, <c>house_keeping_interval</c>,
 * <c>stale_entry_interval</c>, <c>whitelist_sta_entry</c>, <c>bsa_debug_level</c>,
 * <c>bsa_debug_qevt</c>, <c>sid_pair_status</c>, <c>bs_ssid</c>, <c>allow_5g_11ac_only</c>,
 * <c>non_steerable_count</c>, <c>blacklist_timeout_val</c>, <c>inactive_assoc_interval</c>,
 * <c>inactive_runtime_interval</c>, <c>client_inactivity_threshold</c>,
 * <c>runtime_decision_interval</c>, <c>no_steering_interval</c>,<c>assoc_5g_loaded_fat</c>,
 * <c>assoc_5g_unloaded_fat</c>, <c>run_5g_loaded_fat</c>, <c>run_5g_unloaded_fat</c>,
 * <c>assoc_5g_loaded_min_phyrate</c>, <c>assoc_5g_min_phyrate</c>,
 * <c>run_5g_loaded_min_phyrate</c>, <c>run_5g_unloaded_min_phyrate</c>,
 * <c>assoc_5g_loaded_min_rssi</c>, <c>assoc_5g_min_rssi</c>, <c>ssid_check</c>,
 * <c>reload_configuration</c>, <c>bsa_module_restart</c>, <c>clear_dual_band_interval</c>,
 * <c>deauth_for_steer_btm_capa_sta</c>, <c>use_fix_bw_for_5g_phyrate_estimation</c>,
 * <c>rssi_threshold_for_5g_phyrate_estimation</c>
 *
 * <b>Examples</b>
 *
 * To set Mobile domain 1 SSID to Quantenna, enter:
 *
 * <c>call_qcsapi set_bsa_param_ext 1 bs_ssid Quantenna</c>
 *
 * To set RSSI threshold, enter:
 *
 * <c>call_qcsapi set_bsa_param_ext 1 assoc_5g_min_rssi -64</c>
 *
 * \return \zero_or_negative
 */
int qcsapi_bsa_set_parameter_ext(const int bsa_mb_domain, qcsapi_bsa_conf_param bsa_param,
					char *bsa_param_value1, char *bsa_param_value2);

/**
 *
 * \brief Set a BSA parameter.
 *
 * Set a BSA parameter.
 *
 * \note \deprecate_and_replace \ref qcsapi_bsa_get_parameter_ext.
 *
 * \param mobility_domain the Mobile domain identifier.
 * \param bsa_param the bsa parameters. See the definition of the enum <c>qcsapi_bsa_conf_param</c>.
 * \param bsa_param_value1 pointer to the parameter value.
 * \param bsa_param_value2 pointer to the parameter value.
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_bsa_param_ext \<mobility_domain\> \<bsa_param\> \<bsa_param_value1\>
 *	\<bsa_param_value2\></c>
 *
 * where <c>bsa_param</c> is one of <c>enable</c>, <c>house_keeping_interval</c>,
 * <c>stale_entry_interval</c>, <c>whitelist_sta_entry</c>, <c>bsa_debug_level</c>,
 * <c>bsa_debug_qevt</c>, <c>sid_pair_status</c>, <c>bs_ssid</c>, <c>allow_5g_11ac_only</c>,
 * <c>non_steerable_count</c>, <c>blacklist_timeout_val</c>, <c>inactive_assoc_interval</c>,
 * <c>inactive_runtime_interval</c>, <c>client_inactivity_threshold</c>,
 * <c>runtime_decision_interval</c>, <c>no_steering_interval</c>, <c>assoc_5g_loaded_fat</c>,
 * <c>assoc_5g_unloaded_fat</c>, <c>run_5g_loaded_fat</c>, <c>run_5g_unloaded_fat</c>,
 * <c>assoc_5g_loaded_min_phyrate</c>, <c>assoc_5g_min_phyrate</c>,
 * <c>run_5g_loaded_min_phyrate</c>, <c>run_5g_unloaded_min_phyrate</c>,
 * <c>assoc_5g_loaded_min_rssi</c>, <c>assoc_5g_min_rssi</c>, <c>ssid_check</c>,
 * <c>reload_configuration</c>, <c>bsa_module_restart</c>, <c>clear_dual_band_interval</c>,
 * <c>deauth_for_steer_btm_capa_sta</c>, <c>use_fix_bw_for_5g_phyrate_estimation</c>,
 * <c>rssi_threshold_for_5g_phyrate_estimation</c>
 *
 * <b>Examples</b>
 *
 * To set Mobile domain 1 SSID to Quantenna, enter:
 *
 * <c>call_qcsapi set_bsa_param_ext 1 bs_ssid Quantenna</c>
 *
 * To set RSSI threshold, enter:
 *
 * <c>call_qcsapi set_bsa_param_ext 1 assoc_5g_min_rssi -64</c>
 *
 * \return \zero_or_negative
 */
int qcsapi_bsa_set_param_ext(const int bsa_mb_domain, qcsapi_bsa_conf_param bsa_param,
				const char *bsa_param_value1, const char *bsa_param_value2);

/** @} */

/**
 * @addtogroup NonAssociatedClientsAPIs
 * @{
 */

/**
 * \brief Configure non associated clients monitor mode
 *
 * Configure non associated clients monitor mode
 *
 * This API accepts enable/disable values, to enable or disable non associated clients monitor mode
 * and if enable is true duty cycle period and percentage of on time values
 *
 * \note \aponly
 * \note This API is for configuring only on-channel monitor mode. To configure both
 * off-channel and on-channel monitor mode, use \ref qcsapi_wifi_set_scs_nac_monitor_mode.
 *
 * \param ifname \wifi0only
 * \param enable enable or disable
 * \param period 1 cycle time, ranges between 100ms to 5s (default 1000ms)
 * \param percentage_on 1% to 99% (default 1%)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_nac_mon_mode \<WiFi interface\> {enable | disable}
 *	[ [period] [percentage on] ]</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_nac_mon_mode(const char *ifname, const uint8_t enable,
					const uint16_t period, const uint8_t percentage_on);

/**
 * \brief Get non associated clients monitor mode settings or configuration
 *
 * This API gets current non associated clients monitor mode settings
 *
 * \param ifname \wifi0
 * \param enabled 1 (enabled), or 0 (disabled)
 * \param percentage_on percentage of on period in monitor duty cycle
 * \param period current cycle time or duty cycle period in milliseconds
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_nac_mon_mode \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be current monitor mode settings.
 */
extern int qcsapi_wifi_get_nac_mon_mode(const char *ifname, int *enable, int *period,
					int *percentage_on);

/**
 * \brief Get non associated clients stats
 * This API will query WLAN driver and gets statistics on non associated clients
 *
 * \param ifname \wifi0
 * \param report - non associated clients statistics report
 *
 * \return \zero_or_negative
 *
 * Non associated clients statistics report will be in this form \@ref qcsapi_nac_stats_report and
 * \@ref qcsapi_nac_stats.
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_nac_stats \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be string of current non associated clients statistics.
 */
extern int qcsapi_wifi_get_nac_stats(const char *ifname, qcsapi_nac_stats_report *report);

/**
 * \brief Get a debug value.
 *
 * Get a debug value.
 *
 * \param type type of debug value to be retrieved.
 * \param value \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_reboot_cause</c>
 *
 * Unless an error occurs, output will be the returned integer value.
 */
extern int qcsapi_system_get_debug_value(const qcsapi_debug_type type, qcsapi_unsigned_int *value);

/**
 * \brief Get system cpu usage.
 *
 * This function is used to get system cpu usage.
 *
 * \param cpu_usage return parameter to store the system cpu usage.
 * It is a string to describe the cpu usage of the system.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_cpu_usage</c>
 */
extern int qcsapi_get_cpu_usage(string_256 cpu_usage);

/**
 * \brief Get system memory usage.
 *
 * This function is used to get system memory usage.
 *
 * \param memory_usage return parameter to store the system memroy usage.
 * It is a string to describe the memory usage of the system.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_memory_usage</c>
 */
extern int qcsapi_get_memory_usage(string_256 memory_usage);


/**
 * @brief Put 5G or 2.4G radio under or exit low power mode
 *
 * This function is used to put 5G or 2.4G radio under or exit low power mode manually.
 *
 * \param radio_id 0 (5G radio) or 2 (2.4G radio)
 * \param force_pwr_save 1 (Enter power save mode manually) or 0 (Exit power save mode manually)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_radio_pwr_save \<radio_id\> {0 | 1}</c>
 *
 */
extern int qcsapi_set_radio_pwr_save(int radio_id, int force_pwr_save);

/**
 * @brief Put 5G or 2.4G radio under reset
 *
 * This function is used to put 5G or 2.4G radio under reset. Normally, this API is only used for
 * test or debug usage.
 *
 * \param radio_id 0 (5G radio) or 2 (2.4G radio)
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi put_radio_under_reset \<radio_id\></c>
 *
 */
extern int qcsapi_put_radio_under_reset(int radio_id);

/**
 * \internal
 * \brief Reassign VAP interfaces when configuring or unconfiguring Repeater mode.
 *
 * Reassign VAP interfaces when configuring or unconfiguring Repeater mode.
 *
 * \note This API is used internally, and should not be called from a user interface.
 *
 * \param radio_id \radio_id
 * \param for_repeater 1 if configuring repeater mode, or 0 if configuring AP mode
 *
 * \note When changing wifi0 to repeater mode, the VAP interfaces are reassigned for each SSID
 * starting from wifi0_1. When changing wifi0 to AP mode, the VAP interfaces are reassigned for
 * each SSID starting from wifi0_0.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi repeater_mode_cfg \<radio_id\> {0 | 1}</c>
 *
 * \call_qcsapi_string_complete
 */
int qcsapi_wifi_repeater_mode_cfg(const qcsapi_unsigned_int radio_id,
					const unsigned int for_repeater);

/**
 * @brief Set U-Repeater parameters
 *
 * Set U-Repeater parameters
 *
 * \param type \ref qcsapi_urepeater_type
 * \param param_value U-Repeater parameter value
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_urepeater_params \<parameter type\> \<parameter value\></c>
 *
 * where <c>parameter type</c> is <c>max_level</c>
 *
 * \call_qcsapi_string_complete
 */
extern int qcsapi_wifi_set_urepeater_params(const qcsapi_urepeater_type type,
					const int param_value);

/**
 * @brief Get U-Repeater parameters
 *
 * Get U-Repeater parameters
 *
 * \param type \ref qcsapi_urepeater_type
 * \param p_value \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_urepeater_params \<parameter type\></c>
 *
 * where <c>parameter type</c> is <c>max_level</c> or <c>curr_level</c>
 *
 * Unless an error occurs, the output will be the U-Repeater's related parameter value.
 */
extern int qcsapi_wifi_get_urepeater_params(const qcsapi_urepeater_type type, int *p_value);

/** @} */

/**
 * @addtogroup StatelessMode
 * @{
 */

/**
 * @brief Save persistent config files to local storage on a host device.
 *
 * Save persistent config files to local storage on a host device.
 *
 * This API must be called in a loop until the 'done' flag is set, to save the entire contents of
 * the <c>/mnt/jffs2</c> directory to a 'cfg' file on the host system. Refer to the call_qcsapi
 * source code for details.
 *
 * \note This command can only be used from a PCIe RC (host) device.
 *
 * \param buf \valuebuf
 * \param offset offset in tar file from whence to start reading
 * \param bytes_copied buffer to contain the number of bytes retrieved
 * \param bytes_remain 1 if the end of the file was reached, else 0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi save_config [\<output dir\>]</c>
 *
 * This command implements the loop described above.
 * The file is saved to <c>\<output_dir\>qtn-\<MAC address\>-v\<version\>.cfg<\c>, where MAC address
 * is the MAC address of the device's PCIe or primary Ethernet interface in the format XXXXXXXXXXXX,
 * where each XX pair is an expanded hexadecimal character (colons are removed). The output
 * directory defaults to QCSAPI_CONFIG_FILE_PATH, which should be used for PCIe devices.
 * If \<output_dir\> is specified, it must exist.
 * If the default directory is used and does not already exist, it is created.
 * For non-PCIe devices, the file must be saved to (or moved to) the same tftp directory
 * used for installing images onto the device.
 */
int qcsapi_save_config(struct qcsapi_data_3Kbytes *buf, unsigned int offset,
		unsigned int *bytes_copied, unsigned int *bytes_remain);

/**
 * \brief Get config directory status
 *
 * Get a report on whether config has been mounted, and if it was mounted from flash or mounted to
 * RAM from a remote system (which is known as stateless mode).
 *
 * The report is in the following format.
 *
 * @code
 * Config mounted from: <device>
 * Reason: <info>
 * Config version: <version>
 * MAC address: <mac>
 * @endcode
 *
 * Where
 * * \<device\> can be "flash", "RAM" or "none"
 * * \<info\> contains information about why \<device\> was used
 * * \<version\> is the configuration file version, (only printed if \<device\> is RAM)
 * * \<mac\> is the MAC address used to retrieve the config file from the host
 *	(only printed if \<device\> is RAM)
 * \param result \databuf
 * \param len size of the result buffer
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_config_status</c>
 */
int qcsapi_get_config_status(char *result, int len);

/** @} */

/**
 * @addtogroup AACSAPI
 * @{
 */

/**
 * \brief Exclude a contiguous set of channels from AACS
 *
 * Exclude a contiguous set of channels from AACS where the set
 * is inclusive of the channel with starting channel number but
 * exclusive of the channel with ending channel number
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param chstart starting channel number
 * \param chend ending channel number
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_excl_chan \<WiFi interface\> \<chstart\> \<chend\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 */
extern int qcsapi_wifi_aacs_excl_ch_set(const char *ifname,
				uint32_t chstart, uint32_t chend);

/**
 * \brief Retrieve the range of channels excluded from AACS
 *
 * Retrieve the two channel numbers, the starting channel number
 * and ending channel number, that identifies the range of
 * channels being excluded from AACS.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_excl_chan \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will consist of 2 values
 * separated by a space: starting channel number followed by the
 * ending channel number for the excluded channel set.
 */
extern int qcsapi_wifi_aacs_excl_ch_get(const char *ifname,
				qcsapi_unsigned_int *p_chstart, qcsapi_unsigned_int *p_chend);

/**
 * \brief Exclude a contiguous set of channels from Always-on DFS
 *
 * Exclude a contiguous set of channels from Always-on DFS where the set
 * is inclusive of the channel with starting channel number but
 * exclusive of the channel with ending channel number
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param chstart starting channel number
 * \param chend ending channel number
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_alt_excl_chan \<WiFi interface\> \<chstart\> \<chend\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 */
extern int qcsapi_wifi_aacs_alt_excl_ch_set(const char *ifname,
				uint32_t chstart, uint32_t chend);

/**
 * \brief Retrieve the range of channels excluded from Always-on DFS
 *
 * Retrieve the two channel numbers, the starting channel number
 * and ending channel number, that identifies the range of
 * channels being excluded from Always-on DFS.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_alt_excl_chan \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will consist of 2 values
 * separated by a space: starting channel number followed by the
 * ending channel number for the excluded channel set.
 */
extern int qcsapi_wifi_aacs_alt_excl_ch_get(const char *ifname,
				qcsapi_unsigned_int *p_chstart, qcsapi_unsigned_int *p_chend);

/**
 * \brief Configure the minimum threshold table for AACS
 *
 * Configures all entries in the minimum threshold table for AACS. Entries
 * in this table defines the minimum value above which the channel metric
 * is scaled with the corresponding factor.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param v0 1st entry in table
 * \param v1 2nd entry in table
 * \param v2 3rd entry in table
 * \param v3 4th entry in table
 * \param v4 5th entry in table
 * \param v5 6th entry in table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_thres_min_tbl \<WiFi interface\> \<v0\> \<v1\>
 * \<v2\> \<v3\> \<v4\> \<v5\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>.
 */
extern int qcsapi_wifi_aacs_thres_min_tbl_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve the minimum threshold table for AACS
 *
 * Retrieve all entries in the minimum threshold table for AACS. Entries
 * in this table defines the minimum value above which the channel metric
 * is scaled with the corresponding factor.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_thres_min_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the six space-separated
 * entries from the minimum threshold table.
 */
extern int qcsapi_wifi_aacs_thres_min_tbl_get(const char *ifname,
				struct qcsapi_int_array32 *buf,
				qcsapi_unsigned_int *p_length);

/**
 * \brief Configure the maximum threshold table for AACS
 *
 * Configures all entries in the maximum threshold table for AACS. Entries
 * in this table defines the maximum value below which the channel metric
 * is scaled with the corresponding factor.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param v0 1st entry in table
 * \param v1 2nd entry in table
 * \param v2 3rd entry in table
 * \param v3 4th entry in table
 * \param v4 5th entry in table
 * \param v5 6th entry in table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_thres_max_tbl \<WiFi interface\> \<v0\> \<v1\>
 * \<v2\> \<v3\> \<v4\> \<v5\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_thres_max_tbl_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve the maximum threshold table for AACS
 *
 * Retrieve all entries in the maximum threshold table for AACS. Entries
 * in this table defines the maximum value below which the channel metric
 * is scaled with the corresponding factor.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_thres_max_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the six space-separated
 * entries from the maximum threshold table.
 */
extern int qcsapi_wifi_aacs_thres_max_tbl_get(const char *ifname,
				struct qcsapi_int_array32 *buf,
				qcsapi_unsigned_int *p_length);

/**
 * \brief Configure the DFS threshold table for AACS
 *
 * Configure all entries in the DFS threshold table for AACS.
 * Entries in this table defines the scaling factor with which AACS
 * used in calculate metric for a DFS channel.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param v0 1st entry in table
 * \param v1 2nd entry in table
 * \param v2 3rd entry in table
 * \param v3 4th entry in table
 * \param v4 5th entry in table
 * \param v5 6th entry in table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_dfs_thres_adj_tbl \<WiFi interface\> \<v0\> \<v1>
 * \<v2> \<v3> \<v4> \<v5\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_dfs_thres_adj_tbl_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve DFS threshold table for AACS
 *
 * Retrieve all entries in the DSF threshold table for AACS.
 * Entries in this table defines the scaling factor AACS used to
 * scale a DFS channel's calculated metric.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_dfs_thres_adj_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the six space-separated
 * entries from the DFS threshold table.
 */
extern int qcsapi_wifi_aacs_dfs_thres_adj_tbl_get(const char *ifname,
				struct qcsapi_int_array32 *buf,
				qcsapi_unsigned_int *p_length);

/**
 * \brief Configure virtual node RSSI table for AACS
 *
 * Configure all entries in the virtual node RSSI table for AACS.
 * Entries in this table defines the supposed RSSI for the corresponding
 * virtual node. AACS uses these values to form its selection criteria.
 * All four entries must be provided.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param v0 1st entry in table
 * \param v1 2nd entry in table
 * \param v2 3rd entry in table
 * \param v3 4rd entry in table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_vnode_rssi_tbl \<WiFi interface\> \<v0\> \<v1\> \<v2\> \<v3\> </c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_vnode_rssi_tbl_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve virtual node RSSI table for AACS
 *
 * Retrieve all entries in the virtual node RSSI table for AACS.
 * Entries in this table defines the supposed RSSI for the corresponding
 * virtual node. AACS uses these values to formulate its
 * selection criteria.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_vnode_rssi_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the four space-separated
 * entries from the virtual node RSSI table.
 */
extern int qcsapi_wifi_aacs_vnode_rssi_tbl_get(const char *ifname,
				struct qcsapi_int_array32 *buf,
				qcsapi_unsigned_int *p_length);

/**
 * \brief Configure virtual node weight table for AACS
 *
 * Configure all entries in the virtual node weight table for AACS.
 * Entries in this table defines the weighting for the corresponding
 * virtual node. AACS uses these values to formulate its selection
 * criteria.
 *
 * All four entries must be provided. Each entry must be between 0 and 100.
 * Sum of the four entries must equal to 100, or an equal weight of 25
 * for each entry is automatically applied.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param v0 1st entry in table
 * \param v1 2nd entry in table
 * \param v2 3rd entry in table
 * \param v3 4th entry in table
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_vnode_wgt_tbl \<WiFi interface\> \<v0\> \<v1\> \<v2\> \<v3\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_vnode_wgt_tbl_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve virtual node weight table for AACS
 *
 * Retrieve all entries in the virtual node weight table for AACS.
 * Entries in this table defines the weighting for the corresponding
 * virtual node. AACS uses these values to formulate its selection
 * criteria.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_vnode_wgt_tbl \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the four space-separated
 * entries from the virtual node weight table.
 */
extern int qcsapi_wifi_aacs_vnode_wgt_tbl_get(const char *ifname,
				struct qcsapi_int_array32 *buf,
				qcsapi_unsigned_int *p_length);

/**
 * \brief Update a virtual node that is used by AACS
 *
 * Update one of the four virtual nodes that is used by AACS.
 * Each update must comprise of 3 pieces of information: RSSI, weight,
 * and index of the node to update.
 *
 * Node index is limited between 0 and 3. Weight must be between 0 and 100.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param rssi RSSI
 * \param weight weight
 * \param index node index of entry
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_vnode \<WiFi interface\> \<rssi\> \<weight\> \<index\></c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_vnode_set(const char *ifname,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve information on virtual nodes used by AACS and display
 * the corresponding information.
 *
 * Retrieve information on virtual nodes in use by AACS. Data retrieved
 * contains three pieces of information associated with that virtual node:
 * RSSI, weight, and index of the node.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_vnodes \<WiFi interface\></c>
 *
 * Unless an error occurs, there will be four lines of output. Each line
 * line corresponds to a virtual node and consists three space-separated
 * values: virtual node's index, RSSI, and weight.
 */
extern int qcsapi_wifi_aacs_vnode_get(const char *ifname,
				struct qcsapi_int_array32 *buf, qcsapi_unsigned_int *p_length);

/**
 * \brief Updates user-selectable channel(s) to AACS
 *
 * Updates up to 10 user-selectable channels to AACS or Always-On DFS. This
 * control takes priority over channel exclusion control. All user-selected
 * channels cannot be excluded in channel exclusion operation.
 *
 * If op is 'del' or 'alt_del' and no channel numbers is specified, then the
 * entire user-selectable channel list is cleared.
 *
 * This control may be repeated to expand channel selection.
 *
 * Invalid channel numbers are ignored.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param op Operation to be performed: valid values are add, alt_add, del, or alt_del
 * \param ch1 1st channel
 * \param ch2 2nd channel
 * \param ch3 3rd channel
 * \param ch4 4th channel
 * \param ch5 5th channel
 * \param ch6 6th channel
 * \param ch7 7th channel
 * \param ch8 8th channel
 * \param ch9 9th channel
 * \param ch10 10th channel; must be a valid channel number
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_set_sel_chan \<WiFi interface\> {add | alt_add}
 * \<ch1\> \[ch2\] \[ch3\] \[ch4\] \[ch5\] \[ch6\] \[ch7\] \[ch8\] \[ch9\] \[ch10\]</c>
 *
 * <c>call_qcsapi aacs_set_sel_chan \<WiFi interface\> {del | alt_del}
 * \[ch1\] \[ch2\] \[ch3\] \[ch4\] \[ch5\] \[ch6\] \[ch7\] \[ch8\] \[ch9\] \[ch10\]</c>
 *
 * Unless an error occurs, the output will be the string <c>complete</c>
 */
extern int qcsapi_wifi_aacs_sel_ch_set(const char *ifname, uint32_t op,
				struct qcsapi_int_array32 *buf,
				uint32_t length);

/**
 * \brief Retrieve list of user-selected channels from AACS.
 *
 * Retrieve the list of user-selected channels from AACS. Different interfaces may have
 * different number of channels.
 *
 * \note \aponly
 *
 * \param ifname \wifi0
 * \param op Operation to be performed: valid values are get or alt_get
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi aacs_get_sel_chan \<WiFi interface\> {get | alt_get}</c>
 *
 * Unless an error occurs, the list of user-selected channel numbers
 * for AACS or Always-On DFS is displayed.
 */
extern int	qcsapi_wifi_aacs_sel_ch_get(const char *ifname, uint32_t op,
				struct qcsapi_int_array32 *buf, qcsapi_unsigned_int *p_length);

/**
 * @brief Set the WiFi interface legacy BBIC configuration.
 *
 * Set the WiFi interface legacy BBIC configuration to ensure maximum performance
 * compatibility with QHS710 clients
 *
 * \param ifname \wifi0
 * \param val Compatibility option. Valid values are described in \ref qcsapi_legacy_bbic_opt.
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi set_legacy_bbic \<WiFi interface\> <val></c>
 *
 */
extern int qcsapi_wifi_set_legacy_bbic(const char *ifname, const qcsapi_unsigned_int val);

/**
 * @brief Get the WiFi interface legacy BBIC configuration.
 *
 * Get the WiFi interface legacy BBIC configuration.
 *
 * \param ifname \wifi0
 * \param val \valuebuf
 *
 * \return \zero_or_negative
 *
 * \callqcsapi
 *
 * <c>call_qcsapi get_legacy_bbic \<WiFi interface\></c>
 *
 * Unless an error occurs, the output will be the current legacy BBIC configuration.
 *
 */
extern int qcsapi_wifi_get_legacy_bbic(const char *ifname, qcsapi_unsigned_int *val);

/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* _QCSAPI_H */

