/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2009 - 2018 Quantenna Communications, Inc.          **
**                                                                           **
**  File        : call_qcsapi.c                                              **
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
EH0*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#include <net80211/ieee80211_qos.h>
#include <net80211/ieee80211_dfs_reentry.h>
#include <net80211/ieee80211_ioctl.h>
#include <net80211/_ieee80211.h>
#include <qtn/qtn_vlan_api.h>
#include <qtn/qtn_bs_comm.h>
#include <qtn/qtn_monitor.h>
#include <qtn/ieee80211_aacs.h>
#include "qcsapi.h"
#include <qtn/qtn_nis.h>
#include <qtn/qtn_global.h>
#include <qtn_bits.h>
#include <qtn/qtnis.h>
#include "qcsapi_driver.h"
#include "call_qcsapi.h"
#include "qcsapi_sem.h"
#include "qcsapi_util.h"
#include "qcsapi_grabber.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#endif

#ifndef IS_MULTIPLE_BITS_SET
#define IS_MULTIPLE_BITS_SET(_x)	(((unsigned)(_x)) & (((unsigned)(_x)) - 1))
#endif

#define printf		Do_not_use_printf
#define fprintf		Do_not_use_fprintf

#define IP_ADDR_STR_LEN 16
#define BEACON_INTERVAL_WARNING_LOWER_LIMIT	24
#define BEACON_INTERVAL_WARNING_UPPER_LIMIT	100

const char *qcsapi_ieee80211_reason_str[] = IEEE80211_REASON_STR;

static const struct
{
	qcsapi_entry_point	e_entry_point;
	const char		*api_name;
} qcsapi_entry_name[] =
{
	{ e_qcsapi_errno_get_message,		"get_error_message" },
	{ e_qcsapi_store_ipaddr,		"store_ipaddr" },
	{ e_qcsapi_interface_enable,		"enable_interface" },
	{ e_qcsapi_interface_get_BSSID,		"interface_BSSID" },
	{ e_qcsapi_interface_get_mac_addr,	"get_mac_addr" },
	{ e_qcsapi_interface_get_mac_addr,	"get_macaddr" },
	{ e_qcsapi_interface_set_mac_addr,	"set_mac_addr" },
	{ e_qcsapi_interface_set_mac_addr,	"set_macaddr" },
	{ e_qcsapi_interface_get_counter,	"get_counter" },
	{ e_qcsapi_interface_get_counter64,	"get_counter64" },
	{ e_qcsapi_pm_get_counter,		"get_pm_counter" },
	{ e_qcsapi_pm_get_elapsed_time,		"get_pm_elapsed_time" },
	{ e_qcsapi_flash_image_update,		"flash_image_update" },
	{ e_qcsapi_firmware_get_version,	"get_firmware_version" },
	{ e_qcsapi_system_get_time_since_start,	"get_time_since_start" },
	{ e_qcsapi_get_system_status,		"get_sys_status" },
	{ e_qcsapi_get_cpu_usage,		"get_cpu_usage" },
	{ e_qcsapi_get_memory_usage,		"get_memory_usage" },
	{ e_qcsapi_get_random_seed,		"get_random_seed" },
	{ e_qcsapi_set_random_seed,		"set_random_seed" },
	{ e_qcsapi_led_get,			"get_LED" },
	{ e_qcsapi_led_set,			"set_LED" },
	{ e_qcsapi_led_pwm_enable,		"set_LED_PWM" },
	{ e_qcsapi_led_brightness,		"set_LED_brightness" },
	{ e_qcsapi_gpio_get_config,		"get_GPIO_config" },
	{ e_qcsapi_gpio_set_config,		"set_GPIO_config" },
	{ e_qcsapi_gpio_monitor_reset_device,	"monitor_reset_device" },
	{ e_qcsapi_gpio_enable_wps_push_button,	"enable_wps_push_button" },
	{ e_qcsapi_file_path_get_config,	"get_file_path" },
	{ e_qcsapi_file_path_set_config,	"set_file_path" },
	{ e_qcsapi_wifi_set_wifi_macaddr,	"set_wifi_mac_addr" },
	{ e_qcsapi_wifi_set_wifi_macaddr,	"set_wifi_macaddr" },
	{ e_qcsapi_wifi_create_restricted_bss,	"wifi_create_restricted_bss"},
	{ e_qcsapi_wifi_create_bss,		"wifi_create_bss"},
	{ e_qcsapi_wifi_remove_bss,		"wifi_remove_bss"},
	{ e_qcsapi_wifi_get_primary_interface,	"get_primary_interface"},
	{ e_qcsapi_wifi_get_interface_by_index,	"get_interface_by_index"},
	{ e_qcsapi_wifi_get_interface_by_index_all, "get_interface_by_index_all"},
	{ e_qcsapi_wifi_get_mode,		"get_mode" },
	{ e_qcsapi_wifi_set_mode,		"set_mode" },
	{ e_qcsapi_wifi_get_phy_mode,		"get_phy_mode" },
	{ e_qcsapi_wifi_set_phy_mode,		"set_phy_mode" },
	{ e_qcsapi_wifi_get_phy_mode_required,	"get_phy_mode_required" },
	{ e_qcsapi_wifi_set_phy_mode_required,	"set_phy_mode_required" },
	{ e_qcsapi_wifi_reload_in_mode,		"reload_in_mode" },
	{ e_qcsapi_wifi_rfenable,		"rfenable" },
	{ e_qcsapi_service_control,		"service_control" },
	{ e_qcsapi_wfa_cert,			"wfa_cert" },
	{ e_qcsapi_wifi_rfstatus,		"rfstatus" },
	{ e_qcsapi_wifi_startprod,		"startprod" },
	{ e_qcsapi_wifi_get_freq_bands,		"get_supported_freq_bands" },
	{ e_qcsapi_wifi_get_bw,			"get_bw" },
	{ e_qcsapi_wifi_set_bw,			"set_bw" },
	{ e_qcsapi_wifi_get_24g_bw,		"get_24g_bw" },
	{ e_qcsapi_wifi_set_24g_bw,		"set_24g_bw" },
	{ e_qcsapi_wifi_get_BSSID,		"get_BSSID" },
	{ e_qcsapi_wifi_get_config_BSSID,	"get_config_BSSID" },
	{ e_qcsapi_wifi_ssid_get_bssid,		"get_ssid_bssid" },
	{ e_qcsapi_wifi_ssid_set_bssid,		"set_ssid_bssid" },
	{ e_qcsapi_wifi_get_SSID,		"get_SSID" },
	{ e_qcsapi_wifi_get_SSID2,		"get_SSID2" },
	{ e_qcsapi_wifi_set_SSID,		"set_SSID" },
	{ e_qcsapi_wifi_get_channel,		"get_channel" },
	{ e_qcsapi_wifi_get_chan,		"get_chan" },
	{ e_qcsapi_wifi_set_channel,		"set_channel" },
	{ e_qcsapi_wifi_set_chan,		"set_chan" },
	{ e_qcsapi_wifi_set_channel_and_bw,	"set_chan_and_bw" },
	{ e_qcsapi_wifi_get_channel_and_bw,	"get_chan_and_bw" },
	{ e_qcsapi_wifi_set_wea_cac_en,		"set_wea_cac_en" },
	{ e_qcsapi_wifi_get_auto_channel,	"get_auto_channel" },
	{ e_qcsapi_wifi_set_auto_channel,	"set_auto_channel" },
	{ e_qcsapi_wifi_get_standard,		"get_standard" },
	{ e_qcsapi_wifi_get_standard,		"get_802.11" },
	{ e_qcsapi_wifi_get_dtim,		"get_dtim" },
	{ e_qcsapi_wifi_set_dtim,		"set_dtim" },
	{ e_qcsapi_wifi_get_assoc_limit,	"get_dev_assoc_limit" },
	{ e_qcsapi_wifi_set_assoc_limit,	"set_dev_assoc_limit" },
	{ e_qcsapi_wifi_get_bss_assoc_limit,	"get_bss_assoc_limit" },
	{ e_qcsapi_wifi_set_bss_assoc_limit,	"set_bss_assoc_limit" },
	{ e_qcsapi_wifi_set_SSID_group_id,	"set_SSID_group_id" },
	{ e_qcsapi_wifi_get_SSID_group_id,	"get_SSID_group_id" },
	{ e_qcsapi_wifi_set_SSID_assoc_reserve,	"set_SSID_assoc_reserve" },
	{ e_qcsapi_wifi_get_SSID_assoc_reserve,	"get_SSID_assoc_reserve" },
	{ e_qcsapi_interface_get_status,	"get_status" },
	{ e_qcsapi_interface_set_ip4,		"set_ip" },
	{ e_qcsapi_interface_get_ip4,		"get_ip" },
	{ e_qcsapi_interface_set_mtu,		"set_mtu" },
	{ e_qcsapi_interface_get_mtu,		"get_mtu" },
	{ e_qcsapi_wifi_get_list_channels,	"get_list_of_channels" },
	{ e_qcsapi_wifi_get_list_channels,	"get_channel_list" },
	{ e_qcsapi_wifi_get_chan_list,		"get_chan_list" },
	{ e_qcsapi_wifi_get_supp_chans,		"get_supp_chan" },
	{ e_qcsapi_wifi_get_mode_switch,	"get_mode_switch" },
	{ e_qcsapi_wifi_get_mode_switch,	"get_wifi_mode_switch" },
	{ e_qcsapi_wifi_get_noise,		"get_noise" },
	{ e_qcsapi_wifi_get_rssi_by_chain,	"get_rssi_by_chain" },
	{ e_qcsapi_wifi_get_avg_snr,		"get_avg_snr" },
	{ e_qcsapi_wifi_get_option,		"get_option" },
	{ e_qcsapi_wifi_set_option,		"set_option" },
	{ e_qcsapi_wifi_set_parameter,		"set_wifi_param"},
	{ e_qcsapi_wifi_get_parameter,		"get_wifi_param"},
	{ e_qcsapi_wifi_get_rates,		"get_rates" },
	{ e_qcsapi_wifi_set_rates,		"set_rates" },
	{ e_qcsapi_wifi_get_max_bitrate,	"get_max_bitrate" },
	{ e_qcsapi_wifi_set_max_bitrate,	"set_max_bitrate" },
	{ e_qcsapi_wifi_get_beacon_type,	"get_beacon_type" },
	{ e_qcsapi_wifi_get_beacon_type,	"get_beacon" },
	{ e_qcsapi_wifi_set_beacon_type,	"set_beacon_type" },
	{ e_qcsapi_wifi_set_beacon_type,	"set_beacon" },
	{ e_qcsapi_wifi_get_beacon_interval,		"get_beacon_interval" },
	{ e_qcsapi_wifi_set_beacon_interval,		"set_beacon_interval" },
	{ e_qcsapi_wifi_get_list_regulatory_regions,
						"get_regulatory_regions" },
	{ e_qcsapi_wifi_get_list_regulatory_regions,
						"get_list_regulatory_regions" },
	{ e_qcsapi_wifi_get_regulatory_tx_power,
						"get_regulatory_tx_power" },
	{ e_qcsapi_wifi_get_configured_tx_power,
						"get_configured_tx_power" },
	{ e_qcsapi_wifi_set_regulatory_channel, "set_regulatory_channel" },
	{ e_qcsapi_wifi_set_regulatory_region,	"set_regulatory_region" },
	{ e_qcsapi_wifi_get_regulatory_region,	"get_regulatory_region" },
	{ e_qcsapi_wifi_overwrite_country_code,	"overwrite_country_code" },
	{ e_qcsapi_wifi_get_list_regulatory_channels,
						"get_list_regulatory_channels" },
	{ e_qcsapi_wifi_get_list_regulatory_bands,
						"get_list_regulatory_bands" },
	{ e_qcsapi_wifi_get_regulatory_db_version,
						"get_regulatory_db_version" },
	{ e_qcsapi_wifi_set_regulatory_tx_power_cap,
						"apply_regulatory_cap" },
	{ e_qcsapi_wifi_restore_regulatory_tx_power,
						"restore_regulatory_tx_power"},
	{ e_qcsapi_wifi_set_chan_pri_inactive,	"set_chan_pri_inactive" },
	{ e_qcsapi_wifi_get_chan_pri_inactive,	"get_chan_pri_inactive" },
	{ e_qcsapi_wifi_set_chan_disabled,	"set_chan_disabled" },
	{ e_qcsapi_wifi_get_chan_disabled,	"get_chan_disabled" },
	{ e_qcsapi_wifi_get_chan_usable,	"get_chan_usable" },

	{ e_qcsapi_wifi_get_tx_power,		"get_tx_power" },
	{ e_qcsapi_wifi_set_tx_power,		"set_tx_power" },
	{ e_qcsapi_wifi_get_tx_power_ext,	"get_tx_power_ext" },
	{ e_qcsapi_wifi_set_tx_power_ext,	"set_tx_power_ext" },
	{ e_qcsapi_reg_chan_txpower_get,	"get_chan_power_table" },
	{ e_qcsapi_reg_chan_txpower_get,	"reg_chan_txpower_get" },
	{ e_qcsapi_reg_chan_txpower_set,	"reg_chan_txpower_set" },
	{ e_qcsapi_reg_chan_txpower_path_get,	"reg_chan_txpower_path_get" },
	{ e_qcsapi_wifi_set_chan_power_table,	"set_chan_power_table" },
	{ e_qcsapi_wifi_get_bw_power,		"get_bw_power" },
	{ e_qcsapi_wifi_set_bw_power,		"set_bw_power" },
	{ e_qcsapi_wifi_get_bf_power,		"get_bf_power" },
	{ e_qcsapi_wifi_set_bf_power,		"set_bf_power" },
	{ e_qcsapi_wifi_get_power_selection,	"get_power_selection" },
	{ e_qcsapi_wifi_set_power_selection,	"set_power_selection" },
	{ e_qcsapi_wifi_get_carrier_interference,		"get_carrier_db" },
	{ e_qcsapi_wifi_get_congestion_idx,		"get_congest_idx" },
	{ e_qcsapi_wifi_get_supported_tx_power_levels, "get_supported_tx_power" },
	{ e_qcsapi_wifi_get_current_tx_power_level, "get_current_tx_power" },
	{ e_qcsapi_wifi_set_power_constraint, "set_power_constraint"},
	{ e_qcsapi_wifi_get_power_constraint, "get_power_constraint"},
	{ e_qcsapi_wifi_set_tpc_interval, "set_tpc_query_interval"},
	{ e_qcsapi_wifi_get_tpc_interval, "get_tpc_query_interval"},
	{ e_qcsapi_wifi_get_assoc_records,	"get_assoc_records" },
	{ e_qcsapi_wifi_get_list_DFS_channels,	"get_list_DFS_channels" },
	{ e_qcsapi_wifi_is_channel_DFS,		"is_channel_DFS" },
	{ e_qcsapi_wifi_get_DFS_alt_channel,	"get_DFS_alt_channel" },
	{ e_qcsapi_wifi_set_DFS_alt_channel,	"set_DFS_alt_channel" },
	{ e_qcsapi_wifi_set_DFS_reentry,	"start_dfsreentry"},
	{ e_qcsapi_wifi_set_radar_chain,	"set_radar_chain" },
	{ e_qcsapi_wifi_get_scs_cce_channels,	"get_scs_cce_channels" },
	{ e_qcsapi_wifi_get_dfs_cce_channels,	"get_dfs_cce_channels" },
	{ e_qcsapi_wifi_get_csw_records,	"get_csw_records" },
	{ e_qcsapi_wifi_get_radar_status,	"get_radar_status" },
	{ e_qcsapi_wifi_get_WEP_encryption_level,
						"get_WEP_encryption_level" },
	{ e_qcsapi_wifi_get_WEP_key,		"get_WEP_key" },
	{ e_qcsapi_wifi_set_WEP_key,		"set_WEP_key" },
	{ e_qcsapi_wifi_get_WEP_key_index,	"get_WEP_key_index" },
	{ e_qcsapi_wifi_set_WEP_key_index,	"set_WEP_key_index" },
	{ e_qcsapi_wifi_remove_WEP_config,	"remove_WEP_config" },

	{ e_qcsapi_wifi_get_WPA_encryption_modes, "get_WPA_encryption_modes" },
	{ e_qcsapi_wifi_set_WPA_encryption_modes, "set_WPA_encryption_modes" },
	{ e_qcsapi_wifi_get_WPA_authentication_mode, "get_WPA_authentication_mode" },
	{ e_qcsapi_wifi_set_WPA_authentication_mode, "set_WPA_authentication_mode" },

	{ e_qcsapi_wifi_get_params, "get_params" },
	{ e_qcsapi_wifi_set_params, "set_params" },

	{ e_qcsapi_wifi_get_interworking, "get_interworking" },
	{ e_qcsapi_wifi_set_interworking, "set_interworking" },
	{ e_qcsapi_wifi_get_80211u_params, "get_80211u_params" },
	{ e_qcsapi_wifi_set_80211u_params, "set_80211u_params" },
	{ e_qcsapi_security_set_sec_agent, "sec_agent"},
	{ e_qcsapi_security_get_sec_agent_status, "get_sec_agent_status"},
	{ e_qcsapi_security_get_nai_realms, "get_nai_realms" },
	{ e_qcsapi_security_add_nai_realm, "add_nai_realm" },
	{ e_qcsapi_security_del_nai_realm, "del_nai_realm" },
	{ e_qcsapi_security_add_roaming_consortium, "add_roaming_consortium" },
	{ e_qcsapi_security_del_roaming_consortium, "del_roaming_consortium" },
	{ e_qcsapi_security_get_roaming_consortium, "get_roaming_consortium" },
	{ e_qcsapi_security_get_venue_name, "get_venue_name" },
	{ e_qcsapi_security_add_venue_name, "add_venue_name" },
	{ e_qcsapi_security_del_venue_name, "del_venue_name" },
	{ e_qcsapi_security_get_oper_friendly_name, "get_oper_friendly_name" },
	{ e_qcsapi_security_add_oper_friendly_name, "add_oper_friendly_name" },
	{ e_qcsapi_security_del_oper_friendly_name, "del_oper_friendly_name" },
	{ e_qcsapi_security_get_hs20_conn_capab, "get_hs20_conn_capab" },
	{ e_qcsapi_security_add_hs20_conn_capab, "add_hs20_conn_capab" },
	{ e_qcsapi_security_del_hs20_conn_capab, "del_hs20_conn_capab" },

	{ e_qcsapi_security_add_hs20_icon, "add_hs20_icon" },
	{ e_qcsapi_security_get_hs20_icon, "get_hs20_icon" },
	{ e_qcsapi_security_del_hs20_icon, "del_hs20_icon" },

	{ e_qcsapi_security_add_osu_server_uri, "add_osu_server_uri" },
	{ e_qcsapi_security_get_osu_server_uri, "get_osu_server_uri" },
	{ e_qcsapi_security_del_osu_server_uri, "del_osu_server_uri" },

	{ e_qcsapi_security_add_osu_server_param, "add_osu_server_param" },
	{ e_qcsapi_security_get_osu_server_param, "get_osu_server_param" },
	{ e_qcsapi_security_del_osu_server_param, "del_osu_server_param" },

	{ e_qcsapi_wifi_get_hs20_status, "get_hs20_status" },
	{ e_qcsapi_wifi_set_hs20_status, "set_hs20_status" },
	{ e_qcsapi_wifi_get_hs20_params, "get_hs20_params" },
	{ e_qcsapi_wifi_set_hs20_params, "set_hs20_params" },

	{ e_qcsapi_remove_11u_param, "remove_11u_param" },
	{ e_qcsapi_remove_hs20_param, "remove_hs20_param" },

	{ e_qcsapi_wifi_set_proxy_arp, "set_proxy_arp" },
	{ e_qcsapi_wifi_get_proxy_arp, "get_proxy_arp" },
	{ e_qcsapi_wifi_get_l2_ext_filter, "get_l2_ext_filter" },
	{ e_qcsapi_wifi_set_l2_ext_filter, "set_l2_ext_filter" },

	{ e_qcsapi_wifi_get_IEEE11i_encryption_modes, "get_IEEE11i_encryption_modes" },
	{ e_qcsapi_wifi_set_IEEE11i_encryption_modes, "set_IEEE11i_encryption_modes" },
	{ e_qcsapi_wifi_get_IEEE11i_authentication_mode, "get_IEEE11i_authentication_mode" },
	{ e_qcsapi_wifi_set_IEEE11i_authentication_mode, "set_IEEE11i_authentication_mode" },
	{ e_qcsapi_wifi_get_michael_errcnt, "get_michael_errcnt" },
	{ e_qcsapi_wifi_get_pre_shared_key,	"get_pre_shared_key" },
	{ e_qcsapi_wifi_set_pre_shared_key,	"set_pre_shared_key" },
	{ e_qcsapi_wifi_add_radius_auth_server_cfg,	"add_radius_auth_server_cfg" },
	{ e_qcsapi_wifi_del_radius_auth_server_cfg,	"del_radius_auth_server_cfg" },
	{ e_qcsapi_wifi_get_radius_auth_server_cfg,	"get_radius_auth_server_cfg" },
	{ e_qcsapi_wifi_add_radius_acct_server_cfg,	"add_radius_acct_server_cfg" },
	{ e_qcsapi_wifi_del_radius_acct_server_cfg,	"del_radius_acct_server_cfg" },
	{ e_qcsapi_wifi_get_radius_acct_server_cfg,	"get_radius_acct_server_cfg" },
	{ e_qcsapi_wifi_set_own_ip_addr,	"set_own_ip_addr" },
	{ e_qcsapi_wifi_get_psk_auth_failures,	"get_psk_auth_failures" },
	{ e_qcsapi_wifi_get_pre_shared_key,	"get_PSK" },
	{ e_qcsapi_wifi_set_pre_shared_key,	"set_PSK" },
	{ e_qcsapi_wifi_get_key_passphrase,	"get_passphrase" },
	{ e_qcsapi_wifi_get_key_passphrase,	"get_key_passphrase" },
	{ e_qcsapi_wifi_set_key_passphrase,	"set_passphrase" },
	{ e_qcsapi_wifi_set_key_passphrase,	"set_key_passphrase" },
	{ e_qcsapi_wifi_get_group_key_interval, "get_group_key_interval" },
	{ e_qcsapi_wifi_set_group_key_interval, "set_group_key_interval" },
	{ e_qcsapi_wifi_get_pairwise_key_interval, "get_pairwise_key_interval" },
	{ e_qcsapi_wifi_set_pairwise_key_interval, "set_pairwise_key_interval" },
	{ e_qcsapi_wifi_get_pmf,	"get_pmf" },
	{ e_qcsapi_wifi_set_pmf,	"set_pmf" },
	{ e_qcsapi_wifi_get_count_associations,	"get_count_assoc" },
	{ e_qcsapi_wifi_get_count_associations,	"get_count_associations" },
	{ e_qcsapi_wifi_get_count_associations,	"get_association_count" },
	{ e_qcsapi_wifi_get_associated_device_mac_addr,	"get_associated_device_mac_addr" },
	{ e_qcsapi_wifi_get_associated_device_mac_addr,	"get_station_mac_addr" },
	{ e_qcsapi_wifi_get_associated_device_ip_addr,	"get_associated_device_ip_addr" },
	{ e_qcsapi_wifi_get_associated_device_ip_addr,	"get_station_ip_addr" },
	{ e_qcsapi_wifi_get_link_quality,	"get_link_quality" },
	{ e_qcsapi_wifi_get_rssi_per_association, "get_rssi" },
	{ e_qcsapi_wifi_get_hw_noise_per_association, "get_hw_noise" },
	{ e_qcsapi_wifi_get_rssi_in_dbm_per_association, "get_rssi_dbm" },
	{ e_qcsapi_wifi_get_meas_rssi_in_dbm_per_association,	"get_meas_rssi_dbm" },
	{ e_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association,	"get_meas_rssi_minmax" },
	{ e_qcsapi_wifi_set_rssi_meas_period,	"set_rssi_meas_period" },
	{ e_qcsapi_wifi_get_rssi_meas_period,	"get_rssi_meas_period" },
	{ e_qcsapi_wifi_get_snr_per_association, "get_snr" },
	{ e_qcsapi_wifi_get_rx_bytes_per_association, "get_rx_bytes" },
	{ e_qcsapi_wifi_get_rx_bytes_per_association, "get_assoc_rx_bytes" },
	{ e_qcsapi_wifi_get_tx_bytes_per_association, "get_tx_bytes" },
	{ e_qcsapi_wifi_get_tx_bytes_per_association, "get_assoc_tx_bytes" },
	{ e_qcsapi_wifi_get_rx_packets_per_association, "get_rx_packets" },
	{ e_qcsapi_wifi_get_rx_packets_per_association, "get_assoc_rx_packets" },
	{ e_qcsapi_wifi_get_tx_packets_per_association, "get_tx_packets" },
	{ e_qcsapi_wifi_get_tx_packets_per_association, "get_assoc_tx_packets" },
	{ e_qcsapi_wifi_get_tx_err_packets_per_association,
						"get_tx_err_packets" },
	{ e_qcsapi_wifi_get_tx_err_packets_per_association,
						"get_assoc_tx_err_packets" },
	{ e_qcsapi_wifi_get_tx_allretries_per_association, "get_tx_allretries" },
	{ e_qcsapi_wifi_get_tx_exceed_retry_per_association, "get_tx_exceed_retry" },
	{ e_qcsapi_wifi_get_tx_retried_per_association, "get_tx_retried" },
	{ e_qcsapi_wifi_get_tx_retried_percent_per_association, "get_tx_retried_percent" },
	{ e_qcsapi_wifi_get_bw_per_association, "get_assoc_bw" },
	{ e_qcsapi_wifi_get_tx_phy_rate_per_association, "get_tx_phy_rate" },
	{ e_qcsapi_wifi_get_rx_phy_rate_per_association, "get_rx_phy_rate" },
	{ e_qcsapi_wifi_get_tx_mcs_per_association, "get_tx_mcs" },
	{ e_qcsapi_wifi_get_rx_mcs_per_association, "get_rx_mcs" },
	{ e_qcsapi_wifi_get_achievable_tx_phy_rate_per_association,
						"get_achievable_tx_phy_rate" },
	{ e_qcsapi_wifi_get_achievable_rx_phy_rate_per_association,
						"get_achievable_rx_phy_rate" },
	{ e_qcsapi_wifi_get_auth_enc_per_association, "get_auth_enc_per_assoc" },
	{ e_qcsapi_wifi_get_tput_caps,	"get_tput_caps" },
	{ e_qcsapi_wifi_get_connection_mode,	"get_connection_mode" },
	{ e_qcsapi_wifi_get_vendor_per_association, "get_vendor" },
	{ e_qcsapi_wifi_get_max_mimo,	"get_max_mimo" },

	{ e_qcsapi_wifi_get_node_counter,	"get_node_counter" },
	{ e_qcsapi_wifi_get_node_param,		"get_node_param" },
	{ e_qcsapi_wifi_get_node_stats,		"get_node_stats" },
	{ e_qcsapi_wifi_get_node_list,		"get_node_list" },
	{ e_qcsapi_wifi_get_node_infoset,	"get_node_infoset" },
	{ e_qcsapi_wifi_get_node_infoset_all,	"get_node_infoset_all" },
	{ e_qcsapi_wifi_get_if_infoset,		"get_if_infoset" },

	{ e_qcsapi_wifi_get_max_queued,		"get_max_queued" },

	{ e_qcsapi_wifi_disassociate,	"disassociate" },
	{ e_qcsapi_wifi_disassociate_sta,	"disassociate_sta" },
	{ e_qcsapi_wifi_reassociate,	"reassociate" },

	{ e_qcsapi_wifi_associate,	"associate" },

	{ e_qcsapi_wifi_get_mac_address_filtering, "get_macaddr_filter" },
	{ e_qcsapi_wifi_set_mac_address_filtering, "set_macaddr_filter" },
	{ e_qcsapi_wifi_is_mac_address_authorized, "is_mac_addr_authorized" },
	{ e_qcsapi_wifi_is_mac_address_authorized, "is_macaddr_authorized" },
	{ e_qcsapi_wifi_get_authorized_mac_addresses, "get_authorized_mac_addr" },
	{ e_qcsapi_wifi_get_authorized_mac_addresses, "get_authorized_macaddr" },
	{ e_qcsapi_wifi_get_denied_mac_addresses, "get_blocked_mac_addr" },
	{ e_qcsapi_wifi_get_denied_mac_addresses, "get_blocked_macaddr" },
	{ e_qcsapi_wifi_get_denied_mac_addresses, "get_denied_mac_addr" },
	{ e_qcsapi_wifi_get_denied_mac_addresses, "get_denied_macaddr" },
	{ e_qcsapi_wifi_authorize_mac_address,	"authorize_mac_addr" },
	{ e_qcsapi_wifi_authorize_mac_address,	"authorize_macaddr" },
	{ e_qcsapi_wifi_deny_mac_address,	"block_macaddr" },
	{ e_qcsapi_wifi_deny_mac_address,	"block_mac_addr" },
	{ e_qcsapi_wifi_deny_mac_address,	"deny_macaddr" },
	{ e_qcsapi_wifi_deny_mac_address,	"deny_mac_addr" },
	{ e_qcsapi_wifi_remove_mac_address,	"remove_mac_addr" },
	{ e_qcsapi_wifi_remove_mac_address,	"remove_macaddr" },
	{ e_qcsapi_wifi_clear_mac_address_filters,	"clear_mac_filters" },
	{ e_qcsapi_wifi_set_mac_address_reserve,	"set_macaddr_reserve" },
	{ e_qcsapi_wifi_get_mac_address_reserve,	"get_macaddr_reserve" },
	{ e_qcsapi_wifi_clear_mac_address_reserve,	"clear_macaddr_reserve" },

	{ e_qcsapi_wifi_backoff_fail_max,	"backoff_fail_max" },
	{ e_qcsapi_wifi_backoff_timeout,	"backoff_timeout" },
	{ e_qcsapi_wifi_get_wpa_status,		"get_wpa_status" },
	{ e_qcsapi_wifi_get_auth_state,		"get_auth_state" },
	{ e_qcsapi_wifi_get_disconn_info,	"get_disconn_info" },
	{ e_qcsapi_wifi_reset_disconn_info,	"reset_disconn_info" },
	{ e_qcsapi_wifi_get_pairing_id,		"get_pairing_id"},
	{ e_qcsapi_wifi_set_pairing_id,		"set_pairing_id"},
	{ e_qcsapi_wifi_get_pairing_enable,	"get_pairing_enable"},
	{ e_qcsapi_wifi_set_pairing_enable,	"set_pairing_enable"},

	{ e_qcsapi_wifi_set_txqos_sched_tbl,	"set_txqos_sched_tbl" },
	{ e_qcsapi_wifi_get_txqos_sched_tbl,	"get_txqos_sched_tbl" },

	{ e_qcsapi_wps_registrar_report_button_press, "registrar_report_button_press" },
	{ e_qcsapi_wps_registrar_report_button_press, "registrar_report_pbc" },
	{ e_qcsapi_wps_registrar_report_pin,	"registrar_report_pin" },
	{ e_qcsapi_wps_registrar_get_pp_devname, "registrar_get_pp_devname" },
	{ e_qcsapi_wps_registrar_set_pp_devname, "registrar_set_pp_devname" },
	{ e_qcsapi_wps_enrollee_report_button_press, "enrollee_report_button_press" },
	{ e_qcsapi_wps_enrollee_report_button_press, "enrollee_report_pbc" },
	{ e_qcsapi_wps_enrollee_report_pin,	"enrollee_report_pin" },
	{ e_qcsapi_wps_enrollee_generate_pin,	"enrollee_generate_pin" },
	{ e_qcsapi_wps_get_ap_pin,		"get_wps_ap_pin" },
	{ e_qcsapi_wps_set_ap_pin,		"set_wps_ap_pin" },
	{ e_qcsapi_wps_save_ap_pin,		"save_wps_ap_pin" },
	{ e_qcsapi_wps_enable_ap_pin,		"enable_wps_ap_pin" },
	{ e_qcsapi_wps_get_sta_pin,	"get_wps_sta_pin" },
	{ e_qcsapi_wps_get_state,		"get_wps_state" },
	{ e_qcsapi_wps_get_configured_state,	"get_wps_configured_state" },
	{ e_qcsapi_wps_set_configured_state,	"set_wps_configured_state" },
	{ e_qcsapi_wps_get_runtime_state,	"get_wps_runtime_state" },
	{ e_qcsapi_wps_get_allow_pbc_overlap_status,		"get_allow_pbc_overlap_status" },
	{ e_qcsapi_wps_allow_pbc_overlap,		"allow_pbc_overlap" },
	{ e_qcsapi_wps_check_config,		"check_wps_config" },
	{ e_qcsapi_wps_get_param,		"get_wps_param" },
	{ e_qcsapi_wps_set_param,		"set_wps_param" },
	{ e_qcsapi_wps_set_access_control,	"set_wps_access_control" },
	{ e_qcsapi_wps_get_access_control,	"get_wps_access_control" },
	{ e_qcsapi_non_wps_set_pp_enable,	"set_non_wps_pp_enable" },
	{ e_qcsapi_non_wps_get_pp_enable,	"get_non_wps_pp_enable" },
	{ e_qcsapi_wps_cancel,			"wps_cancel" },
	{ e_qcsapi_wps_set_pbc_in_srcm,		"set_wps_pbc_in_srcm" },
	{ e_qcsapi_wps_get_pbc_in_srcm,		"get_wps_pbc_in_srcm" },
	{ e_qcsapi_wps_timeout,			"wps_set_timeout" },
	{ e_qcsapi_wps_on_hidden_ssid,		"wps_on_hidden_ssid" },
	{ e_qcsapi_wps_on_hidden_ssid_status,	"wps_on_hidden_ssid_status" },
	{ e_qcsapi_wps_upnp_enable,		"wps_upnp_enable" },
	{ e_qcsapi_wps_upnp_status,		"wps_upnp_status" },
	{ e_qcsapi_wps_registrar_set_dfl_pbc_bss, "registrar_set_default_pbc_bss"},
	{ e_qcsapi_wps_registrar_get_dfl_pbc_bss, "registrar_get_default_pbc_bss"},

	{ e_qcsapi_wifi_set_dwell_times,	"set_dwell_times" },
	{ e_qcsapi_wifi_get_dwell_times,	"get_dwell_times" },
	{ e_qcsapi_wifi_set_bgscan_dwell_times,	"set_bgscan_dwell_times" },
	{ e_qcsapi_wifi_get_bgscan_dwell_times,	"get_bgscan_dwell_times" },
	{ e_qcsapi_wifi_get_scan_chan_list,	"get_scan_chan_list" },
	{ e_qcsapi_wifi_set_scan_chan_list,	"set_scan_chan_list" },
	{ e_qcsapi_wifi_start_scan,		"start_scan" },
	{ e_qcsapi_wifi_cancel_scan,		"cancel_scan" },
	{ e_qcsapi_wifi_get_scan_status,	"get_scanstatus" },
	{ e_qcsapi_wifi_get_cac_status,		"get_cacstatus" },
	{ e_qcsapi_wifi_wait_scan_completes,	"wait_scan_completes" },
	{ e_qcsapi_wifi_set_scan_chk_inv,	"set_scan_chk_inv" },
	{ e_qcsapi_wifi_get_scan_chk_inv,	"get_scan_chk_inv" },

	{ e_qcsapi_wifi_update_bss_cfg,		"update_bss_cfg" },
	{ e_qcsapi_wifi_get_bss_cfg,		"get_bss_cfg" },

	{ e_qcsapi_SSID_create_SSID,		"SSID_create_SSID" },
	{ e_qcsapi_SSID_create_SSID,		"create_SSID" },
	{ e_qcsapi_SSID_remove_SSID,		"remove_SSID" },
	{ e_qcsapi_SSID_verify_SSID,		"SSID_verify_SSID" },
	{ e_qcsapi_SSID_verify_SSID,		"verify_SSID" },
	{ e_qcsapi_SSID_rename_SSID,		"SSID_rename_SSID" },
	{ e_qcsapi_SSID_rename_SSID,		"rename_SSID" },
	{ e_qcsapi_SSID_get_SSID_list,		"get_SSID_list" },
	{ e_qcsapi_SSID_get_SSID2_list,		"get_SSID2_list" },
	{ e_qcsapi_SSID_get_protocol,		"get_SSID_proto" },
	{ e_qcsapi_SSID_get_protocol,		"SSID_get_proto" },
	{ e_qcsapi_SSID_set_protocol,		"set_SSID_proto" },
	{ e_qcsapi_SSID_set_protocol,		"SSID_set_proto" },
	{ e_qcsapi_SSID_get_encryption_modes,	"SSID_get_encryption_modes" },
	{ e_qcsapi_SSID_set_encryption_modes,	"SSID_set_encryption_modes" },
	{ e_qcsapi_SSID_get_group_encryption,	"SSID_get_group_encryption" },
	{ e_qcsapi_SSID_set_group_encryption,	"SSID_set_group_encryption" },
	{ e_qcsapi_SSID_get_authentication_mode, "SSID_get_authentication_mode" },
	{ e_qcsapi_SSID_set_authentication_mode, "SSID_set_authentication_mode" },
	{ e_qcsapi_SSID_get_pre_shared_key,	"SSID_get_pre_shared_key" },
	{ e_qcsapi_SSID_set_pre_shared_key,	"SSID_set_pre_shared_key" },
	{ e_qcsapi_SSID_get_key_passphrase,	"SSID_get_key_passphrase" },
	{ e_qcsapi_SSID_get_key_passphrase,	"SSID_get_passphrase" },
	{ e_qcsapi_SSID_set_key_passphrase,	"SSID_set_key_passphrase" },
	{ e_qcsapi_SSID_set_key_passphrase,	"SSID_set_passphrase" },
	{ e_qcsapi_SSID_get_pmf,		"SSID_get_pmf" },
	{ e_qcsapi_SSID_set_pmf,		"SSID_set_pmf" },
	{ e_qcsapi_SSID_get_wps_SSID,		"SSID_get_WPS_SSID" },
	{ e_qcsapi_SSID_get_params,		"SSID_get_params" },
	{ e_qcsapi_SSID_set_params,		"SSID_set_params" },
	{ e_qcsapi_wifi_vlan_config,		"vlan_config" },
	{ e_qcsapi_wifi_show_vlan_config,	"show_vlan_config" },

	{ e_qcsapi_wifi_start_cca,		"start_cca" },
	{ e_qcsapi_wifi_disable_wps,		"disable_wps" },
	{ e_qcsapi_wifi_get_results_AP_scan,	"get_results_AP_scan" },
	{ e_qcsapi_wifi_get_count_APs_scanned,	"get_count_APs_scanned" },
	{ e_qcsapi_wifi_get_properties_AP,	"get_properties_AP" },
	{ e_qcsapi_wifi_get_wps_ie_scanned_AP,	"get_wps_ie_scanned_AP" },

	{e_qcsapi_wifi_get_time_associated_per_association, "get_time_associated" },

	{ e_qcsapi_wifi_wds_add_peer,		"wds_add_peer"},
	{ e_qcsapi_wifi_wds_remove_peer,	"wds_remove_peer"},
	{ e_qcsapi_wifi_wds_get_peer_address,	"wds_get_peer_address"},
	{ e_qcsapi_wifi_wds_set_psk,		"wds_set_psk"},
	{ e_qcsapi_wifi_wds_set_mode,		"wds_set_mode"},
	{ e_qcsapi_wifi_wds_get_mode,		"wds_get_mode"},

	{ e_qcsapi_wifi_qos_get_param,		"get_qos_param" },
	{ e_qcsapi_wifi_qos_set_param,		"set_qos_param" },

	{ e_qcsapi_wifi_get_wmm_ac_map,		"get_wmm_ac_map" },
	{ e_qcsapi_wifi_set_wmm_ac_map,		"set_wmm_ac_map" },

	{ e_qcsapi_wifi_get_dscp_8021p_map,	"get_dscp_8021p_map" },
	{ e_qcsapi_wifi_set_dscp_8021p_map,	"set_dscp_8021p_map" },
	{ e_qcsapi_wifi_get_dscp_ac_map,	"get_dscp_ac_map" },
	{ e_qcsapi_wifi_set_dscp_ac_map,	"set_dscp_ac_map" },
	{ e_qcsapi_wifi_get_dscp_ac_table,	"get_dscp_ac_table" },
	{ e_qcsapi_wifi_set_dscp_ac_table,	"set_dscp_ac_table" },
	{ e_qcsapi_wifi_get_dscp_tid_table,	"get_dscp_tid_table" },
	{ e_qcsapi_wifi_set_dscp_tid_table,	"set_dscp_tid_table" },
	{ e_qcsapi_wifi_get_dscp_vap_link,	"get_dscp_vap_link" },
	{ e_qcsapi_wifi_set_dscp_vap_link,	"set_dscp_vap_link" },

	{ e_qcsapi_wifi_get_ac_agg_hold_time,	"get_ac_agg_hold_time" },
	{ e_qcsapi_wifi_set_ac_agg_hold_time,	"set_ac_agg_hold_time" },

	{ e_qcsapi_wifi_set_qos_map,		"set_qos_map" },
	{ e_qcsapi_wifi_del_qos_map,		"del_qos_map" },
	{ e_qcsapi_wifi_get_qos_map,		"get_qos_map" },
	{ e_qcsapi_wifi_send_qos_map_conf,	"send_qos_map_conf" },
	{ e_qcsapi_wifi_get_dscp_tid_map,	"get_dscp_tid_map" },

	{ e_qcsapi_wifi_get_priority,		"get_priority" },
	{ e_qcsapi_wifi_set_priority,		"set_priority" },
	{ e_qcsapi_wifi_get_airfair,		"get_airfair" },
	{ e_qcsapi_wifi_set_airfair,		"set_airfair" },
	{ e_qcsapi_wifi_get_airquota,		"get_airquota" },
	{ e_qcsapi_wifi_set_airquota,		"set_airquota" },
	{ e_qcsapi_wifi_get_airquota_node,	"get_airquota_node" },
	{ e_qcsapi_wifi_set_airquota_node,	"set_airquota_node" },
	{ e_qcsapi_wifi_get_premier_list,	"get_premier_list" },
	{ e_qcsapi_wifi_set_premier_list,	"set_premier_list" },
	{ e_qcsapi_wifi_get_premier_rule,	"get_premier_rule" },
	{ e_qcsapi_wifi_set_premier_rule,	"set_premier_rule" },

	{ e_qcsapi_config_get_parameter,	"get_config_param"},
	{ e_qcsapi_config_get_parameter,	"get_persistent_param"},
	{ e_qcsapi_config_update_parameter,	"update_config_param"},
	{ e_qcsapi_config_update_parameter,	"update_persistent_param"},
	{ e_qcsapi_bootcfg_get_parameter,	"get_bootcfg_param"},
	{ e_qcsapi_bootcfg_update_parameter,	"update_bootcfg_param"},
	{ e_qcsapi_bootcfg_commit,		"commit_bootcfg"},
	{ e_qcsapi_wifi_get_mcs_rate,		"get_mcs_rate" },
	{ e_qcsapi_wifi_set_mcs_rate,		"set_mcs_rate" },
	{ e_qcsapi_config_get_ssid_parameter,		"get_persistent_ssid_param"},
	{ e_qcsapi_config_update_ssid_parameter,	"update_persistent_ssid_param"},

	{ e_qcsapi_wifi_enable_scs,			"enable_scs" },
	{ e_qcsapi_wifi_scs_set_version,		"scs_set_version" },
	{ e_qcsapi_wifi_scs_switch_channel,		"scs_switch_chan" },
	{ e_qcsapi_wifi_scs_pick_best_channel,		"scs_pick_chan" },
	{ e_qcsapi_wifi_set_scs_verbose,		"set_scs_verbose" },
	{ e_qcsapi_wifi_get_scs_status,			"get_scs_status" },
	{ e_qcsapi_wifi_set_scs_smpl_enable,		"set_scs_smpl_enable" },
	{ e_qcsapi_wifi_set_scs_active_chan_list,	"set_scs_active_chan_list"},
	{ e_qcsapi_wifi_get_scs_active_chan_list,	"get_scs_active_chan_list"},
	{ e_qcsapi_wifi_set_scs_smpl_dwell_time,	"set_scs_smpl_dwell_time" },
	{ e_qcsapi_wifi_get_scs_smpl_dwell_time,	"get_scs_smpl_dwell_time" },
	{ e_qcsapi_wifi_set_scs_smpl_intv,		"set_scs_smpl_intv" },
	{ e_qcsapi_wifi_get_scs_smpl_intv,		"get_scs_smpl_intv" },
	{ e_qcsapi_wifi_set_scs_smpl_type,		"set_scs_smpl_type" },
	{ e_qcsapi_wifi_set_scs_intf_detect_intv,	"set_scs_intf_detect_intv" },
	{ e_qcsapi_wifi_set_scs_thrshld,		"set_scs_thrshld" },
	{ e_qcsapi_wifi_set_scs_report_only,		"set_scs_report_only" },
	{ e_qcsapi_wifi_get_scs_report_stat,		"get_scs_report" },
	{ e_qcsapi_wifi_set_scs_cca_intf_smth_fctr,	"set_scs_cca_intf_smth_fctr" },
	{ e_qcsapi_wifi_set_scs_chan_mtrc_mrgn,		"set_scs_chan_mtrc_mrgn" },
	{ e_qcsapi_wifi_set_scs_nac_monitor_mode,	"set_scs_nac_monitor_mode" },
	{ e_qcsapi_wifi_set_scs_band_margin_check,	"set_scs_band_margin_check" },
	{ e_qcsapi_wifi_set_scs_band_margin,		"set_scs_band_margin" },
	{ e_qcsapi_wifi_get_scs_dfs_reentry_request,	"get_scs_dfs_reentry_request" },
	{ e_qcsapi_wifi_get_scs_cca_intf,		"get_scs_cca_intf" },
	{ e_qcsapi_wifi_get_scs_param,			"get_scs_params" },
	{ e_qcsapi_wifi_set_scs_stats,			"set_scs_stats" },
	{ e_qcsapi_wifi_set_scs_burst_enable,		"set_scs_burst_enable" },
	{ e_qcsapi_wifi_set_scs_burst_window,		"set_scs_burst_window" },
	{ e_qcsapi_wifi_set_scs_burst_thresh,		"set_scs_burst_thresh" },
	{ e_qcsapi_wifi_set_scs_burst_pause,		"set_scs_burst_pause_time" },
	{ e_qcsapi_wifi_set_scs_burst_switch,		"set_scs_burst_force_switch" },
	{ e_qcsapi_wifi_get_scs_chan_pool,		"get_scs_chan_pool" },
	{ e_qcsapi_wifi_set_scs_chan_pool,		"set_scs_chan_pool" },
	{ e_qcsapi_set_acs_param,			"set_acs_param" },

	{ e_qcsapi_wifi_start_ocac,			"start_ocac" },
	{ e_qcsapi_wifi_stop_ocac,			"stop_ocac" },
	{ e_qcsapi_wifi_get_ocac_status,		"get_ocac_status" },
	{ e_qcsapi_wifi_set_ocac_threshold,		"set_ocac_thrshld" },
	{ e_qcsapi_wifi_set_ocac_dwell_time,		"set_ocac_dwell_time" },
	{ e_qcsapi_wifi_set_ocac_duration,		"set_ocac_duration" },
	{ e_qcsapi_wifi_set_ocac_cac_time,		"set_ocac_cac_time" },
	{ e_qcsapi_wifi_set_ocac_report_only,		"set_ocac_report_only" },

	{ e_qcsapi_wifi_start_dfs_s_radio,		"start_dfs_s_radio" },
	{ e_qcsapi_wifi_stop_dfs_s_radio,		"stop_dfs_s_radio" },
	{ e_qcsapi_wifi_get_dfs_s_radio_status,		"get_dfs_s_radio_status" },
	{ e_qcsapi_wifi_get_dfs_s_radio_availability,	"get_dfs_s_radio_availability" },
	{ e_qcsapi_wifi_set_dfs_s_radio_threshold,	"set_dfs_s_radio_thrshld" },
	{ e_qcsapi_wifi_set_dfs_s_radio_dwell_time,	"set_dfs_s_radio_dwell_time" },
	{ e_qcsapi_wifi_set_dfs_s_radio_duration,	"set_dfs_s_radio_duration" },
	{ e_qcsapi_wifi_set_dfs_s_radio_cac_time,	"set_dfs_s_radio_cac_time" },
	{ e_qcsapi_wifi_set_dfs_s_radio_report_only,	"set_dfs_s_radio_report_only" },
	{ e_qcsapi_wifi_set_dfs_s_radio_wea_duration,	"set_dfs_s_radio_wea_duration" },
	{ e_qcsapi_wifi_set_dfs_s_radio_wea_cac_time,	"set_dfs_s_radio_wea_cac_time" },
	{ e_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time,	"set_dfs_s_radio_wea_dwell_time" },
	{ e_qcsapi_wifi_set_dfs_s_radio_chan_off,	"set_dfs_s_radio_chan_off" },
	{ e_qcsapi_wifi_get_dfs_s_radio_chan_off,	"get_dfs_s_radio_chan_off" },

	{ e_qcsapi_wifi_set_vendor_fix,			"set_vendor_fix" },
	{ e_qcsapi_wifi_get_rts_threshold,		"get_rts_threshold" },
	{ e_qcsapi_wifi_set_rts_threshold,		"set_rts_threshold" },
	{ e_qcsapi_set_soc_macaddr,			"set_soc_macaddr" },

	{ e_qcsapi_get_interface_stats,			"get_interface_stats" },
	{ e_qcsapi_get_vap_extstats,			"get_vap_extstats" },
	{ e_qcsapi_get_phy_stats,			"get_phy_stats" },
	{ e_qcsapi_wifi_set_ap_isolate,			"set_ap_isolate" },
	{ e_qcsapi_wifi_get_ap_isolate,			"get_ap_isolate" },
	{ e_qcsapi_power_save,				"pm" },
	{ e_qcsapi_qpm_level,				"qpm_level" },
	{ e_qcsapi_reset_all_stats,			"reset_all_stats" },
	{ e_qcsapi_eth_phy_power_off,			"eth_phy_power_off" },
	{ e_qcsapi_aspm_l1,				"set_aspm_l1"},
	{ e_qcsapi_l1,					"set_l1"},
	{ e_qcsapi_telnet_enable,			"enable_telnet" },
	{ e_qcsapi_restore_default_config,		"restore_default_config" },
	{ e_qcsapi_run_script,				"run_script" },
	{ e_qcsapi_test_traffic,			"test_traffic" },
	{ e_qcsapi_get_temperature,			"get_temperature" },
	{ e_qcsapi_set_accept_oui_filter,		"set_accept_oui_filter" },
	{ e_qcsapi_get_accept_oui_filter,		"get_accept_oui_filter" },

	{ e_qcsapi_get_swfeat_list,			"get_swfeat_list" },

	{ e_qcsapi_wifi_set_vht,			"set_vht" },
	{ e_qcsapi_wifi_get_vht,			"get_vht" },
	{ e_qcsapi_wifi_set_he,				"set_he" },
	{ e_qcsapi_wifi_get_he,				"get_he" },

	{ e_qcsapi_calcmd_set_test_mode,		"set_test_mode" },
	{ e_qcsapi_calcmd_show_test_packet,		"show_test_packet" },
	{ e_qcsapi_calcmd_send_test_packet,		"send_test_packet" },
	{ e_qcsapi_calcmd_stop_test_packet,		"stop_test_packet" },
	{ e_qcsapi_calcmd_send_dc_cw_signal,		"send_dc_cw_signal" },
	{ e_qcsapi_calcmd_stop_dc_cw_signal,		"stop_dc_cw_signal" },
	{ e_qcsapi_calcmd_get_test_mode_antenna_sel,	"get_test_mode_antenna_sel" },
	{ e_qcsapi_calcmd_get_test_mode_mcs,		"get_test_mode_mcs" },
	{ e_qcsapi_calcmd_get_test_mode_bw,		"get_test_mode_bw" },
	{ e_qcsapi_calcmd_get_tx_power,			"get_test_mode_tx_power" },
	{ e_qcsapi_calcmd_set_tx_power,			"set_test_mode_tx_power" },
	{ e_qcsapi_calcmd_get_real_time_txpower,	"get_real_time_txpower" },
	{ e_qcsapi_calcmd_get_test_mode_rssi,		"get_test_mode_rssi" },
	{ e_qcsapi_calcmd_set_mac_filter,		"calcmd_set_mac_filter" },
	{ e_qcsapi_calcmd_get_antenna_count,		"get_test_mode_antenna_count" },
	{ e_qcsapi_calcmd_clear_counter,		"calcmd_clear_counter" },
	{ e_qcsapi_calcmd_get_info,			"get_info" },
	{ e_qcsapi_wifi_disable_dfs_channels,		"disable_dfs_channels" },
	{ e_qcsapi_wifi_get_dfs_channels_status,	"get_dfs_channels_status" },

	{ e_qcsapi_br_vlan_promisc,			"enable_vlan_promisc" },
	{ e_qcsapi_add_multicast,			"add_multicast"},
	{ e_qcsapi_del_multicast,			"del_multicast"},
	{ e_qcsapi_get_multicast_list,			"get_multicast_list"},
	{ e_qcsapi_add_ipff,				"add_ipff" },
	{ e_qcsapi_del_ipff,				"del_ipff" },
	{ e_qcsapi_get_ipff,				"get_ipff" },
	{ e_qcsapi_get_carrier_id,			"get_carrier_id" },
	{ e_qcsapi_set_carrier_id,			"set_carrier_id" },
	{ e_qcsapi_get_platform_id,			"get_platform_id" },
	{ e_qcsapi_get_spinor_jedecid,			"get_spinor_jedecid" },
	{ e_qcsapi_get_custom_value,			"get_custom_value" },
	{ e_qcsapi_set_custom_value,			"set_custom_value" },
	{ e_qcsapi_get_vco_lock_detect_mode,		"get_vco_lock_detect_mode" },
	{ e_qcsapi_set_vco_lock_detect_mode,		"set_vco_lock_detect_mode" },

	{ e_qcsapi_wifi_get_mlme_stats_per_mac,				"get_mlme_stats_per_mac" },
	{ e_qcsapi_wifi_get_mlme_stats_per_association,		"get_mlme_stats_per_association" },
	{ e_qcsapi_wifi_get_mlme_stats_macs_list,			"get_mlme_stats_macs_list" },

	{ e_qcsapi_get_nss_cap,				"get_nss_cap"},
	{ e_qcsapi_set_nss_cap,				"set_nss_cap"},

	{ e_qcsapi_get_security_defer_mode,		"get_security_defer_mode"},
	{ e_qcsapi_set_security_defer_mode,		"set_security_defer_mode"},
	{ e_qcsapi_apply_security_config,		"apply_security_config"},

	{ e_qcsapi_get_board_parameter,			"get_board_parameter" },
	{ e_qcsapi_wifi_set_intra_bss_isolate,		"set_intra_bss_isolate" },
	{ e_qcsapi_wifi_get_intra_bss_isolate,		"get_intra_bss_isolate" },
	{ e_qcsapi_wifi_set_bss_isolate,		"set_bss_isolate" },
	{ e_qcsapi_wifi_get_bss_isolate,		"get_bss_isolate" },

	{ e_qcsapi_wowlan_host_state,			"wowlan_host_state" },
	{ e_qcsapi_wowlan_match_type,			"wowlan_match_type" },
	{ e_qcsapi_wowlan_L2_type,			"wowlan_L2_type" },
	{ e_qcsapi_wowlan_udp_port,			"wowlan_udp_port" },
	{ e_qcsapi_wowlan_pattern,			"wowlan_pattern" },
	{ e_qcsapi_wowlan_get_host_state,		"wowlan_get_host_state" },
	{ e_qcsapi_wowlan_get_match_type,		"wowlan_get_match_type" },
	{ e_qcsapi_wowlan_get_L2_type,			"wowlan_get_L2_type" },
	{ e_qcsapi_wowlan_get_udp_port,			"wowlan_get_udp_port" },
	{ e_qcsapi_wowlan_get_pattern,			"wowlan_get_pattern" },

	{ e_qcsapi_wifi_set_extender_params,		"set_extender_params" },
	{ e_qcsapi_wifi_get_extender_status,		"get_extender_status" },

	{ e_qcsapi_wifi_enable_bgscan,			"enable_bgscan" },
	{ e_qcsapi_wifi_get_bgscan_status,		"get_bgscan_status" },

	{ e_qcsapi_get_uboot_info,			"get_uboot_info"},
	{ e_qcsapi_wifi_get_disassoc_reason,		"disassoc_reason"},

	{ e_qcsapi_is_startprod_done,			"is_startprod_done"},

	{ e_qcsapi_get_bb_param,			"get_bb_param" },
	{ e_qcsapi_set_bb_param,			"set_bb_param" },
	{ e_qcsapi_wifi_get_tx_amsdu,			"get_tx_amsdu" },
	{ e_qcsapi_wifi_set_tx_amsdu,			"set_tx_amsdu" },

	{ e_qcsapi_wifi_set_scan_buf_max_size,		"set_scan_buf_max_size" },
	{ e_qcsapi_wifi_get_scan_buf_max_size,		"get_scan_buf_max_size" },
	{ e_qcsapi_wifi_set_scan_table_max_len,		"set_scan_table_max_len" },
	{ e_qcsapi_wifi_get_scan_table_max_len,		"get_scan_table_max_len" },
	{ e_qcsapi_wifi_set_pref_band,			"set_pref_band" },
	{ e_qcsapi_wifi_get_pref_band,			"get_pref_band" },

	{ e_qcsapi_wifi_set_enable_mu,			"set_enable_mu" },
	{ e_qcsapi_wifi_get_enable_mu,			"get_enable_mu" },
	{ e_qcsapi_wifi_set_mu_use_precode,		"set_mu_use_precode" },
	{ e_qcsapi_wifi_get_mu_use_precode,		"get_mu_use_precode" },
	{ e_qcsapi_wifi_set_mu_use_eq,			"set_mu_use_eq" },
	{ e_qcsapi_wifi_get_mu_use_eq,			"get_mu_use_eq" },
	{ e_qcsapi_wifi_get_mu_groups,			"get_mu_groups" },
	{ e_qcsapi_set_emac_switch,			"set_emac_switch" },
	{ e_qcsapi_get_emac_switch,			"get_emac_switch" },
	{ e_qcsapi_eth_dscp_map,			"eth_dscp_map" },

	{ e_qcsapi_send_file,				"send_file" },
	{ e_qcsapi_wifi_verify_repeater_mode,		"verify_repeater_mode" },
	{ e_qcsapi_wifi_set_ap_interface_name,		"set_ap_interface_name" },
	{ e_qcsapi_wifi_get_ap_interface_name,		"get_ap_interface_name" },

	{ e_qcsapi_set_optim_stats,			"set_optim_stats" },

	{ e_qcsapi_set_sys_time,			"set_sys_time" },
	{ e_qcsapi_get_sys_time,			"get_sys_time" },
	{ e_qcsapi_get_eth_info,			"get_eth_info" },
	{ e_qcsapi_wifi_block_bss,			"block_bss" },
	{ e_qcsapi_wifi_set_txba_disable,		"txba_disable" },
	{ e_qcsapi_wifi_get_txba_disable,		"get_txba_disable" },
	{ e_qcsapi_wifi_set_rxba_decline,		"rxba_decline" },
	{ e_qcsapi_wifi_get_rxba_decline,		"get_rxba_decline" },
	{ e_qcsapi_wifi_set_txburst,			"set_txburst" },
	{ e_qcsapi_wifi_get_txburst,			"get_txburst" },

	{ e_qcsapi_wifi_get_sec_chan,			"get_sec_chan" },
	{ e_qcsapi_wifi_set_sec_chan,			"set_sec_chan" },
	{ e_qcsapi_wifi_set_vap_default_state,		"set_vap_default_state" },
	{ e_qcsapi_wifi_get_vap_default_state,		"get_vap_default_state" },
	{ e_qcsapi_wifi_get_tx_airtime,			"get_tx_airtime"},

	{ e_qcsapi_qwe_command,				"qwe"},

	{ e_qcsapi_get_core_dump,			"get_core_dump"},
	{ e_qcsapi_gather_info,				"gather_info"},

	{ e_qcsapi_get_client_mac_list,			"get_client_mac_list"},
	{ e_qcsapi_wifi_sample_all_clients,		"sample_all_clients"},
	{ e_qcsapi_wifi_get_per_assoc_data,		"get_sampled_assoc_data"},

	{ e_qcsapi_get_wifi_ready,			"is_wifi_ready" },

	{ e_qcsapi_get_cca_stats,			"get_cca_stats" },

	{ e_qcsapi_set_rf_chains,			"set_rf_chains" },
	{ e_qcsapi_get_rf_chains,			"get_rf_chains" },
	{ e_qcsapi_wifi_set_tx_chains,			"set_tx_chains" },
	{ e_qcsapi_wifi_get_tx_chains,			"get_tx_chains" },
	{ e_qcsapi_wifi_set_rx_chains,			"set_rx_chains" },
	{ e_qcsapi_wifi_get_rx_chains,			"get_rx_chains" },

	{ e_qcsapi_wifi_set_vapdebug,			"set_vapdebug" },
	{ e_qcsapi_wifi_get_vapdebug,			"get_vapdebug" },

	{ e_qcsapi_wifi_set_eap_reauth_period,		"set_eap_reauth_period" },
	{ e_qcsapi_wifi_get_eap_reauth_period,		"get_eap_reauth_period" },
	{ e_qcsapi_wifi_remove_eap_reauth_period,	"remove_eap_reauth_period" },

	{ e_qcsapi_wifi_set_cs_thrshld_range,		"set_cs_thrshld_range" },
	{ e_qcsapi_wifi_get_cs_thrshld_range,		"get_cs_thrshld_range" },
	{ e_qcsapi_wifi_set_cs_thrshld_inuse,		"set_cs_thrshld_inuse" },
	{ e_qcsapi_wifi_get_cs_thrshld_inuse,		"get_cs_thrshld_inuse" },

	{ e_qcsapi_get_igmp_snooping_state,		"get_igmp_snooping_state" },
	{ e_qcsapi_set_igmp_snooping_state,		"set_igmp_snooping_state" },
	{ e_qcsapi_br_get_groups,			"get_br_groups" },
	{ e_qcsapi_br_get_interfaces,			"get_br_interfaces" },
	{ e_qcsapi_bsa_get_param,			"get_bsa_param" },
	{ e_qcsapi_bsa_set_param,			"set_bsa_param" },
	{ e_qcsapi_bsa_get_param_ext,			"get_bsa_param_ext" },
	{ e_qcsapi_bsa_set_param_ext,			"set_bsa_param_ext" },
	{ e_qcsapi_wifi_set_radius_max_retries,		"set_radius_max_retries" },
	{ e_qcsapi_wifi_get_radius_max_retries,		"get_radius_max_retries" },
	{ e_qcsapi_wifi_remove_radius_max_retries,	"remove_radius_max_retries" },

	{ e_qcsapi_wifi_set_radius_num_failover,	"set_radius_num_failover" },
	{ e_qcsapi_wifi_get_radius_num_failover,	"get_radius_num_failover" },
	{ e_qcsapi_wifi_remove_radius_num_failover,	"remove_radius_num_failover" },

	{ e_qcsapi_wifi_set_radius_timeout,		"set_radius_timeout" },
	{ e_qcsapi_wifi_get_radius_timeout,		"get_radius_timeout" },
	{ e_qcsapi_wifi_remove_radius_timeout,		"remove_radius_timeout" },

	{ e_qcsapi_wifi_set_pmk_cache_enable,		"set_pmk_cache_enable" },
	{ e_qcsapi_wifi_get_pmk_cache_enable,		"get_pmk_cache_enable" },

	{ e_qcsapi_wifi_set_pmk_cache_lifetime,		"set_pmk_cache_lifetime" },
	{ e_qcsapi_wifi_get_pmk_cache_lifetime,		"get_pmk_cache_lifetime" },
	{ e_qcsapi_wifi_remove_pmk_cache_lifetime,	"remove_pmk_cache_lifetime" },

	{ e_qcsapi_wifi_set_max_auth_attempts,		"set_max_auth_attempts" },
	{ e_qcsapi_wifi_get_max_auth_attempts,		"get_max_auth_attempts" },

	{ e_qcsapi_wifi_set_lockout_period,		"set_lockout_period" },
	{ e_qcsapi_wifi_get_lockout_period,		"get_lockout_period" },
	{ e_qcsapi_wifi_remove_lockout_period,		"remove_lockout_period" },

	{ e_qcsapi_wifi_set_id_request_period,		"set_id_request_period" },
	{ e_qcsapi_wifi_get_id_request_period,		"get_id_request_period" },
	{ e_qcsapi_wifi_remove_id_request_period,	"remove_id_request_period" },

	{ e_qcsapi_wifi_set_auth_quiet_period,		"set_auth_quiet_period" },
	{ e_qcsapi_wifi_get_auth_quiet_period,		"get_auth_quiet_period" },
	{ e_qcsapi_wifi_remove_auth_quiet_period,	"remove_auth_quiet_period" },

	{ e_qcsapi_wlmonitor_enable,			"enable_link_monitor"},
	{ e_qcsapi_wlmonitor_rate_thres_config,		"set_link_monitor_rate_thres"},
	{ e_qcsapi_wlmonitor_period_thres_config,	"set_link_monitor_duration"},

	{ e_qcsapi_wifi_get_beacon_phyrate,		"get_beacon_phyrate" },
	{ e_qcsapi_wifi_set_beacon_phyrate,		"set_beacon_phyrate" },
	{ e_qcsapi_wifi_get_beacon_power_backoff,	"get_beacon_power_backoff" },
	{ e_qcsapi_wifi_set_beacon_power_backoff,	"set_beacon_power_backoff" },
	{ e_qcsapi_wifi_get_mgmt_power_backoff,		"get_mgmt_power_backoff" },
	{ e_qcsapi_wifi_set_mgmt_power_backoff,		"set_mgmt_power_backoff" },
	{ e_qcsapi_qdrv_set_hw_module_state,		"set_hw_module_state" },
	{ e_qcsapi_qdrv_get_hw_module_state,		"get_hw_module_state" },
	{ e_qcsapi_wifi_start_dcs_scan,			"start_dcs_scan"},
	{ e_qcsapi_wifi_stop_dcs_scan,			"stop_dcs_scan"},
	{ e_qcsapi_wifi_get_dcs_scan_params,		"get_dcs_scan_params"},
	{ e_qcsapi_set_vlan_loop_detect,		"set_vlan_loop_detect" },
	{ e_qcsapi_get_vlan_loop_detect,		"get_vlan_loop_detect" },

	{ e_qcsapi_bsa_get_sta_table,			"get_bsa_sta_table" },
	{ e_qcsapi_add_app_ie,				"add_app_ie" },
	{ e_qcsapi_remove_app_ie,			"remove_app_ie" },
	{ e_qcsapi_disable_11b,				"disable_11b" },

	{ e_qcsapi_wifi_get_nac_mon_mode,		"get_nac_mon_mode" },
	{ e_qcsapi_wifi_set_nac_mon_mode,		"set_nac_mon_mode" },
	{ e_qcsapi_wifi_get_nac_stats,			"get_nac_stats" },
	{ e_qcsapi_update_bootcfg_binfile,		"update_bootcfg_binfile" },

	{ e_qcsapi_wifi_set_ieee80211r,			"set_ieee80211r" },
	{ e_qcsapi_wifi_get_ieee80211r,			"get_ieee80211r" },
	{ e_qcsapi_wifi_set_11r_mobility_domain,	"set_11r_mobility_domain" },
	{ e_qcsapi_wifi_get_11r_mobility_domain,	"get_11r_mobility_domain" },
	{ e_qcsapi_wifi_set_11r_nas_id,			"set_11r_nas_id" },
	{ e_qcsapi_wifi_get_11r_nas_id,			"get_11r_nas_id" },
	{ e_qcsapi_wifi_set_11r_ft_over_ds,		"set_11r_ft_over_ds" },
	{ e_qcsapi_wifi_get_11r_ft_over_ds,		"get_11r_ft_over_ds" },
	{ e_qcsapi_wifi_add_11r_neighbour,		"add_11r_neighbour" },
	{ e_qcsapi_wifi_del_11r_neighbour,		"del_11r_neighbour" },
	{ e_qcsapi_wifi_get_11r_neighbour,		"get_11r_neighbour" },

	{ e_qcsapi_wifi_set_11r_r1_key_holder,		"set_11r_r1_key_holder" },
	{ e_qcsapi_wifi_get_11r_r1_key_holder,		"get_11r_r1_key_holder" },
	{ e_qcsapi_wifi_set_11r_r0_key_lifetime,	"set_11r_r0_key_lifetime" },
	{ e_qcsapi_wifi_get_11r_r0_key_lifetime,	"get_11r_r0_key_lifetime" },

	{ e_qcsapi_set_max_boot_cac_duration,		"set_max_boot_cac_duration"},
	{ e_qcsapi_get_icac_status,			"get_icac_status"},

	{ e_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn, "set_scs_leavedfs_chan_mtrc_mrgn" },

	{ e_qcsapi_wifi_is_weather_channel,		"is_weather_channel"},

	{ e_qcsapi_get_reboot_cause,			"get_reboot_cause"},
	{ e_qcsapi_set_radio_pwr_save,			"set_radio_pwr_save"},
	{ e_qcsapi_put_radio_under_reset,		"put_radio_under_reset"},
	{ e_qcsapi_wifi_set_pta_op_mode,		"set_pta" },
	{ e_qcsapi_wifi_get_pta,			"get_pta"},

	{ e_qcsapi_wifi_show_access_points,		"show_access_points" },
	{ e_qcsapi_reg_chan_txpower_backoff_set,	"set_chan_txpower_backoff" },
	{ e_qcsapi_reg_chan_txpower_backoff_get,	"get_chan_txpower_backoff" },

	{ e_qcsapi_grab_config,				"grab_config" },

	{ e_qcsapi_wifi_repeater_mode_cfg,		"repeater_mode_cfg"},
	{ e_qcsapi_wifi_set_urepeater_params,		"set_urepeater_params"},
	{ e_qcsapi_wifi_get_urepeater_params,		"get_urepeater_params"},

	{ e_qcsapi_save_config,				"save_config"},
	{ e_qcsapi_get_config_status,			"get_config_status"},

	{ e_qcsapi_wifi_dpp_parameter,			"dpp_param"},
	{ e_qcsapi_wifi_aacs_thres_min_tbl_set,
		"aacs_thres_min_tbl" },
	{ e_qcsapi_wifi_aacs_thres_min_tbl_get,
		"aacs_get_thres_min_tbl" },
	{ e_qcsapi_wifi_aacs_thres_max_tbl_set,
		"aacs_thres_max_tbl" },
	{ e_qcsapi_wifi_aacs_thres_max_tbl_get,
		"aacs_get_thres_max_tbl" },
	{ e_qcsapi_wifi_aacs_vnode_rssi_tbl_set,
		"aacs_vnode_rssi_tbl" },
	{ e_qcsapi_wifi_aacs_vnode_rssi_tbl_get,
		"aacs_get_vnode_rssi_table" },
	{ e_qcsapi_wifi_aacs_vnode_wgt_tbl_set,
		"aacs_vnode_wgt_tbl" },
	{ e_qcsapi_wifi_aacs_vnode_wgt_tbl_get,
		"aacs_get_vnode_wgt_tbl" },
	{ e_qcsapi_wifi_aacs_vnode_set,
		"aacs_vnode" },
	{ e_qcsapi_wifi_aacs_vnode_get,
		"aacs_get_vnodes" },
	{ e_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set,
		"aacs_dfs_thres_adj_tbl" },
	{ e_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get,
		"aacs_get_dfs_thres_adj_tbl" },
	{ e_qcsapi_wifi_aacs_excl_ch_set,
		"aacs_excl_chan" },
	{ e_qcsapi_wifi_aacs_excl_ch_get,
		"aacs_get_excl_chan" },
	{ e_qcsapi_wifi_aacs_alt_excl_ch_set,
		"aacs_alt_excl_chan" },
	{ e_qcsapi_wifi_aacs_alt_excl_ch_get,
		"aacs_get_alt_excl_chan" },
	{ e_qcsapi_wifi_aacs_sel_ch_set,
		"aacs_set_sel_chan" },
	{ e_qcsapi_wifi_aacs_sel_ch_get,
		"aacs_get_sel_chan" },
	{e_qcsapi_wifi_legacy_bbic_set,		"set_legacy_bbic"},
	{e_qcsapi_wifi_legacy_bbic_get,		"get_legacy_bbic"},

	{ e_qcsapi_nosuch_api, NULL }
};

static const struct
{
	qcsapi_counter_type	counter_type;
	const char		*counter_name;
} qcsapi_counter_name[] =
{
	{ qcsapi_total_bytes_sent,		"tx_bytes" },
	{ qcsapi_total_bytes_received,		"rx_bytes" },
	{ qcsapi_total_packets_sent,		"tx_packets" },
	{ qcsapi_total_packets_received,	"rx_packets" },
	{ qcsapi_discard_packets_sent,		"tx_discard" },
	{ qcsapi_discard_packets_received,	"rx_discard" },
	{ qcsapi_error_packets_sent,		"tx_errors" },
	{ qcsapi_error_packets_received,	"rx_errors" },
	{ qcsapi_vlan_frames_received,		"rx_vlan_pkts" },
	{ qcsapi_fragment_frames_received,	"rx_fragment_pkts" },
	{ qcsapi_nosuch_counter,		NULL }
};

static const struct
{
	qcsapi_option_type	option_type;
	const char		*option_name;
} qcsapi_option_name[] =
{
	{ qcsapi_channel_refresh,	"channel_refresh" },
	{ qcsapi_DFS,			"DFS" },
	{ qcsapi_wmm,			"WiFi_MultiMedia" },
	{ qcsapi_wmm,			"WMM" },
	{ qcsapi_beacon_advertise,	"beacon_advertise" },
	{ qcsapi_beacon_advertise,	"beacon" },
	{ qcsapi_wifi_radio,		"radio" },
	{ qcsapi_autorate_fallback,	"autorate_fallback" },
	{ qcsapi_autorate_fallback,	"autorate" },
	{ qcsapi_security,		"security" },
	{ qcsapi_SSID_broadcast,	"broadcast_SSID" },
	{ qcsapi_SSID_broadcast,	"SSID_broadcast" },
	{ qcsapi_short_GI,		"shortGI" },
	{ qcsapi_short_GI,		"short_GI" },
	{ qcsapi_802_11h,		"802_11h" },
	{ qcsapi_tpc_query,		"tpc_query" },
	{ qcsapi_dfs_fast_channel_switch, "dfs_fast_switch" },
	{ qcsapi_dfs_avoid_dfs_scan,	"avoid_dfs_scan" },
	{ qcsapi_uapsd,			"uapsd" },
	{ qcsapi_sta_dfs,		"sta_dfs" },
	{ qcsapi_specific_scan,		"specific_scan" },
	{ qcsapi_GI_probing,		"GI_probing" },
	{ qcsapi_GI_fixed,		"GI_fixed" },
	{ qcsapi_stbc,			"stbc" },
	{ qcsapi_beamforming,		"beamforming" },
	{ qcsapi_short_slot,		"short_slot" },
	{ qcsapi_short_preamble,	"short_preamble" },
	{ qcsapi_rts_cts,		"rts_cts" },
	{ qcsapi_40M_only,		"40M_bw_only" },
	{ qcsapi_obss_coexist,		"obss_coexist" },
	{ qcsapi_11g_protection,	"11g_protection" },
	{ qcsapi_11n_protection,	"11n_protection" },
	{ qcsapi_qlink,			"qlink" },
	{ qcsapi_allow_11b,		"allow_11b" },
	{ qcsapi_dyn_beacon_period,	"dyn_beacon_period" },
	{ qcsapi_acs_obss_chk,		"acs_obss_chk" },
	{ qcsapi_sta_dfs_strict,	"sta_dfs_strict" },
	{ qcsapi_bw_resume,		"bw_resume" },
	{ qcsapi_subband_radar,		"subband_radar" },
	{ qcsapi_priority_repeater,	"priority_repeater" },
	{ qcsapi_nosuch_option,		 NULL }
};

static const struct
{
	qcsapi_board_parameter_type	board_param;
	const char			*board_param_name;
} qcsapi_board_parameter_name[] =
{
	{ qcsapi_hw_revision,		"hw_revision" },
	{ qcsapi_hw_id,			"hw_id" },
	{ qcsapi_hw_desc,		"hw_desc" },
	{ qcsapi_rf_chipid,		"rf_chipid" },
	{ qcsapi_bond_opt,		"bond_opt" },
	{ qcsapi_vht,			"vht_status" },
	{ qcsapi_bandwidth,		"bw_supported" },
	{ qcsapi_spatial_stream,	"spatial_stream" },
	{ qcsapi_interface_types,	"interface_types" },
	{ qcsapi_rf_chip_verid,		"rf_chip_verid" },
	{ qcsapi_nosuch_parameter,	NULL }
};

static const struct
{
	qcsapi_rate_type	rate_type;
	const char		*rate_name;
} qcsapi_rate_types_name[] =
{
	{ qcsapi_basic_rates,		"basic_rates" },
	{ qcsapi_basic_rates,		"basic" },
	{ qcsapi_operational_rates,	"operational_rates" },
	{ qcsapi_operational_rates,	"operational" },
	{ qcsapi_possible_rates,	"possible_rates" },
	{ qcsapi_possible_rates,	"possible" },
	{ qcsapi_nosuch_rate,		NULL }
};

static const struct {
	qcsapi_mimo_type std_type;
	const char *std_name;
} qcsapi_wifi_std_name[] = {
	{qcsapi_mimo_ht, "ht"},
	{qcsapi_mimo_vht, "vht"},
	{qcsapi_mimo_he, "he"},
	{qcsapi_nosuch_standard, NULL}
};

static const struct
{
	qcsapi_flash_partiton_type	partition_type;
	const char			*partition_name;
} qcsapi_partition_name[] =
{
	{ qcsapi_image_linux_live,	"live" },
	{ qcsapi_image_linux_safety,	"safety" },
	{ qcsapi_image_uboot_live,	"uboot_live" },
	{ qcsapi_image_uboot_safety,	"uboot_safety" },
	{ qcsapi_image_uboot,		"uboot" },
	{ qcsapi_nosuch_partition,	NULL }
};

static const struct
{
	int		qos_queue_type;
	const char	*qos_queue_name;
} qcsapi_qos_queue_table[] =
{
	{ WME_AC_BE,	"BE" },
	{ WME_AC_BK,	"BK" },
	{ WME_AC_VI,	"VI" },
	{ WME_AC_VO,	"VO" },
	{ WME_AC_BE,	"besteffort" },
	{ WME_AC_BK,	"background" },
	{ WME_AC_VI,	"video" },
	{ WME_AC_VO,	"voice" }
};

static const struct
{
	const char	*fix_name;
	unsigned	fix_idx;
} qcsapi_vendor_fix_table[] =
{
	{ "brcm_dhcp",	VENDOR_FIX_IDX_BRCM_DHCP},
	{ "brcm_igmp",	VENDOR_FIX_IDX_BRCM_IGMP},
	{ "brcm_vht", VENDOR_FIX_IDX_BRCM_VHT},
	{ "cc_3ss_snd", VENDOR_FIX_IDX_CC_3SS},
	{ "cc_4ss_snd", VENDOR_FIX_IDX_CC_4SS},
	{ "cc_rfgain_pppc", VENDOR_FIX_IDX_CC_RFGAIN_PPPC},
};

static const struct
{
	int		qos_param_type;
	const char	*qos_param_name;
} qcsapi_qos_param_table[] =
{
	{ IEEE80211_WMMPARAMS_CWMIN,		"cwmin" },
	{ IEEE80211_WMMPARAMS_CWMAX,		"cwmax" },
	{ IEEE80211_WMMPARAMS_AIFS,		"aifs" },
	{ IEEE80211_WMMPARAMS_TXOPLIMIT,	"tx_op" },
	{ IEEE80211_WMMPARAMS_TXOPLIMIT,	"txoplimit" },
	{ IEEE80211_WMMPARAMS_ACM,		"acm" },
	{ IEEE80211_WMMPARAMS_NOACKPOLICY,	"noackpolicy" }
};

static const struct {
	qcsapi_per_assoc_param	pa_param;
	char			*pa_name;
} qcsapi_pa_param_table[] = {
	{QCSAPI_LINK_QUALITY,	"link_quality"},
	{QCSAPI_RSSI_DBM,	"rssi_dbm"},
	{QCSAPI_BANDWIDTH,	"bw"},
	{QCSAPI_SNR,		"snr"},
	{QCSAPI_TX_PHY_RATE,	"tx_phy_rate"},
	{QCSAPI_RX_PHY_RATE,	"rx_phy_rate"},
	{QCSAPI_STAD_CCA,	"stand_cca_req"},
	{QCSAPI_RSSI,		"rssi"},
	{QCSAPI_PHY_NOISE,	"hw_noise"},
	{QCSAPI_SOC_MAC_ADDR,	"soc_macaddr"},
	{QCSAPI_SOC_IP_ADDR,	"soc_ipaddr"},
	{QCSAPI_NODE_MEAS_BASIC,"basic"},
	{QCSAPI_NODE_MEAS_CCA,	"cca"},
	{QCSAPI_NODE_MEAS_RPI,	"rpi"},
	{QCSAPI_NODE_MEAS_CHAN_LOAD, "channel_load"},
	{QCSAPI_NODE_MEAS_NOISE_HIS, "noise_histogram"},
	{QCSAPI_NODE_MEAS_BEACON, "beacon"},
	{QCSAPI_NODE_MEAS_FRAME, "frame"},
	{QCSAPI_NODE_MEAS_TRAN_STREAM_CAT, "tran_stream_cat"},
	{QCSAPI_NODE_MEAS_MULTICAST_DIAG, "multicast_diag"},
	{QCSAPI_NODE_TPC_REP,	"tpc_report"},
	{QCSAPI_NODE_LINK_MEASURE, "link_measure"},
	{QCSAPI_NODE_NEIGHBOR_REP, "neighbor_report"},
	{QCSAPI_NODE_SGI_CAPS, "sgi_caps"},
};

static const struct{
	qcsapi_system_status bit_id;
	char *description;
} qcsapi_sys_status_table[] =
{
	{qcsapi_sys_status_ethernet, "Ethernet interface"},
	{qcsapi_sys_status_pcie_ep, "PCIE EP driver"},
	{qcsapi_sys_status_pcie_rc, "PCIE RC driver"},
	{qcsapi_sys_status_wifi, "WiFi driver"},
	{qcsapi_sys_status_rpcd, "Rpcd server"},
	{qcsapi_sys_status_cal_mode, "Calstate mode"},
	{qcsapi_sys_status_completed, "System boot up completely"},
};

static const struct{
	const char		*name;
	enum qscs_cfg_param_e	index;
} qcsapi_scs_param_names_table[] =
{
	{"scs_smpl_dwell_time",			SCS_SMPL_DWELL_TIME},
	{"scs_sample_intv",			SCS_SAMPLE_INTV},
	{"scs_sample_type",			SCS_SAMPLE_TYPE},
	{"scs_thrshld_smpl_pktnum",		SCS_THRSHLD_SMPL_PKTNUM},
	{"scs_thrshld_smpl_airtime",		SCS_THRSHLD_SMPL_AIRTIME},
	{"scs_thrshld_atten_inc",		SCS_THRSHLD_ATTEN_INC},
	{"scs_thrshld_dfs_reentry",		SCS_THRSHLD_DFS_REENTRY},
	{"scs_thrshld_dfs_reentry_minrate",	SCS_THRSHLD_DFS_REENTRY_MINRATE},
	{"scs_thrshld_dfs_reentry_intf",	SCS_THRSHLD_DFS_REENTRY_INTF},
	{"scs_thrshld_loaded",			SCS_THRSHLD_LOADED},
	{"scs_thrshld_aging_nor",		SCS_THRSHLD_AGING_NOR},
	{"scs_thrshld_aging_dfsreent",		SCS_THRSHLD_AGING_DFSREENT},
	{"scs_enable",				SCS_ENABLE},
	{"scs_debug_enable",			SCS_DEBUG_ENABLE},
	{"scs_smpl_enable",			SCS_SMPL_ENABLE},
	{"scs_smpl_enable_alone",		SCS_SMPL_ENABLE_ALONE},
	{"scs_report_only",			SCS_REPORT_ONLY},
	{"scs_cca_idle_thrshld",		SCS_CCA_IDLE_THRSHLD},
	{"scs_cca_intf_hi_thrshld",		SCS_CCA_INTF_HI_THRSHLD},
	{"scs_cca_intf_lo_thrshld",		SCS_CCA_INTF_LO_THRSHLD},
	{"scs_cca_intf_ratio",			SCS_CCA_INTF_RATIO},
	{"scs_cca_intf_dfs_margin",		SCS_CCA_INTF_DFS_MARGIN},
	{"scs_pmbl_err_thrshld",		SCS_PMBL_ERR_THRSHLD},
	{"scs_cca_sample_dur",			SCS_CCA_SAMPLE_DUR},
	{"scs_cca_intf_smth_fctr",		SCS_CCA_INTF_SMTH_NOXP},
	{"scs_cca_intf_smth_fctr",		SCS_CCA_INTF_SMTH_XPED},
	{"scs_rssi_smth_fctr",			SCS_RSSI_SMTH_UP},
	{"scs_rssi_smth_fctr",			SCS_RSSI_SMTH_DOWN},
	{"scs_chan_mtrc_mrgn",			SCS_CHAN_MTRC_MRGN},
	{"scs_atten_adjust",			SCS_ATTEN_ADJUST},
	{"scs_atten_sw_enable",			SCS_ATTEN_SW_ENABLE},
	{"scs_pmbl_err_smth_fctr",		SCS_PMBL_ERR_SMTH_FCTR},
	{"scs_pmbl_err_range",			SCS_PMBL_ERR_RANGE},
	{"scs_pmbl_err_mapped_intf_range",	SCS_PMBL_ERR_MAPPED_INTF_RANGE},
	{"scs_sp_wf",				SCS_SP_WF},
	{"scs_lp_wf",				SCS_LP_WF},
	{"scs_pmp_rpt_cca_smth_fctr",		SCS_PMP_RPT_CCA_SMTH_FCTR},
	{"scs_pmp_rx_time_smth_fctr",		SCS_PMP_RX_TIME_SMTH_FCTR},
	{"scs_pmp_tx_time_smth_fctr",		SCS_PMP_TX_TIME_SMTH_FCTR},
	{"scs_pmp_stats_stable_percent",	SCS_PMP_STATS_STABLE_PERCENT},
	{"scs_pmp_stats_stable_range",		SCS_PMP_STATS_STABLE_RANGE},
	{"scs_pmp_stats_clear_interval",	SCS_PMP_STATS_CLEAR_INTERVAL},
	{"scs_as_rx_time_smth_fctr",		SCS_AS_RX_TIME_SMTH_FCTR},
	{"scs_as_tx_time_smth_fctr",		SCS_AS_TX_TIME_SMTH_FCTR},
	{"scs_cca_idle_smth_fctr",		SCS_CCA_IDLE_SMTH_FCTR},
	{"scs_tx_time_compensation",		SCS_TX_TIME_COMPENSTATION_START},
	{"scs_rx_time_compensation",		SCS_RX_TIME_COMPENSTATION_START},
	{"scs_tdls_time_compensation",		SCS_TDLS_TIME_COMPENSTATION_START},
	{"scs_leavedfs_chan_mtrc_mrgn",		SCS_LEAVE_DFS_CHAN_MTRC_MRGN},
	{"scs_burst_enable",			SCS_BURST_ENABLE},
	{"scs_burst_window",			SCS_BURST_WINDOW},
	{"scs_burst_thresh",			SCS_BURST_THRESH},
	{"scs_burst_pause_time",		SCS_BURST_PAUSE_TIME},
	{"scs_burst_force_switch",		SCS_BURST_FORCE_SWITCH},
	{"scs_nac_monitor_mode",		SCS_NAC_MONITOR_MODE},
	{"scs_check_band_mrgn",			SCS_CHECK_BAND_MRGN},
	{"scs_out_of_band_mrgn",		SCS_OUT_OF_BAND_MRGN},
};

static const struct
{
	qcsapi_extender_type param_type;
	const char *param_name;
} qcsapi_extender_param_table[] =
{
	{qcsapi_extender_role,	"role"},
	{qcsapi_extender_mbs_best_rssi,	"mbs_best_rssi"},
	{qcsapi_extender_rbs_best_rssi,	"rbs_best_rssi"},
	{qcsapi_extender_mbs_wgt,	"mbs_wgt"},
	{qcsapi_extender_rbs_wgt,	"rbs_wgt"},
	{qcsapi_extender_roaming,	"roaming"},
	{qcsapi_extender_bgscan_interval,	"bgscan_interval"},
	{qcsapi_extender_verbose,	"verbose"},
	{qcsapi_extender_mbs_rssi_margin,	"mbs_rssi_margin"},
	{qcsapi_extender_short_retry_limit,	"short_retry"},
	{qcsapi_extender_long_retry_limit,	"long_retry"},
	{qcsapi_extender_scan_mbs_intvl,	"scan_mbs_interval"},
	{qcsapi_extender_scan_mbs_mode,	"scan_mbs_mode"},
	{qcsapi_extender_scan_mbs_expiry,	"scan_mbs_expiry"},
	{qcsapi_extender_fast_cac,	"fast_cac"},
	{qcsapi_extender_nosuch_param,	NULL},
};

static const struct {
	qcsapi_wifi_qdrv_param_type param_type;
	const char *param_name;
} qcsapi_wifi_qdrv_param_table[] = {
	{qcsapi_wifi_param_twt,		"twt"},
	{qcsapi_wifi_nosuch_parameter,	NULL},
};

static const struct {
	qcsapi_wifi_param_type param_type;
	const char *param_name;
} qcsapi_wifi_param_table[] = {
	{qcsapi_wifi_param_dtim_period,		"dtim_period"},
	{qcsapi_wifi_param_csa_count,		"csa_count"},
	{qcsapi_wifi_param_csa_mode,		"csa_mode"},
	{qcsapi_wifi_param_rrm_nr,		"rrm_nr"},
	{qcsapi_wifi_param_tx_enable,		"tx_enable"},
	{qcsapi_wifi_param_nsm_timeout,		"nsm_timeout"},
	{qcsapi_wifi_param_inact_timeout,	"inact_timeout"},
	{qcsapi_wifi_param_wcac_cfg,		"wcac_cfg"},
	{qcsapi_wifi_param_cfg_4addr,		"cfg_4addr"},
	{qcsapi_wifi_param_repeater_bcn,	"repeater_bcn"},
	{qcsapi_wifi_param_single_rtsthreshold,	"single_rts"},
	{qcsapi_wifi_param_aggr_rtsthreshold,	"aggr_rts"},
	{qcsapi_wifi_param_max_bss_num,		"max_bss_num"},
	{qcsapi_wifi_param_wcac_duration,	"wcac_duration"},
	{qcsapi_wifi_param_wcac_wea_duration,	"wcac_wea_duration"},
	{qcsapi_wifi_param_multiap_backhaul_sta,	"multiap_backhaul_sta"},
	{qcsapi_wifi_param_multi_bssid,		"multi_bssid"},
	{qcsapi_wifi_param_aacs_verbose,	"aacs_verbose"},
	{qcsapi_wifi_param_aacs_excl_tbl,	"aacs_excl_tbl"},
	{qcsapi_wifi_param_aacs_startbw,	"aacs_start_bw"},
	{qcsapi_wifi_param_aacs_startchan,	"aacs_start_chan"},
	{qcsapi_wifi_param_aacs_curr_cfg,	"aacs_current_config"},
	{qcsapi_wifi_param_aacs_avg_dl_phyrate_wgt,	"aacs_dl_phyrate_avg_wgt"},
	{qcsapi_wifi_param_aacs_inc_spf,	"aacs_inc_spf"},
	{qcsapi_wifi_param_aacs_inc_lpf,	"aacs_inc_lpf"},
	{qcsapi_wifi_param_aacs_use_thres_tbl,	"aacs_use_thres_tbl"},
	{qcsapi_wifi_param_aacs_norm_bwctrl,	"aacs_bw_ctrl"},
	{qcsapi_wifi_param_aacs_norm_bwlimit,	"aacs_bw_limit"},
	{qcsapi_wifi_param_aacs_ics_bwctrl,	"aacs_ics_bw_ctrl"},
	{qcsapi_wifi_param_aacs_ics_bwlimit,	"aacs_ics_bw_limit"},
	{qcsapi_wifi_param_aacs_alt_bwctrl,	"aacs_alt_bw_ctrl"},
	{qcsapi_wifi_param_aacs_alt_bwlimit,	"aacs_alt_bw_limit"},
	{qcsapi_wifi_param_aacs_fastmode,	"aacs_fast_mode"},
	{qcsapi_wifi_param_aacs_sw_timebase_sec,	"aacs_sw_timebase_sec"},
	{qcsapi_wifi_param_aacs_sw_timefactor,	"aacs_sw_timefactor"},
	{qcsapi_wifi_param_aacs_sw_max_sec,	"aacs_sw_max_sec"},
	{qcsapi_wifi_param_aacs_sw_reset_sec,	"aacs_sw_reset_sec"},
	{qcsapi_wifi_param_aacs_min_scan_succrate,	"aacs_min_scan_succ_rate"},
	{qcsapi_wifi_param_aacs_dec_max_sec,	"aacs_dec_max_sec"},
	{qcsapi_wifi_param_aacs_apcnt_penalty,	"aacs_apcnt_penalty"},
	{qcsapi_wifi_param_aacs_use_vnode,	"aacs_use_vnode"},
	{qcsapi_wifi_param_aacs_sel_dfs,	"aacs_select_dfs"},
	{qcsapi_wifi_param_aacs_ics_dfs,	"aacs_ics_dfs"},
	{qcsapi_wifi_param_aacs_norm_dfs,	"aacs_norm_dfs"},
	{qcsapi_wifi_param_aacs_alt_dfs,	"aacs_alt_dfs"},
	{qcsapi_wifi_param_aacs_dfs_thres,	"aacs_dfs_thres"},
	{qcsapi_wifi_param_aacs_inc_all_chan,	"aacs_inc_all_chan"},
	{qcsapi_wifi_param_aacs_alt_inc_all_chan,	"aacs_alt_inc_all_chan"},
	{qcsapi_wifi_param_aacs_enable_alt,	"aacs_enable_alt"},
	{qcsapi_wifi_param_aacs_alt_list,	"aacs_alt_list"},
	{qcsapi_wifi_param_aacs_enable_sta_d,	"aacs_enable_sta_d"},
	{qcsapi_wifi_param_aacs_allow_sta_d,	"aacs_allow_sta_d"},
	{qcsapi_wifi_param_aacs_enable_obss,	"aacs_enable_obss"},
	{qcsapi_wifi_param_pps_max_bcast,	"pps_max_bcast"},
	{qcsapi_wifi_param_pps_max_ssdp,	"pps_max_ssdp"},
	{qcsapi_wifi_param_dbvc_level,		"dbvc_level"},
	{qcsapi_wifi_param_dbvc_dwell,		"dbvc_dwell"},
	{qcsapi_wifi_nosuch_parameter,		NULL},
};

static const struct qcsapi_eth_info_result_s {
	qcsapi_eth_info_result result_type;
	const char *result_label;
	const char *result_bit_set;
	const char *result_bit_unset;
} qcsapi_eth_info_result_table[] = {
	{qcsapi_eth_info_connected,	"Connected",		"yes",		"no"},
	{qcsapi_eth_info_speed_unknown,	"Speed",		"unknown",	NULL},
	{qcsapi_eth_info_speed_10M,	"Speed",		"10Mb/s",	NULL},
	{qcsapi_eth_info_speed_100M,	"Speed",		"100Mb/s",	NULL},
	{qcsapi_eth_info_speed_1000M,	"Speed",		"1000Mb/s",	NULL},
	{qcsapi_eth_info_speed_2500M,	"Speed",		"2500Mb/s",	NULL},
	{qcsapi_eth_info_speed_10000M,	"Speed",		"10000Mb/s",	NULL},
	{qcsapi_eth_info_duplex_full,	"Duplex",		"full",		"half"},
	{qcsapi_eth_info_autoneg_on,	"Auto-negotiation",	NULL,		"disabled"},
	{qcsapi_eth_info_autoneg_success,"Auto-negotiation",	"completed",	"failed"},
};

static const struct
{
	qcsapi_eth_info_type type;
	qcsapi_eth_info_type_mask mask;
} qcsapi_eth_info_type_mask_table[] =
{
	{qcsapi_eth_info_link,		qcsapi_eth_info_link_mask},
	{qcsapi_eth_info_speed,		qcsapi_eth_info_speed_mask},
	{qcsapi_eth_info_duplex,	qcsapi_eth_info_duplex_mask},
	{qcsapi_eth_info_autoneg,	qcsapi_eth_info_autoneg_mask},
	{qcsapi_eth_info_all,		qcsapi_eth_info_all_mask},
};

static const struct
{
	qcsapi_hw_module hw_module;
	const char *hw_module_name;
} qcsapi_hw_module_table[] =
{
	{qcsapi_hw_pm_signal,	"pm_signal"},
	{qcsapi_hw_no_module,	NULL }
};

static const char *qcsapi_auth_algo_list[] = {
	"OPEN",
	"SHARED",
};

static const char *qcsapi_auth_keyproto_list[] = {
	"NONE",
	"WPA",
	"WPA2",
};

static const char *qcsapi_auth_keymgmt_list[] = {
	"NONE",
	"WPA-EAP",
	"WPA-PSK",
	"WEP",
};

static const char *qcsapi_auth_cipher_list[] = {
	"WEP",
	"TKIP",
	"OCB",
	"CCMP",
	"CMAC",
	"CKIP",
};

static const char *qcsapi_wifi_modes_strings[] = WLAN_WIFI_MODES_STRINGS;


static const char*
qcsapi_csw_reason_list[] = {
	[IEEE80211_CSW_REASON_UNKNOWN] = "UNKNOWN",
	[IEEE80211_CSW_REASON_SCS] = "SCS",
	[IEEE80211_CSW_REASON_DFS] = "DFS",
	[IEEE80211_CSW_REASON_MANUAL] = "MANUAL",
	[IEEE80211_CSW_REASON_CONFIG] = "CONFIG",
	[IEEE80211_CSW_REASON_SCAN] = "SCAN",
	[IEEE80211_CSW_REASON_OCAC] = "SDFS",
	[IEEE80211_CSW_REASON_CSA] = "CSA",
	[IEEE80211_CSW_REASON_TDLS_CS] = "TDLS",
};

static const struct {
	qcsapi_legacy_phyrate phyrate_idx;
	const char *phyrate_name;
} qcsapi_legacy_phyrate_name[] = {
	{QTN_PHY_RATE_LEGACY_6M, "6M"},
	{QTN_PHY_RATE_LEGACY_9M, "9M"},
	{QTN_PHY_RATE_LEGACY_12M, "12M"},
	{QTN_PHY_RATE_LEGACY_18M, "18M"},
	{QTN_PHY_RATE_LEGACY_24M, "24M"},
	{QTN_PHY_RATE_LEGACY_36M, "36M"},
	{QTN_PHY_RATE_LEGACY_48M, "48M"},
	{QTN_PHY_RATE_LEGACY_54M, "54M"},
	{QTN_PHY_RATE_LEGACY_1M, "1M"},
	{QTN_PHY_RATE_LEGACY_2M, "2M"},
	{QTN_PHY_RATE_LEGACY_5_5M, "5.5M"},
	{QTN_PHY_RATE_LEGACY_11M, "11M"},
	{QTN_PHY_RATE_LEGACY_INVALID, NULL}
};

static const struct {
	qcsapi_ssid_fmt fmt;
	const char *fmt_name;
} qcsapi_ssid_fmt_table[] = {
	{qcsapi_ssid_fmt_str, "str"},
	{qcsapi_ssid_fmt_hex_str, "hexstr"},
};

/*
 * Node information set labels
 */
#define QTN_NIS_LABEL_LEN	35

/*
 * Node information set labels
 * This table must be kept in sync with Node Information Set enums (e.g. qtn_nis_s0_e).
 */
const char *qtn_nis_label[][QTN_NIS_VAL_MAX] = {
	{ /* Set 0 */
	[QTN_NIS_S0_assoc_id] =		"Association ID",
	[QTN_NIS_S0_bw] =		"Bandwidth",

	[QTN_NIS_S0_tx_bytes] =		"Tx bytes",
	[QTN_NIS_S0_tx_packets] =	"Tx packets",
	[QTN_NIS_S0_tx_amsdu_msdus] =	"Tx aggregated MSDUs",
	[QTN_NIS_S0_tx_mpdus] =		"Tx MPDUs",
	[QTN_NIS_S0_tx_ppdus] =		"Tx PPDUs",
	[QTN_NIS_S0_tx_dropped] =	"Tx discards",
	[QTN_NIS_S0_tx_wifi_drop1] =	"Packets failed to transmit on AC 1",
	[QTN_NIS_S0_tx_wifi_drop2] =	"Packets failed to transmit on AC 2",
	[QTN_NIS_S0_tx_wifi_drop3] =	"Packets failed to transmit on AC 3",
	[QTN_NIS_S0_tx_wifi_drop4] =	"Packets failed to transmit on AC 4",
	[QTN_NIS_S0_tx_errors] =	"Tx errors",
	[QTN_NIS_S0_tx_ucast] =		"Tx unicast",
	[QTN_NIS_S0_tx_mcast] =		"Tx multicast",
	[QTN_NIS_S0_tx_bcast] =		"Tx broadcast",
	[QTN_NIS_S0_tx_max_phy_rate] =	"Tx max PHY rate (kbps)",
	[QTN_NIS_S0_tx_max_nss] =	"Tx max NSS",
	[QTN_NIS_S0_tx_max_mcs] =	"Tx max MCS",
	[QTN_NIS_S0_tx_last_phy_rate] =	"Tx last PHY rate (Mbps)",
	[QTN_NIS_S0_tx_median_phyrate] = "Tx median PHY rate (Mbps)",
	[QTN_NIS_S0_tx_last_nss] =	"Tx last NSS",
	[QTN_NIS_S0_tx_last_mcs] =	"Tx last MCS",
	[QTN_NIS_S0_rx_flags] =		"Rx flags",
	[QTN_NIS_S0_tx_retries] =	"Tx retries",
	[QTN_NIS_S0_tx_bw] =		"Tx bandwidth",
	[QTN_NIS_S0_tx_pppc] =		"Tx PPPC (dBm)",

	[QTN_NIS_S0_rx_bytes] =		"Rx bytes",
	[QTN_NIS_S0_rx_packets] =	"Rx packets",
	[QTN_NIS_S0_rx_amsdu_msdus] =	"Rx aggregated MSDUs",
	[QTN_NIS_S0_rx_mpdus] =		"Rx MPDUs",
	[QTN_NIS_S0_rx_ppdus] =		"Rx PPDUs",
	[QTN_NIS_S0_rx_dropped] =	"Rx discards",
	[QTN_NIS_S0_rx_errors] =	"Rx errors",
	[QTN_NIS_S0_rx_ucast] =		"Rx unicast",
	[QTN_NIS_S0_rx_mcast] =		"Rx multicast",
	[QTN_NIS_S0_rx_bcast] =		"Rx broadcast",
	[QTN_NIS_S0_rx_unknown] =	"Rx unknown data",
	[QTN_NIS_S0_rx_max_phy_rate] =	"Rx max PHY rate (kbps)",
	[QTN_NIS_S0_rx_median_phyrate] = "Rx median PHY rate (Mbps)",
	[QTN_NIS_S0_rx_max_nss] =	"Rx max NSS",
	[QTN_NIS_S0_rx_max_mcs] =	"Rx max MCS",
	[QTN_NIS_S0_rx_last_phy_rate] =	"Rx last PHY rate (Mbps)",
	[QTN_NIS_S0_rx_last_nss] =	"Rx last NSS",
	[QTN_NIS_S0_rx_last_mcs] =	"Rx last MCS",
	[QTN_NIS_S0_rx_smthd_rssi] =	"Rx smoothed RSSI (-ve)",
	[QTN_NIS_S0_rx_flags] =		"Rx flags",
	[QTN_NIS_S0_rx_retries] =	"Rx retries",
	[QTN_NIS_S0_rx_bw] =		"Rx bandwidth",
	[QTN_NIS_S0_rx_last_rssi] =	"Rx last RSSI (-ve)",
	[QTN_NIS_S0_rx_last_rssi_tot] =	"Rx last total RSSI (-ve)",
	[QTN_NIS_S0_rx_smthd_rssi_tot] ="Rx smoothed total RSSI (-ve)",
	},
	{ /* Set 1 */
	[QTN_NIS_S1_tx_tid0_bytes] =	"Tx TID0 bytes",
	[QTN_NIS_S1_tx_tid1_bytes] =	"Tx TID1 bytes",
	[QTN_NIS_S1_tx_tid2_bytes] =	"Tx TID2 bytes",
	[QTN_NIS_S1_tx_tid3_bytes] =	"Tx TID3 bytes",
	[QTN_NIS_S1_tx_tid4_bytes] =	"Tx TID4 bytes",
	[QTN_NIS_S1_tx_tid5_bytes] =	"Tx TID5 bytes",
	[QTN_NIS_S1_tx_tid6_bytes] =	"Tx TID6 bytes",
	[QTN_NIS_S1_tx_tid7_bytes] =	"Tx TID7 bytes",

	[QTN_NIS_S1_rx_tid0_bytes] =	"Rx TID0 bytes",
	[QTN_NIS_S1_rx_tid1_bytes] =	"Rx TID1 bytes",
	[QTN_NIS_S1_rx_tid2_bytes] =	"Rx TID2 bytes",
	[QTN_NIS_S1_rx_tid3_bytes] =	"Rx TID3 bytes",
	[QTN_NIS_S1_rx_tid4_bytes] =	"Rx TID4 bytes",
	[QTN_NIS_S1_rx_tid5_bytes] =	"Rx TID5 bytes",
	[QTN_NIS_S1_rx_tid6_bytes] =	"Rx TID6 bytes",
	[QTN_NIS_S1_rx_tid7_bytes] =	"Rx TID7 bytes",
	},
	{ /* Set 2 */
	[QTN_NIS_S2_mu_beamformer] =	"MU beamformer capability",
	[QTN_NIS_S2_mu_beamformee] =	"MU beamformee capability",
	[QTN_NIS_S2_su_beamformer] =	"SU beamformer capability",
	[QTN_NIS_S2_su_beamformee] =	"SU beamformee capability",
	[QTN_NIS_S2_tx_11n_bytes] =	"Tx 802.11n bytes",
	[QTN_NIS_S2_tx_11ac_su_bytes] =	"Tx 802.11ac SU bytes",
	[QTN_NIS_S2_tx_11ac_mu_bytes] =	"Tx 802.11ac MU bytes",
	[QTN_NIS_S2_tx_11ax_su_bytes] =	"Tx 802.11ax SU bytes",
	[QTN_NIS_S2_tx_11ax_mu_bytes] =	"Tx 802.11ax MU bytes",
	[QTN_NIS_S2_tx_11ax_ofdma_bytes] =	"Tx 802.11ax OFDMA SU bytes",
	[QTN_NIS_S2_tx_su_nss] =	"Tx SU beamforee NSS",
	[QTN_NIS_S2_tx_su_is_orthsnd] =	"Tx SU is orthogonal sounding",
	[QTN_NIS_S2_tx_mu_nss] =	"Tx MU beamformee NSS",
	[QTN_NIS_S2_tx_mu_is_orthsnd] =	"Tx MU is orthogonal sounding",
	[QTN_NIS_S2_tx_mu_aid] =	"Tx MU AID",
	},
	{ /* Set 3 */
	[QTN_NIS_S3_tx_amsdu_subfrms_1] =	"Tx A-MSDUs with 1 subframe",
	[QTN_NIS_S3_tx_amsdu_subfrms_2_4] =	"Tx A-MSDUs with 2 to 4 subframes",
	[QTN_NIS_S3_tx_amsdu_subfrms_5_8] =	"Tx A-MSDUs with 5 to 8 subframes",
	[QTN_NIS_S3_tx_amsdu_subfrms_9_16] =	"Tx A-MSDUs with 9 to 16 subframes",
	[QTN_NIS_S3_tx_amsdu_subfrms_17_32] =	"Tx A-MSDUs with 17 to 32 subframes",
	[QTN_NIS_S3_tx_amsdu_subfrms_gt_32] =	"Tx A-MSDUs with 33+ subframes",
	[QTN_NIS_S3_tx_amsdu_lt_2k] =		"Tx A-MSDUs smaller than 2kB",
	[QTN_NIS_S3_tx_amsdu_2k_4k] =		"Tx A-MSDUs from 2kB to 4kB",
	[QTN_NIS_S3_tx_amsdu_4k_8k] =		"Tx A-MSDUs from 4kB to 8kB",
	[QTN_NIS_S3_tx_amsdu_gt_8k] =		"Tx A-MSDUs larger than 8kB",
	[QTN_NIS_S3_rx_amsdu_subfrms_1] =	"Rx A-MSDUs with 1 subframe",
	[QTN_NIS_S3_rx_amsdu_subfrms_2_4] =	"Rx A-MSDUs with 2 to 4 subframes",
	[QTN_NIS_S3_rx_amsdu_subfrms_5_8] =	"Rx A-MSDUs with 5 to 8 subframes",
	[QTN_NIS_S3_rx_amsdu_subfrms_9_16] =	"Rx A-MSDUs with 9 to 16 subframes",
	[QTN_NIS_S3_rx_amsdu_subfrms_17_32] =	"Rx A-MSDUs with 17 to 32 subframes",
	[QTN_NIS_S3_rx_amsdu_subfrms_gt_32] =	"Rx A-MSDUs with 33+ subframes",
	[QTN_NIS_S3_rx_amsdu_lt_2k] =		"Rx A-MSDUs smaller than 2kB",
	[QTN_NIS_S3_rx_amsdu_2k_4k] =		"Rx A-MSDUs from 2kB to 4kB",
	[QTN_NIS_S3_rx_amsdu_4k_8k] =		"Rx A-MSDUs from 4kB to 8kB",
	[QTN_NIS_S3_rx_amsdu_gt_8k] =		"Rx A-MSDUs larger than 8kB",
	}
};

/*
 * All-node information set labels
 * This table must be kept in sync with All-node Information Set enums (e.g. qtn_nis_all_s0_e).
 */
const struct nis_meta_data qtn_nis_all_label[][QTN_NIS_ALL_FIELD_MAX] = {
	{ /* Set 0 */
	[QTN_NIS_ALL_S0_rsn_caps] = {QTN_NIS_VAL_RSN_CAPS, "RSN capabilities"},
	[QTN_NIS_ALL_S0_rsn_ucastcipher] = {QTN_NIS_VAL_RSN_UCASTCIPHER, "RSN unicast ciphers"},
	[QTN_NIS_ALL_S0_rsn_mcastcipher] = {QTN_NIS_VAL_RSN_MCASTCIPHER, "RSN multicast ciphers"},
	[QTN_NIS_ALL_S0_rsn_keymgmt] = {QTN_NIS_VAL_RSN_KEYMGMT, "RSN key management"},
	}
};

static const struct{
	const char		*name;
	enum qcsapi_freq_band	index;
} qcsapi_freq_band_table[] = {
	{"default",	qcsapi_freq_band_default}, /* 2.4GHz or 5GHz */
	{"2.4GHz",	qcsapi_freq_band_2pt4_ghz},
	{"4GHz",	qcsapi_freq_band_4_ghz},
	{"5GHz",	qcsapi_freq_band_5_ghz},
	{"6GHz",	qcsapi_freq_band_6_ghz},
};

/**
 * Interface information set labels
 */
#define QTNIS_IF_LABEL_LEN	35

/**
 * Interface information set labels
 * This table must be kept in sync with Interface Information Set enums (e.g. qtnis_if_s0_e).
 */
const char *qcsapi_qtnis_if_label[][QTNIS_IF_VAL_MAX] = {
	{ /* Set 0 */
	[QTNIS_S0_assoc_id] = "",
	[QTNIS_S0_bw] = "",

	[QTNIS_S0_tx_bytes] = "",
	[QTNIS_S0_tx_packets] = "",
	[QTNIS_S0_tx_amsdu_msdus] = "",
	[QTNIS_S0_tx_mpdus] = "",
	[QTNIS_S0_tx_ppdus] = "",
	[QTNIS_S0_tx_wifi_sent_be] = "",
	[QTNIS_S0_tx_wifi_sent_bk] = "",
	[QTNIS_S0_tx_wifi_sent_vi] = "",
	[QTNIS_S0_tx_wifi_sent_vo] = "",
	[QTNIS_S0_tx_dropped] = "",
	[QTNIS_S0_tx_wifi_drop_be] = "",
	[QTNIS_S0_tx_wifi_drop_bk] = "",
	[QTNIS_S0_tx_wifi_drop_vi] = "",
	[QTNIS_S0_tx_wifi_drop_vo] = "",
	[QTNIS_S0_tx_errors] = "",
	[QTNIS_S0_tx_ucast] = "",
	[QTNIS_S0_tx_mcast] = "",
	[QTNIS_S0_tx_bcast] = "",
	[QTNIS_S0_tx_max_phy_rate] = "",
	[QTNIS_S0_tx_max_nss] = "",
	[QTNIS_S0_tx_max_mcs] = "",
	[QTNIS_S0_tx_last_phy_rate] = "",
	[QTNIS_S0_tx_last_nss] = "",
	[QTNIS_S0_tx_last_mcs] = "",
	[QTNIS_S0_tx_flags] = "",
	[QTNIS_S0_tx_retries] = "",
	[QTNIS_S0_tx_bw] = "",

	[QTNIS_S0_rx_bytes] = "",
	[QTNIS_S0_rx_packets] = "",
	[QTNIS_S0_rx_amsdu_msdus] = "",
	[QTNIS_S0_rx_mpdus] = "",
	[QTNIS_S0_rx_ppdus] = "",
	[QTNIS_S0_rx_dropped] = "",
	[QTNIS_S0_rx_errors] = "",
	[QTNIS_S0_rx_ucast] = "",
	[QTNIS_S0_rx_mcast] = "",
	[QTNIS_S0_rx_bcast] = "",
	[QTNIS_S0_rx_unknown] = "",
	[QTNIS_S0_rx_max_phy_rate] = "",
	[QTNIS_S0_rx_max_nss] = "",
	[QTNIS_S0_rx_max_mcs] = "",
	[QTNIS_S0_rx_last_phy_rate] = "",
	[QTNIS_S0_rx_last_nss] = "",
	[QTNIS_S0_rx_last_mcs] = "",
	[QTNIS_S0_rx_smthd_rssi] = "",
	[QTNIS_S0_rx_flags] = "",
	[QTNIS_S0_rx_retries] = "",
	[QTNIS_S0_rx_bw] = "",
	[QTNIS_S0_rx_last_rssi] = "",
	[QTNIS_S0_rx_last_rssi_tot] = "",
	[QTNIS_S0_rx_smthd_rssi_tot] = "",
	},
	{ /* Set 1 */
	[QTNIS_S1_offset] = "",
	[QTNIS_S1_duration] = "",
	[QTNIS_S1_channel] = "",

	[QTNIS_S1_basic] = "",
	},
	{ /* Set 2 */
	[QTNIS_S2_tx_ack_failures] = "ACK failures",
	[QTNIS_S2_rx_invalid_mac_header] = "Rx with invalid MAC header",
	[QTNIS_S2_rx_non_assoc_packets] = "Rx non-assoc packets",
	[QTNIS_S2_rx_plcp_errors] = "Rx with PLCP errors",
	[QTNIS_S2_rx_fcs_errors] = "Rx with FCS errors",
	},
	{ /* Set 3 */
	[QTNIS_S3_radar_status] = "Radar status",
	[QTNIS_S3_radar_mode] = "Radar mode",
	[QTNIS_S3_radar_bw_1] = "Radar block 1 bandwidth",
	[QTNIS_S3_radar_detections_1] = "Radar block 1 detections",
	[QTNIS_S3_radar_bw_2] = "Radar block 2 bandwidth",
	[QTNIS_S3_radar_detections_2] = "Radar block 2 detections",
	}
};

static int		verbose_flag = 0;
static unsigned int	call_count = 1;
static unsigned int	delay_time = 0;

static unsigned int	internal_flags = 0;

static unsigned int	call_qcsapi_init_count = 1;

/* Prefer a non-reentrant program to allocating 1025 bytes on the stack. */
static string_1024 regulatory_list_buf;

enum {
	m_force_NULL_address = 0x01
};

char *qcsapi_hs20_params[] = {
	"hs20_wan_metrics",
	"disable_dgaf",
	"hs20_operating_class",
	"osu_ssid",
	"osen",
	"hs20_deauth_req_timeout"
};

static const qcsapi_bsa_conf_table bsa_config[] = { QCSAPI_BSA_CONF_PARAMS };
static const struct acs_conf_table{
	qcsapi_acs_param_type param_type;
	char *parameter_name;
}acs_config[] = {
	{qcsapi_acs_param_obss_chk, "obss_chk"},
};

/* returns 1 if successful; 0 if failure */
static int name_to_entry_point_enum(char *lookup_name, qcsapi_entry_point *p_entry_point)
{
	int retval = 1;
	int found_entry = 0, proposed_enum = (int)e_qcsapi_nosuch_api;
	unsigned int iter;
	/*
	 *Silently skip over "qscapi_" ...
	 */
	if (strncasecmp(lookup_name, "qscapi_", 7) == 0)
		lookup_name += 7;

	for (iter = 0; qcsapi_entry_name[iter].api_name != NULL && found_entry == 0; iter++) {
		if (strcasecmp(qcsapi_entry_name[iter].api_name, lookup_name) == 0) {
			found_entry = 1;
			*p_entry_point = qcsapi_entry_name[iter].e_entry_point;
		}
	}

	if (found_entry == 0) {
		*p_entry_point = proposed_enum;
		retval = 0;
	}

	return retval;
}

/* Guaranteed to return a valid string address */
static const char *entry_point_enum_to_name(qcsapi_entry_point e_qcsapi_entry_point)
{
	const char *retaddr = "No such QCSAPI";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_entry_name[iter].api_name != NULL && found_entry == 0; iter++) {
		if (qcsapi_entry_name[iter].e_entry_point == e_qcsapi_entry_point) {
			found_entry = 1;
			retaddr = qcsapi_entry_name[iter].api_name;
		}
	}

	return retaddr;
}

static void list_entry_point_names(qcsapi_output *print)
{
	unsigned int iter;

	print_out(print, "API entry point names (more than one name can refer to the same entry point):\n");

	for (iter = 0; qcsapi_entry_name[iter].api_name != NULL; iter++) {
		print_out(print, "\t%s\n", qcsapi_entry_name[iter].api_name);
	}
}

static void grep_entry_point_names(qcsapi_output *print, const char *reg)
{
	unsigned int iter;

	print_out(print, "API entry point names (more than one name can refer to the same entry point):\n");

	for (iter = 0; qcsapi_entry_name[iter].api_name != NULL; iter++) {
		if (strstr(qcsapi_entry_name[iter].api_name, reg))
			print_out(print, "\t%s\n", qcsapi_entry_name[iter].api_name);
	}
}

/* returns 1 if successful; 0 if failure */
static int name_to_counter_enum(char *lookup_name, qcsapi_counter_type *p_counter_type)
{
	int retval = 0;
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_counter_name[iter].counter_name != NULL && found_entry == 0; iter++) {
		if (strcasecmp(qcsapi_counter_name[iter].counter_name, lookup_name) == 0) {
			found_entry = 1;
			*p_counter_type = qcsapi_counter_name[iter].counter_type;
		}
	}

	if (found_entry)
		retval = 1;

	return retval;
}

/* Guaranteed to return a valid string address */
static const char *counter_enum_to_name(qcsapi_counter_type the_counter_type)
{
	const char *retaddr = "No such QCSAPI counter";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_counter_name[iter].counter_name != NULL && found_entry == 0; iter++) {
		if (qcsapi_counter_name[iter].counter_type == the_counter_type) {
			found_entry = 1;
			retaddr = qcsapi_counter_name[iter].counter_name;
		}
	}

	return retaddr;
}

static void list_counter_names(qcsapi_output *print)
{
	unsigned int iter;

	print_out(print, "API counters:\n");

	for (iter = 0; qcsapi_counter_name[iter].counter_name != NULL; iter++) {
		print_out(print, "\t%s\n", qcsapi_counter_name[iter].counter_name);
	}
}

static void list_per_node_param_names(qcsapi_output *print)
{
	unsigned int iter;

	print_out(print, "Per-node parameters:\n");

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_pa_param_table); iter++) {
		print_out(print, "\t%s\n", qcsapi_pa_param_table[iter].pa_name);
	}
}

/* returns 1 if successful; 0 if failure */
static int name_to_option_enum(char *lookup_name, qcsapi_option_type *p_option)
{
	int retval = 0;
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_option_name[iter].option_name != NULL && found_entry == 0; iter++) {
		if (strcasecmp(qcsapi_option_name[iter].option_name, lookup_name) == 0) {
			found_entry = 1;
			*p_option = qcsapi_option_name[iter].option_type;
		}
	}

	if (found_entry)
		retval = 1;

	return retval;
}

/* Guaranteed to return a valid string address */
static const char *option_enum_to_name(qcsapi_option_type the_option_type)
{
	const char *retaddr = "No such QCSAPI option";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_option_name[iter].option_name != NULL && found_entry == 0; iter++) {
		if (qcsapi_option_name[iter].option_type == the_option_type) {
			found_entry = 1;
			retaddr = qcsapi_option_name[iter].option_name;
		}
	}

	return retaddr;
}

static void list_option_names(qcsapi_output *print)
{
	unsigned int iter;

	print_out(print, "API options (more than one name can refer to the same option):\n");

	for (iter = 0; qcsapi_option_name[iter].option_name != NULL; iter++) {
		print_out(print, "\t%s\n", qcsapi_option_name[iter].option_name);
	}
}

/* returns 1 if successful; 0 if failure */
static int
name_to_board_parameter_enum(char *lookup_name, qcsapi_board_parameter_type *p_boardparam)
{
	int retval = 0;
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_board_parameter_name[iter].board_param_name != NULL
			&& (found_entry == 0); iter++) {
		if (strcasecmp(qcsapi_board_parameter_name[iter].board_param_name,
						lookup_name) == 0) {
			found_entry = 1;
			*p_boardparam = qcsapi_board_parameter_name[iter].board_param;
		}
	}

	if (found_entry)
		retval = 1;

	return retval;
}

static const char *board_paramter_enum_to_name(qcsapi_board_parameter_type the_board_param)
{
	const char *retaddr = "No such QCSAPI option";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_board_parameter_name[iter].board_param_name != NULL
			&& found_entry == 0; iter++) {
		if (qcsapi_board_parameter_name[iter].board_param == the_board_param) {
			found_entry = 1;
			retaddr = qcsapi_board_parameter_name[iter].board_param_name;
		}
	}

	return retaddr;
}

static const char *
local_dpp_parameter_enum_to_name(enum qcsapi_dpp_cmd_param_type dpp_param)
{
	const char *retaddr = "No such QCSAPI option";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_dpp_param_table[iter].param_name != NULL && found_entry == 0;
			iter++) {
		if (qcsapi_dpp_param_table[iter].param_type == dpp_param) {
			found_entry = 1;
			retaddr = qcsapi_dpp_param_table[iter].param_name;
		}
	}

	return retaddr;
}

static void list_board_parameter_names(qcsapi_output *print)
{
	unsigned int iter;

	for (iter = 0; qcsapi_board_parameter_name[iter].board_param_name != NULL; iter++) {
		print_out(print, "\t%s\n", qcsapi_board_parameter_name[iter].board_param_name);
	}
}

static void list_wifi_parameter_names(qcsapi_output *print)
{
	uint32_t i;

	for (i = 0; qcsapi_wifi_param_table[i].param_name != NULL; i++) {
		print_out(print, "\t%s\n", qcsapi_wifi_param_table[i].param_name);
	}

	for (i = 0; qcsapi_wifi_qdrv_param_table[i].param_name != NULL; i++)
		print_out(print, "\t%s\n", qcsapi_wifi_qdrv_param_table[i].param_name);
}

/* returns 1 if successful; 0 if failure */
static int name_to_rates_enum(char *lookup_name, qcsapi_rate_type *p_rates)
{
	int retval = 0;
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_rate_types_name[iter].rate_name != NULL && found_entry == 0; iter++) {
		if (strcasecmp(qcsapi_rate_types_name[iter].rate_name, lookup_name) == 0) {
			found_entry = 1;
			*p_rates = qcsapi_rate_types_name[iter].rate_type;
		}
	}

	if (found_entry)
		retval = 1;

	return retval;
}

/* Guaranteed to return a valid string address */
static const char *rates_enum_to_name(qcsapi_rate_type the_option_type)
{
	const char *retaddr = "No such type of rates";
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_rate_types_name[iter].rate_name != NULL && found_entry == 0; iter++) {
		if (qcsapi_rate_types_name[iter].rate_type == the_option_type) {
			found_entry = 1;
			retaddr = qcsapi_rate_types_name[iter].rate_name;
		}
	}

	return retaddr;
}

static int name_to_wifi_std_enum(const char *lookup_name, qcsapi_mimo_type *p_modulation)
{
	unsigned int iter = 0;
	unsigned int found_entry = 0;

	while (qcsapi_wifi_std_name[iter].std_name && !found_entry) {
		if (!strcasecmp(qcsapi_wifi_std_name[iter].std_name, lookup_name)) {
			*p_modulation = qcsapi_wifi_std_name[iter].std_type;
			found_entry = 1;
		}
		++iter;
	}

	return found_entry;
}

static const char *wifi_std_enum_to_name(const qcsapi_mimo_type lookup_type)
{
	unsigned int iter = 0;
	const char *ret_name = "No such type of standard";

	while (qcsapi_wifi_std_name[iter].std_name) {
		if (qcsapi_wifi_std_name[iter].std_type == lookup_type) {
			ret_name = qcsapi_wifi_std_name[iter].std_name;
			break;
		}
		++iter;
	}

	return ret_name;
}

static int name_to_legacy_phyrate_enum(const char *lookup_name, qcsapi_legacy_phyrate *phyrate_idx)
{
	unsigned int iter = 0;
	unsigned int found_entry = 0;

	while (qcsapi_legacy_phyrate_name[iter].phyrate_name && !found_entry) {
		if (!strcasecmp(qcsapi_legacy_phyrate_name[iter].phyrate_name, lookup_name)) {
			*phyrate_idx = qcsapi_legacy_phyrate_name[iter].phyrate_idx;
			found_entry = 1;
		}
		++iter;
	}

	return found_entry;
}

static const char *legacy_phyrate_enum_to_name(const qcsapi_legacy_phyrate lookup_idx)
{
	unsigned int iter = 0;
	const char *ret_name = "No such legacy PHY rate";

	while (qcsapi_legacy_phyrate_name[iter].phyrate_name) {
		if (qcsapi_legacy_phyrate_name[iter].phyrate_idx == lookup_idx) {
			ret_name = qcsapi_legacy_phyrate_name[iter].phyrate_name;
			break;
		}
		++iter;
	}

	return ret_name;
}

/* returns 1 if successful; 0 if failure */
static int name_to_partition_type(char *lookup_name, qcsapi_flash_partiton_type *p_partition_type)
{
	int retval = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_partition_name[iter].partition_name != NULL && retval == 0; iter++) {
		if (strcasecmp(qcsapi_partition_name[iter].partition_name, lookup_name) == 0) {
			retval = 1;
			*p_partition_type = qcsapi_partition_name[iter].partition_type;
		}
	}

	return retval;
}

static int name_to_qos_queue_type(char *lookup_name, int *p_qos_queue_type)
{
	int retval = 0;
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_qos_queue_table); iter++) {
		if (strcasecmp(qcsapi_qos_queue_table[iter].qos_queue_name, lookup_name) == 0) {
			*p_qos_queue_type = qcsapi_qos_queue_table[iter].qos_queue_type;
			retval = 1;
			break;
		}
	}

	return retval;
}

static int name_to_qos_param_type(char *lookup_name, int *p_qos_param_type)
{
	int retval = 0;
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_qos_param_table); iter++) {
		if (strcasecmp(qcsapi_qos_param_table[iter].qos_param_name, lookup_name) == 0) {
			*p_qos_param_type = qcsapi_qos_param_table[iter].qos_param_type;
			retval = 1;
			break;
		}
	}

	return retval;
}

static int name_to_vendor_fix_idx(char *lookup_name, int *p_vendor_fix_idx)
{
	int retval = 0;
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_vendor_fix_table); iter++) {
		if (strcasecmp(qcsapi_vendor_fix_table[iter].fix_name, lookup_name) == 0) {
			*p_vendor_fix_idx = qcsapi_vendor_fix_table[iter].fix_idx;
			retval = 1;
			break;
		}
	}

	return retval;
}

static int name_to_per_assoc_parameter(const char *param_name,
		qcsapi_per_assoc_param *p_per_assoc_param)
{
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_pa_param_table); iter++) {
		if (strcasecmp(qcsapi_pa_param_table[iter].pa_name, param_name) == 0) {
			*p_per_assoc_param = qcsapi_pa_param_table[iter].pa_param;
			return 1;
		}
	}

	return 0;
}

/* returns 1 if successful; 0 if failure */
static int name_to_hw_module_enum(char *lookup_name, qcsapi_hw_module *p_hw_module)
{
	int retval = 0;
	int found_entry = 0;
	unsigned int iter;

	for (iter = 0; qcsapi_hw_module_table[iter].hw_module_name != NULL && found_entry == 0;
			iter++) {
		if (strcasecmp(qcsapi_hw_module_table[iter].hw_module_name, lookup_name) == 0) {
			found_entry = 1;
			*p_hw_module = qcsapi_hw_module_table[iter].hw_module;
		}
	}

	if (found_entry)
		retval = 1;

	return retval;
}

static int parse_local_remote_flag(qcsapi_output *print, const char *local_remote_str,
		int *p_local_remote_flag)
{
	int local_remote_flag = QCSAPI_LOCAL_NODE;

	if (isdigit(local_remote_str[0])) {
		local_remote_flag = atoi(local_remote_str);
	} else if (strcasecmp(local_remote_str, "remote") == 0) {
		local_remote_flag = QCSAPI_REMOTE_NODE;
	} else if (strcasecmp(local_remote_str, "local") == 0) {
		local_remote_flag = QCSAPI_LOCAL_NODE;
	} else {
		print_err(print, "Invalid value %s for local/remote flag\n", local_remote_str);
		return -1;
	}

	*p_local_remote_flag = local_remote_flag;
	return 0;
}

static int name_to_extender_param_enum(char *lookup_name, qcsapi_extender_type *p_extender_type)
{
	unsigned int iter;

	for (iter = 0; qcsapi_extender_param_table[iter].param_name != NULL; iter++) {
		if (strcasecmp(qcsapi_extender_param_table[iter].param_name, lookup_name) == 0) {
			*p_extender_type = qcsapi_extender_param_table[iter].param_type;
			return 1;
		}
	}

	return 0;
}

static int name_to_wifi_param_enum(char *lookup_name, qcsapi_wifi_param_type *p_type)
{
	uint32_t iter;

	for (iter = 0; qcsapi_wifi_param_table[iter].param_name != NULL; iter++) {
		if (strcasecmp(qcsapi_wifi_param_table[iter].param_name, lookup_name) == 0) {
			*p_type = qcsapi_wifi_param_table[iter].param_type;
			return 1;
		}
	}

	for (iter = 0; qcsapi_wifi_qdrv_param_table[iter].param_name != NULL; iter++) {
		if (strcasecmp(qcsapi_wifi_qdrv_param_table[iter].param_name, lookup_name) == 0) {
			*p_type = qcsapi_wifi_qdrv_param_table[iter].param_type;
			return 1;
		}
	}

	return 0;
}

static int name_to_dpp_param_enum(char *lookup_name, enum qcsapi_dpp_cmd_param_type *p_type)
{
	uint32_t iter;

	for (iter = 0; qcsapi_dpp_param_table[iter].param_name != NULL; iter++) {
		if (strcasecmp(qcsapi_dpp_param_table[iter].param_name,
				lookup_name) == 0) {
			*p_type = qcsapi_dpp_param_table[iter].param_type;
			return 1;
		}
	}

	return 0;
}

static int
parse_generic_parameter_name(qcsapi_output *print, char *generic_parameter_name,
		qcsapi_generic_parameter *p_generic_parameter)
{
	int retval = 1;
	qcsapi_unsigned_int tmpuval = 0;
	qcsapi_extender_type *p_extender_type = NULL;
	qcsapi_wifi_param_type *p_wifi_type = NULL;
	enum qcsapi_dpp_cmd_param_type *p_dpp_type = NULL;

	switch (p_generic_parameter->generic_parameter_type) {
	case e_qcsapi_option:
		retval = name_to_option_enum(generic_parameter_name,
				&(p_generic_parameter->parameter_type.option));
		if (retval == 0)
			print_err(print, "Invalid QCSAPI option %s\n", generic_parameter_name);
		break;
	case e_qcsapi_counter:
		retval = name_to_counter_enum(generic_parameter_name,
				&(p_generic_parameter->parameter_type.counter));
		if (retval == 0)
			print_err(print, "Invalid QCSAPI counter %s\n", generic_parameter_name);
		break;
	case e_qcsapi_rates:
		retval = name_to_rates_enum(generic_parameter_name,
				&(p_generic_parameter->parameter_type.typeof_rates));
		if (retval == 0)
			print_err(print, "Invalid QCSAPI type of rates %s\n",
					generic_parameter_name);
		break;
	case e_qcsapi_modulation:
		retval = name_to_wifi_std_enum(generic_parameter_name,
				&p_generic_parameter->parameter_type.modulation);
		if (!retval)
			print_err(print, "Invalid QCSAPI MIMO modulation %s\n",
					generic_parameter_name);
		break;
	case e_qcsapi_index:
	case e_qcsapi_LED:
		if (!isdigit(generic_parameter_name[0])) {
			if (e_qcsapi_option == e_qcsapi_LED) {
				print_err(print, "LED must be a numeric value\n");
			} else {
				print_err(print, "Node index must be a numeric value\n");
			}
			retval = 0;
		}

		tmpuval = (qcsapi_unsigned_int) atoi(generic_parameter_name);
		if (p_generic_parameter->generic_parameter_type == e_qcsapi_LED
				&& tmpuval > QCSAPI_MAX_LED) {
			print_err(print, "Invalid QSCAPI LED %u\n", tmpuval);
			retval = 0;
		} else
			p_generic_parameter->index =
					(qcsapi_unsigned_int) atoi(generic_parameter_name);
		break;
	case e_qcsapi_select_SSID:
	case e_qcsapi_SSID_index:
		/*
		 *APIs with generic parameter type of e_qcsapi_SSID_index expect both an SSID and
		 *an index.  Get the SSID now. Get the index in the individual call_qcsapi
		 *routines.
		 */
		strncpy(&(p_generic_parameter->parameter_type.the_SSID[0]),
				generic_parameter_name,
				sizeof(p_generic_parameter->parameter_type.the_SSID) - 1);
		p_generic_parameter->parameter_type.the_SSID[sizeof(p_generic_parameter->
						parameter_type.the_SSID) - 1] = '\0';
		break;
	case e_qcsapi_file_path_config:
		if (strcasecmp("security", generic_parameter_name) != 0) {
			print_err(print, "Invalid QCSAPI file path configuration %s\n",
					generic_parameter_name);
			retval = 0;
		} else
			p_generic_parameter->index =
					(qcsapi_unsigned_int) qcsapi_security_configuration_path;
		break;
	case e_qcsapi_board_parameter:
		retval = name_to_board_parameter_enum(generic_parameter_name,
				&(p_generic_parameter->parameter_type.board_param));
		if (retval == 0)
			print_err(print, "Invalid QCSAPI option %s\n", generic_parameter_name);
		break;
	case e_qcsapi_extender_params:
		p_extender_type = &(p_generic_parameter->parameter_type.type_of_extender);
		retval = name_to_extender_param_enum(generic_parameter_name, p_extender_type);
		if (retval == 0)
			print_err(print, "Invalid QCSAPI extender param %s\n",
					generic_parameter_name);
		break;
	case e_qcsapi_legacy_phyrate:
		retval = name_to_legacy_phyrate_enum(generic_parameter_name,
				&p_generic_parameter->parameter_type.phyrate);
		if (!retval)
			print_err(print, "Invalid legacy PHY rate %s\n", generic_parameter_name);
		break;
	case e_qcsapi_wifi_parameter:
		p_wifi_type = &(p_generic_parameter->parameter_type.wifi_param_type);
		retval = name_to_wifi_param_enum(generic_parameter_name, p_wifi_type);
		if (retval == 0)
			print_err(print, "Invalid QCSAPI wifi parameter %s\n",
					generic_parameter_name);
		break;
	case e_qcsapi_hw_module:
		retval = name_to_hw_module_enum(generic_parameter_name,
				&p_generic_parameter->parameter_type.hw_module);
		if (!retval)
			print_err(print, "Invalid hw module %s\n", generic_parameter_name);
		break;
	case e_qcsapi_dpp_parameter:
		p_dpp_type = &(p_generic_parameter->parameter_type.dpp_param_type);
		retval = name_to_dpp_param_enum(generic_parameter_name, p_dpp_type);
		if (retval == 0)
			print_err(print, "Invalid QCSAPI DPP parameter %s\n",
					generic_parameter_name);
		break;
	case e_qcsapi_none:
	default:
		print_err(print, "Programming error in parse generic parameter name:\n");
		if (p_generic_parameter->generic_parameter_type == e_qcsapi_none) {
			print_err(print, "Called with generic parameter type of none.\n");
		} else {
			print_err(print, "Called with unknown parameter type %d.\n",
					p_generic_parameter->generic_parameter_type);
		}
		retval = 0;
		break;
	}

	return retval;
}

static const char *wifi_mode_to_string(qcsapi_output *print, qcsapi_wifi_mode current_wifi_mode)
{
	const char *retaddr = "Unknown WIFI mode";

	switch (current_wifi_mode) {
	case qcsapi_mode_not_defined:
		retaddr = "WIFI mode not defined";
		break;
	case qcsapi_access_point:
		retaddr = "Access point";
		break;
	case qcsapi_station:
		retaddr = "Station";
		break;
	case qcsapi_nosuch_mode:
	default:
		print_out(print, "Unknown WIFI mode\n");
		break;
	}

	return retaddr;
}

static qcsapi_wifi_mode string_to_wifi_mode(const char *str)
{
	if (strcasecmp(str, "ap") == 0) {
		return qcsapi_access_point;
	} else if (strcasecmp(str, "access_point") == 0) {
		return qcsapi_access_point;
	} else if (strcasecmp(str, "access point") == 0) {
		return qcsapi_access_point;
	} else if (strcasecmp(str, "sta") == 0) {
		return qcsapi_station;
	} else if (strcasecmp(str, "station") == 0) {
		return qcsapi_station;
	} else if (strcasecmp(str, "repeater") == 0) {
		return qcsapi_repeater;
	} else {
		return qcsapi_nosuch_mode;
	}
}

static qcsapi_pref_band string_to_wifi_band(const char *str)
{
	if (strcasecmp(str, "2.4ghz") == 0) {
		return qcsapi_band_2_4ghz;
	} else if (strcasecmp(str, "5ghz") == 0) {
		return qcsapi_band_5ghz;
	} else {
		return qcsapi_nosuch_band;
	}
}

static int string_to_list(qcsapi_output *print, void *input_str, uint8_t *output_list,
		unsigned int *number)
{
	uint8_t list_number = 0;
	char *pcur = NULL, *pend = NULL;
	char buffer[256] = { 0 };
	char *input_end;
	int single_len = 0;

	if (!input_str || !output_list || !number)
		return -EINVAL;

	input_end = input_str + strnlen(input_str, 1024);
	pcur = input_str;
	do {
		pend = strchr(pcur, ',');
		if (pend) {
			single_len = pend - pcur;
			strncpy(buffer, pcur, single_len);
			buffer[single_len] = 0;
			pend++;
			output_list[list_number++] = atoi(buffer);
			pcur = pend;
		} else if (pcur) {
			output_list[list_number++] = atoi(pcur);
		}
	} while (pend && pend < input_end);

	*number = list_number;

	return 0;
}

static int
dump_generic_parameter_name(qcsapi_output *print, qcsapi_generic_parameter *p_generic_parameter)
{
	int retval = 1;
	int dpp_param_type;

	switch (p_generic_parameter->generic_parameter_type) {
	case e_qcsapi_option:
		print_out(print, "%s",
				option_enum_to_name(p_generic_parameter->parameter_type.option));
		break;
	case e_qcsapi_counter:
		print_out(print, "%s",
				counter_enum_to_name(p_generic_parameter->parameter_type.counter));
		break;
	case e_qcsapi_rates:
		print_out(print, "%s",
				rates_enum_to_name(p_generic_parameter->parameter_type.
						typeof_rates));
		break;
	case e_qcsapi_modulation:
		print_out(print, "%s",
				wifi_std_enum_to_name(p_generic_parameter->parameter_type.
						modulation));
		break;
	case e_qcsapi_index:
	case e_qcsapi_LED:
		print_out(print, "%u", p_generic_parameter->index);
		break;
	case e_qcsapi_file_path_config:
		print_out(print, "security");
		break;
	case e_qcsapi_select_SSID:
	case e_qcsapi_SSID_index:
		print_out(print, "%s", &(p_generic_parameter->parameter_type.the_SSID[0]));
		break;
	case e_qcsapi_board_parameter:
		print_out(print, "%s",
				board_paramter_enum_to_name(p_generic_parameter->parameter_type.
						board_param));
		break;
	case e_qcsapi_legacy_phyrate:
		print_out(print, "%s",
				legacy_phyrate_enum_to_name(p_generic_parameter->parameter_type.
						phyrate));
		break;
	case e_qcsapi_dpp_parameter:
		dpp_param_type = p_generic_parameter->parameter_type.dpp_param_type;
		print_out(print, "%s", local_dpp_parameter_enum_to_name(dpp_param_type));
		break;
	case e_qcsapi_none:
	default:
		print_out(print, "Programming error in dump generic parameter name:\n");
		if (p_generic_parameter->generic_parameter_type == e_qcsapi_none) {
			print_out(print, "Called with generic parameter type of none.\n");
		} else {
			print_out(print, "Called with unknown parameter type %d.\n",
					p_generic_parameter->generic_parameter_type);
		}
		retval = 0;
		break;
	}

	return retval;
}

static void dump_mac_addr(qcsapi_output *print, qcsapi_mac_addr mac_addr)
{
	print_out(print, MACSTR "\n", MAC2STR(mac_addr));
}

static void dump_data_array(qcsapi_output *print, uint8_t *data, int size, int order,
		char delimiter)
{
	int i;

	if (data == NULL)
		return;

	i = 0;
	if (order == 10) {
		do {
			print_out(print, "%d%c", data[i], delimiter);
			i++;
		} while (i < (size - 1));
		print_out(print, "%d", data[i]);
	} else {
		do {
			print_out(print, "0x%x%c", data[i], delimiter);
			i++;
		} while (i < (size - 1));
		print_out(print, "0x%x", data[i]);
	}

	print_out(print, "\n");
}

static void dump_scs_param(qcsapi_output *print, qcsapi_scs_param_rpt *p_rpt)
{
#define MAX_SCS_PARAM_DESC 35
	int j, loop;
	uint32_t str_len = 0;
	const char *name;
	uint32_t index;

	for (j = 0; j < ARRAY_SIZE(qcsapi_scs_param_names_table); j++) {
		name = qcsapi_scs_param_names_table[j].name;
		index = qcsapi_scs_param_names_table[j].index;

		str_len = min(strlen(name), strlen("scs_tdls_time_compensation"));
		if (!strncmp(name, "scs_tx_time_compensation", str_len) ||
				!strncmp(name, "scs_rx_time_compensation", str_len) ||
				!strncmp(name, "scs_tdls_time_compensation", str_len)) {
			print_out(print, "%-*s ", MAX_SCS_PARAM_DESC, name);
			loop = SCS_MAX_TXTIME_COMP_INDEX;
			do {
				print_out(print, "%u ", p_rpt[index++].scs_cfg_param);
				loop--;
			} while (loop);
			print_out(print, "\n");
		} else {
			if (p_rpt[index].scs_signed_param_flag == 0) {
				print_out(print, "%-*s %u\n", MAX_SCS_PARAM_DESC, name,
						p_rpt[index].scs_cfg_param);
			} else if (p_rpt[index].scs_signed_param_flag == 1) {
				print_out(print, "%-*s %d\n", MAX_SCS_PARAM_DESC, name,
						p_rpt[index].scs_cfg_param);
			} else {
				print_out(print, "invalid param flag!\n");
			}
		}
	}
}

static void
report_qcsapi_error(const call_qcsapi_bundle *p_calling_bundle, const int qcsapi_errorval)
{
	char error_msg[128] = { '\0' };
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_errno_get_message(qcsapi_errorval, &error_msg[0], sizeof(error_msg));
	print_out(print, "QCS API error %d: %s\n", 0 - qcsapi_errorval, &error_msg[0]);
}

static void
qcsapi_report_parameter_count(const call_qcsapi_bundle *p_calling_bundle, const int num)
{
	print_out(p_calling_bundle->caller_output,
			"Not enough parameters in call qcsapi %s, count is %d\n",
			entry_point_enum_to_name(p_calling_bundle->caller_qcsapi), num);
}

static void qcsapi_report_usage(const call_qcsapi_bundle *p_calling_bundle, const char *params)
{
	print_err(p_calling_bundle->caller_output,
			"Usage: call_qcsapi %s %s\n",
			entry_point_enum_to_name(p_calling_bundle->caller_qcsapi), params);
}

static int qcsapi_report_complete(const call_qcsapi_bundle *p_calling_bundle, int qcsapi_retval)
{
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "complete\n");
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
qcsapi_report_str_or_error(const call_qcsapi_bundle *p_calling_bundle, int qcsapi_retval,
		const char *str)
{
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, str);
			if (str[strlen(str) - 1] != '\n')
				print_out(p_calling_bundle->caller_output, "\n");
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static void print_start_scan_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n"
			"    call_qcsapi start_scan <interface> <algorithm> <channels> <flags>\n"
			"Parameters:\n"
			"    <algorithm> reentry, clearest (default), no_pick, background\n"
			"    <channels>  dfs, non_dfs, all (default)\n"
			"    <flags>     flush, active, fast, normal, slow, auto, check\n");
}

static void print_cancel_scan_usage(qcsapi_output *print)
{
	print_out(print, "Usage: call_qcsapi cancel_scan <interface> [force]\n");
}

static int safe_atou32(char *str, uint32_t *p, qcsapi_output *print, uint32_t min, uint32_t max)
{
	uint32_t v;

	if (qcsapi_verify_numeric(str) < 0 || qcsapi_str_to_uint32(str, &v) < 0) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", str);
		return 0;
	}

	if (v < min || v > max) {
		print_err(print, "Invalid parameter %s - value must be between %u and %u\n", str,
				min, max);
		return 0;
	}

	*p = v;

	return 1;
}

static int safe_atou16(char *str, uint16_t *p, qcsapi_output *print, uint16_t min, uint16_t max)
{
	uint32_t v;

	if (safe_atou32(str, &v, print, min, max)) {
		*p = (uint16_t) v;
		return 1;
	}

	return 0;
}

static const char *csw_reason_to_string(uint32_t reason_id)
{
	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_csw_reason_list) == IEEE80211_CSW_REASON_MAX);

	if (reason_id < ARRAY_SIZE(qcsapi_csw_reason_list))
		return qcsapi_csw_reason_list[reason_id];

	return qcsapi_csw_reason_list[IEEE80211_CSW_REASON_UNKNOWN];
}

static int radio_id_param_parse_input(qcsapi_output *print,
		const char *param, qcsapi_unsigned_int *radio_id)
{
	int ret = 0;
	long int parsed_val;
	char *tail_p;

	errno = 0;
	parsed_val = strtol(param, &tail_p, 0);

	if (errno != 0) {
		print_err(print, "Couldn't parse radio ID %s: %s\n", param, strerror(errno));
		return errno;
	}

	if (param == tail_p || *tail_p != '\0' || parsed_val < 0) {
		print_err(print, "Invalid radio ID %s\n", param);
		return 1;
	}

	*radio_id = (qcsapi_unsigned_int) parsed_val;

	return ret;
}

static int name_to_ssid_fmt_enum(const char *parameter,
		qcsapi_ssid_fmt *ssid_fmt)
{
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_ssid_fmt_table); iter++) {
		if (strcasecmp(qcsapi_ssid_fmt_table[iter].fmt_name,
					parameter) == 0) {
			*ssid_fmt = qcsapi_ssid_fmt_table[iter].fmt;
			return 0;
		}
	}

	return -EINVAL;
}

/* interface programs to call individual QCSAPIs */

#define QCSAPI_MAX_ERR_MSG_BUF 80
static int
call_qcsapi_errno_get_message(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	char *error_str = NULL;
	int qcsapi_errorval = 0;
	unsigned int message_size = QCSAPI_MAX_ERR_MSG_BUF;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle,
				"<returned error value> <size of message buffer>\n");
		return 1;
	}

	if (argc >= 2) {
		uint32_t usr_input = 0;

		if (qcsapi_str_to_uint32(argv[1], &usr_input)) {
			print_err(print, "Invalid parameter %s - must be an unsigned integer\n",
					argv[1]);
			return 1;
		}

		message_size = (usr_input > QCSAPI_MSG_BUFSIZE) ? QCSAPI_MSG_BUFSIZE : usr_input;
	}

	error_str = malloc(message_size);
	if (error_str == NULL) {
		print_err(print, "Failed to allocate %u chars\n", message_size);
		return 1;
	}

	qcsapi_errorval = atoi(argv[0]);
	qcsapi_retval = qcsapi_errno_get_message(qcsapi_errorval, error_str, message_size);

	if (qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, error_str))
		statval = 1;

	free(error_str);

	return statval;
}

static int
call_qcsapi_store_ipaddr(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t ipaddr;
	uint32_t netmask;
	int netmask_len;
	char *slash;
	char *usage = "Usage: call_qcsapi store_ipaddr <ip_address>[/<netmask>]\n";

	if (argc != 1) {
		print_out(print, usage);
		return -EINVAL;
	}

	slash = strstr(argv[0], "/");
	if (slash == NULL) {
		netmask = htonl(0xFFFFFF00);
	} else {
		*slash = '\0';
		netmask_len = atoi(slash + 1);
		if (netmask_len < 1 || netmask_len > 32) {
			print_err(print, "invalid network mask %s\n", slash + 1);
			return -EINVAL;
		}
		netmask = htonl(~((1 << (32 - netmask_len)) - 1));
	}

	if (inet_pton(AF_INET, argv[0], &ipaddr) != 1) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	if (ipaddr == 0) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_store_ipaddr(ipaddr, netmask);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return 0;
}

static int
call_qcsapi_interface_enable(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> {0 | 1}\n");
		statval = 1;
	} else {
		uint32_t enable_flag = 0;

		if (safe_atou32(argv[0], &enable_flag, print, 0, 1) == 0)
			return 1;
		/*
		 *This program is a model for all programs that call a QCSAPI.
		 *If the verbose flag is less than 0, do not report nominal (non-error) results.
		 *
		 *Like this, one can test for aging (sockets, files not closed) without
		 *seemingly endless output of "complete", etc.
		 *
		 *And if you want to see that output, just avoid enabling quiet mode.
		 *
		 *Errors though are ALWAYS reported (else how can you see if the aging test
		 *failed?) And keep trying the test; we may want to ensure a test case that is
		 *expected to cause an error does not itself have aging problems.
		 */
		qcsapi_retval = qcsapi_interface_enable(the_interface, enable_flag);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_interface_get_BSSID(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_mac_addr the_mac_addr;
	int retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	retval = qcsapi_interface_get_BSSID(the_interface, the_mac_addr);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return retval;
	}

	dump_mac_addr(print, the_mac_addr);

	return 0;
}

static int
call_qcsapi_interface_get_mac_addr(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_mac_addr the_mac_addr;
	int retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	retval = qcsapi_interface_get_mac_addr(the_interface, the_mac_addr);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return retval;
	}

	dump_mac_addr(print, the_mac_addr);

	return 0;
}

static int
call_qcsapi_interface_set_mac_addr(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi interface set mac address, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		qcsapi_mac_addr the_mac_addr;
		const char *the_interface = p_calling_bundle->caller_interface;

		if (strcmp(argv[0], "NULL") == 0)
			qcsapi_retval = qcsapi_interface_set_mac_addr(the_interface, NULL);
		else {
			int ival = parse_mac_addr(argv[0], the_mac_addr);
			if (ival >= 0)
				qcsapi_retval = qcsapi_interface_set_mac_addr(the_interface,
						the_mac_addr);
			else {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}

			if (ival >= 0) {
				if (qcsapi_retval >= 0) {
					if (verbose_flag >= 0) {
						print_out(print, "complete\n");
					}
				} else {
					report_qcsapi_error(p_calling_bundle, qcsapi_retval);
					statval = 1;
				}
			}
		}
	}

	return statval;
}

static int
call_qcsapi_interface_get_counter(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int counter_value;
	qcsapi_unsigned_int *p_counter_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_counter_type the_counter_type =
			p_calling_bundle->caller_generic_parameter.parameter_type.counter;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_counter_value = &counter_value;

	qcsapi_retval = qcsapi_interface_get_counter(the_interface, the_counter_type,
			p_counter_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", (unsigned int)counter_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_interface_get_counter64(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	uint64_t counter_value;
	uint64_t *p_counter_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_counter_type the_counter_type =
			p_calling_bundle->caller_generic_parameter.parameter_type.counter;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_counter_value = &counter_value;

	qcsapi_retval = qcsapi_interface_get_counter64(the_interface, the_counter_type,
			p_counter_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%llu\n", counter_value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_pm_get_counter(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int counter_value;
	qcsapi_unsigned_int *p_counter_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_counter_type the_counter_type =
			p_calling_bundle->caller_generic_parameter.parameter_type.counter;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_pm_interval = NULL;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi pm_get_counter <WiFi interface> <counter> <PM interval>\n");
		return 1;
	}

	if (strcmp(argv[0], "NULL") != 0) {
		the_pm_interval = argv[0];
	}

	if (argc < 2 || (strcmp(argv[1], "NULL") != 0)) {
		p_counter_value = &counter_value;
	}

	qcsapi_retval = qcsapi_pm_get_counter(the_interface, the_counter_type, the_pm_interval,
			p_counter_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", (unsigned int)counter_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_pm_get_elapsed_time(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_pm_interval = NULL;
	qcsapi_unsigned_int elapsed_time;
	qcsapi_unsigned_int *p_elapsed_time = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi pm_get_elapsed_time <PM interval>\n");
		return 1;
	}

	if (strcmp(argv[0], "NULL") != 0) {
		the_pm_interval = argv[0];
	}

	if (argc < 2 || (strcmp(argv[1], "NULL") != 0)) {
		p_elapsed_time = &elapsed_time;
	}

	qcsapi_retval = qcsapi_pm_get_elapsed_time(the_pm_interval, p_elapsed_time);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", (unsigned int)elapsed_time);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_update_bootcfg_binfile(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<dst> <path_src>");
		return 1;
	}

	qcsapi_retval = qcsapi_update_bootcfg_binfile(argv[0], argv[1]);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_flash_image_update(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_flash_partiton_type partition_type = qcsapi_nosuch_partition;
	const char *image_file_path = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"<image file path> {live | safety | uboot}\n");
		statval = 1;
	} else {
		if (strcmp(argv[0], "NULL") != 0) {
			image_file_path = argv[0];
		}

		if (name_to_partition_type(argv[1], &partition_type) == 0) {
			print_err(print, "Unrecognized flash memory partition type %s\n", argv[1]);
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_flash_image_update(image_file_path, partition_type);
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

#define GET_FIRMWARE_VERSION_MAX_LEN	40

static int
call_qcsapi_firmware_get_version(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	char firmware_version[GET_FIRMWARE_VERSION_MAX_LEN];
	char *p_firmware_version = &firmware_version[0];
	qcsapi_unsigned_int version_size = GET_FIRMWARE_VERSION_MAX_LEN;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_firmware_version = NULL;
		} else if (isdigit(argv[0][0])) {
			version_size = atoi(argv[0]);

			if (version_size > GET_FIRMWARE_VERSION_MAX_LEN) {
				version_size = GET_FIRMWARE_VERSION_MAX_LEN;
			}
		}
	}

	qcsapi_retval = qcsapi_firmware_get_version(p_firmware_version, version_size);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_firmware_version);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_system_get_time_since_start(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int time_since_startup;
	qcsapi_unsigned_int *p_time_since_startup = &time_since_startup;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0) {
		p_time_since_startup = NULL;
	}

	qcsapi_retval = qcsapi_system_get_time_since_start(p_time_since_startup);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", time_since_startup);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_system_status(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int status;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_get_system_status(&status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%X\n", status);
			int id;
			for (id = 0; id < ARRAY_SIZE(qcsapi_sys_status_table); id++) {
				print_out(print, "bit %-2d - %s\n",
						qcsapi_sys_status_table[id].bit_id,
						qcsapi_sys_status_table[id].description);
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_cpu_usage(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_256 cpu_usage = { 0 };
	qcsapi_retval = qcsapi_get_cpu_usage(cpu_usage);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", cpu_usage);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_memory_usage(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_256 memory_usage = { 0 };
	qcsapi_retval = qcsapi_get_memory_usage(memory_usage);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", memory_usage);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_random_seed(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_512bytes *random_buf;
	int i;

	random_buf = malloc(sizeof(*random_buf));

	if (!random_buf) {
		print_err(print, "Failed to allocate %u bytes\n", sizeof(*random_buf));
		return 1;
	}

	qcsapi_retval = qcsapi_get_random_seed(random_buf);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			for (i = 0; i < sizeof(random_buf->data); i++) {
				print_out(print, "%c", random_buf->data[i]);
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	free(random_buf);

	return statval;
}

static int
call_qcsapi_set_random_seed(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_512bytes *random_buf;
	qcsapi_unsigned_int entropy = 0;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_random_seed <random_string> <entropy>\n");
		return 1;
	}

	entropy = atoi(argv[1]);

	random_buf = malloc(sizeof(*random_buf));

	if (!random_buf) {
		print_err(print, "Failed to allocate %u bytes\n", sizeof(*random_buf));
		return 1;
	}

	memset(random_buf, 0, sizeof(*random_buf));
	memcpy((void *)random_buf->data, (void *)argv[0],
			min(sizeof(random_buf->data), strlen(argv[0])));

	qcsapi_retval = qcsapi_set_random_seed(random_buf, entropy);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	free(random_buf);

	return statval;
}

static int call_qcsapi_led_get(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint8_t the_led = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
	uint8_t led_value, *p_led_value = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_led_value = &led_value;

	qcsapi_retval = qcsapi_led_get(the_led, p_led_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", led_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_led_set(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi LED set, count is %d\n",
				argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		uint8_t the_led = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
		uint8_t new_value = (uint8_t) atoi(argv[0]);

		qcsapi_retval = qcsapi_led_set(the_led, new_value);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_led_pwm_enable(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint8_t led_ident = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
	qcsapi_unsigned_int onoff = 0;
	qcsapi_unsigned_int high_count = 0;
	qcsapi_unsigned_int low_count = 0;

	if (argc < 1)
		goto usage;
	if (sscanf(argv[0], "%u", &onoff) != 1)
		goto usage;
	if (onoff != 0 && argc < 3)
		goto usage;
	if (onoff != 0) {
		if (sscanf(argv[1], "%u", &high_count) != 1)
			goto usage;
		if (sscanf(argv[2], "%u", &low_count) != 1)
			goto usage;
	}

	qcsapi_retval = qcsapi_led_pwm_enable(led_ident, (uint8_t) onoff, high_count, low_count);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	print_err(print, "Usage: call_qcsapi set_LED_PWM <led_ident> (1|0) <high_count> <low_count>\n");
	statval = 1;

	return statval;
}

static int
call_qcsapi_led_brightness(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint8_t led_ident = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
	qcsapi_unsigned_int level = 0;

	if (argc < 1)
		goto usage;
	if (sscanf(argv[0], "%u", &level) != 1)
		goto usage;

	qcsapi_retval = qcsapi_led_brightness(led_ident, level);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	print_err(print, "Usage: call_qcsapi set_LED_brightness <led_ident> <level>\n");
	statval = 1;

	return statval;
}

static int
call_qcsapi_gpio_get_config(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint8_t the_gpio = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
	qcsapi_gpio_config gpio_config, *p_gpio_config = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_gpio_config = &gpio_config;

	qcsapi_retval = qcsapi_gpio_get_config(the_gpio, p_gpio_config);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", gpio_config);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_gpio_set_config(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi GPIO set config, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		uint8_t the_gpio = (uint8_t) (p_calling_bundle->caller_generic_parameter.index);
		qcsapi_gpio_config new_value = (qcsapi_gpio_config) atoi(argv[0]);

		qcsapi_retval = qcsapi_gpio_set_config(the_gpio, new_value);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_gpio_enable_wps_push_button(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi GPIO enable wps push button, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		uint8_t use_interrupt_flag = 0;
		uint8_t wps_push_button =
				(uint8_t) (p_calling_bundle->caller_generic_parameter.index);
		uint8_t active_logic = (uint8_t) atoi(argv[0]);

		if (argc > 1 && strcasecmp(argv[1], "intr") == 0)
			use_interrupt_flag = 1;

		qcsapi_retval = qcsapi_gpio_enable_wps_push_button(wps_push_button, active_logic,
				use_interrupt_flag);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_file_path_get_config(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_file_path_config the_file_path_config =
			(qcsapi_file_path_config) (p_calling_bundle->caller_generic_parameter.
			index);
	char file_path[80], *p_file_path = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_file_path = &file_path[0];

	qcsapi_retval = qcsapi_file_path_get_config(the_file_path_config, p_file_path,
			sizeof(file_path));
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &file_path[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_file_path_set_config(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi file path set config, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		qcsapi_file_path_config the_file_path_config =
				(qcsapi_file_path_config) (p_calling_bundle->
				caller_generic_parameter.index);

		qcsapi_retval = qcsapi_file_path_set_config(the_file_path_config, argv[0]);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_wifi_macaddr(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi file path set config, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_mac_addr new_mac_addr;
		int qcsapi_retval;
		int ival = 0;

		if (strcmp("NULL", argv[0]) == 0)
			qcsapi_retval = qcsapi_wifi_set_wifi_macaddr(NULL);
		else {
			ival = parse_mac_addr(argv[0], new_mac_addr);
			if (ival >= 0)
				qcsapi_retval = qcsapi_wifi_set_wifi_macaddr(new_mac_addr);
			else {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}
		}

		if (ival >= 0) {
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_create_restricted_bss(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mac_addr mac_addr = { 0 };

	if (argc == 1) {
		qcsapi_retval = parse_mac_addr(argv[0], mac_addr);
		if (qcsapi_retval < 0) {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	if (qcsapi_retval >= 0) {
		qcsapi_retval = qcsapi_wifi_create_restricted_bss(the_interface, mac_addr);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_create_bss(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mac_addr mac_addr = { 0 };

	if (argc == 1) {
		qcsapi_retval = parse_mac_addr(argv[0], mac_addr);
		if (qcsapi_retval < 0) {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	if (qcsapi_retval >= 0) {
		qcsapi_retval = qcsapi_wifi_create_bss(the_interface, mac_addr);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_remove_bss(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_remove_bss(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_primary_interface(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	char ifname[IFNAMSIZ];
	qcsapi_unsigned_int radio_id = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	memset(ifname, 0, IFNAMSIZ);
	qcsapi_retval = qcsapi_radio_get_primary_interface(radio_id, ifname, IFNAMSIZ - 1);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", ifname);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_interface_by_index(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	char ifname[IFNAMSIZ];
	qcsapi_unsigned_int if_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int radio_id = 0;

	if (argc > 0) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	memset(ifname, 0, IFNAMSIZ);
	qcsapi_retval = qcsapi_radio_get_interface_by_index(radio_id, if_index, ifname,
			IFNAMSIZ - 1);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", ifname);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_interface_by_index_all(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	char ifname[IFNAMSIZ];
	qcsapi_unsigned_int if_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int radio_id = 0;

	if (argc == 0) {
		qcsapi_report_usage(p_calling_bundle, "<if_index> <radio_id>");
		return 1;
	}

	if (radio_id_param_parse_input(print, argv[0], &radio_id))
		return 1;

	memset(ifname, 0, IFNAMSIZ);
	qcsapi_retval = qcsapi_radio_get_interface_by_index_all(radio_id,
			if_index, ifname, IFNAMSIZ - 1);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", ifname);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_mode(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_wifi_mode current_wifi_mode, *p_wifi_mode = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_wifi_mode = &current_wifi_mode;
	qcsapi_retval = qcsapi_wifi_get_mode(the_interface, p_wifi_mode);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 1) {
			print_out(print, "%d (%s)\n", (int)current_wifi_mode,
					wifi_mode_to_string(print, current_wifi_mode));
		} else if (verbose_flag >= 0) {
			print_out(print, "%s\n", wifi_mode_to_string(print, current_wifi_mode));
		}
		/*
		 *Else display nothing in quiet mode (verbose flag < 0)
		 */
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mode(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_wifi_mode new_wifi_mode;

		new_wifi_mode = string_to_wifi_mode(argv[0]);

		if (new_wifi_mode == qcsapi_nosuch_mode) {
			print_err(print, "Unrecognized WiFi mode %s\n", argv[0]);
			statval = 1;
			return statval;
		}

		qcsapi_retval = qcsapi_wifi_set_mode(the_interface, new_wifi_mode);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_phy_mode(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_64 phy_mode;

	if (argc > 0 && (strcmp(argv[0], "NULL") == 0)) {
		qcsapi_retval = -EFAULT;
	} else {
		memset(phy_mode, 0, sizeof(phy_mode));
		qcsapi_retval = qcsapi_wifi_get_phy_mode(the_interface, phy_mode);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", phy_mode);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_phy_mode(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set phy mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		const char *mode = argv[0];

		qcsapi_retval = qcsapi_wifi_set_phy_mode(the_interface, mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_phy_mode_required(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_64 phymode_req;

	memset(phymode_req, 0, sizeof(phymode_req));
	qcsapi_retval = qcsapi_wifi_get_phy_mode_required(the_interface, phymode_req);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", phymode_req);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_phy_mode_required(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> {none | 11n | 11ac | 11ax}\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_phy_mode_required(the_interface, argv[0]);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_reload_in_mode(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi reload in mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_wifi_mode new_wifi_mode;

		new_wifi_mode = string_to_wifi_mode(argv[0]);

		if (new_wifi_mode == qcsapi_nosuch_mode) {
			print_err(print, "Unrecognized WiFi mode %s\n", argv[0]);
			statval = 1;
			return statval;
		}

		qcsapi_retval = qcsapi_wifi_reload_in_mode(the_interface, new_wifi_mode);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_rfenable(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int len, i;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi rfenable, count is %d\n",
				argc);
		statval = 1;
	} else {
		int qcsapi_retval;

		len = strlen(argv[0]);
		for (i = 0; i < len; i++) {
			if (isdigit(argv[0][i]) == 0) {
				print_err(print, "Numerical parameter is required\n");
				statval = 1;
				return statval;
			}
		}

		qcsapi_unsigned_int onoff = ! !atoi(argv[0]);
		qcsapi_retval = qcsapi_radio_rfenable(the_interface, onoff);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_rfstatus(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int rfstatus = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_radio_rfstatus(the_interface, &rfstatus);
	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", rfstatus ? "On" : "Off");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_startprod(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_startprod();

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_supported_freq_bands(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_32 bands = { 0 };

	qcsapi_retval = qcsapi_wifi_get_supported_freq_bands(the_interface, bands);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", bands);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bw(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int current_bw = 0, *p_bw = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_bw = &current_bw;
	qcsapi_retval = qcsapi_wifi_get_bw(the_interface, p_bw);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", current_bw);
		}
		/*
		 *Else display nothing in quiet mode (verbose flag < 0)
		 */
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bw(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> {40 | 20}\n");
		statval = 1;
	} else {
		qcsapi_unsigned_int current_bw = (qcsapi_unsigned_int) atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_bw(the_interface, current_bw);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_24g_bw(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int current_bw = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_24g_bw(the_interface, &current_bw);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", current_bw);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_24g_bw(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int current_bw = (qcsapi_unsigned_int) atoi(argv[0]);
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_24g_bw <WiFi interface> {40 | 20}\n");
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_wifi_set_24g_bw(the_interface, current_bw);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_BSSID(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr the_mac_addr;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_BSSID(the_interface, NULL);
	else
		qcsapi_retval = qcsapi_wifi_get_BSSID(the_interface, the_mac_addr);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			dump_mac_addr(p_calling_bundle->caller_output, the_mac_addr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_config_BSSID(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr the_mac_addr;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0) {
		qcsapi_retval = qcsapi_wifi_get_config_BSSID(the_interface, NULL);
	} else {
		qcsapi_retval = qcsapi_wifi_get_config_BSSID(the_interface, the_mac_addr);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			dump_mac_addr(p_calling_bundle->caller_output, the_mac_addr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_ssid_get_bssid(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr the_mac_addr;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	const char *SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	qcsapi_retval = qcsapi_wifi_ssid_get_bssid(the_interface, SSID, the_mac_addr);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			dump_mac_addr(p_calling_bundle->caller_output, the_mac_addr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_ssid_set_bssid(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr the_mac_addr;
	int qcsapi_retval = 0;
	int ival = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	const char *SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
	qcsapi_output *print = p_calling_bundle->caller_output;

	ival = parse_mac_addr(argv[0], the_mac_addr);

	if (ival >= 0) {
		qcsapi_retval = qcsapi_wifi_ssid_set_bssid(the_interface, SSID, the_mac_addr);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}

	} else {
		print_out(print, "Error parsing MAC address %s\n", argv[0]);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_SSID(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_SSID current_SSID;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	memset(current_SSID, 0, sizeof(current_SSID));
	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_SSID(the_interface, NULL);
	else
		qcsapi_retval = qcsapi_wifi_get_SSID(the_interface, current_SSID);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", current_SSID);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_SSID2(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_SSID2 current_SSID;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_ssid_fmt fmt;

	memset(current_SSID, 0, sizeof(current_SSID));
	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_SSID2(the_interface, NULL, &fmt);
	else
		qcsapi_retval = qcsapi_wifi_get_SSID2(the_interface, current_SSID, &fmt);

	if (qcsapi_retval >= 0) {
		print_out(print, (fmt == qcsapi_ssid_fmt_str) ? "\"%s\"\n" : "%s\n", current_SSID);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_SSID(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> [SSID fmt] <SSID>";
	qcsapi_ssid_fmt fmt;
	int argc_index;
	char *new_SSID;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		statval = 1;
	} else {
		argc_index = 0;
		if (argc < 2) {
			new_SSID = argv[argc_index];
			qcsapi_retval = qcsapi_wifi_set_SSID(the_interface, new_SSID);
		} else {
			if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) < 0) {
				qcsapi_report_usage(p_calling_bundle, usage);
				return 1;
			} else {
				argc_index++;
				new_SSID = argv[argc_index];
				qcsapi_retval = qcsapi_wifi_set_SSID2(the_interface, new_SSID, fmt);
			}
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int channel_value, *p_channel_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_channel_value = &channel_value;
	qcsapi_retval = qcsapi_wifi_get_channel(the_interface, p_channel_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", channel_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_chan(call_qcsapi_bundle *p_calling_bundle,
			int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int band_value;
	qcsapi_unsigned_int channel_value;
	qcsapi_unsigned_int bandwidth_value;
	int qcsapi_retval;

	qcsapi_retval = qcsapi_wifi_get_chan(the_interface,
					&channel_value, &bandwidth_value, &band_value);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return qcsapi_retval;
	}

	if (verbose_flag >= 0)
		print_out(print, "%s %d %d\n", qcsapi_freq_band_table[band_value].name,
					channel_value, bandwidth_value);

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_set_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <channel>\n");
		statval = 1;
	} else {
		qcsapi_unsigned_int channel_value = atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_channel(the_interface, channel_value);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_chan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int channel;
	qcsapi_unsigned_int bw;
	qcsapi_unsigned_int band = qcsapi_freq_band_unknown;
	unsigned int iter;
	int qcsapi_retval;

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle,
			"<WiFi interface> <channel> <bandwidth> <freq_band>\n");
		return 1;
	}

	errno = 0;

	channel = strtol(argv[0], NULL, 10);
	if (errno)
		return 1;

	bw = strtol(argv[1], NULL, 10);
	if (errno)
		return 1;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_freq_band_table); iter++) {
		if (strcasecmp(qcsapi_freq_band_table[iter].name, argv[2]) == 0) {
			band = qcsapi_freq_band_table[iter].index;
			break;
		}
	}
	if (band == qcsapi_freq_band_unknown) {
		print_err(print, "Unknown frequency band\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_chan(the_interface, channel, bw, band);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return qcsapi_retval;
	}

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_get_channel_and_bw(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	struct qcsapi_data_32bytes chan_bw;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	memset(&chan_bw, 0, sizeof(struct qcsapi_data_32bytes));
	qcsapi_retval = qcsapi_wifi_get_channel_and_bw(the_interface, &chan_bw);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", chan_bw.data);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_set_channel_and_bw(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int chan;
	qcsapi_unsigned_int bw;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"Not enough parameters, format is <chan> <bw>\n");
		return -1;
	}

	errno = 0;
	chan = strtol(argv[0], NULL, 10);
	if (errno)
		return 1;
	bw = strtol(argv[1], NULL, 10);
	if (errno)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_channel_and_bw(the_interface, chan, bw);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_set_wea_cac_en(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int en_value;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if ((safe_atou32(argv[0], &en_value, print, 0, 1)) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_wea_cac_en(the_interface, en_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_auto_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	char channel_value_str[QCSAPI_SHORT_PARAM_VAL_LEN] = {0};
	qcsapi_unsigned_int current_channel;

	qcsapi_retval = qcsapi_config_get_parameter(the_interface,
			"channel", channel_value_str, QCSAPI_SHORT_PARAM_VAL_LEN);

	if (qcsapi_retval >= 0) {
		sscanf(channel_value_str, "%u", &current_channel);

		if (verbose_flag >= 0) {
			print_out(print, "%s\n", current_channel == 0 ? "enabled" : "disabled");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_auto_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int current_channel;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char channel_value_str[QCSAPI_SHORT_PARAM_VAL_LEN] = {0};
	char *param = argv[0];

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set auto channel,"
				" count is %d\n", argc);
		statval = 1;
		return statval;
	}

	qcsapi_retval = qcsapi_config_get_parameter(the_interface,
			"channel", channel_value_str, QCSAPI_SHORT_PARAM_VAL_LEN);

	if (qcsapi_retval >= 0) {
		sscanf(channel_value_str, "%u", &current_channel);
	}

	if (qcsapi_retval >= 0 && strncmp(param, "enable", strlen(param)) == 0) {
		if (current_channel > 0) {
			qcsapi_retval = qcsapi_config_update_parameter(the_interface, "channel",
					"0");
			if (qcsapi_retval >= 0) {
				qcsapi_retval = qcsapi_wifi_set_channel(the_interface, 0);
			}
		}
	} else if (qcsapi_retval >= 0 && strncmp(param, "disable", strlen(param)) == 0) {
		if (current_channel == 0) {
			qcsapi_retval = qcsapi_wifi_get_channel(the_interface, &current_channel);
			if (qcsapi_retval >= 0) {
				sprintf(channel_value_str, "%u", current_channel);
				qcsapi_retval = qcsapi_config_update_parameter(the_interface,
						"channel", channel_value_str);
			}
		}
	} else if (qcsapi_retval >= 0) {
		qcsapi_retval = -qcsapi_parameter_not_found;
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_chan_off(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int channel_value;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	uint32_t disable = 1;

	if (argc != 2) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <channel> {0 | 1}");
		return 1;
	}

	if (safe_atou32(argv[0], &channel_value, print, QCSAPI_MIN_CHANNEL,
					QCSAPI_MAX_CHANNEL) == 0)
		return 1;

	if (safe_atou32(argv[1], &disable, print, 0, 1) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_chan_off(the_interface, channel_value, disable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_chan_pri_inactive(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int i;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct ieee80211_inactive_chanlist the_list_channels;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_256bytes) >=
			sizeof(struct ieee80211_inactive_chanlist));

	memset(&the_list_channels, 0, sizeof(the_list_channels));

	if (argc != 0) {
		print_out(print, "call_qcsapi get_chan_pri_inactive wifi0\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_chan_pri_inactive(the_interface,
			(struct qcsapi_data_256bytes *)&the_list_channels);

	if (qcsapi_retval >= 0) {
		for (i = 1; i < IEEE80211_CHAN_MAX; i++) {
			if (the_list_channels.channels[i] & CHAN_PRI_INACTIVE_CFG_USER_OVERRIDE) {
				print_out(print, "%d%s,", i,
						(the_list_channels.
								channels[i] &
								CHAN_PRI_INACTIVE_CFG_AUTOCHAN_ONLY)
						? "(auto)" : "");
			}
		}
		print_out(print, "\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_chan_pri_inactive(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int channel_value = atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int inactive = 1;
		uint32_t flags = 0;

		if (argc >= 2) {
			inactive = atoi(argv[1]);
		}
		if (argc >= 3) {
			if (!strcasecmp(argv[2], "autochan")) {
				flags = QCSAPI_CHAN_PRI_INACTIVE_AUTOCHAN;
			} else {
				print_err(print, "Invalid parameter. Usage:\n");
				print_out(print, "call_qcsapi set_chan_pri_inactive wifi0 <channel> <inactive> <option>\n" "option should be autochan if it is present\n");
				return 1;
			}
		}

		qcsapi_retval = qcsapi_wifi_set_chan_pri_inactive_ext(the_interface,
				channel_value, inactive, flags);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_chan_disabled(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes chan_list;
	const char *the_interface = p_calling_bundle->caller_interface;
	uint32_t listlen = 0;
	uint8_t control_flag = 0;

	if (argc != 2) {
		print_err(print, "Usage: call_qcsapi set_chan_disabled <WiFi interface> "
				"<channel list> <enable/disable>\n");
		return 1;
	}

	if (!isdigit(*argv[1])) {
		print_err(print, "Unrecognized %s; Supported channel control: 0: disable 1:enable\n", argv[1]);
		return 1;
	} else {
		control_flag = atoi(argv[1]);
	}

	memset(&chan_list, 0, sizeof(chan_list));
	statval = string_to_list(print, argv[0], chan_list.data, &listlen);
	if (statval < 0)
		return statval;

	qcsapi_retval = qcsapi_wifi_chan_control(the_interface, &chan_list, listlen, control_flag);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void dump_disabled_chanlist(qcsapi_output *print, uint8_t *data, uint8_t cnt)
{
	int loop;

	if (cnt > 0) {
		print_out(print, "%d", data[0]);
		for (loop = 1; loop < cnt; loop++) {
			print_out(print, ",%d", data[loop]);
		}
		print_out(print, "\n");
	}
}

static int
call_qcsapi_wifi_get_chan_disabled(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes chan_list;
	uint8_t cnt = 0;

	qcsapi_retval = qcsapi_wifi_get_chan_disabled(the_interface, &chan_list, &cnt);

	if (qcsapi_retval >= 0) {
		dump_disabled_chanlist(print, chan_list.data, cnt);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_standard(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char ieee_standard[16], *p_standard = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_standard = &ieee_standard[0];
	qcsapi_retval = qcsapi_wifi_get_IEEE_802_11_standard(the_interface, p_standard);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &ieee_standard[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_get_dtim(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int dtim;
	qcsapi_unsigned_int *p_dtim = NULL;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_dtim = &dtim;

	qcsapi_retval = qcsapi_wifi_get_dtim(interface, p_dtim);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", dtim);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_set_dtim(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set dtim, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int dtim = atoi(argv[0]);
		int qcsapi_retval;
		const char *interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_dtim(interface, dtim);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_assoc_limit(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int assoc_limit;
	qcsapi_unsigned_int *p_assoc_limit = NULL;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_assoc_limit = &assoc_limit;

	qcsapi_retval = qcsapi_wifi_get_assoc_limit(interface, p_assoc_limit);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", assoc_limit);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_assoc_limit(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set assoc_limit, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int assoc_limit = atoi(argv[0]);
		int qcsapi_retval;
		const char *interface = p_calling_bundle->caller_interface;
		int i;

		for (i = 0; argv[0][i] != 0; i++) {
			if (isdigit(argv[0][i]) == 0) {
				print_err(print, "Invalid parameter:%s, should be integer\n",
						argv[0]);
				return 1;
			}
		}

		qcsapi_retval = qcsapi_wifi_set_assoc_limit(interface, assoc_limit);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bss_assoc_limit(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_unsigned_int assoc_limit;
	qcsapi_unsigned_int group;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get bss_assoc_limit,"
				" count is %d\n", argc);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &group)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_bss_assoc_limit(group, &assoc_limit);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "group assoc_limit %d\n", assoc_limit);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_bss_assoc_limit(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_unsigned_int limit;
	qcsapi_unsigned_int group;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi set bss_assoc_limit,"
				" count is %d\n", argc);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &group)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[1], &limit)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_bss_assoc_limit(group, limit);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_SSID_group_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_unsigned_int group;
	int qcsapi_retval;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set SSID_group_id,"
				" count is %d\n", argc);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &group)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_SSID_group_id(interface, group);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_SSID_group_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int group;
	qcsapi_unsigned_int *p_group = &group;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_SSID_group_id(interface, p_group);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "group_id %d\n", group);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_SSID_assoc_reserve(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	qcsapi_unsigned_int group;
	qcsapi_unsigned_int value;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi set_SSID_assoc_reserve,"
				" count is %d\n", argc);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &group)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[1], &value)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_SSID_assoc_reserve(group, value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_SSID_assoc_reserve(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	qcsapi_unsigned_int group;
	qcsapi_unsigned_int value;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get_SSID_assoc_reserve,"
				" count is %d\n", argc);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &group)) {
		print_err(print, "Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_SSID_assoc_reserve(group, &value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "group assoc reserved value : %u\n", value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_interface_get_status(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char interface_status[16], *p_status = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_status = &interface_status[0];
	qcsapi_retval = qcsapi_interface_get_status(the_interface, p_status);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", &interface_status[0]);
	} else {
		if (verbose_flag >= 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		}

		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_interface_set_ip4(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	uint32_t if_param_val;
	uint32_t if_param_val_ne;
	int qcsapi_retval;
	char *if_param = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"<interface> {ipaddr <IP address> | netmask <netmask>}\n");
		statval = 1;
	} else {
		if (strcmp(argv[0], "NULL") != 0)
			if_param = argv[0];

		if (inet_pton(AF_INET, argv[1], &if_param_val) != 1) {
			print_err(print, "invalid IPv4 argument %s\n", argv[1]);
			return -EINVAL;
		}
		if_param_val_ne = htonl(if_param_val);

		qcsapi_retval = qcsapi_interface_set_ip4(the_interface, if_param, if_param_val_ne);

		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			if (verbose_flag >= 0) {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			}
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_interface_get_ip4(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_64 if_param_val;
	char *p_if_param_val = &if_param_val[0];
	int qcsapi_retval;
	char *if_param = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		if_param = argv[0];
	}

	qcsapi_retval = qcsapi_interface_get_ip4(the_interface, if_param, p_if_param_val);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", p_if_param_val);
	} else {
		if (verbose_flag >= 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		}
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_interface_set_mtu(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	int retval;
	uint32_t mtu;

	if (qcsapi_str_to_uint32(argv[0], &mtu) < 0) {
		qcsapi_report_usage(p_calling_bundle, "<interface> <mtu>");
		return 1;
	}

	retval = qcsapi_interface_set_mtu(the_interface, mtu);

	return qcsapi_report_complete(p_calling_bundle, retval);
}

static int
call_qcsapi_interface_get_mtu(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int retval;
	uint32_t mtu;

	if (argc != 0) {
		qcsapi_report_usage(p_calling_bundle, "<interface>");
		return 1;
	}

	retval = qcsapi_interface_get_mtu(the_interface, &mtu);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	print_out(print, "%u\n", mtu);

	return 0;
}

static int
call_qcsapi_wifi_get_list_channels(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char *p_list_channels = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_list_channels = regulatory_list_buf;
	qcsapi_retval = qcsapi_wifi_get_list_channels(the_interface, p_list_channels);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", regulatory_list_buf);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define LOCAL_PRINT_CHARS_PER_CHANNEL	5

static int
local_print_channel_list(const uint8_t *p_ch_list, const size_t size, qcsapi_output *print)
{
	char *out_buf;
	int out_buf_size = size * NBBY * LOCAL_PRINT_CHARS_PER_CHANNEL;
	char *p_buf;
	int indx;
	int len;
	int used_len = 0;
	int free_len;
	int channels_2g = 0;
	int channels_5g = 0;
	int channels_6g = 0;
	int channels_total = 0;

	out_buf = malloc(out_buf_size);
	if (out_buf == NULL) {
		print_err(print, "Failed to allocate %u chars\n", out_buf_size);
		return 1;
	}

	p_buf = out_buf;

	for (indx = 0; indx < (size * NBBY); indx++) {
		if (isset(p_ch_list, indx)) {
			free_len = out_buf_size - used_len;

			if (IC_IEEE_IS_CHAN_IN_2G(indx)) {
				if (channels_2g == 0) {
					len = snprintf(p_buf, free_len, "%s", "2g: ");
					p_buf += len;
					used_len += len;
				}
				channels_2g++;
				channels_total++;
			} else if (IC_IEEE_IS_CHAN_IN_5G(indx)) {
				if (channels_5g == 0) {
					if (channels_total)
						snprintf(p_buf - 1, free_len, "\n");
					len = snprintf(p_buf, free_len, "%s", "5g: ");
					p_buf += len;
					used_len += len;
				}
				channels_5g++;
				channels_total++;
			} else if (IC_IEEE_IS_CHAN_IN_6G(indx)) {
				if (channels_6g == 0) {
					if (channels_total)
						snprintf(p_buf - 1, free_len, "\n");
					len = snprintf(p_buf, free_len, "%s", "6g: ");
					p_buf += len;
					used_len += len;
				}
				channels_6g++;
				channels_total++;
			} else {
				len = snprintf(p_buf, free_len, "%s",
						"\nERROR: Unknown channel in the list ");
				p_buf += len;
				used_len += len;
				break;
			}

			free_len = out_buf_size - used_len;
			len = snprintf(p_buf, free_len, "%d,", (indx & IC_IEEE_CHANNEL_MASK));
			p_buf += len;
			used_len += len;
		}
	}

	out_buf[used_len ? used_len - 1 : 0] = '\0';

	print_out(print, "%s\n", out_buf);

	free(out_buf);
	return 0;
}

static int
call_qcsapi_wifi_get_dfs_s_radio_chan_off(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_data_256bytes chans;

	memset(&chans, 0, sizeof(chans));

	if (argc != 0) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface>");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_chan_list(the_interface, &chans,
						qcsapi_chlist_flag_ocac_off);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return local_print_channel_list(chans.data, sizeof(chans), print);
}

static int
call_qcsapi_wifi_get_chan_list(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	struct qcsapi_data_256bytes chan_list;
	uint32_t flags = 0;
	const char *usage = "<WiFi interface> [ {available | disabled | scan | active |	ocac_off}]";

	if (argc == 0) {
		flags |= qcsapi_chlist_flag_available;
	} else if (argc == 1) {
		if (!strcasecmp(argv[0], "available"))
			flags |= qcsapi_chlist_flag_available;
		else if (!strcasecmp(argv[0], "disabled"))
			flags |= qcsapi_chlist_flag_disabled;
		else if (!strcasecmp(argv[0], "scan"))
			flags |= qcsapi_chlist_flag_scan;
		else if (!strcasecmp(argv[0], "active"))
			flags |= qcsapi_chlist_flag_active;
		else if (!strcasecmp(argv[0], "ocac_off"))
			flags |= qcsapi_chlist_flag_ocac_off;
		else
			goto usage;
	} else {
		goto usage;
	}

	memset(&chan_list, 0, sizeof(chan_list));
	qcsapi_retval = qcsapi_wifi_get_chan_list(the_interface, &chan_list, flags);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0)
		return local_print_channel_list(chan_list.data, sizeof(chan_list), print);

	return 0;
usage:
	qcsapi_report_usage(p_calling_bundle, usage);
	return 1;
}

static int
call_qcsapi_wifi_get_chan_usable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes chan_list = { {0} };

	COMPILE_TIME_ASSERT(sizeof(chan_list) >= IEEE80211_CHAN_MAX / NBBY);

	qcsapi_retval = qcsapi_wifi_get_chan_list(the_interface, &chan_list,
						qcsapi_chlist_flag_active);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return local_print_channel_list(chan_list.data, sizeof(chan_list), print);

}

static int
call_qcsapi_wifi_get_supp_chans(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int ival;
	int statval = 0;
	char *p_list_channels = NULL;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	static string_1024 the_list_channels;
	qcsapi_mac_addr mac_address;

	if (argc < 1) {
		qcsapi_retval = -EFAULT;
	} else {
		p_list_channels = &the_list_channels[0];
		memset(p_list_channels, 0, sizeof(the_list_channels));
	}
	if (qcsapi_retval >= 0) {
		ival = parse_mac_addr(argv[0], mac_address);
		if (ival < 0) {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			qcsapi_retval = -EFAULT;
		} else {
			qcsapi_retval = qcsapi_wifi_get_supp_chans(the_interface,
					mac_address, p_list_channels);
		}
	}

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", &the_list_channels[0]);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mode_switch(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	uint8_t wifi_mode, *p_wifi_mode = NULL;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_wifi_mode = &wifi_mode;
	qcsapi_retval = qcsapi_wifi_get_mode_switch(p_wifi_mode);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%x\n", wifi_mode);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_option(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int wifi_option, *p_wifi_option = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_option_type the_option =
			p_calling_bundle->caller_generic_parameter.parameter_type.option;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_wifi_option = &wifi_option;
	qcsapi_retval = qcsapi_wifi_get_option(the_interface, the_option, p_wifi_option);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (wifi_option == 0)
				print_out(print, "FALSE\n");
			else
				print_out(print, "TRUE\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_dpp_parameter(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int i;
	int j;
	int qcsapi_retval;
	char resp[QCSAPI_DPP_MAX_BUF_SIZE];
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_dpp_set_parameters dpp_params;
	enum qcsapi_dpp_cmd_param_type cmd =
		p_calling_bundle->caller_generic_parameter.parameter_type.dpp_param_type;
	const char usage1[] = "<WiFi interface> cfg_get <param1>";
	const char usage2[] = "<WiFi interface> cfg_set <param1> <value1>";
	const char usage3[] =
		"<WiFi interface> <dpp_command> <param1> <value1> ... [<param8> <value8>]";

	memset(&dpp_params, 0, sizeof(dpp_params));

	if (cmd == qcsapi_dpp_cmd_get_config) {
		if (argc != 1) {
			qcsapi_report_usage(p_calling_bundle, usage1);
			qcsapi_report_usage(p_calling_bundle, usage2);
			qcsapi_report_usage(p_calling_bundle, usage3);
			return 1;
		}
		strncpy(dpp_params.param[0].key, argv[0], sizeof(dpp_params.param[0].key) - 1);
	} else {
		if (argc < 2 || argc > (2 * ARRAY_SIZE(dpp_params.param)) || !!(argc % 2)) {
			qcsapi_report_usage(p_calling_bundle, usage1);
			qcsapi_report_usage(p_calling_bundle, usage2);
			qcsapi_report_usage(p_calling_bundle, usage3);
			return 1;
		}

		for (i = 0, j = 0; i < ARRAY_SIZE(dpp_params.param) && j < argc; i++, j += 2) {
			strncpy(dpp_params.param[i].key, argv[j],
				sizeof(dpp_params.param[i].key) - 1);
			strncpy(dpp_params.param[i].value, argv[j + 1],
				sizeof(dpp_params.param[i].value) - 1);
		}
	}

	qcsapi_retval = qcsapi_wifi_dpp_parameter(the_interface, cmd, &dpp_params,
							resp, sizeof(resp));

	/* Successful set/dpp_command response */
	if (qcsapi_retval >= 0 && (strcasecmp(resp, "OK") == 0))
		snprintf(resp, sizeof(resp), "%s", "complete");

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, resp);
}

static int
call_qcsapi_wifi_get_parameter(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int value;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_wifi_param_type type =
			p_calling_bundle->caller_generic_parameter.parameter_type.wifi_param_type;

	if (argc > 0) {
		qcsapi_report_usage(p_calling_bundle, "<ifname> <parameter name>");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_parameter(the_interface, type, &value);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_parameter(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int value;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_wifi_param_type type =
			p_calling_bundle->caller_generic_parameter.parameter_type.wifi_param_type;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle,
				"<ifname> <parameter name> <parameter value>");
		return 1;
	}

	qcsapi_retval = sscanf(argv[0], "%i", &value);

	if (qcsapi_retval <= 0) {
		print_err(print, "Invalid parameter - must be a signed integer\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_parameter(the_interface, type, value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define BUF_MAX_LEN	40

static int
call_qcsapi_get_board_parameter(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_board_parameter_type the_boardparam =
			p_calling_bundle->caller_generic_parameter.parameter_type.board_param;
	string_64 p_buffer;
	qcsapi_unsigned_int radio_id = 0;

	if (argc > 0) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	if ((argc > 1) && (strcmp(argv[1], "NULL") == 0))
		qcsapi_retval = -EFAULT;

	if (qcsapi_retval >= 0) {
		memset(p_buffer, 0, sizeof(p_buffer));
		qcsapi_retval = qcsapi_radio_get_board_parameter(radio_id, the_boardparam,
				p_buffer);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_buffer);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_noise(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int current_noise, *p_noise = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_noise = &current_noise;
	}

	qcsapi_retval = qcsapi_wifi_get_noise(the_interface, p_noise);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", current_noise);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rssi_by_chain(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get RSSI by chain\n");
		print_err(print, "Usage: call_qcsapi get_rssi_by_chain <WiFi interface> <RF chain>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		int current_rssi = 0, *p_rssi = NULL;
		const char *the_interface = p_calling_bundle->caller_interface;
		int rf_chain = atoi(argv[0]);

		if (argc < 2 || strcmp(argv[1], "NULL") != 0) {
			p_rssi = &current_rssi;
		}

		if (rf_chain == 0 && (qcsapi_verify_numeric(argv[0]) < 0)) {
			print_err(print, "Invalid argument %s - must be an integer\n", argv[0]);
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_get_rssi_by_chain(the_interface, rf_chain, p_rssi);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%d\n", current_rssi);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_avg_snr(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int current_snr, *p_snr = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_snr = &current_snr;
	}

	qcsapi_retval = qcsapi_wifi_get_avg_snr(the_interface, p_snr);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", current_snr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_option(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int wifi_option;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_option_type the_option =
			p_calling_bundle->caller_generic_parameter.parameter_type.option;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set option, count is %d\n", argc);
		statval = 1;
	} else {
		if ((strcasecmp(argv[0], "TRUE") == 0) || (strcasecmp(argv[0], "YES") == 0) ||
				(strcmp(argv[0], "1") == 0))
			wifi_option = 1;
		else if ((strcasecmp(argv[0], "FALSE") == 0) || (strcasecmp(argv[0], "NO") == 0) ||
				(strcmp(argv[0], "0") == 0))
			wifi_option = 0;
		else {
			print_err(print, "Invalid input arguments\n");
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_option(the_interface, the_option, wifi_option);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rates(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char *p_rates = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_rate_type the_rates_type =
			p_calling_bundle->caller_generic_parameter.parameter_type.typeof_rates;
	static string_2048 the_rates;
/*
 * Prefer a non-reentrant program to allocating 2049 bytes on the stack.
 */
	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_rates = &the_rates[0];
	}

	qcsapi_retval = qcsapi_wifi_get_rates(the_interface, the_rates_type, p_rates);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", the_rates);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

/*
 * validate_rates return 1 on success and 0 on failure
 */
static int validate_rates(char *input_rate[], int num_rates)
{
	int rates[] = { 2, 4, 11, 12, 18, 22, 24, 36, 48, 72, 96, 108 };
	int found = 0, i, j, rate;

	for (i = 0; i < num_rates; i++) {
		rate = atoi(input_rate[i]);
		found = 0;
		for (j = 0; j < ARRAY_SIZE(rates); j++) {
			if (rate == rates[j]) {
				found = 1;
				break;
			}
		}
		if (!found) {
			break;
		}
	}
	return found;
}

static int
call_qcsapi_wifi_set_rates(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_data_256bytes current_rates;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_rate_type the_rates_type =
			p_calling_bundle->caller_generic_parameter.parameter_type.typeof_rates;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set rates, count is %d\n", argc);
		statval = 1;
	} else {
		char *p_rates = argv[0];

		if (!validate_rates(argv, argc)) {
			print_err(print, "Invalid input rates, valid rates are 2,4,11,12,18,22,24,36,48,72,96,108 in 500Kbps units\n");
			return 1;
		}

		memset(&current_rates, 0, sizeof(current_rates));
		memcpy(current_rates.data, p_rates, sizeof(current_rates.data));
		qcsapi_retval = qcsapi_wifi_set_wifi_rates(the_interface, the_rates_type,
				&current_rates, argc);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_max_bitrate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	char max_bitrate_str[QCSAPI_MAX_BITRATE_STR_MIN_LEN + 1] = { 0 };

	qcsapi_retval = qcsapi_get_max_bitrate(the_interface, max_bitrate_str,
			sizeof(max_bitrate_str));

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &max_bitrate_str[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_max_bitrate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi wifi set max bitrate, count is %d\n", argc);
		statval = 1;
	}

	qcsapi_retval = qcsapi_set_max_bitrate(the_interface, argv[0]);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_beacon_type(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char beacon_type[16], *p_beacon = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_beacon = &beacon_type[0];
	qcsapi_retval = qcsapi_wifi_get_beacon_type(the_interface, p_beacon);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &beacon_type[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_beacon_type(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set beacon, count is %d\n", argc);
		statval = 1;
	} else {
		char *p_beacon = argv[0];

		/* Beacon type will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_beacon = NULL;
		qcsapi_retval = qcsapi_wifi_set_beacon_type(the_interface, p_beacon);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_beacon_interval(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int bintval_value, *p_bintval_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_bintval_value = &bintval_value;

	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_beacon_interval(the_interface, p_bintval_value);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", bintval_value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_beacon_interval(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int new_bintval = atoi(argv[0]);

	if ((new_bintval > BEACON_INTERVAL_WARNING_LOWER_LIMIT)
			&& (new_bintval < BEACON_INTERVAL_WARNING_UPPER_LIMIT)) {
		print_out(print, "Warning, beacon interval less than 100ms may cause network performance degradation\n");
	}

	qcsapi_retval = qcsapi_wifi_set_beacon_interval(the_interface, new_bintval);

	if (qcsapi_retval >= 0) {
		print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_list_regulatory_regions(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	string_512 supported_regions;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_regions(supported_regions);

		if (qcsapi_retval == -qcsapi_region_database_not_found) {
			qcsapi_retval = qcsapi_wifi_get_list_regulatory_regions(supported_regions);
		}

	} else {

		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_regions(NULL);

		if (qcsapi_retval == -qcsapi_region_database_not_found)
			qcsapi_retval = qcsapi_wifi_get_list_regulatory_regions(NULL);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", supported_regions);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_regulatory_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi get regulatory tx_power\n");
		print_err(print, "Usage: call_qcsapi get_regulatory_tx_power <WiFi interface> <channel> <regulatory region>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = (qcsapi_unsigned_int) atoi(argv[0]);
		const char *regulatory_region = NULL;
		int *p_tx_power = NULL, tx_power = 0;
		qcsapi_unsigned_int the_bw = 0;

		if (strcmp(argv[1], "NULL") != 0)
			regulatory_region = argv[1];

		if (argc < 3 || strcmp(argv[2], "NULL") != 0)
			p_tx_power = &tx_power;

		qcsapi_retval = qcsapi_wifi_get_bw(the_interface, &the_bw);
		/* Call to get the BW might fail if the interface is wrong */
		if (qcsapi_retval < 0) {
			if ((qcsapi_retval == -ENODEV) || (qcsapi_retval == -EOPNOTSUPP)) {
				print_out(print, "Interface %s does not exist"
						"or not a Wireless Extension interface\n",
						the_interface);
			}
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);

			statval = 1;
			goto finish;
		}

		qcsapi_retval = qcsapi_regulatory_get_regulatory_tx_power(the_interface,
				the_channel, regulatory_region, p_tx_power);

		if (qcsapi_retval == -qcsapi_region_database_not_found) {

			qcsapi_retval = qcsapi_wifi_get_regulatory_tx_power(the_interface,
					the_channel, regulatory_region, p_tx_power);
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				/* bit15:8 of tx_power > 0, bw160M tx power using imbalance way */
				if (the_bw == qcsapi_bw_160MHz && (tx_power & 0xFF00)) {
					print_out(print, "bw160M use imbalance regulatory tx power.\n");
					print_out(print, "lower 80M band : %d\n", tx_power & 0xFF);
					print_out(print, "higher 80M band : %d\n",
							(tx_power & 0xFF00) >> 8);
				} else {
					print_out(print, "%d\n", tx_power);
				}
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

finish:
	return statval;
}

static int
call_qcsapi_wifi_get_configured_tx_power(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *iface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int channel;
	const char *region;
	qcsapi_unsigned_int the_bw = 0;
	qcsapi_unsigned_int bf_on;
	qcsapi_unsigned_int number_ss;
	int retval;
	int tx_power = 0;

	const char *msg_usage_mandatory_params =
			"Not enough parameters in call qcsapi get_configured_tx_power\n"
			"Usage: call_qcsapi get_configured_tx_power"
			" <WiFi interface> <channel> <regulatory region>";

	if (argc < 2) {
		print_err(print, "%s [bandwidth] [bf_on] [number_ss]\n",
				msg_usage_mandatory_params);

		statval = 1;
		goto finish;
	}

	channel = (qcsapi_unsigned_int) atoi(argv[0]);
	region = argv[1];

	if (argc < 3) {
		retval = qcsapi_wifi_get_bw(iface, &the_bw);

		/* Call to get the BW might fail if the interface is wrong */
		if (retval < 0) {
			if ((retval == -ENODEV) || (retval == -EOPNOTSUPP)) {
				print_out(print, "Interface %s does not exist"
						"or not a Wireless Extension interface\n", iface);
			} else
				report_qcsapi_error(p_calling_bundle, retval);

			statval = 1;
			goto finish;
		}
	} else
		the_bw = (qcsapi_unsigned_int) atoi(argv[2]);

	if (argc < 4) {
		/* additional parameters are not specified: beamforming off, one spatial stream */
		bf_on = 0;
		number_ss = 1;
	} else if (argc >= 5) {
		bf_on = atoi(argv[3]);
		number_ss = atoi(argv[4]);
	} else {
		/* beamforming and spatial stream must be specified */
		print_err(print, "%s <bandwidth> <bf_on> <number_ss>\n",
				msg_usage_mandatory_params);

		statval = 1;
		goto finish;
	}

	retval = qcsapi_regulatory_get_configured_tx_power_ext(iface,
			channel, region, the_bw, bf_on, number_ss, &tx_power);

	if (retval == -qcsapi_region_database_not_found) {
		retval = qcsapi_wifi_get_configured_tx_power(iface,
				channel, region, the_bw, &tx_power);
	}

	if (retval >= 0) {
		if (verbose_flag >= 0) {
			if ((tx_power >= 0) && ((tx_power >> 8) & 0xFF))
				print_out(print, "%d.%d\n", (tx_power & 0xFF),
						((tx_power >> 8) & 0xFF));
			else
				print_out(print, "%d\n", tx_power);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, retval);
		statval = 1;
	}

finish:

	return statval;
}

static int
call_qcsapi_wifi_set_regulatory_channel(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi set regulatory channel\n");
		print_err(print, "Usage: call_qcsapi set_regulatory_channel <WiFi interface> <channel> <regulatory region> <TX power offset>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = (qcsapi_unsigned_int) atoi(argv[0]);
		const char *regulatory_region = NULL;
		qcsapi_unsigned_int tx_power_offset = 0;

		if (argc >= 3)
			tx_power_offset = (qcsapi_unsigned_int) atoi(argv[2]);

		if (strcmp(argv[1], "NULL") != 0)
			regulatory_region = argv[1];

		qcsapi_retval = qcsapi_regulatory_set_regulatory_channel(the_interface,
				the_channel, regulatory_region, tx_power_offset);

		if (qcsapi_retval == -qcsapi_region_database_not_found) {

			qcsapi_retval = qcsapi_wifi_set_regulatory_channel(the_interface,
					the_channel, regulatory_region, tx_power_offset);
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_regulatory_region(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi set regulatory region\n");
		print_err(print, "Usage: call_qcsapi set_regulatory_region <WiFi interface> <regulatory region>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		const char *regulatory_region = NULL;

		if (strcmp(argv[0], "NULL") != 0)
			regulatory_region = argv[0];

		qcsapi_retval = qcsapi_regulatory_set_regulatory_region(the_interface,
				regulatory_region);

		if (qcsapi_retval == -qcsapi_region_database_not_found) {
			qcsapi_retval = qcsapi_wifi_set_regulatory_region(the_interface,
					regulatory_region);
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_restore_regulatory_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_regulatory_restore_regulatory_tx_power(the_interface);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_regulatory_region(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;

	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char regulatory_region[6];
	char *p_regulatory_region = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_regulatory_region = &regulatory_region[0];
	}

	qcsapi_retval = qcsapi_wifi_get_regulatory_region(the_interface, p_regulatory_region);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", p_regulatory_region);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_overwrite_country_code(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi overwrite coutnry code\n");
		print_err(print, "Usage: call_qcsapi overwrite_country_code <WiFi interface> <curr_country_name> <new_country_name>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		const char *curr_country_name = NULL;
		const char *new_country_name = NULL;

		if (strcmp(argv[0], "NULL") != 0)
			curr_country_name = argv[0];
		if (strcmp(argv[1], "NULL") != 0)
			new_country_name = argv[1];

		qcsapi_retval = qcsapi_regulatory_overwrite_country_code(the_interface,
				curr_country_name, new_country_name);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0)
				print_out(print, "complete\n");
		} else if (qcsapi_retval == -qcsapi_configuration_error) {
			print_err(print, "Error: can't overwrite country code for provision board\n");
			statval = 1;
		} else if (qcsapi_retval == -qcsapi_region_not_supported) {
			print_err(print, "Error: current region is not %s\n", curr_country_name);
			statval = 1;
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wifi_get_list_regulatory_channels(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	char *p_list_channels = NULL;
	const char *regulatory_region = NULL;
	qcsapi_unsigned_int the_bw = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *ifname = NULL;

	if (argc < 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<region> [BW] [iface]\n");
		return 1;
	}

	if (argc >= 3)
		ifname = argv[2];

	if (argc == 1 || ((argc >= 2) && !strcmp(argv[1], "current"))) {
		qcsapi_retval = qcsapi_wifi_get_bw(ifname, &the_bw);
		if (qcsapi_retval < 0)
			goto out;
	} else if (argc >= 2) {
		the_bw = atoi(argv[1]);
	}

	if (strcmp(argv[0], "NULL") != 0)
		regulatory_region = argv[0];

	if (argc <= 3 || strcmp(argv[3], "NULL") != 0)
		p_list_channels = regulatory_list_buf;

	if (ifname)
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_channels_if(ifname,
				regulatory_region, the_bw, -1, p_list_channels);
	else
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_channels(regulatory_region,
				the_bw, p_list_channels);

	if (qcsapi_retval == -qcsapi_region_database_not_found)
		qcsapi_retval = qcsapi_wifi_get_list_regulatory_channels(regulatory_region,
				the_bw, p_list_channels);

out:
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", regulatory_list_buf);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_list_regulatory_bands(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *ifname = NULL;
	int qcsapi_retval;
	char *p_list_bands = NULL;
	const char *regulatory_region = NULL;

	if (argc < 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<region> [iface]\n");
		return 1;
	}

	if (strcmp(argv[0], "NULL") != 0)
		regulatory_region = argv[0];

	if (argc > 1)
		ifname = argv[1];

	if (argc < 3 || strcmp(argv[2], "NULL") != 0)
		p_list_bands = regulatory_list_buf;

	if (ifname)
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_bands_if(ifname,
				regulatory_region, p_list_bands);
	else
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_bands(regulatory_region,
				p_list_bands);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", regulatory_list_buf);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_regulatory_db_version(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	int version = 0;
	int index = 0;
	int retval = 0;
	char ch = 'v';
	int *p_qcsapi_retval = &qcsapi_retval;
	const char *format[2] = { "%c%d", "0%c%x" };

	if (argc > 0) {
		index = atoi(argv[0]);
		ch = 'x';
	}

	if (verbose_flag >= 0)
		print_out(print, "Regulatory db version: ");

	do {
		*p_qcsapi_retval = qcsapi_regulatory_get_db_version(&version, index++);
		if (qcsapi_retval == -1 || retval < 0)
			break;

		print_out(print, format[argc > 0], ch, version);

		ch = '.';
		p_qcsapi_retval = &retval;
	} while (argc == 0 && qcsapi_retval >= 0);

	if (qcsapi_retval == -1) {
		print_out(print, "database not available");
	}

	print_out(print, "\n");

	if (qcsapi_retval < 0)
		statval = 1;

	return statval;
}

static int
call_qcsapi_wifi_set_regulatory_tx_power_cap(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int capped = atoi(argv[0]);
		int qcsapi_retval;

		qcsapi_retval = qcsapi_regulatory_apply_tx_power_cap(capped);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get TX power\n");
		print_err(print, "Usage: call_qcsapi get_tx_power <interface> <channel>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = atoi(argv[0]);
		int the_tx_power = 0;
		int *p_tx_power = NULL;

		if (argc < 2 || strcmp(argv[1], "NULL") != 0) {
			p_tx_power = &the_tx_power;
		}

		qcsapi_retval = qcsapi_wifi_get_tx_power(the_interface, the_channel, p_tx_power);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%d\n", the_tx_power);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int channel;
	int tx_power = 0;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi set_tx_power\n");
		return 1;
	}

	channel = atoi(argv[0]);
	tx_power = atoi(argv[1]);

	qcsapi_retval = qcsapi_wifi_set_tx_power(the_interface, channel, tx_power);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bw_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get_bw_power\n");
		print_err(print, "Usage: call_qcsapi get_bw_power <interface> <channel>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = atoi(argv[0]);
		int power_20M = 0;
		int power_40M = 0;
		int power_80M = 0;

		qcsapi_retval = qcsapi_wifi_get_bw_power(the_interface, the_channel,
				&power_20M, &power_40M, &power_80M);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, " pwr_20M  pwr_40M  pwr_80M\n %7d  %7d  %7d\n",
						power_20M, power_40M, power_80M);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bw_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int channel;
	int power_20M = 0;
	int power_40M = 0;
	int power_80M = 0;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi set_bw_power\n");
		print_err(print, "Usage: call_qcsapi set_bw_power <interface> <channel>"
				" <power_20M> <power_40M> <power_80M>\n");
		return 1;
	}

	channel = atoi(argv[0]);
	power_20M = atoi(argv[1]);
	if (argc >= 3) {
		power_40M = atoi(argv[2]);
		if (argc >= 4) {
			power_80M = atoi(argv[3]);
		}
	}

	qcsapi_retval = qcsapi_wifi_set_bw_power(the_interface, channel,
			power_20M, power_40M, power_80M);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bf_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi get_bf_power\n");
		print_err(print, "Usage: call_qcsapi get_bf_power <interface> <channel> <number_ss>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = atoi(argv[0]);
		int number_ss = atoi(argv[1]);
		int power_20M = 0;
		int power_40M = 0;
		int power_80M = 0;

		qcsapi_retval = qcsapi_wifi_get_bf_power(the_interface, the_channel,
				number_ss, &power_20M, &power_40M, &power_80M);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, " pwr_20M  pwr_40M  pwr_80M\n %7d  %7d  %7d\n",
						power_20M, power_40M, power_80M);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bf_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int channel;
	int power_20M = 0;
	int power_40M = 0;
	int power_80M = 0;
	int number_ss = 0;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi set_bf_power\n");
		print_err(print, "Usage: call_qcsapi set_bf_power <interface> <channel>"
				" <number_ss> <power_20M> <power_40M> <power_80M>\n");
		return 1;
	}

	channel = atoi(argv[0]);
	number_ss = atoi(argv[1]);
	power_20M = atoi(argv[2]);
	if (argc >= 4) {
		power_40M = atoi(argv[3]);
		if (argc >= 5) {
			power_80M = atoi(argv[4]);
		}
	}

	qcsapi_retval = qcsapi_wifi_set_bf_power(the_interface, channel,
			number_ss, power_20M, power_40M, power_80M);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_power_ext(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call_qcsapi get_tx_power_ext\n");
		print_err(print, "Usage: call_qcsapi get_tx_power_ext <interface> <channel> <bf_on> <number_ss>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = atoi(argv[0]);
		int bf_on = ! !atoi(argv[1]);
		int number_ss = atoi(argv[2]);
		int power_20M = 0;
		int power_40M = 0;
		int power_80M = 0;

		qcsapi_retval = qcsapi_wifi_get_tx_power_ext(the_interface, the_channel,
				bf_on, number_ss, &power_20M, &power_40M, &power_80M);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, " pwr_20M  pwr_40M  pwr_80M\n %7d  %7d  %7d\n",
						power_20M, power_40M, power_80M);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_tx_power_ext(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int channel;
	int power_20M = 0;
	int power_40M = 0;
	int power_80M = 0;
	int bf_on = 0;
	int number_ss = 0;

	if (argc < 4) {
		print_err(print, "Not enough parameters in call_qcsapi set_tx_power_ext\n");
		print_err(print, "Usage: call_qcsapi set_tx_power_ext <interface> <channel>"
				" <bf_on> <number_ss> <power_20M> <power_40M> <power_80M>\n");
		return 1;
	}

	channel = atoi(argv[0]);
	bf_on = ! !atoi(argv[1]);
	number_ss = atoi(argv[2]);
	power_20M = atoi(argv[3]);
	if (argc >= 5) {
		power_40M = atoi(argv[4]);
		if (argc >= 6) {
			power_80M = atoi(argv[5]);
		}
	}

	qcsapi_retval = qcsapi_wifi_set_tx_power_ext(the_interface, channel,
			bf_on, number_ss, power_20M, power_40M, power_80M);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

unsigned long regulatory_chan_txpower_input_nss_to_ssidx(unsigned long input)
{
	return input - 1;
}

unsigned long regulatory_chan_txpower_input_bw_to_bwidx(unsigned long input)
{
	unsigned long ret;

	switch (input) {
	case 20:
		ret = QCSAPI_PWR_BW_20M;
		break;
	case 40:
		ret = QCSAPI_PWR_BW_40M;
		break;
	case 80:
		ret = QCSAPI_PWR_BW_80M;
		break;
	case 160:
		ret = QCSAPI_PWR_BW_160M;
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static unsigned regulatory_chan_txpower_input_parse(char *input_str,
		const unsigned num_bits, unsigned long (*transform_func) (unsigned long input))
{
	char *cur;
	unsigned bitmap = 0;
	unsigned long tmp_val;
	char *saved_p;

	if (!input_str)
		goto out;

	cur = strtok_r(input_str, ",", &saved_p);
	if (!cur)
		goto out;

	if (*cur == '*' && *(cur + 1) == '\0') {
		bitmap |= (1 << num_bits) - 1;
	} else {
		char *endptr = NULL;

		do {
			tmp_val = strtoul(cur, &endptr, 10);
			if (transform_func)
				tmp_val = transform_func(tmp_val);

			if (errno || *endptr != '\0' || (tmp_val >= num_bits)) {
				bitmap = 0;
				goto out;
			}

			bitmap |= (1 << tmp_val);
			cur = strtok_r(NULL, ",", &saved_p);
		} while (cur);
	}
out:
	return bitmap;
}

static int regulatory_chan_txpower_input_parse_bitmap(char *input, qcsapi_output *print,
		uint8_t *chan, unsigned *ss_map, unsigned *bf_map, unsigned *fem_pri_map,
		unsigned *bw_map)
{
	char *endptr;
	char *cur;

	cur = strtok(input, ":");
	if (!cur)
		goto error;

	errno = 0;
	*chan = strtoul(cur, &endptr, 10);
	if (errno || *endptr != '\0')
		goto error;

	cur = strtok(NULL, ":");
	if (!cur) {
		/* Specifying channel number only equals to specifying <chan:*:*:*:*> */
		*ss_map = (1 << QCSAPI_PWR_IDX_SS_NUM) - 1;
		*bf_map = (1 << QCSAPI_PWR_IDX_BF_NUM) - 1;
		*fem_pri_map = (1 << QCSAPI_PWR_IDX_FEM_PRIPOS_NUM) - 1;
		*bw_map = (1 << QCSAPI_PWR_BW_NUM) - 1;
		return 0;
	}

	*ss_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_IDX_SS_NUM, &regulatory_chan_txpower_input_nss_to_ssidx);
	if (!*ss_map) {
		print_err(print, "Bad NSS\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*bf_map = regulatory_chan_txpower_input_parse(cur, QCSAPI_PWR_IDX_BF_NUM, NULL);
	if (!*bf_map) {
		print_err(print, "Bad BF\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*fem_pri_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_IDX_FEM_PRIPOS_NUM, NULL);
	if (!*fem_pri_map) {
		print_err(print, "Bad FEM/PRI\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (!cur)
		goto error;

	*bw_map = regulatory_chan_txpower_input_parse(cur,
			QCSAPI_PWR_BW_NUM, &regulatory_chan_txpower_input_bw_to_bwidx);
	if (!*bw_map) {
		print_err(print, "Bad BW\n");
		goto error;
	}

	cur = strtok(NULL, ":");
	if (cur != NULL && cur[0] != '\0')
		goto error;

	return 0;

error:
	return -1;
}

static int
call_qcsapi_reg_chan_txpower_set(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	int power;
	int power_decimal;
	char *cur;
	unsigned fem_pri_map;
	unsigned bf_map;
	unsigned ss_map;
	unsigned bw_map;
	unsigned fem_pri, bf, ss, bw;
	struct qcsapi_chan_tx_powers_with_decimal_info info;
	qcsapi_chan_powers *pwrs = (qcsapi_chan_powers *) & info.maxpwr;
	qcsapi_chan_powers *pwrs_decimal = (qcsapi_chan_powers *) & info.maxpwr_decimal;
	unsigned arg_idx = 0;
	qcsapi_txpwr_value_type set_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;

	if (argc != 2 && argc != 4) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		goto usage;
	}

	while (arg_idx < argc) {
		if (argv[arg_idx][0] != '-')
			break;

		if (!strcmp(argv[arg_idx], "-t")) {
			if (++arg_idx == argc) {
				qcsapi_report_parameter_count(p_calling_bundle, argc);
				goto usage;
			}

			if (!strcmp(argv[arg_idx], "dntx")) {
				set_type = QCSAPI_PWR_VALUE_TYPE_DNTX;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				goto usage;
			}
		} else {
			print_err(print, "Invalid option %s\n", argv[arg_idx]);
			goto usage;
		}

		++arg_idx;
	}

	if ((arg_idx + 2) != argc) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		goto usage;
	}

	memset(&info, 0, sizeof(info));

	if (regulatory_chan_txpower_input_parse_bitmap(argv[arg_idx], print, &info.channel, &ss_map,
					&bf_map, &fem_pri_map, &bw_map))
		goto usage;

	cur = strtok(argv[++arg_idx], ",");

	for (ss = 0; ss < QCSAPI_PWR_IDX_SS_NUM; ++ss) {
		if (!(ss_map & (1 << ss)))
			continue;

		for (bf = 0; bf < QCSAPI_PWR_IDX_BF_NUM; ++bf) {
			if (!(bf_map & (1 << bf)))
				continue;

			for (fem_pri = 0; fem_pri < QCSAPI_PWR_IDX_FEM_PRIPOS_NUM; ++fem_pri) {
				if (!(fem_pri_map & (1 << fem_pri)))
					continue;

				for (bw = 0; bw < QCSAPI_PWR_BW_NUM; ++bw) {
					if (!(bw_map & (1 << bw)))
						continue;

					if (!cur) {
						print_err(print, "Not enough PWR values\n");
						goto usage;
					}

					if (strstr(cur, "."))
						sscanf(cur, "%d.%d", &power, &power_decimal);
					else {
						sscanf(cur, "%d", &power);
						power_decimal = 0;
					}

					if (power > INT8_MAX || power < INT8_MIN
							|| power_decimal > INT8_MAX
							|| power_decimal < 0)
						goto usage;

					cur = strtok(NULL, ",");

					(*pwrs)[fem_pri][bf][ss][bw] = (int8_t) power;
					(*pwrs_decimal)[fem_pri][bf][ss][bw] =
							(int8_t) power_decimal;
				}
			}
		}
	}

	if (cur) {
		print_err(print, "Too many PWR values\n");
		goto usage;
	}

	qcsapi_retval = qcsapi_regulatory_chan_txpower_with_decimal_set(the_interface, &info,
			set_type);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	qcsapi_report_usage(p_calling_bundle,
			"<interface> [-t <dntx>] <chan:nss:bf:fem_pri:bw> <power>\n");
	return 1;
}

static int
call_qcsapi_reg_chan_txpower_path_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	char file_path[80], *p_file_path = NULL;
	uint32_t path_len;
	int qcsapi_retval;

	if ((argc != 0) || (the_interface == NULL)) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		goto usage;
	}

	p_file_path = file_path;
	*p_file_path = 0;
	path_len = (uint32_t) sizeof(file_path);

	qcsapi_retval = qcsapi_regulatory_chan_txpower_path_get(the_interface, p_file_path,
			path_len);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", p_file_path);
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	qcsapi_report_usage(p_calling_bundle, "\n");
	return 1;
}

static inline void reg_chan_txpower_get_print_header(qcsapi_output *print, unsigned print_map)
{
	unsigned bw;

	print_out(print, "Ch:ss:bf:fp ");

	for (bw = QCSAPI_PWR_BW_160M; print_map; --bw) {
		if (!(print_map & (1 << bw)))
			continue;

		print_map &= ~(1 << bw);
		switch (bw) {
		case QCSAPI_PWR_BW_160M:
			print_out(print, "160M ");
			break;
		case QCSAPI_PWR_BW_80M:
			print_out(print, "80M  ");
			break;
		case QCSAPI_PWR_BW_40M:
			print_out(print, "40M  ");
			break;
		case QCSAPI_PWR_BW_20M:
			print_out(print, "20M");
			break;
		}
	}

	print_out(print, "\n");
}

static int
call_qcsapi_reg_chan_txpower_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	unsigned arg_idx = 0;
	int print_hdr = 1;
	unsigned fem_pri_map;
	unsigned bf_map;
	unsigned ss_map;
	unsigned bw_map;
	unsigned fem_pri, bf, ss, bw;
	struct qcsapi_chan_tx_powers_with_decimal_info info;
	qcsapi_chan_powers *pwrs = (qcsapi_chan_powers *) & info.maxpwr;
	qcsapi_chan_powers *pwrs_decimal = (qcsapi_chan_powers *) & info.maxpwr_decimal;
	qcsapi_txpwr_value_type report_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;
	int dntx = 0;

	if (argc < 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		goto usage;
	}

	while (arg_idx < argc) {
		if (argv[arg_idx][0] != '-')
			break;

		if (!strcmp(argv[arg_idx], "-n")) {
			print_hdr = 0;
		} else if (!strcmp(argv[arg_idx], "-f")) {
			if (++arg_idx == argc) {
				qcsapi_report_parameter_count(p_calling_bundle, argc);
				goto usage;
			}

			if (!strcmp(argv[arg_idx], "active")) {
				report_type = QCSAPI_PWR_VALUE_TYPE_ACTIVE;
			} else if (!strcmp(argv[arg_idx], "configured")) {
				report_type = QCSAPI_PWR_VALUE_TYPE_CONFIGURED;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				goto usage;
			}
		} else if (!strcmp(argv[arg_idx], "-t")) {
			if (++arg_idx == argc) {
				qcsapi_report_parameter_count(p_calling_bundle, argc);
				goto usage;
			}

			if (!strcmp(argv[arg_idx], "dntx")) {
				dntx = 1;
			} else {
				print_err(print, "Bad format %s\n", argv[arg_idx]);
				goto usage;
			}

		} else {
			print_err(print, "Invalid option %s\n", argv[arg_idx]);
			goto usage;
		}

		++arg_idx;
	}

	if (dntx == 1) {
		if (report_type == QCSAPI_PWR_VALUE_TYPE_ACTIVE)
			report_type = QCSAPI_PWR_VALUE_TYPE_DNTX;
		else
			report_type = QCSAPI_PWR_VALUE_TYPE_DNTX_CONFIGURED;
	}

	if ((arg_idx + 1) != argc) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		goto usage;
	}

	memset(&info, 0, sizeof(info));

	if (regulatory_chan_txpower_input_parse_bitmap(argv[arg_idx], print, &info.channel, &ss_map,
					&bf_map, &fem_pri_map, &bw_map))
		goto usage;

	qcsapi_retval = qcsapi_regulatory_chan_txpower_with_decimal_get(p_calling_bundle->
			caller_interface, &info, report_type);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (print_hdr)
		reg_chan_txpower_get_print_header(print, bw_map);

	for (fem_pri = 0; fem_pri < QCSAPI_PWR_IDX_FEM_PRIPOS_NUM; ++fem_pri) {
		if (!(fem_pri_map & (1 << fem_pri)))
			continue;

		for (bf = 0; bf < QCSAPI_PWR_IDX_BF_NUM; ++bf) {
			if (!(bf_map & (1 << bf)))
				continue;

			for (ss = 0; ss < QCSAPI_PWR_IDX_SS_NUM; ++ss) {
				if (!(ss_map & (1 << ss)))
					continue;

				print_out(print, "%3d:%d:%d:%d ",
						info.channel, ss + 1, bf, fem_pri);

				for (bw = QCSAPI_PWR_BW_160M; bw < QCSAPI_PWR_BW_NUM; --bw) {
					if (!(bw_map & (1 << bw)))
						continue;

					if ((*pwrs_decimal)[fem_pri][bf][ss][bw] == 0)
						print_out(print, "%4d ",
								(*pwrs)[fem_pri][bf][ss][bw]);
					else
						print_out(print, "%2d.%d ",
								(*pwrs)[fem_pri][bf][ss][bw],
								(*pwrs_decimal)[fem_pri][bf][ss]
								[bw]);
				}

				print_out(print, "\n");
			}
		}
	}

	return 0;

usage:
	qcsapi_report_usage(p_calling_bundle,
			"<interface> [-n] [-f <type>] [-t <dntx>] <channel:nss:bf:fem_pri:bw>\nwhere\n"
			"-n: do not print header (default is to print header)\n"
			"-f active|configured: Tx power values report format (default active)\n"
			"-t dntx: Type of Tx power values (Dynamic number of TX chains only)\n");
	return 1;
}

static int
call_qcsapi_wifi_set_chan_power_table(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_channel_power_table power_table;
	int statval = 0;
	int qcsapi_retval;
	uint8_t channel;
	int8_t max_power;
	int8_t backoff;
	uint32_t backoff_20m = 0;
	uint32_t backoff_40m = 0;
	uint32_t backoff_80m = 0;
	uint32_t backoff_160m = 0;
	char *endptr;
	int i;
	int offset;

	if (argc < 5) {
		print_err(print, "Not enough parameters in call_qcsapi set_chan_power_table\n");
		print_err(print, "Usage: call_qcsapi set_chan_power_table <interface> <channel>"
				" <max_power> <backoff_20M> <backoff_40M> <backoff_80M> <backoff_160M>\n");
		print_err(print, "backoff_20M/40M/80M/160M is a 32bits unsigned value, and every 4bits " "indicate the backoff from the max_power for a bf/ss case.\n" "The least significant 4 bits are for bfoff 1ss, and " "the most significant 4 bits are for bfon 4ss, and so forth.\n" "For example, max_power 23 and backoff_20M 0x54324321 means:\n" "  the power for 20Mhz bfoff 1ss: 23 - 1 = 22dBm\n" "  the power for 20Mhz bfoff 2ss: 23 - 2 = 21dBm\n" "  the power for 20Mhz bfoff 3ss: 23 - 3 = 20dBm\n" "  the power for 20Mhz bfoff 4ss: 23 - 4 = 19dBm\n" "  the power for 20Mhz bfon  1ss: 23 - 2 = 21dBm\n" "  the power for 20Mhz bfon  2ss: 23 - 3 = 20dBm\n" "  the power for 20Mhz bfon  3ss: 23 - 4 = 19dBm\n" "  the power for 20Mhz bfon  4ss: 23 - 5 = 18dBm\n");
		return 1;
	}

	channel = atoi(argv[0]);
	max_power = atoi(argv[1]);
	backoff_20m = strtoul(argv[2], &endptr, 0);
	backoff_40m = strtoul(argv[3], &endptr, 0);
	backoff_80m = strtoul(argv[4], &endptr, 0);
	if (argc > 5) {
		backoff_160m = strtoul(argv[5], &endptr, 0);
	}

	power_table.channel = channel;

	if (max_power <= 0) {
		print_err(print, "Invalid max_power %d\n", max_power);
		return 1;
	}

	for (i = 0, offset = 0; i < QCSAPI_POWER_TOTAL; i++, offset += 4) {
		backoff = (backoff_20m >> offset) & 0xf;
		if (max_power <= backoff) {
			print_err(print, "Invalid backoff_20m, too large backoff"
					" for power index %d, backoff %d\n", i, backoff);
			return 1;
		}
		power_table.power_20M[i] = max_power - backoff;

		backoff = (backoff_40m >> offset) & 0xf;
		if (max_power <= backoff) {
			print_err(print, "Invalid backoff_40m, too large backoff"
					" for power index %d, backoff %d\n", i, backoff);
			return 1;
		}
		power_table.power_40M[i] = max_power - backoff;

		backoff = (backoff_80m >> offset) & 0xf;
		if (max_power <= backoff) {
			print_err(print, "Invalid backoff_80m, too large backoff"
					" for power index %d, backoff %d\n", i, backoff);
			return 1;
		}
		power_table.power_80M[i] = max_power - backoff;

		backoff = (backoff_160m >> offset) & 0xf;
		if (max_power <= backoff) {
			print_err(print, "Invalid backoff_160m, too large backoff"
					" for power index %d, backoff %d\n", i, backoff);
			return 1;
		}
		power_table.power_160M[i] = max_power - backoff;
	}

	qcsapi_retval = qcsapi_wifi_set_chan_power_table(the_interface, &power_table);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_power_selection(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int power_selection;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_power_selection(&power_selection);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", power_selection);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_power_selection(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		print_err(print, "Incorrect parameters in call qcsapi set power selection\n");
		print_err(print, "Usage: call_qcsapi set_power_selection <0/1/2/3>\n");
		statval = 1;
	} else {
		qcsapi_unsigned_int power_selection = atoi(argv[0]);
		int qcsapi_retval;

		qcsapi_retval = qcsapi_wifi_set_power_selection(power_selection);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_carrier_interference(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	int ci = 0;

	qcsapi_retval = qcsapi_wifi_get_carrier_interference(the_interface, &ci);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%ddb\n", ci);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_congestion_idx(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	int ci;

	qcsapi_retval = qcsapi_wifi_get_congestion_index(the_interface, &ci);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d\n", ci);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_supported_tx_power_levels(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_128 power_available = "";
	char *p_power_available = &power_available[0];

	if (argc > 0 && strcmp(argv[0], "NULL") == 0) {
		p_power_available = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_supported_tx_power_levels(the_interface, p_power_available);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_power_available);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_current_tx_power_level(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int current_percentage = 0, *p_current_percentage = &current_percentage;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0) {
		p_current_percentage = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_current_tx_power_level(the_interface, p_current_percentage);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", (int)current_percentage);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_power_constraint(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set power constraint, count is %d\n", argc);
		statval = 1;
	} else {
		int temp = atoi(argv[0]);
		qcsapi_unsigned_int pwr_constraint = (qcsapi_unsigned_int) temp;

		if (temp < 0) {
			qcsapi_retval = -EINVAL;
		} else {
			qcsapi_retval = qcsapi_wifi_set_power_constraint(the_interface,
					pwr_constraint);
		}

		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_power_constraint(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int pwr_constraint, *p_pwr_constraint = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_pwr_constraint = &pwr_constraint;

	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_power_constraint(the_interface, p_pwr_constraint);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", pwr_constraint);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_tpc_interval(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set tpc interval, count is %d\n", argc);
		statval = 1;
	} else {
		int temp = atoi(argv[0]);

		if (temp <= 0) {
			qcsapi_retval = -EINVAL;
		} else {
			qcsapi_retval = qcsapi_wifi_set_tpc_interval(the_interface, temp);
		}

		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tpc_interval(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int tpc_interval, *p_tpc_interval = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tpc_interval = &tpc_interval;

	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_tpc_interval(the_interface, p_tpc_interval);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", tpc_interval);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scan_chk_inv(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "call_qcsapi set_scan_chk_inv wifi0 <scan_chk_inv>\n", argc);
		statval = 1;
	} else {
		int temp = atoi(argv[0]);

		if (temp <= 0 || temp > (24 * 60 * 60)) {
			print_err(print, "value should be limited from 1 second to 24 hours\n");
			qcsapi_retval = -EINVAL;
		} else {
			qcsapi_retval = qcsapi_wifi_set_scan_chk_inv(the_interface, temp);
		}

		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scan_chk_inv(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int scan_chk_inv = 0, *p = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p = &scan_chk_inv;

	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_scan_chk_inv(the_interface, p);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", scan_chk_inv);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void
local_display_assoc_records(qcsapi_output *print,
		const struct qcsapi_assoc_records *p_assoc_records)
{
	int iter;

	for (iter = 0; iter < QCSAPI_ASSOC_MAX_RECORDS; iter++) {
		if (p_assoc_records->timestamp[iter] <= 0) {
			return;
		}

		char mac_addr_string[24];

		snprintf(&mac_addr_string[0], sizeof(mac_addr_string), MACFILTERINGMACFMT,
				p_assoc_records->addr[iter][0],
				p_assoc_records->addr[iter][1],
				p_assoc_records->addr[iter][2],
				p_assoc_records->addr[iter][3],
				p_assoc_records->addr[iter][4], p_assoc_records->addr[iter][5]
				);

		print_out(print, "%s: %d\n", &mac_addr_string[0],
				(int)p_assoc_records->timestamp[iter]);
	}
}

static int
call_qcsapi_wifi_get_assoc_records(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	int reset_flag = 0;
	struct qcsapi_assoc_records assoc_records;
	struct qcsapi_assoc_records *p_assoc_records = &assoc_records;

	if (argc > 0) {
		if (!isdigit(argv[0][0])) {
			print_err(print, "get_assoc_records: reset flag must be a numeric value\n");
			return 1;
		}

		reset_flag = atoi(argv[0]);
	}

	if (argc > 1 && strcmp(argv[1], "NULL") == 0) {
		p_assoc_records = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_assoc_records(the_interface, reset_flag, p_assoc_records);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			local_display_assoc_records(print, &assoc_records);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_list_DFS_channels(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	static string_1024 the_list_channels;
	int qcsapi_retval;
	char *p_list_channels = NULL;
	const char *regulatory_region = NULL;
	int DFS_flag = 0;
	qcsapi_unsigned_int the_bw = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *ifname = NULL;

	if (argc < 2) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<region> {0 | 1} [BW] [iface]\n");
		return 1;
	}

	if (argc >= 4)
		ifname = argv[3];

	if (argc == 2 || ((argc >= 3) && !strcmp(argv[2], "current"))) {
		qcsapi_retval = qcsapi_wifi_get_bw(ifname, &the_bw);
		if (qcsapi_retval < 0)
			goto out;
	} else if (argc >= 3) {
		the_bw = atoi(argv[2]);
	}

	if (strcmp(argv[0], "NULL") != 0)
		regulatory_region = argv[0];

	DFS_flag = atoi(argv[1]);

	if (argc <= 4 || strcmp(argv[4], "NULL") != 0)
		p_list_channels = the_list_channels;

	if (ifname)
		qcsapi_retval = qcsapi_regulatory_get_list_regulatory_channels_if(ifname,
				regulatory_region, the_bw, DFS_flag, p_list_channels);
	else
		qcsapi_retval = qcsapi_regulatory_get_list_DFS_channels(regulatory_region, DFS_flag,
				the_bw, p_list_channels);

	if (qcsapi_retval == -qcsapi_region_database_not_found)
		qcsapi_retval = qcsapi_wifi_get_list_DFS_channels(regulatory_region, DFS_flag,
				the_bw, p_list_channels);

out:
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", the_list_channels);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_is_channel_DFS(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *regulatory_region = NULL;
	int DFS_flag = 0;
	int *p_DFS_flag = NULL;
	qcsapi_unsigned_int the_channel;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *ifname = NULL;

	if (argc < 2) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<region> <channel> [iface]\n");
		return 1;
	}

	if (strcmp(argv[0], "NULL") != 0)
		regulatory_region = argv[0];

	the_channel = (qcsapi_unsigned_int) atoi(argv[1]);

	if (argc >= 3)
		ifname = argv[2];

	if (argc <= 3 || strcmp(argv[3], "NULL") != 0)
		p_DFS_flag = &DFS_flag;

	if (ifname)
		qcsapi_retval = qcsapi_regulatory_is_channel_DFS_if(ifname, regulatory_region,
				the_channel, p_DFS_flag);
	else
		qcsapi_retval = qcsapi_regulatory_is_channel_DFS(regulatory_region, the_channel,
				p_DFS_flag);

	if (qcsapi_retval == -qcsapi_region_database_not_found)
		qcsapi_retval = qcsapi_wifi_is_channel_DFS(regulatory_region, the_channel,
				p_DFS_flag);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d\n", DFS_flag);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_DFS_alt_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int channel_value, *p_channel_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_channel_value = &channel_value;
	qcsapi_retval = qcsapi_wifi_get_DFS_alt_channel(the_interface, p_channel_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", channel_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_DFS_alt_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set DFS alt channel, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int dfs_alt_chan = atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_DFS_alt_channel(the_interface, dfs_alt_chan);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_reentry(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_start_dfs_reentry(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_radar_chain(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call_qcsapi set radar chain, count is %d, cmd format is: set_radar_chain <blockid> <antenna_id> \n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int radar_blockid = atoi(argv[0]);
		qcsapi_unsigned_int radar_chain_selection = atoi(argv[1]);
		int qcsapi_retval;

		qcsapi_retval = qcsapi_wifi_set_radar_chain(radar_blockid, radar_chain_selection);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_cce_channels(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int prev_chan = 0;
	qcsapi_unsigned_int cur_chan = 0;
	qcsapi_unsigned_int *p_prev_chan = &prev_chan;
	qcsapi_unsigned_int *p_cur_chan = &cur_chan;

	if (argc >= 2) {
		if (strcmp(argv[1], "NULL") == 0) {
			p_cur_chan = NULL;
		}
	}

	if (argc >= 1) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_prev_chan = NULL;
		}
	}

	qcsapi_retval = qcsapi_wifi_get_scs_cce_channels(the_interface, p_prev_chan, p_cur_chan);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d %d\n", (int)prev_chan, (int)cur_chan);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dfs_cce_channels(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int prev_chan = 0;
	qcsapi_unsigned_int cur_chan = 0;
	qcsapi_unsigned_int *p_prev_chan = &prev_chan;
	qcsapi_unsigned_int *p_cur_chan = &cur_chan;

	if (argc >= 2) {
		if (strcmp(argv[1], "NULL") == 0) {
			p_cur_chan = NULL;
		}
	}

	if (argc >= 1) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_prev_chan = NULL;
		}
	}

	qcsapi_retval = qcsapi_wifi_get_dfs_cce_channels(the_interface, p_prev_chan, p_cur_chan);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d %d\n", (int)prev_chan, (int)cur_chan);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_csw_records(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int reset = 0;
	int i;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_csw_record records;

	if (argc >= 1) {
		if (strcmp(argv[0], "1") == 0) {
			reset = 1;
		}
	}

	qcsapi_retval = qcsapi_wifi_get_csw_records(the_interface, reset, &records);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "channel switch history record count : %d\n", records.cnt);
			int index = records.index;
			int indextmp = 0;
			for (i = 0; i < records.cnt; i++) {
				indextmp = (index + QCSAPI_CSW_MAX_RECORDS -
						i) % QCSAPI_CSW_MAX_RECORDS;
				print_out(print, "time=%u channel=%u reason=%s\n",
						records.timestamp[indextmp],
						records.channel[indextmp],
						csw_reason_to_string(records.reason[indextmp]));
			}

			if (reset) {
				print_out(print, "clear records complete\n");
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_wifi_get_radar_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_radar_status rdstatus;

	if (argc < 1) {
		print_err(print, "Not enough parameters\n");
		statval = 1;
	} else {
		memset(&rdstatus, 0, sizeof(rdstatus));
		rdstatus.channel = atoi(argv[0]);
		qcsapi_retval = qcsapi_wifi_get_radar_status(the_interface, &rdstatus);

		if (qcsapi_retval >= 0) {
			print_out(print, "channel %d:\nradar_status=%d\nradar_count=%d\n",
					rdstatus.channel, rdstatus.flags,
					rdstatus.ic_radardetected);
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_WEP_encryption_level(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	string_64 WEP_encryption_level;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_WEP_encryption_level(the_interface, NULL);
	else
		qcsapi_retval = qcsapi_wifi_get_WEP_encryption_level(the_interface,
				WEP_encryption_level);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", WEP_encryption_level);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_WEP_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_64 wep_key = { 0 };
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int index = 0;

	if (argc < 1)
		goto usage;

	if (qcsapi_str_to_uint32(argv[0], &index) < 0)
		goto usage;

	qcsapi_retval = qcsapi_wifi_get_WEP_key(the_interface, index, wep_key);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", (char *)&wep_key[0]);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	qcsapi_report_usage(p_calling_bundle, "<interface> <index>\n");
	return 1;
}

static int
call_qcsapi_wifi_set_WEP_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int index = 0;
	char *p_wep_key = NULL;

	if (argc < 2)
		goto usage;

	if (qcsapi_str_to_uint32(argv[0], &index) < 0)
		goto usage;

	p_wep_key = argv[1];
	qcsapi_retval = qcsapi_wifi_set_WEP_key(the_interface, index, p_wep_key);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);

usage:
	qcsapi_report_usage(p_calling_bundle, "<interface> <index> <key>\n");
	return 1;
}

static int
call_qcsapi_wifi_get_WEP_key_index(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int index = 0;

	qcsapi_retval = qcsapi_wifi_get_WEP_key_index(the_interface, &index);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", index);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_WEP_key_index(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int index = 0;

	if (argc < 1)
		goto usage;

	if (qcsapi_str_to_uint32(argv[0], &index) < 0)
		goto usage;

	qcsapi_retval = qcsapi_wifi_set_WEP_key_index(the_interface, index);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);

usage:
	qcsapi_report_usage(p_calling_bundle, "<interface> <index>\n");
	return 1;
}

static int
call_qcsapi_wifi_remove_WEP_config(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_remove_WEP_config(the_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_WPA_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char encryption_modes[36], *p_encryption_modes = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_encryption_modes = &encryption_modes[0];
	qcsapi_retval = qcsapi_wifi_get_WPA_encryption_modes(the_interface, p_encryption_modes);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &encryption_modes[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_WPA_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set encryption mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_encryption_modes = argv[0];

		/* Encryption modes will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_encryption_modes = NULL;
		qcsapi_retval = qcsapi_wifi_set_WPA_encryption_modes(the_interface,
				p_encryption_modes);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_WPA_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char authentication_mode[36], *p_authentication_mode = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_authentication_mode = &authentication_mode[0];
	qcsapi_retval = qcsapi_wifi_get_WPA_authentication_mode(the_interface,
			p_authentication_mode);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &authentication_mode[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_WPA_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set authentication mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_authentication_mode = argv[0];

		/* Authentication mode will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_authentication_mode = NULL;
		qcsapi_retval = qcsapi_wifi_set_WPA_authentication_mode(the_interface,
				p_authentication_mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

void local_printout_get_params(qcsapi_output *print, struct qcsapi_set_parameters *get_params)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(get_params->param); i++) {
		if (get_params->param[i].key[0] == 0)
			break;

		print_out(print, "%s: %s\n", get_params->param[i].key,
				get_params->param[i].value);
	}
}

static int local_get_params(call_qcsapi_bundle *p_calling_bundle, const char *SSID,
				int argc, char *argv[])
{
	int i;
	int j;
	int qcsapi_retval = 0;
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_set_parameters get_params;
	const char usage1[] = "<WiFi interface> <param1> [<param2> ... <param8>]";
	const char usage2[] = "<WiFi interface> <ssid> [<param1> <param2> ... <param8>]";

	if (argc < 1 || argc > ARRAY_SIZE(get_params.param)) {
		if (!SSID)
			qcsapi_report_usage(p_calling_bundle, usage1);
		else
			qcsapi_report_usage(p_calling_bundle, usage2);
		return 1;
	}

	memset(&get_params, 0, sizeof(get_params));

	for (i = 0, j = 0; i < ARRAY_SIZE(get_params.param) && j < argc; i++, j++)
		strncpy(get_params.param[i].key, argv[j], sizeof(get_params.param[i].key) - 1);

	qcsapi_retval = qcsapi_get_params(the_interface, SSID, &get_params);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			local_printout_get_params(print, &get_params);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}


static int
call_qcsapi_wifi_get_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	return local_get_params(p_calling_bundle, NULL, argc, argv);
}

static int
call_qcsapi_wifi_set_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int i;
	int k;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_set_parameters set_params;
	const char usage[] =
		"<WiFi interface> <param1> <value1> [<param2> <value2>]... [<param8> <value8>]";

	if (argc < 2 || argc > (2 * ARRAY_SIZE(set_params.param)) || !!(argc % 2)) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	memset(&set_params, 0, sizeof(set_params));

	for (i = 0, k = 0; i < ARRAY_SIZE(set_params.param) && k < argc; i++, k += 2) {
		strncpy(set_params.param[i].key, argv[k], sizeof(set_params.param[i].key) - 1);
		strncpy(set_params.param[i].value, argv[k + 1],
				sizeof(set_params.param[i].value) - 1);
	}

	qcsapi_retval = qcsapi_set_params(the_interface, NULL, &set_params);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_SSID_get_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	return local_get_params(p_calling_bundle, p_SSID, argc, argv);
}

static int
call_qcsapi_SSID_set_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int i;
	int k;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	const char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
	struct qcsapi_set_parameters set_params;
	const char usage[] =
	"<WiFi interface> <ssid> <param1> <value1> [<param2> <value2>]...[<param8> <value8>]";

	if (argc < 2 || argc > (2 * ARRAY_SIZE(set_params.param)) || !!(argc % 2)) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	memset(&set_params, 0, sizeof(set_params));

	for (i = 0, k = 0; i < ARRAY_SIZE(set_params.param) && k < argc; i++, k += 2) {
		strncpy(set_params.param[i].key, argv[k], sizeof(set_params.param[i].key) - 1);
		strncpy(set_params.param[i].value, argv[k + 1],
				sizeof(set_params.param[i].value) - 1);
	}

	qcsapi_retval = qcsapi_set_params(the_interface, p_SSID, &set_params);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_interworking(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char interworking[2], *p_interworking = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_interworking = &interworking[0];

	qcsapi_retval = qcsapi_wifi_get_interworking(the_interface, p_interworking);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &interworking);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_interworking(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set interworking, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_interworking = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_interworking = NULL;

		qcsapi_retval = qcsapi_wifi_set_interworking(the_interface, p_interworking);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_80211u_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_256 value = { 0 };
	char *p_buffer = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<interface> <80211u_param>");
		return 1;
	}

	if (strcmp(argv[0], "NULL") != 0)
		p_buffer = value;

	qcsapi_retval = qcsapi_wifi_get_80211u_params(the_interface, argv[0], p_buffer);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_80211u_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *p_11u_param = argv[0];
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"set_80211u_params <interface> <param> <value>");
		return 1;
	}

	if (!strcmp(argv[0], "ipaddr_type_availability"))
		if (argc < 3) {
			print_err(print, "%s expects 2 arguments\n", argv[0]);
			return 1;
		}

	if (strcmp(argv[0], "NULL") == 0)
		p_11u_param = NULL;

	qcsapi_retval = qcsapi_wifi_set_80211u_params(the_interface, p_11u_param, argv[1], argv[2]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_set_sec_agent(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "Usage: call_qcsapi sec_agent {enable | disable | start} <key>\n";
	int retval;

	if (argc != 2) {
		print_err(print, usage);
		return 1;
	}

	retval = qcsapi_security_set_sec_agent(argv[0], argv[1], 0);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

static int
call_qcsapi_security_get_sec_agent_status(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_16 status;
	int retval;

	retval = qcsapi_security_get_sec_agent_status(status);
	if (retval) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	print_out(print, status);

	return 0;
}

static int
call_qcsapi_security_get_nai_realms(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_4096 nai_value;
	char *p_buffer = &nai_value[0];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_security_get_nai_realms(the_interface, p_buffer);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_buffer);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_security_add_nai_realm(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi WiFi add_nai_realm,"
				"count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		int encoding;
		char *p_nai_realm = argv[1];
		char *p_eap_method = argv[2];

		if (*argv[0] < '0' || *argv[0] > '1' || strlen(argv[0]) > 1) {
			print_err(print, "invalid encoding\n");
			return statval;
		}

		encoding = atoi(argv[0]);

		if (strcmp(argv[1], "NULL") == 0)
			p_nai_realm = NULL;

		if (strcmp(argv[2], "NULL") == 0)
			p_eap_method = NULL;

		qcsapi_retval = qcsapi_security_add_nai_realm(the_interface,
				encoding, p_nai_realm, p_eap_method);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_del_nai_realm(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi del_nai_realm,"
				"count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_nai_realm = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_nai_realm = NULL;

		qcsapi_retval = qcsapi_security_del_nai_realm(the_interface, p_nai_realm);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_get_roaming_consortium(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	string_1024 roaming_value;
	char *p_buffer = &roaming_value[0];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_security_get_roaming_consortium(the_interface, p_buffer);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_buffer);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_security_add_roaming_consortium(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"add_roaming_consortium count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_value = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_value = NULL;

		qcsapi_retval = qcsapi_security_add_roaming_consortium(the_interface, p_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_del_roaming_consortium(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"del_roaming_consortium count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_value = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_value = NULL;

		qcsapi_retval = qcsapi_security_del_roaming_consortium(the_interface, p_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_get_venue_name(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_4096 venue_name;
	char *p_venue_name = &venue_name[0];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_security_get_venue_name(the_interface, p_venue_name);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", venue_name);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_security_add_venue_name(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"add_venue_name, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_lang_code = argv[0];
		char *p_venue_name = argv[1];

		if (strcmp(argv[0], "NULL") == 0)
			p_lang_code = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_venue_name = NULL;

		qcsapi_retval = qcsapi_security_add_venue_name(the_interface, p_lang_code,
				p_venue_name);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_del_venue_name(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"del_venue_name, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_lang_code = argv[0];
		char *p_venue_name = argv[1];

		if (strcmp(argv[0], "NULL") == 0)
			p_lang_code = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_venue_name = NULL;

		qcsapi_retval = qcsapi_security_del_venue_name(the_interface, p_lang_code,
				p_venue_name);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_get_oper_friendly_name(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	string_4096 value;
	char *p_value = &value[0];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_security_get_oper_friendly_name(the_interface, p_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_security_add_oper_friendly_name(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"add_oper_friendly_name count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_lang_code = argv[0];
		char *p_oper_friendly_name = argv[1];

		if (strcmp(argv[0], "NULL") == 0)
			p_lang_code = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_oper_friendly_name = NULL;

		qcsapi_retval = qcsapi_security_add_oper_friendly_name(the_interface,
				p_lang_code, p_oper_friendly_name);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_del_oper_friendly_name(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"del_oper_friendly_name count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_lang_code = argv[0];
		char *p_oper_friendly_name = argv[1];

		if (strcmp(argv[0], "NULL") == 0)
			p_lang_code = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_oper_friendly_name = NULL;

		qcsapi_retval = qcsapi_security_del_oper_friendly_name(the_interface,
				p_lang_code, p_oper_friendly_name);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_get_hs20_conn_capab(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	string_4096 value;
	char *p_value = &value[0];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_security_get_hs20_conn_capab(the_interface, p_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_security_add_hs20_conn_capab(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"add_hs20_conn_capab count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_ip_proto = argv[0];
		char *p_port_num = argv[1];
		char *p_status = argv[2];

		if (strcmp(argv[0], "NULL") == 0)
			p_ip_proto = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_port_num = NULL;

		if (strcmp(argv[2], "NULL") == 0)
			p_status = NULL;

		qcsapi_retval = qcsapi_security_add_hs20_conn_capab(the_interface,
				p_ip_proto, p_port_num, p_status);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_del_hs20_conn_capab(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi WiFi "
				"del_hs20_conn_capab count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_ip_proto = argv[0];
		char *p_port_num = argv[1];
		char *p_status = argv[2];

		if (strcmp(argv[0], "NULL") == 0)
			p_ip_proto = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_port_num = NULL;

		if (strcmp(argv[2], "NULL") == 0)
			p_status = NULL;

		qcsapi_retval = qcsapi_security_del_hs20_conn_capab(the_interface,
				p_ip_proto, p_port_num, p_status);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_security_add_hs20_icon(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int width;
	qcsapi_unsigned_int height;

	if (argc < 6) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <Icon Width> <Icon Height> <Language Code> "
				"<Icon Type> <Name> <File Path>");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &width) < 0) {
		print_err(print, "<Icon Width> must be a number\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[1], &height) < 0) {
		print_err(print, "<Icon Height> must be a number\n");
		return 1;
	}

	qcsapi_retval = qcsapi_security_add_hs20_icon(the_interface, width, height,
			argv[2], argv[3], argv[4], argv[5]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_get_hs20_icon(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_1024 output_buf = { 0 };
	char *param = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL"))
		param = output_buf;

	qcsapi_retval = qcsapi_security_get_hs20_icon(the_interface, param);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, output_buf);
}

static int
call_qcsapi_security_del_hs20_icon(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<Icon Name>");
		return 1;
	}

	qcsapi_retval = qcsapi_security_del_hs20_icon(the_interface, argv[0]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_add_osu_server_uri(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<OSU Server URI>");
		return 1;
	}

	qcsapi_retval = qcsapi_security_add_osu_server_uri(the_interface, argv[0]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_get_osu_server_uri(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_1024 output_buf = { 0 };
	char *param = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL"))
		param = output_buf;

	qcsapi_retval = qcsapi_security_get_osu_server_uri(the_interface, param);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, output_buf);
}

static int
call_qcsapi_security_del_osu_server_uri(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<OSU Server URI>");
		return 1;
	}

	qcsapi_retval = qcsapi_security_del_osu_server_uri(the_interface, argv[0]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_add_osu_server_param(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle, "<OSU Server URI> <param> <value>");
		return 1;
	}

	qcsapi_retval = qcsapi_security_add_osu_server_param(the_interface,
			argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_security_get_osu_server_param(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_1024 output_buf = { 0 };
	char *param = NULL;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<OSU Server URI> <param>");
		return 1;
	}

	if (argc < 3 || strcmp(argv[1], "NULL"))
		param = output_buf;

	qcsapi_retval = qcsapi_security_get_osu_server_param(the_interface, argv[0],
			argv[1], param);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, output_buf);
}

static int
call_qcsapi_security_del_osu_server_param(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	const char *value = NULL;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<OSU Server URI> <param> [<value>]");
		return 1;
	}
	if (argc >= 3) {
		value = argv[2];
	}

	qcsapi_retval = qcsapi_security_del_osu_server_param(the_interface,
			argv[0], argv[1], value);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_hs20_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	string_32 hs20;
	char *p_hs20 = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_hs20 = &hs20[0];

	qcsapi_retval = qcsapi_wifi_get_hs20_status(the_interface, p_hs20);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, p_hs20);
}

static int
call_qcsapi_wifi_set_hs20_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set hotspot, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_hs20 = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_hs20 = NULL;

		qcsapi_retval = qcsapi_wifi_set_hs20_status(the_interface, p_hs20);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_proxy_arp(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in qcsapi set_proxy_arp, count is %d\n",
				argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *proxy_arp = argv[0];

		if (atoi(argv[0]) != 0 && atoi(argv[0]) != 1) {
			print_err(print, "Invalid input for set_proxy_arp use 0 or 1\n");
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_proxy_arp(the_interface, proxy_arp);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_proxy_arp(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char proxy_arp[2];
	char *p_proxy_arp = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0) {
		qcsapi_retval = -EFAULT;
	} else {
		p_proxy_arp = &proxy_arp[0];

		qcsapi_retval = qcsapi_wifi_get_proxy_arp(the_interface, p_proxy_arp);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_proxy_arp);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_l2_ext_filter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_32 value;

	if (argc < 1) {
		print_err(print, "Not enough parameters in qcsapi get_l2_ext_filter, count is %d\n",
				argc);
		statval = 1;
	} else {
		char *p_value = value;
		char *p_param = argv[0];

		if (strcmp(p_param, "NULL") == 0)
			p_param = NULL;

		qcsapi_retval = qcsapi_wifi_get_l2_ext_filter(the_interface, p_param, p_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", p_value);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_l2_ext_filter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in qcsapi set_l2_ext_filter, count is %d\n",
				argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_param = argv[0];
		char *p_value = argv[1];

		if (strcmp(argv[0], "NULL") == 0)
			p_param = NULL;

		if (strcmp(argv[1], "NULL") == 0)
			p_value = NULL;

		qcsapi_retval = qcsapi_wifi_set_l2_ext_filter(the_interface, p_param, p_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int check_hs20_param(char *lookup_name)
{
	int retval = 1;
	unsigned int iter;

	int hs20_param_count = ARRAY_SIZE(qcsapi_hs20_params);

	for (iter = 0; iter < hs20_param_count; iter++) {
		if (strcmp(qcsapi_hs20_params[iter], lookup_name) == 0) {
			retval = 0;
			break;
		}
	}
	return retval;
}

static int
call_qcsapi_wifi_get_hs20_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_64 value;
	char *p_value = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		print_out(print, "\n call_qcsapi get_hs20_params <interface>" " <hs20_param>\n");
		return 1;
	}

	p_value = &value[0];

	if (check_hs20_param(argv[0])) {
		print_out(print, "\n %s is not hs20 parameter\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_hs20_params(the_interface, argv[0], p_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_hs20_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set_hs20_params, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		if (check_hs20_param(argv[0])) {
			print_out(print, "\n %s is not hs20 parameter\n", argv[0]);
			return 1;
		}

		if (!strcmp(argv[0], "hs20_wan_metrics")) {
			if (argc != 7) {
				print_out(print, "\n call_qcsapi set_hs20_params <interface>"
						" hs20_wan_metrics <WAN_info> <uplink_speed> "
						"<downlink_speed> <uplink_load> "
						"<downlink_load> <LMD>\n");
				return 1;
			}
		}

		if (!strcmp(argv[0], "disable_dgaf")) {
			if (argc != 2) {
				print_out(print, "\n call_qcsapi set_hs20_params "
						"<interface> disable_dgaf <0:disable 1:enable>\n");
				return 1;
			}
		}

		qcsapi_retval = qcsapi_wifi_set_hs20_params(the_interface, argv[0],
				argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_remove_11u_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	return qcsapi_report_complete(p_calling_bundle,
			qcsapi_remove_11u_param(p_calling_bundle->caller_interface, argv[0]));
}

static int
call_qcsapi_remove_hs20_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;

	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi remove hs20_param, count is %d\n", argc);
		statval = 1;
	}

	if (check_hs20_param(argv[0])) {
		print_out(print, "%s is not hs20 parameter\n", argv[0]);
		statval = 1;
	} else {
		char *param = argv[0];

		qcsapi_retval = qcsapi_remove_hs20_param(the_interface, param);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;

}

static int
call_qcsapi_wifi_get_IEEE11i_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char encryption_modes[36], *p_encryption_modes = NULL;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_encryption_modes = &encryption_modes[0];

	qcsapi_retval = qcsapi_wifi_get_IEEE11i_encryption_modes(the_interface, p_encryption_modes);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &encryption_modes[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_IEEE11i_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *p_encryption_mode = argv[0];
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set authentication mode, count is %d\n", argc);
		statval = 1;
	} else {
		if (strcmp(argv[0], "NULL") == 0)
			p_encryption_mode = NULL;

		qcsapi_retval = qcsapi_wifi_set_IEEE11i_encryption_modes(the_interface,
				p_encryption_mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_IEEE11i_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char authentication_mode[36], *p_authentication_mode = NULL;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_authentication_mode = &authentication_mode[0];

	qcsapi_retval = qcsapi_wifi_get_IEEE11i_authentication_mode(the_interface,
			p_authentication_mode);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &authentication_mode[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_IEEE11i_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *p_authentication_mode = argv[0];
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set authentication mode, count is %d\n", argc);
		statval = 1;
	} else {
		if (strcmp(argv[0], "NULL") == 0)
			p_authentication_mode = NULL;

		qcsapi_retval = qcsapi_wifi_set_IEEE11i_authentication_mode(the_interface,
				p_authentication_mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_michael_errcnt(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	uint32_t errcnt;

	qcsapi_retval = qcsapi_wifi_get_michael_errcnt(the_interface, &errcnt);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", errcnt);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_pre_shared_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char pre_shared_key[68], *p_pre_shared_key = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int the_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_pre_shared_key = &pre_shared_key[0];
	qcsapi_retval = qcsapi_wifi_get_pre_shared_key(the_interface, the_index, p_pre_shared_key);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &pre_shared_key[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_pre_shared_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set pre-shared key, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_pre_shared_key = argv[0];
		qcsapi_unsigned_int the_index = p_calling_bundle->caller_generic_parameter.index;

		/* PSK will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_pre_shared_key = NULL;
		qcsapi_retval = qcsapi_wifi_set_pre_shared_key(the_interface, the_index,
				p_pre_shared_key);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_psk_auth_failures(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int psk_auth_failure_cnt = 0;

	qcsapi_retval = qcsapi_wifi_get_psk_auth_failures(the_interface, &psk_auth_failure_cnt);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", psk_auth_failure_cnt);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_key_passphrase(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char passphrase[68], *p_passphrase = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int the_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_passphrase = &passphrase[0];
	qcsapi_retval = qcsapi_wifi_get_key_passphrase(the_interface, the_index, p_passphrase);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &passphrase[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_key_passphrase(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set passphrase, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_index = p_calling_bundle->caller_generic_parameter.index;
		char *p_passphrase = argv[0];

		/* No, you cannot has a passphrase of NULL.  Too bad !! */

		if (strcmp(argv[0], "NULL") == 0)
			p_passphrase = NULL;
		qcsapi_retval = qcsapi_wifi_set_key_passphrase(the_interface, the_index,
				p_passphrase);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
local_get_key_interval(int (*p_key_get_hook) (const char *, unsigned int *),
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	unsigned int key_interval;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = p_key_get_hook(the_interface, &key_interval);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", key_interval);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_group_key_interval(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	return local_get_key_interval(qcsapi_wifi_get_group_key_interval, p_calling_bundle, argc,
			argv);
}

static int
call_qcsapi_wifi_get_pairwise_key_interval(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	return local_get_key_interval(qcsapi_wifi_get_pairwise_key_interval, p_calling_bundle, argc,
			argv);
}

static int
local_set_key_interval(int (*p_key_set_hook) (const char *, unsigned int),
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_group_key_interval = argv[0];
	int key_interval;
	int qcsapi_retval;

	key_interval = atoi(p_group_key_interval);

	qcsapi_retval = p_key_set_hook(the_interface, key_interval);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_group_key_interval(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi set group key interval, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_group_key_interval <WiFi interface> <group key interval>\n");
		print_err(print, " group key interval is in seconds, set to zero to disable group key rotation\n");
		return 1;
	}
	return local_set_key_interval(qcsapi_wifi_set_group_key_interval, p_calling_bundle, argc,
			argv);
}

static int
call_qcsapi_wifi_set_pairwise_key_interval(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi set pairwise key interval, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_pairwise_key_interval <WiFi interface> <pairwise key interval>\n");
		print_err(print, " pairwise key interval is in seconds, set to zero to disable pairwise key rotation\n");
		return 1;
	}
	return local_set_key_interval(qcsapi_wifi_set_pairwise_key_interval, p_calling_bundle, argc,
			argv);
}

static int call_qcsapi_wifi_get_pmf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int pmf_cap = 0;
	int *p_pmf_cap = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_pmf_cap = &pmf_cap;

	qcsapi_retval = qcsapi_wifi_get_pmf(the_interface, p_pmf_cap);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", pmf_cap);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_set_pmf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set pmf, count is %d\n",
				argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int pmf_cap = atoi(argv[0]);

		qcsapi_retval = qcsapi_wifi_set_pmf(the_interface, pmf_cap);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_pairing_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char pairing_id[33];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_pairing_id(the_interface, pairing_id);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", pairing_id);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_pairing_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set pairing ID, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *pairing_id = argv[0];

		qcsapi_retval = qcsapi_wifi_set_pairing_id(the_interface, pairing_id);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_pairing_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char pairing_enable[2];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_pairing_enable(the_interface, pairing_enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", pairing_enable);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_pairing_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set pairing enalbe flag, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *pairing_enable = argv[0];

		qcsapi_retval = qcsapi_wifi_set_pairing_enable(the_interface, pairing_enable);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_txqos_sched_tbl(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int index;
	int qcsapi_retval;
	char *endptr;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> <index>";

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_txqos_sched_tbl <WiFi interface>"
				" <index>\n");
		return 1;
	}

	index = strtol(argv[0], &endptr, 10);
	if (*endptr) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_txqos_sched_tbl(the_interface, index);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_txqos_sched_tbl(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int current_val = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_txqos_sched_tbl(the_interface, &current_val);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d\n", current_val);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_eth_phy_power_off(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Not enough parameters for call_qcsapi eth_phy_power_off %d\n",
				argc);
		print_err(print, "Format: call_qcsapi eth_phy_power_off ifname on_off\n");
		print_err(print, "ifname: interface name of the ethernet; on_off : 1 - off, 0 - on \n");
		statval = 1;
	} else {
		int qcsapi_retval;
		int on_off = atoi(argv[0]);

		qcsapi_retval = qcsapi_eth_phy_power_control(! !on_off, the_interface);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_set_aspm_l1(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters for the call_qcsapi set_aspm_l1 %d\n",
				argc);
		print_err(print, "Format: call_qcsapi set_aspm_l1 enable/disable [latency] \n");
		print_err(print, "1 - enable, 0 - disable; latency(0~6) \n");
		statval = 1;
	} else {
		int qcsapi_retval;
		int enable = atoi(argv[0]);
		int latency = 0;

		if (enable && argc == 1) {
			print_err(print, "please enter latency value \n");
			statval = 1;
			goto end;
		}

		if (enable)
			latency = atoi(argv[1]);

		qcsapi_retval = qcsapi_set_aspm_l1(enable, latency);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}

	}
end:
	return statval;
}

static int call_qcsapi_set_l1(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters for the call_qcsapi set_l1 %d\n", argc);
		print_err(print, "Format: call_qcsapi set_l1 enter/exit \n");
		print_err(print, "1 - enter, 0 - exit \n");
		goto call_qcsapi_set_l1_error;
	}
	int qcsapi_retval;
	int enter = atoi(argv[0]);

	if (enter != 0 && enter != 1) {
		print_err(print, "parameter (%d) is not supported \n");
		goto call_qcsapi_set_l1_error;
	}

	qcsapi_retval = qcsapi_set_l1(enter);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		goto call_qcsapi_set_l1_error;
	}

	return statval;

call_qcsapi_set_l1_error:
	statval = 1;
	return statval;
}

static int
call_qcsapi_wifi_get_mac_address_filtering(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_mac_address_filtering current_mac_address_filtering,
			*p_current_mac_address_filtering = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_current_mac_address_filtering = &current_mac_address_filtering;
	qcsapi_retval = qcsapi_wifi_get_mac_address_filtering(the_interface,
			p_current_mac_address_filtering);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", (int)current_mac_address_filtering);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mac_address_filtering(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set MAC address filtering, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_address_filtering current_mac_address_filtering =
				(qcsapi_mac_address_filtering) atoi(argv[0]);

		qcsapi_retval = qcsapi_wifi_set_mac_address_filtering(the_interface,
				current_mac_address_filtering);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_is_mac_address_authorized(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi is MAC address authorized, count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr the_mac_addr;
		int qcsapi_retval;
		int ival = 0, is_authorized = -1;

		if (strcmp("NULL", argv[0]) == 0)
			qcsapi_retval = qcsapi_wifi_is_mac_address_authorized(the_interface, NULL,
					&is_authorized);
		else {
			ival = parse_mac_addr(argv[0], the_mac_addr);
			if (ival >= 0)
				qcsapi_retval = qcsapi_wifi_is_mac_address_authorized(the_interface,
						the_mac_addr, &is_authorized);
			else {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}
		}

		if (ival >= 0) {
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "%d\n", is_authorized);
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

#define QCSAPI_AUTH_MAC_ADDR_SIZE 126
static int
call_qcsapi_wifi_get_authorized_mac_addresses(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *authorized_mac_addresses = NULL;
	unsigned int sizeof_authorized_mac_addresses = QCSAPI_AUTH_MAC_ADDR_SIZE;

	if (argc > 0) {
		uint32_t usr_input = 0;

		if (qcsapi_str_to_uint32(argv[0], &usr_input)) {
			print_err(print, "Invalid parameter %s - must be an unsigned integer\n",
					argv[0]);
			return 1;
		}

		sizeof_authorized_mac_addresses = (usr_input < QCSAPI_MSG_BUFSIZE) ?
				usr_input : QCSAPI_MSG_BUFSIZE;
	}

	authorized_mac_addresses = malloc(sizeof_authorized_mac_addresses);
	if (authorized_mac_addresses == NULL) {
		print_err(print, "Failed to allocate %u chars\n", sizeof_authorized_mac_addresses);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_authorized_mac_addresses(the_interface,
			authorized_mac_addresses, sizeof_authorized_mac_addresses);

	if (qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, authorized_mac_addresses))
		statval = 1;

	free(authorized_mac_addresses);

	return statval;
}

static int
call_qcsapi_wifi_get_denied_mac_addresses(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *denied_mac_addresses = NULL;
	unsigned int sizeof_denied_mac_addresses = QCSAPI_AUTH_MAC_ADDR_SIZE;

	if (argc > 0) {
		uint32_t usr_input = 0;

		if (qcsapi_str_to_uint32(argv[0], &usr_input)) {
			print_err(print, "Invalid parameter %s - must be an unsigned integer\n",
					argv[0]);
			return 1;
		}

		sizeof_denied_mac_addresses = usr_input < QCSAPI_MSG_BUFSIZE ?
				usr_input : QCSAPI_MSG_BUFSIZE;
	}

	denied_mac_addresses = malloc(sizeof_denied_mac_addresses);
	if (denied_mac_addresses == NULL) {
		print_err(print, "Failed to allocate %u chars\n", sizeof_denied_mac_addresses);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_denied_mac_addresses(the_interface, denied_mac_addresses,
			sizeof_denied_mac_addresses);

	if (qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, denied_mac_addresses))
		statval = 1;

	free(denied_mac_addresses);

	return statval;
}

static int
call_qcsapi_wifi_authorize_mac_address(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi authorize MAC address,count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr_list the_mac_addr_list;
		int qcsapi_retval = 0;
		int count = 0;

		if (strcmp("NULL", argv[0]) == 0)
			qcsapi_retval = qcsapi_wifi_authorize_mac_address_list(the_interface, 0,
					NULL);
		else {
			for (count = 0; count < MIN(argc, MAC_ADDR_LIST_SIZE); count++) {
				qcsapi_retval = parse_mac_addr(argv[count],
						&the_mac_addr_list[count * MAC_ADDR_SIZE]);
				if (qcsapi_retval < 0)
					break;
			}

			if (count > 0) {
				qcsapi_retval = qcsapi_wifi_authorize_mac_address_list
						(the_interface, count, the_mac_addr_list);
			} else {
				print_out(print, "Error parsing MAC address %s\n", argv[count]);
				statval = 1;
			}
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_deny_mac_address(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi deny MAC address, count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr_list the_mac_addr_list;
		int qcsapi_retval = 0;
		int count = 0;

		if (strcmp("NULL", argv[0]) == 0)
			qcsapi_retval = qcsapi_wifi_deny_mac_address_list(the_interface, 0, NULL);
		else {
			for (count = 0; count < MIN(argc, MAC_ADDR_LIST_SIZE); count++) {
				qcsapi_retval = parse_mac_addr(argv[count],
						&the_mac_addr_list[count * MAC_ADDR_SIZE]);
				if (qcsapi_retval < 0)
					break;
			}

			if (count > 0) {
				qcsapi_retval = qcsapi_wifi_deny_mac_address_list(the_interface,
						count, the_mac_addr_list);
			} else {
				print_out(print, "Error parsing MAC address %s\n", argv[count]);
				statval = 1;
			}
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_remove_mac_address(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi remove MAC address, count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr_list the_mac_addr_list;
		int qcsapi_retval = 0;
		int count = 0;

		if (strcmp("NULL", argv[0]) == 0)
			qcsapi_retval = qcsapi_wifi_remove_mac_address_list(the_interface, 0, NULL);
		else {
			for (count = 0; count < MIN(argc, MAC_ADDR_LIST_SIZE); count++) {
				qcsapi_retval = parse_mac_addr(argv[count],
						&the_mac_addr_list[count * MAC_ADDR_SIZE]);
				if (qcsapi_retval < 0)
					break;
			}

			if (count > 0) {
				qcsapi_retval = qcsapi_wifi_remove_mac_address_list(the_interface,
						count, the_mac_addr_list);
			} else {
				print_out(print, "Error parsing MAC address %s\n", argv[count]);
				statval = 1;
			}
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_clear_mac_address_filters(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_clear_mac_address_filters(the_interface);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mac_address_reserve(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	int qcsapi_retval;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi WiFi reserve MAC address, count is %d\n", argc);
		return 1;
	} else if (argc == 3) {
		qcsapi_retval = qcsapi_wifi_set_macaddr_reserve(the_interface, argv[0],
				argv[1], argv[2], "");
	} else {
		qcsapi_retval = qcsapi_wifi_set_macaddr_reserve(the_interface, argv[0],
				argv[1], argv[2], argv[3]);
	}

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

static int
call_qcsapi_wifi_get_mac_address_reserve(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_256 buf;
	int qcsapi_retval;

	qcsapi_retval = qcsapi_wifi_get_mac_address_reserve(the_interface, buf);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_out(print, "%s", buf);

	return 0;
}

static int
call_qcsapi_wifi_clear_mac_address_reserve(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	int qcsapi_retval;

	qcsapi_retval = qcsapi_wifi_clear_mac_address_reserve(the_interface);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

static int
call_qcsapi_wifi_backoff_fail_max(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi backoff fail max, count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		int qcsapi_retval;
		int backoff_fail_max = atoi(argv[0]);

		qcsapi_retval = qcsapi_wifi_backoff_fail_max(the_interface, backoff_fail_max);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_backoff_timeout(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi backoff timeout, count is %d\n", argc);
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		int qcsapi_retval;
		int backoff_timeout = atoi(argv[0]);

		qcsapi_retval = qcsapi_wifi_backoff_timeout(the_interface, backoff_timeout);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wps_registrar_report_button_press(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = qcsapi_wps_registrar_report_button_press(the_interface);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_registrar_report_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "registrar report pin: required WPS PIN not present\n");
		statval = 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		const char *p_wps_pin = NULL;
		int qcsapi_retval;

		if (strcmp(argv[0], "NULL") != 0) {
			p_wps_pin = argv[0];
		}

		qcsapi_retval = qcsapi_wps_registrar_report_pin(the_interface, p_wps_pin);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wps_registrar_get_pp_devname(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_128 pp_devname = "";
	char *p_pp_devname = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int blacklist = 0;

	if (argc == 1 && strcmp(argv[0], "blacklist") == 0) {
		blacklist = 1;
	}
	if (argc >= 1 && strcmp(argv[0], "NULL") != 0) {
		p_pp_devname = &pp_devname[0];
	}

	qcsapi_retval = qcsapi_wps_registrar_get_pp_devname(the_interface, blacklist, p_pp_devname);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_pp_devname);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_registrar_set_pp_devname(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *p_pp_devname = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t wps_pp_status;
	int update_blacklist = 0;

	if (argc == 1) {
		p_pp_devname = strcmp(argv[0], "NULL") == 0 ? NULL : argv[0];
	} else if (argc == 2 && strcmp(argv[0], "blacklist") == 0) {
		update_blacklist = 1;
		p_pp_devname = strcmp(argv[1], "NULL") == 0 ? NULL : argv[1];
	} else {
		print_err(print, "WPS Registrar Set PP Devname: \n"
				"setting white-list: call_qcsapi registrar_set_pp_devname <device name list>\n"
				"setting black-list: call_qcsapi registrar_set_pp_devname blacklist <device name list>\n");
		return 0;
	}

	qcsapi_retval = qcsapi_wps_get_access_control(the_interface, &wps_pp_status);
	if (qcsapi_retval >= 0) {
		if (wps_pp_status == 0) {
			print_err(print, "enable WPS Pairing Protection before setting device name list\n");
			return 1;
		}
	}

	if (qcsapi_retval >= 0)
		qcsapi_retval = qcsapi_wps_registrar_set_pp_devname(the_interface, update_blacklist,
				p_pp_devname);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_enrollee_report_button_press(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int ival = 0;
	qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0) {
		/*
		 *Interpret BSSID parameter of "any" as direction to pass BSSID of all zeros to the
		 *API - so the WPS process will associate with any registrar.
		 */
		if (strcasecmp(argv[0], "any") != 0) {
			ival = parse_mac_addr(argv[0], local_bssid);

			if (ival < 0) {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}
		}
	}

	if (ival >= 0) {
		const char *the_interface = p_calling_bundle->caller_interface;
		int qcsapi_retval = qcsapi_wps_enrollee_report_button_press(the_interface,
				local_bssid);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wps_enrollee_report_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "enrollee report pin: required WPS PIN not present\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
		const char *p_wps_pin = NULL;
		int ival = 0;
		int pin_argv_index = 0;

		if (argc > 1) {
			if (strcasecmp(argv[0], "any") != 0) {
				ival = parse_mac_addr(argv[0], local_bssid);
			}

			if (ival < 0) {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			} else {
				pin_argv_index = 1;
			}
		}

		if (ival >= 0) {
			if (strcmp(argv[pin_argv_index], "NULL") != 0) {
				p_wps_pin = argv[pin_argv_index];
			}

			qcsapi_retval = qcsapi_wps_enrollee_report_pin(the_interface,
					local_bssid, p_wps_pin);
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int
call_qcsapi_wps_enrollee_generate_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int ival = 0;
	qcsapi_mac_addr local_bssid = { 0, 0, 0, 0, 0, 0 };
	char generated_pin[QCSAPI_WPS_MAX_PIN_LEN + 1];
	char *p_generated_pin = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0) {
		if (argc < 2 || strcmp(argv[1], "NULL") != 0) {
			p_generated_pin = &generated_pin[0];
		}
		/*
		 *Interpret BSSID parameter of "any" as direction to pass BSSID of all zeros to the
		 *API - so the WPS process will associate with any registrar.
		 */
		if (strcasecmp(argv[0], "any") != 0) {
			ival = parse_mac_addr(argv[0], local_bssid);

			if (ival < 0) {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}
		}
	} else {
		p_generated_pin = &generated_pin[0];
	}

	if (ival >= 0) {
		const char *the_interface = p_calling_bundle->caller_interface;
		int qcsapi_retval = qcsapi_wps_enrollee_generate_pin(the_interface,
				local_bssid,
				p_generated_pin);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", &generated_pin[0]);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wps_get_ap_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *iface = p_calling_bundle->caller_interface;
	char generated_pin[QCSAPI_WPS_MAX_PIN_LEN + 1];
	qcsapi_output *print = p_calling_bundle->caller_output;
	int force_regenerate = 0;

	if (argc == 1) {
		force_regenerate = atoi(argv[0]);
	} else if (argc > 1) {
		print_err(print, "Too many arguments for wps_get_ap_pin\n");
		return 1;
	}

	if (force_regenerate != 0 && force_regenerate != 1) {
		print_err(print, "Invalid parameter for force regenerate option: \"%s\" - must be 0 or 1", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wps_get_ap_pin(iface, generated_pin, force_regenerate);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", generated_pin);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void local_set_wps_ap_pin_usage(qcsapi_output *print, int out)
{
	if (!out) {
		print_out(print, "usage: call_qscapi set_wps_ap_pin <AP PIN>\n"
				"AP PIN: 8bit or 4 bit digits\n");
	} else {
		print_err(print, "usage: call_qscapi set_wps_ap_pin <AP PIN>\n"
				"AP PIN: 8bit or 4 bit digits\n");
	}
}

static int call_qcsapi_wps_set_ap_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *iface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char wps_pin[2 * QCSAPI_WPS_MAX_PIN_LEN] = { 0 };

	if (argc <= 0) {
		local_set_wps_ap_pin_usage(print, 1);
		return 1;
	}

	strncpy(wps_pin, argv[0], sizeof(wps_pin) - 1);

	qcsapi_retval = qcsapi_wps_set_ap_pin(iface, wps_pin);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_save_ap_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *iface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 0) {
		print_err(print, "usage: call_qscapi save_wps_ap_pin\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wps_save_ap_pin(iface);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		if (qcsapi_retval == -qcsapi_parameter_not_found)
			print_err(print, "no ap PIN exists, set or generate one\n");
		else
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_enable_ap_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *iface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int enable;

	if (argc != 1) {
		print_err(print, "usage: call_qscapi enable_wps_ap_pin {0 | 1}\n");
		return 1;
	}

	enable = atoi(argv[0]);
	if (strlen(argv[0]) > 1 || !isdigit(*argv[0]) || (enable != 0 && enable != 1)) {
		print_err(print, "usage: call_qscapi enable_wps_ap_pin {0 | 1}\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wps_enable_ap_pin(iface, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_generate_random_pin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *iface = p_calling_bundle->caller_interface;
	char generated_pin[QCSAPI_WPS_MAX_PIN_LEN + 1];
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wps_get_sta_pin(iface, generated_pin);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", generated_pin);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define WPS_GET_STATE_MAX_LEN	128

static int call_qcsapi_wps_get_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_unsigned_int message_len = WPS_GET_STATE_MAX_LEN;
	char wps_state[WPS_GET_STATE_MAX_LEN] = "";
	char *p_wps_state = &wps_state[0];

	if (argc > 0) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_wps_state = NULL;
		} else if (isdigit(argv[0][0])) {
			message_len = atoi(argv[0]);

			if (message_len > WPS_GET_STATE_MAX_LEN) {
				message_len = WPS_GET_STATE_MAX_LEN;
			}
		}
	}

	qcsapi_retval = qcsapi_wps_get_state(the_interface, p_wps_state, message_len);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_wps_state);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define WPA_GET_STATUS_MAX_LEN		32
#define MAC_ADDR_STR_LEN		17
static int
call_qcsapi_wifi_get_wpa_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_unsigned_int message_len = WPA_GET_STATUS_MAX_LEN;
	char wpa_status[WPA_GET_STATUS_MAX_LEN] = "";
	char *p_wpa_status = &wpa_status[0];
	char mac_addr[MAC_ADDR_STR_LEN + 1] = { 0 };

	if (argc > 0) {
		if (argc == 2) {
			if (isdigit(argv[1][0])) {
				message_len = atoi(argv[1]);

				if (message_len > WPA_GET_STATUS_MAX_LEN) {
					message_len = WPA_GET_STATUS_MAX_LEN;
				}
			}
		}

		if (strnlen(argv[0], MAC_ADDR_STR_LEN + 1) == MAC_ADDR_STR_LEN) {
			strcpy(mac_addr, argv[0]);
		} else {
			print_out(print, "mac address input error \n");
			return statval;
		}
	}

	qcsapi_retval = qcsapi_wifi_get_wpa_status(the_interface, p_wpa_status, mac_addr,
			message_len);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_wpa_status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_auth_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	char mac_addr[MAC_ADDR_STR_LEN + 1] = { 0 };
	int auth_state = 0;

	if (argc > 0) {
		if (strnlen(argv[0], (MAC_ADDR_STR_LEN + 1)) == MAC_ADDR_STR_LEN) {
			strcpy(mac_addr, argv[0]);
		} else {
			print_out(print, "Mac address input is invalid!\n");
			return statval;
		}
	} else {
		print_out(print, "Mac address should be input!\n");
		return statval;
	}

	qcsapi_retval = qcsapi_wifi_get_auth_state(the_interface, mac_addr, &auth_state);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", auth_state);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_disconn_info(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_disconn_info info;

	memset(&info, 0, sizeof(info));
	qcsapi_retval = qcsapi_wifi_get_disconn_info(the_interface, &info);

	if (qcsapi_retval >= 0) {
		print_out(print, "association\t%d\n"
				"disconnect\t%d\n"
				"sequence\t%d\n"
				"uptime\t%d\n", info.asso_sta_count, info.disconn_count,
				info.sequence, info.up_time);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_reset_disconn_info(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_disconn_info info;

	memset(&info, 0, sizeof(info));
	info.resetflag = 1;
	qcsapi_retval = qcsapi_wifi_get_disconn_info(the_interface, &info);

	if (qcsapi_retval >= 0) {
		print_out(print, "Reset complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_get_configured_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_unsigned_int message_len = WPS_GET_STATE_MAX_LEN;
	char wps_state[WPS_GET_STATE_MAX_LEN] = "";
	char *p_wps_state = &wps_state[0];

	if (argc > 0) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_wps_state = NULL;
		} else if (isdigit(argv[0][0])) {
			message_len = atoi(argv[0]);
			if (message_len > WPS_GET_STATE_MAX_LEN) {
				message_len = WPS_GET_STATE_MAX_LEN;
			}
		}
	}

	qcsapi_retval = qcsapi_wps_get_configured_state(the_interface, p_wps_state, message_len);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_wps_state);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_get_runtime_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	qcsapi_unsigned_int message_len = WPS_GET_STATE_MAX_LEN;
	char wps_state[WPS_GET_STATE_MAX_LEN] = "";
	char *p_wps_state = &wps_state[0];

	if (argc > 0) {
		if (strcmp(argv[0], "NULL") == 0) {
			p_wps_state = NULL;
		} else if (isdigit(argv[0][0])) {
			message_len = atoi(argv[0]);
			if (message_len > WPS_GET_STATE_MAX_LEN) {
				message_len = WPS_GET_STATE_MAX_LEN;
			}
		}
	}

	qcsapi_retval = qcsapi_wps_get_runtime_state(the_interface, p_wps_state, message_len);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", p_wps_state);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_allow_pbc_overlap(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int allow = ! !atoi(argv[0]);

	qcsapi_retval = qcsapi_wps_allow_pbc_overlap(the_interface, allow);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_get_allow_pbc_overlap_status(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int status = -1;
	const char *iface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wps_get_allow_pbc_overlap_status(iface, &status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_check_config(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval;
	const char *ifname = p_calling_bundle->caller_interface;

	retval = qcsapi_wps_check_config(ifname);

	return qcsapi_report_complete(p_calling_bundle, retval);
}

#define WPS_GET_CFG_MAX_LEN 100

static int call_qcsapi_wps_get_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int qcsapi_retval;
	qcsapi_wps_param_type wps_cfg_str_id;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char  *wps_cfg_str;

	if (argc > 0) {
		if (strcmp(argv[0], "uuid") == 0) {
			wps_cfg_str_id = qcsapi_wps_uuid;
		} else if (strcmp(argv[0], "os_version") == 0) {
			wps_cfg_str_id = qcsapi_wps_os_version;
		} else if (strcmp(argv[0], "device_name") == 0) {
			wps_cfg_str_id = qcsapi_wps_device_name;
		} else if (strcmp(argv[0], "config_methods") == 0) {
			wps_cfg_str_id = qcsapi_wps_config_methods;
		} else if (strcmp(argv[0], "ap_setup_locked") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_setup_locked;
		} else if (strcmp(argv[0], "last_config_error") == 0) {
			wps_cfg_str_id = qcsapi_wps_last_config_error;
		} else if (strcmp(argv[0], "registrar_number") == 0) {
			wps_cfg_str_id = qcsapi_wps_registrar_number;
		} else if (strcmp(argv[0], "registrar_established") == 0) {
			wps_cfg_str_id = qcsapi_wps_registrar_established;
		} else if (strcmp(argv[0], "force_broadcast_uuid") == 0) {
			wps_cfg_str_id = qcsapi_wps_force_broadcast_uuid;
		} else if (strcmp(argv[0], "ap_pin_fail_method") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_pin_fail_method;
		} else if (strcmp(argv[0], "auto_lockdown_max_retry") == 0) {
			wps_cfg_str_id = qcsapi_wps_auto_lockdown_max_retry;
		} else if (strcmp(argv[0], "auto_lockdown_fail_num") == 0) {
			wps_cfg_str_id = qcsapi_wps_auto_lockdown_fail_num;
		} else if (strcmp(argv[0], "wps_vendor_spec") == 0) {
			wps_cfg_str_id = qcsapi_wps_vendor_spec;
		} else if (strcmp(argv[0], "last_wps_client") == 0) {
			wps_cfg_str_id = qcsapi_wps_last_successful_client;
		} else if (strcmp(argv[0], "last_wps_client_devname") == 0) {
			wps_cfg_str_id = qcsapi_wps_last_successful_client_devname;
		} else if (strcmp(argv[0], "serial_number") == 0) {
			wps_cfg_str_id = qcsapi_wps_serial_number;
		} else if (strcmp(argv[0], "manufacturer") == 0) {
			wps_cfg_str_id = qcsapi_wps_manufacturer;
		} else if (strcmp(argv[0], "model_name") == 0) {
			wps_cfg_str_id = qcsapi_wps_model_name;
		} else if (strcmp(argv[0], "model_number") == 0) {
			wps_cfg_str_id = qcsapi_wps_model_number;
		} else if (strcmp(argv[0], "pbc_in_m1") == 0) {
			wps_cfg_str_id = qcsapi_wps_pbc_in_m1;
		} else if (strcmp(argv[0], "third_party_band") == 0) {
			wps_cfg_str_id = qcsapi_wps_third_party_band;
		} else if (strcmp(argv[0], "upnp_iface") == 0) {
			wps_cfg_str_id = qcsapi_upnp_iface;
		} else {
			print_err(print, "wps cfg string ID input error! \n");
			return 1;
		}
	} else {
		print_err(print, "please input wps cfg string ID\n");
		return 1;
	}

	wps_cfg_str = calloc(QCSAPI_MAX_PARAM_VAL_LEN, sizeof(char));
	if (!wps_cfg_str)
		return -ENOMEM;
	qcsapi_retval = qcsapi_wps_get_param(the_interface, wps_cfg_str_id, wps_cfg_str,
			QCSAPI_MAX_PARAM_VAL_LEN - 1);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", wps_cfg_str);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	if (wps_cfg_str)
		free(wps_cfg_str);
	return statval;
}

static int call_qcsapi_wps_set_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int qcsapi_retval;
	qcsapi_wps_param_type wps_cfg_str_id;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc >= 2) {
		if (strcmp(argv[0], "ap_pin") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_pin;
		} else if (strcmp(argv[0], "config_methods") == 0) {
			wps_cfg_str_id = qcsapi_wps_config_methods;
		} else if (strcmp(argv[0], "setup_lock") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_setup_locked;
		} else if (strcmp(argv[0], "ap_setup_locked") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_setup_locked;
		} else if (strcmp(argv[0], "uuid") == 0) {
			wps_cfg_str_id = qcsapi_wps_uuid;
		} else if (strcmp(argv[0], "force_broadcast_uuid") == 0) {
			wps_cfg_str_id = qcsapi_wps_force_broadcast_uuid;
		} else if (strcmp(argv[0], "device_name") == 0) {
			wps_cfg_str_id = qcsapi_wps_device_name;
		} else if (strcmp(argv[0], "ap_pin_fail_method") == 0) {
			wps_cfg_str_id = qcsapi_wps_ap_pin_fail_method;
		} else if (strcmp(argv[0], "auto_lockdown_max_retry") == 0) {
			wps_cfg_str_id = qcsapi_wps_auto_lockdown_max_retry;
		} else if (strcmp(argv[0], "wps_vendor_spec") == 0) {
			wps_cfg_str_id = qcsapi_wps_vendor_spec;
		} else if (strcmp(argv[0], "serial_number") == 0) {
			wps_cfg_str_id = qcsapi_wps_serial_number;
		} else if (strcmp(argv[0], "manufacturer") == 0) {
			wps_cfg_str_id = qcsapi_wps_manufacturer;
		} else if (strcmp(argv[0], "model_name") == 0) {
			wps_cfg_str_id = qcsapi_wps_model_name;
		} else if (strcmp(argv[0], "model_number") == 0) {
			wps_cfg_str_id = qcsapi_wps_model_number;
		} else if (strcmp(argv[0], "pbc_in_m1") == 0) {
			wps_cfg_str_id = qcsapi_wps_pbc_in_m1;
		} else if (strcmp(argv[0], "third_party_band") == 0) {
			wps_cfg_str_id = qcsapi_wps_third_party_band;
		} else if (strcmp(argv[0], "upnp_iface") == 0) {
			wps_cfg_str_id = qcsapi_upnp_iface;
		} else {
			print_err(print, "WPS param type string input error or not supported!\n");
			return statval;
		}
	} else {
		print_err(print, "Input WPS param type string and param value!\n");
		return statval;
	}

	qcsapi_retval = qcsapi_wps_set_param(the_interface, wps_cfg_str_id, argv[1]);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_set_configured_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint8_t new_value;
	int qcsapi_retval;

	if (argc < 1) {
		print_err(print, "New WPS state argument required");
		return 1;
	}

	new_value = (uint8_t) atoi(argv[0]);

	qcsapi_retval = qcsapi_wps_set_configured_state(interface, new_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_dwell_times(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	unsigned int max_dwell_time_active_chan;
	unsigned int min_dwell_time_active_chan;
	unsigned int max_dwell_time_passive_chan;
	unsigned int min_dwell_time_passive_chan;
	int statval = 0;

	if (argc < 4) {
		print_err(print, "STA Set Dwell Times requires 4 dwell times\n");
		return 1;
	}

	max_dwell_time_active_chan = (unsigned int)atoi(argv[0]);
	min_dwell_time_active_chan = (unsigned int)atoi(argv[1]);
	max_dwell_time_passive_chan = (unsigned int)atoi(argv[2]);
	min_dwell_time_passive_chan = (unsigned int)atoi(argv[3]);

	qcsapi_retval = qcsapi_wifi_set_dwell_times(the_interface,
			max_dwell_time_active_chan,
			min_dwell_time_active_chan,
			max_dwell_time_passive_chan, min_dwell_time_passive_chan);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dwell_times(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	unsigned int max_dwell_time_active_chan;
	unsigned int min_dwell_time_active_chan;
	unsigned int max_dwell_time_passive_chan;
	unsigned int min_dwell_time_passive_chan;
	int statval = 0;

	qcsapi_retval = qcsapi_wifi_get_dwell_times(the_interface,
			&max_dwell_time_active_chan,
			&min_dwell_time_active_chan,
			&max_dwell_time_passive_chan, &min_dwell_time_passive_chan);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d %d %d %d\n",
					max_dwell_time_active_chan,
					min_dwell_time_active_chan,
					max_dwell_time_passive_chan, min_dwell_time_passive_chan);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bgscan_dwell_times(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	unsigned int dwell_time_active_chan;
	unsigned int dwell_time_passive_chan;
	int statval = 0;

	if (argc < 2) {
		print_err(print, "STA Set BGScan Dwell Times requires 2 dwell times\n");
		return 1;
	}

	dwell_time_active_chan = (unsigned int)atoi(argv[0]);
	dwell_time_passive_chan = (unsigned int)atoi(argv[1]);

	qcsapi_retval = qcsapi_wifi_set_bgscan_dwell_times(the_interface,
			dwell_time_active_chan, dwell_time_passive_chan);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bgscan_dwell_times(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	unsigned int dwell_time_active_chan;
	unsigned int dwell_time_passive_chan;
	int statval = 0;

	qcsapi_retval = qcsapi_wifi_get_bgscan_dwell_times(the_interface,
			&dwell_time_active_chan, &dwell_time_passive_chan);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d %d\n",
					dwell_time_active_chan, dwell_time_passive_chan);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_count_associations(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int association_count, *p_association_count = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_association_count = &association_count;
	qcsapi_retval = qcsapi_wifi_get_count_associations(the_interface, p_association_count);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", (unsigned int)association_count);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_associated_device_mac_addr(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr the_mac_addr;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int device_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_associated_device_mac_addr(the_interface,
				device_index, NULL);
	else
		qcsapi_retval = qcsapi_wifi_get_associated_device_mac_addr(the_interface,
				device_index, the_mac_addr);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			dump_mac_addr(p_calling_bundle->caller_output, the_mac_addr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_associated_device_ip_addr(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	unsigned int ip_addr = 0;
	char ip_str[IP_ADDR_STR_LEN + 1];
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int device_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_wifi_get_associated_device_ip_addr(the_interface,
				device_index, NULL);
	else
		qcsapi_retval = qcsapi_wifi_get_associated_device_ip_addr(the_interface,
				device_index, &ip_addr);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			inet_ntop(AF_INET, &ip_addr, ip_str, IP_ADDR_STR_LEN);
			print_out(print, "%s\n", ip_str);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_link_quality(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int link_quality, *p_link_quality = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_link_quality = &link_quality;
	qcsapi_retval = qcsapi_wifi_get_link_quality(the_interface, association_index,
			p_link_quality);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", link_quality);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rssi_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int rssi, *p_rssi = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_rssi = &rssi;
	qcsapi_retval = qcsapi_wifi_get_rssi_per_association(the_interface, association_index,
			p_rssi);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", rssi);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rssi_in_dbm_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int rssi, *p_rssi = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_rssi = &rssi;
	}

	qcsapi_retval = qcsapi_wifi_get_rssi_in_dbm_per_association(the_interface,
			association_index, p_rssi);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", rssi);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int rssi = 0, *p_rssi = NULL;
	int rssi_min = 0, *p_rssi_min = NULL;
	int rssi_max = 0, *p_rssi_max = NULL;
	uint32_t seq_num = 0, *p_seq_num = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_rssi = &rssi;
		p_rssi_min = &rssi_min;
		p_rssi_max = &rssi_max;
		p_seq_num = &seq_num;
	}

	qcsapi_retval = qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association(the_interface,
			association_index, p_rssi_min, p_rssi_max, p_rssi, p_seq_num);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u: rssi %d min %d max %d\n", seq_num, rssi, rssi_min,
					rssi_max);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_meas_rssi_in_dbm_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int rssi = 0, *p_rssi = NULL;
	uint32_t seq_num = 0, *p_seq_num = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_rssi = &rssi;
		p_seq_num = &seq_num;
	}

	qcsapi_retval = qcsapi_wifi_get_meas_rssi_in_dbm_per_association(the_interface,
			association_index, p_rssi, p_seq_num);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u: rssi %d\n", seq_num, rssi);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_rssi_meas_period(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi\n");
		print_err(print, "Usage: call_qcsapi set_rssi_meas_period wifi0 <period>\n"
				" Unit of period: second. period=0 is to disable RSSI measurement\n");
		statval = 1;
	} else {
		int period = atoi(argv[0]);
		int qcsapi_retval;
		const char *interface = p_calling_bundle->caller_interface;

		if (period < 0) {
			print_out(print, "RSSI measurement peariod can't be negative: period = %d\n", period);
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_wifi_set_rssi_meas_period(interface, period);
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rssi_meas_period(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int period = 0;
	qcsapi_unsigned_int *p_period = NULL;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_period = &period;

	qcsapi_retval = qcsapi_wifi_get_rssi_meas_period(interface, p_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", period);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_snr_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int snr, *p_snr = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_snr = &snr;
	qcsapi_retval = qcsapi_wifi_get_snr_per_association(the_interface, association_index,
			p_snr);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", snr);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_hw_noise_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int hw_noise;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	qcsapi_retval = qcsapi_wifi_get_hw_noise_per_association(the_interface, association_index,
			&hw_noise);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d.%d\n", hw_noise / 10, abs(hw_noise % 10));
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rx_bytes_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	u_int64_t rx_bytes, *p_rx_bytes = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_rx_bytes = &rx_bytes;
	qcsapi_retval = qcsapi_wifi_get_rx_bytes_per_association(the_interface, association_index,
			p_rx_bytes);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%llu\n", rx_bytes);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_bytes_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	u_int64_t tx_bytes, *p_tx_bytes = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_bytes = &tx_bytes;
	qcsapi_retval = qcsapi_wifi_get_tx_bytes_per_association(the_interface, association_index,
			p_tx_bytes);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%llu\n", tx_bytes);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rx_packets_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int rx_packets, *p_rx_packets = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_rx_packets = &rx_packets;

	qcsapi_retval = qcsapi_wifi_get_rx_packets_per_association(the_interface, association_index,
			p_rx_packets);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", rx_packets);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_packets_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int tx_packets, *p_tx_packets = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_packets = &tx_packets;
	qcsapi_retval = qcsapi_wifi_get_tx_packets_per_association(the_interface, association_index,
			p_tx_packets);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_packets);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_err_packets_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int tx_err_packets, *p_tx_err_packets = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_err_packets = &tx_err_packets;

	qcsapi_retval = qcsapi_wifi_get_tx_err_packets_per_association(the_interface,
			association_index, p_tx_err_packets);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_err_packets);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_allretries_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int tx_allretries = 0, *p_tx_allretries = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_allretries = &tx_allretries;

	qcsapi_retval = qcsapi_wifi_get_tx_allretries_per_association(the_interface,
			association_index, p_tx_allretries);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_allretries);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_exceed_retry_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int tx_exceed_retry = 0, *p_tx_exceed_retry = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_exceed_retry = &tx_exceed_retry;

	qcsapi_retval = qcsapi_wifi_get_tx_exceed_retry_per_association(the_interface,
			association_index, p_tx_exceed_retry);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_exceed_retry);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_retried_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int tx_retried = 0, *p_tx_retried = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_retried = &tx_retried;

	qcsapi_retval = qcsapi_wifi_get_tx_retried_per_association(the_interface, association_index,
			p_tx_retried);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_retried);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_retried_percent_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int tx_retried_percent = 0, *p_tx_retried_percent = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_retried_percent = &tx_retried_percent;

	qcsapi_retval = qcsapi_wifi_get_tx_retried_percent_per_association(the_interface,
			association_index, p_tx_retried_percent);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_retried_percent);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bw_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_unsigned_int bw, *p_bw = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_bw = &bw;
	qcsapi_retval = qcsapi_wifi_get_bw_per_association(the_interface, association_index, p_bw);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", bw);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_phy_rate_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int tx_rate, *p_tx_rate = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_tx_rate = &tx_rate;
	qcsapi_retval = qcsapi_wifi_get_tx_phy_rate_per_association(the_interface,
			association_index, p_tx_rate);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", tx_rate);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rx_phy_rate_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int rx_rate, *p_rx_rate = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_rx_rate = &rx_rate;
	qcsapi_retval = qcsapi_wifi_get_rx_phy_rate_per_association(the_interface,
			association_index, p_rx_rate);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", rx_rate);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_mcs_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int tx_mcs;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	qcsapi_retval = qcsapi_wifi_get_tx_mcs_per_association(the_interface,
			association_index, &tx_mcs);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", tx_mcs);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rx_mcs_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int rx_mcs;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	qcsapi_retval = qcsapi_wifi_get_rx_mcs_per_association(the_interface,
			association_index, &rx_mcs);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", rx_mcs);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_achievable_tx_phy_rate_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int achievable_tx_rate, *p_achievable_tx_rate = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_achievable_tx_rate = &achievable_tx_rate;

	qcsapi_retval = qcsapi_wifi_get_achievable_tx_phy_rate_per_association(the_interface,
			association_index, p_achievable_tx_rate);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", achievable_tx_rate);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_achievable_rx_phy_rate_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int achievable_rx_rate, *p_achievable_rx_rate = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_achievable_rx_rate = &achievable_rx_rate;

	qcsapi_retval = qcsapi_wifi_get_achievable_rx_phy_rate_per_association(the_interface,
			association_index, p_achievable_rx_rate);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", achievable_rx_rate);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_auth_enc_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_unsigned_int auth_enc;
	uint8_t *casted_ptr = (uint8_t *) & auth_enc;

	qcsapi_retval = qcsapi_wifi_get_auth_enc_per_association(the_interface, association_index,
			&auth_enc);
	if (qcsapi_retval >= 0) {
		if (casted_ptr[IEEE80211_AUTHDESCR_ALGO_POS] >= ARRAY_SIZE(qcsapi_auth_algo_list) ||
				casted_ptr[IEEE80211_AUTHDESCR_KEYMGMT_POS] >=
				ARRAY_SIZE(qcsapi_auth_keymgmt_list)
				|| casted_ptr[IEEE80211_AUTHDESCR_KEYPROTO_POS] >=
				ARRAY_SIZE(qcsapi_auth_keyproto_list)
				|| casted_ptr[IEEE80211_AUTHDESCR_CIPHER_POS] >=
				ARRAY_SIZE(qcsapi_auth_cipher_list)) {

			print_err(print, "error:unknown auth enc value \"%08X\"\n", auth_enc);
			return 1;
		}

		if (verbose_flag >= 0) {
			if (casted_ptr[IEEE80211_AUTHDESCR_KEYPROTO_POS]) {
				print_out(print, "%s/%s with %s\n",
						qcsapi_auth_keyproto_list[casted_ptr
								[IEEE80211_AUTHDESCR_KEYPROTO_POS]],
						qcsapi_auth_keymgmt_list[casted_ptr
								[IEEE80211_AUTHDESCR_KEYMGMT_POS]],
						qcsapi_auth_cipher_list[casted_ptr
								[IEEE80211_AUTHDESCR_CIPHER_POS]]);
			} else {
				print_out(print, "%s/%s\n",
						qcsapi_auth_algo_list[casted_ptr
								[IEEE80211_AUTHDESCR_ALGO_POS]],
						qcsapi_auth_keymgmt_list[casted_ptr
								[IEEE80211_AUTHDESCR_KEYMGMT_POS]]);
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_vendor_per_association(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_unsigned_int vendor;

	qcsapi_retval = qcsapi_wifi_get_vendor_per_association(the_interface, association_index,
			&vendor);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			switch (vendor) {
			case PEER_VENDOR_QTN:
				print_out(print, "quantenna\n");
				break;
			case PEER_VENDOR_BRCM:
				print_out(print, "broadcom\n");
				break;
			case PEER_VENDOR_ATH:
				print_out(print, "atheros\n");
				break;
			case PEER_VENDOR_RLNK:
				print_out(print, "ralink\n");
				break;
			case PEER_VENDOR_RTK:
				print_out(print, "realtek\n");
				break;
			case PEER_VENDOR_INTEL:
				print_out(print, "intel\n");
				break;
			default:
				print_out(print, "unknown\n");
				break;
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int call_qcsapi_set_rf_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 4) {
		print_err(print, "Not enough parameters in call qcsapi set_rf_chains\n");
		print_err(print, "Usage: call_qcsapi set_rf_chains <tx_chains_5G> <rx_chains_5G>"
				" <tx_chains_2.4G> <rx_chains_2.4G> [ <tx_chains_5G_2> ] [ <rx_chains_5G_2> ]\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		qcsapi_unsigned_int tx_chains_5g = atoi(argv[0]);
		qcsapi_unsigned_int rx_chains_5g = atoi(argv[1]);
		qcsapi_unsigned_int tx_chains_2g = atoi(argv[2]);
		qcsapi_unsigned_int rx_chains_2g = atoi(argv[3]);
		qcsapi_unsigned_int tx_chains_5g_2 = 0;
		qcsapi_unsigned_int rx_chains_5g_2 = 0;

		if (argc >= 6) {
			tx_chains_5g_2 = atoi(argv[4]);
			rx_chains_5g_2 = atoi(argv[5]);
		}

		qcsapi_retval = qcsapi_set_rf_chains(tx_chains_5g, rx_chains_5g,
				tx_chains_2g, rx_chains_2g, tx_chains_5g_2, rx_chains_5g_2);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_get_rf_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int tx_chains_5g;
	qcsapi_unsigned_int rx_chains_5g;
	qcsapi_unsigned_int tx_chains_2g;
	qcsapi_unsigned_int rx_chains_2g;
	qcsapi_unsigned_int tx_chains_5g_2;
	qcsapi_unsigned_int rx_chains_5g_2;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_get_rf_chains(&tx_chains_5g, &rx_chains_5g,
			&tx_chains_2g, &rx_chains_2g, &tx_chains_5g_2, &rx_chains_5g_2);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "5G: %dx%d, 2.4G: %dx%d, 5G_2: %dx%d\n",
					tx_chains_5g, rx_chains_5g, tx_chains_2g,
					rx_chains_2g, tx_chains_5g_2, rx_chains_5g_2);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_tx_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi\n");
		print_err(print, "Usage: call_qcsapi set_tx_chains wifi0 <tx_chains>\n");
		statval = 1;
	} else {
		char *endptr;
		qcsapi_unsigned_int tx_chains = strtoul(argv[0], &endptr, 0);
		int qcsapi_retval;
		const char *interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_tx_chains(interface, tx_chains);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_tx_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int tx_chains;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_tx_chains(interface, &tx_chains);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", tx_chains);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_rx_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi\n");
		print_err(print, "Usage: call_qcsapi set_rx_chains wifi0 <rx_chains>\n");
		statval = 1;
	} else {
		char *endptr;
		qcsapi_unsigned_int rx_chains = strtoul(argv[0], &endptr, 0);
		int qcsapi_retval;
		const char *interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_rx_chains(interface, rx_chains);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rx_chains(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int rx_chains;
	const char *interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_rx_chains(interface, &rx_chains);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", rx_chains);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_eap_reauth_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int eap_reauth_period;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <eap_reauth_period>\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &eap_reauth_period) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_eap_reauth_period(p_calling_bundle->caller_interface,
			eap_reauth_period);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_eap_reauth_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int eap_reauth_period;

	qcsapi_retval = qcsapi_wifi_get_eap_reauth_period(p_calling_bundle->caller_interface,
			&eap_reauth_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", eap_reauth_period);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_eap_reauth_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_eap_reauth_period(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_radius_max_retries(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_max_retries;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <radius_max_retries\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &radius_max_retries) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_radius_max_retries(p_calling_bundle->caller_interface,
			radius_max_retries);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_radius_max_retries(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_max_retries;

	qcsapi_retval = qcsapi_wifi_get_radius_max_retries(p_calling_bundle->caller_interface,
			&radius_max_retries);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", radius_max_retries);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_radius_max_retries(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_radius_max_retries(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_radius_num_failover(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_num_failover;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <radius_num_failover\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &radius_num_failover) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_radius_num_failover(p_calling_bundle->caller_interface,
			radius_num_failover);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_radius_num_failover(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_num_failover;

	qcsapi_retval = qcsapi_wifi_get_radius_num_failover(p_calling_bundle->caller_interface,
			&radius_num_failover);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", radius_num_failover);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_radius_num_failover(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_radius_num_failover(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_radius_timeout(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_timeout;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <radius_timeout\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &radius_timeout) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_radius_timeout(p_calling_bundle->caller_interface,
			radius_timeout);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_radius_timeout(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int radius_timeout;

	qcsapi_retval = qcsapi_wifi_get_radius_timeout(p_calling_bundle->caller_interface,
			&radius_timeout);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", radius_timeout);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_radius_timeout(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_radius_timeout(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_pmk_cache_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int pmk_cache_enable;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <pmk_cache_enable\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &pmk_cache_enable) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_pmk_cache_enable(p_calling_bundle->caller_interface,
			pmk_cache_enable);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_pmk_cache_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int pmk_cache_enable;

	qcsapi_retval = qcsapi_wifi_get_pmk_cache_enable(p_calling_bundle->caller_interface,
			&pmk_cache_enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", pmk_cache_enable);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_set_pmk_cache_lifetime(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int pmk_cache_lifetime;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <pmk_cache_lifetime\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &pmk_cache_lifetime) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_pmk_cache_lifetime(p_calling_bundle->caller_interface,
			pmk_cache_lifetime);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_pmk_cache_lifetime(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int pmk_cache_lifetime;

	qcsapi_retval = qcsapi_wifi_get_pmk_cache_lifetime(p_calling_bundle->caller_interface,
			&pmk_cache_lifetime);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", pmk_cache_lifetime);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_pmk_cache_lifetime(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_pmk_cache_lifetime(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_max_auth_attempts(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int max_auth_attempts;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <max_auth_attempts\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &max_auth_attempts) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_max_auth_attempts(p_calling_bundle->caller_interface,
			max_auth_attempts);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_max_auth_attempts(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int max_auth_attempts;

	qcsapi_retval = qcsapi_wifi_get_max_auth_attempts(p_calling_bundle->caller_interface,
			&max_auth_attempts);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", max_auth_attempts);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_set_lockout_period(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int lockout_period;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <lockout_period\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &lockout_period) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_lockout_period(p_calling_bundle->caller_interface,
			lockout_period);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_lockout_period(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int lockout_period;

	qcsapi_retval = qcsapi_wifi_get_lockout_period(p_calling_bundle->caller_interface,
			&lockout_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", lockout_period);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_lockout_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_lockout_period(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_id_request_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int id_request_period;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <id_request_period\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &id_request_period) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_id_request_period(p_calling_bundle->caller_interface,
			id_request_period);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_id_request_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int id_request_period;

	qcsapi_retval = qcsapi_wifi_get_id_request_period(p_calling_bundle->caller_interface,
			&id_request_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", id_request_period);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_id_request_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_id_request_period(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_set_auth_quiet_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int auth_quiet_period;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <auth_quiet_period\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &auth_quiet_period) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_auth_quiet_period(p_calling_bundle->caller_interface,
			auth_quiet_period);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_auth_quiet_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int auth_quiet_period;

	qcsapi_retval = qcsapi_wifi_get_auth_quiet_period(p_calling_bundle->caller_interface,
			&auth_quiet_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(p_calling_bundle->caller_output, "%u\n", auth_quiet_period);
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_wifi_remove_auth_quiet_period(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_wifi_remove_auth_quiet_period(p_calling_bundle->caller_interface);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_max_mimo(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	string_16 max_mimo;

	qcsapi_retval = qcsapi_wifi_get_max_mimo(the_interface, association_index, max_mimo);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", max_mimo);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_wifi_get_tput_caps(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	int qcsapi_retval;
	struct ieee8011req_sta_tput_caps tput_caps;
	struct ieee80211_ie_vhtcap *ie_vhtcap;
	struct ieee80211_ie_htcap *ie_htcap;

	qcsapi_retval = qcsapi_wifi_get_tput_caps(the_interface, association_index, &tput_caps);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			switch (tput_caps.mode) {
			case IEEE80211_WIFI_MODE_AX:
				print_out(print, "Mode: HE\n");
				/* TODO: Fall through for 5G only as AC is non-standard for 2.4G */
			case IEEE80211_WIFI_MODE_AC:
				if (tput_caps.mode != IEEE80211_WIFI_MODE_AX) {
					print_out(print, "Mode: VHT\n");
				}
				ie_vhtcap = (struct ieee80211_ie_vhtcap *)tput_caps.vhtcap_ie;

				print_out(print, "VHT Capabilities Info: ");
				dump_data_array(print, ie_vhtcap->vht_cap,
						sizeof(ie_vhtcap->vht_cap), 16, ' ');

				print_out(print, "Supported VHT MCS & NSS Set: ");
				dump_data_array(print, ie_vhtcap->vht_mcs_nss_set,
						sizeof(ie_vhtcap->vht_mcs_nss_set), 16, ' ');
				/* Fall through */
			case IEEE80211_WIFI_MODE_NA:
				/* Fall through */
			case IEEE80211_WIFI_MODE_NG:
				if (tput_caps.mode != IEEE80211_WIFI_MODE_AC &&
						tput_caps.mode != IEEE80211_WIFI_MODE_AX) {
					print_out(print, "Mode: HT\n");
				}
				ie_htcap = (struct ieee80211_ie_htcap *)tput_caps.htcap_ie;

				print_out(print, "HT Capabilities Info: ");
				dump_data_array(print, ie_htcap->hc_cap,
						sizeof(ie_htcap->hc_cap), 16, ' ');

				print_out(print, "A-MPDU Parameters: %02X\n", ie_htcap->hc_ampdu);

				print_out(print, "Supported MCS Set: ");
				dump_data_array(print, ie_htcap->hc_mcsset,
						sizeof(ie_htcap->hc_mcsset), 16, ' ');

				print_out(print, "HT Extended Capabilities: ");
				dump_data_array(print, ie_htcap->hc_extcap,
						sizeof(ie_htcap->hc_extcap), 16, ' ');

				print_out(print, "Transmit Beamforming Capabilities: ");
				dump_data_array(print, ie_htcap->hc_txbf,
						sizeof(ie_htcap->hc_txbf), 16, ' ');

				print_out(print, "ASEL Capabilities: %02X\n", ie_htcap->hc_antenna);
				break;
			default:
				print_out(print, "Mode: non HT\n");
				break;
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_wifi_get_connection_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;
	int qcsapi_retval;
	qcsapi_unsigned_int connection_mode;

	qcsapi_retval = qcsapi_wifi_get_connection_mode(the_interface,
			association_index, &connection_mode);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (connection_mode >= IEEE80211_WIFI_MODE_MAX) {
				connection_mode = IEEE80211_WIFI_MODE_NONE;
			}
			print_out(print, "%s\n", qcsapi_wifi_modes_strings[connection_mode]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_wifi_get_node_counter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int node_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_counter_type counter_type = QCSAPI_NOSUCH_COUNTER;
	int local_remote_flag = QCSAPI_LOCAL_NODE;
	uint64_t counter_value = 0;
	uint64_t *p_counter_value = &counter_value;

	if (argc < 1) {
		print_err(print, "Get Counter Per Association: type of counter required\n");
		return 1;
	}

	if (name_to_counter_enum(argv[0], &counter_type) == 0) {
		print_err(print, "No such counter type %s\n", argv[0]);
		return 1;
	}

	if (argc > 1) {
		if (parse_local_remote_flag(print, argv[1], &local_remote_flag) < 0) {
			return 1;
		}
	}

	if (argc > 2 && strcmp(argv[2], "NULL") == 0) {
		p_counter_value = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_node_counter(the_interface,
			node_index, counter_type, local_remote_flag, p_counter_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%llu\n", counter_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int parse_measure_request_param(qcsapi_measure_request_param *param,
		qcsapi_output *print, qcsapi_per_assoc_param type, int argc, char *argv[])
{
	int i;
	int ret;
	int qualified;
	int pre_len;
	int bad_format = 0;
	int mac[6];

	ret = 0;
	qualified = 0;
	switch (type) {
	case QCSAPI_NODE_MEAS_BASIC:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->basic.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "off=", (pre_len = strlen("off="))) ==
								0) && (strlen(argv[i]) > pre_len)) {
					param->basic.offset = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->basic.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else {
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					bad_format = 1;
					break;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "basic measurement param:\n"
						"<du=duration> [ch=channel] "
						"[off=offset to start measuremnt]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_CCA:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->cca.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "off=", (pre_len = strlen("off="))) ==
								0) && (strlen(argv[i]) > pre_len)) {
					param->cca.offset = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->cca.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else {
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					bad_format = 1;
					break;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "cca measurement param:\n"
						"<du=duration> [ch=channel] "
						"[off=offset to start measuremnt]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_RPI:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->rpi.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "off=", (pre_len = strlen("off="))) ==
								0) && (strlen(argv[i]) > pre_len)) {
					param->rpi.offset = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->rpi.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else {
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					bad_format = 1;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "rpi measurement param:\n"
						"<du=duration> [ch=channel] "
						"[off=offset to start measuremnt]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_CHAN_LOAD:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->chan_load.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "op=", (pre_len = strlen("op="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->chan_load.op_class = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->chan_load.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else {
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					bad_format = 1;
					break;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "channel load measurement param:\n"
						"<du=duration> [ch=channel] "
						"[op=operating class]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_NOISE_HIS:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->noise_his.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "op=", (pre_len = strlen("op="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->noise_his.op_class = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->noise_his.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else {
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					bad_format = 1;
					break;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "noise histogram measurement param:\n"
						"<du=duration> [ch=channel] "
						"[op=operating class]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_BEACON:
		{
			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->beacon.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "op=", (pre_len = strlen("op="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->beacon.op_class = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->beacon.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "mode=", (pre_len = strlen("mode=")))
								== 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->beacon.mode = atoi(argv[i] + pre_len);
				} else {
					bad_format = 1;
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					break;
				}
			}

			if (!qualified || bad_format) {
				print_out(print, "beacon measurement param:\n"
						"<du=duration> [ch=channel] "
						"[mode=beacon measurement mode][op=operating class]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_FRAME:
		{
			int cnt;

			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "ch=", (pre_len = strlen("ch="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->frame.channel = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "op=", (pre_len = strlen("op="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->frame.op_class = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->frame.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "type=", (pre_len = strlen("type=")))
								== 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->frame.type = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "mac=", (pre_len = strlen("mac="))) ==
								0) && (strlen(argv[i]) > pre_len)) {
					if (sscanf(argv[i] + pre_len, "%x:%x:%x:%x:%x:%x", &mac[0],
									&mac[1], &mac[2], &mac[3],
									&mac[4], &mac[5]) != 6) {
						bad_format = 1;
						break;
					}
					for (cnt = 0; cnt < 6; cnt++)
						param->frame.mac_address[cnt] = (uint8_t) mac[cnt];
				} else {
					bad_format = 1;
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					break;
				}
			}

			if ((qualified < 2) || bad_format) {
				print_out(print, "frame measurement param:\n"
						"<du=duration>\n"
						"<type=measurement frame type, only 1 supported currently>\n"
						"[ch=channel] [op=operating class] [mac=specified mac address]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_TRAN_STREAM_CAT:
		{
			int cnt;

			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "tid=", (pre_len = strlen("tid="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->tran_stream_cat.tid = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "bin0=", (pre_len = strlen("bin0=")))
								== 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->tran_stream_cat.bin0 = atoi(argv[i] + pre_len);
				} else if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0)
						&& (strlen(argv[i]) > pre_len)) {
					param->tran_stream_cat.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "peer_sta=", (pre_len = strlen("peer_sta="))) == 0) && (strlen(argv[i]) > pre_len)) {
					if (sscanf(argv[i] + pre_len, "%x:%x:%x:%x:%x:%x", &mac[0],
									&mac[1],
									&mac[2],
									&mac[3],
									&mac[4], &mac[5]) != 6) {
						bad_format = 1;
						break;
					}
					for (cnt = 0; cnt < 6; cnt++)
						param->tran_stream_cat.peer_sta[cnt] =
								(uint8_t) mac[cnt];
				} else {
					bad_format = 1;
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					break;
				}
			}

			if ((qualified < 2) || bad_format) {
				print_out(print, "transmit stream category measurement param:\n"
						"<du=duration>\n"
						"<tid=traffic id>\n"
						"[peer_sta=peer station mac address] [bin0=bin0 range]\n");
				ret = 1;
			}
			break;
		}
	case QCSAPI_NODE_MEAS_MULTICAST_DIAG:
		{
			int cnt;

			for (i = 0; i < argc; i++) {
				if ((strncmp(argv[i], "du=", (pre_len = strlen("du="))) == 0) &&
						(strlen(argv[i]) > pre_len)) {
					param->multicast_diag.duration = atoi(argv[i] + pre_len);
					qualified++;
				} else if ((strncmp(argv[i], "group_mac=", (pre_len = strlen("group_mac="))) == 0) && (strlen(argv[i]) > pre_len)) {
					if (sscanf(argv[i] + pre_len, "%x:%x:%x:%x:%x:%x", &mac[0],
									&mac[1],
									&mac[2],
									&mac[3],
									&mac[4], &mac[5]) != 6) {
						bad_format = 1;
						break;
					}
					for (cnt = 0; cnt < 6; cnt++)
						param->multicast_diag.group_mac[cnt] =
								(uint8_t) mac[cnt];
					qualified++;
				} else {
					bad_format = 1;
					print_err(print, "error:unknown parameter \"%s\"\n",
							argv[i]);
					break;
				}
			}

			if ((qualified < 2) || bad_format) {
				print_out(print, "multicast diagnostic measurement param:\n"
						"<du=duration>\n"
						"<group_mac=group mac address>\n");
				ret = 1;
			}
			break;
		}
	default:
		break;
	}

	return ret;
}

static int
call_qcsapi_wifi_get_node_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int node_index = p_calling_bundle->caller_generic_parameter.index;
	qcsapi_per_assoc_param param_type = QCSAPI_NO_SUCH_PER_ASSOC_PARAM;
	int local_remote_flag = QCSAPI_LOCAL_NODE;
	string_128 input_param_str;
	qcsapi_measure_request_param *request_param;
	qcsapi_measure_report_result report_result;
	int *p_param_value;

	if (argc < 1) {
		print_err(print, "Get Parameter Per Association: type of parameter required\n");
		return 1;
	}

	if (name_to_per_assoc_parameter(argv[0], &param_type) == 0) {
		print_err(print, "No such parameter type %s\n", argv[0]);
		return 1;
	}

	if (argc > 1) {
		if (parse_local_remote_flag(print, argv[1], &local_remote_flag) < 0) {
			return 1;
		}
	}

	request_param = (qcsapi_measure_request_param *) input_param_str;
	if (argc >= 2) {
		argc -= 2;
		argv += 2;
		memset(request_param, 0, sizeof(*request_param));
		if (parse_measure_request_param(request_param, print, param_type, argc, argv)) {
			return 1;
		}
	}

	memset(&report_result, 0, sizeof(report_result));
	qcsapi_retval = qcsapi_wifi_get_node_param(the_interface,
			node_index, param_type, local_remote_flag, input_param_str, &report_result);
	p_param_value = report_result.common;
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			switch (param_type) {
			case QCSAPI_SOC_MAC_ADDR:
				{
					qcsapi_mac_addr the_mac_addr;
					memcpy(the_mac_addr, p_param_value,
							sizeof(qcsapi_mac_addr));
					dump_mac_addr(print, the_mac_addr);
					break;
				}
			case QCSAPI_SOC_IP_ADDR:
				print_out(print, "%d.%d.%d.%d\n",
						((char *)p_param_value)[0],
						((char *)p_param_value)[1],
						((char *)p_param_value)[2],
						((char *)p_param_value)[3]);
				break;
			case QCSAPI_NODE_MEAS_RPI:
				dump_data_array(print, report_result.rpi, 8, 10, ' ');
				break;
			case QCSAPI_NODE_TPC_REP:
				print_out(print, "link margin = %d db\n",
						report_result.tpc.link_margin);
				print_out(print, "transmit power = %d dbm\n",
						report_result.tpc.tx_power);
				break;
			case QCSAPI_NODE_MEAS_NOISE_HIS:
				{
					int i;

					print_out(print, "anntenna id = %d\n",
							report_result.noise_histogram.antenna_id);
					print_out(print, "anpi = %d\n",
							(0 - report_result.noise_histogram.anpi));
					for (i = 0; i < 11; i++)
						print_out(print, "ipi%d:%d\n", i,
								report_result.noise_histogram.
								ipi[i]);
					break;
				}
			case QCSAPI_NODE_MEAS_BEACON:
				{
					qcsapi_mac_addr the_mac_addr;

					print_out(print, "report frame info = %x\n",
							report_result.beacon.rep_frame_info);
					print_out(print, "rcpi = %d\n", report_result.beacon.rcpi);
					print_out(print, "rsni = %d\n", report_result.beacon.rsni);
					print_out(print, "mac address:");
					memcpy(the_mac_addr, report_result.beacon.bssid,
							sizeof(qcsapi_mac_addr));
					dump_mac_addr(print, the_mac_addr);
					print_out(print, "antenna id = %d\n",
							report_result.beacon.antenna_id);
					print_out(print, "parent_tsf = %d\n",
							report_result.beacon.parent_tsf);
					break;
				}
			case QCSAPI_NODE_MEAS_FRAME:
				{
					qcsapi_mac_addr the_mac_addr;

					if (report_result.frame.sub_ele_report == 0) {
						print_out(print, "no measurement result\n");
					} else {
						print_out(print, "TA address:");
						memcpy(the_mac_addr, report_result.frame.ta,
								sizeof(qcsapi_mac_addr));
						dump_mac_addr(print, the_mac_addr);
						print_out(print, "BSSID:");
						memcpy(the_mac_addr, report_result.frame.bssid,
								sizeof(qcsapi_mac_addr));
						dump_mac_addr(print, the_mac_addr);
						print_out(print, "phy_type = %d\n",
								report_result.frame.phy_type);
						print_out(print, "average RCPI = %d\n",
								report_result.frame.avg_rcpi);
						print_out(print, "last RSNI = %d\n",
								report_result.frame.last_rsni);
						print_out(print, "last RCPI = %d\n",
								report_result.frame.last_rcpi);
						print_out(print, "antenna id = %d\n",
								report_result.frame.antenna_id);
						print_out(print, "Frame count = %d\n",
								report_result.frame.frame_count);
					}
					break;
				}
			case QCSAPI_NODE_MEAS_TRAN_STREAM_CAT:
				{
					int i;

					print_out(print, "reason = %d\n",
							report_result.tran_stream_cat.reason);
					print_out(print, "transmitted MSDU count = %d\n",
							report_result.tran_stream_cat.
							tran_msdu_cnt);
					print_out(print, "MSDU Discarded Count = %d\n",
							report_result.tran_stream_cat.
							msdu_discard_cnt);
					print_out(print, "MSDU Failed Count = %d\n",
							report_result.tran_stream_cat.
							msdu_fail_cnt);
					print_out(print, "MSDU Multiple retry Count = %d\n",
							report_result.tran_stream_cat.
							msdu_mul_retry_cnt);
					print_out(print, "MSDU Qos CF-Polls Lost Count = %d\n",
							report_result.tran_stream_cat.qos_lost_cnt);
					print_out(print, "Average Queue Delay = %d\n",
							report_result.tran_stream_cat.
							avg_queue_delay);
					print_out(print, "Average Transmit Delay = %d\n",
							report_result.tran_stream_cat.
							avg_tran_delay);
					print_out(print, "Bin0 range = %d\n",
							report_result.tran_stream_cat.bin0_range);
					for (i = 0; i < 6; i++)
						print_out(print, "Bin%d = %d\n", i,
								report_result.tran_stream_cat.
								bins[i]);
					break;
				}
			case QCSAPI_NODE_MEAS_MULTICAST_DIAG:
				print_out(print, "reason = %d\n",
						report_result.multicast_diag.reason);
				print_out(print, "Multicast Received MSDU Count = %d\n",
						report_result.multicast_diag.mul_rec_msdu_cnt);
				print_out(print, "First Sequence Number = %d\n",
						report_result.multicast_diag.first_seq_num);
				print_out(print, "Last Sequence Number = %d\n",
						report_result.multicast_diag.last_seq_num);
				print_out(print, "Multicast Rate = %d\n",
						report_result.multicast_diag.mul_rate);
				break;
			case QCSAPI_NODE_LINK_MEASURE:
				print_out(print, "transmit power = %d\n",
						report_result.link_measure.tpc_report.tx_power);
				print_out(print, "link margin = %d\n",
						report_result.link_measure.tpc_report.link_margin);
				print_out(print, "receive antenna id = %d\n",
						report_result.link_measure.recv_antenna_id);
				print_out(print, "transmit antenna id = %d\n",
						report_result.link_measure.tran_antenna_id);
				print_out(print, "RCPI = %d\n", report_result.link_measure.rcpi);
				print_out(print, "RSNI = %d\n", report_result.link_measure.rsni);
				break;
			case QCSAPI_NODE_NEIGHBOR_REP:
				{
					uint8_t i;
					qcsapi_mac_addr the_mac_addr;

					if (report_result.neighbor_report.item_num == 0) {
						print_out(print, "no neighbor report\n");
					} else {
						for (i = 0; i < report_result.neighbor_report.
								item_num; i++) {
							print_out(print, "bssid=");
							memcpy(the_mac_addr,
									report_result.
									neighbor_report.items[i].
									bssid,
									sizeof(qcsapi_mac_addr));
							dump_mac_addr(print, the_mac_addr);
							print_out(print, "BSSID Info = 0x%x\n",
									report_result.
									neighbor_report.items[i].
									bssid_info);
							print_out(print, "operating class = %d\n",
									report_result.
									neighbor_report.items[i].
									operating_class);
							print_out(print, "channel = %d\n",
									report_result.
									neighbor_report.items[i].
									channel);
							print_out(print, "phy_type = %d\n",
									report_result.
									neighbor_report.items[i].
									phy_type);
						}
					}
				}
				break;
			case QCSAPI_NODE_SGI_CAPS:
				/*
				 * The variable 'report_result.common[0]' returns the SGI Capability
				 * of the node.
				 * If 'report_result.common[0]' is 0, the station is not SGI
				 * capable. If 'report_result.common[0]' is non-zero, the station
				 * is SGI capable.
				 * The following bitmap represents SGI capabilities in different
				 * Bandwidths
				 * - if bit 0 is set, the Station is SGI capable in 20MHz
				 * - if bit 1 is set, the Station is SGI capable in 40MHz
				 * - if bit 2 is set, the Station is SGI capable in 80MHz
				 * - if bit 3 is set, the Station is SGI capable in 160MHz
				 */
				print_out(print, "sgi_caps = 0x%x\n", report_result.common[0]);
				break;
			default:
				print_out(print, "%d\n", *p_param_value);
				break;
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define CALL_QCSAPI_NODE_STATS_LABEL_LEN			20

#define CALL_QCSAPI_NODE_STATS_PRINT(_name, _type, _val)	\
	print_out(print, "%-*s%s%" _type "\n",			\
		CALL_QCSAPI_NODE_STATS_LABEL_LEN, _name, ": ", _val);

static int
call_qcsapi_wifi_get_node_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int node_index = p_calling_bundle->caller_generic_parameter.index;
	int local_remote_flag = QCSAPI_LOCAL_NODE;
	struct qcsapi_node_stats node_stats, *p_node_stats = &node_stats;

	memset(&node_stats, 0, sizeof(node_stats));

	if (argc > 0) {
		if (parse_local_remote_flag(print, argv[0], &local_remote_flag) < 0) {
			return 1;
		}
	}

	if (argc > 1 && strcmp(argv[1], "NULL") == 0) {
		p_node_stats = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_node_stats(the_interface,
			node_index, local_remote_flag, p_node_stats);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (node_stats.snr < 0) {
				node_stats.snr = (node_stats.snr -
						QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
						QCSAPI_RSSI_OR_SNR_FACTOR;
			} else {
				node_stats.snr = (node_stats.snr +
						QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
						QCSAPI_RSSI_OR_SNR_FACTOR;
			}
			node_stats.snr = (0 - node_stats.snr);

			if (node_stats.rssi < 0) {
				node_stats.rssi = 0;
			} else {
				node_stats.rssi = (qcsapi_unsigned_int) (node_stats.rssi +
						QCSAPI_RSSI_OR_SNR_NZERO_CORRECT_VALUE) /
						QCSAPI_RSSI_OR_SNR_FACTOR;
			}

			CALL_QCSAPI_NODE_STATS_PRINT("tx_bytes", "llu", node_stats.tx_bytes);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_pkts", "lu", node_stats.tx_pkts);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_discard", "lu", node_stats.tx_discard);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_be", "lu",
					node_stats.tx_wifi_sent[WMM_AC_BE]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_bk", "lu",
					node_stats.tx_wifi_sent[WMM_AC_BK]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_vi", "lu",
					node_stats.tx_wifi_sent[WMM_AC_VI]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_sent_vo", "lu",
					node_stats.tx_wifi_sent[WMM_AC_VO]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_be", "lu",
					node_stats.tx_wifi_drop[WMM_AC_BE]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_bk", "lu",
					node_stats.tx_wifi_drop[WMM_AC_BK]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_vi", "lu",
					node_stats.tx_wifi_drop[WMM_AC_VI]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_wifi_drop_vo", "lu",
					node_stats.tx_wifi_drop[WMM_AC_VO]);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_err", "lu", node_stats.tx_err);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_unicast", "lu", node_stats.tx_unicast);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_multicast", "lu", node_stats.tx_multicast);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_broadcast", "lu", node_stats.tx_broadcast);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_phy_rate", "lu", node_stats.tx_phy_rate);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_mgmt", "lu", node_stats.tx_mgmt);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_mcs_index", "lu", node_stats.tx_mcs);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_nss", "lu", node_stats.tx_nss);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_bw", "lu", node_stats.tx_bw);
			CALL_QCSAPI_NODE_STATS_PRINT("tx_sgi", "lu", node_stats.tx_sgi);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_bytes", "llu", node_stats.rx_bytes);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_pkts", "lu", node_stats.rx_pkts);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_discard", "lu", node_stats.rx_discard);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_err", "lu", node_stats.rx_err);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_unicast", "lu", node_stats.rx_unicast);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_multicast", "lu", node_stats.rx_multicast);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_broadcast", "lu", node_stats.rx_broadcast);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_phy_rate", "lu", node_stats.rx_phy_rate);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_mgmt", "lu", node_stats.rx_mgmt);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_ctrl", "lu", node_stats.rx_ctrl);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_mcs_index", "lu", node_stats.rx_mcs);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_nss", "lu", node_stats.rx_nss);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_bw", "lu", node_stats.rx_bw);
			CALL_QCSAPI_NODE_STATS_PRINT("rx_sgi", "lu", node_stats.rx_sgi);
			print_out(print, "%-*s: %d.%d\n",
					CALL_QCSAPI_NODE_STATS_LABEL_LEN,
					"hw_noise",
					(node_stats.hw_noise / 10), abs(node_stats.hw_noise % 10));
			CALL_QCSAPI_NODE_STATS_PRINT("snr", "d", node_stats.snr);
			CALL_QCSAPI_NODE_STATS_PRINT("rssi", "d", node_stats.rssi);
			CALL_QCSAPI_NODE_STATS_PRINT("bw", "d", node_stats.bw);
			print_out(print, "%-*s: %02x:%02x:%02x:%02x:%02x:%02x\n",
					CALL_QCSAPI_NODE_STATS_LABEL_LEN,
					"mac_addr", MAC_ADDR_ARG(node_stats.mac_addr));
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_node_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtn_nis_node_list node_list;
	int i;

	memset(&node_list, 0, sizeof(node_list));

	if (argc != 0) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface>\n");
		return 1;
	}

	retval = qcsapi_wifi_get_node_list(the_interface, &node_list);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	if (node_list.cnt > ARRAY_SIZE(node_list.node)) {
		report_qcsapi_error(p_calling_bundle, -E2BIG);
		return 1;
	}

	for (i = 0; i < node_list.cnt; i++)
		print_out(print, "%3u " MACSTR "\n",
				node_list.node[i].idx, MAC2STR(node_list.node[i].mac_addr));

	return 0;
}

static int
local_node_infoset_print(call_qcsapi_bundle *p_calling_bundle, const uint16_t set_id,
		struct qtn_nis_set *nis)
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int i;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qtn_nis_label) <= ARRAY_SIZE(nis->val));

	print_out(print, "%-*s: " MACSTR "\n",
			QTN_NIS_LABEL_LEN, "MAC address", MAC2STR(nis->mac_addr));
	print_out(print, "%-*s: %u\n", QTN_NIS_LABEL_LEN, "Node index", nis->node_index);

	for (i = 0; i < ARRAY_SIZE(nis->val); i++)
		if (QTN_NIS_IS_SET(nis, i))
			print_out(print, "%-*s: %u\n",
					QTN_NIS_LABEL_LEN, qtn_nis_label[set_id][i], nis->val[i]);

	return 0;
}

static int
call_qcsapi_wifi_get_node_infoset(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtn_nis_set nis;
	const char *usage = "<WiFi interface> {<MAC addr> | <node index>} <set id>";
	qcsapi_mac_addr mac_addr;
	uint32_t node_index = 0;
	uint16_t set_id;

	if (argc != 2) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &node_index) < 0 || node_index == 0) {
		retval = parse_mac_addr(argv[0], mac_addr);
		if (retval < 0) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
	}

	if (safe_atou16(argv[1], &set_id, print, 0, QTN_NIS_SET_ID_MAX - 1) <= 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	retval = qcsapi_wifi_get_node_infoset(the_interface, node_index, mac_addr, set_id, &nis);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	local_node_infoset_print(p_calling_bundle, set_id, &nis);

	return retval;
}

static char *local_node_infoset_get_mfp_desc(uint32_t val)
{
	if (val & RSN_CAP_MFP_REQ)
		return "required";
	else if (val & RSN_CAP_MFP_CAP)
		return "capable";

	return "disabled";
}

static char *local_node_infoset_print_get_cipher_desc(qcsapi_output *print, uint32_t val)
{
	switch (val) {
	case IEEE80211_CIPHER_WEP:
		return "WEP";
	case IEEE80211_CIPHER_TKIP:
		return "TKIP";
	case IEEE80211_CIPHER_AES_OCB:
		return "AES-OCB";
	case IEEE80211_CIPHER_AES_CCM:
		return "AES-CCM";
	case IEEE80211_CIPHER_AES_CMAC:
		return "AES-CMAC";
	case IEEE80211_CIPHER_CKIP:
		return "CKIP";
	case IEEE80211_CIPHER_AES_CCM_256:
		return "AES-CCM-256";
	case IEEE80211_CIPHER_AES_GCM:
		return "AES-GCM";
	case IEEE80211_CIPHER_AES_GCM_256:
		return "AES-GCM-256";
	default:
		return "unknown";
	}
}

/*
 * These descriptive names are generally taken from hostapd config.c
 */
static char *local_node_infoset_get_keymgmt_type(uint32_t val)
{
	switch (val) {
	case RSN_ASE_NONE:
		return "none";
	case RSN_ASE_8021X_UNSPEC:
		return "WPA-EAP";
	case RSN_ASE_8021X_PSK:
		return "WPA-PSK";
	case RSN_ASE_FT_8021X:
		return "FT-EAP";
	case RSN_ASE_FT_PSK:
		return "FT-PSK";
	case RSN_ASE_8021X_SHA256:
		return "WPA-EAP-SHA256";
	case RSN_ASE_8021X_PSK_SHA256:
		return "WPA-PSK-SHA256";
	case RSN_SAE:
		return "SAE";
	case RSN_OWE:
		return "OWE";
	case RSN_ASE_DPP:
		return "DPP";
	default:
		return "unknown";
	}
}

static void
local_node_infoset_print_type(qcsapi_output *print, enum qtn_nis_val_type_s val_type, uint32_t val)
{
	switch (val_type) {
	case QTN_NIS_VAL_UNSIGNED:
		print_out(print, "%u\n", val);
		break;
	case QTN_NIS_VAL_SIGNED:
		print_out(print, "%d\n", val);
		break;
	case QTN_NIS_VAL_RSN_CAPS:
		if (val & QTN_NIS_S4_RSN_DISABLED)
			print_out(print, "0x%04x disabled\n");
		else
			print_out(print, "0x%04x MFP:%s\n",
					val, local_node_infoset_get_mfp_desc(val));
		break;
	case QTN_NIS_VAL_RSN_UCASTCIPHER:
		print_out(print, "0x%04x %s\n",
			val, local_node_infoset_print_get_cipher_desc(print, val));
		break;
	case QTN_NIS_VAL_RSN_MCASTCIPHER:
		print_out(print, "0x%04x %s\n",
			val, local_node_infoset_print_get_cipher_desc(print, val));
		break;
	case QTN_NIS_VAL_RSN_KEYMGMT:
		print_out(print, "0x%04x %s\n", val, local_node_infoset_get_keymgmt_type(val));
		break;
	}

	return;
}

static int
local_node_infoset_all_print(call_qcsapi_bundle *p_calling_bundle, const uint16_t set_id,
		struct qtn_nis_all_set *nis)
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtn_nis_all_node *node;
	uint16_t node_idx = 0;
	int node_num;
	int fld;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qtn_nis_all_label) <= ARRAY_SIZE(nis->node[0].val));

	for (node_num = 0; node_num < nis->node_cnt; node_num++) {
		node = &nis->node[node_num];
		node_idx = node->node_index;
		for (fld = 0; fld < ARRAY_SIZE(node->val); fld++) {
			if (QTN_NIS_IS_SET(node, fld)) {
				print_out(print, MACSTR " %4u %-*s: ",
						MAC2STR(node->mac_addr), node_idx,
						QTN_NIS_LABEL_LEN,
						qtn_nis_all_label[set_id][fld].label);
				local_node_infoset_print_type(print,
						qtn_nis_all_label[set_id][fld].type,
						node->val[fld]);
			}
		}
	}

	/* If the report contains max nodes, there may be more to come. */
	if (nis->node_cnt == ARRAY_SIZE(nis->node))
		return node_idx + 1;

	return 0;
}

static int
local_node_infoset_all_print_all_nodes(call_qcsapi_bundle *p_calling_bundle, const uint16_t set_id,
		struct qtn_nis_all_set *nis)
{
	const char *the_interface = p_calling_bundle->caller_interface;
	int retval;
	int first_node_index = 0;

	do {
		retval = qcsapi_wifi_get_node_infoset_all(the_interface, first_node_index,
				set_id, 0, nis);
		if (retval < 0) {
			report_qcsapi_error(p_calling_bundle, retval);
			return 1;
		}
		first_node_index = local_node_infoset_all_print(p_calling_bundle, set_id, nis);
	} while (first_node_index);

	return 0;
}

static int
call_qcsapi_wifi_get_node_infoset_all(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtn_nis_all_set nis;
	const char *usage = "<WiFi interface> <set id>";
	uint16_t set_id;

	if ((argc != 1)) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (safe_atou16(argv[0], &set_id, print, 0, QTN_NIS_ALL_SET_ID_MAX - 1) <= 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (local_node_infoset_all_print_all_nodes(p_calling_bundle, set_id, &nis) < 0)
		return 1;

	return 0;
}

static int
local_if_infoset_print(call_qcsapi_bundle *p_calling_bundle, const uint16_t set_id,
				struct qtnis_if_set *infoset)
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int i;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_qtnis_if_label) < ARRAY_SIZE(infoset->val));

	for (i = 0; i < ARRAY_SIZE(infoset->val); i++) {
		if (QTNIS_IS_SET(infoset, i)) {
			print_out(print, "%-*s: %llu\n",
				QTNIS_IF_LABEL_LEN, qcsapi_qtnis_if_label[set_id][i],
				infoset->val[i]);
		}
	}

	return 0;
}

static int
call_qcsapi_wifi_get_if_infoset(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtnis_if_set infoset;
	const char *usage = "<WiFi interface> <set id>";
	uint16_t set_id;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (safe_atou16(argv[0], &set_id, print, 0, QTNIS_SET_ID_MAX - 1) <= 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	retval = qcsapi_wifi_get_if_infoset(the_interface, set_id, &infoset);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	local_if_infoset_print(p_calling_bundle, set_id, &infoset);

	return retval;
}

static int
call_qcsapi_wifi_get_max_queued(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t node_index = p_calling_bundle->caller_generic_parameter.index;
	int local_remote_flag = QCSAPI_LOCAL_NODE;
	int reset_flag = 0;
	uint32_t max_queued, *p_max_queued = &max_queued;

	if (argc > 0) {
		if (parse_local_remote_flag(print, argv[0], &local_remote_flag) < 0) {
			return 1;
		}
	}

	if (argc > 1) {
		if (!isdigit(*argv[1])) {
			print_err(print, "Invalid format for reset flag\n", argv[1]);
			return 1;
		} else {
			reset_flag = atoi(argv[1]);
		}
	}

	if (argc > 2 && strcmp(argv[2], "NULL") == 0) {
		p_max_queued = NULL;
	}

	qcsapi_retval = qcsapi_wifi_get_max_queued(the_interface,
			node_index, local_remote_flag, reset_flag, p_max_queued);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", max_queued);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_associate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_ssid_fmt fmt;
	int argc_index;
	const char *usage = "<WiFi interface> [SSID fmt] <SSID>";
	char *join_ssid;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	argc_index = 0;
	if (argc < 2) {
		join_ssid = argv[argc_index];
		qcsapi_retval = qcsapi_wifi_associate(the_interface, join_ssid);
	} else {
		if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) < 0) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
		argc_index++;
		join_ssid = argv[argc_index];
		qcsapi_retval = qcsapi_wifi_associate2(the_interface, join_ssid, fmt, 0);
	}

	if (qcsapi_retval >= 0) {
		print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_disassociate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_disassociate(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_disassociate_sta(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 1;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mac_addr mac_addr = { 0 };

	if (argc < 1) {
		print_err(print, "MAC address required to be passed as a parameter\n");
	} else {
		qcsapi_retval = parse_mac_addr(argv[0], mac_addr);

		if (qcsapi_retval >= 0) {
			qcsapi_retval = qcsapi_wifi_disassociate_sta(the_interface, mac_addr);
			if (qcsapi_retval >= 0) {
				statval = 0;

				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			}
		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_reassociate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_reassociate(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_update_bss_cfg(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_wifi_mode mode;
	const char *ifname;
	const char *ssid;
	const char *param_name;
	const char *param_value;
	const char *param_type;
	int argc_index = 0;
	int api_version = 0;
	qcsapi_ssid_fmt fmt;
	const char *usage = "<WiFi interface> {ap <WiFi interface> | sta [<SSID fmt>] <ssid>} "
			"<param_name> <param_value> [<param_type>]";

	if (argc >= 4) {
		if (!strcasecmp(argv[argc_index], "ap")) {
			argc_index++;
			ifname = argv[argc_index];
			argc_index++;
			ssid = NULL;
			mode = qcsapi_access_point;
		} else if (!strcasecmp(argv[argc_index], "sta")) {
			argc_index++;
			if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) == 0) {
				api_version = 1;
				argc_index++;
			}
			ifname = the_interface;
			ssid = argv[argc_index];
			argc_index++;
			mode = qcsapi_station;
		} else {
			report_qcsapi_error(p_calling_bundle, -qcsapi_invalid_wifi_mode);
			return 1;
		}

		if (argc >= argc_index + 2) {
			param_name = argv[argc_index];
			argc_index++;
			param_value = argv[argc_index];
			argc_index++;
			if (argc > argc_index)
				param_type = argv[argc_index];
			else
				param_type = NULL;
			if (api_version)
				qcsapi_retval = qcsapi_wifi_update_bss2_cfg(ifname, mode,
							ssid, fmt, param_name,
							param_value, param_type);
			else
				qcsapi_retval = qcsapi_wifi_update_bss_cfg(ifname, mode,
							ssid, param_name,
							param_value, param_type);
		} else {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}

		return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
	}
	qcsapi_report_usage(p_calling_bundle, usage);

	return 1;
}

static int
call_qcsapi_wifi_get_bss_cfg(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_wifi_mode mode;
	const char *ifname;
	const char *ssid;
	const char *param_name;
	int argc_index = 0;
	int api_version = 0;
	qcsapi_ssid_fmt fmt = qcsapi_ssid_fmt_str;
	string_256 param_value = { '\0' };
	const char *usage =
		"<WiFi interface> {ap <WiFi interface> | sta [<SSID fmt>] <ssid>} <param_name>";

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (!strcasecmp(argv[argc_index], "ap")) {
		argc_index++;
		ifname = argv[argc_index];
		argc_index++;
		ssid = NULL;
		mode = qcsapi_access_point;
	} else if (!strcasecmp(argv[argc_index], "sta")) {
		argc_index++;
		if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) == 0) {
			api_version = 1;
			argc_index++;
		}
		ifname = the_interface;
		ssid = argv[argc_index];
		argc_index++;
		mode = qcsapi_station;
	} else {
		report_qcsapi_error(p_calling_bundle, -qcsapi_invalid_wifi_mode);
		return 1;
	}

	if (argc != argc_index + 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	param_name = argv[argc_index];
	if (api_version)
		qcsapi_retval = qcsapi_wifi_get_bss2_cfg(ifname, mode,
				ssid, fmt, param_name, param_value, sizeof(param_value));
	else
		qcsapi_retval = qcsapi_wifi_get_bss_cfg(ifname, mode,
				ssid, param_name, param_value, sizeof(param_value));

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_out(print, "%s\n", param_value);

	return 0;
}

static int
call_qcsapi_SSID_create_SSID(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> [SSID fmt] <SSID>";
	qcsapi_ssid_fmt fmt;
	int argc_index = 0;
	char *new_SSID;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	if (argc < 2) {
		new_SSID = argv[argc_index];
		qcsapi_retval = qcsapi_SSID_create_SSID(the_interface, new_SSID);
	} else {
		if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) < 0) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		} else {
			argc_index++;
			new_SSID = argv[argc_index];
			qcsapi_retval = qcsapi_SSID_create_SSID2(the_interface, new_SSID, fmt);
		}
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_remove_SSID(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> [SSID fmt] <SSID>";
	qcsapi_ssid_fmt fmt;
	int argc_index = 0;
	char *del_SSID;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	if (argc < 2) {
		del_SSID = argv[argc_index];
		qcsapi_retval = qcsapi_SSID_remove_SSID(the_interface, del_SSID);
	} else {
		if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) < 0) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		} else {
			argc_index++;
			del_SSID = argv[argc_index];
			qcsapi_retval = qcsapi_SSID_remove_SSID2(the_interface, del_SSID, fmt);
		}
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_verify_SSID(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> [SSID fmt] <SSID>";
	qcsapi_ssid_fmt fmt;
	int argc_index = 0;
	char *existing_SSID;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	if (argc < 2) {
		existing_SSID = argv[argc_index];
		qcsapi_retval = qcsapi_SSID_verify_SSID(the_interface, existing_SSID);
	} else {
		if (name_to_ssid_fmt_enum(argv[argc_index], &fmt) < 0) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		} else {
			argc_index++;
			existing_SSID = argv[argc_index];
			qcsapi_retval = qcsapi_SSID_verify_SSID2(the_interface, existing_SSID, fmt);
		}
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_rename_SSID(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage =
		"<WiFi interface> [<current SSID fmt>] <current SSID> [<new SSID fmt>] <new SSID>";
	qcsapi_ssid_fmt curr_fmt;
	qcsapi_ssid_fmt new_fmt;
	int argc_index;
	char *new_SSID;
	char *curr_SSID;
	int qcsapi_retval;
	const char *the_interface;

	the_interface = p_calling_bundle->caller_interface;
	argc_index = 0;
	curr_fmt = qcsapi_ssid_fmt_str;
	new_fmt = qcsapi_ssid_fmt_str;
	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}
	if (argc < 3) {
		curr_SSID = argv[argc_index++];
		new_SSID = argv[argc_index];
		qcsapi_retval = qcsapi_SSID_rename_SSID(the_interface, curr_SSID, new_SSID);
	} else {
		if (name_to_ssid_fmt_enum(argv[argc_index], &curr_fmt) == 0)
			argc_index++;
		curr_SSID = argv[argc_index];
		argc_index++;
		if (name_to_ssid_fmt_enum(argv[argc_index], &new_fmt) == 0) {
			argc_index++;
			if (argc < argc_index + 1) {
				qcsapi_report_usage(p_calling_bundle, usage);
				return 1;
			}
		}
		new_SSID = argv[argc_index];
		qcsapi_retval = qcsapi_SSID_rename_SSID2(the_interface, curr_SSID,
					curr_fmt, new_SSID, new_fmt);
	}
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int local_SSID_get_SSID_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[],
		int api_version)
{
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 * array_SSIDs has the space that receives the SSIDs from the API.
	 * Let this get as large as required, without affecting the integrity of the stack.
	 */
	static qcsapi_SSID2 array_ssids[QCSAPI_SSID_LIST_SIZE_MAX];

	int qcsapi_retval;
	unsigned int iter;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int sizeof_list = QCSAPI_SSID_LIST_SIZE_DEFAULT;
	char *list_ssids[QCSAPI_SSID_LIST_SIZE_MAX];
	struct qcsapi_data_32bytes list_fmt;
	const char *usage = "<WIFI interface> [max SSIDs]";

	if (argc > 0) {
		if (!isdigit(*argv[0])) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}

		sizeof_list = atoi(argv[0]);

		if (sizeof_list > QCSAPI_SSID_LIST_SIZE_MAX) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
	}

	for (iter = 0; iter < sizeof_list; iter++) {
		list_ssids[iter] = array_ssids[iter];
		*list_ssids[iter] = '\0';
	}
	if (api_version == 0)
		qcsapi_retval = qcsapi_SSID_get_SSID_list(the_interface, sizeof_list, list_ssids);
	else
		qcsapi_retval = qcsapi_SSID_get_SSID2_list(the_interface, sizeof_list, list_ssids,
					&list_fmt);
	if (qcsapi_retval >= 0) {
		for (iter = 0; iter < sizeof_list; iter++) {
			if (list_ssids[iter] == NULL || *list_ssids[iter] == '\0')
				break;
			if (api_version == 0)
				print_out(print, "%s\n", list_ssids[iter]);
			else
				print_out(print, (list_fmt.data[iter] == qcsapi_ssid_fmt_str) ?
							"\"%s\"\n" : "%s\n", list_ssids[iter]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;

}

static int
call_qcsapi_SSID_get_SSID_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	return local_SSID_get_SSID_list(p_calling_bundle, argc, argv, 0);
}

static int
call_qcsapi_SSID_get_SSID2_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	return local_SSID_get_SSID_list(p_calling_bundle, argc, argv, 1);
}

static int
call_qcsapi_SSID_get_protocol(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	string_16 SSID_proto;
	char *p_SSID_proto = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
			(strcmp(p_SSID, "NULL") == 0))
		p_SSID = NULL;
	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_SSID_proto = &SSID_proto[0];
	qcsapi_retval = qcsapi_SSID_get_protocol(the_interface, p_SSID, p_SSID_proto);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &SSID_proto[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_get_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char encryption_modes[36], *p_encryption_modes = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
			(strcmp(p_SSID, "NULL") == 0))
		p_SSID = NULL;
	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_encryption_modes = &encryption_modes[0];
	qcsapi_retval = qcsapi_SSID_get_encryption_modes(the_interface, p_SSID, p_encryption_modes);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &encryption_modes[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_get_group_encryption(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char group_encryption[36], *p_group_encryption = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
			(strcmp(p_SSID, "NULL") == 0))
		p_SSID = NULL;
	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_group_encryption = &group_encryption[0];
	qcsapi_retval = qcsapi_SSID_get_group_encryption(the_interface, p_SSID, p_group_encryption);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &group_encryption[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_get_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	char authentication_mode[36], *p_authentication_mode = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
			(strcmp(p_SSID, "NULL") == 0))
		p_SSID = NULL;
	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_authentication_mode = &authentication_mode[0];
	qcsapi_retval = qcsapi_SSID_get_authentication_mode(the_interface, p_SSID,
			p_authentication_mode);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &authentication_mode[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_SSID_set_protocol(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID set protocol, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		char *p_SSID_proto = argv[0];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* SSID protocol will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_SSID_proto = NULL;
		qcsapi_retval = qcsapi_SSID_set_protocol(the_interface, p_SSID, p_SSID_proto);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_set_encryption_modes(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID set encryption modes, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		char *p_encryption_modes = argv[0];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* Encryption modes will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_encryption_modes = NULL;
		qcsapi_retval = qcsapi_SSID_set_encryption_modes(the_interface, p_SSID,
				p_encryption_modes);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_set_group_encryption(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID set group encryption\n");
		print_err(print, "Usage: call_qcsapi SSID_set_group_encryption <WiFi interface> <SSID> <\"TKIP\"|\"CCMP\">\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		char *p_group_encryption = argv[0];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* Group encryption will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_group_encryption = NULL;
		qcsapi_retval = qcsapi_SSID_set_group_encryption(the_interface, p_SSID,
				p_group_encryption);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_set_authentication_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID set authentication mode, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		char *p_authentication_mode = argv[0];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* Authentication mode will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_authentication_mode = NULL;
		qcsapi_retval = qcsapi_SSID_set_authentication_mode(the_interface, p_SSID,
				p_authentication_mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_get_pre_shared_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 *Argument list needs to have the index.
	 */
	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID get key passphrase, count is %d\n", argc);
		statval = 1;
	} else {
		char pre_shared_key[68], *p_pre_shared_key = NULL;
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		qcsapi_unsigned_int the_index = (qcsapi_unsigned_int) atoi(argv[0]);

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		if (argc < 2 || strcmp(argv[1], "NULL") != 0)
			p_pre_shared_key = &pre_shared_key[0];
		qcsapi_retval = qcsapi_SSID_get_pre_shared_key(the_interface, p_SSID, the_index,
				p_pre_shared_key);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", &pre_shared_key[0]);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_get_key_passphrase(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 *Argument list needs to have the index.
	 */
	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID get key passphrase, count is %d\n", argc);
		statval = 1;
	} else {
		char passphrase[68], *p_passphrase = NULL;
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		qcsapi_unsigned_int the_index = (qcsapi_unsigned_int) atoi(argv[0]);

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		if (argc < 2 || strcmp(argv[1], "NULL") != 0)
			p_passphrase = &passphrase[0];
		qcsapi_retval = qcsapi_SSID_get_key_passphrase(the_interface, p_SSID, the_index,
				p_passphrase);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", &passphrase[0]);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_set_pre_shared_key(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 *Argument list needs to have both the index and the PSK.
	 */
	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi SSID set key passphrase, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		qcsapi_unsigned_int the_index = (qcsapi_unsigned_int) atoi(argv[0]);
		char *p_PSK = argv[1];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* PSK will not be NULL. */

		if (strcmp(argv[1], "NULL") == 0)
			p_PSK = NULL;
		qcsapi_retval = qcsapi_SSID_set_pre_shared_key(the_interface, p_SSID, the_index,
				p_PSK);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_add_radius_auth_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <ipaddr> <port> <shared-key>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_add_radius_auth_server_cfg(the_interface,
			argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_del_radius_auth_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <ipaddr> <port>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_del_radius_auth_server_cfg(the_interface, argv[0], argv[1]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_radius_auth_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_1024 radius_auth_server_cfg;

	qcsapi_retval = qcsapi_wifi_get_radius_auth_server_cfg(the_interface,
			radius_auth_server_cfg);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, radius_auth_server_cfg);
}

static int
call_qcsapi_wifi_add_radius_acct_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <ipaddr> <port> <shared-key>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_add_radius_acct_server_cfg(the_interface,
			argv[0], argv[1], argv[2]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_del_radius_acct_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <ipaddr> <port>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_del_radius_acct_server_cfg(the_interface, argv[0], argv[1]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_radius_acct_server_cfg(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_1024 radius_server_cfg;

	qcsapi_retval = qcsapi_wifi_get_radius_acct_server_cfg(the_interface, radius_server_cfg);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, radius_server_cfg);
}

static int
call_qcsapi_wifi_set_own_ip_addr(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi set own ip addr, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_own_ip_addr <WiFi interface> <ipaddr>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_own_ip_addr = argv[0];

		if (strcmp(argv[0], "NULL") == 0)
			p_own_ip_addr = NULL;
		qcsapi_retval = qcsapi_wifi_set_own_ip_addr(the_interface, p_own_ip_addr);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_set_key_passphrase(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 *Argument list needs to have both the index and the passphrase.
	 */
	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi SSID set key passphrase, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;
		qcsapi_unsigned_int the_index = (qcsapi_unsigned_int) atoi(argv[0]);
		char *p_passphrase = argv[1];

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		/* Passphrase of NULL is not valid. */

		if (strcmp(argv[1], "NULL") == 0)
			p_passphrase = NULL;
		qcsapi_retval = qcsapi_SSID_set_key_passphrase(the_interface, p_SSID, the_index,
				p_passphrase);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_SSID_get_pmf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int pmf_cap = 0;
	int *p_pmf_cap = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
			(strcmp(p_SSID, "NULL") == 0))
		p_SSID = NULL;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_pmf_cap = &pmf_cap;

	qcsapi_retval = qcsapi_SSID_get_pmf(the_interface, p_SSID, p_pmf_cap);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", pmf_cap);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_SSID_set_pmf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *p_SSID = p_calling_bundle->caller_generic_parameter.parameter_type.the_SSID;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi SSID set pmf mode, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int pmf_cap = atoi(argv[0]);

		if (((internal_flags & m_force_NULL_address) == m_force_NULL_address) &&
				(strcmp(p_SSID, "NULL") == 0))
			p_SSID = NULL;

		qcsapi_retval = qcsapi_SSID_set_pmf(the_interface, p_SSID, pmf_cap);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_SSID_get_wps_SSID(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_SSID the_wps_SSID = "";
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 0 && strcmp(argv[0], "NULL") == 0)
		qcsapi_retval = qcsapi_SSID_get_wps_SSID(the_interface, NULL);
	else
		qcsapi_retval = qcsapi_SSID_get_wps_SSID(the_interface, the_wps_SSID);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", the_wps_SSID);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_vlanid_valid(char *vlanid, int all)
{
	int vid;

	if (all && !strcasecmp(vlanid, "all"))
		return QVLAN_VID_ALL;

	vid = atoi(vlanid);
	if (vid >= 1 && vid < QVLAN_VID_MAX)
		return vid;
	else
		return -EFAULT;
}

static int call_qcsapi_wifi_vlan_parser(char *argv, int cmd)
{
	if (!strcasecmp(argv, "default"))
		cmd |= e_qcsapi_vlan_pvid;
	else if (!strcasecmp(argv, "tag"))
		cmd |= e_qcsapi_vlan_tag;
	else if (!strcasecmp(argv, "untag"))
		cmd |= e_qcsapi_vlan_untag;
	else if (!strcasecmp(argv, "delete"))
		cmd |= e_qcsapi_vlan_del;
	else if (!strcasecmp(argv, "vlan_prio"))
		cmd |= e_qcsapi_vlan_prio;
	else
		cmd = 0;

	return cmd;
}

static int
call_qcsapi_wifi_vlan_config(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_vlan_cmd cmd = 0;
	int vlanid = 0;
	int index;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *usage = "Usage:\n"
			"    call_qcsapi vlan_config wifi0 enable\n"
			"    call_qcsapi vlan_config wifi0 disable\n"
			"    call_qcsapi vlan_config <interface> reset\n"
			"    call_qcsapi vlan_config <interface> access <VLAN ID> [delete] [vlan_prio]\n"
			"    call_qcsapi vlan_config <interface> trunk <VLAN ID> "
			"[ {tag | untag} ] [default] [delete] [vlan_prio]\n"
			"    call_qcsapi vlan_config <interface> dynamic {0 | 1}\n"
			"    call_qcsapi vlan_config <interface> hybrid <VLAN ID>   (deprecated)\n"
			"    call_qcsapi vlan_config <interface> bind <VLAN ID>     (deprecated)\n"
			"    call_qcsapi vlan_config <interface> unbind <VLAN ID>   (deprecated)\n";

	if (argc < 2) {
		if (!strcasecmp(argv[0], "enable"))
			cmd = e_qcsapi_vlan_enable;
		else if (!strcasecmp(argv[0], "disable"))
			cmd = e_qcsapi_vlan_disable;
		else if (!strcasecmp(argv[0], "reset"))
			cmd = e_qcsapi_vlan_reset;
	} else if (!strcasecmp(argv[0], "bind")) {
		vlanid = call_qcsapi_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc == 2)
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
	} else if (!strcasecmp(argv[0], "unbind")) {
		vlanid = call_qcsapi_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc == 2)
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_del |
					e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
	} else if (!strcasecmp(argv[0], "dynamic")) {
		if (argc == 2) {
			if (atoi(argv[1]))
				cmd = e_qcsapi_vlan_dynamic;
			else
				cmd = e_qcsapi_vlan_undynamic;
		}
	} else if (!strcasecmp(argv[0], "access")) {
		vlanid = call_qcsapi_wifi_vlanid_valid(argv[1], 0);
		if (vlanid >= 0 && argc <= 3) {
			cmd = e_qcsapi_vlan_access | e_qcsapi_vlan_untag | e_qcsapi_vlan_pvid;
			for (index = 2; index < argc; index++)
				cmd = call_qcsapi_wifi_vlan_parser(argv[index], cmd);
		}
	} else if (!strcasecmp(argv[0], "trunk") || !strcasecmp(argv[0], "hybrid")) {
		vlanid = call_qcsapi_wifi_vlanid_valid(argv[1], 1);
		if (vlanid >= 0 && argc <= 5) {
			cmd = e_qcsapi_vlan_trunk;
			for (index = 2; index < argc; index++)
				cmd = call_qcsapi_wifi_vlan_parser(argv[index], cmd);
			if ((vlanid == QVLAN_VID_ALL) && (cmd & e_qcsapi_vlan_pvid))
				cmd = 0;
		}
	}

	qcsapi_retval = qcsapi_wifi_vlan_config(the_interface, cmd, vlanid);
	if (qcsapi_retval < 0) {
		if (qcsapi_retval == -qcsapi_param_value_invalid)
			print_out(print, usage);
		else
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return -EINVAL;
	}

	if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return 0;
}

static void
call_qcsapi_wifi_print_vlan_config(const call_qcsapi_bundle *p_calling_bundle, const char *ifname,
		struct qcsapi_data_2Kbytes *byte)
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qtn_vlan_config *vcfg = (struct qtn_vlan_config *)byte;
	uint16_t vmode;
	uint16_t vid;
	uint16_t i;
	uint16_t j;
	uint32_t tagrx;

	if (vcfg->vlan_cfg) {
		vmode = ((vcfg->vlan_cfg & QVLAN_MASK_MODE) >> QVLAN_SHIFT_MODE);
		vid = (vcfg->vlan_cfg & QVLAN_MASK_VID);
	} else {
		print_out(print, "tagrx VLAN:");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			tagrx = qtn_vlan_get_tagrx(vcfg->u.tagrx_config, i);
			if (tagrx) {
				if ((j++ & 0xF) == 0)
					print_out(print, "\n\t");
				print_out(print, "%u-%u, ", i, tagrx);
			}
		}
		print_out(print, "\n");
		return;
	}

	switch (vmode) {
	case QVLAN_MODE_TRUNK:
		print_out(print, "%s, default VLAN %u\n", QVLAN_MODE_STR_TRUNK, vid);

		print_out(print, "    Member of VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");

		print_out(print, "    Untagged VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i) &&
					is_clr_a(vcfg->u.dev_config.tag_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");

		print_out(print, "    Use VLAN priority VLANs: ");
		for (i = 1, j = 0; i < QVLAN_VID_MAX; i++) {
			if (is_set_a(vcfg->u.dev_config.member_bitmap, i) &&
					is_set_a(vcfg->u.dev_config.prio_bitmap, i)) {
				if ((++j & 0xF) == 0)
					print_out(print, "\n        ");
				print_out(print, "%u,", i);
			}
		}
		print_out(print, "\n");
		break;
	case QVLAN_MODE_ACCESS:
		print_out(print, "%s, VLAN %u%s\n", QVLAN_MODE_STR_ACCESS, vid,
				is_set_a(vcfg->u.dev_config.prio_bitmap,
						vid) ? ", Use VLAN priority" : "");
		break;
	case QVLAN_MODE_DYNAMIC:
		print_out(print, "%s\n", QVLAN_MODE_STR_DYNAMIC);
		break;
	default:
		print_out(print, "VLAN disabled\n");
		break;
	}
}

static int
call_qcsapi_wifi_show_vlan_config(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	struct qtn_vlan_config *vcfg;
	const char *ifname = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc > 1) {
		print_err(print, "Too many parameters for show_vlan_config command\n");
		qcsapi_retval = 1;
	} else {
		vcfg = (struct qtn_vlan_config *)malloc(sizeof(struct qcsapi_data_2Kbytes));
		if (!vcfg) {
			print_err(print, "Not enough memory to execute the API\n");
			return -1;
		}

		memset(vcfg, 0, sizeof(*vcfg));

		if (argc == 1 && !strcmp(argv[0], "tagrx")) {
			qcsapi_retval = qcsapi_wifi_show_vlan_config(ifname,
					(struct qcsapi_data_2Kbytes *)vcfg, argv[0]);
			qtn_vlan_config_ntohl(vcfg, 1);
		} else if (argc == 0) {
			qcsapi_retval = qcsapi_wifi_show_vlan_config(ifname,
					(struct qcsapi_data_2Kbytes *)vcfg, NULL);
			qtn_vlan_config_ntohl(vcfg, 0);
		} else {
			qcsapi_retval = -1;
		}

		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			qcsapi_retval = 1;
		} else {
			call_qcsapi_wifi_print_vlan_config(p_calling_bundle, ifname,
					(struct qcsapi_data_2Kbytes *)vcfg);
			qcsapi_retval = 0;
		}
		free(vcfg);
	}

	return qcsapi_retval;
}

static int
call_qcsapi_enable_vlan_promisc(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int enabled = ! !atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_set_vlan_promisc(enabled);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_set_multicast(call_qcsapi_bundle *p_calling_bundle, int add, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t ipaddr;
	uint32_t ipaddr_ne;
	qcsapi_mac_addr mac_addr = { 0 };
	char *usage = "Usage: call_qcsapi {add_multicast | del_multicast} "
			"<IP address> <MAC address>\n";

	/* FIXME subnets and IPv6 are not yet supported */

	if (argc != 2) {
		print_err(print, usage);
		return -EINVAL;
	}

	if (inet_pton(AF_INET, argv[0], &ipaddr_ne) != 1) {
		/* FIXME support IPv6 */
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	ipaddr = ntohl(ipaddr_ne);

	if (!IN_MULTICAST(ipaddr)) {
		print_err(print, "invalid multicast IPv4 address " NIPQUAD_FMT "\n",
				NIPQUAD(ipaddr_ne));
		return -EINVAL;
	}

	qcsapi_retval = parse_mac_addr(argv[1], mac_addr);
	if (qcsapi_retval < 0) {
		print_err(print, "Error parsing MAC address %s\n", argv[1]);
		return qcsapi_retval;
	}

	if (add)
		qcsapi_retval = qcsapi_wifi_add_multicast(ipaddr, mac_addr);
	else
		qcsapi_retval = qcsapi_wifi_del_multicast(ipaddr, mac_addr);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

#define QCSAPI_FWT_GET_MAX	4096

static int
call_qcsapi_get_multicast_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	char buf[QCSAPI_FWT_GET_MAX];

	qcsapi_retval = qcsapi_wifi_get_multicast_list(buf, sizeof(buf));
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_out(print, "%s", buf);

	if (verbose_flag >= 0)
		print_out(print, "complete\n");

	return 0;
}

static int
call_qcsapi_set_ipff(call_qcsapi_bundle *p_calling_bundle, int add, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t ipaddr;
	uint32_t ipaddr_ne;
	const char *usage = "Usage: call_qcsapi {add_ipff | del_ipff} <ip_address>\n";

	/* FIXME subnets and IPv6 are not yet supported */

	if (argc != 1) {
		print_err(print, usage);
		return -EINVAL;
	}

	if (inet_pton(AF_INET, argv[0], &ipaddr_ne) != 1) {
		print_err(print, "invalid IPv4 address %s\n", argv[0]);
		return -EINVAL;
	}
	ipaddr = ntohl(ipaddr_ne);

	if (!IN_MULTICAST(ipaddr)) {
		print_err(print, "invalid multicast IPv4 address " NIPQUAD_FMT "\n",
				NIPQUAD(ipaddr_ne));
		return -EINVAL;
	}

	if (add) {
		qcsapi_retval = qcsapi_wifi_add_ipff(ipaddr);
	} else {
		qcsapi_retval = qcsapi_wifi_del_ipff(ipaddr);
	}

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return 0;
}

static int call_qcsapi_get_ipff(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
#define QCSAPI_IPFF_GET_MAX	256
	char buf[IP_ADDR_STR_LEN * QCSAPI_IPFF_GET_MAX];

	qcsapi_wifi_get_ipff(buf, sizeof(buf));

	print_out(print, "%s", buf);

	if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return 0;
}

static int
call_qcsapi_wifi_get_rts_threshold(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int rts_threshold;

	qcsapi_retval = qcsapi_wifi_get_rts_threshold(the_interface, &rts_threshold);
	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", rts_threshold);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_set_rts_threshold(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int rts_threshold;
	int32_t arg;

	if (sscanf(argv[0], "%d", &arg) != 1) {
		print_err(print, "Error parsing '%s'\n", argv[0]);
		return 1;
	}

	if (arg < IEEE80211_RTS_MIN) {
		print_err(print, "Value should be non negative\n");
		return 1;
	}

	rts_threshold = arg;

	qcsapi_retval = qcsapi_wifi_set_rts_threshold(the_interface, rts_threshold);
	if (qcsapi_retval >= 0) {
		print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_disable_wps(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int disable_wps = atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_disable_wps(the_interface, disable_wps);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_start_cca(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int channel;
	int duration;

	if (argc < 2) {
		print_err(print, "Format: start_cca <chan-num(36)> <msec-duration(40)> \n");
		return 1;
	}

	channel = atoi(argv[0]);
	duration = atoi(argv[1]);

	qcsapi_retval = qcsapi_wifi_start_cca(the_interface, channel, duration);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_scan_chan_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes chan_list;
	uint32_t count = 0;
	int i;

	memset(&chan_list, 0, sizeof(chan_list));
	qcsapi_retval = qcsapi_wifi_get_scan_chan_list(the_interface, &chan_list, &count);
	if (qcsapi_retval >= 0) {
		print_out(print, "%d channels in scan list: ", count);
		for (i = 0; i < count; i++)
			print_out(print, "%d%c", chan_list.data[i], (i < (count - 1)) ? ',' : '\n');
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scan_chan_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes chan_list;
	struct qcsapi_data_256bytes *p_chan_list;
	uint32_t count = 0;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> {default | <channel list>}\n");
		return 1;
	}

	if (strcmp(argv[0], "default") == 0) {
		p_chan_list = NULL;
		count = 0;
	} else {
		p_chan_list = &chan_list;
		memset(&chan_list, 0, sizeof(chan_list));
		statval = string_to_list(print, argv[0], chan_list.data, &count);
		if (statval < 0)
			return statval;
	}

	qcsapi_retval = qcsapi_wifi_set_scan_chan_list(the_interface, p_chan_list, count);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_start_scan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int pick_flags = 0;

	if (argc > 0) {
		while (argc > 0) {
			if (!strcasecmp("reentry", argv[0])) {
				pick_flags |= IEEE80211_PICK_REENTRY;
			} else if (!strcasecmp("clearest", argv[0])) {
				pick_flags |= IEEE80211_PICK_CLEAREST;
			} else if (!strcasecmp("no_pick", argv[0])) {
				pick_flags |= IEEE80211_PICK_NOPICK;
			} else if (!strcasecmp("background", argv[0])) {
				pick_flags |= IEEE80211_PICK_NOPICK_BG;
			} else if (!strcasecmp("dfs", argv[0])) {
				pick_flags |= IEEE80211_PICK_DFS;
			} else if (!strcasecmp("non_dfs", argv[0])) {
				pick_flags |= IEEE80211_PICK_NONDFS;
			} else if (!strcasecmp("all", argv[0])) {
				pick_flags |= IEEE80211_PICK_ALL;
			} else if (!strcasecmp("flush", argv[0])) {
				pick_flags |= IEEE80211_PICK_SCAN_FLUSH;
			} else if (!strcasecmp("active", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_ACTIVE;
			} else if (!strcasecmp("fast", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_FAST;
			} else if (!strcasecmp("normal", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_NORMAL;
			} else if (!strcasecmp("slow", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_SLOW;
			} else if (!strcasecmp("auto", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_MULTI_SLOTS_AUTO;
			} else if (!strcasecmp("check", argv[0])) {
				pick_flags |= IEEE80211_PICK_BG_CHECK;
			} else {
				goto err_ret;
			}
			argc--;
			argv++;
		}

		if (pick_flags & IEEE80211_PICK_ALGORITHM_MASK) {
			uint32_t algorithm = pick_flags & IEEE80211_PICK_ALGORITHM_MASK;
			uint32_t chan_set = pick_flags & IEEE80211_PICK_DOMIAN_MASK;

			if (IS_MULTIPLE_BITS_SET(algorithm)) {
				print_out(print, "Only one pick algorithm can be specified\n");
				goto err_ret;
			}
			if (chan_set) {
				if (IS_MULTIPLE_BITS_SET(chan_set)) {
					print_out(print, "Only one channel set can be specified\n");
					goto err_ret;
				}
			} else {
				pick_flags |= IEEE80211_PICK_ALL;
			}
		} else {
			print_out(print, "pick algorithm was not specified\n");
			goto err_ret;
		}

		if (pick_flags & IEEE80211_PICK_NOPICK_BG) {
			uint32_t dfs_mode = pick_flags & IEEE80211_PICK_BG_MODE_MASK;

			if (IS_MULTIPLE_BITS_SET(dfs_mode)) {
				print_out(print, "Specify only one background scan mode\n");
				goto err_ret;
			}
		} else if (pick_flags & IEEE80211_PICK_BG_CHECK) {
			print_out(print, "check flag is only for QTN background scan\n");
			goto err_ret;
		}
	}

	qcsapi_retval = qcsapi_wifi_start_scan_ext(the_interface, pick_flags);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

err_ret:
	print_start_scan_usage(print);
	return 1;
}

static int
call_qcsapi_wifi_cancel_scan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int force = 0;
	int qcsapi_retval;

	if (argc == 1) {
		if (!strcasecmp("force", argv[0])) {
			force = 1;
		} else {
			print_out(print, "Unknown parameter: %s\n", argv[0]);
			print_cancel_scan_usage(print);
			return 1;
		}
	} else if (argc != 0) {
		print_cancel_scan_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_cancel_scan(the_interface, force);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_scan_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int scanstatus = -1;

	qcsapi_retval = qcsapi_wifi_get_scan_status(the_interface, &scanstatus);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", scanstatus);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_cac_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int cacstatus = -1;

	qcsapi_retval = qcsapi_wifi_get_cac_status(the_interface, &cacstatus);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", cacstatus);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wait_scan_completes(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	time_t timeout;

	if (argc < 1) {
		print_err(print, "Wait Scan Completes requires a timeout\n");
		return 1;
	}

	timeout = (time_t) atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_wait_scan_completes(the_interface, timeout);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_results_AP_scan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int count_APs_scanned, *p_count_APs_scanned = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_count_APs_scanned = &count_APs_scanned;
	qcsapi_retval = qcsapi_wifi_get_results_AP_scan(the_interface, p_count_APs_scanned);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			/*
			 *Unlike most APIs that return a value by reference, this API permits
			 *that reference address to be 0.
			 *
			 *Primary purpose of this API is to get the results of the last AP scan.
			 */
			if (p_count_APs_scanned != NULL)
				print_out(print, "%d\n", (int)count_APs_scanned);
			else
				print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_count_APs_scanned(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int count_APs_scanned, *p_count_APs_scanned = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_count_APs_scanned = &count_APs_scanned;
	qcsapi_retval = qcsapi_wifi_get_count_APs_scanned(the_interface, p_count_APs_scanned);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", (int)count_APs_scanned);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_properties_AP(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_ap_properties ap_properties;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int ap_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc > 0) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <index>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_properties_AP(the_interface, ap_index, &ap_properties);
	if (qcsapi_retval >= 0) {
		char mac_addr_string[24];

		snprintf(&mac_addr_string[0], sizeof(mac_addr_string), MACFILTERINGMACFMT,
				ap_properties.ap_mac_addr[0],
				ap_properties.ap_mac_addr[1],
				ap_properties.ap_mac_addr[2],
				ap_properties.ap_mac_addr[3],
				ap_properties.ap_mac_addr[4], ap_properties.ap_mac_addr[5]);

		print_out(print, "\"%s\" %s %d %d %x %d %d %d %u %d %d %d %d %d %d %d %d %d %s %s\n", ap_properties.ap_name_SSID, &mac_addr_string[0], ap_properties.ap_channel, ap_properties.ap_RSSI, ap_properties.ap_flags, ap_properties.ap_protocol, ap_properties.ap_authentication_mode, ap_properties.ap_encryption_modes, ap_properties.ap_best_data_rate, ap_properties.ap_wps, ap_properties.ap_80211_proto, ap_properties.ap_qhop_role, ap_properties.ap_bw, ap_properties.ap_noise, ap_properties.ap_opmode, ap_properties.ap_bintval, ap_properties.ap_dtimperiod, ap_properties.ap_11b_present, ap_properties.ap_basic_rates, ap_properties.ap_support_rates);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_wps_ie_scanned_AP(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int retval = 0;
	int qcsapi_retval;
	struct qcsapi_ie_data ie_data;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int ap_index = p_calling_bundle->caller_generic_parameter.index;

	qcsapi_retval = qcsapi_wifi_get_wps_ie_scanned_AP(the_interface, ap_index, &ie_data);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			char wps_ie_hex[QCSAPI_MAX_IE_INFOLEN * 2 + 1];
			int i;
			int pos;

			memset(wps_ie_hex, 0, sizeof(wps_ie_hex));
			pos = 0;
			for (i = 0; i < ie_data.ie_len; i++) {
				snprintf(&wps_ie_hex[pos], sizeof(wps_ie_hex) - pos,
						"%02X", ie_data.ie_buf[i]);
				pos += 2;
			}

			print_out(print, "%d %s\n", ie_data.ie_len, wps_ie_hex);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		retval = 1;
	}

	return retval;
}

static int
call_qcsapi_wifi_get_mcs_rate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char mcs_rate[16];
	char *p_mcs_rate = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	p_mcs_rate = &mcs_rate[0];
	qcsapi_retval = qcsapi_wifi_get_mcs_rate(the_interface, p_mcs_rate);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", &mcs_rate[0]);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mcs_rate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set MCS rate, count is %d\n", argc);
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *p_mcs_rate = argv[0];

		/* MCS rate will not be NULL ... */

		if (strcmp(argv[0], "NULL") == 0)
			p_mcs_rate = NULL;
		qcsapi_retval = qcsapi_wifi_set_mcs_rate(the_interface, p_mcs_rate);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_time_associated_per_association(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int time_associated = 0;
	qcsapi_unsigned_int *p_time_associated = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0)
		p_time_associated = &time_associated;

	qcsapi_retval = qcsapi_wifi_get_time_associated_per_association(the_interface,
			association_index, p_time_associated);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", time_associated);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_add_peer(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr the_mac_addr;
	int ival = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int encryption = 0;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi wds add peer, count is %d\n", argc);
		statval = 1;
	} else {
		ival = parse_mac_addr(argv[0], the_mac_addr);
		if ((argc > 1) && (strcasecmp(argv[1], "encrypt") == 0))
			encryption = 1;

		if (ival >= 0) {
			qcsapi_retval = qcsapi_wds_add_peer_encrypt(the_interface, the_mac_addr,
					encryption);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}

		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_remove_peer(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr the_mac_addr;
	int ival = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi wds remove peer, count is %d\n", argc);
		statval = 1;
	} else {
		ival = parse_mac_addr(argv[0], the_mac_addr);

		if (ival >= 0) {
			qcsapi_retval = qcsapi_wds_remove_peer(the_interface, the_mac_addr);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}

		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_get_peer_address(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr peer_address;
	qcsapi_unsigned_int index = 0;
	char temp_peer_address_str[20];
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi get peer address, count is %d\n", argc);
		statval = 1;
	} else {
		index = (qcsapi_unsigned_int) atoi(argv[0]);
		qcsapi_retval = qcsapi_wds_get_peer_address(the_interface, index, peer_address);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				snprintf(&temp_peer_address_str[0], sizeof(temp_peer_address_str),
						MACFILTERINGMACFMT,
						peer_address[0],
						peer_address[1],
						peer_address[2],
						peer_address[3], peer_address[4], peer_address[5]);
				print_out(print, "%s\n", temp_peer_address_str);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_set_psk(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr peer_address;
	char *p_pre_shared_key = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int ival = 0;
	const char *usage = "<WiFi interface> <BSSID of peer AP> {<WDS PSK> | NULL}</c>";

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, usage);
		statval = 1;
	} else {
		ival = parse_mac_addr(argv[0], peer_address);

		if (ival >= 0) {
			p_pre_shared_key = argv[1];
			if (strcmp(p_pre_shared_key, "NULL") == 0) {
				p_pre_shared_key = NULL;
			}
			qcsapi_retval = qcsapi_wifi_wds_set_psk(the_interface, peer_address,
					p_pre_shared_key);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_set_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr peer_address;
	int rbs_mode;
	int rbs_mask;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int ival = 0;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi WiFi wds set "
				"mode, count is %d\n", argc);
		statval = 1;
	} else {
		ival = parse_mac_addr(argv[0], peer_address);

		if (ival >= 0) {
			if (strcasecmp(argv[1], "rbs") == 0) {
				rbs_mode = IEEE80211_QTN_WDS_RBS;
				rbs_mask = IEEE80211_QTN_WDS_MASK;
			} else if (strcasecmp(argv[1], "mbs") == 0) {
				rbs_mode = IEEE80211_QTN_WDS_MBS;
				rbs_mask = IEEE80211_QTN_WDS_MASK;
			} else if (strcasecmp(argv[1], "wds") == 0) {
				rbs_mode = IEEE80211_QTN_WDS_ONLY;
				rbs_mask = IEEE80211_QTN_WDS_MASK;
			} else if (strcasecmp(argv[1], "reset") == 0) {
				rbs_mode = 0;
				rbs_mask = IEEE80211_QTN_EXTDR_ALLMASK;
			} else {
				print_out(print, "Error parsing WDS mode %s\n", argv[1]);
				return 1;
			}

			qcsapi_retval = qcsapi_wds_set_mode(the_interface, peer_address,
					ieee80211_extdr_combinate(rbs_mode, rbs_mask));

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_wds_get_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int rbs_mode;
	qcsapi_unsigned_int index = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *mode_str[] = { "mbs", "rbs", "none" };

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi get "
				"peer address, count is %d\n", argc);
		statval = 1;
	} else {
		index = (qcsapi_unsigned_int) atoi(argv[0]);
		qcsapi_retval = qcsapi_wds_get_mode(the_interface, index, &rbs_mode);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "wds %s\n", mode_str[rbs_mode]);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_qos_get_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int the_queue = -1;
	int the_param = -1;
	int ap_bss_flag = 0;
	int qos_param_value;
	int i;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi qos_get_param <WiFi interface> "
				"<QoS queue> <QoS param> [AP / BSS flag]\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		the_queue = atoi(argv[0]);
	} else if (name_to_qos_queue_type(argv[0], &the_queue) == 0) {
		print_err(print, "Unrecognized QoS queue %s\n", argv[0]);
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS queue ID and name:\n");
			for (i = 0; i < ARRAY_SIZE(qcsapi_qos_queue_table); i++)
				print_out(print, "%d: %s\n",
						qcsapi_qos_queue_table[i].qos_queue_type,
						qcsapi_qos_queue_table[i].qos_queue_name);
		}
		return 1;
	}

	if (isdigit(*argv[1])) {
		the_param = atoi(argv[1]);
	} else if (name_to_qos_param_type(argv[1], &the_param) == 0) {
		print_err(print, "Unrecognized QoS param %s\n", argv[1]);
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS param ID and name:\n");
			for (i = 0; i < ARRAY_SIZE(qcsapi_qos_param_table); i++)
				print_out(print, "%d: %s\n",
						qcsapi_qos_param_table[i].qos_param_type,
						qcsapi_qos_param_table[i].qos_param_name);
		}
		return 1;
	}

	if (argc > 2) {
		ap_bss_flag = atoi(argv[2]);
	}

	qcsapi_retval = qcsapi_wifi_qos_get_param(the_interface,
			the_queue, the_param, ap_bss_flag, &qos_param_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", qos_param_value);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_qos_set_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int the_queue = -1;
	int the_param = -1;
	int ap_bss_flag = 0;
	int param_value = -1;
	int i;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Usage: call_qcsapi qos_get_param <WiFi interface> "
				"<QoS queue> <QoS param> <value> [AP / BSS flag]\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		the_queue = atoi(argv[0]);
	} else if (name_to_qos_queue_type(argv[0], &the_queue) == 0) {
		print_err(print, "Unrecognized QoS queue %s\n", argv[0]);
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS queue ID and name:\n");
			for (i = 0; i < ARRAY_SIZE(qcsapi_qos_queue_table); i++)
				print_out(print, "%d: %s\n",
						qcsapi_qos_queue_table[i].qos_queue_type,
						qcsapi_qos_queue_table[i].qos_queue_name);
		}
		return 1;
	}

	if (isdigit(*argv[1])) {
		the_param = atoi(argv[1]);
	} else if (name_to_qos_param_type(argv[1], &the_param) == 0) {
		print_err(print, "Unrecognized QoS param %s\n", argv[1]);
		if (verbose_flag >= 0) {
			print_out(print, "Supported QOS param ID and name:\n");
			for (i = 0; i < ARRAY_SIZE(qcsapi_qos_param_table); i++)
				print_out(print, "%d: %s\n",
						qcsapi_qos_param_table[i].qos_param_type,
						qcsapi_qos_param_table[i].qos_param_name);
		}
		return 1;
	}

	if (isdigit(*argv[2])) {
		param_value = atoi(argv[2]);
	} else {
		print_err(print, "Unrecognized QoS param's value %s\n", argv[2]);
		return 1;
	}

	if (argc > 3)
		ap_bss_flag = atoi(argv[3]);

	qcsapi_retval = qcsapi_wifi_qos_set_param(the_interface,
			the_queue, the_param, ap_bss_flag, param_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_wmm_ac_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_64 ac_map;	/* Must be a string for the RPC generation Perl script */
	qcsapi_output *print = p_calling_bundle->caller_output;

	assert(sizeof(ac_map) >= QCSAPI_WIFI_AC_MAP_SIZE);

	memset(ac_map, 0, sizeof(ac_map));
	qcsapi_retval = qcsapi_wifi_get_wmm_ac_map(the_interface, ac_map);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", ac_map);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_wmm_ac_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int user_prio = -1;
	int ac_index = -1;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_wmm_ac_map <WiFi interface> "
				"<user priority> <AC index>\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		user_prio = atoi(argv[0]);
	} else {
		print_err(print, "Unrecognized user priority %s,"
				"Supported user priority range: 0~7\n", argv[0]);
		return 1;
	}

	if (isdigit(*argv[1])) {
		ac_index = atoi(argv[1]);
	} else {
		print_err(print, "Unrecognized AC index %s, "
				"Supported AC index range: 0(AC_BE), 1(AC_BK), 2(AC_VI), 3(AC_VO)\n",
				argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_wmm_ac_map(the_interface, user_prio, ac_index);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dscp_8021p_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int i;
	int statval = 0;
	int qcsapi_retval = 0;
	string_64 dot1p_mapping;	/* Must be a string for the RPC generation Perl script */
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	assert(sizeof(dot1p_mapping) >= IP_DSCP_NUM);

	memset(dot1p_mapping, 0, sizeof(dot1p_mapping));
	qcsapi_retval = qcsapi_wifi_get_dscp_8021p_map(the_interface, (char *)dot1p_mapping);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "IP DSCP/802.1p UP:\n");
			for (i = 0; i < IP_DSCP_NUM; i++) {
				print_out(print, "%2d/%d ", i, dot1p_mapping[i]);
				if ((i + 1) % IEEE8021P_PRIORITY_NUM == 0)
					print_out(print, "\n");
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dscp_8021p_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t dot1p_up = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_dscp_8021p_map <WiFi interface> "
				"<IP DSCP list> <802.1p UP>\n");
		return 1;
	}

	if (isdigit(*argv[1])) {
		dot1p_up = atoi(argv[1]);
	} else {
		print_err(print, "Unrecognized 802.1p UP %s, "
				"Supported 802.1p UP range: 0-7\n", argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dscp_8021p_map(the_interface, argv[0], dot1p_up);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define QCSAPI_BINARY_CONVERT_MASK	0x20

static int
call_qcsapi_wifi_get_dscp_ac_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int i;
	int statval = 0;
	int qcsapi_retval = 0;
	struct qcsapi_data_64bytes ac_mapping;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *acstr[] = { "AC_BE", "AC_BK", "AC_VI", "AC_VO" };

	assert(sizeof(ac_mapping) >= IP_DSCP_NUM);

	memset(&ac_mapping, 0, sizeof(ac_mapping));
	qcsapi_retval = qcsapi_wifi_get_dscp_ac_map(the_interface, &ac_mapping);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "DSCP            AC\n");
			for (i = 0; i < IP_DSCP_NUM; i++) {
				uint8_t mask = QCSAPI_BINARY_CONVERT_MASK;
				/* Print DSCP in binary format */
				while (mask) {
					print_out(print, "%d", i & mask ? 1 : 0);
					mask >>= 1;
				}
				print_out(print, "(0x%02x)    %s\n", i,
						acstr[(uint8_t) ac_mapping.data[i]]);
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

/*
 * Convert given formatted dscp string into digital value
 * Two types of formatted dscp string are acceptable
 * eg,
 * TYPE I  -- 3,4,5,25,38
 * TYPE II -- 3-25
*/
static int call_qcsapi_convert_ipdscp_digital(const char *dscpstr, uint8_t *array,
		uint8_t *number)
{
	uint8_t ip_dscp_number = 0;
	char *pcur;
	char *p;
	char buffer[256] = { 0 };

	if (dscpstr == NULL || array == NULL || number == NULL)
		return -EINVAL;

	strncpy(buffer, dscpstr, (sizeof(buffer) - 1));
	pcur = buffer;
	do {
		p = strchr(pcur, '-');
		if (p) {
			uint8_t dscpstart;
			uint8_t dscpend;
			int i;

			*p = '\0';
			p++;
			if (!isdigit(*pcur) || !isdigit(*p))
				return -EINVAL;
			dscpstart = atoi(pcur);
			dscpend = atoi(p);

			if ((dscpstart > dscpend) || (dscpstart >= IP_DSCP_NUM)
					|| (dscpend >= IP_DSCP_NUM))
				return -EINVAL;
			ip_dscp_number = dscpend - dscpstart;
			for (i = 0; i <= ip_dscp_number; i++)
				array[i] = dscpstart + i;
			break;
		} else {
			if (ip_dscp_number > (IP_DSCP_NUM - 1))
				return -EINVAL;

			p = strchr(pcur, ',');
			if (p) {
				*p = '\0';
				p++;
				array[ip_dscp_number] = atoi(pcur);
				if (array[ip_dscp_number] >= IP_DSCP_NUM)
					return -EINVAL;
				pcur = p;
				ip_dscp_number++;
			} else {
				array[ip_dscp_number] = atoi(pcur);
				if (array[ip_dscp_number] >= IP_DSCP_NUM)
					return -EINVAL;
			}
		}
	} while (p);
	*number = ip_dscp_number + 1;

	return 0;
}

static int
call_qcsapi_wifi_set_dscp_ac_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t listlen = 0;
	uint8_t ac = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_64bytes ip_dscp_value;

	if (argc != 2) {
		print_err(print, "Usage: call_qcsapi set_dscp_ac_map <WiFi interface> "
				"<IP DSCP list> <ac>\n");
		return 1;
	}

	if (!isdigit(*argv[1])) {
		print_err(print, "Unrecognized AC value %s; Supported AC range: 0-3\n", argv[1]);
		return 1;
	} else {
		ac = atoi(argv[1]);
	}

	memset(&ip_dscp_value, 0, sizeof(ip_dscp_value));
	statval = call_qcsapi_convert_ipdscp_digital(argv[0], ip_dscp_value.data, &listlen);
	if (statval < 0)
		return statval;

	qcsapi_retval = qcsapi_wifi_set_dscp_ac_map(the_interface, &ip_dscp_value, listlen, ac);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void
qcsapi_print_dscp_table(qcsapi_output *print, const char *header,
		int convert_to_ac_str, struct qcsapi_data_64bytes *table)
{
	int dscp;
	const char *acstr[] = { "AC_BE", "AC_BK", "AC_VI", "AC_VO" };

	print_out(print, header);
	for (dscp = 0; dscp < IP_DSCP_NUM; dscp++) {
		uint8_t mask = QCSAPI_BINARY_CONVERT_MASK;
		/* Print DSCP in binary format */
		while (mask) {
			print_out(print, "%d", dscp & mask ? 1 : 0);
			mask >>= 1;
		}
		print_out(print, "(0x%02x)    ", dscp);
		if (convert_to_ac_str)
			print_out(print, "%s\n", acstr[table->data[dscp]]);
		else
			print_out(print, "%u\n", table->data[dscp]);
	}
}

static int
qcsapi_validate_dscp_table_index(qcsapi_output *print, const char *str, uint32_t *value)
{
	if (qcsapi_str_to_uint32(str, value) < 0) {
		print_err(print, "Unrecognized table index %s; Supported table index: 0-7\n", str);
		return -1;
	}

	return 0;
}

static int qcsapi_validate_ac_value(qcsapi_output *print, const char *str, uint32_t *value)
{
	if (qcsapi_str_to_uint32(str, value) < 0) {
		print_err(print, "Unrecognized AC value %s; Supported AC range: 0-3\n", str);
		return -1;
	}

	return 0;
}

static int qcsapi_validate_tid_value(qcsapi_output *print, const char *str, uint32_t *value)
{
	if (qcsapi_str_to_uint32(str, value) < 0) {
		print_err(print, "Unrecognized TID value %s; Supported TID range: 0-7\n", str);
		return -1;
	}

	return 0;
}

static int
call_qcsapi_wifi_get_dscp_ac_table(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	uint32_t tsel;
	int statval = 0;
	int qcsapi_retval = 0;
	struct qcsapi_data_64bytes dscp_table = { {0} };
	qcsapi_output *print = p_calling_bundle->caller_output;

	COMPILE_TIME_ASSERT(sizeof(dscp_table) >= IP_DSCP_NUM);

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<table index>");
		return 1;
	}

	if (qcsapi_validate_dscp_table_index(print, argv[0], &tsel))
		return 1;

	qcsapi_retval = qcsapi_wifi_get_dscp_ac_table(tsel, &dscp_table);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			qcsapi_print_dscp_table(print, "DSCP            AC\n", 1, &dscp_table);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dscp_ac_table(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t listlen = 0;
	uint32_t ac = 0;
	uint32_t tsel = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_64bytes ip_dscp_value = { {0} };

	if (argc != 3) {
		qcsapi_report_usage(p_calling_bundle, "<table index> <IP DSCP list> <ac>");
		return 1;
	}

	if (qcsapi_validate_dscp_table_index(print, argv[0], &tsel))
		return 1;

	statval = call_qcsapi_convert_ipdscp_digital(argv[1], ip_dscp_value.data, &listlen);
	if (statval < 0) {
		print_err(print, "invalid DSCP list\n");
		return statval;
	}

	if (qcsapi_validate_ac_value(print, argv[2], &ac))
		return 1;

	qcsapi_retval = qcsapi_wifi_set_dscp_ac_table(tsel, &ip_dscp_value, listlen, ac);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_dscp_tid_table(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	uint32_t tsel;
	int statval = 0;
	int qcsapi_retval = 0;
	struct qcsapi_data_64bytes dscp_table = { {0} };
	qcsapi_output *print = p_calling_bundle->caller_output;

	COMPILE_TIME_ASSERT(sizeof(dscp_table) >= IP_DSCP_NUM);

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<table index>");
		return 1;
	}

	if (qcsapi_validate_dscp_table_index(print, argv[0], &tsel))
		return 1;

	qcsapi_retval = qcsapi_wifi_get_dscp_tid_table(tsel, &dscp_table);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			qcsapi_print_dscp_table(print, "DSCP            TID\n", 0, &dscp_table);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dscp_tid_table(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t listlen = 0;
	uint32_t tid = 0;
	uint32_t tsel = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_64bytes ip_dscp_value = { {0} };

	if (argc != 3) {
		qcsapi_report_usage(p_calling_bundle, "<table index> <IP DSCP list> <TID>");
		return 1;
	}

	if (qcsapi_validate_dscp_table_index(print, argv[0], &tsel))
		return 1;

	statval = call_qcsapi_convert_ipdscp_digital(argv[1], ip_dscp_value.data, &listlen);
	if (statval < 0) {
		print_err(print, "invalid DSCP list\n");
		return statval;
	}

	if (qcsapi_validate_tid_value(print, argv[2], &tid))
		return 1;

	qcsapi_retval = qcsapi_wifi_set_dscp_tid_table(tsel, &ip_dscp_value, listlen, tid);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_dscp_vap_link(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	uint8_t tsel;
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_dscp_vap_link(the_interface, &tsel);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "VAP %s is using DSCP2UP mapping table index: %d.\n",
					the_interface, tsel);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dscp_vap_link(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t tsel = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi set_dscp_ac_map <WiFi interface> "
				"<DSCP2UP table index>\n");
		return 1;
	}

	if (!isdigit(*argv[0])) {
		print_err(print, "Unrecognized DSCP2UP table index value %s; Supported index range: 0-7\n", argv[0]);
		return 1;
	} else {
		tsel = atoi(argv[0]);
	}

	qcsapi_retval = qcsapi_wifi_set_dscp_vap_link(the_interface, tsel);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_ac_agg_hold_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t ac = 0;
	uint32_t agg_hold_time = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *usage = "<WiFi interface> <ac>\n";

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &ac) < 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_ac_agg_hold_time(the_interface, ac, &agg_hold_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", agg_hold_time);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ac_agg_hold_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t ac = 0;
	uint32_t agg_hold_time = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *usage = "<WiFi interface> <ac> <agg_hold_time>\n";

	if (argc != 2) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &ac) < 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[1], &agg_hold_time) < 0) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ac_agg_hold_time(the_interface, ac, agg_hold_time);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_qos_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *ifname = p_calling_bundle->caller_interface;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <QoS Map String>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_qos_map(ifname, argv[0]);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_del_qos_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *ifname = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_del_qos_map(ifname);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_qos_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *ifname = p_calling_bundle->caller_interface;
	string_256 value_buf = { 0 };

	qcsapi_retval = qcsapi_wifi_get_qos_map(ifname, value_buf);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, value_buf);
}

static int
call_qcsapi_wifi_send_qos_map_conf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *ifname = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mac_addr sta_mac_addr = { 0 };

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <STA MAC address>\n");
		return 1;
	}

	if (parse_mac_addr(argv[0], sta_mac_addr)) {
		print_err(print, "\"%s\" is not a valid MAC address\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_send_qos_map_conf(ifname, sta_mac_addr);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_dscp_tid_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *ifname = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_64bytes dscp2tid = { {0} };
	int dscp;

	qcsapi_retval = qcsapi_wifi_get_dscp_tid_map(ifname, &dscp2tid);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_out(print, "DSCP: TID\n");
	for (dscp = 0; dscp < ARRAY_SIZE(dscp2tid.data); dscp++) {
		print_out(print, "%4d: %u\n", dscp, dscp2tid.data[dscp]);
	}

	return 0;
}

static int
call_qcsapi_wifi_get_priority(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t priority;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_priority(the_interface, &priority);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", priority);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_priority_usage(qcsapi_output *print)
{
	print_err(print, "Usage: call_qcsapi set_priority <WiFi interface> <priority>\n");
	print_err(print, "Priority is an integer from 0 to %u.\n", QTN_VAP_PRIORITY_NUM - 1);
}

static int
call_qcsapi_wifi_set_priority(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t priority = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		call_qcsapi_wifi_set_priority_usage(print);
		return 1;
	}

	if (isdigit(*argv[0])) {
		priority = atoi(argv[0]);
		if (priority >= QTN_VAP_PRIORITY_NUM) {
			call_qcsapi_wifi_set_priority_usage(print);
			return 1;
		}
	} else {
		call_qcsapi_wifi_set_priority_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_priority(the_interface, priority);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_airfair(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t airfair;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_airfair(the_interface, &airfair);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", airfair);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_airfair_usage(qcsapi_output *print)
{
	print_err(print, "Usage: call_qcsapi set_airfair <WiFi interface> <status>\n");
	print_err(print, "Status is either 0(disabled) or 1(enabled).\n");
}

static int
call_qcsapi_wifi_set_airfair(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t airfair = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		call_qcsapi_wifi_set_airfair_usage(print);
		return 1;
	}

	if (isdigit(*argv[0])) {
		airfair = atoi(argv[0]);
		if (airfair > 1) {
			call_qcsapi_wifi_set_airfair_usage(print);
			return 1;
		}
	} else {
		call_qcsapi_wifi_set_airfair_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_airfair(the_interface, airfair);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_airquota(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t airquota;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_airquota(the_interface, &airquota);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", airquota);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_airquota_usage(qcsapi_output *print)
{
	print_err(print, "Usage: call_qcsapi set_airquota <WiFi interface> <quota>\n");
	print_err(print, "Quota unit is 1 thousandth. It ranges from 0 to 1000.\n");
}

static int
call_qcsapi_wifi_set_airquota(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t airquota = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		call_qcsapi_wifi_set_airquota_usage(print);
		return 1;
	}

	if (isdigit(*argv[0])) {
		airquota = atoi(argv[0]);
		if (airquota > QTN_QOS_AIRQUOTA_MAX) {
			call_qcsapi_wifi_set_airquota_usage(print);
			return 1;
		}
	} else {
		call_qcsapi_wifi_set_airquota_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_airquota(the_interface, airquota);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_airquota_node(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval = 0;
	int statval = 0;
	string_256 buf = { 0 };
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	retval = qcsapi_wifi_get_airquota_node(the_interface, buf, sizeof(buf));

	if (retval >= 0) {
		print_out(print, "%s\n", buf);
	} else {
		report_qcsapi_error(p_calling_bundle, retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_airquota_node(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	uint16_t airquota = 0;
	qcsapi_mac_addr mac_addr;
	int qcsapi_retval = 0;

	if ((argc < 1) || (argc > 2)) {
		return 1;
	}

	if (strncasecmp("null", argv[0], strlen("null")) == 0) {
		memset(mac_addr, 0, MAC_ADDR_SIZE);
		qcsapi_retval = qcsapi_wifi_set_airquota_node(the_interface, mac_addr, 0);
	} else if (argc == 2) {
		qcsapi_retval = parse_mac_addr(argv[0], mac_addr);
		if (qcsapi_retval < 0) {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}

		if (safe_atou16(argv[1], &airquota, print, 0, QTN_QOS_AIRQUOTA_MAX) <= 0) {
			statval = 1;
			qcsapi_retval = -qcsapi_param_value_invalid;
		}

		if (qcsapi_retval >= 0) {
			qcsapi_retval = qcsapi_wifi_set_airquota_node(the_interface,
					mac_addr, airquota);
		}
	} else {
		print_err(print, "Usage: call_qcsapi set_airquota_node <WiFi interface>"
				" {<mac address> | NULL} [ <quota> ]\n");
		qcsapi_retval = -qcsapi_param_value_invalid;
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_premier_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_mac_addr_list the_mac_addr_list;
	int qcsapi_retval = 0;
	int count = 0;
	int i;
	unsigned char *pmac;

	qcsapi_retval = qcsapi_wifi_get_premier_list(the_interface, &count, the_mac_addr_list);

	if (qcsapi_retval >= 0) {
		for (i = 0; i < count; i++) {
			pmac = ((unsigned char *)the_mac_addr_list) + i * MAC_ADDR_SIZE;
			print_out(print, MACSTR "\n", MAC2STR(pmac));
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_premier_list_usage(qcsapi_output *print)
{
	print_err(print, "Usage: call_qcsapi set_premier_list <WiFi interface> <MAC1> ... <MAC8>\n");
	print_err(print, "Maximum MAC address number is 8\n");
}

static int
call_qcsapi_wifi_set_premier_list(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if ((argc < 1) || (argc > MAC_ADDR_LIST_SIZE)) {
		call_qcsapi_wifi_set_premier_list_usage(print);
		return 1;
	} else {
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_mac_addr_list the_mac_addr_list;
		int qcsapi_retval = 0;
		int count = 0;

		if (strncasecmp("null", argv[0], strlen("null")) == 0)
			qcsapi_retval = qcsapi_wifi_set_premier_list(the_interface, 0, NULL);
		else {
			for (count = 0; count < MIN(argc, MAC_ADDR_LIST_SIZE); count++) {
				qcsapi_retval = parse_mac_addr(argv[count],
						&the_mac_addr_list[count * MAC_ADDR_SIZE]);
				if (qcsapi_retval < 0)
					break;
			}

			if (count > 0) {
				qcsapi_retval = qcsapi_wifi_set_premier_list(the_interface, count,
						the_mac_addr_list);
			} else {
				print_out(print, "Error parsing MAC address %s\n", argv[count]);
				statval = 1;
			}
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static qcsapi_qos_premier_rule string_to_qos_premier_rule(const char *str)
{
	if (strcasecmp(str, "none") == 0) {
		return qcsapi_qos_premier_rule_none;
	} else if (strcasecmp(str, "atf") == 0) {
		return qcsapi_qos_premier_rule_atf;
	} else {
		return qcsapi_qos_premier_nosuch_rule;
	}
}

static char *qos_premier_rule_to_string[qcsapi_qos_premier_rule_num] = {
	"none",
	"atf",
};

static int
call_qcsapi_wifi_get_premier_rule(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_qos_premier_rule rule;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_premier_rule(the_interface, &rule);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", qos_premier_rule_to_string[rule]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_premier_rule_usage(qcsapi_output *print)
{
	qcsapi_qos_premier_rule rule;

	print_err(print, "Usage: call_qcsapi set_premier_rule <WiFi interface> <rule>\n");
	print_err(print, "Valid rules:\n");
	for (rule = 0; rule < qcsapi_qos_premier_rule_num; rule++) {
		print_err(print, "%s\n", qos_premier_rule_to_string[rule]);
	}
}

static int
call_qcsapi_wifi_set_premier_rule(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_qos_premier_rule rule = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		call_qcsapi_wifi_set_premier_rule_usage(print);
		return 1;
	}

	rule = string_to_qos_premier_rule(argv[0]);
	if (rule >= qcsapi_qos_premier_rule_num) {
		call_qcsapi_wifi_set_premier_rule_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_premier_rule(the_interface, rule);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_config_get_parameter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi get_persistent_param <WiFi interface> <parameter name>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *parameter_name = argv[0];
		char *parameter_value;
		size_t parameter_value_size = QCSAPI_MAX_PARAM_VAL_LEN;

		if (strcmp(parameter_name, "NULL") == 0) {
			parameter_name = NULL;
		}

		if (argc > 1) {
			if (strcmp(argv[1], "NULL") == 0) {
				parameter_value = NULL;
			} else {
				if (isdigit(*argv[1])) {
					parameter_value_size = (size_t) atoi(argv[1]);
					if (parameter_value_size > QCSAPI_MAX_PARAM_VAL_LEN) {
						print_err(print, "Parameter length %d is too big\n",
								parameter_value_size);
						return 1;
					}
				} else {
					print_err(print, "Unrecognized param %s, number expected\n",
							argv[1]);
					return 1;
				}
			}
		}

		parameter_value = calloc(parameter_value_size, sizeof(char));
		if (!parameter_value)
			return -ENOMEM;

		qcsapi_retval = qcsapi_config_get_parameter(the_interface,
				parameter_name, parameter_value, parameter_value_size);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", parameter_value);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
		if (parameter_value)
			free(parameter_value);
	}

	return statval;
}

static int
call_qcsapi_config_update_parameter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *parameter_name = argv[0];
	char *parameter_value = argv[1];
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"{<WiFi interface> | global} <parameter name> <value>\n");
		return 1;
	}

	if (strcmp(parameter_name, "NULL") == 0) {
		parameter_name = NULL;
	}

	if (strcmp(parameter_value, "NULL") == 0) {
		parameter_value = NULL;
	}

	qcsapi_retval = qcsapi_config_update_parameter(the_interface, parameter_name,
			parameter_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_config_get_ssid_parameter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi get_persistent_ssid_param <WiFi interface> <parameter name>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *parameter_name = argv[0];
		char *parameter_value;
		size_t parameter_value_size = QCSAPI_MAX_PARAM_VAL_LEN;

		if (strcmp(parameter_name, "NULL") == 0) {
			parameter_name = NULL;
		}

		if (argc > 1) {
			if (strcmp(argv[1], "NULL") == 0) {
				parameter_value = NULL;
			} else {
				if (isdigit(*argv[1])) {
					parameter_value_size = (size_t) atoi(argv[1]);
					if (parameter_value_size > QCSAPI_MAX_PARAM_VAL_LEN) {
						print_err(print, "Parameter length %d is too big\n",
								parameter_value_size);
						return 1;
					}
				} else {
					print_err(print, "Unrecognized param %s, numbers expected\n", argv[1]);
					return 1;
				}
			}
		}

		parameter_value = calloc(parameter_value_size, sizeof(char));
		if (!parameter_value)
			return -ENOMEM;
		qcsapi_retval = qcsapi_config_get_ssid_parameter(the_interface,
				parameter_name, parameter_value, parameter_value_size);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", parameter_value);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
		if (parameter_value)
			free(parameter_value);
	}

	return statval;
}

static int
call_qcsapi_config_update_ssid_parameter(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call_qcsapi update_persistent_ssid_parameter\n");
		print_err(print, "Usage: call_qcsapi update_persistent_ssid_param <WiFi interface> <parameter name> <value>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *parameter_name = argv[0];
		char *parameter_value = argv[1];

		if (strcmp(parameter_name, "NULL") == 0) {
			parameter_name = NULL;
		}

		if (strcmp(parameter_value, "NULL") == 0) {
			parameter_value = NULL;
		}

		qcsapi_retval = qcsapi_config_update_ssid_parameter(the_interface, parameter_name,
				parameter_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_bootcfg_get_parameter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi get bootcfg parameter, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi get_bootcfg_param <parameter name>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		char *parameter_name = argv[0];
		char *param_value_addr;
		size_t parameter_len = QCSAPI_MAX_PARAM_VAL_LEN + 1;

		if (strcmp(parameter_name, "NULL") == 0) {
			parameter_name = NULL;
		}

		if (argc > 1 && strcmp(argv[1], "NULL") == 0) {
			param_value_addr = NULL;
		}

		if (argc > 2 && isdigit(argv[2][0])) {
			parameter_len = atoi(argv[2]);
			if (parameter_len > QCSAPI_MAX_PARAM_VAL_LEN + 1) {
				print_err(print, "Parameter length %d is too big\n",
						parameter_len);
				return 1;
			}
		}

		param_value_addr = calloc(parameter_len, sizeof(char));
		if (!param_value_addr)
			return -ENOMEM;
		qcsapi_retval = qcsapi_bootcfg_get_parameter(parameter_name,
				param_value_addr, parameter_len);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", param_value_addr);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
		if (param_value_addr)
			free(param_value_addr);
	}

	return statval;
}

static int
call_qcsapi_bootcfg_update_parameter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi update bootcfg parameter, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi update_bootcfg_param <parameter name> <value>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		char *parameter_name = argv[0];
		char *param_value_addr = argv[1];

		if (strcmp(parameter_name, "NULL") == 0) {
			parameter_name = NULL;
		}

		if (strcmp(param_value_addr, "NULL") == 0) {
			param_value_addr = NULL;
		}

		qcsapi_retval = qcsapi_bootcfg_update_parameter(parameter_name, param_value_addr);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_bootcfg_commit(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;

	qcsapi_retval = qcsapi_bootcfg_commit();

	if (qcsapi_retval >= 0) {
		print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_service_control(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi service_control, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi service_control <service name> <action>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		char *name = argv[0];
		char *action = argv[1];
		qcsapi_service_name serv_name;
		qcsapi_service_action serv_action;

		if (strcmp(argv[0], "NULL") == 0) {
			name = NULL;
		} else if (strcmp(argv[0], "telnet") == 0) {
			name = "inetd";
		}
		if (strcmp(argv[1], "NULL") == 0) {
			action = NULL;
		}

		qcsapi_retval = qcsapi_get_service_name_enum(name, &serv_name);
		if (qcsapi_retval >= 0) {
			qcsapi_retval = qcsapi_get_service_action_enum(action, &serv_action);
		}

		if (qcsapi_retval >= 0) {
			qcsapi_retval = qcsapi_service_control(serv_name, serv_action);
		}

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}
	return statval;
}

static int call_qcsapi_wfa_cert(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t enable = 1;

	if (argc > 0) {
		if (safe_atou16(argv[0], &enable, print, 0, 1) == 0)
			return 1;
	}

	qcsapi_retval = qcsapi_wfa_cert_mode_enable(! !enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_scs_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t enable = 1;

	if (argc > 0) {
		if (0 == safe_atou16(argv[0], &enable, print, 0, 1))
			return 1;
	}

	qcsapi_retval = qcsapi_wifi_scs_enable(the_interface, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_scs_set_version(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t version = 1;

	if (argc > 0) {
		if (safe_atou16(argv[0], &version, print, 0, 1) == 0)
			return 1;
	}

	qcsapi_retval = qcsapi_wifi_scs_set_version(the_interface, version);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void print_scs_switch_channel_usage(call_qcsapi_bundle *p_calling_bundle)
{
	qcsapi_report_usage(p_calling_bundle,
			"<interface name> <pick flags> [check margin]\n"
			"pick flags should be : dfs, non_dfs, all(default)\n"
			"check margin should be : 0 or 1 which means if we should check the margin");
}

static int
call_qcsapi_wifi_scs_switch_channel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t pick_flags = 0;
	uint16_t check_margin = 0;

	if (argc >= 1 && argc <= 2) {
		if (!strcasecmp("dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY;
		} else if (!strcasecmp("non_dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_NON_DFS_ONLY;
		} else if (!strcasecmp("all", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
		} else {
			print_scs_switch_channel_usage(p_calling_bundle);
			return 1;
		}

		if (argc == 2 && safe_atou16(argv[1], &check_margin, print, 0, 1) == 0)
			return 1;
		else if (check_margin == 0)
			pick_flags |= IEEE80211_SCS_PICK_ANYWAY;
	} else if (argc == 0) {
		pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
	} else {
		print_scs_switch_channel_usage(p_calling_bundle);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_scs_switch_channel(the_interface, pick_flags);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_scs_pick_best_channel(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t pick_flags = IEEE80211_SCS_NOPICK |
			IEEE80211_SCS_PICK_ANYWAY | IEEE80211_SCS_PICK_ALLOW_CURRENT;
	uint16_t check_margin = 0;
	int channel = -1;

	if (argc >= 1 && argc <= 2) {
		if (!strcasecmp("dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_DFS_ONLY;
		} else if (!strcasecmp("non_dfs", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_NON_DFS_ONLY;
		} else if (!strcasecmp("all", argv[0])) {
			pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
		} else {
			print_scs_switch_channel_usage(p_calling_bundle);
			return 1;
		}

		if (argc == 2 && safe_atou16(argv[1], &check_margin, print, 0, 1) == 0)
			return 1;
		else if (check_margin == 1)
			pick_flags &= ~(IEEE80211_SCS_PICK_ANYWAY |
					IEEE80211_SCS_PICK_ALLOW_CURRENT);
	} else if (argc == 0) {
		pick_flags |= IEEE80211_SCS_PICK_AVAILABLE_ANY_CHANNEL;
	} else {
		print_scs_switch_channel_usage(p_calling_bundle);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_scs_pick_best_channel(the_interface, pick_flags, &channel);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", channel);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_scs_verbose(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t enable = 1;

	if (argc > 0) {
		enable = (uint16_t) atoi(argv[0]);
	}

	qcsapi_retval = qcsapi_wifi_set_scs_verbose(the_interface, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int status = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_scs_status(the_interface, &status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (status == 1)
				print_out(print, "Enabled (%d)\n", status);
			else if (status == 0)
				print_out(print, "Disabled (%d)\n", status);
			else
				print_out(print, "Unknown (%d)\n", status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_smpl_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t enable = 1;

	if (argc > 0) {
		enable = (uint16_t) atoi(argv[0]);
	}

	qcsapi_retval = qcsapi_wifi_set_scs_smpl_enable(the_interface, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_wifi_set_scs_active_chan_list_help(call_qcsapi_bundle *p_calling_bundle)
{
	qcsapi_report_usage(p_calling_bundle,
			"<Wifi interface> <channel list> <enable_disable flag>\n");
}

static int
call_qcsapi_wifi_set_scs_active_chan_list(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 1;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_scs_chan_list scs_ch_list;
	uint32_t ch_count;
	uint32_t enable;

	if (argc < 2) {
		call_qcsapi_wifi_set_scs_active_chan_list_help(p_calling_bundle);
		return 1;
	}

	if (safe_atou32(argv[1], &enable, print, 0, 1) == 0) {
		call_qcsapi_wifi_set_scs_active_chan_list_help(p_calling_bundle);
		return 1;
	}

	memset(&scs_ch_list, 0, sizeof(scs_ch_list));
	statval = string_to_list(print, argv[0], scs_ch_list.chan, &ch_count);
	if (statval < 0) {
		call_qcsapi_wifi_set_scs_active_chan_list_help(p_calling_bundle);
		return statval;
	}
	scs_ch_list.num = ch_count;

	qcsapi_retval = qcsapi_wifi_set_scs_active_chan_list(the_interface, &scs_ch_list, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
		statval = 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_active_chan_list(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_scs_chan_list scs_ch_list;
	int i;

	memset(&scs_ch_list, 0, sizeof(scs_ch_list));

	qcsapi_retval = qcsapi_wifi_get_scs_active_chan_list(the_interface, &scs_ch_list);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else {
		print_out(print, "Number of SCS enabled channels: %u\n", scs_ch_list.num);
		for (i = 0; i < scs_ch_list.num; i++)
			print_out(print, "%d ", scs_ch_list.chan[i]);
		print_out(print, "\n");
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_smpl_dwell_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t sample_time = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_out(print, "%s: programming error, expected at least 1 additional parameter\n", __func__);
		return 1;
	}

	if (safe_atou16(argv[0], &sample_time, print,
					IEEE80211_SCS_SMPL_DWELL_TIME_MIN,
					IEEE80211_SCS_SMPL_DWELL_TIME_MAX) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_scs_smpl_dwell_time(the_interface, sample_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_smpl_dwell_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int scs_smpl_dwell_time = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_scs_smpl_dwell_time(the_interface, &scs_smpl_dwell_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", scs_smpl_dwell_time);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_sample_intv(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t sample_intv = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (safe_atou16(argv[0], &sample_intv, print,
					IEEE80211_SCS_SMPL_INTV_MIN,
					IEEE80211_SCS_SMPL_INTV_MAX) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_scs_sample_intv(the_interface, sample_intv);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_sample_type(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	uint16_t sample_type = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (safe_atou16(argv[0], &sample_type, print, 1, 0xFFFF) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_sample_type(the_interface, sample_type);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_scs_sample_intv(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_unsigned_int scs_sample_intv = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_scs_sample_intv(the_interface, &scs_sample_intv);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%u\n", scs_sample_intv);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_intf_detect_intv(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t intv = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (safe_atou16(argv[0], &intv, print,
					IEEE80211_SCS_CCA_DUR_MIN,
					IEEE80211_SCS_CCA_DUR_MAX) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_scs_intf_detect_intv(the_interface, intv);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_thrshld(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call_qcsapi set_scs_thrshld, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_scs_thrshld <Wifi interface> <threshold parameter> <threshold value>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *thrshld_param_name = argv[0];
		uint16_t thrshld_value;

		if (safe_atou16(argv[1], &thrshld_value, print, 0, 0xFFFF) == 0) {
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_scs_thrshld(the_interface, thrshld_param_name,
				thrshld_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_report_only(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{

	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t report_value = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	report_value = atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_set_scs_report_only(the_interface, report_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_report(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int i;

	if (strcmp(argv[0], "current") == 0) {
		struct qcsapi_scs_currchan_rpt rpt;
		qcsapi_retval = qcsapi_wifi_get_scs_currchan_report(the_interface, &rpt);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		} else {
			print_out(print, "SCS: current channel %d, cca_try=%u, cca_idle=%u cca_busy=%u cca_intf=%u" " cca_tx=%u tx_ms=%u rx_ms=%u pmbl_cnt=%u\n", rpt.chan, rpt.cca_try, rpt.cca_idle, rpt.cca_busy, rpt.cca_intf, rpt.cca_tx, rpt.tx_ms, rpt.rx_ms, rpt.pmbl);
		}
	} else if (strcmp(argv[0], "all") == 0) {
		struct qcsapi_scs_ranking_rpt rpt;
		const char *str[] = QTN_CHAN_AVAIL_STATUS_TO_STR;

		qcsapi_retval = qcsapi_wifi_get_scs_stat_report(the_interface, &rpt);

		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		} else {
			print_out(print, "SCS ranking report: chan number = %u\n", rpt.num);
			print_out(print, "chan dfs txpower  numbeacon cca_intf     metric    pmbl_ap   pmbl_sta" "   age duration times status\n");
			for (i = 0; i < rpt.num; i++) {
				print_out(print, "%4d %3d %7d %10u %8u %10d %10d %10d %5u %8u %5u %s\n", rpt.chan[i], rpt.dfs[i], rpt.txpwr[i], rpt.numbeacons[i], rpt.cca_intf[i], rpt.metric[i], rpt.pmbl_ap[i], rpt.pmbl_sta[i], rpt.metric_age[i], rpt.duration[i], rpt.times[i], str[rpt.chan_avail_status[i]]);
			}
		}
	} else if (strcmp(argv[0], "autochan") == 0) {
		struct qcsapi_autochan_rpt rpt;
		qcsapi_retval = qcsapi_wifi_get_autochan_report(the_interface, &rpt);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		} else {
			print_out(print, "AP: initial auto channel ranking table: chan number = %u\n", rpt.num);
			print_out(print, "chan dfs txpower  numbeacon        cci        aci     metric\n");
			for (i = 0; i < rpt.num; i++) {
				print_out(print, "%4d %3d %7d %10u %10d %10d %10d\n",
						rpt.chan[i],
						rpt.dfs[i],
						rpt.txpwr[i],
						rpt.numbeacons[i],
						rpt.cci[i], rpt.aci[i], rpt.metric[i]);
			}
		}
	} else if (strcmp(argv[0], "score") == 0) {
		struct qcsapi_scs_score_rpt rpt;
		qcsapi_retval = qcsapi_wifi_get_scs_score_report(the_interface, &rpt);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		} else {
			print_out(print, "SCS score report: channel number = %u\n", rpt.num);
			print_out(print, "channel  score\n");
			for (i = 0; i < rpt.num; i++) {
				print_out(print, "%4d  %5d\n", rpt.chan[i], rpt.score[i]);
			}
		}
	} else if (strcmp(argv[0], "interference") == 0) {
#define INTF_NUM		6
#define SCS_CCA_INTF_INVALID	0xFFFF
		struct qcsapi_scs_interference_rpt rpt;
		char cca_intf20[INTF_NUM];
		char cca_intf40[INTF_NUM];
		char cca_intf80[INTF_NUM];

		qcsapi_retval = qcsapi_wifi_get_scs_interference_report(the_interface, &rpt);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			return 1;
		}
		print_out(print, "SCS ranking report: chan number = %u\n", rpt.num);
		print_out(print, "chan cca_intf_20 cca_intf_40 cca_intf_80\n");
		for (i = 0; i < rpt.num; i++) {
			snprintf(cca_intf20, INTF_NUM, "%u", rpt.cca_intf_20[i]);
			snprintf(cca_intf40, INTF_NUM, "%u", rpt.cca_intf_40[i]);
			snprintf(cca_intf80, INTF_NUM, "%u", rpt.cca_intf_80[i]);
			print_out(print, "%4d %11s %11s %11s\n",
					rpt.chan[i],
					rpt.cca_intf_20[i] ==
					SCS_CCA_INTF_INVALID ? "-" : cca_intf20,
					rpt.cca_intf_40[i] ==
					SCS_CCA_INTF_INVALID ? "-" : cca_intf40,
					rpt.cca_intf_80[i] ==
					SCS_CCA_INTF_INVALID ? "-" : cca_intf80);
		}
	} else {
		print_err(print, "Invalid parameter:%s\nOptional choice:current all autochan score\n", argv[0]);
		return 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_cca_intf_smth_fctr(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{

	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t fctr_noxp = 0;
	uint8_t fctr_xped = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call_qcsapi set_scs_cca_intf_smth_fctr, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_scs_cca_intf_smth_fctr <Wifi interface> "
				"<factor for never used channel> <factor for used channel>\n");
		statval = 1;
	} else {
		fctr_noxp = atoi(argv[0]);
		fctr_xped = atoi(argv[1]);

		qcsapi_retval = qcsapi_wifi_set_scs_cca_intf_smth_fctr(the_interface, fctr_noxp,
				fctr_xped);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t start = 1;

	if (argc > 0) {
		start = (uint16_t) atoi(argv[0]);
	}

	qcsapi_retval = qcsapi_wifi_set_scs_stats(the_interface, start);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_burst_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t enable_flag = 0;

	if (safe_atou16(argv[0], &enable_flag, print,
					IEEE80211_SCS_BURST_ENABLE_MIN,
					IEEE80211_SCS_BURST_ENABLE_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_burst_enable(the_interface, enable_flag);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_burst_window(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t window = 0;

	if (safe_atou16(argv[0], &window, print,
					IEEE80211_SCS_BURST_WINDOW_MIN,
					IEEE80211_SCS_BURST_WINDOW_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_burst_window(the_interface, window);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_burst_thresh(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t threshold = 0;

	if (safe_atou16(argv[0], &threshold, print,
					IEEE80211_SCS_BURST_THRESH_MIN,
					IEEE80211_SCS_BURST_THRESH_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_burst_thresh(the_interface, threshold);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_burst_pause(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t pause_time = 0;

	if (safe_atou16(argv[0], &pause_time, print,
					IEEE80211_SCS_BURST_PAUSE_MIN,
					IEEE80211_SCS_BURST_PAUSE_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_burst_pause(the_interface, pause_time);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_burst_switch(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t switch_flag = 0;

	if (safe_atou16(argv[0], &switch_flag, print,
					IEEE80211_SCS_BURST_SWITCH_MIN,
					IEEE80211_SCS_BURST_SWITCH_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_burst_switch(the_interface, switch_flag);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_chan_pool(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	char *p_list_channels = NULL;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	static string_1024 the_list_channels;

	p_list_channels = &the_list_channels[0];
	memset(p_list_channels, 0, sizeof(the_list_channels));

	qcsapi_retval = qcsapi_wifi_get_scs_chan_pool(the_interface, p_list_channels);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%s\n", &the_list_channels[0]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_chan_pool(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	uint32_t listlen = 0;
	struct qcsapi_data_256bytes chan_list;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <channel list>\n");
		return 1;
	}

	memset(&chan_list, 0, sizeof(chan_list));
	statval = string_to_list(print, argv[0], chan_list.data, &listlen);
	if (statval < 0)
		return statval;

	qcsapi_retval = qcsapi_wifi_set_scs_chan_pool(the_interface, &chan_list, listlen);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_vendor_fix(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	int fix_param = -1;
	int param_value = -1;
	int i;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_vendor_fix <WiFi interface> "
				"<fix-param> <value>\n");
		return 1;
	}

	if (name_to_vendor_fix_idx(argv[0], &fix_param) == 0) {
		print_err(print, "Unrecognized vendor fix param %s\n", argv[0]);
		if (verbose_flag >= 0) {
			print_out(print, "Supported vendor fix param:\n");
			for (i = 0; i < ARRAY_SIZE(qcsapi_vendor_fix_table); i++)
				print_out(print, "%s\n", qcsapi_vendor_fix_table[i].fix_name);
		}
		return 1;
	}

	if (isdigit(*argv[1])) {
		param_value = atoi(argv[1]);
	} else {
		print_err(print, "Unrecognized vendor fix's value %s\n", argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_vendor_fix(the_interface, fix_param, param_value);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_chan_mtrc_mrgn(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{

	int statval = 0;
	int qcsapi_retval = 0;
	uint8_t value = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call_qcsapi set_scs_chan_mtrc_mrgn, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_scs_chan_mtrc_mrgn <Wifi interface> "
				"<channel metric margin>\n");
		statval = 1;
	} else {
		value = atoi(argv[0]);

		qcsapi_retval = qcsapi_wifi_set_scs_chan_mtrc_mrgn(the_interface, value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_nac_monitor_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t enable;
	uint32_t on_period = MONITOR_DEFAULT_ON_PERIOD * 100 / MONITOR_DEFAULT_CYCLE_PERIOD;
	uint32_t cycle_period = MONITOR_DEFAULT_CYCLE_PERIOD;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 3 && argc != 1) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> {0 | 1} [<on period> <cycle period>]");
		return 1;
	}

	if (safe_atou32(argv[0], &enable, print, 0, 1) == 0)
		return 1;

	if (argc == 3) {
		if (safe_atou32(argv[1], &on_period, print, MONITOR_MIN_ON_PERIOD,
						MONITOR_MAX_ON_PERIOD) == 0)
			return 1;

		if (safe_atou32(argv[2], &cycle_period, print, MONITOR_MIN_CYCLE_PERIOD,
						MONITOR_MAX_CYCLE_PERIOD) == 0)
			return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_scs_nac_monitor_mode(the_interface, enable, on_period,
			cycle_period);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_band_margin_check(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t enable;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> {0 | 1}");
		return 1;
	}

	if (safe_atou16(argv[0], &enable, print, 0, 1) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_band_margin_check(the_interface, enable);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scs_band_margin(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t margin;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <margin>");
		return 1;
	}

	if (safe_atou16(argv[0], &margin, print, 0, IEEE80211_SCS_OUT_OF_BAND_MTRC_MRGN_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_set_scs_band_margin(the_interface, margin);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_dfs_reentry_request(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int status = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_scs_dfs_reentry_request(the_interface, &status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_cca_intf(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi to get scs cca interference\n");
		print_err(print, "Usage: call_qcsapi get_scs_cca_intf <interface> <channel>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_unsigned_int the_channel = atoi(argv[0]);
		int cca_intf = 0;

		qcsapi_retval = qcsapi_wifi_get_scs_cca_intf(the_interface, the_channel, &cca_intf);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%d\n", cca_intf);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scs_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int len = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_scs_param_rpt *p_rpt = NULL;
	struct qcsapi_data_1Kbytes *p_rpt_buf = NULL;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_1Kbytes) >= (sizeof(*p_rpt) * SCS_PARAM_MAX));

	len = sizeof(*p_rpt_buf);
	p_rpt_buf = (struct qcsapi_data_1Kbytes *)malloc(len);
	if (p_rpt_buf == NULL) {
		print_err(print, "malloc failed - %s\n", __func__);
		return 1;
	}

	memset(p_rpt_buf, 0, len);
	qcsapi_retval = qcsapi_wifi_get_scs_params(the_interface, p_rpt_buf, len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			p_rpt = (qcsapi_scs_param_rpt *) p_rpt_buf;
			dump_scs_param(print, p_rpt);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	free(p_rpt_buf);
	p_rpt_buf = NULL;
	p_rpt = NULL;

	return statval;
}

void display_acs_param_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "    call_qcsapi set_acs_param <interface> <acs_param> <value>\n");
	print_out(print, "\n");
	print_out(print, "Parameter\t\t\tValue\t\t\t# Notes\n");
	print_out(print, "    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n",
			acs_config[qcsapi_acs_param_obss_chk].parameter_name);
}

static int name_to_acs_param_enum(const char *parameter, qcsapi_acs_param_type *acs_param_type)
{
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(acs_config); iter++) {
		if (strcmp(acs_config[iter].parameter_name, parameter) == 0) {
			*acs_param_type = acs_config[iter].param_type;
			return 1;
		}
	}

	return -1;
}

static int call_qcsapi_set_acs_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *ifname = p_calling_bundle->caller_interface;;
	char *parameter = NULL;
	char *param_value_str = NULL;
	uint32_t param_value = 0;
	qcsapi_acs_param_type acs_param;

	if ((argv[0]) && (strcmp(argv[0], "--help") == 0)) {
		display_acs_param_usage(print);
		return 0;
	}

	if ((argc != 2)) {
		display_acs_param_usage(print);
		return 1;
	}

	parameter = argv[0];
	param_value_str = argv[1];

	if (name_to_acs_param_enum(parameter, &acs_param) < 0) {
		display_acs_param_usage(print);
		return 1;
	}

	param_value = atoi(param_value_str);
	qcsapi_retval = qcsapi_set_acs_params(ifname, acs_param, param_value);
	if (qcsapi_retval == -qcsapi_param_value_invalid)
		print_out(print, "Invalid value. 'call_qcsapi set_acs_param --help'"
				" to find proper value\n");

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_start_ocac(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t channel_value = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi start_ocac wifi0 {auto | <DFS channel>}\n");
		return 1;
	}

	/* parameter parse */
	if (!strcasecmp("auto", argv[0])) {
		channel_value = 0;
	} else {
		if (safe_atou16(argv[0], &channel_value, print, 0, 0xFFFF) == 0) {
			return 1;
		}
	}

	qcsapi_retval = qcsapi_wifi_start_ocac(the_interface, channel_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_stop_ocac(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_stop_ocac(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_ocac_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int status = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_ocac_status(the_interface, &status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (status == 1)
				print_out(print, "Enabled\n");
			else if (status == 0)
				print_out(print, "Disabled\n");
			else
				print_out(print, "Unknown (%u)\n", status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ocac_dwell_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t dwell_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n" "  call_qcsapi set_ocac_dwell_time wifi0 <msecs>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &dwell_time, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ocac_dwell_time(the_interface, dwell_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ocac_duration(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t duration = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n" "  call_qcsapi set_ocac_duration wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &duration, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ocac_duration(the_interface, duration);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ocac_cac_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t cac_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n" "  call_qcsapi set_ocac_cac_time wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &cac_time, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ocac_cac_time(the_interface, cac_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ocac_report_only(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t value = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n" "  call_qcsapi set_ocac_report_only wifi0 {0 | 1}\n");
		return 1;
	}

	if (safe_atou16(argv[0], &value, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ocac_report_only(the_interface, value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ocac_threshold(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_ocac_thrshld <WiFi interface> <threshold parameter> <threshold value>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *thrshld_param_name = argv[0];
		uint16_t thrshld_value;

		if (safe_atou16(argv[1], &thrshld_value, print, 0, 0xFFFF) == 0) {
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_ocac_thrshld(the_interface, thrshld_param_name,
				thrshld_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_start_dfs_s_radio(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t channel_value = 0;
	uint16_t sdfs_auto_param = 0;
	int16_t channel_idx = -1;

	if ((argc < 1) || (argc > 2)) {
		statval = 1;
		goto usage;
	}

	if (!strcasecmp("auto", argv[0]))
		sdfs_auto_param = 1;

	channel_idx = (sdfs_auto_param && (argc == 2)) ? 1 : (((!sdfs_auto_param)
					&& (argc == 1)) ? 0 : -1);

	if ((channel_idx >= 0) && (safe_atou16(argv[channel_idx], &channel_value, print,
							QCSAPI_ANY_CHANNEL,
							QCSAPI_MAX_CHANNEL) == 0)) {
		if (sdfs_auto_param == 0) {
			/* call_qcsapi start_dfs_s_radio wifi0_0 */
			statval = 1;
			goto usage;
		} else if (argc == 2) {
			/* call_qcsapi start_dfs_s_radio wifi0_0 auto */
			statval = 1;
			goto usage;
		}
		/* call_qcsapi start_dfs_s_radio wifi0_0 auto */
	}

	/* call_qcsapi start_dfs_s_radio <DFS channel> */
	if (argc == 2) {
		if (sdfs_auto_param) {
			/* call_qcsapi start_dfs_s_radio auto <DFS channel> */
			channel_value |= IEEE80211_OCAC_AUTO_WITH_FIRST_DFS_CHAN;
		} else {
			/* call_qcsapi start_dfs_s_radio <DFS channel> */
			statval = 1;
			goto usage;
		}
	}

	qcsapi_retval = qcsapi_wifi_start_dfs_s_radio(the_interface, channel_value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

usage:
	qcsapi_report_usage(p_calling_bundle,
			"<Wifi interface> { {auto | <test_DFS_channel> } | auto <first_DFS_channel>}\n");

	return statval;
}

static int
call_qcsapi_wifi_stop_dfs_s_radio(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_stop_dfs_s_radio(the_interface);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dfs_s_radio_status(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int status = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_dfs_s_radio_status(the_interface, &status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (status == 1)
				print_out(print, "Enabled\n");
			else if (status == 0)
				print_out(print, "Disabled\n");
			else
				print_out(print, "Unknown (%u)\n", status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dfs_s_radio_availability(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int available = 0;

	qcsapi_retval = qcsapi_wifi_get_dfs_s_radio_availability(the_interface, &available);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (available == 1)
				print_out(print, "Available\n");
			else
				print_out(print, "Unavailable\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_dwell_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t dwell_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_dwell_time wifi0 <msecs>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &dwell_time, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_dwell_time(the_interface, dwell_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_duration(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t duration = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_duration wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &duration, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_duration(the_interface, duration);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_cac_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t cac_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_cac_time wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &cac_time, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_cac_time(the_interface, cac_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_report_only(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t value = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_report_only wifi0 {0 | 1}\n");
		return 1;
	}

	if (safe_atou16(argv[0], &value, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_report_only(the_interface, value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_wea_duration(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t duration = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_wea_duration wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou32(argv[0], &duration, print, 0, 0xFFFFFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_wea_duration(the_interface, duration);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_wea_cac_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t cac_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_wea_cac_time wifi0 <seconds>\n");
		return 1;
	}

	if (safe_atou32(argv[0], &cac_time, print, 0, 0xFFFFFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_wea_cac_time(the_interface, cac_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t dwell_time = 0;

	if (argc < 1) {
		print_out(print, "Usage:\n"
				"  call_qcsapi set_dfs_s_radio_wea_dwell_time wifi0 <msecs>\n");
		return 1;
	}

	if (safe_atou16(argv[0], &dwell_time, print, 0, 0xFFFF) == 0) {
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_wea_dwell_time(the_interface, dwell_time);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_dfs_s_radio_threshold(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi set_dfs_s_radio_thrshld <WiFi interface> <threshold parameter> <threshold value>\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		const char *the_interface = p_calling_bundle->caller_interface;
		char *thrshld_param_name = argv[0];
		uint16_t thrshld_value;

		if (safe_atou16(argv[1], &thrshld_value, print, 0, 0xFFFF) == 0) {
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_dfs_s_radio_thrshld(the_interface,
				thrshld_param_name, thrshld_value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ap_isolate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int current_ap_isolate_status;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	char *endptr;

	if (argc < 1) {
		print_err(print, "Parameter count incorrect. Should be 3, is %d\n", argc);
		statval = 1;
	} else {
		endptr = NULL;
		current_ap_isolate_status = strtol(argv[0], &endptr, 10);
		if (!endptr || (*endptr != 0) || ((endptr - 1) != argv[0])) {
			print_err(print, "Invalid isolation settting. Should be 0 or 1\n");
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_wifi_set_ap_isolate(the_interface,
					current_ap_isolate_status);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_ap_isolate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int current_ap_isolate_status = (int)qcsapi_ap_isolate_disabled;
	int *p_current_ap_isolate_status = NULL;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_current_ap_isolate_status = &current_ap_isolate_status;
	}

	qcsapi_retval = qcsapi_wifi_get_ap_isolate(the_interface, p_current_ap_isolate_status);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", current_ap_isolate_status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_interface_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_interface_stats stats;

	qcsapi_retval = qcsapi_get_interface_stats(the_interface, &stats);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (sizeof(long) == 8) {
				print_out(print, "tx_bytes:\t%llu\n"
						"tx_pkts:\t%u\n"
						"tx_discard:\t%u\n"
						"tx_err:\t\t%u\n"
						"tx_unicast:\t%u\n"
						"tx_multicast:\t%u\n"
						"tx_broadcast:\t%u\n"
						"rx_bytes:\t%llu\n"
						"rx_pkts:\t%u\n"
						"rx_discard:\t%u\n"
						"rx_err:\t\t%u\n"
						"rx_unicast:\t%u\n"
						"rx_multicast:\t%u\n"
						"rx_broadcast:\t%u\n"
						"rx_unknown:\t%u\n",
						stats.tx_bytes,
						stats.tx_pkts,
						stats.tx_discard,
						stats.tx_err,
						stats.tx_unicast,
						stats.tx_multicast,
						stats.tx_broadcast,
						stats.rx_bytes,
						stats.rx_pkts,
						stats.rx_discard,
						stats.rx_err,
						stats.rx_unicast,
						stats.rx_multicast,
						stats.rx_broadcast, stats.rx_unknown);
			} else {
				print_out(print, "tx_bytes:\t%llu\n"
						"tx_pkts:\t%lu\n"
						"tx_discard:\t%lu\n"
						"tx_err:\t\t%lu\n"
						"tx_unicast:\t%lu\n"
						"tx_multicast:\t%lu\n"
						"tx_broadcast:\t%lu\n"
						"rx_bytes:\t%llu\n"
						"rx_pkts:\t%lu\n"
						"rx_discard:\t%lu\n"
						"rx_err:\t\t%lu\n"
						"rx_unicast:\t%lu\n"
						"rx_multicast:\t%lu\n"
						"rx_broadcast:\t%lu\n"
						"rx_unknown:\t%lu\n",
						stats.tx_bytes,
						stats.tx_pkts,
						stats.tx_discard,
						stats.tx_err,
						stats.tx_unicast,
						stats.tx_multicast,
						stats.tx_broadcast,
						stats.rx_bytes,
						stats.rx_pkts,
						stats.rx_discard,
						stats.rx_err,
						stats.rx_unicast,
						stats.rx_multicast,
						stats.rx_broadcast, stats.rx_unknown);

			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_vap_extstats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_vap_extstats stats;

	qcsapi_retval = qcsapi_get_vap_extstats(the_interface, &stats);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "ssid_RetransCount:\t%u\n"
					"ssid_FailedRetransCount:\t%u\n"
					"ssid_RetryCount:\t%u\n"
					"ssid_MultipleRetryCount:\t%u\n"
					"ssid_AggregatedPacketCount:\t%u\n"
					"ssid_ACKFailureCount:\t%u\n",
					stats.tx_retrans,
					stats.tx_fretrans,
					stats.tx_retries,
					stats.tx_mretries, stats.tx_aggpkts, stats.tx_toutpkts);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int call_qcsapi_get_phy_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int iter;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_phy_stats stats;

	qcsapi_retval = qcsapi_get_phy_stats(the_interface, &stats);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "tstamp=\t\t%u\n"
					"assoc=\t\t%u\n"
					"channel=\t%u\n"
					"attenuation=\t%u\n"
					"cca_total=\t%u\n"
					"cca_tx=\t\t%u\n"
					"cca_rx=\t\t%u\n"
					"cca_int=\t%u\n"
					"cca_idle\t%u\n"
					"rx_pkts=\t%u\n"
					"last_rx_pkt_timestamp=\t%u\n"
					"rx_gain=\t%u\n"
					"rx_cnt_crc=\t%u\n"
					"rx_noise=\t%5.1f\n"
					"tx_pkts=\t%u\n"
					"last_tx_pkt_timestamp=\t%u\n"
					"tx_defers=\t%d\n"
					"tx_touts=\t%u\n"
					"tx_retries=\t%u\n"
					"cnt_sp_fail=\t%u\n"
					"cnt_lp_fail=\t%u\n"
					"last_rx_mcs=\t%d\n"
					"last_tx_mcs=\t%d\n",
					stats.tstamp,
					stats.assoc,
					stats.channel,
					stats.atten,
					stats.cca_total,
					stats.cca_tx,
					stats.cca_rx,
					stats.cca_int,
					stats.cca_idle,
					stats.rx_pkts,
					stats.last_rx_pkt_timestamp,
					stats.rx_gain,
					stats.rx_cnt_crc,
					stats.rx_noise,
					stats.tx_pkts,
					stats.last_tx_pkt_timestamp,
					stats.tx_defers,
					stats.tx_touts,
					stats.tx_retries,
					stats.cnt_sp_fail,
					stats.cnt_lp_fail, stats.last_rx_mcs, stats.last_tx_mcs);
			print_out(print, "last_evm=\t%5.1f\n", stats.last_evm);
			for (iter = 0; iter < QCSAPI_QDRV_NUM_RF_STREAMS; iter++) {
				print_out(print, "last_evm_%d=\t%5.1f\n", iter,
						stats.last_evm_array[iter]);
			}

			print_out(print, "last_rcpi=\t%5.1f\n", stats.last_rcpi);

			print_out(print, "last_rssi=\t%5.1f\n", stats.last_rssi);
			for (iter = 0; iter < QCSAPI_QDRV_NUM_RF_STREAMS; iter++) {
				print_out(print, "last_rssi_%d=\t%5.1f\n", iter,
						stats.last_rssi_array[iter]);
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_telnet_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int on_off;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi enable_telnet <value>\n");
		print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
		statval = 1;
	} else {
		int qcsapi_retval = 0;
		char *parameter_value = argv[0];

		if (strcmp(parameter_value, "1") && strcmp(parameter_value, "0")) {
			print_err(print, "Usage: call_qcsapi enable_telnet <value>\n");
			print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
			return 1;
		}

		on_off = (qcsapi_unsigned_int) atoi(parameter_value);
		qcsapi_retval = qcsapi_telnet_enable(on_off);

		if (qcsapi_retval == 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wps_set_access_control(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t pp_enable;
	char wps_state[32];

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_wps_access_control <value>\n");
		print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
		statval = 1;
	} else {
		char *parameter_value = argv[0];

		if (!strcmp(parameter_value, "1")) {
			pp_enable = 1;
		} else if (!strcmp(parameter_value, "0")) {
			pp_enable = 0;
		} else {
			print_err(print, "Usage: call_qcsapi set_wps_access_control <value>\n");
			print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
			return 1;
		}

		qcsapi_retval = qcsapi_wps_get_configured_state(the_interface, wps_state,
				sizeof(wps_state));
		if (qcsapi_retval >= 0) {
			if (strncmp(wps_state, "configured", sizeof(wps_state)) != 0) {
				print_err(print, "enable WPS feature before setup WPS Access control\n");
				return 1;
			}
		}

		if (qcsapi_retval >= 0)
			qcsapi_retval = qcsapi_wps_set_access_control(the_interface, pp_enable);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_get_access_control(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t pp_enable;

	if (argc > 0) {
		print_err(print, "Usage: call_qcsapi get_wps_access\n");
		print_err(print, "Usage: This command is used to get pair protection state \n");
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_wps_get_access_control(the_interface, &pp_enable);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", (pp_enable ? "1" : "0"));
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_non_wps_set_pp_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t pp_enable;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_non_wps_pp_enable <value>\n");
		print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
		statval = 1;
	} else {
		char *parameter_value = argv[0];

		if (!strcmp(parameter_value, "1")) {
			pp_enable = 1;
		} else if (!strcmp(parameter_value, "0")) {
			pp_enable = 0;
		} else {
			print_err(print, "Usage: call_qcsapi set_non_wps_pp_enable <value>\n");
			print_err(print, "Usage: value: 0 - disable; 1 - enable\n");
			return 1;
		}

		qcsapi_retval = qcsapi_non_wps_set_pp_enable(the_interface, pp_enable);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_non_wps_get_pp_enable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t pp_enable;

	if (argc > 0) {
		print_err(print, "Usage: call_qcsapi get_non_wps_pp_enable\n");
		print_err(print, "Usage: This command is used to get non_wps pair protection state \n");
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_non_wps_get_pp_enable(the_interface, &pp_enable);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", (pp_enable ? "1" : "0"));
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wps_cancel(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;

	if (argc > 0) {
		print_err(print, "Usage: call_qcsapi wps_cancel <WiFi interface>\n");
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_wps_cancel(the_interface);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wps_set_pbc_in_srcm(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint16_t enabled = 0;

	if (argv[0] != NULL && safe_atou16(argv[0], &enabled, print, 0, 1))
		qcsapi_retval = qcsapi_wps_set_pbc_in_srcm(the_interface, enabled);
	else
		return 1;

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wps_get_pbc_in_srcm(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int enabled = 0;

	qcsapi_retval = qcsapi_wps_get_pbc_in_srcm(the_interface, &enabled);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d\n", enabled);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wps_set_timeout(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	int timeout_val = 0;

	if (argc < 0) {
		print_out(print, "Usage: call_qcsapi wps_timeout <WiFi Interface> <timeout value>\n");
		statval = 1;
	} else {
		timeout_val = atoi(argv[0]);
		if (timeout_val < 120 || timeout_val > 600) {
			print_out(print, "Error: timeout should be limited from 120s to 600s\n");
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_wps_set_timeout(the_interface, timeout_val);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int call_qcsapi_wps_on_hidden_ssid(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	int option = -1;

	if (argc < 0) {
		print_out(print, "Usage: call_qcsapi wps_on_hidden_ssid <WiFi Interface> {0 | 1}\n");
		statval = 1;
	} else {
		option = atoi(argv[0]);
		if ((strlen(argv[0]) != 1) || (!isdigit(*argv[0])) || ((option != 0)
						&& (option != 1))) {
			print_out(print, "Usage: call_qcsapi wps_on_hidden_ssid <WiFi Interface> {0 | 1}\n");
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_wps_on_hidden_ssid(the_interface, option);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int call_qcsapi_wps_on_hidden_ssid_status(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	char state[64];

	if (argc > 0) {
		print_out(print, "Usage: call_qcsapi wps_on_hidden_ssid_status <WiFi Interface>\n");
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_wps_on_hidden_ssid_status(the_interface, state,
				sizeof(state));

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", state);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wps_upnp_enable(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	int option = -1;

	if (argc < 0) {
		print_out(print, "Usage: call_qcsapi wps_upnp_enable <WiFi Interface> {0 | 1}\n");
		statval = 1;
	} else {
		option = atoi(argv[0]);
		if ((strlen(argv[0]) != 1) || (!isdigit(*argv[0])) || ((option != 0)
						&& (option != 1))) {
			print_out(print, "Usage: call_qcsapi wps_upnp_enable <WiFi Interface> {0 | 1}\n");
			statval = 1;
		} else {
			qcsapi_retval = qcsapi_wps_upnp_enable(the_interface, option);

			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;
}

static int call_qcsapi_wps_upnp_status(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	char reply_buffer[16];

	if (argc > 0) {
		print_out(print, "Usage: call_qcsapi wps_upnp_status <WiFi Interface>\n");
		statval = 1;
	} else {
		memset(reply_buffer, 0, sizeof(reply_buffer));
		qcsapi_retval = qcsapi_wps_upnp_status(the_interface, reply_buffer,
				sizeof(reply_buffer));

		if (qcsapi_retval >= 0) {
			print_out(print, "%s\n", reply_buffer);
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wps_registrar_set_dfl_pbc_bss(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;

	if (argc > 0) {
		print_out(print, "Usage: call_qcsapi registrar_set_default_pbc_bss <WiFi Interface>\n");
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_registrar_set_default_pbc_bss(the_interface);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wps_registrar_get_dfl_pbc_bss(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	char reply_buffer[16];
	qcsapi_unsigned_int radio_id = 0;

	if (argc > 1) {
		print_out(print, "Usage: call_qcsapi registrar_get_default_pbc_bss <radio>\n");
		statval = 1;
	} else if (argc == 1) {
		statval = radio_id_param_parse_input(print, argv[0], &radio_id);
	}

	if (statval == 0) {
		memset(reply_buffer, 0, sizeof(reply_buffer));
		qcsapi_retval = qcsapi_radio_registrar_get_default_pbc_bss(radio_id, reply_buffer,
				sizeof(reply_buffer));

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "%s\n", reply_buffer);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_reset_all_counters(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int node_index = p_calling_bundle->caller_generic_parameter.index;
	int local_remote_flag = QCSAPI_LOCAL_NODE;
	int qcsapi_retval = 0;

	if (argc > 0) {
		if (parse_local_remote_flag(print, argv[0], &local_remote_flag) < 0) {
			return 1;
		}
	}

	qcsapi_retval = qcsapi_reset_all_counters(the_interface, node_index, local_remote_flag);

	if (qcsapi_retval == 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_test_traffic(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t period = 0;

	if (argc < 1 || argc > 2) {
		statval = 1;
	} else {
		if ((argc == 2) && (!strcasecmp("start", argv[0]))) {
			sscanf(argv[1], "%u", &period);
			if (period < 10) {
				statval = 1;
				print_err(print, "<period> MUST >= 10 milliseconds for \"start\"\n");
				goto out;
			}
		} else if ((argc == 1) && (!strcasecmp("stop", argv[0]))) {
			period = 0;
		} else {
			statval = 1;
			goto out;
		}

		qcsapi_retval = qcsapi_wifi_test_traffic(the_interface, period);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

out:
	if (statval != 0 && qcsapi_retval >= 0) {
		print_err(print, "Usage: call_qcsapi test_traffic <WiFi interface> <start|stop> <period (unit:ms)>\n");
		print_err(print, "Usage: This command is used to start or stop the test traffic\n");
	}
	return statval;
}

static void call_qcsapi_pm_print_mode(qcsapi_output *print, const int level)
{
	switch (level) {
	case QCSAPI_PM_MODE_DISABLE:
		print_out(print, "off\n");
		break;
	case QCSAPI_PM_MODE_SUSPEND:
		print_out(print, "suspend\n");
		break;
	case QCSAPI_PM_MODE_IDLE:
		print_out(print, "idle\n");
		break;
	default:
		print_out(print, "auto\n");
		break;
	}
}

static int call_qcsapi_pm_str_to_level(qcsapi_output *print,
		const char *const level_str, int *level)
{
	if (strcmp(level_str, "off") == 0) {
		*level = QCSAPI_PM_MODE_DISABLE;
	} else if (strcmp(level_str, "on") == 0 || strcmp(level_str, "auto") == 0) {
		*level = QCSAPI_PM_MODE_AUTO;
	} else if (strcmp(level_str, "suspend") == 0) {
		*level = QCSAPI_PM_MODE_SUSPEND;
	} else if (strcmp(level_str, "idle") == 0) {
		*level = QCSAPI_PM_MODE_IDLE;
	} else {
		print_err(print, "%s: invalid parameter '%s'\n", __FUNCTION__, level_str);
		return -EINVAL;
	}

	return 0;
}

static int call_qcsapi_pm_get_set_mode(call_qcsapi_bundle *call, int argc, char *argv[])
{
	qcsapi_output *print = call->caller_output;
	int level;
	int rc = 0;
	int demac = 0;

	if (argc > 0) {
		if (strcmp("dual_emac", argv[argc - 1]) == 0) {
			demac = 1;
			argc--;
		}
	}

	if (!argc) {
		if (demac) {
			rc = qcsapi_pm_dual_emac_get_mode(&level);
		} else {
			rc = qcsapi_pm_get_mode(&level);
		}
		if (rc >= 0 && verbose_flag >= 0) {
			call_qcsapi_pm_print_mode(print, level);
		}
	} else if (argc == 1) {
		rc = call_qcsapi_pm_str_to_level(print, argv[0], &level);
		if (rc >= 0) {
			if (demac) {
				rc = qcsapi_pm_dual_emac_set_mode(level);
			} else {
				rc = qcsapi_pm_set_mode(level);
			}
		}
	} else {
		rc = -EINVAL;
	}

	if (rc < 0) {
		report_qcsapi_error(call, rc);
	}

	return rc;
}

static int call_qcsapi_qpm_get_level(call_qcsapi_bundle *call, int argc, char *argv[])
{
	qcsapi_output *print = call->caller_output;
	int qpm_level;
	int rc = 0;

	if (argc == 0) {
		rc = qcsapi_get_qpm_level(&qpm_level);
		if (rc < 0 || verbose_flag < 0) {
			goto out;
		}
		print_out(print, "%d\n", qpm_level);

	} else {
		rc = -EINVAL;
	}

out:
	if (rc < 0) {
		report_qcsapi_error(call, rc);
	}

	return rc;
}

static int call_qcsapi_restore_default_config(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int flag = 0;
	char *argp;
	int rc = 0;
	int offset = 0;
	int arg_iter = 0;
#define MODE_NONE		0
#define MODE_SYS_AP		1
#define MODE_SYS_STA		2
#define MODE_PER_RADIO		3
	int mode_force = MODE_NONE;

	while (argc > 0) {
		offset = QCSAPI_RESTORE_FG_MODE_OFFSET;

		argp = argv[arg_iter];
		if (strcmp(argp, "1") == 0 || strcmp(argp, "ip") == 0) {
			flag |= QCSAPI_RESTORE_FG_IP;
		} else if (strcmp(argp, "noreboot") == 0) {
			flag |= QCSAPI_RESTORE_FG_NOREBOOT;
		} else if (strcmp(argp, "ap") == 0) {
			if ((MODE_SYS_STA == mode_force) || (MODE_PER_RADIO == mode_force)) {
				rc = -EINVAL;
				break;
			}
			flag |= (QCSAPI_RESTORE_FG_AP << offset);
			mode_force = MODE_SYS_AP;
		} else if (strcmp(argp, "sta") == 0) {
			if ((MODE_SYS_AP == mode_force) || (MODE_PER_RADIO == mode_force)) {
				rc = -EINVAL;
				break;
			}
			flag |= (QCSAPI_RESTORE_FG_STA << offset);
			mode_force = MODE_SYS_STA;
		} else if (strncmp(argp, "wifi", strlen("wifi")) == 0) {
			unsigned int radio = 0;

			if ((MODE_SYS_STA == mode_force) || (MODE_SYS_AP == mode_force)) {
				rc = -EINVAL;
				break;
			}
			if (argc < 2) {
				rc = -EINVAL;
				break;
			}

			if ((rc = qcsapi_get_radio_from_ifname(argp, &radio)) < 0) {
				break;
			}
			offset += (radio + 1) * QCSAPI_RESTORE_FG_MODE_BITS;
			if (strcmp(argv[arg_iter + 1], "ap") == 0) {
				mode_force = MODE_PER_RADIO;
				flag |= (QCSAPI_RESTORE_FG_AP << offset);
			} else if (strcmp(argv[arg_iter + 1], "sta") == 0) {
				mode_force = MODE_PER_RADIO;
				flag |= (QCSAPI_RESTORE_FG_STA << offset);
			} else {
				rc = -EINVAL;
				break;
			}
			argc--;
			arg_iter++;
		}
		argc--;
		arg_iter++;
	}
	if (rc < 0) {
		qcsapi_report_usage(call,
				" [ {ap | sta | <WiFi interface> {ap | sta} } ] [ip] [noreboot]\n");
		return rc;
	}

	rc = qcsapi_restore_default_config(flag);

	if (rc < 0)
		report_qcsapi_error(call, rc);

	return rc;
#undef MODE_NONE
#undef MODE_SYS_AP
#undef MODE_SYS_STA
#undef MODE_PER_RADIO
}

static int call_qcsapi_run_script(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	int i = 0;
	char *scriptname = NULL;
	char param[QCSAPI_CMD_BUFSIZE], *param_p;
	int len = 0;
	int space = sizeof(param) - 1;
	int qcsapi_retval;
	qcsapi_output *print = call->caller_output;
	param_p = param;

	if (argc == 0) {
		print_err(print, "Not enough parameters\n");
		return 1;
	}

	scriptname = argv[0];

	for (i = 1; i < argc; i++) {
		if (strlen(argv[i]) + 1 < space) {
			len = sprintf(param_p, "%s ", argv[i]);
			param_p += len;
			space -= len;
		} else {
			print_err(print, "Parameter string is too long\n");
			return 1;
		}
	}

	*param_p = '\0';
	qcsapi_retval = qcsapi_wifi_run_script(scriptname, param);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(call, qcsapi_retval);
		if (qcsapi_retval == -qcsapi_not_authorized)
			return 1;
		statval = 1;
	}
#ifdef BUILDIN_TARGET_BOARD
	{
		char strbuf[QCSAPI_MSG_BUFSIZE] = { 0 };
		FILE *log_file;
		/* output the script message */
		log_file = fopen(QCSAPI_SCRIPT_LOG, "r");
		if (log_file != NULL) {
			while (fgets(strbuf, sizeof(strbuf), log_file))
				print_out(print, "%s", strbuf);
			fclose(log_file);
		} else {
			print_err(print, "Failed to open file %s\n", QCSAPI_SCRIPT_LOG);
			return 1;
		}
	}
#endif

	return statval;
}

static int call_qcsapi_get_temperature(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	struct qcsapi_int_array32 temps_in;
	struct qcsapi_int_array32 temps_out;

	temps_in.val[0] = QCSAPI_TEMPSENS_INFO_TYPE_DEFAULT;

	qcsapi_output *print = call->caller_output;

	qcsapi_retval = qcsapi_get_temperature_info_ext(&temps_in, &temps_out);

	if (qcsapi_retval >= 0) {
		if (temps_out.val[QCSAPI_TEMP_INDEX_RFIC_EXT] != QDRV_TEMPSENS_INVALID) {
			print_out(print, "temperature_rfic_external = %3.1f\n",
					(float)temps_out.val[QCSAPI_TEMP_INDEX_RFIC_EXT] /
					QDRV_TEMPSENS_COEFF);
		}
		if (temps_out.val[QCSAPI_TEMP_INDEX_RFIC_INT] != QDRV_TEMPSENS_INVALID) {
			print_out(print, "temperature_rfic_internal = %3.1f\n",
					(float)temps_out.val[QCSAPI_TEMP_INDEX_RFIC_INT] /
					QDRV_TEMPSENS_COEFF10);
		}
		if (temps_out.val[QCSAPI_TEMP_INDEX_BBIC_INT] != QDRV_TEMPSENS_INVALID) {
			print_out(print, "temperature_bbic_internal = %3.1f\n",
					(float)temps_out.val[QCSAPI_TEMP_INDEX_BBIC_INT] /
					QDRV_TEMPSENS_COEFF10);
		}
		if (temps_out.val[QCSAPI_TEMP_INDEX_BBIC_EXT] != QDRV_TEMPSENS_INVALID) {
			print_out(print, "temperature_bbic_external = %3.1f\n",
					(float)temps_out.val[QCSAPI_TEMP_INDEX_BBIC_EXT] /
					QDRV_TEMPSENS_COEFF10);
		}
	} else {
		report_qcsapi_error(call, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_set_accept_oui_filter(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = call->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters\n");
		statval = 1;
	} else {
		const char *the_interface = call->caller_interface;
		qcsapi_mac_addr the_mac_addr;
		qcsapi_mac_addr oui = { 0 };
		int qcsapi_retval;
		int ival = 0;
		int action;

		action = atoi(argv[1]);
		ival = parse_mac_addr(argv[0], the_mac_addr);

		if (ival >= 0) {
			memcpy(oui, the_mac_addr, 3);
			qcsapi_retval = qcsapi_wifi_set_accept_oui_filter(the_interface, oui,
					action);
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0)
					print_out(print, "complete\n");

			} else {
				report_qcsapi_error(call, qcsapi_retval);
				statval = 1;
			}

		} else {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			statval = 1;
		}
	}

	return statval;
}

#define QCSAPI_OUI_LIST_SIZE 126
static int call_qcsapi_get_accept_oui_filter(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = call->caller_interface;
	qcsapi_output *print = call->caller_output;
	char *oui_list = NULL;
	unsigned int sizeof_oui_list = QCSAPI_OUI_LIST_SIZE;

	if (argc > 0) {
		uint32_t usr_input = 0;

		if (qcsapi_str_to_uint32(argv[0], &usr_input)) {
			print_err(print, "Invalid parameter %s - must be an unsigned integer\n",
					argv[0]);
			return 1;
		}

		sizeof_oui_list = (usr_input < QCSAPI_MSG_BUFSIZE) ? usr_input : QCSAPI_MSG_BUFSIZE;
	}

	oui_list = malloc(sizeof_oui_list);
	if (oui_list == NULL) {
		print_err(print, "Failed to allocate %u chars\n", sizeof_oui_list);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_accept_oui_filter(the_interface, oui_list, sizeof_oui_list);

	if (qcsapi_report_str_or_error(call, qcsapi_retval, oui_list))
		statval = 1;

	free(oui_list);

	return statval;
}

static int call_qcsapi_wifi_set_vht(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	qcsapi_output *print = call->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_vht <WiFi interface> {0 | 1}\n");
		rc = 1;
	} else {
		qcsapi_unsigned_int vht_status = (qcsapi_unsigned_int) atoi(argv[0]);
		const char *the_interface = call->caller_interface;
		int qcsapi_retval;

		qcsapi_retval = qcsapi_wifi_set_vht(the_interface, vht_status);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(call, qcsapi_retval);
			rc = 1;
		}
	}
	return rc;
}

static int call_qcsapi_wifi_set_he(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	qcsapi_output *print = call->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_he <WiFi interface> {0 | 1}\n");
		return 1;
	} else {
		qcsapi_unsigned_int he_status = (qcsapi_unsigned_int) atoi(argv[0]);
		const char *the_interface = call->caller_interface;
		int qcsapi_retval = 0;

		qcsapi_retval = qcsapi_wifi_set_he(the_interface, he_status);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(call, qcsapi_retval);
			rc = 1;
		}
	}

	return rc;
}

static int call_qcsapi_wifi_get_he(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int he_status;
	qcsapi_output *print = call->caller_output;
	const char *the_interface = call->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_he(the_interface, &he_status);
	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", he_status);
	} else {
		report_qcsapi_error(call, qcsapi_retval);
		rc = 1;
	}

	return rc;
}

static int call_qcsapi_get_swfeat_list(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = call->caller_output;
	string_4096 buf;
	unsigned int radio_id = 0;

	if (argc > 0) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	qcsapi_retval = qcsapi_radio_get_swfeat_list(radio_id, buf);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(call, qcsapi_retval);
		return 1;
	}

	print_out(print, "%s\n", buf);

	return 0;
}

/*
 * Pass-in epoch time (UTC secs) to convert to readable date string
 */
static void
local_qcsapi_timestr(char *const buf, const size_t bufsize, const uint32_t utc_time_secs)
{
	const time_t epoch_seconds = utc_time_secs;
	struct tm tm_parsed;

	gmtime_r(&epoch_seconds, &tm_parsed);

	strftime(buf, bufsize, "%d %B %Y %H:%M:%S", &tm_parsed);
}

static char *uboot_type_to_str(char type)
{
	char *ptr;

	switch (type - '0') {
	case UBOOT_INFO_LARGE:
		ptr = "Large";
		break;
	case UBOOT_INFO_MINI:
		ptr = "Mini";
		break;
	case UBOOT_INFO_TINY:
		ptr = "Tiny";
		break;
	default:
		ptr = "Unknown";
	}

	return ptr;
}

/*
 * Primary userspace call_qcsapi handler to get u-boot information
 */
static int call_qcsapi_get_uboot_info(call_qcsapi_bundle *call, int argc, char *argv[])
{
	qcsapi_output *print = call->caller_output;
	struct early_flash_config ef_config;
	string_32 version_str;
	string_32 built_str = { 0 };
	uint32_t u_boot_time;
	int qcsapi_retval;
	int uboot_info;
	char *file;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call_qcsapi get_uboot_info, "
				"count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi get_uboot_info <info> : "
				"0 - ver, 1 - built, 2 - type, 3 - all\n");
		return -1;
	}

	file = (argc > 1) ? argv[1] : NULL;

	qcsapi_retval = qcsapi_get_uboot_img_info(version_str, &ef_config, file);
	if (qcsapi_retval) {
		print_err(print, "Call to qcsapi_get_uboot_info failed qcsapi_retval=%d\n",
				qcsapi_retval);
		return -1;
	}

	errno = 0;
	u_boot_time = strtol((char *)ef_config.built_time_utc_sec, NULL, 10);
	if (errno) {
		print_err(print, "strtol(%s) failed, errno=-%d\n",
				(char *)ef_config.built_time_utc_sec, errno);
		return -errno;
	}

	/* Convert UTC seconds to readable date string */
	local_qcsapi_timestr(built_str, sizeof(built_str) - 1, u_boot_time);

	uboot_info = atoi(argv[0]);
	switch (uboot_info) {
	case UBOOT_INFO_VER:
		print_out(print, "Version: %s\n", version_str);
		break;
	case UBOOT_INFO_BUILT:
		print_out(print, "Built: %s\n", built_str);
		break;
	default:
	case UBOOT_INFO_TYPE:
	case UBOOT_INFO_ALL:
		if (uboot_info == UBOOT_INFO_ALL) {
			print_out(print, "Version: %s\nBuilt  : %s\n", version_str, built_str);
		}
		print_out(print, "Type   : U-boot (%s)\n", uboot_type_to_str(ef_config.uboot_type));
		break;
	}

	return 0;
}

static int call_qcsapi_wifi_get_vht(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	int qcsapi_retval;

	qcsapi_unsigned_int vht_status;
	qcsapi_output *print = call->caller_output;
	const char *the_interface = call->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_vht(the_interface, &vht_status);
	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", vht_status);
	} else {
		report_qcsapi_error(call, qcsapi_retval);
		rc = 1;
	}

	return rc;
}

static int
call_qcsapi_calcmd_set_test_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int channel;
	int antenna;
	int mcs;
	int bw;
	int pkt_size;
	int eleven_n;
	int primary_chan;
	if (argc < 7) {
		print_err(print, "Format: set_test_mode calcmd <channel> <antenna> <mcs> <bw> <packet size> <11n> <bf>\n");
		print_err(print, "Example: set_test_mode calcmd 36 127 7 40 40 1 1\n");
		return 1;
	}

	channel = atoi(argv[0]);
	antenna = atoi(argv[1]);
	mcs = atoi(argv[2]);
	bw = atoi(argv[3]);
	pkt_size = atoi(argv[4]);
	eleven_n = atoi(argv[5]);
	primary_chan = atoi(argv[6]);

	qcsapi_retval = qcsapi_calcmd_set_test_mode(channel, antenna, mcs, bw, pkt_size, eleven_n,
			primary_chan);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_calcmd_show_test_packet(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	uint32_t txnum;
	uint32_t rxnum;
	uint32_t crc;

	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_show_test_packet(&txnum, &rxnum, &crc);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "tx_pkts# = \t%d\nrx_pkts# = \t%d\nCRC_err# = \t%d\n",
					txnum, rxnum, crc);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_calcmd_send_test_packet(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	int packet_num;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Format: send_test_packet calcmd <pkt_num>\n");
		print_err(print, "Example: send_test_packet calcmd 0\n");
		return 1;
	}

	packet_num = atoi(argv[0]);

	qcsapi_retval = qcsapi_calcmd_send_test_packet(packet_num);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_stop_test_packet(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_stop_test_packet();
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_send_dc_cw_signal(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int channel;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Format: send_dc_cw_signal calcmd <channel>\n");
		print_err(print, "Example: send_dc_cw_signal calcmd 36\n");
		return 1;
	}

	channel = atoi(argv[0]);
	qcsapi_retval = qcsapi_calcmd_send_dc_cw_signal(channel);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_stop_dc_cw_signal(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_stop_dc_cw_signal();
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_test_mode_antenna_sel(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int antenna;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_test_mode_antenna_sel(&antenna);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", antenna);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_test_mode_mcs(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int mcs;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_test_mode_mcs(&mcs);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", mcs);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_test_mode_bw(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int bw;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_test_mode_bw(&bw);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", bw);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_calcmd_tx_power_rsp tx_power;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_tx_power(&tx_power);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d.%ddBm %d.%ddBm %d.%ddBm %d.%ddBm\n",
					tx_power.value[0] / 10, tx_power.value[0] % 10,
					tx_power.value[1] / 10, tx_power.value[1] % 10,
					tx_power.value[2] / 10, tx_power.value[2] % 10,
					tx_power.value[3] / 10, tx_power.value[3] % 10);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_set_tx_power(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int tx_power;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <tx_power>\n");
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[0], &tx_power) < 0) {
		print_err(p_calling_bundle->caller_output,
				"Invalid parameter %s - must be an unsigned integer\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_radio_calcmd_set_tx_power(p_calling_bundle->caller_interface,
			tx_power);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_real_time_txpower(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval;
	qcsapi_calcmd_tx_power_rsp tx_power;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int i;

	qcsapi_retval = qcsapi_calcmd_get_real_time_txpower(p_calling_bundle->caller_interface,
			&tx_power);
	if (qcsapi_retval >= 0) {
		for (i = 0; i < QCSAPI_QDRV_NUM_RF_STREAMS; i++) {
			print_out(print, " %u.%udBm",
					tx_power.value[i] / 10, tx_power.value[i] % 10);
		}
		print_out(print, "\n");
		return 0;
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
}

static int
call_qcsapi_calcmd_get_test_mode_rssi(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_calcmd_rssi_rsp rssi;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_test_mode_rssi(&rssi);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d.%d %d.%d %d.%d %d.%d\n",
					rssi.value[0] / 10, rssi.value[0] % 10,
					rssi.value[1] / 10, rssi.value[1] % 10,
					rssi.value[2] / 10, rssi.value[2] % 10,
					rssi.value[3] / 10, rssi.value[3] % 10);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_set_mac_filter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int sec_enable;
	int q_num;
	qcsapi_mac_addr mac_addr;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 3) {
		print_out(print, "Parameter input error! \n");
		print_out(print, "Format:\n");
		print_out(print, "call_qcsapi set_mac_filter wifi0 #q_num #sec_enable #mac_addr \n");
		print_out(print, "Example: call_qcsapi set_mac_filter wifi0 1 2 00:11:22:33:44:55\n");

		return qcsapi_retval;
	}

	q_num = atoi(argv[0]);
	sec_enable = atoi(argv[1]);
	qcsapi_retval = parse_mac_addr(argv[2], mac_addr);

	if (qcsapi_retval >= 0) {
		qcsapi_retval = qcsapi_calcmd_set_mac_filter(q_num, sec_enable, mac_addr);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		qcsapi_retval = 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_antenna_count(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_unsigned_int antenna_count;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_get_antenna_count(&antenna_count);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", antenna_count);
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_clear_counter(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_calcmd_clear_counter();
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "Complete.\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_retval;
}

static int
call_qcsapi_calcmd_get_info(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_1024 output_info = { 0 };

	qcsapi_retval = qcsapi_calcmd_get_info(output_info);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_out(print, "%s", output_info);

	return qcsapi_retval;
}

int
call_qcsapi_get_dfs_channels_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_unsigned_int dfs_channels_status;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_dfs_channels_status(the_interface, &dfs_channels_status);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", dfs_channels_status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		qcsapi_retval = 1;
	}

	return qcsapi_retval;
}

int call_qcsapi_disable_dfs_channels(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	int new_channel = 0;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi disable_dfs_channels {0 | 1} [new channel]\n");
		return 1;
	} else if (argc > 1) {
		new_channel = atoi(argv[1]);
	}

	qcsapi_retval = qcsapi_wifi_disable_dfs_channels(the_interface, atoi(argv[0]), new_channel);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_set_soc_macaddr(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_mac_addr new_mac_addr;
	int qcsapi_retval = 0;
	int ival = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi file path, count is %d\n",
				argc);
		statval = 1;
	} else {
		if (strcmp("NULL", argv[0]) == 0) {
			print_out(print, "Mac addr is NULL \n");
			statval = 1;
		} else {
			ival = parse_mac_addr(argv[0], new_mac_addr);
			if (ival >= 0)
				qcsapi_retval = qcsapi_set_soc_mac_addr(the_interface,
						new_mac_addr);
			else {
				print_out(print, "Error parsing MAC address %s\n", argv[0]);
				statval = 1;
			}
		}

		if (ival >= 0) {
			if (qcsapi_retval >= 0) {
				if (verbose_flag >= 0) {
					print_out(print, "complete\n");
				}
			} else {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		}
	}

	return statval;

}

static int call_qcsapi_get_carrier_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int carrier_id = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_get_carrier_id(&carrier_id);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", carrier_id);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_set_carrier_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t carrier_id;
	uint32_t update_uboot = 0;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_carrier_id <carrier ID> [<update uboot flag>]\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		carrier_id = atoi(argv[0]);
	} else {
		print_err(print, "Unrecognized carrier id value %s\n", argv[0]);
		return 1;
	}

	if (argc > 1) {
		if (isdigit(*argv[1])) {
			update_uboot = atoi(argv[1]);
		} else {
			print_err(print, "Unrecognized uboot update flag %s\n", argv[1]);
			return 1;
		}
	}

	qcsapi_retval = qcsapi_set_carrier_id(carrier_id, update_uboot);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_platform_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int platform_id = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_get_platform_id(&platform_id);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", platform_id);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_spinor_jedecid(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	unsigned int jedecid;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_spinor_jedecid(the_interface, &jedecid);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "0x%08x\n", jedecid);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_custom_value(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *key;
	char value[QCSAPI_CUSTOM_VALUE_MAX_LEN] = { '\0' };

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi get_custom_value <key>\n");
		return 1;
	}

	key = argv[0];
	qcsapi_retval = qcsapi_get_custom_value(key, value);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_custom_value(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *key;
	char *value;

	if (argc != 2) {
		print_err(print, "Usage: call_qcsapi set_custom_value <key> <value>\n");
		return 1;
	}

	key = argv[0];
	value = argv[1];
	qcsapi_retval = qcsapi_set_custom_value(key, value);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_vco_lock_detect_mode(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int vco_lock_detect_mode;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi get_vco_lock_detect \n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_vco_lock_detect_mode(the_interface, &vco_lock_detect_mode);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", vco_lock_detect_mode);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_vco_lock_detect_mode(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{

	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set_vco_lock, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int vco_lock_detect_mode = atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_vco_lock_detect_mode(the_interface,
				&vco_lock_detect_mode);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mlme_stats_per_mac(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mac_addr the_mac_addr;
	qcsapi_mlme_stats stats;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc >= 1 && strcmp(argv[0], "NULL") != 0) {
		if (parse_mac_addr(argv[0], the_mac_addr) < 0) {
			print_out(print, "Error parsing MAC address %s\n", argv[0]);
			return 1;
		}
	} else {
		memset(the_mac_addr, 0x00, sizeof(the_mac_addr));
	}

	qcsapi_retval = qcsapi_wifi_get_mlme_stats_per_mac(the_mac_addr, &stats);

	if (qcsapi_retval >= 0) {
		print_out(print, "auth:\t\t%u\n"
				"auth_fails:\t%u\n"
				"assoc:\t\t%u\n"
				"assoc_fails:\t%u\n"
				"deauth:\t\t%u\n"
				"diassoc:\t%u\n",
				stats.auth,
				stats.auth_fails,
				stats.assoc, stats.assoc_fails, stats.deauth, stats.diassoc);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mlme_stats_per_association(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_mlme_stats stats;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int association_index = p_calling_bundle->caller_generic_parameter.index;

	qcsapi_retval = qcsapi_wifi_get_mlme_stats_per_association(the_interface, association_index,
			&stats);

	if (qcsapi_retval >= 0) {
		print_out(print, "auth:\t\t%u\n"
				"auth_fails:\t%u\n"
				"assoc:\t\t%u\n"
				"assoc_fails:\t%u\n"
				"deauth:\t\t%u\n"
				"diassoc:\t%u\n",
				stats.auth,
				stats.auth_fails,
				stats.assoc, stats.assoc_fails, stats.deauth, stats.diassoc);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mlme_stats_macs_list(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mlme_stats_macs mac_list;
	qcsapi_mac_addr terminator_addr;
	int i;

	memset(&terminator_addr, 0xFF, sizeof(terminator_addr));

	qcsapi_retval = qcsapi_wifi_get_mlme_stats_macs_list(&mac_list);

	if (qcsapi_retval >= 0) {
		for (i = 0; i < QCSAPI_MLME_STATS_MAX_MACS; ++i) {
			if (memcmp(mac_list.addr[i], terminator_addr, sizeof(qcsapi_mac_addr)) == 0) {
				break;
			}
			dump_mac_addr(print, mac_list.addr[i]);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_nss_cap(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_mimo_type modulation;
	int qcsapi_retval;
	unsigned int nss;

	modulation = p_calling_bundle->caller_generic_parameter.parameter_type.modulation;
	qcsapi_retval = qcsapi_wifi_get_nss_cap(p_calling_bundle->caller_interface,
			modulation, &nss);

	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", nss);
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_nss_cap(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *const print = p_calling_bundle->caller_output;
	qcsapi_mimo_type modulation;
	int retval = 0;

	modulation = p_calling_bundle->caller_generic_parameter.parameter_type.modulation;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi set_nss_cap "
				"<WiFi interface> {ht|vht|he} <nss>\n");
		retval = 1;
	} else {
		qcsapi_unsigned_int nss = (qcsapi_unsigned_int) atoi(argv[0]);
		int qcsapi_retval;

		qcsapi_retval = qcsapi_wifi_set_nss_cap(p_calling_bundle->caller_interface,
				modulation, nss);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			retval = 1;
		}
	}

	return retval;
}

static int
call_qcsapi_wifi_get_security_defer_mode(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval;
	int defer;

	qcsapi_retval = qcsapi_wifi_get_security_defer_mode(p_calling_bundle->caller_interface,
			&defer);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", defer);
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_security_defer_mode(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *const print = p_calling_bundle->caller_output;
	int retval = 0;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi set_defer <WiFi interface> {0 | 1}\n");
		retval = 1;
	} else {
		int defer = (qcsapi_unsigned_int) atoi(argv[0]);
		int qcsapi_retval;

		qcsapi_retval = qcsapi_wifi_set_security_defer_mode(p_calling_bundle->
				caller_interface, defer);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			retval = 1;
		}
	}

	return retval;
}

static int
call_qcsapi_wifi_apply_security_config(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *const print = p_calling_bundle->caller_output;
	int retval = 0;

	if (argc != 0) {
		print_err(print, "Usage: call_qcsapi apply_security_config " "<WiFi interface>\n");
		retval = 1;
	} else {
		int qcsapi_retval = 0;

		qcsapi_retval = qcsapi_wifi_apply_security_config(p_calling_bundle->
				caller_interface);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			retval = 1;
		}
	}

	return retval;
}

static int
call_qcsapi_wifi_set_intra_bss_isolate(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int enable;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	enable = (qcsapi_unsigned_int) atoi(argv[0]);
	if (enable > 1) {
		print_err(print, "bad parameter %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_intra_bss_isolate(the_interface, enable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_intra_bss_isolate(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int enable;

	qcsapi_retval = qcsapi_wifi_get_intra_bss_isolate(the_interface, &enable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", enable);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bss_isolate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int enable;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	enable = (qcsapi_unsigned_int) atoi(argv[0]);
	if (enable > 1) {
		print_err(print, "bad parameter %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_bss_isolate(the_interface, enable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_bss_isolate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int enable;

	qcsapi_retval = qcsapi_wifi_get_bss_isolate(the_interface, &enable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", enable);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_host_state_set(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	uint16_t host_state;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi wowlan_host_state <WiFi interface> {0 | 1}\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		host_state = atoi(argv[0]);
	} else {
		return 1;
	}
	qcsapi_retval = qcsapi_set_host_state(the_interface, host_state);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_host_state_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t host_state;
	qcsapi_unsigned_int host_state_len = sizeof(host_state);

	qcsapi_retval = qcsapi_wifi_wowlan_get_host_state(the_interface, &host_state,
			&host_state_len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", host_state);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_match_type_set(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	uint16_t wowlan_match;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "not enough params\n");
		print_err(print, "Usage: call_qcsapi wowlan_match_type "
				"<WiFi interface> <protocol> "
				"protocol should be 0, 1(L2) or 2(L3) "
				"0 means match standard magic L2 type(0x0842) or L3 UDP destination port(7 or 9)\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		wowlan_match = atoi(argv[0]);
	} else {
		return 1;
	}
	qcsapi_retval = qcsapi_wowlan_set_match_type(the_interface, wowlan_match);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_match_type_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t match_type;
	qcsapi_unsigned_int len = sizeof(match_type);

	qcsapi_retval = qcsapi_wifi_wowlan_get_match_type(the_interface, &match_type, &len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", match_type);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_L2_type_set(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	uint16_t ether_type;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "not enough params\n");
		print_err(print, "Usage: call_qcsapi wowlan_L2_type "
				"<WiFi interface> <Ether type>\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		ether_type = atoi(argv[0]);
	} else {
		return 1;
	}
	qcsapi_retval = qcsapi_wowlan_set_L2_type(the_interface, ether_type);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_L2_type_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t l2_type;
	qcsapi_unsigned_int len = sizeof(l2_type);

	qcsapi_retval = qcsapi_wifi_wowlan_get_l2_type(the_interface, &l2_type, &len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", l2_type);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_udp_port_set(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	uint16_t udp_port;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "not enough params\n");
		print_err(print, "Usage: call_qcsapi wowlan_udp_port "
				"<WiFi interface> <udp port>\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		udp_port = atoi(argv[0]);
	} else {
		return 1;
	}
	qcsapi_retval = qcsapi_wowlan_set_udp_port(the_interface, udp_port);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wowlan_udp_port_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t udp_port;
	qcsapi_unsigned_int len = sizeof(udp_port);

	qcsapi_retval = qcsapi_wifi_wowlan_get_udp_port(the_interface, &udp_port, &len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", udp_port);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wlmonitor_enable(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	uint32_t enable = 1;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_mac_addr the_mac_addr;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <MAC> <enable>\n");
		return 1;
	}

	if ((strcmp(argv[0], "NULL") == 0) || !isdigit(*argv[1]))
		return 1;

	enable = atoi(argv[1]);

	if (parse_mac_addr(argv[0], the_mac_addr) >= 0)
		qcsapi_retval = qcsapi_wifi_wlmonitor_enable(the_interface, the_mac_addr, enable);
	else {
		print_out(print, "Error parsing MAC address %s\n", argv[0]);
		statval = 1;
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_wifi_wlmonitor_rate_thres(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint32_t rate_thres;
	uint8_t op_severity;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <severity> <PHY rate threshold>\n");
		return 1;
	}

	if (!isdigit(*argv[0]) || !isdigit(*argv[1]))
		return 1;

	op_severity = atoi(argv[0]);
	rate_thres = atoi(argv[1]);

	qcsapi_retval = qcsapi_wifi_wlmonitor_config_threshold(the_interface,
			WLAN_MONITOR_LINK_RATE, op_severity, 0, rate_thres);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_wlmonitor_period_thres(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint32_t period_thres;
	uint8_t op_severity;
	uint8_t period_unit;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <severity> <unit> <monitor period threshold>\n");
		return 1;
	}

	if (!isdigit(*argv[0]) || !isdigit(*argv[1]) || !isdigit(*argv[2]))
		return 1;

	op_severity = atoi(argv[0]);
	period_unit = atoi(argv[1]);
	period_thres = atoi(argv[2]);

	qcsapi_retval = qcsapi_wifi_wlmonitor_config_threshold(the_interface,
			WLAN_MONITOR_LINK_PERIOD, op_severity, period_unit, period_thres);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_set_vlan_loop_detect(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	uint32_t type = 1;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if ((argc < 1) || !isdigit(*argv[0])) {
		print_err(print, "Usage: call_qcsapi vlan_loop_detect <type>\n");
		return 1;
	}

	type = atoi(argv[0]);

	qcsapi_retval = qcsapi_set_vlan_loop_detect(type);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static int
call_qcsapi_get_vlan_loop_detect(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	uint32_t type = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_get_vlan_loop_detect(&type);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", type);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_add_app_ie(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call qcsapi set_app_ie, count is %d\n",
				argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int frametype = (qcsapi_unsigned_int) atoi(argv[0]);
		qcsapi_unsigned_int ieindex = (qcsapi_unsigned_int) atoi(argv[1]);
		char *app_ie = argv[2];
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;
		qcsapi_retval = qcsapi_add_app_ie(the_interface, frametype, ieindex, app_ie);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0)
				print_out(print, "complete\n");

		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}
	return statval;
}

static int
call_qcsapi_remove_app_ie(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Not enough parameters in call qcsapi remove_app_ie, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int frametype = (qcsapi_unsigned_int) atoi(argv[0]);
		qcsapi_unsigned_int ieindex = (qcsapi_unsigned_int) atoi(argv[1]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_remove_app_ie(the_interface, frametype, ieindex);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0)
				print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}
	return statval;
}

static int
call_qcsapi_wifi_disable_11b(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "Usage: call_qcsapi disable_11b <WiFi interface> {0 | 1}\n";

	if (argc != 1) {
		print_out(print, usage);
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_out(print, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_disable_11b(the_interface, atoi(argv[0]));

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int call_qcsapi_wifi_is_weather_channel(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t chan;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<ifname> <channel>");
		return 1;
	}

	if (safe_atou16(argv[0], &chan, print, 0, IEEE80211_CHAN_MAX) == 0)
		return 1;

	qcsapi_retval = qcsapi_wifi_is_weather_channel(the_interface, chan);
	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", qcsapi_retval);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_nac_mon_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint16_t period = MONITOR_DEFAULT_CYCLE_PERIOD;
	uint16_t percentage_on = MONITOR_DEFAULT_ON_PERIOD * 100 / MONITOR_DEFAULT_CYCLE_PERIOD;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint8_t enable = 2;
	const char *usage = "{enable | disable} [ <cycle time> [ <percentage on> ]]\n";

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	} else {
		if (!strcmp(argv[0], "enable")) {
			enable = 1;
		} else if (!strcmp(argv[0], "disable")) {
			enable = 0;
		} else {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
		if (enable == 1) {

			if (argc >= 3) {
				if (0 == safe_atou16(argv[2], &percentage_on, print, 1, 99)) {
					return -EINVAL;
				}
			}
			if (argc >= 2) {
				if (0 == safe_atou16(argv[1], &period, print, 200, 5000)) {
					return -EINVAL;
				}

			}
		}

		qcsapi_retval = qcsapi_wifi_set_nac_mon_mode(the_interface, enable, period,
				(uint8_t) percentage_on);
		if (qcsapi_retval >= 0) {
			if (verbose_flag > 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_get_nac_mon_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int enable = 0;
	int percentage_on = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int period = 0;

	qcsapi_retval = qcsapi_wifi_get_nac_mon_mode(the_interface, &enable, &period,
			&percentage_on);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "status: %s\n Duty Cycle time: %d\n Percentage on: %d\n",
					enable ? "enabled" : "disabled", period, percentage_on);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_get_nac_stats(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *intf = p_calling_bundle->caller_interface;
	int qcsapi_retval;
	qcsapi_nac_stats_report *report = NULL;
	int i = 0;

	report = calloc(1, sizeof(qcsapi_nac_stats_report));
	if (!report)
		return -ENOMEM;
	qcsapi_retval = qcsapi_wifi_get_nac_stats(intf, report);

	if (qcsapi_retval >= 0) {
		if ((verbose_flag >= 0) && (report->num_valid_entries)) {
			print_out(print, "  MAC Address      RSSI(dB)  Timestamp  Channel  Packet Type\n");
			for (i = 0; i < report->num_valid_entries; i++) {
				print_out(print, MACSTR " %9d %10llu %8d   %-10s\n",
						MAC2STR(&report->stats[i].txmac[0]),
						report->stats[i].average_rssi,
						report->stats[i].timestamp,
						report->stats[i].channel,
						(report->stats[i].packet_type == 1) ? "Control" :
						((report->stats[i].packet_type ==
										2) ? "Data" :
								"Management"));
			}
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	free(report);
	return statval;
}

#define MAX_USER_DEFINED_MAGIC	256
void str_to_hex(uint8_t *pbDest, const char *pbSrc, int nLen)
{
	char h1, h2;
	uint8_t s1, s2;
	int i;

	for (i = 0; i < nLen; i++) {
		h1 = pbSrc[2 * i];
		h2 = pbSrc[2 * i + 1];

		s1 = toupper(h1) - 0x30;
		if (s1 > 9)
			s1 -= 7;

		s2 = toupper(h2) - 0x30;
		if (s2 > 9)
			s2 -= 7;

		pbDest[i] = s1 * 16 + s2;
	}
}

int get_pattern_string(const char *arg, uint8_t *pattern)
{
	int loop = 0;
	int num = 0;
	int pattern_len = strnlen(arg, MAX_USER_DEFINED_MAGIC << 1);

	while (loop < pattern_len) {
		if (isxdigit(arg[loop]) && isxdigit(arg[loop + 1])) {
			str_to_hex(&pattern[num], &arg[loop], 1);
			num++;
			loop += 2;
		} else {
			loop++;
		}
	}
	return num;
}

static int
call_qcsapi_wifi_wowlan_pattern_set(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint8_t pattern[MAX_USER_DEFINED_MAGIC];
	struct qcsapi_data_256bytes pattern_data;
	uint32_t input_string_len;
	uint32_t actual_string_len;

	if (argc < 1) {
		print_err(print, "not enough params\n");
		print_err(print, "Usage: call_qcsapi wowlan_pattern "
				"<WiFi interface> <pattern> "
				"pattern should be aabb0a0b and 256 bytes in total length\n");
		return 1;
	}

	memset(pattern, 0, MAX_USER_DEFINED_MAGIC);
	if ((input_string_len = strnlen(argv[0], (MAX_USER_DEFINED_MAGIC << 1) + 1)) >
			(MAX_USER_DEFINED_MAGIC << 1)) {
		print_err(print, "pattern should be 256 bytes in total length\n");
		return 1;
	}

	actual_string_len = get_pattern_string(argv[0], pattern);
	if (actual_string_len != (input_string_len >> 1)) {
		print_err(print, "there are unrecognized chars\n");
		return 1;
	}

	memset(&pattern_data, 0, sizeof(pattern_data));
	memcpy(pattern_data.data, pattern, actual_string_len);
	qcsapi_retval = qcsapi_wowlan_set_magic_pattern(the_interface, &pattern_data,
			actual_string_len);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	return statval;
}

static void
dump_magic_pattern(qcsapi_output *print, struct qcsapi_data_256bytes *magic_pattern,
		qcsapi_unsigned_int pattern_len)
{
	int i;

	for (i = 0; i < pattern_len; i++) {
		print_out(print, "%02X", magic_pattern->data[i]);
	}
	print_out(print, "\n");
}

static int
call_qcsapi_wifi_wowlan_pattern_get(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_256bytes magic_pattern;
	qcsapi_unsigned_int pattern_len = sizeof(magic_pattern);

	memset(&magic_pattern, 0, sizeof(magic_pattern));
	qcsapi_retval = qcsapi_wifi_wowlan_get_magic_pattern(the_interface, &magic_pattern,
			&pattern_len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			dump_magic_pattern(print, &magic_pattern, pattern_len);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_set_extender_params(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_extender_type type =
			p_calling_bundle->caller_generic_parameter.parameter_type.type_of_extender;
	int value = 0;

	if (argc < 1) {
		print_err(print, "Not enough parameters\n");
		statval = 1;
		goto out;
	}

	switch (type) {
	case qcsapi_extender_role:
		if (strcasecmp(argv[0], "mbs") == 0) {
			value = IEEE80211_EXTENDER_ROLE_MBS;
		} else if (strcasecmp(argv[0], "rbs") == 0) {
			value = IEEE80211_EXTENDER_ROLE_RBS;
		} else if (strcasecmp(argv[0], "none") == 0) {
			value = IEEE80211_EXTENDER_ROLE_NONE;
		} else {
			print_err(print, "invalid role [%s]\n", argv[0]);
			statval = 1;
			goto out;
		}
		break;
	case qcsapi_extender_mbs_best_rssi:
	case qcsapi_extender_rbs_best_rssi:
	case qcsapi_extender_mbs_wgt:
	case qcsapi_extender_rbs_wgt:
	case qcsapi_extender_verbose:
	case qcsapi_extender_roaming:
	case qcsapi_extender_bgscan_interval:
	case qcsapi_extender_mbs_rssi_margin:
	case qcsapi_extender_short_retry_limit:
	case qcsapi_extender_long_retry_limit:
	case qcsapi_extender_scan_mbs_intvl:
	case qcsapi_extender_scan_mbs_expiry:
	case qcsapi_extender_scan_mbs_mode:
	case qcsapi_extender_fast_cac:
		if (sscanf(argv[0], "%d", &value) != 1) {
			print_err(print, "Error parsing '%s'\n", argv[0]);
			return 1;
		}
		break;
	default:
		statval = 1;
		goto out;
		break;
	}

	qcsapi_retval = qcsapi_wifi_set_extender_params(the_interface, type, value);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
out:
	return statval;
}

static int
call_qcsapi_wifi_get_bgscan_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int enable = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_bgscan_status(the_interface, &enable);

	if (qcsapi_retval >= 0) {
		print_out(print, "Bgscan enable: %d\n", enable);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_enable_bgscan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int enable = 0;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	if (isdigit(*argv[0])) {
		enable = atoi(argv[0]);
	} else {
		print_err(print, "Unrecognized parameter value %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_enable_bgscan(the_interface, enable);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void
print_extender_params(qcsapi_extender_type type, int value, qcsapi_output *print, int iter)
{
	char *role = NULL;
	if (type == qcsapi_extender_role) {
		switch (value) {
		case IEEE80211_EXTENDER_ROLE_NONE:
			role = "NONE";
			break;
		case IEEE80211_EXTENDER_ROLE_MBS:
			role = "MBS";
			break;
		case IEEE80211_EXTENDER_ROLE_RBS:
			role = "RBS";
			break;
		default:
			break;
		}
		print_out(print, "%s: %s\n", qcsapi_extender_param_table[iter].param_name, role);
	} else {
		print_out(print, "%s: %d\n", qcsapi_extender_param_table[iter].param_name, value);
	}
}

static void
print_eth_info(qcsapi_eth_info_type type, qcsapi_eth_info_result value, qcsapi_output *print)
{
	int iter;
	int mask = 0;
	const struct qcsapi_eth_info_result_s *result;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_eth_info_type_mask_table); iter++) {
		if (qcsapi_eth_info_type_mask_table[iter].type == type) {
			mask = qcsapi_eth_info_type_mask_table[iter].mask;
			break;
		}
	}

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_eth_info_result_table); iter++) {
		result = &qcsapi_eth_info_result_table[iter];
		if (!(mask & result->result_type))
			continue;
		if (value & result->result_type) {
			if (result->result_bit_set)
				print_out(print, "%s: %s\n",
					result->result_label, result->result_bit_set);
		} else {
			if (result->result_bit_unset)
				print_out(print, "%s: %s\n",
					result->result_label, result->result_bit_unset);
		}
	}
}

#define QCSAPI_TX_AMSDU_BE_OFFSET 4
#define QCSAPI_TX_AMSDU_BK_OFFSET 5
#define QCSAPI_TX_AMSDU_VI_OFFSET 6
#define QCSAPI_TX_AMSDU_VO_OFFSET 7
static int
call_qcsapi_wifi_get_tx_amsdu(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int enable, qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint8_t be_en;
	uint8_t bk_en;
	uint8_t vi_en;
	uint8_t vo_en;

	qcsapi_retval = qcsapi_wifi_get_tx_amsdu(wifi, &enable);

	if (qcsapi_retval >= 0) {
		if (enable == 1) {
			be_en = 1;
			bk_en = 1;
			vi_en = 1;
			vo_en = 1;
		} else {
			be_en = (enable >> QCSAPI_TX_AMSDU_BE_OFFSET) & 1;
			bk_en = (enable >> QCSAPI_TX_AMSDU_BK_OFFSET) & 1;
			vi_en = (enable >> QCSAPI_TX_AMSDU_VI_OFFSET) & 1;
			vo_en = (enable >> QCSAPI_TX_AMSDU_VO_OFFSET) & 1;
		}
		print_out(print, "BE %d\n", be_en);
		print_out(print, "BK %d\n", bk_en);
		print_out(print, "VI %d\n", vi_en);
		print_out(print, "VO %d\n", vo_en);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_tx_amsdu(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t enable;
	const char *usage = "<WiFi interface> { 0 | 1 | <AC BE> <AC BK> <AC VI> <AC VO> }";
	uint32_t be_en, bk_en, vi_en, vo_en;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (argc == 4) {
		if (qcsapi_str_to_uint32(argv[0], &be_en) ||
					qcsapi_str_to_uint32(argv[1], &bk_en) ||
					qcsapi_str_to_uint32(argv[2], &vi_en) ||
					qcsapi_str_to_uint32(argv[3], &vo_en)) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
		qcsapi_retval = qcsapi_wifi_set_tx_amsdu_per_ac(wifi, be_en, bk_en, vi_en, vo_en);
	} else {
		if (qcsapi_str_to_uint32(argv[0], &enable)) {
			qcsapi_report_usage(p_calling_bundle, usage);
			return 1;
		}
		qcsapi_retval = qcsapi_wifi_set_tx_amsdu(wifi, enable);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_beacon_phyrate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int beacon_phyrate;
	int qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_beacon_phyrate(wifi, &beacon_phyrate);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", legacy_phyrate_enum_to_name(beacon_phyrate));
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_beacon_phyrate(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_legacy_phyrate phyrate;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_beacon_rate "
				"<WiFi interface> <qcsapi_legacy_phyrate>\n");
		return 1;
	}

	qcsapi_retval = name_to_legacy_phyrate_enum(argv[0], &phyrate);
	if (!qcsapi_retval) {
		print_err(print, "Invalid QCSAPI legacy PHY rate %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_beacon_phyrate(wifi, phyrate);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_beacon_power_backoff(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int power_backoff;

	qcsapi_retval = qcsapi_wifi_get_beacon_power_backoff(the_interface, &power_backoff);

	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", power_backoff);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_beacon_power_backoff(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int power_backoff;

	if (argc != 1)
		goto usage;

	if (qcsapi_str_to_uint32(argv[0], &power_backoff) < 0)
		goto usage;

	qcsapi_retval = qcsapi_wifi_set_beacon_power_backoff(the_interface, power_backoff);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
usage:
	qcsapi_report_usage(p_calling_bundle, "<interface> <backoff in dB>\n");
	return 1;
}

static int
call_qcsapi_wifi_get_mgmt_power_backoff(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int power_backoff;

	qcsapi_retval = qcsapi_wifi_get_mgmt_power_backoff(the_interface, &power_backoff);

	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", power_backoff);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_mgmt_power_backoff(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int power_backoff;

	if (argc != 1)
		goto usage;

	if (qcsapi_str_to_uint32(argv[0], &power_backoff) < 0)
		goto usage;

	qcsapi_retval = qcsapi_wifi_set_mgmt_power_backoff(the_interface, power_backoff);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;

usage:
	qcsapi_report_usage(p_calling_bundle, "<interface> <backoff in dB>\n");
	return 1;
}

static int
check_dcs_parameter(char *str_s, char *str_t, char *str_v, uint16_t *p, qcsapi_output *print)
{
	if ((strcmp(str_s, str_t) == 0) && safe_atou16(str_v, p, print, 0, 0xFFFF))
		return 1;

	return 0;
}

static int
call_qcsapi_wifi_start_dcs_scan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int statval = 0;
	int index = 0;
	uint32_t chan_num = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_dcs_params dcs_params;
	uint16_t interval_sec = 0;

	memset(&dcs_params, 0, sizeof(qcsapi_dcs_params));
	if (argc >= 1) {
		for (index = 0; index < (argc - 1); index++) {
			if ((strcmp(argv[index], "interval") == 0) && safe_atou16(argv[index + 1],
							&dcs_params.scan_interval, print, 0, 0xFF))
				index++;
			else if ((strcmp(argv[index], "interval_sec") == 0)
					&& safe_atou16(argv[index + 1], &interval_sec, print, 0,
							0xFF))
				index++;
			else if (check_dcs_parameter(argv[index], "duration", argv[index + 1],
							&dcs_params.scan_duration, print))
				index++;
			else if (check_dcs_parameter(argv[index], "dwell", argv[index + 1],
							&dcs_params.dwell_time, print))
				index++;
			else if (check_dcs_parameter(argv[index], "spacing", argv[index + 1],
							&dcs_params.spacing, print))
				index++;
			else if ((strcmp(argv[index], "chanlist") == 0)
					&& (string_to_list(print, argv[index + 1],
									dcs_params.chan_list,
									&chan_num) == 0))
				index++;
			else
				break;
		}

		if (index < argc) {
			print_out(print, "Usage: call_qcsapi start_dcs_scan "
					"<WiFi interface> [ interval <interval> ] "
					"[ interval_sec <interval_sec> ] "
					"[ duration <duration> ] [ dwell <dwell> ] "
					"[ spacing <spacing> ] [ chanlist <chanlist> ]\n");
			return 1;
		}
	}

	dcs_params.scan_interval = QCSAPI_DCS_INTERVAL_PACK(dcs_params.scan_interval, interval_sec);
	qcsapi_retval = qcsapi_wifi_start_dcs_scan(the_interface, &dcs_params);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_stop_dcs_scan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int statval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_stop_dcs_scan(the_interface);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_dcs_scan_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	int statval = 0;
	int index = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_dcs_params dcs_params;

	memset(&dcs_params, 0, sizeof(qcsapi_dcs_params));

	qcsapi_retval = qcsapi_wifi_get_dcs_scan_params(the_interface, &dcs_params);
	if (qcsapi_retval >= 0) {
		print_out(print, "DCS:%s interval:%u(min)%u(sec)"
				" duration:%u(msec) dwell time:%u(msec)"
				" spacing:%u(msec)\n",
				dcs_params.dcs_status ? "started" : "stopped",
				QCSAPI_DCS_INTERVAL_GET_MINUTES(dcs_params.scan_interval),
				QCSAPI_DCS_INTERVAL_GET_SECONDS(dcs_params.scan_interval),
				dcs_params.scan_duration, dcs_params.dwell_time,
				dcs_params.spacing);
		print_out(print, "channel list:");
		for (index = 0; index < ARRAY_SIZE(dcs_params.chan_list); index++) {
			if (dcs_params.chan_list[index] == 0)
				break;
			print_out(print, "%u ", dcs_params.chan_list[index]);
		}
		print_out(print, "\n");

	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_extender_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_extender_type type = 0;
	int value = 0;
	unsigned int iter;

	for (iter = 0; iter < ARRAY_SIZE(qcsapi_extender_param_table); iter++) {
		type = qcsapi_extender_param_table[iter].param_type;
		if (type == qcsapi_extender_nosuch_param)
			continue;
		qcsapi_retval = qcsapi_wifi_get_extender_params(the_interface, type, &value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_extender_params(type, value, print, iter);
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			return 1;
		}
	}

	return 0;
}

static int
call_qcsapi_is_startprod_done(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval;
	int status = 0;

	qcsapi_output *print = p_calling_bundle->caller_output;

	retval = qcsapi_is_startprod_done(&status);
	if (retval < 0) {
		report_qcsapi_error(p_calling_bundle, retval);
		return 1;
	}

	print_out(print, "%d\n", status);

	return 0;
}

static int call_qcsapi_wifi_get_disassoc_reason(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int qcsapi_retval;

	qcsapi_unsigned_int disassoc_reason;
	qcsapi_output *print = call->caller_output;
	const char *the_interface = call->caller_interface;

	COMPILE_TIME_ASSERT(ARRAY_SIZE(qcsapi_ieee80211_reason_str) == IEEE80211_REASON_MAX);

	qcsapi_retval = qcsapi_wifi_get_disassoc_reason(the_interface, &disassoc_reason);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(call, qcsapi_retval);
		return qcsapi_retval;
	}
	if (disassoc_reason < ARRAY_SIZE(qcsapi_ieee80211_reason_str))
		print_out(print, "Disassoc Reason Code - %u: %s\n", disassoc_reason,
				qcsapi_ieee80211_reason_str[disassoc_reason]);
	else
		print_out(print, "Reserved Code [%d]\n", disassoc_reason);

	return 0;
}

static int
call_qcsapi_wifi_get_bb_param(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_unsigned_int bb_param;

	qcsapi_retval = qcsapi_wifi_get_bb_param(the_interface, &bb_param);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", bb_param);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_bb_param(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{

	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi bb_param, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int bb_param = atoi(argv[0]);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_bb_param(the_interface, bb_param);
		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scan_buf_max_size(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int max_buf_size;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	max_buf_size = (qcsapi_unsigned_int) atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_set_scan_buf_max_size(the_interface, max_buf_size);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scan_buf_max_size(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int max_buf_size;

	qcsapi_retval = qcsapi_wifi_get_scan_buf_max_size(the_interface, &max_buf_size);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", max_buf_size);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_scan_table_max_len(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int max_table_len;

	if (argc < 1) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	max_table_len = (qcsapi_unsigned_int) atoi(argv[0]);
	qcsapi_retval = qcsapi_wifi_set_scan_table_max_len(the_interface, max_table_len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_scan_table_max_len(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int max_table_len;

	qcsapi_retval = qcsapi_wifi_get_scan_table_max_len(the_interface, &max_table_len);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%u\n", max_table_len);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_enable_mu(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int mu_enable;
	const char *param_value = argv[0];

	if (argc < 1 || strcmp(argv[0], "NULL") == 0) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	if (strcmp(param_value, "0") != 0 && strcmp(param_value, "1") != 0) {
		print_err(print, "Invalid value %s - must be 0 or 1\n", param_value);
		return 1;
	}

	mu_enable = (qcsapi_unsigned_int) atoi(param_value);
	qcsapi_retval = qcsapi_wifi_set_enable_mu(the_interface, mu_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		qcsapi_retval = qcsapi_config_update_parameter(the_interface, "mu", param_value);
		print_out(print, "complete\n");
	}

	return statval;
}

static int
call_qcsapi_wifi_get_enable_mu(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int mu_enable;

	qcsapi_retval = qcsapi_wifi_get_enable_mu(the_interface, &mu_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%u\n", mu_enable);
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mu_use_precode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int prec_enable;
	qcsapi_unsigned_int grp;

	if (argc < 2 || strcmp(argv[1], "NULL") == 0 || strcmp(argv[0], "NULL") == 0) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	grp = (qcsapi_unsigned_int) atoi(argv[0]);
	prec_enable = (qcsapi_unsigned_int) atoi(argv[1]);

	qcsapi_retval = qcsapi_wifi_set_mu_use_precode(the_interface, grp, prec_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mu_use_precode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int prec_enable;
	qcsapi_unsigned_int grp;

	if (argc < 1 || strcmp(argv[0], "NULL") == 0) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	grp = (qcsapi_unsigned_int) atoi(argv[0]);
	qcsapi_retval = qcsapi_wifi_get_mu_use_precode(the_interface, grp, &prec_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%u\n", prec_enable);
	}

	return statval;
}

static int
call_qcsapi_wifi_set_mu_use_eq(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int eq_enable;

	if (argc < 1 || strcmp(argv[0], "NULL") == 0) {
		print_err(print, "Not enough parameters, count is %d\n", argc);
		return 1;
	}

	eq_enable = (qcsapi_unsigned_int) atoi(argv[0]);
	qcsapi_retval = qcsapi_wifi_set_mu_use_eq(the_interface, eq_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "complete\n");
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mu_use_eq(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int eq_enable;

	qcsapi_retval = qcsapi_wifi_get_mu_use_eq(the_interface, &eq_enable);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%u\n", eq_enable);
	}

	return statval;
}

static int
call_qcsapi_wifi_get_mu_groups(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char buf[1024];

	qcsapi_retval = qcsapi_wifi_get_mu_groups(the_interface, &buf[0], sizeof(buf));

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%s", buf);
	}

	return statval;
}

static int
call_qcsapi_wifi_set_optim_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *str_ptr;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call qcsapi WiFi set_optim_stats, count is %d\n", argc);
		statval = 1;
	} else {
		qcsapi_unsigned_int rx_optim_stats = strtol(argv[0], &str_ptr, 10);
		int qcsapi_retval;
		const char *the_interface = p_calling_bundle->caller_interface;

		qcsapi_retval = qcsapi_wifi_set_optim_stats(the_interface, rx_optim_stats);
		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_send_file(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *image_file_path = NULL;
	int image_flags = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1 && argc != 2) {
		print_err(print, "Usage: call_qcsapi send_file <image file path> <flags>\n");
		statval = 1;
	} else {
		if (strcmp(argv[0], "NULL") != 0) {
			image_file_path = argv[0];

			qcsapi_retval = qcsapi_send_file(image_file_path, image_flags);
			if (qcsapi_retval < 0) {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		} else {
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_dscp_fill(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 3) {
		statval = 1;
		print_err(print, "Usage: call_qcsapi dscp <fill> <emac0|emac1> <value>\n");
	} else {
		int qcsapi_retval;
		const char *eth_type = argv[1];
		const char *value = argv[2];

		if (strcmp(eth_type, "NULL") == 0) {
			eth_type = NULL;
		}
		if (strcmp(value, "NULL") == 0) {
			value = NULL;
		}

		qcsapi_retval = qcsapi_eth_dscp_map(qcsapi_eth_dscp_fill,
				eth_type, NULL, value, NULL, 0);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_dscp_poke(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 4) {
		print_err(print, "Usage: call_qcsapi dscp <poke> <emac0|emac1> <level> <value>\n");
		statval = 1;
	} else {
		int qcsapi_retval;
		const char *eth_type = argv[1];
		const char *level = argv[2];
		const char *value = argv[3];

		if (strcmp(eth_type, "NULL") == 0) {
			eth_type = NULL;
		}
		if (strcmp(level, "NULL") == 0) {
			level = NULL;
		}
		if (strcmp(value, "NULL") == 0) {
			value = NULL;
		}

		qcsapi_retval = qcsapi_eth_dscp_map(qcsapi_eth_dscp_poke,
				eth_type, level, value, NULL, 0);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_dscp_dump(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char buf[2048] = { 0 };
	char *eth_type = argv[1];

	if (strcmp(eth_type, "NULL") == 0) {
		eth_type = NULL;
	}

	qcsapi_retval = qcsapi_eth_dscp_map(qcsapi_eth_dscp_dump,
			eth_type, NULL, NULL, &buf[0], sizeof(buf));

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%s", buf);
	}

	return statval;
}

static int
call_qcsapi_get_emac_switch(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	char buf[2048] = { 0 };

	qcsapi_retval = qcsapi_get_emac_switch(buf);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	} else if (verbose_flag >= 0) {
		print_out(print, "%s\n", buf);
	}

	return statval;
}

static int
call_qcsapi_set_emac_switch(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int value;

	value = (qcsapi_unsigned_int) atoi(argv[0]);
	if (value == 0) {
		qcsapi_retval = qcsapi_set_emac_switch(qcsapi_emac_switch_enable);
	} else {
		qcsapi_retval = qcsapi_set_emac_switch(qcsapi_emac_switch_disable);
	}

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_eth_dscp_map(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi dscp <fill|poke|dump>"
				" <emac0|emac1> [level] [value]\n");
		statval = 1;
	} else {
		char *param = argv[0];

		if (strcmp(param, "fill") == 0) {
			statval = call_qcsapi_dscp_fill(p_calling_bundle, argc, argv);
		} else if (strcmp(param, "poke") == 0) {
			statval = call_qcsapi_dscp_poke(p_calling_bundle, argc, argv);
		} else if (strcmp(param, "dump") == 0) {
			statval = call_qcsapi_dscp_dump(p_calling_bundle, argc, argv);
		} else {
			print_err(print, "Usage: call_qcsapi dscp <fill|poke|dump>"
					" <emac0|emac1> [level] [value]\n");
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_wifi_set_pref_band(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	qcsapi_output *print = call->caller_output;
	int qcsapi_retval = 0;
	const char *usage = "Usage: call_qcsapi set_pref_band <WiFi interface> {2.4ghz | 5ghz}\n";

	if (argc < 1) {
		print_err(print, usage);
		rc = 1;
	} else {
		qcsapi_pref_band pref_band;
		const char *the_interface = call->caller_interface;

		pref_band = string_to_wifi_band(argv[0]);

		if (pref_band == qcsapi_nosuch_band) {
			print_err(print, usage);
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_pref_band(the_interface, pref_band);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(call, qcsapi_retval);
			rc = 1;
		}
	}

	return rc;
}

static int call_qcsapi_wifi_get_pref_band(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int rc = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int pref_band;
	qcsapi_output *print = call->caller_output;
	const char *the_interface = call->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_pref_band(the_interface, &pref_band);

	if (qcsapi_retval >= 0) {
		if (pref_band == qcsapi_band_2_4ghz) {
			print_out(print, "Preferred Band Set - 2.4Ghz\n");
		} else if (pref_band == qcsapi_band_5ghz) {
			print_out(print, "Preferred Band Set - 5Ghz\n");
		}
	} else {
		report_qcsapi_error(call, qcsapi_retval);
		rc = 1;
	}

	return rc;
}

static int call_qcsapi_set_sys_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	unsigned long sec;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi set_sys_time <seconds since epoch>\n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Invalid value for seconds since epoch\n");
		return 1;
	}

	sec = strtoul(argv[0], NULL, 10);
	if (sec == 0 || sec >= UINT32_MAX) {
		print_err(print, "Invalid value for seconds since epoch\n");
		return 1;
	}

	statval = qcsapi_wifi_set_sys_time((uint32_t) sec);
	if (statval >= 0 && verbose_flag >= 0)
		print_out(print, "complete\n");
	else
		report_qcsapi_error(p_calling_bundle, statval);

	return statval;
}

static int call_qcsapi_get_sys_time(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t sec;

	if (argc != 0) {
		print_err(print, "Usage: call_qcsapi get_sys_time\n");
		return 1;
	}

	statval = qcsapi_wifi_get_sys_time(&sec);
	if (statval == 0) {
		print_out(print, "%u\n", sec);
	} else {
		report_qcsapi_error(p_calling_bundle, statval);
	}

	return statval;
}

static int call_qcsapi_get_eth_info(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_eth_info_type eth_info_type = qcsapi_eth_nosuch_type;
	qcsapi_eth_info_result eth_info_result = qcsapi_eth_info_unknown;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	const char *usage = "<interface> {link | speed | duplex | autoneg}\n";

	if (argc != 0 && argc != 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	if (argc == 0) {
		for (eth_info_type = qcsapi_eth_info_start;
				eth_info_type < qcsapi_eth_info_all; eth_info_type++) {
			qcsapi_retval = qcsapi_get_eth_info(the_interface, eth_info_type);
			if (qcsapi_retval < 0) {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				return 1;
			}
			eth_info_result |= (qcsapi_eth_info_result) qcsapi_retval;
		}
		print_eth_info(eth_info_type, eth_info_result, print);
		return 0;
	}

	if (!strcmp("link", argv[0])) {
		eth_info_type = qcsapi_eth_info_link;
	} else if (!strcmp("speed", argv[0])) {
		eth_info_type = qcsapi_eth_info_speed;
	} else if (!strcmp("duplex", argv[0])) {
		eth_info_type = qcsapi_eth_info_duplex;
	} else if (!strcmp("autoneg", argv[0])) {
		eth_info_type = qcsapi_eth_info_autoneg;
	} else {
		print_out(print, "Invalid option\n");
		return 1;
	}

	qcsapi_retval = qcsapi_get_eth_info(the_interface, eth_info_type);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	print_eth_info(eth_info_type, qcsapi_retval, print);

	return 0;
}

static int
call_qcsapi_wifi_set_ap_interface_name(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval;
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		print_out(print, "Usage: call_qcsapi " "set_ap_interface_name <interface name>\n");
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_wifi_set_ap_interface_name(argv[0]);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_ap_interface_name(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char ifname[IFNAMSIZ] = { 0 };
	qcsapi_unsigned_int radio_id = 0;

	if (argc > 1) {
		print_out(print, "Usage: call_qcsapi " "get_ap_interface_name <radio>\n");
		return -EINVAL;
	}

	if (argc == 1) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	qcsapi_retval = qcsapi_radio_get_ap_interface_name(radio_id, ifname);
	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", ifname);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_verify_repeater_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int radio_id = 0;

	if (argc > 1) {
		print_out(print, "Usage: call_qcsapi " "verify_repeater_mode <radio>\n");
		return -EINVAL;
	}

	if (argc == 1) {
		if (radio_id_param_parse_input(print, argv[0], &radio_id))
			return 1;
	}

	if (qcsapi_retval >= 0)
		qcsapi_retval = qcsapi_radio_verify_repeater_mode(radio_id);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", qcsapi_retval);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_wifi_block_bss(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int flag;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi block_bss <WiFi interface> {0 | 1}\n");
		return 1;
	}

	if (isdigit(*argv[0])) {
		flag = atoi(argv[0]);
	} else {
		print_err(print, "Unrecognized %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_block_bss(the_interface, flag);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_txba_disable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi txba_disable <WiFi interface> {0 | 1}\n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Unrecognized %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_txba_disable(the_interface, atoi(argv[0]));

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_txba_disable(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int txba_disable_status;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_txba_disable(the_interface, &txba_disable_status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", txba_disable_status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_rxba_decline(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Usage:  call_qcsapi rxba_decline <WiFi interface> {0 | 1} \n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Unrecognized %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_rxba_decline(the_interface, atoi(argv[0]));

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_rxba_decline(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int rxba_decline_status;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_rxba_decline(the_interface, &rxba_decline_status);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", rxba_decline_status);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_txburst(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi set_txburst <WiFi interface> {0 | 1} \n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Unrecognized %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_txburst(the_interface, atoi(argv[0]));
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_txburst(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int flag;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_get_txburst(the_interface, &flag);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", flag);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_sec_chan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int chan, sec_chan, qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Usage: call_qcsapi get_sec_chan_offset "
				"<WiFi interface> <chan>\n");
		return 1;
	}

	chan = atoi(argv[0]);
	if (chan > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %s\n", chan);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_get_sec_chan(wifi, chan, &sec_chan);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", sec_chan);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_sec_chan(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int chan, offset, qcsapi_retval;
	const char *wifi = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		print_err(print, "Usage: call_qcsapi get_sec_chan_offset "
				"<WiFi interface> <chan> <offset>\n");
		return 1;
	}

	chan = atoi(argv[0]);
	offset = atoi(argv[1]);
	if (chan > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %s\n", chan);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_sec_chan(wifi, chan, offset);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "success\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_set_vap_default_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<radio> {0 | 1}\n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Bad radio index %s\n", argv[0]);
		return 1;
	}

	if (qcsapi_verify_numeric(argv[1]) < 0) {
		print_err(print, "Bad default state value %s\n", argv[1]);
		return 1;
	}

	qcsapi_retval = qcsapi_radio_wifi_set_vap_default_state(atoi(argv[0]), atoi(argv[1]));
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_vap_default_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int vap_default_state;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<radio>\n");
		return 1;
	}

	if (qcsapi_verify_numeric(argv[0]) < 0) {
		print_err(print, "Bad radio index %s\n", argv[0]);
		return 1;
	}

	qcsapi_retval = qcsapi_radio_wifi_get_vap_default_state(atoi(argv[0]), &vap_default_state);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", vap_default_state);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void
dump_tx_airtime_per_node(qcsapi_output *print, uint32_t idx, qcsapi_node_tx_airtime *nta)
{
	print_out(print, "%-17s %8s %8s %20s\n", "MAC", "Idx", "airtime", "airtime_accumulative");
	print_out(print, MACSTR " %8d %8d %20d\n", MAC2STR(nta->addr), idx,
			nta->tx_airtime, nta->tx_airtime_accum);
}

static void dump_tx_airtime_buffer(qcsapi_output *print, const struct qcsapi_data_3Kbytes *buffer)
{
	uint32_t *p_airtime_accum;
	uint32_t *p_airtime;
	uint16_t nr_nodes;
	uint16_t *p_idx;
	uint8_t *p_mac;
	char *pos;
	uint16_t fat;
	int i;

	pos = (char *)buffer->data;

	nr_nodes = *(uint16_t *) pos;
	pos += sizeof(uint16_t);

	fat = *(uint16_t *) pos;
	pos += sizeof(uint16_t);

	print_out(print, "Free airtime: %8u\n", fat);
	print_out(print, "%-17s %8s %8s %20s\n", "MAC", "Idx", "airtime", "airtime_accumulative");

	for (i = 0; i < nr_nodes; i++) {
		p_idx = (uint16_t *) pos;
		p_mac = (uint8_t *) (pos + sizeof(*p_idx));
		p_airtime = (uint32_t *) (p_mac + sizeof(qcsapi_mac_addr));
		p_airtime_accum = p_airtime + 1;

		print_out(print, MACSTR " %8u %8u %20u\n", MAC2STR(p_mac), *p_idx,
				*p_airtime, *p_airtime_accum);

		pos += sizeof(*p_idx) + sizeof(qcsapi_mac_addr) + sizeof(*p_airtime) +
				sizeof(*p_airtime_accum);
	}
}

static int
call_qcsapi_wifi_get_tx_airtime(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *intf = p_calling_bundle->caller_interface;
	qcsapi_node_tx_airtime nta;
	struct qcsapi_data_3Kbytes *buffer = NULL;
	int qcsapi_retval;
	uint32_t idx = 0;
	int statval = 0;
	int for_all = 0;
	int control = 0;

	COMPILE_TIME_ASSERT(sizeof(struct qcsapi_data_3Kbytes) >= sizeof(qcsapi_node_tx_airtime));

	if (argc < 1) {
		statval = 1;
		goto out_usage;
	}

	if (!strcmp(argv[0], "all"))
		for_all = 1;
	else
		idx = strtoul(argv[0], NULL, 10);

	if (argc == 2) {
		if (!strcmp(argv[1], "start")) {
			control = qcsapi_accum_airtime_start;
		} else if (!strcmp(argv[1], "stop")) {
			control = qcsapi_accum_airtime_stop;
		} else {
			print_err(print, "The argument \"%s\" is invalid\n", argv[1]);
			statval = 1;
			goto out_usage;
		}
	}

	buffer = (struct qcsapi_data_3Kbytes *)malloc(sizeof(struct qcsapi_data_3Kbytes));
	if (buffer == NULL) {
		print_err(print, "malloc failed - %s\n", __func__);
		return -1;
	}
	memset(buffer, 0, sizeof(*buffer));

	if (control) {
		if (for_all)
			qcsapi_retval = qcsapi_wifi_tx_airtime_accum_control(intf, control);
		else
			qcsapi_retval = qcsapi_wifi_node_tx_airtime_accum_control(intf, idx,
					control);
	} else {
		if (for_all)
			qcsapi_retval = qcsapi_wifi_get_tx_airtime(intf, buffer);
		else
			qcsapi_retval = qcsapi_wifi_node_get_tx_airtime(intf, idx, &nta);
	}

	if (qcsapi_retval >= 0) {
		if (control) {
			print_out(print, "complete\n");
		} else {
			if (for_all)
				dump_tx_airtime_buffer(print, buffer);
			else
				dump_tx_airtime_per_node(print, idx, &nta);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	if (NULL != buffer)
		free(buffer);

	return statval;

out_usage:
	qcsapi_report_usage(p_calling_bundle,
			"<interface> {<node_idx> | all} [ {start | stop} ]\n");

	return statval;
}

static int
call_qcsapi_wifi_set_cs_thrshld_range(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int min = 0;
	int max = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> <min> <max>\n";

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	min = atoi(argv[0]);
	max = atoi(argv[1]);

	if (min > max) {
		print_out(print, "max must be greater than min\n");
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_cs_thrshld_range(the_interface, min, max);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_cs_thrshld_range(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int min;
	int max;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_cs_thrshld_range(the_interface, &min, &max);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d %d\n", min, max);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_cs_thrshld_inuse(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_out(print, "%s: programming error, expected at least 1 additional parameter\n", __func__);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_cs_thrshld_inuse(the_interface, atoi(argv[0]));
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_cs_thrshld_inuse(const call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int inuse;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_cs_thrshld_inuse(the_interface, &inuse);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%d\n", inuse);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_qwe_command(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	char *command = argv[0];
	char *param1 = (argc >= 2) ? argv[1] : NULL;
	char *param2 = (argc >= 3) ? argv[2] : NULL;
	char *param3 = (argc >= 4) ? argv[3] : NULL;
	char output[1024];

	if (argc < 1 || argc > 4) {
		print_err(print, "Usage: call_qcsapi qwe <command> [<param1>] [<param2>] [<param3>]\n");
		return 1;
	}

	qcsapi_retval = qcsapi_qwe_command(command, param1, param2, param3, output, sizeof(output));
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%s\n", output);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_get_core_dump2(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_4Kbytes *buf = NULL;
	unsigned int bytes_copied;
	unsigned int bytes_written;
	unsigned int start_offset;

	buf = calloc(1, sizeof(*buf));
	if (!buf) {
		print_err(print, "Could not allocate %u bytes of memory\n", sizeof(*buf));
		retval = 1;
		goto out;
	}

	start_offset = 0;

	while (1) {
		qcsapi_retval = qcsapi_get_core_dump2(buf, sizeof(*buf), start_offset,
				&bytes_copied);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			retval = 1;
			goto out;
		}

		if (!bytes_copied)
			break;

		bytes_written = write(STDOUT_FILENO, buf->data, bytes_copied);
		if ((bytes_written == -1) || (bytes_written != bytes_copied)) {
			retval = 1;
			goto out;
		}

		start_offset += bytes_copied;
	}

out:
	if (buf)
		free(buf);

	return retval;
}

static int
call_qcsapi_gather_info(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int retval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_3Kbytes *buf = NULL;
	unsigned int offset = 0;
	unsigned int bytes_copied = 0;
	unsigned int bytes_remain = 0;
	unsigned int bytes_remain_last;
	int dump_finished;
	int pid = 0;

	qcsapi_retval = qcsapi_gather_info_start(&pid);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		retval = 1;
		goto out;
	}

	buf = malloc(sizeof(struct qcsapi_data_3Kbytes));
	if (!buf) {
		print_err(print, "allocate memory failed\n");
		retval = 1;
		goto out;
	}

	dump_finished = 0;
	bytes_remain_last = (unsigned int)(-1);	/*initialize with the max value */
	while ((dump_finished == 0) || (bytes_remain > 0)) {
		if (dump_finished == 0) {
			/* the gathering process not finished yet, so don't hurry */
			usleep(500000);
			qcsapi_retval = qcsapi_gather_info_check_cmd_status(pid, &dump_finished);
			if (qcsapi_retval < 0) {
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				retval = 1;
				goto out;
			}
		}

		memset(buf, 0, sizeof(struct qcsapi_data_3Kbytes));
		bytes_copied = 0;
		bytes_remain = 0;
		qcsapi_retval = qcsapi_read_gathered_info(buf, offset, &bytes_copied,
				&bytes_remain);
		if (qcsapi_retval < 0) {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			retval = 1;
			goto out;
		}
		offset += bytes_copied;

		if (write(STDOUT_FILENO, buf, bytes_copied) == -1) {
			retval = -errno;
			print_err(print, "write buffer: %s\n", strerror(errno));
			break;
		}

		/* make sure the loop will end after dump finished */
		if (dump_finished) {
			if ((bytes_copied == 0) || (bytes_remain >= bytes_remain_last)) {
				break;
			}
			bytes_remain_last = bytes_remain;
		}
	}

out:
	if (buf) {
		free(buf);
	}

	return retval;
}

static int
call_qcsapi_get_client_mac_list(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *intf = p_calling_bundle->caller_interface;
	int index = (argc >= 1) ? atoi(argv[0]) : 0;
	struct qcsapi_mac_list *mlist = NULL;

	if (argc != 1) {
		print_err(print, "Usage: call_qcsapi get_client_mac_list <interface> <node_idx>\n");
		return statval;
	}

	mlist = calloc(1, sizeof(struct qcsapi_mac_list));
	qcsapi_retval = qcsapi_get_client_mac_list(intf, index, mlist);
	if (qcsapi_retval >= 0) {
		if ((verbose_flag >= 0) && (mlist->num_entries)) {
			int i, k;
			if (mlist->flags & 0x2) {
				print_out(print, "Node supports 4 address\n");
			}
			if (mlist->flags & 0x1) {
				print_out(print, "Results are truncated to Max[%d]\n",
						QCSAPI_MAX_MACS_IN_LIST);
			}

			for (i = 0, k = 0; i < mlist->num_entries; i++, k += 6)
				print_out(print, "\t" MACSTR " \n", MAC2STR(&mlist->macaddr[k]));
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}
	free(mlist);
	return statval;
}

static int
call_qcsapi_wifi_sample_all_clients(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint8_t sta_count = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *intf = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_sample_all_clients(intf, &sta_count);
	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", sta_count);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define MAX_ASSOC_STA 2007
static int
call_qcsapi_wifi_get_per_assoc_data(const call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int num_entry;
	int offset;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *intf = p_calling_bundle->caller_interface;
	struct qcsapi_sample_assoc_data *data;
	char ip_str[IP_ADDR_STR_LEN + 1];
	int i;
	int j;

	if (argc < 2) {
		qcsapi_report_usage(p_calling_bundle, "<interface> <num_entry> <offset>\n");
		return 1;
	}

	num_entry = atoi(argv[0]);
	if ((num_entry < 1) || (num_entry > MAX_ASSOC_STA))
		num_entry = MAX_ASSOC_STA;

	offset = atoi(argv[1]);

	data = calloc(num_entry, sizeof(struct qcsapi_sample_assoc_data));

	if (data == NULL)
		qcsapi_retval = -EFAULT;

	if (qcsapi_retval >= 0)
		qcsapi_retval = qcsapi_wifi_get_per_assoc_data(intf, data, num_entry, offset);

	if (qcsapi_retval >= 0) {
		for (i = 0; i < num_entry; i++) {
			print_out(print, "Assoc ID: %u\nMacaddr: " MACSTR "\nTx: %u\nRx: %u\n"
					"Tx_rate(Max): %u\nRx_rate(Max): %u\n"
					"Mode: %s\nBw: %u\nAssoc_time: %usec\n",
					data[i].assoc_id,
					MAC2STR(data[i].mac_addr),
					data[i].tx_stream,
					data[i].rx_stream,
					data[i].achievable_tx_phy_rate,
					data[i].achievable_rx_phy_rate,
					qcsapi_wifi_modes_strings[data[i].protocol],
					data[i].bw, data[i].time_associated);
			print_out(print, "Rx_bytes: %llu\nTx_bytes: %llu\nRx_pkts: %u\n"
					"Tx_pkts: %u\nRx_errors: %u\nTx_errors: %u\n"
					"Rx_dropped %u\nTx_dropped: %u\nRx_ucast: %u\n"
					"Tx_ucast: %u\nRx_mcast: %u\nTx_mcast: %u\n"
					"Rx_bcast: %u\nTx_bcast: %u\nLink_quality: %u\n",
					data[i].rx_bytes,
					data[i].tx_bytes,
					data[i].rx_packets,
					data[i].tx_packets,
					data[i].rx_errors,
					data[i].tx_errors,
					data[i].rx_dropped,
					data[i].tx_dropped,
					data[i].rx_ucast,
					data[i].tx_ucast,
					data[i].rx_mcast,
					data[i].tx_mcast,
					data[i].rx_bcast, data[i].tx_bcast, data[i].link_quality);
			print_out(print, "tx_wifi_drop: %u %u %u %u\n",
					data[i].tx_wifi_drop[WMM_AC_BE],
					data[i].tx_wifi_drop[WMM_AC_BK],
					data[i].tx_wifi_drop[WMM_AC_VI],
					data[i].tx_wifi_drop[WMM_AC_VO]);

			print_out(print, "\nRSSI\t RCPI\t EVM\t HW_NOISE\n");
			for (j = 0; j < QCSAPI_NUM_ANT; j++) {
				if (j == (QCSAPI_NUM_ANT - 1))
					print_out(print, "\n(AVG)\t(Max)\t(Sum)\t(Avg)\n");

				print_out(print, "%4d.%d\t",
						((int)(data[i].last_rssi_dbm[j])) / 10,
						abs((int)data[i].last_rssi_dbm[j]) % 10);

				print_out(print, "%4d.%d\t",
						((int)(data[i].last_rcpi_dbm[j])) / 10,
						abs((int)data[i].last_rcpi_dbm[j]) % 10);
				print_out(print, "%4d.%d\t",
						((int)(data[i].last_evm_dbm[j])) / 10,
						abs(((int)data[i].last_evm_dbm[j])) % 10);

				print_out(print, "%4d.%d\n",
						((int)(data[i].last_evm_dbm[j])) / 10,
						abs(((int)data[i].last_evm_dbm[j])) % 10);
			}

			switch (data[i].vendor) {
			case PEER_VENDOR_QTN:
				print_out(print, "vendor: quantenna\n");
				break;
			case PEER_VENDOR_BRCM:
				print_out(print, "vendor: broadcom\n");
				break;
			case PEER_VENDOR_ATH:
				print_out(print, "vendor: atheros\n");
				break;
			case PEER_VENDOR_RLNK:
				print_out(print, "vendor: ralink\n");
				break;
			case PEER_VENDOR_RTK:
				print_out(print, "vendor: realtek\n");
				break;
			case PEER_VENDOR_INTEL:
				print_out(print, "vendor: intel\n");
				break;
			default:
				print_out(print, "vendor: unknown\n");
			}

			inet_ntop(AF_INET, &data[i].ip_addr, ip_str, IP_ADDR_STR_LEN);
			print_out(print, "Ipaddr: %s\n\n", ip_str);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	if (data)
		free(data);

	return statval;
}

static int call_qcsapi_get_wifi_ready(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_unsigned_int wifi_ready = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;

	qcsapi_retval = qcsapi_wifi_is_iface_ready(the_interface, &wifi_ready);

	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", wifi_ready);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_get_cca_stats(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_cca_stats stats;

	qcsapi_retval = qcsapi_get_cca_stats(the_interface, &stats);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "cca_occupy=\t%u\n"
					"cca_intf=\t%u\n"
					"cca_trfc=\t%u\n"
					"cca_tx=\t\t%u\n"
					"cca_rx=\t\t%u\n",
					stats.cca_occupy,
					stats.cca_intf, stats.cca_trfc, stats.cca_tx, stats.cca_rx);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_vapdebug(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int bitmap = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if ((argc != 1) || (sscanf(argv[0], "%i", &bitmap) != 1)) {
		print_err(print, "Usage: call_qcsapi set_vapdebug <interface> <bitmap>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_vapdebug(the_interface, bitmap);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "complete\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_get_vapdebug(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	int bitmap = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_vapdebug(the_interface, &bitmap);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			print_out(print, "%#x\n", bitmap);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_get_igmp_snoop(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t igmp_snooping_state = 0;
	qcsapi_output *print = call->caller_output;
	const char *the_interface = call->caller_interface;
	char *usage = "Usage: call_qcsapi get_igmp_snoop <bridge interface>\n";

	if (argc >= 1) {
		print_out(print, usage);
		statval = 1;
	} else {
		qcsapi_retval = qcsapi_get_igmp_snooping_state(the_interface, &igmp_snooping_state);

		if (qcsapi_retval >= 0) {
			print_out(print, "%u\n", igmp_snooping_state);
		} else {
			report_qcsapi_error(call, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_set_igmp_snoop(call_qcsapi_bundle *call, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t igmp_snooping_state = 0;
	const char *the_interface = call->caller_interface;

	if (isdigit(*argv[0])) {
		switch (atoi(argv[0])) {
		case QCSAPI_IGMP_SNOOPING_ENABLE:
			igmp_snooping_state = QCSAPI_IGMP_SNOOPING_ENABLE;
			break;
		case QCSAPI_IGMP_SNOOPING_DISABLE:
			igmp_snooping_state = QCSAPI_IGMP_SNOOPING_DISABLE;
			break;
		default:
			qcsapi_retval = -EINVAL;
		}
	} else {
		qcsapi_retval = -EINVAL;
	}

	if (qcsapi_retval == 0) {
		qcsapi_retval = qcsapi_set_igmp_snooping_state(the_interface, igmp_snooping_state);
	}

	if (qcsapi_retval < 0) {
		report_qcsapi_error(call, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int call_qcsapi_br_get_groups(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_2048 *list = NULL;

	list = calloc(sizeof(char), sizeof(string_2048));
	if (list == NULL) {
		print_err(print, "alloc memory failed\n");
		return 1;
	}

	qcsapi_retval = qcsapi_br_get_groups(*list);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", *list);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	free(list);

	return statval;
}

static int call_qcsapi_br_get_interfaces(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_2048 *list = NULL;

	list = calloc(sizeof(char), sizeof(string_2048));
	if (list == NULL) {
		print_err(print, "alloc memory failed\n");
		return 1;
	}

	qcsapi_retval = qcsapi_br_get_interfaces(the_interface, *list);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", *list);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	free(list);

	return statval;
}

static int name_to_bsa_param_enum(char *lookup_name, qcsapi_bsa_conf_param *p_bsa_param)
{
	unsigned int iter;
	for (iter = 0; bsa_config[iter].bsa_name != NULL; iter++) {
		if (strcasecmp(bsa_config[iter].bsa_name, lookup_name) == 0) {
			*p_bsa_param = bsa_config[iter].bsa_enum;
			return 1;
		}
	}
	return 0;
}

void display_get_bsa_param_usage(qcsapi_output *print)
{
	print_out(print, "Usage: call_qcsapi get_bsa_param_ext <mobility_domain> <bsa_param>\n");
	print_out(print, "Try 'call_qcsapi get_bsa_param_ext --help' for complete list of "
			"<bsa_param> supported\n");
}

void display_set_bsa_param_usage(qcsapi_output *print)
{
	print_out(print, "Usage: call_qcsapi set_bsa_param_ext <mobility_domain> <bsa_param> <value>\n");
	print_out(print, "Try 'call_qcsapi set_bsa_param_ext --help' for complete list of "
			"<bsa_param> supported\n");
}

static int call_qcsapi_bsa_get_parameter(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	char store_bsa_param[QCSAPI_MSG_BUFSIZE] = { 0 };
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_bsa_conf_param bsa_param = -1;

	if (argc != 1) {
		print_out(print, "Usage: call_qcsapi get_bsa_param  <bsa_param>\n");
		return 1;
	}

	if ((argv[0]) && (strcmp(argv[0], "--help") == 0)) {
		display_get_bsa_param_usage(print);
		return 0;
	}

	if ((name_to_bsa_param_enum(argv[0], &bsa_param) == 0)
			|| (bsa_param == qcsapi_bsa_reload_new_config)) {
		display_get_bsa_param_usage(print);
		return 0;
	}

	qcsapi_retval = qcsapi_bsa_get_parameter(bsa_param, store_bsa_param);

	if (qcsapi_retval == E_PARAMETER_FOUND)
		print_out(print, "%s", store_bsa_param);
	else {
		if (qcsapi_retval == E_PARAMETER_NOT_FOUND)
			qcsapi_retval = -qcsapi_parameter_not_found;
		if (qcsapi_retval == E_PARAMETER_INVALID)
			qcsapi_retval = -qcsapi_param_value_invalid;
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}
	return 0;
}

void display_bsa_param_usage(qcsapi_output *print)
{
	print_out(print, "NOTE: The value ranges below are experimental and subject to change\n");
	print_out(print, "Usage:\n");
	print_out(print, "    call_qcsapi set_bsa_param_ext <mobility_domain> <bsa_param> <value>\n");
	print_out(print, "    call_qcsapi get_bsa_param_ext <mobility_domain> <bsa_param>\n");
	print_out(print, "\n");
	print_out(print, "Parameter\t\t\tValue\t\t\t# Notes\n");
	print_out(print, "    GLOBAL Parameters---------------------------------- # global parameters not limited to mobility domain\n" "    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* enable */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* house_keeping_interval */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* stale_entry_interval */
			"    %-27s {%d - %d}\t\t# \n"	/* clear_dual_band_interval */
			"    %-27s <mac addr> {0 | 1}\t# 0 (delete) or 1 (add)\n"	/* whitelist_sta_entry */
			"    %-27s {0 - %d}\t\t\t#\n"	/* bsa_debug_level */
			"    %-27s {0 - 1}\t\t\t#\n"	/* bsa_debug_qevt */
			"    Per Mobile domain Parameters----------------------- # ssid parameters for each mobility domain\n" "    %-27s {0 - 1}\t\t\t#\n"	/* ssid_pair_status */
			"    %-27s <ssid>\t\t\t# max 32 characters\n"	/* bs_ssid */
			"    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* allow_5g_11ac_only */
			"    %-27s {%d - %d}\t\t\t#\n"	/* non_steerable_count */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* blacklist_timeout_val */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* inactive_assoc_interval */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* inactive_runtime_interval */
			"    %-27s {%d - %d}\t\t# packets/second\n"	/* client_inactivity_threshold */
			"    %-27s {%d - %d}\t\t# seconds\n"	/* runtime_decision_interval */
			"    %-27s {%d - %d}\t\t#\n"	/* no_steering_interval */
			"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_loaded_fat */
			"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_unloaded_fat */
			"    %-27s {%d - %d}\t\t#\n"	/* run_5g_loaded_fat */
			"    %-27s {%d - %d}\t\t#\n"	/* run_5g_unloaded_fat */
			"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_loaded_min_phyrate */
			"    %-27s {%d - %d}\t\t#\n"	/* assoc_5g_min_phyrate */
			"    %-27s {%d - %d}\t\t#\n"	/* run_5g_loaded_min_phyrate */
			"    %-27s {%d - %d}\t\t#\n"	/* run_5g_unloaded_min_phyrate */
			"    %-27s {%d - %d}\t\t# dBm\n"	/* assoc_5g_loaded_min_rssi */
			"    %-27s {%d - %d}\t\t# dBm\n"	/* assoc_5g_min_rssi */
			"    %-27s {0 | 1}\t\t\t# 0 (disable) or 1 (enable)\n"	/* ssid_check */
			"    %-27s\t\t\t\t# Trigger BSA App to reload new config\n"	/* bsa_reload_new_config */
			"    %-27s\t\t\t\t# restart BSA module\n"	/* bsa_module_restart */
			"\n"
			"Note: FAT (Free Air Time) values are in tenths of a percent. E.g. 450 is 45%%\n",
			bsa_config[qcsapi_enable].bsa_name,
			bsa_config[qcsapi_house_keeping_interval].bsa_name, BSA_HOUSE_KEEP_MIN,
			BSA_HOUSE_KEEP_MAX, bsa_config[qcsapi_stale_entry_interval].bsa_name,
			BSA_STALE_ENTRY_INTERVAL_MIN, BSA_STALE_ENTRY_INTERVAL_MAX,
			bsa_config[qcsapi_clear_dual_band_interval].bsa_name, BSA_CLEAR_DUAL_BAND_MIN, BSA_CLEAR_DUAL_BAND_MAX,
			bsa_config[qcsapi_whitelist_sta_entry].bsa_name,
			bsa_config[qcsapi_bsa_debug_level].bsa_name, BSA_DEBUG_LEVEL_MAX,
			bsa_config[qcsapi_bsa_debug_qevt].bsa_name,
			bsa_config[qcsapi_ssid_pair_status].bsa_name,
			bsa_config[qcsapi_bs_ssid].bsa_name,
			bsa_config[qcsapi_allow_5g_11ac_only].bsa_name,
			bsa_config[qcsapi_non_steerable_count].bsa_name, BSA_NON_STEERABLE_CNT_MIN,
			BSA_NON_STEERABLE_CNT_MAX,
			bsa_config[qcsapi_blacklist_timeout_val].bsa_name,
			BSA_BLACKLIST_TIMEOUT_MIN, BSA_BLACKLIST_TIMEOUT_MAX,
			bsa_config[qcsapi_inactive_assoc_interval].bsa_name,
			BSA_INACTV_ASSOC_INTERVAL_MIN, BSA_INACTV_ASSOC_INTERVAL_MAX,
			bsa_config[qcsapi_inactive_runtime_interval].bsa_name,
			BSA_INACTV_RUNTIME_INTERVAL_MIN, BSA_INACTV_RUNTIME_INTERVAL_MAX,
			bsa_config[qcsapi_client_inactivity_threshold].bsa_name,
			BSA_CLI_INACTV_THRSHLD_MIN, BSA_CLI_INACTV_THRSHLD_MAX,
			bsa_config[qcsapi_runtime_decision_interval].bsa_name,
			BSA_RUNTIME_DECISION_INTV_MIN, BSA_RUNTIME_DECISION_INTV_MAX,
			bsa_config[qcsapi_no_steering_interval].bsa_name, BSA_NO_STEERING_INTV_MIN,
			BSA_NO_STEERING_INTV_MAX, bsa_config[qcsapi_assoc_5g_loaded_fat].bsa_name,
			BSA_ASSOC_5G_LOAD_FAT_MIN, BSA_ASSOC_5G_LOAD_FAT_MAX,
			bsa_config[qcsapi_assoc_5g_unloaded_fat].bsa_name,
			BSA_ASSOC_5G_UNLOAD_FAT_MIN, BSA_ASSOC_5G_UNLOAD_FAT_MAX,
			bsa_config[qcsapi_run_5g_loaded_fat].bsa_name, BSA_RUN_5G_LOAD_FAT_MIN,
			BSA_RUN_5G_LOAD_FAT_MAX, bsa_config[qcsapi_run_5g_unloaded_fat].bsa_name,
			BSA_RUN_5G_UNLOAD_FAT_MIN, BSA_RUN_5G_UNLOAD_FAT_MAX,
			bsa_config[qcsapi_assoc_5g_loaded_min_phyrate].bsa_name,
			BSA_ASSOC_5G_LOAD_PHYRATE_MIN, BSA_ASSOC_5G_LOAD_PHYRATE_MAX,
			bsa_config[qcsapi_assoc_5g_min_phyrate].bsa_name, BSA_ASSOC_5G_PHYRATE_MIN,
			BSA_ASSOC_5G_PHYRATE_MAX,
			bsa_config[qcsapi_run_5g_loaded_min_phyrate].bsa_name,
			BSA_RUN_5G_LOAD_PHYRATE_MIN, BSA_RUN_5G_LOAD_PHYRATE_MAX,
			bsa_config[qcsapi_run_5g_unloaded_min_phyrate].bsa_name,
			BSA_RUN_5G_UNLOAD_PHYRATE_MIN, BSA_RUN_5G_UNLOAD_PHYRATE_MAX,
			bsa_config[qcsapi_assoc_5g_loaded_min_rssi].bsa_name,
			BSA_ASSOC_5G_LOAD_RSSI_MIN, BSA_ASSOC_5G_LOAD_RSSI_MAX,
			bsa_config[qcsapi_assoc_5g_min_rssi].bsa_name, BSA_ASSOC_5G_RSSI_MIN,
			BSA_ASSOC_5G_RSSI_MAX, bsa_config[qcsapi_ssid_check].bsa_name,
			bsa_config[qcsapi_bsa_reload_new_config].bsa_name,
			bsa_config[qcsapi_bsa_module_restart].bsa_name);
}

static int call_qcsapi_bsa_set_parameter(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	const char *param_value1 = NULL, *param_value2 = NULL;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_bsa_conf_param bsa_param = -1;
	const char *usage = "Usage:\n"
			"    call_qcsapi set_bsa_param <bsa_param> <value>\n"
			"    call_qcsapi set_bsa_param --help\n";

	if ((argc < 2)) {
		if (argv[0]) {
			if (strcmp(argv[0], "--help") == 0) {
				display_bsa_param_usage(print);
				return 0;
			}
			if (strcmp(argv[0], bsa_config[qcsapi_bsa_reload_new_config].bsa_name) == 0) {
				qcsapi_retval = qcsapi_bsa_set_parameter
						(qcsapi_bsa_reload_new_config, NULL, NULL);
				return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
			}
		}
		print_out(print, "%s", usage);
		return 1;
	}

	if (name_to_bsa_param_enum(argv[0], &bsa_param) == 0) {
		print_out(print, "%s", usage);
	}

	param_value1 = argv[1];
	if (argc > 2)
		param_value2 = argv[2];

	qcsapi_retval = qcsapi_bsa_set_param(bsa_param, param_value1, param_value2);
	if (qcsapi_retval == -qcsapi_config_update_failed)
		print_out(print, "BSA App is active. Operation not allowed.\n");
	else if (qcsapi_retval == -qcsapi_param_value_invalid)
		print_out(print, "Invalid value. 'call_qcsapi set_bsa_param --help'"
				" to find proper value\n");

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int call_qcsapi_bsa_get_parameter_ext(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_bsa_conf_param bsa_param = -1;
	uint32_t bsa_mb_domain;

	if ((argv[0]) && (strcmp(argv[0], "--help") == 0)) {
		display_bsa_param_usage(print);
		return 0;
	}

	if (argc != 2) {
		display_get_bsa_param_usage(print);
		statval = 1;
		return statval;
	} else {
		if (0 == safe_atou32(argv[0], &bsa_mb_domain, print, 0, BSA_PAIRS_NUM_MAX))
			return 1;

		if (name_to_bsa_param_enum(argv[1], &bsa_param) == 0) {
			display_get_bsa_param_usage(print);
			return 1;
		}

		if (((!bsa_mb_domain) && (bsa_param <= qcsapi_bsa_global_param_end)) ||
				((bsa_mb_domain) &&
						(bsa_param <= qcsapi_mb_domain_parameter_end)
						&& (bsa_param > qcsapi_bsa_global_param_end))) {
			char store_bsa_param[QCSAPI_MSG_BUFSIZE] = { 0 };
			char *p_store = &store_bsa_param[0];

			qcsapi_retval = qcsapi_bsa_get_parameter_ext(bsa_mb_domain, bsa_param,
					p_store);
			if (qcsapi_retval == E_PARAMETER_FOUND) {
				print_out(print, "%s", p_store);
			} else {
				if (qcsapi_retval == E_PARAMETER_NOT_FOUND)
					qcsapi_retval = -qcsapi_parameter_not_found;
				if (qcsapi_retval == E_PARAMETER_INVALID)
					qcsapi_retval = -qcsapi_param_value_invalid;
				report_qcsapi_error(p_calling_bundle, qcsapi_retval);
				statval = 1;
			}
		} else {
			print_out(print, "Invalid <bsa_param>\n");
			display_get_bsa_param_usage(print);
			statval = 1;
		}
	}

	return statval;
}

static int call_qcsapi_bsa_set_parameter_ext(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	char *param_value1 = NULL;
	char *param_value2 = NULL;
	uint32_t bsa_mb_domain = 0;
	int cmd_err = 1;
	qcsapi_bsa_conf_param bsa_param;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if ((argv[0]) && (strcmp(argv[0], "--help") == 0)) {
		display_bsa_param_usage(print);
		return 0;
	}

	if ((argc < 2)) {
		print_out(print, "Too few command line parameters in call_qcsapi\n");
		display_set_bsa_param_usage(print);
		return 1;
	}

	if (0 == safe_atou32(argv[0], &bsa_mb_domain, print, 0, BSA_MB_DOMAIN_ALL))
		return 1;

	if (name_to_bsa_param_enum(argv[1], &bsa_param) == 0) {
		display_set_bsa_param_usage(print);
		return 1;
	}

	if (argc > 2)
		param_value1 = argv[2];

	if (argc > 3)
		param_value2 = argv[3];

	if (!bsa_mb_domain) {
		if ((bsa_param <= qcsapi_bsa_global_param_end) && (argc >= 3)) {
			cmd_err = 0;
		} else if ((bsa_param <= qcsapi_bsa_action_parameter_end)
				&& (bsa_param > qcsapi_mb_domain_parameter_end) && (argc == 2)) {
			cmd_err = 0;
		}
	} else if ((bsa_mb_domain <= BSA_PAIRS_NUM_MAX) || (bsa_mb_domain == BSA_MB_DOMAIN_ALL)) {
		if ((bsa_param > qcsapi_bsa_global_param_end)
				&& (bsa_param <= qcsapi_mb_domain_parameter_end) && (argc >= 3)) {
			cmd_err = 0;
		}
	}

	if (cmd_err) {
		print_out(print, "Invalid <bsa_param value>\n");
		display_set_bsa_param_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_bsa_set_param_ext(bsa_mb_domain, bsa_param, param_value1,
			param_value2);
	if (qcsapi_retval == -qcsapi_config_update_failed)
		print_out(print, "BSA App is active. Operation not allowed.\n");
	else if (qcsapi_retval == -qcsapi_param_value_invalid)
		print_out(print, "Invalid value. 'call_qcsapi set_bsa_param_ext --help'"
				" to find proper value\n");

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static void display_set_radio_pwr_save_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "	call_qcsapi set_radio_pwr_save <radio_id> <force_power_save>\n");
	print_out(print, "Parameter\t\t\tValue\t# Notes\n");
	print_out(print, "\t<radio_id>\t\t{0|2}\t# 0 for 5G or 2 for 2.4G\n"
			"\t<force_power_save>\t{1|0}\t# 1 (force radio enter power save) or 0 (force radio exit power save)\n");
}

static int call_qcsapi_set_radio_pwr_save(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t force_pwr_save;
	uint32_t radio;

	if (argc != 2) {
		display_set_radio_pwr_save_usage(print);
		return 0;
	}

	if (qcsapi_str_to_uint32(argv[0], &radio) < 0) {
		display_set_radio_pwr_save_usage(print);
		return 1;
	}

	if (radio != 0 && radio != 2) {
		display_set_radio_pwr_save_usage(print);
		return 1;
	}

	if (qcsapi_str_to_uint32(argv[1], &force_pwr_save) < 0) {
		display_set_radio_pwr_save_usage(print);
		return 1;
	}

	if (force_pwr_save != 0 && force_pwr_save != 1) {
		display_set_radio_pwr_save_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_set_radio_pwr_save(radio, force_pwr_save);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static void display_put_radio_under_reset_usage(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "	call_qcsapi put_radio_under_reset <radio_id>\n");
	print_out(print, "Parameter\t\t\tValue\t# Notes\n");
	print_out(print, "\t<radio_id>\t\t{0|2}\t# 0 for 5G or 2 for 2.4G\n");
}

static int call_qcsapi_put_radio_under_reset(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int qcsapi_retval = 0;
	uint32_t radio;

	if (argc != 1) {
		display_put_radio_under_reset_usage(print);
		return 0;
	}

	if (qcsapi_str_to_uint32(argv[0], &radio) < 0) {
		display_put_radio_under_reset_usage(print);
		return 1;
	}

	if (radio != 0 && radio != 2) {
		display_put_radio_under_reset_usage(print);
		return 1;
	}

	qcsapi_retval = qcsapi_put_radio_under_reset(radio);
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

#define BSA_INVALID_BAND		2
static int print_sta_cnt = 0;
static void
dump_bsa_sta_table(qcsapi_output *print, const struct qcsapi_data_3Kbytes *buffer, int sta_cnt,
		int is_assoc)
{
	int i;
	char *pos;
	struct bsa_sta_entry *psta;
	char int2str_buf[20][16];

	pos = (char *)buffer->data;

	for (i = 0; i < sta_cnt; i++) {
		psta = (struct bsa_sta_entry *)pos;
		pos += sizeof(struct bsa_sta_entry);

		if (is_assoc && psta->curr_band == BSA_INVALID_BAND)
			continue;

		if ((print_sta_cnt % 20) == 0) {
			print_out(print, "\n"
					"MAC_Adress        Assoc 2G+5G    Not    |"
					"   LastSteeringAttempt   |"
					" SteerStats |"
					"         5G_CapabilitiesAndAssocStats         |"
					"   2G_CapabilitiesAndAssocStats\n");
			print_out(print, "                              Steerable |"
					" Target  Mode  time  phy |"
					"  OK   Fail |"
					" RSSI    CAP VHT 11v MU max_phy phy_RX phy_TX |"
					" RSSI    CAP max_phy phy_RX phy_TX\n");
		}
#define OP_YORN(con)			((con) ? "Y" : "N")
#define OP_INT2STR(str, val, format)	(val ? ((sprintf(str, format, val) > 0) ? str : "-") : "-")
#define OP_BAND_STR(band)		((band == 0) ? "2G" : ((band == 1) ? "5G" : "-"))
#define OP_STEER_MODE_STR(mode)		((mode == 1) ? "assoc" : ((mode == 2) ? "runtime" : "-"))

		print_out(print, MACSTR " %5s %5s %9s |"
				" %3s %7s %6s %4s |"
				" %4d %4d  |"
				" %4s %6s %3s %3s %2s %7s %6s %6s |"
				" %4s %#6s %7s %6s %6s\n",
				MAC2STR(psta->sta_mac), OP_BAND_STR(psta->curr_band),
				OP_YORN(psta->dual_band), OP_YORN(psta->not_steerable),
				OP_BAND_STR(psta->target_band),
				OP_STEER_MODE_STR(psta->steering_mode), OP_INT2STR(int2str_buf[0],
						(uint32_t) psta->ts_steering_decision, "%u"),
				OP_INT2STR(int2str_buf[1], psta->steer_phyrate, "%u"),
				psta->steer_counter, psta->failed_steer_total,
				OP_INT2STR(int2str_buf[2], psta->info_5g.last_rssi, "%d"),
				OP_INT2STR(int2str_buf[3], psta->info_5g.sta_capab, "%#x"),
				OP_YORN(psta->info_5g.phy_mode_supported),
				OP_YORN(psta->info_5g.btm_11v_support),
				OP_YORN(psta->info_5g.mumimo_support), OP_INT2STR(int2str_buf[4],
						psta->info_5g.supported_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[5], psta->info_5g.avg_tx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[6], psta->info_5g.avg_rx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[7], psta->info_2g.last_rssi, "%d"),
				OP_INT2STR(int2str_buf[8], psta->info_2g.sta_capab, "%#x"),
				OP_INT2STR(int2str_buf[9], psta->info_2g.supported_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[10], psta->info_2g.avg_tx_phy_rate, "%u"),
				OP_INT2STR(int2str_buf[11], psta->info_2g.avg_rx_phy_rate, "%u"));
		print_sta_cnt++;
	}
}

static int
call_qcsapi_bsa_get_sta_table(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_data_3Kbytes *buf = NULL;
	unsigned int sta_cnt = 0;
	unsigned int total_sta_cnt = 0;
	unsigned int remain_sta_cnt = 0;
	unsigned short session_id = 0;
	int sta_assoc = 1;

	if (argc > 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "[all|assoc]");
		return 1;
	} else if (argc == 1) {
		if (strcasecmp(argv[0], "all") == 0) {
			sta_assoc = 0;
		} else if (strcasecmp(argv[0], "assoc") == 0) {
			sta_assoc = 1;
		} else {
			qcsapi_report_usage(p_calling_bundle, "[all|assoc]");
			return 1;
		}
	}

	buf = malloc(sizeof(struct qcsapi_data_3Kbytes));
	if (!buf) {
		print_err(print, "allocate memory failed\n");
		statval = 1;
		return statval;
	}

	qcsapi_retval = qcsapi_bsa_start_dump_sta_table(&session_id, &total_sta_cnt);
	if (qcsapi_retval >= 0) {
		if (total_sta_cnt == 0) {
			print_out(print, "Station table of BSA is empty\n");
		} else {
			print_sta_cnt = 0;
			while (remain_sta_cnt < total_sta_cnt) {
				qcsapi_retval = qcsapi_bsa_get_sta_table_item(session_id, buf,
						&sta_cnt);
				if (qcsapi_retval < 0 || sta_cnt == 0)
					break;
				dump_bsa_sta_table(print, buf, sta_cnt, sta_assoc);
				remain_sta_cnt += sta_cnt;
			}	/* while (remain_sta_cnt < total_sta_cnt) */

			if (!print_sta_cnt)
				print_out(print, "\nBSA %s client table is empty\n",
						sta_assoc ? "Assoc" : "");
		}		/* if (total_sta_cnt == 0) */
		qcsapi_retval = qcsapi_bsa_stop_dump_sta_table(session_id);
	}
	/* if (qcsapi_retval >= 0) */
	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	if (NULL != buf)
		free(buf);

	return statval;
}

static int
call_qcsapi_set_hw_module_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int hw_module_state = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_hw_module hw_module =
			p_calling_bundle->caller_generic_parameter.parameter_type.hw_module;

	if (argc < 1) {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<interface> pm_signal 1|0");
		statval = 1;
	} else {
		if ((strcasecmp(argv[0], "TRUE") == 0) || (strcasecmp(argv[0], "YES") == 0) ||
				(strcmp(argv[0], "1") == 0)) {
			hw_module_state = 1;
		} else if ((strcasecmp(argv[0], "FALSE") == 0) || (strcasecmp(argv[0], "NO") == 0)
				|| (strcmp(argv[0], "0") == 0)) {
			hw_module_state = 0;
		} else {
			print_err(print, "Invalid input arguments\n");
			return 1;
		}

		qcsapi_retval = qcsapi_qdrv_set_hw_module_state(hw_module, hw_module_state);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_get_hw_module_state(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_unsigned_int hw_module_state = 0, *p_hw_module_state = NULL;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_hw_module hw_module =
			p_calling_bundle->caller_generic_parameter.parameter_type.hw_module;

	if (argc < 1 || strcmp(argv[0], "NULL") != 0) {
		p_hw_module_state = &hw_module_state;
	} else {
		qcsapi_report_parameter_count(p_calling_bundle, argc);
		qcsapi_report_usage(p_calling_bundle, "<interface> pm_signal");
		return 1;
	}

	qcsapi_retval = qcsapi_qdrv_get_hw_module_state(hw_module, p_hw_module_state);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			if (hw_module_state == 0)
				print_out(print, "FALSE\n");
			else
				print_out(print, "TRUE\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_wifi_set_ieee80211r(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> {0 | 1}\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ieee80211r_str(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		if (qcsapi_retval == -qcsapi_option_not_supported) {
			print_out(print, "802.11r only supported with WPA2-PSK and WPA2-EAP modes\n");
		}
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_ieee80211r(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	string_16 value = { 0 };
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_ieee80211r(the_interface, &value[0]);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", ((atoi(value) == 1) ? "enabled" : "disabled"));
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_11r_mobility_domain(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <mobility_domain>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ieee80211r_mobility_domain_str(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_mobility_domain(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	string_16 value = { 0 };
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_ieee80211r_mobility_domain(the_interface, &value[0]);

	if (qcsapi_retval >= 0)
		print_out(print, "%s\n", value);
	else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static void call_qcsapi_check_11r_retval(qcsapi_output *print, int qcsapi_retval)
{
	if (qcsapi_retval == -qcsapi_option_not_supported)
		print_out(print, "Configuration only supported when 802.11r is enabled\n");
}

static int
call_qcsapi_wifi_set_11r_nas_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <nas_id>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ieee80211r_nas_id(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		call_qcsapi_check_11r_retval(print, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_nas_id(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	string_64 value = { 0 };
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_ieee80211r_nas_id_64(the_interface, &value[0]);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_11r_ft_over_ds(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> {0 | 1}\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_ieee80211r_ft_over_ds_str(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		call_qcsapi_check_11r_retval(print, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_ft_over_ds(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	string_16 value = { 0 };
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_ieee80211r_ft_over_ds(the_interface, &value[0]);

	if (qcsapi_retval >= 0)
		print_out(print, "%s\n", ((atoi(value) == 1) ? "enabled" : "disabled"));
	else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_add_11r_neighbour(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 4) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <MAC> "
				"<NAS Identifier> <128-bit key as hex string> <R1KH-ID(mac address)>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_add_11r_neighbour_str(the_interface, argv[0], argv[1], argv[2],
			argv[3]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		call_qcsapi_check_11r_retval(print, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_del_11r_neighbour(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <MAC>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_del_11r_neighbour_str(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_neighbour(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{

	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	string_4096 buf = { 0 };
	int buflen = sizeof(string_4096);

	qcsapi_retval = qcsapi_wifi_get_11r_neighbour(the_interface, buf, buflen);

	return qcsapi_report_str_or_error(p_calling_bundle, qcsapi_retval, buf);
}

static int
call_qcsapi_wifi_set_11r_r1_key_holder(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle,
				"<WiFi interface> <r1_key_holder | 0 (delete)>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_11r_r1_key_holder_str(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		call_qcsapi_check_11r_retval(print, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_r1_key_holder(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	string_16 value = { 0 };

	qcsapi_retval = qcsapi_wifi_get_11r_r1_key_holder(the_interface, &value[0]);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_11r_r0_key_lifetime(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		qcsapi_report_usage(p_calling_bundle, "<WiFi interface> <value>\n");
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_11r_r0_key_lifetime(the_interface, argv[0]);

	if (qcsapi_retval < 0) {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		call_qcsapi_check_11r_retval(print, qcsapi_retval);
		return 1;
	}

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_wifi_get_11r_r0_key_lifetime(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	string_128 value = { 0 };
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_11r_r0_key_lifetime(the_interface, value);

	if (qcsapi_retval >= 0) {
		print_out(print, "%s\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{

	int statval = 0;
	int qcsapi_retval = 0;
	uint32_t value = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call_qcsapi set_scs_leavedfs_chan_mtrc_mrgn, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_scs_leavedfs_chan_mtrc_mrgn <Wifi interface> " "<channel metric margin>\n");
		statval = 1;
	} else {
		if (0 == safe_atou32(argv[0], &value, print, 0, IEEE80211_SCS_CHAN_MTRC_MRGN_MAX)) {
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn(the_interface, value);

		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0) {
				print_out(print, "complete\n");
			}
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}

	return statval;
}

static int
call_qcsapi_set_max_boot_cac_duration(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	int max_boot_cac_duration;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call_qcsapi set_max_boot_cac_duration, count is %d\n", argc);
		print_err(print, "Usage: call_qcsapi set_max_boot_cac_duration <WiFi interface> -1(disabled)\n" "call_qcsapi set_max_boot_cac_duration <WiFi interface> 0\n" "call_qcsapi set_max_boot_cac_duration <WiFi interface> <seconds>\n");
		statval = 1;
	} else {
		if (atoi(argv[0]) == -1) {
			max_boot_cac_duration = -1;
		} else if ((safe_atou32(argv[0], (uint32_t *) & max_boot_cac_duration, print, 0,
								MAX_BOOT_CAC_DURATION)) == 0) {
			print_err(print, "Usage: call_qcsapi set_max_boot_cac_duration <WiFi interface> -1(disabled)\n" "call_qcsapi set_max_boot_cac_duration <WiFi interface> 0\n" "call_qcsapi set_max_boot_cac_duration <WiFi interface> <seconds>\n");
			return 1;
		}

		qcsapi_retval = qcsapi_wifi_set_max_boot_cac_duration(the_interface,
				max_boot_cac_duration);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0)
				print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = 1;
		}
	}
	return statval;
}

static int
call_qcsapi_get_icac_status(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	int status = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_icac_status(the_interface, &status);

	if (qcsapi_retval >= 0)
		print_out(print, "%s\n", status ? "Active" : "Inactive");
	else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static int
call_qcsapi_get_reboot_cause(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_unsigned_int value = 0;
	int qcsapi_retval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_system_get_debug_value(QCSAPI_REBOOT_CAUSE, &value);

	if (qcsapi_retval >= 0) {
		print_out(print, "Reboot Cause - %u\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
aacs_print_vector(call_qcsapi_bundle *p_calling_bundle, int32_t *pbuf, uint32_t len)
{
#define AACS_VEC_BUFLEN 256
#define AACS_VEC_MAXPR 250
	qcsapi_output *print = p_calling_bundle->caller_output;

	int i = 0;
	int j = 0;
	char tmpbuf[AACS_VEC_BUFLEN];

	for (i = 0; i < len && j < AACS_VEC_MAXPR; i++)
		j += sprintf((tmpbuf + j), "%d ", *(pbuf + i));
	print_out(print, "%s\n", tmpbuf);

	return 0;
}

static int
aacs_load_vector(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[],
		int32_t *pbuf, uint32_t *len, uint32_t offset)
{
	int i;

	/* skip command */
	if (*len < (argc - offset))
		return -EINVAL;

	*len = argc - offset;

	for (i = offset; i < argc; i++)
		pbuf[i-offset] = (uint32_t) atoi(argv[i]);

	return 0;
}

static int
call_qcsapi_wifi_aacs_thres_min_tbl_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	if (argc != AACS_THTBL_SIZE) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_thres_min_tbl_set(the_interface, &buf, length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_thres_min_tbl_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	qcsapi_retval = qcsapi_wifi_aacs_thres_min_tbl_get(the_interface,
				&buf, &length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			aacs_print_vector(p_calling_bundle, buf.val, length);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_thres_max_tbl_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	if (argc != AACS_THTBL_SIZE) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_thres_max_tbl_set(the_interface, &buf, length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_thres_max_tbl_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	qcsapi_retval = qcsapi_wifi_aacs_thres_max_tbl_get(the_interface,
				&buf, &length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			aacs_print_vector(p_calling_bundle, buf.val, length);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_rssi_tbl_set(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_VNODETBL_SIZE;

	if (argc != AACS_VNODETBL_SIZE) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_vnode_rssi_tbl_set(
			the_interface, &buf, length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_rssi_tbl_get(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	qcsapi_retval = qcsapi_wifi_aacs_vnode_rssi_tbl_get(
				the_interface, &buf, &length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			aacs_print_vector(p_calling_bundle, buf.val, length);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_wgt_tbl_set(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_VNODETBL_SIZE;

	if (argc != AACS_VNODETBL_SIZE) {
		print_err(print, "Must provide %d weights.\n", AACS_VNODETBL_SIZE);
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_vnode_wgt_tbl_set(
			the_interface, &buf, length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_wgt_tbl_get(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_VNODETBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	qcsapi_retval = qcsapi_wifi_aacs_vnode_wgt_tbl_get(
			the_interface, &buf, &length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			aacs_print_vector(p_calling_bundle, buf.val, length);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_VNODETBL_SIZE;

	if (argc != AACS_VNODEQRY_SIZE) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_vnode_set(the_interface,
				&buf, length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_vnode_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length;
	int i;

	for (i = 0; i < AACS_VNODETBL_SIZE; i++) {
		length = AACS_VNODEQRY_SIZE;
		memset(buf.val, 0, sizeof(buf.val));
		buf.val[0] = i;
		qcsapi_retval = qcsapi_wifi_aacs_vnode_get(the_interface,
					&buf, &length);
		if (qcsapi_retval >= 0) {
			if (verbose_flag >= 0)
				print_out(print, "%d: rssi=%d weight=%d index=%d\n",
						i,
						(int16_t) buf.val[AACS_VNODEQRY_OFF_RSSI],
						(int16_t) buf.val[AACS_VNODEQRY_OFF_WGT],
						(int16_t) buf.val[AACS_VNODEQRY_OFF_IDX]
						);
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
			statval = -EINVAL;
		}
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	if (argc != AACS_THTBL_SIZE) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 0);
	qcsapi_retval = qcsapi_wifi_aacs_dfs_thres_adj_tbl_set(the_interface,
			&buf, length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_THTBL_SIZE;

	memset(buf.val, 0, sizeof(buf.val));
	qcsapi_retval = qcsapi_wifi_aacs_dfs_thres_adj_tbl_get(the_interface,
				&buf, &length);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			aacs_print_vector(p_calling_bundle, buf.val, length);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_excl_ch_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t int1_val = AACS_UINT16_MAX;
	uint16_t int2_val = AACS_UINT16_MAX;

	if (argc < 1) {
		print_err(print, "Incorrect number of arguments\n");
		return -EINVAL;
	}
	if (argc > 1) {
		if ((safe_atou16(argv[0], &int1_val, print, 0, AACS_UINT16_MAX) == 0) ||
				(safe_atou16(argv[1], &int2_val, print, 0,
					AACS_UINT16_MAX) == 0)) {
			print_err(print, "Invalid input\n");
			return -EINVAL;
		}
	}
	qcsapi_retval = qcsapi_wifi_aacs_excl_ch_set(the_interface,
			int1_val, int2_val);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_excl_ch_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int int1_val = 0;
	qcsapi_unsigned_int int2_val = 0;

	qcsapi_retval = qcsapi_wifi_aacs_excl_ch_get(the_interface,
			&int1_val, &int2_val);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d %d\n", int1_val, int2_val);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_alt_excl_ch_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint16_t int1_val = AACS_UINT16_MAX;
	uint16_t int2_val = AACS_UINT16_MAX;

	if (argc < 1) {
		print_err(print, "Incorrect number of arguments\n");
		return -EINVAL;
	}

	if (argc > 1) {
		/* implicit clear */
		if ((safe_atou16(argv[0], &int1_val, print, 0, AACS_UINT16_MAX) == 0) ||
				(safe_atou16(argv[1], &int2_val, print, 0,
					AACS_UINT16_MAX) == 0)) {
			print_err(print, "Invalid input\n");
			return -EINVAL;
		}
	}

	qcsapi_retval = qcsapi_wifi_aacs_alt_excl_ch_set(the_interface,
			int1_val, int2_val);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_alt_excl_ch_get(
		call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int int1_val = 0;
	qcsapi_unsigned_int int2_val = 0;

	qcsapi_retval = qcsapi_wifi_aacs_alt_excl_ch_get(the_interface,
			&int1_val, &int2_val);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "%d %d\n", int1_val, int2_val);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_sel_ch_set(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = -1;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_SELCH_SIZE + 1;
	uint32_t op;
	uint32_t cmd_len;

	if (argc == 0 || argc > length) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	cmd_len = strlen(argv[0]);
	if (!strncmp(argv[0], AACS_CH_OP_ADD_STR, strlen(AACS_CH_OP_ADD_STR)) &&
			cmd_len == strlen(AACS_CH_OP_ADD_STR))
		op = AACS_CH_OP_ADD;
	else if (!strncmp(argv[0], AACS_CH_OP_DEL_STR, strlen(AACS_CH_OP_DEL_STR)) &&
			cmd_len == strlen(AACS_CH_OP_DEL_STR))
		op = AACS_CH_OP_DEL;
	else if (!strncmp(argv[0], AACS_CH_OP_ADD_STR_ALT, strlen(AACS_CH_OP_ADD_STR_ALT)) &&
			cmd_len == strlen(AACS_CH_OP_ADD_STR_ALT))
		op = AACS_CH_OP_ADD_ALT;
	else if (!strncmp(argv[0], AACS_CH_OP_DEL_STR_ALT, strlen(AACS_CH_OP_DEL_STR_ALT)) &&
			cmd_len == strlen(AACS_CH_OP_DEL_STR_ALT))
		op = AACS_CH_OP_DEL_ALT;
	else {
		print_err(print, "Invalid input\n");
		return -EINVAL;
	}

	memset(buf.val, 0, sizeof(buf.val));
	aacs_load_vector(p_calling_bundle, argc, argv, buf.val, &length, 1);
	qcsapi_retval = qcsapi_wifi_aacs_sel_ch_set(the_interface, op, &buf, length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static int
call_qcsapi_wifi_aacs_sel_ch_get(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	int statval = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	struct qcsapi_int_array32 buf;
	uint32_t length = AACS_INCL_CH_LIST_LEN;
	int i;
	int j;
	int num_ch = 0;
	int op;

	if (argc > 1) {
		print_err(print, "Wrong number of arguments\n");
		return -EINVAL;
	}

	if ((argc == 0) || (!strncmp(argv[0], AACS_CH_OP_GET_STR, strlen(AACS_CH_OP_GET_STR)) &&
				strlen(argv[0]) == strlen(AACS_CH_OP_GET_STR)))
		op = AACS_CH_OP_ADD;
	else if (!strncmp(argv[0], AACS_CH_OP_GET_STR_ALT, strlen(AACS_CH_OP_GET_STR_ALT)) &&
				strlen(argv[0]) == strlen(AACS_CH_OP_GET_STR_ALT))
		op = AACS_CH_OP_ADD_ALT;
	else {
		print_err(print, "Invalid input\n");
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_wifi_aacs_sel_ch_get(the_interface, op,
			&buf, &length);

	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0) {
			for (i = 0; i < length; i++)
				for (j = 0; j < AACS_UINT32_BIT_SIZE; j++)
					if (buf.val[i] & (1 << j)) {
						print_out(print, "%d ",
								i * AACS_UINT32_BIT_SIZE + j);
						num_ch++;
					}
			if (!num_ch)
				print_out(print, "No channel selected");
			print_out(print, "\n");
		}
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = -EINVAL;
	}

	return statval;
}

static const char *protocol_name_table[] = {
	"None",
	"WPA",
	"WPA2",
	"WPA WPA2",
	"WPA3"
};

static const char *encryption_name_table[] = {
	"None",
	"TKIP",
	"CCMP",
	"TKIP CCMP"
};

static const char *authentication_name_table[] = {
	"None",
	"PSK",
	"EAP",
	"SAE",
	"OWE",
	"DPP"
};

static const char *get_name_by_value(const int qcsapi_value, const char **qcsapi_lookup_table,
		const size_t lookup_table_size)
{
	const char *retaddr = NULL;

	if (qcsapi_value >= 0 && qcsapi_value < (int)lookup_table_size)
		retaddr = qcsapi_lookup_table[qcsapi_value];

	return retaddr;
}

static void local_snprint_bitrate(char *buffer, int len, unsigned int bitrate)
{
	int i = 0;
	char ch[] = { ' ', 'k', 'M', 'G' };
	int remainder = 0;
	int val;

	for (i = 0; i < 3 && bitrate >= 1000; i++) {
		val = bitrate / 1000;
		remainder = (bitrate % 1000);
		bitrate = val;
	}
	if (remainder) {
		snprintf(buffer, len, "%d.%1.1d %cb/s", bitrate, remainder, ch[i]);
	} else {
		snprintf(buffer, len, "%d %cb/s", bitrate, ch[i]);
	}
}

static void
local_show_ap_properties(qcsapi_output *print, const qcsapi_unsigned_int index_ap,
		const qcsapi_ap_properties *p_ap_properties)
{
	char mac_addr_string[24];
	char buffer[32];

	print_out(print, "AP %d:\n", index_ap);
	print_out(print, "\tSSID: %s\n", p_ap_properties->ap_name_SSID);
	sprintf(&mac_addr_string[0], MACFILTERINGMACFMT,
			p_ap_properties->ap_mac_addr[0],
			p_ap_properties->ap_mac_addr[1],
			p_ap_properties->ap_mac_addr[2],
			p_ap_properties->ap_mac_addr[3],
			p_ap_properties->ap_mac_addr[4], p_ap_properties->ap_mac_addr[5]
			);
	print_out(print, "\tMAC address: %s\n", &mac_addr_string[0]);
	print_out(print, "\tChannel: %d\n", p_ap_properties->ap_channel);
	print_out(print, "\tBandwidth: %d\n", p_ap_properties->ap_bw);
	print_out(print, "\tRSSI: %d\n", p_ap_properties->ap_RSSI);
	print_out(print, "\tHT secondary offset: %s\n",
			p_ap_properties->ap_ht_secoffset ==
			IEEE80211_HTINFO_EXTOFFSET_ABOVE ? "Above" : p_ap_properties->
			ap_ht_secoffset == IEEE80211_HTINFO_EXTOFFSET_BELOW ? "Below" : "None");
	print_out(print, "\tcenter channel 1: %d\n", p_ap_properties->ap_chan_center1);
	print_out(print, "\tcenter channel 2: %d\n", p_ap_properties->ap_chan_center2);
	print_out(print, "\tLast seen: %u\n", p_ap_properties->ap_last_beacon);
	local_snprint_bitrate(buffer, sizeof(buffer), p_ap_properties->ap_best_data_rate);
	print_out(print, "\tBest Data Rate: %s\n", buffer);

	print_out(print, "\tSGI capability:", buffer);
	if (p_ap_properties->ap_flags &
			((1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ) | (1 <<
							QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ) | (1
							<< QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ) |
					(1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ))) {
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_20MHZ))
			print_out(print, " 20MHz", buffer);
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_40MHZ))
			print_out(print, " 40MHz", buffer);
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_80MHZ))
			print_out(print, " 80MHz", buffer);
		if (p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SGI_CAPS_IN_160MHZ))
			print_out(print, " 160MHz", buffer);
		print_out(print, "\n", buffer);
	} else
		print_out(print, " None\n", buffer);

	if ((p_ap_properties->ap_flags & (1 << QCSAPI_AP_FLAG_BIT_SEC_ENABLE)) != 0) {
		const char *value_name = NULL;

		print_out(print, "\tsecurity enabled\n");

		value_name = get_name_by_value(p_ap_properties->ap_protocol,
				protocol_name_table, ARRAY_SIZE(protocol_name_table));
		if (value_name == NULL)
			value_name = "(unknown)";
		print_out(print, "\tprotocol: %s\n", value_name);

		if (verbose_flag > 0) {
			value_name = get_name_by_value(p_ap_properties->ap_authentication_mode,
					authentication_name_table,
					ARRAY_SIZE(authentication_name_table)
					);
			if (value_name == NULL)
				value_name = "(unknown)";
			print_out(print, "\tauthentication mode: %s\n", value_name);

			value_name = get_name_by_value(p_ap_properties->ap_encryption_modes,
					encryption_name_table, ARRAY_SIZE(encryption_name_table)
					);
			if (value_name == NULL)
				value_name = "(unknown)";
			print_out(print, "\tencryption modes: %s\n", value_name);
		}
	} else {
		print_out(print, "\tsecurity disabled\n");
	}

	print_out(print, "\n");
}

static int
call_qcsapi_wifi_show_access_points(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	uint32_t count_APs;
	uint32_t offchan = 0;

	if (argc >= 1 && 0 == safe_atou32(argv[0], &offchan, print, 0, 1))
		return 1;

	if (offchan)
		statval = qcsapi_wifi_get_results_AP_scan_by_scs(the_interface, &count_APs);
	else
		statval = qcsapi_wifi_get_results_AP_scan(the_interface, &count_APs);
	if (statval >= 0) {
		qcsapi_unsigned_int iter;
		qcsapi_ap_properties ap_properties;

		for (iter = 0; iter < count_APs && statval >= 0; iter++) {
			statval = qcsapi_wifi_get_properties_AP(the_interface, iter,
					&ap_properties);
			if (statval >= 0)
				local_show_ap_properties(print, iter + 1, &ap_properties);
		}
	} else {
		report_qcsapi_error(p_calling_bundle, statval);
		return 1;
	}

	return statval;
}

static int call_qcsapi_wifi_get_pta(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	uint32_t value = 0;
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	qcsapi_retval = qcsapi_wifi_get_pta(the_interface, &value);

	if (qcsapi_retval >= 0) {
		print_out(print, "%d\n", value);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_pta_op_mode(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = 0;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "Usage: call_qcsapi set_pta <WiFi interface> <mode code>\n";

	if (argc != 1) {
		print_out(print, usage);
		return 1;
	}

	qcsapi_retval = qcsapi_wifi_set_pta_op_mode(the_interface, strtoul(argv[0], NULL, 16));

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);
}

static int
call_qcsapi_reg_chan_txpower_backoff_set(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *the_interface = p_calling_bundle->caller_interface;
	int qcsapi_retval;
	uint8_t channel;
	uint8_t is_percentage;
	uint8_t backoff;

	if (argc < 3) {
		print_err(print, "Not enough parameters in call_qcsapi set_chan_txpower_backoff\n");
		print_err(print, "Usage: call_qcsapi set_chan_txpower_backoff <interface> <channel>"
				" <is_percentage> <backoff>\n");
		return 1;
	}

	channel = atoi(argv[0]);
	is_percentage = ! !atoi(argv[1]);
	backoff = atoi(argv[2]);

	if (channel > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %s\n", channel);
		return 1;
	}
	if (is_percentage && (backoff >= 100)) {
		print_err(print, "Invalid backoff %d\n", backoff);
		return 1;
	}

	qcsapi_retval = qcsapi_reg_chan_txpower_backoff_set(the_interface, channel, is_percentage,
			backoff);

	return qcsapi_report_complete(p_calling_bundle, qcsapi_retval);

}

static int
call_qcsapi_reg_chan_txpower_backoff_get(call_qcsapi_bundle *p_calling_bundle, int argc,
		char *argv[])
{
	int statval = 0;
	int qcsapi_retval;
	uint8_t chan;
	uint8_t is_percentage;
	uint8_t backoff;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1) {
		print_err(print, "Not enough parameters in call_qcsapi get_chan_txpower_backoff\n");
		print_err(print, "Usage: call_qcsapi get_chan_txpower_backoff <interface> <channel>\n");
		return 1;
	}

	chan = atoi(argv[0]);
	if (chan > QCSAPI_MAX_CHANNEL) {
		print_err(print, "bad channel parameter %s\n", chan);
		return 1;
	}
	qcsapi_retval = qcsapi_reg_chan_txpower_backoff_get(the_interface, chan, &is_percentage,
			&backoff);

	if (qcsapi_retval >= 0) {
		if (backoff > 0)
			print_out(print, "%s: %d\n", is_percentage ? "percentage" : "absolute",
					backoff);
		else
			print_out(print, "backoff off\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

static void call_qcsapi_display_grab_config_usage(qcsapi_output *print)
{
	print_out(print, "usage\n");
	print_out(print, "	call_qcsapi grab_config <output file>\n");
}

static int call_qcsapi_grab_config(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int ret = 0;
	const char *output_path;
	FILE *output_stream = NULL;
	size_t bytes_written = 0;

	if ((argc < 1)) {
		call_qcsapi_display_grab_config_usage(print);
		return 1;
	}

	output_path = argv[0];
	output_stream = fopen(output_path, "w");
	if (output_stream == NULL) {
		print_err(print, "Failed to open %s: %s\n", output_path, strerror(errno));
		ret = -EFAULT;
		goto out;
	}

	print_err(print, "Grabbing config...\n");
	ret = qcsapi_grabber_write_config_blob(output_stream, QCSAPI_GRABBER_PARAM_ALL,
			&bytes_written);
	if (ret) {
		print_err(print, "Can not write blob to %s\n", output_path);
		goto out;
	}

out:
	if (output_stream)
		fclose(output_stream);

	return qcsapi_report_complete(p_calling_bundle, ret);
}

static int
call_qcsapi_repeater_mode_cfg(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = -1;
	qcsapi_output *print = p_calling_bundle->caller_output;
	qcsapi_unsigned_int radio_id = 0;
	unsigned int for_repeater;

	if (argc != 2)
		goto out;

	if (radio_id_param_parse_input(print, argv[0], &radio_id))
		goto out;

	if (strcmp(argv[1], "0") == 0)
		for_repeater = 0;
	else if (strcmp(argv[1], "1") == 0)
		for_repeater = 1;
	else
		goto out;

	qcsapi_retval = qcsapi_wifi_repeater_mode_cfg(radio_id, for_repeater);

	if (verbose_flag >= 0) {
		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		}
	}

	return qcsapi_retval;

out:
	print_out(print, "Usage: call_qcsapi repeater_mode_cfg <radio_id> {0 | 1}\n");
	return -1;
}

struct urepeater_params {
	const char *str;
	qcsapi_urepeater_type type;
	uint16_t is_get;
	uint16_t is_set;
};

static const struct urepeater_params urep_params[] = {
	{"max_level", qcsapi_urepeater_max_level, 1, 1},
	{"curr_level", qcsapi_urepeater_curr_level, 1, 0}
};

static int
call_qcsapi_set_urepeater_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = -1;
	qcsapi_urepeater_type type = qcsapi_urepeater_none;
	uint32_t value;
	int i;
	int ret;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 2) {
		qcsapi_report_usage(p_calling_bundle, "<parameter type> <value>\n");
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(urep_params); i++) {
		if (strcmp(argv[0], urep_params[i].str) == 0) {
			if (!urep_params[i].is_set)
				break;

			type = urep_params[i].type;
			break;
		}
	}

	if (type == qcsapi_urepeater_none) {
		print_out(print, "Invalid parameter type \"%s\"\n", argv[0]);
		return -EINVAL;
	}

	ret = safe_atou32(argv[1], &value, print, REPEATER_MIN_LEVEL, REPEATER_MAX_LEVEL);
	if (ret == 0) {
		print_out(print, "Invalid parameter value \"%s\"\n", argv[1]);
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_wifi_set_urepeater_params(type, (int)value);

	if (verbose_flag >= 0) {
		if (qcsapi_retval >= 0) {
			print_out(print, "complete\n");
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		}
	}

	return qcsapi_retval;
}

static int
call_qcsapi_get_urepeater_params(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int qcsapi_retval = -1;
	qcsapi_urepeater_type type = qcsapi_urepeater_none;
	int value;
	int i;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, "<parameter type>\n");
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(urep_params); i++) {
		if (strcmp(argv[0], urep_params[i].str) == 0) {
			if (!urep_params[i].is_get)
				break;

			type = urep_params[i].type;
			break;
		}
	}

	if (type == qcsapi_urepeater_none) {
		print_out(print, "Invalid parameter type \"%s\"\n", argv[0]);
		return -EINVAL;
	}

	qcsapi_retval = qcsapi_wifi_get_urepeater_params(type, &value);

	if (verbose_flag >= 0) {
		if (qcsapi_retval >= 0) {
			print_out(print, "%d\n", value);
		} else {
			report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		}
	}

	return qcsapi_retval;
}

static int
call_qcsapi_wifi_get_legacy_bbic(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_unsigned_int current_val = 0;
	qcsapi_unsigned_int *val = NULL;
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;
	qcsapi_output *print = p_calling_bundle->caller_output;

	if (argc < 1)
		val = &current_val;
	qcsapi_retval = qcsapi_wifi_get_legacy_bbic(the_interface, val);
	if (qcsapi_retval >= 0) {
		print_out(print, "%u\n", current_val);
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		return 1;
	}

	return 0;
}

static int
call_qcsapi_wifi_set_legacy_bbic(const call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;
	const char *usage = "<WiFi interface> <0 | 1 | 2 | 3>\n";
	int qcsapi_retval;
	const char *the_interface = p_calling_bundle->caller_interface;

	if (argc != 1) {
		qcsapi_report_usage(p_calling_bundle, usage);
		return 1;
	}

	qcsapi_unsigned_int val = (qcsapi_unsigned_int) atoi(argv[0]);

	qcsapi_retval = qcsapi_wifi_set_legacy_bbic(the_interface, val);
	if (qcsapi_retval >= 0) {
		if (verbose_flag >= 0)
			print_out(print, "complete\n");
	} else {
		report_qcsapi_error(p_calling_bundle, qcsapi_retval);
		statval = 1;
	}

	return statval;
}

#define QCSAPI_CONFIG_NAME_LEN	128
#define QCSAPI_CONFIG_FILE_PATH "/lib/firmware/qtn/"

static void local_macaddr_remove_colons(char *mac)
{
	int i;
	int j = 0;

	for (i = 0; mac[i] != '\0'; i++) {
		if (mac[i] != ':') {
			if (i != j)
				mac[j] = mac[i];
			j++;
		}
	}
	mac[j] = '\0';
}

static int local_get_qtn_ep_cfg_file_name(FILE *f, char *output_dir,
		char *output_name, unsigned int name_len)
{
	char last_line[64];
	char *macaddr = NULL;
	char *version = NULL;
	uint32_t parsed_ver = 0;
	uint64_t pos;
	int ret;
	uint64_t end;

	if (!f)
		return -EINVAL;

	if (fseek(f, 0, SEEK_END))
		return -EINVAL;

	/*
	 * The config file is a tar file followed by a newline character and
	 * metadata, which contains (for version 1) the following fields, separated by spaces.
	 * - config file version
	 * - EP MAC address
	 * - Tar file checksum
	 * Refer to the save_config script for the full format.
	 */
	end = ftell(f);
	pos = end;
	while (pos) {
		if (!fseek(f, --pos, SEEK_SET)) {
			if (fgetc(f) == '\n') {
				if (pos < (end - 1))
					break;
			}
		} else {
			return -EINVAL;
		}
	}

	/* first 2 values are in "version macaddr" format */
	if (fgets(last_line, sizeof(last_line), f) == NULL)
		return -EFAULT;
	last_line[sizeof(last_line) - 1] = '\0';

	version = strtok(last_line, " ");
	macaddr = strtok(NULL, " ");
	if (!strlen(version) || !strlen(macaddr))
		return -EFAULT;

	local_macaddr_remove_colons(macaddr);

	parsed_ver = strtol(version, NULL, 10);
	if (!parsed_ver)
		return -EFAULT;

	ret = snprintf(output_name, name_len,
			"%sqtn-%s-v%u.cfg",
			output_dir[0] ? output_dir : QCSAPI_CONFIG_FILE_PATH,
			macaddr, parsed_ver);

	if (ret >= name_len)
		return -qcsapi_buffer_overflow;

	return ret;
}

static int call_qcsapi_save_config(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int ret = 0;
	const int buflen = QCSAPI_CONFIG_NAME_LEN;
	char output_name[buflen];
	char output_dir[buflen];
	char output_name_tmp[buflen];
	FILE *output_fp = NULL;
	unsigned int bytes_remain = 0;
	unsigned int bytes_copied = 0;
	unsigned int offset = 0;
	struct qcsapi_data_3Kbytes *buf = NULL;
	int suc = 0;
	int output_dir_len = 0;
	int copy_len = 0;

	if (access(QCSAPI_SYSTEM_STATUS_FILE, F_OK) == 0) {
		print_err(print, "%s must be run from the host system\n",
			entry_point_enum_to_name(p_calling_bundle->caller_qcsapi));
		return -EINVAL;
	}

	if (argc > 1) {
		qcsapi_report_usage(p_calling_bundle, "[<output file>]\n");
		return 1;
	}

	if (getuid() != 0) {
		print_err(print, "only root can do that\n");
		return -EINVAL;
	}

	memset(output_name, 0, buflen);
	memset(output_dir, 0, buflen);
	memset(output_name_tmp, 0, buflen);
	if (argv[0]) {
		output_dir_len = snprintf(output_dir, buflen, "%s", argv[0]);
		if (output_dir_len >= (buflen - 2)) {
			print_err(print, "Output directory name is too long\n");
			return -EINVAL;
		}
		if (output_dir[output_dir_len - 1] != '/') {
			output_dir[output_dir_len] = '/';
			output_dir_len++;
		}
	}

	if (output_dir_len) {
		copy_len = snprintf(output_name_tmp, buflen, "%sqtn_ep_cfg.tmp", output_dir);
		if (copy_len >= buflen) {
			print_err(print, "Output filename is too long\n");
			return -EINVAL;
		}
	} else {
		copy_len = snprintf(output_name_tmp, buflen,
				QCSAPI_CONFIG_FILE_PATH "qtn_ep_cfg.tmp");
		if (copy_len >= buflen) {
			print_err(print, "Output filename is too long\n");
			return -EINVAL;
		}
		ret = mkdir(QCSAPI_CONFIG_FILE_PATH, 0755);
		if ((ret != 0) && (errno != EEXIST)) {
			print_err(print, "Make %s directory failed\n",
					QCSAPI_CONFIG_FILE_PATH);
			return -EINVAL;
		}
	}

	output_fp = fopen(output_name_tmp, "wb+");
	if (output_fp == NULL) {
		ret = -EFAULT;
		goto out;
	}

	buf = malloc(sizeof(*buf));
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = -EFAULT;
	do {
		memset(buf, 0, sizeof(*buf));
		bytes_copied = 0;
		bytes_remain = 0;
		ret = qcsapi_save_config(buf, offset, &bytes_copied, &bytes_remain);
		if (ret < 0)
			goto out;

		offset += bytes_copied;

		if (fwrite(buf, 1, bytes_copied, output_fp) != bytes_copied)
			break;

		if (bytes_remain == 0) {
			suc = 1;
			ret = 0;
		}

	} while (bytes_remain > 0);

out:
	if (buf)
		free(buf);

	if (suc) {
		ret = local_get_qtn_ep_cfg_file_name(output_fp, output_dir,
				output_name, sizeof(output_name));
		fclose(output_fp);
		if ((ret <= 0) || (output_name[0] == '\0')) {
			print_err(print, "Cannot get saved file name\n");
			return -EINVAL;
		}

		ret = rename(output_name_tmp, output_name);
	} else {
		if (output_fp)
			fclose(output_fp);
		unlink(output_name_tmp);
	}

	if (ret == 0)
		print_out(print, "Config saved to %s\n", output_name);

	return qcsapi_report_complete(p_calling_bundle, ret);
}

static int call_qcsapi_get_config_status(call_qcsapi_bundle *p_calling_bundle,
		int argc, char *argv[])
{
	qcsapi_output *print = p_calling_bundle->caller_output;
	int retval = 0;
	char result[QCSAPI_MSG_BUFSIZE] = {0};

	if (argc >= 1) {
		qcsapi_report_usage(p_calling_bundle, "\n");
		return 1;
	}

	retval = qcsapi_get_config_status(result, sizeof(result));
	if (retval == 0)
		print_out(print, "%s", result);
	else
		report_qcsapi_error(p_calling_bundle, retval);

	return retval;
}

/* end of programs to call individual QCSAPIs */

static int call_particular_qcsapi(call_qcsapi_bundle *p_calling_bundle, int argc, char *argv[])
{
	int statval = 0;
	qcsapi_output *print = p_calling_bundle->caller_output;

	/*
	 *Interface programs that SET a parameter require the
	 *current list of arguments to get additional parameters
	 */
	switch (p_calling_bundle->caller_qcsapi) {
	case e_qcsapi_errno_get_message:
		statval = call_qcsapi_errno_get_message(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_store_ipaddr:
		statval = call_qcsapi_store_ipaddr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_enable:
		statval = call_qcsapi_interface_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_BSSID:
		statval = call_qcsapi_interface_get_BSSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_mac_addr:
		statval = call_qcsapi_interface_get_mac_addr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_set_mac_addr:
		statval = call_qcsapi_interface_set_mac_addr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_counter:
		statval = call_qcsapi_interface_get_counter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_counter64:
		statval = call_qcsapi_interface_get_counter64(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_flash_image_update:
		statval = call_qcsapi_flash_image_update(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_firmware_get_version:
		statval = call_qcsapi_firmware_get_version(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_system_get_time_since_start:
		statval = call_qcsapi_system_get_time_since_start(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_system_status:
		statval = call_qcsapi_get_system_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_cpu_usage:
		statval = call_qcsapi_get_cpu_usage(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_memory_usage:
		statval = call_qcsapi_get_memory_usage(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_random_seed:
		statval = call_qcsapi_get_random_seed(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_set_random_seed:
		statval = call_qcsapi_set_random_seed(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_led_get:
		statval = call_qcsapi_led_get(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_led_set:
		statval = call_qcsapi_led_set(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_led_pwm_enable:
		statval = call_qcsapi_led_pwm_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_led_brightness:
		statval = call_qcsapi_led_brightness(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_gpio_get_config:
		statval = call_qcsapi_gpio_get_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_gpio_set_config:
		statval = call_qcsapi_gpio_set_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_gpio_enable_wps_push_button:
		statval = call_qcsapi_gpio_enable_wps_push_button(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_file_path_get_config:
		statval = call_qcsapi_file_path_get_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_file_path_set_config:
		statval = call_qcsapi_file_path_set_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_wifi_macaddr:
		statval = call_qcsapi_wifi_set_wifi_macaddr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_create_restricted_bss:
		statval = call_qcsapi_wifi_create_restricted_bss(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_create_bss:
		statval = call_qcsapi_wifi_create_bss(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_remove_bss:
		statval = call_qcsapi_wifi_remove_bss(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_primary_interface:
		statval = call_qcsapi_wifi_get_primary_interface(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_interface_by_index:
		statval = call_qcsapi_wifi_get_interface_by_index(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_interface_by_index_all:
		statval = call_qcsapi_wifi_get_interface_by_index_all(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_mode:
		statval = call_qcsapi_wifi_get_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_mode:
		statval = call_qcsapi_wifi_set_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_phy_mode:
		statval = call_qcsapi_wifi_get_phy_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_phy_mode:
		statval = call_qcsapi_wifi_set_phy_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_phy_mode_required:
		statval = call_qcsapi_wifi_get_phy_mode_required(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_phy_mode_required:
		statval = call_qcsapi_wifi_set_phy_mode_required(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_reload_in_mode:
		statval = call_qcsapi_wifi_reload_in_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_rfenable:
		statval = call_qcsapi_wifi_rfenable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_rfstatus:
		statval = call_qcsapi_wifi_rfstatus(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_startprod:
		statval = call_qcsapi_wifi_startprod(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bw:
		statval = call_qcsapi_wifi_get_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_bw:
		statval = call_qcsapi_wifi_set_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_24g_bw:
		statval = call_qcsapi_wifi_get_24g_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_24g_bw:
		statval = call_qcsapi_wifi_set_24g_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_freq_bands:
		statval = call_qcsapi_wifi_get_supported_freq_bands(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_noise:
		statval = call_qcsapi_wifi_get_noise(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rssi_by_chain:
		statval = call_qcsapi_wifi_get_rssi_by_chain(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_avg_snr:
		statval = call_qcsapi_wifi_get_avg_snr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_BSSID:
		statval = call_qcsapi_wifi_get_BSSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_config_BSSID:
		statval = call_qcsapi_wifi_get_config_BSSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_ssid_get_bssid:
		statval = call_qcsapi_wifi_ssid_get_bssid(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_ssid_set_bssid:
		statval = call_qcsapi_wifi_ssid_set_bssid(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_SSID:
		statval = call_qcsapi_wifi_get_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_SSID2:
		statval = call_qcsapi_wifi_get_SSID2(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_SSID:
		statval = call_qcsapi_wifi_set_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_channel:
		statval = call_qcsapi_wifi_get_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_chan:
		statval = call_qcsapi_wifi_get_chan(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_channel:
		statval = call_qcsapi_wifi_set_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_chan:
		statval = call_qcsapi_wifi_set_chan(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_channel_and_bw:
		statval = call_qcsapi_wifi_get_channel_and_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_channel_and_bw:
		statval = call_qcsapi_wifi_set_channel_and_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_wea_cac_en:
		statval = call_qcsapi_wifi_set_wea_cac_en(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_auto_channel:
		statval = call_qcsapi_wifi_get_auto_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_auto_channel:
		statval = call_qcsapi_wifi_set_auto_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_standard:
		statval = call_qcsapi_wifi_get_standard(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dtim:
		statval = call_qcsapi_wifi_get_dtim(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dtim:
		statval = call_qcsapi_wifi_set_dtim(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_assoc_limit:
		statval = call_qcsapi_wifi_get_assoc_limit(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_assoc_limit:
		statval = call_qcsapi_wifi_set_assoc_limit(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bss_assoc_limit:
		statval = call_qcsapi_wifi_get_bss_assoc_limit(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_bss_assoc_limit:
		statval = call_qcsapi_wifi_set_bss_assoc_limit(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_SSID_group_id:
		statval = call_qcsapi_wifi_set_SSID_group_id(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_SSID_group_id:
		statval = call_qcsapi_wifi_get_SSID_group_id(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_SSID_assoc_reserve:
		statval = call_qcsapi_wifi_set_SSID_assoc_reserve(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_SSID_assoc_reserve:
		statval = call_qcsapi_wifi_get_SSID_assoc_reserve(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_status:
		statval = call_qcsapi_interface_get_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_pm_get_counter:
		statval = call_qcsapi_pm_get_counter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_pm_get_elapsed_time:
		statval = call_qcsapi_pm_get_elapsed_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_set_ip4:
		statval = call_qcsapi_interface_set_ip4(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_ip4:
		statval = call_qcsapi_interface_get_ip4(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_set_mtu:
		statval = call_qcsapi_interface_set_mtu(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_interface_get_mtu:
		statval = call_qcsapi_interface_get_mtu(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_list_channels:
		statval = call_qcsapi_wifi_get_list_channels(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_chan_list:
		statval = call_qcsapi_wifi_get_chan_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_supp_chans:
		statval = call_qcsapi_wifi_get_supp_chans(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_mode_switch:
		statval = call_qcsapi_wifi_get_mode_switch(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_option:
		statval = call_qcsapi_wifi_get_option(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_board_parameter:
		statval = call_qcsapi_get_board_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_option:
		statval = call_qcsapi_wifi_set_option(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rates:
		statval = call_qcsapi_wifi_get_rates(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_rates:
		statval = call_qcsapi_wifi_set_rates(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_max_bitrate:
		statval = call_qcsapi_wifi_get_max_bitrate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_max_bitrate:
		statval = call_qcsapi_wifi_set_max_bitrate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_beacon_type:
		statval = call_qcsapi_wifi_get_beacon_type(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_beacon_type:
		statval = call_qcsapi_wifi_set_beacon_type(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_beacon_interval:
		statval = call_qcsapi_wifi_get_beacon_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_beacon_interval:
		statval = call_qcsapi_wifi_set_beacon_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_list_regulatory_regions:
		statval = call_qcsapi_wifi_get_list_regulatory_regions(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_regulatory_tx_power:
		statval = call_qcsapi_wifi_get_regulatory_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_configured_tx_power:
		statval = call_qcsapi_wifi_get_configured_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_regulatory_channel:
		statval = call_qcsapi_wifi_set_regulatory_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_regulatory_region:
		statval = call_qcsapi_wifi_set_regulatory_region(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_restore_regulatory_tx_power:
		statval = call_qcsapi_wifi_restore_regulatory_tx_power(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_regulatory_region:
		statval = call_qcsapi_wifi_get_regulatory_region(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_overwrite_country_code:
		statval = call_qcsapi_wifi_overwrite_country_code(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_list_regulatory_channels:
		statval = call_qcsapi_wifi_get_list_regulatory_channels(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_list_regulatory_bands:
		statval = call_qcsapi_wifi_get_list_regulatory_bands(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_regulatory_db_version:
		statval = call_qcsapi_wifi_get_regulatory_db_version(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_regulatory_tx_power_cap:
		statval = call_qcsapi_wifi_set_regulatory_tx_power_cap(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_chan_pri_inactive:
		statval = call_qcsapi_wifi_set_chan_pri_inactive(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_chan_pri_inactive:
		statval = call_qcsapi_wifi_get_chan_pri_inactive(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dfs_s_radio_chan_off:
		statval = call_qcsapi_wifi_set_dfs_s_radio_chan_off(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dfs_s_radio_chan_off:
		statval = call_qcsapi_wifi_get_dfs_s_radio_chan_off(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_chan_disabled:
		statval = call_qcsapi_wifi_set_chan_disabled(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_chan_disabled:
		statval = call_qcsapi_wifi_get_chan_disabled(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_power:
		statval = call_qcsapi_wifi_get_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_tx_power:
		statval = call_qcsapi_wifi_set_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_power_ext:
		statval = call_qcsapi_wifi_get_tx_power_ext(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_tx_power_ext:
		statval = call_qcsapi_wifi_set_tx_power_ext(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_reg_chan_txpower_set:
		statval = call_qcsapi_reg_chan_txpower_set(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_reg_chan_txpower_get:
		statval = call_qcsapi_reg_chan_txpower_get(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_chan_power_table:
		statval = call_qcsapi_wifi_set_chan_power_table(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bw_power:
		statval = call_qcsapi_wifi_get_bw_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_bw_power:
		statval = call_qcsapi_wifi_set_bw_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bf_power:
		statval = call_qcsapi_wifi_get_bf_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_bf_power:
		statval = call_qcsapi_wifi_set_bf_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_power_selection:
		statval = call_qcsapi_wifi_get_power_selection(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_power_selection:
		statval = call_qcsapi_wifi_set_power_selection(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_carrier_interference:
		statval = call_qcsapi_wifi_get_carrier_interference(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_congestion_idx:
		statval = call_qcsapi_wifi_get_congestion_idx(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_supported_tx_power_levels:
		statval = call_qcsapi_wifi_get_supported_tx_power_levels(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_current_tx_power_level:
		statval = call_qcsapi_wifi_get_current_tx_power_level(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_power_constraint:
		statval = call_qcsapi_wifi_set_power_constraint(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_power_constraint:
		statval = call_qcsapi_wifi_get_power_constraint(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_tpc_interval:
		statval = call_qcsapi_wifi_set_tpc_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tpc_interval:
		statval = call_qcsapi_wifi_get_tpc_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_assoc_records:
		statval = call_qcsapi_wifi_get_assoc_records(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_list_DFS_channels:
		statval = call_qcsapi_wifi_get_list_DFS_channels(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_is_channel_DFS:
		statval = call_qcsapi_wifi_is_channel_DFS(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_DFS_alt_channel:
		statval = call_qcsapi_wifi_get_DFS_alt_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_DFS_alt_channel:
		statval = call_qcsapi_wifi_set_DFS_alt_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_DFS_reentry:
		statval = call_qcsapi_wifi_set_dfs_reentry(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_radar_chain:
		statval = call_qcsapi_wifi_set_radar_chain(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_cce_channels:
		statval = call_qcsapi_wifi_get_scs_cce_channels(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dfs_cce_channels:
		statval = call_qcsapi_wifi_get_dfs_cce_channels(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_csw_records:
		statval = call_qcsapi_wifi_get_csw_records(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_radar_status:
		statval = call_qcsapi_wifi_get_radar_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_WEP_encryption_level:
		statval = call_qcsapi_wifi_get_WEP_encryption_level(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_WEP_key:
		statval = call_qcsapi_wifi_get_WEP_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_WEP_key:
		statval = call_qcsapi_wifi_set_WEP_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_WEP_key_index:
		statval = call_qcsapi_wifi_get_WEP_key_index(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_WEP_key_index:
		statval = call_qcsapi_wifi_set_WEP_key_index(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_remove_WEP_config:
		statval = call_qcsapi_wifi_remove_WEP_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_WPA_encryption_modes:
		statval = call_qcsapi_wifi_get_WPA_encryption_modes(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_WPA_encryption_modes:
		statval = call_qcsapi_wifi_set_WPA_encryption_modes(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_WPA_authentication_mode:
		statval = call_qcsapi_wifi_get_WPA_authentication_mode(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_WPA_authentication_mode:
		statval = call_qcsapi_wifi_set_WPA_authentication_mode(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_params:
		statval = call_qcsapi_wifi_get_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_params:
		statval = call_qcsapi_wifi_set_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_interworking:
		statval = call_qcsapi_wifi_get_interworking(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_interworking:
		statval = call_qcsapi_wifi_set_interworking(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_80211u_params:
		statval = call_qcsapi_wifi_get_80211u_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_80211u_params:
		statval = call_qcsapi_wifi_set_80211u_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_set_sec_agent:
		statval = call_qcsapi_security_set_sec_agent(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_sec_agent_status:
		statval = call_qcsapi_security_get_sec_agent_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_nai_realms:
		statval = call_qcsapi_security_get_nai_realms(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_nai_realm:
		statval = call_qcsapi_security_add_nai_realm(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_nai_realm:
		statval = call_qcsapi_security_del_nai_realm(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_roaming_consortium:
		statval = call_qcsapi_security_add_roaming_consortium(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_roaming_consortium:
		statval = call_qcsapi_security_del_roaming_consortium(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_roaming_consortium:
		statval = call_qcsapi_security_get_roaming_consortium(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_venue_name:
		statval = call_qcsapi_security_get_venue_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_venue_name:
		statval = call_qcsapi_security_add_venue_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_venue_name:
		statval = call_qcsapi_security_del_venue_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_oper_friendly_name:
		statval = call_qcsapi_security_get_oper_friendly_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_oper_friendly_name:
		statval = call_qcsapi_security_add_oper_friendly_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_oper_friendly_name:
		statval = call_qcsapi_security_del_oper_friendly_name(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_hs20_conn_capab:
		statval = call_qcsapi_security_get_hs20_conn_capab(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_hs20_conn_capab:
		statval = call_qcsapi_security_add_hs20_conn_capab(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_hs20_conn_capab:
		statval = call_qcsapi_security_del_hs20_conn_capab(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_hs20_icon:
		statval = call_qcsapi_security_add_hs20_icon(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_hs20_icon:
		statval = call_qcsapi_security_get_hs20_icon(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_hs20_icon:
		statval = call_qcsapi_security_del_hs20_icon(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_osu_server_uri:
		statval = call_qcsapi_security_add_osu_server_uri(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_osu_server_uri:
		statval = call_qcsapi_security_get_osu_server_uri(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_osu_server_uri:
		statval = call_qcsapi_security_del_osu_server_uri(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_add_osu_server_param:
		statval = call_qcsapi_security_add_osu_server_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_get_osu_server_param:
		statval = call_qcsapi_security_get_osu_server_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_security_del_osu_server_param:
		statval = call_qcsapi_security_del_osu_server_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_hs20_status:
		statval = call_qcsapi_wifi_get_hs20_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_hs20_status:
		statval = call_qcsapi_wifi_set_hs20_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_hs20_params:
		statval = call_qcsapi_wifi_get_hs20_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_hs20_params:
		statval = call_qcsapi_wifi_set_hs20_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_remove_11u_param:
		statval = call_qcsapi_remove_11u_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_remove_hs20_param:
		statval = call_qcsapi_remove_hs20_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_proxy_arp:
		statval = call_qcsapi_wifi_set_proxy_arp(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_proxy_arp:
		statval = call_qcsapi_wifi_get_proxy_arp(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_l2_ext_filter:
		statval = call_qcsapi_wifi_get_l2_ext_filter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_l2_ext_filter:
		statval = call_qcsapi_wifi_set_l2_ext_filter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_IEEE11i_encryption_modes:
		statval = call_qcsapi_wifi_get_IEEE11i_encryption_modes(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_IEEE11i_encryption_modes:
		statval = call_qcsapi_wifi_set_IEEE11i_encryption_modes(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_IEEE11i_authentication_mode:
		statval = call_qcsapi_wifi_get_IEEE11i_authentication_mode(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_IEEE11i_authentication_mode:
		statval = call_qcsapi_wifi_set_IEEE11i_authentication_mode(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_michael_errcnt:
		statval = call_qcsapi_wifi_get_michael_errcnt(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_pre_shared_key:
		statval = call_qcsapi_wifi_get_pre_shared_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_pre_shared_key:
		statval = call_qcsapi_wifi_set_pre_shared_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_psk_auth_failures:
		statval = call_qcsapi_wifi_get_psk_auth_failures(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_key_passphrase:
		statval = call_qcsapi_wifi_get_key_passphrase(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_key_passphrase:
		statval = call_qcsapi_wifi_set_key_passphrase(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_group_key_interval:
		statval = call_qcsapi_wifi_get_group_key_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_group_key_interval:
		statval = call_qcsapi_wifi_set_group_key_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_pairwise_key_interval:
		statval = call_qcsapi_wifi_get_pairwise_key_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_pairwise_key_interval:
		statval = call_qcsapi_wifi_set_pairwise_key_interval(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_pmf:
		statval = call_qcsapi_wifi_get_pmf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_pmf:
		statval = call_qcsapi_wifi_set_pmf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_wps_SSID:
		statval = call_qcsapi_SSID_get_wps_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_params:
		statval = call_qcsapi_SSID_get_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_params:
		statval = call_qcsapi_SSID_set_params(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_vlan_config:
		statval = call_qcsapi_wifi_vlan_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_show_vlan_config:
		statval = call_qcsapi_wifi_show_vlan_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_br_vlan_promisc:
		statval = call_qcsapi_enable_vlan_promisc(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_add_multicast:
		statval = call_qcsapi_set_multicast(p_calling_bundle, 1, argc, argv);
		break;

	case e_qcsapi_del_multicast:
		statval = call_qcsapi_set_multicast(p_calling_bundle, 0, argc, argv);
		break;

	case e_qcsapi_get_multicast_list:
		statval = call_qcsapi_get_multicast_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_add_ipff:
		statval = call_qcsapi_set_ipff(p_calling_bundle, 1, argc, argv);
		break;

	case e_qcsapi_del_ipff:
		statval = call_qcsapi_set_ipff(p_calling_bundle, 0, argc, argv);
		break;

	case e_qcsapi_get_ipff:
		statval = call_qcsapi_get_ipff(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rts_threshold:
		statval = call_qcsapi_wifi_get_rts_threshold(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_rts_threshold:
		statval = call_qcsapi_wifi_set_rts_threshold(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_mac_address_filtering:
		statval = call_qcsapi_wifi_get_mac_address_filtering(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_mac_address_filtering:
		statval = call_qcsapi_wifi_set_mac_address_filtering(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_is_mac_address_authorized:
		statval = call_qcsapi_wifi_is_mac_address_authorized(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_authorized_mac_addresses:
		statval = call_qcsapi_wifi_get_authorized_mac_addresses(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_denied_mac_addresses:
		statval = call_qcsapi_wifi_get_denied_mac_addresses(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_authorize_mac_address:
		statval = call_qcsapi_wifi_authorize_mac_address(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_deny_mac_address:
		statval = call_qcsapi_wifi_deny_mac_address(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_remove_mac_address:
		statval = call_qcsapi_wifi_remove_mac_address(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_clear_mac_address_filters:
		statval = call_qcsapi_wifi_clear_mac_address_filters(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_mac_address_reserve:
		statval = call_qcsapi_wifi_set_mac_address_reserve(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_mac_address_reserve:
		statval = call_qcsapi_wifi_get_mac_address_reserve(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_clear_mac_address_reserve:
		statval = call_qcsapi_wifi_clear_mac_address_reserve(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_backoff_fail_max:
		statval = call_qcsapi_wifi_backoff_fail_max(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_backoff_timeout:
		statval = call_qcsapi_wifi_backoff_timeout(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_registrar_report_button_press:
		statval = call_qcsapi_wps_registrar_report_button_press(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wps_registrar_report_pin:
		statval = call_qcsapi_wps_registrar_report_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_registrar_get_pp_devname:
		statval = call_qcsapi_wps_registrar_get_pp_devname(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_registrar_set_pp_devname:
		statval = call_qcsapi_wps_registrar_set_pp_devname(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_enrollee_report_button_press:
		statval = call_qcsapi_wps_enrollee_report_button_press(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wps_enrollee_report_pin:
		statval = call_qcsapi_wps_enrollee_report_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_enrollee_generate_pin:
		statval = call_qcsapi_wps_enrollee_generate_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_sta_pin:
		statval = call_qcsapi_wps_generate_random_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_ap_pin:
		statval = call_qcsapi_wps_get_ap_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_set_ap_pin:
		statval = call_qcsapi_wps_set_ap_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_save_ap_pin:
		statval = call_qcsapi_wps_save_ap_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_enable_ap_pin:
		statval = call_qcsapi_wps_enable_ap_pin(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_state:
		statval = call_qcsapi_wps_get_state(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_configured_state:
		statval = call_qcsapi_wps_get_configured_state(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_set_configured_state:
		statval = call_qcsapi_wps_set_configured_state(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_runtime_state:
		statval = call_qcsapi_wps_get_runtime_state(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_allow_pbc_overlap:
		statval = call_qcsapi_wps_allow_pbc_overlap(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_allow_pbc_overlap_status:
		statval = call_qcsapi_wps_get_allow_pbc_overlap_status(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wps_check_config:
		statval = call_qcsapi_wps_check_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_param:
		statval = call_qcsapi_wps_get_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_set_param:
		statval = call_qcsapi_wps_set_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_set_access_control:
		statval = call_qcsapi_wps_set_access_control(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_access_control:
		statval = call_qcsapi_wps_get_access_control(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_non_wps_set_pp_enable:
		statval = call_qcsapi_non_wps_set_pp_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_non_wps_get_pp_enable:
		statval = call_qcsapi_non_wps_get_pp_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_cancel:
		statval = call_qcsapi_wps_cancel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_set_pbc_in_srcm:
		statval = call_qcsapi_wps_set_pbc_in_srcm(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_get_pbc_in_srcm:
		statval = call_qcsapi_wps_get_pbc_in_srcm(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_timeout:
		statval = call_qcsapi_wps_set_timeout(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_on_hidden_ssid:
		statval = call_qcsapi_wps_on_hidden_ssid(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_on_hidden_ssid_status:
		statval = call_qcsapi_wps_on_hidden_ssid_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_upnp_enable:
		statval = call_qcsapi_wps_upnp_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_upnp_status:
		statval = call_qcsapi_wps_upnp_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_registrar_set_dfl_pbc_bss:
		statval = call_qcsapi_wps_registrar_set_dfl_pbc_bss(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wps_registrar_get_dfl_pbc_bss:
		statval = call_qcsapi_wps_registrar_get_dfl_pbc_bss(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_wpa_status:
		statval = call_qcsapi_wifi_get_wpa_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_auth_state:
		statval = call_qcsapi_wifi_get_auth_state(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_disconn_info:
		statval = call_qcsapi_wifi_get_disconn_info(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_reset_disconn_info:
		statval = call_qcsapi_wifi_reset_disconn_info(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dwell_times:
		statval = call_qcsapi_wifi_set_dwell_times(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dwell_times:
		statval = call_qcsapi_wifi_get_dwell_times(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_bgscan_dwell_times:
		statval = call_qcsapi_wifi_set_bgscan_dwell_times(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bgscan_dwell_times:
		statval = call_qcsapi_wifi_get_bgscan_dwell_times(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_count_associations:
		statval = call_qcsapi_wifi_get_count_associations(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_associated_device_mac_addr:
		statval = call_qcsapi_wifi_get_associated_device_mac_addr(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_associated_device_ip_addr:
		statval = call_qcsapi_wifi_get_associated_device_ip_addr(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_link_quality:
		statval = call_qcsapi_wifi_get_link_quality(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rssi_per_association:
		statval = call_qcsapi_wifi_get_rssi_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rssi_in_dbm_per_association:
		statval = call_qcsapi_wifi_get_rssi_in_dbm_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_meas_rssi_in_dbm_per_association:
		statval = call_qcsapi_wifi_get_meas_rssi_in_dbm_per_association(p_calling_bundle,
				argc, argv);
		break;

	case e_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association:
		statval = call_qcsapi_wifi_get_meas_rssi_minmax_in_dbm_per_association
				(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_rssi_meas_period:
		statval = call_qcsapi_wifi_set_rssi_meas_period(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rssi_meas_period:
		statval = call_qcsapi_wifi_get_rssi_meas_period(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_snr_per_association:
		statval = call_qcsapi_wifi_get_snr_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_hw_noise_per_association:
		statval = call_qcsapi_wifi_get_hw_noise_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_rx_bytes_per_association:
		statval = call_qcsapi_wifi_get_rx_bytes_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_tx_bytes_per_association:
		statval = call_qcsapi_wifi_get_tx_bytes_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_rx_packets_per_association:
		statval = call_qcsapi_wifi_get_rx_packets_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_tx_packets_per_association:
		statval = call_qcsapi_wifi_get_tx_packets_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_tx_err_packets_per_association:
		statval = call_qcsapi_wifi_get_tx_err_packets_per_association(p_calling_bundle,
				argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_allretries_per_association:
		statval = call_qcsapi_wifi_get_tx_allretries_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_tx_exceed_retry_per_association:
		statval = call_qcsapi_wifi_get_tx_exceed_retry_per_association(p_calling_bundle,
				argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_retried_per_association:
		statval = call_qcsapi_wifi_get_tx_retried_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_tx_retried_percent_per_association:
		statval = call_qcsapi_wifi_get_tx_retried_percent_per_association(p_calling_bundle,
				argc, argv);
		break;

	case e_qcsapi_wifi_get_bw_per_association:
		statval = call_qcsapi_wifi_get_bw_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_phy_rate_per_association:
		call_qcsapi_wifi_get_tx_phy_rate_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rx_phy_rate_per_association:
		call_qcsapi_wifi_get_rx_phy_rate_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tx_mcs_per_association:
		call_qcsapi_wifi_get_tx_mcs_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_rx_mcs_per_association:
		call_qcsapi_wifi_get_rx_mcs_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_achievable_tx_phy_rate_per_association:
		call_qcsapi_wifi_get_achievable_tx_phy_rate_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_achievable_rx_phy_rate_per_association:
		call_qcsapi_wifi_get_achievable_rx_phy_rate_per_association(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_auth_enc_per_association:
		call_qcsapi_wifi_get_auth_enc_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_tput_caps:
		call_qcsapi_wifi_get_tput_caps(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_connection_mode:
		call_qcsapi_wifi_get_connection_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_vendor_per_association:
		call_qcsapi_wifi_get_vendor_per_association(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_max_mimo:
		call_qcsapi_wifi_get_max_mimo(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_counter:
		statval = call_qcsapi_wifi_get_node_counter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_param:
		statval = call_qcsapi_wifi_get_node_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_stats:
		statval = call_qcsapi_wifi_get_node_stats(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_list:
		statval = call_qcsapi_wifi_get_node_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_infoset:
		statval = call_qcsapi_wifi_get_node_infoset(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_node_infoset_all:
		statval = call_qcsapi_wifi_get_node_infoset_all(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_if_infoset:
		statval = call_qcsapi_wifi_get_if_infoset(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_max_queued:
		statval = call_qcsapi_wifi_get_max_queued(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_disassociate:
		statval = call_qcsapi_wifi_disassociate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_disassociate_sta:
		statval = call_qcsapi_wifi_disassociate_sta(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_reassociate:
		statval = call_qcsapi_wifi_reassociate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_associate:
		statval = call_qcsapi_wifi_associate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_update_bss_cfg:
		statval = call_qcsapi_wifi_update_bss_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_create_SSID:
		statval = call_qcsapi_SSID_create_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_remove_SSID:
		statval = call_qcsapi_SSID_remove_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_bss_cfg:
		statval = call_qcsapi_wifi_get_bss_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_verify_SSID:
		statval = call_qcsapi_SSID_verify_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_rename_SSID:
		statval = call_qcsapi_SSID_rename_SSID(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_SSID_list:
		statval = call_qcsapi_SSID_get_SSID_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_SSID2_list:
		statval = call_qcsapi_SSID_get_SSID2_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_protocol:
		statval = call_qcsapi_SSID_get_protocol(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_encryption_modes:
		statval = call_qcsapi_SSID_get_encryption_modes(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_group_encryption:
		statval = call_qcsapi_SSID_get_group_encryption(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_authentication_mode:
		statval = call_qcsapi_SSID_get_authentication_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_protocol:
		statval = call_qcsapi_SSID_set_protocol(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_encryption_modes:
		statval = call_qcsapi_SSID_set_encryption_modes(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_group_encryption:
		statval = call_qcsapi_SSID_set_group_encryption(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_authentication_mode:
		statval = call_qcsapi_SSID_set_authentication_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_pre_shared_key:
		statval = call_qcsapi_SSID_get_pre_shared_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_pre_shared_key:
		statval = call_qcsapi_SSID_set_pre_shared_key(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_add_radius_auth_server_cfg:
		statval = call_qcsapi_wifi_add_radius_auth_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_del_radius_auth_server_cfg:
		statval = call_qcsapi_wifi_del_radius_auth_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_radius_auth_server_cfg:
		statval = call_qcsapi_wifi_get_radius_auth_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_add_radius_acct_server_cfg:
		statval = call_qcsapi_wifi_add_radius_acct_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_del_radius_acct_server_cfg:
		statval = call_qcsapi_wifi_del_radius_acct_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_radius_acct_server_cfg:
		statval = call_qcsapi_wifi_get_radius_acct_server_cfg(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_own_ip_addr:
		statval = call_qcsapi_wifi_set_own_ip_addr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_key_passphrase:
		statval = call_qcsapi_SSID_get_key_passphrase(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_key_passphrase:
		statval = call_qcsapi_SSID_set_key_passphrase(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_get_pmf:
		statval = call_qcsapi_SSID_get_pmf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_SSID_set_pmf:
		statval = call_qcsapi_SSID_set_pmf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scan_chan_list:
		statval = call_qcsapi_wifi_get_scan_chan_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scan_chan_list:
		statval = call_qcsapi_wifi_set_scan_chan_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_start_scan:
		statval = call_qcsapi_wifi_start_scan(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_cancel_scan:
		statval = call_qcsapi_wifi_cancel_scan(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scan_status:
		statval = call_qcsapi_wifi_get_scan_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_cac_status:
		statval = call_qcsapi_wifi_get_cac_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wait_scan_completes:
		statval = call_qcsapi_wifi_wait_scan_completes(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scan_chk_inv:
		statval = call_qcsapi_wifi_set_scan_chk_inv(p_calling_bundle, argc, argv);

		break;

	case e_qcsapi_wifi_get_scan_chk_inv:
		statval = call_qcsapi_wifi_get_scan_chk_inv(p_calling_bundle, argc, argv);

		break;

	case e_qcsapi_wifi_start_cca:
		statval = call_qcsapi_wifi_start_cca(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_disable_wps:
		statval = call_qcsapi_wifi_disable_wps(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_results_AP_scan:
		statval = call_qcsapi_wifi_get_results_AP_scan(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_count_APs_scanned:
		statval = call_qcsapi_wifi_get_count_APs_scanned(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_properties_AP:
		statval = call_qcsapi_wifi_get_properties_AP(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_wps_ie_scanned_AP:
		statval = call_qcsapi_wifi_get_wps_ie_scanned_AP(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_mcs_rate:
		statval = call_qcsapi_wifi_get_mcs_rate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_mcs_rate:
		statval = call_qcsapi_wifi_set_mcs_rate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_time_associated_per_association:
		statval = call_qcsapi_wifi_get_time_associated_per_association(p_calling_bundle,
				argc, argv);
		break;

	case e_qcsapi_wifi_wds_add_peer:
		statval = call_qcsapi_wifi_wds_add_peer(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wds_remove_peer:
		statval = call_qcsapi_wifi_wds_remove_peer(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wds_get_peer_address:
		statval = call_qcsapi_wifi_wds_get_peer_address(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wds_set_psk:
		statval = call_qcsapi_wifi_wds_set_psk(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wds_set_mode:
		statval = call_qcsapi_wifi_wds_set_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_wds_get_mode:
		statval = call_qcsapi_wifi_wds_get_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_qos_get_param:
		statval = call_qcsapi_wifi_qos_get_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_qos_set_param:
		statval = call_qcsapi_wifi_qos_set_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_wmm_ac_map:
		statval = call_qcsapi_wifi_get_wmm_ac_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_wmm_ac_map:
		statval = call_qcsapi_wifi_set_wmm_ac_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dscp_8021p_map:
		statval = call_qcsapi_wifi_get_dscp_8021p_map(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dscp_8021p_map:
		statval = call_qcsapi_wifi_set_dscp_8021p_map(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dscp_ac_map:
		statval = call_qcsapi_wifi_get_dscp_ac_map(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dscp_ac_map:
		statval = call_qcsapi_wifi_set_dscp_ac_map(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dscp_ac_table:
		statval = call_qcsapi_wifi_get_dscp_ac_table(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dscp_ac_table:
		statval = call_qcsapi_wifi_set_dscp_ac_table(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dscp_tid_table:
		statval = call_qcsapi_wifi_get_dscp_tid_table(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dscp_tid_table:
		statval = call_qcsapi_wifi_set_dscp_tid_table(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dscp_vap_link:
		statval = call_qcsapi_wifi_get_dscp_vap_link(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_dscp_vap_link:
		statval = call_qcsapi_wifi_set_dscp_vap_link(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_ac_agg_hold_time:
		statval = call_qcsapi_wifi_get_ac_agg_hold_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ac_agg_hold_time:
		statval = call_qcsapi_wifi_set_ac_agg_hold_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_qos_map:
		statval = call_qcsapi_wifi_set_qos_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_del_qos_map:
		statval = call_qcsapi_wifi_del_qos_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_qos_map:
		statval = call_qcsapi_wifi_get_qos_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_send_qos_map_conf:
		statval = call_qcsapi_wifi_send_qos_map_conf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dscp_tid_map:
		statval = call_qcsapi_wifi_get_dscp_tid_map(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_priority:
		statval = call_qcsapi_wifi_get_priority(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_priority:
		statval = call_qcsapi_wifi_set_priority(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_airfair:
		statval = call_qcsapi_wifi_get_airfair(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_airfair:
		statval = call_qcsapi_wifi_set_airfair(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_airquota:
		statval = call_qcsapi_wifi_get_airquota(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_airquota:
		statval = call_qcsapi_wifi_set_airquota(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_airquota_node:
		statval = call_qcsapi_wifi_get_airquota_node(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_airquota_node:
		statval = call_qcsapi_wifi_set_airquota_node(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_premier_list:
		statval = call_qcsapi_wifi_get_premier_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_premier_list:
		statval = call_qcsapi_wifi_set_premier_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_premier_rule:
		statval = call_qcsapi_wifi_get_premier_rule(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_premier_rule:
		statval = call_qcsapi_wifi_set_premier_rule(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_config_get_parameter:
		statval = call_qcsapi_config_get_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_config_update_parameter:
		statval = call_qcsapi_config_update_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_config_get_ssid_parameter:
		statval = call_qcsapi_config_get_ssid_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_config_update_ssid_parameter:
		statval = call_qcsapi_config_update_ssid_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_service_control:
		statval = call_qcsapi_service_control(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wfa_cert:
		statval = call_qcsapi_wfa_cert(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_enable_scs:
		statval = call_qcsapi_wifi_scs_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_scs_set_version:
		statval = call_qcsapi_wifi_scs_set_version(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_scs_switch_channel:
		statval = call_qcsapi_wifi_scs_switch_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_scs_pick_best_channel:
		statval = call_qcsapi_wifi_scs_pick_best_channel(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_verbose:
		statval = call_qcsapi_wifi_set_scs_verbose(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_status:
		statval = call_qcsapi_wifi_get_scs_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_smpl_enable:
		statval = call_qcsapi_wifi_set_scs_smpl_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_active_chan_list:
		statval = call_qcsapi_wifi_set_scs_active_chan_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_active_chan_list:
		statval = call_qcsapi_wifi_get_scs_active_chan_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_smpl_dwell_time:
		statval = call_qcsapi_wifi_set_scs_smpl_dwell_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_smpl_dwell_time:
		statval = call_qcsapi_wifi_get_scs_smpl_dwell_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_smpl_intv:
		statval = call_qcsapi_wifi_set_scs_sample_intv(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_smpl_type:
		statval = call_qcsapi_wifi_set_scs_sample_type(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_smpl_intv:
		statval = call_qcsapi_wifi_get_scs_sample_intv(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_intf_detect_intv:
		statval = call_qcsapi_wifi_set_scs_intf_detect_intv(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_thrshld:
		statval = call_qcsapi_wifi_set_scs_thrshld(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_report_only:
		statval = call_qcsapi_wifi_set_scs_report_only(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_report_stat:
		statval = call_qcsapi_wifi_get_scs_report(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_cca_intf_smth_fctr:
		statval = call_qcsapi_wifi_set_scs_cca_intf_smth_fctr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_chan_mtrc_mrgn:
		statval = call_qcsapi_wifi_set_scs_chan_mtrc_mrgn(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_nac_monitor_mode:
		statval = call_qcsapi_wifi_set_scs_nac_monitor_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_dfs_reentry_request:
		statval = call_qcsapi_wifi_get_scs_dfs_reentry_request(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_get_scs_cca_intf:
		statval = call_qcsapi_wifi_get_scs_cca_intf(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_param:
		statval = call_qcsapi_wifi_get_scs_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_stats:
		statval = call_qcsapi_wifi_set_scs_stats(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_burst_enable:
		statval = call_qcsapi_wifi_set_scs_burst_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_burst_window:
		statval = call_qcsapi_wifi_set_scs_burst_window(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_burst_thresh:
		statval = call_qcsapi_wifi_set_scs_burst_thresh(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_burst_pause:
		statval = call_qcsapi_wifi_set_scs_burst_pause(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_burst_switch:
		statval = call_qcsapi_wifi_set_scs_burst_switch(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_scs_chan_pool:
		statval = call_qcsapi_wifi_get_scs_chan_pool(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_scs_chan_pool:
		statval = call_qcsapi_wifi_set_scs_chan_pool(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_set_acs_param:
		statval = call_qcsapi_set_acs_param(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_start_ocac:
		statval = call_qcsapi_wifi_start_ocac(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_stop_ocac:
		statval = call_qcsapi_wifi_stop_ocac(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_ocac_status:
		statval = call_qcsapi_wifi_get_ocac_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ocac_threshold:
		statval = call_qcsapi_wifi_set_ocac_threshold(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ocac_dwell_time:
		statval = call_qcsapi_wifi_set_ocac_dwell_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ocac_duration:
		statval = call_qcsapi_wifi_set_ocac_duration(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ocac_cac_time:
		statval = call_qcsapi_wifi_set_ocac_cac_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ocac_report_only:
		statval = call_qcsapi_wifi_set_ocac_report_only(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_start_dfs_s_radio:
		statval = call_qcsapi_wifi_start_dfs_s_radio(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_stop_dfs_s_radio:
		statval = call_qcsapi_wifi_stop_dfs_s_radio(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dfs_s_radio_status:
		statval = call_qcsapi_wifi_get_dfs_s_radio_status(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_dfs_s_radio_availability:
		statval = call_qcsapi_wifi_get_dfs_s_radio_availability(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_threshold:
		statval = call_qcsapi_wifi_set_dfs_s_radio_threshold(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_dwell_time:
		statval = call_qcsapi_wifi_set_dfs_s_radio_dwell_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_duration:
		statval = call_qcsapi_wifi_set_dfs_s_radio_duration(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_cac_time:
		statval = call_qcsapi_wifi_set_dfs_s_radio_cac_time(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_report_only:
		statval = call_qcsapi_wifi_set_dfs_s_radio_report_only(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_wea_duration:
		statval = call_qcsapi_wifi_set_dfs_s_radio_wea_duration(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_wea_cac_time:
		statval = call_qcsapi_wifi_set_dfs_s_radio_wea_cac_time(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time:
		statval = call_qcsapi_wifi_set_dfs_s_radio_wea_dwell_time(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_wifi_set_vendor_fix:
		statval = call_qcsapi_wifi_set_vendor_fix(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_ap_isolate:
		statval = call_qcsapi_wifi_set_ap_isolate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_ap_isolate:
		statval = call_qcsapi_wifi_get_ap_isolate(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_power_save:
		statval = call_qcsapi_pm_get_set_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_qpm_level:
		statval = call_qcsapi_qpm_get_level(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_interface_stats:
		statval = call_qcsapi_get_interface_stats(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_vap_extstats:
		statval = call_qcsapi_get_vap_extstats(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_phy_stats:
		statval = call_qcsapi_get_phy_stats(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_bootcfg_get_parameter:
		statval = call_qcsapi_bootcfg_get_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_bootcfg_update_parameter:
		statval = call_qcsapi_bootcfg_update_parameter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_bootcfg_commit:
		statval = call_qcsapi_bootcfg_commit(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_telnet_enable:
		statval = call_qcsapi_telnet_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_restore_default_config:
		statval = call_qcsapi_restore_default_config(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_reset_all_stats:
		statval = call_qcsapi_reset_all_counters(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_run_script:
		statval = call_qcsapi_run_script(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_pairing_id:
		statval = call_qcsapi_wifi_get_pairing_id(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_pairing_id:
		statval = call_qcsapi_wifi_set_pairing_id(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_pairing_enable:
		statval = call_qcsapi_wifi_get_pairing_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_pairing_enable:
		statval = call_qcsapi_wifi_set_pairing_enable(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_txqos_sched_tbl:
		statval = call_qcsapi_wifi_set_txqos_sched_tbl(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_txqos_sched_tbl:
		statval = call_qcsapi_wifi_get_txqos_sched_tbl(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_eth_phy_power_off:
		statval = call_qcsapi_eth_phy_power_off(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_aspm_l1:
		statval = call_qcsapi_set_aspm_l1(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_l1:
		statval = call_qcsapi_set_l1(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_test_traffic:
		statval = call_qcsapi_test_traffic(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_temperature:
		statval = call_qcsapi_get_temperature(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_set_accept_oui_filter:
		statval = call_qcsapi_set_accept_oui_filter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_accept_oui_filter:
		statval = call_qcsapi_get_accept_oui_filter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_get_swfeat_list:
		statval = call_qcsapi_get_swfeat_list(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_vht:
		statval = call_qcsapi_wifi_set_vht(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_vht:
		statval = call_qcsapi_wifi_get_vht(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_set_he:
		statval = call_qcsapi_wifi_set_he(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_get_he:
		statval = call_qcsapi_wifi_get_he(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_set_test_mode:
		statval = call_qcsapi_calcmd_set_test_mode(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_show_test_packet:
		statval = call_qcsapi_calcmd_show_test_packet(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_send_test_packet:
		statval = call_qcsapi_calcmd_send_test_packet(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_stop_test_packet:
		statval = call_qcsapi_calcmd_stop_test_packet(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_send_dc_cw_signal:
		statval = call_qcsapi_calcmd_send_dc_cw_signal(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_stop_dc_cw_signal:
		statval = call_qcsapi_calcmd_stop_dc_cw_signal(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_test_mode_antenna_sel:
		statval = call_qcsapi_calcmd_get_test_mode_antenna_sel(p_calling_bundle, argc,
				argv);
		break;

	case e_qcsapi_calcmd_get_test_mode_mcs:
		statval = call_qcsapi_calcmd_get_test_mode_mcs(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_test_mode_bw:
		statval = call_qcsapi_calcmd_get_test_mode_bw(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_tx_power:
		statval = call_qcsapi_calcmd_get_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_set_tx_power:
		statval = call_qcsapi_calcmd_set_tx_power(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_real_time_txpower:
		statval = call_qcsapi_calcmd_get_real_time_txpower(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_test_mode_rssi:
		statval = call_qcsapi_calcmd_get_test_mode_rssi(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_set_mac_filter:
		statval = call_qcsapi_calcmd_set_mac_filter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_antenna_count:
		statval = call_qcsapi_calcmd_get_antenna_count(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_clear_counter:
		statval = call_qcsapi_calcmd_clear_counter(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_calcmd_get_info:
		statval = call_qcsapi_calcmd_get_info(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_set_soc_macaddr:
		statval = call_qcsapi_wifi_set_soc_macaddr(p_calling_bundle, argc, argv);
		break;

	case e_qcsapi_wifi_disable_dfs_channels:
		statval = call_qcsapi_disable_dfs_channels(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dfs_channels_status:
		statval = call_qcsapi_get_dfs_channels_status(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_carrier_id:
		statval = call_qcsapi_get_carrier_id(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_carrier_id:
		statval = call_qcsapi_set_carrier_id(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_platform_id:
		statval = call_qcsapi_get_platform_id(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_spinor_jedecid:
		statval = call_qcsapi_wifi_get_spinor_jedecid(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_custom_value:
		statval = call_qcsapi_wifi_get_custom_value(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_custom_value:
		statval = call_qcsapi_wifi_set_custom_value(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_vco_lock_detect_mode:
		statval = call_qcsapi_wifi_get_vco_lock_detect_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_vco_lock_detect_mode:
		statval = call_qcsapi_wifi_set_vco_lock_detect_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mlme_stats_per_mac:
		statval = call_qcsapi_wifi_get_mlme_stats_per_mac(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mlme_stats_per_association:
		statval = call_qcsapi_wifi_get_mlme_stats_per_association(p_calling_bundle, argc,
				argv);
		break;
	case e_qcsapi_wifi_get_mlme_stats_macs_list:
		statval = call_qcsapi_wifi_get_mlme_stats_macs_list(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_nss_cap:
		statval = call_qcsapi_wifi_get_nss_cap(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_nss_cap:
		statval = call_qcsapi_wifi_set_nss_cap(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_security_defer_mode:
		statval = call_qcsapi_wifi_get_security_defer_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_security_defer_mode:
		statval = call_qcsapi_wifi_set_security_defer_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_apply_security_config:
		statval = call_qcsapi_wifi_apply_security_config(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_intra_bss_isolate:
		statval = call_qcsapi_wifi_set_intra_bss_isolate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_intra_bss_isolate:
		statval = call_qcsapi_wifi_get_intra_bss_isolate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_bss_isolate:
		statval = call_qcsapi_wifi_set_bss_isolate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_bss_isolate:
		statval = call_qcsapi_wifi_get_bss_isolate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_host_state:
		statval = call_qcsapi_wifi_host_state_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_match_type:
		statval = call_qcsapi_wifi_wowlan_match_type_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_L2_type:
		statval = call_qcsapi_wifi_wowlan_L2_type_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_udp_port:
		statval = call_qcsapi_wifi_wowlan_udp_port_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_pattern:
		statval = call_qcsapi_wifi_wowlan_pattern_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_get_host_state:
		statval = call_qcsapi_wifi_host_state_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_get_match_type:
		statval = call_qcsapi_wifi_wowlan_match_type_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_get_L2_type:
		statval = call_qcsapi_wifi_wowlan_L2_type_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_get_udp_port:
		statval = call_qcsapi_wifi_wowlan_udp_port_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wowlan_get_pattern:
		statval = call_qcsapi_wifi_wowlan_pattern_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_extender_params:
		statval = call_qcsapi_wifi_set_extender_params(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_extender_status:
		statval = call_qcsapi_wifi_get_extender_status(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_enable_bgscan:
		statval = call_qcsapi_wifi_enable_bgscan(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_bgscan_status:
		statval = call_qcsapi_wifi_get_bgscan_status(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_uboot_info:
		statval = call_qcsapi_get_uboot_info(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_disassoc_reason:
		statval = call_qcsapi_wifi_get_disassoc_reason(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_is_startprod_done:
		statval = call_qcsapi_is_startprod_done(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_bb_param:
		statval = call_qcsapi_wifi_get_bb_param(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_bb_param:
		statval = call_qcsapi_wifi_set_bb_param(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_tx_amsdu:
		statval = call_qcsapi_wifi_get_tx_amsdu(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_tx_amsdu:
		statval = call_qcsapi_wifi_set_tx_amsdu(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_scan_buf_max_size:
		statval = call_qcsapi_wifi_set_scan_buf_max_size(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_scan_buf_max_size:
		statval = call_qcsapi_wifi_get_scan_buf_max_size(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_scan_table_max_len:
		statval = call_qcsapi_wifi_set_scan_table_max_len(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_scan_table_max_len:
		statval = call_qcsapi_wifi_get_scan_table_max_len(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_enable_mu:
		statval = call_qcsapi_wifi_set_enable_mu(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_enable_mu:
		statval = call_qcsapi_wifi_get_enable_mu(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_mu_use_precode:
		statval = call_qcsapi_wifi_set_mu_use_precode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mu_use_precode:
		statval = call_qcsapi_wifi_get_mu_use_precode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_mu_use_eq:
		statval = call_qcsapi_wifi_set_mu_use_eq(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mu_use_eq:
		statval = call_qcsapi_wifi_get_mu_use_eq(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mu_groups:
		statval = call_qcsapi_wifi_get_mu_groups(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_send_file:
		statval = call_qcsapi_send_file(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_emac_switch:
		statval = call_qcsapi_get_emac_switch(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_emac_switch:
		statval = call_qcsapi_set_emac_switch(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_eth_dscp_map:
		statval = call_qcsapi_eth_dscp_map(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_optim_stats:
		statval = call_qcsapi_wifi_set_optim_stats(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_pref_band:
		statval = call_qcsapi_wifi_set_pref_band(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_pref_band:
		statval = call_qcsapi_wifi_get_pref_band(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_sys_time:
		statval = call_qcsapi_set_sys_time(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_sys_time:
		statval = call_qcsapi_get_sys_time(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_eth_info:
		statval = call_qcsapi_get_eth_info(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_verify_repeater_mode:
		statval = call_qcsapi_wifi_verify_repeater_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_ap_interface_name:
		statval = call_qcsapi_wifi_set_ap_interface_name(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_ap_interface_name:
		statval = call_qcsapi_wifi_get_ap_interface_name(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_block_bss:
		statval = call_qcsapi_wifi_block_bss(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_txba_disable:
		statval = call_qcsapi_wifi_set_txba_disable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_txba_disable:
		statval = call_qcsapi_wifi_get_txba_disable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_rxba_decline:
		statval = call_qcsapi_wifi_set_rxba_decline(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_rxba_decline:
		statval = call_qcsapi_wifi_get_rxba_decline(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_txburst:
		statval = call_qcsapi_wifi_set_txburst(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_txburst:
		statval = call_qcsapi_wifi_get_txburst(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_sec_chan:
		statval = call_qcsapi_wifi_get_sec_chan(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_sec_chan:
		statval = call_qcsapi_wifi_set_sec_chan(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_vap_default_state:
		statval = call_qcsapi_set_vap_default_state(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_vap_default_state:
		statval = call_qcsapi_get_vap_default_state(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_tx_airtime:
		statval = call_qcsapi_wifi_get_tx_airtime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_qwe_command:
		statval = call_qcsapi_qwe_command(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_core_dump:
		statval = call_qcsapi_get_core_dump2(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_gather_info:
		statval = call_qcsapi_gather_info(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_client_mac_list:
		statval = call_qcsapi_get_client_mac_list(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_sample_all_clients:
		statval = call_qcsapi_wifi_sample_all_clients(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_per_assoc_data:
		statval = call_qcsapi_wifi_get_per_assoc_data(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_wifi_ready:
		statval = call_qcsapi_get_wifi_ready(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_cca_stats:
		statval = call_qcsapi_get_cca_stats(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_rf_chains:
		statval = call_qcsapi_set_rf_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_rf_chains:
		statval = call_qcsapi_get_rf_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_tx_chains:
		statval = call_qcsapi_wifi_set_tx_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_tx_chains:
		statval = call_qcsapi_wifi_get_tx_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_rx_chains:
		statval = call_qcsapi_wifi_set_rx_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_rx_chains:
		statval = call_qcsapi_wifi_get_rx_chains(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_vapdebug:
		statval = call_qcsapi_wifi_set_vapdebug(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_vapdebug:
		statval = call_qcsapi_wifi_get_vapdebug(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_eap_reauth_period:
		statval = call_qcsapi_wifi_set_eap_reauth_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_eap_reauth_period:
		statval = call_qcsapi_wifi_get_eap_reauth_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_eap_reauth_period:
		statval = call_qcsapi_wifi_remove_eap_reauth_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_cs_thrshld_range:
		statval = call_qcsapi_wifi_set_cs_thrshld_range(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_cs_thrshld_range:
		statval = call_qcsapi_wifi_get_cs_thrshld_range(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_cs_thrshld_inuse:
		statval = call_qcsapi_wifi_set_cs_thrshld_inuse(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_cs_thrshld_inuse:
		statval = call_qcsapi_wifi_get_cs_thrshld_inuse(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_igmp_snooping_state:
		statval = call_qcsapi_get_igmp_snoop(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_igmp_snooping_state:
		statval = call_qcsapi_set_igmp_snoop(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_br_get_groups:
		statval = call_qcsapi_br_get_groups(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_br_get_interfaces:
		statval = call_qcsapi_br_get_interfaces(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_bsa_get_param:
		statval = call_qcsapi_bsa_get_parameter(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_bsa_set_param:
		statval = call_qcsapi_bsa_set_parameter(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_bsa_get_param_ext:
		statval = call_qcsapi_bsa_get_parameter_ext(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_bsa_set_param_ext:
		statval = call_qcsapi_bsa_set_parameter_ext(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_radius_max_retries:
		statval = call_qcsapi_wifi_set_radius_max_retries(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_radius_max_retries:
		statval = call_qcsapi_wifi_get_radius_max_retries(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_radius_max_retries:
		statval = call_qcsapi_wifi_remove_radius_max_retries(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_radius_num_failover:
		statval = call_qcsapi_wifi_set_radius_num_failover(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_radius_num_failover:
		statval = call_qcsapi_wifi_get_radius_num_failover(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_radius_num_failover:
		statval = call_qcsapi_wifi_remove_radius_num_failover(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_radius_timeout:
		statval = call_qcsapi_wifi_set_radius_timeout(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_radius_timeout:
		statval = call_qcsapi_wifi_get_radius_timeout(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_radius_timeout:
		statval = call_qcsapi_wifi_remove_radius_timeout(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_pmk_cache_enable:
		statval = call_qcsapi_wifi_set_pmk_cache_enable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_pmk_cache_enable:
		statval = call_qcsapi_wifi_get_pmk_cache_enable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_pmk_cache_lifetime:
		statval = call_qcsapi_wifi_set_pmk_cache_lifetime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_pmk_cache_lifetime:
		statval = call_qcsapi_wifi_get_pmk_cache_lifetime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_pmk_cache_lifetime:
		statval = call_qcsapi_wifi_remove_pmk_cache_lifetime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_max_auth_attempts:
		statval = call_qcsapi_wifi_set_max_auth_attempts(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_max_auth_attempts:
		statval = call_qcsapi_wifi_get_max_auth_attempts(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_lockout_period:
		statval = call_qcsapi_wifi_set_lockout_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_lockout_period:
		statval = call_qcsapi_wifi_get_lockout_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_lockout_period:
		statval = call_qcsapi_wifi_remove_lockout_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_id_request_period:
		statval = call_qcsapi_wifi_set_id_request_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_id_request_period:
		statval = call_qcsapi_wifi_get_id_request_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_id_request_period:
		statval = call_qcsapi_wifi_remove_id_request_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_auth_quiet_period:
		statval = call_qcsapi_wifi_set_auth_quiet_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_auth_quiet_period:
		statval = call_qcsapi_wifi_get_auth_quiet_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_remove_auth_quiet_period:
		statval = call_qcsapi_wifi_remove_auth_quiet_period(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wlmonitor_enable:
		statval = call_qcsapi_wifi_wlmonitor_enable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wlmonitor_rate_thres_config:
		statval = call_qcsapi_wifi_wlmonitor_rate_thres(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wlmonitor_period_thres_config:
		statval = call_qcsapi_wifi_wlmonitor_period_thres(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_beacon_phyrate:
		statval = call_qcsapi_wifi_get_beacon_phyrate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_beacon_phyrate:
		statval = call_qcsapi_wifi_set_beacon_phyrate(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_beacon_power_backoff:
		statval = call_qcsapi_wifi_get_beacon_power_backoff(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_beacon_power_backoff:
		statval = call_qcsapi_wifi_set_beacon_power_backoff(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_mgmt_power_backoff:
		statval = call_qcsapi_wifi_get_mgmt_power_backoff(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_mgmt_power_backoff:
		statval = call_qcsapi_wifi_set_mgmt_power_backoff(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_qdrv_set_hw_module_state:
		statval = call_qcsapi_set_hw_module_state(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_qdrv_get_hw_module_state:
		statval = call_qcsapi_get_hw_module_state(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_start_dcs_scan:
		statval = call_qcsapi_wifi_start_dcs_scan(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_stop_dcs_scan:
		statval = call_qcsapi_wifi_stop_dcs_scan(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_dcs_scan_params:
		statval = call_qcsapi_wifi_get_dcs_scan_params(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_vlan_loop_detect:
		statval = call_qcsapi_set_vlan_loop_detect(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_vlan_loop_detect:
		statval = call_qcsapi_get_vlan_loop_detect(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_bsa_get_sta_table:
		statval = call_qcsapi_bsa_get_sta_table(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_add_app_ie:
		statval = call_qcsapi_add_app_ie(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_remove_app_ie:
		statval = call_qcsapi_remove_app_ie(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_disable_11b:
		statval = call_qcsapi_wifi_disable_11b(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_nac_mon_mode:
		statval = call_qcsapi_wifi_set_nac_mon_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_nac_mon_mode:
		statval = call_qcsapi_wifi_get_nac_mon_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_nac_stats:
		statval = call_qcsapi_wifi_get_nac_stats(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_update_bootcfg_binfile:
		statval = call_qcsapi_update_bootcfg_binfile(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_ieee80211r:
		statval = call_qcsapi_wifi_set_ieee80211r(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_ieee80211r:
		statval = call_qcsapi_wifi_get_ieee80211r(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_11r_mobility_domain:
		statval = call_qcsapi_wifi_set_11r_mobility_domain(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_mobility_domain:
		statval = call_qcsapi_wifi_get_11r_mobility_domain(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_parameter:
		statval = call_qcsapi_wifi_get_parameter(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_11r_nas_id:
		statval = call_qcsapi_wifi_set_11r_nas_id(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_parameter:
		statval = call_qcsapi_wifi_set_parameter(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_nas_id:
		statval = call_qcsapi_wifi_get_11r_nas_id(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_11r_ft_over_ds:
		statval = call_qcsapi_wifi_set_11r_ft_over_ds(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_ft_over_ds:
		statval = call_qcsapi_wifi_get_11r_ft_over_ds(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_add_11r_neighbour:
		statval = call_qcsapi_wifi_add_11r_neighbour(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_del_11r_neighbour:
		statval = call_qcsapi_wifi_del_11r_neighbour(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_neighbour:
		statval = call_qcsapi_wifi_get_11r_neighbour(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_11r_r1_key_holder:
		statval = call_qcsapi_wifi_set_11r_r1_key_holder(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_r1_key_holder:
		statval = call_qcsapi_wifi_get_11r_r1_key_holder(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_11r_r0_key_lifetime:
		statval = call_qcsapi_wifi_set_11r_r0_key_lifetime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_11r_r0_key_lifetime:
		statval = call_qcsapi_wifi_get_11r_r0_key_lifetime(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_reg_chan_txpower_path_get:
		statval = call_qcsapi_reg_chan_txpower_path_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_chan_usable:
		statval = call_qcsapi_wifi_get_chan_usable(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn:
		statval = call_qcsapi_wifi_set_scs_leavedfs_chan_mtrc_mrgn(p_calling_bundle, argc,
				argv);
		break;
	case e_qcsapi_set_max_boot_cac_duration:
		statval = call_qcsapi_set_max_boot_cac_duration(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_icac_status:
		statval = call_qcsapi_get_icac_status(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_is_weather_channel:
		statval = call_qcsapi_wifi_is_weather_channel(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_reboot_cause:
		statval = call_qcsapi_get_reboot_cause(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_set_radio_pwr_save:
		statval = call_qcsapi_set_radio_pwr_save(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_put_radio_under_reset:
		statval = call_qcsapi_put_radio_under_reset(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_show_access_points:
		statval = call_qcsapi_wifi_show_access_points(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_pta_op_mode:
		statval = call_qcsapi_wifi_set_pta_op_mode(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_pta:
		statval = call_qcsapi_wifi_get_pta(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_reg_chan_txpower_backoff_set:
		statval = call_qcsapi_reg_chan_txpower_backoff_set(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_reg_chan_txpower_backoff_get:
		statval = call_qcsapi_reg_chan_txpower_backoff_get(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_grab_config:
		statval = call_qcsapi_grab_config(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_scs_band_margin_check:
		statval = call_qcsapi_wifi_set_scs_band_margin_check(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_scs_band_margin:
		statval = call_qcsapi_wifi_set_scs_band_margin(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_repeater_mode_cfg:
		statval = call_qcsapi_repeater_mode_cfg(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_set_urepeater_params:
		statval = call_qcsapi_set_urepeater_params(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_get_urepeater_params:
		statval = call_qcsapi_get_urepeater_params(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_save_config:
		statval = call_qcsapi_save_config(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_get_config_status:
		statval = call_qcsapi_get_config_status(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_dpp_parameter:
		statval = call_qcsapi_wifi_dpp_parameter(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_thres_min_tbl_set:
		statval = call_qcsapi_wifi_aacs_thres_min_tbl_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_thres_min_tbl_get:
		statval = call_qcsapi_wifi_aacs_thres_min_tbl_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_thres_max_tbl_set:
		statval = call_qcsapi_wifi_aacs_thres_max_tbl_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_thres_max_tbl_get:
		statval = call_qcsapi_wifi_aacs_thres_max_tbl_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_rssi_tbl_set:
		statval = call_qcsapi_wifi_aacs_vnode_rssi_tbl_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_rssi_tbl_get:
		statval = call_qcsapi_wifi_aacs_vnode_rssi_tbl_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_wgt_tbl_set:
		statval = call_qcsapi_wifi_aacs_vnode_wgt_tbl_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_wgt_tbl_get:
		statval = call_qcsapi_wifi_aacs_vnode_wgt_tbl_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_set:
		statval = call_qcsapi_wifi_aacs_vnode_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_vnode_get:
		statval = call_qcsapi_wifi_aacs_vnode_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set:
		statval = call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get:
		statval = call_qcsapi_wifi_aacs_dfs_thres_adj_tbl_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_excl_ch_set:
		statval = call_qcsapi_wifi_aacs_excl_ch_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_excl_ch_get:
		statval = call_qcsapi_wifi_aacs_excl_ch_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_alt_excl_ch_set:
		statval = call_qcsapi_wifi_aacs_alt_excl_ch_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_alt_excl_ch_get:
		statval = call_qcsapi_wifi_aacs_alt_excl_ch_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_sel_ch_set:
		statval = call_qcsapi_wifi_aacs_sel_ch_set(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_aacs_sel_ch_get:
		statval = call_qcsapi_wifi_aacs_sel_ch_get(
				p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_legacy_bbic_set:
		statval = call_qcsapi_wifi_set_legacy_bbic(p_calling_bundle, argc, argv);
		break;
	case e_qcsapi_wifi_legacy_bbic_get:
		statval = call_qcsapi_wifi_get_legacy_bbic(p_calling_bundle, argc, argv);
		break;
	default:
		print_out(print, "no interface program (yet) for QCS API enum %d\n",
				p_calling_bundle->caller_qcsapi);
	}

	return statval;
}

static int call_qcsapi(qcsapi_output *print, int argc, char *argv[])
{
	qcsapi_entry_point e_the_entry_point = e_qcsapi_nosuch_api;
	int ok_to_continue = 1;
	int expected_argc = 1;
	call_qcsapi_bundle calling_bundle;
	const struct qcsapi_entry *qcsapi_table_entry = NULL;
	int statval = 0;

	calling_bundle.caller_output = print;

	/*
	 *Argument count (argc) required to be at least 1, the name of the QCS API to be called.
	 */
	if (argc < 1) {
		print_out(print, "programming error in call_qcsapi, argc = %d\n", argc);
		ok_to_continue = 0;
	}

	if (ok_to_continue) {
		if (name_to_entry_point_enum(argv[0], &e_the_entry_point) == 0) {
			print_out(print, "QCSAPI entry point %s not found\n", argv[0]);
			ok_to_continue = 0;
		}
	}
	/*
	 *Selected QCSAPIs are NOT supported by call_qcsapi.
	 */
	if (ok_to_continue) {
		if (e_the_entry_point == e_qcsapi_gpio_monitor_reset_device) {
			print_out(print, "GPIO monitor reset device cannot be accessed from call_qcsapi\n");
			ok_to_continue = 0;
		}
	}

	if (ok_to_continue) {
		qcsapi_table_entry = entry_point_enum_to_table_entry(e_the_entry_point);

		if (qcsapi_table_entry == NULL) {
			print_out(print, "programming error in call_qcsapi, no entry for enum %d\n",
					(int)e_the_entry_point);
			ok_to_continue = 0;
		} else {
			/*
			 *Originally all APIs expected an interface name.  Now a few APIs apply to
			 *the entire system, and thus do not require an interface name.  These new
			 *APIs are identified as get system value and set system value.  Older APIs
			 *are identified as get and set APIs.  They require an interface, which now
			 *needs to be accounted for here.  And set system value APIs will require
			 *an additional parameter, the new system-wide value.
			 *
			 *APIs that expect an additional parameter (counters, rates, etc.) require
			 *an additional parameter APIs that expect an SSID AND an index (SSID get
			 *passphrase) require yet another parameter APIs that SET a value require
			 *yet another parameter
			 *
			 *No interdependencies.
			 */
			if (qcsapi_table_entry->e_typeof_api == e_qcsapi_get_api ||
					qcsapi_table_entry->e_typeof_api == e_qcsapi_set_api)
				expected_argc++;	// account for the interface
			if (qcsapi_table_entry->e_generic_param_type != e_qcsapi_none)
				expected_argc++;
			if (qcsapi_table_entry->e_generic_param_type == e_qcsapi_SSID_index)
				expected_argc++;
			if (qcsapi_table_entry->e_typeof_api == e_qcsapi_set_api ||
					qcsapi_table_entry->e_typeof_api ==
					e_qcsapi_set_system_value)
				expected_argc++;
			if (qcsapi_table_entry->e_typeof_api == e_qcsapi_set_api_without_parameter)
				expected_argc++;

			if (expected_argc > argc) {
				print_out(print, "Too few command line parameters in call_qcsapi, expected %d, found %d\n", expected_argc, argc);
				ok_to_continue = 0;
			}
		}

		if (ok_to_continue) {
			/* Eliminate the QCS API name from the argument list. */

			argc--;
			argv++;

			/* Begin filling in the calling bundle ... */

			calling_bundle.caller_qcsapi = e_the_entry_point;

			if (qcsapi_table_entry->e_typeof_api == e_qcsapi_get_api ||
					qcsapi_table_entry->e_typeof_api == e_qcsapi_set_api ||
					qcsapi_table_entry->e_typeof_api ==
					e_qcsapi_set_api_without_parameter) {
				calling_bundle.caller_interface = argv[0];
				argc--;
				argv++;
			} else
				calling_bundle.caller_interface = NULL;

			calling_bundle.caller_generic_parameter.generic_parameter_type =
					qcsapi_table_entry->e_generic_param_type;
		}
	}

	if (ok_to_continue) {
		if (calling_bundle.caller_generic_parameter.generic_parameter_type != e_qcsapi_none) {
			/* Again we checked previously that enough arguments were present ... */

			if (parse_generic_parameter_name(print, argv[0],
							&(calling_bundle.
									caller_generic_parameter))
					== 0)
				ok_to_continue = 0;
			else {
				/* And remove the parameter name from the argument list. */

				argc--;
				argv++;
			}
		}
	}

	if (ok_to_continue) {
		unsigned int iter;

		if (verbose_flag > 0) {
			print_out(print, "call QCSAPI: %s",
					entry_point_enum_to_name(calling_bundle.caller_qcsapi));

			if (qcsapi_table_entry->e_typeof_api == e_qcsapi_get_api ||
					qcsapi_table_entry->e_typeof_api == e_qcsapi_set_api ||
					qcsapi_table_entry->e_typeof_api ==
					e_qcsapi_set_api_without_parameter) {
				print_out(print, " %s", calling_bundle.caller_interface);
			}

			if (calling_bundle.caller_generic_parameter.generic_parameter_type !=
					e_qcsapi_none) {
				print_out(print, " ");
				dump_generic_parameter_name(print,
						&(calling_bundle.caller_generic_parameter));
			}

			if (argc > 0) {
				for (iter = 0; iter < argc; iter++)
					print_out(print, " %s", argv[iter]);
			}

			print_out(print, "\n");
		}

		if (call_qcsapi_init_count > 0) {
			if (call_qcsapi_init_count == 1) {
				if (qcsapi_init() != 0)
					return 1;
			} else {
				for (iter = 0; iter < call_qcsapi_init_count; iter++)
					qcsapi_init();
			}
		}

		if (call_count < 2) {
			statval = call_particular_qcsapi(&calling_bundle, argc, argv);
		} else {
			for (iter = 0; iter < call_count - 1; iter++) {
				call_particular_qcsapi(&calling_bundle, argc, argv);
				if (delay_time > 0) {
					sleep(delay_time);
				}
			}

			call_particular_qcsapi(&calling_bundle, argc, argv);
		}
	}
	return statval;
}

static int process_options(qcsapi_output *print, int argc, char **argv)
{
	int local_index = 0;

	while (local_index < argc && *(argv[local_index]) == '-') {
		char *option_arg = argv[local_index];
		unsigned int length_option = strlen(option_arg);

		if (length_option > 1) {
			char option_letter = option_arg[1];

			if (option_letter == 'v') {
				unsigned int index_2 = 1;

				while (option_arg[index_2] == 'v') {
					verbose_flag++;
					index_2++;
				}
			} else if (option_letter == 'q') {
				unsigned int index_2 = 1;

				while (option_arg[index_2] == 'q') {
					verbose_flag--;
					index_2++;
				}
			}
			/*
			 *Process all options that require a numeric (integer) value here.
			 */
			else if (option_letter == 'n' || option_letter == 'd'
					|| option_letter == 'i') {
				char *local_addr = NULL;

				if (length_option > 2) {
					local_addr = option_arg + 2;
				} else {
					if (local_index + 1 >= argc) {
						print_err(print, "Missing numeric value for %c option\n", option_letter);
					} else {
						local_index++;
						local_addr = argv[local_index];
					}

				}

				if (local_addr != NULL) {
					int min_value = 1;
					int local_value = atoi(local_addr);
					/*
					 *Most options require a numeric value to be greater than
					 *0.  'i' is an exception.
					 */
					if (option_letter == 'i')
						min_value = 0;

					if (local_value < min_value) {
						print_err(print, "Invalid numeric value %d for %c option\n", local_value, option_letter);
						return -EINVAL;
					} else {
						if (option_letter == 'n')
							call_count = (unsigned int)local_value;
						else if (option_letter == 'i')
							call_qcsapi_init_count =
									(unsigned int)local_value;
						else
							delay_time = (unsigned int)local_value;
					}
				}
				/*
				 *Error causing local_addr to be NULL has already been reported.
				 */
			} else if (option_letter == 'h') {
				if (local_index + 1 >= argc) {
					list_entry_point_names(print);
				} else {
					char *local_addr = NULL;

					local_index++;
					local_addr = argv[local_index];

					if (strcasecmp(local_addr, "options") == 0)
						list_option_names(print);
					else if (strcasecmp(local_addr, "entry_points") == 0)
						list_entry_point_names(print);
					else if (strcasecmp(local_addr, "counters") == 0)
						list_counter_names(print);
					else if (strcasecmp(local_addr, "per_node_params") == 0)
						list_per_node_param_names(print);
					else if (strcasecmp(local_addr, "board_parameters") == 0)
						list_board_parameter_names(print);
					else if (strcasecmp(local_addr, "wifi_parameters") == 0)
						list_wifi_parameter_names(print);
					else {
						print_err(print, "Unrecognized help option %s\n",
								local_addr);
						print_err(print, "Choose from 'entry_points', 'counters', 'options'," "'per_node_params', 'board_parameters', or " "'wifi_parameters'\n");
					}
				}

				return -EINVAL;
			} else if (option_letter == 'g') {
				char *reg;

				if (local_index + 1 >= argc) {
					return -EINVAL;
				}

				reg = argv[++local_index];

				grep_entry_point_names(print, reg);

				return -EINVAL;
			} else if (option_letter == 'f') {
				if (local_index + 1 >= argc) {
					print_err(print, "Missing numeric value for %c option\n",
							option_letter);
				} else {
					char *local_addr = NULL;

					local_index++;
					local_addr = argv[local_index];

					if (strcmp("force_NULL", local_addr) == 0) {
						internal_flags |= m_force_NULL_address;
					} else {
						print_err(print, "Unrecognized parameter %s for %c option\n", local_addr, option_letter);
					}
				}
			} else if (option_letter == 'u') {
				qcsapi_sem_disable();
			} else {
				print_out(print, "unrecognized option '%c'\n", option_letter);
			}
		}
		/*
		 *Control would take the non-existent else clause if the argument were just "-".
		 */
		local_index++;
	}

	if (verbose_flag > 1) {
		print_out(print, "Verbose flag: %d, call count: %u\n", verbose_flag, call_count);
	}

	return local_index;
}

static void call_qscapi_help(qcsapi_output *print)
{
	print_out(print, "Usage:\n");
	print_out(print, "    To get a parameter value: call_qcsapi <QCS API> <interface>\n");
	print_out(print, "                              call_qcsapi <QCS API> <interface> <type of parameter>\n");
	print_out(print, "    To set a parameter: call_qcsapi <QCS API> <interface> <parameter value>\n");
	print_out(print, "                        call_qcsapi <QCS API> <interface> <type of parameter> <parameter value>\n");
}

int qcsapi_main(qcsapi_output *print, int argc, char **argv)
{
	int ival;
	int exitval = 0;

	if (argc <= 1) {
		call_qscapi_help(print);
		return -EINVAL;
	}

	argc--;
	argv++;

	ival = process_options(print, argc, argv);
	if (ival < 0) {
		exitval = ival;
	} else {
		argc = argc - ival;
		argv += ival;

		exitval = call_qcsapi(print, argc, argv);
	}

	return exitval;
}
