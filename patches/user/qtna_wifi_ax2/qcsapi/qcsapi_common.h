/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2018 Quantenna Communications Inc                   **
**                                                                           **
**  File        : qcsapi_common.h                                            **
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
 * The common QCSAPI header file containing data types etc.
 */

#ifndef _QCSAPI_COMMON_H
#define _QCSAPI_COMMON_H

/**
 * @addtogroup DetailedDataTypes
 * @{
 */

/**
 * \anchor QCSAPI_ERRNO
 *
 * \brief These are return codes that may be set by QCSAPIs.
 *
 * These are return codes that may be set by QCSAPIs.
 *
 * To get an error string associated with the error message, use the API call
 * qcsapi_errno_get_message.
 *
 * In addition to the error codes listed in the following sections (which start at
 * error number 1000 - <c>qcsapi_errno_base</c>), the following POSIX defined errors
 * are used in the QCSAPI:
 *
 * <TABLE>
 * <TR><TD>ERRNO value</TD><TD>QCSAPI Error</TD><TD>Description</TD></TR>
 * <TR><TD><c>-EFAULT</c></TD><TD>QCS API error 14: Bad address</TD>
 * <TD>The QCSAPI found a problem with an argument passed by reference;
 * most likely the address was the NULL address.</TD></TR>
 * <TR><TD><c>-EINVAL</c></TD><TD>QCS API error 22: Invalid argument</TD>
 * <TD>The QCSAPI found the value of an argument is not valid. Examples are
 * numeric value out of range (eg, WiFi channel larger than 255), or a parameter
 * value not allowed by the WiFi standard.</TD></TR>
 * <TR><TD><c>-ENODEV</c></TD><TD>QCS API error 19: No such device</TD>
 * <TD>No such device. An operation was attempted using a device that does not
 * exist.</TD></TR>
 * <TR><TD><c>-EOPNOTSUPP</c></TD><TD>QCS API error 95: Operation not supported</TD>
 * <TD>Operation not supported. For example, an operation limited to a WiFi device
 * such as get 802.11 standard or get beacon type was attempted using an interface
 * that is not a WiFi device.</TD></TR>
 * <TR><TD><c>-ERANGE</c></TD><TD>QCS API error 34: Parameter value out of range</TD>
 * <TD>This error occurs when the API accesses an element in an array using an index
 * parameter, and the index is too large or out of range. An example is the per-association
 * APIs.</TD></TR>
 * </TABLE>
 *
 * \sa qcsapi_errno_get_message
 */
enum qcsapi_errno {
	qcsapi_errno_base = 1000,
	/**
	 * This error code is returned when attempts are made to apply changes when
	 * the wireless system is not started. The most typical situation this error
	 * message is returned is when the Quantenna kernel modules have not been loaded.
	 *
	 * Many different QCSAPI function calls attempt to apply changes, and the
	 * majority of QCSAPI calls dealing with the wireless driver may return this
	 * value.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1000: System not started</c>
	 */
	qcsapi_system_not_started = qcsapi_errno_base,
	/**
	 * This error code is returned when an attempt to read in an unknown parameter
	 * via the qcsapi_config_get_parameter.
	 *
	 * \sa qcsapi_config_get_parameter
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1001: Parameter not found</c>
	 */
	qcsapi_parameter_not_found = qcsapi_errno_base + 1,
	/**
	 * This error code is returned when an SSID API call is made, but the SSID referred
	 * to does not exist.
	 *
	 * The SSID may not exist due to the config file being missing, or due to the config
	 * file not containing the passed in SSID. See \ref SSIDAPIs "SSID APIs".
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1002: SSID not found</c>
	 */
	qcsapi_SSID_not_found = qcsapi_errno_base + 2,
	/**
	 * This error code is returned when a QCSAPI call is attempted on an STA device, but
	 * the call only applies to the AP.
	 *
	 * This return value is used in many different QCSAPIs, across all functional areas.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1003: Operation only available on an AP</c>
	 */
	qcsapi_only_on_AP = qcsapi_errno_base + 3,
	/**
	 * This error code is returned when a QCSAPI call is attempted on an AP device, but
	 * the call only applies to the STA.
	 *
	 * This return value is used in many different QCSAPIs, across all functional areas.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1004: Operation only available on a STA</c>
	 */
	qcsapi_only_on_STA = qcsapi_errno_base + 4,
	/**
	 * This error code is returned when the action implied by the API conflicts with the
	 * current configuration.
	 *
	 * An example is getting a list of authorized MAC addresses when MAC address filtering
	 * is not enabled.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1005: Configuration error</c>
	 */
	qcsapi_configuration_error = qcsapi_errno_base + 5,
	/**
	 * This error code is returned when a variable length input buffer is too small for
	 * the QCSAPI result. For example, when retrieving error messages.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1006: Insufficient space in the string to receive results</c>
	 */
	qcsapi_buffer_overflow = qcsapi_errno_base + 6,
	/**
	 * This error code is returned when an internal error is detected when parsing config
	 * files or other data sets.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1007: Internal formatting error</c>
	 */
	qcsapi_internal_format_error = qcsapi_errno_base + 7,
	/**
	 * This error code is returned when a system call is made in the code and it fails for
	 * some reason.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1008: Internal API programming error</c>
	 */
	qcsapi_programming_error = qcsapi_errno_base + 8,
	/**
	 * This error code is returned when a QCSAPI call is made that is only supported in
	 * bringup mode.
	 *
	 * See @ref mysection4_1_5 "Production Mode vs Bringup Mode"
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1009: Operation only available in bringup mode</c>
	 */
	qcsapi_bringup_mode_only = qcsapi_errno_base + 9,
	/**
	 * This error code is returned when a socket connection to the security daemon (opened
	 * to send a command to the running daemon) fails for whatever reason.
	 *
	 * If this error is returned, one or more of the sequence of events in the QCSAPI call
	 * has failed, and the system may be in an inconsistent state.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1010: Cannot contact security manager</c>
	 */
	qcsapi_daemon_socket_error = qcsapi_errno_base + 10,
	/**
	 * This error code is returned when the request was rejected due to conflicting configuration.
	 */
	qcsapi_conflicting_options = qcsapi_errno_base + 11,
	/**
	 * This error code is returned when the SSID cannot be found (when searching to see if
	 * an SSID is present).
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1012: Required parameter not found in the SSID configuration block</c>
	 */
	qcsapi_SSID_parameter_not_found = qcsapi_errno_base + 12,
	/**
	 * This error code is returned when qcsapi_init has not been called prior to invoking
	 * certain APIs (that require qcsapi_init to be called).
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1013: Initialization API qcsapi_init has not been called</c>
	 */
	qcsapi_not_initialized = qcsapi_errno_base + 13,
	/**
	 * This error code is returned when the flash upgrade image is not a regular file on
	 * the filesystem (eg, is a directory or device special file).
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1014: Invalid file type for a flash image update file</c>
	 */
	qcsapi_invalid_type_image_file = qcsapi_errno_base + 14,
	/**
	 * This error code is returned when the flash upgrade image fails verification checks.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1015: chkimage utility failed for the flash image update file</c>
	 */
	qcsapi_image_file_failed_chkimage = qcsapi_errno_base + 15,
	/**
	 * This error code is returned when the flash upgrade partition is not found or is
	 * invalid.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1016: flash partition not found</c>
	 */
	qcsapi_flash_partition_not_found = qcsapi_errno_base + 16,
	/**
	 * This error code is returned when the command to erase the flash partition failed.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1017: failed to erase the flash memory partition</c>
	 */
	qcsapi_erase_flash_failed = qcsapi_errno_base + 17,
	/**
	 * This error code is returned when the copy of the flash image into the flag part
	 * failed.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1018: failed to copy the new image to the flash memory partition</c>
	 */
	qcsapi_copy_image_flash_failed = qcsapi_errno_base + 18,
	/**
	 * This error code is returned when a call is made into an API where the operational
	 * state of the system is not known. This is an internal error, and should never be
	 * seen in ordinary circumstances.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1019: invalid WiFi mode</c>
	 */
	qcsapi_invalid_wifi_mode = qcsapi_errno_base + 19,
	/**
	 * This error code is returned when the call to qcsapi_console_disconnect fails due
	 * to not enough system resources.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 * <c>QCS API error 1020: Process table is full</c>
	 */
	qcsapi_process_table_full = qcsapi_errno_base + 20,
	/**
	 * This error code is deprecated and not returned by any current API.
	 */
	qcsapi_measurement_not_available = qcsapi_errno_base + 21,
	/**
	 * This error code is returned when trying to create a new BSS, but the maximum number of
	 * BSSes are already created.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1022: Maximum number of BSSIDs / VAPs exceeded</c>
	 */
	qcsapi_too_many_bssids = qcsapi_errno_base + 22,
	/**
	 * This error code is returned when an operation is attempted on a non-primary interface.
	 * This can happen for certain security settings and when performing WDS functions.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1023: Operation only available on the primary WiFi interface</c>
	 */
	qcsapi_only_on_primary_interface = qcsapi_errno_base + 23,
	/**
	 * This error code is returned when trying to create a new WDS link, but the maximum
	 * number of WDS links are already created.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1024: Maximum number of WDS links exceeded</c>
	 */
	qcsapi_too_many_wds_links = qcsapi_errno_base + 24,
	/**
	 * This error code is returned when an attempt to update a config file (persistent file)
	 * fails.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1025: Failed to update persistent configuration</c>
	 */
	qcsapi_config_update_failed = qcsapi_errno_base + 25,
	/**
	 * This error code is returned when the /proc/net/dev or /proc/net/packets device files
	 * are not present on the filesystem.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1026: Cannot access network counters</c>
	 */
	qcsapi_no_network_counters = qcsapi_errno_base + 26,
	/**
	 * This error code is returned when the PM interval passed in is invalid.
	 * That is, it is not one of the supported interval device files.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1027: Invalid performance monitoring interval</c>
	 */
	qcsapi_invalid_pm_interval = qcsapi_errno_base + 27,
	/**
	 * This error code is returned when an operation relevant only to WDS mode is attempted on
	 * a non-WDS operational mode device.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1028: Operation only available on a WDS device</c>
	 */
	qcsapi_only_on_wds = qcsapi_errno_base + 28,
	/**
	 * This error code is returned when a multicast or broadcast MAC
	 * is used where only unicast MAC is allowed.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1029: Only unicast MAC address is allowed</c>
	 */
	qcsapi_only_unicast_mac = qcsapi_errno_base + 29,
	/**
	 * This error code is returned when performing an invalid operation.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1030: Operation is not available on the primary interface</c>
	 */
	qcsapi_primary_iface_forbidden = qcsapi_errno_base + 30,
	/**
	 * This error code is returned when attempting to create a BSS with an invalid interface
	 * name. The BSS name prefix must be the string 'wifi'.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1031: Invalid BSS name</c>
	 */
	qcsapi_invalid_ifname = qcsapi_errno_base + 31,
	/**
	 * This error code is returned when an error happens on interface.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1032: An error happened on interface</c>
	 */
	qcsapi_iface_error = qcsapi_errno_base + 32,
	/**
	 * This error code is returned when a semaphore takes too long to initialize.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1033: Semaphore initialization error</c>
	 */
	qcsapi_sem_error = qcsapi_errno_base + 33,
	/**
	 * This error code is returned when a command is issued for a feature that is not
	 * supported in this image.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1034: Feature is not supported</c>
	 */
	qcsapi_not_supported = qcsapi_errno_base + 34,
	/**
	 * This error code is returned when a channel as input is not a dfs channel
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1035: API requires a dfs channel</c>
	 */
	qcsapi_invalid_dfs_channel = qcsapi_errno_base + 35,
	/**
	 * This error code is returned when a file can not be found.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1036: Script failed</c>
	 */
	qcsapi_script_error = qcsapi_errno_base + 36,
	/**
	 * This error code is returned when set mac address of wds peer is local address.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1037: Local Mac address can't be used as wds peer address</c>
	 */
	qcsapi_invalid_wds_peer_addr = qcsapi_errno_base + 37,
	/**
	 * This error code is returned when band is not supported.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1038: Band is not supported</c>
	 */
	qcsapi_band_not_supported = qcsapi_errno_base + 38,
	/**
	 * This error code is returned when region is not supported.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1039: Region is not supported</c>
	 */
	qcsapi_region_not_supported = qcsapi_errno_base + 39,
	/**
	 * This error code is returned when region database is not found.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1040: Region database is not found</c>
	 */
	qcsapi_region_database_not_found = qcsapi_errno_base + 40,
	/**
	 * This error code is returned when a parameter name is not supported
	 * by wireless_conf.txt.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1041: Parameter name is not supported</c>
	 */
	qcsapi_param_name_not_supported = qcsapi_errno_base + 41,
	/**
	 * This error code is returned when parameter value is invalid.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1042: Parameter value is invalid</c>
	 */
	qcsapi_param_value_invalid = qcsapi_errno_base + 42,
	/**
	 * This error code is returned when an input MAC address is invalid
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1043: Invalid MAC address</c>
	 */
	qcsapi_invalid_mac_addr = qcsapi_errno_base + 43,
	/**
	 * This error code is returned when an option is not supported.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <QCS API error 1044: Option is not supported>
	 */
	qcsapi_option_not_supported = qcsapi_errno_base + 44,
	/**
	 * This error code is returned when a wps overlap detected
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <QCS API error 1045: WPS Overlap detected>
	 */
	qcsapi_wps_overlap_detected = qcsapi_errno_base + 45,
	/**
	 * This error code is returned when a statistics module is not supported
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <QCS API error 1046: MLME statistics is not supported>
	 */
	qcsapi_mlme_stats_not_supported = qcsapi_errno_base + 46,
	/**
	 * This error code is returned when a board parameter requested for is not supported.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <QCS API error 1047: Board parameter is not supported>
	 */
	qcsapi_board_parameter_not_supported = qcsapi_errno_base + 47,
	/**
	 * This error code is returned when a WDS peer cannot be added
	 * because the peer is currently associated as a station.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1048: WDS peer is associated</c>
	 */
	qcsapi_peer_in_assoc_table = qcsapi_errno_base + 48,
	/**
	 * This error code is returned when an operation is attempted on a mac address
	 * that is not in the association table, for example, because the station has
	 * disassociated.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1049: MAC address is not in association list</c>
	 */
	qcsapi_mac_not_in_assoc_list = qcsapi_errno_base + 49,
	/**
	 * This error code is returned when a parameter is specified too many times
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1050: param exceeds the limit </c>
	 */
	qcsapi_param_count_exceeded = qcsapi_errno_base + 50,
	/**
	 * This error code is returned when attempting to add a parameter
	 * that is already defined
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1049: duplicate param found</c>
	 */
	qcsapi_duplicate_param = qcsapi_errno_base + 51,
	/**
	 * This error code is returned when a QCSAPI call is not permitted on the
	 * specified interface.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1052: Operation is not supported on this interface</c>
	 */
	qcsapi_iface_invalid = qcsapi_errno_base + 52,
	/**
	 * This error code is returned when an invalid radio ID was provided to a QCSAPI call.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1053: invalid radio device index</c>
	 */
	qcsapi_invalid_radio_id = qcsapi_errno_base + 53,
	/**
	 * This error code is returned when the QCSAPI call is attempting to add too many
	 * values to multi-configuration options. For example, if more than three radius
	 * servers are added
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1054: Exceeds the allowed multi-config limit</c>
	 */
	qcsapi_exceed_config_number = qcsapi_errno_base + 54,
	/**
	 * This error code is returned when the QCSAPI call is no longer supported.
	 * Refer to the API documentation for information about the replacement
	 * function, if any.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1055: QCSAPI is deprecated</c>
	 */
	qcsapi_deprecated = qcsapi_errno_base + 55,
	/**
	 * This error code is returned when attempting to set an invalid or unsupported PHY rate.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1056: Invalid or unsupported 802.11 PHY rate</c>
	 */
	qcsapi_invalid_11b_phyrate = qcsapi_errno_base + 56,
	/**
	 * This error code is returned when mode is not station or rf chip id is not dual.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 *<c>QCS API error 1057: This API can only be used on a dual band radio in Station mode</c>
	 */
	qcsapi_only_dual_band_sta_mode = qcsapi_errno_base + 57,
	/**
	 * This error code is returned when the user is not authorized to perform the requested
	 * action.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 *<c>QCS API error 1058: Not authorized</c>
	 */
	qcsapi_not_authorized = qcsapi_errno_base + 58,
	/**
	 * This error code is returned when attempting to apply configuration to a secondary
	 * repeater interface when in a repeater mode that does not require configuration on
	 * the secondary interface.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1059: Secondary repeater interface is not configurable</c>
	 */
	qcsapi_secondary_repeater_not_configurable = qcsapi_errno_base + 59,
	/**
	 * This error code is returned when attempting to apply configuration that is not supported
	 * for the current repeater mode.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 *<c>QCS API error 1060: Invalid configuration for current repeater mode</c>
	 */
	qcsapi_invalid_config_for_repeater = qcsapi_errno_base + 60,
	/**
	 * This error code is returned when a QCSAPI call is attempted on airtime quota entry, but
	 * the call only applies to the throughput quota entry.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1061: Operation only available on throughput entry</c>
	 */
	qcsapi_only_on_throughput_qos_class = qcsapi_errno_base + 61,
	/**
	 * This error code is returned when a QCSAPI call is attempted on throughput quota entry,
	 * but the call only applies to the airtime quota entry.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1062: Operation only available on airtime entry</c>
	 */
	qcsapi_only_on_airtime_qos_class = qcsapi_errno_base + 62,
	/**
	 * This error code is returned when the requested parameter has not been configured.
	 *
	 * <c>call_qcsapi</c> printed error message:
	 *
	 * <c>QCS API error 1063: Requested parameter not configured</c>
	 */
	qcsapi_parameter_not_configured = qcsapi_errno_base + 63,
};

/** @} */

#endif /* _QCSAPI_COMMON_H */
