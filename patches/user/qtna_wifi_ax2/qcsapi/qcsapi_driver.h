/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2009 - 2015 Quantenna Communications, Inc.          **
**                                                                           **
**  File        : qcsapi_driver.h                                            **
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

#ifndef _QCSAPI_DRIVER_H
#define _QCSAPI_DRIVER_H

#include "qcsapi_output.h"

/*
 * get_api and set_api expect an interface (wifi0_0, eth1_0, etc.).
 * get_system_value and set_system_value do not expect an interface. They apply to the entire
 * device.
 */
typedef enum {
	e_qcsapi_get_api = 1,
	e_qcsapi_set_api,
	e_qcsapi_get_system_value,
	e_qcsapi_set_system_value,
	/* Get API without interface name and other parameters */
	e_qcsapi_get_api_without_ifname_parameter,
	/* Set API with interface name but without other parameters */
	e_qcsapi_set_api_without_parameter,
	e_qcsapi_get_api_without_ifname,
	e_qcsapi_set_api_without_ifname,
} qcsapi_typeof_api;

typedef enum {
	e_qcsapi_none,
	e_qcsapi_option = 1,
	e_qcsapi_counter,
	e_qcsapi_rates,
	e_qcsapi_modulation,
	e_qcsapi_index,
	e_qcsapi_select_SSID,
	e_qcsapi_SSID_index,
	e_qcsapi_LED,
	e_qcsapi_file_path_config,
	e_qcsapi_board_parameter,
	e_qcsapi_extender_params,
	e_qcsapi_wifi_parameter,
	e_qcsapi_bcn_phyrate,
	e_qcsapi_hw_module,
	e_qcsapi_urepeater_params,
	e_qcsapi_eap_params,
	e_qcsapi_dpp_parameter,
	e_qcsapi_zsdfs_parameter,
} call_generic_param_type;

typedef struct qcsapi_generic_param {
	/* Some entry points take both an SSID and and index */
	qcsapi_unsigned_int		index;
	union {
		qcsapi_counter_type	counter;
		qcsapi_option_type	option;
		qcsapi_rate_type	typeof_rates;
		qcsapi_mimo_type	modulation;
		qcsapi_board_parameter_type board_param;
		char			SSID[IW_ESSID_STR_MAX_SIZE];
		qcsapi_extender_type type_of_extender;
		qcsapi_legacy_phyrate	phyrate;
		qcsapi_hw_module	hw_module;
		qcsapi_wifi_param_type	wifi_param_type;
		qcsapi_eap_param_type	eap_param_type;
		qcsapi_urepeater_type	urepeater_param_type;
		enum qcsapi_dpp_cmd_param_type	dpp_param_type;
		qcsapi_zsdfs_param	zsdfs_param;
	} parameter_type;
} qcsapi_generic_param;

typedef struct call_bundle {
	const struct call_table_entry_s	*call_entry;
	const char			*interface;
	qcsapi_generic_param		generic_param;
	qcsapi_output			*output;
	int				delay_time;
	int				call_cnt;
	int				init_cnt;
} call_bundle;

/*
 * These values are for the number of arguments passed to the individual call function,
 * after removing special arguments indicated by call_generic_param_type (such as index) and
 * qcsapi_typeof_api (such as interface name).
 */
struct call_table_entry_args {
	int	min;	/* Min number of args */
	int	max;	/* Max number of args, or -1 for not validated */
};

struct call_table_entry_s {
	int (*fn)(const call_bundle *cb, qcsapi_output *print, const char *interface,
							int argc, char *argv[]);
	const char			*cmdname;
	const char			*cmdname_alt;
	const char			*usage;
	struct call_table_entry_args	args;
	qcsapi_typeof_api		call_type;
	call_generic_param_type		generic_param_type;
};

#endif /* _QCSAPI_DRIVER_H */
