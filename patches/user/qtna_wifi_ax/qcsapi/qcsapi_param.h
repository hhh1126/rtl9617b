/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2020 Quantenna Communications, Inc.                 **
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
EH0*/

/*
 * Parameter name table look-up functions.
 */
#ifndef _QCSAPI_PARAM_H
#define _QCSAPI_PARAM_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define QCSAPI_PARAM_NAME_TBL_DECL(_tbl)	extern struct qcsapi_param_name_tbl _tbl
#define QCSAPI_PARAM_NAME_TBL_NAME_LEN		32
#define QCSAPI_PARAM_NAME_TBL(_tbl, _name) \
	struct qcsapi_param_name_tbl _tbl = { ARRAY_SIZE(_ ## _tbl), _name, _ ## _tbl }

struct qcsapi_param_name_ent {
	uint32_t	enum_val;
	const char	*name;
};

struct qcsapi_param_name_tbl {
	uint32_t	size;
	char		tbl_name[QCSAPI_PARAM_NAME_TBL_NAME_LEN];
	struct qcsapi_param_name_ent	*ent;
};

QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_counter);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_option);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_board_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_rate_types);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_wifi_std);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_partition);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_qos_queue);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_vendor_fix);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_qos_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_per_node_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_sys_status);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_scs_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_extender_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_wifi_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_eap);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_hw_module);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_ssid_fmt);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_phyrate);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_dpp_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_acs);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_param_name_zsdfs_param);
QCSAPI_PARAM_NAME_TBL_DECL(qcsapi_error_msg_thermal);

const char *qcsapi_param_enum2name(const struct qcsapi_param_name_tbl *tbl, uint32_t enum_val);
int qcsapi_param_name2enum(const struct qcsapi_param_name_tbl *tbl, const char *name,
				uint32_t *enum_val);
int qcsapi_param_idx_is_defined(const struct qcsapi_param_name_tbl *tbl, uint32_t idx);
const char *qcsapi_param_idx2name(const struct qcsapi_param_name_tbl *tbl, uint32_t idx);
uint32_t qcsapi_param_idx2enum(const struct qcsapi_param_name_tbl *tbl, uint32_t idx);

#endif /* _QCSAPI_PARAM_H */
