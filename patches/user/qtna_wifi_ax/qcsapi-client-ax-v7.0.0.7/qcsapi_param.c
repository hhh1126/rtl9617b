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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "qcsapi_param.h"

/*
 * Check if an entry exists in a parameter name table.
 */
int qcsapi_param_idx_is_defined(const struct qcsapi_param_name_tbl *tbl, uint32_t idx)
{
	return idx < tbl->size && tbl->ent[idx].name;
}

/*
 * Get an enum value from a parameter name table using the index.
 * This function assumes the supplied index is within the table range and returns
 * index 0 only to avoid overrun.
 */
uint32_t qcsapi_param_idx2enum(const struct qcsapi_param_name_tbl *tbl, uint32_t idx)
{
	if (idx >= tbl->size)
		return 0;

	return tbl->ent[idx].enum_val;
}

/*
 * Get a parameter name from a parameter name table using the index.
 */
const char *qcsapi_param_idx2name(const struct qcsapi_param_name_tbl *tbl, uint32_t idx)
{
	if (idx >= tbl->size)
		return "";

	return tbl->ent[idx].name;
}

/*
 * Get a parameter name from a parameter name table using the enum value.
 */
const char *qcsapi_param_enum2name(const struct qcsapi_param_name_tbl *tbl, uint32_t enum_val)
{
	uint32_t i;
	struct qcsapi_param_name_ent *ent = tbl->ent;

	for (i = 0; i < tbl->size; i++) {
		if (ent[i].enum_val == enum_val)
			return ent[i].name;
	}

	return "Invalid QCSAPI value";
}

/*
 * Get an enum value from a parameter table entry using the parameter name.
 */
int qcsapi_param_name2enum(const struct qcsapi_param_name_tbl *tbl, const char *name,
				uint32_t *enum_val)
{
	uint32_t i;
	struct qcsapi_param_name_ent *ent = &tbl->ent[0];

	for (i = 0; i < tbl->size; i++) {
		if (ent[i].name && strcasecmp(ent[i].name, name) == 0) {
			*enum_val = ent[i].enum_val;
			return 0;
		}
	}

	return -1;
}

