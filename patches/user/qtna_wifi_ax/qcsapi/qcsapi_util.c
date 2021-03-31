/*SH0
*******************************************************************************
**                                                                           **
**           Copyright (c) 2015 Quantenna Communications, Inc.               **
**                                                                           **
**  File        : qcsapi_util.h                                              **
**  Description : utility functions to be used by qcsapi_* and call_qcsapi   **
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

#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "qcsapi_util.h"

int qcsapi_verify_numeric(const char *parameter_value)
{
	while (*parameter_value != '\0') {
		if (!isdigit(*parameter_value))
			return -1;
		parameter_value++;
	}

	return 0;
}

int qcsapi_util_val_is_zero_or_one(const int val)
{
	if (val != 0 && val != 1)
		return 0;

	return 1;
}

/*
 * Conversion from hex string to unsigned integer.
 * Handles invalid strings and integer overflows.
 * return:
 *  0 - on success
 *  -1 - on error
 */
int qcsapi_util_hexstr_to_uint32(const char *str, uint32_t *result)
{
	char *endptr = NULL;
	uint32_t res;

	if (str == NULL)
		return -EFAULT;

	if (!strstr(str, "0x"))
		return -EINVAL;

	str += 2;

	if (!isdigit(*str) && !(*str <= 'f' && *str >= 'a') && !(*str <= 'F' && *str >= 'A'))
		return -EINVAL;

	errno = 0;
	res = strtoul(str, &endptr, 16);
	if (errno != 0)
		return -EINVAL;

	if (!endptr || endptr == str)
		return -EINVAL;

	if (*endptr != '\0')
		return -EINVAL;

	*result = res;
	return 0;
}

int qcsapi_str_to_uint32(const char *str, uint32_t *result)
{
	char *endptr = NULL;
	uint32_t res;

	while (isspace(*str))
		str++;

	if (!isdigit(*str))
		return -1;

	errno = 0;
	res = strtoul(str, &endptr, 10);
	if (errno != 0)
		return -1;

	if (!endptr || endptr == str)
		return -1;

	while (isspace(*endptr))
		endptr++;

	if (*endptr != '\0')
		return -1;

	*result = res;

	return 0;
}

int qcsapi_list_to_array32(const char *input_list, uint32_t *output_array,
		const uint32_t max_count, uint32_t *count)
{
	uint32_t cnt = 0;
	int retval = 0;
	const char *delim = ",";
	char *token;
	char *str = strdup(input_list);
	char *rest = str;

	if (!input_list || !output_array || !count || !str) {
		free(str);
		return -EINVAL;
	}

	while ((token = strtok_r(rest, delim, &rest))) {
		if (cnt >= max_count) {
			retval = -ERANGE;
			break;
		}

		retval = qcsapi_str_to_uint32(token, &output_array[cnt++]);

		if (retval != 0)
			break;
	}

	*count = cnt;
	free(str);

	return retval;
}

int validate_mac_addr(const char *mac_addr_as_str)
{
	int i;
	int retval;
	unsigned int tmp[MAC_ADDR_SIZE];
	int mac_len = strnlen(mac_addr_as_str, QCSAPI_MAX_ETHER_STRING + 1);

	if (mac_addr_as_str == NULL)
		return -qcsapi_invalid_mac_addr;

	if (mac_len > QCSAPI_MAX_ETHER_STRING)
		return -qcsapi_invalid_mac_addr;

	for (i = 0; i < mac_len; i++) {
		if (!(isxdigit(mac_addr_as_str[i]) || (mac_addr_as_str[i] == ':')))
			return -qcsapi_invalid_mac_addr;
	}

	retval = sscanf(mac_addr_as_str, "%x:%x:%x:%x:%x:%x",
			&tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]);
	if (retval != MAC_ADDR_SIZE)
		return -qcsapi_invalid_mac_addr;

	for (i = 0; i < MAC_ADDR_SIZE; i++) {
		if (tmp[i] > 0xff)
			return -qcsapi_invalid_mac_addr;
	}

	return 0;
}

int validate_mac_addr_unicast(qcsapi_mac_addr mac_addr)
{
	const char zero_mac[ETH_ALEN] = {0};

	if (memcmp(zero_mac, mac_addr, ETH_ALEN) == 0)
		return -1;
	else if ((mac_addr[0] & 0x1) == 0x1)
		return -1;
	else
		return 0;
}

int parse_mac_addr(const char *mac_addr_as_str, qcsapi_mac_addr mac_addr)
{
	if (validate_mac_addr(mac_addr_as_str) < 0)
		return -qcsapi_invalid_mac_addr;

	(void)sscanf(mac_addr_as_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			&mac_addr[0], &mac_addr[1], &mac_addr[2],
			&mac_addr[3], &mac_addr[4], &mac_addr[5]);

	return 0;
}

int local_isspace(char str)
{
	if ((str == ' ') || (str == '\t'))
		return 1;

	return 0;
}

int qcsapi_ascii_to_hexstr(const char *str, char *hex, int bufsize)
{
	int i;
	int length;

	length = strlen(str);

	if (bufsize < ((length * 2) + 1))
		return -1;

	for (i = 0; i < length; i++)
		snprintf(hex + i * 2, 3, "%X", str[i]);

	hex[length * 2] = '\0';

	return 0;
}

