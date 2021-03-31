/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2017 - 2018 Quantenna Communications Inc            **
**                                                                           **
**  File        : qcsapi_grabber.c                                           **
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
EH0*/

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <sys/time.h>
#include "qcsapi_rpc_common/common/qtn_endian.h"
#include "qcsapi_grabber.h"

static int local_qcsapi_grabber_get_hdr(uint8_t *buf, size_t buf_size, size_t *bytes_copied)
{
	struct qcsapi_grab_info_hdr hdr = {
					.magic = QCSAPI_GRAB_INFO_MAGIC,
					.version = htoqt32(QCSAPI_GRAB_INFO_VERSION),
					.header_len = htoqt32(sizeof(struct qcsapi_grab_info_hdr)),
					.hash_offset = 0,
					.creation_time = 0 };
	struct timeval tv;

	if (buf == NULL || buf_size < sizeof(hdr))
		return -EINVAL;

	if (gettimeofday(&tv, NULL) == 0)
		hdr.creation_time = htoqt64(tv.tv_sec);

	memcpy(buf, &hdr, sizeof(hdr));

	if (bytes_copied)
		*bytes_copied = sizeof(hdr);

	return 0;
}

static int local_qcsapi_grabber_get_param_content(uint32_t param_num, uint8_t *buf, size_t buf_size,
							size_t *bytes_copied)
{
	int ret = 0;
	ssize_t buf_used = 0;
	uint32_t bytes_grab;

	if (buf == NULL || buf_size < sizeof(struct qcsapi_data_1Kbytes))
		return -EINVAL;

	do {
		if (buf_used + sizeof(struct qcsapi_data_1Kbytes) > buf_size)
			break;

		bytes_grab = 0;
		ret = qcsapi_grab_config(param_num, buf_used, 0,
					(struct qcsapi_data_1Kbytes *)((uint8_t *)buf + buf_used),
					&bytes_grab);
		if (ret < 0)
			return ret;

		buf_used += bytes_grab;
	} while (bytes_grab == sizeof(struct qcsapi_data_1Kbytes));

	if (bytes_copied)
		*bytes_copied = buf_used;

	return ret;
}

static int local_qcsapi_grabber_get_param(uint32_t param_num, uint8_t *buf, size_t buf_size,
						size_t *bytes_copied)
{
	int ret = 0;
	size_t buf_used = 0;
	size_t content_len = 0;
	uint32_t hdr_len = 0;
	struct qcsapi_grab_info_cmn *hdr = (void *)buf;
	struct qcsapi_grab_info_cmn hdr_cpu;

	if (buf == NULL || buf_size < sizeof(struct qcsapi_data_1Kbytes))
		return -EINVAL;

	memset(buf, 0, buf_size);
	if (bytes_copied)
		*bytes_copied = 0;

	ret = qcsapi_grab_config(param_num, 0, QCSAPI_GRAB_ONLY_INFO_FL,
				(struct qcsapi_data_1Kbytes *)buf, &hdr_len);

	if (ret < 0)
		return ret;

	buf_used += hdr_len;
	if (bytes_copied)
		*bytes_copied = buf_used;

	if (buf_used < sizeof(*hdr))
		return -EFAULT;

	hdr_cpu.type = qt16toh(hdr->type);
	hdr_cpu.status = qt16toh(hdr->status);
	hdr_cpu.header_len = qt32toh(hdr->header_len);
	hdr_cpu.len = qt64toh(hdr->len);

	if (buf_used != hdr_cpu.header_len)
		return -EFAULT;

	if (hdr_cpu.status == QCSAPI_GRAB_INFO_STATUS_ERROR)
		return 0;

	ret = local_qcsapi_grabber_get_param_content(param_num, buf + buf_used, buf_size - buf_used,
							&content_len);
	if (ret < 0) {
		content_len = 0;
		hdr_cpu.status = QCSAPI_GRAB_INFO_STATUS_ERROR;
	}

	buf_used += content_len;
	if (bytes_copied)
		*bytes_copied = buf_used;

	/* Calculate real size if needed */
	if (hdr_cpu.len + hdr_cpu.header_len != buf_used) {
		hdr_cpu.len = buf_used - hdr_cpu.header_len;

		if (hdr_cpu.status == QCSAPI_GRAB_INFO_STATUS_OK &&
				(buf_used + sizeof(struct qcsapi_data_1Kbytes)) > buf_size) {
			hdr_cpu.status = QCSAPI_GRAB_INFO_STATUS_TRUNCATED;
		}
	}

	hdr->len = htoqt64(hdr_cpu.len);
	hdr->status = htoqt16(hdr_cpu.status);

	return 0;
}

int qcsapi_grabber_write_config_blob(FILE *s, uint32_t param_num, size_t *bytes_written)
{
	int ret = 0;
	int one_shot = 0;
	uint8_t *buf = NULL;
	const int max_chunks_to_grab = 256;
	size_t buf_size = max_chunks_to_grab * sizeof(struct qcsapi_data_1Kbytes);
	size_t bytes_copied = 0;

	if (s == NULL) {
		ret = -EINVAL;
		goto out;
	}

	buf = malloc(buf_size);
	if (buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = local_qcsapi_grabber_get_hdr(buf, buf_size, &bytes_copied);
	if (ret < 0)
		goto out;

	if (fwrite(buf, 1, bytes_copied, s) != bytes_copied) {
		ret = -EFAULT;
		goto out;
	}

	if (bytes_written)
		*bytes_written += bytes_copied;

	if (param_num == QCSAPI_GRABBER_PARAM_ALL) {
		one_shot = 0;
		param_num = 0;
	} else {
		one_shot = 1;
	}

	do {
		ret = local_qcsapi_grabber_get_param(param_num, buf, buf_size, &bytes_copied);
		if (ret < 0)
			break;

		if (fwrite(buf, 1, bytes_copied, s) != bytes_copied) {
			ret = -EFAULT;
			goto out;
		}

		if (bytes_written)
			*bytes_written += bytes_copied;
		param_num++;
	} while(!one_shot);

	/* Last parameter reached */
	if (ret == -qcsapi_param_value_invalid && !one_shot)
		ret = 0;

out:
	if (buf)
		free(buf);

	return ret;
}
