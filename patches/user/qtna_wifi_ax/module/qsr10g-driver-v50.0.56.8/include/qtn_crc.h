/*
 * (C) Copyright 2018 Quantenna Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __QTN_CRC_H
#define __QTN_CRC_H

static int qtn_crc_table_empty = 1;
#define QTN_CRC_TABLE_LEN	256
int32_t qtn_crc_table[QTN_CRC_TABLE_LEN];

/* This CRC algorithm must be identical to crc32.c in u-boot */
inline void qtn_crc_make_table(void)
{
	uint32_t c;
	int n;
	int k;
	uint32_t poly;
	static const uint8_t p[] = {0, 1, 2, 4, 5, 7, 8, 10, 11, 12, 16, 22, 23, 26};

	/* make exclusive-or pattern from polynomial (0xedb88320L) */
	poly = 0L;
	for (n = 0; n < sizeof(p) / sizeof(uint8_t); n++)
		poly |= 1L << (31 - p[n]);

	for (n = 0; n < QTN_CRC_TABLE_LEN; n++) {
		c = (uint32_t) n;
		for (k = 0; k < 8; k++)
			c = c & 1 ? poly ^ (c >> 1) : c >> 1;
		qtn_crc_table[n] = c;
	}
	qtn_crc_table_empty = 0;
}

inline uint32_t qtn_crc_32(uint32_t crc, const uint8_t *buf, uint32_t len)
{
	if (qtn_crc_table_empty)
		qtn_crc_make_table();

	crc = crc ^ 0xffffffffL;
	while (len > 0) {
		crc = qtn_crc_table[((int)crc ^ (*buf)) & 0xff] ^ (crc >> 8);
		buf++;
		len--;
	}

	return crc ^ 0xffffffffL;
}

#endif /* __QTN_CRC_H */
