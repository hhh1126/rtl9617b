/*
 * (C) Copyright 2017 - 2018 Quantenna Communications Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#ifndef _COVERITY_COMPAT_H_
#define _COVERITY_COMPAT_H_

/*
 * Hide defines in ctype.h from Coverity to avoid
 * false positive TAINTED_SCALAR defects.
 */
#ifdef __COVERITY__
#undef isupper
#undef islower
#undef isalpha
#undef isdigit
#undef isxdigit
#undef isspace
#undef ispunct
#undef isalnum
#undef isgraph
#undef isprint
#undef iscntrl
#undef toupper
#undef tolower
#endif

#endif
