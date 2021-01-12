/*
 * (C) Copyright 2010 Quantenna Communications Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Header file which describes Ruby platform.
 * Has to be used by both kernel and bootloader.
 */

#ifndef __RUBY_CONFIG_H
#define __RUBY_CONFIG_H

#ifndef __QTN_CONFIG_H_INCLUDED
#error "Include qtn_config.h instead"
#endif

#include "base/qtn_base_config.h"

/* Set to 1 if MuC need to enable TLB, otherwise set to 0 */
#define RUBY_MUC_TLB_ENABLE		1

/* Define some constants for Linux ARC kernel */
#define CONFIG_ARC700_SERIAL_BAUD	RUBY_SERIAL_BAUD
#define CONFIG_ARC700_DEV_CLK		RUBY_FIXED_DEV_CLK

#endif // #ifndef __RUBY_CONFIG_H
