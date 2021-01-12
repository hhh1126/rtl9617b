/**
 * Copyright (c) 2015 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

/*
 * Platform dependant implement. Customer needs to modify this file.
 */
#include <linux/interrupt.h>
#include <qdpc_platform.h>
#include <pearl_vnet.h>
#include <linux/netdevice.h>
#include <linux/pci.h>

/*
 * Enable interrupt for detecting EP reset.
 */
void enable_ep_rst_detection(struct net_device *ndev)
{
}

/*
 * Disable interrupt for detecting EP reset.
 */
void disable_ep_rst_detection(struct net_device *ndev)
{
}

/*
 * Interrupt context for detecting EP reset.
 * This function should do:
 *   1. check interrupt status to see if EP reset.
 *   2. if EP reset, handle it.
 */
void handle_ep_rst_int(struct net_device *ndev)
{
}
