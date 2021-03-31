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
 **/

#ifndef __TOPAZ_PCIE_TQE_H
#define __TOPAZ_PCIE_TQE_H

#include <qtn/tqe_cpuif.h>
#include <qtn/topaz_fwt_db.h>
#define TOPAZ_TQE_PCIE_REL_PORT TQE_PORT_AUC_2
#define pearl_tqe_pcieif_descr tqe_cpuif_descr

int topaz_pcie_tqe_xmit(fwt_db_entry * fwt_ent, void *data_bus, int data_len);
fwt_db_entry *vmac_get_tqe_ent(const unsigned char *src_mac_be,
			       const unsigned char *dst_mac_be,
			       const uint16_t vlan_id);
struct net_device *tqe_pcie_netdev_init(struct net_device *pcie_ndev);
void tqe_pcie_netdev_term(struct net_device *pcie_ndev);
#endif
