/*
 * Copyright (c) Cortina-Access Limited 2015.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 * ca_ni_rx.c: Cortina NI Rx handler
 *
 */

#define pr_fmt(fmt)     KBUILD_MODNAME ":%s:%d: " fmt, __func__, __LINE__

#include <generated/ca_ne_autoconf.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_arp.h>
#include <linux/igmp.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/addrconf.h>

#include "osal_cmn.h"

#include "aal_common.h"
#include "aal_port.h"
#include "aal_ni.h"
#include "aal_l3qm.h"
#include "aal_fbm.h"
#include "aal_fdb.h"
#include "aal_l3_pe.h"

#include "ca_ni_rx.h"
#include "ca_virt_ni.h"
#include "ca_ni.h"
#include "ca_ni_proc.h"
#include "ca_ni_dump.h"

#include "l2.h"
#include "rx.h"
#include "vlan.h"
#include "port.h"
#include "ptp.h"

#include "scfg.h"

#include "ca_mut.h"

#ifdef CONFIG_NE_L2FP
#include "ca_l2fp.h"
#endif

#include "aal_pon.h"

#ifdef CONFIG_RTK_PERF_EVENTS
#include <soc/cortina/rtk_pmu.h>
#endif

#if defined(CONFIG_RTL_FPGA_PHY_TEST)
extern ca_ni_rx_tx_test_t ni_rx_tx_test;
#endif

#define CA_NI_VIRTUAL_DEV		0
#define LINUX_FREE_BUF			8

/* mcgid bit 0-2 for software learning */
#define CA_NI_SW_LEARN_NEW		0x1
#define CA_NI_SW_LEARN_HASH_COLLISION	0x2
#define CA_NI_SW_LEARN_EXCEED_LIMIT	0x4

extern ca_uint32_t ca_pon_debug;
extern void  ca_buffer_dump( char *tp, ca_uint8_t *dp, ca_uint16_t len);
int qm_acp_enabled = 0;

ca_status_t ca_ni_netlink_send_packet(int cpu_port, HEADER_A_T *hdr_a, ca_uint8_t *data, ca_uint32_t pkt_size, HEADER_CPU_T *hdr_cpu, struct sk_buff *skb);

#define CA_NI_RX_MIN_REFILL_COUNT 8
#define CA_NI_RX_MAX_REFILL_COUNT 16
static refill_pyh_addr_info_t refill_phy_addr_pool0[CA_NI_RX_MAX_REFILL_COUNT];
static refill_pyh_addr_info_t refill_phy_addr_pool1[CA_NI_RX_MAX_REFILL_COUNT];
static int rx_refill_count_pool0=0;
static int rx_refill_count_pool1=0;
static int ca_ni_rx_get_bad_packet=0;

static ca_uint8_t l3qm_cpu_pool0_eq_id = 8;
static ca_uint8_t l3qm_cpu_pool1_eq_id = 9;

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
#define EPP_DESC_SIZE 4
//extern special_ff_priv_t special_ff_port;
extern rtk_scfg_t rtkScfg;
int up_stream_inner_extra_len = 0;
int down_stream_inner_extra_len = 0;
//atomic_t perCPUPort_fastFwd_statistic_ingress[8][2];
//atomic_t perCPUPort_fastFwd_statistic_egress[8][2];


#endif

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_RTL_MULTI_LAN_DEV)
struct net_device* ca_ni_decide_rx_device(ca_eth_private_t *cep, unsigned int phy_src_port){
#if !defined(CONFIG_RTK_PE_FC_SUPPORT)
	unsigned int num = (phy_src_port >= CA_MAX_ETHERNET_PORT_NUMBER) ? 
		(0) : phy_src_port ;
#else
	unsigned int num = (phy_src_port >= CA_MAX_ETHERNET_PORT_NUMBER) ?
		((phy_src_port == PE_PORT) ? phy_src_port : 0) : phy_src_port ;
#endif

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: DEV=%s\n", __func__, cep->port2dev[num]->name);
	}
	
	if(unlikely((cep->cpu_port < (AAL_LPORT_CPU_2- AAL_LPORT_CPU_0)) && (cep->port2dev[num] == NULL))) {
		if (printk_ratelimit())
			printk("%s: device port mapping error - lspid = %d force rx dev = %s\n", __func__, phy_src_port, cep->dev);
		return cep->dev;
	}
	
	return cep->port2dev[num];
}
#endif

#if defined(CONFIG_LUNA_G3_SERIES)
#if defined(CONFIG_RG_G3_L2FE_POL_OFFLOAD)
void ca_ni_decide_rx_lspib_by_fakeVlan(struct sk_buff *skb, unsigned int *phy_src_port, HEADER_A_T *hdr_a)
{
	ca_uint16_t fake_vlan_vid = -1;
	struct vlan_hdr *fake_vlan_hdr = NULL;

	//if packet ingress from LAN port (upstream), remove fake vlan
	if(((u16)skb->data[12]) == htons(0x8100))
	{
		fake_vlan_hdr = (struct vlan_hdr *)(&skb->data[14]);
		fake_vlan_vid = ntohs(fake_vlan_hdr->h_vlan_TCI)&VLAN_VID_MASK;
		if((fake_vlan_vid < G3_LOOPBACK_UPSTREAM_VID_MIN) || (fake_vlan_vid > G3_LOOPBACK_UPSTREAM_VID_MAX))
		{
			//not upstream fake vlan, skip
		}
		else
		{
			//upstream fake vlan, remove it
			ca_uint8_t *pData = skb->data;
			memmove(pData+VLAN_HLEN, pData, (ETH_HLEN - 2));
			skb->data += VLAN_HLEN;
			skb->len -= VLAN_HLEN;
			skb_reset_mac_header(skb);
			hdr_a->bits.lspid = G3_LOOPBACK_UPSTREAM_LSPID(fake_vlan_vid);
			*phy_src_port = PORT_ID(hdr_a->bits.lspid);
		}	
	}
	return ;
}

#endif
#endif


ca_uint8_t pon_mac_mode = 0;
#if defined(CONFIG_NE_PON)
extern ca_status_t aal_pdc_lspid_gem_index_get(
    CA_IN ca_device_id_t device_id,
    CA_IN ca_uint32_t    lspid,
    CA_OUT ca_uint32_t   *gem_index
);
#endif

extern ca_uint8_t netlink_rx_ack;
extern int netlink_rx_queue_is_full;
extern int netlink_rx_free_skb_count;

extern ca_status_t ca_pon_exit(ca_device_id_t device_id);

static ca_boolean ca_ni_rx_check_hw_support_l4_csum(uint8_t *data, uint8_t *data_end)
{
        unsigned char *cp = data;
        uint16_t ethertype = 0;
        uint8_t ip_prot = 0xff;

	ethertype = ca_ni_parse_l2(cp, NULL, NULL, NULL, &cp, false);

        if (ntohs(ethertype) == ETH_P_IP) {
                if (cp + sizeof(struct iphdr) <= data_end)
			ip_prot = ca_ni_parse_ipv4(cp, &cp, false, NULL);
        }

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: ip_prot=%d\n", __func__, ip_prot);
	}
        switch (ip_prot) {
                case IPPROTO_TCP:
                case IPPROTO_UDP:
                case IPPROTO_UDPLITE:
                case 27: /* RDPv1, RDPv2 */
                        return CA_TRUE;
                        break;
                default:
                        return CA_FALSE;
        }
}

ca_boolean_t ca_ni_rx_check_ptp(uint8_t *data, uint8_t *data_end, ca_boolean_t dump)
{
        unsigned char *cp = data;
        uint16_t ethertype = 0;
        uint8_t ip_prot = 0xff;
        uint16_t l4_sport = 0, l4_dport = 0;

	ethertype = ca_ni_parse_l2(cp, NULL, NULL, NULL, &cp, dump);

        if (dump) {
                printk("%s: ethertype=0x%x, ETH_P_1588=0x%x\n", __func__, ntohs(ethertype), ETH_P_1588);
        }

        if (ntohs(ethertype) == ETH_P_IP) {
                if (dump) {
                        printk("%s: cp=0x%p, data_end=0x%p, sizeof(struct iphdr)=%ld\n",
                                __func__, cp, data_end, sizeof(struct iphdr));
                }
                if (cp + sizeof(struct iphdr) <= data_end) {
			ip_prot = ca_ni_parse_ipv4(cp, &cp, dump, NULL);
                }
        }
        else if (ntohs(ethertype) == ETH_P_IPV6) {
                if (cp + sizeof(struct ipv6hdr) <= data_end)
			ip_prot = ca_ni_parse_ipv6(cp, &cp, dump, NULL);
        }
        else if (ntohs(ethertype) == ETH_P_1588) {
                if (dump) {
                        printk("%s: It's a L2 PTP packet!!\n", __func__);
                }
                return CA_TRUE;
        }

        if (dump) {
                printk("%s: ip_prot=%d\n", __func__, ip_prot);
        }

        if (ip_prot == IPPROTO_UDP) {
                if (cp + sizeof(struct udphdr) <= data_end) {
			ca_ni_parse_udp(cp, &cp, &l4_sport, &l4_dport, dump);
                        if (l4_dport == 319 || l4_dport == 320) {
                                if (dump) {
                                        printk("%s: It's a L4 PTP packet!!\n", __func__);
                                }
                                return CA_TRUE;
                        }
                }
        }
        return CA_FALSE;
}

void ca_ni_rx_append_ptp_data(char *data, ca_uint32_t *pkt_size, ca_uint64_t nsec, ca_boolean_t dump)
{
        struct timeval tv;
        ca_ptp_local_clock_t local_clock;

        do_gettimeofday(&tv);
        local_clock.sec = tv.tv_sec;
        local_clock.nsec = nsec;

        if (dump) {
                printk("%s: Got a L2 or L4 PTP packet!!\n", __func__);
                printk("%s: local_clock.sec=%lld, local_clock.nsec=%d\n", __func__, local_clock.sec, local_clock.nsec);
        }

        /* there are only 30 bits in Header CPU to carry the nano-second part */
        memcpy(data + *pkt_size, &local_clock, sizeof(ca_ptp_local_clock_t));
        *pkt_size += sizeof(ca_ptp_local_clock_t);

        /* need to re-calculate the UDP checksum ? */
}

void ca_ni_rx_fill_pkt(int cpu_port, ca_pkt_t *pkt, HEADER_A_T *hdr_a, HEADER_CPU_T *hdr_cpu)
{
	ca_uint32_t     port_type;
#if defined(CONFIG_NE_PON)
	ca_uint32_t	gem_index;
	ca_uint32_t	llid;
#endif

	/* fill sw_id[4] with header CPU */
	pkt->sw_id[0] = hdr_cpu->bits64;
	pkt->sw_id[1] = hdr_cpu->bits64 >> 8;
	pkt->sw_id[2] = hdr_cpu->bits64 >> 16;
	pkt->sw_id[3] = hdr_cpu->bits64 >> 24;

	/* WAN port should add gem index */
	if (cpu_port == 0) {
#if defined(CONFIG_NE_PON)
		if (CA_PON_MODE_IS_EPON_FAMILY(pon_mac_mode)) {
			port_type = CA_PORT_TYPE_EPON;
			llid = pkt->pkt_data->data[11] & 0x3f;	/* llid is located at MAC SA[5] */
			pkt->src_sub_port_id = CA_PORT_ID(CA_PORT_TYPE_SUBPORT, llid);
		}
		else if (CA_PON_MODE_IS_GPON_FAMILY(pon_mac_mode)) {
			port_type = CA_PORT_TYPE_GPON;
			if (aal_pdc_lspid_gem_index_get(0, hdr_a->bits.lspid, &gem_index) != CA_E_OK) {
				gem_index = CA_UINT32_INVALID;
			}
			pkt->src_sub_port_id = CA_PORT_ID(CA_PORT_TYPE_SUBPORT, gem_index);
		}
		else {
			port_type = CA_PORT_TYPE_ETHERNET;
		}
#else
		port_type = CA_PORT_TYPE_ETHERNET;
#endif

		pkt->src_port_id = CA_PORT_ID(port_type, wan_port_id);
		pkt->dst_port_id = CA_PORT_ID(CA_PORT_TYPE_CPU, hdr_a->bits.ldpid);
	}
	else {
		pkt->src_port_id = CA_PORT_ID(CA_PORT_TYPE_ETHERNET, hdr_a->bits.lspid);
		pkt->dst_port_id = CA_PORT_ID(CA_PORT_TYPE_CPU, hdr_a->bits.ldpid);
	}

	/* if packet type is OAM/OMCI should remove the header */
	if (pkt->pkt_type == CA_PKT_TYPE_OMCI || pkt->pkt_type == CA_PKT_TYPE_OAM) {
		pkt->pkt_data->data += 16;
		pkt->pkt_data->len -= 16;
		pkt->pkt_len -= 16;
	}
}

static ca_status_t ca_ni_rx_register_callback(int cpu_port, HEADER_A_T *hdr_a, ca_uint8_t *data, ca_uint32_t pkt_size, HEADER_CPU_T *hdr_cpu)
{
	ca_pkt_t        pkt;
	ca_pkt_block_t  pkt_blk;
	ca_uint8_t	*tmp_buf;
	ca_pkt_type_t 	pkt_type;

	/* check whether the pkt type has been registered */
	if (__ca_rx_register_callback_check_pkt_type(data, &pkt_type) != CA_E_OK) {
		return CA_E_ERROR;
	}

	tmp_buf = kmalloc(CA_NI_SKB_PKT_LEN, GFP_KERNEL);
	if (tmp_buf == NULL) {
		printk("%s: No memory for packet data!!\n", __func__);
		return CA_E_ERROR;
	}
	memset(&pkt, 0, sizeof(ca_pkt_t));
	memset(&pkt_blk, 0, sizeof(ca_pkt_block_t));

	if (pkt_size > CA_NI_SKB_PKT_LEN)
		pkt_size = CA_NI_SKB_PKT_LEN;

	memcpy(tmp_buf, data, pkt_size);

	pkt_blk.len = pkt_size;
	pkt.pkt_data = &pkt_blk;
	pkt.block_count = 1;
	pkt.device_id = 0;
	pkt.pkt_len = pkt_size;
	pkt_blk.data = tmp_buf;
	pkt.pkt_type = pkt_type;

	/* fill other fields of ca_pkt_t */
	ca_ni_rx_fill_pkt(cpu_port, &pkt, hdr_a, hdr_cpu);

	/* call the called back routine registered for rx */
	__ca_rx_register_callback(&pkt);

	kfree(tmp_buf);

	return CA_E_OK;
}

static void ca_ni_rx_get_startup_config(void)
{
	static int init_done = 0;

	if (init_done == 0) {
		aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_EQ_ID, 1, &l3qm_cpu_pool0_eq_id);
		aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_EQ_ID, 1, &l3qm_cpu_pool1_eq_id);
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: read from starup config, l3qm_cpu_pool0_eq_id=%d\n", __func__, l3qm_cpu_pool0_eq_id);
			printk("%s: read from starup config, l3qm_cpu_pool1_eq_id=%d\n", __func__, l3qm_cpu_pool1_eq_id);
		}

#if defined(CONFIG_NE_PON)
		aal_scfg_read(CFG_ID_PON_MAC_MODE, 1, &pon_mac_mode);
#endif
		init_done = 1;
	}
}
#if defined(CONFIG_LUNA_G3_SERIES) && (defined(CONFIG_RTK_NIC_RX_HOOK) || defined(CONFIG_RTK_NIC_TX_HOOK))
void ca_ni_l2_addr_add(ca_uint16_t vid, ca_mac_addr_t *sa_mac, int port_id, ca_uint8_t lrn_info)
#else
static void ca_ni_l2_addr_add(ca_uint16_t vid, ca_mac_addr_t *sa_mac, int port_id, ca_uint8_t lrn_info)
#endif
{
	aal_fdb_entry_cfg_t  l2_addr;
	ca_port_id_t	     phy_port;
	ca_vlan_port_control_t	vlan_port_control;

	memset(&vlan_port_control, 0, sizeof(ca_vlan_port_control_t));
	phy_port = CA_PORT_ID(CA_PORT_TYPE_ETHERNET, port_id);

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: phy_port=0x%x\n", __func__, phy_port);
	}
	ca_l2_vlan_port_control_get(0, phy_port, &vlan_port_control);
	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: vlan_port_control.outer_tpid=0x%x\n", __func__, vlan_port_control.outer_tpid);
	}

	memset(&l2_addr, 0, sizeof(ca_l2_addr_entry_t));
	memcpy(l2_addr.mac, sa_mac, sizeof(ca_mac_addr_t));
	l2_addr.sa_permit = 1;
	l2_addr.da_permit = 1;
	l2_addr.port_id = port_id;
	l2_addr.aging_sts = 255;
	l2_addr.key_vid = vid;
	l2_addr.key_sc_ind = vlan_port_control.outer_tpid;
	l2_addr.lrnt_vid = vid;
	l2_addr.lrnt_sc_ind = vlan_port_control.outer_tpid;

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: vid=%d, port_id=%d, sa_mca=%pM, lrn_info=%d\n", __func__, vid, port_id, sa_mac, lrn_info);
	}

	switch (lrn_info) {
		case CA_NI_SW_LEARN_NEW:
			aal_fdb_sw_learning(0, &l2_addr);
			break;
		case CA_NI_SW_LEARN_HASH_COLLISION:
			aal_fdb_hash_collision(0, &l2_addr);
			break;
		case CA_NI_SW_LEARN_EXCEED_LIMIT:
			aal_fdb_mac_limit_exceed(0, port_id, &l2_addr);
			break;
	}
}

static void ca_ni_xmit_first_uuc(int lso_txq_index, ca_uint16_t ldpid, struct sk_buff *skb, struct napi_struct *napi)
{
	int length;
	char *src_data, *dst_data;
	struct sk_buff *clone_skb;
	struct net_device *dev = napi->dev;
	ca_ni_tx_config_t tx_config;

	/* for jumbo packets */
	struct sk_buff *frag_skb = skb_shinfo(skb)->frag_list;

	memset(&tx_config, 0, sizeof(tx_config));
	tx_config.core_config.bf.txq_index = lso_txq_index;
	tx_config.core_config.bf.ldpid = ldpid;

	/* tell DMA LSO donot do segmentation, send out the origianl packet */
	tx_config.bypass_lso = 1;

	if (frag_skb == 0) {
		/* should clone skb since __ca_ni_start_xmit() will try to free skb after transmit */
		clone_skb = ca_skb_clone(skb, GFP_ATOMIC, true);
		clone_skb->data = skb_mac_header(clone_skb);            /* reset skb->data to MAC header */

		skb_put(clone_skb, ETH_HLEN);   /* skb->tail += ETH_HLEN, skb->len += ETH_HLEN */
		__ca_ni_start_xmit(clone_skb, dev, &tx_config);
	}
	else {
		/* if there is more than 1 fragment in this skb allocate a skb with size skb->len + ETH_HLEN */
		clone_skb = ca_netdev_alloc_skb(dev, skb->len + ETH_HLEN, true);
		if (!clone_skb) {
			printk(KERN_ERR "%s: Not enough memory to allocate skb of size %d.\n", dev->name, length);
			return;
		}

		/* set skb->network_header=skb->head+ETH_HLEN for lso segment size calculation */
		skb_set_network_header(clone_skb, ETH_HLEN);

		clone_skb->len = skb->len + ETH_HLEN;

		/* copy first fragment of packet */
		length = skb->len + ETH_HLEN - skb->data_len;
		src_data = skb->data - ETH_HLEN;
		dst_data = clone_skb->data;
		memcpy(dst_data, src_data, length);
		dst_data += length;

		/* copy reset of fragment of packet */
		while (frag_skb) {

			memcpy(dst_data, frag_skb->data, frag_skb->len);
			dst_data += frag_skb->len;

			frag_skb = frag_skb->next;
		}
		__ca_ni_start_xmit(clone_skb, dev, &tx_config);
	}
}
#if defined(CONFIG_LUNA_G3_SERIES) && (defined(CONFIG_RTK_NIC_RX_HOOK) || defined(CONFIG_RTK_NIC_TX_HOOK))
void ca_ni_l2_flooding(ca_uint16_t vid, ca_mac_addr_t da_mac, struct sk_buff *skb, struct napi_struct *napi, HEADER_A_T *hdr_a)
#else
static void ca_ni_l2_flooding(ca_uint16_t vid, ca_mac_addr_t da_mac, struct sk_buff *skb, struct napi_struct *napi, HEADER_A_T *hdr_a)
#endif
{
	int i;
	ca_uint16_t cpu_port;
	ca_status_t ret;
	ca_uint8_t port_type;
	ca_uint8_t member_count;
	ca_flooding_port_t members[64];
	int lso_txq_index;
	ca_uint16_t ldpid;
	aal_fdb_entry_cfg_t fdb_entry;
	ca_port_cfg_info_t *port_cfg;
	ca_vlan_port_control_t vlan_port_control;
	ca_port_id_t port_id;

	/* skip Multicast and Broadcast packets */
	if (da_mac[0] & (ca_uint8_t)0x1) return;

	/* according from System Arch Doc. we regard untag as vid=0 in stead of 0xffff here */
	if (vid == 0xffff) vid = 0;

#ifdef CONFIG_NET_SCH_MULTIQ
	lso_txq_index = skb_get_queue_mapping(skb);
#else
	lso_txq_index = skb->priority;
#endif

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: vid=%d, DA_MAC=%pM, cpu_port=%d, lspid=%d\n", __func__, vid, da_mac, cpu_port, hdr_a->bits.lspid);
		printk("%s: rx_pkt_type=%d\n", __func__, hdr_a->bits.bits_32_63.pkt_info.rx_pkt_type);
	}

	/* mcgid stores the learning information */
	/* Bit[7:5]: 011b
	   Bit[4:3]: reserved
	   Bit[2]: lrn_lmt_excd_flg
	   Bit[1]: lrn_clsn_flg
	   Bit[0]: sw_new_lrn_flg
	*/
	/* In first UUC Bit[7:5]=3 and Bit[0]=1 */
	if ((hdr_a->bits.bits_32_63.pkt_info.mcgid & 0xe0) != 0x60) return;
	if ((hdr_a->bits.bits_32_63.pkt_info.mcgid & 0x1) == 0) return;

        if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: First UUC has received!!\n", __func__);
	}


	/* if the DA MAC has been learned then send to that port */
	if (aal_fdb_entry_get(0, da_mac, vid, &fdb_entry) == CA_E_OK) {

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: Got this DA MAC in FDB\n", __func__);
		}
		if (fdb_entry.valid) {
			ldpid = PORT_ID(fdb_entry.port_id);

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: i=%d, lso_txq_index=%d, ldpid=%d\n", __func__, i, lso_txq_index, ldpid);
			}
			ca_ni_xmit_first_uuc(lso_txq_index, ldpid, skb, napi);
		}
		return;
	}

	/* determine port_type from lspid because cpu_port is not correct */
	port_type = CA_L2_ING_PORT_TYPE_WAN;
	port_cfg = &port_cfg_infos[0];
	for (i = 0; i < port_cfg->num_phy_port; i++) {
		if (hdr_a->bits.lspid == port_cfg->port_id[i]) {
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: lspid=%d is WAN port\n", __func__, hdr_a->bits.lspid);
			}
			cpu_port = CA_NI_CPU_PORT0_LPID;
			break;
		}
	}

	if (i == port_cfg->num_phy_port) { /* not found in WAN port */
		port_type = CA_L2_ING_PORT_TYPE_LAN;
		port_cfg = &port_cfg_infos[1];
		for (i = 0; i < port_cfg->num_phy_port; i++) {
			if (hdr_a->bits.lspid == port_cfg->port_id[i]) {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: lspid=%d is LAN port\n", __func__, hdr_a->bits.lspid);
				}
				cpu_port = CA_NI_CPU_PORT1_LPID;
				break;
			}
		}
	}

	if (i == port_cfg->num_phy_port) {
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: can not determine lspid=%d is WAN port or LAN port!!\n", __func__, hdr_a->bits.lspid);
		}
		return;
	}

	ret = ca_l2_vlan_flooding_domain_members_get(0, vid, port_type, &member_count, members);
	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: ca_l2_vlan_flooding_domain_members_get(vid=%d, port_type=%d), ret=%d\n", __func__, vid, port_type, ret);
	}
	if (ret == CA_E_OK) {
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: ca_l2_vlan_flooding_domain_members_get() return OK\n", __func__);
		}
		for (i = 0; i < member_count; i++) {
			ldpid = PORT_ID(members[i].egress_port);
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: i=%d, lso_txq_index=%d, ldpid=%d\n", __func__, i, lso_txq_index, ldpid);
			}
			ca_ni_xmit_first_uuc(lso_txq_index, ldpid, skb, napi);
		}
	}
	else if (ret == CA_E_NOT_FOUND) {   /* go through Ethernet ports and CPU ports */

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: go through Ethernet ports and CPU ports\n", __func__);
		}
		for (i = 0; i < CA_MAX_ETHERNET_PORT_NUMBER; i++) {

			/* skip the source port of packet comes in */
			if (i == hdr_a->bits.lspid) continue;

			port_id = CA_PORT_ID(CA_PORT_TYPE_ETHERNET, i);
			if (ca_l2_vlan_port_control_get(0, port_id, &vlan_port_control) == CA_E_OK) {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: vlan_port_control.drop_unknown_vlan=%d\n", __func__, vlan_port_control.drop_unknown_vlan);
				}
				if (vlan_port_control.drop_unknown_vlan == CA_FALSE) {
					ldpid = PORT_ID(i);
					if (ca_ni_debug & NI_DBG_DUMP_RX) {
						printk("%s: i=%d, lso_txq_index=%d, ldpid=%d\n", __func__, i, lso_txq_index, ldpid);
					}
					ca_ni_xmit_first_uuc(lso_txq_index, ldpid, skb, napi);
				}
			}
		}
		for (i = AAL_LPORT_CPU_0; i <= AAL_LPORT_CPU_7; i++) {

			/* CPU port 2 has special use and skip the cpu_port which packet comes from */
			if (i == cpu_port || i == AAL_LPORT_CPU_2) continue;

			port_id = CA_PORT_ID(CA_PORT_TYPE_CPU, i);

			if (ca_l2_vlan_port_control_get(0, port_id, &vlan_port_control) == CA_E_OK) {
				if (vlan_port_control.drop_unknown_vlan == CA_FALSE) {
					ldpid = PORT_ID(i);
					if (ca_ni_debug & NI_DBG_DUMP_RX) {
						printk("%s: i=%d, lso_txq_index=%d, ldpid=%d\n", __func__, i, lso_txq_index, ldpid);
					}
					ca_ni_xmit_first_uuc(lso_txq_index, ldpid, skb, napi);
				}
			}
		}
	}
}

#define CA_NI_IRQ_PE  3

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_RTK_NIC_RX_HOOK)
static inline int get_dev_index_cpu_hdr(HEADER_A_T *hdr_a)
{
	int i;

	/* use ldpid to return the related net_device */
	if(hdr_a->bits.ldpid >= CA_PORT_ID_CPU0 && hdr_a->bits.ldpid <= CA_PORT_ID_CPU7)
		return hdr_a->bits.ldpid-CA_PORT_ID_CPU0;

	printk("%s: HEADER_A ldpid=%d is not valid!!!\n", __func__, hdr_a->bits.ldpid);
	return 0;
}
#endif

static inline struct net_device *get_dev_by_cpu_hdr(HEADER_A_T *hdr_a, struct net_device *orig_dev)
{
	int i;
	struct net_device *tmp_dev;
	ca_eth_private_t *cep;

	/* use ldpid to return the related net_device */
	for (i = 0; i < ca_ni_num_intf; i++) {
		tmp_dev = ni_info_data.dev[i];
		cep = netdev_priv(tmp_dev);

		/* at receive side check the logical destination port id */
		if (hdr_a->bits.ldpid == cep->port_cfg.rx_ldpid)
			return tmp_dev;
	}
	printk("%s: HEADER_A lspid=%d is not valid!!!\n", __func__, hdr_a->bits.lspid);
	return ni_info_data.dev[0];
}

static int ca_ni_rx_napi_get_first_segment_length(HEADER_A_T *hdr_a)
{
	int seg_len;
	int head_room = aal_l3qm_get_cpu_port_head_room_first();

	seg_len = aal_l3qm_eq_get_cpu_pool0_buf_size() - head_room - sizeof(HEADER_A_T);
	if (hdr_a->bits.bits_32_63.pkt_info.cpu_flg != 0)
		seg_len -= sizeof(HEADER_CPU_T);

        /* Header PTP is included in Header A */
	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: head_room=%d, seg_len=%d\n", __func__, head_room, seg_len);
	}

	return seg_len;
}

static void ca_ni_rx_napi_refill_empty_buffer(void)
{
	/* if FBM is enabled, recycle the buffer to FBM */
	if (aal_l3qm_eq_get_cpu_use_fbm()) {
		int i;
		ca_uint_t virt_start_addr;
		u32 entry_size, entry_count;
		u8 fbm_pool_id;
		u32 rx_dma_addr;
		u16 eqid;
		u8 cpu_pool0_eq_id, cpu_pool1_eq_id;

		aal_l3qm_eq_get_cpu_pool0_1_eq_id(&cpu_pool0_eq_id, &cpu_pool1_eq_id);

		if (rx_refill_count_pool0 == CA_NI_RX_MAX_REFILL_COUNT) {
			for (i = 0; i < rx_refill_count_pool0; i++) {
				aal_l3qm_eq_get_cpu_pool0_info(&virt_start_addr, &entry_size, &entry_count, &fbm_pool_id);

				rx_dma_addr = refill_phy_addr_pool0[i].phy_addr;
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: refill FBM pool, eqid=%d, fbm_pool_id=%d, rx_dma_addr=0x%x\n", __func__, eqid, fbm_pool_id, rx_dma_addr);
				}
				aal_fbm_buf_push(FBM_CPU_ARM0, fbm_pool_id, rx_dma_addr);
			}
			rx_refill_count_pool0 = 0;
		}
		if (rx_refill_count_pool1 == CA_NI_RX_MAX_REFILL_COUNT) {
			for (i = 0; i < rx_refill_count_pool1; i++) {
				aal_l3qm_eq_get_cpu_pool1_info(&virt_start_addr, &entry_size, &entry_count, &fbm_pool_id);

				rx_dma_addr = refill_phy_addr_pool1[i].phy_addr;
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: refill FBM pool, eqid=%d, fbm_pool_id=%d, rx_dma_addr=0x%x\n", __func__, eqid, fbm_pool_id, rx_dma_addr);
				}
				aal_fbm_buf_push(FBM_CPU_ARM0, fbm_pool_id, rx_dma_addr);
			}
			rx_refill_count_pool1 = 0;
		}
	}
	else if (aal_l3qm_eq_get_cpu_rule()) {
		int i;
		u16 inactive_bid_cntr;
		u16 eqid;
		u32 rx_dma_addr;

		if (rx_refill_count_pool0 == CA_NI_RX_MAX_REFILL_COUNT) {
			for (i = 0; i < rx_refill_count_pool0; i++) {
				eqid = refill_phy_addr_pool0[i].eqid;
				rx_dma_addr = refill_phy_addr_pool0[i].phy_addr;
				inactive_bid_cntr = aal_l3qm_get_inactive_bid_cntr(eqid);
				if (inactive_bid_cntr > 0) {
					/* check ready bit */
					if (aal_l3qm_check_cpu_push_ready(0)) {
						if (ca_ni_debug & NI_DBG_DUMP_RX) {
							printk("%s: refill QM pool0/1 buffer, eqid=%d, rx_dma_addr=0x%x\n", __func__, eqid, rx_dma_addr);
						}
						aal_l3qm_set_cpu_push_paddr(0, eqid, rx_dma_addr);
					}
				}
			}
			rx_refill_count_pool0 = 0;
		}

		if (rx_refill_count_pool1 == CA_NI_RX_MAX_REFILL_COUNT) {
			for (i = 0; i < rx_refill_count_pool1; i++) {
				eqid = refill_phy_addr_pool1[i].eqid;
				rx_dma_addr = refill_phy_addr_pool1[i].phy_addr;
				inactive_bid_cntr = aal_l3qm_get_inactive_bid_cntr(eqid);
				if (inactive_bid_cntr > 0) {
					/* check ready bit */
					if (aal_l3qm_check_cpu_push_ready(0)) {
						if (ca_ni_debug & NI_DBG_DUMP_RX) {
							printk("%s: refill QM pool0/1 buffer, eqid=%d, rx_dma_addr=0x%x\n", __func__, eqid, rx_dma_addr);
						}
						aal_l3qm_set_cpu_push_paddr(0, eqid, rx_dma_addr);
					}
				}
			}
			rx_refill_count_pool1 = 0;
		}
	}
}

static void ca_ni_rx_napi_read_epp(int cpu_port, u8 voq, struct napi_struct *napi, u8 **ret_rx_virt_addr, CPU_EPP_FIFO_CMD_T *ret_epp_fifo_cmd,
		HEADER_A_T *ret_hdr_a, HEADER_CPU_T *ret_hdr_cpu, HEADER_PTP_T *ret_hdr_ptp, u32 *hw_rd_ptr)
{
	ca_eth_private_t *cep = netdev_priv(napi->dev);
	struct platform_device *pdev = cep->pdev;
	int pkt_len, seg_len;
	u8 *rx_virt_addr;
	u16 epp_desc_size;
	int head_room = aal_l3qm_get_cpu_port_head_room_first();
	u8 cpu_pool0_eq_id, cpu_pool1_eq_id;

	dma_addr_t rx_dma_addr;

	CPU_EPP_FIFO_CMD_T epp_fifo_cmd;
	HEADER_A_T hdr_a;
	HEADER_CPU_T hdr_cpu;
	HEADER_PTP_T hdr_ptp;

	if (napi != NULL) {
		cep = netdev_priv(napi->dev);
	}
	else {
		cep = netdev_priv(ni_info_data.dev[0]);
	}
	pdev = cep->pdev;

	epp_desc_size = aal_l3qm_eq_get_desc_per_epp() * aal_l3qm_eq_get_desc_size();

	/* construct start pointer of CPU EPP FIFO command block */
	/* bit[31:7] - physical address */
	/* bit[6] - EOP */
	/* bit[5] - Layer 4 checksum error */
	/* bit[4] - SOP */
	/* bit[3-0] - EQID */

	rx_virt_addr = phys_to_virt(aal_l3qm_get_rx_start_addr(cpu_port, voq));

	/* should do cache invalidate if EPP not locates at non-cache */
	if (aal_l3qm_get_ace_test() == 0) {
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: call dma_sync_single_for_cpu()  for CPU EPP FIFO!!!\n", __func__);
		}
		dma_sync_single_for_cpu(&(pdev->dev), virt_to_phys(rx_virt_addr + *hw_rd_ptr), sizeof(CPU_EPP_FIFO_CMD_T), DMA_FROM_DEVICE);
	}

	epp_fifo_cmd = *(CPU_EPP_FIFO_CMD_T *)(rx_virt_addr + *hw_rd_ptr);

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: phy_addr=0x%x, eop=%d, l4_csum_err=%d, sop=%d, eqid=%d\n", __func__,
				epp_fifo_cmd.bits.phy_addr << CA_L3QM_PHY_ADDR_SHIFT, epp_fifo_cmd.bits.eop,
				epp_fifo_cmd.bits.l4_csum_err, epp_fifo_cmd.bits.sop, epp_fifo_cmd.bits.eqid);
		printk("%s: *hw_rd_ptr=%d\n", __func__, *hw_rd_ptr);
	}

	rx_dma_addr = epp_fifo_cmd.bits.phy_addr << CA_L3QM_PHY_ADDR_SHIFT;
	rx_virt_addr = phys_to_virt(rx_dma_addr);

	/* record the physical address and eqid for refilling */
	if (aal_l3qm_eq_get_cpu_rule()) {

		aal_l3qm_eq_get_cpu_pool0_1_eq_id(&cpu_pool0_eq_id, &cpu_pool1_eq_id);
		if (epp_fifo_cmd.bits.eqid == cpu_pool0_eq_id) {
			if (rx_refill_count_pool0 < CA_NI_RX_MAX_REFILL_COUNT) {

				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: rx_refill_count_pool0=%d\n", __func__, rx_refill_count_pool0);
				}
				refill_phy_addr_pool0[rx_refill_count_pool0].phy_addr = rx_dma_addr;
				refill_phy_addr_pool0[rx_refill_count_pool0].eqid = epp_fifo_cmd.bits.eqid;
				rx_refill_count_pool0++;
			}
			else {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: rx_refill_count_pool0=%d, exceed CA_NI_RX_MAX_REFILL_COUNT=%d!!\n",
							__func__, rx_refill_count_pool0, CA_NI_RX_MAX_REFILL_COUNT);
				}
			}
		}
		if (epp_fifo_cmd.bits.eqid == cpu_pool1_eq_id) {
			if (rx_refill_count_pool1 < CA_NI_RX_MAX_REFILL_COUNT) {

				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: rx_refill_count_pool1=%d\n", __func__, rx_refill_count_pool1);
				}
				refill_phy_addr_pool1[rx_refill_count_pool1].phy_addr = rx_dma_addr;
				refill_phy_addr_pool1[rx_refill_count_pool1].eqid = epp_fifo_cmd.bits.eqid;
				rx_refill_count_pool1++;
			}
			else {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: rx_refill_count_pool1=%d, exceed CA_NI_RX_MAX_REFILL_COUNT=%d!!\n",
							__func__, rx_refill_count_pool1, CA_NI_RX_MAX_REFILL_COUNT);
				}
			}
		}
	}

	if (epp_fifo_cmd.bits.sop == 1) {

		if (aal_l3qm_get_ace_test() == 0) {
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: call dma_sync_single_for_cpu() for HEADERs!!!\n", __func__);
			}
			dma_sync_single_for_cpu(&(pdev->dev), rx_dma_addr, head_room + sizeof(HEADER_A_T) + sizeof(HEADER_CPU_T) + sizeof(HEADER_PTP_T), DMA_FROM_DEVICE);
		}

		rx_virt_addr += head_room; /* skip the head room which is preallocated by software */
		hdr_a = (*(HEADER_A_T *)rx_virt_addr);
		hdr_a.bits32.bits32_h = be32_to_cpu(hdr_a.bits32.bits32_h);
		hdr_a.bits32.bits32_l = be32_to_cpu(hdr_a.bits32.bits32_l);

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: hdr_a.bits64=0x%llx\n", __func__, hdr_a.bits64);
			printk("%s: HEADER_A bits: ldpid=%d, deep_q=%d, lspid=%d, cpu_flg=%d, pkt_size=%d, rx_pkt_type=%d\n", __func__,
					hdr_a.bits.ldpid, hdr_a.bits.bits_32_63.pkt_info.deep_q, hdr_a.bits.lspid, hdr_a.bits.bits_32_63.pkt_info.cpu_flg,
					hdr_a.bits.pkt_size, hdr_a.bits.bits_32_63.pkt_info.rx_pkt_type);
		}

		pkt_len = hdr_a.bits.pkt_size;     /* get the packet size from HEADER_A, 14 bits=16K */

		seg_len = min_t(u16, CA_NI_SKB_PKT_LEN, pkt_len);

		if (aal_l3qm_get_ace_test() == 0) {
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: call dma_sync_single_for_cpu() for packet data!!!, rx_dma_addr=%p, seg_len=%d\n", __func__, (void *)rx_dma_addr, seg_len);
			}
			dma_sync_single_for_cpu(&(pdev->dev), rx_dma_addr, seg_len + head_room + sizeof(HEADER_A_T) + sizeof(HEADER_CPU_T) + sizeof(HEADER_PTP_T), DMA_FROM_DEVICE);
		}

		rx_virt_addr += sizeof(HEADER_A_T);

		/* the following if statement might not be satisfied in a rare
		 * case that when RE sends a packet marked with FE_BYPASS, CPU
		 * headers will not be inserted. */
		if (likely(hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)) {
			hdr_cpu = (*(HEADER_CPU_T *)rx_virt_addr);
			hdr_cpu.mdata_raw.mdata_h = be32_to_cpu(hdr_cpu.mdata_raw.mdata_h);
			hdr_cpu.mdata_raw.mdata_l = be32_to_cpu(hdr_cpu.mdata_raw.mdata_l);
			rx_virt_addr += sizeof(HEADER_CPU_T);
		}

		*ret_hdr_a = hdr_a;
		*ret_hdr_cpu = hdr_cpu;
		*ret_hdr_ptp = hdr_ptp;
	}

	*ret_epp_fifo_cmd = epp_fifo_cmd;
	memcpy(ret_epp_fifo_cmd, &epp_fifo_cmd, sizeof(CPU_EPP_FIFO_CMD_T));

	*ret_rx_virt_addr = (u8 *)rx_virt_addr;

	/* advance to next CPU EPP FIFO */
	*hw_rd_ptr += epp_desc_size;

	/* wrap around here */
	if (*hw_rd_ptr >= aal_l3qm_eq_get_max_write_ptr()) {
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: read pointer wrap around at %d\n", __func__, *hw_rd_ptr);
		}
		*hw_rd_ptr = 0;
	}

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: epp_desc_size=%d, hw_rd_ptr=%d\n", __func__, epp_desc_size, *hw_rd_ptr);
	}
	aal_l3qm_set_rx_read_ptr(cpu_port, voq, *hw_rd_ptr);
}

ca_uint16_t ca_ni_rx_get_vid(ca_uint8_t *skb_data)
{
        u16 *tmp_ptr;
        u16 vid;
	u16 tpid;
	struct ethhdr *p_ethhdr;

	p_ethhdr = (struct ethhdr *)skb_data;
	tpid = ntohs(p_ethhdr->h_proto);
	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: TPID of skb = 0x%x\n", __func__, tpid);
		printk("%s: ETH_P_8021Q=0x%x, ETH_P_8021AD=0x%x, ETH_P_QINQ1=0x%x, ETH_P_QINQ2=0x%x\n", __func__,
			ETH_P_8021Q, ETH_P_8021AD, ETH_P_QINQ1, ETH_P_QINQ2);
	}
	/* the vid to pass to AAL level should be 0 as untag */
	vid = 0;
	if (tpid == ETH_P_8021Q || tpid == ETH_P_8021AD || tpid == ETH_P_QINQ1 || tpid == ETH_P_QINQ2) {
		tmp_ptr = (u16 *)(skb_data + sizeof(struct ethhdr));
		vid = (ntohs(*tmp_ptr) & 0x0fff);
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: vid of skb = 0x%x\n", __func__, vid);
		}
	}
	return vid;
}

static int ca_ni_rx_napi_rule1(int cpu_port, u8 voq, struct napi_struct *napi, int budget)
{
	ca_eth_private_t *cep = netdev_priv(napi->dev);
	int received_pkts = 0;
	int pkt_len, seg_len, seg_len1;
	u32 refill_cnt = 0;
	u16 count;
	u32 hw_wr_ptr, hw_rd_ptr;
	u8 *rx_virt_addr, *rx_virt_addr1;
	u16 epp_desc_size;
	int head_room = aal_l3qm_get_cpu_port_head_room_first();
	u8 sop, eop;
	u8 *ptr;
        ca_mac_addr_t sa_mac;
	u16 vid;
	u32 payload_len;

	struct net_device *dev = napi->dev;
	struct sk_buff *skb;
	u8 *save_skb_data;

	CPU_EPP_FIFO_CMD_T epp_fifo_cmd;
	HEADER_A_T hdr_a;
	HEADER_CPU_T hdr_cpu;
	HEADER_PTP_T hdr_ptp;

	ca_status_t rx_callback_ret = CA_E_ERROR;

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: cpu_port=%d, voq=%d, budget=%d\n", __func__, cpu_port, voq, budget);
	}
	epp_desc_size = aal_l3qm_eq_get_desc_per_epp() * aal_l3qm_eq_get_desc_size();

	/* get hardware write pointer */
	hw_wr_ptr = aal_l3qm_get_rx_write_ptr(cpu_port, voq, &count);
	hw_rd_ptr = aal_l3qm_get_rx_read_ptr(cpu_port, voq);

	/* loop until all of packets available are recived or budget is reached */
	/* the unit of refill_cnt is number of CPU EPP command */
	/* the unit of budget is number of packet */
	while ((refill_cnt < count) && (received_pkts < budget)) {

		/* start of packet include packet length */
		ca_ni_rx_napi_read_epp(cpu_port, voq, napi, &rx_virt_addr, &epp_fifo_cmd, &hdr_a, &hdr_cpu, &hdr_ptp, &hw_rd_ptr);
		sop = epp_fifo_cmd.bits.sop;
		eop = epp_fifo_cmd.bits.eop;

		pkt_len = hdr_a.bits.pkt_size;
		if (likely(hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)) {
			pkt_len -= sizeof(HEADER_CPU_T);

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: Header CPU exists, pkt_len=%d\n", __func__, pkt_len);
			}
                }

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: rx_virt_addr=%p, sop=%d, eop=%d, pkt_len=%d\n", __func__, rx_virt_addr, sop, eop, pkt_len);
		}

		if (sop != 1) {
			printk("%s: no SOP is given in packet!!\n", __func__);
			ca_ni_rx_get_bad_packet = 1;
			return received_pkts;
		}


		/* only one segment */
		if (sop == 1 && eop == 1) {
			rx_virt_addr1 = 0;
			received_pkts++;
			refill_cnt++;
		}
		else {

			/* 2nd segment contains data only and we only support 2 segments data, so this should be a EOP */
			ca_ni_rx_napi_read_epp(cpu_port, voq, napi, &rx_virt_addr1, &epp_fifo_cmd, &hdr_a, &hdr_cpu, &hdr_ptp, &hw_rd_ptr);
			eop = epp_fifo_cmd.bits.eop;

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: rx_virt_addr1=%p, eop=%d\n", __func__, rx_virt_addr1, eop);
			}

			if (eop != 1) {
				printk("%s: no EOP is given in packet in rule 1!!\n", __func__);
				ca_ni_rx_get_bad_packet = 1;
				return received_pkts;
			}
			refill_cnt += 2;
			received_pkts++;
		}

		/* allocate skb and fill skb structure */
		skb = ca_netdev_alloc_skb(dev, CA_NI_SKB_PKT_LEN + 0x100, true);

		if (!skb) {
			pr_warning("%s: ca_netdev_alloc_skb() failed!!!\n", dev->name);
			return 0;
		}

		/* fill the structure of skb */
		skb_put(skb, pkt_len);   /* skb->tail += seg_len, skb->len += seg_len */
		skb->len = pkt_len;
		skb->data_len = 0;

		/* end of skb list */
		skb->next = NULL;
		skb->prev = NULL;

		/* copy data from 2 segments */
		seg_len = ca_ni_rx_napi_get_first_segment_length(&hdr_a);
		seg_len1 = hdr_a.bits.pkt_size - 4 - seg_len;

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: seg_len=%d, seg_len1=%d\n", __func__, seg_len, seg_len1);
		}
		ptr = skb->data;
		memcpy(ptr, rx_virt_addr, seg_len);

		/* save the skb->data for later use */
		save_skb_data = skb->data;

		/* store da_mac for later use */
		memcpy(&sa_mac, (skb->data+6), sizeof(ca_mac_addr_t));

		if (rx_virt_addr1 != NULL) {
			ptr += seg_len;
			memcpy(ptr, rx_virt_addr1 + head_room, seg_len1);
		}

		if (dev == NULL) {
			if (hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)
				skb->dev = get_dev_by_cpu_hdr(&hdr_a, ni_info_data.dev[0]);
			else
				skb->dev = ni_info_data.dev[0];
			dev = skb->dev;
		} else {
			skb->dev = dev;
		}

		skb->ip_summed = CHECKSUM_NONE;

		/* Default RX HW checksum enable */
		/* check whether hw support L4 checksum for this packet type */
		if (cep->rx_checksum == CA_ENABLE) {
			if (ca_ni_rx_check_hw_support_l4_csum(save_skb_data, save_skb_data+skb->len) == CA_TRUE) {
				if (epp_fifo_cmd.bits.l4_csum_err == 0) {
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				} else {
					skb->dev->stats.rx_errors++;
				}
			}
		}

		skb->protocol = eth_type_trans(skb, skb->dev);
		skb->pkt_type = PACKET_HOST;
		/* end of fill the sturcture of skb */

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			ca_ni_dump_rx_pkts(skb, &hdr_a, &hdr_cpu, &hdr_ptp);
		}

		/* modify the statistics count of stats */
		if (cep != NULL) {
			int extra_padding = 0;

			if (pkt_len > (dev->mtu + ETH_HLEN + extra_padding)) {
				skb->dev->stats.rx_errors++;
				skb->dev->stats.rx_bytes += pkt_len;
				pr_err("%s:: RX [ldpid %d] pkt_len %d > %d\n", __func__, cep->port_cfg.rx_ldpid, pkt_len, dev->mtu + ETH_HLEN + extra_padding);
			} else {
				skb->dev->stats.rx_packets++;
				skb->dev->stats.rx_bytes += skb->len;
			}
		}

		/* call ca_l2_addr_add() to add a FDB entry */
		/* if hdr type = 2 (CPU bound for l2 learning then call l2 addr add */
		/* should be called before hand out skb to upper layer */
		if (hdr_a.bits.hdr_type == 0x2) {
			vid = ca_ni_rx_get_vid(save_skb_data);
			ca_ni_l2_addr_add(vid, &sa_mac, hdr_a.bits.lspid, hdr_a.bits.bits_32_63.pkt_info.mcgid & 0x7);
		}

		/* call netlink to send packet to user space packet size should decrement by 4 (CRC) */
		payload_len = skb->len + 14;
		rx_callback_ret = ca_ni_netlink_send_packet(cpu_port, &hdr_a, save_skb_data, payload_len, &hdr_cpu, skb);

		/* if Netlink RX queue is full the skb will be freed in netlink send packet */
		if (rx_callback_ret == CA_E_FULL)
			goto out_of_while1;

#ifdef CONFIG_NE_L2FP
		if (ca_l2fp_receive_pkt(cpu_port, voq, skb) == CA_E_OK) {
		} else
#endif
		ca_skb_out(skb);
		if ((dev->features) & NETIF_F_GRO)
			napi_gro_receive(napi, skb);
		else
			netif_receive_skb(skb);
	}

out_of_while1:

	if (rx_refill_count_pool0 == CA_NI_RX_MAX_REFILL_COUNT) {
		ca_ni_rx_napi_refill_empty_buffer();
	}
	if (rx_refill_count_pool1 == CA_NI_RX_MAX_REFILL_COUNT) {
		ca_ni_rx_napi_refill_empty_buffer();
	}

	return received_pkts;
}

/* get net_device pointer from lspid/ldpid of headr A to support WAN port bridge mode */
static struct net_device *ca_ni_rx_get_dev_by_hdr_a(HEADER_A_T *hdr_a, struct napi_struct *napi)
{
	int i, j;

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: ldpid=%d, lspid=%d, wan_port_id=%d\n",
				__func__, hdr_a->bits.ldpid, hdr_a->bits.lspid, wan_port_id);
	}

	if (hdr_a->bits.ldpid == CA_PORT_ID_CPU0) {
		/* CPU#0
		 *	1. packets from lspid 0x18 or 0x7 (ARP is in cos#6.)
		 *	2. Packets trapped by "MTU check fail" and its egress_behavor.ldpid is 0x7, 0x18, or other than 0x19 (egress is WAN)
		 *	3. For Hash type#0 or type#1, Hash mask check fail
		 *	4. Packets which go into Hash profile#0 and not hit
		 *	5. For HashLite type#0, Hash mask check fail
		 *	6. Packets for FDB SW learning (L2FE_PLC_LRN_FWD_CTRL_0.l2_mac_sw_lrn_ldpid)
		 *	7. Packets for FDB hash collsion, SA exceed limit
		 *	8. HashLite profile#0 default rule or ARP miss (MAC inexistent).
		 */
		if ((hdr_a->bits.lspid == CA_PORT_ID_L3_WAN) || (hdr_a->bits.lspid == wan_port_id)) {
			/* case#1 */
			if (ca_ni_debug & NI_DBG_DUMP_RX)
				printk("%s:%d Set to CPU#0 dev %s\n",
						__func__, __LINE__, ni_info_data.dev[0] ? ni_info_data.dev[0]->name: "null");
			return ni_info_data.dev[0];
		}
		else {
			if ((hdr_a->bits.lspid <= CA_PORT_ID_NI7) && (hdr_a->bits.lspid >= CA_PORT_ID_NI0)) {
				/* case#3~7: assign device pointer per lspid */
				for (i = 0; (i < CA_NI_MAX_INTF) && (i < ca_ni_num_intf); i++) {
					for (j = 0; j < port_cfg_infos[i].num_phy_port; j++) {
						if (hdr_a->bits.lspid == port_cfg_infos[i].port_id[j]) {
							if (ca_ni_debug & NI_DBG_DUMP_RX)
								printk("%s:%d Set to dev[%d] %s\n",
										__func__, __LINE__, i, ni_info_data.dev[i] ? ni_info_data.dev[i]->name: "null");
							return ni_info_data.dev[i];
						}
					}
				}
			} else if (hdr_a->bits.lspid == CA_PORT_ID_L3_LAN) {
				/* case#2 & #8: assign device pointer by tx_ldpid */
				for (i = 0; (i < CA_NI_MAX_INTF) && (i < ca_ni_num_intf); i++) {
					if (hdr_a->bits.lspid == port_cfg_infos[i].tx_ldpid) {
						if (ca_ni_debug & NI_DBG_DUMP_RX)
							printk("%s:%d Set to dev[%d] %s\n",
									__func__, __LINE__, i, ni_info_data.dev[i] ? ni_info_data.dev[i]->name: "null");
						return ni_info_data.dev[i];
					}
				}
			}
			/* unknown lspid And set to napi dev by default */
			if (ca_ni_debug & NI_DBG_DUMP_RX)
				printk("%s:%d Set to napi->dev %s\n",
						__func__, __LINE__, napi->dev ? napi->dev->name: "null");
			return napi->dev;
		}
	} else {
		/* For CPU#1
		 *	1. For A53, packets from lspid 0x19 (ARP is in cos#6).
		 *	2. Packets is flooding by unknown VLAN flooding domain (lspid 0x1b, ldpid 0x11)
		 *	3. Packets trapped by "MTU check fail" and its egress_behavor.ldpid is 0x19
		 *	   NOTE: DSLite packets which size is under 94 (without CRC) trigger "MTU check fail"; this is ASIC bug.
		 *	4. Packets which go into Hash profile#1 and not hit (lspid 0x19)
		 */
		if ((hdr_a->bits.lspid == CA_PORT_ID_L3_WAN) || (hdr_a->bits.lspid == wan_port_id)) {
			/* case#3 */
			if (ca_ni_debug & NI_DBG_DUMP_RX)
				printk("%s:%d Set to CPU#0 dev %s\n",
						__func__, __LINE__, ni_info_data.dev[0] ? ni_info_data.dev[0]->name: "null");
			return ni_info_data.dev[0];
		}
		/* set to napi dev by default */
		if (ca_ni_debug & NI_DBG_DUMP_RX)
			printk("%s:%d Set to napi->dev %s\n",
					__func__, __LINE__, napi->dev ? napi->dev->name: "null");
		return napi->dev;
	}
}

int ca_ni_rx_poll_block_mode(int cpu_port, u8 voq, int budget, ca_pkt_t *pkt)
{
        int received_pkts = 0;
        int pkt_len;
        u32 refill_cnt = 0;
        u32 refill_cnt_pool0 = 0;
        u32 refill_cnt_pool1 = 0;
        u16 count;
        u32 hw_wr_ptr, hw_rd_ptr;
        u8 *rx_virt_addr;
        u16 epp_desc_size;
        u8 eqid;
        int header_size;
        int head_room;
        int buf_size;
        struct sk_buff *skb;
        ca_uint8_t *save_skb_data;
        dma_addr_t rx_dma_addr;

        CPU_EPP_FIFO_CMD_T epp_fifo_cmd;
        HEADER_A_T hdr_a;
        HEADER_CPU_T hdr_cpu;
        HEADER_PTP_T hdr_ptp;

        if (ca_ni_debug & NI_DBG_DUMP_RX) {
                printk("%s: cpu_port=%d, voq=%d, budget=%d\n", __func__, cpu_port, voq, budget);
        }

        head_room = aal_l3qm_get_cpu_port_head_room_first();
        buf_size = aal_l3qm_eq_get_cpu_pool0_buf_size();

        epp_desc_size = aal_l3qm_eq_get_desc_per_epp() * aal_l3qm_eq_get_desc_size();

        /* get hardware write pointer */
        hw_wr_ptr = aal_l3qm_get_rx_write_ptr(cpu_port, voq, &count);
        hw_rd_ptr = aal_l3qm_get_rx_read_ptr(cpu_port, voq);

        /* loop until all of packets available are recived or budget is reached */
        /* the unit of refill_cnt is number of CPU EPP command */
        /* the unit of budget is number of packet */
        while ((refill_cnt < count) && (received_pkts < budget)) {

                if (ca_ni_debug & NI_DBG_DUMP_RX) {
                        printk("%s: hw_wr_ptr=%d, hw_rd_ptr=%d, refill_cnt=%d, count=%d, received_pkts=%d, budget=%d\n", __func__,
                                        hw_wr_ptr, hw_rd_ptr, refill_cnt, count, received_pkts, budget);
                }
                header_size = 0;

		ca_ni_rx_napi_read_epp(cpu_port, voq, NULL, &rx_virt_addr, &epp_fifo_cmd, &hdr_a, &hdr_cpu, &hdr_ptp, &hw_rd_ptr);

                /* the rx_virt_addr here has been skip head_room and headers */
                rx_dma_addr = epp_fifo_cmd.bits.phy_addr << CA_L3QM_PHY_ADDR_SHIFT;
                rx_virt_addr = phys_to_virt(rx_dma_addr);

                eqid = epp_fifo_cmd.bits.eqid;

                if (ca_ni_debug & NI_DBG_DUMP_RX) {
                        printk("%s: rx_virt_addr=%p, sop=%d, eop=%d, pkt_len=%d\n", __func__, rx_virt_addr, epp_fifo_cmd.bits.sop, epp_fifo_cmd.bits.eop, hdr_a.bits.pkt_size);
                }

                if (epp_fifo_cmd.bits.sop != 1) {
                        printk("%s: no SOP is given in packet!!\n", __func__);
                        ca_ni_rx_get_bad_packet = 1;
#ifdef CONFIG_RTK_PERF_EVENTS
                        stop_perf_counter(2, 0x1);
#endif
                        return received_pkts;
                }

                /* use build_skb() here when there is no sk_buff stored in QM pool */
                if (ca_ni_use_build_skb) {
                        /* check whether this is a packet for this cpu port */
                        if (hdr_a.bits.pkt_size == 0 || (hdr_a.bits.ldpid != (AAL_LPORT_CPU_0 + cpu_port))) {
                                skb_free_frag(rx_virt_addr);
                                printk("%s: hdr_a.bits.pkt_size=%d OR hdr_a.bits.ldpid=%d not correct!!\n", __func__, hdr_a.bits.pkt_size, hdr_a.bits.ldpid);
                                return 0;
                        }
                        save_skb_data = rx_virt_addr;
                }
                else {
                        skb = (struct sk_buff *)(*((ca_uint_t *)(rx_virt_addr)));
                        save_skb_data = skb->data;
                }

                header_size += head_room;

                pkt_len = hdr_a.bits.pkt_size;     /* get the packet size from HEADER_A, 14 bits=16K */

                if (ca_ni_debug & NI_DBG_DUMP_RX) {
                        printk("%s: save_skb_data=%p, pkt_len=%d\n", __func__, save_skb_data, pkt_len);
                }

                header_size += sizeof(HEADER_A_T);

		/* the following if statement might not be satisfied in a rare
		 * case that when RE sends a packet marked with FE_BYPASS, CPU
		 * headers will not be inserted. */
                if (likely(hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)) {
                        header_size += sizeof(HEADER_CPU_T);
                        /* pkt_len does not include header size */
                        pkt_len -= sizeof(HEADER_CPU_T);
                        if (ca_ni_debug & NI_DBG_DUMP_RX) {
                                printk("%s: Header CPU exists, pkt_len=%d\n", __func__, pkt_len);
                        }
                }

                save_skb_data += header_size;
                {
                        copy_to_user(pkt->pkt_data->data, save_skb_data, pkt_len);
                        copy_to_user(&(pkt->pkt_len), &pkt_len, sizeof(ca_uint16_t));
                        copy_to_user(&(pkt->pkt_data->len), &pkt_len, sizeof(ca_uint16_t));
                }

                refill_cnt++;

                /* record refill_cnt for EQ pool0/1 */
                if (eqid == l3qm_cpu_pool0_eq_id)
                        refill_cnt_pool0++;

                if (eqid == l3qm_cpu_pool1_eq_id)
                        refill_cnt_pool1++;

                if (ca_ni_debug & NI_DBG_DUMP_RX) {
                        printk("%s: skb->data[]=\n", __func__);
                        print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, (void *)save_skb_data, pkt_len, true);
                }
                if (ca_ni_use_build_skb)
                        skb_free_frag(save_skb_data);
                else
                        ca_dev_kfree_skb(skb);
                received_pkts++;
                pkt++;
        }

        /* if CFG_ID_NE_USE_QM_EQ_REFILL_INT=1, refill will be done in interrupt service routine */
        if (ca_ni_use_qm_eq_refill_int == 0)  {
                struct net_device *dev = ni_info_data.dev[0];
                if (aal_l3qm_eq_get_cpu_use_fbm()){
                        if (refill_cnt_pool0)
                                ca_ni_refill_eq_buf_pool(dev, cpu_port, l3qm_cpu_pool0_eq_id, refill_cnt_pool0);
                        if (refill_cnt_pool1)
                                ca_ni_refill_eq_buf_pool(dev, cpu_port, l3qm_cpu_pool1_eq_id, refill_cnt_pool1);
                }
                else {
                        eqid = l3qm_cpu_pool0_eq_id;
                        if (refill_cnt_pool0) {
                                refill_cnt = aal_l3qm_get_inactive_bid_cntr(eqid);
                                if (refill_cnt)
                                        ca_ni_refill_eq_buf_pool(dev, cpu_port, eqid, refill_cnt);
                        }

                        eqid = l3qm_cpu_pool1_eq_id;
                        if (refill_cnt_pool1) {
                                refill_cnt = aal_l3qm_get_inactive_bid_cntr(eqid);
                                if (refill_cnt)
                                        ca_ni_refill_eq_buf_pool(dev, cpu_port, eqid, refill_cnt);
                        }
                }
        }
        printk("RX %d pkts\n", received_pkts);
        return received_pkts;
}

static int ca_ni_rx_napi(int cpu_port, u8 voq, struct napi_struct *napi, int budget)
{
	ca_eth_private_t *cep;
	struct platform_device *pdev;
	int received_pkts = 0;
	int jumbo_pkt_index = 0;
	int pkt_len, seg_len;
	u32 refill_cnt = 0;
        u32 refill_cnt_pool0 = 0;
        u32 refill_cnt_pool1 = 0;
	u16 count;
	u32 hw_wr_ptr, hw_rd_ptr;
	u8 *rx_virt_addr;
	u16 epp_desc_size;
	u8 eqid;
	int header_size;
	int head_room;
	int buf_size;
	ca_mac_addr_t sa_mac;
	ca_mac_addr_t da_mac;
	u16 vid;
	ca_status_t rx_callback_ret = CA_E_ERROR;

	dma_addr_t rx_dma_addr;
	struct net_device *dev;
	struct sk_buff *skb, *tmp_skb = NULL, *tail_skb = NULL;
	ca_uint8_t *save_skb_data;

	CPU_EPP_FIFO_CMD_T epp_fifo_cmd;
	HEADER_A_T hdr_a;
	HEADER_CPU_T hdr_cpu;
	HEADER_PTP_T hdr_ptp;
	u32 payload_len;

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: cpu_port=%d, voq=%d, budget=%d\n", __func__, cpu_port, voq, budget);
	}

#ifdef CONFIG_RTK_PERF_EVENTS
	start_perf_counter(0x1); // save to idx 0
#endif

	cep = netdev_priv(napi->dev);
	pdev = cep->pdev;

	head_room = aal_l3qm_get_cpu_port_head_room_first();
	buf_size = aal_l3qm_eq_get_cpu_pool0_buf_size();

	epp_desc_size = aal_l3qm_eq_get_desc_per_epp() * aal_l3qm_eq_get_desc_size();

	/* get hardware write pointer */
	hw_wr_ptr = aal_l3qm_get_rx_write_ptr(cpu_port, voq, &count);
	hw_rd_ptr = aal_l3qm_get_rx_read_ptr(cpu_port, voq);

#ifdef CONFIG_RTK_PERF_EVENTS
	stop_perf_counter(0, 0x1);
#endif

	/* loop until all of packets available are recived or budget is reached */
	/* the unit of refill_cnt is number of CPU EPP command */
	/* the unit of budget is number of packet */
	while ((refill_cnt < count) && (received_pkts < budget)) {

#ifdef CONFIG_RTK_PERF_EVENTS
		start_perf_counter(0x1); // save to idx 2
#endif

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: hw_wr_ptr=%d, hw_rd_ptr=%d, refill_cnt=%d, count=%d, received_pkts=%d, budget=%d\n", __func__,
				hw_wr_ptr, hw_rd_ptr, refill_cnt, count, received_pkts, budget);
		}
		header_size = 0;

		ca_ni_rx_napi_read_epp(cpu_port, voq, napi, &rx_virt_addr, &epp_fifo_cmd, &hdr_a, &hdr_cpu, &hdr_ptp, &hw_rd_ptr);

		/* get net_device pointer according to header A for WAN port bridge mode */
		dev = ca_ni_rx_get_dev_by_hdr_a(&hdr_a, napi);

		/* the rx_virt_addr here has been skip head_room and headers */
		rx_dma_addr = epp_fifo_cmd.bits.phy_addr << CA_L3QM_PHY_ADDR_SHIFT;
		rx_virt_addr = phys_to_virt(rx_dma_addr);

		eqid = epp_fifo_cmd.bits.eqid;

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: rx_virt_addr=%p, sop=%d, eop=%d, pkt_len=%d\n", __func__, rx_virt_addr, epp_fifo_cmd.bits.sop, epp_fifo_cmd.bits.eop, hdr_a.bits.pkt_size);
		}

		if (epp_fifo_cmd.bits.sop != 1) {
			printk("%s: no SOP is given in packet!!\n", __func__);
			ca_ni_rx_get_bad_packet = 1;
#ifdef CONFIG_RTK_PERF_EVENTS
			stop_perf_counter(2, 0x1);
#endif
			return received_pkts;
		}

		/* use build_skb() here when there is no sk_buff stored in QM pool */
		if (ca_ni_use_build_skb) {
			int tmp_len = CA_NI_SKB_PKT_LEN + head_room + 16;

			/* check whether this is a packet for this cpu port */
			if (hdr_a.bits.pkt_size == 0 || (hdr_a.bits.ldpid != (AAL_LPORT_CPU_0 + cpu_port))) {
				skb_free_frag(rx_virt_addr);
				printk("%s: hdr_a.bits.pkt_size=%d OR hdr_a.bits.ldpid=%d not correct!!\n", __func__, hdr_a.bits.pkt_size, hdr_a.bits.ldpid);
#ifdef CONFIG_RTK_PERF_EVENTS
				stop_perf_counter(2, 0x1);
#endif
				return 0;
			}
			tmp_len += NET_SKB_PAD + SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
			tmp_len = SKB_DATA_ALIGN(tmp_len);
			skb = build_skb(rx_virt_addr, tmp_len);
			if (skb == NULL) {
				skb_free_frag(rx_virt_addr);
				printk("%s: not able to allocate skb here!!\n", __func__);
#ifdef CONFIG_RTK_PERF_EVENTS
				stop_perf_counter(2, 0x1);
#endif
				return 0;
			}
			skb->dev = dev;

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: build_skb skb=0x%p, rx_virt_addr=0x%p, skb->data=0x%p\n", __func__, skb, rx_virt_addr, skb->data);
			}
		}
		else {
			/* the sk_buff address stored at first 8 byte of packet buffer */
			skb = (struct sk_buff *)(*((ca_uint_t *)(rx_virt_addr)));
		}

#ifdef CONFIG_RTK_CONTROL_PACKET_PROTECTION
		skb->priority = hdr_a.bits.cos;
#endif

		skb->data += head_room; /* skip head_room */
		header_size += head_room;

		pkt_len = hdr_a.bits.pkt_size;     /* get the packet size from HEADER_A, 14 bits=16K */

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: pkt_len=%d\n", __func__, pkt_len);
		}

		skb->data += sizeof(HEADER_A_T);
		header_size += sizeof(HEADER_A_T);

		/* the following if statement might not be satisfied in a rare
		 * case that when RE sends a packet marked with FE_BYPASS, CPU
		 * headers will not be inserted. */
		if (likely(hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)) {
			skb->data += sizeof(HEADER_CPU_T);
			header_size += sizeof(HEADER_CPU_T);
			/* pkt_len does not include header size */
			pkt_len -= sizeof(HEADER_CPU_T);
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: Header CPU exists, pkt_len=%d\n", __func__, pkt_len);
			}
		}

		/* keep the skb->data which point to the beginning of packet data */
		save_skb_data = skb->data;

#if defined(CONFIG_RTK_NIC_RX_HOOK)
		/* not required */
#else
		/* store sa_mac and da_mac for later use */
		memcpy(&da_mac, (skb->data), sizeof(ca_mac_addr_t));
		memcpy(&sa_mac, (skb->data+6), sizeof(ca_mac_addr_t));
#endif

		skb_reset_tail_pointer(skb);

		if (pkt_len > (buf_size - header_size)) {
			seg_len = buf_size - header_size;
		}
		else {
			seg_len = pkt_len;
		}

		skb_put(skb, seg_len);   /* skb->tail += seg_len, skb->len += seg_len */
		payload_len = pkt_len;
		skb->len = pkt_len;

		/* if not a jumbo packet, the pkt_len will become to 0 */
		pkt_len -= seg_len;

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s: seg_len=%d, pkt_len=%d, buf_size=%d payload_len=%d\n",
				__func__, seg_len, pkt_len, buf_size, payload_len);
		}

		skb->data_len = pkt_len;   /* skb->data_len will be 0 or greater than 0 (jumbo packet) */
		jumbo_pkt_index = 0;

		refill_cnt++;

		/* record refill_cnt for EQ pool0/1 */
		if (eqid == l3qm_cpu_pool0_eq_id)
			refill_cnt_pool0++;

		if (eqid == l3qm_cpu_pool1_eq_id)
			refill_cnt_pool1++;

		/* consider jumbo frame */
		while (pkt_len > 0  && !epp_fifo_cmd.bits.eop) {

			ca_ni_rx_napi_read_epp(cpu_port, voq, napi, &rx_virt_addr, &epp_fifo_cmd, &hdr_a, &hdr_cpu, &hdr_ptp, &hw_rd_ptr);

			rx_dma_addr = virt_to_phys(rx_virt_addr);

			/* use build_skb() here when there is no sk_buff stored in QM pool */
			if (ca_ni_use_build_skb) {
				int tmp_len = CA_NI_SKB_PKT_LEN + head_room + 16;

				tmp_len += NET_SKB_PAD + SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
				tmp_len = SKB_DATA_ALIGN(tmp_len);
				tmp_skb = build_skb(rx_virt_addr, tmp_len);
				if (tmp_skb == NULL) {
					skb_free_frag(rx_virt_addr);
					printk("%s: not able to allocate skb here!!\n", __func__);
#ifdef CONFIG_RTK_PERF_EVENTS
					stop_perf_counter(2, 0x1);
#endif
					return 0;
				}
				tmp_skb->dev = dev;

				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: build_skb skb=0x%p, rx_virt_addr=0x%p, tmp_skb->data=0x%p\n", __func__, skb, rx_virt_addr, tmp_skb->data);
				}
			}
			else {
				/* the sk_buff address stored at first 8 byte of packet buffer */
				tmp_skb = (struct sk_buff *)(*((ca_uint_t *)(rx_virt_addr)));
			}

			tmp_skb->data += head_room; 	/* skip head room rest same size as head room first */

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: jumbo_pkt_index=%d, tmp_skb=%p, tmp_skb->data=%p, phy_data=0x%llx, tmp_skb->head=%p\n",
						__func__, jumbo_pkt_index, tmp_skb, tmp_skb->data, (ca_uint64_t)virt_to_phys(tmp_skb->data), tmp_skb->head);
			}

			/* the buffer size should minus head romm reset size */
			seg_len = min_t(u16, buf_size - head_room, pkt_len);

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: seg_len=%d, buf_size=%d, pkt_len=%d\n", __func__, seg_len, buf_size, pkt_len);
			}

			/* 3.4.11 pass NULL to force to do arm dma map if QM ACP disabled */
			if (aal_l3qm_get_ace_test() == 0) {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: call dma_sync_single_for_cpu() for packet fragment data!!!\n", __func__);
				}
				dma_sync_single_for_cpu(&(pdev->dev), rx_dma_addr, seg_len + head_room, DMA_FROM_DEVICE);
			}

			tmp_skb->len = 0;
			skb_put(tmp_skb, seg_len);   /* tmp_skb->len += seg_len, tmp_skb->tail += seg_len */
			tmp_skb->next = NULL;
			tmp_skb->prev = NULL;
			if (jumbo_pkt_index == 0) {
				skb_shinfo(skb)->frag_list = tmp_skb;
				tail_skb = tmp_skb;
			} else {
				tail_skb->next = tmp_skb;
				tail_skb = tmp_skb;
			}
			pkt_len -= seg_len;

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: after fragment pkt_len=%d\n", __func__, pkt_len);
			}
			jumbo_pkt_index++;
			refill_cnt++;

			/* record refill_cnt for EQ pool0/1 */
			if (eqid == l3qm_cpu_pool0_eq_id)
				refill_cnt_pool0++;

			if (eqid == l3qm_cpu_pool1_eq_id)
				refill_cnt_pool1++;

			/* end of packet */
			if (epp_fifo_cmd.bits.eop) {
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s: end of fragment packet!!!\n", __func__);
				}
				break;
			}
		}

		/* end of skb list */
		skb->next = NULL;
		skb->prev = NULL;

		/* fill the structure of skb */
		if (dev == NULL) {
			if (hdr_a.bits.bits_32_63.pkt_info.cpu_flg != 0)
				skb->dev = get_dev_by_cpu_hdr(&hdr_a, ni_info_data.dev[0]);
			else
				skb->dev = ni_info_data.dev[0];
			dev = skb->dev;
		} else {
			skb->dev = dev;
		}

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_RTL_MULTI_LAN_DEV)
#if defined(CONFIG_HWNAT_RG) || defined(CONFIG_HWNAT_RG_MODULE) || defined(CONFIG_HWNAT_FLEETCONNTRACK)

		unsigned int phy_src_port = 0, phy_dst_port = 0;
		unsigned char is_flowHit_to_wifi = 0;
		phy_src_port = PORT_ID(hdr_a.bits.lspid);
		phy_dst_port = PORT_ID(hdr_a.bits.ldpid);
		is_flowHit_to_wifi = ((0x12 <= phy_dst_port) && (phy_dst_port <= 0x17))?1:0;
#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_RG_G3_L2FE_POL_OFFLOAD)
		if((phy_src_port == G3_LOOPBACK_P_NEWSPA) && (!is_flowHit_to_wifi))
			ca_ni_decide_rx_lspib_by_fakeVlan(skb, &phy_src_port, &hdr_a); //decide source port by fake vlan
#endif		
		skb->dev = ca_ni_decide_rx_device(cep, phy_src_port);

#else

		aal_fdb_entry_cfg_t fdb_entry;
		unsigned int phy_src_port = 0;
		vid = ca_ni_rx_get_vid(save_skb_data);
		if (aal_fdb_entry_get(0, sa_mac, vid, &fdb_entry) == CA_E_OK) {
			if (fdb_entry.valid) {
				phy_src_port = PORT_ID(fdb_entry.port_id);
			}
		}
		skb->dev = ca_ni_decide_rx_device(cep, phy_src_port);
#endif
#endif

		skb->ip_summed = CHECKSUM_NONE;

		/* Default RX HW checksum enable */
		/* check whether hw support L4 checksum for this packet type */
		if (cep->rx_checksum == CA_ENABLE) {
			if (ca_ni_rx_check_hw_support_l4_csum(save_skb_data, save_skb_data+skb->len) == CA_TRUE) {
				if (epp_fifo_cmd.bits.l4_csum_err == 0) {
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				} else {
					skb->dev->stats.rx_errors++;
				}
			}
		}

#ifdef CONFIG_NE_L2FP
		if (ca_l2fp_receive_pkt(cpu_port, voq, skb) == CA_E_OK) {
#if defined(CONFIG_LUNA_G3_SERIES)
			// HWNAT: cpu port 0x12~0x17 are used to fast forward for wifi data, keep receiving via NIC rx function.
			skb->dev = cep->dev;
#else
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s cpu_port 0x%x, voq %d, skb %p to l2fp module\n", __func__, cpu_port, voq, skb);
			}
			ca_skb_out(skb);
#ifdef CONFIG_RTK_PERF_EVENTS
			stop_perf_counter(2, 0x1);
#endif
			goto skb_handled;
#endif
		}
#endif

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_RTK_NIC_RX_HOOK)

#ifdef CONFIG_PROC_FS
		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			ca_ni_dump_rx_pkts(skb, &hdr_a, &hdr_cpu, &hdr_ptp);  /* for segmented packets */
		}
#endif

	{
		nic_hook_private_t nh_priv;
		nh_priv.save_skb_data = save_skb_data;
#if defined(CONFIG_HWNAT_RG) || defined(CONFIG_HWNAT_RG_MODULE) || defined(CONFIG_HWNAT_FLEETCONNTRACK)
		//no needed for RG/FC
#else
		memcpy(&(nh_priv.da_mac), &(da_mac), sizeof(ca_mac_addr_t));
		memcpy(&(nh_priv.sa_mac), &(sa_mac), sizeof(ca_mac_addr_t));
#endif
		nh_priv.hdr_a = (HEADER_A_T *)&hdr_a;
		nh_priv.hdr_cpu = (HEADER_CPU_T *)&hdr_cpu;		
		nh_priv.hdr_ptp = (HEADER_PTP_T *)&hdr_ptp;				
		/* modify the statistics count of stats */
		if (cep != NULL) {
			int extra_padding = 0;

			if (pkt_len > (dev->mtu + ETH_HLEN + extra_padding)) {
				skb->dev->stats.rx_errors++;
				skb->dev->stats.rx_bytes += pkt_len;
#ifdef CONFIG_FC_RTL8198F_SERIES
				/* modify multi-lan dev statistics counter */
				stats_rx_netdev = &skb->dev->stats;
				stats_rx_netdev->rx_errors++;
				stats_rx_netdev->rx_bytes += pkt_len;
#endif
				pr_err("%s:: RX [ldpid %d] pkt_len %d > %d\n", __func__, cep->port_cfg.rx_ldpid, pkt_len, dev->mtu + ETH_HLEN + extra_padding);
			} else {
				skb->dev->stats.rx_packets++;
				skb->dev->stats.rx_bytes += skb->len;
#ifdef CONFIG_FC_RTL8198F_SERIES
				/* modify multi-lan dev statistics counter */
				stats_rx_netdev = &skb->dev->stats;
				stats_rx_netdev->rx_packets++;
				stats_rx_netdev->rx_bytes += skb->len;
#endif
			}
		}

#if defined(CONFIG_RTL_FPGA_PHY_TEST)
		if(ni_rx_tx_test.test_mode){
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s-%d: Enter ni rx tx test mode %d\n", __func__,__LINE__, ni_rx_tx_test.test_mode);
			}
			if(ni_rx_tx_test.remarking_dip){	//always remarking original dip as new one
				//ATTENTATION: save_skb_data ptr will be update due to this function.
				if(ntohs(ca_ni_parse_l2(save_skb_data, NULL, NULL, NULL, &save_skb_data, false)) == ETH_P_IP){
					if (save_skb_data + sizeof(struct iphdr) <= save_skb_data+skb->len){
						struct iphdr *iph = (struct iphdr *) save_skb_data;
						if (ca_ni_debug & NI_DBG_DUMP_RX)
							printk("%s-%d: Force remarking dip from %pI4 to 0x%x\n", __func__,__LINE__, &iph->daddr, ni_rx_tx_test.remarking_dip);
						iph->daddr = htonl(ni_rx_tx_test.remarking_dip);
					}
				}
			}
			if(hdr_a.bits.ldpid >=0x12 && hdr_a.bits.ldpid <=0x17)
			{
				ca_uint8_t *temp_skb_data;
				int ori_tos = 0, ssid = 0;
				temp_skb_data = skb->data;
				
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s-%d: Fast Forward to WIFI! ldpid = %x\n", __func__,__LINE__, hdr_a.bits.ldpid);
				}
				if(ni_rx_tx_test.verify_ldpid != 0xffffffff)
					ni_rx_tx_test.ni_tx_config.core_config.bf.ldpid = ni_rx_tx_test.verify_ldpid;
				if(ntohs(ca_ni_parse_l2(temp_skb_data, NULL, NULL, NULL, &temp_skb_data, false)) == ETH_P_IP && ni_rx_tx_test.hash_act_dscp_msk!=0){
					if (ca_ni_debug & NI_DBG_DUMP_RX) {
						printk("%s-%d: rx test mode!ip packet! temp_skb_data = %p\n", __func__,__LINE__,temp_skb_data);
					}
					if (temp_skb_data + sizeof(struct iphdr) <= temp_skb_data+skb->len){
						struct iphdr *iph = (struct iphdr *) temp_skb_data;

						ori_tos = iph->tos;
						if(skb->data[0]&0x1)	// MC: fwd by l2FE mce + arb table, pol_id stands for wifi flow id
							ssid = hdr_a.bits.bits_32_63.pkt_info.pol_id;
						else					// UC: fwd by L3FE mainhash, matadata stands for wifi flow id 
							ssid = (hdr_cpu.mdata_raw.mdata_l&0x3f);

						if(ssid != 0)
							iph->tos =ssid<<2 ;

						if (ca_ni_debug & NI_DBG_DUMP_RX)
							printk("%s-%d: Force remarking tos from %d to %d\n", __func__,__LINE__, ori_tos, iph->tos);
					}
				}
				ca_ni_start_xmit_buf(skb->data, skb->len, NULL, 0, skb, &ni_rx_tx_test.ni_tx_config);
			}
			else if(hdr_a.bits.lspid >=0x12 && hdr_a.bits.lspid <=0x17)
			{
				
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s-%d: WIFI HW Lookup miss! lspid = %x\n", __func__,__LINE__, hdr_a.bits.lspid);
				}
				if(ni_rx_tx_test.verify_ldpid != 0xffffffff)
					ni_rx_tx_test.ni_tx_config.core_config.bf.ldpid = ni_rx_tx_test.verify_ldpid;
				
				
				ca_ni_start_xmit_buf(skb->data, skb->len, NULL, 0, skb, &ni_rx_tx_test.ni_tx_config);
			}
			else
				ca_ni_start_xmit_buf(skb->data, skb->len, NULL, 0, skb, &ni_rx_tx_test.ni_tx_config);
		}
		else
#endif
		{
			int netID=0;
			netID = get_dev_index_cpu_hdr(&hdr_a);
			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s-%d: netID=%d dev=%s\n", __func__,__LINE__, netID, cep->dev->name);
			}
			//skb->dev = cep->port2dev[hdr_a.bits.lspid];
			if(netID<CA_NI_MAX_INTF && cep->port2rxfunc[netID])
				cep->port2rxfunc[netID](napi,dev,skb,&nh_priv);
			else
			{
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					printk("%s-%d: ERROR ! should not happend ! packet received from unexpected CPU port ! (netID=%d dev=%s)\n", __func__,__LINE__, netID, cep->dev->name);
				}
				skb->protocol = eth_type_trans(skb, skb->dev);
				skb->pkt_type = PACKET_HOST;
				if ((dev->features) & NETIF_F_GRO)
					napi_gro_receive(napi, skb);
				else
					netif_receive_skb(skb);
			}
		}
	}	
#else /*CONFIG_RTK_NIC_RX_HOOK*/

		skb->protocol = eth_type_trans(skb, skb->dev);
		skb->pkt_type = PACKET_HOST;
		/* end of fill the sturcture of skb */

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			ca_ni_dump_rx_pkts(skb, &hdr_a, &hdr_cpu, &hdr_ptp);  /* for segmented packets */
		}

		if (*(skb->data - 2) == 0xff && *(skb->data - 1) == 0xf1) {
			ca_uint8_t pon_mode = 0;
			scfg_read(0, CFG_ID_PON_MAC_MODE, sizeof(pon_mode), (ca_uint8_t *) &pon_mode);
			if (CA_PON_MODE_IS_GPON_FAMILY(pon_mode)) {
				if ((PON_DBG_GPON_OMCI_RX & ca_pon_debug)) {
					ca_buffer_dump("Rx OMCI <=", skb->data - ETH_HLEN, skb->len + ETH_HLEN);
				}
			} else if ((PON_DBG_EPON_OAM_RX & ca_pon_debug) ) {
				ca_buffer_dump("Rx OAM <=", skb->data - ETH_HLEN, skb->len + ETH_HLEN);
			}

		}while(0);

		/* call ca_l2_addr_add() to add a FDB entry */
		/* if hdr type = 2 (CPU bound for l2 learning then call l2 addr add */
		if (hdr_a.bits.hdr_type == 0x2) {

			vid = ca_ni_rx_get_vid(save_skb_data);
			ca_ni_l2_addr_add(vid, &sa_mac, hdr_a.bits.lspid, hdr_a.bits.bits_32_63.pkt_info.mcgid & 0x7);

			/* should check if UUC packet flooding to other ports in the same flooding domain */
			ca_ni_l2_flooding(vid, da_mac, skb, napi, &hdr_a);
		}

		/* call netlink to send packet to user space packet size should decrement by 4 (CRC) */
		rx_callback_ret = ca_ni_netlink_send_packet(cpu_port, &hdr_a, save_skb_data, payload_len, &hdr_cpu, skb);

		/* if Netlink RX queue is full the skb will be freed in netlink send packet */
		if (rx_callback_ret == CA_E_FULL)
			goto out_of_while;

		/* call back to the routines registered by kernel mode __ca_rx_register() */
		/* if user space has register this packet type use the return value from ca_ni_netlink_send_packet() */
		if (ca_ni_rx_register_callback(cpu_port, &hdr_a, save_skb_data, payload_len, &hdr_cpu) == CA_E_OK) {
			rx_callback_ret = CA_E_OK;
		}

		/* modify the statistics count of stats */
		if (cep != NULL) {
			int extra_padding = 0;

			if (payload_len > (dev->mtu + ETH_HLEN + extra_padding)) {
				skb->dev->stats.rx_errors++;
				skb->dev->stats.rx_bytes += payload_len;
				if (ca_ni_debug & NI_DBG_DUMP_RX) {
					pr_err("%s:: RX [ldpid %d] pkt_len %d > %d\n", __func__, cep->port_cfg.rx_ldpid, payload_len, dev->mtu + ETH_HLEN + extra_padding);
				}
			} else {
				skb->dev->stats.rx_packets++;
				skb->dev->stats.rx_bytes += payload_len;
			}
		}

		ca_skb_out(skb);

#ifdef CONFIG_RTK_PERF_EVENTS
		stop_perf_counter(2, 0x1);
#endif

		/* if the incoming packet has been registered, then drop the packet */
		if (rx_callback_ret == CA_E_OK) {
			/* if use Netlink RX sync mode drop by Netlink kernel thread */
			if (!netlink_rx_ack) {
				netlink_rx_free_skb_count++;
				//printk("R");
				ca_dev_kfree_skb(skb);
			}
			if (ca_ni_debug & NI_DBG_DUMP_RX) {

				printk("%s: packet is dropped!!\n", __func__);
			}
		}
		else {
			if ((dev->features) & NETIF_F_GRO)
				napi_gro_receive(napi, skb);
			else
				netif_receive_skb(skb);
		}
		
#endif /*CONFIG_RTK_NIC_RX_HOOK*/

		received_pkts++;
	}

#ifdef CONFIG_RTK_PERF_EVENTS
	start_perf_counter(0x1); // save to idx 5
#endif

	/* if CFG_ID_NE_USE_QM_EQ_REFILL_INT=1, refill will be done in interrupt service routine */
	if (ca_ni_use_qm_eq_refill_int == 0)  {
		if (aal_l3qm_eq_get_cpu_use_fbm()){
			if (refill_cnt_pool0)
				ca_ni_refill_eq_buf_pool(dev, cpu_port, l3qm_cpu_pool0_eq_id, refill_cnt_pool0);
			if (refill_cnt_pool1)
				ca_ni_refill_eq_buf_pool(dev, cpu_port, l3qm_cpu_pool1_eq_id, refill_cnt_pool1);
		}
		else {
			eqid = l3qm_cpu_pool0_eq_id;
			if (refill_cnt_pool0) {
				refill_cnt = aal_l3qm_get_inactive_bid_cntr(eqid);
				if (refill_cnt)
					ca_ni_refill_eq_buf_pool(dev, cpu_port, eqid, refill_cnt);
			}

			eqid = l3qm_cpu_pool1_eq_id;
			if (refill_cnt_pool1) {
				refill_cnt = aal_l3qm_get_inactive_bid_cntr(eqid);
				if (refill_cnt)
					ca_ni_refill_eq_buf_pool(dev, cpu_port, eqid, refill_cnt);
			}
		}
	}

out_of_while:

#ifdef CONFIG_RTK_PERF_EVENTS
	stop_perf_counter(5, 0x1);
#endif
	

	return received_pkts;
}

/*********************************************
 * Packet receive routine from Management FE
 * Expects a previously allocated buffer and
 * fills the length
 * Retruns 0 on success -1 on failure
 *******************************************/
int ca_ni_mgmt_recv(int instance, struct napi_struct *napi, int budget)
{
	struct net_device *dev = napi->dev;
	ca_eth_private_t *cep = netdev_priv(napi->dev);
	struct ca_ni_priv   *priv = (struct ca_ni_priv *) &(cep->mgmt_priv);
	NI_HEADER_X_T           header_x;
	u32                     pktlen=0;
	u32                     sw_rx_rd_ptr;
	u32                     hw_rx_wr_ptr;
	volatile u32            *rx_xram_ptr;
	int                     loop;
	u32                     *data_ptr;
	unsigned char    	*pkt_buf;
	struct sk_buff *skb;
	NI_PACKET_STATUS_T      packet_status;
	NI_HV_XRAM_CPUXRAM_CPU_STA_RX_0_t cpuxram_cpu_sta_rx;
	NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0_t cpuxram_cpu_cfg_rx;
	int received_pkts = 0;
	dma_addr_t rx_dma_addr;

	/* get the hw write pointer */
	cpuxram_cpu_sta_rx.wrd = CA_NI_REG_READ(NI_HV_XRAM_CPUXRAM_CPU_STA_RX_0);
	hw_rx_wr_ptr = cpuxram_cpu_sta_rx.bf.pkt_wr_ptr;

	/* get the sw read pointer */
	cpuxram_cpu_cfg_rx.wrd = CA_NI_REG_READ(NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0);
	sw_rx_rd_ptr = cpuxram_cpu_cfg_rx.bf.pkt_rd_ptr;

	while (sw_rx_rd_ptr != hw_rx_wr_ptr) {

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s : RX hw_wr_ptr=%d sw_rd_ptr=%d\n", __func__, hw_rx_wr_ptr, sw_rx_rd_ptr);
		}

		skb = ca_netdev_alloc_skb(dev, CA_NI_SKB_PKT_LEN + 0x100, true);

		if (!skb) {
			printk(KERN_ERR "%s: Not enough memory to allocate skb of size %d.\n", dev->name, CA_NI_SKB_PKT_LEN + 0x100);
			return -ENOMEM;
		}
		pkt_buf = skb->data;

		/* Point to the absolute memory address of XRAM where read pointer is */
		rx_xram_ptr = (u32 *)((ca_uint_t)AAL_XRAM_REG_ADDR(sw_rx_rd_ptr * 8));

		/* Wrap around if required */
		if (rx_xram_ptr >= (u32 *)(ca_uint_t)(priv->rx_xram_end_addr)) {
			rx_xram_ptr = (u32 *)(ca_uint_t)(priv->rx_xram_base_addr);
		}

		/* Checking header XR. Do not update the read pointer yet */
		rx_xram_ptr++;  /* skip unused 32-bit in Header XR */

		header_x = (NI_HEADER_X_T)(*rx_xram_ptr);   /* Header XR [31:0] */

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			if (*rx_xram_ptr == 0xffffffff)	{
				printk("%s: XRAM Error !\n", __func__);
			}
			printk("%s : RX next link 0x%x(%d) \n" ,__func__, header_x.bf.next_link, header_x.bf.next_link);
			printk("%s : bytes_valid 0x%x\n", __func__, header_x.bf.bytes_valid);
		}
		if (header_x.bf.ownership == 0) {

			rx_xram_ptr++;  /* point to Packet status [31:0] */

			packet_status = (NI_PACKET_STATUS_T)(*rx_xram_ptr);

			if (packet_status.bf.valid == 0) {
				printk("%s: Invalid Packet !!\n", __func__);

				/* Update the software read pointer */
				CA_NI_REG_WRITE(header_x.bf.next_link, NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0);
				return 0;
			}

			if (packet_status.bf.drop || packet_status.bf.runt || packet_status.bf.oversize ||
					packet_status.bf.jabber || packet_status.bf.crc_error || packet_status.bf.jumbo) {
				printk("%s: Error Packet !! Packet status=0x%x \n", __func__, packet_status.wrd);

				/* Update the software read pointer */
				CA_NI_REG_WRITE(header_x.bf.next_link, NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0);
				return 0;
			}

			/* Wrap around if required */
			if (rx_xram_ptr >= (u32 *)(ca_uint_t)(priv->rx_xram_end_addr)) {
				rx_xram_ptr = (u32 *)(ca_uint_t)(priv->rx_xram_base_addr);
			}

			rx_xram_ptr++;  /* skip Packet status [31:0] */

			/* Wrap around if required */
			if (rx_xram_ptr >= (u32 *)(ca_uint_t)(priv->rx_xram_end_addr)) {
				rx_xram_ptr = (u32 *)(ca_uint_t)(priv->rx_xram_base_addr);
			}

			pktlen = packet_status.bf.packet_size;

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s : rx packet length=%d\n", __func__, packet_status.bf.packet_size);
			}
			rx_xram_ptr++;  /* point to data address */

			/* Wrap around if required */
			if (rx_xram_ptr >= (u32 *)(ca_uint_t)(priv->rx_xram_end_addr)) {
				rx_xram_ptr = (u32 *)(ca_uint_t)(priv->rx_xram_base_addr);
			}

			rx_dma_addr = virt_to_phys(rx_xram_ptr);

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: rx_xram_ptr[]=\n", __func__);
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, (void *)rx_xram_ptr, pktlen, true);
			}

			data_ptr = (u32 *)pkt_buf;

			/* Read out the packet */
			/* Data is in little endian form in the XRAM */
			/* Send the packet to upper layer */
			for(loop=0; loop<= pktlen/4; loop++) {
				*data_ptr++ = *rx_xram_ptr++;
				/* Wrap around if required */
				if (rx_xram_ptr >= (u32 *)(ca_uint_t)(priv->rx_xram_end_addr)) {
					rx_xram_ptr = (u32 *)(ca_uint_t)(priv->rx_xram_base_addr);
				}
			}

			//ca_ni_netlink_igmpsnooping_reply(0, 1, skb->data, pktlen);

			/* fill skb structure */
#if BITS_PER_LONG > 32
			skb->tail = (unsigned int)(ca_uint_t)(skb->data + pktlen);
#else
			skb->tail = skb->data + pktlen;
#endif
			skb->len = pktlen;
			skb->data_len = 0;   /* skb->data_len will be 0 or greater than 0 (jumbo packet) */

			/* end of skb list */
			skb->next = NULL;
			skb->prev = NULL;

			skb->dev = dev;
			skb->ip_summed = CHECKSUM_NONE;

			/* after eth_type_trans(), skb->data move to begine of IP header */
			skb->protocol = eth_type_trans(skb, skb->dev);

			skb->pkt_type = PACKET_HOST;

			if (ca_ni_debug & NI_DBG_DUMP_RX) {
				printk("%s: skb->data[]=\n", __func__);
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, skb->data, pktlen, true);
			}

			/* Update the software read pointer */
			CA_NI_REG_WRITE(header_x.bf.next_link, NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0);

			ca_skb_out(skb);
			if ((dev->features) & NETIF_F_GRO)
				napi_gro_receive(napi, skb);
			else
				netif_receive_skb(skb);
		}

		/* get the hw write pointer */
		cpuxram_cpu_sta_rx.wrd = CA_NI_REG_READ(NI_HV_XRAM_CPUXRAM_CPU_STA_RX_0);
		hw_rx_wr_ptr = cpuxram_cpu_sta_rx.bf.pkt_wr_ptr;

		/* get the sw read pointer */
		cpuxram_cpu_cfg_rx.wrd = CA_NI_REG_READ(NI_HV_XRAM_CPUXRAM_CPU_CFG_RX_0);
		sw_rx_rd_ptr = cpuxram_cpu_cfg_rx.bf.pkt_rd_ptr;;

		if (ca_ni_debug & NI_DBG_DUMP_RX) {
			printk("%s : RX hw_wr_ptr=%d sw_rd_ptr=%d\n", __func__, hw_rx_wr_ptr, sw_rx_rd_ptr);
		}
		received_pkts++;
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += skb->len;

	}
	return received_pkts;
}

static int ca_ni_rx_get_voq(int cpu_port, u8 index, u8 *voq)
{
	int ret = 0;
	u32 rx_status;
	int tmp_cpu_port = cpu_port;

	*voq = 0;

	/* CPU port 0-3 use QM_QM_CPU_EPP_STATUS0 and 4-7 use QM_QM_CPU_EPP_STATUS1 */
	if (cpu_port < 4) {
		rx_status = aal_l3qm_get_rx_status0();
	}
	else {
		rx_status = aal_l3qm_get_rx_status1();
		tmp_cpu_port = cpu_port - 4;
	}
	if (rx_status & (1 << (index + (tmp_cpu_port * 8)))) {
		*voq =  index;
		ret = 1;
	}

	if (ca_ni_debug & NI_DBG_DUMP_RX) {
		printk("%s: cpu_port=%d, index=%d, *voq=%d\n", __func__, cpu_port, index, *voq);
	}
	return ret;
}

int ca_ni_rx_poll(int cpu_port, struct napi_struct *napi, int budget)
{
	int i;
	int received_pkts = 0;
	int total_received_pkts = 0;
	u8 voq;
	int rest_budget = budget;

	/* get starup config constants */
	ca_ni_rx_get_startup_config();

	if (aal_l3qm_eq_get_cpu_rule() == 0) {
		for (i = 7; i >= 0; i--) {
			if (ca_ni_rx_get_voq(cpu_port, i, &voq)) {
				received_pkts = ca_ni_rx_napi(cpu_port, voq, napi, rest_budget);
				rest_budget -= received_pkts;
				total_received_pkts += received_pkts;
			}
		}
	}
	else {
		for (i = 7; i >= 0; i--) {
			if (ca_ni_rx_get_voq(cpu_port, i, &voq)) {
				received_pkts = ca_ni_rx_napi_rule1(cpu_port, voq, napi, rest_budget);
				rest_budget -= received_pkts;
				total_received_pkts += received_pkts;
			}
		}
	}

	if (total_received_pkts < budget) {

		napi_complete(napi);

		/* do not diable interrupt if we got bad packet */
		//if (ca_ni_rx_get_bad_packet == 0)
		aal_ni_enable_rx_interrupt_by_cpu_port(cpu_port, 1);

		return 0;
	}

	return total_received_pkts;
}

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
ca_uint32_t ni_ff_refill_cnt[8][8];		// per cpu

int ca_ni_refill_fastFwd(u8 cpu_port, u8 eqid, int refill_cnt, dma_swtxq_t *swtxq, u8 voq)
{	//REF API: ca_ni_refill_eq_buf_pool+ca_ni_fill_empty_buffer, ca_dma_tx_complete
	int i;
	u32 phy_addr = 0;
	u32 complete_cnt = 0;
	int push_count = 0;
	dma_rptr_t rptr_reg;
	unsigned int curr_idx;
	//=========== maybe remove below parameter later ==============
	//int free_desc;

	/* [FastForward-ca_ni_fill_empty_buffer] asume aal_l3qm_eq_get_cpu_rule=0, but we only use pool0 */
	/* [FastForward] ignore spin_lock_irqsave, but we should make sure EQ pool should not share by multiple CPU ports */
	/* [FastForward] asume rule=0, ca_ni_use_build_skb=0, ca_ni_use_qm_eq_refill_int=0, aal_l3qm_get_ace_test=1 */
	/* [FastForward] ignore ca_ni_enable_qm_interrupts, ca_ni_do_queue_work due to we not allocate skb here */
	/* [FastForward-ca_dma_tx_complete] ignore cep, xmit_done_cb_fn, ca_dev_kfree_skb_any, netif_queue_stopped, free_desc check */
	/* [FastForward] asume CONFIG_NET_SCH_MULTIQ=0 */
	/* [FastForward] ignore spin_lock_bh, spin_unlock_bh */

	/* push physical address to Empty Buffer */
	for (i = 0; i < refill_cnt; i++) {
		if(complete_cnt == 0) {
			rptr_reg.wrd = aal_ni_get_tx_read_ptr(swtxq->rptr_reg);
			
			complete_cnt = (rptr_reg.bf.rptr >= swtxq->finished_idx) ? (rptr_reg.bf.rptr - swtxq->finished_idx) : (rptr_reg.bf.rptr + swtxq->total_desc_num - swtxq->finished_idx);
		}

		if (rptr_reg.bf.rptr != swtxq->finished_idx) {
			complete_cnt--;
			curr_idx = swtxq->finished_idx;
			if (swtxq->nic_ff_phy_addr[curr_idx] == NULL){
				if (ca_ni_debug & NI_DBG_FASTFWD)
					printk("%s: ignore null tx_skb, rptr=%d, curr_idx=%d\n", __func__, rptr_reg.bf.rptr, curr_idx);
				swtxq->finished_idx = CA_NI_RWPTR_ADVANCE_ONE(swtxq->finished_idx, swtxq->total_desc_num);
				continue;
			}

			phy_addr = virt_to_phys(swtxq->nic_ff_phy_addr[curr_idx]);
			if (aal_l3qm_check_cpu_push_ready(cpu_port)) {	/* check ready bit */
				aal_l3qm_set_cpu_push_paddr(cpu_port, eqid, phy_addr);
				push_count++;

				if (ca_ni_debug & NI_DBG_FASTFWD)
					printk("%s: push to EQ, rptr=%d, curr_idx=%d, phy_addr=0x%x, finished_idx=%d, complete_cnt=%d\n", __func__, rptr_reg.bf.rptr, curr_idx, phy_addr, swtxq->finished_idx+1, complete_cnt);

				swtxq->nic_ff_phy_addr[curr_idx] = NULL;
				swtxq->total_finished++;
				swtxq->finished_idx = CA_NI_RWPTR_ADVANCE_ONE(swtxq->finished_idx, swtxq->total_desc_num);
			} else if (ca_ni_debug & NI_DBG_FASTFWD){
				printk("%s: fill buffer fail at cpu_port[%d] eqid[%d] with phy_addr=0x%x\n", __func__, cpu_port, eqid, phy_addr);
			}

		}

	} /* for loop */

	ni_ff_refill_cnt[cpu_port][voq] -= push_count;
	if (ca_ni_debug & NI_DBG_FASTFWD)
		printk("%s: cpu_port=%d, eqid=%d, refill_cnt=%d, push_count=%d, ni_ff_refill_cnt0=%d\n", __func__, cpu_port, eqid, refill_cnt, push_count, ni_ff_refill_cnt[cpu_port][voq]);
	

	return 0;
}

netdev_tx_t ca_ni_tx_hdr_fastFwd_preAllocate_txBuff_multiCPU_hwLookup_68bytes(dma_swtxq_t *swtxq, u8 *phy_addr, int head_room)
{	//REF API: __ca_ni_start_xmit
	char *pkt_datap;
	dma_txdesc_t *curr_desc;
	int desp_no;
	int j;
	//=========== maybe remove below parameter later ==============
	dma_txbuf_fastfwd_t *tx_buf;
	for(desp_no = 0; desp_no < 2; desp_no++) {	//0: only header, 1: only one skb
		curr_desc = swtxq->desc_base + swtxq->wptr;
		//curr_desc->word1.wrd = curr_desc->word0.wrd = 0;

		if (desp_no == 0) {		
			/*
				Value of 1st descriptor word0, word1, lso word0, lso word1 should be set when add special fastforward flow 
				Reference: ca_ni_init_tx_dma_lso_special_fast_fwd
			*/
		
			
			if (ca_ni_debug & NI_DBG_FASTFWD)
			{
				// for debug
				printk("phys_to_virt(curr_desc->word0.wrd) = %p\n",phys_to_virt(curr_desc->word0.wrd));
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1,
								phys_to_virt(curr_desc->word0.wrd), sizeof(dma_lsopara_0_t) + sizeof(dma_lsopara_1_t), true);

			}

			swtxq->nic_ff_phy_addr[swtxq->wptr] = NULL;

		} else {	//desp_no == 1
			/* 
				Value of 2nd descriptor word0, word1, lso word0, lso word1 should be set when add special fastforward flow 
				In here only need to set descriptor word0 for inner packet physical address.
				Reference: ca_ni_init_tx_dma_lso_special_fast_fwd
			*/
			pkt_datap = (char *)(phy_addr + head_room + sizeof(HEADER_A_T)+ sizeof(HEADER_CPU_T));
			curr_desc->word0.wrd = virt_to_phys(pkt_datap);
			swtxq->nic_ff_phy_addr[swtxq->wptr] = phy_addr;

		}

		swtxq->wptr = CA_NI_RWPTR_ADVANCE_ONE(swtxq->wptr, swtxq->total_desc_num);
		//printk("%s:swtxq->wptr=%d\n", __func__, swtxq->wptr);
	}

	return NETDEV_TX_OK;
}

netdev_tx_t ca_ni_tx_hdr_fastFwd_preAllocate_txBuff_multiCPU_hwLookup_others(dma_swtxq_t *swtxq, u8 *phy_addr, int ori_pkt_len, int head_room)
{	//REF API: __ca_ni_start_xmit
	char *pkt_datap;
	dma_txdesc_t *curr_desc;
	int desp_no;
	//=========== maybe remove below parameter later ==============
	dma_txbuf_fastfwd_t *tx_buf;
	for(desp_no = 0; desp_no < 2; desp_no++) {	//0: only header, 1: only one skb
		curr_desc = swtxq->desc_base + swtxq->wptr;
		curr_desc->word1.wrd = curr_desc->word0.wrd = 0;
	
		if (ca_ni_debug & NI_DBG_FASTFWD)
			printk("%s: desp_no=%d, curr_desc=%p, swtxq->wptr=%d ori_pkt_len = %d\n", __func__, desp_no, curr_desc, swtxq->wptr, ori_pkt_len);

		if (desp_no == 0) {

			curr_desc->word1.bf.sop_eop |= CA_NI_DMA_LSO_SOF; 
			curr_desc->word1.bf.hp1 = 0;	
			curr_desc->word1.bf.hp0 = 0;    
			curr_desc->word1.bf.buf_len = 50 + sizeof(dma_lsopara_0_t) + sizeof(dma_lsopara_1_t);
			
			tx_buf = ((dma_txbuf_fastfwd_t *)swtxq->buf_base + swtxq->wptr);

			tx_buf->lso_word1.bf.packet_size = ori_pkt_len+50/*outer header len*/;
			tx_buf->lso_word0.bf.udp_en = 1;
			tx_buf->lso_word0.bf.ipv4_en = 1;

			
			curr_desc->word0.wrd = swtxq->buf_dma_addr + (swtxq->wptr * sizeof(dma_txbuf_fastfwd_t));
			tx_buf = ((dma_txbuf_fastfwd_t *)swtxq->buf_base + swtxq->wptr);
			*( (u16*)(&tx_buf->pkthdr[0] + 16)) = htons(ori_pkt_len + 20/*iph len*/ + 8/*udp len*/ + 8/*vxlan hdrlen*/);
			*( (u16*)(&tx_buf->pkthdr[0] + 38)) = htons(ori_pkt_len + 8/*udp len*/ + 8/*vxlan hdrlen*/);
			tx_buf->lso_word1.bf.packet_size = ori_pkt_len+50;

			
			swtxq->nic_ff_phy_addr[swtxq->wptr] = NULL;

		} else {	//desp_no == 1

			curr_desc->word1.bf.sop_eop |= CA_NI_DMA_LSO_EOF; /* EOF */
			curr_desc->word1.bf.buf_addr_39_32 = 2;	//dma_lso_ace_test=1
			curr_desc->word1.bf.buf_len = ori_pkt_len;
			
			pkt_datap = (char *)(phy_addr + head_room + sizeof(HEADER_A_T)+ sizeof(HEADER_CPU_T));
			curr_desc->word0.wrd = virt_to_phys(pkt_datap);
			swtxq->nic_ff_phy_addr[swtxq->wptr] = phy_addr;


		}

		swtxq->wptr = CA_NI_RWPTR_ADVANCE_ONE(swtxq->wptr, swtxq->total_desc_num);
		//printk("%s:swtxq->wptr=%d\n", __func__, swtxq->wptr);
	}

	return NETDEV_TX_OK;
}


netdev_tx_t ca_ni_tx_fastFwd(dma_swtxq_t *swtxq, u8 *phy_addr, int data_offset, int len)
{	//REF API: ca_ni_start_xmit_buf
	char *pkt_datap;
	dma_txdesc_t *curr_desc;
	//=========== maybe remove below parameter later ==============
	dma_txbuf_t	*tx_buf;

	curr_desc = swtxq->desc_base + swtxq->wptr;
	curr_desc->word1.wrd = 0;
	curr_desc->word1.bf.sop_eop = CA_NI_DMA_LSO_SOF|CA_NI_DMA_LSO_EOF;
	curr_desc->word1.bf.buf_addr_39_32 = 2;	//dma_lso_ace_test=1

	
	curr_desc->word1.bf.hp1 = 1;	/* HP[1:0]=11 - lso_parameters, header_a and header_cpu not exist */
	curr_desc->word1.bf.hp0 = 1;
	pkt_datap = (char *)(phy_addr + data_offset);
	curr_desc->word1.bf.buf_len = len;
	if (ca_ni_debug & NI_DBG_FASTFWD)
		printk("%s: hardware lookup without any header! pkt len = %d\n", __func__, len);

	curr_desc->word0.wrd = virt_to_phys(pkt_datap);

	if (ca_ni_debug & NI_DBG_FASTFWD) {
		printk("%s: buf=%p, buf_len=%d(len=%d), tx_desc0.wrd=0x%x, tx_desc1.wrd=0x%x\n", __func__, pkt_datap, curr_desc->word1.bf.buf_len, len, curr_desc->word0.wrd, curr_desc->word1.wrd);
		ca_ni_dump_tx_pkts((struct sk_buff *)(*((ca_uint_t *)(phy_addr))));
	}

	swtxq->nic_ff_phy_addr[swtxq->wptr] = phy_addr;
	swtxq->wptr = CA_NI_RWPTR_ADVANCE_ONE(swtxq->wptr, swtxq->total_desc_num);

	return NETDEV_TX_OK;
}


int ca_ni_rx_fastFwd(int cpu_port, u8 voq, struct napi_struct *napi, int budget)
{	//REF API: ca_ni_rx_napi
	u8 eqid;
	u8 *rx_epp_virt_addr_start;
	u8 *rx_virt_addr;
	u16 count;
	ca_uint32_t hw_wr_ptr, hw_rd_ptr;
	CPU_EPP_FIFO_CMD_T epp_fifo_cmd;
	dma_swtxq_t *swtxq;
	HEADER_A_T hdr_a_tx;
	int received_pkts = 0, header_size = 0;
	ca_uint16_t pkt_len = 0;
	ca_uint8_t direction = 0 ;
	int i = 0, needReadHdrA = 0;
	int head_room = aal_l3qm_get_cpu_port_head_room_first();
	HEADER_A_T hdr_a;


	hw_wr_ptr = aal_l3qm_special_fastFwd_get_rx_read_write_ptr(cpu_port, voq, &count, &hw_rd_ptr);
	
	if(hw_wr_ptr - hw_rd_ptr ==0)
	{
		goto error_stop;
	}
	/*
		Use voq to check packet size, for better performance when testing 68 bytes packet
		So should add corresponding l3 cls for this purpose. 
		For cpu port 4-7:

		voq 3: down stream 68 bytes 
		voq 2: up stream 68 bytes
		voq 1: down stream other length
		voq 0: up stream other length
		
	*/

	
	if(voq == NI_FASTFWD_QUEUE3) // down stream 68 bytes
	{
		direction = NI_FASTFWD_DOWNSTREAM;
	}
	else if(voq == NI_FASTFWD_QUEUE2) // up stream 68 bytes
	{
		direction = NI_FASTFWD_UPSTREAM;
	}
	else if(voq == NI_FASTFWD_QUEUE1)
	{
		needReadHdrA = 1;
		direction = NI_FASTFWD_DOWNSTREAM;
	}
	else if(voq == NI_FASTFWD_QUEUE0)
	{
		needReadHdrA = 1;
		direction = NI_FASTFWD_UPSTREAM;
	}
	
	rx_epp_virt_addr_start = phys_to_virt(aal_l3qm_get_rx_start_addr(cpu_port, voq));
	swtxq = &(ni_info_data.swtxq[cpu_port][voq]);
		
	/* loop until all of packets available are recived or budget is reached */
	while ((received_pkts < count) && (received_pkts < budget)) {

		/* get physicall address from EPP_CMD, and get packet virtual address */
		epp_fifo_cmd = *(CPU_EPP_FIFO_CMD_T *)(rx_epp_virt_addr_start + hw_rd_ptr);

		if ((epp_fifo_cmd.bits.sop != 1) || (epp_fifo_cmd.bits.eop != 1)) {
			printk("%s: [NIC_FF] nic fast forward not support multiple packet!(sop %d, eop %d)\n", __func__, epp_fifo_cmd.bits.sop, epp_fifo_cmd.bits.eop);
			goto error_stop;
		}

		rx_virt_addr = phys_to_virt(epp_fifo_cmd.bits.phy_addr << CA_L3QM_PHY_ADDR_SHIFT);
		eqid = epp_fifo_cmd.bits.eqid;


		/* ready to get next CPU EPP FIFO, and care wrap around */
		hw_rd_ptr += EPP_DESC_SIZE;
		if (hw_rd_ptr >= aal_l3qm_eq_get_max_write_ptr())
			hw_rd_ptr = 0;

		if(needReadHdrA==0)
		{
		
			if(voq == NI_FASTFWD_QUEUE3 ) // 68bytes downstream
			{
				pkt_len = rtkScfg.smallest_packet_size+down_stream_inner_extra_len;//64 + inner extra len (e.g. inner packet maybe with vlan)
				header_size = head_room + sizeof(HEADER_A_T)+ sizeof(HEADER_CPU_T)+50;
			}
			else if(voq == NI_FASTFWD_QUEUE2 ) // 68bytes upstream
			{ 
				pkt_len = rtkScfg.smallest_packet_size+up_stream_inner_extra_len; //64 +inner extra len (e.g. inner packet maybe with vlan)
			}
		}
		else if(needReadHdrA==1)
		{
			hdr_a = (*(HEADER_A_T *)(rx_virt_addr + head_room));
			hdr_a.bits32.bits32_h = be32_to_cpu(hdr_a.bits32.bits32_h);
			hdr_a.bits32.bits32_l = be32_to_cpu(hdr_a.bits32.bits32_l);
			pkt_len = hdr_a.bits.pkt_size - sizeof(HEADER_CPU_T); // 124
			
			if(direction == NI_FASTFWD_DOWNSTREAM)
			{
				pkt_len -= 50; // Because when downstream packet in, outer header always pop vlan, pppoe, 
				               // so outer header only left 50 bytes (outer l2 + outer iph + outer udph + outer vxlanHdr)
				header_size = head_room + sizeof(HEADER_A_T)+ sizeof(HEADER_CPU_T)+50;
			}
			
		}


		
		if (ca_ni_debug & NI_DBG_FASTFWD)
		{
			printk("%s: cpuPort = %d voq = %d needReadHdrA = %d pkt_len = %d up_stream_inner_extra_len = %d down_stream_inner_extra_len = %d\n",
				__func__, cpu_port, voq, needReadHdrA, pkt_len, up_stream_inner_extra_len, down_stream_inner_extra_len );
		}

		if(direction == NI_FASTFWD_DOWNSTREAM)	//hw lookup: tx without hdr_a/lso_par
			ca_ni_tx_fastFwd(swtxq, rx_virt_addr, header_size, pkt_len);
		else if(direction ==NI_FASTFWD_UPSTREAM && voq ==NI_FASTFWD_QUEUE2 ) // 68bytes tx
			ca_ni_tx_hdr_fastFwd_preAllocate_txBuff_multiCPU_hwLookup_68bytes(swtxq, rx_virt_addr, head_room);
		else if(direction ==NI_FASTFWD_UPSTREAM )
			ca_ni_tx_hdr_fastFwd_preAllocate_txBuff_multiCPU_hwLookup_others(swtxq, rx_virt_addr, pkt_len, head_room);


		swtxq->intr_cnt++;
		smp_wmb();
		if (ca_ni_debug & NI_DBG_FASTFWD)
			printk("%s: swtxq->rptr_reg=0x%x, swtxq->wptr_reg=0x%x, next swtxq->wptr=%d\n", __func__, swtxq->rptr_reg, swtxq->wptr_reg, swtxq->wptr);

		ni_ff_refill_cnt[cpu_port][voq]++;


		if(((received_pkts < (count-1)) && (received_pkts % rtkScfg.special_fastFwd_refill_batch_number ==0)) || (received_pkts == (count-1))) {
			aal_l3qm_set_rx_read_ptr(cpu_port, voq, hw_rd_ptr);
			aal_ni_set_tx_write_ptr(swtxq->wptr_reg, swtxq->wptr);

			/* refill EQ pool after tx finish */
			if (ni_ff_refill_cnt[cpu_port][voq] != 0)
				ca_ni_refill_fastFwd(cpu_port, eqid, ni_ff_refill_cnt[cpu_port][voq], swtxq, voq);
		}
		
		received_pkts++;
	}

error_stop:
		
	return received_pkts;
}

int ca_ni_rx_poll_fastFwd(int cpu_port, struct napi_struct *napi, int budget)
{	//REF API: ca_ni_rx_poll
	int received_pkts = 0;
	int total_received_pkts = 0;
	int rest_budget = budget;
	int i = 0, still_have_packet = 0;;

	for (i = NI_FASTFWD_QUEUE3; i >= NI_FASTFWD_QUEUE0; i--) {
		received_pkts = ca_ni_rx_fastFwd(cpu_port, i, napi, NI_FASTFWD_BUDGET);
		if(received_pkts == NI_FASTFWD_BUDGET)
			still_have_packet =1;
		rest_budget -= received_pkts;
		total_received_pkts += received_pkts;
	}

	//printk("total_received_pkts = %d still_have_packet = %d\n", total_received_pkts, still_have_packet);

	if ( (total_received_pkts < budget) && still_have_packet==0) {
		napi_complete(napi);
		aal_ni_enable_rx_interrupt_by_cpu_port(cpu_port, 1);
		return 0;
	}

	return total_received_pkts;
}
#endif	//CONFIG_FC_SPECIAL_FAST_FORWARD

