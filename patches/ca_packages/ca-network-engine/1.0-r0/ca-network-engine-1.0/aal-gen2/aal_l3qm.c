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
 * aal_l3qm.c: Hardware Abstraction Layer for Layer-3 Queue Manager to access hardware regsiters
 *
 */

#include <generated/ca_ne_autoconf.h>
#include <linux/sched.h>

#include "scfg.h"
#include "osal_cmn.h"
#include "aal_common.h"
#include "aal_table.h"
#include "aal_fbm.h"
#include "aal_l3qm.h"
#include "ca_aal_proc.h"

#if defined(CONFIG_NE_CPU256)
#include "aal_l3qm_cpu256.h"
#endif
#if defined(CONFIG_LUNA_G3_SERIES)
#include "aal_l3_te_cb.h"
#endif

#define CA_L3QM_MAGIC_NUMBER		0xDEADBEEF

#define CA_L3QM_AXI_TOP_DDR_NONCACHE_BIT		1
#define CA_L3QM_AXI_TOP_DDR_COHERENT_BIT	 	4

#define CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT		300

#define CA_L3QM_AXI_ATTRIBUTE_EQ_BASE		0		/* EQ0-15 */

#define CA_L3QM_AXI_ATTRIBUTE_CPU256_BASE	16		/* CPU256 port 0-31 */
#define CA_L3QM_AXI_ATTRIBUTE_CPU_BASE		48		/* CPU port 0-7 */

#define CA_L3QM_CPU_EPP_FIFO_SIZE_UINT		4		/* 4-FIFO entries */

#if defined(CONFIG_LUNA_G3_SERIES)
rtk_scfg_t rtkScfg;

int DQ_bid_start=0;											// CPU port used BID count, also means the number NI port BID start.

#define	EQ_POOL_SRAM_SIZE	(0x8000)					
#define	EQ_POOL_DDR_SIZE	(0x100000)

#else
#define	EQ_POOL_SRAM_SIZE	(0x8000)
#define	EQ_POOL_DDR_SIZE	(0x300000)
#endif

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
rtk_l3qm_specFastFwd_t l3qm_ff_cpuCfg[MAX_FF_CPU_COUNT];
#endif

/* user defined constants should get from startup configs */
ca_uint8_t	l3qm_magic_marker_len = 0;			/* length of magic marker */
ca_uint8_t	l3qm_desc_size = 4;  				/* size of RX descriptor */
ca_uint8_t	l3qm_desc_per_epp = 2;				/* number of RX descriptor per EPP */
ca_uint32_t	l3qm_pe0_1_port_count = 1;			/* PE0/PE1 port count */
ca_uint8_t	l3qm_pe0_1_voq_per_port = 8;			/* PE0/PE1 VoQ per port */
ca_uint32_t	l3qm_pe0_1_epp_per_voq = 32;			/* PE0/PE1 EPP per VoQ */
ca_uint32_t	l3qm_pe0_1_pool_bid_count = 128;		/* PE0/PE1 pool BID count */
#if defined(CONFIG_LUNA_G3_SERIES)
ca_uint8_t	l3qm_eq0_sram_eq1_dram = 1;			/* EQ0 SRAM and EQ1 DRAM Mode */
ca_uint8_t	l3qm_eq0_eq1_all_dram = 0;				/* Force EQ0 and EQ1 share DRAM buffers */

uint32_t		sram_dts_conf_size = 0;					/* HW support 256KB - dtsi define 240KB are avaliable for NE */
uint32_t		sram_cur_used_size = 0;					/* init as BL2 reserved size */
ca_uint32_t 	epp64_paddr_start[8][8] = {0};
#else
ca_uint8_t	l3qm_eq0_sram_eq1_dram = 0;			/* EQ0 SRAM and EQ1 DRAM Mode*/
#endif
ca_uint8_t	l3qm_eq_share_pool_id = 1;			/* EQ0 SRAM and EQ1 DRAM Mode*/

ca_uint32_t     l3qm_cpu_port_count = 6;                   	/* CPU port count */
ca_uint8_t      l3qm_cpu_voq_per_port = 8;                      /* CPU VoQ per port */
ca_uint32_t     l3qm_cpu_epp_per_voq = 128;                     /* CPU EPP per VoQ */
ca_uint32_t     l3qm_cpu_pool_bid_count = 1500;                 /* CPU pool BID count */
ca_uint8_t	l3qm_cpu_cache_line_size = 64;			/* CPU cache line size */
ca_uint32_t	l3qm_eq_buffer_align_size = 128;		/* EQ buffer align size */
ca_uint8_t	l3qm_pe_buffer_align_size = 4;			/* PE buffer align size */
ca_uint32_t	l3qm_cci0_start_addr = 0x00000000;		/* CCI0 start address */
ca_uint32_t	l3qm_cci1_start_addr = 0x00000000;		/* CCI1 start address */
ca_uint32_t	l3qm_sram_start_addr = 0xc0000000;		/* SRAM start address */
ca_uint32_t	l3qm_ddr_cache_coherent_start_addr = 0x00000000;	/* DDR cache coherent start address */
ca_uint32_t	l3qm_ddr_non_cacheable_start_addr = 0x80000000;		/* DDR non-cacheable start address */
ca_uint8_t	l3qm_eq_axi_top8_bits = 4;			/* 0x400000000, in DDR cache coherent */
ca_uint8_t	l3qm_epp_axi_top8_bits = 4;			/* 0x400000000, in DDR cache coherent */
ca_uint8_t	l3qm_deepq_test_with_sram = 0;			/* use sram to test DEEPQ function */
ca_uint8_t	l3qm_ace_test = 0;				/* do QM ACE test do not do dma sync by software */
ca_uint8_t	l3qm_main_hash_bm_test = 0;			/* do Main Hash Bench Mark test */

/* caculated from defined constants */
ca_uint8_t      l3qm_epp_size = 0;      /* EPP size in byte */

ca_uint32_t     l3qm_eq_profile_pe0_1_total_epp_count = 0;      /* PE0/PE1 total EPP count */

ca_uint32_t     l3qm_epp_profile_cpu_total_epp_count = 0;       /* CPU total EPP count */

ca_uint32_t	l3qm_cpu_port_head_room_first = 32;		/* head room first for CPU port */

/* EQ(QM_CFG3/4_EQ%d) startup configures */
ca_uint32_t	l3qm_eq_cfg3_values[CFG_ID_L3QM_EQ_CFG3_VALUES_LEN] =
{
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1a008017, 0x1a008017, 0x0, 0x0, 0x0, 0x0, 0x10, 0x10
};

ca_uint32_t     l3qm_eq_cfg4_values[CFG_ID_L3QM_EQ_CFG4_VALUES_LEN] =
{
	0x0, 0x1, 0x0, 0x1, 0x0, 0x1, 0x1, 0x1, 0x4, 0x4, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1
};

/* QM_AXI_ATTRIBUTE for EQ 0-15 */
ca_uint32_t	l3qm_axi_attrib_eq_values[CFG_ID_L3QM_AXI_ATTRIB_EQ_VALUES_LEN] =
{
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x04000010, 0x04000010
};

/* QM_AXI_ATTRIBUTE for CPU EPP */
ca_uint32_t     l3qm_axi_attrib_cpu_epp_values[CFG_ID_L3QM_AXI_ATTRIB_CPU_EPP_VALUES_LEN] =
{
	0x12008060, 0x12008060, 0x12008060, 0x12008060, 0x12008060, 0x12008060, 0x12008060, 0x12008060
};

/* configuration parameters for PE0 need to be got from start config */
ca_uint8_t      l3qm_eq_profile_pe0_id = 0;                     /* EQ profile 0 */
ca_uint8_t      l3qm_eq_profile_pe0_rule = 1;                   /* use pool 0 for header */
ca_uint8_t      l3qm_eq_profile_pe0_use_fbm = 1;                /* use FBM pool */
ca_uint8_t      l3qm_eq_profile_pe0_pool0_eq_id = 1;            /* QM_EQ_PROFILE0.eqid0=1 */
ca_uint32_t     l3qm_eq_profile_pe0_pool0_bid_start = 0;        /* strating buffer id (must start from 0) */
ca_uint8_t      l3qm_eq_profile_pe0_pool0_fbm_pool_id = 0;      /* in SRAM */
ca_uint32_t	l3qm_eq_profile_pe0_pool0_physical_start_address = 0xffffffff;	/* pool0 physical start address */
ca_uint8_t      l3qm_eq_profile_pe0_pool1_eq_id = 2;            /* QM_EQ_PROFILE1.eqid0=2 */
ca_uint32_t     l3qm_eq_profile_pe0_pool1_buf_count = 128;        /* number of  buffers for PE0 pool 1 */
ca_uint32_t     l3qm_eq_profile_pe0_pool1_bid_start = 128;       /* starting buffer id */
ca_uint32_t     l3qm_eq_profile_pe0_pool1_buf_sz = 2048;        /* pool 1 buffer size(payload) */
ca_uint8_t      l3qm_eq_profile_pe0_pool1_fbm_pool_id = 1;      /* in DDR */
ca_uint32_t	l3qm_eq_profile_pe0_pool1_physical_start_address = 0xffffffff;	/* pool1 physical start address */
ca_uint8_t      l3qm_epp_profile_pe0_id = 0;                    /* EPP (RX FIFO) profile id */
ca_uint32_t     l3qm_epp_profile_pe0_sz = 0;                    /* RX FIFO size per entry, 4B */
ca_uint32_t     l3qm_epp_profile_pe0_cn = 0;	                /* 8 queues, 64 entries per queue */
ca_uint32_t     l3qm_epp_profile_pe0_start_addresses[8] = {0xffffffff};       /* VoQs 0-7 for PE0  */
ca_uint32_t	l3qm_epp_profile_pe0_map_mode = 0;		/* QM_CPU_EPP0_CFG map_mode=0 */

/* configuration parameters for PE1 need to be got from start config */
ca_uint8_t      l3qm_eq_profile_pe1_id = 1;                     /* EQ profile 1 */
ca_uint8_t      l3qm_eq_profile_pe1_rule = 1;                   /* use pool 0 for header */
ca_uint8_t      l3qm_eq_profile_pe1_use_fbm = 1;                /* use FBM pool */
ca_uint8_t      l3qm_eq_profile_pe1_pool0_eq_id = 3;            /* QM_EQ_PROFILE1.eqid0=3 */
ca_uint32_t     l3qm_eq_profile_pe1_pool0_bid_start = 256;      /* strating buffer id */
ca_uint8_t      l3qm_eq_profile_pe1_pool0_fbm_pool_id = 1;      /* in SRAM */
ca_uint32_t	l3qm_eq_profile_pe1_pool0_physical_start_address = 0xffffffff;	/* pool0 physical start address */
ca_uint8_t      l3qm_eq_profile_pe1_pool1_eq_id = 4;            /* QM_EQ_PROFILE1.eqid1=4 */
ca_uint32_t     l3qm_eq_profile_pe1_pool1_buf_count = 128;        /* 64 buffers for PE1 pool 1 */
ca_uint32_t     l3qm_eq_profile_pe1_pool1_bid_start = 384;   /* starting buffer id */
ca_uint32_t     l3qm_eq_profile_pe1_pool1_buf_sz = 2048;        /* pool 1 buffer size(payload) */
ca_uint8_t      l3qm_eq_profile_pe1_pool1_fbm_pool_id = 0;      /* in DDR */
ca_uint32_t	l3qm_eq_profile_pe1_pool1_physical_start_address = 0xffffffff;	/* pool1 physical start address */
ca_uint8_t      l3qm_epp_profile_pe1_id = 1;                    /* EPP (RX FIFO) profile id */
ca_uint32_t     l3qm_epp_profile_pe1_sz = 0;                    /* RX FIFO size per entry, 4B */
ca_uint32_t     l3qm_epp_profile_pe1_cn = 0;                    /* 8 queues, 64 entries per queue */
ca_uint32_t     l3qm_epp_profile_pe1_start_addresses[8] = {0xffffffff};       /* VoQs 0-7 for PE1  */
ca_uint32_t     l3qm_epp_profile_pe1_map_mode = 0;              /* QM_CPU_EPP1_CFG map_mode=0 */

/* configuration parameters for CPU need to be got from start config */
ca_uint8_t      l3qm_eq_profile_cpu_id = 2;                     /* EQ profile 2 */
ca_uint8_t      l3qm_eq_profile_cpu_rule = 0;                   /* use pool 0 then pool 1 */
ca_uint8_t      l3qm_eq_profile_cpu_use_fbm = 0;                /* do not use FBM pool */
ca_uint8_t      l3qm_eq_profile_cpu_pool0_eq_id = 5;            /* QM_EQ_PROFILE2.eqid0=5 */
ca_uint32_t     l3qm_eq_profile_cpu_pool0_buf_count = 1500;        /* 1024 buffers for CPU pool 0 */
ca_uint32_t     l3qm_eq_profile_cpu_pool0_bid_start = 4608;     /* strating buffer id */
ca_uint32_t     l3qm_eq_profile_cpu_pool0_buf_sz = 2048;        /* pool 0 buffer size */
ca_uint8_t      l3qm_eq_profile_cpu_pool0_fbm_pool_id = 0;      /* SRAM */
ca_uint32_t	l3qm_eq_profile_cpu_pool0_physical_start_address = 0xffffffff;	/* pool0 physical start address */
ca_uint8_t      l3qm_eq_profile_cpu_pool1_eq_id = 6;            /* QM_EQ_PROFILE2.eqid0=6 */
ca_uint32_t     l3qm_eq_profile_cpu_pool1_buf_count = 1500;        /* 2048 buffers for CPU pool 1 */
ca_uint32_t     l3qm_eq_profile_cpu_pool1_bid_start = 6108;     /* starting buffer id */
ca_uint32_t     l3qm_eq_profile_cpu_pool1_buf_sz = 2048;        /* pool 1 buffer size */
ca_uint8_t      l3qm_eq_profile_cpu_pool1_fbm_pool_id = 0;      /* SRAM */
ca_uint32_t	l3qm_eq_profile_cpu_pool1_physical_start_address = 0xffffffff;	/* pool1 physical start address */
ca_uint8_t      l3qm_epp_profile_cpu_id = 2;                    		/* EPP (RX FIFO) profile id */

ca_uint32_t     l3qm_epp_profile_cpu_sz = 0;                    /* RX FIFO size per entry, 8B */
ca_uint32_t     l3qm_epp_profile_cpu_cn = 256 * 8 * 6;          /* CPU ports 0-5, 6 ports, 8 queues per port, 256 entries per queue */

ca_uint32_t     l3qm_epp_profile_cpu_map_mode = 0;              /* QM_CPU_EPP1_CFG map_mode=0 */

/* configuration parameters for Deep Queue need to be got from start config, only use pool0 and no EPP profile */
ca_uint8_t      l3qm_eq_profile_dq_id = 3;                     /* EQ profile 3 */
ca_uint8_t      l3qm_eq_profile_dq_rule = 0;                   /* use pool 0 then pool 1 */
ca_uint8_t      l3qm_eq_profile_dq_pool0_eq_id = 8;            /* QM_EQ_PROFILE3.eqid0=0, eqid1=0 */
ca_uint32_t     l3qm_eq_profile_dq_pool0_buf_count = 4096;        /* 8192 buffers for DQ pool 0 */
ca_uint32_t     l3qm_eq_profile_dq_pool0_bid_start = 7608;     /* strating buffer id */
ca_uint32_t     l3qm_eq_profile_dq_pool0_buf_sz = 2048;        /* pool 0 buffer size */
ca_uint32_t     l3qm_eq_profile_dq_pool0_start_physical_addr = 0xffffffff;  	/* QM_CFG0_EQ0.phy_addr_start = DDR addess of DQ EQ pool */
ca_uint8_t      l3qm_eq_profile_dq_pool0_fbm_pool_id = 7;      /* DDR (pushed into FBM) */
ca_uint8_t      l3qm_eq_profile_dq_pool1_eq_id = 9;            /* QM_EQ_PROFILE3.eqid0=0, eqid1=0 */
ca_uint32_t     l3qm_eq_profile_dq_pool1_buf_count = 4096;        /* 8192 buffers for DQ pool 0 */
ca_uint32_t     l3qm_eq_profile_dq_pool1_bid_start = 11704;    /* strating buffer id */
ca_uint32_t     l3qm_eq_profile_dq_pool1_buf_sz = 2048;        /* pool 0 buffer size */
ca_uint32_t     l3qm_eq_profile_dq_pool1_start_physical_addr = 0xffffffff;      /* QM_CFG0_EQ0.phy_addr_start = DDR addess of DQ EQ pool */
ca_uint8_t      l3qm_eq_profile_dq_pool1_fbm_pool_id = 7;      /* DDR (pushed into FBM) */

/* configuration parameters for main hash table need to be got from start config */
ca_uint32_t	l3fe_main_hash_overflow_hash_fib_length	= 32;		/* FIB length of main hash overflow hash table */
ca_uint32_t	l3fe_main_hash_overflow_hash_fib_count	= 32;		/* number of FIB of main hash overflow hash table */
ca_uint32_t	l3fe_main_hash_default_hash_fib_length	= 32;		/* FIB length of main hash default hash table */
ca_uint32_t	l3fe_main_hash_default_hash_fib_count	= 32;		/* number of FIB of main hash default hash table */
ca_uint32_t	l3fe_main_hash_action_cache_fib_length	= 32;		/* FIB length of main hash action cache table */
ca_uint32_t	l3fe_main_hash_action_cache_fib_count	= 2048;		/* number of FIB of main hash action cache table */
ca_uint32_t	l3fe_main_hash_table_entry_length	= 4;		/* length of main hash table entry */
ca_uint32_t	l3fe_main_hash_table_entry_count	= 65536;	/* count of main hash table entry */
ca_uint32_t	l3fe_main_hash_action_table_entry_length= 32;		/* length of main hash action table entry */
ca_uint32_t	l3fe_main_hash_action_table_entry_count = 65536;	/* count of main hash action table entry */

/* SRAM/DRR addresses calculated from startup config */
/* virtual address should be 64-bit long */
/* SRAM/DRR addresses calculated from startup config */
/* virtual addresses in DDR non-cache area should be 64-bit long */
ca_uint_t     l3qm_eq_profile_dq_pool0_buff_start_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_dq_pool1_buff_start_addr = 0xffffffff;
ca_uint_t     l3fe_main_hash_table_start_addr = 0xffffffff;
ca_uint_t     l3fe_main_action_table_start_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_pe0_pool1_buff_start_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_pe1_pool1_buff_start_addr = 0xffffffff;

/* physical addresses in DDR non-cache area should be 32-bit long */
ca_uint32_t     l3qm_eq_profile_dq_pool0_buff_phy_addr = 0xffffffff;
ca_uint32_t     l3qm_eq_profile_dq_pool1_buff_phy_addr = 0xffffffff;
ca_uint32_t     l3fe_main_hash_table_phy_addr = 0xffffffff;
ca_uint32_t     l3fe_main_action_table_phy_addr = 0xffffffff;
ca_uint32_t     l3qm_eq_profile_pe0_pool1_buff_phy_addr = 0xffffffff;
ca_uint32_t     l3qm_eq_profile_pe1_pool1_buff_phy_addr = 0xffffffff;

/* virtual addresses in DDR cache coherent area should be 64-bit long */
ca_uint_t     l3qm_eq_profile_cpu_pool0_start_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_cpu_pool1_start_addr = 0xffffffff;
ca_uint_t     l3qm_epp_profile_cpu_rx_fifo_start_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_cpu_pool0_phy_addr = 0xffffffff;
ca_uint_t     l3qm_eq_profile_cpu_pool1_phy_addr = 0xffffffff;
ca_uint_t     l3qm_epp_profile_cpu_rx_fifo_phy_addr = 0xffffffff;

/* virtual addresses in SRAM area should be 64-bit long */
ca_uint_t     l3fe_main_hash_overflow_hash_fib_start_addr = 0xffffffff;
ca_uint_t     l3fe_main_hash_default_hash_fib_start_addr = 0xffffffff;
ca_uint_t     l3fe_main_hash_action_cache_fib_start_addr = 0xffffffff;

/* physical address in SRAM area should be 32-bit long */
ca_uint32_t     l3fe_main_hash_overflow_hash_fib_phy_addr = 0xffffffff;
ca_uint32_t     l3fe_main_hash_default_hash_fib_phy_addr = 0xffffffff;
ca_uint32_t     l3fe_main_hash_action_cache_fib_phy_addr = 0xffffffff;

spinlock_t l3qm_epp64_inte_lock;

#if defined(CONFIG_NE_CPU256)
ca_uint8_t support_cpu256_in_linux = 0;
#endif

ca_uint8_t l3qm_jumbo_buf_packing_size = 0;
ca_uint8_t l3qm_jumbo_buf_pingpong_eqs = 0;
ca_uint8_t l3qm_ldpid_to_pon_port_map_offset = 0;

void aal_l3qm_init_jumbo_buf(void)
{
	ca_uint32_t reg_off;
	QM_QM_BUFFER_CNTL_t buf_cntl;
	QM_QM_EQ_PROFILE0_t eq_profile;

	buf_cntl.wrd = CA_NE_REG_READ(QM_QM_BUFFER_CNTL);
	buf_cntl.bf.jumbo_buffer_en = l3qm_jumbo_buf_packing_size;
	buf_cntl.bf.alternate_eq_en = l3qm_jumbo_buf_pingpong_eqs;
	CA_NE_REG_WRITE(buf_cntl.wrd, QM_QM_BUFFER_CNTL);

	/* when alternate EQ is enabled DeepQ should be rule=1 */
	if (buf_cntl.bf.alternate_eq_en) {
		reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * l3qm_eq_profile_dq_id);
		eq_profile.wrd = CA_NE_REG_READ(reg_off);
		eq_profile.bf.rule = 1;
		CA_NE_REG_WRITE(eq_profile.wrd, reg_off);
		aal_l3qm_load_eq_config();
	}
}

void aal_l3qm_init_upper_ldpid_map(void)
{
	QM_QM_UPPER_LDPID_MAP_t	upper_ldpid_map;

	upper_ldpid_map.wrd = CA_NE_REG_READ(QM_QM_UPPER_LDPID_MAP);
	if (l3qm_ldpid_to_pon_port_map_offset)
		upper_ldpid_map.bf.pon_en = 1;
#if defined(CONFIG_LUNA_G3_SERIES)
	upper_ldpid_map.bf.pon_offset = l3qm_ldpid_to_pon_port_map_offset >> 3;
#else
	upper_ldpid_map.bf.pon_offset = l3qm_ldpid_to_pon_port_map_offset << 1;
#endif
	CA_NE_REG_WRITE(upper_ldpid_map.wrd, QM_QM_UPPER_LDPID_MAP);
}

void aal_l3qm_dump_registers(void)
{
        u32     reg;
        u32     value;

        printk("====================================L3 QM=================================\n");
        for (reg = QM_QM_REVISION; reg <= QM_QM_SPARE_STS; reg+=4) {
                value = CA_NE_REG_READ(reg);
                printk("register=0x%x, value=0x%x\n", reg, value);
        }
}

int aal_l3qm_get_ace_test(void)
{
	return l3qm_ace_test;
}

static void aal_l3qm_get_axi_attrib(int entry, QM_QM_AXI_ATTRIBUTE_DATA0_t *attrib_data0, QM_QM_AXI_ATTRIBUTE_DATA1_t *attrib_data1)
{
        int i;
        QM_QM_AXI_ATTRIBUTE_ACCESS_t    attrib;

	attrib.wrd = 0;
        attrib.bf.access = 1;
        attrib.bf.rbw = 0;
        attrib.bf.ADDR = entry;
	CA_NE_REG_WRITE(attrib.wrd, QM_QM_AXI_ATTRIBUTE_ACCESS);
        for (i = 0; i < CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT; i++) {
                attrib.wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_ACCESS);
                if (!attrib.bf.access)
                        break;
        }
        if (i == CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT) {
                printk("%s: CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT at entry=%d!!!\n", __func__, entry);
                return;
        }

        attrib_data0->wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_DATA0);
        attrib_data1->wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_DATA1);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("[%d] data0=0x%x, data1=0x%x\n", entry, attrib_data0->wrd, attrib_data1->wrd);
		printk("     qos=%d, cache=%d, snoop=%d, bar=%d, domain=%d, user=%d, prot=%d\n", attrib_data0->bf.qos, attrib_data0->bf.cache, attrib_data0->bf.snoop,
				attrib_data0->bf.bar, attrib_data0->bf.domain, attrib_data0->bf.user, attrib_data0->bf.prot);

		printk("     axi_top_bit=0x%x, ace_cmd=%d\n", attrib_data0->bf.axi_top_bit | (attrib_data1->bf.axi_top_bit << 6), attrib_data0->bf.ace_cmd);
	}
}

int aal_l3qm_get_cpuepp_axi_attrib(int cpu_port)
{
	QM_QM_AXI_ATTRIBUTE_DATA0_t attrib_data0;
	QM_QM_AXI_ATTRIBUTE_DATA1_t attrib_data1;

	aal_l3qm_get_axi_attrib(CA_L3QM_AXI_ATTRIBUTE_CPU_BASE + cpu_port, &attrib_data0, &attrib_data1);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: cpu_port=%d, axi_top_bit=%d\n", __func__,  cpu_port, attrib_data0.bf.axi_top_bit | (attrib_data1.bf.axi_top_bit << 6));
	}
	return attrib_data0.bf.axi_top_bit | (attrib_data1.bf.axi_top_bit << 6);
}

int aal_l3qm_get_eq_pool_axi_top_8bit(int eqid)
{
	QM_QM_CFG4_EQ0_t	cfg4_eq;

	cfg4_eq.wrd = CA_NE_REG_READ(QM_QM_CFG4_EQ0 + eqid * QM_QM_CFG4_EQ0_STRIDE);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: eqid=%d, axi_top_bit=%d\n", __func__,  eqid, cfg4_eq.bf.axi_top_bit);
	}

	return cfg4_eq.bf.axi_top_bit;
}

int aal_l3qm_get_cpu_port_head_room_first(void)
{
	return l3qm_cpu_port_head_room_first;
}

int aal_l3qm_eq_get_cpu_rule(void)
{
	return l3qm_eq_profile_cpu_rule;
}

int aal_l3qm_eq_get_cpu_use_fbm(void)
{
        return l3qm_eq_profile_cpu_use_fbm;
}

int aal_l3qm_eq_get_cpu_pool0_buf_size(void)
{
	return l3qm_eq_profile_cpu_pool0_buf_sz;
}

int aal_l3qm_eq_get_desc_per_epp(void)
{
	return l3qm_desc_per_epp;
}

int aal_l3qm_eq_get_desc_size(void)
{
	return l3qm_desc_size;
}

int aal_l3qm_eq_get_max_write_ptr(void)
{
	return (l3qm_cpu_epp_per_voq * l3qm_desc_size * l3qm_desc_per_epp);
}

int aal_l3qm_eq_get_cpu_pool_bid_count(void)
{
	return l3qm_cpu_pool_bid_count;
}

void aal_l3qm_eq_get_cpu_pool0_1_start_addr(ca_uint_t *cpu_pool0_start_addr, ca_uint_t *cpu_pool1_start_addr)
{
	*cpu_pool0_start_addr = l3qm_eq_profile_cpu_pool0_start_addr;
	*cpu_pool1_start_addr = l3qm_eq_profile_cpu_pool1_start_addr;
}

void aal_l3qm_eq_get_cpu_pool0_1_eq_id(ca_uint8_t *cpu_pool0_eq_id, ca_uint8_t *cpu_pool1_eq_id)
{
	*cpu_pool0_eq_id = l3qm_eq_profile_cpu_pool0_eq_id;
	*cpu_pool1_eq_id = l3qm_eq_profile_cpu_pool1_eq_id;
}

void aal_l3qm_eq_get_cpu_pool0_info(ca_uint_t *virt_start_addr, ca_uint32_t *entry_size, ca_uint32_t *entry_count, ca_uint8_t *fbm_pool_id)
{
       *virt_start_addr = l3qm_eq_profile_cpu_pool0_start_addr;
       *entry_size = l3qm_eq_profile_cpu_pool0_buf_sz;
       *entry_count = l3qm_eq_profile_cpu_pool0_buf_count;
       *fbm_pool_id = l3qm_eq_profile_cpu_pool0_fbm_pool_id;
}

void aal_l3qm_eq_get_cpu_pool1_info(ca_uint_t *virt_start_addr, ca_uint32_t *entry_size, ca_uint32_t *entry_count, ca_uint8_t *fbm_pool_id)
{
        *virt_start_addr = l3qm_eq_profile_cpu_pool1_start_addr;
        *entry_size = l3qm_eq_profile_cpu_pool1_buf_sz;
        *entry_count = l3qm_eq_profile_cpu_pool1_buf_count;
        *fbm_pool_id = l3qm_eq_profile_cpu_pool1_fbm_pool_id;
}

void aal_l3qm_get_l3fe_main_hash_action_info(aal_l3qm_l3fe_main_hash_info_t *l3fe_main_hash_info)
{
        l3fe_main_hash_info->overflow_action_start_virt_addr = l3fe_main_hash_overflow_hash_fib_start_addr;
        l3fe_main_hash_info->default_action_start_virt_addr  = l3fe_main_hash_default_hash_fib_start_addr;
        l3fe_main_hash_info->action_cache_start_virt_addr    = l3fe_main_hash_action_cache_fib_start_addr;
        l3fe_main_hash_info->hash_table_start_virt_addr      = l3fe_main_hash_table_start_addr;
        l3fe_main_hash_info->action_table_start_virt_addr    = l3fe_main_action_table_start_addr;
        l3fe_main_hash_info->overflow_action_start_phy_addr  = l3fe_main_hash_overflow_hash_fib_phy_addr;
        l3fe_main_hash_info->default_action_start_phy_addr   = l3fe_main_hash_default_hash_fib_phy_addr;
        l3fe_main_hash_info->action_cache_start_phy_addr     = l3fe_main_hash_action_cache_fib_phy_addr;
        l3fe_main_hash_info->hash_table_start_phy_addr       = l3fe_main_hash_table_phy_addr;
        l3fe_main_hash_info->action_table_start_phy_addr     = l3fe_main_action_table_phy_addr;
        l3fe_main_hash_info->overflow_hash_fib_length	     = l3fe_main_hash_overflow_hash_fib_length;
        l3fe_main_hash_info->overflow_hash_fib_count         = l3fe_main_hash_overflow_hash_fib_count;
        l3fe_main_hash_info->default_hash_fib_length	     = l3fe_main_hash_default_hash_fib_length;
        l3fe_main_hash_info->default_hash_fib_count	     = l3fe_main_hash_default_hash_fib_count;
        l3fe_main_hash_info->action_cache_fib_length	     = l3fe_main_hash_action_cache_fib_length;
        l3fe_main_hash_info->action_cache_fib_count	     = l3fe_main_hash_action_cache_fib_count;
        l3fe_main_hash_info->table_entry_length	     	     = l3fe_main_hash_table_entry_length;
        l3fe_main_hash_info->table_entry_count	             = l3fe_main_hash_table_entry_count;
        l3fe_main_hash_info->action_table_entry_length	     = l3fe_main_hash_action_table_entry_length;
        l3fe_main_hash_info->action_table_entry_count	     = l3fe_main_hash_action_table_entry_count;
	l3fe_main_hash_info->main_hash_bm_test = l3qm_main_hash_bm_test;
}

void aal_l3qm_set_l3fe_main_hash(ca_uint_t main_hash_table_saddr, ca_uint32_t main_hash_table_paddr,
					ca_uint_t main_hash_action_saddr, ca_uint32_t main_hash_action_paddr)
{
	l3fe_main_hash_table_start_addr = main_hash_table_saddr;
	l3fe_main_action_table_start_addr = main_hash_action_saddr;
	l3fe_main_hash_table_phy_addr = main_hash_table_paddr;
	l3fe_main_action_table_phy_addr = main_hash_action_paddr;
}

void aal_l3qm_set_rx_read_ptr(ca_uint16_t cpu_port, ca_uint8_t voq, ca_uint32_t value)
{
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_t	rdptr;

	rdptr.wrd = 0;
	rdptr.bf.rptr = value;

	CA_NE_REG_WRITE(rdptr.wrd, QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR + (cpu_port * l3qm_cpu_voq_per_port + voq) * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_STRIDE);
}

ca_uint32_t aal_l3qm_get_rx_read_ptr(ca_uint16_t cpu_port, ca_uint8_t voq)
{
        QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_t      rdptr;

	/* 64 instances 8 CPU ports each port has 8 VOQs */
	rdptr.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR + (cpu_port * l3qm_cpu_voq_per_port + voq) * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_STRIDE);

	return rdptr.bf.rptr;
}

ca_uint32_t aal_l3qm_get_rx_write_ptr(ca_uint16_t cpu_port, ca_uint8_t voq, ca_uint16_t *count)
{
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_t	wrptr;
	ca_uint32_t rdptr;
	int epp_size, size;

	/* 64 instances 8 CPU ports each port has 8 VOQs */
	wrptr.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR + (cpu_port * l3qm_cpu_voq_per_port + voq) * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_STRIDE);

	rdptr = aal_l3qm_get_rx_read_ptr(cpu_port, voq);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: wrptr.bf.wptr=%d, rdptr=%d\n", __func__, wrptr.bf.wptr, rdptr);
	}

	/* take care of wraping around case */
	epp_size = l3qm_desc_size * l3qm_desc_per_epp;
	if (wrptr.bf.wptr >= rdptr) {
		*count = (wrptr.bf.wptr - rdptr) / epp_size;
	}
	else {
		size = aal_l3qm_eq_get_max_write_ptr();
		*count = ((size - rdptr) + wrptr.bf.wptr) / epp_size;
		if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: max write pointer=%d\n", __func__, size);
		}
	}

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: wrptr.bf.wptr=%d, rdptr=%d, count=%d\n", __func__, wrptr.bf.wptr, rdptr, *count);
	}

	return wrptr.bf.wptr;
}
#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)

ca_uint32_t aal_l3qm_special_fastFwd_get_rx_read_write_ptr(ca_uint16_t cpu_port, ca_uint8_t voq, ca_uint16_t *count, ca_uint32_t *rdptr)
{
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_t	wrptr;
	//ca_uint32_t rdptr;
	int epp_size, size;

	/* 64 instances 8 CPU ports each port has 8 VOQs */
	wrptr.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR + (cpu_port * l3qm_cpu_voq_per_port + voq) * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_STRIDE);

	*rdptr = aal_l3qm_get_rx_read_ptr(cpu_port, voq);


	/* take care of wraping around case */
	//epp_size = l3qm_desc_size * l3qm_desc_per_epp;
	if (wrptr.bf.wptr >= *rdptr) {
		//*count = (wrptr.bf.wptr - *rdptr) / epp_size;
		*count = (wrptr.bf.wptr - *rdptr) >>2; // 4
	}
	else {
		size = aal_l3qm_eq_get_max_write_ptr();
		//*count = ((size - *rdptr) + wrptr.bf.wptr) / epp_size;
		*count = ((size - *rdptr) + wrptr.bf.wptr) >>2;
	}


	return wrptr.bf.wptr;
}
#endif

ca_uint32_t aal_l3qm_get_rx_start_addr(ca_uint16_t cpu_port, ca_uint8_t voq)
{
#if defined(CONFIG_LUNA_G3_SERIES)
	return epp64_paddr_start[cpu_port][voq];
#else
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_t paddr_start;

	paddr_start.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START + (cpu_port * l3qm_cpu_voq_per_port + voq) *
					   QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE);

        if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: cpu_port=%d, voq=%d, paddr_start.bf.phy_addr=0x%x\n", __func__, cpu_port, voq, paddr_start.bf.phy_addr);
	}

	return paddr_start.bf.phy_addr;
#endif
}

ca_uint32_t aal_l3qm_get_rx_status0(void)
{
	/* get CPU EPP FIFO (VoQ#31-0) status */
	QM_QM_CPU_EPP_STATUS0_t	status0;

	status0.wrd = CA_NE_REG_READ(QM_QM_CPU_EPP_STATUS0);

	return status0.bf.status;
}

ca_uint32_t aal_l3qm_get_rx_status1(void)
{
        /* get CPU EPP FIFO (VoQ#63-32) status */
        QM_QM_CPU_EPP_STATUS1_t status1;

        status1.wrd = CA_NE_REG_READ(QM_QM_CPU_EPP_STATUS1);

        return status1.bf.status;
}

ca_uint16_t aal_l3qm_get_inactive_bid_cntr(int eqid)
{
	int j;
	QM_QM_EQM_PA_REQ0_t eqm_pa_req;

	eqm_pa_req.wrd = 0;
	for (j = 0; j < L3QM_INACTIVE_BID_CNTR; j++) {
		eqm_pa_req.wrd = CA_NE_REG_READ(QM_QM_EQM_PA_REQ0 + QM_QM_EQM_PA_REQ0_STRIDE * eqid);
		if (eqm_pa_req.bf.req) {
			break;
		}
	}

	if (j == L3QM_INACTIVE_BID_CNTR) {
		if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: EQM is not ready for push physical address!!!\n", __func__);
		}
		return 0;
	}

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: eqid=%d, bid_cntr=%d\n", __func__, eqid, eqm_pa_req.bf.inactive_bid_cntr);
	}
	return eqm_pa_req.bf.inactive_bid_cntr;
}

void aal_l3qm_load_eq_config(void)
{
	/* load the configuration */
	CA_NE_REG_WRITE(0x05102013, QM_QM_HDM_WRITE_PROTECTION);
	CA_NE_REG_WRITE(0xffff, QM_QM_EQ_CFG_LOAD);
	CA_NE_REG_WRITE(0, QM_QM_EQ_CFG_LOAD);
	CA_NE_REG_WRITE(0, QM_QM_HDM_WRITE_PROTECTION);
}

void aal_l3qm_load_epp_config(void)
{
}

ca_uint32_t aal_l3qm_check_cpu_push_ready(int cpu_port)
{
	int i;
	QM_QM_CPU_PUSH_RDY0_t	cpu_push_rdy;

	/* check ready bit */
	for (i = 0; i < L3QM_CPU_PUSH_RDY_TIMEOUT; i++) {
		cpu_push_rdy.wrd = CA_NE_REG_READ(QM_QM_CPU_PUSH_RDY0 + (QM_QM_CPU_PUSH_RDY0_STRIDE * cpu_port));
		if (cpu_push_rdy.bf.rdy) {
			break;
		}
	}
	if (i == L3QM_CPU_PUSH_RDY_TIMEOUT) {
		if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: EQM is not ready for push physical address!!!\n", __func__);
		}
		return 0;
	}
	return 1;
}

void aal_l3qm_set_cpu_push_paddr(int cpu_port, int eqid, ca_uint32_t phy_addr)
{
	QM_QM_CPU_PUSH_PADDR0_t cpu_push_paddr;

	cpu_push_paddr.bf.addr = phy_addr >> CA_L3QM_PHY_ADDR_SHIFT;
	cpu_push_paddr.bf.eqid = eqid;

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: cpu_port=%d, eqid=%d, phy_addr=0x%x, cpu_push_paddr=0x%x\n", __func__, cpu_port, eqid, phy_addr, cpu_push_paddr.wrd);
	}

	CA_NE_REG_WRITE(cpu_push_paddr.wrd, QM_QM_CPU_PUSH_PADDR0 + (QM_QM_CPU_PUSH_PADDR0_STRIDE * cpu_port));
}

void aal_l3qm_set_epp_axi_attrib(int entry, ca_uint32_t data0)
{
	int i;
	QM_QM_AXI_ATTRIBUTE_ACCESS_t	attrib;
	QM_QM_AXI_ATTRIBUTE_DATA0_t	attrib_data0;

	attrib_data0.wrd = data0;
	CA_NE_REG_WRITE(attrib_data0.wrd, QM_QM_AXI_ATTRIBUTE_DATA0);

	attrib.bf.access = 1;
	attrib.bf.rbw = 1;
	attrib.bf.ADDR = entry;
	CA_NE_REG_WRITE(attrib.wrd, QM_QM_AXI_ATTRIBUTE_ACCESS);
	for (i = 0; i < CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT; i++) {
		attrib.wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_ACCESS);
		if (!attrib.bf.access)
			break;
	}
	if (i == CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT) {
		printk("%s: CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT!!!\n", __func__);
		return;
	}

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: attrib_data0.wrd=0x%x, QM_QM_AXI_ATTRIBUTE_DATA0=0x%x\n", __func__, attrib_data0.wrd, QM_QM_AXI_ATTRIBUTE_DATA0);
	}
}

#define AAL_L3QM_MAX_BID	16384
typedef struct aal_l3qm_bid_entry {
	ca_uint8_t  eqid;
	ca_uint16_t bid_start;
	ca_uint16_t bid_count;
} aal_l3qm_bid_entry_t;
static int aal_l3qm_bid_entry_count = 0;
static aal_l3qm_bid_entry_t bid_entry[QM_QM_CFG1_EQ0_COUNT];
static void aal_l3qm_add_bid(ca_uint8_t eqid, ca_uint16_t bid_start, ca_uint16_t bid_count)
{
	int i;

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: aal_l3qm_bid_entry_count=%d\n", __func__, aal_l3qm_bid_entry_count);
	}
	if (aal_l3qm_bid_entry_count >= AAL_L3QM_MAX_BID) {
		printk("%s: aal_l3qm_bid_entry_count=%d bid table is full!!\n", __func__, aal_l3qm_bid_entry_count);
		return;
	}

	/* find the index for new added bid */
	i = 0;
	while (i < aal_l3qm_bid_entry_count) {
		if (bid_start < bid_entry[i].bid_start) {
			if (ca_aal_debug & AAL_DBG_L3QM) {
				printk("%s: aal_l3qm_bid_entry_count - i=%d\n", __func__, aal_l3qm_bid_entry_count - i);
			}
			memcpy(&(bid_entry[i+1]), &(bid_entry[i]), (aal_l3qm_bid_entry_count - i) * sizeof(aal_l3qm_bid_entry_t));
			break;
		}
		i++;
	}
	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: i=%d, eqid=%d, bid_start=%d, bid_count=%d\n", __func__, i, eqid, bid_start, bid_count);
	}
	bid_entry[i].eqid = eqid;
	bid_entry[i].bid_start = bid_start;
	bid_entry[i].bid_count = bid_count;
	aal_l3qm_bid_entry_count++;
}
ca_status_t aal_l3qm_check_bid_start(ca_device_id_t device_id)
{
	int i;
	ca_uint16_t next_bid_start;
	ca_uint16_t total_bid_count;
	QM_QM_CFG1_EQ0_t cfg1_eq;

	next_bid_start = 0;

	aal_l3qm_bid_entry_count = 0;
	memset(&(bid_entry[0]), 0, sizeof(bid_entry));

	for (i = 0; i < QM_QM_CFG1_EQ0_COUNT; i++) {
		cfg1_eq.wrd = CA_NE_REG_READ(QM_QM_CFG1_EQ0 + i * QM_QM_CFG1_EQ0_STRIDE);
		if (cfg1_eq.bf.total_buffer_num == 0)		/* skip the eqid not in use */
			continue;

		aal_l3qm_add_bid(i, cfg1_eq.bf.bid_start, cfg1_eq.bf.total_buffer_num);
	}
#if !defined(CONFIG_LUNA_G3_SERIES)
	printk("%s: aal_l3qm_bid_entry_count=%d\n", __func__, aal_l3qm_bid_entry_count);
	total_bid_count = 0;
	for (i = 0; i < aal_l3qm_bid_entry_count; i++) {
		printk("[%d]: eqid=%d bid_start=%d bid_count=%d\n", i, bid_entry[i].eqid, bid_entry[i].bid_start, bid_entry[i].bid_count);
		printk("next_bid_start=%d\n", next_bid_start);

		/* check whether the bid range overlapped */
		if (next_bid_start == 0) {
			next_bid_start = bid_entry[i].bid_start + bid_entry[i].bid_count;
			total_bid_count += bid_entry[i].bid_count;
			continue;
		}
		if (bid_entry[i].bid_start < next_bid_start) {
			printk("%s: bid overlap at eqid=%d, bid_entry[%d].bid_start=%d, next_bid_start=%d!!\n",
					__func__, bid_entry[i].eqid, i, bid_entry[i].bid_start, next_bid_start);
			BUG();
		}
		next_bid_start = bid_entry[i].bid_start + bid_entry[i].bid_count;
		total_bid_count += bid_entry[i].bid_count;

		printk("total_bid_count=%d\n", total_bid_count);
		if (total_bid_count > AAL_L3QM_MAX_BID) {
			printk("%s: total bid count %d exceed %d!!\n", __func__, total_bid_count, AAL_L3QM_MAX_BID);
			BUG();
		}
	}
#endif
	return CA_E_OK;
}

ca_status_t aal_l3qm_check_epps(ca_device_id_t device_id)
{
	int i, j, size;
	ca_uint8_t *virt_addr;

	/* check magic number among CPU EPPs */
	size = l3qm_cpu_epp_per_voq * l3qm_desc_size * l3qm_desc_per_epp + l3qm_magic_marker_len;
	for (j = 0; j < ca_ni_cpu_cpu_port_count; j++) {
		for (i = 0; i < l3qm_cpu_voq_per_port; i++) {
			virt_addr = (ca_uint8_t *)(l3qm_epp_profile_cpu_rx_fifo_start_addr + (i * size) + (j * l3qm_cpu_voq_per_port * size));
			if (ca_aal_debug & AAL_DBG_L3QM) {
				printk("%s: CPU RX FIFO start address(%d)=0x%p\n", __func__, i, virt_addr + (size - l3qm_magic_marker_len));
			}
			if (aal_l3qm_check_magic_number(virt_addr + (size - l3qm_magic_marker_len)) != CA_E_OK) {
				printk("%s: magic number check failed at CPU EPP (%d, %d)!!!\n", __func__, j, i);
				return CA_E_ERROR;
			}
		}
	}
	printk("Magic number among CPU EPPs are OK\n");

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_cpu256_check_magic_number();
#endif

	return CA_E_OK;
}

static void aal_l3qm_init_qm_axi_attrib(void)
{
	int i;
	ca_uint32_t offset;
	QM_AXIM2_CONFIG_t	axim2_cfg;
	QM_QM_AXI_ATTRIBUTE_DATA0_t data0;

	/* AXIM2 */
	axim2_cfg.wrd = CA_NE_REG_READ(QM_AXIM2_CONFIG);
	axim2_cfg.bf.axi_write_outtrans_num = 0xf;
	axim2_cfg.bf.axi_read_outtrans_num = 0xf;
	axim2_cfg.bf.write_cacheline_trans_en = 0;
	axim2_cfg.bf.read_cacheline_trans_en = 0;
	CA_NE_REG_WRITE(axim2_cfg.wrd, QM_AXIM2_CONFIG);

	/* EQ 0-15 */
	for (i = 0; i < CFG_ID_L3QM_AXI_ATTRIB_EQ_VALUES_LEN; i++) {
		offset = CA_L3QM_AXI_ATTRIBUTE_EQ_BASE + i;

		/* if QM ACE not turned on, ignore the startup configure settings if ace_cmd=1 or axi_top_bit=2 */
		data0.wrd = l3qm_axi_attrib_eq_values[i];
		if (!l3qm_ace_test) {
			if (data0.bf.axi_top_bit == 2 || data0.bf.ace_cmd == 1)
				data0.wrd = 0;
		}
		aal_l3qm_set_epp_axi_attrib(offset, data0.wrd);

		if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: EQ[%d] offset=0x%x, data0.wrd=0x%x, l3qm_axi_attrib_eq_values=0x%x\n",
				__func__, i, offset, data0.wrd, l3qm_axi_attrib_eq_values[i]);
		}
	}

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_init_cpu256_qm_axi_attrib();
#endif

	/* CPU EPP port 0-7 */
	for (i = 0; i < CFG_ID_L3QM_AXI_ATTRIB_CPU_EPP_VALUES_LEN; i++) {
		offset = CA_L3QM_AXI_ATTRIBUTE_CPU_BASE + i;

		/* if QM ACE not turned on, ignore the startup configure settings if ace_cmd=1 or axi_top_bit=2 */
		data0.wrd = l3qm_axi_attrib_cpu_epp_values[i];
		if (!l3qm_ace_test) {
			if (data0.bf.axi_top_bit == 2 || data0.bf.ace_cmd == 1)
				data0.wrd = 0;
		}

		aal_l3qm_set_epp_axi_attrib(offset, data0.wrd);

                if (ca_aal_debug & AAL_DBG_L3QM) {
                        printk("%s: CPU EPP[%d] offset=0x%x, data0.wrd=0x%x, l3qm_axi_attrib_cpu_epp_values=0x%x\n",
				__func__, i, offset, data0.wrd, l3qm_axi_attrib_cpu_epp_values[i]);
                }
	}
}

/* Sending packets to CPU port through L3QM - Transmitting */
/* Configure CPU EPP FIFO for receiving packets for CPU */
void aal_l3qm_init_cpu_epp(void)
{
	int i, j, size;
	ca_uint32_t reg_off;
	QM_QM_CPU_EPP0_CFG_t cpu_epp0_cfg;
	QM_QM_CPU_EPP_FIFO_CFG_profile0_t cpu_epp_fifo_cfg_profile;
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_t cpu_epp64_fifo_paddr_start;
	ca_uint8_t *virt_addr;

	/* CPU EPP Port Configuration Register */
	/* Select per-destionation VOQ-to-EPP-FIFO-Ring map-mode */
	for (i = 0; i < ca_ni_cpu_cpu_port_count; i++) {
		reg_off = QM_QM_CPU_EPP0_CFG + (QM_QM_CPU_EPP0_CFG_STRIDE * i);
		cpu_epp0_cfg.wrd = CA_NE_REG_READ(reg_off);
		cpu_epp0_cfg.bf.map_mode = l3qm_epp_profile_cpu_map_mode;
		CA_NE_REG_WRITE(cpu_epp0_cfg.wrd, reg_off);
	}

	/* CPU EPP Interrupt Coalescing Timer Configuration Register */
	/* Select coalescing Timer-base */
	CA_NE_REG_WRITE(0, QM_QM_CPU_EPP_CT_CFG);

	/* CPU EPP FIFO Configuration Register */
	/* only one EPP profile is used for CPU, the size is for one VoQ */
	reg_off = QM_QM_CPU_EPP_FIFO_CFG_profile0 + (QM_QM_CPU_EPP_FIFO_CFG_profile0_STRIDE * l3qm_epp_profile_cpu_id);
	cpu_epp_fifo_cfg_profile.wrd = CA_NE_REG_READ(reg_off);
	cpu_epp_fifo_cfg_profile.bf.size = l3qm_cpu_epp_per_voq / CA_L3QM_CPU_EPP_FIFO_SIZE_UINT;  	/* the unit is 4 FIFO entries */
	cpu_epp_fifo_cfg_profile.bf.timer_ths = 0xf;
	if ((ca_ni_napi_budget / 16) > 0)
		cpu_epp_fifo_cfg_profile.bf.high_ths = (ca_ni_napi_budget / 16) + 1;	/* Interrupt to CPU is generated when EPP-FIFO accumulates at least ca_napi_budget entries */
	else
		cpu_epp_fifo_cfg_profile.bf.high_ths = 2;				/* Interrupt to CPU is generated when EPP-FIFO accumulates at least 16 entries */
	CA_NE_REG_WRITE(cpu_epp_fifo_cfg_profile.wrd, reg_off);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: l3qm_cpu_epp_per_voq=%d\n", __func__, l3qm_cpu_epp_per_voq);
	}

	/* CPU EPP FIFO Starting address Register */
	/* Select one of 4 profiles for EPP-FIFO size, occupancy-high-threshold and time-threshold for CPU ports 0-5 */
	for (i = 0; i < ca_ni_cpu_cpu_port_count; i++) {
		for (j = 0; j < l3qm_cpu_voq_per_port; j++) {
			/* select profile 2 */
			reg_off = QM_QM_CPU_EPP_FIFO0_0_CFG + (QM_QM_CPU_EPP_FIFO0_0_CFG_STRIDE * i) + (4 * j);
			CA_NE_REG_WRITE(l3qm_epp_profile_cpu_id, reg_off);
		}
	}
	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: l3qm_epp_profile_cpu_id=%d\n", __func__, l3qm_epp_profile_cpu_id);
	}

	/* fill the physical start address for CPU EPPs */
	size = l3qm_cpu_epp_per_voq * l3qm_desc_size * l3qm_desc_per_epp + l3qm_magic_marker_len;
        for (j = 0; j < ca_ni_cpu_cpu_port_count; j++) {
                for (i = 0; i < l3qm_cpu_voq_per_port; i++) {
                        reg_off = QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START + (i * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE) +
				(j * l3qm_cpu_voq_per_port * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE);
                        cpu_epp64_fifo_paddr_start.bf.phy_addr = l3qm_epp_profile_cpu_rx_fifo_phy_addr + (i * size) + (j * l3qm_cpu_voq_per_port * size);
                        CA_NE_REG_WRITE(cpu_epp64_fifo_paddr_start.wrd, reg_off);
#if defined(CONFIG_LUNA_G3_SERIES)
				epp64_paddr_start[j][i] = cpu_epp64_fifo_paddr_start.bf.phy_addr;
#endif

                        if (ca_aal_debug & AAL_DBG_L3QM) {
				printk("%s: CPU RX FIFO register offset(%d, %d)=0x%x\n", __func__, j, i, reg_off);
                                printk("%s: CPU RX FIFO start address(%d,%d)=0x%x\n", __func__, j, i, cpu_epp64_fifo_paddr_start.bf.phy_addr);
				printk("%s: size=%d, offset=%d\n", __func__, size, (i * size) + (j * l3qm_cpu_voq_per_port * size));
                        }
			virt_addr = (ca_uint8_t *)(l3qm_epp_profile_cpu_rx_fifo_start_addr + (i * size) + (j * l3qm_cpu_voq_per_port * size) + (size - l3qm_magic_marker_len));
			aal_l3qm_insert_magic_number(virt_addr);
                }
        }

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_init_cpu256_epp();
#endif

	aal_l3qm_load_epp_config();
}

void aal_l3qm_axi_reo_rd_init(ca_device_id_t device_id)
{
	QM_AXI_REO_AXI_REO_RD_ORIG_ID_t orig_id;
	QM_AXI_REO_AXI_REO_RD_NEW_ID_t new_id;
	QM_AXI_REO_AXI_REO_RD_TOP_ADDR0_t top_addr;
	QM_AXI_REO_AXI_REO_RD_TOP_ADDR_MASK0_t top_addr_mask;
	QM_AXI_REO_AXI_REO_RD_NEW_ID0_t new_id_0;

	/* AXI reoder for deepq read*/
	orig_id.wrd = 0;
	orig_id.bf.axi_id = 15;

	new_id.wrd = 0;
	new_id.bf.axi_id = 12;
	new_id.bf.axi_id_valid = 1;

	top_addr.bf.id_addr = 0x10000000;
	top_addr_mask.bf.id_mask = 0x10000000;

	new_id_0.wrd = 0;
	new_id_0.bf.axi_id = 13;
	new_id_0.bf.axi_id_valid = 1;

	CA_NE_AXI_REO_REG_WRITE(orig_id.wrd, QM_AXI_REO_AXI_REO_RD_ORIG_ID);
	CA_NE_AXI_REO_REG_WRITE(new_id.wrd, QM_AXI_REO_AXI_REO_RD_NEW_ID);
	CA_NE_AXI_REO_REG_WRITE(top_addr.wrd, QM_AXI_REO_AXI_REO_RD_TOP_ADDR0);
	CA_NE_AXI_REO_REG_WRITE(top_addr_mask.wrd, QM_AXI_REO_AXI_REO_RD_TOP_ADDR_MASK0);
	CA_NE_AXI_REO_REG_WRITE(new_id_0.wrd, QM_AXI_REO_AXI_REO_RD_NEW_ID0);
}


void aal_l3qm_axi_reo_wr_init(ca_device_id_t device_id)
{
	QM_AXI_REO_AXI_REO_WR_ORIG_ID_t orig_id;
	QM_AXI_REO_AXI_REO_WR_NEW_ID_t new_id;
	QM_AXI_REO_AXI_REO_WR_TOP_ADDR0_t top_addr;
	QM_AXI_REO_AXI_REO_WR_TOP_ADDR_MASK0_t top_addr_mask;
	QM_AXI_REO_AXI_REO_WR_NEW_ID0_t new_id_0;

	/* AXI reoder for deepq write*/
	orig_id.wrd = 0;
	orig_id.bf.axi_id = 15;

	new_id.wrd = 0;
	new_id.bf.axi_id = 12;
	new_id.bf.axi_id_valid = 1;

	top_addr.bf.id_addr = 0x10000000;
	top_addr_mask.bf.id_mask = 0x10000000;

	new_id_0.wrd = 0;
	new_id_0.bf.axi_id = 13;
	new_id_0.bf.axi_id_valid = 1;

	CA_NE_AXI_REO_REG_WRITE(orig_id.wrd, QM_AXI_REO_AXI_REO_WR_ORIG_ID);
	CA_NE_AXI_REO_REG_WRITE(new_id.wrd, QM_AXI_REO_AXI_REO_WR_NEW_ID);
	CA_NE_AXI_REO_REG_WRITE(top_addr.wrd, QM_AXI_REO_AXI_REO_WR_TOP_ADDR0);
	CA_NE_AXI_REO_REG_WRITE(top_addr_mask.wrd, QM_AXI_REO_AXI_REO_WR_TOP_ADDR_MASK0);
	CA_NE_AXI_REO_REG_WRITE(new_id_0.wrd, QM_AXI_REO_AXI_REO_WR_NEW_ID0);

	/* AXI reoder for epp write*/	
	orig_id.wrd = 0;
	orig_id.bf.axi_id = 2;

	new_id.wrd = 0;
	new_id.bf.axi_id = 8;
	new_id.bf.axi_id_valid = 1;

	top_addr.bf.id_addr = 0x10000000;
	top_addr_mask.bf.id_mask = 0x10000000;

	new_id_0.wrd = 0;
	new_id_0.bf.axi_id = 9;
	new_id_0.bf.axi_id_valid = 1;

	CA_NE_AXI_REO_REG_WRITE(orig_id.wrd, QM_AXI_REO_AXI_REO_WR_ORIG_ID + APB2_QM_AXI_REO_AXI_REO_WR_STRIDE);
	CA_NE_AXI_REO_REG_WRITE(new_id.wrd, QM_AXI_REO_AXI_REO_WR_NEW_ID + APB2_QM_AXI_REO_AXI_REO_WR_STRIDE);
	CA_NE_AXI_REO_REG_WRITE(top_addr.wrd, QM_AXI_REO_AXI_REO_WR_TOP_ADDR0 + APB2_QM_AXI_REO_AXI_REO_WR_STRIDE);
	CA_NE_AXI_REO_REG_WRITE(top_addr_mask.wrd, QM_AXI_REO_AXI_REO_WR_TOP_ADDR_MASK0 + APB2_QM_AXI_REO_AXI_REO_WR_STRIDE);
	CA_NE_AXI_REO_REG_WRITE(new_id_0.wrd, QM_AXI_REO_AXI_REO_WR_NEW_ID0 + APB2_QM_AXI_REO_AXI_REO_WR_STRIDE);
}

ca_uint8_t aal_l3qm_get_buffer_size_index(ca_uint32_t buffer_size)
{
	ca_uint8_t index;

	switch (buffer_size) {
		case 128:
			index = 1;
			break;
		case 256:
			index = 2;
			break;
		case 512:
			index = 3;
			break;
		case 1024:
			index = 4;
			break;
		case 2048:
			index = 5;
			break;
		case 4096:
			index = 6;
			break;
		default:
			printk("%s: buffer_size=%d is not defined!!\n", __func__, buffer_size);
			index = 0;
			break;
	}
	return index;
}

static void aal_l3qm_init_empty_buffer_CPU(void)
{
	int i;
	ca_uint32_t reg_off;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	QM_QM_DEST_PORT0_PKT_BUF_CFG_t dest_port_pkt_buf_cfg;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG3_EQ0_t cfg3_eq;
        QM_QM_CFG4_EQ0_t cfg4_eq;

	/* only one profile is assigned to CPU */
	eq_profile.wrd = 0;
	eq_profile.bf.eqp0 = l3qm_eq_profile_cpu_pool0_eq_id;
	eq_profile.bf.eqp1 = l3qm_eq_profile_cpu_pool1_eq_id;
	eq_profile.bf.rule = l3qm_eq_profile_cpu_rule;
	reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * l3qm_eq_profile_cpu_id);
	CA_NE_REG_WRITE(eq_profile.wrd, reg_off);

        if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: CPU pool0 eq_id=%d, rule=%d\n", __func__, l3qm_eq_profile_cpu_pool0_eq_id, l3qm_eq_profile_cpu_rule);
	}

	/* for CPU ports profile select, profile 2 for CPU CPU ports 0-5 */
	for (i = 0; i < ca_ni_cpu_cpu_port_count; i++) {
		dest_port_eq_cfg.bf.profile_sel = l3qm_eq_profile_cpu_id;
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * i);
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
	}

        if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: CPU profile id=%d\n", __func__, l3qm_eq_profile_cpu_id);
	}

	for (i = 0; i < ca_ni_cpu_cpu_port_count; i++) {
		reg_off = QM_QM_DEST_PORT0_PKT_BUF_CFG + (QM_QM_DEST_PORT0_PKT_BUF_CFG_STRIDE * i);
		dest_port_pkt_buf_cfg.wrd = CA_NE_REG_READ(reg_off);
		dest_port_pkt_buf_cfg.bf.head_room_first = l3qm_cpu_port_head_room_first / 16;       /* reserved 32 bytes for first fragment of packet  */
		dest_port_pkt_buf_cfg.bf.head_room_rest = dest_port_pkt_buf_cfg.bf.head_room_first;  /* reserved 32 bytes for reset of segments of packet  */
		CA_NE_REG_WRITE(dest_port_pkt_buf_cfg.wrd, reg_off);
	}

	/* configure total_buffer_num and bid_start for CPU ports, EB pool 0-13 */
	/* only two EB pools are assigned to CPU */
	for (i = l3qm_eq_profile_cpu_pool0_eq_id; i <= l3qm_eq_profile_cpu_pool1_eq_id; i++) {

		/* enable Empty Buffer */
		cfg0_eq.wrd = 0;
		cfg0_eq.bf.eq_en = 1;
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * i);
		CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

		/* configure total_buffer_num and bid_start */
		cfg1_eq.wrd = 0;
		cfg1_eq.bf.total_buffer_num = l3qm_eq_profile_cpu_pool0_buf_count;

		if (i == l3qm_eq_profile_cpu_pool0_eq_id)
			cfg1_eq.bf.bid_start = l3qm_eq_profile_cpu_pool0_bid_start;
		else
			cfg1_eq.bf.bid_start = l3qm_eq_profile_cpu_pool1_bid_start;

		reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * i);
		CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	        if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: CPU eq_id=%d, pool0 bid_start=%d\n", __func__, i, l3qm_eq_profile_cpu_pool0_bid_start);
			printk("%s: CPU pool0 total_buffer_num=%d\n", __func__, cfg1_eq.bf.total_buffer_num);
		}

		/* configure buffer size for Empty Buffer */
		reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * i);
		cfg2_eq.wrd = CA_NE_REG_READ(reg_off);

		printk("%s: l3qm_eq_profile_cpu_use_fbm=%d, l3qm_eq_profile_cpu_pool0_fbm_pool_id=%d, l3qm_eq_profile_cpu_pool1_fbm_pool_id=%d\n", __func__, l3qm_eq_profile_cpu_use_fbm, l3qm_eq_profile_cpu_pool0_fbm_pool_id, l3qm_eq_profile_cpu_pool1_fbm_pool_id);

		if (l3qm_eq_profile_cpu_use_fbm) {
			cfg2_eq.bf.cpu_eq = 1;

			if (i == l3qm_eq_profile_cpu_pool0_eq_id)
				cfg2_eq.bf.refill_fbm_eqid = l3qm_eq_profile_cpu_pool0_fbm_pool_id;
			else
				cfg2_eq.bf.refill_fbm_eqid = l3qm_eq_profile_cpu_pool1_fbm_pool_id;

			/* use the same threshold as non-FBM, because we wish QM REFILL interrupt happened every 160 BIDs comsumed */
			/* set threshold to be (1500 - 160) / 8, regard the 160 as budget in RX routine */
			/* the budget value should not be too small otherwise the interrupts happened too frequently */
                        cfg2_eq.bf.refill_ths = (l3qm_cpu_pool_bid_count - ca_ni_napi_budget) / 8;

			cfg2_eq.bf.refill_en = 1;
		}
		else {
			cfg2_eq.bf.cpu_eq = 1;
			cfg2_eq.bf.refill_fbm_eqid = 0;

			/* set threshold to be (1500 - 160) / 8, regard the 160 as budget in RX routine */
			/* the budget value should not be too small otherwise the interrupts happened too frequently */
                        cfg2_eq.bf.refill_ths = (l3qm_cpu_pool_bid_count - ca_ni_napi_budget) / 8;

			cfg2_eq.bf.refill_en = 0;
			cfg2_eq.bf.refill_fbm_eqid = 0;
		}

		if (i == l3qm_eq_profile_cpu_pool0_eq_id)
			cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(l3qm_eq_profile_cpu_pool0_buf_sz);      /* 2048 bytes */
		else
			cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(l3qm_eq_profile_cpu_pool1_buf_sz);      /* 2048 bytes */

		reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * i);
		CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

		printk("%s: i=%d, reg_off=0x%x, cfg2_eq.wrd=0x%x\n", __func__, i, reg_off, cfg2_eq.wrd);

	        if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: CPU pool buf_sz=%d, size_index=%d\n", __func__, l3qm_eq_profile_cpu_pool0_buf_sz, cfg2_eq.bf.buffer_size);
		}
	}

	/* CPU pool 0 */
	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * l3qm_eq_profile_cpu_pool0_eq_id);
	/* if QM ACE not turned on, ignore the startup configure settings if acd_cmd=1 */
	cfg3_eq.wrd = l3qm_eq_cfg3_values[l3qm_eq_profile_cpu_pool0_eq_id];
	if (!l3qm_ace_test && cfg3_eq.bf.acd_cmd == 1) {
		cfg3_eq.wrd = 0;
	}
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * l3qm_eq_profile_cpu_pool0_eq_id);
	cfg4_eq.wrd = 0;
	if (l3qm_ace_test) {
		cfg4_eq.bf.axi_top_bit = l3qm_eq_cfg4_values[l3qm_eq_profile_cpu_pool0_eq_id];
	}

	printk("%s: l3qm_eq_profile_cpu_pool0_eq_id=%d, cfg4_eq.bf.axi_top_bit=%d\n", __func__, l3qm_eq_profile_cpu_pool0_eq_id, cfg4_eq.bf.axi_top_bit);
        CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	/* CPU pool1 */
	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * l3qm_eq_profile_cpu_pool1_eq_id);
	/* if QM ACE not turned on, ignore the startup configure settings if acd_cmd=1 */
	cfg3_eq.wrd = l3qm_eq_cfg3_values[l3qm_eq_profile_cpu_pool1_eq_id];
	if (!l3qm_ace_test && cfg3_eq.bf.acd_cmd == 1) {
		cfg3_eq.wrd = 0;
	}
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * l3qm_eq_profile_cpu_pool1_eq_id);
	cfg4_eq.wrd = 0;
	if (l3qm_ace_test) {
		cfg4_eq.bf.axi_top_bit = l3qm_eq_cfg4_values[l3qm_eq_profile_cpu_pool1_eq_id];
	}
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);
}
#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
static void aal_l3qm_init_empty_buffer_CPU_fastFwd(void)
{
	/*
	 * Overwrite cpu port setting 
	 *  - Case 1: use 2 cpu ports
	 *   -- 0x16 select profile #6 ( sel pool id #11 & #15 constructed by  1280 + 0 buffers)
	 *   -- 0x17 select profile #7 ( sel pool id #12 & #15 constructed by  1280 + 0 buffers)
	 *  - Case 2: use 4 ports if (CONFIG_FC_SPECIAL_FAST_FORWARD)
	 *   -- 0x14 select profile #6 ( sel pool id #11 & #15 constructed by  1280 + 0 buffers)
	 *   -- 0x15 select profile #7 ( sel pool id #12 & #15 constructed by  1280 + 0 buffers)
	 *   -- 0x16 select profile #2 ( sel pool id #13 & #15 constructed by  1280 + 0 buffers)
	 *   -- 0x17 select profile #3 ( sel pool id #14 & #15 constructed by  1280 + 0 buffers)
	*/
	int i;
	ca_uint32_t reg_off;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	QM_QM_DEST_PORT0_PKT_BUF_CFG_t dest_port_pkt_buf_cfg;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG3_EQ0_t cfg3_eq;
       QM_QM_CFG4_EQ0_t cfg4_eq;



	for(i = 0; i <MAX_FF_CPU_COUNT; i++) {

		if(!l3qm_ff_cpuCfg[i].valid)
			continue;

		// eq profile
		eq_profile.wrd = 0;
		eq_profile.bf.eqp0 = l3qm_ff_cpuCfg[i].eq_pool_id;
		eq_profile.bf.eqp1 = L3QM_FASTFWD_EQ_RSVD_POOL_ID;
		eq_profile.bf.rule = 0;
		reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * l3qm_ff_cpuCfg[i].eq_profile_id);
		CA_NE_REG_WRITE(eq_profile.wrd, reg_off);

		// for CPU ports profile selection: 0x16 -> 6, 0x17 -> 7
		dest_port_eq_cfg.bf.profile_sel = l3qm_ff_cpuCfg[i].eq_profile_id;
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * l3qm_ff_cpuCfg[i].cpu_port_id);
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);

		/* enable Empty Buffer pool0 and disable pool1 */
		cfg0_eq.wrd = 0;
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * L3QM_FASTFWD_EQ_RSVD_POOL_ID);
		CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);
		
		cfg0_eq.bf.eq_en = 1;
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

		/* configure total_buffer_num and bid_start */
		cfg1_eq.wrd = 0;
		cfg1_eq.bf.total_buffer_num = l3qm_ff_cpuCfg[i].eq_buffer_cnt;
		cfg1_eq.bf.bid_start = l3qm_ff_cpuCfg[i].eq_buffer_bid_start;

		reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	        if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: CPU eq_id=%d, pool0 bid_start=%d\n", __func__, l3qm_ff_cpuCfg[i].eq_pool_id, cfg1_eq.bf.bid_start);
			printk("%s: CPU pool0 total_buffer_num=%d\n", __func__, cfg1_eq.bf.total_buffer_num);
		}

		/* configure buffer size for Empty Buffer */
		reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		cfg2_eq.wrd = CA_NE_REG_READ(reg_off);

		printk("%s: l3qm_eq_profile_cpu_use_fbm=%d, l3qm_eq_profile_cpu_pool0_fbm_pool_id=%d, l3qm_eq_profile_cpu_pool1_fbm_pool_id=%d\n", __func__, l3qm_eq_profile_cpu_use_fbm, l3qm_eq_profile_cpu_pool0_fbm_pool_id, l3qm_eq_profile_cpu_pool1_fbm_pool_id);

		if (l3qm_eq_profile_cpu_use_fbm) {
			printk("ERROR: fbm is not support, %s, %d\n", __FUNCTION__, __LINE__);
		}
		else {
			cfg2_eq.bf.cpu_eq = 1;
			cfg2_eq.bf.refill_fbm_eqid = 0;

			/* set threshold to be (1500 - 160) / 8, regard the 160 as budget in RX routine */
			/* the budget value should not be too small otherwise the interrupts happened too frequently */
                        cfg2_eq.bf.refill_ths = (L3QM_FASTFWD_EQ0_BUFFER_CNT - ca_ni_napi_budget) / 8;

			cfg2_eq.bf.refill_en = 0;
			cfg2_eq.bf.refill_fbm_eqid = 0;
		}

		cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(l3qm_eq_profile_cpu_pool1_buf_sz);      /* 2048 bytes */

		reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

		printk("%s: i=%d, reg_off=0x%x, cfg2_eq.wrd=0x%x\n", __func__, l3qm_ff_cpuCfg[i].eq_pool_id, reg_off, cfg2_eq.wrd);

	        if (ca_aal_debug & AAL_DBG_L3QM) {
			printk("%s: CPU pool buf_sz=%d, size_index=%d\n", __func__, l3qm_eq_profile_cpu_pool0_buf_sz, cfg2_eq.bf.buffer_size);
		}

		/* CPU pool 0 */
		/* configure cache_eos/domain/snoop/cache */
		reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		/* if QM ACE not turned on, ignore the startup configure settings if acd_cmd=1 */
		cfg3_eq.wrd = l3qm_eq_cfg3_values[l3qm_eq_profile_cpu_pool0_eq_id];
		if (!l3qm_ace_test && cfg3_eq.bf.acd_cmd == 1) {
			cfg3_eq.wrd = 0;
		}
		CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

		/* configure AXI address top 8 bits */
		reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * l3qm_ff_cpuCfg[i].eq_pool_id);
		cfg4_eq.wrd = 0;
		if (l3qm_ace_test) {
			cfg4_eq.bf.axi_top_bit = l3qm_eq_cfg4_values[l3qm_eq_profile_cpu_pool0_eq_id];
		}

		printk("%s: l3qm_eq_profile_cpu_pool0_eq_id=%d, cfg4_eq.bf.axi_top_bit=%d\n", __func__, l3qm_ff_cpuCfg[i].eq_pool_id, cfg4_eq.bf.axi_top_bit);
	        CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);


		/*
		 * L3TE re-init
		*/
		{
		
			aal_l3_te_cb_free_buf_cnt_t port_buf;
			aal_l3_te_cb_free_buf_cnt_mask_t port_buf_mask;
			
			port_buf_mask.u32 = 0;
			memset(&port_buf, 0, sizeof(aal_l3_te_cb_free_buf_cnt_t));

			port_buf_mask.s.cnt1_msb = 1;
			port_buf_mask.s.cnt0_msb = 1;
			port_buf_mask.s.cnt1 = 1;
			port_buf_mask.s.cnt0 = 1;
			
			port_buf.cnt1_msb = 1;
			port_buf.cnt0_msb = 1;
			port_buf.cnt0 = l3qm_ff_cpuCfg[i].eq_buffer_cnt;
			port_buf.cnt1 = 0;
			
            		aal_l3_te_cb_port_free_buf_cnt_set(0, l3qm_ff_cpuCfg[i].cpu_port_id, port_buf_mask, &port_buf);
		}
	}

}
#endif

static void aal_l3qm_init_empty_buffer_DQ(void)
{
	int i;
	ca_uint32_t reg_off;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG2_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;

	/* one profile for DQ or normal NI ports */
	eq_profile.wrd = 0;
	eq_profile.bf.eqp0 = l3qm_eq_profile_dq_pool0_eq_id;
	eq_profile.bf.eqp1 = l3qm_eq_profile_dq_pool1_eq_id;
	eq_profile.bf.rule = l3qm_eq_profile_dq_rule;
	reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * l3qm_eq_profile_dq_id);
	CA_NE_REG_WRITE(eq_profile.wrd, reg_off);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: DQ pool0 eq_id=%d, rule=%d\n", __func__, l3qm_eq_profile_dq_pool0_eq_id, l3qm_eq_profile_dq_rule);
	}

	dest_port_eq_cfg.wrd = 0;

	/* for CPU ports profile select, profile 0 for DQ at NI ports  */
	for (i = 0; i < CA_NI_TOTAL_NI_PORT; i++) {
		dest_port_eq_cfg.bf.profile_sel = l3qm_eq_profile_dq_id;
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (i + CA_NI_TOTAL_CPU_PORT));
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
	}

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: DQ profile id=%d\n", __func__, l3qm_eq_profile_dq_id);
	}

	/* configure EB pool 0 for DQ */
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	if (l3qm_deepq_test_with_sram)
		cfg0_eq.bf.phy_addr_start = (0xc0028000) >> CA_L3QM_PHY_ADDR_SHIFT;
	else
		cfg0_eq.bf.phy_addr_start = (l3qm_eq_profile_dq_pool0_buff_phy_addr) >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * l3qm_eq_profile_dq_pool0_eq_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: DQ pool0 buff_phy_addr=0x%x\n", __func__, l3qm_eq_profile_dq_pool0_buff_phy_addr);
	}

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = l3qm_eq_profile_dq_pool0_buf_count;
	cfg1_eq.bf.bid_start = l3qm_eq_profile_dq_pool0_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * l3qm_eq_profile_dq_pool0_eq_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: DQ pool0 buf_count=%d, bid_start=%d\n", __func__, l3qm_eq_profile_dq_pool0_buf_count, l3qm_eq_profile_dq_pool0_bid_start);
	}

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * l3qm_eq_profile_dq_pool0_eq_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
	cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(l3qm_eq_profile_dq_pool0_buf_sz);      /* 2048 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: DQ pool0 buf_sz=%d, size_index=%d\n", __func__, l3qm_eq_profile_dq_pool0_buf_sz, cfg2_eq.bf.buffer_size);
	}

        /* configure cache_eos/domain/snoop/cache */
        reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * l3qm_eq_profile_dq_pool0_eq_id);
        cfg3_eq.wrd = l3qm_eq_cfg3_values[l3qm_eq_profile_dq_pool0_eq_id];
        CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

        /* configure AXI address top 8 bits */
        /* because PE0 pool1 locates at DDR start from 0x180000000 */
        reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * l3qm_eq_profile_dq_pool0_eq_id);
	if (l3qm_deepq_test_with_sram)
		cfg4_eq.wrd = 0;
	else
		cfg4_eq.wrd = l3qm_eq_cfg4_values[l3qm_eq_profile_dq_pool0_eq_id];
        CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

        /* configure EB pool 1 for DQ */
        /* set phy_addr_start and enable Empty Buffer */
        cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	if (l3qm_deepq_test_with_sram)
		cfg0_eq.bf.phy_addr_start = (0xc0030000) >> CA_L3QM_PHY_ADDR_SHIFT;
	else
		cfg0_eq.bf.phy_addr_start = (l3qm_eq_profile_dq_pool1_buff_phy_addr) >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * l3qm_eq_profile_dq_pool1_eq_id);
        CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

        if (ca_aal_debug & AAL_DBG_L3QM) {
                printk("%s: DQ pool1 buff_phy_addr=0x%x\n", __func__, l3qm_eq_profile_dq_pool1_buff_phy_addr);
        }

        /* configure total buffer number and bid_start */
        cfg1_eq.wrd = 0;
        cfg1_eq.bf.total_buffer_num = l3qm_eq_profile_dq_pool1_buf_count;
        cfg1_eq.bf.bid_start = l3qm_eq_profile_dq_pool1_bid_start;
        reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * l3qm_eq_profile_dq_pool1_eq_id);
        CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

        if (ca_aal_debug & AAL_DBG_L3QM) {
                printk("%s: DQ pool1 buf_count=%d, bid_start=%d\n", __func__,
                        l3qm_eq_profile_dq_pool1_buf_count, l3qm_eq_profile_dq_pool1_bid_start);
        }

        /* configure buffer size for Empty Buffer */
        reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * l3qm_eq_profile_dq_pool1_eq_id);
        cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
        cfg2_eq.bf.cpu_eq = 0;
        cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(l3qm_eq_profile_dq_pool1_buf_sz);      /* 2048 bytes */
        CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

        if (ca_aal_debug & AAL_DBG_L3QM) {
                printk("%s: DQ pool1 buf_sz=%d, size_index=%d\n", __func__, l3qm_eq_profile_dq_pool1_buf_sz, cfg2_eq.bf.buffer_size);
        }

        /* configure cache_eos/domain/snoop/cache */
        reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * l3qm_eq_profile_dq_pool1_eq_id);
        cfg3_eq.wrd = l3qm_eq_cfg3_values[l3qm_eq_profile_dq_pool1_eq_id];
        CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	/* because PE0 pool1 locates at DDR start from 0x180000000 */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * l3qm_eq_profile_dq_pool1_eq_id);
	if (l3qm_deepq_test_with_sram)
		cfg4_eq.wrd = 0;
	else
		cfg4_eq.wrd = l3qm_eq_cfg4_values[l3qm_eq_profile_dq_pool1_eq_id];
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);
	
}

/* Sending packets to CPU port through L3QM - Receiving */
/* Configure Empty Buffer Pool for CPU and NI and set up the physical start address for NI ports */
static void aal_l3qm_init_empty_buffer_DQ_1(void);
void aal_l3qm_init_empty_buffer(ca_uint8_t is_use_fbm)
{
	aal_l3qm_init_empty_buffer_CPU();

#if defined(CONFIG_LUNA_G3_SERIES) && defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
	aal_l3qm_init_empty_buffer_CPU_fastFwd();
#endif

	if(l3qm_eq0_sram_eq1_dram == 1)
		aal_l3qm_init_empty_buffer_DQ_1();
	else
	aal_l3qm_init_empty_buffer_DQ();

	/* must be called before aal_l3qm_load_eq_config() */
#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_init_cpu256_empty_buffer();
#endif

	aal_l3qm_load_eq_config();

	aal_l3qm_init_qm_axi_attrib();

	aal_l3qm_check_bid_start(0);

	spin_lock_init(&l3qm_epp64_inte_lock);
}

void aal_l3qm_enable_rx(ca_uint8_t enable)
{
	QM_QM_RMU0_CTRL_t rmu0_ctrl;

	rmu0_ctrl.wrd = CA_NE_REG_READ(QM_QM_RMU0_CTRL);
	if (enable) {
		rmu0_ctrl.bf.rx_en = 1;
	}
	else {
		rmu0_ctrl.bf.rx_en = 0;
	}
	CA_NE_REG_WRITE(rmu0_ctrl.wrd, QM_QM_RMU0_CTRL);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: enable=%d\n", __func__, enable);
	}
}

void aal_l3qm_enable_tx(ca_uint8_t enable)
{
	QM_QM_ES_CTRL_t es_ctrl;

	es_ctrl.wrd = CA_NE_REG_READ(QM_QM_ES_CTRL);
	if (enable) {
		es_ctrl.bf.tx_en = 1;
		es_ctrl.bf.cpu_en = 0;
		es_ctrl.bf.ni_en = 0xff;
	}
	else {
		es_ctrl.bf.tx_en = 0;
		es_ctrl.bf.cpu_en = 0;
		es_ctrl.bf.ni_en = 0;
	}
	CA_NE_REG_WRITE(es_ctrl.wrd, QM_QM_ES_CTRL);

        if (ca_aal_debug & AAL_DBG_L3QM) {
                printk("%s: enable=%d\n", __func__, enable);
        }
}

void aal_l3qm_enable_tx_cpu(ca_uint16_t cpu_port, ca_uint8_t enable)
{
        QM_QM_ES_CTRL_t es_ctrl;

        es_ctrl.wrd = CA_NE_REG_READ(QM_QM_ES_CTRL);
        if (enable) {
                es_ctrl.bf.cpu_en |= (1 << cpu_port);
        }
        else {
                es_ctrl.bf.cpu_en &= ~(1 << cpu_port);
        }
        CA_NE_REG_WRITE(es_ctrl.wrd, QM_QM_ES_CTRL);

        if (ca_aal_debug & AAL_DBG_L3QM) {
                printk("%s: cpu_port=%d, enable=%d, es_ctrl.bf.cpu_en=0x%x\n", __func__, cpu_port, enable, es_ctrl.bf.cpu_en);
        }
}

void aal_l3qm_config_DWRR(void)
{
	int i;
	QM_QM_DRR_WEIGHT_BASE_t drr_weight_base;
	QM_QM_VOQ3_0_WEIGHT_RATIO_CFG0_t voq3_0_weight_ratio;
	QM_QM_VOQ7_4_WEIGHT_RATIO_CFG0_t voq7_4_weight_ratio;

	QM_QM_PORT7_0_WEIGHT_RATIO_CFG_t   port7_0_weight_ratio;
	QM_QM_PORT15_8_WEIGHT_RATIO_CFG_t   port15_8_weight_ratio;

	/* configure DRR Scheduler Weight Base */
	drr_weight_base.wrd = CA_NE_REG_READ(QM_QM_DRR_WEIGHT_BASE);

	CA_NE_REG_WRITE(drr_weight_base.wrd, QM_QM_DRR_WEIGHT_BASE);

	/* Configure Weight Ratio for VOQ */
	for (i = 0; i < QM_QM_VOQ3_0_WEIGHT_RATIO_CFG0_COUNT; i++) {
		voq3_0_weight_ratio.bf.voq0 = 1;
		voq3_0_weight_ratio.bf.voq1 = 1;
		voq3_0_weight_ratio.bf.voq2 = 1;
		voq3_0_weight_ratio.bf.voq3 = 1;
		CA_NE_REG_WRITE(voq3_0_weight_ratio.wrd, QM_QM_VOQ3_0_WEIGHT_RATIO_CFG0 + (QM_QM_VOQ3_0_WEIGHT_RATIO_CFG0_STRIDE * i));
	}
	for (i = 0; i < QM_QM_VOQ7_4_WEIGHT_RATIO_CFG0_COUNT; i++) {
		voq7_4_weight_ratio.bf.voq4 = 1;
		voq7_4_weight_ratio.bf.voq5 = 1;
		voq7_4_weight_ratio.bf.voq6 = 1;
		voq7_4_weight_ratio.bf.voq7 = 1;
		CA_NE_REG_WRITE(voq3_0_weight_ratio.wrd, QM_QM_VOQ7_4_WEIGHT_RATIO_CFG0 + (QM_QM_VOQ7_4_WEIGHT_RATIO_CFG0_STRIDE * i));
	}

	/* Configure Weight Ratio for CPU ports */
	port7_0_weight_ratio.bf.port0 = 1;
	port7_0_weight_ratio.bf.port1 = 1;
	port7_0_weight_ratio.bf.port2 = 1;
	port7_0_weight_ratio.bf.port3 = 1;
	port7_0_weight_ratio.bf.port4 = 1;
	port7_0_weight_ratio.bf.port5 = 1;
	port7_0_weight_ratio.bf.port6 = 1;
	port7_0_weight_ratio.bf.port7 = 1;
	CA_NE_REG_WRITE(port7_0_weight_ratio.wrd, QM_QM_PORT7_0_WEIGHT_RATIO_CFG);

	/* Configure Weight Ratio for NI ports */
	/* the DRR_WEIGHT_BASE*weight_ratio=128*6=768= should equal to (max packet size/2)=1514/2=757 */
	port15_8_weight_ratio.bf.port8  = 1;
	port15_8_weight_ratio.bf.port9  = 1;
	port15_8_weight_ratio.bf.port10 = 1;
	port15_8_weight_ratio.bf.port11 = 1;
	port15_8_weight_ratio.bf.port12 = 1;
	port15_8_weight_ratio.bf.port13 = 1;
	port15_8_weight_ratio.bf.port14 = 1;
	port15_8_weight_ratio.bf.port15 = 1;
	CA_NE_REG_WRITE(port15_8_weight_ratio.wrd, QM_QM_PORT15_8_WEIGHT_RATIO_CFG);
}

void aal_l3qm_enable_int_src(void)
{
	QM_QM_INT_SRCE_t	int_srcE;

	int_srcE.wrd = 0;
	int_srcE.bf.header_ecc_error_int_srcE 		= 1;
	int_srcE.bf.voqinfo_ecc_error_int_srcE 		= 1;
	int_srcE.bf.mmu_lkup_bid_out_of_range_int_srcE 	= 1;
	int_srcE.bf.rmu0_no_buffer_drop_int_srcE	= 1;
	int_srcE.bf.rmu0_check_error_int_srcE		= 1;
	int_srcE.bf.rmu0_fifo_error_int_srcE		= 1;
	int_srcE.bf.ni_bridge_fifo_error_int_srcE	= 1;
	int_srcE.bf.dqm_fifo_error_int_srcE		= 1;
	int_srcE.bf.eqm_fifo_error_int_srcE		= 1;
	int_srcE.bf.cpuepp_fifo_error_int_srcE		= 1;
	int_srcE.bf.tmu_fifo_error_int_srcE		= 1;
	int_srcE.bf.eqm_cfg_error_int_srcE		= 1;
	int_srcE.bf.axiarb_fifo_error_int_srcE		= 1;
	int_srcE.bf.pkt_lenght_error_int_srcE		= 1;
	int_srcE.bf.fbm_returns_no_buffer_int_srcE	= 0;
	int_srcE.bf.eq_overflow_int_srcE		= 1;
	int_srcE.bf.axim_rmu0_resp_error_int_srcE	= 1;
	int_srcE.bf.axim_cpuepp_resp_error_int_srcE	= 1;
	int_srcE.bf.axim_qm_int_srcE			= 1;
	CA_NE_REG_WRITE(int_srcE.wrd, QM_QM_INT_SRCE);
}

ca_uint32_t aal_l3qm_read_rx_interrupt_status(void)
{
	QM_QM_VOQ_STATUS0_t	voq_status0;
	QM_QM_VOQ_STATUS1_t	voq_status1;
	QM_QM_VOQ_STATUS2_t	voq_status2;
	QM_QM_VOQ_STATUS3_t	voq_status3;

	voq_status0.wrd = CA_NE_REG_READ(QM_QM_VOQ_STATUS0);
	voq_status1.wrd = CA_NE_REG_READ(QM_QM_VOQ_STATUS1);
	voq_status2.wrd = CA_NE_REG_READ(QM_QM_VOQ_STATUS2);
	voq_status3.wrd = CA_NE_REG_READ(QM_QM_VOQ_STATUS3);

	printk("%s: voq_status0.wrd=0x%x, voq_status1.wrd=0x%x, voq_status2.wrd=0x%x, voq_status3.wrd=0x%x\n", __func__,
		voq_status0.wrd, voq_status1.wrd, voq_status2.wrd, voq_status3.wrd);

	return 0;
}

void aal_l3qm_write_rx_interrupt_status(ca_uint32_t status)
{
	QM_QM_INT_SRC_t         int_src;

	printk("%s: status = 0x%x\n", __func__, status);
	int_src.wrd = status;
	CA_NE_REG_WRITE(int_src.wrd, QM_QM_INT_SRC);
}

void aal_l3qm_enable_refill_threshold_interrupt(ca_uint16_t eqid, ca_uint8_t enable)
{
        QM_QM_EQ_REFILL_INTERRUPT_EN_t refill_int_en;

        refill_int_en.wrd = CA_NE_REG_READ(QM_QM_EQ_REFILL_INTERRUPT_EN);
        if (enable)
                refill_int_en.bf.eq_refill_en |= (1 << eqid);
        else
                refill_int_en.bf.eq_refill_en &= ~(1 << eqid);

        CA_NE_REG_WRITE(refill_int_en.wrd, QM_QM_EQ_REFILL_INTERRUPT_EN);
}

void aal_l3qm_setup_refill_threshold_interrup(void)
{
	CA_NE_REG_WRITE(0, QM_QM_EQ_REFILL_INTERRUPT_EN);
	aal_l3qm_enable_refill_threshold_interrupt(l3qm_eq_profile_cpu_pool0_eq_id, 1);
	aal_l3qm_enable_refill_threshold_interrupt(l3qm_eq_profile_cpu_pool1_eq_id, 1);
}

void aal_l3qm_enable_cpu_epp_interrupt(ca_uint16_t cpu_port, ca_uint8_t enable)
{
	QM_QM_EPP64_P3_0_INTERRUPT_EN_t	p3_0_int_en;
	QM_QM_EPP64_P7_4_INTERRUPT_EN_t p7_4_int_en;
	unsigned long flags;

	/* these registers are shared by all of CPUs */
	spin_lock_irqsave(&l3qm_epp64_inte_lock, flags);

	if (cpu_port >= 0 && cpu_port <= 3) {
		p3_0_int_en.wrd = CA_NE_REG_READ(QM_QM_EPP64_P3_0_INTERRUPT_EN);
		switch (cpu_port) {
			case 0:
				if (enable)
					p3_0_int_en.bf.p0_int_en = 0xff;
				else
					p3_0_int_en.bf.p0_int_en = 0;
				break;
			case 1:
                                if (enable)
                                        p3_0_int_en.bf.p1_int_en = 0xff;
                                else
                                        p3_0_int_en.bf.p1_int_en = 0;
                                break;
			case 2:
                                if (enable)
                                        p3_0_int_en.bf.p2_int_en = 0xff;
                                else
                                        p3_0_int_en.bf.p2_int_en = 0;
                                break;
			case 3:
                                if (enable)
                                        p3_0_int_en.bf.p3_int_en = 0xff;
                                else
                                        p3_0_int_en.bf.p3_int_en = 0;
                                break;
		}
		CA_NE_REG_WRITE(p3_0_int_en.wrd, QM_QM_EPP64_P3_0_INTERRUPT_EN);
	}
	else {
		p7_4_int_en.wrd = CA_NE_REG_READ(QM_QM_EPP64_P7_4_INTERRUPT_EN);
		switch (cpu_port) {
			case 4:
				if (enable)
					p7_4_int_en.bf.p4_int_en = 0xff;
				else
					p7_4_int_en.bf.p4_int_en = 0;
				break;
			case 5:
				if (enable)
					p7_4_int_en.bf.p5_int_en = 0xff;
				else
					p7_4_int_en.bf.p5_int_en = 0;
				break;
			case 6:
				if (enable)
					p7_4_int_en.bf.p6_int_en = 0xff;
				else
					p7_4_int_en.bf.p6_int_en = 0;
				break;
			case 7:
				if (enable)
					p7_4_int_en.bf.p7_int_en = 0xff;
				else
					p7_4_int_en.bf.p7_int_en = 0;
				break;
		}
		CA_NE_REG_WRITE(p7_4_int_en.wrd, QM_QM_EPP64_P7_4_INTERRUPT_EN);
	}
	spin_unlock_irqrestore(&l3qm_epp64_inte_lock, flags);
}

int aal_l3qm_insert_magic_number(void *addr)
{
	int i;
        ca_uint32_t *magic_number = (ca_uint32_t *)addr;

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: addr=%p, l3qm_magic_marker_len=%d\n", __func__, addr, l3qm_magic_marker_len);
	}

        for (i = 0; i < l3qm_magic_marker_len/4; i++) {
                *magic_number++ = CA_L3QM_MAGIC_NUMBER;
        }
	return l3qm_magic_marker_len;
}

ca_status_t aal_l3qm_check_magic_number(void *addr)
{
        int i;
        ca_uint32_t *magic_number = (ca_uint32_t *)addr;

        for (i = 0; i < l3qm_magic_marker_len/4; i++) {
                if (*magic_number != CA_L3QM_MAGIC_NUMBER) {
			printk("%s: addr=%p, magic_number=0x%x\n", __func__, magic_number, *magic_number);
			return CA_E_ERROR;
		}
		magic_number++;
        }
        return CA_E_OK;
}

static void aal_l3qm_dump_sram_map(void)
{
	printk("%s: Dump SRAM map...\n", __func__);

	if (l3qm_main_hash_bm_test) {
		printk("[ Main hash ]\n");
		printk("overflow hash fib start addr = 0x%llx\n", (ca_uint64_t)l3fe_main_hash_overflow_hash_fib_start_addr);
		printk("overflow hash fib physical addr = 0x%x\n", l3fe_main_hash_overflow_hash_fib_phy_addr);
		printk("overflow hash fib length = %d\n", l3fe_main_hash_overflow_hash_fib_length);
		printk("overflow hash fib count = %d\n", l3fe_main_hash_overflow_hash_fib_count);

		printk("default hash fib start addr = 0x%llx\n", (ca_uint64_t)l3fe_main_hash_default_hash_fib_start_addr);
		printk("default hash fib physical addr = 0x%x\n", l3fe_main_hash_default_hash_fib_phy_addr);
		printk("default hash fib length = %d\n", l3fe_main_hash_default_hash_fib_length);
		printk("default hash fib count = %d\n", l3fe_main_hash_default_hash_fib_count);

		printk("hash action cache fib start addr = 0x%llx\n", (ca_uint64_t)l3fe_main_hash_action_cache_fib_start_addr);
		printk("hash action cache fib physical addr = 0x%x\n", l3fe_main_hash_action_cache_fib_phy_addr);
		printk("hash action cache fib length = %d\n", l3fe_main_hash_action_cache_fib_length);
		printk("hash action cache fib count = %d\n", l3fe_main_hash_action_cache_fib_count);
	}
}

static ca_status_t aal_l3qm_create_sram_map(void *sram_base, ca_uint32_t sram_size, void *sram_phy_base)
{
	ca_uint32_t size = 0;
	ca_uint8_t *sram_start_addr = sram_base;
	ca_uint8_t *sram_phy_addr = sram_phy_base;

	if (l3qm_main_hash_bm_test) {
		/* Main hash overflow action */
		l3fe_main_hash_overflow_hash_fib_start_addr = (ca_uint_t)sram_start_addr;
		l3fe_main_hash_overflow_hash_fib_phy_addr   = (ca_uint_t)sram_phy_base;
		size += l3fe_main_hash_overflow_hash_fib_length * l3fe_main_hash_overflow_hash_fib_count;

		/* Magic number */
		size += aal_l3qm_insert_magic_number(sram_start_addr + size);

		/* Main hash default action */
		l3fe_main_hash_default_hash_fib_start_addr = (ca_uint_t)sram_start_addr + size;
		l3fe_main_hash_default_hash_fib_phy_addr = (ca_uint_t)sram_phy_addr + size;
		size += l3fe_main_hash_default_hash_fib_length * l3fe_main_hash_default_hash_fib_count;

		/* Magic number */
		size += aal_l3qm_insert_magic_number(sram_start_addr + size);

		/* Main hash action */
		l3fe_main_hash_action_cache_fib_start_addr = (ca_uint_t)sram_start_addr + size;
		l3fe_main_hash_action_cache_fib_phy_addr = (ca_uint_t)sram_phy_addr + size;
		size += l3fe_main_hash_action_cache_fib_length * l3fe_main_hash_action_cache_fib_count;

		/* Magic number */
		size += aal_l3qm_insert_magic_number(sram_start_addr + size);
	}

	printk("%s: total required SRAM size 0x%x\n", __func__, size);

	if (size > sram_size) {
		printk(KERN_ERR "%s: out of SRAM space! (required=0x%x, total=0x%x)\n", __func__, size, sram_size);
		printk(KERN_ERR "%s: NE driver initialized failed!!!\n", __func__);
	}
	BUG_ON(size > sram_size);

	aal_l3qm_dump_sram_map();

#if defined(CONFIG_LUNA_G3_SERIES)	
	sram_dts_conf_size = sram_size;
	sram_cur_used_size += size;
#endif

	return CA_E_OK;
}

static void aal_l3qm_dump_ddr_map(void)
{
	printk("%s: Dump DDR map...\n", __func__);
	printk("[ DDR coherent ]\n");

	printk("CPU pool0 buffer start addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_cpu_pool0_start_addr);
	printk("CPU pool0 buffer physical addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_cpu_pool0_phy_addr);
	printk("CPU pool0 buffer count = %d\n", l3qm_eq_profile_cpu_pool0_buf_count);
	printk("CPU pool0 buffer size = %d\n", l3qm_eq_profile_cpu_pool0_buf_sz);
	printk("CPU pool0 consumed size = %lld\n", (ca_uint64_t)l3qm_eq_profile_cpu_pool1_start_addr - l3qm_eq_profile_cpu_pool0_start_addr);

        printk("CPU pool1 buffer start addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_cpu_pool1_start_addr);
        printk("CPU pool1 buffer physical addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_cpu_pool1_phy_addr);
        printk("CPU pool1 buffer count = %d\n", l3qm_eq_profile_cpu_pool1_buf_count);
        printk("CPU pool1 buffer size = %d\n", l3qm_eq_profile_cpu_pool1_buf_sz);
	printk("CPU pool1 consumed size = %lld\n", (ca_uint64_t)l3qm_epp_profile_cpu_rx_fifo_start_addr - l3qm_eq_profile_cpu_pool1_start_addr);

        printk("CPU RX FIFO buffer start addr = 0x%llx\n", (ca_uint64_t)l3qm_epp_profile_cpu_rx_fifo_start_addr);
        printk("CPU RX FIFO buffer physical addr = 0x%llx\n", (ca_uint64_t)l3qm_epp_profile_cpu_rx_fifo_phy_addr);
        printk("CPU RX FIFO count = %d\n", l3qm_epp_profile_cpu_cn);
        printk("CPU RX FIFO size = %d\n", l3qm_epp_profile_cpu_sz);
	printk("CPU RX FIFO consumed size = %d\n", l3qm_epp_profile_cpu_cn * l3qm_epp_profile_cpu_sz);

	printk("[ DDR non-cache ]\n");
        printk("DEEPQ pool0 buffer start addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_dq_pool0_buff_start_addr);
	printk("DEEPQ pool0 buffer physical addr = 0x%x\n", l3qm_eq_profile_dq_pool0_buff_phy_addr);
        printk("DEEPQ pool0 buffer count = %d\n", l3qm_eq_profile_dq_pool0_buf_count);
        printk("DEEPQ pool0 buffer size = %d\n", l3qm_eq_profile_dq_pool0_buf_sz);
	printk("DEEPQ pool0 consumed size = %lld\n", (ca_uint64_t)l3qm_eq_profile_dq_pool1_buff_start_addr - l3qm_eq_profile_dq_pool0_buff_start_addr);

        printk("DEEPQ pool1 buffer start addr = 0x%llx\n", (ca_uint64_t)l3qm_eq_profile_dq_pool1_buff_start_addr);
	printk("DEEPQ pool1 buffer physical addr = 0x%x\n", l3qm_eq_profile_dq_pool1_buff_phy_addr);
        printk("DEEPQ pool1 buffer count = %d\n", l3qm_eq_profile_dq_pool1_buf_count);
        printk("DEEPQ pool1 buffer size = %d\n", l3qm_eq_profile_dq_pool1_buf_sz);
	printk("DEEPQ pool1 consumed size = %lld\n", (ca_uint64_t)l3fe_main_hash_table_start_addr - l3qm_eq_profile_dq_pool1_buff_start_addr);

        printk("Main Hash table entry start addr = 0x%llx\n", (ca_uint64_t)l3fe_main_hash_table_start_addr);
	printk("Main Hash table entry physical addr = 0x%x\n", l3fe_main_hash_table_phy_addr);
        printk("Main Hash tabel entry length = %d\n", l3fe_main_hash_table_entry_length);
        printk("Main Hash table entry count = %d\n", l3fe_main_hash_table_entry_count);
	printk("Main Hash table consumed size = %lld\n", (ca_uint64_t)l3fe_main_action_table_start_addr - l3fe_main_hash_table_start_addr);

        printk("Main FIB action table entry addr = 0x%llx\n", (ca_uint64_t)l3fe_main_action_table_start_addr);
	printk("Main FIB action table entry physical addr = 0x%x\n", l3fe_main_action_table_phy_addr);
        printk("Main FIB action table entry length = %d\n", l3fe_main_hash_action_table_entry_length);
        printk("Main FIB action table entry count = %d\n", l3fe_main_hash_action_table_entry_count);
	printk("Main FIB action table consumed size = %lld\n", (ca_uint64_t)l3qm_eq_profile_pe0_pool1_buff_start_addr - l3fe_main_action_table_start_addr);

        if (l3qm_main_hash_bm_test == 0) {
                printk("Main hash overflow hash fib start addr = 0x%llx\n", (ca_uint64_t)l3fe_main_hash_overflow_hash_fib_start_addr);
                printk("Main hash overflow hash fib physical addr = 0x%x\n", l3fe_main_hash_overflow_hash_fib_phy_addr);
                printk("Main hash overflow hash fib length = %d\n", l3fe_main_hash_overflow_hash_fib_length);
                printk("Main hash overflow hash fib count = %d\n", l3fe_main_hash_overflow_hash_fib_count);

                printk("Main hash default hash fib start addr = 0x%llx\n", (ca_uint64_t)(ca_uint64_t)l3fe_main_hash_default_hash_fib_start_addr);
                printk("Main hash default hash fib physical addr = 0x%x\n", l3fe_main_hash_default_hash_fib_phy_addr);
                printk("Main hash default hash fib length = %d\n", l3fe_main_hash_default_hash_fib_length);
                printk("Main hash default hash fib count = %d\n", l3fe_main_hash_default_hash_fib_count);

                printk("Main hash hash action cache fib start addr = 0x%llx\n", (ca_uint64_t)(ca_uint64_t)l3fe_main_hash_action_cache_fib_start_addr);
                printk("Main hash hash action cache fib physical addr = 0x%x\n", l3fe_main_hash_action_cache_fib_phy_addr);
                printk("Main hash hash action cache fib length = %d\n", l3fe_main_hash_action_cache_fib_length);
                printk("Main hash hash action cache fib count = %d\n", l3fe_main_hash_action_cache_fib_count);
        }

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_dump_cpu256_ddr_map();
#endif
}

static ca_status_t aal_l3qm_create_ddr_map(void *ddr_coherent_base, void *ddr_noncache_base, void *ddr_noncache_phy_base)
{
	ca_uint32_t size = 0;
	ca_uint8_t *ddr_coherent_start_addr = ddr_coherent_base;
	ca_uint8_t *ddr_noncache_start_addr = ddr_noncache_base;
 	ca_uint8_t *ddr_noncache_phy_addr = ddr_noncache_phy_base;

	/* DDR coherent */
	/* CPU pool0 and pool1 */
	/* ddr_coherent_base is physical address */

#if defined(CONFIG_LUNA_G3_SERIES)

	/* CPU RX FIFO ,  for memory shrink, Patched from Aaron/Cortina 1116 */
	l3qm_epp_profile_cpu_rx_fifo_start_addr = (ca_uint_t)phys_to_virt((ca_uint_t)ddr_coherent_start_addr);
#else
	l3qm_eq_profile_cpu_pool0_phy_addr = (ca_uint_t)ddr_coherent_start_addr + size;
	l3qm_eq_profile_cpu_pool0_start_addr = (ca_uint_t)phys_to_virt(l3qm_eq_profile_cpu_pool0_phy_addr);
	size = l3qm_eq_profile_cpu_pool0_buf_sz * l3qm_eq_profile_cpu_pool0_buf_count;

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(l3qm_eq_profile_cpu_pool0_start_addr + size));

	l3qm_eq_profile_cpu_pool1_start_addr = ((l3qm_eq_profile_cpu_pool0_start_addr + size + l3qm_eq_buffer_align_size) /
			l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
	l3qm_eq_profile_cpu_pool1_phy_addr = virt_to_phys((void *)l3qm_eq_profile_cpu_pool1_start_addr);
	size = l3qm_eq_profile_cpu_pool1_buf_sz * l3qm_eq_profile_cpu_pool1_buf_count;

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_eq_profile_cpu_pool1_start_addr + size);

	/* CPU RX FIFO */
	l3qm_epp_profile_cpu_rx_fifo_start_addr = ((l3qm_eq_profile_cpu_pool1_start_addr + size + l3qm_eq_buffer_align_size) /
			l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
#endif
	l3qm_epp_profile_cpu_rx_fifo_phy_addr = virt_to_phys((void *)l3qm_epp_profile_cpu_rx_fifo_start_addr);
	size = l3qm_epp_profile_cpu_sz * l3qm_epp_profile_cpu_cn + (l3qm_magic_marker_len * ca_ni_cpu_cpu_port_count * l3qm_cpu_voq_per_port);

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_epp_profile_cpu_rx_fifo_start_addr + size);

#if defined(CONFIG_LUNA_G3_SERIES)
   /* for memory shrink , Patched from Aaron/Cortina 1116 */
	//size = (l3qm_epp_profile_cpu_rx_fifo_start_addr + size) - l3qm_eq_profile_cpu_pool0_start_addr;
#else
	size = (l3qm_epp_profile_cpu_rx_fifo_start_addr + size) - l3qm_eq_profile_cpu_pool0_start_addr;
#endif

	printk("%s: total size of DDR coherent consumed=0x%x\n", __func__, size);

	if (size > ca_ne_ddr_coherent_reserved_size) {
		printk("%s: out of DDR Coherent! NE driver initialized failed!!!\n", __func__);
		aal_l3qm_dump_ddr_map();
	}
	BUG_ON(size > ca_ne_ddr_coherent_reserved_size);

	/* DDR non-cacheable */
	/* DEEPQ pool0 buffer */
	size = 0;
	l3qm_eq_profile_dq_pool0_buff_start_addr = (ca_uint_t)ddr_noncache_start_addr + size;
	l3qm_eq_profile_dq_pool0_buff_phy_addr = (ca_uint32_t)(uintptr_t) ddr_noncache_phy_addr + size;
	
	if(l3qm_eq0_sram_eq1_dram == 1) {
		// eq0 use sram, so keep size as 0 and continue eq1 config.
		size = 0;
	} else {
		size = l3qm_eq_profile_dq_pool0_buf_count * l3qm_eq_profile_dq_pool0_buf_sz;
	}

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_eq_profile_dq_pool0_buff_start_addr + size);

	/* DEEPQ pool1 buffer */
	l3qm_eq_profile_dq_pool1_buff_start_addr = (ca_uint_t)l3qm_eq_profile_dq_pool0_buff_start_addr + size;
	l3qm_eq_profile_dq_pool1_buff_phy_addr = (ca_uint32_t)l3qm_eq_profile_dq_pool0_buff_phy_addr + size;
	size = l3qm_eq_profile_dq_pool1_buf_count * l3qm_eq_profile_dq_pool1_buf_sz;

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_eq_profile_dq_pool1_buff_start_addr + size);

	/* Main hash */
	l3fe_main_hash_table_start_addr = (ca_uint_t)l3qm_eq_profile_dq_pool1_buff_start_addr + size;
	l3fe_main_hash_table_phy_addr = (ca_uint_t)l3qm_eq_profile_dq_pool1_buff_phy_addr + size;
	size = l3fe_main_hash_table_entry_count * l3fe_main_hash_table_entry_length;

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3fe_main_hash_table_start_addr + size);

	/* Main FIB */
	l3fe_main_action_table_start_addr = (ca_uint_t)l3fe_main_hash_table_start_addr + size;
	l3fe_main_action_table_phy_addr = (ca_uint_t)l3fe_main_hash_table_phy_addr + size;
	size = l3fe_main_hash_action_table_entry_count * l3fe_main_hash_action_table_entry_length;

	/* Magic number */
	size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3fe_main_action_table_start_addr + size);

       if (l3qm_main_hash_bm_test == 0) {
                /* Main hash overflow action */
                l3fe_main_hash_overflow_hash_fib_start_addr = (ca_uint_t)l3fe_main_action_table_start_addr + size;
                l3fe_main_hash_overflow_hash_fib_phy_addr   = (ca_uint_t)l3fe_main_action_table_phy_addr + size;
                size += l3fe_main_hash_overflow_hash_fib_length * l3fe_main_hash_overflow_hash_fib_count;

                /* Magic number */
                size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3fe_main_action_table_start_addr + size);

                /* Main hash default action */
                l3fe_main_hash_default_hash_fib_start_addr = (ca_uint_t)l3fe_main_action_table_start_addr + size;
                l3fe_main_hash_default_hash_fib_phy_addr = (ca_uint_t)l3fe_main_action_table_phy_addr + size;
                size += l3fe_main_hash_default_hash_fib_length * l3fe_main_hash_default_hash_fib_count;

                /* Magic number */
                size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3fe_main_action_table_start_addr + size);

                /* Main hash action */
                l3fe_main_hash_action_cache_fib_start_addr = (ca_uint_t)l3fe_main_action_table_start_addr + size;
                l3fe_main_hash_action_cache_fib_phy_addr = (ca_uint_t)l3fe_main_action_table_phy_addr + size;
                size += l3fe_main_hash_action_cache_fib_length * l3fe_main_hash_action_cache_fib_count;

                /* Magic number */
                size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3fe_main_action_table_start_addr + size);
        }

	/* PE0 EQ pool1 */
	l3qm_eq_profile_pe0_pool1_buff_start_addr = (ca_uint_t)((l3fe_main_action_table_start_addr + size + l3qm_eq_buffer_align_size) /
								l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
	l3qm_eq_profile_pe0_pool1_buff_phy_addr = (ca_uint_t)((l3fe_main_action_table_phy_addr + size + l3qm_eq_buffer_align_size) /
			l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
	size = l3qm_eq_profile_pe0_pool1_buf_count * l3qm_eq_profile_pe0_pool1_buf_sz;

        /* Magic number */
        size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_eq_profile_pe0_pool1_buff_start_addr + size);

        /* PE1 EQ pool1 */
        l3qm_eq_profile_pe1_pool1_buff_start_addr = (ca_uint_t)((l3qm_eq_profile_pe0_pool1_buff_start_addr + size + l3qm_eq_buffer_align_size) /
                                                                l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
        l3qm_eq_profile_pe1_pool1_buff_phy_addr = (ca_uint_t)((l3qm_eq_profile_pe0_pool1_buff_phy_addr + size + l3qm_eq_buffer_align_size) /
                                                                l3qm_eq_buffer_align_size) * l3qm_eq_buffer_align_size;
        size = l3qm_eq_profile_pe1_pool1_buf_count * l3qm_eq_profile_pe1_pool1_buf_sz;

        /* Magic number */
        size += aal_l3qm_insert_magic_number((void *)(ca_uint_t) l3qm_eq_profile_pe1_pool1_buff_start_addr + size);

#if defined(CONFIG_NE_CPU256)
        if (support_cpu256_in_linux)
		size = aal_l3qm_create_cpu256_ddr_map();
#endif

        printk("%s: total size of DDR non-cache consumed=0x%x\n", __func__, size);

        if (size > ca_ne_ddr_noncache_reserved_size) {
                printk("%s: out of DDR non-cache! NE driver initialized failed!!!\n", __func__);
		aal_l3qm_dump_ddr_map();
	}
        BUG_ON(size > ca_ne_ddr_noncache_reserved_size);

	aal_l3qm_dump_ddr_map();

	return CA_E_OK;
}

void aal_l3qm_init_startup_config(void)
{
	/* read additional configure */
	/* do not use CFG_ID_L3QM_MAGIC_MARKER_LEN, set l3qm_magic_marker_len=0 always */
	aal_scfg_read(CFG_ID_L3QM_DESC_SIZE, 1, &l3qm_desc_size);
	aal_scfg_read(CFG_ID_L3QM_DESC_PER_EPP, 1, &l3qm_desc_per_epp);
	//aal_scfg_read(CFG_ID_L3QM_EQ0_SRAM_EQ1_DRAM, 1, &l3qm_eq0_sram_eq1_dram);

	aal_scfg_read(CFG_ID_L3QM_CPU_PORT_COUNT, 4, &l3qm_cpu_port_count);
	aal_scfg_read(CFG_ID_L3QM_CPU_VOQ_PER_PORT, 1, &l3qm_cpu_voq_per_port);
	aal_scfg_read(CFG_ID_L3QM_CPU_EPP_PER_VOQ, 4, &l3qm_cpu_epp_per_voq);
	aal_scfg_read(CFG_ID_L3QM_CPU_POOL_BID_COUNT, 4, &l3qm_cpu_pool_bid_count);
	aal_scfg_read(CFG_ID_L3QM_EQ_BUFFER_ALIGN_SIZE, 4, &l3qm_eq_buffer_align_size);

	aal_scfg_read(CFG_ID_L3QM_BUF_PACKING_SIZE, 1, &l3qm_jumbo_buf_packing_size);
	aal_scfg_read(CFG_ID_L3QM_JUMBO_BUF_PINGPONG_EQS, 1, &l3qm_jumbo_buf_pingpong_eqs);
	aal_scfg_read(CFG_ID_L3QM_LDPID_TO_PON_PORT_MAP_OFFSET, 1, &l3qm_ldpid_to_pon_port_map_offset);

	/* read CPU configure */
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_ID, 1, &l3qm_eq_profile_cpu_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_RULE, 1, &l3qm_eq_profile_cpu_rule);

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_USE_FBM, 1, &l3qm_eq_profile_cpu_use_fbm);
#if defined(CONFIG_CORTINA_BOARD_FPGA)
	/* if no FBM module included the always set l3qm_eq_profile_cpu_use_fbm=0 */
        if (SOC_HAS_FBM() == 0) {
                l3qm_eq_profile_cpu_use_fbm = 0;
                printk("%s: set l3qm_eq_profile_cpu_use_fbm=0 because no FBM module\n", __func__);
        }
#endif

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_EQ_ID, 1, &l3qm_eq_profile_cpu_pool0_eq_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_BUF_COUNT, 4, &l3qm_eq_profile_cpu_pool0_buf_count);

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_BID_START, 4, &l3qm_eq_profile_cpu_pool0_bid_start);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_BUF_SZ, 4, &l3qm_eq_profile_cpu_pool0_buf_sz);

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL0_FBM_POOL_ID, 1, &l3qm_eq_profile_cpu_pool0_fbm_pool_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_EQ_ID, 1, &l3qm_eq_profile_cpu_pool1_eq_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_BUF_COUNT, 4, &l3qm_eq_profile_cpu_pool1_buf_count);

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_BID_START, 4, &l3qm_eq_profile_cpu_pool1_bid_start);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_BUF_SZ, 4, &l3qm_eq_profile_cpu_pool1_buf_sz);

	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_CPU_POOL1_FBM_POOL_ID, 1, &l3qm_eq_profile_cpu_pool1_fbm_pool_id);
	aal_scfg_read(CFG_ID_L3QM_EPP_PROFILE_CPU_ID, 1, &l3qm_epp_profile_cpu_id);
	aal_scfg_read(CFG_ID_L3QM_EPP_PROFILE_CPU_MAP_MODE, 4, &l3qm_epp_profile_cpu_map_mode);

	/* read DEEPQ configure */
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_ID, 1, &l3qm_eq_profile_dq_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_RULE, 1, &l3qm_eq_profile_dq_rule);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL0_EQ_ID, 1, &l3qm_eq_profile_dq_pool0_eq_id);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL0_BUF_COUNT, 4, &l3qm_eq_profile_dq_pool0_buf_count);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL0_BID_START, 4, &l3qm_eq_profile_dq_pool0_bid_start);
	aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL0_BUF_SZ, 4, &l3qm_eq_profile_dq_pool0_buf_sz);
        aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL1_EQ_ID, 1, &l3qm_eq_profile_dq_pool1_eq_id);
        aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL1_BUF_COUNT, 4, &l3qm_eq_profile_dq_pool1_buf_count);
        aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL1_BID_START, 4, &l3qm_eq_profile_dq_pool1_bid_start);
        aal_scfg_read(CFG_ID_L3QM_EQ_PROFILE_DQ_POOL1_BUF_SZ, 4, &l3qm_eq_profile_dq_pool1_buf_sz);

	/* read L3FE Main hash configure */
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_OVERFLOW_FIB_LENGTH, 4, &l3fe_main_hash_overflow_hash_fib_length);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_OVERFLOW_FIB_COUNT, 4, &l3fe_main_hash_overflow_hash_fib_count);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_DEFAULT_HASH_FIB_LENGTH, 4, &l3fe_main_hash_default_hash_fib_length);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_DEFAULT_HASH_FIB_COUNT, 4, &l3fe_main_hash_default_hash_fib_count);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_ACTION_CACHE_FIB_LENGTH, 4, &l3fe_main_hash_action_cache_fib_length);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_ACTION_CACHE_FIB_COUNT, 4, &l3fe_main_hash_action_cache_fib_count);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_TABLE_ENTRY_LENGTH, 4, &l3fe_main_hash_table_entry_length);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_TABLE_ENTRY_COUNT, 4, &l3fe_main_hash_table_entry_count);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_ACTION_TABLE_ENTRY_LENGTH, 4, &l3fe_main_hash_action_table_entry_length);
	aal_scfg_read(CFG_ID_L3FE_MAIN_HASH_ACTION_TABLE_ENTRY_COUNT, 4, &l3fe_main_hash_action_table_entry_count);

	/* read CPU port head room first */
	aal_scfg_read(CFG_ID_L3QM_CPU_PORT_HEAD_ROOM_FIRST, 4, &l3qm_cpu_port_head_room_first);

	/* read QM ACE test bit */
	aal_scfg_read(CFG_ID_L3QM_ACE_TEST, 1, &l3qm_ace_test);

	/* read Main Hash Bench Mark test */
	aal_scfg_read(CFG_ID_MAIN_HASH_BM_TEST, 1, &l3qm_main_hash_bm_test);

	/* read EQ QM_CFG3/4 */
	aal_scfg_read(CFG_ID_L3QM_EQ_CFG3_VALUES, CFG_ID_L3QM_EQ_CFG3_VALUES_LEN * sizeof(ca_uint32_t), l3qm_eq_cfg3_values);
	aal_scfg_read(CFG_ID_L3QM_EQ_CFG4_VALUES, CFG_ID_L3QM_EQ_CFG4_VALUES_LEN * sizeof(ca_uint32_t), l3qm_eq_cfg4_values);

	/* read QM AXI ATTRIBUTE */
	aal_scfg_read(CFG_ID_L3QM_AXI_ATTRIB_EQ_VALUES, CFG_ID_L3QM_AXI_ATTRIB_EQ_VALUES_LEN * sizeof(ca_uint32_t), l3qm_axi_attrib_eq_values);

	l3qm_epp_profile_cpu_total_epp_count = (ca_ni_cpu_cpu_port_count * l3qm_cpu_voq_per_port * l3qm_cpu_epp_per_voq);  /* CPU total EPP count */

	aal_scfg_read(CFG_ID_L3QM_AXI_ATTRIB_CPU_EPP_VALUES, CFG_ID_L3QM_AXI_ATTRIB_CPU_EPP_VALUES_LEN * sizeof(ca_uint32_t), l3qm_axi_attrib_cpu_epp_values);

	/* calculate from startup configs */
	l3qm_epp_size = (l3qm_desc_size * l3qm_desc_per_epp);   /* EPP size in byte */

	l3qm_epp_profile_pe0_sz = l3qm_epp_size;
	l3qm_epp_profile_pe0_cn = l3qm_eq_profile_pe0_1_total_epp_count;

	l3qm_epp_profile_pe1_sz = l3qm_epp_size;
	l3qm_epp_profile_pe1_cn = l3qm_eq_profile_pe0_1_total_epp_count;

	l3qm_epp_profile_cpu_sz = l3qm_epp_size;
	l3qm_epp_profile_cpu_cn = l3qm_epp_profile_cpu_total_epp_count;
#if defined(CONFIG_NE_CPU256)
	aal_scfg_read(CFG_ID_SUPPORT_CPU256_IN_LINUX, 1, &support_cpu256_in_linux);
	if (support_cpu256_in_linux) {
		/* start up configure initilaization for CPU256 */
		aal_l3qm_init_cpu256_startup_config();
	}
#endif

#if defined(CONFIG_LUNA_G3_SERIES)
	memset(&rtkScfg, 0, sizeof(rtkScfg));
	aal_scfg_read(CFG_ID_RTK_SPECIAL_FASTFWD_REFILL_BATCH_NUMBER, 4, &rtkScfg.special_fastFwd_refill_batch_number);
	aal_scfg_read(CFG_ID_RTK_SPECIAL_FASTFWD_SMALLEST_PACKET_SIZE, 4, &rtkScfg.smallest_packet_size);
	DQ_bid_start = l3qm_eq_profile_cpu_pool0_buf_count + l3qm_eq_profile_cpu_pool1_buf_count;

#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)

	memset(&l3qm_ff_cpuCfg, 0, sizeof(l3qm_ff_cpuCfg));
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
		// MAX - LAN - WAN -CPU_pool0 - CPU_pool1 -special usage
		l3qm_eq_profile_dq_pool1_buf_count = (AAL_L3QM_MAX_BID - 256 - 92 - l3qm_eq_profile_cpu_pool0_buf_count - l3qm_eq_profile_cpu_pool1_buf_count - L3QM_FASTFWD_4CPU_RSRD_BUFFER_CNT);
		
		l3qm_ff_cpuCfg[0].valid = TRUE;
		l3qm_ff_cpuCfg[0].cpu_port_id = L3QM_FASTFWD_CPU0_PORT_ID;
		l3qm_ff_cpuCfg[0].cpu_lport_id = L3QM_FASTFWD_CPU0_LPORT_ID;
		l3qm_ff_cpuCfg[0].eq_profile_id = L3QM_FASTFWD_EQ0_PROFILE_ID;
		l3qm_ff_cpuCfg[0].eq_pool_id = L3QM_FASTFWD_EQ0_POOL0_ID;
		l3qm_ff_cpuCfg[0].eq_buffer_cnt = L3QM_FASTFWD_EQ0_BUFFER_CNT;
		l3qm_ff_cpuCfg[0].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[0].eq_buffer_cnt;
		
		l3qm_ff_cpuCfg[1].valid = TRUE;
		l3qm_ff_cpuCfg[1].cpu_port_id = L3QM_FASTFWD_CPU1_PORT_ID;
		l3qm_ff_cpuCfg[1].cpu_lport_id = L3QM_FASTFWD_CPU1_LPORT_ID;
		l3qm_ff_cpuCfg[1].eq_profile_id = L3QM_FASTFWD_EQ1_PROFILE_ID;
		l3qm_ff_cpuCfg[1].eq_pool_id = L3QM_FASTFWD_EQ1_POOL0_ID;
		l3qm_ff_cpuCfg[1].eq_buffer_cnt = L3QM_FASTFWD_EQ1_BUFFER_CNT;
		l3qm_ff_cpuCfg[1].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[1].eq_buffer_cnt;
		
		l3qm_ff_cpuCfg[2].valid = TRUE;
		l3qm_ff_cpuCfg[2].cpu_port_id = L3QM_FASTFWD_CPU2_PORT_ID;
		l3qm_ff_cpuCfg[2].cpu_lport_id = L3QM_FASTFWD_CPU2_LPORT_ID;
		l3qm_ff_cpuCfg[2].eq_profile_id = L3QM_FASTFWD_EQ2_PROFILE_ID;
		l3qm_ff_cpuCfg[2].eq_pool_id = L3QM_FASTFWD_EQ2_POOL0_ID;
		l3qm_ff_cpuCfg[2].eq_buffer_cnt = L3QM_FASTFWD_EQ2_BUFFER_CNT;
		l3qm_ff_cpuCfg[2].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[2].eq_buffer_cnt;
		
		l3qm_ff_cpuCfg[3].valid = TRUE;
		l3qm_ff_cpuCfg[3].cpu_port_id = L3QM_FASTFWD_CPU3_PORT_ID;
		l3qm_ff_cpuCfg[3].cpu_lport_id = L3QM_FASTFWD_CPU3_LPORT_ID;
		l3qm_ff_cpuCfg[3].eq_profile_id = L3QM_FASTFWD_EQ3_PROFILE_ID;
		l3qm_ff_cpuCfg[3].eq_pool_id = L3QM_FASTFWD_EQ3_POOL0_ID;
		l3qm_ff_cpuCfg[3].eq_buffer_cnt = L3QM_FASTFWD_EQ3_BUFFER_CNT;
		l3qm_ff_cpuCfg[3].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[3].eq_buffer_cnt;

		
#else
		l3qm_eq_profile_dq_pool1_buf_count = (AAL_L3QM_MAX_BID - 256 - 92 - l3qm_eq_profile_cpu_pool0_buf_count - l3qm_eq_profile_cpu_pool1_buf_count - L3QM_FASTFWD_2CPU_RSRD_BUFFER_CNT);

		l3qm_ff_cpuCfg[0].valid = TRUE;
		l3qm_ff_cpuCfg[0].cpu_port_id = L3QM_FASTFWD_CPU0_PORT_ID;
		l3qm_ff_cpuCfg[0].cpu_lport_id = L3QM_FASTFWD_CPU0_LPORT_ID;
		l3qm_ff_cpuCfg[0].eq_profile_id = L3QM_FASTFWD_EQ0_PROFILE_ID;
		l3qm_ff_cpuCfg[0].eq_pool_id = L3QM_FASTFWD_EQ0_POOL0_ID;
		l3qm_ff_cpuCfg[0].eq_buffer_cnt = L3QM_FASTFWD_EQ0_BUFFER_CNT;
		l3qm_ff_cpuCfg[0].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[0].eq_buffer_cnt;
		
		l3qm_ff_cpuCfg[1].valid = TRUE;
		l3qm_ff_cpuCfg[1].cpu_port_id = L3QM_FASTFWD_CPU1_PORT_ID;
		l3qm_ff_cpuCfg[1].cpu_lport_id = L3QM_FASTFWD_CPU1_LPORT_ID;
		l3qm_ff_cpuCfg[1].eq_profile_id = L3QM_FASTFWD_EQ1_PROFILE_ID;
		l3qm_ff_cpuCfg[1].eq_pool_id = L3QM_FASTFWD_EQ1_POOL0_ID;
		l3qm_ff_cpuCfg[1].eq_buffer_cnt = L3QM_FASTFWD_EQ1_BUFFER_CNT;
		l3qm_ff_cpuCfg[1].eq_buffer_bid_start = DQ_bid_start;
		DQ_bid_start += l3qm_ff_cpuCfg[1].eq_buffer_cnt;

#endif	
	
#endif

#endif

}

void aal_l3qm_set_cpu_rx_fifo_addr(ca_uint_t cpu_rx_fifo_phy_addr, ca_uint_t cpu_rx_fifo_virt_addr)
{
        l3qm_epp_profile_cpu_rx_fifo_phy_addr = cpu_rx_fifo_phy_addr;
        l3qm_epp_profile_cpu_rx_fifo_start_addr = cpu_rx_fifo_virt_addr;
}

ca_uint32_t aal_l3qm_get_cpu_rx_fifo_size(void)
{
        return l3qm_epp_profile_cpu_sz * l3qm_epp_profile_cpu_cn + (l3qm_magic_marker_len * ca_ni_cpu_cpu_port_count * l3qm_cpu_voq_per_port);
}

void aal_l3qm_init_reserved_mem(void *sram_base, ca_uint32_t sram_size,
	void *sram_phy_base, void *ddr_coherent_base, void *ddr_noncache_base, void *ddr_noncache_phy_base)
{
	/* create sram map */
	aal_l3qm_create_sram_map(sram_base, sram_size, sram_phy_base);

	/* create ddr map */
	aal_l3qm_create_ddr_map(ddr_coherent_base, ddr_noncache_base, ddr_noncache_phy_base);
}

void aal_l3qm_init_voq(void)
{
	int i;
	QM_QM_SCH_CFG0_t sch_cfg;

	sch_cfg.wrd = CA_NE_REG_READ(QM_QM_SCH_CFG0);
	for (i = 0; i < QM_QM_SCH_CFG0_COUNT; i++) {

		sch_cfg.bf.voq_en = 0xff;
		CA_NE_REG_WRITE(sch_cfg.wrd, QM_QM_SCH_CFG0 + (i * QM_QM_SCH_CFG0_STRIDE));
	}
#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_init_cpu256_voq();
#endif
}

ca_uint32_t aal_l3qm_check_init_done(void)
{
	int i;
	QM_QM_PHY_PORT_STS_t	init_done;

        /* Check initializtion of NI block is done */
        for (i = 0; i < CA_L3QM_INIT_DONE_LOOP; i++) {
                init_done.wrd = CA_NE_REG_READ(QM_QM_PHY_PORT_STS);
                if (init_done.bf.qm_init_done)
                        break;
        }
        if (i == CA_L3QM_INIT_DONE_LOOP) {
                printk("%s: initializtion of QM block is NOT done!!!\n", __func__);
		return -1;
        }
	return 0;
}

/*
 * Procedure of retrieving a buffer back to CPU:
 * 1. Disable the Empty-buffer pool which physical address is going to be
 * retrieveled from;
 * 2. Enable "buf_retrieval" by writing "1" to this field and
 * the corresponding EQ-ID;
 * 3. read "EQ_RETURN_PHY_ADDR%d_phy_addr", with valid entry indication;
 * 4. repeat "#2" if more buffers need to be retrieved;
 * 5. Write "0" to "buf_retrieval_en" to disable this function;
*/
void aal_l3qm_get_return_phy_addr_enable(int eq_id, int enable)
{
	volatile ca_uint32_t reg_off;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_EQ_PHY_ADDR_RETURN_CTRL_t   phy_addr_return_ctrl;

	if (enable) {
		/* disable Empty Buffer Pool */
		cfg0_eq.wrd = 0;
		cfg0_eq.bf.eq_en = 0;
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * eq_id);
		CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

		/* Enable "buf_retieval" */
		phy_addr_return_ctrl.bf.phy_addr_retrieval_en = 1;
		phy_addr_return_ctrl.bf.eq_id = eq_id;
		CA_NE_REG_WRITE(phy_addr_return_ctrl.wrd, QM_QM_EQ_PHY_ADDR_RETURN_CTRL);
	}
	else {
		/* disable "buf_retieval" */
		phy_addr_return_ctrl.bf.phy_addr_retrieval_en = 0;
		phy_addr_return_ctrl.bf.eq_id = eq_id;
		CA_NE_REG_WRITE(phy_addr_return_ctrl.wrd, QM_QM_EQ_PHY_ADDR_RETURN_CTRL);

		/* enable Empty Buffer Pool */
		cfg0_eq.wrd = 0;
		cfg0_eq.bf.eq_en = 1;
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * eq_id);
		CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);
	}
}

ca_uint32_t aal_l3qm_get_return_phy_addr(int eq_id)
{
	QM_QM_EQ_PHY_ADDR_RETURN_CTRL_t   phy_addr_return_ctrl;
	QM_QM_EQ_RETURN_PHY_ADDR_t        return_phy_addr;

	/* Enable "buf_retieval" */
	phy_addr_return_ctrl.bf.phy_addr_retrieval_en = 1;
	phy_addr_return_ctrl.bf.eq_id = eq_id;
	CA_NE_REG_WRITE(phy_addr_return_ctrl.wrd, QM_QM_EQ_PHY_ADDR_RETURN_CTRL);

        /* read "EQ_RETURN_PHY_ADD%d_phy_addr" */
        return_phy_addr.wrd = CA_NE_REG_READ(QM_QM_EQ_RETURN_PHY_ADDR);

	if (ca_aal_debug & AAL_DBG_L3QM) {
		printk("%s: eqid=%d, retrieval_done=%d, valid=%d\n", __func__, eq_id, return_phy_addr.bf.retrieval_done, return_phy_addr.bf.valid);
		printk("%s: return_phy_addr.wrd=0x%x, phy_addr=0x%x\n", __func__, return_phy_addr.wrd, return_phy_addr.bf.phy_addr << 7);
	}

	/* check retrieval_done and valid bits */
	if (return_phy_addr.bf.retrieval_done) {
		return 0;
	}

	if (return_phy_addr.bf.valid)
		return (return_phy_addr.bf.phy_addr << 7);

        return 0;
}

/* functions for debugging print out used with iROS */
#define VOQ_INFO_ACCESS_TIMEOUT         1000000
#define HEADER_MEMORY_ACCESS_TIMEOUT    1000000
#define BID_PADDR_ACCESS_TIMEOUT    	1000000
ca_status_t aal_l3qm_get_voq_info_mem(ca_device_id_t device_id, ca_uint16_t addr, QM_QM_VOQ_INFO_MEMORY_DATA0_t *data0,
					QM_QM_VOQ_INFO_MEMORY_DATA1_t *data1, QM_QM_VOQ_INFO_MEMORY_DATA2_t *data2)
{
	int i;
	QM_QM_VOQ_INFO_MEMORY_ACCESS_t	access;

	access.bf.access = 1;
	access.bf.rbw = 0;
	access.bf.ADDR = addr;
	CA_NE_REG_WRITE(access.wrd, QM_QM_VOQ_INFO_MEMORY_ACCESS);

	for (i = 0; i < VOQ_INFO_ACCESS_TIMEOUT; i++) {
		access.wrd = CA_NE_REG_READ(QM_QM_VOQ_INFO_MEMORY_ACCESS);
		if (access.bf.access == 0) {
			break;
		}
	}
	if (i == VOQ_INFO_ACCESS_TIMEOUT) {
		printk("%s: VoQ addr=%d access timeout!!!\n", __func__, addr);
		return CA_E_ERROR;
	}
	data0->wrd = CA_NE_REG_READ(QM_QM_VOQ_INFO_MEMORY_DATA0);
	data1->wrd = CA_NE_REG_READ(QM_QM_VOQ_INFO_MEMORY_DATA1);
	data2->wrd = CA_NE_REG_READ(QM_QM_VOQ_INFO_MEMORY_DATA2);

	printk("[addr=%d]\n", addr);
	printk("voq_header=0x%x\n", data0->bf.voq_header);

	printk("voq_header=%d, voq_tail=0x%x, voq_head=0x%x, bcntr=0x%x\n",
		data1->bf.voq_header, data1->bf.voq_tail, data1->bf.voq_head, data1->bf.bcntr);

	printk("bcntr=0x%x, ecc=0x%x\n", data2->bf.bcntr, data2->bf.ecc);

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_get_voq_info_mem);

ca_status_t aal_l3qm_get_header_mem(ca_device_id_t device_id, ca_uint16_t addr, QM_QM_HEADER_MEMORY_DATA0_t *data0, QM_QM_HEADER_MEMORY_DATA1_t *data1)
{
        int i;
        QM_QM_HEADER_MEMORY_ACCESS_t      access;

        access.bf.access = 1;
        access.bf.rbw = 0;
        access.bf.ADDR = addr;
        CA_NE_REG_WRITE(access.wrd, QM_QM_HEADER_MEMORY_ACCESS);

        for (i = 0; i < HEADER_MEMORY_ACCESS_TIMEOUT; i++) {
                access.wrd = CA_NE_REG_READ(QM_QM_HEADER_MEMORY_ACCESS);
                if (access.bf.access == 0) {
                        break;
                }
        }
        if (i == HEADER_MEMORY_ACCESS_TIMEOUT) {
                printk("%s: header memory addr=%d access timeout!!!\n", __func__, addr);
                return CA_E_ERROR;
        }
        data0->wrd = CA_NE_REG_READ(QM_QM_HEADER_MEMORY_DATA0);
        data1->wrd = CA_NE_REG_READ(QM_QM_HEADER_MEMORY_DATA1);

	printk("[addr=%d]\n", addr);

	printk("next_buffer_ptr=%d, sop=%d, eop=%d, pkt_length=%d\n", data0->bf.next_buffer_ptr, data0->bf.sop, data0->bf.eop, data0->bf.pkt_length);

	printk("ecc=%d, pkt_type_ptp=%d\n", data1->bf.ecc, data1->bf.pkt_type_ptp);

        return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_get_header_mem);

ca_status_t aal_l3qm_get_bid_paddr(ca_device_id_t device_id, ca_uint16_t addr, QM_QM_BID_PADDR_LKUP_DATA_t *data)
{
        int i;
        QM_QM_BID_PADDR_LKUP_ACCESS_t      access;

        access.bf.access = 1;
        access.bf.rbw = 0;
        access.bf.ADDR = addr;
        CA_NE_REG_WRITE(access.wrd, QM_QM_BID_PADDR_LKUP_ACCESS);

        for (i = 0; i < BID_PADDR_ACCESS_TIMEOUT; i++) {
                access.wrd = CA_NE_REG_READ(QM_QM_BID_PADDR_LKUP_ACCESS);
                if (access.bf.access == 0) {
                        break;
                }
        }
        if (i == BID_PADDR_ACCESS_TIMEOUT) {
                printk("%s: BID paddr addr=%d access timeout!!!\n", __func__, addr);
                return CA_E_ERROR;
        }
        data->wrd = CA_NE_REG_READ(QM_QM_BID_PADDR_LKUP_DATA);

        printk("%s: paddr=0x%x\n", __func__, data->bf.phy_addr);

        return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_get_bid_paddr);

ca_status_t aal_l3qm_get_tx_rx_cntr(ca_device_id_t device_id)
{
	QM_QM_TX_PKT_CNTR_t  tx_pkt_cntr;
	QM_QM_RMU0_RX_PKT_CNTR_t		rmu0_rx_pkt_cntr;

	tx_pkt_cntr.wrd = CA_NE_REG_READ(QM_QM_TX_PKT_CNTR);
	printk("%s: packtes transmitted from L3QM count=%d\n", __func__, tx_pkt_cntr.bf.cntr);

	rmu0_rx_pkt_cntr.wrd = CA_NE_REG_READ(QM_QM_RMU0_RX_PKT_CNTR);
	printk("%s: packets received by L3QM count=%d\n", __func__, rmu0_rx_pkt_cntr.bf.cntr);

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_get_tx_rx_cntr);

ca_status_t aal_l3qm_get_monitor_eq(ca_device_id_t device_id)
{
	QM_QM_MONITOR0_EQ0_t monitor0_eq0;
	QM_QM_MONITOR1_EQ0_t monitor1_eq0;
	QM_QM_MONITOR2_EQ0_t monitor2_eq0;

	monitor0_eq0.wrd = CA_NE_REG_READ(QM_QM_MONITOR0_EQ0);
	printk("%s: Monitor0 EQ0, head=%d, tail=%d\n", __func__, monitor0_eq0.bf.head, monitor0_eq0.bf.tail);

	monitor1_eq0.wrd = CA_NE_REG_READ(QM_QM_MONITOR1_EQ0);
	printk("%s: Monitor1 EQ0, inactive_head=%d, cntr=%d\n", __func__, monitor1_eq0.bf.inactive_head, monitor1_eq0.bf.cntr);

	monitor2_eq0.wrd = CA_NE_REG_READ(QM_QM_MONITOR2_EQ0);
	printk("%s: Monitor2 EQ0, inactive_tail=%d, inactive_cntr=%d\n", __func__, monitor2_eq0.bf.inactive_tail, monitor2_eq0.bf.inactive_cntr);

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_get_monitor_eq);

ca_status_t aal_l3qm_dump_epps(ca_device_id_t device_id)
{
	int i, j;
	ca_uint32_t reg_off;
	QM_QM_CPU_EPP0_CFG_t cpu_epp0_cfg;
	QM_QM_CPU_EPP_FIFO_CFG_profile0_t cpu_epp_fifo_cfg_profile;
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_t cpu_epp64_fifo_paddr_start;
	QM_QM_CPU_EPP_FIFO0_0_CFG_t cpu_epp_fifo_cfg;

	/* CPU EPP Port Configuration Register */
	/* Select per-destionation VOQ-to-EPP-FIFO-Ring map-mode */
	printk("=== QM_QM_CPU_EPP0_CFG ===\n");
	for (i = 0; i < CA_NI_TOTAL_CPU_PORT; i++) {
		reg_off = QM_QM_CPU_EPP0_CFG + (QM_QM_CPU_EPP0_CFG_STRIDE * i);
		cpu_epp0_cfg.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, map_mode=%d\n", i, reg_off, cpu_epp0_cfg.bf.map_mode);
	}

	/* CPU EPP FIFO Configuration Register */
	printk("=== QM_QM_CPU_EPP_FIFO_CFG_profile0 ===\n");
	for (i = 0; i < QM_QM_CPU_EPP_FIFO_CFG_profile0_COUNT; i++) {
		reg_off = QM_QM_CPU_EPP_FIFO_CFG_profile0 + (QM_QM_CPU_EPP_FIFO_CFG_profile0_STRIDE * i);
		cpu_epp_fifo_cfg_profile.wrd = CA_NE_REG_READ(reg_off);
		printk("profile id=%d, reg_off=0x%x, size=%d\n", i, reg_off, cpu_epp_fifo_cfg_profile.bf.size);
	}

        printk("=== QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START/QM_QM_CPU_EPP_FIFO0_0_CFG ===\n");
	for (j = 0; j < QM_QM_CPU_EPP_FIFO0_0_CFG_COUNT; j++) {
		for (i = 0; i < l3qm_cpu_voq_per_port; i++) {

                        reg_off = QM_QM_CPU_EPP_FIFO0_0_CFG + (QM_QM_CPU_EPP_FIFO0_0_CFG_STRIDE * j) + (4 * i);
                        cpu_epp_fifo_cfg.wrd = CA_NE_REG_READ(reg_off);

			reg_off = QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START + (i * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE) +
				(j * l3qm_cpu_voq_per_port * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE);
			cpu_epp64_fifo_paddr_start.wrd = CA_NE_REG_READ(reg_off);

			printk("port id=%d, voq id=%d, reg_off=0x%x, paddr_start=0x%x, profile sel=%d\n", j, i, reg_off,
					cpu_epp64_fifo_paddr_start.bf.phy_addr, cpu_epp_fifo_cfg.bf.profile_sel);
		}
	}

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_dump_cpu256_epps();
#endif

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_epps);

ca_status_t aal_l3qm_dump_empty_buffers(ca_device_id_t device_id)
{
	int i;
	volatile ca_uint32_t reg_off;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG3_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;

	printk("=== QM_QM_EQ_PROFILE0 ===\n");
	for (i = 0; i < QM_QM_EQ_PROFILE0_COUNT; i++) {
		reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * i);
		eq_profile.wrd = CA_NE_REG_READ(reg_off);
		printk("profile id=%d, reg_off=0x%x, rule=%d, eqp0=%d, eqp1=%d\n", i, reg_off, eq_profile.bf.rule, eq_profile.bf.eqp0, eq_profile.bf.eqp1);
	}

	printk(" === QM_QM_DEST_PORT0_EQ_CFG ===\n");
	for (i = 0; i < QM_QM_DEST_PORT0_EQ_CFG_COUNT; i++) {
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * i);
		dest_port_eq_cfg.wrd = CA_NE_REG_READ(reg_off);
		printk("port id=%d, reg_off=0x%x, profile_sel=%d\n", i, reg_off, dest_port_eq_cfg.bf.profile_sel);
	}

	printk("=== QM_QM_CFG0_EQ0 ===\n");
	for (i = 0; i < QM_QM_CFG0_EQ0_COUNT; i++) {
		reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * i);
		cfg0_eq.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, phy_addr_start=0x%x, eq_en=%d\n", i, reg_off, cfg0_eq.bf.phy_addr_start << CA_L3QM_PHY_ADDR_SHIFT, cfg0_eq.bf.eq_en);
	}

	printk("=== QM_QM_CFG1_EQ0 ===\n");
	for (i = 0; i < QM_QM_CFG1_EQ0_COUNT; i++) {
		reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * i);
		cfg1_eq.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, total_buffer_num=%d, bid_start=%d\n", i, reg_off, cfg1_eq.bf.total_buffer_num, cfg1_eq.bf.bid_start);
	}

	printk("=== QM_QM_CFG2_EQ0 ===\n");
	for (i = 0; i < QM_QM_CFG2_EQ0_COUNT; i++) {
		reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * i);
		cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, refill_ths=%d, refill_en=%d, refill_fbm_eqid=%d, cpu_eq=%d, buffer_size=%d\n",
				i, reg_off, cfg2_eq.bf.refill_ths, cfg2_eq.bf.refill_en, cfg2_eq.bf.refill_fbm_eqid, cfg2_eq.bf.cpu_eq, cfg2_eq.bf.buffer_size);
	}

	printk("=== QM_QM_CFG3_EQ0 ===\n");
	for (i = 0; i < QM_QM_CFG3_EQ0_COUNT; i++) {
		reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * i);
		cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, cfg3_eq.wrd=0x%x\n", i, reg_off, cfg3_eq.wrd);
		printk("qos=%d, cache=%d, snoop=%d, bar=%d, domain=%d, user=%d, acd_cmd=%d, cache_line_split=%d, cache_eos=%d\n",
			cfg3_eq.bf.qos, cfg3_eq.bf.cache, cfg3_eq.bf.snoop, cfg3_eq.bf.bar, cfg3_eq.bf.domain, cfg3_eq.bf.user,
			cfg3_eq.bf.acd_cmd, cfg3_eq.bf.cache_line_split, cfg3_eq.bf.cache_eos);
	}

	printk("=== QM_QM_CFG4_EQ0 ===\n");
	for (i = 0; i < QM_QM_CFG4_EQ0_COUNT; i++) {
		reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * i);
		cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
		printk("index=%d, reg_off=0x%x, cfg4_eq.wrd=0x%x\n", i, reg_off, cfg4_eq.wrd);
	}
	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_empty_buffers);

ca_status_t aal_l3qm_dump_empty_buffers_phy_addr(ca_device_id_t device_id)
{
	int i;
	ca_uint32_t	phy_addr;
#if defined(CONFIG_LUNA_G3_SERIES)
	QM_QM_EQ_REFILL_INTERRUPT_EN_t refill_int_en;
	u8 *rx_virt_addr;
	struct sk_buff *skb;

	/* backup/disable refill interrupt */
	refill_int_en.wrd = CA_NE_REG_READ(QM_QM_EQ_REFILL_INTERRUPT_EN);
	CA_NE_REG_WRITE(0, QM_QM_EQ_REFILL_INTERRUPT_EN);
#endif

	printk("=== QM_QM_EQ_RETURN_PHY_ADDR ===\n");
	printk("EQ=%d\n", l3qm_eq_profile_cpu_pool0_eq_id);
	aal_l3qm_get_return_phy_addr_enable(l3qm_eq_profile_cpu_pool0_eq_id, 1);
	i=0;
	while ((phy_addr = aal_l3qm_get_return_phy_addr(l3qm_eq_profile_cpu_pool0_eq_id)) != 0) {
		printk("[%d] phy_addr=0x%x\n", i, phy_addr);
		i++;

#if defined(CONFIG_LUNA_G3_SERIES)

		/* the rx_virt_addr here has been skip head_room and headers */
		rx_virt_addr = phys_to_virt(phy_addr);
		skb = (struct sk_buff *)(*((ca_uint_t *)(rx_virt_addr)));
		dev_kfree_skb_any(skb);
#endif
	}
	aal_l3qm_get_return_phy_addr_enable(l3qm_eq_profile_cpu_pool0_eq_id, 0);

        printk("EQ=%d\n", l3qm_eq_profile_cpu_pool1_eq_id);
        aal_l3qm_get_return_phy_addr_enable(l3qm_eq_profile_cpu_pool1_eq_id, 1);
        i=0;
        while ((phy_addr = aal_l3qm_get_return_phy_addr(l3qm_eq_profile_cpu_pool1_eq_id)) != 0) {
                printk("[%d] phy_addr=0x%x\n", i, phy_addr);
                i++;
#if defined(CONFIG_LUNA_G3_SERIES)
		/* the rx_virt_addr here has been skip head_room and headers */
		rx_virt_addr = phys_to_virt(phy_addr);
		skb = (struct sk_buff *)(*((ca_uint_t *)(rx_virt_addr)));
		dev_kfree_skb_any(skb);
#endif
        }
        aal_l3qm_get_return_phy_addr_enable(l3qm_eq_profile_cpu_pool1_eq_id, 0);

#if defined(CONFIG_LUNA_G3_SERIES)
	/* restore/enable refill interrupt */
	CA_NE_REG_WRITE(refill_int_en.wrd, QM_QM_EQ_REFILL_INTERRUPT_EN);
#endif

#if defined(CONFIG_NE_CPU256)
	if (support_cpu256_in_linux)
		aal_l3qm_dump_cpu256_empty_buffers_phy_addr();
#endif

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_empty_buffers_phy_addr);

ca_status_t aal_l3qm_dump_rmu_fe_drop_counter(ca_device_id_t device_id)
{
	int addr;
	QM_QM_RMU0_FE_DROP_PKT_CNTR_t	drop_cntr;
	QM_QM_RMU0_CTRL_t		ctrl;
	QM_QM_INT_SRC_t			int_src;
	QM_QM_RMU0_RX_PKT_CNTR_t	rx_pkt_cntr;
	QM_QM_RMU0_RX_PACKET_HEADER_INFO1_t	header_info1;
	QM_QM_RMU0_RX_PACKET_HEADER_INFO0_t	header_info0;
	QM_QM_RMU0_NO_BUF_DROP_PKT_INFO_t	no_buf_drop_info;
	QM_QM_RMU0_NO_BUF_DROP_PKT_CNTR_t	no_buf_drop_cntr;
	QM_QM_RMU0_FE_DROP_PKT_CNTR_t		fe_drop_cntr;
	QM_QM_RMU0_FIFO_STS_t			fifo_sts;
	QM_QM_VOQ_INFO_MEMORY_DATA0_t		voq_data0;
	QM_QM_VOQ_INFO_MEMORY_DATA1_t		voq_data1;
	QM_QM_VOQ_INFO_MEMORY_DATA2_t		voq_data2;
	QM_QM_HEADER_MEMORY_DATA0_t		header_data0;
	QM_QM_HEADER_MEMORY_DATA1_t		header_data1;
	HEADER_A_T header_a;

	printk("=== QM_QM_RMU0_FE_DROP_PKT_CNTR ===\n");
	drop_cntr.wrd = CA_NE_REG_READ(QM_QM_RMU0_FE_DROP_PKT_CNTR);
	printk("cntr=%d\n", drop_cntr.wrd);

	printk("=== QM_QM_RMU0_CTRL ===\n");
	ctrl.wrd = CA_NE_REG_READ(QM_QM_RMU0_CTRL);
	printk("voq_drop_en=%d, rx_en=%d\n", ctrl.bf.voq_drop_en, ctrl.bf.rx_en);

	printk("=== QM_QM_INT_SRC ===\n");
	int_src.wrd = CA_NE_REG_READ(QM_QM_INT_SRC);
	printk("rmu0_no_buffer_drop_int_src=%d, rmu0_check_error_int_src=%d, rmu0_fifo_error_int_src=%d, pkt_lenght_error_int_src=%d, dqm_fe_drop_int_src=%d\n",
			int_src.bf.rmu0_no_buffer_drop_int_src, int_src.bf.rmu0_check_error_int_src, int_src.bf.rmu0_fifo_error_int_src,
			int_src.bf.pkt_lenght_error_int_src, int_src.bf.dqm_fe_drop_int_src);

	printk("=== QM_QM_RMU0_RX_PKT_CNTR ===\n");
	rx_pkt_cntr.wrd = CA_NE_REG_READ(QM_QM_RMU0_RX_PKT_CNTR);
	printk("cntr=%d\n", rx_pkt_cntr.bf.cntr);

	printk("=== QM_QM_RMU0_RX_PACKET_HEADER_INFO ===\n");
	header_info1.wrd = CA_NE_REG_READ(QM_QM_RMU0_RX_PACKET_HEADER_INFO1);
	header_info0.wrd = CA_NE_REG_READ(QM_QM_RMU0_RX_PACKET_HEADER_INFO0);
	printk("header_info1.wrd=0x%x, header_info0.wrd=0x%x\n", header_info1.wrd, header_info0.wrd);

	header_a.bits64 = ((ca_uint64_t)header_info1.wrd << 32) | header_info0.wrd;
	printk("ldpid=%d, lspid=%d, pkt_size=%d, fe_bypass=%d, hdr_type=%d\n",
		header_a.bits.ldpid, header_a.bits.lspid, header_a.bits.pkt_size, header_a.bits.fe_bypass, header_a.bits.hdr_type);

        printk("=== QM_QM_RMU0_NO_BUF_DROP_PKT_INFO ===\n");
	no_buf_drop_info.wrd = CA_NE_REG_READ(QM_QM_RMU0_NO_BUF_DROP_PKT_INFO);
	printk("eqid=%d, voqid=%d\n", no_buf_drop_info.bf.eqid, no_buf_drop_info.bf.voqid);

        printk("=== QM_QM_RMU0_NO_BUF_DROP_PKT_CNTR ===\n");
	no_buf_drop_cntr.wrd = CA_NE_REG_READ(QM_QM_RMU0_NO_BUF_DROP_PKT_CNTR);
	printk("cntr=%d\n", no_buf_drop_cntr.bf.cntr);

        printk("=== QM_QM_RMU0_FE_DROP_PKT_CNTR ===\n");
	fe_drop_cntr.wrd = CA_NE_REG_READ(QM_QM_RMU0_FE_DROP_PKT_CNTR);
	printk("cntr=%d\n", fe_drop_cntr.bf.cntr);

        printk("=== QM_QM_RMU0_FIFO_STS ===\n");
	fifo_sts.wrd = CA_NE_REG_READ(QM_QM_RMU0_FIFO_STS);
	printk("data_fifo_depth=%d, header_fifo_depth=%d, cmd_fifo_depth=%d, cmd_resp_fifo_depth=%d, axi_fifo_depth=%d\n",
		fifo_sts.bf.data_fifo_depth, fifo_sts.bf.header_fifo_depth, fifo_sts.bf.cmd_fifo_depth, fifo_sts.bf.cmd_resp_fifo_depth, fifo_sts.bf.axi_fifo_depth);

        printk("=== QM_QM_VOQ_INFO_MEMORY_DATA0 ===\n");
	for (addr = 0; addr < 64; addr++) {
		aal_l3qm_get_voq_info_mem(0, addr, &voq_data0, &voq_data1, &voq_data2);
	}

        printk("=== QM_QM_HEADER_MEMORY_DATA0 ===\n");
	for (addr = 0; addr < 64; addr++) {
		aal_l3qm_get_header_mem(0, addr, &header_data0, &header_data1);
	}

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_rmu_fe_drop_counter);

ca_uint32_t aal_l3qm_dump_cpu64_fifo_read_ptr(ca_device_id_t device_id)
{
	int i;
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_t      rdptr;

	printk("=== QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR ===\n");
	for (i = 0; i < QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_COUNT; i++) {
		rdptr.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR + i * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_RDPTR_STRIDE);
		printk("[%d] rdptr=%d\n", i, rdptr.bf.rptr);
	}
	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_cpu64_fifo_read_ptr);

ca_uint32_t aal_l3qm_dump_cpu64_fifo_write_ptr(ca_device_id_t device_id)
{
	int i;
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_t      wrptr;

	printk("=== QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR ===\n");
	for (i = 0; i < QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_COUNT; i++) {
		wrptr.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR + i * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_WRPTR_STRIDE);
		printk("[%d] wrptr=%d\n", i, wrptr.bf.wptr);
	}
	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_cpu64_fifo_write_ptr);

ca_uint32_t aal_l3qm_dump_cpu64_fifo_paddr_start(ca_device_id_t device_id)
{
	int i;
	QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_t      paddr_start;

	printk("=== QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START ===\n");
	for (i = 0; i < QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_COUNT; i++) {
		paddr_start.wrd = CA_NE_REG_READ(QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START + i * QM_QM_CPUEPP_POINTER_CPUEPP64_FIFO_PADDR_START_STRIDE);
		printk("[%d] phy_addr=0x%x\n", i, paddr_start.bf.phy_addr);
	}
	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_cpu64_fifo_paddr_start);

ca_uint32_t aal_l3qm_dump_monitor_eq(ca_device_id_t device_id)
{
	int i;
	QM_QM_MONITOR0_EQ0_t  monitor0_eq;
	QM_QM_MONITOR1_EQ0_t  monitor1_eq;
	QM_QM_MONITOR2_EQ0_t  monitor2_eq;
	
#if defined(CONFIG_LUNA_G3_SERIES)
	QM_QM_MONITOR3_EQ0_t  monitor3_eq;

	printk("=== QM_QM_MONITOR0_EQ0/1/2/3 ===\n");
	for (i = 0; i < QM_QM_MONITOR0_EQ0_COUNT; i++) {
		
		QM_QM_CFG0_EQ0_t cfg0_eq;
		cfg0_eq.wrd = CA_NE_REG_READ(QM_QM_CFG0_EQ0 + i * QM_QM_CFG0_EQ0_STRIDE);
			
		if(cfg0_eq.bf.eq_en == 0) {
			printk("eq_id[%02d] -- disabled\n", i);
			continue;
		}
		
		printk("eq_id[%02d]\n", i);
		monitor0_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR0_EQ0 + i * QM_QM_MONITOR0_EQ0_STRIDE);
		monitor1_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR1_EQ0 + i * QM_QM_MONITOR1_EQ0_STRIDE);
		monitor2_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR2_EQ0 + i * QM_QM_MONITOR2_EQ0_STRIDE);
		monitor3_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR3_EQ0 + i * QM_QM_MONITOR3_EQ0_STRIDE);
		
		printk(" - active cntr=%d, inactive_cntr=%d, inactive_head=%d, inactive_tail=%d\n", 
			monitor1_eq.bf.cntr, monitor2_eq.bf.inactive_cntr, monitor1_eq.bf.inactive_head, monitor2_eq.bf.inactive_tail);
		printk(" - active_stack_cntr=%d, inactive_stack_cntr=%d\n", 
			monitor3_eq.bf.stack_cntr, monitor3_eq.bf.inactive_stack_cntr);
		printk(" - head=%d, tail=%d\n", 
			monitor0_eq.bf.head, monitor0_eq.bf.tail);
	}
		
#else
	printk("=== QM_QM_MONITOR0_EQ0 ===\n");
	for (i = 0; i < QM_QM_MONITOR0_EQ0_COUNT; i++) {
		monitor0_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR0_EQ0 + i * QM_QM_MONITOR0_EQ0_STRIDE);
		printk("[%d] head=0x%x, tail=0x%x\n", i, monitor0_eq.bf.head, monitor0_eq.bf.tail);
	}

        printk("=== QM_QM_MONITOR1_EQ1 ===\n");
        for (i = 0; i < QM_QM_MONITOR1_EQ0_COUNT; i++) {
                monitor1_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR1_EQ0 + i * QM_QM_MONITOR1_EQ0_STRIDE);
                printk("[%d] inactive_head=0x%x, cntr=0x%x\n", i, monitor1_eq.bf.inactive_head, monitor1_eq.bf.cntr);
        }

        printk("=== QM_QM_MONITOR2_EQ1 ===\n");
        for (i = 0; i < QM_QM_MONITOR2_EQ0_COUNT; i++) {
                monitor2_eq.wrd = CA_NE_REG_READ(QM_QM_MONITOR2_EQ0 + i * QM_QM_MONITOR2_EQ0_STRIDE);
                printk("[%d] inactive_cntr=0x%x, inactive_tail=0x%x\n", i, monitor2_eq.bf.inactive_cntr, monitor2_eq.bf.inactive_tail);
        }
#endif

	return CA_E_OK;
}
EXPORT_SYMBOL(aal_l3qm_dump_monitor_eq);

static void aal_l3qm_dump_axi_attrib(int entry)
{
	int i;
        QM_QM_AXI_ATTRIBUTE_ACCESS_t    attrib;
        QM_QM_AXI_ATTRIBUTE_DATA0_t   attrib_data0;
        QM_QM_AXI_ATTRIBUTE_DATA1_t   attrib_data1;

	attrib.wrd = 0;
	attrib.bf.access = 1;
	attrib.bf.rbw = 0;
	attrib.bf.ADDR = entry;
	CA_NE_REG_WRITE(attrib.wrd, QM_QM_AXI_ATTRIBUTE_ACCESS);
	for (i = 0; i < CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT; i++) {
		attrib.wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_ACCESS);
		if (!attrib.bf.access)
			break;
	}
	if (i == CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT) {
		printk("%s: CA_L3QM_AXI_ATTRIBUTE_ACCESS_TIMEOUT at entry=%d!!!\n", __func__, entry);
		return;
	}

        attrib_data0.wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_DATA0);
        attrib_data1.wrd = CA_NE_REG_READ(QM_QM_AXI_ATTRIBUTE_DATA1);

	printk("[%d] data0=0x%x, data1=0x%x\n", entry, attrib_data0.wrd, attrib_data1.wrd);
	printk("     qos=%d, cache=%d, snoop=%d, bar=%d, domain=%d, user=%d, prot=%d\n", attrib_data0.bf.qos, attrib_data0.bf.cache, attrib_data0.bf.snoop,
		attrib_data0.bf.bar, attrib_data0.bf.domain, attrib_data0.bf.user, attrib_data0.bf.prot);

	printk("     axi_top_bit=0x%x, ace_cmd=%d\n", attrib_data0.bf.axi_top_bit | (attrib_data1.bf.axi_top_bit << 6), attrib_data0.bf.ace_cmd);
}

ca_status_t aal_l3qm_dump_axi_attribs(ca_device_id_t device_id)
{
	int i;

	printk("=== EQ 0-15 AXI ATTRIBUTE ===\n");

	for (i = CA_L3QM_AXI_ATTRIBUTE_EQ_BASE; i < CA_L3QM_AXI_ATTRIBUTE_EQ_BASE+16; i++) {
		aal_l3qm_dump_axi_attrib(i);
	}

        printk("=== CPUEPP-256 port 0-31 AXI ATTRIBUTE ===\n");

        for (i = CA_L3QM_AXI_ATTRIBUTE_CPU256_BASE; i < CA_L3QM_AXI_ATTRIBUTE_CPU256_BASE+32; i++) {
                aal_l3qm_dump_axi_attrib(i);
        }

        printk("=== CPUEPP port 0-7 AXI ATTRIBUTE ===\n");

        for (i = CA_L3QM_AXI_ATTRIBUTE_CPU_BASE; i < CA_L3QM_AXI_ATTRIBUTE_CPU_BASE+8; i++) {
                aal_l3qm_dump_axi_attrib(i);
        }

	return CA_E_OK;
}

EXPORT_SYMBOL(aal_l3qm_dump_axi_attrib);

static ca_uint32_t aal_l3qm_calc_jumbo_buf_count(ca_uint32_t buf_count, ca_uint32_t buf_size)
{
        ca_uint32_t jumbo_buf_size;
        ca_uint32_t jumbo_buf_count;

        jumbo_buf_count = buf_count;

        /* when Jumbo buffer mode is enabled the total buffer number should be re-cacluated */
        if (l3qm_jumbo_buf_packing_size > 0) {

                //buf_count = l3qm_eq_profile_dq_pool1_buf_count;
                //buf_size = l3qm_eq_profile_dq_pool1_buf_sz;

                jumbo_buf_size = buf_size;
                if (l3qm_jumbo_buf_packing_size == 1) {
                        jumbo_buf_size = 2048;
                }
                else if (l3qm_jumbo_buf_packing_size == 2) {
                        jumbo_buf_size = 4096;
                }
                else if (l3qm_jumbo_buf_packing_size == 3) {
                        jumbo_buf_size = 16384;
                }
                jumbo_buf_count = (buf_count * buf_size) / jumbo_buf_size;
        }
        printk("%s: buf_count=%d buf_size=%d jumbo_buf_count=%d\n", __func__, buf_count, buf_size, jumbo_buf_count);
        return jumbo_buf_count;
}


#if defined(CONFIG_LUNA_G3_SERIES)

static void aal_l3qm_init_DQ_shared_dram(int profile_index, ca_uint32_t sram_phy_start, ca_uint32_t ddr_phy_start, ca_uint32_t mode)
{
	int i, pool_id, buf_cnt, buf_size;
	ca_uint32_t reg_off;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG2_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;
	ca_uint32_t pool_ddr_phy_start;
	aal_l3_te_cb_free_buf_cnt_t port_buf;
	aal_l3_te_cb_free_buf_cnt_mask_t port_buf_mask;
	ca_uint32_t profileIdx = 0;
	aal_l3_te_cb_comb_threshold_mask_t threshold_msk;
	aal_l3_te_cb_comb_threshold_t threshold;

	if(profile_index == 5) {
		aal_l3_te_cb_free_buf_cnt_t port_buf;
		aal_l3_te_cb_free_buf_cnt_mask_t port_buf_mask;

		buf_cnt = l3qm_eq_profile_dq_pool1_buf_count/3;
		buf_size = l3qm_eq_profile_dq_pool1_buf_sz;
		pool_id = profile_index * 2;	
		
		pool_ddr_phy_start = ddr_phy_start + (buf_cnt * buf_size * 1);

		port_buf_mask.u32 = 0;
		port_buf_mask.s.cnt0 = 1;
		port_buf.cnt0 = buf_cnt;
		for(i = (CA_NI_TOTAL_CPU_PORT+4); i < (CA_NI_TOTAL_CPU_PORT+8) ; i++) {
            		aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
		}
#if defined(CONFIG_RG_G3_WAN_PORT_INDEX) && (CONFIG_RG_G3_WAN_PORT_INDEX!=7)
            	aal_l3_te_cb_port_free_buf_cnt_set(0, CA_NI_TOTAL_CPU_PORT+CONFIG_RG_G3_WAN_PORT_INDEX, port_buf_mask, &port_buf);
#endif
		
	} else {
		buf_cnt = l3qm_eq_profile_dq_pool1_buf_count/3;
		buf_size = l3qm_eq_profile_dq_pool1_buf_sz;
		pool_id = profile_index * 2;

		pool_ddr_phy_start = ddr_phy_start + (buf_cnt * buf_size * 0);	
		
		port_buf_mask.u32 = 0;
		port_buf_mask.s.cnt0 = 1;
		port_buf.cnt0 = buf_cnt;
		for(i = (CA_NI_TOTAL_CPU_PORT); i < (CA_NI_TOTAL_CPU_PORT+4) ; i++) {
            		aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
		}
	}


	printk("%s: sram_phy_start=0x%x, ddr_phy_start=0x%x\n", __func__, sram_phy_start, ddr_phy_start);
	printk("%s: pool_id=%d, bid_start=%d\n", __func__, pool_id, DQ_bid_start);

	/* configure EB pool 0 for DQ */
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = pool_ddr_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
	cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 512 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);
	
	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);

	/* configure EB pool 1 for DQ */

	
	pool_ddr_phy_start = ddr_phy_start + (buf_cnt * buf_size * 2);

	pool_id += 1;
	DQ_bid_start += buf_cnt;
	if(pool_id == 11) {
		buf_cnt = 64;
		buf_size = 512;
	} else {
		buf_cnt = l3qm_eq_profile_dq_pool1_buf_count/3;
		buf_size = l3qm_eq_profile_dq_pool1_buf_sz;
	}

	if((mode == L3QM_EQ_SHARE_MODE && pool_id == l3qm_eq_share_pool_id) || mode != L3QM_EQ_SHARE_MODE) {
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = pool_ddr_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
		cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 1024 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);

	DQ_bid_start += buf_cnt;	

	port_buf_mask.u32 = 0;
	port_buf_mask.s.cnt1 = 1;
	port_buf.cnt1 = buf_cnt;
	for(i = (CA_NI_TOTAL_CPU_PORT); i < (CA_NI_TOTAL_CPU_PORT+8) ; i++) {
        	aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
	}

	// adjust voq threshold
	threshold_msk.s.threshold_hi = 1;
	threshold_msk.s.threshold_lo = 1;
	threshold.threshold_hi = (buf_cnt*2) / 2;
	threshold.threshold_lo = (buf_cnt*2) / 3;	

	// lan
	aal_l3_te_cb_voq_profile_sel_get(0, 64, &profileIdx);
	aal_l3_te_cb_voq_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);
	
	// wan
	aal_l3_te_cb_voq_profile_sel_get(0, 120, &profileIdx);
	aal_l3_te_cb_voq_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);


	// adjust port threshold
	threshold_msk.s.threshold_hi = 1;
	threshold_msk.s.threshold_lo = 1;
	threshold.threshold_hi = ((buf_cnt*2) / 3) * 2;
	threshold.threshold_lo = ((buf_cnt*2) / 3);	

	// lan
	aal_l3_te_cb_port_profile_sel_get(0, CA_NI_TOTAL_CPU_PORT + 0, &profileIdx);
	aal_l3_te_cb_port_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);

	// wan
	aal_l3_te_cb_port_profile_sel_get(0, CA_NI_TOTAL_CPU_PORT + 7, &profileIdx);
	aal_l3_te_cb_port_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);
	
	}
}

static void aal_l3qm_init_DQ_pools_pool0(int profile_index, ca_uint32_t sram_phy_start, uint32_t sram_size)
{
	int i, pool_id, buf_cnt, buf_size;
	ca_uint32_t reg_off;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG2_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;
	aal_l3_te_cb_free_buf_cnt_t port_buf;
	aal_l3_te_cb_free_buf_cnt_mask_t port_buf_mask;

	if(profile_index == 5) {			
		// remaining  sram
		buf_size = 512;
		buf_cnt =  sram_size / buf_size;
		pool_id = profile_index * 2;	

		port_buf_mask.u32 = 0;
		port_buf_mask.s.cnt0 = 1;
		port_buf.cnt0 = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
		for(i = (CA_NI_TOTAL_CPU_PORT+4); i < __AAL_L3_TE_CB_PORT_NUM ; i++) {
            		aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
		}
#if defined(CONFIG_RG_G3_WAN_PORT_INDEX) && (CONFIG_RG_G3_WAN_PORT_INDEX!=7)
            	aal_l3_te_cb_port_free_buf_cnt_set(0, CA_NI_TOTAL_CPU_PORT+CONFIG_RG_G3_WAN_PORT_INDEX, port_buf_mask, &port_buf);
#endif
		
	} else {
		buf_size = 512;
		buf_cnt = sram_size / buf_size;
		pool_id = profile_index * 2;

		port_buf_mask.u32 = 0;
		port_buf_mask.s.cnt0 = 1;
		port_buf.cnt0 = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
		
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)

		for(i = (CA_NI_TOTAL_CPU_PORT); i < (CA_NI_TOTAL_CPU_PORT+4) ; i++) {
            		aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
		}
#else
		
            	aal_l3_te_cb_port_free_buf_cnt_set(0, CA_NI_TOTAL_CPU_PORT+profile_index, port_buf_mask, &port_buf);
#endif
	}


	printk("%s: sram_phy_start=0x%x\n", __func__, sram_phy_start);
	printk("%s: pool_id=%d, bid_start=%d\n", __func__, pool_id, DQ_bid_start);

	/* configure EB pool 0 for DQ */
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = sram_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
	cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 512 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);
	
	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);


	DQ_bid_start += buf_cnt;
	
}

static void aal_l3qm_init_DQ_pools_pool1(ca_uint32_t ddr_phy_start)
{
	int i, pool_id, buf_cnt, buf_size;
	ca_uint32_t reg_off;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG2_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;
	aal_l3_te_cb_free_buf_cnt_t port_buf;
	aal_l3_te_cb_free_buf_cnt_mask_t port_buf_mask;
	ca_uint32_t profileIdx = 0;
	aal_l3_te_cb_comb_threshold_mask_t threshold_msk;
	aal_l3_te_cb_comb_threshold_t threshold;

	/* pool1 is shared by all NI ports */

	pool_id = l3qm_eq_share_pool_id;
	buf_cnt = l3qm_eq_profile_dq_pool1_buf_count;
	buf_size = l3qm_eq_profile_dq_pool1_buf_sz;

	
	printk("%s: dram_phy_start=0x%x\n", __func__, ddr_phy_start);
	printk("%s: pool_id=%d, bid_start=%d\n", __func__, pool_id, DQ_bid_start);
	
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = ddr_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
		cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 1024 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);

	DQ_bid_start += buf_cnt;	

	port_buf_mask.u32 = 0;
	port_buf_mask.s.cnt1 = 1;
	port_buf.cnt1 = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	for(i = (CA_NI_TOTAL_CPU_PORT); i < __AAL_L3_TE_CB_PORT_NUM ; i++) {
        	aal_l3_te_cb_port_free_buf_cnt_set(0, i, port_buf_mask, &port_buf);
	}


	// adjust voq threshold
	threshold_msk.s.threshold_hi = 1;
	threshold_msk.s.threshold_lo = 1;
	threshold.threshold_hi = port_buf.cnt1 / 2;
	threshold.threshold_lo = port_buf.cnt1 / 3;	

	// lan
	aal_l3_te_cb_voq_profile_sel_get(0, 64, &profileIdx);
	aal_l3_te_cb_voq_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);
	
	// wan
	aal_l3_te_cb_voq_profile_sel_get(0, 120, &profileIdx);
	aal_l3_te_cb_voq_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);


	// adjust port threshold
	threshold_msk.s.threshold_hi = 1;
	threshold_msk.s.threshold_lo = 1;
	threshold.threshold_hi = (port_buf.cnt1 / 3) * 2;
	threshold.threshold_lo = (port_buf.cnt1 / 3);	

	// lan
	aal_l3_te_cb_port_profile_sel_get(0, CA_NI_TOTAL_CPU_PORT + 0, &profileIdx);
	aal_l3_te_cb_port_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);

	// wan
	aal_l3_te_cb_port_profile_sel_get(0, CA_NI_TOTAL_CPU_PORT + 7, &profileIdx);
	aal_l3_te_cb_port_threshold_profile_set(0, profileIdx, threshold_msk, &threshold);
	
}


static void aal_l3qm_init_empty_buffer_DQ_1(void)
{
	int i;
	ca_uint32_t reg_off;
	uint32_t eq_pool_sram_size = 0;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	// shared sram for DQ usage starts after hash function.
	ca_uint32_t l3fe_dq_eq0_phy_addr = (l3fe_main_hash_action_cache_fib_phy_addr + 
										(l3fe_main_hash_action_cache_fib_length * l3fe_main_hash_action_cache_fib_count) );

	/* l3qm_eq_share_pool_id should be read from scfg */
	l3qm_eq_share_pool_id = 1;


#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)

		/*
		 * NI/LAN ports select EQ profile 0
		 * 	eq profile 0 select pool 0 and 1
		 * NI/WAN port select EQ profile 5
		 *	eq profile 5 select pool 10 and 1
		 * CPU ports select EQ profile 4
		 *	eq profile 4 select pool 8 and 9
		 */
		/* profile 0 for DQ NI lan ports */
		eq_profile.wrd = 0;
		eq_profile.bf.eqp0 = 0;
		eq_profile.bf.eqp1 = l3qm_eq_share_pool_id;
		eq_profile.bf.rule = 0;
		reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * 0);
		CA_NE_REG_WRITE(eq_profile.wrd, reg_off);
#else
		/* 4 profile for DQ or normal NI ports */
		for (i = 0; i < 4; i++) {
			eq_profile.wrd = 0;
			eq_profile.bf.eqp0 = i*2;
			//eq_profile.bf.eqp1 = i*2+1;
			eq_profile.bf.eqp1 = l3qm_eq_share_pool_id;
			eq_profile.bf.rule = 0;
			reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * i);
			CA_NE_REG_WRITE(eq_profile.wrd, reg_off);
		}
#endif
	

	/* profile 5 for NI port 7 */
	i = 5;
	eq_profile.wrd = 0;
	eq_profile.bf.eqp0 = i*2;
	eq_profile.bf.eqp1 = l3qm_eq_share_pool_id;
	eq_profile.bf.rule = 0;
	reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * i);
	CA_NE_REG_WRITE(eq_profile.wrd, reg_off);

	/* [LAN] select profile 0 [1,2,3] for DQ at NI ports  */
	dest_port_eq_cfg.wrd = 0;
	for (i = 0; i < 4; i++) {
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)
		dest_port_eq_cfg.bf.profile_sel = 0;
#else
		if(l3qm_eq0_eq1_all_dram == 1)
			dest_port_eq_cfg.bf.profile_sel = 0;
		else
			dest_port_eq_cfg.bf.profile_sel = i;
#endif
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (i + CA_NI_TOTAL_CPU_PORT));
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
	}

	/* [WAN] select profile 5 for NI port 7 */
	i = 5;
	dest_port_eq_cfg.bf.profile_sel = i;
	
	for (i = 4; i < 8; i++) {
		dest_port_eq_cfg.bf.profile_sel = 5;
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (i + CA_NI_TOTAL_CPU_PORT));
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
	}
#if defined(CONFIG_RG_G3_WAN_PORT_INDEX) && (CONFIG_RG_G3_WAN_PORT_INDEX!=7)
	// aditional config if enabling lan port as wan
	dest_port_eq_cfg.bf.profile_sel = 5;
	reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (CONFIG_RG_G3_WAN_PORT_INDEX + CA_NI_TOTAL_CPU_PORT));
	CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
#endif

	if((sram_dts_conf_size < sram_cur_used_size)) {
		eq_pool_sram_size = EQ_POOL_SRAM_SIZE;
		printk("ERROR: dtb sram size is %d but currect used size %d over that\n", sram_dts_conf_size, sram_cur_used_size);
	}else {
		eq_pool_sram_size = ( sram_dts_conf_size - sram_cur_used_size ) / 5;
		if(eq_pool_sram_size > EQ_POOL_SRAM_SIZE)
			eq_pool_sram_size = EQ_POOL_SRAM_SIZE;
		printk("capsram size is %d  currect used size is %d.\n", sram_dts_conf_size, sram_cur_used_size);
	}

	if(l3qm_eq0_eq1_all_dram == 1) {
		
		/* test mode */
		
		/* Init DQ private SRAM pool - LAN ports */
		aal_l3qm_init_DQ_shared_dram(0, l3fe_dq_eq0_phy_addr, l3qm_eq_profile_dq_pool0_buff_phy_addr, L3QM_EQ_SHARE_MODE);

		/* Init DQ private SRAM pool - WAN ports */
		aal_l3qm_init_DQ_shared_dram(5, l3fe_dq_eq0_phy_addr, l3qm_eq_profile_dq_pool0_buff_phy_addr, L3QM_EQ_SHARE_MODE);
	}else {
	
#if defined(CONFIG_FC_SPECIAL_FAST_FORWARD)

		/* Init DQ private SRAM pool - LAN ports */
		aal_l3qm_init_DQ_pools_pool0(0, l3fe_dq_eq0_phy_addr, eq_pool_sram_size*4);
		sram_cur_used_size += eq_pool_sram_size*4;

		/* Init DQ private SRAM pool - WAN ports */
		aal_l3qm_init_DQ_pools_pool0(5, l3fe_dq_eq0_phy_addr + eq_pool_sram_size*4, ( sram_dts_conf_size - sram_cur_used_size ));
		sram_cur_used_size += ( sram_dts_conf_size - sram_cur_used_size );
		
		/* Init DQ shared DRAM pool - ALL ports */
		aal_l3qm_init_DQ_pools_pool1(l3qm_eq_profile_dq_pool0_buff_phy_addr);
#else

		/* Init DQ private SRAM pool - LAN ports */
		for (i = 0; i < 4; i++) {
			aal_l3qm_init_DQ_pools_pool0(i, l3fe_dq_eq0_phy_addr + (i * eq_pool_sram_size), eq_pool_sram_size);
			sram_cur_used_size += eq_pool_sram_size;
		}

		/* Init DQ private SRAM pool - WAN ports */
		aal_l3qm_init_DQ_pools_pool0(i + 1, l3fe_dq_eq0_phy_addr + (i * eq_pool_sram_size), ( sram_dts_conf_size - sram_cur_used_size ));
		sram_cur_used_size += ( sram_dts_conf_size - sram_cur_used_size );

		/* Init DQ shared DRAM pool - ALL ports */
		aal_l3qm_init_DQ_pools_pool1(l3qm_eq_profile_dq_pool0_buff_phy_addr);
#endif

	}

}

EXPORT_SYMBOL(rtkScfg);

#else

//re-configure DeepQ
static void aal_l3qm_init_DQ_pools(int profile_index, ca_uint32_t sram_phy_start, ca_uint32_t ddr_phy_start, ca_uint32_t mode);
static void aal_l3qm_init_empty_buffer_DQ_1(void)
{
	int i;
	ca_uint32_t reg_off;
	QM_QM_EQ_PROFILE0_t eq_profile;
	QM_QM_DEST_PORT0_EQ_CFG_t dest_port_eq_cfg;
	// shared sram for DQ usage starts after hash function.
	ca_uint32_t l3fe_dq_eq0_phy_addr = (l3fe_main_hash_action_cache_fib_phy_addr + 
										(l3fe_main_hash_action_cache_fib_length * l3fe_main_hash_action_cache_fib_count) );

	/* l3qm_eq_share_pool_id should be read from scfg */
	l3qm_eq_share_pool_id = 1;

	/* 4 profile for DQ or normal NI ports */
	for (i = 0; i < 4; i++) {
		eq_profile.wrd = 0;
		eq_profile.bf.eqp0 = i*2;
		//eq_profile.bf.eqp1 = i*2+1;
		eq_profile.bf.eqp1 = l3qm_eq_share_pool_id;
		eq_profile.bf.rule = 0;
		reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * i);
		CA_NE_REG_WRITE(eq_profile.wrd, reg_off);
	}

	/* profile 5 for NI port 7 */
	i = 5;
	eq_profile.wrd = 0;
	eq_profile.bf.eqp0 = i*2;
	eq_profile.bf.eqp1 = l3qm_eq_share_pool_id;
	eq_profile.bf.rule = 0;
	reg_off = QM_QM_EQ_PROFILE0 + (QM_QM_EQ_PROFILE0_STRIDE * i);
	CA_NE_REG_WRITE(eq_profile.wrd, reg_off);

	/* for NI ports profile select, profile 0 for DQ at NI ports  */
	dest_port_eq_cfg.wrd = 0;
	for (i = 0; i < 4; i++) {
		dest_port_eq_cfg.bf.profile_sel = i;
		reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (i + CA_NI_TOTAL_CPU_PORT));
		CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);
	}

	/* select profile 5 for NI port 7 */
	i = 5;
	dest_port_eq_cfg.bf.profile_sel = i;
	reg_off = QM_QM_DEST_PORT0_EQ_CFG + (QM_QM_DEST_PORT0_EQ_CFG_STRIDE * (7 + CA_NI_TOTAL_CPU_PORT));
	CA_NE_REG_WRITE(dest_port_eq_cfg.wrd, reg_off);

	for (i = 0; i < 4; i++) {
		aal_l3qm_init_DQ_pools(i, l3fe_dq_eq0_phy_addr + (i * EQ_POOL_SRAM_SIZE), l3qm_eq_profile_dq_pool0_buff_phy_addr + (i * EQ_POOL_DDR_SIZE), L3QM_EQ_SHARE_MODE);
	}

	/* for NI port 7, EQ pool 10, 1 */
	aal_l3qm_init_DQ_pools(i + 1, l3fe_dq_eq0_phy_addr + (i * EQ_POOL_SRAM_SIZE), l3qm_eq_profile_dq_pool0_buff_phy_addr + (i * EQ_POOL_DDR_SIZE), L3QM_EQ_SHARE_MODE);

}

static void aal_l3qm_init_DQ_pools(int profile_index, ca_uint32_t sram_phy_start, ca_uint32_t ddr_phy_start, ca_uint32_t mode)
{
	int i, pool_id, buf_cnt, buf_size;
	ca_uint32_t reg_off;
	QM_QM_CFG0_EQ0_t cfg0_eq;
	QM_QM_CFG1_EQ0_t cfg1_eq;
	QM_QM_CFG2_EQ0_t cfg2_eq;
	QM_QM_CFG2_EQ0_t cfg3_eq;
	QM_QM_CFG4_EQ0_t cfg4_eq;
	static int DQ_bid_start = 3000;	// cpu eq0 1500 + eq1 1500

	// 64 buffer * 512B
	buf_cnt = 64;
	buf_size = 512;
	pool_id = profile_index * 2;

	printk("%s: sram_phy_start=0x%x, ddr_phy_start=0x%x\n", __func__, sram_phy_start, ddr_phy_start);
	printk("%s: pool_id=%d, bid_start=%d\n", __func__, pool_id, DQ_bid_start);

	/* configure EB pool 0 for DQ */
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = sram_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
	cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 512 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);
	
	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);

	/* configure EB pool 1 for DQ */

	pool_id += 1;
	DQ_bid_start += buf_cnt;
	if(pool_id == 11) {
		buf_cnt = 64;
		buf_size = 512;
	} else {
		buf_cnt = l3qm_eq_profile_dq_pool1_buf_count;
		buf_size = l3qm_eq_profile_dq_pool1_buf_sz;
	}

	if((mode == L3QM_EQ_SHARE_MODE && pool_id == l3qm_eq_share_pool_id) || mode != L3QM_EQ_SHARE_MODE) {
	/* set phy_addr_start and enable Empty Buffer */
	cfg0_eq.wrd = 0;
	cfg0_eq.bf.eq_en = 1;
	cfg0_eq.bf.phy_addr_start = ddr_phy_start >> CA_L3QM_PHY_ADDR_SHIFT;
	reg_off = QM_QM_CFG0_EQ0 + (QM_QM_CFG0_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg0_eq.wrd, reg_off);

	printk("%s: cfg0_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg0_eq.wrd, reg_off);

	/* configure total buffer number and bid_start */
	cfg1_eq.wrd = 0;
	cfg1_eq.bf.total_buffer_num = aal_l3qm_calc_jumbo_buf_count(buf_cnt, buf_size);
	cfg1_eq.bf.bid_start = DQ_bid_start;
	reg_off = QM_QM_CFG1_EQ0 + (QM_QM_CFG1_EQ0_STRIDE * pool_id);
	CA_NE_REG_WRITE(cfg1_eq.wrd, reg_off);

	printk("%s: cfg1_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg1_eq.wrd, reg_off);

	/* configure buffer size for Empty Buffer */
	reg_off = QM_QM_CFG2_EQ0 + (QM_QM_CFG2_EQ0_STRIDE * pool_id);
	cfg2_eq.wrd = CA_NE_REG_READ(reg_off);
	cfg2_eq.bf.cpu_eq = 0;
		cfg2_eq.bf.buffer_size = aal_l3qm_get_buffer_size_index(buf_size);      /* 1024 bytes */
	cfg2_eq.bf.refill_en = 0;
	CA_NE_REG_WRITE(cfg2_eq.wrd, reg_off);

	printk("%s: cfg2_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg2_eq.wrd, reg_off);

	/* configure cache_eos/domain/snoop/cache */
	reg_off = QM_QM_CFG3_EQ0 + (QM_QM_CFG3_EQ0_STRIDE * pool_id);
	cfg3_eq.wrd = 0x10;
	CA_NE_REG_WRITE(cfg3_eq.wrd, reg_off);

	cfg3_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg3_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg3_eq.wrd, reg_off);

	/* configure AXI address top 8 bits */
	reg_off = QM_QM_CFG4_EQ0 + (QM_QM_CFG4_EQ0_STRIDE * pool_id);
	cfg4_eq.wrd = 0;
	CA_NE_REG_WRITE(cfg4_eq.wrd, reg_off);

	cfg4_eq.wrd = CA_NE_REG_READ(reg_off);
	printk("%s: cfg4_eq.wrd=0x%x, reg_off=0x%x\n", __func__, cfg4_eq.wrd, reg_off);

	DQ_bid_start += buf_cnt;	
	}
}

#endif

