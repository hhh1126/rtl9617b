/*SH1
*******************************************************************************
**                                                                           **
**         Copyright (c) 2014 - 2018 Quantenna Communications Inc            **
**                            All Rights Reserved                            **
**                                                                           **
**  Author      : Quantenna Communications Inc                               **
**  File        : shared_defs.h                                              **
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
**  In the case this software is distributed under the GPL license,          **
**  you should have received a copy of the GNU General Public License        **
**  along with this software; if not, write to the Free Software             **
**  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA  **
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
EH1*/

#ifndef _QTN_BASE_SHARED_DEFS_H_
#define _QTN_BASE_SHARED_DEFS_H_

#define QTN_PEARL_FPGA_BOARD				1232
#define QTN_PEARL_FPGA_RC_BOARD				1233
#define QTN_PEARL_FPGA_EP_BOARD				1234
#define QTN_PEARL_BOARD					1235
#define QTN_PEARL_RC_BOARD				1236
#define QTN_PEARL_EP_BOARD				1237
#define QTN_PEARL_RC_GEN3_BOARD				1238
#define QTN_PEARL_EP_GEN3_BOARD				1239
#define QTN_PEARL_QSR5G_VB_BOARD			1240
#define QTN_PEARL_QSR5G_EP_BOARD			1241
#define QTN_PEARL_XAUI_BOARD				1245
#define QTN_PEARL_SGMII_BOARD				1246

#define QTN_JADE_BOARD					1260

#define QTN_RUBY_AUTOCONFIG_ID				32768
#define QTN_RUBY_UNIVERSAL_BOARD_ID			65535

#define QTN_RUBY_NOSUCH_BOARD_TYPE			-1

#define QTN_RUBY_BRINGUP_RWPA				0
#define QTN_RUBY_REF_RWPA				1
#define QTN_RUBY_SIGE					2
/* 3: Can be reused */
#define QTN_RUBY_WIFI_NONE				4
#define	QTN_TPZ_SE5003L1				5
#define	QTN_TPZ_SE5003L1_INV				6
/* 7: Can be reused */
#define QTN_TPZ_SKY85405_BPF840				8
/* [9, 12]: Can be reused */
#define	QTN_TPZ_DBS_5591				13	/* BBIC4 A2 + RFIC6 */
#define	QTN_TPZ_DBS_NXP_BGU7224_BGU7258			14	/* BBIC4 A2 + RFIC6  DBS support*/
#define	QTN_TPZ_2_4GHZ_NXP_BGU7224			15	/* BBIC4 A2 + RFIC6 2.4ghz only */
#define	QTN_TPZ_5GHZ_NXP_BGU7258			16	/* BBIC4 A2 + RFIC6 5ghz only */
#define	QTN_TPZ_5GHZ_SKY85728				17	/* BBIC4 A2 + RFIC4 5ghz only and BBIC4 A2 + RFIC6 5ghz only */
#define	QTN_PRL_5GHZ_SKY85717				18	/* BBIC5 + RFIC78 5ghz only – 22 dBm FEM */
#define	QTN_PRL_5GHZ_SKY85731				19	/* BBIC5 + RFIC78 5ghz only – 25 dBm FEM */
#define	QTN_PRL_2_4GHZ_SKY85310				20	/* BBIC5 + RFIC62 2.4ghz only */
#define	QTN_PRL_5GHZ_MPWR_2x2				21	/* BBIC5 + RFIC8 5ghz only - 22 dBm FEM, 2X2 boards */
#define	QTN_PRL_2_4GHZ_SKY85331				22	/* BBIC5 + RFIC62 2.4ghz only - 2018 addition */
#define	QTN_PRL_5GHZ_SKY85743				23	/* BBIC5 + RFIC8 5ghz only - 2018, 27 dBm FEM */
#define	QTN_PRL_2_4GHZ_SKY85333				24	/* BBIC5 + RFIC62 2.4ghz only - 2018 addition */

#define QTN_DEF_PLATFORM_ID				0       /* default pltform ID is 0 */

#endif /* _QTN_BASE_SHARED_DEFS_H_ */

