/*
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Copyright (c) 2009-2010, NXP Semiconductors
 * Copyright (c) 2014, CoolStream Internation Ltd.
 * All Rights Reserved.
 *
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "mysti110e_reg.h"

#define AUTO_NEG_DELAY_MULTIPLIER (300)

/* Defines */

#define PHY_TIMEOUT                 (100000)


#define ANAR_DEFAULT_VAL	(MYSTI110E_ANAR_AP		| \
				 MYSTI110E_ANAR_PAUSE		| \
				 MYSTI110E_ANAR_100B_TX_FD	| \
                        	 MYSTI110E_ANAR_100B_TX_HD	| \
				 MYSTI110E_ANAR_10B_TX_FD	| \
				 MYSTI110E_ANAR_10B_TX_HD	| \
				 MYSTI110E_ANAR_SELECTOR_FIELD)

#define ANDSP_DEBUG 0

#define MYSTI110E_APOLLO_M0_ID1 (0)
#define MYSTI110E_APOLLO_M0_ID2 (0)
#define MYSTI110E_APOLLO_M1_ID1 (0x0293)
#define MYSTI110E_APOLLO_M1_ID2 (0xC411)
#define MYSTI110E_KRONOS_A0_ID1 (0x0293)
#define MYSTI110E_KRONOS_A0_ID2 (0xC412)

#define MYSTI110E_DSPW_DCBLW_REG      (0x00)
#define MYSTI110E_DSPW_DBGCNTL_REG    (0x01)
#define MYSTI110E_DSPW_ATHR1_REG      (0x02)
#define MYSTI110E_DSPW_ATHR2_REG      (0x03)
#define MYSTI110E_DSPW_ATHR3_REG      (0x04)
#define MYSTI110E_DSPW_ATHR4_REG      (0x05)
#define MYSTI110E_DSPW_ATHR5_REG      (0x06)
#define MYSTI110E_DSPW_ATHR6_REG      (0x07)
#define MYSTI110E_DSPW_ATHR7_REG      (0x08)
#define MYSTI110E_DSPW_ONLVL_REG      (0x09)
#define MYSTI110E_DSPW_OFFLVL0123_REG (0x0A)
#define MYSTI110E_DSPW_OFFLVL4567_REG (0x0B)
#define MYSTI110E_DSPW_TIMESET_REG    (0x0C)
#define MYSTI110E_DSPW_BINIT1_REG     (0x0D)
#define MYSTI110E_DSPW_BINIT2_REG     (0x0E)
#define MYSTI110E_DSPW_TIMING1_REG    (0x0F)
#define MYSTI110E_DSPW_TIMING2_REG    (0x10)
#define MYSTI110E_DSPW_CONFIG_REG     (0x11)
#define MYSTI110E_DSPW_A1CFG_REG      (0x12)
#define MYSTI110E_DSPW_A2CFG_REG      (0x13)
#define MYSTI110E_DSPW_A3CFG_REG      (0x14)
#define MYSTI110E_DSPW_A4CFG_REG      (0x15)
#define MYSTI110E_DSPW_A5CFG_REG      (0x16)
#define MYSTI110E_DSPW_A6CFG_REG      (0x17)
#define MYSTI110E_DSPW_A7CFG_REG      (0x18)
#define MYSTI110E_DSPW_DSPCONFIG2_REG (0x19)
#define MYSTI110E_DSPW_A8CFG_REG      (0x1A)
#define MYSTI110E_DSPW_A9CFG_REG      (0x1B)
#define MYSTI110E_DSPW_A10CFG_REG     (0x1C)
#define MYSTI110E_DSPW_A11CFG_REG     (0x1D)
#define MYSTI110E_DSPW_GAIN1_REG      (0x1E)
#define MYSTI110E_DSPW_GAIN2_REG      (0x1F)

#define MYSTI110E_DSPR_GPSTS_REG      (0x00)
#define MYSTI110E_DSPR_FEQ1_REG       (0x01)
#define MYSTI110E_DSPR_FEQ2_REG       (0x02)
#define MYSTI110E_DSPR_DFE01_REG      (0x03)
#define MYSTI110E_DSPR_DFE02_REG      (0x04)
#define MYSTI110E_DSPR_DFE03_REG      (0x05)
#define MYSTI110E_DSPR_DFE04_REG      (0x06)
#define MYSTI110E_DSPR_DFE05_REG      (0x07)
#define MYSTI110E_DSPR_DFE06_REG      (0x08)
#define MYSTI110E_DSPR_DFE07_REG      (0x09)
#define MYSTI110E_DSPR_DFE08_REG      (0x0A)
#define MYSTI110E_DSPR_DFE09_REG      (0x0B)
#define MYSTI110E_DSPR_DFE10_REG      (0x0C)
#define MYSTI110E_DSPR_DFE11_REG      (0x0D)
#define MYSTI110E_DSPR_DFE12_REG      (0x0E)
#define MYSTI110E_DSPR_EQ_OUT_P2_REG  (0x0F)
#define MYSTI110E_DSPR_DSPRES_REG     (0x10)
#define MYSTI110E_DSPR_MAXERR_REG     (0x11)
#define MYSTI110E_DSPR_FEQOUT1_REG    (0x12)
#define MYSTI110E_DSPR_FEQOUT2_REG    (0x13)
#define MYSTI110E_DSPR_A3CFG_REG      (0x14)
#define MYSTI110E_DSPR_A4CFG_REG      (0x15)
#define MYSTI110E_DSPR_A5CFG_REG      (0x16)
#define MYSTI110E_DSPR_A6CFG_REG      (0x17)
#define MYSTI110E_DSPR_DF_REG         (0x18)
#define MYSTI110E_DSPR_VBERLOCK_REG   (0x19)
#define MYSTI110E_DSPR_Z_REG          (0x1A)
#define MYSTI110E_DSPR_DZ_REG         (0x1B)
#define MYSTI110E_DSPR_MAX_REGS       (MYSTI110E_DSPR_DZ_REG + 1)

#define MYSTI110E_DSPW_OFFLVL_MASK    (7)
#define MYSTI110E_DSPW_OFFLVL0_SHIFT  (0)
#define MYSTI110E_DSPW_OFFLVL1_SHIFT  (4)
#define MYSTI110E_DSPW_OFFLVL2_SHIFT  (8)
#define MYSTI110E_DSPW_OFFLVL3_SHIFT  (12)
#define MYSTI110E_DSPW_OFFLVL4_SHIFT  (0)
#define MYSTI110E_DSPW_OFFLVL5_SHIFT  (4)
#define MYSTI110E_DSPW_OFFLVL6_SHIFT  (8)
#define MYSTI110E_DSPW_OFFLVL7_SHIFT  (12)
#define MYSTI110E_DSPW_OFFLVL0(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL0_SHIFT)
#define MYSTI110E_DSPW_OFFLVL1(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL1_SHIFT)
#define MYSTI110E_DSPW_OFFLVL2(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL2_SHIFT)
#define MYSTI110E_DSPW_OFFLVL3(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL3_SHIFT)
#define MYSTI110E_DSPW_OFFLVL4(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL4_SHIFT)
#define MYSTI110E_DSPW_OFFLVL5(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL5_SHIFT)
#define MYSTI110E_DSPW_OFFLVL6(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL6_SHIFT)
#define MYSTI110E_DSPW_OFFLVL7(x)     (((x)&MYSTI110E_DSPW_OFFLVL_MASK)<<MYSTI110E_DSPW_OFFLVL7_SHIFT)


#define MYSTI110E_DSPW_OFFLVL0123_VAL ( MYSTI110E_DSPW_OFFLVL0(2) \
                                     | MYSTI110E_DSPW_OFFLVL1(2) \
                                     | MYSTI110E_DSPW_OFFLVL2(2) \
                                     | MYSTI110E_DSPW_OFFLVL3(2) )
#define MYSTI110E_DSPW_OFFLVL4567_VAL ( MYSTI110E_DSPW_OFFLVL4(2) \
                                     | MYSTI110E_DSPW_OFFLVL5(2) \
                                     | MYSTI110E_DSPW_OFFLVL6(2) \
                                     | MYSTI110E_DSPW_OFFLVL7(2) )

#define MYSTI110E_DSPW_A2CFG_RX_SRC_NORMAL_SHIFT (0)
#define MYSTI110E_DSPW_A2CFG_RX_SRC_NORMAL       (0 << MYSTI110E_DSPW_A2CFG_RX_SRC_NORMAL_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RX_SRC_BYREGS       (1 << MYSTI110E_DSPW_A2CFG_RX_SRC_NORMAL_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT        (1)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_000       (0 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_250       (1 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_375       (2 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_500       (3 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_625       (4 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_750       (5 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X1_875       (6 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXGAIN_X2_000       (7 << MYSTI110E_DSPW_A2CFG_RXGAIN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXPD_SHIFT          (4)
#define MYSTI110E_DSPW_A2CFG_RXPD_POWERED_UP     (0 << MYSTI110E_DSPW_A2CFG_RXPD_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXPD_POWERED_DN     (1 << MYSTI110E_DSPW_A2CFG_RXPD_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXMDIXEN_SHIFT      (5)
#define MYSTI110E_DSPW_A2CFG_RXMDIXEN_MDI        (0 << MYSTI110E_DSPW_A2CFG_RXMDIXEN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXMDIXEN_MDIX       (1 << MYSTI110E_DSPW_A2CFG_RXMDIXEN_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXPREPGA_SHIFT      (6)
#define MYSTI110E_DSPW_A2CFG_RXPREPGA_X0_278     (0 << MYSTI110E_DSPW_A2CFG_RXPREPGA_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXPREPGA_X0_416     (1 << MYSTI110E_DSPW_A2CFG_RXPREPGA_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXLPFBYP_SHIFT      (7)
#define MYSTI110E_DSPW_A2CFG_RXLPFBYP_ENABLED    (0 << MYSTI110E_DSPW_A2CFG_RXLPFBYP_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXLPFBYP_BYPASSED   (1 << MYSTI110E_DSPW_A2CFG_RXLPFBYP_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXCLKINV_SHIFT      (8)
#define MYSTI110E_DSPW_A2CFG_RXCLKINV_NORMAL     (0 << MYSTI110E_DSPW_A2CFG_RXCLKINV_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXCLKINV_INVERTED   (1 << MYSTI110E_DSPW_A2CFG_RXCLKINV_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXVMIDADJ_SHIFT     (9)
#define MYSTI110E_DSPW_A2CFG_RXVMIDADJ_MASK      (7)
#define MYSTI110E_DSPW_A2CFG_RXVMIDADJ_VAL(x)    (((x)&MYSTI110E_DSPW_A2CFG_RXVMIDADJ_MASK) << MYSTI110E_DSPW_A2CFG_RXVMIDADJ_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXITRIM_SHIFT       (12)
#define MYSTI110E_DSPW_A2CFG_RXITRIM_MASK        (7)
#define MYSTI110E_DSPW_A2CFG_RXITRIM_VAL(x)      (((x)&MYSTI110E_DSPW_A2CFG_RXITRIM_MASK) << MYSTI110E_DSPW_A2CFG_RXITRIM_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXDASEL_SHIFT       (15)
#define MYSTI110E_DSPW_A2CFG_RXDASEL_ADCOUT      (0 << MYSTI110E_DSPW_A2CFG_RXDASEL_SHIFT)
#define MYSTI110E_DSPW_A2CFG_RXDASEL_RAWDATA     (1 << MYSTI110E_DSPW_A2CFG_RXDASEL_SHIFT)

/* A2CFG = 0x5600 */
#define MYSTI110E_DSPW_A2CFG_VAL1 ( /*15 0  */ MYSTI110E_DSPW_A2CFG_RXDASEL_ADCOUT   \
                                | /*12 101*/ MYSTI110E_DSPW_A2CFG_RXITRIM_VAL(5)   \
                                | /*9  011*/ MYSTI110E_DSPW_A2CFG_RXVMIDADJ_VAL(3) \
                                | /*8  0  */ MYSTI110E_DSPW_A2CFG_RXCLKINV_NORMAL  \
                                | /*7  0  */ MYSTI110E_DSPW_A2CFG_RXLPFBYP_ENABLED \
                                | /*6  0  */ MYSTI110E_DSPW_A2CFG_RXPREPGA_X0_278  \
                                | /*5  0  */ MYSTI110E_DSPW_A2CFG_RXMDIXEN_MDI     \
                                | /*4  0  */ MYSTI110E_DSPW_A2CFG_RXPD_POWERED_UP  \
                                | /*1  000*/ MYSTI110E_DSPW_A2CFG_RXGAIN_X1_000    \
                                | /*0  0  */ MYSTI110E_DSPW_A2CFG_RX_SRC_NORMAL    )

#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
/* A2CFG = 0x5603 */
#define MYSTI110E_DSPW_A2CFG_VAL2 ( /*15 0  */ MYSTI110E_DSPW_A2CFG_RXDASEL_ADCOUT   \
                                | /*12 101*/ MYSTI110E_DSPW_A2CFG_RXITRIM_VAL(5)   \
                                | /*9  011*/ MYSTI110E_DSPW_A2CFG_RXVMIDADJ_VAL(3) \
                                | /*8  0  */ MYSTI110E_DSPW_A2CFG_RXCLKINV_NORMAL  \
                                | /*7  0  */ MYSTI110E_DSPW_A2CFG_RXLPFBYP_ENABLED \
                                | /*6  0  */ MYSTI110E_DSPW_A2CFG_RXPREPGA_X0_278  \
                                | /*5  0  */ MYSTI110E_DSPW_A2CFG_RXMDIXEN_MDI     \
                                | /*4  0  */ MYSTI110E_DSPW_A2CFG_RXPD_POWERED_UP  \
                                | /*1  001*/ MYSTI110E_DSPW_A2CFG_RXGAIN_X1_250    \
                                | /*0  1  */ MYSTI110E_DSPW_A2CFG_RX_SRC_BYREGS    )
#endif

#define MYSTI110E_DSPW_RXGAIN_MASK     (7)
#define MYSTI110E_DSPW_RXGAIN0_SHIFT   (0)
#define MYSTI110E_DSPW_RXGAIN0(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN0_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA0_SHIFT (3)
#define MYSTI110E_DSPW_RXPREPGA0(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA0_SHIFT)
#define MYSTI110E_DSPW_RXGAIN1_SHIFT   (4)
#define MYSTI110E_DSPW_RXGAIN1(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN1_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA1_SHIFT (7)
#define MYSTI110E_DSPW_RXPREPGA1(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA1_SHIFT)
#define MYSTI110E_DSPW_RXGAIN2_SHIFT   (8)
#define MYSTI110E_DSPW_RXGAIN2(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN2_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA2_SHIFT (11)
#define MYSTI110E_DSPW_RXPREPGA2(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA2_SHIFT)
#define MYSTI110E_DSPW_RXGAIN3_SHIFT   (12)
#define MYSTI110E_DSPW_RXGAIN3(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN3_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA3_SHIFT (15)
#define MYSTI110E_DSPW_RXPREPGA3(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA3_SHIFT)
#define MYSTI110E_DSPW_RXGAIN4_SHIFT   (0)
#define MYSTI110E_DSPW_RXGAIN4(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN4_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA4_SHIFT (3)
#define MYSTI110E_DSPW_RXPREPGA4(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA4_SHIFT)
#define MYSTI110E_DSPW_RXGAIN5_SHIFT   (4)
#define MYSTI110E_DSPW_RXGAIN5(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN5_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA5_SHIFT (7)
#define MYSTI110E_DSPW_RXPREPGA5(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA5_SHIFT)
#define MYSTI110E_DSPW_RXGAIN6_SHIFT   (8)
#define MYSTI110E_DSPW_RXGAIN6(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN6_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA6_SHIFT (11)
#define MYSTI110E_DSPW_RXPREPGA6(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA6_SHIFT)
#define MYSTI110E_DSPW_RXGAIN7_SHIFT   (12)
#define MYSTI110E_DSPW_RXGAIN7(x)      (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXGAIN7_SHIFT)
#define MYSTI110E_DSPW_RXPREPGA7_SHIFT (15)
#define MYSTI110E_DSPW_RXPREPGA7(x)    (((x)&MYSTI110E_DSPW_RXGAIN_MASK) << MYSTI110E_DSPW_RXPREPGA7_SHIFT)

/* GAIN1 = 0x8640 */
#define MYSTI110E_DSPW_GAIN1_VAL ( MYSTI110E_DSPW_RXGAIN0(0) | MYSTI110E_DSPW_RXPREPGA0(0) \
                                | MYSTI110E_DSPW_RXGAIN1(4) | MYSTI110E_DSPW_RXPREPGA1(0) \
                                | MYSTI110E_DSPW_RXGAIN2(6) | MYSTI110E_DSPW_RXPREPGA2(0) \
                                | MYSTI110E_DSPW_RXGAIN3(0) | MYSTI110E_DSPW_RXPREPGA3(1) )

/* GAIN2 = 0xFEAA */
#define MYSTI110E_DSPW_GAIN2_VAL ( MYSTI110E_DSPW_RXGAIN4(2) | MYSTI110E_DSPW_RXPREPGA4(1) \
                                | MYSTI110E_DSPW_RXGAIN5(2) | MYSTI110E_DSPW_RXPREPGA5(1) \
                                | MYSTI110E_DSPW_RXGAIN6(6) | MYSTI110E_DSPW_RXPREPGA6(1) \
                                | MYSTI110E_DSPW_RXGAIN7(7) | MYSTI110E_DSPW_RXPREPGA7(1) )

#define MYSTI110E_DSPW_DBGCNTL_NRZIOUT_SHIFT           (1)
#define MYSTI110E_DSPW_DBGCNTL_NRZIOUT_NO_SIGNAL       (0 << MYSTI110E_DSPW_DBGCNTL_NRZIOUT_SHIFT)
#define MYSTI110E_DSPW_DBGCNTL_NRZIOUT_ENABLE_SIGNAL   (1 << MYSTI110E_DSPW_DBGCNTL_NRZIOUT_SHIFT)
#define MYSTI110E_DSPW_DBGCNTL_TSTPTS_SHIFT            (2)
#define MYSTI110E_DSPW_DBGCNTL_TSTPTS_NORMAL           (0 << MYSTI110E_DSPW_DBGCNTL_TSTPTS_SHIFT)
#define MYSTI110E_DSPW_DBGCNTL_TSTPTS_TRISTATE         (1 << MYSTI110E_DSPW_DBGCNTL_TSTPTS_SHIFT)
#define MYSTI110E_DSPW_DBGCNTL_TRIGMODE_SHIFT          (3)
#define MYSTI110E_DSPW_DBGCNTL_FREEMEM_SHIFT           (7)
#define MYSTI110E_DSPW_DBGCNTL_CLK_OUTM_SHIFT          (9)
#define MYSTI110E_DSPW_DBGCNTL_CLK_OUTM_RX_125         (1 << MYSTI110E_DSPW_DBGCNTL_CLK_OUTM_SHIFT)
#define MYSTI110E_DSPW_DBGCNTL_MONITOR_SHIFT           (12)
#define MYSTI110E_DSPW_DBGCNTL_MONITOR_GAIN_BBLDTCT    (4 << MYSTI110E_DSPW_DBGCNTL_MONITOR_SHIFT)

#define MYSTI110E_DSPW_DBGCNTL_VAL                     (  MYSTI110E_DSPW_DBGCNTL_MONITOR_GAIN_BBLDTCT \
                                                       | MYSTI110E_DSPW_DBGCNTL_CLK_OUTM_RX_125)


#define MYSTI110E_DSPW_ONLVL_ONLEVEL_SHIFT             (0)
#define MYSTI110E_DSPW_ONLVL_HTHD_SHIFT                (5)
#define MYSTI110E_DSPW_ONLVL_BLTHD_SHIFT               (11)
#define MYSTI110E_DSPW_ONLVL_BLTHD_FEC_1_4             (3 << MYSTI110E_DSPW_ONLVL_BLTHD_SHIFT)

#define MYSTI110E_DSPW_ONLVL_VAL                       ( 0x03 << MYSTI110E_DSPW_ONLVL_ONLEVEL_SHIFT | \
							0x27 << MYSTI110E_DSPW_ONLVL_HTHD_SHIFT    | \
							MYSTI110E_DSPW_ONLVL_BLTHD_FEC_1_4)

#define MYSTI110E_DSPW_DCBLW_WINDOW_SHIFT              (0)
#define MYSTI110E_DSPW_DCBLW_ENABLE_SHIFT              (2)
#define MYSTI110E_DSPW_DCBLW_BYPASS_SHIFT              (3)
#define MYSTI110E_DSPW_DCBLW_COEF_SHIFT                (4)
#define MYSTI110E_DSPW_DCBLW_POLARITY_SHIFT            (7)
#define MYSTI110E_DSPW_DCBLW_FILT_MU_SHIFT             (8)
#define MYSTI110E_DSPW_DCBLW_DACPD_SHIFT               (11)
#define MYSTI110E_DSPW_DCBLW_AVG_MU_SHIFT              (12)
#define MYSTI110E_DSPW_DCBLW_DACPD_SRC_SHIFT           (15)

#define MYSTI110E_DSPW_DCBLW_VAL                       ( 1 << MYSTI110E_DSPW_DCBLW_ENABLE_SHIFT  | \
							4 << MYSTI110E_DSPW_DCBLW_COEF_SHIFT    | \
							3 << MYSTI110E_DSPW_DCBLW_FILT_MU_SHIFT | \
							1 << MYSTI110E_DSPW_DCBLW_AVG_MU_SHIFT)

#define MYSTI110E_DSPW_ATHR7_THRES7_SHIFT              (0)

#define MYSTI110E_DSPW_ATHR7_VAL                       (0x0118 << MYSTI110E_DSPW_ATHR7_THRES7_SHIFT)

#define MYSTI110E_SMI_TSTCNTL_READ_SHIFT      (15)
#define MYSTI110E_SMI_TSTCNTL_READ            (1 << MYSTI110E_SMI_TSTCNTL_READ_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_WRITE_SHIFT     (14)
#define MYSTI110E_SMI_TSTCNTL_WRITE           (1 << MYSTI110E_SMI_TSTCNTL_WRITE_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_TSTMODE_SHIFT   (10)
#define MYSTI110E_SMI_TSTCNTL_TSTMODE         (1 << MYSTI110E_SMI_TSTCNTL_TSTMODE_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_TSTMODE_SET     (1 << MYSTI110E_SMI_TSTCNTL_TSTMODE_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_TSTMODE_CLEAR   (0 << MYSTI110E_SMI_TSTCNTL_TSTMODE_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_READADDR_SHIFT  (5)
#define MYSTI110E_SMI_TSTCNTL_READADDR(x)     ((x) << MYSTI110E_SMI_TSTCNTL_READADDR_SHIFT)
#define MYSTI110E_SMI_TSTCNTL_WRITEADDR_SHIFT (0)
#define MYSTI110E_SMI_TSTCNTL_WRITEADDR(x)    ((x) << MYSTI110E_SMI_TSTCNTL_WRITEADDR_SHIFT)


/* Global Data */

/* Register Names -- addresses in decimal because datasheet uses decimal addresses */
typedef enum _MYSTI110EReg_t {
	/* Basic mode control */
	MYSTI110EBmcr = 0,

	/* Basic mode status */
	MYSTI110EBmsr = 1,

	/* PHY ID1 register */
	MYSTI110EPhyIdr1 = 2,

	/* PHY ID2 register */
	MYSTI110EPhyIdr2 = 3,

	/* Auto negotiation advertisement register */
	MYSTI110EAnar = 4,

	/* Auto negotiation link partner ability register */
	MYSTI110EAnlpar = 5,

	/* Auto negotiation expansion register */
	MYSTI110EAner = 6,

	/* Auto negotiation next page transmit register */
	MYSTI110EAnnPtr = 7,

	/* Mode Control/Status Register */
	MYSTI110EMcsr = 17,

	/* Special Mode Register */
	MYSTI110ESpecialModes = 18,

	/*SMII Configuration status Register */
	MYSTI110ESMIICfgStatus = 19,

	/* Testability / Configuration Control register */
	MYSTI110ETstCntl = 20,

	/* Testability data Read for LSB Register  */
	MYSTI110ETstRead1 = 21,

	/* Testability data Read for MSB Register  */
	MYSTI110ETstRead2 = 22,

	/* Testability / Cofiguration data Write Register */
	MYSTI110ETstWrite = 23,

	/* LED Direct Control Register (this is not a register in MYSTI110E) */
	MYSTI110ELedcr = 0x18,

	/* Vender specific register for power down control */
	MYSTI110PowerDnCtrl = 24,

	/* Control / Status Indication Register */
	MYSTI110ECtrlStatusInd = 27,

	/* Special Internal testability controls register */
	MYSTI110ESitc = 28,

	/* Interrupt Source Register register */
	MYSTI110EIntStatus = 29,

	/* Interrupt Mask Register register */
	MYSTI110EIntMask = 30,

	/* PHY SPecial Control/Status Register register (31) */
	MYSTI110EPhystsr = 31
} MYSTI110EReg_t, *pMYSTI110EReg_t;

#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
static int mysti110e_dump_smi_regs(struct phy_device *phy);

static unsigned int mysti110e_smi_reg_dump[32];
#endif

static int mysti110e_andsp_write_reg(struct phy_device *phy, u16 inRegAddr, u16 inDataVal);
static unsigned int mysti110e_andsp_reg_dump[MYSTI110E_DSPR_MAX_REGS];

static u32 phy_wr_delay;

#define MYSTI110E_TESTPAIRS  (32)
#define MYSTI110E_STARTPAIR  (0)
#define MYSTI110E_INVALIDREG (0x0BAD0000)
#define MYSTI110E_INVALIDVAL (0x0BAD0000)
#define MYSTI110E_MINREGS    (0)
#define MYSTI110E_MAXREGS    (32)
#define MYSTI110E_MINVAL     (0)
#define MYSTI110E_MAXVAL     (65535)

typedef struct _mysti110e_pair {
	u32 reg;
	u32 val;
} mysti110e_pair;

static mysti110e_pair testpairs[MYSTI110E_TESTPAIRS] = {
#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
	/*00 */ {MYSTI110E_DSPW_A2CFG_REG, MYSTI110E_DSPW_A2CFG_VAL1},
	/*01 */ {MYSTI110E_DSPW_OFFLVL0123_REG, MYSTI110E_DSPW_OFFLVL0123_VAL},
	/*02 */ {MYSTI110E_DSPW_OFFLVL4567_REG, MYSTI110E_DSPW_OFFLVL4567_VAL},
	/*03 */ {MYSTI110E_DSPW_GAIN1_REG, MYSTI110E_DSPW_GAIN1_VAL},
	/*04 */ {MYSTI110E_DSPW_GAIN2_REG, MYSTI110E_DSPW_GAIN2_VAL},
	/*05 */ {MYSTI110E_DSPW_A9CFG_REG, 0xA1},
	/*06 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*07 */ {MYSTI110E_DSPW_ONLVL_REG, MYSTI110E_DSPW_ONLVL_VAL},
	/*08 */ {MYSTI110E_DSPW_DCBLW_REG, MYSTI110E_DSPW_DCBLW_VAL},
	/*09 */ {MYSTI110E_DSPW_ATHR7_REG, MYSTI110E_DSPW_ATHR7_VAL},
	/*10 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*11 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*12 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*13 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*14 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*15 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*16 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*17 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*18 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*19 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*20 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*21 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*22 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*23 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*24 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*25 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*26 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*27 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*28 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*29 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*30 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*31 */ {MYSTI110E_DSPW_A2CFG_REG, MYSTI110E_DSPW_A2CFG_VAL2}
#else
	/*00 */ {MYSTI110E_DSPW_OFFLVL0123_REG, MYSTI110E_DSPW_OFFLVL0123_VAL},
	/*01 */ {MYSTI110E_DSPW_OFFLVL4567_REG, MYSTI110E_DSPW_OFFLVL4567_VAL},
	/*02 */ {MYSTI110E_DSPW_GAIN1_REG, MYSTI110E_DSPW_GAIN1_VAL},
	/*03 */ {MYSTI110E_DSPW_GAIN2_REG, MYSTI110E_DSPW_GAIN2_VAL},
	/*04 */ {MYSTI110E_DSPW_A2CFG_REG, MYSTI110E_DSPW_A2CFG_VAL1},
	/*05 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*06 */ {MYSTI110E_DSPW_ONLVL_REG, MYSTI110E_DSPW_ONLVL_VAL},
	/*07 */ {MYSTI110E_DSPW_DCBLW_REG, MYSTI110E_DSPW_DCBLW_VAL},
	/*08 */ {MYSTI110E_DSPW_ATHR7_REG, MYSTI110E_DSPW_ATHR7_VAL},
	/*09 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*10 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*11 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*12 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*13 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*14 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*15 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*16 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*17 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*18 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*19 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*20 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*21 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*22 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*23 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*24 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*25 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*26 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*27 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*28 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*29 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*30 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL},
	/*31 */ {MYSTI110E_INVALIDREG, MYSTI110E_INVALIDVAL}
#endif
};

static int mysti110e_read_reg(struct phy_device *phy, u16 reg, u16 *val)
{
	int ret;

	ret = phy_read(phy, reg);
	if (ret < 0)
		return ret;

	*val = (u16)ret;

	return 0;
}

static int mysti110e_write_reg(struct phy_device *phy, u16 reg, u16 val)
{
	int ret;

	ret = phy_write(phy, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}

#if 0
static void mysti110e_set_pair(u32 id, u32 reg, u32 val, u32 w_delay)
{
	phy_wr_delay = w_delay;

	if ((id >= MYSTI110E_STARTPAIR) && (id < MYSTI110E_TESTPAIRS)) {
		if ((reg >= MYSTI110E_MINREGS) && (reg < MYSTI110E_MAXREGS)) {
			testpairs[id].reg = reg;
			testpairs[id].val = val;
		} else {
			testpairs[id].reg = MYSTI110E_INVALIDREG;
			testpairs[id].val = MYSTI110E_INVALIDVAL;
		}
	}
}

static void mysti110e_get_pair(u32 id, u32 *preg, u32 *pval)
{
	*preg = MYSTI110E_INVALIDREG;
	*pval = MYSTI110E_INVALIDVAL;

	if ((id >= MYSTI110E_STARTPAIR) && (id < MYSTI110E_TESTPAIRS)) {
		*preg = testpairs[id].reg;
		*pval = testpairs[id].val;
	}
}
#endif

//-----------------------------------------------------------------------------
// FUNCTION:    MYSTI110ELockAnDSPRegs:
//
// DESCRIPTION: Function locks access to the Mysticom Analog and DSP Register
//              set. This is done by clearing TSTMODE bit[10] in SMI 
//                   TSTCNTL register. 
//
// RETURN: int
//
// NOTES:
//-----------------------------------------------------------------------------
//
static int  mysti110e_lock_andsp_regs(struct phy_device *phy)
{
	int ret;
	u32 i;

#if (ANDSP_DEBUG == 1)
	pr_err( "EPHY AnDSP LOCKED\n");
#endif

	// Setting TSTMODE=0 Twice Locks down access to DSP Regs
	// Two writes ensure any intermediate TSTMODE state is
	// reset into the locked state. 
	for (i = 0; i < 2; i++) {
		ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, MYSTI110E_SMI_TSTCNTL_TSTMODE_CLEAR);
		if (ret < 0)
			return ret;
	}

	return 0;
}

//-----------------------------------------------------------------------------
// FUNCTION:    mysti110e_unlock_andsp_regs:
//
// DESCRIPTION: Function unlocks access to the Mysticom Analog and DSP Register
//              set. This is done by toggling the TSTMODE bit[10] in MII 
//                              TSTCNTL register. 
//
// RETURN: int
//
// NOTES:
//-----------------------------------------------------------------------------
//
static int mysti110e_unlock_andsp_regs(struct phy_device *phy)
{
	int ret;

	// Ensure that if the unlock procedure is to be done that we
	// are in a known good TSTMODE state.  Safest state is from
	// the locked mode.

	ret = mysti110e_lock_andsp_regs(phy);
	if (ret < 0)
		return ret;
	// To enable TSTMODE: 1-SET 2-CLEAR 3-SET 4-PROFIT

	// STEP 1-SET
	ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, MYSTI110E_SMI_TSTCNTL_TSTMODE_SET);
	if (ret < 0)
		return ret;

	// STEP 2-CLEAR
	ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, MYSTI110E_SMI_TSTCNTL_TSTMODE_CLEAR);
	if (ret < 0)
		return ret;

	// STEP 3-SET
	ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, MYSTI110E_SMI_TSTCNTL_TSTMODE_SET);
	if (ret < 0)
		return (ret);

#if (ANDSP_DEBUG == 1)
	pr_err( "EPHY AnDSP UNLOCKED\n");
#endif

	// STEP 4-PROFIT
	return (0);
}


//-----------------------------------------------------------------------------
// FUNCTION:    mysti110e_andsp_write_reg:
//
// DESCRIPTION: Function writes the data value(input param 2) and writes it to 
//              the Analog DSP Register Address supplied in the first parameter. 
//
// RETURN: int
//
//-----------------------------------------------------------------------------
//
static int mysti110e_andsp_write_reg(struct phy_device *phy, u16 inRegAddr, u16 inDataVal)
{
	int ret = 0;
	u16 regval = 0;

	// TST_WRITE_DATA = inDataVal
	ret = mysti110e_write_reg(phy, MYSTI110ETstWrite, inDataVal);
	if (ret != 0) {
		return (ret);
	}

	regval = (MYSTI110E_SMI_TSTCNTL_WRITE | MYSTI110E_SMI_TSTCNTL_TSTMODE | MYSTI110E_SMI_TSTCNTL_WRITEADDR(inRegAddr)
	    );
	//- TST_CTRL_REG = write, testmode, write address
	ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, regval);
	if (ret != 0) {
		return (ret);
	}
#if (ANDSP_DEBUG == 1)
	pr_err( "EPHY AnDSP WRITE REG %02Xh (%02d) = %04xh\n", inRegAddr, inRegAddr, inDataVal);
#endif

	return (0);
}

static int mysti110e_install_settings(struct phy_device *phy)
{
	u32 i;

	for (i = MYSTI110E_STARTPAIR; i < MYSTI110E_TESTPAIRS; i++) {
		if ((testpairs[i].reg >= MYSTI110E_MINREGS)
		    && (testpairs[i].reg < MYSTI110E_MAXREGS)
		    && (testpairs[i].val >= MYSTI110E_MINVAL)
		    && (testpairs[i].val <= MYSTI110E_MAXVAL)) {
			int ret;

			ret = mysti110e_andsp_write_reg(phy, testpairs[i].reg & 0xFFFF, testpairs[i].val & 0xFFFF);
			if (ret < 0) {
				/* 
				   NOTE - bail on error -- phy might need to
				   be reset to get back int order.
				 */
				return (ret);
			}

			if (phy_wr_delay)
				mdelay(phy_wr_delay);
		}
	}

	return 0;
}

#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
//-----------------------------------------------------------------------------
// FUNCTION:    MYSTI110EDumpAnSmiRegs:
//
// DESCRIPTION: Function dumps the register contents of registers 0 to 31 of 
//              the Mysticom EPHY SMI Register Set. 

// RETURN: int
//
// NOTES: A local copy of the register set is stored in 
//        MYSTI110EAnSmiRegDump.
//-----------------------------------------------------------------------------
//
static int mysty110e_dump_smi_regs(struct phy_device *phy)
{
	int ret = 0;
	u16 regval = 0, regAddr;

	for (regAddr = MYSTI110EBmcr; regAddr <= MYSTI110EPhystsr; regAddr++) {
		ret = mysti110e_read_reg(phy, regAddr, &regval);
		if (ret < 0)
			return ret;

		mysti110e_smi_reg_dump[regAddr] = regval;
	}

#if (ANDSP_DEBUG == 1)
	//printk(KERN_INFO"EPHY SMI Registers Dump:\n");
	printk(KERN_INFO "EPHY SMI 00--07: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_smi_reg_dump[0], mysti110e_smi_reg_dump[1], mysti110e_smi_reg_dump[2], mysti110e_smi_reg_dump[3], mysti110e_smi_reg_dump[4], mysti110e_smi_reg_dump[5], mysti110e_smi_reg_dump[6], mysti110e_smi_reg_dump[7]);
	printk(KERN_INFO "EPHY SMI 08--0F: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_smi_reg_dump[8], mysti110e_smi_reg_dump[9], mysti110e_smi_reg_dump[10], mysti110e_smi_reg_dump[11], mysti110e_smi_reg_dump[12], mysti110e_smi_reg_dump[13], mysti110e_smi_reg_dump[14], mysti110e_smi_reg_dump[15]);
	printk(KERN_INFO "EPHY SMI 10--17: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_smi_reg_dump[16], mysti110e_smi_reg_dump[17], mysti110e_smi_reg_dump[18], mysti110e_smi_reg_dump[19], mysti110e_smi_reg_dump[20], mysti110e_smi_reg_dump[21], mysti110e_smi_reg_dump[22], mysti110e_smi_reg_dump[23]);
	printk(KERN_INFO "EPHY SMI 18--1f: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_smi_reg_dump[24], mysti110e_smi_reg_dump[25], mysti110e_smi_reg_dump[26], mysti110e_smi_reg_dump[27], mysti110e_smi_reg_dump[28], mysti110e_smi_reg_dump[29], mysti110e_smi_reg_dump[30], mysti110e_smi_reg_dump[31]);
#endif
	return (0);
}
#endif

//-----------------------------------------------------------------------------
// FUNCTION:    mysti110e_dump_andsp_regs:
//
// DESCRIPTION: Function dumps the register contents of registers 0 to 27 of 
//              the Mysticom EPHY Analog and DSP Register Set. 

// RETURN: int
//
// NOTES: A local copy of the register set is stored in 
//        mysti110e_andsp_reg_dump.
//-----------------------------------------------------------------------------
//
static int mysti110e_dump_andsp_regs(struct phy_device *phy)
{
	int ret = 0;
	u16 regval = 0, anDSPRegAddr, anDSPRegVal1, anDSPRegVal2;

	for (anDSPRegAddr = 0; anDSPRegAddr < MYSTI110E_DSPR_MAX_REGS; anDSPRegAddr++) {
		//- READ=1, TSTMODE=1, READ_ADDRESS=0x15
		regval = (MYSTI110E_SMI_TSTCNTL_READ | MYSTI110E_SMI_TSTCNTL_TSTMODE | MYSTI110E_SMI_TSTCNTL_READADDR(anDSPRegAddr)
		    );
		//- EPHY TSTCTL (Reg20). TSTMODE[10]=1
		ret = mysti110e_write_reg(phy, MYSTI110ETstCntl, regval);
		if (ret != 0) {
			return (ret);
		}
		//- 0x15 = TSTREAD1
		ret = mysti110e_read_reg(phy, MYSTI110ETstRead1, &anDSPRegVal1);
		if (ret != 0) {
			return (ret);
		}
		//- 0x16 = TSTREAD2
		ret = mysti110e_read_reg(phy, MYSTI110ETstRead2, &anDSPRegVal2);
		if (ret != 0) {
			return (ret);
		}
		mysti110e_andsp_reg_dump[anDSPRegAddr] = ((((int) (anDSPRegVal2)) << 16) | anDSPRegVal1);
	}
#if (ANDSP_DEBUG == 1)
	//pr_err("EPHY AnDSP READ Registers Dump:\n");
	pr_err( "EPHY AnDSP 00--07: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_andsp_reg_dump[0], mysti110e_andsp_reg_dump[1], mysti110e_andsp_reg_dump[2], mysti110e_andsp_reg_dump[3], mysti110e_andsp_reg_dump[4], mysti110e_andsp_reg_dump[5], mysti110e_andsp_reg_dump[6], mysti110e_andsp_reg_dump[7]);
	pr_err( "EPHY AnDSP 08--0F: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_andsp_reg_dump[0x8], mysti110e_andsp_reg_dump[0x9], mysti110e_andsp_reg_dump[0xA], mysti110e_andsp_reg_dump[0xB], mysti110e_andsp_reg_dump[0xC], mysti110e_andsp_reg_dump[0xD], mysti110e_andsp_reg_dump[0xE], mysti110e_andsp_reg_dump[0xF]);
	pr_err( "EPHY AnDSP 10--17: %06x %06x %06x %06x %06x %06x %06x %06x\n", mysti110e_andsp_reg_dump[0x10], mysti110e_andsp_reg_dump[0x11], mysti110e_andsp_reg_dump[0x12], mysti110e_andsp_reg_dump[0x13], mysti110e_andsp_reg_dump[0x14], mysti110e_andsp_reg_dump[0x15], mysti110e_andsp_reg_dump[0x16], mysti110e_andsp_reg_dump[0x17]);
	pr_err( "EPHY AnDSP 18--1B: %06x %06x %06x %06x\n", mysti110e_andsp_reg_dump[0x18], mysti110e_andsp_reg_dump[0x19], mysti110e_andsp_reg_dump[0x1A], mysti110e_andsp_reg_dump[0x1B]);
#endif
	return (0);
}

static int mysti110e_reset(struct phy_device *phy)
{
	int ret;
	u16 bmcr;

	/* Software Reset PHY */
	ret = mysti110e_read_reg(phy, MII_BMCR, &bmcr);
	if (ret < 0)
		return ret;
	bmcr |= BMCR_RESET;
	ret = mysti110e_write_reg(phy, MII_BMCR, bmcr);
	if (ret < 0)
		return ret;

	do {
		ret = mysti110e_read_reg(phy, MII_BMCR, &bmcr);
		if (ret < 0)
			return ret;
	} while (bmcr & BMCR_RESET);

	return 0;
}

//-----------------------------------------------------------------------------
// FUNCTION:    MYSTI110ESoftReset:
//
// DESCRIPTION: Function will do the soft reset of the PHY device

// RETURN:      0
//
// NOTES:
//-----------------------------------------------------------------------------
//
static int mysti110e_softreset(struct phy_device *phy)
{
	int ret = 0;

#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
	bool initDone = false;
	u32 retryCnt = 0;

	while (!initDone && retryCnt < 10) {
#if (ANDSP_DEBUG == 1)
		printk(KERN_INFO "**** RETRY COUNT = %u *****\n", retryCnt);
		printk(KERN_INFO "==== Powering down EPHY via SMI Register = %d   Val = 0x%lx \n", MYSTI110EBmcr, TMBSL_PHYMYSTI110E_BMCR_PWRDN_EN);
#endif

		/* Hard power down */
		ret = mysti110e_write_reg(phy, MII_BMCR, BMCR_PDOWN);
		if (ret < 0) {
			printk("Powering down phy failed\n");
			return ret;
		}

		mdelay(100);
#if (ANDSP_DEBUG == 1)
		printk(KERN_INFO " Powering up EPHY via SMI Register = %d   Val = 0x%lx \n", MYSTI110EBmcr, 0);
#endif

		/* Hard power up */
		ret = mysti110e_write_reg(phy, MII_BMCR, 0);
		if (ret != 0) {
			pr_err( "Powering up phy failed\n");
			return ret;
		}
		mdelay(100);
#endif
		/* All the registers will be reset */

		ret = mysti110e_reset(phy);
		if (ret < 0) {
			return (ret);
		}
//#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
//		mdelay(100);
//#endif

		/* Unlocking Access to the Mysticom's EPHY Analog and DSP Register Set */
		ret = mysti110e_unlock_andsp_regs(phy);
		if (ret != 0) {
			return (ret);
		}

		/* Display contents of the Mysticom EPHY Analog and DSP Register Set */
		ret = mysti110e_dump_andsp_regs(phy);
		if (ret != 0) {
			return (ret);
		}
#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
		ret = mysti110e_dump_smi_regs(phy);
		if (ret != 0) {
			printk("Display contents failed for EPHY SMI regs\n");
			return (ret);
		}
#endif
		ret = mysti110e_install_settings(phy);
		if (ret != 0) {
			printk("mysti110e_install_settings failed\n");
			return (ret);
		}

#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
		mdelay(100);

		ret = mysti110e_write_reg(phy, MYSTI110EMcsr, 0x00);
		if (ret != 0) {
			return (ret);
		}

		mdelay(100);
#endif
		/* Display contents of the Mysticom EPHY Analog and DSP Register Set */
		ret = mysti110e_dump_andsp_regs(phy);
		if (ret != 0) {
			return (ret);
		}
#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
		ret = mysti110e_dump_smi_regs(phy);
		if (ret != 0) {
			printk("Display contents failed for EPHY SMI regs\n");
			return (ret);
		}

		if (!(mysti110e_smi_reg_dump[MYSTI110EMcsr] & 0x40)) {
			printk("MYSTI110EMcsr value = 0x%x\n", mysti110e_smi_reg_dump[MYSTI110EMcsr]);
			initDone = 1;
		}
		retryCnt++;
#endif

		/* All AnDSP changes are done. Lock down AnDSP Register Access */
		ret = mysti110e_lock_andsp_regs(phy);
		if (ret != 0) {
			return (ret);
		}
#ifdef CONFIG_PACKET_LOSS_FIX_FOR_BAD_ONT
	}
#endif
	return ret;

}

static int mysti110e_config_init(struct phy_device *phy)
{

	int ret = 0;
	u16 regval = 0;

	/* This is an undocumented register -- clearing bit 0 has unknown effects */
	ret = mysti110e_read_reg(phy, MYSTI110ELedcr, &regval);
	if (ret != 0) {
		return ret;
	}

	regval &= 0xfffe;
	ret = mysti110e_write_reg(phy, MYSTI110ELedcr, regval);
	if (ret != 0) {
		return ret;
	}

	/* Soft Reset the PHY and initialze the AnDSP registers */
	ret = mysti110e_softreset(phy);
	if (ret != 0) {
		return (ret);
	}

	return 0;
}

static int mysti110e_suspend(struct phy_device *phy)
{
	int ret;
	u16 bmcr;

	mutex_lock(&phy->lock);
#if defined(CONFIG_ARCH_APOLLO)
	ret = mysti110e_read_reg(phy, MII_BMCR, &bmcr);
	if (ret < 0)
		return ret;

	ret = mysti110e_write_reg(phy, MII_BMCR, bmcr | BMCR_PDOWN);
	if (ret < 0)
		return ret;
#endif
#if (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) || defined (CONFIG_ARCH_KORE3) )
	ret = mysti110e_read_reg(phy, MYSTI110PowerDnCtrl, &bmcr);
	if (ret < 0)
		return ret;

	/* Move AHB clock to XTAL */
	writel((readl(SOC_CLK_GMAC0_AHBCLK) & ~(SOC_CLK_GMAC0_AHBCLK_MASK)) | 0x0, SOC_CLK_GMAC0_AHBCLK);
	bmcr = (bmcr & ~(MYSTI110E_POWERDN_TRUE_PWRDN_MASK)) | MYSTI110E_POWERDN_TRUE_PWRDN_EN;

	ret = mysti110e_write_reg(phy, MYSTI110PowerDnCtrl, bmcr);
	if (ret < 0)
		return ret;
#endif

	mutex_unlock(&phy->lock);

	return 0;
}

static int mysti110e_resume(struct phy_device *phy)
{
	int ret;
	u16 bmcr;
	
	mutex_lock(&phy->lock);

#if defined(CONFIG_ARCH_APOLLO)
	ret = mysti110e_read_reg(phy, MII_BMCR, &bmcr);
	if (ret < 0)
		return ret;

	ret = mysti110e_write_reg(phy, MII_BMCR, bmcr & ~BMCR_PDOWN);
	if (ret < 0)
		return ret;
#endif
#if (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) || defined (CONFIG_ARCH_KORE3) )
	ret = mysti110e_read_reg(phy, MYSTI110PowerDnCtrl, &bmcr);
	if (ret < 0)
		return ret;
	writel((readl(SOC_CLK_GMAC0_AHBCLK) & ~(SOC_CLK_GMAC0_AHBCLK_MASK)) | 0x2, SOC_CLK_GMAC0_AHBCLK);
	bmcr = (bmcr & ~(MYSTI110E_POWERDN_TRUE_PWRDN_MASK)) | MYSTI110E_POWERDN_TRUE_PWRDN_CLR;

	ret = mysti110e_write_reg(phy, MYSTI110PowerDnCtrl, bmcr);
	if (ret < 0)
		return ret;
#endif

	mutex_unlock(&phy->lock);

	return 0;
}

//-----------------------------------------------------------------------------

static struct phy_driver mysti110e_driver = {
	.phy_id		= 0x0293c410,
	.phy_id_mask	= 0xfffffff0,
	.name		= "MYSTI110E",
	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause | SUPPORTED_Asym_Pause),
	.flags		= PHY_POLL,
	.config_init	= mysti110e_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= mysti110e_suspend,
	.resume		= mysti110e_resume,
	.driver 	= { .owner = THIS_MODULE,},
};

static int __init mysti110e_init(void)
{
	return phy_driver_register(&mysti110e_driver);
}

static void __exit mysti110e_exit(void)
{
	phy_driver_unregister(&mysti110e_driver);
}

module_init(mysti110e_init);
module_exit(mysti110e_exit);

static struct mdio_device_id __maybe_unused mysti110e_tbl[] = {
	{ 0x0293c410, 0xfffffff0 },
	{ } /* sentinel */
};

MODULE_DEVICE_TABLE(mdio, mysti110e_tbl);
