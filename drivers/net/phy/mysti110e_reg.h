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
 * Copyright (c) 2014, CoolStream International Ltd.
 * All Rights Reserved.
 *
 */

#ifndef __MYSTI110E_REG_H__
#define __MYSTI110E_REG_H__

/*************** Basic Mode control register ************/
#define MYSTI110E_BMCR_RST_VAL			(0x8000)
/* Enable loopback */
#define MYSTI110E_BMCR_LPBK_VAL			(0x4000)
#define MYSTI110E_BMCR_LPBK_CLR			(0xBFFF)
/* Set Speed to 1Gbps */
#define MYSTI110E_BMCR_SPEED_1G			(0x40)
/* Set Speed to 100Mbps */
#define MYSTI110E_BMCR_SPEED_100		(0x2000)
/* Set Speed */
#define MYSTI110E_BMCR_SPEED_10			(0)
/* Speed mask */
#define MYSTI110E_BMCR_SPEED_MSK		(0x2000)
/* Enable autonegotiation */
#define MYSTI110E_BMCR_AN_EN			(0x1000)
#define MYSTI110E_BMCR_AN_CLR			(0xFFFFEFFF)
/* Set power down mode */
#define MYSTI110E_BMCR_PWRDN_EN			(0x800)
/* Disable  power down mode */
#define MYSTI110E_BMCR_PWRDN_CLR		(0xF7FF)
/* Isolate PHY enable */
#define MYSTI110E_BMCR_ISO_PHY			(0x400)
/* Auto negotiation restart */
#define MYSTI110E_BMCR_AN_RESTART		(0x200)
/* Collision Test enable */
#define MYSTI110E_BMCR_COLTEST			(0x80)
/* Full duplex enable */
#define MYSTI110E_BMCR_FD_EN			(0x100)

/****** Basic Mode status Register bits ******/

/* Autonegotiation complete value */
#define MYSTI110E_BMSR_T4100BASE		(0x8000)
#define MYSTI110E_BMSR_X100BASEFD		(0x4000)
#define MYSTI110E_BMSR_X100BASEHD		(0x2000)
#define MYSTI110E_BMSR_10MBPSFD			(0x1000)
#define MYSTI110E_BMSR_10MBPSHD			(0x800)
#define MYSTI110E_BMSR_T2100BASEFD		(0x400)
#define MYSTI110E_BMSR_T2100BASEHD		(0x200)

/* Preamble suppression capability */
#define MYSTI110E_BMSR_PREAMBLE_SUP		(0x40)
#define MYSTI110E_BMSR_AN_VAL			(0x20)

#define MYSTI110E_BMCR_LPBK_VAL			(0x4000)

/* Remote fault value */
#define MYSTI110E_BMSR_RF_VAL			(0x10)

/* PHY is able to perform auto negotiation */
#define MYSTI110E_BMSR_AN_ABLE			(0x8)

#define MYSTI110E_BMSR_LINK_STAT		(0x4)

/* Jabber detected */
#define MYSTI110E_BMSR_JAB_VAL			(0x2)

/****** Auto Negotiation Advertisement Register bits ******/

/* Advertise Next page desired */
#define MYSTI110E_ANAR_NP			(0x8000)

/* Advertise remote fault */
#define MYSTI110E_ANAR_ADV_RF			(0x2000)

/* Advertise asymmetric pause */
#define MYSTI110E_ANAR_AP			(0x800)
/* Advertise pause frame support */
#define MYSTI110E_ANAR_PAUSE			(0x400)
/* Advertise 100Base-TX full duplex support */
#define MYSTI110E_ANAR_100B_TX_FD		(0x100)
/* Advertise 100Base-TX half duplex support */
#define MYSTI110E_ANAR_100B_TX_HD		(0x80)
/* Advertise 10Base-TX full duplex support */
#define MYSTI110E_ANAR_10B_TX_FD		(0x40)
/* Advertise 10Base-TX half duplex support */
#define MYSTI110E_ANAR_10B_TX_HD		(0x20)
/* Selector Field Read-only  */
#define MYSTI110E_ANAR_SELECTOR_FIELD		(0x01)

/****** 1KTCR : 1000 Base-T Master-Slave Control Register ******/

/* Maser/Slave config enable */
#define MYSTI110E_1KTCR_MS_CONFIG		(0x1000)
/* Set PHY as master */
#define MYSTI110E_1KTCR_MASTER_EN		(0x800)
/* Advertise device as Multiport */
#define MYSTI110E_1KTCR_MULTIPORT_EN		(0x400)
/* 1000 Base-T Full duplex capable */
#define MYSTI110E_1KTCR_1000BT_FD		(0x200)
/* 1000 Base-T Half duplex capable */
#define MYSTI110E_1KTCR_1000BT_HD		(0x100)

/********1KSTSR 1000 BASE-T Master-Slave Status Register *****/
#define MYSTI110E_1KSTSR_MAN_FAULT		(0x8000)
#define MYSTI110E_1KSTSR_MASTER			(0x4000)
#define MYSTI110E_1KSTSR_LOCAL_RX_STAT		(0x2000)
#define MYSTI110E_1KSTSR_REMOTE_RX_STAT		(0x1000)
#define MYSTI110E_1KSTSR_PART_FD_CAP		(0x800)
#define MYSTI110E_1KSTSR_PART_HD_CAP		(0x400)

/********** EXTENDED STATUS REGISTER ******************/
#define MYSTI110E_1KSCR_1000BASEX_FD		(0x8000)
#define MYSTI110E_1KSCR_1000BASEX_HD		(0x4000)
#define MYSTI110E_1KSCR_1000BASET_FD		(0x2000)
#define MYSTI110E_1KSCR_1000BASET_HD		(0x1000)

/**************** VENDOR SPECIFIC REGISTERS **************/
/****** STRAP options register ******/
#define MYSTI110E_STRAP_ANE			(0x8000)
#define MYSTI110E_STRAP_DUP			(0x4000)

/* Bit 13:12  similar to bits 6:13 in basic mode control register */
#define MYSTI110E_STRAP_SPD_MSK			(0x3000)
#define MYSTI110E_STRAP_SPD_1G			(0x2000)
#define MYSTI110E_STRAP_SPD_100			(0x1000)
#define MYSTI110E_STRAP_SPD_10			(0x0)

#define MYSTI110E_PHYSTS_SPEED_MASK		(0x1C)
#define MYSTI110E_PHYSTS_SPEED_10_HD		(0x4)
#define MYSTI110E_PHYSTS_SPEED_10_FD		(0x14)
#define MYSTI110E_PHYSTS_SPEED_100_HD		(0x8)
#define MYSTI110E_PHYSTS_SPEED_100_FD		(0x18)

#define MYSTI110E_POWERDN_TRUE_PWRDN_MASK	(0x1)
#define MYSTI110E_POWERDN_TRUE_PWRDN_EN		(0x1)
#define MYSTI110E_POWERDN_TRUE_PWRDN_CLR	(0x0)

#endif /* __MYSTI110E_REG_H__ */
