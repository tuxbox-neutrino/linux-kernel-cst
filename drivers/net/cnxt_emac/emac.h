/****************************************************************************
 *
 *  drivers/net/cnxt_emac/emac.h
 *
 *  Copyright (C) 2007 Conexant Systems,Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as 
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or 
 *  NONINFRINGEMENT.See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */
/****************************************************************************
 *$Id$
 ****************************************************************************/

#ifndef __emac_h__
#define __emac_h__

#define PECOS_ETH_DEBUG		/* define this Macro to include print statements in emac driver */
							       /*#define PECOS_ETH_TESTPRINT *//* define this Macro to include print statements in key test points 
							          of emac driver */

#define EMAC_DRIVER_STATISTICS	/* Enable this macro to provide ioctl and addtional driver 
				   statistics useful to check complete error conditions, perfromance points */

#define EMAC_MEDIA_MAINTANANCE	/* Enable this macro to enable thread and logic, which 
				   checks and updates changes in the media */
#ifdef PECOS_ETH_DEBUG
#define EMACDBG(_x...) printk(KERN_EMERG _x)
#else
#define EMACDBG(_x...)
#endif

/* ToDo: Remove below macro's later, if buffer descriptor macro's are moved to enet_pecos.h */

/* Tx Control Information Written By the CPU Bit Masks */
/* Tx Length in this buffer to be Xmitted */
#define ENET_TX_CTL_INFO_CPU_TX_LEN (0x000007FF)
/* First Tx buffer in the packet */
#define ENET_TX_CTL_INFO_CPU_FIRST  (0x00010000)
/* Last Tx buffer in the packet */
#define ENET_TX_CTL_INFO_CPU_LAST   (0x00020000)
 /* Add the CRC tp the pkt */
#define ENET_TX_CTL_INFO_CPU_ADDCRC (0x00040000)
/* CPU/EMAC Ownership of buffer */
#define ENET_TX_CTL_INFO_CPU_OWN    (0x80000000)

/* Tx Control Information Written By the EMAC Bit Masks */
/* Tx Length in this buffer to be Xmitted */
#define ENET_TX_CTL_INFO_TX_LEN     (0x000007FF)
/* First Tx buffer in the packet */
#define ENET_TX_CTL_INFO_FIRST      (0x00010000)
/* Last Tx buffer in the packet */
#define ENET_TX_CTL_INFO_LAST       (0x00020000)
/* Add the CRC to the pkt that is transmitted */
#define ENET_TX_CTL_INFO_ADDCRC     (0x00040000)
/* Carrier Lost during xmission */
#define ENET_TX_CTL_INFO_CARR_LOSS  (0x00200000)
/* xmission deferred due to traffic */
#define ENET_TX_CTL_INFO_DEFER      (0x00400000)
/* pkt dropped after 16 retries */
#define ENET_TX_CTL_INFO_DROPPED    (0x00800000)
/* Retry count for Tx */
#define ENET_TX_CTL_INFO_RETRY      (0x0F000000)
#define ENET_TX_CTL_INFO_RETRY_SHIFT 24
/* Late Collision */
#define ENET_TX_CTL_INFO_LATE_COLL  (0x10000000)
/* Data not available on time */
#define ENET_TX_CTL_INFO_UFLO       (0x20000000)
/* Buffer error - bad FIRST and LAST */
#define ENET_TX_CTL_INFO_BUFF       (0x40000000)
/* CPU/EMAC Ownership of buffer */
#define ENET_TX_CTL_INFO_OWN        (0x80000000)

/* Pointer To Tx Buffer Bit Mask */
/* Physical address of the start of the buffer of data */
#define ENET_TX_BUFFER_PTR          (0xFFFFFFFF)

/* Rx Control Information Written By the EMAC Bit Masks */
/* Rx Length in this buffer to be Xmitted */
#define ENET_RX_CTL_INFO_RX_LEN     (0x000007FF)
/* First Rx buffer in the packet */
#define ENET_RX_CTL_INFO_FIRST      (0x00010000)
/* Last Rx buffer in the packet */
#define ENET_RX_CTL_INFO_LAST       (0x00020000)
/* buffer error in the packet */
#define ENET_RX_CTL_INFO_BUFF_ERROR (0x40000000)
/* Last Rx buffer in the packet */
#define ENET_RX_CTL_INFO_OWN        (0x80000000)

/* Rx Control Information Written By the CPU Bit Masks */
/* Rx Length in this buffer to be Xmitted */
#define ENET_RX_CTL_INFO_CPU_RX_LEN (0x000007FF)
/* First Rx buffer in the packet */
#define ENET_RX_CTL_INFO_CPU_FIRST  (0x00010000)
/* Last Rx buffer in the packet */
#define ENET_RX_CTL_INFO_CPU_LAST   (0x00020000)
/* CPU/VMAC Ownership of buffer */
#define ENET_RX_CTL_INFO_CPU_OWN    (0x80000000)

/* Pointer To Rx Buffer Bit Mask */
/* Physical address of the start of the buffer of data */
#define ENET_RX_BUFFER_PTR          (0xFFFFFFFF)


/* Macro for maximum PHY ID supported */
#define PHY_ADDR_MAX 0x20

/*PHY Register definations*/
/* Control register bit definitions */
#define PHY_CTRL_RESET         0x8000
#define PHY_CTRL_LOOPBACK      0x4000
#define PHY_CTRL_SPEED         0x2000
#define PHY_CTRL_AUTONEG       0x1000
#define PHY_CTRL_POWERDOWN     0x0800
#define PHY_CTRL_ISOLATE       0x0400
#define PHY_CTRL_DUPLEX        0x0100
#define PHY_CTRL_RESTART_AUTO  0x0200
#define PHY_CTRL_COLL          0x0080

/* Status register bit definitions */
#define PHY_STATUS_COMPLETE     0x20
#define PHY_STATUS_AUTONEG_ABLE 0x04
#define PHY_STATUS_100BASE_T4                  0x8000
#define PHY_STATUS_100BASEX_FULLDUPLEX         0x4000
#define PHY_STATUS_100BASEX_HALFDUPLEX         0x2000
#define PHY_STATUS_10MBPS_FULLDUPLEX           0x1000
#define PHY_STATUS_10MBPS_HALFDUPLEX           0x800
#define PHY_STATUS_MF_PREAMBLE_SUPRESSION      0x40
#define PHY_STATUS_AUTONEGOTIATION_COMPLETE    0x20
#define PHY_STATUS_REMOTE_FAULT                0x10
#define PHY_STATUS_AUTONEGOTIATION_ABILITY     0x8
#define PHY_STATUS_LINK_STATUS                 0x4
#define PHY_STATUS_JABBER_DETECT               0x2
#define PHY_STATUS_EXTENDED_CAPABILITY         0x1


/* Auto-negatiation advertisement register bit definitions */
#define PHY_AUTONEG_ADV_100BTX_FULL     0x100
#define PHY_AUTONEG_ADV_100BTX          0x80
#define PHY_AUTONEG_ADV_10BTX_FULL      0x40
#define PHY_AUTONEG_ADV_10BT            0x20
#define AUTONEG_ADV_IEEE_8023           0x1

/* Auto-negatiation Link register bit definitions */
#define PHY_AUTONEG_LINK_100BTX_FULL     0x100
#define PHY_AUTONEG_LINK_100BTX          0x80
#define PHY_AUTONEG_LINK_10BTX_FULL      0x40

/* PHY Registers */
#define PHY_CTRL_REG               0x00
#define PHY_STATUS_REG             0x01
#define PHY_IDENTIFIER_REG1        0x02
#define PHY_IDENTIFIER_REG2        0x03
#define PHY_AUTONEG_ADV_REG        0x4
#define PHY_AUTONEG_LINK_REG       0x5
#define PHY_MIRROR_REG             0x10

#define PHY_REG_MASK          0x1F
#define PHY_ADDR_MASK         0x1F

/* Specific to PHY's */
#define REALTEK8201_MEDIACHEK_REG  0x00	/* RealTek PHY speed/duplex check register */
#define REALTEK8201_PHY_SPEED      0x2000	/* RealTek PHY speed check bit */
#define REALTEK8201_PHY_MODE       0x0100	/* RealTek PHY mode check bit */


#define LXT972_MEDIACHEK_REG       0x11	/* LXT972 PHY speed/duplex check register */
#define LXT972_PHY_SPEED           0x4000	/* LXT972 PHY speed check bit */
#define LXT972_PHY_MODE            0x200	/* LXT972 PHY mode check bit */
#endif

/****************************************************************************
 * Modifications:
 * $Log$
 *
 ****************************************************************************/
