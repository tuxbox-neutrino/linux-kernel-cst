/****************************************************************************
 *
 *  drivers/net/cnxt_emac/emac_drv.c
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
 *$Id: emac_drv.c,v 1.2, 2007-08-03 09:19:05Z, Upakul Barkakaty$
 ****************************************************************************/

#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/interrupt.h>	/* mark_bh */
#include <linux/in.h>
#include <linux/netdevice.h>	/* struct device, and other headers */
#include <linux/etherdevice.h>	/* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/if.h>
#include <linux/mii.h>
#include <linux/ethtool.h>

#include "asm/arch/cx2450x.h"	/* file to get register definitions */
#include "emac.h"
#include "emac_prv.h"

MODULE_AUTHOR("Conexant Systems, Inc.");
MODULE_DESCRIPTION("CNXT Pecos ethernet driver");
MODULE_LICENSE("GPL");

#define DRV_NAME  "CNXT_EMAC"
#define VER_INFO  "V 1.0"
#define FRM_VER   "Firmware V 1.0"
#define BUS_INFO  "BVCI Interface"

/* Linux net_device structure representing an ethernet interface*/
struct net_device *gNetDevice[MAX_EMAC];
/* Linux device statistics structure*/
struct net_device_stats gNetDevStats[MAX_EMAC];
#ifdef EMAC_MEDIA_MAINTANANCE
/* There is only one thread to manage, so no need to duplicate these */
struct completion gThreadExitComplete;
volatile int gThreadExitReq;
#endif

static int emacdev_ethtool_ioctl(struct net_device *dev, struct ifreq *ifr);

/* Function provides interface/device number based on passed network device */
static int get_deviceno(struct net_device *dev, int *deviceNo)
{
	int retVal = -ENODEV;
	int devNo;

	*deviceNo = 0;
	for (devNo = 0; devNo < MAX_EMAC; devNo++) {
		if (dev == gNetDevice[devNo]) {
			*deviceNo = devNo;
			retVal = 0;
			break;
		}
	}

	return retVal;
}


/* Initialise and start the EMAC for operation*/
int emacdev_open(struct net_device *dev)
{
	int retVal, deviceNo;
	int retValAPI;

	/* Get the interface number */
	retVal = get_deviceno(dev, &deviceNo);
	if (0 == retVal) {
		retValAPI = emac_open(deviceNo, dev);
		if (0 == retValAPI) {
			netif_start_queue(dev);
		} else {
			retVal = 1;
		}
	} else {
		EMACDBG("Error : Can not Open Emac %d\n", retVal);
	}

	return retVal;
}

/* Stop EMAC operation */
int emacdev_stop(struct net_device *dev)
{
	int retVal;

	/* stop the device */
	if (netif_device_present(dev)) {
		netif_stop_queue(dev);
	}
	retVal = emac_close(dev);

	return retVal;
}

/* Registered Transmit function. Traverse through the TX buffer descriptor 
 * table. If OWN bit of the INFO field is 0 then put the buffer in the 
 * descriptor and inform  EMAC to start the packet transmission immediately
 */
int emacdev_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int retVal;

	retVal = emac_start_transmit(dev, (void *) skb, skb->len);

	return retVal;
}

/* Return the device statistics.. ex. tx_packets, rx_packets, rx_errors, 
 * rx_dropped etc...All these are fields in struct net_device_stats
 */
struct net_device_stats *emacdev_get_stats(struct net_device *dev)
{
	int retVal;
	int deviceNo;
	EMAC_STATS emacStats;

	retVal = get_deviceno(dev, &deviceNo);
	if(retVal < 0) {
		return NULL;
	}

	retVal = emac_get_device_statistics(dev, &emacStats);
	if(retVal < 0) {
		return NULL;
	}

	gNetDevStats[deviceNo].rx_packets = emacStats.uRxPktCnt;	/* total packets received       */
	gNetDevStats[deviceNo].tx_packets = emacStats.uTxPktCnt;	/* total packets transmitted    */
	gNetDevStats[deviceNo].rx_bytes = emacStats.uRxByteCnt;	/* total bytes received         */
	gNetDevStats[deviceNo].tx_bytes = emacStats.uTxByteCnt;	/* total bytes transmitted      */
	gNetDevStats[deviceNo].rx_errors = emacStats.uRxCRCErrDropCnt + emacStats.uRxFrameErrDropCnt + emacStats.uBDMissPktCnt + emacStats.uRxFifoOverflowDropCnt;	/* bad packets received         */
	gNetDevStats[deviceNo].tx_errors = emacStats.uTxUnderflowErrCnt + emacStats.uTxDropCnt + emacStats.uTxLateCollisionDropCnt + emacStats.uTxChainingErrCnt + emacStats.uTxCarrLossCnt;
	/* packet transmit problems     */
	gNetDevStats[deviceNo].tx_dropped = 0;	/* no space available in linux  */
	gNetDevStats[deviceNo].rx_dropped = 0;	/* no space in linux buffers    */
	gNetDevStats[deviceNo].multicast = emacStats.uRxMulticastPktCnt;	/* multicast packets received   */
	gNetDevStats[deviceNo].collisions = emacStats.uTxLateCollisionDropCnt;
	gNetDevStats[deviceNo].rx_length_errors = emacStats.uRxShortPktErrCnt + emacStats.uRxLongPktErrCnt;
	gNetDevStats[deviceNo].rx_over_errors = 0;	/* receiver ring buff overflow  *//* ToDo: check emacStats.uBDMissPktCnt */
	gNetDevStats[deviceNo].rx_crc_errors = emacStats.uRxCRCErrDropCnt;	/* recved pkt with crc error    */
	gNetDevStats[deviceNo].rx_frame_errors = emacStats.uRxFrameErrDropCnt;	/* recv'd frame alignment error */
	gNetDevStats[deviceNo].rx_fifo_errors = emacStats.uRxFifoOverflowDropCnt;	/* recv'r fifo overrun          */
	gNetDevStats[deviceNo].rx_missed_errors = emacStats.uBDMissPktCnt;	/* receiver missed packet       */
	gNetDevStats[deviceNo].tx_aborted_errors = 0;
	gNetDevStats[deviceNo].tx_carrier_errors = emacStats.uTxCarrLossCnt;
	gNetDevStats[deviceNo].tx_fifo_errors = emacStats.uTxUnderflowErrCnt;
	gNetDevStats[deviceNo].tx_heartbeat_errors = 0;
	gNetDevStats[deviceNo].tx_window_errors = 0;
	gNetDevStats[deviceNo].rx_compressed = 0;
	gNetDevStats[deviceNo].tx_compressed = 0;

	return (&gNetDevStats[deviceNo]);
}

/* Set the logical filtering */
void emacdev_set_multicast_list(struct net_device *dev)
{
	emac_set_multicast_list(dev);
}


/* Called by networking subsystem when a packet transmission fails to complete
 * within a reasonable period (value set earlier), on the assumtion that an 
 * interrupt has been missed or the interface has locked up. This function will
 * handle the problem and resume packet transmission It also will gather the 
 * error statistics.
 */
void emacdev_tx_timeout(struct net_device *dev)
{
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	EMACDBG("%s: transmit timed out?\n", dev->name);
	/* Reset transmit activity */
	emac_set_transmit_activity(dev, 0);
	/* If MDIO operation supported */
	if (pEmacDevice->uMDIO_support) {
		/* restart transceiver */
		phy_reset(dev);
	}
	/* Set transmit activity */
	emac_set_transmit_activity(dev, 1);
	/* logic to error state increments */
	pEmacDevice->Stats.uTxDropCnt++;
	/* Try to restart the adaptor. */
	netif_wake_queue(dev);
}

/* Perform interface specific ioctl commands*/
int emacdev_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	EMAC_DEVICE *pEmacDevice;
	int retVal = 0;
	struct mii_ioctl_data *data = if_mii(ifr);

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	/* All operation carried over MDIO interface, check it is supported */
	if (!(pEmacDevice->uMDIO_support)) {
		return -EOPNOTSUPP;
	}

	spin_lock(&pEmacDevice->lock);	/* Preempt protection */
	switch (cmd) {
	case SIOCETHTOOL:
		retVal = emacdev_ethtool_ioctl(dev, ifr);
		break;

	case SIOCGMIIPHY:	/* Get PHY address */
		data->phy_id = pEmacDevice->mii_if.phy_id;
		break;

	case SIOCGMIIREG:	/* Read MII register */
		data->val_out = pEmacDevice->mii_if.mdio_read(dev, data->phy_id, data->reg_num);
		break;

	case SIOCSMIIREG:	/* Write MII register */
		pEmacDevice->mii_if.mdio_write(dev, data->phy_id, data->reg_num, data->val_in);
		break;
#ifdef EMAC_DRIVER_STATISTICS
	case EMAC_STATISTICS_DISPLAY:
		emac_stats_display(dev);
		break;
	case EMAC_STATISTICS_CLEAR:
		emac_stats_clear(dev);
		break;
#endif

	default:
		retVal = -EINVAL;
		break;
	}
	spin_unlock(&pEmacDevice->lock);

	return retVal;
}


static int emacdev_ethtool_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct ethtool_cmd ecmd;
	EMAC_DEVICE *pEmacDevice;
#ifdef PECOS_ETH_TESTPRINT
	unsigned long tmpVal;
#endif

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	if (copy_from_user(&ecmd, ifr->ifr_data, sizeof(ecmd))) {
		return -EFAULT;
	}
#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("start: cmd = %u, supported = %x, adv = %x, speed = %u, duplex = %u \n", ecmd.cmd, ecmd.supported, ecmd.advertising, ecmd.speed, ecmd.duplex);
	EMACDBG("port = %u, phy_add = %x, transceiver = %x, autoneg = %u, maxtx = %x, maxrx = %x\n", ecmd.port, ecmd.phy_address, ecmd.transceiver, ecmd.autoneg, ecmd.maxtxpkt, ecmd.maxrxpkt);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	EMACDBG("PHY_CTRL_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	EMACDBG("PHY_STATUS_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG1);
	EMACDBG("PHY_IDENTIFIER_REG1 = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG2);
	EMACDBG("PHY_IDENTIFIER_REG2 = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_ADV_REG);
	EMACDBG("PHY_AUTONEG_ADV_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_LINK_REG);
	EMACDBG("PHY_AUTONEG_LINK_REG = %08x\n", tmpVal);
#endif

	if ((!netif_running(dev))) {
		return -EINVAL;
	}

	/* spin_lock_irq(&pEmacDevice->lock); */
	switch (ecmd.cmd) {
	case ETHTOOL_GSET:
		{
			memset((void *) &ecmd, 0, sizeof(ecmd));
			/* MII port, Can be connected to twisted pair as well SUPPORTED_TP */
			ecmd.supported = SUPPORTED_Autoneg | SUPPORTED_MII | SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full;
			ecmd.port = PORT_MII;	/* MII port */
			ecmd.transceiver = XCVR_EXTERNAL;
			ecmd.phy_address = pEmacDevice->mii_if.phy_id;
			ecmd.speed = pEmacDevice->uSpeed;
			ecmd.duplex = pEmacDevice->uMode;
			ecmd.advertising = ADVERTISED_MII;

			/* Interface running in auto negotiation */
			if (pEmacDevice->uAutoneg_enable) {
				/* Update the advartised capability */
				if ((pEmacDevice->uCurrent_speed_select == AUTO_SPEED) && (pEmacDevice->uCurrent_duplex_select == AUTO_DUPLEX)) {
					ecmd.advertising |= ADVERTISED_Autoneg | ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full;
				} else {
					ecmd.advertising |= ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full;
					if (pEmacDevice->uCurrent_speed_select == SPEED_10MBPS) {
						ecmd.advertising &= ~(ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full);
					} else if (pEmacDevice->uCurrent_speed_select == SPEED_100MBPS) {
						ecmd.advertising &= ~(ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full);
					}
					if (pEmacDevice->uCurrent_duplex_select == HALF_DUPLEX) {
						ecmd.advertising &= ~(ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Full);
					} else if (pEmacDevice->uCurrent_duplex_select == FULL_DUPLEX) {
						ecmd.advertising &= ~(ADVERTISED_10baseT_Half | ADVERTISED_100baseT_Half);
					}
				}
				/* Interface running in auto negotiation */
				ecmd.autoneg = AUTONEG_ENABLE;
			} else {
				/* Interface not running in auto negotiation because either not supports / disabled */
				ecmd.autoneg = AUTONEG_DISABLE;
			}
			if (copy_to_user(ifr->ifr_data, &ecmd, sizeof(ecmd))) {
				return -EFAULT;
			}
		}
		break;

	case ETHTOOL_SSET:
		{
			if (!capable(CAP_NET_ADMIN)) {
				return -EPERM;
			}
			if (ecmd.autoneg == AUTONEG_ENABLE) {
				if (ecmd.advertising & (ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full)) {
					pEmacDevice->uCurrent_speed_select = SPEED_100MBPS;
				} else if (ecmd.advertising & (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full)) {
					pEmacDevice->uCurrent_speed_select = SPEED_10MBPS;
				}

				if ((pEmacDevice->uCurrent_speed_select == SPEED_100MBPS) & (ecmd.advertising & (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full))) {
					pEmacDevice->uCurrent_speed_select = AUTO_SPEED;
				}
				if (ecmd.advertising & (ADVERTISED_10baseT_Full | ADVERTISED_100baseT_Full)) {
					pEmacDevice->uCurrent_duplex_select = FULL_DUPLEX;
				} else if (ecmd.advertising & (ADVERTISED_10baseT_Half | ADVERTISED_100baseT_Half)) {
					pEmacDevice->uCurrent_duplex_select = HALF_DUPLEX;
				}
				if ((pEmacDevice->uCurrent_duplex_select == FULL_DUPLEX) & (ecmd.advertising & (ADVERTISED_10baseT_Half | ADVERTISED_100baseT_Half))) {
					pEmacDevice->uCurrent_speed_select = AUTO_DUPLEX;
				}
				emac_media_autonegotiate(dev);
				pEmacDevice->uAutoneg_enable = 1;
			} else {
				emac_media_force(dev, ecmd.speed, ecmd.duplex);
				pEmacDevice->uAutoneg_enable = 0;
			}
		}
		break;

	case ETHTOOL_GDRVINFO:	/* Provides the driver information */
		{
			struct ethtool_drvinfo info;
			memset((void *) &info, 0, sizeof(info));
			strncpy(info.driver, DRV_NAME, sizeof(info.driver) - 1);
			strncpy(info.version, VER_INFO, sizeof(info.version) - 1);
			strncpy(info.fw_version, FRM_VER, sizeof(info.fw_version) - 1);
			strncpy(info.bus_info, BUS_INFO, sizeof(info.bus_info) - 1);
			info.regdump_len = 0;
			info.eedump_len = 0;
			info.testinfo_len = 0;
			if (copy_to_user(ifr->ifr_data, &info, sizeof(info))) {
				return -EFAULT;
			}
		}
		break;

	case ETHTOOL_NWAY_RST:
		{
			if (phy_autonegotiation_supported(dev)) {
				pEmacDevice->uCurrent_speed_select = AUTO_SPEED;
				pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;
				emac_media_autonegotiate(dev);
				pEmacDevice->uAutoneg_enable = 1;
			} else {
				return -EOPNOTSUPP;
			}
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	/* spin_unlock_irq(&pEmacDevice->lock); */

#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("end: cmd = %u, supported = %x, adv = %x, speed = %u, duplex = %u \n", ecmd.cmd, ecmd.supported, ecmd.advertising, ecmd.speed, ecmd.duplex);
	EMACDBG("port = %u, phy_add = %x, transceiver = %x, autoneg = %u, maxtx = %x, maxrx = %x\n", ecmd.port, ecmd.phy_address, ecmd.transceiver, ecmd.autoneg, ecmd.maxtxpkt, ecmd.maxrxpkt);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	EMACDBG("PHY_CTRL_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	EMACDBG("PHY_STATUS_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG1);
	EMACDBG("PHY_IDENTIFIER_REG1 = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG2);
	EMACDBG("PHY_IDENTIFIER_REG2 = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_ADV_REG);
	EMACDBG("PHY_AUTONEG_ADV_REG = %08x\n", tmpVal);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_LINK_REG);
	EMACDBG("PHY_AUTONEG_LINK_REG = %08x\n", tmpVal);
#endif

	return 0;
}

/* Changes the interface configuration like I/O address, intterrupt number etc. 
 * at runtime.
 */
int emacdev_set_config(struct net_device *dev, struct ifmap *map)
{
	int retVal = 0;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	if (!netif_running(dev)) {
		return -EINVAL;
	}

	if (!(pEmacDevice->uMDIO_support)) {
		return -EOPNOTSUPP;
	}

	spin_lock(&pEmacDevice->lock);	/* Preempt protection */

	switch (map->port) {
	case IF_PORT_UNKNOWN:
		/* Use autoneg */
		pEmacDevice->uCurrent_speed_select = AUTO_SPEED;
		pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;
		emac_media_autonegotiate(dev);
		pEmacDevice->uAutoneg_enable = 1;
		break;

	case IF_PORT_10BASET:
		pEmacDevice->uCurrent_speed_select = SPEED_10MBPS;
		pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;
		emac_media_autonegotiate(dev);
		pEmacDevice->uAutoneg_enable = 1;
		break;

	case IF_PORT_100BASET:
	case IF_PORT_100BASETX:
		pEmacDevice->uCurrent_speed_select = SPEED_100MBPS;
		pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;
		emac_media_autonegotiate(dev);
		pEmacDevice->uAutoneg_enable = 1;
		break;

	case IF_PORT_100BASEFX:
	case IF_PORT_10BASE2:
	case IF_PORT_AUI:
		retVal = -EOPNOTSUPP;	/* EOPNOTSUPP */
		break;

	default:
		EMACDBG("%s: Invalid media selected\n", dev->name);
		retVal = -EOPNOTSUPP;	/* EOPNOTSUPP */
		break;
	}

	spin_unlock(&pEmacDevice->lock);

	return retVal;
}

void init_netdevice(struct net_device *dev)
{
	dev->watchdog_timeo = 2 * HZ;
	if (dev->dma == 0) {
		dev->base_addr = EMAC0_BASE;
		dev->irq = IRQ_EMAC0;

		return;
	}

	dev->base_addr	= EMAC1_BASE;
	dev->irq	= IRQ_EMAC1;

	return;
}

void Initialize_private(EMAC_DEVICE * pEmacDevice)
{
	/* Initialize speed indicator stuff. */
	pEmacDevice->uCurrent_speed_select = AUTO_SPEED;	/* Auto negotiation mode */
	pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;	/* Auto negotiation mode */
	pEmacDevice->uMaxTxQDepth = MAX_TRANSMIT_QUEUE_LEN;
	pEmacDevice->puDescriptorBase = NULL;
	/* Initialize mii interface */
	pEmacDevice->uMDIO_support = 0;
	pEmacDevice->mii_if.mdio_read = NULL;	/* PHY read function */
	pEmacDevice->mii_if.mdio_write = NULL;	/* PHY write function */
	pEmacDevice->mii_if.dev = NULL;
	pEmacDevice->mii_if.phy_id_mask = PHY_ADDR_MASK;
	pEmacDevice->mii_if.reg_num_mask = PHY_REG_MASK;
	pEmacDevice->mii_if.phy_id = 0xFFFFFFFF;	/* Not valid PHY Id for this port */

	if (pEmacDevice->hNetDev->dma == 0) {
		pEmacDevice->uInterfaceNo = 0;
		pEmacDevice->uBaseAddress = EMAC0_BASE;
		pEmacDevice->iIrq = IRQ_EMAC0;
		pEmacDevice->uSpeed = EMAC0_DEFAULT_SPEED;	/* set default speed */
		pEmacDevice->uMode = EMAC0_DEFAULT_MODE;	/* set default mode */
		pEmacDevice->uTxDescCount = EMAC0_TX_DESCRIPTOR_CNT;
		pEmacDevice->uRxDescCount = EMAC0_RX_DESCRIPTOR_CNT;
		pEmacDevice->uMulticastEnable = EMAC0_DEFAULT_MULTICAST;
		pEmacDevice->uPromiscousEnable = EMAC0_DEFAULT_PROMISCOUS;
#ifdef CONFIG_EMAC0_MDIO_SUPPORT
		pEmacDevice->uMDIO_support = 1;
		pEmacDevice->mii_if.phy_id = CONFIG_EMAC0_MDIO_ADDRESS;	/* PHY Id for this port */
		pEmacDevice->mii_if.mdio_read = EmacReadReg_PHY;	/* PHY read function */
		pEmacDevice->mii_if.mdio_write = EmacWriteReg_PHY;	/* PHY write function */
		pEmacDevice->mii_if.dev = pEmacDevice->hNetDev;
#endif
	} else {
		pEmacDevice->uInterfaceNo = 1;
		pEmacDevice->uBaseAddress = EMAC1_BASE;
		pEmacDevice->iIrq = IRQ_EMAC1;
		pEmacDevice->uSpeed = EMAC1_DEFAULT_SPEED;	/* set default speed */
		pEmacDevice->uMode = EMAC1_DEFAULT_MODE;	/* set default mode */
		pEmacDevice->uTxDescCount = EMAC1_TX_DESCRIPTOR_CNT;
		pEmacDevice->uRxDescCount = EMAC1_RX_DESCRIPTOR_CNT;
		pEmacDevice->uMulticastEnable = EMAC1_DEFAULT_MULTICAST;
		pEmacDevice->uPromiscousEnable = EMAC1_DEFAULT_PROMISCOUS;
#ifdef CONFIG_EMAC1_MDIO_SUPPORT
		pEmacDevice->uMDIO_support = 1;
		pEmacDevice->mii_if.phy_id = CONFIG_EMAC1_MDIO_ADDRESS;	/* PHY Id for this port */
		pEmacDevice->mii_if.mdio_read = EmacReadReg_PHY;	/* PHY read function */
		pEmacDevice->mii_if.mdio_write = EmacWriteReg_PHY;	/* PHY write function */
		pEmacDevice->mii_if.dev = pEmacDevice->hNetDev;
#endif
	}
	/* Set the MAC address in private structure */
	Get_mac_Address(pEmacDevice->mac, pEmacDevice->uInterfaceNo, MAC_ADDR_LEN);

	spin_lock_init(&pEmacDevice->lock);
	init_MUTEX(&pEmacDevice->mutex);
}
static int emacdev_init(struct net_device *dev)
{
	EMACDBG("Inside emacdev_init function\n");
	dev->open = emacdev_open;
	dev->stop = emacdev_stop;
	dev->set_config = emacdev_set_config;
	dev->hard_start_xmit = emacdev_hard_start_xmit;
	dev->do_ioctl = emacdev_do_ioctl;
	dev->get_stats = emacdev_get_stats;
	dev->tx_timeout = emacdev_tx_timeout;
	dev->set_multicast_list = emacdev_set_multicast_list;

	Get_mac_Address(dev->dev_addr, dev->dma, MAC_ADDR_LEN);
	dev->addr_len = MAC_ADDR_LEN;

	return 0;
}
void check_autonegotiation(struct net_device *dev)
{
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);


	if (phy_autonegotiation_supported(dev)) {
		pEmacDevice->uCurrent_speed_select = AUTO_SPEED;
		pEmacDevice->uCurrent_duplex_select = AUTO_DUPLEX;
		emac_media_autonegotiate(dev);
		pEmacDevice->uAutoneg_enable = 1;
	} else {
		pEmacDevice->uAutoneg_enable = 0;
		emac_media_force(dev, pEmacDevice->uMode, pEmacDevice->uSpeed);
	}

}
static int __init emacdev_load(void)
{
	int devNo, retVal = 1;
#ifdef EMAC_MEDIA_MAINTANANCE
	int threadID;
#endif
	EMAC_DEVICE *pEmacDevice;

	EMACDBG("Inside emacdev_load function\n");
#ifdef CONFIG_CNXT_EMAC0_ENABLE
#ifdef CONFIG_CNXT_EMAC1_ENABLE
	for (devNo = 0; devNo < MAX_EMAC; devNo++)
#else
	for (devNo = 0; devNo < (MAX_EMAC - 1); devNo++)
#endif
#else
#ifdef CONFIG_CNXT_EMAC1_ENABLE
	for (devNo = 1; devNo < MAX_EMAC; devNo++)
#endif
#endif
	{
		gNetDevice[devNo] = alloc_etherdev(sizeof(EMAC_DEVICE));
		if (gNetDevice[devNo] == NULL) {
			EMACDBG("EMAC%d: Could not allocate ethernet device\n", devNo);
			return -1;
		}
		gNetDevice[devNo]->dma = devNo;
		/* Init function for EMAC */
		gNetDevice[devNo]->init = emacdev_init;
		/* Initialize global structure */
		init_netdevice(gNetDevice[devNo]);

		pEmacDevice = (EMAC_DEVICE *) netdev_priv(gNetDevice[devNo]);
		memset(gNetDevice[devNo]->priv, 0, sizeof(EMAC_DEVICE));
		/* place net_device handle in private structure */
		pEmacDevice->hNetDev = gNetDevice[devNo];
		/* Initialize private data structure */
		Initialize_private(pEmacDevice);

		/* MDIO operation Supported and MDIO(PHY) address unknown */
		if ((pEmacDevice->uMDIO_support) && (pEmacDevice->mii_if.phy_id == 0xFFFFFFFF)) {
			/* Detecting the PHY */
			if (phy_detect(gNetDevice[devNo])) {
				EMACDBG("Unable to detect the PHY\n");
				/* Free the allocated memory for emac net_device (free_netdev) */
				free_netdev(gNetDevice[devNo]);
				return -ENODEV;
			}
#ifdef EMAC_MEDIA_MAINTANANCE
			/* Selecting Media check function  */
			select_media_check_function(gNetDevice[devNo]);
#endif
		}

		if (pEmacDevice->uMDIO_support) {
			/* Check the PHY's autonegotiation capabilities */
			check_autonegotiation(gNetDevice[devNo]);
		}

		retVal = register_netdev(gNetDevice[devNo]);
		if (retVal == 0) {
			EMACDBG("EMAC %d Initialized\n", devNo);
		} else {
			EMACDBG("EMAC %d Failed To Initialize code=%d\n", devNo, retVal);
		}
	}
	if (devNo == 0) {
		EMACDBG("Failed to discover any MAC blocks. Suspicious as they are built-in\n");
		return -ENODEV;
	}
#ifdef EMAC_MEDIA_MAINTANANCE
	else {
		gThreadExitReq = 0;
		init_completion(&gThreadExitComplete);
		threadID = kernel_thread(emac_media_check, NULL, CLONE_FS);
		if (threadID < 0)
			EMACDBG("Failed to start ethernet maintenance thread: %d\n", threadID);
	}
#endif
	return retVal;
}

static void __exit emacdev_unload(void)
{
	/*Unregister the device (unregister_netdev) */
	int devNo = 0;
#ifdef EMAC_MEDIA_MAINTANANCE
	gThreadExitReq = 1;
#endif

#ifdef CONFIG_CNXT_EMAC0_ENABLE
#ifdef CONFIG_CNXT_EMAC1_ENABLE
	for (devNo = 0; devNo < MAX_EMAC; devNo++)
#else
	for (devNo = 0; devNo < (MAX_EMAC - 1); devNo++)
#endif
#else
#ifdef CONFIG_CNXT_EMAC1_ENABLE
	for (devNo = 1; devNo < MAX_EMAC; devNo++)
#endif
#endif
	{
		if (gNetDevice[devNo]) {
			unregister_netdev(gNetDevice[devNo]);
		}

		/*Free the allocated memory for emac net_device (free_netdev) */
		free_netdev(gNetDevice[devNo]);
	}
#ifdef EMAC_MEDIA_MAINTANANCE
	wait_for_completion(&gThreadExitComplete);
#endif
	EMACDBG("emac exit function\n");
}

module_init(emacdev_load);
module_exit(emacdev_unload);

/****************************************************************************
 * Modifications:
 * $Log:
 *  3    Linux_SDK 1.2         8/3/07 2:49:05 PM IST  Upakul Barkakaty
 *       Configurable flags and logic added to enable the Second Ethernet
 *       interface with 2xMode operation for Emac.
 *  2    Linux_SDK 1.1         4/16/07 1:25:22 PM IST Rajesha Kini    Updated
 *       with GPL header.
 *  1    Linux_SDK 1.0         3/2/07 3:42:22 PM IST  Vineet Seth     
 * $
 *
 ****************************************************************************/
