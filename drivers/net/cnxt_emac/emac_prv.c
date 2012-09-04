/****************************************************************************
 *
 *  drivers/net/cnxt_emac/emac_prv.c
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
 *$Id: emac_prv.c,v 1.6, 2007-08-09 14:21:14Z, Upakul Barkakaty$
 ****************************************************************************/

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>	/* struct device, and other headers */
#include <linux/etherdevice.h>	/* eth_type_trans */
#include <linux/if_arp.h>	/* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>	/* for clearing the cache */
#include <asm-arm/delay.h>

#include "net/dst.h"
#include "net/xfrm.h"
#include "asm/arch/cx2450x.h"	/* file to get register definitions */
#include "emac.h"
#include "emac_prv.h"

#define FLASH_MAC0_ADDRESS
unsigned char EMAC0_ADDR[6] = { 0x00, 0x30, 0xCD, 0x01, 0x07, 0x0F };

#define FLASH_MAC1_ADDRESS
unsigned char EMAC1_ADDR[6] = { 0x00, 0x30, 0xCD, 0x01, 0x07, 0x0E };

extern unsigned char mac_address[6];
/* Value for alignment of ethernet frame field to word boundary */
#define SKB_PREFIX_LEN 2

extern struct net_device *gNetDevice[MAX_EMAC];

#ifdef EMAC_MEDIA_MAINTANANCE
extern struct completion gThreadExitComplete;
extern volatile int gThreadExitReq;
#endif

static unsigned long crc_bit(unsigned long initialResidue, unsigned long theBit);
static unsigned long crc_byte(unsigned long initialResidue, unsigned long theByte);
static unsigned long CaclulateHashBitNumber(unsigned char *uMacAddress);
static void AddMulticastHash(EMAC_DEVICE * pEmacDevice, unsigned char *uHwAddress, unsigned char uAddrLen);
static int CreateBufferDescriptorTable(EMAC_DEVICE * pEmacDevice);
static int DeleteBufferDescriptorTable(EMAC_DEVICE * pEmacDevice);
static int InitialiseEMAC(EMAC_DEVICE * pEmacDevice);
static int ResetEMAC(EMAC_DEVICE * pEmacDevice);
static int EnableISR(EMAC_DEVICE * pEmacDevice);
static int StartEMAC(EMAC_DEVICE * pEmacDevice);
static int StopEMAC(EMAC_DEVICE * pEmacDevice);
static int StartTransmit(EMAC_DEVICE * pEmacDevice);
static int emac_isr(int irq, void *dev_id);
static void RxInterruptPostProcessing(unsigned long data);
static void TxInterruptPostProcessing(unsigned long data);

#if 0
static void RxBufferRefillHandler(EMAC_DEVICE * pEmacDevice);
#endif

#ifdef EMAC_MEDIA_MAINTANANCE
/* Transceivers (PHY) specific function to check changes in mode/speed */
static void realtek8201_media_check(struct net_device *dev);
static void ip101_media_check(struct net_device *dev);
static void lxt972_media_check(struct net_device *dev);
static void general_media_check(struct net_device *dev);

/* Basic structure which holds specific media (mode/speed) check function */
struct phy_media_check {
	unsigned int phy_id;	/* phy_identifier */
	void (*media_check_func) (struct net_device * dev);	/* media checking function */
};

/* Structure which holds speed/mode checking function pointers */
static struct phy_media_check phy_list[] = {
	{ 0x02430C54, ip101_media_check       },	/* IP101 Plus */
	{ 0x00008201, realtek8201_media_check },	/* RealTek 8201 */
	{ 0x001378e2, lxt972_media_check      },	/* Intel LXT 972 */
	{ 0x00000000, general_media_check     }	        /* Generic, must be last */
};
#endif

/*Tasklet for ISR TX and Rx post processing function*/
struct tasklet_struct gRxTasklet[MAX_EMAC];
struct tasklet_struct gTxTasklet[MAX_EMAC];

#ifdef SKB_CLONE		/* sk buffer reuse logic enable */
static int refill_queue_len = 0;
struct sk_buff_head refillQueue[MAX_EMAC];
#endif


/* Open the EMAC for the operation */
int emac_open(unsigned short uInterfaceNumber, struct net_device *dev)
{
	int retVal;
	EMAC_DEVICE *pEmacDevice;
	unsigned long flags;
#ifdef SKB_CLONE		/*sk buffer reuse logic enable */
	unsigned long count;
	struct sk_buff *pBuf;
#endif

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;

	/* If interface is already in use return error else mark it as in use */
	if (0 == pEmacDevice->uInUse) {
		memcpy(pEmacDevice->mac, dev->dev_addr, MAC_ADDR_LEN);
		if (0 == uInterfaceNumber) {
			pEmacDevice->uInUse = 1;
		} else if (1 == uInterfaceNumber) {
			pEmacDevice->uInUse = 1;
		}
		/* Initialize no Transmit buffer pending */
		pEmacDevice->TxQDepth = 0;
		skb_queue_head_init(&pEmacDevice->TxQueue);	/* Tx sk buffer Queue */

#ifdef SKB_CLONE		/* sk buffer reuse logic enable */
		skb_queue_head_init(&refillQueue[pEmacDevice->uInterfaceNo]);	/* Refill sk buffer Queue */
		for (count = 0; count < MIN_REFILL_QUEUE_LEN; count++) {
			pBuf = dev_alloc_skb(EMAC_FRAMESIZE_MAX);
			if (NULL != pBuf) {
				__skb_queue_tail(&refillQueue[pEmacDevice->uInterfaceNo], pBuf);
			}
		}
		refill_queue_len = MIN_REFILL_QUEUE_LEN;
#endif

		/* Allocate memory for TX RX descriptor table and corresponding buffer */
		CreateBufferDescriptorTable(pEmacDevice);

		spin_lock_irqsave(&pEmacDevice->lock, flags);
		/* Initialise EMAC registers with proper values */
		InitialiseEMAC(pEmacDevice);

		/*  Initialise the tasklet for TX and RX operation */
		EnableISR(pEmacDevice);
		spin_unlock_irqrestore(&pEmacDevice->lock, flags);
		/* request isr */
		retVal = request_irq(pEmacDevice->iIrq, emac_isr, 0, pEmacDevice->hNetDev->name, pEmacDevice);
		spin_lock_irqsave(&pEmacDevice->lock, flags);
		/*Start EMAC operation */
		StartEMAC(pEmacDevice);
		spin_unlock_irqrestore(&pEmacDevice->lock, flags);

	} else {
		retVal = 1;
		EMACDBG("Device already opened\n");
	}
	return retVal;
}

/*Close the EMAC instance*/
int emac_close(struct net_device *dev)
{
	int retVal;
	EMAC_DEVICE *pEmacDevice;
	unsigned long flags;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;
	if (1 == pEmacDevice->uInUse) {
		spin_lock_irqsave(&pEmacDevice->lock, flags);
		/*Stop operation */
		StopEMAC(pEmacDevice);

		/*Reset registers */
		ResetEMAC(pEmacDevice);
		spin_unlock_irqrestore(&pEmacDevice->lock, flags);
		/*Deallocate memory for TX RX descriptor table and corresponding buffer */
		DeleteBufferDescriptorTable(pEmacDevice);

		/*Mark the interface as free */
		pEmacDevice->uInUse = 0;
	}

	return retVal;
}


static unsigned long crc_bit(unsigned long initialResidue, unsigned long theBit)
{
	/* Uses the given bit to continue a CRC calculation */
	unsigned long residue = initialResidue;

	if (((residue >> 31) & 1) ^ theBit) {
		residue <<= 1;
		residue ^= 0x04c11db7;
	} else {
		residue <<= 1;
	}

	return (residue);
}


static unsigned long crc_byte(unsigned long initialResidue, unsigned long theByte)
{
	/* Submits each bit of the given byte to have CRC calculated, LS Bit first */
	int loop;
	unsigned long residue = initialResidue;

	for (loop = 0; loop < 8; loop++) {
		residue = crc_bit(residue, theByte & 1);
		theByte >>= 1;
	}

	return (residue);
}


static unsigned long CaclulateHashBitNumber(unsigned char *uMacAddress)
{
	unsigned long residue = 0xffffffff;
	unsigned long a;
	unsigned long b = 0;
	unsigned long i;

	for (i = 0; i < 6; i++) {
		unsigned long my_byte = uMacAddress[i];
		residue = crc_byte(residue, my_byte);
	}

	/* The ARC block is different to other ethernet blocks :
	 * The LS 6 bits of the CRC are used, and they must
	 * be bit reversed
	 */

	/* Get LS 6 bits */
	a = residue & 0x3f;

	/* Bit reverse these 6 bits */
	for (i = 0; i < 6; i++) {
		if (a & (1 << i)) {
			b |= (1 << (5 - i));
		}
	}

	return (b);
}

static void AddMulticastHash(EMAC_DEVICE * pEmacDevice, unsigned char *uHwAddress, unsigned char uAddrLen)
{
	unsigned long maskbit = 0;
	unsigned long hashbit = 0;

	hashbit = CaclulateHashBitNumber(uHwAddress);

	/* set the hash registers high or low depending on the hashbit. */
	if (hashbit < (2 * 32)) {
		if (hashbit <= 31) {
			/* set the bit in the lower hash register */
			maskbit = 1 << hashbit;
			/* set the bit in the hashlow register */
			pEmacDevice->uHashLow = (pEmacDevice->uHashLow | maskbit);
			(*((volatile unsigned long *) EMAC_LAFL_REG(pEmacDevice->uInterfaceNo))) = pEmacDevice->uHashLow;
		} else {
			maskbit = 1 << (hashbit - 32);
			pEmacDevice->uHashHigh = (pEmacDevice->uHashHigh | maskbit);
			(*((volatile unsigned long *) EMAC_LAFH_REG(pEmacDevice->uInterfaceNo))) = pEmacDevice->uHashHigh;
		}
	}
}

int emac_set_multicast_list(struct net_device *dev)
{
	EMAC_DEVICE *pEmacDevice;
	unsigned long tmpVal;
	struct dev_mc_list *mcList;
	int retVal;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;

	pEmacDevice->uHashHigh = 0;
	pEmacDevice->uHashLow = 0;
	pEmacDevice->uMulticastEnable = 0;
	pEmacDevice->uPromiscousEnable = 0;
	/* Clear (set 0) hash filter values (to accept or reject the 64 group of 
	   logical address) to LAF  registers (LAFL, LAFH). */
	(*((volatile unsigned long *) EMAC_LAFL_REG(pEmacDevice->uInterfaceNo))) = 0;
	(*((volatile unsigned long *) EMAC_LAFH_REG(pEmacDevice->uInterfaceNo))) = 0;
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) & (~(1UL << 11));
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	if ((dev->flags & IFF_PROMISC)) {	/*Promiscous enable */
		tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
		tmpVal |= (1UL << 11);
		(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;
		pEmacDevice->uPromiscousEnable = 1;
	} else if (dev->flags & IFF_ALLMULTI) {	/* All multi cast enable */
		(*((volatile unsigned long *) EMAC_LAFL_REG(pEmacDevice->uInterfaceNo))) = 0xFFFFFFFF;
		(*((volatile unsigned long *) EMAC_LAFH_REG(pEmacDevice->uInterfaceNo))) = 0xFFFFFFFF;
		pEmacDevice->uMulticastEnable = 1;

	} else {
		if (0 != dev->mc_count) {	/*Multiast enabled */
			for (mcList = dev->mc_list; (NULL != mcList); mcList = mcList->next) {
				AddMulticastHash(pEmacDevice, mcList->dmi_addr, mcList->dmi_addrlen);
				pEmacDevice->uMulticastEnable = 1;
			}
		}
	}

	return retVal;
}

/* Update the BDT to transmit the frame, if queue length is 1. else update the queue */
int emac_start_transmit(struct net_device *dev, void *pData, unsigned long uDataLen)
{
	EMAC_DEVICE *pEmacDevice;
	int retVal;
	struct sk_buff *pBuf = (struct sk_buff *) pData;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;
	/* Non Transmitted sk buffers numbers crossed maximum limit? */
	if (pEmacDevice->TxQDepth < pEmacDevice->uMaxTxQDepth) {

		__skb_queue_tail(&pEmacDevice->TxQueue, pBuf);	/* Update latest received sk buffer to queue */
		pEmacDevice->TxQDepth++;	/* increment Non Transmitted sk buffers number */

		/* Transmit sk buffer immediately, if number of Tx sk buffer is 1 */
		if (pEmacDevice->TxQDepth == 1) {
			retVal = StartTransmit(pEmacDevice);
		}

	} else {
		netif_stop_queue(dev);	/* CR 30348 */
		retVal = -1;	/* return driver busy condition */
	}

	return retVal;
}

int emac_get_device_attribute(struct net_device *dev, EMAC_ATTRIBUTES * pAttributes)
{
	int retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 0;
	pAttributes->uInterfaceNo = pEmacDevice->uInterfaceNo;
	memcpy(pAttributes->uMACaddress, pEmacDevice->mac, 6);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG1);
	tmpVal = (tmpVal << 16);
	tmpVal |= EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG2);
	pAttributes->uPHYIndentifier = tmpVal;
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	if (PHY_STATUS_100BASE_T4 == (PHY_STATUS_100BASE_T4 & tmpVal)) {
		pAttributes->u100BaseT4Ability = 1;
	} else {
		pAttributes->u100BaseT4Ability = 0;
	}

	if (PHY_STATUS_100BASEX_FULLDUPLEX == (PHY_STATUS_100BASEX_FULLDUPLEX & tmpVal)) {
		pAttributes->u100BaseXFullDuplexAbility = 1;
	} else {
		pAttributes->u100BaseXFullDuplexAbility = 0;
	}

	if (PHY_STATUS_100BASEX_HALFDUPLEX == (PHY_STATUS_100BASEX_HALFDUPLEX & tmpVal)) {
		pAttributes->u100BaseXHalfDuplexAbility = 1;
	} else {
		pAttributes->u100BaseXHalfDuplexAbility = 0;
	}

	if (PHY_STATUS_10MBPS_FULLDUPLEX == (PHY_STATUS_10MBPS_FULLDUPLEX & tmpVal)) {
		pAttributes->u10MbsFullDuplexAbility = 1;
	} else {
		pAttributes->u10MbsFullDuplexAbility = 0;
	}

	if (PHY_STATUS_10MBPS_HALFDUPLEX == (PHY_STATUS_10MBPS_HALFDUPLEX & tmpVal)) {
		pAttributes->u10MbsHalfDuplexAbility = 1;
	} else {
		pAttributes->u10MbsHalfDuplexAbility = 0;
	}

	if (PHY_STATUS_MF_PREAMBLE_SUPRESSION == (PHY_STATUS_MF_PREAMBLE_SUPRESSION & tmpVal)) {
		pAttributes->uMFPreambleSuppression = 1;
	} else {
		pAttributes->uMFPreambleSuppression = 0;
	}

	if (PHY_STATUS_AUTONEGOTIATION_ABILITY == (PHY_STATUS_AUTONEGOTIATION_ABILITY & tmpVal)) {
		pAttributes->uAutoNegotiationCap = 1;
	} else {
		pAttributes->uAutoNegotiationCap = 0;
	}


	if (PHY_STATUS_LINK_STATUS == (PHY_STATUS_LINK_STATUS & tmpVal)) {
		pAttributes->uLinkStatus = 1;
	} else {
		pAttributes->uLinkStatus = 0;
	}


	if (PHY_STATUS_EXTENDED_CAPABILITY == (PHY_STATUS_EXTENDED_CAPABILITY & tmpVal)) {
		pAttributes->uExtendedPHYRegisterSet = 1;
	} else {
		pAttributes->uExtendedPHYRegisterSet = 0;
	}

	return retVal;
}

int emac_get_device_statistics(struct net_device *dev, EMAC_STATS * emacStats)
{
	int retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 0;
	/*Read the Missed Packet Counter register */
	tmpVal = (*((volatile unsigned long *) EMAC_MISS_REG(pEmacDevice->uInterfaceNo)));
	pEmacDevice->Stats.uBDMissPktCnt += ((1UL << 19) & tmpVal);
	/*Read the Missed Receive Error Counter register */
	tmpVal = (*((volatile unsigned long *) EMAC_RXERR_REG(pEmacDevice->uInterfaceNo)));
	pEmacDevice->Stats.uRxCRCErrDropCnt += (0x000000FF & tmpVal);
	pEmacDevice->Stats.uRxFrameErrDropCnt += ((0x0000FF00 & tmpVal) >> 8);
	pEmacDevice->Stats.uRxFifoOverflowDropCnt += ((0x00FF0000 & tmpVal) >> 16);
	*emacStats = pEmacDevice->Stats;

	return retVal;
}




/*Allocate memory for the Tx and Rx Buffer Descriptor Tables, Also allocate memory for Receive buffer*/
static int CreateBufferDescriptorTable(EMAC_DEVICE * pEmacDevice)
{
	unsigned long sizeBDT;
	EMAC_BUF_DESCR *bufDescr;
	EMAC_LOCAL_BUF *localBuf;
	int i;
	unsigned long tmpAddr;
	unsigned long lenBuf;
	struct sk_buff *pBuf;

	int retVal;
	retVal = 0;


	if (NULL != pEmacDevice->puDescriptorBase) {

		pEmacDevice->puDescriptorBase = NULL;
		/* return error: unexpected path */
		return -1;
	}
#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("CNXT_DMA_ALLOC_MEM \n");
#endif

	sizeBDT = ((pEmacDevice->uTxDescCount + pEmacDevice->uRxDescCount) * sizeof(EMAC_BUF_DESCR)) + ((pEmacDevice->uTxDescCount + pEmacDevice->uRxDescCount) * sizeof(EMAC_LOCAL_BUF)) + 32;

	pEmacDevice->puDescriptorBase = (unsigned char *) dma_alloc_coherent(NULL, sizeBDT, &pEmacDevice->uDmaAddr, GFP_KERNEL);

#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("pEmacDevice->puDescriptorBase  = %lx, sizeBDT = %lx,pEmacDevice->uDmaAddr = %lx \n", (unsigned long) pEmacDevice->puDescriptorBase, sizeBDT, (unsigned long) pEmacDevice->uDmaAddr);
#endif


	if (NULL != pEmacDevice->puDescriptorBase) {
		pEmacDevice->uDescriptorSize = sizeBDT;
		tmpAddr = (unsigned long) (pEmacDevice->puDescriptorBase);
		tmpAddr += 0x7;
		tmpAddr &= ~(0x7);
		/*TX Buffer Descriptor Table Information */
		pEmacDevice->tx_first_desc = (EMAC_BUF_DESCR *) tmpAddr;
		pEmacDevice->tx_last_desc = pEmacDevice->tx_first_desc + (pEmacDevice->uTxDescCount - 1);
		pEmacDevice->tx_first_local = (EMAC_LOCAL_BUF *) (pEmacDevice->tx_last_desc + 1);
		pEmacDevice->tx_refil_local = pEmacDevice->tx_first_local;
		/*TX BDT ptr field initialization */
		bufDescr = pEmacDevice->tx_first_desc;
		localBuf = pEmacDevice->tx_first_local;
		for (i = 0; i < pEmacDevice->uTxDescCount; i++) {
			/* Prepare the descriptor */
			localBuf->puData = NULL;
			bufDescr->puData = NULL;
			bufDescr->uInfo = ENET_TX_CTL_INFO_CPU_FIRST | ENET_TX_CTL_INFO_CPU_LAST;
			bufDescr++;
			localBuf++;
		}
		/*TX Buffer Descriptor Table Information */
		pEmacDevice->tx_curr = pEmacDevice->tx_first_desc;
		pEmacDevice->tx_curr_local = pEmacDevice->tx_first_local;
		pEmacDevice->tx_refil = pEmacDevice->tx_first_desc;

		/* First round up base pointer */
		tmpAddr = (unsigned long) (pEmacDevice->tx_first_local + pEmacDevice->uTxDescCount);
		tmpAddr += 0x7;
		tmpAddr &= ~(0x7);
		pEmacDevice->rx_first_desc = (EMAC_BUF_DESCR *) tmpAddr;
		pEmacDevice->rx_last_desc = pEmacDevice->rx_first_desc + (pEmacDevice->uRxDescCount - 1);
		pEmacDevice->rx_first_local = (EMAC_LOCAL_BUF *) (pEmacDevice->rx_last_desc + 1);
		pEmacDevice->rx_refil_local = pEmacDevice->rx_first_local;

		/* Fill them in */
		bufDescr = pEmacDevice->rx_first_desc;
		localBuf = pEmacDevice->rx_first_local;
		for (i = 0; i < pEmacDevice->uRxDescCount; i++) {
			lenBuf = EMAC_FRAMESIZE_MAX;
			pBuf = dev_alloc_skb(lenBuf);
			skb_reserve(pBuf, SKB_PREFIX_LEN);
			localBuf->puData = pBuf;
			bufDescr->puData = (unsigned char *) dma_map_single(NULL, pBuf->tail, EMAC_RX_FRAMESIZE_MAX, DMA_FROM_DEVICE);
			/* To prevent Rx buffer chaining the value should be greater than max frame size expected */
			bufDescr->uInfo = ENET_RX_CTL_INFO_OWN | EMAC_RX_FRAMESIZE_MAX;
			bufDescr++;
			localBuf++;
		}
		pEmacDevice->rx_refil = pEmacDevice->rx_first_desc;
		pEmacDevice->rx_curr = pEmacDevice->rx_first_desc;
		pEmacDevice->rx_curr_local = pEmacDevice->rx_first_local;
	} else {
		EMACDBG("EMAC: Failed to allocate memory for buffer descriptor table\n");
		retVal = 1;
	}

	return retVal;
}

/*Free memory allocated earlier for the Tx and Rx Buffer Descriptor Tables, Also free memory for Receive buffer*/
static int DeleteBufferDescriptorTable(EMAC_DEVICE * pEmacDevice)
{
	EMAC_BUF_DESCR *bufDescr;
	EMAC_LOCAL_BUF *localBuf;
	unsigned long i;
	struct sk_buff *pBuf;
	int retVal = 0;

	bufDescr = pEmacDevice->rx_first_desc;
	localBuf = pEmacDevice->rx_first_local;
	/* Free all Rx SK buff's */
	for (i = 0; i < pEmacDevice->uRxDescCount; i++) {
		if (NULL != localBuf->puData) {
			pBuf = (struct sk_buff *) (localBuf->puData);
			dev_kfree_skb(pBuf);
			bufDescr->puData = NULL;
			localBuf->puData = NULL;
		}

		bufDescr++;
		localBuf++;
	}
	/* Free all Tx SK buff's */
	bufDescr = pEmacDevice->tx_first_desc;
	localBuf = pEmacDevice->tx_first_local;
	for (i = 0; i < pEmacDevice->uTxDescCount; i++) {
		if (NULL != localBuf->puData) {
			pBuf = (struct sk_buff *) (localBuf->puData);
			dev_kfree_skb(pBuf);
			bufDescr->puData = NULL;
			localBuf->puData = NULL;
		}
		bufDescr++;
		localBuf++;
	}

	/* Delete all non transmitted SK buff */
	while (0 != skb_queue_len(&pEmacDevice->TxQueue)) {
		pBuf = __skb_dequeue(&pEmacDevice->TxQueue);
		dev_kfree_skb(pBuf);
	}
#ifdef SKB_CLONE
	/* Delete all non used refil SK buff */
	while (0 != skb_queue_len(&refillQueue[pEmacDevice->uInterfaceNo])) {
		pBuf = __skb_dequeue(&refillQueue[pEmacDevice->uInterfaceNo]);
		dev_kfree_skb(pBuf);
	}
#endif
#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("Free CNXT_DMA_ALLOC_MEM\n");
#endif
	/*Free the memory allocated for Tx and Rx BDT */
	dma_free_coherent(NULL, pEmacDevice->uDescriptorSize, pEmacDevice->puDescriptorBase, pEmacDevice->uDmaAddr);
	pEmacDevice->puDescriptorBase = NULL;

	return retVal;
}


/*Initialise registers with proper values. Values are written into the registers
  by using the memory base address for EMAC and the register offset*/
static int InitialiseEMAC(EMAC_DEVICE * pEmacDevice)
{
	unsigned long tmpVal;
	unsigned long tmpAddr;
	unsigned long AddHi, AddLw;
	int retVal = 0;

	/*Read the ID register and verify the revision of the EMAC hardware. */
	tmpVal = (*((volatile unsigned long *) EMAC_ID_REG(pEmacDevice->uInterfaceNo)));
	if (tmpVal == 0x0007FD02) {
		/*Write the Ethernet MAC address to ADDR registers (ADDRL, ADDRS). */
		tmpVal = (pEmacDevice->mac[3] << 24) | (pEmacDevice->mac[2] << 16) | (pEmacDevice->mac[1] << 8) | (pEmacDevice->mac[0] << 0);
		(*((volatile unsigned long *) EMAC_ADDRL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;
		tmpVal = (pEmacDevice->mac[5] << 8) | (pEmacDevice->mac[4] << 0);
		(*((volatile unsigned long *) EMAC_ADDRH_REG(pEmacDevice->uInterfaceNo))) = tmpVal;
		/*Clear (set 0) hash filter values (to accept or reject the 64 group of 
		   logical address) to LAF  registers (LAFL, LAFH). */
		(*((volatile unsigned long *) EMAC_LAFL_REG(pEmacDevice->uInterfaceNo))) = 0;
		(*((volatile unsigned long *) EMAC_LAFH_REG(pEmacDevice->uInterfaceNo))) = 0;

		AddLw = (unsigned long) pEmacDevice->puDescriptorBase;
		AddHi = (unsigned long) pEmacDevice->tx_first_desc;

		/*Write address of the start of transmit buffer descriptor table to TXRINGPTR register. */
		tmpVal = AddHi - AddLw;

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("AddHi  = %lx, AddLw = %lx,tmpVal = %lx \n", AddHi, AddLw, tmpVal);
#endif
		tmpAddr = ((unsigned long) (pEmacDevice->uDmaAddr) + tmpVal);

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("pEmacDevice->uDmaAddr  = %lx, tmpAddr = %lx \n", (unsigned long) pEmacDevice->uDmaAddr, tmpAddr);
#endif

		(*((volatile unsigned long *) EMAC_TXRINGPTR_REG(pEmacDevice->uInterfaceNo))) = tmpAddr;

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("TXRINGPTR_REG = %lx \n", tmpAddr);
#endif


		AddLw = (unsigned long) pEmacDevice->puDescriptorBase;
		AddHi = (unsigned long) pEmacDevice->rx_first_desc;

		/*Write address of the start of transmit buffer descriptor table to TXRINGPTR register. */
		tmpVal = AddHi - AddLw;

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("AddHi  = %lx, AddLw = %lx,tmpVal = %lx \n", AddHi, AddLw, tmpVal);
#endif

		tmpAddr = ((unsigned long) (pEmacDevice->uDmaAddr) + tmpVal);

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("pEmacDevice->uDmaAddr  = %lx, tmpAddr = %lx \n", (unsigned long) pEmacDevice->uDmaAddr, tmpAddr);
#endif

		(*((volatile unsigned long *) EMAC_RXRINGPTR_REG(pEmacDevice->uInterfaceNo))) = tmpAddr;

#ifdef PECOS_ETH_TESTPRINT
		EMACDBG("RXRINGPTR_REG = %lx \n", tmpAddr);
#endif

		/*Right the poll rate value to POOLRATE register. */
		(*((volatile unsigned long *) EMAC_POLLRATE_REG(pEmacDevice->uInterfaceNo))) = EMAC_POLLRATE_VALUE;
		/*Write No of descriptor in TX and RX BDT to CONTROL register.
		   Depending on the value supplied in pCaps Set  the Mode 
		   (Full duplex, half duplex) and promiscous flag by writing into MDIO register. */
		tmpVal = pEmacDevice->uTxDescCount << 16;
		tmpVal |= pEmacDevice->uRxDescCount << 24;
		if (FULL_DUPLEX == pEmacDevice->uMode) {
			tmpVal |= (1UL << 10);

		}
		if (1 == pEmacDevice->uPromiscousEnable) {
			tmpVal |= (1UL << 11);
		}

		(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

		if (pEmacDevice->uInterfaceNo) {
			/* Change the default tx fifo threshold */
			tmpVal = ((*((volatile unsigned long *) EMAC_XTRACTRL_REG(pEmacDevice->uInterfaceNo))) | (0x000003FF)) & EMAC1_TX_FIFO_THRESHOLD;
			tmpVal |= (1UL << 19) | (1UL << 18) | (1UL << 17) | (1UL << 16) | (1UL << 22) | (1UL << 23);
			(*((volatile unsigned long *) EMAC_XTRACTRL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;
		} else {
			/* Change the default tx fifo threshold */
			tmpVal = ((*((volatile unsigned long *) EMAC_XTRACTRL_REG(pEmacDevice->uInterfaceNo))) | (0x000003FF)) & EMAC0_TX_FIFO_THRESHOLD;
			tmpVal |= (1UL << 19) | (1UL << 18) | (1UL << 17) | (1UL << 16) | (1UL << 22) | (1UL << 23);
			(*((volatile unsigned long *) EMAC_XTRACTRL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

		}

		/*Set all member in Status to 0 */
		memset(&pEmacDevice->Stats, 0, sizeof(EMAC_STATS));
	} else {
		retVal = 1;
	}

	return retVal;
}


static int ResetEMAC(EMAC_DEVICE * pEmacDevice)
{
	unsigned long count, tmpVal;
	int retVal = 0;

	/*Disable the EMAC by writing 0 to Control Register */
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = 0;
	/*Disable All Interrupt */
	(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = 0;
	/* Clear Tx/RX and ERR Interrupt Status bits */
	(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = ((1UL << 1) + 1 + (1UL << 2));
	free_irq(pEmacDevice->iIrq, pEmacDevice);
	(*((volatile unsigned long *) EMAC_LAFL_REG(pEmacDevice->uInterfaceNo))) = 0;
	(*((volatile unsigned long *) EMAC_LAFH_REG(pEmacDevice->uInterfaceNo))) = 0;
	(*((volatile unsigned long *) EMAC_TXRINGPTR_REG(pEmacDevice->uInterfaceNo))) = 0;
	(*((volatile unsigned long *) EMAC_RXRINGPTR_REG(pEmacDevice->uInterfaceNo))) = 0;
	/* Clear the Error counters */
	(*((volatile unsigned long *) EMAC_RXERR_REG(pEmacDevice->uInterfaceNo)));
	(*((volatile unsigned long *) EMAC_MISS_REG(pEmacDevice->uInterfaceNo)));
	/* Wait till 0 is written into Ring pointer registers */
	count = 0;
	do {
		count++;
		tmpVal = (*((volatile unsigned long *) EMAC_TXPTRREAD_REG(pEmacDevice->uInterfaceNo)));
		tmpVal |= (*((volatile unsigned long *) EMAC_RXPTRREAD_REG(pEmacDevice->uInterfaceNo)));
		udelay(EMAC_DELAY);
		if (count > EMAC_LOCKUP_WAIT) {
			break;
		}
	}
	while (tmpVal != 0);
#ifdef PECOS_ETH_TESTPRINT
	EMACDBG("Reset_EMAC time count = %lu \n", count);
#endif
	/*Set all member in Status to 0 */
	memset(&pEmacDevice->Stats, 0, sizeof(EMAC_STATS));

	return retVal;
}

/* Register the ISR, Initialise the tasklet for TX and RX operation */
static int EnableISR(EMAC_DEVICE * pEmacDevice)
{
	int retVal = 0;
//	unsigned long flags;

	/* Initialise the tasklet for the ISR Tx and RX post processing function */
	tasklet_init(&gRxTasklet[pEmacDevice->uInterfaceNo], RxInterruptPostProcessing, (long) pEmacDevice);
	tasklet_init(&gTxTasklet[pEmacDevice->uInterfaceNo], TxInterruptPostProcessing, (long) pEmacDevice);

	if (0 == retVal) {
		/* Initialise the tasklet for the ISR Tx and RX post processing function */
		/* Clear all Interrupt Status bits */
		(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = INTERRUPT_STATUS_CLEAR;
		/* Enable all Interrupts */
		(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = (1 | (1UL << 1) | (1UL << 2));
	}

	return retVal;
}



/*Start EMAC operation*/
static int StartEMAC(EMAC_DEVICE * pEmacDevice)
{
	unsigned long tmpVal;
	int retVal = 0;

	/*Write No of descriptor in TX and RX BDT to CONTROL register.
	   Full duplex/half duplex and promiscous flag. */
	tmpVal = pEmacDevice->uTxDescCount << 16;
	tmpVal |= pEmacDevice->uRxDescCount << 24;

	if (FULL_DUPLEX == pEmacDevice->uMode) {
		tmpVal |= (1UL << 10);
	}
	if (1 == pEmacDevice->uPromiscousEnable) {
		tmpVal |= (1UL << 11);
	}

	/*Start Emac operation */
	tmpVal |= (1 | (1UL << 3) | (1UL << 4));
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return retVal;
}



/*Stop EMAC operation*/
static int StopEMAC(EMAC_DEVICE * pEmacDevice)
{
	unsigned long tmpVal;
	int retVal = 0;

	/* Stop the EMAC operation */
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	tmpVal &= (~(1 | (1UL << 3) | (1UL << 4)));
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return retVal;
}

/* Get the mac address from the uMacDest */
void Get_mac_Address(unsigned char *uMacDest, unsigned short uInterfaceNo, unsigned short uAddrlen)
{
	unsigned short tmpVal;
	unsigned char *uMacSource = NULL;

	if (0 == uInterfaceNo) {
		if ((mac_address[0] == 0) && (mac_address[1] == 0) && (mac_address[2] == 0) && (mac_address[3] == 0) && (mac_address[4] == 0) && (mac_address[5] == 0)) {
			uMacSource = (char *) EMAC0_ADDR;
		} else {
			uMacSource = (char *) mac_address;
		}
	} else if (1 == uInterfaceNo) {
		uMacSource = (char *) EMAC1_ADDR;
	}

	for (tmpVal = 0; tmpVal < uAddrlen; tmpVal++) {
		*uMacDest = *uMacSource;
		uMacDest++;
		uMacSource++;
	}

}

static int StartTransmit(EMAC_DEVICE * pEmacDevice)
{
	struct sk_buff *puDataBuf;
	EMAC_BUF_DESCR *tx_desc_ptr;
	EMAC_LOCAL_BUF *tx_local_ptr;
	int done = 0;
	int retVal = 0;

	tx_desc_ptr = pEmacDevice->tx_curr;
	tx_local_ptr = pEmacDevice->tx_curr_local;
	while (0 != skb_queue_len(&pEmacDevice->TxQueue)) {	/* loop till no sk buffers are in queue */
		/* Is the descriptor owned by us, and empty? */
		if (!(tx_desc_ptr->uInfo & ENET_TX_CTL_INFO_OWN)) {
			/* Is the buffer 0?, whether previous transmission complete? */
			if (NULL == tx_desc_ptr->puData) {
				done = 1;
				{
					puDataBuf = __skb_dequeue(&pEmacDevice->TxQueue);
					/* update data pointer(puData)field before control(uInfo)field */
					/* synchronize cached area */
					tx_desc_ptr->puData = (unsigned char *) dma_map_single(NULL, puDataBuf->data, puDataBuf->len, DMA_TO_DEVICE);
					tx_desc_ptr->uInfo = ENET_TX_CTL_INFO_CPU_FIRST | ENET_TX_CTL_INFO_CPU_LAST | ENET_TX_CTL_INFO_OWN | puDataBuf->len;

				}
				/* Keep copy so we can free the buffer after transmission complete */
				tx_local_ptr->puData = puDataBuf;
				tx_desc_ptr++;
				tx_local_ptr++;
				/* Update the BDT polling pointers to start point, if it crosses end point */
				if (tx_desc_ptr > pEmacDevice->tx_last_desc) {
					tx_desc_ptr = pEmacDevice->tx_first_desc;
					tx_local_ptr = pEmacDevice->tx_first_local;
				}
			} else {
				/* Come out if BDT is not processed by the Tx ISR process */
				break;
			}
			pEmacDevice->hNetDev->trans_start = jiffies;
		} else {
			/* No TX BD entry available, droping packet */
			pEmacDevice->Stats.uTxBDEntryNotAvailable++;
			break;	/* come out if EMAC has control of the BDT */
		}
	}
	/* Update the EMAC current ring pointer */
	if (1 == done) {
		pEmacDevice->tx_curr = tx_desc_ptr;
		pEmacDevice->tx_curr_local = tx_local_ptr;
		(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = (1UL << 31);
	}

	return retVal;
}

/* ISR routine handles Rx/Tx and Error processing */
static irqreturn_t emac_isr(int irq, void *dev_id)
{
	unsigned long uIrq;
	EMAC_DEVICE *pEmacDevice;
	irqreturn_t retVal = IRQ_HANDLED;

	pEmacDevice = (EMAC_DEVICE *) dev_id;
	uIrq = (*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) & ~(1UL << 12);

	/*Clear the interrupt */
	(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = uIrq;

	/*If Receive Interrupt has come */
	if ((1UL << 1) & uIrq) {
		/* Schedule Rx interrupt processing routine */
		tasklet_schedule(&gRxTasklet[pEmacDevice->uInterfaceNo]);
	}
	if (1 & uIrq) {
		/* Schedule Tx interrupt processing routine */
		tasklet_schedule(&gTxTasklet[pEmacDevice->uInterfaceNo]);
	}
	/* Update error counter */
	if ((1UL << 2) & uIrq) {
		if ((1UL << 3) & uIrq) {
			pEmacDevice->Stats.uTxChainingErrCnt++;
		}
		if ((1UL << 4) & uIrq) {
			pEmacDevice->Stats.uBDMissPktCnt += ERROR_OVERFLOW_COUNTER;
		}
		if ((1UL << 8) & uIrq) {
			pEmacDevice->Stats.uRxCRCErrDropCnt += ERROR_OVERFLOW_COUNTER;
		}
		if ((1UL << 9) & uIrq) {
			pEmacDevice->Stats.uRxFrameErrDropCnt += ERROR_OVERFLOW_COUNTER;
		}
		if ((1UL << 10) & uIrq) {
			pEmacDevice->Stats.uRxFifoOverflowDropCnt += ERROR_OVERFLOW_COUNTER;
		}
	}

	return retVal;
}

/* Tasklet handler for TXINT post processing . Traverse through the TX buffer 
 * descriptors. If OWN bit is 0 for a descriptor, then free its buffer memory. 
 */
static void TxInterruptPostProcessing(unsigned long data)
{
	EMAC_BUF_DESCR *tx_desc_ptr;
	EMAC_LOCAL_BUF *tx_refil_local_ptr;
	register unsigned long info;
	register unsigned char *ptr;
	unsigned long status;
	unsigned long lenBuf;
	unsigned short pktSent;

	EMAC_DEVICE *pEmacDevice = (EMAC_DEVICE *) data;

	local_irq_disable();
	status = (*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) & (~1);
	(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = status;
	local_irq_enable();

	tx_desc_ptr = pEmacDevice->tx_refil;
	tx_refil_local_ptr = pEmacDevice->tx_refil_local;

	/* Ensure info and ptr has updated value */
	info = tx_desc_ptr->uInfo;
	ptr = tx_desc_ptr->puData;
	/* Check we own the buffer, and it is valid */
	while ((!(info & ENET_TX_CTL_INFO_OWN)) && (ptr != NULL)) {
		struct sk_buff *pBuf;
		pBuf = (struct sk_buff *) (tx_refil_local_ptr->puData);
		lenBuf = pBuf->len;

#ifdef SKB_CLONE
		if ((pBuf->end - pBuf->head >= EMAC_RX_FRAMESIZE_MAX) && (skb_queue_len(&refillQueue[pEmacDevice->uInterfaceNo]) < refill_queue_len) && (!pBuf->cloned)
		    && (atomic_read(&pBuf->users) == 1)
		    && (atomic_read(&(skb_shinfo(pBuf)->dataref)) == 1)
		    && ((skb_shinfo(pBuf)->nr_frags == 0))
		    && (skb_shinfo(pBuf)->frag_list == NULL)) {
			dst_release(pBuf->dst);

#ifdef CONFIG_XFRM
			secpath_put(pBuf->sp);
#endif

			if (pBuf->destructor) {
				pBuf->destructor(pBuf);
			}
#ifdef CONFIG_NETFILTER
			nf_conntrack_put(pBuf->nfct);
#ifdef CONFIG_BRIDGE_NETFILTER
			nf_bridge_put(pBuf->nf_bridge);
#endif
#endif

			__skb_queue_tail(&refillQueue[pEmacDevice->uInterfaceNo], pBuf);
		} else
#endif
		{
			dev_kfree_skb(pBuf);	/*  delete sk buffer */
		}
		pktSent = 1;

		if (ENET_TX_CTL_INFO_CARR_LOSS == (ENET_TX_CTL_INFO_CARR_LOSS & info)) {
			pEmacDevice->Stats.uTxCarrLossCnt++;
			pktSent = 0;
		}
		if (ENET_TX_CTL_INFO_DEFER == (ENET_TX_CTL_INFO_DEFER & info)) {
			pEmacDevice->Stats.uTxDeferredCnt++;
		}
		if (ENET_TX_CTL_INFO_DROPPED == (ENET_TX_CTL_INFO_DROPPED & info)) {
			pEmacDevice->Stats.uTxDropCnt++;
			pktSent = 0;
		}
		pEmacDevice->Stats.uTxRetryCnt += ((ENET_TX_CTL_INFO_RETRY & info) >> ENET_TX_CTL_INFO_RETRY_SHIFT);
		if (ENET_TX_CTL_INFO_LATE_COLL == (ENET_TX_CTL_INFO_LATE_COLL & info)) {
			pEmacDevice->Stats.uTxLateCollisionDropCnt++;
			pktSent = 0;
		}
		if (ENET_TX_CTL_INFO_UFLO == (ENET_TX_CTL_INFO_UFLO & info)) {
			pEmacDevice->Stats.uTxUnderflowErrCnt++;
			pktSent = 0;
		}
		if (1 == pktSent) {
			pEmacDevice->Stats.uTxPktCnt++;
			if (lenBuf < (EMAC_FRAMESIZE_MIN - FRAME_CHECKSUM_SIZE)) {
				pEmacDevice->Stats.uTxByteCnt += (EMAC_FRAMESIZE_MIN - FRAME_CHECKSUM_SIZE);	/* Auto Pad included */
			} else {
				pEmacDevice->Stats.uTxByteCnt += lenBuf;
			}
		}
		/* Clear the descriptor */
		tx_desc_ptr->puData = NULL;
		tx_refil_local_ptr->puData = NULL;
		tx_desc_ptr++;
		tx_refil_local_ptr++;

		/* Update the BDT polling pointers to start point, if it crosses end point */
		if (tx_desc_ptr > pEmacDevice->tx_last_desc) {
			tx_desc_ptr = pEmacDevice->tx_first_desc;
			tx_refil_local_ptr = pEmacDevice->tx_first_local;
		}
		/* decrement sk buffers transmission pending counter */
		pEmacDevice->TxQDepth--;

		if (pEmacDevice->TxQDepth < EMAC_WAKE_QUEUE_THRESHHOLD) {
			netif_wake_queue(pEmacDevice->hNetDev);
		}

		/* Ensure info and ptr has updated value */
		info = tx_desc_ptr->uInfo;
		ptr = tx_desc_ptr->puData;
	}

	pEmacDevice->tx_refil = tx_desc_ptr;
	pEmacDevice->tx_refil_local = tx_refil_local_ptr;
	/* Fill the Tx Descriptor which are empty */
	StartTransmit(pEmacDevice);

	local_irq_disable();
	status = (*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) | 1;
	(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = status;
	local_irq_enable();

}


/* Tasklet handler for RXINT post processing. Traverse through the RX buffer 
 * descriptors.  If OWN bit is 0 and LAST bit is 1 in INFO field of the buffer 
 * descriptor, then call netif_rx(Buf) to pass the buffer to higher protocol 
 * layer.
 */
static void RxInterruptPostProcessing(unsigned long data)
{
	EMAC_BUF_DESCR *rx_desc_ptr;
	EMAC_LOCAL_BUF *rx_local_ptr;
	register unsigned long info;
	register unsigned char *ptr;
	unsigned long status;
	struct sk_buff *pBuf, *rpBuf;
	unsigned long lenBuf = 0;
	EMAC_DEVICE *pEmacDevice = (EMAC_DEVICE *) data;
	lenBuf = EMAC_FRAMESIZE_MAX;
	local_irq_disable();
	status = (*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) & (~(1UL << 1));
	(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = status;
	local_irq_enable();

	rx_desc_ptr = pEmacDevice->rx_curr;
	rx_local_ptr = pEmacDevice->rx_curr_local;

	/* Ensure info and ptr has updated value */
	info = rx_desc_ptr->uInfo;
	ptr = rx_desc_ptr->puData;
	/* Check we own the buffer */
	while (!(info & ENET_RX_CTL_INFO_OWN)) {
		pBuf = (struct sk_buff *) (rx_local_ptr->puData);
		rpBuf = dev_alloc_skb(lenBuf);
		if (rpBuf != NULL) {
			/* Confirm packet is stored in single BDT's */
			if ((info & ENET_RX_CTL_INFO_FIRST) && (info & ENET_RX_CTL_INFO_LAST)) {
				unsigned long len;
				len = info & ENET_RX_CTL_INFO_CPU_RX_LEN;
				/* Valid Ethernet Packet */
				if ((len >= EMAC_FRAMESIZE_MIN) && (len <= EMAC_FRAMESIZE_MAX)) {
					skb_reserve(rpBuf, SKB_PREFIX_LEN);
					rx_desc_ptr->puData = (unsigned char *) dma_map_single(NULL, rpBuf->tail, EMAC_RX_FRAMESIZE_MAX, DMA_FROM_DEVICE);
					rx_local_ptr->puData = rpBuf;
					pEmacDevice->Stats.uRxPktCnt++;
					pEmacDevice->Stats.uRxByteCnt += (len - FRAME_CHECKSUM_SIZE);	/* Remove FCS */
					skb_put(pBuf, len);
					pBuf->dev = pEmacDevice->hNetDev;
					pBuf->dev->last_rx = jiffies;
					/* find the frame protocol */
					pBuf->protocol = eth_type_trans(pBuf, pBuf->dev);
					if (pBuf->pkt_type == PACKET_MULTICAST) {
						pEmacDevice->Stats.uRxMulticastPktCnt++;
					}
					pBuf->ip_summed = CHECKSUM_NONE;
					/* Pass packet to upper layer */
					netif_rx(pBuf);
				} else {
					if ((len < EMAC_FRAMESIZE_MIN)) {
						pEmacDevice->Stats.uRxShortPktErrCnt++;
						dev_kfree_skb(rpBuf);	/*  delete sk buffer */
#ifdef PECOS_ETH_TESTPRINT
						EMACDBG("Rx Short PKT ignored \n");
#endif
					} else {
						pEmacDevice->Stats.uRxLongPktErrCnt++;
						dev_kfree_skb(rpBuf);	/*  delete sk buffer */
#ifdef PECOS_ETH_TESTPRINT
						EMACDBG("Rx Long PKT ignored \n");
#endif
					}
				}
			} else {
				pEmacDevice->Stats.uRxChainErrCnt++;
				dev_kfree_skb(rpBuf);	/*  delete sk buffer */
#ifdef PECOS_ETH_TESTPRINT
				EMACDBG("Receive Interrupt but not the last bit set PKT ignored\n");
#endif

			}
		} else {
			pEmacDevice->Stats.uRxBuffDropCnt++;
#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("No SKBUFF PKT ignored\n");
#endif
		}
		/* Update the BDT to receive packet */
		rx_desc_ptr->uInfo = EMAC_RX_FRAMESIZE_MAX | ENET_RX_CTL_INFO_OWN;

		/* Update descriptor */
		rx_desc_ptr++;
		rx_local_ptr++;
		/* Update the BDT polling pointers to start point, if it crosses end point */
		if (rx_desc_ptr > pEmacDevice->rx_last_desc) {
			rx_desc_ptr = pEmacDevice->rx_first_desc;
			rx_local_ptr = pEmacDevice->rx_first_local;
		}
		info = rx_desc_ptr->uInfo;
		ptr = rx_desc_ptr->puData;
	}

	pEmacDevice->rx_curr = rx_desc_ptr;
	pEmacDevice->rx_curr_local = rx_local_ptr;

	local_irq_disable();
	status = (*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) | (1UL << 1);
	(*((volatile unsigned long *) EMAC_ENABLE_REG(pEmacDevice->uInterfaceNo))) = status;
	local_irq_enable();

}

#if 0
/* Allocate buffer for the Rx buffer descriptors for whom the buffer ptr is 
 * marked as NULL
 */
static void RxBufferRefillHandler(EMAC_DEVICE * pEmacDevice)
{
	EMAC_BUF_DESCR *rx_refil_ptr;
	EMAC_LOCAL_BUF *rx_refil_local_ptr;
	unsigned char *ptr;
	unsigned char *puData;
	unsigned long lenBuf = 0;
	struct sk_buff *pBuf = NULL;

#ifdef SKB_CLONE
	unsigned long trueSize = 0;
#endif

	lenBuf = EMAC_FRAMESIZE_MAX;
	rx_refil_ptr = pEmacDevice->rx_refil;
	rx_refil_local_ptr = pEmacDevice->rx_refil_local;
	ptr = rx_refil_ptr->puData;
	/*refillTime_stamp[pEmacDevice->PhysicalPort]=jiffies; */
	while (ptr == NULL) {
#ifdef SKB_CLONE
		pBuf = 0;
		if (refillQueue[pEmacDevice->uInterfaceNo].qlen != 0) {
			pBuf = __skb_dequeue(&refillQueue[pEmacDevice->uInterfaceNo]);
		}
		if (pBuf) {
			trueSize = pBuf->truesize;
			memset(pBuf, 0, offsetof(struct sk_buff, truesize));
			pBuf->truesize = trueSize;
			pBuf->data = pBuf->head;
			pBuf->tail = pBuf->head;
			/* pBuf->end = pBuf->head + EMAC_FRAMESIZE_MAX; */
			atomic_set(&(pBuf->users), 1);
			atomic_set(&(skb_shinfo(pBuf)->dataref), 1);
			skb_shinfo(pBuf)->nr_frags = 0;
			skb_shinfo(pBuf)->tso_size = 0;
			skb_shinfo(pBuf)->tso_segs = 0;
			skb_shinfo(pBuf)->frag_list = 0;
		}
#if 0
		else if (refill_queue_len < pEmacDevice->uRxDescCount) {
			refill_queue_len = pEmacDevice->uMaxTxQDepth;
		}
#endif

#endif

#ifdef SKB_CLONE
		if (!pBuf) {
#endif
			pBuf = dev_alloc_skb(lenBuf);

#ifdef SKB_CLONE
		}
#endif

		skb_reserve(pBuf, SKB_PREFIX_LEN);
		puData = pBuf->data;

		/* Fill in the descriptor, and give to mac */
		if (pBuf) {
			rx_refil_local_ptr->puData = pBuf;
			/* The compiler would turn the write of info and ptr into
			 * a stm, but if it did not (eg optimization was not possible),
			 * the driver might fail as ptr must be updated at the same
			 * time as info
			 * So we manually stm it
			 */
			{
				rx_refil_ptr->puData = (unsigned char *) dma_map_single(NULL, pBuf->tail, EMAC_RX_FRAMESIZE_MAX, DMA_FROM_DEVICE);
				/* To prevent Rx buffer chaining the value should be greater than max frame size expected */
				rx_refil_ptr->uInfo = EMAC_RX_FRAMESIZE_MAX | ENET_RX_CTL_INFO_OWN;
			}

			rx_refil_ptr++;
			rx_refil_local_ptr++;
			if (rx_refil_ptr > pEmacDevice->rx_last_desc) {
				rx_refil_ptr = pEmacDevice->rx_first_desc;
				rx_refil_local_ptr = pEmacDevice->rx_first_local;
			}
			ptr = rx_refil_ptr->puData;
		} else {
			break;
		}
	}

	pEmacDevice->rx_refil = rx_refil_ptr;
	pEmacDevice->rx_refil_local = rx_refil_local_ptr;

}
#endif

/* Enables autonegotiation and sets the advertise parameter as per set values */
void emac_media_autonegotiate(struct net_device *dev)
{
	unsigned long uData;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	/* Get current MII_ADVERTISE value */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_ADV_REG);
	/* Discard old speed and duplex settings */
	uData &= ~(ADVERTISE_100HALF | ADVERTISE_100FULL | ADVERTISE_10HALF | ADVERTISE_10FULL);

	switch (pEmacDevice->uCurrent_speed_select) {
		/* Advertising Speed set for 10 Mbps,check mode of operation as below */
	case SPEED_10MBPS:
		if (pEmacDevice->uCurrent_duplex_select == FULL_DUPLEX) {
			uData |= ADVERTISE_10FULL;
		} else if (pEmacDevice->uCurrent_duplex_select == HALF_DUPLEX) {
			uData |= ADVERTISE_10HALF;
		} else {
			uData |= ADVERTISE_10HALF | ADVERTISE_10FULL;
		}

		break;
		/* Advertising Speed set for 100 Mbps,check mode of operation as below */
	case SPEED_100MBPS:	/* 100 Mbps */
		if (pEmacDevice->uCurrent_duplex_select == FULL_DUPLEX) {
			uData |= ADVERTISE_100FULL;
		} else if (pEmacDevice->uCurrent_duplex_select == HALF_DUPLEX) {
			uData |= ADVERTISE_100HALF;
		} else {
			uData |= ADVERTISE_100HALF | ADVERTISE_100FULL;
		}

		break;
		/* Advertising Speed set for autoneg default,check mode of operation as below */
	case AUTO_SPEED:	/* Auto */
		if (pEmacDevice->uCurrent_duplex_select == FULL_DUPLEX) {
			uData |= ADVERTISE_100FULL | ADVERTISE_10FULL;
		} else if (pEmacDevice->uCurrent_duplex_select == HALF_DUPLEX) {
			uData |= ADVERTISE_100HALF | ADVERTISE_10HALF;
		} else {
			uData |= ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL;
		}

		break;

	default:		/* assume autoneg default speed and duplex */
		uData |= ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL;
	}

	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_AUTONEG_ADV_REG, uData);
	/* Renegotiate with link partner */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	uData |= BMCR_ANENABLE | BMCR_ANRESTART;
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, uData);
	pEmacDevice->mii_if.force_media = 0;
}

/* Disables autonegotiation and forces the speed/duplex */
void emac_media_force(struct net_device *dev, EMAC_SPEED uSpeed, EMAC_DUPLEX uDuplex)
{
	EMAC_DEVICE *pEmacDevice;
	int uData;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	/* Disable autonegotiation and set default speed 10Mbps and Half duplex */
	uData &= ~(PHY_CTRL_AUTONEG + PHY_CTRL_SPEED + PHY_CTRL_DUPLEX);
	pEmacDevice->uSpeed = SPEED_10MBPS;
	pEmacDevice->uMode = HALF_DUPLEX;
	if (uSpeed == SPEED_100MBPS) {
		pEmacDevice->uSpeed = SPEED_100MBPS;
		uData |= PHY_CTRL_SPEED;	/* 100Mbps */
	}
	if (uDuplex == FULL_DUPLEX) {
		pEmacDevice->uMode = FULL_DUPLEX;
		uData |= PHY_CTRL_DUPLEX;	/* Full duplex */
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, uData);
	/* Duplex changed, Update EMAC */
	uData = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (FULL_DUPLEX == pEmacDevice->uMode) {
		uData |= (1UL << 10);
	} else {
		uData &= (~(1UL << 10));
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = uData;
	pEmacDevice->mii_if.force_media = 1;
}

#ifdef EMAC_MEDIA_MAINTANANCE
/* Thread to update Media condition */
int emac_media_check(void *dev)
{
	int devNo;
	unsigned int uTmpVal;
	EMAC_DUPLEX old_duplex;
	EMAC_DEVICE *pEmacDevice;

	while (1) {

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

			pEmacDevice = (EMAC_DEVICE *) netdev_priv(gNetDevice[devNo]);
			old_duplex = pEmacDevice->uMode;
			if (pEmacDevice->uMDIO_support) {
				/* Check media changes */
				pEmacDevice->media_check_func(gNetDevice[devNo]);
				if (old_duplex != pEmacDevice->uMode) {
					/* Duplex changed, Update EMAC */
					uTmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
					if (FULL_DUPLEX == pEmacDevice->uMode) {
						uTmpVal |= (1UL << 10);
					} else {
						uTmpVal &= (~(1UL << 10));
					}
					(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = uTmpVal;
				}
				pEmacDevice->mii_if.full_duplex = pEmacDevice->uMode;
			}

		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(1 * HZ);
		if (gThreadExitReq) {
			break;
		}
	}
	complete_and_exit(&gThreadExitComplete, 0);
	return 0;
}
#endif

#ifdef EMAC_MEDIA_MAINTANANCE

/* Function which check media speed and mode for realtek8201 */
static void realtek8201_media_check(struct net_device *dev)
{
	unsigned long uData, old_carrier, new_carrier;
	EMAC_DEVICE *pEmacDevice;
	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	/* Check current link speed / mode */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, REALTEK8201_MEDIACHEK_REG);
	/* Realtek8201 Speed check bit */
	if (uData & REALTEK8201_PHY_SPEED) {
		pEmacDevice->uSpeed = SPEED_100MBPS;
	} else {
		pEmacDevice->uSpeed = SPEED_10MBPS;
	}
	/* Realtek8201 Mode check bit */
	if (uData & REALTEK8201_PHY_MODE) {
		pEmacDevice->uMode = FULL_DUPLEX;
	} else {
		pEmacDevice->uMode = HALF_DUPLEX;
	}
	/* previous link status */
	old_carrier = netif_carrier_ok(pEmacDevice->mii_if.dev) ? 1 : 0;
	/* get link status from PHY */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	/* Current link status */
	new_carrier = (uData & PHY_STATUS_LINK_STATUS) ? 1 : 0;

	/* Changes in Link status */
	if (old_carrier != new_carrier) {
		/* Link is Up */
		if (new_carrier) {
			netif_carrier_on(pEmacDevice->mii_if.dev);
//#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("RealTek 8201 PHY %s: Link Up %d Mbps %s \n", dev->name, ((pEmacDevice->uSpeed == SPEED_100MBPS) ? 100 : 10), ((pEmacDevice->uMode == FULL_DUPLEX) ? "Full Duplex" : "Half Duplex"));
//#endif
		} else {
			netif_carrier_off(pEmacDevice->mii_if.dev);
//#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("RealTek 8201 PHY %s: Link down\n", dev->name);
//#endif
		}
	}

	return;
}

/* IP101 is compatible with the 8201 */
static void ip101_media_check(struct net_device *dev)
{
	realtek8201_media_check(dev);
}

/* Function which check media speed and mode for Lxt972 */
static void lxt972_media_check(struct net_device *dev)
{

	unsigned long uData, old_carrier, new_carrier;
	EMAC_DEVICE *pEmacDevice;
	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	/* Check current link speed / mode */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, LXT972_MEDIACHEK_REG);

	/* Lxt972 Speed check bit */
	if (uData & LXT972_PHY_SPEED) {
		pEmacDevice->uSpeed = SPEED_100MBPS;
	} else {
		pEmacDevice->uSpeed = SPEED_10MBPS;
	}
	/* Lxt972 Mode check bit */
	if (uData & LXT972_PHY_MODE) {
		pEmacDevice->uMode = FULL_DUPLEX;
	} else {
		pEmacDevice->uMode = HALF_DUPLEX;
	}

	/* previous link status */
	old_carrier = netif_carrier_ok(pEmacDevice->mii_if.dev) ? 1 : 0;
	/* get link status from PHY */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	/* Current link status */
	new_carrier = (uData & PHY_STATUS_LINK_STATUS) ? 1 : 0;
	/* Changes in Link status */
	if (old_carrier != new_carrier) {
		/* Link is Up */
		if (new_carrier) {
			netif_carrier_on(pEmacDevice->mii_if.dev);
#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("LXT 972 PHY %s: Link Up %d Mbps %s \n", dev->name, ((pEmacDevice->uSpeed == SPEED_100MBPS) ? 100 : 10), ((pEmacDevice->uMode == FULL_DUPLEX) ? "Full Duplex" : "Half Duplex"));
#endif
		} else {
			netif_carrier_off(pEmacDevice->mii_if.dev);
#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("LXT 972 PHY %s: Link down\n", dev->name);
#endif
		}
	}

	return;
}

/* Function which check link condition is for general devices,
 where speed/mode is fixed */
static void general_media_check(struct net_device *dev)
{
	unsigned long uData, old_carrier, new_carrier;
	EMAC_DEVICE *pEmacDevice;
	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	/* previous link status */
	old_carrier = netif_carrier_ok(pEmacDevice->mii_if.dev) ? 1 : 0;
	/* get link status from PHY */
	uData = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	/* Current link status */
	new_carrier = (uData & PHY_STATUS_LINK_STATUS) ? 1 : 0;
	/* Changes in Link status */
	if (old_carrier != new_carrier) {
		/* Link is Up */
		if (new_carrier) {
			netif_carrier_on(pEmacDevice->mii_if.dev);
#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("General PHY %s: Link Up %d Mbps %s \n", dev->name, ((pEmacDevice->uSpeed == SPEED_100MBPS) ? 100 : 10), ((pEmacDevice->uMode == FULL_DUPLEX) ? "Full Duplex" : "Half Duplex"));
#endif
		} else {
			netif_carrier_off(pEmacDevice->mii_if.dev);
#ifdef PECOS_ETH_TESTPRINT
			EMACDBG("General PHY %s: Link down\n", dev->name);
#endif
		}
	}

	return;
}
#endif

int EmacReadReg_PHY(struct net_device *dev, int PhyAddr, int RegNo)
{
	unsigned long readData;
	unsigned long mdio_reg;
	unsigned long count;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	mdio_reg = (1UL << 30) | (2UL << 28) | (1UL << 17);
	mdio_reg |= ((PhyAddr << 23) & 0x0F800000);
	mdio_reg |= ((RegNo << 18) & 0x007C0000);

	/* Lock the hardware */
	down(&pEmacDevice->mutex);
	/* Start read */
	(*((volatile unsigned long *) EMAC_MDIO_REG(pEmacDevice->uInterfaceNo))) = mdio_reg;
	/* Wait for completion */
	count = 0;
	while (!((*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) & (1UL << 12))) {
		count++;
		udelay(EMAC_DELAY);
		if (count > EMAC_LOCKUP_WAIT) {
			break;
		}
	}
	/* Read data */
	mdio_reg = (*((volatile unsigned long *) EMAC_MDIO_REG(pEmacDevice->uInterfaceNo)));
	/* Ack irq */
	(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = (1UL << 12);
	/* unlock */
	up(&pEmacDevice->mutex);
	readData = mdio_reg & 0x0000FFFF;

	return readData;
}

void EmacWriteReg_PHY(struct net_device *dev, int PhyAddr, int RegNo, int Data)
{
	unsigned long mdio_reg;
	unsigned long count;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	mdio_reg = (1UL << 30) | (1UL << 28) | (1UL << 17);
	mdio_reg |= ((PhyAddr << 23) & 0x0F800000);
	mdio_reg |= ((RegNo << 18) & 0x007C0000);
	mdio_reg |= (Data & 0x0000FFFF);

	/* Lock the hardware */
	down(&pEmacDevice->mutex);
	/* Start read */
	(*((volatile unsigned long *) EMAC_MDIO_REG(pEmacDevice->uInterfaceNo))) = mdio_reg;
	/* Wait for completion */
	count = 0;
	while (!((*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) & (1UL << 12))) {
		count++;
		udelay(EMAC_DELAY);
		if (count > EMAC_LOCKUP_WAIT) {
			break;
		}
	}

	/* Ack irq */
	(*((volatile unsigned long *) EMAC_STAT_REG(pEmacDevice->uInterfaceNo))) = (1UL << 12);
	/* unlock */
	up(&pEmacDevice->mutex);
}

/* EMAC CONTROL/STATUS FUNCTIONS */
int emac_set_activity(struct net_device *dev, unsigned short uEnable)
{
	int retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 0;
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (1 == uEnable) {
		tmpVal |= 1;
	} else {
		tmpVal &= (~1);
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return retVal;
}

int emac_set_transmit_activity(struct net_device *dev, unsigned short uEnable)
{
	int retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 0;
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (1 == uEnable) {
		tmpVal |= (1UL << 3);
	} else {
		tmpVal &= (~(1UL << 3));
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return retVal;
}

int emac_set_receive_activity(struct net_device *dev, unsigned short uEnable)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (1 == uEnable) {
		tmpVal |= (1UL << 4);
	} else {
		tmpVal &= (~(1UL << 4));
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return 0;
}

int emac_set_broadcast(struct net_device *dev, unsigned short uEnable)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (1 == uEnable) {
		tmpVal &= (~(1UL << 8));
	} else {
		tmpVal |= (1UL << 8);
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return 0;
}

int emac_set_promiscuous(struct net_device *dev, unsigned short uEnable)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	if (1 == uEnable) {
		tmpVal |= (1UL << 11);
	} else {
		tmpVal &= (~(1UL << 11));
	}
	(*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo))) = tmpVal;

	return 0;
}

int emac_set_bdt_poolrate(struct net_device *dev, unsigned long uRate)
{
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	(*((volatile unsigned long *) EMAC_POLLRATE_REG(pEmacDevice->uInterfaceNo))) = uRate;

	return 0;
}

int emac_get_bdt_poolrate(struct net_device *dev, unsigned long *puRate)
{
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	*puRate = (*((volatile unsigned long *) EMAC_POLLRATE_REG(pEmacDevice->uInterfaceNo)));

	return 0;
}

unsigned long emac_get_transmit_bdt_length(struct net_device *dev)
{
	int retVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;

	retVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	retVal &= 0x00FF0000;
	retVal >>= 16;

	return retVal;
}

unsigned long emac_get_receive_bdt_length(struct net_device *dev)
{
	int retVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	retVal = 0;
	retVal = (*((volatile unsigned long *) EMAC_CONTROL_REG(pEmacDevice->uInterfaceNo)));
	retVal &= 0xFF000000;
	retVal >>= 24;

	return retVal;
}

/* PHY DETECT FUNCTION */
int phy_detect(struct net_device *dev)
{

	unsigned int count;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	/* Probe MDIO physical address */
	for (count = 0; count < PHY_ADDR_MAX; count++) {
		if (EmacReadReg_PHY(dev, count, PHY_STATUS_REG) != 0xffff) {
			break;
		}
	}

	if (count == PHY_ADDR_MAX) {
		return -1;	/*ENODEV */
	}

	/* Asign MDIO address */
	pEmacDevice->mii_if.phy_id = count;

	return 0;
}

#ifdef EMAC_MEDIA_MAINTANANCE
void select_media_check_function(struct net_device *dev)
{
	unsigned int phyid_high;
	unsigned int phyid_low;
	unsigned int phyid;
	struct phy_media_check *phy = NULL;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);

	/* Get Phy ID */
	phyid_high = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG1);
	phyid_low = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_IDENTIFIER_REG2);
	phyid = (phyid_high << 16) | phyid_low;	/* Get PHY ID */

	for (phy = &phy_list[0]; phy->phy_id; phy++) {
		if (phy->phy_id == phyid) {	/* Is read phyid matches with phy_id in function table */
			/* Phy media check function */
			pEmacDevice->media_check_func = phy->media_check_func;
			return;
		}
	}

	pEmacDevice->media_check_func = phy->media_check_func;
	return;

}
#endif
/* PHY CONTROLLING / STATUS FUNCTIONS */

void phy_reset(struct net_device *dev)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	tmpVal |= PHY_CTRL_RESET;
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}

/* Depending on the EMAC setting for Duplex mode set the PHY */
void phy_set_mode(struct net_device *dev, EMAC_DUPLEX uMode)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (FULL_DUPLEX == uMode) {
		tmpVal |= PHY_CTRL_DUPLEX;
	} else {
		tmpVal &= (~PHY_CTRL_DUPLEX);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
	pEmacDevice->uMode = uMode;
}


void phy_set_speed(struct net_device *dev, EMAC_SPEED uSpeed)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (SPEED_100MBPS == uSpeed) {
		tmpVal |= PHY_CTRL_SPEED;
	} else {
		tmpVal &= (~PHY_CTRL_SPEED);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
	pEmacDevice->uSpeed = uSpeed;
}


/* Depending on the EMAC setting for Duplex mode set the PHY */
void phy_loopback(struct net_device *dev, unsigned short uMode)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (1 == uMode) {
		tmpVal |= PHY_CTRL_LOOPBACK;
	} else {
		tmpVal &= (~PHY_CTRL_LOOPBACK);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}

/* Set the PHY into powerdwn mode */
void phy_powerdown(struct net_device *dev, unsigned short uMode)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (1 == uMode) {
		tmpVal |= PHY_CTRL_POWERDOWN;
	} else {
		tmpVal &= (~PHY_CTRL_POWERDOWN);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}

/* Isolate PHY */
void phy_isolate(struct net_device *dev, unsigned short uMode)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (1 == uMode) {
		tmpVal |= PHY_CTRL_ISOLATE;
	} else {
		tmpVal &= (~PHY_CTRL_ISOLATE);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}


/* Enable collosion test */
void phy_collisiontest(struct net_device *dev, unsigned short uMode)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (1 == uMode) {
		tmpVal |= PHY_CTRL_COLL;
	} else {
		tmpVal &= (~PHY_CTRL_COLL);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}

void phy_set_autonegotiation(struct net_device *dev, unsigned short uValue)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	if (1 == uValue) {
		tmpVal |= PHY_CTRL_AUTONEG;
	} else {
		tmpVal &= (~PHY_CTRL_AUTONEG);
	}
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}

void phy_restart_autonegotiation(struct net_device *dev)
{
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG);
	tmpVal |= PHY_CTRL_RESTART_AUTO;
	EmacWriteReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_CTRL_REG, tmpVal);
}


unsigned short phy_is_autonegotiation_completed(struct net_device *dev)
{
	unsigned short retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;

	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 1;
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	tmpVal &= PHY_STATUS_AUTONEGOTIATION_COMPLETE;
	if (PHY_STATUS_AUTONEGOTIATION_COMPLETE == tmpVal) {
		retVal = 0;
	}

	return retVal;
}

unsigned short phy_autonegotiation_supported(struct net_device *dev)
{
	unsigned short retVal;
	unsigned long tmpVal;
	EMAC_DEVICE *pEmacDevice;


	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	retVal = 0;
	tmpVal = EmacReadReg_PHY(dev, pEmacDevice->mii_if.phy_id, PHY_STATUS_REG);
	tmpVal &= PHY_STATUS_AUTONEGOTIATION_ABILITY;
	if (PHY_STATUS_AUTONEGOTIATION_ABILITY == tmpVal) {
		retVal = 1;
	}

	return retVal;
}

#ifdef EMAC_DRIVER_STATISTICS
void emac_stats_clear(struct net_device *dev)
{
	EMAC_DEVICE *pEmacDevice;
	EMAC_STATS emacStats;
	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	emac_get_device_statistics(dev, &emacStats);

	/*Set all member in Status to 0 */
	memset(&pEmacDevice->Stats, 0, sizeof(EMAC_STATS));
	EMACDBG("Tx Packet Count %lu\n", pEmacDevice->Stats.uTxPktCnt);
	EMACDBG("Tx Byte Count %lu\n", pEmacDevice->Stats.uTxByteCnt);
	EMACDBG("Rx Packet Count %lu\n", pEmacDevice->Stats.uRxPktCnt);
	EMACDBG("Rx Byte Count %lu\n", pEmacDevice->Stats.uRxByteCnt);
	EMACDBG("Rx Multicast Pkts %lu\n", pEmacDevice->Stats.uRxMulticastPktCnt);
	EMACDBG("Tx Carrier Lost Count %lu\n", pEmacDevice->Stats.uTxCarrLossCnt);
	EMACDBG("Tx Deferred Count %lu\n", pEmacDevice->Stats.uTxDeferredCnt);
	EMACDBG("Tx Retry Count %lu\n", pEmacDevice->Stats.uTxRetryCnt);
	EMACDBG("Tx drop Count %lu\n", pEmacDevice->Stats.uTxDropCnt);
	EMACDBG("Tx Latecollision Count %lu\n", pEmacDevice->Stats.uTxLateCollisionDropCnt);

	EMACDBG("Tx Underflow error Count %lu\n", pEmacDevice->Stats.uTxUnderflowErrCnt);
	EMACDBG("Tx Chaining error Count %lu\n", pEmacDevice->Stats.uTxChainingErrCnt);
	EMACDBG("Tx BDT not available %lu\n", pEmacDevice->Stats.uTxBDEntryNotAvailable);

	EMACDBG("Rx CRC ERR %lu\n", pEmacDevice->Stats.uRxCRCErrDropCnt);
	EMACDBG("Rx Frame ERR %lu\n", pEmacDevice->Stats.uRxFrameErrDropCnt);
	EMACDBG("Rx FIFO ERR %lu\n", pEmacDevice->Stats.uRxFifoOverflowDropCnt);
	EMACDBG("Rx BDT MISS ERR %lu\n", pEmacDevice->Stats.uBDMissPktCnt);
	EMACDBG("Rx SHORT PKT %lu\n", pEmacDevice->Stats.uRxShortPktErrCnt);
	EMACDBG("Rx LONG PKT %lu\n", pEmacDevice->Stats.uRxLongPktErrCnt);
	EMACDBG("Rx Chain ERR %lu\n", pEmacDevice->Stats.uRxChainErrCnt);
	EMACDBG("Rx Drop No SKBUFF %lu\n", pEmacDevice->Stats.uRxBuffDropCnt);
}

void emac_stats_display(struct net_device *dev)
{
	EMAC_DEVICE *pEmacDevice;
	EMAC_STATS emacStats;
	pEmacDevice = (EMAC_DEVICE *) netdev_priv(dev);
	emac_get_device_statistics(dev, &emacStats);
	EMACDBG("Tx Packet Count %lu\n", pEmacDevice->Stats.uTxPktCnt);
	EMACDBG("Tx Byte Count %lu\n", pEmacDevice->Stats.uTxByteCnt);
	EMACDBG("Rx Packet Count %lu\n", pEmacDevice->Stats.uRxPktCnt);
	EMACDBG("Rx Byte Count %lu\n", pEmacDevice->Stats.uRxByteCnt);
	EMACDBG("Rx Multicast Pkts %lu\n", pEmacDevice->Stats.uRxMulticastPktCnt);
	EMACDBG("Tx Carrier Lost Count %lu\n", pEmacDevice->Stats.uTxCarrLossCnt);
	EMACDBG("Tx Deferred Count %lu\n", pEmacDevice->Stats.uTxDeferredCnt);
	EMACDBG("Tx Retry Count %lu\n", pEmacDevice->Stats.uTxRetryCnt);
	EMACDBG("Tx drop Count %lu\n", pEmacDevice->Stats.uTxDropCnt);
	EMACDBG("Tx Latecollision Count %lu\n", pEmacDevice->Stats.uTxLateCollisionDropCnt);

	EMACDBG("Tx Underflow error Count %lu\n", pEmacDevice->Stats.uTxUnderflowErrCnt);
	EMACDBG("Tx Chaining error Count %lu\n", pEmacDevice->Stats.uTxChainingErrCnt);
	EMACDBG("Tx BDT not available %lu\n", pEmacDevice->Stats.uTxBDEntryNotAvailable);
	EMACDBG("Rx CRC ERR %lu\n", pEmacDevice->Stats.uRxCRCErrDropCnt);
	EMACDBG("Rx Frame ERR %lu\n", pEmacDevice->Stats.uRxFrameErrDropCnt);
	EMACDBG("Rx FIFO ERR %lu\n", pEmacDevice->Stats.uRxFifoOverflowDropCnt);
	EMACDBG("Rx BDT MISS ERR %lu\n", pEmacDevice->Stats.uBDMissPktCnt);
	EMACDBG("Rx SHORT PKT %lu\n", pEmacDevice->Stats.uRxShortPktErrCnt);
	EMACDBG("Rx LONG PKT %lu\n", pEmacDevice->Stats.uRxLongPktErrCnt);
	EMACDBG("Rx Chain ERR %lu\n", pEmacDevice->Stats.uRxChainErrCnt);
	EMACDBG("Rx Drop No SKBUFF %lu\n", pEmacDevice->Stats.uRxBuffDropCnt);
}

#endif

/****************************************************************************
 * Modifications:
 * $Log:
 *  7    Linux_SDK 1.6         8/9/07 7:51:14 PM IST  Upakul Barkakaty
 *       Implement network stop and wake routine to avoid continuous polling
 *       by the networking system under Hardware BDTs unavailable condition.
 *  6    Linux_SDK 1.5         8/3/07 2:49:56 PM IST  Upakul Barkakaty
 *       Configurable flags and logic added to enable the Second Ethernet
 *       interface with 2xMode operation for Emac.
 *  5    Linux_SDK 1.4         4/16/07 1:25:55 PM IST Rajesha Kini    Updated
 *       with GPL header.
 *  4    Linux_SDK 1.3         4/10/07 8:52:17 PM IST Rajesha Kini    Updated
 *       emac driver to get MAC address from EEPROM for the first
 *       interface(eth0).
 *  3    Linux_SDK 1.2         3/20/07 4:14:26 PM IST Har Yash Bahadur For NFS
 *  2    Linux_SDK 1.1         3/15/07 11:58:45 AM ISTHar Yash Bahadur Apply
 *       memory alignment
 *  1    Linux_SDK 1.0         3/2/07 3:42:24 PM IST  Vineet Seth     
 * $
 *
 ****************************************************************************/
