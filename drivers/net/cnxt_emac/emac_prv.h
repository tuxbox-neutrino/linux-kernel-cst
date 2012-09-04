/****************************************************************************
 *
 *  drivers/net/cnxt_emac/emac_prv.h
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
 *$Id: emac_prv.h,v 1.3, 2007-08-09 14:21:38Z, Upakul Barkakaty$
 ****************************************************************************/

#ifndef __emac_prv_h
#define __emac_prv_h

#include <linux/mii.h>		/* MII structure */
#include <linux/sockios.h>	/* SIOCDEVPRIVATE */

		       /* #define SKB_CLONE *//* sk buffer reuse logic enable */
/* maximum emac supported */
#define MAX_EMAC 2

/* ToDo: below feature, need to be provided as configuration element */
/*Total number of descriptos on the TX Descriptor Table in EMAC0 */
#define EMAC0_TX_DESCRIPTOR_CNT         32
/*Total number of descriptos on the RX Descriptor Table in EMAC0 */
#define EMAC0_RX_DESCRIPTOR_CNT         32
/*Total number of descriptos on the TX Descriptor Table in EMAC1 */
#define EMAC1_TX_DESCRIPTOR_CNT         32
/*Total number of descriptos on the RX Descriptor Table in EMAC1 */
#define EMAC1_RX_DESCRIPTOR_CNT         32
/* Default speed/mode etc  */
#define EMAC0_DEFAULT_MODE              FULL_DUPLEX	/*Full Duplex */
#define EMAC0_DEFAULT_SPEED             SPEED_100MBPS	/*100 Mbps */
#define EMAC1_DEFAULT_MODE              FULL_DUPLEX	/*Full Duplex */
#define EMAC1_DEFAULT_SPEED             SPEED_100MBPS	/*100 Mbps */
#define EMAC0_DEFAULT_MULTICAST         0	/*Multicast Enable */
#define EMAC0_DEFAULT_PROMISCOUS        0	/* Promiscous enable */
#define EMAC1_DEFAULT_MULTICAST         0	/*Multicast Enable */
#define EMAC1_DEFAULT_PROMISCOUS        0	/* Promiscous enable */
/* Tx FIFO threshold -start sending after 64 bytes are accumulated in FIFO, 
 * note that fifo depth is 512 bytes 
 */
#define EMAC0_TX_FIFO_THRESHOLD 0xFFFFFC40
#define EMAC1_TX_FIFO_THRESHOLD 0xFFFFFC40


#define MAC_ADDR_LEN              6
#define FRAME_CHECKSUM_SIZE       4
#define MULTICAST_BIT_SET         0x01
/* Transmit sk_buf queue length  */
#define MAX_TRANSMIT_QUEUE_LEN    256

#ifdef SKB_CLONE		/* sk buffer reuse logic enable */
#define MIN_REFILL_QUEUE_LEN      100
#endif
#define EMAC_FRAMESIZE_MIN        64
#define EMAC_FRAMESIZE_MAX        1518	/* Maximum size of the ethernet frame */
#define EMAC_RX_FRAMESIZE_MAX     1520	/* Keep the size more than maximum frame size */
#define EMAC_POLLRATE_VALUE       1
/* Counter value results in overflow interrupt */
#define ERROR_OVERFLOW_COUNTER    0x100
#define EMAC_FRAMESIZE_NIBBLE_MAX 3072	/* Keep it to maximum value possible */
#define EMAC_DELAY                 25	/* in us */
#define EMAC_LOCKUP_WAIT          (100)	/* worst case delay 2.5ms = EMAC_LOCKUP_WAIT * EMAC_DELAY */
#define EMAC_WAKE_QUEUE_THRESHHOLD 64	/* If secondary buffer below the threshold wake the stopped network queue */
/* maximum allowed multicast filter size */
/* #define MULTICAST_FILTER_LIMIT    64 */

#define INTERRUPT_STATUS_CLEAR    0x171F
#ifdef EMAC_DRIVER_STATISTICS
#define EMAC_STATISTICS_DISPLAY  SIOCDEVPRIVATE	/* 89F0 */
#define EMAC_STATISTICS_CLEAR    SIOCDEVPRIVATE+1	/* 89F1 */
#endif

/* EMAC capability and data structure */
typedef struct {
	/* 0 for eth0, 1 for eth1 */
	unsigned short uInterfaceNo;

	/* MAC address of the interface */
	unsigned char uMACaddress[6];
	unsigned long uPHYIndentifier;

	/* 1= PHY has the ability to perform link transmission and reception using 
	 * the 100BASE-T4 signaling specification.0 = Lack the ability 
	 */
	unsigned short u100BaseT4Ability;

	/* 1= PHY has the ability to perform full-duplex link transmission and 
	 * reception using the 100BASE-X signaling specification.0 = Lack the ability 
	 */
	unsigned short u100BaseXFullDuplexAbility;

	/* 1= PHY has the ability to perform half-duplex link transmission and 
	 * reception using the 100BASE-X signaling specification.0 = Lack the ability 
	 */
	unsigned short u100BaseXHalfDuplexAbility;

	/* 1= PHY has the ability to perform full duplex link transmission and 
	 * reception while operating at 10 Mb/s.0 = Lack the ability 
	 */
	unsigned short u10MbsFullDuplexAbility;

	/* 1= PHY has the ability to perform half duplex link transmission and 
	 * reception while operating at 10 Mb/s.0 = Lack the ability 
	 */
	unsigned short u10MbsHalfDuplexAbility;

	/* 1 = PHY will accept management frames with preamble suppressed.
	 * 0 = Lack the ability 
	 */
	unsigned short uMFPreambleSuppression;

	/* Interface is on or off */
	unsigned short uLinkStatus;

	/* 0-Support base regsiter set, 1-support extended register set */
	unsigned short uExtendedPHYRegisterSet;

	/* how often BDT state machine poll the BDT */
	unsigned short uBDTPoolRate;

	/* PHY Autonegotiation Capability */
	unsigned short uAutoNegotiationCap;

	/* PHY Autonegotiation Advertisement Capability */
	unsigned short uAutoNegotiationAdvertisementCap;

	/*Additional PHY register information */
} EMAC_ATTRIBUTES;


/* EMAC Statistic data structure */
typedef struct {
	/* Total number of packets successfully transmitted by the interface. */
	unsigned long uTxPktCnt;

	/* Total number of bytes successfully transmitted by the interface. */
	unsigned long uTxByteCnt;

	/* Total number of packets received by the interface. */
	unsigned long uRxPktCnt;

	/* Total number of bytes received by the interface. */
	unsigned long uRxByteCnt;

	/* Total number of Multicast Packets received by the interface. */
	unsigned long uRxMulticastPktCnt;

	/* CARLOSS Count i.e. Total No of time Carrier Sense was lost during 
	 * transmission. 
	 */
	unsigned long uTxCarrLossCnt;

	/* Defer Count i.e. Total No of time transmission was deferred due to 
	 * traffic on the wire. 
	 */
	unsigned long uTxDeferredCnt;

	/* Total Retry count i.e. Number of times the packet was retired because it 
	 * was not successfully transmitted in the last try. 
	 */
	unsigned long uTxRetryCnt;


	/* Drop Count value i.e. No of time packet was dropped because even after 
	 * retying maximum no(16) of times packet was not transmitted successfully.
	 */
	unsigned long uTxDropCnt;

	/* Late collision error count, i.e. No of time Packet dropped due to late 
	 * collision. 
	 */
	unsigned long uTxLateCollisionDropCnt;

	/* Underflow Error Count i.e. Total no of Packet data corrupted and dropped 
	 * because data was not available in time. 
	 */
	unsigned long uTxUnderflowErrCnt;

	/* TX Chaining Error Count i.e. A bad combination of FIRST and LAST bits has
	 * been encountered. 
	 */
	unsigned long uTxChainingErrCnt;

	/* No of time packet was dropped because there was no free Tx Buffer Descriptor
	 * to accomodate this data 
	 */
	unsigned long uTxBDEntryNotAvailable;

	/* CRC Errors i.e. Total No of receive packets dropped due to CRC errors */
	unsigned long uRxCRCErrDropCnt;

	/* FRAME Errors i.e. Total Number of receive packets dropped due to framing 
	 * errors 
	 */
	unsigned long uRxFrameErrDropCnt;

	/* Overflow Errors i.e. Total Number of receive packets dropped due to FIFO 
	 * overflows 
	 */
	unsigned long uRxFifoOverflowDropCnt;

	/* Missed packet counter i.e. Total Number of packets that were dropped 
	 * because a BD was not available. 
	 */
	unsigned long uBDMissPktCnt;

	/* Short Packet Error Count i.e. Total Receive Short Packet Error (Packet 
	 * less than 64 bytes). 
	 */
	unsigned long uRxShortPktErrCnt;
	/* Larger Packet Count i.e. Total Receive Larger Packet count (Packet 
	 * greater than standard ethernet supported). 
	 */
	unsigned long uRxLongPktErrCnt;

	/* Larger Packet Count i.e. Total Receive Larger Packet count (Packet 
	 * greater than standard ethernet supported). 
	 */
	unsigned long uRxChainErrCnt;

	/* Packet droppped due non availability of network buffers */
	unsigned long uRxBuffDropCnt;


} EMAC_STATS;

/*Hardware Buffer Descriptor data structure*/
typedef struct {
	unsigned long uInfo;	/* Info field of the Buffer Descriptor */
	unsigned char *puData;	/* Pointer (to the data) field of the Buffer Descriptor */

	/* dma_addr_t puData; */
} EMAC_BUF_DESCR;

/* Buffer contains Hardware buffer descriptor's Pointer field */
typedef struct {
	struct sk_buff *puData;
} EMAC_LOCAL_BUF;

typedef enum {
	HALF_DUPLEX = 0,
	FULL_DUPLEX,
	AUTO_DUPLEX
} EMAC_DUPLEX;

typedef enum {
	SPEED_10MBPS = 10,
	SPEED_100MBPS = 100,
	SPEED_200MBPS = 200,	/* Over clocked MII */
	AUTO_SPEED
} EMAC_SPEED;

/* EMAC private Data structure */
typedef struct {
	/* Handle of net_device this private structure is attached to */
	struct net_device *hNetDev;

	/* 0 for eth0, 1 for eth1 */
	unsigned short uInterfaceNo;

	/* 0 = Interface Not In Use, 1 = In Use */
	unsigned short uInUse;

	/* I/O Base address of the EMAC */
	unsigned long uBaseAddress;

	/* 0 = 10 Mbps, 100 = 100 Mbps, 200=200 Mbps */
	EMAC_SPEED uSpeed;

	/* 0 = Half Duplex, 1 = Full Duplex */
	EMAC_DUPLEX uMode;

	/* 0 = Disabled, 1 = Enabled */
	unsigned short uMulticastEnable;

	/* 0 = Disabled, 1 = Enabled */
	unsigned short uPromiscousEnable;

	/* The MAC address of this port */
	unsigned char mac[6];

	/* The return value from mac_Claim */
	unsigned long uMacIndex;	/* ToDo: need to be removed */

	/* Total no of Tx Buffer Descriptors in the Tx BDT */
	unsigned long uTxDescCount;

	/* Total no of Rx Buffer Descriptors in the Rx BDT */
	unsigned long uRxDescCount;

	/* Start of Memory block from where Tx and Rx BDT memory are allocated */
	unsigned char *puDescriptorBase;

	/* Size of Memory block allocated for Tx and Rx BDT */
	unsigned long uDescriptorSize;

	/* Points to the Buffer Descriptor in Tx BDT which will be used to store the
	 * upper layer supplied transmission buffer in the next Transmit call 
	 */
	EMAC_BUF_DESCR *tx_curr;

	/* Pointer to buffer which will be scheduled for transmission, when TXINT 
	 * will come this buffer will be freed. Corresponds to tx_curr 
	 */
	EMAC_LOCAL_BUF *tx_curr_local;

	/* Points to the Buffer Descriptor in Tx BDT from where we will start 
	 * checking for BD for which transmission has been completed during TXINT 
	 * processing 
	 */
	EMAC_BUF_DESCR *tx_refil;

	/* Pointer to buffer ,Corresponds to tx_refil */
	EMAC_LOCAL_BUF *tx_refil_local;

	/* Address of the First Buffer Descriptor in the Tx BDT */
	EMAC_BUF_DESCR *tx_first_desc;

	/* Address of the last Buffer Descriptor in the Tx BDT */
	EMAC_BUF_DESCR *tx_last_desc;

	/* Address of the First Buffer Corresponding to the first Buffer Descriptor 
	 * of Tx BDT 
	 */
	EMAC_LOCAL_BUF *tx_first_local;

	/* Points to the Buffer Descriptor in Rx BDT from which we will start 
	 * checking for BD which contains valid data 
	 */
	EMAC_BUF_DESCR *rx_curr;

	/* Pointer to received buffer ,Corresponds to rx_curr */
	EMAC_LOCAL_BUF *rx_curr_local;

	/* Points to the Buffer Descriptor in Rx BDT from which we will add buffer 
	 * which will be used to hold receive data by EMAC 
	 */
	EMAC_BUF_DESCR *rx_refil;

	/* Pointer to supplied buffer ,Corresponds to rx_refil */
	EMAC_LOCAL_BUF *rx_refil_local;

	/* Address of the First Buffer Descriptor in the Rx BDT */
	EMAC_BUF_DESCR *rx_first_desc;

	/* Address of the Last Buffer Descriptor in the Rx BDT */
	EMAC_BUF_DESCR *rx_last_desc;

	/* Address of the First Buffer Corresponding to the first Buffer Descriptor of Rx BDT */
	EMAC_LOCAL_BUF *rx_first_local;

	/* Interface Statistics */
	EMAC_STATS Stats;

	/* IRQ number for this interface */
	long iIrq;

	/* Maximum number of Buffer Descriptor that can be processed in one TX 
	 * Interrupt processing 
	 */
	unsigned short uTxIntBDProcessCnt;	/* ToDo: may required for Tx max processing in Tasklet */

	/* Maximum number of Buffer Descriptor that can be processed in one RX 
	 * Interrupt processing 
	 */
	unsigned short uRxIntBDProcessCnt;	/* ToDo: may required for Tx max processing in Tasklet */

	/* Queue for storing the data that has to be transmitted */
	struct sk_buff_head TxQueue;

	/* Number of currently available Tx buffer  */
	unsigned long TxQDepth;
	/* Maximum number of buffer that we can store locally for transmission */
	unsigned long uMaxTxQDepth;

	unsigned long uBufLen;	/* ToDo: need to be removed */

	spinlock_t lock;
	struct semaphore mutex;

	/* Logical filter setting value */
	unsigned long uHashLow;
	unsigned long uHashHigh;

	/* 0 for disable, 1 for enable */
	unsigned short uAutoneg_enable;
	EMAC_SPEED uCurrent_speed_select;
	EMAC_DUPLEX uCurrent_duplex_select;

#ifdef EMAC_MEDIA_MAINTANANCE
	void (*media_check_func) (struct net_device * dev);
#endif
	/* 0 MDIO operation not supported,1 MDIO operation supported */
	unsigned short uMDIO_support;

	struct mii_if_info mii_if;
	dma_addr_t uDmaAddr;

} EMAC_DEVICE;

unsigned short phy_autonegotiation_supported(struct net_device *dev);
int emac_open(unsigned short uInterfaceNumber, struct net_device *dev);
int emac_close(struct net_device *dev);
int emac_set_multicast_list(struct net_device *dev);
int emac_start_transmit(struct net_device *dev, void *pData, unsigned long uDataLen);
int emac_get_device_statistics(struct net_device *dev, EMAC_STATS * emacStats);
void emac_media_autonegotiate(struct net_device *dev);
void emac_media_force(struct net_device *dev, EMAC_SPEED uSpeed, EMAC_DUPLEX uDuplex);
int EmacReadReg_PHY(struct net_device *dev, int PhyAddr, int RegNo);
void EmacWriteReg_PHY(struct net_device *dev, int PhyAddr, int RegNo, int Data);
int emac_set_transmit_activity(struct net_device *dev, unsigned short uEnable);
/* function to read mac address */
void Get_mac_Address(unsigned char *uMacDest, unsigned short uInterfaceNo, unsigned short uAddrlen);
int phy_detect(struct net_device *dev);

#ifdef EMAC_MEDIA_MAINTANANCE
void select_media_check_function(struct net_device *dev);
int emac_media_check(void *);
#endif

int emac_get_device_attribute(struct net_device *dev, EMAC_ATTRIBUTES * pAttributes);
int emac_set_activity(struct net_device *dev, unsigned short uEnable);
int emac_set_receive_activity(struct net_device *dev, unsigned short uEnable);
int emac_set_broadcast(struct net_device *dev, unsigned short uEnable);
int emac_set_promiscuous(struct net_device *dev, unsigned short uEnable);
int emac_set_bdt_poolrate(struct net_device *dev, unsigned long uRate);
int emac_get_bdt_poolrate(struct net_device *dev, unsigned long *puRate);
unsigned long emac_get_transmit_bdt_length(struct net_device *dev);
unsigned long emac_get_receive_bdt_length(struct net_device *dev);
void phy_reset(struct net_device *dev);
void phy_set_mode(struct net_device *dev, EMAC_DUPLEX uMode);
void phy_set_speed(struct net_device *dev, EMAC_SPEED uSpeed);
void phy_loopback(struct net_device *dev, unsigned short uMode);
void phy_powerdown(struct net_device *dev, unsigned short uMode);
void phy_isolate(struct net_device *dev, unsigned short uMode);
void phy_collisiontest(struct net_device *dev, unsigned short uMode);
void phy_set_autonegotiation(struct net_device *dev, unsigned short uValue);
void phy_restart_autonegotiation(struct net_device *dev);
unsigned short phy_is_autonegotiation_completed(struct net_device *dev);

#ifdef EMAC_DRIVER_STATISTICS
void emac_stats_display(struct net_device *dev);
void emac_stats_clear(struct net_device *dev);
#endif
#endif

/****************************************************************************
 * Modifications:
 * $Log:
 *  4    Linux_SDK 1.3         8/9/07 7:51:38 PM IST  Upakul Barkakaty
 *       Implement network stop and wake routine to avoid continuous polling
 *       by the networking system under Hardware BDTs unavailable condition.
 *  3    Linux_SDK 1.2         8/3/07 2:49:58 PM IST  Upakul Barkakaty
 *       Configurable flags and logic added to enable the Second Ethernet
 *       interface with 2xMode operation for Emac.
 *  2    Linux_SDK 1.1         4/16/07 1:26:15 PM IST Rajesha Kini    Updated
 *       with GPL header.
 *  1    Linux_SDK 1.0         3/2/07 3:42:25 PM IST  Vineet Seth     
 * $
 *
 ****************************************************************************/
