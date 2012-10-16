/*
 * ARC VMAC Driver
 *
 * Copyright (C) 2009-2012 Andreas Fenkart
 * All Rights Reserved.
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
 *
 * Initial work taken from arc linux distribution, any bugs are mine
 *
 *	-----<snip>-----
 * Copyright (C) 2003-2006 Codito Technologies, for linux-2.4 port
 * Copyright (C) 2006-2007 Celunite Inc, for linux-2.6 port
 * Authors: amit.bhor@celunite.com, sameer.dhavale@celunite.com
 *	-----<snip>-----
 */

#ifndef _ARCVMAC_H
#define _ARCVMAC_H

#define DRV_NAME		"arcvmac"
#define DRV_VERSION		"1.0"

/* Buffer descriptors */
#define TX_BDT_LEN		128    /* Number of receive BD's */
#define RX_BDT_LEN		128    /* Number of transmit BD's */

/* BD poll rate, in 1024 cycles. @100Mhz: x * 1024 cy * 10ns = 1ms */
#define POLLRATE_TIME		200

/* next power of two, bigger than ETH_FRAME_LEN + VLAN  */
#define MAX_RX_BUFFER_LEN	0x800	/* 2^11 = 2048 = 0x800 */
#define MAX_TX_BUFFER_LEN	0x800	/* 2^11 = 2048 = 0x800 */

/* 14 bytes of ethernet header, 4 bytes VLAN, FCS,
 * plus extra pad to prevent buffer chaining of
 * maximum sized ethernet packets (1514 bytes) */
#define VMAC_BUFFER_PAD		(ETH_HLEN + 4 + ETH_FCS_LEN + 4)

/* VMAC register definitions, offsets in bytes */
#define VMAC_ID			0x00

/* stat/enable use same bit mask */
#define VMAC_STAT		0x04
#define VMAC_ENABLE		0x08
#  define TXINT_MASK		0x00000001 /* Transmit interrupt */
#  define RXINT_MASK		0x00000002 /* Receive interrupt */
#  define ERR_MASK		0x00000004 /* Error interrupt */
#  define TXCH_MASK		0x00000008 /* Transmit chaining error */
#  define MSER_MASK		0x00000010 /* Missed packet counter error */
#  define RXCR_MASK		0x00000100 /* RXCRCERR counter rolled over	 */
#  define RXFR_MASK		0x00000200 /* RXFRAMEERR counter rolled over */
#  define RXFL_MASK		0x00000400 /* RXOFLOWERR counter rolled over */
#  define MDIO_MASK		0x00001000 /* MDIO complete */
#  define TXPL_MASK		0x80000000 /* TXPOLL */

#define VMAC_CONTROL		0x0c
#  define EN_MASK		0x00000001 /* VMAC enable */
#  define TXRN_MASK		0x00000008 /* TX enable */
#  define RXRN_MASK		0x00000010 /* RX enable */
#  define DSBC_MASK		0x00000100 /* Disable receive broadcast */
#  define ENFL_MASK		0x00000400 /* Enable Full Duplex */
#  define PROM_MASK		0x00000800 /* Promiscuous mode */

#define VMAC_POLLRATE		0x10

#define VMAC_RXERR		0x14
#  define RXERR_CRC		0x000000ff
#  define RXERR_FRM		0x0000ff00
#  define RXERR_OFLO		0x00ff0000 /* fifo overflow */

#define VMAC_MISS		0x18
#define VMAC_TXRINGPTR		0x1c
#define VMAC_RXRINGPTR		0x20
#define VMAC_ADDRL		0x24
#define VMAC_ADDRH		0x28
#define VMAC_LAFL		0x2c
#define VMAC_LAFH		0x30
#define VMAC_MAC_TXRING_HEAD	0x38
#define VMAC_MAC_RXRING_HEAD	0x3C

#define VMAC_MDIO_DATA		0x34
#  define MDIO_SFD		0xC0000000
#  define MDIO_OP		0x30000000
#  define MDIO_ID_MASK		0x0F800000
#  define MDIO_REG_MASK		0x007C0000
#  define MDIO_TA		0x00030000
#  define MDIO_DATA_MASK	0x0000FFFF
/* common combinations */
#  define MDIO_BASE		0x40020000
#  define MDIO_OP_READ		0x20000000
#  define MDIO_OP_WRITE		0x10000000

/* Buffer descriptor INFO bit masks */
#define BD_DMA_OWN		0x80000000 /* buffer ownership, 0 CPU, 1 DMA */
#define BD_BUFF			0x40000000 /* buffer invalid, rx */
#define BD_UFLO			0x20000000 /* underflow, tx */
#define BD_LTCL			0x10000000 /* late collision, tx  */
#define BD_RETRY_CT		0x0f000000 /* tx */
#define BD_DROP			0x00800000 /* drop, more than 16 retries, tx */
#define BD_DEFER		0x00400000 /* traffic on the wire, tx */
#define BD_CARLOSS		0x00200000 /* carrier loss while transmission, tx, rx? */
/* 20:19 reserved */
#define BD_ADCR			0x00040000 /* add crc, ignored if not disaddcrc */
#define BD_LAST			0x00020000 /* Last buffer in chain */
#define BD_FRST			0x00010000 /* First buffer in chain */
/* 15:11 reserved */
#define BD_LEN			0x000007FF

/* common combinations */
#define BD_TX_ERR		(BD_UFLO | BD_LTCL | BD_RETRY_CT | BD_DROP | \
				 BD_DEFER | BD_CARLOSS)


/* arcvmac private data structures */
struct vmac_buffer_desc {
	__le32 info;
	__le32 data;
};

struct dma_fifo {
	int head; /* head */
	int tail; /* tail */
	int size;
};

struct vmac_priv {
	struct net_device *dev;
	struct platform_device *pdev;

	struct completion mdio_complete;
	spinlock_t lock; /* protects structure plus hw regs of device */

	/* base address of register set */
	char __iomem *regs;
	struct resource *mem;

	/* DMA ring buffers */
	struct vmac_buffer_desc *rxbd;
	dma_addr_t rxbd_dma;

	struct vmac_buffer_desc *txbd;
	dma_addr_t txbd_dma;

	/* socket buffers */
	struct sk_buff *rx_skbuff[RX_BDT_LEN];
	struct sk_buff *tx_skbuff[TX_BDT_LEN];
	int rx_skb_size;

	/* skb / dma desc managing */
	struct dma_fifo rx_ring; /* valid rx buffers */
	struct dma_fifo tx_ring;

	/* descriptor last polled/processed by the VMAC */
	unsigned long dma_rx_head;

	/* timer to retry rx skb allocation, if failed during receive */
	struct timer_list refill_timer;
	spinlock_t refill_lock;

	struct napi_struct napi;

	/* rx buffer chaining */
	int rx_merge_error;
	int tx_timeout_error;

	/* PHY stuff */
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;

	int link;
	int speed;
	int duplex;

	/* debug */
	bool shutdown;
};

/* DMA ring management */

/* for a fifo with size n,
 * - [0..n] fill levels are n + 1 states
 * - there are only n different deltas (head - tail) values
 * => not all fill levels can be represented with head, tail
 *    pointers only
 * we give up the n fill level, aka fifo full */

/* sacrifice one elt as a sentinel */
static inline int fifo_used(struct dma_fifo *f);
static inline int fifo_inc_ct(int ct, int size);
static inline void fifo_dump(struct dma_fifo *fifo);

static inline bool fifo_empty(struct dma_fifo *f)
{
	return f->head == f->tail;
}

static inline int fifo_free(struct dma_fifo *f)
{
	int free;

	free = f->tail - f->head;
	if (free <= 0)
		free += f->size;

	return free;
}

static inline int fifo_used(struct dma_fifo *f)
{
	int used;

	used = f->head - f->tail;
	if (used < 0)
		used += f->size;

	return used;
}

static inline bool fifo_full(struct dma_fifo *f)
{
	return (fifo_used(f) + 1) == f->size;
}

/* manipulate */
static inline void fifo_init(struct dma_fifo *fifo, int size)
{
	fifo->size = size;
	fifo->head = fifo->tail = 0; /* empty */
}

static inline void fifo_inc_head(struct dma_fifo *fifo)
{
	BUG_ON(fifo_full(fifo));
	fifo->head = fifo_inc_ct(fifo->head, fifo->size);
}

static inline void fifo_inc_tail(struct dma_fifo *fifo)
{
	BUG_ON(fifo_empty(fifo));
	fifo->tail = fifo_inc_ct(fifo->tail, fifo->size);
}

/* internal funcs */
static inline void fifo_dump(struct dma_fifo *fifo)
{
	pr_info("fifo: head %d, tail %d, size %d\n", fifo->head,
		fifo->tail,
		fifo->size);
}

static inline int fifo_inc_ct(int ct, int size)
{
	return (++ct == size) ? 0 : ct;
}

#endif	  /* _ARCVMAC_H */
