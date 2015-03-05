/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NX_DMAC_H
#define _NX_DMAC_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>

/**
 * Number of descriptors per channel
 **/
#define NR_DESCRIPTORS      16
/**
* Flow control 
*/
typedef enum tag_dmac_fl {
	nx_dmac_mem2mem_dma=0x0,		/* mem2mem with DMA flow control */
	nx_dmac_mem2per_dma=0x1,		/* mem2per with DMA flow control */
	nx_dmac_per2mem_dma=0x2,		/* per2mem with DMA flow control */
	nx_dmac_per2per_dma=0x3,		/* per2per with DMA flow control */
	nx_dmac_per2per_dstper=0x4,		/* per2per with dst per flow control */
	nx_dmac_mem2per_per=0x5,		/* mem2per with per flow control */
	nx_dmac_per2mem_per=0x6,		/* per2mem with per flow control */
	nx_dmac_per2per_srcper=0x7,		/* per2per with src per flow control */
} nx_dmac_flctl_t;					

/**
* DMAC burst size 
*/
typedef enum tag_dmac_burst {
	nx_dmac_1=0x0,				/*	1 word */	
	nx_dmac_4=0x1,				/*	4 words */	
	nx_dmac_8=0x2,				/*	8 words */	
	nx_dmac_16=0x3,				/*	16 words */	
	nx_dmac_32=0x4,				/*	32 words */	
	nx_dmac_64=0x5,				/*	64 words */	
	nx_dmac_128=0x6,			/*	128 words */	
	nx_dmac_256=0x7,			/*	256 words */	
} nx_dmac_burst_t;					

/**
* DMAC width 
*/
typedef enum tag_dmac_width {
	nx_dmac_width_8=0x0,			/*	8 bits */	
	nx_dmac_width_16=0x1,			/*	16 bits */	
	nx_dmac_width_32=0x2,			/*	32 bits */	
} nx_dmac_width_t;					

/**
* DMAC scatter gather request structure 
*/
typedef struct tag_dmac_stgt {
	uint32_t		src_addr;	/* Source physical address */
	uint32_t		dst_addr;	/* Destination physical address */
	uint32_t		tfr_size;	/* Transfer size in words */
	nx_dmac_flctl_t		flowctl;	/* Flow control */
	uint32_t		src_per;	/* Source peripheral number */
	uint32_t		dst_per;	/* Destination perpheral number */
	uint32_t		src_ahb;	/* Source AHB master */
	uint32_t		dst_ahb;	/* Destination AHB master */
	bool			src_inc;	/* Source Increment flag */
	bool			dst_inc;	/* Destination Increment flag */
	nx_dmac_burst_t		src_brst;	/* Source burst size */
	nx_dmac_burst_t		dst_brst;	/* Destination burst size */
	nx_dmac_width_t		src_width;	/* Source width */
	nx_dmac_width_t		dst_width;	/* Destination width */
} nx_dmac_stgt_t;

/**
* DMAC Transfer request structure 
*/
typedef struct tag_dmac_tfr {
 	int                 num_reqs;         /* # of requests in scatter gather */
 	nx_dmac_stgt_t	    *req; 	      /* Scatter gather structure */
} nx_dmac_tfr_t;

/**
* DMAC Transfer function 
*/
extern int nx_dmac_tfr(nx_dmac_tfr_t *req);

/**
* DMAC Transfer complete function 
*/
extern int nx_dmac_tfr_comp(int chanid);

/** 
 * Following are the declarations to be 
 * used only by the 
 * device driver source file
 **/
#define 	NX_DMAC_INT_STATUS		(0x000)
#define 	NX_DMAC_INT_TC_STATUS		(0x004)
#define 	NX_DMAC_INT_TC_CLR		(0x008)
#define 	NX_DMAC_INT_ERR_STATUS		(0x00C)
#define 	NX_DMAC_INT_ERR_CLR		(0x010)
#define 	NX_DMAC_INT_RAW_TC_STATUS	(0x014)
#define 	NX_DMAC_INT_RAW_ERR_STATUS	(0x018)
#define 	NX_DMAC_ENLD_CHANS		(0x01C)
#define 	NX_DMAC_SOFT_BREQ		(0x020)
#define 	NX_DMAC_SOFT_SREQ		(0x024)
#define 	NX_DMAC_SOFT_LBREQ		(0x028)
#define 	NX_DMAC_SOFT_LSREQ		(0x02C)
#define 	NX_DMAC_CONFIG			(0x030)
#define 	NX_DMAC_SYNC			(0x034)
#define 	NX_DMAC_CHAN0_SRC		(0x100)
#define 	NX_DMAC_CHAN0_DST		(0x104)
#define 	NX_DMAC_CHAN0_LLI		(0x108)
#define 	NX_DMAC_CHAN0_CTRL		(0x10C)
#define 	NX_DMAC_CHAN0_CONFIG		(0x110)
#define 	NX_DMAC_PERI_ID0		(0xFE0)
#define 	NX_DMAC_PERI_ID1		(0xFE4)
#define 	NX_DMAC_PERI_ID2		(0xFE8)
#define 	NX_DMAC_PERI_ID3		(0xFEC)
#define 	NX_DMAC_CELL_ID0		(0xFF0)
#define 	NX_DMAC_CELL_ID1		(0xFF4)
#define 	NX_DMAC_CELL_ID2		(0xFF8)
#define 	NX_DMAC_CELL_ID3		(0xFFC)

#define		NX_DMAC_CHAN_OFF		(0x020)

/**
* DMAC LLI structure 
*/
struct nx_dmac_lli_t {
	uint32_t	src_addr;	/* Source physical address */
	uint32_t	dst_addr;	/* Source physical address */
	uint32_t	next_lli;	/* Next LLI address */
	uint32_t	ctrl;		/* Control word for transfer */
};

/**
* DMAC channel structure 
*/
struct nx_dmac_chan_t {
 	int			chanid;		/* channel ID */
	bool			busy;		/* In use */
 	struct mutex		chan_lock;      /* Channel mutex */
 	int			chan_status;    /* DMAC channel status */
	wait_queue_head_t	chan_queue;     /* DMAC channel queue */     
	struct nx_dmac_lli_t	*lli_ptr;	/* LLI ptr for channel */
	dma_addr_t          	lli_phy;	/* Physical address of LLI */
};

/**
* DMAC device structure 
*/
struct nx_dmac_t {
	void __iomem        	*dmac_base; 	/* DMAC base address */
	int			num_chans;	/* # of supported channels */
	int			num_ahb_mas;	/* # of AHB masters */
	int			num_reqs;	/* # of requestors */
 	struct mutex		dmac_lock;    	/* DMAC lock */
	struct	nx_dmac_chan_t	*chans;		/* Channel structure */
};

/* NAND+NOR DMA support -> used by nx_dmac.c and nx_nor.c */
#define NX_DMAC_NOR_DMA_CHANNEL_ID  (7)
#define NX_DMAC_SFC_DMA_CHANNEL_ID  (6)

#endif /* __KERNEL__ */

#endif /* _NX_DMAC_H */
