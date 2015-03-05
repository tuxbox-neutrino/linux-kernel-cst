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

#ifndef _NX_DMAC_1902_PRIVATE_H
#define _NX_DMAC_1902_PRIVATE_H

#include <linux/nx_dmac_1902.h>

#define MAX_NX_DMAC_1902_CHANNELS (8)
#define NX_DMAC_1902_MOD_ID (0x00041080)
#define NX_DMAC_1903_MOD_ID (0x00041081)

/* register offset macros */
#define NX_DMAC_1902_REG_INT_STATUS         (0x000)
#define NX_DMAC_1902_REG_INT_TC_STATUS      (0x004)
#define NX_DMAC_1902_REG_INT_TC_CLEAR       (0x008)
#define NX_DMAC_1902_REG_INT_ERR_STATUS     (0x00C)
#define NX_DMAC_1902_REG_INT_ERR_CLEAR      (0x010)
#define NX_DMAC_1902_REG_INT_RAW_TC_STATUS  (0x014)
#define NX_DMAC_1902_REG_INT_RAW_ERR_STATUS (0x018)
#define NX_DMAC_1902_REG_ENABLED_CHANS      (0x01C)

#define NX_DMAC_1902_REG_DMAC_CONFIG        (0x030)
#define NX_DMAC_1902_REG_DMAC_SYNC          (0x034)

/* channel specific register */
#define NX_DMAC_1902_REG_CHAN_BASE(x) (0x100+((x)*0x20))
#define NX_DMAC_1902_REG_CHAN_SRC_ADDR      (0x000)
#define NX_DMAC_1902_REG_CHAN_DST_ADDR      (0x004)
#define NX_DMAC_1902_REG_CHAN_LLI_ADDR      (0x008)
#define NX_DMAC_1902_REG_CHAN_CONTROL       (0x00C)
#define NX_DMAC_1902_REG_CHAN_CONFIG        (0x010)

#define NX_DMAC_1902_REG_PERID_0            (0xFE0)
#define NX_DMAC_1902_REG_PERID_1            (0xFE4)
#define NX_DMAC_1902_REG_PERID_2            (0xFE8)
#define NX_DMAC_1902_REG_PERID_3            (0xFEC)

#define NX_DMAC_1902_REG_CELLID_0           (0xFF0)
#define NX_DMAC_1902_REG_CELLID_1           (0xFF4)
#define NX_DMAC_1902_REG_CELLID_2           (0xFF8)
#define NX_DMAC_1902_REG_CELLID_3           (0xFFC)

#define NX_DMAC_1902_DMAC_ENABLE            (0x00000001)
#define NX_DMAC_1902_DMAC_DISABLE           (0x00000000)


#define NX_DMAC_1902_CHAN_CNTRL_DST_INCR(x) ((x)<<27)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(x) ((x)<<26)
#define NX_DMAC_1902_CHAN_CNTRL_DST_AHB(x) ((x)<<25)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(x) ((x)<<24)
#define NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(x) ((x)<<21)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(x) ((x)<<18)
#define NX_DMAC_1902_CHAN_CNTRL_DST_BURST(x) ((x)<<15)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(x) ((x)<<12)

#define NX_DMAC_1902_CHAN_CNTRL_HPROT(x) ((x)<<28)

#define NX_DMAC_1902_HPROT_PRIVILEGE_MODE (0x01)
#define NX_DMAC_1902_HPROT_BUFFERABLE (0x02)
#define NX_DMAC_1902_HPROT_CACHEABLE (0x04)


#define NX_DMAC_1902_INT_ENABLE             (0x80000000UL)

#define NX_DMAC_1902_UNMASK_INT             (0x0000C000UL)
#define NX_DMAC_1902_CHAN_CONFIG_FLOWCNTRL(x) ((x) << 11)
#define NX_DMAC_1902_CHAN_CONFIG_DST_PER(x) ((x) << 6)	// destination
#define NX_DMAC_1902_CHAN_CONFIG_SRC_PER(x) ((x) << 1)	// source

#define NX_DMAC_1902_DMAC_CHAN_ENABLE       (0x00000001)
#define NX_DMAC_1902_DMAC_CHAN_DISABLE      (0x00000000)

#define NX_DMAC_1902_DMAC_CHAN_HALT         (0x00040000)
#define NX_DMAC_1902_DMAC_CHAN_ACTIVE       (0x00020000)

/* max number of elements to be transferred at a single dma descriptor node */
#define NX_DMAC_1902_MAX_XFER_SIZE          (0xFFF)

enum nx_dmac_1902_xfer_status {
	nx_dmac_1902_xfer_status_nothing=0,
	nx_dmac_1902_xfer_status_completed,
	nx_dmac_1902_xfer_status_failed
};

/* Notifications for native driver */
typedef enum nx_dmac_event{
	EVENT_COMPLETE = 1,
	EVENT_ERROR = 2,
	EVENT_TERM = 3,
	EVENT_NONE = 4,
}NX_DMAC_EVENT;

/* callback data */
struct callback_data
{
	unsigned int  chanNo;
	unsigned int  length;
	NX_DMAC_EVENT  event;
	unsigned int   async;
	void           *unit_instance;
	void           *pTag;
};

/* lli as in hardware */
struct nx_dmac_1902_lli {
	dma_addr_t src_addr;  /* source address */
	dma_addr_t dst_addr;  /* destination address */
	dma_addr_t llp;       /* link to next descriptor */
	u32        control;   /* control word for the IP_1902 channel */
	u32        config;    /* channel_config */
	u32        val_memset; /* value to be set during memset operation */
};

/* DMA descriptor */
struct nx_dmac_1902_desc {
	struct nx_dmac_1902_lli		lli;
	/* for driver housekeeping */
	struct list_head		desc_node;
	struct list_head		tx_list;
	struct dma_async_tx_descriptor	txd;
	dma_addr_t			p_addr_memset; /* physical memory address for lli.val_memset */
	size_t				len;
	struct callback_data		cbd;           /* Callback data */
};

/* channel specific structure */
struct nx_dmac_1902_chan {
	struct dma_chan		     chan;
	void __iomem *               ch_regs;           /* this will be same as that of device regs */
	u8                           chan_num;          /* channel number */
	spinlock_t                   lock;
	u8                           chan_free;         /* to indicate the channel is free or is allocated */
	struct nx_dmac_1902_slave    *nxs;              /* pointer to the slave specific config such as burst size, width size, etc.. */       
	dma_cookie_t                 completed;         /* indicates whether the transfer is completed or not. depending upon this, the tasklet will invoke the desc->callback */
	struct device                *dev;
	enum nx_dmac_1902_xfer_status xfer_status;      /* this indicates the xfer status of the channel */
	/* lists */
	struct list_head              active_list;       /* contains the descriptors which are currently active or to be programmed into the hardware */
	struct list_head              queue;             /* contains the next nodes which needs to be made active */
	struct list_head              free_list;         /* contains the descriptors which are free */

	unsigned int                  descs_allocated;   /* number of descriptors allocated */
	struct callback_data          cbd;               /* Callback data */
};

/* driver private structure */
struct nx_dmac_1902 {
	struct dma_device        dma;
	void __iomem *           regs;                /* points to the DMA IP register base address virtual address */

	struct tasklet_struct    tasklet;             /* tasklet to indicate success/failure of DMA transfer */
	/** DMA IP configuration */
	int                      irq_num;             /* irq number for DMAC */
	int                      num_channels;        /* number of channels supported by the IP */
	int                      num_masters;         /* number of AHB masters configured in the IP */
	int                      num_perreqs;         /* number of peripheral requests */
	struct nx_dmac_1902_chan chan[0];             /* channel specific data */
};


static inline struct nx_dmac_1902_chan *to_nx_dmac_1902_chan(struct dma_chan *chan)
{
	return container_of(chan, struct nx_dmac_1902_chan, chan);
}

static inline struct nx_dmac_1902 *to_nx_dmac_1902(struct dma_device *ddev)
{
	return container_of(ddev, struct nx_dmac_1902, dma);
}

static inline struct nx_dmac_1902_desc *txd_to_nx_dmac_1902_desc(struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct nx_dmac_1902_desc, txd);
}

#endif /* NX_DMAC_1902_PRIVATE_H */

