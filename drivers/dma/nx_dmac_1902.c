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

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/version.h>

#include "nx_dmac_1902_private.h"
#include "dmaengine.h"

#define DRIVER_NAME          "nx_dmac_1902"
#define NR_DESCS_PER_CHANNEL  CONFIG_NX_DMAC_NR_OF_DESCS

/* Global declartion of gAsyncCallBackFunct(callback) function */
dma_async_tx_callback gAsyncCallBackFunct = NULL;

/*Global*/
struct platform_device *nativePdev;

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}
static struct device *chan2parent(struct dma_chan *chan)
{
	return chan->dev->device.parent;
}

static struct nx_dmac_1902_desc *nx_dmac_1902_first_active(struct nx_dmac_1902_chan *nxc)
{
	return list_entry(nxc->active_list.next, struct nx_dmac_1902_desc, desc_node);
}

static struct nx_dmac_1902_desc *nx_dmac_1902_first_queued(struct nx_dmac_1902_chan *nxc)
{
	return list_entry(nxc->queue.next, struct nx_dmac_1902_desc, desc_node);
}

static struct nx_dmac_1902_desc *nx_dmac_1902_get_desc(struct nx_dmac_1902_chan *nxc)
{
	struct nx_dmac_1902_desc *desc, *_desc;
	struct nx_dmac_1902_desc *ret = NULL;
	unsigned int i = 0;

	spin_lock_bh(&(nxc->lock));
	list_for_each_entry_safe(desc, _desc, &nxc->free_list, desc_node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del(&desc->desc_node);
			ret = desc;
			break;
		}
		dev_dbg(chan2dev(&nxc->chan), "get_desc: desc %p not ACKed\n", desc);
		i++;
	}
	spin_unlock_bh(&nxc->lock);

	dev_vdbg(chan2dev(&nxc->chan), "get_desc: scanned %u descriptors on freelist\n", i);

	return ret;
}

static bool is_chan_enabled(struct nx_dmac_1902 *nxd, int chan_num)
{
	u32 reg_val;

	reg_val = readl(nxd->regs+NX_DMAC_1902_REG_ENABLED_CHANS);
	if( (reg_val & ( 1 << chan_num)) )
		return 1;
	else
		return 0;
}

static void nx_dmac_1902_channel_disable( struct nx_dmac_1902 *nxd, int channel_num)
{
	u32 status;

	/*
	 * just do a abrupt disable instead of clean disable. Because, even if the
	 * channel is enabled and has some error with transfer and if caller calls
	 * terminate_all, where it is expected to terminate the transfers which are
	 * in whatever condition it is. So, do not go for a clean disable
	 */

	/* check whether the channel is enabled or not */
	status = readl(nxd->regs+NX_DMAC_1902_REG_ENABLED_CHANS);

	if( ( status & ( 0x01 << channel_num) ) ) {
		/* do a clean disable */
		status = readl(nxd->regs+NX_DMAC_1902_REG_CHAN_BASE(channel_num)+NX_DMAC_1902_REG_CHAN_CONFIG);
		status &= ~NX_DMAC_1902_DMAC_CHAN_ENABLE;
		writel(status,(nxd->regs+NX_DMAC_1902_REG_CHAN_BASE(channel_num)+NX_DMAC_1902_REG_CHAN_CONFIG));

		/* wait till the channel is not active */
		do {
			status = readl(nxd->regs+NX_DMAC_1902_REG_CHAN_BASE(channel_num)+NX_DMAC_1902_REG_CHAN_CONFIG);
		} while( ( status & NX_DMAC_1902_DMAC_CHAN_ACTIVE ) > 0);
	}

	return;
}

/*nx_disable_wrapper function is supporte for disabling the channel*/
int nx_disable_wrapper(struct dma_chan *chan)
{
	unsigned long flags;
	struct nx_dmac_1902_chan *nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902	*nxd = to_nx_dmac_1902(nxc->chan.device);

	spin_lock_irqsave(&nxc->lock, flags);

	/* just call the nx_dmac_1902_channel_disable for adrupt disable of chan, 
	   clean disable support will be avilable in next version of implemetation */
	nx_dmac_1902_channel_disable(nxd,chan->chan_id);
	spin_unlock_irqrestore(&nxc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(nx_disable_wrapper);

static void nx_dmac_1902_sync_desc_for_cpu(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_desc *desc)
{
	struct nx_dmac_1902_desc	*child;

	list_for_each_entry(child, &desc->tx_list, desc_node)
		dma_sync_single_for_cpu(chan2parent(&nxc->chan),
				child->txd.phys, sizeof(child->lli),
				DMA_TO_DEVICE);
	dma_sync_single_for_cpu(chan2parent(&nxc->chan),
			desc->txd.phys, sizeof(desc->lli),
			DMA_TO_DEVICE);
}

/*
 * Move a descriptor, including any children, to the free list.
 * `desc' must not be on any lists.
 */
static void nx_dmac_1902_desc_move_to_freelist(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_desc *desc)
{

	if (desc) {
		struct nx_dmac_1902_desc *child;

		nx_dmac_1902_sync_desc_for_cpu(nxc, desc);

		spin_lock_bh(&nxc->lock);
		list_for_each_entry(child, &desc->tx_list, desc_node)
			dev_vdbg(chan2dev(&nxc->chan),
					"move_to_freelist: moving child desc %p to freelist\n",
					child);
		list_splice_init(&desc->tx_list, &nxc->free_list);
		//dev_vdbg(chan2dev(&nxc->chan), "move_to_freelist: moving desc %p to freelist\n", desc);
		list_add(&desc->desc_node, &nxc->free_list);
		spin_unlock_bh(&nxc->lock);
	}
}

/* Called with nxc->lock held and bh disabled */
	static dma_cookie_t
nx_dmac_1902_assign_cookie(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_desc *desc)
{
	dma_cookie_t cookie = nxc->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	nxc->chan.cookie = cookie;
	desc->txd.cookie = cookie;

	return cookie;
}

static void nx_dmac_1902_clear_int_status(struct nx_dmac_1902_chan *nxc)
{
	writel((0x01<<nxc->chan_num), nxc->ch_regs+NX_DMAC_1902_REG_INT_ERR_CLEAR); /* clear the error interrupt status */
	writel((0x01<<nxc->chan_num), nxc->ch_regs+NX_DMAC_1902_REG_INT_TC_CLEAR); /* clear the tc interrupt status */
	return;
}

/* Called with nxc->lock held and bh disabled */
static void nx_dmac_1902_start_xfer(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_desc *first)
{
	struct nx_dmac_1902	*nxd = to_nx_dmac_1902(nxc->chan.device);

	/* ASSERT:  channel is idle */
	if (is_chan_enabled(nxd, nxc->chan_num)) {
		dev_err(chan2dev(&nxc->chan),
				"BUG: Attempted to start non-idle channel\n");
		dev_err(chan2dev(&nxc->chan),
				"  SAR: 0x%x DAR: 0x%x LLP: 0x%x CTL: 0x%08x CFG:0x%08x\n",
				readl(nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_SRC_ADDR),
				readl(nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_DST_ADDR),
				readl(nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_LLI_ADDR),
				readl(nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_CONTROL),
				readl(nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_CONFIG));
		return;
	}

	dev_vdbg(chan2dev(&nxc->chan),
			"start_xfer: SAR: 0x%x DAR: 0x%x LLP: 0x%x CTL: 0x%08x CFG:0x%08x\n",
			first->lli.src_addr, first->lli.dst_addr, first->lli.llp, first->lli.control, first->lli.config);

	nx_dmac_1902_clear_int_status(nxc);
	writel(first->lli.src_addr, nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_SRC_ADDR);
	writel(first->lli.dst_addr, nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_DST_ADDR);
	writel(first->lli.llp, nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_LLI_ADDR);
	writel(first->lli.control, nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_CONTROL);
	writel((first->lli.config | NX_DMAC_1902_DMAC_CHAN_ENABLE), nxc->ch_regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_CONFIG);

	return;
}


int nx_chan_enable_wrapper(struct dma_chan *chan)
{
	struct nx_dmac_1902_chan *nxc = to_nx_dmac_1902_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&nxc->lock, flags);

	/* check the active queue if its not empty call the queued transfer first */
	if(!list_empty(&nxc->active_list))
	{
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_active(nxc));
	}
	/* check the queue if its not empty call the queued transfer first */
	else if (!list_empty(&nxc->queue))
	{
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_queued(nxc));
		list_splice_init(&nxc->queue, &nxc->active_list);
	}

	spin_unlock_irqrestore(&nxc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(nx_chan_enable_wrapper);


int nx_dmac_1902_async_memcpy_buf_available(
		struct dma_chan *chan,
		size_t len)
{
	struct nx_dmac_1902_desc *desc, *_desc;
	unsigned long flags;
	struct nx_dmac_1902_chan  *nxc = to_nx_dmac_1902_chan(chan);
	size_t max_size_by_width = (NX_DMAC_1902_MAX_XFER_SIZE * (1<<DMA_SLAVE_WIDTH_32BIT)); 
	size_t buf_available = 0;
	int retval = -1;

	spin_lock_irqsave(&(nxc->lock), flags);
	list_for_each_entry_safe(desc, _desc, &nxc->free_list, desc_node) {
		if (async_tx_test_ack(&desc->txd)) {
			buf_available += max_size_by_width;
			if (buf_available >= len) {
				retval = 0;
				break;
			}
		}
	}
	spin_unlock_irqrestore(&nxc->lock, flags);

	return retval;
}
EXPORT_SYMBOL(nx_dmac_1902_async_memcpy_buf_available);

static void nx_dmac_1902_desc_complete(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_desc *desc, NX_DMAC_EVENT event)
{
	dma_async_tx_callback		callback;
	void				*param;
	struct dma_async_tx_descriptor	*txd = &desc->txd;
	struct callback_data callbackData;

	dev_vdbg(chan2dev(&nxc->chan), "descriptor %u complete\n", txd->cookie);

	nxc->completed = txd->cookie;
	dma_cookie_complete(txd);
	callback = txd->callback;
	param = txd->callback_param;

	nx_dmac_1902_sync_desc_for_cpu(nxc, desc);
	list_splice_init(&desc->tx_list, &nxc->free_list);
	list_move(&desc->desc_node, &nxc->free_list);

	if (!nxc->chan.private) {
		struct device *parent = chan2parent(&nxc->chan);
		if (!(txd->flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
			if (txd->flags & DMA_COMPL_DEST_UNMAP_SINGLE)
				dma_unmap_single(parent, desc->lli.dst_addr, desc->len, DMA_FROM_DEVICE);
			else
				dma_unmap_page(parent, desc->lli.dst_addr, desc->len, DMA_FROM_DEVICE);
		}
		if (!(txd->flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
			if (txd->flags & DMA_COMPL_SRC_UNMAP_SINGLE)
				dma_unmap_single(parent, desc->lli.src_addr, desc->len, DMA_TO_DEVICE);
			else
				dma_unmap_page(parent, desc->lli.src_addr, desc->len, DMA_TO_DEVICE);
		}
	}

	/*
	 * The API requires that no submissions are done from a
	 * callback, so we don't need to drop the lock here
	 */
	if (callback)
	{
		if(param)
		{
			callback(param);
		}
		else
		{        
			if(event != EVENT_NONE)
			{
				callbackData.chanNo = (unsigned int)nxc->chan_num;
				callbackData.event = event;
				callbackData.length = desc->len;
				callbackData.async = desc->cbd.async;
				callbackData.unit_instance = (void *)desc->cbd.unit_instance;
				callbackData.pTag = (void *)desc->cbd.pTag;
				callback(&callbackData);
			}
		}
	}

}

static void nx_dmac_1902_scan_descriptors(struct nx_dmac_1902 *nxd, struct nx_dmac_1902_chan *nxc)
{
	dma_addr_t llp;
	struct nx_dmac_1902_desc *desc, *_desc;
	struct nx_dmac_1902_desc *child;
	NX_DMAC_EVENT event;

	/* get the current lli present in the DMAC hardware */
	llp = readl(nxd->regs+NX_DMAC_1902_REG_CHAN_BASE(nxc->chan_num)+NX_DMAC_1902_REG_CHAN_LLI_ADDR);
	dev_vdbg(chan2dev(&nxc->chan), "scan_descriptors: llp=0x%x\n", llp);

	list_for_each_entry_safe(desc, _desc, &nxc->active_list, desc_node) {
		if (desc->lli.llp == llp)
			/* This one is currently in progress */
			return;

		list_for_each_entry(child, &desc->tx_list, desc_node)
			if (child->lli.llp == llp)
				/* Currently in progress */
				return;

		/* post the event none if no descriptor is found */
		event = EVENT_NONE;

		/*
		 * No descriptors so far seem to be in progress, i.e.
		 * this one must be done.
		 */
		nx_dmac_1902_desc_complete(nxc, desc,event);
	}

	/* 
	 * since if a transfer is already complete, the channel is disabled. 
	 * just start the new transfer if it is queued 
	 * */
	if (!list_empty(&nxc->queue)) {
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_queued(nxc));
		list_splice_init(&nxc->queue, &nxc->active_list);
	}
}

static void nx_dmac_1902_dump_lli(struct nx_dmac_1902_chan *nxc, struct nx_dmac_1902_lli *lli)
{
	dev_crit(chan2dev(&nxc->chan),
			"  desc: src_addr=0x%08x dst_addr=0x%08x llp=0x%08x control=0x%08x config=0x%08X\n",
			lli->src_addr, lli->dst_addr, lli->llp, lli->control, lli->config);
	return;
}

static void nx_dmac_1902_handle_error(struct nx_dmac_1902 *nxd, struct nx_dmac_1902_chan *nxc)
{
	struct nx_dmac_1902_desc *desc, *child;
	NX_DMAC_EVENT event;

	dev_vdbg(chan2dev(&nxc->chan), "handle_failure: for channel %d\n", nxc->chan_num );

	/* 
	 * here in this, just free up the descriptor and queue the next 
	 * one for trasnfer if it exists 
	 */
	BUG_ON(list_empty(&nxc->active_list));

	desc = nx_dmac_1902_first_active(nxc);
	list_del_init(&desc->desc_node);

	/* 
	 * the channel will be automatically disabled, no need to disable 
	 * channel explicitly 
	 */

	nxc->xfer_status = nx_dmac_1902_xfer_status_nothing;

	list_splice_init(&nxc->queue, nxc->active_list.prev); 
	if (!list_empty(&nxc->queue))
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_queued(nxc));

	/* here just bark about the lli */
	dev_printk(KERN_CRIT, chan2dev(&nxc->chan), "Bad descriptor submitted for DMA!\n");
	dev_printk(KERN_CRIT, chan2dev(&nxc->chan), "  cookie: %d\n", desc->txd.cookie);
	nx_dmac_1902_dump_lli(nxc, &desc->lli);
	list_for_each_entry(child, &desc->tx_list, desc_node)
		nx_dmac_1902_dump_lli(nxc, &child->lli);

	/*Post the error event */
	event = EVENT_ERROR;

	/* call the callback */
	nx_dmac_1902_desc_complete(nxc, desc,event);

}  


static void nx_dmac_1902_handle_success(struct nx_dmac_1902 *nxd, struct nx_dmac_1902_chan *nxc)
{
	struct nx_dmac_1902_desc *desc;

	NX_DMAC_EVENT event;

	dev_vdbg(chan2dev(&nxc->chan), "handle_success: for channel %d\n", nxc->chan_num );

	if( list_empty(&nxc->queue) ) {
		dev_vdbg(chan2dev(&nxc->chan), "handle_success: begin: queue is EMPTY\n");
	}
	else {
		dev_vdbg(chan2dev(&nxc->chan), "handle_success: begin: queue is NOT EMPTY\n");
	}

	/* 
	 * here in this, just free up the descriptor and queue the next one for 
	 * trasnfer if it exists 
	 */
	BUG_ON(list_empty(&nxc->active_list));

	desc = nx_dmac_1902_first_active(nxc);
	list_del_init(&desc->desc_node);

	nxc->xfer_status = nx_dmac_1902_xfer_status_nothing;

	/* the channel is already disabled, as transfer is succefully completed */


	/* 
	 * we are starting another DMA transfer that is queued before calling the
	 * previous transfer callback() 
	 */
	if (list_empty(&nxc->active_list))
	{
		if (!list_empty(&nxc->queue)) {
			dev_vdbg(chan2dev(&nxc->chan), "handle_success: queue is not empty. Starting that transfer\n");
			list_splice_init(&nxc->queue, &nxc->active_list);
		}
		else {
			dev_vdbg(chan2dev(&nxc->chan), "handle_success: Queue is EMPTY!!!\n");
		}
	}

	if (!list_empty(&nxc->active_list)) {
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_active(nxc));
	}

	/* Post the event complete */
	event = EVENT_COMPLETE;

	/* call the callback */
	nx_dmac_1902_desc_complete(nxc, desc,event);
}  


static void nx_dmac_1902_tasklet(unsigned long data)
{
	struct nx_dmac_1902 *nxd = (struct nx_dmac_1902 *)data;
	struct nx_dmac_1902_chan *nxc;
	int i;

	for (i = 0; i < nxd->num_channels; i++) {
		nxc = &nxd->chan[i];

		spin_lock(&nxc->lock);
		if((nxc->xfer_status == nx_dmac_1902_xfer_status_failed )) {
			dev_vdbg(chan2dev(&nxc->chan), "tasklet: xfer_failed for channel %d\n", nxc->chan_num );
			nx_dmac_1902_handle_error(nxd, nxc);
		}
		else if((nxc->xfer_status == nx_dmac_1902_xfer_status_completed )) {
			dev_vdbg(chan2dev(&nxc->chan), "tasklet: xfer_completed for channel %d\n", nxc->chan_num );
			nx_dmac_1902_handle_success(nxd, nxc);
		}
		else {
			/* this should not come here at all */
		}
		spin_unlock(&nxc->lock);
	}
	return;   
}

static irqreturn_t nx_dmac_1902_isr(int irq, void *dev_id)
{
	/*
	 * here we will clear the interrupt TC status and interrupt ERR status and 
	 * then schedule a tasklet which will take care of going through the RAW 
	 * interrupt status and call the callbacks of those descriptors which 
	 * is complete
	 *
	 * The lli, src addr, dst addr etc are still be present in the channel register
	 */
	struct nx_dmac_1902 *nxd = (struct nx_dmac_1902 *) dev_id;
	struct nx_dmac_1902_chan *nxc = &nxd->chan[0];
	u32 int_status;
	u32 int_tc_status;
	u32 int_err_status;
	u32 int_raw_tc_status;
	u32 int_raw_err_status;
	int i;

	dev_vdbg(nxd->dma.dev, "nx_dmac_1902_isr: got interrupt\n");

	int_status = readl(nxd->regs);
	if( !(int_status & 0xFF )) {
		/* spurious interrupt. just return with IRQ_HANDLED */
		return IRQ_HANDLED;
	}

	int_tc_status = readl(nxd->regs+NX_DMAC_1902_REG_INT_TC_STATUS);
	int_err_status = readl(nxd->regs+NX_DMAC_1902_REG_INT_ERR_STATUS);
	int_raw_tc_status = readl(nxd->regs+NX_DMAC_1902_REG_INT_RAW_TC_STATUS);
	int_raw_err_status = readl(nxd->regs+NX_DMAC_1902_REG_INT_RAW_ERR_STATUS);

	dev_vdbg(nxd->dma.dev, "  tc_status      = 0x%08x\n", int_tc_status);
	dev_vdbg(nxd->dma.dev, "  err_status     = 0x%08x\n", int_err_status);
	dev_vdbg(nxd->dma.dev, "  raw_tc_status  = 0x%08x\n", int_raw_tc_status);
	dev_vdbg(nxd->dma.dev, "  raw_err_status = 0x%08x\n", int_raw_err_status);

	/**
	 * Here just read the interrupt status and then post the same status onto the
	 * channel specific structure, indicating that the interrupt has occured and
	 * transfser completion/failure status is set.
	 */
	for(i=0; i < nxd->num_channels; i++) {
		nxc = &nxd->chan[i];
		if(int_tc_status & (0x01 << i)) {
			nxc->xfer_status = nx_dmac_1902_xfer_status_completed;
		}
		else if(int_err_status & (0x01 << i)) {
			nxc->xfer_status = nx_dmac_1902_xfer_status_failed;
		}
		else {
			/* this condition will not arise at all */
		}
	}

	/* clear the interrupts */
	writel(int_tc_status, nxd->regs+NX_DMAC_1902_REG_INT_TC_CLEAR);
	writel(int_err_status, nxd->regs+NX_DMAC_1902_REG_INT_ERR_CLEAR);

	tasklet_schedule(&nxd->tasklet);

	return IRQ_HANDLED;
}

void nx_dmac_1902_set_async_tx_callback_funct(dma_async_tx_callback funct)
{
	gAsyncCallBackFunct = funct;
}
EXPORT_SYMBOL(nx_dmac_1902_set_async_tx_callback_funct);

void nx_dmac_1902_set_async_tx_callback_param(struct dma_chan *chan, void *param)
{
	struct nx_dmac_1902_chan *nxc = to_nx_dmac_1902_chan(chan);

	memcpy(&nxc->cbd, param, sizeof(struct callback_data));
}
EXPORT_SYMBOL(nx_dmac_1902_set_async_tx_callback_param);

void nx_dmac_1902_async_tx_callback(void *dma_async_param) 
{
	if(gAsyncCallBackFunct)
	{
		gAsyncCallBackFunct(dma_async_param);
	}
}

static dma_cookie_t nx_dmac_1902_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct nx_dmac_1902_desc	*desc = txd_to_nx_dmac_1902_desc(tx);
	struct nx_dmac_1902_chan	*nxc = to_nx_dmac_1902_chan(tx->chan);
	dma_cookie_t	cookie;

	dev_vdbg(chan2dev(tx->chan), "tx_submit: chan: %d\n", nxc->chan_num);

	if(!tx->callback)
		tx->callback = nx_dmac_1902_async_tx_callback; 

	spin_lock_bh(&nxc->lock);
	cookie = nx_dmac_1902_assign_cookie(nxc, desc);

	if (list_empty(&nxc->active_list)) {
		dev_vdbg(chan2dev(tx->chan), "tx_submit: chan: %d started %u\n",
				nxc->chan_num, desc->txd.cookie);
		list_add_tail(&desc->desc_node, &nxc->active_list);
		nx_dmac_1902_start_xfer(nxc, desc);		// start transfer after add to list
	} else {
		dev_vdbg(chan2dev(tx->chan), "tx_submit: chan: %d queued %u\n",
				nxc->chan_num, desc->txd.cookie);
		list_add_tail(&desc->desc_node, &nxc->queue);

		if(list_empty(&nxc->queue)){
			dev_vdbg(chan2dev(tx->chan), "tx_submit: queue is empty !! which is just queued???\n");
		}
		else {
			dev_vdbg(chan2dev(tx->chan), "tx_submit: queue is not empty !!thats great\n");
		}
	}
	spin_unlock_bh(&nxc->lock);

	return cookie;
}

static struct dma_async_tx_descriptor * nx_dmac_1902_prep_dma_memcpy( 
		struct dma_chan *chan, 
		dma_addr_t dest, 
		dma_addr_t src,	
		size_t len, 
		unsigned long flags)
{
	struct nx_dmac_1902_chan  *nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902_desc  *desc;
	struct nx_dmac_1902_desc  *first;
	struct nx_dmac_1902_desc  *prev;
	size_t                     xfer_count;
	size_t                     offset;
	u32                        control;
	unsigned int		           src_width;
	unsigned int		           dst_width;
	size_t                     max_size_by_width;

	dev_dbg(chan2dev(chan), "memcpy d0x%x s0x%x l0x%zx f0x%lx\n", dest, src, len, flags);

	if (unlikely(!len)) {
		dev_err(chan2dev(chan), "  memcpy: length is zero!\n");
		return NULL;
	}

	/* 
	 * the information about the slave is specified in the nx_dmac_1902_chan structure. 
	 * So get the following information:
	 nx_dmac_1902_chan->src_per_num; --> always 0
	 nx_dmac_1902_chan->dst_per_num; --> always 0
	 nx_dmac_1902_chan->src_burst; --> caller has to fill it
	 nx_dmac_1902_chan->dst_burst; --> caller has to fill it.
	 nx_dmac_1902_chan->src_incr; --> always 1
	 nx_dmac_1902_chan->dst_incr; --> always 1
	 nx_dmac_1902_chan->flow_cntrl; --> always nx_dmac_1902_fcntl_DMA_M_to_M to be enabled in tx_submit()
	 nx_dmac_1902_chan->src_select; --> caller has to fill it
	 nx_dmac_1902_chan->dst_select; --> caller has to fill it
	 dma_slave->reg_width; --> caller has to fill it 
	 */

	/*
	 * always handled only source_width=dest_width
	 */
#if 0
	dev_vdbg(chan2dev(chan), "  memcpy: src_burst:   %d\n", nxs->src_burst);
	dev_vdbg(chan2dev(chan), "  memcpy: dst_burst:   %d\n", nxs->dst_burst);
	dev_vdbg(chan2dev(chan), "  memcpy: src_select:  %d\n", nxs->src_select);
	dev_vdbg(chan2dev(chan), "  memcpy: dst_select:  %d\n", nxs->dst_select);
	dev_vdbg(chan2dev(chan), "  memcpy: width:       %d\n", nxs->reg_width);
#endif
	/*
	 * We can be a lot more clever here, but this should take care
	 * of the most common optimization.
	 */
	if (!((src | dest  | len) & 3))
		src_width = dst_width = DMA_SLAVE_WIDTH_32BIT;
	else if (!((src | dest | len) & 1))
		src_width = dst_width = DMA_SLAVE_WIDTH_16BIT;
	else
		src_width = dst_width = DMA_SLAVE_WIDTH_8BIT;

	max_size_by_width = (NX_DMAC_1902_MAX_XFER_SIZE * (1<<src_width));

	prev = first = NULL;
	for (offset = 0; offset < len; ) {
		xfer_count = min_t(size_t, (len - offset), max_size_by_width);
		desc = nx_dmac_1902_get_desc(nxc);
		if(!desc)
			goto _err_get_desc;

		/* got the LLI, just fill it */
		desc->lli.src_addr = src + offset;
		desc->lli.dst_addr = dest + offset;
		control = 0;

		control |= NX_DMAC_1902_CHAN_CNTRL_HPROT(NX_DMAC_1902_HPROT_PRIVILEGE_MODE);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_INCR(1);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(1);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_AHB(nx_dmac_1902_ahb_master_1);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(nx_dmac_1902_ahb_master_1);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(dst_width);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(src_width);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_BURST(nx_dmac_1902_burst_256);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(nx_dmac_1902_burst_256);
		control |= (xfer_count/(1<<src_width));

		/* 
		 * the length will always be aligned with reg_width and is always 
		 * divisible by reg_width
		 */
		desc->lli.control = control;

		desc->lli.config = NX_DMAC_1902_UNMASK_INT; /* memory to mem transfer */

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->txd.phys;
			dma_sync_single_for_device(chan2parent(chan),
					prev->txd.phys, sizeof(prev->lli),
					DMA_TO_DEVICE);
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
		offset += xfer_count;

#if 0
		dev_err(chan2dev(&nxc->chan),
				"  desc: src_addr=0x%08x dst_addr=0x%08x control=0x%08x config=0x%08X xfer_count=%d\n",
				desc->lli.src_addr, desc->lli.dst_addr, desc->lli.control, desc->lli.config, xfer_count);
#endif
	}

	/* 
	 * since the caller (dma_async_memcpy_*) functions do not pass 
	 * DMA_PREP_INTERRUPT flag, by default we are enabling the int for 
	 * tc and err
	 */
	prev->lli.control |= NX_DMAC_1902_INT_ENABLE;
	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan),
			prev->txd.phys, sizeof(prev->lli),
			DMA_TO_DEVICE);

	first->txd.flags = flags;
	first->len = len;
	first->txd.callback_param = NULL;	// to fix the SPI DMAC issue
	memcpy(&(first->cbd), &(nxc->cbd), sizeof(struct callback_data));

	return &first->txd;

_err_get_desc:
	dev_dbg(chan2dev(&nxc->chan), "  memcpy: All descriptors are exhausted for channel %d\n", nxc->chan_num);
	nx_dmac_1902_desc_move_to_freelist(nxc, first);
	return NULL;
}

static struct dma_async_tx_descriptor * nx_dmac_1902_prep_dma_memset(
		struct dma_chan *chan,
		dma_addr_t dest,
		int value,
		size_t len,
		unsigned long flags)
{
	struct nx_dmac_1902_chan  *nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902_desc  *desc;
	struct nx_dmac_1902_desc  *first;
	struct nx_dmac_1902_desc  *prev;
	size_t                     xfer_count;
	size_t                     offset;
	u32                        control;
	unsigned int		           dst_width;  
	size_t                     max_size_by_width;

	dev_vdbg( chan2dev(chan), "memset d0x%x l0x%zx f0x%lx v0x%X\n", dest, len, flags, value);

	if (unlikely(!len)) {
		dev_dbg(chan2dev(chan), "  memset: length is zero!\n");
		return NULL;
	}

	/* 
	 * the information about the slave is specified in the nx_dmac_1902_chan 
	 * structure. So get the following information:
	 nx_dmac_1902_chan->src_per_num; --> always 0
	 nx_dmac_1902_chan->dst_per_num; --> always 0
	 nx_dmac_1902_chan->src_burst; --> caller has to fill it
	 nx_dmac_1902_chan->dst_burst; --> caller has to fill it.
	 nx_dmac_1902_chan->src_incr; --> always 0
	 nx_dmac_1902_chan->dst_incr; --> always 1
	 nx_dmac_1902_chan->flow_cntrl; --> always nx_dmac_1902_fcntl_DMA_M_to_M to be enabled in tx_submit()
	 nx_dmac_1902_chan->src_select; --> caller has to fill it
	 nx_dmac_1902_chan->dst_select; --> caller has to fill it
	 dma_slave->reg_width; --> caller has to fill it

	 */

	/*
	 * always handled only source_width=dest_width
	 */
#if 0
	dev_vdbg(chan2dev(chan), "  memset: src_burst:   %d\n", nxs->src_burst);
	dev_vdbg(chan2dev(chan), "  memset: dst_burst:   %d\n", nxs->dst_burst);
	dev_vdbg(chan2dev(chan), "  memset: src_select:  %d\n", nxs->src_select);
	dev_vdbg(chan2dev(chan), "  memset: dst_select:  %d\n", nxs->dst_select);
	dev_vdbg(chan2dev(chan), "  memset: width:       %d\n", nxs->reg_width);
#endif

	/*
	 * We can be a lot more clever here, but this should take care
	 * of the most common optimization.
	 */
	if (!((dest  | len) & 3))
		dst_width = DMA_SLAVE_WIDTH_32BIT;
	else if (!((dest | len) & 1))
		dst_width = DMA_SLAVE_WIDTH_16BIT;
	else
		dst_width = DMA_SLAVE_WIDTH_8BIT;

	max_size_by_width = (NX_DMAC_1902_MAX_XFER_SIZE * (1<<dst_width));

	prev = first = NULL;
	for (offset = 0; offset < len; ) {
		xfer_count = min_t(size_t, (len - offset), max_size_by_width);
		desc = nx_dmac_1902_get_desc(nxc);
		if(!desc)
			goto _err_get_desc;


		/* got the LLI, just fill it */
		desc->lli.src_addr = desc->p_addr_memset;
		desc->lli.val_memset = value;
		desc->lli.dst_addr = dest + offset;
		control = 0;

		control |= NX_DMAC_1902_CHAN_CNTRL_HPROT(NX_DMAC_1902_HPROT_PRIVILEGE_MODE);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_INCR(1);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(0);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_AHB(nx_dmac_1902_ahb_master_1);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(nx_dmac_1902_ahb_master_1);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(dst_width);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(dst_width);
		control |= NX_DMAC_1902_CHAN_CNTRL_DST_BURST(nx_dmac_1902_burst_256);
		control |= NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(nx_dmac_1902_burst_256);
		control |= (xfer_count/(1<<dst_width));

		desc->lli.control = control;
		desc->lli.config = NX_DMAC_1902_UNMASK_INT; /* memory to mem transfer */

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->txd.phys;
			dma_sync_single_for_device(chan2parent(chan),
					prev->txd.phys, sizeof(prev->lli),
					DMA_TO_DEVICE);
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
		offset += xfer_count;
#if 0
		dev_vdbg(chan2dev(&nxc->chan),
				"  desc: src_addr=0x%08x dst_addr=0x%08x control=0x%08x config=0x%08X xfer_count=%d\n",
				desc->lli.src_addr, desc->lli.dst_addr, desc->lli.control, desc->lli.config, xfer_count);
#endif
	}

	/* 
	 * similar to dma_async_memcpy_* functions, we will by default enabling 
	 * the int for tc and err 
	 */

	prev->lli.control |= NX_DMAC_1902_INT_ENABLE;
	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan),
			prev->txd.phys, sizeof(prev->lli),
			DMA_TO_DEVICE);

	first->txd.flags = flags;
	first->len = len;

	return &first->txd;

_err_get_desc:
	dev_dbg(chan2dev(&nxc->chan), "  memset: All descriptors are exhausted for channel %d\n", nxc->chan_num);
	nx_dmac_1902_desc_move_to_freelist(nxc, first);
	return NULL;

}

static struct dma_async_tx_descriptor * nx_dmac_1902_prep_slave_sg(
		struct dma_chan *chan, 
		struct scatterlist *sgl,
		unsigned int sg_len, 
		enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct nx_dmac_1902_chan  *nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902_slave *nxs = chan->private;
	struct nx_dmac_1902_desc  *first;
	struct nx_dmac_1902_desc  *prev;
	size_t                     xfer_count;
	size_t                     offset;
	u32                        control;
	u32                        l_control;
	unsigned int	             i;
	dma_addr_t		             reg;
	struct scatterlist	      *sg;
	size_t                     max_size_by_width;
	u32 len=0;
	u32 mem=0;

	/*
	 * the information about the slave is specified in the nx_dmac_1902_chan 
	 * structure. So get the following information:
	 nx_dmac_1902_chan->src_per_num; --> caller has to fill it
	 nx_dmac_1902_chan->dst_per_num; --> caller has to fill it
	 nx_dmac_1902_chan->src_burst; --> caller has to fill it
	 nx_dmac_1902_chan->dst_burst; --> caller has to fill it.
	 nx_dmac_1902_chan->src_incr; --> caller has to fill it
	 nx_dmac_1902_chan->dst_incr; --> caller has to fill it
	 nx_dmac_1902_chan->flow_cntrl; --> caller has to fill it
	 nx_dmac_1902_chan->src_select; --> caller has to fill it
	 nx_dmac_1902_chan->dst_select; --> caller has to fill it
	 nx_dmac_1902_chan->src_width; --> caller has to fill it
	 nx_dmac_1902_chan->dst_width; --> caller has to fill it
	 dma_slave->tx_reg --> caller has to fill it
	 dma_slave->rx_reg --> caller has to fill it
	 */

	if (unlikely(!nxs || !sg_len)) {
		dev_dbg(chan2dev(chan), "  slave_sg: dma_slave structure is NULL or sg_len is zero!\n");
		return NULL;
	}

	BUG_ON(nxs->flow_cntrl==nx_dmac_1902_fcntl_DMA_M_to_M);

	prev = first = NULL;

#if 0
	dev_vdbg(&chan->dev, "  slave_sg: src_per_num: %d\n", nxs->src_per_num);
	dev_vdbg(&chan->dev, "  slave_sg: dst_per_num: %d\n", nxs->dst_per_num);
	dev_vdbg(&chan->dev, "  slave_sg: src_burst:   %d\n", nxs->src_burst);
	dev_vdbg(&chan->dev, "  slave_sg: dst_burst:   %d\n", nxs->dst_burst);
	dev_vdbg(&chan->dev, "  slave_sg: src_incr:    %d\n", nxs->src_incr);
	dev_vdbg(&chan->dev, "  slave_sg: dst_incr:    %d\n", nxs->dst_incr);
	dev_vdbg(&chan->dev, "  slave_sg: flow_cntrl:  %d\n", nxs->flow_cntrl);
	dev_vdbg(&chan->dev, "  slave_sg: src_select:  %d\n", nxs->src_select);
	dev_vdbg(&chan->dev, "  slave_sg: dst_select:  %d\n", nxs->dst_select);
	dev_vdbg(&chan->dev, "  slave_sg: src_width:   %d\n", nxs->src_width);
	dev_vdbg(&chan->dev, "  slave_sg: dst_width:   %d\n", nxs->dst_width);
#endif

	// map the dma sg page
#ifdef CONFIG_UART_NX_DMAC_1902
	if( direction == DMA_MEM_TO_DEV)
#endif
	{
		sg_len = dma_map_sg(chan2parent(chan), sgl, sg_len, direction);
		if(sg_len == 0) {
			dev_err(chan2dev(chan), "  slave_sg: dma_map_sg is failed\n");
			return NULL;
		}
	}

	control = 0;
	control |= NX_DMAC_1902_CHAN_CNTRL_HPROT(NX_DMAC_1902_HPROT_PRIVILEGE_MODE | NX_DMAC_1902_HPROT_CACHEABLE);
	control |= NX_DMAC_1902_CHAN_CNTRL_DST_INCR(nxs->dst_incr);
	control |= NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(nxs->src_incr);
	control |= NX_DMAC_1902_CHAN_CNTRL_DST_AHB(nxs->dst_select);
	control |= NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(nxs->src_select);
	control |= NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(nxs->dst_width);
	control |= NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(nxs->src_width);
	control |= NX_DMAC_1902_CHAN_CNTRL_DST_BURST(nxs->dst_burst);
	control |= NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(nxs->src_burst);

	/* 
	 * the size of data transfer depends upon the width of the source only.
	 * This is what I understand
	 * and this is applicable only if the DMAC act as flow controller
	 */
	max_size_by_width = (NX_DMAC_1902_MAX_XFER_SIZE * (1<<nxs->src_width));

	switch (direction) {
		case DMA_MEM_TO_DEV:

			BUG_ON(!((nxs->flow_cntrl==nx_dmac_1902_fcntl_DMA_M_to_P) ||( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_M_to_P)) );
			reg = nxs->tx_reg;

			for_each_sg(sgl, sg, sg_len, i) {
				struct nx_dmac_1902_desc  *desc;

				mem = sg_phys(sg);
				len = sg_dma_len(sg);

				if( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_M_to_P) {
					/*
					 * peripheral is the flow controller, hence DMAC does not have 
					 * control over transfer count. making len=1 just to allocate one 
					 * descriptor and break from switch 
					 */
					len = 1;
				}

				for (offset = 0; offset < len; ) {
					xfer_count = min_t(size_t, (len - offset), max_size_by_width);

					desc = nx_dmac_1902_get_desc(nxc);
					if(!desc)
						goto _err_get_desc;

					/* got the LLI, just fill it */
					desc->lli.src_addr = mem + offset;
					desc->lli.dst_addr = reg;
					l_control = control;
					desc->lli.config = 0;

					if( nxs->flow_cntrl == nx_dmac_1902_fcntl_DMA_M_to_P) {
						l_control |= (xfer_count/(1<<nxs->src_width));
						desc->lli.config = NX_DMAC_1902_UNMASK_INT; /* this is valid only if the flow controller is DMAC */
					}
					else if( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_M_to_P) {
						/* 
						 * peripheral is the flow controller, hence DMAC does not have 
						 * control over transfer count 
						 */
					}
					else {
						/* this type of transfer is not supported. ?????? */
					}
					desc->lli.control = l_control;
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_FLOWCNTRL(nxs->flow_cntrl);
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_SRC_PER(0);
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_DST_PER(nxs->dst_per_num);

					if (!first) {
						first = desc;
					} else {
						prev->lli.llp = desc->txd.phys;
						dma_sync_single_for_device(chan2parent(chan),
								prev->txd.phys,
								sizeof(prev->lli),
								DMA_TO_DEVICE);
						list_add_tail(&desc->desc_node,
								&first->tx_list);
					}
					prev = desc;
					offset += xfer_count;
#if 0
					dev_vdbg(chan2dev(&nxc->chan),
							"  desc: src_addr=0x%08x dst_addr=0x%08x control=0x%08x config=0x%08X xfer_count=%d\n",
							desc->lli.src_addr, desc->lli.dst_addr, desc->lli.control, desc->lli.config, xfer_count);
#endif
				}
			}
			break;

		case DMA_DEV_TO_MEM:

			BUG_ON(!((nxs->flow_cntrl==nx_dmac_1902_fcntl_DMA_P_to_M) ||( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_P_to_M)) );

			reg = nxs->rx_reg;

			for_each_sg(sgl, sg, sg_len, i) {
				struct nx_dmac_1902_desc  *desc;

#ifdef CONFIG_UART_NX_DMAC_1902
				mem = sg->dma_address;
				len = sg->dma_length;
#else
				mem = sg_phys(sg);
				len = sg_dma_len(sg);
#endif
				if( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_P_to_M) {
					/* 
					 * peripheral is the flow controller, hence DMAC does not have 
					 * control over transfer count. making len=1 just to allocate one 
					 * descriptor and break from switch 
					 */
					len = 1;
				}

				for (offset = 0; offset < len; ) {
					xfer_count = min_t(size_t, (len - offset), max_size_by_width);

					desc = nx_dmac_1902_get_desc(nxc);
					if(!desc)
						goto _err_get_desc;

					/* got the LLI, just fill it */
					desc->lli.src_addr = reg;
					desc->lli.dst_addr = mem + offset;
					l_control = control;
					desc->lli.config = 0;

					if( nxs->flow_cntrl == nx_dmac_1902_fcntl_DMA_P_to_M) {
						l_control |= (xfer_count/(1<<nxs->src_width));
						desc->lli.config = NX_DMAC_1902_UNMASK_INT; /* this is valid only if the flow controller is DMAC */
					}
					else if( nxs->flow_cntrl == nx_dmac_1902_fcntl_per_P_to_M) {
						/* 
						 * peripheral is the flow controller, hence DMAC does not have 
						 * control over transfer count 
						 */
					}
					else {
						/* this type of transfer is not supported. ?????? */
					}
					desc->lli.control = l_control;
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_FLOWCNTRL(nxs->flow_cntrl);
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_SRC_PER(nxs->src_per_num);
					desc->lli.config |= NX_DMAC_1902_CHAN_CONFIG_DST_PER(0);

					if (!first) {
						first = desc;
					} else {
						prev->lli.llp = desc->txd.phys;
						dma_sync_single_for_device(chan2parent(chan),
								prev->txd.phys,
								sizeof(prev->lli),
								DMA_TO_DEVICE);
						list_add_tail(&desc->desc_node,
								&first->tx_list);
					}
					prev = desc;
					offset += xfer_count;
#if 0          
					dev_vdbg(chan2dev(&nxc->chan),
							"  desc: src_addr=0x%08x dst_addr=0x%08x control=0x%08x config=0x%08X xfer_count=%d\n",
							desc->lli.src_addr, desc->lli.dst_addr, desc->lli.control, desc->lli.config, xfer_count);
#endif

				}
			}
			break;

		default:
			return NULL;
	}

	if (flags & DMA_PREP_INTERRUPT)
		/* Trigger interrupt after last block */
		prev->lli.control |= NX_DMAC_1902_INT_ENABLE;

	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan),
			prev->txd.phys, sizeof(prev->lli),
			DMA_TO_DEVICE);

	first->txd.flags = flags;
	first->len = len;

	return &first->txd;

_err_get_desc:
	dev_dbg(chan2dev(&nxc->chan), "  slave_sg: All descriptors are exhausted for channel %d\n", nxc->chan_num);
	nx_dmac_1902_desc_move_to_freelist(nxc, first);
	return NULL;
}


static void nx_dmac_1902_terminate_all(
		struct dma_chan *chan)
{
	struct nx_dmac_1902_chan	*nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902		*nxd = to_nx_dmac_1902(chan->device);
	struct nx_dmac_1902_desc		*desc, *_desc;
	NX_DMAC_EVENT event;
	LIST_HEAD(list);

	/*
	 * This is only called when something went wrong elsewhere, so
	 * we don't really care about the data. Just disable the
	 * channel. We still have to poll the channel enable bit due
	 * to AHB/HSB limitations.
	 */
	spin_lock_bh(&nxc->lock);

	dev_vdbg(chan2dev(chan),"nx_dmac_1902_terminate_all for channel %d\n", nxc->chan_num);

	/* just forcefully disable the channel. */
	nx_dmac_1902_channel_disable( nxd, nxc->chan_num);

	/* active_list entries will end up before queued entries */
	list_splice_init(&nxc->queue, &list);
	list_splice_init(&nxc->active_list, &list);

	spin_unlock_bh(&nxc->lock);

	/* post the event term */
	event = EVENT_TERM;

	/* Flush all pending and queued descriptors */
	list_for_each_entry_safe(desc, _desc, &list, desc_node)
		nx_dmac_1902_desc_complete(nxc, desc,event);

	return;
}

static int nx_dmac_1902_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
		case DMA_PAUSE:
		case DMA_RESUME:
			return -EINVAL;
		case DMA_TERMINATE_ALL:
			nx_dmac_1902_terminate_all(chan);
			break;
		case DMA_SLAVE_CONFIG:
			//ret = dma_set_runtime_config(chan, (struct dma_slave_config *)arg);
			break;
		default:
			ret = -ENOSYS;
	}

	return ret;
}

static enum dma_status nx_dmac_1902_tx_status(struct dma_chan *chan, dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct nx_dmac_1902_chan	*nxc = to_nx_dmac_1902_chan(chan);
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret != DMA_SUCCESS) {
		nx_dmac_1902_scan_descriptors(to_nx_dmac_1902(chan->device), nxc);
		ret = dma_cookie_status(chan, cookie, txstate);
	}

	return ret;
}

static void nx_dmac_1902_issue_pending(
		struct dma_chan *chan)
{
	struct nx_dmac_1902_chan	*nxc = to_nx_dmac_1902_chan(chan);

	spin_lock_bh(&nxc->lock);

	/* if the active list is empty, then issue the tx list */
	if (!list_empty(&nxc->queue) && list_empty(&nxc->active_list)) {
		nx_dmac_1902_start_xfer(nxc, nx_dmac_1902_first_queued(nxc));
		list_splice_init(&nxc->queue, nxc->active_list.prev); 
	}
	else {

#if 0
		if(list_empty(&nxc->queue))
			dev_vdbg(chan2dev(chan),"nx_dmac_1902_issue_pending: queue list is empty\n");
		else
			dev_vdbg(chan2dev(chan),"nx_dmac_1902_issue_pending: active list contains a transfer\n");
#endif
	}

	spin_unlock_bh(&nxc->lock);

	return;
}

static int nx_dmac_1902_alloc_chan_resources(struct dma_chan *chan)
{
	struct nx_dmac_1902_chan	*nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902		*nxd = to_nx_dmac_1902(chan->device);
	struct nx_dmac_1902_desc	*desc;
	struct nx_dmac_1902_slave	*nxs;
	int			i;

	dev_vdbg(chan2dev(chan), "alloc_chan_resources\n");

	/* ASSERT:  channel is idle */
	if( is_chan_enabled(nxd, nxc->chan_num)) {
		dev_err(chan2dev(chan), "  DMA channel is already enabled\n");
		return -EIO;
	}

	nxc->completed = chan->cookie = 1;
	dma_cookie_init(chan);

	nxs = chan->private;
	if (nxs) {
		/*
		 * We need controller-specific data to set up slave
		 * transfers.
		 */
		BUG_ON(!nxs->dma_dev || nxs->dma_dev != nxd->dma.dev);
	}

	spin_lock_bh(&nxc->lock);
	i = nxc->descs_allocated;
	while (nxc->descs_allocated < NR_DESCS_PER_CHANNEL) {
		spin_unlock_bh(&nxc->lock);

		desc = kzalloc(sizeof(struct nx_dmac_1902_desc), GFP_KERNEL);
		if (!desc) {
			dev_info(chan2dev(chan), "  only allocated %d descriptors\n", i);
			spin_lock_bh(&nxc->lock);
			break;
		}

		INIT_LIST_HEAD(&desc->tx_list);
		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.tx_submit = nx_dmac_1902_tx_submit;
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.callback = nx_dmac_1902_async_tx_callback; 
		desc->txd.phys = dma_map_single(chan2parent(chan), &desc->lli,
				sizeof(desc->lli), DMA_TO_DEVICE);
		desc->p_addr_memset = (desc->txd.phys + sizeof(struct nx_dmac_1902_lli)-4);
		//dev_vdbg(chan2dev(chan), "  Allocated descriptor (virt)=0x%08X lli(v)=0x%08X lli(p)=0x%08X\n", (u32)desc, (u32)&desc->lli, desc->txd.phys);

		nx_dmac_1902_desc_move_to_freelist(nxc, desc);

		spin_lock_bh(&nxc->lock);
		i = ++nxc->descs_allocated;
	}
	spin_unlock_bh(&nxc->lock);

	dev_dbg(chan2dev(chan), "  alloc_chan_resources allocated %d descriptors\n", i);
	return i;

}

static void nx_dmac_1902_free_chan_resources(
		struct dma_chan *chan)
{
	struct nx_dmac_1902_chan *nxc = to_nx_dmac_1902_chan(chan);
	struct nx_dmac_1902		   *nxd = to_nx_dmac_1902(chan->device);
	struct nx_dmac_1902_desc *desc, *_desc;
	LIST_HEAD(list);

	dev_dbg(chan2dev(chan), "free_chan_resources (descs allocated=%u)\n",
			nxc->descs_allocated);

	/* ASSERT:  if transfers are active */ 
	BUG_ON(!list_empty(&nxc->active_list));
	BUG_ON(!list_empty(&nxc->queue));
	BUG_ON(is_chan_enabled(nxd, chan->chan_id));


	spin_lock_bh(&nxc->lock);
	list_splice_init(&nxc->free_list, &list);
	nxc->descs_allocated = 0;

	spin_unlock_bh(&nxc->lock);

	list_for_each_entry_safe(desc, _desc, &list, desc_node) {
		//dev_vdbg(chan2dev(chan), "  freeing descriptor %p\n", desc);
		dma_unmap_single(chan2parent(chan), desc->txd.phys,
				sizeof(desc->lli), DMA_TO_DEVICE);
		kfree(desc);
	}

	dev_vdbg(chan2dev(chan), "free_chan_resources done\n");
}

static int nx_dmac_1902_probe(struct platform_device *pdev)
{
	struct nx_dmac_1902 *nxd;
	struct resource *res_reg;
	void __iomem *ioaddr;
	int irq;
	int i, ret = 0;
	u32 rev, rev0, rev1, rev2, rev3;
	u32 num_channels, num_masters, num_perreqs;

	res_reg = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);

	if (res_reg == NULL || irq < 0)
		return -ENXIO;

	ioaddr = devm_ioremap( &pdev->dev, res_reg->start, (res_reg->end - res_reg->start + 1 ));
	if( !ioaddr )
		return -ENXIO;

	/*
	 * Now detect the IP whether it contains proper vendor id or not
	 */
	rev0 = readl(ioaddr + NX_DMAC_1902_REG_PERID_0);
	rev1 = readl(ioaddr + NX_DMAC_1902_REG_PERID_1);
	rev2 = readl(ioaddr + NX_DMAC_1902_REG_PERID_2);
	rev3 = readl(ioaddr + NX_DMAC_1902_REG_PERID_3);

	rev = ((rev0 & 0xFF) | ((rev1&0xFF)<<8) | ((rev2&0x0F)<<16) );

	if( ( rev != NX_DMAC_1902_MOD_ID ) && ( rev != NX_DMAC_1903_MOD_ID ) ) {
		ret = -ENODEV;
		goto _unmap;
	}

	num_channels = (2<<(rev3 & 0x07));
	num_masters = ((rev3 & 0x08) ? 2:1);
	num_perreqs = ((rev3 & 0x80) ? 32:16);

	dev_vdbg(&pdev->dev,"nx_dmac_1902_probe: num_channels = %d num_masters= %d num_perreqa= %d\n", num_channels, num_masters, num_perreqs);

	nxd = kzalloc( (sizeof(struct nx_dmac_1902) + sizeof(struct nx_dmac_1902_chan) * num_channels ), GFP_KERNEL);
	if( nxd == NULL ) {
		ret = -ENOMEM;
		goto _unmap;
	}

	nxd->regs = ioaddr;
	nxd->num_channels = num_channels;
	nxd->num_masters = num_masters;
	nxd->num_perreqs = num_perreqs;

	ret = request_irq(irq, nx_dmac_1902_isr, 0x00, "nx_1902_dma_irq", nxd);
	if(ret) {
		ret = -ENXIO;
		goto _free_mem;
	}

	tasklet_init(&nxd->tasklet, nx_dmac_1902_tasklet, (unsigned long)nxd);

	platform_set_drvdata(pdev, nxd);

	writel(NX_DMAC_1902_DMAC_DISABLE,ioaddr+NX_DMAC_1902_REG_DMAC_CONFIG); /* disable the DMA controller */

	INIT_LIST_HEAD(&nxd->dma.channels);
	for (i = 0; i < num_channels; i++, nxd->dma.chancnt++) {
		struct nx_dmac_1902_chan	*nxc = &nxd->chan[i];

		nxc->chan.device = &nxd->dma;
		nxc->chan.cookie = 1;
		nxc->chan.chan_id = i;
		list_add_tail(&nxc->chan.device_node, &nxd->dma.channels);

		nxc->ch_regs = ioaddr;
		nxc->chan_num = i;
		spin_lock_init(&nxc->lock);
		INIT_LIST_HEAD(&nxc->active_list);
		INIT_LIST_HEAD(&nxc->queue);
		INIT_LIST_HEAD(&nxc->free_list);

		writel(0x00, ioaddr+NX_DMAC_1902_REG_CHAN_BASE(i)); /* disable the DMA channel */
	}

	writel(((0x01<<num_channels)-1), ioaddr+NX_DMAC_1902_REG_INT_ERR_CLEAR); /* clear the error interrupt status */
	writel(( (0x01<<num_channels)-1), ioaddr+NX_DMAC_1902_REG_INT_TC_CLEAR); /* clear the tc interrupt status */

	dma_cap_set(DMA_MEMCPY, nxd->dma.cap_mask);
	dma_cap_set(DMA_MEMSET, nxd->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, nxd->dma.cap_mask);
	nxd->dma.dev = &pdev->dev;

	nxd->dma.device_alloc_chan_resources = nx_dmac_1902_alloc_chan_resources;
	nxd->dma.device_free_chan_resources = nx_dmac_1902_free_chan_resources;
	nxd->dma.device_prep_dma_memcpy = nx_dmac_1902_prep_dma_memcpy;
	nxd->dma.device_prep_dma_memset = nx_dmac_1902_prep_dma_memset;
	nxd->dma.device_prep_slave_sg = nx_dmac_1902_prep_slave_sg;
	nxd->dma.device_control = nx_dmac_1902_control;
	nxd->dma.device_tx_status = nx_dmac_1902_tx_status;
	nxd->dma.device_issue_pending = nx_dmac_1902_issue_pending;

	writel(NX_DMAC_1902_DMAC_ENABLE, ioaddr+NX_DMAC_1902_REG_DMAC_CONFIG); /* enable the DMA controller */

	if(dma_async_device_register(&nxd->dma)) {
		writel(NX_DMAC_1902_DMAC_DISABLE, ioaddr+NX_DMAC_1902_REG_DMAC_CONFIG); /* disable the DMA controller */
		ret = -ENXIO;
		goto _free_mem;
	}

	dev_info(&pdev->dev, "NXP DMA Controller %s found @ [%08x] (rev %x), %d channels\n",
			((rev == NX_DMAC_1902_MOD_ID) ? "IP_1902" : "IP_1903"),
			(int)ioaddr, (int)rev, num_channels);

	/* save some golobal varible for use of native wrapper */
	nativePdev = pdev;
	return 0;

_free_mem:
	if(nxd) kfree(nxd);
	platform_set_drvdata(pdev, NULL);
_unmap:
	devm_iounmap( &pdev->dev, ioaddr );
	return ret;
}

EXPORT_SYMBOL(nativePdev);
static int nx_dmac_1902_remove(struct platform_device *pdev)
{
	struct nx_dmac_1902 *nxd = platform_get_drvdata(pdev);
	struct nx_dmac_1902_chan *nxc, *_nxc;

	dma_async_device_unregister(&nxd->dma);
	free_irq(nxd->irq_num, nxd);
	tasklet_kill(&nxd->tasklet);

	list_for_each_entry_safe(nxc, _nxc, &nxd->dma.channels,
			chan.device_node) {
		list_del(&nxc->chan.device_node);
		nx_dmac_1902_channel_disable(nxd, nxc->chan_num);
	}
	writel(NX_DMAC_1902_DMAC_DISABLE, nxd->regs+NX_DMAC_1902_REG_DMAC_CONFIG); /* disable the DMA controller */

	devm_iounmap(&pdev->dev, nxd->regs);
	nxd->regs = NULL;

	kfree(nxd);

	return 0;
}


static const struct of_device_id acp_nx_dmac_dt_match[] = {
	{ .compatible = "entr,stb-acpdma" },
	{},
};
MODULE_DEVICE_TABLE(of, acp_nx_dmac_dt_match);

/*!
 * Through this variable, the Linux DMA framework based controller driver for
 * IP_1902 publishes the routines that will be called by the Linux kernel to 
 * probe the device present, remove the device, suspend the operation of the 
 * device and resume the operation of device.
 *
 * This structure will be registered with platform in \a nx_dmac_1902_init()
 * routine.
 *
 */
struct platform_driver nx_dmac_1902_driver = {
	.probe		= nx_dmac_1902_probe,
	.remove		= nx_dmac_1902_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= acp_nx_dmac_dt_match,
	},
};

/*!\fn  int __init nx_dmac_1902_init(void);
 *        This routine will be called when the driver is loaded onto memory or
 *        during the loading of the modules by the kernel during startup.
 *
 * \return Value returned by \a platform_driver_register() routine.
 * 
 * - PseudoCode:
 * - Call \a platform_driver_register() with \a nx_dmac_1902_driver 
 *   variable address.
 * - Return the value returned by \a platform_driver_register() function.
 *        
 */
int __init nx_dmac_1902_init(void)
{
	return platform_driver_register(&nx_dmac_1902_driver);
}

/*!\fn  void __exit nx_dmac_1902_exit(void);
 *        This routine will be called when the driver is being unloaded from 
 *        the memory by kernel.
 *
 * - PseudoCode:
 * - Call \a platform_driver_unregister() with \a nx_dmac_1902_driver 
 *   variable address.
 * - Return from the function.
 *        
 */
void __exit nx_dmac_1902_exit(void)
{
	platform_driver_unregister(&nx_dmac_1902_driver);
}

module_init(nx_dmac_1902_init);
module_exit(nx_dmac_1902_exit);

MODULE_DESCRIPTION("NXP DMA Controller IP_1902 driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP B.V.");

