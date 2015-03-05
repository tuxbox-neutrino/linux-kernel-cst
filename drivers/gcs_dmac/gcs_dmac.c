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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <linux/nx_dmac.h>

#define ATTRIBUTE_UNUSED __attribute__ ((__unused__))

volatile bool nor_dma_done = false, sfc_dma_done = false;
volatile bool nor_dma_error = false, sfc_dma_error = false;

/* DMAC device structure */
static struct nx_dmac_t	*nx_dmac=NULL;

/*----------------------------------------------------------------------
 * Internal function prototypes 
 *---------------------------------------------------------------------*/
static irqreturn_t nx_dmac_isr(int irq_no ATTRIBUTE_UNUSED, void *dev_id);

/**
* nx_dmac_probe - DMAC probe function
* @pdev: Platform device structure
*
* Probes the DMAC device & do the initialisation
*/
static int nx_dmac_probe(struct platform_device *pdev)
{
	struct resource *res1, *res2;
	int	i, ret = 0;
	uint32_t	per_id3;
	int	num_chans;
	struct nx_dmac_chan_t *chan;
	uint8_t *lli_ptr;
	dma_addr_t lli_phy;
	
	/* Allocate memory for nand control structure */
	nx_dmac = kzalloc(sizeof(struct nx_dmac_t), GFP_KERNEL);
	if (!nx_dmac) {
		printk(KERN_ERR "nx_dmac: DMAC dev memory alloc \r\n");
		return -ENOMEM;
	}

	/* Get the resource */
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res1) {
		printk(KERN_ERR "nx_dmac: DMAC get res failure \r\n");	
		ret = -EBUSY;
		goto out1_free;
	}

	/* IO remap controller base */
	nx_dmac->dmac_base = devm_ioremap(&pdev->dev, 
								res1->start, res1->end - res1->start + 1);
	if (!nx_dmac->dmac_base) {
		printk(KERN_ERR "nx_dmac: DMAC base devm_ioremap failure \r\n");	
		ret = -ENOMEM;
		goto out1_free;
	}
	
	/* Get interrupt resource */
	res2 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(!res2) {
		printk(KERN_ERR "nx_dmac: DMAC wrong irq resource \r\n");	
		ret = -EBUSY;
		goto out1_free;
	}

	/* Alocate interrupt */
	ret = devm_request_irq (&pdev->dev, res2->start, 
			nx_dmac_isr, IRQF_DISABLED, "gcs_nx_1902_dma", nx_dmac);
	if (ret < 0) {
		printk(KERN_ERR "nx_dmac: DMAC irq alloc failure \r\n");	
		goto out1_free;
	}
	
	/* Store in dev struct */
	dev_set_drvdata(&pdev->dev, nx_dmac);

	/* Get HW info */
	per_id3 = readl(nx_dmac->dmac_base + NX_DMAC_PERI_ID3);

	/* # of channels */
	num_chans = per_id3 & 0x7;
	nx_dmac->num_chans = (0x2 << num_chans);
	
	/* # of AHB masters */
	if(per_id3 & 0x4) {
		nx_dmac->num_ahb_mas = 2;
	}
	else {
		nx_dmac->num_ahb_mas = 1;
	}
	
	/* # of requestors */
	if(per_id3 & 0x80) {
		nx_dmac->num_reqs = 32;
	}
	else {
		nx_dmac->num_reqs = 16;
	}

	mutex_init(&nx_dmac->dmac_lock);
	
	/* Allocate memory for channels */
	nx_dmac->chans = kzalloc(sizeof(struct nx_dmac_chan_t) * 
			nx_dmac->num_chans, GFP_KERNEL);
	if(!nx_dmac->chans) {
		printk(KERN_ERR "nx_dmac: DMAC chan mem alloc failure \r\n");	
		ret = -ENOMEM;
		goto out1_free;
	}

	lli_ptr = dmam_alloc_coherent(&pdev->dev, 
			sizeof(struct nx_dmac_lli_t) * nx_dmac->num_chans * NR_DESCRIPTORS,
			&lli_phy, GFP_KERNEL);

	if (lli_ptr == NULL) {
			dev_err(&pdev->dev, "Unable to allocate DMA descriptors\r\n");
			kfree (nx_dmac);
			return -ENOMEM;
	}

	/*Initialise channel */
	for(i=0; i < nx_dmac->num_chans; i++) {
		chan = &nx_dmac->chans[i];
		chan->chanid = i;
		chan->busy = false;
		chan->lli_ptr = (void *)(lli_ptr + (i * sizeof(struct nx_dmac_lli_t) * NR_DESCRIPTORS));
		chan->lli_phy = lli_phy + (i * sizeof(struct nx_dmac_lli_t) * NR_DESCRIPTORS);
		mutex_init(&chan->chan_lock);
		chan->chan_status = 0;
		init_waitqueue_head(&chan->chan_queue);
	}

	/* Enable DMA HW, AHB mast1, AHB mast2 to little endian */
	writel(0x1, nx_dmac->dmac_base + NX_DMAC_CONFIG);

	printk(KERN_INFO "nx_dmac: Registered with %d channels \r\n", nx_dmac->num_chans);

	return 0;

out1_free:	
	kfree(nx_dmac);

	return ret;
}

/**
* nx_dmac_remove - DMAC remove function 
* @pdev: Platform device structure
*
* Deinitialise the device
*/
static int nx_dmac_remove(struct platform_device *pdev)
{
	struct nx_dmac_t *nxdmac = dev_get_drvdata(&pdev->dev);
	
	/* Disable DMAC HW */
	writel(0x0, nxdmac->dmac_base + NX_DMAC_CONFIG);

	/* Free channel alloc */
	kfree(nxdmac->chans);
	
	/* Free DMAC control */
	kfree(nxdmac);

	return 0;
}

/**
* nx_dmac_tfr - DMAC transfer function 
* @tfr: DMAC transfer request structure
*
* Start the DMAC transfers
*/
int	nx_dmac_tfr(nx_dmac_tfr_t *tfr)
{
	int i, j, chanid;
	uint32_t	ctrl_wrd, chan_config;
	struct nx_dmac_chan_t *chan;
	void __iomem	*chan_start;
	
	if (tfr->num_reqs > NR_DESCRIPTORS) {
			printk(KERN_ERR "Too many transfer requests.\r\n");
			return -EAGAIN;
	}

	/* Acquire DMAC lock */
	mutex_lock(&nx_dmac->dmac_lock);

	/* Get free channel */
	i=0;
	while(i < nx_dmac->num_chans) { 
		if(!(nx_dmac->chans[i].busy)) {
			nx_dmac->chans[i].busy = true;
			break;
		}
		i++;
	}

	/* Release DMAC lock */
	mutex_unlock(&nx_dmac->dmac_lock);
	
	/* Return if channel not free */
	if(i >= nx_dmac->num_chans) {
		printk(KERN_ERR "nx_dmac: DMAC no free chan \r\n");	
		return -EBUSY;	
	}

	/* Acquire channel */
	chan = &nx_dmac->chans[i];
	chanid = chan->chanid;
	mutex_lock(&chan->chan_lock);
	
#if 0
	/* Allocate DMAC LLI array */
	chan->lli_ptr = kmalloc(sizeof(struct nx_dmac_lli_t) * 
			tfr->num_reqs, GFP_DMA | GFP_KERNEL);
	if(!chan->lli_ptr) {
		printk(KERN_ERR "nx_dmac: DMAC LLI mem failure \r\n");	
		mutex_unlock(&chan->chan_lock);
		return -ENOMEM;		
	}
#endif

	/* Initialise lli array */
	for(j=0; j < tfr->num_reqs; j++) {
		chan->lli_ptr[j].src_addr = tfr->req[j].src_addr;
		chan->lli_ptr[j].dst_addr = tfr->req[j].dst_addr;
		
		ctrl_wrd = 0;
		if(j == (tfr->num_reqs - 1)) {
			chan->lli_ptr[j].next_lli = (uint32_t) NULL;
			ctrl_wrd |= (1 << 31);										/* Enable TC interrupt */
		}
		else {
			chan->lli_ptr[j].next_lli = chan->lli_phy + ((j+1) * sizeof(struct nx_dmac_lli_t));
		}
	
		/* Control */
		ctrl_wrd |= tfr->req[j].tfr_size & 0xFFF; /* Transfer size */
		ctrl_wrd |= (tfr->req[j].src_brst << 12); /* Source burst size */
		ctrl_wrd |= (tfr->req[j].dst_brst << 15); /* Destination burst size */
		ctrl_wrd |= (tfr->req[j].src_width << 18); /* Source width - 32 bit */
		ctrl_wrd |= (tfr->req[j].dst_width << 21); /* Destination width - 32 bit */
		ctrl_wrd |= (tfr->req[j].src_inc << 26);	/* Source increment */
		ctrl_wrd |= (tfr->req[j].dst_inc << 27);	/* Destination increment */
		ctrl_wrd |= (tfr->req[j].src_ahb << 24); 	/* Source AHB */
		ctrl_wrd |= (tfr->req[j].dst_ahb << 25); 	/* Destination AHB */
		chan->lli_ptr[j].ctrl = ctrl_wrd;
	}

	/* Enable DMA HW, AHB mast1, AHB mast2 to little endian */
	//writel(0x1, nx_dmac->dmac_base + NX_DMAC_CONFIG);
	
	/* Write the first LLI to HW */
	chan_start = nx_dmac->dmac_base + (NX_DMAC_CHAN0_SRC + (NX_DMAC_CHAN_OFF * chanid));
	writel(chan->lli_ptr[0].src_addr, chan_start);
	writel(chan->lli_ptr[0].dst_addr, chan_start+0x4);
	writel(chan->lli_ptr[0].next_lli, chan_start+0x8);
	writel(chan->lli_ptr[0].ctrl, chan_start+0xC);

	/* Clear the interrupts */
	writel((1<<chanid), nx_dmac->dmac_base + NX_DMAC_INT_TC_CLR);
	writel((1<<chanid), nx_dmac->dmac_base + NX_DMAC_INT_ERR_CLR);

	/* Configure the channel */
	chan_config = 0;
	chan_config |= (tfr->req[0].src_per << 1);	/* Source peripheral number */
	chan_config |= (tfr->req[0].dst_per << 6);	/* Destination peripheral number */
	chan_config |= (tfr->req[0].flowctl << 11);	/* Flow control */
	chan_config |= (1 << 15);			/* Unmask TC interrupt */
	chan_config |= (1 << 14);			/* Unmask Error interrupt */
	chan_config |= 0x1;				/* Enable channel */

	chan->chan_status = 0xABCD;

	/* Write chan config to HW */
	writel(chan_config, chan_start + 0x10);
	
	/* Release the mutex */
	mutex_unlock(&chan->chan_lock);

	return chan->chanid;
}

/**
* nx_dmac_complete - DMAC transfer complete function 
* @chanid: Channel id
*
* Complete the DMAC transfers
*/
int	nx_dmac_tfr_comp(int chanid)
{
	int res;
	struct nx_dmac_chan_t *chan;
	void __iomem	*chan_start;
	
	/* Acquire DMAC lock */
	mutex_lock(&nx_dmac->dmac_lock);
	
	/* Get the free channel */
	chan = &nx_dmac->chans[chanid];
	if(!(chan->busy)) {
		printk(KERN_ERR "nx_dmac: DMAC tfr complete \r\n");
		mutex_unlock(&nx_dmac->dmac_lock);
		return -EIO;
	}
	
	/* Release DMAC lock */
	mutex_unlock(&nx_dmac->dmac_lock);

	/* Acquire channel */
	mutex_lock(&chan->chan_lock);
	
	/* Get the channel start */
	chan_start = nx_dmac->dmac_base + (NX_DMAC_CHAN0_SRC + (NX_DMAC_CHAN_OFF * chanid));
	
	/* Wait for completion */
	wait_event(chan->chan_queue, (((volatile __u32)chan->chan_status) != 0xABCD));
	
	/* status */
	res = chan->chan_status;
	
	/* Disable channel */
	writel(0x0, chan_start + 0x10);
	
	/* Disable DMA HW, AHB mast1, AHB mast2 to little endian */
	//writel(0x0, nx_dmac->dmac_base + NX_DMAC_CONFIG);

	/* Free memory */
	//kfree(chan->lli_ptr);

	/* Free channel */
	chan->busy = false;
	
	/* Release channel */
	mutex_unlock(&chan->chan_lock);

	return res;
}

/**
* nx_dmac_isr - DMAC ISR function 
* @irq_no: IRQ number
* @dev_id: Device ID
*
* Handles the DMAC interrupt events
*/
static irqreturn_t nx_dmac_isr(int irq_no ATTRIBUTE_UNUSED, void *dev_id)
{
	int	chanid;
	uint32_t	int_stat, tc_stat, err_stat;
	struct nx_dmac_chan_t *chan;
	struct nx_dmac_t	*dmac=(struct nx_dmac_t *)dev_id;
	
	/* Read the interrupt status & chan ID */
	int_stat = readl(dmac->dmac_base + NX_DMAC_INT_STATUS);
	
	chanid=0;
	while(chanid < nx_dmac->num_chans) { 
		if(int_stat & (1 << chanid)) {
			/* Check TC interrupt */
			chan = &dmac->chans[chanid];
			tc_stat = readl(dmac->dmac_base + NX_DMAC_INT_TC_STATUS);
			if(tc_stat & (1<<chanid)) {
				chan->chan_status = 0;
				writel((1<<chanid), dmac->dmac_base + NX_DMAC_INT_TC_CLR);
				if (chanid==NX_DMAC_NOR_DMA_CHANNEL_ID)
				{
					nor_dma_done = true;
				}
				else if(chanid==NX_DMAC_SFC_DMA_CHANNEL_ID)
				{
					sfc_dma_done = true;
				}
				else
				{
					wake_up(&chan->chan_queue);
				}
			}
				
			/* Check Error interrupt */
			err_stat = readl(dmac->dmac_base + NX_DMAC_INT_ERR_STATUS);
			if(err_stat & (1<<chanid)) {
				chan->chan_status = -EIO;
				writel((1<<chanid), dmac->dmac_base + NX_DMAC_INT_ERR_CLR);
				if (chanid==NX_DMAC_NOR_DMA_CHANNEL_ID)
				{
					nor_dma_error = true;
					nor_dma_done = true;
				}
				else if(chanid==NX_DMAC_SFC_DMA_CHANNEL_ID)
				{
					sfc_dma_error = true;
					sfc_dma_done = true;
				}
				else
				{
					wake_up(&chan->chan_queue);
				}
			}
		}
		chanid++;
	}
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id nx_gcs_dmac_dt_match[] = {
	{ .compatible = "entr,gcs-dmac" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_gcs_dmac_dt_match);
#endif

/**
* DMAC device registration
*/
static struct platform_driver nx_dmac_driver = {
	.probe    = nx_dmac_probe,
	.remove   = nx_dmac_remove,
	.driver   = {
		.name = "gcs_dma",
		.owner  = THIS_MODULE,
		.of_match_table = nx_gcs_dmac_dt_match,
	},
};

/**
* DMAC module init function
*
* Register the DMAC driver 
*/
static int __init nx_dmac_init(void)
{
	return platform_driver_register(&nx_dmac_driver);
}

/**
* DMAC module exit function
*
* Unregister the DMAC driver
*/
static void __exit nx_dmac_exit(void)
{
	platform_driver_unregister(&nx_dmac_driver);
}


EXPORT_SYMBOL(nx_dmac_tfr);
EXPORT_SYMBOL(nx_dmac_tfr_comp);

arch_initcall(nx_dmac_init);
module_exit(nx_dmac_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP B.V.");
MODULE_DESCRIPTION("Linux DMAC IP_1902 driver");
