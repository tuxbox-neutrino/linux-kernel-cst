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

#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "arcvmac.h"

/* Register access macros */
#define vmac_writel(port, value, reg)	\
	writel(cpu_to_le32(value), (port)->regs + VMAC_##reg)
#define vmac_readl(port, reg)	le32_to_cpu(readl((port)->regs + VMAC_##reg))

static int get_register_map(struct vmac_priv *ap);
static int put_register_map(struct vmac_priv *ap);
static void update_vmac_stats_unlocked(struct net_device *dev);
static int vmac_tx_reclaim_unlocked(struct net_device *dev, bool force);

static unsigned char *read_mac_reg(struct net_device *dev,
				   unsigned char hwaddr[ETH_ALEN])
{
	struct vmac_priv *ap = netdev_priv(dev);
	u32 mac_lo, mac_hi;

	WARN_ON(!hwaddr);
	mac_lo = vmac_readl(ap, ADDRL);
	mac_hi = vmac_readl(ap, ADDRH);

	hwaddr[0] = (mac_lo >> 0) & 0xff;
	hwaddr[1] = (mac_lo >> 8) & 0xff;
	hwaddr[2] = (mac_lo >> 16) & 0xff;
	hwaddr[3] = (mac_lo >> 24) & 0xff;
	hwaddr[4] = (mac_hi >> 0) & 0xff;
	hwaddr[5] = (mac_hi >> 8) & 0xff;
	return hwaddr;
}

static void write_mac_reg(struct net_device *dev, unsigned char* hwaddr)
{
	struct vmac_priv *ap = netdev_priv(dev);
	u32 mac_lo, mac_hi;

	mac_lo = hwaddr[3] << 24 | hwaddr[2] << 16 | hwaddr[1] << 8 |
		hwaddr[0];
	mac_hi = hwaddr[5] << 8 | hwaddr[4];

	vmac_writel(ap, mac_lo, ADDRL);
	vmac_writel(ap, mac_hi, ADDRH);
}

static void vmac_mdio_xmit(struct vmac_priv *ap, u32 val)
{
	init_completion(&ap->mdio_complete);
	vmac_writel(ap, val, MDIO_DATA);
	wait_for_completion(&ap->mdio_complete);
}

static int vmac_mdio_read(struct mii_bus *bus, int phy_id, int phy_reg)
{
	struct vmac_priv *vmac = bus->priv;
	u32 val;

	/* only 5 bits allowed for phy-addr and reg_offset */
	WARN_ON(phy_id & ~0x1f || phy_reg & ~0x1f);

	val = MDIO_BASE | MDIO_OP_READ;
	val |= phy_id << 23 | phy_reg << 18;
	vmac_mdio_xmit(vmac, val);

	val = vmac_readl(vmac, MDIO_DATA);
	return val & MDIO_DATA_MASK;
}

static int vmac_mdio_write(struct mii_bus *bus, int phy_id, int phy_reg,
			   u16 value)
{
	struct vmac_priv *vmac = bus->priv;
	u32 val;

	/* only 5 bits allowed for phy-addr and reg_offset */
	WARN_ON(phy_id & ~0x1f || phy_reg & ~0x1f);

	val = MDIO_BASE | MDIO_OP_WRITE;
	val |= phy_id << 23 | phy_reg << 18;
	val |= (value & MDIO_DATA_MASK);
	vmac_mdio_xmit(vmac, val);

	return 0;
}

static void vmac_handle_link_change(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev = ap->phy_dev;
	unsigned long flags;
	bool report_change = false;

	spin_lock_irqsave(&ap->lock, flags);

	if (phydev->duplex != ap->duplex) {
		u32 tmp;

		tmp = vmac_readl(ap, ENABLE);

		if (phydev->duplex)
			tmp |= ENFL_MASK;
		else
			tmp &= ~ENFL_MASK;

		vmac_writel(ap, tmp, ENABLE);

		ap->duplex = phydev->duplex;
		report_change = true;
	}

	if (phydev->speed != ap->speed) {
		ap->speed = phydev->speed;
		report_change = true;
	}

	if (phydev->link != ap->link) {
		ap->link = phydev->link;
		report_change = true;
	}

	spin_unlock_irqrestore(&ap->lock, flags);

	if (report_change)
		phy_print_status(ap->phy_dev);
}

static int __devinit vmac_mii_probe(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	struct clk *vmac_clk;
	unsigned long clock_rate;
	int phy_addr, err;

	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (ap->mii_bus->phy_map[phy_addr]) {
			phydev = ap->mii_bus->phy_map[phy_addr];
			break;
		}
	}

	if (!phydev) {
		dev_err(&ap->pdev->dev, "no PHY found\n");
		return -ENODEV;
	}

	/* FIXME: add pin_irq, if avail */

	phydev = phy_connect(dev, dev_name(&phydev->dev),
			     &vmac_handle_link_change, 0,
			     PHY_INTERFACE_MODE_MII);

	if (IS_ERR(phydev)) {
		err = PTR_ERR(phydev);
		dev_err(&ap->pdev->dev, "could not attach to PHY %d\n", err);
		goto err_out;
	}

	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->supported |= SUPPORTED_Asym_Pause | SUPPORTED_Pause;

	vmac_clk = clk_get(&ap->pdev->dev, "arcvmac");
	if (IS_ERR(vmac_clk)) {
		err = PTR_ERR(vmac_clk);
		goto err_disconnect;
	}

	clock_rate = clk_get_rate(vmac_clk);
	clk_put(vmac_clk);

	dev_dbg(&ap->pdev->dev, "vmac_clk: %lu Hz\n", clock_rate);

	if (clock_rate < 25000000)
		phydev->supported &= ~(SUPPORTED_100baseT_Half |
				       SUPPORTED_100baseT_Full);

	phydev->advertising = phydev->supported;

	ap->link = 0;
	ap->speed = 0;
	ap->duplex = -1;
	ap->phy_dev = phydev;

	return 0;

err_disconnect:
#if 0
	phy_disconnect(phydev);
#endif
err_out:
	return err;
}

static int __devinit vmac_mii_init(struct vmac_priv *ap)
{
	unsigned long flags;
	int err, i;

	//spin_lock_irqsave(&ap->lock, flags);

	ap->mii_bus = mdiobus_alloc();
	if (!ap->mii_bus)
		return -ENOMEM;

	ap->mii_bus->name = "vmac_mii_bus";
	ap->mii_bus->read = &vmac_mdio_read;
	ap->mii_bus->write = &vmac_mdio_write;

	snprintf(ap->mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);

	ap->mii_bus->priv = ap;

	err = -ENOMEM;
	ap->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!ap->mii_bus->irq)
		goto err_out;

	for (i = 0; i < PHY_MAX_ADDR; i++)
		ap->mii_bus->irq[i] = PHY_POLL;

	//spin_unlock_irqrestore(&ap->lock, flags);

	/* locking: mdio concurrency */

	err = mdiobus_register(ap->mii_bus);
	if (err)
		goto err_out_free_mdio_irq;

	err = vmac_mii_probe(ap->dev);
	if (err)
		goto err_out_unregister_bus;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(ap->mii_bus);
err_out_free_mdio_irq:
	kfree(ap->mii_bus->irq);
err_out:
	mdiobus_free(ap->mii_bus);
	return err;
}

static void vmac_mii_exit_unlocked(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);

	if (ap->phy_dev)
		phy_disconnect(ap->phy_dev);

	mdiobus_unregister(ap->mii_bus);
	kfree(ap->mii_bus->irq);
	mdiobus_free(ap->mii_bus);
}

static int vmacether_get_settings(struct net_device *dev,
				  struct ethtool_cmd *cmd)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev = ap->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int vmacether_set_settings(struct net_device *dev,
				  struct ethtool_cmd *cmd)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev = ap->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static int vmac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev = ap->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	return phy_mii_ioctl(phydev, if_mii(rq), cmd);
}

static void vmacether_get_drvinfo(struct net_device *dev,
				  struct ethtool_drvinfo *info)
{
	struct vmac_priv *ap = netdev_priv(dev);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	snprintf(info->bus_info, sizeof(info->bus_info),
		 "platform 0x%pP", &ap->mem->start);
}

static int update_error_counters_unlocked(struct net_device *dev, int status)
{
	struct vmac_priv *ap = netdev_priv(dev);

	dev_dbg(&ap->pdev->dev, "rx error counter overrun. status = 0x%x\n",
		status);

	/* programming error */
	WARN_ON(status & TXCH_MASK);
	WARN_ON(!(status & (MSER_MASK | RXCR_MASK | RXFR_MASK | RXFL_MASK)));

	if (status & MSER_MASK)
		dev->stats.rx_over_errors += 256; /* ran out of BD */
	if (status & RXCR_MASK)
		dev->stats.rx_crc_errors += 256;
	if (status & RXFR_MASK)
		dev->stats.rx_frame_errors += 256;
	if (status & RXFL_MASK)
		dev->stats.rx_fifo_errors += 256;

	return 0;
}

static void update_tx_errors_unlocked(struct net_device *dev, int status)
{
	struct vmac_priv *ap = netdev_priv(dev);

	if (status & BD_UFLO)
		dev->stats.tx_fifo_errors++;

	if (ap->duplex)
		return;

	/* half duplex flags */
	if (status & BD_LTCL)
		dev->stats.tx_window_errors++;
	if (status & BD_RETRY_CT)
		dev->stats.collisions += (status & BD_RETRY_CT) >> 24;
	if (status & BD_DROP)  /* too many retries */
		dev->stats.tx_aborted_errors++;
	if (status & BD_DEFER)
		dev_vdbg(&ap->pdev->dev, "\"defer to traffic\"\n");
	if (status & BD_CARLOSS)
		dev->stats.tx_carrier_errors++;
}

static int vmac_rx_reclaim_force_unlocked(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	int ct;

	/* locking: no concurrency, runs only during shutdown */
	WARN_ON(!ap->shutdown);

	dev_dbg(&ap->pdev->dev, "need to release %d rx sk_buff\n",
		fifo_used(&ap->rx_ring));

	ct = 0;
	while (!fifo_empty(&ap->rx_ring) && ct++ < ap->rx_ring.size) {
		struct vmac_buffer_desc *desc;
		struct sk_buff *skb;
		int desc_idx;

		desc_idx = ap->rx_ring.tail;
		desc = &ap->rxbd[desc_idx];
		fifo_inc_tail(&ap->rx_ring);

		if (!ap->rx_skbuff[desc_idx]) {
			dev_err(&ap->pdev->dev,
				"non-populated rx_skbuff found %d\n",
				desc_idx);
			continue;
		}

		skb = ap->rx_skbuff[desc_idx];
		ap->rx_skbuff[desc_idx] = NULL;

		dma_unmap_single(&ap->pdev->dev, desc->data, skb->len,
				 DMA_TO_DEVICE);

		dev_kfree_skb(skb);
	}

	if (!fifo_empty(&ap->rx_ring))
		dev_err(&ap->pdev->dev, "failed to reclaim %d rx sk_buff\n",
			fifo_used(&ap->rx_ring));

	return 0;
}

/* Function refills empty buffer descriptors and passes ownership to DMA */
static int vmac_rx_refill_unlocked(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);

	/* locking1: protect from refill_timer
	 * locking2: fct owns area outside rx_ring, head exclusive tail,
	 *	modifies head
	 */

	spin_lock(&ap->refill_lock);

	WARN_ON(fifo_full(&ap->rx_ring));

	while (!fifo_full(&ap->rx_ring)) {
		struct vmac_buffer_desc *desc;
		struct sk_buff *skb;
		dma_addr_t p;
		int desc_idx;

		desc_idx = ap->rx_ring.head;
		desc = &ap->rxbd[desc_idx];

		/* make sure we read the actual descriptor status */
		rmb();

		if (ap->rx_skbuff[desc_idx]) {
			/* dropped packet / buffer chaining */
			fifo_inc_head(&ap->rx_ring);

			/* return to DMA */
			wmb();
			desc->info = cpu_to_le32(BD_DMA_OWN | ap->rx_skb_size);
			continue;
		}

		skb = netdev_alloc_skb_ip_align(dev, ap->rx_skb_size);
		if (!skb) {
			dev_info(&ap->pdev->dev,
				 "failed to allocate rx_skb, skb's left %d\n",
				 fifo_used(&ap->rx_ring));
			break;
		}

		ap->rx_skbuff[desc_idx] = skb;

		p = dma_map_single(&ap->pdev->dev, skb->data, ap->rx_skb_size,
				   DMA_FROM_DEVICE);

		desc->data = p;

		wmb();
		desc->info = cpu_to_le32(BD_DMA_OWN | ap->rx_skb_size);

		fifo_inc_head(&ap->rx_ring);
	}

	spin_unlock(&ap->refill_lock);

	/* If rx ring is still empty, set a timer to try allocating
	 * again at a later time.
	 */
	if (fifo_empty(&ap->rx_ring) && netif_running(dev)) {
		dev_warn(&ap->pdev->dev, "unable to refill rx ring\n");
		ap->refill_timer.expires = jiffies + HZ;
		add_timer(&ap->refill_timer);
	}

	return 0;
}

/* timer callback to defer refill rx queue in case we're OOM */
static void vmac_refill_rx_timer(unsigned long data)
{
	vmac_rx_refill_unlocked((struct net_device *)data);
}

/* merge buffer chaining  */
static struct sk_buff *vmac_merge_rx_buffers(struct net_device *dev,
					     struct vmac_buffer_desc *after,
					     int pkt_len) /* data */
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct sk_buff *first_skb, **tail, *cur_skb;
	struct dma_fifo *rx_ring;
	struct vmac_buffer_desc *desc;

	/* locking1: same as vmac_rx_receive */
	/* desc->info = le32_to_cpu(desc-info) already done */

	first_skb = NULL;
	tail = NULL;
	rx_ring = &ap->rx_ring;
	desc = &ap->rxbd[rx_ring->tail];

	WARN_ON(desc == after);

	pkt_len -= ETH_FCS_LEN; /* this might obsolete 'after' */

	while (desc != after && pkt_len) {
		int buf_len, valid;

		/* desc needs wrapping */
		desc = &ap->rxbd[rx_ring->tail];
		cur_skb = ap->rx_skbuff[rx_ring->tail];
		ap->rx_skbuff[rx_ring->tail] = NULL;
		WARN_ON(!cur_skb);

		dma_unmap_single(&ap->pdev->dev, desc->data, ap->rx_skb_size,
				 DMA_FROM_DEVICE);

		/* do not copy FCS */
		buf_len = desc->info & BD_LEN;
		valid = min(pkt_len, buf_len);
		pkt_len -= valid;

		skb_put(cur_skb, valid);

		if (!first_skb) {
			first_skb = cur_skb;
			tail = &skb_shinfo(first_skb)->frag_list;
		} else {
			first_skb->truesize += cur_skb->truesize;
			first_skb->data_len += valid;
			first_skb->len += valid;
			*tail = cur_skb;
			tail = &cur_skb->next;
		}

		fifo_inc_tail(rx_ring);
	}

#ifdef DEBUG
	if (unlikely(pkt_len != 0))
		dev_err(&ap->pdev->dev, "buffer chaining bytes missing %d\n",
			pkt_len);

	if (desc != after) {
		int buf_len = le32_to_cpu(after->info) & BD_LEN;
		WARN_ON(buf_len > ETH_FCS_LEN);
	}
#endif

	return first_skb;
}

static int vmac_rx_receive(struct net_device *dev, int budget)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct vmac_buffer_desc *first;
	int processed, pkt_len, pkt_err;
	struct dma_fifo lookahead;

	/* true concurrency -> DMA engine running in parallel */
	/* locking1: fct owns rx_ring tail to current DMA read position, alias
	 * 'received packets'. rx_refill owns area outside rx_ring, doesn't
	 * modify tail
	 */

	processed = 0;

	first = NULL;
	pkt_err = pkt_len = 0;

	/* look ahead, till packet complete */
	lookahead = ap->rx_ring;

	do {
		struct vmac_buffer_desc *desc; /* cur_ */
		int desc_idx; /* cur_ */
		struct sk_buff *skb; /* pkt_ */

		desc_idx = lookahead.tail;
		desc = &ap->rxbd[desc_idx];

		/* make sure we read the actual descriptor status */
		rmb();

		/* break if dma ownership belongs to hw */
		if (desc->info & cpu_to_le32(BD_DMA_OWN)) {
			/* safe the dma position */
			ap->dma_rx_head = vmac_readl(ap, MAC_RXRING_HEAD);
			break;
		}

		desc->info = le32_to_cpu(desc->info);
		if (desc->info & BD_FRST) {
			pkt_len = 0;
			pkt_err = 0;

			/* free packets up till current */
			ap->rx_ring.tail = lookahead.tail;
			first = desc;
		}

		fifo_inc_tail(&lookahead);

		/* check bd */
		pkt_len += desc->info & BD_LEN;
		pkt_err |= desc->info & BD_BUFF;

		if (!(desc->info & BD_LAST))
			continue;

		/* received complete packet */
		if (unlikely(pkt_err || !first)) {
			/* recycle buffers */
			ap->rx_ring.tail = lookahead.tail;
			continue;
		}

#ifdef DEBUG
		WARN_ON(!(first->info & BD_FRST) || !(desc->info & BD_LAST));
		WARN_ON(pkt_err);
#endif

		/* -- valid packet -- */

		if (first != desc) {
			skb = vmac_merge_rx_buffers(dev, desc, pkt_len);

			if (!skb) {
				/* kill packet */
				ap->rx_ring.tail = lookahead.tail;
				ap->rx_merge_error++;
				continue;
			}
		} else {
			dma_unmap_single(&ap->pdev->dev, desc->data,
					 ap->rx_skb_size, DMA_FROM_DEVICE);

			skb = ap->rx_skbuff[desc_idx];
			ap->rx_skbuff[desc_idx] = NULL;
			/* desc->data != skb->data => desc->data DMA mapped */

			/* strip FCS */
			skb_put(skb, pkt_len - ETH_FCS_LEN);
		}

		/* free buffers */
		ap->rx_ring.tail = lookahead.tail;

#ifdef DEBUG
		WARN_ON(skb->len != pkt_len - ETH_FCS_LEN);
#endif
		processed++;
		skb->protocol = eth_type_trans(skb, dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += skb->len;
		netif_receive_skb(skb);

	} while (!fifo_empty(&lookahead) && (processed < budget));

	dev_vdbg(&ap->pdev->dev, "processed pkt %d, remaining rx buff %d\n",
		 processed,
		 fifo_used(&ap->rx_ring));

	if (processed || fifo_empty(&ap->rx_ring))
		vmac_rx_refill_unlocked(dev);

	return processed;
}

static void vmac_toggle_irqmask_unlocked(struct net_device *dev, bool enable,
					 int mask)
{
	struct vmac_priv *ap = netdev_priv(dev);
	u32 tmp;

	tmp = vmac_readl(ap, ENABLE);
	if (enable)
		tmp |= mask;
	else
		tmp &= ~mask;
	vmac_writel(ap, tmp, ENABLE);
}

static void vmac_toggle_rxint_unlocked(struct net_device *dev, bool enable)
{
	vmac_toggle_irqmask_unlocked(dev, enable, RXINT_MASK);
}

static void vmac_toggle_txint_unlocked(struct net_device *dev, bool enable)
{
	vmac_toggle_irqmask_unlocked(dev, enable, TXINT_MASK);
}

static int vmac_poll(struct napi_struct *napi, int budget)
{
	struct vmac_priv *ap;
	int rx_work_done;

	ap = container_of(napi, struct vmac_priv, napi);

	vmac_tx_reclaim_unlocked(ap->dev, false);

	rx_work_done = vmac_rx_receive(ap->dev, budget);
	if (rx_work_done >= budget) {
		/* rx queue is not yet empty/clean */
		return rx_work_done;
	}

	/* no more packet in rx/tx queue, remove device from poll queue */
	napi_complete(napi);

	/* clear status, only 1' affect register state */
	vmac_writel(ap, RXINT_MASK | TXINT_MASK, STAT);

	/* reenable IRQ */
	vmac_toggle_rxint_unlocked(ap->dev, true);
	vmac_toggle_txint_unlocked(ap->dev, true);

	return rx_work_done;
}

static irqreturn_t vmac_intr(int irq, void *dev_instance)
{
	struct net_device *dev = dev_instance;
	struct vmac_priv *ap = netdev_priv(dev);
	u32 status;

	spin_lock(&ap->lock);

	status = vmac_readl(ap, STAT);
	vmac_writel(ap, status, STAT);

	if (unlikely(ap->shutdown))
		dev_err(&ap->pdev->dev, "ISR during close\n");

	if (unlikely(!status & (RXINT_MASK|MDIO_MASK|ERR_MASK)))
		dev_err(&ap->pdev->dev, "Spurious IRQ\n");

	if ((status & RXINT_MASK) && (vmac_readl(ap, ENABLE) & RXINT_MASK) &&
	    (ap->dma_rx_head != vmac_readl(ap, MAC_RXRING_HEAD))) {
		vmac_toggle_rxint_unlocked(dev, false);
		napi_schedule(&ap->napi);
	}

	if ((status & TXINT_MASK) && (vmac_readl(ap, ENABLE) & TXINT_MASK)) {
		vmac_toggle_txint_unlocked(dev, false);
		napi_schedule(&ap->napi);
	}

	if (status & MDIO_MASK)
		complete(&ap->mdio_complete);

	if (unlikely(status & ERR_MASK))
		update_error_counters_unlocked(dev, status);

	spin_unlock(&ap->lock);

	return IRQ_HANDLED;
}

static int vmac_tx_reclaim_unlocked(struct net_device *dev, bool force)
{
	struct vmac_priv *ap = netdev_priv(dev);
	int released = 0;

	/* locking1: modifies tx_ring tail, head only during shutdown */
	/* locking2: call with ap->lock held */
	WARN_ON(force && !ap->shutdown);

	/* buffer chaining not used, see vmac_start_xmit */

	while (!fifo_empty(&ap->tx_ring)) {
		struct vmac_buffer_desc *desc;
		struct sk_buff *skb;
		int desc_idx;

		desc_idx = ap->tx_ring.tail;
		desc = &ap->txbd[desc_idx];

		/* ensure other field of the descriptor were not read
		 * before we checked ownership
		 */
		rmb();

		if ((desc->info & cpu_to_le32(BD_DMA_OWN)) && !force)
			break;

		if (desc->info & cpu_to_le32(BD_TX_ERR))
			/* recycle packet, let upper level deal with it */
			update_tx_errors_unlocked(dev, le32_to_cpu(desc->info));

		skb = ap->tx_skbuff[desc_idx];
		ap->tx_skbuff[desc_idx] = NULL;
		WARN_ON(!skb);

		dma_unmap_single(&ap->pdev->dev, desc->data, skb->len,
				 DMA_TO_DEVICE);

		dev_kfree_skb(skb);

		released++;
		fifo_inc_tail(&ap->tx_ring);
	}

	if (unlikely(netif_queue_stopped(dev)) && released)
		netif_wake_queue(dev);

	if (unlikely(force && !fifo_empty(&ap->tx_ring)))
		dev_err(&ap->pdev->dev, "failed to reclaim %d tx sk_buff\n",
			fifo_used(&ap->tx_ring));

	return released;
}

static int vmac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct vmac_buffer_desc *desc;

	/* running under xmit lock */
	/* locking: modifies tx_ring head, tx_reclaim only tail */

	/* no scatter/gatter see features below */
	WARN_ON(skb_shinfo(skb)->nr_frags != 0);
	WARN_ON(skb->len > MAX_TX_BUFFER_LEN);

	if (unlikely(fifo_full(&ap->tx_ring))) {
		netif_stop_queue(dev);
		dev_err(&ap->pdev->dev,
			"xmit called while no tx desc available\n");
		return NETDEV_TX_BUSY;
	}

	if (unlikely(skb->len < ETH_ZLEN)) {
		if (skb_padto(skb, ETH_ZLEN))
			return NETDEV_TX_OK;
		skb_put(skb, ETH_ZLEN - skb->len);
	}

	/* fill descriptor */
	ap->tx_skbuff[ap->tx_ring.head] = skb;
	desc = &ap->txbd[ap->tx_ring.head];
	WARN_ON(desc->info & cpu_to_le32(BD_DMA_OWN));

	desc->data = dma_map_single(&ap->pdev->dev, skb->data, skb->len,
				    DMA_TO_DEVICE);

	/* dma might already be polling */
	wmb();
	desc->info = cpu_to_le32(BD_DMA_OWN | BD_FRST | BD_LAST | skb->len);

	/* kick tx dma, only 1' affect register */
	vmac_writel(ap, TXPL_MASK, STAT);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	fifo_inc_head(&ap->tx_ring);

	/* stop queue if no more desc available */
	if (fifo_full(&ap->tx_ring))
		netif_stop_queue(dev);

	return NETDEV_TX_OK;
}

static int alloc_buffers_unlocked(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	int size, err = -ENOMEM;

	fifo_init(&ap->rx_ring, RX_BDT_LEN);
	fifo_init(&ap->tx_ring, TX_BDT_LEN);

	/* initialize skb list */
	memset(ap->rx_skbuff, 0, sizeof(ap->rx_skbuff));
	memset(ap->tx_skbuff, 0, sizeof(ap->tx_skbuff));

	/* allocate DMA received descriptors */
	size = sizeof(*ap->rxbd) * ap->rx_ring.size;
	ap->rxbd = dma_alloc_coherent(&ap->pdev->dev, size,
				      &ap->rxbd_dma,
				      GFP_KERNEL);
	if (!ap->rxbd)
		goto err_out;

	/* allocate DMA transmit descriptors */
	size = sizeof(*ap->txbd) * ap->tx_ring.size;
	ap->txbd = dma_alloc_coherent(&ap->pdev->dev, size,
				      &ap->txbd_dma,
				      GFP_KERNEL);
	if (!ap->txbd)
		goto err_free_rxbd;

	/* ensure 8-byte aligned */
	WARN_ON(((uintptr_t)ap->txbd & 0x7) || ((uintptr_t)ap->rxbd & 0x7));

	memset(ap->txbd, 0, sizeof(*ap->txbd) * ap->tx_ring.size);
	memset(ap->rxbd, 0, sizeof(*ap->rxbd) * ap->rx_ring.size);

	/* allocate rx skb */
	err = vmac_rx_refill_unlocked(dev);
	if (err)
		goto err_free_txbd;

	return 0;

err_free_txbd:
	dma_free_coherent(&ap->pdev->dev, sizeof(*ap->txbd) * ap->tx_ring.size,
			  ap->txbd, ap->txbd_dma);
err_free_rxbd:
	dma_free_coherent(&ap->pdev->dev, sizeof(*ap->rxbd) * ap->rx_ring.size,
			  ap->rxbd, ap->rxbd_dma);
err_out:
	return err;
}

static int free_buffers_unlocked(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);

	/* free skbuff */
	vmac_tx_reclaim_unlocked(dev, true);
	vmac_rx_reclaim_force_unlocked(dev);

	/* free DMA ring */
	dma_free_coherent(&ap->pdev->dev, sizeof(ap->txbd) * ap->tx_ring.size,
			  ap->txbd, ap->txbd_dma);
	dma_free_coherent(&ap->pdev->dev, sizeof(ap->rxbd) * ap->rx_ring.size,
			  ap->rxbd, ap->rxbd_dma);

	return 0;
}

static int vmac_hw_init(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);

	/* clear IRQ mask */
	vmac_writel(ap, 0, ENABLE);

	/* clear pending IRQ */
	vmac_writel(ap, 0xffffffff, STAT);

	/* Initialize logical address filter */
	vmac_writel(ap, 0x0, LAFL);
	vmac_writel(ap, 0x0, LAFH);

	return 0;
}

static int vmac_open(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	struct phy_device *phydev;
	unsigned long flags;
	u32 mask, ctrl;
	int err = 0;

	/* locking: no concurrency yet */

	if (!ap)
		return -ENODEV;

	//spin_lock_irqsave(&ap->lock, flags);
	ap->shutdown = false;

	vmac_hw_init(dev);

	/* mac address changed? */
	write_mac_reg(dev, dev->dev_addr);

	err = alloc_buffers_unlocked(dev);
	if (err)
		return err;

	/* install DMA ring pointers */
	vmac_writel(ap, ap->rxbd_dma, RXRINGPTR);
	vmac_writel(ap, ap->txbd_dma, TXRINGPTR);

	/* set poll rate to 1 ms */
	vmac_writel(ap, POLLRATE_TIME, POLLRATE);

	/* Set control */
	ctrl = (RX_BDT_LEN << 24) | (TX_BDT_LEN << 16) | TXRN_MASK | RXRN_MASK;
	vmac_writel(ap, ctrl, CONTROL);

	/* make sure we enable napi before rx interrupt  */
	napi_enable(&ap->napi);

	err = request_irq(dev->irq, &vmac_intr, 0, dev->name, dev);
	if (err) {
		dev_err(&ap->pdev->dev, "Unable to request IRQ %d (error %d)\n",
			dev->irq, err);
		goto err_free_buffers;
	}

	/* IRQ mask */
	mask = RXINT_MASK | MDIO_MASK | TXINT_MASK;
	mask |= ERR_MASK | TXCH_MASK | MSER_MASK | RXCR_MASK | RXFR_MASK |
		RXFL_MASK;
	vmac_writel(ap, mask, ENABLE);

	/* enable, after all other bits are set */
	vmac_writel(ap, ctrl | EN_MASK, CONTROL);
	//spin_unlock_irqrestore(&ap->lock, flags);

	/* locking: concurrency */

	netif_start_queue(dev);
	netif_carrier_off(dev);

	/* register the PHY board fixup, if needed */
	err = vmac_mii_init(ap);
	if (err)
		goto err_free_irq;

	/* schedule a link state check */
	phy_start(ap->phy_dev);

	phydev = ap->phy_dev;
	dev_info(&ap->pdev->dev,
		 "PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		 phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
err_free_buffers:
	napi_disable(&ap->napi);
	free_buffers_unlocked(dev);
	return err;
}

static int vmac_close(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	unsigned long flags;
	u32 tmp;

	netif_stop_queue(dev);
	napi_disable(&ap->napi);
	del_timer_sync(&ap->refill_timer);

	/* locking: protect everything, DMA / IRQ / timer */
	spin_lock_irqsave(&ap->lock, flags);

	/* complete running transfer, then stop */
	tmp = vmac_readl(ap, CONTROL);
	tmp &= ~(TXRN_MASK | RXRN_MASK);
	vmac_writel(ap, tmp, CONTROL);

	/* save statistics, before unmapping */
	update_vmac_stats_unlocked(dev);

	/* reenable IRQ, process pending */
	spin_unlock_irqrestore(&ap->lock, flags);

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(20));

	/* shut it down now */
	spin_lock_irqsave(&ap->lock, flags);
	ap->shutdown = true;

	/* disable phy */
	phy_stop(ap->phy_dev);
	vmac_mii_exit_unlocked(dev);
	netif_carrier_off(dev);

	/* disable interrupts */
	vmac_writel(ap, 0, ENABLE);
	free_irq(dev->irq, dev);

	/* turn off vmac */
	vmac_writel(ap, 0, CONTROL);
	/* vmac_reset_hw(vmac) */

	/* locking: concurrency off */
	spin_unlock_irqrestore(&ap->lock, flags);

	free_buffers_unlocked(dev);

	return 0;
}

static void update_vmac_stats_unlocked(struct net_device *dev)
{
	struct net_device_stats *_stats = &dev->stats;
	struct vmac_priv *ap = netdev_priv(dev);
	unsigned int rxfram, rxcrc, rxoflow;
	u32 miss, rxerr;

	/* compare with /proc/net/dev,
	 * see net/core/dev.c:dev_seq_printf_stats */

	if (ap->shutdown)
		/* device already unmapped */
		return;

	/* rx stats */
	rxerr = vmac_readl(ap, RXERR);
	miss = vmac_readl(ap, MISS);

	rxcrc = (rxerr & RXERR_CRC);
	rxfram = (rxerr & RXERR_FRM) >> 8;
	rxoflow = (rxerr & RXERR_OFLO) >> 16;

	_stats->rx_length_errors = 0;
	_stats->rx_over_errors += miss;
	_stats->rx_crc_errors += rxcrc;
	_stats->rx_frame_errors += rxfram;
	_stats->rx_fifo_errors += rxoflow;
	_stats->rx_missed_errors = 0;

	/* TODO check rx_dropped/rx_errors/tx_dropped/tx_errors have not
	 * been updated elsewhere
	 */
	_stats->rx_dropped = _stats->rx_over_errors +
		_stats->rx_fifo_errors +
		ap->rx_merge_error;

	_stats->rx_errors = _stats->rx_length_errors + _stats->rx_crc_errors +
		_stats->rx_frame_errors +
		_stats->rx_missed_errors +
		_stats->rx_dropped;

	/* tx stats */
	_stats->tx_dropped = 0; /* otherwise queue stopped */

	_stats->tx_errors = _stats->tx_aborted_errors +
		_stats->tx_carrier_errors +
		_stats->tx_fifo_errors +
		_stats->tx_heartbeat_errors +
		_stats->tx_window_errors +
		_stats->tx_dropped +
		ap->tx_timeout_error;
}

static struct net_device_stats *vmac_stats(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&ap->lock, flags);
	update_vmac_stats_unlocked(dev);
	spin_unlock_irqrestore(&ap->lock, flags);

	return &dev->stats;
}

static void vmac_tx_timeout(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&ap->lock, flags);

	/* queue did not progress for timeo jiffies */
	WARN_ON(!netif_queue_stopped(dev));
	WARN_ON(!fifo_full(&ap->tx_ring));

	/* TX IRQ lost? */
	status = vmac_readl(ap, STAT);
	if (status & TXINT_MASK) {
		dev_err(&ap->pdev->dev, "lost tx interrupt, IRQ mask %x\n",
			vmac_readl(ap, ENABLE));
		vmac_writel(ap, TXINT_MASK, STAT);
	}

	/* TODO RX/MDIO/ERR as well? */

	vmac_tx_reclaim_unlocked(dev, false);
	if (fifo_full(&ap->tx_ring))
		dev_err(&ap->pdev->dev, "DMA state machine not active\n");

	/* We can accept TX packets again */
	ap->tx_timeout_error++;
	dev->trans_start = jiffies;
	netif_wake_queue(dev);

	spin_unlock_irqrestore(&ap->lock, flags);
}

static void create_multicast_filter(struct net_device *dev,
				    int32_t *bitmask)
{
	char *addrs;
	u32 crc;

	/* locking: done by net_device */

	WARN_ON(netdev_mc_count(dev) == 0);
	WARN_ON(dev->flags & IFF_ALLMULTI);

	bitmask[0] = bitmask[1] = 0;

	{
		struct dev_addr_list *ha;
		netdev_for_each_mc_addr(ha, dev) {
			addrs = ha->da_addr;

			/* skip non-multicast addresses */
			if (!(*addrs & 1))
				continue;

			/* map to 0..63 */
			crc = ether_crc_le(ETH_ALEN, addrs) >> 26;
			bitmask[crc >= 32] |= 1UL << (crc & 31);
		}
	}
}

static void vmac_set_multicast_list(struct net_device *dev)
{
	struct vmac_priv *ap = netdev_priv(dev);
	unsigned long flags;
	u32 bitmask[2], reg;
	bool promisc;

	spin_lock_irqsave(&ap->lock, flags);

	promisc = !!(dev->flags & IFF_PROMISC);
	reg = vmac_readl(ap, ENABLE);
	if (promisc != !!(reg & PROM_MASK)) {
		reg ^= PROM_MASK;
		vmac_writel(ap, reg, ENABLE);
	}

	if (dev->flags & IFF_ALLMULTI)
		memset(bitmask, 1, sizeof(bitmask));
	else if (netdev_mc_count(dev) == 0)
		memset(bitmask, 0, sizeof(bitmask));
	else
		create_multicast_filter(dev, bitmask);

	vmac_writel(ap, bitmask[0], LAFL);
	vmac_writel(ap, bitmask[1], LAFH);

	spin_unlock_irqrestore(&ap->lock, flags);
}

static const struct ethtool_ops vmac_ethtool_ops = {
	.get_settings		= vmacether_get_settings,
	.set_settings		= vmacether_set_settings,
	.get_drvinfo		= vmacether_get_drvinfo,
	.get_link		= ethtool_op_get_link,
};

static const struct net_device_ops vmac_netdev_ops = {
	.ndo_open		= vmac_open,
	.ndo_stop		= vmac_close,
	.ndo_get_stats		= vmac_stats,
	.ndo_start_xmit		= vmac_start_xmit,
	.ndo_do_ioctl		= vmac_ioctl,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_tx_timeout		= vmac_tx_timeout,
	.ndo_set_rx_mode	= vmac_set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

static int get_register_map(struct vmac_priv *ap)
{
	int err;

	err = -EBUSY;
	if (!request_mem_region(ap->mem->start, resource_size(ap->mem),
				DRV_NAME)) {
		dev_err(&ap->pdev->dev, "no memory region available\n");
		return err;
	}

	err = -ENOMEM;
	ap->regs = ioremap(ap->mem->start, resource_size(ap->mem));
	if (!ap->regs) {
		dev_err(&ap->pdev->dev, "failed to map registers, aborting.\n");
		goto err_out_release_mem;
	}

	return 0;

err_out_release_mem:
	release_mem_region(ap->mem->start, resource_size(ap->mem));
	return err;
}

static int put_register_map(struct vmac_priv *ap)
{
	iounmap(ap->regs);
	release_mem_region(ap->mem->start, resource_size(ap->mem));
	return 0;
}

static int __devinit vmac_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct vmac_priv *ap;
	struct resource *mem;
	int err;

	/* locking: no concurrency */

	if (dma_get_mask(&pdev->dev) > DMA_BIT_MASK(32) ||
	    pdev->dev.coherent_dma_mask > DMA_BIT_MASK(32)) {
		dev_err(&pdev->dev,
			"arcvmac supports only 32-bit DMA addresses\n");
		return -ENODEV;
	}

	dev = alloc_etherdev(sizeof(*ap));
	if (!dev) {
		dev_err(&pdev->dev, "etherdev alloc failed, aborting.\n");
		return -ENOMEM;
	}

	ap = netdev_priv(dev);

	err = -ENODEV;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mmio resource defined\n");
		goto err_out;
	}
	ap->mem = mem;

	err = platform_get_irq(pdev, 0);
	if (err < 0) {
		dev_err(&pdev->dev, "no irq found\n");
		goto err_out;
	}
	dev->irq = err;

	spin_lock_init(&ap->lock);

	SET_NETDEV_DEV(dev, &pdev->dev);
	ap->dev = dev;
	ap->pdev = pdev;

	/* init rx timeout (used for oom) */
	init_timer(&ap->refill_timer);
	ap->refill_timer.function = vmac_refill_rx_timer;
	ap->refill_timer.data = (unsigned long)dev;
	spin_lock_init(&ap->refill_lock);

	netif_napi_add(dev, &ap->napi, vmac_poll, 64);
	dev->netdev_ops = &vmac_netdev_ops;
	dev->ethtool_ops = &vmac_ethtool_ops;

	dev->base_addr = (unsigned long)ap->regs;

	/* prevent buffer chaining, favor speed over space */
	ap->rx_skb_size = ETH_FRAME_LEN + VMAC_BUFFER_PAD;

	/* private struct functional */

	/* temporarily map registers to fetch mac addr */
	err = get_register_map(ap);
	if (err)
		goto err_out;

	/* mac address intialize, set vmac_open  */
	read_mac_reg(dev, dev->dev_addr);

	if (!is_valid_ether_addr(dev->dev_addr))
		random_ether_addr(dev->dev_addr);

	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Cannot register net device, aborting.\n");
		goto err_out;
	}

	dev_info(&pdev->dev, "ARC VMAC at 0x%pP irq %d %pM\n", &mem->start,
		 dev->irq, dev->dev_addr);
	platform_set_drvdata(pdev, ap);

	return 0;

err_out:
	free_netdev(dev);
	return err;
}

static int __devexit vmac_remove(struct platform_device *pdev)
{
	struct vmac_priv *ap;

	/* locking: no concurrency */

	ap = platform_get_drvdata(pdev);
	if (!ap) {
		dev_err(&pdev->dev, "vmac_remove no valid dev found\n");
		return 0;
	}

	/* MAC */
	unregister_netdev(ap->dev);

	/* release the memory region */
	put_register_map(ap);

	netif_napi_del(&ap->napi);

	platform_set_drvdata(pdev, NULL);
	free_netdev(ap->dev);
	return 0;
}

static struct platform_driver arcvmac_driver = {
	.probe		= vmac_probe,
	.remove		= __devexit_p(vmac_remove),
	.driver		= {
		.name		= "arcvmac",
	},
};

static int __init vmac_init(void)
{
	return platform_driver_register(&arcvmac_driver);
}

static void __exit vmac_exit(void)
{
	platform_driver_unregister(&arcvmac_driver);
}

module_init(vmac_init);
module_exit(vmac_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ARC VMAC Ethernet driver");
MODULE_AUTHOR("afenkart@gmail.com");
