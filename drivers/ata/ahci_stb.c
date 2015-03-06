/*
 * Entropic STB AHCI SATA platform driver
 * Copyright 2013 Olliver Schinagl <oliver@schinagl.nl>
 * Copyright 2014 Hans de Goede <hdegoede@redhat.com>
 * Copyright 2014 Bas van Loon <bas@coolstreamtech.com>
 *
 * based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/ahci_platform.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/soc.h>
#include "ahci.h"

enum {
	/* Additional vendor-specific port registers */
	PORT_DMACR		= 0x70, /* DMA Control Register */
	PORT_PHYCR		= 0x78, /* PHY Control Register */
	PORT_PHYSR		= 0x7c, /* PHY Status Register */

	/* Additional STB SATA vendor-specific registers */
	STB_TX_CTRL_REG			= 0x500,
	STB_PLL_RESET_CTRL_REG		= 0x504,
	STB_RX_CTRL_REG			= 0x508,
	STB_MPLL_CTRL_REG		= 0x50c,
	STB_LVL_CTRL_REG		= 0x510,
	STB_CR_CTRL_REG			= 0x514,
	STB_SR_DOUT_REG			= 0x518,
	STB_TXRX_STATUS_REG		= 0x51c,
	STB_LLC_CTRL_REG		= 0x600,
	STB_SATA_MODULE_ID_REG		= 0xffc,
};


#define STB_SATA_BASE		UL(0xE0638000)

/* Level control register */
#define SATA_HOST_RX_CTRL_REG	(SOC_MMIO_SATA_BASE + 0x508)

/* MPLL control Register */
#define SATA_MPLL_CTRL_REG	(SOC_MMIO_SATA_BASE + 0x50C)

/* Level control register */
#define SATA_HOST_LVL_CTRL_REG	(SOC_MMIO_SATA_BASE + 0x510)

/* LLC control register */
#define SATA_HOST_LLC_CTRL_REG	(SOC_MMIO_SATA_BASE + 0x600)

/* SATA host CR Port Control Register */
#define SATA_HOST_CR_CTRL_REG	(SOC_MMIO_SATA_BASE + 0x514)
/* SATA host CR Port Output Register */
#define SATA_HOST_SR_DOUT_REG	(SOC_MMIO_SATA_BASE + 0x518)

/* Global Controller Registers */
#define HOST_CAP              0x00       /* Host Capabilities */
#define HOST_PORTS_IMPL       0x0C       /* Bitmap of implemented ports */

/* Host Capability Bits and Bitmaps */
    /* HOST_CAP_NP (Number of Ports) = bits 0:4
     *   0x00 : 1 Port
     *   0x01 : 2 Ports
     *   0x02 : 3 Ports
     *   ...etc
     * Reset Value = AHSATA_NUM_PORTS-1
     * For Apollo, AHSATA_NUM_PORTS = 2
     */
#define HOST_CAP_NP(np)       ((np) << 0)    /* Number of Ports */
#define HOST_CAP_SXS          (1 << 5)       /* Supports External SATA */
#define HOST_CAP_EMS          (1 << 6)       /* Enclosure Management support */
#define HOST_CAP_CCCS         (1 << 7)       /* Command Completion Coelescing Support */
    /* HOST_CAP_NCS (Number of Command Slots) = bits 8:12
     *   Supports 32 commands slots per Port
     * Reset Value = 0x1F
     */
#define HOST_CAP_NCS(ncs)     ((ncs) << 8)   /* Number of Command Slots */
#define HOST_CAP_PSC          (1 << 13)      /* Partial State Capable */
#define HOST_CAP_SSC          (1 << 14)      /* Slumber State capable */
#define HOST_CAP_PMD          (1 << 15)      /* PIO Multiple DRQ Block */
#define HOST_CAP_PMP          (1 << 17)      /* Port Multiplier support */
#define HOST_CAP_SAM          (1 << 18)      /* Supports AHCI Mode Only */
#define HOST_CAP_SNZO         (1 << 19)      /* Supports Non-Zero DMA Offets */
    /* HOST_CAP_ISS (Interface Speed Support) = bits 20:23
     *   Supports SATA Interface speeds
     *   1.5 Gb/s = 1
     *   3.0 Gb/s = 2
     *   6.0 Gb/s = 3
     * Reset Value = 2
     */
#define HOST_CAP_ISS(iss)     ((iss) << 20)  /* Interface Speed Support */
#define HOST_CAP_CLO          (1 << 24)      /* Command List Override support */
#define HOST_CAP_SAL          (1 << 25)      /* Supports Activity LED */
#define HOST_CAP_ALPM         (1 << 26)      /* Aggressive Link PM support */
#define HOST_CAP_SSS          (1 << 27)      /* Supports Staggered Spin-up */
#define HOST_CAP_SMPS         (1 << 28)      /* Supports Mechanical Presense Switch */
#define HOST_CAP_SNTF         (1 << 29)      /* SNotification register */
#define HOST_CAP_NCQ          (1 << 30)      /* Native Command Queueing */
#define HOST_CAP_64           (1 << 31)      /* PCI DAC (64-bit DMA) support */

/* Port Registers */
#define PORT0_CMD             0x118          /* Port 0 Command Register */
#define PORT1_CMD             0x198          /* Port 1 Command Register */

/* Port Command Bits and Bitmaps */
#define PORT_CMD_ESP          (1 << 21)      /* External SATA Port */

#if 0
/* SATA Clocks */
#define CLK_SATA_HCLK_CTL     0x308
#define CLK_SATA_ASIC0_CTL    0x30C
#define CLK_SATA_ASIC1_CTL    0x310
#define CLK_SATA_RBC0_CTL     0x314
#define CLK_SATA_RBC1_CTL     0x318
#define CLK_SATA_RXOOB_CTL    0x31C
#define CLK_SATA_PMALIVE_CTL  0x320
#endif

static int stb_sata_phy_adjust(void)
{
	int status = 0;
	unsigned rd_val = 0, wr_val = 0;

	/* Refer SATA2 PHY data book version: Early Adopter Edition March 31, 2009 
	 * for SATA PHY adjustments. */
#if defined(CONFIG_ARCH_APOLLO)
	/* Apollo_SoC_PRCR 1133:
	 * To pass OOB-01 (OOB Signal Level Threshold Level Detect) adjust 
	 * Loss of Signal Detector level los_lvl[9:5] to 10111b for SATA2x as 
	 * recommended in PHY databook section 2.3.5.1 Recommended Rx Settings. */
	rd_val	= readl(SATA_HOST_LVL_CTRL_REG);
	wr_val	= rd_val & ~((0x1F)<<5);
	wr_val  |= ((0x17)<<5);
	pr_info("SATA: Adjusting los_lvl (rd_val=0x%x, wr_val=0x%x)\n",
		rd_val, wr_val);
	writel(wr_val, SATA_HOST_LVL_CTRL_REG);

	/* Apollo_SoC_PRCR 1175: 
	 * Partial/Slumber power modes only if receiver terminations are 
	 * enabled by setting rx_term_en0/1 to 1b as recommended in 
	 * PHY databook section 2.3.2.2 Rx Power Controls. */
	rd_val	= readl(SATA_HOST_RX_CTRL_REG);
	wr_val	= rd_val | (1<<1) | (1<<17);
	pr_info("SATA: Adjusting rx_term_en0/1 (rd_val=0x%x, wr_val=0x%x)\n",
		rd_val, wr_val);
	writel(wr_val, SATA_HOST_RX_CTRL_REG);
#elif defined(CONFIG_ARCH_KRONOS)
	/* Following are the SATA PHY recommended settings from databook:
	 * DesignWare Cores SATA2 PHY.
	 * For TSMC 40-nm LP (Wirebond/Flip-Chip) April 21, 2010
	 */

	/* Spread Spectrum enable */
	rd_val	= readl(SATA_MPLL_CTRL_REG);
	wr_val	= rd_val | (1<<5);
	pr_info("SATA: Enable mpll_ss_en (rd_val=0x%x, wr_val=0x%x)\n",
		rd_val, wr_val);
	writel(wr_val, SATA_MPLL_CTRL_REG);

	/* Apollo_SoC_PRCR 1175:.
	 * Partial/Slumber power modes only if receiver terminations are.
	 * enabled by setting rx_term_en0 to 1b as recommended in.
	 * PHY databook section 2.3.2.2 Rx Power Controls. */
	rd_val	= readl(SATA_HOST_RX_CTRL_REG);
	wr_val	= rd_val | (1<<1);
	pr_info("SATA: Enable rx_term_en0 (rd_val=0x%x, wr_val=0x%x)\n",
		rd_val, wr_val);
	writel(wr_val, SATA_HOST_RX_CTRL_REG);

#endif
	return status;
}

/* SATA device registration */
static int ahci_stb_phy_init(struct device *dev, void __iomem *reg_base)
{
	unsigned long capabilities = 0;
	unsigned long lockstat = 0;
	unsigned long greset = 0;
	int status = 0, mask;

	/* Unlock the RGU Lock Status register */
	writel(0x000000F8, RST_LOCKCMD_REG);
	writel(0x0000002B, RST_LOCKCMD_REG);

	/* Unlock the Global Reset registers */
	lockstat = readl(RST_LOCKSTAT_REG);
	lockstat &= ~RST_LOCKSTAT_LOCK;
	writel(lockstat, RST_LOCKSTAT_REG);

	/* Bring SATA out of reset */
	greset = readl(RST_GRESET1_REG);
	greset &= ~RST_GRESET1_SATA;
	writel(greset, RST_GRESET1_REG);

	/* Lock the RGU Lock Status register */
	writel(0x00000000, RST_LOCKCMD_REG);

	/* Adjust SATA PHY */
	status = stb_sata_phy_adjust();
	if (status)
		return status;

	/* Enhance performance of SATA interface by changing 
	 * the AHB transactions to be bufferable and cacheable. */
	mask  = readl(SATA_HOST_LLC_CTRL_REG);
	mask |= 0x40;
	writel(mask, SATA_HOST_LLC_CTRL_REG);

	/* Initialize SATA capabilities. Reset of system sets this register to 0x0. */
	capabilities  = (HOST_CAP_NCQ | HOST_CAP_SNTF | HOST_CAP_ALPM |
			 HOST_CAP_SAL | HOST_CAP_CLO  | HOST_CAP_SAM  |
			 HOST_CAP_PMP | HOST_CAP_PMD  | HOST_CAP_SSC  |
			 HOST_CAP_PSC | HOST_CAP_CCCS | HOST_CAP_SSS  |
			 HOST_CAP_SXS | HOST_CAP_ISS(0x2) | HOST_CAP_NCS(0x1F) |
			 HOST_CAP_NP(0x1));

	writel(capabilities, (SOC_MMIO_SATA_BASE + HOST_CAP));

	/* PI = use 1st port (emulator does not have a PHY on 2nd port) */
#ifdef CONFIG_ARCH_APOLLO
	/* Apollo platform has 2 sata controllers */
	writel(0x00000003, (SOC_MMIO_SATA_BASE + HOST_PORTS_IMPL));
	/* SATA Port 1 - Used for eSATA
	 * ESP (21)
	 */
	writel(PORT_CMD_ESP, (SOC_MMIO_SATA_BASE + PORT0_CMD));

	/* SATA Port 2
	* no HPCP (18), CPD (20), or ESP (21)
	*/
	writel(0x00000000, (SOC_MMIO_SATA_BASE + PORT1_CMD));
#else
	writel(0x00000001,(SOC_MMIO_SATA_BASE + HOST_PORTS_IMPL));
	/* SATA Port 1
	 * no HPCP (18), CPD (20), or ESP (21)
	 */

	writel(0x00000000, (SOC_MMIO_SATA_BASE + PORT0_CMD));
#endif

	return 0;
}

static void ahci_stb_start_engine(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	u32 tmp;
#ifdef CONFIG_ARCH_KRONOS
	struct ata_link *link = &ap->link;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	u32 sstatus, spd;
	int rc;

	/* Following are the SATA PHY recommended settings mentioned in:
	 * DesignWare Cores SATA2 PHY databook
	 * For TSMC 40-nm LP (Wirebond/Flip-Chip) April 21, 2010.
	 * and SOC JIRA at
	 * http://jira-soc.entropic-communications.com/browse/SOCK-3806
	 */
	/* If SCR can be read, use it to determine the current SPD.
	 * If not, use cached value in link->sata_spd.
	 */
	rc = sata_scr_read(link, SCR_STATUS, &sstatus);;
	if (rc == 0 && ata_link_online(link))
		spd = (sstatus >> 4) & 0xf;
	else
		spd = link->sata_spd;

	switch(spd) {
	case 1:
		ata_link_printk(link, KERN_INFO,
				"Setting SATA PHY at 1.5 Gbps\n");

		/* Set recommeded PHY values for SATA 1i,1m */

		/* Default value of TX_CTRL_REG = 0x13A
		 * Setting as per SOCK-3806.
		 */
		writel(0x0000011a, hpriv->mmio + STB_TX_CTRL_REG);

		/* Default value of LVL_CTRL_REG = 0x4246
		 * los_lvl[4:0] = 10000b, tx_lvl[4:0] = 00110b.
		 */
		writel(0x00004206, hpriv->mmio + STB_LVL_CTRL_REG);
		break;
	case 2:
		ata_link_printk(link, KERN_INFO,
				"Setting SATA PHY at 3.0 Gbps\n");

		/* Set recommeded PHY values for SATA 2i,2m */

		/* Default value of TX_CTRL_REG = 0x13A
		 * Setting as per SOCK-3806
		 */
		writel(0x0000012a, hpriv->mmio + STB_TX_CTRL_REG);

		/* Default value of LVL_CTRL_REG = 0x4246
		 * los_lvl[4:0] = 10010b, tx_lvl[4:0] = 10001b.
		 */
		writel(0x00004251, hpriv->mmio + STB_LVL_CTRL_REG);
		break;
	case 3:
		ata_link_printk(link, KERN_INFO,
				"Setting SATA PHY at 6.0 Gbps\n");
		break;
	default:
		ata_link_printk(link, KERN_INFO,
		"SATA PHY offline\n");
		break;
	}
#endif

	/* set DMA RXTS/TXTS */
	tmp = readl(port_mmio + PORT_DMACR);
	tmp &= ~0xFFUL;
	tmp |= 0x77UL;
	writel(tmp, port_mmio + PORT_DMACR);
	wmb();

	ahci_start_engine(ap);
}

static const struct ata_port_info ahci_stb_port_info = {
	AHCI_HFLAGS(AHCI_HFLAG_32BIT_ONLY),
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static int ahci_stb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv;
	int rc;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	hpriv->start_engine = ahci_stb_start_engine;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_stb_phy_init(dev, hpriv->mmio);
	if (rc)
		goto disable_resources;

	rc = ahci_platform_init_host(pdev, hpriv, &ahci_stb_port_info, 0, 0);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

#ifdef CONFIG_PM_SLEEP
int ahci_stb_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	int rc;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_stb_phy_init(dev, hpriv->mmio);
	if (rc)
		goto disable_resources;

	rc = ahci_platform_resume_host(dev);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}
#endif

static SIMPLE_DEV_PM_OPS(ahci_stb_pm_ops, ahci_platform_suspend,
			 ahci_platform_resume);

static const struct of_device_id ahci_stb_of_match[] = {
	{ .compatible = "entr,stb-ahci", },
	{ },
};
MODULE_DEVICE_TABLE(of, ahci_stb_of_match);

static struct platform_driver ahci_stb_driver = {
	.probe = ahci_stb_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = "ahci-stb",
		.owner = THIS_MODULE,
		.of_match_table = ahci_stb_of_match,
		.pm = &ahci_stb_pm_ops,
	},
};
module_platform_driver(ahci_stb_driver);

MODULE_DESCRIPTION("STB AHCI SATA driver");
MODULE_AUTHOR("Bas van Loon <bas@coolstreamtech.com>");
MODULE_LICENSE("GPL");
