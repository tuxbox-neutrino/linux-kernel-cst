/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author: Srinivas Rao L <srinivas.rao@entropic.com>
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
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/io.h>
#include "sdhci.h"

#define MAX_SLOTS			8
#define NX_SDIOMC_VENDOR_REVISION	(0x75)
#define NX_SDIOMC_VENDOR_REVISION_1	(0x90)
#define NX_SDIOMC_VENDOR_REVISION_2	(0x96)
#define NX_SDIOMC_VENDOR_REVISION_3	(0x99)

#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
#define DUALSD_S1_CD    gpio_get_pin_num[CONFIG_DUALSD_S1_CD_PIN]
#define DUALSD_S1_WP    gpio_get_pin_num[CONFIG_DUALSD_S1_WP_PIN]
#define DUALSD_S2_CD    gpio_get_pin_num[CONFIG_DUALSD_S2_CD_PIN]
#define DUALSD_S2_WP    gpio_get_pin_num[CONFIG_DUALSD_S2_WP_PIN]
#define DUALSD_S2_LINE  gpio_get_pin_num[CONFIG_DUALSD_S2_LINE_PIN]
#ifdef CONFIG_MMC_SDHCI_DUALSD_S2_PWRCNTRL
#define DUALSD_S2_PWR   gpio_get_pin_num[CONFIG_DUALSD_S2_PWR_PIN]
#endif
#endif

struct sdhci_stb_slot {
	/* Chip specific information (IP_3413 wide) */
	struct sdhci_stb_chip	*chip;
	/* SDHCI host structure that points to the instantiation of a MMC device */
	struct sdhci_host	*host;
	/* Slot number */
	int			slot_num;
};

struct sdhci_stb_chip {
	/* platform specific device structure */
	struct platform_device	*pdev;
	/* Any hardware quirks that need to be taken care of */
	unsigned int		quirks;
	/* Number of slots present in the IP_3413 controller */
	int			num_slots;
	/* Chip specific register port */
	void __iomem		*ioaddr;
	/* Slot specific info in the IP_3413 controller */
	struct sdhci_stb_slot	*slots[MAX_SLOTS];
};

static struct sdhci_ops sdhci_stb_ops = {
	.enable_dma	= NULL,
};

#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
static struct sdhci_host *oth_host1, *oth_host2;
#endif

static const struct of_device_id sdhci_stb_dt_match[] = {
	{ .compatible = "entr,stb-sdhci"},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_stb_dt_match);

struct sdhci_stb_slot *sdhci_stb_probe_slot(struct platform_device *pdev,
					    struct sdhci_stb_chip *chip,
					    void __iomem *ioaddr,
					    int irq, int slot_num)
{
	struct sdhci_stb_slot *slot;
	struct sdhci_host *host;
	int ret;

	host = sdhci_alloc_host( &pdev->dev, sizeof(struct sdhci_stb_slot));
	if (IS_ERR(host)) {
		return NULL;
	}

	slot = sdhci_priv(host);

	slot->chip = chip;
	slot->host = host;
	slot->slot_num = slot_num;

	host->hw_name = "IP_3413";
	host->ops = &sdhci_stb_ops;
	host->quirks = chip->quirks;
	/* Even though IP supports DDR modes etc., 
	 * SoC does not support 1.8V signalling
	 */
	host->quirks2 = SDHCI_QUIRK2_NO_1_8_V;
	host->irq = irq;
	host->ioaddr = ioaddr;
	host->flags = (SDHCI_USE_ADMA);
#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
	if(slot_num == 0) {
		host->wp_pin = DUALSD_S1_WP;
		host->cd_pin = DUALSD_S1_CD;
		oth_host1 = host;
	}else {
		host->wp_pin = DUALSD_S2_WP;
		host->cd_pin = DUALSD_S2_CD;
		oth_host2 = host;
	}
	host->vsltn = slot_num+1;
#endif
	ret = sdhci_add_host(host);
	if (ret)
		goto _free_host;

	return slot;

_free_host:
	sdhci_free_host(host);
	return NULL;
}

static void sdhci_stb_remove_slot(struct sdhci_stb_slot *slot)
{
	int dead;
	u32 scratch;

	dead = 0;
	scratch = readl(slot->host->ioaddr + 0x100*slot->slot_num + SDHCI_INT_STATUS);
	if (scratch == (u32)-1)
		dead = 1;

	sdhci_remove_host(slot->host, dead);

	sdhci_free_host(slot->host);
}

static int sdhci_stb_probe(struct platform_device *pdev)
{
	struct sdhci_stb_chip *chip = NULL;
	struct sdhci_stb_slot *slot = NULL;
	const __be32  *prop;
	int num_slots;
	struct resource *res_reg;
	void __iomem *ioaddr;
	int irq;
	int i, ret = 0;
	u16 rev, rev_no;

	prop = of_get_property(pdev->dev.of_node, "nslots", NULL);
	num_slots  = of_read_number(prop, 1);

	if (num_slots == 0) {
		dev_err(&pdev->dev, "no slots\n");
		return -ENXIO;
	}

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
	rev = readw(ioaddr + SDHCI_HOST_VERSION);
	rev_no = (( rev & SDHCI_VENDOR_VER_MASK ) >> SDHCI_VENDOR_VER_SHIFT );
	dev_info(&pdev->dev, "SDHC/MMC controller IP_3413 Version is 0x%x \n",
			rev_no);

	if( (rev_no  != NX_SDIOMC_VENDOR_REVISION_3) && (rev_no  != NX_SDIOMC_VENDOR_REVISION_2)
			&& (rev_no  != NX_SDIOMC_VENDOR_REVISION_1) && (rev_no  != NX_SDIOMC_VENDOR_REVISION ) ) {
		dev_info(&pdev->dev, "SDHC/MMC controller IP_3413 0x%x Version not matched\n",
				rev_no);
		ret = -ENXIO;
		goto _unmap;
	}

	dev_info(&pdev->dev, "SDHC/MMC controller IP_3413 found @ [%08x] (rev %x)\n",
			(int)ioaddr, (int)rev);

	/** allocate memory to chip-specific structure */
	chip = kzalloc(sizeof(struct sdhci_stb_chip), GFP_KERNEL);
	if (chip == NULL) {
		ret = -ENOMEM;
		goto _unmap;
	}

	chip->num_slots = num_slots;
	chip->pdev = pdev;
	chip->ioaddr = ioaddr;
	chip->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
	chip->quirks |= SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC;
	chip->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	chip->quirks |= SDHCI_QUIRK_FORCE_BLK_SZ_2048;;

#ifdef CONFIG_MMC_SDHCI_NX_SDIOMC_SD1_BIT_ALWAYS
	chip->quirks |= SDHCI_QUIRK_FORCE_1_BIT_DATA;
#endif

#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
	chip->quirks |=  SDHCI_QUIRK_NO_CARD_NO_RESET;

	/* SD Slot1 interface */
	gpio_direction_input(DUALSD_S1_CD);
	gpio_direction_input(DUALSD_S1_WP);

	/* SD Slot2 interface */
	gpio_direction_input(DUALSD_S2_CD);
	gpio_direction_input(DUALSD_S2_WP);
	gpio_direction_output(DUALSD_S2_LINE, 0);
#ifdef CONFIG_MMC_SDHCI_DUALSD_S2_PWRCNTRL
	gpio_direction_output(DUALSD_S2_PWR, 1);
#endif
#endif
	platform_set_drvdata(pdev, chip);

	for (i = 0;i < num_slots;i++) {
		slot = sdhci_stb_probe_slot( pdev, chip, ioaddr, irq, i );
		if (IS_ERR(slot)) {
			for (i--;i >= 0;i--)
				sdhci_stb_remove_slot(chip->slots[i]);
			ret = PTR_ERR(slot);
			goto _free_mem;
		}
#ifndef CONFIG_MMC_SDHCI_NX_DUALSD
		ioaddr += 0x100;
#endif
		chip->slots[i] = slot;
	}
#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
	oth_host1->oth_host = oth_host2;
	oth_host2->oth_host = oth_host1;
#endif
	return 0;

_free_mem:
	kfree(chip);
	platform_set_drvdata(pdev, NULL);
_unmap:
	devm_iounmap( &pdev->dev, ioaddr );
	return ret;
}

static int sdhci_stb_remove(struct platform_device *pdev)
{
	int i;
	struct sdhci_stb_chip *chip;

	chip = platform_get_drvdata(pdev);

	if (chip) {
		for (i = 0;i < chip->num_slots; i++)
			sdhci_stb_remove_slot(chip->slots[i]);
		platform_set_drvdata(pdev, NULL);
		devm_iounmap( &pdev->dev, chip->ioaddr);
		kfree(chip);
	}
	return 0;
}

#ifdef CONFIG_PM
static int sdhci_stb_suspend (struct platform_device *pdev, pm_message_t state)
{
	struct sdhci_stb_chip *chip;
	struct sdhci_stb_slot *slot;
	int i, ret;

	chip = platform_get_drvdata(pdev);
	if (!chip)
		return 0;

	for (i = 0;i < chip->num_slots;i++) {
		slot = chip->slots[i];
		if (!slot)
			continue;

		ret = sdhci_suspend_host(slot->host);

		if (ret) {
			for (i--;i >= 0;i--)
				sdhci_resume_host(chip->slots[i]->host);
			return ret;
		}
	}

	/** todo - switch off the power and clock ?? */
	return 0;
}

static int sdhci_stb_resume (struct platform_device *pdev)
{
	struct sdhci_stb_chip *chip;
	struct sdhci_stb_slot *slot;
	int i, ret;

	chip = platform_get_drvdata(pdev);
	if (!chip)
		return 0;

	/** todo - switch on power and clock ?? */
	for (i = 0;i < chip->num_slots;i++) {
		slot = chip->slots[i];
		if (!slot)
			continue;

		ret = sdhci_resume_host(slot->host);
		if (ret)
			return ret;
	}

	return 0;
}

#else /* CONFIG_PM */

#define sdhci_stb_suspend NULL
#define sdhci_stb_resume NULL

#endif /* CONFIG_PM */

struct platform_driver sdhci_stb_driver = {
	.probe		= sdhci_stb_probe,
	.remove		= sdhci_stb_remove,
	.suspend	= sdhci_stb_suspend,
	.resume		= sdhci_stb_resume,
	.driver		= {
		.name		= "stb-sdhci",
		.owner		= THIS_MODULE,
		.of_match_table	= sdhci_stb_dt_match,
	},
};

int __init sdhci_stb_init(void)
{
#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
	int  error;
	error = gpio_request(DUALSD_S1_CD, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of  SD_CD pin returned %d\n", error);
		goto sd_error_1;
	}

	error = gpio_request(DUALSD_S1_WP, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of SD_WP pin returned %d\n", error);
		goto sd_error_2;
	}

	error = gpio_request(DUALSD_S2_CD, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of front SD_CD pin returned %d\n", error);
		goto sd_error_3;
	}

	error = gpio_request(DUALSD_S2_WP, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of front SD_WP pin returned %d\n", error);
		goto sd_error_4;
	}

	error = gpio_request(DUALSD_S2_LINE, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of SD_LINE pin returned %d\n", error);
		goto sd_error_5;
	}
#ifdef CONFIG_MMC_SDHCI_DUALSD_S2_PWRCNTRL
	error = gpio_request(DUALSD_S2_PWR, "nx_sdiomc");
	if (error < 0) {
		printk(KERN_ERR "gpio_request of SD_PWR pin returned %d\n", error);
		goto sd_error_6;
	}
#endif
#endif
	return platform_driver_register(&sdhci_stb_driver);

#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
#ifdef CONFIG_MMC_SDHCI_DUALSD_S2_PWRCNTRL
sd_error_6:
	gpio_free(DUALSD_S2_LINE);
#endif
sd_error_5:
	gpio_free(DUALSD_S2_WP);
sd_error_4:
	gpio_free(DUALSD_S2_CD);
sd_error_3:
	gpio_free(DUALSD_S1_WP);
sd_error_2:
	gpio_free(DUALSD_S1_CD);
sd_error_1:
	return error;
#endif
}

void __exit sdhci_stb_exit(void)
{
#ifdef CONFIG_MMC_SDHCI_NX_DUALSD
	gpio_free(DUALSD_S1_CD);
	gpio_free(DUALSD_S1_WP);
	gpio_free(DUALSD_S2_CD);
	gpio_free(DUALSD_S2_WP);
	gpio_free(DUALSD_S2_LINE);
#ifdef CONFIG_MMC_SDHCI_DUALSD_S2_PWRCNTRL
	gpio_free(DUALSD_S2_PWR);
#endif
#endif
	platform_driver_unregister(&sdhci_stb_driver);
}

module_init(sdhci_stb_init);
module_exit(sdhci_stb_exit);

MODULE_DESCRIPTION("STB SDIO/SD/MMC Controller driver");
MODULE_LICENSE("GPL");
