/*======================================================================

    drivers/mtd/maps/cx2450x-flash.c
    map driver for CFI compliant NOR-flashes connected to an
    Conexant CX2450x SoC

    copyright (C) 2008 Coolstream International Limited

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

======================================================================*/

#include <asm/arch/cx2450x.h>
#include <linux/module.h>
#include <linux/ioport.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#define	VERSION	"1.0"

/* a default Flash map for the case that no map given by the cmdline */
static const struct mtd_partition default_partitions[] = {
	{
	 .name = "Bootloader",
	 .size = 0x00060000,	/* 384K */
	 .offset = 0,
	 .mask_flags = 0	/* to make it readonly */
	 },
	{
	 .name = "Bootloader Splash Image",
	 .size = 0x00020000,	/* 128K */
	 .offset = 0x00060000,
	 .mask_flags = 0	/* to make it readonly */
	 },
	{
	 .name = "Compressed Kernel",
	 .size = 0x00400000,
	 .offset = 0x00080000,
	 .mask_flags = 0	/* to make it readonly */
	 },
	{
	 .name = "systemFS",
	 .size = 0x01B80000,
	 .offset = 0x00480000,
	 .mask_flags = 0}
#if 0
	,
	{
	 .name = "Root Filesystem (JFFS2)",
	 .size = 0x01000000,
	 .offset = 0x01000000,
	 .mask_flags = 0}
#endif
};

#include <asm/arch/platform.h>

inline void enter_flash_program_mode(void)
{
	writel(readl(PCI_ROM_DESC0_REG) | (1UL << 23), PCI_ROM_DESC0_REG);
	writel(readl(PCI_ISAROM_DESC1_REG) | (1UL << 23), PCI_ISAROM_DESC1_REG);
}

inline void exit_flash_program_mode(void)
{
	writel(readl(PCI_ROM_DESC0_REG) & ~(1UL << 23), PCI_ROM_DESC0_REG);
	writel(readl(PCI_ISAROM_DESC1_REG) & ~(1UL << 23), PCI_ISAROM_DESC1_REG);
}

/*****************************************/
/*       CNXT Flash map APIs             */
/*****************************************/

static map_word cnxt_map_read(struct map_info *map, unsigned long ofs)
{
	map_word r;
	r.x[0] = readw(map->virt + ofs);
	return r;
}

static void cnxt_map_write(struct map_info *map, const map_word datum, unsigned long ofs)
{
	enter_flash_program_mode();
	writew(datum.x[0], map->virt + ofs);
	exit_flash_program_mode();
	mb();
	/*printk("[0x%x]<=0x%x\n", map->virt + ofs, datum.x[0]); */
}

static void cnxt_map_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	uint16_t buf;
	unsigned char *to_lval = to;
	while (len > 0) {
		buf = readw(map->virt + (from & 0xFFFFFFFE));
		if (!(from & 1))
			*((unsigned char *) to_lval) = (buf & 0xff);
		else
			*((unsigned char *) to_lval) = ((buf & 0xff00) >> 8);
		to_lval++;
		len--;
		from++;
	}
}

static void cnxt_map_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	uint16_t buf;
	uint16_t *from_lval = (uint16_t *) from;
	enter_flash_program_mode();
	while (len > 0) {
		buf = readw(map->virt + (to & 0xFFFFFFFE));
		if (!(to & 1))
			buf = (buf & 0xff00) | *((unsigned char *) from_lval);
		else
			buf = (buf & 0xff) | (((uint16_t) (*((unsigned char *) from_lval))) << 8);
		writew(buf, map->virt + (to & 0xFFFFFFFE));
		from_lval++;
		len--;
		to++;
	}
	exit_flash_program_mode();
}

void cnxt_map_init(struct map_info *map)
{
	map->read = cnxt_map_read;
	map->write = cnxt_map_write;
	map->copy_from = cnxt_map_copy_from;
	map->copy_to = cnxt_map_copy_to;
	writel(0x1000, PCI_ROM_DESC0_REG2);
}

struct cx2450xflash_info {
	struct resource *res;
	struct map_info map;
	struct mtd_info *mtd;
	struct mtd_partition *parts;
};

static u32 num_banks = 0;
static struct cx2450xflash_info *info = NULL;

/*******************************************************************************/

static void cx2450xflash_set_vpp(struct map_info *map, int on)
{
	volatile u32 *reg;
	u32 bank;

	for (bank = 0; bank < num_banks; bank++) {
		reg = (volatile u32 *) ROM_DESC_REG_BASE(bank);
		if (on)
			*reg |= 0x00800000;
		else
			*reg &= 0xFF7FFFFF;
	}
}

/*******************************************************************************/

static int __init cx2450xflash_init(void)
{
	const char *probes[] = { "cmdlinepart", NULL };
	int bank;
	volatile u32 *reg;
	u32 descreg, mapreg, bwidth;
	u32 fl_size = 0;
	u32 fl_start = 0xF0000000;
	u32 b_start, b_size;
	u32 b_ws0 = 0;
	void __iomem *base = NULL;
	int err = 0;

	printk("CX2450x MTD driver v%s (%s, %s) (c) Coolstream International Ltd.\n", VERSION, __DATE__, __TIME__);

	num_banks = 0;
	info = kzalloc(sizeof(struct cx2450xflash_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* we can have more than one Flash chip (theoretical 4 chips), so read the
	   ISA descriptors, which have to be configured correctly by the bootloader */
	for (bank = 0; bank < 4; bank++) {
		reg = (volatile u32 *) ROM_DESC_REG_BASE(bank);
		descreg = *reg;

		if ((descreg & 0x80000000) == 0) {	/* if bit 31 not set, ROM is present */
			bwidth = (descreg & 0x00000030) >> 4;	/* bit 5..4 encodes the bus width ( 8 or 16 bit) */
			if ((bwidth == 1) || (bwidth == 2)) {
				reg = (volatile u32 *) ROM_MAP_REG_BASE(bank);
				mapreg = *reg;

				b_start = (mapreg & 0x0FFF0000) + 0xF0000000;
				b_size = (0x1000 - (mapreg & 0x00000FFF)) * 0x00010000;
				if ((b_start == fl_start) && (b_size >= 0x00010000)) {
					fl_size += b_size;
					fl_start += b_size;
					if (bank == 0)
						b_ws0 = bwidth;
					num_banks++;
				} else {
					printk("  Start/Size missmatch [0x%.8X / 0x%.8X]\n", b_start, fl_start);
					bank = 4;
				}
			} else
				bank = 4;	/* do not allow non continous range */
		} else
			bank = 4;	/* do not allow non continous range */
	}
	if ((fl_size) && (b_ws0)) {
		fl_start = 0xF0000000;

		base = ioremap(fl_start, fl_size);

		if (!base) {
			kfree(info);
			return -ENOMEM;
		}

		info->res = request_mem_region(fl_start, fl_size, "cx2450xflash");
		if (!info->res) {
			iounmap(base);
			kfree(info);
			return -ENOMEM;
		}

		info->map.size = fl_size;
		info->map.bankwidth = b_ws0;
		info->map.phys = fl_start;
		info->map.virt = base;
		info->map.name = "cx2450xflash";
		info->map.set_vpp = cx2450xflash_set_vpp;

		cnxt_map_init(&info->map);

		/* Note: We must enable VPP here manually, because CFI is not doing this for probe.
		   On the CX2450x device, read's are made in a burst mode with cache. This mode
		   does not allow writings, so the set_vpp() function switch into write-mode, which
		   disabled burst and cache, but in fact we can not read in write mode. I.e. data
		   written as 0x1122334455667788 is read as 0x1122112255665566 in write mode. */

		cx2450xflash_set_vpp(&info->map, 1);
		info->mtd = do_map_probe("cfi_probe", &info->map);
		cx2450xflash_set_vpp(&info->map, 0);

		if (!info->mtd) {
			release_resource(info->res);
			iounmap(base);
			kfree(info->res);
			kfree(info);
			return -ENXIO;
		}

		info->mtd->owner = THIS_MODULE;

		err = parse_mtd_partitions(info->mtd, probes, &info->parts, 0);
		if (err >= 0) {
			if (err == 0) {
				printk("  no partitioning via cmdline present. Using defaults.\n");
				err = add_mtd_partitions(info->mtd, default_partitions, ARRAY_SIZE(default_partitions));

			} else
				err = add_mtd_partitions(info->mtd, info->parts, err);
			if (err)
				printk(KERN_ERR "mtd partition registration failed: %d\n", err);
		}

		/* If we got an error, free all resources. */
		if (err < 0) {
			if (info->mtd) {
				del_mtd_partitions(info->mtd);
				map_destroy(info->mtd);
			}
			kfree(info->parts);
			info->parts = NULL;

			release_resource(info->res);
			iounmap(base);
			kfree(info->res);
			info->res = NULL;

			kfree(info);
			info = NULL;
		}
	}
	return err;
}

/*******************************************************************************/

static void __exit cx2450xflash_exit(void)
{
	if (info) {
		if (info->mtd) {
			del_mtd_partitions(info->mtd);
			map_destroy(info->mtd);
		}
		if (info->parts) {
			kfree(info->parts);
			info->parts = NULL;
		}

		iounmap(info->map.virt);
		release_resource(info->res);

		kfree(info->res);
		kfree(info);
	}
}

/*******************************************************************************/

module_init(cx2450xflash_init);
module_exit(cx2450xflash_exit);

EXPORT_SYMBOL(cnxt_map_init);

MODULE_AUTHOR("Coolstream International Limited Ltd..");
MODULE_DESCRIPTION("Conexant CX2450x CFI MTD map driver");
MODULE_LICENSE("GPL");
