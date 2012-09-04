/****************************************************************************
 *
 *  arch/arm/mach-nevis/pci.c
 *
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2as published by
 *  the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 *
 ****************************************************************************/
/*$Id: pci.c,v 1.3, 2007-05-22 13:32:05Z, Nitin Garg$
 ****************************************************************************/

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>
#include <asm/arch/dma.h>
#include <asm/arch/platform.h>
#include <asm/arch/intid.h>

#include <asm/mach-types.h>

#ifdef CONFIG_PCI

#define WORD_SIZE               4	/* bytes */

#undef  DEBUG

static u8 pecos_swizzle(struct pci_dev *dev, u8 * pin);
static int pecos_setup(int nr, struct pci_sys_data *sys);
static int pecos_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 * val);
static int pecos_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val);
static void pci_arch_init(void);
/*static void pci_arch_reset ( void );*/

/***********************************************************************
 *  dev->devfn:
 *	     7:3 = slot
 *	     2:0 = function
 ***********************************************************************/
static int pecos_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 * val)
{
	switch (size) {
	case 1:
		{
			__raw_writel((where & 0xFC) | (devfn << PCI_CFG_ADDR_FUNCTION_SHIFT), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			*val = __raw_readb(ASX_TO_VIRT(PCI_CFG_DATA_REG | (where & 3)));

#ifdef DEBUG
			printk(KERN_INFO "CNXT: read_config_8 : addr:0x%08x idsel:0x%02x func:0x%02x data:0x%02x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), *val);
#endif
		}
		break;

	case 2:
		{
			__raw_writel((where & 0xFC) | (devfn << 8), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			*val = __raw_readw(ASX_TO_VIRT(PCI_CFG_DATA_REG | (where & 3)));
#ifdef DEBUG
			printk(KERN_INFO "  CNXT: read_config_16 : addr:0x%08x idsel:0x%02x func:0x%02x data:0x%04x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), *val);
#endif
		}
		break;

	default:
		{
			__raw_writel(where | (devfn << PCI_CFG_ADDR_FUNCTION_SHIFT), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			*val = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
#ifdef DEBUG
			printk(KERN_INFO "  CNXT: read_config_32 : addr:0x%08x idsel:0x%02x func:0x%02x data:0x%08x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), *val);
#endif
		}
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}


static int pecos_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	switch (size) {
	case 1:
		{
#ifdef DEBUG
			printk(KERN_INFO "CNXT: write_config_8 : addr:%08x idsel:%02x func:%02x data:%02x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), val);
#endif
			__raw_writel((where & 0xFC) | (devfn << PCI_CFG_ADDR_FUNCTION_SHIFT), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			__raw_writeb(val, ASX_TO_VIRT(PCI_CFG_DATA_REG | (where & 3)));
		}
		break;

	case 2:
		{
#ifdef DEBUG
			printk(KERN_INFO "CNXT: write_config_16 : addr:%08x idsel:%02x func:%02x data:%04x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), val);
#endif
			__raw_writel((where & 0xFC) | (devfn << PCI_CFG_ADDR_FUNCTION_SHIFT), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			__raw_writew(val, ASX_TO_VIRT(PCI_CFG_DATA_REG | (where & 3)));
		}
		break;

	default:
		{
#ifdef DEBUG
			printk(KERN_INFO "CNXT: write_config_32 : addr:%08x idsel:%02x func:%02x data:%08x\n", where, PCI_SLOT(devfn), PCI_FUNC(devfn), val);
#endif
			__raw_writel(where | (devfn << PCI_CFG_ADDR_FUNCTION_SHIFT), ASX_TO_VIRT(PCI_CFG_ADDR_REG));
			__raw_writel(val, ASX_TO_VIRT(PCI_CFG_DATA_REG));
		}
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops edwards_ops = {
	.read = pecos_read_config,
	.write = pecos_write_config
};

static int __init pecos_setup(int nr, struct pci_sys_data *sys)
{
	struct resource *res;

#ifdef DEBUG
	printk(KERN_INFO "CNXT: pecos_setup called\n");
#endif

	if (nr != 0)
		return 0;

	res = kmalloc(sizeof(struct resource) * 2, GFP_KERNEL);
	if (!res)
		panic("PCI: unable to alloc resources");

	memset(res, 0, sizeof(struct resource) * 2);

	res[0].name	= "CX2427X PCI I/O Bridge",
	res[0].start	= EXT_IO_PHYS_BASE,
	res[0].end	= EXT_IO_PHYS_BASE + PCI_IO_SIZE + -1,
	res[0].flags	= IORESOURCE_IO,
	res[1].name	= "CX2427X PCI MEM Bridge",
	res[1].start	= PCI_MEM_PHYS_BASE, res[1].end =
	    PCI_MEM_PHYS_BASE + PCI_MEM_SIZE - 1, res[1].flags = IORESOURCE_MEM, request_resource(&ioport_resource, &res[0]);

	request_resource(&iomem_resource, &res[1]);

	sys->resource[0] = &res[0];
	sys->resource[1] = &res[1];
	sys->resource[2] = NULL;

	return 1;
}

struct pci_bus *__init pecos_scan_bus(int nr, struct pci_sys_data *sys)
{
#ifdef DEBUG
	printk(KERN_INFO "CNXT: pecos_scan_bus called\n");
#endif

	return pci_scan_bus(sys->busnr, &edwards_ops, sys);
}

/***********************************************************************/

static void __init pci_arch_init(void)
{				/* struct pci_sys_data sysdata  */
	volatile unsigned int v32;

#ifdef DEBUG
	printk(KERN_INFO "CNXT: pci_arch_init:\n");
#endif

	/* Initialize Pecos PCI controller  */

	/* Remap register: 31:29 Memory remapping bits, 15:4 IO remap bits  */
	/*  This gives us a straight-thru mapping i.e. CPU addr == PCI addr */
	__raw_writel(0x8000E100, ASX_TO_VIRT(PCI_REMAP_REG));

	/* On newer chips (Wabash and beyond), need to set the PCI sync bits */
	v32 = __raw_readl(ASX_TO_VIRT(PCI_ROM_MODE_REG));
	__raw_writel((1UL << PCI_ROM_REQ0_SYNC_SHIFT)
		     | (1UL << PCI_ROM_REQ1_SYNC_SHIFT)
		     | (1UL << PCI_ROM_REQ2_SYNC_SHIFT), ASX_TO_VIRT(PCI_ROM_MODE_REG));

	/* Set alt func register so PCI_REQGNT1 is on PIO006-7 instead of normal PCI_REQ pins */
	v32 = __raw_readl(ASX_TO_VIRT(PLL_PIN_ALT_FUNC_REG));
	__raw_writel((v32 & ~(PLL_PIN_ALT_FUNC_PCI_REQGNT2_MASK | PLL_PIN_ALT_FUNC_PCI_REQGNT1_MASK | PLL_PIN_ALT_FUNC_IO_RW_MASK))
		     | PLL_PIN_ALT_FUNC_PCI_REQGNT2_PIO_64_63_5 | PLL_PIN_ALT_FUNC_PCI_REQGNT1_PIO_7_6 | PLL_PIN_ALT_FUNC_IO_RW_NORMAL, ASX_TO_VIRT(PLL_PIN_ALT_FUNC_REG));

	/* Take PCI out of reset */
	__raw_writel(PCI_RESET_DEASSERTED, ASX_TO_VIRT(PCI_RESET_REG));

	/* Enable the bridge mode to forward interrupts */
	__raw_writel(PCI_INTR_INTA_ENABLE_MASK, ASX_TO_VIRT(PCI_INTR_ENABLE_REG));

	/* set Memory Space / Bus Master bits */
	__raw_writel(PCI_CMD_STAT_OFF << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));

	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	__raw_writel(v32 | PCI_CMD_MASTER_ENABLE_MASK | PCI_CMD_MEM_ACCESS_ENABLE_MASK, ASX_TO_VIRT(PCI_CFG_DATA_REG));

#ifdef DEBUG
	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	printk(KERN_INFO "  CNXT: wrote 0x%x to bridge cmd_status\n", v32);
#endif

   /************************************************************************/
   /******** NOTE: further treatment of Pecos Bridge in bios32.c **********/
   /************************************************************************/
	/*
	 * Clear any error conditions
	 */
	__raw_writel(PCI_CMD_STAT_OFF << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));
	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	__raw_writel(v32 | 0xF8000000, ASX_TO_VIRT(PCI_CFG_DATA_REG));

	__raw_writel(PCI_CLS_LT_HT_BIST_OFF << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));
	__raw_writel(0x4000, ASX_TO_VIRT(PCI_CFG_DATA_REG));

	__raw_writel(PCI_BASE_ENABLE_SIZE << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));
	__raw_writel(0x00000095, ASX_TO_VIRT(PCI_CFG_DATA_REG));

#ifdef DEBUG
	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	printk(KERN_INFO "  CNXT: cmd_status - 0x%x\n", v32);
	__raw_writel(PCI_BASE_ENABLE_SIZE << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));
	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	printk(KERN_INFO "  CNXT: base0 remap / size - 0x%x\n", v32);
	__raw_writel(PCI_BASE_REMAP << 2, ASX_TO_VIRT(PCI_CFG_ADDR_REG));
	v32 = __raw_readl(ASX_TO_VIRT(PCI_CFG_DATA_REG));
	printk(KERN_INFO "  CNXT: base1 remap / size - 0x%x\n", v32);
#endif

}

static u8 pecos_swizzle(struct pci_dev *dev, u8 * pin)
{
#ifdef DEBUG
	printk(KERN_INFO "CNXT: pecos_swizzle called\n");
#endif

	return 0;
}

static int __init pci_arch_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
#ifdef DEBUG
	printk(KERN_INFO "CNXT: pci_arch_map_irq called\n");
#endif

	return INT_PCI;
}

struct hw_pci pecos_pci __initdata = {
	.nr_controllers = 1,
	.setup = pecos_setup,
	.scan = pecos_scan_bus,
	.preinit = pci_arch_init,
	.swizzle = pecos_swizzle,
	.map_irq = pci_arch_map_irq
};

/*
static void pci_arch_reset ( void )
{
#ifdef DEBUG
    printk( KERN_INFO "CNXT: pci_arch_reset called\n" );
#endif

    __raw_writel(PCI_RESET_ASSERTED , ASX_TO_VIRT( PCI_RESET_REG ));
    __raw_writel(PCI_RESET_DEASSERTED , ASX_TO_VIRT( PCI_RESET_REG ));

    return;
}
*/

int __init pecos_pci_init(void)
{
#ifdef DEBUG
	printk(KERN_INFO "CNXT: pecos_pci_init called\n");
#endif
	pci_common_init(&pecos_pci);
	return 0;
}

subsys_initcall(pecos_pci_init);
#endif

/****************************************************************************
 * Modifications:
 * $Log:  
 *  4    Linux_SDK 1.3         5/22/07 7:02:05 PM IST Nitin Garg      28041
 *       28042: Changed the PCI memory aperture to 512MB. No need to set the
 *       Base 0 register.
 *  3    Linux_SDK 1.2         5/18/07 2:33:39 PM IST Nitin Garg      27910
 *       27911: Changed PCI Base 0 register to map to 256MB.
 *  2    Linux_SDK 1.1         3/27/07 5:17:41 PM IST Satpal Parmar   GPL
 *       header and starteam footer addition.
 *  1    Linux_SDK 1.0         3/16/07 11:12:29 AM ISTNitin Garg      
 * $
 *
 ****************************************************************************/
