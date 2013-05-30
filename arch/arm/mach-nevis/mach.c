/***************************************************************************
 * linux/arch/arm/mach-nevis/mach.c
 *
 * Copyright (C) 2011 Coolstream International Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published 
 * by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,51 Franklin St, FIfth Floor, Boston, MA  02111-1307  USA
 *
 ****************************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
/* #include <linux/memblock.h> */
#include <linux/bootmem.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/irq.h>
#include <mach/cx2450x.h>
#include "devices.h"
#include "clock.h"
#include <mach/memmap.h>
#include <mach/platform.h>

#define FALSE 0
#define TRUE 1

/* should be inside some config.h */
#define ATAGLIST_MAX_WORDS		192

/* ToCheck : Should this be in gpi_pecos.h */
#define DRIVE_GPIO_LOW_BANK_CLEAR(b, x)	*(volatile u32 *)(GPI_DRIVE_LOW_REG +  \
						((b)*GPI_BANK_SIZE)) &= (~(1<<(x)))

#define DRIVE_GPIO_HIGH_BANK_SET(b, x)	*(volatile u32 *)(GPI_DRIVE_HIGH_REG +  \
						((b)*GPI_BANK_SIZE)) |= (1<<(x))

#define PERI_PORT_REMAP_REG_VALUE	0xE8000009

/*   **************************************************************  */
/*   * PIT bit definition values (used in hardware config files)  *  */
/*   **************************************************************  */
#define PIT_GEN_CLOCK_SYNC			0x00000001
#define PIT_GEN_ASTB				0x00000002
#define PIT_DATA_BURST_WHEN_INDIC		0x00000001
#define PIT_DATA_MEM_BURST			0x00000002
#define PIT_DATA_LOCAL_READ_CACHE		0x00000004
#define PIT_DATA_WRITE_GATHER			0x00000008
#define PIT_DATA_READ_REORDER			0x00000010
#define PIT_INSTR_BUFFER			0x00000001
#define PIT_INSTR_BUFFER_NONCACHE		0x00000002
#define PIT_INSTR_CONCUR_MEMXFER		0x00000004
#define PIT_INSTR_AUTO_POSTLOAD			0x00000008
#define PIT_INSTR_PGTBL_PARLEL_FETCH		0x00000010
#define PIT_INSTR_PARSER_BUF			0x00002000
#define PIT_INSTR_ROM_BUF			0x00004000
#define PIT_INSTR_MEM_BUF			0x00008000
#define PIT_INSTR_AUTO_BUF_REFETCH		0x00010000
#define PIT_INSTR_PREFETCH_INDICATOR		0x00020000

#define HSX_PIT_GENERAL_REG_DEFAULT		(PIT_GEN_CLOCK_SYNC)
#define HSX_PIT_DATA_REG_DEFAULT		(0)
#define HSX_PIT_INSTR_REG_DEFAULT		(0x00800000 | PIT_INSTR_PREFETCH_INDICATOR | PIT_INSTR_MEM_BUF | PIT_INSTR_CONCUR_MEMXFER | PIT_INSTR_BUFFER)
#define HSX_PIT_GENERAL_REG_DEFAULT_DEBUG	(PIT_GEN_CLOCK_SYNC)
#define HSX_PIT_DATA_REG_DEFAULT_DEBUG		(0)
#define HSX_PIT_INSTR_REG_DEFAULT_DEBUG		(0x00800000)

#define UPPER_PLL_SELECT(x)			(((x) & 0x0F000000UL) >> 24)
#define UPPER_PLL_DIV(x)			(((x) & 0x00FF0000UL) >> 16)
#define LOWER_PLL_SELECT(x)			(((x) & 0x0000F000UL) >> 8)
#define LOWER_PLL_DIV(x)			(((x) & 0x000000FFUL))

#define PLL_POST_DIV(x)				(((x) & 0x00F00000UL) >> 20)
#define PLL_PRE_DIV(x)				(((x) & 0x000F0000UL) >> 16)
#define PLL_XTAL_CLK_BYPASS(x)			(((x) & 0x02000000UL))
#define PLL_FERNUS_DIV_CLK(x)			(((x) & 0x3F000000UL) >> 24)
#define PLL_FERNUS_DIV_2(x)			(((x) & 0x00000001UL))
#define xtal_frequency				60000000UL

/*
 * Removed this from 'setup.c' to get a cleaner patch. There is no need to put this
 * inside a generic file.
 * Conexant MAC address from TAG List
 */
u8 mac_address[6];
EXPORT_SYMBOL(mac_address);

unsigned long ChipID;
EXPORT_SYMBOL(ChipID);

unsigned long ChipRevID;
EXPORT_SYMBOL(ChipRevID);

u32 uKernelAtaglist[ATAGLIST_MAX_WORDS];
EXPORT_SYMBOL(uKernelAtaglist);

unsigned int decarm_shared_start = 0;
unsigned int decarm_shared_size = 0;
unsigned int decarm_code_start = 0;
unsigned int decarm_code_size = 0;
unsigned int arm11_pll; /* For use in mipidle routine */

EXPORT_SYMBOL(decarm_shared_start);
EXPORT_SYMBOL(decarm_shared_size);
EXPORT_SYMBOL(decarm_code_start);
EXPORT_SYMBOL(decarm_code_size);

#if 0 /* keep, we can use this for MMU debugging  */
static int cx2450x_abort(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	printk("Data abort: address = 0x%08lx "
		    "fsr = 0x%03x PC = 0x%08lx LR = 0x%08lx",
		addr, fsr, regs->ARM_pc, regs->ARM_lr);

	/*  If it was an imprecise abort, then we need to correct the
	 *  return address to be _after_ the instruction.
	 */
	if (fsr & (1 << 10))
		regs->ARM_pc += 4;

	return 0;
}
#endif

/*
 * 0x242714F1 / NEVIS: Reading MAC address from ATAG list 
 * WARNING: Linux will NEVER accept this patch as is. We better get the MAC address
 * from the NIC directly. For now it works ;)
 */
static int __init parse_tag_mac_address(const struct tag *tag)
{
	int ic;

	for (ic = 0; ic < 6; ic++)
		mac_address[ic] = (u8) (tag->u.mac.mac_addr[ic]);

	return 0;
}
__tagtable(ATAG_MAC, parse_tag_mac_address);

/*
 * Setup Mapping for Register Space and Flash needed by BSP
 */
static struct map_desc cnxt_io_desc[] = {
	{
		.virtual = ASX_VADDR_BASE,
		.pfn = __phys_to_pfn(ASX_PHYS_BASE),
		.length = ASX_IO_SIZE,
		.type = MT_DEVICE,
	}, {
		.virtual = EXT_IO_VADDR_BASE,
		.pfn = __phys_to_pfn(EXT_IO_PHYS_BASE),
		.length = EXT_IO_SIZE,
		.type = MT_DEVICE,
	}, {
		.virtual = TEMPEST_VADDR_BASE,
		.pfn = __phys_to_pfn(TEMPEST_PHYS_BASE),
		.length = TEMPEST_IO_SIZE,
		.type = MT_DEVICE,
#ifdef CONFIG_PCI
	}, {
		.virtual = PCI_MEM_VADDR_BASE,
		.pfn = __phys_to_pfn(PCI_MEM_PHYS_BASE),
		.length = PCI_MEM_SIZE,
		.type = MT_DEVICE,
#endif
	}, {
		.virtual = APP_VADDR_BASE,
		.pfn = __phys_to_pfn(APP_PHYS_BASE),
		.length = APP_IO_SIZE,
		.type = MT_DEVICE,
	}
};

#define UPPER_PLL_SELECT(x)	(((x) & 0x0F000000UL) >> 24)
#define UPPER_PLL_DIV(x)	(((x) & 0x00FF0000UL) >> 16)
#define LOWER_PLL_SELECT(x)	(((x) & 0x0000F000UL) >> 8)
#define LOWER_PLL_DIV(x)	(((x) & 0x000000FFUL))

#define PLL_POST_DIV(x)		(((x) & 0x00F00000UL) >> 20)
#define PLL_PRE_DIV(x)		(((x) & 0x000F0000UL) >> 16)
#define PLL_XTAL_CLK_BYPASS(x)	(((x) & 0x02000000UL))
#define PLL_FERNUS_DIV_CLK(x)	(((x) & 0x3F000000UL) >> 24)
#define PLL_FERNUS_DIV_2(x)	(((x) & 0x00000001UL))
#define xtal_frequency		60000000UL

static unsigned long long getpll_clk_val(uint32_t pll_select, uint32_t pll_div)
{
	struct pll_reg_data_struct {
		uint32_t intfrac;
		uint32_t ctrl;
	};			/* E0440000 -- E0440034 */

	unsigned long long freq;
	static volatile struct pll_reg_data_struct __iomem *pll_data;

	pll_data = (volatile struct pll_reg_data_struct __iomem *)PLL_BASE;

	if (pll_select > 8) {
		return 0;
	} else if (pll_select > 3) {
		pll_data += pll_select - 1;
	} else {
		pll_data += pll_select;
	}
	/* Freq = (Fin pre_div) * (lpdiv_int + lpdiv_freq/(1<<25) * (1/post_div) */
	if (!PLL_XTAL_CLK_BYPASS(pll_data->ctrl)) {
		printk("Value of Bypass clk not known\n");
		return 0;
	}

	if (pll_select == 7) {
		freq =
		    (unsigned long long)(xtal_frequency *
					 PLL_FERNUS_DIV_CLK(pll_data->intfrac));
		if (PLL_FERNUS_DIV_2(pll_data->intfrac)) {
			freq = freq / 2;
		}
	} else {
		freq = (unsigned long long)pll_data->intfrac;
	}

	freq *= (unsigned long long)xtal_frequency;
	if (pll_select != 7) {
		freq /= (unsigned long long)PLL_PRE_DIV(pll_data->ctrl);
		freq /= (unsigned long long)PLL_POST_DIV(pll_data->ctrl);
	}
	freq /= (unsigned long long)(1 << 25);

	if (freq > 0x00000000FFFFFFFFLL) {
		return 0;
	}

	return (freq / pll_div);
}

/* Get arm11 pll value */
static void cx2450x_getpll_values(void)
{
	static volatile uint32_t __iomem div_mux_ctl1;

	div_mux_ctl1	= *((volatile uint32_t __iomem *)PLL_DIV_MUX_CTRL1_REG);
	arm11_pll	= (unsigned int)getpll_clk_val(UPPER_PLL_SELECT(div_mux_ctl1), UPPER_PLL_DIV(div_mux_ctl1));
}

static void __init cx2450x_map_io(void)
{
	iotable_init(cnxt_io_desc, ARRAY_SIZE(cnxt_io_desc));
}

/* This function is used to get the chipid and chipRevId */

static void __init cnxt_get_chip_rev_id(struct machine_desc *pmach, struct tag *ptag, char **pptr, struct meminfo *pmem)
{
	volatile u32 *pPCICfgAddr;
	volatile u32 *pPCICfgData;
	volatile u32 *pFuse = (volatile u32 *) 0xe0440148;

	iotable_init(cnxt_io_desc, 1);
	//iotable_init(cnxt_io_desc + 3, 1);


	/* Take care of some special cases first. Pecos and Nevis need fixing   */
	/* up for A0 and B0, Nevis needs fixup for C0.                          */
	/* PCI Chip ID | PCI Chip Rev | PBIN Fuse | Build Target | "Real" Chip  */
	/* ===================================================================  */
	/*      X      |       0      |     X     |     Pecos    |   Pecos A0   */
	/*      X      |       0      |     X     |     Nevis    |   Nevis A0   */
	/*   <Pecos>   |      0x10    |     0     |       X      |   Pecos B0   */
	/*   <Pecos>   |      0x10    |     1     |       X      |   Nevis B0   */
	/*   <Nevis>   |      0x10    |     X     |       X      |   Nevis C0   */

	pPCICfgAddr = (volatile u32 *) PCI_CFG_ADDR_REG;
	pPCICfgData = (volatile u32 *) PCI_CFG_DATA_REG;

	/* Read vendor/device ID */
	*pPCICfgAddr = 0x0;
	ChipID = *pPCICfgData;

	/* Read Rev Id */
	*pPCICfgAddr = 0x8;
	ChipRevID = *pPCICfgData & 0xFF;

	/* paraphrased from cnxt_base/drivers/startup/api.s */
	if (ChipRevID < 0x10) {
		/* TODO:  When mach-nevis and mach-pecos get merged, we'll need to 
		   use a KCONFIG switch to decide which chip we are. For Rev A chips
		   we're always going to read "Pecos", but obviously we want to report
		   nevis when we're running on a nevis.  For now, I KNOW I'm a nevis 
		   because I am the file mach-nevis/mach.c */
		ChipID = 0x245014F1;
	} else if (ChipRevID == 0x10) {
		if (ChipID == 0x242714F1) {
			/* We could be a Pecos OR a nevis.  Look at the fuses to tell */
			if (*pFuse == 0x0) {
				/* fuses have not been set, follow the same logic for RevID < 0x10 */
				ChipID = 0x245014F1;
			} else {
				/* Fuses are set so we can differentiate between Pecos and Nevis. */
				if (*pFuse & 0x01000000) {
					ChipID = 0x242714F1;
				} else {
					ChipID = 0x245014F1;
				}
			}
		} else if (ChipID == 0x245014F1) {
			/* If we read a nevis and we thought we were a B0, we're really a C0 */
			ChipRevID = 0x20;
		}
	} else {		/* our chiprev > 0x10 */

		/* do nothing, our ChipID is already correct. What needs to be done
		   with the rev is undefined at the time of this writing. */
	}
#ifdef DEBUG
	printk("cx2450x: ChipID=0x%lx ChipRevID=0x%lx\n", ChipID, ChipRevID);
#endif
}


/* fixup function */
static void __init cx2450x_fixup(struct machine_desc *pmach, struct tag *ptag, char **pptr, struct meminfo *pmem)
{
	/* copy the ataglist to the global variable uKernelAtaglist */
	memcpy(uKernelAtaglist, ptag, ATAGLIST_MAX_WORDS * 4);

	cnxt_get_chip_rev_id(pmach, ptag, pptr, pmem);
}

unsigned int GetChipRev(void)
{
	return ChipRevID;
}
EXPORT_SYMBOL(GetChipRev);

unsigned int GetChipID(void)
{
	return ChipID;
}
EXPORT_SYMBOL(GetChipID);

struct cx2450x_pll_config {
	volatile u32 *preg;
	unsigned long val;
};

#define PLLCONF(__reg, __val)				\
	{						\
		.preg	= (volatile u32 *)(__reg),	\
		.val	= (__val)			\
	}

#define PLLCONF_GPIO(__name, __idx)			\
	PLLCONF(SREG_##__name##_MUX_REG_BASE(__idx),CONFIG_PLL_##__name##_GPIO_MUX##__idx##_REG_DEFAULT)

static const struct cx2450x_pll_config cx2450x_conf_pll[] __initdata = {
	/* Initialize the GPIO pin mux and alt func registers */
#ifdef CONFIG_PLL_CONFIG0_REG_DEFAULT
	PLLCONF(PLL_CONFIG0_REG, CONFIG_PLL_CONFIG0_REG_DEFAULT),
#endif
	/* Initialize the primary pin MUX registers */
	PLLCONF_GPIO(PRI, 0),
	PLLCONF_GPIO(PRI, 1),
	PLLCONF_GPIO(PRI, 2),
	PLLCONF_GPIO(PRI, 3),
	PLLCONF_GPIO(PRI, 4),
	PLLCONF_GPIO(PRI, 5),
	PLLCONF_GPIO(PRI, 6),
	/*Initialize the secondary pin MUX registers */
	PLLCONF_GPIO(SEC, 0),
	PLLCONF_GPIO(SEC, 1),
	PLLCONF_GPIO(SEC, 2),
	PLLCONF_GPIO(SEC, 3),
	PLLCONF_GPIO(SEC, 4),
	PLLCONF_GPIO(SEC, 5),
	PLLCONF_GPIO(SEC, 6),
	PLLCONF(SREG_ALT_PIN_FUNC_REG, CONFIG_PLL_ALT_FUNC_REG_DEFAULT)
};

static void cx2450x_register_devices(void)
{
#ifdef CONFIG_CX2450X_USB0
	/* register USB controller #0 as platform device, if enabled */
	if (platform_device_register(&cx2450x_device_usb0))
		printk(KERN_ERR "cx2450x: failed to add device %s\n", cx2450x_device_usb0.name);
#endif

#ifdef CONFIG_CX2450X_USB1
	/* register USB controller #1 as platform device, if enabled */
	if (platform_device_register(&cx2450x_device_usb1))
		printk(KERN_ERR "cx2450x: failed to add device %s\n", cx2450x_device_usb1.name);
#endif

#ifdef CONFIG_ARCVMAC
	if (platform_device_register(&cx2450x_device_vmac))
		printk(KERN_ERR "cx2450x: failed to add device %s\n", cx2450x_device_vmac.name);
#endif
}

static void cx2450x_setup_hsx(void)
{
	/*
	 * Re-Load the PIT values, to allow customers with "locked" boot loaders to
	 * alter the PIT settings
	 */
#ifdef DEBUG
	__raw_writel(HSX_PIT_GENERAL_REG_DEFAULT_DEBUG, (void __iomem *)HSX_PIT_GENERAL_REG);
	__raw_writel(HSX_PIT_DATA_REG_DEFAULT_DEBUG, (void __iomem *)HSX_PIT_DATA_REG);
	__raw_writel(HSX_PIT_INSTR_REG_DEFAULT_DEBUG, (void __iomem *)HSX_PIT_INSTR_REG);
#else
	__raw_writel(HSX_PIT_GENERAL_REG_DEFAULT, (void __iomem *)HSX_PIT_GENERAL_REG);
	__raw_writel(HSX_PIT_DATA_REG_DEFAULT, (void __iomem *)HSX_PIT_DATA_REG);
	__raw_writel(HSX_PIT_INSTR_REG_DEFAULT, (void __iomem *)HSX_PIT_INSTR_REG);
#endif
}

static void cx2450x_setup_pio_mux(void)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(cx2450x_conf_pll); i++)
		__raw_writel(cx2450x_conf_pll[i].val, (void __iomem *)cx2450x_conf_pll[i].preg);
}

static void __init cx2450x_init(void)
{
#if 0 /* keep, we can use this for MMU debugging */
	hook_fault_code(16 +6, cx2450x_abort, SIGBUS, 0, "imprecise external abort");
#endif
	cx2450x_setup_pio_mux();

	/* Setup the HSX PIT registers */
	cx2450x_clk_init();
	cx2450x_setup_hsx();
	cx2450x_register_devices();
	cx2450x_getpll_values();
}

static void __init cx2450x_reserve(void)
{
	/* Reserve memory for decarm F/W start code */
	/* reserve_bootmem(0x10000, 0x10000, BOOTMEM_EXCLUSIVE); */
}

extern struct sys_timer cx2450x_sys_timer;	/* in arch/arm/mach-nevis/time.c */

/*
 * boot_pararms will be placed (by the loader) at the megabyte boundary
 * just below where the kernel is loaded.  Since the kernel is loaded 
 * at a 0x18000 boundary (in conexant's case, 0x48000), we're always
 * guaranteed to have a PTE so the kernel can find the ATAG list here. 
 * The ATAG list is only temporary and will eventually get over-written
 * so we copy it in our machine fixup call.
 */
MACHINE_START(NEVIS, "CST HDx IRD")
	.boot_params	=  0x00000100,
	.map_io		=  cx2450x_map_io,
	.init_machine	=  cx2450x_init,
	.init_irq	=  cx2450x_init_irq,
	.timer		= &cx2450x_sys_timer,
	.fixup		=  cx2450x_fixup,
	//.reserve	=  cx2450x_reserve
MACHINE_END
