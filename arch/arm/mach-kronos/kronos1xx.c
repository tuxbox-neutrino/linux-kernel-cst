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


#include <linux/amba/pl022.h>
#include <linux/clk.h>
#include <linux/dw_dmac.h>
#include <linux/err.h>
#include <linux/of_irq.h>
#include <linux/clocksource.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>
#include <asm/smp_twd.h>
#include <mach/generic.h>
#include <mach/soc.h>
#include <mach/iomap.h>
#include <mach/globaltimer.h>

#ifdef CONFIG_CACHE_L2X0
static int  __init kronos_l2x0_init(void)
{
	/*
	 * Data and Instruction prefetch,
	 * 128KB (16KB/way),
	 * 8-way associativity,
	 * Exclusive,
	 * Zero enable
	 * Bits:	0011 0010 0000 0010 0001 0000 0000 0001
	 */
#ifdef CONFIG_OF
	l2x0_of_init(0x32021001|L2X0_AUX_CTRL_SHARE_OVERRIDE_MASK,
		     0xffffffff);
#else
	l2x0_init(__io_address(CORTEX_A9_L2CACHE_BASE),
		  0x32021001|L2X0_AUX_CTRL_SHARE_OVERRIDE_MASK,
		  0xffffffff);
#endif

	/* Change the ACTRL register to Exlusive, Zero Enable, L1 prefetch, L2 prefetch */
	__asm__("mrc p15, 0, r12, c1, c0, 1\n\t"
		"orr r12, r12, #0x8e\n\t"
		"mcr p15, 0, r12, c1, c0, 1"
		: : : "r12","cc", "memory");

	return 0;

}
early_initcall(kronos_l2x0_init);
#endif

struct map_desc kronos_io_desc[] __initdata = {
	{
		.virtual = (unsigned long)IO_ARM_VIRT,
		.pfn	 = __phys_to_pfn(IO_ARM_OFFSET),
		.length	 = IO_ARM_SIZE,
		.type	 = MT_DEVICE
	}, {
		.virtual = (unsigned long)IO_PERI_VIRT, 
		.pfn	 = __phys_to_pfn(IO_PERI_OFFSET),
		.length	 = IO_PERI_SIZE,
		.type	 = MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init kronos_map_io(void)
{
	printk (KERN_INFO "Early platform io descriptors mapping...\n");
	iotable_init(kronos_io_desc, ARRAY_SIZE(kronos_io_desc));
}

void __init kronos_timer_init(void)
{
	extern unsigned long get_arm_freq(void);
	unsigned long freq =  SOC_PERIPHERALS_FREQ;
	kronos_global_timer_init(freq);
//	twd_local_timer_of_register();
	clocksource_of_init();	
}

#define CHIP_ID_MASK   0xf000
#define CHIP_ID_SHIFT  0
#define CHIP_REV_MASK  0xf00
#define CHIP_REV_SHIFT 0

unsigned long ChipID    = 0;
unsigned long ChipRevID = 0;

/* This function is to get the SoC major and minor Rev. Id's */
void __init stb_get_chip_rev_id(void)
{
	unsigned long glb_modid;
	unsigned long function_code;

	glb_modid  = readl(GLOBALREG_MODULE_ID_REG);
	ChipID     = (glb_modid & CHIP_ID_MASK) >> CHIP_ID_SHIFT;
	ChipRevID  = (glb_modid & CHIP_REV_MASK) >> CHIP_REV_SHIFT;

	printk(KERN_ERR "ChipID=0x%lx ChipRevID=0x%lx\n", ChipID, ChipRevID);
}

unsigned int GetChipRev (void)
{
    return ChipRevID;
}

unsigned int GetChipID (void)
{
    return ChipID;
}
EXPORT_SYMBOL(ChipID);
EXPORT_SYMBOL(ChipRevID);
EXPORT_SYMBOL(GetChipRev);
EXPORT_SYMBOL(GetChipID);
