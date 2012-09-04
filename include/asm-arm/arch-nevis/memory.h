/********************************************************************************
 *  linux/include/asm-arm/arch-nevis/memory.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
 *  Copyright (C) 2008 Coolstream International Limited.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2 as published by
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
 *********************************************************************************/

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* for PAGE_SHIFT */
#include <asm/page.h>
#include <asm/memory.h>

/*
 * Physical start address of the RAM
 */
#define PHYS_OFFSET		UL(0x00000000)

/*
 * Page offset of the kernel memory
 */
#define PAGE_OFFSET		UL(0x80000000)

/*
 * Task size: 1GB
 */
#define TASK_SIZE       	UL(0x7F000000)
#define TASK_UNMAPPED_BASE	(PAGE_ALIGN((TASK_SIZE + 0x10000000) / 3))

/*
 * On brazos and trinity, the dram is contiguous
 */
#define __virt_to_phys__is_a_macro
#define __virt_to_phys(vpage) ((vpage) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt__is_a_macro
#define __phys_to_virt(ppage) ((ppage) + PAGE_OFFSET - PHYS_OFFSET)

#define __virt_to_bus__is_a_macro
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt__is_a_macro
#define __bus_to_virt(x)	__phys_to_virt(x)

#if 0
/*
 * For now we disable this. This might be needed in the future when
 * we have our own driver code.
 */

#ifndef __ASSEMBLY__
/*
 * Restrict DMA-able region to workaround silicon bug.  The bug
 * restricts buffers available for DMA to video hardware to be
 * below 128M
 */
static inline void
__arch_adjust_zones(int node, unsigned long *size, unsigned long *holes)
{
        unsigned int sz = (x << 20) >> PAGE_SHIFT;

        if (node != 0)
                sz = 0;

        size[1] = size[0] - sz;
        size[0] = sz;
}

#define arch_adjust_zones(node, zone_size, holes) \
		__arch_adjust_zones(node, zone_size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + (x<<20) - 1)
#endif

#endif

/*
 * Given a physical address, convert it to a node id.
 * WARNING: HACK! this might be causing issues when we don't
 *	    have contiguous memory.
 */
#define PHYS_TO_NID(addr)	(0)

/*
 * Needed for discontiguous memory. Defines the maximum nr of bits
 * that one node needs to define one bank.
 */
/*#define NODE_MEM_SIZE_BITS	27*/

#endif /* __ASM_ARCH_MEMORY_H */
