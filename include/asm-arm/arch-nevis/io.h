/*******************************************************************************
 *  linux/include/asm-arm/arch-bronco/io.h
 *
 *  Copyright (C) 2004 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 ********************************************************************************/

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

/*
 * Generic virtual read/write
 */
#define IO_SPACE_LIMIT		0xffffffff

/*
 * Generic function to handle IO addresses. In case
 * we have special handling we could add this here.
 */
static inline void __iomem *__io(unsigned long addr)
{
	return (void __iomem *) addr;
}

#define __io(a)                 __io((a))
#define __mem_pci(a)		(a)
#define __mem_isa(a)		(a)

/*
 * Validate the pci memory address for ioremap.
 * WARNING: address are now always assumed correct.
 */
#define iomem_valid_addr(iomem,size)	(1)

/*
 * Convert PCI memory space to a CPU physical address
 * WARNING: this is only true if we have a 1:1 mapping.
 */
#define iomem_to_phys(iomem)		(iomem)

#endif /* __ASM_ARM_ARCH_IO_H */
