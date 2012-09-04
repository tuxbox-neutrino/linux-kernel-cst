/************************************************************************************
 *  include/asm-arm/arch-nevis/hardware.h
 *
 * Copyright (C) 2007 Conexant Systems Inc.
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
 *************************************************************************************/
#ifndef __ASM_ARCH_NEVIS_HARDWARE_H
#define __ASM_ARCH_NEVIS_HARDWARE_H

#include <asm/sizes.h>
#include <asm/arch/platform.h>
#include <asm/arch/memmap.h>

/*
 * Conversion from Phys to Virt
 */
#define ASX_TO_VIRT(x)		(((unsigned int)(x) - (unsigned int)ASX_PHYS_BASE ) + (unsigned int)ASX_VADDR_BASE)
#define EXT_IO_TO_VIRT(x)	(((unsigned int)(x) - (unsigned int)EXT_IO_PHYS_BASE ) + (unsigned int)EXT_IO_VADDR_BASE )
//#define PCI_MEM_TO_VIRT(x)	(unsigned int)(x)
#define PCIBIOS_MIN_IO		0x6000
#define PCIBIOS_MIN_MEM		0x10000

/*
 * Override the logic in pci_scan_bus
 * for skipping already-configured bus numbers.
 */
#define pcibios_assign_all_busses() 1

#endif /* __ASM_ARCH_HARDWARE_H */
