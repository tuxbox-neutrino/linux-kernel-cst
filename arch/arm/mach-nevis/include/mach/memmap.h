/************************************************************************************
 * linux/include/asm-arm/arch-nevis/cx2450x.h
 *
 * global memory map
 *
 *  Copyright (C) 2008 Coolstream International Limited
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

#ifndef __MEMMAP_H
#define __MEMMAP_H

#include <asm/sizes.h>

/*
 * Fixed Mappings
 */

/* Virtual Addresses */
/* #define PCI_MEM_VADDR_BASE	0x80000000  8000.0000 - 9FFF.FFFF */
#define ASX_VADDR_BASE		0xE0000000 /* E000.0000 - E07F.FFFF */
#define EXT_IO_VADDR_BASE	0xE1000000 /* E100.0000 - E17F.FFFF */
#define TEMPEST_VADDR_BASE	0xE4000000 /* E400.0000 - E407.FFFF */
#define APP_VADDR_BASE		0xE8000000 /* 8000.0000 - 9FFF.FFFF */
/* #define FLASH_VADDR_BASE	0xF0000000  FC00.0000 - FCFF.FFFF */

/* MFB: TODO: remove me */
#define IO_BASE			(ASX_VADDR_BASE)
#define IO_ADDRESS(x)		(((x) & 0x00ffffff) | IO_BASE)

/* Mapping Range */
#define PCI_MEM_SIZE		0x20000000
#define PCI_IO_SIZE		0x00100000
#define ASX_IO_SIZE		0x01000000
#define EXT_IO_SIZE     	0x01000000	/* EXT IO ecompases PCI IO and ISA IO */
#define TEMPEST_IO_SIZE		0x01000000
#define APP_IO_SIZE		0x00040000 // SZ_64K
#define FLASH_IO_SIZE		SZ_32M

/* Physical Addresses */
#define FLASH_PHYS_BASE		0xF0000000
#define ASX_PHYS_BASE		0xE0000000
#define EXT_IO_PHYS_BASE	0xE1000000	/* Include PCI IO and ISA IO */
#define TEMPEST_PHYS_BASE	0xE4000000
#define PCI_MEM_PHYS_BASE	0x80000000
#define APP_PHYS_BASE		0xE8000000

#endif /*__MEMMAP_H */
