/*******************************************************************************************
 *  linux/include/asm-arm/arch-nevis/vmalloc.h
 *
 *  Copyright (C) 2008 CoolStream International Limited.
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
 *********************************************************************************************/

#ifndef __ASM_ARCH_VMALLOC_H
#define __ASM_ARCH_VMALLOC_H
/*
 * The maximum addressable memory on Nevis is 512MB, so we need to put the kernel virtual
 * memory above at least 512MB to be able to address the whole range. Also we have an hole
 * of 8 MB to detect out of bound ranges. So pick a number here that is at least above 512 MB 
 * and the 8MB hole. WARNING: Make sure we don't overlap with 0x80000000 PCI memory! In case
 * we have a 1:1 mapping and put this inside the static mapping table, it will be overwritten
 * by the vm.
 *
 */
#define VMALLOC_OFFSET    (8*1024*1024)
#define VMALLOC_START     (MODULE_END + VMALLOC_OFFSET)
#define VMALLOC_END       (VMALLOC_START + CONFIG_CNXT_VMALLOC_SIZE)
#endif
