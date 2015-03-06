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


#ifndef __MACH_IO_H
#define __MACH_IO_H

#include <asm/cacheflush.h>

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

/* DMA operations */
inline static void dma_cache_inv(unsigned long start, unsigned long size)
{
        void *_start = (void*) start;
	dmac_map_area(_start,size,2); //DMA_FROM_DEVICE); Due to recursive includes, linux/dma-mapping.h is not visible
}

inline static void dma_cache_wback(unsigned long start, unsigned long size)
{
        void *_start = (void*) start;
	dmac_unmap_area(_start,size,1); //DMA_TO_DEVICE);
}

inline static void dma_cache_wback_inv(unsigned long start, unsigned long size)
{
        void *_start = (void*) start;
        void *_end = (void*) (start + size);
	dmac_flush_range(_start,_end);
}

#endif
