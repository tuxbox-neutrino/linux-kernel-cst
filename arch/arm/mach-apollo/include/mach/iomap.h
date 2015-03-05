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


#ifndef __ASM_MACH_IOMAP_H
#define __ASM_MACH_IOMAP_H

#include <asm/sizes.h>

#define IO_ARM_OFFSET	0xE0100000
#define IO_ARM_VIRT	0xFEA00000
#define IO_ARM_SIZE	0x00003000
#define IO_PERI_OFFSET	0xE0600000
#define IO_PERI_VIRT	0xFEB00000
#define IO_PERI_SIZE	0x00200000

#ifdef CONFIG_MMU
#define IO_XLATE(x, off, virt)	(((x) - (off)) + (virt))

#define IO_ADDRESS(x)	(\
	((x) >= IO_ARM_OFFSET && (x) < (IO_ARM_OFFSET + IO_ARM_SIZE)) ? \
	IO_XLATE((x), IO_ARM_OFFSET, IO_ARM_VIRT) :  \
	((x) >= IO_PERI_OFFSET && (x) < (IO_PERI_OFFSET + IO_PERI_SIZE)) ? \
	IO_XLATE((x), IO_PERI_OFFSET, IO_PERI_VIRT) :  \
	0)

#define __IOMEM(x)	((void __force __iomem *)(IO_ADDRESS(x)))
#else
#define IO_ADDRESS(x)	(x)
#define __IOMEM(x)	(x)
#endif
#define __io_address(n)	__io(IO_ADDRESS(n))

#endif
