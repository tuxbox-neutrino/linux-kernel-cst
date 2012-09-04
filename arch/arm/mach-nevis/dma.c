/*****************************************************************************
 * linux/arch/arm/mach-nevis/dma.c
 * 
 * Copyright (C) 2007 Conexant Systems Inc, USA.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published by
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
 *****************************************************************************/
/*$Id$
 ******************************************************************************/

#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/init.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include <asm/mach/dma.h>

void __init arch_dma_init(dma_t * dma)
{
}
