/*********************************************************************************
 *  linux/include/asm-arm/arch-nevis/irq.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2  as published by
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
 ***********************************************************************************/
/* $Id$
 ***********************************************************************************/
#ifndef __ASM_ARCH_NEVIS_IRQ_H
#define __ASM_ARCH_NEVIS_IRQ_H

#define LATENCY_COUNT	1000

struct s_latency {
	unsigned int delta[LATENCY_COUNT];
	unsigned int duration[LATENCY_COUNT];
	unsigned int count;
	unsigned int dur_count;
};

extern void __init cx2450x_init_irq(void);

/* #define fixup_irq(i)  (edwards_cascade_irq(i)) */

#endif /* __ASM_ARCH_IRQ_H */
