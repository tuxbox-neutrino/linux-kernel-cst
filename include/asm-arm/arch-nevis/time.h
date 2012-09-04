/***********************************************************************************
 *  linux/include/asm-arm/arch-nevis/time.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
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
 ************************************************************************************/

#ifndef __LINUX_ARCH_TIME_H__
#define __LINUX_ARCH_TIME_H__

#include <asm/system.h>
#include <asm/irq.h>
#include <asm-arm/io.h>
#include <linux/interrupt.h>


/* Clock ticks at 54 Mhz, giving maximum 79.5sec count */
#define uSEC_1 54
#define TIMER_INTERVAL	((uSEC_1 * 1000000) / HZ)

void cx2450x_unlock_timer(u32 timer_number);
s32 cx2450x_lock_timer(u32 timer_number);
s32 cx2450x_softlock_timer(u32 timer_number);

#endif	/* __LINUX_ARCH_TIME_H__ */
