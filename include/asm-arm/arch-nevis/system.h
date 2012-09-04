/***********************************************************************************
 *  linux/include/asm-arm/arch-nevis/system.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published by
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

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/cx2450x.h>

void cnxt_mip_idle(void);

static inline void arch_idle(void)
{
	/* The 'normal' thing is to call cpu_do_idle();     */
	/* which puts the cpu in 'wait for interrupt' mode. */

	/* FIXME! Implement mipidle stuff from here - DAVEM */
	/* cpu_do_idle(); */
	cnxt_mip_idle();	/* noida  neeraj */

}

static inline void arch_reset(char mode)
{
	/* Any write to this register resets the IRD! */
	writel(0x0, (void *) ASX_TO_VIRT(SYS_SOFTRESET_REG));
	while (1); /* not-reached */
}

#endif /* __ASM_ARCH_SYSTEM_H */
