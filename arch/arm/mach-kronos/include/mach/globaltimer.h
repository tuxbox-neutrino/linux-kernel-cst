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

#ifndef __ASM_ARCH_GLOBALTIMER_H
#define __ASM_ARCH_GLOBALTIMER_H

#define GLOBAL_TIMER_COUNT_LOW			0x00
#define GLOBAL_TIMER_COUNT_HIGH			0x04
#define GLOBAL_TIMER_CONTROL			0x08
#define GLOBAL_TIMER_STATUS			0x0C
#define GLOBAL_TIMER_COMPARATOR_LOW		0x10
#define GLOBAL_TIMER_COMPARATOR_HIGH		0x14
#define GLOBAL_TIMER_AUTO_INCREMENT		0x18

#define GLOBAL_TIMER_CTRL_AUTO_INC		0x08
#define GLOBAL_TIMER_CTRL_IRQ_ENA		0x04
#define GLOBAL_TIMER_CTRL_COMP_ENA		0x02
#define GLOBAL_TIMER_CTRL_TIMER_ENA		0x01

#define GLOBAL_TIMER_STAT_EVENT			0x01

extern void kronos_core_intc_init(unsigned long vaddr);
extern void kronos_global_timer_init(unsigned long freq);

struct stb_pm_timer_save {
	u32     low;
	u32     high;
	u32     control;
	u32     status; /*no need to restore....*/
	u32     cmp_low;
	u32     cmp_high;
	u32     auto_inc;
};
extern void stb_pm_timer_save(void);
extern void stb_pm_timer_restore(void);
#endif
