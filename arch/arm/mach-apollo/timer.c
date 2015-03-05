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


#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/time.h>
#include <asm/sched_clock.h>

#include <mach/timer.h>
#include <mach/core.h>
#include <mach/soc.h>

void stb_pm_timer_restore(void)
{
	/* TODO */
}
