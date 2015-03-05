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

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/clockchips.h>
#include <asm/irq.h>
#include <asm/smp_twd.h>
#include <asm/localtimer.h>

/*
 * Setup the local clock events for a CPU.
 */
void __cpuinit local_timer_setup(struct clock_event_device *evt)
{
        evt->irq = IRQ_LOCALTIMER;
        twd_timer_setup(evt);
}

