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

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/dmaengine.h>
#include <asm/mach/time.h>

/* Add kronos structure declarations here */
extern struct sys_timer kronos_timer;

/* Add kronos family function declarations here */
void __init kronos_setup_of_timer(void);
void __init kronos_map_io(void);
void __init kronos_dt_init_irq(void);
void __init stb_get_chip_rev_id(void);
void kronos_restart(char, const char *);
void kronos_secondary_startup(void);
void __cpuinit kronos_cpu_die(unsigned int cpu);

extern struct smp_operations kronos_smp_ops;

#endif /* __MACH_GENERIC_H */
