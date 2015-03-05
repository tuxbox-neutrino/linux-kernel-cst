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

/* Add apollo structure declarations here */
extern struct sys_timer apollo_timer;

/* Add apollo family function declarations here */
void __init apollo_setup_of_timer(void);
void __init apollo_map_io(void);
void __init apollo_dt_init_irq(void);
void __init stb_get_chip_rev_id(void);
void apollo_restart(char, const char *);
void apollo_secondary_startup(void);
void __cpuinit apollo_cpu_die(unsigned int cpu);

extern struct smp_operations apollo_smp_ops;

#endif /* __MACH_GENERIC_H */
