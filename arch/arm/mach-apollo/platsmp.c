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
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/unified.h>
#include <asm/smp_scu.h>
//#include <asm/hardware/gic.h>

#include <mach/soc.h>
#include <mach/core.h>
#include <mach/scu.h>
#include <asm/smp_scu.h>

//static cpumask_t apollo_cpu_init_mask;

extern void apollo_secondary_startup(void);
//extern void gic_raise_softirq(const struct cpumask *mask, unsigned int irq);

/*
 * Control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
//volatile int pen_release = -1;

static inline void __iomem *scu_base_addr(void)
{
	return CORTEX_A9_SCU_BASE;
}

static inline unsigned int get_core_count(void)
{
	return 1;
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit apollo_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit apollo_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	flush_cache_all();
	outer_clean_range(__pa(&secondary_data), __pa(&secondary_data + 1));
	pen_release = cpu;
	flush_cache_all();
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));

	/*
	 * XXX
	 *
	 * This is a later addition to the booting protocol: the
	 * bootMonitor now puts secondary cores into WFI, so
	 * wakeup_secondary() no longer gets the cores moving; we need
	 * to send a soft interrupt to wake the secondary core.
	 * Use smp_cross_call() for this, since there's little
	 * point duplicating the code here
	 */

	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}
	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init wakeup_secondary(void)
{
	 unsigned long val = 0;

	/* nobody is to be released from the pen yet */
	pen_release = -1;

	/*
	 * Secondary cores will be waiting on the scratch pad reg20
	 * value to be SECONDARY_CPU_HOLDING_PEN 
	 */
	val = readl(SCRATCH_REG_CPU_PEN);
	writel(SECONDARY_CPU_HOLDING_PEN, (SCRATCH_REG_CPU_PEN));

	mb();
	while(val<=2)
		val = readl(SCRATCH_REG_CPU_PEN);

	printk("woken up secondary core \n");
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init apollo_smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

void __init apollo_smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = get_core_count();
//	unsigned int cpu = smp_processor_id();
	int i;

	/* sanity check */
	if (ncores == 0) {
		printk(KERN_ERR
		       "apollo: strange CM count of 0? Default to 1\n");

		ncores = 1;
	}

	if (ncores > NR_CPUS) {
		printk(KERN_WARNING
		       "apollo: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

//	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start. 
	 */
	if (max_cpus > 1) {
		/*
		 * Enable the local timer or broadcast device for the
		 * boot CPU, but only if we have more than one CPU.
		 */
//		percpu_timer_setup();

		scu_enable(scu_base_addr());
		wakeup_secondary();
	}
	else if(max_cpus == 1)
	{
//		percpu_timer_setup();
		scu_enable(scu_base_addr());
	}
}

struct smp_operations apollo_smp_ops __initdata = {
       .smp_init_cpus		= apollo_smp_init_cpus,
       .smp_prepare_cpus	= apollo_smp_prepare_cpus,
       .smp_secondary_init	= apollo_secondary_init,
       .smp_boot_secondary	= apollo_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
//      .cpu_die		= apollo_cpu_die,
#endif
};
