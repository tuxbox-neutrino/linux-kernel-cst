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

#include <mach/core.h>
#include <mach/globaltimer.h>
#include <mach/soc.h>

#if 0
static void __iomem *global_timer_vaddr;
static unsigned long global_timer_freq=0;

static void clockevent_set_mode(enum clock_event_mode mode,
		struct clock_event_device *clk_event_dev);
static int clockevent_next_event(unsigned long evt,
		struct clock_event_device *clk_event_dev);

static u32 notrace kronos_read_sys_clock(void)
{
#if 0
	u32 upper, lower;

	/* The upper 32 bits might wrap around when reading the lower 32 bits,
	 * so we continue reading until we are sure that the upper 32 bits
	 * stayed the same while reading the lower 32 bits */
	do {
		upper = readl_relaxed(global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH);
		lower = readl_relaxed(global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
	} while (upper != readl_relaxed(global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH));

	return (((u64)upper << 32) + lower);
#else
	return readl_relaxed(global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
#endif
}

static void kronos_clocksource_init(void)
{
	int ret;
	unsigned long ctrl = 0;
	unsigned long count = (global_timer_freq / HZ);

	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_HIGH);
	writel(count, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_LOW);
	writel(count, global_timer_vaddr + GLOBAL_TIMER_AUTO_INCREMENT);
	ctrl = (GLOBAL_TIMER_CTRL_IRQ_ENA   | GLOBAL_TIMER_CTRL_COMP_ENA |
	GLOBAL_TIMER_CTRL_TIMER_ENA | GLOBAL_TIMER_CTRL_AUTO_INC);
	writel(ctrl, global_timer_vaddr + GLOBAL_TIMER_CONTROL);

	setup_sched_clock(kronos_read_sys_clock, 32, global_timer_freq);

	/* register the clocksource */
	ret = clocksource_mmio_init(global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW,
		"global_timer_src", global_timer_freq, 300, 32, clocksource_mmio_readl_up);
	if(ret)
		printk("\n %s: Clock source init failed \n", __func__);
}

static struct clock_event_device clkevt = {
    .name 		= "glb-timer_evt",
    .features 		= CLOCK_EVT_FEAT_PERIODIC,
    .set_mode		= clockevent_set_mode,
    .set_next_event	= clockevent_next_event,
    .rating		= 300,
    .cpumask		= cpu_all_mask,
};

static void clockevent_set_mode(enum clock_event_mode mode,
                struct clock_event_device *clk_event_dev)
{
	unsigned long ctrl = 0;

	ctrl = readl(global_timer_vaddr + GLOBAL_TIMER_CONTROL);
	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		ctrl |= (GLOBAL_TIMER_CTRL_IRQ_ENA |
			GLOBAL_TIMER_CTRL_COMP_ENA |
			GLOBAL_TIMER_CTRL_TIMER_ENA |
			GLOBAL_TIMER_CTRL_AUTO_INC);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH);
		writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
		writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_HIGH);
		writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_LOW);
		ctrl = 0x0;
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		ctrl &= ~GLOBAL_TIMER_CTRL_IRQ_ENA;
		break;

	case CLOCK_EVT_MODE_RESUME:
		ctrl |= GLOBAL_TIMER_CTRL_IRQ_ENA;
		break;

	default:
		printk("Unknown timer event = %d\n", mode);
		ctrl = 0x00;
		break;
	}

	writel(ctrl, global_timer_vaddr + GLOBAL_TIMER_CONTROL);

}

static int clockevent_next_event(unsigned long evt,
                 struct clock_event_device *clk_event_dev)
{
	unsigned long ctrl;

	/* The comparator needs to be disabled as race conditions
	 * exist: we need to update the comparator value in two
	 * steps as it is a 64 bit value */
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_CONTROL);
	writel(evt, global_timer_vaddr + GLOBAL_TIMER_AUTO_INCREMENT);
	writel(evt, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_LOW);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_HIGH);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH);
	ctrl = (GLOBAL_TIMER_CTRL_COMP_ENA |
		GLOBAL_TIMER_CTRL_TIMER_ENA |
		GLOBAL_TIMER_CTRL_IRQ_ENA);

	/* Enable comparator + interrupt again */
	writel(ctrl, global_timer_vaddr + GLOBAL_TIMER_CONTROL);

	return 0;
}

static irqreturn_t kronos_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clkevt;

	/* clear the interrupt */
	writel(GLOBAL_TIMER_STAT_EVENT, global_timer_vaddr + GLOBAL_TIMER_STATUS);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction kronos_timer_irq = {
	.name    = "sys_timer",
	.flags   = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = kronos_timer_interrupt,
	.dev_id  = &clkevt,
};

static void __init kronos_clockevent_init(int irq)
{
	int ret;

	ret = setup_percpu_irq(irq, &kronos_timer_irq);
	if(ret) {
		pr_err("Sytem Timer IRQ register failed: %d \n", ret);
		BUG();
	}

	enable_percpu_irq(irq, 0);
	clkevt.cpumask = cpumask_of(0);
	clkevt.irq = irq;

	clockevents_config_and_register(&clkevt, global_timer_freq, 0xf, 0xffffffff);

}

const static struct of_device_id timer_of_match[] __initconst = {
	{ .compatible = "entr,arm-glb-timer", },
	{ },
};

void __init kronos_global_timer_init(unsigned long freq)
{
	struct device_node *np;
	int irq;

	global_timer_freq   = freq;

	np = of_find_matching_node(NULL, timer_of_match);
	if (!np) {
		pr_err("%s: No timer passed via DT\n", __func__);
		return;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%s: No irq passed for timer via DT\n", __func__);
		return;
	}

	global_timer_vaddr = of_iomap(np, 0);
	if (!global_timer_vaddr) {
		pr_err("%s: of iomap failed\n", __func__);
		return;
	}


	kronos_clocksource_init();
	kronos_clockevent_init(irq);
}
#else
extern unsigned long global_timer_freq;
static void __iomem *global_timer_vaddr;

const static struct of_device_id timer_of_match[] __initconst = {
	{ .compatible = "arm,cortex-a9-global-timer", },
	{ },
};

void __init kronos_global_timer_init(unsigned long freq)
{
	struct device_node *np;

	global_timer_freq = freq;

	np = of_find_matching_node(NULL, timer_of_match);
	if (!np) {
		pr_err("%s: No timer passed via DT\n", __func__);
		return;
	}

	global_timer_vaddr = of_iomap(np, 0);
	if (!global_timer_vaddr) {
		pr_err("%s: of iomap failed\n", __func__);
		return;
	}

	pr_info("Initializing Cortex A9 Global Timer at Vir:0x%08X,"
		" using, at Freq:%lu MHz\n",
		(unsigned int)global_timer_vaddr, (freq/1000)/1000);

}
#endif

void stb_pm_timer_restore(void)
{
	unsigned long ctrl = 0;
	unsigned long count = (global_timer_freq / HZ) + 1;
   
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_CONTROL);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_HIGH);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COUNT_LOW);
	writel(0x0, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_HIGH);
	writel(count, global_timer_vaddr + GLOBAL_TIMER_COMPARATOR_LOW);
	writel(count, global_timer_vaddr + GLOBAL_TIMER_AUTO_INCREMENT);
	ctrl |=  (GLOBAL_TIMER_CTRL_COMP_ENA |
			GLOBAL_TIMER_CTRL_TIMER_ENA | GLOBAL_TIMER_CTRL_AUTO_INC);
	writel(ctrl, global_timer_vaddr + GLOBAL_TIMER_CONTROL);

	return;
}

