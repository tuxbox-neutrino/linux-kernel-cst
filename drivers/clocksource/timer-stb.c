/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2014 CoolStream International Ltd. 
 *
 * Code based on tegra20_timer.c by Colin Cross.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/mach/time.h>
#include <asm/smp_twd.h>
#include <asm/sched_clock.h>

/* General Purpose Timers */
#define TIMER0_VALUE		0x00
#define TIMER0_LIMIT		0x04
#define TIMER0_MODE		0x08
#define TIMER_EN_INT		BIT(3)
#define TIMER_RST_CNTR		BIT(1)
#define TIMER_EN_COUNTER	BIT(0)
#define TIMER0_BASE		0x0C

#define TIMER1_VALUE		0x10
#define TIMER1_LIMIT		0x14
#define TIMER1_MODE		0x18
#define TIMER1_BASE		0x1C

#define TIMER_INT_STAT		0x200

static void __iomem *timer_reg_base;

#define timer_writel(value, reg) \
	writel_relaxed(value, timer_reg_base + (reg))
#define timer_readl(reg) \
	readl_relaxed(timer_reg_base + (reg))

static int stb_timer_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	u32 reg;

	timer_writel(0, TIMER0_MODE);
	reg = timer_readl(TIMER0_VALUE);
	timer_writel((reg + cycles) & 0xFFFFFFFF, TIMER0_LIMIT);
	reg = TIMER_EN_INT | TIMER_EN_COUNTER | TIMER_RST_CNTR;
	timer_writel(reg, TIMER0_MODE);

	return 0;
}

static void stb_timer_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	timer_writel(0, TIMER0_MODE); /* Disable timer */

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		pr_debug("Periodic!!!\n");
		timer_writel((1000000/HZ), TIMER0_LIMIT);
		timer_writel(TIMER_EN_COUNTER, TIMER0_MODE);
		timer_writel(0, TIMER0_VALUE);
		timer_writel(TIMER_EN_INT | TIMER_EN_COUNTER, TIMER0_MODE);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		pr_debug("Single Shot!!!\n");
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device stb_clockevent = {
	.name		= "timer0",
	.rating		= 300,
	.features	= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.set_next_event	= stb_timer_set_next_event,
	.set_mode	= stb_timer_set_mode,
};

static u32 notrace stb_read_sched_clock(void)
{
	return timer_readl(TIMER1_VALUE);
}

static irqreturn_t stb_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	u32 mode = timer_readl(TIMER0_MODE);

	if (evt->mode == CLOCK_EVT_MODE_ONESHOT) {
		pr_debug("Interrupt Single Shot!!!\n");
		mode &= ~(TIMER_EN_INT | TIMER_EN_COUNTER);
	}

	/* Clear interrupt */
	timer_writel(mode, TIMER0_MODE);

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction stb_timer_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= stb_timer_interrupt,
	.dev_id		= &stb_clockevent,
};

static void __init stb_init_timer(struct device_node *np)
{
	struct clk *clk;
	unsigned long rate;
	int ret;

	timer_reg_base = of_iomap(np, 0);
	if (!timer_reg_base) {
		pr_err("Can't map timer registers\n");
		BUG();
	}

	stb_timer_irq.irq = irq_of_parse_and_map(np, 0);
	if (stb_timer_irq.irq <= 0) {
		pr_err("Failed to map timer IRQ\n");
		BUG();
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_warn("Unable to get timer clock. Assuming 50Mhz input clock.\n");
		rate = 50000000;
	} else {     
		clk_prepare_enable(clk);
		rate = clk_get_rate(clk);
	}

	of_node_put(np);

	timer_writel(0, TIMER0_MODE);
	timer_writel(0, TIMER1_MODE);

	/* Make both timer 0 and 1 run at 1Mhz */
	switch (rate) {
	case 50000000:
		timer_writel(0x0032, TIMER0_BASE); /* 50Mhz / 50 = 1Mhz - 1us */
		timer_writel(0x0032, TIMER1_BASE);
		break;
	case 25000000:
		timer_writel(0x0019, TIMER0_BASE); /* 25Mhz / 25 = 1Mhz - 1us */
		timer_writel(0x0019, TIMER1_BASE);
		break;
	default:
		WARN(1, "Unknown clock rate");
	}

	timer_writel(0xFFFFFFFF, TIMER1_LIMIT);
	timer_writel(TIMER_EN_COUNTER | TIMER_RST_CNTR , TIMER1_MODE);
	timer_writel(0, TIMER1_VALUE);

	setup_sched_clock(stb_read_sched_clock, 32, 1000000);

	if (clocksource_mmio_init(timer_reg_base + TIMER1_VALUE,
		"timer_us", 1000000, 300, 32, clocksource_mmio_readl_up)) {
		pr_err("Failed to register clocksource\n");
		BUG();
	}

	ret = setup_irq(stb_timer_irq.irq, &stb_timer_irq);
	if (ret) {
		pr_err("Failed to register timer IRQ: %d\n", ret);
		BUG();
	}

	stb_clockevent.cpumask = cpu_all_mask;
	stb_clockevent.irq = stb_timer_irq.irq;
	clockevents_config_and_register(&stb_clockevent, 1000000,
					0x1, 0xffffffff);
	pr_info("stb-timer: system timer (irq = %d, rate %d)\n",
		stb_timer_irq.irq, rate);
}
CLOCKSOURCE_OF_DECLARE(stb_timer, "entr,stb-timer", stb_init_timer);

#ifdef CONFIG_PM
static u32 timer_mode;

void stb_timer_suspend(void)
{
	timer_mode = timer_readl(TIMER0_MODE);
	timer_writel(0, TIMER0_MODE);
}

void stb_timer_resume(void)
{
	timer_writel(timer_mode, TIMER0_MODE);
}
#endif
