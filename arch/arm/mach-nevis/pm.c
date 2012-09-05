/*
 * arch/arm/mach-nevis/pm.c
 * CX2450x Power Management
 *
 * Copyright (C) 2009 CoolStream International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/cx2450x.h>
#include <mach/gpio.h>

static inline void ddr_selfrefresh_enable(void)
{
	volatile u32 *reg = (volatile u32 *)RST_LPCONFIG_REG;

	*reg	|= 2; /* Self Refresh Enable in LP */
}

static inline void ddr_selfrefresh_disable(void)
{
	volatile u32 *reg = (volatile u32 *)RST_LPCONFIG_REG;

	*reg	&= ~2; /* Self Refresh Disable in LP */
}

static int cx2450x_pm_is_valid_state(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
			return 1;
		default:
			return 0;
	}
}

static suspend_state_t target_state;

/*
 * Called after processes are frozen, but before we shutdown devices.
 */
static int cx2450x_pm_begin(suspend_state_t state)
{
	volatile u32 *reg	= (volatile u32 *)RST_LPCONFIG_REG;
	volatile u32 *wakeup	= (volatile u32 *)RST_WAKEUP_CTRL_REG;

	*reg |= 0x1FF0;

	*wakeup |= (1 << 2);

	target_state = state;
	return 0;
}

/*
 *
 */

static int cx2450x_pm_enter(suspend_state_t state)
{
//	at91_gpio_suspend();
//	at91_irq_suspend();

	printk(KERN_DEBUG "cx2450x: PM - pm state %d\n", state);

	switch (state) {
		case PM_SUSPEND_STANDBY:
			/*
			 * NOTE: the Wait-for-Interrupt instruction needs to be
			 * in icache so no SDRAM accesses are needed until the
			 * wakeup IRQ occurs and self-refresh is terminated.
			 */
			asm("b 1f; .align 5; 1:");
			asm("mcr p15, 0, r0, c7, c10, 4");	/* drain write buffer */
			ddr_selfrefresh_enable();
			asm("mcr p15, 0, r0, c7, c0, 4");	/* wait for interrupt */
			ddr_selfrefresh_disable();
			break;

		case PM_SUSPEND_ON:
			asm("mcr p15, 0, r0, c7, c0, 4");	/* wait for interrupt */
			break;

		default:
			printk(KERN_DEBUG "cx2450x: PM - bogus suspend state %d\n", state);
			goto error;
	}

	printk(KERN_DEBUG "cx2450x: PM - wakeup %08x\n", readl(RST_WAKEUP_CTRL_REG) & readl(RST_WAKEUP_STAT_REG));

error:
	ddr_selfrefresh_disable();
	target_state = PM_SUSPEND_ON;
//	at91_irq_resume();
//	at91_gpio_resume();
	return 0;
}

/*
 * Called right prior to thawing processes.
 */
static void cx2450x_pm_end(void)
{
	volatile u32 *reg	= (volatile u32 *)RST_LPCONFIG_REG;
	volatile u32 *wakeup	= (volatile u32 *)RST_WAKEUP_CTRL_REG;


	*reg &= ~0x1FF0;
	*wakeup &= ~(1 << 2);

	target_state = PM_SUSPEND_ON;
}

static struct platform_suspend_ops cx2450x_pm_ops ={
	.valid	= cx2450x_pm_is_valid_state,
	.begin	= cx2450x_pm_begin,
	.enter	= cx2450x_pm_enter,
	.end	= cx2450x_pm_end,
};

static int __init cx2450x_pm_init(void)
{
	printk("CX2450x: Power Management\n");


	suspend_set_ops(&cx2450x_pm_ops);

	return 0;
}
arch_initcall(cx2450x_pm_init);
