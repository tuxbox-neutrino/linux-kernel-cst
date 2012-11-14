/*
 * arch/arm/mach-nevis/clock.c
 * CX2450x Clock Management
 *
 * Copyright (C) 2009 CoolStream International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <asm/clkdev.h>
#include "clock.h"

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

/* enable and disable do nothing */
int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

static struct clk clk_25M = {
	.rate = 25 * 1000 * 1000,
};

#if 0 /* What to do with this .. */
/*
 * Catch-all default clock to satisfy drivers using the clk API.  We don't
 * model the actual hardware clocks yet.
 */
static struct clk clk_default;
#endif

#define CLK(_clk, dev)				\
	{					\
		.clk		= _clk,		\
		.dev_id		= dev,		\
	}

static struct clk_lookup lookups[] = {
	CLK(&clk_25M, "arcvmac"),
};

int __init cx2450x_clk_init(void)
{
	clkdev_add_table(lookups, ARRAY_SIZE(lookups));

	return 0;
}
