
/*
 *  arch/arm/mach-nevis/clock.h
 *
 *  Copyright (C) 2011 CoolStream International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
struct clk {
	unsigned long		rate;
};

int __init cx2450x_clk_init(void);
