/*
 * drivers/char/hw_random/cx2450x-rng.c
 *
 * RNG driver for Trident/NXP CX2450x type SoC's 
 *
 * Author: The Coolstream Development Team
 *
 * Copyright 2010 (c) Coolstream Intl. Ltd.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/hw_random.h>

#include <asm/io.h>
#include <asm/hardware.h>

#define MC_RANDOM_REG 0xE0500398

/******************************************************************************/

static int cx2450x_rng_data_read(struct hwrng *rng, u32 *buffer)
{
	*buffer = *((volatile u32*)MC_RANDOM_REG);

	return 4;
}

/******************************************************************************/

static struct hwrng cx2450x_rng_ops = {
	.name		= "cx2450x",
	.data_read	= cx2450x_rng_data_read,
};

/******************************************************************************/

static int __init cx2450x_rng_init(void)
{
	return hwrng_register(&cx2450x_rng_ops);
}

/******************************************************************************/

static void __exit cx2450x_rng_exit(void)
{
	hwrng_unregister(&cx2450x_rng_ops);
}

/******************************************************************************/

module_init(cx2450x_rng_init);
module_exit(cx2450x_rng_exit);

MODULE_AUTHOR("The Coolstream Development Team");
MODULE_DESCRIPTION("H/W Random Number Generator (RNG) driver for CX2450x");
MODULE_LICENSE("GPL");
