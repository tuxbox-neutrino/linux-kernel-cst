/*
 * (C) Copyright 2008
 * Coolstream Internation Limited
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/input.h>	/* for EXPORT_SYMBOL */
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/sysdev.h>
#include <linux/spinlock.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/cx2450x.h>

struct cx2450x_gpio_chip {
	struct gpio_chip chip;
	void __iomem *regbase;
};

DEFINE_SPINLOCK(gpio_lock);

/* some helpers for the GPIO controller */
/*******************************************************************************/
/* switch the pio number to the given state (PIO_HIGH, PIO_LOW, PIO_OFF)       */

void gpio_drive(u32 pio, u32 state)
{
	unsigned long flags;
	volatile u32 *reg, val;
	/* decode the requested PIO to the assigned bank/bit */
	u32 bank = pio / 32;
	u32 bit = pio % 32;

	val = state + (bank * 0x40);

	if (bank < 7) {
		spin_lock_irqsave(&gpio_lock, flags);
		reg = (volatile u32 *) val;
		*reg = (1 << bit);
		spin_unlock_irqrestore(&gpio_lock, flags);
	}
}

/*******************************************************************************/
/* read the state of one GPIO pin (if configured as input before)              */

u32 gpio_read(u32 pio)
{
	unsigned long flags;
	volatile u32 *reg;
	u32 bank = pio / 32;
	u32 bit = pio % 32;
	u32 ret = 0;

	if (bank < 7) {
		spin_lock_irqsave(&gpio_lock, flags);
		reg = (volatile u32 *) (PIO_READ_REG + (bank * 0x40));
		if ((*reg) & (1 << bit))
			ret = 1;
		spin_unlock_irqrestore(&gpio_lock, flags);
	}

	return ret;
}

/*******************************************************************************/

EXPORT_SYMBOL(gpio_drive);
EXPORT_SYMBOL(gpio_read);

#define PIO_BASE		0xE0470000
#define PIO_BANKS		0x07
#define PIO_BANK_SIZE		0x40
#define PIO_READ_OFFSET		0x00
#define PIO_HIGH_OFFSET		0x04
#define PIO_LOW_OFFSET		0x08
#define PIO_OFF_OFFSET		0x0C

static int cx2450x_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	u32 mask = 1 << offset;
	u32 value;
	unsigned long flags;
	struct cx2450x_gpio_chip *cx;
	void __iomem *pio_off;

	cx = container_of(chip, struct cx2450x_gpio_chip, chip);
	pio_off = cx->regbase + PIO_OFF_OFFSET;

	spin_lock_irqsave(&gpio_lock, flags);
	value = readl(pio_off);
	value |= mask;
	writel(value, pio_off);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return 0;
}

static int cx2450x_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 mask = 1 << offset;
	unsigned long flags;
	struct cx2450x_gpio_chip *cx;
	void __iomem *pio_off;

	cx = container_of(chip, struct cx2450x_gpio_chip, chip);
	pio_off = cx->regbase + (value ? PIO_HIGH_OFFSET : PIO_LOW_OFFSET);

	spin_lock_irqsave(&gpio_lock, flags);
	writel(mask, pio_off);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return 0;
}

/*
 * Return GPIO level
 */
static int cx2450x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 mask = 1 << offset;
	struct cx2450x_gpio_chip *cx;

	cx = container_of(chip, struct cx2450x_gpio_chip, chip);
	return !!(readl(cx->regbase + PIO_READ_OFFSET) & mask);
}

/*
 * Set output GPIO level
 */
static void cx2450x_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 mask = 1 << offset;
	struct cx2450x_gpio_chip *cx;

	cx = container_of(chip, struct cx2450x_gpio_chip, chip);

	if (value)
		writel(mask, cx->regbase + PIO_HIGH_OFFSET);
	else
		writel(mask, cx->regbase + PIO_LOW_OFFSET);
}

#define GPIO_CHIP(_n)									\
	[_n] = {									\
		.regbase = (void __iomem *)(PIO_BASE + ((_n) * PIO_BANK_SIZE)),		\
		.chip = {								\
			.label			= "gpio-" #_n,				\
			.direction_input	= cx2450x_gpio_direction_input,		\
			.direction_output	= cx2450x_gpio_direction_output,	\
			.get			= cx2450x_gpio_get,			\
			.set			= cx2450x_gpio_set,			\
			.base			= (_n) * 32,				\
			.ngpio			= 32,					\
		},									\
	}

static struct cx2450x_gpio_chip cx2450x_gpio_chip[] = {
	GPIO_CHIP(0),
	GPIO_CHIP(1),
	GPIO_CHIP(2),
	GPIO_CHIP(3),
	GPIO_CHIP(4),
	GPIO_CHIP(5),
	GPIO_CHIP(6),
};

static const u32 pio_excl_mask[] = {
	CONFIG_PIO_EXCL_MASK_PIO_031_000,
	CONFIG_PIO_EXCL_MASK_PIO_063_032,
	CONFIG_PIO_EXCL_MASK_PIO_095_064,
	CONFIG_PIO_EXCL_MASK_PIO_127_096,
	CONFIG_PIO_EXCL_MASK_PIO_159_128,
	CONFIG_PIO_EXCL_MASK_PIO_191_160,
	CONFIG_PIO_EXCL_MASK_PIO_223_192
};

int __init cx2450x_init_gpio(void)
{
	u32 i;
#ifdef CONFIG_PIO_INIT_ON_BOOT
	volatile u32 *reg;

	if (ARRAY_SIZE(pio_excl_mask) != PIO_BANKS)
		printk(KERN_ERR "banks are not matching the masks\n");

	/* Clear out GPIO registers. */
	for (i = 0; i < ARRAY_SIZE(pio_excl_mask); i++) {
		/* Init all GPIO interrupt sources to known inactive state */
		reg = (volatile u32 *) GPIO_POS_EDGE_REG_BASE(i);
		*reg = 0;
		reg = (volatile u32 *) GPIO_NEG_EDGE_REG_BASE(i);
		*reg = 0;

		/* Make sure all GPIOs are inactive and in high impedance state */
		reg = (volatile u32 *) GPIO_DRIVE_OFF_REG_BASE(i);
		*reg = (0xFFFFFFFF ^ pio_excl_mask[i]);
	}
#endif				/* CONFIG_PIO_INIT_ON_BOOT */

	for (i = 0; i < PIO_BANKS; i++)
		gpiochip_add(&cx2450x_gpio_chip[i].chip);

	return 0;
}


static struct sysdev_class cx2450x_gpio_sysclass = {
	.name		= "gpio",
	.suspend	= NULL, /*_gpio_suspend,*/
	.resume		= NULL, /* _gpio_resume,*/
};

static int __init cx2450x_gpio_init_sys(void)
{
	return sysdev_class_register(&cx2450x_gpio_sysclass);
}

core_initcall(cx2450x_gpio_init_sys);
arch_initcall(cx2450x_init_gpio);
