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

#ifndef __ASM_ARCH_NEVIS_GPIO_H__
#define __ASM_ARCH_NEVIS_GPIO_H__

#ifdef __KERNEL__

#include <linux/types.h>
#include <asm/irq.h>
#include <asm-generic/gpio.h>
#include <asm/hardware.h>

#define PIO_READ_REG		0xE0470000
#define PIO_DRIVE_HIGH_REG	0xE0470004
#define PIO_DRIVE_LOW_REG	0xE0470008
#define PIO_DRIVE_OFF_REG	0xE047000C

#define PIO_HIGH		PIO_DRIVE_HIGH_REG	/* drive PIO HIGH */
#define PIO_LOW			PIO_DRIVE_LOW_REG	/* drive PIO LOW */
#define PIO_OFF			PIO_DRIVE_OFF_REG	/* switch PIO into input mode */

#define GPIO_MAX		224
#define GPIO_bit(_n)		(1 << ((_n) & 0x1F))
#define GPLR(_n)		(*((u32*)(PIO_READ_REG       + ((_n) * 0x40))))
#define GPSR(_n)		(*((u32*)(PIO_DRIVE_HIGH_REG + ((_n) * 0x40))))
#define GPCR(_n)		(*((u32*)(PIO_DRIVE_LOW_REG  + ((_n) * 0x40))))

void gpio_drive(u32 pio, u32 state);
u32 gpio_read(u32 pio);

static inline int gpio_get_value(unsigned gpio)
{
	if (__builtin_constant_p(gpio) && (gpio < GPIO_MAX))
		return GPLR(gpio) & GPIO_bit(gpio);
	else
		return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(gpio) && (gpio < GPIO_MAX))
		if (value)
			GPSR(gpio) = GPIO_bit(gpio);
		else
			GPCR(gpio) = GPIO_bit(gpio);
	else
		__gpio_set_value(gpio, value);
}

#define gpio_cansleep   __gpio_cansleep

static inline unsigned gpio_to_irq(unsigned gpio)
{
	return gpio + 128;
}

static inline unsigned irq_to_gpio(unsigned irq)
{
	return irq - 128;
}


#endif /* __KERNEL__ */

#endif /* __ASM_ARCH_NEVIS_GPIO_H__ */
