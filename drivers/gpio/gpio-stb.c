/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author:
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


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <mach/gpio.h>
#include <linux/of_device.h>
#include "gpio-stb.h"

#ifdef CONFIG_ARCH_APOLLO
const int gpio_get_pin_num[] = 
{
	PIOXXX, PIO001, PIO002, PIO003, PIO004, PIO005, PIO006, PIO007, PIO008, PIO009,	
	PIO010, PIO011, PIO012, PIO013, PIO014, PIO015, PIO016, PIOXXX, PIOXXX, PIO019,	
	PIO020, PIO021, PIO022, PIOXXX, PIO024, PIO025, PIOXXX, PIO027, PIOXXX, PIOXXX,	
	PIOXXX, PIO031, PIO032, PIO033, PIO034, PIO035, PIO036, PIO037, PIOXXX, PIO039, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO044, PIO045, PIOXXX, PIOXXX, PIO048, PIO049, 
	PIOXXX, PIO051, PIO052, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO063, PIO064, PIO065, PIOXXX, PIOXXX, PIO068, PIO069, 
	PIO070, PIO071, PIO072, PIO073, PIO074, PIO075, PIOXXX, PIO077, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO082, PIO083, PIO084, PIO085, PIO086, PIO087, PIO088, PIO089, 
	PIO090, PIO091, PIO092, PIO093, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO103, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO109, 
	PIO110, PIO111, PIO112, PIO113, PIO114, PIO115, PIO116, PIO117, PIO118, PIO119, 
	PIO120, PIO121, PIO122, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO128, PIO129, 
	PIO130, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIO141, PIO142, PIO143, PIO144, PIO145, PIO146, PIO147, PIO148, PIO149, 
	PIO150, PIO151, PIO152, PIO153, PIO154, PIO155, PIO156, PIO157, PIO158, PIO159, 
	PIOXXX, PIO161, PIO162, PIOXXX, PIO164, PIO165, PIOXXX, PIOXXX, PIOXXX, PIO169, 
	PIO170, PIO171, PIO172, PIO173, PIO174, PIO175, PIO176, PIO177, PIO178, PIO179, 
	PIO180, PIO181, PIO182, PIO183, PIO184, PIO185, PIO186, PIO187, PIO188, PIO189, 
	PIO190, PIO191, PIOXXX, PIO193, PIO194, PIO195, PIO196, PIO197, PIO198, PIO199 
};
#elif defined CONFIG_ARCH_KRONOS
const int gpio_get_pin_num[] = 
{
	PIOXXX, PIOXXX, PIOXXX, PIO003, PIO004, PIO005, PIO006, PIO007, PIO008, PIO009,	
	PIO010, PIO011, PIO012, PIO013, PIO014, PIO015, PIO016, PIO017, PIO018, PIOXXX,	
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO024, PIOXXX, PIOXXX, PIO027, PIOXXX, PIOXXX,	
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO039, 
	PIO040, PIO041, PIO042, PIO043, PIO044, PIO045, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO053, PIO054, PIO055, PIO056, PIO057, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO062, PIO063, PIO064, PIO065, PIOXXX, PIOXXX, PIO068, PIO069, 
	PIO070, PIO071, PIO072, PIO073, PIO074, PIO075, PIOXXX, PIO077, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO082, PIO083, PIO084, PIO085, PIO086, PIO087, PIO088, PIO089, 
	PIO090, PIO091, PIO092, PIO093, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO103, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO118, PIO119, 
	PIO120, PIO121, PIO122, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO128, PIO129, 
	PIO130, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO146, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO162, PIOXXX, PIO164, PIO165, PIOXXX, PIOXXX, PIOXXX, PIO169, 
	PIO170, PIO171, PIO172, PIO173, PIO174, PIO175, PIO176, PIO177, PIO178, PIO179, 
	PIO180, PIOXXX, PIO182, PIO183, PIO184, PIO185, PIO186, PIO187, PIO188, PIO189, 
	PIOXXX, PIOXXX, PIOXXX, PIO193, PIO194, PIO195, PIO196, PIO197, PIO198, PIOXXX 
};

#elif defined CONFIG_ARCH_KROME
const int gpio_get_pin_num[] = 
{
	PIOXXX, PIOXXX, PIOXXX, PIO003, PIO004, PIO005, PIO006, PIO007, PIO008, PIO009,
	PIO010, PIO011, PIO012, PIO013, PIO014, PIO015, PIO016, PIO017, PIO018, PIOXXX,
	PIOXXX, PIOXXX, PIOXXX, PIO023, PIO024, PIOXXX, PIOXXX, PIO027, PIOXXX, PIOXXX,
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO039, 
	PIO040, PIO041, PIOXXX, PIOXXX, PIO044, PIO045, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO053, PIO054, PIO055, PIO056, PIO057, PIO058, PIO059, 
	PIO060, PIO061, PIO062, PIO063, PIO064, PIO065, PIOXXX, PIOXXX, PIO068, PIO069, 
	PIO070, PIO071, PIO072, PIO073, PIO074, PIO075, PIOXXX, PIO077, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO082, PIO083, PIO084, PIO085, PIO086, PIO087, PIO088, PIO089, 
	PIO090, PIO091, PIO092, PIO093, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO103, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO109, 
	PIO110, PIO111, PIO112, PIO113, PIO114, PIO115, PIO116, PIO117, PIO118, PIO119, 
	PIO120, PIO121, PIO122, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO128, PIO129, 
	PIO130, PIO131, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO146, PIO147, PIO148, PIO149, 
	PIO150, PIO151, PIO152, PIO153, PIO154, PIO155, PIO156, PIO157, PIO158, PIO159, 
	PIOXXX, PIOXXX, PIO162, PIOXXX, PIO164, PIO165, PIOXXX, PIOXXX, PIOXXX, PIO169, 
	PIO170, PIO171, PIO172, PIO173, PIO174, PIO175, PIO176, PIO177, PIO178, PIO179, 
	PIO180, PIOXXX, PIO182, PIO183, PIO184, PIO185, PIO186, PIO187, PIO188, PIO189, 
	PIOXXX, PIOXXX, PIOXXX, PIO193, PIO194, PIO195, PIO196, PIO197, PIO198, PIOXXX 
};
#elif defined CONFIG_ARCH_KORE3
const int gpio_get_pin_num[] = 
{
	PIOXXX, PIOXXX, PIOXXX, PIO003, PIO004, PIOXXX, PIO006, PIO007, PIO008, PIO009,
	PIO010, PIOXXX, PIO012, PIOXXX, PIO014, PIO015, PIO016, PIO017, PIO018, PIOXXX,
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO024, PIOXXX, PIOXXX, PIO027, PIOXXX, PIOXXX,
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO039, 
	PIOXXX, PIO041, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO053, PIO054, PIO055, PIO056, PIO057, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO063, PIO064, PIO065, PIOXXX, PIOXXX, PIO068, PIO069, 
	PIOXXX, PIO071, PIO072, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO077, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO082, PIO083, PIOXXX, PIO085, PIO086, PIOXXX, PIO088, PIOXXX, 
	PIOXXX, PIO091, PIO092, PIO093, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIO103, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIO120, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO129, 
	PIO130, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO146, PIO147, PIO148, PIO149, 
	PIO150, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIOXXX, PIO162, PIOXXX, PIO164, PIO165, PIOXXX, PIOXXX, PIOXXX, PIOXXX, 
	PIOXXX, PIO171, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO176, PIO177, PIO178, PIO179, 
	PIO180, PIOXXX, PIO182, PIO183, PIO184, PIO185, PIO186, PIO187, PIO188, PIO189, 
	PIOXXX, PIOXXX, PIOXXX, PIO193, PIO194, PIO195, PIO196, PIO197, PIO198, PIOXXX,
	PIO200, PIO201, PIO202, PIO203, PIO204, PIO205, PIOXXX, PIOXXX, PIOXXX, PIOXXX 
};
#elif defined CONFIG_ARCH_PULSAR
const int gpio_get_pin_num[] = 
{
	PIO000, PIO001, PIO002, PIO003, PIO004, PIO005, PIO006, PIO007, PIO008, PIO009,
	PIO010, PIO011, PIO012, PIO013, PIO014, PIO015, PIO016, PIO017, PIO018, PIO019,
	PIO020, PIO021, PIO022, PIO023, PIO024, PIO025, PIO026, PIO027, PIO028, PIO029,
	PIO030, PIO031, PIO032, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO038, PIOXXX, 
	PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO046, PIO047, PIO048, PIO049, 
	PIO050, PIO051, PIO052, PIO053, PIO054, PIO055, PIO056, PIO057, PIO058, PIO059, 
	PIO060, PIO061, PIO062, PIO063, PIO064, PIO065, PIO066, PIO067, PIO068, PIO069, 
	PIO070, PIOXXX, PIO072, PIO073, PIO074, PIO075, PIO076, PIO077, PIO078, PIO079, 
	PIO080, PIO081, PIO082, PIO083, PIO084, PIO085, PIO086, PIO087, PIO088, PIO089, 
	PIO090, PIOXXX, PIOXXX, PIOXXX, PIOXXX, PIO095, PIO096, PIO097 
};
#else
#error "Chip not supported"
#endif
EXPORT_SYMBOL(gpio_get_pin_num);

static const struct of_device_id gpio_stb_dt_match[];

/*
 * This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

/**
 * Functions to Support interrupt functionality
 **/

static int stb_gpio_int_set_type(struct irq_data *data, unsigned int trigger)
{
	struct stb_gpio_chip *ach = irq_data_get_irq_chip_data(data);
	u32 val = 0;
	unsigned long flags;

	// printk("%s: IRQBASE: %u HWIRQ: %u irq-ach->irq_base %u\n", __func__, ach->irq_base, irqd_to_hwirq(data), data->irq-ach->irq_base);

	switch (trigger & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		val = 0x2;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	 	val = 0x1;
		break;
	case IRQ_TYPE_EDGE_BOTH:
	 	val = 0x3;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	 	val = 0x0;
		break;
	default:
	    	pr_err("GPIO: invalid IRQ type %u\n", trigger);
		return -EINVAL;
	}

	if (trigger & IRQ_TYPE_LEVEL_HIGH) {
		__irq_set_handler_locked(data->irq, handle_level_irq);
	} else if (trigger & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)) {
		__irq_set_handler_locked(data->irq, handle_edge_irq);
	}

	spin_lock_irqsave(&ach->irq_lock, flags);
	gpset_type(ach->base, irqd_to_hwirq(data), val);
	spin_unlock_irqrestore(&ach->irq_lock, flags);

	return 0;
}

static void stb_gpio_int_mask(struct irq_data *data)
{
	struct stb_gpio_chip *ach = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	//printk("%s: %u\n", __func__, irqd_to_hwirq(data));

	spin_lock_irqsave(&ach->irq_lock, flags);
	gpset_int_enable(ach->base, irqd_to_hwirq(data), 0);
	spin_unlock_irqrestore(&ach->irq_lock, flags);
}

static void stb_gpio_int_unmask(struct irq_data *data)
{
	struct stb_gpio_chip *ach = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	// printk("%s: %u\n", __func__, irqd_to_hwirq(data));

	spin_lock_irqsave(&ach->irq_lock, flags);
	gpset_int_enable(ach->base, irqd_to_hwirq(data), 1);
	spin_unlock_irqrestore(&ach->irq_lock, flags);
}

static void stb_gpio_int_ack(struct irq_data *data)
{
	struct stb_gpio_chip *ach = irq_data_get_irq_chip_data(data);
	unsigned long flags;

	// printk("%s: %u\n", __func__, irqd_to_hwirq(data));

	spin_lock_irqsave(&ach->irq_lock, flags);
	gpclear_int(ach->base, irqd_to_hwirq(data));
	spin_unlock_irqrestore(&ach->irq_lock, flags);
}

static int apget_pending_irq(struct stb_gpio_chip *ach)
{
	int stray = ach->chip.ngpio & 0xF;
	int i, cnt = (ach->chip.ngpio >> 4) + (stray != 0);
	unsigned long flags;
	int ret = 0;

	local_irq_save(flags);
	local_irq_disable();

	for (i = 0; i < cnt; i++) {
		unsigned long val;

		val = gpget_masked_int_status(ach->base, i);
		if (!val)
			continue;

		ret = find_first_bit(&val, sizeof(u32) * BITS_PER_BYTE);
		ret &= 0xF; /*To take care of the interrupt overrun status case in which the last 16bit contains the set status bit*/
		break;
	}

	local_irq_restore(flags);

	if (i >= cnt)
		return 0;

	if (unlikely(ret >= 16)) {
		dev_warn(ach->chip.dev, "Interrupt overflow at IRQ %d, GPIO %d\n",
				ach->irq_base + (ret - 16) + (i << 4),
				ach->chip.base + (ret - 16) + (i << 4));
		return 0;
	}

	// printk(KERN_INFO "apget_pending_irq--> base %x i=%d ret=%d returnval=%d\n",ach->base, i,ret,ret + (i << 4) + 1);

	return ret + (i << 4) + 1;
}

static void handle_stb_gpio_irq(unsigned int parent_irq, struct irq_desc *desc)
{
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct stb_gpio_chip *ach;
	int pend, hcnt = 0;

	chained_irq_enter(irqchip, desc);

	ach = irq_get_handler_data(parent_irq);

	/* Let us dispatch all the IRQs in one shot */
	while((pend = apget_pending_irq(ach))) {
		int irq = gpio_to_irq(ach->chip.base + pend - 1);
		generic_handle_irq(irq);
		hcnt++;
	}

	if (unlikely(!hcnt)) {
		dev_err(ach->chip.dev, "GPIO:: Spurious interrupt @ IRQ %d\n", parent_irq);
	}

	chained_irq_exit(irqchip, desc);
}

static struct irq_chip stb_gpio_irq_chip = {
	.name		= "stb_gpio",
	.irq_mask	= stb_gpio_int_mask,
	.irq_unmask	= stb_gpio_int_unmask,
	.irq_set_type	= stb_gpio_int_set_type,
	.irq_ack       	= stb_gpio_int_ack,
};

static int stb_gpio_int_init(struct stb_gpio_chip *ach)
{
	int ret = 0;
	int gpio;

	for (gpio = 0; gpio < ach->chip.ngpio; gpio++) {
		int irq = irq_create_mapping(ach->irq_domain, gpio);

		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, ach);
		irq_set_chip_and_handler(irq, &stb_gpio_irq_chip,
					 handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	/* replace generic PIH handler (handle_simple_irq) */
	irq_set_handler_data(ach->irq, ach);
	irq_set_chained_handler(ach->irq, handle_stb_gpio_irq);

	return ret;
}

void gpio_set_mode(unsigned int piono, unsigned int pinmode)
{
	unsigned int irq = gpio_to_irq(piono); 	/*unsigned int irq = GPIO0_IRQ_BASE + piono;*/
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);	
	struct irq_data *data = irq_get_irq_data(irq);
	unsigned long flags;

	// printk("%s: GPIO: %u IRQBASE: %d HWIRQ: %d irq-ach->irq_base %ld\n", __func__, piono, ach->irq_base, irqd_to_hwirq(data), data->irq-ach->irq_base);

	spin_lock_irqsave(&ach->lock, flags);
	gpset_mode(ach->base, irqd_to_hwirq(data), pinmode);
	spin_unlock_irqrestore(&ach->lock, flags);
}
EXPORT_SYMBOL(gpio_set_mode);

int gpio_get_mode(unsigned int piono)
{
	unsigned int irq = gpio_to_irq(piono); 	/*unsigned int irq = GPIO0_IRQ_BASE + piono;*/
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	struct irq_data *data = irq_get_irq_data(irq);
	return gpget_mode(ach->base, irqd_to_hwirq(data));
}
EXPORT_SYMBOL(gpio_get_mode);

#if 1 /*remove only for test*/
void stb_gpio_set_interrupt(unsigned int piono)
{
	unsigned int irq = gpio_to_irq(piono); 	/*unsigned int irq = GPIO0_IRQ_BASE + piono;*/
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	struct irq_data *data = irq_get_irq_data(irq);
	unsigned long flags;

	// printk("%s: GPIO: %u IRQBASE: %d HWIRQ: %d irq-ach->irq_base %ld\n", __func__, piono, ach->irq_base, irqd_to_hwirq(data), data->irq-ach->irq_base);
	spin_lock_irqsave(&ach->irq_lock, flags);

	gpset_int(ach->base, irqd_to_hwirq(data));
	spin_unlock_irqrestore(&ach->irq_lock, flags);
}
EXPORT_SYMBOL(stb_gpio_set_interrupt);
#endif

int gpio_interrupt_set_type(unsigned piono, unsigned int trigger)
{
	unsigned int irq = gpio_to_irq(piono); 	/*unsigned int irq = GPIO0_IRQ_BASE + piono;*/
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	struct irq_data *data = irq_get_irq_data(irq);
	u32 val = 0;
	unsigned long flags;

	// printk("%s: IRQBASE: %d HWIRQ: %d irq-ach->irq_base %ld\n", __func__, ach->irq_base, data->hwirq, data->irq-ach->irq_base);

	switch (trigger & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		val = 0x2;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	 	val = 0x1;
		break;
	case IRQ_TYPE_EDGE_BOTH:
	 	val = 0x3;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	 	val = 0x0;
		break;
	default:
	    	pr_err("GPIO: invalid IRQ type %u\n", trigger);
		return -EINVAL;
	}

	if (trigger & IRQ_TYPE_LEVEL_HIGH) {
		__irq_set_handler_locked(irq, handle_level_irq);
	} else if (trigger & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)) {
		__irq_set_handler_locked(irq, handle_edge_irq);
	}

	spin_lock_irqsave(&ach->irq_lock, flags);
	gpset_type(ach->base, irqd_to_hwirq(data), val);
	spin_unlock_irqrestore(&ach->irq_lock, flags);

	return 0;

}
EXPORT_SYMBOL(gpio_interrupt_set_type);

/* For testing purpose */
int gpio_interrupt_get_type(unsigned piono)
{
	unsigned int irq = gpio_to_irq(piono); 	
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	struct irq_data *data = irq_get_irq_data(irq);

	return gpget_type(ach->base, irqd_to_hwirq(data));
}
EXPORT_SYMBOL(gpio_interrupt_get_type);


void gpio_interrupt_clear(unsigned int piono)
{
	unsigned int irq = gpio_to_irq(piono);
	struct irq_data *d = irq_get_irq_data(irq);	

	stb_gpio_int_ack(d);
}
EXPORT_SYMBOL(gpio_interrupt_clear);


int gpio_is_interrupt_enabled(unsigned int piono)
{
	unsigned int irq = gpio_to_irq(piono); 	
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	struct irq_data *data = irq_get_irq_data(irq);
	int isirqenabled = 0;

	isirqenabled = gpget_int_enable(ach->base, irqd_to_hwirq(data));

	return isirqenabled;
}
EXPORT_SYMBOL(gpio_is_interrupt_enabled);

int gpio_is_interrupt_pending(unsigned int piono)
{
	unsigned int irq = gpio_to_irq(piono); 	
	struct irq_data *data = irq_get_irq_data(irq);
	struct stb_gpio_chip *ach = irq_get_chip_data(irq);
	int isirqpending = 0;
	unsigned long val=0;
	unsigned int pioregindex, piobitno;
	
	pioregindex = irqd_to_hwirq(data) >> 4;
	piobitno = irqd_to_hwirq(data) & 0xF; 
	val = gpget_masked_int_status(ach->base, pioregindex);
	if ( (1 << piobitno) & val) {
		isirqpending = 1;
	}
	
	return isirqpending;
}
EXPORT_SYMBOL(gpio_is_interrupt_pending);

/**
 * Functions to support pins functionality
 * of GPIO. Must be used with gpiolib :)
 **/
static int stb_gpio_direction_in(struct gpio_chip *chip,
						unsigned offset)
{
	struct stb_gpio_chip *ach = to_ach(chip);
	unsigned long flags;

	BUG_ON(offset >= chip->ngpio);

	spin_lock_irqsave(&ach->lock, flags);
	/* Set mode to GPIO */
	gpset_mode(ach->base, offset, GPIO_MODE_IO);
	gpset_dir(ach->base, offset, GPIO_DIR_IN);
	spin_unlock_irqrestore(&ach->lock, flags);
	return 0;
}

static int stb_gpio_direction_out (struct gpio_chip *chip,
						unsigned offset, int value)
{
	struct stb_gpio_chip *ach = to_ach(chip);
	unsigned long flags;

	BUG_ON(offset >= chip->ngpio);

	spin_lock_irqsave(&ach->lock, flags);
	/* Set mode to GPIO */
	gpset_dir(ach->base, offset, 
			(value & 1) ? GPIO_DIR_OUT_HI : GPIO_DIR_OUT_LO);
	gpset_mode(ach->base, offset, GPIO_MODE_IO);
	spin_unlock_irqrestore(&ach->lock, flags);
	return 0;
}

static void stb_gpio_set(struct gpio_chip *chip,
						unsigned offset, int value)
{
	struct stb_gpio_chip *ach = to_ach(chip);
	unsigned long flags;

	BUG_ON(offset >= chip->ngpio);
	spin_lock_irqsave(&ach->lock, flags);
	gpset_dir(ach->base, offset, 
			(value & 1) ? GPIO_DIR_OUT_HI : GPIO_DIR_OUT_LO);
	spin_unlock_irqrestore(&ach->lock, flags);
}

static int stb_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct stb_gpio_chip *ach = to_ach(chip);

	BUG_ON(offset >= chip->ngpio);

	return gpget_value(ach->base, offset);
}

static int stb_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct stb_gpio_chip *ach = to_ach(chip);

	return irq_find_mapping(ach->irq_domain, offset);
}

static void apgpio_init_chip(struct stb_gpio_chip *ach)
{
	ach->chip.label		= "gpio_stb";
	ach->chip.owner		= THIS_MODULE;
	ach->chip.can_sleep	= 0;

	/**
	 * All the functions are called with spin locked
	 * So, we don't have to bother about SMP race.
	 **/
#if 0
	/**
	 * TODO: May be useful later
	 **/
	ach->chip.request           = stb_gpio_request;
	ach->chip.free              = stb_gpio_free;
#endif
	ach->chip.direction_input   = stb_gpio_direction_in;
	ach->chip.direction_output  = stb_gpio_direction_out;
	ach->chip.get               = stb_gpio_get;
	ach->chip.set               = stb_gpio_set;
	ach->chip.to_irq            = stb_gpio_to_irq;
}

/**
 * Platform driver init and exit functions
 **/
static int gpio_stb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	struct stb_gpio_platform_data *pdata;
	int ret = 0;
	struct stb_gpio_chip *ach;
	struct resource *res;
	int i;

	match = of_match_device(gpio_stb_dt_match, dev);
	if (unlikely(!match)) {
		dev_err(dev, "GPIO parameters invalid\n");
		return -EINVAL;
	}

	pdata = (struct stb_gpio_platform_data *)match->data;
	if (unlikely(pdata == NULL) || unlikely(pdata->nr_gpio <= 0)) {
		dev_err(dev, "GPIO parameters invalid\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(dev, "No Memory resource found\n");
		return -ENXIO;
	}

	ach = kzalloc(sizeof(*ach), GFP_KERNEL);
	if (unlikely(ach == NULL)) {
		dev_err(dev, "Unable to allocate chip memory\n");
		return -ENOMEM;
	}

	ach->irq = platform_get_irq(pdev, 0);
	if (unlikely(!ach->irq)) {
		dev_err(dev, "No IRQ resource found\n");
		ach->no_irq = 1;
	} else if (is_module()) {
		dev_warn(dev, "Interrupt cannot be dispatched "
				"from a module!\n");
		ach->no_irq = 1;
	} else if (unlikely(!pdata->irq_base)) 
		ach->no_irq = 1;

	ach->irq_base = irq_alloc_descs(-1, pdata->irq_base, pdata->nr_gpio, -1);
	if (ach->irq_base < 0) {
		dev_err(dev, "Failed to allocate IRQ numbers\n");
		kfree(ach);
		return ach->irq_base;
	}

	ach->irq_domain = irq_domain_add_legacy(node, pdata->nr_gpio, ach->irq_base,
						0, &irq_domain_simple_ops, NULL);
	if (!ach->irq_domain) {
		dev_err(dev, "Couldn't register an IRQ domain\n");
		irq_free_descs(ach->irq_base, pdata->nr_gpio);
		kfree(ach);
		return -ENODEV;
	}

	ach->chip.dev     = dev;
	ach->chip.base    = pdata->first_pin;
	ach->chip.ngpio   = pdata->nr_gpio;
	ach->chip.of_node = node;
	apgpio_init_chip(ach);

	ach->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ach->base)) {
		dev_err(dev, "Unable to ioremap the address range\n");
		kfree(ach);
		return PTR_ERR(ach->base);
	}

	pr_info("GPIO: base %p (phys %x)\n", ach->base, res->start);

	if (unlikely(!apgpio_check_moduleid(ach->base))) {
		dev_err (dev, "No GPIO device found @ %p\n",
				(void __iomem *) res->start);
		kfree(ach);
		return -ENODEV;
	}

	spin_lock_init(&ach->lock);
	spin_lock_init(&ach->irq_lock);

	/* Disable and clear all interrupts at this point */
	for (i = 0; i < (pdata->nr_gpio >> 5) + ((pdata->nr_gpio & 0x1F) != 0); i++) {
		writel(0, ach->base + GPIO_INT_ENABLE + (i * 4));
 		writel(0xFFFFFFFF, ach->base + GPIO_INT_CLEAR + (i * 4));
	}

	for (i = 0; i < (pdata->nr_gpio >> 4) + ((pdata->nr_gpio & 0x0F) != 0); i++) {
		writel(0x55555555, ach->base + GPIO_CTRL_REG + (i * 4)); /* All low-edge */
	}

	ret = gpiochip_add(&ach->chip);
	if (ret < 0) {
		dev_err (dev, "Unable to register GPIO Chip\n");
		kfree(ach);
		return ret;
	}

	if(!ach->no_irq)
		stb_gpio_int_init(ach);
	else
		dev_warn(dev, "No Interrupt controller "
			      "functionality will be available\n");

	return 0;
}

#if 0
static int gpio_stb_remove(struct platform_device *pdev)
{
	/**
	 * We only have to free our device structure
	 * everything else is removed by devm layer.
	 * If we are here we had definitely skipped
	 * interrupt part in probe :)
	 **/
	kfree(platform_get_drvdata(pdev));
	return 0;
}
#endif

#ifdef CONFIG_OF
static struct stb_gpio_platform_data gpio0_platform_data = {
        .nr_gpio        = GPIO0_MAX_NR,
        .irq_base       = GPIO0_IRQ_BASE,
        .first_pin      = GPIO0_START_PIN
};

static struct stb_gpio_platform_data gpio1_platform_data = {
        .nr_gpio	= GPIO1_MAX_NR,
        .irq_base	= GPIO1_IRQ_BASE,
        .first_pin	= GPIO1_START_PIN
};

static const struct of_device_id gpio_stb_dt_match[] = {
        { .compatible = "entr,stb-gpio0", .data = &gpio0_platform_data },
        { .compatible = "entr,stb-gpio1", .data = &gpio1_platform_data },
        {},
};
MODULE_DEVICE_TABLE(of, gpio_stb_dt_match);
#endif

static struct platform_driver gpio_stb_driver = {
	.probe		= gpio_stb_probe,
	/*.remove	= __devexit_p(gpio_stb_remove),*/
	.driver		= {
			.name 		= "gpio_stb",
			.owner		= THIS_MODULE,
			.of_match_table	= gpio_stb_dt_match,
	}
};

/**
 * This call must be under subsys_initcall as
 * Some of the GPIOs are used as interrupt pins
 * So, before any init call of driver is made
 * GPIO must be set up and ready
 */
static int __init gpio_stb_init(void)
{
	return platform_driver_register(&gpio_stb_driver);
}
subsys_initcall(gpio_stb_init);

static void __exit gpio_stb_exit(void)
{
	platform_driver_unregister(&gpio_stb_driver);
}
module_exit(gpio_stb_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STB Platform GPIO device driver");
