/*****************************************************************************
 *  linux/arch/arm/mach-nevis/irq.c
 *
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
 *  Copyright (C) 2008 Coolstream International
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/arch/param.h>
#include <asm/arch/irq.h>
#include <asm/arch/cx2450x.h>

static volatile u32 *intena_reg[4] = {
	(volatile u32 *)ITC_ENABLE_REG_BASE(0),
	(volatile u32 *)ITC_ENABLE_REG_BASE(1),
	(volatile u32 *)ITC_ENABLE_REG_BASE(2),
	(volatile u32 *)ITC_ENABLE_REG_BASE(3)
};

static volatile u32 *intclr_reg[4] = {
	(volatile u32 *)ITC_STATCLR_REG_BASE(0),
	(volatile u32 *)ITC_STATCLR_REG_BASE(1),
	(volatile u32 *)ITC_STATCLR_REG_BASE(2),
	(volatile u32 *)ITC_STATCLR_REG_BASE(3)
};

static volatile u32 *intreq_reg[4] = {
	(volatile u32 *)ITC_IRQREQ_REG_BASE(0),
	(volatile u32 *)ITC_IRQREQ_REG_BASE(1),
	(volatile u32 *)ITC_IRQREQ_REG_BASE(2),
	(volatile u32 *)ITC_IRQREQ_REG_BASE(3),
};

static volatile u32 *piointstat_reg[7] = {
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(0),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(1),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(2),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(3),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(4),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(5),
	(volatile u32 *)GPIO_INTSTAT_REG_BASE(6)
};

static volatile u32 *piointena_reg[7] = {
	(volatile u32 *)GPIO_INTENA_REG_BASE(0),
	(volatile u32 *)GPIO_INTENA_REG_BASE(1),
	(volatile u32 *)GPIO_INTENA_REG_BASE(2),
	(volatile u32 *)GPIO_INTENA_REG_BASE(3),
	(volatile u32 *)GPIO_INTENA_REG_BASE(4),
	(volatile u32 *)GPIO_INTENA_REG_BASE(5),
	(volatile u32 *)GPIO_INTENA_REG_BASE(6)
};

/* a simple array to mask out one of 32 possible interrupts */
static const u32 irq_mask[128] = {
	/* Group 1 Interrupts */
	0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000,	0x40000000, 0x80000000,
	/* Group 2 Interrupts */
	0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
	/* Group 3 Interrupts */
	0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
	/* Group 4 Interrupts */
	0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
};

static const u32 irq_idx[128] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
};

#ifdef ENABLE_LATENCY_TIMER
/**************************************************/
/*    TYPEDEFs                                    */
/**************************************************/
struct s_latency latency = {
	.count	= 0,
};

/**************************************************/
/*    STATIC GLOBALS                              */
/**************************************************/

int get_latency_list(char *buf)
{
	int min = 1000000, avg = 0, max = 0;
	int mindur = 1000000, avgdur = 0, maxdur = 0;
	int i;
	char *p = buf;

	p += sprintf(p,
		     "Sample size = %d.  freq(sample) = freq(tick) (%d mS)\n\n",
		     LATENCY_COUNT, 1000 / HZ);

	for (i = 0; i < LATENCY_COUNT; i++) {
		if (latency.duration[i] < mindur)
			mindur = latency.duration[i];
		if (latency.duration[i] > maxdur)
			maxdur = latency.duration[i];
		if (latency.delta[i] < min)
			min = latency.delta[i];
		if (latency.delta[i] > max)
			max = latency.delta[i];
		avg	+= latency.delta[i];
		avgdur	+= latency.duration[i];
	}
	p += sprintf(p, "min lat : %d uS\n", min / 54);
	p += sprintf(p, "avg lat : %d uS\n", avg / (LATENCY_COUNT * 54));
	p += sprintf(p, "max lat : %d uS\n", max / 54);

	p += sprintf(p, "min dur : %d uS\n", mindur);
	p += sprintf(p, "avg dur : %d uS\n", avgdur / LATENCY_COUNT);
	p += sprintf(p, "max dur : %d uS\n", maxdur);

	return p - buf;
}
#endif

/*******************************************************************************/

/* Interrupt handlers for the main interrupt controller */
static void ack_irq_itc(u32 irq)
{
	u32 tmp;
	tmp = *intreq_reg[irq_idx[irq]];
#if 0
	*intclr_reg[irq_idx[irq]] = irq_mask[irq];	/* clear the interrupt */
#endif
	*intena_reg[irq_idx[irq]] &= ~irq_mask[irq];	/* disable the interrupt */
}

static void mask_irq_itc(u32 irq)
{
	u32 tmp;
	tmp = *intreq_reg[irq_idx[irq]];
	*intena_reg[irq_idx[irq]] &= ~irq_mask[irq];	/* disable the interrupt */
}

static void unmask_irq_itc(u32 irq)
{
	*intclr_reg[irq_idx[irq]] = irq_mask[irq];	/* clear the interrupt */
	*intena_reg[irq_idx[irq]] |= irq_mask[irq];	/* enable interrupt */
}

/*******************************************************************************/

#ifdef CONFIG_PCI
/* Interrupt Handlers for the PCI-controller. We use an own 'chip' here, even if
   the IRQ comes from the normal ITC. The reason is, that, beside resetting the
   status in the ITC, also the status in the PCI-controller must be set. PCI-
   devicedrivers didn't know the PCI-Controller itself, so we must setup it here.
   If we put INT_PCI into the main handler, we need a "if" statement to filter
   this IRQ, which costs time, much time */

static void ack_irq_pci(u32 irq)
{
	volatile u32 *pcistat_reg = (volatile u32 *)PCI_INTSTAT_REG;

	*intena_reg[irq_idx[irq]] &= ~irq_mask[irq];	/* disable the interrup */
	*intclr_reg[irq_idx[irq]] = irq_mask[irq];	/* clear the interrupt */
	*pcistat_reg = 1;	/* set INTA and reset all other stats in PCI-controller */
}

static void mask_irq_pci(u32 irq)
{
	*intena_reg[irq_idx[irq]] &= ~irq_mask[irq];	/* disable the interrupt */
}

static void unmask_irq_pci(u32 irq)
{
	*intena_reg[irq_idx[irq]] |= irq_mask[irq];	/* enable the interrupt */
}
#endif
/*******************************************************************************/

/* Interrupt handlers for the general purpose I/O pins
 * (if not used by other fuctions)
 */
static void ack_irq_gpio(u32 irq)
{
	u32 bank, bit;

	irq -= IRQ_GPIO(0);
	bank = irq / 32;
	bit = irq % 32;
	*piointena_reg[bank] &= ~irq_mask[bit];
	*piointstat_reg[bank] |= irq_mask[bit];
}

static void mask_irq_gpio(u32 irq)
{
	irq -= IRQ_GPIO(0);
	*piointena_reg[irq / 32] &= ~irq_mask[irq % 32];
}

static void unmask_irq_gpio(u32 irq)
{
	irq -= IRQ_GPIO(0);
	*piointena_reg[irq / 32] |= irq_mask[irq % 32];
}

static int set_type_irq_gpio(u32 irq, u32 type)
{
	u32 bank, bit;
	volatile u32 *piolevel_reg;
	volatile u32 *piopedge_reg;
	volatile u32 *pionedge_reg;

	if (irq < IRQ_GPIO(0))
		return -EINVAL;

	irq -= IRQ_GPIO(0);
	bank = irq / 32;
	bit = irq % 32;

	piolevel_reg = (volatile u32 *)(GPIO_LEVEL_REG_BASE(bank));
	piopedge_reg = (volatile u32 *)(GPIO_POS_EDGE_REG_BASE(bank));
	pionedge_reg = (volatile u32 *)(GPIO_NEG_EDGE_REG_BASE(bank));

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:	/* High level trigger */
		*piopedge_reg &= ~irq_mask[bit];	/* disable rising edge trigger */
		*pionedge_reg &= ~irq_mask[bit];	/* disable falling edge trigger */
		*piolevel_reg |= irq_mask[bit];	/* enable high level trigger */
		break;
	case IRQ_TYPE_LEVEL_LOW:	/* Low level trigger */
		*piopedge_reg &= ~irq_mask[bit];	/* disable rising edge trigger */
		*pionedge_reg &= ~irq_mask[bit];	/* disable falling edge trigger */
		*piolevel_reg &= ~irq_mask[bit];	/* disable high level trigger */
		break;
	case IRQ_TYPE_EDGE_RISING:	/* Rising edge trigger */
		*pionedge_reg &= ~irq_mask[bit];	/* disable falling edge trigger */
		*piopedge_reg |= irq_mask[bit];	/* enable rising edge trigger */
		break;
	case IRQ_TYPE_EDGE_FALLING:	/* Falling edge trigger */
		*piopedge_reg &= ~irq_mask[bit];	/* disable rising edge trigger */
		*pionedge_reg |= irq_mask[bit];	/* enable falling edge trigger */
		break;
	case IRQ_TYPE_EDGE_BOTH:	/* Rising AND Falling edge trigger */
		*piopedge_reg |= irq_mask[bit];	/* enable rising edge trigger */
		*pionedge_reg |= irq_mask[bit];	/* enable falling edge trigger */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************/

static void nevis_irq_handle_gpio_chain(unsigned int irq, struct irq_desc *desc)
{
	u32 bank, bit, irqno;
	u32 active;

	for (bank = 0; bank < 7; bank++) {
		active = *piointena_reg[bank] & *piointstat_reg[bank];
		if (!active)
			continue;	/* no pending interrupt in this bank */

		for(bit = 0; bit < 32; bit++) {
			if (active & irq_mask[bit]) {
				irqno = IRQ_GPIO(0) + (bank << 5) + bit;
				desc_handle_irq(irqno, irq_desc + irqno);
			}
		}
	}
}

/*******************************************************************************/

static struct irq_chip cx2450x_itc = {
	.name	= "ITC",
	.mask	= mask_irq_itc,
	.unmask	= unmask_irq_itc,
	.ack	= ack_irq_itc
};

#ifdef CONFIG_PCI
static struct irq_chip cx2450x_chip_pci = {
	.name	= "PCI",
	.mask	= mask_irq_pci,
	.unmask	= unmask_irq_pci,
	.ack	= ack_irq_pci
};
#endif

static struct irq_chip cx2450x_chip_gpio = {
	.name	= "GPIO",
	.mask	= mask_irq_gpio,
	.unmask	= unmask_irq_gpio,
	.ack	= ack_irq_gpio,
	.set_type = set_type_irq_gpio
};

/*******************************************************************************/

void __init cx2450x_init_irq(void)
{
	u32 irq;
	volatile u32 *reg;

	/* Disable Interrupt Generation */
	*intena_reg[0] = 0x00000000;
	*intena_reg[1] = 0x00000000;
	*intena_reg[2] = 0x00000000;
	*intena_reg[3] = 0x00000000;

	for (irq = 0; irq < NR_IRQS; irq++) {
#if defined(SKIP_LATCHED_TIMER)
		/* dont interrupt for the 2nd timer unlatched or register */
		if (irq == (32 + 7))
			continue;
#endif
#if defined(SKIP_LATCHED_GPIO)
		if (irq == (64 + 24))
			continue;
#endif

		switch (irq) {
		case 0 ... (IRQ_PCI - 1): /* interrupts from the 4 banks of the ITC except PCI */
		case (IRQ_PCI + 1) ... 127:
			set_irq_chip(irq, &cx2450x_itc);
			set_irq_flags(irq, IRQF_VALID);
			set_irq_handler(irq, handle_level_irq);
			break;
#ifdef CONFIG_PCI
		case IRQ_PCI:		/* for speedup, the PCI interrupt is handled by it's own "ITC" */
			set_irq_chip(irq, &cx2450x_chip_pci);
			set_irq_flags(irq, IRQF_VALID);
			set_irq_handler(irq, handle_level_irq);
			break;
#endif
		case 128 ... 351:	/* PIO 0 to 223 in 7 banks */
			set_irq_chip(irq, &cx2450x_chip_gpio);
			set_irq_flags(irq, IRQF_VALID);
			set_irq_handler(irq, handle_level_irq);
			break;
		default:
			break;
		}
	}

	/* setup chained handler for the GPIO controller */
	set_irq_chained_handler(IRQ_GPIOC, nevis_irq_handle_gpio_chain);

	/* Enable IRQ mode */
	reg = (volatile u32 *)ITC_DEST_REG_BASE(0);
	*reg = 0xFFFFFFFF;
	reg = (volatile u32 *)ITC_DEST_REG_BASE(1);
	*reg = 0xFFFFFFFF;
	reg = (volatile u32 *)ITC_DEST_REG_BASE(2);
	*reg = 0xFFFFFFFF;
	reg = (volatile u32 *)ITC_DEST_REG_BASE(3);
	*reg = 0xFFFFFFFF;
}
