/************************************************************************************
 *  linux/include/asm-arm/arch-nevis/irqs.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
 *  Copyright (C) 2008 Coolstream International Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as published by
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
 *************************************************************************************/

#ifndef __ASM_ARCH_NEVIS_IRQS_H
#define __ASM_ARCH_NEVIS_IRQS_H

/* Use the architecture-specific definitions */
#include <asm/arch/platform.h>

/* Misc. interrupt definitions */
#define NR_IRQS			352
#define VALID_PRI_INT_SRC	0x0000FFFF	/* non-reserved bits in primary pic */
#define VALID_SEC_INT_SRC	0xFFFFC3FF	/* non-reserved bits in secondary pic */

#define IRQ_UART1		 0		/* first UART */
#define IRQ_UART2		 1		/* second UART */
#define IRQ_UART3		 2		/* third UART */
#define IRQ_I2C0		 6		/* first I2C bus */
#define IRQ_I2C1		 7		/* second I2C bus */
#define IRQ_IR0			 9		/* first Infrared receiver */
#define IRQ_IR1			10		/* second Infrared receiver */
#define IRQ_USB0		13		/* first USB interface */
#define IRQ_GPIOC		14		/* GPIO controller (global interrupt for chained handler) */
#define IRQ_USB1		17		/* second USB interface */
#define IRQ_I2C2		19		/* third I2C bus */
#define IRQ_USB2		20		/* third USB interface */
#define IRQ_EMAC0		24		/* first Media Access Controller */
#define IRQ_EMAC1		25		/* second Media Access Controller */
#define IRQ_I2C3		27		/* fourth I2C bus */
#define IRQ_SATA1		35		/* second SATA controller */
#define IRQ_PCI			37		/* PCI controller */
#define IRQ_SATA0		42		/* first SATA controller */
#define IRQ_SC(x)		(  3 + (x))	/* first of the two smardcard interface controllers */
#define IRQ_IR(x)		(  9 + (x))	/* first of the two Infrared remote controllers */
#define IRQ_TIMER(x)		( 64 + (x))	/* first of the 16 hardware timer starts at IRQ 64 (count 0 to 15) */
#define IRQ_GPIO(x)		(128 + (x))	/* first of the 224 GPIO's starts at IRQ 128 (count 0 to 223) */

#endif /* __ASM_ARCH_IRQS_H */
