/* linux/arch/arm/mach-nevis/devices.c
 *
 * Platform device definitions and helper functions for systems based
 * on the Conexant CX2450x (Nevis) SoC
 *
 * Copyright (C) 2008 CoolStream International Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/memory.h>
#include "devices.h"

static u64 cx2450x_device_dmamask = DMA_BIT_MASK(32);

/* 
 * USB Host Controllers
 */
#ifdef CONFIG_CX2450X_USB0
static struct resource cx2450x_usb_res0[] = {
	[0] = {
	       .start = 0xE8000100,
	       .end = 0xE80001FB,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_USB0,
	       .end = IRQ_USB0,
	       .flags = IORESOURCE_IRQ,
	       }
};
#endif

#ifdef CONFIG_CX2450X_USB1
static struct resource cx2450x_usb_res1[] = {
	[0] = {
	       .start = 0xE8001100,
	       .end = 0xE80011FB,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_USB1,
	       .end = IRQ_USB1,
	       .flags = IORESOURCE_IRQ,
	       }
};
#endif

#ifdef CONFIG_SERIAL_CNXT_UART
#if CONFIG_SERIAL_CNXT_UART >= 1

/*
 * Serial port #0, by default we use Conexant UART #3 for console etc.
 * Resources from 0xE041x00 - 0xE041x3C, where x is 0, 0x1000, 0x2000, 0x3000
 * Note: we reversed the mappings because the serial console is hooked up at UART #3.
 *	 This simplifies the mapping. So ttyRI0 will be the real console.
 *	 WARNING: Do not change the id because the driver depends on this.
 */
static struct resource cx2450x_ser0_res[] = {
	[0] = {
	       .start = 0xE0412000,
	       .end   = 0xE041203F,
	       .flags = IORESOURCE_MEM,
	      },
	[1] = {
	       .start = IRQ_UART3,
	       .end   = IRQ_UART3,
	       .flags = IORESOURCE_IRQ,
	      }
};

/*******************************************************************************/

struct platform_device cx2450x_device_ser0 = {
	.name = "cnxt_uart",
	.id = 0,
	.num_resources = ARRAY_SIZE (cx2450x_ser0_res),
	.resource = cx2450x_ser0_res,
	.dev = {
		.dma_mask = &cx2450x_device_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK (32),
		}
};
#endif

/*******************************************************************************/

#if CONFIG_SERIAL_CNXT_UART >= 2
static struct resource cx2450x_ser1_res[] = {
	[0] = {
	       .start = 0xE0411000,
	       .end   = 0xE041103F,
	       .flags = IORESOURCE_MEM,
	      },
	[1] = {
	       .start = IRQ_UART2,
	       .end   = IRQ_UART2,
	       .flags = IORESOURCE_IRQ,
	      }
};

/*******************************************************************************/

struct platform_device cx2450x_device_ser1 = {
	.name = "cnxt_uart",
	.id = 1,
	.num_resources = ARRAY_SIZE (cx2450x_ser1_res),
	.resource = cx2450x_ser1_res,
	.dev = {
		.dma_mask = &cx2450x_device_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK (32),
	       }
};
#endif

/*******************************************************************************/

#if CONFIG_SERIAL_CNXT_UART >= 3
static struct resource cx2450x_ser2_res[] = {
	[0] = {
	       .start = 0xE0410000,
	       .end   = 0xE041003B,
	       .flags = IORESOURCE_MEM,
	      },
	[1] = {
	       .start = IRQ_UART1,
	       .end   = IRQ_UART1,
	       .flags = IORESOURCE_IRQ,
	      }
};

/*******************************************************************************/

struct platform_device cx2450x_device_ser2 = {
	.name = "cnxt_uart",
	.id = 2,
	.num_resources = ARRAY_SIZE (cx2450x_ser2_res),
	.resource = cx2450x_ser2_res,
	.dev = {
		.dma_mask = &cx2450x_device_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK (32),
		}
};

#endif
#endif

/*******************************************************************************/

#ifdef CONFIG_CX2450X_USB0
struct platform_device cx2450x_device_usb0 = {
	.name = "cx2450x-ehci",
	.id = 0,
	.num_resources = ARRAY_SIZE (cx2450x_usb_res0),
	.resource = cx2450x_usb_res0,
	.dev = {
		.dma_mask = &cx2450x_device_dmamask,
		.coherent_dma_mask = 0xFFFFFFFFUL}
};
#endif

#ifdef CONFIG_CX2450X_USB1
struct platform_device cx2450x_device_usb1 = {
	.name = "cx2450x-ehci",
	.id = 1,
	.num_resources = ARRAY_SIZE (cx2450x_usb_res1),
	.resource = cx2450x_usb_res1,
	.dev = {
		.dma_mask = &cx2450x_device_dmamask,
		.coherent_dma_mask = 0xFFFFFFFFUL}
};
#endif

/*******************************************************************************/

#ifdef CONFIG_CX2450X_USB0
EXPORT_SYMBOL (cx2450x_device_usb0);
#endif

#ifdef CONFIG_CX2450X_USB1
EXPORT_SYMBOL (cx2450x_device_usb1);
#endif

#ifdef CONFIG_SERIAL_CNXT_UART
#if CONFIG_SERIAL_CNXT_UART >= 1
EXPORT_SYMBOL (cx2450x_device_ser0);
#endif
#if CONFIG_SERIAL_CNXT_UART >= 2
EXPORT_SYMBOL (cx2450x_device_ser1);
#endif
#if CONFIG_SERIAL_CNXT_UART >= 3
EXPORT_SYMBOL (cx2450x_device_ser2);
#endif
#endif
