/* arch/arm/mach-nevis/devs.h
 *
 * Header file for cx2450x platform devices
 *
 * Copyright (C) 2008 Coolstream International Limited
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
 * Modifications:
*/
#ifndef __DEVICES_H
#define __DEVICES_H

#include <linux/autoconf.h>
#include <linux/platform_device.h>

#ifdef CONFIG_CX2450X_USB0
extern struct platform_device cx2450x_device_usb0;	/* USB EHCI Controller #0 */
#endif
#ifdef CONFIG_CX2450X_USB1
extern struct platform_device cx2450x_device_usb1;	/* USB EHCI Controller #1 */
#endif

#ifdef CONFIG_SERIAL_CNXT_UART
#if CONFIG_SERIAL_CNXT_UART >= 1
extern struct platform_device cx2450x_device_ser0;	/* UART #2 */
#endif
#if CONFIG_SERIAL_CNXT_UART >= 2
extern struct platform_device cx2450x_device_ser1;	/* UART #1 */
#endif
#if CONFIG_SERIAL_CNXT_UART >= 3
extern struct platform_device cx2450x_device_ser2;	/* UART #0 */
#endif
#endif

#endif /* __DEVICES_H */
