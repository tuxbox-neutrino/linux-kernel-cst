/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author: Srinivas Rao L <srinivas.rao@entropic.com>
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


#include <mach/soc.asmh>

#ifdef CONFIG_DEBUG_LL
#define DEBUG_UART_PHY_OFFSET (UART1_PHY_OFFSET+SOC_UART_MMIO_SIZE*(CONFIG_DEBUG_STB_UART-1))
#define DEBUG_UART_VIRT_OFFSET (UART1_VIRT_OFFSET+SOC_UART_MMIO_SIZE*(CONFIG_DEBUG_STB_UART-1))

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =DEBUG_UART_PHY_OFFSET	@ Physical base
	ldr	\rv, =DEBUG_UART_VIRT_OFFSET	@ Virtual base
	.endm

	.macro	senduart, rd, rx
	strb	\rd, [\rx ]			@ THR_Buffer
	.endm

	.macro	waituart, rd, rx
1:	ldr	\rd, [\rx, #0x14]    		@ Line Status
	ands	\rd, \rd, #0x20		 	@ THR Empty
	beq	1b
	.endm

	.macro	busyuart, rd, rx
	ldr	\rd, [\rx, #0x14]		@ Line Status  
#2:	ldr	\rd, [\rx, #0x14]		@ 
#	ands	\rd, \rd, #0x20			@ THR Full
#	bne	2b
	.endm
#endif
