/****************************************************************************
 *
 *  include/asm-arm/arch-pecos/debug-macro.S
 *
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
 *  Copyright (C) 2008 CoolStream International Limited.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2 as published by
 *  the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 *
 ****************************************************************************/
		/* .equ	io_virt, ASX_VADDR_BASE @ 0xFD00.0000 */
		/* .equ	io_phys, ASX_PHYS_BASE  @ 0xE000.0000 */
		.equ	io_virt, 0xE0000000 @ 0xFD00.0000  
		.equ	io_phys, 0xE0000000 @ 0xE000.0000

		.macro	addruart,rx
		mrc     p15, 0, \rx, c1, c0
		tst     \rx, #1			@ MMU enabled?
		mov     \rx, #0x2000
		orr	\rx, #0x410000
		orreq   \rx, \rx, #io_phys	@ ASX physical base address
		orrne   \rx, \rx, #io_virt	@ ASX virtual address
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx]		@ SER_DATA_REG
		.endm

		.macro	waituart,rd,rx
1001:		ldrb	\rd,    [\rx, #0x14]    @ SER_STATUS_REG
		tst     \rd, #(1 << 6)            @ SER_STATUS_TXIDLE
		beq     1001b
		.endm

		.macro	busyuart,rd,rx
1002:           ldrb     \rd,[\rx, #0x14]	@ SER_STATUS_REG
                tst     \rd, #(1 << 6)		@ SER_STATUS_TXIDLE
		bne	1002b
		.endm

