/******************************************************************************
 * include/asm-arm/arch-nevis/entry-macro.S
 *
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
 *  Copyright (C) 2010 CoolStream Int Ltd.
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

#include <asm/arch/cx2450x.h>
#define MAXIRQNUM 32

		.equ itc_intstatset_1   , ITC_STATSET_REG_BASE(0)
		.equ itc_intstatenable_1, ITC_ENABLE_REG_BASE(0)
		.equ itc_intstatset_2   , ITC_STATSET_REG_BASE(1)
		.equ itc_intstatenable_2, ITC_ENABLE_REG_BASE(1)
		.equ itc_intstatset_3   , ITC_STATSET_REG_BASE(2)
		.equ itc_intstatenable_3, ITC_ENABLE_REG_BASE(2)
		.equ itc_intstatset_4   , ITC_STATSET_REG_BASE(3)
		.equ itc_intstatenable_4, ITC_ENABLE_REG_BASE(3)

		.macro  disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
		/* Check IRQ controller #0 */
		ldr	\tmp, =itc_intstatset_1
		ldr	\irqstat, [\tmp]
		ldr	\tmp, =itc_intstatenable_1
		ldr	\tmp, [\tmp]
		ands	\irqstat, \irqstat, \tmp	@ Any IRQ set ?
		beq	1001f				@ No IRQ set, check IRQ block 1

		mov	\irqnr, #0			@ IRQ nr = 0
1100:		movs	\irqstat, \irqstat, lsr #1	@ Shift bit into carry
		bcs	1004f				@ IRQ found, get out of here.
		add	\irqnr, \irqnr, #1		@ Increase irqnr
		bne	1100b				@ Check next IRQ
		b	1005f				@ Uhhh...

1001:		/* Check IRQ controller #1 */
		ldr	\tmp, =itc_intstatset_2
		ldr	\irqstat, [\tmp]
		ldr	\tmp, =itc_intstatenable_2
		ldr	\tmp, [\tmp]
		ands	\irqstat, \irqstat, \tmp	@ Any IRQ set ?
		beq	1002f				@ No IRQ set, check IRQ block 2

		mov	\irqnr, #32			@ IRQ nr = 32
1200:		movs	\irqstat, \irqstat, lsr #1	@ Shift bit into carry
		bcs	1004f				@ IRQ found, get out of here.
		add	\irqnr, \irqnr, #1		@ Increase irqnr
		bne	1200b				@ Check next IRQ
		b	1005f				@ Uhhh...

1002:		/* Check IRQ controller #2 */
		ldr	\tmp, =itc_intstatset_3	
		ldr	\irqstat, [\tmp]
		ldr	\tmp, =itc_intstatenable_3
		ldr	\tmp, [\tmp]
		ands	\irqstat, \irqstat, \tmp	@ Any IRQ set ?
		beq	1003f				@ No IRQ set, check IRQ block 3
		mov	\irqnr, #64			@ IRQ nr = 64

1300:		movs	\irqstat,\irqstat,lsr #1	@ Shift bit into carry
		bcs	1004f				@ IRQ found, get out of here.
		add	\irqnr, \irqnr, #1		@ Increase irqnr
		bne	1300b				@ Check next IRQ
		b	1005f				@ Uhhh...

1003:		/* Check IRQ controller #3 */
		ldr	\tmp, =itc_intstatset_4
		ldr	\irqstat, [\tmp]
		ldr	\tmp, =itc_intstatenable_4
		ldr	\tmp, [\tmp]
		ands	\irqstat, \irqstat, \tmp	@ Any IRQ set ?
		beq	1005f				@ No IRQ set, get out of here
		mov	\irqnr, #96			@ IRQ nr = 96

1400:		movs	\irqstat, \irqstat, lsr #1	@ Shift bit into carry
		bcs	1004f				@ IRQ found, get out of here.
		add	\irqnr, \irqnr, #1		@ Increase irqnr
		bne	1400b				@ Check next IRQ
		b	1005f				@ No IRQs set, hmm
		/* IRQ found */
1004:		movs	\irqstat, #1			@ Force !Z 
1005:		/* If No IRQ are set */
		.endm

		.macro  irq_prio_table
		.endm
