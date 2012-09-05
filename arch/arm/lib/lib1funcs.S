/*
 * linux/arch/arm/lib/lib1funcs.S: Optimized ARM division routines
 *
 * Author: Nicolas Pitre <nico@fluxnic.net>
 *   - contributed to gcc-3.4 on Sep 30, 2003
 *   - adapted for the Linux kernel on Oct 2, 2003
 */

/* Copyright 1995, 1996, 1998, 1999, 2000, 2003 Free Software Foundation, Inc.

This file is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

In addition to the permissions in the GNU General Public License, the
Free Software Foundation gives you unlimited permission to link the
compiled version of this file into combinations with other programs,
and to distribute those combinations without any restriction coming
from the use of this file.  (The General Public License restrictions
do apply in other respects; for example, they cover modification of
the file, and distribution when not linked into a combine
executable.)

This file is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; see the file COPYING.  If not, write to
the Free Software Foundation, 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.  */


#include <linux/linkage.h>
#include <asm/assembler.h>


.macro ARM_DIV_BODY dividend, divisor, result, curbit

#if __LINUX_ARM_ARCH__ >= 5

	clz	\curbit, \divisor
	clz	\result, \dividend
	sub	\result, \curbit, \result
	mov	\curbit, #1
	mov	\divisor, \divisor, lsl \result
	mov	\curbit, \curbit, lsl \result
	mov	\result, #0
	
#else

	@ Initially shift the divisor left 3 bits if possible,
	@ set curbit accordingly.  This allows for curbit to be located
	@ at the left end of each 4 bit nibbles in the division loop
	@ to save one loop in most cases.
	tst	\divisor, #0xe0000000
	moveq	\divisor, \divisor, lsl #3
	moveq	\curbit, #8
	movne	\curbit, #1

	@ Unless the divisor is very big, shift it up in multiples of
	@ four bits, since this is the amount of unwinding in the main
	@ division loop.  Continue shifting until the divisor is 
	@ larger than the dividend.
1:	cmp	\divisor, #0x10000000
	cmplo	\divisor, \dividend
	movlo	\divisor, \divisor, lsl #4
	movlo	\curbit, \curbit, lsl #4
	blo	1b

	@ For very big divisors, we must shift it a bit at a time, or
	@ we will be in danger of overflowing.
1:	cmp	\divisor, #0x80000000
	cmplo	\divisor, \dividend
	movlo	\divisor, \divisor, lsl #1
	movlo	\curbit, \curbit, lsl #1
	blo	1b

	mov	\result, #0

#endif

	@ Division loop
1:	cmp	\dividend, \divisor
	subhs	\dividend, \dividend, \divisor
	orrhs	\result,   \result,   \curbit
	cmp	\dividend, \divisor,  lsr #1
	subhs	\dividend, \dividend, \divisor, lsr #1
	orrhs	\result,   \result,   \curbit,  lsr #1
	cmp	\dividend, \divisor,  lsr #2
	subhs	\dividend, \dividend, \divisor, lsr #2
	orrhs	\result,   \result,   \curbit,  lsr #2
	cmp	\dividend, \divisor,  lsr #3
	subhs	\dividend, \dividend, \divisor, lsr #3
	orrhs	\result,   \result,   \curbit,  lsr #3
	cmp	\dividend, #0			@ Early termination?
	movnes	\curbit,   \curbit,  lsr #4	@ No, any more bits to do?
	movne	\divisor,  \divisor, lsr #4
	bne	1b

.endm


.macro ARM_DIV2_ORDER divisor, order

#if __LINUX_ARM_ARCH__ >= 5

	clz	\order, \divisor
	rsb	\order, \order, #31

#else

	cmp	\divisor, #(1 << 16)
	movhs	\divisor, \divisor, lsr #16
	movhs	\order, #16
	movlo	\order, #0

	cmp	\divisor, #(1 << 8)
	movhs	\divisor, \divisor, lsr #8
	addhs	\order, \order, #8

	cmp	\divisor, #(1 << 4)
	movhs	\divisor, \divisor, lsr #4
	addhs	\order, \order, #4

	cmp	\divisor, #(1 << 2)
	addhi	\order, \order, #3
	addls	\order, \order, \divisor, lsr #1

#endif

.endm


.macro ARM_MOD_BODY dividend, divisor, order, spare

#if __LINUX_ARM_ARCH__ >= 5

	clz	\order, \divisor
	clz	\spare, \dividend
	sub	\order, \order, \spare
	mov	\divisor, \divisor, lsl \order

#else

	mov	\order, #0

	@ Unless the divisor is very big, shift it up in multiples of
	@ four bits, since this is the amount of unwinding in the main
	@ division loop.  Continue shifting until the divisor is 
	@ larger than the dividend.
1:	cmp	\divisor, #0x10000000
	cmplo	\divisor, \dividend
	movlo	\divisor, \divisor, lsl #4
	addlo	\order, \order, #4
	blo	1b

	@ For very big divisors, we must shift it a bit at a time, or
	@ we will be in danger of overflowing.
1:	cmp	\divisor, #0x80000000
	cmplo	\divisor, \dividend
	movlo	\divisor, \divisor, lsl #1
	addlo	\order, \order, #1
	blo	1b

#endif

	@ Perform all needed substractions to keep only the reminder.
	@ Do comparisons in batch of 4 first.
	subs	\order, \order, #3		@ yes, 3 is intended here
	blt	2f

1:	cmp	\dividend, \divisor
	subhs	\dividend, \dividend, \divisor
	cmp	\dividend, \divisor,  lsr #1
	subhs	\dividend, \dividend, \divisor, lsr #1
	cmp	\dividend, \divisor,  lsr #2
	subhs	\dividend, \dividend, \divisor, lsr #2
	cmp	\dividend, \divisor,  lsr #3
	subhs	\dividend, \dividend, \divisor, lsr #3
	cmp	\dividend, #1
	mov	\divisor, \divisor, lsr #4
	subges	\order, \order, #4
	bge	1b

	tst	\order, #3
	teqne	\dividend, #0
	beq	5f

	@ Either 1, 2 or 3 comparison/substractions are left.
2:	cmn	\order, #2
	blt	4f
	beq	3f
	cmp	\dividend, \divisor
	subhs	\dividend, \dividend, \divisor
	mov	\divisor,  \divisor,  lsr #1
3:	cmp	\dividend, \divisor
	subhs	\dividend, \dividend, \divisor
	mov	\divisor,  \divisor,  lsr #1
4:	cmp	\dividend, \divisor
	subhs	\dividend, \dividend, \divisor
5:
.endm


ENTRY(__udivsi3)
ENTRY(__aeabi_uidiv)

	subs	r2, r1, #1
	moveq	pc, lr
	bcc	Ldiv0
	cmp	r0, r1
	bls	11f
	tst	r1, r2
	beq	12f

	ARM_DIV_BODY r0, r1, r2, r3

	mov	r0, r2
	mov	pc, lr

11:	moveq	r0, #1
	movne	r0, #0
	mov	pc, lr

12:	ARM_DIV2_ORDER r1, r2

	mov	r0, r0, lsr r2
	mov	pc, lr

ENDPROC(__udivsi3)
ENDPROC(__aeabi_uidiv)

ENTRY(__umodsi3)

	subs	r2, r1, #1			@ compare divisor with 1
	bcc	Ldiv0
	cmpne	r0, r1				@ compare dividend with divisor
	moveq   r0, #0
	tsthi	r1, r2				@ see if divisor is power of 2
	andeq	r0, r0, r2
	movls	pc, lr

	ARM_MOD_BODY r0, r1, r2, r3

	mov	pc, lr

ENDPROC(__umodsi3)

ENTRY(__divsi3)
ENTRY(__aeabi_idiv)

	cmp	r1, #0
	eor	ip, r0, r1			@ save the sign of the result.
	beq	Ldiv0
	rsbmi	r1, r1, #0			@ loops below use unsigned.
	subs	r2, r1, #1			@ division by 1 or -1 ?
	beq	10f
	movs	r3, r0
	rsbmi	r3, r0, #0			@ positive dividend value
	cmp	r3, r1
	bls	11f
	tst	r1, r2				@ divisor is power of 2 ?
	beq	12f

	ARM_DIV_BODY r3, r1, r0, r2

	cmp	ip, #0
	rsbmi	r0, r0, #0
	mov	pc, lr

10:	teq	ip, r0				@ same sign ?
	rsbmi	r0, r0, #0
	mov	pc, lr

11:	movlo	r0, #0
	moveq	r0, ip, asr #31
	orreq	r0, r0, #1
	mov	pc, lr

12:	ARM_DIV2_ORDER r1, r2

	cmp	ip, #0
	mov	r0, r3, lsr r2
	rsbmi	r0, r0, #0
	mov	pc, lr

ENDPROC(__divsi3)
ENDPROC(__aeabi_idiv)

ENTRY(__modsi3)

	cmp	r1, #0
	beq	Ldiv0
	rsbmi	r1, r1, #0			@ loops below use unsigned.
	movs	ip, r0				@ preserve sign of dividend
	rsbmi	r0, r0, #0			@ if negative make positive
	subs	r2, r1, #1			@ compare divisor with 1
	cmpne	r0, r1				@ compare dividend with divisor
	moveq	r0, #0
	tsthi	r1, r2				@ see if divisor is power of 2
	andeq	r0, r0, r2
	bls	10f

	ARM_MOD_BODY r0, r1, r2, r3

10:	cmp	ip, #0
	rsbmi	r0, r0, #0
	mov	pc, lr

ENDPROC(__modsi3)

#ifdef CONFIG_AEABI

ENTRY(__aeabi_uidivmod)

	stmfd	sp!, {r0, r1, ip, lr}
	bl	__aeabi_uidiv
	ldmfd	sp!, {r1, r2, ip, lr}
	mul	r3, r0, r2
	sub	r1, r1, r3
	mov	pc, lr

ENDPROC(__aeabi_uidivmod)

ENTRY(__aeabi_idivmod)

	stmfd	sp!, {r0, r1, ip, lr}
	bl	__aeabi_idiv
	ldmfd	sp!, {r1, r2, ip, lr}
	mul	r3, r0, r2
	sub	r1, r1, r3
	mov	pc, lr

ENDPROC(__aeabi_idivmod)

#endif

#ifdef CONFIG_AEABI

/* Patch added by Trident for 64-bit division in kernel
	Due to a bug in GCC, it replaces the 64-bit by 64-bit/32-bit division and modulus function with __aeabi_uldivmod
    Inputs:-    r0, r1   -   divisor
                r2, r3   -   divident  ( the higher bit is ignored in current implementation as we call __do_div64
    Outputs:-   r0, r1   -   quotient
                r2, r3   -   remainder
*/

ENTRY(__aeabi_uldivmod)
    stmfd    sp!, {r4, lr}

#ifdef __ARMEB__  	          /* If we have big-endian then move r3 to r4, because __do_div64 has 32-bit divisor */
    mov      r4, r3
#else
    mov      r4, r2
#endif
    bl       __do_my_div64    /* The result is in r2 and r3 . and this is also written in assembly */

    mov      r4, r0
    mov      r0, r2
    mov      r2, r4

    mov      r4, r1
    mov      r1, r3
    mov      r3, r4

#ifndef __ARMEB__
    mov      r4, r2
    mov      r2, r3
    mov      r3, r4
#endif

    ldmfd    sp!, {r4, lr}
    mov      pc, lr
#endif

Ldiv0:

	str	lr, [sp, #-8]!
	bl	__div0
	mov	r0, #0			@ About as wrong as it could be.
	ldr	pc, [sp], #8

ENDPROC(__aeabi_uldivmod)


#ifdef __ARMEB__
#define xh r0
#define xl r1
#define yh r2
#define yl r3
#else
#define xl r0
#define xh r1
#define yl r2
#define yh r3
#endif

/*
 * This is a copy of __do_div64 from div64.S, Copy is needed otherwise bzImage does not work
 * __do_my_div64: perform a division with 64-bit dividend and 32-bit divisor.
 *
 * Note: Calling convention is totally non standard for optimal code.
 *       This is meant to be used by do_div() from include/asm/div64.h only.
 *
 * Input parameters:
 * 	xh-xl	= dividend (clobbered)
 * 	r4	= divisor (preserved)
 *
 * Output values:
 * 	yh-yl	= result
 * 	xh	= remainder
 *
 * Clobbered regs: xl, ip
 */

ENTRY(__do_my_div64)

	@ Test for easy paths first.
	subs	ip, r4, #1
	bls	9f			@ divisor is 0 or 1
	tst	ip, r4
	beq	8f			@ divisor is power of 2

	@ See if we need to handle upper 32-bit result.
	cmp	xh, r4
	mov	yh, #0
	blo	3f

	@ Align divisor with upper part of dividend.
	@ The aligned divisor is stored in yl preserving the original.
	@ The bit position is stored in ip.

#if __LINUX_ARM_ARCH__ >= 5

	clz	yl, r4
	clz	ip, xh
	sub	yl, yl, ip
	mov	ip, #1
	mov	ip, ip, lsl yl
	mov	yl, r4, lsl yl

#else

	mov	yl, r4
	mov	ip, #1
1:	cmp	yl, #0x80000000
	cmpcc	yl, xh
	movcc	yl, yl, lsl #1
	movcc	ip, ip, lsl #1
	bcc	1b

#endif

	@ The division loop for needed upper bit positions.
 	@ Break out early if dividend reaches 0.
2:	cmp	xh, yl
	orrcs	yh, yh, ip
	subcss	xh, xh, yl
	movnes	ip, ip, lsr #1
	mov	yl, yl, lsr #1
	bne	2b

	@ See if we need to handle lower 32-bit result.
3:	cmp	xh, #0
	mov	yl, #0
	cmpeq	xl, r4
	movlo	xh, xl
	movlo	pc, lr

	@ The division loop for lower bit positions.
	@ Here we shift remainer bits leftwards rather than moving the
	@ divisor for comparisons, considering the carry-out bit as well.
	mov	ip, #0x80000000
4:	movs	xl, xl, lsl #1
	adcs	xh, xh, xh
	beq	6f
	cmpcc	xh, r4
5:	orrcs	yl, yl, ip
	subcs	xh, xh, r4
	movs	ip, ip, lsr #1
	bne	4b
	mov	pc, lr

	@ The top part of remainder became zero.  If carry is set
	@ (the 33th bit) this is a false positive so resume the loop.
	@ Otherwise, if lower part is also null then we are done.
6:	bcs	5b
	cmp	xl, #0
	moveq	pc, lr

	@ We still have remainer bits in the low part.  Bring them up.

#if __LINUX_ARM_ARCH__ >= 5

	clz	xh, xl			@ we know xh is zero here so...
	add	xh, xh, #1
	mov	xl, xl, lsl xh
	mov	ip, ip, lsr xh

#else

7:	movs	xl, xl, lsl #1
	mov	ip, ip, lsr #1
	bcc	7b

#endif

	@ Current remainder is now 1.  It is worthless to compare with
	@ divisor at this point since divisor can not be smaller than 3 here.
	@ If possible, branch for another shift in the division loop.
	@ If no bit position left then we are done.
	movs	ip, ip, lsr #1
	mov	xh, #1
	bne	4b
	mov	pc, lr

8:	@ Division by a power of 2: determine what that divisor order is
	@ then simply shift values around

#if __LINUX_ARM_ARCH__ >= 5

	clz	ip, r4
	rsb	ip, ip, #31

#else

	mov	yl, r4
	cmp	r4, #(1 << 16)
	mov	ip, #0
	movhs	yl, yl, lsr #16
	movhs	ip, #16

	cmp	yl, #(1 << 8)
	movhs	yl, yl, lsr #8
	addhs	ip, ip, #8

	cmp	yl, #(1 << 4)
	movhs	yl, yl, lsr #4
	addhs	ip, ip, #4

	cmp	yl, #(1 << 2)
	addhi	ip, ip, #3
	addls	ip, ip, yl, lsr #1

#endif

	mov	yh, xh, lsr ip
	mov	yl, xl, lsr ip
	rsb	ip, ip, #32
 ARM(	orr	yl, yl, xh, lsl ip	)
 THUMB(	lsl	xh, xh, ip		)
 THUMB(	orr	yl, yl, xh		)
	mov	xh, xl, lsl ip
	mov	xh, xh, lsr ip
	mov	pc, lr

	@ eq -> division by 1: obvious enough...
9:	moveq	yl, xl
	moveq	yh, xh
	moveq	xh, #0
	moveq	pc, lr

	@ Division by 0:
	str	lr, [sp, #-8]!
	bl	__div0

	@ as wrong as it could be...
	mov	yl, #0
	mov	yh, #0
	mov	xh, #0
	ldr	pc, [sp], #8

ENDPROC(__do_my_div64)

