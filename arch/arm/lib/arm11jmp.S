/* linux/arch/arm/lin/arm11jmp.S
 *
 * ASM helpers for longjmp and setjmp
 * 
 * Copyright (C) 2010 CoolStream International Limited.
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


.global longjmp
.align 2

longjmp:
    mov		ip, r0
    movs	r0, r1
    moveq	r0, #1
    ldmia	ip!,  {v1-v6, sl, fp, sp, lr}
#ifdef CONFIG_FRAME_POINTER
    bx		lr
#else
    mov		pc, lr
#endif

/*******************************************************************************/

.global _setjmp
.align 2

_setjmp:
    mov		ip, r0
    stmia	ip!, {v1-v6, sl, fp, sp, lr}
    mov		r0, #0
#ifdef CONFIG_FRAME_POINTER
    bx		lr
#else
    mov		pc, lr
#endif

