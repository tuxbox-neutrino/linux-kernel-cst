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

#include <linux/linkage.h>
#include <linux/init.h>

#include <mach/soc.asmh>

__INIT

/*
 * Krome specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
ENTRY(kronos_secondary_startup)
	ldr r0, =(SCRATCH_REG_CPU_PEN)
	mov r6, #2
	str r6, [r0]
	dmb
	mov   r6, #4	@ Primary core keeps waiting till 
	str r6, [r0]	@ this value is greater than 2
	dmb

	mrc p15, 0, r0, c0, c0, 5
	and r0, r0, #15
	adr r4, 1f
	ldmia   r4, {r5, r6}
	sub r4, r4, r5
	add r6, r6, r4
pen:	ldr r7, [r6]
	cmp r7, r0
	bne pen

/*
 * we've been released from the holding pen: secondary_stack
 * should now contain the SVC stack for this core
 */
	b   secondary_startup

1:	.long   .
	.long   pen_release

ENDPROC(kronos_secondary_startup)
