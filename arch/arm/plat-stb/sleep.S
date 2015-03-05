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
#include <asm/assembler.h>
#include <asm/memory.h>
#include <mach/soc.asmh>

#define SCRATCHPAD_RESTORE_PC_OFFSET 		0x74
#define SCRATCHPAD_RESTORE_PC_LOC          	SCRATCHPAD_REG_BASE + SCRATCHPAD_RESTORE_PC_OFFSET
#define SCRATCHPAD_CPU_CONTEXT_LOC_OFFSET 	0x70
#define SCRATCHPAD_CPU_CONTEXT_LOC     		SCRATCHPAD_REG_BASE + SCRATCHPAD_CPU_CONTEXT_LOC_OFFSET
#define SCRATCHPAD_REBOOT_INDICATOR 		0x6C
#define SCRATCHPAD_REBOOT_INDICATOR_LOC    	SCRATCHPAD_REG_BASE + SCRATCHPAD_REBOOT_INDICATOR
#define SCRATCHPAD_SAVE_REGS_WHEN_MMU_OFF  	0x68
#define SCRATCHPAD_SAVE_REGS_WHEN_MMU_OFF_LOC  	SCRATCHPAD_REG_BASE + SCRATCHPAD_SAVE_REGS_WHEN_MMU_OFF 
#define SCRATCHPAD_DEBUG_SAVE 			0x00
#define SCRATCHPAD_DEBUG_SAVE_LOC 		SCRATCHPAD_REG_BASE + SCRATCHPAD_DEBUG_SAVE 

.text
/* Function call to get the restore pointer for resume from OFF */
ENTRY(stb_get_restore_pointer)
	stmfd   sp!, {lr}     @ save registers on stack
	adr	r0, restore
	bic     r0, #PAGE_OFFSET
	ldmfd   sp!, {pc}     @ restore regs and return
ENTRY(get_restore_pointer_sz)
	.word   . - get_restore_pointer_sz
	/*
	 * Forces STB Platform into idle state
	 *
	 * stb_cpu_suspend() - This bit of code just executes the WFI
	 * for normal idles.
	 *
	 * Note: This code get's copied to internal SRAM at boot. When the STB Platform
	 *	 wakes up it continues execution at the point it went to sleep.
	 */
ENTRY(stb_cpu_suspend)
	/* stmfd	sp!, {r0-r12, lr} */		@ save registers on stack
	/* storing the address of regs_save in scratchpad */
	stmia r0!, {r0-r3}              @ save r0, r1, r2 in offset 0 to 2
	mov r3, r0
	sub r0, r0, #0x10                @ get back r0 to regs_save[0] 
	stmia r3!, {r4-r12, lr}		@ save registers in offset 3 to 19  

	ldr r8, scratchpad_save_regs_mmu_off_loc
	bic r2, #PAGE_OFFSET
	str r2, [r8]
	ldr r7, scratchpad_cpu_context_loc
	bic r8, r0, #PAGE_OFFSET
	str r8, [r7]
#if SCRATCHPAD_SAVE_DEBUG
	/* storing the restore entry point address in scratchpad. Microloader should jump to this loc  */
	ldr r7, scratchpad_restore_pc_loc
	adr r8, restore
	bic r8, #PAGE_OFFSET
	str r8, [r7]
#endif
	loop:
	/*b	loop*/	@Enable to debug by stepping through code
	/* r0 contains restore pointer in sdram */
	/* r1 contains information about saving context */

	cmp	r1, #0x0
	/* If context save is required, do that and execute wfi */
	bne	save_context_wfi
	/* Data memory barrier and Data sync barrier */
	mov	r1, #0
	mcr	p15, 0, r1, c7, c10, 4
	mcr	p15, 0, r1, c7, c10, 5

	wfi				@ wait for interrupt

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ldmfd	sp!, {r0-r12, pc}		@ restore regs and return
	restore:
	/*b restore */	@ Enable to debug restore code
	/* Check what was the reason for mpu reset and store the reason in r9*/
	/* 1 - Only L1 and logic lost */
	/* 2 - Only L2 lost - In this case, we wont be here */
	/* 3 - Both L1 and L2 lost */
	/*FIXME: Read and tell if we restored from OFF. Ashwin. r2 will carry 3 indicating OFF*/

	setmode PSR_F_BIT | PSR_I_BIT | SVC_MODE

	mov     r2, #0x3
	and     r2, r2, #0x3
	cmp     r2, #0x0	@ Check if target power state was OFF or RET
	moveq   r9, #0x3        @ MPU OFF => L1 and L2 lost
	movne	r9, #0x1	@ Only L1 and L2 lost => avoid L2 invalidation
	bne	logic_l1_restore
	/* Execute smi to invalidate L2 cache */
	mov r12, #0x1                         @ set up to invalide L2
	/*smi:    .word 0xE1600070                @ Call SMI monitor (smieq)*/
	logic_l1_restore:
	mov	r1, #0
	/* Invalidate all instruction caches to PoU
	 * and flush branch target cache */
	mcr	p15, 0, r1, c7, c5, 0

	/*	ldr	r4, scratchpad_base*/
	/*	ldr	r3, [r4,#0xBC]*/
	/* storing the address os regs_save in scratchpad */
	/* Uncomment this below line and comment the following two lines 
	 * if u want to restore from scratchpad */
	/* ldr r12, scratchpad_debug_save_loc */
	ldr     r1, scratchpad_cpu_context_loc
	ldr     r12, [r1]
	mov	r3, r12
	add r3, #0x78      @ 30th offset onwards 

	ldmia	r3!, {r4-r6}
	mov	sp, r4
	msr	spsr_cxsf, r5
	mov	lr, r6

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x0]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x4]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x8]
	/* Saving in scratchpad during restore >> */
#endif

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xac
	str r5, [r4]
	/* Debug >> */
#endif

	ldmia	r3!, {r4-r9}
	/* Coprocessor access Control Register */
	mcr p15, 0, r4, c1, c0, 2


	/* TTBR0 */
	MCR p15, 0, r5, c2, c0, 0
	/* TTBR1 */
	MCR p15, 0, r6, c2, c0, 1
	/* Translation table base control register */
	MCR p15, 0, r7, c2, c0, 2
	/*domain access Control Register */
	MCR p15, 0, r8, c3, c0, 0
	/* data fault status Register */
	MCR p15, 0, r9, c5, c0, 0

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0xc]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x10]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x14]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x18]
	ldr r12, scratchpad_debug_save_loc
	str r8, [r12, #0x1c]
	ldr r12, scratchpad_debug_save_loc
	str r9, [r12, #0x20]
	/* Saving in scratchpad during restore >> */
#endif

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xad
	str r5, [r4]
	/* Debug >> */
#endif

	ldmia  r3!,{r4-r8}
	/* instruction fault status Register */
	MCR p15, 0, r4, c5, c0, 1
	/*Data Auxiliary Fault Status Register */
	MCR p15, 0, r5, c5, c1, 0
	/*Instruction Auxiliary Fault Status Register*/
	MCR p15, 0, r6, c5, c1, 1
	/*Data Fault Address Register */
	MCR p15, 0, r7, c6, c0, 0
	/*Instruction Fault Address Register*/
	MCR p15, 0, r8, c6, c0, 2

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x24]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x28]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x2c]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x30]
	ldr r12, scratchpad_debug_save_loc
	str r8, [r12, #0x34]
	/* Saving in scratchpad during restore >> */
#endif

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xae
	str r5, [r4]
	/* Debug >> */
#endif

	ldmia  r3!,{r4-r7}

	/* user r/w thread and process ID */
	MCR p15, 0, r4, c13, c0, 2
	/* user ro thread and process ID */
	MCR p15, 0, r5, c13, c0, 3
	/*Privileged only thread and process ID */
	MCR p15, 0, r6, c13, c0, 4
	/* cache size selection */
	MCR p15, 2, r7, c0, c0, 0

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x38]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x3c]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x40]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x44]
	/* Saving in scratchpad during restore >> */
#endif

	ldmia  r3!,{r4-r8}
	/* Data TLB lockdown registers */
	MCR p15, 0, r4, c10, c0, 0
	/* Instruction TLB lockdown registers */
	MCR p15, 0, r5, c10, c0, 1
	/* Secure or Nonsecure Vector Base Address */
	MCR p15, 0, r6, c12, c0, 0
	/* FCSE PID */
	MCR p15, 0, r7, c13, c0, 0
	/* Context PID */
	MCR p15, 0, r8, c13, c0, 1

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x48]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x4c]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x50]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x54]
	ldr r12, scratchpad_debug_save_loc
	str r8, [r12, #0x58]
	/* Saving in scratchpad during restore >> */
#endif


#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xaf
	str r5, [r4]
	/* Debug >> */
#endif

	ldmia  r3!,{r4-r5}
	/* primary memory remap register */
	MCR p15, 0, r4, c10, c2, 0
	/*normal memory remap register */
	MCR p15, 0, r5, c10, c2, 1

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x5c]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x60]
	/* Saving in scratchpad during restore >> */
#endif


#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xb0
	str r5, [r4]
	/* Debug >> */
#endif

	/* Restore cpsr */
	ldmia	r3!,{r4}	/*load CPSR from SDRAM*/
	/*msr	cpsr, r4 */	/*store cpsr */


	/* Enabling MMU here */
	mrc	p15, 0, r7, c2, c0, 2 /* Read TTBRControl */

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during restore << */
	/*Original TTBR in r7 --> sc pad -- 0x64*/
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x64]
	/* Saving in scratchpad during restore >> */
#endif

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe0
	str r5, [r4]
	/* Debug >> */
#endif


	/* Extract N (0:2) bits and decide whether to use TTBR0 or TTBR1*/
	and	r7, #0x7
	cmp	r7, #0x0
	beq	usettbr0
	ttbr_error:
	/* More work needs to be done to support N[0:2] value other than 0
	 * So looping here so that the error can be detected
	 */
#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe1
	str r5, [r4]
	/* Debug >> */
#endif
	/* b	ttbr_error */
	mov r5, #0
	mcr     p15, 0, r5, c2, c0, 2

	usettbr0:
	/* b       hack_cont */
	mrc	p15, 0, r2, c2, c0, 0
	ldr	r5, ttbrbit_mask
	and	r2, r5
	mov	r4, pc
	ldr	r5, table_index_mask
	and	r4, r5 /* r4 = 31 to 20 bits of pc */
	/* Extract the value to be written to table entry */
	ldr	r1, table_entry
	add	r1, r1, r4 /* r1 has value to be written to table entry*/
	/* Getting the address of table entry to modify */
	lsr	r4, #18
	add	r2, r4 /* r2 has the location which needs to be modified */
	ldr     r4, [r2]

	/* Storing previous entry of location being modified */
	ldr     r5, scratchpad_save_regs_mmu_off_loc
	ldr     r6, [r5]
	str     r4, [r6, #0x4]
	/* Storing previous location being modified */
	ldr     r5, scratchpad_save_regs_mmu_off_loc
	ldr     r6, [r5]
	str     r2, [r6, #0x0] 


	/* Modify the table entry */
	str     r1, [r2]



#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe2
	str r5, [r4]
	/* Debug >> */
#endif


	hack_cont: /*FIXME*/
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4	@ Flush prefetch buffer

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe4
	str r5, [r4]
	/* Debug >> */
#endif

	mcr	p15, 0, r0, c7, c5, 6	@ Invalidate branch predictor array

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe5
	str r5, [r4]
	/* Debug >> */
#endif

	mcr	p15, 0, r0, c8, c5, 0	@ Invalidate instruction TLB

#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xe6
	str r5, [r4]
	/* Debug >> */
#endif

	mcr	p15, 0, r0, c8, c6, 0	@ Invalidate data TLB
	/* Restore control register  but dont enable caches here*/
	/* Caches will be enabled after restoring MMU table entry */
	/*FIXME: removed the loading into r4 here and moved it down.*/


#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xb1
	str r5, [r4]
	/* Debug >> */
#endif

	ldmia	r3!, {r4}
	ldr	r2, cache_pred_disable_mask 

	/* Storing previous SCTLR.. will be restored back after MMU is switched on */
	ldr     r5, scratchpad_save_regs_mmu_off_loc
	ldr     r6, [r5]
	str     r4, [r6, #0x8] 

	and	r4, r2 

	mov r4, r4
	mcr	p15, 0, r4, c1, c0, 0
	nop
	nop
	nop
	mov r0, r0


#ifdef CONFIG_PM_DEBUG
	/* Debug << */
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xb2
	str r5, [r4]
	/* Debug >> */
#endif

	ldr r7, scratchpad_cpu_context_loc
	ldr r8, [r7]
	orr r8, #PAGE_OFFSET @MMU is enabled so use VA
	ldmia r8!, {r0-r1}

#ifdef CONFIG_PM_DEBUG
	ldr r4, scratchpad_reboot_indicator_loc
	mov r5, #0xb3
	str r5, [r4]
#endif

	/* Using r1 as it is not used after return. Had to do this as something like ldmia r8, {r0-r8}   
	 * has unpredictable result */
	mov r1, r8 
	ldmia r1!, {r2-r12, pc}
	/* ldmfd	sp!, {r0-r12, pc} */		@ restore regs and return
	save_context_wfi:
	/*b	save_context_wfi*/	@ enable to debug save code
	mov	r8, r0 /* Store SDRAM address in r8 */
	add r8, #0x78    @ 30th offset onwards 
	/* Check what that target sleep state is:stored in r1*/
	/* 1 - Only L1 and logic lost */
	/* 2 - Only L2 lost */
	/* 3 - Both L1 and L2 lost */
	cmp	r1, #0x2 /* Only L2 lost */
	beq	clean_l2
	cmp	r1, #0x1 /* L2 retained */
	/* r9 stores whether to clean L2 or not*/
	moveq	r9, #0x0 /* Dont Clean L2 */
	movne	r9, #0x1 /* Clean L2 */
	l1_logic_lost:
	/*FIXME: Using r12 for saving the reg_save pointer - ashwin:*/
	/* mov	r12, r8 */
	/* Store sp and spsr to SDRAM */
	mov	r4, sp
	mrs	r5, spsr
	mov	r6, lr
	stmia	r8!, {r4-r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x0]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x4]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x8]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Save all ARM registers */
	/* Coprocessor access control register */
	mrc	p15, 0, r6, c1, c0, 2
	stmia	r8!, {r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0xc]
	/* Saving in scratchpad during suspend >> */
#endif


	/* TTBR0, TTBR1 and Translation table base control */
	mrc	p15, 0, r4, c2, c0, 0
	mrc	p15, 0, r5, c2, c0, 1
	mrc	p15, 0, r6, c2, c0, 2
	stmia	r8!, {r4-r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x10]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x14]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x18]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Domain access control register, data fault status register,
	   and instruction fault status register */
	mrc	p15, 0, r4, c3, c0, 0
	mrc	p15, 0, r5, c5, c0, 0
	mrc	p15, 0, r6, c5, c0, 1
	stmia	r8!, {r4-r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x1c]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x20]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x24]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Data aux fault status register, instruction aux fault status,
	   datat fault address register and instruction fault address register*/
	mrc	p15, 0, r4, c5, c1, 0
	mrc	p15, 0, r5, c5, c1, 1
	mrc	p15, 0, r6, c6, c0, 0
	mrc	p15, 0, r7, c6, c0, 2
	stmia	r8!, {r4-r7}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x28]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x2c]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x30]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x34]
	/* Saving in scratchpad during suspend >> */
#endif

	/* user r/w thread and process ID, user r/o thread and process ID,
	   priv only thread and process ID, cache size selection */
	mrc	p15, 0, r4, c13, c0, 2
	mrc	p15, 0, r5, c13, c0, 3
	mrc	p15, 0, r6, c13, c0, 4
	mrc	p15, 2, r7, c0, c0, 0
	stmia	r8!, {r4-r7}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x38]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x3c]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x40]
	ldr r12, scratchpad_debug_save_loc
	str r7, [r12, #0x44]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Data TLB lockdown, instruction TLB lockdown registers */
	mrc	p15, 0, r5, c10, c0, 0
	mrc	p15, 0, r6, c10, c0, 1
	stmia	r8!, {r5-r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x48]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x4c]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Secure or non secure vector base address, FCSE PID, Context PID*/
	mrc	p15, 0, r4, c12, c0, 0
	mrc	p15, 0, r5, c13, c0, 0
	mrc	p15, 0, r6, c13, c0, 1
	stmia	r8!, {r4-r6}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x50]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x54]
	ldr r12, scratchpad_debug_save_loc
	str r6, [r12, #0x58]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Primary remap, normal remap registers */
	mrc	p15, 0, r4, c10, c2, 0
	mrc	p15, 0, r5, c10, c2, 1
	stmia	r8!,{r4-r5}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r4, [r12, #0x5c]
	ldr r12, scratchpad_debug_save_loc
	str r5, [r12, #0x60]
	/* Saving in scratchpad during suspend >> */
#endif

	/* Store current cpsr*/
	mrs	r2, cpsr
	stmia	r8!, {r2}

#if SCRATCHPAD_SAVE_DEBUG
	/* Saving in scratchpad during suspend << */
	ldr r12, scratchpad_debug_save_loc
	str r2, [r12, #0x64]
	/* Saving in scratchpad during suspend >> */
#endif

	mrc	p15, 0, r4, c1, c0, 0
	/* save control register */
	stmia	r8!, {r4}


clean_caches:
	/* Clean Data or unified cache to POU*/
	/* How to invalidate only L1 cache???? - #FIX_ME# */
	/* mcr	p15, 0, r11, c7, c11, 1 */
	cmp	r9, #1 /* Check whether L2 inval is required or not*/
	bne	skip_l2_inval
clean_l2:
	/* read clidr */
	mrc     p15, 1, r0, c0, c0, 1
	/* extract loc from clidr */
	ands    r3, r0, #0x7000000
	/* left align loc bit field */
	mov     r3, r3, lsr #23
	/* if loc is 0, then no need to clean */
	beq     finished
	/* start clean at cache level 0 */
	mov     r10, #0
loop1:
	/* work out 3x current cache level */
	add     r2, r10, r10, lsr #1
	/* extract cache type bits from clidr*/
	mov     r1, r0, lsr r2
	/* mask of the bits for current cache only */
	and     r1, r1, #7
	/* see what cache we have at this level */
	cmp     r1, #2
	/* skip if no cache, or just i-cache */
	blt     skip
	/* select current cache level in cssr */
	mcr     p15, 2, r10, c0, c0, 0
	/* isb to sych the new cssr&csidr */
	isb
	/* read the new csidr */
	mrc     p15, 1, r1, c0, c0, 0
	/* extract the length of the cache lines */
	and     r2, r1, #7
	/* add 4 (line length offset) */
	add     r2, r2, #4
	ldr     r4, assoc_mask
	/* find maximum number on the way size */
	ands    r4, r4, r1, lsr #3
	/* find bit position of way size increment */
	clz     r5, r4
	ldr     r7, numset_mask
	/* extract max number of the index size*/
	ands    r7, r7, r1, lsr #13
loop2:
	mov     r9, r4
	/* create working copy of max way size*/
loop3:
	/* factor way and cache number into r11 */
	orr     r11, r10, r9, lsl r5
	/* factor index number into r11 */
	orr     r11, r11, r7, lsl r2
	/*clean & invalidate by set/way */
        /* Ranjith -- Modified the below line */
	/* mcr     p15, 0, r11, c7, c10, 2 */
        mcr     p15, 0, r11, c7, c14, 2         @ clean & invalidate by set/way
	/* decrement the way*/
	subs    r9, r9, #1
	bge     loop3
	/*decrement the index */
	subs    r7, r7, #1
	bge     loop2
skip:
	add     r10, r10, #2
	/* increment cache number */
	cmp     r3, r10
	bgt     loop1
finished:
	/*swith back to cache level 0 */
	mov     r10, #0
	/* select current cache level in cssr */
	mcr     p15, 2, r10, c0, c0, 0
        /* Ranjith -- Added the below line */
        dsb
	isb
        mcr     p15, 0, r10, c7, c5, 0           @ I+BTB cache invalidate

skip_l2_inval:
	/* Data memory barrier and Data sync barrier */
	mov     r1, #0
	mcr     p15, 0, r1, c7, c10, 4
	mcr     p15, 0, r1, c7, c10, 5

/* 
	wfi                             @ wait for interrupt
*/
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
#ifdef CONFIG_PM_DEBUG
        /* Debug << */
        ldr r4, scratchpad_reboot_indicator_loc
        mov r5, #0x88
        str r5, [r4]
        /* Debug >> */
#endif
	/* restore regs and return */
        ldr r7, scratchpad_cpu_context_loc
        ldr r8, [r7]
        orr r8, #PAGE_OFFSET @MMU is enabled so use VA
        ldmia r8!, {r0-r4}
	mov r1, #0x5
	str r1, [r3]
        /* Using r1 as it is not used after return. Had to do this as something like ldmia r8, {r0-r8}   
         * has unpredictable result */
        mov r1, r8
        /*b . */
        ldmia r1!, {r5-r12, pc}

	/* ldmfd   sp!, {r0-r11} 
	mov r1, #0x5
	str r1, [r3]
	ldmfd   sp!, {r12, pc} */

assoc_mask:
	.word	0x3ff
numset_mask:
	.word	0x7fff
ttbrbit_mask:
	.word	0xFFFFC000
table_index_mask:
	.word	0xFFF00000
table_entry:
	.word	0x00000C02
cache_pred_disable_mask:
	.word	0xFFFFE7FB
scratchpad_cpu_context_loc:
        .word   SCRATCHPAD_CPU_CONTEXT_LOC 
scratchpad_restore_pc_loc:
        .word   SCRATCHPAD_RESTORE_PC_LOC        
scratchpad_reboot_indicator_loc:
        .word   SCRATCHPAD_REBOOT_INDICATOR_LOC
scratchpad_debug_save_loc:
        .word   SCRATCHPAD_DEBUG_SAVE_LOC 
scratchpad_save_regs_mmu_off_loc:
        .word   SCRATCHPAD_SAVE_REGS_WHEN_MMU_OFF_LOC  
