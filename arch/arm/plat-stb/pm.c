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

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/cacheflush.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach/arch.h>
#include <asm/tlbflush.h>
#include <plat/pm.h>
#include <mach/irqs.h>
#include <mach/core.h>
#include <mach/globaltimer.h>
#include <mach/soc.h>


/* STB Platform uses ARM Cortex-A9 CPU and has a Cortex-M3 as the standby
 * controller processor
 */

#define ITC_CPU_PRIMASK_REG		(CORTEX_A9_INTC_BASE + GIC_CPU_PRIMASK)
#define ITC_CPU_EOI_REG			(CORTEX_A9_INTC_BASE + GIC_CPU_EOI)
#define ITC_DISTR_PENDCLEAR_REG(bank)	(CORTEX_A9_DISTR_BASE + (4*bank)+GIC_DIST_PENDING_CLEAR)
#define ITC_DISTR_ENABLECLEAR_REG(bank)	(CORTEX_A9_DISTR_BASE + (4*bank)+GIC_DIST_ENABLE_CLEAR)
#define ITC_DISTR_PRI_REG(bank)		(CORTEX_A9_DISTR_BASE + (4*bank)+GIC_DIST_PRI)

/* Addresses of IPC registers to control interrupts from other processors to
 * Cortex-A9.
 */
#define INTR_CLR_ENABLE_IPC0  (SOC_MMIO_GLB_BASE + 0xF00)
#define INTR_SET_ENABLE_IPC0  (SOC_MMIO_GLB_BASE + 0xF04)
#define INTR_ENABLE_IPC0      (SOC_MMIO_GLB_BASE + 0xF0C)

/* The following represent the bits representing each of the processors which
 * can interrupt Cortex-A9 through IPC
 */
#define A9_REASON_BIT       0x00000001
#define ADSP_REASON_BIT     0x00000002
#define ARM926_REASON_BIT   0x00000004
#define VDSP_REASON_BIT     0x00000008
#define M3_REASON_BIT       0x00000010

#define IPC0_REASON_MASK    (A9_REASON_BIT | ADSP_REASON_BIT |      \
                             ARM926_REASON_BIT | VDSP_REASON_BIT |  \
                             M3_REASON_BIT)

/* Once in standby state, interrupt from standby processor will wake us up */
#define A9_WKUP_INTID	IRQ_IPC_CORTEXM3

#define PIC_FRM_INTID(id)     ((id) / 32)
#define INTPOS_FRM_INTID(id)  ((id) % 32)

/* higher than 0xc0 which will be the new priority mask */
#define WAKEUP_INT_PRI 0xb0

#ifdef CONFIG_PM_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_NOTICE fmt, ##args)
#else
#define DPRINTK(fmt, args...)
#endif

void __iomem *gic_cpu_base_addr = CORTEX_A9_INTC_BASE;	/* used by entry-macro.S */
#ifdef CONFIG_ARCH_KORE3
static unsigned int cpipe_layer_ctl_reg[8];
static unsigned long hd_cpipe_status,sd_cpipe_status;  
#endif

extern int stb_cpu_suspend(unsigned long int *, int , unsigned long *, unsigned long *);
extern int stb_get_restore_pointer(void);
static void stb_restore_table_entry(unsigned long *);
static void stb_restore_control_register(u32);

/* Static function prototypes */
static void stb_pm_enable_wakeup_intr(void);
static void stb_pm_disable_wakeup_intr(void);
static void stb_pm_save_standby_context(void);
static void stb_pm_restore_standby_context(void);

static int stb_pm_valid(suspend_state_t state);
static int stb_pm_prepare(void);
static int stb_pm_enter(suspend_state_t state);
static void stb_pm_standby(void);
static void stb_pm_suspend(void);
static void stb_pm_finish(void);

static inline int default_op(void)
{
	return 0;
}

static inline int default_enter(unsigned long resumeAddr)
{
	return 0;
}

/* Callback routine to activate standby controller */
static struct stb_pm_stdby_ctrlr_ops ops =
{
	.prepare = default_op,
	.enter   = default_enter,
	.exit    = default_op,
	.finish  = default_op,
	.recover = default_op
};

/* Context of cortex-A9 stored before going into standby state */
typedef struct
{
	unsigned long  ipc0Mask;
	unsigned long  primask;
}stb_standby_context_t;

static stb_standby_context_t stb_standby_cxt;

static void stb_pm_disable_wakeup_intr(void)
{
	disable_irq(A9_WKUP_INTID);
	__raw_writel(M3_REASON_BIT, INTR_CLR_ENABLE_IPC0);
}

static void stb_pm_enable_wakeup_intr(void)
{
	/* Set the priority of standby controller interrupt to be higher than
	 * priority mask
	 */
	unsigned long num   = (A9_WKUP_INTID / 4);
	unsigned long pri   = __raw_readl(ITC_DISTR_PRI_REG(num));
	unsigned long tmp   = 0;
	unsigned int  shift = (A9_WKUP_INTID % 4) * 8;

	tmp = 0xFFFFFFFF ^ (0xFF << shift);
	pri &= tmp;
	pri |= (WAKEUP_INT_PRI << shift);

	__raw_writel(pri, ITC_DISTR_PRI_REG(num));

	enable_irq(A9_WKUP_INTID);

	__raw_writel(M3_REASON_BIT, INTR_CLR_ENABLE_IPC0);
	__raw_writel(M3_REASON_BIT, INTR_SET_ENABLE_IPC0);
}

static void stb_pm_clear_wakeup_intr(void)
{
	__raw_writel(M3_REASON_BIT, INTR_CLR_ENABLE_IPC0);

	__raw_writel((1 << INTPOS_FRM_INTID(A9_WKUP_INTID)),
			ITC_DISTR_ENABLECLEAR_REG(PIC_FRM_INTID(A9_WKUP_INTID)));

	__raw_writel(A9_WKUP_INTID, ITC_CPU_EOI_REG);

	__raw_writel((1 << INTPOS_FRM_INTID(A9_WKUP_INTID)),
			ITC_DISTR_PENDCLEAR_REG(PIC_FRM_INTID(A9_WKUP_INTID)));
}

/* Wakeup interrupt handler */
static irqreturn_t stb_wakeup_interrupt(int irq, void *dev)
{
	stb_pm_clear_wakeup_intr();
	return IRQ_HANDLED;
}

/* Wakeup interrupt configuration */
static struct irqaction stb_wakeup_irq = {
	.name    = "STB Platform wakeup",
	.flags   = IRQF_DISABLED | IRQF_SHARED,
	.handler = stb_wakeup_interrupt
};

/* Save the context before entering into standby state */
static void stb_pm_save_standby_context(void)
{
	stb_pm_disable_wakeup_intr();

	/* Store the existing priority mask */
	stb_standby_cxt.primask = __raw_readl(ITC_CPU_PRIMASK_REG);

	/* change it to higher than all existing interrupts */
	__raw_writel(0xC0, ITC_CPU_PRIMASK_REG);

	/* Store the enabled IPC0 interrupts configuration */
	stb_standby_cxt.ipc0Mask = __raw_readl(INTR_ENABLE_IPC0);

	__raw_writel(IPC0_REASON_MASK, INTR_CLR_ENABLE_IPC0);

	/* Enable standby controller interrupt with priority higher than priority
	 * mask.
	 */
	stb_pm_enable_wakeup_intr();
}

/* Restore the saved context */
static void stb_pm_restore_standby_context()
{
	/* Restore the priority mask */
	__raw_writel(stb_standby_cxt.primask, ITC_CPU_PRIMASK_REG);

	/* Store the enabled IPC0 interrupts configuration */
	__raw_writel(stb_standby_cxt.ipc0Mask, INTR_CLR_ENABLE_IPC0);
	__raw_writel(stb_standby_cxt.ipc0Mask, INTR_SET_ENABLE_IPC0);
}

struct stb_pm_gic_dist_save   pm_pic_save;

void stb_pm_gic_dist_save(void)
{
	unsigned int max_irq, i,j;
	void *__iomem dist_base;
	u32 cpumask = 1 << smp_processor_id();
	dist_base = (void *__iomem)(CORTEX_A9_DISTR_BASE);

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	/*
	 * Find out how many interrupts are supported.
	 */
	max_irq = readl(dist_base + GIC_DIST_CTR) & 0x1f;
	max_irq = (max_irq + 1) * 32;

	/*
	 * The GIC only supports up to 1020 interrupt sources.
	 * Limit this to either the architected maximum, or the
	 * platform maximum.
	 */
	if (max_irq > max(1020, NR_IRQS))
		max_irq = max(1020, NR_IRQS);


	/*
	 * Save Interrupt distributor enable clear.
	 * Remember this has to be toggled while restoring....
	 */
	for (i = 0,j = 0; i < max_irq; i += 32, j++)
	{
		pm_pic_save.dist_enable_clear[j] = readl(dist_base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);
	}

	/* saving distributor enable set.
	 * 
	 */
	for (i = 0,j = 0; i < max_irq; i += 32, j++)
	{
		pm_pic_save.dist_enable_set[j] = readl(dist_base + GIC_DIST_ENABLE_SET + i * 4 / 32);
	}

	return;
}

void stb_pm_gic_dist_disable(void)
{
	unsigned int max_irq, i, j;
	void *__iomem dist_base;
	dist_base = (void *__iomem)(CORTEX_A9_DISTR_BASE);

	writel(0x0, (dist_base+GIC_DIST_CTRL));
	writel(0x0, (gic_cpu_base_addr+GIC_CPU_CTRL ));

	/*
	 * Find out how many interrupts are supported.
	 */
	max_irq = readl(dist_base + GIC_DIST_CTR) & 0x1f;
	max_irq = (max_irq + 1) * 32;

	/*
	 * The GIC only supports up to 1020 interrupt sources.
	 * Limit this to either the architected maximum, or the
	 * platform maximum.
	 */
	if (max_irq > max(1020, NR_IRQS))
		max_irq = max(1020, NR_IRQS);

	/*
	 * Disable all interrupts.
	 */
	for (i = 0; i < max_irq; i += 32)
	{
		writel(0xffffffff, dist_base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);
	}

	/* saving distributor set-pending */
	for (i = 0,j = 0; i < max_irq; i += 32, j++) 
	{
		pm_pic_save.dist_pending_set[j] = readl(dist_base + GIC_DIST_PENDING_SET + i * 4 / 32);
	}

	/* clear pending interrupts */
	for (i = 0; i < max_irq; i += 32)
	{
		writel(0xffffffff, dist_base + GIC_DIST_PENDING_CLEAR + i * 4 / 32);
	}

	for (i = 0; i < max_irq; i += 32)
	{
		writel(0x0, dist_base + GIC_DIST_PENDING_SET + i * 4 / 32);
	}

	writel(0x1, (dist_base +GIC_DIST_CTRL));
	writel(0xf0, (gic_cpu_base_addr+ GIC_CPU_PRIMASK));
	writel(0x1, (gic_cpu_base_addr+GIC_CPU_CTRL ));

	return;
}

void stb_pm_gic_dist_restore(void)
{
	unsigned int max_irq, i,j;
	void *__iomem dist_base;
	u32 cpumask = 1 << smp_processor_id();
	dist_base = (void *__iomem)(CORTEX_A9_DISTR_BASE);

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	writel(0x0, (dist_base+GIC_DIST_CTRL));
	writel(0x0, (gic_cpu_base_addr+GIC_CPU_CTRL ));
	/*
	 * Find out how many interrupts are supported.
	 */
	max_irq = readl(dist_base + GIC_DIST_CTR) & 0x1f;
	max_irq = (max_irq + 1) * 32;

	/*
	 * The GIC only supports up to 1020 interrupt sources.
	 * Limit this to either the architected maximum, or the
	 * platform maximum.
	 */
	if (max_irq > max(1020, NR_IRQS))
		max_irq = max(1020, NR_IRQS);

	/*
	 * Restore Interrupt distributor enable clear.
	 * Remember this has to be toggled while restoring....
	 */
	for (i = 0,j = 0; i < max_irq; i += 32, j++)
	{
		writel(~(pm_pic_save.dist_enable_clear[j]),
			(dist_base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32));
	}

	/* Restore distributor enable set.
	 * 
	 */
	for (i = 0,j = 0; i < max_irq; i += 32, j++)
	{
		writel(pm_pic_save.dist_enable_set[j],
			(dist_base + GIC_DIST_ENABLE_SET + i * 4 / 32));
	}

	writel(0x1, (dist_base +GIC_DIST_CTRL));
	writel(0xf0, (gic_cpu_base_addr+ GIC_CPU_PRIMASK));
	writel(0x1, (gic_cpu_base_addr+GIC_CPU_CTRL ));
	return;
}

static void stb_restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/* During the MMU restoration on the restore path from MPU OFF, the page
 * table entry for the page consisting of the code being executed is
 * modified to make MMU return VA=PA.

 * The MMU is then enabled and the original entry is being stored in
 * scratchpad.  This functio reads the original values stored in
 * scratchpad, and restores them back. */

static void stb_restore_table_entry(unsigned long *restoreAddr)
{
	u32 previous_value, control_reg_value;
	u32 *address;

	/* Get address of entry that was modified */
	address = (u32 *)restoreAddr[0];
	/* Get the previous value which needs to be restored */
	previous_value = restoreAddr[1];
	address = __va(address);
	*address = previous_value;
	flush_tlb_all();
	control_reg_value = restoreAddr[2];
	/* This will enable caches and prediction */
	stb_restore_control_register(control_reg_value);
}


/* state change to s2ram */
static void stb_pm_suspend(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in stb_cpu_suspend */
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 3;
	/* Offset 1-20 -- r0-r12, lr 
	 * Offset 2- 64 -- Other CPU and co-processot registers */
	unsigned long reg_save[128] = {0, };
	/* Offset - Reg
	 * 0x0    - Page table entry location
	 * 0x4    - Previous value in page table entry location 
	 * 0x8    - Previous value of SCTLR */
	unsigned long restore_mmu_on_regs[4] = {0, };
	unsigned long ret_val = 0;

	/*
	 * Step 1: turn off interrupts (FIXME: Already disabled??)
	 */
	local_irq_disable();
	local_fiq_disable();
	DPRINTK("PM: STB Platform S2RAM \n");
	stb_pm_gic_dist_save();

	DPRINTK("PM: STB Platform Suspend - Preparing standby controller.\n");
	ret_val = ops.prepare();
	if(ret_val != 0)
	{
		DPRINTK("SCD Prepare failed with reason %lu\n", ret_val);
		goto resume;
	}
	{
		DPRINTK("reg_save = [%08x] restore_mmu_on_regs = [%08x]\n", 
				(unsigned int)reg_save, (unsigned int)restore_mmu_on_regs);
		DPRINTK("Scratchpad = [0x70 == %08x], [0x74 == %08x] ",
				readl(SCRATCHPAD_REG(29)),
				readl(SCRATCHPAD_REG(29)));
		DPRINTK("Ready to go to suspend!\n");
	}
	/*idle*/
	stb_cpu_suspend(reg_save,save_state, restore_mmu_on_regs, &ret_val);

	if(ret_val == 0x5)
	{
		/* Call the SCD Enter API here */
		__raw_writel(0xbb, SCRATCHPAD_REG(27));
		/* Calling the SCD Enter API here */
		DPRINTK("PM: STB Platform Suspend - Activating standby controller.\n");
		ret_val = ops.enter(stb_get_restore_pointer());
		__raw_writel(0xbc, SCRATCHPAD_REG(27));
		if(ret_val != 0)
		{
			DPRINTK("SCD Enter failed with reason %lu\n", ret_val);
			__raw_writel(0xbd, SCRATCHPAD_REG(27));
			ops.recover();
			goto resume;
		}
		__raw_writel(0xbe, SCRATCHPAD_REG(27));
		asm("b .");
		//while(1); /* M3 will power off the A9 waiting... */
	}

	/*reset all the ARM mode pointers*/
resume: cpu_init();

	stb_restore_table_entry(restore_mmu_on_regs);

	if(ops.exit() != 2)
	{
		ops.finish();
	}

	//stb_pm_restore_context();
	stb_pm_gic_dist_restore();
	//stb_init_irq();
	stb_pm_timer_restore();


	DPRINTK("PM: STB Platform S2RAM -- Resumed\n");
	local_irq_enable();
	local_fiq_enable();

	DPRINTK("PM: STB Platform resuming from suspend state...\n");
	return;
}

/* state change to standby */
static void stb_pm_standby(void)
{
#if  defined(CONFIG_ARCH_KRONOS) || defined(CONFIG_ARCH_KROME)
	unsigned int cpipe_layer_ctl_reg[8];
	int i;
	unsigned long flags;
#endif

	printk("PM: STB Platform entering standby state...\n");

#if  defined(CONFIG_ARCH_KRONOS) || defined(CONFIG_ARCH_KROME)
	/* Make sure the CPIPE layers are all blanked. The CPIPE RIF will
	 * continue to read memory unless the layers are off. This would
	 * cause random failures when the A9 resumes. The driver needs to
	 * manage this. This is a safe guard to avoid issues since the
	 * splash screen sets up the CPIPE outside of the CPIPE driver's
	 * scope. */
	for ( i=0; i<8; i++)
	{
		if ( i < 4 )
		{
			cpipe_layer_ctl_reg[i] = readl(ARM_A9_HOST_MMIO_BASE
					+ 0x128400 + (i * 0x400)) & 0x1;
			writel(0x0, (ARM_A9_HOST_MMIO_BASE + 0x128400 + (i * 0x400)) );
		}
		else
		{
			cpipe_layer_ctl_reg[i] = readl(ARM_A9_HOST_MMIO_BASE
					+ 0x124400 + ((i-4) * 0x400)) & 0x1;
			writel(0x0, (ARM_A9_HOST_MMIO_BASE + 0x124400 + ((i-4) * 0x400)) );
		}
	}

	/**
	 * Step 1: turn off interrupts
	 */
	local_irq_save(flags);
	local_irq_disable();
	local_fiq_disable();

	/* Make sure ALL the interrupts are off. This is just
	 * to guarantee the registers are disabled.
	 */

	/* Save any that are still on. */
	stb_pm_gic_dist_save();

	/* Disable them */
	stb_pm_gic_dist_disable();

	for ( i = 0; i<8; i++)
	{
		if ( pm_pic_save.dist_pending_set[i] )
		{
			printk("WARNING: You have a pending irq.\n\t\tindex: %d\n\t\tvalue: %x\n", 
				i, pm_pic_save.dist_pending_set[i]);
		}
	}
#endif

	/* Enable the IPC interrupt from M3 */
	__raw_writel(0x2, CORTEX_A9_DISTR_BASE + 0x104);

	/* Save the current context */
	stb_pm_save_standby_context();

	/* Give control to standby controller before doing WFI */
	printk("PM: STB Platform preparing standby controller.\n");

	/* In case we can't activate the on-chip standby controller, bailout and
	 * fake that we are resuming.
	 */
	if(ops.prepare() == 0)
	{
		printk("PM: STB Platform activating standby controller.\n");
		if(ops.enter(0) != 0)
		{
			/* Revert the effect of prepare */
			ops.recover();
			goto resume;
		}
	}
	else
	{
		goto resume;
	}

	for(;;)
	{
		flush_cache_all();

		/* Entering into WFI */
		asm("dsb\n"
		    "wfi");

		printk("PM: STB Platform out of WFI...\n");

		if(ops.exit() != 2)
		{
			ops.finish();
			break;
		}
	}

resume:
#if !defined(CONFIG_ARCH_KRONOS) && !defined(CONFIG_ARCH_KROME)
	local_irq_disable();
#endif
	/* Resumed...restore the context */
	stb_pm_restore_standby_context();

#if defined(CONFIG_ARCH_KRONOS) || defined(CONFIG_ARCH_KROME)
	/* Restore any of the PM saved interrupts. */
	stb_pm_gic_dist_restore();

	local_irq_restore(flags);
	local_fiq_enable();

	/* Restore the CPIPE layer enables */
	for ( i=0; i<8; i++)
	{
		if ( i < 4 )
		{
			writel( cpipe_layer_ctl_reg[i],
					(ARM_A9_HOST_MMIO_BASE + 0x128400 + (i * 0x400)) );
		}
		else
		{
			writel( cpipe_layer_ctl_reg[i],
					(ARM_A9_HOST_MMIO_BASE + 0x124400 + ((i-4) * 0x400)) );
		}
	}
#endif

	printk("PM: STB Platform resuming from standby state...\n");
}

/*
 * stb_pm_prepare - Do preliminary suspend work.
 *
 */
static int stb_pm_prepare(void)
{
	int ret = 0;

	printk("PM: STB Platform power mgmt prepare for suspend...\n");

	/* Setup interrupt from M3 through IPC0 */
	ret = request_irq(A9_WKUP_INTID, 
			stb_wakeup_irq.handler,
			stb_wakeup_irq.flags,
			stb_wakeup_irq.name,
			&stb_wakeup_irq);

	if(ret < 0)
	{
		printk("PM: STB Platform wakeup interrupt request failed, %d\n", ret);
		return -1;
	}

	return 0;
}

#if !defined(CONFIG_ARCH_KRONOS) && !defined(CONFIG_ARCH_KROME)
static int stb_pm_prepare_late(void)
{
	int i;


	/* Make sure the CPIPE layers are all blanked. The CPIPE RIF will
	 * continue to read memory unless the layers are off. This would
	 * cause random failures when the A9 resumes. The driver needs to
	 * manage this. This is a safe guard to avoid issues since the
	 * splash screen sets up the CPIPE outside of the CPIPE driver's
	 * scope. */

	/* First check if the CPIPE is already powered down. Access to CPIPE
	 * registers when it is powered down generates imprecise aborts
	 */

	hd_cpipe_status = readl(ARM_A9_HOST_MMIO_BASE + 0x128FF4);
	sd_cpipe_status = readl(ARM_A9_HOST_MMIO_BASE + 0x124FF4);

	for ( i=0; i<8; i++)
	{
		if ( (i < 4 ) && !(hd_cpipe_status & 0x80000000))
		{
			cpipe_layer_ctl_reg[i] = readl(ARM_A9_HOST_MMIO_BASE
					+ 0x128400 + (i * 0x400)) & 0x1;
			writel(0x0, (ARM_A9_HOST_MMIO_BASE + 0x128400 + (i * 0x400)) );
		}
		else if( (i >=4) && !(sd_cpipe_status & 0x80000000))
		{
			cpipe_layer_ctl_reg[i] = readl(ARM_A9_HOST_MMIO_BASE
					+ 0x124400 + ((i-4) * 0x400)) & 0x1;
			writel(0x0, (ARM_A9_HOST_MMIO_BASE + 0x124400 + ((i-4) * 0x400)) );
		}
	}



	/* Make sure ALL the interrupts are off. This is just
	 * to guarantee the registers are disabled.
	 */

	/* Save any that are still on. */
	stb_pm_gic_dist_save();

	/* Disable them */
	stb_pm_gic_dist_disable();

	return 0;
}

static void stb_pm_wake(void)
{
	int i;

	/* Restore any of the PM saved interrupts. */
	stb_pm_gic_dist_restore();

	/* Restore the CPIPE layer enables */
	for ( i=0; i<8; i++)
	{
		if ( (i < 4 ) && !(hd_cpipe_status & 0x80000000))
		{
			writel( cpipe_layer_ctl_reg[i],
					(ARM_A9_HOST_MMIO_BASE + 0x128400 + (i * 0x400)) );
		}
		else if( (i >= 4) && !(sd_cpipe_status & 0x80000000))
		{
			writel( cpipe_layer_ctl_reg[i],
					(ARM_A9_HOST_MMIO_BASE + 0x124400 + ((i-4) * 0x400)) );
		}
	}

	return;
}
#endif

/*
 * stb_pm_enter - Actually enter a sleep state.
 * @state:     State we're entering.
 *
 */
static int stb_pm_enter(suspend_state_t state)
{
	switch (state)
	{
		case PM_SUSPEND_STANDBY:
			stb_pm_standby();
			break;
		case PM_SUSPEND_MEM:
			stb_pm_suspend();
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


/**
 * stb_pm_finish - Finish up suspend sequence.
 *
 * This is called after we wake back up (or if entering the sleep state
 * failed).
 */
static void stb_pm_finish(void)
{
	printk("PM: STB Platform power mgmt finish.\n");

	disable_irq(A9_WKUP_INTID);

	free_irq(A9_WKUP_INTID, &stb_wakeup_irq);

	/* 
	 * re-enable the IPC0 interrupt so that the exiting ISRs 
	 * from other subsystems will be called. 
	 */
	enable_irq(A9_WKUP_INTID);
}

static int stb_pm_valid(suspend_state_t state)
{
	/* Currently support for standby state only */
	switch (state)
	{
		case PM_SUSPEND_MEM:
		case PM_SUSPEND_STANDBY:
			return 1;
	}

	return 0;
}

static struct platform_suspend_ops stb_pm_ops =
{
	.prepare    = stb_pm_prepare,
#if !defined(CONFIG_ARCH_KRONOS) && !defined(CONFIG_ARCH_KROME)
	.prepare_late = stb_pm_prepare_late,
	.wake       = stb_pm_wake,
#endif
	.enter      = stb_pm_enter,
	.finish     = stb_pm_finish,
	.valid      = stb_pm_valid
};

/* function to register routines to activate standby controller */
int stb_pm_register(struct stb_pm_stdby_ctrlr_ops *pOps)
{
	/* enter and exit are mandatory, check to see they are valid */
	if(!pOps->enter || !pOps->exit)
		return -1;

	ops.enter   = pOps->enter;
	ops.exit    = pOps->exit;

	/* prepare, recover and finish are optional, copy them conditionally */
	if(pOps->prepare)
		ops.prepare = pOps->prepare;
	if(pOps->finish)
		ops.finish  = pOps->finish;
	if(pOps->recover)
		ops.recover = pOps->recover;

	return 0;
}
EXPORT_SYMBOL(stb_pm_register);

static int __init stb_pm_init(void)
{
	printk("PM: STB Platform power mgmt initialization.\n");

	suspend_set_ops(&stb_pm_ops);

	return 0;
}
__initcall(stb_pm_init);
