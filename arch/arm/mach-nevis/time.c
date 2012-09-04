/* linux/arch/arm/mach-cx2450x_sys/time.c
 *
 * system (tick) timer functions for systems based
 * on the Conexant CX2450x (cx2450x) SoC
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/irq.h>
#include <asm/arch/cx2450x.h>

#define LOCKCMD_REG	0xE0440120
#define LOCKSTAT_REG	0xE0440124

#define SYSTEM_TIMER_NUMBER	0
#define MIPIDLE_TIMER_NUMBER	7

static u32 old_mode = 0x0B;
static u32 softlock_state = 0;

extern void idleAddCount(void);

#ifdef ENABLE_LATENCY_TIMER
extern struct s_latency latency;	/* Conexants Latency dump */
#endif

/*******************************************************************************/

void cx2450x_unlock_timer(u32 timer_number)
{
    volatile u32 *reg = (volatile u32*) LOCKCMD_REG;

    *reg = 0x00;
    *reg = 0xF8;
    *reg = 0x2B;

     reg = (volatile u32*) LOCKSTAT_REG;
    *reg &= ~(1 << (16 + timer_number));

     reg = (volatile u32*) LOCKCMD_REG;
    *reg = 0x00;
}

/*******************************************************************************/

s32 cx2450x_lock_timer(u32 timer_number)
{
    volatile u32 *reg;
    s32 ret = 0;

     reg = (volatile u32*) LOCKCMD_REG;
    *reg = 0x00;
    *reg = 0xF8;
    *reg = 0x2B;

    if (softlock_state & (1 << timer_number))	/* timer is locked in software */
	ret = -1;
    else
    {
	 reg = (volatile u32*) LOCKSTAT_REG;
	*reg |= (1 << (16 + timer_number));
    }

     reg = (volatile u32*) LOCKCMD_REG;
    *reg = 0x00;

    return ret;
}

/*******************************************************************************/

s32 cx2450x_softlock_timer(u32 timer_number)
{
    volatile u32 *reg;
    s32 ret = 0;

    reg = (volatile u32*) LOCKSTAT_REG;

    if (*reg & (1 << (16 + timer_number)))	/* timer is locked in hardware */
	ret = -1;
    else
	softlock_state |= (1 << timer_number);

    return ret;
}

/*******************************************************************************/
/* return time since last timer tick in us. Always called with interrupts      */
/* disabled.                                                                   */

static unsigned long cx2450x_sys_timer_gettimeoffset(void)
{
    /* get current counter value */
    volatile u32 *reg = (volatile u32*) TIMER_VALUE_REG_BASE(SYSTEM_TIMER_NUMBER);

    return (unsigned long) *reg;
}

/*******************************************************************************/
/* IRQ handler for the timer                                                   */

static irqreturn_t cx2450x_sys_timer_interrupt(int irq, void *dev_id)
{
    volatile u32 *mod_reg	= (volatile u32*) TIMER_MODE_REG_BASE(SYSTEM_TIMER_NUMBER);
    volatile u32 *val_reg	= (volatile u32*) TIMER_VALUE_REG_BASE(SYSTEM_TIMER_NUMBER);
    volatile u32 *limit_reg	= (volatile u32*) TIMER_LIMIT_REG_BASE(SYSTEM_TIMER_NUMBER);
    const u32 limit		= *limit_reg;


    *mod_reg &= ~0x08; /* Disable IRQs */

#ifdef ENABLE_LATENCY_TIMER
    latency.delta[latency.count++] = *val_reg;
    if (latency.count >= LATENCY_COUNT)
            latency.count = 0;
#endif

    /* clear count to zero - even if the datasheet tells us, that this is not
        needed (mode reg bit 1 = 0), the timer does not clear itself */
    do {
	    *val_reg      -= limit;
	    timer_tick();
    } while (*val_reg >= limit);

    *mod_reg |= 0x08; /* Enable IRQs */
    idleAddCount();

    return IRQ_HANDLED;
}

/*******************************************************************************/

static struct irqaction cx2450x_sys_timer_irq =
{
    .name    = "System Timer (10ms)",
    .flags   = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .handler = cx2450x_sys_timer_interrupt,
};

/*******************************************************************************/

static void __init cx2450x_sys_timer_init(void)
{
    volatile u32 *reg;
    u32 cnt;

    softlock_state = 0;

    for (cnt = 0; cnt < 16; cnt++)
    {
	cx2450x_unlock_timer(cnt);
	/* switch interrupt generation off */
	reg = (volatile u32*) TIMER_MODE_REG_BASE(cnt);
	*reg = 0;

	/* set counter value to zero */
	 reg = (volatile u32*) TIMER_VALUE_REG_BASE(SYSTEM_TIMER_NUMBER);
	*reg = 0;
    }

    /* setup the timebase (number of clock cycles after which the timer is
       increased by one). Set it to 54 cause the timer to tick every 1 usec.
       (fixed clock = 54 MHz, divided by 54 = 1 MHz, 1/1 MHz = 1 us) */
     reg = (volatile u32*) TIMER_BASE_REG_BASE(SYSTEM_TIMER_NUMBER);
    *reg = 54;

    /* Set limit to 10ms */
     reg = (volatile u32*) TIMER_LIMIT_REG_BASE(SYSTEM_TIMER_NUMBER);
    *reg = 10000;	/* 10000us = 10ms */

    setup_irq(IRQ_TIMER(SYSTEM_TIMER_NUMBER), &cx2450x_sys_timer_irq);

    /* enable timer and interrupt generation. Allow the timer to overrun the limit for get_time_offset() */
     reg = (volatile u32*) TIMER_MODE_REG_BASE(SYSTEM_TIMER_NUMBER);
    *reg = 0x0B;


    /* the MIP idle timer - runs continous without any interrupt */
     reg = (volatile u32*) TIMER_BASE_REG_BASE(MIPIDLE_TIMER_NUMBER);
    *reg = 54;
     reg = (volatile u32*) TIMER_LIMIT_REG_BASE(MIPIDLE_TIMER_NUMBER);
    *reg = 0xFFFFFFFF;
     reg = (volatile u32*) TIMER_MODE_REG_BASE(MIPIDLE_TIMER_NUMBER);
    *reg = 0x01;

    cx2450x_softlock_timer(0);
}

/*******************************************************************************/

static void cx2450x_sys_timer_suspend(void)
{
    volatile u32 *reg = (volatile u32*) TIMER_MODE_REG_BASE(SYSTEM_TIMER_NUMBER);
    old_mode = *reg;
    *reg = 0;
}

/*******************************************************************************/

static void cx2450x_sys_timer_resume(void)
{
    volatile u32 *reg = (volatile u32*) TIMER_MODE_REG_BASE(SYSTEM_TIMER_NUMBER);
    *reg = old_mode;
}

/*******************************************************************************/

struct sys_timer cx2450x_sys_timer =
{
    .init    = cx2450x_sys_timer_init,
    .offset  = cx2450x_sys_timer_gettimeoffset,
    .suspend = cx2450x_sys_timer_suspend,
    .resume  = cx2450x_sys_timer_resume,
};
			    
/*******************************************************************************/

EXPORT_SYMBOL(cx2450x_unlock_timer);
EXPORT_SYMBOL(cx2450x_lock_timer);
EXPORT_SYMBOL(cx2450x_softlock_timer);
