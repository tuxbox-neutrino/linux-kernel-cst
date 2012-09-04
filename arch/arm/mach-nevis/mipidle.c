/****************************************************************************
 *
 *  arch/arm/mach-nevis/mipidle.c
 *
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
 *  Copyright (C) 2008 Coolstream International Limited
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
 *  Foundation, Inc., Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 *
 ****************************************************************************/

#include <linux/kernel.h>
#include <asm/param.h>		/* for HZ */
#include <asm/arch/cx2450x.h>
#include <asm/proc-fns.h>
/**********
* Globals *
**********/
extern unsigned int arm11_pll;
unsigned long gIdleCount = 0;

#define TICKS_PER_MIN  ((HZ) * 60)	/* ooops, hello Conexant ! That is only true, as long as HZ is defined as 100 !!! */

static unsigned long idleSample[TICKS_PER_MIN];
static int index = 0;

void idleAddCount(void);
void cnxt_mip_idle(void);

void idleAddCount()
{
	idleSample[index++] = gIdleCount;
	if (index == TICKS_PER_MIN)
		index = 0;
	gIdleCount = 0;
}

int get_idle_list(char *buf)
{
	unsigned int i;
	unsigned int j;		/* Cycles per second */
	unsigned int k;		/* seconds per minute */
	unsigned int sum;
	unsigned int sum2 = 0;
	unsigned int avg_mips = 0;
	unsigned int min_mips = 450000000;
	unsigned int max_mips = 0;

	char *p = buf;

	if (arm11_pll != 0) {
		min_mips = arm11_pll;
	} else {
		min_mips = 595000000;
	}

	p += sprintf(p, "Idle MIPS for last minute:\n\n");

	if ((index + 1) == TICKS_PER_MIN)
		i = 0;
	else
		i = index;

	k = 60;
	while (k--) {
		sum = 0;
		j = HZ;
		while (j--) {
			sum += idleSample[i++];
			if (i == TICKS_PER_MIN)
				i = 0;
		}
		p += sprintf(p, "%2d: %10d\n", k, sum);
		sum2 += (sum / 10000);
		if (sum < min_mips)
			min_mips = sum;
		if (sum > max_mips)
			max_mips = sum;
	}
	avg_mips = (sum2 / 60);

	p += sprintf(p, "Avg Idle MIPS = %5d  x 10^4\n", avg_mips);
	p += sprintf(p, "Min Idle MIPS = %10d\n", min_mips);
	p += sprintf(p, "Max Idle MIPS = %10d\n", max_mips);
	p += sprintf(p, "\n");

	return p - buf;
}

void cnxt_mip_idle()
{
	/* The timer is started together with the system timer in time.c. It's 
	 * setup with a base of 54, so the timer counts up by one every 1 us
	 */
	u32 timein, timeout;
	volatile u32 *ptimVal = (u32 *) TIMER_VALUE_REG_BASE(7);
	unsigned int pll_mul;

	if (arm11_pll != 0) {
		pll_mul = arm11_pll / 1000000;
	} else {
		pll_mul = 595;
	}

	timein = *ptimVal;
	cpu_do_idle();
	timeout = *ptimVal;

	/* Nevis C is running at 594 MHz */
	gIdleCount += ((timeout - timein) * pll_mul) / 54;
}

/****************************************************************************
 * Modifications:
 * $Log:
 *  5    Coolstream Mods       8/7/08 16:58 CET       Coolstrem Dev. Team
 *       moved timer setup to systimer setup (prevents an if state ment.
 *       Changed timer to tic at 1 us to prevent an division.
 *  4    Linux_SDK 1.3         8/2/07 5:37:11 PM IST  Har Yash Bahadur Add
 *       calculation of Average, Minimum and Maximum Idle MIPS in the last one
 *        minute.
 *  3    Linux_SDK 1.2         6/5/07 3:15:16 PM IST  Vineet Seth     Cosmetic
 *       changes to remove warning
 *  2    Linux_SDK 1.1         3/27/07 5:16:05 PM IST Satpal Parmar   GPL
 *       header and starteam footer addition.
 *  1    Linux_SDK 1.0         3/1/07 10:53:09 PM IST Vineet Seth     
 * $
 *
 ****************************************************************************/
