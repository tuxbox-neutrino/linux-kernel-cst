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


#include <linux/io.h>
#include <asm/system_misc.h>
#include <mach/soc.h>
#include <mach/generic.h>

void apollo_restart(char mode, const char *cmd)
{
	if (mode == 's') {
		/* soft reset */
		soft_restart(0);
	} else {
		/* SoC reset registers, hard reset */
		__raw_writel(0xf8, SOC_RESET_LOCKCMD);
		__raw_writel(0x2b, SOC_RESET_LOCKCMD);
		__raw_writel(0x1, SOC_RESET_LOCKSTAT);
		__raw_writel(0x1, SOC_RESET_SOFTRESET);
	}
}
