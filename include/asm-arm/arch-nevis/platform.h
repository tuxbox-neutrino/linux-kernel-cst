/************************************************************************************
 * include/asm-arm/arch-nevis/platform.h
 *
 * Copyright (C) 2008 CoolStream International Limited.
 * Copyright (C) 2007 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA
 **************************************************************************************/

#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <asm/arch/hardware.h>
#include <asm/types.h>

typedef enum _PLL_SOURCE {
        ARM_PLL_SOURCE = 0,
        MEM_PLL_SOURCE,
        MPG0_PLL_SOURCE,
        FENRUS_PLL_SOURCE
} PLL_SOURCE;

extern void CalcClkFreqAndPeriod(unsigned int *frequency,
                                 unsigned int *period,
                                 unsigned int pll_source,
                                 unsigned int xtal_freq);


#endif /* __PLATFORM_H */
