/*
 *
 *  arch/arm/mach-nevis/pll.c
 *
 *  Copyright (C) 2008 Coolstream International Limited
 *  Copyright (C) 2007 Conexant Systems Inc, USA.
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
 *
 */
#include <linux/clocksource.h>
#include <asm/arch/hardware.h>
#include <asm/arch/cx2450x.h>

/* FIXME: TODO:  Need to use  a global definition of CNXT_GET */

#define RMO(y)                   ( ((y) & 0x00000001) ?  0 : \
                                   ( ((y) & 0x00000002) ?  1 : \
                                     ( ((y) & 0x00000004) ?  2 : \
                                       ( ((y) & 0x00000008) ?  3 : \
                                         ( ((y) & 0x00000010) ?  4 : \
                                           ( ((y) & 0x00000020) ?  5 : \
                                             ( ((y) & 0x00000040) ?  6 : \
                                               ( ((y) & 0x00000080) ?  7 : \
                                                 ( ((y) & 0x00000100) ?  8 : \
                                                   ( ((y) & 0x00000200) ?  9 : \
                                                     ( ((y) & 0x00000400) ? 10 : \
                                                       ( ((y) & 0x00000800) ? 11 : \
                                                         ( ((y) & 0x00001000) ? 12 : \
                                                           ( ((y) & 0x00002000) ? 13 : \
                                                             ( ((y) & 0x00004000) ? 14 : \
                                                               ( ((y) & 0x00008000) ? 15 : \
                                                                 ( ((y) & 0x00010000) ? 16 : \
                                                                   ( ((y) & 0x00020000) ? 17 : \
                                                                     ( ((y) & 0x00040000) ? 18 : \
                                                                       ( ((y) & 0x00080000) ? 19 : \
                                                                         ( ((y) & 0x00100000) ? 20 : \
                                                                           ( ((y) & 0x00200000) ? 21 : \
                                                                             ( ((y) & 0x00400000) ? 22 : \
                                                                               ( ((y) & 0x00800000) ? 23 : \
                                                                                 ( ((y) & 0x01000000) ? 24 : \
                                                                                   ( ((y) & 0x02000000) ? 25 : \
                                                                                     ( ((y) & 0x04000000) ? 26 : \
                                                                                       ( ((y) & 0x08000000) ? 27 : \
                                                                                         ( ((y) & 0x10000000) ? 28 : \
                                                                                           ( ((y) & 0x20000000) ? 29 : \
                                                                                             ( ((y) & 0x40000000) ? 30 : \
                                                                                               ( ((y) & 0x80000000) ? 31 : 0 ))))))))))))))))))))))))))))))))

#define CNXT_GET(a, b)		(*((volatile unsigned int*)ASX_TO_VIRT(a)) & b)
#define CNXT_GET_VAL(reg,mask)	((*((volatile unsigned int*)ASX_TO_VIRT(reg)) & (mask)) >> RMO(mask))
typedef unsigned int u_int32;
typedef unsigned long long u_int64;

/********************************************************************/
/*  CalcFreqFromFracPart                                            */
/*                                                                  */
/*  DESCRIPTION:                                                    */
/*      Calculate ( freq * pll_frac ) / ( 2 ^ frac_len )            */
/*      Note that ( freq * pll_frac ) is much bigger                */
/*      than unsigned 0xffffffff.  The algorithm used may result    */
/*      up to 3 less than what should be                            */
/*                                                                  */
/*  PARAMETERS:                                                     */
/*      See description                                             */
/*                                                                  */
/*  RETURNS:                                                        */
/*      result described above                                      */
/********************************************************************/
static u_int32 CalcFreqFromFracPart(
                    u_int32 uFreq,
                    u_int32 uPLLDivider,
                    u_int32 uDividerWidth)
{
    u_int64 uCalc;

    uCalc = ((u_int64)uFreq * (u_int64)(uPLLDivider & ((1 << uDividerWidth)-1)))/(u_int64)(1 << uDividerWidth);

    return((u_int32)uCalc);
}

/******************************************************************************/
/*  CalkClkFreqAndPeriod                                                      */
/*                                                                            */
/*  DESCRIPTION:                                                              */
/*      Calculate clock frequency and period for a specific PLL.              */
/*                                                                            */
/*  PARAMETERS:                                                               */
/*      pll_source - One of:  ARM_PLL_SOURCE, MEM_PLL_SOURCE, MPG0_PLL_SOURCE */
/*      xtal_freq  - Xtal frequency in Hz                                     */
/*                                                                            */
/*  RETURNS:                                                                  */
/*      frequency  - Returned clock frequency in Hz                           */
/*      period     - Returned clock period in 100ns increments                */
/******************************************************************************/
void CalcClkFreqAndPeriod(u_int32 * frequency,
			  u_int32 * period,
			  PLL_SOURCE pll_source, u_int32 xtal_freq)
{
	u_int32 pll_int;
	u_int32 pll_frac;
	u_int32 pll_div;
	u_int32 pll_prescale = 0;
	u_int32 clk_freq;
	u_int32 remainder;
	u_int32 frac_shift = 0;
	u_int32 clk_period;
	u_int32 pll_sel = 0;

	*frequency = 0;
	*period = 0;

	/*
	 * Calculate the clk_freq and clk_period
	 */

	/* Calculation below depends on the PLL details of the chip.  */
	/* According to Eric Deal (1/29/04) Trinity uses a XTAL       */
	/* frequency of 74.25 MHz, but there is a divide by 2         */
	/* before the clock ever reaches the PLL logic                */

	/* We need to do a XTAL /2 for MEM and FENRUS PLLs only, the other Verve PLLs take the 60 Mhz crystal directly */
	if (pll_source == MEM_PLL_SOURCE) {
		//xtal_freq >>= 1;
	}

	/* In the following calculations, the frac value read from the */
	/* PLL multiplier must be first converted to a floating       */
	/* point value, then divided by the number of bits used to    */
	/* represent the frac value (2^16) represented as a floating  */
	/* point value. This will cause the fixed-point integer       */
	/* representation of a fractional value to be converted to a  */
	/* floating-point fractional value.                           */

	/*
	 * Read the PLL values from the chip
	 */
	switch (pll_source) {
	case ARM_PLL_SOURCE:
		/* For Pecos, there is no dedicated ARM PLL, any of the 9 PLLs can source it.
		 * We need to derive this from the PLL_SEL bits for the ARM clock 
		 * We are handling only the 7 Verve PLLs here.*/

		pll_sel = CNXT_GET_VAL(PLL_DIV_MUX_CTRL1_REG, 0x00000F00);

		switch (pll_sel) {
			/* Handle the Verve PLLs in a common fashion */
			/* All the INTFRAC and CTRL regs for the Verve PLLs start at offset 0 of 
			 * PLL_BASE with 0x4 increments. We derive the corresponding INTFRAC and
			 * CTRL reg offsets from the corresponding PLL_SEL value
			 */
		case MPG0PLL:
		case MPG1PLL:
		case HDPLL:
		case AUDPLL:
			pll_int      = CNXT_GET_VAL((PLL_BASE + (2 * pll_sel * 0x04)), 0x7E000000);
			pll_frac     = CNXT_GET_VAL((PLL_BASE + (2 * pll_sel * 0x04)), 0x01FFFFFF);
			pll_div      = CNXT_GET_VAL(PLL_DIV_MUX_CTRL1_REG, 0x000000FF);
			pll_prescale = CNXT_GET_VAL((PLL_BASE + 4 + (2 * pll_sel * 0x4)), 0x000F0000);
			break;
		case PLL0:
		case PLL1:
		case PLL2:
			pll_int      = CNXT_GET_VAL((PLL_BASE + (2 * (pll_sel - 1) * 0x4)), 0x7E000000);
			pll_frac     = CNXT_GET_VAL((PLL_BASE + (2 * (pll_sel - 1) * 0x4)), 0x01FFFFFF);
			pll_div      = CNXT_GET_VAL(PLL_DIV_MUX_CTRL1_REG, 0x000000FF);
			pll_prescale = CNXT_GET_VAL((PLL_BASE + 4 + (2 * (pll_sel - 1) * 0x4)), 0x000F0000);
			break;
		default:
			return;
		}
		break;

	case MEM_PLL_SOURCE:
		/* For Pecos, use the INT/FRAC masks from the MEM_PLL_CTRL2 register, and the
		 * MEM_PLL_CTRL1 register for the POSTDIV/PREDIV masks
		 */
		pll_int      = CNXT_GET_VAL(MEM_PLL_CTRL2_REG, 0x000001FF);
		pll_frac     = CNXT_GET_VAL(MEM_PLL_CTRL2_REG, 0xFFFFF000);
		pll_div      = CNXT_GET_VAL(MEM_PLL_CTRL1_REG, 0x00000700);
		pll_prescale = CNXT_GET_VAL(MEM_PLL_CTRL1_REG, 0x00000070);
		frac_shift   = 12;
		printk("pll_int:      %08x\n", pll_int);
		printk("pll_frac:     %08x\n", pll_frac);
		printk("pll_div:      %08x\n", pll_div);
		printk("pll_prescale: %08x\n", pll_prescale);
		break;

	case MPG0_PLL_SOURCE:
		pll_int      = CNXT_GET_VAL(PLL_MPG0_INTFRAC_REG, 0x7E000000);
		pll_frac     = CNXT_GET_VAL(PLL_MPG0_INTFRAC_REG, 0x01FFFFFF);
		pll_div      = 1;	/* We want the basic PLL rate here, not the MPG0 clock rate */
		pll_prescale = CNXT_GET_VAL(PLL_MPG0_CTRL_REG, 0x000F0000);
		frac_shift   = 0;
		break;
	case FENRUS_PLL_SOURCE:
		/* For Pecos, there is no dedicated I/O PLL, any of the 9 PLLs can source it.
		 * We need to derive this from the PLL_SEL bits for the HSX0 clock
		 */

		pll_sel = CNXT_GET_VAL(PLL_DIV_MUX_CTRL10_REG, 0x00000F00);

		switch (pll_sel) {
			/* Handle the Verve PLLs in a common fashion */
			/* All the INTFRAC and CTRL regs for the Verve PLLs start at offset 0 of 
			 * PLL_BASE with 0x4 increments. We derive the corresponding INTFRAC and
			 * CTRL reg offsets from the corresponding PLL_SEL value
			 *
			 * INT_MASK = 0x7E000000 
			 * FRAC_MASK = 0x01FFFFFF 
			 * PREDIV_MASK = 0x000F0000
			 */
		case MPG0PLL:
		case MPG1PLL:
		case HDPLL:
		case AUDPLL:
			pll_int      = CNXT_GET_VAL((PLL_BASE + (2 * pll_sel * 0x4)), 0x7E000000);
			pll_frac     = CNXT_GET_VAL((PLL_BASE + (2 * pll_sel * 0x4)), 0x01FFFFFF);
			pll_div      = CNXT_GET_VAL(PLL_DIV_MUX_CTRL10_REG, 0x000000FF);
			pll_prescale = CNXT_GET_VAL((PLL_BASE + 4 + (2 * pll_sel * 0x4)), 0x000F0000);
			break;
		case PLL0:
		case PLL1:
		case PLL2:
			pll_int      = CNXT_GET_VAL((PLL_BASE + (2 * (pll_sel - 1) * 0x4)), 0x7E000000);
			pll_frac     = CNXT_GET_VAL((PLL_BASE + (2 * (pll_sel - 1) * 0x4)), 0x01FFFFFF);
			pll_div      = CNXT_GET_VAL(PLL_DIV_MUX_CTRL10_REG, 0x000000FF);
			pll_prescale = CNXT_GET_VAL((PLL_BASE + 4 + (2 * (pll_sel - 1) * 0x4)), 0x000F0000);
			break;
		case FENRUSPLL:
			pll_int      = CNXT_GET_VAL(PLL_FENRUS_CTRL_REG, 0x3F000000);
			pll_frac     = 0;
			pll_div      = CNXT_GET_VAL(PLL_DIV_MUX_CTRL10_REG, 0x000000FF);
			xtal_freq    = CHIP_CRYSTAL_FREQUENCY;
			pll_prescale = XTAL_PRESCALE_FACTOR;
			break;
		default:
			return;
		}
		break;
	default:
		return;
	}

#ifdef DLOAD
	/*
	 * If called from the Download (DLOAD) Utility, run-time select the
	 * appropriate calculation function.
	 */

	xtal_freq = CHIP_CRYSTAL_FREQUENCY;
	pll_prescale = XTAL_PRESCALE_FACTOR;
#endif /* DLOAD */
	pll_div = pll_div * pll_prescale;

	/* 
	 * calculate clk_freq = xtal_freq*(pll_int+pll_frac/2^n)/pll_div
	 * note that algorithm used here may result up to 4Hz less than what it should be
	 */
	clk_freq = xtal_freq / pll_div;
	remainder = xtal_freq % pll_div;

	/* 
	 * The no. of fractional bits have not changed across Pecos and Trinity, except
	 * for the MEM PLL in Pecos which is 20 fractional bits, instead of the usual 25 bits
	 */
	if (pll_source == MEM_PLL_SOURCE) {
		clk_freq =
		    CalcFreqFromFracPart(clk_freq, pll_frac,
					 (32 - frac_shift))
		    + (clk_freq * pll_int) +
		    ((remainder * pll_int) / pll_div);
	} else if ((pll_source == FENRUS_PLL_SOURCE)
		   && (pll_sel == FENRUSPLL)) {
		clk_freq = (clk_freq / pll_div) * pll_int;
	} else {
		clk_freq =
		    CalcFreqFromFracPart(clk_freq, pll_frac,
					 (25 - frac_shift))
		    + (clk_freq * pll_int) +
		    ((remainder * pll_int) / pll_div);
	}

	/*
	 * Calculate clk_period = (10^11)/clk_freq
	 * note that the algorithm will be broken if clk_freq is higher
	 * than 429.5MHz.  it should not happen in the predictable future.
	 */
	clk_period = (1000000000 / clk_freq) * 100;
	remainder = ( 1000000000 % clk_freq) * 10;

	clk_period += (remainder / clk_freq) * 10;
	remainder = (remainder % clk_freq) * 10;

	clk_period += remainder / clk_freq;

	/*
	 * Return the values
	 */
	*frequency = clk_freq;
	*period = clk_period;
}

