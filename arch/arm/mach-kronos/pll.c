/****************************************************************************/
/*                                                                          */
/*  Copyright (C) 2012 Trident Microsystems (Far East) Ltd.                 */
/*                                                                          */
/*  This program is free software; you can redistribute it and/or modify    */
/*  it under the terms of the GNU General Public License as published by    */
/*  the Free Software Foundation, using version 2 of the License.           */
/*                                                                          */
/*  This program is distributed in the hope that it will be useful,         */
/*  but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            */
/*  GNU General Public License for more details.                            */
/*                                                                          */
/*  You should have received a copy of the GNU General Public License       */
/*  along with this program. If not, see <http://www.gnu.org/licenses/>.    */
/*                                                                          */
/****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/mach-types.h>

#include <mach/soc.h>

/* This file is derived from SCD clock driver code */

/*
 * ARM PLL
 */
#define MEGA_HERTZ                     (1000UL*1000UL)
#define CRYSTAL_FREQUENCY              (50*MEGA_HERTZ)
 
#define HM_CGU_OFFSET                  (0x6B000)
#define HM_CGU_BASE                    (ARM_A9_HOST_MMIO_BASE + HM_CGU_OFFSET)
 
#define CGU_PLL_ARM_CON1_CTL_REG       (HM_CGU_BASE + 0x0c)
 
#define CGU_PLL_ARM_CON2_CTL_REG       (HM_CGU_BASE + 0x10)
/* PLL DIRECTO */
#define    CGU_PLL_CTL_DIRECTO_MASK    (0x01 << 3)
#define    CGU_PLL_CTL_DIRECTO_SHIFT   3
/* PLL DIRECTI */
#define    CGU_PLL_CTL_DIRECTI_MASK    (0x01 << 4)
#define    CGU_PLL_CTL_DIRECTI_SHIFT   4
 
#define CGU_PLL_ARM_STA_CTL_REG        (HM_CGU_BASE + 0x14)
#define CGU_PLL_STA_LOCK_MASK          (0x01 << 0)
#define CGU_PLL_STA_LOCK               (0x01)
 
/* NDEC */
/* for Core integer PLLs, NDEC is bit 16:25 */
#define    CGU_PLL_CORE_INT_NDEC_MASK  (0x03ff << 16)
#define    CGU_PLL_CORE_INT_NDEC_SHIFT 16
 
/* PDEC */
/* for Core PLLs, PDEC is bit 9:15 */
#define    CGU_PLL_CORE_PDEC_MASK      (0x07f << 9)
#define    CGU_PLL_CORE_PDEC_SHIFT     9
 
/* MDEC */
/* for Core PLLs, MDEC is bit 0:16 */
#define    CGU_PLL_CORE_MDEC_MASK      (0x1ffff << 0)
#define    CGU_PLL_CORE_MDEC_SHIFT     0
 
#define MDIVIDER  0
#define NDIVIDER  1
#define PDIVIDER  2
 
/* Round frequency to nearest x Hz(such as 10, 100) */
#define ROUND_FREQ(freq, base )              \
       freq += (base>>1);                    \
       freq /= base;                         \
       freq *= base;                         \
 
/* derived from standby clock driver code */
static bool Mdiv2Mdec(u32 div, u32* dec)
{
    bool result=true;
    u32 i;

    if (div == 1)
    {
        *dec=0x18003; //0b11000000000000011
    }
    else if (div == 2)
    {
        *dec=0x10003; //0b10000000000000011
    }
    else if ((div > 2) && (div <= 32769))
    {
        //starting from Mdiv=3,Mdec=0x00001, inverse lfsr algo
        //15,14 = 0,1 -> msb,0
        *dec=0x00001;
        for (i=3; i<div; i++)
        {
           if( (((*dec>>14)&1)^((*dec>>0)&1)) != 0)
            *dec=*dec<<1 | 1;
        else
            *dec=*dec<<1;
        }
        *dec&=0x07FFF;
    }
    else
        result=false;

    return result;
}

static bool Ndiv2Ndec(u32 div, u32* dec)
{
    bool result=true;
    u32 i;

    if (div == 1)
    {
        *dec=0x302; //0b1100000010
    }
    else if (div == 2)
    {
        *dec=0x202; //0b1000000010
    }
    else if ((div > 2) && (div <= 257))
    {
        //starting from div=3,dec=0x001, inverse lfsr algo
        //8,6,5,4 = 0,2,3,4 -> msb,1,2,3
        *dec=0x001;
        for (i=3; i<div; i++)
        {
            if( (((*dec>>7)&1)^((*dec>>1)&1)^((*dec>>2)&1)^((*dec>>3)&1)) != 0)
                *dec=*dec<<1 | 1;
            else
                *dec=*dec<<1;
        }
        *dec&=0x0FF;
    }
    else
        result=false;

    return result;
}

static bool Pdiv2Pdec(u32 div, u32* dec)
{
    bool result=true;
    u32 i;

    if (div == 1)
    {
        *dec=0x62; //0b1100010
    }
    else if (div == 2)
    {
        *dec=0x42; //0b1000010
    }
    else if ((div > 2) && (div <= 33))
    {
        //starting from div=3,dec=0x001, inverse lfsr algo
        //5,3 = 0,2 -> msb,1
        *dec=0x001;
        for (i=3; i<div; i++)
        {
            if( (((*dec>>4)&1)^((*dec>>1)&1)) != 0)
                *dec=*dec<<1 | 1;
            else
                *dec=*dec<<1;
        }
        *dec&=0x01F;
    }
    else
        result=false;

    return result;
}

/* div -> dec*/
static bool internal_pll_get_dec( u32 uDiv, 
                                  u32 *puDev, 
                                  u32 uDivType )
{
   bool bRet = false;

   switch( uDivType )
   {
      case MDIVIDER:
         bRet = Mdiv2Mdec( uDiv, puDev );
         break;
      case NDIVIDER:
         bRet = Ndiv2Ndec( uDiv, puDev );
         break;
      case PDIVIDER:
         bRet = Pdiv2Pdec( uDiv, puDev );
         break;
      default:
         break;
   }

   return bRet;
}

/* dec -> div */
static bool internal_pll_get_div( u32 uDec, 
                                  u32 *puDiv, 
                                  u32 uDivType )
{
   bool           bRet = false;
   u32  uDecTemp, uDivLoop;
   u32  uMaxDiv;

   switch( uDivType )
   {
      case MDIVIDER:       uMaxDiv = 32769;       break;
      case NDIVIDER:       uMaxDiv = 257;         break;
      case PDIVIDER:       uMaxDiv = 33;          break;
      default:             return bRet;
   }

   for( uDivLoop = 1; uDivLoop <= uMaxDiv; uDivLoop++ )
   {
      bRet = internal_pll_get_dec(uDivLoop, &uDecTemp, uDivType );
      if( !bRet )
      {
         /* error */
         return bRet;
      }
      if( uDecTemp == uDec )
      {
         *puDiv = uDivLoop;
         bRet = true;
         break;
      } else {
         bRet = false;
      }
   }

   return bRet;
}

static bool internal_pll_get_dividers( u32 *puBypassMul,
                                       u32 *puNdiv,
                                       u32 *puPdiv,
                                       u32 *puMdiv)
{
   u32  uValue;
   u32  uMul, uNdiv, uPdiv, uNdec, uMdiv, uMdec=0, uPdec;
   u32  uMask;
   bool           bBypassN, bBypassP;
   bool bRet;

   /* 
    * The formula for fractional frequency is Fin = CRYSTAL_FREQUENCY
    * Fout = M*Fin/NP; 
    * if pre-divider is bypassed(directi is set), then N = 1;
    * if post-divider is bypassed(directo is set), then P = 1/2;
    * Mode        directi(N)     directo(P)         formular
    * 1a             1              1               M*Fin/NP = M*Fin/(1*1/2)=2*M*Fin
    * 1b             1              0               M*Fin/NP = M*Fin/(1*P)=M*Fin/P
    * 1c             0              1               M*Fin/NP = M*Fin/(N*1/2)=2*M*Fin/N
    * 1d             0              0               M*Fin/NP = M*Fin/(N*P)=M*Fin/NP
    */

   /* set multipler to be 1 */
   uMul = 1;
   uValue = readl(CGU_PLL_ARM_CON2_CTL_REG) & (CGU_PLL_CTL_DIRECTI_MASK|CGU_PLL_CTL_DIRECTO_MASK);
   bBypassN = ((uValue & CGU_PLL_CTL_DIRECTI_MASK)==0)?false:true;
   bBypassP = ((uValue & CGU_PLL_CTL_DIRECTO_MASK)==0)?false:true;

   /* N Divider */
   if( bBypassN )
   {
      uNdiv = 1;
   }
   else
   {
      /* get the N( pre-divider) */
      /* PLL in CORE: register index 1 has the Ndec */
      uMask = CGU_PLL_CORE_INT_NDEC_MASK;

      uNdec = (readl(CGU_PLL_ARM_CON2_CTL_REG) & uMask) >> CGU_PLL_CORE_INT_NDEC_SHIFT;

      bRet = internal_pll_get_div( uNdec, &uNdiv, NDIVIDER );
      if (!bRet)
          return bRet;
   }

   /* P Divider */
   if( bBypassP )
   {
      uMul = 2;
      uPdiv = 1;
   }
   else
   {
      /* get the P(post-divider) */
      uMask = CGU_PLL_CORE_PDEC_MASK;

      uPdec = (readl(CGU_PLL_ARM_CON2_CTL_REG) & uMask) >> CGU_PLL_CORE_PDEC_SHIFT;
      bRet = internal_pll_get_div( uPdec, &uPdiv, PDIVIDER);
      if (!bRet)
          return bRet;

   }

   /* M Divider */
   /* PLL in CORE: register index 0 has the Mdec */
   uMask = CGU_PLL_CORE_MDEC_MASK;

   uMdec = readl(CGU_PLL_ARM_CON1_CTL_REG) & uMask;

   /* get mdiv from mdev if frac divider is not used */
   bRet = internal_pll_get_div( uMdec, &uMdiv, MDIVIDER );
   if (!bRet)
       return bRet;


   /* return */
   *puBypassMul = uMul;
   *puNdiv      = uNdiv;
   *puPdiv      = uPdiv;
   *puMdiv      = uMdiv;

   return true;
}

unsigned long get_arm_freq(void)
{
   u32  uMul, uNdiv, uPdiv, uMdiv;
   u64  ullFreq;
   bool bRet;

   bRet = internal_pll_get_dividers(&uMul, &uNdiv, &uPdiv, &uMdiv);
   if (!bRet)
       return 500000000;

   ullFreq =  uMdiv;

   /* Step 1: Multiply by the bypass multipler */
   ullFreq *=  uMul;

   /* Step 2: Multiply by the input crystal frequency */
   ullFreq *=  CRYSTAL_FREQUENCY;

   /* Step 3: Divide by the pre-divider */
   ullFreq /=  uNdiv;

   /* Step 4: Divide by the post-divider */
   ullFreq /=  uPdiv;

   /* Round ullFreq to nearest 100 Hz */
   ROUND_FREQ(ullFreq, 100);

   /* there is always external /2 for arm clock */
   ullFreq = ullFreq >> 1;

   printk(KERN_INFO "Cortex A9 frequency at %llu MHz\n", (ullFreq/1000)/1000);

   return (u32)ullFreq;
}

EXPORT_SYMBOL_GPL(get_arm_freq);
