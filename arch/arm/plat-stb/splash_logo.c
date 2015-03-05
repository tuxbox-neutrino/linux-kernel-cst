/****************************************************************************/
/*                                                                          */
/*  Copyright (C) 2012 Trident Microsystems (Far East) Ltd.                 */
/*  Copyright (C) 2014, Entropic Communications. All Rights Reserved        */
/*  Copyright (C) 2014, Coolstream International Ltd. All Rights Reserved   */
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

#include <linux/init.h>
#include <asm/io.h>
#include <plat/splash_logo.h>
#include <plat/splash_img.h>
#include <mach/soc.h>

#if defined(CONFIG_ARCH_KRONOS)
#define HOST_CGU_BASE		(ARM_A9_HOST_MMIO_BASE + 0x06A000)
#define SOC_GBL_REG_BASE	(ARM_A9_HOST_MMIO_BASE + 0x0BB000)
#elif defined (CONFIG_ARCH_APOLLO)
#define HOST_CGU_BASE		(ARM_A9_HOST_MMIO_BASE + 0x0E7000)
#define SOC_GBL_REG_BASE	(ARM_A9_HOST_MMIO_BASE + 0x0EA000)
#else
#error ARCH not supported.
#endif

#define SOC_CPIPE_BASE		(ARM_A9_HOST_MMIO_BASE + 0x120000)
#define SOC_HDMI_TX_BASE	(ARM_A9_HOST_MMIO_BASE + 0x179000)
#define SOC_DENC_BASE		(ARM_A9_HOST_MMIO_BASE + 0x17B000)

//#define GLOBALREG_MODULE_ID_REG1 0xe06bbffc
#define CHIP_REV_MASK  0xf00
#define CHIP_REV_SHIFT 8
#define SOC_VARI_FORMAT_ARGB 0xfff7efe7
#define SOC_VARI_FORMAT_ABGR 0xffe7eff7

#define XRES 720
#define YRES 480

extern unsigned long uMALONE_start;

void __init stb_splash( void )
{
	int i, line_pixel, line;
	u32 __iomem *buffer =  __va(uMALONE_start);
	u32 __iomem *pInc;
	unsigned char pixel_data[3];
	unsigned int glb_modid,ChipRevID;

	/* fix kronos rev B */
	glb_modid = readl(GLOBALREG_MODULE_ID_REG);
	ChipRevID = (glb_modid & CHIP_REV_MASK) >> CHIP_REV_SHIFT;

	/* HD DENC programming (HD-480p raster) */
	writel( 0x00000001, (SOC_DENC_BASE + 0x120)); /* DENC csc sel  - YCbCr out */
	writel( 0x80000104, (SOC_DENC_BASE + 0x104)); /* DENC raster   -  eactive & 480p */
	writel( 0x0000035a, (SOC_DENC_BASE + 0x108)); /* DENC Htotal */
	writel( 0x02d0007a, (SOC_DENC_BASE + 0x10c)); /* DENC HActive */
	writel( 0x01e00024, (SOC_DENC_BASE + 0x110)); /* DENC Vertical timing */
	writel( 0x0000003f, (SOC_DENC_BASE + 0x00c)); /* DENC enable dacs - enable all the 6 DACs */

#if defined (CONFIG_ARCH_KRONOS) /*FIXME: this could come from kernel config/dts */
	writel( 0x00020001, (SOC_DENC_BASE + 0x010)); /* DENC  {CBA} dac selection - {rpr, bpb, gy} */
#elif defined (CONFIG_ARCH_APOLLO)
	writel( 0x00000201, (SOC_DENC_BASE + 0x010)); /* DENC  {CBA} dac selection - {rpr, bpb, gy} */
#else
#error ARCH not supported.
#endif
	writel( 0x00050603, (SOC_DENC_BASE + 0x014)); /* DENC  {FED} dac selection - all cvbs for initial validation */
	writel( 0x00000000, (SOC_DENC_BASE + 0x180)); /* DENC Sync Control- don't invert Odd/even signal */

	/* DENC programming (SD - 480i) */
#if defined (CONFIG_ARCH_KRONOS)/*FIXME?*/
	writel( 0x80000110, (SOC_DENC_BASE + 0x304)); /* DENC raster	- enable bits eactive & 480i(NTSC) */
#elif defined (CONFIG_ARCH_APOLLO)
	writel( 0x00000110, (SOC_DENC_BASE + 0x304)); /* DENC raster	- enable bits eactive & 480i(NTSC) */
#else
#error ARCH not supported.
#endif
	writel( 0x000006b4, (SOC_DENC_BASE + 0x308)); /* DENC Htotal */
	writel( 0x05a00104, (SOC_DENC_BASE + 0x30c)); /* DENC HActive (orig - 0x0590010c) */
	writel( 0x00f00013, (SOC_DENC_BASE + 0x310)); /* DENC Vertical timing */
	writel( 0x007e9054, (SOC_DENC_BASE + 0x318)); /* DENC analog timing */
	writel( 0x0085e574, (SOC_DENC_BASE + 0x324)); /* DENC amplitude */
	writel( 0x009b86bd, (SOC_DENC_BASE + 0x328)); /* DENC YUV MULT */
	writel( 0x80a4f000, (SOC_DENC_BASE + 0x32c)); /* DENC YALT Luma control reg */
	writel( 0x21f07c1f, (SOC_DENC_BASE + 0x330)); /* DENC secam reg */
	writel( 0x029d3000, (SOC_DENC_BASE + 0x380)); /* DENC Sync Control- don't invert Odd/even signal */

	/* Disable SD DENC to supply syncs to HD CPIPE: */
	writel( 0x00000000, (SOC_DENC_BASE + 0x008));
	writel( 0x00000000, (SOC_DENC_BASE + 0x394)); /* Macrovision OFF */
	writel( 0x00000000, (SOC_DENC_BASE + 0x194)); /* Macrovision OFF */

#if defined (CONFIG_ARCH_KRONOS)
	/* DAC0 Programming (Component DACs) */
	if(ChipRevID == 0x1) { /* for Kronos rev B only */
		writel( 0x00108000, (SOC_GBL_REG_BASE + 0x8fc)); /*VDAC0_CTRL0 */
		writel( 0x52001712, (SOC_GBL_REG_BASE + 0x900)); /*VDAC0_CTRL1 */
	} else {
		writel( 0x00111f00, (SOC_GBL_REG_BASE + 0x8fc)); /*VDAC0_CTRL0 */
		writel( 0x42001712, (SOC_GBL_REG_BASE + 0x900)); /*VDAC0_CTRL1 */
	}
	writel( 0x00000ff0, (SOC_GBL_REG_BASE + 0x904)); /*VDAC0_CTRL2 */
#elif defined (CONFIG_ARCH_APOLLO)
	writel( 0x0011c00e, (SOC_GBL_REG_BASE + 0x8fc)); /*VDAC0_CTRL0 */
	writel( 0x52201712, (SOC_GBL_REG_BASE + 0x900)); /*VDAC0_CTRL1 */
	writel( 0x00000ff0, (SOC_GBL_REG_BASE + 0x904)); /*VDAC0_CTRL2 */
	writel( 0x00000124, (SOC_GBL_REG_BASE + 0x908)); /*VDAC0_CTRL3 source from MPEG0 PLL */
#else
#error ARCH not supported.
#endif
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x90c)); /*VDAC0_CTRL4 71-->70-->71 */
	writel( 0x3f000070, (SOC_GBL_REG_BASE + 0x90c)); /*VDAC0_CTRL4 71-->70-->71 */
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x90c)); /*VDAC0_CTRL4 71-->70-->71 */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x910)); /*VDAC0_TEST_CTRL */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x914)); /*VDAC0_DTO_INCR0 */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x918)); /*VDAC0_DTO_INCR1 */

	/* DAC1 Programming (SD - 480i) */
	writel( 0x0011c00e, (SOC_GBL_REG_BASE + 0x920)); /*VDAC1_CTRL0 */
	writel( 0x52201712, (SOC_GBL_REG_BASE + 0x924)); /*VDAC1_CTRL1 */
	writel( 0x00000ff0, (SOC_GBL_REG_BASE + 0x928)); /*VDAC1_CTRL2 */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x92c)); /*VDAC1_CTRL3 source from MPEG0 PLL */
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x930)); /*VDAC1_CTRL4 71-->70-->71 */
	writel( 0x3f000070, (SOC_GBL_REG_BASE + 0x930)); /*VDAC1_CTRL4 71-->70-->71 */
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x930)); /*VDAC1_CTRL4 71-->70-->71 */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x934)); /*VDAC1_TEST_CTRL */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x938)); /*VDAC1_DTO_INCR0 */
	writel( 0x00000000, (SOC_GBL_REG_BASE + 0x93c)); /*VDAC1_DTO_INCR1 */

	/*  Set up the DAC and CPIPE clocks */
#if defined (CONFIG_ARCH_KRONOS)
	writel( 0x0024f881, (HOST_CGU_BASE + 0x02c)); /* vdac0_sel -- 74.25MHz mpg0 pll */
	writel( 0x00000001, (HOST_CGU_BASE + 0x038)); /* vdac1_sel -- 74.25MHz */
	writel( 0x000058e2, (HOST_CGU_BASE + 0x030)); /* vdac1_sel -- 74.25MHz */
	writel( 0x76cccccd, (HOST_CGU_BASE + 0x03c)); /* vdac2_sel -- 74.25MHz */
	/* vdacs 3,4 and 5 are always Mpeg0 PLL, 74.25 MHz */	
	writel( 0x00000000, (HOST_CGU_BASE + 0x038)); /* vdac4_sel -- 74.25MHz */
	writel( 0x00000000, (HOST_CGU_BASE + 0x040)); /* vdac5_sel -- 74.25MHz */
	writel( 0x0024f88a, (HOST_CGU_BASE + 0x02c)); /* VCGEN_G1SEL -- 27MHz, MPG0 PLL */
	writel( 0x0024f881, (HOST_CGU_BASE + 0x044)); /* VCGEN_G2SEL -- 54MHz, MPG0 PLL */
	writel( 0x00000001, (HOST_CGU_BASE + 0x050)); /* VCGEN_G3SEL -- 27MHz, MPG0 PLL */
	writel( 0x000058e2, (HOST_CGU_BASE + 0x048)); /* VCGEN_G4SEL -- 13.5MHz, MPG0 PLL */
	writel( 0x76ae6ae7, (HOST_CGU_BASE + 0x054)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x058)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x050)); /* VCGEN_G4SEL -- 13.5MHz, MPG0 PLL */
	writel( 0x0024f888, (HOST_CGU_BASE + 0x044)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x0024f88a, (HOST_CGU_BASE + 0x044)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000005, (HOST_CGU_BASE + 0x190)); /* VCGEN_G2SEL -- 54MHz, MPG0 PLL */
	writel( 0x00000002, (HOST_CGU_BASE + 0x194)); /* VCGEN_G3SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000006, (HOST_CGU_BASE + 0x198)); /* VCGEN_G4SEL -- 13.5MHz, MPG0 PLL */
	writel( 0x00000005, (HOST_CGU_BASE + 0x19c)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000006, (HOST_CGU_BASE + 0x1a0)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x1a4)); /* VCGEN_G4SEL -- 13.5MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x1a8)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x1ac)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x1b0)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x1b4)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000003, (HOST_CGU_BASE + 0x160)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
#elif defined(CONFIG_ARCH_APOLLO)
	writel( 0x00000002, (HOST_CGU_BASE + 0x658)); /* vdac0_sel -- 74.25MHz mpg0 pll */
	writel( 0x00000002, (HOST_CGU_BASE + 0x65c)); /* vdac1_sel -- 74.25MHz */
	writel( 0x00000002, (HOST_CGU_BASE + 0x660)); /* vdac2_sel -- 74.25MHz */
	/* vdacs 3,4 and 5 are always Mpeg0 PLL, 74.25 MHz */
	writel( 0x00000002, (HOST_CGU_BASE + 0x664)); /* vdac3_sel -- 74.25MHz mpg0 pll */
	writel( 0x00000002, (HOST_CGU_BASE + 0x668)); /* vdac4_sel -- 74.25MHz */
	writel( 0x00000002, (HOST_CGU_BASE + 0x66c)); /* vdac5_sel -- 74.25MHz */
	writel( 0x00000005, (HOST_CGU_BASE + 0x640)); /* VCGEN_G1SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000004, (HOST_CGU_BASE + 0x644)); /* VCGEN_G2SEL -- 54MHz, MPG0 PLL */
	writel( 0x00000006, (HOST_CGU_BASE + 0x648)); /* VCGEN_G3SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000005, (HOST_CGU_BASE + 0x64c)); /* VCGEN_G4SEL -- 13.5MHz, MPG0 PLL */
	writel( 0x00000006, (HOST_CGU_BASE + 0x650)); /* VCGEN_G5SEL -- 27MHz, MPG0 PLL */
	writel( 0x00000000, (HOST_CGU_BASE + 0x654)); /* VCGEN_G6SEL -- 27MHz, MPG0 PLL */
#else
#error ARCH not supported.
#endif

	/* CPIPE Sync Timing Generator settings */
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x8000)); /* progressive */
	writel( 0x00000f33, (SOC_CPIPE_BASE + 0x8040)); /* enable STG + f,h,v inputs, invert f,h,v */
	writel( 0x0359020c, (SOC_CPIPE_BASE + 0x8044)); /* 858 wide, 525 tall */
	writel( 0x03580087, (SOC_CPIPE_BASE + 0x8048)); /* blank from 0x34a to 0x79 */
	writel( 0x02090028, (SOC_CPIPE_BASE + 0x804c)); /* blank from line 0x20a to 0x29 */
	writel( 0x000c0021, (SOC_CPIPE_BASE + 0x8050)); /* hsync */
	writel( 0x00050007, (SOC_CPIPE_BASE + 0x8054)); /* vsync (not used) */
	writel( 0x00000012, (SOC_CPIPE_BASE + 0x8058)); /* vsync starts at (SOC_CPIPE_BASE + 0x horiz */
	writel( 0x00000012, (SOC_CPIPE_BASE + 0x805c)); /* vsync ends at (SOC_CPIPE_BASE + 0x */
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x8060));
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x8064));
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x8068)); /* ext h sets hctr to 0x25, ext v sets vctr to 2 */
	writel( 0x000e004b, (SOC_CPIPE_BASE + 0x8094)); /* hsync for hdmi starts at 1 ends at 0x3e */
	writel( 0x0005000a, (SOC_CPIPE_BASE + 0x8098)); /* 6 lines wide hdmi vsync */
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x809c));
	writel( 0x0000000e, (SOC_CPIPE_BASE + 0x80a0)); /* hdmi vsync starts at 14 horiz count */
	writel( 0x0000000e, (SOC_CPIPE_BASE + 0x80a4)); /* vsync ends, ditto */
	writel( 0x030f0002, (SOC_CPIPE_BASE + 0x94e0)); /* !hsync, !vsync, blank, odd/even to HD DENC */
	writel( 0x070f0002, (SOC_CPIPE_BASE + 0x98e0)); /* hsync, vsync, !blank, odd/even to HDMI */

	/* SD CPIPE Sync Timing Generator settings */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x4000)); /* interlaced */
	writel( 0x00000033, (SOC_CPIPE_BASE + 0x4040)); /* enable STG + f input only */
	writel( 0x03590106, (SOC_CPIPE_BASE + 0x4044)); /* 858 wide, 263 tall */
	writel( 0x03590088, (SOC_CPIPE_BASE + 0x4048)); /* h blank */
	writel( 0x01040013, (SOC_CPIPE_BASE + 0x404c)); /* v blank odd */
	writel( 0x00070010, (SOC_CPIPE_BASE + 0x4050)); /* hsync from 7 to 16 */
	writel( 0x00050007, (SOC_CPIPE_BASE + 0x4054)); /* vsync from 5 to 7 odd (not used) */
	writel( 0x01ae0001, (SOC_CPIPE_BASE + 0x4058)); /* vsync starts at 1 horiz */
	writel( 0x01ae0001, (SOC_CPIPE_BASE + 0x405c)); /* vsync ends at 1 */
	writel( 0x01040013, (SOC_CPIPE_BASE + 0x4060)); /* v blank even */
	writel( 0x00050007, (SOC_CPIPE_BASE + 0x4064)); /* vsync from 5 to 7 even (not used) */
	writel( 0x00000000, (SOC_CPIPE_BASE + 0x4068)); /* ext f sets h and v to 0 */
	writel( 0x02060002, (SOC_CPIPE_BASE + 0x54e0)); /* !hsync, blank to SD DENC */

	/* Bypass SD Out CSC */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x5690)); /* Bypass DS Out CSC */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x5400)); /* cause above to take effect on next shadow reload cycle */

	/* Bypass DENC CSC */
	writel( 0x00000000, (SOC_DENC_BASE + 0x120)); /* Bypass DENC CSC */

	/* Set up the formatting PLL - programmed thru the HDMI I/F - and the serializer PLL */
#if defined(CONFIG_ARCH_KRONOS)
	writel( 0x00000003, (HOST_CGU_BASE + 0x164)); /* Enable TMDS clock for HDMI */
	writel( 0x00000020, (HOST_CGU_BASE + 0x098)); /* PLL_HDMI_CON3_CTL - use MPG0 PLL for Fmt PLL Source */
	writel( 0x80000002, (HOST_CGU_BASE + 0x08c)); /* PLL_HDMI_CON0_CTL - use HDMI register I/F rather than direct I/F */
#elif defined(CONFIG_ARCH_APOLLO)
	writel( 0x00000003, (HOST_CGU_BASE + 0x2fc));     /* Enable TMDS clock for HDMI */
	writel( 0x00000020, (HOST_CGU_BASE + 0x11c));     /* PLL_HDMI_CON3_CTL - use MPG0 PLL for Fmt PLL Source */
	writel( 0x80000002, (HOST_CGU_BASE + 0x110));     /* PLL_HDMI_CON0_CTL - use HDMI register I/F rather than direct I/F */
#else
#error ARCH not supported.
#endif
	writel( 0x0102040a, (SOC_HDMI_TX_BASE + 0x034)); /* FMT_PLL_SETTINGS - 480i, 480p */
	writel( 0x00000203, (SOC_HDMI_TX_BASE + 0x030)); /* SER_PLL_SETTINGS - 480i, 480p (27 MHz) */
	writel( 0x00000100, (SOC_HDMI_TX_BASE + 0x038)); /* PHY_CTRL - apply the pll settings */
	writel( 0x00000130, (SOC_HDMI_TX_BASE + 0x038)); /* PHY_CTRL */
	writel( 0x00000100, (SOC_HDMI_TX_BASE + 0x038)); /* PHY_CTRL */

	/* HDMI programming */
	writel( 0x0000003f, (SOC_HDMI_TX_BASE + 0x038));
	writel( 0x00012761, (SOC_HDMI_TX_BASE + 0x044));
	writel( 0xffffffff, (SOC_HDMI_TX_BASE + 0x4d8));
	writel( 0xffffffff, (SOC_HDMI_TX_BASE + 0x4e8));
	writel( 0x00100000, (SOC_HDMI_TX_BASE + 0x4dc));
	writel( 0x00100000, (SOC_HDMI_TX_BASE + 0x4e8));
	writel( 0x00080000, (SOC_HDMI_TX_BASE + 0x4dc));
	writel( 0x00080000, (SOC_HDMI_TX_BASE + 0x4e8));
	writel( 0x00040100, (SOC_HDMI_TX_BASE + 0x038));
	writel( 0x00180000, (SOC_HDMI_TX_BASE + 0x4d8));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x4dc));
	writel( 0x42000510, (SOC_HDMI_TX_BASE + 0x000));
	writel( 0x42000410, (SOC_HDMI_TX_BASE + 0x000));
	writel( 0x31000009, (SOC_HDMI_TX_BASE + 0x400));
	writel( 0x00000016, (SOC_HDMI_TX_BASE + 0x404));
	writel( 0x000a0184, (SOC_HDMI_TX_BASE + 0x180));
	writel( 0x00000170, (SOC_HDMI_TX_BASE + 0x184));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x188));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x18c));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x300));
	writel( 0x00000004, (SOC_HDMI_TX_BASE + 0x304));
	writel( 0x10006978, (SOC_HDMI_TX_BASE + 0x308));
	writel( 0x00001800, (SOC_HDMI_TX_BASE + 0x30c));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x314));
	writel( 0x00000001, (SOC_HDMI_TX_BASE + 0x0a8)); /* format 1 for 480p59 */
	writel( 0x00080013, (SOC_HDMI_TX_BASE + 0x0ac)); /* ext h sets hctr to 0x13, ext v sets vctr to 8 */
	writel( 0x00003000, (SOC_HDMI_TX_BASE + 0x0d4));
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x0d8));
	writel( 0x00010800, (SOC_HDMI_TX_BASE + 0x0e0));
	writel( 0x04020200, (SOC_HDMI_TX_BASE + 0x0e4));
	writel( 0x00000004, (SOC_HDMI_TX_BASE + 0x0e8));
	writel( 0x91c1c240, (SOC_HDMI_TX_BASE + 0x0ec));

#if defined(CONFIG_ARCH_APOLLO)
	/*----------------------- Calibrate VDAC's ----------------------*/
	writel( 0x0009c00e, (SOC_GBL_REG_BASE + 0x8fc)); /*reset VDAC0 */
	writel( 0x0009c00e, (SOC_GBL_REG_BASE + 0x920)); /*reset VDAC1 */
	writel( 0x0019f60e, (SOC_GBL_REG_BASE + 0x8fc)); /*VDAC0_CTRL0 30 MHz power */
	writel( 0x0019f60e, (SOC_GBL_REG_BASE + 0x920)); /*VDAC1_CTRL0 30 MHz power */
	writel( 0x52201750, (SOC_GBL_REG_BASE + 0x900)); /*VDAC0_CTRL1 */
	writel( 0x52201750, (SOC_GBL_REG_BASE + 0x924)); /*VDAC1_CTRL1 */
	writel( 0x52201752, (SOC_GBL_REG_BASE + 0x900)); /*VDAC0_CTRL1 */
	writel( 0x52201752, (SOC_GBL_REG_BASE + 0x924)); /*VDAC1_CTRL1 */
	writel( 0x3f000070, (SOC_GBL_REG_BASE + 0x90c)); /*VDAC0_CTRL4 70-->71 */
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x90c));
	writel( 0x3f000070, (SOC_GBL_REG_BASE + 0x930)); /*VDAC1_CTRL4 70-->71 */
	writel( 0x3f000071, (SOC_GBL_REG_BASE + 0x930));
#endif
	/*** Put the splash screen in memory ***/
	pInc = buffer;
	/* Clear the buffer to a white background. */
	for ( i=0; i<(XRES * YRES); i++) {
		writel( 0x0, pInc);
		pInc++;
	}

	/* Calculate Address to begin write. */
	pInc = (u32 *)((u32)buffer + ((XRES * ( (YRES - logo_height) / 2 ) * 4 )));
	for ( line = 0; line< logo_height; line++) {
		pInc += (XRES - logo_width) / 2;
		for ( line_pixel = 0; line_pixel< logo_width; line_pixel++) {
			HEADER_PIXEL(header_data, pixel_data)
			*pInc = 0xff000000 + (pixel_data[2] << 16) + (pixel_data[1] << 8) + pixel_data[0];
			pInc++;
		}
		pInc += (XRES - logo_width) / 2;
	}

	/* set up RIF on GFX layer 1 */
	writel( 0xa000001f, (SOC_CPIPE_BASE + 0x8c44)); /* 32 bit variable width */
	writel( SOC_VARI_FORMAT_ARGB, (SOC_CPIPE_BASE + 0x8c48)); /* 32 bit variable width */
	writel( 0x80500027, (SOC_CPIPE_BASE + 0x8c54)); /* start DMA at line 8 pixel 0x90 */
	writel( 0x00000b3f, (SOC_CPIPE_BASE + 0x8c5c)); /* width is 719 pels */
	writel( uMALONE_start, (SOC_CPIPE_BASE + 0x8c64)); /* data is located at 0xb10000 */
	writel( 0x00000b40, (SOC_CPIPE_BASE + 0x8c6c)); /* stride is 720*4 */
	writel( 0x00000b40, (SOC_CPIPE_BASE + 0x8c70)); /* stride is 720*4 */
	writel( 0x00000b40, (SOC_CPIPE_BASE + 0x8c84)); /* stride is 720*4 */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x8d90)); /* bypass alpha multi */

	/* hdmi Mixer */
	writel( 0xffff0000, (SOC_CPIPE_BASE + 0x8db0)); /* alpha blend enable */
	writel( 0x000003fc, (SOC_CPIPE_BASE + 0x8db8)); /* per pixel alpha */

	/* denc Mixer */
	writel( 0xffff0000, (SOC_CPIPE_BASE + 0x8dd0)); /* alpha blend enable */
	writel( 0x000003fc, (SOC_CPIPE_BASE + 0x8dd8)); /* per pixel alpha */

	/* Set up HD layer */
	writel( 0x01e002d0, (SOC_CPIPE_BASE + 0x8c04)); /* width and height */
	writel( 0x01e002d0, (SOC_CPIPE_BASE + 0x8c08)); /* width and height */
	writel( 0x00880028, (SOC_CPIPE_BASE + 0x8c10)); /* h and v start position */
	writel( 0x00000002, (SOC_CPIPE_BASE + 0x8c20)); /* enable crop */
	writel( 0x02d001e0, (SOC_CPIPE_BASE + 0x8c28)); /* pix dim after crop */
	writel( 0x00000006, (SOC_CPIPE_BASE + 0x8c34)); /* crop enable match STG field */

	/* GFX VCBM*/
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x8e90)); /* Bypass GFX VCBM */
// writel( 0x00000001, (SOC_CPIPE_BASE + 0x8c00)); /* apply settings to layer */

	/* GNSH Settings */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x94a0)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x94a4)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x94a8)); /* Turn off MSB inversion */
	writel( 0x00000004, (SOC_CPIPE_BASE + 0x94b0)); /* Turn off MSB inversion */
	writel( 0x00000007, (SOC_CPIPE_BASE + 0x94b4)); /* Turn off MSB inversion */

	/* Set up color space conversions  RGB255 -> 601 */
	writel( 0x00000016, (SOC_CPIPE_BASE + 0x9690)); /* width and height */
	writel( 0x020e0408, (SOC_CPIPE_BASE + 0x9694)); /* width and height */
	writel( 0x00c91ed0, (SOC_CPIPE_BASE + 0x9698)); /* width and height */
	writel( 0x1dac0384, (SOC_CPIPE_BASE + 0x969c)); /* width and height */
	writel( 0x03841d0f, (SOC_CPIPE_BASE + 0x96a0)); /* width and height */
	writel( 0x1f6e0000, (SOC_CPIPE_BASE + 0x96a4)); /* width and height */
	writel( 0x00400000, (SOC_CPIPE_BASE + 0x96a8)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x96ac)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x96b0)); /* width and height */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x9400)); /* width and height */

	/* HDMI AVI Infoframe */
	writel( 0x000d0282, (SOC_HDMI_TX_BASE + 0x140));
	writel( 0x005840D5, (SOC_HDMI_TX_BASE + 0x144));
	writel( 0x00000002, (SOC_HDMI_TX_BASE + 0x148)); /* format = 2 */
	writel( 0x00000000, (SOC_HDMI_TX_BASE + 0x14c));
	writel( 0x00001403, (SOC_HDMI_TX_BASE + 0x310));

	/* HDMI GNSH Settings */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x98a0)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x98a4)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x98a8)); /* Turn off MSB inversion */
	writel( 0x00000004, (SOC_CPIPE_BASE + 0x98b0)); /* Turn off MSB inversion */
	writel( 0x00000007, (SOC_CPIPE_BASE + 0x98b4)); /* Turn off MSB inversion */

	/* HDMI OUTC */
	writel( 0x04080200, (SOC_CPIPE_BASE + 0x98c0));	 /* For CPIPE-HDMI */
	writel( 0x030f0002, (SOC_CPIPE_BASE + 0x98e0));	 /* For CPIPE-HDMI RGB match */
	writel( 0x000fff00, (SOC_CPIPE_BASE + 0x98e4));	 /* For CPIPE-HDMI RGB match */
	writel( 0x000fff00, (SOC_CPIPE_BASE + 0x98e8));	 /* For CPIPE-HDMI RGB match */

	/* HDMI Set up color space conversions  RGB255 -> 601 */
	writel( 0x00000016, (SOC_CPIPE_BASE + 0x9a90)); /* width and height */
	writel( 0x020e0408, (SOC_CPIPE_BASE + 0x9a94)); /* width and height */
	writel( 0x00c91ed0, (SOC_CPIPE_BASE + 0x9a98)); /* width and height */
	writel( 0x1dac0384, (SOC_CPIPE_BASE + 0x9a9c)); /* width and height */
	writel( 0x03841d0f, (SOC_CPIPE_BASE + 0x9aa0)); /* width and height */
	writel( 0x1f6e0000, (SOC_CPIPE_BASE + 0x9aa4)); /* width and height */
	writel( 0x00400000, (SOC_CPIPE_BASE + 0x9aa8)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x9aac)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x9ab0)); /* width and height */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x9800)); /* width and height */

	/* set up SD RIF for GFX */
	writel( 0xa000001f, (SOC_CPIPE_BASE + 0x4c44)); /* 32 bit variable width */
	writel( SOC_VARI_FORMAT_ARGB, (SOC_CPIPE_BASE + 0x4c48)); /* 32 bit variable width */
	writel( 0x80500013, (SOC_CPIPE_BASE + 0x4c54)); /* start DMA at line 8 pixel 0x90 */
	writel( 0x00000b3f, (SOC_CPIPE_BASE + 0x4c5c)); /* width is 719 pels */
	writel( uMALONE_start, (SOC_CPIPE_BASE + 0x4c64)); /* Buffer A Address */
	writel( uMALONE_start + 0xb40, (SOC_CPIPE_BASE + 0x4c68)); /* Buffer B Address */
	writel( 0x00001680, (SOC_CPIPE_BASE + 0x4c6c)); /* stride is 720*4 */
	writel( 0x00001680, (SOC_CPIPE_BASE + 0x4c70)); /* stride is 720*4 */
	writel( 0x00001680, (SOC_CPIPE_BASE + 0x4c84)); /* stride is 720*4 */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x4d90)); /* bypass alpha multi */

	/* Setup SD layer */
	writel( 0x00f002d0, (SOC_CPIPE_BASE + 0x4c04)); /* width and height */
	writel( 0x00f002d0, (SOC_CPIPE_BASE + 0x4c08)); /* width and height */
	writel( 0x00880014, (SOC_CPIPE_BASE + 0x4c10)); /* h and v start position */
	writel( 0x00000002, (SOC_CPIPE_BASE + 0x4c20)); /* enable crop */
	writel( 0x02d000f0, (SOC_CPIPE_BASE + 0x4c28)); /* pix dim after crop */
	writel( 0x00000006, (SOC_CPIPE_BASE + 0x4c34)); /* crop enable match STG field */

	/* GFX VCBM */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x4e90)); /* Bypass GFX VCBM */
	// writel( 0x00000001, (SOC_CPIPE_BASE + 0x4c00)); /* apply settings to layer */

	/* GNSH Settings */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x54a0)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x54a4)); /* Turn off MSB inversion */
	writel( 0x00000010, (SOC_CPIPE_BASE + 0x54a8)); /* Turn off MSB inversion */
	writel( 0x00000004, (SOC_CPIPE_BASE + 0x54b0)); /* Turn off MSB inversion */
	writel( 0x00000007, (SOC_CPIPE_BASE + 0x54b4)); /* Turn off MSB inversion */

	/* Set up color space conversions  RGB255 -> 601 */
	writel( 0x00000016, (SOC_CPIPE_BASE + 0x5690)); /* width and height */
	writel( 0x020e0408, (SOC_CPIPE_BASE + 0x5694)); /* width and height */
	writel( 0x00c91ed0, (SOC_CPIPE_BASE + 0x5698)); /* width and height */
	writel( 0x1dac0384, (SOC_CPIPE_BASE + 0x569c)); /* width and height */
	writel( 0x03841d0f, (SOC_CPIPE_BASE + 0x56a0)); /* width and height */
	writel( 0x1f6e0000, (SOC_CPIPE_BASE + 0x56a4)); /* width and height */
	writel( 0x00400000, (SOC_CPIPE_BASE + 0x56a8)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x56ac)); /* width and height */
	writel( 0x02000000, (SOC_CPIPE_BASE + 0x56b0)); /* width and height */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x5400)); /* width and height */

#if defined (CONFIG_ARCH_KRONOS)
	/* Color fix for Kronos Rev B */
	if (ChipRevID >= 0x1) {
		writel( 0x02060502, (SOC_CPIPE_BASE + 0x54e0)); /* !hsync, blank to SD DENC */
		writel( 0x030f0502, (SOC_CPIPE_BASE + 0x94e0)); /* !hsync, !vsync, blank, odd/even to HD DENC */
		writel( 0x030f0502, (SOC_CPIPE_BASE + 0x98e0)); /* For CPIPE-HDMI RGB match */
	}
#endif
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x4c00)); /* apply settings to layer */
	writel( 0x00000001, (SOC_CPIPE_BASE + 0x8c00)); /* apply settings to layer */
}
