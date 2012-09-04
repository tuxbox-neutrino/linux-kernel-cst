/************************************************************************************
 * linux/include/asm-arm/arch-nevis/cx2450x.h
 *
 * global register definitions
 *
 *  Copyright (C) 2008 Coolstream International Limited
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
 *************************************************************************************/

#ifndef __CX2450X_H
#define __CX2450X_H

/* CX2450X Base clock frequency */
#define NEVIS_XTAL_FREQUENCY		UL(60000000)

//#define HSX_BASE                	(0xE0000000)
#define HSX_PIT_GENERAL_REG		(0xE0000100)
#define HSX_PIT_DATA_REG		(0xE0000104)
#define HSX_PIT_INSTR_REG		(0xE0000108)

/* ISA Aperture */
#define ISA_IO_BASE_ADDR		(0xE1000000)
#define ISA_IO_BANK_SIZE		(0x00100000)

/* ROM/ISA Mapping */
#define ROM_DESC_REG_BASE(x)		(0xE0010000 + ((x) * 4))	/* ROM descriptor base (x = 0 ... 7) */
#define ROM_MAP_REG_BASE(x)		(0xE0010020 + ((x) * 4))	/* ROM mapping register base (x = 0 ... 7) */
#define ROM_EXT_DESC_REG_BASE(x)	(0xE0010080 + ((x) * 4))	/* ROM ext. descriptor base (x = 0 ... 7) */
/* PCI Interface */
//#define PCI_BASE			(0xE0010000)
#define PCI_ROM_DESC0_REG		(0xE0010000)
#define PCI_ISAROM_DESC1_REG		(0xE0010004)
#define PCI_ROM_DESC0_REG2		(0xE0010080)
#define PCI_CFG_ADDR_REG		(0xE0010040)
#define PCI_CFG_DATA_REG		(0xE0010044)
#define PCI_INTSTAT_REG			(0xE0010054)			/* PCI Controller Interrupt Status Register */
/* System controller */
#define SYS_SOFTRESET_REG		(0xE040001C)			/* writing anything causes a reset */

/* UART's 
   (16550 compatible except: FIFOs allways on, no DMA mode select, 
   no support for 5 and 6 bit data frames, no scratch register) */
#define	UART_FIFO_BRDL_REG(x)		(0xE0410000 + ((x) * 0x1000))	/* FIFO (BDS=0) or lower baud rate divisor (BDS=1) register (x = 0 ... 3) */
#define	UART_IRQE_BRDU_REG(x)		(0xE0410004 + ((x) * 0x1000))	/* Interrupt enable (BDS=0) or upper baud rate divisor (BDS=1) register (x = 0 ... 3) */
#define	UART_FIFC_REG(x)		(0xE0410008 + ((x) * 0x1000))	/* FIFO control register (x = 0 ... 3) */
#define	UART_FRMC_REG(x)		(0xE041000C + ((x) * 0x1000))	/* Frame control register (x = 0 ... 3) */
#define	UART_STAT_REG(x)		(0xE0410014 + ((x) * 0x1000))	/* Status register (x = 0 ... 3) */
#define	UART_IRLVL_REG(x)		(0xE0410018 + ((x) * 0x1000))	/* Interrupt level register (x = 0 ... 3) */
#define	UART_IRDC_REG(x)		(0xE0410020 + ((x) * 0x1000))	/* IrDA control register (x = 0 ... 3) */
#define	UART_TXSTA_REG(x)		(0xE0410028 + ((x) * 0x1000))	/* Transmit FIFO status register (x = 0 ... 3) */
#define	UART_RXSTA_REG(x)		(0xE041002C + ((x) * 0x1000))	/* Receive FIFO status register (x = 0 ... 3) */
#define	UART_EXP_REG(x)			(0xE0410030 + ((x) * 0x1000))	/* Expansion register (x = 0 ... 3) */

/* General Purpos Timers */
#define TIMER_VALUE_REG_BASE(x)		(0xE0430000 + ((x) * 0x10))	/* the current timer value (x = 0 ... 15) */
#define TIMER_LIMIT_REG_BASE(x)		(0xE0430004 + ((x) * 0x10))	/* the timer limit value (x = 0 ... 15) */
#define TIMER_MODE_REG_BASE(x)		(0xE0430008 + ((x) * 0x10))	/* the mode bits (x = 0 ... 15) */
#define TIMER_BASE_REG_BASE(x)		(0xE043000C + ((x) * 0x10))	/* the clock base (x = 0 ... 15) */
#define TIMER_INT_STAT_REG		(0xE0430100)			/* the interrupt status */

/* PLL */
#define PLL_BASE			(0xE0440000)
#define PLL_MPG0_INTFRAC_REG		(0xE0440000)
#define PLL_MPG0_CTRL_REG		(0xE0440004)
#define PLL_FENRUS_CTRL_REG		(0xE0440048)
#define PLL_DIV_MUX_CTRL1_REG		(0xE0440054)
#define PLL_DIV_MUX_CTRL10_REG		(0xE0440078)

#define PLL_CONFIG0_REG			(0xE0440100)

/* System Registers */
#define SREG_ALT_PIN_FUNC_REG		(0xE0440110)			/* alternate pin function register */
#define	SREG_USB_ENABLE_REG		(0xE0440138)
#define SREG_PRI_MUX_REG_BASE(x)	(0xE0440180 + ((x) * 4))	/* Primary PIO multiplex select register (x = 0 ... 6) */
#define SREG_SEC_MUX_REG_BASE(x)	(0xE04401C0 + ((x) * 4))	/* Secondary PIO multiplex select register (x = 0 ... 6) */

/* Interrupt Controller */
#define ITC_DEST_REG_BASE(x)		(0xE0450000 + ((x) * 0x20))	/* Interrupt Destination Registers (x = 0 ... 3) */
#define ITC_ENABLE_REG_BASE(x)		(0xE0450004 + ((x) * 0x20))	/* Interrupt Enable Registers (x = 0 ... 3) */
#define ITC_IRQREQ_REG_BASE(x)		(0xE0450008 + ((x) * 0x20))	/* Interrupt Enable Registers (x = 0 ... 3) */
#define ITC_STATCLR_REG_BASE(x)		(0xE0450010 + ((x) * 0x20))	/* Interrupt Status/Clear Registers (x = 0 ... 3) */
#define ITC_STATSET_REG_BASE(x)		(0xE0450014 + ((x) * 0x20))	/* Interrupt Status/Set Registers (x = 0 ... 3) */

/* Infrared Remote Controllers */
#define IR_CTRL_REG_BASE(x)		(0xE0460000 + ((x) * 0x1000))	/* Control register (x = 0 ... 1) */
#define IR_TXCLK_REG_BASE(x)		(0xE0460004 + ((x) * 0x1000))	/* TX-clock divider register (x = 0 ... 1) */
#define IR_RXCLK_REG_BASE(x)		(0xE0460008 + ((x) * 0x1000))	/* RX-clock divider register (x = 0 ... 1) */
#define IR_CDUTY_REG_BASE(x)		(0xE046000C + ((x) * 0x1000))	/* TX-carrier duty cycle register (x = 0 ... 1) */
#define IR_STAT_REG_BASE(x)		(0xE0460010 + ((x) * 0x1000))	/* Status register (x = 0 ... 1) */
#define IR_IRQEN_REG_BASE(x)		(0xE0460014 + ((x) * 0x1000))	/* Interrupt enable register (x = 0 ... 1) */
#define IR_FILTER_REG_BASE(x)		(0xE0460018 + ((x) * 0x1000))	/* Low pass filter register (x = 0 ... 1) */
#define IR_FIFO_REG_BASE(x)		(0xE0460040 + ((x) * 0x1000))	/* FIFO register (x = 0 ... 1) */

/* General Purpose I/O Controller */
#define GPIO_READ_REG_BASE(x)		(0xE0470000 + ((x) * 0x40))	/* Read registers (x = 0 ... 6) */
#define GPIO_DRIVE_HIGH_REG_BASE(x)	(0xE0470004 + ((x) * 0x40))	/* Drive high registers (x = 0 ... 6) */
#define GPIO_DRIVE_LOW_REG_BASE(x)	(0xE0470008 + ((x) * 0x40))	/* Drive low registers (x = 0 ... 6) */
#define GPIO_DRIVE_OFF_REG_BASE(x)	(0xE047000C + ((x) * 0x40))	/* Drive off registers (x = 0 ... 6) */
#define GPIO_INTSTAT_REG_BASE(x)	(0xE0470010 + ((x) * 0x40))	/* Interrupt Status Registers (x = 0 ... 6) */
#define GPIO_INTENA_REG_BASE(x)		(0xE0470014 + ((x) * 0x40))	/* Interrupt Enable Registers (x = 0 ... 6 ) */
#define GPIO_POS_EDGE_REG_BASE(x)	(0xE0470018 + ((x) * 0x40))	/* positive edge trigger register (x = 0 ... 6) */
#define GPIO_NEG_EDGE_REG_BASE(x)	(0xE047001C + ((x) * 0x40))	/* negative trigger registers (x = 0 ... 6) */
#define GPIO_LEVEL_REG_BASE(x)		(0xE0470020 + ((x) * 0x40))	/* level trigger registers (x = 0 ... 6) */

/* memory controller */
//#define MEM_PLL_CTRL0_REG		0xE0500800
#define MEM_PLL_CTRL1_REG		0xE0500804
#define MEM_PLL_CTRL2_REG		0xE0500808
//#define MEM_PLL_CTRL3_REG		0xE050080C

/* Network controller(s) */
#define EMAC0_BASE			(0xE8003000)
#define EMAC1_BASE			(0xE8004000)

#define EMAC_ID_REG(x)			(0xE8003000 + ((x) * 0x1000))
#define EMAC_STAT_REG(x)		(0xE8003004 + ((x) * 0x1000))
#define EMAC_ENABLE_REG(x)		(0xE8003008 + ((x) * 0x1000))
#define EMAC_CONTROL_REG(x)		(0xE800300C + ((x) * 0x1000))
#define EMAC_POLLRATE_REG(x)		(0xE8003010 + ((x) * 0x1000))
#define EMAC_RXERR_REG(x)		(0xE8003014 + ((x) * 0x1000))
#define EMAC_MISS_REG(x)		(0xE8003018 + ((x) * 0x1000))
#define EMAC_TXRINGPTR_REG(x)		(0xE800301C + ((x) * 0x1000))
#define EMAC_RXRINGPTR_REG(x)		(0xE8003020 + ((x) * 0x1000))
#define EMAC_ADDRL_REG(x)		(0xE8003024 + ((x) * 0x1000))
#define EMAC_ADDRH_REG(x)		(0xE8003028 + ((x) * 0x1000))
#define EMAC_LAFL_REG(x)		(0xE800302C + ((x) * 0x1000))
#define EMAC_LAFH_REG(x)		(0xE8003030 + ((x) * 0x1000))
#define EMAC_MDIO_REG(x)		(0xE8003034 + ((x) * 0x1000))
#define EMAC_TXPTRREAD_REG(x)		(0xE8003038 + ((x) * 0x1000))
#define EMAC_RXPTRREAD_REG(x)		(0xE800303C + ((x) * 0x1000))
#define EMAC_XTRACTRL_REG(x)		(0xE8003040 + ((x) * 0x1000))

/* vaious definitions */
#define CHIP_CRYSTAL_FREQUENCY		60000000			/* SoC is clocked by a 60.000 MHz crystal */
#define XTAL_PRESCALE_FACTOR		1

#define MPG0PLL				0x00
#define MPG1PLL				0x01
#define HDPLL				0x02
#define AUDPLL				0x03
//#define MEMPLL				0x04
#define PLL0				0x05
#define PLL1				0x06
#define PLL2				0x07
#define FENRUSPLL			0x08

#endif /* __CX2450X_H */
