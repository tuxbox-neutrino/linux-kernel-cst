/* linux/include/asm-arm/arch-nevis/uncompress.h
 *
 *  Copyright (C) 2008 Coolstream International Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 *(at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

/* This code is only necessary if:
 *  - you use a zImage 
 *  - your architecture can only talk to you via serial ports
 *  - you wanna see the "Uncompressing Linux..." on the serial port
 */

#include <linux/autoconf.h>
#include <asm/arch/cx2450x.h>

#ifdef CONFIG_SERIAL_CX2450X_BOOTMSG
#define BAUDRATE	CONFIG_CX2450X_BAUD_RATE

static int have_uart[3];

#endif	/* CONFIG_SERIAL_CX2450X_BOOTMSG */

/*******************************************************************************/

static inline void putc(char c)
{
#ifdef CONFIG_SERIAL_CX2450X_BOOTMSG
    u32 val;
    volatile u8 *FIFO;
    volatile u8 *TXST;

    for (val = 0; val < 3; val++)
    {
	if (have_uart[val])
	{
	    FIFO = (volatile u8*)(UART_FIFO_BRDL_REG(val));
	    TXST = (volatile u8*)(UART_TXSTA_REG(val));

	    while (*TXST & 0x1F);
	    *FIFO = c;

	    if (c == '\n')
	    {
		while (*TXST & 0x1F);
		*FIFO = '\r';
	    }
	}
    }
#endif	/* CONFIG_SERIAL_CX2450X_BOOTMSG */
}

/*******************************************************************************/

static inline void flush(void)
{
	/* nothing to do here */
}

/*******************************************************************************/

static __inline__ void arch_decomp_setup(void)
{
#ifdef CONFIG_SERIAL_CX2450X_BOOTMSG
    volatile u32 *reg;
    volatile u8 *FIFO;
    volatile u8 *IRQE;
    volatile u8 *FIFC;
    volatile u8 *FRMC;
    u32 val;
    u32 brdiv = (54000000 / (16 * BAUDRATE)) - 1;

#ifdef CONFIG_SERIAL_CX2450X_UART1_ENABLE
    /* check for UART 1 (PIO 1 (TX), PIO 2 (RX)) */
    have_uart[0] = 0;
    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
    if ((*reg & 0x00000006) == 0)
    {
	reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
	if ((*reg & 0x00000006) == 0x00000006)
	    have_uart[0] = 1;
    }
#endif

#ifdef CONFIG_SERIAL_CX2450X_UART2_ENABLE
    /* check for UART 2 (can be configured on various pins) */
    have_uart[1] = 0;
    reg = (volatile u32*) SREG_ALT_PIN_FUNC_REG;
    val = (*reg >> 4) & 0x03;
    if (val == 0)	/* PIO 3 (TX), 4 (RX) */
    {
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
	if ((*reg & 0x00000018) == 0)
	{
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
	    if ((*reg & 0x00000018) == 0x00000018)
		have_uart[1] = 1;
	}
    
    }
    else if (val == 1)	/* PIO 71 (RX), 72 (TX) */
    {
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(2);
	if ((*reg & 0x00000018) == 0)
	{
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(2);
	    if ((*reg & 0x00000018) == 0x00000018)
		have_uart[1] = 1;
	}
    }
    else if (val == 2)	/* PIO 11 (RX), 73 (TX) */
    {
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(2);
	if ((*reg & 0x00000020) == 0)
	{
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(2);
	    if ((*reg & 0x00000020) == 0x00000020)
	    {
		reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
		if ((*reg & 0x00000800) == 0)
		{
		    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
		    if ((*reg & 0x00000800) == 0x00000800)
			have_uart[1] = 1;
		}
	    }
	}
    }
#endif

#ifdef CONFIG_SERIAL_CX2450X_UART3_ENABLE
    /* check for UART 3 (PIO 14 (TX), PIO 15 (RX)) */
    have_uart[2] = 0;
    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
    if ((*reg & 0x0000C000) == 0)
    {
	reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
	if ((*reg & 0x0000C000) == 0x0000C000)
	    have_uart[2] = 1;
    }

    for (val = 0; val < 3; val++)
    {
	if (have_uart[val])
	{
	    FIFO = (volatile u8*)(UART_FIFO_BRDL_REG(val));
	    IRQE = (volatile u8*)(UART_IRQE_BRDU_REG(val));
	    FIFC = (volatile u8*)(UART_FIFC_REG(val));
	    FRMC = (volatile u8*)(UART_FRMC_REG(val));

    	    /* setup Baudradte */
    	    *FRMC |= 0x80;		/* set BDS to access FIFO and IRQE as baudrate registers */
    	    *FIFO = brdiv & 0xFF;
    	    *IRQE = (brdiv >> 8) & 0xFF;
    	    *FRMC &= 0x7F;
    	    *IRQE = 0x00;
    	    *FRMC = 0x01;		/* 8 databits, 1 stopbit, no parity */
    	    *FIFC = 0x03;		/* clear RX-RX-FIFO */
	}
    }
#endif
#endif	/* CONFIG_SERIAL_CX2450X_BOOTMSG */
}

/*******************************************************************************/

#define arch_decomp_wdog()

#endif /* __ASM_ARCH_UNCOMPRESS_H */
