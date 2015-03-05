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


#include <mach/soc.h>
#include <asm/io.h>
#include <asm/hardware/ip3106.h>

#if defined(CONFIG_STB_EARLY_UART1)
#define UART_BASE	(IO_PERI_OFFSET+UART1_PORT0_OFFSET)
#elif defined(CONFIG_STB_EARLY_UART2)
#define UART_BASE	(IO_PERI_OFFSET+UART2_PORT1_OFFSET)
#elif defined(CONFIG_STB_EARLY_UART3)
#define UART_BASE	(IO_PERI_OFFSET+UART3_PORT2_OFFSET)
#elif defined(CONFIG_STB_EARLY_UART4)
#define UART_BASE	(IO_PERI_OFFSET+UART4_PORT3_OFFSET)
#else
#define UART_BASE	(IO_PERI_OFFSET+UART1_PORT0_OFFSET)
#endif

#define UART_THR ((volatile void __iomem *)(UART_BASE+IP3106_UART_THR_REG))
#define UART_LSR ((volatile void __iomem *)(UART_BASE+IP3106_UART_LSR_REG))

static void putc(const char c)
{
	int status;

	/* Transmit fifo not full? */
	do {
		status = __raw_readl(UART_LSR);
	} while (!(status & IP3106_UART_LSR_THRE_MSK));

	__raw_writel(c,UART_THR);
}

static void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
