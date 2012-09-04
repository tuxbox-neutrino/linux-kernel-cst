/*
 *  linux/drivers/serial/cx2450x_uart.c
 *
 *  Driver for Conexant CX2450x SoC serial ports
 *
 *  Based on drivers/serial/cnxt.c, by Deep Blue Solutions which was 
 *    based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/autoconf.h>

#if defined(CONFIG_SERIAL_CX2450X_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/memmap.h>
#include <asm/arch/cx2450x.h>
#include <asm/arch/irqs.h>

#include <linux/serial_core.h>
#include <asm/hardware/serial_cnxt.h>

#if defined(CONFIG_ARCH_PECOS) || defined(CONFIG_ARCH_NEVIS)
#define CNXT_SET(reg,mask,val)	(*(LPREG)(reg)) =  ((*(LPREG)(reg) & ~(mask)) | ((val) & (mask)))
#endif

#define CNXT_ISR_PASS_LIMIT	256

/* Access macros for the CNXT UARTs */
#define UART_GET_INT_STATUS(p)	readl((p)->membase + CNXT_IRLVL)

#define UART_PUT_BRDL(p, c)     writel((c), (p)->membase + CNXT_BRDL)
#define UART_PUT_BRDH(p, c)     writel((c), (p)->membase + CNXT_BRDH)

#define UART_GET_CHAR(p)	readl((p)->membase + CNXT_FIFO)
#define UART_PUT_CHAR(p, c)	writel((c), (p)->membase + CNXT_FIFO)

#define UART_GET_STAT(p)	readl((p)->membase + CNXT_STAT)

#define UART_GET_IRQE(p)	readl((p)->membase + CNXT_IRQE)
#define UART_PUT_IRQE(p,c)	writel((c), (p)->membase + CNXT_IRQE)

#define UART_GET_FIFC(p)	readl((p)->membase + CNXT_FIFC)
#define UART_PUT_FIFC(p,c)	writel((c), (p)->membase + CNXT_FIFC)

#define UART_GET_FRMC(p)	readl((p)->membase + CNXT_FRMC)
#define UART_PUT_FRMC(p,c)	writel((c), (p)->membase + CNXT_FRMC)

#define UART_RX_NUMCHARS(p)	readl((p)->membase + CNXT_RXSTA)

#define UART_TX_READY(s)	(((s) & CNXT_STAT_TID) != 0)

#define UART_TX_EMPTY(p)	((UART_GET_STAT(p) & CNXT_STAT_TID) != 0)

#define UART_PUT_EXP(p, c)    writel((c), (p)->membase + CNXT_EXP)

#define UART_DUMMY_RSR_RX	256
#define UART_PORT_SIZE		0x34

/* On the CNXT boards, we're not going to implement RTS/CTS flow control
 * (not yet anyway).  These would be GPIO's however.
 */
#define SC_CTRLC	(while (0);)
#define SC_CTRLS	(while (0);)

/* We wrap our port structure around the generic uart_port. */

struct uart_cnxt_port 
{
    struct uart_port port;
    u32 	     num;
    u32		     dtr_enable;
    u32		     rts_enable;
    u32		     old_status;
};

static u32 have_uart[3];

#ifdef CONFIG_GDB_STUB_FOR_ARM
/* Added by harsh for GDB kernel debugging support */
int irq2_status = 0;
/* Added for GDB kernel debugging support */
/* Serial port has to be initialized */
static volatile unsigned char *port = NULL;

#define GDB_UART_BASE			IO_ADDRESS(CNXT_PORT0_BASE)
#define GDB_UART_GET_CHAR(p)		readl((p) + CNXT_FIFO)
#define GDB_UART_PUT_CHAR(p, c)		writel((c), (p) + CNXT_FIFO)
#define GDB_UART_GET_STAT(p)		readl((p) + CNXT_STAT)

/* Get a character from serial port */
int debug_getc(void)
{
	unsigned int status;
	int c;

	port = (unsigned char *) GDB_UART_BASE;

	status = GDB_UART_GET_STAT(port);

	while (!(status & CNXT_STAT_RSR)) {
		status = GDB_UART_GET_STAT(port);
	}
	c = GDB_UART_GET_CHAR(port);

	c = c & 0x7f;

	return c;
}

EXPORT_SYMBOL(debug_getc);
/* Put a character to the serial port */
int debug_putc(int c)
{
	unsigned int status;

	port = (unsigned char *) GDB_UART_BASE;

	status = GDB_UART_GET_STAT(port);

	while (!(status & CNXT_STAT_TSR)) {
		status = GDB_UART_GET_STAT(port);
	}

	GDB_UART_PUT_CHAR(port, (c | 0x80));
	return c;
}

EXPORT_SYMBOL(debug_putc);
#endif

static void cnxtuart_stop_tx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IRQE(port);
	/* turn off fifo threshold interrupt and transmitter idle interrupt */
	/* TODO: Check if this is appropriate.  Only one may need to go off */
	cr &= ~(CNXT_IRQE_TIDE | CNXT_IRQE_TSRE);
	UART_PUT_IRQE(port, cr);
}

static void cnxtuart_start_tx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IRQE(port);
	cr |= (CNXT_IRQE_TIDE);
	UART_PUT_IRQE(port, cr);
}

static void cnxtuart_stop_rx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IRQE(port);
	cr &= ~(CNXT_IRQE_RSRE);
	UART_PUT_IRQE(port, cr);
}

/* CNXT doesn't really have modem status */
static void cnxtuart_enable_ms(struct uart_port *port)
{

}

static void cnxtuart_rx_chars(struct uart_port *port)
{
	struct tty_struct *tty = port->info->tty;
	unsigned int ch, rsr, max_count = 256;
	unsigned int rxchars;
	char flag;

	rxchars = UART_RX_NUMCHARS(port);
	while (rxchars && max_count--) {
		/* Status bits on CNXT refer to the bottom character
		 * on the FIFO.   If we read the character before the status 
		 * register, the status won't apply to the current character.
		 */
		rsr = UART_GET_STAT(port) | UART_DUMMY_RSR_RX;
		ch = UART_GET_CHAR(port);

		tty_flip_buffer_push(tty);

		flag = TTY_NORMAL;
		port->icount.rx++;

		if (rsr & CNXT_STAT_ANY) {
			if (rsr & CNXT_STAT_RBK) {
				rsr &= ~(CNXT_STAT_FRE | CNXT_STAT_PAE);
				port->icount.brk++;
				if (uart_handle_break(port)) {
					goto ignore_char;
				}
			} else if (rsr & CNXT_STAT_PAE)
				port->icount.parity++;
			else if (rsr & CNXT_STAT_FRE)
				port->icount.frame++;
			if (rsr & CNXT_STAT_RFO)
				port->icount.overrun++;

			rsr &= port->read_status_mask;

			if (rsr & CNXT_STAT_RBK)
				flag = TTY_BREAK;

			else if (rsr & CNXT_STAT_PAE)
				flag = TTY_PARITY;
			else if (rsr & CNXT_STAT_FRE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		uart_insert_char(port, rsr, CNXT_STAT_RFO, ch, flag);

	      ignore_char:
		rxchars = UART_RX_NUMCHARS(port);
	}
	/*
	 * Drop the lock here since it might end up calling
	 * uart_start(), which takes the lock.
	 */

	tty_flip_buffer_push(tty);
	return;
}

static void cnxtuart_tx_chars(struct uart_port *port)
{
	/* This funciton is called from the UART ISR if the transmitter
	 * empty interrupt fired */
	struct circ_buf *xmit = &port->info->xmit;
	int count;
	unsigned int status;

	if (port->x_char)
	{
		do {
			status = UART_GET_STAT(port);
			if (UART_TX_READY(status))
				break;
			cpu_relax();
		} while (1);

		UART_PUT_CHAR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) 
	{
		cnxtuart_stop_tx(port);
		return;
	}

	count = port->fifosize >> 1;
	do 
	{
		do 
		{
			status = UART_GET_STAT(port);
			if (UART_TX_READY(status))
				break;
			cpu_relax();
		} while (1);

		UART_PUT_CHAR(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		cnxtuart_stop_tx(port);
}

static irqreturn_t cnxtuart_int(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status, pass_counter = CNXT_ISR_PASS_LIMIT;

	/* there should really be a need to read the interrupt level register.
	 * In either of the interruptable cases below, they will be handled
	 * quickly.  Furthermore, the framing and parity errors will be handled
	 * in the rx() routine
	 */
	status = UART_GET_STAT(port);
	do {
		if (status & (CNXT_STAT_RSR))
			cnxtuart_rx_chars(port);
		if (status & CNXT_STAT_TSR)
			cnxtuart_tx_chars(port);

		if (pass_counter-- == 0)
			break;

		status = UART_GET_STAT(port);
	} while (status & (CNXT_STAT_RSR | CNXT_STAT_TSR));

	return IRQ_HANDLED;
}

static unsigned int cnxtuart_tx_empty(struct uart_port *port)
{
	return UART_GET_STAT(port) & CNXT_STAT_TID ? 0 : TIOCSER_TEMT;
}

static unsigned int cnxtuart_get_mctrl(struct uart_port *port)
{
	unsigned int result = 0;

	/* TODO: This may need changing in the future.
	 *
	 * Lie and say we're good to go for apps that are picky about 
	 * hardware flow control
	 */
	result = TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;

	return result;
}

static void cnxtuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* No modem control stuff on CNXT for now... */
}

static void cnxtuart_break_ctl(struct uart_port *port, int break_state)
{
	/* Dummy */
}

static int cnxtuart_startup(struct uart_port *port)
{
	struct uart_cnxt_port *uap = (struct uart_cnxt_port *) port;
	int retval;

	/*  Allocate the IRQ */
	retval = request_irq(port->irq, cnxtuart_int, /*IRQF_DISABLED | */IRQF_SHARED, "CX2450x UART", port);
	if (retval)
		return retval;

	/*  initialise the old status of the modem signals */
	uap->old_status = 0;

	UART_PUT_FIFC(port, CNXT_FIFC_RFT_EIGHT | CNXT_FIFC_TFT_SXTN | CNXT_FIFC_TFC | CNXT_FIFC_RFC);

	/* Finally, enable interrupts */
	UART_PUT_IRQE(port, CNXT_IRQE_TIDE | CNXT_IRQE_TSRE | CNXT_IRQE_RSRE);

#ifdef CONFIG_GDB_STUB_FOR_ARM
	/* Set that IRQ2 is allocated :to be used by GDB */
	irq2_status = 1;
#endif
	return 0;
}

static void cnxtuart_shutdown(struct uart_port *port)
{
	/* Free the interrupt */
	free_irq(port->irq, port);
	/* disable all interrupts, disable the port */
	UART_PUT_IRQE(port, 0);
	UART_PUT_FIFC(port, CNXT_FIFC_TFC | CNXT_FIFC_RFC);
}

static void cnxtuart_set_termios(struct uart_port *port, struct ktermios *new1, struct ktermios *old)
{
#warning check baudrate setup
	unsigned long flags;
	int baud = uart_get_baud_rate(port, new1, old, 9600, 115200);
	unsigned int fcr, old_cr, quot;
	unsigned int cflag = new1->c_cflag;

	quot = (port->uartclk / (16 * baud));
	switch (cflag & CSIZE) {
	case CS7:
		fcr = 0;
		break;
	default:		// CS8
		fcr = CNXT_FRMC_FRS;
		break;
	}

	if (cflag & CSTOPB)
		fcr |= CNXT_FRMC_SBS;

	if (cflag & PARENB) {
		fcr |= CNXT_FRMC_PEN;

		if (!(cflag & PARODD))
			fcr |= CNXT_FRMC_EOP;
	}

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, cflag, baud);
	/*  Ignore all characters if CREAD is not set.  */
	if ((cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_RSR_RX;

	old_cr = UART_GET_FRMC(port);

	UART_PUT_FRMC(port, 0x80);

	/* Set baud rate */
	/*
	 * Set the fractional part, based on the fact that we shifted the
	 * uartclk by 2, to allow 2 bits of fraction to be carried in the
	 * quotient value passed into this routine
	 */
	UART_PUT_EXP(port, (quot & 0x00003));
	quot = quot >> 2;

	quot -= 1;
	UART_PUT_BRDH(port, ((quot & 0xff00) >> 8));
	UART_PUT_BRDL(port, (quot & 0xff));

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	UART_PUT_FRMC(port, fcr);
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *cnxtuart_type(struct uart_port *port)
{
	return port->type == PORT_CNXT ? "CX2450x UART" : NULL;
}

/* Release the memory region(s) being used by 'port' */
static void cnxtuart_release_port(struct uart_port *port)
{
    release_mem_region(port->mapbase, UART_PORT_SIZE);
}

/* Request the memory region(s) being used by 'port' */
static int cnxtuart_request_port(struct uart_port *port)
{
    return request_mem_region(port->mapbase, UART_PORT_SIZE,  "CX2450x UART") != NULL ? 0 : -EBUSY;
}

/* Configure/autoconfigure the port.  */
static void cnxtuart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE) 
    {
	port->type = PORT_CNXT;
	cnxtuart_request_port(port);
    }
}

/* verify the new serial_struct (for TIOCSSERIAL).  */
static int cnxtuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret;
	/* TODO:  MFB - evaluate if this is really necessary later */
	ret = 0;
	return ret;
}

static struct uart_ops cnxt_pops = 
{
    .tx_empty	  = cnxtuart_tx_empty,
    .set_mctrl	  = cnxtuart_set_mctrl,
    .get_mctrl	  = cnxtuart_get_mctrl,
    .stop_tx	  = cnxtuart_stop_tx,
    .start_tx	  = cnxtuart_start_tx,
    .stop_rx	  = cnxtuart_stop_rx,
    .enable_ms	  = cnxtuart_enable_ms,
    .break_ctl	  = cnxtuart_break_ctl,
    .startup	  = cnxtuart_startup,
    .shutdown	  = cnxtuart_shutdown,
    .set_termios  = cnxtuart_set_termios,
    .type 	  = cnxtuart_type,
    .release_port = cnxtuart_release_port,
    .request_port = cnxtuart_request_port,
    .config_port  = cnxtuart_config_port,
    .verify_port  = cnxtuart_verify_port
};

/*******************************************************************************/

/* note, addresses and interrupts here are only valid placeholders. They are 
   overwritten in the init routine for proper sorting (to have the system 
   console while boot identical to the working console) */
static struct uart_cnxt_port cnxt_ports[3] =
{
    {
	.port = 
	{
	    .membase  = (void *) IO_ADDRESS(UART_FIFO_BRDL_REG(0)),
	    .mapbase  = UART_FIFO_BRDL_REG(0),
	    .iotype   = UPIO_MEM,
	    .irq      = IRQ_UART1,
	    .uartclk  = (54000000 << 2),
	    .fifosize = 16,
	    .ops      = &cnxt_pops,
	    .flags    = UPF_BOOT_AUTOCONF,
	    .line     = 2,
	},
	.dtr_enable = 0,
	.rts_enable = 0,
    },
    {
	.port = 
	{
	    .membase  = (void *) IO_ADDRESS(UART_FIFO_BRDL_REG(1)),
	    .mapbase  = UART_FIFO_BRDL_REG(1),
	    .iotype   = UPIO_MEM,
	    .irq      = IRQ_UART2,
	    .uartclk  = (54000000 << 2),
	    .fifosize = 16,
	    .ops      = &cnxt_pops,
	    .flags    = UPF_BOOT_AUTOCONF,
	    .line     = 1,
	},
	.dtr_enable = 0,
	.rts_enable = 0,
    },
    {
	.port = 
	{
	    .membase  = (void *) IO_ADDRESS(UART_FIFO_BRDL_REG(2)),
	    .mapbase  = UART_FIFO_BRDL_REG(2),
	    .iotype   = UPIO_MEM,
	    .irq      = IRQ_UART3,
	    .uartclk  = (54000000 << 2),
	    .fifosize = 16,
	    .ops      = &cnxt_pops,
	    .flags    = UPF_BOOT_AUTOCONF,
	    .line     = 0,
	},
	.dtr_enable = 0,
	.rts_enable = 0,
    },
};

/*******************************************************************************/

#ifdef CONFIG_SERIAL_CX2450X_CONSOLE

static void cnxtuart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &cnxt_ports[co->index].port;
	unsigned int status, old_cr;
	int i;

	/* First save the CR then disable the interrupts */
	old_cr = UART_GET_IRQE(port);
	UART_PUT_IRQE(port, 0);

	/* Now, do each character */
	for (i = 0; i < count; i++) {
		do {
			status = UART_GET_STAT(port);
			if (UART_TX_READY(status))
				break;
			cpu_relax();
		}
		while (1);
		UART_PUT_CHAR(port, s[i]);
		if (s[i] == '\n') {
			do {
				status = UART_GET_STAT(port);
				if (UART_TX_READY(status))
					break;
				cpu_relax();
			} while (1);
			UART_PUT_CHAR(port, '\r');
		}
	}

	/* Finally, wait for transmitter to become empty and restore the TCR */
	do {
		status = UART_GET_STAT(port);
	}
	while ((status & CNXT_STAT_TID) == 0);
	UART_PUT_IRQE(port, old_cr);
}

struct tty_driver *cnxtuart_console_device(struct console *co, int *index)
{
	struct uart_driver *p = co->data;

	*index = co->index;
	return p->tty_driver;
}

/*
static void __init cnxtuart_console_get_options(struct uart_port *port, int *baud, int *parity, int *bits)
{
#warning report correct settings
	*baud = 115200;
	*parity = 'n';
	*bits = 8;
}
*/
/*******************************************************************************/

static int __init cnxtuart_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    /* get default baudrate from Kernel config */
    #ifdef CONFIG_SERIAL_CX2450X_BAUDRATE_230400
    baud = 230400;
    #else
    #ifdef CONFIG_SERIAL_CX2450X_BAUDRATE_115200
    baud = 115200;
    #else
    #ifdef  CONFIG_SERIAL_CX2450X_BAUDRATE_57600
    baud = 57600;
    #else
    #ifdef  CONFIG_SERIAL_CX2450X_BAUDRATE_38400
    baud = 38400;
    #else
    #ifdef  CONFIG_SERIAL_CX2450X_BAUDRATE_19200
    baud = 19200;
    #else
    #ifdef  CONFIG_SERIAL_CX2450X_BAUDRATE_9600
    baud = 9600;
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_9600   */
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_19200  */
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_38400  */
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_57600  */
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_115200 */
    #endif  /* CONFIG_SERIAL_CX2450X_BAUDRATE_230400 */

    /* Check whether an invalid uart number has been specified, and if so, 
       search for the first available port that does have console support. */
    if (co->index > 2)
	co->index = 0;
    port = &cnxt_ports[co->index].port;

    if (options)
	uart_parse_options(options, &baud, &parity, &bits, &flow);

    return uart_set_options(port, co, baud, parity, bits, flow);
}

/*******************************************************************************/

static struct console cnxt_console;
static struct uart_driver cnxt_reg;

/*******************************************************************************/

static struct console cnxt_console = 
{
    .name   = "ttyRI",
    .write  = cnxtuart_console_write,
    .device = cnxtuart_console_device,
    .setup  = cnxtuart_console_setup,
    .flags  = CON_PRINTBUFFER,
    .index  = -1,
    .data   = &cnxt_reg,
};

static int __init cnxtuart_console_init(void)
{
    u32 val;
    u32 num = 0;
    u32 irqtab[3] = {IRQ_UART1, IRQ_UART2, IRQ_UART3};
    volatile u32 *reg;

    #ifdef CONFIG_SERIAL_CX2450X_BOOTMSG

    /* check for available UART's (enabled by the Bootloader/BIOS).
       You may think, this is stupid, yes I think so too, but to have the early 
       bootmessages on the specified console, we have to detect the ports and to 
       sort the addresses and interrupts. */
#ifdef CONFIG_SERIAL_CX2450X_UART1_ENABLE
    /* UART 1 (PIO 1 (TX), PIO 2 (RX)) */
    have_uart[0] = 0xFF;
    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
    if ((*reg & 0x00000006) == 0)
    {
        reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
        if ((*reg & 0x00000006) == 0x00000006)
        {
	    have_uart[0] = num;
	    num++;
	}
    }

#endif
#ifdef CONFIG_SERIAL_CX2450X_UART2_ENABLE
    /* check for UART 2 (can be configured on various pins) */
    have_uart[1] = 0xFF;
    reg = (volatile u32*) SREG_ALT_PIN_FUNC_REG;
    val = (*reg >> 4) & 0x03;
    if (val == 0)       /* PIO 3 (TX), 4 (RX) */
    {
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
	if ((*reg & 0x00000018) == 0)
	{
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
	    if ((*reg & 0x00000018) == 0x00000018)
	    {
	        have_uart[1] = num;
	        num++;
	    }
	}
    }
    else if (val == 1)  /* PIO 71 (RX), 72 (TX) */
    {
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(2);
	if ((*reg & 0x00000018) == 0)
	{
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(2);
	    if ((*reg & 0x00000018) == 0x00000018)
	    {
	        have_uart[1] = num;
		num++;
	    }
	}
    }
    else if (val == 2)  /* PIO 11 (RX), 73 (TX) */
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
		    {
	                have_uart[1] = num;
			num++;
		    }
	        }
	    }
	}
    }
#endif
#ifdef CONFIG_SERIAL_CX2450X_UART3_ENABLE
    /* check for UART 3 (PIO 14 (TX), PIO 15 (RX)) */
    have_uart[2] = 0xFF;
    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
    if ((*reg & 0x0000C000) == 0)
    {
	reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
    	if ((*reg & 0x0000C000) == 0x0000C000)
    	    have_uart[2] = num;
    }

    /* finally register the detected ports */
    for (val = 0; val < 3; val++)
    {
	if (have_uart[val] < 3)
	{
	    cnxt_ports[have_uart[val]].port.membase = (void *) IO_ADDRESS(UART_FIFO_BRDL_REG(val));
	    cnxt_ports[have_uart[val]].port.mapbase = UART_FIFO_BRDL_REG(val);
	    cnxt_ports[have_uart[val]].port.irq     = irqtab[val];
	    cnxt_ports[have_uart[val]].port.line    = have_uart[val];
	}
    }
#endif
#endif /* CONFIG_SERIAL_CX2450X_BOOTMSG */

    register_console(&cnxt_console);
    return 0;
}

/*******************************************************************************/

console_initcall(cnxtuart_console_init);
#define CNXT_CONSOLE	&cnxt_console
#else
/* no console support enabled */
#define CNXT_CONSOLE	NULL
#endif	/* CONFIG_SERIAL_CX2450X_CONSOLE */

/*******************************************************************************/

static struct uart_driver cnxt_reg = 
{
    .owner	 = THIS_MODULE,
    .driver_name = "CX2450X UART",
    .dev_name	 = "ttyRI",
    .major	 = 204,
    .minor	 = 16,
    .nr		 = 3,		/* max count of supported ports */
    .cons	 = CNXT_CONSOLE,
    .state	 = NULL,
    .tty_driver	 = NULL,
};

/*******************************************************************************/

static int __init cnxtuart_init(void)
{
    int ret;
    u32 val;
    #ifndef CONFIG_SERIAL_CX2450X_BOOTMSG
    u32 num = 0;
    u32 irqtab[3] = {IRQ_UART1, IRQ_UART2, IRQ_UART3};
    volatile u32 *reg;
    #endif

    ret = uart_register_driver(&cnxt_reg);

    if (ret == 0)
    {
	#ifndef CONFIG_SERIAL_CX2450X_BOOTMSG

#ifdef CONFIG_SERIAL_CX2450X_UART1_ENABLE
	/* if early boot messages are enabled, the detection is done in the console 
	   init. In that case the kernel calls that function before. */

	/* check for available UART's (enabled by the Bootloader/BIOS) */

	/* UART 1 (PIO 1 (TX), PIO 2 (RX)) */
	have_uart[0] = 0xFF;
	reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
	if ((*reg & 0x00000006) == 0)
	{
    	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
    	    if ((*reg & 0x00000006) == 0x00000006)
	    {
        	have_uart[0] = num;
		num++;
	    }
	}
#endif
#ifdef CONFIG_SERIAL_CX2450X_UART2_ENABLE							    
	/* check for UART 2 (can be configured on various pins) */
	have_uart[1] = 0xFF;
	reg = (volatile u32*) SREG_ALT_PIN_FUNC_REG;
	val = (*reg >> 4) & 0x03;
	if (val == 0)       /* PIO 3 (TX), 4 (RX) */
	{
	    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
	    if ((*reg & 0x00000018) == 0)
	    {
	        reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
	        if ((*reg & 0x00000018) == 0x00000018)
		{
	    	    have_uart[1] = num;
		    num++;
		}
	    }
	}
	else if (val == 1)  /* PIO 71 (RX), 72 (TX) */
        {
	    reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(2);
	    if ((*reg & 0x00000018) == 0)
	    {
	        reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(2);
	        if ((*reg & 0x00000018) == 0x00000018)
		{
	            have_uart[1] = num;
		    num++;
		}
	    }
	}
	else if (val == 2)  /* PIO 11 (RX), 73 (TX) */
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
			{
	                    have_uart[1] = num;
			    num++;
			}
	            }
		}
	    }
	}
#endif
#ifdef CONFIG_SERIAL_CX2450X_UART3_ENABLE
	/* check for UART 3 (PIO 14 (TX), PIO 15 (RX)) */
        have_uart[2] = 0xFF;
        reg = (volatile u32*) SREG_SEC_MUX_REG_BASE(0);
	if ((*reg & 0x0000C000) == 0)
        {
	    reg = (volatile u32*) SREG_PRI_MUX_REG_BASE(0);
    	    if ((*reg & 0x0000C000) == 0x0000C000)
        	have_uart[2] = num;
        }
#endif
	#endif /* CONFIG_SERIAL_CX2450X_BOOTMSG */

	/* finally register the detected ports */
	for (val = 0; val < 3; val++)
	{
	    if (have_uart[val] < 3)
	    {
		#ifndef CONFIG_SERIAL_CX2450X_BOOTMSG
	        cnxt_ports[have_uart[val]].port.membase = (void *) IO_ADDRESS(UART_FIFO_BRDL_REG(val));
	        cnxt_ports[have_uart[val]].port.mapbase = UART_FIFO_BRDL_REG(val);
	        cnxt_ports[have_uart[val]].port.irq     = irqtab[val];
	        cnxt_ports[have_uart[val]].port.line    = have_uart[val];
		#endif /* CONFIG_SERIAL_CX2450X_BOOTMSG */
    	        uart_add_one_port(&cnxt_reg, &cnxt_ports[have_uart[val]].port);
	    }
	}
    }

    return ret;
}

/*******************************************************************************/

static void __exit cnxtuart_exit(void)
{
    if (have_uart[0] != 0xFF)
    {
	uart_remove_one_port(&cnxt_reg, &cnxt_ports[have_uart[0]].port);
	have_uart[0] = 0xFF;
    }

    if (have_uart[1] != 0xFF)
    {
	uart_remove_one_port(&cnxt_reg, &cnxt_ports[have_uart[1]].port);
	have_uart[1] = 0xFF;
    }

    if (have_uart[2] != 0xFF)
    {
	uart_remove_one_port(&cnxt_reg, &cnxt_ports[have_uart[2]].port);
	have_uart[2] = 0xFF;
    }

    uart_unregister_driver(&cnxt_reg);
}

/*******************************************************************************/

module_init(cnxtuart_init);
module_exit(cnxtuart_exit);

MODULE_AUTHOR("Conexant Systems, Inc. & Coolstream International Ltd.");
MODULE_DESCRIPTION("Conexant CX2450x UART driver");
MODULE_LICENSE("GPL");
