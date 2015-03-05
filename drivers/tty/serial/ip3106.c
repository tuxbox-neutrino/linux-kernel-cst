/*
 * Copyright (C) 2013, Entropic Communications. All Rights Reserved
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


#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/slab.h>

#include <linux/types.h>

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>

#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/soc.h>
#include <asm/hardware/ip3106.h>

#ifdef CONFIG_UART_NX_DMAC_1902
#include <linux/dmaengine.h>
#include <linux/scatterlist.h>
#include <linux/nx_dmac_1902.h>
#include <mach/nx_uart_dma.h>
#endif

#define PORT_UARTIP3106		(0x13)

#define UART_DUMMY_RSR_RX	(0)
#define UART_PORT_SIZE		(4096)

#ifdef CONFIG_EXECUTE_ON_EMULATOR
#define BASE_BAUD		(9600)
#else
#define BASE_BAUD		(115200)
#endif

#define UART_NR			4

/*---------------------------------------------------------------------------
 * Access macros for the UART IP_3106
 *---------------------------------------------------------------------------*/
#define UART_GET_LSR(p)		__raw_readl((p)->membase + IP3106_UART_LSR_REG)
#define UART_GET_MCR(p) 	__raw_readl((p)->membase + IP3106_UART_MCR_REG)
#define UART_SET_MCR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_MCR_REG)
#define UART_GET_MSR(p)		__raw_readl((p)->membase + IP3106_UART_MSR_REG)
#define UART_SET_IER(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_IER_REG)
#define UART_GET_IER(p)		__raw_readl((p)->membase + IP3106_UART_IER_REG)
#define UART_GET_LCR(p)		__raw_readl((p)->membase + IP3106_UART_LCR_REG)
#define UART_SET_LCR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_LCR_REG)
#define UART_GET_CFG(p)		__raw_readl((p)->membase + IP3106_UART_CFG_REG)
#define UART_SET_DLL(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_DLL_REG)
#define UART_SET_DLM(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_DLM_REG)
#define UART_GET_DLL(p)		__raw_readl((p)->membase + IP3106_UART_DLL_REG)
#define UART_GET_DLM(p)		__raw_readl((p)->membase + IP3106_UART_DLM_REG)
#define UART_SET_FCR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_FCR_REG)
#define UART_GET_RBR(p)		__raw_readl((p)->membase + IP3106_UART_RBR_REG)
#define UART_PUT_THR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_THR_REG)
#define UART_GET_IIR(p)		__raw_readl((p)->membase + IP3106_UART_IIR_REG)
#define UART_GET_MOD(p)		__raw_readl((p)->membase + IP3106_UART_MID_REG)
#define UART_GET_CFG(p)		__raw_readl((p)->membase + IP3106_UART_CFG_REG)
#define UART_GET_OSR(p)		__raw_readl((p)->membase + IP3106_UART_OSR_REG)
#define UART_SET_OSR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_OSR_REG)
#define UART_SET_MODE(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_MODE_REG)
#define UART_SET_FDR(p,c)	__raw_writel((c),(p)->membase + IP3106_UART_FDR_REG)

/*--------------------------------------------------------------------------*
 * UART IP_3106 port structure                                              *
 *--------------------------------------------------------------------------*/
struct ip3106_port
{
	struct uart_port  				port;
	unsigned int      				old_status;
	int               				flowctrl;
	int               				autortscts;
#ifdef CONFIG_UART_NX_DMAC_1902
	int  dma_enabled;
  	struct dma_chan					*chan_tx;
	struct dma_chan					*chan_rx;
	struct device					*dma_dev;
	struct dma_async_tx_descriptor	*desc_tx;
	struct dma_async_tx_descriptor	*desc_rx[2];
	dma_cookie_t					cookie_tx;
	dma_cookie_t					cookie_rx[2];
	dma_cookie_t					active_rx;
	struct scatterlist				sg_tx;
	unsigned int					sg_len_tx;
	struct scatterlist				sg_rx[2];
	size_t							buf_len_rx;
	struct work_struct				work_tx;
	struct work_struct				work_rx;
	struct timer_list		   		rx_timer;
   	struct nx_dmac_1902_slave* 		client_tx;
	struct nx_dmac_1902_slave* 		client_rx;
	__u8 chantx_alloted;
	__u8 chanrx_alloted;
   u8 txchan_num;
   u8 rxchan_num;
#endif
};

#ifdef CONFIG_UART_NX_DMAC_1902
struct ip3106_dmae_regs {
	u32 sar; /* SAR / source address */
	u32 dar; /* DAR / destination address */
	u32 tcr; /* TCR / transfer count */
};

struct ip3106_desc {
	struct ip3106_dmae_regs hw;
	struct list_head node;
	struct dma_async_tx_descriptor async_tx;
	enum dma_data_direction direction;
	dma_cookie_t cookie;
	size_t partial;
	int chunks;
	int mark;
};
#endif

/* Added for serial modem notification to edwards layer */
/* Driver notification events*/
typedef enum
{
   IP3106_TERM = 0,
   IP3106_TIMEOUT,
   IP3106_READ_COMPLETE,
   IP3106_WRITE_COMPLETE,
   IP3106_RXCHAR,
   IP3106_TXEMPTY,
   IP3106_RXOVERFLOW,
   IP3106_CTS,
   IP3106_DSR,
   IP3106_RLSD,
   IP3106_RING,
   IP3106_DDCD,
   IP3106_PARITY_ERROR,
   IP3106_FRAMING_ERROR,
   IP3106_RX_BREAK,
   IP3106_LAST = IP3106_RX_BREAK
} IP3106_EVENT;


typedef void (*pfnnotify)(unsigned int, bool*, bool*, bool*, unsigned int);

pfnnotify modem_notify = NULL;
EXPORT_SYMBOL(modem_notify);

int ip3106_set_modem_notification(pfnnotify pfn)
{
	if (!pfn)
		return -1;
	if (pfn)
		modem_notify = pfn;
	return 0;
}
EXPORT_SYMBOL(ip3106_set_modem_notification);

#ifdef CONFIG_IP3106_UART0
static struct ip3106_port ip3106_ports0;
#endif
static struct ip3106_port ip3106_ports1;
static struct ip3106_port ip3106_ports2;
#ifdef CONFIG_ARCH_APOLLO
static struct ip3106_port ip3106_ports3;
#endif

#ifndef CONFIG_IP3106_UART0
static int ip3106_line = 1;
#else
static int ip3106_line = 0;
#endif

/*--------------------------------------------------------------------------*
 * Control functions:                                                       *
 *--------------------------------------------------------------------------*/

static unsigned int ip3106_tx_empty (struct uart_port *port)
{
	unsigned int status = 0;

	/* Get the UART status */
	if (UART_GET_LSR(port) & IP3106_UART_LSR_TEMT_MSK)
		status = TIOCSER_TEMT;
	else
		status = 0;

	return (status);
}

static void ip3106_set_modem_ctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned int mcr = 0;
	unsigned int ctrls = 0;
	unsigned int ctrlc = 0;

	if (mctrl & TIOCM_RTS)
		ctrls |= IP3106_UART_MCR_RTS_MSK;
	else
		ctrlc |= IP3106_UART_MCR_RTS_MSK;

	if (mctrl & TIOCM_DTR)
		ctrls |= IP3106_UART_MCR_DTR_MSK;
	else
		ctrlc |= IP3106_UART_MCR_DTR_MSK;

	if (mctrl & (TIOCM_OUT1 | TIOCM_OUT2))
		ctrls |= IP3106_UART_MCR_OUT_MSK;
	else
		ctrlc |= IP3106_UART_MCR_OUT_MSK;

	/* Get the modem control */
	mcr = UART_GET_MCR(port);
	mcr |= ctrls;
	mcr &= ~ctrlc;

	/* Set the new modem control value */
	UART_SET_MCR(port, mcr);
}

static unsigned int ip3106_get_modem_ctrl(struct uart_port *port)
{
	unsigned int msr = 0;
	unsigned int status = 0;

	/* Get the modem status */
	msr = UART_GET_MSR(port);

	if (msr & IP3106_UART_MSR_DCD_MSK)
		status |= TIOCM_CAR;

	if (msr & IP3106_UART_MSR_DSR_MSK)
		status |= TIOCM_DSR;

	if (msr & IP3106_UART_MSR_CTS_MSK)
		status |= TIOCM_CTS;

	if (msr & IP3106_UART_MSR_RI_MSK)
		status |= TIOCM_RI;

	return status;
}

#ifdef CONFIG_UART_NX_DMAC_1902
void ip3106_dma_tx_complete(void *arg)
{
	struct ip3106_port *s = (struct ip3106_port *)arg;
	struct uart_port *port = &s->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);

	spin_lock_irqsave(&port->lock, flags);

	xmit->tail += s->sg_tx.length;
	xmit->tail &= UART_XMIT_SIZE - 1;

	port->icount.tx += s->sg_tx.length;

	async_tx_ack(s->desc_tx);
	s->cookie_tx = -EINVAL;
	s->desc_tx = NULL;

	spin_unlock_irqrestore(&port->lock, flags);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_chars_pending(xmit))
		schedule_work(&s->work_tx);
}

/* Locking: called with port lock held */
int ip3106_dma_rx_push(struct ip3106_port *s, struct tty_struct *tty,
			   size_t count)
{
	struct uart_port *port = &s->port;
	int i, active, room;

	room = tty_buffer_request_room(tty, count);

	if (s->active_rx == s->cookie_rx[0]) {
		active = 0;
	} else if (s->active_rx == s->cookie_rx[1]) {
		active = 1;
	} else {
		dev_err(port->dev, "cookie %d not found!\n", s->active_rx);
		return 0;
	}

	if (room < count)
		dev_warn(port->dev, "Rx overrun: dropping %u bytes\n",
			 count - room);
	if (!room)
		return room;

	struct scatterlist *sg = &s->sg_rx[0];
	for (i = 0; i < room; i++) {
		tty_insert_flip_char(tty, ((u8 *)(sg->page_link + sg->offset))[i],
				     TTY_NORMAL);
	}
	port->icount.rx += room;

	return room;
}

void ip3106_dma_rx_complete(void *arg)
{
	struct ip3106_port *s = (struct ip3106_port *)arg;
	struct uart_port *port = &s->port;
	struct tty_struct *tty = port->state->port.tty;
	unsigned long flags;
	int count;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);
	spin_lock_irqsave(&port->lock, flags);

	count = ip3106_dma_rx_push(s, tty, s->buf_len_rx);

	mod_timer(&s->rx_timer, jiffies + msecs_to_jiffies(4));

	spin_unlock_irqrestore(&port->lock, flags);

	if (count)
		tty_flip_buffer_push(&port->state->port);

	schedule_work(&s->work_rx);
}


static void ip3106_start_tx(struct uart_port *port);
void ip3106_start_rx(struct uart_port *port);

void ip3106_rx_dma_release(struct ip3106_port *s, bool enable_pio)
{
	struct dma_chan *chan = s->chan_rx;
	struct uart_port *port = &s->port;
	struct scatterlist *sg = &s->sg_rx[0];

	s->chan_rx = NULL;
	s->cookie_rx[0] = s->cookie_rx[1] = -EINVAL;
	dma_sync_wait(chan,0);
   	chan->device->device_terminate_all(chan);
   dma_free_coherent(port->dev, s->buf_len_rx * 2, sg->page_link + sg->offset, sg->dma_address);
	if (enable_pio)
		ip3106_start_rx(port);
}

void ip3106_tx_dma_release(struct ip3106_port *s, bool enable_pio)
{
	struct dma_chan *chan = s->chan_tx;
	struct uart_port *port = &s->port;

	s->chan_tx = NULL;
	s->cookie_tx = -EINVAL;
	dma_sync_wait(chan,0);
   	chan->device->device_terminate_all(chan);
	dma_release_channel(chan);
	if (enable_pio)
		ip3106_start_tx(port);
}

void ip3106_dma_submit_rx(struct ip3106_port *s)
{
	struct dma_chan *chan = s->chan_rx;
	int i;

	for (i = 0; i < 2; i++) 
	{
		struct scatterlist *sg = &s->sg_rx[i];
		struct dma_async_tx_descriptor *desc;

		desc = chan->device->device_prep_slave_sg(chan,
			sg, 1, DMA_FROM_DEVICE, DMA_PREP_INTERRUPT);

		if (desc) 
		{
			s->desc_rx[i] = desc;
			desc->callback = ip3106_dma_rx_complete;
			desc->callback_param = s;
			s->cookie_rx[i] = desc->tx_submit(desc);
		}

		if (!desc || s->cookie_rx[i] < 0) 
		{
			if (i) {
				async_tx_ack(s->desc_rx[0]);
				s->cookie_rx[0] = -EINVAL;
			}
			if (desc) {
				async_tx_ack(desc);
				s->cookie_rx[i] = -EINVAL;
			}
			dev_warn(s->port.dev,
				 "failed to re-start DMA, using PIO\n");
			ip3106_rx_dma_release(s, true);
			return;
		}
	}
	s->active_rx = s->cookie_rx[0];
	dma_async_issue_pending(chan);
}

void work_fn_rx(struct work_struct *work)
{
	struct ip3106_port *s = container_of(work, struct ip3106_port, work_rx);
	struct uart_port *port = &s->port;
	struct dma_async_tx_descriptor *desc;
	int new;

	if (s->active_rx == s->cookie_rx[0]) 
	{
		new = 0;
	}
	else if (s->active_rx == s->cookie_rx[1])
	{
		new = 1;
	}
	else
	{
		dev_err(port->dev, "cookie %d not found!\n", s->active_rx);
		return;
	}
	desc = s->desc_rx[new];

	if (dma_async_is_tx_complete(s->chan_rx, s->active_rx, NULL, NULL) != DMA_SUCCESS)
	{
		/* Handle incomplete DMA receive */
		struct tty_struct *tty = port->state->port.tty;
		struct dma_chan *chan = s->chan_rx;
		struct ip3106_desc *ip3106_desc = container_of(desc, struct ip3106_desc,
						       async_tx);
		unsigned long flags;
		int count;

		chan->device->device_terminate_all(chan);
		dev_dbg(port->dev, "Read %u bytes with cookie %d\n",
			ip3106_desc->partial, ip3106_desc->cookie);

		spin_lock_irqsave(&port->lock, flags);
		count = ip3106_dma_rx_push(s, tty, ip3106_desc->partial);
		spin_unlock_irqrestore(&port->lock, flags);

		if (count)
			tty_flip_buffer_push(&port->state->port);
	   	ip3106_dma_submit_rx(s);
		return;
	}
	s->cookie_rx[new] = desc->tx_submit(desc);
	if (s->cookie_rx[new] < 0) 
	{
		dev_warn(port->dev, "Failed submitting Rx DMA descriptor\n");
		ip3106_rx_dma_release(s, true);
		return;
	}

	dev_dbg(port->dev, "%s: cookie %d #%d\n", __func__,
		s->cookie_rx[new], new);

	s->active_rx = s->cookie_rx[!new];
}

void work_fn_tx(struct work_struct *work)
{
	struct ip3106_port *s = container_of(work, struct ip3106_port, work_tx);
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *chan = s->chan_tx;
	struct uart_port *port = &s->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct scatterlist *sg = &s->sg_tx;
	unsigned long flags;

	/*
	 * DMA is idle now.
	 * Port xmit buffer is already mapped, and it is one page... Just adjust
	 * offsets and lengths. Since it is a circular buffer, we have to
	 * transmit till the end, and then the rest. Take the port lock to get a
	 * consistent xmit buffer state.
	 */
	spin_lock_irqsave(&port->lock, flags);

	sg->offset = xmit->tail & (UART_XMIT_SIZE - 1);
	sg->length = min((int)CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE),	
                         CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));
	sg->dma_length = sg->length;
	spin_unlock_irqrestore(&port->lock, flags);

	BUG_ON(!sg->length);

	desc = chan->device->device_prep_slave_sg(chan,	sg, s->sg_len_tx, DMA_TO_DEVICE, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {

		/* switch to PIO */
		ip3106_tx_dma_release(s, true);
		return;
	}

	spin_lock_irqsave(&port->lock, flags);
	s->desc_tx = desc;
	desc->callback = ip3106_dma_tx_complete;
	desc->callback_param = s;
	spin_unlock_irqrestore(&port->lock, flags);
	s->cookie_tx = desc->tx_submit(desc);
	if (s->cookie_tx < 0) 
	{
		dev_warn(port->dev, "Failed submitting Tx DMA descriptor\n");
		/* switch to PIO */
		ip3106_tx_dma_release(s, true);
		return;
	}

	dev_dbg(port->dev, "%s: %p: %d...%d, cookie %d\n", __func__,
		xmit->buf, xmit->tail, xmit->head, s->cookie_tx);
	dma_async_issue_pending(chan);
}
#endif



static void ip3106_stop_tx(struct uart_port *port)
{
	unsigned int ier = 0;
	unsigned int lcr = 0;

	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Disable the Tx channel interrupt */
	ier = UART_GET_IER(port);
	ier &= ~IP3106_UART_IER_THREI_E_MSK;
	UART_SET_IER(port, ier);
}

static void ip3106_start_tx(struct uart_port *port)
{
	unsigned int ier = 0;
	unsigned int lcr = 0;
	unsigned char ch;
	unsigned int status = 0;
	struct circ_buf *xmit = &port->state->xmit;

#ifdef CONFIG_UART_NX_DMAC_1902
	struct ip3106_port *s = (struct ip3106_port *)port;
	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

    if (s->dma_enabled)
    {
		if (s->chan_tx) 
	    {
			if (!uart_circ_empty(&s->port.state->xmit) && s->cookie_tx < 0)
				schedule_work(&s->work_tx);
	    }
	   	return;
	}
#endif
	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Start the transmission by transmitting first character */
	if (port->x_char) {
		ch = port->x_char;
		port->icount.tx++;
		port->x_char = 0;
	} else {
		ch = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	/* Wait till the Tx FIFO is empty */
	do {
		status = UART_GET_LSR(port);
	} while (!(status & IP3106_UART_LSR_THRE_MSK));

	/* Write the character */
	UART_PUT_THR(port,ch);

	/* Enable the Tx channel interrupt */
	ier = UART_GET_IER(port);
	ier |= IP3106_UART_IER_THREI_E_MSK;
	UART_SET_IER(port, ier);
}

static void ip3106_stop_rx(struct uart_port *port)
{
	unsigned int ier = 0;
	unsigned int lcr = 0;

	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Diasble the Rx channel interrupt */
	ier = UART_GET_IER(port);
	ier &= ~(IP3106_UART_IER_RDAI_E_MSK | IP3106_UART_IER_RLSI_E_MSK);
	UART_SET_IER(port, ier);
}

static void ip3106_enable_modem_status(struct uart_port *port)
{
	unsigned int      ier = 0;
	unsigned int      lcr = 0;
	struct ip3106_port *uport = NULL;

	/* Get the UART IP_3106 port*/
	uport = (struct ip3106_port *) port;
	if (!uport)
		return;

	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Enable the modem status interrupt */
	ier = UART_GET_IER(port);
	ier |= (IP3106_UART_IER_MSI_E_MSK);

	/* If auto CTS is present, enable the auto CTS interrupt also */
	if (uport->autortscts == 1)
		ier |= IP3106_UART_IER_CTSI_E_MSK;

	UART_SET_IER(port, ier);
}

static void ip3106_break_ctrl(struct uart_port *port, int break_state)
{
	unsigned long flags = 0;
	unsigned int lcr = 0;

	spin_lock_irqsave(&port->lock, flags);
	lcr = UART_GET_LCR(port);
	if (break_state == -1)
		lcr |= IP3106_UART_LCR_BRK_CTL_MSK;
	else
		lcr &= ~IP3106_UART_LCR_BRK_CTL_MSK;
	UART_SET_LCR(port, lcr);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void ip3106_rx_chars(struct uart_port *port)
{
	unsigned int      lcr = 0, status = 0, ch = 0, rsr = 0;
	unsigned int      max_count = 256;
	unsigned int	  flag = 0;

	/* To access RBR,  set DLAB bit in LCR to 0 */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Read the status of the Rx FIFO */
	status = UART_GET_LSR(port);
	/* Read till the FIFO is empty or max_count is 0 */
        /*
         * Note: For some reason that isn't clear, we sometimes get an
         * interrupt where IIR claims there's data in the RX FIFO,
         * but the FIFO has no data. To work around that, this
         * is a do-while loop instead of a while loop. That way
         * it always reads a char, even if the FIFO is empty.
         */
	do {
		/* Read the character from Rx FIFO */
		ch = UART_GET_RBR(port);
		flag = TTY_NORMAL;
		port->icount.rx++;

		/* Note that the error handling code is
		 *  out of the main execution path
		 */
		rsr = (UART_GET_LSR(port) | UART_DUMMY_RSR_RX);
		if (rsr & IP3106_UART_LSR_REG_MSK) {
			if (rsr & IP3106_UART_LSR_BI_MSK) {
				rsr &= ~(IP3106_UART_LSR_FE_MSK |
					 IP3106_UART_LSR_PE_MSK);
				port->icount.brk++;
				if (uart_handle_break(port))
					goto ignore_char;
			} else if (rsr & IP3106_UART_LSR_PE_MSK) {
				port->icount.parity++;
			} else if (rsr & IP3106_UART_LSR_FE_MSK) {
				port->icount.frame++;
			}

			if (rsr & IP3106_UART_LSR_OE_MSK) {
				port->icount.overrun++;
			}

			rsr &= port->read_status_mask;
			if (rsr & IP3106_UART_LSR_BI_MSK) {
				flag = TTY_BREAK;
			} else if (rsr & IP3106_UART_LSR_PE_MSK) {
				flag = TTY_PARITY;
			} else if (rsr & IP3106_UART_LSR_FE_MSK) {
				flag = TTY_FRAME;
			}
		}

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		uart_insert_char(port, rsr, IP3106_UART_LSR_OE_MSK, ch, flag);

		if ((rsr & port->ignore_status_mask) == 0)
			tty_flip_buffer_push(&port->state->port);

ignore_char:
		status = UART_GET_LSR(port);
	} while ( ((status & IP3106_UART_LSR_DR_MSK) != 0) && max_count--);
	tty_flip_buffer_push(&port->state->port);
	
	if (modem_notify) 
	{
		modem_notify (IP3106_RXCHAR, 
					NULL, 
					NULL, 
					NULL, 
					port->line);
	}
}

static void ip3106_tx_chars(struct uart_port *port)
{
	int             count = 0;
	unsigned int    lcr = 0;
	struct circ_buf *xmit = &port->state->xmit;

	/* To access THR,  set DLAB bit in LCR to 0 */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	if (port->x_char) {
		UART_PUT_THR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		ip3106_stop_tx(port);
		return;
	}

	count = port->fifosize >> 1;
	do {
		UART_PUT_THR(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		ip3106_stop_tx(port);

	if (modem_notify) 
	{
		modem_notify (IP3106_TXEMPTY, 
					NULL, 
					NULL, 
					NULL, 
					port->line);
	}	
}

#ifdef CONFIG_IP3106_UART0
static void ip3106_modem_status_mdm(struct uart_port *port)
{
	unsigned int status = 0, delta = 0;
	struct ip3106_port *uap = NULL;
	struct uart_state *info = NULL;
	bool cts = false;
	bool dsr = false;
	bool dcd = false;
	IP3106_EVENT event;

	uap = (struct ip3106_port *) port;
	if( !uap )
		return;

	/* Added for event notification */
	status = UART_GET_MSR(port);

	if (status & IP3106_UART_MSR_CTS_MSK)
	{
		cts = true;
		event = IP3106_CTS;
	}
	if (status & IP3106_UART_MSR_DDSR_MSK)
	{
		dsr = true;
		event = IP3106_DSR;
	}
	if (status & IP3106_UART_MSR_TERI_MSK)
	{
		event = IP3106_RING;
	}
	if (status & IP3106_UART_MSR_DDCD_MSK) 
	{
		dcd = true;
		event = IP3106_DDCD;
	}

	if (modem_notify)
	{
		modem_notify((unsigned int)event, 
					&cts, 
					&dsr, 
					&dcd, 
					port->line);
	}

	status &= (IP3106_UART_MSR_DCD_MSK |
	                IP3106_UART_MSR_DSR_MSK |
	                IP3106_UART_MSR_CTS_MSK );
	delta = status ^ uap->old_status;
	uap->old_status = status;
	//printk ("uap->old_statue = 0x%x, status=0x%x", uap->old_status, status);
	if (!delta)
		return;

	if (delta & IP3106_UART_MSR_DCD_MSK) {
		info = port->state;

		if(status & IP3106_UART_MSR_DCD_MSK)
		{
			port->icount.dcd++;
			wake_up_interruptible(&info->port.open_wait);
			dcd = true;		
		}
		else
			dcd = false;		
		if (modem_notify)
		{
			modem_notify(IP3106_DDCD, 
						NULL, 
						NULL, 
						&dcd, 
						port->line);
		}
	}
	if (delta & IP3106_UART_MSR_DSR_MSK) {
		uap->port.icount.dsr++;

		if(status & IP3106_UART_MSR_DCD_MSK)
			dsr = true;
		else
			dsr = false;
		if (modem_notify)
		{
			modem_notify(IP3106_DSR, 
						NULL, 
						&dsr, 
						NULL, 
						port->line);
		}
	}
	if (delta & IP3106_UART_MSR_CTS_MSK) {
		uart_handle_cts_change(&uap->port,
			status & IP3106_UART_MSR_CTS_MSK);
		
		if (status & IP3106_UART_MSR_CTS_MSK)
			cts = true;
		else
			cts = false;
		if (modem_notify)
		{
			modem_notify(IP3106_CTS, 
						&cts, 
						NULL, 
						NULL, 
						port->line);
		}
	}
	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
}


static void ip3106_modem_status(struct uart_port *port)
{
	unsigned int status = 0, delta = 0;
	struct ip3106_port *uap = NULL;
	bool cts = false;
	bool dsr = false;
	bool dcd = false;
	IP3106_EVENT event;

	uap = (struct ip3106_port *) port;
	if( !uap )
		return;

	/* Added for event notification */
	status = UART_GET_MSR(port);

	if (status & IP3106_UART_MSR_CTS_MSK)
	{
		cts = true;
		event = IP3106_CTS;
	}
	if (status & IP3106_UART_MSR_DDSR_MSK)
	{
		dsr = true;
		event = IP3106_DSR;
	}
	if (status & IP3106_UART_MSR_TERI_MSK)
	{
		event = IP3106_RING;
	}
	if (status & IP3106_UART_MSR_DDCD_MSK) 
	{
		dcd = true;
		event = IP3106_DDCD;
	}

	if (modem_notify)
	{
		modem_notify((unsigned int)event, 
					&cts, 
					&dsr, 
					&dcd, 
					port->line);
	}

	status &= (IP3106_UART_MSR_DCD_MSK |
	                IP3106_UART_MSR_DSR_MSK |
	                IP3106_UART_MSR_CTS_MSK );
	delta = status ^ uap->old_status;
	uap->old_status = status;
	if (!delta)
		return;

	if (delta & IP3106_UART_MSR_DCD_MSK) {
		uart_handle_dcd_change(&uap->port,
			status & IP3106_UART_MSR_DCD_MSK);

		if(status & IP3106_UART_MSR_DCD_MSK)
			dcd = true;		
		else
			dcd = false;		
		if (modem_notify)
		{
			modem_notify(IP3106_DDCD, 
						NULL, 
						NULL, 
						&dcd, 
						port->line);
		}
	}
	if (delta & IP3106_UART_MSR_DSR_MSK) {
		uap->port.icount.dsr++;

		if(status & IP3106_UART_MSR_DCD_MSK)
			dsr = true;
		else
			dsr = false;
		if (modem_notify)
		{
			modem_notify(IP3106_DSR, 
						NULL, 
						&dsr, 
						NULL, 
						port->line);
		}
	}
	if (delta & IP3106_UART_MSR_CTS_MSK) {
		uart_handle_cts_change(&uap->port,
			status & IP3106_UART_MSR_CTS_MSK);
		
		if (status & IP3106_UART_MSR_CTS_MSK)
			cts = true;
		else
			cts = false;
		if (modem_notify)
		{
			modem_notify(IP3106_CTS, 
						&cts, 
						NULL, 
						NULL, 
						port->line);
		}
	}
	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
}
#endif

static irqreturn_t ip3106_int(int irq, void *dev_id)
{
	struct ip3106_port *uap = (struct ip3106_port *) dev_id;
	struct uart_port *port = (struct uart_port *) &(uap->port);
	unsigned int 	 irq_status = 0;
	unsigned int     status = 0;
	unsigned int	 mcr = 0;
	int              handled = 0;
#ifdef CONFIG_UART_NX_DMAC_1902
	int rsr = 0;
	unsigned long tout;
	unsigned int	   ier = 0;
	unsigned long     flag = 0;

  	if (1 == uap->dma_enabled) 
	{
		struct tty_struct *tty = port->state->port.tty;
		spin_lock(&uap->port.lock);
		status = UART_GET_IIR(port);
		if (!(status & IP3106_UART_IIR_PENDING_MSK)) 
		{
			irq_status = (status & IP3106_UART_IIR_INT_MASK) >> 1;

			if ((irq_status  == IP3106_UART_IIR_RLS_INT_ID) &&
				(UART_GET_IER(port) & IP3106_UART_IER_RLSI_E_MSK)) 
			{
				rsr = UART_GET_LSR(port);
				if (rsr & IP3106_UART_LSR_REG_MSK) 
				{
					if (rsr & IP3106_UART_LSR_BI_MSK) 
					{
						rsr &= ~(IP3106_UART_LSR_FE_MSK | IP3106_UART_LSR_PE_MSK);
						port->icount.brk++;
						if (uart_handle_break(port))
						goto ignore_char;
					}
					else if (rsr & IP3106_UART_LSR_PE_MSK) 
					{
						port->icount.parity++;
					}
					else if (rsr & IP3106_UART_LSR_FE_MSK)
					{
						port->icount.frame++;
					}

					if (rsr & IP3106_UART_LSR_OE_MSK) 
					{
						port->icount.overrun++;
					}

					rsr &= port->read_status_mask;
					if (rsr & IP3106_UART_LSR_BI_MSK)
			 		{
						flag = TTY_BREAK;
					}
					else if (rsr & IP3106_UART_LSR_PE_MSK) 
					{
						flag = TTY_PARITY;
					}
					else if (rsr & IP3106_UART_LSR_FE_MSK) 
					{
						flag = TTY_FRAME;
					}

					if (tty_insert_flip_char(tty, 0, flag))
					{
			             tty_flip_buffer_push(&port->state->port);
					}

				}
			}
			if (((irq_status == IP3106_UART_IIR_CTI_INT_ID) ||
  				 (irq_status == IP3106_UART_IIR_RDA_INT_ID)) &&
				 (UART_GET_IER(port) & IP3106_UART_IER_RDAI_E_MSK))
			{
       			ier = UART_GET_IER(port);
			    ier &= ~(IP3106_UART_IER_RDAI_E_MSK);
	    		UART_SET_IER(port, ier);
		
     			/* Calculate delay for 1.5 DMA buffers */
				tout = (port->timeout - HZ / 50) * uap->buf_len_rx * 3 / port->fifosize / 2;
				dev_dbg(port->dev, "Rx IRQ: setup timeout in %lu ms\n",
				tout * 1000 / HZ);
				if (tout < 2)
					tout = 2;
         		mod_timer(&uap->rx_timer, jiffies + tout);
			}
ignore_char:		
			status = UART_GET_IIR(port);
			handled = 1;
		}
		spin_unlock(&uap->port.lock);
	}
  	else
#endif
  	{
		spin_lock(&uap->port.lock);
		status = UART_GET_IIR(port);
		if (!(status & IP3106_UART_IIR_PENDING_MSK)) {
			irq_status = (status & IP3106_UART_IIR_INT_MASK) >> 1;

			if ((irq_status  == IP3106_UART_IIR_RLS_INT_ID) &&
				(UART_GET_IER(port) & IP3106_UART_IER_RLSI_E_MSK)) {
				UART_GET_LSR(port);
			}
			if (((irq_status == IP3106_UART_IIR_CTI_INT_ID) ||
				(irq_status == IP3106_UART_IIR_RDA_INT_ID)) &&
				(UART_GET_IER(port) & IP3106_UART_IER_RDAI_E_MSK)) {
				if (uap->flowctrl == CRTSCTS) {
					if (uap->autortscts == 0) {
						/* Deassert the RTS */
						mcr = UART_GET_MCR(port);
						mcr &= ~IP3106_UART_MCR_RTS_MSK;
						UART_SET_MCR(port,mcr);
					}
				}
				if(uap->flowctrl == (IXON|IXOFF)) {
					UART_PUT_THR(port,0x13);
				}
				ip3106_rx_chars(port);

				if(uap->flowctrl == CRTSCTS) {
					if(uap->autortscts == 0) {
						/* Assert the RTS */
						mcr = UART_GET_MCR(port);
						mcr |= IP3106_UART_MCR_RTS_MSK;
						UART_SET_MCR(port,mcr);
					}
				}
				if(uap->flowctrl == (IXON|IXOFF)) {
					UART_PUT_THR(port,0x11);
				}
			}
			if ((irq_status == IP3106_UART_IIR_THRE_INT_ID) &&
				(UART_GET_IER(port) & IP3106_UART_IER_THREI_E_MSK)) {
				ip3106_tx_chars(port);
			}
#ifdef CONFIG_IP3106_UART0        
			if ((irq_status == IP3106_UART_IIR_MSI_INT_ID) &&
				(UART_GET_IER(port) & IP3106_UART_IER_MSI_E_MSK )) {
				ip3106_modem_status(port);
			}
#endif        
			status = UART_GET_IIR(port);
			handled = 1;
		}
		spin_unlock(&uap->port.lock);
	}
  
	return IRQ_RETVAL(handled);
}

#ifdef CONFIG_IP3106_UART0
static irqreturn_t ip3106_modem_int(int irq, void *dev_id)
{
	struct ip3106_port *uap = (struct ip3106_port *) dev_id;
	struct uart_port *port = (struct uart_port *) &(uap->port);
	unsigned int 	 irq_status = 0;
	unsigned int     status = 0;
	unsigned int	 mcr = 0;
	int              handled = 0;

	spin_lock(&uap->port.lock);
	status = UART_GET_IIR(port);
	if (!(status & IP3106_UART_IIR_PENDING_MSK)) {
		irq_status = (status & IP3106_UART_IIR_INT_MASK) >> 1;

		if ((irq_status  == IP3106_UART_IIR_RLS_INT_ID) &&
			(UART_GET_IER(port) & IP3106_UART_IER_RLSI_E_MSK)) {
			UART_GET_LSR(port);
		}
		if (((irq_status == IP3106_UART_IIR_CTI_INT_ID) ||
			(irq_status == IP3106_UART_IIR_RDA_INT_ID)) &&
			(UART_GET_IER(port) & IP3106_UART_IER_RDAI_E_MSK)) {
			if (uap->flowctrl == CRTSCTS) {
				if (uap->autortscts == 0) {
					/* Deassert the RTS */
					mcr = UART_GET_MCR(port);
					mcr &= ~IP3106_UART_MCR_RTS_MSK;
					UART_SET_MCR(port,mcr);
				}
			}
			if(uap->flowctrl == (IXON|IXOFF)) {
				UART_PUT_THR(port,0x13);
			}
			ip3106_rx_chars(port);

			if(uap->flowctrl == CRTSCTS) {
				if(uap->autortscts == 0) {
					/* Assert the RTS */
					mcr = UART_GET_MCR(port);
					mcr |= IP3106_UART_MCR_RTS_MSK;
					UART_SET_MCR(port,mcr);
				}
			}
			if(uap->flowctrl == (IXON|IXOFF)) {
				UART_PUT_THR(port,0x11);
			}
		}
		if ((irq_status == IP3106_UART_IIR_THRE_INT_ID) &&
			(UART_GET_IER(port) & IP3106_UART_IER_THREI_E_MSK)) {
			ip3106_tx_chars(port);
		}
		if ((irq_status == IP3106_UART_IIR_MSI_INT_ID) &&
			(UART_GET_IER(port) & IP3106_UART_IER_MSI_E_MSK )) {
			ip3106_modem_status_mdm(port);
		}
		status = UART_GET_IIR(port);
		handled = 1;
	}
	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}
#endif


#ifdef CONFIG_UART_NX_DMAC_1902

void ip3106_start_rx(struct uart_port *port)
{
	unsigned int lcr = 0;
	unsigned int ier  = 0;

	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	ier = UART_GET_IER(port);
	ier |= (IP3106_UART_IER_RDAI_E_MSK | IP3106_UART_IER_RLSI_E_MSK );
	UART_SET_IER(port, ier);
}

bool filter_tx(struct dma_chan *chan, void *slave)
{
	struct ip3106_port *uport = (struct ip3106_port *)slave;

	if( chan->device->dev != uport->client_tx->dma_dev) {
		return false;
	}

	if( chan->chan_id == uport->txchan_num) {
		uport->chan_tx = chan;
		chan->private =  uport->client_tx;
		uport->chantx_alloted = 1;

		return true;
	}
	else {
		return false;
	}
}

bool filter_rx(struct dma_chan *chan, void *slave)
{
	struct ip3106_port *uport = (struct ip3106_port *)slave;
	if( chan->device->dev != uport->client_rx->dma_dev) {
		return false;
	}

	if( chan->chan_id == uport->rxchan_num) {
		uport->chan_rx = chan;
		chan->private =  uport->client_rx;
		uport->chanrx_alloted = 1;

		return true;
	}
	else {
		return false;
	}

}


void rx_timer_fn(unsigned long arg)
{
	struct ip3106_port *s = (struct ip3106_port *)arg;
	struct uart_port *port = &s->port;

    unsigned int lcr = 0;
    unsigned int ier  = 0;
    /* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
    lcr = UART_GET_LCR(port);
    lcr &= ~IP3106_UART_LCR_DLAB_MSK;
    UART_SET_LCR(port, lcr);

    ier = UART_GET_IER(port);
    ier |= (IP3106_UART_IER_RDAI_E_MSK | IP3106_UART_IER_RLSI_E_MSK );  
    UART_SET_IER(port, ier);
	dev_dbg(port->dev, "DMA Rx timed out\n");
}

void ip3106_request_dma(struct uart_port *port)
{
	struct ip3106_port *s = (struct ip3106_port *)port;
	
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int nent;

	dev_dbg(port->dev, "%s: port %d DMA %p\n", __func__, port->line, s->dma_dev);

    dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	s->cookie_tx = -EINVAL;
	chan = dma_request_channel(mask, filter_tx, s); 
	dev_dbg(port->dev, "%s: TX: got channel %p\n", __func__, chan);
	if (chan) {
		s->chan_tx = chan;
		sg_init_table(&s->sg_tx, 1);
		/* UART circular tx buffer is an aligned page. */
		BUG_ON((int)port->state->xmit.buf & ~PAGE_MASK);

		sg_set_page(&s->sg_tx, virt_to_page(port->state->xmit.buf),
			    UART_XMIT_SIZE, (int)port->state->xmit.buf & ~PAGE_MASK);
		nent = dma_map_sg(port->dev, &s->sg_tx, 1, DMA_TO_DEVICE);
		if (!nent)
			ip3106_tx_dma_release(s, false);
		else {
			dev_dbg(port->dev, "%s: mapped %d@%p to %x\n", __func__,
				sg_dma_len(&s->sg_tx),
				port->state->xmit.buf, sg_dma_address(&s->sg_tx));
        }
		s->sg_len_tx = nent;
		INIT_WORK(&s->work_tx, work_fn_tx);
	}		

	chan = dma_request_channel(mask, filter_rx, s);
	dev_dbg(port->dev, "%s: RX: got channel %p\n", __func__, chan);
	if (chan) {
		dma_addr_t dma[2];
		void *buf[2];
		int i;

		s->chan_rx = chan;

      	s->buf_len_rx = (int)port->fifosize;
		buf[0] = dma_alloc_coherent(port->dev, s->buf_len_rx * 2,
					    &dma[0], GFP_KERNEL);

		if (!buf[0]) {
			dev_warn(port->dev,
				 "failed to allocate dma buffer, using PIO\n");
			ip3106_rx_dma_release(s, true);
			return;
		}

		buf[1] = buf[0] + s->buf_len_rx;
		dma[1] = dma[0] + s->buf_len_rx;

		for (i = 0; i < 2; i++) {
			struct scatterlist *sg = &s->sg_rx[i];
			sg->page_link   = buf[i];
			sg->offset	    = (int)buf[i]  & ~PAGE_MASK;
			sg->length      = s->buf_len_rx;
			sg->dma_address = dma[i];
			sg->dma_length  = sg->length;
		}

		INIT_WORK(&s->work_rx, work_fn_rx);
		setup_timer(&s->rx_timer, rx_timer_fn, (unsigned long)s);

		ip3106_dma_submit_rx(s);
	}
}

void ip3106_free_dma(struct uart_port *port)
{
	struct ip3106_port *s = (struct ip3106_port *)port;

	if (!s->dma_enabled)
		return;

	if (s->chan_tx)
		ip3106_tx_dma_release(s, false);
	//if (s->chan_rx)
	//	ip3106_rx_dma_release(s, false);
}
#endif

static int ip3106_startup(struct uart_port *port)
{
	unsigned int       retval = 0;
	unsigned int       divisor = 0;
	__u8               dll = 0;
	__u8		          dlm = 0;
	unsigned int       lcr = 0;
	unsigned int	    fcr = 0;
	unsigned int	    ier = 0;
	unsigned int       baud = BASE_BAUD;
#ifndef CONFIG_EXECUTE_ON_EMULATOR
	unsigned int	    osr = 0;
#endif
	struct ip3106_port *uport = (struct ip3106_port *)port;

	/* Initilize the Line Control Settings
	 * 8- data bits, 1- stop bits, No Parity
	 * DLAB bit in the LineControl Register is set to zero
	 */
	lcr = ((IP3106_UART_LCR_WORDLEN_8BIT << IP3106_UART_LCR_WORD_LEN_POS)|
		(IP3106_UART_LCR_1_STOP_BIT << IP3106_UART_LCR_STOPB_POS)    |
		(IP3106_UART_LCR_NO_PARITY << IP3106_UART_LCR_PAR_EN_POS)    |
		(IP3106_UART_LCR_NO_BREAK_TR << IP3106_UART_LCR_BRK_CTL_POS) |
		(IP3106_UART_LCR_DIV_LAB << IP3106_UART_LCR_DLAB_POS) );
	UART_SET_LCR(port, lcr);

	/* Set the Baud Rate to BASE_BAUD
	 * To access the Baud Rate Register DLAB bit in LineControl = 1
	 * after setting the Baud Rate, RESET the DLAB Bit again to Zero.
	 */
	divisor = ((port->uartclk) + (8 * baud)) / (baud * 16);
        if (divisor == 0)
        {
            /* The baud rate is higher than the UART clock using
            * the divider latch.  Setup the over-sampling register.
            */
            divisor = 1;
        }

	if(divisor <= 1)
	{
#ifndef CONFIG_EXECUTE_ON_EMULATOR
           osr = (((port->uartclk / baud) - 1) << 4);
           osr |= (((port->uartclk % baud) / (baud / 8)) << 1);
           UART_SET_OSR(port, osr);
#endif
        }

	dll = (__u8) divisor;
	dlm = (__u8) (divisor >> 8);
	UART_SET_DLL(port, dll);
	UART_SET_DLM(port, dlm);

	/* Enable the RBR and THR for transmitt and Receive
	 * by Disabling the DLAB bit in LCR */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Enable the TX FIFO's and Reset to 1
	 * set the Trigger level to 56
	 * Do not reset RX FIFO or the kernel hangs during 
	 * bootup if user enters a key (the interrupt is pending but
	 * no char in the RX FIFO)
	 */
#ifdef CONFIG_UART_NX_DMAC_1902
	fcr =((IP3106_UART_FCR_FIF_ENA_MSK)	|		/* Bit 0 FIFO Enable */
		  (IP3106_UART_FCR_TXF_RES_MSK)  |
		  (IP3106_UART_FCR_RXF_RES_MSK) |
    	  (IP3106_UART_FCR_RX_TRIG_LEVEL_3
           << IP3106_UART_FCR_RX_TRIGGER_POS));		/* TX FIFO Reset */
        if (1 == uport->dma_enabled)
        {
            fcr |= (IP3106_UART_FCR_DMA_MSK); 
        }
#else
	fcr =((IP3106_UART_FCR_FIF_ENA_MSK)	|		/* Bit 0 FIFO Enable */
		  (IP3106_UART_FCR_TXF_RES_MSK)	|		/* TX FIFO Reset */
		  (IP3106_UART_FCR_RX_TRIG_LEVEL_3
		<< IP3106_UART_FCR_RX_TRIGGER_POS));
#endif
	   
	UART_SET_FCR(port,fcr);

	/* Clear the Interrupt status signals */
	UART_GET_MSR(port);
	UART_GET_LSR(port);
	UART_GET_IIR(port);

	/* Set the Modem Control Register to ZERO */
	UART_SET_MCR(port, 0);

	/* Store modem status signals */
	uport->old_status = UART_GET_MSR(port) &
	                          (IP3106_UART_MSR_DCD_MSK |
	                           IP3106_UART_MSR_DSR_MSK |
	                           IP3106_UART_MSR_CTS_MSK );

#ifdef CONFIG_IP3106_UART0
	/* Allocate the IRQ */
	if (port->irq == ip3106_ports0.port.irq)
		retval = request_irq(port->irq, ip3106_modem_int, 0, "uart-ip3106", port);
	else
#endif        
		retval = request_irq(port->irq, ip3106_int, 0, "uart-ip3106", port);

	if (retval > 0) {
		return retval;
	}

#ifdef CONFIG_UART_NX_DMAC_1902
	if (1 == uport->dma_enabled)
    {
		ip3106_request_dma(port);
		ip3106_start_tx(port);
		ip3106_start_rx(port);
    }
#endif	   

	/* Finally enable the receive interrupts interrupts */
	ier = (IP3106_UART_IER_RDAI_E_MSK | IP3106_UART_IER_RLSI_E_MSK );
	UART_SET_IER(port, ier);
	return 0;
}

static void ip3106_shutdown(struct uart_port *port)
{

	unsigned int lcr = 0;

	/* Free the interrupt */
	free_irq(port->irq, port);

	/* disable all interrupts, disable the port */
	/* To access Interrupt Enable Register, DLab bit in LCR must be ZERO */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Disable the port */
	UART_SET_IER(port, 0);

	/* disable break condition and fifos */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_BRK_CTL_MSK;
	UART_SET_LCR(port, lcr);

	/* Disable the FIFOs */
	UART_SET_FCR(port, 0);

#ifdef CONFIG_UART_NX_DMAC_1902
	struct ip3106_port *uport = (struct ip3106_port *)port;
    if (1 == uport->dma_enabled)
    {
    	ip3106_free_dma(port);
    }
#endif
}

static void ip3106_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	unsigned int      lcr = 0;
	unsigned int	  lcr1 = 0;
	unsigned int	  old_ier = 0;
	unsigned long     flags = 0;
	unsigned int      baud = 0;
	unsigned int	  quot = 0;
	unsigned int      mcr = 0;
#ifndef CONFIG_EXECUTE_ON_EMULATOR
	unsigned int      osr = 0;
#endif
	struct ip3106_port *uport = NULL;
	unsigned int      fcr = 0;

	uport = (struct ip3106_port *)port; /* Get the UART IP_3106 port*/

	/* Ask the core to calculate the divisor for us. */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);
	quot = uart_get_divisor(port, baud);
        if (quot == 0)
        {
            /* The baud rate is set faster than the UART clock using
             * the divider latch.  Setup the over-sampling register.
             */
            quot = 1;
        }

        if (quot <= 1)
        {
#ifndef CONFIG_EXECUTE_ON_EMULATOR
            osr = (((port->uartclk / baud) - 1) << 4);
            osr |= (((port->uartclk % baud) / (baud / 8)) << 1);
	    	UART_SET_OSR(port, osr);
#endif
        }

	switch (termios->c_cflag & CSIZE) {
		case CS5:
			lcr = IP3106_UART_LCR_WORDLEN_5BIT
				<< IP3106_UART_LCR_WORD_LEN_POS;
			break;
		case CS6:
			lcr = IP3106_UART_LCR_WORDLEN_6BIT
				<< IP3106_UART_LCR_WORD_LEN_POS;
			break;
		case CS7:
			lcr = IP3106_UART_LCR_WORDLEN_7BIT
				<< IP3106_UART_LCR_WORD_LEN_POS;
			break;
		default: /* CS8 */
			lcr = IP3106_UART_LCR_WORDLEN_8BIT
				<< IP3106_UART_LCR_WORD_LEN_POS;
			break;
	}

	if (termios->c_cflag & CSTOPB) {
		lcr |= (IP3106_UART_LCR_2_STOP_BIT
			<< IP3106_UART_LCR_STOPB_POS);
	}
	if (termios->c_cflag & PARENB) {
		lcr |= (IP3106_UART_LCR_PARITY << IP3106_UART_LCR_PAR_EN_POS);
		if (!(termios->c_cflag & PARODD)) {
			lcr |= (IP3106_UART_LCR_PARITY_EVEN
				<< IP3106_UART_LCR_PAR_SEL_POS) ;
		}
	}
	if (port->fifosize > 1) {
#ifdef CONFIG_UART_NX_DMAC_1902
		fcr =((IP3106_UART_FCR_FIF_ENA_MSK)	|	/* Bit 0 FIFO Enable */
			(IP3106_UART_FCR_RXF_RES_MSK)	|	/* RX FIFO Reset */
			(IP3106_UART_FCR_TXF_RES_MSK)	|	/* TX FIFO Reset */
			(IP3106_UART_FCR_RX_TRIG_LEVEL_3
				<< IP3106_UART_FCR_RX_TRIGGER_POS));
        if (1 == uport->dma_enabled)
        {
            fcr |= (IP3106_UART_FCR_DMA_MSK);
        }
#else
      fcr =((IP3106_UART_FCR_FIF_ENA_MSK) |  /* Bit 0 FIFO Enable */
         (IP3106_UART_FCR_RXF_RES_MSK) |  /* RX FIFO Reset */
         (IP3106_UART_FCR_TXF_RES_MSK) |  /* TX FIFO Reset */
         (IP3106_UART_FCR_RX_TRIG_LEVEL_3
            << IP3106_UART_FCR_RX_TRIGGER_POS));
#endif
	}

	spin_lock_irqsave(&port->lock, flags);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);
	port->read_status_mask = IP3106_UART_LSR_OE_MSK;

	if (termios->c_iflag & INPCK) {
		port->read_status_mask |= IP3106_UART_LSR_FE_MSK |
						IP3106_UART_LSR_PE_MSK;
	}
	if (termios->c_iflag & (BRKINT | PARMRK)) {
		port->read_status_mask |= IP3106_UART_LSR_BI_MSK;
	}

	/* Characters to ignore  */
	port->ignore_status_mask = 0;

	if (termios->c_iflag & IGNPAR) {
		port->ignore_status_mask |= IP3106_UART_LSR_FE_MSK |
						IP3106_UART_LSR_PE_MSK;
	}
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= IP3106_UART_LSR_BI_MSK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= IP3106_UART_LSR_OE_MSK;
	}
	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0) {
		port->ignore_status_mask |= UART_DUMMY_RSR_RX;
	}
	lcr1 = UART_GET_LCR(port);
	lcr1 &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port,lcr1);

	/* first, disable everything */
	old_ier = UART_GET_IER(port) & ~IP3106_UART_IER_MSI_E_MSK;
	if (UART_ENABLE_MS(port, termios->c_cflag)) {
		old_ier |= IP3106_UART_IER_MSI_E_MSK;
		/* If Auto RTSCTS present, use it */
		if (uport->autortscts == 1) {
			old_ier  |=  IP3106_UART_IER_CTSI_E_MSK;
			mcr = UART_GET_MCR(port);
			mcr |= IP3106_UART_MCR_ARTS_EN_MSK |
					IP3106_UART_MCR_ACTS_EN_MSK;
			UART_SET_MCR(port,mcr);
		}
		if (uport->flowctrl != 0)
			uport->flowctrl = CRTSCTS;
	} else if ( (termios->c_iflag & (IXON|IXOFF)) &&
			(uport->flowctrl = 0) ) {
		uport->flowctrl = IXON|IXOFF;
	} else {
		uport->flowctrl = 0;
	}

	UART_SET_IER(port,0);
	UART_SET_FCR(port,0);

	/* Set baud rate */
	/* To access DLL,DLM registers, DLab bit in LCR must be set */
	lcr1 = UART_GET_LCR(port);
	lcr1 |= IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr1);
	UART_SET_DLM(port, ((quot & 0xf00) >> 8));
	UART_SET_DLL(port, (quot & 0xff));

	/* To access Interrupt Enable Register,
	 * DLab bit in LCR must be ZERO
	 */
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);
	UART_SET_FCR(port, fcr);
	UART_SET_IER(port, old_ier);

#ifdef CONFIG_IP3106_UART0
	/* Enable Modem Status interrupt for UART0 Always */
	if (port->line == ip3106_ports0.port.line)
	{
		old_ier  = UART_GET_IER(port);
		if (!(old_ier & IP3106_UART_IER_MSI_E_MSK)) {
			old_ier |= IP3106_UART_IER_MSI_E_MSK;
			UART_SET_IER(port, old_ier);	
		}
	}
#endif

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ip3106_type(struct uart_port *port)
{
	return (port->type == PORT_UARTIP3106) ? "uart-ip3106" : NULL;
}

static void ip3106_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

static int ip3106_request_port(struct uart_port *port)
{
	return request_mem_region( port->mapbase, UART_PORT_SIZE,
			"uart-ip3106") != NULL ? 0 : -EBUSY;
}

static void ip3106_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_UARTIP3106;
		ip3106_request_port(port);
	}
}

static int
ip3106_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_UARTIP3106))
		ret = -EINVAL;
	if ((ser->irq < 0) || (ser->irq >= NR_IRQS))
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
static int ip3106_get_poll_char(struct uart_port *port)
{
    unsigned int  ch;
    unsigned int  status;

	/* Read till the FIFO is not empty  */
	do {
	   /* Read the status of the Rx FIFO */
	   status = UART_GET_LSR(port);
	} while ((status & IP3106_UART_LSR_DR_MSK) == 0); 
    
	/* Read the character from Rx FIFO */
	ch = UART_GET_RBR(port);
	return (ch & 0x7f);
}

static void ip3106_put_poll_char(struct uart_port *port,
			 unsigned char ch)
{
	unsigned int status = 0;

	/* Wait till the Tx FIFO is empty */
	do {
		status = UART_GET_LSR(port);
	} while (!(status & IP3106_UART_LSR_THRE_MSK));

	/* Write the character */
	UART_PUT_THR(port,(ch | 0x80));
	return;

}

#endif /* CONFIG_CONSOLE_POLL */

/*--------------------------------------------------------------------------*
 * Port Control Functions for UART IP3106                                   *
 *--------------------------------------------------------------------------*/
static struct uart_ops ip3106_pops =
{
	.tx_empty     = ip3106_tx_empty,
	.set_mctrl    = ip3106_set_modem_ctrl,
	.get_mctrl    = ip3106_get_modem_ctrl,
	.stop_tx      = ip3106_stop_tx,
	.start_tx     = ip3106_start_tx,
	.stop_rx      = ip3106_stop_rx,
	.enable_ms    = ip3106_enable_modem_status,
	.break_ctl    = ip3106_break_ctrl,
	.startup      = ip3106_startup,
	.shutdown     = ip3106_shutdown,
	.set_termios  = ip3106_set_termios,
	.type	      = ip3106_type,
	.release_port = ip3106_release_port,
	.request_port = ip3106_request_port,
	.config_port  = ip3106_config_port,
	.verify_port  = ip3106_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = ip3106_get_poll_char,
	.poll_put_char = ip3106_put_poll_char,
#endif
};

#if defined(CONFIG_SERIAL_IP3106_CONSOLE)
/*--------------------------------------------------------------------------*
 * Console functions for UART IP3106                                        *
 *--------------------------------------------------------------------------*/
#ifdef CONFIG_IP3106_UART0
static struct ip3106_port ip3106_ports0 =
{
	.port   =
	{
		.membase        = IP3106_UART_PORT_0,
		.mapbase        = (resource_size_t)IP3106_UART_PORT_0,
		.iotype         = SERIAL_IO_MEM,
		.irq            = IRQ_UART_0,
		.uartclk        = UART_CLOCK_RATE,
		.fifosize       = 64,
		.ops            = &ip3106_pops,
		.flags          = UPF_BOOT_AUTOCONF,
		.line           = 0,
	},
	.autortscts = 0,
	.flowctrl   = CRTSCTS, /* Flow control should be enabled for modem port */
};
#endif

static struct ip3106_port ip3106_ports1 =
{
	.port   =
	{
		.membase        = IP3106_UART_PORT_1,
		.mapbase        = (resource_size_t)IP3106_UART_PORT_1,
		.iotype         = SERIAL_IO_MEM,
		.irq            = IRQ_UART_1,
		.uartclk        = UART_CLOCK_RATE,
		.fifosize       = 64,
		.ops            = &ip3106_pops,
		.flags          = UPF_BOOT_AUTOCONF,
		.line           = 1,
	},
	.autortscts = 0,
	.flowctrl   = 0, /* Flow control is not supported on our target */
};

static struct ip3106_port ip3106_ports2 =
{
	.port   =
	{
		.membase        = IP3106_UART_PORT_2,
		.mapbase        = (resource_size_t)IP3106_UART_PORT_2,
		.iotype         = SERIAL_IO_MEM,
		.irq            = IRQ_UART_2, /* LET OP IRQ_UART-nbr !! */
		.uartclk        = UART_CLOCK_RATE,
		.fifosize       = 64,
		.ops            = &ip3106_pops,
		.flags          = UPF_BOOT_AUTOCONF,
		.line           = 2,
	},
	.autortscts = 0,
	.flowctrl   = 0, /* Flow control is not supported on our target */
};

#ifdef CONFIG_ARCH_APOLLO
static struct ip3106_port ip3106_ports3 =
{
	.port   =
	{
		.membase        = IP3106_UART_PORT_3,
		.mapbase        = (resource_size_t)IP3106_UART_PORT_3,
		.iotype         = SERIAL_IO_MEM,
		.irq            = IRQ_UART_3, /* LET OP IRQ_UART-nbr !! */
		.uartclk        = UART_CLOCK_RATE,
		.fifosize       = 64,
		.ops            = &ip3106_pops,
		.flags          = UPF_BOOT_AUTOCONF,
		.line           = 3,
	},
	.autortscts = 0,
	.flowctrl   = 0, /* Flow control is not supported on our target */
};
#endif

static void ip3106_console_write(struct console *co,
	const char *s, unsigned int count)
{
	unsigned int 	status = 0;
	struct ip3106_port *uap = NULL;
	int i = 0;
	unsigned long flags;

#ifdef CONFIG_IP3106_UART0
	if(co->index == 0)
		uap = &ip3106_ports0;
	else
#endif        
    if(co->index == 1)
		uap = &ip3106_ports1;
	else if(co->index == 2)
		uap = &ip3106_ports2;
#ifdef CONFIG_ARCH_APOLLO
	else 
		uap = &ip3106_ports3;
#endif

	spin_lock_irqsave(&uap->port.lock, flags);

	/* Now write the each character */
	for (i = 0; i < count; i++) {
		/* Wait till the Tx FIFO is ready */
		do {
			status = __raw_readl(uap->port.membase +
					IP3106_UART_LSR_REG);
		} while (!(status & IP3106_UART_LSR_THRE_MSK));

		__raw_writel(s[i],uap->port.membase + IP3106_UART_THR_REG);
		if (s[i] == '\n') {
			do {
				status = __raw_readl(uap->port.membase +
						IP3106_UART_LSR_REG);
			} while (!(status & IP3106_UART_LSR_THRE_MSK));
			__raw_writel('\r',uap->port.membase +
					IP3106_UART_THR_REG);
		}
	}
	/* Wait for transmitter to become empty
	 *	and restore the IER */
	do {
		status = __raw_readl(uap->port.membase + IP3106_UART_LSR_REG);
	} while (!(status & IP3106_UART_LSR_THRE_MSK));

	spin_unlock_irqrestore(&uap->port.lock, flags);

}

static void __init ip3106_console_get_options(struct uart_port *port,
	int *baud, int *parity, int *bits)
{
	unsigned int lcr = 0;
	unsigned int quot = 0;
	unsigned int word_len = 0;

	lcr = UART_GET_LCR(port);
	*parity = 'n';

	if (lcr & IP3106_UART_LCR_PAR_EN_MSK) {
		if (lcr & IP3106_UART_LCR_PAR_SEL_MSK)
			*parity = 'e';
		else
			*parity = 'o';
	}
	word_len = lcr & IP3106_UART_LCR_WORD_LEN_MSK;
	switch (word_len) {
		case IP3106_UART_LCR_WORDLEN_5BIT:
			*bits = 5;
			break;
		case IP3106_UART_LCR_WORDLEN_6BIT:
			*bits = 6;
			break;
		case IP3106_UART_LCR_WORDLEN_7BIT:
			*bits = 7;
			break;
		default:
			*bits = 8;
			break;
	}

	/* To access DLL,DLM, DLab bit in LCR must be 1 */
	lcr = UART_GET_LCR(port);
	lcr |= IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);

	/* Calculate the baud rate from DLL,DLM values */
	quot = UART_GET_DLL(port) | UART_GET_DLM(port) << 8;
	*baud = port->uartclk / (quot + 1);

	/* Disable the access to DLL,DLM */
	lcr = UART_GET_LCR(port);
	lcr &= ~IP3106_UART_LCR_DLAB_MSK;
	UART_SET_LCR(port, lcr);
}

static int __init ip3106_console_setup(struct console *co, char *options)
{
	struct ip3106_port *uap = NULL;
	int baud = BASE_BAUD;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/* Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if ((co->index >= UART_NR) || (co->index < 0)) {
		co->index = 0;
	}

#ifdef CONFIG_IP3106_UART0
	if(co->index == 0)
		uap = &ip3106_ports0;
	else
#endif
    if(co->index == 1)
		uap = &ip3106_ports1;
	else if(co->index == 2)
		uap = &ip3106_ports2;
#ifdef CONFIG_ARCH_APOLLO
	else 
		uap = &ip3106_ports3;
#endif
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else {
		ip3106_console_get_options(&uap->port, &baud, &parity, &bits);
	}

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

/*--------------------------------------------------------------------------*
 * Console strcuture for UART IP3106                                        *
 *--------------------------------------------------------------------------*/
static struct uart_driver ip3106_uart; /* Forward declaration */
static struct console ip3106_console = {
	.name		= "ttyS",
	.write		= ip3106_console_write,
	.device		= uart_console_device,
	.setup		= ip3106_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &ip3106_uart,
};

static int __init ip3106_console_init(void)
{
	register_console(&ip3106_console);
	return 0;
}
console_initcall(ip3106_console_init);

static int __init ip3106_late_console_init(void)
{
	if (!(ip3106_console.flags & CON_ENABLED)) {
		register_console(&ip3106_console);
	}
	return 0;
}
late_initcall(ip3106_late_console_init);

#endif /* CONFIG_SERIAL_IP3106_CONSOLE */

/*--------------------------------------------------------------------------*
 * Driver Registration structure for UART IP3106                            *
 *--------------------------------------------------------------------------*/
static struct uart_driver ip3106_uart = {
	.owner			= THIS_MODULE,
	.driver_name	= "ttyS",
	.dev_name		= "ttyS",
	.major			= 4,	/* Major device number */
	.minor			= 64,	/* Minor device number */
	.nr				= UART_NR,
#if defined(CONFIG_SERIAL_IP3106_CONSOLE)
	.cons			= &ip3106_console,
#else
	.cons			= NULL;
#endif
};

/*
 * Amba device driver
 */
static int ip3106_probe(struct platform_device *dev)
{
	struct ip3106_port *uap = NULL;
	struct resource *res;
	void __iomem *base;


	int ret = -ENODEV;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	uap = kmalloc(sizeof(struct ip3106_port), GFP_KERNEL);
	if (uap == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	memset(uap, 0, sizeof(struct ip3106_port));

	base = ioremap(res->start, res->end - res->start + 1);
	if (!base) {
		ret = -ENOMEM;
		goto free;
	}

	uap->port.dev = &dev->dev;
	uap->port.ops = &ip3106_pops;
	uap->port.membase = base;
	uap->port.mapbase = res->start;
	uap->port.iotype  = SERIAL_IO_MEM;
	uap->port.irq     = platform_get_irq(dev, 0);
	uap->port.uartclk = UART_CLOCK_RATE;
	uap->port.fifosize= 64;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line  = ip3106_line;
	uap->autortscts = 0;
	uap->flowctrl   = 0;
#ifdef CONFIG_UART_NX_DMAC_1902
    struct nx_uart_dma_platform_data *plfdata;
    plfdata = (struct nx_uart_dma_platform_data*)(dev->dev.platform_data);
          
   	uap->dma_enabled = plfdata->dma_enabled;
    if (1 == uap->dma_enabled)
    {
    	uap->client_rx = plfdata->slave_rx;
        uap->client_tx = plfdata->slave_tx;
		uap->chantx_alloted = false;
		uap->chanrx_alloted = false;

	    uap->txchan_num = plfdata->txchan_num;
	    uap->rxchan_num = plfdata->rxchan_num;

        uap->chan_rx = NULL;
        uap->chan_tx = NULL;

        uap->client_tx->tx_reg =  (u32)((uap->port.mapbase & 0xffff) + IP3106_UART_THR_REG);
        uap->client_rx->rx_reg =  (u32)((uap->port.mapbase& 0xffff) + IP3106_UART_RBR_REG);

	}
#endif

	spin_lock_init(&uap->port.lock);
	platform_set_drvdata(dev, uap);
	ip3106_line++;

	ret = uart_add_one_port(&ip3106_uart, &uap->port);
	if (ret) {
		platform_set_drvdata(dev, NULL);
		iounmap(base);
 free:
		kfree(uap);
	}

 out:
	return ret;
}

static int ip3106_remove(struct platform_device *dev)
{
	struct ip3106_port *uap = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	ip3106_line--;
	uart_remove_one_port(&ip3106_uart, &uap->port);
	iounmap(uap->port.membase);
	kfree(uap);
	return 0;
}

#ifdef CONFIG_PM
static int ip3106_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct ip3106_port *uap = platform_get_drvdata(dev);

	if (uap)
		uart_suspend_port(&ip3106_uart, &uap->port);

	return 0;
}

static int ip3106_resume(struct platform_device *dev)
{
	struct ip3106_port *uap = platform_get_drvdata(dev);

	if (uap)
		uart_resume_port(&ip3106_uart, &uap->port);
	return 0;
}
#else
#define ip3106_suspend	NULL
#define ip3106_resume	NULL
#endif


static struct platform_device_id ip3106_driver_ids[] = {
    {
        .name       = "stb-uart",
    }, 
	{ },
};
MODULE_DEVICE_TABLE(platform, ip3106_driver_ids);

static const struct of_device_id ip3106_uart_dt_match[] = {
    { .compatible = "entr,stb-uart"},
    {},
};
MODULE_DEVICE_TABLE(of, ip3106_uart_dt_match);

static struct platform_driver ip3106_driver = {
	.driver		= {
		.name	= "uart-ip3106",
		.bus	= &platform_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = ip3106_uart_dt_match,
	},
	.id_table   = ip3106_driver_ids,
	.probe		= ip3106_probe,
	.remove		= ip3106_remove,
	.suspend	= ip3106_suspend,
	.resume		= ip3106_resume,
};

/* Added for Linux native serial wrapper */
struct uart_driver *serial_wrapper_ip3106_uart = &ip3106_uart;
EXPORT_SYMBOL(serial_wrapper_ip3106_uart);

static int __init ip3106_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&ip3106_uart);
	if (ret == 0) {
		ret = platform_driver_register(&ip3106_driver);
		if (ret)
			uart_unregister_driver(&ip3106_uart);
	} else {
		printk (KERN_ALERT "IP3106: failed to register UART driver!! ret = %d\n", ret);
	}
	return ret;
}

static void __exit ip3106_exit(void)
{
	platform_driver_unregister(&ip3106_driver);
	uart_unregister_driver(&ip3106_uart);
}

module_init(ip3106_init);
module_exit(ip3106_exit);

MODULE_DESCRIPTION("ip3106 serial port driver");
MODULE_LICENSE("GPL");
