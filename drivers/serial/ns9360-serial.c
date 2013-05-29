/*
 * drivers/serial/ns9360-serial.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Based on drivers/serial/ns9xxx_serial.c by Markus Pietrek
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>

/* register offsets */

#define UART_CTRLA	0x00
#define UART_CTRLA_CE			(1 << 31)
#define UART_CTRLA_BRK			(1 << 30)
#define UART_CTRLA_EPS			(1 << 28)
#define UART_CTRLA_PE			(1 << 27)
#define UART_CTRLA_STOP			(1 << 26)
#define UART_CTRLA_WLS_5		(0)
#define UART_CTRLA_WLS_6		(1 << 24)
#define UART_CTRLA_WLS_7		(2 << 24)
#define UART_CTRLA_WLS_8		(3 << 24)
#define UART_CTRLA_DTR			(1 << 17)
#define UART_CTRLA_RTS			(1 << 16)
#define UART_CTRLA_RIE_RDY		(1 << 11)
#define UART_CTRLA_RIE_CLOSED		(1 << 9)
#define UART_CTRLA_RIE_MASK		(0xe00)
#define UART_CTRLA_RIC_MASK		(0xe0)
#define UART_CTRLA_TIC_HALF		(1 << 2)
#define UART_CTRLA_TIC_MASK		(0x1e)

#define UART_CTRLB	0x04
#define UART_CTRLB_RCGT			(1 << 26)

#define UART_STATUSA		0x08
#define UART_STATUSA_RXFDB_MASK		(3 << 20)
#define UART_STATUSA_DCD		(1 << 19)
#define UART_STATUSA_RI			(1 << 18)
#define UART_STATUSA_DSR		(1 << 17)
#define UART_STATUSA_CTS		(1 << 16)
#define UART_STATUSA_RBRK		(1 << 15)
#define UART_STATUSA_RFE		(1 << 14)
#define UART_STATUSA_RPE		(1 << 13)
#define UART_STATUSA_ROVER		(1 << 12)
#define UART_STATUSA_RRDY		(1 << 11)
#define UART_STATUSA_RBC		(1 << 9)
#define UART_STATUSA_TRDY		(1 << 3)
#define UART_STATUSA_TEMPTY		(1 << 0)

#define UART_FIFO		0x10

#define UART_BITRATE		0x0c
#define UART_BITRATE_EBIT		(1 << 31)
#define UART_BITRATE_TMODE		(1 << 30)
#define UART_BITRATE_CLKMUX_BCLK	(1 << 24)
#define UART_BITRATE_TCDR_16		(1 << 20)
#define UART_BITRATE_RCDR_16		(1 << 18)
#define UART_BITRATE_N_MASK		(0x7fff)

#define UART_RXCHARTIMER	0x18
#define UART_RXCHARTIMER_TRUN		(1 << 31)

#if defined(CONFIG_SERIAL_NS9360_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#define DRIVER_NAME "ns9360-serial"

#ifdef CONFIG_SERIAL_NS9360_COMPAT
# define NS9360_TTY_NAME	"ttyS"
# define NS9360_TTY_MAJOR	TTY_MAJOR
# define NS9360_TTY_MINOR_START	64
#else
# define NS9360_TTY_NAME	"ttyNS"
# define NS9360_TTY_MAJOR	204
# define NS9360_TTY_MINOR_START	200
#endif

#define NS9360_UART_NR 4

#define up2unp(up) container_of(up, struct uart_ns9360_port, port)
struct uart_ns9360_port {
	struct uart_port port;
	struct clk *clk;
};

#ifdef CONFIG_SERIAL_NS9750_CONSOLE

#endif

static inline u32 uartread32(struct uart_port *port, unsigned int offset)
{
	u32 ret = ioread32(port->membase + offset);

#if defined(DEBUG_UARTRW)
	dev_info(port->dev, "read  0x%p -> 0x%08x\n",
			port->membase + offset, ret);
#endif

	return ret;
}

static inline void uartwrite32(struct uart_port *port,
		u32 value, unsigned int offset)
{
#if defined(DEBUG_UARTRW)
	dev_info(port->dev, "write 0x%p <- 0x%08x\n",
			port->membase + offset, value);
#endif
	iowrite32(value, port->membase + offset);
}

static inline void uartwrite8(struct uart_port *port, u8 value,
		unsigned int offset)
{
#if defined(DEBUG_UARTRW)
	dev_info(port->dev, "write 0x%p <- 0x%02x\n",
			port->membase + offset, value);
#endif
	iowrite8(value, port->membase + offset);
}

static inline void ns9360_uart_wait_fifo_empty(struct uart_port *port)
{
	while (!(uartread32(port, UART_STATUSA) & UART_STATUSA_TEMPTY))
		udelay(1);

	/* TODO: until we have buffer closed again, do just a delay, enough
	 * for 38400 baud */
	mdelay(1);
}

static inline void ns9360_uart_wait_xmitr(struct uart_port *port)
{
	while (!(uartread32(port, UART_STATUSA) & UART_STATUSA_TRDY))
		udelay(1);
}

/* called with port->lock taken */
static void ns9360_uart_stop_tx(struct uart_port *port)
{
	u32 ctrl;
	struct uart_ns9360_port *unp = up2unp(port);

	assert_spin_locked(&unp->port.lock);

	/* disable TIC_HALF */
	ctrl = uartread32(port, UART_CTRLA);
	ctrl &= ~UART_CTRLA_TIC_HALF;
	uartwrite32(port, ctrl, UART_CTRLA);
}

/* receive chars from serial fifo and store them in tty
 * This is called with port->lock taken */
static void ns9360_uart_rx_chars(struct uart_ns9360_port *unp)
{
	struct tty_struct *tty = unp->port.state->port.tty;
	u32 status, available, characters, flag;

	/* acknowledge rbc if set */
	status = uartread32(&unp->port, UART_STATUSA);
	if (status & UART_STATUSA_RBC) {
		uartwrite32(&unp->port, UART_STATUSA_RBC, UART_STATUSA);
		status = uartread32(&unp->port, UART_STATUSA);
	}

	while (status & UART_STATUSA_RRDY) {
		available = (status & UART_STATUSA_RXFDB_MASK) >> 20;
		available = available ? : 4;

		flag = TTY_NORMAL;

		if (status & UART_STATUSA_ROVER) {
			unp->port.icount.overrun++;
			flag |= TTY_OVERRUN;
		}
		if (status & UART_STATUSA_RFE) {
			unp->port.icount.frame++;
			flag |= TTY_FRAME;
		}
		if (status & UART_STATUSA_RPE) {
			unp->port.icount.parity++;
			flag |= TTY_PARITY;
		}
		if (status & UART_STATUSA_RBRK) {
			unp->port.icount.brk++;
			flag |= TTY_BREAK;
		}

		/* read characters from fifo */
		characters = uartread32(&unp->port, UART_FIFO);
		do {
			unp->port.icount.rx++;
			/* TODO: switch to uart_insert_char */
			tty_insert_flip_char(tty, characters & 0xff, flag);
			characters >>= 8;
		} while (--available);

		status = uartread32(&unp->port, UART_STATUSA);
	}

	if (!tty->low_latency)
		tty_flip_buffer_push(tty);

#ifdef SUPPORT_SYSRQ
	sport->port.sysrq = 0;
#endif
}

/* send out chars in xmit buffer.  This is called with port->lock taken */
static void ns9360_uart_tx_chars(struct uart_ns9360_port *unp)
{
	struct circ_buf *xmit = &unp->port.state->xmit;

	assert_spin_locked(&unp->port.lock);

	if (unp->port.x_char) {
		uartwrite8(&unp->port, unp->port.x_char, UART_FIFO);
		unp->port.icount.tx++;
		unp->port.x_char = 0;
	} else
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(&unp->port)) {
			while (uartread32(&unp->port, UART_STATUSA) &
					UART_STATUSA_TRDY) {
				uartwrite8(&unp->port, xmit->buf[xmit->tail],
						UART_FIFO);
				xmit->tail = (xmit->tail + 1) &
					(UART_XMIT_SIZE - 1);
				unp->port.icount.tx++;
				if (uart_circ_empty(xmit))
					break;
			}

			/* wakup ourself if number of pending chars do not reach
			 * the minimum wakeup level */
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(&unp->port);
		}

	/* if uart is empty now ? */
	if (uart_circ_empty(xmit) || uart_tx_stopped(&unp->port))
		ns9360_uart_stop_tx(&unp->port);
}

static irqreturn_t ns9360_uart_rxint(int irq, void *dev_id)
{
	struct uart_ns9360_port *unp = dev_id;

	spin_lock(&unp->port.lock);
	ns9360_uart_rx_chars(unp);
	spin_unlock(&unp->port.lock);

	return IRQ_HANDLED;
}

static irqreturn_t ns9360_uart_txint(int irq, void *dev_id)
{
	struct uart_ns9360_port *unp = dev_id;

	spin_lock(&unp->port.lock);
	ns9360_uart_tx_chars(unp);
	spin_unlock(&unp->port.lock);

	return IRQ_HANDLED;
}

static void ns9360_uart_release_port(struct uart_port *port)
{
	/* XXX: release_mem_region is marked as Compatibility cruft ??? */
	release_mem_region(port->mapbase, 0x3f);
}

static int ns9360_uart_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, 0x3f,
			DRIVER_NAME) ? 0 : -EBUSY;
}

static void ns9360_uart_enable_ms(struct uart_port *port)
{
	u32 status;

	status = uartread32(port, UART_STATUSA);
	/* status |= NS_SER_CTRL_A_RIC_MA; */
	/* FIXME: sense? */
	uartwrite32(port, status, UART_STATUSA);
}

static void ns9360_uart_shutdown(struct uart_port *port)
{
	struct uart_ns9360_port *unp = up2unp(port);
	unsigned long flags;
	u32 ctrl;

	ns9360_uart_wait_fifo_empty(port);

	spin_lock_irqsave(&port->lock, flags);
	ctrl = uartread32(port, UART_CTRLA);
	ctrl &= ~UART_CTRLA_RIE_RDY;
	uartwrite32(port, ctrl, UART_CTRLA);
	spin_unlock_irqrestore(&port->lock, flags);

	free_irq(port->irq, unp);
	free_irq(port->irq + 1, unp);

	clk_disable(unp->clk);
}

static int ns9360_uart_startup(struct uart_port *port)
{
	struct uart_ns9360_port *unp = up2unp(port);
	int ret;
	u32 ctrl;
	unsigned long flags;

	ret = clk_enable(unp->clk);
	if (ret) {
		dev_info(port->dev, "%s: err_clk_enable", __func__);
		goto err_clk_enable;
	}

	unp->port.uartclk = clk_get_rate(unp->clk);

	ret = request_irq(unp->port.irq, ns9360_uart_rxint, 0,
			DRIVER_NAME, unp);
	if (ret) {
		dev_info(port->dev, "%s: err_request_irq_rx", __func__);
		goto err_request_irq_rx;
	}

	ret = request_irq(unp->port.irq + 1, ns9360_uart_txint, 0,
			DRIVER_NAME, unp);
	if (ret) {
		dev_info(port->dev, "%s: err_request_irq_tx", __func__);
		goto err_request_irq_tx;
	}

	/* enable receive interrupts */
	spin_lock_irqsave (&unp->port.lock, flags);
	ctrl = uartread32(&unp->port, UART_CTRLA);
	ctrl |= UART_CTRLA_CE | UART_CTRLA_RIE_RDY | UART_CTRLA_RIE_CLOSED;
	uartwrite32(&unp->port, ctrl, UART_CTRLA);

	/* enable modem status interrupts */
	ns9360_uart_enable_ms(&unp->port);
	spin_unlock_irqrestore (&unp->port.lock, flags);

	/* enable character gap timer */
	uartwrite32(&unp->port, UART_CTRLB_RCGT, UART_CTRLB);

	return 0;

err_request_irq_tx:
	free_irq(unp->port.irq, unp);
err_request_irq_rx:
	clk_disable(unp->clk);
err_clk_enable:
	return ret;
}

static void ns9360_uart_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	struct uart_ns9360_port *unp = up2unp(port);
	unsigned long flags;
	unsigned int baud, quot, nr_bits;
	u32 ctrl, bitrate, gap_timer;

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	/* read out configuration and mask out bits going to be updated */
	ctrl = uartread32(port, UART_CTRLA);
	ctrl &= ~(UART_CTRLA_WLS_8 | UART_CTRLA_STOP | UART_CTRLA_PE |
			UART_CTRLA_EPS);

	ctrl |= UART_CTRLA_CE;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		ctrl |= UART_CTRLA_WLS_5;
		nr_bits = 5;
		break;
	case CS6:
		ctrl |= UART_CTRLA_WLS_6;
		nr_bits = 6;
		break;
	case CS7:
		ctrl |= UART_CTRLA_WLS_7;
		nr_bits = 7;
		break;
	default:
	case CS8:
		ctrl |= UART_CTRLA_WLS_8;
		nr_bits = 8;
		break;
	}

	if (termios->c_cflag & CSTOPB) {
		ctrl |= UART_CTRLA_STOP;
		nr_bits++;
	}

	if (termios->c_cflag & PARENB) {
		ctrl |= UART_CTRLA_PE;
		nr_bits++;
	}

	if (!(termios->c_cflag & PARODD)) {
		ctrl |= UART_CTRLA_EPS;
		nr_bits++;
	}

	/* set configuration */
	ns9360_uart_wait_fifo_empty(port);
	uartwrite32(port, ctrl, UART_CTRLA);

	/* set baudrate */
	bitrate = UART_BITRATE_EBIT | UART_BITRATE_CLKMUX_BCLK |
		UART_BITRATE_TMODE | UART_BITRATE_TCDR_16 |
		UART_BITRATE_RCDR_16 | ((quot - 1) & UART_BITRATE_N_MASK);
	uartwrite32(port, bitrate, UART_BITRATE);

	/* set character gap timer */
	gap_timer = UART_RXCHARTIMER_TRUN;

	/* calculate gap timer */
	gap_timer |= (clk_get_rate(unp->clk) * nr_bits / baud / 8) - 1;
	pr_debug(DRIVER_NAME " %s: gap-timer = %08x (baud=%i, clk=%li)\n",
			__func__, gap_timer, baud, clk_get_rate(unp->clk));

	uartwrite32(port, gap_timer, UART_RXCHARTIMER);
	spin_unlock_irqrestore(&port->lock, flags);
}

static unsigned int ns9360_uart_tx_empty(struct uart_port *port)
{
	u32 status;

	status = uartread32(port, UART_STATUSA);
	return (status & UART_STATUSA_TRDY) ? TIOCSER_TEMT : 0;
}

static void ns9360_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 mcr;

	mcr = uartread32(port, UART_CTRLA);

	if (mctrl & TIOCM_RTS)
		mcr &= ~UART_CTRLA_RTS;
	else
		mcr |= UART_CTRLA_RTS;

	if (mctrl & TIOCM_DTR)
		mcr &= ~UART_CTRLA_DTR;
	else
		mcr |= UART_CTRLA_DTR;

	uartwrite32(port, mcr, UART_CTRLA);
}

static unsigned int ns9360_uart_get_mctrl(struct uart_port *port)
{
	unsigned int status;
	unsigned int ret = 0;

	status = uartread32(port, UART_STATUSA);

	if (!(status & UART_STATUSA_DCD))
		ret |= TIOCM_CD;
	if (!(status & UART_STATUSA_CTS))
		ret |= TIOCM_CTS;
	if (!(status & UART_STATUSA_DSR))
		ret |= TIOCM_DSR;
	if (!(status & UART_STATUSA_RI))
		ret |= TIOCM_RI;

	return ret;
}

static int ns9360_uart_verify_port(struct uart_port *port,
		struct serial_struct *ser)
{
	struct uart_ns9360_port *unp = up2unp(port);
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NS9360)
		ret = -EINVAL;

	if (ser->irq != unp->port.irq)
		ret = -EINVAL;

	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;

	return ret;
}

static void ns9360_uart_start_tx(struct uart_port *port)
{
	u32 ctrl;

	ctrl = uartread32(port, UART_CTRLA);
	ctrl |= UART_CTRLA_TIC_HALF;
	uartwrite32(port, ctrl, UART_CTRLA);
}

static void ns9360_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		if (!ns9360_uart_request_port(port))
			port->type = PORT_NS9360;
}

static const char *ns9360_uart_type(struct uart_port *port)
{
	struct platform_device *pdev;

	if (port->type == PORT_NS9360) {
		pdev = to_platform_device(port->dev);
		if (0 == pdev->id)
			return "NS9360 PORT B";
		else if (1 == pdev->id)
			return "NS9360 PORT A";
		else if (2 == pdev->id)
			return "NS9360 PORT C";
		else if (3 == pdev->id)
			return "NS9360 PORT D";
		else
			return NULL;
	}
	else
		return NULL;
}

static void ns9360_uart_break_ctl(struct uart_port *port, int break_state)
{
	u32 ctrl;

	ctrl = uartread32(port, UART_CTRLA);

	if (break_state == -1)
		ctrl |= UART_CTRLA_BRK;
	else
		ctrl &= ~UART_CTRLA_BRK;

	uartwrite32(port, ctrl, UART_CTRLA);
}

static void ns9360_uart_stop_rx(struct uart_port *port)
{
	u32 ctrl;

	ctrl = uartread32(port, UART_CTRLA);
	ctrl &= ~(UART_CTRLA_RIE_RDY | UART_CTRLA_RIE_CLOSED);
	uartwrite32(port, ctrl, UART_CTRLA);
}

static struct uart_ops ns9360_uart_pops = {
	.stop_rx = ns9360_uart_stop_rx,
	.enable_ms = ns9360_uart_enable_ms,
	.break_ctl = ns9360_uart_break_ctl,
	.type = ns9360_uart_type,
	.config_port = ns9360_uart_config_port,
	.start_tx = ns9360_uart_start_tx,
	.verify_port = ns9360_uart_verify_port,
	.set_mctrl = ns9360_uart_set_mctrl,
	.get_mctrl = ns9360_uart_get_mctrl,
	.tx_empty = ns9360_uart_tx_empty,
	.set_termios = ns9360_uart_set_termios,
	.shutdown = ns9360_uart_shutdown,
	.startup = ns9360_uart_startup,
	.stop_tx = ns9360_uart_stop_tx,
	.release_port = ns9360_uart_release_port,
	.request_port = ns9360_uart_request_port,
};

static struct uart_ns9360_port *ns9360_uart_ports[NS9360_UART_NR];

#ifdef CONFIG_SERIAL_NS9360_CONSOLE

static void ns9360_uart_console_write(struct console *co,
		const char *s, unsigned int count)
{
	struct uart_ns9360_port *unp = ns9360_uart_ports[co->index];
	u32 ctrl, saved_ctrl;

	/* save current state */
	saved_ctrl = uartread32(&unp->port, UART_CTRLA);

	/* keep configuration, disable interrupts and enable uart */
	ctrl = (saved_ctrl | UART_CTRLA_CE) & ~(UART_CTRLA_TIC_MASK |
			UART_CTRLA_RIC_MASK | UART_CTRLA_RIE_MASK);
	uartwrite32(&unp->port, ctrl, UART_CTRLA);

	while (count) {
		ns9360_uart_wait_xmitr(&unp->port);

		if (*s == '\n') {
			uartwrite8(&unp->port, '\r', UART_FIFO);
			ns9360_uart_wait_xmitr(&unp->port);
		}

		uartwrite8(&unp->port, *s, UART_FIFO);

		count--;
		s++;
	}

	/* wait for fifo empty if uart was (and will be) disabled */
	if (!(saved_ctrl & UART_CTRLA_CE))
		ns9360_uart_wait_fifo_empty(&unp->port);

	/* recover state */
	uartwrite32(&unp->port, saved_ctrl, UART_CTRLA);
}

/* If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup. */
static void __init ns9360_uart_console_get_options(struct uart_ns9360_port *unp,
		int *baud, int *parity, int *bits, int *flow)
{
	u32 ctrl, bitrate;

	ctrl = uartread32(&unp->port, UART_CTRLA);
	bitrate = uartread32(&unp->port, UART_BITRATE);

	/* not enabled, don't modify defaults */
	if (!(ctrl & UART_CTRLA_CE))
		return;

	switch (ctrl & UART_CTRLA_WLS_8) {
	case UART_CTRLA_WLS_5:
		*bits = 5;
		break;
	case UART_CTRLA_WLS_6:
		*bits = 6;
		break;
	case UART_CTRLA_WLS_7:
		*bits = 7;
		break;
	default:
	case UART_CTRLA_WLS_8:
		*bits = 8;
		break;
	}

	*parity = 'n';
	if (ctrl & UART_CTRLA_PE) {
		/* parity enabled */
		if (ctrl & UART_CTRLA_EPS)
			*parity = 'o';
		else
			*parity = 'e';
	}

	*baud = unp->port.uartclk /
		(((bitrate & UART_BITRATE_N_MASK) + 1) * 16);
}

static int __init ns9360_uart_console_setup(struct console *co, char *options)
{
	struct uart_ns9360_port *unp;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= NS9360_UART_NR)
		co->index = 0;

	unp = ns9360_uart_ports[co->index];
	if (!unp)
		return -ENODEV;

	/* XXX: assert unp->clk is enabled */
	unp->port.uartclk = clk_get_rate(unp->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		ns9360_uart_console_get_options(unp,
				&baud, &parity, &bits, &flow);

	/* Enable UART. For some strange reason, in NS9360 the UART
	 * must be enabled in the setup, rather than in the write
	 * function, otherwise nothing is printed and the system
	 * doesn't boot */
	uartwrite32(&unp->port,
		   uartread32(&unp->port, UART_CTRLA) | UART_CTRLA_CE,
		   UART_CTRLA);

	return uart_set_options(&unp->port, co, baud, parity, bits, flow);
}

static struct uart_driver ns9360_uart_reg;
static struct console ns9360_uart_console = {
	.name = NS9360_TTY_NAME,
	.write = ns9360_uart_console_write,
	.device = uart_console_device,
	.setup = ns9360_uart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &ns9360_uart_reg,
};

#define NS9360_UART_CONSOLE (&ns9360_uart_console)
#else
#define NS9360_UART_CONSOLE NULL
#endif

static struct uart_driver ns9360_uart_reg = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = NS9360_TTY_NAME,
	.major = NS9360_TTY_MAJOR,
	.minor = NS9360_TTY_MINOR_START,
	.nr = NS9360_UART_NR,
	.cons = NS9360_UART_CONSOLE,
};

static __devinit int ns9360_uart_pdrv_probe(struct platform_device *pdev)
{
	int ret, line, irq;
	struct uart_ns9360_port *unp;
	struct resource *mem;
	void __iomem *base;

	line = pdev->id;
	if (line < 0 || line >= ARRAY_SIZE(ns9360_uart_ports)) {
		ret = -ENODEV;
		dev_info(&pdev->dev, "%s: err_line\n", __func__);
		goto err_line;
	}

	if (ns9360_uart_ports[line] != NULL) {
		ret = -EBUSY;
		dev_info(&pdev->dev, "%s: err_line\n", __func__);
		goto err_line;
	}

	unp = kzalloc(sizeof(struct uart_ns9360_port), GFP_KERNEL);
	ns9360_uart_ports[line] = unp;

	if (unp == NULL) {
		ret = -ENOMEM;
		dev_info(&pdev->dev, "%s: err_alloc\n", __func__);
		goto err_alloc;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENOENT;
		dev_info(&pdev->dev, "%s: err_get_irq\n", __func__);
		goto err_get_irq;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		dev_info(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	base = ioremap(mem->start, 0x3f);
	if (!base) {
		ret = -ENOMEM;
		dev_info(&pdev->dev, "%s: err_ioremap\n", __func__);
		goto err_ioremap;
	}
	dev_info(&pdev->dev, "%s: membase = %p, unp = %p\n",
			__func__, base, unp);

	unp->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(unp->clk)) {
		ret = PTR_ERR(unp->clk);
		dev_info(&pdev->dev, "%s: err_clk_get -> %d\n", __func__, ret);
		goto err_clk_get;
	}

	unp->port.dev = &pdev->dev;
	unp->port.mapbase = mem->start;
	unp->port.membase = base;
	unp->port.iotype = UPIO_MEM;
	unp->port.irq = irq;
	unp->port.fifosize = 64; /* XXX */
	unp->port.ops = &ns9360_uart_pops;
	unp->port.flags = UPF_BOOT_AUTOCONF;
	unp->port.line = line;

	ret = uart_add_one_port(&ns9360_uart_reg, &unp->port);
	if (ret) {
		dev_info(&pdev->dev, "%s: err_uart_add1port -> %d\n",
				__func__, ret);
		goto err_uart_add1port;
	}

	return 0;

err_uart_add1port:
	clk_put(unp->clk);
err_clk_get:
	iounmap(base);
err_ioremap:
	release_resource(mem);
err_get_mem:
err_get_irq:
	kfree(unp);
	ns9360_uart_ports[line] = NULL;
err_alloc:
err_line:
	return ret;
}

static __devexit int ns9360_uart_pdrv_remove(struct platform_device *pdev)
{
	int line;
	struct uart_ns9360_port *unp = platform_get_drvdata(pdev);

	line = unp->port.line;

	uart_remove_one_port(&ns9360_uart_reg, &unp->port);
	clk_put(unp->clk);
	iounmap(unp->port.membase);
	kfree(unp);
	ns9360_uart_ports[line] = NULL;

	return 0;
}

static struct platform_driver ns9360_uart_pdriver = {
	.probe = ns9360_uart_pdrv_probe,
	.remove = __devexit_p(ns9360_uart_pdrv_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ns9360_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&ns9360_uart_reg);
	if (ret) {
		pr_debug("%s: unable to register uart driver\n", __func__);
		goto err_uart_register_driver;
	}

	ret = platform_driver_register(&ns9360_uart_pdriver);
	if (ret) {
		pr_debug("%s: unable to register platform driver\n", __func__);
		goto err_platform_driver_register;
	}

	pr_info("Digi NS9360 UART driver\n");

	return 0;

err_platform_driver_register:
	uart_unregister_driver(&ns9360_uart_reg);
err_uart_register_driver:
	return ret;
}

static void __exit ns9360_uart_exit(void)
{
	platform_driver_unregister(&ns9360_uart_pdriver);
	uart_unregister_driver(&ns9360_uart_reg);
}

module_init(ns9360_uart_init);
module_exit(ns9360_uart_exit);

MODULE_DESCRIPTION("Digi NS9360 UART driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
