/*
 * drivers/serial/ns921x-serial.c
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#if defined(CONFIG_SERIAL_NS921X_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>

#include <mach/ns921x-serial.h>
#include <mach/gpio.h>

#define HUB_IFS		0x0000
#define HUB_IFS_RXFSRIP		(1 << 25)
#define HUB_IFS_TXFSRIP		(1 << 19)
#define HUB_IFS_MODIP		(1 << 18)
#define HUB_IFS_RXFE		(1 << 13)
#define HUB_IFS_TXFF		(1 << 11)
#define HUB_IFS_TXFE		(1 << 10)

#define HUB_DMARXCTRL	0x0004
#define HUB_DMARXCTRL_DIRECT	(1 << 28)

#define HUB_RXIC	0x000c
#define HUB_RXIC_RXTHRS		(15 << 28)
#define HUB_RXIC_RXFSRIE	(1 << 25)

#define HUB_DMRXSF	0x0010
#define HUB_DMRXSF_BYTE		(7 << 9)
#define HUB_DMRXSF_FULL		(1 << 7)

#define HUB_DMRXDF	0x0014

#define HUB_DMATXCTRL	0x0018
#define HUB_DMATXCTRL_DIRECT	(1 << 28)

#define HUB_TXIC	0x0020
#define HUB_TXIC_TXTHRS		(15 << 28)
#define HUB_TXIC_TXFUFIE	(1 << 26)
#define HUB_TXIC_TXFSRIE	(1 << 25)

#define HUB_DMTXDF	0x0028

#define UART_WC		0x1000
#define UART_WC_RXEN		(1 << 30)
#define UART_WC_TXEN		(1 << 29)
#define UART_WC_RTSEN		(1 << 19)
#define UART_WC_RXFLUSH		(1 << 17)
#define UART_WC_TXFLUSH		(1 << 16)
#define UART_WC_TXFLOW_CHAR	(1 << 11)
#define UART_WC_TXFLOW_SOFT	(1 << 10)
#define UART_WC_TXFLOW_RI	(1 << 9)
#define UART_WC_TXFLOW_DSR	(1 << 8)
#define UART_WC_TXFLOW_DCD	(1 << 7)
#define UART_WC_TXFLOW_CTS	(1 << 6)
#define UART_WC_TXFLOW	(UART_WC_TXFLOW_RI | UART_WC_TXFLOW_DSR | \
			 UART_WC_TXFLOW_DCD | UART_WC_TXFLOW_CTS)

#define UART_IE		0x1004
#define UART_IE_OFLOW		(1 << 19)
#define UART_IE_PARITY		(1 << 18)
#define UART_IE_FRAME		(1 << 17)
#define UART_IE_BREAK		(1 << 16)
#define UART_IE_DSR		(1 <<  7)
#define UART_IE_DCD		(1 <<  6)
#define UART_IE_CTS		(1 <<  5)
#define UART_IE_RI		(1 <<  4)
#define UART_IE_TBC		(1 <<  3)
#define UART_IE_RBC		(1 <<  2)
#define UART_IE_TXIDLE		(1 <<  1)

#define UART_IS		0x1008
#define UART_IS_OFLOW		(1 << 19)
#define UART_IS_PARITY		(1 << 18)
#define UART_IS_FRAME		(1 << 17)
#define UART_IS_BREAK		(1 << 16)
#define UART_IS_DSR		(1 <<  7)
#define UART_IS_DCD		(1 <<  6)
#define UART_IS_CTS		(1 <<  5)
#define UART_IS_RI		(1 <<  4)
#define UART_IS_TBC		(1 <<  3)
#define UART_IS_RBC		(1 <<  2)
#define UART_IS_TXIDLE		(1 <<  1)

#define UIE_RX (UART_IE_OFLOW | UART_IE_PARITY | UART_IE_FRAME | \
		UART_IE_BREAK | UART_IE_RBC)
#define UIS_RX (UART_IS_OFLOW | UART_IS_PARITY | UART_IS_FRAME | \
		UART_IS_BREAK | UART_IS_RBC)
#define UIE_MS (UART_IE_DSR | UART_IE_DCD | UART_IE_CTS | UART_IE_RI)
#define UIS_MS (UART_IS_DSR | UART_IS_DCD | UART_IS_CTS | UART_IS_RI)

#define UART_CGAPCTRL	0x100c
#define UART_CGAPCTRL_EN	(1 << 31)

#define UART_BGAPCTRL	0x1010
#define UART_BGAPCTRL_EN	(1 << 31)

#define UART_RCMC0	0x1014
#define UART_RCMC0_ENABLE	(1 << 31)
#define UART_RCMC0_MASK		(0xff << 16)
#define UART_RCMC0_DATA		(0xff << 0)

#define UART_AWC	0x1030
#define UART_AWC_ENABLE		(1 << 0)

#define UART_BRDL       0x1100	/* DLAB = 1 */

#define UART_UIE	0x1104	/* DLAB = 0 */
#define UART_UIE_ETBEI		(1 << 1)

#define UART_BRDM	0x1104	/* DLAB = 1 */

#define UART_FCR	0x1108
#define UART_FCR_TXCLR		(1 << 2)
#define UART_FCR_RXCLR		(1 << 1)
#define UART_FCR_FIFOEN		(1 << 0)

#define UART_LCR	0x110c
#define UART_LCR_DLAB		(1 << 7)
#define UART_LCR_SBC		(1 << 6)
#define UART_LCR_SPAR		(1 << 5)
#define UART_LCR_EPAR		(1 << 4)
#define UART_LCR_PARITY		(1 << 3)
#define UART_LCR_STOP		(1 << 2)
#define UART_LCR_WLEN		0x0003
#define UART_LCR_WLEN_5		0x0000
#define UART_LCR_WLEN_6		0x0001
#define UART_LCR_WLEN_7		0x0002
#define UART_LCR_WLEN_8		0x0003

#define UART_MCR	0x1110
#define UART_MCR_AFE		(1 << 5)
#define UART_MCR_LOOP		(1 << 4)
#define UART_MCR_RTS		(1 << 1)
#define UART_MCR_DTR		(1 << 0)

#define UART_LSR	0x1114
#define UART_LSR_TEMT		(1 << 6)

#define UART_MSR	0x1118
#define UART_MSR_DCD		(1 << 7)
#define UART_MSR_RI		(1 << 6)
#define UART_MSR_DSR		(1 << 5)
#define UART_MSR_CTS		(1 << 4)
#define UART_MSR_DDCD		(1 << 3)
#define UART_MSR_TERI		(1 << 2)
#define UART_MSR_DDSR		(1 << 1)
#define UART_MSR_DCTS		(1 << 0)

#define UMSR_ANYDELTA		(UART_MSR_DDCD | UART_MSR_TERI | \
		UART_MSR_DDSR | UART_MSR_DCTS)

#define DRIVER_NAME "ns921x-serial"
#define NS921X_TTY_NAME "ttyNS"
#define NS921X_TTY_MAJOR 204 /* XXX */
#define NS921X_TTY_MINOR_START 196 /* XXX */

#define NS921X_UART_NR 4

#define FIFOSIZE 16
#define TXTHRESHOLD 1
#define RXTHRESHOLD 5

#define up2unp(up) container_of(up, struct uart_ns921x_port, port)
struct uart_ns921x_port {
	struct uart_port port;
	struct clk *clk;
#define NSUPF_TXIRQPENDING 1
#define NSUPF_SCHEDSTOPTX 2
	unsigned int flags;
	u32 ifs2ack;
	u32 is2ack;
	struct ns921x_uart_data *data;
};

#if defined(CONFIG_DEBUG_LL) && 0
void printch(char);
void printhex2(unsigned);
void printhex4(unsigned);
void printhex8(unsigned);
void printascii(const char *);
#else
#define printch(c) ((void)0)
#define printhex2(u) ((void)0)
#define printhex4(u) ((void)0)
#define printhex8(u) ((void)0)
#define printascii(s) ((void)0)
#endif

static inline u32 uartread32(struct uart_port *port, unsigned int offset)
{
	u32 ret = ioread32(port->membase + offset);

#if defined(DEBUG_UARTRW)
	dev_dbg(port->dev, "read  0x%p -> 0x%08x\n",
			port->membase + offset, ret);
#endif

	return ret;
}

static inline void uartwrite32(struct uart_port *port,
		u32 value, unsigned int offset)
{
#if defined(DEBUG_UARTRW)
	dev_dbg(port->dev, "write 0x%p <- 0x%08x\n",
			port->membase + offset, value);
#endif
	iowrite32(value, port->membase + offset);
}

static inline void uartwrite8(struct uart_port *port, u8 value,
		unsigned int offset)
{
#if defined(DEBUG_UARTRW)
	dev_dbg(port->dev, "write 0x%p <- 0x%02x\n",
			port->membase + offset, value);
#endif
	iowrite8(value, port->membase + offset);
}

/*
 * bits 19 to 31 of HUB_IFS need to be cleard by writing a 1 to it.
 * ns921x_uart_read_ifs and ns921x_uart_clear_ifs help debugging missed clears
 * by tracking the bits set
 */
#define IFS_BITSTOACK 0xfff80000
static inline u32 ns921x_uart_read_ifs(struct uart_ns921x_port *unp)
{
	u32 ret = uartread32(&unp->port, HUB_IFS);
	if (unp->ifs2ack)
		dev_dbg(unp->port.dev, "%s: %s still unacked: %x "
				"(called from %p)\n", __func__, "IFS",
				unp->ifs2ack, __builtin_return_address(0));
	unp->ifs2ack |= ret & IFS_BITSTOACK;

	if (unp->ifs2ack & ~ret)
		dev_dbg(unp->port.dev, "%s: rereading %s doesn't "
				"yield unacked bits (called from %p): "
				"%08x %08x\n", __func__, "IFS",
				__builtin_return_address(0), unp->ifs2ack, ret);

	return ret;
}

static inline void ns921x_uart_clear_ifs(struct uart_ns921x_port *unp,
		unsigned int mask)
{
	if (mask & ~unp->ifs2ack) {
		dev_dbg(unp->port.dev, "%s: unp->%s = %08x, mask = %08x\n",
				__func__, "ifs2ack", unp->ifs2ack, mask);
		BUG();
	}
	BUG_ON(mask & ~IFS_BITSTOACK);
	unp->ifs2ack &= ~mask;
	uartwrite32(&unp->port, mask, HUB_IFS);
}

/*
 * all bits of UART_IS need to be cleard by writing a 1 to it.
 * ns921x_uart_read_is and ns921x_uart_clear_is help debugging missed clears by
 * tracking the bits set
 */
static inline u32 ns921x_uart_read_is(struct uart_ns921x_port *unp)
{
	u32 ret = uartread32(&unp->port, UART_IS);
	if (unp->is2ack)
		dev_dbg(unp->port.dev, "%s: %s still unacked: %x "
				"(called from %p)\n", __func__, "IS",
				unp->is2ack, __builtin_return_address(0));
	unp->is2ack |= ret;

	if (unp->is2ack & ~ret)
		dev_dbg(unp->port.dev, "%s: rereading %s doesn't "
				"yield unacked bits (called from %p): "
				"%08x %08x\n", __func__, "IS",
				__builtin_return_address(0), unp->is2ack, ret);

	return ret;
}

static inline void ns921x_uart_clear_is(struct uart_ns921x_port *unp,
		u32 mask)
{
	if (mask & ~unp->is2ack) {
		dev_dbg(unp->port.dev, "%s: unp->%s = %08x, mask = %08x\n",
				__func__, "is2ack", unp->is2ack, mask);
		BUG();
	}
	unp->is2ack &= ~mask;
	uartwrite32(&unp->port, mask, UART_IS);
}

static inline void ns921x_uart_rmw_uartie(struct uart_port *port,
		u32 mask, u32 value)
{
	u32 ie = uartread32(port, UART_IE);

	BUG_ON((value & mask) != value);

	ie = (ie & ~mask) | value;

	uartwrite32(port, ie, UART_IE);
}

static unsigned int ns921x_uart_tx_empty(struct uart_port *port)
{
	struct uart_ns921x_port *unp = up2unp(port);
	/* Should I use UART_IS_TXIDLE? HUB_IFS_TXFE? */

	u32 lsr;

#if defined(NSUPF_TXIRQPENDING)
	if (unp->flags & NSUPF_TXIRQPENDING)
		return 0;
#endif

	lsr = uartread32(&unp->port, UART_LSR);

	if (!(lsr & UART_LSR_TEMT))
		return 0;

	return TIOCSER_TEMT;
}

static void ns921x_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 mcr = 0;

	/* RTS signal should be controlled as gpio to allow using AFE */
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;

	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;

	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	uartwrite32(port, mcr, UART_MCR);
}


/*
 * Depending on the configuration of the HW flow control, the CTS line is
 * not be passed to the uart, making impossible to read the status of some
 * modem lines through the modem status register.
 * Following function workarounds that by reading the CTS line as gpio.
 */
static unsigned int ns921x_uart_check_msr(struct uart_port *port, u32 irqstat)
{
	struct uart_ns921x_port *unp = up2unp(port);
	u32 status, cts_gpio;

	/* We can read the status of the modem lines through the
	 * modem status register, but we need to add the value of
	 * CTS line from the gpio value */
	status = uartread32(port, UART_MSR);
	cts_gpio = unp->data->gpios[2];
	status |= (gpio_get_value(cts_gpio) != 0) ? 0 : UART_MSR_CTS;

	/* For event detection on the modem lines, we use the interrupt status
	 * register instead of the delta section of the modem status register.
	 * The interrupt flags are valid in all the cases */
	if ((irqstat & (UART_IS_DSR|UART_IS_DCD|UART_IS_CTS|UART_IS_RI)) &&
	     port->state != NULL) {
		if (irqstat & UART_IS_RI)
			port->icount.rng++;
		if (irqstat & UART_IS_DSR)
			port->icount.dsr++;
		if (irqstat & UART_IS_DCD)
			uart_handle_dcd_change(port, status & UART_MSR_DCD);
		if (irqstat & UART_IS_CTS)
			uart_handle_cts_change(port, status & UART_MSR_CTS);

		wake_up_interruptible(&port->state->port.delta_msr_wait);
	}

	return status;
}

static unsigned int ns921x_uart_get_mctrl(struct uart_port *port)
{
	unsigned int status;
	unsigned int ret = 0;

	status = ns921x_uart_check_msr(port, 0);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

static void ns921x_uart_stop_tx_real(struct uart_port *port)
{
	struct uart_ns921x_port *unp = up2unp(port);

	u32 txic = uartread32(&unp->port, HUB_TXIC);

	printch('.');

	ns921x_uart_rmw_uartie(&unp->port, UART_IE_TXIDLE, 0);

	uartwrite32(port, txic & ~HUB_TXIC_TXFSRIE, HUB_TXIC);

#if defined(NSUPF_TXIRQPENDING)
	unp->flags &= ~NSUPF_TXIRQPENDING;
#endif
}

/* called with port->lock taken */
static void ns921x_uart_stop_tx(struct uart_port *port)
{
#if defined(NSUPF_TXIRQPENDING) && defined(NSUPF_SCHEDSTOPTX)
	struct uart_ns921x_port *unp = up2unp(port);

	/* don't stop the port if there is a tx irq pending */
	if (unp->flags & NSUPF_TXIRQPENDING) {
		unp->flags |= NSUPF_SCHEDSTOPTX;
	} else {
		ns921x_uart_stop_tx_real(port);

		unp->flags &= ~NSUPF_SCHEDSTOPTX;
	}
#else
	ns921x_uart_stop_tx_real(port);
#endif
}

/* send out chars in xmit buffer.  This is called with port->lock taken */
static void ns921x_uart_tx_chars(struct uart_ns921x_port *unp,
		unsigned int freebuffers)
{
	struct circ_buf *xmit = &unp->port.state->xmit;
	unsigned long ifs;

	BUG_ON(!freebuffers);

	assert_spin_locked(&unp->port.lock);

	/*
	 * If the FIFO is not empty return at this point, then the
	 * interrupt-handler will recall this function
	 */
	ifs = ns921x_uart_read_ifs(unp);
	if (!(ifs & HUB_IFS_TXFE))
		return;

	if (unp->port.x_char) {
		uartwrite8(&unp->port, unp->port.x_char, HUB_DMTXDF);
		unp->port.icount.tx++;
		unp->port.x_char = 0;
		freebuffers--;
#if defined(NSUPF_TXIRQPENDING)
		unp->flags |= NSUPF_TXIRQPENDING;
#endif
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&unp->port)) {
		ns921x_uart_stop_tx(&unp->port);
		return;
	}

	printch('T');
	printhex4(uart_circ_chars_pending(xmit));
	while (freebuffers && uart_circ_chars_pending(xmit)) {
		if (uart_circ_chars_pending(xmit) >= 4) {
			u32 fourchars = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			fourchars |= xmit->buf[xmit->tail] << 8;
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			fourchars |= xmit->buf[xmit->tail] << 16;
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			fourchars |= xmit->buf[xmit->tail] << 24;
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

			uartwrite32(&unp->port, fourchars, HUB_DMTXDF);
			printhex8(cpu_to_be32(fourchars));
			unp->port.icount.tx += 4;
		} else {
			uartwrite8(&unp->port,
					xmit->buf[xmit->tail], HUB_DMTXDF);
			printhex2(xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			unp->port.icount.tx++;
		}
#if defined(NSUPF_TXIRQPENDING)
		unp->flags |= NSUPF_TXIRQPENDING;
#endif
		--freebuffers;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&unp->port);

	if (uart_circ_empty(xmit))
		ns921x_uart_stop_tx(&unp->port);
}

#define maskshiftfac(mask)		((mask) & (-(mask)))
#define im2mask(val, mask)						\
	(((val) * maskshiftfac(mask)) & (mask))

/* called with port->lock taken */
static void ns921x_uart_start_tx(struct uart_port *port)
{
	struct uart_ns921x_port *unp = up2unp(port);

	u32 txic = uartread32(port, HUB_TXIC);

	uartwrite32(port, txic | im2mask(TXTHRESHOLD, HUB_TXIC_TXTHRS) |
			HUB_TXIC_TXFSRIE, HUB_TXIC);
	ns921x_uart_rmw_uartie(&unp->port, UART_IE_TXIDLE, UART_IE_TXIDLE);

#if defined(NSUPF_SCHEDSTOPTX)
	unp->flags &= ~NSUPF_SCHEDSTOPTX;
#endif

	if (
#if defined(NSUPF_TXIRQPENDING)
			!(unp->flags & NSUPF_TXIRQPENDING) &&
#endif
			!(ns921x_uart_read_ifs(unp) & HUB_IFS_TXFF)) {
		unsigned int freebuffers = 1;
		ns921x_uart_tx_chars(up2unp(port), freebuffers);
	}
}

static void ns921x_uart_stop_rx(struct uart_port *port)
{
	dev_vdbg(port->dev, "%s\n", __func__);

	ns921x_uart_rmw_uartie(port, UIE_RX, 0);
}

static void ns921x_uart_enable_ms(struct uart_port *port)
{
	dev_vdbg(port->dev, "%s\n", __func__);

	ns921x_uart_rmw_uartie(port, UIE_MS, UIE_MS);
}

static void ns921x_uart_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	u32 lcr;

	dev_vdbg(port->dev, "%s\n", __func__);

	spin_lock_irqsave(&port->lock, flags);

	lcr = uartread32(port, UART_LCR);

	/* XXX: amba_pl011_break_ctl tests for break_state being -1,
	 * Documentation/serial/driver tells to tests for != 0 */
	if (break_state)
		lcr |= UART_LCR_SBC;
	else
		lcr &= ~UART_LCR_SBC;

	uartwrite32(port, lcr, UART_LCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void ns921x_uart_rx_char(struct uart_ns921x_port *unp,
		unsigned int ch, u32 is)
{
	unsigned int flag = TTY_NORMAL;

	unp->port.icount.rx++;
#define ISERR	(UART_IS_BREAK | UART_IS_PARITY | UART_IS_FRAME | UART_IS_OFLOW)

	dev_vdbg(unp->port.dev, "%s: IS=%x\n", __func__, is);
	if (unlikely(is & ISERR)) {
		if (is & UART_IS_BREAK) {
			dev_vdbg(unp->port.dev, "break, IS=%x\n", is);
			unp->port.icount.brk++;
			if (uart_handle_break(&unp->port))
				return;
		} else if (is & UART_IS_PARITY)
			unp->port.icount.parity++;
		else if (is & UART_IS_FRAME)
			unp->port.icount.frame++;

		if (is & UART_IS_OFLOW)
			unp->port.icount.overrun++;

		is &= unp->port.read_status_mask;

		if (is & UART_IS_BREAK)
			flag = TTY_BREAK;
		else if (is & UART_IS_PARITY)
			flag = TTY_PARITY;
		else if (is & UART_IS_FRAME)
			flag = TTY_FRAME;
	}

	if (uart_handle_sysrq_char(&unp->port, ch))
		return;

	dev_vdbg(unp->port.dev, "%s: insert %x (IS=%x, ism=%x, flag=%x)\n",
			__func__, ch, is, unp->port.ignore_status_mask, flag);
	uart_insert_char(&unp->port, is, UART_IS_OFLOW, ch, flag);
}

/* This is called with port->lock taken */
static void ns921x_uart_rx_chars(struct uart_ns921x_port *unp,
		u32 ifs, u32 is)
{
	struct tty_struct *tty = unp->port.state->port.tty;

	if (is & (UART_IS_RBC | UIS_RX))
		ns921x_uart_clear_is(unp, is & (UART_IS_RBC | UIS_RX));

	if (ifs & HUB_IFS_RXFE) {
		if (is & (UART_IS_BREAK | UART_IS_OFLOW))
			ns921x_uart_rx_char(unp, 0, is);
	}

	while (!(ifs & HUB_IFS_RXFE)) {
		u32 dmrxsf = uartread32(&unp->port, HUB_DMRXSF);
		u32 dmrxdf = uartread32(&unp->port, HUB_DMRXDF);
		/* XXX: better put this into a macro */
		unsigned int bytes = (dmrxsf & HUB_DMRXSF_BYTE) >> 9;
		int i;

		dev_vdbg(unp->port.dev, "%s: DMRXSF = %x, DMRXDF = %x\n",
				__func__, dmrxsf, dmrxdf);

		BUG_ON(bytes > 4);

		for (i = 0; i < bytes; ++i) {
			unsigned int ch = (dmrxdf >> (8 * i)) & 0xff;

			/*
			 * assume break and errors only apply to the last
			 * character.
			 */
			ns921x_uart_rx_char(unp, ch, i == bytes - 1 ? is : 0);
		}
		ifs = ns921x_uart_read_ifs(unp);
	}
	spin_unlock(&unp->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&unp->port.lock);
}

static irqreturn_t ns921x_uart_int(int irq, void *dev_id)
{
	struct uart_ns921x_port *unp = dev_id;
	u32 ifs, uninitialized_var(is);

	spin_lock(&unp->port.lock);

	ifs = ns921x_uart_read_ifs(unp);

	if (ifs & HUB_IFS_TXFSRIP) {
		ns921x_uart_clear_ifs(unp, HUB_IFS_TXFSRIP);

		/* don't clear NSUPF_TXIRQPENDING, we still wait for TXdone */

		ns921x_uart_tx_chars(unp, FIFOSIZE - TXTHRESHOLD - 1);
	}

	if ((ifs & HUB_IFS_RXFSRIP) || (ifs & HUB_IFS_MODIP))
		is = ns921x_uart_read_is(unp);

	if (ifs & HUB_IFS_RXFSRIP) {
		ns921x_uart_clear_ifs(unp, HUB_IFS_RXFSRIP);

		ns921x_uart_rx_chars(unp, ifs, is);
	}

	if (ifs & HUB_IFS_MODIP) {
		dev_vdbg(unp->port.dev, "%s: IS=%x\n", __func__, is);

		/* test for HUB_IFS_RXFSRIP not being set to only fetch
		 * characters once.
		 */
		if (is & UIS_RX && !(ifs & HUB_IFS_RXFSRIP))
			ns921x_uart_rx_chars(unp, ifs, is);

		if (is & UIS_MS)
			ns921x_uart_clear_is(unp, is & UIS_MS);

		/* XXX: call this only after an MS irq? */
		ns921x_uart_check_msr(&unp->port, is);

		if (is & UART_IS_TXIDLE) {
			unsigned int freebuffers = TXTHRESHOLD;
#if defined(NSUPF_TXIRQPENDING)
			unp->flags &= ~NSUPF_TXIRQPENDING;
#endif

			ns921x_uart_clear_is(unp, UART_IS_TXIDLE);

			/* The fifo might not be empty.  The following can
			 * happen:
			 * - irq for HUB_IFS_TXFSRIP serviced without
			 *   UART_IS_TXIDLE
			 * - before new data is written the fifo becomes empty
			 *   and UART_IS_TXIDLE is pending
			 * - UART_IS_TXIDLE is serviced with fifo still having
			 *   some data from servicing UART_IS_TXIDLE.
			 *
			 * So we only write TXTHRESHOLD buffers.  Only if the
			 * fifo is reported to be empty and we didn't wrote
			 * something above, the full fifo is used.
			 */
			if ((ifs & (HUB_IFS_TXFE | HUB_IFS_TXFSRIP)) ==
					HUB_IFS_TXFE)
				freebuffers = FIFOSIZE;

			ns921x_uart_tx_chars(unp, freebuffers);
		}

		if (unp->is2ack)
			dev_dbg(unp->port.dev,
					"%s: unacked flags in UART_IS: %x %x\n",
					__func__, unp->is2ack, is);
	}

	if (unp->ifs2ack)
		dev_dbg(unp->port.dev, "%s: unacked flags in HUB_IFS: %x\n",
				__func__, unp->ifs2ack);

#if defined(NSUPF_SCHEDSTOPTX) && defined(NSUPF_TXIRQPENDING)
	if ((unp->flags & (NSUPF_TXIRQPENDING | NSUPF_SCHEDSTOPTX)) ==
			NSUPF_SCHEDSTOPTX) {
		ns921x_uart_stop_tx_real(&unp->port);
		unp->flags &= ~NSUPF_SCHEDSTOPTX;
	}
#endif

	spin_unlock(&unp->port.lock);

	return IRQ_HANDLED;
}

static int ns921x_uart_startup(struct uart_port *port)
{
	struct uart_ns921x_port *unp = up2unp(port);
	int ret;
	u32 rxic;
	u32 wc;

	unp->flags = 0;

	ret = clk_enable(unp->clk);
	if (ret) {
		dev_dbg(port->dev, "%s: err_clkenable", __func__);
		goto err_clkenable;
	}

	unp->port.uartclk = clk_get_rate(unp->clk);

	ret = request_irq(unp->port.irq, ns921x_uart_int, 0, DRIVER_NAME, unp);
	if (ret) {
		dev_dbg(port->dev, "%s: err_request_irq", __func__);

		clk_disable(unp->clk);
err_clkenable:

		return ret;
	}

	ns921x_uart_rmw_uartie(port, UIE_RX, UIE_RX);

	rxic = uartread32(port, HUB_RXIC);
	uartwrite32(port, rxic | im2mask(RXTHRESHOLD, HUB_RXIC_RXTHRS)
			| HUB_RXIC_RXFSRIE, HUB_RXIC);

	wc = UART_WC_RXEN | UART_WC_TXEN;
	if (unp->data->rtsen)
		wc |= UART_WC_RTSEN;
	uartwrite32(port, wc, UART_WC);
	uartwrite32(&unp->port, UART_FCR_FIFOEN, UART_FCR);

	return 0;
}

static void ns921x_uart_shutdown(struct uart_port *port)
{
	struct uart_ns921x_port *unp = up2unp(port);

	/* wait up to 10ms for the fifo to empty */
	unsigned long expire = jiffies + msecs_to_jiffies(10);

	printch('\\');

	while (!ns921x_uart_tx_empty(port)) {
		msleep(1);
		if (time_after(jiffies, expire))
			break;
	}

	ns921x_uart_rmw_uartie(port, -1, 0);
	uartwrite32(port, 0, UART_WC);

	ns921x_uart_break_ctl(port, 0);

	free_irq(port->irq, unp);

	clk_disable(unp->clk);
}

static void ns921x_uart_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	unsigned long flags;
	u32 cval = 0;

	unsigned int baud, quot;
	u32 saved_wc;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN_5;
		break;
	case CS6:
		cval = UART_LCR_WLEN_6;
		break;
	case CS7:
		cval = UART_LCR_WLEN_7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN_8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;

	if (termios->c_cflag & PARENB) {
		cval |= UART_LCR_PARITY;

		if (!(termios->c_cflag & PARODD))
			cval |= UART_LCR_EPAR;
#ifdef CMSPAR
		/* stick parity:
		 *   - MARK parity requires flags PARENB | CMSPAR | PARODD
		 *     and in the controller the EPS and PEN bitfields
		 *     must be set to 0
		 *   - SPACE parity requires flags PARENB | CMSPAR | ~PARODD
		 *     and in the controller the EPS and PEN bitfields
		 *     must be set to 1 (these are already set by the
		 *     code above this, so there is nothing to do here)
		 *   In any case SP bitfield must be set to 1
		 */
		if(termios->c_cflag & CMSPAR) {
			cval |= UART_LCR_SPAR;	/* set stick parity */
			if (termios->c_cflag & PARODD) {
				cval &= ~UART_LCR_EPAR;
				cval &= ~UART_LCR_PARITY;
			}
		}
#endif
	}

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_IS_OFLOW;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_IS_FRAME | UART_IS_PARITY;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_IS_BREAK;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_IS_FRAME | UART_IS_PARITY;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_IS_BREAK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_IS_OFLOW;
	}

	/* disable everything, prepare fifo flushing */
	saved_wc = uartread32(port, UART_WC);
	uartwrite32(port, UART_WC_RXFLUSH | UART_WC_TXFLUSH, UART_WC);

	if (UART_ENABLE_MS(port, termios->c_cflag))
		ns921x_uart_enable_ms(port);

	/* set character gap period */
	uartwrite32(port, UART_CGAPCTRL_EN | (10 * (port->uartclk / baud) - 1),
			UART_CGAPCTRL);

	/* set buffer gap period */
	uartwrite32(port, UART_BGAPCTRL_EN | (640 * (port->uartclk / baud) - 1),
			UART_BGAPCTRL);

	/* prepare to access baud rate registers */
	uartwrite32(port, UART_LCR_DLAB, UART_LCR);

	/* set baud rate */
	uartwrite32(port, quot & 0xff, UART_BRDL);
	uartwrite32(port, (quot >> 8) & 0xff, UART_BRDM);

	udelay(1);
	uartwrite32(port, cval, UART_LCR);

	/* flush fifos and restore state */
	uartwrite32(port, saved_wc, UART_WC);
	uartwrite32(port, UART_UIE_ETBEI, UART_UIE);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ns921x_uart_type(struct uart_port *port)
{
	struct platform_device *pdev;

	if (PORT_NS921X == port->type) {
		pdev = to_platform_device(port->dev);
		if (0 == pdev->id)
			return "NS921X PORT A";
		else if (1 == pdev->id)
			return "NS921X PORT B";
		else if (2 == pdev->id)
			return "NS921X PORT C";
		else if (3 == pdev->id)
			return "NS921X PORT D";
		else
			return NULL;
	}
	else
		return NULL;
}

static void ns921x_uart_release_port(struct uart_port *port)
{
	/* XXX: release_mem_region is marked as Compatibility cruft ??? */
	release_mem_region(port->mapbase, 0x8000);
}

static int ns921x_uart_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase,
			0x8000, DRIVER_NAME) ? 0 : -EBUSY;
}

static void ns921x_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_NS921X;
		ns921x_uart_request_port(port);
	}
}

static int ns921x_uart_verify_port(struct uart_port *port,
		struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NS921X)
		ret = -EINVAL;

	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;

	if (ser->baud_base < 9600)
		ret = -EINVAL;

	return ret;
}

static struct uart_ops ns921x_uart_pops = {
	.tx_empty = ns921x_uart_tx_empty,
	.set_mctrl = ns921x_uart_set_mctrl,
	.get_mctrl = ns921x_uart_get_mctrl,
	.stop_tx = ns921x_uart_stop_tx,
	.start_tx = ns921x_uart_start_tx,
	.stop_rx = ns921x_uart_stop_rx,
	.enable_ms = ns921x_uart_enable_ms,
	.break_ctl = ns921x_uart_break_ctl,
	.startup = ns921x_uart_startup,
	.shutdown = ns921x_uart_shutdown,
	.set_termios = ns921x_uart_set_termios,
	/* .pm = ns921x_uart_pm, */
	/* .set_wake = ns921x_uart_set_wake, */
	.type = ns921x_uart_type,
	.release_port = ns921x_uart_release_port,
	.request_port = ns921x_uart_request_port,
	.config_port = ns921x_uart_config_port,
	.verify_port = ns921x_uart_verify_port,
	/* .ioctl */
};

static struct uart_ns921x_port *ns921x_uart_ports[NS921X_UART_NR];

#if defined(CONFIG_SERIAL_NS921X_CONSOLE)

static void ns921x_uart_console_putchar(struct uart_port *port, int ch)
{
	struct uart_ns921x_port *unp = up2unp(port);

	while (ns921x_uart_read_ifs(unp) & HUB_IFS_TXFF)
		barrier();

	uartwrite8(&unp->port, ch, HUB_DMTXDF);
}

/* called with console_sem hold.  irqs locally disabled */
static void ns921x_uart_console_write(struct console *co,
		const char *s, unsigned int count)
{
	struct uart_ns921x_port *unp = ns921x_uart_ports[co->index];
	u32 saved_txic, saved_wc, saved_uie;
	u32 new_wc;

	BUG_ON(!irqs_disabled());

	saved_txic = uartread32(&unp->port, HUB_TXIC);
	saved_wc = uartread32(&unp->port, UART_WC);
	saved_uie = uartread32(&unp->port, UART_UIE);

	new_wc = saved_wc | UART_WC_RXEN | UART_WC_TXEN;
	new_wc &= ~(UART_WC_RXFLUSH | UART_WC_TXFLUSH);

	/* XXX: assert baud, bits, parity etc. to be correct. */

	uartwrite32(&unp->port, 0, HUB_TXIC);
	uartwrite32(&unp->port, UART_FCR_FIFOEN, UART_FCR);
	uartwrite32(&unp->port, UART_UIE_ETBEI, UART_UIE);
	uartwrite32(&unp->port, new_wc, UART_WC);

	uart_console_write(&unp->port, s, count, ns921x_uart_console_putchar);

	/* wait for HUB fifo to become empty */
	while (!(ns921x_uart_read_ifs(unp) & HUB_IFS_TXFE))
		barrier();

	while (!(uartread32(&unp->port, UART_LSR) & UART_LSR_TEMT))
		barrier();

	uartwrite32(&unp->port, saved_wc, UART_WC);
	uartwrite32(&unp->port, saved_uie, UART_UIE);
	uartwrite32(&unp->port, saved_txic, HUB_TXIC);
}

static void __init ns921x_uart_console_get_options(struct uart_ns921x_port *unp,
		int *baud, int *parity, int *bits, int *flow)
{
	if (uartread32(&unp->port, UART_WC) & UART_WC_TXEN) {
		u32 cval = uartread32(&unp->port, UART_LCR);
		unsigned int quot = 0;

		uartwrite32(&unp->port, UART_LCR_DLAB, UART_LCR);

		/* XXX: write barrier */

		quot = (uartread32(&unp->port, UART_BRDM) & 0xff) << 8;
		quot |= uartread32(&unp->port, UART_BRDL) & 0xff;

		uartwrite32(&unp->port, cval, UART_LCR);

		*baud = unp->port.uartclk / (16 * quot);

		*parity = 'n';
		if (cval & UART_LCR_PARITY) {
			if (cval & UART_LCR_EPAR)
				*parity = 'e';
			else
				*parity = 'o';
		}
		if (cval & UART_LCR_SPAR) {
			if( (cval & UART_LCR_PARITY) &&
			    (cval & UART_LCR_EPAR) )
				*parity = 'm';
			else
				*parity = 's';
		}

		switch (cval & UART_LCR_WLEN) {
		case UART_LCR_WLEN_5:
			*bits = 5;
			break;
		case UART_LCR_WLEN_6:
			*bits = 6;
			break;
		case UART_LCR_WLEN_7:
			*bits = 7;
			break;
		case UART_LCR_WLEN_8:
			*bits = 8;
			break;
		}

		/* XXX: *flow = ... */
	}
}

static int __init ns921x_uart_console_setup(struct console *co, char *options)
{
	struct uart_ns921x_port *unp;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	if (co->index >= NS921X_UART_NR)
		co->index = 0;
	unp = ns921x_uart_ports[co->index];
	if (!unp)
		return -ENODEV;

	ret = clk_enable(unp->clk);
	if (ret)
		return ret;

	unp->port.uartclk = clk_get_rate(unp->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		ns921x_uart_console_get_options(unp,
				&baud, &parity, &bits, &flow);

	ret = uart_set_options(&unp->port, co, baud, parity, bits, flow);

	/* disable the clock only in the error path, because
	 * otherwise it would need to be enabled and disabled in console_write
	 * where it must not sleep.  But as clk_disable does a gpio_free it
	 * might.
	 */
	if (ret)
		clk_disable(unp->clk);

	return ret;
}

static struct uart_driver ns921x_uart_reg;
static struct console ns921x_uart_console = {
	.name = NS921X_TTY_NAME,
	.write = ns921x_uart_console_write,
	/* .read */
	.device = uart_console_device,
	/* .unblank */
	.setup = ns921x_uart_console_setup,
	/* .early_setup */
	.flags = CON_PRINTBUFFER,
	.index = -1,
	/* .cflag */
	.data = &ns921x_uart_reg,
	/* .next */
};

#define NS921X_UART_CONSOLE (&ns921x_uart_console)
#else
#define NS921X_UART_CONSOLE NULL
#endif

static struct uart_driver ns921x_uart_reg = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = NS921X_TTY_NAME,
	.major = NS921X_TTY_MAJOR,
	.minor = NS921X_TTY_MINOR_START,
	.nr = NS921X_UART_NR,
	.cons = NS921X_UART_CONSOLE,
};

static __devinit int ns921x_uart_pdrv_probe(struct platform_device *pdev)
{
	int ret;
	struct uart_ns921x_port *unp;
	struct resource *mem;
	void __iomem *base;
	int line, irq;

	line = pdev->id;
	if (line < 0 || line >= ARRAY_SIZE(ns921x_uart_ports)) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_line\n", __func__);
		goto err_line;
	}

	if (ns921x_uart_ports[line] != NULL) {
		ret = -EBUSY;
		dev_dbg(&pdev->dev, "%s: err_line\n", __func__);
		goto err_line;
	}

	unp = kzalloc(sizeof(struct uart_ns921x_port), GFP_KERNEL);
	ns921x_uart_ports[line] = unp;

	if (unp == NULL) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc\n", __func__);
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
		dev_dbg(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	base = ioremap(mem->start, 0x8000);
	if (!base) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_ioremap\n", __func__);
		goto err_ioremap;
	}
	dev_dbg(&pdev->dev, "%s: membase = %p, unp = %p\n",
			__func__, base, unp);

	unp->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(unp->clk)) {
		ret = PTR_ERR(unp->clk);
		dev_dbg(&pdev->dev, "%s: err_clk_get -> %d\n", __func__, ret);
		goto err_clk_get;
	}

	unp->data = pdev->dev.platform_data;
	unp->port.dev = &pdev->dev;
	unp->port.mapbase = mem->start;
	unp->port.membase = base;
	unp->port.iotype = UPIO_MEM;
	unp->port.irq = irq;
	unp->port.fifosize = 64; /* XXX */
	unp->port.ops = &ns921x_uart_pops;
	unp->port.flags = UPF_BOOT_AUTOCONF;
	unp->port.line = line;

	/* no DMA */
	uartwrite32(&unp->port, HUB_DMARXCTRL_DIRECT, HUB_DMARXCTRL);
	uartwrite32(&unp->port, HUB_DMATXCTRL_DIRECT, HUB_DMATXCTRL);

	ret = uart_add_one_port(&ns921x_uart_reg, &unp->port);
	if (ret) {

		dev_dbg(&pdev->dev, "%s: err_uart_add1port -> %d\n",
				__func__, ret);

		clk_put(unp->clk);
err_clk_get:

		iounmap(base);
err_ioremap:
err_get_mem:
err_get_irq:

		kfree(unp);
		ns921x_uart_ports[line] = NULL;
err_alloc:

err_line:

		return ret;
	}

	/* Make the device wakeup capable, but disabled by default */
	device_init_wakeup(&pdev->dev, 1);
	device_set_wakeup_enable(&pdev->dev, 0);

	platform_set_drvdata(pdev, unp);

	return 0;
}

static __devexit int ns921x_uart_pdrv_remove(struct platform_device *pdev)
{
	int line;
	struct uart_ns921x_port *unp = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	line = unp->port.line;

	uart_remove_one_port(&ns921x_uart_reg, &unp->port);
	clk_put(unp->clk);
	iounmap(unp->port.membase);
	kfree(unp);
	ns921x_uart_ports[line] = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int ns921x_uart_pdrv_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct uart_ns921x_port *unp = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (device_may_wakeup(&pdev->dev)) {
		ret = enable_irq_wake(unp->port.irq);
		if (ret)
			dev_dbg(&pdev->dev, "%s: err_enable_irq_wake -> %d\n",
					__func__, ret);

		else {
			uartwrite32(&unp->port, UART_RCMC0_ENABLE |
					UART_RCMC0_MASK, UART_RCMC0);
			/* Write a 0 and then enable to clear previous interrupts */
			uartwrite32(&unp->port, 0x0, UART_AWC);
			uartwrite32(&unp->port, UART_AWC_ENABLE, UART_AWC);
		}
	} else
		ret = uart_suspend_port(&ns921x_uart_reg, &unp->port);

	return ret;
}

static int ns921x_uart_pdrv_resume(struct platform_device *pdev)
{
	struct uart_ns921x_port *unp = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (device_may_wakeup(&pdev->dev)) {
		uartwrite32(&unp->port, 0x0, UART_RCMC0);
		ret = disable_irq_wake(unp->port.irq);
	}
	else
		ret = uart_resume_port(&ns921x_uart_reg, &unp->port);

	return ret;
}
#else
#define ns921x_uart_pdrv_suspend	NULL
#define ns921x_uart_pdrv_resume		NULL
#endif

static struct platform_driver ns921x_uart_pdriver = {
	.probe = ns921x_uart_pdrv_probe,
	.remove = __devexit_p(ns921x_uart_pdrv_remove),
	.suspend = ns921x_uart_pdrv_suspend,
	.resume = ns921x_uart_pdrv_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ns921x_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&ns921x_uart_reg);
	if (ret) {
		pr_debug("%s: err_uart_register_driver\n", __func__);
		goto err_uart_register_driver;
	}

	ret = platform_driver_register(&ns921x_uart_pdriver);

	if (ret) {
		pr_debug("%s: err_pdrv_register\n", __func__);

		uart_unregister_driver(&ns921x_uart_reg);
err_uart_register_driver:

		return ret;
	}
	pr_info("Digi NS921x UART driver\n");

	return 0;
}

static void __exit ns921x_uart_exit(void)
{
	platform_driver_unregister(&ns921x_uart_pdriver);
	uart_unregister_driver(&ns921x_uart_reg);
}

module_init(ns921x_uart_init);
module_exit(ns921x_uart_exit);

MODULE_AUTHOR("Uwe Kleine-Koenig");
MODULE_DESCRIPTION("Digi NS921x UART driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
