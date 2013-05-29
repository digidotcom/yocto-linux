/* -*- linux-c -*-
 *
 * drivers/fims/serial/fim_serial.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.1.1.1 $
 *  !Author:     Luis Galdos
 *  !Descr:
 *  !References:
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <asm/delay.h>

#include <mach/gpio.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-iohub-ns921x.h>

/* For registering the FIM-driver */
#include <mach/fim-ns921x.h>

/*
 * If the driver is being compiled as a built-in driver, then include the header file
 * which contains the firmware for this FIM-device
 */
#if !defined(MODULE)
#include "fim_serial.h"
extern const unsigned char fim_serial_firmware[];
#define FIM_SERIAL_FIRMWARE_FILE		(NULL)
#define FIM_SERIAL_FIRMWARE_CODE		fim_serial_firmware
#else
const unsigned char *fim_serial_firmware = NULL;
#define FIM_SERIAL_FIRMWARE_FILE		"fim_serial.bin"
#define FIM_SERIAL_FIRMWARE_CODE		(NULL)
#endif

/* Driver informations */
#define DRIVER_VERSION				"0.2"
#define DRIVER_AUTHOR				"Silvano Najera, Luis Galdos"
#define DRIVER_DESC				"FIM serial driver"
#define FIM_DRIVER_NAME				"fim-serial"
#define FIM_PLATFORM_DRIVER_NAME		"platform:fim-serial"
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Module parameter for selection the FIMs */
NS921X_FIM_NUMBERS_PARAM(fims_number);

/* Firmware-dependent interrupts from the ARM to the FIM */
#define FIM_SERIAL_INT_INSERT_CHAR		0x01
#define FIM_SERIAL_INT_MATCH_CHAR		0x02
#define FIM_SERIAL_INT_BITS_CHAR		0x04
#define FIM_SERIAL_INT_CHAR_GAP 		0x08
#define FIM_SERIAL_INT_BIT_POS			0x10
#define FIM_SERIAL_INT_PRESCALE			0x20
#define FIM_SERIAL_INT_BIT_TIME 		0x40

/* Interrupts from the FIM to the driver */
#define FIM_INT_FROM_MATCH_CHAR1		0x01
#define FIM_INT_FROM_MATCH_CHAR2		0x02
#define FIM_INT_FROM_RX_OVERFLOW		0x04

/* FIM status */
#define FIM_SERIAL_STAT_COMPLETE		0x01
#define FIM_SERIAL_STAT_TX_ENABLE		0x02
#define FIM_SERIAL_STAT_HW_FLOW 		0x04
#define FIM_SERIAL_STAT_MATCH_CHAR1		0x08
#define FIM_SERIAL_STAT_MATCH_CHAR2		0x10

/* Special control registers */
#define FIM_SERIAL_TXIO_REG			0
#define FIM_SERIAL_RXIO_REG			1
#define FIM_SERIAL_RTSIO_REG			2
#define FIM_SERIAL_CTSIO_REG			3
#define FIM_SERIAL_CTRL_REG			6

/* This is used only for the internal FIM-buffers */
#define FIM_SERIAL_BUF_PRIVATE			(void *)(0xffffffff)

/*
 * IMPORTANT: The number of chars per DMA-transfer must be low, otherwise
 * the latency times by user-interrupts (CTRL-C) will be too high (some seconds!)
 * The problem is that we can't stop the DMA-transfer in an easy way and the
 * FIM-firmware can't be stopped on the fly.
 */
#define FIM_SERIAL_TX_CHARS_PER_DMA		32
#define FIM_SERIAL_TX_DMA_BUFFER_SIZE		(FIM_SERIAL_TX_CHARS_PER_DMA * 2)
#define FIM_SERIAL_RX_DMA_BUFFER_SIZE		(PAGE_SIZE / 8)
#define FIM_SERIAL_TX_DMA_BUFFERS		64
#define FIM_SERIAL_RX_DMA_BUFFERS		32

#define FIM_SERIAL_GPIO_RX			(0)
#define FIM_SERIAL_GPIO_TX			(1)
#define FIM_SERIAL_GPIO_CTS			(2)
#define FIM_SERIAL_GPIO_RTS			(3)
#define FIM_SERIAL_GPIO_LAST			(4)

#define printk_err(fmt, args...)                printk(KERN_DEBUG "[ ERROR ] fim-serial: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_DEBUG "fim-serial: " fmt, ## args)
#define printk_dbg(fmt, args...)                printk(KERN_DEBUG "fim-serial: " fmt, ## args)

#define print_parse_err(parse, port, raw, pos)	printk_debug(parse " | Byte %i | Raw %x | Data %i | Total %i | Mask %x\n", pos, raw, port->numbits, port->totalbits, port->bitsmask)

#if 0
#define FIM_SERIAL_DEBUG
#endif

#ifdef FIM_SERIAL_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "fim-serial: " fmt, ## args)
#else
#  define printk_debug(fmt, args...)
#endif

#define FIM_CONSOLE_NAME			"ttyFIM"
#define FIM_SERIAL_DEV_NAME			"ttyFIM"

/* @XXX: Remove this macro (currently used for the aximal number of GPIOs) */
#define FIM_SERIAL_MAX_GPIOS			(8)

/*
 * Masks for the incoming PIC-data. The definition are coming from the Net+OS driver
 * and are in some cases unknown, then there is no exact documentation in the
 * PIC-code that defines the bit fields of the data bytes
 */
#define FIM_DATA_PARENB(data, port)             ((data >> port->numbits) & 0x01)
#define FIM_DATA_STOPB(data, port)              ((data >> (port->totalbits - 2)) & 0x03)
#define FIM_DATA_BITS(data, port)               ((data >> (port->totalbits - 1)) & 0x01)


/* Due to the fact that we can handle only one port per FIM */
struct fim_serial_t {
	dev_t devnr;
	struct fim_driver fim;
	struct uart_port uart;
	struct uart_driver *driver;
	struct device *device;
	int minor;
	int index;
	int open_count;
	int configured;
	unsigned int totalbits;
	unsigned short bitsmask;
	unsigned short numbits;
	unsigned int cflag;
	unsigned int baud;
	unsigned int last_totalbits;
	struct fim_gpio_t gpios[FIM_SERIAL_MAX_GPIOS];
	struct clk *sys_clk;
	spinlock_t tx_lock;
	struct tasklet_struct tasklet;
	int reg;
	struct ktermios *termios;
};

/* Main structure for the port-handling */
struct fim_serials_t {
	struct uart_driver driver;
	struct fim_serial_t *ports;
	int fims;
};

static struct fim_serials_t *fim_serials;

/* Firmware-dependent interrupts from the ARM to the FIM */
#define FIM_SERIAL_INT_INSERT_CHAR		0x01
#define FIM_SERIAL_INT_MATCH_CHAR		0x02
#define FIM_SERIAL_INT_BITS_CHAR		0x04
#define FIM_SERIAL_INT_CHAR_GAP 		0x08
#define FIM_SERIAL_INT_BIT_POS			0x10
#define FIM_SERIAL_INT_PRESCALE			0x20
#define FIM_SERIAL_INT_BIT_TIME 		0x40

/* Interrupts from the FIM to the driver */
#define FIM_INT_FROM_MATCH_CHAR1		0x01
#define FIM_INT_FROM_MATCH_CHAR2		0x02
#define FIM_INT_FROM_RX_OVERFLOW		0x04


/* FIM status */
#define FIM_SERIAL_STAT_COMPLETE		0x01
#define FIM_SERIAL_STAT_TX_ENABLE		0x02
#define FIM_SERIAL_STAT_HW_FLOW 		0x04
#define FIM_SERIAL_STAT_MATCH_CHAR1		0x08
#define FIM_SERIAL_STAT_MATCH_CHAR2		0x10

inline static struct fim_serial_t *get_port_from_uart(struct uart_port *uart)
{
	return dev_get_drvdata(uart->dev);
}


inline static struct uart_port *get_uart_from_port(struct fim_serial_t *port)
{
	return &port->uart;
}

inline static struct tty_struct *get_tty_from_port(struct fim_serial_t *port)
{
	struct uart_state *state;

	state = port->uart.state;
	return (state) ? state->port.tty : NULL;
}

inline static struct fim_serial_t *get_port_by_index(int index)
{
	if (index < 0 || index > fim_serials->fims)
		return NULL;

	return fim_serials->ports + index;
}

inline static int fim_serial_reset_matchs(struct fim_driver *fim)
{
	fim_set_ctrl_reg(fim, 0, 0xFF);
	fim_set_ctrl_reg(fim, 1, 0xFF);
	fim_set_ctrl_reg(fim, 2, 0xFF);
	fim_set_ctrl_reg(fim, 3, 0xFF);
	fim_set_ctrl_reg(fim, 4, 0x00);
	fim_set_ctrl_reg(fim, 5, 0x00);
	return fim_send_interrupt2(fim, FIM_SERIAL_INT_MATCH_CHAR);
}

inline static int fim_serial_reset_all(struct fim_driver *fim)
{
	int retval;

	fim_set_ctrl_reg(fim, 0, 0);
	fim_set_ctrl_reg(fim, 1, 0);
	fim_set_ctrl_reg(fim, 2, 0);
	fim_set_ctrl_reg(fim, 3, 0);
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_BIT_POS);
	if (!retval)
		retval = fim_serial_reset_matchs(fim);

	return retval;
}


static unsigned char get_parity(unsigned char data, unsigned char mask, int odd)
{
	unsigned char ones = 0;
	mask = (mask >> 1) + 1;
	do {
		ones += (data & mask) ? 1 : 0;
		mask >>= 1;
	} while (mask);
	if (odd)
		return ((ones & 1) ^ 1);
	else
		return (ones & 1);
}

/*
 * Send a char over an interrupt
 * Only implemented for sending the XOFF/XON chars. Otherwise use the send buffer
 * function, which uses the DMA-channel for a performanter data transfer
 */
static int fim_serial_send_char(struct fim_serial_t *port, unsigned char ch)
{
	unsigned int status;
	int retval;
	struct fim_driver *fim;
	unsigned char data = 1;
	unsigned int timeout;
	struct tty_struct *tty;

	fim = &port->fim;
	if (!(tty = get_tty_from_port(port))) {
		printk_err("No TTY-port found for the FIM %i\n", fim->picnr);
		return -ENODEV;
	}

	/* First check if the FIM is tasked with another send-char request */
	timeout = 0xFFFF;
	do {
		timeout--;
		fim_get_exp_reg(fim, 0, &status);
	} while (timeout && (status & FIM_SERIAL_INT_INSERT_CHAR));

	if (!timeout) {
		printk_err("Timeout by sending a char over the FIM %i\n", fim->picnr);
		return -EAGAIN;
	}

	if (C_CSTOPB(tty))
		data = (data << 1) | 0x01;

	if (C_PARENB(tty))
		data = (data << 1) | get_parity(ch, port->bitsmask, C_PARODD(tty));

	data = (data << port->numbits) | (ch & port->bitsmask);

	/* And send the char using the interrupt function */
	fim_set_ctrl_reg(fim, 0, data & 0xFF);
	fim_set_ctrl_reg(fim, 1, (data >> 8) & 0xFF);
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_INSERT_CHAR);
	if (retval)
		printk_err("Send char by the FIM %i failed, %i\n", fim->picnr, retval);

	return retval;
}

/*
 * Return value of this function is the number of copied bytes from `src' to `dest'
 */
static int fim_serial_parse_data(struct tty_struct *tty,
				 struct fim_serial_t *port,
				 unsigned char *src, int length,
				 unsigned char *dest)
{
	unsigned short data;
	int copied;
	struct uart_port *uart;

	uart = get_uart_from_port(port);
	for (copied=0; length > 1; length -= 2) {
		if (port->totalbits < 8) {
			data = *src >> (8 - port->totalbits);
			src++;
		} else {
			data = *src;
			src++;
			data |= (((unsigned short)*src >> (16 - port->totalbits)) << 8);
		}

		/* Ignore VSTART and VSTOP characters if hadshake is enabled */
		if (I_IXOFF(tty)) {
			if ((data & port->bitsmask) == START_CHAR(tty) ||
			    (data & port->bitsmask) == STOP_CHAR(tty)) {
				src++;
				length -= 2;
				continue;
			}
		}

		*dest = data & port->bitsmask;

		if (C_PARENB(tty)) {
			if (FIM_DATA_PARENB(data, port) != get_parity(*dest,
								      port->bitsmask,
								      C_PARODD(tty))) {
				uart->icount.parity++;
 				print_parse_err("Parity", port, data, copied);
				continue;
			}
		}

		if (C_CSTOPB(tty)) {
			if (FIM_DATA_STOPB(data, port) != 0x03) {

				/* @BUG: With 5N2 the stop bit flag is incorrect! */
				if (port->totalbits == 8) {
					uart->icount.frame++;
					print_parse_err("Stop  ", port, data, copied);
					continue;
				}
			}
		} else {
			if (!FIM_DATA_BITS(data, port)) {
				uart->icount.frame++;
 				print_parse_err("Data  ", port, data, copied);
				continue;
			}
		}

		src++;
		dest++;
		copied++;
	}

	return copied;
}

/*
 * The following function is coming from the Net+OS driver
 * This functions depends on the number of total bits too. For this reason is
 * required to call it if the number of bits changes
 */
static int fim_serial_baudrate(struct fim_serial_t *port, struct ktermios *termios)
{
	unsigned int div, prescale = 1;
	unsigned long clock;
	int baud, cnt, retval;
	struct fim_driver *fim;
	unsigned int gap;
	unsigned int bit_time;
	unsigned int cflag;

	fim = &port->fim;
	cflag = termios->c_cflag;
	baud = tty_termios_baud_rate(termios);

	/* Check if really need to change something */
	if (port->baud == baud) {
		if (port->last_totalbits != port->totalbits) {
			printk_debug("Setting only the gap timer (%iBps | %i | %i)\n",
				     baud, port->totalbits, port->last_totalbits);
			clock = clk_get_rate(port->sys_clk) / baud;
			goto set_char_gap;
		}
		return 0;
	}

	clock = clk_get_rate(port->sys_clk);
	printk_debug("Obtained SYS clock is %luHz\n", clock);
	clock = clock / baud;
	div = (clock / 256) + 1;

	/* Must round up to next power of 2 (see NET+OS driver) */
	for (cnt = 1; cnt <= 8; cnt++) {
		if (div < (unsigned int)(1 << cnt)) {
			div = 1 << cnt;
			prescale = cnt - 1;
			break;
		}
	}

	/* This sanity check is coming from the Net+OS driver */
	if (cnt > 8) {
		printk_debug("Wrong prescale value %i by the baud %i\n", cnt, baud);
		return -EINVAL;
	}

	/* The Net+OS driver has another calculation of the bit time */
	bit_time = (clock / div) - 1;

	printk_debug("%iB | Pre %i | Btime %u | Tbits %i | Dbits %i | S %i | Pa %i\n",
		     baud, prescale, bit_time, port->totalbits, port->numbits,
		     cflag & CSTOPB, cflag & PARENB);

	/* Set the bit time */
	fim_set_ctrl_reg(fim, 0, bit_time);
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_BIT_TIME);
	if (retval) {
		printk_err("Couldn't set the bit time of the FIM %i\n", fim->picnr);
		return -EAGAIN;
	}

	/* Set the prescale value */
	fim_set_ctrl_reg(fim, 0, prescale);
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_PRESCALE);
	if (retval) {
		printk_err("Prescale configuration of the FIM %i failed.\n",
			   fim->picnr);
		return retval;
	}

 set_char_gap:
	gap = (2 * clock * port->totalbits)/30;
	fim_set_ctrl_reg(fim, 0, (gap & 0xFF) + 1);
	fim_set_ctrl_reg(fim, 1, ((gap >> 8) & 0xFF) + 1);
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_CHAR_GAP);
	if (retval) {
		printk_err("GAP timer by the FIM %i (Total bytes %i)\n",
			   fim->picnr, port->totalbits);
		return retval;
	}

	port->baud = baud;
	port->last_totalbits = port->totalbits;
	return 0;
}

static int fim_sw_flowctrl(struct fim_serial_t *port, struct ktermios *termios)
{
	struct fim_driver *fim;
	int retval;
	unsigned short start_match, stop_match;
	unsigned short mask;
	unsigned int cflag, iflag;

	cflag = termios->c_cflag;
	iflag = termios->c_iflag;

	fim = &port->fim;
	start_match = 1;
	stop_match = 1;

	printk_debug("Calling %s\n", __func__);

	/* Check if need to set the SW flow control, otherwise reset it */
	if (iflag & IXON || iflag & IXOFF) {
		printk_debug("Active SW flow control? @XXX: Test it first.\n");
		if (cflag & CSTOPB) {
			start_match = (start_match << 1) | 1;
			stop_match = (stop_match << 1) | 1;
		}

		if (cflag & PARENB) {

			start_match = (start_match << 1) |
				get_parity(termios->c_cc[VSTART],
					   port->bitsmask,
					   cflag & PARENB & PARODD);

			stop_match = (stop_match << 1) |
				get_parity(termios->c_cc[VSTOP],
					   port->bitsmask,
					   cflag & PARENB & PARODD);
		}

		mask = start_match | port->bitsmask;

		if (port->totalbits < 8) {
			start_match = start_match << (8 - port->totalbits);
			stop_match = stop_match << (8 - port->totalbits);
			mask = port->bitsmask << (8 - port->totalbits);
		}

		fim_set_ctrl_reg(fim, 0, start_match & 0xff);
		fim_set_ctrl_reg(fim, 1, 0x00);
		fim_set_ctrl_reg(fim, 2, stop_match & 0xff);
		fim_set_ctrl_reg(fim, 3, 0x00);
		fim_set_ctrl_reg(fim, 4, mask & 0xff);
		fim_set_ctrl_reg(fim, 5, 0x00);

	} else {
		fim_serial_reset_matchs(fim);
	}

	/* Now send the interrupt for the SW flow control */
	retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_MATCH_CHAR);
	if (retval) {
		printk_err("Couldn't set the bits match\n");
		return retval;
	}

	return 0;
}

/* Function for the setup of the bit positions in the FIM-firmware */
static int fim_serial_setup_bitpos(struct fim_serial_t *port, ulong hwflow)
{
	struct fim_driver *fim;
	struct fim_gpio_t *gpios;
	unsigned int cts, rts;
	int offset;

	/* Depending on the processor we have different offset */
	if (processor_is_ns9215())
		offset = 68;
	else if (processor_is_ns9210())
		offset = 0;
	else
		return -EINVAL;

	fim = &port->fim;
	gpios = port->gpios;

	/* Set the TX- and RX-line */
	printk_debug("TX %i | RX %i | RTS %i | CTS %i | Offset %i\n",
		     gpios[FIM_SERIAL_GPIO_TX].nr, gpios[FIM_SERIAL_GPIO_RX].nr,
		     gpios[FIM_SERIAL_GPIO_RTS].nr, gpios[FIM_SERIAL_GPIO_CTS].nr,
		     offset);

	fim_set_ctrl_reg(fim, FIM_SERIAL_TXIO_REG,
			 1 << (gpios[FIM_SERIAL_GPIO_TX].nr - offset));

	fim_set_ctrl_reg(fim, FIM_SERIAL_RXIO_REG,
			 1 << (gpios[FIM_SERIAL_GPIO_RX].nr - offset));

	/* Check if the GPIOs are available for the HW flow control */
	if (hwflow && (gpios[FIM_SERIAL_GPIO_CTS].nr == FIM_GPIO_DONT_USE ||
		       gpios[FIM_SERIAL_GPIO_RTS].nr == FIM_GPIO_DONT_USE)) {
		printk_dbg("HW flow control not supported (GPIOs not defined)\n");
		return -EOPNOTSUPP;
	}

	/* Setup the CTS and RTS if HW flow controls is requested */
	cts = hwflow ? 1 << (gpios[FIM_SERIAL_GPIO_CTS].nr - offset) : 0;
	rts = hwflow ? 1 << (gpios[FIM_SERIAL_GPIO_RTS].nr - offset) : 0;
	fim_set_ctrl_reg(fim, FIM_SERIAL_CTSIO_REG, cts);
	fim_set_ctrl_reg(fim, FIM_SERIAL_RTSIO_REG, rts);

	return fim_send_interrupt2(fim, FIM_SERIAL_INT_BIT_POS);
}

/*
 * The ktermios structure is declared under: include/asm-arm/termbits.h
 */
static int fim_serial_configure_port(struct fim_serial_t *port,
				   struct ktermios *termios,
				   struct ktermios *old_termios)
{
	struct fim_driver *fim;
	unsigned int cflag, old_cflag;
	int databits, stopbits, parbit;
	int retval;
	unsigned int regval;
	int timeout;
	unsigned long flags;

	/* That's really bad but it can happen */
	if (!termios)
		return -EINVAL;

	/* @BUG: The info pointer is NULL when the console is started! */
	fim = &port->fim;
	cflag = termios->c_cflag;
	old_cflag = 0;
	if (old_termios)
		old_cflag = old_termios->c_cflag;

	/* Check the number of bits to set */
	switch (cflag & CSIZE) {
	case CS5:
		databits = 5;
		break;
	case CS6:
		databits = 6;
		break;
	case CS7:
		databits = 7;
		break;
	case CS8:
		databits = 8;
		break;
	default:
		printk_err("Invalid number of data bits flag 0x%04X\n", cflag & CSIZE);
		return -EINVAL;
	}

	/* Check if the STOPB flag is set, in that case use two stop bits */
	stopbits = 1;
	if (cflag & CSTOPB)
		stopbits = 2;

	parbit = 0;
	if (cflag & PARENB)
		parbit = 1;

	/* Lock the below code segment first */
	spin_lock_irqsave(&port->tx_lock, flags);

	/* Now, we must wait for the empty DMA-buffers */
	timeout = 1000;
	while (timeout && fim_tx_buffers_level(&port->fim)) {
		udelay(1000);
		cpu_relax();
		timeout--;
	}

	if (!timeout) {
		printk_err("Timeout waiting for an empty TX-DMA buffer!\n");
		retval = -ETIME;
		goto exit_unlock;
	}

	/* Ok, we are save, we can try to configure the FIM with the valid byte length */
	if (port->totalbits != databits + stopbits + parbit) {
		fim_set_ctrl_reg(fim, 0, databits + stopbits + parbit);
		retval = fim_send_interrupt2(fim, FIM_SERIAL_INT_BITS_CHAR);
		if (retval) {
			printk_err("Failed config of the data bits by the FIM %i\n",
				   fim->picnr);
			goto exit_unlock;
		}
	}

	/* Save the number of bits for the future operations */
	port->numbits = databits;
	port->totalbits = databits + stopbits + parbit;
	port->bitsmask = (1 << databits) - 1;
	port->cflag = cflag;

	/* Configure the software flow control */
	retval = fim_sw_flowctrl(port, termios);
	if (retval) {
		printk_err("SW flow control of the FIM %i\n", fim->picnr);
		goto exit_unlock;
	}

	/* Now configure the baudrate and bit timing parameters */
	retval = fim_serial_baudrate(port, termios);
	if (retval) {
		printk_err("Couldn't set the baud rate by the FIM %i\n", fim->picnr);
		goto exit_unlock;
	}

	/*
	 * Check if need to enable the CRTSCTS-lines. In that case first configure
	 * the bits position and then enable the HW-flow control in the config register
	 */
	if ((cflag & CRTSCTS) != (old_cflag & CRTSCTS)) {

		retval = fim_serial_setup_bitpos(port, cflag & CRTSCTS);
		if (retval) {

			/*
			 * This return value means that the HW flow control could not
			 * be enabled. Reset this flag so that the higher TTY layer
			 * knows that a requested operation failed and informs the user
			 * space about this.
			 */
			if (retval == -EOPNOTSUPP)
				termios->c_cflag &= ~CRTSCTS;

			printk_err("Couldn't setup the FIM port %i\n",
				   fim->picnr);
			goto exit_unlock;
		}

		/* Now set the control status register to the correct value */
		fim_get_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, &regval);
		if (cflag & CRTSCTS)
			regval |= FIM_SERIAL_STAT_HW_FLOW;
		else
			regval &= ~FIM_SERIAL_STAT_HW_FLOW;
		fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, regval);
	}

	/* After each reconfiguration we need to re-init the FIM-firmware */
	fim_get_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, &regval);
	fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG,
			 regval | FIM_SERIAL_STAT_TX_ENABLE | FIM_SERIAL_STAT_COMPLETE);

	retval = 0;
	port->termios = termios;

 exit_unlock:
	spin_unlock_irqrestore(&port->tx_lock, flags);
	return retval;
}

/*
 * Send a buffer over the FIM-core and the DMA-controller
 * For being faster we use an internal private FIM-buffer for sending the data
 * to the DMA-channel
 */
static int fim_serial_send_buffer(struct fim_serial_t *port,
				  struct uart_port *uart)
{
	struct fim_buffer_t buf;
	unsigned char *pdest;
	unsigned short data;
	unsigned char ch;
	struct fim_driver *fim;
	int bytes_per_char, stop_bits;
	int len, chars_to_send;
	struct circ_buf *xmit;
	unsigned char buf_data[FIM_SERIAL_TX_DMA_BUFFER_SIZE];
	int retval = 0;

	if (!port || !uart) {
		printk_err("NULL pointer passed (port %p | uart %p)\n", port, uart);
		return -EINVAL;
	}

	spin_lock(&port->tx_lock);

	xmit  = &uart->state->xmit;
	len = uart_circ_chars_pending(xmit);
	data = 1;
	if (port->cflag & CSTOPB)
		data = (data << 1) | 0x01;
	stop_bits = data;

	if (port->cflag & PARENB)
		data = (data << 1);
	data = (data << port->numbits);

	bytes_per_char = 1;
	if (data & 0xFF00)
		bytes_per_char = 2;

	fim = &port->fim;
	len = (len > FIM_SERIAL_TX_CHARS_PER_DMA) ? FIM_SERIAL_TX_CHARS_PER_DMA : len;

	/* Now copy the data into the FIM-buffer */
	pdest = buf_data;
	chars_to_send = 0;
	while (!uart_circ_empty(xmit) && chars_to_send < len) {
		ch = xmit->buf[xmit->tail];
		data = stop_bits;
		if (port->cflag & PARENB)
			data = (data << 1) | get_parity(ch, port->bitsmask,
							port->cflag & PARODD);

		data = (data << port->numbits) | (ch & port->bitsmask);

		/* For char length greater than 8 bits use two bytes for the TX-data */
		*pdest++ = data & 0xFF;
		if(bytes_per_char == 2)
			*pdest++ = (data >> 8) & 0xFF;

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uart->icount.tx++;
		chars_to_send += 1;
	}


        /* Check a last time if we need to send the data */
	if (uart_tx_stopped(uart) || !chars_to_send) {
		printk_debug("Aborting TX due a stopped queue!\n");
		retval = -EIO;
		goto exit_unlock;
	}

        /*
	 * Setup the FIM-buffer with the information that we have a private
	 * buffer and don't want to free it inside the TX-callback
	 */
	buf.private = FIM_SERIAL_BUF_PRIVATE;
	buf.length = chars_to_send * bytes_per_char;
	buf.data = buf_data;
	if (fim_send_buffer(fim, &buf)) {
		printk_err("FIM send buffer request failed (len %i)\n", buf.length);
		retval = -ENOMEM;
		goto exit_unlock;
	}

 exit_unlock:
	spin_unlock(&port->tx_lock);

	return retval;
}

static int fim_serial_transmit(struct uart_port *uart)
{
	struct circ_buf *xmit;
	struct fim_serial_t *port;
	struct fim_driver *fim;

	port = get_port_from_uart(uart);
	fim = &port->fim;
	xmit  = &uart->state->xmit;

	/*
	 * Check if we have enough space for the request, if not then the tasklet
	 * will be responsible for recalling this function and for resending the data
	 * of the UART buffer
	 */
	if (fim_tx_buffers_room(fim) < uart_circ_chars_pending(xmit)) {
		printk_debug("Skipping transmit request with length %lu\n",
			     uart_circ_chars_pending(xmit));
		return 0;
	}

	printk_debug("Request to send %lu chars | stopped %i | empty %i\n",
		     uart_circ_chars_pending(xmit),
		     uart_tx_stopped(uart), uart_circ_empty(xmit));

	/* Check if need to send only one char */
	if (uart->x_char) {
		printk_debug("Sending only one char\n");
		fim_serial_send_char(port, uart->x_char);
		uart->x_char = 0;
		uart->icount.tx++;
		return 1;
	}

	/* Check if have something to send */
	if (uart_circ_empty(xmit) || uart_tx_stopped(uart)) {
		printk_debug("Circular buffer empty or TX stopped\n");
		return 0;
	}

	/* IMPORTANT: Always use the "tail" of the xmit-buffer */
	fim_serial_send_buffer(port, uart);

	/* Tell the higher layer that we can "probably" send more data */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uart);

	return 0;
}

/* Function for transmitting data to the FIM-port */
static void fim_serial_start_tx(struct uart_port *uart)
{
	printk_debug("Calling %s\n", __func__);
	fim_serial_transmit(uart);
}

/* Return zero if the TX-DMA buffers are NOT empty */
static unsigned int fim_serial_tx_empty(struct uart_port *uart)
{
	struct fim_driver *fim;
	unsigned int level;
	struct fim_serial_t *port;

	port = get_port_from_uart(uart);
	fim = &port->fim;
	level = fim_tx_buffers_level(fim);

	printk_debug("TX empty called (Level %i)\n", level);

	return (level) ? 0 : TIOCSER_TEMT;
}

/* Handler for the incoming FIM-interrupts */
static void fim_serial_isr(struct fim_driver *driver, int irq, unsigned char code,
			   unsigned int rx_fifo)
{
	struct uart_port *uart;
	struct fim_serial_t *port;

	if(!code)
		return;

	port = (struct fim_serial_t *)driver->driver_data;
	uart = get_uart_from_port(port);

	switch (code) {
	case FIM_INT_FROM_MATCH_CHAR1:
		printk_err("@TODO: Match char1 interrupt received.\n");
		break;
	case FIM_INT_FROM_MATCH_CHAR2:
		printk_err("@TODO: Match char2 interrupt received.\n");
		break;
	case FIM_INT_FROM_RX_OVERFLOW:
		uart->icount.overrun++;
		break;
	default:
		break;
	}

	return;
}

/* This tasklet will restart the function for transmitting data to the FIM */
static void fim_serial_tasklet_func(unsigned long data)
{
	struct fim_serial_t *port;
	struct circ_buf *xmit;
	struct uart_port *uart;

	port = (struct fim_serial_t *)data;
	printk_debug("Tasklet for port %p\n", port);

	if (!port)
		return;

	uart = get_uart_from_port(port);
	if (!uart)
		return;

	xmit = &uart->state->xmit;
	if (uart_circ_chars_pending(xmit))
		fim_serial_transmit(uart);
}

/*
 * This is the TX-callback that the FIM-core call after a DMA-buffer was closed
 * The fim buffer structure contains our internal private data
 */
static void fim_serial_tx_isr(struct fim_driver *driver, int irq,
			      struct fim_buffer_t *pdata)
{
	struct fim_buffer_t *buf;
	struct fim_serial_t *port;

	port = (struct fim_serial_t *)driver->driver_data;
	buf = (struct fim_buffer_t *)pdata->private;
	printk_debug("TX-callback | FIM %i\n", driver->picnr);

	/*
	 * Free the allocated FIM-buffer
	 */
	if (buf && buf != FIM_SERIAL_BUF_PRIVATE)
		fim_free_buffer(&port->fim, buf);

	/* Schedule the tasklet for continuing with the data transmission */
	tasklet_schedule(&port->tasklet);

	return;
}

/* Called when a receive DMA-buffer was closed */
static void fim_serial_rx_isr(struct fim_driver *driver, int irq,
			      struct fim_buffer_t *pdata)
{
	struct fim_serial_t *port;
	struct uart_port *uart;
	struct tty_struct *tty;
	int length;

	/* Get the correct port from the FIM-driver structure */
	port = (struct fim_serial_t *)driver->driver_data;
	uart = &port->uart;
	tty = get_tty_from_port(port);

	/* If the port is closed then the tty structure will be NULL */
	if (!port || !tty) {
		printk_debug("uart %p | tty %p | port %p\n", uart, tty, port);
		return;
	}

	/* By errors pass nothing to the TTY-layer */
	if ((length = fim_serial_parse_data(tty, port, pdata->data, pdata->length,
					   pdata->data)) <= 0) {
		printk_err("Parsing the RX-DMA data (len %i)\n", length);
		return;
	}

	tty_insert_flip_string(tty, pdata->data, length);
	tty_flip_buffer_push(tty);
	return;
}

static unsigned int fim_serial_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void fim_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* @TODO? */
}

/* @TODO: This function isn't called for stopping the TX */
static void fim_serial_stop_tx(struct uart_port *port)
{
	printk_debug("Calling %s\n", __func__);
}

static void fim_serial_stop_rx(struct uart_port *port)
{
	printk_debug("Calling %s\n", __func__);
	/* @TODO? */
}

static void fim_serial_enable_ms(struct uart_port *port)
{
	/* @TODO? */
}

static void fim_serial_break_ctl(struct uart_port *port, int ctl)
{
	/* @TODO? */
}

/*
 * Called when the port is opened. By the boot-console it will be called before
 * the port configuration using set_termios
 */
static int fim_serial_startup(struct uart_port *uart)
{
	struct fim_serial_t *port;
	unsigned int regval;

	printk_debug("Calling %s\n", __func__);
	port = get_port_from_uart(uart);

	fim_enable_irq(&port->fim);

        fim_get_ctrl_reg(&port->fim, FIM_SERIAL_CTRL_REG, &regval);
        fim_set_ctrl_reg(&port->fim,
                         FIM_SERIAL_CTRL_REG,
                         regval | FIM_SERIAL_STAT_TX_ENABLE | FIM_SERIAL_STAT_COMPLETE);

	return 0;
}

/* Normally we must free the IRQ here, but the current FIM-firmware doesn't allow it */
static void fim_serial_shutdown(struct uart_port *port)
{
	printk_debug("@TODO: Extend the firmware for %s\n", __func__);
}

/* Called for setting the termios config */
static void fim_serial_set_termios(struct uart_port *uart, struct ktermios *termios,
				   struct ktermios *old)
{
	struct fim_serial_t *port;
	unsigned int cflag;

	printk_debug("Calling %s | termios %p | old %p\n", __func__, termios, old);

	cflag = (old) ? old->c_cflag : 0;
	if (cflag == termios->c_cflag) {
		printk_debug("Skipping the termios configuration\n");
		return;
	}

	port = get_port_from_uart(uart);
	fim_serial_configure_port(port, termios, old);
}

/* Return a string describing the type of the port */
static const char *fim_serial_type(struct uart_port *port)
{
	return FIM_DRIVER_NAME;
}

static void fim_serial_release_port(struct uart_port *port)
{
	printk_debug("Calling %s\n", __func__);
}

static int fim_serial_request_port(struct uart_port *port)
{
	printk_debug("Calling %s\n", __func__);
	return 0;
}

/* @TODO: Get more infos about the UART configuration */
static void fim_serial_config_port(struct uart_port *port, int flags)
{
	printk_debug("Calling %s\n", __func__);
	port->type = UPIO_MEM;
}

static int fim_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk_debug("Verify port called\n");
	return 0;
}

static struct uart_ops fim_serial_ops = {
	.tx_empty	= fim_serial_tx_empty,
	.set_mctrl	= fim_serial_set_mctrl,
	.get_mctrl	= fim_serial_get_mctrl,
	.stop_tx	= fim_serial_stop_tx,
	.start_tx	= fim_serial_start_tx,
	.stop_rx	= fim_serial_stop_rx,
	.enable_ms	= fim_serial_enable_ms,
	.break_ctl	= fim_serial_break_ctl,
	.startup	= fim_serial_startup,
	.shutdown	= fim_serial_shutdown,
	.set_termios	= fim_serial_set_termios,
	.type		= fim_serial_type,
	.release_port	= fim_serial_release_port,
	.request_port	= fim_serial_request_port,
	.config_port	= fim_serial_config_port,
	.verify_port	= fim_serial_verify_port
};

#ifdef CONFIG_SERIAL_FIM_CONSOLE
static void fim_serial_wait_tx(struct fim_driver *fim)
{
	unsigned int timeout;
	unsigned int status;

	/* First check if the FIM is tasked with another send-char request */
	timeout = 0xFFFF;
	do {
		timeout--;
		fim_get_exp_reg(fim, 0, &status);
		cpu_relax();
	} while (timeout && (status & FIM_SERIAL_INT_INSERT_CHAR));
}

static void fim_serial_console_putchar(struct uart_port *uart, int ch)
{
	struct fim_driver *fim;
	struct fim_serial_t *port;
	unsigned short data = 1;
	unsigned int cflag;

	port = get_port_from_uart(uart);
	fim = &port->fim;
	cflag = port->cflag;

	/* First wait for a free TX-FIFO */
	fim_serial_wait_tx(fim);

	if (cflag & CSTOPB)
		data = (data << 1) | 0x01;

	if (cflag & PARENB)
		data = (data << 1) | get_parity(ch, port->bitsmask,
						cflag & PARENB & PARODD);

	data = (data << port->numbits) | (ch & port->bitsmask);

	/* And send the char using the interrupt function */
	fim_set_ctrl_reg(fim, 0, data & 0xFF);
	fim_set_ctrl_reg(fim, 1, (data >> 8) & 0xFF);
	fim_send_interrupt2(fim, FIM_SERIAL_INT_INSERT_CHAR);
}

static void fim_serial_console_write(struct console *co, const char *str,
				   unsigned int count)
{
	struct fim_serial_t *port;
	struct uart_port *uart;
	unsigned long flags;

	port = get_port_by_index(co->index);
	uart = get_uart_from_port(port);

/* 	if (oops_in_progress) @TODO? */
	spin_lock_irqsave(&port->tx_lock, flags);

	/* @TODO: Test this function more intensive! */

	uart_console_write(uart, str, count, fim_serial_console_putchar);

	fim_serial_wait_tx(&port->fim);

	spin_unlock_irqrestore(&port->tx_lock, flags);
}

static int __init fim_serial_console_setup(struct console *co, char *options)
{
	struct fim_serial_t *port;
	struct uart_port *uart;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	printk_debug("Calling %s for index %i\n", __func__, co->index);

	/* @FIXME: Sanity checks required */
	port = get_port_by_index(co->index);
	if (!port->reg)
		return -ENODEV;

	uart = get_uart_from_port(port);
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	printk_debug("Console %p: B%i | P%i | B%i\n", uart, baud, parity, bits);
	fim_serial_startup(uart);
	return uart_set_options(uart, co, baud, parity, bits, flow);
}

/* The data of the console is being initialized inside the init-function */
static struct console fim_console = {
	.name	= FIM_SERIAL_DEV_NAME,
	.write	= fim_serial_console_write,
	.device	= uart_console_device,
	.setup	= fim_serial_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
};
#define fim_console_ptr (&fim_console)
#else
#define fim_console_ptr	(NULL)
#endif /* CONFIG_SERIAL_FIM_CONSOLE */

static int fim_serial_unregister_port(struct fim_serial_t *port)
{
	int cnt;
	struct fim_driver *fim;
	struct uart_port *uart;

	if (!port || !port->reg)
		return -ENODEV;

	tasklet_kill(&port->tasklet);

	uart = &port->uart;
	fim = &port->fim;
	fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, 0x00);

	/* Reset all the control register */
	fim_serial_reset_all(fim);

	fim_unregister_driver(fim);
	uart_remove_one_port(&fim_serials->driver, uart);

	/* Free all the requested GPIOs */
	for (cnt=0; cnt < FIM_SERIAL_MAX_GPIOS; cnt++) {
		printk_debug("Freeing GPIO %i\n", port->gpios[cnt].nr);
		if (port->gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		else if (port->gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;
		else
			gpio_free(port->gpios[cnt].nr);
	}

	/* Free the clock */
        clk_put(port->sys_clk);

	port->reg = 0;
	return 0;
}

static int fim_serial_register_port(struct device *dev,
				    struct fim_serial_t *port, int minor, int picnr,
				    struct fim_gpio_t gpios[])
{
	int retval;
	int cnt, func;
	struct fim_driver *fim;
	struct uart_port *uart;
	struct fim_dma_cfg_t dma_cfg;

	fim = &port->fim;
	uart = &port->uart;

	port->index = minor;
	fim->picnr = picnr;
	fim->driver.name = FIM_DRIVER_NAME;
	fim->fim_isr = fim_serial_isr;
	fim->dma_tx_isr = fim_serial_tx_isr;
	fim->dma_rx_isr = fim_serial_rx_isr;
	fim->driver_data = port;

	/* Set our desired DMA-channel configuration */
	dma_cfg.rxnr = FIM_SERIAL_RX_DMA_BUFFERS;
	dma_cfg.txnr = FIM_SERIAL_TX_DMA_BUFFERS;
	dma_cfg.rxsz = FIM_SERIAL_RX_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_SERIAL_TX_DMA_BUFFER_SIZE;
	fim->dma_cfg = &dma_cfg;

	/* Check if have a firmware code for using to */
	fim->fw_name = FIM_SERIAL_FIRMWARE_FILE;
	fim->fw_code = FIM_SERIAL_FIRMWARE_CODE;
	retval = fim_register_driver(fim);
	if (retval) {
		printk_err("Couldn't register the FIM driver.\n");
		return retval;
	}

	/* This is required for the FIM-firmware */
	fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, 0x00);

	/* Setup the bit positions but disable the HW flow control first */
	memcpy(port->gpios, gpios, FIM_SERIAL_MAX_GPIOS * sizeof(struct fim_gpio_t));
	retval = fim_serial_setup_bitpos(port, 0);
	if (retval) {
		printk_err("Setting the default GPIOs\n");
		goto err_unreg_fim;
	}

	retval = fim_serial_reset_matchs(fim);
	if (retval) {
		printk_err("Resetting the match registers\n");
		goto err_unreg_fim;
	}

	/* Request the corresponding GPIOs (@XXX: Check the returned values) */
	printk_debug("Requesting and configuring the GPIO's\n");
	for (cnt=0; gpios[cnt].nr != FIM_LAST_GPIO; cnt++) {

		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;

		printk_debug("Going to request the GPIO %i\n", gpios[cnt].nr);
		retval = gpio_request(gpios[cnt].nr, FIM_DRIVER_NAME);
		if (!retval) {

			func = gpios[cnt].func;

			gpio_configure_ns921x_unlocked(gpios[cnt].nr,
						       NS921X_GPIO_INPUT,
						       NS921X_GPIO_DONT_INVERT,
						       func,
						       NS921X_GPIO_ENABLE_PULLUP);
		} else {
			/* Free the already requested GPIOs */
			printk_err("Couldn't request the GPIO %i\n", gpios[cnt].nr);
			while (cnt) gpio_free(gpios[--cnt].nr);
			goto err_free_gpios;
		}
	}

	/* Get a reference to the SYS clock for setting the baudrate */
	if (IS_ERR(port->sys_clk = clk_get(port->fim.dev, "systemclock"))) {
		printk_err("Couldn't get the SYS clock.\n");
		goto err_free_gpios;
	}

	/* Init the internal data before registering the UART port */
        tasklet_init(&port->tasklet, fim_serial_tasklet_func, (unsigned long)port);

	spin_lock_init(&port->tx_lock);
	port->minor = minor;
	port->reg = 1;
	port->driver = &fim_serials->driver;

	/*
	 * Register with the corresponding minor number
	 * IMPORTANT: Set the internal fim-serial structure as driver-data before
	 * calling the function for adding a new uart port, then in some functions
	 * the pointer to the FIM-port is obtained from the driver data
	 */
	uart->ops = &fim_serial_ops;
	uart->flags = UPF_BOOT_AUTOCONF;
	uart->dev = dev;
	uart->type = PORT_UNKNOWN;
	uart->line = minor;
	uart->iotype = UPIO_PORT;
	uart->mapbase = 0x1234;
	dev_set_drvdata(uart->dev, port);
	if ((retval = uart_add_one_port(&fim_serials->driver, uart))) {
		dev_set_drvdata(uart->dev, NULL);
		printk_err("Couldn't register the UART port %i\n", minor);
		goto err_put_clk;
	}

	return 0;

 err_put_clk:
	clk_put(port->sys_clk);

 err_free_gpios:
	for (cnt=0; gpios[cnt].nr < FIM_SERIAL_MAX_GPIOS; cnt++) {
		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;

		if (gpios[cnt].nr != FIM_GPIO_DONT_USE)
			gpio_free(gpios[cnt].nr);
	}

 err_unreg_fim:
	fim_unregister_driver(&port->fim);

	return retval;
}

/*
 * Probe function
 */
static int __devinit fim_serial_probe(struct platform_device *pdev)
{
	struct fim_serial_t *port;
	struct fim_gpio_t gpios[FIM_SERIAL_MAX_GPIOS];
	int retval;
	struct fim_serial_platform_data *pdata;

	printk_info("Probe function called for device ID %i\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

        if (fim_check_device_id(fims_number, pdata->fim_nr)) {
#if defined(MODULE)
		printk_dbg("Skipping FIM%i (not selected)\n", pdata->fim_nr);
#else
		printk_err("Invalid FIM number '%i' in platform data\n", pdata->fim_nr);
#endif
		return -ENODEV;
        }

	port = fim_serials->ports + pdata->fim_nr;

	/* @XXX: The below code is really ugly, remove it! */
	gpios[FIM_SERIAL_GPIO_RX].nr   = pdata->rx_gpio_nr;
	gpios[FIM_SERIAL_GPIO_RX].func = pdata->rx_gpio_func;
	gpios[FIM_SERIAL_GPIO_TX].nr   = pdata->tx_gpio_nr;
	gpios[FIM_SERIAL_GPIO_TX].func = pdata->tx_gpio_func;
	gpios[FIM_SERIAL_GPIO_CTS].nr   = pdata->cts_gpio_nr;
	gpios[FIM_SERIAL_GPIO_CTS].func = pdata->cts_gpio_func;
	gpios[FIM_SERIAL_GPIO_RTS].nr   = pdata->rts_gpio_nr;
	gpios[FIM_SERIAL_GPIO_RTS].func = pdata->rts_gpio_func;
	gpios[FIM_SERIAL_GPIO_LAST].nr   = FIM_LAST_GPIO;

	/* And try to register the FIM */
	printk_debug("FIM %i | Port %p | Uart %p)\n", pdata->fim_nr, port, &port->uart);
	retval = fim_serial_register_port(&pdev->dev, port,
					  pdata->fim_nr, pdata->fim_nr,
					  gpios);

	return retval;
}

static int __devexit fim_serial_remove(struct platform_device *pdev)
{
	struct fim_serial_t *port;
	int retval;

	port = dev_get_drvdata(&pdev->dev);

	retval = fim_serial_unregister_port(port);
	if (!retval)
		dev_set_drvdata(&pdev->dev, NULL);

	return retval;
}

#if defined(CONFIG_PM) && !defined(CONFIG_FIM_SERIAL_MODULE)

static int fim_serial_suspend(struct platform_device *pdev,
			      pm_message_t state)
{
	int ret;
	struct fim_serial_t *port;

	/* Only stop the FIM before entering the suspend mode */
	port = dev_get_drvdata(&pdev->dev);
	ret = fim_send_stop(&port->fim);
	if (ret)
		printk_err("Couldn't stop the FIM\n");

	return ret;
}

static int fim_serial_resume(struct platform_device *pdev)
{
	struct fim_serial_t *port;
	int retval;
	struct fim_driver *fim;

	port = dev_get_drvdata(&pdev->dev);
	fim = &port->fim;

	/*
	 * Reset the internal values for being able to restore the port settings,
	 * otherwise the port will not work correctly after the wakeup reset.
	 */
	port->totalbits = 0;
	port->numbits = 0;
	port->last_totalbits = 0;
	port->baud = 0;

	retval = fim_download_firmware(&port->fim);
	if (retval) {
		printk_err("FIM download failed\n");
		goto exit_resume;
	}

	/* This is required for the FIM-firmware */
        fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, 0xff);

	retval = fim_send_start(&port->fim);
	if (retval) {
		printk_err("FIM start failed\n");
		goto exit_resume;
	}

        fim_set_ctrl_reg(fim, FIM_SERIAL_CTRL_REG, 0x00);

        retval = fim_serial_setup_bitpos(port, 0);
        if (retval) {
                printk_err("Setting the default GPIOs\n");
                goto exit_resume;
        }

        retval = fim_serial_reset_matchs(fim);
        if (retval) {
                printk_err("Resetting the match registers\n");
                goto exit_resume;
        }

	/*
	 * If the port was not opened yet, then no termios will be available, so
	 * only return at this point.
	 */
	if (port->termios) {
		retval = fim_serial_configure_port(port, port->termios, NULL);
		if (retval)
			printk_err("Port configuration failed\n");
	}

 exit_resume:
	return retval;
}
#else
#define fim_serial_suspend			NULL
#define fim_serial_resume			NULL
#endif /* CONFIG_PM */

/* @XXX: Work with hotplug and coldplug? */
MODULE_ALIAS(FIM_PLATFORM_DRIVER_NAME);

static struct platform_driver fim_serial_platform_driver = {
	.probe	 = fim_serial_probe,
	.remove	 = __devexit_p(fim_serial_remove),
	.suspend = fim_serial_suspend,
	.resume  = fim_serial_resume,
	.driver	 = {
		.owner = THIS_MODULE,
		.name  = FIM_DRIVER_NAME,
	},
};

static int __init fim_serial_init(void)
{
	int retval;
	int nrpics;

	printk_debug("Starting the FIM serial driver.\n");

	/* Get the number of maximal available FIMs */
	nrpics = fim_number_pics();

	/* Check for the passed number parameter */
	if (fim_check_numbers_param(fims_number)) {
		printk_err("Invalid number '%i' of FIMs to handle\n", fims_number);
		return -EINVAL;
	}

	fim_serials = kzalloc(sizeof(struct fim_serials_t) +
			      (nrpics * sizeof(struct fim_serial_t)), GFP_KERNEL);
	if (!fim_serials)
		return -ENOMEM;

	fim_serials->fims = nrpics;
	fim_serials->ports = (void *)fim_serials + sizeof(struct fim_serials_t);

	/* Init the UART driver */
	fim_serials->driver.owner = THIS_MODULE;
	fim_serials->driver.driver_name	= FIM_DRIVER_NAME;
	fim_serials->driver.dev_name = FIM_SERIAL_DEV_NAME;
	fim_serials->driver.nr = nrpics;

	/* @FIXME: This is ugly! */
#if defined(CONFIG_SERIAL_FIM_CONSOLE)
	fim_console_ptr->data = &fim_serials->driver;
#endif

	fim_serials->driver.cons = fim_console_ptr;

	printk_debug("Going to register the driver\n");
	retval = uart_register_driver(&fim_serials->driver);
	if (retval)
		goto err_free_mem;

	printk_debug("Trying to register the platform driver\n");
	retval = platform_driver_register(&fim_serial_platform_driver);
	if (retval)
		goto err_unreg_uart;

	printk_info(DRIVER_DESC " v" DRIVER_VERSION "\n");
	return 0;

 err_unreg_uart:
	uart_unregister_driver(&fim_serials->driver);

 err_free_mem:
	kfree(fim_serials);

	return retval;
}

static void __exit fim_serial_exit(void)
{
	printk_info("Removing the FIM serial driver\n");

	platform_driver_unregister(&fim_serial_platform_driver);
	uart_unregister_driver(&fim_serials->driver);

	kfree(fim_serials);
}

module_init(fim_serial_init);
module_exit(fim_serial_exit);

