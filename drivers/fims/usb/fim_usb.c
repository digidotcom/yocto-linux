/* -*- linux-c -*-
 *
 * driver/fims/usb/fim_usb.c
 *
 * Copyright (C) 2009 Digi International Inc.
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
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/reboot.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/completion.h>

#include <mach/fim-ns921x.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/regs-iohub-ns921x.h>

/* #include "fim_usb.h" */
/*
 * If the driver is being compiled as a built-in driver, then include the header file
 * which contains the firmware for this FIM-device
 */
#if !defined(MODULE)
#include "fim_usb.h"
extern const unsigned char fim_usb_firmware[];
#define FIM_USB_FIRMWARE_FILE			(NULL)
#define FIM_USB_FIRMWARE_CODE			fim_usb_firmware
#else
const unsigned char *fim_usb_firmware = NULL;
#define FIM_USB_FIRMWARE_FILE			"fim_usb.bin"
#define FIM_USB_FIRMWARE_CODE			fim_usb_firmware
#endif

/* Driver informations */
#define DRIVER_VERSION				"1.0"
#define FIM_DRIVER_AUTHOR			"Hector Oron, Luis Galdos"
#define FIM_DRIVER_DESC				"FIM USB serial device driver"
#define FIM_DRIVER_NAME				"fim-usb"
#define FIM_USB_UART_DEV_NAME			"ttyFUSB"

/* Module parameters */
NS921X_FIM_NUMBERS_PARAM(fims_number);

/* Macros used for the USB descriptors and resources */
#define FIM_USB_MANU				"Digi"
#define FIM_USB_PROD_DESC			"FIM-USBx" /* @FIXME: Cant change to an odd length */
#define FIM_USB_SERIALNR			"NS921X"
#define FIM_USB_IN_EPNR				(USB_DIR_IN | FIM_USB_INT_EP_NR)
#define FIM_USB_OUT_EPNR			(USB_DIR_OUT | FIM_USB_INT_EP_NR)

/* MAX GPIO NUMBER */
#define FIM_USB_MAX_GPIOS			(6)

#define FIM_USB_GPIO_DP				(0)
#define FIM_USB_GPIO_DM				(1)
#define FIM_USB_GPIO_RCV			(2)
#define FIM_USB_GPIO_OE				(3)
#define FIM_USB_GPIO_ENUM			(4)
#define FIM_USB_GPIO_SPND			(5)

/* Macros for controlling the enumeration GPIO of the MAX */
#define FIM_USB_GPIO_ENUM_ENABLE		(1)
#define FIM_USB_GPIO_ENUM_DISABLE		(0)


#define FIM_USB_MAX_PACK_SIZE			(8)
#define FIM_USB_INT_EP_NR			(1)


/* Firmware dependent status/error flags */
#define FIM_USB_BIT_STUFF_ERROR                 (1 << 0)
#define FIM_USB_CRC_ERROR                       (1 << 1)
#define FIM_USB_RX_OVERFLOW                     (1 << 2)
#define FIM_USB_DEV_ADDR_NO_MATCH               (1 << 3)
#define FIM_USB_SYNC_ERROR                      (1 << 4)
#define FIM_USB_IN_DATA_SENT_MARKER             (1 << 5)
#define FIM_USB_BUS_RESET                       (1 << 6)
#define FIM_USB_KEEP_ALIVE                      (1 << 7)

/* When a data packet is received it includes the sync, token, CRC, etc. */
#define FIM_USB_DATA_OVERHEAD			(5)
#define FIM_USB_DATA_HEAD			(2)
#define FIM_USB_DATA_TAIL			(3)

/*
 * Since the FIMs doesn't support the CRC calculation and other additional features
 * (like the setup of the different tokens), we must create the complete USB frame
 * before putting it into the DMA-buffer (sucks!). So, we need the below token values
 * for each frame.
 *
 * (Luis Galdos)
 */
#define USB_TOKEN_IN				(0x69)
#define USB_TOKEN_OUT				(0xe1)
#define USB_TOKEN_DATA0				(0xc3)
#define USB_TOKEN_DATA1				(0x4b)
#define USB_TOKEN_SETUP				(0x2d)
#define USB_TOKEN_SYNC				(0x80)

/* Configuration status registers of the FIM firmware */
#define FIM_USB_ADDR_REG                        2
#define FIM_USB_CONTROL3                        3
#define FIM_USB_CONTROL4                        4
#define FIM_USB_CONTROL5                        5
#define FIM_USB_CONTROL6                        6

#define FIM_USB_CTRL10_REG			(10)
#define FIM_USB_STAT10_REG			(10)
#define FIM_USB_CTRL3_REG			(3)
#define FIM_USB_STAT3_REG			(3)
#define FIM_USB_ALIVE_COUNT_REG			FIM_USB_CTRL10_REG
#define FIM_USB_ALIVE_COUNT_STAT		FIM_USB_STAT10_REG

/* Main control register */
#define FIM_USB_MAIN_REG			(7)
#define FIM_USB_MAIN_START			(1 << 0)
#define FIM_USB_MAIN_NOKEEPALIVE		(1 << 1)
#define FIM_USB_MAIN_NOINIRQ			(1 << 2)
#define FIM_USB_MAIN_ALIVE_COUNT		(1 << 3)

#define FIM_USB_REVISION_REG			(0)

/* Macros for the configuration of the DMA-channel */
#define FIM_USB_DMA_BUFFER_SIZE			(512)
#define FIM_USB_DMA_RX_BUFFERS			(61)
#define FIM_USB_DMA_TX_BUFFERS			(4)

/* For printing messages (errors, warning, etc.) */
#define pk_err(fmt, args...)			printk(KERN_ERR "[ ERROR ] fim-usb: " fmt, ## args)
#define pk_info(fmt, args...)			printk(KERN_INFO "fim-usb: " fmt, ## args)
#define pk_dbg(fmt, args...)			printk(KERN_DEBUG "fim-usb: " fmt, ## args)
#define pk_dump_ctrl_packet(p)			printk(KERN_DEBUG "bRequestType 0x%02x | bRequest 0x%02x | " \
							"wValue 0x%04x | wIndex 0x%04x | wLength %u\n", \
							p->bRequestType, p->bRequest, \
							p->wValue, p->wIndex, p->wLength);

#if 0
#define FIM_USB_DEBUG
#endif

/* Macros for printing debug messages */
#ifdef FIM_USB_DEBUG
#define dbg_func(fmt, args...)			printk(KERN_DEBUG "fim-usb: %s() " fmt, __func__, ## args)
#define dbg_naked(fmt, args...)			printk(KERN_DEBUG "fim-usb: " fmt, ## args)
#define dbg_pr(fmt, args...)			printk(KERN_DEBUG "fim-usb: " fmt, ## args)
#else
#define dbg_pr(fmt, args...)			do { } while (0)
#define dbg_naked(fmt, args...)			do { } while (0)
#define dbg_func(fmt, args...)			do { } while (0)
#endif

/* Enable the debug messages for the UART operations/functions */
#if 0
#define FIM_USB_UART_DEBUG
#endif

#ifdef FIM_USB_UART_DEBUG
#define dbg_uart(fmt, args...)			printk(KERN_DEBUG "fim-usb UART: " fmt, ## args)
#else
#define dbg_uart(fmt, args...)			do { } while (0)
#endif

/* Enable the debug messages for the enumeration sequence */
#if 0
#define FIM_USB_ENUM_DEBUG
#endif

#ifdef FIM_USB_ENUM_DEBUG
#define dbg_enum(fmt, args...)			printk(KERN_DEBUG "fim-usb ENUM: " fmt, ## args)
#else
#define dbg_enum(fmt, args...)			do { } while (0)
#endif

/* Enable the debug messages for the received tokens */
#if 0
#define FIM_USB_TOKEN_DEBUG
#endif

#ifdef FIM_USB_TOKEN_DEBUG
#define dbg_token(fmt, args...)			printk(KERN_DEBUG "fim-usb TN: " fmt, ## args)
#else
#define dbg_token(fmt, args...)			do { } while (0)
#endif

/* Enable the debug messages for the received tokens */
#if 0
#define FIM_USB_WRITE_DEBUG
#endif

#ifdef FIM_USB_WRITE_DEBUG
#define dbg_write(fmt, args...)			printk(KERN_DEBUG "fim-usb WRITE: " fmt, ## args)
#else
#define dbg_write(fmt, args...)			do { } while (0)
#endif

#if 0
#define FIM_USB_FORCE_ENUM_TEST
#endif

/* Data structure used for passing a data buffer to the FIM core */
struct fim_usb_in_frame {
	u8 epnr;
	u8 bytes;
	u8 token;
	u8 data[32];
	u16 *crc;
}__attribute__((__packed__));

/*
 * Let's use this structure for handling the received FIM-buffers
 *
 */
struct fim_usb_token {
	unsigned long type:8;
	unsigned long addr:7;
	unsigned long ep:4;
} __attribute__((packed));

/* Structure for the data received after the OUT tokens */
struct fim_usb_data {
	u8 sync;     /* Sync token 0x80 */
	u8 token;    /* DATA0 or DATA1 */
	u8 data[32]; /* @XXX: Quick and dirty! Need a macro for this value */
} __attribute__((packed));

struct fim_usb_ep {
	u8 nr;
	u8 addr;

	u16 bytes;
	u16 requested;
	u8 *data;

	u8 zlp;
	u8 last_data_token;

	struct fim_usb_port *port;

	struct work_struct tx_work; /* Worker for the IN transfers */
	struct uart_port *tx_uart;
	struct semaphore tx_sem;
};

struct fim_usb_port {
	int					index;
	struct fim_driver			fim;
	struct device				*dev;
 	struct fim_gpio_t 			gpios[FIM_USB_MAX_GPIOS];
	struct clk				*clk;
	int 					reg;

	/* We have three endpoints: control and two interrupts */
#define FIM_USB_NR_EP_CTRL			(0)
#define FIM_USB_NR_EP_OUT			(1)
#define FIM_USB_NR_EP_IN			(2)
#define FIM_USB_NR_EPS				(3)
	struct fim_usb_ep			eps[FIM_USB_NR_EPS];
	int					ep_next_data;

	/* Data used for the emulation of the serial port */
	struct uart_port uart;
	struct work_struct uart_work;
	int uart_created;
	struct semaphore uart_sem;

	/* Used for restarting the FIM after heavy failures */
	struct work_struct restart_work;
	struct completion restart_completed;

	/* Timer used for detecting the hotplug events */
	struct delayed_work hotplug_work;
	int unplug_detected;
#define FIM_USB_HOTPLUG_ALIVE_TIMER			(HZ / 5)
#define FIM_USB_HOTPLUG_ALIVE_COUNTER			(150)
};

#define FIM_USB_STRING_MAX_LEN				(32)

struct fim_usb_string {
        u8 id;
        const char *s;

	/* This data will be send to the host */
	struct usb_descriptor_header desc;
	u16 data[FIM_USB_STRING_MAX_LEN];
} __attribute__((packed));

/* The default endpoint descriptors include the members for the audio extension (see include/linux/usb/ch9.h) */
struct usb_endpoint_descriptor2 {
        __u8  bLength;
        __u8  bDescriptorType;

        __u8  bEndpointAddress;
        __u8  bmAttributes;
        __le16 wMaxPacketSize;
        __u8  bInterval;
} __attribute__ ((packed));

/* Let's keep it simple by using a static configuration descriptor */
struct fim_usb_config_descriptor {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor2 ep_in;
	struct usb_endpoint_descriptor2 ep_out;
} __attribute__((__packed__));


/* Prototypes */
static void fim_usb_regs_reset(struct fim_usb_port *port);
static void fim_usb_uart_tx_work_func(struct work_struct *work);
static int fim_usb_reboot_notifier_func(struct notifier_block *this, unsigned long code,
					void *unused);
static int fim_usb_restart_fim(struct fim_usb_port *port);

static struct uart_driver fim_usb_uart_driver;
static struct fim_usb_port **fim_usb_ports;
static int fim_usb_ports_max;
static struct notifier_block fim_usb_reboot_notifier = {
	.notifier_call	= fim_usb_reboot_notifier_func,
	.next		= NULL,
	.priority	= 0
};

static inline struct fim_usb_port *port_from_fim(struct fim_driver *fim)
{
	return fim->driver_data;
}

static inline struct tty_struct *tty_from_port(struct fim_usb_port *port)
{
        struct uart_state *state;

        state = port->uart.state;
        return (state) ? state->port.tty : NULL;
}

static inline struct fim_usb_port *port_from_uart(struct uart_port *uart)
{
        return dev_get_drvdata(uart->dev);
}

static void fim_usb_enable_pullup(struct fim_usb_port *port)
{
	dbg_func("FIM %i\n", port->index);

	/*
	 * Enable the pulldown for the enumeration process and remove the suspend
	 * control for starting the externa PHY. The other GPIOs are controller by
	 * the FIM, specially the Output Enable.
	 */
	gpio_set_value(port->gpios[FIM_USB_GPIO_SPND].nr, 0);
	gpio_set_value(port->gpios[FIM_USB_GPIO_ENUM].nr, 1);
}

static void fim_usb_disable_pullup(struct fim_usb_port *port)
{
	dbg_func("FIM %i\n", port->index);

	/* Disable the pulldown and set the PHY in the suspend mode */
	gpio_set_value(port->gpios[FIM_USB_GPIO_ENUM].nr, 0);
	gpio_set_value(port->gpios[FIM_USB_GPIO_SPND].nr, 1);
}

static void fim_usb_regs_reset(struct fim_usb_port *port)
{
        /* FIM control registers configuration */
        fim_set_ctrl_reg(&port->fim, 0, 0);
        fim_set_ctrl_reg(&port->fim, 1, 0);
        fim_set_ctrl_reg(&port->fim, 2, 0);
        fim_set_ctrl_reg(&port->fim, 3, 0);
        fim_set_ctrl_reg(&port->fim, 4, 0);
        fim_set_ctrl_reg(&port->fim, 5, 0);
        fim_set_ctrl_reg(&port->fim, 6, 0);
	fim_set_ctrl_reg(&port->fim, FIM_USB_MAIN_REG, 0);
}

/*
 * According to the function[1] of the NetOS driver, we must first configure the
 * dedicated GPIOs
 *
 * [1] iop_config.c:naIopUsbInit()
 */
static int fim_usb_config(struct fim_usb_port *port)
{
        unsigned long clks_per_bit;
	unsigned long timer0_init_value;
	struct fim_driver *fim;
	int offset;
	struct fim_gpio_t *gpios;

	fim = &port->fim;
	gpios = port->gpios;

        /* Depending on the processor we have different offset */
        if (processor_is_ns9215())
                offset = 68;
        else if (processor_is_ns9210())
                offset = 0;
        else
                return -EINVAL;

        /* FIM control registers configuration */
	fim_set_ctrl_reg(fim, 3, 1 << (gpios[FIM_USB_GPIO_DP].nr - offset));
	fim_set_ctrl_reg(fim, 4, 1 << (gpios[FIM_USB_GPIO_DM].nr - offset));
	fim_set_ctrl_reg(fim, 5, 1 << (gpios[FIM_USB_GPIO_RCV].nr - offset));
	fim_set_ctrl_reg(fim, 6, 1 << (gpios[FIM_USB_GPIO_OE].nr - offset));

        /* Calculate timer setting */
        /* FIM init Configuration. XXX port->clk must be MHz */
        clks_per_bit = clk_get_rate(port->clk) / 1000000;
        clks_per_bit = (clks_per_bit * 2) / 3;
        dbg_pr("Clocks Per Bit: 0x%lx [Sysclock %lu]\n", clks_per_bit,
	       clk_get_rate(port->clk));

        /* Sanity check for clock */
        if ((clks_per_bit > 270) || (clks_per_bit < 0)) {
                pk_err("Invalid calculated clocks per bit %lu\n", clks_per_bit);
		return -EINVAL;
        }

        /* Load values according to the NetOS driver */
        fim_set_ctrl_reg(&port->fim, 0, 231);

	timer0_init_value = 256 - (clks_per_bit - 11);
        fim_set_ctrl_reg(&port->fim, 1, timer0_init_value);

        /* Set the initial device address to zero */
        fim_set_ctrl_reg(&port->fim, FIM_USB_ADDR_REG, 0);

	return 0;
}

/* We only provide two interrupt endpoints and one control EP */
static void fim_usb_init_eps(struct fim_usb_port *port)
{
	struct fim_usb_ep *ep;

	memset(&port->eps, 0, sizeof(port->eps));

	/* Setup the control EP0 */
	ep = &port->eps[FIM_USB_NR_EP_CTRL];
	ep->addr = 0;
	ep->port = port;
	ep->nr = FIM_USB_NR_EP_CTRL;

	/* Setup the OUT EP1 */
	ep = &port->eps[FIM_USB_NR_EP_OUT];
	ep->addr = FIM_USB_INT_EP_NR;
	ep->port = port;
	ep->nr = FIM_USB_NR_EP_OUT;

	/* Setup the IN EP1. Init the worker for the IN transfers too! */
	ep = &port->eps[FIM_USB_NR_EP_IN];
	ep->addr = FIM_USB_INT_EP_NR;
	INIT_WORK(&ep->tx_work, fim_usb_uart_tx_work_func);
	ep->port = port;
	ep->nr = FIM_USB_NR_EP_IN;

	/* @XXX: Define a correct reset value for this member */
	port->ep_next_data = 0;
}

/* Be sure that you have disconnected the pull-up first */
static void fim_usb_init_port(struct fim_usb_port *port)
{
	/* Reset the registers */
	fim_usb_regs_reset(port);

	/* Download config to enpoint buffer */
	fim_usb_config(port);
	fim_usb_init_eps(port);
}

static int fim_usb_reboot_notifier_func(struct notifier_block *this, unsigned long code,
					void *unused)
{
	struct fim_usb_port *port;
	int cnt;

	for (cnt = 0; cnt < fim_usb_ports_max; cnt++) {
		port = fim_usb_ports[cnt];
		if (!port)
			continue;

		fim_usb_disable_pullup(port);
	}

	return NOTIFY_DONE;
}

static void fim_usb_hotplug_work_func(struct work_struct *work)
{
	struct fim_driver *fim;
	int reg;
	int cnt_curr, cnt_init;
	struct fim_usb_port *port;

	port = container_of(work, struct fim_usb_port, hotplug_work.work);
	fim = &port->fim;

	/* Get the status of the main register and the current countdown value */
	fim_get_ctrl_reg(fim, FIM_USB_MAIN_REG, &reg);
	fim_get_stat_reg(fim, FIM_USB_ALIVE_COUNT_STAT, &cnt_init);

	/* Set the initial value for the countdown register */
	fim_set_ctrl_reg(fim, FIM_USB_ALIVE_COUNT_REG, FIM_USB_HOTPLUG_ALIVE_COUNTER);

	/* Inform the FIM about the new reloaded value */
	fim_set_ctrl_reg(fim, FIM_USB_MAIN_REG, reg | FIM_USB_MAIN_ALIVE_COUNT);

	/* We need to wait for the status register */
	msleep_interruptible(50);
	fim_get_stat_reg(fim, FIM_USB_ALIVE_COUNT_STAT, &cnt_curr);

	/* Reset the flag of the control register */
	fim_set_ctrl_reg(fim, FIM_USB_MAIN_REG, reg);

	/* Re-trigger the timer function if all looks OK */
	if (cnt_init != cnt_curr) {
		port->unplug_detected = 0;
		schedule_delayed_work(&port->hotplug_work, FIM_USB_HOTPLUG_ALIVE_TIMER);
	} else {
		/* In the case that the timer has expired we need to wait for the RXNRIP */
		port->unplug_detected = 1;

		/* Remove the serial port for canceling the UART operations */
		if (port->uart_created) {
			struct uart_port *uart;

			uart = &port->uart;
			uart_remove_one_port(&fim_usb_uart_driver, uart);
			port->uart_created = 0;
		}

		fim_usb_restart_fim(port);

		pk_dbg("Keep alive counter expired (init 0x%x | curr 0x%x). Port %i removed\n",
		       cnt_init, cnt_curr, fim->picnr);
	}
}

/* Function used for creating or removing the UART port of this FIM */
static struct uart_ops fim_usb_uart_ops;
static void fim_usb_uart_work_func(struct work_struct *work)
{
	struct fim_usb_port *port;
	int ret;
	struct uart_port *uart;
	struct fim_driver *fim;

	port = container_of(work, struct fim_usb_port, uart_work);
	uart = &port->uart;
	fim = &port->fim;

	/*
	 * This can happen when the device entered the suspend mode and restarts the
	 * enumeration sequence
	 */
	if (port->uart_created) {
		pk_dbg("UART port %i already created.\n", fim->picnr);
		goto exit_sched;
	}

	/* We really need to restart the structure at this place */
	uart->ops = &fim_usb_uart_ops;
	uart->flags = UPF_BOOT_AUTOCONF;
	uart->dev = port->dev;
	uart->type = UPIO_MEM;
	uart->line = fim->picnr; /* Use the FIM number as minor */
	uart->iotype = UPIO_PORT;
	dev_set_drvdata(uart->dev, port);

	/*
	 * @FIXME: Lock this section, otherwise we will have some problems by the plug-unplug
	 * tests of the SALAB.
	 */
	pk_dbg("Adding the new port %i\n", fim->picnr);
	ret = uart_add_one_port(&fim_usb_uart_driver, uart);
	if (ret) {
                dev_set_drvdata(uart->dev, NULL);
                pk_err("Couldn't register the UART port %i\n", fim->picnr);
                goto err_add;
        }

	port->uart_created = 1;

 exit_sched:
	/* @FIXME: Is this the correct place for our FIM timer loader? */
	schedule_delayed_work(&port->hotplug_work, 0);

 err_add:
	return;
}

/*
 * Send a buffer over the FIM-API. We are using static buffers for sending the data
 * to the FIMs, we don't need to have a new allocated buffer, since the FIM only accepts
 * frames with a maximal sizo of 13 bytes (8 with the EP data)
 * (Luis Galdos)
 */
static int __fim_usb_ep_send_buffer(struct fim_usb_port *port, void *buffer, int len, void *priv)
{
        struct fim_driver *fim;
        int retval;
	struct fim_buffer_t buf;

        if (!port || !len)
                return -EINVAL;

        fim = &port->fim;

        buf.private = priv;
	buf.data = buffer;
	buf.length = len;

	retval = fim_send_buffer(fim, &buf);
        if (retval) {
		int level;

		level = fim_tx_buffers_level(&port->fim);
		pk_dbg("FIM%i send %i bytes failed (err %i, level %i)\n",
		       fim->picnr, buf.length, retval, level);
		schedule_work(&port->restart_work);
	}

        return retval;
}

/* Calculate the CRC16 for the passed buffer */
static inline u16 fim_usb_calc_buffer_crc(const u8 *buffer, unsigned long length)
{
	u16 retval, bx;
	unsigned char crc16;
	int i;
	ulong cnt;

	retval = 0xffff;
	for (cnt = 0; cnt <length; cnt++) {
		for (i=0; i < 8; i++) {
			crc16 = retval >> (15 - i);
			bx = crc16 ^ buffer[cnt];
			retval <<= 1;

			if (bx & (0x1 << i))
				retval ^= 0x8005;
		}
	}

	bx = ~retval;
	retval = 0;
	for (i=0; i < 8; i++)
		retval |= ((bx & (0x1 << i)) << (15 - (i * 2)) |
			   (bx & (0x100 << i)) >> (1 + (i * 2)));

	return retval;
}

#define fim_usb_ep_next_data_token(ep)			((ep->last_data_token == USB_TOKEN_DATA0) ? \
								USB_TOKEN_DATA1 : USB_TOKEN_DATA0)

/*
 * Call this function for sending a ZLP by the passed EP
 * If force is true, the function will ignore the current TX fifo level and will queue
 * the new IN data packet.
 */
static int fim_usb_write_zlp(struct fim_usb_port *port, struct fim_usb_ep *ep, int force)
{
	struct fim_usb_in_frame fin;
	int level;

	dbg_write("EP%u: Sending ZLP\n", ep->nr);

	level = fim_tx_buffers_level(&port->fim);
	if (level && !force) {
		pk_dbg("ZLP: TX level > 0 (%i). FIM restart.\n", level);
		fim_usb_restart_fim(port);
		return -ERESTART;
	}

	memset(&fin, 0, sizeof(fin));
	fin.epnr = ep->addr & ~USB_DIR_IN;
	fin.bytes = FIM_USB_DATA_TAIL;
	fin.token = fim_usb_ep_next_data_token(ep);

	/* @XXX: Reset this flag depending on the return value of the send buffer function */
	ep->zlp = 0;

	return __fim_usb_ep_send_buffer(port, &fin, FIM_USB_DATA_OVERHEAD, ep);
}

/* IMPORTANT: This function returns the number of transferred bytes! */
static int fim_usb_write_packet(struct fim_usb_port *port, struct fim_usb_ep *ep, u8 *buf, u16 len)
{
	u16 crc, val;
	struct fim_usb_in_frame fin;
	int ret, level;

	/* @FIXME: Use a correct sanity check! */
	level = fim_tx_buffers_level(&port->fim);
	if (level) {
		pk_dbg("write packet: TX level = %i. FIM restart.\n", level);
		fim_usb_restart_fim(port);
		return -EINVAL;
	}

	val = min(len, (u16)FIM_USB_MAX_PACK_SIZE);
	ep->bytes = len - val;
	ep->data = buf + val;
	ep->requested -= val;
	dbg_enum("EP%u: Sending %u bytes [%p], pending %u [%p], requested %u\n",
		 ep->nr, val, buf, ep->bytes, ep->data, ep->requested);

	/*
	 * The FIM expects that the IN-data is placed in the TX-FIFO.
	 *
	 */
	crc = fim_usb_calc_buffer_crc(buf, val);
#if defined(FIM_USB_DEBUG_SEND_BUFFER)
	dbg_func("Calculated CRC 0x%04x\n", crc);
#endif

	/* Format: Endpoint number | Number of bytes | Token | Data | CRC16 */
	fin.epnr = ep->addr & ~USB_DIR_IN;
	fin.bytes = val + FIM_USB_DATA_TAIL; /* Token and CRC16 */

	fin.token = fim_usb_ep_next_data_token(ep);
	ep->last_data_token = fin.token;

	fin.crc = (u16 *)(fin.data + val);
	memcpy(fin.data, buf, val);

	*fin.crc = crc;
	ret = __fim_usb_ep_send_buffer(port, &fin, val + FIM_USB_DATA_OVERHEAD, ep);
	return ret ? ret : val;
}

#define FIM_USB_VENDOR_ID			(0x0210)
#define FIM_USB_PRODUCT_ID			(0xbe13)

#define STRING_MANUFACTURER			1
#define STRING_PRODUCT				2
#define STRING_SERIALNUM			3

#define DEV_CONFIG_VALUE			1
#define FIM_USB_INTERFACE			0

static struct usb_device_descriptor fim_usb_device_desc = {
        .bLength =              sizeof(fim_usb_device_desc),
        .bDescriptorType =      USB_DT_DEVICE,
        .bcdUSB =               __constant_cpu_to_le16(0x0110),
        .bDeviceClass =         0,
        .bDeviceSubClass =      0,
        .bDeviceProtocol =      0,
	.bMaxPacketSize0 =	8,
        .idVendor =             __constant_cpu_to_le16(FIM_USB_VENDOR_ID),
        .idProduct =            __constant_cpu_to_le16(FIM_USB_PRODUCT_ID),
        .iManufacturer =        STRING_MANUFACTURER,
        .iProduct =             STRING_PRODUCT,
        .iSerialNumber =        STRING_SERIALNUM,
        .bNumConfigurations =   1
};

static struct fim_usb_config_descriptor fim_usb_config_desc2 = {
	.config = {
		.bLength =              USB_DT_CONFIG_SIZE,
		.bDescriptorType =      USB_DT_CONFIG,

		/* compute wTotalLength on the fly */
		.bNumInterfaces =       1,
		.bConfigurationValue =  DEV_CONFIG_VALUE,
		.iConfiguration =       0,
		.bmAttributes =         USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
		.bMaxPower =            0
	},
	.interface = {
		.bLength =              USB_DT_INTERFACE_SIZE,
		.bDescriptorType =      USB_DT_INTERFACE,
		.bInterfaceNumber =     FIM_USB_INTERFACE,
		.bNumEndpoints =        2,
		.bInterfaceClass =      0,
		.bInterfaceSubClass =   0,
		.bInterfaceProtocol =   0,
		.iInterface =           0
	},
	.ep_out = {
		.bLength =              USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =      USB_DT_ENDPOINT,
		.bEndpointAddress =     FIM_USB_OUT_EPNR,
		.bmAttributes =         USB_ENDPOINT_XFER_INT,
		.wMaxPacketSize =	__constant_cpu_to_le16(FIM_USB_MAX_PACK_SIZE),
		.bInterval =		1,
	},
	.ep_in = {
		.bLength =              USB_DT_ENDPOINT_SIZE,
		.bDescriptorType =      USB_DT_ENDPOINT,
		.bEndpointAddress =     FIM_USB_IN_EPNR,
		.bmAttributes =         USB_ENDPOINT_XFER_INT,
		.wMaxPacketSize =	__constant_cpu_to_le16(FIM_USB_MAX_PACK_SIZE),
		.bInterval =		1,
	}
};

static struct fim_usb_string fim_usb_strings [] = {
        {
		.id = STRING_MANUFACTURER,
		.s  = FIM_USB_MANU
	}, {
		.id = STRING_PRODUCT,
		.s  = FIM_USB_PROD_DESC
	}, {
		.id = STRING_SERIALNUM,
		.s  = FIM_USB_SERIALNR
	}
};

/* Send the requested string descriptor to the host */
static int fim_usb_send_string_descriptor(struct fim_usb_port *port, struct fim_usb_ep *ep,
					  struct usb_ctrlrequest *ctrl)
{
	int ret, cnt;
	u8 value;

	ret = -EINVAL;
	value = le16_to_cpu(ctrl->wValue);

	if (value == 0) {
		struct usb_string_descriptor desc;

		desc.bLength = sizeof(desc);
		desc.bDescriptorType = USB_DT_STRING;
		desc.wData[0] = __constant_cpu_to_le16(0x0409);
		ret = fim_usb_write_packet(port, ep, (u8 *)&desc, sizeof(desc));

	} else {
		struct fim_usb_string *pstr;

		pstr = NULL;
		for (cnt = 0; cnt < ARRAY_SIZE(fim_usb_strings); cnt++) {
			if (fim_usb_strings[cnt].id == value) {
				pstr = &fim_usb_strings[cnt];
				break;
			}
		}

		if (!pstr) {
			pk_dbg("Unavailable string descriptor 0x%04x request!\n", value);
			ret = -EINVAL;
			goto exit_str_desc;
		}

		/* OK, we have a string descriptor, send it now! */
		ret = fim_usb_write_packet(port, ep, (u8 *)&pstr->desc, pstr->desc.bLength);
	}

 exit_str_desc:
	return ret;
}

static int fim_usb_handle_ep0_get_descriptor(struct fim_usb_port *port, struct usb_ctrlrequest *ctrl)
{
	u8 desc;
	u16 wvalue, len, total, val;
	struct fim_usb_ep *ep;
	int ret;

	len = le16_to_cpu(ctrl->wLength);
	wvalue = le16_to_cpu(ctrl->wValue);
	desc = wvalue >> 8;
	ep = &port->eps[FIM_USB_NR_EP_CTRL];

	ep->requested = len;
	dbg_enum("Handling get descriptor 0x%02x (len %u)\n", desc, len);

	ret = 0;
	switch (desc) {

		/* Descriptor type device */
	case USB_DT_DEVICE:
		ep->last_data_token = USB_TOKEN_DATA0;
		ret = fim_usb_write_packet(port, ep, (u8 *)&fim_usb_device_desc, sizeof(fim_usb_device_desc));
		break;

		/* Descriptor type configuration */
	case USB_DT_CONFIG:
		ep->last_data_token = USB_TOKEN_DATA0;

		/* Setup the total length of the configuration descriptor */
		total = sizeof(fim_usb_config_desc2);
		val = min(len, total);
		fim_usb_config_desc2.config.wTotalLength = cpu_to_le16(total);

		ret = fim_usb_write_packet(port, ep, (u8 *)&fim_usb_config_desc2, val);
		dbg_enum("Configuration desc. size = %u | sent %u | requested %u\n",
			 total, ret, ep->requested);
		break;

		/* Descriptor type string */
	case USB_DT_STRING:

		ep->last_data_token = USB_TOKEN_DATA0;

		/* According to the spec: by index zero return the table with the supported languages */
		ret = fim_usb_send_string_descriptor(port, ep, ctrl);
		break;

	default:
		pk_err("Unhandled GET descriptor request 0x%02x\n", desc);
		break;
	}

	return ret;
}

static int fim_usb_handle_ep0_standard(struct fim_usb_port *port, struct usb_ctrlrequest *ctrl)
{
	int ret = 0;
	struct fim_usb_ep *ep;
	u8 addr;
	u16 value, idx, type;
	int level;

	ep = &port->eps[FIM_USB_NR_EP_CTRL];

	switch (ctrl->bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;

		dbg_enum("Get descriptor 0x%02x\n", le16_to_cpu(ctrl->wValue));
		ret = fim_usb_handle_ep0_get_descriptor(port, ctrl);
		break;

		/* We must responde with a zero packet length */
	case USB_REQ_SET_ADDRESS:
		addr = le16_to_cpu(ctrl->wValue);
		ep->last_data_token = USB_TOKEN_DATA0;
		fim_usb_write_zlp(port, ep, 0);

		/* @FIXME: Arrgghhh...! Give the FIM some time for sending the ZLP */
		udelay(1500);

		fim_set_ctrl_reg(&port->fim, FIM_USB_ADDR_REG, addr);
		dbg_enum("Set address request (%d)\n", addr);
		break;

		/* @FIXME: Check for the configuration number */
	case USB_REQ_SET_CONFIGURATION:
		value = le16_to_cpu(ctrl->wValue);
		fim_usb_write_zlp(port, ep, 0);
		dbg_enum("Set configuration %u request\n", value);

		/* So, let's create the serial device port */
		schedule_work(&port->uart_work);
		break;

		/* @FIXME: Check for the interface number */
	case USB_REQ_SET_INTERFACE:

		level = fim_tx_buffers_level(&port->fim);
		if (level) {

			/*
			 * That's really bad, cause the host wants to start a new data transfer but
			 * the TX FIFO is not free!
			 */
			pk_dbg("TX FIFO restart for SET interface (level = %i)\n", level);
			schedule_work(&port->restart_work);
			return -ERESTART;
		}

		value = le16_to_cpu(ctrl->wValue);
		fim_usb_write_zlp(port, ep, 0);
		dbg_enum("Set configuration %u request\n", value);
		break;

	case USB_REQ_CLEAR_FEATURE:
		idx = le16_to_cpu(ctrl->wIndex);
		value = le16_to_cpu(ctrl->wValue);
		type = le16_to_cpu(ctrl->bRequest);
		dbg_enum("Clear feature type 0x%x | index 0x%x | value 0x%0x\n",
			 type, idx, value);

		/* Only EPs supported now */
		if (type != 2 && (!idx || idx == FIM_USB_IN_EPNR || idx == FIM_USB_OUT_EPNR))
			fim_usb_write_zlp(port, ep, 0);
		else {
			/* @FIXME: Send a STALL at this point */
			pk_err("Invalid clear feature request (idx 0x%x | type 0x%x)\n", idx, type);
			ret = -EINVAL;
		}

		break;

	default:
		pk_err("Unhandled standard request 0x%02x\n", ctrl->bRequest);
		pk_dump_ctrl_packet(ctrl);
		/* @FIXME: Send a STALL to the host! */
		break;
	}

	return ret;
}

static int fim_usb_handle_ep1_out(struct fim_usb_port *port, u8 *data, int len)
{
	struct tty_struct *tty;
	int count;

	dbg_uart("FIM%i: Writing %i bytes into UART fifo\n", port->index, len);

	tty = tty_from_port(port);
	if (!port->uart_created || !tty) {

		/* This happens when the serial port was not opened yet */
		dbg_uart("Found an uninitialized port %p | tty %p\n", port, tty);
		return -ENODEV;
	}

	/* Don't copy more bytes than there is room for in the buffer */
	count = tty_buffer_request_room(tty, len);
	if (count < len) {
		pk_dbg("Dropping %i chars [ len %i > %i count ]\n", len - count, len, count);
		len = count;

		if (len == 0xfffe)
			return 0;
	}

	tty_insert_flip_string(tty, data, len);
        tty_flip_buffer_push(tty);

	return 0;
}

/* Handle the requests arrived for the EP0 (control EP) */
static int fim_usb_handle_ep0(struct fim_usb_port *port,
			      struct usb_ctrlrequest *pctrl)
{
	int ret = 0;
	u16 windex;

	if (!pctrl || !port)
		return -ENODEV;

	windex = le16_to_cpu(pctrl->wIndex);

	switch (pctrl->bRequestType & USB_TYPE_MASK) {

	case USB_TYPE_STANDARD:
		ret = fim_usb_handle_ep0_standard(port, pctrl);
		break;

	default:
		pk_err("bRequestType 0x%02x | bRequest 0x%02x | "
		       "wValue 0x%04x | wIndex 0x%04x | wLength %u\n",
		       pctrl->bRequestType, pctrl->bRequest,
		       pctrl->wValue, pctrl->wIndex, pctrl->wLength);
		break;
	}

	return ret;
}

static void fim_usb_error_isr(struct fim_driver *driver, ulong rx_err, ulong tx_err)
{
	struct fim_usb_port *port;

	port = driver->driver_data;

	if ((rx_err & IOHUB_IFS_RXNRIP) || port->unplug_detected) {

		if (!work_pending(&port->restart_work)) {
			/* pk_dbg("Scheduling restart worker\n"); */
			schedule_work(&port->restart_work);
		}
	} else
		pk_dbg("Unhandled FIM error (rx 0x%lx | tx 0x%lx)\n", rx_err, tx_err);
}

/*
 * Handler for the incoming FIM-interrupts. Available interrupts:
 * -
 */
static void fim_usb_isr(struct fim_driver *driver, int irq, unsigned char code,
		       unsigned int rx_fifo)
{
	struct fim_usb_port *port;

        dbg_func("FIM IRQ %i\n", irq);
	port = port_from_fim(driver);

	switch (code) {
	default:
		pk_err("Unknown IRQ %i | FIM %i | %x\n",
			   code, port->fim.picnr, rx_fifo);
		break;
	}
}

/*
 * This is the TX-callback that the API call after a DMA-package was closed
 * The fim buffer structure contains our internal private data
 * Free the allocated FIM-buffer that was used for sending the DMA-data
 */
static void fim_usb_tx_isr(struct fim_driver *fim, int irq,
			  struct fim_buffer_t *pdata)
{
	struct fim_usb_port *port;
	struct fim_usb_ep *ep;
	struct uart_port *uart;
	struct circ_buf *xmit;

	dbg_write("TX IRQ\n");

	/* Reset the internal buffer descriptor of the EP */
	port = fim->driver_data;

	/* Here we can start the next data transfer if there is something pending */
	ep = &port->eps[FIM_USB_NR_EP_CTRL];
	if (!ep->addr && ep->bytes) {
		int sent;

		sent = fim_usb_write_packet(port, ep, ep->data, ep->bytes);
		if (sent == FIM_USB_MAX_PACK_SIZE && !ep->bytes && ep->requested) {
			fim_usb_write_zlp(port, ep, 1);
			dbg_enum("ZLP for big packet (request pending %u)\n", ep->requested);
		}

		/* @XXX: Should we return at this point? */
		/* return; */
	}

	/* Schedule the work queue at this point if there is some remaining data */
	ep = &port->eps[FIM_USB_NR_EP_IN];
	uart = ep->tx_uart;
	if (uart) {

		/* The worker will wakeup the device if the circular buffer is empty */
		xmit = &uart->state->xmit;
		if (xmit && !uart_circ_empty(xmit) && !uart_tx_stopped(uart)) {
			dbg_uart("EP%02x Scheduling TX work\n", ep->addr);
			schedule_work(&ep->tx_work);
		}
	}
}

static int fim_usb_restart_fim(struct fim_usb_port *port)
{
        int ret;
	struct fim_driver *fim;

        if (!port)
                return -ENODEV;

	fim = &port->fim;

	fim_usb_disable_pullup(port);

	/* Reset the main register */
	fim_usb_regs_reset(port);

	/* First try to stop the FIM */
	fim_disable_irq(fim);
	ret = fim_send_stop(fim);
        if (ret) {
		pk_err("Couldn't stop the FIM%i\n", fim->picnr);
                return ret;
	}

	fim_dma_stop(&port->fim);
	fim_dma_start(&port->fim, NULL);

        ret = fim_send_start(fim);
        if (ret) {
		pk_err("Couldn't start the FIM%i\n", fim->picnr);
                return -EAGAIN;
	}

	/* Restart the DMA channel when we know that we are safe! */
        fim_enable_irq(fim);

	/* This function resets the address number to zero */
	fim_usb_init_port(port);

	/* This will enable the start of the FIM in the firmware */
	fim_set_ctrl_reg(&port->fim, FIM_USB_MAIN_REG,
			 FIM_USB_MAIN_START |  FIM_USB_MAIN_NOKEEPALIVE | FIM_USB_MAIN_NOINIRQ);

	fim_usb_enable_pullup(port);

	return ret;
}

static void fim_usb_restart_work_func(struct work_struct *work)
{
	struct fim_usb_port *port;

	port = container_of(work, struct fim_usb_port, restart_work);
	fim_usb_restart_fim(port);
	complete(&port->restart_completed);
}

/* Handle the bus resets reported by the FIM */
static void fim_usb_bus_reset(struct fim_usb_port *port)
{
	unsigned int addr;
	int level;

	dbg_enum("Bus reset (addr = 0)\n");

	/* We need to restore the TX buffers at this point */
	fim_get_ctrl_reg(&port->fim, FIM_USB_ADDR_REG, &addr);
	level = fim_tx_buffers_level(&port->fim);
	if (level && addr) {
		dbg_enum("Need to restart the FIM (level %i, curr addr. %u)\n",
			 level, addr);
		schedule_work(&port->restart_work);
	} else {
		addr = 0;
		fim_set_ctrl_reg(&port->fim, FIM_USB_ADDR_REG, addr);
	}
}

/*
 * Called when a receive DMA-buffer was closed.
 * The first byte contains the status of the closed buffer.
 */
static void fim_usb_rx_isr(struct fim_driver *fim, int irq,
			   struct fim_buffer_t *pdata)
{
	u8 status, err;
	struct usb_ctrlrequest *pctrl;
	struct fim_usb_port *port;
	struct fim_usb_ep *ep;
	struct fim_usb_token *token;

	port = fim->driver_data;
	status = *(pdata->data);
	err = *(pdata->data + pdata->length);
	token = (struct fim_usb_token *)(pdata->data + 1);

	if (pdata->length == 1) {

		switch (status) {

		case FIM_USB_BUS_RESET:
			dbg_pr("Bus reset received!\n");
			fim_usb_bus_reset(port);
			break;

		case FIM_USB_KEEP_ALIVE:
			dbg_pr("Alive\n");
			break;

		case FIM_USB_IN_DATA_SENT_MARKER:
			dbg_write("FIM sending IN frame\n");
			break;

		default:
			dbg_pr("Unexpected status 0x%x\n", status);
			break;
		}

	} else {

		switch (token->type) {

		case USB_TOKEN_DATA0:
		case USB_TOKEN_DATA1:
			dbg_token("DATA%c %u bytes for EP%u\n",
			       (token->type == USB_TOKEN_DATA0) ? '0' : '1',
			       pdata->length, port->ep_next_data);

			/* The complete frame contains the sync byte, the CRC and the error */
			if (port->ep_next_data == 0) {

				/* @XXX: Stupid sanity check for let it be for now */
				if (pdata->length != FIM_USB_DATA_OVERHEAD && pdata->length != FIM_USB_DATA_TAIL) {
					pctrl = (struct usb_ctrlrequest *)(pdata->data + 2);
					fim_usb_handle_ep0(port, pctrl);
				}

			} else {
				struct fim_usb_data *fdata;
				u16 bytes;

				/* Remove the overhead from the data frame */
				bytes = pdata->length - FIM_USB_DATA_OVERHEAD;
				fdata = (struct fim_usb_data *)pdata->data;

				/* We have only one EP, so let us call it directly from here */
				fim_usb_handle_ep1_out(port, pdata->data + FIM_USB_DATA_HEAD, bytes);
			}
			break;

			/* Get the endpoint number for the incoming data */
		case USB_TOKEN_OUT:
			dbg_token("OUT token for EP %x\n", token->ep);
			port->ep_next_data = token->ep;

			/* Reset the data token of the IN EP81 */
			ep = &port->eps[FIM_USB_NR_EP_IN];
			ep->last_data_token = USB_TOKEN_DATA1;
			break;

			/* DONT print the IN tokens cause they are generated continously */
		case USB_TOKEN_IN:
			/* dbg_token("IN token for EP %x\n", token->ep); */
			port->ep_next_data = token->ep;
			break;

		case USB_TOKEN_SETUP:
			port->ep_next_data = 0;
			/* dbg_token("SETUP token for EP %x\n", token->ep); */

			/* Reset the data token of the IN EP81 */
			ep = &port->eps[FIM_USB_NR_EP_IN];
			ep->last_data_token = USB_TOKEN_DATA1;
			break;

			/* Here the tokens that only generate noise */
		case USB_TOKEN_SYNC:
		default:
			break;
		}
	}
}

static int fim_usb_unregister_port(struct fim_usb_port *port)
{
	int cnt, ret;
	struct fim_driver *fim;
	struct uart_port *uart;

	if (!port || !port->reg)
		return -ENODEV;

	fim = &port->fim;
	pk_dbg("Going to unregister the FIM USB port %i\n", fim->picnr);

	flush_work(&port->uart_work);
	cancel_work_sync(&port->restart_work);
	cancel_delayed_work_sync(&port->hotplug_work);

	/* Disable the FIM from the connected host */
	fim_usb_disable_pullup(port);

	ret = fim_unregister_driver(fim);
	if (ret)
		goto exit_unreg;

	/* Unregister the UART port */
	if (port->uart_created) {
		dbg_uart("Removing the port %i\n", fim->picnr);
		uart = &port->uart;
		uart_remove_one_port(&fim_usb_uart_driver, uart);
		port->uart_created = 0;
	}

	/* Free the requested GPIO */
	for (cnt = 0; cnt < FIM_USB_MAX_GPIOS; cnt++) {
		dbg_pr("Freeing GPIO %i\n", port->gpios[cnt].nr);
                if (port->gpios[cnt].nr == FIM_LAST_GPIO)
                        break;
                else if (port->gpios[cnt].nr == FIM_GPIO_DONT_USE)
                        continue;
                else
                        gpio_free(port->gpios[cnt].nr);
	}

	/* Free the requested clock */
	if (!IS_ERR(port->clk))
	        clk_put(port->clk);

	fim_usb_ports[fim->picnr] = NULL;
	kfree(port);
	ret = 0;

 exit_unreg:
	return ret;
}

/*
 * Called when the port is opened. By the boot-console it will be called before
 * the port configuration using set_termios
 */
static int fim_usb_uart_startup(struct uart_port *uart)
{
        struct fim_usb_port *port;
	int ret = 0;

	dbg_uart("Calling %s()\n", __func__);
        port = port_from_uart(uart);
	if (!port)
		return -ENODEV;

#if defined(FIM_USB_FORCE_ENUM_TEST)
	/* This will restart the enumeration sequence */
	pk_info("Forcing the enumeration test!\n");
	schedule_work(&port->restart_work);
	ret = -ENODEV;
#else
	/*
	 * We must flush the TX FIFO when there is some data pending.
	 * @XXX: Use a waitqueue with timeout for returning a failure,
	 * otherwise the user must reopen the port (annoying)
	 */
	if (fim_tx_buffers_level(&port->fim)) {
		pk_dbg("TX FIFO contains data, FIM restart required.\n");
		schedule_work(&port->restart_work);
		wait_for_completion(&port->restart_completed);

		pk_dbg("Restart completed. Re-checking TX FIFO state.\n");
		if (fim_tx_buffers_level(&port->fim)) {
			pk_dbg("TX FIFO invalid state. Retry required.\n");
			ret = -EAGAIN;
		}
	}
#endif /* FIM_USB_FORCE_ENUM_TEST */

	return ret;
}

/* Return a string describing the type of the port */
static const char *fim_usb_uart_type(struct uart_port *port)
{
	dbg_uart("Calling %s\n", __func__);
        return FIM_DRIVER_NAME;
}

static void fim_usb_uart_release_port(struct uart_port *port)
{
        dbg_uart("Calling %s\n", __func__);
}

static int fim_usb_uart_request_port(struct uart_port *port)
{
        dbg_uart("Calling %s\n", __func__);
        return 0;
}

/* @TODO: Get more infos about the UART configuration */
static void fim_usb_uart_config_port(struct uart_port *port, int flags)
{
        dbg_uart("Calling %s\n", __func__);
        port->type = UPIO_MEM;
}

static int fim_usb_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
        dbg_uart("Verify port called\n");
        return 0;
}

/* Called for setting the termios config */
static void fim_usb_uart_set_termios(struct uart_port *uart, struct ktermios *termios,
				     struct ktermios *old)
{
        dbg_uart("Calling %s\n", __func__);
}

static unsigned int fim_usb_uart_tx_empty(struct uart_port *uart)
{
        return TIOCSER_TEMT;
}

/* Handle the data transfer from the UART circular buffer to the USB host (IN transfers) */
static void fim_usb_uart_tx_work_func(struct work_struct *work)
{
	struct fim_usb_ep *ep;
	struct circ_buf *xmit;
	u8 buf[FIM_USB_MAX_PACK_SIZE];
	struct uart_port *uart;
	ulong pending, pos;
	int ret, tail;
	struct fim_usb_port *port;
	__u32 tx;

	ep = container_of(work, struct fim_usb_ep, tx_work);
	port = ep->port;

	if (down_interruptible(&port->uart_sem))
		return;

	uart = ep->tx_uart;
	xmit  = &uart->state->xmit;
	tail = xmit->tail;
	tx = uart->icount.tx;
	pending = uart_circ_chars_pending(xmit);
	pos = ret = 0;

	while (pending--) {

		buf[pos++] = xmit->buf[tail];

		if (pos == FIM_USB_MAX_PACK_SIZE || pending == 0) {

			/*
			 * Check if there is enough space in the DMA buffers for the next
			 * transfer request. Otherwise we must wait for the completion of
			 * some TX transfer and continue sending the data
			 */
			while (fim_tx_buffers_level(&port->fim)) {
				schedule_timeout_interruptible(HZ / 1000);
				if (signal_pending(current))
					goto exit_tx_worker;
			}

			/* Send the data to the FIM */
			dbg_uart("EP%02x Sending %lu bytes | remaining %lu\n", ep->addr, pos, pending);
			ret = fim_usb_write_packet(port, ep, buf, pos);
			if (ret <= 0) {
				pr_err("Couldn't send the buffer (err %i)\n", ret);
				goto exit_tx_worker;
			}
		}

		tail = (tail + 1) & (UART_XMIT_SIZE - 1);
		tx++;

		/* Only one messages per call! */
		if (pos == FIM_USB_MAX_PACK_SIZE || pending == 0)
			break;
	}

	/* OK, it looks we have sent some data, so update the UART structure */
	xmit->tail = tail;
	uart->icount.tx = tx;

	/* Check for the ZLP condition on this EP */
	if (uart_circ_empty(xmit) && pos == FIM_USB_MAX_PACK_SIZE) {
		dbg_uart("Need to send a ZLP\n");

		/* @XXX: Quick and dirty! */
		/* Wait for an empty TX FIFO cause we can't queue messages */
		while (fim_tx_buffers_level(&port->fim)) {
			schedule_timeout_interruptible(HZ / 1000);
			if (signal_pending(current))
				goto exit_tx_worker;
		}

		fim_usb_write_zlp(port, ep, 0);
	}

	/*
	 * Tell the higher layer that we can send more data
	 * IMPORTANT: We need to check if TX was stopped before calling the wakeup cause
	 * the bove ZLP write function seems to take a lot of time and we have probably
	 * received a signal during this time
	 */
        if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS && !uart_tx_stopped(uart)) {
		uart_write_wakeup(uart);
	}

 exit_tx_worker:
	up(&port->uart_sem);
}

/* This function is called when the TTY layer has some data to be sent */
static void fim_usb_uart_start_tx(struct uart_port *uart)
{
	ulong pending;
	struct fim_usb_ep *ep;
	struct fim_usb_port *port;
	struct circ_buf *xmit;

	port = port_from_uart(uart);

	/*
	 * IMPORTANT: The data token is modified in the function that puts the data
	 * buffer to the FIM-core.
	 */
	ep = &port->eps[FIM_USB_NR_EP_IN];
	ep->tx_uart = uart;

	xmit  = &uart->state->xmit;
	pending = uart_circ_chars_pending(xmit);

	dbg_uart("Scheduling TX worker | %lu bytes\n", pending);
	schedule_work(&ep->tx_work);
}

static unsigned int fim_usb_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void fim_usb_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
        /* @TODO? */
	dbg_uart("Calling %s\n", __func__);
}

static void fim_usb_uart_stop_tx(struct uart_port *port)
{
	/* flush_work(&port->uart_work); */
	dbg_uart("Calling %s\n", __func__);
}

static void fim_usb_uart_stop_rx(struct uart_port *port)
{
	dbg_uart("Calling %s\n", __func__);
}

static void fim_usb_uart_enable_ms(struct uart_port *port)
{
        /* @TODO? */
	dbg_uart("Calling %s\n", __func__);
}

static void fim_usb_uart_break_ctl(struct uart_port *port, int ctl)
{
        /* @TODO? */
	dbg_uart("Calling %s\n", __func__);
}

static void fim_usb_uart_shutdown(struct uart_port *uart)
{
	struct fim_usb_port *port;
	struct fim_usb_ep *ep;

	port = port_from_uart(uart);
	ep = &port->eps[FIM_USB_NR_EP_IN];

	flush_work(&ep->tx_work);
	dbg_uart("Calling %s\n", __func__);
}

static struct uart_ops fim_usb_uart_ops = {
        .tx_empty       = fim_usb_uart_tx_empty,
        .set_mctrl      = fim_usb_uart_set_mctrl,
        .get_mctrl      = fim_usb_uart_get_mctrl,
        .stop_tx        = fim_usb_uart_stop_tx,
        .start_tx       = fim_usb_uart_start_tx,
        .stop_rx        = fim_usb_uart_stop_rx,
        .enable_ms      = fim_usb_uart_enable_ms,
        .break_ctl      = fim_usb_uart_break_ctl,
	.startup        = fim_usb_uart_startup,
        .shutdown       = fim_usb_uart_shutdown,
        .set_termios    = fim_usb_uart_set_termios,
	.type           = fim_usb_uart_type,
        .release_port   = fim_usb_uart_release_port,
        .request_port   = fim_usb_uart_request_port,
	.config_port    = fim_usb_uart_config_port,
        .verify_port    = fim_usb_uart_verify_port
};

static int fim_usb_register_port(struct device *dev, int picnr, struct fim_gpio_t gpios[])
{
	int ret, cnt;
	struct fim_dma_cfg_t dma_cfg;
	struct fim_usb_port *port;
	unsigned int fwver;

	port = kzalloc(sizeof(* port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	/* Get a reference to the SYS clock for setting CLK rate */
	ret = IS_ERR(port->clk = clk_get(port->fim.dev, "systemclock"));
	if (ret) {
		pk_err("Couldn't get the SYS clock.\n");
		goto err_free_port;
	}

	/* Request GPIO */
	for (cnt = 0; cnt < FIM_USB_MAX_GPIOS; cnt++) {

		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;

		dbg_pr("Requesting the GPIO %i (Function %i)\n",
		       gpios[cnt].nr, gpios[cnt].func);
		ret = gpio_request(gpios[cnt].nr, FIM_DRIVER_NAME);
		if (!ret) {
			int dir, pullup;

			/*
			 * According to the documentation, the pins ENUM and SUSPEND
			 * must be controlled by the driver (they are not by the
			 * FIM-firmware).
			 */
			if (cnt == FIM_USB_GPIO_ENUM || cnt == FIM_USB_GPIO_SPND) {
				dir = NS921X_GPIO_OUTPUT;
				pullup = NS921X_GPIO_DISABLE_PULLUP;
			} else {
				dir = NS921X_GPIO_INPUT;
				pullup = NS921X_GPIO_ENABLE_PULLUP;
			}

			gpio_configure_ns921x_unlocked(gpios[cnt].nr,
						       dir,
						       NS921X_GPIO_DONT_INVERT,
						       gpios[cnt].func,
						       pullup);
		} else {
			/* Free the already requested GPIOs */
			pk_err("Couldn't request the GPIO %i\n", gpios[cnt].nr);
			while (cnt)
				gpio_free(gpios[--cnt].nr);
			goto err_free_clk;
		}
	}

	/* Try to register FIM driver */
	port->index		= picnr;
	port->fim.picnr		= picnr;
	port->fim.driver.name	= FIM_DRIVER_NAME;
	port->fim.driver_data	= port;
	port->fim.fim_isr	= fim_usb_isr;
	port->fim.dma_tx_isr	= fim_usb_tx_isr;
	port->fim.dma_rx_isr	= fim_usb_rx_isr;
	port->fim.dma_error_isr	= fim_usb_error_isr;
	port->fim.driver_data	= port;

	/* Specific DMA config */
	dma_cfg.rxnr = FIM_USB_DMA_RX_BUFFERS;
	dma_cfg.txnr = FIM_USB_DMA_TX_BUFFERS;
	dma_cfg.rxsz = FIM_USB_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_USB_DMA_BUFFER_SIZE;
	port->fim.dma_cfg = &dma_cfg;

	/* Check if have a firmware code for using to */
	port->fim.fw_name = FIM_USB_FIRMWARE_FILE;
	port->fim.fw_code = FIM_USB_FIRMWARE_CODE;
	ret = fim_register_driver(&port->fim);
	if (ret) {
		pk_err("Couldn't register the FIM %i USB driver.\n", picnr);
		goto err_free_gpios;
	}

	port->dev = dev;
	memcpy(port->gpios, gpios, sizeof(port->gpios));

	port->reg = 1;
	dev_set_drvdata(dev, port);

	INIT_WORK(&port->uart_work, fim_usb_uart_work_func);
	INIT_WORK(&port->restart_work, fim_usb_restart_work_func);
	init_completion(&port->restart_completed);
	init_MUTEX(&port->uart_sem);

        /* This is the timer for checking the state of the card detect line */
	INIT_DELAYED_WORK(&port->hotplug_work, fim_usb_hotplug_work_func);

	/* And enable the FIM-interrupt */
	ret = fim_enable_irq(&port->fim);
	if (ret) {
		pk_err("Couldn't enable the FIM IRQ\n");
		goto err_unreg_fim;
	}

	/* First do it at this point! */
	fim_usb_disable_pullup(port);
	fim_usb_init_port(port);

	/* Print the firmware version */
        fim_get_stat_reg(&port->fim, FIM_USB_REVISION_REG, &fwver);
        pk_dbg("FIM%d running [fw rev 0x%02x]\n", picnr, fwver);

	fim_usb_ports[picnr] = port;

	/* This will enable the start of the FIM in the firmware */
	fim_set_ctrl_reg(&port->fim, FIM_USB_MAIN_REG,
			 FIM_USB_MAIN_START |  FIM_USB_MAIN_NOKEEPALIVE | FIM_USB_MAIN_NOINIRQ);
	fim_usb_enable_pullup(port);
	return 0;

 err_unreg_fim:
	fim_unregister_driver(&port->fim);

 err_free_gpios:
	for (cnt = 0; cnt < FIM_USB_MAX_GPIOS; cnt++) {

		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;

		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;

		gpio_free(gpios[cnt].nr);
	}

 err_free_clk:
	clk_put(port->clk);

 err_free_port:
	kfree(port);

	return ret;
}

static int __devinit fim_usb_probe(struct platform_device *pdev)
{
        int nrpics;
	struct fim_usb_platform_data *pdata;
	struct fim_gpio_t gpios[FIM_USB_MAX_GPIOS];

	dbg_func("New device with ID %i\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

        /* Get the number of available PICs from the FIM-core */
        nrpics = fim_number_pics();

	if (fim_check_device_id(fims_number, pdata->fim_nr)) {
#if defined(MODULE)
                pk_dbg("Skipping FIM%i (not selected)\n", pdata->fim_nr);
#else
                pk_err("Invalid FIM number '%i' in platform data\n", pdata->fim_nr);
#endif

                return -ENODEV;
        }

	/*
	 * Start with the registration of the pdata ports
	 */
	gpios[FIM_USB_GPIO_DP].nr	= pdata->vp_gpio_nr;
	gpios[FIM_USB_GPIO_DP].func	= pdata->vp_gpio_func;
	gpios[FIM_USB_GPIO_DM].nr	= pdata->vm_gpio_nr;
	gpios[FIM_USB_GPIO_DM].func	= pdata->vm_gpio_func;
	gpios[FIM_USB_GPIO_RCV].nr	= pdata->rcv_gpio_nr;
	gpios[FIM_USB_GPIO_RCV].func	= pdata->rcv_gpio_func;
	gpios[FIM_USB_GPIO_OE].nr	= pdata->oe_l_gpio_nr;
	gpios[FIM_USB_GPIO_OE].func	= pdata->oe_l_gpio_func;
	gpios[FIM_USB_GPIO_ENUM].nr	= pdata->enum_gpio_nr;
	gpios[FIM_USB_GPIO_ENUM].func	= pdata->enum_gpio_func;
	gpios[FIM_USB_GPIO_SPND].nr	= pdata->spnd_gpio_nr;
	gpios[FIM_USB_GPIO_SPND].func	= pdata->spnd_gpio_func;

	dbg_pr("Pins: DP %d | DM %d | RCV %d | OE %d | ENUM %d | SPND %d\n",
	       gpios[0].nr, gpios[1].nr, gpios[2].nr,
	       gpios[3].nr, gpios[4].nr, gpios[5].nr );

	return fim_usb_register_port(&pdev->dev, pdata->fim_nr, gpios);
}

static int __devexit fim_usb_remove(struct platform_device *pdev)
{
	struct fim_usb_port *port;
	struct fim_usb_platform_data *pdata;
	int retval;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	port = dev_get_drvdata(&pdev->dev);
	if (!port)
		return -ENXIO;

	retval = fim_usb_unregister_port(port);
	if (retval)
		goto exit_remove;

	platform_set_drvdata(pdev, NULL);

 exit_remove:
	return retval;
}

#ifdef	CONFIG_PM
/* Here we need to desconnect from the host and stop the FIM */
static int fim_usb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fim_usb_port *port;
	int ret;
	struct fim_driver *fim;

	port = dev_get_drvdata(&pdev->dev);
	if (!port)
		return -ENXIO;

	cancel_delayed_work_sync(&port->hotplug_work);

	/* Disable the device and stop the FIM */
	fim_usb_disable_pullup(port);

	/* Stop the FIM now */
	fim = &port->fim;
	ret = fim_send_stop(fim);
	if (ret)
		pk_err("Couldn't stop the FIM %i\n", fim->picnr);

	return ret;
}

static int fim_usb_resume(struct platform_device *pdev)
{
	struct fim_usb_port *port;

	port = dev_get_drvdata(&pdev->dev);
	if (!port)
		return -ENXIO;

	return fim_usb_restart_fim(port);
}
#else
#define	fim_usb_suspend	NULL
#define	fim_usb_resume	NULL
#endif

static struct platform_driver fim_usb_driver = {
	.driver		= {
		.name	= FIM_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe          = fim_usb_probe,
	.remove		= __devexit_p(fim_usb_remove),
	.suspend	= fim_usb_suspend,
	.resume		= fim_usb_resume,
};

static int __init fim_usb_init(void)
{
	int cnt, pos, ret, pics;
	struct fim_usb_string *pstr;
	u16 *pdata;

        /* Check for the passed number parameter */
        if (fim_check_numbers_param(fims_number)) {
                pk_err("Invalid number '%i' of FIMs to handle\n", fims_number);
                return -EINVAL;
        }

	pk_info("%s: version %s [%s | %s]\n", FIM_DRIVER_NAME, DRIVER_VERSION,
		__DATE__, __TIME__);

	/* For having an easier descriptors handling we only want to use static data buffers */
	for (cnt = 0; cnt < ARRAY_SIZE(fim_usb_strings); cnt++) {
		pstr = &fim_usb_strings[cnt];
		pdata = pstr->data;

		for (pos = 0; pos < strlen(pstr->s); pos++)
			*(pdata + pos) = cpu_to_le16(*(pstr->s + pos));

		pstr->desc.bLength = sizeof(pstr->desc) + (2 * pos);
	}

	pics = fim_number_pics();
	fim_usb_ports_max = pics;
	fim_usb_ports = kzalloc(pics, sizeof(*fim_usb_ports));
	if (!fim_usb_ports)
		return -ENOMEM;

	/* We need a reboot notifier for disabling the pullup */
	ret = register_reboot_notifier(&fim_usb_reboot_notifier);
	if (ret) {
		pk_err("Reboot notifier register failed, %i\n", ret);
		goto err_exit_init;
	}

	/* Register the internal UART driver */
        /* Get the number of maximal available FIMs */
        fim_usb_uart_driver.owner = THIS_MODULE;
        fim_usb_uart_driver.driver_name = FIM_DRIVER_NAME;
        fim_usb_uart_driver.dev_name = FIM_USB_UART_DEV_NAME;
        fim_usb_uart_driver.nr = pics;
        ret = uart_register_driver(&fim_usb_uart_driver);
        if (ret)
		goto err_unreg_noti;

	ret = platform_driver_register(&fim_usb_driver);
	if (ret)
		goto err_unreg_uart;

	return 0;

 err_unreg_uart:
	uart_unregister_driver(&fim_usb_uart_driver);

 err_unreg_noti:
	unregister_reboot_notifier(&fim_usb_reboot_notifier);

 err_exit_init:
	kfree(fim_usb_ports);
	return ret;
}

static void __exit fim_usb_exit(void)
{
	pk_info("Removing the FIM USB driver [%s | %s]\n", __DATE__, __TIME__);
	platform_driver_unregister(&fim_usb_driver);
	uart_unregister_driver(&fim_usb_uart_driver);

	/* Free the resources for the reboot notifier */
	unregister_reboot_notifier(&fim_usb_reboot_notifier);
	kfree(fim_usb_ports);
}

module_init(fim_usb_init);
module_exit(fim_usb_exit);

MODULE_DESCRIPTION(FIM_DRIVER_DESC);
MODULE_AUTHOR(FIM_DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fim_usb");

