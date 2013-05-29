/*
 * drivers/usb/gadget/s3c-udc.h
 * Samsung S3C on-chip full/high speed USB device controllers
 * $Id: s3c-udc.h,v 1.6 2006/11/30 22:55:18 dasan Exp $*
 * Copyright (C) 2005 for Samsung Electronics 
 * 		- by Jaswinder Singh Brar <jassi@samsung.com>
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
 *
 */
 
#ifndef __S3C_USB_GADGET
#define __S3C_USB_GADGET

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-fns.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>


#include <plat/regs-udc.h>
#include <plat/udc.h>
#include <linux/io.h>
#include <mach/regs-s3c2443-clock.h>
#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>


// Max packet size
#ifdef CONFIG_USB_GADGET_S3C_FS
#define EP0_FIFO_SIZE		8
#define EP_FIFO_SIZE		64
#define S3C_MAX_ENDPOINTS	5
#else
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define EP_FIFO_SIZE2		1024
#define S3C_MAX_ENDPOINTS	9
#endif

#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4





typedef enum ep_type {
	ep_control,
	ep_bulk_in,
	ep_bulk_out,
	ep_interrupt
} ep_type_t;



struct s3c_ep {
	struct usb_ep ep;
	struct s3c24xx_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;

	u8 stopped;
	u8 bEndpointAddress;
	u8 bmAttributes;

	ep_type_t ep_type;
	u32 fifo;
#ifdef CONFIG_USB_GADGET_S3C_FS
	u32 csr1;
	u32 csr2;
#endif

	/* Tasklet for the data handling of the IN-endpoints (Luis Galdos) */
	struct tasklet_struct in_tasklet;
	spinlock_t lock;
};



struct s3c_request {
	struct usb_request req;
	struct list_head queue;
};



struct s3c24xx_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *dev;
	spinlock_t lock;

	int ep0state;
	struct s3c_ep ep[S3C_MAX_ENDPOINTS];

	unsigned char usb_address;

	unsigned req_pending:1, req_std:1, req_config:1;

	
	struct s3c2410_udc_mach_info *mach_info;
	struct resource *mem;
	void __iomem *base;
	u8 vbus;
	
	/* Interrupts for bus detection and UDC internal */
	int irq_vbus;
	int irq_udc;
};

extern struct s3c24xx_udc *the_controller;

#define ep_is_in(ep) 		(((ep)->bEndpointAddress&USB_DIR_IN) == USB_DIR_IN)
#define ep_index(ep) 		((ep)->bEndpointAddress&0xF)
#define ep_maxpacket(ep) 	((ep)->ep.maxpacket)

#endif
