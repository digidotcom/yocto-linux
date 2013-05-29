/* -*- linux-c -*-
 *
 * drivers/usb/gadget/s3c24xx_udc.c
 *
 * Samsung S3C on-chip full/high speed USB device controllers
 *
 * $Id: s3c-udc-hs.c,v 1.26 2007/02/22 09:45:04 ihlee215 Exp $*
 *
 * Copyright (C) 2006 for Samsung Electronics
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

#include "s3c2443_udc.h"
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/bug.h>

/* @TODO: USB Device DMA support */
#define RX_DMA_MODE 0
#define TX_DMA_MODE 0

#if 0
#define DEBUG_S3C2443_UDC
#endif

#define pk_err(fmt, args...)			printk(KERN_ERR "[ ERROR ] s3c2443-udc: " fmt, ## args)
#define pk_info(fmt, args...)			printk(KERN_DEBUG "s3c2443-udc: " fmt, ## args)

#ifdef DEBUG_S3C2443_UDC
#define pk_dbg(fmt, args...)			printk(KERN_DEBUG "s3c2443-udc: %s() " fmt, __func__ , \
							## args)
#else
#define pk_dbg(fmt, args...)			do { } while(0)
#endif

#if 0
#define S3C2443_UDC_DBG_OUT
#endif

#if defined(S3C2443_UDC_DBG_OUT)
#define pk_dbg_out(fmt, args...)		printk(KERN_DEBUG "[OUT] " fmt, ## args)
#else
#define pk_dbg_out(fmt, args...)		do { } while(0)
#endif /* S3C2443_UDC_DBG_OUT */

/*
 * This macro enables the debug messages when the driver is going to access to the
 * internal queue of the IN-endpoints
 */
#if 0
#define DEBUG_S3C2443_UDC_QUEUE
#endif

/* Some driver infos */
#define	DRIVER_DESC		                "S3C2443 Dual-speed USB Device"
#define DRIVER_NAME                             "s3c2443_udc"
#define	DRIVER_BUILD_TIME	                 __TIME__
#define	DRIVER_BUILD_DATE	                 __DATE__

#define IOMEMSIZE(s)		                (s->end - s->start + 1)

/* Internal variables */
struct s3c24xx_udc *the_controller;
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

/* Max packet sizes */
static u32 ep0_fifo_size = 64;
static u32 ep_fifo_size =  512;
static u32 ep_fifo_size2 = 1024;

/* Internal functions */
static int s3c24xx_udc_ep_enable(struct usb_ep *ep,
				 const struct usb_endpoint_descriptor *);
static int s3c24xx_udc_ep_disable(struct usb_ep *ep);
static struct usb_request *s3c24xx_udc_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
static void s3c24xx_udc_free_request(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc_queue(struct usb_ep *ep, struct usb_request *, gfp_t gfp_flags);
static int s3c24xx_udc_dequeue(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc_set_halt(struct usb_ep *ep, int);
static int s3c24xx_udc_fifo_status(struct usb_ep *ep);
static void s3c24xx_udc_fifo_flush(struct usb_ep *ep);
static void s3c24xx_udc_ep0_kick(struct s3c24xx_udc *udc, struct s3c_ep *ep);
static void s3c24xx_handle_ep0(struct s3c24xx_udc *udc);
static void done(struct s3c_ep *ep, struct s3c_request *req, int status);
static void stop_activity(struct s3c24xx_udc *dev, struct usb_gadget_driver *driver);
static int s3c24xx_udc_enable(struct s3c24xx_udc *udc);
static void s3c24xx_udc_set_address(struct s3c24xx_udc *dev, unsigned char address);
static void reconfig_usbd(struct s3c24xx_udc *udc);
static void s3c24xx_ep0_setup(struct s3c24xx_udc *udc);
static int s3c24xx_udc_write_fifo(struct s3c_ep *ep, struct s3c_request *req);


static inline struct s3c24xx_udc *gadget_to_udc(struct usb_gadget *gadget)
{
        return container_of(gadget, struct s3c24xx_udc, gadget);
}

static spinlock_t regs_lock = SPIN_LOCK_UNLOCKED;

static inline void s3c2443_print_err_packet_setup(int errcode,
						  struct usb_ctrlrequest *pctrl)
{
	printk(KERN_DEBUG "[ ERROR ] s3c2443-udc: Err %i | bRequestType 0x%02x | "
	       "bRequest 0x%02x | wValue 0x%04x | wIndex 0x%04x | wLength %u\n",
	       errcode, pctrl->bRequestType, pctrl->bRequest,
	       pctrl->wValue, pctrl->wIndex, pctrl->wLength);
}

/* Read access to one of the indexed registers */
static inline ulong usb_read(struct s3c24xx_udc *udc, ulong port, u8 ind)
{
	ulong retval;

	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	retval = readl(udc->base + port);
	spin_unlock(&regs_lock);
	return retval;
}

/* Write access to one of the indexed registers */
static inline void usb_write(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(val, udc->base + port);
	spin_unlock(&regs_lock);
}

static inline void usb_set(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(readl(udc->base + port) | val, udc->base + port);
	spin_unlock(&regs_lock);
}

static inline void usb_clear(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(readl(udc->base + port) & ~val, udc->base + port);
	spin_unlock(&regs_lock);
}

/* Return a value different than zero if the EP is enabled */
static inline int s3c24xx_ep_enabled(struct s3c24xx_udc *udc, int epnr)
{
	ulong regval;

	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	return (regval & (1 << epnr));
}

/* Enable/disable the interrupt of the passed EP-number */
static inline void s3c24xx_ep_irq_enable(struct s3c24xx_udc *udc, int epnr, int enable)
{
	ulong eier;

	eier = readl(udc->base + S3C24XX_UDC_EIER_REG);
	if (enable)
		eier |= (1 << epnr);
	else
		eier &= ~(1 << epnr);
	writel(eier, udc->base + S3C24XX_UDC_EIER_REG);
}

static inline void s3c2443_udc_print_regs(char *marke, struct s3c24xx_udc *udc, int epnr)
{
	struct regs_t {
		char *name;
		ulong addr;
	};

	int pos, old_epnr;
	ulong regval;
	static const struct regs_t regs[] = {
		{ "EIR   ", S3C24XX_UDC_EIR_REG },
		{ "EIER  ", S3C24XX_UDC_EIER_REG },
		{ "EDR   ", S3C24XX_UDC_EDR_REG },
		{ "TR    ", S3C24XX_UDC_TR_REG },
		{ "SSR   ", S3C24XX_UDC_SSR_REG },
		{ "SCR   ", S3C24XX_UDC_SCR_REG },
		{ "EP0SR ", S3C24XX_UDC_EP0SR_REG },
		{ "FCON  ", S3C24XX_UDC_FIFO_CON_REG },
		{ "FSTAT ", S3C24XX_UDC_FIFO_STATUS_REG },
		{ "ESR   ", S3C24XX_UDC_ESR_REG },
		{ "ECR   ", S3C24XX_UDC_ECR_REG },
		{ "BRCR  ", S3C24XX_UDC_BRCR_REG },
		{ "BWCR  ", S3C24XX_UDC_BWCR_REG },
	};

	/* First get a backup of the current EP number */
	old_epnr = readl(udc->base + S3C24XX_UDC_IR_REG);
	writel(epnr, udc->base + S3C24XX_UDC_IR_REG);

	/* Now print all the registers */
	printk(KERN_DEBUG "%s\n", marke);
	for (pos = 0; pos < ARRAY_SIZE(regs); pos++) {
		regval = usb_read(udc, regs[pos].addr, epnr);
		printk(KERN_DEBUG "%s: 0x%08lx\n", regs[pos].name, regval);
	}

	writel(old_epnr, udc->base + S3C24XX_UDC_IR_REG);
}

static struct usb_ep_ops s3c24xx_ep_ops = {
	.enable = s3c24xx_udc_ep_enable,
	.disable = s3c24xx_udc_ep_disable,

	.alloc_request = s3c24xx_udc_alloc_request,
	.free_request = s3c24xx_udc_free_request,

	.queue = s3c24xx_udc_queue,
	.dequeue = s3c24xx_udc_dequeue,

	.set_halt = s3c24xx_udc_set_halt,
	.fifo_status = s3c24xx_udc_fifo_status,
	.fifo_flush = s3c24xx_udc_fifo_flush,
};

/*
 * Function for writing from the request buffer into the EP-FIFO
 * The function updates the internal actual length of the USB-request for a possible
 * next transfer of the same request.
 * The return value is the number of remaining bytes in the request. If the return
 * value is equal zero, then there is no more data to process in the request
 * (Luis Galdos)
 */
static inline int s3c24xx_udc_write_packet(struct s3c_ep *ep, struct s3c_request *req)
{
	u16 *buf;
	int length, count;
	u32 fifo = ep->fifo;
	struct s3c24xx_udc *udc;
	int max, remaining, epnr;
	u8 *ptr;

	/* @XXX: Need some sanity checks (Luis Galdos) */
	udc = ep->dev;
	max = ep->ep.maxpacket;
	epnr = ep_index(ep);

	/* Get the number of remaining bytes */
	remaining = req->req.length - req->req.actual;
	if (!remaining) {
		pk_dbg("EP%i: Sending ZLP (actual: %i)\n",
			     epnr, req->req.actual);

		/* Send a frame with zero length */
		/* usb_set(udc, S3C24XX_UDC_ECR_TZLS, S3C24XX_UDC_ECR_REG, epnr); */
		usb_write(udc, 0, S3C24XX_UDC_BWCR_REG, epnr);

		length = remaining;
		goto exit_write_packet;
	}

	/* Use first a u8 pointer for obtaining the correct buffer address */
	ptr = req->req.buf + req->req.actual;
	buf = (u16 *)ptr;
	prefetch(buf);

	/* Only send the maximal allowed number of bytes */
	length = min(remaining, max);
	req->req.actual += length;

	/* First write the number of bytes to transfer, and then fill the FIFO */
	usb_write(udc, length, S3C24XX_UDC_BWCR_REG, epnr);
	for (count = 0; count < length; count += 2)
		writel(*buf++, udc->base + fifo);

	/* Return the number of remaining bytes of the passed request */
exit_write_packet:
	return (remaining - length);
}

/*
 * Test function which returns the number of bytes written into the FIFO.
 * This new function was implemented due an unknown issue registered with the
 * Ethernet-gadget (the UDC send packets with size of 514 bytes!)
 * (Luis Galdos)
 */
static inline int s3c24xx_udc_write_packet2(struct s3c_ep *ep, struct s3c_request *req)
{
	u16 *buf;
	int length, count;
	u32 fifo = ep->fifo;
	struct s3c24xx_udc *udc;
	int max, remaining, epnr;
	u8 *ptr;

	udc = ep->dev;
	max = ep->ep.maxpacket;
	epnr = ep_index(ep);

	/* Get the number of remaining bytes */
	remaining = req->req.length - req->req.actual;
	if (!remaining) {

		pk_dbg("EP%i: Sending ZLP (actual: %i)\n", epnr,
			     req->req.actual);

		/*
		 * Send a frame with zero length.
		 * DONT use the TZLS control bit of the EP control register ECR.
		 */
		usb_write(udc, 0, S3C24XX_UDC_BWCR_REG, epnr);
		length = 0;
		goto exit_write_packet;
	}

	/* Use first an u8 pointer for obtaining the correct buffer address */
	ptr = req->req.buf + req->req.actual;
	buf = (u16 *)ptr;
	prefetch(buf);

	/* Only send the maximal allowed number of bytes */
	length = min(remaining, max);
	req->req.actual += length;

	/* First write the number of bytes to transfer, and then fill the FIFO */
	usb_write(udc, length, S3C24XX_UDC_BWCR_REG, epnr);
	for (count = 0; count < length; count += 2)
		writel(*buf++, udc->base + fifo);

	/* Sanity check before writting into the FIFO */
#if defined(DEBUG_S3C2443_UDC_QUEUE)
	{
		ulong esr;

		esr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));
		printk(KERN_DEBUG
		       "%p: len=%i, act=%i, ep=%02x, bwcr=0x%04x, esr=0x%04lx\n",
		       req, req->req.length, req->req.actual, epnr, length, esr);
	}
#endif

	/* Return the number of remaining bytes of the passed request */
 exit_write_packet:
	return length;
}

/*
 * Check the current state of the VBUS pin. If no VBUS pin was passed through the
 * platform data, then assume the bus is always ON.
 * (Luis Galdos)
 */
static inline int s3c2443_udc_vbus_state(struct s3c24xx_udc *udc)
{
	int retval;
	struct s3c2410_udc_mach_info *info;

	info = udc->mach_info;
	retval = 1;
	if (info && info->vbus_pin) {
		unsigned long iocfg;

		/* @XXX: Do we really need to change to INPUT first? */
		iocfg = s3c_gpio_getcfg(info->vbus_pin);
		s3c_gpio_cfgpin(info->vbus_pin, S3C2410_GPIO_INPUT);
                retval = s3c2410_gpio_getpin(info->vbus_pin);
                s3c_gpio_cfgpin(info->vbus_pin, iocfg);

                if (info->vbus_pin_inverted)
                        retval = !retval;
	}

	return retval;
}

/*
 * Disable the controller by resetting the PHY for informing the USB-host
 * that the device was disconnected
 * (Luis Galdos)
 */
static void s3c24xx_udc_disable(struct s3c24xx_udc *udc)
{
	ulong regval;

	pk_dbg("UDC disable called\n");

	/* Disable the EP interrupts */
	writel(0, udc->base + S3C24XX_UDC_EIER_REG);
	writel(0xff, udc->base + S3C24XX_UDC_EIR_REG);

	/* Clear all the status bits of the EP0 and flush it */
	writel(S3C24XX_UDC_EP0SR_RSR | S3C24XX_UDC_EP0SR_TST |
	       S3C24XX_UDC_EP0SR_SHT | S3C24XX_UDC_EP0SR_LWO,
	       udc->base + S3C24XX_UDC_EP0SR_REG);
	writel(0, udc->base + S3C24XX_UDC_EP0CR_REG);

	/* Unset the function address */
	s3c24xx_udc_set_address(udc, 0);

	udc->ep0state = WAIT_FOR_SETUP;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->usb_address = 0;

	/* Clear all the status bits from the system status register */
	regval = S3C24XX_UDC_INT_RESET | S3C24XX_UDC_INT_SUSPEND |
		S3C24XX_UDC_INT_RESUME | S3C24XX_UDC_INT_SDE |
		S3C24XX_UDC_SSR_TBM | S3C24XX_UDC_INT_VBUSON |
		S3C24XX_UDC_SSR_VBUSOFF;
	writel(regval, udc->base + S3C24XX_UDC_SSR_REG);

	/* Reset the USB-function and the PHY */
	writel(S3C2443_URSTCON_PHY | S3C2443_URSTCON_FUNC, S3C2443_URSTCON);

	/* PHY power disable */
	regval = readl(S3C2443_PWRCFG);
	regval &= ~S3C2443_PWRCFG_USBPHY_ON;
	writel(regval, S3C2443_PWRCFG);
}

/*
 * Function for sending request data to the FIFO
 * This function uses the EP-lock for avoiding the wrong queue order of the packets
 * that are incoming from the Gadget-driver
 * (Luis Galdos)
 */
static void s3c24xx_udc_epin_tasklet_func(unsigned long data)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	int retval;
	ulong esr;
	struct s3c24xx_udc *udc;

	ep = (struct s3c_ep *)data;
	if (!ep) {
		pk_err("Invalid EP pointer. Aborting %s\n", __func__);
		return;
	}

	spin_lock(&ep->lock);

	udc = ep->dev;
	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));

	/*
	 * Paranoic sanity check: If the FIFO has still a packet, then abort this
	 * tasklet and wait for the call from the interrupt handler (TPS)
	 */
	if (S3C24XX_UDC_ESR_PSIFNR(esr) == 2) {
		pk_dbg("The FIFO seems to have still a packet\n");
		goto exit_unlock;
	}

	/* Check if there is a pending request for us */
	if (list_empty(&ep->queue))
		goto exit_unlock;

	/* Get the next request from the queue of the endpoint */
	req = list_entry(ep->queue.next, struct s3c_request, queue);
	if (!req) {
		pk_err("EP%i: NULL request pointer.\n", ep_index(ep));
		goto exit_unlock;
	}

#if defined(DEBUG_S3C2443_UDC_QUEUE)
	{
		u8 ch1, ch2;
		int len, act;
		u8 *ptr = (u8 *)req->req.buf;
		len = req->req.length;
		act = req->req.actual;
		ch1 = *ptr;
		ch2 = *(ptr + len - 1);
		printk(KERN_DEBUG "%p: act=%i, ep=%02x, 0x%02x ... 0x%02x\n",
		       req, act, ep_index(ep), ch1, ch2);
	}
#endif
	retval = s3c24xx_udc_write_fifo(ep, req);

 exit_unlock:
	spin_unlock(&ep->lock);
}

/*
 * Restart the UDC and the corresponding resources (tasklet, queues, etc.)
 * (Luis Galdos)
 */
static void s3c24xx_udc_reinit(struct s3c24xx_udc *udc)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	udc->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */
}

#define BYTES2MAXP(x)	(x / 8)
#define MAXP2BYTES(x)	(x * 8)

/*
 * Until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static int s3c24xx_udc_enable(struct s3c24xx_udc *udc)
{
	unsigned long regval;

	pk_dbg("UDC enable called\n");

	/* First disable the HOST functionality! */
	regval = __raw_readl(S3C2443_UCLKCON);
	regval &= ~S3C2443_UCLKCON_HOST_ENABLE;
	regval |= S3C2443_UCLKCON_THOST_DISABLE;
	__raw_writel(regval, S3C2443_UCLKCON);

	/* if reset by sleep wakeup, control the retention I/O cell */
	if (__raw_readl(S3C2443_RSTSTAT) & 0x8)
		__raw_writel(__raw_readl(S3C2443_RSTCON)|(1<<16), S3C2443_RSTCON);

	/* PHY power enable */
	regval = __raw_readl(S3C2443_PWRCFG);
	regval |= S3C2443_PWRCFG_USBPHY_ON;
	__raw_writel(regval, S3C2443_PWRCFG);

	/*
	 * USB device 2.0 must reset like bellow,
	 * 1st phy reset and after at least 10us, func_reset & host reset
	 * phy reset can reset bellow registers.
	 */
	/* PHY 2.0 S/W reset */
	regval = S3C2443_URSTCON_PHY;
	__raw_writel(regval, S3C2443_URSTCON);
	udelay(20);
	__raw_writel(0x00, S3C2443_URSTCON);

	/* Function reset, but DONT TOUCH THE HOST! */
	regval = S3C2443_URSTCON_FUNC;
	__raw_writel(regval, S3C2443_URSTCON);
	__raw_writel(0x00, S3C2443_URSTCON);

	/* 48Mhz, Oscillator, External X-tal, device */
	regval = S3C2443_PHYCTRL_EXTCLK_OSCI;
	__raw_writel(regval, S3C2443_PHYCTRL);

	/*
	 * D+ pull up disable(VBUS detect), USB2.0 Function clock Enable,
	 * USB1.1 HOST disable, USB2.0 PHY test enable
	 */
	regval = __raw_readl(S3C2443_UCLKCON);
	regval |= S3C2443_UCLKCON_FUNC_ENABLE;
	__raw_writel(regval, S3C2443_UCLKCON);

	reconfig_usbd(udc);

	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/*
	 * So, now enable the pull up, USB2.0 Function clock Enable and
	 * USB2.0 PHY test enable
	 */
	regval = __raw_readl(S3C2443_UCLKCON);
	regval |= S3C2443_UCLKCON_VBUS_PULLUP | S3C2443_UCLKCON_FUNC_ENABLE |
		S3C2443_UCLKCON_TFUNC_ENABLE | S3C2443_UCLKCON_THOST_DISABLE;
	__raw_writel(regval, S3C2443_UCLKCON);
	return 0;
}

/*
 * Function called from the Gadget-drivers for registering a new profile.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc *udc = the_controller;
	int retval;

	if (!driver)
		return -EINVAL;

	pk_dbg("Starting to register '%s'\n", driver->driver.name);

	if (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH) {
		pk_err("Only Full and High speed supported.\n");
		return -EINVAL;
	}

	/*
	 * The 'unbind' function is not required when the Gadget driver is compiled
	 * as built-in (Luis Galdos)
	 */
	if (!driver->bind || !driver->disconnect || !driver->setup) {
		pk_err("Missing function: Bind %p | Disconnect %p | Setup %p\n",
			   driver->bind, driver->disconnect, driver->setup);
		return -EINVAL;
	}

	if (!udc) {
		pk_err("No UDC-controller probed? Aborting.\n");
		return -ENODEV;
	}

	if (udc->driver) {
		pk_err("UDC already in use by '%s'\n", udc->driver->driver.name);
		return -EBUSY;
	}

	/* first hook up the driver ... */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;

	retval = device_add(&udc->gadget.dev);
	if (retval) {
		pk_err("Couldn't add the new Gadget device (%i)\n", retval);
		goto err_exit;
	}

	retval = driver->bind(&udc->gadget);
	if (retval) {
		pk_err("%s: bind to driver %s --> error %d\n", udc->gadget.name,
		       driver->driver.name, retval);
		goto err_del_device;
	}

	enable_irq(IRQ_USBD);

	/*
	 * If a host was already detected, then only call the UDC enable function,
	 * otherwise check over the configured GPIO if a host is connected.
	 */
	if (udc->vbus)
		s3c24xx_udc_enable(udc);
	else {
		struct s3c2410_udc_mach_info *info;
		unsigned long state, iocfg;

		info = udc->mach_info;

		iocfg = s3c_gpio_getcfg(info->vbus_pin);
		s3c2410_gpio_cfgpin(info->vbus_pin, S3C2410_GPIO_INPUT);
		state = s3c2410_gpio_getpin(info->vbus_pin);
		s3c2410_gpio_cfgpin(info->vbus_pin, iocfg);

		if (info->vbus_pin_inverted)
			state = !state;

		if (state)
			s3c24xx_udc_enable(udc);
	}

	pk_dbg("Gadget '%s' registered\n", driver->driver.name);
	return 0;

 err_del_device:
	device_del(&udc->gadget.dev);

 err_exit:
	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/*
 * Unregister entry point for the peripheral controller driver.
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc *udc = the_controller;
	unsigned long flags;

	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);
	udc->driver = NULL;
	stop_activity(udc, driver);
	spin_unlock_irqrestore(&udc->lock, flags);

	driver->unbind(&udc->gadget);

	device_del(&udc->gadget.dev);

	disable_irq(IRQ_USBD);

	pk_dbg("Unregistered gadget driver '%s'\n", driver->driver.name);

	/* Disable the pull-up for informing the host about the removed driver */
	s3c24xx_udc_disable(udc);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*
 * Write request to FIFO (max write == maxp size)
 * Return:  0 = still running, 1 = completed, negative = errno
 */
static int s3c24xx_udc_write_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	int max, count;
	int is_last, is_short;

	count = s3c24xx_udc_write_packet2(ep, req);
	max = le16_to_cpu(ep->desc->wMaxPacketSize);

	is_short = (count !=  max) ? (1) : (0);

	/* If the packet is short, we dont need to send an additional ZLP */
	if (is_short) {
		pk_dbg("EP%i: Short packet\n", ep_index(ep));
		is_last = 1;
	} else {
		if (req->req.length != req->req.actual || req->req.zero)
			is_last = 0;
		else {
			pk_dbg("EP%i: Clear ZLP\n", ep_index(ep));
			is_last = 1;
		}
	}

	pk_dbg("TX EP%i: C %i | L %i - A %i | %c %c %c\n",
		     ep_index(ep), count, req->req.length, req->req.actual,
		     is_last ? 'L':' ', is_short ? 'S':' ', req->req.zero ? 'Z':' ');

	/* If this was the last packet, then call the done callback */
	if (is_last) {

#if defined(DEBUG_S3C2443_UDC_QUEUE)
		int len, act;
		len = req->req.length;
		act = req->req.actual;
		printk(KERN_DEBUG "%p: len=%i, act=%i, ep=%02x [D]\n",
		       req, len, act, ep_index(ep));
#endif

		done(ep, req, 0);
	}

	return 0;
}

/*
 * Read to request from FIFO (max read == bytes in fifo)
 * Return: 0 = still running, 1 = completed, negative = errno
 */
static int s3c24xx_udc_read_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, count_bytes, is_short = 0, is_done = 0;
	u32 fifo = ep->fifo;
	struct s3c24xx_udc *udc;

	udc = ep->dev;
	csr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));

	/*
	 * If the FIFO is empty then return zero, so that a caller, like the queue-
	 * function, doesn't fail. Returning zero means that the request is not done
	 * and it can be added to the internal EP-request queue
	 * (Luis Galdos)
	 */
	if (!(csr & S3C24XX_UDC_ESR_RPS)) {
		pk_dbg("EP%i: No packet to read.\n", ep_index(ep));
		return 0;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);

	/* Calculate the current buffer space */
	bufferspace = req->req.length - req->req.actual;

	/* Read all bytes from this packet */
	count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_index(ep));
	if (csr & S3C24XX_UDC_ESR_LWO)
		count_bytes = count * 2 - 1;
	else
		count_bytes = count * 2;

	/* Update the actual variable of the request */
	req->req.actual += min(count_bytes, bufferspace);

	is_short = (count_bytes < ep->ep.maxpacket);
	is_done = (req->req.actual == req->req.length) ? 1 : 0;

	/*
	 * G : Got
	 * A : Actual
	 * T : Total to get
	 */
	if (is_short || is_done) {
		pk_dbg_out("EP%u: G %d | A %d | T %d [%c%c]\n",
			   ep_index(ep), count_bytes, req->req.actual, req->req.length,
			   is_short ? 'S' : ' ', is_done ? 'D' : ' ');
	}

	while (likely(count-- != 0)) {
		u16 byte = (u16)readl(udc->base + fifo);

		/*
		 * If there is no more space in the request-buffer, then continue
		 * reading from the FIFO and return with the done value
		 * (Luis Galdos)
		 */
		if (unlikely(bufferspace == 0)) {
			req->req.status = -EOVERFLOW;
			is_short = 1;
		} else {
			*buf++ = byte;
			bufferspace--;
		}
	}

	/*
	 * If the complete FIFO-data passed into the request-buffer, then
	 * return one, otherwise skip the return
	 * (Luis Galdos)
	 */
	if (is_short || is_done) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/*
 * Retire a request from the internal EP-queue and call the complete
 * function of the Gadget-request
 * (Luis Galdos)
 */
static void done(struct s3c_ep *ep, struct s3c_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	list_del_init(&req->queue);

	/*
	 * If the queue is empty and the EP has the OUT direction, then disable
	 * the receive operation, otherwise we will lost some packets from
	 * the host.
	 */
	if (!ep_is_in(ep) && list_empty(&ep->queue)) {
		ulong ecr;
		struct s3c24xx_udc *udc;

		udc = ep->dev;
		ecr = usb_read(udc, S3C24XX_UDC_ECR_REG, ep_index(ep));
		ecr |= S3C24XX_UDC_ECR_OUTPKTHLD;
		usb_write(udc, ecr, S3C24XX_UDC_ECR_REG, ep_index(ep));
	}

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN) {
		pk_dbg("EP%i: done req %p | stat %d | actual %u | length %u\n",
			     ep_index(ep),
			     &req->req, status, req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	/*
	 * We must unlock the queue of the EP at this place, then the Gadget-driver
	 * probably will try to enqueue a new request by calling our queue-function.
	 * (Luis Galdos)
	 */
/* 	spin_unlock(&ep->lock); */
/* 	spin_unlock(&ep->dev->lock); */
	req->req.complete(&ep->ep, &req->req);
/* 	spin_lock(&ep->dev->lock); */
/* 	spin_lock(&ep->lock); */

	ep->stopped = stopped;
}

/* Nuke/dequeue all the requested transfers */
void nuke(struct s3c_ep *ep, int status)
{
	struct s3c_request *req;

	pk_dbg("EP%i: Nuke function called\n", ep_index(ep));

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c_request, queue);
		done(ep, req, status);
	}
}

/*
 * This function handles the IN-operations of the endpoints different than zero
 */
static void s3c24xx_udc_in_epn(struct s3c24xx_udc *udc, u32 epnr)
{
	ulong esr, handled;
	struct s3c_ep *ep = &udc->ep[epnr];

	handled = 0;

	spin_lock(&ep->lock);

	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, epnr);

	/* ACK the function stall condition */
	if (esr & S3C24XX_UDC_ESR_FSC) {
		pk_dbg("EP%i: Function stall\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FSC, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* The flush operation generates an interrupt too */
	if (esr & S3C24XX_UDC_ESR_FFS) {
		pk_dbg("EP%i: FIFO flush detected\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FFS, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* Underflow check */
	if (esr & S3C24XX_UDC_ESR_FUDR) {
		pk_dbg("EP%i: Underflow detected\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FUDR, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* Overflow check */
	if (esr & S3C24XX_UDC_ESR_FOVF) {
		pk_dbg("EP%i: Overflow detected\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FOVF, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* By successed transfer of a IN-packet then only schedule the tasklet */
	if (esr & S3C24XX_UDC_ESR_TPS) {
		usb_set(udc, S3C24XX_UDC_ESR_TPS, S3C24XX_UDC_ESR_REG, epnr);
		tasklet_hi_schedule(&ep->in_tasklet);
		handled = 1;
	}

	spin_unlock(&ep->lock);

	if (!handled)
		pk_info("EP%i: Unhandled IRQ (ESR 0x%04lx)\n", epnr, esr);
}

/*
 * This function is used for reading OUT-frames from the EP0. We can't use the same
 * function for the SETUP-requests, then here we must pass the data to the
 * higher Gadget-driver.
 */
static void s3c2443_udc_ep0_read(struct s3c24xx_udc *udc)
{
	ulong ep0sr;
	int bytes, count, bufferspace;
	struct s3c_ep *ep;
	struct s3c_request *req;
	u16 *buf;

	ep = &udc->ep[0];

	spin_lock(&ep->lock);

	/*
	 * @FIXME: Remove this delay. At this moment we need it for having a
	 * working RNDIS-support when connected to a WinXP host machine.
	 * (Luis Galdos)
	 */
	if (udc->ep0state == DATA_STATE_RECV)
		udelay(100);

	/* If there is nothing to read only return at this point */
	ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
        if (!(ep0sr & S3C24XX_UDC_EP0SR_RSR))
		goto exit_unlock;

	/* Check if we are waiting for a setup frame */
	if (udc->ep0state == WAIT_FOR_SETUP) {
		s3c24xx_ep0_setup(udc);
		goto exit_unlock;
	}

	pk_dbg("Current state of EP0 is %i\n", udc->ep0state);

	/* Now get the number of bytes to read from the FIFO */
        count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_index(ep));
        if (ep0sr & S3C24XX_UDC_EP0SR_LWO)
                bytes = count * 2 - 1;
        else
                bytes = count * 2;

	/* Check if we have a request for this data */
	req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (!req) {
		pk_err("Going to flush a EP0 frame\n");
		goto exit_ack;
	}

        buf = req->req.buf + req->req.actual;
        prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;
	req->req.actual += min(bytes, bufferspace);

	pk_dbg("EP0 READ: %i bytes | space %i | req.len %i | reg.act %i\n",
		     bytes, bufferspace, req->req.length, req->req.actual);
	while (likely(count-- != 0)) {
                u16 byte = (u16)readl(udc->base + ep->fifo);
		*buf++ = byte;
	}

	/* If we are done with this request then call the corresponding function */
	if (req->req.length == req->req.actual) {
		udc->ep0state = WAIT_FOR_SETUP;
		done(ep, req, 0);
	}

 exit_ack:
	writel(S3C24XX_UDC_EP0_RX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG);

 exit_unlock:
	spin_unlock(&ep->lock);
}


/*
 * The below function is called when data was received with an OUT-transaction
 */
static void s3c24xx_udc_out_epn(struct s3c24xx_udc *udc, u32 ep_idx)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	ulong esr, epnr, handled;

	ep = &udc->ep[ep_idx];
	epnr = ep_index(ep);
	if (epnr != ep_idx) {
		pk_err("Invalid EP structure (%lu) or index (%u) passed\n",
			   epnr, ep_idx);
		return;
	}

	/* Read the status register of the EP */
	handled = 0;
	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, epnr);
	pk_dbg("EP%lu: Status reg 0x%08lx\n", epnr, esr);

	if (unlikely(!(ep->desc))) {
		pk_err("No descriptor for EP%lu\n", epnr);
		return;
	}

	if (esr & S3C24XX_UDC_ESR_FSC) {
		pk_dbg("EP%lu stall sent\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FSC, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	if (esr & S3C24XX_UDC_ESR_FFS) {
		pk_dbg("EP%lu FIFO flush\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FFS, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* RX means we have received some data over an OUT-transaction */
	if (esr & S3C24XX_UDC_ESR_RPS) {
		int retval;
		ulong packets;

		/*
		 * The read-fifo function returns zero if there is
		 * additional data in the FIFO. In that case read once
		 * again from the FIFO
		 */
		packets = (esr & S3C24XX_UDC_ESR_PSIF_TWO) ? 2 : 1;
		while (packets--) {

			req = (list_empty(&ep->queue)) ? NULL :
				list_entry(ep->queue.next, struct s3c_request, queue);

			/*
			 * If we dont have a request for the received data, then only
			 * break the loop and return without flushing the FIFO.
			 */
			if (unlikely(!req)) {
				ulong count_bytes, count;
				ulong csr;

				/* Read all bytes from this packet */
				csr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));
				count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_idx);
				if (csr & S3C24XX_UDC_ESR_LWO)
					count_bytes = count * 2 - 1;
				else
					count_bytes = count * 2;

				pk_dbg_out("EP%lu: No OUT req. queued (len %lu)\n",
					   epnr, count_bytes);
				break;
			}

			retval = s3c24xx_udc_read_fifo(ep, req);
			if (retval < 0) {
				pk_err("EP%lu: FIFO read (%i)\n", epnr, retval);
				break;
			}
		}

		handled = 1;
	}

	/* Handlings for the overflow and underrun */
	if (esr & S3C24XX_UDC_ESR_FOVF) {
		pk_err("EP%lu FIFO overflow\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FOVF, S3C24XX_UDC_ESR_REG, epnr);
	}

	if (esr & S3C24XX_UDC_ESR_FUDR) {
		pk_err("EP%lu FIFO underrun\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FUDR, S3C24XX_UDC_ESR_REG, epnr);
	}

	/*
	 * Check if the function was handled, otherwise only uses a debug message
	 * for informing about the error
	 */
	if (!handled) {
		pk_dbg("EP%lu: Unhandled OUT | ESR 0x%08lx.\n", epnr, esr);
	}
}

static void stop_activity(struct s3c24xx_udc *udc,
			  struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &udc->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&udc->lock);
		driver->disconnect(&udc->gadget);
		spin_lock(&udc->lock);
	}

	/* re-init driver-visible data structures */
	s3c24xx_udc_reinit(udc);
}

static void reconfig_usbd(struct s3c24xx_udc *udc)
{
	struct s3c_ep *ep;
	int cnt;
	unsigned long edr;

	/*
	 * Configure the endpoints depending on the defined structure which
	 * will be used by the gadget-drivers (see below)
	 */
	edr = 0;
	for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &the_controller->ep[cnt];
		if (ep->bEndpointAddress & USB_DIR_IN)
			edr |= (1 << cnt);
	}
	writel(edr, udc->base + S3C24XX_UDC_EDR_REG);

	/* Reset the endpoint configuration registers */
	for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &the_controller->ep[cnt];
		usb_write(udc, 0, S3C24XX_UDC_ECR_REG, ep_index(ep));
	}

	/* Only enable the EP0 */
	writel(0x1, udc->base + S3C24XX_UDC_EIER_REG);

 	writel(0x0, udc->base + S3C24XX_UDC_TR_REG);

	/* error interrupt enable, 16bit bus, Little format,
	 * suspend&reset enable
	 */
	writel(S3C24XX_UDC_EIE_EN |
	       S3C24XX_UDC_RRD_EN |
	       S3C24XX_UDC_SUS_EN |
	       S3C24XX_UDC_RST_EN,
	       udc->base + S3C24XX_UDC_SCR_REG);

	writel(0x0000, udc->base + S3C24XX_UDC_EP0CR_REG);

	writel(0, udc->base + S3C24XX_UDC_IR_REG);
}

static void s3c24xx_set_max_pktsize(struct s3c24xx_udc *udc, enum usb_device_speed speed)
{
	if (speed == USB_SPEED_HIGH) {
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		udc->gadget.speed = USB_SPEED_HIGH;
	} else {
		ep0_fifo_size = 64;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;
		udc->gadget.speed = USB_SPEED_FULL;
	}

	udc->ep[0].ep.maxpacket = ep0_fifo_size;
	udc->ep[1].ep.maxpacket = ep_fifo_size;
	udc->ep[2].ep.maxpacket = ep_fifo_size;
	udc->ep[3].ep.maxpacket = ep_fifo_size;
	udc->ep[4].ep.maxpacket = ep_fifo_size;
	udc->ep[5].ep.maxpacket = ep_fifo_size;
	udc->ep[6].ep.maxpacket = ep_fifo_size;
	udc->ep[7].ep.maxpacket = ep_fifo_size;
	udc->ep[8].ep.maxpacket = ep_fifo_size;

	usb_write(udc, ep0_fifo_size, S3C24XX_UDC_MAXP_REG, 0);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 1);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 2);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 3);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 4);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 5);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 6);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 7);
	usb_write(udc, ep_fifo_size, S3C24XX_UDC_MAXP_REG, 8);
}

static int s3c24xx_udc_set_pullup(struct s3c24xx_udc *udc, int is_on)
{
	struct s3c2410_udc_mach_info *info;
	enum s3c2410_udc_cmd_e cmd;

	info = udc->mach_info;

	if (is_on)
		cmd = S3C2410_UDC_P_ENABLE;
	else
		cmd = S3C2410_UDC_P_DISABLE;

	/* Call the platform dependet function if it's available */
	if (info && info->udc_command) {
		info->udc_command(cmd);
	} else {
                if (is_on) {
			/*
			 * Only enable the UDC if a Gadget-driver was already registered,
			 * otherwise by the registration the UDC-enable function should
			 * be called.
			 * (Luis Galdos)
			 */
			if (udc->driver)
				s3c24xx_udc_enable(udc);
		} else {
                        if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
                                if (udc->driver && udc->driver->disconnect)
                                        udc->driver->disconnect(&udc->gadget);

                        }
                        s3c24xx_udc_disable(udc);
                }
        }

        return 0;
}

static int s3c24xx_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
        struct s3c24xx_udc *udc = gadget_to_udc(gadget);

        udc->vbus = (is_active != 0);
        s3c24xx_udc_set_pullup(udc, is_active);
        return 0;
}

/*
 * This function is called by detection of the bus-power over the requested IRQ
 */
static irqreturn_t s3c24xx_udc_vbus_irq(int irq, void *_udc)
{
	struct s3c24xx_udc	*udc = _udc;
	unsigned int		value;
	struct s3c2410_udc_mach_info *info;
	ulong cfg;

	info = udc->mach_info;

	/* Some cpus cannot read from an line configured to IRQ! */
	cfg = s3c_gpio_getcfg(info->vbus_pin);
	s3c2410_gpio_cfgpin(info->vbus_pin, S3C2410_GPIO_INPUT);
	value = s3c2410_gpio_getpin(info->vbus_pin);
	s3c2410_gpio_cfgpin(info->vbus_pin, cfg);

	if (info->vbus_pin_inverted)
		value = !value;

	pk_dbg("Bus detect %s: vbus %i | value %i\n",
		     info->vbus_pin_inverted ? "inverted" : "", udc->vbus, value);

	if (value != udc->vbus)
		s3c24xx_udc_vbus_session(&udc->gadget, value);

	return IRQ_HANDLED;
}

/*
 * Interrupt handler of the USB-function. The interrupts to detect are coming
 * from the SMDK, but it doesn't consider the speed detection, which can lead
 * to some failures.
 *
 * (Luis Galdos)
 */
#define S3C2443_UDC_INT_CHECK			(0xff8f | S3C24XX_UDC_INT_HSP)
static irqreturn_t s3c24xx_udc_irq(int irq, void *_udc)
{
	struct s3c24xx_udc *udc = _udc;
	u32 intr_out, intr_in, intr_all;
	u32 sys_stat, sys_stat_chk;
	u32 stat, cnt;
	unsigned long flags;
	struct s3c_ep *ep;

	spin_lock_irqsave(&udc->lock, flags);

	sys_stat = readl(udc->base + S3C24XX_UDC_SSR_REG);
	stat = sys_stat;
	intr_all = readl(udc->base + S3C24XX_UDC_EIR_REG);

	/* We have only 3 usable eps now */
	sys_stat_chk = sys_stat & S3C2443_UDC_INT_CHECK;

	/* Only check for the correct endpoints (Luis Galdos) */
	for (cnt = 0, intr_in = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];

		/* Skip the OUT-endpoints different than zero */
		if (!(ep->bEndpointAddress & USB_DIR_IN) && cnt != 0)
			continue;

		if (s3c24xx_ep_enabled(udc, cnt))
			intr_in |= (1 << cnt);
	}
	intr_in &= intr_all;

	/* Check for the OUT-EPs that have generated an interrupt (Luis Galdos) */
	for (cnt = 0, intr_out = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];

		/* Skip the IN-endpoints */
		if (ep->bEndpointAddress & USB_DIR_IN)
			continue;

		if (s3c24xx_ep_enabled(udc, cnt))
			intr_out |= (1 << cnt);
	}
	intr_out &= intr_all;

	pk_dbg("UDC IRQ: stat 0x%08x (0x%08x) | in 0x%08x | out 0x%08x\n",
		     stat, sys_stat_chk, intr_in, intr_out);

	if (!intr_out && !intr_in && !sys_stat_chk)
		goto exit_ack;

	if (sys_stat) {
		if (sys_stat & S3C24XX_UDC_INT_VBUSON) {
			pk_dbg("Vbus ON interrupt\n");
			writel(S3C24XX_UDC_INT_VBUSON, udc->base + S3C24XX_UDC_SSR_REG);
			udc->vbus = 1;
		}

		if (sys_stat & S3C24XX_UDC_INT_ERR) {
			pk_dbg("ERROR interrupt\n");
			writel(S3C24XX_UDC_INT_ERR,
			       udc->base + S3C24XX_UDC_SSR_REG);
		}

		if (sys_stat & S3C24XX_UDC_INT_SDE) {

			writel(S3C24XX_UDC_INT_SDE,
			       udc->base + S3C24XX_UDC_SSR_REG);

			if (sys_stat & S3C24XX_UDC_INT_HSP) {
				pk_dbg("HIGH SPEED detection\n");
				s3c24xx_set_max_pktsize(udc, USB_SPEED_HIGH);
			} else {
				pk_dbg("FULL SPEED detection\n");
				s3c24xx_set_max_pktsize(udc, USB_SPEED_FULL);
			}
		}

		if (sys_stat & S3C24XX_UDC_INT_HSP) {

			writel(S3C24XX_UDC_INT_HSP, udc->base + S3C24XX_UDC_SSR_REG);
			pk_dbg("High Speed interrupt\n");
		}

		/*
		 * @HW-BUG: If we get a suspend interrupt BUT are still connected to
		 * the bus, then the host-port number 2 registers a status change
		 * and tries to enable the port. This leads to a failure (see [1])
		 * since we are using the USB-PHY as device, and NOT as host-port.
		 *
		 * [1] hub 1-0:1.0: Cannot enable port 2.  Maybe the USB cable is bad?
		 *
		 * The only option to avoid this error is disabling the USB-PHY so that
		 * a RESUME condition is generated and the host will ONLY try to
		 * enumerate an apparently connected USB-device
		 */
		if (sys_stat & S3C24XX_UDC_INT_SUSPEND) {

			pk_dbg("SUSPEND interrupt\n");

			/* First ACK the interrupt after the bug fix! */
			writel(S3C24XX_UDC_INT_SUSPEND,
			       udc->base + S3C24XX_UDC_SSR_REG);
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
			    && udc->driver
			    && udc->driver->suspend) {
				udc->driver->suspend(&udc->gadget);
			}
		}

		/*
		 * By the resume interrupts the following error message is printed:
		 * hub 1-0:1.0: unable to enumerate USB device on port 2
		 */
		if (sys_stat & S3C24XX_UDC_INT_RESUME) {
			pk_dbg("RESUME interrupt\n");
			writel(S3C24XX_UDC_INT_RESUME,
			       udc->base + S3C24XX_UDC_SSR_REG);
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
			    && udc->driver
			    && udc->driver->resume) {
				udc->driver->resume(&udc->gadget);
			}
		}

		if (sys_stat & S3C24XX_UDC_INT_RESET) {
			pk_dbg("RESET interrupt\n");
			writel(S3C24XX_UDC_INT_RESET,
			       udc->base + S3C24XX_UDC_SSR_REG);
			reconfig_usbd(udc);
			udc->ep0state = WAIT_FOR_SETUP;
		}

		if (sys_stat & (S3C24XX_UDC_SSR_TBM | S3C24XX_UDC_SSR_EOERR |
			    S3C24XX_UDC_SSR_DCERR))
			pk_err("Unexpected sys failure: 0x%08x\n", sys_stat);

	}

	if (intr_in) {
		unsigned long cnt, epm;

		/* FIXME: Workaround for preventing FIFO under/overrun */
		udelay(300);

		if (intr_in & S3C24XX_UDC_INT_EP0) {
			ulong ep0sr;

			/* First handle the arrived data, and then clear the IRQ */
			s3c24xx_handle_ep0(udc);
			writel(S3C24XX_UDC_INT_EP0, udc->base + S3C24XX_UDC_EIR_REG);

			/*
			 * By long setup-handlings it's possible to have a TST at
			 * this point.
			 */
			ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
			if (ep0sr & S3C24XX_UDC_EP0SR_TST)
				writel(S3C24XX_UDC_EP0SR_TST,
				       udc->base + S3C24XX_UDC_EP0SR_REG);
		}


		/* First get the EP-number that generated the interrupt */
		for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
			epm = (1 << cnt);
			if (intr_in & epm) {
				writel(epm, udc->base + S3C24XX_UDC_EIR_REG);
				s3c24xx_udc_in_epn(udc, cnt);
				/* writel(epm, udc->base + S3C24XX_UDC_EIR_REG); */
			}
		}
	}

	/* Check for OUT-frames by the endpoints */
	if (intr_out) {
		unsigned long cnt, epm;

		/* FIXME: Workaround for preventing FIFO under/overrun */
		udelay(100);

		/* And the EP0 can receive OUT-transfers too! */
		if (intr_out & 0x1)
			s3c2443_udc_ep0_read(udc);

		for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
			epm = (1 << cnt);
			if (intr_out & epm) {
				writel(epm, udc->base + S3C24XX_UDC_EIR_REG);
				s3c24xx_udc_out_epn(udc, cnt);
			}
		}
	}

 exit_ack:
	/* writel(stat, udc->base + S3C24XX_UDC_SSR_REG); */
	spin_unlock_irqrestore(&udc->lock, flags);
	return IRQ_HANDLED;
}

/*
 * Enable one EP, but only if it's different than the EP zero
 * This function is called from the gadget-drivers over the UDC-operation functions
 */
static int s3c24xx_udc_ep_enable(struct usb_ep *_ep,
				 const struct usb_endpoint_descriptor *desc)
{
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	unsigned long flags, epnr, regval;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !desc || ep->desc) {
		pk_err("NULL pointer or bad EP or descriptor found.\n");
		return -EINVAL;
	} else if (_ep->name == ep0name) {
		pk_err("Invalid EP name (%s) to enable.\n", _ep->name);
		return -EINVAL;
	} else if (desc->bDescriptorType != USB_DT_ENDPOINT) {
		pk_err("Invalid descriptor type (USB_DT_ENDPOINT)\n");
		return -EINVAL;
	} else if (ep->bEndpointAddress != desc->bEndpointAddress) {
		pk_err("Invalid EP address found (valid %x | invalid %x)\n",
			   ep->bEndpointAddress, desc->bEndpointAddress);
		return -EINVAL;
	} else if (ep_maxpacket(ep) < le16_to_cpu(desc->wMaxPacketSize)) {
		pk_err("Invalid EP size %u (max. %u)\n",
			   le16_to_cpu(desc->wMaxPacketSize), ep_maxpacket(ep));
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		pk_err("Type mismatch by EP %s\n", _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize) > ep_maxpacket(ep))
	    || !desc->wMaxPacketSize) {
		pk_err("Bad %s maxpacket (desc %u | max %u)\n", _ep->name,
			   le16_to_cpu(desc->wMaxPacketSize), ep_maxpacket(ep));
		return -ERANGE;
	}

	udc = ep->dev;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		pk_err("Bogus device state\n");
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* Enable the interrupt for this EP */
	epnr = ep_index(ep);
	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	regval |= (1 << epnr);
	writel(regval, udc->base + S3C24XX_UDC_EIER_REG);

	/* Enable the dual FIFO mode */
	regval = usb_read(udc, S3C24XX_UDC_ECR_REG, epnr);
	regval |= S3C24XX_UDC_ECR_DUEN;
        usb_write(udc, regval, S3C24XX_UDC_ECR_REG, epnr);

	/* Reset halt state */
	s3c24xx_udc_set_halt(_ep, 0);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	pk_dbg("Enabled %s | Addr. 0x%02x\n", _ep->name, ep->bEndpointAddress);
	return 0;
}

static int s3c24xx_udc_ep_disable(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	unsigned long flags;
	int epnr;
	struct s3c24xx_udc *udc;
	ulong regval;

	if (!_ep) {
		pk_err("Null pointer passed! Aborting.\n");
		return -EINVAL;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!ep->desc) {
		pk_dbg("%s has an empty descriptor\n", ep->ep.name);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Disable the corresponding IRQ */
	udc = ep->dev;
	epnr = ep_index(ep);
	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	regval &= ~(1 << epnr);
	writel(regval, udc->base + S3C24XX_UDC_EIER_REG);

	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

static struct usb_request *s3c24xx_udc_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct s3c_request *req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void s3c24xx_udc_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c_request *req;

	req = container_of(_req, struct s3c_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*
 * This function is called by the Gadget-drivers when they have a request for
 * the UDC (for us).
 */
static int s3c24xx_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
			     gfp_t gfp_flags)
{
	struct s3c_request *req;
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&udc->lock, flags);

	req = container_of(_req, struct s3c_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf ||
		     !list_empty(&req->queue))) {
		pk_err("Bad params for a new EP queue\n");
		retval = -EINVAL;
		goto exit_queue_unlock;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		pk_err("Bad EP or invalid descriptor\n");
		retval = -EINVAL;
		goto exit_queue_unlock;
	}

	udc = ep->dev;
	if (unlikely(!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		pk_err("Bogus device state %p\n", udc->driver);
		retval = -ESHUTDOWN;
		goto exit_queue_unlock;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* Is this possible? */
	if (!_req->length) {
		pk_dbg("EP%i: Empty request | Zero %i\n",
			     ep_index(ep), req->req.zero);
		done(ep, req, 0);
		retval = 0;
		goto exit_queue_unlock;
	}

	/*
	 * By the IN-endpoints only add the new request to the
	 * internal EP-queue and schedule the tasklet. The tasklet
	 * function will check if the request data can be sent or
	 * not.
	 * (Luis Galdos)
	 */
	if (ep_is_in(ep) && ep_index(ep) != 0) {
		/* unsigned long flags; */

		/* spin_lock_irqsave(&ep->lock, flags); */
#if defined(DEBUG_S3C2443_UDC_QUEUE)
		{
			u8 ch1, ch2;
			int len;
			u8 *ptr;

			ptr = (u8 *)_req->buf;
			len = _req->length;
			ch1 = *ptr;
			ch2 = *(ptr + len - 1);
			printk(KERN_DEBUG "%p: len=%i, ep=%02x, 0x%02x ... 0x%02x [N]\n",
			       req, len, ep_index(ep), ch1, ch2);
		}
#endif
		list_add_tail(&req->queue, &ep->queue);
		/* spin_unlock_irqrestore(&ep->lock, flags); */
		tasklet_hi_schedule(&ep->in_tasklet);
	} else {
		int handled;

		/*
		 * If the request couldn't be handled, then tail it into the
		 * queue of the endpoint
		 */
		handled = 0;
		if (list_empty(&ep->queue) && likely(!ep->stopped)) {

			if (unlikely(ep_index(ep) == 0)) {
				list_add_tail(&req->queue, &ep->queue);
				s3c24xx_udc_ep0_kick(udc, ep);
				handled = 1;

			} else {
				/*
				 * The read-function returns zero if the request is not
				 * done (there is free available buffer space). In this
				 * case we must add the request to the internal queue.
				 * (Luis Galdos)
				 */
				retval = s3c24xx_udc_read_fifo(ep, req);
				handled = (retval == 1) ?  (1) : (0);

				/* Error handling */
				if (retval < 0) {
					pk_err("EP%i: Read FIFO error\n",
						   ep_index(ep));
					goto exit_queue_unlock;
				}
			}
		}

		/* Advances the queue with the non handled request */
		if (!handled) {

			if (!ep_is_in(ep)) {
				/* Free the hold configuration bit for receiving further packets */
				ulong ecr;

				ecr = usb_read(udc, S3C24XX_UDC_ECR_REG, ep_index(ep));
				ecr &= ~S3C24XX_UDC_ECR_OUTPKTHLD;
				usb_write(udc, ecr, S3C24XX_UDC_ECR_REG, ep_index(ep));

				pk_dbg_out("EP%i: Queued len %u\n", ep_index(ep), _req->length);
			}

			list_add_tail(&req->queue, &ep->queue);
		}
	}

	retval = 0;

 exit_queue_unlock:
	spin_unlock_irqrestore(&udc->lock, flags);
	return retval;
}

/* Dequeue one USB-request */
static int s3c24xx_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	unsigned long flags;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	pk_dbg("EP%i: Dequeue called\n", ep_index(ep));

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/*
 * Halt specific EP. If the value is equal one, then halt the EP, by zero
 * enable the EP once again
 */
static int s3c24xx_udc_set_halt(struct usb_ep *_ep, int halt)
{
	struct s3c_ep *ep;
	unsigned long flags;
	int retval, epnr;
	struct s3c24xx_udc *udc;

	ep = container_of(_ep, struct s3c_ep, ep);
	udc = ep->dev;
	epnr = ep_index(ep);

	pk_dbg("%s the EP%i\n", halt ? "Halting" : "Enabling", epnr);

	if (!ep->desc) {
                pk_err("Attempted to halt uninitialized ep %s\n", ep->ep.name);
                return -ENODEV;
        }

	spin_lock_irqsave(&udc->lock, flags);

        /*
	 * Don't halt the EP if it's an IN and not empty
	 */
	retval = 0;
        if (ep_is_in(ep) && !list_empty(&ep->queue)) {
                retval = -EAGAIN;
        } else {
                if (halt)
			usb_set(udc, S3C24XX_UDC_ECR_ESS, S3C24XX_UDC_ECR_REG, epnr);
                else
			usb_clear(udc, S3C24XX_UDC_ECR_ESS, S3C24XX_UDC_ECR_REG, epnr);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return retval;
}

/*
 * Return the available data bytes of the EP-FIFO
 */
static int s3c24xx_udc_fifo_status(struct usb_ep *_ep)
{
	u32 csr;
	int count = 0;
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep) {
		pk_dbg("%s: bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	csr = usb_read(ep->dev, S3C24XX_UDC_EP_STATUS_REG, ep_index(ep));
	if (ep->dev->gadget.speed != USB_SPEED_UNKNOWN ||
	    csr & S3C24XX_UDC_EP_RX_SUCCESS) {

		count = usb_read(ep->dev, S3C24XX_UDC_BYTE_READ_CNT_REG, ep_index(ep));

		if (usb_read(ep->dev, S3C24XX_UDC_EP_STATUS_REG, ep_index(ep))
		    & S3C24XX_UDC_EP_LWO)
			count = count * 2 -1;
		else
			count = count * 2;
	}

	return count;
}

/*
 * Flush the FIFO of the endpoint
 */
static void s3c24xx_udc_fifo_flush(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	int epnr;

	if (!_ep) {
		pk_err("Can't flush an EP, NULL pointer passed\n");
		return;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	epnr = ep_index(ep);
	udc = ep->dev;

	if (unlikely(epnr == 0)) {
		pk_err("EP0 can't be flushed. Aborting.\n");
		return;
	}

	/*
	 * Flush the EP by using the control register
	 * IMPORTANT: Dont enable the below code, otherwise is not possible to
	 * recover the EP from it, we need to restart the UDC for this purpose (we dont
	 * really want to do it!).
	 */
#if 0
	{
		ulong ecr;

		ecr = usb_read(udc, S3C24XX_UDC_ECR_REG, epnr);
		pk_err("EP%i: Flushing now [ecr 0x%08lx]\n", epnr, ecr);
		usb_write(udc, ecr | S3C24XX_UDC_ECR_FLUSH, S3C24XX_UDC_ECR_REG, epnr);
	}
#endif
}

/*
 * Function used for reading the data from the FIFO to the passed buffer
 * This function is only used for the setup-handling
 */
static inline int s3c24xx_udc_ep0_setup_read(struct s3c_ep *ep, u16 *cp, int max)
{
        int bytes;
        int count, pending;
        struct s3c24xx_udc *udc;
        ulong ep0sr;

        udc = ep->dev;

        ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
        if (!(ep0sr & S3C24XX_UDC_EP0SR_RSR)) {
                pk_dbg("RSR-bit unset. Aborting setup read.\n");
                return 0;
        }

        /* Now get the number of bytes to read from the FIFO */
        count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_index(ep));
        if (ep0sr & S3C24XX_UDC_EP0SR_LWO)
                bytes = count * 2 - 1;
        else
                bytes = count * 2;

	/*
	 * If we not enough space, then only process maximal number of bytes
	 */
	pending = 0;
        if (bytes > max) {
                pk_info("Setup packet length %i exceeds max. %i\n", bytes, max);
                count = max / 2;
                bytes = max;
		pending = 1;
        }

        while (count--)
                *cp++ = (u16)readl(udc->base + S3C24XX_UDC_EP0BR_REG);

        /*
         * IMPORTANT: Dont delete the below line, then otherwise the controller will
         * not work (but why not?)
         * (Luis Galdos)
         */
	if (!pending)
		writel(S3C24XX_UDC_EP0_RX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG);

	return bytes;
}

/*
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function
 * after it decodes a set address setup packet.
 */
static void s3c24xx_udc_set_address(struct s3c24xx_udc *udc, unsigned char address)
{
	udc->usb_address = address;
}

/* Write data into the FIFO of the EP0 */
static void s3c24xx_udc_ep0_write(struct s3c24xx_udc *udc)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &udc->ep[0];
	int ret, completed, need_zlp = 0;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (!req) {
		pk_dbg("EP0: NULL write request?\n");
		return;
	}

	if (req->req.length == 0) {
		udc->ep0state = WAIT_FOR_SETUP;
	   	done(ep, req, 0);
		return;
	}

	if (req->req.length - req->req.actual == ep0_fifo_size) {
		/* Next write will end with the packet size, */
		/* so we need Zero-length-packet */
		need_zlp = 1;
	}

	/* The write function returns the number of remaining bytes of this request */
	ret = s3c24xx_udc_write_packet(ep, req);
	completed = (ret == 0) ? 1 : 0;

	if (completed && !need_zlp) {
		pk_dbg("EP0: Finished, waiting for status\n");
		udc->ep0state = WAIT_FOR_SETUP;
		done(ep, req, 0);
	} else if (need_zlp) {
		/* The next TX-interrupt will send the ZLP */
		udc->ep0state = DATA_STATE_NEED_ZLP;
	} else
		/* We need to send more data to the host in the next transfer */
		pk_dbg("EP0: not finished | %p\n", req);
}

/* Return zero if NO additional request is available */
static inline int s3c2443_ep0_fix_set_setup(struct s3c24xx_udc *udc,
					    struct usb_ctrlrequest *ctrl)
{
	int timeout_us, cnt;
	ulong ep0sr;
	int retval;
	struct s3c_ep *ep;

	ep = &udc->ep[0];
	timeout_us = 1000;
	do {
		udelay(1);
		ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
		timeout_us--;
	} while (timeout_us && !(ep0sr & S3C24XX_UDC_EP0SR_RSR));

	/* If a timeout happens then returns zero */
	retval = 0;
	if (timeout_us) {
		cnt = s3c24xx_udc_ep0_setup_read(ep, (u16 *)ctrl,
						 sizeof(struct usb_ctrlrequest));
		if (cnt > 0)
			retval = 1;
	}

	return retval;
}

/*
 * Wait for a setup packet and read if from the FIFO before passint it to the
 * gadget driver
 */
#define S3C2443_SETUP_IS_SET_REQ(ctrl) \
			(ctrl->bRequest == USB_REQ_SET_CONFIGURATION || \
			ctrl->bRequest == USB_REQ_SET_INTERFACE || \
			ctrl->bRequest == USB_REQ_SET_DESCRIPTOR || \
			ctrl->bRequest == USB_REQ_SET_FEATURE || \
			ctrl->bRequest == USB_REQ_SET_ADDRESS)

#define S3C2443_MAX_NUMBER_SETUPS			(10)
static void s3c24xx_ep0_setup(struct s3c24xx_udc *udc)
{
	struct s3c_ep *ep;
	int retval, bytes, cnt;
	struct usb_ctrlrequest *pctrl;
	struct usb_ctrlrequest ctrls[S3C2443_MAX_NUMBER_SETUPS];
	int recv_ctrls[S3C2443_MAX_NUMBER_SETUPS];

	memset(recv_ctrls, 0x0, sizeof(recv_ctrls));
	memset(ctrls, 0x0, sizeof(ctrls));

	/* Nuke all previous transfers of this control EP */
	ep = &udc->ep[0];
	nuke(ep, -EPROTO);

	/*
	 * @FIXME: This is a really bad code, but at this moment there is no better
	 * proposal for a workaround, since I dont know why the HELL the controller
	 * stops working by some SETUP-frames which require a status stage.
	 * Probably the problem is that the controller sends automatically the
	 * IN-Data of the status stage (with length zero) and it doesn't NAK the
	 * proceeded IN-request. Since, we have still data from the first IN-frame in
	 * the FIFO and the second arrived very fast, the FIFO gets stuck.
	 * Unfortunately we can't reset the FIFO (the control bit for flushing the
	 * FIFO seems to be only for the EPs different than zero).
	 *
	 * For avoiding the described problem probably a delayed-work function can help,
	 * otherwise the below code can't be removed.
	 * (Luis Galdos)
	 */

	/* Read the first SETUP frame as a normal frame */
	pctrl = &ctrls[0];
        recv_ctrls[0] = s3c24xx_udc_ep0_setup_read(ep, (u16 *)pctrl, sizeof(* pctrl));
	if (recv_ctrls[0] <= 0) {
		pk_dbg("Nothing to process? Aborting.\n");
		return;
	}

	/* Here we need to read the other packets */
	for (cnt = 1; S3C2443_SETUP_IS_SET_REQ(pctrl) && cnt < S3C2443_MAX_NUMBER_SETUPS;
		     cnt++) {
		pctrl = &ctrls[cnt];
		bytes = s3c2443_ep0_fix_set_setup(udc, pctrl);

		if (bytes <= 0)
			break;

		recv_ctrls[cnt] = bytes;
	}

	if (cnt == S3C2443_MAX_NUMBER_SETUPS)
		pk_err("@FIXME: Handling by SETUP overflows\n");

	/* Now start with the processing of the SETUP-frames */
	retval = 0;
	for (cnt = 0; recv_ctrls[cnt] > 0 && cnt < S3C2443_MAX_NUMBER_SETUPS; cnt++) {

		pctrl = &ctrls[cnt];
		bytes = recv_ctrls[cnt];

		/* Set the correct direction for this SETUP-frame */
		if (likely(pctrl->bRequestType & USB_DIR_IN)) {
			pk_dbg("EP0: Preparing new IN frame (%i bytes)\n", bytes);
			ep->bEndpointAddress |= USB_DIR_IN;
		} else {
			pk_dbg("EP0: Preparing new OUT frame (%i bytes)\n", bytes);
			ep->bEndpointAddress &= ~USB_DIR_IN;
		}

		/* Handle some SETUP packets ourselves */
		retval = 0;
		switch (pctrl->bRequest) {
		case USB_REQ_SET_ADDRESS:
			if (pctrl->bRequestType != (USB_TYPE_STANDARD |
						    USB_RECIP_DEVICE))
				break;

			/*
			 * If the setup frame was for us, then continue with the
			 * next available packet
			 */
			pk_dbg("Set address request (%d)\n", pctrl->wValue);
			s3c24xx_udc_set_address(udc, pctrl->wValue);
			continue;

		default:
			pk_dbg("bRequestType 0x%02x | bRequest 0x%02x | "
				     "wValue 0x%04x | wIndex 0x%04x | wLength %u\n",
				     pctrl->bRequestType, pctrl->bRequest,
				     pctrl->wValue, pctrl->wIndex, pctrl->wLength);
			break;
		}

		/* Check if need to call the Gadget-setup handler */
		if (likely(udc->driver)) {
			spin_unlock(&udc->lock);
			retval = udc->driver->setup(&udc->gadget, pctrl);
			spin_lock(&udc->lock);

			/* Error values are littler than zero */
			if (retval < 0)
				s3c2443_print_err_packet_setup(retval, pctrl);
		}
	}

	/* By error-free setups return at this place */
	if (!retval)
		return;

	/* @XXX: Test if the STALL is really send to the host */
	udc->ep0state = WAIT_FOR_SETUP;
	writel(S3C24XX_UDC_EP0CR_ESS, udc->base + S3C24XX_UDC_EP0CR_REG);
}

/*
 * handle ep0 interrupt
 */
static void s3c24xx_handle_ep0(struct s3c24xx_udc *udc)
{
	struct s3c_ep *ep = &udc->ep[0];
	u32 csr;
	unsigned long handled;

	handled = 0;
 	csr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);

	/* Clear the STALL bit */
	if (csr & S3C24XX_UDC_EP0_STALL) {
		pk_dbg("EP0: Stall success\n");
		writel(S3C24XX_UDC_EP0_STALL, udc->base + S3C24XX_UDC_EP0SR_REG);
		nuke(ep, -ECONNABORTED);
		udc->ep0state = WAIT_FOR_SETUP;
		handled = 1;
	}

	/*
	 * We must check if there is additional data to send. We send at this
	 * place the ZLP too (DONT try to do it inside the EP0-write function!)
	 */
	if (csr & S3C24XX_UDC_EP0_TX_SUCCESS) {
		struct s3c_ep *ep0 = &udc->ep[0];
		struct s3c_request *req;
		int left;

		req = list_entry(ep0->queue.next, struct s3c_request, queue);
		if (req) {
			left = req->req.length - req->req.actual;
			pk_dbg("EP0: TX success | %p: left %i\n", req, left);

			/* Send the pending ZLP of the last request */
			if (left || udc->ep0state == DATA_STATE_NEED_ZLP)
				s3c24xx_udc_ep0_write(udc);
		}

		/*
		 * Clear the status bit ONLY if we are not going to send a ZLP in the
		 * next IN-transfer
		 */
		if (udc->ep0state != DATA_STATE_NEED_ZLP)
			writel(S3C24XX_UDC_EP0_TX_SUCCESS,
			       udc->base + S3C24XX_UDC_EP0SR_REG);

		handled = 1;
	}

	/* Check if we have received data from the host (SETUP-frames) */
	if (csr & S3C24XX_UDC_EP0_RX_SUCCESS) {
		if (udc->ep0state == WAIT_FOR_SETUP) {
			pk_dbg("EP0: RX success | Wait for setup\n");
			s3c24xx_ep0_setup(udc);
		} else if (udc->ep0state == DATA_STATE_RECV) {
			s3c2443_udc_ep0_read(udc);
		} else {
			pk_err("EP0: RX success | Strange state %i\n",
				   udc->ep0state);
			udc->ep0state = WAIT_FOR_SETUP;
			writel(S3C24XX_UDC_EP0SR_RSR, udc->base + S3C24XX_UDC_EP0SR_REG);
		}

		handled = 1;
	}

	/*
	 * Under unknown conditions the EP0 is generating some interrupts which we can't
	 * identify. Since these IRQs seem to have none side effects, we only print
	 * a debug message and return inmediately. Please note that the IRQs are
	 * being generated only by the enumeration of the device, just when the EP0 is
	 * in use.
	 * @XXX: Add some additional register outprints (EP0SR, EP0CR, SSR, etc.)
	 * (Luis Galdos)
	 */
	if (!handled) {
		pk_dbg("[ ERROR ] s3c24xx-udc: Unhandled EP0 IRQ.\n");
	}
}

/*
 * This function is called for the gadget-drivers which uses the EP0 for transferring
 * data to/from the host. This is the case of the RNDIS driver.
 * (Luis Galdos)
 */
static void s3c24xx_udc_ep0_kick(struct s3c24xx_udc *udc, struct s3c_ep *ep)
{
	if (ep_is_in(ep)) {
		udc->ep0state = DATA_STATE_XMIT;
		s3c24xx_udc_ep0_write(udc);
	} else {
		/* Always set the state of the endpoint first */
		udc->ep0state = DATA_STATE_RECV;
		s3c2443_udc_ep0_read(udc);
	}
}

static int s3c_udc_get_frame(struct usb_gadget *_gadget)
{
	u32 frame;
	struct s3c24xx_udc *udc;

	udc = the_controller;
	if (!udc)
		return -ENODEV;

	frame = readl(udc->base + S3C24XX_UDC_FNR_REG);
	return (frame & 0x7ff);
}

static int s3c_udc_wakeup(struct usb_gadget *_gadget)
{
	return -ENOTSUPP;
}

static const struct usb_gadget_ops s3c_udc_ops = {
	.get_frame = s3c_udc_get_frame,
	.wakeup = s3c_udc_wakeup,
	/* current versions must always be self-powered */
};

static void nop_release(struct device *dev)
{
	pk_dbg("%s %s\n", __FUNCTION__, dev->init_name);
}

/*
 * At this moment provide only non-configurable endpoints (address, direction and
 * type fixed). The syntax definition is coming from the file[1].
 *
 * [1] drivers/usb/gadget/autoconf.c:ep_matches()
 */
static struct s3c24xx_udc memory = {
	.usb_address = 0,

	.gadget = {
		.ops = &s3c_udc_ops,
		.ep0 = &memory.ep[0].ep,
		.name = DRIVER_NAME,
		.dev = {
			.init_name = "gadget",
			.release = nop_release,
		},
	},

	.ep[0] = {
		.ep = {
			.name = ep0name,
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP0_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 0,
		.bmAttributes = 0,
		.ep_type = ep_control,
		.fifo = S3C24XX_UDC_EP0BR_REG,
	},

	.ep[1] = {
		.ep = {
			.name = "ep1in-bulk",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 1,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.ep_type = ep_bulk_in,
		.fifo = S3C24XX_UDC_EP1BR_REG,
	},

	.ep[2] = {
		.ep = {
			.name = "ep2out-bulk",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 2,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.ep_type = ep_bulk_out,
		.fifo = S3C24XX_UDC_EP2BR_REG,
	},

	.ep[3] = {
		.ep = {
			.name = "ep3in-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 3,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP3BR_REG,
	},

	.ep[4] = {
		.ep = {
			.name = "ep4out-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 4,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP4BR_REG,
	},
	.ep[5] = {
		.ep = {
			.name = "ep5in-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 5,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP5BR_REG,
	},
	.ep[6] = {
		.ep = {
			.name = "ep6out-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 6,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP6BR_REG,
	},
	.ep[7] = {
		.ep = {
			.name = "ep7in-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 7,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP7BR_REG,
	},
	.ep[8] = {
		.ep = {
			.name = "ep8out-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 8,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP8BR_REG,
	},
};


/*
 * probe - binds to the platform device
 */
static int s3c24xx_udc_probe(struct platform_device *pdev)
{
	struct s3c24xx_udc *udc = &memory;
	int retval;
	struct s3c2410_udc_mach_info *mach_info;
	int cnt;

	pk_dbg("Probing a new device ID %i\n", pdev->id);

	/* Get the platform data */
	mach_info = pdev->dev.platform_data;
	if (!mach_info) {
		pk_err("No platform data? Aborting.\n");
		retval = -EINVAL;
		goto err_exit;
	}

	udc->mach_info = mach_info;
	udc->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!udc->mem) {
		pk_err("Couldn't get the IO memory resource\n");
		retval = -EINVAL;
		goto err_exit;
	}

        udc->mem = request_mem_region(udc->mem->start, IOMEMSIZE(udc->mem), pdev->name);
        if (!udc->mem) {
                pk_err("Failed to request IO memory region.\n");
                retval = -ENOENT;
                goto err_exit;
        }

	udc->base = ioremap(udc->mem->start, IOMEMSIZE(udc->mem));
	if (!udc->base) {
		pk_err("Couldn't ioremap the IO memory region\n");
		retval = -EINVAL;
		goto err_free_mem;
	}

	/* Init the internal gadget device */
	device_initialize(&udc->gadget.dev);
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

	/* @XXX: We can use only one device, right? */
	the_controller = udc;
	platform_set_drvdata(pdev, udc);

	/* Init the EPs */
	s3c24xx_udc_reinit(udc);

	/* Disable the platform dependent UDC-hardware */
	s3c24xx_udc_disable(udc);

	spin_lock_init(&udc->lock);
	udc->dev = pdev;

	udc->gadget.is_dualspeed = 1;
	udc->gadget.is_otg = 0;
	udc->gadget.is_a_peripheral = 0;
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	/* Get the IRQ for the internal handling of the EPs */
	retval = request_irq(IRQ_USBD, s3c24xx_udc_irq,
			     IRQF_DISABLED, pdev->name, udc);
	if (retval) {
		pk_err("Cannot get irq %i, err %d\n", IRQ_USBD, retval);
		retval = -EBUSY;
		goto err_iounmap;
	}

	/* Activate the driver first when is going to be used */
	disable_irq(IRQ_USBD);
	pk_dbg("IRQ %i for the UDC\n", IRQ_USBD);

	if (mach_info && mach_info->vbus_pin > 0) {
		udc->irq_vbus = gpio_to_irq(mach_info->vbus_pin);
		retval = request_irq(udc->irq_vbus,
				     s3c24xx_udc_vbus_irq,
				     IRQF_DISABLED | IRQF_SHARED |
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				     pdev->name, udc);
		if (retval) {
			pk_err("Can't get vbus IRQ (%i) for IO %i (err %d)\n",
				   udc->irq_vbus, mach_info->vbus_pin, retval);
			goto err_free_udc_irq;
		}
		pk_dbg("IRQ %i for vbus detection\n", udc->irq_vbus);
	} else
		udc->vbus = 1;

	/*
	 * Init the tasklet for the IN-endpoints at this place, so that we can kill
	 * the tasklets when the module is going to be removed
	 * (Luis Galdos)
	 */
	for (cnt = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		struct s3c_ep *ep = &udc->ep[cnt];

		if (ep_is_in(ep)) {
			tasklet_init(&ep->in_tasklet,
				     s3c24xx_udc_epin_tasklet_func,
				     (unsigned long)ep);
		}

		spin_lock_init(&ep->lock);
	}

	/*
	 * Enable the wakeup support only if a interrupt IO for the bus detection
	 * was passed with the platform data
	 */
	if (udc->irq_vbus) {
		pk_dbg("Enabling the wakeup IRQ %i\n", udc->irq_vbus);
		device_init_wakeup(&pdev->dev, 1);
		device_set_wakeup_enable(&pdev->dev, 0);
	}

	return 0;

 err_free_udc_irq:
	free_irq(IRQ_USBD, udc);

 err_iounmap:
	iounmap(udc->base);

 err_free_mem:
	release_mem_region(udc->mem->start, IOMEMSIZE(udc->mem));

 err_exit:
	platform_set_drvdata(pdev, NULL);

	/*
	 * Unset the controller pointer otherwise the function for registering
	 * a new gadget can crash the system (it uses this pointer)
	 * (Luis Galdos)
	 */
	the_controller = NULL;
	return retval;
}


static int s3c24xx_udc_remove(struct platform_device *pdev)
{
	struct s3c2410_udc_mach_info *imach;
	struct s3c24xx_udc *udc;
	int cnt;
	struct s3c_ep *ep;

	pk_dbg("Removing the UDC driver (ID %i)\n", pdev->id);

	udc = platform_get_drvdata(pdev);
	imach = pdev->dev.platform_data;

	/* Kill the tasklet of all the IN-endpoints */
	for (cnt = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];
		if (ep_is_in(ep))
			tasklet_kill(&ep->in_tasklet);
	}

	s3c24xx_udc_disable(udc);
	usb_gadget_unregister_driver(udc->driver);

	free_irq(IRQ_USBD, udc);

	if (udc->irq_vbus) {
		pk_dbg("Disabling the wakeup IRQ %i\n", udc->irq_vbus);
		device_init_wakeup(&pdev->dev, 0);
	}

	/*
	 * If an IRQ for the vbus was passed, then disable it too
	 * (Luis Galdos)
	 */
	if (imach && udc->irq_vbus) {
		pk_dbg("Freeing the vbus IRQ %i\n", udc->irq_vbus);
		free_irq(udc->irq_vbus, udc);
		udc->irq_vbus = 0;
	}

	release_mem_region(udc->mem->start, IOMEMSIZE(udc->mem));

	platform_set_drvdata(pdev, NULL);

	the_controller = NULL;

	return 0;
}

/*
 * From another UDC-drivers seems to be, that is required to disconnect the
 * USB-device if it's connected to a host.
 */
#ifdef CONFIG_PM
static int s3c2443_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s3c24xx_udc *udc;
	int retval;

	udc = platform_get_drvdata(pdev);

	/* Need to disconnect first */
	if (udc->vbus)
		s3c24xx_udc_disable(udc);

	/* Enable the wakeup if requested */
	retval = 0;
	if (device_may_wakeup(&pdev->dev)) {

		/* Paranoic sanity check! */
		if (!udc->irq_vbus) {
			pk_err("No wakeup IRQ defined?\n");
			retval = -EINVAL;
			goto exit_suspend;
		}

		retval = enable_irq_wake(udc->irq_vbus);
		if (retval)
			goto exit_suspend;
	}

exit_suspend:
	return retval;
}

static int s3c2443_udc_resume(struct platform_device *pdev)
{
	struct s3c24xx_udc *udc;
	int retval;

	udc = platform_get_drvdata(pdev);

	retval = 0;
	if (device_may_wakeup(&pdev->dev)) {

		/* Paranoic sanity check */
		if (!udc->irq_vbus) {
			pk_err("No wakeup IRQ defined?\n");
			retval = -EINVAL;
			goto exit_resume;
		}

		disable_irq_wake(udc->irq_vbus);
	}

	/*
	 * Check the current state of the VBUS, then probably a host was connected
	 * during the suspend-time and this will not generate an interrupt
	 */
	udc->vbus = s3c2443_udc_vbus_state(udc);

	/* Enable the UDC for starting the enumeration */
	if (udc->vbus && udc->driver)
		retval = s3c24xx_udc_enable(udc);

exit_resume:
	return retval;
}
#else
#define s3c2443_udc_suspend				NULL
#define s3c2443_udc_resume				NULL
#endif

static struct platform_driver s3c24xx_udc_driver = {
	.probe		= s3c24xx_udc_probe,
	.remove		= s3c24xx_udc_remove,
	.suspend        = s3c2443_udc_suspend,
	.resume         = s3c2443_udc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
};

static int __init udc_init(void)
{
	int ret;

	pk_info("Loading (%s - %s)\n", DRIVER_BUILD_DATE, DRIVER_BUILD_TIME);
	ret = platform_driver_register(&s3c24xx_udc_driver);
	return ret;
}

static void __exit udc_exit(void)
{
	pk_info("Unloading (%s - %s)\n", DRIVER_BUILD_DATE, DRIVER_BUILD_TIME);
	platform_driver_unregister(&s3c24xx_udc_driver);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Luis Galdos, luis.galdos[at]digi.com");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
