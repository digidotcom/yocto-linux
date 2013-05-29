/* -*- linux-c -*-
 *
 * linux/arch/arm/mach-s3c2443/cc9m2443js-usb.c
 *
 * Copyright (c) 2004,2005 Simtec Electronics
 *   Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.simtec.co.uk/products/EB2410ITX/
 *
 * Based on the Simtec platform code
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/bast-map.h>
#include <mach/bast-irq.h>
#include <plat/usb-control.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>

#include <mach/hardware.h>
#include <asm/irq.h>

#include <plat/devs.h>

#include "cc9m2443js-usb.h"

/* control power and monitor over-current events on various Simtec
 * designed boards.
 */

static unsigned int power_state[2];

static void cc9m2443js_usb_timer_func(unsigned long _info)
{
	struct s3c2410_hcd_info *info;

	info = (struct s3c2410_hcd_info *)_info;

	s3c2410_usb_report_oc(info, 0);

	/*
	 * IMPORTANT: Some power switches, like the MAX890, reports an overload
	 * when they are powered on. For avoiding endless overload reports it's
	 * required to wait for some time (> 400usecs.) before reenabling the
	 * interrupt line of the overload detection.
	 * (Luis Galdos)
	 */
	mdelay(1);
	
	enable_irq(info->oc_irq);
}

/*
 * If the USB-PHY is used as second root hub port, then we must take care of its
 * current state, otherwise only the first port is considered.
 */
#if defined(CONFIG_S3C2443_USB_PHY_PORT)
# define CC9M2443JS_USB_PORT_ON(state)		(state[0] && state[1])
#else
# define CC9M2443JS_USB_PORT_ON(state)		(state[0])
#endif /* CONFIG_S3C2443_USB_PHY_PORT */

static void cc9m2443js_usb_powercontrol(struct s3c2410_hcd_info * info, int port, int to)
{
	pr_debug("USB power: port %d | to %d [%d %d]\n",
		 port, to, power_state[0], power_state[1]);

	power_state[port] = to;

	/* @XXX: Power control without a configured GPIO? */
	if (!info->pw_gpio)
		return;
	
	if (CC9M2443JS_USB_PORT_ON(power_state))
		s3c2443_gpio_setpin(info->pw_gpio, 1 ^ info->pw_gpio_inv);
	else
		s3c2443_gpio_setpin(info->pw_gpio, 0 ^ info->pw_gpio_inv);
}

static irqreturn_t cc9m2443js_usb_ocirq(int irq, void *_info)
{
	struct s3c2410_hcd_info *info = _info;

	if (s3c2410_gpio_getpin(info->oc_gpio) == 0) {
		s3c2410_usb_report_oc(info, 3);
		pr_info("USB IRQ: Overcurrent Detected\n");
	} else {

		disable_irq_nosync(info->oc_irq);
		pr_debug("USB IRQ: Scheduling timer for clearing OC\n");
		mod_timer(&info->timer, jiffies + msecs_to_jiffies(100));
	}

	return IRQ_HANDLED;
}

/*
 * The S3C2443 doesn't have a dedicated interrupt for the OC detection.
 * We use the external interrupt line connected to the GPG8
 */
static void cc9m2443js_usb_enableoc(struct s3c2410_hcd_info *info, int on)
{
	int ret;

	pr_debug("Command to %s OC detection\n", on ? "enable" : "disable");
	
	info->oc_irq = gpio_to_irq(info->oc_gpio);
	if (info->oc_irq < 0) {
		printk(KERN_ERR "[ ERROR ] Couldn't get a GPIO interrupt\n");
		return;
	}
	
	if (on) {
		ret = request_irq(info->oc_irq, cc9m2443js_usb_ocirq,
				  IRQF_DISABLED | IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "s3c2443 USB OC", info);
		if (ret != 0)
			printk(KERN_ERR "[ ERROR ] Failed to request usb oc irq\n");
	} else
		free_irq(info->oc_irq, info);
}

static struct s3c2410_hcd_info cc9m2443js_usb_info = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED
	},

	/* Enable the second port if required */
#if defined(CONFIG_S3C2443_USB_PHY_PORT)
	.port[1]	= {
		.flags	= S3C_HCDFLG_USED
	},
#endif /* CONFIG_S3C2443_USB_PHY_PORT */
	
	.power_control	= cc9m2443js_usb_powercontrol,
	.enable_oc	= cc9m2443js_usb_enableoc,

	/* Over current GPIO */
	.oc_gpio	= S3C2410_GPG(8),
	.oc_gpio_cfg	= S3C2410_GPG8_EINT16,

	/* Power switch GPIO (inverted by the CC9M2443JS) */
	.pw_gpio	= S3C2410_GPA(14),
	.pw_gpio_cfg	= (0<<14), /* Output */
	.pw_gpio_inv	= 1,
};

int __init cc9m2443js_usb_init(void)
{
	struct s3c2410_hcd_info *info = &cc9m2443js_usb_info;

	s3c_device_ohci.dev.platform_data = &cc9m2443js_usb_info;

	/* Configure the GPIO and disable the pullups */
	if (info->oc_gpio) {
		s3c2443_gpio_cfgpin(info->oc_gpio, info->oc_gpio_cfg);
		s3c2443_gpio_extpull(info->oc_gpio, 0);
		s3c2443_gpio_set_udp(info->oc_gpio, 1);
	}

	if (info->pw_gpio) {
		s3c2443_gpio_cfgpin(info->pw_gpio, info->pw_gpio_cfg);

		/* At first power off the ports */
		s3c2443_gpio_setpin(info->pw_gpio, 0 ^ info->pw_gpio_inv);
	}

	/* Init the internal timer */
	init_timer(&info->timer);
	info->timer.expires = 0;
	info->timer.data = (unsigned long)info;
	info->timer.function = cc9m2443js_usb_timer_func;
	add_timer(&info->timer);

	return 0;
}
