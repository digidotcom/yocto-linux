/*
 * arch/arm/mach-ns9xxx/leds.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>

#include <asm/leds.h>

#include <mach/module.h>

static void cc9p9215_leds_event(led_event_t evt)
{
	switch(evt) {
	case led_idle_start:
		gpio_set_value(89, 1);
		break;
	case led_idle_end:
		gpio_set_value(89, 0);
		break;
	case led_green_on:
		if (module_is_ccw9p9215())
			gpio_set_value(88, 0);
		break;
	case led_green_off:
		if (module_is_ccw9p9215())
			gpio_set_value(88, 1);
		break;
	default:
		break;
	}
}

static int __init ns9xxx_init_leds(void)
{
	int ret;

	if (!module_is_cc9p9215() && !module_is_ccw9p9215())
		return -ENODEV;

	ret = gpio_request(89, "idle led");
	if (ret)
		return ret;

	if (module_is_ccw9p9215()) {
		ret = gpio_request(88, "wifi led");
		if (ret)
			return ret;
		/* switch the led, just in case it were on */
		gpio_set_value(88, 1);
	}

	leds_event = cc9p9215_leds_event;

	return 0;
}
device_initcall(ns9xxx_init_leds);
