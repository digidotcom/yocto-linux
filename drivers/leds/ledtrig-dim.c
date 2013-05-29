/*
 * LED Dim Trigger
 *
 * Copyright (C) 2008 Bill Gatliff <bgat <at> billgatliff.com>
 *
 * "Dims" an LED based on system load.  Derived from Atsushi Nemoto's
 * ledtrig-heartbeat.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/leds.h>
#include <linux/slab.h>

#include "leds.h"

struct dim_trig_data {
	struct timer_list timer;
};


static void
led_dim_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *)data;
	struct dim_trig_data *dim_data = led_cdev->trigger_data;
	unsigned int brightness;

	brightness = ((LED_FULL - LED_OFF) * avenrun[0]) / EXP_1;
	if (brightness > LED_FULL)
		brightness = LED_FULL;

	led_set_brightness(led_cdev, brightness);
	mod_timer(&dim_data->timer, jiffies + msecs_to_jiffies(500));
}


static void
dim_trig_activate(struct led_classdev *led_cdev)
{
	struct dim_trig_data *dim_data;

	dim_data = kzalloc(sizeof(*dim_data), GFP_KERNEL);
	if (!dim_data)
		return;

	led_cdev->trigger_data = dim_data;
	setup_timer(&dim_data->timer,
		    led_dim_function, (unsigned long)led_cdev);
	led_dim_function(dim_data->timer.data);
}


static void
dim_trig_deactivate(struct led_classdev *led_cdev)
{
	struct dim_trig_data *dim_data = led_cdev->trigger_data;

	if (dim_data) {
		del_timer_sync(&dim_data->timer);
		kfree(dim_data);
	}
}


static struct led_trigger dim_led_trigger = {
	.name     = "dim",
	.activate = dim_trig_activate,
	.deactivate = dim_trig_deactivate,
};


static int __init dim_trig_init(void)
{
	return led_trigger_register(&dim_led_trigger);
}
module_init(dim_trig_init);


static void __exit dim_trig_exit(void)
{
	led_trigger_unregister(&dim_led_trigger);
}
module_exit(dim_trig_exit);


MODULE_AUTHOR("Bill Gatliff <bgat <at> billgatliff.com>");
MODULE_DESCRIPTION("Dim LED trigger");
MODULE_LICENSE("GPL");
