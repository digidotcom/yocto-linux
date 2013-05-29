/*
 * drivers/leds/leds-pwm.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * Code rebased on original leds-pwm.c from Bill Gatliff
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
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/pwm-led.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>


struct led_pwm {
	struct led_classdev	led;
	struct pwm_channel	*pwm;
	unsigned long period;
};

static void
led_pwm_brightness_set(struct led_classdev *c,
		       enum led_brightness b)
{
	struct led_pwm *led;
	unsigned long period;

	period = 1000000000UL;
	led = container_of(c, struct led_pwm, led);
	led->period = period;
	pwm_set_period_ns(led->pwm, period);
}


static enum led_brightness
led_pwm_brightness_get(struct led_classdev *c)
{
	struct led_pwm *led;
	led = container_of(c, struct led_pwm, led);
	return led->period;
}

static int
led_pwm_blink_set(struct led_classdev *c,
		  unsigned long *on_ms,
		  unsigned long *off_ms)
{
	struct led_pwm *led;
	struct pwm_channel_config cfg;

	led = container_of(c, struct led_pwm, led);

	if (*on_ms == 0 && *off_ms == 0) {
		*on_ms = 1000UL;
		*off_ms = 1000UL;
	}

	cfg.config_mask = PWM_CONFIG_DUTY_NS
		| PWM_CONFIG_PERIOD_NS;

	cfg.duty_ns = *on_ms * 1000000000UL;
	cfg.period_ns = (*on_ms + *off_ms) * 1000000000UL;

	return pwm_config(led->pwm, &cfg);
}


static int __init
led_pwm_probe(struct platform_device *pdev)
{
	struct pwm_led_platform_data *pdata = pdev->dev.platform_data;
	struct led_pwm *led;
	int ret;

	if (!pdata || !pdata->led_info) {
		return -EINVAL;
	}

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->pwm = pwm_request(pdata->bus_id, pdata->chan,
				pdata->led_info->name);

	if (!led->pwm) {
		ret = -EINVAL;
		goto err_pwm_request;
	}

	platform_set_drvdata(pdev, led);

	led->led.name = pdata->led_info->name;
	led->led.default_trigger = pdata->led_info->default_trigger;
	led->led.brightness_set = led_pwm_brightness_set;
	led->led.brightness_get = led_pwm_brightness_get;
	led->led.blink_set = led_pwm_blink_set;
	led->led.brightness = LED_OFF;

	ret = pwm_config(led->pwm, pdata->config);
	if (ret)
		goto err_pwm_config;

	pwm_start(led->pwm);

	ret = led_classdev_register(&pdev->dev, &led->led);
	if (ret < 0)
		goto err_classdev_register;

	return 0;

err_classdev_register:
	pwm_stop(led->pwm);
err_pwm_config:
	pwm_free(led->pwm);
err_pwm_request:
	kfree(led);

	return ret;
}

static int
led_pwm_remove(struct platform_device *pdev)
{
	struct led_pwm *led = platform_get_drvdata(pdev);

	led_classdev_unregister(&led->led);

	if (led->pwm) {
		pwm_stop(led->pwm);
		pwm_free(led->pwm);
	}

	kfree(led);
	platform_set_drvdata(pdev, NULL);

	return 0;
}


static struct platform_driver led_pwm_driver = {
	.driver = {
		.name =		"leds-pwm",
		.owner =	THIS_MODULE,
	},
	.probe = led_pwm_probe,
	.remove = led_pwm_remove,
};


static int __init led_pwm_modinit(void)
{
	return platform_driver_register(&led_pwm_driver);
}
late_initcall(led_pwm_modinit);


static void __exit led_pwm_modexit(void)
{
	platform_driver_unregister(&led_pwm_driver);
}
module_exit(led_pwm_modexit);


MODULE_AUTHOR("Hector Oron <Hector.Oron <at> digi.com>");
MODULE_DESCRIPTION("Driver for LEDs with PWM-controlled brightness");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
