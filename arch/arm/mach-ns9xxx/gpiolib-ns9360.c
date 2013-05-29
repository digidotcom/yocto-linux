/*
 * arch/arm/mach-ns9xxx/gpiolib-ns9360.c
 *
 * Copyright (C) 2006-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/gpio.h>

static int ns9360_gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	return gpio_get_value_ns9360(chip->base + offset);
}

static void ns9360_gpio_chip_set(struct gpio_chip *chip,
		unsigned offset, int value)
{
	gpio_set_value_ns9360_unlocked(chip->base + offset, value);
}

static int ns9360_gpio_chip_dir_input(struct gpio_chip *chip, unsigned offset)
{
	return gpio_direction_irqinput_ns9360_unlocked(chip->base + offset);
}

static int ns9360_gpio_chip_dir_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	return gpio_direction_output_ns9360_unlocked(chip->base + offset,
			value);
}

static struct gpio_chip ns9360_gpio_chip = {
	.label = "ns9360-system-gpio",
	.direction_input = ns9360_gpio_chip_dir_input,
	.direction_output = ns9360_gpio_chip_dir_output,
	.get = ns9360_gpio_chip_get,
	.set = ns9360_gpio_chip_set,
	.base = 0,
	.ngpio = 73,
};

int __init ns9360_gpio_init(void)
{
	if (!processor_is_ns9360())
		return -EINVAL;

	return gpiochip_add(&ns9360_gpio_chip);
}

