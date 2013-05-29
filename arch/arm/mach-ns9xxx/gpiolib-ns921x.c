/*
 * arch/arm/mach-ns9xxx/gpiolib-ns921x.c
 *
 * Copyright (C) 2007-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>

#include <asm/gpio.h>

#include <mach/processor.h>

static int ns921x_gpio_chip_get(struct gpio_chip *chip, unsigned offset)
{
	return gpio_get_value_ns921x(chip->base + offset);
}

static void ns921x_gpio_chip_set(struct gpio_chip *chip,
		unsigned offset, int value)
{
	gpio_set_value_ns921x_unlocked(chip->base + offset, value);
}

static int ns921x_gpio_chip_dir_input(struct gpio_chip *chip, unsigned offset)
{
	return gpio_direction_irqinput_ns921x_unlocked(chip->base + offset);
}

static int ns921x_gpio_chip_dir_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	return gpio_direction_output_ns921x_unlocked(chip->base + offset,
			value);
}

static struct gpio_chip ns921x_gpio_chip = {
	.label = "ns921x-system-gpio",
	.direction_input = ns921x_gpio_chip_dir_input,
	.direction_output = ns921x_gpio_chip_dir_output,
	.get = ns921x_gpio_chip_get,
	.set = ns921x_gpio_chip_set,
	.base = 0,
};

static struct gpio_chip ns921x_gpio_a_chip = {
	.label = "ns921x-system-gpio",
	.direction_input = ns921x_gpio_chip_dir_input,
	.direction_output = ns921x_gpio_chip_dir_output,
	.get = ns921x_gpio_chip_get,
	.set = ns921x_gpio_chip_set,
	.base = 104,
	.ngpio = 4,
};

int __init ns921x_gpio_init(void)
{
	int ret;

	if (processor_is_ns9210())
		ns921x_gpio_chip.ngpio = 50;
	else if (processor_is_ns9215())
		ns921x_gpio_chip.ngpio = 104;
	else
		return -EINVAL;

	ret = gpiochip_add(&ns921x_gpio_chip);
	if (ret)
		return ret;

	return gpiochip_add(&ns921x_gpio_a_chip);
}
