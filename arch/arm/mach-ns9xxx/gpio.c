/*
 * arch/arm/mach-ns9xxx/gpio.c
 *
 * Copyright (C) 2006-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <asm/bug.h>
#include <asm/types.h>
#include <asm/bitops.h>

#include <mach/gpio.h>
#include <mach/processor.h>

DEFINE_SPINLOCK(gpio_lock);

/* only access gpiores with atomic ops */
static DECLARE_BITMAP(gpiores, NS9XXX_NUM_GPIO);

int gpio_request(unsigned gpio, const char *label)
{
	if (likely(gpio_issocgpio(gpio)))
		return test_and_set_bit(gpio, gpiores) ? -EBUSY : 0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	clear_bit(gpio, gpiores);
	return;
}
EXPORT_SYMBOL(gpio_free);
