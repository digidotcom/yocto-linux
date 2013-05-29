/*
 * arch/arm/mach-ns9xxx/gpio-ns9360.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/module.h>

#include <asm/gpio.h>
#include <mach/irqs.h>

static const struct gpio_to_irq_map gpio_to_irq_map_ns9360[] = {
	{ .gpio =   1, .irq = IRQ_NS9XXX_EXT0, .func = 2, },
	{ .gpio =   7, .irq = IRQ_NS9XXX_EXT1, .func = 2, },
	{ .gpio =  11, .irq = IRQ_NS9XXX_EXT2, .func = 1, },
	{ .gpio =  13, .irq = IRQ_NS9XXX_EXT0, .func = 1, },
	{ .gpio =  18, .irq = IRQ_NS9XXX_EXT3, .func = 2, },
	{ .gpio =  28, .irq = IRQ_NS9XXX_EXT1, .func = 0, },
	{ .gpio =  32, .irq = IRQ_NS9XXX_EXT2, .func = 0, },
	{ .gpio =  40, .irq = IRQ_NS9XXX_EXT3, .func = 1, },
	{ .gpio =  68, .irq = IRQ_NS9XXX_EXT0, .func = 2, },
	{ .gpio =  69, .irq = IRQ_NS9XXX_EXT1, .func = 2, },
};

const struct gpio_to_irq_map *gpio_get_map_ns9360(unsigned gpio)
{
	return gpio_get_map(gpio, gpio_to_irq_map_ns9360,
			ARRAY_SIZE(gpio_to_irq_map_ns9360));
}
EXPORT_SYMBOL(gpio_get_map_ns9360);
