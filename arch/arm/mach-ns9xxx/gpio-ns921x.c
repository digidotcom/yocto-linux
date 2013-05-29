/*
 * arch/arm/mach-ns9xxx/gpio-ns921x.c
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

static const struct gpio_to_irq_map gpio_to_irq_map_ns921x[] = {
        { .gpio =   1, .irq = IRQ_NS9XXX_EXT0, .func = 1, },
        { .gpio =   2, .irq = IRQ_NS9XXX_EXT1, .func = 1, },
        { .gpio =   4, .irq = IRQ_NS9XXX_EXT2, .func = 1, },
        { .gpio =   5, .irq = IRQ_NS9XXX_EXT3, .func = 1, },
        { .gpio =   9, .irq = IRQ_NS9XXX_EXT0, .func = 2, },
        { .gpio =  10, .irq = IRQ_NS9XXX_EXT1, .func = 2, },
        { .gpio =  16, .irq = IRQ_NS9XXX_EXT0, .func = 2, },
        { .gpio =  17, .irq = IRQ_NS9XXX_EXT1, .func = 2, },
        { .gpio =  18, .irq = IRQ_NS9XXX_EXT2, .func = 2, },
        { .gpio =  19, .irq = IRQ_NS9XXX_EXT3, .func = 2, },
#if defined(CONFIG_PROCESSOR_NS9215)
        { .gpio =  67, .irq = IRQ_NS9XXX_EXT3, .func = 2, },
        { .gpio = 101, .irq = IRQ_NS9XXX_EXT3, .func = 2, },
#endif
        { .gpio = 104, .irq = IRQ_NS9XXX_EXT0, .func = 2, },
        { .gpio = 105, .irq = IRQ_NS9XXX_EXT1, .func = 2, },
        { .gpio = 106, .irq = IRQ_NS9XXX_EXT2, .func = 2, },
};

const struct gpio_to_irq_map *gpio_get_map_ns921x(unsigned gpio)
{
	return gpio_get_map(gpio, gpio_to_irq_map_ns921x,
			ARRAY_SIZE(gpio_to_irq_map_ns921x));
}
EXPORT_SYMBOL(gpio_get_map_ns921x);
