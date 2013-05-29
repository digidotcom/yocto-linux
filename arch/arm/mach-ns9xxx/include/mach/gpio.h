/*
 * arch/arm/mach-ns9xxx/include/mach/gpio.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
*/
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#include <linux/spinlock.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/processor.h>

#include <mach/regs-io-ns921x.h>
#include <mach/regs-bbu.h>


/* Macros for configuring the GPIO-functions */
#if defined(CONFIG_PROCESSOR_NS9210) || defined(CONFIG_PROCESSOR_NS9215)
# define NS921X_GPIO_FUNC_0                      (0x00)
# define NS921X_GPIO_FUNC_1                      (0x01)
# define NS921X_GPIO_FUNC_2                      (0x02)
# define NS921X_GPIO_FUNC_3                      (0x03)
# define NS921X_GPIO_FUNC_4                      (0x04)

# define NS921X_GPIO_FUNC_GPIO			NS921X_GPIO_FUNC_3

# define NS921X_GPIO_INPUT                       (0x00)
# define NS921X_GPIO_OUTPUT                      (0x01)

# define NS921X_GPIO_INVERT                      (0x01)
# define NS921X_GPIO_DONT_INVERT                 (0x00)

# define NS921X_GPIO_ENABLE_PULLUP               (0x00)
# define NS921X_GPIO_DISABLE_PULLUP              (0x01)
#endif

/* Macros for configuring the GPIO-functions */
#if defined(CONFIG_PROCESSOR_NS9360)
# define NS9360_GPIO_FUNC_0                      (0x00)
# define NS9360_GPIO_FUNC_1                      (0x01)
# define NS9360_GPIO_FUNC_2                      (0x02)
# define NS9360_GPIO_FUNC_3                      (0x03)
# define NS9360_GPIO_FUNC_4                      (0x04)

# define NS9360_GPIO_FUNC_GPIO			NS9360_GPIO_FUNC_3

# define NS9360_GPIO_INPUT                       (0x00)
# define NS9360_GPIO_OUTPUT                      (0x01)

# define NS9360_GPIO_INVERT                      (0x01)
# define NS9360_GPIO_DONT_INVERT                 (0x00)
#endif


struct gpio_to_irq_map {
	unsigned gpio;
	unsigned irq;
	int func;
};

#define NS9XXX_NUM_GPIO 0

#if defined(CONFIG_PROCESSOR_NS9210)
# define NS9XXX_NUM_GPIO_NS9210 108
# if NS9XXX_NUM_GPIO < NS9XXX_NUM_GPIO_NS9210
#  undef NS9XXX_NUM_GPIO
#  define NS9XXX_NUM_GPIO NS9XXX_NUM_GPIO_NS9210
# endif
static inline int gpio_issocgpio_ns9210(unsigned gpio)
{
	return gpio < NS9XXX_NUM_GPIO_NS9210 && !(gpio >= 50 && gpio < 105);
}
#endif

#if defined(CONFIG_PROCESSOR_NS9215)
# define NS9XXX_NUM_GPIO_NS9215 108
# if NS9XXX_NUM_GPIO < NS9XXX_NUM_GPIO_NS9215
#  undef NS9XXX_NUM_GPIO
#  define NS9XXX_NUM_GPIO NS9XXX_NUM_GPIO_NS9215
# endif
static inline int gpio_issocgpio_ns9215(unsigned gpio)
{
	return gpio < NS9XXX_NUM_GPIO_NS9215;
}
#endif

static const inline struct gpio_to_irq_map *gpio_get_map(unsigned gpio,
		const struct gpio_to_irq_map map[], size_t array_size)
{
	/* TODO: check if a binary search yields some performance advantage */
	int i;

	for (i = 0; i < array_size; ++i) {
		if (map[i].gpio == gpio)
			return &map[i];
	}

	return NULL;
}

#if defined(CONFIG_PROCESSOR_NS921X)
const struct gpio_to_irq_map *gpio_get_map_ns921x(unsigned gpio) __attribute__((const));

static inline void gpio_configure_ns921x_unlocked(unsigned gpio,
		int dir, int inv, int func, int dispullup)
{
	void __iomem *conf = NS921X_IO_GPIOCONFx(gpio / 4);
	u32 confval;

	confval = __raw_readl(conf);
	REGSETIM_IDX(confval, NS921X_IO_GPIOCONFx, DIR, gpio & 3, dir);
	REGSETIM_IDX(confval, NS921X_IO_GPIOCONFx, INV, gpio & 3, inv);
	REGSETIM_IDX(confval, NS921X_IO_GPIOCONFx, FUNC, gpio & 3, func);
	if (gpio != 9 && gpio != 12 && (gpio < 102 || gpio > 105))
		REGSETIM_IDX(confval, NS921X_IO_GPIOCONFx, PUEN, gpio & 3, dispullup);
	else {
		if (dispullup)
			pr_warning("cannot disable pullup for gpio %u\n", gpio);

		REGSETIM_IDX(confval, NS921X_IO_GPIOCONFx, PUEN, gpio & 3, 0);
	}

	__raw_writel(confval, conf);
}

static inline void gpio_configure_ns921x(unsigned gpio,
		int dir, int inv, int func, int dispullup)
{
	unsigned long flags;

	local_irq_save(flags);

	gpio_configure_ns921x_unlocked(gpio, dir, inv, func, dispullup);

	local_irq_restore(flags);
}

static inline int gpio_get_value_ns921x(unsigned gpio)
{
	void __iomem *stat = NS921X_IO_GPIOSTATx(gpio / 32);

	return (__raw_readl(stat) >> (gpio & 31)) & 1;
}

static inline void gpio_set_value_ns921x_unlocked(unsigned gpio, int value)
{
	void __iomem *ctrl = NS921X_IO_GPIOCTRLx(gpio / 32);
	u32 ctrlval;

	ctrlval = __raw_readl(ctrl);

	if (value)
		ctrlval |= 1 << (gpio & 31);
	else
		ctrlval &= ~(1 << (gpio & 31));

	__raw_writel(ctrlval, ctrl);
}

static inline int gpio_direction_input_ns921x_unlocked(unsigned gpio)
{
	gpio_configure_ns921x_unlocked(gpio, 0, 0, 3, 0);
	return 0;
}

static inline int gpio_direction_irqinput_ns921x_unlocked(unsigned gpio)
{
	const struct gpio_to_irq_map *map = gpio_get_map_ns921x(gpio);

	if (map) {
		gpio_configure_ns921x_unlocked(gpio, 0, 0, map->func, 0);
		return 0;
	} else
		return gpio_direction_input_ns921x_unlocked(gpio);
}

static inline int gpio_direction_output_ns921x_unlocked(unsigned gpio,
		int value)
{
	gpio_set_value_ns921x_unlocked(gpio, value);
	gpio_configure_ns921x_unlocked(gpio, 1, 0, 3, 0);
	return 0;
}

#endif /* if defined(CONFIG_PROCESSOR_NS921X) */

#if defined(CONFIG_PROCESSOR_NS9360)
# define NS9XXX_NUM_GPIO_NS9360 73
# if NS9XXX_NUM_GPIO < NS9XXX_NUM_GPIO_NS9360
#  undef NS9XXX_NUM_GPIO
#  define NS9XXX_NUM_GPIO NS9XXX_NUM_GPIO_NS9360
# endif
static inline void __iomem *gpio_ns9360_gstataddr(unsigned gpio)
{
	if (gpio < 32)
		return NS9360_BBU_GSTAT1;
	else if (gpio < 64)
		return NS9360_BBU_GSTAT2;
	else
		return NS9360_BBU_GSTAT3;
}

static inline void __iomem *gpio_ns9360_gctrladdr(unsigned gpio)
{
	if (gpio < 32)
		return NS9360_BBU_GCTRL1;
	else if (gpio < 64)
		return NS9360_BBU_GCTRL2;
	else
		return NS9360_BBU_GCTRL3;
}

static inline void __iomem *gpio_ns9360_gconfaddr(unsigned gpio)
{
	if (gpio < 56)
		return NS9360_BBU_GCONFb1(gpio / 8);
	else
		/* this could be optimised away on
		 * ns9750 only builds, but it isn't ...
		 */
		return NS9360_BBU_GCONFb2((gpio - 56) / 8);
}

static inline int gpio_issocgpio_ns9360(unsigned gpio)
{
	return gpio <= 72;
}

const struct gpio_to_irq_map *gpio_get_map_ns9360(unsigned gpio) __attribute__((const));

static inline void gpio_configure_ns9360_unlocked(unsigned gpio,
		int dir, int inv, int func)
{
	void __iomem *conf = gpio_ns9360_gconfaddr(gpio);
	u32 confval;

	confval = __raw_readl(conf);
	REGSETIM_IDX(confval, NS9360_BBU_GCONFx, DIR, gpio & 7, dir);
	REGSETIM_IDX(confval, NS9360_BBU_GCONFx, INV, gpio & 7, inv);
	REGSETIM_IDX(confval, NS9360_BBU_GCONFx, FUNC, gpio & 7, func);
	__raw_writel(confval, conf);
}

static inline void gpio_configure_ns9360(unsigned gpio,
		int dir, int inv, int func)
{
	unsigned long flags;

	local_irq_save(flags);

	gpio_configure_ns9360_unlocked(gpio, dir, inv, func);

	local_irq_restore(flags);
}

static inline int gpio_get_value_ns9360(unsigned gpio)
{
	void __iomem *stat = gpio_ns9360_gstataddr(gpio);

	return (__raw_readl(stat) >> (gpio & 31)) & 1;
}

static inline void gpio_set_value_ns9360_unlocked(unsigned gpio, int value)
{
	void __iomem *ctrl = gpio_ns9360_gctrladdr(gpio);
	u32 ctrlval;

	ctrlval = __raw_readl(ctrl);

	if (value)
		ctrlval |= 1 << (gpio & 31);
	else
		ctrlval &= ~(1 << (gpio & 31));

	__raw_writel(ctrlval, ctrl);
}

static inline int gpio_direction_input_ns9360_unlocked(unsigned gpio)
{
	gpio_configure_ns9360_unlocked(gpio, 0, 0, 3);
	return 0;
}

static inline int gpio_direction_irqinput_ns9360_unlocked(unsigned gpio)
{
	const struct gpio_to_irq_map *map = gpio_get_map_ns9360(gpio);

	if (map) {
		gpio_configure_ns9360_unlocked(gpio, 0, 0, map->func);
		return 0;
	} else
		return gpio_direction_input_ns9360_unlocked(gpio);
}

static inline int gpio_direction_output_ns9360_unlocked(unsigned gpio,
		int value)
{
	gpio_set_value_ns9360_unlocked(gpio, value);
	gpio_configure_ns9360_unlocked(gpio, 1, 0, 3);
	return 0;
}

#endif /* if defined(CONFIG_PROCESSOR_NS9360) */

static inline int gpio_issocgpio(unsigned gpio)
{
#if defined(CONFIG_PROCESSOR_NS9210)
	if (processor_is_ns9210())
		return gpio_issocgpio_ns9210(gpio);
	else
#endif
#if defined(CONFIG_PROCESSOR_NS9215)
	if (processor_is_ns9215())
		return gpio_issocgpio_ns9215(gpio);
	else
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
	if (processor_is_ns9360())
		return gpio_issocgpio_ns9360(gpio);
	else
#endif
		BUG();

	BUG();

	return 0;
}

#if defined(CONFIG_GPIOLIB)

#include <asm-generic/gpio.h>

static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	return __gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return __gpio_cansleep(gpio);
}

#else /* if defined(CONFIG_GPIOLIB) */

extern spinlock_t gpio_lock;

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);

static inline int gpio_get_value(unsigned gpio)
{
#if defined(CONFIG_PROCESSOR_NS921X)
	if (processor_is_ns921x())
		return gpio_get_value_ns921x(gpio);
	else
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
	if (processor_is_ns9360())
		return gpio_get_value_ns9360(gpio);
	else
#endif
		BUG();
}

static inline void gpio_set_value_unlocked(unsigned gpio, int value)
{
#if defined(CONFIG_PROCESSOR_NS921X)
	if (processor_is_ns921x())
		gpio_set_value_ns921x_unlocked(gpio, value);
	else
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
	if (processor_is_ns9360())
		gpio_set_value_ns9360_unlocked(gpio, value);
	else
#endif
		BUG();
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	unsigned long flags;

	BUG_ON(!gpio_issocgpio(gpio));

	spin_lock_irqsave(&gpio_lock, flags);

	gpio_set_value_unlocked(gpio, value);

	spin_unlock_irqrestore(&gpio_lock, flags);
}

static inline int gpio_direction_input(unsigned gpio)
{
	if (likely(gpio_issocgpio(gpio))) {
		int ret = -EINVAL;
		unsigned long flags;

		spin_lock_irqsave(&gpio_lock, flags);

#if defined(CONFIG_PROCESSOR_NS921X)
		if (processor_is_ns921x())
			ret = gpio_direction_irqinput_ns921x_unlocked(gpio);
		else
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
		if (processor_is_ns9360())
			ret = gpio_direction_irqinput_ns9360_unlocked(gpio);
		else
#endif
			BUG();

		spin_unlock_irqrestore(&gpio_lock, flags);

		return ret;
	} else
		return -EINVAL;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	if (likely(gpio_issocgpio(gpio))) {
		int ret = -EINVAL;
		unsigned long flags;

		spin_lock_irqsave(&gpio_lock, flags);

#if defined(CONFIG_PROCESSOR_NS921X)
		if (processor_is_ns921x())
			ret = gpio_direction_output_ns921x_unlocked(gpio,
					value);
		else
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
		if (processor_is_ns9360())
			ret = gpio_direction_output_ns9360_unlocked(gpio,
					value);
		else
#endif
			BUG();

		spin_unlock_irqrestore(&gpio_lock, flags);

		return ret;
	} else
		return -EINVAL;
}

#include <asm-generic/gpio.h>

#endif /* if defined(CONFIG_GPIOLIB) / else */


/*
 * ns9xxx can use gpio pins to trigger an irq, but it's not generic
 * enough to be supported by the gpio_to_irq/irq_to_gpio interface
 */
static inline int gpio_to_irq(unsigned gpio)
{
#if defined(CONFIG_PROCESSOR_NS921X)
	if (processor_is_ns921x()) {
		const struct gpio_to_irq_map *map = gpio_get_map_ns921x(gpio);

		if (map)
			return map->irq;
	}
#endif
#if defined(CONFIG_PROCESSOR_NS9360)
	if (processor_is_ns9360()) {
		const struct gpio_to_irq_map *map = gpio_get_map_ns9360(gpio);

		if (map)
			return map->irq;
	}
#endif

	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#endif /* ifndef __ASM_ARCH_GPIO_H */
