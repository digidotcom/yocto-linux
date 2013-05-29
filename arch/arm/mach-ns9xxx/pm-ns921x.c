/*
 * arch/arm/mach-ns9xxx/pm-ns921x.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/leds.h>

#include <mach/processor.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/irqs.h>

#include "processor-ns921x.h"

#define NS921X_PM_ENET		(1 << 0)
#define NS921X_PM_UART		(0xf << 1)
#define NS921X_PM_UARTA		(1 << 1)
#define NS921X_PM_UARTB		(1 << 2)
#define NS921X_PM_UARTC		(1 << 3)
#define NS921X_PM_UARTD		(1 << 4)
#define NS921X_PM_SPI		(1 << 5)
#define NS921X_PM_I2C		(1 << 11)
#define NS921X_PM_RTC		(1 << 12)
#define NS921X_PM_I2C		(1 << 11)
#define NS921X_PM_EXT0		(1 << 16)
#define NS921X_PM_EXT1		(1 << 17)
#define NS921X_PM_EXT2		(1 << 18)
#define NS921X_PM_EXT3		(1 << 19)
#define NS921X_PM_ALL_WK	0xf183f

#if defined(DEBUG)
#define REGMAP(reg) { .name = #reg, .addr = reg, }
static const struct {
	const char *name;
	void __iomem *addr;
} regmap[] = {
	REGMAP(SYS_ISA),
	REGMAP(SYS_ISR),
	REGMAP(SYS_CLOCK),
	REGMAP(SYS_POWER),
};

static void dump_regs(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(regmap); ++i)
		pr_debug("%s = 0x%08x\n", regmap[i].name,
				__raw_readl(regmap[i].addr));
}
#else
#define dump_regs() ((void)0)
#endif

static irqreturn_t ns921x_cpuwake_irq(int irq, void *dev_id)
{
	u32 power = __raw_readl(SYS_POWER);

	/* ack wake irq */
	__raw_writel(power | SYS_POWER_INTCLR, SYS_POWER);
	__raw_writel(power, SYS_POWER);

	return IRQ_HANDLED;
}

static int ns921x_pm_prepare(void)
{
	int ret;

	ret = request_irq(IRQ_NS921X_CPUWAKE, ns921x_cpuwake_irq, 0, "cpuwake",
			NULL);
	if (ret) {
		pr_debug("%s: err_%s -> %d\n",
				__func__, "request_irq_cpuwake", ret);
	}

	return ret;
}

extern int ns9xxx_is_enabled_irq(unsigned int irq);

static int ns921x_pm_enter(suspend_state_t state)
{
	u32 power = __raw_readl(SYS_POWER) | SYS_POWER_SLFRFSH;

	/* ack any possible wake up irq before going to sleep... */
	__raw_writel(power | SYS_POWER_INTCLR, SYS_POWER);
	__raw_writel(power, SYS_POWER);

	power &= NS921X_PM_ALL_WK;
	if (!power)
		goto pm_enter_err;

	/* Verify that, if only one source is enabled that at least
	 * that irq is enabled */
	if (!(power & ~NS921X_PM_ENET)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9XXX_ETHRX))
			goto pm_enter_err;
	}

	if ((!(power & ~NS921X_PM_UARTA) && !ns9xxx_is_enabled_irq(IRQ_NS921X_UARTA)) ||
	    (!(power & ~NS921X_PM_UARTB) && !ns9xxx_is_enabled_irq(IRQ_NS921X_UARTB)) ||
	    (!(power & ~NS921X_PM_UARTC) && !ns9xxx_is_enabled_irq(IRQ_NS921X_UARTC)) ||
	    (!(power & ~NS921X_PM_UARTD) && !ns9xxx_is_enabled_irq(IRQ_NS921X_UARTD)))
		goto pm_enter_err;

	if (!(power & ~NS921X_PM_RTC)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9215_RTC))
			goto pm_enter_err;
	}

	if (!(power & ~NS921X_PM_EXT0)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9XXX_EXT0))
			goto pm_enter_err;
	}

	if (!(power & ~NS921X_PM_EXT1)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9XXX_EXT1))
			goto pm_enter_err;
	}

	if (!(power & ~NS921X_PM_EXT2)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9XXX_EXT2))
			goto pm_enter_err;
	}

	if (!(power & ~NS921X_PM_EXT3)) {
		if (!ns9xxx_is_enabled_irq(IRQ_NS9XXX_EXT3))
			goto pm_enter_err;
	}

	leds_event(led_idle_start);
	asm volatile("mcr p15, 0, %0, c7, c0, 4" : : "r" (0));
	leds_event(led_idle_end);

	return 0;

pm_enter_err:
	pr_warning("No wakeup source or interface disabled, not going to sleep (0x%08x)\n",
		   __raw_readl(SYS_POWER));
	return -EDEADLK;

}

static void ns921x_pm_finish(void)
{
	free_irq(IRQ_NS921X_CPUWAKE, NULL);
}

static struct platform_suspend_ops ns921x_pm_ops = {
	.valid = suspend_valid_only_mem,
	.prepare = ns921x_pm_prepare,
	.enter = ns921x_pm_enter,
	.finish = ns921x_pm_finish,
};

static int __init ns921x_pm_init(void)
{
	if (!processor_is_ns921x())
		return -ENODEV;

	suspend_set_ops(&ns921x_pm_ops);
	return 0;
}
arch_initcall(ns921x_pm_init);
