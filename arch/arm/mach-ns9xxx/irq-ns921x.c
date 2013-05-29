/*
 * arch/arm/mach-ns9xxx/irq-ns921x.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/regs-sys-ns921x.h>

#include "processor-ns921x.h"

static const u32 ns921x_irq2powermask[] = {
	[IRQ_NS9XXX_ETHRX] = SYS_POWER_ETH,
	[IRQ_NS921X_UARTA] = SYS_POWER_UARTA,
	[IRQ_NS921X_UARTB] = SYS_POWER_UARTB,
	[IRQ_NS921X_UARTC] = SYS_POWER_UARTC,
	[IRQ_NS921X_UARTD] = SYS_POWER_UARTD,
	[IRQ_NS921X_SPI] = SYS_POWER_SPI,
	[IRQ_NS921X_I2C] = SYS_POWER_I2C,
	[IRQ_NS9215_RTC] = SYS_POWER_RTC,
	[IRQ_NS9XXX_EXT0] = SYS_POWER_EXTIRQ0,
	[IRQ_NS9XXX_EXT1] = SYS_POWER_EXTIRQ1,
	[IRQ_NS9XXX_EXT2] = SYS_POWER_EXTIRQ2,
	[IRQ_NS9XXX_EXT3] = SYS_POWER_EXTIRQ3,
};

int ns921x_set_wake_irq(unsigned int irq, unsigned int on)
{
	unsigned long flags;
	u32 power;

	if (irq > ARRAY_SIZE(ns921x_irq2powermask) ||
			!ns921x_irq2powermask[irq])
		return -ENOENT;

	local_irq_save(flags);

	power = __raw_readl(SYS_POWER);

	if (on)
		power |= ns921x_irq2powermask[irq];
	else
		power &= ~ns921x_irq2powermask[irq];

	__raw_writel(power, SYS_POWER);

	local_irq_restore(flags);

	return 0;
}
