/*
 * arch/arm/mach-ns9xxx/time-ns921x.c
 *
 * Copyright (C) 2007 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/stringify.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <mach/regs-sys-ns921x.h>
#include <mach/irqs.h>

#include "irq.h"
#include "processor-ns921x.h"

#define TIMER_CLOCKSOURCE 0
#define TIMER_CLOCKEVENT 1
static u32 latch;

/**
 * The Read an Capture register is divided in two parts of 16 bits that allows
 * to configure the timer as 16 or 32 bits. When configured as 32 bits, it
 * seems like the read is not totally atomic, as many times the lower 2 bytes
 * are stuck in 0xffff, producing a wrong value which might result in readings
 * going backwards in time. In this case a new read is necessary.
 * TODO: this is a HW bug to be verified with the ASIC team
 */
static cycle_t ns921x_clocksource_read(struct clocksource *cs)
{
	u32 newval, clkval;

	clkval = __raw_readl(SYS_TRC(TIMER_CLOCKSOURCE));
	if ((u16)clkval == 0xffff) {
		newval = __raw_readl(SYS_TRC(TIMER_CLOCKSOURCE));
		if (newval < clkval)
		       clkval = newval;
	}
	return clkval;
}

static struct clocksource ns921x_clocksource = {
	.name	= "ns921x-timer" __stringify(TIMER_CLOCKSOURCE),
	.rating	= 300,
	.read	= ns921x_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 20,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void ns921x_clockevent_setmode(enum clock_event_mode mode,
		struct clock_event_device *clk)
{
	u32 oldtc, tc = __raw_readl(SYS_TC(TIMER_CLOCKEVENT));
	oldtc = tc;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		__raw_writel(latch, SYS_TRCC(TIMER_CLOCKEVENT));
		REGSET(tc, SYS_TCx, RELENBL, EN);
		REGSET(tc, SYS_TCx, INTSEL, EN);
		REGSET(tc, SYS_TCx, TE, EN);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		REGSET(tc, SYS_TCx, RELENBL, DIS);
		REGSET(tc, SYS_TCx, INTSEL, EN);
		REGSET(tc, SYS_TCx, TE, DIS);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		REGSET(tc, SYS_TCx, TE, DIS);
		break;
	}

	/* ack an possibly pending irq
	 * This might stuck the irq priority encoder, but only until the next
	 * timer irq ... */
	if (REGGET(tc, SYS_TCx, TE) == SYS_TCx_TE_DIS &&
			REGGET(oldtc, SYS_TCx, TE) == SYS_TCx_TE_EN) {
		REGSETIM(tc, SYS_TCx, INTCLR, 1);
		__raw_writel(tc, SYS_TC(TIMER_CLOCKEVENT));
		REGSETIM(tc, SYS_TCx, INTCLR, 0);
	}

	__raw_writel(tc, SYS_TC(TIMER_CLOCKEVENT));
}

static int ns921x_clockevent_setnextevent(unsigned long evt,
		struct clock_event_device *clk)
{
	u32 tc = __raw_readl(SYS_TC(TIMER_CLOCKEVENT));

	if (REGGET(tc, SYS_TCx, TE)) {
		REGSET(tc, SYS_TCx, TE, DIS);
		__raw_writel(tc, SYS_TC(TIMER_CLOCKEVENT));
	}

	REGSET(tc, SYS_TCx, TE, EN);

	__raw_writel(evt, SYS_TRCC(TIMER_CLOCKEVENT));

	__raw_writel(tc, SYS_TC(TIMER_CLOCKEVENT));

	return 0;
}

static struct clock_event_device ns921x_clockevent_device = {
	.name		= "ns921x-timer" __stringify(TIMER_CLOCKEVENT),
	.shift		= 20,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= ns921x_clockevent_setmode,
	.set_next_event	= ns921x_clockevent_setnextevent,
};

static irqreturn_t ns921x_clockevent_handler(int irq, void *dev_id)
{
	int timerno = irq - IRQ_NS921X_TIMER0;
	u32 tc;

	struct clock_event_device *evt = &ns921x_clockevent_device;

	/* clear irq */
	tc = __raw_readl(SYS_TC(timerno));
	if (REGGET(tc, SYS_TCx, RELENBL) == SYS_TCx_RELENBL_DIS) {
		REGSET(tc, SYS_TCx, TE, DIS);
		__raw_writel(tc, SYS_TC(timerno));
	}
	REGSETIM(tc, SYS_TCx, INTCLR, 1);
	__raw_writel(tc, SYS_TC(timerno));
	REGSETIM(tc, SYS_TCx, INTCLR, 0);
	__raw_writel(tc, SYS_TC(timerno));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction ns921x_clockevent_action = {
	.name		= "ns921x-timer" __stringify(TIMER_CLOCKEVENT),
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= ns921x_clockevent_handler,
};

static void __init ns921x_timer_init(void)
{
	int tc;

	tc = __raw_readl(SYS_TC(TIMER_CLOCKSOURCE));
	if (REGGET(tc, SYS_TCx, TE)) {
		REGSET(tc, SYS_TCx, TE, DIS);
		__raw_writel(tc, SYS_TC(TIMER_CLOCKSOURCE));
	}

	__raw_writel(0, SYS_TRC(TIMER_CLOCKSOURCE));

	REGSET(tc, SYS_TCx, TE, EN);
	REGSET(tc, SYS_TCx, DEBUG, STOP);
	REGSET(tc, SYS_TCx, TCS, 2AHB);
	REGSET(tc, SYS_TCx, MODE, INTERNAL);
	REGSET(tc, SYS_TCx, INTSEL, DIS);
	REGSET(tc, SYS_TCx, UPDOWN, UP);
	REGSET(tc, SYS_TCx, BITTIMER, 32);
	REGSET(tc, SYS_TCx, RELENBL, EN);

	__raw_writel(tc, SYS_TC(TIMER_CLOCKSOURCE));

	ns921x_clocksource.mult = clocksource_hz2mult(ns921x_ahbclock() * 2,
			ns921x_clocksource.shift);

	clocksource_register(&ns921x_clocksource);

	latch = SH_DIV(ns921x_ahbclock() * 2, HZ, 0);

	tc = __raw_readl(SYS_TC(TIMER_CLOCKEVENT));
	REGSET(tc, SYS_TCx, TE, DIS);
	REGSET(tc, SYS_TCx, DEBUG, STOP);
	REGSET(tc, SYS_TCx, TCS, 2AHB);
	REGSET(tc, SYS_TCx, MODE, INTERNAL);
	REGSET(tc, SYS_TCx, INTSEL, DIS);
	REGSET(tc, SYS_TCx, UPDOWN, DOWN);
	REGSET(tc, SYS_TCx, BITTIMER, 32);
	REGSET(tc, SYS_TCx, RELENBL, EN);
	__raw_writel(tc, SYS_TC(TIMER_CLOCKEVENT));

	ns921x_clockevent_device.mult = div_sc(ns921x_ahbclock() * 2,
			NSEC_PER_SEC, ns921x_clockevent_device.shift);
	ns921x_clockevent_device.max_delta_ns =
		clockevent_delta2ns(-1, &ns921x_clockevent_device);
	ns921x_clockevent_device.min_delta_ns =
		clockevent_delta2ns(1, &ns921x_clockevent_device);

	ns921x_clockevent_device.cpumask = cpumask_of(0);
	clockevents_register_device(&ns921x_clockevent_device);

	setup_irq(IRQ_NS921X_TIMER0 + TIMER_CLOCKEVENT,
			&ns921x_clockevent_action);
}

struct sys_timer ns921x_timer = {
	.init = ns921x_timer_init,
};
