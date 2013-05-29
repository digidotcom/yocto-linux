/*
 * arch/arm/mach-ns9xxx/ns9215_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/ns9xxx-pwm.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/pwm-led.h>

#include <mach/hardware.h>
#include <mach/regs-sys-ns921x.h>

#include "clock.h"
#include "processor-ns921x.h"

#if defined(CONFIG_ARCH_NS9XXX)
static struct led_info ns9xxx_leds_pdata_info = {
        	.name			= "leds-pwm",
        	.default_trigger	= "ledtrig-dim",
        	.flags			= 0,
};

static struct pwm_channel_config ns9xxx_leds_pdata_config = {
	.duty_ns	= 0,
	.period_ns	= 0,
	
};

static struct pwm_led_platform_data ns9xxx_leds_pdata = {
	.bus_id		= "ns9xxx_pwmc.0",
	.chan		= 0,
	.led_info	= &ns9xxx_leds_pdata_info,
	.config		= &ns9xxx_leds_pdata_config,
};

static struct platform_device ns9xxx_device_ns9215_leds = {
	.name		= "leds-pwm",
	.id		= 0,
	.dev		= {
		.platform_data = &ns9xxx_leds_pdata,
	}
};

static struct ns9xxx_pwm_channel ns9215_pwm_channels[] = {
	[0] = {
		.timer		= 6,
		.gpio		= 5,
	},
	[1] = {
		.timer		= 7,
		.gpio		= 7,
	},
	[2] = {
		.timer		= 8,
		.gpio		= 8,
	},
	[3] = {
		.timer		= 9,
		.gpio		= 13,
	}
};


/* This structure will be initialized in the init function (see below) */
static struct ns9xxx_pwm_pdata ns9215_pwm_pdata;


static struct platform_device ns9xxx_device_ns9215_pwm = {
	.name		= "ns9xxx_pwmc",
	.id		= 0,
	.dev		= {
		.platform_data = &ns9215_pwm_pdata,
	}
};

void __init ns9xxx_add_device_ns9215_leds(void)
{
 	platform_device_register(&ns9xxx_device_ns9215_leds);

#if 1 /* Enabled PWM by the user configuration */
	ns9215_pwm_pdata.channels = ns9215_pwm_channels;
	ns9215_pwm_pdata.number_channels = ARRAY_SIZE(ns9215_pwm_channels);
	platform_device_register(&ns9xxx_device_ns9215_pwm);
#endif

}
#else
void __init ns9xxx_add_device_ns9215_leds(void) {}
#endif

#if defined(CONFIG_ADC_NS9215) || defined(CONFIG_ADC_NS9215_MODULE)
static struct ns921x_sysclk adc_clk = {
	.clk = {
		.name	= "adc-ns9215",
		.id	= -1,
		.owner	= THIS_MODULE,
	},
	.mask = SYS_CLOCK_ADC,
};

static struct resource adc_resources[] = {
	{
		.start	= 0x90039000,
		.end	= 0x90039027,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ns9xxx_device_ns9215_adc = {
	.name		= "adc-ns9215",
	.id		= -1,
	.resource	= adc_resources,
	.num_resources	= ARRAY_SIZE(adc_resources),
};

void __init ns9xxx_add_device_ns9215_adc(void)
{
	if (clk_register(&adc_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns9215_adc);
}
#else
void __init ns9xxx_add_device_ns9215_adc(void) {}
#endif

#if defined(CONFIG_RTC_DRV_NS9XXX) || defined(CONFIG_RTC_DRV_NS9XXX_MODULE)
static irqreturn_t ns9xxx_plat_rtc_irq(int irq, void *data)
{
	u32 sysrtcmc = __raw_readl(SYS_RTCMC);

	if (sysrtcmc & SYS_RTCMC_RIS) {
		REGSETIM(sysrtcmc, SYS_RTCMC, RIC, 1);
		__raw_writel(sysrtcmc, SYS_RTCMC);
		REGSETIM(sysrtcmc, SYS_RTCMC, RIC, 0);
		__raw_writel(sysrtcmc, SYS_RTCMC);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static inline int wait_for_clkrdy(void)
{
	u32 sysrtcmc;
	unsigned int timeout = 0x20;

wait:
	sysrtcmc = __raw_readl(SYS_RTCMC);
	if (!(sysrtcmc & SYS_RTCMC_RIS)) {
		if (unlikely(!--timeout))
			return -ETIMEDOUT;
		udelay(1);
		goto wait;
	}
	pr_debug("%s: SYS_RTCMC = 0x%x\n", __func__, sysrtcmc);

	REGSETIM(sysrtcmc, SYS_RTCMC, RIC, 1);
	__raw_writel(sysrtcmc, SYS_RTCMC);
	REGSETIM(sysrtcmc, SYS_RTCMC, RIC, 0);
	__raw_writel(sysrtcmc, SYS_RTCMC);

	return sysrtcmc & SYS_RTCMC_SS ? 0 : -EIO;
}

static int ns9215_rtc_endisable(struct clk *clk, int enable)
{
	u32 sysrtcmc = __raw_readl(SYS_RTCMC);

	pr_debug("%s: enable=%d, SYS_RTCMC=%08x\n", __func__, enable, sysrtcmc);

	if (enable && (sysrtcmc & (SYS_RTCMC_SS | SYS_RTCMC_MODE)) ==
			SYS_RTCMC_SS) {
		/* the clock was disabled but is still active */

		unsigned int timeout = 0x20;

		pr_debug("%s: wait until disabled clock becomes inactive\n",
		__func__);
wait:
		sysrtcmc = __raw_readl(SYS_RTCMC);
		if (sysrtcmc & SYS_RTCMC_SS) {
			if (unlikely(!--timeout))
				return -ETIMEDOUT;
			udelay(1);
			goto wait;
		}
	}

	if (enable && (sysrtcmc & SYS_RTCMC_SS)) {
		pr_debug("%s: RTC clock already on\n", __func__);
		return 0;
	}

	if (enable) {
		int ret;

		/* disable rtc irq because the irq is acked in wait_for_clkrdy
		 * and this might stuck the irq controller.  (It doesn't like an
		 * irq to become active and then inactive before it is serviced.
		 *
		 * Not acking doesn't do the trick.  Then the following might
		 * happen in an endless loop:
		 *   - clk_enable + clk_disable with irqs off.  This makes
		 *     IRQ_NS9215_RTC pending.
		 *   - reenable irqs
		 *   - service IRQ_NS9215_RTC
		 *   - platform handler runs and acks irq.
		 *   - driver handler enables clk, sees that there is nothing to
		 *     do, and disables clk again.  All this with IRQ_NS9215_RTC
		 *     being masked.
		 */
		disable_irq(IRQ_NS9215_RTC);

		__raw_writel(SYS_RTCMC_MODE_NORMAL, SYS_RTCMC);

		ret = wait_for_clkrdy();

		enable_irq(IRQ_NS9215_RTC);

		return ret;
	} else {
		__raw_writel(SYS_RTCMC_MODE_STANDBY, SYS_RTCMC);
		sysrtcmc = __raw_readl(SYS_RTCMC);
		pr_debug("%s: disable done, SYS_RTCMC=%08x\n",
				__func__, sysrtcmc);

		return 0;
	}
}

static struct ns921x_sysclk rtc_pm_clk = {
	.clk = {
		.name		= "rtc-ns9xxx-pm",
		.id		= -1,
		.owner		= THIS_MODULE,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_RTC,
 };

static struct clk rtc_clk = {
	.name		= "rtc-ns9xxx",
	.id		= 0,
	.owner		= THIS_MODULE,
	.endisable	= ns9215_rtc_endisable,
};

static struct resource rtc_resources[] = {
	{
		.start	= 0x90060000,
		.end	= 0x900600ff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9215_RTC,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device ns9xxx_device_ns9215_rtc = {
	.name		= "rtc-ns9xxx",
	.id		= 0,
	.resource	= rtc_resources,
	.num_resources	= ARRAY_SIZE(rtc_resources),
};

void __init ns9xxx_add_device_ns9215_rtc(void)
{
	if (clk_register(&rtc_pm_clk.clk))
		return;

	rtc_clk.parent = &rtc_pm_clk.clk;
	if (clk_register(&rtc_clk))
		return;

	if (request_irq(IRQ_NS9215_RTC, ns9xxx_plat_rtc_irq, IRQF_SHARED,
				"plat-rtc-ns9xxx", ns9xxx_plat_rtc_irq))
		return;

	platform_device_register(&ns9xxx_device_ns9215_rtc);
}
#else
void __init ns9xxx_add_device_ns9215_rtc(void) {}
#endif
