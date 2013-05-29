/*
 * arch/arm/mach-ns9xxx/processor-ns921x.c
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/cpufreq.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <asm/page.h>
#include <asm/mach/map.h>

#include <mach/regs-mem.h>
#include <mach/regs-sys-ns921x.h>

#include "clock.h"

#include "processor-ns921x.h"
#include "gpiolib-ns921x.h"

void ns921x_reset(char mode)
{
	unsigned long pll = __raw_readl(SYS_PLL);
	__raw_writel(pll, SYS_PLL);
}

#define DIVMASK(mask) ((mask) & (-(mask)))
#define MASKVALUE(value, mask) (((value) & (mask)) / DIVMASK(mask))

unsigned long ns921x_systemclock(void)
{
	unsigned long pll = __raw_readl(SYS_PLL);
	unsigned pll_nr = MASKVALUE(pll, SYS_PLL_NR);
	unsigned pll_nf = MASKVALUE(pll, SYS_PLL_NF);
	unsigned pll_od = MASKVALUE(pll, SYS_PLL_OD);

	return NS921X_REFCLOCK / (pll_nr + 1) * (pll_nf + 1) / (pll_od + 1);
}
static unsigned long ns921x_get_systemclock_rate(struct clk *clk)
{
	return ns921x_systemclock();
}

static inline unsigned long ahbclock(u32 clock)
{
	return ns921x_systemclock() >> (MASKVALUE(clock, SYS_CLOCK_CSC) + 2);
}

unsigned long ns921x_ahbclock(void)
{
	unsigned long clock = __raw_readl(SYS_CLOCK);
	return ahbclock(clock);
}

static unsigned long ns921x_get_ahbclock_rate(struct clk *clk)
{
	return ns921x_ahbclock();
}

static inline unsigned long cpuclock(u32 clock)
{
	return ahbclock(clock) << MASKVALUE(clock, SYS_CLOCK_CSSEL);
}

static unsigned long ns921x_cpuclock(void)
{
	unsigned long clock = __raw_readl(SYS_CLOCK);
	return cpuclock(clock);
}

static unsigned long ns921x_get_cpuclock_rate(struct clk *clk)
{
	return ns921x_cpuclock();
}

#if defined(CONFIG_CPU_FREQ)
static inline int ns921x_freq2clock(unsigned int freq /* [kHz] */,
		u32 cssel)
{
	/* valid range for CSC = [0...4] */
	int i, found = 0;

	BUG_ON(cssel && (cssel != SYS_CLOCK_CSSEL));
	pr_debug("%s: freq = %u\n", __func__, freq);

	for (i = 4; i >= 0; --i) {
		u32 clock = i << 29 | cssel;
		found = cpuclock(clock) >= 1000UL * freq;
		if (found)
			break;
	}
	if (!found) {
		pr_debug("%s(%u, 0x%x) failed\n", __func__, freq, cssel);
		BUG();
	}

	return i << 29 | cssel;
}

/* refresh_timer hold the value of the MEM_DMRT register for full speed
 * operation (SYS_CLOCK_SYS = 0b000)
 */
static u32 refresh_timer;

static int ns921x_cpufreq_verify(struct cpufreq_policy *policy)
{
	unsigned int freq;
	u32 cssel = 0;

	pr_debug("%s\n", __func__);
	/* This is an UP machine */
	if (policy->cpu)
		return -EINVAL;

	if (policy->policy == CPUFREQ_POLICY_PERFORMANCE)
		cssel = SYS_CLOCK_CSSEL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);

	/* XXX: make sure there is at least one frequency within the policy */
	freq = cpuclock(ns921x_freq2clock(policy->min, cssel)) / 1000;
	if (freq > policy->max)
		policy->max = freq;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);

	pr_debug("/%s\n", __func__);
	return 0;
}

static inline void update_refresh_timer(u32 clock)
{
	u32 rt = (refresh_timer >> (clock >> 29));

	if (unlikely(!rt)) {
		pr_warning("Cannot adapt the Dynamic Memory Refresh Timer: "
				"clock=0x%08x, rftimer=0x%08x",
				clock, refresh_timer);
		rt = 1;
	} else {
		if (unlikely(rt > 0x7ff))
			rt = 0x7ff;
		pr_debug("%s: MEM_DMRT <- 0x%08x\n", __func__, rt);
	}
	__raw_writel(rt, MEM_DMRT);
}

static int ns921x_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int target_freq,
		unsigned int relation)
{
	struct cpufreq_freqs freqs;
	unsigned long flags;
	u32 cssel = 0;
	u32 clock;

	pr_debug("%s\n", __func__);
	if (policy->policy == CPUFREQ_POLICY_PERFORMANCE)
		cssel = SYS_CLOCK_CSSEL;

	clock = ns921x_freq2clock(target_freq, cssel);

	pr_debug("%s: targetfreq = %u, relation = %u, new clock = %lx\n",
			__func__, target_freq, relation, (unsigned long)clock);

	if (relation == CPUFREQ_RELATION_H &&
			cpuclock(clock) / 1000 != target_freq) {
		clock += 1 << 29;
		BUG_ON((clock >> 29) > 4);
	}

	freqs.cpu = policy->cpu;
	freqs.old = policy->cur;

	freqs.new = cpuclock(clock) / 1000;

	pr_debug("%s: new = %u kHz, CLOCK=%lx\n", __func__, freqs.new,
			(unsigned long)clock);

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	local_irq_save(flags);

	if (freqs.old > freqs.new)
		update_refresh_timer(clock);

	clock |= __raw_readl(SYS_CLOCK) & ~(SYS_CLOCK_CSC | SYS_CLOCK_CSSEL);

	pr_debug("%s: SYS_CLOCK <- %x (%lu)\n", __func__, clock, cpuclock(clock));
	__raw_writel(clock, SYS_CLOCK);

	if (freqs.old < freqs.new)
		update_refresh_timer(clock);

	local_irq_restore(flags);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	pr_debug("/%s\n", __func__);

	return 0;
}

static unsigned int ns921x_cpufreq_get(unsigned int cpu)
{
	BUG_ON(cpu);

	return ns921x_cpuclock() / 1000;
}

static int ns921x_cpufreq_init(struct cpufreq_policy *policy)
{
	u32 clock = __raw_readl(SYS_CLOCK);
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->policy = CPUFREQ_POLICY_PERFORMANCE;
	policy->cpuinfo.min_freq = cpuclock(4 << 29 | SYS_CLOCK_CSSEL) / 1000;
	policy->cpuinfo.max_freq = cpuclock(SYS_CLOCK_CSSEL) / 1000;
	policy->cpuinfo.transition_latency = 1; /* XXX ??? */
	policy->cur = ns921x_cpufreq_get(0);
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;

	refresh_timer = __raw_readl(MEM_DMRT) << (clock >> 29);

	pr_info("ns921x CPU frequency change support initialized\n");
	pr_debug("%s: cur = %u, min = %d, max = %d\n", __func__, policy->cur,
			policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

	return 0;
}

static struct cpufreq_driver ns921x_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = ns921x_cpufreq_verify,
	.target = ns921x_cpufreq_target,
	.get = ns921x_cpufreq_get,
	.init = ns921x_cpufreq_init,
	.name = "ns921x",
};
#endif

static struct map_desc ns921x_io_desc[] __initdata = {
	{ /* Memory Controller */
		.virtual = io_p2v(0xa0700000),
		.pfn = __phys_to_pfn(0xa0700000),
		.length = 0x27c,
		.type = MT_DEVICE,
	},
	{ /* External DMA Module */
		.virtual = io_p2v(0xa0800000),
		.pfn = __phys_to_pfn(0xa0800000),
		.length = 0x20,
		.type = MT_DEVICE,
	},
	{ /* System Control Module */
		.virtual = io_p2v(0xa0900000),
		.pfn = __phys_to_pfn(0xa0900000),
		.length = 0x1000,
		.type = MT_DEVICE,
	},
	{ /* I/O Control Module */
		.virtual = io_p2v(0xa0902000),
		.pfn = __phys_to_pfn(0xa0902000),
		.length = 0x90,
		.type = MT_DEVICE,
	},
};

void __init ns921x_map_io(void)
{
	iotable_init(ns921x_io_desc, ARRAY_SIZE(ns921x_io_desc));
}

static inline void ns921x_setclock(u32 mask, u32 value)
{
	u32 oldclock = __raw_readl(SYS_CLOCK), clock;

	BUG_ON(value & ~mask);

	clock = (oldclock & ~mask) | value;

	__raw_writel(clock, SYS_CLOCK);
}

static inline void ns921x_setreset(u32 mask, u32 value)
{
	u32 oldreset = __raw_readl(SYS_RESET), reset;

	BUG_ON(value & ~mask);

	reset = (oldreset & ~mask) | value;

	if (reset != oldreset)
		__raw_writel(reset, SYS_RESET);
}

int ns921x_endisable_sysclock(struct clk *clk, int enable)
{
	struct ns921x_sysclk *sysclk =
		container_of(clk, struct ns921x_sysclk, clk);
	unsigned long flags;

	local_irq_save(flags);

	if (enable) {
		ns921x_setreset(sysclk->mask, sysclk->mask);
		ns921x_setclock(sysclk->mask, sysclk->mask);
	} else {
		ns921x_setclock(sysclk->mask, 0);
#if defined(CONFIG_RESET_DISABLED_MODULES)
		ns921x_setreset(sysclk->mask, 0);
#endif
	}

	local_irq_restore(flags);

	return 0;
}

void ns921x_init_machine(void)
{
	/* register system, ahb and cpu clocks */
	struct clk *clk = kzalloc(3 * sizeof(*clk), GFP_KERNEL);
	struct ns921x_sysclk *dmaclk;
	int ret, i;

	if (!clk) {
		pr_warning("%s: could not register system clocks\n", __func__);
		return;
	}

	clk[0].name = "systemclock";
	clk[0].get_rate = ns921x_get_systemclock_rate;

	clk[1].name = "ahbclock";
	clk[1].get_rate = ns921x_get_ahbclock_rate;

	clk[2].name = "cpuclock";
	clk[2].get_rate = ns921x_get_cpuclock_rate;

	for (i = 0; i < 3; ++i) {
		clk[i].id = -1;
		ret = clk_register(&clk[i]);
		if (ret)
			pr_warning("%s: could not register %s\n",
					__func__, clk[i].name);
	}

	dmaclk = kzalloc(sizeof(*dmaclk), GFP_KERNEL);
	if (!dmaclk) {
		pr_warning("%s: could not register dma clock\n", __func__);
		return;
	}

	dmaclk->clk.name = "dmaclock";
	dmaclk->clk.id = -1;
	dmaclk->clk.endisable = ns921x_endisable_sysclock;
	dmaclk->clk.owner = THIS_MODULE;
	dmaclk->mask = 1 << 14;

	ret = clk_register(&dmaclk->clk);
	if (ret)
		pr_warning("%s: could not register dma clock\n", __func__);

#if defined(CONFIG_CPU_FREQ)
	cpufreq_register_driver(&ns921x_cpufreq_driver);
#endif

	/* add system GPIOs */
	ns921x_gpio_init();
}
