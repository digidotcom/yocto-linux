/*
 * drivers/pwm/ns9xxx-pwm.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

# define LAST_COUNT	0xFFFFFFFF	/* 32 bit counter */
// # define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <mach/gpio.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-io-ns921x.h>
#include <mach/ns9xxx-pwm.h>

struct ns9xxx_pwm {
	struct pwm_device pwm;
	spinlock_t lock;
	struct clk *clk;

	struct ns9xxx_pwm_pdata *pdata;
};

/* @XXX: Return NULL by failures */
static inline struct ns9xxx_pwm_channel *ns9xxx_pwm_to_channel(struct pwm_channel *p)
{
	struct ns9xxx_pwm *np;
	struct ns9xxx_pwm_channel *retval;


	if (!p) {
		pr_err("%s: NULL pointer passed!\n", __func__);
		return NULL;
	}

	np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	if (!np) {
		pr_err("%s: NO np found!\n", __func__);
		return NULL;
	}

	if (!np->pdata) {
		pr_err("%s: NO platform data found!\n", __func__);
		return NULL;
	}

	if (!np->pdata->channels)
		pr_err("NO channels passed?\n");

	retval = np->pdata->channels + p->chan;
	if (!retval)
		pr_err("%s: Invalid channel number passed?\n", __func__);

	return retval;
}

/* Checks if pwm is being used */
static inline int __ns9xxx_pwm_is_on(struct pwm_channel *p)
{
	struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	u32 timer_control;
//	struct ns9xxx_pwm_pdata *pdata;
	struct ns9xxx_pwm_channel *pch;

	if (!np) {
		pr_err("Unable to fetch structure ns9xxx_pwm.\n");
		return -EINVAL;
	}

	//pdata = np->pdata;
	//pch = pdata->channels + p->chan;
	pch = ns9xxx_pwm_to_channel(p);
	timer_control = readl(SYS_TC(pch->timer));

	return (timer_control & NS921X_TCR_TE) ? 1 : 0;
}

/* Starts PWM output */
static inline void __ns9xxx_pwm_start(struct pwm_channel *p)
{
	//struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	struct ns9xxx_pwm_channel *pch;

	pch = ns9xxx_pwm_to_channel(p);

	writel(NS921X_TCR_RELOADEN | NS921X_TCR_BITTIMER |
	       NS921X_TCR_TCLKSEL_AHB | NS921X_TCR_TE |
	       NS921X_TCR_TMODE2_PWM, SYS_TC(pch->timer));

	p->is_running = true;
}

/* Stops PWM output */
static inline void __ns9xxx_pwm_stop(struct pwm_channel *p)
{
	struct ns9xxx_pwm_channel *ch;

	ch = ns9xxx_pwm_to_channel(p);
	writel(0x0, SYS_TC(ch->timer));

	p->is_running = false;
}

/* Configures GPIO */
static inline int __ns9xxx_pwm_config_gpio(struct pwm_channel *p, int invert)
{
	int gpionr;
	struct ns9xxx_pwm_channel *ch;

	if (!p) {
		pr_err("Unable to fetch structure ns9xxx_pwm.\n");
		return -EINVAL;
	}

	ch = ns9xxx_pwm_to_channel(p);
	gpionr = ch->gpio;
	gpio_configure_ns921x_unlocked(gpionr, 0,
				       invert ? NS921X_GPIO_INVERT : NS921X_GPIO_DONT_INVERT,
				       NS921X_GPIO_FUNC_2, 0);

	return 0;
}

/* Toggles PWM output polarity */
static inline int
__ns9xxx_pwm_config_polarity(struct pwm_channel *p, struct pwm_channel_config *c)
{
	return __ns9xxx_pwm_config_gpio(p, c->polarity);
}

/* Configures duty_ticks */
static inline int
__ns9xxx_pwm_config_duty_ticks(struct pwm_channel *p, struct pwm_channel_config *c)
{
	struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	struct ns9xxx_pwm_channel *ch;
	u32 reload, low;

	ch = ns9xxx_pwm_to_channel(p);

	if ((np == NULL) || (ch == NULL)) {
		pr_err("Incorrect parameters in __ns9xxx_pwm_config_duty_ticks()\n");
		return -EINVAL;
	}

	/* While trying to configure duty without period, assume 50% duty cycle */
	if (p->period_ticks == 0)
		p->period_ticks = c->duty_ticks * 2;

	reload = LAST_COUNT - p->period_ticks;
	low = reload + c->duty_ticks;

	/*
	 * There are [0-9] Timers, but PWM extended runs on [6-9] SYS_THR
	 * and SYS_TLR is just available for [6-9], so the mapping
	 * would be: 0-> Timer 6, .., 3-> Timer 9
	 */
	writel(reload, SYS_TRELCCR(ch->timer));
	writel(reload, SYS_THR(ch->timer - 6));	/* high equals to reload */
	writel(low, SYS_TLR(ch->timer - 6));

	p->duty_ticks = c->duty_ticks;

	pr_debug
	    ("\np->period_ticks: %lu chan: %i p->duty_ticks: %lu cduty_ticks: %lu cperiod_ticks: %lu \n",
	     p->period_ticks, p->chan, p->duty_ticks, c->duty_ticks, c->period_ticks);
	pr_debug("period_ns: %lu chan: %i duty_ns: %lu config_mask: %i gpio: %i\n",
		 c->period_ns, p->chan, c->duty_ns, c->config_mask, ch->gpio);
	pr_debug("LAST_COUNT: %X reload: %X low: %X\n", LAST_COUNT, reload, low);

	return 0;
}


/*
 * Figure to understand PWM concepts
 *		-----------------
 *		|		|
 *		|		|
 * --------------		-----------------
 * ^		^		^		^
 * RELOAD	HIGH		LOW		LAST_COUNT
 *
 * Following driver assumes RELOAD = HIGH
 */

/* Configures period_ticks */
static inline int
__ns9xxx_pwm_config_period_ticks(struct pwm_channel *p, struct pwm_channel_config *c)
{
	struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	struct ns9xxx_pwm_channel *ch;
	u32 reload, low;

	ch = ns9xxx_pwm_to_channel(p);

	if ((np == NULL) || (ch == NULL)) {
		pr_err("Incorrect parameters in __ns9xxx_pwm_config_duty_ticks()\n");
		return -EINVAL;
	}

	reload = LAST_COUNT - c->period_ticks;

	/* While trying to configure period without duty, assume 50% duty cycle */
	if (p->duty_ticks == 0)
		p->duty_ticks = c->period_ticks / 2;

	low = reload + p->duty_ticks;

	/*
	 * There are [0-9] Timers, but PWM extended runs on [6-9]
	 * SYS_THR and SYS_TLR is just available for [6-9], so the mnpping
	 * would be: 0-> Timer 6, .., 3-> Timer 9
	 */
	writel(reload, SYS_TRELCCR(ch->timer));
	writel(reload, SYS_THR(ch->timer - 6));
	writel(low, SYS_TLR(ch->timer - 6));

	p->period_ticks = c->period_ticks;

	pr_debug
	    ("\np->period_ticks: %lu chan: %i p->duty_ticks: %lu cduty_ticks: %lu cperiod_ticks: %lu\n",
	     p->period_ticks, p->chan, p->duty_ticks, c->duty_ticks, c->period_ticks);
	pr_debug("period_ns: %lu chan: %i duty_ns: %lu config_mask: %i gpio: %i\n",
		 c->period_ns, p->chan, c->duty_ns, c->config_mask, ch->gpio);
	pr_debug("LAST_COUNT: %X reload: %X low: %X \n", LAST_COUNT, reload, low);

	return 0;
}

/* No sleeping configuration */
static int ns9xxx_pwm_config_nosleep(struct pwm_channel *p, struct pwm_channel_config *c)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);

	switch (c->config_mask) {

	case PWM_CONFIG_DUTY_TICKS:
		__ns9xxx_pwm_config_duty_ticks(p, c);
		pr_debug("duty_ns: %lu period_ns: %lu\n", c->duty_ns, c->period_ns);
		break;

	case PWM_CONFIG_PERIOD_TICKS:
		__ns9xxx_pwm_config_period_ticks(p, c);
		pr_debug("duty_ns: %lu period_ns: %lu\n", c->duty_ns, c->period_ns);
		break;

	case PWM_CONFIG_STOP:
		__ns9xxx_pwm_stop(p);
		pr_debug("%s:%d stop\n", p->pwm->bus_id, p->chan);
		break;

	case PWM_CONFIG_START:
		__ns9xxx_pwm_start(p);
		pr_debug("%s:%d start\n", p->pwm->bus_id, p->chan);
		break;

	case PWM_CONFIG_POLARITY:
		__ns9xxx_pwm_config_polarity(p, c);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&p->lock, flags);
	return ret;
}

/* PWM configuration */
static int ns9xxx_pwm_config(struct pwm_channel *p, struct pwm_channel_config *c)
{
	int was_on = 0;

	if (p->pwm->config_nosleep) {
		if (!p->pwm->config_nosleep(p, c))
			return 0;
	}

	might_sleep();

	was_on = __ns9xxx_pwm_is_on(p);
	if (was_on < 0)
		return was_on;

	if (c->config_mask & PWM_CONFIG_PERIOD_TICKS) {
		__ns9xxx_pwm_config_period_ticks(p, c);
		if (!(c->config_mask & PWM_CONFIG_DUTY_TICKS)) {
			struct pwm_channel_config d = {
				.config_mask = PWM_CONFIG_DUTY_TICKS,
				.duty_ticks = p->duty_ticks,
			};
			__ns9xxx_pwm_config_duty_ticks(p, &d);
		}
	}

	if (c->config_mask & PWM_CONFIG_DUTY_TICKS)
		__ns9xxx_pwm_config_duty_ticks(p, c);

	if (c->config_mask & PWM_CONFIG_POLARITY)
		__ns9xxx_pwm_config_polarity(p, c);

	if ((c->config_mask & PWM_CONFIG_START)
	    || (was_on && !(c->config_mask & PWM_CONFIG_STOP)))
		__ns9xxx_pwm_start(p);

	pr_debug("%s:%d config_mask %x\n", p->pwm->bus_id, p->chan, c->config_mask);

	return 0;
}

/* PWM request function */
static int ns9xxx_pwm_request(struct pwm_channel *p)
{
	struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	struct ns9xxx_pwm_channel *chi;
	unsigned long flags;
	char pwmgpio[20];
	int ret;

	chi = ns9xxx_pwm_to_channel(p);

	spin_lock_irqsave(&p->lock, flags);

	sprintf(pwmgpio, "%s.%d", p->pwm->bus_id, p->chan);
	ret = gpio_request(chi->gpio, pwmgpio);
	if (ret) {
		pr_err("Request of GPIO %i failure\n", chi->gpio);
		goto gpio_err;
	}

	ret = __ns9xxx_pwm_config_gpio(p, 0);
	if (ret) {
		pr_err("Configuring GPIO %i failure\n", chi->gpio);
		goto gpio_cfg;
	}

	/* TODO, request also the timer? use the enable bit of the timer to
	 * determine if the timer is alredy enabled == requested
	 *
	 * This can not be done, ATM, as Uboot sets TE active by default
	 */

	clk_enable(np->clk);
	p->tick_hz = clk_get_rate(np->clk);
	__ns9xxx_pwm_stop(p);
	spin_unlock_irqrestore(&p->lock, flags);

	pr_debug("%s: tick_hz = %lu\n", __func__, p->tick_hz);

	return 0;

gpio_cfg:
	gpio_configure_ns921x_unlocked(chi->gpio, 1, NS921X_GPIO_DONT_INVERT,
				       NS921X_GPIO_FUNC_3, 0);
	gpio_free(chi->gpio);
gpio_err:
	return ret;
}

/* Frees resources taken by PWM */
static void ns9xxx_pwm_free(struct pwm_channel *p)
{
	struct ns9xxx_pwm *np = container_of(p->pwm, struct ns9xxx_pwm, pwm);
	struct ns9xxx_pwm_channel *chi;
	chi = ns9xxx_pwm_to_channel(p);


	pr_debug("%s\n", __func__);

	clk_disable(np->clk);

	gpio_configure_ns921x_unlocked(chi->gpio, 1, NS921X_GPIO_DONT_INVERT,
				       NS921X_GPIO_FUNC_3, 0);
	gpio_free(chi->gpio);
}

static int __devinit ns9xxx_pwmc_probe(struct platform_device *pdev)
{
	struct ns9xxx_pwm *np;
	struct ns9xxx_pwm_pdata *pdata;
	int ret;

	pr_debug("Probing a new device (ID %i)\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err("No platform data passed? Aborting.\n");
		return -EINVAL;
	}

	/* Sanity check */
	if (pdata->number_channels > NS9XXX_PWM_CHANNEL_MAX) {
		pr_err("Invalid number '%i' of channels\n", pdata->number_channels);
		return -EINVAL;
	}

	pr_debug("%i PWM channels registered.\n", pdata->number_channels);

	np = kzalloc(sizeof(*np), GFP_KERNEL);
	if (!np) {
		ret = -ENOMEM;
		goto err_ns9xxx_pwm_alloc;
	}

	spin_lock_init(&np->lock);
	platform_set_drvdata(pdev, np);

	/* @XXX: Probably not the best place for this assignment */
	np->pdata = pdata;

	np->pwm.bus_id = dev_name(&pdev->dev);
	np->pwm.nchan = pdata->number_channels;

	/* Copy the external platform data to our internal structure */
	//memcpy(&np->pdata, pdata, sizeof(*pdata));

	np->pwm.owner = THIS_MODULE;
	np->pwm.request = ns9xxx_pwm_request;
	np->pwm.free = ns9xxx_pwm_free;
	np->pwm.config_nosleep = ns9xxx_pwm_config_nosleep;
	np->pwm.config = ns9xxx_pwm_config;

	ret = pwm_register(&np->pwm);
	if (ret) {
		pr_debug("Failure registering pwm\n");
		goto err_pwm_register;
	}

	np->clk = clk_get(&pdev->dev, "ahbclock");
	if (IS_ERR(np->clk)) {
		pr_info("%s: clk_get error %ld\n", np->pwm.bus_id, PTR_ERR(np->clk));
		ret = -ENODEV;
		goto err_clk_get;
	}

	return 0;

err_pwm_register:
	clk_put(np->clk);
err_clk_get:
	platform_set_drvdata(pdev, NULL);
	kfree(np);
err_ns9xxx_pwm_alloc:
	return ret;
}

static int __devexit ns9xxx_pwmc_remove(struct platform_device *pdev)
{
	struct ns9xxx_pwm *np;
	struct ns9xxx_pwm_pdata *pdata;
	struct device *d = &pdev->dev;
	struct ns9xxx_pwm_channel *chi;
	struct pwm_channel *p;
	int ret;
	u32 i;

	pr_debug("Removing device (ID %i)\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err("No platform data passed? Aborting.\n");
		return -EINVAL;
	}

	np = platform_get_drvdata(pdev);
	p = np->pwm.channels;

	ret = pwm_unregister(&np->pwm);
	if (ret) {
		pr_debug("Failure unregistering pwm\n");
	}

	/*
	 * @TODO: We might want to reset Read and Capture Register, but we can not write to it.
	 */
	for (i=0; i < pdata->number_channels; i++) {

		chi = ns9xxx_pwm_to_channel(p+i);

		/* Resets Timer Control Register */
		writel(0x00, SYS_TC(chi->timer));

		/* Resets High, Low, Reload Registers*/
		writel(0x00, SYS_TRELCCR(chi->timer));
		writel(0x00, SYS_THR(chi->timer - 6));	/* high equals to reload */
		writel(0x00, SYS_TLR(chi->timer - 6));

		/* Sets GPIO as inputs */
		gpio_configure_ns921x_unlocked(chi->gpio, 1, NS921X_GPIO_DONT_INVERT,
					       NS921X_GPIO_FUNC_3, 0);

		/* Frees GPIO */
		gpio_free(chi->gpio);
	}

	clk_put(np->clk);
	platform_set_drvdata(pdev, NULL);

	kfree(np);
	module_put(d->driver->owner);

	return 0;
}

static struct platform_driver ns9xxx_pwm_driver = {
	.driver = {
		   .name = "ns9xxx_pwmc",
		   .owner = THIS_MODULE,
		   },
	.probe = ns9xxx_pwmc_probe,
	.remove = __devexit_p(ns9xxx_pwmc_remove),
};

static int __init ns9xxx_pwm_init(void)
{
	return platform_driver_register(&ns9xxx_pwm_driver);
}

module_init(ns9xxx_pwm_init);

static void ns9xxx_pwm_exit(void)
{
	platform_driver_unregister(&ns9xxx_pwm_driver);
}

module_exit(ns9xxx_pwm_exit);

MODULE_AUTHOR("Hector Oron <Hector.Oron <at> digi.com>");
MODULE_DESCRIPTION("Driver for ns9xxx PWMC peripheral");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ns9xxx_pwmc");
