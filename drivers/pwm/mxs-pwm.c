/*
 * drivers/pwm/mxs-pwm.c
 *
 * Copyright (C) 2012 by Digi International Inc.
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

#define LAST_COUNT	0xFFFFFFFF	/* 32 bit counter */
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <mach/regs-pwm.h>
#include <mach/mxs-pwm.h>
#include <mach/system.h>

#define SET	0x4
#define CLR	0x8
#define TOG	0xc

#define  PERIOD_PERIOD(p)	((p) & 0xffff)
#define  PERIOD_PERIOD_MAX	0x10000
#define  PERIOD_ACTIVE_HIGH	(3 << 16)
#define  PERIOD_ACTIVE_LOW	(2 << 16)
#define  PERIOD_INACTIVE_HIGH	(3 << 18)
#define  PERIOD_INACTIVE_LOW	(2 << 18)
#define  PERIOD_CDIV(div)	(((div) & 0x7) << 20)

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)

#define DRIVER_NAME "mxs-pwm"

struct mxs_pwm {
	struct pwm_device pwm;
	spinlock_t lock;
	struct clk *clk;
	void __iomem *base;

	struct mxs_pwm_pdata *pdata;
};

/* @XXX: Return NULL by failures */
static inline struct mxs_pwm_channel *mxs_pwm_to_channel(struct pwm_channel *p)
{
	struct mxs_pwm *mp;
	struct mxs_pwm_channel *retval;

	if (!p) {
		pr_err("%s: NULL pointer passed!\n", __func__);
		return NULL;
	}

	mp = container_of(p->pwm, struct mxs_pwm, pwm);
	if (!mp) {
		pr_err("%s: NO mp found!\n", __func__);
		return NULL;
	}

	if (!mp->pdata) {
		pr_err("%s: NO platform data found!\n", __func__);
		return NULL;
	}

	if (!mp->pdata->channels) {
		pr_err("%s: NO channels passed?\n", __func__);
		return NULL;
	}

	retval = mp->pdata->channels + p->chan;
	if (!retval)
		pr_err("%s: Invalid channel number passed?\n", __func__);

	return retval;
}

/* Checks if pwm is being used */
static inline int __mxs_pwm_is_on(struct pwm_channel *p)
{
	struct mxs_pwm *np = container_of(p->pwm, struct mxs_pwm, pwm);
	struct mxs_pwm_channel *pch;

	if (!np) {
		pr_err("Unable to fetch structure mxs_pwm.\n");
		return -EINVAL;
	}

	pch = mxs_pwm_to_channel(p);

	return (__raw_readl(REGS_PWM_BASE + HW_PWM_CTRL) & (BM_PWM_CTRL_PWM0_ENABLE << pch->channel)) ? 1 : 0;
}

static void __mxs_pwm_enable(int channel, int enable)
{
	void __iomem *reg;

	if (enable)
		reg = REGS_PWM_BASE + HW_PWM_CTRL_SET;
	else
		reg = REGS_PWM_BASE + HW_PWM_CTRL_CLR;

	__raw_writel((BM_PWM_CTRL_PWM0_ENABLE << channel),
		     reg);
}

/* Starts PWM output */
static inline void __mxs_pwm_start(struct pwm_channel *p)
{
	struct mxs_pwm_channel *pch;

	pch = mxs_pwm_to_channel(p);

	__mxs_pwm_enable(pch->channel, 1);

	p->is_started = true;
}

/* Stops PWM output */
static inline void __mxs_pwm_stop(struct pwm_channel *p)
{
	struct mxs_pwm_channel *pch;

	pch = mxs_pwm_to_channel(p);

	__mxs_pwm_enable(pch->channel, 0);

	p->is_started = false;
}

static inline int
__mxs_pwm_config_polarity(struct pwm_channel *p, struct pwm_channel_config *c)
{
	struct mxs_pwm *mp = container_of(p->pwm, struct mxs_pwm, pwm);
	struct mxs_pwm_channel *ch;
	unsigned int reg;

	ch = mxs_pwm_to_channel(p);

	if ((mp == NULL) || (ch == NULL)) {
		pr_err("Incorrect parameters in __mxs_pwm_config_polarity()\n");
		return -EINVAL;
	}

	reg = __raw_readl(mp->base + HW_PWM_PERIODn(ch->channel));
	reg &= ~(BM_PWM_PERIODn_ACTIVE_STATE | BM_PWM_PERIODn_INACTIVE_STATE);

	if (c->polarity) {
		reg |= PERIOD_ACTIVE_HIGH | PERIOD_INACTIVE_LOW;
	}
	else {
		reg |= PERIOD_ACTIVE_LOW | PERIOD_INACTIVE_HIGH;
	}

	__raw_writel(reg, mp->base + HW_PWM_PERIODn(ch->channel));

	/* Invert polarity for active low */
	p->active_low = c->polarity ? 0 : 1;

	return 0;
}

/* Configures duty_ticks and period_ticks */
static inline int
__mxs_pwm_config_ticks(struct pwm_channel *p, struct pwm_channel_config *c)
{
	struct mxs_pwm *mp = container_of(p->pwm, struct mxs_pwm, pwm);
	struct mxs_pwm_channel *ch;
	int div;
	int divisor[] = {1, 2, 4, 8, 16, 64, 256, 1024};
	unsigned int period_cycles, duty_cycles;
	unsigned long rate;
	unsigned long long cl;
	unsigned int reg;

	ch = mxs_pwm_to_channel(p);

	if ((mp == NULL) || (ch == NULL)) {
		pr_err("Incorrect parameters in __mxs_pwm_config_ticks()\n");
		return -EINVAL;
	}

	rate = clk_get_rate(mp->clk);
	for (div=0; div < ARRAY_SIZE(divisor); div++) {
		cl = rate / divisor[div];
		cl = cl * c->period_ticks;
		do_div(cl, 1000000000);
		if (cl < PERIOD_PERIOD_MAX)
			break;
	}
	if (div >= ARRAY_SIZE(divisor)) {
		pr_err("Max period exceeded\n");
		return -EINVAL;
	}

	period_cycles = cl;

	cl *= c->duty_ticks;
	do_div(cl, c->period_ticks);
	duty_cycles = cl;

	__raw_writel(duty_cycles << 16,
			mp->base + HW_PWM_ACTIVEn(ch->channel));

	p->duty_ticks = c->duty_ticks;

	reg = PERIOD_PERIOD(period_cycles) | PERIOD_CDIV(div);
	if (p->active_low) {
		reg |= PERIOD_ACTIVE_LOW | PERIOD_INACTIVE_HIGH;
	}
	else {
		reg |= PERIOD_ACTIVE_HIGH | PERIOD_INACTIVE_LOW;
	}

	__raw_writel(reg, mp->base + HW_PWM_PERIODn(ch->channel));

	p->period_ticks = c->period_ticks;

	return 0;
}

/* No sleeping configuration */
static int mxs_pwm_config_nosleep(struct pwm_channel *p, struct pwm_channel_config *c)
{
	int ret = 0;
	unsigned long flags;

	if (!test_bit(FLAG_REQUESTED, &p->flags)) {
		pr_err("Channel not requested\n");
		return -EPERM;
	}

	spin_lock_irqsave(&p->lock, flags);

	switch (c->config_mask) {

	case PWM_CONFIG_PERIOD_TICKS:
	case PWM_CONFIG_DUTY_TICKS:
		if (!(c->config_mask & PWM_CONFIG_DUTY_TICKS)) {
			c->config_mask |= PWM_CONFIG_DUTY_TICKS;
			c->duty_ticks = p->duty_ticks;
		}

		if (!(c->config_mask & PWM_CONFIG_PERIOD_TICKS)) {
			c->config_mask |= PWM_CONFIG_PERIOD_TICKS;
			c->period_ticks = p->period_ticks;
		}

		pr_debug("duty_ns: %lu period_ns: %lu polarity: %d\n", c->duty_ticks, c->period_ticks, p->active_low ? 0 : 1);

		__mxs_pwm_config_ticks(p, c);

		break;

	case PWM_CONFIG_POLARITY:
		__mxs_pwm_config_polarity(p, c);
		pr_debug("polarity: %d\n", c->polarity);
		break;

	case PWM_CONFIG_STOP:
		__mxs_pwm_stop(p);
		pr_debug("%s:%d stop\n", p->pwm->bus_id, p->chan);
		break;

	case PWM_CONFIG_START:
		__mxs_pwm_start(p);
		pr_debug("%s:%d start\n", p->pwm->bus_id, p->chan);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&p->lock, flags);
	return ret;
}

static int mxs_pwm_config(struct pwm_channel *p, struct pwm_channel_config *c)
{
	int was_on = 0;

	if (!test_bit(FLAG_REQUESTED, &p->flags)) {
		pr_err("Channel not requested\n");
		return -EPERM;
	}

	if (p->pwm->config_nosleep) {
		if (!p->pwm->config_nosleep(p, c))
			return 0;
	}

	might_sleep();

	was_on = __mxs_pwm_is_on(p);
	if (was_on < 0)
		return was_on;

	/* The processor writes the ACTIVE register's content only after the PERIOD reg. writing */
	/* So run a PERIOD reg. writing after an ACTIVE register writing */
	if (c->config_mask & (PWM_CONFIG_PERIOD_TICKS | PWM_CONFIG_DUTY_TICKS)) {
		if (!(c->config_mask & PWM_CONFIG_DUTY_TICKS)) {
			c->config_mask |= PWM_CONFIG_DUTY_TICKS;
			c->duty_ticks = p->duty_ticks;
		}

		if (!(c->config_mask & PWM_CONFIG_PERIOD_TICKS)) {
			c->config_mask |= PWM_CONFIG_PERIOD_TICKS;
			c->period_ticks = p->period_ticks;
		}
		__mxs_pwm_config_ticks(p, c);
	}

	if (c->config_mask & PWM_CONFIG_POLARITY) {
		__mxs_pwm_config_polarity(p, c);
	}

	if ((c->config_mask & PWM_CONFIG_START)
	    || (was_on && !(c->config_mask & PWM_CONFIG_STOP)))
		__mxs_pwm_start(p);

	pr_debug("%s:%d config_mask %x\n", p->pwm->bus_id, p->chan, c->config_mask);

	return 0;
}

/* PWM request function */
static int mxs_pwm_request(struct pwm_channel *p)
{
	struct mxs_pwm *mp = container_of(p->pwm, struct mxs_pwm, pwm);
	struct mxs_pwm_channel *chi;
	unsigned long flags;

	chi = mxs_pwm_to_channel(p);

	spin_lock_irqsave(&p->lock, flags);

	clk_enable(mp->clk);

	/* Enable PWM block */
	mxs_reset_block(REGS_PWM_BASE, 1);

	/* We need the period and duty values in ns, not in ticks, so set the tick_hz to 1GHz */
	p->tick_hz = 1000000000;

	__mxs_pwm_stop(p);
	spin_unlock_irqrestore(&p->lock, flags);

	return 0;
}

/* Frees resources taken by PWM */
static void mxs_pwm_free(struct pwm_channel *p)
{
	struct mxs_pwm *mp = container_of(p->pwm, struct mxs_pwm, pwm);
	struct mxs_pwm_channel *chi;
	chi = mxs_pwm_to_channel(p);

	pr_debug("%s\n", __func__);

	__mxs_pwm_stop(p);

	clk_disable(mp->clk);
}

static int __devinit mxs_pwmc_probe(struct platform_device *pdev)
{
	struct mxs_pwm *mp;
	struct mxs_pwm_pdata *pdata;
	struct resource *res;
	int ret;
	unsigned int i, reg;

	pr_debug(DRIVER_NAME ": Probing a new device (ID %i)\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err(DRIVER_NAME ": No platform data passed? Aborting.\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err(DRIVER_NAME ": No resource data passed? Aborting.\n");
		return -ENODEV;
	}

	/* Sanity check */
	if (pdata->number_channels > CONFIG_MXS_PWM_CHANNELS) {
		pr_err(DRIVER_NAME ": Max. PWM channel number: %d.\n", CONFIG_MXS_PWM_CHANNELS);
		return -EFAULT;
	}

	/* Read and mask for PWM channel present information */
	reg = (__raw_readl(IO_ADDRESS(res->start)) & 0x3FC00000) >> 22;

	/* Check whether channels exists or not */
	for (i = 0; i < pdata->number_channels; i++) {
		if ( (pdata->channels[i].channel >= CONFIG_MXS_PWM_CHANNELS) || !(1 << pdata->channels[i].channel) ) {
			pr_err(DRIVER_NAME ": PWM channel %d doesn't exists.\n", pdata->channels[i].channel);
		}
	}

	pr_debug(DRIVER_NAME ": %i PWM channels registered.\n", pdata->number_channels);

	mp = kzalloc(sizeof(*mp), GFP_KERNEL);
	if (!mp) {
		ret = -ENOMEM;
		goto err_mxs_pwm_alloc;
	}

	spin_lock_init(&mp->lock);
	platform_set_drvdata(pdev, mp);

	/* @XXX: Probably not the best place for this assignment */
	mp->pdata = pdata;

	mp->pwm.bus_id = dev_name(&pdev->dev);
	mp->pwm.nchan = pdata->number_channels;

	/* Copy the external platform data to our internal structure */
	//memcpy(&mp->pdata, pdata, sizeof(*pdata));

	mp->pwm.owner = THIS_MODULE;
	mp->pwm.request = mxs_pwm_request;
	mp->pwm.free = mxs_pwm_free;
	mp->pwm.config_nosleep = mxs_pwm_config_nosleep;
	mp->pwm.config = mxs_pwm_config;

	mp->base = IO_ADDRESS(res->start);

	mp->clk = clk_get(&pdev->dev, "pwm");
	if (IS_ERR(mp->clk)) {
		pr_info(DRIVER_NAME ": %s: clk_get error %ld\n", mp->pwm.bus_id, PTR_ERR(mp->clk));
		ret = -ENODEV;
		goto err_clk_get;
	}

	ret = pwm_register(&mp->pwm);
	if (ret) {
		pr_debug(DRIVER_NAME ": Failure registering pwm\n");
		goto err_pwm_register;
	}

	return 0;

err_pwm_register:
	clk_put(mp->clk);
err_clk_get:
	platform_set_drvdata(pdev, NULL);
	kfree(mp);
err_mxs_pwm_alloc:
	return ret;
}

static int __devexit mxs_pwmc_remove(struct platform_device *pdev)
{
	struct mxs_pwm *np;
	struct mxs_pwm_pdata *pdata;
	struct device *d = &pdev->dev;
	struct pwm_channel *p;
	int ret;

	pr_debug(DRIVER_NAME ": Removing device (ID %i)\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err(DRIVER_NAME ": No platform data passed? Aborting.\n");
		return -EINVAL;
	}

	np = platform_get_drvdata(pdev);
	p = np->pwm.channels;

	ret = pwm_unregister(&np->pwm);
	if (ret) {
		pr_debug(DRIVER_NAME ": Failure unregistering pwm\n");
	}

	/* Intentionally don't disable PWM block because the LEDs driver
	 * could be using it, and could stop working if we disable it. */

	clk_put(np->clk);
	platform_set_drvdata(pdev, NULL);

	kfree(np);
	module_put(d->driver->owner);

	return 0;
}

static struct platform_driver mxs_pwm_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = mxs_pwmc_probe,
	.remove = __devexit_p(mxs_pwmc_remove),
};

static int __init mxs_pwm_init(void)
{
	return platform_driver_register(&mxs_pwm_driver);
}

module_init(mxs_pwm_init);

static void mxs_pwm_exit(void)
{
	platform_driver_unregister(&mxs_pwm_driver);
}

module_exit(mxs_pwm_exit);

MODULE_AUTHOR("Robert Hodaszi <robert.hodaszi <at> digi.com>");
MODULE_DESCRIPTION("Driver for mxs PWM peripheral");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
