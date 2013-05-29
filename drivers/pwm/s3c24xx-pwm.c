/*
 * drivers/pwm/s3c24xx-pwm.c
 *
 * Copyright (C) 2010 by Digi International Inc.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <plat/pwm.h>
#include <plat/regs-timer.h>

#define MAX_TIMER_COUNT	0xffff

struct s3c24xx_pwm {
	struct pwm_device pwm;
	spinlock_t lock;

	struct s3c24xx_pwm_pdata *pdata;
};

static struct clk *clk_scaler[2];
/* Prescaler of each timer */
static int timers_prescaler[5] = {0, 0, 1, 1, 1};

#define pwm_tcon_start(pch) (1 << (pch->tcon_base + 0))
#define pwm_tcon_invert(pch) (1 << (pch->tcon_base + 2))
#define pwm_tcon_autoreload(pch) (1 << (pch->tcon_base + 3))
#define pwm_tcon_manulupdate(pch) (1 << (pch->tcon_base + 1))

static inline int pwm_is_tdiv(struct s3c24xx_pwm_channel *pch)
{
	return clk_get_parent(pch->clk) == pch->clk_div;
}

/* @XXX: Return NULL by failures */
static inline struct s3c24xx_pwm_channel *s3c24xx_pwm_to_channel(struct pwm_channel *p)
{
	struct s3c24xx_pwm *np;
	struct s3c24xx_pwm_channel *retval;


	if (!p) {
		pr_err("%s: NULL pointer passed!\n", __func__);
		return NULL;
	}

	np = container_of(p->pwm, struct s3c24xx_pwm, pwm);
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

/* Starts PWM output */
static inline void __s3c24xx_pwm_start(struct pwm_channel *p)
{
	unsigned long flags;
	unsigned long tcon;
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	local_irq_save(flags);

	tcon = __raw_readl(S3C2410_TCON);
	tcon |= pwm_tcon_start(pch);
	__raw_writel(tcon, S3C2410_TCON);

	local_irq_restore(flags);

	pch->running = 1;
	p->is_running = true;
}

/* Stops PWM output */
static inline void __s3c24xx_pwm_stop(struct pwm_channel *p)
{
	unsigned long flags;
	unsigned long tcon;
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	local_irq_save(flags);

	tcon = __raw_readl(S3C2410_TCON);
	tcon &= ~pwm_tcon_start(pch);
	__raw_writel(tcon, S3C2410_TCON);

	local_irq_restore(flags);

	pch->running = 0;
	p->is_running = false;
}

/* Toggles PWM output polarity */
static inline void
__s3c24xx_pwm_config_polarity(struct pwm_channel *p, int polarity)
{
	unsigned long flags;
	unsigned long tcon;
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	local_irq_save(flags);

	tcon = __raw_readl(S3C2410_TCON);
	if (polarity) {
		/* Don't use inverter */
		tcon &= ~pwm_tcon_invert(pch);
	}
	else {
		/* Use inverter */
		tcon |= pwm_tcon_invert(pch);
	}
	__raw_writel(tcon, S3C2410_TCON);

	local_irq_restore(flags);
}

static unsigned long pwm_calc_tin(struct s3c24xx_pwm_channel *pch, unsigned long freq)
{
	unsigned long tin_parent_rate;
	unsigned int div;

	tin_parent_rate = clk_get_rate(clk_get_parent(pch->clk_div));

	for (div = 2; div <= 16; div *= 2) {
		if ((tin_parent_rate / (div << 16)) < freq)
			return tin_parent_rate / div;
	}

	return tin_parent_rate / 16;
}

#define NS_IN_HZ (1000000000UL)

/*
 * Figure to understand PWM concepts
 *        -------------------------                 -----
 *        |                        |                |
 *        |                        |                |
 * --------                        -----------------
 *
 *        ^                        ^                ^
 *        |-- DUTY (polarity=1) ---|                |
 *        |                                         |
 *        |----------- PERIOD-----------------------|
 *
 */

/* Configures duty_ticks */
static inline int
__s3c24xx_pwm_config_duty_ticks(struct pwm_channel *p, unsigned long duty_ticks)
{
	unsigned long tcon;
	unsigned long tcmp;
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);
	unsigned long flags;

	if (duty_ticks > p->period_ticks)
		return -ERANGE;

	p->duty_ticks = duty_ticks;

	/* Update the PWM register block. */
	local_irq_save(flags);

	tcmp = duty_ticks;
	__raw_writel(tcmp, S3C2410_TCMPB(pch->timer));

	tcon = __raw_readl(S3C2410_TCON);
	tcon |= pwm_tcon_manulupdate(pch);
	tcon |= pwm_tcon_autoreload(pch);
	__raw_writel(tcon, S3C2410_TCON);

	tcon &= ~pwm_tcon_manulupdate(pch);
	__raw_writel(tcon, S3C2410_TCON);

	local_irq_restore(flags);
 	return 0;
}

/* Configures period_ticks */
static inline int
__s3c24xx_pwm_config_period_ticks(struct pwm_channel *p, unsigned long period_ticks)
{
	unsigned long period_ns;
	unsigned long period;
	unsigned long tin_rate;
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long flags;
	unsigned long pclk_rate;
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	period_ns = pwm_ticks_to_ns(p, period_ticks);
	if (!period_ns)
		return -ERANGE;
	period = NS_IN_HZ / period_ns;

	/* Adjust prescaler (only applicable to prescaler 0 -timers 0 and 1-) */
	if (0 == timers_prescaler[pch->timer]) {
		pclk_rate = clk_get_rate(clk_get_parent(clk_scaler[0]));
#ifdef CONFIG_S3C2443_PWM_PRESCALER_0
		/* Set prescaler to fixed prescaler set by user */
		clk_set_rate(clk_get_parent(pch->clk_div), pclk_rate/(CONFIG_S3C2443_PWM_PRESCALER_0+1));
#endif
	}

	if (pwm_is_tdiv(pch)) {
		tin_rate = pwm_calc_tin(pch, period);
		clk_set_rate(pch->clk_div, tin_rate);
	} else {
		tin_rate = clk_get_rate(pch->clk);
	}
	/* After changing the period, the divisor might
	 * have changed, and with it, the tick, which must
	 * be now recalculated */
	p->tick_hz = clk_get_rate(pch->clk);

	/* Recalculate ticks with the new tick_hz, just in case */
	tcnt = pwm_ns_to_ticks(p, period_ns);
	if (tcnt > MAX_TIMER_COUNT)
		tcnt = MAX_TIMER_COUNT;

	/* Save new ticks */
	p->period_ticks = tcnt;

	/* Update the PWM register block. */
	local_irq_save(flags);

	__raw_writel(tcnt, S3C2410_TCNTB(pch->timer));

	tcon = __raw_readl(S3C2410_TCON);
	tcon |= pwm_tcon_manulupdate(pch);
	tcon |= pwm_tcon_autoreload(pch);
	__raw_writel(tcon, S3C2410_TCON);

	tcon &= ~pwm_tcon_manulupdate(pch);
	__raw_writel(tcon, S3C2410_TCON);

	local_irq_restore(flags);

	return 0;
}

/* No sleeping configuration */
static int s3c24xx_pwm_config_nosleep(struct pwm_channel *p, struct pwm_channel_config *c)
{
	int ret = 0;
	unsigned long flags;
	unsigned long duty_ns;

	spin_lock_irqsave(&p->lock, flags);

	switch (c->config_mask) {

	case PWM_CONFIG_DUTY_TICKS:
		ret = __s3c24xx_pwm_config_duty_ticks(p, c->duty_ticks);
		break;

	case PWM_CONFIG_PERIOD_TICKS:
		/* Save original duty cycle */
		duty_ns =  pwm_ticks_to_ns(p, p->duty_ticks);
		ret = __s3c24xx_pwm_config_period_ticks(p, c->period_ticks);
		if (!ret) {
			/* If period has changed, reconfigure duty cycle */
			__s3c24xx_pwm_config_duty_ticks(p, pwm_ns_to_ticks(p, duty_ns));
			/* Do not bother to return error code if this fails or
			 * the set period function will be called again later */
		}
		break;

	case PWM_CONFIG_STOP:
		__s3c24xx_pwm_stop(p);
		pr_debug("%s:%d stop\n", p->pwm->bus_id, p->chan);
		break;

	case PWM_CONFIG_START:
		__s3c24xx_pwm_start(p);
		pr_debug("%s:%d start\n", p->pwm->bus_id, p->chan);
		break;

	case PWM_CONFIG_POLARITY:
		__s3c24xx_pwm_config_polarity(p, c->polarity);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&p->lock, flags);

	return ret;
}

/* PWM configuration */
static int s3c24xx_pwm_config(struct pwm_channel *p, struct pwm_channel_config *c)
{
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	/* We currently avoid using 64bit arithmetic by using the
	 * fact that anything faster than 1Hz is easily representable
	 * by 32bits. */

	if (p->pwm->config_nosleep) {
		if (!p->pwm->config_nosleep(p, c))
			return 0;
	}

	might_sleep();

	if (c->config_mask & PWM_CONFIG_PERIOD_TICKS) {
		__s3c24xx_pwm_config_period_ticks(p, c->period_ticks);
		if (!(c->config_mask & PWM_CONFIG_DUTY_TICKS)) {
			__s3c24xx_pwm_config_duty_ticks(p, p->duty_ticks);
		}
	}

	if (c->config_mask & PWM_CONFIG_DUTY_TICKS)
		__s3c24xx_pwm_config_duty_ticks(p, c->duty_ticks);

	if (c->config_mask & PWM_CONFIG_POLARITY)
		__s3c24xx_pwm_config_polarity(p, c->polarity);

	if ((c->config_mask & PWM_CONFIG_START)
	    || (pch->running && !(c->config_mask & PWM_CONFIG_STOP)))
		__s3c24xx_pwm_start(p);

	pr_debug("%s:%d config_mask %x\n", p->pwm->bus_id, p->chan, c->config_mask);

	return 0;
}

/* PWM request function */
static int s3c24xx_pwm_request(struct pwm_channel *p)
{
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);
	unsigned long flags;
	char pwmgpio[20];
	int ret;

	spin_lock_irqsave(&p->lock, flags);

	/* Determine if the timer is alredy enabled == requested */
	if (pch->use_count == 0) {
		pch->use_count = 1;
	} else {
		ret = -EBUSY;
		goto gpio_err;
	}

	/* Request the GPIO */
	sprintf(pwmgpio, "%s.%d", p->pwm->bus_id, p->chan);
	ret = gpio_request(pch->gpio, pwmgpio);
	if (ret) {
		pr_err("Request of GPIO %i failure\n", pch->gpio);
		goto gpio_err;
	}
	/* Configures GPIO for TOUTn function */
	s3c2410_gpio_cfgpin(pch->gpio, S3C2410_GPIO_SFN2);

	p->tick_hz = clk_get_rate(pch->clk);
	__s3c24xx_pwm_stop(p);
	spin_unlock_irqrestore(&p->lock, flags);

	pr_debug("%s: tick_hz = %lu\n", __func__, p->tick_hz);

	return 0;

gpio_err:
	return ret;
}

/* Frees resources taken by PWM */
static void s3c24xx_pwm_free(struct pwm_channel *p)
{
	struct s3c24xx_pwm_channel *pch = s3c24xx_pwm_to_channel(p);

	pr_debug("%s\n", __func__);

	if (pch->use_count) {
		pch->use_count--;
	} else {
		printk(KERN_ERR "PWM channel %d already freed\n", pch->timer);
	}

	s3c2410_gpio_cfgpin(pch->gpio, S3C2410_GPIO_INPUT);
	gpio_free(pch->gpio);
}

static int __devinit s3c24xx_pwmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s3c24xx_pwm *np = NULL;
	struct s3c24xx_pwm_pdata *pdata;
	struct s3c24xx_pwm_channel *pch;
	int ret;
	int i;

	pr_debug("Probing a new device (ID %i)\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pr_err("No platform data passed? Aborting.\n");
		return -EINVAL;
	}

	/* Sanity check */
	if (pdata->number_channels > S3C24XX_PWM_CHANNEL_MAX) {
		pr_err("Invalid number '%i' of channels\n", pdata->number_channels);
		return -EINVAL;
	}

	/* Initialize channels */
	for (i = 0; i < pdata->number_channels; i++) {
		pch = &pdata->channels[i];
		/* calculate base of control bits in TCON */
		pch->tcon_base = (i == 0) ? 0 : (i * 4) + 4;
		pch->use_count = 0;
		pch->running = 0;

		/* Hack pdev->id to get the proper clock producer for
		 * each channel */
		pdev->id = i;

		pch->clk = clk_get(dev, "pwm-tin");
		if (IS_ERR(pch->clk)) {
			dev_err(dev, "failed to get pwm tin clk\n");
			ret = PTR_ERR(pch->clk);
			goto err_clk_tin;
		}

		pch->clk_div = clk_get(dev, "pwm-tdiv");
		if (IS_ERR(pch->clk_div)) {
			dev_err(dev, "failed to get pwm tdiv clk\n");
			ret = PTR_ERR(pch->clk_div);
			goto err_clk_tdiv;
		}
	}
	/* Restore previously hacked pdev->id */
	pdev->id = -1;

	pr_debug("%i PWM channels registered.\n", pdata->number_channels);

	np = kzalloc(sizeof(struct s3c24xx_pwm), GFP_KERNEL);
	if (!np) {
		ret = -ENOMEM;
		goto err_clk_tdiv;
	}
	spin_lock_init(&np->lock);

	np->pwm.bus_id = dev_name(&pdev->dev);
	np->pwm.nchan = pdata->number_channels;

	/* Copy the external platform data to our internal structure */
	//memcpy(&np->pdata, pdata, sizeof(*pdata));

	np->pwm.owner = THIS_MODULE;
	np->pwm.request = s3c24xx_pwm_request;
	np->pwm.free = s3c24xx_pwm_free;
	np->pwm.config_nosleep = s3c24xx_pwm_config_nosleep;
	np->pwm.config = s3c24xx_pwm_config;

	ret = pwm_register(&np->pwm);
	if (ret) {
		pr_debug("Failure registering pwm\n");
		goto err_clk_tdiv;
	}

	platform_set_drvdata(pdev, np);
	/* @XXX: Probably not the best place for this assignment */
	np->pdata = pdata;
	return 0;

 err_clk_tdiv:
	pch = &pdata->channels[i];
	clk_put(pch->clk);

 err_clk_tin:
	while (i > 0) {
		i--;
		pch = &pdata->channels[i];
		clk_put(pch->clk_div);
		clk_put(pch->clk);
	}

	kfree(np);

	return ret;
}

static int __devexit s3c24xx_pwmc_remove(struct platform_device *pdev)
{
	struct s3c24xx_pwm *np;
	struct s3c24xx_pwm_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct s3c24xx_pwm_channel *pch;
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

	for(i = 0; i < pdata->number_channels; i++) {
		pch = &pdata->channels[i];
		if (pch->use_count) {
			/* Stop running channels */
			if (pch->running) {
				__s3c24xx_pwm_stop(p + i);
			}
			/* Free requested channels */
			s3c24xx_pwm_free(p + i);
		}
		/* Free clocks */
		clk_put(pch->clk_div);
		clk_put(pch->clk);
	}

	ret = pwm_unregister(&np->pwm);
	if (ret) {
		pr_debug("Failure unregistering pwm\n");
	}

	platform_set_drvdata(pdev, NULL);

	kfree(np);
	module_put(dev->driver->owner);

	return 0;
}

static struct platform_driver s3c24xx_pwm_driver = {
	.driver = {
		   .name = "s3c24xx-pwm",
		   .owner = THIS_MODULE,
		   },
	.probe = s3c24xx_pwmc_probe,
	.remove = __devexit_p(s3c24xx_pwmc_remove),
};

static char banner[] __initdata =
        KERN_INFO "S3C24XX PWM driver\n";

static int __init s3c24xx_pwm_init(void)
{
	int ret;

	clk_scaler[0] = clk_get(NULL, "pwm-scaler0");
	clk_scaler[1] = clk_get(NULL, "pwm-scaler1");

	if (IS_ERR(clk_scaler[0]) || IS_ERR(clk_scaler[1])) {
		printk(KERN_ERR "%s: failed to get scaler clocks\n", __func__);
		return -EINVAL;
	}

	printk(banner);
	ret = platform_driver_register(&s3c24xx_pwm_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add pwm driver\n", __func__);

	return ret;
}

module_init(s3c24xx_pwm_init);

static void s3c24xx_pwm_exit(void)
{
	platform_driver_unregister(&s3c24xx_pwm_driver);
}

module_exit(s3c24xx_pwm_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Driver for s3c24xx PWM peripheral");
MODULE_LICENSE("GPL");
