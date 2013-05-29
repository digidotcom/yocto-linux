/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * $Id: s3c-ts.c,v 1.7 2007/02/01 05:08:41 yongkal Exp $
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 * 2008-10-20: Luis Galdos <luis.galdos@digi.com>
 *      - Modifications for using a platform device
 *      - Close and open function implemented
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/signal.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/irqs.h>

#include <mach/map.h>
#include <plat/ts.h>
#include <plat/regs-adc.h>


#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] s3c24xx-ts: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "s3c24xx-ts: " fmt, ## args)

#if 0
#define S3C24XX_TS_DEBUG
#endif

#ifdef S3C24XX_TS_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "s3c24xx-ts: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif



/* @TODO: Remove the below macro */
#define S3C24XX_SAMPLE_NUM_MAX			(20)


/* For ts->dev.id.version */
#define S3C_TSVERSION				0x0101

#define WAIT4INT(x)				(((x)<<8) | \
						S3C2410_ADCTSC_YM_SEN | \
						S3C2410_ADCTSC_YP_SEN | \
						S3C2410_ADCTSC_XP_SEN | \
						S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST					(S3C2410_ADCTSC_YM_SEN | \
						S3C2410_ADCTSC_YP_SEN | \
						S3C2410_ADCTSC_XP_SEN | \
						S3C2410_ADCTSC_AUTO_PST | \
						S3C2410_ADCTSC_XY_PST(0))

/*
 * Definitions & global arrays.
 */
static char *s3c_ts_name = "s3c TouchScreen";


/*
 * Per-touchscreen data.
 */
struct s3c_ts {
	ulong xp;
	ulong yp;
	char phys[32];
	struct input_dev *indev;
	struct clk *clk;
	void __iomem *base;
	struct resource *mem;
	struct timer_list timer;
	int timer_enabled;
	int irq_adc;
	int irq_tc;
	atomic_t closed;
	unsigned int adccon;
	unsigned int adctsc;
	unsigned int adcdly;

	ulong xps[S3C24XX_SAMPLE_NUM_MAX];
	ulong yps[S3C24XX_SAMPLE_NUM_MAX];
	int sample;
	struct s3c_ts_mach_info *info;
	int skip_this_sample;
};


#define MAX_DIFF_BETWEEN_SAMPLES        200
static ulong filter_raw_values(struct s3c_ts *ts, ulong *samples)
{
        ulong diff12, diff23, diff31;
        ulong vals[3];
        int cnt;

	/* First reset the internal flag */
	ts->skip_this_sample = 0;

        for (cnt = 0; cnt < 3; cnt++)
                vals[cnt] = samples[cnt];

        diff12 = (vals[0] > vals[1]) ? vals[0] - vals[1] : vals[1] - vals[0];
        if (diff12 > MAX_DIFF_BETWEEN_SAMPLES) {
                ts->skip_this_sample = 1;
                return 0;
        }

        diff23 = (vals[1] > vals[2]) ? vals[1] - vals[2] : vals[2] - vals[1];
        if (diff23 > MAX_DIFF_BETWEEN_SAMPLES) {
                ts->skip_this_sample = 1;
                return 0;
        }

        diff31 = (vals[2] > vals[0]) ? vals[2] - vals[0] : vals[0] - vals[2];
        if (diff31 > MAX_DIFF_BETWEEN_SAMPLES) {
                ts->skip_this_sample = 1;
                return 0;
        }

        if (diff12 < diff23 && diff12 < diff31)
                return (vals[0] + vals[1]) / 2;
        if (diff23 < diff12 && diff23 < diff31)
                return (vals[1] + vals[2]) / 2;

        return (vals[0] + vals[2]) / 2;
}

static inline int s3c_ts_calc_sample(struct s3c_ts *ts)
{
	int offset;
	struct s3c_ts_mach_info *info;
	unsigned long px, py, tx, ty;
	int cnt;

	info = ts->info;
	px = py = 0;
	for (cnt = 0, offset = 0; offset < info->probes / 3; offset++) {

		tx = filter_raw_values(ts, &ts->xps[offset * 3]);
		if (ts->skip_this_sample)
			continue;

		ty = filter_raw_values(ts, &ts->yps[offset * 3]);
		if (ts->skip_this_sample)
			continue;

		cnt++;
		px += tx;
		py += ty;
	}

	if (cnt) {
		ts->xp = px / cnt;
		ts->yp = py / cnt;
	}

	return cnt;
}

/* static struct s3c_ts *ts; */

static void touch_timer_fire(unsigned long data)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	struct s3c_ts *ts;
	unsigned int adccon;

	ts = (struct s3c_ts *)data;

	data0 = readl(ts->base + S3C2410_ADCDAT0);
	data1 = readl(ts->base + S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) &&
		(!(data1 & S3C2410_ADCDAT1_UPDOWN));

	if (updown) {
		if (ts->timer_enabled) {
			struct s3c_ts_mach_info *info;
			info = ts->info;

			ts->xps[ts->sample] = ts->xp;
			ts->yps[ts->sample] = ts->yp;
			ts->sample += 1;

			if (ts->sample == info->probes) {
				if (s3c_ts_calc_sample(ts)) {
					printk_debug("XP %lu | YP %lu\n",
						     ts->xp, ts->yp);
					input_report_abs(ts->indev, ABS_X, ts->xp);
					input_report_abs(ts->indev, ABS_Y, ts->yp);

					input_report_key(ts->indev, BTN_TOUCH, 1);
					input_report_abs(ts->indev, ABS_PRESSURE, 1);
					input_sync(ts->indev);
				}

				/* Reset the number of samples */
				ts->sample = 0;
			}
		}

		ts->xp = 0;
		ts->yp = 0;
		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST,
		       ts->base + S3C2410_ADCTSC);
		adccon = readl(ts->base + S3C2410_ADCCON);
		writel(adccon | S3C2410_ADCCON_ENABLE_START, ts->base + S3C2410_ADCCON);
	} else {
		input_report_key(ts->indev, BTN_TOUCH, 0);
		input_report_abs(ts->indev, ABS_PRESSURE, 0);
		input_sync(ts->indev);
		writel(WAIT4INT(0), ts->base + S3C2410_ADCTSC);
	}

	ts->timer_enabled = 0;
}


static irqreturn_t stylus_updown(int irq, void *dev_id)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	struct s3c_ts *ts;

	ts = (struct s3c_ts *)dev_id;

	disable_irq(ts->irq_adc);
	data0 = readl(ts->base + S3C2410_ADCDAT0);
	data1 = readl(ts->base + S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) &&
		(!(data1 & S3C2410_ADCDAT1_UPDOWN));

	printk_debug("   %c\n",	updown ? 'D' : 'U');

	/* TODO we should never get an interrupt with updown set while
	 * the timer is running, but maybe we ought to verify that the
	 * timer isn't running anyways. */

       if (updown) {
	       ts->sample = 0;
	       touch_timer_fire((unsigned long)ts);
       }

#ifdef CONFIG_ARCH_S3C6400
        __raw_writel(0x0, ts->base + S3C_ADCCLRWK);
        __raw_writel(0x0, ts->base + S3C_ADCCLRINT);
#endif

	if (!atomic_read(&ts->closed))
		enable_irq(ts->irq_adc);

	return IRQ_HANDLED;
}

static irqreturn_t stylus_action(int irq, void *dev_id)
{
	unsigned long data0;
	unsigned long data1;
	struct s3c_ts *ts;
	unsigned int adccon;
	struct s3c_ts_mach_info *info;

	ts = (struct s3c_ts *)dev_id;
	info = ts->info;

	disable_irq(ts->irq_tc);

	data0 = readl(ts->base + S3C2410_ADCDAT0);
	data1 = readl(ts->base + S3C2410_ADCDAT1);
	adccon = readl(ts->base + S3C2410_ADCCON);

	ts->xp = (data0 & S3C2410_ADCDAT0_XPDATA_MASK);
	ts->yp = (data1 & S3C2410_ADCDAT1_YPDATA_MASK);

	ts->timer_enabled = 1;

	/* Use the correct values for updating the timer */
	mod_timer(&ts->timer, jiffies + msecs_to_jiffies(info->trigger_ms));
	writel(WAIT4INT(1), ts->base + S3C2410_ADCTSC);

#ifdef CONFIG_ARCH_S3C6400
        __raw_writel(0x0, ts->base + S3C_ADCCLRWK);
        __raw_writel(0x0, ts->base + S3C_ADCCLRINT);
#endif

	if (!atomic_read(&ts->closed))
		enable_irq(ts->irq_tc);

	return IRQ_HANDLED;
}


static void s3c_ts_close(struct input_dev *dev)
{
	struct s3c_ts *ts;

	printk_debug("Close function called\n");

	ts = input_get_drvdata(dev);
	atomic_set(&ts->closed, 1);
	disable_irq(ts->irq_adc);
	disable_irq(ts->irq_tc);
}



/* Only disable the interrupts */
static int s3c_ts_open(struct input_dev *dev)
{
	struct s3c_ts *ts;

	printk_debug("Open function called\n");

	ts = input_get_drvdata(dev);
	atomic_set(&ts->closed, 0);
	enable_irq(ts->irq_adc);
	enable_irq(ts->irq_tc);
	return 0;
}


/*
 * The functions for inserting/removing us as a module.
 */
static int __devinit s3c_ts_probe(struct platform_device *pdev)
{
	struct s3c_ts_mach_info *info;
	struct input_dev *indev;
	int err;
	struct s3c_ts *ts;

	info = pdev->dev.platform_data;
	if (!info) {
		printk_err("Too bad: no platform data attached. Aborting.\n");
		return -EINVAL;
	}

	/* Sanity checks */
	if (!info->trigger_ms || info->probes > S3C24XX_SAMPLE_NUM_MAX) {
		printk_err("Found an invalid configuration.\n");
		return -EINVAL;
	}


	ts = kzalloc(sizeof(struct s3c_ts), GFP_KERNEL);
	ts->info = info;
	indev = input_allocate_device();
	if (!ts || !indev) {
		if (ts)
			kfree(ts);
		if (indev)
			kfree(indev);
		return -ENOMEM;
	} else
		ts->indev = indev;

	/* Get the required source clock */
	ts->clk = clk_get(&pdev->dev, "adc");
	if (!ts->clk) {
		printk_err("Failed to get adc clock source\n");
		err = -ENOENT;
		goto err_free_ts;
	}
	clk_enable(ts->clk);

	/* @XXX: Get the memory area from the configuration structure */
	ts->base = ioremap(S3C2410_PA_ADC, 0x20);
	if (!ts->base) {
		printk_err("Failed to ioremap register block\n");
		err = -EINVAL;
		goto err_put_clock;
	}

	/* Prepare the passed user configuration values */
	if ((info->presc & 0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN |
		       S3C2410_ADCCON_PRSCVL(info->presc & 0xFF),
		       ts->base + S3C2410_ADCCON);
	else
		writel(0, ts->base + S3C2410_ADCCON);


	/* Initialise the start delay register (Manual: Do not use zero value) */
	if ((info->delay & 0xffff) > 0)
		writel(info->delay & 0xffff,  ts->base + S3C2410_ADCDLY);
	else
		writel(10000,  ts->base + S3C2410_ADCDLY);

	/* Configure the touch screen controller for "Waiting for Interrupt Mode" */
	writel(WAIT4INT(0), ts->base + S3C2410_ADCTSC);

	ts->indev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->indev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/*
	 * The data values for the position conversion are coming from the data
	 * sheet of the touch controller (S3C2443: 24-10)
	 */
	input_set_abs_params(ts->indev, ABS_X, info->xmin, info->xmax, 0, 0);
	input_set_abs_params(ts->indev, ABS_Y, info->ymin, info->ymax, 0, 0);
	input_set_abs_params(ts->indev, ABS_PRESSURE, 0, 10, 0, 0);

	sprintf(ts->phys, "ts0");

	indev->open = s3c_ts_open;
	indev->close = s3c_ts_close;
	indev->name = s3c_ts_name;
	indev->phys = ts->phys;
	indev->id.bustype = BUS_RS232;
	indev->id.vendor = 0xDEAD;
	indev->id.product = 0xBEEF;
	indev->id.version = S3C_TSVERSION;

	/* Get the irqs from the platform device data */
	ts->irq_adc = platform_get_irq(pdev, 0);
	err = request_irq(ts->irq_adc, stylus_action, 0, "s3c_action", ts);
	if (err) {
		printk_err("Requesting the ADC IRQ %i\n", ts->irq_adc);
		goto err_put_clock;
	}
 	printk_debug("Got ADC IRQ %i\n", ts->irq_adc);

	ts->irq_tc = platform_get_irq(pdev, 1);
	err = request_irq(ts->irq_tc, stylus_updown, 0, "s3c_updown", ts);
	if (err) {
		printk_err("Requesting the touch screen IRQ %i\n", ts->irq_tc);
		goto err_free_adcirq;
	}
 	printk_debug("Got TC IRQ %i\n", ts->irq_tc);


	/* Disable the interrupts first */
	disable_irq(ts->irq_adc);
	disable_irq(ts->irq_tc);

	printk_info("%s successfully loaded\n", s3c_ts_name);

	/* All went ok, so register to the input system */
	err = input_register_device(ts->indev);
	if (err)
		goto err_free_tcirq;


        /* Configure and init the timer for the command timeouts */
        init_timer(&ts->timer);
        ts->timer.function = touch_timer_fire;
        ts->timer.data = (unsigned long)ts;

	input_set_drvdata(indev, ts);
	platform_set_drvdata(pdev, ts);

	return 0;

 err_free_tcirq:
	free_irq(ts->irq_tc, ts);

 err_free_adcirq:
	free_irq(ts->irq_adc, ts);

 err_put_clock:
	clk_disable(ts->clk);
	clk_put(ts->clk);

 err_free_ts:
	input_free_device(ts->indev);
	kfree(ts);

	return err;
}

static int __devexit s3c_ts_remove(struct platform_device *pdev)
{
	struct s3c_ts *ts;

	ts = platform_get_drvdata(pdev);

	printk_debug("Removing the device with the ID %i\n", pdev->id);

	if (!atomic_read(&ts->closed)) {
		disable_irq(ts->irq_adc);
		disable_irq(ts->irq_tc);
	}

	free_irq(ts->irq_adc, ts);
	free_irq(ts->irq_tc, ts);

	if (ts->clk) {
		clk_disable(ts->clk);
		clk_put(ts->clk);
		ts->clk = NULL;
	}

	input_unregister_device(ts->indev);
	iounmap(ts->base);

	input_free_device(ts->indev);
	kfree(ts);
	platform_set_drvdata(pdev, NULL);
	return 0;
}


#ifdef CONFIG_PM
static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	struct s3c_ts *ts;

	ts = platform_get_drvdata(dev);

	ts->adccon = readl(ts->base + S3C2410_ADCCON);
	ts->adctsc = readl(ts->base + S3C2410_ADCTSC);
	ts->adcdly = readl(ts->base + S3C2410_ADCDLY);

	/* Don't forget to re-enable the interrupts! */
	disable_irq(ts->irq_adc);
	disable_irq(ts->irq_tc);
	clk_disable(ts->clk);
	return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
	struct s3c_ts *ts;

	ts = platform_get_drvdata(pdev);

	clk_enable(ts->clk);
	writel(ts->adccon, ts->base + S3C2410_ADCCON);
	writel(ts->adctsc, ts->base + S3C2410_ADCTSC);
	writel(ts->adcdly, ts->base + S3C2410_ADCDLY);
	writel(WAIT4INT(0), ts->base + S3C2410_ADCTSC);

	enable_irq(ts->irq_adc);
	enable_irq(ts->irq_tc);

	return 0;
}
#else
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif

static struct platform_driver s3c_ts_driver = {
       .probe		= s3c_ts_probe,
       .remove		= __devexit_p(s3c_ts_remove),
       .suspend		= s3c_ts_suspend,
       .resume		= s3c_ts_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c24xx-ts",
	},
};

static char banner[] __initdata = KERN_INFO "S3C Touchscreen driver, (c) 2006 Samsung Electronics\n";

static int __init s3c_ts_init(void)
{
	printk("%s", banner);
	return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
	platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("s3c touchscreen driver");
MODULE_LICENSE("GPL");
