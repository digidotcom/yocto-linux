/*
 * drivers/watchdog/ns9xxx.c
 *
 * based on at91rm9200_wdt.c by Andrew Victor
 *
 * Copyright (C) 2009 Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

#define WDT_CONFIG			(0)
#define WDT_TIMER			(4)

#define WDT_CONFIG_ENABLE		(1 << 7)
#define WDT_CONFIG_IRQCLEAR		(1 << 5)
#define WDT_CONFIG_RESPONSE		(1 << 4)
#define WDT_CONFIG_DIV64		(0x5)

#define DRIVER_NAME			"ns9xxx-wdt"
#define DEFAULT_TIME			10

#define ns9xxx_wdt_pat()				\
{							\
	iowrite32(pdata.multiplier * timeout,		\
			pdata.ioaddr + WDT_TIMER);	\
}

struct ns9xxx_wdt_pdata {
	void __iomem		*ioaddr;
	struct resource		*mem;
	struct clk		*clk;

	unsigned int		multiplier;
	unsigned int		timeout_max;

	unsigned long		busy;
};

static struct ns9xxx_wdt_pdata pdata;

static unsigned int timeout = DEFAULT_TIME;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog Timeout in seconds. "
		"(default=" __MODULE_STRING(DEFAULT_TIME) ")");

static struct watchdog_info ns9xxx_wdt_info = {
	.identity	= "ns9xxx watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

/* disable watchdog */
static inline void ns9xxx_wdt_stop(void)
{
	unsigned long cfg;

	/* watchdog cannot be disabled,
	 * but stopped by clearing WDT_CONFIG_IRQCLEAR  */
	cfg = ioread32(pdata.ioaddr + WDT_CONFIG) & ~WDT_CONFIG_IRQCLEAR;
	iowrite32(cfg, pdata.ioaddr + WDT_CONFIG);
}

/* enable/reset watchdog */
static inline void ns9xxx_wdt_start(void)
{
	ns9xxx_wdt_pat();

	/* enable watchdog
	 * reset interrupt
	 * action = reset device
	 * divisor = 64 (slowest)
	 */
	iowrite32(WDT_CONFIG_ENABLE | WDT_CONFIG_IRQCLEAR
			| WDT_CONFIG_RESPONSE | WDT_CONFIG_DIV64,
			pdata.ioaddr + WDT_CONFIG);
}

/* device is opened, start watchdog */
static int ns9xxx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &pdata.busy))
		return -EBUSY;

	ns9xxx_wdt_start();
	return nonseekable_open(inode, file);
}

/* device is closed, watchdog will not(!) be disabled */
static int ns9xxx_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &pdata.busy);
	return 0;
}

/* update the time interval */
static int ns9xxx_wdt_settimeout(int new_time)
{
	/* skip check if we do not have a clock speed yet */
	if ((new_time <= 0) || (new_time > pdata.timeout_max))
		return -EINVAL;

	timeout = new_time;
	return 0;
}

/* handle ioctl */
static int ns9xxx_wdt_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;
	int ret;

	switch (cmd) {
	case WDIOC_KEEPALIVE:
		ns9xxx_wdt_pat();
		return 0;
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ns9xxx_wdt_info,
				sizeof(ns9xxx_wdt_info)) ? -EFAULT : 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		ret = ns9xxx_wdt_settimeout(new_value);
		if (ret)
			return ret;
		ns9xxx_wdt_start();
		return put_user(timeout, p);
	case WDIOC_GETTIMEOUT:
		return put_user(timeout, p);
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		if (new_value & WDIOS_ENABLECARD)
			ns9xxx_wdt_start();
		return 0;
	default:
		return -ENOTTY;
	}
}

/* pat watchdog on every write to the device */
static ssize_t ns9xxx_wdt_write(struct file *file,
		const char __user *data, size_t len, loff_t *ppos)
{
	ns9xxx_wdt_pat();
	return len;
}

static const struct file_operations ns9xxx_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.ioctl		= ns9xxx_wdt_ioctl,
	.open		= ns9xxx_wdt_open,
	.release	= ns9xxx_wdt_close,
	.write		= ns9xxx_wdt_write,
};

static struct miscdevice ns9xxx_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= DRIVER_NAME,
	.fops		= &ns9xxx_wdt_fops,
};

static int __devinit ns9xxx_wdt_probe(struct platform_device *pdev)
{
	int ret;

	if (ns9xxx_wdt_miscdev.parent)
		return -EBUSY;

	ns9xxx_wdt_miscdev.parent = &pdev->dev;

	pdata.mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata.mem) {
		pr_err(DRIVER_NAME ": memory not available\n");
		return -ENOENT;
	}

	if (!request_mem_region(pdata.mem->start,
				pdata.mem->end - pdata.mem->start + 1,
				DRIVER_NAME)) {
		pr_err(DRIVER_NAME ": memory already mapped\n");
		ret = -EIO;
		goto err_mem;
	}

	pdata.ioaddr = ioremap(pdata.mem->start,
			pdata.mem->end - pdata.mem->start + 1);
	if (!pdata.ioaddr) {
		pr_err(DRIVER_NAME ": unable to remap IO memory\n");
		ret = -EIO;
		goto err_remap;
	}

	pdata.clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(pdata.clk)) {
		pr_err(DRIVER_NAME ": clock not available\n");
		ret = PTR_ERR(pdata.clk);
		goto err_clk_get;
	}

	if (!clk_get_rate(pdata.clk)) {
		pr_err(DRIVER_NAME ": cannot get clock rate\n");
		ret = -EIO;
		goto err_clk_rate;
	}

	/* calculate counter speed and maximum time
	 * clock speed is cpu speed divided by 64 (slowest mode)
	 * counter uses full 32bit register
	 */

#if defined(CONFIG_PROCESSOR_NS9210) || defined(CONFIG_PROCESSOR_NS9215)
	pdata.multiplier = 2 * clk_get_rate(pdata.clk) / 64;
#else
	pdata.multiplier = clk_get_rate(pdata.clk) / 64;
#endif
	pdata.timeout_max = 0;
	pdata.timeout_max = ~pdata.timeout_max / pdata.multiplier;

	/* recheck timeout for overflow */
	ns9xxx_wdt_settimeout(timeout);

	ret = misc_register(&ns9xxx_wdt_miscdev);
	if (ret) {
		pr_err(DRIVER_NAME ": cannot register misc device\n");
		goto err_register;
	}

	dev_info(&pdev->dev, "NS9xxx watchdog timer at 0x%p\n",
			pdata.ioaddr);
	return 0;

err_register:
err_clk_rate:
	clk_put(pdata.clk);
err_clk_get:
	iounmap(pdata.ioaddr);
err_remap:
	release_mem_region(pdata.mem->start,
			 pdata.mem->end - pdata.mem->start + 1);
err_mem:
	release_resource(pdata.mem);

	return ret;
}

static int __devexit ns9xxx_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&ns9xxx_wdt_miscdev);
	if (!res)
		ns9xxx_wdt_miscdev.parent = NULL;

	return res;
}

static void ns9xxx_wdt_shutdown(struct platform_device *pdev)
{
	ns9xxx_wdt_stop();
}

#ifdef CONFIG_PM

static int ns9xxx_wdt_suspend(struct platform_device *pdev,
		pm_message_t message)
{
	ns9xxx_wdt_stop();
	return 0;
}

static int ns9xxx_wdt_resume(struct platform_device *pdev)
{
	if (pdata.busy)
		ns9xxx_wdt_start();
	return 0;
}

#else
# define ns9xxx_wdt_suspend NULL
# define ns9xxx_wdt_resume	NULL
#endif


static struct platform_driver ns9xxx_wdt_driver = {
	.probe		= ns9xxx_wdt_probe,
	.remove		= __devexit_p(ns9xxx_wdt_remove),
	.shutdown	= ns9xxx_wdt_shutdown,
	.suspend	= ns9xxx_wdt_suspend,
	.resume		= ns9xxx_wdt_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ns9xxx_wdt_init(void)
{
	return platform_driver_register(&ns9xxx_wdt_driver);
}

static void __exit ns9xxx_wdt_exit(void)
{
	return platform_driver_unregister(&ns9xxx_wdt_driver);
}

module_init(ns9xxx_wdt_init);
module_exit(ns9xxx_wdt_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Digi NS9xxx Watchdog Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:" DRIVER_NAME);
