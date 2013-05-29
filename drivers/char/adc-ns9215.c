/*
 * linux/drivers/char/adc-ns9215.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

/* registers */
#define ADC_CONFIG			(0x0)
#define ADC_CLOCK			(0x4)
#define ADC_OUTPUT			(0x8)
#define ADC_OUTPUT_OFFS			(0x4)

/* configuration bit fields */
#define ADC_CONFIG_DISABLE		(0 << 31)
#define ADC_CONFIG_ENABLE		(1 << 31)
#define ADC_CONFIG_ENABLECHANNELS	(7)

#define DRIVER_NAME	"adc-ns9215"
#define CHANNELS	8
#define MAXCLOCK	14000000	/* max 14MHz */
#define MAXCLOCKN	0x3FF

struct ns9215_adc_channel {
	struct semaphore	sem;
	int			pos;
	unsigned char		lb;
};

struct ns9215_adc_pdata {
	void __iomem			*ioaddr;
	struct resource			*mem;
	struct clk			*clk;
	struct cdev			cdev;
	struct ns9215_adc_channel	channel[CHANNELS];
	dev_t				dev_id;
};

static struct ns9215_adc_pdata pdata;

ssize_t ns9215_adc_read(struct file *filep, char __user *buff,
		size_t count, loff_t *offp)
{
	unsigned int val;
	unsigned char buf[2];
	int channel = (int)filep->private_data;

	if (pdata.channel[channel].pos == 1) {
		copy_to_user(buff, &pdata.channel[channel].lb, 1);
		pdata.channel[channel].pos = 0;
		return 1;
	}

	val = ioread32(pdata.ioaddr + ADC_OUTPUT + (channel * ADC_OUTPUT_OFFS));
	buf[1] = (val & 0xf00) >> 8;
	buf[0] = val & 0xff;

	if (count == 1) {
		copy_to_user(buff, buf , 1);
		pdata.channel[channel].pos = 1;
		pdata.channel[channel].lb = buf[1];
		return 1;
	}

	copy_to_user(buff, buf, 2);
	return 2;
}

int ns9215_adc_open(struct inode *inode, struct file *filep)
{
	int channel = iminor(inode);

	if (down_trylock(&pdata.channel[channel].sem) < 0)
		return -EBUSY;

	filep->private_data = (void *)channel;
	pdata.channel[channel].pos = 0;

	return 0;
}

int ns9215_adc_release(struct inode *inode, struct file *filep)
{
	int channel = iminor(inode);

	up(&pdata.channel[channel].sem);

	return 0;
}

struct file_operations ns9215_adc_fops = {
	.owner = THIS_MODULE,
	.read = ns9215_adc_read,
	.open = ns9215_adc_open,
	.release = ns9215_adc_release,
};

static int __devinit ns9215_adc_probe(struct platform_device *pdev)
{
	int i, ret;
	unsigned int clockn;

	pdata.mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata.mem) {
		pr_err(DRIVER_NAME ": memory not available\n");
		return -ENOENT;
	}

	if (!request_mem_region(pdata.mem->start,
			pdata.mem->end - pdata.mem->start + 1,
			DRIVER_NAME)) {
		pr_err(DRIVER_NAME ": memory already mapped\n");
		goto ns9215_err_mem;
	}

	pdata.ioaddr = ioremap(pdata.mem->start,
			pdata.mem->end - pdata.mem->start + 1);
	if (pdata.ioaddr <= 0) {
		pr_err(DRIVER_NAME ": unable to remap IO memory\n");
		goto ns9215_err_remap;
	}
	pr_debug(DRIVER_NAME ": mapped ADC to virtual address 0x%x\n",
			(int)pdata.ioaddr);

	if (alloc_chrdev_region(&pdata.dev_id, 0, CHANNELS, DRIVER_NAME) < 0) {
		pr_err(DRIVER_NAME ": requesting chrdev_region fails\n");
		goto ns9215_err_cdev_reg;
	}

	pdata.clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(pdata.clk)) {
		ret = PTR_ERR(pdata.clk);
		goto ns9215_err_clk_get;
	}

	ret = clk_enable(pdata.clk);
	if (ret) {
		pr_err(DRIVER_NAME ": cannot enable clock\n");
		goto ns9215_err_clk_enable;
	}

	/* set clock */
	clockn = ((clk_get_rate(pdata.clk)) - 2 * MAXCLOCK)
		/ (2 * MAXCLOCK) + 1;
	if (clockn > MAXCLOCKN) {
		pr_warning(DRIVER_NAME ": cannot set a proper ADC clock,"
				" ADC will not work correctly!\n");
		clockn = MAXCLOCKN;
	}

	iowrite32(clockn, pdata.ioaddr + ADC_CLOCK);

	/* initialize mutexes for every channel */
	for (i = 0; i < CHANNELS; i++) {
		init_MUTEX(&pdata.channel[i].sem);
		pdata.channel[i].pos = 0;
	}

	/* enable adc & all channels */
	iowrite32(ADC_CONFIG_ENABLE | ADC_CONFIG_ENABLECHANNELS,
			pdata.ioaddr + ADC_CONFIG);

	cdev_init(&pdata.cdev, &ns9215_adc_fops);
	if (cdev_add(&pdata.cdev, pdata.dev_id, CHANNELS) < 0) {
		pr_err(DRIVER_NAME ": cannot add character device\n");
		goto ns9215_err_cdev_add;
	}

	pr_info(DRIVER_NAME ": ADC available on MAJOR %i\n",
			MAJOR(pdata.dev_id));

	return 0;

ns9215_err_cdev_add:
	clk_disable(pdata.clk);
ns9215_err_clk_enable:
	clk_put(pdata.clk);
ns9215_err_clk_get:
	unregister_chrdev_region(pdata.dev_id, CHANNELS);
ns9215_err_cdev_reg:
	iounmap(pdata.ioaddr);
ns9215_err_remap:
	release_mem_region(pdata.mem->start,
			pdata.mem->end - pdata.mem->start + 1);
ns9215_err_mem:

	return -EIO;
}

static int ns9215_adc_remove(struct platform_device *pdev)
{
	int i;

	/* lock/check all channels */
	for (i = 0; i < CHANNELS; i++)
		if (down_trylock(&pdata.channel[i].sem))
			/* channel still locked! */
			goto ns9215_adc_remove_unlock;

	/* disable ADC */
	iowrite32(ADC_CONFIG_DISABLE,  pdata.ioaddr + ADC_CONFIG);

	cdev_del(&pdata.cdev);
	clk_disable(pdata.clk);
	clk_put(pdata.clk);
	unregister_chrdev_region(pdata.dev_id, CHANNELS);
	iounmap(pdata.ioaddr);
	release_mem_region(pdata.mem->start,
			pdata.mem->end - pdata.mem->start + 1);

	return 0;

ns9215_adc_remove_unlock:
	for (i--; i >= 0; i--)
		up(&pdata.channel[i].sem);
	return -EBUSY;
}

static struct platform_driver ns9215_adc_driver = {
	.probe		= ns9215_adc_probe,
	.remove		= ns9215_adc_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	/* TODO: power management */
};

static int __init ns9215_adc_init(void)
{
	return platform_driver_register(&ns9215_adc_driver);
}

static void __exit ns9215_adc_exit(void)
{
	platform_driver_unregister(&ns9215_adc_driver);
}

module_init(ns9215_adc_init);
module_exit(ns9215_adc_exit);

MODULE_AUTHOR("Matthias Ludwig");
MODULE_DESCRIPTION("ADC driver for Digi NS9215");
MODULE_LICENSE("GPL v2");
