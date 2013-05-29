/*
 * linux/drivers/char/adc-s3c2443.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
/*
 * In future, this driver should be replaced by the general adc core support
 * for the Samsung platforms, plus the corresponding hwmon driver and the touch
 * driver that uses that common adc support.
 */

/*
 * TODO, split from this driver and from the touch driver the common stuff and
 * move it to a core adc module.
 */


#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <plat/regs-adc.h>
#include <plat/ts.h>

#define DRIVER_NAME	"s3c2443-adc"
#ifndef NUM_CHANNELS
# define NUM_CHANNELS	10
#endif

struct s3c2443_adc_channel {
	struct semaphore	sem;
};

struct s3c2443_adc_dev {
	void __iomem			*ioaddr;
	struct resource			*res_iomem;
	struct clk			*clk;
	struct cdev			cdev;
	struct s3c2443_adc_channel	channel[NUM_CHANNELS];
	dev_t				dev_id;
	spinlock_t			lock;
	u32				adccon;
	u32				adctsc;
  	u32				adcdly;
};

static struct s3c2443_adc_dev *adc;

static u16 adc_read_sample(struct s3c2443_adc_dev *adcdev, int channel)
{
	u32 adccon, timeout = 5000;

	/* Select channel, start conversion and wait until its completed */
	writel(channel, adcdev->ioaddr + S3C2410_ADCMUX);
	adccon = readl(adcdev->ioaddr + S3C2410_ADCCON);
	writel(adccon | S3C2410_ADCCON_ENABLE_START, adcdev->ioaddr + S3C2410_ADCCON);

	while (timeout--) {
		adccon = readl(adcdev->ioaddr + S3C2410_ADCCON);
		if (adccon & S3C2410_ADCCON_ECFLG)
			break;
		ndelay(100);
	}

	return readl(adcdev->ioaddr + S3C2410_ADCDAT0) & S3C2410_ADCDAT0_XPDATA_MASK;
}


static ssize_t s3c2443_adc_read(struct file *filep, char __user *buff,
				size_t count, loff_t *offp)
{
	struct s3c2443_adc_dev *adcdev = (struct s3c2443_adc_dev *)filep->private_data;
	int ch = iminor(filep->f_dentry->d_inode);
	u16 val12;
	u8 val8;

	val12 = adc_read_sample(adcdev, ch);
	val8 = val12 >> 4;

	count = (count > 2) ? 2 : count;

	if (count == 1)
		copy_to_user(buff, &val8, count);
	else
		copy_to_user(buff, &val12, count);

	return count;
}

static int s3c2443_adc_open(struct inode *inode, struct file *filep)
{
	struct s3c2443_adc_dev *adcdev = container_of(inode->i_cdev,
						      struct s3c2443_adc_dev, cdev);
	int ch = iminor(inode);

	if (ch >= NUM_CHANNELS)
		return -EINVAL;

	if (down_trylock(&adcdev->channel[ch].sem) < 0)
		return -EBUSY;

	filep->private_data = (void *)adcdev;

	return 0;
}

static int s3c2443_adc_release(struct inode *inode, struct file *filep)
{
	struct s3c2443_adc_dev *adcdev = container_of(inode->i_cdev,
						      struct s3c2443_adc_dev, cdev);
	int ch = iminor(inode);

	up(&adcdev->channel[ch].sem);

	return 0;
}

struct file_operations s3c2443_adc_fops = {
	.owner		= THIS_MODULE,
	.read		= s3c2443_adc_read,
	.open		= s3c2443_adc_open,
	.release	= s3c2443_adc_release,
};

static int __devinit s3c2443_adc_probe(struct platform_device *pdev)
{
	int ret, i;
	struct s3c_ts_mach_info *info;

	info = pdev->dev.platform_data;
	if (!info) {
		pr_err(DRIVER_NAME ": no platform data provided\n");
		return -EINVAL;
	}

	adc = kzalloc(sizeof(*adc), GFP_KERNEL);
	if (!adc) {
		pr_err(DRIVER_NAME ": mem alloc error\n");
		return -ENOMEM;
	}

	adc->res_iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM, DRIVER_NAME);
	if (!adc->res_iomem) {
		pr_err(DRIVER_NAME ": unable to find iomem resource\n");
		ret = -ENOENT;
		goto error_get_res;
	}
	/*
	 * Dont request mem region becasue the address space is shared with the touch
         * driver. This will be resolved in future when both drivers use the same core
	 * adc funcionality
	 */
	adc->ioaddr = ioremap(adc->res_iomem->start,
			      adc->res_iomem->end - adc->res_iomem->start + 1);
	if (!adc->ioaddr) {
		pr_err(DRIVER_NAME ": unable to remap IO memory\n");
		ret = -EBUSY;
		goto error_remap;
	}
	pr_debug(DRIVER_NAME ": mapped ADC to virtual address 0x%x\n", (u32)adc->ioaddr);

	ret = alloc_chrdev_region(&adc->dev_id, 0, NUM_CHANNELS, DRIVER_NAME);
	if (ret < 0) {
		pr_err(DRIVER_NAME ": error requesting chrdev_region for %d\n", adc->dev_id);
		goto error_cdev_reg;
	}

	/* Get the required source clock */
	adc->clk = clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)) {
		pr_err(DRIVER_NAME ": error getting adc clock source\n");
		ret = PTR_ERR(adc->clk);
		goto error_get_clk;
	}
	clk_enable(adc->clk);

	/* initialize mutexes for every channel */
	for (i = 0; i < NUM_CHANNELS; i++)
		init_MUTEX(&adc->channel[i].sem);

	cdev_init(&adc->cdev, &s3c2443_adc_fops);
	ret = cdev_add(&adc->cdev, adc->dev_id, NUM_CHANNELS);
	if (ret < 0) {
		pr_err(DRIVER_NAME ": cannot add character device\n");
 		goto error_cdev_add;
	}

	/* Prepare the passed user configuration values */
	if ((info->presc & 0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN |
		       S3C2410_ADCCON_PRSCVL(info->presc & 0xFF),
		       adc->ioaddr + S3C2410_ADCCON);
	else
		writel(0, adc->ioaddr + S3C2410_ADCCON);


	/* Initialise the start delay register (Manual: Do not use zero value) */
	if ((info->delay & 0xffff) > 0)
		writel(info->delay & 0xffff, adc->ioaddr + S3C2410_ADCDLY);
	else
		writel(10000,  adc->ioaddr + S3C2410_ADCDLY);

	platform_set_drvdata(pdev, adc);

	pr_info(DRIVER_NAME ": ADC available on MAJOR %i\n", MAJOR(adc->dev_id));

	return 0;

error_cdev_add:
	clk_put(adc->clk);
error_get_clk:
	unregister_chrdev_region(adc->dev_id, NUM_CHANNELS);
error_cdev_reg:
	iounmap(adc->ioaddr);
error_remap:
error_get_res:
	kfree(adc);
	return ret;
}

int s3c2443_adc_remove(struct platform_device *pdev)
{
	int i;

	if (!adc)
		return -EIO;

	/* lock/check all channels */
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (down_trylock(&adc->channel[i].sem))
			/* channel still locked! */
			goto adc_remove_unlock;
	}

	cdev_del(&adc->cdev);
	clk_disable(adc->clk);
	clk_put(adc->clk);
 	unregister_chrdev_region(adc->dev_id, NUM_CHANNELS);
	iounmap(adc->ioaddr);
	kfree(adc);
	platform_set_drvdata(pdev, NULL);

	return 0;

adc_remove_unlock:
	for (i--; i >= 0; i--)
		up(&adc->channel[i].sem);
	return -EBUSY;
}

#ifdef CONFIG_PM
static int s3c2443_adc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct s3c2443_adc_dev *adc;

	adc = platform_get_drvdata(dev);

	/* save registers and disable the clock */
	adc->adccon = readl(adc->ioaddr + S3C2410_ADCCON);
	adc->adctsc = readl(adc->ioaddr + S3C2410_ADCTSC);
	adc->adcdly = readl(adc->ioaddr + S3C2410_ADCDLY);
	clk_disable(adc->clk);

	return 0;
}

static int s3c2443_adc_resume(struct platform_device *pdev)
{
	struct s3c2443_adc_dev *adc;

	adc = platform_get_drvdata(pdev);

	/* restore registers and enable the clock */
	clk_enable(adc->clk);
	writel(adc->adccon, adc->ioaddr + S3C2410_ADCCON);
	writel(adc->adctsc, adc->ioaddr + S3C2410_ADCTSC);
	writel(adc->adcdly, adc->ioaddr + S3C2410_ADCDLY);

	return 0;
}
#else
#define s3c2443_adc_suspend	NULL
#define s3c2443_adc_resume	NULL
#endif

static struct platform_driver s3c2443_adc_driver = {
	.probe		= s3c2443_adc_probe,
	.remove		= __devexit_p(s3c2443_adc_remove),
	.suspend	= s3c2443_adc_suspend,
	.resume		= s3c2443_adc_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init s3c2443_adc_init(void)
{
	return platform_driver_register(&s3c2443_adc_driver);
}

static void __exit s3c2443_adc_exit(void)
{
	platform_driver_unregister(&s3c2443_adc_driver);
}

module_init(s3c2443_adc_init);
module_exit(s3c2443_adc_exit);

MODULE_AUTHOR("Digi International");
MODULE_DESCRIPTION("ADC driver for S3C2443");
MODULE_LICENSE("GPL v2");
