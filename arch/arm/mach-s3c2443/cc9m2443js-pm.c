/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/cc9m2443js-pm.c
 *
 * Copyright (c) 2009 Digi International Spain
 *
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author : Luis Galdos <luis.galdos@digi.com>
 *
 */

#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/memory.h>
#include <asm/memory.h>
#include <mach/map.h>
#include <mach/regs-s3c2443-mem.h>
#include <mach/hardware.h>

#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>
#include <mach/regs-power.h>

#include <linux/interrupt.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>

#include "cc9m2443js-pm.h"


#define CC9M2443_PM_ITEM_OFFSET(off)           { .offset = off }
#define CC9M2443_PM_ITEM_BASE(bas)             { .base = bas }

struct cc9m2443js_pm_gpio {
	int gpio;
	unsigned long config;
	int irq;
	int state;
	unsigned long flags;
	irqreturn_t (* irq_handler)(int irq, void *gpio);
};

struct cc9m2443_pm_reg {
	void __iomem *base;
	unsigned long offset;
	unsigned long val;
};

/*
 * The SSMC registers are not part of the minimal IO mapping of the CPU
 */
static void __iomem *ssmc_vptr = NULL;

/* Converts the virtual to physical address of the stack pointer */
unsigned long sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

/* These are the register for the external Ethernet controller (CS5) */
static struct cc9m2443_pm_reg s3c2443_regs_ssmc[] = {
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBIDCYR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTRDR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTWRR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTOENR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTWENR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBCR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTBRDR5),
};

/* We MUST save the reset configuration register */
static struct cc9m2443_pm_reg s3c2443_regs_syscon[] = {
	CC9M2443_PM_ITEM_BASE(S3C2443_RSTCON),
	CC9M2443_PM_ITEM_BASE(S3C2443_PWRCFG),
	CC9M2443_PM_ITEM_BASE(S3C2443_OSCSET),
};

/* If the passed register doesn't have a base register, the offset will be used */
static void cc9m2443_pm_restore_regs(struct cc9m2443_pm_reg *regs, int count,
				     void __iomem *offptr)
{
	int cnt;
	void __iomem *reg_ptr;
	
	if (!regs) {
		printk(KERN_ERR "[ RESUME ] NULL pointer passed.\n");
		return;
	}

	for (cnt = 0; cnt < count; cnt++, regs++) {
		reg_ptr = (regs->base) ? (regs->base) : (offptr + regs->offset);
		__raw_writel(regs->val, reg_ptr);
	}
}

/* If the passed register doesn't have a base register, the offset will be used */
static void cc9m2443_pm_save_regs(struct cc9m2443_pm_reg *regs, int count,
				  void __iomem *offptr)
{
	int cnt;
	void __iomem *reg_ptr;
	
	if (!regs) {
		printk(KERN_ERR "[ SUSPEND ] NULL pointer passed!\n");
		return;
	}
	
	for (cnt = 0; cnt < count; cnt++, regs++) {
		reg_ptr = (regs->base) ? (regs->base) : (offptr + regs->offset);
		regs->val = __raw_readl(reg_ptr);
	}
}

/* Save all the registers that are part of the CC9M2443 */
static int cc9m2443_pm_save(void)
{
	/* First map to our virtual memory if required */
	if (!ssmc_vptr)
		ssmc_vptr = ioremap_nocache(S3C2443_PA_SSMC, S3C2443_SZ_SSMC);

	if (!ssmc_vptr)
		printk(KERN_ERR "Couldn't save the regs of the SSMC\n");
	else
		cc9m2443_pm_save_regs(s3c2443_regs_ssmc, ARRAY_SIZE(s3c2443_regs_ssmc),
				      ssmc_vptr);

	cc9m2443_pm_save_regs(s3c2443_regs_syscon, ARRAY_SIZE(s3c2443_regs_syscon),
			      NULL);
	
	return 0;
}

static int cc9m2443_pm_restore(void)
{
	if (ssmc_vptr)
		cc9m2443_pm_restore_regs(s3c2443_regs_ssmc,
					 ARRAY_SIZE(s3c2443_regs_ssmc), ssmc_vptr);

	cc9m2443_pm_restore_regs(s3c2443_regs_syscon,
				 ARRAY_SIZE(s3c2443_regs_syscon), NULL);
	
	return 0;
}

/*
 * This is the CPU-specific prepare function (Code coming from the WinCE world)
 * (Luis Galdos)
 */
static void s3c2443_pm_prepare(void)
{
	__raw_writel(0xff80, S3C2443_RSTCON);
	__raw_writel(0x8000, S3C2443_OSCSET);
	__raw_writel(0x8008, S3C2443_PWRCFG);
}

static int cc9m2443js_pm_suspend(struct sys_device *sd, pm_message_t state)
{
        /* Write the magic value u-boot uses to check for resume into
         * the INFORM0 register, and ensure INFORM1 is set to the
         * correct address to resume from. */

	printk(KERN_DEBUG "[ SUSPEND ] Setting resume address to %p [0x%lx]\n",
	       s3c_cpu_resume,
	       virt_to_phys(s3c_cpu_resume));
        __raw_writel(0x2BED, S3C2412_INFORM0);
        __raw_writel(virt_to_phys(s3c_cpu_resume), S3C2412_INFORM1);

	/*
	 * This two function pointers are used by the platform driver (see [1])
	 * for preparing and entering in the sleep mode.
	 * [1] arch/arm/plat-s3c24xx/pm.c
	 */
	pm_cpu_prep = s3c2443_pm_prepare;
	pm_cpu_sleep = s3c2443_cpu_suspend;

	/* Save the registers */
	cc9m2443_pm_save();
	
        return 0;
}

static int cc9m2443js_pm_resume(struct sys_device *sd)
{
        __raw_writel(0x0, S3C2412_INFORM0);
	__raw_writel(0x0, S3C2412_INFORM1);
	__raw_writel(0x0, S3C2412_INFORM2);
	
	cc9m2443_pm_restore();
	
        return 0;
}

/* Internal system class/device for accessing to the PM-functions */
static struct sysdev_class cc9m2443js_pm_sysclass = {
        .name           = "cc9m2443js-pm",
        .suspend        = cc9m2443js_pm_suspend,
        .resume         = cc9m2443js_pm_resume,
};

static struct sys_device cc9m2443js_pm_sysdev = {
        .cls            = &cc9m2443js_pm_sysclass,
};

/* This function generates the required sysclass for our platform */
void __init cc9m2443js_pm_init(void)
{	
	sysdev_class_register(&cc9m2443js_pm_sysclass);
	sysdev_register(&cc9m2443js_pm_sysdev);

	/* This is the main function for the PM of the Samsung-platforms */
	s3c_pm_init();
}
