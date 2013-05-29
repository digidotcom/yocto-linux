/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/cc9m2443js-pm.h
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

#ifndef _CC9M2443JS_PM_H
#define _CC9M2443JS_PM_H

#include <linux/sysdev.h>
#include <plat/pm.h>

unsigned long sleep_phys_sp(void *sp);

/* These are coming from the assembler code */
extern void s3c2443_cpu_suspend(void);
extern void s3c2443_cpu_resume(void);

void cc9m2443js_pm_init(void);

#endif /* _CC9M2443JS_PM_H */
