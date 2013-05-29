/* -*- linux-c -*-
 *
 * include/asm-arm/plat-s3c24xx/pwm.h
 *
 * Copyright (c) 2010 Digi International Spain
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_ARM_S3C24XX_PWM_H
#define __ASM_ARM_S3C24XX_PWM_H

#include <linux/clk.h>

/* Maximal number of available channels */
#define S3C24XX_PWM_CHANNEL_MAX                4

/* This is the data for the PWM channels */
struct s3c24xx_pwm_channel {
	/* platform defined */
	int timer;
	int gpio;
	/* internal variables */
	unsigned char tcon_base;
	unsigned char running;
	unsigned char use_count;
	struct clk *clk;
	struct clk *clk_div;

	/* Additional channel configuration variables ... */
};

/* PWM data */
struct s3c24xx_pwm_pdata {
	int number_channels;
	struct s3c24xx_pwm_channel *channels;
};

#endif /* __ASM_ARM_S3C24XX_PWM_H */
