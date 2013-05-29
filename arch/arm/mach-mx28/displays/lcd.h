/*
 * arch/arm/mach-mx28/displays/lcd.h
 *
 * Copyright (C) 2012 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXS_CCARDIMX28_DISPLAYS_LCD_H__
#define __ASM_ARCH_MXS_CCARDIMX28_DISPLAYS_LCD_H__

#include <linux/fsl_devices.h>
#include <linux/fb.h>

extern void lcd_bl_enable(int enable);

static void lcd_init(struct mxs_fb_platform_data *fb_pdata)
{
	/* TODO: Initialize lcd enable gpio and video interface lines */
}

static struct fb_videomode lq70y3dg3b = {
	.name		= "LQ070Y3DG3B",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= 50000,
	.left_margin	= 87,
	.right_margin	= 41,
	.upper_margin	= 10,
	.lower_margin	= 25,
	.hsync_len	= 128,
	.vsync_len	= 10,
	.vmode		= FB_VMODE_NONINTERLACED,
	//.sync          = FB_SYNC_EXT,
	.flag		= 0,
};

static struct fb_videomode f07a0102 = {
	.name          = "F07A0102",
	.refresh       = 60,
	.xres          = 800,
	.yres          = 480,
	.pixclock	= 50000,
	.left_margin	= 87,
	.right_margin	= 41,
	.upper_margin	= 10,
	.lower_margin	= 25,
	.hsync_len	= 128,
	.vsync_len	= 10,
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= 0,
};

static struct fb_videomode lcd_custom_1 = {
	.name		= "custom1",
	.refresh	= 0,			/* Refresh rate in Hz */
	.xres		= 0,			/* Resolution in the x axis */
	.yres		= 0,			/* Reslution in the y axis */
	.pixclock	= 0,			/* Pixelclock/dotclock,picoseconds. */
	.left_margin	= 0,			/* Horizontal Back Porch (HBP) */
	.right_margin	= 0,			/* Horizontal Front Porch (HFP) */
	.upper_margin	= 0,			/* Vertical Back Porch (VBP) */
	.lower_margin	= 0,			/* Vetical Front Porch (VFP) */
	.hsync_len	= 0,			/* Horizontal sync pulse width */
	.vsync_len	= 0,			/* Vertical sync pulse width */
	.vmode		= FB_VMODE_NONINTERLACED,/* Video mode */
	//.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

static struct fb_videomode lcd_custom_2 = {
	.name		= "custom2",
	.refresh	= 0,			/* Refresh rate in Hz */
	.xres		= 0,			/* Resolution in the x axis */
	.yres		= 0,			/* Reslution in the y axis */
	.pixclock	= 0,			/* Pixelclock or dotclock,picoseconds.*/
	.left_margin	= 0,			/* Horizontal Back Porch (HBP) */
	.right_margin	= 0,			/* Horizontal Front Porch (HFP) */
	.upper_margin	= 0,			/* Vertical Back Porch (VBP) */
	.lower_margin	= 0,			/* Vetical Front Porch (VFP) */
	.hsync_len	= 0,			/* Horizontal sync pulse width */
	.vsync_len	= 0,			/* Vertical sync pulse width */
	.vmode		= FB_VMODE_NONINTERLACED,/* Video mode */
	//.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

/* The following entries must remain untouched.
 * They are placeholders to be filled with LCD
 * displays information defined in NVRAM memory
 * by the user and passed through shared memory
 * by the boot loader.
 */
static struct fb_videomode lcd_custom3_nvram = {
	.name		= "custom3_nv",
	.refresh	= 0,			/* Refresh rate in Hz */
	.xres		= 0,			/* Resolution in the x axis */
	.yres		= 0,			/* Reslution in the y axis */
	.pixclock	= 0,			/* Pixelclock/dotclock,picoseconds. */
	.left_margin	= 0,			/* Horizontal Back Porch (HBP) */
	.right_margin	= 0,			/* Horizontal Front Porch (HFP) */
	.upper_margin	= 0,			/* Vertical Back Porch (VBP) */
	.lower_margin	= 0,			/* Vetical Front Porch (VFP) */
	.hsync_len	= 0,			/* Horizontal sync pulse width */
	.vsync_len	= 0,			/* Vertical sync pulse width */
	.vmode		= FB_VMODE_NONINTERLACED,/* Video mode */
	//.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

static struct fb_videomode lcd_custom4_nvram = {
	.name		= "custom4_nv",
	.refresh	= 0,			/* Refresh rate in Hz */
	.xres		= 0,			/* Resolution in the x axis */
	.yres		= 0,			/* Reslution in the y axis */
	.pixclock	= 0,			/* Pixelclock/dotclock,picoseconds. */
	.left_margin	= 0,			/* Horizontal Back Porch (HBP) */
	.right_margin	= 0,			/* Horizontal Front Porch (HFP) */
	.upper_margin	= 0,			/* Vertical Back Porch (VBP) */
	.lower_margin	= 0,			/* Vetical Front Porch (VFP) */
	.hsync_len	= 0,			/* Horizontal sync pulse width */
	.vsync_len	= 0,			/* Vertical sync pulse width */
	.vmode		= FB_VMODE_NONINTERLACED,/* Video mode */
	//.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

struct mx28_lcd_pdata lcd_panel_list[] = {
	{
		.fb_pdata = {
			.mode_str = "LQ070Y3DG3B",
			.mode = &lq70y3dg3b,
			.num_modes = 1,
		},
		.bl_enable = lcd_bl_enable,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "F07A0102",
			.mode = &f07a0102,
			.num_modes = 1,
		},
		.bl_enable = lcd_bl_enable,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "custom1",
			.mode = &lcd_custom_1,
			.num_modes = 1,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "custom2",
			.mode = &lcd_custom_2,
			.num_modes = 1,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	},
	/* The following entries must remain untouched.
	 * They are placeholders to be filled with LCD
	 * displays information defined in NVRAM memory
	 * by the user and passed through shared memory
	 * by the boot loader.
	 */
	{
		.fb_pdata = {
			.mode_str = "custom3_nv",
			.mode = &lcd_custom3_nvram,
			.num_modes = 1,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "custom4_nv",
			.mode = &lcd_custom4_nvram,
			.num_modes = 1,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	},
};
#endif /* __ASM_ARCH_MXS_CCARDIMX28_DISPLAYS_LCD_H__ */

