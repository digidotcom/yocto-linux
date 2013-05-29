/*
 * arch/arm/mach-mx5/displays/hdmi_ad9389.h
 *
 * Copyright (C) 2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCIMX51_DISPLAYS_HDMI_AD9389_H__
#define __ASM_ARCH_MXC_CCIMX51_DISPLAYS_HDMI_AD9389_H__

static struct fb_videomode ad9389_1280x720x24 = {
	.name		= "1280x720",
	.refresh	= 60,
	.xres		= 1280,
	.yres		= 720,
	.pixclock	= 20100,
	.left_margin	= 32,
	.right_margin	= 48,
	.upper_margin	= 7,
	.lower_margin	= 3,
	.hsync_len	= 32,
	.vsync_len	= 6,
};

static struct fb_videomode ad9389_1360x768x24 = {
	.name		= "1360x768",
	.refresh	= 60,
	.xres		= 1360,
	.yres		= 768,
	.pixclock	= 16000,
	.left_margin	= 139,
	.right_margin	= 256,
	.upper_margin	= 3,
	.lower_margin	= 18,
	.hsync_len	= 76,
	.vsync_len	= 6,
};


static struct fb_videomode ad9389_1366x768x24 = {
	.name		= "1366x768",
	.refresh	= 60,
	.xres		= 1366,
	.yres		= 768,
	.pixclock	= 16000,
	.left_margin	= 139,
	.right_margin	= 256,
	.upper_margin	= 3,
	.lower_margin	= 18,
	.hsync_len	= 76,
	.vsync_len	= 6,
};

static struct fb_videomode ad9389_1920x1080x24 = {
	.name		= "1920x1080",
	.refresh	= 60,
	.xres 		= 1920,
	.yres 		= 1080,
	.pixclock 	= 7560,
	.left_margin 	= 148,
	.right_margin 	= 88,
	.upper_margin	= 36,
	.lower_margin 	= 4,
	.hsync_len 	= 44,
	.vsync_len 	= 5,
};

static struct fb_videomode ad9389_1024x768x24 = {
	.name		= "1024x768",
	.refresh	= 60,
	.xres		= 1024,
	.yres		= 768,
	.pixclock	= 15384, /* pico seconds of 65.0MHz */
	.left_margin	= 160,
	.right_margin	= 24,
	.upper_margin	= 29,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
};

static struct fb_videomode ad9389_custom_1 = {
	.name		= "custom1",
	.refresh	= 0,
	.xres		= 0,
	.yres		= 0,
	.pixclock	= 0,
	.left_margin	= 0,
	.right_margin	= 0,
	.upper_margin	= 0,
	.lower_margin	= 0,
	.hsync_len	= 0,
	.vsync_len	= 0,
	.sync		= 0,
};

static struct fb_videomode ad9389_custom_2 = {
	.name		= "custom2",
	.refresh	= 0,
	.xres		= 0,
	.yres		= 0,
	.pixclock	= 0,
	.left_margin	= 0,
	.right_margin	= 0,
	.upper_margin	= 0,
	.lower_margin	= 0,
	.hsync_len	= 0,
	.vsync_len	= 0,
	.sync		= 0,
};

struct ccimx5x_lcd_pdata ad9389_panel_list[] = {
	{
		.fb_pdata = {
			.mode_str = "1280x720",
			.mode = &ad9389_1280x720x24,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "1360x768",
			.mode = &ad9389_1360x768x24,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "1366x768",
			.mode = &ad9389_1366x768x24,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "1920x1080",
			.mode = &ad9389_1920x1080x24,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "1024x768",
			.mode = &ad9389_1024x768x24,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "custom1",
			.mode = &ad9389_custom_1,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "custom2",
			.mode = &ad9389_custom_2,
		},
		.bl_enable = NULL,
	},
};
#endif /* __ASM_ARCH_MXC_CCIMX51_DISPLAYS_HDMI_AD9389_H__ */