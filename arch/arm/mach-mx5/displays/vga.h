/*
 * arch/arm/mach-mx5/displays/vga.h
 *
 * Copyright (C) 2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCIMX51_DISPLAYS_VGA_H__
#define __ASM_ARCH_MXC_CCIMX51_DISPLAYS_VGA_H__

static struct fb_videomode vga_custom_1 = {
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
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

static struct fb_videomode vga_custom_2 = {
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
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

struct ccimx5x_lcd_pdata vga_panel_list[] = {
	{
		.fb_pdata = {
			.mode_str = "custom1",
			.mode = &vga_custom_1,
		},
		.bl_enable = NULL,
	}, {
		.fb_pdata = {
			.mode_str = "custom2",
			.mode = &vga_custom_2,
		},
		.bl_enable = NULL,
	},
};
#endif /* __ASM_ARCH_MXC_CCIMX51_DISPLAYS_VGA_H__ */