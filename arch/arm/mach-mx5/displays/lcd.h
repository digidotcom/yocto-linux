/*
 * arch/arm/mach-mx5/displays/lcd.h
 *
 * Copyright (C) 2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCIMX51_DISPLAYS_LCD_H__
#define __ASM_ARCH_MXC_CCIMX51_DISPLAYS_LCD_H__

#if defined(CONFIG_MODULE_CCIMX51)
#include "../iomux.h"
#include "../mx51_pins.h"
#elif defined(CONFIG_MODULE_CCIMX53)
#include <mach/iomux-v3.h>
#endif

static void lcd_bl_enable(int enable, int vif)
{
#if defined(CONFIG_MODULE_CCIMX51)
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), !enable);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), !enable);
	if (vif == 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), !enable);
#if defined(CONFIG_JSCCIMX51_V2)
	else if (vif == 1)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), !enable);
#endif

#elif defined(CONFIG_MODULE_CCIMX53) && defined(DISP1_ENABLE_GPIO)
#ifdef DISP1_ENABLE_ACT_HIGH
	gpio_set_value(DISP1_ENABLE_GPIO, enable);
#else
	gpio_set_value(DISP1_ENABLE_GPIO, !enable);
#endif
#endif /* CONFIG_MODULE_CCIMX51 */
}

static void lcd_init(int vif)
{
	/* Initialize lcd enable gpio and video interface lines */
	gpio_video_active(vif,
#if defined(CONFIG_MODULE_CCIMX51)
			  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
#else
			  PAD_CTL_DSE_HIGH);
#endif
}

static struct fb_videomode lq70y3dg3b = {
	.name          = "LQ070Y3DG3B",
	.refresh       = 60,
	.xres          = 800,
	.yres          = 480,
	.pixclock	= 54000,
	.left_margin	= 0,
	.right_margin	= 50,
	.upper_margin  = 25,
	.lower_margin  = 10,
	.hsync_len     = 128,
	.vsync_len     = 10,
	.vmode         = FB_VMODE_NONINTERLACED,
	.sync          = FB_SYNC_EXT,
	.flag          = 0,
};

static struct fb_videomode hsd101pfw2 = {
	.name          = "HSD101PFW2",
	.refresh       = 60,
	.xres          = 1024,
	.yres          = 600,
	.pixclock      = 22222, /* 45 MHz in ps  */
	.left_margin   = 0,
	.right_margin  = 0,
	.upper_margin  = 0,
	.lower_margin  = 0,
	.hsync_len     = 176,
	.vsync_len     = 25,
	.vmode         = FB_VMODE_NONINTERLACED,
	.sync          = FB_SYNC_CLK_LAT_FALL | FB_SYNC_EXT,
	.flag          = 0,
};

static struct fb_videomode f04b0101 = {
	.name          = "F04B0101",
	.refresh       = 60,
	.xres          = 480,
	.yres          = 272,
	.pixclock      = 111111, /* 9 MHz in ps  */
	.left_margin   = 30,
	.right_margin  = 5,
	.upper_margin  = 0,
	.lower_margin  = 8,
	.hsync_len     = 10,
	.vsync_len     = 8,
	.vmode         = FB_VMODE_NONINTERLACED,
	.sync          = FB_SYNC_EXT,
	.flag          = 0,
};

static struct fb_videomode f07a0102 = {
	.name          = "F07A0102",
	.refresh       = 60,
	.xres          = 800,
	.yres          = 480,
	.pixclock      = 30000,
	.left_margin   = 0,
	.right_margin  = 50,
	.upper_margin  = 25,
	.lower_margin  = 10,
	.hsync_len     = 128,
	.vsync_len     = 10,
	.vmode         = FB_VMODE_NONINTERLACED,
	.sync          = FB_SYNC_EXT,
	.flag          = 0,
};


static struct fb_videomode lq121k1lg11 = {
	.name          = "LQ121K1LG11",
	.refresh       = 60,
	.xres          = 1280,
	.yres          = 800,
	.pixclock      = 18000, /* 55.55 MHz in ps  */
	.left_margin   = 10,
	.right_margin  = 370,
	.upper_margin  = 10,
	.lower_margin  = 10,
	.hsync_len     = 10,
	.vsync_len     = 10,
	.vmode         = FB_VMODE_NONINTERLACED,
	.sync          = FB_SYNC_CLK_LAT_FALL | FB_SYNC_EXT,
	.flag          = 0,
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
	.sync		= FB_SYNC_EXT,		/* Data clock polarity */
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
	.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

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
	.sync		= FB_SYNC_EXT,		/* Data clock polarity */
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
	.sync		= FB_SYNC_EXT,		/* Data clock polarity */
};

struct ccimx5x_lcd_pdata lcd_panel_list[] = {
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
			.mode_str = "HSD101PFW2",
			.mode = &hsd101pfw2,
			.num_modes = 1,
			.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		},
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "F04B0101",
			.mode = &f04b0101,
			.num_modes = 1,
			.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		},
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "F07A0102",
			.mode = &f07a0102,
			.num_modes = 1,
		},
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.mode_str = "LQ121K1LG11",
			.mode = &lq121k1lg11,
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
	}, {
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
#endif /* __ASM_ARCH_MXC_CCIMX51_DISPLAYS_LCD_H__ */

