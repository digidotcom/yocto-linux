/*
 * linux/drivers/video/s3c2443fb_tft.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Derived from s3c2410fb.c from Arnaud Patard and Ben Dooks
 * It could be merged in future
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <asm/mach/map.h>
#include <mach/idle.h>
#include <mach/s3c2443-fb.h>
#include <mach/regs-gpio.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#define printk_err(fmt, args...)	printk(KERN_ERR "[ ERROR ] " DRV_NAME ": " fmt, ## args)
#define printk_info(fmt, args...)	printk(KERN_INFO "[ INFO ] " DRV_NAME ": " fmt, ## args)

#if 0
#define S3C2443_FB_DEBUG
#endif

#ifdef S3C2443_FB_DEBUG
#  define printk_debug(fmt, args...)	printk(KERN_DEBUG DRV_NAME ": " fmt, ## args)
#else
#  define printk_debug(fmt, args...)
#endif

#define DRV_NAME "s3c2443fb-tft"
#define PALETTE_BUFF_CLEAR (0x80000000)

struct s3c2443fb_info {
	struct device	*dev;
	struct clk	*clk;
	struct resource	*mem;
	void __iomem	*io;
	void __iomem	*irq_base;
	unsigned int	palette_ready;
	u32		palette_buffer[256];
	u32		pseudo_pal[16];
};

static int s3c2443_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;
	struct s3c2443fb_mach_info *mach_info = fbi->dev->platform_data;
	struct s3c2443fb_display *display = NULL;

	display = mach_info->display;
	if (!display) {
		dev_err(fbi->dev, "unable to access display info(null)\n");
		return -EINVAL;
	}

	/* it is always the size as the display */
	var->xres_virtual = display->xres;
	var->yres_virtual = display->yres;
	var->height = display->height;
	var->width = display->width;

	/* copy lcd settings */
	var->pixclock = display->pixclock;
	var->left_margin = display->left_margin;
	var->right_margin = display->right_margin;
	var->upper_margin = display->upper_margin;
	var->lower_margin = display->lower_margin;
	var->vsync_len = display->vsync_len;
	var->hsync_len = display->hsync_len;
	var->transp.offset = 0;
	var->transp.length = 0;

	/* set r/g/b positions */
	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
		var->red.offset	= 0;
		var->red.length	= var->bits_per_pixel;
		var->green	= var->red;
		var->blue	= var->red;
		break;
	case 8:
		/* 8 bpp 332 */
		var->red.length		= 3;
		var->red.offset		= 5;
		var->green.length	= 3;
		var->green.offset	= 2;
		var->blue.length	= 2;
		var->blue.offset	= 0;
		break;
	case 12:
		/* 12 bpp 444 */
		var->red.length		= 4;
		var->red.offset		= 8;
		var->green.length	= 4;
		var->green.offset	= 4;
		var->blue.length	= 4;
		var->blue.offset	= 0;
		break;
	case 16:
		/* 16 bpp, 565 format */
		if (display->bpp_mode == S3C24XX_LCD_WINCON_16BPP_565) {
			/* 16 bpp, 565 format */
			var->red.offset		= 11;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 6;
			var->blue.length	= 5;
		} else {
			/* 16 bpp, 1555 format */
			var->red.offset		= 10;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 5;
			var->blue.length	= 5;
		}
		break;
	case 18:
		/* 18bpp only supports 666 */
		var->red.offset		= 12;
		var->green.offset	= 6;
		var->blue.offset	= 0;
		var->red.length		= 6;
		var->green.length	= 6;
		var->blue.length	= 6;
		break;
	case 32:
		/* 24 bpp 888 and 8 dummy */
		var->red.length		= 8;
		var->red.offset		= 16;
		var->green.length	= 8;
		var->green.offset	= 8;
		var->blue.length	= 8;
		var->blue.offset	= 0;
		break;
	}

	return 0;
}

/* Write into the register the passed display configuration */
static void s3c2443fb_activate_var(struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;
	struct s3c2443fb_mach_info *mach_info = fbi->dev->platform_data;
	struct s3c2443fb_display *display = mach_info->display;
	unsigned short hsync_cnt, vclk_cnt;
	unsigned char clkval;
	unsigned long pixel_clk, lcd_clk;
	unsigned long vidcon0, vidcon1, vidtcon0, vidtcon1, vidtcon2;
	unsigned long wincon0, wincon1;
	unsigned long vidosd0a, vidosd0b;

	/* Compute number of clocks per frame */
	hsync_cnt = display->lower_margin + /* VBPD */
		display->upper_margin +     /* VFPD */
		display->vsync_len +        /* VSPW */
		display->height +           /* LINEVAL */
		4;

	vclk_cnt = display->right_margin +  /* HBPD */
		display->left_margin +      /* HFPD */
		display->hsync_len +        /* HSPW */
		display->width +            /* HOZVAL */
		4;

	pixel_clk = (display->frame_rate * (vclk_cnt+1) *  hsync_cnt);
	lcd_clk = clk_get_rate(fbi->clk);
        clkval = (lcd_clk / pixel_clk) - 1;

	if (abs((lcd_clk / clkval) - pixel_clk) >
	    abs((lcd_clk / (clkval + 1)) - pixel_clk))
		clkval++;

	printk_debug("Calculated CLKVAL is 0x%x (pixel clk: %lu | lcd clk %lu)\n",
		     clkval, pixel_clk, lcd_clk);

	vidcon1 = readl(fbi->io + S3C24XX_LCD_VIDCON1);
	vidcon0 = readl(fbi->io + S3C24XX_LCD_VIDCON0);
	wincon1 = readl(fbi->io + S3C24XX_LCD_WINCON1);

	/* Configure the clock */
	vidcon0 &= ~(S3C24XX_LCD_VIDCON0_CLKSEL_MASK |
		     S3C24XX_LCD_VIDCON0_CLKVAL_MASK);
	if (!strcmp(display->clock_source, "display-if"))
		vidcon0 |= S3C24XX_LCD_VIDCON0_CLKSEL_LCD;

	vidcon0 |=	(display->vidcon0 | S3C24XX_LCD_VIDCON0_CLKVAL(clkval));
	vidcon1 |=	display->vidcon1;

	vidtcon0 =	S3C24XX_LCD_VIDTCON0_VSPW(display->vsync_len - 1) |
			S3C24XX_LCD_VIDTCON0_VFPD(display->upper_margin - 1) |
			S3C24XX_LCD_VIDTCON0_VBPD(display->lower_margin - 1);

	vidtcon1 =	S3C24XX_LCD_VIDTCON1_HSPW(display->hsync_len - 1) |
			S3C24XX_LCD_VIDTCON1_HFPD(display->left_margin - 1) |
			S3C24XX_LCD_VIDTCON1_HBPD(display->right_margin - 1);

	vidtcon2 =	S3C24XX_LCD_VIDTCON2_HOZVAL(display->width - 1) |
			S3C24XX_LCD_VIDTCON2_LINEVAL(display->height - 1);

	/* Write the user configuration too */
	wincon0	=	display->bpp_mode |
			S3C24XX_LCD_WINCON0_ENWIN_F |
			S3C24XX_LCD_WINCON0_HAWSWP |
			S3C24XX_LCD_WINCON0_16WBURST;

	vidosd0a =	0x00;
	vidosd0b =	S3C24XX_LCD_VIDOSD0B_RIGHT_X(display->width - 1) |
			S3C24XX_LCD_VIDOSD0B_RIGHT_Y(display->height - 1);

	/* And now write the configuration into the corresponding registers */
	writel(vidcon0, fbi->io + S3C24XX_LCD_VIDCON0);
	writel(vidcon1, fbi->io + S3C24XX_LCD_VIDCON1);
	writel(vidtcon0, fbi->io + S3C24XX_LCD_VIDTCON0);
	writel(vidtcon1, fbi->io + S3C24XX_LCD_VIDTCON1);
	writel(vidtcon2, fbi->io + S3C24XX_LCD_VIDTCON2);
	writel(wincon0, fbi->io + S3C24XX_LCD_WINCON0);
	writel(vidosd0a, fbi->io + S3C24XX_LCD_VIDOSD0A);
	writel(vidosd0b, fbi->io + S3C24XX_LCD_VIDOSD0B);

	/* These are the configuration addresses for the frame buffer */
	writel(info->fix.smem_start, fbi->io + S3C24XX_LCD_VIDW00ADD0B0);
	writel(info->fix.smem_start + info->fix.smem_len,
	       fbi->io + S3C24XX_LCD_VIDW00ADD1B0);

	printk_debug("vidcon0 0x%08x | vidcon1 0x%08x\n",
		     (unsigned int)vidcon0,
		     (unsigned int)vidcon1);
	printk_debug("vidtcon0 0x%08x | vidtcon1 0x%08x | vidtcon2 0x%08x\n",
		     (unsigned int)vidtcon0,
		     (unsigned int)vidtcon1,
		     (unsigned int)vidtcon2);
}

/* Write the display configuration to the hardware */
static int s3c2443fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
	case 32:
	case 16:
	case 12:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
		info->fix.visual = FB_VISUAL_MONO01;
		break;
	default:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	}


	info->fix.line_length = (var->xres_virtual * var->bits_per_pixel) / 8;

	/* activate this new configuration */
	s3c2443fb_activate_var(info);
	return 0;
}

static void s3c2443fb_lcd_enable(struct s3c2443fb_info *fbi, int enable)
{
	struct s3c2443fb_mach_info *mach_info = fbi->dev->platform_data;
	struct s3c2443fb_display *display = mach_info->display;
	unsigned long flags;
	unsigned long vidcon0;

	local_irq_save(flags);
	vidcon0 = readl(fbi->io + S3C24XX_LCD_VIDCON0);
	if (enable)
		vidcon0 |= S3C24XX_LCD_VIDCON0_ENVID | S3C24XX_LCD_VIDCON0_ENVID_F;
	else
		vidcon0 &= ~(S3C24XX_LCD_VIDCON0_ENVID | S3C24XX_LCD_VIDCON0_ENVID_F);

	writel(vidcon0, fbi->io + S3C24XX_LCD_VIDCON0);

	if (display->display_power_enable)
		display->display_power_enable(enable);

	local_irq_restore(flags);
}

static int s3c2443fb_blank(int blank_mode, struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;

	printk_debug("Blank (mode=%d, info=%p)\n", blank_mode, info);

	if (blank_mode == FB_BLANK_POWERDOWN) {
		s3c2443fb_lcd_enable(fbi, 0);
	} else {
		s3c2443fb_lcd_enable(fbi, 1);
	}

	return 0;
}

static void schedule_palette_update(struct s3c2443fb_info *fbi,
				    unsigned int regno, unsigned int val)
{
	unsigned long flags;
	unsigned long irqen;
	void __iomem *irq_base = fbi->irq_base;

	printk_debug("Calling %s\n", __func__);

	local_irq_save(flags);

	fbi->palette_buffer[regno] = val;

	if (!fbi->palette_ready) {
		fbi->palette_ready = 1;
		/* enable IRQ */
		irqen = readl(irq_base + S3C24XX_LCDINTMSK);
		irqen &= ~S3C2410_LCDINT_FRSYNC;
		writel(irqen, irq_base + S3C24XX_LCDINTMSK);
	}

	local_irq_restore(flags);
}


/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan,
				         struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int s3c2443fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;
	void __iomem *regs = fbi->io;
	unsigned int val;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseudo-palette */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red,   &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue,  &info->var.blue);

			pal[regno] = val;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
			/* currently assume RGB 5-6-5 mode */

			val  = (red   >>  0) & 0xf800;
			val |= (green >>  5) & 0x07e0;
			val |= (blue  >> 11) & 0x001f;

			writel(val, regs + S3C2410_TFTPAL(regno));
			schedule_palette_update(fbi, regno, val);
		}

		break;

	default:
		return 1;	/* unknown type */
	}

	return 0;
}

/* These are the available device operations */
static struct fb_ops s3c2443fb_tft_fbops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= s3c2443_check_var,
	.fb_set_par	= s3c2443fb_set_par,
	.fb_blank	= s3c2443fb_blank,
	.fb_setcolreg	= s3c2443fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/*
 * Allocates the DRAM memory for the frame buffer.  This buffer is
 * remapped into a non-cached, non-buffered, memory region to
 * allow palette and pixel writes to occur without flushing the
 * cache.  Once this area is remapped, all virtual memory
 * access to the video memory should occur at the new region.
 */
static int __init s3c2443fb_map_video_memory(struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;
	dma_addr_t map_dma;
	unsigned map_size = PAGE_ALIGN(info->fix.smem_len);

	info->screen_base = dma_alloc_writecombine(fbi->dev, map_size,
						   &map_dma, GFP_KERNEL);

	if (info->screen_base) {
		/* clear screen */
		memset(info->screen_base, 0x00, map_size);
		info->fix.smem_start = map_dma;
	}

	return info->screen_base ? 0 : -ENOMEM;
}

static inline void modify_gpio(void __iomem *reg,
			       unsigned long set, unsigned long mask)
{
	unsigned long tmp;

	tmp = readl(reg) & ~mask;
	writel(tmp | set, reg);
}

static int s3c2443fb_init_registers(struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;
	struct s3c2443fb_mach_info *mach_info = fbi->dev->platform_data;
	unsigned long flags;

	/* modify the gpio(s) with interrupts set (bjd) */
	local_irq_save(flags);
	/* Select display type and config the gpios */
	writel(readl(S3C24XX_MISCCR) | (1 << 28), S3C24XX_MISCCR);
	modify_gpio(S3C2410_GPCUP,  mach_info->gpcup,  mach_info->gpcup_mask);
	modify_gpio(S3C2410_GPCCON, mach_info->gpccon, mach_info->gpccon_mask);
	modify_gpio(S3C2410_GPDUP,  mach_info->gpdup,  mach_info->gpdup_mask);
	modify_gpio(S3C2410_GPDCON, mach_info->gpdcon, mach_info->gpdcon_mask);
	local_irq_restore(flags);

	return 0;
}

static inline void s3c2443fb_unmap_video_memory(struct fb_info *info)
{
	struct s3c2443fb_info *fbi = info->par;

	dma_free_writecombine(fbi->dev, PAGE_ALIGN(info->fix.smem_len),
			      info->screen_base, info->fix.smem_start);
}

int __devinit s3c2443_fb_probe(struct platform_device *pdev)
{
	struct s3c2443fb_info *info;
	struct s3c2443fb_display *display;
	struct s3c2443fb_mach_info *mach_info;
	struct fb_info *fbinfo;
	struct resource *res;
	int ret, i;
	char *option = NULL;

	/* Get the user defined LCD controller configuration */
	mach_info = pdev->dev.platform_data;
	if (mach_info == NULL) {
		dev_err(&pdev->dev, "no platform data for lcd driver\n");
		return -EINVAL;
	}

	/* Get the display information */
	if (!mach_info->num_displays) {
		dev_err(&pdev->dev, "no display information available\n");
		return -EINVAL;
	}

	if (mach_info->num_displays > 1) {
		/*
		 * If there are multiple displays, use the one specified
		 * through the command line parameter vith following
		 * format video=displayfb:<display_name>
		 */
		if (fb_get_options("displayfb", &option)) {
			dev_err(&pdev->dev, "no display information available in commnad line\n");
			return -ENODEV;
		}
		if (!option)
			return -ENODEV;

		printk_debug("display options: %s\n", option);
		for (i = 0; i < mach_info->num_displays; i++) {
			if (!strcmp(option, mach_info->displays[i].display_name))
				mach_info->display = &mach_info->displays[i];
		}
	} else {
		/* If there is only one, that is what we use */
		mach_info->display = mach_info->displays;
	}

	if ((display = mach_info->display) == NULL) {
		dev_err(&pdev->dev, "no display information available\n");
		return -EINVAL;
	}

	/* Allocate the frame buffer for this device */
	fbinfo = framebuffer_alloc(sizeof(struct s3c2443fb_info), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;

	platform_set_drvdata(pdev, fbinfo);

	/* Use the private data for our internal settings */
	info = fbinfo->par;
	info->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory registers\n");
		ret = -ENXIO;
		goto dealloc_fb;
	}
	info->mem = request_mem_region(res->start,
					res->end - res->start + 1, pdev->name);
	if (info->mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto dealloc_fb;
	}

	info->io = ioremap(res->start, res->end - res->start + 1);
	if (info->io == NULL) {
		dev_err(&pdev->dev, "ioremap of registers failed\n");
		ret = -ENXIO;
		goto release_mem;
	}

	strcpy(fbinfo->fix.id, DRV_NAME);

	/* Start the initial fix configuration */
	fbinfo->fix.type	    = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux	    = 0;
	fbinfo->fix.xpanstep	    = 0;
	fbinfo->fix.ypanstep	    = 0;
	fbinfo->fix.ywrapstep	    = 0;
	fbinfo->fix.accel	    = FB_ACCEL_NONE;

	fbinfo->var.nonstd	    = 0;
	fbinfo->var.activate	    = FB_ACTIVATE_NOW;
	fbinfo->var.accel_flags     = 0;
	fbinfo->var.vmode	    = FB_VMODE_NONINTERLACED;

	fbinfo->fbops		    = &s3c2443fb_tft_fbops;
	fbinfo->flags		    = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette      = &info->pseudo_pal;

	for (i = 0; i < 256; i++)
		info->palette_buffer[i] = PALETTE_BUFF_CLEAR;

	info->clk = clk_get(NULL, mach_info->display->clock_source);
	if (!info->clk || IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed to get lcd clock source %s\n",
			mach_info->display->clock_source);
		ret = -ENOENT;
		goto release_regs;
	}

	clk_enable(info->clk);
	msleep(1);

	/* Initialize video memory */
	fbinfo->var.xres = display->xres;
	fbinfo->var.yres = display->yres;
	fbinfo->var.bits_per_pixel = display->bpp;
	fbinfo->fix.smem_len = display->xres * display->yres * (display->bpp >> 3);

	ret = s3c2443fb_map_video_memory(fbinfo);
	if (ret) {
		dev_err(&pdev->dev, "failed to map video memory\n");
		ret = -ENOMEM;
		goto release_clock;
	}

	s3c2443fb_init_registers(fbinfo);

	s3c2443_check_var(&fbinfo->var, fbinfo);

	ret = register_framebuffer(fbinfo);
	if (ret) {
		dev_err(&pdev->dev, "failed to register framebuffer device\n");
		goto free_video_memory;
	}

	s3c2443fb_lcd_enable(info, 1);

	printk(KERN_INFO DRV_NAME ": frame buffer fb%d: %s, display config %s\n",
	       fbinfo->node, fbinfo->fix.id, mach_info->display->display_name);
	return 0;

free_video_memory:
	s3c2443fb_unmap_video_memory(fbinfo);

release_clock:
	clk_disable(info->clk);
	clk_put(info->clk);

release_regs:
	iounmap(info->io);

release_mem:
	release_resource(info->mem);
	kfree(info->mem);

dealloc_fb:
	platform_set_drvdata(pdev, NULL);
	framebuffer_release(fbinfo);

	return ret;
}

static int s3c2443_fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct s3c2443fb_info *info = fbinfo->par;

	fbinfo = platform_get_drvdata(pdev);
	info = fbinfo->par;

	s3c2443fb_lcd_enable(info, 0);
	unregister_framebuffer(fbinfo);
	s3c2443fb_unmap_video_memory(fbinfo);

	if (info->clk) {
		clk_disable(info->clk);
		clk_put(info->clk);
		info->clk = NULL;
	}

	iounmap(info->io);
	release_resource(info->mem);
	kfree(info->mem);
	platform_set_drvdata(pdev, NULL);
	framebuffer_release(fbinfo);

	return 0;
}

#ifdef CONFIG_PM
/* suspend and resume support for the lcd controller */
static int s3c2443_fb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info *fbinfo = platform_get_drvdata(dev);
	struct s3c2443fb_info *info = fbinfo->par;

	if (state.event == PM_EVENT_SUSPEND) {
		s3c2443fb_lcd_enable(info, 0);
		/* sleep before disabling the clock, we need to ensure
		* the LCD DMA engine is not going to get back on the bus
		* before the clock goes off again (bjd) */
		msleep(1);
		clk_disable(info->clk);
	}

	return 0;
}

static int s3c2443_fb_resume(struct platform_device *dev)
{
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct s3c2443fb_info *info = fbinfo->par;

	clk_enable(info->clk);
	msleep(1);
	s3c2443fb_init_registers(fbinfo);
	s3c2443_check_var(&fbinfo->var, fbinfo);
	s3c2443fb_activate_var(fbinfo);
	s3c2443fb_lcd_enable(info, 1);

	return 0;
}
#else
#define s3c2443_fb_suspend NULL
#define s3c2443_fb_resume  NULL
#endif

static struct platform_driver s3c2443_fb_driver = {
	.probe		= s3c2443_fb_probe,
	.remove		= s3c2443_fb_remove,
	.suspend	= s3c2443_fb_suspend,
	.resume		= s3c2443_fb_resume,
        .driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

int __devinit s3c2443_fb_init(void)
{
	return platform_driver_register(&s3c2443_fb_driver);
}

static void __exit s3c2443_fb_cleanup(void)
{
	platform_driver_unregister(&s3c2443_fb_driver);
}

module_init(s3c2443_fb_init);
module_exit(s3c2443_fb_cleanup);

MODULE_AUTHOR("Luis Galdos & Pedro Perez de Heredia");
MODULE_DESCRIPTION("Framebuffer driver for the S3C2443");
MODULE_LICENSE("GPL");

