/*
 * drivers/video/ns9360fb.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/regs-sys-ns9360.h>
#include <mach/ns9360fb.h>
#include <mach/gpio.h>

#define LCD_TIMING(x)	((x) * 4)
#define LCD_TIMING2_BCD		(1 << 26)
#define LCD_TIMING2_PCD(x)	(((x) & 0x1F))
#define LCD_UPBASE	0x10
#define LCD_CONTROL	0x1c

#define DRIVER_NAME	"ns9360fb"

#define fb2nfb(fbi) container_of(fbi, struct ns9360fb_info, fb)
struct ns9360fb_info {
	struct fb_info		fb;

	struct resource		*mem;
	struct clk		*clk;
	struct ns9360fb_pdata	*pdata;

	void __iomem		*ioaddr;

	dma_addr_t		fb_p;
	unsigned char		*fb_v;
	unsigned int		fb_size;

	u32			pseudo_pal[16];
};

static void ns9360fb_select_video_clk_src(int source)
{
	u32 reset, clock;

	reset = __raw_readl(SYS_RESET) & ~SYS_RESET_LCDC;
	__raw_writel(reset, SYS_RESET);

	clock = __raw_readl(SYS_CLOCK) & ~SYS_CLOCK_LPCSEXT;
	if (!source) {
		/* Use external clock */
		if (!gpio_request(15, "ns9360fb-extclk")) {
			gpio_configure_ns9360(15, 0, 0, 2);
			clock |= SYS_CLOCK_LPCSEXT;
		} else {
			printk(KERN_ERR DRIVER_NAME
			       ": unable to request gpio for external video clock\n");
		}
	}
	__raw_writel(clock, SYS_CLOCK);

	reset = __raw_readl(SYS_RESET) | SYS_RESET_LCDC;
	__raw_writel(reset, SYS_RESET);
}

static int ns9360fb_set_par(struct fb_info *fb)
{
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = fb->var.width * fb->var.bits_per_pixel / 8;

	return 0;
}

static int ns9360fb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *fb)
{
	struct ns9360fb_info *info = fb2nfb(fb);

	if (regno > info->fb.var.bits_per_pixel)
		return -EINVAL;

	info->pseudo_pal[regno] = (red & 0xF800) |
	       ((green & 0xF800) >> 5) | ((green & 0xF800) >> 10);

	return 0;
}

static struct fb_ops ns9360fb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = ns9360fb_set_par,
	.fb_setcolreg = ns9360fb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int __devinit ns9360fb_probe(struct platform_device *pdev)
{
	struct ns9360fb_info *info;
	struct ns9360fb_display *display;
	char *option = NULL;
	int ret, i;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_dbg(&pdev->dev, "%s: err_alloc_info\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_info;
	}
	platform_set_drvdata(pdev, info);

	info->pdata = pdev->dev.platform_data;
	if (!info->pdata) {
		dev_dbg(&pdev->dev, "%s: err_pdata\n", __func__);
		ret = -ENOENT;
		goto err_pdata;
	}

	/* Get the display information */
	if (!info->pdata->num_displays) {
		dev_err(&pdev->dev, "no display information available\n");
		ret = -EINVAL;
		goto err_pdata;
	}

	if (info->pdata->num_displays > 1) {
		/* 
		 * If there are multiple displays, use the one specified
		 * through the command line parameter vith following 
		 * format video=displayfb:<display_name>
		 */
		if (fb_get_options("displayfb", &option)) {
			dev_err(&pdev->dev, 
				"no display information available in commnad line\n");
			ret = -ENODEV;
			goto err_pdata;
		}
		if (!option) {
			ret = -ENODEV;
			goto err_pdata;
		}
		dev_info(&pdev->dev, "display options: %s\n", option);

		for (i = 0; i < info->pdata->num_displays; i++) {
			if (!strcmp(option, info->pdata->displays[i].display_name))
				info->pdata->display = &info->pdata->displays[i];
		}
	} else {
		/* If there is only one, that is what we use */
		info->pdata->display = info->pdata->displays;
	}
  
	if ((display = info->pdata->display) == NULL) {
		dev_err(&pdev->dev, "no display information available\n");
		ret = -ENODEV;
		goto err_pdata;
	}

	info->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!info->mem) {
		dev_dbg(&pdev->dev, "%s: err_mem\n", __func__);
		ret = -ENOENT;
		goto err_mem;
	}

	if (!request_mem_region(info->mem->start,
				info->mem->end - info->mem->start + 1,
				DRIVER_NAME)) {
		dev_dbg(&pdev->dev, "%s: err_req_mem\n", __func__);
		ret = -EBUSY;
		goto err_req_mem;
	}

	info->ioaddr = ioremap(info->mem->start,
			info->mem->end - info->mem->start + 1);
	if (info->ioaddr <= 0) {
		dev_dbg(&pdev->dev, "%s: err_map_mem\n", __func__);
		ret = -EBUSY;
		goto err_map_mem;
	}

	strcpy(info->fb.fix.id, DRIVER_NAME);
	info->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	info->fb.fix.accel = FB_ACCEL_NONE;

	info->fb.flags = FBINFO_FLAG_DEFAULT;
	info->fb.fbops = &ns9360fb_ops;
	info->fb.pseudo_palette = &info->pseudo_pal;

	info->fb.var.activate = FB_ACTIVATE_NOW;
	info->fb.var.height = display->height;
	info->fb.var.yres = display->height;
	info->fb.var.yres_virtual = display->height;
	info->fb.var.width = display->width;
	info->fb.var.xres = display->width;
	info->fb.var.xres_virtual = display->width;
	info->fb.var.vmode = FB_VMODE_NONINTERLACED;
	info->fb.var.bits_per_pixel = 16;

	info->fb.var.red.offset = 10;
	info->fb.var.green.offset = 5;
	info->fb.var.blue.offset = 0;
	info->fb.var.red.length = 5;
	info->fb.var.green.length = 5;
	info->fb.var.blue.length = 5;

	info->fb.fix.smem_len = info->fb.var.xres * info->fb.var.yres *
		info->fb.var.bits_per_pixel / 8;

	pdev->dev.coherent_dma_mask = (u32)-1;

	info->fb_size = PAGE_ALIGN(info->fb.fix.smem_len + PAGE_SIZE);
	info->fb_v = dma_alloc_writecombine(&pdev->dev, info->fb_size,
			&info->fb_p, GFP_KERNEL);
	if (!info->fb_v) {
		dev_dbg(&pdev->dev, "%s: err_alloc_fb\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_fb;
	}

	info->fb.screen_base = info->fb_v + PAGE_SIZE;
	info->fb.fix.smem_start = info->fb_p + PAGE_SIZE;

	info->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(info->clk)) {
		dev_dbg(&pdev->dev, "%s: err_clk_get\n", __func__);
		ret = PTR_ERR(info->clk);
		goto err_clk_get;
	}

	memset(info->fb.screen_base, 0, info->fb.fix.smem_len);

	ret = clk_enable(info->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_clk_enable\n", __func__);
		goto err_clk_enable;
	}

	/* display->clock = 0 means use external source */
	ns9360fb_select_video_clk_src(display->clock);

	if (!(display->timing[2] & LCD_TIMING2_BCD)) {
		display->timing[2] |=
			LCD_TIMING2_PCD((clk_get_rate(info->clk) /
					display->clock) - 1);
	}

	/* write configuration to video controller */
	for (i = 0; i < 4; i++)
		iowrite32(display->timing[i],
				info->ioaddr + LCD_TIMING(i));

	iowrite32(info->fb.fix.smem_start, info->ioaddr + LCD_UPBASE);
	iowrite32(display->control, info->ioaddr + LCD_CONTROL);

	ret = register_framebuffer(&info->fb);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_reg_fb\n", __func__);
		goto err_reg_fb;
	}

	dev_info(&pdev->dev, "fb mapped to 0x%p (0x%p), display config %s\n",
			(void *)info->fb.fix.smem_start, info->fb.screen_base,
			display->display_name);

	return 0;

err_reg_fb:
	clk_disable(info->clk);
err_clk_enable:
	clk_put(info->clk);
err_clk_get:
	dma_free_writecombine(&pdev->dev, info->fb_size,
			info->fb_v, info->fb_p);
err_alloc_fb:
	iounmap(info->ioaddr);
err_map_mem:
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);
err_req_mem:
err_mem:
err_pdata:
	kfree(info);
err_alloc_info:

	return ret;
}

static int ns9360fb_remove(struct platform_device *pdev)
{
	struct ns9360fb_info *info = platform_get_drvdata(pdev);

	clk_disable(info->clk);
	clk_put(info->clk);

	dma_free_writecombine(&pdev->dev, info->fb_size,
			info->fb_v, info->fb_p);

	iounmap(info->ioaddr);
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);

	kfree(info);
	return 0;
}

static struct platform_driver ns9360fb_driver = {
	.probe		= ns9360fb_probe,
	.remove		= ns9360fb_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __devinit ns9360fb_init(void)
{
	return platform_driver_register(&ns9360fb_driver);
}

static void __exit ns9360fb_cleanup(void)
{
	platform_driver_unregister(&ns9360fb_driver);
}

module_init(ns9360fb_init);
module_exit(ns9360fb_cleanup);

MODULE_AUTHOR("Matthias Ludwig");
MODULE_DESCRIPTION("Driver for Digi ns9360 SoC framebuffer");
MODULE_LICENSE("GPL v2");
