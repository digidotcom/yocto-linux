/* -*- linux-c -*-
 * 
 * linux/drivers/video/s3cfb.c
 *
 * $Id: s3cfb.c,v 1.63 2007/07/12 05:26:04 yreom Exp $
 *
 * Revision 1.16  2006/09/14 04:45:15  ihlee215
 * OSD support added
 *
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.c
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
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <linux/math64.h>

#include <asm/mach/map.h>
/* #include <asm/arch/registers.h> */
#include <mach/idle.h>
#include <mach/fb.h>
#include <mach/regs-gpio.h>

#include "s3c2410fb.h"


#ifdef CONFIG_PM
#include <linux/pm.h>
#endif




#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] s3cfb: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "[ INFO ] s3cfb: " fmt, ## args)


#if 1
#define S3CFB_DEBUG
#endif

#ifdef S3CFB_DEBUG
#  define printk_debug(fmt, args...)            printk(KERN_DEBUG "s3cfb: " fmt, ## args)
#else
#  define printk_debug(fmt, args...)
#endif



#define S3CFB_DRIVER_NAME			"s3c2410fb-tft"


static int s3c2410fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;
	struct s3c2410fb_mach_info *mach_info = fbi->dev->platform_data;
	struct s3c2410fb_display *display = NULL;
	struct s3c2410fb_display *default_display = mach_info->displays +
		mach_info->default_display;
	int type = default_display->type;
	unsigned i;

	printk_debug("Calling %s: var=%p, info=%p\n", __func__, var, info);

	/* validate x/y resolution */
	/* choose default mode if possible */
	if (var->yres == default_display->yres &&
	    var->xres == default_display->xres &&
	    var->bits_per_pixel == default_display->bpp)
		display = default_display;
	else
		for (i = 0; i < mach_info->num_displays; i++)
			if (type == mach_info->displays[i].type &&
			    var->yres == mach_info->displays[i].yres &&
			    var->xres == mach_info->displays[i].xres &&
			    var->bits_per_pixel == mach_info->displays[i].bpp) {
				display = mach_info->displays + i;
				break;
			}

	if (!display) {
		printk_err("wrong resolution or depth %dx%d at %d bpp\n",
			   var->xres, var->yres, var->bits_per_pixel);
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

	/*
	 * If using the Power Signal then request the GPIO
	 * (@XXX: Use the correct request and configuration function for the GPIO)
	 * Luis Galdos
	 */
	if (display->lcdcon5 & S3C2410_LCDCON5_PWREN) {
		printk_info("Configuring the power LED GPIO\n");
		s3c2410_gpio_cfgpin(S3C2410_GPG4, S3C2410_GPG4_LCDPWREN);
	}
	
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
		if (S3C24XX_LCD_WINCON_BPP(display->wincon0) ==
		    S3C24XX_LCD_WINCON_16BPP_565) {
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


#if 0
static void s3cfb_dummy_values(struct s3c2410fb_info *fbi)
{

	writel(0x10415, fbi->io + S3C24XX_LCD_WINCON0);
	writel(0x00, fbi->io + S3C24XX_LCD_WINCON1);
	
	writel(0x173, fbi->io + S3C24XX_LCD_VIDCON0);
	writel(0x195D060, fbi->io + S3C24XX_LCD_VIDCON1);
	
	writel(0x1F0203, fbi->io + S3C24XX_LCD_VIDTCON0);
	writel(0x40618, fbi->io + S3C24XX_LCD_VIDTCON1);
	writel(0xEFA7F, fbi->io + S3C24XX_LCD_VIDTCON2);
	
	writel(0x00, fbi->io + S3C24XX_LCD_VIDOSD0A);
	writel(0x13F9DF, fbi->io + S3C24XX_LCD_VIDOSD0B);
}
#endif /* 0: Only for testing */


/* Write into the register the passed display configuration */
static void s3c2410fb_activate_var(struct fb_info *info)
{
	struct s3c2410fb_info *fbi;
	unsigned short hsync_cnt, vclk_cnt;
	unsigned char clkval;
	struct s3c2410fb_display *display;
	struct s3c2410fb_mach_info *mach_info;
	unsigned long pixel_clk, lcd_clk;
	unsigned long vidcon0, vidcon1, vidtcon0, vidtcon1, vidtcon2;
	unsigned long wincon0, wincon1;
	unsigned long vidosd0a, vidosd0b;
	
	printk_debug("Calling %s\n", __func__);
	
	fbi = info->par;
	mach_info = fbi->dev->platform_data;
	display = mach_info->displays + mach_info->default_display;
	
	/* Write the configuration into the register VIDCON0 */
	hsync_cnt = display->lower_margin + /* VBPD */
		display->upper_margin +     /* VFPD */
		display->vsync_len +        /* VSPW */
		display->height;            /* LINEVAL */

	vclk_cnt = display->right_margin +  /* HBPD */
		display->left_margin +      /* HFPD */
		display->hsync_len +        /* HSPW */
		display->width;             /* HOZVAL */

	pixel_clk = (display->frame_rate * vclk_cnt *  hsync_cnt);
	lcd_clk = clk_get_rate(fbi->clk);

	/*
	 * @FIXME: The U-Boot has another clock calculation. See under:
	 * cpu/s3c24xx/s3c2443/fb.c
	 */
        clkval = (lcd_clk / pixel_clk) - 1;
	printk_debug("Calculated CLKVAL is 0x%x (pixel clk: %lu | lcd clk %lu)\n",
		     clkval, pixel_clk, lcd_clk);
	
	vidcon1 = readl(fbi->io + S3C24XX_LCD_VIDCON1);
	vidcon0 = readl(fbi->io + S3C24XX_LCD_VIDCON0);
	wincon1 = readl(fbi->io + S3C24XX_LCD_WINCON1);
	
	/* Configure the clock */
	vidcon0 |= (display->vidcon0 | S3C24XX_LCD_VIDCON0_CLKVAL(clkval));

	vidcon1 |= display->vidcon1;
	
	vidtcon0 = S3C24XX_LCD_VIDTCON0_VSPW(display->vsync_len - 1) |
		S3C24XX_LCD_VIDTCON0_VFPD(display->upper_margin - 1) |
		S3C24XX_LCD_VIDTCON0_VBPD(display->lower_margin - 1);

	vidtcon1 = S3C24XX_LCD_VIDTCON1_HSPW(display->hsync_len - 1) |
		S3C24XX_LCD_VIDTCON1_HFPD(display->left_margin - 1) |
		S3C24XX_LCD_VIDTCON1_HBPD(display->right_margin - 1);

	
	vidtcon2 = S3C24XX_LCD_VIDTCON2_HOZVAL(display->width - 1) |
		S3C24XX_LCD_VIDTCON2_LINEVAL(display->height - 1);

	/* Write the user configuration too */
	wincon0 = (display->wincon0 |
		   S3C24XX_LCD_WINCON0_ENWIN_F |
		   S3C24XX_LCD_WINCON0_HAWSWP |
		   S3C24XX_LCD_WINCON0_4WBURST);

	
	vidosd0a = 0x00;
	vidosd0b = S3C24XX_LCD_VIDOSD0B_RIGHT_X(display->width - 1) |
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
static int s3c2410fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	printk_debug("Calling %s\n", __func__);

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
	printk_debug("Line length is %i\n", info->fix.line_length);

	
	/* activate this new configuration */
	s3c2410fb_activate_var(info);
	return 0;
}


static void s3c2410fb_lcd_enable(struct s3c2410fb_info *fbi, int enable)
{
	unsigned long flags;
	unsigned long vidcon0;
	
	local_irq_save(flags);

	printk_debug("%s the TFT LCD\n", enable ? "Enabling" : "Disabling");

	vidcon0 = readl(fbi->io + S3C24XX_LCD_VIDCON0);
	if (enable)
		vidcon0 |= (S3C24XX_LCD_VIDCON0_ENVID | S3C24XX_LCD_VIDCON0_ENVID_F);
	else
		vidcon0 &= ~(S3C24XX_LCD_VIDCON0_ENVID | S3C24XX_LCD_VIDCON0_ENVID_F);

	writel(vidcon0, fbi->io + S3C24XX_LCD_VIDCON0);
	local_irq_restore(flags);
}



static int s3c2410fb_blank(int blank_mode, struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;

	printk_debug("Blank (mode=%d, info=%p, power down=%i)\n",
		     blank_mode, info, FB_BLANK_POWERDOWN);

	if (blank_mode == FB_BLANK_POWERDOWN) {
		s3c2410fb_lcd_enable(fbi, 0);
	} else {
		s3c2410fb_lcd_enable(fbi, 1);
	}

	return 0;
}

static void schedule_palette_update(struct s3c2410fb_info *fbi,
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



static int s3c2410fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;
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



/* Function called when the frame buffer device (/dev/fbX) is being opened */
static int s3c2410fb_tft_open(struct fb_info *info, int user)
{
	struct s3c2410fb_info *fbi;

	fbi = info->par;
	
	printk_debug("Calling %s\n", __func__);
	s3c2410fb_lcd_enable(fbi, 1);
	return 0;
}


static int s3c2410fb_tft_release(struct fb_info *info, int user)
{
	struct s3c2410fb_info *fbi;

	fbi = info->par;
	printk_debug("Calling %s\n", __func__);
	s3c2410fb_lcd_enable(fbi, 0);
	return 0;
}


static int s3c2410fb_tft_pan(struct fb_var_screeninfo *var, struct fb_info *info)
{



	return 0;
}

/* These are the available device operations */
static struct fb_ops s3cfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= s3c2410fb_check_var,
	.fb_set_par	= s3c2410fb_set_par,
	.fb_blank	= s3c2410fb_blank,
	.fb_setcolreg	= s3c2410fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_open	= s3c2410fb_tft_open,
	.fb_release	= s3c2410fb_tft_release,
	.fb_pan_display = s3c2410fb_tft_pan,
};



static irqreturn_t s3cfb_irq(int irq, void *dev_id)
{
	printk_info("IRQ of the TFT LCD enabled?\n");
/* 	struct s3c2410fb_info *fbi = dev_id; */
/* 	void __iomem *irq_base = fbi->irq_base; */
/* 	unsigned long lcdirq = readl(irq_base + S3C24XX_LCDINTPND); */
  
/* 	if (lcdirq & S3C2410_LCDINT_FRSYNC) { */
/* 		if (fbi->palette_ready) */
/* 			s3c2410fb_write_palette(fbi); */

/* 		writel(S3C2410_LCDINT_FRSYNC, irq_base + S3C24XX_LCDINTPND); */
/* 		writel(S3C2410_LCDINT_FRSYNC, irq_base + S3C24XX_LCDSRCPND); */
/* 	} */

	return IRQ_HANDLED;
}


/*
 * s3c2410fb_map_video_memory():
 *	Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *	allow palette and pixel writes to occur without flushing the
 *	cache.  Once this area is remapped, all virtual memory
 *	access to the video memory should occur at the new region.
 */
static int __init s3c2410fb_map_video_memory(struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;
	dma_addr_t map_dma;
	unsigned map_size = PAGE_ALIGN(info->fix.smem_len);

	printk_debug("map_video_memory(fbi=%p) map_size %u\n", fbi, map_size);

	info->screen_base = dma_alloc_writecombine(fbi->dev, map_size,
						   &map_dma, GFP_KERNEL);

	if (info->screen_base) {
		/* prevent initial garbage on screen */
		printk_debug("map_video_memory: clear %p:%08x\n",
			     info->screen_base, map_size);
		memset(info->screen_base, 0x00, map_size);

		info->fix.smem_start = map_dma;
		info->fix.smem_len = map_size;

		printk_debug("map_video_memory: dma=%08lx cpu=%p size=%08x\n",
			     info->fix.smem_start, info->screen_base, map_size);
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



static int s3c2410fb_init_registers(struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;
	struct s3c2410fb_mach_info *mach_info = fbi->dev->platform_data;
	unsigned long flags;

	/* Initialise LCD with values from haret */

	local_irq_save(flags);

	/* modify the gpio(s) with interrupts set (bjd) */

	modify_gpio(S3C2410_GPCUP,  mach_info->gpcup,  mach_info->gpcup_mask);
	modify_gpio(S3C2410_GPCCON, mach_info->gpccon, mach_info->gpccon_mask);
	modify_gpio(S3C2410_GPDUP,  mach_info->gpdup,  mach_info->gpdup_mask);
	modify_gpio(S3C2410_GPDCON, mach_info->gpdcon, mach_info->gpdcon_mask);

	local_irq_restore(flags);

/* 	printk_debug("LPCSEL    = 0x%08lx\n", mach_info->lpcsel); */
/* 	writel(mach_info->lpcsel, lpcsel); */

/* 	printk_debug("replacing TPAL %08x\n", readl(tpal)); */

	/* ensure temporary palette disabled */
/* 	writel(0x00, tpal); */

	return 0;
}



static inline void s3c2410fb_unmap_video_memory(struct fb_info *info)
{
	struct s3c2410fb_info *fbi = info->par;

	dma_free_writecombine(fbi->dev, PAGE_ALIGN(info->fix.smem_len),
			      info->screen_base, info->fix.smem_start);
}



int __init s3c_fb_probe(struct platform_device *pdev)
{
	struct s3c2410fb_info *info;
        struct s3c2410fb_display *display;
	struct fb_info *fbinfo;
	struct s3c2410fb_mach_info *mach_info;
	struct resource *res;
	int ret;
	int irq;
        int i;
        int size;
        u32 vidcon0;

	/* @XXX: Use a higher probe for setting the type */
	enum s3c_drv_type drv_type = DRV_S3C2410;

	printk_debug("Probing a new device ID %i\n", pdev->id);

	
	/* Get the user defined LCD controller configuration */
	mach_info = pdev->dev.platform_data;
	if (mach_info == NULL) {
                printk_err("No platform data for lcd, cannot attach\n");
                return -EINVAL;
        }

	if (mach_info->default_display >= mach_info->num_displays) {
                printk_err("Default is %d but only %d displays\n",
			   mach_info->default_display, mach_info->num_displays);
                return -EINVAL;
        }

	/* Get the user defined display configuration */
        display = mach_info->displays + mach_info->default_display;

	/* Get the IRQ for the display controller */
        irq = platform_get_irq(pdev, 0);
        if (irq < 0) {
                printk_err("No irq for device.\n");
                return -ENOENT;
        }

	/* Allocate the frame buffer for this device */
	fbinfo = framebuffer_alloc(sizeof(struct s3c2410fb_info), &pdev->dev);
        if (!fbinfo)
                return -ENOMEM;

        platform_set_drvdata(pdev, fbinfo);

	/* Use the private data for our internal settings */
        info = fbinfo->par;	
        info->dev = &pdev->dev;
        info->drv_type = drv_type;

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (res == NULL) {
                dev_err(&pdev->dev, "failed to get memory registers\n");
                ret = -ENXIO;
                goto dealloc_fb;
        }

        size = (res->end - res->start) + 1;
        info->mem = request_mem_region(res->start, size, pdev->name);
        if (info->mem == NULL) {
                dev_err(&pdev->dev, "failed to get memory region\n");
                ret = -ENOENT;
                goto dealloc_fb;
        }

        info->io = ioremap(res->start, size);
        if (info->io == NULL) {
                dev_err(&pdev->dev, "ioremap() of registers failed\n");
                ret = -ENXIO;
                goto release_mem;
        }

	info->irq_base = info->io +
		((drv_type == DRV_S3C2412) ? S3C2412_LCDINTBASE : S3C2410_LCDINTBASE);

	/* @XXX: Do we really need the below code? */
	strcpy(fbinfo->fix.id, S3CFB_DRIVER_NAME);

        /* Stop the video first */
        vidcon0 = readl(info->io + S3C24XX_LCD_VIDCON0);
	vidcon0 &= (S3C24XX_LCD_VIDCON0_ENVID | S3C24XX_LCD_VIDCON0_ENVID_F);
	writel(vidcon0, info->io + S3C24XX_LCD_VIDCON0);

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

	fbinfo->fbops		    = &s3cfb_ops;
	fbinfo->flags		    = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette      = &info->pseudo_pal;

	for (i = 0; i < 256; i++)
		info->palette_buffer[i] = PALETTE_BUFF_CLEAR;

	/* Request our IRQ */
	ret = request_irq(irq, s3cfb_irq, IRQF_DISABLED, pdev->name, info);
	if (ret) {
		printk_err("Cannot get irq %d - err %d\n", irq, ret);
		ret = -EBUSY;
		goto release_regs;
	}

	/*
	 * We are using the HCLK, then the U-Boot is using this clock too
	 */
	if (display->vidcon0 & S3C24XX_LCD_VIDCON0_CLKSEL_LCD) {
		printk_err("Invalid clock (only tested with the HCLK).\n");
		ret = -EINVAL;
		goto release_irq;
	}

	/* @FIXME: Select the clock depending on the passed display configuration */
	info->clk = clk_get(NULL, "hclk");
	if (!info->clk || IS_ERR(info->clk)) {
		printk_err("Failed to get lcd clock source\n");
		ret = -ENOENT;
		goto release_irq;
	}

	clk_enable(info->clk);
	printk_debug("Got and enabled the HCLK clock\n");

	msleep(1);

	/* find maximum required memory size for display */
	for (i = 0; i < mach_info->num_displays; i++) {
		unsigned long smem_len = mach_info->displays[i].xres;

		smem_len *= mach_info->displays[i].yres;
		smem_len *= mach_info->displays[i].bpp;
		smem_len >>= 3;
		if (fbinfo->fix.smem_len < smem_len)
			fbinfo->fix.smem_len = smem_len;
	}

	/* Initialize video memory */
	ret = s3c2410fb_map_video_memory(fbinfo);
	if (ret) {
		printk_err("Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto release_clock;
	}


	fbinfo->var.xres = display->xres;
	fbinfo->var.yres = display->yres;
	fbinfo->var.bits_per_pixel = display->bpp;


	s3c2410fb_init_registers(fbinfo);


	s3c2410fb_check_var(&fbinfo->var, fbinfo);
	

	ret = register_framebuffer(fbinfo);
	if (ret) {
		printk_err("Failed to register the frambuffer device, %i\n", ret);
		goto free_video_memory;
	}

	
	printk_info("New frame buffer fb%d: %s frame buffer device\n",
		    fbinfo->node, fbinfo->fix.id);
	return 0;

 free_video_memory:
	s3c2410fb_unmap_video_memory(fbinfo);

 release_clock:
	clk_disable(info->clk);
	clk_put(info->clk);

 release_irq:
	free_irq(irq, info);
	
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

/* s3c_fb_stop_lcd
 *
 * shutdown the lcd controller
 */
void s3c_fb_stop_lcd(void)
{
/* 	unsigned long flags; */
/* 	unsigned long tmp; */
/* 	printk("s3c_fb_stop_lcd() called.\n"); */

/* 	local_irq_save(flags); */

/* #if defined  (CONFIG_FB_LTV350QV ) ||  defined  (CONFIG_FB_LMS430WQ ) || defined(CONFIG_ARCH_S3C64XX) */
/* 	tmp = __raw_readl(S3C_VIDCON0); */
/* 	__raw_writel(tmp & ~(ENVID|ENVID_F), S3C_VIDCON0); */
/* #else */
/* 	tmp = __raw_readl(S3C_LCDCON1); */
/* 	__raw_writel(tmp & ~(ENVID|ENVID_F), S3C_LCDCON1); */
/* #endif */
/* 	local_irq_restore(flags); */
}


void s3c_fb_start_lcd(void) {
/* 	unsigned long flags; */
/* 	unsigned long tmp; */
/* 	printk("s3c_fb_start_lcd() called.\n"); */

/* 	local_irq_save(flags); */

/* #if defined  (CONFIG_FB_LTV350QV ) || defined  (CONFIG_FB_LMS430WQ ) || defined(CONFIG_ARCH_S3C64XX) */
/* 	tmp = __raw_readl(S3C_VIDCON0); */
/* 	__raw_writel(tmp | ENVID | ENVID_F, S3C_VIDCON0); */
/* #else */
/* 	tmp = __raw_readl(S3C_LCDCON1); */
/* 	__raw_writel(tmp | ENVID | ENVID_F, S3C_LCDCON1); */
/* #endif */
/* 	local_irq_restore(flags);	 */
}

/*
 *  Cleanup
 */
static int s3c_fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo;
	struct s3c2410fb_info *info;
	int irq;

	fbinfo = platform_get_drvdata(pdev);
	info = fbinfo->par;

	unregister_framebuffer(fbinfo);

	/* Free the requested clock */
	if (info->clk) {
		clk_disable(info->clk);
		clk_put(info->clk);
		info->clk = NULL;
	}
	
	/* Free the requested IRQ */
	irq = platform_get_irq(pdev, 0);
	free_irq(irq, info);

	/* Unamp que requested memory mapped registers */
	iounmap(info->io);

	
	release_resource(info->mem);
	kfree(info->mem);

	platform_set_drvdata(pdev, NULL);

	framebuffer_release(fbinfo);
	
/* 	struct s3c_fb_info *info = fbinfo->par; */
/* 	int irq; */
/* 	int index=0; */

/* 	s3c_fb_stop_lcd(); */
/* 	msleep(1); */

/* 	for(index=0; index<S3C_FB_NUM; index++){  */
/* 		s3c_fb_unmap_video_memory((struct s3c_fb_info *)&info[index]); */

/* 	 	if (lcd_clock) { */
/* 	 		clk_disable(lcd_clock); */
/* 	 		clk_put(lcd_clock); */
/* 	 		lcd_clock = NULL; */
/* 		} */

/* 		irq = platform_get_irq(pdev, 0); */
/* 		free_irq(irq,&info[index]); */
/* 		release_mem_region((unsigned long)S3C_VA_LCD, S3C_SZ_LCD); */
/* 		unregister_framebuffer(&(info[index].fb)); */
/* 	} // for(index=0; index<CONFIG_FB_NUM; index++) */
	return 0;
}

#ifdef CONFIG_PM
/* suspend and resume support for the lcd controller */
static int s3c_fb_suspend(struct platform_device *dev, pm_message_t state)
{
/* 	s3c_fb_stop_lcd(); */

/* 	/\* sleep before disabling the clock, we need to ensure */
/* 	 * the LCD DMA engine is not going to get back on the bus */
/* 	 * before the clock goes off again (bjd) *\/ */

/* 	msleep(1); */
/* 	clk_disable(lcd_clock); */

	return 0;
}

static int s3c_fb_resume(struct platform_device *dev)
{
/* 	clk_enable(lcd_clock); */
/* 	msleep(1); */

/*         Init_LDI(); */
/* #if 0 */
/* 	for(int index=0; index<S3C_FB_NUM; index++)  */
/* 		s3c_fb_init_registers(&info[index]); */
/* #endif */

	return 0;
}

#else
#define s3c_fb_suspend NULL
#define s3c_fb_resume  NULL
#endif

static struct platform_driver s3c_fb_driver = {
	.probe		= s3c_fb_probe,
	.remove		= s3c_fb_remove,
	.suspend	= s3c_fb_suspend,
	.resume		= s3c_fb_resume,
        .driver		= {
		.name	= S3CFB_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};


int __devinit s3c_fb_init(void)
{
	return platform_driver_register(&s3c_fb_driver);
}


static void __exit s3c_fb_cleanup(void)
{
	platform_driver_unregister(&s3c_fb_driver);
}

EXPORT_SYMBOL(s3c_fb_stop_lcd);
EXPORT_SYMBOL(s3c_fb_start_lcd);

module_init(s3c_fb_init);
module_exit(s3c_fb_cleanup);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Framebuffer driver for the S3C");
MODULE_LICENSE("GPL");
