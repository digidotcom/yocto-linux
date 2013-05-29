/* arch/arm/mach-s3c2410/include/mach/fb.h
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 * Inspired by pxafb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_FB_H
#define __ASM_ARM_FB_H

#include <mach/regs-lcd.h>

struct s3c2410fb_hw {
	unsigned long	lcdcon1;
	unsigned long	lcdcon2;
	unsigned long	lcdcon3;
	unsigned long	lcdcon4;
	unsigned long	lcdcon5;
};

/* LCD description */
struct s3c2410fb_display {
	/* LCD type */
	unsigned type;

	/* Screen size */
	unsigned short width;
	unsigned short height;

	/* Screen info */
	unsigned short xres;
	unsigned short yres;
	unsigned short bpp;

	unsigned pixclock;		/* pixclock in picoseconds */
	unsigned short left_margin;  /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short right_margin; /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short hsync_len;    /* value in pixels (TFT) or HCLKs (STN) */
	unsigned short upper_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short lower_margin;	/* value in lines (TFT) or 0 (STN) */
	unsigned short vsync_len;	/* value in lines (TFT) or 0 (STN) */

	/* lcd configuration registers */
	unsigned long	lcdcon5;

        /* Additional registers for the TFT-LCD (Luis Galdos) */
        int frame_rate;
        unsigned long vidcon0;
        unsigned long vidcon1;
        unsigned long vidtcon0;
        unsigned long vidtcon1;
        unsigned long vidtcon2;
        unsigned long wincon0;
        unsigned long wincon1;
        unsigned long vidosd0a;
        unsigned long vidosd0b;
        unsigned long vidosd0c;
        unsigned long vidosd1a;
        unsigned long vidosd1b;
        unsigned long vidosd1c;

        unsigned long vidw00add0b0;
        unsigned long vidw00add0b1;
        unsigned long vidw01add0;

        unsigned long vidw00add1b0;
        unsigned long vidw00add1b1;
        unsigned long vidw01add1;

        unsigned long vidw00add2b0;
        unsigned long vidw00add2b1;
        unsigned long vidw01add2;

        unsigned long vidintcon;
        unsigned long w1keycon0;
        unsigned long w1keycon1;
        unsigned long w2keycon0;
        unsigned long w2keycon1;
        unsigned long w3keycon0;
        unsigned long w3keycon1;
        unsigned long w4keycon0;
        unsigned long w4keycon1;

        unsigned long win0map;
        unsigned long win1map;

        unsigned long wpalcon;
        unsigned long dithmode;
        unsigned long intclr0;
        unsigned long intclr1;
        unsigned long intclr2;

        unsigned long win0pal;
        unsigned long win1pal;
};

struct s3c2410fb_mach_info {

	struct s3c2410fb_display *displays;	/* attached diplays info */
	unsigned num_displays;			/* number of defined displays */
	unsigned default_display;

	/* GPIOs */

	unsigned long	gpcup;
	unsigned long	gpcup_mask;
	unsigned long	gpccon;
	unsigned long	gpccon_mask;
	unsigned long	gpdup;
	unsigned long	gpdup_mask;
	unsigned long	gpdcon;
	unsigned long	gpdcon_mask;

	/* lpc3600 control register */
	unsigned long	lpcsel;
};

extern void __init s3c24xx_fb_set_platdata(struct s3c2410fb_mach_info *);

#endif /* __ASM_ARM_FB_H */
