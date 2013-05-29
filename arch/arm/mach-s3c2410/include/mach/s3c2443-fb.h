/* arch/arm/mach-s3c2410/include/mach/s3c2443-fb.h
 *
 * Copyright (c) 2009 Luis Galdos & Pedro Perez de Heredia
 *
 * Inspired by fb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_S3C2443_FB_H
#define __ASM_ARM_S3C2443_FB_H

#include <mach/regs-lcd.h>

/* LCD description */
struct s3c2443fb_display {
	/* LCD type */
	unsigned type;

	/* Display name */
	char *display_name;		
	
	/* Clock parent in ascii: "lcd" -> hclck, "display-if" -> epll */
	char *clock_source;		

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

	unsigned int bpp_mode;
	unsigned int frame_rate;

	/* lcd configuration registers */
	unsigned long vidcon0;
	unsigned long vidcon1;

	void (*display_power_enable)(int);
};

struct s3c2443fb_mach_info {

	struct s3c2443fb_display *displays;	/* attached diplays info */
	struct s3c2443fb_display *display;	/* attached diplays info */
	unsigned num_displays;			/* number of defined displays */
	/* GPIOs */
	unsigned long	gpcup;
	unsigned long	gpcup_mask;
	unsigned long	gpccon;
	unsigned long	gpccon_mask;
	unsigned long	gpdup;
	unsigned long	gpdup_mask;
	unsigned long	gpdcon;
	unsigned long	gpdcon_mask;
};

#endif /* __ASM_ARM_S3C2443_FB_H */
