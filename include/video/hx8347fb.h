/*
 * hx8347fb.h - definitions for the HX8347 framebuffer driver
 *
 * Copyright (C) 2009 by Digi International Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#ifndef __HX8347FB_H_
#define __HX8347FB_H_

/* Index Register values */
#define	DISP_MODE_CTRL		0x01
#define	COL_ADDR_START_2	0x02
#define	COL_ADDR_START_1	0x03
#define	COL_ADDR_END_2		0x04
#define	COL_ADDR_END_1		0x05
#define	ROW_ADDR_START_2	0x06
#define	ROW_ADDR_START_1	0x07
#define	ROW_ADDR_END_2		0x08
#define	ROW_ADDR_END_1		0x09

/* Cont her*/
#define	PARTIAL_AREA_START_R2	0x0a
/* End Cont her*/

#define	MEM_ACCESS_CTRL		0x16
/* Cont her*/
/* End Cont her*/

#define	SRAM_WR_CTRL		0x22
/* Cont her*/
/* End Cont her*/


#define	DISP_CTRL_1		0x26
#define	DISP_CTRL_2		0x27
#define	DISP_CTRL_3		0x28
#define	DISP_CTRL_4		0x29
#define	DISP_CTRL_5		0x2a
#define	POWER_CTRL_11		0x2b
#define	DISP_CTRL_6		0x2c
/* Cont her*/
/* End Cont her*/



#define	HIMAX_ID_CODE		0x67
/* Cont her*/
/* End Cont her*/


/* Display Mode Control Register Masks */
#define	DMC_PTLO		(0x01 << 0)
#define	DMC_NORON		(0x01 << 1)
#define	DMC_INVON		(0x01 << 2)
#define	DMC_IDMON		(0x01 << 3)

struct hx8347fb_par {
	spinlock_t		lock;
	struct fb_info		*info;
	void __iomem		*mmio_cmd;
	void __iomem		*mmio_data;
	dma_addr_t		fb_p;
	struct hx8347fb_pdata	*pdata;
};

 /* Platform data structure */
struct hx8347fb_pdata {
	struct module		*owner;
	int			irq;
	struct platform_device	*pdev;
	struct clk		*bus_clk;
	struct clk		*lcdc_clk;
	int			rst_gpio;
	int			enable_gpio;
	int			usedma;
	unsigned int		xres;
	unsigned int		yres;
	unsigned int		bits_per_pixel;
	u32			pseudo_pal[16];
	void 			(*reset)(struct hx8347fb_par *);
	void			(*bl_enable)(struct hx8347fb_par *, int);
	void			(*cleanup)(struct hx8347fb_par *);
	int			(*init)(struct hx8347fb_par *);
	void			(*set_idx)(struct hx8347fb_par *, u8);
	void			(*wr_reg)(struct hx8347fb_par *, u8, u16);
	u16			(*rd_reg)(struct hx8347fb_par *, u8);
	void			(*wr_data)(struct hx8347fb_par *, u16 *, int);
};

#endif /* __HX8347FB_H_ */
