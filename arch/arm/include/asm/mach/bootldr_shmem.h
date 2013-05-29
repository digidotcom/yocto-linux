/*
 * Copyright 2010 Digi International, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARM_MACH_BOOTLDR_SHMEM_H__
#define __ASM_ARM_MACH_BOOTLDR_SHMEM_H__

#define BL_SHARED_RAM_OFFS_LCD		0x1980	/* Offset for LCD configuration in bootloader shared RAM space */

#define NV_LCD_CONFIG_MAGIC		"LCD_CONF"
#define NV_LCD_CONFIG_VERSION 		2	/* increase number when structure is changed */

typedef struct nv_lcd_config_header_t {
	char		magic_string[8];	/* LCD_CONF */
	uint32_t	version;		/* This struct version */
	uint32_t	crc;			/* Crc of configuration sections */
	uint8_t		lcd1_valid;		/* LCD 1's configuration valid */
	uint8_t		lcd2_valid;		/* LCD 2's configuration valid */
	uint8_t		padding[110];
} nv_lcd_config_header_t;

typedef struct nv_lcd_config_data_t {
	struct {				/* Exact copy of the struct fb_videomode */
		const char	*name;		/* not used */
		uint32_t	refresh;	/* optional */
		uint32_t	xres;
		uint32_t	yres;
		uint32_t	pixclock;
		uint32_t	left_margin;
		uint32_t	right_margin;
		uint32_t	upper_margin;
		uint32_t	lower_margin;
		uint32_t	hsync_len;
		uint32_t	vsync_len;
		uint32_t	sync;
		uint32_t	vmode;
		uint32_t	flag;
	} video_mode;
	uint8_t			is_bl_enable_func;
	uint8_t			padding1[3];
	struct {
		uint32_t	pix_data_offset;
		uint32_t	pix_clk_up;
		uint32_t	pix_clk_down;
	} pix_cfg;
	uint8_t			padding2[56];
} nv_lcd_config_data_t;

typedef struct nv_lcd_config_t {
	nv_lcd_config_header_t	header;
	nv_lcd_config_data_t	lcd1;
	nv_lcd_config_data_t	lcd2;
} nv_lcd_config_t;

#endif		/* __ASM_ARM_MACH_BOOTLDR_SHMEM_H__ */
