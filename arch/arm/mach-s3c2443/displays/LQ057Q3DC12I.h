
static void lcd_power_enable_lq57(int enable)
{
	s3c2443_gpio_cfgpin(S3C2410_GPG(4), (0x01 << 8)); /* Output */
	s3c2410_gpio_setpin(S3C2410_GPG(4), enable ? 0 : 1);
}

#define LQ057Q3DC12I_DISPLAY		\
{					\
	.display_name = "LQ057Q3DC12I",	\
	/* epll clock, 96MHz */		\
	.clock_source = "display-if",	\
	.width		= 320,		\
	.height		= 240,		\
	.xres           = 320,		\
	.yres           = 240,		\
	.bpp            = 16,		\
	.frame_rate	= 60,		\
	/* horizontal front porch */	\
	.left_margin    = 3,		\
	/* vertical front porch */	\
	.upper_margin   = 3,		\
	/* horizontal back porch */	\
	.right_margin   = 5, 	  	\
	/* vertical back porch */	\
	.lower_margin   = 3,		\
	.hsync_len      = 12,		\
	.vsync_len      = 4,		\
	.vidcon1	= S3C24XX_LCD_VIDCON1_IHSYNC |	\
			  S3C24XX_LCD_VIDCON1_IVSYNC,	\
	.vidcon0	= S3C24XX_LCD_VIDCON0_VIDOUT_RGB_IF |	\
			  S3C24XX_LCD_VIDCON0_RGB_PAR |		\
			  S3C24XX_LCD_VIDCON0_VCLK_OFF |	\
			  S3C24XX_LCD_VIDCON0_CLKDIR_DIVIDED,	\
	.bpp_mode	= S3C24XX_LCD_WINCON_16BPP_565,		\
	.display_power_enable = &lcd_power_enable_lq57,		\
}

