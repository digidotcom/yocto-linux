
#define VGA_DISPLAY			\
{					\
	.display_name = "VGA",		\
	/* epll clock, 96MHz */		\
	.clock_source = "display-if",	\
	.width		= 640,		\
	.height		= 480,		\
	.xres           = 640,		\
	.yres           = 480,		\
	.bpp            = 16,		\
	.frame_rate	= 60,		\
	/* horizontal front porch */	\
	.left_margin    = 8,		\
	/* vertical front porch */	\
	.upper_margin   = 2,		\
	/* horizontal back porch */	\
	.right_margin   = 40,   	\
	/* vertical back porch */	\
	.lower_margin   = 25,		\
	.hsync_len      = 96,		\
	.vsync_len      = 2,		\
	.vidcon1	= S3C24XX_LCD_VIDCON1_IHSYNC |	\
			  S3C24XX_LCD_VIDCON1_IVSYNC,	\
	.vidcon0	= S3C24XX_LCD_VIDCON0_VIDOUT_RGB_IF |	\
			  S3C24XX_LCD_VIDCON0_RGB_PAR |		\
			  S3C24XX_LCD_VIDCON0_VCLK_OFF |	\
			  S3C24XX_LCD_VIDCON0_CLKDIR_DIVIDED,	\
	.bpp_mode	= S3C24XX_LCD_WINCON_16BPP_565,		\
}

