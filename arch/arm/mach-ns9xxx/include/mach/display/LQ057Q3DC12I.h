
#define LQ057Q3DC12I_DISPLAY				\
{							\
	.display_name = "LQ057Q3DC12I",			\
	.width		= 320,				\
	.height		= 240,				\
	.control        = LCD_CONTROL_WATERMARK | 	\
			  LCD_CONTROL_PWR | 		\
			  LCD_CONTROL_TFT | 		\
			  LCD_CONTROL_BGR | 		\
			  LCD_CONTROL_BPP_16 | 		\
			  LCD_CONTROL_EN,		\
	.timing = {					\
			  LCD_TIMING0_HBP(8) | 		\
			  LCD_TIMING0_HFP(39) | 	\
			  LCD_TIMING0_HSW(2) | 		\
			  LCD_TIMING0_PPL(320 / 16 - 1),\
			  LCD_TIMING1_VBP(6) | 		\
			  LCD_TIMING1_VFP(5) | 		\
			  LCD_TIMING1_VSW(0) | 		\
			  LCD_TIMING1_LPP(240 - 1),	\
			  LCD_TIMING2_CPL(320 - 1) |	\
			  LCD_TIMING2_IHS | 		\
			  LCD_TIMING2_IVS,		\
	},						\
	.clock		= 7000000,			\
}
