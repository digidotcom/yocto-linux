
#define VGA_DISPLAY					\
{							\
	.display_name = "VGA",				\
	.width		= 640,				\
	.height		= 480,				\
	.control        = LCD_CONTROL_WATERMARK | 	\
			  LCD_CONTROL_PWR | 		\
			  LCD_CONTROL_TFT | 		\
			  LCD_CONTROL_BGR | 		\
			  LCD_CONTROL_BPP_16 |		\
			  LCD_CONTROL_EN,		\
	.timing = {					\
			  LCD_TIMING0_HBP(40) | 	\
			  LCD_TIMING0_HFP(8) | 		\
			  LCD_TIMING0_HSW(96) | 	\
			  LCD_TIMING0_PPL(640 / 16 - 1),\
			  LCD_TIMING1_VBP(25) | 	\
			  LCD_TIMING1_VFP(2) | 		\
			  LCD_TIMING1_VSW(2) | 		\
			  LCD_TIMING1_LPP(480 - 1),	\
			  LCD_TIMING2_CPL(640 - 1) | 	\
			  LCD_TIMING2_BCD | 		\
			  LCD_TIMING2_IPC | 		\
			  LCD_TIMING2_IHS | 		\
			  LCD_TIMING2_IVS,		\
	},						\
	.clock		= 0, /* external */		\
}
