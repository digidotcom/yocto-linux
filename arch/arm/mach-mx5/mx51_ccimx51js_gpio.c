/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009-2010 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "iomux.h"
#include "mx51_pins.h"
#include "board-ccimx51.h"


/**
 * iomux settings for the external ethernet mac
 */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccimx51_iomux_ext_eth_pins[] = {
        {MX51_PIN_EIM_CS5, IOMUX_CONFIG_ALT0,
                        (PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_MEDIUM), },
	{MX51_PIN_EIM_OE, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA0, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA1, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA2, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA3, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA4, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA5, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA6, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA7, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D18, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D19, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D20, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D22, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D23, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D24, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D25, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D26, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D27, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D28, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D29, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D30, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D31, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_GPIO1_9, IOMUX_CONFIG_ALT0, (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU), },
};
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static struct mxc_iomux_pin_cfg ccimx51_iomux_mmc1_pins[] = {
	/* SDHC1*/
	{
		MX51_PIN_SD1_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA1, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA2, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA3, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
#ifdef CONFIG_JSCCIMX51_V1
	{
		MX51_PIN_GPIO1_0, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},
#endif
};

#if defined(CONFIG_WIRELESS)
/* IOMUX settings, for the wireless interface on Wi-i.MX51 module */
#define SD2_PAD_CFG	(PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH | \
			 PAD_CTL_22K_PU | PAD_CTL_PUE_PULL | \
			 PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST)
static struct mxc_iomux_pin_cfg ccimx51_iomux_mmc2_pins[] = {
	/* SDHC2*/
	{
		MX51_PIN_SD2_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA1, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA2, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA3, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
};

#else
/* IOMUX settings, SD2 is not connected on JSK, copy the SD1 configuration */
/* This assumed it will be used as an SD card reader */
/* Please note that for SD2 to work on a wireless module, a 0Ω resistor
 * needs to be placed on R120 to power the interface. */
#define SD2_PAD_CFG	(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | \
		PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU | PAD_CTL_SRE_FAST)
static struct mxc_iomux_pin_cfg ccimx51_iomux_mmc2_pins[] = {
	/* SDHC2*/
	{
		MX51_PIN_SD2_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0 ,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA1, IOMUX_CONFIG_ALT0 ,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA2, IOMUX_CONFIG_ALT0 ,
		SD2_PAD_CFG,
	},
	{
		MX51_PIN_SD2_DATA3, IOMUX_CONFIG_ALT0 ,
		SD2_PAD_CFG,
	},
};
#endif

static struct mxc_iomux_pin_cfg ccimx51_iomux_mmc3_pins[] = {
	/* SDHC3*/
	{
		MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	},
	{	/* SD3 DATA0 */
		MX51_PIN_NANDF_D8, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT0_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA1 */
		MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT1_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA2 */
		MX51_PIN_NANDF_D10, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT2_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA3 */
		MX51_PIN_NANDF_D11, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT3_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 Card detect */
		MX51_PIN_GPIO_NAND, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},
	{	/* SD3 Write protect */
		MX51_PIN_NANDF_CS1, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},
};

void gpio_sdhc_active(int interface)
{
	int i;

	switch (interface) {
	  case 0:
		for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_mmc1_pins); i++) {
			mxc_request_iomux(ccimx51_iomux_mmc1_pins[i].pin,
					  ccimx51_iomux_mmc1_pins[i].mux_mode);
			if (ccimx51_iomux_mmc1_pins[i].pad_cfg)
				mxc_iomux_set_pad(ccimx51_iomux_mmc1_pins[i].pin,
						  ccimx51_iomux_mmc1_pins[i].pad_cfg);
			if (ccimx51_iomux_mmc1_pins[i].in_select)
				mxc_iomux_set_input(ccimx51_iomux_mmc1_pins[i].in_select,
						    ccimx51_iomux_mmc1_pins[i].in_mode);
		}
		break;
	case 1:
		for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_mmc2_pins); i++) {
			mxc_request_iomux(ccimx51_iomux_mmc2_pins[i].pin,
					  ccimx51_iomux_mmc2_pins[i].mux_mode);
			if (ccimx51_iomux_mmc2_pins[i].pad_cfg)
				mxc_iomux_set_pad(ccimx51_iomux_mmc2_pins[i].pin,
						  ccimx51_iomux_mmc2_pins[i].pad_cfg);
			if (ccimx51_iomux_mmc2_pins[i].in_select)
				mxc_iomux_set_input(ccimx51_iomux_mmc2_pins[i].in_select,
						    ccimx51_iomux_mmc2_pins[i].in_mode);
		}
		break;

	case 2:
		for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_mmc3_pins); i++) {
			mxc_request_iomux(ccimx51_iomux_mmc3_pins[i].pin,
					  ccimx51_iomux_mmc3_pins[i].mux_mode);
			if (ccimx51_iomux_mmc3_pins[i].pad_cfg)
				mxc_iomux_set_pad(ccimx51_iomux_mmc3_pins[i].pin,
						  ccimx51_iomux_mmc3_pins[i].pad_cfg);
			if (ccimx51_iomux_mmc3_pins[i].in_select)
				mxc_iomux_set_input(ccimx51_iomux_mmc3_pins[i].in_select,
						    ccimx51_iomux_mmc3_pins[i].in_mode);
		}
		break;
	}
}
EXPORT_SYMBOL(gpio_sdhc_active);
void gpio_sdhc_inactive(int module) {}
EXPORT_SYMBOL(gpio_sdhc_inactive);
#endif

#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccimx51_iomux_usbh1_pins[] = {
	{	/* USBH1_STP */
		MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_CLK */
		MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_DIR */
		MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_NXT */
		MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_DATA0 */
		MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA1 */
		MX51_PIN_USBH1_DATA1, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA2 */
		MX51_PIN_USBH1_DATA2, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA3 */
		MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA4 */
		MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA5 */
		MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA6 */
		MX51_PIN_USBH1_DATA6, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA7 */
		MX51_PIN_USBH1_DATA7, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH PHY RESET */
		MX51_PIN_DISPB2_SER_RS, IOMUX_CONFIG_ALT4,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		MUX_IN_GPIO3_IPP_IND_G_IN_8_SELECT_INPUT, INPUT_CTL_PATH1
	},
};
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#if defined(CONFIG_CCIMX5X_DISP0)
#define DISP1_PAD0		(PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST)
static struct mxc_iomux_pin_cfg ccimx51_iomux_video1_pins[] = {
        {       /* DISP1 DAT0 */
		MX51_PIN_DISP1_DAT0, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT1 */
		MX51_PIN_DISP1_DAT1, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT2 */
		MX51_PIN_DISP1_DAT2, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT3 */
		MX51_PIN_DISP1_DAT3, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT4 */
		MX51_PIN_DISP1_DAT4, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT5 */
		MX51_PIN_DISP1_DAT5, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT6 */
		MX51_PIN_DISP1_DAT6, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT7 */
		MX51_PIN_DISP1_DAT7, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT8 */
		MX51_PIN_DISP1_DAT8, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT9 */
		MX51_PIN_DISP1_DAT9, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT10 */
		MX51_PIN_DISP1_DAT10, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT11 */
		MX51_PIN_DISP1_DAT11, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT12 */
		MX51_PIN_DISP1_DAT12, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT13 */
		MX51_PIN_DISP1_DAT13, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT14 */
		MX51_PIN_DISP1_DAT14, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT15 */
		MX51_PIN_DISP1_DAT15, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT16 */
		MX51_PIN_DISP1_DAT16, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT17 */
		MX51_PIN_DISP1_DAT17, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT18 */
		MX51_PIN_DISP1_DAT18, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT19 */
		MX51_PIN_DISP1_DAT19, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT20 */
		MX51_PIN_DISP1_DAT20, IOMUX_CONFIG_ALT0,
		DISP1_PAD0,
	},
	{	/* DISP1 DAT21 */
                MX51_PIN_DISP1_DAT21, IOMUX_CONFIG_ALT0,
                DISP1_PAD0,
        },
#if !defined(CONFIG_CCIMX5X_DISP1)
        {       /* DISP1 DAT22 */
                MX51_PIN_DISP1_DAT22, IOMUX_CONFIG_ALT0,
                DISP1_PAD0,
	},
	{	/* DISP1 DAT23 */
                MX51_PIN_DISP1_DAT23, IOMUX_CONFIG_ALT0,
                DISP1_PAD0,
        },
#endif
};
#endif

#if defined(CONFIG_CCIMX5X_DISP1)
#define DISP2_PAD0		(PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST)
static struct mxc_iomux_pin_cfg ccimx51_iomux_video2_pins[] = {
        /* This interface can be enabled only if the FEC interface is disabled */
        {       /* DISP2 DAT0 */
                MX51_PIN_DISP2_DAT0, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT1 */
                MX51_PIN_DISP2_DAT1, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT2 */
                MX51_PIN_DISP2_DAT2, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT3 */
                MX51_PIN_DISP2_DAT3, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT4 */
                MX51_PIN_DISP2_DAT4, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT5 */
                MX51_PIN_DISP2_DAT5, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT6 */
                MX51_PIN_DISP2_DAT6, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT7 */
                MX51_PIN_DISP2_DAT7, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT8 */
                MX51_PIN_DISP2_DAT8, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT9 */
                MX51_PIN_DISP2_DAT9, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT10 */
                MX51_PIN_DISP2_DAT10, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT11 */
                MX51_PIN_DISP2_DAT11, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT12 */
                MX51_PIN_DISP2_DAT12, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT13 */
                MX51_PIN_DISP2_DAT13, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT14 */
                MX51_PIN_DISP2_DAT14, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DAT15 */
                MX51_PIN_DISP2_DAT15, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 16 (also DISP1 DAT22) */
                MX51_PIN_DISP1_DAT22, IOMUX_CONFIG_ALT5,
                DISP2_PAD0,
        },
        {       /* DISP2 17 (also DISP1 DAT23) */
                MX51_PIN_DISP1_DAT23, IOMUX_CONFIG_ALT5,
                DISP2_PAD0,
        },
        {       /* DISP2 HSYNC */
                MX51_PIN_DI2_PIN2, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 VSYNC */
                MX51_PIN_DI2_PIN3, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 PCLK */
                MX51_PIN_DI2_DISP_CLK, IOMUX_CONFIG_ALT0,
                DISP2_PAD0,
        },
        {       /* DISP2 DRDY */
                MX51_PIN_DI_GP4, IOMUX_CONFIG_ALT4,
                DISP2_PAD0,
        },
};
#endif
#endif

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE) || \
	defined(CONFIG_I2C_IMX) || defined(CONFIG_I2C_IMX_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccimx51_iomux_i2c_pins[] = {
#if defined (CONFIG_I2C_MX_SELECT1)
	/* These are muxed with wireless so they are only available in non
	 * wireless variants. This option is not selectable for the CCWMX51. */
	{
		MX51_PIN_SD2_CMD, IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_FAST | PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_HYS_ENABLE |
		PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH),
		MUX_IN_I2C1_IPP_SDA_IN_SELECT_INPUT, INPUT_CTL_PATH2,
	},
	{
		MX51_PIN_SD2_CLK, IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_FAST | PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_HYS_ENABLE |
		PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH),
		MUX_IN_I2C1_IPP_SCL_IN_SELECT_INPUT, INPUT_CTL_PATH2,
	},
#endif
#ifdef CONFIG_I2C_MX_SELECT2
	{
		MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_FAST | PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE),
		MUX_IN_I2C2_IPP_SCL_IN_SELECT_INPUT, INPUT_CTL_PATH3,
	},
	{
		MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_FAST | PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE),
		MUX_IN_I2C2_IPP_SDA_IN_SELECT_INPUT, INPUT_CTL_PATH3,
	},
#endif
#ifdef CONFIG_I2C_MX_SELECT3
	/* These are the HS I2C port which is not currently supported
	 * This option is not selectable for the CCXMX51 */
	{
		MX51_PIN_I2C1_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE |
		PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_I2C1_DAT, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE |
		PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU | PAD_CTL_SRE_SLOW),
	}
#endif
};
#endif /* defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE) */

static struct mxc_iomux_pin_cfg __initdata ccimx51_iomux_devices_pins[] = {
	{	/* PMIC interrupt line */
		MX51_PIN_GPIO1_5, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_100K_PU |
		PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_VOT_HIGH),
	},
#if defined(CONFIG_INPUT_MMA7455L) || defined(CONFIG_INPUT_MMA7455L_MODULE)
	{	/* MMA7455L interrupt line */
		MX51_PIN_GPIO1_6, IOMUX_CONFIG_GPIO,
	},
	{
		MX51_PIN_GPIO1_7, IOMUX_CONFIG_ALT0,
		(PAD_CTL_DRV_HIGH | PAD_CTL_PUE_PULL |
		PAD_CTL_100K_PU | PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST),
	},
#endif
};

#if defined(CONFIG_SND_SOC_WM8753) || defined(CONFIG_SND_SOC_WM8753_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccimx51_audio_pins[] = {

	/* TODO: the SSI interface should be selectable through configuration */
	{	/* AUD3_BB_CK */
		MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0 ,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_PKE_ENABLE |
		PAD_CTL_PUE_KEEPER ),
	},
	{	/* AUD3_BB_FS */
		MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT0 ,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_HYS_NONE |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE),
	},
	{	/* AUD3_BB_RXD */
		MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0 ,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_HYS_NONE |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE),
	},
	{	/* AUD3_BB_TXD */
		MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT0 ,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_PKE_ENABLE |
		PAD_CTL_PUE_KEEPER ),
	},
};
#endif

#if defined CONFIG_VIDEO_MXC_IPU_CAMERA
static struct mxc_iomux_pin_cfg __initdata ccimx51_camera_pins[] = {
		/* CSI0 camera interface 1 */
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{
		MX51_PIN_CSI1_D12, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D13, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D14, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D15, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D16, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D17, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D18, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_D19, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI1_VSYNC, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_CSI1_HSYNC, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	},
	/* Configure GPIO3_13 as RESET for camera 1 */
	{
		MX51_PIN_CSI1_D9, IOMUX_CONFIG_ALT3,
		(PAD_CTL_HYS_NONE | PAD_CTL_DRV_MEDIUM | PAD_CTL_SRE_FAST),
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	/* CSI2 camera interface 2 */
	{
		MX51_PIN_CSI2_D12, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D13, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D14, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D15, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D16, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D17, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D18, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_D19, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	},
	{
		MX51_PIN_CSI2_VSYNC, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_CSI2_HSYNC, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_CSI2_PIXCLK, IOMUX_CONFIG_ALT0,
		(PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	},
	/* Configure GPIO3_7 as RESET for camera 2 */
	{
		MX51_PIN_DISPB2_SER_CLK, IOMUX_CONFIG_ALT4,
		(PAD_CTL_HYS_NONE | PAD_CTL_DRV_MEDIUM | PAD_CTL_SRE_FAST | IOMUX_CONFIG_SION),
		MUX_IN_GPIO3_IPP_IND_G_IN_7_SELECT_INPUT,
		INPUT_CTL_PATH1
	 },
#endif
};
#endif /* #if defined CONFIG_VIDEO_MXC_IPU_CAMERA */

#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccimx51_cspi_pins[] = {
#ifdef CONFIG_SPI_MXC_SELECT1
	/* ECSPI1 */
	{	/* MISO */
		MX51_PIN_CSPI1_MISO, IOMUX_CONFIG_ALT0,
		(PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
		PAD_CTL_SRE_FAST),
	},
	{	/* MOSI */
		MX51_PIN_CSPI1_MOSI, IOMUX_CONFIG_ALT0,
		(PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
		PAD_CTL_SRE_FAST),
	},
	{	/* SCLK */
		MX51_PIN_CSPI1_SCLK, IOMUX_CONFIG_ALT0,
		(PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
		PAD_CTL_SRE_FAST),
	},
	{	/* SS0 */
		MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE),
	},

#if defined(CONFIG_JSCCIMX51_V1)
	{	/* TS CS for LCD1 on CCIMX51 EAK */
		MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_ALT4,
		(PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_ODE_OPENDRAIN_NONE |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_NONE),
		MUX_IN_GPIO3_IPP_IND_G_IN_4_SELECT_INPUT,INPUT_CTL_PATH1
	},
	{	/* TS CS for LCD2 on CCIMX51 EAK */
		MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT3,
		(PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_ODE_OPENDRAIN_NONE |
		PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_NONE),
		MUX_IN_AUDMUX_P4_INPUT_TXFS_AMX_SELECT_INPUT,INPUT_CTL_PATH1
	},
#else
	{	/* TS CS for LCD1 and  LCD2 on CCIMX51 JSK */
		MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT3,
		(PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_ODE_OPENDRAIN_NONE |
		PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_NONE),
		MUX_IN_AUDMUX_P4_INPUT_TXFS_AMX_SELECT_INPUT,INPUT_CTL_PATH1
	},
#endif

#ifdef CONFIG_SPI_MXC_SELECT1_SS1
	{	/* SS1 */
		MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_GPIO,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE),
	},
#endif
#endif
#if defined(CONFIG_SPI_MXC_SELECT2) && (!defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE))
	/* ECSPI2 */
	{	/* SCLK */
		MX51_PIN_NANDF_RB2, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_LOW |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
	{	/* MISO */
		MX51_PIN_NANDF_RB3, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_HIGH |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
	{	/* MOSI */
		MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_HIGH |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
	{	/* SS0 */
		MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_LOW |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
	{	/* SI_VER_TO2, SS1 */
		MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_HIGH |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
	{	/* SI_VER_TO2, RDY */
		MX51_PIN_NANDF_RB1, IOMUX_CONFIG_ALT2,
		(PAD_CTL_SRE_SLOW | PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_DRV_LOW |
		PAD_CTL_PUE_KEEPER | PAD_CTL_HYS_ENABLE),
	},
#endif
#ifdef CONFIG_SPI_MXC_SELECT3
	/* ECSPI3 */
	{
		MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU |
		 PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER |
		PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER |
		PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER |
		PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU |
		PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
	{
		MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT2,
		(PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER |
		PAD_CTL_DRV_MEDIUM | PAD_CTL_HYS_ENABLE | PAD_CTL_SRE_SLOW),
	},
#endif
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
void ccimx51_gpio_spi_chipselect_active(int busnum, int ssb_pol, int chipselect)
{
	u8 mask = 0x1 << (chipselect - 1);

	/* Deassert/Assert the different CS lines for the different buses */
	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0),
				       (ssb_pol & mask) ?  1 : 0);
			break;
		case 0x2:
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1),
				       (ssb_pol & mask) ?  1 : 0);
			break;
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(IOMUX_TO_GPIO(SECOND_TS_SPI_SS_PIN), 0);
			break;
#endif
		default:
			break;
		}
		break;
	case 2:
	case 3:
	default:
		break;
	}
}
EXPORT_SYMBOL(ccimx51_gpio_spi_chipselect_active);

void ccimx51_gpio_spi_chipselect_inactive(int busnum, int ssb_pol,
					  int chipselect)
{
	u8 mask = 0x1 << (chipselect - 1);

	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0),
				       (ssb_pol & mask) ?  0 : 1);
			break;
		case 0x2:
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1),
				       (ssb_pol & mask) ?  0 : 1);
			break;
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(IOMUX_TO_GPIO(SECOND_TS_SPI_SS_PIN), 1);
			break;
#endif
		default:
			break;
		}
		break;
	case 2:
	case 3:
	default:
		break;
        }
}
EXPORT_SYMBOL(ccimx51_gpio_spi_chipselect_inactive);
#endif /* defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE) */

#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
static struct mxc_iomux_pin_cfg ata_iomux_pins[] = {
        {
                MX51_PIN_NANDF_ALE, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_RE_B, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_WE_B, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_CLE, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_RB0, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_WP_B, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_GPIO_NAND, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_RB1, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D0, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D1, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D2, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D3, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D4, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D5, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D6, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D7, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D8, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D10, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D11, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D13, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D14, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
        {
                MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT1,
                (PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH),
        },
};

void gpio_ata_active(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ata_iomux_pins); i++) {
		mxc_request_iomux(ata_iomux_pins[i].pin,
				  ata_iomux_pins[i].mux_mode);
		if (ata_iomux_pins[i].pad_cfg)
			mxc_iomux_set_pad(ata_iomux_pins[i].pin,
					  ata_iomux_pins[i].pad_cfg);
		if (ata_iomux_pins[i].in_select)
			mxc_iomux_set_input(ata_iomux_pins[i].in_select,
					    ata_iomux_pins[i].in_mode);
	}
}
EXPORT_SYMBOL(gpio_ata_active);
void gpio_ata_inactive(void) {}
EXPORT_SYMBOL(gpio_ata_inactive);
#endif /* CONFIG_PATA_FSL || CONFIG_PATA_FSL_MODULE */

void __init ccimx51_io_init(void)
{
        int i;

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
        for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_ext_eth_pins); i++) {
                mxc_request_iomux(ccimx51_iomux_ext_eth_pins[i].pin,
                                ccimx51_iomux_ext_eth_pins[i].mux_mode);
		if (ccimx51_iomux_ext_eth_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_iomux_ext_eth_pins[i].pin,
					ccimx51_iomux_ext_eth_pins[i].pad_cfg);
		if (ccimx51_iomux_ext_eth_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_iomux_ext_eth_pins[i].in_select,
					ccimx51_iomux_ext_eth_pins[i].in_mode);
	}
#endif

#if defined CONFIG_VIDEO_MXC_IPU_CAMERA
	for (i = 0; i < ARRAY_SIZE(ccimx51_camera_pins); i++) {
		mxc_request_iomux(ccimx51_camera_pins[i].pin,
				ccimx51_camera_pins[i].mux_mode);
		if (ccimx51_camera_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_camera_pins[i].pin,
					ccimx51_camera_pins[i].pad_cfg);
		if (ccimx51_camera_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_camera_pins[i].in_select,
					ccimx51_camera_pins[i].in_mode);
	}

	/* Configure non muxed pins */
	mxc_iomux_set_pad(MX51_PIN_CSI1_PIXCLK,PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW);
	mxc_iomux_set_pad(MX51_PIN_CSI2_PIXCLK,PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW);
	mxc_iomux_set_pad(MX51_PIN_CSI1_MCLK,PAD_CTL_DRV_HIGH | PAD_CTL_SRE_SLOW);

#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	/* Camera 1 reset */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), "gpio3_13");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), 0);
	// Take camera out of reset
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9), 1);
	msleep(100);
#endif /* CAMERA 1 */
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	/* Camera 2 reset */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), "gpio3_7");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 0);
	// Take camera out of reset
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 1);
	msleep(100);
#endif /* CAMERA 2 */
#endif

#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_usbh1_pins); i++) {
		mxc_request_iomux(ccimx51_iomux_usbh1_pins[i].pin,
				  ccimx51_iomux_usbh1_pins[i].mux_mode);
		if (ccimx51_iomux_usbh1_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_iomux_usbh1_pins[i].pin,
					  ccimx51_iomux_usbh1_pins[i].pad_cfg);
		if (ccimx51_iomux_usbh1_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_iomux_usbh1_pins[i].in_select,
					    ccimx51_iomux_usbh1_pins[i].in_mode);
	}
#endif

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE) || \
	defined(CONFIG_I2C_IMX) || defined(CONFIG_I2C_IMX_MODULE)
        for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_i2c_pins); i++) {
                mxc_request_iomux(ccimx51_iomux_i2c_pins[i].pin,
				ccimx51_iomux_i2c_pins[i].mux_mode);
		if (ccimx51_iomux_i2c_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_iomux_i2c_pins[i].pin,
					ccimx51_iomux_i2c_pins[i].pad_cfg);
		if (ccimx51_iomux_i2c_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_iomux_i2c_pins[i].in_select,
					ccimx51_iomux_i2c_pins[i].in_mode);
	}
#endif

#if defined(CONFIG_SND_SOC_WM8753) || defined(CONFIG_SND_SOC_WM8753_MODULE)
	for (i = 0; i < ARRAY_SIZE(ccimx51_audio_pins); i++) {
		mxc_request_iomux(ccimx51_audio_pins[i].pin,
				  ccimx51_audio_pins[i].mux_mode);
		if (ccimx51_audio_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_audio_pins[i].pin,
					  ccimx51_audio_pins[i].pad_cfg);
		if (ccimx51_audio_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_audio_pins[i].in_select,
					    ccimx51_audio_pins[i].in_mode);
	}
#endif

#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)
	for (i = 0; i < ARRAY_SIZE(ccimx51_cspi_pins); i++) {
		mxc_request_iomux(ccimx51_cspi_pins[i].pin,
				  ccimx51_cspi_pins[i].mux_mode);
		if (ccimx51_cspi_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_cspi_pins[i].pin,
					  ccimx51_cspi_pins[i].pad_cfg);
		if (ccimx51_cspi_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_cspi_pins[i].in_select,
					    ccimx51_cspi_pins[i].in_mode);
	}
#ifdef CONFIG_SPI_MXC_SELECT1
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), "cspi1_ss0");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), 0);
#ifdef CONFIG_SPI_MXC_SELECT1_SS1
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), "cspi1_ss1");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), 0);
#endif
#endif

#ifndef CONFIG_SPI_MXC_SELECT2
        /* Configure as GPIO to be used to read LED status */
        mxc_config_iomux(MX51_PIN_NANDF_RB2,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
        mxc_iomux_set_pad(MX51_PIN_NANDF_RB2,PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU);
#if !defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE)
        mxc_config_iomux(MX51_PIN_NANDF_RB1,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
        mxc_iomux_set_pad(MX51_PIN_NANDF_RB1,PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU);
#endif /* !defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE) */
#endif

#endif

        for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_devices_pins); i++) {
		mxc_request_iomux(ccimx51_iomux_devices_pins[i].pin,
				  ccimx51_iomux_devices_pins[i].mux_mode);
		if (ccimx51_iomux_devices_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccimx51_iomux_devices_pins[i].pin,
					  ccimx51_iomux_devices_pins[i].pad_cfg);
		if (ccimx51_iomux_devices_pins[i].in_select)
			mxc_iomux_set_input(ccimx51_iomux_devices_pins[i].in_select,
					    ccimx51_iomux_devices_pins[i].in_mode);
	}

	/* PMIC interrupt line */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), "gpio1_5");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	/* USB PHY/HUB reset*/
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), "gpio3_8");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 0);
	msleep(1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 1);
#endif

	/* Configure user key 1 as GPIO */
#if defined(CONFIG_JSCCIMX51_V2)
	mxc_config_iomux(MX51_PIN_DISPB2_SER_DIO,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT,INPUT_CTL_PATH1);
#endif

	/* Configure Digital IO as GPIO */
#if defined(CONFIG_JSCCIMX51_V1)
#if !defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE)
	mxc_config_iomux(MX51_PIN_NANDF_CS4,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_CS5,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_CS6,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
#endif
#if !defined(CONFIG_MMC_IMX_ESDHCI) && !defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
	mxc_config_iomux(MX51_PIN_NANDF_CS7,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
#endif

	mxc_config_iomux(MX51_PIN_DISPB2_SER_DIN,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_5_SELECT_INPUT,INPUT_CTL_PATH1);
	mxc_config_iomux(MX51_PIN_DISPB2_SER_DIO,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT,INPUT_CTL_PATH1);
#if !defined (CONFIG_VIDEO_MXC_IPU_CAMERA)
	mxc_config_iomux(MX51_PIN_DISPB2_SER_CLK,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_7_SELECT_INPUT,INPUT_CTL_PATH1);
#endif
#if !defined(CONFIG_USB_EHCI_ARC_H1) && !defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	mxc_config_iomux(MX51_PIN_DISPB2_SER_RS,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_8_SELECT_INPUT,INPUT_CTL_PATH1);
#endif
#endif

#if defined(CONFIG_JSCCIMX51_V2)
#if !defined (CONFIG_SPI_MXC_SELECT2)
	mxc_config_iomux(MX51_PIN_NANDF_RB3,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
#endif

#if !defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE)
	mxc_config_iomux(MX51_PIN_NANDF_CS2,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_CS4,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_CS5,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_CS6,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
#endif

#if !defined(CONFIG_SPI_MXC_SELECT2) || (!defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE))
	mxc_config_iomux(MX51_PIN_NANDF_RB1,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
	mxc_config_iomux(MX51_PIN_NANDF_RB2,IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
#endif
	mxc_config_iomux(MX51_PIN_DISPB2_SER_DIO,IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT,INPUT_CTL_PATH1);
#endif

}

#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
void ccimx51_2nd_touch_gpio_init(void)
{
	/* Second touch interface interrupt line */
	mxc_request_iomux(SECOND_TS_IRQ_PIN, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(SECOND_TS_IRQ_PIN, PAD_CTL_SRE_FAST | PAD_CTL_HYS_ENABLE);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_3_SELECT_INPUT, INPUT_CTL_PATH1);

	/* SECOND_TS_SPI_SS_PIN depends on configuration, check board-ccimx51 to see options */
	mxc_request_iomux(SECOND_TS_SPI_SS_PIN, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(SECOND_TS_SPI_SS_PIN, PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH |
			  PAD_CTL_47K_PU | PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE);

	/* Configure the Slave Select signal as gpio, to workaround a silicon errata */
	gpio_request(IOMUX_TO_GPIO(SECOND_TS_SPI_SS_PIN), "ts2_spi_ss");
	gpio_direction_output(IOMUX_TO_GPIO(SECOND_TS_SPI_SS_PIN), 1);
	gpio_set_value(IOMUX_TO_GPIO(SECOND_TS_SPI_SS_PIN), 1);

	/* Configure 2nd touch interrupt line */
	gpio_request(IOMUX_TO_GPIO(SECOND_TS_IRQ_PIN), "ts2_irq");
	gpio_direction_input(IOMUX_TO_GPIO(SECOND_TS_IRQ_PIN));

	/**
	 * Configure gpio line to detect which touch is connected to each
	 * display interface
	 */
	mxc_config_iomux(MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX51_PIN_DI1_D1_CS, PAD_CTL_SRE_FAST | PAD_CTL_HYS_ENABLE);
	mxc_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_4_SELECT_INPUT, INPUT_CTL_PATH1);
}
#else
void ccimx51_2nd_touch_gpio_init(void) {}
#endif

#if defined(CONFIG_SERIAL_MXC) || defined(CONFIG_SERIAL_MXC_MODULE)
#define SERIAL_PORT_PAD		(PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | \
				 PAD_CTL_PUE_PULL | PAD_CTL_DRV_HIGH | \
				 PAD_CTL_SRE_FAST)

void gpio_uart_active(int port, int no_irda)
{
	/* Configure the IOMUX control registers for the UART signals */
	switch (port) {

	case 0:		/* UART 1 IOMUX Configs */
#ifdef CONFIG_UART1_ENABLED
		mxc_request_iomux(MX51_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
		mxc_request_iomux(MX51_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_RXD, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_UART1_TXD, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART1_IPP_UART_RXD_MUX_SELECT_INPUT, INPUT_CTL_PATH0);

#if defined(CONFIG_UART1_MODE_RS485)
		mxc_request_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_UART1_CTS, SERIAL_PORT_PAD);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_UART1_CTS), "rs485_uart1_dir");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_UART1_CTS), !UART1_RS485_TXDIR_LVL);
#endif
#if defined(CONFIG_UART1_CTS_RTS_ENABLED) || defined(CONFIG_UART1_FULL_UART_ENABLED)
		mxc_request_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_CTS, SERIAL_PORT_PAD);
		mxc_request_iomux(MX51_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_RTS, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART1_IPP_UART_RTS_B_SELECT_INPUT, INPUT_CTL_PATH0);
#endif /* CONFIG_UART1_CTS_RTS_ENABLED || CONFIG_UART1_FULL_UART_ENABLED */
#ifdef CONFIG_UART1_FULL_UART_ENABLED
		mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_ALT1);	/* DCD */
		mxc_request_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_ALT1);	/* RI */
		mxc_request_iomux(MX51_PIN_UART3_TXD, IOMUX_CONFIG_ALT0);	/* DSR */
		mxc_request_iomux(MX51_PIN_UART3_RXD, IOMUX_CONFIG_ALT0);	/* DTR */
		mxc_iomux_set_pad(MX51_PIN_KEY_COL5, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_KEY_COL4, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_UART3_TXD, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_UART3_RXD, SERIAL_PORT_PAD);
#endif /* CONFIG_UART1_FULL_UART_ENABLED */
#endif /* CONFIG_UART1_ENABLED */
		break;

	case 1:		/* UART 2 IOMUX Configs */
#ifdef CONFIG_UART2_ENABLED
		mxc_request_iomux(MX51_PIN_UART2_RXD, IOMUX_CONFIG_ALT0);
		mxc_request_iomux(MX51_PIN_UART2_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART2_RXD, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_UART2_TXD, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART2_IPP_UART_RXD_MUX_SELECT_INPUT, INPUT_CTL_PATH2);
#if defined(CONFIG_UART2_MODE_RS485)
		mxc_request_iomux(MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_GPIO);	/* CTS */
		mxc_iomux_set_pad(MX51_PIN_USBH1_DATA0, SERIAL_PORT_PAD);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_USBH1_DATA0), "rs485_uart2_dir");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_USBH1_DATA0), !UART2_RS485_TXDIR_LVL);
#endif
#if defined(CONFIG_UART2_CTS_RTS_ENABLED)
#if !defined(CONFIG_USB_EHCI_ARC_H1) && !defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
		mxc_request_iomux(MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_ALT1);	/* CTS */
		mxc_iomux_set_pad(MX51_PIN_USBH1_DATA0, SERIAL_PORT_PAD);
		mxc_request_iomux(MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_ALT1);	/* RTS */
		mxc_iomux_set_pad(MX51_PIN_USBH1_DATA3, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART2_IPP_UART_RTS_B_SELECT_INPUT, INPUT_CTL_PATH5);
#endif /* CONFIG_USB_EHCI_ARC_H1 && CONFIG_USB_EHCI_ARC_H1_MODULE */
#endif /* CONFIG_UART2_CTS_RTS_ENABLED */
#endif /* CONFIG_UART2_ENABLED */
		break;
	case 2:		/* UART 3 IOMUX Configs */
#ifdef CONFIG_UART3_ENABLED
		mxc_request_iomux(MX51_PIN_UART3_RXD, IOMUX_CONFIG_ALT1);
		mxc_request_iomux(MX51_PIN_UART3_TXD, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_UART3_RXD, SERIAL_PORT_PAD);
		mxc_iomux_set_pad(MX51_PIN_UART3_TXD, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RXD_MUX_SELECT_INPUT, INPUT_CTL_PATH4);
#if defined(CONFIG_UART3_MODE_RS485)
		mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_GPIO);	/* CTS */
		mxc_iomux_set_pad(MX51_PIN_KEY_COL5, SERIAL_PORT_PAD);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_KEY_COL5), "rs485_uart3_dir");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_KEY_COL5), !UART3_RS485_TXDIR_LVL);
#endif
#if defined(CONFIG_UART3_CTS_RTS_ENABLED)
		mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_ALT2);	/* CTS */
		mxc_iomux_set_pad(MX51_PIN_KEY_COL5, SERIAL_PORT_PAD);
		mxc_request_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_ALT2);	/* RTS */
		mxc_iomux_set_pad(MX51_PIN_KEY_COL4, SERIAL_PORT_PAD);
		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RTS_B_SELECT_INPUT, INPUT_CTL_PATH4);
#endif /* CONFIG_UART3_CTS_RTS_ENABLED */
#endif /* CONFIG_UART3_ENABLED */
		break;
	default:
		break;
	}
}
#else
void gpio_uart_active(int port, int no_irda) {}
#endif
void gpio_uart_inactive(int port, int no_irda) {}
EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);

void gpio_video_active(int vif, u32 pad)
{
	int i;

#if defined(CONFIG_CCIMX5X_DISP0)
	if (vif == 0) {
		for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_video1_pins); i++) {
			mxc_request_iomux(ccimx51_iomux_video1_pins[i].pin,
					  ccimx51_iomux_video1_pins[i].mux_mode);

			if (ccimx51_iomux_video1_pins[i].in_select)
				mxc_iomux_set_input(ccimx51_iomux_video1_pins[i].in_select,
						    ccimx51_iomux_video1_pins[i].in_mode);
			if (!pad)
				mxc_iomux_set_pad(ccimx51_iomux_video1_pins[i].pin,
						  ccimx51_iomux_video1_pins[i].pad_cfg);
			else
				mxc_iomux_set_pad(ccimx51_iomux_video1_pins[i].pin,
						  pad);
		}

		/* LCD1 Power Enable, as gpio */
		mxc_request_iomux(MX51_PIN_DI1_PIN11, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DI1_PIN11,
				  PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

		gpio_request(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), "gpio3_0");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), 0);
	}
#endif
#if defined(CONFIG_CCIMX5X_DISP1)
	if (vif == 1) {
		for (i = 0; i < ARRAY_SIZE(ccimx51_iomux_video2_pins); i++) {
			mxc_request_iomux(ccimx51_iomux_video2_pins[i].pin,
					  ccimx51_iomux_video2_pins[i].mux_mode);

			if (ccimx51_iomux_video2_pins[i].in_select)
				mxc_iomux_set_input(ccimx51_iomux_video2_pins[i].in_select,
						    ccimx51_iomux_video2_pins[i].in_mode);
			if (!pad)
				mxc_iomux_set_pad(ccimx51_iomux_video2_pins[i].pin,
						  ccimx51_iomux_video2_pins[i].pad_cfg);
			else
				mxc_iomux_set_pad(ccimx51_iomux_video2_pins[i].pin,
						  pad);
		}

		/* LCD2 Power Enable, as gpio */
#ifdef CONFIG_JSCCIMX51_V1
		mxc_request_iomux(MX51_PIN_DI2_PIN4, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_DI2_PIN4, pad);
#else
		mxc_request_iomux(MX51_PIN_DI1_PIN12, IOMUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX51_PIN_DI1_PIN12,
				  PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

		gpio_request(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), "gpio3_1");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), 0);
#endif
	}
#endif /* defined(CONFIG_CCIMX5X_DISP1) */
}

void gpio_video_inactive(int vif, u32 pad)
{
}
EXPORT_SYMBOL(gpio_video_active);
EXPORT_SYMBOL(gpio_video_inactive);

void gpio_spi_active(void) {}
EXPORT_SYMBOL(gpio_spi_active);
