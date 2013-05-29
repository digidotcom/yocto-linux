/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
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
#include <mach/iomux-mx53.h>

#include "board-ccimx53.h"
#include "devices_ccimx53.h"

#if defined(CONFIG_SERIAL_MXC) || defined(CONFIG_SERIAL_MXC_MODULE)

#ifdef CONFIG_UART1_ENABLED
static iomux_v3_cfg_t ccimx53_uart1_pads[] = {
	MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
	MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
#if defined(CONFIG_UART1_CTS_RTS_ENABLED) || defined(CONFIG_UART1_FULL_UART_ENABLED)
	MX53_PAD_PATA_IORDY__UART1_RTS,
	MX53_PAD_PATA_RESET_B__UART1_CTS,
#endif /* (CONFIG_UART1_CTS_RTS_ENABLED) || (CONFIG_UART1_FULL_UART_ENABLED) */
#if defined(CONFIG_UART1_MODE_RS485)
	MX53_PAD_PATA_RESET_B__GPIO7_4,
#endif /* CONFIG_UART1_MODE_RS485 */
#ifdef CONFIG_UART1_FULL_UART_ENABLED
	MX53_PAD_EIM_D23__UART1_DCD,
	MX53_PAD_EIM_EB3__UART1_RI,
	MX53_PAD_EIM_D24__UART1_DTR,
	MX53_PAD_EIM_D25__UART1_DSR,
#endif /* CONFIG_UART1_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART1_ENABLED */

#ifdef CONFIG_UART2_ENABLED
static iomux_v3_cfg_t ccimx53_uart2_pads[] = {
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
#if defined(CONFIG_UART2_CTS_RTS_ENABLED) || defined(CONFIG_UART2_FULL_UART_ENABLED)
	MX53_PAD_PATA_DIOR__UART2_RTS,
	MX53_PAD_PATA_INTRQ__UART2_CTS,
#endif /* (CONFIG_UART2_CTS_RTS_ENABLED) || (CONFIG_UART2_FULL_UART_ENABLED) */
#if defined(CONFIG_UART2_MODE_RS485)
	MX53_PAD_PATA_INTRQ__GPIO7_2,
#endif /* CONFIG_UART2_MODE_RS485 */
};
#endif /* CONFIG_UART2_ENABLED */

#ifdef CONFIG_UART3_ENABLED
static iomux_v3_cfg_t ccimx53_uart3_pads[] = {
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
#if defined(CONFIG_UART3_CTS_RTS_ENABLED) || defined(CONFIG_UART3_FULL_UART_ENABLED)
	MX53_PAD_PATA_DA_2__UART3_RTS,
	MX53_PAD_PATA_DA_1__UART3_CTS,
#endif /* (CONFIG_UART3_CTS_RTS_ENABLED) || (CONFIG_UART3_FULL_UART_ENABLED) */
#if defined(CONFIG_UART3_MODE_RS485)
	MX53_PAD_PATA_DA_1__GPIO7_7,
#endif /* CONFIG_UART3_MODE_RS485 */
};
#endif /* CONFIG_UART3_ENABLED */

#ifdef CONFIG_UART4_ENABLED


static iomux_v3_cfg_t ccimx53_uart4_pads[] = {
#if defined(CONFIG_UART4_CTS_RTS_ENABLED) || defined(CONFIG_UART4_FULL_UART_ENABLED)
#endif /* (CONFIG_UART4_CTS_RTS_ENABLED) || (CONFIG_UART4_FULL_UART_ENABLED) */
#ifdef CONFIG_UART4_FULL_UART_ENABLED
#endif /* CONFIG_UART4_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART4_ENABLED */

#ifdef CONFIG_UART5_ENABLED
static iomux_v3_cfg_t ccimx53_uart5_pads[] = {
#if defined(CONFIG_UART5_CTS_RTS_ENABLED) || defined(CONFIG_UART5_FULL_UART_ENABLED)
#endif /* (CONFIG_UART5_CTS_RTS_ENABLED) || (CONFIG_UART5_FULL_UART_ENABLED) */
#ifdef CONFIG_UART5_FULL_UART_ENABLED
#endif /* CONFIG_UART5_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART5_ENABLED */


void gpio_uart_active(int port, int no_irda)
{
	/* Configure the IOMUX control registers for the UART signals */
	switch (port) {
#ifdef CONFIG_UART1_ENABLED
	case 0:		/* UART 1 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccimx53_uart1_pads,
						 ARRAY_SIZE(ccimx53_uart1_pads));
#if defined(CONFIG_UART1_MODE_RS485)
		gpio_request(MX53_GPIO(7, 4), "rs485_uart1_dir");
		gpio_direction_output(MX53_GPIO(7, 4), !UART1_RS485_TXDIR_LVL);
#endif
		break;
#endif /* CONFIG_UART1_ENABLED */
#ifdef CONFIG_UART2_ENABLED
	case 1:		/* UART 2 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccimx53_uart2_pads,
						 ARRAY_SIZE(ccimx53_uart2_pads));
#if defined(CONFIG_UART2_MODE_RS485)
		gpio_request(MX53_GPIO(7, 2), "rs485_uart2_dir");
		gpio_direction_output(MX53_GPIO(7, 2), !UART2_RS485_TXDIR_LVL);
#endif
		break;
#endif /* CONFIG_UART2_ENABLED */
#ifdef CONFIG_UART3_ENABLED
	case 2:		/* UART 3 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccimx53_uart3_pads,
						 ARRAY_SIZE(ccimx53_uart3_pads));
#if defined(CONFIG_UART3_MODE_RS485)
		gpio_request(MX53_GPIO(7, 7), "rs485_uart3_dir");
		gpio_direction_output(MX53_GPIO(7, 7), !UART3_RS485_TXDIR_LVL);
#endif
		break;
#endif /* CONFIG_UART3_ENABLED */
#ifdef CONFIG_UART4_ENABLED
	case 3:		/* UART 4 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccimx53_uart4_pads,
						 ARRAY_SIZE(ccimx53_uart4_pads));
		break;
#endif
#ifdef CONFIG_UART5_ENABLED
	case 4:		/* UART 5 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccimx53_uart5_pads,
						 ARRAY_SIZE(ccimx53_uart5_pads));
		break;
#endif
	}
}
#else
void gpio_uart_active(int port, int no_irda) {}
#endif
void gpio_uart_inactive(int port, int no_irda) {}
EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);


#if defined(CONFIG_ESDHCI_MXC_SELECT1)
static iomux_v3_cfg_t ccimx53_mmc1_pads[] = {
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
#if defined(ESDHCI_MXC_SELECT1_8BIT_PORT) || defined(ESDHCI_MXC_SELECT1_4BIT_PORT)
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__SD1_DATA3,
#endif /* (ESDHCI_MXC_SELECT1_8BIT_PORT) || (ESDHCI_MXC_SELECT1_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT1_8BIT_PORT
	MX53_PAD_PATA_DATA8__SD1_DATA4,
	MX53_PAD_PATA_DATA9__SD1_DATA5,
	MX53_PAD_PATA_DATA10__SD1_DATA6,
	MX53_PAD_PATA_DATA11__SD1_DATA7,
#endif /* ESDHCI_MXC_SELECT1_8BIT_PORT */
#ifdef ESDHC1_WP_PAD
	ESDHC1_WP_PAD,
#endif
#ifdef ESDHC1_CD_PAD
	ESDHC1_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT1 */

#if defined(CONFIG_ESDHCI_MXC_SELECT2)
static iomux_v3_cfg_t ccimx53_mmc2_pads[] = {
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
#if defined(ESDHCI_MXC_SELECT2_8BIT_PORT) || defined(ESDHCI_MXC_SELECT2_4BIT_PORT)
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
#endif /* (ESDHCI_MXC_SELECT2_8BIT_PORT) || (ESDHCI_MXC_SELECT2_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT2_8BIT_PORT
	MX53_PAD_PATA_DATA12__ESDHC2_DAT4,
	MX53_PAD_PATA_DATA13__ESDHC2_DAT5,
	MX53_PAD_PATA_DATA14__ESDHC2_DAT6,
	MX53_PAD_PATA_DATA15__ESDHC2_DAT7,
#endif /* ESDHCI_MXC_SELECT2_8BIT_PORT */
#ifdef ESDHC2_WP_PAD
	ESDHC2_WP_PAD,
#endif
#ifdef ESDHC2_CD_PAD
	ESDHC2_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT2 */

#if defined(CONFIG_ESDHCI_MXC_SELECT3)
static iomux_v3_cfg_t ccimx53_mmc3_pads[] = {
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
#if defined(ESDHCI_MXC_SELECT3_8BIT_PORT) || defined(ESDHCI_MXC_SELECT3_4BIT_PORT)
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
#endif /* (ESDHCI_MXC_SELECT3_8BIT_PORT) || (ESDHCI_MXC_SELECT3_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT3_8BIT_PORT
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
#endif /* ESDHCI_MXC_SELECT3_8BIT_PORT */
#ifdef ESDHC3_WP_PAD
	ESDHC3_WP_PAD,
#endif
#ifdef ESDHC3_CD_PAD
	ESDHC3_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT3 */

void gpio_sdhc_active(int interface)
{
	switch (interface) {
#if defined(CONFIG_ESDHCI_MXC_SELECT1)
	case 0:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_mmc1_pads,
						 ARRAY_SIZE(ccimx53_mmc1_pads));
		break;
#endif /* CONFIG_ESDHCI_MXC_SELECT1 */
#if defined(CONFIG_ESDHCI_MXC_SELECT2)
	case 1:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_mmc2_pads,
						 ARRAY_SIZE(ccimx53_mmc2_pads));
		break;
#endif /* CONFIG_ESDHCI_MXC_SELECT2 */
#if defined(CONFIG_ESDHCI_MXC_SELECT3)
	case 2:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_mmc3_pads,
						 ARRAY_SIZE(ccimx53_mmc3_pads));
		break;
#endif /* CONFIG_ESDHCI_MXC_SELECT2 */
#if defined(CONFIG_ESDHCI_MXC_SELECT4)
	case 3:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_mmc4_pads,
						 ARRAY_SIZE(ccimx53_mmc4_pads));
		break;
#endif /* CONFIG_ESDHCI_MXC_SELECT4 */
	}
}
EXPORT_SYMBOL(gpio_sdhc_active);

#if defined(CONFIG_ESDHCI_MXC_SELECT1)
static iomux_v3_cfg_t ccimx53_mmc1_gpio_pads[] = {
	MX53_PAD_SD1_DATA0__GPIO1_16,
	MX53_PAD_SD1_CMD__GPIO1_18,
	MX53_PAD_SD1_DATA0__GPIO1_16,
#if defined(ESDHCI_MXC_SELECT1_8BIT_PORT) || defined(ESDHCI_MXC_SELECT1_4BIT_PORT)
	MX53_PAD_SD1_DATA1__GPIO1_17,
	MX53_PAD_SD1_DATA2__GPIO1_19,
	MX53_PAD_SD1_DATA3__GPIO1_21,
#endif /* (ESDHCI_MXC_SELECT1_8BIT_PORT) || (ESDHCI_MXC_SELECT1_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT1_8BIT_PORT
	MX53_PAD_PATA_DATA8__GPIO2_8,
	MX53_PAD_PATA_DATA9__GPIO2_9,
	MX53_PAD_PATA_DATA10__GPIO2_10,
	MX53_PAD_PATA_DATA11__GPIO2_11,
#endif /* ESDHCI_MXC_SELECT1_8BIT_PORT */
};
#endif /* ESDHCI_MXC_SELECT1 */

void gpio_sdhc_inactive(int interface)
{
	switch (interface) {
#if defined(CONFIG_ESDHCI_MXC_SELECT1)
	case 0:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_mmc1_gpio_pads,
						 ARRAY_SIZE(ccimx53_mmc1_gpio_pads));
		break;
#endif /* CONFIG_ESDHCI_MXC_SELECT1 */
	}
}
EXPORT_SYMBOL(gpio_sdhc_inactive);


static iomux_v3_cfg_t ccimx53_audio_pads[] = {
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
};

void gpio_activate_audio_ports( void )
{
#if defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000) || defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000_MODULE)
	mxc_iomux_v3_setup_multiple_pads(ccimx53_audio_pads,ARRAY_SIZE(ccimx53_audio_pads));
#endif
}
EXPORT_SYMBOL(gpio_activate_audio_ports);


#if defined(CONFIG_WIRELESS)
static iomux_v3_cfg_t ccimx53_wireless_pads[] = {
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
};

void gpio_wireless_active(void)
{
	/* Wireless module is connected to SD1 interface */
	mxc_iomux_v3_setup_multiple_pads(ccimx53_wireless_pads,
					 ARRAY_SIZE(ccimx53_wireless_pads));
}
#else
void gpio_wireless_active(void) {}
#endif


#if defined(CONFIG_CCIMX5X_DISP0)
static iomux_v3_cfg_t ccimx53_disp0_pads[] = {
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
#if defined(CONFIG_CCIMX5X_DISP0_RGB888)
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
#endif
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
#if defined(DISP1_ENABLE_PAD)
	DISP1_ENABLE_PAD,
#endif
};
#endif /* CONFIG_CCIMX53_DISP0 */

void gpio_video_active(int vif, u32 pad)
{
	int i;

#define DSE_MASK	((iomux_v3_cfg_t)PAD_CTL_DSE_MAX << MUX_PAD_CTRL_SHIFT)

#if defined(CONFIG_CCIMX5X_DISP0)
	if( vif == 0 ){
		for (i = 0; i < (ARRAY_SIZE(ccimx53_disp0_pads) - 1); i++) {
			ccimx53_disp0_pads[i] = (ccimx53_disp0_pads[i] & ~DSE_MASK) |
						((iomux_v3_cfg_t)pad << MUX_PAD_CTRL_SHIFT);
		}

		mxc_iomux_v3_setup_multiple_pads(ccimx53_disp0_pads,
						 ARRAY_SIZE(ccimx53_disp0_pads));
#if defined(DISP1_ENABLE_PAD) && defined(DISP1_ENABLE_GPIO)
		gpio_request(DISP1_ENABLE_GPIO, "lcd1_enable");
		gpio_direction_output(DISP1_ENABLE_GPIO, 0);
#endif /* DISP1_ENABLE_PAD && DISP1_ENABLE_GPIO */
	}
#endif /* CONFIG_CCIMX5X_DISP0 */
}

void gpio_video_inactive(int vif, u32 pad)
{
}
EXPORT_SYMBOL(gpio_video_active);
EXPORT_SYMBOL(gpio_video_inactive);


#if defined(CONFIG_CCIMX5X_DISP0)
static iomux_v3_cfg_t ccimx53_lvds0_pads[] = {
#if defined(CONFIG_CCIMX5X_DISP0_RGB888)
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
#endif
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
};
#endif /* CONFIG_CCIMX5X_DISP0 */

#if defined(CONFIG_CCIMX5X_DISP1)
static iomux_v3_cfg_t ccimx53_lvds1_pads[] = {
#if defined(CONFIG_CCIMX5X_DISP1_RGB888)
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
#endif
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
};
#endif /* CONFIG_CCIMX5X_DISP1 */

void gpio_lvds_active(int interf)
{
#if defined(CONFIG_CCIMX5X_DISP0)
	if (interf == 0) {
		mxc_iomux_v3_setup_multiple_pads(ccimx53_lvds0_pads,
						 ARRAY_SIZE(ccimx53_lvds0_pads));
	}
#endif /* CONFIG_CCIMX5X_DISP0 */
#if defined(CONFIG_CCIMX5X_DISP1)
	if (interf == 1) {
		mxc_iomux_v3_setup_multiple_pads(ccimx53_lvds1_pads,
						 ARRAY_SIZE(ccimx53_lvds1_pads));
	}
#endif /* CONFIG_CCIMX5X_DISP1 */
}

void gpio_lvds_inactive(int interf)
{
}
EXPORT_SYMBOL(gpio_lvds_active);
EXPORT_SYMBOL(gpio_lvds_inactive);

#if defined(CONFIG_CCIMX5X_DISP1)
static iomux_v3_cfg_t ccimx53_vga_pads[] = {
	/* VSYNC */
	MX53_PAD_EIM_DA12__IPU_DI1_PIN3,
	/* HSYNC */
	MX53_PAD_EIM_DA11__IPU_DI1_PIN2,
};
#endif /* CONFIG_CCIMX5X_DISP1 */

void gpio_vga_active(int interf)
{
#if defined(CONFIG_CCIMX5X_DISP0)
	if (interf == 0) {
		printk(KERN_INFO "%s: VGA interface not supported on video DISP0\n", __func__);
	}
#endif /* CONFIG_CCIMX5X_DISP0 */
#if defined(CONFIG_CCIMX5X_DISP1)
	if (interf == 1) {
		mxc_iomux_v3_setup_multiple_pads(ccimx53_vga_pads,
						 ARRAY_SIZE(ccimx53_vga_pads));
	}
#endif /* CONFIG_CCIMX5X_DISP1 */
}

void gpio_vga_inactive(int interf)
{
}
EXPORT_SYMBOL(gpio_vga_active);
EXPORT_SYMBOL(gpio_vga_inactive);

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
static iomux_v3_cfg_t ccimx53_fec_pads[] = {
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
};

void gpio_fec_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccimx53_fec_pads,
					 ARRAY_SIZE(ccimx53_fec_pads));
}
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static iomux_v3_cfg_t ccimx53_smsc911x_pads[] = {
	MX53_PAD_EIM_EB3__GPIO2_31,
	MX53_PAD_EIM_D16__EMI_WEIM_D_16,
	MX53_PAD_EIM_D17__EMI_WEIM_D_17,
	MX53_PAD_EIM_D18__EMI_WEIM_D_18,
	MX53_PAD_EIM_D19__EMI_WEIM_D_19,
	MX53_PAD_EIM_D20__EMI_WEIM_D_20,
	MX53_PAD_EIM_D21__EMI_WEIM_D_21,
	MX53_PAD_EIM_D22__EMI_WEIM_D_22,
	MX53_PAD_EIM_D23__EMI_WEIM_D_23,
	MX53_PAD_EIM_D24__EMI_WEIM_D_24,
	MX53_PAD_EIM_D25__EMI_WEIM_D_25,
	MX53_PAD_EIM_D26__EMI_WEIM_D_26,
	MX53_PAD_EIM_D27__EMI_WEIM_D_27,
	MX53_PAD_EIM_D28__EMI_WEIM_D_28,
	MX53_PAD_EIM_D29__EMI_WEIM_D_29,
	MX53_PAD_EIM_D30__EMI_WEIM_D_30,
	MX53_PAD_EIM_D31__EMI_WEIM_D_31,
	MX53_PAD_EIM_DA0__EMI_NAND_WEIM_DA_0,
	MX53_PAD_EIM_DA1__EMI_NAND_WEIM_DA_1,
	MX53_PAD_EIM_DA2__EMI_NAND_WEIM_DA_2,
	MX53_PAD_EIM_DA3__EMI_NAND_WEIM_DA_3,
	MX53_PAD_EIM_DA4__EMI_NAND_WEIM_DA_4,
	MX53_PAD_EIM_DA5__EMI_NAND_WEIM_DA_5,
	MX53_PAD_EIM_DA6__EMI_NAND_WEIM_DA_6,
	_MX53_PAD_EIM_OE__EMI_WEIM_OE,
	_MX53_PAD_EIM_RW__EMI_WEIM_RW,
	_MX53_PAD_EIM_CS1__EMI_WEIM_CS_1,
	MX53_PAD_GPIO_12__GPIO4_2,     /* IRQ */
};

void gpio_smsc911x_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccimx53_smsc911x_pads,
					 ARRAY_SIZE(ccimx53_smsc911x_pads));

	/* Configure interrupt line as GPIO input, the iomux should be already setup */
	gpio_request(CCIMX53_EXT_IRQ_GPIO, "ext-eth-irq");
	gpio_direction_input(CCIMX53_EXT_IRQ_GPIO);
}
#endif

#if (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))

#ifdef CONFIG_SPI_MXC_SELECT1
static iomux_v3_cfg_t ccimx53_spi1_pads[] = {
		MX53_PAD_CSI0_DAT7__ECSPI1_SS0,
		MX53_PAD_CSI0_DAT4__ECSPI1_SCLK,
		MX53_PAD_CSI0_DAT6__ECSPI1_MISO,
		MX53_PAD_CSI0_DAT5__ECSPI1_MOSI,
		MX53_PAD_GPIO_19__GPIO4_5,
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
		MX53_PAD_CSI0_DAT7__GPIO5_25,
		MX53_PAD_CSI0_DAT10__GPIO5_28,
#endif
};
#endif

#ifdef CONFIG_SPI_MXC_SELECT2
static iomux_v3_cfg_t ccimx53_spi2_pads[] = {
	// Depends on the hardware. On the CCIMX53JS these are not available.
};
#endif

#ifdef CONFIG_SPI_MXC_SELECT3
static iomux_v3_cfg_t ccimx53_spi3_pads[] = {
	// Depends on the hardware. On the CCIMX53JS these are not available.
};
#endif

void gpio_spi_active(void)
{
#ifdef CONFIG_SPI_MXC_SELECT1
	mxc_iomux_v3_setup_multiple_pads(ccimx53_spi1_pads,
					 ARRAY_SIZE(ccimx53_spi1_pads));
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
	/* Configure external touch interrupt line */
	gpio_request(SECOND_TS_IRQ_PIN, "ts2_irq");
	gpio_direction_input(SECOND_TS_IRQ_PIN);

	/* Configure the Slave Select signal as gpio */
	gpio_request(SECOND_TS_SPI_SS_PIN, "ts2_spi_ss");
	gpio_direction_output(SECOND_TS_SPI_SS_PIN, 1);
	gpio_set_value(SECOND_TS_SPI_SS_PIN, 1);
#endif
#endif
#ifdef CONFIG_SPI_MXC_SELECT2
	mxc_iomux_v3_setup_multiple_pads(ccimx53_spi2_pads,
					 ARRAY_SIZE(ccimx53_spi1_pads));
#endif
#ifdef CONFIG_SPI_MXC_SELECT3
	mxc_iomux_v3_setup_multiple_pads(ccimx53_spi3_pads,
					 ARRAY_SIZE(ccimx53_spi1_pads));
#endif
}

void ccimx53_gpio_spi_chipselect_active(int busnum, int ssb_pol, int chipselect)
{
	/* Deassert/Assert the different CS lines for the different buses */
	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(SECOND_TS_SPI_SS_PIN, 0);
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
EXPORT_SYMBOL(ccimx53_gpio_spi_chipselect_active);

void ccimx53_gpio_spi_chipselect_inactive(int busnum, int ssb_pol,
					  int chipselect)
{
	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(SECOND_TS_SPI_SS_PIN, 1);
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
EXPORT_SYMBOL(ccimx53_gpio_spi_chipselect_inactive);
#endif

/* CAN */
#ifdef CONFIG_CCIMX53_CAN1
static iomux_v3_cfg_t ccimx53_can0_pads[] = {
	MX53_PAD_GPIO_7__CAN1_TXCAN,
	MX53_PAD_GPIO_8__CAN1_RXCAN,
};
#endif

#ifdef CONFIG_CCIMX53_CAN2
static iomux_v3_cfg_t ccimx53_can1_pads[] = {
	MX53_PAD_KEY_COL4__CAN2_TXCAN,
	MX53_PAD_KEY_ROW4__CAN2_RXCAN,
};
#endif

void gpio_can_active(int interface)
{
	switch (interface) {
#ifdef CONFIG_CCIMX53_CAN1
	case 0:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_can0_pads,
						 ARRAY_SIZE(ccimx53_can0_pads));
		break;
#endif /* CONFIG_CCIMX53_CAN1 */
#ifdef CONFIG_CCIMX53_CAN2
	case 1:
		mxc_iomux_v3_setup_multiple_pads(ccimx53_can1_pads,
						 ARRAY_SIZE(ccimx53_can1_pads));
		break;
#endif /* CONFIG_CCIMX53_CAN2 */
	}
}
EXPORT_SYMBOL(gpio_can_active);
void gpio_can_inactive(int module) {}
EXPORT_SYMBOL(gpio_can_inactive);

#if defined CONFIG_VIDEO_MXC_IPU_CAMERA

#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
static iomux_v3_cfg_t ccimx53_cam1_pads[] = {
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_DAT9__GPIO5_27
};
#endif

#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
static iomux_v3_cfg_t ccimx53_cam2_pads[] = {
	MX53_PAD_EIM_A17__IPU_CSI1_D_12,
	MX53_PAD_EIM_A18__IPU_CSI1_D_13,
	MX53_PAD_EIM_A19__IPU_CSI1_D_14,
	MX53_PAD_EIM_A20__IPU_CSI1_D_15,
	MX53_PAD_EIM_A21__IPU_CSI1_D_16,
	MX53_PAD_EIM_A22__IPU_CSI1_D_17,
	MX53_PAD_EIM_A23__IPU_CSI1_D_18,
	MX53_PAD_EIM_A24__IPU_CSI1_D_19,
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	MX53_PAD_EIM_A16__IPU_CSI1_PIXCLK,
	MX53_PAD_EIM_DA11__IPU_CSI1_HSYNC,
	MX53_PAD_EIM_DA12__IPU_CSI1_VSYNC,
	MX53_PAD_CSI0_DAT8__GPIO5_26
};
#endif

void gpio_camera_active(void)
{
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	mxc_iomux_v3_setup_multiple_pads(ccimx53_cam1_pads, ARRAY_SIZE(ccimx53_cam1_pads));

	/* Camera 1 reset */
	gpio_request(MX53_GPIO(5,27), "gpio5_27");
	gpio_direction_output(MX53_GPIO(5,27), 0);
	// Take camera out of reset
	gpio_set_value(MX53_GPIO(5,27), 0);
	msleep(100);
	gpio_set_value(MX53_GPIO(5,27), 1);
	msleep(100);
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	mxc_iomux_v3_setup_multiple_pads(ccimx53_cam2_pads, ARRAY_SIZE(ccimx53_cam2_pads));

	/* Camera 2 reset */
	gpio_request(MX53_GPIO(5,26), "gpio5_26");
	gpio_direction_output(MX53_GPIO(5,26), 0);
	// Take camera out of reset
	gpio_set_value(MX53_GPIO(5,26), 0);
	msleep(100);
	gpio_set_value(MX53_GPIO(5,26), 1);
	msleep(100);
#endif
}
EXPORT_SYMBOL(gpio_camera_active);
#endif //CONFIG_VIDEO_MXC_IPU_CAMERA

static iomux_v3_cfg_t ccimx53_dio_pads[] = {
#if !defined CONFIG_CERTMX53_V1 && !defined(DISP1_ENABLE_PAD)
	MX53_PAD_NANDF_CS2__GPIO6_15,
#endif
	MX53_PAD_PATA_DA_0__GPIO7_6,
	MX53_PAD_NANDF_CS1__GPIO6_14,
	MX53_PAD_NANDF_CS3__GPIO6_16,
};

void gpio_dio_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccimx53_dio_pads, ARRAY_SIZE(ccimx53_dio_pads));
}
EXPORT_SYMBOL(gpio_dio_active);

static iomux_v3_cfg_t ccimx53_usb_pads[] = {
	MX53_PAD_CSI0_DAT11__GPIO5_29,
};

void gpio_usb_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccimx53_usb_pads, ARRAY_SIZE(ccimx53_usb_pads));

	gpio_request(CCIMX53_USB_HUB_RESET, "usb-hub-reset");
	gpio_direction_output(CCIMX53_USB_HUB_RESET, 0);
}
EXPORT_SYMBOL(gpio_usb_active);
