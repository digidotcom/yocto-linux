/*
 * arch/arm/mach-ns9xxx/cc9p9215_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/fim-ns921x.h>
#include <mach/hardware.h>
#include <mach/regs-sys-common.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-mem.h>

#include <video/hx8347fb.h>
#include <linux/fb.h>
#include <linux/mmc/host.h>
#include <linux/w1-gpio.h>

#include "ns921x_devices.h"
#include "cc9p9215_devices.h"
#include "clock.h"

/*
 * Pick Digi's internal FIM board
 * Use internal board, defined to 1
 * Use newer boards, defined to 0
 */
#if 0
#define INT_FIM_BOARD
#endif

#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
static int cc9p9215_phy_endisable(struct clk *clk, int enable)
{
	int ret;

	if (enable) {
		ret = gpio_request(90, "ns9xxx-eth-phy");
		if (ret)
			return ret;

		gpio_direction_output(90, 1);
	} else {
		gpio_set_value(90, 0);
		gpio_free(90);
	}

	return 0;
}

static struct clk phyclk = {
	.name		= "ns9xxx-eth-phy",
	.id		= -1,
	.owner		= THIS_MODULE,
	.endisable	= cc9p9215_phy_endisable,
};

void __init ns9xxx_add_device_cc9p9215_eth(void)
{
	int gpio[] = {32, 33, 34, 35, 36, 37, 38, 39, 40,
		      41, 42, 43, 44, 45, 46, 47, 48, 49};
	int func[] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
		      0, 0, 0, 0, 0, 0, 0, 0, 0};
	int dir[] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
		      0, 0, 0, 0, 0, 0, 0, 0, 0};

	if (clk_register(&phyclk))
		return;

	ns9xxx_add_device_ns921x_eth(&phyclk, 0, gpio, func,
				     dir, ARRAY_SIZE(gpio));
}
#else
void __init ns9xxx_add_device_cc9p9215_eth(void) {}
#endif

#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static void cc9p9215_i2c_gpio_reconfigure(void)
{
	gpio_configure_ns921x(102, 0, 0, 2, 0);
	gpio_configure_ns921x(103, 0, 0, 2, 0);
}

static struct plat_ns9xxx_i2c ns9xxx_device_cc9p9215_i2c_data = {
	.gpio_scl = 102,
	.gpio_sda = 103,
	.speed = 100000,	/* 100000Hz or 400000Hz */
	.gpio_configuration_func = cc9p9215_i2c_gpio_reconfigure,
};

void __init ns9xxx_add_device_cc9p9215_i2c(void)
{
	ns9xxx_add_device_ns921x_i2c(&ns9xxx_device_cc9p9215_i2c_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_i2c(void) {}
#endif

#if defined(CONFIG_SERIAL_NS921X) || defined(CONFIG_SERIAL_NS921X_MODULE)
void __init ns9xxx_add_device_cc9p9215_uarta(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uarta(0, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartb(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartb(51, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartc(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartc(8, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartd(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartd(59, gpio_nr, 0);
}
#else
void __init ns9xxx_add_device_cc9p9215_uarta(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartb(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartc(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartd(int gpio_nr) {}
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP)
static struct physmap_flash_data ns9xxx_device_cc9p9215_flash_data = {
	.width	= 2,
};

void __init ns9xxx_add_device_cc9p9215_flash(void)
{
	ns9xxx_add_device_ns921x_flash(&ns9xxx_device_cc9p9215_flash_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_flash(void) {}
#endif

#if defined(CONFIG_SPI_NS921X) || defined(CONFIG_SPI_NS921X_MODULE)
/* SPI ports and their related GPIOs */
static struct spi_ns9xxx_data ns9xxx_device_cc9p9215_spi_data = {
	.gpios = {7, 3, 5, 0},
	.gpio_funcs = { NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4 },
	.nr_gpios = 4,
};

void __init ns9xxx_add_device_cc9p9215_spi(void) {
	ns9xxx_add_device_ns921x_spi(&ns9xxx_device_cc9p9215_spi_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_spi(void) {}
#endif


#if defined(CONFIG_FIM_ZERO_SERIAL)
static struct fim_serial_platform_data fim_serial_data0 = {
	.fim_nr        = 0,
#if defined(CONFIG_FIM_ZERO_SERIAL_CTSRTS)
	NS921X_FIM_SERIAL_GPIOS(69, 68, /* RX + TX */
				70, 71, /* RTS + CTS */
				NS921X_GPIO_FUNC_0),
#else
	NS921X_FIM_SERIAL_GPIOS(69, 68, /* RX + TX */
				FIM_GPIO_DONT_USE, /* RTS */
				FIM_GPIO_DONT_USE, /* CTS */
				NS921X_GPIO_FUNC_0),
#endif
};
struct platform_device ns921x_fim_serial0 = {
	.name              = "fim-serial",
	.id                = 0,
	.dev.platform_data = &fim_serial_data0,
};
EXPORT_SYMBOL(ns921x_fim_serial0);
#endif /* CONFIG_FIM_ZERO_SERIAL */

#if defined(CONFIG_FIM_ONE_SERIAL)
static struct fim_serial_platform_data fim_serial_data1 = {
	.fim_nr        = 1,
#if defined(CONFIG_FIM_ONE_SERIAL_CTSRTS)
	NS921X_FIM_SERIAL_GPIOS(73, 72, /* RX + TX */
				74, 75, /* RTS + CTS */
				NS921X_GPIO_FUNC_1),
#else
	NS921X_FIM_SERIAL_GPIOS(73, 72, /* RX + TX */
				FIM_GPIO_DONT_USE, /* RTS */
				FIM_GPIO_DONT_USE, /* CTS */
				NS921X_GPIO_FUNC_1),
#endif
};
struct platform_device ns921x_fim_serial1 = {
	.name              = "fim-serial",
	.id                = 1,
	.dev.platform_data = &fim_serial_data1,
};
EXPORT_SYMBOL(ns921x_fim_serial1);
#endif /* CONFIG_FIM_ONE_SERIAL */


#if defined(CONFIG_FIM_ZERO_SDIO)
static struct fim_sdio_platform_data fim_sdio_data0 = {
	.fim_nr        = 0,
	.host_caps     = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ,
	.min_clk       = 320000,
	.max_clk       = 5000000,
#if defined(INT_FIM_BOARD)
	NS921X_FIM_SDIO_GPIOS(68, 69, 70, 71, /* D0 to D3 */
			      72, 73,         /* WP + CD */
			      76, 77,         /* CLK + CMD */
			      NS921X_GPIO_FUNC_0),
#else
	NS921X_FIM_SDIO_GPIOS_FIM(68, 69, 70, 71, /* D0 to D3 */
				  76, 77,         /* CLK + CMD */
				  NS921X_GPIO_FUNC_0),
#if defined(CONFIG_FIM_APPKIT_BOARD)
	.cd_gpio_nr   = 101,         /* CD as external interrupt */
	.cd_gpio_func = NS921X_GPIO_FUNC_2,

	.wp_gpio_nr   = 100,         /* WP as normal GPIO */
	.wp_gpio_func = NS921X_GPIO_FUNC_GPIO,
#else
#if defined(CONFIG_FIM_ZERO_SDIO_CD)
	/* CD GPIO and function provided by config */
	.cd_gpio_nr   = CONFIG_FIM_ZERO_SDIO_CD_GPIO,
	.cd_gpio_func = CONFIG_FIM_ZERO_SDIO_CD_GPIO_FUNC,
#else
	.cd_gpio_nr   = FIM_GPIO_DONT_USE,
#endif
#if defined(CONFIG_FIM_ZERO_SDIO_WP)
	/* WP GPIO provided by config. Assume standard GPIO function */
	.wp_gpio_nr   = CONFIG_FIM_ZERO_SDIO_WP_GPIO,
	.wp_gpio_func = NS921X_GPIO_FUNC_GPIO,
#else
	.wp_gpio_nr   = FIM_GPIO_DONT_USE,
#endif
#endif /* CONFIG_FIM_APPKIT_BOARD */
#endif /* CONFIG_FIM_ZERO_SDIO */
};
struct platform_device ns921x_fim_sdio0 = {
	.name              = "fim-sdio",
	.id                = 0,
	.dev.platform_data = &fim_sdio_data0,
};
EXPORT_SYMBOL(ns921x_fim_sdio0);
#endif /* CONFIG_FIM_ZERO_SDIO */


#if defined(CONFIG_FIM_ONE_SDIO)
static struct fim_sdio_platform_data fim_sdio_data1 = {
	.fim_nr        = 1,
	.host_caps     = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ,
	.min_clk       = 320000,
	.max_clk       = 5000000,
#if defined(INT_FIM_BOARD)
	NS921X_FIM_SDIO_GPIOS(68, 69, 70, 71, /* D0 to D3 */
			      72, 73,         /* WP + CD */
			      76, 77,         /* CLK + CMD */
			      NS921X_GPIO_FUNC_1),
#else
	NS921X_FIM_SDIO_GPIOS_FIM(72, 73, 74, 75, /* D0 to D3 */
				  78, 79,         /* CLK + CMD */
				  NS921X_GPIO_FUNC_1),
#if defined(CONFIG_FIM_APPKIT_BOARD)
	.cd_gpio_nr   = 101,         /* CD as external interrupt */
	.cd_gpio_func = NS921X_GPIO_FUNC_2,

	.wp_gpio_nr   = 100,         /* WP as normal GPIO */
	.wp_gpio_func = NS921X_GPIO_FUNC_GPIO,
#else
#if defined(CONFIG_FIM_ZERO_SDIO_CD)
	/* CD GPIO and function provided by config */
	.cd_gpio_nr   = CONFIG_FIM_ONE_SDIO_CD_GPIO,
	.cd_gpio_func = CONFIG_FIM_ONE_SDIO_CD_GPIO_FUNC,
#else
	.cd_gpio_nr   = FIM_GPIO_DONT_USE,
#endif
#if defined(CONFIG_FIM_ZERO_SDIO_WP)
	/* WP GPIO provided by config. Assume standard GPIO function */
	.wp_gpio_nr   = CONFIG_FIM_ONE_SDIO_WP_GPIO,
	.wp_gpio_func = NS921X_GPIO_FUNC_GPIO,
#else
	.wp_gpio_nr   = FIM_GPIO_DONT_USE,
#endif
#endif /* CONFIG_FIM_APPKIT_BOARD */
#endif /* CONFIG_FIM_ONE_SDIO */
};
struct platform_device ns921x_fim_sdio1 = {
	.name              = "fim-sdio",
	.id                = 1,
	.dev.platform_data = &fim_sdio_data1,
};
EXPORT_SYMBOL(ns921x_fim_sdio1);
#endif /* CONFIG_FIM_ONE_SDIO */

#if defined(CONFIG_FIM_ZERO_SPI)
static struct spi_ns921x_fim spi_fim0_data = {
        .fim_nr             = 0,
#if defined(CONFIG_FIM_ZERO_SPI_WANT_MASTER_CS)
        .flags              = SPI_NS921X_SUPPORT_MASTER_CS,
#else
        .flags              = 0,
#endif
#if defined(CONFIG_FIM_ZERO_SPI_GPIO_68_TO_71)
	.gpio_base = 68,
#else
	.gpio_base = 0,
#endif
#if defined(CONFIG_FIM_ZERO_SPI_CS_0_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(0, true, CONFIG_FIM_ZERO_SPI_CS_0),
#else
        NS921X_FIM_SPI_CS_GPIOS(0, false, 0),
#endif
#if defined(CONFIG_FIM_ZERO_SPI_CS_1_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(1, true, CONFIG_FIM_ZERO_SPI_CS_1),
#else
        NS921X_FIM_SPI_CS_GPIOS(1, false, 0),
#endif
#if defined(CONFIG_FIM_ZERO_SPI_CS_2_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(2, true, CONFIG_FIM_ZERO_SPI_CS_2),
#else
        NS921X_FIM_SPI_CS_GPIOS(2, false, 0),
#endif
#if defined(CONFIG_FIM_ZERO_SPI_CS_3_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(3, true, CONFIG_FIM_ZERO_SPI_CS_3),
#else
        NS921X_FIM_SPI_CS_GPIOS(3, false, 0),
#endif
};

struct platform_device ns921x_fim_spi0 = {
    .name                   = "fim-spi",
    .id                     = 2,        /* The internal SPI ports are 0 and 1, so the
                                           FIM SPI ports are numbered 2 and 3*/
    .dev.platform_data      = &spi_fim0_data
};
EXPORT_SYMBOL(ns921x_fim_spi0);
#endif /* CONFIG_FIM_ZERO_SPI */

#if defined(CONFIG_FIM_ONE_SPI)
static struct spi_ns921x_fim spi_fim1_data = {
        .fim_nr             = 1,
#if defined(CONFIG_FIM_ONE_SPI_WANT_MASTER_CS)
        .flags              = SPI_NS921X_SUPPORT_MASTER_CS,
#else
        .flags              = 0,
#endif
        .gpio_base = 68,
#if defined(CONFIG_FIM_ONE_SPI_CS_0_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(0, true, CONFIG_FIM_ONE_SPI_CS_0),
#else
        NS921X_FIM_SPI_CS_GPIOS(0, false, 0),
#endif
#if defined(CONFIG_FIM_ONE_SPI_CS_1_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(1, true, CONFIG_FIM_ONE_SPI_CS_1),
#else
        NS921X_FIM_SPI_CS_GPIOS(1, false, 0),
#endif
#if defined(CONFIG_FIM_ONE_SPI_CS_2_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(2, true, CONFIG_FIM_ONE_SPI_CS_2),
#else
        NS921X_FIM_SPI_CS_GPIOS(2, false, 0),
#endif
#if defined(CONFIG_FIM_ONE_SPI_CS_3_ENABLED)
        NS921X_FIM_SPI_CS_GPIOS(3, true, CONFIG_FIM_ONE_SPI_CS_3),
#else
        NS921X_FIM_SPI_CS_GPIOS(3, false, 0),
#endif
};

struct platform_device ns921x_fim_spi1 = {
    .name                   = "fim-spi",
    .id                     = 3,
    .dev.platform_data      = &spi_fim1_data
};
EXPORT_SYMBOL(ns921x_fim_spi1);
#endif /* CONFIG_FIM_ONE_SPI */

#if defined(CONFIG_FIM_ZERO_CAN)
static struct fim_can_platform_data fim_can_data0 = {
	.fim_nr			= 0,
	.fim_can_bitrate	= CONFIG_FIM_CAN_DEFAULT_BITRATE,
	NS921X_FIM_CAN_GPIOS(   96, 97, /* RX + TX */
				NS921X_GPIO_FUNC_2),
};
struct platform_device ns921x_fim_can0 = {
	.name              = "fim-can",
	.id                = 0,
	.dev.platform_data = &fim_can_data0,
};
EXPORT_SYMBOL(ns921x_fim_can0);
#endif /* CONFIG_FIM_ZERO_CAN */

#if defined(CONFIG_FIM_ONE_CAN)
static struct fim_can_platform_data fim_can_data1 = {
	.fim_nr			= 1,
	.fim_can_bitrate	= CONFIG_FIM_CAN_DEFAULT_BITRATE,
#if defined(INT_FIM_BOARD)
	NS921X_FIM_CAN_GPIOS(   68, 69, /* RX + TX */
				NS921X_GPIO_FUNC_2),
#else
	NS921X_FIM_CAN_GPIOS(   98, 99, /* RX + TX */
				NS921X_GPIO_FUNC_2),
#endif
};
struct platform_device ns921x_fim_can1 = {
	.name              = "fim-can",
	.id                = 1,
	.dev.platform_data = &fim_can_data1,
};
EXPORT_SYMBOL(ns921x_fim_can1);
#endif /* CONFIG_FIM_ONE_CAN */

#if defined(CONFIG_FIM_ZERO_W1)
static struct w1_gpio_platform_data fim_w1_data0 = {
	.pin           = 68,
	.is_open_drain = 0,
};
struct platform_device ns921x_fim0_w1 = {
	.name		= "w1-gpio",
	.id		= 0,
	.dev		= {
		.platform_data		= &fim_w1_data0,
	},
};
EXPORT_SYMBOL(ns921x_fim0_w1);
#endif /* CONFIG_FIM_ZERO_W1 */

#if defined(CONFIG_FIM_ONE_W1)
static struct w1_gpio_platform_data fim_w1_data1 = {
	.pin           = 72,
	.is_open_drain = 0,
};
struct platform_device ns921x_fim1_w1 = {
	.name		= "w1-gpio",
	.id		= 1,
	.dev		= {
		.platform_data		= &fim_w1_data1,
	},
};
EXPORT_SYMBOL(ns921x_fim1_w1);
#endif /* CONFIG_FIM_ONE_W1 */

#if defined(CONFIG_FIM_ZERO_USB)
static struct fim_usb_platform_data fim_usb_data0 = {
        .fim_nr            = 0,

	/* The setup of the pull-ups must be fixed in the driver, however print a warning */
#if defined(INT_FIM_BOARD)
#warning "Internal FIM board has a different pull-ups configuration"
	NS921X_FIM_USB_GPIOS(68, 69, 70, 71,    /* VP + VM + RCV + OE_L */
			     72, 73,            /* ENUM + SPND */
                             NS921X_GPIO_FUNC_0,
                             NS921X_GPIO_FUNC_GPIO),
#else
        NS921X_FIM_USB_GPIOS(68, 69, 70, 71,    /* VP + VM + RCV + OE_L */
                             76, 77,            /* ENUM + SPND */
                             NS921X_GPIO_FUNC_0,
                             NS921X_GPIO_FUNC_GPIO),
#endif
};
struct platform_device ns921x_fim_usb0 = {
        .name              = "fim-usb",
        .id                = 0,
        .dev.platform_data = &fim_usb_data0,
};
EXPORT_SYMBOL(ns921x_fim_usb0);
#endif /* CONFIG_FIM_ZERO_USB */

#if defined(CONFIG_FIM_ONE_USB)
static struct fim_usb_platform_data fim_usb_data1 = {
	.fim_nr            = 1,
	NS921X_FIM_USB_GPIOS(72, 73, 74, 75,    /* VP + VM + RCV + OE_L */
			     78, 79,            /* ENUM + SPND */
			     NS921X_GPIO_FUNC_1,
			     NS921X_GPIO_FUNC_GPIO),
};
struct platform_device ns921x_fim_usb1 = {
	.name              = "fim-usb",
	.id                = 1,
	.dev.platform_data = &fim_usb_data1,
};
EXPORT_SYMBOL(ns921x_fim_usb1);
#endif /* CONFIG_FIM_ONE_USB */

#if defined(CONFIG_CC9P9215JS_EDT_DISPLAY_QVGA)
#ifdef CONFIG_CC9P9215JS_DISPLAY_USES_DMA
#include <mach/dma-ns921x.h>
struct ext_dma_desc_t dmadesc[3];
#endif

void __init cc9p9215_edt_qvga_lcd_setup_cs(void)
{
	/* LCD required the extended wait register to be set */
	writel(3, MEM_SMEW);
	/* 16 bit bus width and enable exteneded wait */
	writel(MEM_SMC_PB_1 | MEM_SMC_EW_ON | MEM_SMC_MW_16, MEM_SMC(0));
	/* Static Memory Write Enable Delay x */
	writel(4, MEM_SMWED(0));
	/* Static Memory Output Enable Delay x */
	writel(0, MEM_SMOED(0));
	/* Static Memory Read Delay x */
	writel(0, MEM_SMRD(0));
	/* Static Memory Page Mode Read Delay 0 */
	writel(0, MEM_SMPMRD(0));
	/* Static Memory Write Delay */
	writel(0, MEM_SMWD(0));
	/* Static Memory Turn Round Delay x */
	writel(0, MEM_SWT(0));
	/* Enable the CS0 access */
	writel(readl(SYS_SMCSSMM(0)) | SYS_SMCSSMM_CSEx_EN, SYS_SMCSSMM(0));
}

int cc9p9215_edt_qvga_lcd_register_gpios(struct hx8347fb_pdata *pdata)
{
	if (gpio_request(pdata->rst_gpio, "lcd-rst"))
		goto err;

	gpio_direction_output(pdata->rst_gpio, 0);

	if (gpio_request(pdata->enable_gpio, "lcd-enable"))
		goto err1;

	gpio_direction_output(pdata->enable_gpio, 0);

	return 0;
err1:
	gpio_free(pdata->rst_gpio);
err:
	return -EBUSY;
}

/* Configuration for the EDT QVGA display, most of the settings have
 * been taken from a Himax application note */
unsigned char edt_qvga_lcd_init[][3] = {
	/* Index, value, delay to write next register in ms*/
	{0x46, 0x94, 0},
	{0x47, 0x41, 0},
	{0x48, 0x00, 0},
	{0x49, 0x33, 0},
	{0x4a, 0x23, 0},
	{0x4b, 0x45, 0},
	{0x4c, 0x44, 0},
	{0x4d, 0x77, 0},
	{0x4e, 0x12, 0},
	{0x4f, 0xcc, 0},
	{0x50, 0x46, 0},
	{0x51, 0x82, 0},
	{0x02, 0x00, 0},	/* Column address start 2 */
	{0x03, 0x00, 0},	/* Column address start 1 */
	{0x04, 0x01, 0},	/* Column address end 2 */
	{0x05, 0x3f, 0},	/* Column address end 1 */
	{0x06, 0x00, 0},	/* Row address start 2 */
	{0x07, 0x00, 0},	/* Row address start 1 */
	{0x08, 0x00, 0},	/* Row address end 2 */
	{0x09, 0xef, 0},	/* Row address end 1 */
	{0x01, 0x06, 0},
	{0x16, 0x68, 0},
	{0x23, 0x95, 0},
	{0x24, 0x95, 0},
	{0x25, 0xff, 0},
	{0x27, 0x02, 0},
	{0x28, 0x02, 0},
	{0x29, 0x02, 0},
	{0x2a, 0x02, 0},
	{0x2c, 0x02, 0},
	{0x2d, 0x02, 0},
	{0x3a, 0x01, 0},
	{0x3b, 0x01, 0},
	{0x3c, 0xf0, 0},
	{0x3d, 0x00, 20},
	{0x35, 0x38, 0},
	{0x36, 0x78, 0},
	{0x3e, 0x38, 0},
	{0x40, 0x0f, 0},
	{0x41, 0xf0, 0},
	{0x19, 0x49, 0},
	{0x93, 0x0f, 10},
	{0x20, 0x40, 0},
	{0x1d, 0x07, 0},
	{0x1e, 0x00, 0},
	{0x1f, 0x04, 0},
	{0x44, 0x40, 0},
	{0x45, 0x12, 10},
	{0x1c, 0x04, 20},
	{0x43, 0x80, 5},
	{0x1b, 0x08, 40},
	{0x1b, 0x10, 40},
	{0x90, 0x7f, 0},
	{0x26, 0x04, 40},
	{0x26, 0x24, 0},
	{0x26, 0x2c, 40},
	{0x26, 0x3c, 0},
	{0x57, 0x02, 0},
	{0x55, 0x00, 0},
	{0x57, 0x00, 0}
};

static void cc9p9215_lcd_reset(struct hx8347fb_par *par)
{
	gpio_set_value(par->pdata->rst_gpio, 0);
	mdelay(100);
	gpio_set_value(par->pdata->rst_gpio, 1);
}

static void cc9p9215_lcd_enable(struct hx8347fb_par *par, int state)
{
	gpio_set_value(par->pdata->enable_gpio, state);
}

static void cc9p9215_lcd_set_idx(struct hx8347fb_par *par, u8 idx)
{
	writeb(idx, par->mmio_cmd);
}

static void cc9p9215_lcd_wr_reg(struct hx8347fb_par *par, u8 reg, u16 data)
{
	writeb(reg, par->mmio_cmd);
	writew(data, par->mmio_data);
}

#ifdef CONFIG_CC9P9215JS_DISPLAY_USES_DMA
static irqreturn_t fb_extdma(int irq, void *dev_id)
{
	u32 status;

	status = readl(NS921X_DMA_STIE(1));

	if (status & (NS921X_DMA_STIE_NCIP | NS921X_DMA_STIE_NRIP)) {
		writel(status, NS921X_DMA_STIE(1));
	}

	return IRQ_HANDLED;
}
#endif

static void cc9p9215_lcd_wr_data(struct hx8347fb_par *par, u16 *buf, int len)
{
#ifdef CONFIG_CC9P9215JS_DISPLAY_USES_DMA
#if 0
	/* CE=0, CG=1, SW=16bit, DW=16bit, SB=16 bytes, DB=1 unit, SAI=0, DAI=1 */
	writel(NS921X_DMA_CR_CG | NS921X_DMA_CR_SW_16b | \
	       NS921X_DMA_CR_DW_16b | NS921X_DMA_CR_SB_16B | \
	       NS921X_DMA_CR_DB_1B | NS921X_DMA_CR_DINC_N, NS921X_DMA_CR(1));
        /* CE=1, CG=1, SW=16bit, DW=16bit, SB=16 bytes, DB=1 unit, SAI=0, DAI=1 */
	writel(NS921X_DMA_CR_CE | readl(NS921X_DMA_CR(1)), NS921X_DMA_CR(1));*/*/*/
#endif
#else
	int i, wcycles;
	u16 *data = buf;

	wcycles = len >> 1;

	for (i = 0; i < wcycles; i++)
		writew(*data++, par->mmio_data);
#endif
}

static int cc9p9215_lcd_init(struct hx8347fb_par *par)
{
	int i, ret = 0;
	u8 reg;

	if (cc9p9215_edt_qvga_lcd_register_gpios(par->pdata))
		return -EINVAL;

	cc9p9215_lcd_enable(par, 0);
	cc9p9215_lcd_reset(par);
	cc9p9215_edt_qvga_lcd_setup_cs();

	mdelay(50);

	writeb(HIMAX_ID_CODE, par->mmio_cmd);
	if ((reg = readb(par->mmio_data)) != 0x47) {
		pr_debug("%s: HX8347 controller not detected REG[0x67] = 0x%02x\n", __func__, reg);
		ret = -EIO;
		goto err_detect;
	}

#ifdef CONFIG_CC9P9215JS_DISPLAY_USES_DMA
	writel(SYS_CLOCK_EXTDMA | readl(SYS_RESET), SYS_RESET);
	writel(SYS_CLOCK_EXTDMA | readl(SYS_CLOCK), SYS_CLOCK);

	dmadesc[0].src = (u32)par->info->screen_base;
	dmadesc[0].dest = (u32)par->info->fix.smem_start;
	dmadesc[0].length = (u16)(par->info->fix.smem_len / 3);
	dmadesc[0].control = EXT_DMA_DESC_CTRL_FULL;

	dmadesc[1].src = (u32)(par->info->screen_base + par->info->fix.smem_len / 3);
	dmadesc[1].dest = (u32)par->info->fix.smem_start;
	dmadesc[1].length = (u16)(par->info->fix.smem_len / 3);
	dmadesc[1].control = EXT_DMA_DESC_CTRL_FULL;

	dmadesc[2].src = (u32)(dmadesc[1].src + par->info->fix.smem_len / 3);
	dmadesc[2].dest = (u32)par->info->fix.smem_start;
	dmadesc[2].length = (u16)(par->info->fix.smem_len / 3);
	dmadesc[2].control = EXT_DMA_DESC_CTRL_WRAP | \
			     EXT_DMA_DESC_CTRL_LAST | \
			     EXT_DMA_DESC_CTRL_FULL;

	/* Setup the DMA Controllers */
	writel(&dmadesc[0], NS921X_DMA_BDP(1));
	writel(NS921X_DMA_STIE_NCIE | NS921X_DMA_STIE_ECIE | \
	       NS921X_DMA_STIE_NRIE | NS921X_DMA_STIE_CAIE, NS921X_DMA_STIE(1));

	ret = request_irq(IRQ_NS921X_EXTDMA, fb_extdma, 0, "fb_extdma", NULL);
	if (ret) {
		pr_debug("%s: err_request_irq_extdmairq %d -> %d\n", __func__, IRQ_NS921X_EXTDMA, ret);
		return ret;
	}
#endif

	for (i=0; i < (sizeof(edt_qvga_lcd_init)/3); i++) {

		cc9p9215_lcd_wr_reg(par, edt_qvga_lcd_init[i][0],
				    edt_qvga_lcd_init[i][1]);
		mdelay(edt_qvga_lcd_init[i][2]);
	}

	return 0;

err_detect:
	gpio_free(par->pdata->rst_gpio);
	gpio_free(par->pdata->enable_gpio);

	return ret;
}

static void cc9p9215_lcd_cleanup(struct hx8347fb_par *par)
{
	gpio_direction_input(par->pdata->rst_gpio);
	gpio_direction_input(par->pdata->enable_gpio);
	gpio_free(par->pdata->rst_gpio);
	gpio_free(par->pdata->enable_gpio);
}

static struct resource cc9p9215js_lcd_res[] = {
	{
		.start	= 0x40000000,
		.end	= 0x40000001,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x40000002,
		.end	= 0x40000003,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_EXTDMA,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct hx8347fb_pdata c9p9215js_lcd_pdata = {
	.owner			= THIS_MODULE,
	.reset			= cc9p9215_lcd_reset,
	.bl_enable		= cc9p9215_lcd_enable,
	.init			= cc9p9215_lcd_init,
	.cleanup		= cc9p9215_lcd_cleanup,
	.set_idx		= cc9p9215_lcd_set_idx,
	.wr_reg			= cc9p9215_lcd_wr_reg,
	.wr_data		= cc9p9215_lcd_wr_data,
	.rst_gpio		= 86,
	.enable_gpio		= 87,
	.usedma			= 0,
	.xres 			= 320,
	.yres			= 240,
	.bits_per_pixel		= 16,
};

static struct platform_device edt_qvga_lcd = {
	.name		= "hx8347",
	.id		= 0,
	.dev            = {
		.platform_data = &c9p9215js_lcd_pdata,
	},
	.resource	= cc9p9215js_lcd_res,
	.num_resources	= ARRAY_SIZE(cc9p9215js_lcd_res),
};

void __init ns9xxx_add_device_cc9p9215_edt_diplay(void)
{
	platform_device_register(&edt_qvga_lcd);
}
#else
void __init ns9xxx_add_device_cc9p9215_edt_diplay(void) {}
#endif
