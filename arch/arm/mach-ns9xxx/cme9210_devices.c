 /*
 * arch/arm/mach-ns9xxx/cme9210_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/mtd/physmap.h>
#include <mach/fim-ns921x.h>
#include <mach/regs-sys-common.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/w1-gpio.h>

#include "ns921x_devices.h"
#include "cme9210_devices.h"

enum cme9210_variant get_cme9210_variant(void)
{
	return __raw_readl(SYS_GENID) & 0x7ff;
}

#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
void __init ns9xxx_add_device_cme9210_eth(int act_led_gpio)
{
	int gpio[] = {
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
			-1,
#endif
			32, 33, 34, 35, 36, 37, 38, 39, 40,
			41, 42, 43, 44, 45, 46, 47, 48, 49};
	int func[] = {
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
			3,
#endif
			0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0};
	int dir[] = {
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
			1,
#endif
			0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0};

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	gpio[0] = act_led_gpio;
#endif
	ns9xxx_add_device_ns921x_eth(NULL, 0, gpio,
				     func, dir, ARRAY_SIZE(gpio));
}
#else
void __init ns9xxx_add_device_cme9210_eth(int act_led_gpio) {}
#endif

#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static void cme9210_i2c_gpio_reconfigure(void)
{
	gpio_configure_ns921x(9, 0, 0, 1, 0);
	gpio_configure_ns921x(12, 0, 0, 1, 0);
}

static struct plat_ns9xxx_i2c ns9xxx_device_cme9210_i2c_data = {
	.gpio_scl = 9,
	.gpio_sda = 12,
	.speed = 100000,	/* 100000Hz only */
	.gpio_configuration_func = cme9210_i2c_gpio_reconfigure,
};

void __init ns9xxx_add_device_cme9210_i2c(void)
{
	ns9xxx_add_device_ns921x_i2c(&ns9xxx_device_cme9210_i2c_data);
}
#else
void __init ns9xxx_add_device_cme9210_i2c(void) {}
#endif

#if defined(CONFIG_SERIAL_NS921X) || defined(CONFIG_SERIAL_NS921X_MODULE)
void __init ns9xxx_add_device_cme9210_uarta(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uarta(0, gpio_nr, 0);
}

void __init ns9xxx_add_device_cme9210_uartb(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartb(16, gpio_nr, 1);
}

void __init ns9xxx_add_device_cme9210_uartc(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartc(8, gpio_nr, 0);
}
#else
void __init ns9xxx_add_device_cme9210_uarta(int gpio_nr) {}
void __init ns9xxx_add_device_cme9210_uartb(int gpio_nr) {}
void __init ns9xxx_add_device_cme9210_uartc(int gpio_nr) {}
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP)
static struct physmap_flash_data ns9xxx_device_cme9210_flash_data = {
	.width	= 2,
};

void __init ns9xxx_add_device_cme9210_flash(void)
{
	ns9xxx_add_device_ns921x_flash(&ns9xxx_device_cme9210_flash_data);
}
#else
void __init ns9xxx_add_device_cme9210_flash(void) {}
#endif

#if defined(CONFIG_SPI_NS921X) || defined(CONFIG_SPI_NS921X_MODULE)
/* SPI ports and their related GPIOs */
static struct spi_ns9xxx_data ns9xxx_device_cme9210_spi_data = {
	.gpios = {7, 3, 5, 0},
	.gpio_funcs = { NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4 },
	.nr_gpios = 4,
};

void __init ns9xxx_add_device_cme9210_spi(void)
{
	ns9xxx_add_device_ns921x_spi(&ns9xxx_device_cme9210_spi_data);
}
#else
void __init ns9xxx_add_device_cme9210_spi(void) {}
#endif

/*
 * XXX Need testing and it might be incomplete add_device()?
 */

#if defined(CONFIG_FIM_ZERO_SERIAL)
static struct fim_serial_platform_data fim_serial_data0 = {
	.fim_nr        = 0,
#if defined(CONFIG_FIM_ZERO_SERIAL_CTSRTS)
	NS921X_FIM_SERIAL_GPIOS(2, 1, /* RX(2,23) + TX(1) */
				0, 3, /* RTS(0) + CTS(3) */
				NS921X_GPIO_FUNC_2),
#else
	NS921X_FIM_SERIAL_GPIOS(2, 1, /* RX(2,23) + TX(1) */
				FIM_GPIO_DONT_USE, /* RTS */
				FIM_GPIO_DONT_USE, /* CTS */
				NS921X_GPIO_FUNC_2),
#endif
};
struct platform_device ns921x_fim_serial0 = {
	.name              = "fim-serial",
	.id                = 0,
	.dev.platform_data = &fim_serial_data0,
};
EXPORT_SYMBOL(ns921x_fim_serial0);
#endif /* CONFIG_FIM_ZERO_SERIAL */

#if defined(CONFIG_FIM_ZERO_CAN_SELECTED) || defined(CONFIG_FIM_CAN_MODULE)
static struct fim_can_platform_data fim_can_data0 = {
	.fim_nr        		= 0,
	.fim_can_bitrate	= CONFIG_FIM_CAN_DEFAULT_BITRATE,
	NS921X_FIM_CAN_GPIOS( 14, 15, /* RX(2,14) + TX(6,15) */
				NS921X_GPIO_FUNC_2),
};
struct platform_device ns921x_fim_can0 = {
	.name              = "fim-can",
	.id                = 0,
	.dev.platform_data = &fim_can_data0,
};
EXPORT_SYMBOL(ns921x_fim_can0);
#endif /* FIM_ZERO_CAN_SELECTED */

#if defined(CONFIG_FIM_ZERO_SDIO)
static struct fim_sdio_platform_data fim_sdio_data0 = {
	.fim_nr        = 0,
	.host_caps     = 0,
	.min_clk       = 320000,
	.max_clk       = 4500000,

	.d0_gpio_nr    = 0,
	.d0_gpio_func  = NS921X_GPIO_FUNC_2,
	.d1_gpio_nr    = FIM_GPIO_DONT_USE,
	.d2_gpio_nr    = FIM_GPIO_DONT_USE,
	.d3_gpio_nr    = FIM_GPIO_DONT_USE,

	.clk_gpio_nr   = 1,
	.clk_gpio_func = NS921X_GPIO_FUNC_2,

	.cmd_gpio_nr   = 2,
	.cmd_gpio_func = NS921X_GPIO_FUNC_2,

#if defined(CONFIG_FIM_APPKIT_BOARD)
	.cd_gpio_nr    = 9,
	.cd_gpio_func  = NS921X_GPIO_FUNC_2,

	.wp_gpio_nr    = 6,
	.wp_gpio_func  = NS921X_GPIO_FUNC_GPIO,
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
};
struct platform_device ns921x_fim_sdio0 = {
	.name              = "fim-sdio",
	.id                = 0,
	.dev.platform_data = &fim_sdio_data0,
};
EXPORT_SYMBOL(ns921x_fim_sdio0);
#endif /* CONFIG_FIM_ZERO_SDIO */

#if defined(CONFIG_FIM_ZERO_W1)
static struct w1_gpio_platform_data fim_w1_data0 = {
	.pin           = 1,
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

#if defined(CONFIG_FIM_ZERO_USB)
static struct fim_usb_platform_data fim_usb_data0 = {
	.fim_nr                 = 0,
	NS921X_FIM_USB_GPIOS(0, 1, 2,          /* VP + VM + RCV */
			     3, 6, 9,        /* OE_L + ENUM + SPND */
			     NS921X_GPIO_FUNC_2,
			     NS921X_GPIO_FUNC_GPIO),
};
struct platform_device ns921x_fim_usb0 = {
	.name              = "fim-usb",
	.id                = 2,
	.dev.platform_data = &fim_usb_data0,
};
EXPORT_SYMBOL(ns921x_fim_usb0);
#endif /* CONFIG_FIM_ZERO_USB */


#if defined(CONFIG_FIM_ZERO_SPI)
static struct spi_ns921x_fim spi_fim0_data = {
        .fim_nr             = 0,
#if defined(CONFIG_FIM_ZERO_SPI_WANT_MASTER_CS)
        .flags              = SPI_NS921X_SUPPORT_MASTER_CS,
#else
        .flags              = 0,
#endif
        .gpio_base = 0,
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
        NS921X_FIM_SPI_CS_GPIOS(3, false, 0),   /* CS3 can't be supported on the CME because there aren't any free pins*/
};

struct platform_device ns921x_fim_spi0 = {
    .name                   = "fim-spi",
    .id                     = 2,        /* The internal SPI ports are numbered 0 and 1, so
                                           the FIM SPI port is numbered 2 */
    .dev.platform_data      = &spi_fim0_data
};
EXPORT_SYMBOL(ns921x_fim_spi0);
#endif /* CONFIG_FIM_ZERO_SPI */



