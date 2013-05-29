/*
 * arch/arm/mach-ns9xxx/ccx9c_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include "ns9360_devices.h"
#include "ccx9c_devices.h"

#if defined(CONFIG_SPI_NS9360) || defined(CONFIG_SPI_NS9360_MODULE)
/* SPI ports and their related GPIOs */
static struct spi_ns9xxx_data ns9xxx_device_ccx9c_spi_porta_data = {
	.gpios = {8, 9, 14, 15},
	.nr_gpios = 4,
};

static struct spi_ns9xxx_data ns9xxx_device_ccx9c_spi_portb_data = {
	.gpios = {0, 1, 6, 7},
	.nr_gpios = 4,
};

static struct spi_ns9xxx_data ns9xxx_device_ccx9c_spi_portc_data = {
	.gpios = {40, 41, 22, 23},
	.nr_gpios = 4,
};

static struct spi_ns9xxx_data ns9xxx_device_ccx9c_spi_portd_data = {
	.gpios = {44, 45, 26, 27},
	.nr_gpios = 4,
};
#endif

#if defined(CONFIG_MTD_NAND_CCX9X) || defined(CONFIG_MTD_NAND_CCX9X_MODULE)
static struct ccx9x_nand_info ns9xxx_device_ccx9c_nand_data = {
	.addr_offset	= 0x8000,
	.cmd_offset	= 0x10000,
	.delay		= 25,
};
void __init ns9xxx_add_device_ccx9c_nand(void)
{
	ns9xxx_add_device_ns9360_nand(&ns9xxx_device_ccx9c_nand_data);
}
#else
void __init ns9xxx_add_device_ccx9c_nand(void) {}
#endif

#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static void ccx9c_i2c_gpio_reconfigure(void)
{
	gpio_configure_ns9360(70, 0, 0, 2);
	gpio_configure_ns9360(71, 0, 0, 2);
}

static struct plat_ns9xxx_i2c ns9xxx_device_ccx9c_i2c_data = {
	.gpio_scl = 70,
	.gpio_sda = 71,
	.speed = 100000,	/* 100000Hz or 400000Hz */
	.gpio_configuration_func = ccx9c_i2c_gpio_reconfigure,
};

void __init ns9xxx_add_device_ccx9c_i2c(void)
{
	ns9xxx_add_device_ns9360_i2c(&ns9xxx_device_ccx9c_i2c_data);
}
#else
void __init ns9xxx_add_device_ccx9c_i2c(void) {}
#endif

#if defined(CONFIG_SERIAL_NS9360) || defined(CONFIG_SERIAL_NS9360_MODULE)
void __init ns9xxx_add_device_ccx9c_uarta_rxtx(void)
{
	int uarta_gpio[] = {8, 9};
	ns9xxx_add_device_ns9360_uarta(uarta_gpio, ARRAY_SIZE(uarta_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uarta_ctsrtsrxtx(void)
{
	int uarta_gpio[] = {8, 9, 10, 11};
	ns9xxx_add_device_ns9360_uarta(uarta_gpio, ARRAY_SIZE(uarta_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uarta_full(void)
{
	int uarta_gpio[] = { 8,  9, 10, 11, 12, 13, 14, 15};
	ns9xxx_add_device_ns9360_uarta(uarta_gpio, ARRAY_SIZE(uarta_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartb_rxtx(void)
{
	int uartb_gpio[] = {0,  1};
	ns9xxx_add_device_ns9360_uartb(uartb_gpio, ARRAY_SIZE(uartb_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartb_ctsrtsrxtx(void)
{
	int uartb_gpio[] = {0, 1, 2, 3};
	ns9xxx_add_device_ns9360_uartb(uartb_gpio, ARRAY_SIZE(uartb_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartb_full(void)
{
	int uartb_gpio[] = { 0,  1,  2,  3,  4,  5,  6,  7};
	ns9xxx_add_device_ns9360_uartb(uartb_gpio, ARRAY_SIZE(uartb_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartc_rxtx(void)
{
	int uartc_gpio[] = {40, 41};
	ns9xxx_add_device_ns9360_uartc(uartc_gpio, ARRAY_SIZE(uartc_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartc_ctsrtsrxtx(void)
{
	int uartc_gpio[] = {40, 41, 42, 43};
	ns9xxx_add_device_ns9360_uartc(uartc_gpio, ARRAY_SIZE(uartc_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartd_rxtx(void)
{
	int uartd_gpio[] = {44, 45};
	ns9xxx_add_device_ns9360_uartd(uartd_gpio, ARRAY_SIZE(uartd_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartd_ctsrtsrxtx(void)
{
	int uartd_gpio[] = {44, 45, 46, 47};
	ns9xxx_add_device_ns9360_uartd(uartd_gpio, ARRAY_SIZE(uartd_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartc_full(void)
{
	int uartc_gpio[] = {40, 41, 42, 43, 20, 21, 22, 23};
	ns9xxx_add_device_ns9360_uartc(uartc_gpio, ARRAY_SIZE(uartc_gpio), 0);
}

void __init ns9xxx_add_device_ccx9c_uartd_full(void)
{
	int uartd_gpio[] = {44, 45, 46, 47, 24, 25, 26, 27};
	ns9xxx_add_device_ns9360_uartd(uartd_gpio, ARRAY_SIZE(uartd_gpio), 0);
}
#else
void __init ns9xxx_add_device_ccx9c_uarta_rxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uarta_ctsrtsrxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uarta_full(void) {}
void __init ns9xxx_add_device_ccx9c_uartb_rxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartb_ctsrtsrxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartb_full(void) {}
void __init ns9xxx_add_device_ccx9c_uartc_rxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartc_ctsrtsrxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartc_full(void) {}
void __init ns9xxx_add_device_ccx9c_uartd_rxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartd_ctsrtsrxtx(void) {}
void __init ns9xxx_add_device_ccx9c_uartd_full(void) {}
#endif

#if defined(CONFIG_FB_NS9360) || defined(CONFIG_FB_NS9360_MODULE)
void __init ns9xxx_add_device_ccx9c_fb(int power)
{
	int gpio[] = {19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30,
		      31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41};
	int func[] = { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
		       1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2};

	ns9xxx_add_device_ns9360_fb(gpio, ARRAY_SIZE(gpio), func, power);
}
#else
void __init ns9xxx_add_device_ccx9c_fb(int power) {}
#endif

#if defined(CONFIG_SPI_NS9360) || defined(CONFIG_SPI_NS9360_MODULE)
void __init ns9xxx_add_device_ccx9c_spi_porta(void)
{
	ns9xxx_add_device_ns9360_spi_porta( &ns9xxx_device_ccx9c_spi_porta_data );
}

void __init ns9xxx_add_device_ccx9c_spi_portb(void)
{
	ns9xxx_add_device_ns9360_spi_portb( &ns9xxx_device_ccx9c_spi_portb_data );
}

void __init ns9xxx_add_device_ccx9c_spi_portc(void)
{
	ns9xxx_add_device_ns9360_spi_portc( &ns9xxx_device_ccx9c_spi_portc_data );
}

void __init ns9xxx_add_device_ccx9c_spi_portd(void)
{
	ns9xxx_add_device_ns9360_spi_portd( &ns9xxx_device_ccx9c_spi_portd_data );
}

#else
void __init ns9xxx_add_device_ccx9c_spi_porta(void) {}
void __init ns9xxx_add_device_ccx9c_spi_portb(void) {}
void __init ns9xxx_add_device_ccx9c_spi_portc(void) {}
void __init ns9xxx_add_device_ccx9c_spi_portd(void) {}
#endif

#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
void __init ns9xxx_add_device_ccx9c_eth(void)
{
	int gpio[] = {50, 51, 52, 53, 54, 55, 56, 57,
		      58, 59, 60, 61, 62, 63, 64};

	ns9xxx_add_device_ns9360_eth(gpio, 15, 0, 0xfffffffd);
}
#else
void __init ns9xxx_add_device_ccx9c_eth(void) {}
#endif
