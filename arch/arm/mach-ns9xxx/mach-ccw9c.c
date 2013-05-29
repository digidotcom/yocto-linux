/*
 * arch/arm/mach-ns9xxx/mach-ccw9c.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/gpio.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include "irq.h"
#include "processor-ns9360.h"
#include "ns9360_devices.h"
#include "ccx9c_devices.h"

/* I2C devices */
#if defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE)
static struct pca953x_platform_data pca9554_data = {
	.gpio_base	= 108,
};
#endif

static struct i2c_board_info i2c_devices[] __initdata = {
#if defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE)
	{
		I2C_BOARD_INFO("pca9554", 0x20),
		.platform_data = &pca9554_data,
	},
#endif
};

#ifdef CONFIG_CCX9C_TOUCH
static int touch_pendown_state(void)
{
	return gpio_get_value(18) ? 0 : 1;
}

static struct ads7846_platform_data ccx9c_touch_data = {
	.model			= 7843,
	.get_pendown_state	= touch_pendown_state,
	.x_min			= 100,
	.y_min			= 100,
	.x_max			= 4000,
	.y_max			= 4000,
	.rotate			= 180,
	.buflen			= 20,
	.skip_samples		= 0,
};

void __init ccx9c_add_device_touch(void)
{
	if (gpio_request(18, "ads7846"))
		return;

	gpio_configure_ns9360(18, 0, 0, 2);
}

#define CCX9C_SPI_TOUCH \
	{							\
		.modalias	= "ads7846",			\
		.max_speed_hz	= 200000,			\
		.irq		= IRQ_NS9XXX_EXT3,		\
		.bus_num        = 0,				\
		.chip_select    = 0,				\
		.platform_data	= &ccx9c_touch_data,		\
	},

#else
#define	CCX9C_SPI_TOUCH
void __init ccx9c_add_device_touch(void) {}
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
/* Define here the GPIO that will be used for Card Detection */
//#define	MMC_SPI_CD_GPIO		72
/* Define here the GPIO that will be used for Read Only switch */
//#define MMC_SPI_RO_GPIO		26

#ifdef MMC_SPI_CD_GPIO
static int mmc_spi_get_cd(struct device *dev)
{
	return !gpio_get_value(MMC_SPI_CD_GPIO);
}
#endif

#ifdef MMC_SPI_RO_GPIO
static int mmc_spi_get_ro(struct device *dev)
{
	return gpio_get_value(MMC_SPI_RO_GPIO);
}
#endif

void __init ns9360_add_device_mmc_spi(void)
{
#ifdef MMC_SPI_CD_GPIO
	if (gpio_request(MMC_SPI_CD_GPIO, "mmc_spi"))
		return;

	gpio_configure_ns9360(MMC_SPI_CD_GPIO, NS9360_GPIO_INPUT,
			      NS9360_GPIO_DONT_INVERT,
			      NS9360_GPIO_FUNC_GPIO);
#endif
#ifdef MMC_SPI_RO_GPIO
	if (gpio_request(MMC_SPI_RO_GPIO, "mmc_spi"))
		return;

	gpio_configure_ns9360(MMC_SPI_RO_GPIO, NS9360_GPIO_INPUT,
			      NS9360_GPIO_DONT_INVERT,
			      NS9360_GPIO_FUNC_GPIO);
#endif
}

static struct mmc_spi_platform_data mmc_spi_info = {
#ifdef MMC_SPI_RO_GPIO
	.get_ro = mmc_spi_get_ro,
#endif
#ifdef MMC_SPI_CD_GPIO
	.get_cd = mmc_spi_get_cd,
	.caps = MMC_CAP_NEEDS_POLL,
#endif
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V only */
};
#endif

/* Array of SPI devices */
static struct spi_board_info spi_devices[] __initdata = {
	CCX9C_SPI_TOUCH
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	/* MMC over SPI: mmc_spi */
	/* Set 'bus_num' to the SPI master index where the MMC
	 * socket will be connected:
	 *   PortA = 1
	 *   PortB = 0 (SPI connector on JSK board)
	 *   PortC = 2
	 *   PortD = 3
	 * If PortB (bus_num=0) is to be used, the touch screen
	 * device must be disabled in the kernel configuration.
	 */
	{
		.modalias	= "mmc_spi",
		.max_speed_hz	= 5000000,
		.bus_num        = 0,
		.chip_select    = 0,
		.platform_data	= &mmc_spi_info,
	},
#endif
	/* Add here other SPI devices, if any... */
};

static void __init mach_ccw9cjs_init_machine(void)
{
	/* register several system clocks */
	ns9360_init_machine();

	/* UARTs */
#if defined(CONFIG_CCX9C_SERIAL_PORTA_RXTX)
	ns9xxx_add_device_ccx9c_uarta_rxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTA_CTSRTSRXTX)
	ns9xxx_add_device_ccx9c_uarta_ctsrtsrxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTA_FULL)
	ns9xxx_add_device_ccx9c_uarta_full();
#endif
#if defined(CONFIG_CCX9C_SERIAL_PORTB_RXTX)
	ns9xxx_add_device_ccx9c_uartb_rxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTB_CTSRTSRXTX)
	ns9xxx_add_device_ccx9c_uartb_ctsrtsrxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTB_FULL)
	ns9xxx_add_device_ccx9c_uartb_full();
#endif
#if defined(CONFIG_CCX9C_SERIAL_PORTC_RXTX)
	ns9xxx_add_device_ccx9c_uartc_rxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTC_CTSRTSRXTX)
	ns9xxx_add_device_ccx9c_uartc_ctsrtsrxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTC_FULL)
	ns9xxx_add_device_ccx9c_uartc_full();
#endif
#if defined(CONFIG_CCX9C_SERIAL_PORTD_RXTX)
	ns9xxx_add_device_ccx9c_uartd_rxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTD_CTSRTSRXTX)
	ns9xxx_add_device_ccx9c_uartd_ctsrtsrxtx();
#elif defined(CONFIG_CCX9C_SERIAL_PORTD_FULL)
	ns9xxx_add_device_ccx9c_uartd_full();
#endif

	/* SPI */
#if defined(CONFIG_CCX9C_SPI_PORTA)
	ns9xxx_add_device_ccx9c_spi_porta();
#endif
#if defined(CONFIG_CCX9C_SPI_PORTB)
	ns9xxx_add_device_ccx9c_spi_portb();
#endif
#if defined(CONFIG_CCX9C_SPI_PORTC)
	ns9xxx_add_device_ccx9c_spi_portc();
#endif
#if defined(CONFIG_CCX9C_SPI_PORTD)
	ns9xxx_add_device_ccx9c_spi_portd();
#endif

	/* Ethernet */
	ns9xxx_add_device_ccx9c_eth();

	/* NAND flash */
	ns9xxx_add_device_ccx9c_nand();

	/* Watchdog timer */
	ns9xxx_add_device_ns9360_wdt();

	/* USB host */
	ns9xxx_add_device_ns9360_usbh();

	/* I2C controller */
	ns9xxx_add_device_ccx9c_i2c();

	/* I2C devices */
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	/* Framebuffer */
#if defined(CONFIG_CCX9C_FB)
	ns9xxx_add_device_ccx9c_fb(23);
#endif

	/* Touchscreen */
	ccx9c_add_device_touch();

	/* MMC over SPI */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	ns9360_add_device_mmc_spi();
#endif

	/* SPI devices */
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

	/* RTC */
	ns9xxx_add_device_ns9360_rtc();
}

MACHINE_START(CCW9C, "ConnectCore Wi-9C")
	.map_io = ns9360_map_io,
	.init_irq = ns9xxx_init_irq,
	.init_machine = mach_ccw9cjs_init_machine,
	.timer = &ns9360_timer,
	.boot_params = 0x100,
MACHINE_END
