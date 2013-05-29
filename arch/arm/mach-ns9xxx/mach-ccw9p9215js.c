/*
 * arch/arm/mach-ns9xxx/mach-cc9p9215.c
 *
 * Copyright (C) 2007 by Digi International Inc.
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
#include <linux/crc32.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <asm/leds.h>

#include "irq.h"
#include "pipermain.h"
#include "processor-ns921x.h"
#include "ns921x_devices.h"
#include "ns9215_devices.h"
#include "cc9p9215_devices.h"
#include "ccw9p9215_devices.h"


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

#ifdef CONFIG_CC9P9215JS_TOUCH
static int touch_pendown_state(void)
{
	return gpio_get_value(101) ? 0 : 1;
}

static struct ads7846_platform_data cc9p9215js_touch_data = {
	.model =		7843,
	.get_pendown_state =	touch_pendown_state,
	.x_min =		100,
	.y_min =		100,
	.x_max =		4000,
	.y_max =		4000,
	.rotate =		180,
	.buflen =		20,
	.skip_samples =		0,
};

void __init cc9p9215js_add_device_touch(void)
{
	if (gpio_request(101, "ads7846"))
		return;
	gpio_configure_ns921x(101, 0, 0, 2, 0);
}

#define CC9P9215JS_TOUCH					\
	{							\
		.modalias	= "ads7846",			\
		.max_speed_hz	= 300000,			\
		.irq		= IRQ_NS9XXX_EXT3,		\
		.bus_num        = 1,				\
		.chip_select    = 0,				\
		.platform_data	= &cc9p9215js_touch_data,	\
	},

#else
#define	CC9P9215JS_TOUCH
void __init cc9p9215js_add_device_touch(void) {}
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

void __init ns921x_add_device_mmc_spi(void)
{
#ifdef MMC_SPI_CD_GPIO
	if (gpio_request(MMC_SPI_CD_GPIO, "mmc_spi"))
		return;

	gpio_configure_ns921x(MMC_SPI_CD_GPIO, NS921X_GPIO_INPUT,
			      NS921X_GPIO_DONT_INVERT,
			      NS921X_GPIO_FUNC_GPIO,
			      NS921X_GPIO_ENABLE_PULLUP);
#endif
#ifdef MMC_SPI_RO_GPIO
	if (gpio_request(MMC_SPI_RO_GPIO, "mmc_spi"))
		return;

	gpio_configure_ns921x(MMC_SPI_RO_GPIO, NS921X_GPIO_INPUT,
			      NS921X_GPIO_DONT_INVERT,
			      NS921X_GPIO_FUNC_GPIO,
			      NS921X_GPIO_ENABLE_PULLUP);
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

/* SPIDEV devices for the FIM SPI ports and chip selects.
 * bus_num = FIM number + 2 (bus_num 1 is native SPI port)
 */
#define SPIDEV_FIM_CS(fim, cs)					\
	{							\
		.modalias	= "spidev",			\
		.max_speed_hz	= 10000000,			\
		.bus_num        = fim + 2,			\
		.chip_select    = cs,				\
	}

/* Array of SPI devices (only one device should be enabled at a time) */
static struct spi_board_info spi_devices[] __initdata = {
	CC9P9215JS_TOUCH
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	/* SPIDEV */
	{
		.modalias	= "spidev",
		.max_speed_hz	= 10000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
#endif
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	/* MMC over SPI: mmc_spi */
	{
		.modalias	= "mmc_spi",
		.max_speed_hz	= 5000000,
		.bus_num        = 1,
		.chip_select    = 0,
		.platform_data	= &mmc_spi_info,
	},
#endif
#if defined(CONFIG_FIM_ZERO_SPIDEV_MASTER_CS) || defined(CONFIG_FIM_ZERO_SPIDEV_CS0)
	SPIDEV_FIM_CS(0, 0),
#endif
#if defined(CONFIG_FIM_ZERO_SPIDEV_CS1)
	SPIDEV_FIM_CS(0, 1),
#endif
#if defined(CONFIG_FIM_ZERO_SPIDEV_CS2)
	SPIDEV_FIM_CS(0, 2),
#endif
#if defined(CONFIG_FIM_ZERO_SPIDEV_CS3)
	SPIDEV_FIM_CS(0, 3),
#endif
#if defined(CONFIG_FIM_ONE_SPIDEV_MASTER_CS) || defined(CONFIG_FIM_ONE_SPIDEV_CS0)
	SPIDEV_FIM_CS(1, 0),
#endif
#if defined(CONFIG_FIM_ONE_SPIDEV_CS1)
	SPIDEV_FIM_CS(1, 1),
#endif
#if defined(CONFIG_FIM_ONE_SPIDEV_CS2)
	SPIDEV_FIM_CS(1, 2),
#endif
#if defined(CONFIG_FIM_ONE_SPIDEV_CS3)
	SPIDEV_FIM_CS(1, 3),
#endif
	/* Add here other SPI devices, if any... */
};

#if defined(CONFIG_DIGI_PIPER_WIFI)
static struct piper_pdata ccw9p9215_piper_pdata = {
	.rst_gpio		= 92,
	.irq_gpio		= 104,
	.i2c_adapter_num	= 0,
};

static void __init ccw9p9215js_fixup(struct machine_desc *desc,
				     struct tag *tags, char **cmdline,
				     struct meminfo *mi)
{
	unsigned char *mac = phys_to_virt(desc->boot_params) + 0xf00;
	wcd_data_t *pwcal;
	u32 crc;

	/* 8 bytes after the mac address, its located the calibration data */
	pwcal = (wcd_data_t *)(mac + 8);
	memcpy(&ccw9p9215_piper_pdata.macaddr[0], mac, 6);

	if (!strncmp(pwcal->header.magic_string, WCD_MAGIC,
	    sizeof(pwcal->header.magic_string))) {
		/* check version */
		if (((pwcal->header.ver_major >= '1') && (pwcal->header.ver_major <= '9')) &&
		    ((pwcal->header.ver_minor >= '0') && (pwcal->header.ver_minor <= '9'))) {
			crc = ~crc32_le(~0, (unsigned char const *)pwcal->cal_curves_bg,
					pwcal->header.wcd_len);
			if (crc == pwcal->header.wcd_crc) {
				memcpy(&ccw9p9215_piper_pdata.wcd, pwcal, sizeof(wcd_data_t));
				return;
			}
		}
	}

	memset(&ccw9p9215_piper_pdata.wcd, 0, sizeof(wcd_data_t));
}
#endif

static void __init mach_ccw9p9215js_init_machine(void)
{
	/* register several system clocks */
	ns921x_init_machine();

	/* UARTs */
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTA_RXTX)
	ns9xxx_add_device_cc9p9215_uarta_rxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTA_CTSRTSRXTX) || \
	defined(CONFIG_CC9P9215JS_SERIAL_PORTA_RXTX485)
	ns9xxx_add_device_cc9p9215_uarta_ctsrtsrxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTA_FULL)
	ns9xxx_add_device_cc9p9215_uarta_full();
#endif
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTB_RXTX)
	ns9xxx_add_device_cc9p9215_uartb_rxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTB_CTSRTSRXTX) || \
	defined(CONFIG_CC9P9215JS_SERIAL_PORTB_RXTX485)
	ns9xxx_add_device_cc9p9215_uartb_ctsrtsrxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTB_FULL)
	ns9xxx_add_device_cc9p9215_uartb_full();
#endif
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTC_RXTX)
	ns9xxx_add_device_cc9p9215_uartc_rxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTC_CTSRTSRXTX) || \
	defined(CONFIG_CC9P9215JS_SERIAL_PORTC_RXTX485)
	ns9xxx_add_device_cc9p9215_uartc_ctsrtsrxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTC_FULL)
	ns9xxx_add_device_cc9p9215_uartc_full();
#endif
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTD_RXTX)
	ns9xxx_add_device_cc9p9215_uartd_rxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTD_CTSRTSRXTX) || \
	defined(CONFIG_CC9P9215JS_SERIAL_PORTD_RXTX485)
	ns9xxx_add_device_cc9p9215_uartd_ctsrtsrxtx();
#elif defined(CONFIG_CC9P9215JS_SERIAL_PORTD_FULL)
	ns9xxx_add_device_cc9p9215_uartd_full();
#endif

	/* Ethernet */
	ns9xxx_add_device_cc9p9215_eth();

	/* 802.11 */
#if defined(CONFIG_DIGI_PIPER_WIFI)
	ns9xxx_add_device_ccw9p9215_wifi(&ccw9p9215_piper_pdata);
#endif

	/* SPI */
#ifdef CONFIG_CC9P9215JS_SPI
	ns9xxx_add_device_cc9p9215_spi();
#endif

	/* Touchscreen */
	cc9p9215js_add_device_touch();

	/* MMC over SPI */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	ns921x_add_device_mmc_spi();
#endif

	/* SPI devices */
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

	/* Watchdog timer */
	ns9xxx_add_device_ns921x_wdt();

	/* NOR Flash */
	ns9xxx_add_device_cc9p9215_flash();

	/* I2C controller */
	ns9xxx_add_device_cc9p9215_i2c();

	/* I2C devices */
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	/* Leds */
	ns9xxx_add_device_ns9215_leds();

	/* Analog Digital Converter */
	ns9xxx_add_device_ns9215_adc();

	/* AES HW Encryption module */
#if defined(CONFIG_CRYPTO_DEV_NS921X_AES) || \
	defined(CONFIG_CRYPTO_DEV_NS921X_AES_MODULE)
	ns9xxx_add_device_ns921x_aes();
#endif

	/* Real Time Clock */
	ns9xxx_add_device_ns9215_rtc();

	/* GPIO 4 is used as wake up gpio */
	(void)ns921x_extgpio_pm_wakeup_init(4);

	/* Init the FIM devices */
	ns9xxx_add_device_ns921x_fims();

	/* Video */
	ns9xxx_add_device_cc9p9215_edt_diplay();
}

MACHINE_START(CCW9P9215JS, "ConnectCore Wi-9P 9215 on a JSCCW9P9215 Devboard")
	.map_io = ns921x_map_io,
	.init_irq = ns9xxx_init_irq,
#if defined(CONFIG_DIGI_PIPER_WIFI)
	.fixup = ccw9p9215js_fixup,
#endif
	.init_machine = mach_ccw9p9215js_init_machine,
	.timer = &ns921x_timer,
	.boot_params = 0x100,
MACHINE_END
