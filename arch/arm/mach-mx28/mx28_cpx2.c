/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2011 Digi International, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/phy.h>
#include <linux/micrel_phy.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>

#if defined(CONFIG_MXS_PWM) || defined(CONFIG_MXS_PWM_MODULE)
# include <mach/mxs-pwm.h>
#endif

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include "device.h"
#include "mx28_cpx2.h"

static struct i2c_board_info __initdata mxs_i2c_device[] = {
	{ I2C_BOARD_INFO("sgtl5000-i2c", 0xa), .flags = I2C_M_TEN }
};

static void __init i2c_device_init(void)
{
	i2c_register_board_info(0, mxs_i2c_device, ARRAY_SIZE(mxs_i2c_device));
}
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct flash_platform_data mx28_spi_flash_data = {
	.name = "m25p80",
	.type = "w25x80",
};
#endif

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
	{
		.modalias = "spidev", /* use the generic spi driver */
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 0,
		.platform_data = NULL, /* no data for this device */
	},
#endif
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 1, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &mx28_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx28_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if (defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)) \
	&& (defined (CONFIG_LEDS_GPIO_PLATFORM) || defined (CONFIG_LEDS_GPIO_PLATFORM_MODULE) )
#include <linux/leds.h>

static struct gpio_led mxs_cpx2_leds[] = {
	[0] = {
		.gpio			= 122, /* GPIO3_26 */
		.name			= "mxs-cpx2-network:green",
		.default_trigger	= "none",
		.active_low		= 0,
	},
	[1] = {
		.gpio			= 123, /* GPIO3_27 */
		.name			= "mxs-cpx2-network:yellow",
		.default_trigger	= "none",
		.active_low		= 0,
	},
	[2] = {
		.gpio			= 68, /* ERT_ACTIVITY */
		.name			= "mxs-cpx2-ert-act",
		.default_trigger	= "none",
		.active_low		= 0,
	},
	[3] = {
		.gpio			= 118, /* UTILITY_ACT1 */
		.name			= "mxs-cpx2-utility-act1",
		.default_trigger	= "none",
		.active_low		= 0,
	},
	[4] = {
		.gpio			= 119, /* UTILITY_ACT2 */
		.name			= "mxs-cpx2-utility-act2",
		.default_trigger	= "none",
		.active_low		= 0,
	},
};

static struct gpio_led_platform_data mxs_cpx2_leds_pdata = {
	.num_leds	= ARRAY_SIZE(mxs_cpx2_leds),
	.leds		= mxs_cpx2_leds,
};

static struct platform_device mxs_cpx2_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &mxs_cpx2_leds_pdata,
	},
};

static void __init mx28_cpx2_init_leds(void)
{
	platform_device_register(&mxs_cpx2_led_device);
}
#else
static inline void mx28_cpx2_init_leds(void) {}
#endif

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28_cpx2_led_pwm[2] = {
	[0] = {
		.name = "led-pwm0",
		.pwm = 0,
		},
	[1] = {
		.name = "led-pwm1",
		.pwm = 1,
		},
};

struct mxs_pwm_leds_plat_data mx28_cpx2_led_data = {
	.num = ARRAY_SIZE(mx28_cpx2_led_pwm),
	.leds = mx28_cpx2_led_pwm,
};

static struct resource mx28_cpx2_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28_cpx2_init_pwm_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28_cpx2_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28_cpx2_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_cpx2_init_pwm_leds(void)
{
	;
}
#endif

#if defined(CONFIG_MXS_PWM) || defined(CONFIG_MXS_PWM_MODULE)
static struct mxs_pwm_channel mx28_pwm_channels[] = {
	[0] = {
		.channel = 7,
	},
};

/* This structure will be initialized in the init function (see below) */
static struct mxs_pwm_pdata mx28_pwm_pdata;

static struct platform_device mx28_pwm_device = {
	.name		= "mxs-pwm",
	.id		= 0,
	.dev		= {
		.platform_data = &mx28_pwm_pdata,
	},
};

static struct resource mx28_pwm_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void mx28_pwm_init(void)
{
	mx28_pwm_pdata.channels = mx28_pwm_channels;
	mx28_pwm_pdata.number_channels = ARRAY_SIZE(mx28_pwm_channels);

	mx28_pwm_device.resource = &mx28_pwm_res;
	mx28_pwm_device.num_resources = 1;

	platform_device_register(&mx28_pwm_device);
}
#else
static void mx28_pwm_init(void)
{
	;
}
#endif

static void __init mx28_cpx2_device_init(void)
{
	/* Add mx28_cpx2 special code */
	i2c_device_init();
	spi_device_init();
	mx28_cpx2_init_pwm_leds();
	mx28_cpx2_init_leds();
	mx28_pwm_init();
}

	// Digi product_id value; defaults to "cpx2e-se"
	// since initial cpx2e-se units were not manufactured
	// with digi_product_id= in ubootenv
unsigned long digi_product_id = { 0x00008001 };

static int __init digi_product_id_setup(char *value)
{
	if (value != NULL && !strict_strtoul(value, 16, &digi_product_id))
		return 1;

	printk(KERN_ERR "bad value for digi_product_id\n");
	return 0;
}

__setup("digi_product_id=", digi_product_id_setup);

static int ksz8031_phy_fixup(struct phy_device *phydev)
{
	phydev->dev_flags |= MICREL_PHY_50MHZ_CLK;

	return 0;
}

static void __init mx28_cpx2_init_machine(void)
{
	printk(KERN_INFO "digi_product_id=0x%08lx\n", digi_product_id);

	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif

	mx28_gpio_init();
	mx28_cpx2_pins_init();
	mx28_device_init();
	mx28_cpx2_device_init();

	/* Register the fixup for the PHY */
	phy_register_fixup_for_uid(PHY_ID_KSZ8031, MICREL_PHY_ID_MASK, ksz8031_phy_fixup);
	phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK, ksz8031_phy_fixup);

	/* init the system revesion */
	system_rev = 0x28011;
}

MACHINE_START(CPX2, "Digi ConnectPort X2")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28_cpx2_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
