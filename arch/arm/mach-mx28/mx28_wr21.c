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

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include "device.h"
#include "mx28_wr21.h"

static struct i2c_board_info __initdata mxs_i2c_device[] = {
	{ I2C_BOARD_INFO("ds1337", 0x68) }
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

static struct gpio_led mxs_wr21_leds[] = {
	[0] = {
		.gpio			= 42, /* GPIO1_10 */
		.name			= "mxs-wr21-service",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[1] = {
		.gpio			= 43, /* GPIO1_11 */
		.name			= "mxs-wr21-wwan",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[2] = {
		.gpio			= 44, /* GPIO1_12 */
		.name			= "mxs-wr21-signal1",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[3] = {
		.gpio			= 45, /* GPIO1_13 */
		.name			= "mxs-wr21-signal2",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[4] = {
		.gpio			= 46, /* GPIO1_14 */
		.name			= "mxs-wr21-signal3",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[5] = {
		.gpio			= 47, /* GPIO1_15 */
		.name			= "mxs-wr21-lan0",
		.default_trigger	= "none",
		.active_low		= 1,
	},
	[6] = {
		.gpio			= 48, /* GPIO1_16 */
		.name			= "mxs-wr21-lan1",
		.default_trigger	= "none",
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data mxs_wr21_leds_pdata = {
	.num_leds	= ARRAY_SIZE(mxs_wr21_leds),
	.leds		= mxs_wr21_leds,
};

static struct platform_device mxs_wr21_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &mxs_wr21_leds_pdata,
	},
};

static void __init mx28_wr21_init_leds(void)
{
	platform_device_register(&mxs_wr21_led_device);
}
#else
static inline void mx28_wr21_init_leds(void) {}
#endif

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28_wr21_led_pwm[2] = {
	[0] = {
		.name = "led-pwm0",
		.pwm = 0,
		},
	[1] = {
		.name = "led-pwm1",
		.pwm = 1,
		},
};

struct mxs_pwm_leds_plat_data mx28_wr21_led_data = {
	.num = ARRAY_SIZE(mx28_wr21_led_pwm),
	.leds = mx28_wr21_led_pwm,
};

static struct resource mx28_wr21_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28_wr21_init_pwm_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28_wr21_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28_wr21_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_wr21_init_pwm_leds(void)
{
	;
}
#endif

static void __init mx28_wr21_device_init(void)
{
	/* Add mx28_wr21 special code */
	i2c_device_init();
	spi_device_init();
	mx28_wr21_init_pwm_leds();
	mx28_wr21_init_leds();
}

static int ksz8031_phy_fixup(struct phy_device *phydev)
{
	phydev->dev_flags |= MICREL_PHY_50MHZ_CLK;

	return 0;
}

static void __init mx28_wr21_init_machine(void)
{
	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif

	mx28_gpio_init();
	mx28_wr21_pins_init();
	mx28_device_init();
	mx28_wr21_device_init();

	/* Register the fixup for the PHYs */
	phy_register_fixup_for_uid(PHY_ID_KSZ8031, MICREL_PHY_ID_MASK, ksz8031_phy_fixup);

	/* init the system revesion */
	system_rev = 0x28011;
}

MACHINE_START(WR21, "Digi TransPort WR21")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28_wr21_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
