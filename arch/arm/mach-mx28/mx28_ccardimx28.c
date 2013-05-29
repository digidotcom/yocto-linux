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
#include <asm/mach/bootldr_shmem.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>
#include <mach/lcdif.h>
#include <mach/mxs-pwm.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include "device.h"
#include "mx28_ccardimx28.h"
#include "mx28_pins.h"

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
#include <asm/mach/bootldr_shmem.h>
#include <linux/crc32.h>
#include "displays/lcd.h"
#endif

static struct i2c_board_info __initdata mxs_i2c_device[] = {
#if defined(CONFIG_SND_MXS_SOC_CCARDIMX28) || defined(CONFIG_SND_MXS_SOC_CCARDIMX28_MODULE)
	{ I2C_BOARD_INFO("sgtl5000-i2c", 0xa), .flags = I2C_M_TEN },
#endif
#if defined(CONFIG_W1_MASTER_DS2482) || defined(CONFIG_W1_MASTER_DS2482_MODULE)
	{ I2C_BOARD_INFO("ds2482", 0x18), /*.flags = 0 */}
#endif
};

#if defined(CONFIG_BACKLIGHT_MXS) || \
	defined(CONFIG_BACKLIGHT_MXS_MODULE)

static struct mxs_platform_bl_data bl_data = {
	.bl_max_intensity = 100,
	.bl_default_intensity = 70,
	.pwm = 0,
	.inverted = 1,
};

static void __init mx28_ccardimx28_init_backlight(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-bl", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &bl_data;
}
#else
static void __init mx28_ccardimx28_init_backlight(void)
{
	;
}
#endif

static void __init i2c_device_init(void)
{
	i2c_register_board_info(1, mxs_i2c_device, ARRAY_SIZE(mxs_i2c_device));
}

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE) || \
    defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
	{
		.modalias = "spidev", /* use the generic spi driver */
		.max_speed_hz = 20000000,
		.bus_num = 3,        /* !!!you have to make sure master->bus_num (in spi_xms.c::mxs_spi_probe())is the same as this number!!! */
		.chip_select = 0,
		.platform_data = NULL, /* no data for this device */
	},
#endif
#if (defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)) && \
	defined(CONFIG_CCARDIMX28_SPI_SSP1_ENABLE)
	{
		.modalias = "spidev", 		/* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     	/* max spi clock (SCK) speed in HZ */
		.bus_num = 1, 			/* Framework bus number */
		.chip_select = 0, 		/* Framework chip select. */
		.controller_data	= (void *) MXS_PIN_TO_GPIO(PINID_SSP1_DATA3),
		.platform_data = NULL, 		/* no data for this device */
	},
#endif
#if (defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)) && \
	defined(CONFIG_CCARDIMX28_SPI_SSP3_ENABLE)
	{
		.modalias = "spidev", 		/* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     	/* max spi clock (SCK) speed in HZ */
		.bus_num = 3, 			/* Framework bus number */
		.chip_select = 0, 		/* Framework chip select. */
		.controller_data	= (void *) MXS_PIN_TO_GPIO(PINID_SSP3_SS0),
		.platform_data = NULL, 		/* no data for this device */
	},
#endif
};
#endif /* SPI || SPI_GPIO */

#if defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
#ifdef CONFIG_CCARDIMX28_SPI_SSP1_ENABLE
static struct spi_gpio_platform_data spi1_gpio_data = {
          .sck = MXS_PIN_TO_GPIO(PINID_SSP1_SCK),
          .mosi = MXS_PIN_TO_GPIO(PINID_SSP1_CMD),
	  .miso = MXS_PIN_TO_GPIO(PINID_SSP1_DATA0),
          .num_chipselect = 1,
};
static struct platform_device spi1_gpio_device = {
	  .name      = "spi_gpio",
	  .id      = 1,   	/*This has to the same as .bus_num = 3, Framework bus number above */
	  .dev      = {
	    .platform_data   = &spi1_gpio_data,
	  },
};
#endif

#ifdef CONFIG_CCARDIMX28_SPI_SSP3_ENABLE
static struct spi_gpio_platform_data spi3_gpio_data = {
          .sck = MXS_PIN_TO_GPIO(PINID_SSP3_SCK),
          .mosi = MXS_PIN_TO_GPIO(PINID_SSP3_MOSI),
	  .miso = MXS_PIN_TO_GPIO(PINID_SSP3_MISO),
          .num_chipselect = 1,
};

static struct platform_device spi3_gpio_device = {
	  .name      = "spi_gpio",
	  .id      = 3,   	/*This has to the same as .bus_num = 3, Framework bus number above */
	  .dev      = {
	    .platform_data   = &spi3_gpio_data,
	  },
};
#endif
#endif /* CONFIG_SPI_GPIO */

static void spi_device_init(void)
{
#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
      spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
#if defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
      spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#ifdef CONFIG_CCARDIMX28_SPI_SSP1_ENABLE
      platform_device_register(&spi1_gpio_device);
#endif
#ifdef CONFIG_CCARDIMX28_SPI_SSP3_ENABLE
      platform_device_register(&spi3_gpio_device);
#endif
#endif /* CONFIG_SPI_GPIO */
}

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static struct resource framebuffer_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LCDIF_PHYS_ADDR,
	 .end   = LCDIF_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LCDIF,
	 .end   = IRQ_LCDIF,
	 },
};

struct mx28_lcds_plat_data lcds_pdata = {
	.displays = lcd_panel_list,
	.num_displays = ARRAY_SIZE(lcd_panel_list),
#if defined(CONFIG_BACKLIGHT_MXS) || defined(CONFIG_BACKLIGHT_MXS_MODULE)
	.bl_data = &bl_data,
#endif /* CONFIG_BACKLIGHT_MXS */
};

static void __init mx28_init_lcdif(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = framebuffer_resource;
	pdev->num_resources = ARRAY_SIZE(framebuffer_resource);
	pdev->dev.platform_data = &lcds_pdata;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_lcdif(void)
{
	;
}
#endif /* CONFIG_FB_MXS */

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static void mx28_set_lcd_struct_from_bl_shared(nv_lcd_config_t *lcd_config)
{
	int i;
	uint8_t lcd1_valid, lcd2_valid;
	uint32_t crc;

	lcd1_valid = 0;
	lcd2_valid = 0;

	/* In case of valid structure and valid configuration, copy the LCD settings */
	/* Otherwise clear the LCD settings structure */
	if ( strncmp(lcd_config->header.magic_string, NV_LCD_CONFIG_MAGIC,
		sizeof(lcd_config->header.magic_string)) ) {
		return;
	}

	/* Check version */
	if (lcd_config->header.version < 2) {
		return;
	}

	/* Calculate and check CRC */
	crc = ~crc32_le(~0,
			(const uint8_t *)&lcd_config->lcd1,
			sizeof(nv_lcd_config_data_t));
	crc = ~crc32_le(~crc,
			(const uint8_t *)&lcd_config->lcd2,
			sizeof(nv_lcd_config_data_t));
	if ( crc != lcd_config->header.crc ) {
		return;
	}

	if ( lcd_config->header.lcd1_valid ) {
		lcd1_valid = 1;
	}

	if ( lcd_config->header.lcd2_valid ) {
		lcd2_valid = 1;
	}

	/* Search for custom_nv structures */
	for (i = 0; i < ARRAY_SIZE(lcd_panel_list); i++) {
		if (!strncmp(lcd_panel_list[i].fb_pdata.mode_str,
			"custom3_nv", strlen(lcd_panel_list[i].fb_pdata.mode->name))) {
			struct fb_videomode *videomode = lcd_panel_list[i].fb_pdata.mode;

			if ( lcd1_valid ) {
				/* Skip the name field */
				memcpy(&videomode->refresh, &lcd_config->lcd1.video_mode.refresh,
					sizeof(lcd_custom3_nvram) - sizeof(lcd_custom3_nvram.name));

				/* Set backlight enable function, if it is necessarry */
				if (lcd_config->lcd1.is_bl_enable_func) {
					lcd_panel_list[i].bl_enable = lcd_bl_enable;
				}
			}
		}
		else if (!strncmp(lcd_panel_list[i].fb_pdata.mode_str,
			"custom4_nv", strlen(lcd_panel_list[i].fb_pdata.mode->name))) {
			struct fb_videomode *videomode = lcd_panel_list[i].fb_pdata.mode;

			if ( lcd2_valid ) {
				/* Skip the name field */
				memcpy(&videomode->refresh, &lcd_config->lcd2.video_mode.refresh,
					sizeof(lcd_custom4_nvram) - sizeof(lcd_custom4_nvram.name));

				/* Set backlight enable function, if it is necessarry */
				if (lcd_config->lcd2.is_bl_enable_func) {
					lcd_panel_list[i].bl_enable = lcd_bl_enable;
				}
			}
		}
	}
}
#endif /* CONFIG_FB_MXS */

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx28_set_input_clk(24000000, 24000000, 32000, 50000000);

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
	mx28_set_lcd_struct_from_bl_shared(
		(nv_lcd_config_t *)((unsigned int)desc->boot_params + BL_SHARED_RAM_OFFS_LCD));
#endif
}

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28_ccardimx28_led_pwm[2] = {
	[0] = {
		.name = "led-pwm0",
		.pwm = 0,
		},
	[1] = {
		.name = "led-pwm1",
		.pwm = 1,
		},
};

struct mxs_pwm_leds_plat_data mx28_ccardimx28_led_data = {
	.num = ARRAY_SIZE(mx28_ccardimx28_led_pwm),
	.leds = mx28_ccardimx28_led_pwm,
};

static struct resource mx28_ccardimx28_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28_ccardimx28_init_pwm_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28_ccardimx28_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28_ccardimx28_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_ccardimx28_init_pwm_leds(void)
{
	;
}
#endif

#if defined(CONFIG_MXS_PWM) || defined(CONFIG_MXS_PWM_MODULE)
static struct mxs_pwm_channel mx28_pwm_channels[] = {
#if defined(CONFIG_CCARDIMX28_PWM_CH0)
	{
		.channel = 0,
	},
#endif /* CONFIG_CCARDIMX28_PWM_CH0 */
#if defined(CONFIG_CCARDIMX28_PWM_CH1)
	{
		.channel = 1,
	},
#endif
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
static void mx28_pwm_init(void) {}
#endif

static void __init mx28_ccardimx28_device_init(void)
{
	/* Add mx28_ccardimx28 special code */
	i2c_device_init();
	spi_device_init();
	mx28_pwm_init();
	mx28_ccardimx28_init_pwm_leds();
	mx28_ccardimx28_init_backlight();
	mx28_init_lcdif();
}

#ifdef CONFIG_MICREL_PHY
static int ksz8031_phy_fixup(struct phy_device *phydev)
{
	phydev->dev_flags |= MICREL_PHY_50MHZ_CLK;

	return 0;
}
#endif

static void __init mx28_ccardimx28_init_machine(void)
{
	/* Setup hwid information, passed through Serial ATAG */
	ccardimx28_set_hwid(system_serial_low, system_serial_high);

	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif
	mx28_gpio_init();
	mx28_ccardimx28_pins_init();
	mx28_device_init();
	mx28_ccardimx28_device_init();

	ccardimx28_create_sysfs_entries();

#ifdef CONFIG_MICREL_PHY
	/* Register the fixup for the PHYs */
	phy_register_fixup_for_uid(PHY_ID_KSZ8031, MICREL_PHY_ID_MASK, ksz8031_phy_fixup);
#endif

	/* init the system revision */
	system_rev = 0x28011;
}

#define CCARDXMX28_BACKLIGHT_GPIO	PINID_AUART1_RX
/* backlight enable function */
void lcd_bl_enable(int enable)
{
#if defined(CONFIG_BACKLIGHT_MXS) || defined(CONFIG_BACKLIGHT_MXS_MODULE)
	/* Placeholder for LCD enabling operations */
#else
	/* GPIO on/off control */
	/* inverse logic: 0 == backlight enabled */
	gpio_set_value(MXS_PIN_TO_GPIO(CCARDXMX28_BACKLIGHT_GPIO), !enable);
#endif
}

MACHINE_START(CCARDWMX28JS, "ConnectCard Wi-i.MX28")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28_ccardimx28_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END

MACHINE_START(CCARDMX28JS, "ConnectCard i.MX28")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28_ccardimx28_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END

MACHINE_START(CCARDIMX28JS, "ConnectCard for MX28")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28_ccardimx28_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
