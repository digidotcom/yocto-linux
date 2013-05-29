/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/pinctrl.h>

#include "mx28_pins.h"
#include "regs-pinctrl.h"
#include "mx28_ccardimx28.h"

#define DEV_NAME	"ConnectCard i.MX28"

static void __init mx28_ccardimx28_init_pin_group(struct pin_desc *pins, unsigned count)
{
	int i;
	struct pin_desc *pin;

	for (i = 0; i < count; i++) {
		pin = pins + i;

		if (0 == pin->id && NULL == pin->name) {
			/* empty item in array (id=0, name=NULL). Skip */
			continue;
		}

		if (pin->fun == PIN_GPIO) {
			if (!pin->sysfs)
				gpio_request(MXS_PIN_TO_GPIO(pin->id), pin->name);
		} else
			mxs_request_pin(pin->id, pin->fun, pin->name);

		if (pin->drive) {
			mxs_set_strength(pin->id, pin->strength, pin->sysfs , pin->name);
			mxs_set_voltage(pin->id, pin->voltage, pin->sysfs, pin->name);
		}

		if (pin->pull) {
			mxs_set_pullup(pin->id, pin->pullup, pin->sysfs, pin->name);
		}

		if ((pin->fun == PIN_GPIO) && (!pin->sysfs)) {
			if (pin->output)
				gpio_direction_output(MXS_PIN_TO_GPIO(pin->id),
							pin->data);
			else
				gpio_direction_input(MXS_PIN_TO_GPIO(pin->id));
		}
	}
}

/* This function frees the GPIOs after reconfiguring them as inputs.
 * This allows to reduce consumption for power management.
 */
static void mx28_ccardimx28_free_pin_group(struct pin_desc *pins, unsigned count)
{
	int i;
	struct pin_desc *pin;

	for (i = 0; i < count; i++) {
		pin = pins + i;

		if (pin->fun == PIN_GPIO) {
			if (!pin->sysfs)
				gpio_free(MXS_PIN_TO_GPIO(pin->id));
		} else
			mxs_release_pin(pin->id, pin->name);

		gpio_request(MXS_PIN_TO_GPIO(pin->id), pin->name);
		gpio_direction_output(MXS_PIN_TO_GPIO(pin->id), 0);
	}
}

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
static struct pin_desc mx28_ccardimx28_mmc_data_pins[][4] = {
	[0] = {
		{
			.name		= "SSP0_DATA0",
			.id		= PINID_SSP0_DATA0,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP0_DATA1",
			.id		= PINID_SSP0_DATA1,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive 		= 1,
			.pull 		= 1,
		}, {
			.name		= "SSP0_DATA2",
			.id		= PINID_SSP0_DATA2,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP0_DATA3",
			.id		= PINID_SSP0_DATA3,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive 		= 1,
			.pull		= 1,
		},
	},
	[2] = { /* SSP2 (connected to WLAN on wireless variant) */
		{
			.name		= "SSP2_DATA0",
			.id		= PINID_SSP0_DATA4,
			.fun		= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP2_DATA1",
			.id		= PINID_SSP2_SS1,
			.fun		= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP2_DATA2",
			.id		= PINID_SSP2_SS2,
			.fun		= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
		      .name		= "SSP2_DATA3",
		      .id		= PINID_SSP0_DATA5,
		      .fun		= PIN_FUN2,
		      .strength		= PAD_4MA,
		      .voltage		= PAD_3_3V,
		      .pullup		= 1,
		      .drive		= 1,
		      .pull		= 1,
		},
	}
};

static struct pin_desc mx28_ccardimx28_mmc_ctrl_pins[][3] = {
	[0] = {
		{
			.name		= "SSP0_CMD",
			.id		= PINID_SSP0_CMD,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP0_SCK",
			.id		= PINID_SSP0_SCK,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 0,
			.drive		= 1,
			.pull		= 0,
		},
#ifdef CONFIG_MMC_MXS_SSP0_CARD_DETECT
		{
			.name		= "SSP0_DETECT",
			.id		= PINID_SSP0_DETECT,
			.fun		= PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
			.sysfs		= 1,
		},
#endif
	},
	[2] = {
		{
			.name		= "SSP2_CMD",
			.id		= PINID_SSP0_DATA6,
			.fun		= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 1,
			.drive		= 1,
			.pull		= 1,
		}, {
			.name		= "SSP2_SCK",
			.id		= PINID_SSP0_DATA7,
			.fun		= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup		= 0,
			.drive		= 1,
			.pull		= 0,
		},
	}
};

void mx28_ccardimx28_mmc_pins_init(void)
{
#if defined(CONFIG_MMC_MXS_SSP0_ENABLE)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_mmc_data_pins[0],
				       CONFIG_MMC_MXS_SSP0_DATA_BITS);

	/* Initialize the control pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_mmc_ctrl_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_mmc_ctrl_pins[0]));
#endif
#if defined(CONFIG_MMC_MXS_SSP2_ENABLE)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_mmc_data_pins[2],
				       ARRAY_SIZE(mx28_ccardimx28_mmc_data_pins[2]));

	/* Initialize the control pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_mmc_ctrl_pins[2],
				       ARRAY_SIZE(mx28_ccardimx28_mmc_ctrl_pins[2]));
#endif
}
#else
int mx28_ccardimx28_mmc_pins_init(void) {return 0;}
#endif /* defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE) */

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)\
	|| defined(CONFIG_FEC_L2SWITCH)
static struct pin_desc mx28_ccardimx28_eth_common_pins[] = {
	/* Common Ethernet lines */
	{
		.name = "ENET_MDC",
		.id = PINID_ENET0_MDC,
		.fun = PIN_FUN1,
		.strength = PAD_8MA,
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
	{
		.name = "ENET_MDIO",
		.id = PINID_ENET0_MDIO,
		.fun = PIN_FUN1,
		.strength = PAD_8MA,
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
	{
		.name = "ENET_CLK",
		.id = PINID_ENET_CLK,
		.fun = PIN_FUN1,
		.strength = PAD_8MA,
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
};

/* Pins use to program the PHY addresses */
static struct pin_desc mx28_ccardimx28_eth_phy_addr_pins[] = {
	{
		.name = "ENET0_PHY_ADDR",
		.id = PINID_ENET0_RX_EN,
		.fun = PIN_GPIO,
		.strength = PAD_4MA,
		.output = 1,
		.data = 0,	/* = PHY Addr 0 */
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
	{
		.name = "ENET1_PHY_ADDR",
		.id = PINID_ENET0_CRS,
		.fun = PIN_GPIO,
		.strength = PAD_4MA,
		.output = 1,
		.data = 1,	/* = PHY Addr 3 */
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
};

/* Used to set the PHY address pins back to there normal MAC functions */
static struct pin_desc mx28_ccardimx28_eth_phy_addr_mac_pins[] = {
	{
		.name = "ENET0_RX_EN",
		.id = PINID_ENET0_RX_EN,
		.fun = PIN_FUN1,
		.strength = PAD_8MA,
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive	= 1,
	},
	{
		.name = "ENET1_RX_EN",
		.id = PINID_ENET0_CRS,
		.fun = PIN_FUN2,
		.strength = PAD_8MA,
		.pull = 1,
		.pullup = 1,
		.voltage = PAD_3_3V,
		.drive = 1,
	},
};

static struct pin_desc mx28_ccardimx28_eth_port_pins[][7] = {
	[0] = {
		{
			.name = "ENET0_RXD0",
			.id = PINID_ENET0_RXD0,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_RXD1",
			.id = PINID_ENET0_RXD1,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_TX_EN",
			.id = PINID_ENET0_TX_EN,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_TXD0",
			.id = PINID_ENET0_TXD0,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_TXD1",
			.id = PINID_ENET0_TXD1,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_RST",
			.id = PINID_PWM4,
			.fun = PIN_GPIO,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET0_RX_EN",
			.id = PINID_ENET0_RX_EN,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
	},
	[1] = {
		{
			.name = "ENET1_RXD0",
			.id = PINID_ENET0_RXD2,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name = "ENET1_RXD1",
			.id = PINID_ENET0_RXD3,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name = "ENET1_TX_EN",
			.id = PINID_ENET0_COL,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name = "ENET1_TXD0",
			.id = PINID_ENET0_TXD2,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name = "ENET1_TXD1",
			.id = PINID_ENET0_TXD3,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name = "ENET1_RST",
			.id = PINID_ENET0_RX_CLK,
			.fun = PIN_GPIO,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "ENET1_RX_EN",
			.id = PINID_ENET0_CRS,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
	},
};

static struct pin_desc mx28_ccardimx28_eth_led_pins[2][2] = {
#if !defined(CONFIG_MXS_AUART1_4WIRE) && defined(CONFIG_CCARDIMX28_ENET0_LEDS)
	[0] = {
		/* Ethernet 0 LEDs (conflict with AUART1 CTS/RTS) */
		{
			.name  = "ENET0_LINK_LED",
			.id    = PINID_AUART1_CTS,
			.fun   = PIN_GPIO,
			.strength = PAD_4MA,
			.output = 1,
			.data = 1,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name  = "ENET0_ACTIVE_LED",
			.id    = PINID_AUART1_RTS,
			.fun   = PIN_GPIO,
			.strength = PAD_4MA,
			.output = 1,
			.data = 1,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
	},
#endif
#if defined(CONFIG_CCARDIMX28_ENET1_LEDS) && \
   !defined(CONFIG_MXS_AUART3_DEVICE_ENABLE) && !defined(CONFIG_CCMX28_CAN1)
	[1] = {
	 /* LEDs (conflict with AUART3 TX/RX and LRADC) */
		{
			.name  = "ENET1_LINK_LED",
			.id    = PINID_AUART3_TX,
			.fun   = PIN_GPIO,
			.strength = PAD_4MA,
			.output = 1,
			.data = 1,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
		{
			.name  = "ENET1_ACTIVE_LED",
			.id    = PINID_AUART3_RX,
			.fun   = PIN_GPIO,
			.strength = PAD_4MA,
			.output = 1,
			.data = 1,
			.pull = 1,
			.pullup = 1,
			.voltage = PAD_3_3V,
			.drive = 1,
		},
	},
#endif /* CONFIG_CCARDIMX28_ENET1_LEDS */
};

int mx28_ccardimx28_eth_pins_init(void)
{
	/* Configure common pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_common_pins,
				       ARRAY_SIZE(mx28_ccardimx28_eth_common_pins));

	/* Configure port 0 pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_port_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_eth_port_pins[0]));
	/* Configure port 0 LEDs pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_led_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_eth_led_pins[0]));


#if defined(CONFIG_CCARDIMX28_ENET1)
	/* Configure port 1 pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_port_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_eth_port_pins[1]));
	/* Configure port 1 LEDs pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_led_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_eth_led_pins[1]));
#endif /* CONFIG_CCARDIMX28_ENET1 */

	return 0;
}

int mx28_ccardimx28_eth_pins_deinit(void)
{
	/* Free common pins */
	mx28_ccardimx28_free_pin_group(mx28_ccardimx28_eth_common_pins,
				       ARRAY_SIZE(mx28_ccardimx28_eth_common_pins));

	/* Free port 0 pins */
	mx28_ccardimx28_free_pin_group(mx28_ccardimx28_eth_port_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_eth_port_pins[0]));
	/* Free port 0 LEDs pins */
	mx28_ccardimx28_free_pin_group(mx28_ccardimx28_eth_led_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_eth_led_pins[0]));


#if defined(CONFIG_CCARDIMX28_ENET1)
	/* Configure port 1 pins */
	mx28_ccardimx28_free_pin_group(mx28_ccardimx28_eth_port_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_eth_port_pins[1]));
	/* Configure port 1 LEDs pins */
	mx28_ccardimx28_free_pin_group(mx28_ccardimx28_eth_led_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_eth_led_pins[1]));
#endif /* CONFIG_CCARDIMX28_ENET1 */

	return 0;
}
#define ENET0_RESET		PINID_PWM4
#define ENET0_PHY_ADD	0

#define ENET1_RESET		PINID_ENET0_RX_CLK
#define ENET1_PHY_ADD	1

int mx28_ccardimx28_enet_gpio_init(void)
{
#if defined(CONFIG_CCARDIMX28_AUTODETECT_MODULE)
	int hv = get_hwid_hv();

	if (1 == hv) {
		/* Module V1 has bad PHY address strapping. Force it here */
		/* Set PHY addresses */
		mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_phy_addr_pins,
					ARRAY_SIZE(mx28_ccardimx28_eth_phy_addr_pins));
	}
#elif defined(CONFIG_CCARDIMX28_V1)
	/* Module V1 has bad PHY address strapping. Force it here */
	/* Set PHY addresses */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_phy_addr_pins,
				ARRAY_SIZE(mx28_ccardimx28_eth_phy_addr_pins));
#endif

	/* Reset PHY's */
	gpio_request(MXS_PIN_TO_GPIO(ENET0_RESET), "ENET0_RST");
	gpio_direction_output(MXS_PIN_TO_GPIO(ENET0_RESET), 0);

#ifdef CONFIG_CCARDIMX28_ENET1
	gpio_request(MXS_PIN_TO_GPIO(ENET1_RESET), "ENET1_RST");
	gpio_direction_output(MXS_PIN_TO_GPIO(ENET1_RESET), 0);
#endif

	mdelay(50);

	/* Take PHY's out of reset */
	gpio_direction_output(MXS_PIN_TO_GPIO(ENET0_RESET), 1);

#ifdef CONFIG_CCARDIMX28_ENET1
	gpio_direction_output(MXS_PIN_TO_GPIO(ENET1_RESET), 1);
#endif

	/* The PHY will latch the address pin and set the addresses */
	mdelay(50);

#if defined(CONFIG_CCARDIMX28_AUTODETECT_MODULE)
	if (1 == hv) {
		/* Release PHY address pins */
		gpio_free(MXS_PIN_TO_GPIO(mx28_ccardimx28_eth_phy_addr_pins[0].id));
#ifdef CONFIG_CCARDIMX28_ENET1
		gpio_free(MXS_PIN_TO_GPIO(mx28_ccardimx28_eth_phy_addr_pins[1].id));
#endif

		/* Program back to ENET/MAC funtions */
		mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_phy_addr_mac_pins,
					ARRAY_SIZE(mx28_ccardimx28_eth_phy_addr_mac_pins));
	}
#elif defined(CONFIG_CCARDIMX28_V1)
	/* Release PHY address pins */
	gpio_free(MXS_PIN_TO_GPIO(mx28_ccardimx28_eth_phy_addr_pins[0].id));
#ifdef CONFIG_CCARDIMX28_ENET1
	gpio_free(MXS_PIN_TO_GPIO(mx28_ccardimx28_eth_phy_addr_pins[1].id));
#endif

	/* Program back to ENET/MAC funtions */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_eth_phy_addr_mac_pins,
				ARRAY_SIZE(mx28_ccardimx28_eth_phy_addr_mac_pins));
#endif

	return 0;
}

#else
int mx28_ccardimx28_eth_pins_init(void)
{
	return 0;
}
int mx28_ccardimx28_eth_pins_deinit(void)
{
	return 0;
}
int mx28_ccardimx28_enet_gpio_init(void)
{
	return 0;
}
#endif /* CONFIG_FEC */

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static struct pin_desc mx28_ccardimx28_lcd_data_pins[] = {
	{
		.name		= "LCD_D00",
		.id		= PINID_LCD_D00,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name 		= "LCD_D01",
		.id		= PINID_LCD_D01,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name 		= "LCD_D02",
		.id		= PINID_LCD_D02,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D03",
		.id		= PINID_LCD_D03,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D04",
		.id		= PINID_LCD_D04,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D05",
		.id		= PINID_LCD_D05,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D06",
		.id		= PINID_LCD_D06,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D07",
		.id		= PINID_LCD_D07,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D08",
		.id		= PINID_LCD_D08,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D09",
		.id		= PINID_LCD_D09,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D10",
		.id		= PINID_LCD_D10,
		.fun		= PIN_FUN1,
		.strength 	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D11",
		.id		= PINID_LCD_D11,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D12",
		.id		= PINID_LCD_D12,
		.fun		= PIN_FUN1,
		.strength	 = PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D13",
		.id		= PINID_LCD_D13,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D14",
		.id		= PINID_LCD_D14,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D15",
		.id		= PINID_LCD_D15,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D16",
		.id		= PINID_LCD_D16,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D17",
		.id		= PINID_LCD_D17,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_D18",
		.id		= PINID_LCD_D18,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D19",
		.id		= PINID_LCD_D19,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D20",
		.id		= PINID_LCD_D20,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D21",
		.id		= PINID_LCD_D21,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D22",
		.id		= PINID_LCD_D22,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 }, {
		.name		= "LCD_D23",
		.id		= PINID_LCD_D23,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	 },
};

static struct pin_desc mx28_ccardimx28_lcd_ctrl_pins[] = {
	{
#ifdef CONFIG_JSCCARDIMX28_V1
		.name		= "LCD_RESET",
		.id		= PINID_LCD_RESET,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
#endif
		.name		= "LCD_DOTCLK",
		.id		= PINID_LCD_DOTCK,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_ENABLE",
		.id		= PINID_LCD_ENABLE,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_VSYNC",
#ifdef CONFIG_JSCCARDIMX28_V1
		.id		= PINID_LCD_VSYNC,
		.fun		= PIN_FUN1,
#else
		.id		= PINID_LCD_RESET,
		.fun		= PIN_FUN2,
#endif /* CONFIG_JSCCARDIMX28_V1 */
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LCD_HSYNC",
		.id		= PINID_LCD_HSYNC,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
};

static struct pin_desc mx28_ccardimx28_lcd_backlight_pins[] = {
#ifndef CONFIG_JSCCARDIMX28_V1
	{
		.name		= "LCD_BACKLIGHT_ENABLE",
		.id 		= PINID_AUART1_RX,
#if defined(CONFIG_BACKLIGHT_MXS) || defined(CONFIG_BACKLIGHT_MXS_MODULE)
		.fun		= PIN_FUN3,
#else
		.fun		= PIN_GPIO,
#endif /* CONFIG_BACKLIGHT_MXS */
		.strength	= PAD_8MA,
		.pull		= 1,
		.pullup		= 1,
		.output		= 1,
		.data		= 1,	/* initially disabled (inverse logic) */
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
#endif /* CONFIG_JSCCARDIMX28_V1 */
};

int mx28_ccardimx28_video_pins_init(int databits)
{
	/* Initialize the data pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_lcd_data_pins,
				       databits);

	/* Initialize the control pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_lcd_ctrl_pins,
				       ARRAY_SIZE(mx28_ccardimx28_lcd_ctrl_pins));

	/* Initialize the backlight pin(s) */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_lcd_backlight_pins,
				       ARRAY_SIZE(mx28_ccardimx28_lcd_backlight_pins));

	return 0;
}
#else
int mx28_ccardimx28_video_pins_init(int databits)
{
	return 0;
}
#endif /* defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE) */

#if (defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)) && \
	!defined(CONFIG_MXS_AUART1_4WIRE)
static struct pin_desc mx28_ccardimx28_touch_pins[] = {
	{
		.name		= "LRADC2-DaisyChain",
		.id		= PINID_SSP1_DATA0,
		.fun		= PIN_GPIO,
		.output		= 0,
		.pull		= 1,
		.pullup		= 1,	/* A 3.3V pull-up is needed in this pin
					 * to avoid a strange voltage divisor
					 * that sets this pin to 0.68V when the
					 * touch enables the touch detect.
					 * This is due to daisy-chain GPIO. */
		.voltage	= PAD_3_3V,
		.drive		= 1,
	}, {
		.name		= "LRADC3-DaisyChain",
		.id		= PINID_SSP1_SCK,
		.fun		= PIN_GPIO,
		.output		= 0,
	}, {
		.name		= "LRADC4-DaisyChain",
		.id		= PINID_AUART1_CTS,
		.fun		= PIN_GPIO,
		.output		= 0,
	}, {
		.name		= "LRADC5-DaisyChain",
		.id		= PINID_AUART1_RTS,
		.fun		= PIN_GPIO,
		.output		= 0,
	},
};

void mx28_ccardimx28_touch_pins_init(void)
{
	/* Initialize the data pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_touch_pins,
				       ARRAY_SIZE(mx28_ccardimx28_touch_pins));
}
#else
void mx28_ccardimx28_touch_pins_init(void) {}
#endif /* defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE) */

#if (defined(CONFIG_SERIAL_MXS_DUART) || defined(CONFIG_SERIAL_MXS_DUART_MODULE)) && \
    !defined(CONFIG_I2C_MXS_SELECT0)
static struct pin_desc mx28_ccardimx28_duart_pins[] = {
	{
		.name = "DUART.RX",
		.id = PINID_I2C0_SCL,
		.fun = PIN_FUN3,
	},
	{
		.name = "DUART.TX",
		.id = PINID_I2C0_SDA,
		.fun = PIN_FUN3,
	},
};

int mx28_ccardimx28_duart_pins_init(void)
{
	/* Initialize the pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_duart_pins,
				       ARRAY_SIZE(mx28_ccardimx28_duart_pins));

	return 0;
}
#else
int mx28_ccardimx28_duart_pins_init(void)
{
	return 0;
}
#endif /* (CONFIG_SERIAL_MXS_DUART || CONFIG_SERIAL_MXS_DUART_MODULE) &&
	  !CONFIG_I2C_MXS_SELECT0 */

#if defined(CONFIG_SERIAL_MXS_AUART) || defined(CONFIG_SERIAL_MXS_AUART_MODULE)
static struct pin_desc mx28_ccardimx28_auart_pins[][4] = {
	[1] = {
		{
			.name  = "AUART1.RX",
			.id    = PINID_AUART1_RX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART1.TX",
			.id    = PINID_AUART1_TX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART1.RTS",
			.id    = PINID_AUART1_RTS,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART1.CTS",
			.id    = PINID_AUART1_CTS,
			.fun   = PIN_FUN1,
		},
	},
	[2] = {
		{
			.name  = "AUART2.RX",
			.id    = PINID_AUART2_RX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART2.TX",
			.id    = PINID_AUART2_TX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART2.RTS",
			.id    = PINID_AUART2_RTS,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART2.CTS",
			.id    = PINID_AUART2_CTS,
			.fun   = PIN_FUN1,
		},
	},
	[3] = {
		{
			.name  = "AUART3.RX",
			.id    = PINID_AUART3_RX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART3.TX",
			.id    = PINID_AUART3_TX,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART3.RTS",
			.id    = PINID_AUART3_RTS,
			.fun   = PIN_FUN1,
		},
		{
			.name  = "AUART3.CTS",
			.id    = PINID_AUART3_CTS,
			.fun   = PIN_FUN1,
		},
	},
	[4] = {
		{
			.name  = "AUART4.RX",
			.id    = PINID_SAIF0_BITCLK,
			.fun   = PIN_FUN3,
		},
		{
			.name  = "AUART4.TX",
			.id    = PINID_SAIF0_SDATA0,
			.fun   = PIN_FUN3,
		},
		{
			.name  = "AUART4.RTS",
			.id    = PINID_SAIF0_LRCLK,
			.fun   = PIN_FUN3,
		},
		{
			.name  = "AUART4.CTS",
			.id    = PINID_SAIF0_MCLK,
			.fun   = PIN_FUN3,
		},
	},
};

void mx28_ccardimx28_auarts_pins_init(void)
{
	/* AUART1 */
#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
#ifdef CONFIG_MXS_AUART1_4WIRE
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[1], 4);
#else
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[1], 2);
#endif /* CONFIG_MXS_AUART1_4WIRE */
#endif /* CONFIG_MXS_AUART1_DEVICE_ENABLE */

	/* AUART2 */
#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
#ifdef CONFIG_MXS_AUART2_4WIRE
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[2], 4);
#else
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[2], 2);
#endif /* CONFIG_MXS_AUART2_4WIRE */
#endif /* CONFIG_MXS_AUART2_DEVICE_ENABLE */

	/* AUART3 */
#if defined(CONFIG_MXS_AUART3_DEVICE_ENABLE) && !defined(CONFIG_CCMX28_CAN1) && \
   !defined(CONFIG_CCARDIMX28_ENET1_LEDS)
#ifdef CONFIG_MXS_AUART3_4WIRE
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[3], 4);
#else
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[3], 2);
#endif /* CONFIG_MXS_AUART3_4WIRE */
#endif /* CONFIG_MXS_AUART3_DEVICE_ENABLE && !CONFIG_CCMX28_CAN1 &&
	 !CONFIG_CCARDIMX28_ENET1_LEDS */

	/* AUART4 */
#ifdef CONFIG_MXS_AUART4_DEVICE_ENABLE
#if defined(CONFIG_MXS_AUART4_4WIRE) && !defined(CONFIG_CCARDIMX28_ENET1_LEDS)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[4], 4);
#else
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_auart_pins[4], 2);
#endif /* CONFIG_MXS_AUART4_4WIRE && !CONFIG_CCARDIMX28_ENET1_LEDS */
#endif /* CONFIG_MXS_AUART4_DEVICE_ENABLE */
}
#else
void mx28_ccardimx28_auarts_pins_init(void) {}
#endif /* CONFIG_SERIAL_MXS_AUART || CONFIG_SERIAL_MXS_AUART_MODULE */

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
static struct pin_desc mx28_ccardimx28_can_pins[][2] = {
	[0] = {
		{
			.name	= "CAN0_TX",
			.id	= PINID_GPMI_RDY2,
			.fun	= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup	= 0,
			.drive 	= 1,
			.pull 	= 0,
		},
		{
			.name	= "CAN0_RX",
			.id	= PINID_GPMI_RDY3,
			.fun	= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup	= 0,
			.drive 	= 1,
			.pull 	= 0,
		},
	},
	[1] = {
		{
			.name	= "CAN1_TX",
			.id	= PINID_GPMI_CE2N,
			.fun	= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup	= 0,
			.drive 	= 1,
			.pull 	= 0,
		},
		{
			.name	= "CAN1_RX",
			.id	= PINID_GPMI_CE3N,
			.fun	= PIN_FUN2,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.pullup	= 0,
			.drive 	= 1,
			.pull 	= 0,
		},
	},
};

void mx28_ccardimx28_can_pins_init(void)
{
#ifdef CONFIG_CCMX28_CAN0
	/* Initialize the pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_can_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_can_pins[0]));
#endif /* CONFIG_CCMX28_CAN0 */

#if defined(CONFIG_CCMX28_CAN1) && !defined(CONFIG_MXS_AUART3_DEVICE_ENABLE)
	/* Initialize the pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_can_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_can_pins[1]));
#endif /* CONFIG_CCMX28_CAN1 && !CONFIG_MXS_AUART3_DEVICE_ENABLE */
}
#else
void mx28_ccardimx28_can_pins_init(void) {}
#endif /* CONFIG_CAN_FLEXCAN || CONFIG_CAN_FLEXCAN_MODULE */

#if defined(CONFIG_I2C_MXS) || defined(CONFIG_I2C_MXS_MODULE)
static struct pin_desc mx28_ccardimx28_i2c_pins[][2] = {
	[0] = {
		{
			.name = "I2C0.SCL",
			.id = PINID_I2C0_SCL,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "I2C0.SDA",
			.id = PINID_I2C0_SDA,
			.fun = PIN_FUN1,
			.strength = PAD_8MA,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
	},
	[1] = {
		{
			.name = "I2C1.SCL",
			.id = PINID_PWM0,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
		{
			.name = "I2C1.SDA",
			.id = PINID_PWM1,
			.fun = PIN_FUN2,
			.strength = PAD_8MA,
			.voltage = PAD_3_3V,
			.drive	= 1,
		},
	},
};

void mx28_ccardimx28_i2c_pins_init(void)
{
	/* I2C0 */
#if defined(CONFIG_I2C_MXS_SELECT0) && !defined(CONFIG_SERIAL_MXS_DUART)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_i2c_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_i2c_pins[0]));
#endif /* CONFIG_I2C_MXS_SELECT0 && !CONFIG_SERIAL_MXS_DUART */

	/* I2C1 */
#if defined(CONFIG_I2C_MXS_SELECT1)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_i2c_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_i2c_pins[1]));
#endif /* CONFIG_I2C_MXS_SELECT1 */
}
#else
void mx28_ccardimx28_i2c_pins_init(void) {}
#endif /* CONFIG_I2C_MXS || CONFIG_I2C_MXS_MODULE */

#if (defined(CONFIG_SND_MXS_SOC_DAI) || defined(CONFIG_SND_MXS_SOC_DAI_MODULE)) && \
	defined(CONFIG_SND_MXS_SOC_CCARDIMX28) && !defined(CONFIG_MXS_AUART4_DEVICE_ENABLE)
static struct pin_desc mx28_ccardimx28_audio_pins[] = {
	/* Audio lines are daisy chained SSP3 lines */
	{
		.name	= "SAIF0_MCLK",
		.id	= PINID_SAIF0_MCLK,
		.fun	= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.pullup	= 1,
		.drive 	= 1,
		.pull 	= 1,
	},
	{
		.name	= "SAIF0_LRCLK",
		.id	= PINID_SAIF0_LRCLK,
		.fun	= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.pullup	= 1,
		.drive 	= 1,
		.pull 	= 1,
	},
	{
		.name	= "SAIF0_BITCLK",
		.id	= PINID_SAIF0_BITCLK,
		.fun	= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.pullup	= 1,
		.drive 	= 1,
		.pull 	= 1,
	},
	{
		.name	= "SAIF0_SDATA0",
		.id	= PINID_SAIF0_SDATA0,
		.fun	= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.pullup	= 1,
		.drive 	= 1,
		.pull 	= 1,
	},
	{
		.name	= "SAIF1_SDATA0",
		.id	= PINID_LCD_VSYNC,
		.fun	= PIN_FUN2,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.pullup	= 1,
		.drive 	= 1,
		.pull 	= 1,
	},
};

int mx28_ccardimx28_audio_pins_init(void)
{
	/* Initialize the pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_audio_pins,
				       ARRAY_SIZE(mx28_ccardimx28_audio_pins));

	return 0;
}
#else
int mx28_ccardimx28_audio_pins_init(void)
{
	return 0;
}
#endif /* CONFIG_SND_MXS_SOC_DAI && ... */

#if (defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)) && \
	(defined(CONFIG_CCARDIMX28_SPI_SSP1_ENABLE) || \
	 defined(CONFIG_CCARDIMX28_SPI_SSP3_ENABLE))
static struct pin_desc mx28_ccardimx28_spi_pins[][4] = {
	[1] = {
		/* Configurations for SSP1 (SPI) */
		{
			.name  = "SSP1_SCK",
			.id    = PINID_SSP1_SCK,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI1_MOSI",
			.id    = PINID_SSP1_CMD,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI1_MISO",
			.id    = PINID_SSP1_DATA0,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI1_SSn",
			.id    = PINID_SSP1_DATA3,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
	},
	[3] = {
		{
			.name  = "SSP3_SCK",
			.id    = PINID_SSP3_SCK,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI3_MOSI",
			.id    = PINID_SSP3_MOSI,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI3_MISO",
			.id    = PINID_SSP3_MISO,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
		{
			.name  = "SPI3_SSn",
			.id    = PINID_SSP3_SS0,
			.fun   = PIN_FUN1,
			.strength	= PAD_4MA,
			.voltage	= PAD_3_3V,
			.drive 	= 1,
		},
	},
};

void mx28_ccardimx28_spi_pins_init(void)
{
	/* SPI */
#if defined(CONFIG_CCARDIMX28_SPI_SSP1_ENABLE) && \
    !(defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE))
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_spi_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_spi_pins[1]));
#endif

#if defined(CONFIG_CCARDIMX28_SPI_SSP3_ENABLE) && \
    !defined(CONFIG_MXS_AUART4_DEVICE_ENABLE) && \
    !defined(CONFIG_SND_MXS_SOC_CCARDIMX28)
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_spi_pins[3],
				       ARRAY_SIZE(mx28_ccardimx28_spi_pins[3]));
#endif
}
#else
void mx28_ccardimx28_spi_pins_init(void) {}
#endif /* CONFIG_SPI_MXS || CONFIG_SPI_MXS_MODULE */

#if defined(CONFIG_USB_OTG)
static struct pin_desc mx28_ccardimx28_usb_otg_pins[] = {
	{
		.name 	= "USB0_ID",
		.id 	= PINID_PWM2,
		.fun	= PIN_FUN2,
		.data 	= 1,
		.pull 	= 1,
		.pullup = 1,
	}
};

int mx28_ccardimx28_usb_otg_pins_init(void)
{
	/* Initialize the pins */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_usb_otg_pins,
				       ARRAY_SIZE(mx28_ccardimx28_usb_otg_pins));

	return 0;
}
#else
int mx28_ccardimx28_usb_otg_pins_init(void)
{
	return 0;
}
#endif /* CONFIG_USB_OTG */

int enable_gpmi = { 0 };
static int __init gpmi_setup(char *__unused)
{
	enable_gpmi = 1;
	return 1;
}

__setup("gpmi", gpmi_setup);

static struct pin_desc mx28_ccardimx28_gpmi_pins[] = {
	{
		 .name     = "GPMI D0",
		 .id       = PINID_GPMI_D00,
		 .fun      = PIN_FUN1,
		 .strength = PAD_4MA,
		 .voltage  = PAD_1_8V,
		 .pullup   = 0,
		 .drive    = 1
	},
	{
		.name     = "GPMI D1",
		.id       = PINID_GPMI_D01,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D2",
		.id       = PINID_GPMI_D02,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D3",
		.id       = PINID_GPMI_D03,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D4",
		.id       = PINID_GPMI_D04,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D5",
		.id       = PINID_GPMI_D05,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D6",
		.id       = PINID_GPMI_D06,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI D7",
		.id       = PINID_GPMI_D07,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI CE0-",
		.id       = PINID_GPMI_CE0N,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 1,
		.drive    = 1
	},
	{
		.name     = "GPMI RDY0",
		.id       = PINID_GPMI_RDY0,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 1,
		.drive    = 1
	},
	{
		.name     = "GPMI RD-",
		.id       = PINID_GPMI_RDN,
		.fun      = PIN_FUN1,
		.strength = PAD_8MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI WR-",
		.id       = PINID_GPMI_WRN,
		.fun      = PIN_FUN1,
		.strength = PAD_8MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI ALE",
		.id       = PINID_GPMI_ALE,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI CLE",
		.id       = PINID_GPMI_CLE,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
	{
		.name     = "GPMI RST-",
		.id       = PINID_GPMI_RESETN,
		.fun      = PIN_FUN1,
		.strength = PAD_4MA,
		.voltage  = PAD_1_8V,
		.pullup   = 0,
		.drive    = 1
	},
};


#if defined(CONFIG_BT)

#define BT_PWD_ID   PINID_GPMI_RDY1

static struct pin_desc mx28_ccardimx28_bt_pins[] = {
	{
	 .name  = "AUART0.RX",
	 .id       = PINID_AUART0_RX,
	 .fun      = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.TX",
	 .id       = PINID_AUART0_TX,
	 .fun      = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.CTS",
	 .id       = PINID_AUART0_CTS,
	 .fun      = PIN_FUN1,
	 },
	{
	 .name  = "AUART0.RTS",
	 .id       = PINID_AUART0_RTS,
	 .fun      = PIN_FUN1,
	 },
	{
	 .name  = "BT.DISABLE",
	 .id       = PINID_SAIF1_SDATA0,
	 .fun      = PIN_GPIO,
	 .voltage  = PAD_3_3V,
	 .output    = 1,
	 .strength = PAD_4MA,
	 .drive    = 1,
	 .data     = 1,                 /* pull high to enable BT radio*/
	 },
	{
	 .name  = "BT.GPIO7",
	 .id       = PINID_SPDIF,
	 .fun      = PIN_GPIO,
	 .voltage  = PAD_3_3V,
	 .output    = 0,
	 .strength = PAD_4MA,
	 .drive    = 1,
	 },
	{
	 .name  = "BT.PWD.L",
	 .id       = BT_PWD_ID,
	 .fun      = PIN_GPIO,
	 .voltage  = PAD_3_3V,
	 .output    = 1,
	 .strength = PAD_4MA,
	 .drive    = 1,
	 .data     = 0,                 /* start with this pin low, then set hi after 5 ms*/
	 .sysfs    = 0,
	 },
	{
	 .name  = "BT.HOST.WAKE",
	 .id       = PINID_GPMI_CE1N,
	 .fun      = PIN_GPIO,
	 .voltage  = PAD_3_3V,
	 .output    = 0,
	 .strength = PAD_4MA,
	 .drive    = 1,
	 },
	{
	 .name  = "BT.WAKE",
	 .id       = PINID_SSP2_SCK,
	 .fun      = PIN_GPIO,
	 .voltage  = PAD_3_3V,
	 .output    = 1,
	 .strength = PAD_4MA,
	 .drive    = 1,
	 },
};


static int mx28_ccardimx28_bt_pins_init(void)
{
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_bt_pins,
				ARRAY_SIZE(mx28_ccardimx28_bt_pins));

	/* Wait a bit and Power ON Bluetooth */
	msleep(5);
	gpio_set_value(BT_PWD_ID, 1);
	/* Free the BT power pin to allow controlling it from user space */
	gpio_free(MXS_PIN_TO_GPIO(BT_PWD_ID));

	return 0;
}
#else
static int mx28_ccardimx28_bt_pins_init(void)
{
	return 0;
}
#endif

#if defined(CONFIG_MXS_PWM) || defined(CONFIG_MXS_PWM_MODULE)
static struct pin_desc mx28_ccardimx28_pwm_pins[][2] = {
	[0] = {
		{
			.name		= "PWM0",
			.id 		= PINID_AUART1_RX,
			.fun		= PIN_FUN3,
		},
	},
	[1] = {
		{
			.name		= "PWM1",
			.id 		= PINID_AUART1_TX,
			.fun		= PIN_FUN3,
		},
	},
};

void mx28_ccardimx28_pwm_pins_init(void)
{
#if defined(CONFIG_CCARDIMX28_PWM_CH0)
	/* PWM0 */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_pwm_pins[0],
				       ARRAY_SIZE(mx28_ccardimx28_pwm_pins[0]));
#endif

#if defined(CONFIG_CCARDIMX28_PWM_CH1)
	/* PWM1 */
	mx28_ccardimx28_init_pin_group(mx28_ccardimx28_pwm_pins[1],
				       ARRAY_SIZE(mx28_ccardimx28_pwm_pins[1]));
#endif
}
#else
void mx28_ccardimx28_pwm_pins_init(void) {}
#endif /* CONFIG_MXS_PWM || CONFIG_MXS_PWM_MODULE */

void __init mx28_ccardimx28_pins_init(void)
{
	int ret;

	mx28_ccardimx28_duart_pins_init();
	mx28_ccardimx28_auarts_pins_init();
	mx28_ccardimx28_can_pins_init();
	mx28_ccardimx28_i2c_pins_init();
	mx28_ccardimx28_audio_pins_init();
	mx28_ccardimx28_spi_pins_init();
	mx28_ccardimx28_pwm_pins_init();

	if (enable_gpmi) {
		mx28_ccardimx28_init_pin_group(mx28_ccardimx28_gpmi_pins,
						ARRAY_SIZE(mx28_ccardimx28_gpmi_pins));
	}
	mx28_ccardimx28_eth_pins_init();
	mx28_ccardimx28_usb_otg_pins_init();

	ret = mx28_ccardimx28_video_pins_init(CONFIG_CCARDIMX28_DISP_DATABUS_WIDTH);
	if (ret) {
		pr_info(DEV_NAME ": unable to initialize the video pins\n");
	}

	mx28_ccardimx28_mmc_pins_init();
	mx28_ccardimx28_touch_pins_init();
	mx28_ccardimx28_bt_pins_init();
}
