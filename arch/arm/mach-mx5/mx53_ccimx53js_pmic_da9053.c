/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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


/*
 * mx53_ccimx53js_pmic_da9053.c  --  i.MX53 LOCO driver for pmic da9053
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/pm.h>
#include <linux/mfd/da9052/led.h>
#include <linux/mfd/da9052/tsi.h>
#include <mach/irqs.h>
#include <mach/iomux-mx53.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

#include "devices_ccimx5x.h"

#define DA9052_LDO(max, min, rname, suspend_mv) \
{\
	.constraints = {\
		.name		= (rname), \
		.max_uV		= (max) * 1000,\
		.min_uV		= (min) * 1000,\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE\
		|REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,\
		.valid_modes_mask = REGULATOR_MODE_NORMAL,\
		.state_mem = { \
			.uV = suspend_mv * 1000, \
			.mode = REGULATOR_MODE_NORMAL, \
			.enabled = (0 == suspend_mv) ? 0 : 1, \
			.disabled = 0, \
		}, \
	},\
}

/* currently the suspend_mv here takes no effects for DA9053
preset-voltage have to be done in the latest stage during
suspend*/
static struct regulator_init_data da9052_regulators_init[] = {
	DA9052_LDO(DA9052_LDO1_VOLT_UPPER,
		DA9052_LDO1_VOLT_LOWER, "DA9052_LDO1", 1300),
	DA9052_LDO(DA9052_LDO2_VOLT_UPPER,
		DA9052_LDO2_VOLT_LOWER, "DA9052_LDO2", 0),
	DA9052_LDO(DA9052_LDO34_VOLT_UPPER,
		DA9052_LDO34_VOLT_LOWER, "DA9052_LDO3", 3300),
	DA9052_LDO(DA9052_LDO34_VOLT_UPPER,
		DA9052_LDO34_VOLT_LOWER, "DA9052_LDO4", 2775),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO5", 1300),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO6", 3150),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO7", 2775),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO8", 1800),
	DA9052_LDO(DA9052_LDO9_VOLT_UPPER,
		DA9052_LDO9_VOLT_LOWER, "DA9052_LDO9", 2775),
	DA9052_LDO(DA9052_LDO567810_VOLT_UPPER,
		DA9052_LDO567810_VOLT_LOWER, "DA9052_LDO10", 3150),

	/* BUCKS */
	DA9052_LDO(DA9052_BUCK_CORE_PRO_VOLT_UPPER,
		DA9052_BUCK_CORE_PRO_VOLT_LOWER, "DA9052_BUCK_CORE", 1250),
	DA9052_LDO(DA9052_BUCK_CORE_PRO_VOLT_UPPER,
		DA9052_BUCK_CORE_PRO_VOLT_LOWER, "DA9052_BUCK_PRO", 1300),
	DA9052_LDO(DA9052_BUCK_MEM_VOLT_UPPER,
		DA9052_BUCK_MEM_VOLT_LOWER, "DA9052_BUCK_MEM", 1800),
	DA9052_LDO(DA9052_BUCK_PERI_VOLT_UPPER,
		DA9052_BUCK_PERI_VOLT_LOWER, "DA9052_BUCK_PERI", 2500)
};


static struct da9052_tsi_platform_data da9052_tsi = {
	.pen_up_interval = 50,
	.tsi_delay_bit_shift = 6,
	.tsi_skip_bit_shift = 3,
	.num_gpio_tsi_register = 3,
	.tsi_supply_voltage = 2500,
	 /* This is the DA9052 LDO number used for powering the TSI */
	.tsi_ref_source = 9,
	.max_tsi_delay = TSI_DELAY_4SLOTS,
	.max_tsi_skip_slot = TSI_SKIP_330SLOTS,
};

static struct da9052_led_platform_data da9052_gpio_led[] = {
	{
		.id = DA9052_LED_4,
		.name = "LED_GPIO14",
	},
	{
		.id = DA9052_LED_5,
		.name = "LED_GPIO15",
	},
};

static struct da9052_leds_platform_data da9052_gpio_leds = {
	.num_leds = ARRAY_SIZE(da9052_gpio_led),
	.led = da9052_gpio_led,
};


static struct da9052_bat_platform_data da9052_bat = {
	.monitoring_interval = 5000,
	.bat_volt_cutoff = 2800,
	.filter_size = 1,
	.bat_capacity_limit_high = 85,
	.bat_capacity_limit_low = 25,
	.bat_capacity_full = 100,
};


static struct da9052_backlight_data da9052_blit = {
	.current_brightness = 0,
#if defined(CONFIG_DA9052_BL_HAS_LED4)
	.is_led4_present = 1,
#endif
#if defined(CONFIG_DA9052_BL_HAS_LED5)
	.is_led5_present = 1,
#endif
#if defined(CONFIG_DA9052_BL_ACTIVE_HIGH)
	.active_high = 1,
#endif
};

static void da9052_init_ssc_cache(struct da9052 *da9052)
{
	unsigned char cnt;

	/* First initialize all registers as Non-volatile */
	for (cnt = 0; cnt < DA9052_REG_CNT; cnt++) {
		da9052->ssc_cache[cnt].type = NON_VOLATILE;
		da9052->ssc_cache[cnt].status = INVALID;
		da9052->ssc_cache[cnt].val = 0;
	}

	/* Now selectively set type for all Volatile registers */
	/* Reg 1 - 9 */
	da9052->ssc_cache[DA9052_STATUSA_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSB_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSC_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_STATUSD_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTA_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTB_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTC_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_EVENTD_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_FAULTLOG_REG].type = VOLATILE;

	/* Reg 15 */
	da9052->ssc_cache[DA9052_CONTROLB_REG].type = VOLATILE;
	/* Reg - 17 */
	da9052->ssc_cache[DA9052_CONTROLD_REG].type = VOLATILE;
	/* Reg - 60 */
	da9052->ssc_cache[DA9052_SUPPLY_REG].type = VOLATILE;
	/* Reg - 62 */
	da9052->ssc_cache[DA9052_CHGBUCK_REG].type = VOLATILE;

	/* Reg 67 - 68 */
	da9052->ssc_cache[DA9052_INPUTCONT_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_CHGTIME_REG].type = VOLATILE;

	/* Reg - 70 */
	da9052->ssc_cache[DA9052_BOOST_REG].type = VOLATILE;

	/* Reg - 81 */
	da9052->ssc_cache[DA9052_ADCMAN_REG].type = VOLATILE;

	/* Reg - 83 - 85 */
	da9052->ssc_cache[DA9052_ADCRESL_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_ADCRESH_REG].type = VOLATILE;
	da9052->ssc_cache[DA9052_VDDRES_REG].type = VOLATILE;

	/* Reg - 87 */
	da9052->ssc_cache[DA9052_ICHGAV_REG].type = VOLATILE;

	/* Reg - 90 */
	da9052->ssc_cache[DA9052_TBATRES_REG].type = VOLATILE;

	/* Reg - 95 */
	da9052->ssc_cache[DA9052_ADCIN4RES_REG].type = VOLATILE;

	/* Reg - 98 */
	da9052->ssc_cache[DA9052_ADCIN5RES_REG].type = VOLATILE;

	/* Reg - 101 */
	da9052->ssc_cache[DA9052_ADCIN6RES_REG].type = VOLATILE;

	/* Reg - 104 */
	da9052->ssc_cache[DA9052_TJUNCRES_REG].type = VOLATILE;

	/* Reg 106 - 110 */
	da9052->ssc_cache[DA9052_TSICONTB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIXMSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIYMSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSILSB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_TSIZMSB_REG].type	= VOLATILE;

	/* Reg 111 - 117 */
	da9052->ssc_cache[DA9052_COUNTS_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTMI_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTH_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTD_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTMO_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_COUNTY_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_ALARMMI_REG].type	= VOLATILE;

	/* Reg 122 - 125 */
	da9052->ssc_cache[DA9052_SECONDA_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDB_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDC_REG].type	= VOLATILE;
	da9052->ssc_cache[DA9052_SECONDD_REG].type	= VOLATILE;

	/* Following addresses are not assigned to any register */
	da9052->ssc_cache[126].type			= VOLATILE;
	da9052->ssc_cache[127].type			= VOLATILE;
}



static int __init ccimx53js_da9052_init(struct da9052 *da9052)
{
	/* Configuring for DA905x interrupt line */
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_16__GPIO7_11);
	gpio_request(CCIMX53_DA9052_IRQ_GPIO, "pmic_irq");
	gpio_direction_input(CCIMX53_DA9052_IRQ_GPIO);

	/* Set interrupt as LOW LEVEL interrupt source */
	set_irq_type(gpio_to_irq(CCIMX53_DA9052_IRQ_GPIO), IRQF_TRIGGER_LOW);

	da9052_init_ssc_cache(da9052);

	return 0;
}

static struct da9052_platform_data __initdata da9052_plat = {
	.init = ccimx53js_da9052_init,
	.num_regulators = ARRAY_SIZE(da9052_regulators_init),
	.regulators = da9052_regulators_init,
	.led_data = &da9052_gpio_leds,
	.tsi_data = &da9052_tsi,
	.bat_data = &da9052_bat,
	.gpio_base = GPIO_PMIC_START,
	.backlight_data = &da9052_blit,
};

static struct i2c_board_info __initdata da9052_i2c_device = {
	I2C_BOARD_INFO(DA9052_SSC_I2C_DEVICE_NAME, DA9052_I2C_ADDR >> 1),
	.irq = gpio_to_irq(CCIMX53_DA9052_IRQ_GPIO),
	.platform_data = &da9052_plat,
};

int __init mx53_ccimx53js_init_da9052(void)
{
	u8 modrev = ccimx5x_get_mod_revision();

	/* Early variants use a different (0x48) address for the PMIC */
	if (modrev == 0x01)
		da9052_i2c_device.addr = DA9052_I2C_ADDR_ALT >> 1;

	return i2c_register_board_info(2, &da9052_i2c_device, 1);
}
