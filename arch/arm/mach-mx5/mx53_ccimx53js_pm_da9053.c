/*
 * Copyright (C) 2012 Digi International Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/da9052.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/i2c.h>

#define IRQMASKA_MASKS		~(DA9052_IRQMASKA_MALRAM)
#define IRQMASKB_MASKS		~(DA9052_IRQMASKB_MNONKEY)
#define IRQMASKC_MASKS		0xff
#define IRQMASKD_MASKS		0xff

static u8 irq_masks[DA9052_IRQMASKD_REG - DA9052_IRQMASKA_REG + 1];
extern int pm_i2c_imx_xfer(struct i2c_msg *msgs, int num);
static int i2caddr = DA9052_I2C_ADDR >> 1;

static void pm_da9053_read_reg(u8 reg, u8 *value)
{
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	buf[0] = reg;
	i2cmsg[0].addr  = i2caddr ;
	i2cmsg[0].len   = 1;
	i2cmsg[0].buf   = &buf[0];

	i2cmsg[0].flags = 0;

	i2cmsg[1].addr  = i2caddr ;
	i2cmsg[1].len   = 1;
	i2cmsg[1].buf   = &buf[1];

	i2cmsg[1].flags = I2C_M_RD;

	pm_i2c_imx_xfer(i2cmsg, 2);
	*value = buf[1];
}

static void pm_da9053_write_reg(u8 reg, u8 value)
{
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	buf[0] = reg;
	buf[1] = value;
	i2cmsg[0].addr  = i2caddr ;
	i2cmsg[0].len   = 2;
	i2cmsg[0].buf   = &buf[0];
	i2cmsg[0].flags = 0;
	pm_i2c_imx_xfer(i2cmsg, 1);
}


#if 0
static void pm_da9053_dump(int start, int end)
{
	u8 reg, data;
	for (reg = start; reg <= end; reg++) {
		pm_da9053_read_reg(reg, &data);
		pr_info("reg %u = 0x%2x\n",
			reg, data);
	}
}
#endif

int ccimx53_pm_da9053_mask_irqs(void)
{
	u8 reg, data;
	struct clk *i2c_clk;
	u8 irq_newmasks[4] = {
		IRQMASKA_MASKS, IRQMASKB_MASKS,
		IRQMASKC_MASKS, IRQMASKD_MASKS
	};

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	/* Save irq masks, and mask all irqs except nONKEY and RTC alarm*/
	for (reg = DA9052_IRQMASKA_REG;
		reg <= DA9052_IRQMASKD_REG; reg++) {
		pm_da9053_read_reg(reg, &data);
		irq_masks[reg - DA9052_IRQMASKA_REG] = data;
		pm_da9053_write_reg(reg, irq_newmasks[reg - DA9052_IRQMASKA_REG]);
	}

	/* Park */
	pm_da9053_write_reg(0xff, 0xff);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);

	return 0;
}

int ccimx53_pm_da9053_unmask_irqs(void)
{
	u8 reg;
	struct clk *i2c_clk;

	i2c_clk = clk_get(NULL, "i2c_clk");
	if (IS_ERR(i2c_clk)) {
		pr_err("unable to get i2c clk\n");
		return PTR_ERR(i2c_clk);
	}
	clk_enable(i2c_clk);

	/* Save irq masks, and mask all irqs except nONKEY */
	for (reg = DA9052_IRQMASKA_REG;
		reg <= DA9052_IRQMASKD_REG; reg++) {
		pm_da9053_write_reg(reg, irq_masks[reg - DA9052_IRQMASKA_REG]);
	}

	/* Park */
	pm_da9053_write_reg(0xff, 0xff);

	clk_disable(i2c_clk);
	clk_put(i2c_clk);

	return 0;
}
