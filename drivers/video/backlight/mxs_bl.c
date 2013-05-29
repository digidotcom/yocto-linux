/*
 * Backlight Driver for Freescale MXS
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <mach/lcdif.h>
#include <mach/regulator.h>
#include <mach/system.h>
#include <mach/regs-pwm.h>

/* PWM clock divisor from 24MHz */
#define BL_CLK_DIV	400	/* 60KHz */

struct mxs_bl_data {
	struct notifier_block nb;
	struct notifier_block reg_nb;
	struct notifier_block reg_init_nb;
	struct backlight_device *bd;
	struct mxs_platform_bl_data *pdata;
	int current_intensity;
	int saved_intensity;
	int mxsbl_suspended;
	int mxsbl_constrained;
};

static struct clk *pwm_clk;

static int mxsbl_do_probe(struct mxs_bl_data *data,
		struct mxs_platform_bl_data *pdata);
static int mxsbl_set_intensity(struct backlight_device *bd);

static int bl_setup(struct mxs_platform_bl_data *pdata)
{
	int ret = 0;

	pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(pwm_clk)) {
		ret = PTR_ERR(pwm_clk);
		return ret;
	}
	clk_enable(pwm_clk);

	mxs_reset_block(REGS_PWM_BASE, 1);

	return 0;
}

#ifdef CONFIG_PM
static int mxsbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;

	data->mxsbl_suspended = 1;
	if (pdata) {
		dev_dbg(&pdev->dev, "real suspend\n");
		mxsbl_set_intensity(data->bd);
	}
	return 0;
}

static int mxsbl_resume(struct platform_device *pdev)
{
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;

	data->mxsbl_suspended = 0;
	if (pdata) {
		dev_dbg(&pdev->dev, "real resume\n");
		mxsbl_set_intensity(data->bd);
	}

	return 0;
}
#else
#define mxsbl_suspend	NULL
#define mxsbl_resume	NULL
#endif
/*
 *  This function should be called with bd->ops_lock held
 *  Suspend/resume ?
 */
static int mxsbl_set_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;
	u32 inact_state;

	if (pdata) {
		int intensity = bd->props.brightness;
		int scaled_int;

		if (bd->props.power != FB_BLANK_UNBLANK)
			intensity = 0;
		if (bd->props.fb_blank != FB_BLANK_UNBLANK)
			intensity = 0;
		if (data->mxsbl_suspended)
			intensity = 0;

		/* The intensity is then scaled to the PWM period */
		scaled_int = intensity * BL_CLK_DIV / pdata->bl_max_intensity;

		__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
			     BF_PWM_ACTIVEn_ACTIVE(0),
			     REGS_PWM_BASE + HW_PWM_ACTIVEn(pdata->pwm));

		/* PWM polarity may be inverted via platform data */
		if (pdata->inverted)
			/* Active low, inactive high */
			inact_state = BF_PWM_PERIODn_INACTIVE_STATE(3) |
				      BF_PWM_PERIODn_ACTIVE_STATE(2);
		else
			/* Active high, inactive low */
			inact_state = BF_PWM_PERIODn_INACTIVE_STATE(2) |
				      BF_PWM_PERIODn_ACTIVE_STATE(3);
		__raw_writel(inact_state | BF_PWM_PERIODn_CDIV(0) |
			     BF_PWM_PERIODn_PERIOD((BL_CLK_DIV - 1)),
			     REGS_PWM_BASE + HW_PWM_PERIODn(pdata->pwm));

		data->current_intensity = bd->props.brightness;
		return 0;
	} else
		return -ENODEV;
}

static int mxsbl_get_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct mxs_bl_data *data = platform_get_drvdata(pdev);

	return data->current_intensity;
}

static struct backlight_ops mxsbl_ops = {
	.get_brightness = mxsbl_get_intensity,
	.update_status  = mxsbl_set_intensity,
};

static int mxsbl_do_probe(struct mxs_bl_data *data,
		struct mxs_platform_bl_data *pdata)
{
	int ret = bl_setup(pdata);

	if (ret)
		goto out;

	data->bd->props.power = FB_BLANK_UNBLANK;
	data->bd->props.fb_blank = FB_BLANK_UNBLANK;

	if (!data->mxsbl_suspended) {
		if (data->mxsbl_constrained) {
			data->bd->props.max_brightness = pdata->bl_cons_intensity;
			data->bd->props.brightness = pdata->bl_cons_intensity;
		} else {
			data->bd->props.max_brightness = pdata->bl_max_intensity;
			data->bd->props.brightness = pdata->bl_default_intensity;
		}
	}

	data->pdata = pdata;
	mxsbl_set_intensity(data->bd);
	/* Enable PWM */
	__raw_writel((1 << pdata->pwm), REGS_PWM_BASE + HW_PWM_CTRL_SET);

out:
	return ret;
}

static int __devinit mxsbl_probe(struct platform_device *pdev)
{
	struct mxs_bl_data *data;
	struct mxs_platform_bl_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto out;
	}
	data->bd = backlight_device_register(pdev->name, &pdev->dev, pdev,
					&mxsbl_ops, NULL);
	if (IS_ERR(data->bd)) {
		ret = PTR_ERR(data->bd);
		goto out_1;
	}

	get_device(&pdev->dev);

	platform_set_drvdata(pdev, data);

	if (pdata) {
		ret = mxsbl_do_probe(data, pdata);
		if (ret)
			goto out_2;
	}

	goto out;

out_2:
	put_device(&pdev->dev);
out_1:
	kfree(data);
out:
	return ret;
}

static int mxsbl_remove(struct platform_device *pdev)
{
	struct mxs_platform_bl_data *pdata = pdev->dev.platform_data;
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct backlight_device *bd = data->bd;

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.fb_blank = FB_BLANK_POWERDOWN;
	bd->props.brightness = 0;
	data->current_intensity = bd->props.brightness;

	if (pdata) {
		/* Disable the selected PWM channel */
		__raw_writel((1 << pdata->pwm), REGS_PWM_BASE + HW_PWM_CTRL_CLR);

		clk_disable(pwm_clk);
		clk_put(pwm_clk);
	}

	backlight_device_unregister(bd);
	if (pdata->regulator)
		regulator_put(pdata->regulator);
	put_device(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	mxs_lcdif_unregister_client(&data->nb);
	kfree(data);

	return 0;
}

static struct platform_driver mxsbl_driver = {
	.probe		= mxsbl_probe,
	.remove		= __devexit_p(mxsbl_remove),
	.suspend	= mxsbl_suspend,
	.resume		= mxsbl_resume,
	.driver		= {
		.name	= "mxs-bl",
		.owner	= THIS_MODULE,
	},
};

static int __init mxs_init(void)
{
	return platform_driver_register(&mxsbl_driver);
}

static void __exit mxs_exit(void)
{
	platform_driver_unregister(&mxsbl_driver);
}

module_init(mxs_init);
module_exit(mxs_exit);

MODULE_AUTHOR("Embedded Alley Solutions, Inc <sources@embeddedalley.com>");
MODULE_DESCRIPTION("MXS Backlight Driver");
MODULE_LICENSE("GPL");
