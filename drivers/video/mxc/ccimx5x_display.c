/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved
 * Copyright 2010 - 2011 Digi International, Inc. All Rights Reserved
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <mach/hardware.h>
#include <mach/mxc.h>
#include <linux/fsl_devices.h>

#define MAX_DISPLAYS	2
#define DISP0_ID	"DISP3 BG"
#define DISP1_ID	"DISP3 BG - DI1"

static void lcd_poweron(struct ccimx5x_lcd_pdata *plat);
static void lcd_poweroff(struct ccimx5x_lcd_pdata *plat);

static struct platform_device *plcd_dev[MAX_DISPLAYS] = {NULL, NULL};

static int lcd_get_index(struct fb_info *info)
{
	if (!strcmp(info->fix.id, DISP0_ID))
		return 0;
	else if (!strcmp(info->fix.id, DISP1_ID))
		return 1;
	return -1;
}

static void lcd_init_fb(struct fb_info *info)
{
	struct fb_var_screeninfo var;
	struct ccimx5x_lcd_pdata *plat;
	int i = lcd_get_index(info);

	if (i < 0)
		return;

	plat = plcd_dev[i]->dev.platform_data;
	memset(&var, 0, sizeof(var));
	fb_videomode_to_var(&var, plat->fb_pdata.mode);
	var.activate = FB_ACTIVATE_ALL;
	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
}

static int lcd_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct ccimx5x_lcd_pdata *plat = event->info->dev->platform_data;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		lcd_init_fb(event->info);
		lcd_poweron(plat);
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			lcd_poweron(plat);
		} else {
			lcd_poweroff(plat);
		}
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lcd_fb_event,
};

static int __devinit lcd_sync_probe(struct platform_device *pdev)
{
	struct ccimx5x_lcd_pdata *plat = pdev->dev.platform_data;

	if (!plat)
		return -ENODEV;

	if (plat->vif < 0 || plat->vif > (MAX_DISPLAYS - 1))
		return -EINVAL;

	if (plat->init)
		plat->init(plat->vif);

	plcd_dev[plat->vif] = pdev;

	/**
	 * Register the block notifier only once. The device being notified can be
	 * retrieved from the received event. There are some issues when the same
	 * notifier is registered multiple times.
	 */
	if (plcd_dev[0] == NULL && plcd_dev[1] == NULL)
		fb_register_client(&nb);
	lcd_poweron(plat);

	return 0;
}

static int __devexit lcd_sync_remove(struct platform_device *pdev)
{
	struct ccimx5x_lcd_pdata *plat = pdev->dev.platform_data;

	lcd_poweroff(plat);
	if (plat->deinit)
		plat->deinit(plat->vif);

	plcd_dev[plat->vif] = NULL;

	if (plcd_dev[0] == NULL && plcd_dev[1] == NULL)
		fb_unregister_client(&nb);

	return 0;
}

#ifdef CONFIG_PM
static int lcd_sync_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int lcd_sync_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lcd_sync_suspend NULL
#define lcd_sync_resume NULL
#endif

static struct platform_driver lcd_driver = {
	.driver = {
		   .name = "ccimx51_display"
	},
	.probe = lcd_sync_probe,
	.remove = __devexit_p(lcd_sync_remove),
	.suspend = lcd_sync_suspend,
	.resume = lcd_sync_resume,
};

static void lcd_poweron(struct ccimx5x_lcd_pdata *plat)
{
        if (plat && plat->bl_enable)
                plat->bl_enable(1, plat->vif);
}

static void lcd_poweroff(struct ccimx5x_lcd_pdata *plat)
{
        if (plat && plat->bl_enable)
                plat->bl_enable(0, plat->vif);
}

static int __init lcd_sync_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit lcd_sync_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(lcd_sync_init);
module_exit(lcd_sync_exit);

MODULE_AUTHOR("Digi International, Inc.");
MODULE_DESCRIPTION("ConnectCore Wi-i.MX51 LCD init driver");
MODULE_LICENSE("GPL");
