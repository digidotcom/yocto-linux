/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/smsc911x.h>
#include <linux/sysfs.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#if defined(CONFIG_MODULE_CCIMX53)
#include <mach/iomux-mx53.h>
#endif

#include "devices_ccimx5x.h"

static u8 ccimx5x_mod_variant = 0;
static u8 ccimx5x_mod_rev = 0;
static u32 ccimx5x_mod_sn = 0;
static u8 ccimx5x_bb_rev = BASE_BOARD_REV;

void ccimx5x_set_mod_variant(u8 variant)
{
	ccimx5x_mod_variant = variant;
}
u8 ccimx5x_get_mod_variant(void)
{
	return ccimx5x_mod_variant;
}
void ccimx5x_set_mod_revision(u8 revision)
{
	ccimx5x_mod_rev = revision;
}
u8 ccimx5x_get_mod_revision(void)
{
	return ccimx5x_mod_rev;
}
void ccimx5x_set_mod_sn(u32 sn)
{
	ccimx5x_mod_sn = sn;
}

#ifdef CONFIG_SYSFS
static ssize_t ccimx5x_mod_variant_attr_show(struct kobject *kobj,
					     struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccimx5x_mod_variant);
}

static ssize_t ccimx5x_mod_rev_attr_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccimx5x_mod_rev);
}

static ssize_t ccimx5x_mod_sn_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccimx5x_mod_sn);
}

static ssize_t ccimx5x_bb_rev_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccimx5x_bb_rev);
}

static ssize_t ccimx5x_machinename_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CCIMX5X_SYSFS_FNAME);
}

static struct kobj_attribute ccimx5x_mod_variant_attr =
	__ATTR(mod_variant, S_IRUGO, ccimx5x_mod_variant_attr_show, NULL);
static struct kobj_attribute ccimx5x_mod_rev_attr =
	__ATTR(mod_rev, S_IRUGO, ccimx5x_mod_rev_attr_show, NULL);
static struct kobj_attribute ccimx5x_mod_sn_attr =
	__ATTR(mod_sn, S_IRUGO, ccimx5x_mod_sn_attr_show, NULL);
static struct kobj_attribute ccimx5x_bb_rev_attr =
	__ATTR(bb_rev, S_IRUGO, ccimx5x_bb_rev_attr_show, NULL);
static struct kobj_attribute ccimx5x_machinename_attr =
	__ATTR(name, S_IRUGO, ccimx5x_machinename_attr_show, NULL);

int ccimx5x_create_sysfs_entries(void)
{
	struct kobject *ccimx5x_kobj;
	int ret;

	ccimx5x_kobj = kobject_create_and_add(CCIMX5X_SYSFS_FNAME, kernel_kobj);
	if (!ccimx5x_kobj) {
		printk(KERN_WARNING "kobject_create_and_add %s failed\n",
		       CCIMX5X_SYSFS_FNAME);
		return -EINVAL;
	}

	ret = sysfs_create_file(ccimx5x_kobj, &ccimx5x_mod_variant_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware variant\n",
		       CCIMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccimx5x_kobj, &ccimx5x_mod_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware revision\n",
		       CCIMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccimx5x_kobj, &ccimx5x_mod_sn_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware SN\n",
		       CCIMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccimx5x_kobj, &ccimx5x_bb_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s base board hardware revision\n",
		       CCIMX5X_SYSFS_FNAME);
		return ret;
	}

	ccimx5x_kobj = kobject_create_and_add(CCIMX5X_SYSFS_MNAME, kernel_kobj);
	if (!ccimx5x_kobj) {
		printk(KERN_WARNING "kobject_create_and_add %s failed\n",
				CCIMX5X_SYSFS_MNAME);
		return -EINVAL;
	}

	ret = sysfs_create_file(ccimx5x_kobj, &ccimx5x_machinename_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for machine\n");
		return ret;
	}

	return 0;
}
#else
int ccimx5x_create_sysfs_entries(void) { return 0; }
#endif /* CONFIG_SYSFS */
