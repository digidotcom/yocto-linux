/*
 * Copyright 2012 Digi International, Inc. All Rights Reserved.
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
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include "mx28_ccardimx28.h"

static u8		hwid_tf = 0;		/* location */
static u8		hwid_variant = 0;	/* module variant */
static unsigned char	hwid_hv = 0;		/* hardware version */
static unsigned char	hwid_cert = 0;		/* type of wifi certification */
static u8		hwid_year = 0;		/* manufacturing year */
static unsigned char	hwid_month = 0;		/* manufacutring month */
static u32		hwid_sn = 0;		/* serial number */

static int hwid_available = 0;

void ccardimx28_set_hwid(u32 low, u32 high)
{
	u32 cust0, cust1;

	/* HWID in serial ATAG was stored by U-Boot in BIG ENDIAN.
	 * Swap endiannes */
	cust0 = ((low & 0x000000FFUL) << 24) |
                ((low & 0x0000FF00UL) <<  8) |
                ((low & 0x00FF0000UL) >>  8) |
                ((low & 0xFF000000UL) >> 24);
	cust1 = ((high & 0x000000FFUL) << 24) |
                ((high & 0x0000FF00UL) <<  8) |
                ((high & 0x00FF0000UL) >>  8) |
                ((high & 0xFF000000UL) >> 24);

	hwid_tf = (cust0 >> 16) & 0xff;
	hwid_variant = (cust0 >> 8) & 0xff;
	hwid_hv = (cust0 >> 4) & 0xf;
	hwid_cert = cust0 & 0xf;
	hwid_year = (cust1 >> 24) & 0xff;
	hwid_month = (cust1 >> 20) & 0xf;
	hwid_sn = cust1 & 0xfffff;

	hwid_available = 1;
}

unsigned char get_hwid_hv(void)
{
	if (hwid_available)
		return hwid_hv;
	return 0;
}

#ifdef CONFIG_SYSFS

static ssize_t hwid_tf_attr_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", hwid_tf);
}

static ssize_t hwid_variant_attr_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", hwid_variant);
}

static ssize_t hwid_hv_attr_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", hwid_hv);
}

static ssize_t hwid_cert_attr_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", hwid_cert);
}

static ssize_t hwid_year_attr_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "20%02d\n", hwid_year);
}

static ssize_t hwid_month_attr_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%02d\n", hwid_month);
}

static ssize_t hwid_sn_attr_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", hwid_sn);
}

static ssize_t hwid_machinename_attr_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CCARDIMX28_SYSFS_FNAME);
}

static struct kobj_attribute hwid_tf_attr =
	__ATTR(mod_tf, S_IRUGO, hwid_tf_attr_show, NULL);
static struct kobj_attribute hwid_variant_attr =
	__ATTR(mod_variant, S_IRUGO, hwid_variant_attr_show, NULL);
static struct kobj_attribute hwid_hv_attr =
	__ATTR(mod_ver, S_IRUGO, hwid_hv_attr_show, NULL);
static struct kobj_attribute hwid_cert_attr =
	__ATTR(mod_cert, S_IRUGO, hwid_cert_attr_show, NULL);
static struct kobj_attribute hwid_year_attr =
	__ATTR(mod_year, S_IRUGO, hwid_year_attr_show, NULL);
static struct kobj_attribute hwid_month_attr =
	__ATTR(mod_month, S_IRUGO, hwid_month_attr_show, NULL);
static struct kobj_attribute hwid_sn_attr =
	__ATTR(mod_sn, S_IRUGO, hwid_sn_attr_show, NULL);
static struct kobj_attribute hwid_machinename_attr =
	__ATTR(name, S_IRUGO, hwid_machinename_attr_show, NULL);

int ccardimx28_create_sysfs_entries(void)
{
	struct kobject *ccardimx28_kobj;
	int ret;

	ccardimx28_kobj = kobject_create_and_add(CCARDIMX28_SYSFS_FNAME, kernel_kobj);
	if (!ccardimx28_kobj) {
		printk(KERN_WARNING "kobject_create_and_add %s failed\n",
		       CCARDIMX28_SYSFS_FNAME);
		return -EINVAL;
	}

	ret = sysfs_create_file(ccardimx28_kobj, &hwid_tf_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s TF (location)\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_variant_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s variant\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_hv_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware version\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_cert_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s cert\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_year_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s year\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_month_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s month\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccardimx28_kobj, &hwid_sn_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s serial number\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}

	ccardimx28_kobj = kobject_create_and_add(CCARDIMX28_SYSFS_MNAME, kernel_kobj);
	if (!ccardimx28_kobj) {
		printk(KERN_WARNING "kobject_create_and_add %s failed\n",
				CCARDIMX28_SYSFS_MNAME);
		return -EINVAL;
	}

	ret = sysfs_create_file(ccardimx28_kobj, &hwid_machinename_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s.\n",
		       CCARDIMX28_SYSFS_FNAME);
		return ret;
	}

	return 0;
}
#else
int ccardimx28_create_sysfs_entries(void) { return 0; }
#endif /* CONFIG_SYSFS */
