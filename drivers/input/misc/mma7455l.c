/* Linux kernel driver for the Freescale MMA7455L 3-axis accelerometer
 *
 * Copyright (C) 2009 by Always Innovating, Inc.
 * Copyright 2009 Digi International, Inc. All Rights Reserved.
 * Author: Gregoire Gentil <gregoire@gentil.com>
 * Author: Tim Yamin <plasm@roo.me.uk>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

/*
 * What this driver doesn't yet support:
 *
 * - INT2 handling
 * - Pulse detection (and the sysctls to control it)
 * - 10-bit measurement
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <linux/mma7455l.h>
#include <linux/i2c.h>

#include "../input-polldev.c"

#define MMA7455L_WHOAMI_MAGIC		0x55
#define POLL_INTERVAL			100
#define POLL_INTERVAL_MIN		1
#define POLL_INTERVAL_MAX		200
#define DEVICE_NAME			"mma7455l"

#define INPUT_FUZZ  8
#define INPUT_FLAT  2

enum mma7455l_reg {
	MMA7455L_REG_XOUTL		= 0x00,
	MMA7455L_REG_XOUTH		= 0x01,
	MMA7455L_REG_YOUTL		= 0x02,
	MMA7455L_REG_YOUTH		= 0x03,
	MMA7455L_REG_ZOUTL 		= 0x04,
	MMA7455L_REG_ZOUTH		= 0x05,
	MMA7455L_REG_XOUT8		= 0x06,
	MMA7455L_REG_YOUT8		= 0x07,
	MMA7455L_REG_ZOUT8		= 0x08,
	MMA7455L_REG_STATUS		= 0x09,
	MMA7455L_REG_DETSRC		= 0x0a,
	MMA7455L_REG_TOUT		= 0x0b,
	MMA7455L_REG_RESERVED1		= 0x0c,
	MMA7455L_REG_I2CAD		= 0x0d,
	MMA7455L_REG_USRINF		= 0x0e,
	MMA7455L_REG_WHOAMI		= 0x0f,
	MMA7455L_REG_XOFFL		= 0x10,
	MMA7455L_REG_XOFFH		= 0x11,
	MMA7455L_REG_YOFFL		= 0x12,
	MMA7455L_REG_YOFFH		= 0x13,
	MMA7455L_REG_ZOFFL		= 0x14,
	MMA7455L_REG_ZOFFH		= 0x15,
	MMA7455L_REG_MCTL		= 0x16,
	MMA7455L_REG_INTRST		= 0x17,
	MMA7455L_REG_CTL1		= 0x18,
	MMA7455L_REG_CTL2		= 0x19,
	MMA7455L_REG_LDTH		= 0x1a,
	MMA7455L_REG_PDTH		= 0x1b,
	MMA7455L_REG_PW			= 0x1c,
	MMA7455L_REG_LT			= 0x1d,
	MMA7455L_REG_TW			= 0x1e,
	MMA7455L_REG_RESERVED2		= 0x1f,
};

enum mma7455l_reg_status {
	MMA7455L_STATUS_XDA		= 0x08,
	MMA7455L_STATUS_YDA		= 0x10,
	MMA7455L_STATUS_ZDA		= 0x20,
};

enum mma7455l_mode {
	/* +HP: Normally we would want Data Ready Status to be
	 * signaled to INT1/DRDY PIN only in measurement mode, but
	 * that would cause continued interrupts that we are not
	 * going to handle for measurement (we would probably need
	 * a DMA buffer for that), so do not output
	 * Data Ready to INT1/DRDY PIN in any mode */
	MMA7455L_MODE_STANDBY		= 0x40,
	MMA7455L_MODE_MEASUREMENT	= 0x41,
	MMA7455L_MODE_LEVELDETECTION	= 0x42,
	MMA7455L_MODE_PULSEDETECTION	= 0x43,
	MMA7455L_MODE_MASK		= 0x43,
};

enum mma7455l_gselect {
	MMA7455L_GSELECT_8		= 0x0,
	MMA7455L_GSELECT_2		= 0x4,
	MMA7455L_GSELECT_4		= 0x8,
	MMA7455L_GSELECT_MASK		= 0xC,
};

/* FIXME */
#define MMA7455L_F_FS			0x0020 	/* ADC full scale */

static struct mma7455l_info *mma_global;

struct mma7455l_info {
	/* Calibration data */
	s16 calibration_x;
	s16 calibration_y;
	s16 calibration_z;

	/* Defaults to I2c access */
	struct i2c_client *client;
	struct input_polled_dev *input_polled_dev;
	struct mutex lock;		/* Struct mutex lock */
	struct delayed_work work;

	u8 mode;
	u8 gSelect;

	u8 flags;
	u8 working;
};

/* lowlevel register access functions */

#define WRITE_BIT	(1 << 7)
#define ADDR_SHIFT	1


#if !defined(CONFIG_MODULE_CCIMX5X)
/* Defaults to I2c access */
static inline u_int8_t __reg_read(struct mma7455l_info *mma, u_int8_t reg)
{
	unsigned char buf;
	int ret;

	ret = i2c_master_send(mma->client, &reg, 1);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(mma->client, &buf, 1);
	if (ret < 0)
		return ret;
	return buf;
}
#else
static inline u_int8_t __reg_read(struct mma7455l_info *mma, u_int8_t reg)
{
	unsigned char buf[2] = {0, 0};
	struct i2c_msg i2cmsg[2];
	int ret = 0;

	buf[0] = reg;

	/* Write the register to access */
	i2cmsg[0].addr  =  mma->client->addr;
	i2cmsg[0].len   = 1;
	i2cmsg[0].buf   = &buf[0];
	i2cmsg[0].flags = 0;

	/* Read the data */
	i2cmsg[1].addr  = mma->client->addr;
	i2cmsg[1].len   = 1;
	i2cmsg[1].buf   = &buf[1];
	i2cmsg[1].flags = I2C_M_RD;

	ret = i2c_transfer(mma->client->adapter, i2cmsg, 2);

	if (ret < 0) {
		dev_dbg(&mma->client->dev,"%s:master_xfer Failed!!\n", __func__);
		return -EIO;
	}

	return buf[1];
}
#endif

static inline int __reg_write(struct mma7455l_info *mma,
				u_int8_t reg, u_int8_t val)
{

	u8 data[2];

	data[0] = reg;
	data[1] = val;

	if( i2c_master_send(mma->client, data, 2) < 0 ) {
		dev_dbg(&mma->client->dev, "%s: write error\n", __func__);
		return -EIO;
	}
	return 0;
}

static u_int8_t reg_read(struct mma7455l_info *mma, u_int8_t reg)
{
	u_int8_t ret;

	mutex_lock(&mma->lock);
	ret = __reg_read(mma, reg);
	mutex_unlock(&mma->lock);

	return ret;
}

static s16 __reg_read_10(struct mma7455l_info *mma, u8 reg1, u8 reg2)
{
	u8 v1, v2;
	s16 ret;

	v1 = __reg_read(mma, reg1);
	v2 = __reg_read(mma, reg2);

	// ret is 11bits, 3 MSB from v2 and 8 LSB from v1.
	ret = (((v2 & 0x7) << 8 | v1) & 0x7FF) << 5;
	// Sign extension. Being ret signed, this will fill the MSBs with
	// either 0s o 1s depending on whether bit10 is 0 or 1.
	return (ret >>= 5);
}

static int reg_write(struct mma7455l_info *mma, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&mma->lock);
	ret = __reg_write(mma, reg, val);
	mutex_unlock(&mma->lock);

	return ret;
}

static s16 __reg_write_10(struct mma7455l_info *mma,
				u8 reg1, u8 reg2, s16 value)
{
	int ret;
	u8 v1, v2;

	v1 = value & 0xFF;
	if (value < 0)
		v2 = ((value >> 8) & 0x3) | 0x4;
	else
		v2 = 0;

	ret = __reg_write(mma, reg1, v1);
	ret = __reg_write(mma, reg2, v2);
	return ret;
}

static void mma7455l_work(struct work_struct *work)
{
	struct mma7455l_info *mma =
			container_of(work, struct mma7455l_info, work.work);

	s8 val;
	mma->working = 1;

	/* FIXME: 10 bit accuracy? */
	if (!(mma->flags & MMA7455L_STATUS_XDA)) {
		val = reg_read(mma, MMA7455L_REG_XOUT8);
		input_report_abs(mma->input_polled_dev->input, ABS_X, val);
	}
	if (!(mma->flags & MMA7455L_STATUS_YDA)) {
		val = reg_read(mma, MMA7455L_REG_YOUT8);
		input_report_abs(mma->input_polled_dev->input, ABS_Y, val);
	}
	if (!(mma->flags & MMA7455L_STATUS_ZDA)) {
		val = reg_read(mma, MMA7455L_REG_ZOUT8);
		input_report_abs(mma->input_polled_dev->input, ABS_Z, val);
	}

	mma->working = 0;
	input_sync(mma->input_polled_dev->input);
	put_device(&mma->client->dev);

	/* Enable IRQ and clear out interrupt */
	reg_write(mma, MMA7455L_REG_INTRST, 0x3);
	reg_write(mma, MMA7455L_REG_INTRST, 0x0);

	enable_irq(mma->client->irq);
}

static void mma7455l_schedule_work(struct mma7455l_info *mma)
{
	int status;

	get_device(&mma->client->dev);
	status = schedule_delayed_work(&mma->work, HZ / 10);
}

static irqreturn_t mma7455l_interrupt(int irq, void *_mma)
{
	struct mma7455l_info *mma = _mma;
	mma7455l_schedule_work(mma);

	/* Disable any further interrupts until we have processed
	 * the current one */
	disable_irq(mma->client->irq);
	return IRQ_HANDLED;
}

/* sysfs */

static void get_mode(struct mma7455l_info *mma, u8 *mode, u8 *gSelect)
{
	u8 tmp = reg_read(mma, MMA7455L_REG_MCTL);

	*mode = tmp & MMA7455L_MODE_MASK;
	*gSelect = tmp & MMA7455L_GSELECT_MASK;
}

static void set_mode(struct mma7455l_info *mma, u8 mode, u8 gSelect)
{
	reg_write(mma, MMA7455L_REG_MCTL, mode | gSelect);
}

static void update_mode(struct mma7455l_info *mma, u8 mode, u8 gSelect)
{
	mma->mode = mode;
	mma->gSelect = gSelect;

	reg_write(mma, MMA7455L_REG_MCTL, mma->mode | mma->gSelect);
}

static ssize_t show_measure(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mma7455l_info *mma = dev_get_drvdata(dev);
	s8 x, y, z;
	u8 old_Mode, old_gSelect;

	get_mode(mma, &old_Mode, &old_gSelect);
	set_mode(mma, MMA7455L_MODE_MEASUREMENT, MMA7455L_GSELECT_2);

	while (reg_read(mma, MMA7455L_REG_STATUS) == 0)
		msleep(10);

	x = reg_read(mma, MMA7455L_REG_XOUT8);
	y = reg_read(mma, MMA7455L_REG_YOUT8);
	z = reg_read(mma, MMA7455L_REG_ZOUT8);

// Sign inversion to compensate the accelerometer position in the board.
#if defined(CONFIG_MODULE_CCIMX51)
	x = -(x);
	y = -(y);
#endif

#if defined(CONFIG_MODULE_CCIMX5X)
	msleep(10);
#endif

	set_mode(mma, old_Mode, old_gSelect);
	return sprintf(buf, "%d %d %d\n", x, y, z);
}

static ssize_t show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	switch (mma->mode) {
	case MMA7455L_MODE_STANDBY:
		return sprintf(buf, "Standby\n");
		break;
	case MMA7455L_MODE_MEASUREMENT:
		return sprintf(buf, "Measurement\n");
		break;
	case MMA7455L_MODE_LEVELDETECTION:
		return sprintf(buf, "Level Detection\n");
		break;
	case MMA7455L_MODE_PULSEDETECTION:
		return sprintf(buf, "Pulse Detection\n");
		break;
	}

	return sprintf(buf, "Unknown mode!\n");
}

static ssize_t show_gSelect(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	switch (mma->gSelect) {
	case MMA7455L_GSELECT_8:
		return sprintf(buf, "8\n");
		break;
	case MMA7455L_GSELECT_4:
		return sprintf(buf, "4\n");
		break;
	case MMA7455L_GSELECT_2:
		return sprintf(buf, "2\n");
		break;
	}

	return sprintf(buf, "Unknown gSelect!\n");
}

static ssize_t show_level_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mma7455l_info *mma = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", reg_read(mma, MMA7455L_REG_LDTH));
}

static ssize_t show_calibration(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	s16 x, y, z;
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	mutex_lock(&mma->lock);
	x = __reg_read_10(mma, MMA7455L_REG_XOFFL, MMA7455L_REG_XOFFH);
	y = __reg_read_10(mma, MMA7455L_REG_YOFFL, MMA7455L_REG_YOFFH);
	z = __reg_read_10(mma, MMA7455L_REG_ZOFFL, MMA7455L_REG_ZOFFH);
	mutex_unlock(&mma->lock);

	return sprintf(buf, "%d %d %d\n", x, y, z);
}

static ssize_t write_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	if (!strncmp(buf, "Standby", count))
		update_mode(mma, MMA7455L_MODE_STANDBY, mma->gSelect);
	else if (!strncmp(buf, "Measurement", count))
		update_mode(mma, MMA7455L_MODE_MEASUREMENT, mma->gSelect);
	else if (!strncmp(buf, "Level Detection", count))
		update_mode(mma, MMA7455L_MODE_LEVELDETECTION, mma->gSelect);
	else if (!strncmp(buf, "Pulse Detection", count))
		update_mode(mma, MMA7455L_MODE_PULSEDETECTION, mma->gSelect);

	return count;
}

static ssize_t write_gSelect(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long v;
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &v) == 0) {

		switch (v) {
		case 8:
			update_mode(mma, mma->mode, MMA7455L_GSELECT_8);
			break;
		case 4:
			update_mode(mma, mma->mode, MMA7455L_GSELECT_4);
			break;
		case 2:
			update_mode(mma, mma->mode, MMA7455L_GSELECT_2);
			break;
		default:
			return -EINVAL;
			break;
		}
		return count;
	}

	return -EINVAL;
}

static ssize_t write_level_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long v;
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &v) == 0) {
		if (v <= 0xFF) {
			reg_write(mma, MMA7455L_REG_LDTH, v);
			return count;
		} else
			return -EINVAL;
	}

	return -EINVAL;
}

static ssize_t write_calibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int x, y, z;
	struct mma7455l_info *mma = dev_get_drvdata(dev);

	if (sscanf(buf, "%d %d %d", &x, &y, &z) == 3) {
		mutex_lock(&mma->lock);
		__reg_write_10(mma, MMA7455L_REG_XOFFL, MMA7455L_REG_XOFFH, x);
		__reg_write_10(mma, MMA7455L_REG_YOFFL, MMA7455L_REG_YOFFH, y);
		__reg_write_10(mma, MMA7455L_REG_ZOFFL, MMA7455L_REG_ZOFFH, z);
		mutex_unlock(&mma->lock);

		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR(measure, S_IRUGO, show_measure, NULL);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO, show_mode, write_mode);
static DEVICE_ATTR(gSelect, S_IRUGO | S_IWUGO, show_gSelect, write_gSelect);
static DEVICE_ATTR(level_threshold, S_IRUGO | S_IWUGO, show_level_threshold,
			write_level_threshold);
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUGO, show_calibration,
			write_calibration);

static struct attribute *mma7455l_sysfs_entries[] = {
       &dev_attr_measure.attr,
	&dev_attr_mode.attr,
	&dev_attr_gSelect.attr,
	&dev_attr_level_threshold.attr,
	&dev_attr_calibration.attr,
	NULL
};

static struct attribute_group mma7455l_attr_group = {
       .attrs  = mma7455l_sysfs_entries,
};

/* input device handling and driver core interaction */
static int mma7455l_input_open(struct input_dev *inp)
{
	struct mma7455l_info *mma = input_get_drvdata(inp);
	if (mma->mode == MMA7455L_MODE_STANDBY)
		update_mode(mma, MMA7455L_MODE_MEASUREMENT, mma->gSelect);

	return 0;
}

static void mma7455l_input_close(struct input_dev *inp)
{
	struct mma7455l_info *mma = input_get_drvdata(inp);
	update_mode(mma, MMA7455L_MODE_STANDBY, MMA7455L_GSELECT_2);
}

static void mma7455l_dev_poll(struct input_polled_dev *dev)
{
	s8 x, y, z;
	u8 old_Mode, old_gSelect;

	struct mma7455l_info *mma = mma_global;

	get_mode(mma, &old_Mode, &old_gSelect);
	set_mode(mma, MMA7455L_MODE_MEASUREMENT, MMA7455L_GSELECT_2);

	while (reg_read(mma, MMA7455L_REG_STATUS) == 0)
		msleep(10);

	x = 0xFF & reg_read(mma, MMA7455L_REG_XOUT8);
	y = 0xFF & reg_read(mma, MMA7455L_REG_YOUT8);
	z = 0xFF & reg_read(mma, MMA7455L_REG_ZOUT8);

	if ( reg_read(mma, MMA7455L_REG_STATUS) & 0x02 ) {  /* data has been overwritten */
	  return;
	}

// Sign inversion to compensate the accelerometer position in the board.
#if defined(CONFIG_MODULE_CCIMX51)
	x = -(x);
	y = -(y);
#endif

	input_report_abs(mma->input_polled_dev->input, ABS_X, x);
	input_report_abs(mma->input_polled_dev->input, ABS_Y, y);
	input_report_abs(mma->input_polled_dev->input, ABS_Z, z);
	input_sync(mma->input_polled_dev->input);

	set_mode(mma, old_Mode, old_gSelect);
}

static int __devinit mma7455l_probe(struct i2c_client * client , const struct i2c_device_id * id )
{
	int rc;
	struct mma7455l_info *mma;
	struct input_dev *idev;
	u_int8_t wai;

	mma = kzalloc(sizeof(*mma), GFP_KERNEL);
	if (!mma)
		return -ENOMEM;

	mutex_init(&mma->lock);
	INIT_DELAYED_WORK(&mma->work, mma7455l_work);
	mma->client = client;
	mma->flags = 0;
	mma->working = 0;

	/* Set initial calibration data */
	mma->calibration_x = 0;
	mma->calibration_y = 0;
	mma->calibration_z = 0;

	i2c_set_clientdata(client, mma);

	wai = reg_read(mma, MMA7455L_REG_WHOAMI);
	if (wai != MMA7455L_WHOAMI_MAGIC) {
		printk(KERN_ERR
			"mma7455l unknown whoami signature 0x%02x\n", wai);
		dev_set_drvdata(&client->dev, NULL);
		kfree(mma);
		return -ENODEV;
	}

	rc = request_irq(client->irq, mma7455l_interrupt,
			IRQF_TRIGGER_HIGH, "mma7455l", mma);
	if (rc < 0) {
		dev_err(&client->dev, "mma7455l error requesting IRQ %d\n",
				client->irq);
		/* FIXME */
		return rc;
	}

	rc = sysfs_create_group(&client->dev.kobj, &mma7455l_attr_group);
	if (rc) {
		dev_err(&client->dev, "error creating sysfs group\n");
		return rc;
	}

	set_mode(mma, MMA7455L_MODE_MEASUREMENT, MMA7455L_GSELECT_2);

	/*input poll device register */
	mma->input_polled_dev = input_allocate_polled_device();
	if (!mma->input_polled_dev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		return -ENOMEM;
	}
	mma->input_polled_dev->poll = mma7455l_dev_poll;
	mma->input_polled_dev->poll_interval = POLL_INTERVAL;
	mma->input_polled_dev->poll_interval_min = POLL_INTERVAL_MIN;
	mma->input_polled_dev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mma->input_polled_dev->input;
	idev->name = DEVICE_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -128, 128, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -128, 128, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -128, 128, INPUT_FUZZ, INPUT_FLAT);

	rc = input_register_polled_device(mma->input_polled_dev);
	if (!rc) {
		mma_global = mma;
		return 0;
	}

	dev_err(&client->dev, "register poll device failed!\n");
	input_free_polled_device(mma->input_polled_dev);

	set_bit(EV_ABS, mma->input_polled_dev->input->evbit);
	set_bit(ABS_X, mma->input_polled_dev->input->absbit);
	set_bit(ABS_Y, mma->input_polled_dev->input->absbit);
	set_bit(ABS_Z, mma->input_polled_dev->input->absbit);

	input_set_drvdata(mma->input_polled_dev->input, mma);
	mma->input_polled_dev->input->name = DEVICE_NAME;
	mma->input_polled_dev->input->open = mma7455l_input_open;
	mma->input_polled_dev->input->close = mma7455l_input_close;

	update_mode(mma, MMA7455L_MODE_STANDBY, MMA7455L_GSELECT_2);

	mutex_lock(&mma->lock);
	__reg_write_10(mma, MMA7455L_REG_XOFFL,
			MMA7455L_REG_XOFFH, mma->calibration_x);
	__reg_write_10(mma, MMA7455L_REG_YOFFL,
			MMA7455L_REG_YOFFH, mma->calibration_y);
	__reg_write_10(mma, MMA7455L_REG_ZOFFL,
			MMA7455L_REG_ZOFFH, mma->calibration_z);
	mutex_unlock(&mma->lock);

	return rc;
}

static int __devexit mma7455l_remove(struct i2c_client *client)
{
	struct mma7455l_info *mma = dev_get_drvdata(&client->dev);

	free_irq(client->irq, mma);

	sysfs_remove_group(&client->dev.kobj, &mma7455l_attr_group);
	input_unregister_polled_device(mma->input_polled_dev);
	dev_set_drvdata(&client->dev, NULL);
	kfree(mma);

	return 0;
}

#ifdef CONFIG_PM
static int mma7455l_suspend(struct i2c_client *client, pm_message_t message)
{
	struct mma7455l_info *mma = dev_get_drvdata(&client->dev);
	get_mode(mma, &mma->mode, &mma->gSelect);
	set_mode(mma, MMA7455L_MODE_STANDBY, MMA7455L_GSELECT_2);

	return 0;
}

static int mma7455l_resume(struct i2c_client *client)
{
	struct mma7455l_info *mma = dev_get_drvdata(&client->dev);
	update_mode(mma, mma->mode, mma->gSelect);

	return 0;
}
#else
#define mma7455l_suspend NULL
#define mma7455l_resume  NULL
#endif

static const struct i2c_device_id mma7455l_id[] = {
	{ "mma7455l", 1 },
	{ },
};

static struct i2c_driver mma7455l_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},

	.probe	 	= mma7455l_probe,
	.remove	 	= __devexit_p(mma7455l_remove),
	.suspend 	= mma7455l_suspend,
	.resume	 	= mma7455l_resume,
	.id_table	= mma7455l_id,
};

static int __init mma7455l_init(void)
{
	return i2c_add_driver(&mma7455l_driver);
}

static void __exit mma7455l_exit(void)
{
	i2c_del_driver(&mma7455l_driver);
}

MODULE_AUTHOR("Gregoire Gentil <gregoire@gentil.com>");
MODULE_LICENSE("GPL");

module_init(mma7455l_init);
module_exit(mma7455l_exit);
