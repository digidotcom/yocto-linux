/*
 * linux/drivers/rtc/rtc-ns9xxx.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/clk.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/mach/time.h>

/* registers */
#define RTC_CONFIG			(0x00)
#define RTC_CONFIG_TIMEON			(0)
#define RTC_CONFIG_CALON			(0)
#define RTC_CONFIG_TIMEOFF			(1 << 0)
#define RTC_CONFIG_CALOFF			(1 << 1)

#define RTC_1224H			(0x04)
#define RTC_1224H_MODE				(1 << 0)
#define RTC_1224H_MODE_12H			(1 << 0)
#define RTC_1224H_MODE_24H			(0 << 0)

#define RTC_TIME			(0x08)
#define RTC_TIME_PM			(1 << 30)
#define RTC_TIME_HOUR_MASK		(0x3f000000)
#define RTC_TIME_MINUTE_MASK		(0x007f0000)
#define RTC_TIME_SECOND_MASK		(0x00007f00)
#define RTC_TIME_MONTH_MASK		(0x000000f8)
#define RTC_TIME_DATE_MASK		(0x00003f00)
#define RTC_TIME_DAY_MASK		(0x00000007)
#define RTC_TIME_CENTURY_MASK		(0x3f000000)
#define RTC_TIME_YEAR_MASK		(0x00ff0000)

#define RTC_CAL				(0x0c)
#define RTC_ALARMTIME			(0x10)
#define RTC_ALARMCAL			(0x14)

#define RTC_ALARMENABLE			(0x18)
#define RTC_ALARMENABLE_HSEC			(1 << 0)
#define RTC_ALARMENABLE_SEC			(1 << 1)
#define RTC_ALARMENABLE_MIN			(1 << 2)
#define RTC_ALARMENABLE_HOUR			(1 << 3)
#define RTC_ALARMENABLE_DATE			(1 << 4)
#define RTC_ALARMENABLE_MNTH			(1 << 5)
#define RTC_ALARMENABLE_ALL			(0x3f)

#define RTC_EVENT			(0x1c)

#define RTC_IRQENABLE			(0x20)
#define RTC_IRQDISABLE			(0x24)
#define RTC_IRQSTATUS			(0x28)
#define RTC_IRQ_HUNDREDTH		(1 << 0)
#define RTC_IRQ_SECOND			(1 << 1)
#define RTC_IRQ_MINUTE			(1 << 2)
#define RTC_IRQ_HOUR			(1 << 3)
#define RTC_IRQ_DATE			(1 << 4)
#define RTC_IRQ_MONTH			(1 << 5)
#define RTC_IRQ_ALARM			(1 << 6)

#define RTC_STATUS			(0x2c)
#define RTC_STATUS_TIMEVALID			(1 << 0)
#define RTC_STATUS_CALVALID			(1 << 1)
#define RTC_STATUS_ALARMTIMEVALID		(1 << 2)
#define RTC_STATUS_ALARMCALVALID		(1 << 3)

/* configuration/status shifts */
#define RTC_TIME_HOUR_SHIFT		(24)
#define RTC_TIME_MINUTE_SHIFT		(16)
#define RTC_TIME_SECOND_SHIFT		(8)
#define RTC_CAL_MONTH_SHIFT		(3)
#define RTC_CAL_DATE_SHIFT		(8)
#define RTC_CAL_DAY_SHIFT		(0)
#define RTC_CAL_CENTURY_SHIFT		(24)
#define RTC_CAL_YEAR_SHIFT		(16)

#define RTC_IRQ_PERIOD			RTC_IRQ_SECOND
#define RTC_IRQ_ALL							\
		(RTC_IRQ_HUNDREDTH | RTC_IRQ_SECOND |			\
		RTC_IRQ_MINUTE | RTC_IRQ_HOUR |				\
		RTC_IRQ_DATE | RTC_IRQ_MONTH |				\
		RTC_IRQ_ALARM)

#define RTC_RAM				(0xc0)
#define RTC_RAM_SIZE			(0x100 - RTC_RAM)

#define DRIVER_NAME "rtc-ns9xxx"

struct ns9xxx_rtc_pdata {
	struct rtc_device *rtc;
	struct clk *clk;
	struct clk *pmclk;
	void __iomem *ioaddr;
	struct resource *mem;
	int irq;
	u8 irq_status;
	u8 event;
};

static int __ns9xxx_rtc_timeon(struct ns9xxx_rtc_pdata *pdata, u32 config)
{
	if (config & (RTC_CONFIG_TIMEOFF | RTC_CONFIG_CALOFF)) {
		dev_dbg(&pdata->rtc->dev,
				"time or cal is stopped, RTC_CONFIG=0x%08x\n",
				config);

		return 0;
	}
	return 1;
}

static int ns9xxx_rtc_timeon(struct ns9xxx_rtc_pdata *pdata)
{
	u32 config = ioread32(pdata->ioaddr + RTC_CONFIG);
	return __ns9xxx_rtc_timeon(pdata, config);
}

static int __ns9xxx_rtc_timevalid(struct ns9xxx_rtc_pdata *pdata, u32 status)
{
	if (~status & (RTC_STATUS_TIMEVALID | RTC_STATUS_CALVALID)) {
		dev_dbg(&pdata->rtc->dev,
				"time is invalid, RTC_STATUS=0x%08x\n",
				status);
		return 0;
	}

	return 1;
}
static int ns9xxx_rtc_timevalid(struct ns9xxx_rtc_pdata *pdata)
{
	u32 status = ioread32(pdata->ioaddr + RTC_STATUS);
	return __ns9xxx_rtc_timevalid(pdata, status);
}

static int ns9xxx_rtc_timeonandvalid(struct ns9xxx_rtc_pdata *pdata)
{
	return ns9xxx_rtc_timeon(pdata) && ns9xxx_rtc_timevalid(pdata);
}

static int ns9xxx_rtc_alarmvalid(struct ns9xxx_rtc_pdata *pdata)
{
	int ret;
	u32 status;

	ret = ns9xxx_rtc_timeon(pdata);
	if (!ret)
		return ret;

	status = ioread32(pdata->ioaddr + RTC_STATUS);
	ret = __ns9xxx_rtc_timevalid(pdata, status);
	if (!ret)
		return ret;

	if (~status & (RTC_STATUS_ALARMTIMEVALID |
				(RTC_STATUS_ALARMCALVALID))) {
		dev_dbg(&pdata->rtc->dev,
				"alarm is invalid, RTC_STATUS=0x%08x\n",
				status);
		return 0;
	}

	return 1;
}

#ifdef CONFIG_RTC_INTF_DEV

static int ns9xxx_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s(cmd=%u, arg=%lu)\n", __func__, cmd, arg);
	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	pdata->event |= ioread32(pdata->ioaddr + RTC_EVENT);

	switch (cmd) {
	case RTC_AIE_OFF:
		/* disable alarm-irq */
		pdata->irq_status &= ~RTC_IRQ_ALARM;
		iowrite32(RTC_IRQ_ALARM, pdata->ioaddr + RTC_IRQDISABLE);
		break;

	case RTC_AIE_ON:
		if (ns9xxx_rtc_alarmvalid(pdata)) {
			/* enable alarm-irq */
			pdata->irq_status |= RTC_IRQ_ALARM;
			iowrite32(RTC_IRQ_ALARM, pdata->ioaddr + RTC_IRQENABLE);
			pdata->event &= ~RTC_IRQ_ALARM;
		} else {
			ret = -EINVAL;
			dev_dbg(dev, "%s: alarm not valid\n", __func__);
		}

		break;

	case RTC_UIE_OFF:
		/* disable periodic irq */
		pdata->irq_status &= ~RTC_IRQ_PERIOD;
		iowrite32(RTC_IRQ_PERIOD, pdata->ioaddr + RTC_IRQDISABLE);
		break;

	case RTC_UIE_ON:
		if (ns9xxx_rtc_timeonandvalid(pdata)) {
			/* enable periodic irq */
			pdata->irq_status |= RTC_IRQ_PERIOD;
			iowrite32(RTC_IRQ_PERIOD,
					pdata->ioaddr + RTC_IRQENABLE);
			pdata->event &= ~RTC_IRQ_PERIOD;
		} else {
			ret = -EINVAL;
			dev_dbg(dev, "%s: time not valid\n", __func__);
		}

		break;

	default:
		ret = -ENOIOCTLCMD;
	}

	clk_disable(pdata->clk);

	dev_dbg(dev, "%s(cmd=%u, arg=%lu) -> %d\n", __func__, cmd, arg, ret);
	return ret;
}

#else
# define ns9xxx_rtc_ioctl NULL
#endif /* CONFIG_RTC_INTF_DEV */

static int ns9xxx_rtc_get_time_sync(struct ns9xxx_rtc_pdata *pdata,
		unsigned int *time, unsigned int *cal)
{
	u32 cal1, time1, cal2;
	int tries = 5;

	dev_dbg(&pdata->rtc->dev, "%s\n", __func__);

	if (!ns9xxx_rtc_timeonandvalid(pdata)) {
		dev_dbg(&pdata->rtc->dev, "%s -> %d\n", __func__, -EINVAL);
		return -EINVAL;
	}

	/* necessary to avoid time and date beeing out of sync */
	do {
		cal1 = ioread32(pdata->ioaddr + RTC_CAL);
		time1 = ioread32(pdata->ioaddr + RTC_TIME);
		cal2 = ioread32(pdata->ioaddr + RTC_CAL);
	} while ((cal1 != cal2) && --tries);

	if (!tries) {
		dev_dbg(&pdata->rtc->dev, "%s -> %d\n", __func__, -EBUSY);
		return -EBUSY;
	}

	*time = time1;
	*cal = cal1;

	dev_dbg(&pdata->rtc->dev, "%s: cal/time = %08x/%08x -> 0\n",
			__func__, cal1, time1);
	return 0;
}

static void ns9xxx_rtc_decode_time(struct rtc_time *tm,
		u32 time, u32 cal, u32 mode1224)
{
	tm->tm_hour = bcd2bin((time & RTC_TIME_HOUR_MASK)
			>> RTC_TIME_HOUR_SHIFT);
	if (unlikely((mode1224 & RTC_1224H_MODE) == RTC_1224H_MODE_12H)) {
		if (time & RTC_TIME_PM)
			tm->tm_hour += 12;
	}
	tm->tm_min = bcd2bin((time & RTC_TIME_MINUTE_MASK)
			>> RTC_TIME_MINUTE_SHIFT);
	tm->tm_sec = bcd2bin((time & RTC_TIME_SECOND_MASK)
			>> RTC_TIME_SECOND_SHIFT);

	tm->tm_mon = (bcd2bin((cal & RTC_TIME_MONTH_MASK)
			>> RTC_CAL_MONTH_SHIFT) - 1);
	tm->tm_mday = bcd2bin((cal & RTC_TIME_DATE_MASK)
			>> RTC_CAL_DATE_SHIFT);
	tm->tm_wday = (bcd2bin(cal & RTC_TIME_DAY_MASK)
			>> RTC_CAL_DAY_SHIFT) - 1;
	tm->tm_year = bcd2bin((cal & RTC_TIME_CENTURY_MASK)
			>> RTC_CAL_CENTURY_SHIFT) * 100;
	tm->tm_year += bcd2bin((cal & RTC_TIME_YEAR_MASK)
			>> RTC_CAL_YEAR_SHIFT);
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	tm->tm_year -= 1900;

	pr_debug("%s: %08x/%08x -> %d-%d-%d %d:%d:%d\n",
			__func__, cal, time, tm->tm_year,
			tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static void ns9xxx_rtc_encode_time(struct rtc_time *tm,
		unsigned int *time, unsigned int *cal)
{
	*time  = bin2bcd(tm->tm_hour) << RTC_TIME_HOUR_SHIFT;
	*time |= bin2bcd(tm->tm_min) << RTC_TIME_MINUTE_SHIFT;
	*time |= bin2bcd(tm->tm_sec) << RTC_TIME_SECOND_SHIFT;

	*cal  = bin2bcd(tm->tm_mon + 1) << RTC_CAL_MONTH_SHIFT;
	*cal |= bin2bcd(tm->tm_mday) << RTC_CAL_DATE_SHIFT;
	/* NET+OS uses 1 to encode Sunday, so we follow that practise */
	*cal |= bin2bcd(tm->tm_wday + 1) << RTC_CAL_DAY_SHIFT;
	*cal |= bin2bcd((tm->tm_year + 1900) % 100)
			<< RTC_CAL_YEAR_SHIFT;
	*cal |= bin2bcd((tm->tm_year + 1900) / 100)
			<< RTC_CAL_CENTURY_SHIFT;
	pr_debug("%s: cal/time = %08x/%08x\n", __func__, *cal, *time);
}

static int ns9xxx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 uninitialized_var(time), uninitialized_var(cal);
	u32 mode1224;
	int ret;
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	ret = ns9xxx_rtc_get_time_sync(pdata, &time, &cal);

	mode1224 = ioread32(pdata->ioaddr + RTC_1224H);

	clk_disable(pdata->clk);

	if (!ret)
		ns9xxx_rtc_decode_time(tm, time, cal, mode1224);

	return ret;
}

static int ns9xxx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int time, cal, stat;
	int ret;
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (rtc_valid_tm(tm) != 0)
		return -EINVAL;

	ns9xxx_rtc_encode_time(tm, &time, &cal);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	/* disable operation */
	iowrite32(RTC_CONFIG_TIMEOFF | RTC_CONFIG_CALOFF,
			pdata->ioaddr + RTC_CONFIG);

	iowrite32(time, pdata->ioaddr + RTC_TIME);
	iowrite32(cal, pdata->ioaddr + RTC_CAL);

	iowrite32(RTC_1224H_MODE_24H, pdata->ioaddr + RTC_1224H);

	/* reenable operation */
	iowrite32(RTC_CONFIG_TIMEON | RTC_CONFIG_CALON,
			pdata->ioaddr + RTC_CONFIG);

	/* make sure configs are valid */
	stat = ioread32(pdata->ioaddr + RTC_STATUS);

	clk_disable(pdata->clk);

	if (~stat & (RTC_STATUS_TIMEVALID | RTC_STATUS_CALVALID)) {
		dev_dbg(dev, "%s: invalid date/time: RTC_STATUS = 0x%08x\n",
				__func__, stat);
		return -EINVAL;
	}

	return ret;
}

static int ns9xxx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned int time, cal;
	int ret;
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);
	u32 mode1224, status;
	u32 almenable;

	dev_dbg(dev, "%s\n", __func__);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	status = ioread32(pdata->ioaddr + RTC_STATUS);
	if (~status & (RTC_STATUS_ALARMTIMEVALID | RTC_STATUS_ALARMCALVALID)) {
		dev_dbg(dev, "time or cal is invalid, RTC_STATUS=0x%08x\n",
				status);
		return -EINVAL;
	}

	mode1224 = ioread32(pdata->ioaddr + RTC_1224H);
	time = ioread32(pdata->ioaddr + RTC_ALARMTIME);
	cal  = ioread32(pdata->ioaddr + RTC_ALARMCAL);

	dev_dbg(dev, "%s: cal/time = %08x/%08x\n", __func__, cal, time);
	ns9xxx_rtc_decode_time(&(alm->time), time, cal, mode1224);

	almenable = ioread32(pdata->ioaddr + RTC_ALARMENABLE);
	dev_dbg(dev, "%s: ALARMENABLE = %08x\n", __func__, almenable);

	if (!(almenable & RTC_ALARMENABLE_SEC))
		alm->time.tm_sec = -1;

	if (!(almenable & RTC_ALARMENABLE_MIN))
		alm->time.tm_min = -1;

	if (!(almenable & RTC_ALARMENABLE_HOUR))
		alm->time.tm_hour = -1;

	if (!(almenable & RTC_ALARMENABLE_DATE))
		alm->time.tm_mday = -1;

	if (!(almenable & RTC_ALARMENABLE_MNTH))
		alm->time.tm_mon = -1;

	alm->time.tm_year = -1;

	alm->enabled = !!(pdata->irq_status & RTC_IRQ_ALARM);

	clk_disable(pdata->clk);

	return 0;
}

static int ns9xxx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned int time, cal, stat;
	int ret = 0;
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);

	if (rtc_valid_tm(&(alm->time)) != 0)
		return -EINVAL;

	ns9xxx_rtc_encode_time(&(alm->time), &time, &cal);

	dev_dbg(dev, "%s: %08x/%08x\n", __func__, cal, time);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	if (!alm->enabled) {
		iowrite32(RTC_IRQ_ALARM, pdata->ioaddr + RTC_IRQDISABLE);
		pdata->irq_status &= ~RTC_IRQ_ALARM;
	}

	iowrite32(time, pdata->ioaddr + RTC_ALARMTIME);
	iowrite32(cal, pdata->ioaddr + RTC_ALARMCAL);

	if (alm->enabled) {
		iowrite32(RTC_ALARMENABLE_ALL, pdata->ioaddr + RTC_ALARMENABLE);

		/* make sure configs are valid */
		stat = ioread32(pdata->ioaddr + RTC_STATUS);
		if (!(stat & (RTC_STATUS_ALARMTIMEVALID |
						RTC_STATUS_ALARMCALVALID))) {
			dev_dbg(dev, "%s: invalid date/time: "
					"RTC_STATUS = 0x%08x\n",
					__func__, stat);
			ret = -EINVAL;
		}

		iowrite32(RTC_IRQ_ALARM, pdata->ioaddr + RTC_IRQENABLE);
		pdata->irq_status |= RTC_IRQ_ALARM;
	}

	clk_disable(pdata->clk);

	return ret;
}

static int ns9xxx_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned int stat;
	int ret;
	struct ns9xxx_rtc_pdata *pdata = dev_get_drvdata(dev);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	stat = ioread32(pdata->ioaddr + RTC_IRQSTATUS);

	clk_disable(pdata->clk);

	return seq_printf(seq,
		"IRQ on:%s%s%s%s%s%s\n",
		(stat & RTC_IRQ_ALARM)  ? "" : " alarm",
		(stat & RTC_IRQ_MONTH)  ? "" : " month_rollover",
		(stat & RTC_IRQ_DATE)   ? "" : " day_rollover",
		(stat & RTC_IRQ_HOUR)   ? "" : " hour_rollover",
		(stat & RTC_IRQ_MINUTE) ? "" : " minute_rollover",
		(stat & RTC_IRQ_SECOND) ? "" : " second_rollover");
}

static struct rtc_class_ops ns9xxx_rtc_ops = {
	.ioctl		= ns9xxx_rtc_ioctl,
	.read_time	= ns9xxx_rtc_read_time,
	.set_time	= ns9xxx_rtc_set_time,
	.read_alarm	= ns9xxx_rtc_read_alarm,
	.set_alarm	= ns9xxx_rtc_set_alarm,
	.proc		= ns9xxx_rtc_proc,
};

static irqreturn_t ns9xxx_rtc_irq_handler(int irq, void *irq_data)
{
	u8 event;
	struct ns9xxx_rtc_pdata *pdata = irq_data;
	int handled = 0, ret;

	ret = clk_enable(pdata->clk);
	if (unlikely(ret)) {
		dev_warn(&pdata->rtc->dev, "%s: Could not enable clock, "
				"error is %d\n", __func__, ret);
		return IRQ_NONE;
	}

	pdata->event |= ioread32(pdata->ioaddr + RTC_EVENT);
	event = pdata->event & pdata->irq_status;
	dev_dbg(&pdata->rtc->dev, "%s: RTC_IRQSTATUS = %x, RTC_EVENT = %x\n",
			__func__, pdata->irq_status, pdata->event);
	pdata->event = 0;

	if (event & (RTC_IRQ_ALARM | RTC_IRQ_PERIOD)) {
		unsigned long rtcevents = RTC_IRQF;

		if (event & RTC_IRQ_ALARM) {
			rtcevents |= RTC_AF;

			/* according to the comment in rtc_sysfs_show_wakealarm
			 * further alarm irqs must not occur.
			 */
			pdata->irq_status &= ~RTC_IRQ_ALARM;
			iowrite32(RTC_IRQ_ALARM,
					pdata->ioaddr + RTC_IRQDISABLE);
		}

		if (event & RTC_IRQ_PERIOD)
			rtcevents |= RTC_UF;

		rtc_update_irq(pdata->rtc, 1, rtcevents);

		handled = 1;
	}

	clk_disable(pdata->clk);

	return IRQ_RETVAL(handled);
}

static ssize_t ns9xxx_rtc_nvram_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf,
		loff_t pos, size_t size)
{
	loff_t npos = pos;
	struct ns9xxx_rtc_pdata *pdata = bin_attr->private;
	u32 data;
	int ret;

	if (pos >= RTC_RAM_SIZE)
		return 0;

	if (pos + size > RTC_RAM_SIZE)
		size = RTC_RAM_SIZE - pos;

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	if (npos & 3) {
		data = ioread32(pdata->ioaddr + RTC_RAM + (npos & ~3));
		pr_debug("%s: nvram[0x%04llx]  -> 0x%08x\n", __func__, npos & ~3, data);
	} else
		data = 0; /* silence gcc */

	for (; size > 0; npos++, size--) {
		if (!(npos & 3)) {
			data = ioread32(pdata->ioaddr + RTC_RAM + npos);
			pr_debug("%s: nvram[0x%04llx]  -> 0x%08x\n", __func__, npos, data);
		}

		*buf++ = data >> ((npos & 3) << 3);
	}

	clk_disable(pdata->clk);

	return npos - pos;
}

static ssize_t ns9xxx_rtc_nvram_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr, char *buf,
		loff_t pos, size_t size)
{
	loff_t npos = pos;
	struct ns9xxx_rtc_pdata *pdata = bin_attr->private;
	int ret;
	u32 data;

	if (pos >= RTC_RAM_SIZE)
		return 0;

	if (pos + size > RTC_RAM_SIZE)
		size = RTC_RAM_SIZE - pos;

	pr_debug("%s: pos = 0x%04llx, size = 0x%x\n", __func__, pos, size);
	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	if (npos & 3) {
		data = ioread32(pdata->ioaddr + RTC_RAM + (npos & ~3));
		pr_debug("%s a: nvram[0x%04llx]  -> 0x%08x\n", __func__, npos & ~3, data);
		data &= (1 << ((npos & 3) << 3)) - 1;
	} else
		data = 0;

	for (; size > 0; npos++, size--) {
		data |= *buf++ << ((npos & 3) << 3);

		if ((npos & 3) == 3) {
			pr_debug("%s b: nvram[0x%04llx] <-  0x%08x\n", __func__, npos & ~3, data);
			iowrite32(data, pdata->ioaddr + RTC_RAM + (npos & ~3));

			/* the next write doesn't fill a complete u32.  Note
			 * that size isn't decremented yet for the last byte */
			if (size < 5 && size > 1) {
				data = ioread32(pdata->ioaddr + RTC_RAM + npos + 1);
				pr_debug("%s c: nvram[0x%04llx]  -> 0x%08x\n", __func__, npos + 1, data);
				data &= ~((1 << ((size - 1) << 3)) - 1);
			} else
				data = 0;
		}
	}
	if (npos & 3) {
		pr_debug("%s d: nvram[0x%04llx] <-  0x%08x\n", __func__, npos & ~3, data);
		iowrite32(data, pdata->ioaddr + RTC_RAM + (npos & ~3));
	}

	clk_disable(pdata->clk);

	return npos - pos;
}

static struct bin_attribute ns9xxx_rtc_nvram_attr = {
	.attr = {
		.name = "nvram",
		.mode = S_IRUGO | S_IWUGO,
		.owner = THIS_MODULE,
	},
	.size = RTC_RAM_SIZE,
	.read = ns9xxx_rtc_nvram_read,
	.write = ns9xxx_rtc_nvram_write,
};

static int __devinit ns9xxx_rtc_probe(struct platform_device *pdev)
{
	struct ns9xxx_rtc_pdata *pdata;
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, pdata);

	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "err_get_irq\n");
		goto err_get_irq;
	}

	pdata->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->mem) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "err_get_resource\n");
		goto err_get_resource;
	}

	if (!request_mem_region(pdata->mem->start,
			pdata->mem->end - pdata->mem->start + 1,
			DRIVER_NAME)) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "err_request_mem_region\n");
		goto err_request_mem_region;
	}

	pdata->ioaddr = ioremap(pdata->mem->start,
			pdata->mem->end - pdata->mem->start + 1);
	if (!pdata->ioaddr) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "err_ioremap\n");
		goto err_ioremap;
	} else
		dev_dbg(&pdev->dev, "ioaddr = 0x%p\n", pdata->ioaddr);

	pdata->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(pdata->clk)) {
		ret = PTR_ERR(pdata->clk);
		dev_dbg(&pdev->dev, "err_clk_get -> %d\n", ret);
		goto err_clk_get;
	}

	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "err_clk_enable -> %d\n", ret);
		goto err_clk_enable;
	}

#if defined(CONFIG_PM)
	pdata->pmclk = clk_get(&pdev->dev, DRIVER_NAME "-pm");
	if (IS_ERR(pdata->pmclk)) {
		ret = PTR_ERR(pdata->pmclk);
		dev_dbg(&pdev->dev, "err_clk_get_pm -> %d\n", ret);
		goto err_clk_get_pm;
	}

	/* Make the device wakeup capable, but disabled by default */
	device_init_wakeup(&pdev->dev, 1);
	device_set_wakeup_enable(&pdev->dev, 0);
#endif

	/* disable interrupts and clear event flags */
	iowrite32(RTC_IRQ_ALL, pdata->ioaddr + RTC_IRQDISABLE);
	ioread32(pdata->ioaddr + RTC_EVENT);

	pdata->rtc = rtc_device_register(DRIVER_NAME, &pdev->dev,
			&ns9xxx_rtc_ops, THIS_MODULE);
	if (IS_ERR(pdata->rtc))	{
		ret = PTR_ERR(pdata->rtc);
		dev_dbg(&pdev->dev, "err_rtc_device_register -> %d\n", ret);
		goto err_rtc_device_register;
	}

	ns9xxx_rtc_nvram_attr.private = pdata;
	ret = sysfs_create_bin_file(&pdev->dev.kobj, &ns9xxx_rtc_nvram_attr);
	if (ret)
		goto err_sysfs_bin;

	ret = request_irq(pdata->irq, ns9xxx_rtc_irq_handler, IRQF_SHARED,
			dev_name(&pdata->rtc->dev), pdata);
	if (ret) {
		dev_dbg(&pdev->dev, "err_request_irq -> %d\n", ret);

		rtc_device_unregister(pdata->rtc);
err_rtc_device_register:
		sysfs_remove_bin_file(&pdev->dev.kobj, &ns9xxx_rtc_nvram_attr);
err_sysfs_bin:
#if defined(CONFIG_PM)
		clk_put(pdata->pmclk);
err_clk_get_pm:

#endif
		clk_disable(pdata->clk);
err_clk_enable:

		clk_put(pdata->clk);
err_clk_get:

		iounmap(pdata->ioaddr);
err_ioremap:

		release_mem_region(pdata->mem->start,
				pdata->mem->end - pdata->mem->start + 1);
err_request_mem_region:
err_get_resource:
err_get_irq:

		kfree(pdata);
		return ret;
	}

	clk_disable(pdata->clk);

	return 0;
}

static int __devexit ns9xxx_rtc_remove(struct platform_device *pdev)
{
	struct ns9xxx_rtc_pdata *pdata = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	/* disable interrupts */
	iowrite32(RTC_IRQ_ALL, pdata->ioaddr + RTC_IRQDISABLE);

	clk_disable(pdata->clk);

	free_irq(pdata->irq, pdata);
	sysfs_remove_bin_file(&pdev->dev.kobj, &ns9xxx_rtc_nvram_attr);
	rtc_device_unregister(pdata->rtc);
	clk_put(pdata->clk);
	iounmap(pdata->ioaddr);
	release_mem_region(pdata->mem->start,
			pdata->mem->end - pdata->mem->start + 1);
	kfree(pdata);

	return 0;
}

#ifdef CONFIG_PM

static int ns9xxx_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ns9xxx_rtc_pdata *pdata = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	ret = clk_enable(pdata->clk);
	if (ret)
		return ret;

	if (device_may_wakeup(&pdev->dev)) {

		ret = clk_enable(pdata->pmclk);
		if (ret) {
			dev_dbg(&pdev->dev, "%s: err_enable_pmclk -> %d\n",
					__func__, ret);
			goto err;
		}

		enable_irq_wake(pdata->irq);
		dev_dbg(&pdev->dev, "%s: irq_status = %08x\n",
				__func__, pdata->irq_status);

	} else
		/* disable irqs */
		iowrite32(RTC_IRQ_ALL, pdata->ioaddr + RTC_IRQDISABLE);

err:
	clk_disable(pdata->clk);

	return ret;
}

static int ns9xxx_rtc_resume(struct platform_device *pdev)
{
	struct ns9xxx_rtc_pdata *pdata = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_clk_enable -> %d\n",
				__func__, ret);
		return ret;
	}

	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(pdata->irq);
		clk_disable(pdata->pmclk);
	} else
		iowrite32(pdata->irq_status, pdata->ioaddr + RTC_IRQENABLE);

	clk_disable(pdata->clk);

	return 0;
}

#else
# define ns9xxx_rtc_suspend NULL
# define ns9xxx_rtc_resume NULL
#endif /* CONFIG_PM */

static void ns9xxx_rtc_shutdown(struct platform_device *pdev)
{
	struct ns9xxx_rtc_pdata *pdata = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	ret = clk_enable(pdata->clk);
	if (ret) {
		dev_warn(&pdev->dev, "%s: cannot enable rtc clock to disable "
				"interrupts\n", __func__);
		return;
	}

	/* just disable interrupts */
	iowrite32(RTC_IRQ_ALL,
			pdata->ioaddr + RTC_IRQDISABLE);

	clk_disable(pdata->clk);
}

static struct platform_driver ns9xxx_rtc_driver = {
	.probe		= ns9xxx_rtc_probe,
	.remove		= __devexit_p(ns9xxx_rtc_remove),
	.suspend	= ns9xxx_rtc_suspend,
	.resume		= ns9xxx_rtc_resume,
	.shutdown	= ns9xxx_rtc_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ns9xxx_rtc_init(void)
{
	return platform_driver_register(&ns9xxx_rtc_driver);
}

static void __exit ns9xxx_rtc_exit(void)
{
	platform_driver_unregister(&ns9xxx_rtc_driver);
}

module_init(ns9xxx_rtc_init);
module_exit(ns9xxx_rtc_exit);

MODULE_AUTHOR("Uwe Kleine-Koenig <Uwe.Kleine-Koenig@digi.com>");
MODULE_DESCRIPTION("RTC driver for Digi ns9xxx");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
