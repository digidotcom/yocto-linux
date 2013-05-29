/*
 * da9052 BAT module declarations.
  *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 * Copyright(c) 2011 Digi International Ltd.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_BAT_CCIMX53JS_H
#define __LINUX_MFD_DA9052_BAT_CCIMX53JS_H

#include <linux/power_supply.h>

/* STATIC CONFIGURATION */
#define BAT_TYPE				POWER_SUPPLY_TECHNOLOGY_LION
#define DA9052_NUMBER_OF_STORE_CURENT_READING	4
#define CURRENT_MONITORING_WINDOW		10
#define FILTER_SIZE				4

enum charge_status_enum {
	DA9052_NONE = 1,
	DA9052_CHARGING_CC,
	DA9052_CHARGING_CV,
	DA9052_DISCHARGING_WITH_CHARGER,
	DA9052_DISCHARGING_WITHOUT_CHARGER,
	DA9052_PRECHARGING,
	DA9052_LINEARCHARGING,
	DA9052_CHARGEEND
};

enum charger_type_enum {
	DA9052_NOCHARGER = 1,
	DA9052_USB_HUB,
	DA9052_USB_CHARGER,
	DA9052_WALL_CHARGER
};

enum precharge_enum {
	PRE_CHARGE_0MA = 0,
	PRE_CHARGE_20MA = 20,
	PRE_CHARGE_40MA = 40,
	PRE_CHARGE_60MA = 60
};

struct da9052_bat_threshold {
	u16	vddout_mon;
	u16	ichg_thr;
	u16	tbat_thr_min;
	u16	tbat_thr_max;
	u16	tbat_thr_highns;
	u16	tbat_thr_limit;
	u16	tjunc_thr_limit;
	u16	ichg_av_thr_min;
	u16	ichg_av_thr_max;
};

struct da9052_bat_status {
	u8	cal_capacity;
	u8	charging_mode;
	u8	charger_type;
	u8	health;
	u8	status;
	u8	illegalbattery;
};

struct monitoring_state {
	u16	vddout_value;
	u16	current_value;
	u8	bat_level;
	u8	vddout_status:1;
	u8	current_status:1;
	u8	bat_level_status:1;
};

struct da9052_bat_device {
	u16	chg_current_raw[DA9052_NUMBER_OF_STORE_CURENT_READING];
	u16	chg_current;
	u16	bat_voltage;
	u16	backup_bat_voltage;
	u16	vddout;
};


struct da9052_charger_device {
	struct da9052_bat_threshold	threshold;
	struct da9052			*da9052;
	struct da9052_bat_platform_data	*bat_pdata;
	struct delayed_work		work;
	struct power_supply		psy;
	u16				monitoring_interval;
	u16				charger_voltage_drop;
	u16				bat_target_voltage;
	u16				voltage_threshold;
	u16				dcin_current;
	u16				vbus_current;
	u16				usb_charger_current;;
	u16				chg_end_current;
	u16				precharging_current;
	u16				charging_time;
	u8				timer_mode:1;
	u8				charger_buck_lp:1;
	u8				usb_charger_det:1;
	u8				ichg_low_cntr:1;
	u8				sw_temp_cntr:1;
	u8				auto_temp_cntr:1;
};

static inline  u8 bat_temp_reg_to_C(u16 value) { return (55 - value); }
static inline  u8 bat_mV_to_reg(u16 value) { return (((value-4100)/100)<<4); }
static inline  u8 bat_drop_mV_to_reg(u16 value)
		{ return (((value-100)/100)<<6); }
static inline  u16 bat_reg_to_mV(u8 value) { return ((value*100) + 4100); }
static inline  u16 bat_drop_reg_to_mV(u8 value) { return ((value*100)+100); }
static inline  u8 vch_thr_mV_to_reg(u16 value) { return ((value-3700)/100); }
static inline  u8 precharge_mA_to_reg(u8 value) { return ((value/20)<<6); }
static inline  u8 vddout_mon_mV_to_reg(u16 value)
		{ return (((value-2500)*128)/1000); }
static inline  u16 vddout_reg_to_mV(u8 value)
		{ return ((value*1000)/128)+2500; }
static inline  u16 volt_reg_to_mV(u16 value)
		{ return ((value*1000)/512)+2500; }
static inline  u8 ichg_mA_to_reg(u16 value) { return (value/4); }
static inline  u16 ichg_reg_to_mA(u8 value) { return ((value*3900)/1000); }
static inline u8 iset_mA_to_reg(u16 iset_value)
		{\
		if ((70 <= iset_value) && (iset_value <= 120)) \
			return (iset_value-70)/10; \
		else if ((400 <= iset_value) && (iset_value <= 700)) \
			return ((iset_value-400)/50)+6; \
		else if ((900 <= iset_value) && (iset_value <= 1300)) \
			return ((iset_value-900)/200)+13; else return 0;
		}

#define DA9052_BAT_DEBUG 		0

#define SUCCESS				0
#define FAILURE				1

#define TRUE				1
#define FALSE				0

#define set_bits(value, mask)		(value | mask)
#define clear_bits(value, mask)		(value & ~(mask))

#undef DA9052_DEBUG
#if DA9052_BAT_DEBUG
#define DA9052_DEBUG(fmt, args...) printk(KERN_CRIT "[%s:%d]" fmt, __FUNCTION__\
		,__LINE__,	##args)
#else
#define DA9052_DEBUG(fmt, args...)
#endif

/* SSC Read or Write Error */
#define DA9052_SSC_FAIL			150

/* Battery PCB resistance (mR) */
#define DISCHARGE_VDROP    200
#define CHARGE_CC_VDROP    150

/* Battery thesholds(mV, mA) */
#define VCHARGE_MAX        4150
#define VDISCHG_MIN        3100
#define ICHARGE_CC         990
#define ICHARGE_END        100
#define ICHARGE_PRECHG     10

/* CHARGER BUCK REGISTER */
#define DA9052_CHGBUCK_CHGTEMP     (1<<7)
#define DA9052_CHGBUCK_CHGUSBILIM  (1<<6)
#define DA9052_CHGBUCK_CHGBUCKLP   (1<<5)
#define DA9052_CHGBUCK_CHGBUCKEN   (1<<4)
#define DA9052_CHGBUCK_ISETBUCK    (15<<0)
#define ISET_BUCK_80               (0<<0)
#define ISET_BUCK_90               (1<<0)
#define ISET_BUCK_100              (2<<0)
#define ISET_BUCK_110              (3<<0)
#define ISET_BUCK_120              (4<<0)
#define ISET_BUCK_400              (5<<0)
#define ISET_BUCK_450              (6<<0)
#define ISET_BUCK_500              (7<<0)
#define ISET_BUCK_550              (8<<0)
#define ISET_BUCK_600              (9<<0)
#define ISET_BUCK_800              (10<<0)
#define ISET_BUCK_1000             (11<<0)
#define ISET_BUCK_1200             (12<<0)
#define ISET_BUCK_1400             (13<<0)
#define ISET_BUCK_1600             (14<<0)
#define ISET_BUCK_1800             (15<<0)

/* ISET CONTROL REGISTER */
#define DA9052_ISET_ISETDCIN       (15<<4)
#define ISET_DCIN_80               (0<<4)
#define ISET_DCIN_90               (1<<4)
#define ISET_DCIN_100              (2<<4)
#define ISET_DCIN_110              (3<<4)
#define ISET_DCIN_120              (4<<4)
#define ISET_DCIN_400              (5<<4)
#define ISET_DCIN_450              (6<<4)
#define ISET_DCIN_500              (7<<4)
#define ISET_DCIN_550              (8<<4)
#define ISET_DCIN_600              (9<<4)
#define ISET_DCIN_800              (10<<4)
#define ISET_DCIN_1000             (11<<4)
#define ISET_DCIN_1200             (12<<4)
#define ISET_DCIN_1400             (13<<4)
#define ISET_DCIN_1600             (14<<4)
#define ISET_DCIN_1800             (15<<4)

#define DA9052_ISET_ISETVBUS       (15<<0)
#define ISET_VBUS_80               (0<<0)
#define ISET_VBUS_90               (1<<0)
#define ISET_VBUS_100              (2<<0)
#define ISET_VBUS_110              (3<<0)
#define ISET_VBUS_120              (4<<0)
#define ISET_VBUS_400              (5<<0)
#define ISET_VBUS_450              (6<<0)
#define ISET_VBUS_500              (7<<0)
#define ISET_VBUS_550              (8<<0)
#define ISET_VBUS_600              (9<<0)
#define ISET_VBUS_800              (10<<0)
#define ISET_VBUS_1000             (11<<0)
#define ISET_VBUS_1200             (12<<0)
#define ISET_VBUS_1400             (13<<0)
#define ISET_VBUS_1600             (14<<0)
#define ISET_VBUS_1800             (15<<0)

/* BATTERY CHARGER CONTROL REGISTER */
#define DA9052_BATCHG_ICHGPRE    (3<<6)
#define ICHG_PRE_0               (0<<6)
#define ICHG_PRE_20              (1<<6)
#define ICHG_PRE_40              (2<<6)
#define ICHG_PRE_60              (3<<6)

#define DA9052_BATCHG_ICHGBAT    (63<<0)
#define ICHG_BAT_0               (0<<0)
#define ICHG_BAT_120             (4<<0)
#define ICHG_BAT_240             (8<<0)
#define ICHG_BAT_360             (12<<0)
#define ICHG_BAT_480             (16<<0)
#define ICHG_BAT_600             (20<<0)
#define ICHG_BAT_720             (24<<0)
#define ICHG_BAT_840             (28<<0)
#define ICHG_BAT_960             (32<<0)
#define ICHG_BAT_1080            (36<<0)
#define ICHG_BAT_1200            (40<<0)
#define ICHG_BAT_1320            (44<<0)
#define ICHG_BAT_1440            (48<<0)
#define ICHG_BAT_1560            (52<<0)
#define ICHG_BAT_1680            (56<<0)
#define ICHG_BAT_1800            (60<<0)
#define ICHG_BAT_1890            (63<<0)

/* CHARGER COUNTER REGISTER */
#define DA9052_CHGCONT_VCHGBAT    (31<<3)
#define VCHG_BAT_LIION            (22<<3)

#define DA9052_CHGCONT_VCHTHR     (7<<0)
#define VCH_THR_38                (0<<0)
#define VCH_THR_40                (1<<0)
#define VCH_THR_41                (2<<0)
#define VCH_THR_42                (3<<0)
#define VCH_THR_43                (4<<0)
#define VCH_THR_44                (5<<0)
#define VCH_THR_46                (6<<0)
#define VCH_THR_48                (7<<0)

/* INPUT CONTROL REGISTER */
#define DA9053_INPUTCONT_TCTRMODE   (1<<7)
#define DA9053_INPUTCONT_VCHG_DROP  (1<<6)
#define DA9053_INPUTCONT_VBUSSUSP   (1<<4)
#define DA9053_INPUTCONT_DCINSUSP   (1<<5)
#define DA9053_INPUTCONT_TCTR       (15<<0)
#define TCTR_DIS                    (0<<0)

/* Battery discharge voltage level (mV) */
#define VDISCHARGE_5       3500-DISCHARGE_VDROP
#define VDISCHARGE_10      3600-DISCHARGE_VDROP
#define VDISCHARGE_15      3630-DISCHARGE_VDROP
#define VDISCHARGE_20      3660-DISCHARGE_VDROP
#define VDISCHARGE_25      3690-DISCHARGE_VDROP
#define VDISCHARGE_30      3700-DISCHARGE_VDROP
#define VDISCHARGE_35      3720-DISCHARGE_VDROP
#define VDISCHARGE_40      3740-DISCHARGE_VDROP
#define VDISCHARGE_45      3760-DISCHARGE_VDROP
#define VDISCHARGE_50      3780-DISCHARGE_VDROP
#define VDISCHARGE_55      3810-DISCHARGE_VDROP
#define VDISCHARGE_60      3830-DISCHARGE_VDROP
#define VDISCHARGE_65      3850-DISCHARGE_VDROP
#define VDISCHARGE_70      3870-DISCHARGE_VDROP
#define VDISCHARGE_75      3890-DISCHARGE_VDROP
#define VDISCHARGE_80      3920-DISCHARGE_VDROP
#define VDISCHARGE_85      3960-DISCHARGE_VDROP
#define VDISCHARGE_90      4010-DISCHARGE_VDROP
#define VDISCHARGE_95      4050-DISCHARGE_VDROP
#define VDISCHARGE_100     4100-DISCHARGE_VDROP

/* Battery charge voltage level (mV) */
#define VCHARGE_5          3600+CHARGE_CC_VDROP
#define VCHARGE_10         3700+CHARGE_CC_VDROP
#define VCHARGE_15         3750+CHARGE_CC_VDROP
#define VCHARGE_20         3780+CHARGE_CC_VDROP
#define VCHARGE_25         3790+CHARGE_CC_VDROP
#define VCHARGE_30         3800+CHARGE_CC_VDROP
#define VCHARGE_35         3810+CHARGE_CC_VDROP
#define VCHARGE_40         3820+CHARGE_CC_VDROP
#define VCHARGE_45         3840+CHARGE_CC_VDROP
#define VCHARGE_50         3860+CHARGE_CC_VDROP
#define VCHARGE_55         3880+CHARGE_CC_VDROP
#define VCHARGE_60         3910+CHARGE_CC_VDROP
#define VCHARGE_65         3940+CHARGE_CC_VDROP
#define VCHARGE_70         3970+CHARGE_CC_VDROP
#define VCHARGE_75         4010+CHARGE_CC_VDROP
#define VCHARGE_80         4050+CHARGE_CC_VDROP

/* Battery charge current level (mA) */
#define ICHARGE_85         800
#define ICHARGE_90         600
#define ICHARGE_95         300

#endif /* __LINUX_MFD_DA9052_BAT_CCIMX53JS_H */
