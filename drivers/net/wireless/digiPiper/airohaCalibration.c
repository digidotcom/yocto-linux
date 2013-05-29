/*
 * This file contains the code which performs automatic recalibration of the
 * Airoha transceiver.
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free softbware; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/crc32.h>

#include "pipermain.h"
#include "mac.h"
#include "airoha.h"
#include "airohaCalibration.h"
#include "adc121c027.h"


#define POWER_INDEX_STEP                (1)	/* TODO: come up with a rational value for this */

#define SAMPLE_TIMEOUT              (HZ * 10)	/* TODO: What is a good sample timeout?  Do we need one? */
#define RECALIBRATION_PERIOD        (HZ * 15)	/* amount of time to wait between recalibrations */

/*
 * This defines the amount of time to wait for the power levels
 * to settle down after making a large correction (user has
 * changed power level).
 */
#define DEBOUNCE_DELAY				(HZ * 2)

#define MAX_TOLERATED_ERROR_HIGH		(300)	/* maximum over target we will allow */
#define MAX_TOLERATED_ERROR_LOW			(500)	/* maximum under the target we will allow */
#define CONVERT_TO_MDBM(x)              (1000 * (x))	/* power levels are dBm externally, but dBm/1000 internally */

#define NVRAM_WCAL_SIGNATURE            "WCALDATA"

#define MINIMUM_POWER_INDEX             (10)

/*
 * Set piperp->calibrationTxRate to this value to make the
 * transmit routine use the data rate specified by the
 * mac80211 library.
 */
#define USE_MAC80211_DATA_RATE			(NULL)

/*
 * Events we will wait for, also return values for waitForEvent().
 */
#define TIMED_OUT_EVENT                 (1 << 0)
#define TRANSMIT_DONE_EVENT             (1 << 1)
#define SHUTDOWN_AUTOCALIBRATION_EVENT  (1 << 2)
#define RESTART_AUTOCALIBRATION_EVENT   (1 << 3)

/*
 * Set this constant to (1) if you want to force calibration status to
 * be printed.
 */
#define WANT_CALIBRATION_STATUS		(0)


static struct airohaCalibrationData calibration;

static DECLARE_WAIT_QUEUE_HEAD(waitQueue);



/*
 * This routine is called to shut down the transmit ADC sampler.
 */
static void stopSampler(struct piper_priv *digi)
{
	digi->tx_calib_cb = NULL;
}

/*
 * This routine is called to update the state of the calibration state machine
 * and wake up its thread.
 */
static void kickCalibrationThread(struct piper_priv *digi, unsigned int event)
{
	unsigned long spinlockFlags;

	spin_lock_irqsave(&calibration.lock, spinlockFlags);
	calibration.events |= event;
	spin_unlock_irqrestore(&calibration.lock, spinlockFlags);
	wake_up_interruptible(&waitQueue);
}


/*
 * This routine is called each time we complete a transmit while we are in
 * the sampling state.  We record the peak ADC reading.  We kick the state
 * machine if we now have all the samples we need.
 */
static void processSample(struct piper_priv *digi)
{
#define MINIMUM_ADC_VALUE       (10)	/* if ADC is below this value, it's probably bad */
	if (calibration.sampleCount < MAX_SAMPLES) {
		/*
		 * Read the ADC value.  It is a 12-bit value.  We shift it 4 bits to
		 * create an 8-bit value.
		 */
		calibration.sample[calibration.sampleCount].sample =
		    (calibration.cops->adc_read_peak(&calibration) >> 4);
		if (calibration.sample[calibration.sampleCount].sample >
		    MINIMUM_ADC_VALUE) {
			calibration.sampleCount++;
		}
	}
}

static void transmitHasCompleted(struct piper_priv *digi)
{
	stopSampler(digi);
	kickCalibrationThread(digi, TRANSMIT_DONE_EVENT);
}



/*
 * Determine the appropriate transmit rate to use during calibration.
 */
static struct ieee80211_rate *determineCalibrationTxRate(struct piper_priv
							 *digi)
{
	unsigned int rates = digi->ac->rd_reg(digi, MAC_SSID_LEN);
	struct ieee80211_rate *calibrationTxRate;

	rates &= (MAC_PSK_BRS_MASK | MAC_OFDM_BRS_MASK);

	if ((digi->rf->getBand(digi->channel) == IEEE80211_BAND_2GHZ)
	    && (rates & MAC_PSK_BRS_MASK)) {
		calibrationTxRate =
		    (struct ieee80211_rate *) digi->rf->
		    getRate(AIROHA_LOWEST_PSK_RATE_INDEX);
	} else {
		calibrationTxRate =
		    (struct ieee80211_rate *) digi->rf->
		    getRate(AIROHA_LOWEST_OFDM_RATE_INDEX);
	}

	return calibrationTxRate;
}




/*
 * Start collecting sample ADC peak measurements for calibration.  Start
 * the process by installing the callbacks which the transmit code will
 * use to notify us when transmit frames go out.
 */
static void startSampleCollection(struct piper_priv *digi)
{
	calibration.cops->adc_clear_peak(&calibration);
	digi->tx_calib_cb = transmitHasCompleted;
}


static unsigned int waitForEvent(unsigned int timeout, unsigned int eventToWaitFor)
{
#define ALL_EVENTS_TO_WAIT_FOR(x)   (eventToWaitFor \
                                     | SHUTDOWN_AUTOCALIBRATION_EVENT \
	                                 | RESTART_AUTOCALIBRATION_EVENT)

	unsigned long spinlockFlags;
	int ccode;
	unsigned int event;
	int result = TIMED_OUT_EVENT;

	if (timeout != 0) {
		ccode = wait_event_interruptible_timeout(waitQueue,
							 ((calibration.
							   events &
							   ALL_EVENTS_TO_WAIT_FOR
							   (eventToWaitFor))
							  != 0), timeout);
		__set_current_state(TASK_RUNNING);

	} else {
		ccode = 0;
	}
	spin_lock_irqsave(&calibration.lock, spinlockFlags);
	event = calibration.events;
	calibration.events = 0;
	spin_unlock_irqrestore(&calibration.lock, spinlockFlags);

	if ((ccode < 0) || (event & SHUTDOWN_AUTOCALIBRATION_EVENT)) {
		result = SHUTDOWN_AUTOCALIBRATION_EVENT;
	} else if (event & RESTART_AUTOCALIBRATION_EVENT) {
		result = RESTART_AUTOCALIBRATION_EVENT;
	} else if (event & eventToWaitFor) {
		result = (event & eventToWaitFor);
	} else {
		result = TIMED_OUT_EVENT;
	}

	return result;
}


#ifdef WANT_CAL_DEBUG
static void printPoint(wcd_point_t * p)
{
	printk("(%d, %d, %d)", p->out_power, p->adc_val, p->power_index);
}
#endif


/*
 * This routine finds the closest pair of points in a calibration curve.
 *
 *      curve       calibration curve
 *      value       look for the pr of points closest to this value
 *      p1          storage for one point
 *      p2          storage for another point
 *      field       tells us which field in the point struct to compare
 */
static void findClosestPoints(wcd_curve_t * curve, int value,
			      wcd_point_t ** p1, wcd_point_t ** p2, int field)
{
	if (value <= curve->points[0].out_power) {
		*p1 = &curve->points[0];
		*p2 = &curve->points[1];
	} else if (value >=
		   curve->points[calibration.nvram->header.numcalpoints - 1].out_power) {
		*p1 = &curve->points[calibration.nvram->header.numcalpoints - 2];
		*p2 = &curve->points[calibration.nvram->header.numcalpoints - 1];
	} else {
		unsigned int idx;

		for (idx = 1; idx < calibration.nvram->header.numcalpoints; idx++) {
			if ((value < curve->points[idx].out_power)
			    || (idx == (calibration.nvram->header.numcalpoints - 1))) {
				*p1 = &curve->points[idx - 1];
				*p2 = &curve->points[idx];
				break;
			} else if (value == curve->points[idx].out_power) {
				/*
				 * Note that the if statement befpre the for loop already tested for the
				 * value being equal to the first or last point in the curve, so we don't
				 * have to worry about that condition in the code below.
				 */
				if ((value -
				     curve->points[idx - 1].out_power) >=
				    (curve->points[idx + 1].out_power - value)) {
					/*
					 * If the two points are equal distant, then favor the larger pair
					 * because I think the values on the low end are screwy.
					 */
					*p1 = &curve->points[idx];
					*p2 = &curve->points[idx + 1];
				} else {
					*p1 = &curve->points[idx - 1];
					*p2 = &curve->points[idx];
				}
				break;
			}
		}
	}
}


/*
 * Compute the slope of a curve between 2 points.  The slope is the rise over the run,
 * or (Y2 - Y1)/(X2 - X1).  This function handles more than one type of slope.
 */
static int computeSlopeTimes1000(wcd_point_t * p1, wcd_point_t * p2, int slopeType)
{
	int slope = 0;
	int divisor;

	switch (slopeType) {
	default:
		digi_dbg("Unexpected slope type %d.\n", slopeType);
		break;
	case POWER_INDEX_OVER_OUT_POWER:
		divisor = (p2->out_power - p1->out_power);
		if (divisor != 0) {
			slope =
			    (((p2->power_index - p1->power_index) * 1000) +
			     (divisor / 2)) / divisor;
		} else {
			digi_dbg("divisor is zero\n");
		}
		break;
	case ADC_OVER_OUT_POWER:
		divisor = (p2->out_power - p1->out_power);
		if (divisor != 0) {
			slope =
			    (((p2->adc_val - p1->adc_val) * 1000) +
			     (divisor / 2)) / divisor;
		} else {
			digi_dbg("divisor is zero\n");
		}
		break;
	case OUT_POWER_OVER_ADC:
		divisor = (p2->adc_val - p1->adc_val);
		if (divisor != 0) {
			slope =
			    (((p2->out_power - p1->out_power) * 1000) +
			     (divisor / 2)) / divisor;
		} else {
			digi_dbg("divisor is zero\n");
		}
		break;
	case POWER_INDEX_OVER_ADC:
		divisor = (p2->adc_val - p1->adc_val);
		if (divisor != 0) {
			slope =
			    (((p2->power_index - p1->power_index) * 1000) +
			     (divisor / 2)) / divisor;
		} else {
			digi_dbg("divisor is zero\n");
		}
		break;
	}

	return slope;
}


/*
 * If (x,y) is a point on a curve, then compute y given x, the slope of the curve,
 * and a known point on the curve.
 *
 * If (Xd, Yd) is the desired point, p1 is the known point, and m the slope, then
 *
 *      Yd - p1->y = m(Xd - p1->x)
 *      Yd         = m(Xd - p1->x) + p1->y
 *      Yd         = m(Xd) - m(p1->x) + p1->y
 */
static int computeY(wcd_point_t * p1, int slopeTimes1000, int x, int slopeType)
{
	int y = 0;

	switch (slopeType) {
	default:
		digi_dbg("Unexpected slope type %d.\n", slopeType);
		break;
	case POWER_INDEX_OVER_OUT_POWER:
		y = (((slopeTimes1000 * x) -
		      (slopeTimes1000 * p1->out_power) + 500) / 1000) + p1->power_index;
		break;
	case ADC_OVER_OUT_POWER:
		y = (((slopeTimes1000 * x) -
		      (slopeTimes1000 * p1->out_power) + 500) / 1000) + p1->adc_val;
		break;
	case OUT_POWER_OVER_ADC:
		y = (((slopeTimes1000 * x) -
		      (slopeTimes1000 * p1->adc_val) + 500) / 1000) + p1->out_power;
		break;
	case POWER_INDEX_OVER_ADC:
		y = (((slopeTimes1000 * x) -
		      (slopeTimes1000 * p1->adc_val) + 500) / 1000) + p1->power_index;
		break;
	}

	return y;
}


/*
 * Return a pointer to the curve we should use for the currently selected
 * channel.
 */
static wcd_curve_t *determineCurve(struct piper_priv *digi)
{
	unsigned int rates = digi->ac->rd_reg(digi, MAC_SSID_LEN);
	wcd_curve_t *curve = NULL;

	if (digi->rf->getBand(digi->channel) == IEEE80211_BAND_2GHZ) {
		if (rates & MAC_PSK_BRS_MASK) {
			/*
			 * This is the normal case for 802.11b and 802.11bg.  We select
			 * the PSK curve.
			 */
			digi_dbg("Using bg curve [%d][%d]\n",
				 digi->channel, WCD_B_CURVE_INDEX);
			curve = &calibration.nvram->cal_curves_bg[digi->channel]
			    [WCD_B_CURVE_INDEX];
		} else {	/* if associated with AP that only supports G rates */

			/*
			 * This is a very unusual, but theoretically possible case.  We
			 * are associated with an AP that only supports OFDM modulation
			 * (G only, no B).  We determine this by looking at the BRS
			 * setting.  If no PSK rates are set for BRS, then we assume that
			 * this must be a G only AP.  Obviously, we must select the OFDM curve.
			 */
			digi_dbg("Using bg curve [%d][%d]\n",
				 digi->channel, WCD_G_CURVE_INDEX);
			curve = &calibration.nvram->cal_curves_bg[digi->channel]
			    [WCD_G_CURVE_INDEX];
		}
	} else {
		/*
		 * An 802.11a channel is selected.
		 */
		curve = &calibration.nvram->cal_curves_a[digi->channel - BAND_A_OFFSET];
		digi_dbg("Using A curve [%d]\n", digi->channel - BAND_A_OFFSET);
	}

	return curve;
}

/*
 * Determine the maximum mdBm allowed for the current channel.  Return the
 * lesser of the max and the mdBm value passed to us.
 */
static int getFilteredPower(struct piper_priv *digi, int mdBm)
{
	int max = 0;

	if (digi->channel < BAND_A_OFFSET) {
		max = 16000;	/* all BG channels can go to 16 dBm */
	} else if (digi->channel < (BAND_A_OFFSET + 4)) {
		max = 3000;	/* first 4 A channels max out at 3 dBm */
	} else {
		max = 8000;	/* all other A channels can handle 8 dBm */
	}

	if (mdBm > max) {
		mdBm = max;
	}

	return mdBm;
}


/*
 * This routine performs open loop calibration for Airoha.  It takes a value in mdbm
 * and uses the factory calibration routines to determine the appropriate register
 * value to write to airoha.
 */
static void setInitialPowerLevel(struct piper_priv *digi, int mdBm)
{
	wcd_point_t *p1, *p2;

	mdBm = getFilteredPower(digi, mdBm);
	digi_dbg("Setting initial powerlevel %d milli dBm.\n", mdBm);
	calibration.curve = determineCurve(digi);
	findClosestPoints(calibration.curve, mdBm, &p1, &p2, OUT_POWER);
	calibration.slopeTimes1000 =
	    computeSlopeTimes1000(p1, p2, POWER_INDEX_OVER_OUT_POWER);
	if (abs(p1->out_power - mdBm) < abs(p2->out_power - mdBm)) {
		calibration.p1 = p1;
	} else {
		calibration.p1 = p2;
	}
	calibration.powerIndex =
	    computeY(calibration.p1, calibration.slopeTimes1000, mdBm,
		     POWER_INDEX_OVER_OUT_POWER);
	calibration.correctedPowerIndex = calibration.powerIndex;

	digi_dbg("Computed power index = %d.\n", calibration.powerIndex);
	digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);

	/*
	 * Let's compute and save the expected ADC value while we have all the necessary
	 * information handy.
	 */
#ifdef WANT_CAL_DEBUG
	digi_dbg("Using points ");
	printPoint(p1);
	printPoint(p2);
	printk("\n");
	digi_dbg("Max ADC = %d.\n", calibration.curve->max_adc_value);
#endif
	calibration.adcSlopeTimes1000 = computeSlopeTimes1000(p1, p2, ADC_OVER_OUT_POWER);
	calibration.expectedAdc =
	    computeY(calibration.p1, calibration.adcSlopeTimes1000, mdBm,
		     ADC_OVER_OUT_POWER);
	if (calibration.expectedAdc > calibration.curve->max_adc_value) {
		calibration.expectedAdc = calibration.curve->max_adc_value;
	}
	digi_dbg("adcSlopeTimes1000 = %d, expectedAdc = %d\n",
		 calibration.adcSlopeTimes1000, calibration.expectedAdc);
	calibration.outPowerSlopeTimes1000 =
	    computeSlopeTimes1000(p1, p2, OUT_POWER_OVER_ADC);
	calibration.powerIndexSlopeTimes1000 =
	    computeSlopeTimes1000(p1, p2, POWER_INDEX_OVER_ADC);
}



/*
 * This routine performs closed loop recalibration.  It is called periodically
 * to adjust the transmit power level.  It will be called after the ADC levels
 * for several transmit frames have been sampled.  It does the following:
 *
 *      1.  Average the samples together.
 *      2.  If the measured ADC level is too low, then increase the power
 *          level one step.
 *      3.  If the measured ADC level is too high, then decrease the power
 *          level one step.
 */
static void recalibrate(struct piper_priv *digi)
{
	unsigned int idx;
	int actualAdc = 0;
	int errorInAdc, errorInMdbm;
	wcd_point_t p = {
		.out_power = 0,
		.adc_val = 0
	};
	int needCorrection = 0;

#ifdef WANT_CAL_DEBUG_1
	digi_dbg("Samples: ");
#endif
	for (idx = 0; idx < calibration.sampleCount; idx++) {
#ifdef WANT_CAL_DEBUG_1
		printk("%d, ", calibration.sample[idx].sample);
#endif
		actualAdc += calibration.sample[idx].sample;
	}
#ifdef WANT_CAL_DEBUG_1
	printk("\n");
#endif
	actualAdc = actualAdc / calibration.sampleCount;

#ifdef WANT_TO_FORCE_26

	digi->rf->set_pwr_index(digi->hw, 26);
#else

	errorInAdc = actualAdc - calibration.expectedAdc;
	errorInMdbm =
	    computeY(&p, calibration.outPowerSlopeTimes1000, errorInAdc,
		     OUT_POWER_OVER_ADC);
	needCorrection = (errorInMdbm > MAX_TOLERATED_ERROR_HIGH);
	if (errorInMdbm < 0) {
		needCorrection = ((MAX_TOLERATED_ERROR_LOW + errorInMdbm) < 0);
	}
	if (needCorrection) {
		int correction = computeY(calibration.p1,
					  calibration.powerIndexSlopeTimes1000,
					  actualAdc, POWER_INDEX_OVER_ADC);
#if defined(WANT_CAL_DEBUG)
		int oldIndex = calibration.correctedPowerIndex;
#endif
		correction = (3 * (calibration.powerIndex - correction)) / 4;
		if (correction == 0) {
			if (errorInAdc > 0) {
				/*
				 * If power level is too high but the minimum correction would
				 * overshoot the target, do it anyway because we would rather
				 * be below the target rather than over the target.
				 */
				correction = -1;
			} else {
				/*
				 * But if the power level is too low, but the minimum correction
				 * would overshoot, then leave it be.  Better too low than too high.
				 */
			}
		}
		calibration.correctedPowerIndex += correction;
		if (calibration.correctedPowerIndex < MINIMUM_POWER_INDEX) {
			calibration.correctedPowerIndex = MINIMUM_POWER_INDEX;
		}
		digi->rf->set_pwr_index(digi->hw, calibration.correctedPowerIndex);
#ifdef WANT_CAL_DEBUG
		digi_dbg
		    ("actualAdc = %d, expectedAdc = %d, error mdbm = %d\n",
		     actualAdc, calibration.expectedAdc, errorInMdbm);
		digi_dbg
		    ("Power index corrected by %d: was %d, set to %d.\n",
		     correction, oldIndex, calibration.correctedPowerIndex);
#endif
#if WANT_CALIBRATION_STATUS
		if (correction != 0)
			printk(KERN_ERR
			       "error mdBm = %d, correcting airoha index by %d\n",
			       errorInMdbm, correction);
#endif
	}
#endif
}


/*
 * This routine is called by the 80211mac library to set a new power level.  We
 * update the value in context memory and then kick the autocalibration thread.
 */
static int setNewPowerLevel(struct ieee80211_hw *hw, uint8_t newPowerLevel)
{
	struct piper_priv *digi = hw->priv;

	(void) newPowerLevel;	/* has already been written into piper->tx_power */
	/*
	 * Kick the calibration thread.
	 */
	stopSampler(digi);
	digi->calibrationTxRate = USE_MAC80211_DATA_RATE;
	kickCalibrationThread(digi, RESTART_AUTOCALIBRATION_EVENT);

	return 0;
}

void digiWifiCalibrationRestartCalibration(struct piper_priv *digi)
{
	/*
	 * Kick the calibration thread.
	 */
	stopSampler(digi);
	kickCalibrationThread(digi, RESTART_AUTOCALIBRATION_EVENT);
}


/*
 * Compute the maximum ADC value for a curve given the maximum
 * allowed power in dBm.
 */
static u8 getMaxAdcValue(wcd_curve_t * curve, int dBm)
{
	wcd_point_t *p1, *p2;
	int slopeTimes1000;
	u8 result = 0;
	int mdBm = 1000 * dBm;

	findClosestPoints(curve, mdBm, &p1, &p2, OUT_POWER);
	slopeTimes1000 = computeSlopeTimes1000(p1, p2, ADC_OVER_OUT_POWER);

	if (abs(p1->out_power - mdBm) < abs(p2->out_power - mdBm)) {
		result = computeY(p1, slopeTimes1000, mdBm, ADC_OVER_OUT_POWER);
	} else {
		result = computeY(p2, slopeTimes1000, mdBm, ADC_OVER_OUT_POWER);
	}

	return result;
}


/*
 * The version 1.0 calibration data provided maximum Airoha index
 * values, but later versions provided maximum ADC values.  This
 * routine replaces the max Airoha indexes with max ADC values.
 */
static void determineMaxAdcValues(wcd_data_t * cdata)
{
	int i;

	for (i = 0; i < WCD_CHANNELS_BG; i++) {
		/*
		 * All BG channels are limited to 16 dBm.
		 */
		cdata->cal_curves_bg[i][0].max_adc_value =
		    getMaxAdcValue(&cdata->cal_curves_bg[i][0], 16);
		cdata->cal_curves_bg[i][1].max_adc_value =
		    getMaxAdcValue(&cdata->cal_curves_bg[i][1], 16);
	}
	for (i = 0; i < 4; i++) {
		/*
		 * First 4 802.11a channels are limited to 3 dBm.
		 */
		cdata->cal_curves_a[i].max_adc_value =
		    getMaxAdcValue(&cdata->cal_curves_a[i], 3);
	}
	for (i = 4; i < WCD_CHANNELS_A; i++) {
		/*
		 * All other 802.11a channels are limited to 8 dBm.
		 */
		cdata->cal_curves_a[i].max_adc_value =
		    getMaxAdcValue(&cdata->cal_curves_a[i], 8);
	}
}



/*
 * This routine writes a default set of calibration values into the
 * calibration data structure.  Maximum power output is severely limited.
 */
static void setDefaultCalibrationValues(struct piper_priv *piperp)
{
#define DEFAULT_NUM_POINTS	(DEFAULT_NUM_POINTS)
#define BAND_A_1_START		(0)
#define BAND_A_2_START		(4)
#define BAND_A_3_START		(7)
#define BAND_A_4_START		(15)
#define BAND_A_5_START		(19)
#define BAND_A_6_START		(30)

	int i;

	calibration.nvram->header.numcalpoints = 2;

#ifdef CONFIG_MACH_CCW9P9215JS
#define MIN_MDBM			(-2905)
#define MAX_BG_MDBM			(6000)
    piperp->pdata->wcd.header.hw_platform = WCD_HW_REV_A | WCD_CCW9P_PLATFORM;
	for (i = 0; i < WCD_CHANNELS_BG; i++) {
		calibration.nvram->cal_curves_bg[i][0].max_adc_value = 52;
		calibration.nvram->cal_curves_bg[i][0].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_bg[i][0].points[0].adc_val = 19;
		calibration.nvram->cal_curves_bg[i][0].points[0].power_index = 0;
		calibration.nvram->cal_curves_bg[i][0].points[1].out_power = 6000;
		calibration.nvram->cal_curves_bg[i][0].points[1].adc_val = 52;
		calibration.nvram->cal_curves_bg[i][0].points[1].power_index = 16;

		calibration.nvram->cal_curves_bg[i][1].max_adc_value = 48;
		calibration.nvram->cal_curves_bg[i][1].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_bg[i][1].points[0].adc_val = 12;
		calibration.nvram->cal_curves_bg[i][1].points[0].power_index = 0;
		calibration.nvram->cal_curves_bg[i][1].points[1].out_power = 6000;
		calibration.nvram->cal_curves_bg[i][1].points[1].adc_val = 48;
		calibration.nvram->cal_curves_bg[i][1].points[1].power_index = 24;
	}

	for (i = BAND_A_1_START; i < BAND_A_2_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 22;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 11;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 0;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 22;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 19;
	}
	for (i = BAND_A_2_START; i < BAND_A_3_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 29;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 13;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 29;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 20;
	}
	for (i = BAND_A_3_START; i < BAND_A_4_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 42;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 15;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 4000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 42;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 22;
	}
	for (i = BAND_A_4_START; i < BAND_A_5_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 54;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 21;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 54;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 18;
	}
	for (i = BAND_A_5_START; i < BAND_A_6_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 39;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 13;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 39;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 26;
	}
	for (i = BAND_A_6_START; i < WCD_CHANNELS_A; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 31;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 11;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 31;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 30;
	}
#else
#define MIN_MDBM			(-1000)
#define MIN_OFDM_MDBM		(-5000)
#define MAX_BG_MDBM			(6000)
    piperp->pdata->wcd.header.hw_platform = WCD_HW_REV_A | WCD_CCW9M_PLATFORM;
	for (i = 0; i < WCD_CHANNELS_BG; i++) {
		calibration.nvram->cal_curves_bg[i][0].max_adc_value = 52;
		calibration.nvram->cal_curves_bg[i][0].points[0].out_power = MIN_MDBM;
		calibration.nvram->cal_curves_bg[i][0].points[0].adc_val = 19;
		calibration.nvram->cal_curves_bg[i][0].points[0].power_index = 0;
		calibration.nvram->cal_curves_bg[i][0].points[1].out_power = 6000;
		calibration.nvram->cal_curves_bg[i][0].points[1].adc_val = 52;
		calibration.nvram->cal_curves_bg[i][0].points[1].power_index = 16;

		calibration.nvram->cal_curves_bg[i][1].max_adc_value = 48;
		calibration.nvram->cal_curves_bg[i][1].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_bg[i][1].points[0].adc_val = 12;
		calibration.nvram->cal_curves_bg[i][1].points[0].power_index = 0;
		calibration.nvram->cal_curves_bg[i][1].points[1].out_power = 6000;
		calibration.nvram->cal_curves_bg[i][1].points[1].adc_val = 48;
		calibration.nvram->cal_curves_bg[i][1].points[1].power_index = 24;
	}

	for (i = BAND_A_1_START; i < BAND_A_2_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 22;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 11;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 0;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 22;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 19;
	}
	for (i = BAND_A_2_START; i < BAND_A_3_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 29;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 13;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 29;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 20;
	}
	for (i = BAND_A_3_START; i < BAND_A_4_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 42;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 15;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 4000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 42;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 22;
	}
	for (i = BAND_A_4_START; i < BAND_A_5_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 54;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 21;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 54;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 18;
	}
	for (i = BAND_A_5_START; i < BAND_A_6_START; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 39;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 13;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 11;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 26;
	}
	for (i = BAND_A_6_START; i < WCD_CHANNELS_A; i++) {
		calibration.nvram->cal_curves_a[i].max_adc_value = 31;
		calibration.nvram->cal_curves_a[i].points[0].out_power = MIN_OFDM_MDBM;
		calibration.nvram->cal_curves_a[i].points[0].adc_val = 0;
		calibration.nvram->cal_curves_a[i].points[0].power_index = 0;
		calibration.nvram->cal_curves_a[i].points[1].out_power = 2000;
		calibration.nvram->cal_curves_a[i].points[1].adc_val = 31;
		calibration.nvram->cal_curves_a[i].points[1].power_index = 30;
	}
#endif
}


/*
 * The calibration data is passed to the kernel from U-Boot.  The kernel
 * start up routines will have copied the data into digi->pdata->wcd.
 * We do a few sanity checks on the data and set up our own pointers to
 * it.
 */
static int getCalibrationData(struct piper_priv *digi)
{
	int result = 0;

	calibration.nvram = &digi->pdata->wcd;

	if (strncmp(calibration.nvram->header.magic_string,
		    NVRAM_WCAL_SIGNATURE, strlen(NVRAM_WCAL_SIGNATURE)) == 0) {
		unsigned int crc = ~crc32_le(~0,
					     (unsigned char const *) calibration.nvram->
					     cal_curves_bg,
					     calibration.nvram->header.wcd_len);

		if (crc == calibration.nvram->header.wcd_crc) {
			printk(KERN_ERR "CRC and signature for calibration data is okay\n");
			result = 0;
			if ((calibration.nvram->header.ver_major == '1')
			    && (calibration.nvram->header.ver_minor == '0')) {
				printk(KERN_ERR "Converting version 1.0 calibration data\n");
				determineMaxAdcValues(calibration.nvram);
				/*
				 * Now that we have updated the format of the data, we need
				 * to recompute the check sum and set the new version.
				 */
				calibration.nvram->header.ver_minor = '1';
				calibration.nvram->header.wcd_crc =
				    ~crc32_le(~0, (unsigned char const *)
					      calibration.nvram->
					      cal_curves_bg,
					      calibration.nvram->header.wcd_len);
			    calibration.nvramDataState = NV_CONVERTED_10;
			} else {
			    calibration.nvramDataState = NV_OKAY;
			}
			digi->rf->set_hw_info(digi->hw, digi->channel,
								  calibration.nvram->header.hw_platform);
		} else {
			printk(KERN_ERR "Calibration data has invalid CRC.\n");
			setDefaultCalibrationValues(digi);
			calibration.nvramDataState = NV_CRC_FAILED;
		}
	} else {
		printk(KERN_ERR "Calibration data has invalid signature.\n");
		setDefaultCalibrationValues(digi);
		calibration.nvramDataState = NV_INVALID_SIGNATURE;
	}

	return result;
}

/*
 * This routine dumps the calibration data read from NVRAM.
 */
void digiWifiCalibrationDumpNvram(struct piper_priv *piperp)
{
    switch (calibration.nvramDataState) {
        case NV_NOT_READ:
            printk(KERN_ERR "\nWARNING: Calibration data has not been read and processed by the driver.\n");
            break;
        case NV_OKAY:
            printk(KERN_ERR "\nCalibration data is okay\n");
            break;
        case NV_CRC_FAILED:
            printk(KERN_ERR "\nERROR: Calibration data failed CRC check.\n");
            break;
        case NV_INVALID_SIGNATURE:
            printk(KERN_ERR "\nERROR: Calibration has an invalid signature.\n");
            break;
        case NV_CONVERTED_10:
            printk(KERN_ERR "\nCalibration data was converted from 1.0 format.\n");
            break;
        default:
            printk(KERN_ERR "\nERROR: Unsupported calibration data state %d.\n", calibration.nvramDataState);
            break;
    }

    if (calibration.nvramDataState != NV_NOT_READ) {
        unsigned int i, column = 0;
        unsigned int *p = (unsigned int *) calibration.nvram;

        for (i = 0; i < (sizeof(wcd_data_t) / sizeof(unsigned int)); i++) {
            if (column == 0) {
                printk(KERN_ERR);
            }
            printk("%8.8X ", *p);
            p++;
            column++;
            if (column == 4) {
                printk("- ");
            }
            else if (column == 8) {
                printk("\n");
                column = 0;
            }
        }

        if (column != 0) {
            printk("\n\n");
        }
        else {
            printk(KERN_ERR "\n");
        }
    }
}


/*
 * This routine:
 *
 *      1. Loads the ADC driver.
 *      2. Loads the calibration data.
 *      3. Implements the automatic calibration state machine.
 *
 */
static int digiWifiCalibrationThreadEntry(void *data)
{
	struct piper_priv *digi = data;
	int state;

	__set_current_state(TASK_RUNNING);

	while (1) {
		/*
		 * We, the wireless driver, may be loaded before the I2C core has
		 * loaded.  Therefore we may not be able to load our ADC driver,
		 * which is an I2C client driver, when we load.  This loop tries
		 * and retries to load the ADC driver until it succeeds.
		 */

		/* TODO, FIXME make following code dependent on platform information
		 * allowign to initialize different adc */
		if (adc121C027_init(&calibration, digi->pdata->i2c_adapter_num) == 0) {
			digi_dbg("ADC driver loaded...\n");
			break;
		}
		digi_dbg("Will try to load ADC driver again...\n");
		ssleep(10);
	}

	if (getCalibrationData(digi) == 0) {
		digi->rf->set_pwr = setNewPowerLevel;

		state = RESTART_STATE;

		digi_dbg("Starting autocalibration state machine.\n");
		do {
			int event;
			int timeout = 0;
			int expectedEvent = TIMED_OUT_EVENT;
			int nextState = RESTART_STATE;

			switch (state) {
			case RESTART_STATE:
				setInitialPowerLevel(digi,
						     CONVERT_TO_MDBM(digi->tx_power));
	            calibration.initialized = true;
				calibration.sampleCount = 0;
				nextState = COLLECT_SAMPLES_STATE;
				timeout = DEBOUNCE_DELAY;
				expectedEvent = TIMED_OUT_EVENT;
				break;
			case COLLECT_SAMPLES_STATE:
				digi->calibrationTxRate =
				    determineCalibrationTxRate(digi);
				startSampleCollection(digi);
				nextState = GOT_SAMPLE_STATE;
				timeout = SAMPLE_TIMEOUT;
				expectedEvent = TRANSMIT_DONE_EVENT;
				break;
			case GOT_SAMPLE_STATE:
				processSample(digi);
				if (calibration.sampleCount < MAX_SAMPLES) {
					nextState = COLLECT_SAMPLES_STATE;
					timeout = 0;
					break;
				}
				/* fall through is intended operation */
			case RECALIBRATE_STATE:
				recalibrate(digi);
				calibration.sampleCount = 0;
				digi->calibrationTxRate = USE_MAC80211_DATA_RATE;
				nextState = COLLECT_SAMPLES_STATE;
				timeout = RECALIBRATION_PERIOD;
				expectedEvent = TIMED_OUT_EVENT;
				break;
			default:
				digi_dbg("Unknown state %d\n", state);
				nextState = COLLECT_SAMPLES_STATE;
				timeout = RECALIBRATION_PERIOD;
				expectedEvent = TIMED_OUT_EVENT;
				break;
			}

			state = nextState;
			event = waitForEvent(timeout, expectedEvent);

			switch (event) {
			case SHUTDOWN_AUTOCALIBRATION_EVENT:
				digi_dbg("Received SHUTDOWN_AUTOCALIBRATION_EVENT\n");
				break;
			case RESTART_AUTOCALIBRATION_EVENT:
				digi_dbg("Received RESTART_AUTOCALIBRATION_EVENT\n");
				state = RESTART_STATE;
				break;
			case TRANSMIT_DONE_EVENT:
				break;
			case TIMED_OUT_EVENT:
				if (state == GOT_SAMPLE_STATE) {
					state = COLLECT_SAMPLES_STATE;
				}
			}
		} while (!kthread_should_stop());
	} else {
		printk(KERN_ERR
		       "\nWarning: Wireless interface calibration data is corrupted.\n");
		printk(KERN_ERR
		       "         The wireless interface will operate at a low power level.\n");
		while (!kthread_should_stop()) {
			ssleep(1);
		}
	}
	calibration.cops->adc_shutdown(&calibration);

	return 0;
}



/*
 * This routine is called at initialization to set up the Airoha calibration routines.
 */
void digiWifiInitCalibration(struct piper_priv *digi)
{

	calibration.events = 0;
	calibration.sampleCount = 0;
	calibration.initialized = false;
    calibration.nvramDataState = NV_NOT_READ;

	spin_lock_init(&calibration.lock);

	calibration.threadCB =
	    kthread_run(digiWifiCalibrationThreadEntry, digi,
			PIPER_DRIVER_NAME " - calibration");
}

int digiWifiCalibrationPowerIndex(struct piper_priv *piperp)
{
    if (calibration.initialized)
        return calibration.correctedPowerIndex;
    else
        return -1;
}


void digiWifiDeInitCalibration(struct piper_priv *digi)
{
	calibration.events = SHUTDOWN_AUTOCALIBRATION_EVENT;
	wake_up_interruptible(&waitQueue);
	kthread_stop(calibration.threadCB);
	calibration.initialized = false;
}

EXPORT_SYMBOL_GPL(digiWifiDeInitCalibration);
