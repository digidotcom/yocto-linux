/*
 * This file contains the code which performs automatic recalibration of the
 * Airoha transceiver.
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef AIROHA_CALIBRATION_H
#define AIROHA_CALIBRATION_H

#include "pipermain.h"


#define MAX_SAMPLES                     (3)

/*
 * Field values used for computing ABS values.
 */
enum {
	OUT_POWER,
	ADC_VAL,
	POWER_INDEX
};

#ifdef WANT_CAL_DEBUG
static const char *fieldDescription[] = {
	"OUT_POWER",
	"ADC_VAL",
	"POWER_INDEX"
};
#endif

/*
 * States for the auto calibration thread.
 */
enum {
	RESTART_STATE,
	COLLECT_SAMPLES_STATE,
	GOT_SAMPLE_STATE,
	RECALIBRATE_STATE
};

#ifdef WANT_CAL_DEBUG
static const char *stateText[] = {
	"RESTART_STATE",
	"COLLECT_SAMPLES_STATE",
	"GOT_SAMPLE_STATE",
	"RECALIBRATE_STATE"
};
#endif


/*
 * Slope types accepted by computeSlope().
 */
enum {
	POWER_INDEX_OVER_OUT_POWER,
	ADC_OVER_OUT_POWER,
	OUT_POWER_OVER_ADC,
	POWER_INDEX_OVER_ADC
};


typedef struct {
	unsigned rate;		/* rate packet transmitted at */
	unsigned int sample;	/* measured sample */
} sampleInfo_t;

enum NvramDataState {
    NV_NOT_READ,
    NV_OKAY,
    NV_CRC_FAILED,
    NV_INVALID_SIGNATURE,
    NV_CONVERTED_10
};

struct airohaCalibrationData {
	struct task_struct *threadCB;
	spinlock_t lock;
	volatile unsigned int events;
	unsigned int sampleCount;
	sampleInfo_t sample[MAX_SAMPLES];
	wcd_data_t *nvram;
	wcd_curve_t *curve;
	int slopeTimes1000;
	int adcSlopeTimes1000;
	int outPowerSlopeTimes1000;
	int powerIndexSlopeTimes1000;
	int expectedAdc;
	int powerIndex, correctedPowerIndex;
	wcd_point_t *p1;
	void *priv;
	struct calibration_ops *cops;
	enum NvramDataState nvramDataState;
	bool initialized;
};

struct calibration_ops {
	u16(*adc_read_peak) (struct airohaCalibrationData *);
	void (*adc_clear_peak) (struct airohaCalibrationData *);
	 u16(*adc_read_last_val) (struct airohaCalibrationData *);
	void (*adc_shutdown) (struct airohaCalibrationData *);
};


void digiWifiInitCalibration(struct piper_priv *digi);
void digiWifiDeInitCalibration(struct piper_priv *digi);
int digiWifiCalibrationPowerIndex(struct piper_priv *piperp);
void digiWifiCalibrationDumpNvram(struct piper_priv *piperp);
void digiWifiCalibrationRestartCalibration(struct piper_priv *digi);

#endif				/* AIROHA_CALIBRATION_H */
