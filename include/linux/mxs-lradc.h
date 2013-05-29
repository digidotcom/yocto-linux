/*
 * Copyright (C) 2012 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __MXS_LRADC_H
#define __MXS_LRADC_H

#include <linux/ioctl.h>

typedef struct {
	/*!
	 * channel or channels to be sampled.
	 */
	int channel;
	/*!
	 * holds up to 16 sampling results
	 */
	unsigned short result[16];
} t_adc_convert_param;

#define LRADC_CONVERT                _IOWR('p', 0xb0, int)

#endif /* __MXS_LRADC_H */
