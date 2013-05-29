/*
 * drivers/pwm/mxs-pwm.h
 *
 * Copyright (C) 2012 Digi International Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __MXS_PWM_HEADER_H
#define __MXS_PWM_HEADER_H

/* This is the data for the PWM channels */
struct mxs_pwm_channel {
	int		channel;

	/* Additional channel configuration variables ... */
};

/* */
struct mxs_pwm_pdata {
	unsigned int number_channels;
	struct mxs_pwm_channel *channels;
};

#endif