#ifndef __LINUX_PWM_LED_H
#define __LINUX_PWM_LED_H

/*
 * include/linux/pwm-led.h
 *
 * Copyright (C) 2008 Bill Gatliff
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

struct led_info;
struct pwm_channel_config;

struct pwm_led_platform_data {
	const char *bus_id;
	int chan;
	struct pwm_channel_config *config;
	struct led_info *led_info;
};

#endif /* __LINUX_PWM_LED_H */

