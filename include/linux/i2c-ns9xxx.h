/*
 * include/linux/i2c-ns9xxx.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _LINUX_I2C_NS9XXX_H
#define _LINUX_I2C_NS9XXX_H

struct plat_ns9xxx_i2c {
	unsigned int gpio_scl;
	unsigned int gpio_sda;
	unsigned int speed;
	void (*gpio_configuration_func)(void);
};

#endif /* ifndef _LINUX_I2C_NS9XXX_H */

