/*
 * arch/arm/mach-ns9xxx/ns921x_devices.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/i2c-ns9xxx.h>
#include <linux/init.h>
#include <linux/mtd/physmap.h>
#include <mach/spi.h>

#include "clock.h"

void __init ns9xxx_add_device_ns921x_wdt(void);
void __init ns9xxx_add_device_ns921x_eth(struct clk *phyclk, u32 phy_mask,
		int gpio[], int func[], int dir[], int num);
void __init ns9xxx_add_device_ns921x_i2c(struct plat_ns9xxx_i2c *);
void __init ns9xxx_add_device_ns921x_uarta(int gpio_start,
		int gpio_nr, int func);
void __init ns9xxx_add_device_ns921x_uartb(int gpio_start,
		int gpio_nr, int func);
void __init ns9xxx_add_device_ns921x_uartc(int gpio_start,
		int gpio_nr, int func);
void __init ns9xxx_add_device_ns921x_uartd(int gpio_start,
		int gpio_nr, int func);
void __init ns9xxx_add_device_ns921x_flash(
		struct physmap_flash_data *flash_data);
void __init ns9xxx_add_device_ns921x_spi(struct spi_ns9xxx_data *data);
int __init ns921x_extgpio_pm_wakeup_init(unsigned int gpio);
void __init ns9xxx_add_device_ns921x_fims(void);
void __init ns9xxx_add_device_ns921x_aes(void);
