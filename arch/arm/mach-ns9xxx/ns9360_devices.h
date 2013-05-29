/*
 * arch/arm/mach-ns9xxx/ns9360_devices.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/i2c-ns9xxx.h>
#include <linux/init.h>
#include <linux/mtd/ccx9x_nand.h>
#include <mach/spi.h>

void __init ns9xxx_add_device_ns9360_usbh(void);
void __init ns9xxx_add_device_ns9360_wdt(void);
void __init ns9xxx_add_device_ns9360_rtc(void);
void __init ns9xxx_add_device_ns9360_eth(int gpio[], int gpio_nr, int func, u32 phy_mask);
void __init ns9xxx_add_device_ns9360_uarta(int gpio[], int gpio_nr, int func);
void __init ns9xxx_add_device_ns9360_uartb(int gpio[], int gpio_nr, int func);
void __init ns9xxx_add_device_ns9360_uartc(int gpio[], int gpio_nr, int func);
void __init ns9xxx_add_device_ns9360_uartd(int gpio[], int gpio_nr, int func);
void __init ns9xxx_add_device_ns9360_nand(struct ccx9x_nand_info *);
void __init ns9xxx_add_device_ns9360_i2c(struct plat_ns9xxx_i2c *);
void __init ns9xxx_add_device_ns9360_fb(int gpio[], int gpio_nr,
		int func[], int power);
void __init ns9xxx_add_device_ns9360_spi_porta(struct spi_ns9xxx_data *data);
void __init ns9xxx_add_device_ns9360_spi_portb(struct spi_ns9xxx_data *data);
void __init ns9xxx_add_device_ns9360_spi_portc(struct spi_ns9xxx_data *data);
void __init ns9xxx_add_device_ns9360_spi_portd(struct spi_ns9xxx_data *data);
