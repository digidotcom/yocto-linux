/*
 * arch/arm/mach-ns9xxx/cc9p9215_devices.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

void __init ns9xxx_add_device_cc9p9215_eth(void);
void __init ns9xxx_add_device_cc9p9215_i2c(void);
void __init ns9xxx_add_device_cc9p9215_uarta(int gpio_start);
void __init ns9xxx_add_device_cc9p9215_uartb(int gpio_start);
void __init ns9xxx_add_device_cc9p9215_uartc(int gpio_start);
void __init ns9xxx_add_device_cc9p9215_uartd(int gpio_start);
void __init ns9xxx_add_device_cc9p9215_flash(void);
void __init ns9xxx_add_device_cc9p9215_spi(void);
void __init ns9xxx_add_device_cc9p9215_edt_diplay(void);

#define ns9xxx_add_device_cc9p9215_uarta_rxtx() \
	ns9xxx_add_device_cc9p9215_uarta(2)
#define ns9xxx_add_device_cc9p9215_uarta_ctsrtsrxtx() \
	ns9xxx_add_device_cc9p9215_uarta(4)
#define ns9xxx_add_device_cc9p9215_uarta_full() \
	ns9xxx_add_device_cc9p9215_uarta(8)
#define ns9xxx_add_device_cc9p9215_uartb_rxtx() \
	ns9xxx_add_device_cc9p9215_uartb(2)
#define ns9xxx_add_device_cc9p9215_uartb_ctsrtsrxtx() \
	ns9xxx_add_device_cc9p9215_uartb(4)
#define ns9xxx_add_device_cc9p9215_uartb_full() \
	ns9xxx_add_device_cc9p9215_uartb(8)
#define ns9xxx_add_device_cc9p9215_uartc_rxtx() \
	ns9xxx_add_device_cc9p9215_uartc(2)
#define ns9xxx_add_device_cc9p9215_uartc_ctsrtsrxtx() \
	ns9xxx_add_device_cc9p9215_uartc(4)
#define ns9xxx_add_device_cc9p9215_uartc_full() \
	ns9xxx_add_device_cc9p9215_uartc(8)
#define ns9xxx_add_device_cc9p9215_uartd_rxtx() \
	ns9xxx_add_device_cc9p9215_uartd(2)
#define ns9xxx_add_device_cc9p9215_uartd_ctsrtsrxtx() \
	ns9xxx_add_device_cc9p9215_uartd(4)
#define ns9xxx_add_device_cc9p9215_uartd_full() \
	ns9xxx_add_device_cc9p9215_uartd(8)
