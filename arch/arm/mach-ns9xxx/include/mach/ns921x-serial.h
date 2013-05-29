/*
 * include/linux/ns9360.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __NS921X_SERIAL_H__
#define __NS921X_SERIAL_H__


struct ns921x_uart_data {
	unsigned int gpios[8];
	unsigned int nr_gpios;
	unsigned int rtsen;	/* RTS for 485 transceiver control */
	unsigned int rtsinvert;	/* RTS invert polarity for 485 transceiver control */
};

#endif
