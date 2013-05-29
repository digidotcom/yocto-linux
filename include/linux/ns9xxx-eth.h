/*
 * include/linux/ns9xxx-eth.h
 *
 * Copyright (C) 2007 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _LINUX_NS9XXX_ETH_H
#define _LINUX_NS9XXX_ETH_H

struct plat_ns9xxx_eth {
	unsigned int irqrx;
	unsigned int irqtx;
	u32 phy_mask;
	unsigned int activityled;
};

#endif /* ifndef _LINUX_NS9XXX_ETH_H */
