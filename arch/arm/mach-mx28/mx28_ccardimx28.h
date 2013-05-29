/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2011 Digi International, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARM_MODULE_CCARDIMX28_H
#define __ASM_ARM_MODULE_CCARDIMX28_H

extern void __init mx28_ccardimx28_pins_init(void);
extern int mx28_ccardimx28_enet_gpio_init(void);
extern int mx28_ccardimx28_eth_pins_init(void);
extern int mx28_ccardimx28_eth_pins_deinit(void);

extern void ccardimx28_set_hwid(u32 cust0, u32 cust1);
extern unsigned char get_hwid_hv(void);
extern int ccardimx28_create_sysfs_entries(void);

#define CCARDIMX28_SYSFS_FNAME	"ccardimx28"
#define CCARDIMX28_SYSFS_MNAME	"machine"

#endif /* __ASM_ARM_MODULE_CCARDIMX28_H */
