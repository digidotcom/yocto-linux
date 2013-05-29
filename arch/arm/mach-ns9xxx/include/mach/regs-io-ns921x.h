/*
 * arch/arm/mach-ns9xxx/include/mach/regs-io-ns921x.h
 *
 * Copyright (C) 2007-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __ASM_ARCH_REGSIONS921X_H
#define __ASM_ARCH_REGSIONS921X_H

#include <mach/hardware.h>

#define __NS921X_IO_GPIOCONF_SHIFT(m)			(((m) & 3) << 3)

/* I/O Control Module */

/* NOTE: the first GPIO has number #0, this is different from NS9360 which
 * starts at #1 */

/* GPIO Configuration Register */
#define NS921X_IO_GPIOCONFx(x)	__REG2(0xa0902000, (x))

#define NS921X_IO_GPIOCONFx_FUNC(m)	__REGBITS_SHIFT(5, 3, __NS921X_IO_GPIOCONF_SHIFT(m))
#define NS921X_IO_GPIOCONFx_DIR(m)	__REGBIT(2 + __NS921X_IO_GPIOCONF_SHIFT(m))
#define NS921X_IO_GPIOCONFx_DIR_IN(m)		__REGVAL(NS921X_IO_GPIOCONF_DIR(m), 0)
#define NS921X_IO_GPIOCONFx_DIR_OUT(m)		__REGVAL(NS921X_IO_GPIOCONF_DIR(m), 1)
#define NS921X_IO_GPIOCONFx_INV(m)	__REGBIT(1 + __NS921X_IO_GPIOCONF_SHIFT(m))
#define NS921X_IO_GPIOCONFx_INV_OFF(m)		__REGVAL(NS921X_IO_GPIOCONF_INV(m), 0)
#define NS921X_IO_GPIOCONFx_INV_ON(m)		__REGVAL(NS921X_IO_GPIOCONF_INV(m), 1)
#define NS921X_IO_GPIOCONFx_PUEN(m)	__REGBIT(__NS921X_IO_GPIOCONF_SHIFT(m))
#define NS921X_IO_GPIOCONFx_PUEN_EN(m)		__REGVAL(NS921X_IO_GPIOCONF_PUEN(m), 0)
#define NS921X_IO_GPIOCONFx_PUEN_DIS(m)	__REGVAL(NS921X_IO_GPIOCONF_PUEN(m), 1)

#define NS921X_IO_GPIOCTRLx(x)	__REG2(0xa090206c, (x))

#define NS921X_IO_GPIOSTATx(x)	__REG2(0xa090207c, (x))

#endif /* ifndef __ASM_ARCH_REGSSYSNS921X_H */
