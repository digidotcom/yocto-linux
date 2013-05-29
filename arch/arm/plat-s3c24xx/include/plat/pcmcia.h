/* linux/include/asm-arm/arch-s3c2410/udc.h
 *
 * Copyright (c) 2005 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *  Changelog:
 *	14-Mar-2005	RTP	Created file
 *	02-Aug-2005	RTP	File rename
 *	07-Sep-2005	BJD	Minor cleanups, changed cmd to enum
 *	18-Jan-2007	HMW	Add per-platform vbus_draw function
*/

#ifndef __ASM_ARM_ARCH_PCMCIA_H
#define __ASM_ARM_ARCH_PCMCIA_H

struct s3c2443_pcmcia_pdata {
	unsigned int gpio_detect;
};

#endif /* __ASM_ARM_ARCH_PCMCIA_H */
