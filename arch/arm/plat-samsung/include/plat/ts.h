/* arch/arm/plat-samsung/include/plat/ts.h
 *
 * Copyright (c) 2005 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_TS_H
#define __ASM_ARM_TS_H

struct s3c2410_ts_mach_info {
       int             delay;
       int             presc;
       int             oversampling_shift;
	void    (*cfg_gpio)(struct platform_device *dev);
};

/* Digi driver */
struct s3c_ts_mach_info {
       int             delay;
       int             presc;
       int             oversampling_shift;
       int             probes;
       int             trigger_ms;
       unsigned long   xmin;
       unsigned long   xmax;
       unsigned long   ymin;
       unsigned long   ymax;
};

void __init set_s3c_ts_info(struct s3c_ts_mach_info *hard_s3c_ts_info);

extern void s3c24xx_ts_set_platdata(struct s3c2410_ts_mach_info *);

/* defined by architecture to configure gpio */
extern void s3c24xx_ts_cfg_gpio(struct platform_device *dev);

#endif /* __ASM_ARM_TS_H */
