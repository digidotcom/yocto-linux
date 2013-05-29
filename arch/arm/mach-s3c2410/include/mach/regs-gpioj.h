/* arch/arm/mach-s3c2410/include/mach/regs-gpioj.h
 *
 * Copyright (c) 2004 Simtec Electronics <linux@simtec.co.uk>
 *		      http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2440 GPIO J register definitions
*/


#ifndef __ASM_ARCH_REGS_GPIOJ_H
#define __ASM_ARCH_REGS_GPIOJ_H "gpioj"

/* Port J consists of 13 GPIO/Camera pins
 *
 * GPJCON has 2 bits for each of the input pins on port F
 *   00 = 0 input, 1 output, 2 Camera
 *
 * pull up works like all other ports.
*/

#define S3C2413_GPJCON		S3C2410_GPIOREG(0x80)
#define S3C2413_GPJDAT		S3C2410_GPIOREG(0x84)
#define S3C2413_GPJUP		S3C2410_GPIOREG(0x88)
#define S3C2413_GPJSLPCON	S3C2410_GPIOREG(0x8C)

#define S3C2440_GPJ0_OUTP       (0x01 << 0)
#define S3C2440_GPJ0_CAMDATA0   (0x02 << 0)

#define S3C2440_GPJ1_OUTP       (0x01 << 2)
#define S3C2440_GPJ1_CAMDATA1   (0x02 << 2)

#define S3C2440_GPJ2_OUTP       (0x01 << 4)
#define S3C2440_GPJ2_CAMDATA2   (0x02 << 4)

#define S3C2440_GPJ3_OUTP       (0x01 << 6)
#define S3C2440_GPJ3_CAMDATA3   (0x02 << 6)

#define S3C2440_GPJ4_OUTP       (0x01 << 8)
#define S3C2440_GPJ4_CAMDATA4   (0x02 << 8)

#define S3C2440_GPJ5_OUTP       (0x01 << 10)
#define S3C2440_GPJ5_CAMDATA5   (0x02 << 10)

#define S3C2440_GPJ6_OUTP       (0x01 << 12)
#define S3C2440_GPJ6_CAMDATA6   (0x02 << 12)

#define S3C2440_GPJ7_OUTP       (0x01 << 14)
#define S3C2440_GPJ7_CAMDATA7   (0x02 << 14)

#define S3C2440_GPJ8_OUTP       (0x01 << 16)
#define S3C2440_GPJ8_CAMPCLK    (0x02 << 16)

#define S3C2440_GPJ9_OUTP       (0x01 << 18)
#define S3C2440_GPJ9_CAMVSYNC   (0x02 << 18)

#define S3C2440_GPJ10_OUTP      (0x01 << 20)
#define S3C2440_GPJ10_CAMHREF   (0x02 << 20)

#define S3C2440_GPJ11_OUTP      (0x01 << 22)
#define S3C2440_GPJ11_CAMCLKOUT (0x02 << 22)

#define S3C2440_GPJ12_OUTP      (0x01 << 24)
#define S3C2440_GPJ12_CAMRESET  (0x02 << 24)

#define S3C2440_GPJ13_INP       (0x00 << 26)
#define S3C2440_GPJ13_OUTP      (0x01 << 26)
#define S3C2440_GPJ13_SD0LED    (0x02 << 26)

#define S3C2440_GPJ14_INP       (0x00 << 28)
#define S3C2440_GPJ14_OUTP      (0x01 << 28)
#define S3C2440_GPJ14_SD0CD     (0x02 << 28)

#define S3C2440_GPJ15_INP       (0x00 << 30)
#define S3C2440_GPJ15_OUTP      (0x01 << 30)
#define S3C2440_GPJ15_SD0WP     (0x02 << 30)


/* Port L consists of 14 GPIO
 *
 * GPLCON has 2 bits for each of the input pins on port K
 *   00 = 0 input, 1 output, 2 SD
 *
 * pull down works like all other ports.
*/
#define S3C2443_GPL0_OUTP       (0x1 << 0)
#define S3C2443_GPL0_SD0DAT0    (0x2 << 0)

#define S3C2443_GPL1_OUTP       (0x1 << 2)
#define S3C2443_GPL1_SD0DAT1    (0x2 << 2)

#define S3C2443_GPL2_OUTP       (0x1 << 4)
#define S3C2443_GPL2_SD0DAT2    (0x2 << 4)

#define S3C2443_GPL3_OUTP	(0x1 << 6)
#define S3C2443_GPL3_SD0DAT3	(0x2 << 6)

#define S3C2443_GPL4_OUTP	(0x1 << 8)
#define S3C2443_GPL4_SD0DAT4	(0x2 << 8)

#define S3C2443_GPL5_OUTP	(0x1 << 10)
#define S3C2443_GPL5_SD0DAT5	(0x2 << 10)

#define S3C2443_GPL6_OUTP	(0x1 << 12)
#define S3C2443_GPL6_SD0DAT6	(0x2 << 12)

#define S3C2443_GPL7_OUTP	(0x1 << 14)
#define S3C2443_GPL7_SD0DAT7	(0x2 << 14)

#define S3C2443_GPL8_OUTP	(0x1 << 16)
#define S3C2443_GPL8_SD0CMD	(0x2 << 16)

#define S3C2443_GPL9_OUTP	(0x1 << 18)
#define S3C2443_GPL9_SD0CLK	(0x2 << 18)

#define S3C2443_GPL10_OUTP	(0x1 << 20)
#define S3C2443_GPL10_SPICLK1	(0x2 << 20)

#define S3C2443_GPL11_OUTP	(0x1 << 22)
#define S3C2443_GPL11_SPIMOSI1	(0x2 << 22)

#define S3C2443_GPL12_OUTP	(0x1 << 24)
#define S3C2443_GPL12_SPIMISO1	(0x2 << 24)

#define S3C2443_GPL13_OUTP	(0x1 << 26)
#define S3C2443_GPL13_nSS0	(0x2 << 26)

#define S3C2443_GPL14_OUTP	(0x1 << 28)
#define S3C2443_GPL14_nSS1	(0x2 << 28)

/* Port M */
#define S3C2443_GPMCON          S3C2410_GPIOREG(0x100)
#define S3C2443_GPMDAT          S3C2410_GPIOREG(0x104)
#define S3C2443_GPMUDP          S3C2410_GPIOREG(0x108)
#define S3C2443_GPMDN           S3C2410_GPIOREG(0x108)

#endif	/* __ASM_ARCH_REGS_GPIOJ_H */

