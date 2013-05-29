/* linux/arch/arm/mach-s3c24xx/sleep-s3c2443.S
 *
 * $Id: sleep-s3c2443.S,v 1.3 2006/11/08 01:07:53 eyryu Exp $
 *
 * Copyright (c) 2006 Samsung Electronics
 *	Ryu Euiyoul <steven.ryu@samsung.com>
 *
 * S3C Power Manager (Suspend-To-RAM) support
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

@ #include <linux/config.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>


#if 0
#define ENABLE_GPIO_TOGGLE
#endif
	
	.text

/* s3c_cpu_suspend
 *
 * put the cpu into sleep mode
 *
 * entry:
 *	r0 = sleep save block
*/

ENTRY(s3c2443_cpu_suspend)
        @@ flush the caches to ensure everything is back out to
        @@ SDRAM before the core powers down

        bl      arm920_flush_kern_cache_all
	
	b	s3c_goto_sleep

	.align	5


 	/* 
	 * Before entering the sleep mode we must store the jump address in the
 	 * configuration register INFORM1
	 */	
s3c_goto_sleep:
        ldr     r0, =0xf4100040 @ =S3C_PWRMODE
        ldr     r1, =0x2BED
        str     r1, [r0]

@ _sleeping:
@ 	nop
@ 	nop
@ 	nop
@ 	b	_sleeping
