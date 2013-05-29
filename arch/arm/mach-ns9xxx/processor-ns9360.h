/*
 * arch/arm/mach-ns9xxx/processor-ns9360.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __NS9XXX_PROCESSOR_NS9360_H
#define __NS9XXX_PROCESSOR_NS9360_H

#include <linux/init.h>

#include "clock.h"

void ns9360_reset(char mode);

unsigned long ns9360_systemclock(void) __attribute__((const));

unsigned long ns9360_cpuclock(void) __attribute__((const));

void __init ns9360_map_io(void);

extern struct sys_timer ns9360_timer;

void __init ns9360_init_machine(void);

#endif /* ifndef __NS9XXX_PROCESSOR_NS9360_H */
