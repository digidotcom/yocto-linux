/*
 * arch/arm/mach-ns9xxx/processor-ns921x.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __NS9XXX_PROCESSOR_NS921X_H
#define __NS9XXX_PROCESSOR_NS921X_H
#include <linux/init.h>

#include "clock.h"

#define NS921X_REFCLOCK 29491200

void ns921x_reset(char mode);

unsigned long ns921x_systemclock(void) __attribute__((const));

unsigned long ns921x_ahbclock(void) __attribute__((const));

void __init ns921x_map_io(void);

extern struct sys_timer ns921x_timer;

struct ns921x_sysclk {
	struct clk clk;
	u32 mask;
};
/*
 * only use ns921x_endisable_sysclock as callback for struct ns921x_sysclk's
 * because there is no additional locking to serialize access to SYS_CLOCK.
 */
int ns921x_endisable_sysclock(struct clk *clk, int enable);

void __init ns921x_init_machine(void);

/* irq-ns921x.c */
int ns921x_set_wake_irq(unsigned int irq, unsigned int on);

#endif /* ifndef __NS9XXX_PROCESSOR_NS921X_H */
