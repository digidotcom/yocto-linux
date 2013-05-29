/*
 * arch/arm/mach-ns9xxx/include/mach/system.h
 *
 * Copyright (C) 2006-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/proc-fns.h>
#include <mach/processor.h>

static inline void arch_idle(void)
{
	/*
	 * When a Camry (i.e. ns921x) executes
	 *
	 * 	mcr     p15, 0, r0, c7, c0, 4        @ Wait for interrupt
	 *
	 * the CPU is stopped, so are all timers.  This is not something I want
	 * to handle.  As the "wait for interrupt" instruction is part of
	 * cpu_arm926_do_idle, it's not called for it.
	 */
	if (!processor_is_ns921x())
		cpu_do_idle();
}

void ns921x_reset(char);
void ns9360_reset(char);

static inline void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_PROCESSOR_NS921X
	if (processor_is_ns921x())
		ns921x_reset(mode);
	else
#endif
#ifdef CONFIG_PROCESSOR_NS9360
	if (processor_is_ns9360())
		ns9360_reset(mode);
	else
#endif
		BUG();

	BUG();
}

#endif /* ifndef __ASM_ARCH_SYSTEM_H */
