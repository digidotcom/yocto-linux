/*
 * arch/arm/mach-mx5/displays/displays.h
 *
 * Copyright (C) 2009-2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCIMX5X_DISPLAYS_H__
#define __ASM_ARCH_MXC_CCIMX5X_DISPLAYS_H__

extern void gpio_video_active(int vif, u32 pad);
extern void gpio_video_inactive(int vif);
extern void gpio_vga_active(int interf);
extern void gpio_vga_inactive(int interf);

#ifdef CONFIG_MODULE_CCIMX51
#ifdef CONFIG_CCIMX5X_DISP0_RGB888
#define DISP0_PIX_FMT	IPU_PIX_FMT_RGB24
#else
#define DISP0_PIX_FMT	IPU_PIX_FMT_RGB666
#endif

#define DISP1_PIX_FMT	IPU_PIX_FMT_RGB666
#endif /* CONFIG_MODULE_CCIMX51 */

#ifdef CONFIG_MODULE_CCIMX53
/* The ConnectCore Wi-i.MX53 can select the pixel format per DISP interface */
#ifdef CONFIG_CCIMX5X_DISP0_RGB888
#define DISP0_PIX_FMT	IPU_PIX_FMT_RGB24
#elif defined(CONFIG_CCIMX5X_DISP0_RGB666)
#define DISP0_PIX_FMT	IPU_PIX_FMT_RGB666
#else
#define DISP0_PIX_FMT	IPU_PIX_FMT_RGB565
#endif

#ifdef CONFIG_CCIMX5X_DISP1_RGB888
#define DISP1_PIX_FMT	IPU_PIX_FMT_RGB24
#elif defined(CONFIG_CCIMX5X_DISP1_RGB666)
#define DISP1_PIX_FMT	IPU_PIX_FMT_RGB666
#else
#define DISP1_PIX_FMT	IPU_PIX_FMT_RGB565
#endif
#endif /* CONFIG_MODULE_CCIMX53 */

#include "hdmi_ad9389.h"
#include "vga.h"
#include "lcd.h"

#endif /* __ASM_ARCH_MXC_CCIMX5X_DISPLAYS_H__ */
