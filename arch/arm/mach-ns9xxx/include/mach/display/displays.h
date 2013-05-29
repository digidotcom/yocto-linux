/*
 * arch/arm/mach-ns9xxx/displays/displays.h
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <mach/ns9360fb.h>

#if defined(CONFIG_NS9XXX_FB_VGA)
#include <mach/display/VGA.h>
#endif

#if defined(CONFIG_NS9XXX_FB_LQ057Q3DC12I)
#include <mach/display/LQ057Q3DC12I.h>
#endif

#if defined(CONFIG_NS9XXX_FB_LQ064V3DG01)
#include <mach/display/LQ064V3DG01.h>
#endif

#if defined(CONFIG_NS9XXX_FB_CUSTOM)
#include <mach/display/CUSTOM.h>
#endif

/* List of supported displays */
struct ns9360fb_display display_list[] = {
#if defined(CONFIG_NS9XXX_FB_VGA)
	VGA_DISPLAY,
#endif
#if defined(CONFIG_NS9XXX_FB_LQ057Q3DC12I)
	LQ057Q3DC12I_DISPLAY,
#endif
#if defined(CONFIG_NS9XXX_FB_LQ064V3DG01)
	LQ064V3DG01_DISPLAY,
#endif
#if defined(CONFIG_NS9XXX_FB_CUSTOM)
	CUSTOM_DISPLAY,
#endif
};


