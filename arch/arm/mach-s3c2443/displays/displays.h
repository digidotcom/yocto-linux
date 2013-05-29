/*
 * arch/arm/mach-s3c2443/displays/displays.h
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <mach/s3c2443-fb.h>

#if defined(CONFIG_CC9M2443_VGA)
#include "VGA.h"
#endif

#if defined(CONFIG_CC9M2443_LQ057Q3DC12I)
#include "LQ057Q3DC12I.h"
#endif

#if defined(CONFIG_CC9M2443_LQ064V3DG01)
#include "LQ064V3DG01.h"
#endif

#if defined(CONFIG_CC9M2443_LQ070Y3DG3B1)
#include "LQ070Y3DG3B1.h"
#endif

#if defined(CONFIG_CC9M2443_CUSTOM)
#include "CUSTOM.h"
#endif

/* List of supported displays */
struct s3c2443fb_display display_list[] = {
#if defined(CONFIG_CC9M2443_VGA)
	VGA_DISPLAY,
#endif
#if defined(CONFIG_CC9M2443_LQ057Q3DC12I)
	LQ057Q3DC12I_DISPLAY,
#endif
#if defined(CONFIG_CC9M2443_LQ064V3DG01)
	LQ064V3DG01_DISPLAY,
#endif
#if defined(CONFIG_CC9M2443_LQ070Y3DG3B1)
	LQ070Y3DG3B1_DISPLAY,
#endif
#if defined(CONFIG_CC9M2443_CUSTOM)
	CUSTOM_DISPLAY,
#endif
};


