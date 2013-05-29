/*
 * ad9389.h
 *
 * Copyright 2010 - Digi International, Inc. All Rights Reserved.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_VIDE0_AD9389_H
#define __LINUX_VIDE0_AD9389_H

enum hdmi_mode {
	MODE_AUTO,
	MODE_AUTO_STRING,
	MODE_STRING,
	MODE_FORCED,
	MODE_UNKNOWN,
};

struct ad9389_dev {
	u8			chiprev;
	struct mutex		irq_lock;
	struct i2c_client	*client;
	struct work_struct	work;
	struct timer_list	timer;
	struct i2c_client	*edid_ram;
	struct fb_info		*fbi;
	u8			*edid_data;
	int			dvi;
};

struct ad9389_pdata {
	int		dispif;
	int		fbidx;
	enum hdmi_mode	mode;
	void		*data;
	unsigned int	debounce_ms;
	unsigned char	edid_addr;

	/* function callbacks */
	int		(*hw_init)(struct ad9389_dev *);
	int		(*hw_deinit)(struct ad9389_dev *);
	void		(*disp_connected)(struct ad9389_dev *);
	void		(*disp_disconnected)(struct ad9389_dev *);
	void		(*vmode_to_modelist)(struct fb_videomode *, int, struct list_head *, struct fb_var_screeninfo *);
	void		(*vmode_to_var)(struct ad9389_dev *, struct fb_var_screeninfo *);
};

#endif	/* __LINUX_VIDE0_AD9389_H */
