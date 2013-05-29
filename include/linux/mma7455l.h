#ifndef _LINUX_MMA7455L_H
#define _LINUX_MMA7455L_H

struct mma7455l_platform_data {
	/* Calibration offsets */
	s16 calibration_x;
	s16 calibration_y;
	s16 calibration_z;
};

#endif /* _LINUX_MMA7455L_H */
