#ifndef __ASM_S3C_HSMMC_H
#define __ASM_S3C_HSMMC_H

#ifndef __ASSEMBLY__

#include <linux/mmc/host.h>

struct s3c_hsmmc_cfg {
	u32 hwport;		/* hardware port number */
	u32 host_caps;		/* host capabilities */
	u32 bus_width;		/* host capabilities */

	void *base;		/* base address of host */
	ulong ctrl3[2];		/* preset value of control3 reg */
	char *clk_name[3];	/* HS-MMC has 3 clock source */
	void (*set_gpio)(void);


	/* GPIOs for additional features */
	unsigned int wprotect_invert : 1;
	unsigned int detect_invert : 1;   /* set => detect active high. */
	unsigned int gpio_detect;
	unsigned int gpio_wprotect;
	unsigned int gpio_led;
};

#endif /* __ASSEMBLY__ */

#endif /* __ASM_S3C_HSMMC_H */

