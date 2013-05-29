#ifndef __CCX9X_NAND_H__
#define __CCX9X_NAND_H__

struct ccx9x_nand_info {
	unsigned int addr_offset;
	unsigned int cmd_offset;
	unsigned int delay;
	unsigned int busy_pin;
};

#endif
