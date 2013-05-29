/*
 * arch/arm/mach-ns9xxx/include/mach/fim-firmware.h
 *
 * Copyright (C) 2006 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.0 $
 *  !Author:     Luis Galdos
 *  !Desc:
 *  !References:
 */


#ifndef _FIM_FIRMWARE_H
#define _FIM_FIRMWARE_H



#define FIM_NUM_HWA_CONF_REGS                           (14)
#define FIM_NS9215_MAX_INSTRUCTIONS                     (1024)


typedef enum {
	FIM_NS9215 = 0,
} fim_processors_t;

#define PROCESSOR_TYPE_VALID(t) 	(t == FIM_NS9215) /* || (t == ADD_NEW_ONE_HERE) */


typedef enum {
	FIM_FORMAT_0 = 0,
} fim_formats_t;

#define FORMAT_TYPE_VALID(t) 	(t == FIM_FORMAT_0) /* || (t == ADD_NEW_ONE_HERE) */

typedef enum {
	FIM_PROCESSOR_PIC0,
	FIM_PROCESSOR_PIC1
} fim_processor_index_t;

typedef enum {
	FIM_HW_ASSIST_MODE_NONE,
	FIM_HW_ASSIST_MODE_GENERIC,
	FIM_HW_ASSIST_MODE_CAN,
} fim_hw_assit_t;

typedef enum {
	FIM_CLK_DIV_2,
	FIM_CLK_DIV_4,
	FIM_CLK_DIV_8,
	FIM_CLK_DIV_16,
	FIM_CLK_DIV_32,
	FIM_CLK_DIV_64,
	FIM_CLK_DIV_128,
	FIM_CLK_DIV_256
} fim_output_clk_div_t;

typedef enum {
	FIM_SIGBUS_SIGNAL_0,
	FIM_SIGBUS_SIGNAL_1,
	FIM_SIGBUS_SIGNAL_2,
	FIM_SIGBUS_SIGNAL_3,
	FIM_SIGBUS_SIGNAL_4,
	FIM_SIGBUS_SIGNAL_5,
	FIM_SIGBUS_SIGNAL_6,
	FIM_SIGBUS_SIGNAL_7,
	FIM_SIGBUS_CONTROL_0,
	FIM_SIGBUS_CONTROL_1,
	FIM_SIGBUS_CONTROL_2,
	FIM_SIGBUS_CONTROL_3,
	FIM_SIGBUS_16_BIT_BUS,
	FIM_SIGBUS_24_BIT_BUS
} fim_signal_bus_t;



/*
 * The macro FIM_FIRMWARE_BUILDER is set by the build-system of the FIM-firmware
 * See the Makefile under ../firmware/Makefile for more infos.
 */
struct fim_program_t {
	fim_processors_t processor;
	fim_formats_t format;
	fim_hw_assit_t hw_mode;
	fim_output_clk_div_t clkdiv;
	unsigned int hwa_cfg[FIM_NUM_HWA_CONF_REGS];
	unsigned int length;
#ifndef FIM_FIRMWARE_BUILDER
	unsigned short data[FIM_NS9215_MAX_INSTRUCTIONS];
#endif
} __attribute__((packed, aligned));


#endif /* _FIM_FIRMWARE_H */



