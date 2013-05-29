/*
 *  "fusion_F04B" touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define fusion_F04B_VERSION			"V1.0"

#define fusion_F04B_MAX_FINGER			4

/* report info is used to store driver information for upper application to get them*/
#define COMMAND				1

#define PTFLAG_TIPSW			0x80
#define PTFLAG_INRANGE			0x40
#define PTFLAG_VALID			0x20
#define fusion_F04B_TOUCH_DOWN			(PTFLAG_TIPSW|PTFLAG_INRANGE|PTFLAG_VALID)
#define fusion_F04B_TOUCH_UP			PTFLAG_VALID

#define REPORT_TOUCH			0x01
#define REPORT_SENSITIVITY		0x02
#define REPORT_RAW_DATA			0x03
#define REPORT_BASELINE			0x04
#define REPORT_CMOD				0x05
#define CMD_RESPONSE			0X80
#define AP_MODE					0x64
#define BL_MODE					0xa0

#define MAX_PACKET_LEN			64


struct fusion_F04B_report_format{
	unsigned char pt_flag;
	unsigned char contactID;
	unsigned short x;  //[low byte][high byte]
	unsigned short y;  //[low byte][high byte]
	unsigned short z;   //[low byte][high byte]
	unsigned char width:3;
	unsigned char height:5;
}__attribute__((packed));


struct fusion_F04B_data {
	struct i2c_client *client;
	struct workqueue_struct	*workq;
	struct input_dev *input_dev;
	struct work_struct  work;
	struct completion fusion_F04B_done;
	int drv_cnt;
	int sen_cnt;
	unsigned char response_data[MAX_PACKET_LEN];
};
