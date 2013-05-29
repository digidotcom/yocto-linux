/**
* Copyright (C) 2011 Touch Revolution Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/errno.h>

#include "fusion_F04B.h"


static struct fusion_F04B_data fusion_F04B;


#if COMMAND

struct fusion_F04B_command{
	int retval;
	unsigned char command;
	unsigned char cmd_response[8];
	struct kobject fusion_F04B_kobj;
};

static struct fusion_F04B_command test_cmd =
{
	.command = 'h',
};


static ssize_t fusion_F04B_show(struct kobject *kobject, struct attribute *attr,char *buf);
static ssize_t fusion_F04B_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count);
static void fusion_F04B_release(struct kobject *kobj);
static int fusion_F04B_get_res(unsigned short *x, unsigned short *y);
static int get_firmware_version(unsigned char *version);
static int fusion_F04B_get_fwinfo(unsigned char *buf, int size);
static int reset(void);
static int start_report(void);
static int stop_report(void);


static struct attribute fusion_F04B_attr =
{
	.name = "report_info",
	.mode = S_IRWXUGO,
};

static struct attribute *def_attrs[] = {
	&fusion_F04B_attr,
	NULL,
};

static struct sysfs_ops fusion_F04B_sysops =
{
	.show = fusion_F04B_show,
	.store = fusion_F04B_store,
};

static struct kobj_type fusion_F04B_ktype =
{
	.release = fusion_F04B_release,
	.sysfs_ops = &fusion_F04B_sysops,
	.default_attrs = def_attrs,
};

/*fusion_F04B touch command*/
static int write_user_register(unsigned char *reg1, unsigned char *reg2)
{
	int ret;
	unsigned char user_register[4] = {0x83, 0x21, *reg1, *reg2};

	ret = i2c_master_send(fusion_F04B.client, user_register, sizeof(user_register));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send write user register command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int read_user_register(unsigned char *reg1, unsigned char *reg2)
{
	int ret;
	unsigned char read_user_register[2] = {0x81, 0x31};

	ret = i2c_master_send(fusion_F04B.client, read_user_register, sizeof(read_user_register));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send read user register command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	*reg1 = fusion_F04B.response_data[1];
	*reg2 = fusion_F04B.response_data[2];

	return 0;
}

static int set_report_range(unsigned short *left, unsigned short *top,
			unsigned short *right, unsigned short *bottom)
{
	int ret;
	unsigned char report_range[10] = {0x89, 0x23, (unsigned char)(*left & 0x00ff),
		(unsigned char)((*left & 0xff00) >> 8), (unsigned char)(*top & 0x00ff),
		(unsigned char)((*top & 0xff00) >> 8), (unsigned char)(*right & 0x00ff),
		(unsigned char)((*right & 0xff00) >> 8), (unsigned char)(*bottom & 0x00ff),
		(unsigned char)((*bottom & 0xff00) >> 8)};

	ret = i2c_master_send(fusion_F04B.client, report_range, sizeof(report_range));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send set report range command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int get_report_range(unsigned short *left, unsigned short *top,
			unsigned short *right, unsigned short *bottom)
{
	int ret;
	unsigned char temp[8];
	unsigned char get_report_range[2] = {0x81, 0x33};

	ret = i2c_master_send(fusion_F04B.client, get_report_range, sizeof(get_report_range));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get report range command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(temp, &(fusion_F04B.response_data[1]), 8);

	*left = temp[0];
	*left |= (unsigned short)temp[1] << 8;
	*top = temp[2];
	*top |= (unsigned short)temp[3] << 8;
	*right = temp[4];
	*right |= (unsigned short)temp[5] << 8;
	*bottom = temp[6];
	*bottom |= (unsigned short)temp[7] << 8;

	return 0;
}

static int set_orientation(unsigned char *i)
{
	int ret;
	unsigned char set_orientation[3] = {0x82, 0x25, *i};

	if((*i > 7) || (*i < 0))
		*i = 1;//default value

	ret = i2c_master_send(fusion_F04B.client, set_orientation, sizeof(set_orientation));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send set orientation command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int get_orientation(unsigned char *orientation)
{
	int ret;
	unsigned char get_orientation[2] = {0x81, 0x35};

	ret = i2c_master_send(fusion_F04B.client, get_orientation, sizeof(get_orientation));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get orientation command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(orientation, &(fusion_F04B.response_data[1]), 1);

	return 0;
}

static int get_physical_size(unsigned short *x_size, unsigned short *y_size)
{
	int ret;
	unsigned char temp[4];
	unsigned char get_physical_size[2] = {0x81, 0x36};

	ret = i2c_master_send(fusion_F04B.client, get_physical_size, sizeof(get_physical_size));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get physical size command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(temp, &(fusion_F04B.response_data[1]), 4);

	*x_size = temp[0];
	*x_size |= (unsigned char)temp[1] << 8;
	*y_size = temp[2];
	*y_size |= (unsigned char)temp[3] << 8;

	return 0;
}

static int set_report_count(unsigned char *count)
{
	int ret;
	unsigned char set_report_count[3] = {0x82, 0x27, *count};

	if((*count > 4) || (*count < 0))
	{
		printk("fusion_F04B touch panel support max 4 points\n");
		*count = 2;//default report two points
	}

	ret = i2c_master_send(fusion_F04B.client, set_report_count, sizeof(set_report_count));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send set report count command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int get_report_count(unsigned char *count)
{
	int ret;
	unsigned char get_report_count[3] = {0x81, 0x37, *count};

	ret = i2c_master_send(fusion_F04B.client, get_report_count, sizeof(get_report_count));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get report count command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(count, &(fusion_F04B.response_data[1]), 1);

	if((*count > 4) || (*count < 0))
	{
		printk("get report count is error:count=%d\n", *count);
		return -1;
	}

	return 0;
}

static int get_report_type(unsigned char *type)
{
	int ret;
	unsigned char get_report_type[2] = {0x81, 0x39};

	ret = i2c_master_send(fusion_F04B.client, get_report_type, sizeof(get_report_type));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get report type command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(type, &(fusion_F04B.response_data[1]), 1);

	return 0;
}

static int modify_cmod(unsigned char *addr)
{
	int ret;
	unsigned char modify_cmod[2] = {0x81, 0x10};

	ret = i2c_master_send(fusion_F04B.client, modify_cmod, sizeof(modify_cmod));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send modify cmod command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(addr, &(fusion_F04B.response_data[1]), 2);

	return 0;
}

static int enter_isp(void)
{
	int ret;
	unsigned char enter_isp[2] = {0x81, 0x12};

	ret = i2c_master_send(fusion_F04B.client, enter_isp, sizeof(enter_isp));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send enter isp command error\n");
		return -1;
	}

	mdelay(500);

	return 0;
}

static int exit_isp(void)
{
	int ret;
	unsigned char exit_isp[2] = {0x81, 0x8f};

	ret = i2c_master_send(fusion_F04B.client, exit_isp, sizeof(exit_isp));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send exit isp command error\n");
		return -1;
	}

	mdelay(500);

	return 0;
}

static int save_power_mode(void)
{
	int ret;
	unsigned char save_power_mode[2] = {0x81, 0x13};

	ret = i2c_master_send(fusion_F04B.client, save_power_mode, sizeof(save_power_mode));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send save power mode command error\n");
		return -1;
	}

	return 0;
}

static int get_debug_info(unsigned char flag)
{
	int ret;
	unsigned char debug_info[3] = {0x82, 0x29, flag};

	ret = i2c_master_send(fusion_F04B.client, debug_info, sizeof(debug_info));
	if(ret < 0)
	{
		printk(KERN_ERR "get debug info command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static void fusion_F04B_release(struct kobject *kobj)
{
	return;
}

static ssize_t fusion_F04B_show(struct kobject *kobject, struct attribute *attr,char *buf)
{
	switch(test_cmd.command)
	{
		case 'a':
			if(test_cmd.retval < 0)
				return sprintf(buf, "reset touch panel failed\n");

			return sprintf(buf, "reset touch panel successfully\n");
		case 'b':
			if(test_cmd.retval < 0)
				return sprintf(buf, "write user register failed\n");

			return sprintf(buf, "write user register\n");
		case 'c':
			if(test_cmd.retval < 0)
				return sprintf(buf, "write user register failed\n");

			return sprintf(buf,"user register value: %d, %d\n", test_cmd.cmd_response[0], test_cmd.cmd_response[1]);
		case 'd':
			if(test_cmd.retval < 0)
				return sprintf(buf, "set report area failed\n");

			return sprintf(buf, "set report area successfully\n");
		case 'e':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get report area failed\n");

			return sprintf(buf,"report area is: left=%d, top=%d, right=%d, bottom=%d\n",
				((unsigned short)test_cmd.cmd_response[1] << 8) | test_cmd.cmd_response[0],
				((unsigned short)test_cmd.cmd_response[3] << 8) | test_cmd.cmd_response[2],
				((unsigned short)test_cmd.cmd_response[5] << 8) | test_cmd.cmd_response[4],
				((unsigned short)test_cmd.cmd_response[7] << 8) | test_cmd.cmd_response[6]);
		case 'f':
			if(test_cmd.retval < 0)
				return sprintf(buf, "set orientation failed\n");

			return sprintf(buf,"set orientation successfully\n");
		case 'g':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get orientation failed\n");

			return sprintf(buf,"orientation value is: %d\n", test_cmd.cmd_response[0]);
		case 'i':
			if(test_cmd.retval < 0)
				return sprintf(buf, "set report points failed\n");

			return sprintf(buf, "set report points successfully\n");
		case 'j':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get report points failed\n");

			return sprintf(buf,"report points:%d(points)\n", test_cmd.cmd_response[0]);
		case 'k':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get resolution failed\n");

			return sprintf(buf,"resolution: x=%d, y=%d\n", ((unsigned short)test_cmd.cmd_response[1] << 8) | test_cmd.cmd_response[0],
				((unsigned short)test_cmd.cmd_response[3] << 8) | test_cmd.cmd_response[2]);
		case 'l':
			return sprintf(buf,"firmware information:%02x-%02x-%02x-%02x-%02x-%02x\n",
				test_cmd.cmd_response[0], test_cmd.cmd_response[1], test_cmd.cmd_response[2], test_cmd.cmd_response[3],
				test_cmd.cmd_response[4], test_cmd.cmd_response[5]);

		case 'n':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get report type failed\n");

			return sprintf(buf,"report type: %d\n", test_cmd.cmd_response[0]);
		case 'o':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get report type failed\n");

			return sprintf(buf,"firmware version: ver_maj=%02x ver_min=%02x, revise=%02x\n", test_cmd.cmd_response[0], test_cmd.cmd_response[1],
				test_cmd.cmd_response[2]);
		case 'p':
			if(test_cmd.retval < 0)
				return sprintf(buf, "CMOD parameter calibration failed\n");
			return sprintf(buf,"CMOD parameter addr: 0x%04x\n", ((unsigned short)test_cmd.cmd_response[0] << 8) | test_cmd.cmd_response[1]);
		case 'q':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get debug info failed\n");

			return sprintf(buf, "get debug info successfully\n");
		case 'r':
			if(test_cmd.retval < 0)
				return sprintf(buf, "enter isp mode failed\n");

			return sprintf(buf, "enter isp mode successfully\n");
		case 's':
			if(test_cmd.retval < 0)
				return sprintf(buf, "exit isp mode failed\n");

			return sprintf(buf, "exit isp mode successfully\n");
		case 't':
			if(test_cmd.retval < 0)
				return sprintf(buf, "enter save power mode failed\n");

			return sprintf(buf, "enter save power mode successfully\n");
		case 'u':
			if(test_cmd.retval < 0)
				return sprintf(buf, "get physical size failed\n");

			return sprintf(buf,"physical size: x=%d, y=%d\n", ((unsigned short)test_cmd.cmd_response[1] << 8) | test_cmd.cmd_response[0],
				((unsigned short)test_cmd.cmd_response[3] << 8) | test_cmd.cmd_response[2]);
		case 'v':
			if(test_cmd.retval < 0)
				return sprintf(buf, "start report touch data failed\n");

			return sprintf(buf, "start report touch data successfully\n");
		case 'w':
			if(test_cmd.retval < 0)
				return sprintf(buf, "stop report touch data failed\n");

			return sprintf(buf, "stop report touch data successfully\n");
		case 'h':
			printk("Usage:\n");
			printk("	command [par1 par2 par3 par4]\n");
			printk("Command:\n");
			printk(" a: 							reset touch panel\n");
			printk(" b: <dat1> <dat2> 				write user register\n");
			printk(" c: 							read user register\n");
			printk(" d: <par1> <par2> <par3> <par4> set report area(left,top,right,bottom)\n");
			printk(" e: 							get report area\n");
			printk(" f: <par>						set coordinate orientation(1,2,3,4,5,6,7)\n");
			printk(" g: 							get coordinate orientation\n");
			printk(" i: <par>						set report point count(1,2)\n");
			printk(" j: 							get report point count\n");
			printk(" k: 							get touch panel resolution\n");
			printk(" l: 							get firmware information\n");
			printk(" n: 							get report type\n");
			printk(" o: 							get firmware version\n");
			printk(" p: 							CMOD calibration\n");
			printk(" q: <par> 						get debug info(2,3,4,5)\n");
			printk(" r: 							enter isp mode\n");
			printk(" s: 							exit isp mode\n");
			printk(" t: 							enter save power mode\n");
			printk(" u: 							get physical size\n");
			printk(" v: 							start report touch data\n");
			printk(" w: 							stop report touch data\n");
			break;
		default:
				return sprintf(buf, "error command !\n");
	}

	return 0;
}

static ssize_t fusion_F04B_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count)
{
	int para[4];
	int ret = 0;

	sscanf(buf, "%c %d %d %d %d",
		&(test_cmd.command), &para[0], &para[1], &para[2], &para[3]);

	printk(KERN_DEBUG "cmd:%c, para:%d %d %d %d\n",
		test_cmd.command, para[0], para[1], para[2], para[3]);

	switch(test_cmd.command)
	{
		case 'a':
			ret = reset();
			break;
		case 'b':
			ret = write_user_register((unsigned char *)&para[0], (unsigned char *)&para[1]);
			break;
		case 'c':
			ret = read_user_register(&test_cmd.cmd_response[0], &test_cmd.cmd_response[1]);
			break;
		case 'd':
			ret = set_report_range((unsigned short *)&para[0], (unsigned short *)&para[1],
				(unsigned short *)&para[2], (unsigned short *)&para[3]);
			break;
		case 'e':
			ret = get_report_range((unsigned short *)&test_cmd.cmd_response[0], (unsigned short *)&test_cmd.cmd_response[2],
				(unsigned short *)&test_cmd.cmd_response[4], (unsigned short *)&test_cmd.cmd_response[6]);
			break;
		case 'f':
			ret = set_orientation((unsigned char *)&para[0]);
			break;
		case 'g':
			ret = get_orientation(&test_cmd.cmd_response[0]);
			break;
		case 'i':
			ret = set_report_count((unsigned char *)&para[0]);
			break;
		case 'j':
			ret = get_report_count(&test_cmd.cmd_response[0]);
			break;
		case 'k':
			ret = fusion_F04B_get_res((unsigned short *)&test_cmd.cmd_response[0], (unsigned short *)&test_cmd.cmd_response[2]);
			break;
		case 'l':
			ret = fusion_F04B_get_fwinfo(&test_cmd.cmd_response[0], 6);
			break;
		case 'n':
			ret = get_report_type(&test_cmd.cmd_response[0]);
			break;
		case 'o':
			ret = get_firmware_version(&test_cmd.cmd_response[0]);
			break;
		case 'p':
			ret = modify_cmod(&test_cmd.cmd_response[0]);
			break;
		case 'q':
			ret = get_debug_info((unsigned char)para[0]);
			break;
		case 'r':
			ret = enter_isp();
			break;
		case 's':
			ret = exit_isp();
			break;
		case 't':
			ret = save_power_mode();
			break;
		case 'u':
			ret = get_physical_size((unsigned short *)&test_cmd.cmd_response[0], (unsigned short *)&test_cmd.cmd_response[2]);
		case 'v':
			ret = start_report();
			break;
		case 'w':
			ret = stop_report();
			break;
		case 'h':
			printk("help information\n");
			break;
		default:
			break;
	}

	if(ret < 0)
	{
		test_cmd.retval = -1;
		return test_cmd.retval;
	}
	test_cmd.retval = 0;
	return count;
}

#endif

static int reset(void)
{
	int ret;
	unsigned char reset_cmd[2] = {0x81, 0x11};

	ret = i2c_master_send(fusion_F04B.client, reset_cmd, sizeof(reset_cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send reset cmd command error\n");
		return -1;
	}

	mdelay(2000);

	return 0;
}

static int start_report(void)
{
	int ret;
	unsigned char start_cmd[3] = {0x82, 0x29, 0x01};

	ret = i2c_master_send(fusion_F04B.client, start_cmd, sizeof(start_cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send start report command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int stop_report(void)
{
	int ret;
	unsigned char stop_cmd[3] = {0x82, 0x29, 0xff};

	ret = i2c_master_send(fusion_F04B.client, stop_cmd, sizeof(stop_cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send stop report command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	return 0;
}

static int get_firmware_version(unsigned char *version)
{
	int ret;
	unsigned char get_firmware_version[2] = {0x81, 0x3a};

	ret = i2c_master_send(fusion_F04B.client, get_firmware_version, sizeof(get_firmware_version));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get firmware template version command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(version, &(fusion_F04B.response_data[1]), 3);

	return 0;
}

static int fusion_F04B_get_fwinfo(unsigned char *buf, int size)
{
	int ret;
	unsigned char get_fwinfo_cmd[2] = {0x81, 0x3b};

	ret = i2c_master_send(fusion_F04B.client, get_fwinfo_cmd, sizeof(get_fwinfo_cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get product fwver command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(buf, &(fusion_F04B.response_data[1]), size > 6 ? 6 : size);

	return 0;
}

static int fusion_F04B_get_res(unsigned short *x, unsigned short *y)
{
	int ret;
	unsigned char get_res_cmd[2] = {0x81, 0x34};
	unsigned char rec_buf[6];

	ret = i2c_master_send(fusion_F04B.client, get_res_cmd, sizeof(get_res_cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "fusion_F04B send get_res command error\n");
		return -1;
	}

	ret = wait_for_completion_timeout(&fusion_F04B.fusion_F04B_done, msecs_to_jiffies(3000));
	if( ret == 0)
	{
		printk("wait for completion is timeout !\n");
		return -1;
	}

	memcpy(rec_buf, &(fusion_F04B.response_data[1]), 6);

	fusion_F04B.drv_cnt = (int)rec_buf[0];
	fusion_F04B.sen_cnt = (int)rec_buf[1];
	*x = rec_buf[2];
	*x |= (unsigned short)(rec_buf[3]) << 8;
	*y = rec_buf[4];
	*y |= (unsigned short)(rec_buf[5]) << 8;
	if((*x > 0) && (*y > 0))
		return 0;

	return -1;
}

static void fusion_F04B_work_func(struct work_struct *work)
{
	int ret, fingers,length, i, j;
	unsigned char packet_header;
	unsigned char *temp;
	unsigned char continue_packet_flag;

	int data_length = 0;
	unsigned char buf[MAX_PACKET_LEN];
	unsigned char raw_data[fusion_F04B.drv_cnt][fusion_F04B.sen_cnt];
	unsigned char data[fusion_F04B.drv_cnt * fusion_F04B.sen_cnt + 1];

	temp = data;
	memset(buf, 0, sizeof(buf));
	memset(data, 0, sizeof(data));
	memset(raw_data, 0, sizeof(raw_data));

	do{
		ret = i2c_master_recv(fusion_F04B.client, &packet_header, 1);
		if (ret < 0)
		{
			dev_err(&fusion_F04B.client->dev,
				"i2c_master_recv failed: %d\n", ret);
		}

		length = (int)(packet_header & 0x3f);
		continue_packet_flag = packet_header & 0x40;

		ret = i2c_master_recv(fusion_F04B.client, buf, length + 1/*one byte packet header*/);
		if (ret < 0)
		{
			dev_err(&fusion_F04B.client->dev,
				"i2c_master_recv failed: %d\n", ret);
		}

		memcpy(temp, &buf[1], length);
		temp += length;
		data_length += length;
	}while(continue_packet_flag);

	switch(data[0])
	{
		case REPORT_TOUCH:
			{
				fingers = (int)(data[1]);//check the report actual count
				if((fingers > fusion_F04B_MAX_FINGER) || (fingers < 0))
				{
					printk(KERN_ERR "receive the number of touch point is:%d\n", fingers);
				}
				else
				{
					struct fusion_F04B_report_format *ptr_report = (struct fusion_F04B_report_format *)(&data[2]);
					#if defined(CONFIG_TOUCHSCREEN_FUSION_4_MULTITOUCH)
					int f;

					printk(KERN_DEBUG "fingers %d, tip1,tid1,x1,y1,z1(%x,%x,%d,%d,%d),tip2,tid2,x2,y2,z2(%x,%x,%d,%d,%d),tip3,tid3,x3,y3,z3(%x,%x,%d,%d,%d),tip4,tid4,x4,y4,z4(%x,%x,%d,%d,%d)\n",
					   fingers,
					   ptr_report[0].pt_flag == fusion_F04B_TOUCH_DOWN ? 1 : 0, ptr_report[0].contactID, ptr_report[0].x, ptr_report[0].y, ptr_report[0].z,
					   ptr_report[1].pt_flag == fusion_F04B_TOUCH_DOWN ? 1 : 0, ptr_report[1].contactID, ptr_report[1].x, ptr_report[1].y, ptr_report[1].z,
					   ptr_report[2].pt_flag == fusion_F04B_TOUCH_DOWN ? 1 : 0, ptr_report[2].contactID, ptr_report[2].x, ptr_report[2].y, ptr_report[2].z,
					   ptr_report[3].pt_flag == fusion_F04B_TOUCH_DOWN ? 1 : 0, ptr_report[3].contactID, ptr_report[3].x, ptr_report[3].y, ptr_report[3].z);

					for (f = 0; f < fingers; f++, ptr_report++)
					{
						if ((ptr_report->pt_flag == fusion_F04B_TOUCH_DOWN) || (ptr_report->pt_flag == fusion_F04B_TOUCH_UP/*?*/))//Finger down or up
						{
							int tip = (ptr_report->pt_flag == fusion_F04B_TOUCH_UP) ?  0 : 1;

							input_report_abs(fusion_F04B.input_dev, ABS_MT_TOUCH_MAJOR, tip);
							input_report_abs(fusion_F04B.input_dev, ABS_MT_WIDTH_MAJOR, ptr_report->contactID);
							input_report_abs(fusion_F04B.input_dev, ABS_MT_POSITION_X, ptr_report->x);
							input_report_abs(fusion_F04B.input_dev, ABS_MT_POSITION_Y, ptr_report->y);
							input_mt_sync(fusion_F04B.input_dev);
						}
						else
						{
							printk(KERN_INFO "[2]fusion_F04B_work_func: bad packet : finger#%d : ptr_report->pt_flag = 0x%x, Actual Count = 0x%x\n",
								(f+1), ptr_report->pt_flag, data[1]/*Actual Count*/);
						}
					}
#else
					input_report_abs(fusion_F04B.input_dev, ABS_PRESSURE, ptr_report[0].pt_flag == fusion_F04B_TOUCH_DOWN ? 1 : 0);
					input_report_abs(fusion_F04B.input_dev, ABS_X, ptr_report[0].x);
					input_report_abs(fusion_F04B.input_dev, ABS_Y, ptr_report[0].y);
#endif
					input_sync(fusion_F04B.input_dev);
				}
			}
			break;

		case CMD_RESPONSE:
			{
				memset(fusion_F04B.response_data, 0, MAX_PACKET_LEN);
				memcpy(fusion_F04B.response_data, data, data_length);
				complete(&fusion_F04B.fusion_F04B_done);
			}
			break;

		case REPORT_SENSITIVITY:/*Sensitivity*/
		case REPORT_RAW_DATA:/*RawData*/
		case REPORT_BASELINE:/*BaseLine*/
		case REPORT_CMOD:/*CMOD*/
			{
				printk("\n\n\t**********firmware team debug information**********\n");

				temp = data;
				for(i = 0; i < fusion_F04B.drv_cnt; i++)
					for(j = 0; j < fusion_F04B.sen_cnt; j++)
					{
						raw_data[i][j] = *(++temp);
					}

				for(j = 0; j < fusion_F04B.sen_cnt; j++)
				{
					if(j == 0)
					{
						for(i = 0; i < fusion_F04B.drv_cnt; i++)
						{
							if(i == 0)
								printk("\t");

							printk("%3d ", i);
						}
						printk("\n");
					}

					for(i = 0; i < fusion_F04B.drv_cnt; i++)
					{
						if(i == 0)
							printk("\n%3d\t", j);

						printk("%03d ", raw_data[i][j]);
					}
				}
				msleep(1000);
			}
			break;

		default:
			printk(KERN_INFO "fusion_F04B: The packet data is other type: data[0]=0x%02x, data[1]=0x%02x\n", data[0], data[1]);

		return;
	}
}

static irqreturn_t fusion_F04B_irq_handler(int irq, void *dev_id)
{
	queue_work(fusion_F04B.workq, &fusion_F04B.work);

	return IRQ_HANDLED;
}

static int fusion_F04B_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	unsigned char fwver[3];
	unsigned char fwinfo[6];
	unsigned short x_res, y_res;

	printk("fusion_F04B: Touchscreen Driver %s, (c) 2011 Touch Revolution Inc.\n", fusion_F04B_VERSION);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "fusion_F04B_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto bail1;
	}

	init_completion(&fusion_F04B.fusion_F04B_done);
	INIT_WORK(&fusion_F04B.work, fusion_F04B_work_func);
	fusion_F04B.client = i2c;
	i2c_set_clientdata(i2c, &fusion_F04B);

	printk(KERN_INFO "fusion_F04B: Touchscreen registered with bus id (%d) with slave address 0x%x\n",
			i2c_adapter_id(fusion_F04B.client->adapter), fusion_F04B.client->addr);

	fusion_F04B.workq = create_singlethread_workqueue("fusion_F04B");
	if (fusion_F04B.workq == NULL) {
		dev_err(&i2c->dev, "can't create work queue\n");
		ret = -ENOMEM;
		goto bail1;
	}

	fusion_F04B.input_dev = input_allocate_device();
	if (fusion_F04B.input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "failed to allocate input device: %d\n", ret);
		goto bail2;
	}

	ret = request_irq(i2c->irq, fusion_F04B_irq_handler, IRQ_DISABLED | IRQF_TRIGGER_FALLING, i2c->name, &fusion_F04B);
	if (ret < 0) {
		dev_err(&i2c->dev, "can't get irq %d: %d\n", i2c->irq, ret);
		goto bail2;
	}

	/*reset touch panel*/
	ret = reset();
	if(ret < 0)
	{
		dev_err(&i2c->dev, "fusion_F04B reset fail: %d\n", ret);
		goto bail3;
	}

	/* read some info from touch panel*/
	ret = get_firmware_version(fwver);
	if(ret < 0)
	{
		dev_err(&i2c->dev, "fusion_F04B get template fwver fail: %d\n", ret);
		goto bail3;
	}

	if((fwver[0] & 0xf0) == BL_MODE)
	{
		printk("fusion_F04B is in bootloader mode, bootloader version:0x%02x-0x%02x-0x%02x\n",fwver[0],fwver[1],fwver[2]);
		printk("Please check the touch firmware\n");
		ret = -1;
		goto bail3;
	}

	ret = fusion_F04B_get_fwinfo(fwinfo, 6);
	if(ret < 0)
	{
		dev_err(&i2c->dev, "fusion_F04B get product fwver information fail: %d\n", ret);
		goto bail3;
	}

	printk(KERN_INFO "fusion_F04B: touch firmware version: 0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x\n",
		fwinfo[0], fwinfo[1], fwinfo[2], fwinfo[3], fwinfo[4], fwinfo[5]);

	ret = fusion_F04B_get_res(&x_res, &y_res);
	if(ret < 0)
	{
		dev_err(&i2c->dev, "fusion_F04B_get_res fail: %d\n", ret);
		goto bail3;
	}
	printk(KERN_INFO "fusion_F04B: Touchscreen resolution: x= %d, y= %d\n", x_res, y_res);

	fusion_F04B.input_dev->name = "fusion_F04B";
	set_bit(EV_SYN, fusion_F04B.input_dev->evbit);
	set_bit(EV_ABS, fusion_F04B.input_dev->evbit);

#if defined(CONFIG_TOUCHSCREEN_FUSION_4_MULTITOUCH)
	input_set_abs_params(fusion_F04B.input_dev, ABS_MT_POSITION_X, 0, x_res, 0, 0);
	input_set_abs_params(fusion_F04B.input_dev, ABS_MT_POSITION_Y, 0, y_res, 0, 0);
	input_set_abs_params(fusion_F04B.input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(fusion_F04B.input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#else
	set_bit(ABS_X, fusion_F04B.input_dev->absbit);
	set_bit(ABS_Y, fusion_F04B.input_dev->absbit);
	set_bit(ABS_PRESSURE, fusion_F04B.input_dev->absbit);
	input_set_abs_params(fusion_F04B.input_dev, ABS_X, 0, x_res, 0, 0);
	input_set_abs_params(fusion_F04B.input_dev, ABS_Y, 0, y_res, 0, 0);
	input_set_abs_params(fusion_F04B.input_dev, ABS_PRESSURE, 0, 1 ,0, 0);
#endif

	ret = input_register_device(fusion_F04B.input_dev);
	if (ret) {
		dev_err(&i2c->dev, "can't register input: %d\n", ret);
		goto bail3;
	}

	ret = start_report();
	if(ret < 0)
	{
		dev_err(&i2c->dev, "fusion_F04B start report touch fail: %d\n", ret);
		goto bail4;
	}

#if COMMAND
	ret = kobject_init_and_add( &test_cmd.fusion_F04B_kobj, &fusion_F04B_ktype, &fusion_F04B.input_dev->dev.kobj, "fusion_F04B");
	if (ret)
	{
		printk(KERN_WARNING "fusion_F04B kobject_init_and_add error: %d\n", ret);
	}
#endif

	return 0;

	bail4:
	input_unregister_device(fusion_F04B.input_dev);

	bail3:
	free_irq(i2c->irq, &fusion_F04B);

	bail2:
	destroy_workqueue(fusion_F04B.workq);
	fusion_F04B.workq = NULL;

	bail1:
	return ret;
}

#ifdef CONFIG_PM
static int fusion_F04B_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	if (i2c->irq) {
		disable_irq(i2c->irq);
		flush_workqueue(fusion_F04B.workq);
	}
	return 0;
}

static int fusion_F04B_resume(struct i2c_client *i2c)
{
	if (i2c->irq) {
		enable_irq(i2c->irq);
	}
	return 0;
}
#endif

static int fusion_F04B_remove(struct i2c_client *i2c)
{
	stop_report();
	destroy_workqueue(fusion_F04B.workq);
	free_irq(i2c->irq, &fusion_F04B);
	input_unregister_device(fusion_F04B.input_dev);
	i2c_set_clientdata(i2c, NULL);
#if COMMAND
	kobject_del(&test_cmd.fusion_F04B_kobj);
#endif
	printk(KERN_INFO "fusion_F04B driver removed\n");

	return 0;
}

static const struct i2c_device_id fusion_F04B_id[] = {
	{ "fusion_F04B", 0 },
	{ }
};

static struct i2c_driver fusion_F04B_driver = {
	.driver = {
		.name	= "fusion_F04B",
	},
	.probe		= fusion_F04B_probe,
	.remove		= fusion_F04B_remove,
#ifdef CONFIG_PM
	.suspend		= fusion_F04B_suspend,
	.resume			= fusion_F04B_resume,
#endif
	.id_table	= fusion_F04B_id,
};

static int __init fusion_F04B_init(void)
{
	int ret;

	memset(&fusion_F04B, 0, sizeof(fusion_F04B));

	/* Probe for fusion_F04B on I2C. */
	ret = i2c_add_driver(&fusion_F04B_driver);
	if (ret < 0) {
		printk(KERN_ERR  "fusion_F04B_init can't add i2c driver: %d\n", ret);
	}
	return ret;
}

static void __exit fusion_F04B_exit(void)
{
	i2c_del_driver(&fusion_F04B_driver);
}

module_init(fusion_F04B_init);
module_exit(fusion_F04B_exit);

MODULE_DESCRIPTION("fusion_F04B Touchscreen Driver");
MODULE_LICENSE("GPL");
