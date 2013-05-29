/*
    adc121C027.c - Analog to Digital converter integrated into Piper.

    Copyright (C) 2009 Digi International <sales2@digi.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>

#include "airohaCalibration.h"
#include "pipermain.h"

/* Addresses to scan: none, device is not autodetected */
/* static const unsigned short normal_i2c[] = { I2C_CLIENT_END }; */

#define ADC_I2C_ADDR		(0x51)
#define ADC_CYCLE_TIME		(0x20)

static const unsigned short normal_i2c[] = { ADC_I2C_ADDR, I2C_CLIENT_END };
static const unsigned short dummy_i2c_addrlist[] = { I2C_CLIENT_END };

enum adc121C027_cmd {
        ADC_RESULT              = 0,
        ADC_ALERT_STATUS        = 1,
        ADC_CONFIGURATION       = 2,
        ADC_LOW_LIMIT           = 3,
        ADC_HIGH_LIMIT          = 4,
        ADC_HYSTERESIS          = 5,
        ADC_LOWEST_VALUE        = 6,
        ADC_HIGHEST_VALUE       = 7,
};

static u16 adc121C027_read_peak(struct airohaCalibrationData *cal)
{
	struct i2c_client *i2cclient = (struct i2c_client *)cal->priv;

	return be16_to_cpu(i2c_smbus_read_word_data(i2cclient, ADC_HIGHEST_VALUE));
}

static void adc121C027_clear_peak(struct airohaCalibrationData *cal)
{
	struct i2c_client *i2cclient = (struct i2c_client *)cal->priv;

	i2c_smbus_write_word_data(i2cclient, ADC_HIGHEST_VALUE, 0);
}

static u16 adc121C027_read_last_sample(struct airohaCalibrationData *cal)
{
	struct i2c_client *i2cclient = (struct i2c_client *)cal->priv;

	return be16_to_cpu(i2c_smbus_read_word_data(i2cclient, ADC_RESULT));
}

static int adc121C027_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	return (client->addr != ADC_I2C_ADDR) ? -EINVAL : 0;
}

static int adc121C027_remove(struct i2c_client *client)
{
	/* Real shut down will be done by adc121C027_shutdown() */
	return 0;
}

static const struct i2c_device_id adc121C027_id[] = {
	{ "adc121C027", 0 },
	{}
};

static struct i2c_driver adc121C027_driver = {
	.driver = {
		.name   = "adc121C027",
	},
	.probe          = adc121C027_probe,
	.remove         =  __devexit_p(adc121C027_remove),
	.id_table       = adc121C027_id,
};

/* Turn on automatic A/D process by setting a non zero cycle time */
static void adc121C027_hw_init(struct airohaCalibrationData *cal)
{
	struct i2c_client *i2cclient = (struct i2c_client *)cal->priv;

	i2c_smbus_write_word_data(i2cclient, ADC_CONFIGURATION, ADC_CYCLE_TIME);
}

void adc121C027_shutdown(struct airohaCalibrationData *cal)
{
	struct i2c_client *i2cclient = (struct i2c_client *)cal->priv;

	if (i2cclient) {
		i2c_unregister_device(i2cclient);
		cal->priv = NULL;
	}
	i2c_del_driver(&adc121C027_driver);
}

int adc121C027_init(struct airohaCalibrationData *cal, int i2cadapter)
{
	struct i2c_board_info board_info = {
		.type = "adc121C027",
		.addr = ADC_I2C_ADDR,
	};
	struct i2c_adapter *adapter;
	struct i2c_client *adc_i2c_client;
	int ret;

	ret = i2c_add_driver(&adc121C027_driver);
	if (ret) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
			": error adding driver adc121C027_driver (%d)\n", ret);
		return ret;
	}

	adapter = i2c_get_adapter(i2cadapter);
	if (!adapter) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
			": error getting i2c adapter\n");
		return -EINVAL;
	}

	adc_i2c_client = i2c_new_device(adapter, &board_info);
	if (!adc_i2c_client) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
			": error creating new i2c client\n");
		return -EINVAL;
	}

	cal->priv = (void *)adc_i2c_client;
	adc121C027_hw_init(cal);

	cal->cops = kmalloc(sizeof(struct calibration_ops), GFP_KERNEL);
	if (!cal->cops) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
			": unable to allocate memory for cal->cops\n");
		return -ENOMEM;
	}
	cal->cops->adc_read_peak = adc121C027_read_peak;
	cal->cops->adc_clear_peak = adc121C027_clear_peak;
	cal->cops->adc_read_last_val = adc121C027_read_last_sample;
	cal->cops->adc_shutdown = adc121C027_shutdown;

	return 0;
}
EXPORT_SYMBOL_GPL(adc121C027_init);


