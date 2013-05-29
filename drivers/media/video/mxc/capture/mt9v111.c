/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mt9v111.c
 *
 * @brief mt9v111 camera driver functions
 *
 * @ingroup Camera
 */

//#define MT9V111_DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include "mt9v111.h"

#ifdef MT9V111_DEBUG
static u16 testpattern;
#endif

extern void gpio_camera_active(void);
static mt9v111_conf mt9v111_device;

/*!
 * Holds the current frame rate.
 */

struct sensor {
	const struct mt9v111_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	bool on;
	bool used;

	/* control settings */
	int brightness;
	int saturation;
	int sharpness;
	int gain;
	int ae_mode;

};

static int mt9v111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int mt9v111_remove(struct i2c_client *client);

static const struct i2c_device_id mt9v111_id[] = {
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{"mt9v111_1", 2},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	{"mt9v111_2", 3},
#endif
	{},
};

struct sensor mt9v111_data[ARRAY_SIZE(mt9v111_id)-1];

MODULE_DEVICE_TABLE(i2c, mt9v111_id);

static int mt9v111_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("In mt9v111_suspend\n");

	return 0;
}

static int mt9v111_resume(struct i2c_client *client)
{
	pr_debug("In mt9v111_resume\n");

	return 0;
}

static struct i2c_driver mt9v111_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mt9v111",
		   },
	.probe = mt9v111_probe,
	.remove = mt9v111_remove,
	.id_table = mt9v111_id,
	.suspend = mt9v111_suspend,
	.resume = mt9v111_resume,
};

/*
 * Function definitions
 */

static int mt9v111_id_from_name ( const char * name )
{
	int i;

	if( name == NULL || ( strlen(name) < strlen("mt9v111_n") ) )
		return -1;

	for (i=0; i < ARRAY_SIZE(mt9v111_id); i++) {
		if (!strcmp(name, mt9v111_id[i].name))
			return i;
	}

	return -1;
}

static inline int mt9v111_read_reg(int sensorid , u8 reg)
{
	int val = i2c_smbus_read_word_data(mt9v111_data[sensorid].i2c_client, reg);
	if (val != -1)
		val = cpu_to_be16(val);
	return val;
}

/*!
 * Writes to the register via I2C.
 */
static inline int mt9v111_write_reg(int sensorid , u8 reg, u16 val)
{
	pr_debug("[%d] In mt9v111_write_reg (0x%x, 0x%x)\n", sensorid , reg, val);
	pr_debug("   write reg %x val %x.\n", reg, val);

	return i2c_smbus_write_word_data(mt9v111_data[sensorid].i2c_client,
					 reg, cpu_to_be16(val));
}

/*!
 * Initialize mt9v111_sensor_lib_datasheet
 * Libarary for Sensor configuration through I2C
 *
 * @param       coreReg       Core Registers
 * @param       ifpReg        IFP Register
 *
 * @return status
 */
static u8 mt9v111_sensor_lib_datasheet(int sensorid , mt9v111_coreReg * coreReg, mt9v111_IFPReg * ifpReg)
{
	u8 reg;
	u16 data;
	u8 error = 0;

	pr_debug("In mt9v111_sensor_lib\n");

	/* IFP R51(0x33)=5137,R57(0x39)=290,R59(0x3B)=1068,R62(0x3E)=4095,R89(0x59)=504,R90(0x5A)=605,R92(0x5C)=8222,R93(0x5D)=10021,R100(0x64)=4477 */

	/*
	 * setup to IFP registers
	 */
	reg = MT9V111I_ADDR_SPACE_SEL;
	data = ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_LIMIT_SHARP_SATU_CTRL;
	data = ifpReg->limitSharpSatuCtrl;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_UPPER_SHUTTER_DELAY_LIM;
	data = ifpReg->upperShutterDelayLi;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_IPF_BLACK_LEVEL_SUB;
	data = ifpReg->ipfBlackLevelSub;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_GAIN_THRE_CCAM_ADJ;
	data = ifpReg->agimnThreCamAdj;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_SHUTTER_60;
	data = ifpReg->shutter_width_60;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_AUTO_EXPOSURE_17;
	data = ifpReg->auto_exposure_17;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_SEARCH_FLICK_60;
	data = ifpReg->search_flicker_60;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_RESERVED93;
	data = ifpReg->reserved93;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_RESERVED100;
	data = ifpReg->reserved100;
	mt9v111_write_reg(sensorid,reg, data);

	/*
	 * setup to sensor core registers
	 */
	reg = MT9V111I_ADDR_SPACE_SEL;
	data = coreReg->addressSelect;
	mt9v111_write_reg(sensorid,reg, data);

	/* Core R5=46, R7[4]=0 (DEFAULT) ,R33=58369*/

	reg = MT9V111S_HOR_BLANKING;
	data = coreReg->horizontalBlanking;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111S_RESERVED33;
	data = coreReg->reserved33;
	mt9v111_write_reg(sensorid,reg, data);

	// Digi: 180º image rotation to compensate sensor position in the CAM_APPKIT
	reg = MT9V111S_READ_MODE;
	data = MT9V111S_RM_RIGTH_TO_LEFT | MT9V111S_RM_BOTTOM_TO_TOP | MT9V111S_RM_COLUMN_LATE | MT9V111S_RM_ROW_LATE;
	mt9v111_write_reg(sensorid,reg, data);

	return error;
}

void mt9v111_config_datasheet(void)
{
	pr_debug("In mt9v111_config_datasheet\n");

	mt9v111_device.coreReg->addressSelect = MT9V111I_SEL_SCA;

	/* MT9V111I_ADDR_SPACE_SEL */
	mt9v111_device.ifpReg->addrSpaceSel = MT9V111I_SEL_IFP;

	/* Recommended values for 30fps @ 27MHz from datasheet*/

	/* Core R5=132, R6=10 , R7[4]=0 (DEFAULT) ,R33=58369*/

	mt9v111_device.coreReg->horizontalBlanking = 132;
	mt9v111_device.coreReg->verticalBlanking = 10;
	mt9v111_device.coreReg->reserved33 = 58369;

	/* IFP R51(0x33)=5137,R57(0x39)=290,R59(0x3B)=1068,R62(0x3E)=4095,R89(0x59)=504,R90(0x5A)=605,R92(0x5C)=8222,R93(0x5D)=10021,R100(0x64)=4477 */

	mt9v111_device.ifpReg->limitSharpSatuCtrl = 5137;
	mt9v111_device.ifpReg->upperShutterDelayLi = 290;
	mt9v111_device.ifpReg->ipfBlackLevelSub = 1068;
	mt9v111_device.ifpReg->agimnThreCamAdj = 4095;

	mt9v111_device.ifpReg->shutter_width_60 = 504;
	mt9v111_device.ifpReg->auto_exposure_17 = 605;
	mt9v111_device.ifpReg->search_flicker_60 = 8222;
	mt9v111_device.ifpReg->reserved93 = 10021;
	mt9v111_device.ifpReg->reserved100 = 4477;
}


/*!
 * mt9v111 sensor set saturtionn
 *
 * @param saturation   int

 * @return  Error code of 0.
 */
static int mt9v111_set_saturation(int sensorid , int saturation)
{
	u8 reg;
	u16 data;
	pr_debug("In mt9v111_set_saturation(%d)\n",
		saturation);

	switch (saturation) {
	case 150:
		mt9v111_device.ifpReg->awbSpeed = 0x6D14;
		break;
	case 100:
		mt9v111_device.ifpReg->awbSpeed = 0x4514;
		break;
	case 75:
		mt9v111_device.ifpReg->awbSpeed = 0x4D14;
		break;
	case 50:
		mt9v111_device.ifpReg->awbSpeed = 0x5514;
		break;
	case 37:
		mt9v111_device.ifpReg->awbSpeed = 0x5D14;
		break;
	case 25:
		mt9v111_device.ifpReg->awbSpeed = 0x6514;
		break;
	case 0:
		mt9v111_device.ifpReg->awbSpeed = 0x7514;
		break;
	default:
		mt9v111_device.ifpReg->awbSpeed = 0x4514;
		break;
	}

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111I_AWB_SPEED;
	data = mt9v111_device.ifpReg->awbSpeed;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

#if 0
/*!
 * mt9v111 sensor set digital zoom
 *
 * @param on/off   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalzoom(int sensorid , unsigned int on)
{
	u8 reg;
	u16 data;
	pr_debug("In mt9v111_set_digitalzoom(%d)\n",on);

	if( on > 1 )
	    return -1;

	mt9v111_device.coreReg->digitalZoom = on;

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.coreReg->addressSelect;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111S_DIGITAL_ZOOM;
	data = mt9v111_device.coreReg->digitalZoom;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set digital pan
 *
 * @param pan_level   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalpan (int sensorid , int pan_level)
{
	u8 reg;
	u16 data;
	pr_debug("In mt9v111_set_digitalpan(%d)\n",
		pan_level);

	mt9v111_device.ifpReg->HPan = 8;
	if (pan_level & 0xFFFF0000) {
		pan_level = (0xFFFFFFFF - pan_level);
		pan_level = pan_level / 0x14;
		mt9v111_device.ifpReg->HPan =
			mt9v111_device.ifpReg->HPan - (pan_level & 0x3FF);
	} else {
		pan_level = pan_level / 0x14;
		mt9v111_device.ifpReg->HPan =
			mt9v111_device.ifpReg->HPan + (pan_level - 1);
	}

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111i_H_PAN;
	data = mt9v111_device.ifpReg->HPan;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set digital tilt
 *
 * @param tilt_level   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitaltilt (int sensorid , int tilt_level)
{
	u8 reg;
	u16 data;
	pr_debug("In mt9v111_set_digitaltilt(%d)\n",
		tilt_level);

	mt9v111_device.ifpReg->VPan = 8;
        if( tilt_level & 0xFFFF0000 ) {
				tilt_level = (0xFFFFFFFF - tilt_level);
				tilt_level = tilt_level / 0x14;
                mt9v111_device.ifpReg->VPan = mt9v111_device.ifpReg->VPan - (tilt_level & 0x3FF);
        }
        else {
				tilt_level = tilt_level / 0x14;
                mt9v111_device.ifpReg->VPan = mt9v111_device.ifpReg->VPan + (tilt_level - 1);
        }

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111i_V_PAN;
	data = mt9v111_device.ifpReg->VPan;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set output resolution
 *
 * @param resolution   res

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_outputresolution(int sensorid , MT9V111_OutputResolution res)
{
	u8 reg;
	u16 data;
	int zoom = 0;

	pr_debug("In mt9v111_set_outputresolution(%d)\n",res);

	switch (res) {
	    case MT9V111_OutputResolution_VGA:
			/* 640x480 */
			mt9v111_device.ifpReg->HSize = 0x0280;
			mt9v111_device.ifpReg->VSize = 0x01E0;
			break;

	    case MT9V111_OutputResolution_QVGA:
			/* 320x240 */
			mt9v111_device.ifpReg->HSize = 0x0140;
			mt9v111_device.ifpReg->VSize = 0x00F0;
			break;

	    case MT9V111_OutputResolution_CIF:
			/* 352x288 */
			mt9v111_device.ifpReg->HSize = 0x0160;
			mt9v111_device.ifpReg->VSize = 0x0120;
			mt9v111_device.ifpReg->HZoom = 0x0160;
			mt9v111_device.ifpReg->VZoom = 0x0120;
			zoom = 1;
			break;

	    case MT9V111_OutputResolution_QCIF:
			/* 176X220 */
			mt9v111_device.ifpReg->HSize = 0x00B0;
			mt9v111_device.ifpReg->VSize = 0x0090;
			mt9v111_device.ifpReg->HZoom = 0x00B0;
			mt9v111_device.ifpReg->VZoom = 0x0090;
			zoom = 1;
			break;

	    case MT9V111_OutputResolution_QQVGA:
			/* 2048*1536 */
			mt9v111_device.ifpReg->HSize = 0x00A0;
			mt9v111_device.ifpReg->VSize = 0x0078;
			mt9v111_device.ifpReg->HZoom = 0x00A0;
			mt9v111_device.ifpReg->VZoom = 0x0078;
			zoom = 1;
			break;

	    case MT9V111_OutputResolution_SXGA:
		/* 1280x1024 */
			break;

		default:
			break;
	}

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111i_V_SIZE;
	data = mt9v111_device.ifpReg->VSize;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111i_H_SIZE;
	data = mt9v111_device.ifpReg->HSize;
	mt9v111_write_reg(sensorid,reg, data);

	if ( zoom ) {
	    reg = MT9V111i_V_ZOOM;
	    data = mt9v111_device.ifpReg->VZoom;
	    mt9v111_write_reg(sensorid,reg, data);

	    reg = MT9V111i_H_ZOOM;
	    data = mt9v111_device.ifpReg->HZoom;
	    mt9v111_write_reg(sensorid,reg, data);
	}

	return 0;
}

/*!
 * mt9v111 sensor set digital flash
 *
 * @param flash_level   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalflash (int sensorid , int flash_level)
{
	u8 reg;
	u16 data = mt9v111_read_reg(sensorid,MT9V111i_FLASH_CTRL);
	pr_debug("In mt9v111_set_digitalflash(%d)\n",
			flash_level);

	if(flash_level) {
	    data &= (0xFF00);
	    data |= ((flash_level & 0x00FF) | (1<<13));
	}
	else {
	    data &= ~(1<<13);
	}

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111i_FLASH_CTRL;
	mt9v111_device.ifpReg->flashCtrl = data;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set digital monochrome
 *
 * @param on   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalmonochrome (int sensorid , int on)
{
	u8 reg;
	u16 data = mt9v111_read_reg(sensorid,MT9V111I_FORMAT_CONTROL);
	pr_debug("In mt9v111_set_digitalmonochrome(%d)\n",
			on);

        /* clear the monochrome bit field */
        data &= ~(1<<5);

         /* enable or disable monochrome mode */
         if( on )
                 data |= (0<<5);
         else
                 data |= (1<<5);

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	/* Operation Mode Control */
	reg = MT9V111I_FORMAT_CONTROL;
	mt9v111_device.ifpReg->formatControl = data;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}
#endif

/*!
 * mt9v111 sensor set digital sharpness
 *
 * @param value   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalsharpness (int sensorid , int value)
{
	u8 reg;
	u16 data ;

	pr_debug("In mt9v111_set_digitalsharpness(%d)\n",value);

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	data = mt9v111_read_reg(sensorid,MT9V111I_APERTURE_GAIN);

	/* erase current and remove auto reduce sharpness in low light */
	 data &= ~(0x000F);
	 data |= (value & (0x000F));
	 if( data > (0x000F) )
			 return -1;

	/* Operation Mode Control */
	reg = MT9V111I_APERTURE_GAIN;
	mt9v111_device.ifpReg->apertureGain = data;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set digital brightness
 *
 * @param value   int

 * @return  0 on success, -1 on error.
 */
static int mt9v111_set_digitalbrightness (int sensorid , int value)
{
	u8 reg;
	u16 data;
	u32 max_brightness, min_brightness;

	pr_debug("In mt9v111_set_digitalbrightness(%d)\n", value);

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid, reg, data);

	data = mt9v111_read_reg(sensorid,MT9V111I_CLIP_LIMIT_OUTPUT_LUMI);
	max_brightness = data >> 8;
	min_brightness = (u8)data;

	if( value > max_brightness )
		value = max_brightness;
	else if( value < min_brightness )
		value = min_brightness;

	data = mt9v111_read_reg(sensorid,MT9V111I_AE_PRECISION_TARGET);
	data &= 0xFF00; /* Clear target luminance */
	data |= ((u8)value );

	/* Operation Mode Control */
	reg = MT9V111I_AE_PRECISION_TARGET;
	mt9v111_device.ifpReg->AEPrecisionTarget = data;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

/*!
 * mt9v111 sensor set Auto Exposure measurement window mode configuration
 *
 * @param ae_mode      int
 * @return  Error code of 0 (no Error)
 */
static int mt9v111_set_ae_mode(int sensorid , int ae_mode)
{
	u8 reg;
	u16 data;

	pr_debug("In mt9v111_set_ae_mode(%d)\n",
		ae_mode);

	/* Currently this driver only supports auto and manual exposure
	 * modes. */
	if ((ae_mode > 1) || (ae_mode << 0))
		return -EPERM;

	/*
	 * The auto exposure is set in bit 14.
	 * Other values are set for:
	 *  -on the fly defect correction is on (bit 13).
	 *  -aperature correction knee enabled (bit 12).
	 *  -ITU_R BT656 synchronization codes are embedded in the image (bit 7)
	 *  -AE measurement window is weighted sum of large and center windows
	 *     (bits 2-3).
	 *  -auto white balance is on (bit 1).
	 *  -normal color processing (bit 4 = 0).
	 */
	/* V4L2_EXPOSURE_AUTO = 0; needs register setting of 0x708E */
	/* V4L2_EXPOSURE_MANUAL = 1 needs register setting of 0x308E */
	mt9v111_device.ifpReg->modeControl &= 0x3fff;
	mt9v111_device.ifpReg->modeControl |= (ae_mode & 0x03) << 14;
	mt9v111_data[sensorid].ae_mode = ae_mode;

	reg = MT9V111I_ADDR_SPACE_SEL;
	data = mt9v111_device.ifpReg->addrSpaceSel;
	mt9v111_write_reg(sensorid,reg, data);

	reg = MT9V111I_MODE_CONTROL;
	data = mt9v111_device.ifpReg->modeControl;
	mt9v111_write_reg(sensorid,reg, data);

	return 0;
}

#if 0
/*!
 * mt9v111 sensor get AE measurement window mode configuration
 *
 * @param ae_mode      int *
 * @return  None
 */
static void mt9v111_get_ae_mode(int *ae_mode)
{
	pr_debug("In mt9v111_get_ae_mode(%d)\n", *ae_mode);

	if (ae_mode != NULL) {
		*ae_mode = (mt9v111_device.ifpReg->modeControl & 0xc) >> 2;
	}
}
#endif

#ifdef MT9V111_DEBUG
/*!
 * Set sensor to test mode, which will generate test pattern.
 *
 * @return none
 */
static void mt9v111_test_pattern(int sensorid , bool flag)
{
	u16 data;

	/* switch to sensor registers */
	mt9v111_write_reg(sensorid,MT9V111I_ADDR_SPACE_SEL, MT9V111I_SEL_SCA);

	if (flag == true) {
		testpattern = MT9V111S_OUTCTRL_TEST_MODE;

		data = mt9v111_read_reg(sensorid,MT9V111S_ROW_NOISE_CTRL) & 0xBF;
		mt9v111_write_reg(sensorid,MT9V111S_ROW_NOISE_CTRL, data);

		mt9v111_write_reg(sensorid,MT9V111S_TEST_DATA, 0);

		/* changes take effect */
		data = MT9V111S_OUTCTRL_CHIP_ENABLE | testpattern | 0x3000;
		mt9v111_write_reg(sensorid,MT9V111S_OUTPUT_CTRL, data);
	} else {
		testpattern = 0;

		data = mt9v111_read_reg(sensorid,MT9V111S_ROW_NOISE_CTRL) | 0x40;
		mt9v111_write_reg(MT9V111S_ROW_NOISE_CTRL, data);

		/* changes take effect */
		data = MT9V111S_OUTCTRL_CHIP_ENABLE | testpattern | 0x3000;
		mt9v111_write_reg(sensorid,MT9V111S_OUTPUT_CTRL, data);
	}
}
#endif


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	pr_debug("In mt9v111:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = MT9V111_MCLK;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.bt_sync_correct = 1;	// translates to CSI ext vsync
	p->u.bt656.clock_min = MT9V111_CLK_MIN;
	p->u.bt656.clock_max = MT9V111_CLK_MAX;

	p->u.bt656.nobt_vs_inv = 1; /* Vs polarity. 1 is active low. */
	p->u.bt656.nobt_hs_inv = 0; /* Hs polarity. 0 is active high. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	pr_debug("In mt9v111:ioctl_s_power\n");

	sensor->on = on;

	if(on) {
		ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */, true, true);
	}
	else {
		ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */, false, false);
	}

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	/* s->priv points to mt9v111_data */
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	pr_debug("In mt9v111:ioctl_g_parm\n");

	if( sensorid < 0 )
		return ret;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = mt9v111_data[sensorid].streamcap.capability;
		cparm->timeperframe =
				mt9v111_data[sensorid].streamcap.timeperframe;
		cparm->capturemode = mt9v111_data[sensorid].streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	/* s->priv points to mt9v111_data */
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	pr_debug("In mt9v111:ioctl_s_parm\n");

	if( sensorid < 0 )
		return ret;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		/* Check that the new frame rate is allowed.
		 * Changing the frame rate is not allowed on this
		 *camera. */
		if (cparm->timeperframe.denominator !=
		    mt9v111_data[sensorid].streamcap.timeperframe.denominator) {
			pr_err("ERROR: mt9v111: ioctl_s_parm: " \
			       "This camera does not allow frame rate "
			       "changes.\n");
			ret = -EINVAL;
		} else {
			mt9v111_data[sensorid].streamcap.timeperframe =
						cparm->timeperframe;
		      /* Call any camera functions to match settings. */
		}

		/* Check that new capture mode is supported. */
		if ((cparm->capturemode != 0) &&
		    !(cparm->capturemode & V4L2_MODE_HIGHQUALITY)) {
			pr_err("ERROR: mt9v111: ioctl_s_parm: " \
				"unsupported capture mode\n");
			ret  = -EINVAL;
		} else {
			mt9v111_data[sensorid].streamcap.capturemode =
						cparm->capturemode;
		      /* Call any camera functions to match settings. */
		      /* Right now this camera only supports 1 mode. */
		}
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;
	/* s->priv points to mt9v111_data */

	pr_debug("In mt9v111:ioctl_g_fmt_cap.\n");
	pr_debug("   Returning size of %dx%d\n",
		sensor->pix.width, sensor->pix.height);

	f->fmt.pix = sensor->pix;

	return 0;
}

#if 0
/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	pr_debug("In mt9v111:ioctl_queryctrl\n");

	return 0;
}
#endif

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	pr_debug("In mt9v111:ioctl_g_ctrl\n");

	if( sensorid < 0 )
		return 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		vc->value = mt9v111_data[sensorid].brightness;
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		vc->value = mt9v111_data[sensorid].saturation;
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		vc->value = mt9v111_data[sensorid].ae_mode;
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		vc->value = mt9v111_data[sensorid].gain;
		break;
	default:
		pr_debug("   Default case\n");
		return -EPERM;
		break;
	}

	return 0;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	pr_debug("In mt9v111:ioctl_s_ctrl %d\n",
		vc->id);

	if( sensorid < 0 )
		return retval;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		mt9v111_set_digitalbrightness(sensorid,vc->value);
		mt9v111_data[sensorid].brightness = vc->value;
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		retval = mt9v111_set_saturation(sensorid,vc->value);
		mt9v111_data[sensorid].saturation = vc->value;
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		retval = mt9v111_set_ae_mode(sensorid,vc->value);
		mt9v111_data[sensorid].ae_mode = vc->value;
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		mt9v111_set_digitalsharpness(sensorid,vc->value);
		mt9v111_data[sensorid].gain = vc->value;
		break;
	default:
		pr_debug("   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

static void mt9v111_ifp_reset ( int sensorid )
{
	mt9v111_write_reg(sensorid,MT9V111S_ADDR_SPACE_SEL, 0x0001);
	mt9v111_write_reg(sensorid,MT9V111I_SOFT_RESET, 0x0001);
	msleep(100);
	mt9v111_write_reg(sensorid,MT9V111I_SOFT_RESET, 0x0000);
	msleep(100);
}

static void mt9v111_sensor_reset ( int sensorid )
{
	mt9v111_write_reg(sensorid,MT9V111S_ADDR_SPACE_SEL, 0x0004);
	mt9v111_write_reg(sensorid,MT9V111S_RESET, 0x0001);
	msleep(100);
	mt9v111_write_reg(sensorid,MT9V111S_RESET, 0x0000);
	msleep(100);
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	int sensorid = 0;

	sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);
	if( sensorid < 0 )
		return 0;

	pr_debug("In mt9v111:ioctl_init for sensor %d\n",sensorid);

	mt9v111_sensor_reset(sensorid);
	mt9v111_ifp_reset(sensorid);
	mt9v111_sensor_lib_datasheet(sensorid,mt9v111_device.coreReg, mt9v111_device.ifpReg);

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	int sensorid = 0;
	uint32_t clock_rate = MT9V111_MCLK;

	sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);
	if( sensorid < 0 )
		return 0;

	pr_debug("In mt9v111:ioctl_dev_init\n");

	set_mclk_rate(&clock_rate, 0);	// Both sensors use mclk0 on Digi ccimx51

	mt9v111_sensor_reset(sensorid);
	mt9v111_ifp_reset(sensorid);
	mt9v111_sensor_lib_datasheet(sensorid,mt9v111_device.coreReg, mt9v111_device.ifpReg);

	return 0;
}

/* list of image formats supported by sensor */
static const struct v4l2_fmtdesc mt9v111_formats[] = {
	{
		.description = "RGB565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		.description = "YUV422 UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

#define MT9V111_NUM_CAPTURE_FORMATS	ARRAY_SIZE(mt9v111_formats)

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= MT9V111_NUM_CAPTURE_FORMATS)
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}

	fmt->flags = mt9v111_formats[index].flags;
	strlcpy(fmt->description, mt9v111_formats[index].description,
		sizeof(fmt->description));
	fmt->pixelformat = mt9v111_formats[index].pixelformat;

	return 0;
}

static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
			struct v4l2_format *f)
{
	unsigned short reg;
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);
	struct sensor *sensor = s->priv;
	/* s->priv points to mt9v111_data */

	if( sensorid < 0 )
		return -ENODEV;

	/* Select IFP registers */
	mt9v111_write_reg (sensorid,MT9V111S_ADDR_SPACE_SEL, 0x0001);

	switch (f->fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_RGB565:
			/*MT9V111I_OUTPUT_FORMAT_CTRL2*/
			reg = mt9v111_read_reg (sensorid,MT9V111I_OUTPUT_FORMAT_CTRL2);
			reg &= ~(0x3 << 6);
			mt9v111_write_reg (sensorid,MT9V111I_OUTPUT_FORMAT_CTRL2, reg);

			/* MT9V111I_FORMAT_CONTROL */
			reg = mt9v111_read_reg(sensorid,MT9V111I_FORMAT_CONTROL);
			reg |= 1 << 12;
			mt9v111_write_reg(sensorid,MT9V111I_FORMAT_CONTROL, reg);
			break;

		case V4L2_PIX_FMT_YUV444:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_YUYV:
			/* MT9V111I_FORMAT_CONTROL */
			reg = mt9v111_read_reg(sensorid,MT9V111I_FORMAT_CONTROL);
			reg &= ~(1 << 12);
			mt9v111_write_reg(sensorid,MT9V111I_FORMAT_CONTROL, reg);
			break;

		default:
			return -EINVAL;
	}

	sensor->pix.width = f->fmt.pix.width;
	sensor->pix.height = f->fmt.pix.height;
	sensor->pix.sizeimage = f->fmt.pix.sizeimage;
	sensor->pix.pixelformat = f->fmt.pix.pixelformat;
	return 0;
}

static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	int i;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	for( i=0 ; i < MT9V111_NUM_CAPTURE_FORMATS ; i++) {
		if( f->fmt.pix.pixelformat == mt9v111_formats[i].pixelformat )
			return 0;
	}

	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ioctl_get_register(struct v4l2_int_device *s,struct v4l2_dbg_register * dreg)
{
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */ , true, true);
	dreg->val = mt9v111_read_reg (sensorid,dreg->reg);
	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */ , false, false);
	return 0;
}

static int ioctl_set_register(struct v4l2_int_device *s,struct v4l2_dbg_register * dreg)
{
	int sensorid = mt9v111_id_from_name(((struct sensor *)s->priv)->v4l2_int_device->name);

	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */ , true, true);
	mt9v111_write_reg (sensorid,dreg->reg, dreg->val);
	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */ , false, false);
	return 0;
}
#endif

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc mt9v111_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *) ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *) ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *) ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},


#ifdef CONFIG_VIDEO_ADV_DEBUG
	{vidioc_int_g_register_num,
				(v4l2_int_ioctl_func *) ioctl_get_register},
	{vidioc_int_s_register_num,
				(v4l2_int_ioctl_func *) ioctl_set_register},
#endif
	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_try_fmt_cap},

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*) ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *) ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*) ioctl_s_ctrl},
};

static struct v4l2_int_slave mt9v111_slave[] = {
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{
		.ioctls = mt9v111_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(mt9v111_ioctl_desc),
		.attach_to = "mxc_v4l2_cap_1",
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	{
		.ioctls = mt9v111_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(mt9v111_ioctl_desc),
		.attach_to = "mxc_v4l2_cap_2",
	},
#endif
};

static struct v4l2_int_device mt9v111_int_device [] = {
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
		{
			.module = THIS_MODULE,
			.type = v4l2_int_type_slave,
		},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
		{
			.module = THIS_MODULE,
			.type = v4l2_int_type_slave,
 		},
#endif
};

static int mt9v111_read_id( int sensoridx )
{
	int sensorid = 0;
	int ret = 0;

	mt9v111_write_reg (sensoridx,MT9V111S_ADDR_SPACE_SEL, 0x0004);

	sensorid = mt9v111_read_reg (sensoridx,MT9V111S_CHIP_VERSION);
	if( sensorid == 0x823a )
	{
		printk(KERN_INFO" MT9V111 ID %x\n",sensorid);
	}
	else
	{
		printk(KERN_ERR" MT9V111 Could not detect sensor (read %x)\n",sensorid);
		ret = -ENODEV;
	}
	return ret;
}

/*!
 * mt9v111 I2C probe function
 * Function set in i2c_driver struct.
 * Called by insmod mt9v111_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static int mt9v111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int retval;
	int sensorid;

	pr_debug("In mt9v111_probe  device id is %s\n", id->name);

	sensorid = mt9v111_id_from_name(id->name);

	if( sensorid < 0 )
		return -ENODEV;

	gpio_camera_active();

	/* Set initial values for the sensor struct. */
	memset(&mt9v111_data[sensorid], 0, sizeof(struct sensor));
	mt9v111_data[sensorid].pix.pixelformat = V4L2_PIX_FMT_UYVY;
	mt9v111_data[sensorid].pix.width = MT9V111_MAX_WIDTH;
	mt9v111_data[sensorid].pix.height = MT9V111_MAX_HEIGHT;
	mt9v111_data[sensorid].streamcap.capability = 0; /* No higher resolution or frame
						* frame rate changes supported.*/
	mt9v111_data[sensorid].streamcap.timeperframe.denominator = MT9V111_FRAME_RATE;
	mt9v111_data[sensorid].streamcap.timeperframe.numerator = 1;

	strcpy(mt9v111_int_device[sensorid].name,id->name);
	mt9v111_int_device[sensorid].u.slave = &mt9v111_slave[sensorid];
	mt9v111_data[sensorid].v4l2_int_device = &mt9v111_int_device[sensorid];
	mt9v111_int_device[sensorid].priv = &mt9v111_data[sensorid];
	mt9v111_data[sensorid].i2c_client = client;
	pr_debug("   client name is %s\n", client->name);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9v111_data structure here.*/
	/* We need to register the device even if the sensor is not detected,
	 * otherwise the camera2 would not be detected if the camera1 is not
	 * connected.
	 */
	retval = v4l2_int_device_register(&mt9v111_int_device[sensorid]);
	if( retval == 0 )
		mt9v111_data[sensorid].used = 1;

	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, sensorid , true, true);

	if( mt9v111_read_id(sensorid) != 0) {
		printk(KERN_ERR"mt9v111_probe: No sensor found\n");
		v4l2_int_device_unregister(&mt9v111_int_device[sensorid]);
		mt9v111_data[sensorid].used = 0;
		retval = -ENXIO;
	}
	else {
#ifdef MT9V111_DEBUG
		mt9v111_test_pattern(1);
#endif
		pr_debug("   type is %d (expect %d)\n",
			mt9v111_int_device[sensorid].type, v4l2_int_type_slave);
		pr_debug("   num ioctls is %d\n",
			mt9v111_int_device[sensorid].u.slave->num_ioctls);
	}

	ipu_csi_enable_mclk_if(CSI_MCLK_I2C, sensorid , false, false);

	return retval;
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod mt9v111_camera.ko
 */
static int mt9v111_remove(struct i2c_client *client)
{
	int i;

	pr_debug("In mt9v111_remove\n");

	for ( i=0 ; i < ARRAY_SIZE(mt9v111_int_device) ; i++ ) {
		if( mt9v111_data[i].used ){
			v4l2_int_device_unregister(&mt9v111_int_device[i]);
			mt9v111_data[i].used = 0;
		}
	}
	return 0;
}

/*!
 * MT9V111 init function.
 * Called by insmod mt9v111_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int mt9v111_init(void)
{
	u8 err;

	pr_debug("In mt9v111_init\n");

	/* Allocate memory for state structures. */
	mt9v111_device.coreReg = (mt9v111_coreReg *)
				kmalloc(sizeof(mt9v111_coreReg), GFP_KERNEL);
	if (!mt9v111_device.coreReg)
		return -1;
	memset(mt9v111_device.coreReg, 0, sizeof(mt9v111_coreReg));

	mt9v111_device.ifpReg = (mt9v111_IFPReg *)
				kmalloc(sizeof(mt9v111_IFPReg), GFP_KERNEL);
	if (!mt9v111_device.ifpReg) {
		kfree(mt9v111_device.coreReg);
		mt9v111_device.coreReg = NULL;
		return -1;
	}
	memset(mt9v111_device.ifpReg, 0, sizeof(mt9v111_IFPReg));

	/* Set contents of the just created structures. */
	mt9v111_config_datasheet();

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&mt9v111_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			   __func__, err);

	return err;
}

/*!
 * MT9V111 cleanup function.
 * Called on rmmod mt9v111_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit mt9v111_clean(void)
{
	pr_debug("In mt9v111_clean()\n");

	i2c_del_driver(&mt9v111_i2c_driver);

	if (mt9v111_device.coreReg) {
		kfree(mt9v111_device.coreReg);
		mt9v111_device.coreReg = NULL;
	}

	if (mt9v111_device.ifpReg) {
		kfree(mt9v111_device.ifpReg);
		mt9v111_device.ifpReg = NULL;
	}
}

module_init(mt9v111_init);
module_exit(mt9v111_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Mt9v111 Camera Driver");
MODULE_LICENSE("GPL");
