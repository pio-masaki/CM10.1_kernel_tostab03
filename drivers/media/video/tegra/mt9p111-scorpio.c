/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9P111 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <media/mt9p111-scorpio.h>

#include "mt9p111_reg-scorpio.h"


#define AF_PROGRESS_ADDR        0xB006
#define OTPM_DATA_L_ADDR        0x380C
#define OTPM_DATA_H_ADDR        0x380E
#define OTPM_DATA_L_VAL         0x4090
#define OTPM_DATA_H_VAL         0x0000
#define AE_TRACK_FDZONE_ADDR	        0xA826
#define AE_TRACK_INT_TIME_LINES_ADDR    0xA83A
#define LINE_LENGTH_PCK_ADDR            0x300C
#define FINE_INTEGRATION_TIME_ADDR      0x3014
#define OUTPUT_CLK                      118

static struct mt9p111_info *info;
static int mode_table_status = MT9P111_MODE_UNINITED;
static int current_af_mode = 0;
static struct mt9p111_reg *config_table[] = {
	[MT9P111_COLOR_EFFECT_CONFIG] = color_effect_none,
	[MT9P111_WHITE_BALANCE_CONFIG] = white_balance_auto,
	[MT9P111_FPS_CONFIG] = fps_default,
	[MT9P111_FLASH_CONFIG] = Flash_AUTO,
	NULL
};


static int mt9p111_read_reg16(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u16 buf;
	int retry = 0;

	addr = swab16(addr);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &addr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *) &buf;

	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return swab16(buf);

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__, addr);
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return -EIO;
}

static int mt9p111_read_reg8(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u8 buf[3];
	int ret, retry = 0;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	/* high byte goes out first */
	buf[0] = (u8) (addr >> 8);;
	buf[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf + 2;

	do {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret >= 0) {
			memcpy(&ret, buf + 2, 1);
			return ret;
		}

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__, addr);
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return ret;
}

static int mt9p111_write_reg16(struct i2c_client *client, u16 addr, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int err, retry = 0;

	addr = swab16(addr);
	val = swab16(val);

	memcpy(buf, &addr, 2);
	memcpy(buf + 2, &val, 2);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = buf;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, add: 0x%x, val:0x%x\n",
		       __func__, addr, val);
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return err;
}

static int mt9p111_write_reg8(struct i2c_client *client, u16 addr, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int err, retry = 0;

	addr = swab16(addr);
	memcpy(buf, &addr, 2);
	buf[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, add: 0x%x, val:0x%x\n",
		       __func__, addr, val);
		msleep(MT9P111_MAX_WAITMS);
	} while (retry < MT9P111_MAX_RETRIES);

	return err;
}

static int mt9p111_poll_reg16(struct i2c_client *client, u16 addr, u8 expect_val)
{
	int i, ret;
	for (i = 0; i < MT9P111_POLL_RETRIES; i++) {
		ret = mt9p111_read_reg16(client, addr);
		if (ret == expect_val)
			return 0;

		msleep(MT9P111_POLL_WAITMS);
	}

	return -EINVAL;
}

static int mt9p111_poll_reg8(struct i2c_client *client, u16 addr, u8 expect_val)
{
	int i, ret;
	for (i = 0; i < MT9P111_POLL_RETRIES; i++) {
		ret = mt9p111_read_reg8(client, addr);
		if (ret == expect_val)
			return 0;

		msleep(MT9P111_POLL_WAITMS);
	}

	return -EINVAL;
}

static int mt9p111_write_table(struct i2c_client *client,
			       const struct mt9p111_reg table[])
{
	int err = -EIO;
	const struct mt9p111_reg *next;

	for (next = table; next->purpose != MT9P111_TABLE_END; next++) {
		switch (next->purpose) {
		case MT9P111_REG16:
			err = mt9p111_write_reg16(client, next->addr,
						  next->val);
			break;

		case MT9P111_REG8:
			err = mt9p111_write_reg8(client, next->addr, next->val);
			break;

		case MT9P111_POLL8:
			err = mt9p111_poll_reg8(client, next->addr, next->val);
			break;

		case MT9P111_POLL16:
			err = mt9p111_poll_reg16(client, next->addr, next->val);
			break;

		case MT9P111_WAIT_MS:
			msleep(next->val);
			break;

		default:
			pr_err("%s: invalid operation 0x%x\n", __func__,
			       next->purpose);
			break;
		}
	}

	return err;
}

static int mt9p111_set_mode(struct mt9p111_info *info,
			    struct mt9p111_mode *mode)
{
	int old_table = mode_table_status;
	int err, i;
	//int ret = 0 ;

	pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 2592 && mode->yres == 1944) {
		mode_table_status = MT9P111_MODE_2592x1944;
	} else if (mode->xres == 1280 && mode->yres == 960) {
		mode_table_status = MT9P111_MODE_1280x960;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	info->mode = mode_table_status;
	if (old_table == MT9P111_MODE_UNINITED) {
		err = mt9p111_write_table(info->i2c_client, mode_init_start);
		if (err)
			return err;

		/* Check OTPM */

		mt9p111_write_table(info->i2c_client, check_otpm);
		if (mt9p111_read_reg16(info->i2c_client,
			OTPM_DATA_L_ADDR) == OTPM_DATA_L_VAL &&
			mt9p111_read_reg16(info->i2c_client,
			OTPM_DATA_H_ADDR) == OTPM_DATA_H_VAL)
			err = mt9p111_write_table(info->i2c_client,
						  apga_otpm_lsc);
		else
			err = mt9p111_write_table(info->i2c_client,
						  apga_patch_ram_lsc);

		if (err)
			return err;

		err = mt9p111_write_table(info->i2c_client,mode_table[MT9P111_MODE_1280x960]);
		
	}
	else
		err = mt9p111_write_table(info->i2c_client,mode_table[mode_table_status]);

	if (err)
		return err;

	if (old_table == MT9P111_MODE_UNINITED) {
		for (i = 0; config_table[i]; i++)
			mt9p111_write_table(info->i2c_client, config_table[i]);		

	}

	return 0;
}

static long mt9p111_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mt9p111_info *info = file->private_data;
	struct mt9p111_mode mode;
	struct mt9p111_reg *config;
	//int t_brightness_val = 0 ;
    struct mt9p111_iso *iso;
    int ret =0; 
	int et = 0;


	switch (cmd) {
	case MT9P111_IOCTL_GET_FLASH_STATUS:
			ret = mt9p111_read_reg8(info->i2c_client, 0xC032) ;
		break ;
	case MT9P111_IOCTL_SET_MODE:
		if (copy_from_user(&mode, (const void __user *)arg,
				   sizeof(struct mt9p111_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = mt9p111_set_mode(info, &mode);
		break;

		
	case MT9P111_IOCTL_SET_FLASH_MODE:
		if(arg != MT9P111_FLASH_TORCH && arg != MT9P111_FLASH_TORCH_OFF)
			info->flash_mode  = arg;
		//pr_info("%s : MT9P111_IOCTL_SET_FLASH_MODE! arg = %d\n", __func__ , arg);
		switch (arg ) {
			case MT9P111_FLASH_AUTO :
				
				info->pdata->torch_off() ;
				//pr_info("%s : MT9P111_FLASH_AUTO! \n", __func__);
				config = Flash_AUTO; 
				break ;
			case MT9P111_FLASH_ON :
				
				info->pdata->torch_off() ;
				//pr_info("%s : MT9P111_FLASH_ON! \n", __func__);
				config = Flash_ON;  
				break ;
			case MT9P111_FLASH_OFF:
				
				info->pdata->torch_off() ;
				//pr_info("%s : MT9P111_FLASH_OFF! \n", __func__);
				config = Flash_OFF;  
				break ;
			case MT9P111_FLASH_TORCH:
				info->pdata->torch_on() ;
				break ;
			case MT9P111_FLASH_TORCH_OFF:
				info->pdata->torch_off() ;
				break ;

			}
		if (config) {
			if (mode_table_status == MT9P111_MODE_UNINITED)
				config_table[MT9P111_FLASH_CONFIG] = config;
			else
				ret = mt9p111_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}
		break ;

	case MT9P111_IOCTL_SET_COLOR_EFFECT:
		switch (arg & MT9P111_COLOR_EFFECT_MASK) {
		case MT9P111_COLOR_EFFECT_NONE:
			config = color_effect_none;
			break;

		case MT9P111_COLOR_EFFECT_MONO:
			config = color_effect_mono;
			break;

		case MT9P111_COLOR_EFFECT_SEPIA:
			config = color_effect_sepia;
			break;

		case MT9P111_COLOR_EFFECT_NEGATIVE:
			config = color_effect_negative;
			break;

		case MT9P111_COLOR_EFFECT_SOLARIZE:
			config = color_effect_solarize;
			break;

		default:
			config = NULL;
			break;
		}

		if (config) {
			if (mode_table_status == MT9P111_MODE_UNINITED)
				config_table[MT9P111_COLOR_EFFECT_CONFIG] = config;
			else
				ret = mt9p111_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}

		break;

	case MT9P111_IOCTL_SET_WHITE_BALANCE:
		switch (arg) {
		case MT9P111_WHITE_BALANCE_AUTO:
		//pr_info("%s : MT9P111_WHITE_BALANCE_AUTO! arg = %d\n", __func__ , arg);
			config = white_balance_auto;
			break;

		case MT9P111_WHITE_BALANCE_INCANDESCENT:
		//pr_info("%s : MT9P111_WHITE_BALANCE_INCANDESCENT! arg = %d\n", __func__ , arg);
			config = white_balance_incandescent;
			break;

		case MT9P111_WHITE_BALANCE_DAYLIGHT:
		//pr_info("%s : MT9P111_WHITE_BALANCE_DAYLIGHT! arg = %d\n", __func__ , arg);
			config = white_balance_daylight;
			break;

		case MT9P111_WHITE_BALANCE_FLUORESCENT:
		//pr_info("%s : MT9P111_WHITE_BALANCE_FLUORESCENT! arg = %d\n", __func__ , arg);
			config = white_balance_fluorescent;
			break;

		case MT9P111_WHITE_BALANCE_CLOUDY:
		//pr_info("%s : MT9P111_WHITE_BALANCE_CLOUDY! arg = %d\n", __func__ , arg);
			config = white_balance_cloudy;
			break;

		default:
			config = NULL;
			break;
		}

		if (config) {
			if (mode_table_status == MT9P111_MODE_UNINITED)
				config_table[MT9P111_WHITE_BALANCE_CONFIG] = config;
			else
				ret = mt9p111_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}

		break;

	case MT9P111_IOCTL_SET_EXPOSURE:
		//pr_info("%s : MT9P111_IOCTL_SET_EXPOSURE! arg = %d\n", __func__ , arg);
		switch ((arg >> MT9P111_EXPOSURE_SHIFT) & MT9P111_EXPOSURE_MASK) {
		case MT9P111_EXPOSURE_0:
			ret = mt9p111_write_table(info->i2c_client, exposure_0);
			break;

		case MT9P111_EXPOSURE_PLUS_1:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_plus_1);
			break;

		case MT9P111_EXPOSURE_PLUS_2:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_plus_2);
			break;

		case MT9P111_EXPOSURE_MINUS_1:
			ret = mt9p111_write_table(info->i2c_client,
						  exposure_minus_1);
			break;

		case MT9P111_EXPOSURE_MINUS_2:
			ret = mt9p111_write_table(info->i2c_client,

			exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_AF_MODE:
		//pr_info("%s : MT9P111_IOCTL_SET_AF_MODE! arg = %d\n", __func__ , arg);
                if (mode_table_status == MT9P111_MODE_UNINITED ||
                        current_af_mode == (unsigned int)arg)
                        break;

		switch ((unsigned int)arg) {
		case MT9P111_FOCUS_AUTO:
			//pr_info("%s : MT9P111_FOCUS_AUTO! arg = %d\n", __func__ , arg);
			ret = mt9p111_write_table(info->i2c_client, focus_auto);
			current_af_mode = MT9P111_FOCUS_AUTO;
			/*
			pr_info("%s : MT9P111_IOCTL_SET_AF_MODE!  %d\n", __func__ , arg);
			t_brightness_val = mt9p111_read_reg8(info->i2c_client, 0xA404) ;
			pr_info("%s : 0xA404 = %d\n", __func__ , t_brightness_val);
			if (t_brightness_val < 40 || t_brightness_val >= 65)
				ret = mt9p111_write_table(info->i2c_client, focus_auto);
			else
				ret = mt9p111_write_table(info->i2c_client, focus_init);
			*/
			break;


		case MT9P111_FOCUS_INFINITY:
			//pr_info("%s : MT9P111_FOCUS_INFINITY! arg = %d\n", __func__ , arg);
			ret = mt9p111_write_table(info->i2c_client,
						  focus_infinity);
			current_af_mode = MT9P111_FOCUS_INFINITY;
			break;

		case MT9P111_FOCUS_MACRO:
			//pr_info("%s : MT9P111_FOCUS_MACRO! arg = %d\n", __func__ , arg);
			ret = mt9p111_write_table(info->i2c_client,
						  focus_macro);
			current_af_mode = MT9P111_FOCUS_MACRO;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9P111_IOCTL_SET_AF_TRIGGER:
		//pr_info("%s : MT9P111_IOCTL_SET_AF_TRIGGER!  %d\n", __func__ , arg);
		/* MT9P111_FOCUS_TRIGGER = 2000 MT9P111_FOCUS_FALSH_TRIGGER = 3000 */
		/* Scene mode Sunset and Steady = 2000 other 3000 */
		//if (arg == MT9P111_FOCUS_TRIGGER || arg == MT9P111_FOCUS_FALSH_TRIGGER ||  arg == 4000)
		//if (arg == MT9P111_FOCUS_TRIGGER)
			ret = mt9p111_write_table(info->i2c_client,focus_trigger);

		break;

	case MT9P111_IOCTL_GET_AF_STATUS:
		ret = (mt9p111_read_reg8(info->i2c_client, 0xB006)) ? 0 : 1;
		//pr_info("%s : MT9P111_IOCTL_GET_AF_STATUS!  ret=%d\n", __func__ , ret);

		break;

	case MT9P111_IOCTL_SET_FPS:
		//pr_info("%s : MT9P111_IOCTL_SET_FPS!  ret=%d\n", __func__ , ret);
		switch (arg) {
		case MT9P111_FPS_MIN:
			config = fps_min;
			break;

		case MT9P111_FPS_MAX:
			config = fps_max;
			break;

		case MT9P111_FPS_MID:
		case MT9P111_FPS_DEFAULT:
			config = fps_default;
			break;

		default:
			config = NULL;
			break;
		}

		if (config) {
			if (mode_table_status == MT9P111_MODE_UNINITED)
				config_table[MT9P111_FPS_CONFIG] = config;
			else
				ret = mt9p111_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}

		break;
	case MT9P111_IOCTL_GET_ISO:
            //pr_info("%s : MT9P111_IOCTL_GET_ISO!  ret=%d\n", __func__ , ret);
            ret = mt9p111_read_reg16(info->i2c_client,
                                     AE_TRACK_FDZONE_ADDR);
            if (ret < 0)
                    break;
			
            for (iso = iso_table + 1; iso->value >= 0; iso++) {
                    struct mt9p111_iso *pre_iso;
					
                    if (iso->fdzone < ret)
                            continue;

                    pre_iso = iso - 1;
                    ret = (iso->value - pre_iso->value) *
                            (ret - pre_iso->fdzone) /
                            (iso->fdzone - pre_iso->fdzone) + pre_iso->value;
                    break;
            }
			break;
    case MT9P111_IOCTL_GET_EXPOSURE_TIME:
            //pr_info("%s : MT9P111_IOCTL_GET_EXPOSURE_TIME!  ret=%d\n", __func__ , ret);
            ret = mt9p111_read_reg16(info->i2c_client,
                                     AE_TRACK_INT_TIME_LINES_ADDR);
            if (ret < 0)
                    break;
            et = ret;
            ret = mt9p111_read_reg16(info->i2c_client,
                                     LINE_LENGTH_PCK_ADDR);
            if (ret < 0)
                    break;
            et = ret * et;
            ret = mt9p111_read_reg16(info->i2c_client,
                                   FINE_INTEGRATION_TIME_ADDR);
            if (ret < 0)
                    break;
            ret = (ret + et) / OUTPUT_CLK;
            break;			
	default:
		ret = -EINVAL;
		break;
	}

	//pr_info("%s : ret = %d\n", __func__  , ret);

	return ret;
}

static int mt9p111_open(struct inode *inode, struct file *file)
{
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	info->flash_mode = MT9P111_FLASH_AUTO ;

	mode_table_status = MT9P111_MODE_UNINITED;
	current_af_mode = 0;
	return 0;
}

static int mt9p111_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;
	return 0;
}


static const struct file_operations mt9p111_fileops = {
	.owner = THIS_MODULE,
	.open = mt9p111_open,
	.unlocked_ioctl = mt9p111_ioctl,
	.release = mt9p111_release,
};

static struct miscdevice mt9p111_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9P111_NAME,
	.fops = &mt9p111_fileops,
};

static int mt9p111_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	info = kzalloc(sizeof(struct mt9p111_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s : Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	err = misc_register(&mt9p111_device);
	if (err) {
		pr_err("%s : Unable to register misc device!\n", __func__);
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	return 0;
}

static int mt9p111_remove(struct i2c_client *client)
{
	struct mt9p111_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&mt9p111_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id mt9p111_id[] = {
	{ MT9P111_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mt9p111_id);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		.name = MT9P111_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p111_probe,
	.remove = mt9p111_remove,
	.id_table = mt9p111_id,
};

static int __init mt9p111_init(void)
{
	return i2c_add_driver(&mt9p111_i2c_driver);
}

static void __exit mt9p111_exit(void)
{
	i2c_del_driver(&mt9p111_i2c_driver);
}

module_init(mt9p111_init);
module_exit(mt9p111_exit);
