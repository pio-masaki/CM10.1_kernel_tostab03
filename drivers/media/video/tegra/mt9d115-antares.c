/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
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
#include <media/mt9d115.h>

#include "mt9d115_reg-antares.h"

#define AE_VIRT_GAIN_ADDR               0xA21C
#define AE_R9_ADDR                      0xA222
#define LINE_LENGTH_PCK_ADDR            0x300C
#define FINE_INTEGRATION_TIME_ADDR      0x3014
#define OUTPUT_CLK                      42

static struct mt9d115_info *info;
static int mode_table_status = MT9D115_MODE_UNINITED;
static struct mt9d115_reg *config_table[] = {
	[MT9D115_COLOR_EFFECT_CONFIG] = color_effect_none,
	[MT9D115_WHITE_BALANCE_CONFIG] = white_balance_auto,
	NULL
};

static int mt9d115_read_reg(struct i2c_client *client, u16 addr)
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

	/*
	 * if return value of this function is < 0,
	 * it mean error.
	 * else, under 16bit is valid data.
	 */

	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return swab16(buf);

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__, addr);
		msleep(MT9D115_MAX_WAITMS);
	} while (retry < MT9D115_MAX_RETRIES);

	return -EIO;
}

static int mt9d115_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int retry = 0;

	addr = swab16(addr);
	val = swab16(val);

	memcpy(buf + 0, &addr, 2);
	memcpy(buf + 2, &val, 2);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = buf;

	do {
		if (i2c_transfer(client->adapter, &msg, 1) == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, add: 0x%x, val:0x%x\n",
		       __func__, addr, val);
		msleep(MT9D115_MAX_WAITMS);
	} while (retry < MT9D115_MAX_RETRIES);

	return -EIO;
}

static int mt9d115_poll(struct i2c_client *client, u16 addr, u16 expect_val)
{
	int i;

	for (i = 0; i < MT9D115_POLL_RETRIES; i++) {
		if (addr <= 0x0100) {
			if (mt9d115_read_reg(client, addr) == expect_val)
				return 0;
		} else {
			mt9d115_write_reg(client, 0x098C, addr);
			if (mt9d115_read_reg(client, 0x0990) == expect_val)
				return 0;
		}

		msleep(MT9D115_POLL_WAITMS);
	}

	return -ETIME;
}

static int mt9d115_write_table(struct i2c_client *client,
			      const struct mt9d115_reg table[])
{
	int err = -EINVAL;
	const struct mt9d115_reg *next;

	for (next = table; next->purpose != MT9D115_TABLE_END; next++) {
		switch (next->purpose) {
		case MT9D115_REG:
			err = mt9d115_write_reg(client, next->addr, next->val);
			break;

		case MT9D115_POLL:
			err = mt9d115_poll(client, next->addr, next->val);
			break;

		case MT9D115_WAIT_MS:
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

static int mt9d115_set_mode(struct mt9d115_info *info,
			    struct mt9d115_mode *mode)
{
	int old_table = mode_table_status;
	int err, i;

	pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 1600 && mode->yres == 1200)
		mode_table_status = MT9D115_MODE_1600x1200;
	else if (mode->xres == 800 && mode->yres == 600)
		mode_table_status = MT9D115_MODE_800x600;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	info->mode = mode_table_status;
	if (old_table == MT9D115_MODE_UNINITED) {
		err = mt9d115_write_table(info->i2c_client, mode_init);
		if (err)
			return err;
	}

	err = mt9d115_write_table(info->i2c_client,
				  mode_table[mode_table_status]);
	if (err)
		return err;

	if (old_table == MT9D115_MODE_UNINITED) {
		for (i = 0; config_table[i]; i++)
			mt9d115_write_table(info->i2c_client, config_table[i]);
	}

	return 0;
}

static long mt9d115_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mt9d115_info *info = file->private_data;
	struct mt9d115_mode mode;
	struct mt9d115_reg *config;
	struct mt9d115_iso *iso;
	int ret, et;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case MT9D115_IOCTL_SET_MODE:
		if (copy_from_user(&mode,(const void __user *)arg,
				   sizeof(struct mt9d115_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = mt9d115_set_mode(info, &mode);
		break;

	case MT9D115_IOCTL_SET_COLOR_EFFECT:
		switch ((unsigned int)arg & MT9D115_COLOR_EFFECT_MASK) {
		case MT9D115_COLOR_EFFECT_NONE:
			config = color_effect_none;
			break;

		case MT9D115_COLOR_EFFECT_MONO:
			config = color_effect_mono;
			break;

		case MT9D115_COLOR_EFFECT_SEPIA:
			config = color_effect_sepia;
			break;

		case MT9D115_COLOR_EFFECT_NEGATIVE:
			config = color_effect_negative;
			break;

		case MT9D115_COLOR_EFFECT_SOLARIZE:
			config = color_effect_solarize;
			break;

		default:
			config = NULL;
			break;
		}

		if (config) {
			if (mode_table_status == MT9D115_MODE_UNINITED)
				config_table[MT9D115_COLOR_EFFECT_CONFIG] = config;
			else
				ret = mt9d115_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}

		break;

	case MT9D115_IOCTL_SET_WHITE_BALANCE:
		switch ((unsigned int)arg) {
		case MT9D115_WHITE_BALANCE_AUTO:
			config = white_balance_auto;
			break;

		case MT9D115_WHITE_BALANCE_INCANDESCENT:
			config = white_balance_incandescent;
			break;

		case MT9D115_WHITE_BALANCE_DAYLIGHT:
			config = white_balance_daylight;
			break;

		case MT9D115_WHITE_BALANCE_FLUORESCENT:
			config = white_balance_fluorescent;
			break;

		case MT9D115_WHITE_BALANCE_CLOUDY:
			config = white_balance_cloudy;
			break;

		default:
			config = NULL;
			break;
		}

		if (config) {
			if (mode_table_status == MT9D115_MODE_UNINITED)
				config_table[MT9D115_WHITE_BALANCE_CONFIG] = config;
			else
				ret = mt9d115_write_table(info->i2c_client,
							  config);
		} else {
			ret = -EINVAL;
		}

		break;

	case MT9D115_IOCTL_SET_EXPOSURE:
		switch ((int)arg) {
		case MT9D115_EXPOSURE_0:
			ret = mt9d115_write_table(info->i2c_client, exposure_0);
			break;

		case MT9D115_EXPOSURE_PLUS_1:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_plus_1);
			break;

		case MT9D115_EXPOSURE_PLUS_2:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_plus_2);
			break;

		case MT9D115_EXPOSURE_MINUS_1:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_minus_1);
			break;

		case MT9D115_EXPOSURE_MINUS_2:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9D115_IOCTL_GET_ISO:
		mt9d115_write_reg(info->i2c_client, 0x098C, AE_VIRT_GAIN_ADDR);
		ret = mt9d115_read_reg(info->i2c_client, 0x0990);
		if (ret < 0)
			break;

		for (iso = iso_table + 1; iso->value >= 0; iso++) {
			struct mt9d115_iso *pre_iso;

			if (iso->again < ret)
				continue;

			pre_iso = iso - 1;
			ret = (iso->value - pre_iso->value) *
				(ret - pre_iso->again) /
				(iso->again - pre_iso->again) + pre_iso->value;
			break;
		}

		break;

	case MT9D115_IOCTL_GET_EXPOSURE_TIME:
		mt9d115_write_reg(info->i2c_client, 0x098C, AE_R9_ADDR);
		ret = mt9d115_read_reg(info->i2c_client, 0x0990);
		if (ret < 0)
			break;

		et = ret * 625;
		ret = mt9d115_read_reg(info->i2c_client, LINE_LENGTH_PCK_ADDR);
		if (ret < 0)
			break;

		et = ret * et;
		ret = mt9d115_read_reg(info->i2c_client,
				       FINE_INTEGRATION_TIME_ADDR);
		if (ret < 0)
			break;

		ret = (ret + et) / OUTPUT_CLK;
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int mt9d115_open(struct inode *inode, struct file *file)
{
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	mode_table_status = MT9D115_MODE_UNINITED;
	return 0;
}

static int mt9d115_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;
	return 0;
}


static const struct file_operations mt9d115_fileops = {
	.owner = THIS_MODULE,
	.open = mt9d115_open,
	.unlocked_ioctl = mt9d115_ioctl,
	.release = mt9d115_release,
};

static struct miscdevice mt9d115_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9D115_NAME,
	.fops = &mt9d115_fileops,
};

static int mt9d115_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	info = kzalloc(sizeof(struct mt9d115_info), GFP_KERNEL);
	if (!info) {
		pr_err("mt9d115 : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&mt9d115_device);
	if (err) {
		pr_err("mt9d115 : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	mutex_init(&info->lock);
	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	return 0;
}

static int mt9d115_remove(struct i2c_client *client)
{
	struct mt9d115_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&mt9d115_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id mt9d115_id[] = {
	{ MT9D115_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mt9d115_id);

static struct i2c_driver mt9d115_i2c_driver = {
	.driver = {
		.name = MT9D115_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9d115_probe,
	.remove = mt9d115_remove,
	.id_table = mt9d115_id,
};

static int __init mt9d115_init(void)
{
	return i2c_add_driver(&mt9d115_i2c_driver);
}

static void __exit mt9d115_exit(void)
{
	i2c_del_driver(&mt9d115_i2c_driver);
}

module_init(mt9d115_init);
module_exit(mt9d115_exit);
