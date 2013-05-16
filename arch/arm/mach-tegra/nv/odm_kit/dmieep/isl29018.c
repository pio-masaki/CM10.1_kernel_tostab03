/*
 * A iio driver for the light sensor ISL 29018.
 *
 * Hwmon driver for monitoring ambient light intensity in luxi, proximity
 * sensing and infrared sensing.
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>

#include "../iio.h"

/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/

#define CONVERSION_TIME_MS		100

#define ISL29018_REG_ADD_COMMAND1	0x00
#define COMMMAND1_OPMODE_SHIFT		5
#define COMMMAND1_OPMODE_MASK		(7 << COMMMAND1_OPMODE_SHIFT)
#define COMMMAND1_OPMODE_POWER_DOWN	0
#define COMMMAND1_OPMODE_ALS_ONCE	1
#define COMMMAND1_OPMODE_IR_ONCE	2
#define COMMMAND1_OPMODE_PROX_ONCE	3

#define ISL29018_REG_ADD_COMMANDII	0x01
#define COMMANDII_RESOLUTION_SHIFT	2
#define COMMANDII_RESOLUTION_MASK	(0x3 << COMMANDII_RESOLUTION_SHIFT)

#define COMMANDII_RANGE_SHIFT		0
#define COMMANDII_RANGE_MASK		(0x3 << COMMANDII_RANGE_SHIFT)

#define COMMANDII_SCHEME_SHIFT		7
#define COMMANDII_SCHEME_MASK		(0x1 << COMMANDII_SCHEME_SHIFT)

#define ISL29018_REG_ADD_DATA_LSB	0x02
#define ISL29018_REG_ADD_DATA_MSB	0x03
#define ISL29018_MAX_REGS		ISL29018_REG_ADD_DATA_MSB

#define DEFAULT_RANGE           (64000)
#define DEFAULT_PROX_SCHEME     1
#define LIGHT_THREAD_TIMEOUT    (500)
#define DEBUGLOG                0
#if DEBUGLOG
#define pr_db(fmt, ...) \
        printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_db(fmt, ...)  ((void)0)
#endif

struct isl29018_chip {
	struct iio_dev		*indio_dev;
	struct i2c_client	*client;
    struct input_dev    *input_lux_dev;     /* input device for lux */
    struct input_dev    *input_prox_dev;    /* input device for prox */
    struct early_suspend early_suspend;
    struct task_struct  *task;
	struct mutex		lock;
	unsigned int		range;
	unsigned int		adc_bit;
    int                 enabled;
    int                 suspended;
    int                 on_before_suspend;  // remember the state before suspend
    u8                  pwr_status;
	int			        prox_scheme;
	u8			        reg_cache[ISL29018_MAX_REGS];
};

static int isl29018_suspend(struct i2c_client *client, pm_message_t mesg);
static int isl29018_resume(struct i2c_client *client);
static void isl29018_early_suspend(struct early_suspend *handler);
static void isl29018_early_resume(struct early_suspend *handler);

static bool isl29018_write_data(struct i2c_client *client, u8 reg,
			u8 val, u8 mask, u8 shift)
{
	u8 regval;
	int ret = 0;
	struct isl29018_chip *chip = i2c_get_clientdata(client);

	regval = chip->reg_cache[reg];
	regval &= ~mask;
	regval |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, regval);
	if (ret) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	chip->reg_cache[reg] = regval;
	return true;
}

static bool isl29018_set_range(struct i2c_client *client, unsigned long range,
		unsigned int *new_range)
{
	unsigned long supp_ranges[] = {1000, 4000, 16000, 64000};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_ranges) -1); ++i) {
		if (range <= supp_ranges[i])
			break;
	}
	*new_range = (unsigned int)supp_ranges[i];
	return isl29018_write_data(client, ISL29018_REG_ADD_COMMANDII,
			i, COMMANDII_RANGE_MASK, COMMANDII_RANGE_SHIFT);
}

static bool isl29018_set_resolution(struct i2c_client *client,
			unsigned long adcbit, unsigned int *conf_adc_bit)
{
	unsigned long supp_adcbit[] = {16, 12, 8, 4};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_adcbit)); ++i) {
		if (adcbit == supp_adcbit[i])
			break;
	}
	*conf_adc_bit = (unsigned int)supp_adcbit[i];
	return isl29018_write_data(client, ISL29018_REG_ADD_COMMANDII,
			i, COMMANDII_RESOLUTION_MASK, COMMANDII_RESOLUTION_SHIFT);
}

static int isl29018_read_sensor_input(struct i2c_client *client, int mode)
{
	bool status;
	int lsb;
	int msb;

	/* Set mode */
	status = isl29018_write_data(client, ISL29018_REG_ADD_COMMAND1,
			mode, COMMMAND1_OPMODE_MASK, COMMMAND1_OPMODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return -EBUSY;
	}
	msleep(CONVERSION_TIME_MS);
	lsb = i2c_smbus_read_byte_data(client, ISL29018_REG_ADD_DATA_LSB);
	if (lsb < 0) {
		dev_err(&client->dev, "Error in reading LSB DATA\n");
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29018_REG_ADD_DATA_MSB);
	if (msb < 0) {
		dev_err(&client->dev, "Error in reading MSB DATA\n");
		return msb;
	}
	dev_vdbg(&client->dev, "MSB 0x%x and LSB 0x%x\n", msb, lsb);
	return ((msb << 8) | lsb);
}

static bool isl29018_read_lux(struct i2c_client *client, int *lux)
{
	int lux_data;
	struct isl29018_chip *chip = i2c_get_clientdata(client);

	lux_data = isl29018_read_sensor_input(client, COMMMAND1_OPMODE_ALS_ONCE);
	if (lux_data > 0) {
		*lux = (lux_data * chip->range) >> chip->adc_bit;
		return true;
	}
	return false;
}

static bool isl29018_read_ir(struct i2c_client *client, int *ir)
{
	int ir_data;

	ir_data = isl29018_read_sensor_input(client, COMMMAND1_OPMODE_IR_ONCE);
	if (ir_data > 0) {
		*ir = ir_data;
		return true;
	}
	return false;
}

static bool isl29018_read_proximity_ir(struct i2c_client *client, int scheme,
		int *near_ir)
{
	bool status;
	int prox_data = -1;
	int ir_data = -1;

	/* Do proximity sensing with required scheme */
	status = isl29018_write_data(client, ISL29018_REG_ADD_COMMANDII,
			scheme, COMMANDII_SCHEME_MASK, COMMANDII_SCHEME_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return false;
	}
	prox_data = isl29018_read_sensor_input(client,
					COMMMAND1_OPMODE_PROX_ONCE);
	if (scheme == 1) {
		if (prox_data >= 0) {
			*near_ir = prox_data;
			return true;
		}
		return false;
	}
	if (prox_data >= 0)
		ir_data = isl29018_read_sensor_input(client,
					COMMMAND1_OPMODE_IR_ONCE);
	if (prox_data >= 0 && ir_data >= 0) {
		if (prox_data >= ir_data) {
			*near_ir = prox_data - ir_data;
			return true;
		}
	}
	return false;
}

static ssize_t get_sensor_data(struct device *dev, char *buf, int mode)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	int value = 0;
	bool status;

	mutex_lock(&chip->lock);
	switch (mode) {
		case COMMMAND1_OPMODE_PROX_ONCE:
			status = isl29018_read_proximity_ir(client,
					chip->prox_scheme, &value);
			break;

		case COMMMAND1_OPMODE_ALS_ONCE:
			status = isl29018_read_lux(client, &value);
			break;

		case COMMMAND1_OPMODE_IR_ONCE:
			status = isl29018_read_ir(client, &value);
			break;

		default:
			dev_err(&client->dev,"Mode %d is not supported\n",mode);
			mutex_unlock(&chip->lock);
			return -EBUSY;
	}
	if (!status) {
		dev_err(&client->dev, "Error in Reading data#1");
		mutex_unlock(&chip->lock);
		return -EBUSY;
	}

	mutex_unlock(&chip->lock);
	return sprintf(buf, "%d\n", value);
}

/* Sysfs interface */
/* range */
static ssize_t show_range(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", chip->range);
}

static ssize_t store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	bool status;
	unsigned long lval;
	unsigned int new_range;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;

	if (!(lval == 1000UL || lval == 4000UL ||
			lval == 16000UL || lval == 64000UL)) {
		dev_err(dev, "The range is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29018_set_range(client, lval, &new_range);
	if (!status) {
		mutex_unlock(&chip->lock);
		dev_err(dev, "Error in setting max range\n");
		return -EINVAL;
	}
	chip->range = new_range;
	mutex_unlock(&chip->lock);
	return count;
}

/* resolution */
static ssize_t show_resolution(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", chip->adc_bit);
}

static ssize_t store_resolution(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;
	struct i2c_client *client = chip->client;
	bool status;
	unsigned long lval;
	unsigned int new_adc_bit;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;
	if (!(lval == 4 || lval == 8 || lval == 12 || lval == 16)) {
		dev_err(dev, "The resolution is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29018_set_resolution(client, lval, &new_adc_bit);
	if (!status) {
		mutex_unlock(&chip->lock);
		dev_err(dev, "Error in setting resolution\n");
		return -EINVAL;
	}
	chip->adc_bit = new_adc_bit;
	mutex_unlock(&chip->lock);
	return count;
}

/* proximity scheme */
static ssize_t show_prox_scheme(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%d\n", chip->prox_scheme);
}

static ssize_t store_prox_scheme(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;
	unsigned long lval;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;
	if (!(lval == 0UL || lval == 1UL)) {
		dev_err(dev, "The scheme is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	chip->prox_scheme = (int)lval;
	mutex_unlock(&chip->lock);
	return count;
}

/* Read lux */
static ssize_t show_lux(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_ALS_ONCE);
}

/* Read ir */
static ssize_t show_ir(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_IR_ONCE);
}

/* Read nearest ir */
static ssize_t show_proxim_ir(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_PROX_ONCE);
}

/* Read name */
static ssize_t show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = indio_dev->dev_data;
	return sprintf(buf, "%s\n", chip->client->name);
}

/* enable */
static ssize_t show_enable_control(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct isl29018_chip *chip = indio_dev->dev_data;

    dev_vdbg(dev, "%s()\n", __func__);
    return sprintf(buf, "%d\n", chip->enabled);
}

static ssize_t store_enable_control(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct isl29018_chip *chip = indio_dev->dev_data;
    pm_message_t mesg;
    unsigned long lval;
    int enable_ctrl;

    dev_vdbg(dev, "%s()\n", __func__);

    if (strict_strtoul(buf, 10, &lval))
        return -EINVAL;
    if (!(lval == 0UL || lval == 1UL)) {
        dev_err(dev, "The scheme is not supported\n");
        return -EINVAL;
    }
    enable_ctrl = (int) lval;
    if ( chip->enabled != enable_ctrl )
    {
        if ( enable_ctrl == 0 && chip->suspended == 0 )
            isl29018_suspend(chip->client, mesg);
        else if ( enable_ctrl == 1 && chip->suspended == 1 )
            isl29018_resume(chip->client);
    }
    mutex_lock(&chip->lock);
    chip->prox_scheme = (int)lval;
    mutex_unlock(&chip->lock);
    return count;
}

static IIO_DEVICE_ATTR(range, S_IRUGO | S_IWUSR, show_range, store_range, 0);
static IIO_DEVICE_ATTR(resolution, S_IRUGO | S_IWUSR,
					show_resolution, store_resolution, 0);
static IIO_DEVICE_ATTR(proximity_scheme, S_IRUGO | S_IWUSR,
					show_prox_scheme, store_prox_scheme, 0);
static IIO_DEVICE_ATTR(lux, S_IRUGO, show_lux, NULL, 0);
static IIO_DEVICE_ATTR(ir, S_IRUGO, show_ir, NULL, 0);
static IIO_DEVICE_ATTR(proxim_ir, S_IRUGO, show_proxim_ir, NULL, 0);
static IIO_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
                    show_enable_control, store_enable_control, 0);

static struct attribute *isl29018_attributes[] = {
	&iio_dev_attr_name.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	&iio_dev_attr_resolution.dev_attr.attr,
	&iio_dev_attr_proximity_scheme.dev_attr.attr,
	&iio_dev_attr_lux.dev_attr.attr,
	&iio_dev_attr_ir.dev_attr.attr,
	&iio_dev_attr_proxim_ir.dev_attr.attr,
    &iio_dev_attr_enable.dev_attr.attr,
	NULL
};

static const struct attribute_group isl29108_group = {
	.attrs = isl29018_attributes,
};

static int isl29018_chip_init(struct i2c_client *client)
{
	struct isl29018_chip *chip = i2c_get_clientdata(client);
	bool status;
	int i;
	int new_adc_bit;
	unsigned int new_range;

	for (i = 0; i < ARRAY_SIZE(chip->reg_cache); i++) {
		chip->reg_cache[i] = 0;
	}

	/* set defaults */
	status = isl29018_set_range(client, chip->range, &new_range);
	if (status)
		status = isl29018_set_resolution(client, chip->adc_bit,
							&new_adc_bit);
	if (!status) {
		dev_err(&client->dev, "Init of isl29018 fails\n");
		return -ENODEV;
	}
	return 0;
}

static int tegra_light_thread(void *pdata)
{
    struct isl29018_chip *chip = (struct isl29018_chip *) pdata;
    struct i2c_client *client;
    int value = 0;
    bool status;

    while(1)
    {
        if ( ! chip->enabled )
        {
            pr_db("chip disabled, sleep and do nothing");
            msleep_interruptible (LIGHT_THREAD_TIMEOUT);
            goto next_iteration;
        }
//        set_current_state(TASK_RUNNING);
        client = chip->client;        
        // read lux
        mutex_lock(&chip->lock);
        status = isl29018_read_lux(client, &value);
        mutex_unlock(&chip->lock);
        if (!status) {
            pr_db("Error in Reading data#2");
            goto next_iteration;
        }
        pr_db("%s(%d) : lux = %d\n",__FUNCTION__,__LINE__, value);
        input_report_abs(chip->input_lux_dev, ABS_MISC, value);
        pr_db("%s(%d) : input_report_abs:lux()\n",__FUNCTION__,__LINE__);
        input_sync(chip->input_lux_dev);        
        
        // read prox
        mutex_lock(&chip->lock);
        status = isl29018_read_proximity_ir(client,
                    chip->prox_scheme, &value);
        mutex_unlock(&chip->lock);
        if (!status) {
            pr_db("Error in Reading data#3");
            goto next_iteration;
        }
        pr_db("%s(%d) : prox = %d\n",__FUNCTION__,__LINE__, value);
        input_report_abs(chip->input_prox_dev, ABS_DISTANCE, value);
        pr_db("%s(%d) : input_report_abs:prox()\n",__FUNCTION__,__LINE__);
        input_sync(chip->input_prox_dev);
next_iteration:
;
        msleep_interruptible (LIGHT_THREAD_TIMEOUT);
    }
    return 0;
}

static int __devinit isl29018_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct isl29018_chip *chip;
    struct input_dev *input_dev = NULL;    
	int err;

	chip = kzalloc(sizeof (struct isl29018_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, chip);
	chip->client = client;

	mutex_init(&chip->lock);

	chip->range = DEFAULT_RANGE; //1000;
	chip->adc_bit = 16;
    chip->prox_scheme= DEFAULT_PROX_SCHEME;

	err = isl29018_chip_init(client);
	if (err) {
        dev_err(&client->dev, "ISL29018 init fails\n");
		goto exit_free;
    }

	chip->indio_dev = iio_allocate_device();
	if (!chip->indio_dev) {
		dev_err(&client->dev, "iio allocation fails\n");
		goto exit_free;
	}
	chip->indio_dev->attrs = &isl29108_group;
	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->dev_data = (void *)(chip);
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_device_register(chip->indio_dev);
	if (err) {
		dev_err(&client->dev, "iio registration fails\n");
		goto exit_iio_free;
	}

    input_dev = input_allocate_device();
    if (input_dev == NULL) {
        err = -ENOMEM;
        pr_err("%s(%d) : Failed to allocate input lux device\n",__FUNCTION__,__LINE__);
        //goto allocate_dev_fail;
    }
    chip->input_lux_dev = input_dev;
    set_bit(EV_ABS, chip->input_lux_dev->evbit);
    input_set_abs_params(chip->input_lux_dev, ABS_MISC, 0*10000, 12*10000, 0, 0);
    chip->input_lux_dev->name = "light_tegra";
    err = input_register_device(chip->input_lux_dev);
    if (err) {
        pr_err("isl29018_probe: Unable to register %s\
                input lux device\n", chip->input_lux_dev->name);
        goto exit_iio_free;
    }

    input_dev = input_allocate_device();
    if (input_dev == NULL) {
        err = -ENOMEM;
        pr_err("%s(%d) : Failed to allocate input prox device\n",__FUNCTION__,__LINE__);
        //goto allocate_dev_fail;
    }
    chip->input_prox_dev = input_dev;
    set_bit(EV_ABS, chip->input_prox_dev->evbit);
    input_set_abs_params(chip->input_prox_dev, ABS_DISTANCE, 0, 1000, 0, 0);
    chip->input_prox_dev->name = "prox_tegra";
    err = input_register_device(chip->input_prox_dev);
    if (err) {
        pr_err("isl29018_probe: Unable to register %s\
                input prox device\n", chip->input_prox_dev->name);
        goto exit_iio_free;
    }
    
    //start the Int thread.
    chip->task = kthread_create(tegra_light_thread,
        chip, "tegra_light_thread");
    if (chip->task == NULL) {
        err = -1;
        pr_db("%s(%d) : chip->task == NULL\n",__FUNCTION__,__LINE__);
        goto exit_iio_free;
    }

    chip->early_suspend.suspend = isl29018_early_suspend;
    chip->early_suspend.resume = isl29018_early_resume;
    register_early_suspend(&(chip->early_suspend));
    chip->enabled=1;
    wake_up_process(chip->task);

    
	return 0;
exit_iio_free:
	iio_free_device(chip->indio_dev);
    if (chip->input_lux_dev)
        input_unregister_device(chip->input_lux_dev);
    if (chip->input_prox_dev)
        input_unregister_device(chip->input_prox_dev);
exit_free:
	kfree(chip);
exit:
	return err;
}

static int __devexit isl29018_remove(struct i2c_client *client)
{
	struct isl29018_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	iio_device_unregister(chip->indio_dev);
    input_unregister_device(chip->input_lux_dev);
    input_unregister_device(chip->input_prox_dev);
	kfree(chip);
	return 0;
}

static int isl29018_enable(struct isl29018_chip *chip, int enabled)
{
    int tobe_enabled =0;
    tobe_enabled = enabled?1:0;
    chip->enabled=tobe_enabled;
    return 0;
}

static int isl29018_set_suspend(struct isl29018_chip *chip, int suspend)
{
    int tobe_suspend =0;
    tobe_suspend = suspend?1:0;
    chip->suspended=tobe_suspend;
    return 0;
}

static void isl29018_early_suspend(struct early_suspend *handler)
{
    struct isl29018_chip *chip =
        container_of((struct early_suspend *)handler, struct isl29018_chip, early_suspend);
    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);
    chip->on_before_suspend = chip->enabled;
    isl29018_enable(chip, 0);
}

static void isl29018_early_resume(struct early_suspend *handler)
{
    struct isl29018_chip *chip =
        container_of((struct early_suspend *)handler, struct isl29018_chip, early_suspend);
    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);        
    if (chip->on_before_suspend) {
        isl29018_enable(chip, 1);
    }
}

#ifdef CONFIG_PM    /* if define power manager, define suspend and resume function */
static int isl29018_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct isl29018_chip *chip = i2c_get_clientdata(client);
    u32 buf_32;
    int rv;

    isl29018_enable(chip, 0);
    isl29018_set_suspend(chip, 1);
//    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    /* save power set now, and then set isl29018 to power down mode */
    buf_32 = i2c_smbus_read_byte_data(chip->client,0x00);
    if (buf_32 < 0) return -EBUSY;

    chip->pwr_status = (u8)buf_32 & 0xe0;
    rv = i2c_smbus_write_byte_data(client,0x00,(u8)buf_32 & (~0xe0));
    pr_db("%s(%d) : OK\n",__FUNCTION__,__LINE__);
//    kthread_stop(chip->task);
    return rv;
}

static int isl29018_resume(struct i2c_client *client)
{
    struct isl29018_chip *chip = i2c_get_clientdata(client);    
    u32 buf_32;
    u8  buf_8;
    int rv;

    isl29018_enable(chip, 1);
    isl29018_set_suspend(chip, 0);
//    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);
//    wake_up_process(chip->task);    
    /* resume the power staus of isl29018 */
    buf_32 = i2c_smbus_read_byte_data(chip->client,0x00);
    if (buf_32 < 0) return -EBUSY;

    buf_8 = (buf_32 & (~0xe0)) | chip->pwr_status;

    rv = i2c_smbus_write_byte_data(client,0x00,buf_8);
    pr_db("%s(%d) : OK\n",__FUNCTION__,__LINE__);    
    return rv;
}
#else
#define isl29018_suspend    NULL
#define isl29018_resume     NULL
#endif      /*ifdef CONFIG_PM end*/

static const struct i2c_device_id isl29018_id[] = {
	{"isl29018", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl29018_id);

static struct i2c_driver isl29018_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
			.name = "isl29018",
			.owner = THIS_MODULE,
		    },
	.probe	 = isl29018_probe,
	.remove	 = __devexit_p(isl29018_remove),
	.id_table = isl29018_id,
    .suspend        = isl29018_suspend,
    .resume         = isl29018_resume,
};

static int __init isl29018_init(void)
{
	return i2c_add_driver(&isl29018_driver);
}

static void __exit isl29018_exit(void)
{
	i2c_del_driver(&isl29018_driver);
}

module_init(isl29018_init);
module_exit(isl29018_exit);
