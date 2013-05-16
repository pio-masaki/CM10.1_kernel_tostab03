/*
 * drivers/misc/nct1008.c
 *
 * Driver for NCT1008, temperature monitoring device from ON Semiconductors
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/i2c.h>
//#include <linux/i2c/nct1008.h>
#include <linux/nct1008.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/err.h>
//#include <linux/device.h>
//#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h> /* for misc register, let user to use ioctl */
#include "nct1008_dev.h"

#define DEVICE_NAME     "nct1008"

#define DEBUGLOG 0
#define THOM_THREAD_TIMEOUT     (1000)
#define ENABLE_IDLE_TIMEOUT     0
#define THREAD_IDLE_TIMEOUT     (5*HZ) // 5 seconds
#define ENABLE_INPUT_EVENT      0

#if DEBUGLOG
#define pr_db(fmt, ...) \
        printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_db(fmt, ...)  ((void)0)
#endif

#define NCT1008_LOCAL_TEMP_RD			0x00
#define NCT1008_REMOTE_TEMPH_RD			0x01
#define NCT1008_REMOTE_TEMPL_RD			0x10
#define NCT1008_CONFIG_RD				0x03
#define NCT1008_MFR_ID_RD				0xFE

#define NCT1008_CONFIG_WR				0x09
#define NCT1008_CONV_RATE_WR			0x0A
#define NCT1008_OFFSET_WR				0x11
#define NCT1008_REMOTE_THERM_LIMIT_WR	0x19
#define NCT1008_LOCAL_THERM_LIMIT_WR	0x20

/* Configuration Register Bits */
#define EXTENDED_RANGE_BIT              (0x1 << 2)
#define THERM2_BIT                      (0x1 << 5)
#define STANDBY_BIT                     (0x1 << 6)

/* config params */
#define NCT1008_CONFIG_ALERT_DISABLE    0x80
#define NCT1008_CONFIG_RUN_STANDBY      0x40
#define NCT1008_CONFIG_ALERT_THERM2     0x20
#define NCT1008_CONFIG_ENABLE_EXTENDED  0x04

#define DRIVER_NAME "nct1008"

struct nct1008_data {
	struct device                   *hwmon_dev;
    struct mutex                    update_lock;  
    struct mutex                    cache_lock;     
	struct i2c_client               *client;   
	struct nct1008_platform_data    plat_data;
    struct task_struct              *task;
    struct input_dev                *input_dev; /* input devices */
    struct early_suspend            early_suspend;    
    int                             enabled;
    int                             on_before_suspend;
    u8                              dev_open_cnt;   
    int                             temp_save;
    int                             local_temp_save;    
    unsigned long                   idle_timeout;
    int                             now_throttling;
};

static struct nct1008_data * nct1008_chip;

static int nct1008_enable(struct nct1008_data *data, int enabled);
static void nct1008_early_suspend(struct early_suspend *handler);
static void nct1008_early_resume(struct early_suspend *handler);

#ifdef CONFIG_PM
static int nct1008_suspend(struct i2c_client *client, pm_message_t state);
static int nct1008_resume(struct i2c_client *client);
#endif

static void nct1008_reset_idle_timer ( struct nct1008_data * pdata )
{
pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);
        pdata -> idle_timeout = jiffies + THREAD_IDLE_TIMEOUT;
        pdata->enabled = 1;
}

static int nct1008_get_temp_val ( struct nct1008_data * pdata, int * val )
{
    struct i2c_client *client = pdata -> client;
    signed int temp_value = 0;
    u8 data = 0;

    if ( !pdata || !val )
        return -EINVAL;

    data = i2c_smbus_read_byte_data(client, NCT1008_LOCAL_TEMP_RD);
    if (data < 0) {
        dev_err(&client->dev, "%s: failed to read "
            "temperature\n", __func__);
        return -EINVAL;
    }

    temp_value = (signed int)data;
    * val = temp_value; 
    return 0;
}

static int nct1008_get_remote_temp_val (struct nct1008_data * pdata, int * val )
{
        /* int err; */
        struct i2c_client *client = pdata -> client;        
        u8 oldh, newh, l;
        u8 regh, regl;
        
        regh = NCT1008_REMOTE_TEMPH_RD;
        regl = NCT1008_REMOTE_TEMPL_RD;
        
        /*
         * There is a trick here. We have to read two registers to have the
         * sensor temperature, but we have to beware a conversion could occur
         * inbetween the readings. The datasheet says we should either use
         * the one-shot conversion register, which we don't want to do
         * (disables hardware monitoring) or monitor the busy bit, which is
         * impossible (we can't read the values and monitor that bit at the
         * exact same time). So the solution used here is to read the high
         * byte once, then the low byte, then the high byte again. If the new
         * high byte matches the old one, then we have a valid reading. Else
         * we have to read the low byte again, and now we believe we have a
         * correct reading.
         */        
        
        oldh = i2c_smbus_read_byte_data(client, regh );
        if ( oldh < 0 ) return oldh;
        l = i2c_smbus_read_byte_data(client, regl );  
        if ( l < 0 ) return l;
        newh = i2c_smbus_read_byte_data(client, regh );
        if ( newh < 0 ) return newh;  
        if ( oldh != newh )
        {
            l = i2c_smbus_read_byte_data(client, regl );  
            if ( l < 0 ) return l;        
        }
//        *val = (newh << 8) | l;
        *val = newh;
        return 0;

#if 0
        if ((err = lm90_read_reg(client, regh, &oldh))
         || (err = lm90_read_reg(client, regl, &l))
         || (err = lm90_read_reg(client, regh, &newh)))
                return err;
        if (oldh != newh) {
                err = lm90_read_reg(client, regl, &l);
                if (err)
                        return err;
        }
        *value = (newh << 8) | l;

        return 0;
#endif        
}

static ssize_t nct1008_show_temp(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nct1008_data *pdata = i2c_get_clientdata(client);
    signed int temp_value = 0;
    int rv;

    if (!dev || !buf || !attr)
        return -EINVAL;
    rv = 0;
    nct1008_reset_idle_timer (pdata);
    if ( pdata->enabled )
    {
        mutex_lock(&pdata->cache_lock);
        temp_value = pdata -> temp_save;
        mutex_unlock(&pdata->cache_lock);
    }
    else
    {
        mutex_lock(&pdata->update_lock);
//        rv = nct1008_get_temp_val ( pdata, &temp_value );        
        rv = nct1008_get_remote_temp_val ( pdata, &temp_value );         
        mutex_unlock(&pdata->update_lock);
    }
    if ( rv != 0 )
        return -EINVAL;        
    return sprintf(buf, "%d\n", temp_value);
}

static ssize_t nct1008_show_local_temp (struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct nct1008_data *pdata = i2c_get_clientdata(client);
    signed int temp_value = 0;
    int rv;

    if (!dev || !buf || !attr)
        return -EINVAL;
    rv = 0;
    nct1008_reset_idle_timer (pdata);
    if ( pdata->enabled )
    {
        mutex_lock(&pdata->cache_lock);
        temp_value = pdata -> local_temp_save;
        mutex_unlock(&pdata->cache_lock);
    }
    else
    {
        mutex_lock(&pdata->update_lock);
        rv = nct1008_get_temp_val ( pdata, &temp_value );        
//        rv = nct1008_get_remote_temp_val ( pdata, &temp_value );         
        mutex_unlock(&pdata->update_lock);
    }
    if ( rv != 0 )
        return -EINVAL;        
    return sprintf(buf, "%d\n", temp_value);
}


#if 0
static ssize_t nct1008_show_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	signed int temp_value = 0;
	u8 data = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	data = i2c_smbus_read_byte_data(client, NCT1008_LOCAL_TEMP_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"temperature\n", __func__);
		return -EINVAL;
	}

	temp_value = (signed int)data;
	return sprintf(buf, "%d\n", temp_value);
}
#endif

static ssize_t nct1008_show_enable(struct device *dev, struct device_attribute *devattr,
                 char *buf)
{
    struct i2c_client *client;
    struct nct1008_data *data;
    int temp;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    data = i2c_get_clientdata(client);
    
    temp = data->enabled;

    pr_db("%s(%d) : OUT %d\n",__FUNCTION__,__LINE__,temp);
    return sprintf(buf, "%d\n", temp);
}

static ssize_t nct1008_set_enable(struct device *dev, struct device_attribute *dummy,
                const char *buf, size_t count)
{
    struct i2c_client *client;
    struct nct1008_data *data;
    int val;    

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    data = i2c_get_clientdata(client);
    val = simple_strtol(buf, NULL, 10);

    mutex_lock(&data->update_lock);
    if(val==0)
    {
        pr_db("%s(%d) : val=%d\n",__FUNCTION__,__LINE__,val);
        nct1008_enable(data, 0);
    }
    else
    {
        pr_db("%s(%d) : val=%d\n",__FUNCTION__,__LINE__,val);
        nct1008_enable(data, 1);
    }
    mutex_unlock(&data->update_lock);
    return count;
}

static ssize_t nct1008_show_remote_crit(struct device *dev, struct device_attribute *devattr,
                 char *buf)
{
    struct i2c_client *client;
    struct nct1008_data *data;
    int temp;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    data = i2c_get_clientdata(client);
    
    temp = data->plat_data.shutdown_ext_limit;

    pr_db("%s(%d) : OUT %d\n",__FUNCTION__,__LINE__,temp);
    return sprintf(buf, "%d\n", temp);
}

static ssize_t nct1008_set_remote_crit(struct device *dev, struct device_attribute *dummy,
                const char *buf, size_t count)
{
    struct i2c_client *client;
    struct nct1008_data *pdata;
    int val;  
    int err;    
    u8  data = 0;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    pdata = i2c_get_clientdata(client);
    val = simple_strtol(buf, NULL, 10);
    data = val;
    
    mutex_lock(&pdata->update_lock);
    pdata->plat_data.shutdown_ext_limit = val;
    
    //err = i2c_smbus_write_byte_data(client, NCT1008_LOCAL_THERM_LIMIT_WR, data);      //vv      
    err = i2c_smbus_write_byte_data(client, NCT1008_REMOTE_THERM_LIMIT_WR, data);      //vv  
    if (err < 0) {      
        dev_err(&client->dev,"%s: failed to set THERM# limit\n", __func__);
        return 0;
    }    
    mutex_unlock(&pdata->update_lock);
    return count;
}

static ssize_t nct1008_show_local_crit(struct device *dev, struct device_attribute *devattr,
                 char *buf)
{
    struct i2c_client *client;
    struct nct1008_data *data;
    int temp;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    data = i2c_get_clientdata(client);
    
    temp = data->plat_data.shutdown_local_limit;

    pr_db("%s(%d) : OUT %d\n",__FUNCTION__,__LINE__,temp);
    return sprintf(buf, "%d\n", temp);
}

static ssize_t nct1008_set_local_crit(struct device *dev, struct device_attribute *dummy,
                const char *buf, size_t count)
{
    struct i2c_client *client;
    struct nct1008_data *pdata;
    int val;  
    int err;    
    u8  data = 0;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    pdata = i2c_get_clientdata(client);
    val = simple_strtol(buf, NULL, 10);
    data = val;
    
    mutex_lock(&pdata->update_lock);
    pdata->plat_data.shutdown_local_limit = val;
    
    //err = i2c_smbus_write_byte_data(client, NCT1008_LOCAL_THERM_LIMIT_WR, data);      //vv      
    err = i2c_smbus_write_byte_data(client, NCT1008_LOCAL_THERM_LIMIT_WR, data);      //vv  
    if (err < 0) {      
        dev_err(&client->dev,"%s: failed to set THERM# limit\n", __func__);
        return 0;
    }    
    mutex_unlock(&pdata->update_lock);
    return count;
}

static ssize_t nct1008_show_remote_throttling (struct device *dev, struct device_attribute *devattr,
                 char *buf)
{
    struct i2c_client *client;
    struct nct1008_data *data;
    int temp;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    data = i2c_get_clientdata(client);
    
    temp = data->plat_data.throttling_ext_limit;

    pr_db("%s(%d) : OUT %d\n",__FUNCTION__,__LINE__,temp);
    return sprintf(buf, "%d\n", temp);
}

static ssize_t nct1008_set_remote_throttling (struct device *dev, struct device_attribute *dummy,
                const char *buf, size_t count)
{
    struct i2c_client *client;
    struct nct1008_data *pdata;
    int val;  
    /* int err; */    
    u8  data = 0;

    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);    
    client = to_i2c_client(dev);
    pdata = i2c_get_clientdata(client);
    val = simple_strtol(buf, NULL, 10);
    data = val;
    
    mutex_lock(&pdata->update_lock);
    pdata->plat_data.throttling_ext_limit = val; 
    mutex_unlock(&pdata->update_lock);
    return count;
}

static DEVICE_ATTR(temperature, S_IRUGO, nct1008_show_temp, NULL);
static DEVICE_ATTR(local_temp, S_IRUGO, nct1008_show_local_temp, NULL);
static DEVICE_ATTR(enable, 0777, nct1008_show_enable, nct1008_set_enable);
static DEVICE_ATTR(local_critical, 0777, nct1008_show_local_crit, nct1008_set_local_crit);
static DEVICE_ATTR(remote_critical, 0777, nct1008_show_remote_crit, nct1008_set_remote_crit);
static DEVICE_ATTR(remote_throttling, 0777, nct1008_show_remote_throttling, nct1008_set_remote_throttling);

static struct attribute *nct1008_attributes[] = {
	&dev_attr_temperature.attr,
        &dev_attr_local_temp.attr,        
        &dev_attr_enable.attr,
        &dev_attr_local_critical.attr,         
        &dev_attr_remote_critical.attr,        
        &dev_attr_remote_throttling.attr,         
	NULL
};

static const struct attribute_group nct1008_group = {
	.attrs = nct1008_attributes,
};

//===============================================================
static int nct1008_open(struct inode *inode, struct file *filp)
{
    u8 i_minor;
    struct nct1008_data *p_data = nct1008_chip;

    i_minor = MINOR(inode->i_rdev);

    if ( ! p_data ) return -ENODEV;
    /* check open file conut */
    mutex_lock(&p_data->update_lock);
//    if ( i_minor != p_data->minor) goto no_dev_err;
    if ( p_data->dev_open_cnt > 0) goto has_open_err;
    p_data->dev_open_cnt++;
    mutex_unlock(&p_data->update_lock);

    filp->private_data = (void *)p_data;

    return 0;

//no_dev_err:
    mutex_unlock(&p_data->update_lock);
    return -ENODEV;
has_open_err:
    mutex_unlock(&p_data->update_lock);
    return -EMFILE;
}

static int nct1008_close(struct inode *inode, struct file *filp)
{
    u8 i_minor;
    struct nct1008_data *p_data;
    pm_message_t mesg;

    p_data = (struct nct1008_data *)filp->private_data;
    i_minor = MINOR(inode->i_rdev);

    mutex_lock(&(p_data->update_lock));
//    if ( i_minor != p_data->minor) goto no_dev_err;
    if (p_data->dev_open_cnt > 0)
        p_data->dev_open_cnt--;
    else {
        /* power off the nct1008 */
        mesg.event = PM_EVENT_SUSPEND;
        nct1008_suspend(p_data->client, mesg);
    }
    mutex_unlock(&(p_data->update_lock));

    return 0;

//no_dev_err:
    mutex_unlock(&(p_data->update_lock));
    return -ENODEV;
}

static long nct1008_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int value = 0;
    struct nct1008_data *p_data;

    p_data = (struct nct1008_data *)filp->private_data;
    if (p_data == NULL) return -EFAULT;

    switch (cmd)
    {
        case THOM_RD_TEMP:
            pr_db( "*** VV *** %s: RD_TEMP invoked *****\n", __FUNCTION__ );
            nct1008_get_temp_val ( p_data, & value );
            pr_db( "*** VV *** %s: RD_TEMP = %d *****\n", __FUNCTION__, (int)value );
            return put_user((u16)value, (u16 __user*)argp);

        case THOM_GET_ENABLE:
            pr_db( "*** VV *** %s: GET_ENABLE invoked *****\n", __FUNCTION__ );
            value = p_data -> enabled;
            return put_user((u16)value, (u16 __user*)argp);

        case THOM_SET_ENABLE:
            pr_db( "*** VV *** %s: SET_ENABLE = %d *****\n", __FUNCTION__, (int)arg );
            if (arg > 1)
                return -EINVAL;
            nct1008_enable(p_data, arg);
            break;

        default: return -EINVAL;
    }

    return 0;
}

static int nct1008_check_throttle ( struct nct1008_data * data, int temp )
{
      int throttling_temp;
      
      throttling_temp = data->plat_data.throttling_ext_limit;
      if ( temp < throttling_temp )
      {
          if ( data->now_throttling > 0 )
          {
              // temp under and is now throttling, cancel it
              mutex_lock(&data->update_lock);
                data->plat_data.alarm_fn ( false );
                data->now_throttling = 0;
              mutex_unlock(&data->update_lock);              
          }
          return 0;
      }
      // exceed throttling temperature, try throttling cpu frequency
      if ( ! data->plat_data.alarm_fn || data->now_throttling )
          return 0;     // no alarm_fn been set, or now is throttling, do it once
      
      mutex_lock(&data->update_lock);
          data->plat_data.alarm_fn ( true );
          data->now_throttling = 1;
      mutex_unlock(&data->update_lock);
      return 1;
}

static struct file_operations nct1008_fops = {
    .owner = THIS_MODULE,
    .open = nct1008_open,
    .release = nct1008_close,
    .unlocked_ioctl = nct1008_ioctl
};

static struct miscdevice nct1008_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &nct1008_fops,
};
//===============================================================

static int tegra_thom_thread(void *pdata)
{
    struct nct1008_data *data = (struct nct1008_data*)pdata;

    int temp;

    //temp = show_thom(data->hwmon_dev,&sensor_dev_attr_temp2_input.dev_attr);
    pr_db("%s(%d) : HZ=%d\n",__FUNCTION__,__LINE__, HZ);
    while(1)
    {
        if ( ! data->enabled )
        {
            pr_db("%s(%d) : chip disabled, sleep and do nothing\n",__FUNCTION__,__LINE__);
            msleep_interruptible (THOM_THREAD_TIMEOUT);
            goto next_iteration;
        }

        nct1008_get_temp_val ( data, & temp );
        mutex_lock(&data->cache_lock);
        data -> local_temp_save = temp;
        mutex_unlock(&data->cache_lock);
        pr_db("%s(%d) : local temp = %d\n",__FUNCTION__,__LINE__, temp);
        
        nct1008_get_remote_temp_val ( data, & temp );
        mutex_lock(&data->cache_lock);
        data -> temp_save = temp;
        mutex_unlock(&data->cache_lock);
        pr_db("%s(%d) : remote temp = %d\n",__FUNCTION__,__LINE__, temp);        
#if ENABLE_INPUT_EVENT
        input_report_abs(data->input_dev, ABS_THROTTLE, temp);
        input_sync(data->input_dev);
        pr_db("%s(%d) : input_report_abs()\n",__FUNCTION__,__LINE__);
#endif        
        
#if ENABLE_IDLE_TIMEOUT        
        pr_db("%s(%d) = [%d,%d]\n",__FUNCTION__,__LINE__, jiffies, data->idle_timeout);
        if ( time_after ( jiffies,data->idle_timeout ) ) // expired, disable thread by self
            data -> enabled = 0;        
#endif
        nct1008_check_throttle ( data, temp );

next_iteration:
        msleep_interruptible (THOM_THREAD_TIMEOUT);
    }
    return 0;
}

static int __devinit nct1008_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct nct1008_data *pdata;
    struct input_dev *input_dev = NULL;    
	int err;
	u8 data = 0;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL, exiting\n");
		return -ENODEV;
	}

	pdata = kzalloc(sizeof(struct nct1008_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "%s: failed to allocate "
			"device\n", __func__);
		return -ENOMEM;
	}
        mutex_init(&(pdata->update_lock)); 
        mutex_init(&(pdata->cache_lock));
	pdata->client = client;
	i2c_set_clientdata(client, pdata);
   

	memcpy(&pdata->plat_data, client->dev.platform_data,
		sizeof(struct nct1008_platform_data));

	data = i2c_smbus_read_byte_data(client, NCT1008_MFR_ID_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read manufacturer "
			"id\n", __func__);
		err = data;
		goto fail_alloc;
	}
	dev_info(&client->dev, "%s:0x%x chip found\n", client->name, data);

	/* set conversion rate (conv/sec) */
	data = pdata->plat_data.conv_rate;
	err = i2c_smbus_write_byte_data(client, NCT1008_CONV_RATE_WR, data);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set conversion "
			"rate\n", __func__);
		goto fail_alloc;
	}

	/* set config params */
//	data = pdata->plat_data.config;
        data = i2c_smbus_read_byte_data(client, NCT1008_CONFIG_RD);
        data &= ~EXTENDED_RANGE_BIT;
//        data = (STANDBY_BIT | THERM2_BIT);
	err = i2c_smbus_write_byte_data(client, NCT1008_CONFIG_WR, data);
        pr_db("%s(%d) : NCT1008_CONFIG_WR %d\n",__FUNCTION__,__LINE__,data);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		goto fail_alloc;
	}

	/* set offset value */
	data = pdata->plat_data.offset;
	err = i2c_smbus_write_byte_data(client, NCT1008_OFFSET_WR, data);
//        pr_db("%s(%d) : NCT1008_OFFSET_WR %d\n",__FUNCTION__,__LINE__,data);
	if (err < 0) {
		dev_err(&client->dev,
				"%s: failed to set offset\n", __func__);
		goto fail_alloc;
	}

	/* set cpu shutdown threshold */
        data = pdata->plat_data.shutdown_local_limit;
	err = i2c_smbus_write_byte_data(client, NCT1008_LOCAL_THERM_LIMIT_WR, data);
        if (err < 0) {
                dev_err(&client->dev,
                                "%s: failed to set THERM# limit\n", __func__);
                goto fail_alloc;
        }        
//        pr_db("%s(%d) : NCT1008_LOCAL_THERM_LIMIT_WR %d\n",__FUNCTION__,__LINE__,data);        
        data = pdata->plat_data.shutdown_ext_limit;        
        err = i2c_smbus_write_byte_data(client, NCT1008_REMOTE_THERM_LIMIT_WR, data);      //vv  
	if (err < 0) {
		dev_err(&client->dev,
				"%s: failed to set THERM# limit\n", __func__);
		goto fail_alloc;
	}

    /* Register sysfs hooks */
    if ((err = sysfs_create_group(&client->dev.kobj, &nct1008_group)))
        goto fail_alloc;
#if 0    
    if ((err = device_create_file(&client->dev, &dev_attr_enable)))
            goto exit_remove_files;
#endif    
    
	pdata->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(pdata->hwmon_dev)) {
		err = PTR_ERR(pdata->hwmon_dev);
		dev_err(&client->dev, "%s: hwmon_device_register "
			"failed\n", __func__);
		goto fail_alloc;
	}

    nct1008_chip = pdata;

    input_dev = input_allocate_device();
    if (input_dev == NULL) {
        err = -ENOMEM;
        dev_err(&client->dev, "%s: Failed to allocate input device\n ", __func__);
    }
    pdata->input_dev = input_dev;
//    set_bit(EV_KEY, data->input_dev->evbit);
    set_bit(EV_ABS, pdata->input_dev->evbit);
    input_set_abs_params(pdata->input_dev, ABS_THROTTLE, 0*1000, 150*1000, 0, 0);

    input_dev->name = "thom_tegra";

    err = input_register_device(input_dev);
    if (err) {
        pr_err("tegra_cap_probe: Unable to register %s\
                input device\n", input_dev->name);
        goto input_register_device_failed;
    }    
    
	/* register sysfs hooks */
#if 0    
	err = sysfs_create_group(&client->dev.kobj, &nct1008_group);
	if (err < 0)
		goto fail_sys;
#endif    

    err = misc_register(&nct1008_device);
    if (err) {
        printk(KERN_ERR
               "nct1008_probe: nct1008_device register failed\n");
        goto exit_remove_files;
    }
        //start the Int thread.
    pdata->task = kthread_create(tegra_thom_thread,
        pdata, "tegra_thom_thread");
    if (pdata->task == NULL) {
        err = -1;
        pr_err("%s(%d) : data->task == NULL\n",__FUNCTION__,__LINE__);
        //goto thread_create_failed;
    }

    pdata->early_suspend.suspend = nct1008_early_suspend;
    pdata->early_suspend.resume = nct1008_early_resume;
    register_early_suspend(&(pdata->early_suspend));

    pdata->now_throttling = 0;
    pdata->enabled=0;
    nct1008_reset_idle_timer (pdata);
    
    wake_up_process(pdata->task);

	dev_info(&client->dev, "%s: initialized\n", __func__);
	return 0;

exit_remove_files:
    input_unregister_device(input_dev);
    sysfs_remove_group(&client->dev.kobj, &nct1008_group);
    device_remove_file(&client->dev, &dev_attr_enable);
/* fail_sys: */
	hwmon_device_unregister(pdata->hwmon_dev);
input_register_device_failed:
    if ( input_dev )
        input_free_device(input_dev);    
fail_alloc:
	kfree(pdata);
	return err;
}

static int __devexit nct1008_remove(struct i2c_client *client)
{
	struct nct1008_data *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	hwmon_device_unregister(pdata->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj, &nct1008_group);
    device_remove_file(&client->dev, &dev_attr_enable);
    if ( pdata->input_dev )
        input_free_device(pdata->input_dev);     
	kfree(pdata);
	return 0;
}

static int nct1008_enable(struct nct1008_data *data, int enabled)
{
    int tobe_enabled =0;
    tobe_enabled = enabled?1:0;
    data->enabled=tobe_enabled;
    return 0;
}

static void nct1008_early_suspend(struct early_suspend *handler)
{
    struct nct1008_data *data =
        container_of((struct early_suspend *)handler, struct nct1008_data, early_suspend);
    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);
    data->on_before_suspend = data->enabled;
    nct1008_enable(data, 0);
}

static void nct1008_early_resume(struct early_suspend *handler)
{
    struct nct1008_data *data =
        container_of((struct early_suspend *)handler, struct nct1008_data, early_suspend);
    pr_db("%s(%d) : IN\n",__FUNCTION__,__LINE__);
    if (data->on_before_suspend) {
        nct1008_enable(data, 1);
    }
}

#ifdef CONFIG_PM
static int nct1008_suspend(struct i2c_client *client, pm_message_t state)
{
	u8 config;
	int err;

	config = i2c_smbus_read_byte_data(client, NCT1008_CONFIG_RD);
	if (config < 0) {
		dev_err(&client->dev, "%s: failed to read config\n", __func__);
		return -EIO;
	}

	/* take device to standby state */
	config |= NCT1008_CONFIG_RUN_STANDBY;

	err = i2c_smbus_write_byte_data(client, NCT1008_CONFIG_WR, config);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		return -EIO;
	}

	return 0;
}

static int nct1008_resume(struct i2c_client *client)
{
	u8 config = 0;
	int err;

	config = i2c_smbus_read_byte_data(client, NCT1008_CONFIG_RD);
	if (config < 0) {
		dev_err(&client->dev, "%s: failed to read config\n", __func__);
		return -EIO;
	}

	/* take device out of standby state */
	config &= ~NCT1008_CONFIG_RUN_STANDBY;

	err = i2c_smbus_write_byte_data(client, NCT1008_CONFIG_WR, config);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		return -EIO;
	}
	return 0;
}
#endif

static const struct i2c_device_id nct1008_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct1008_id);

static struct i2c_driver nct1008_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= nct1008_probe,
	.remove		= __devexit_p(nct1008_remove),
	.id_table	= nct1008_id,
#ifdef CONFIG_PM
	.suspend = nct1008_suspend,
	.resume = nct1008_resume,
#endif
};

static int __init nct1008_init(void)
{
	return i2c_add_driver(&nct1008_driver);
}

static void __exit nct1008_exit(void)
{
	i2c_del_driver(&nct1008_driver);
}

module_init (nct1008_init);
module_exit (nct1008_exit);

#define DRIVER_DESC "NCT1008 temperature sensor driver"
#define DRIVER_LICENSE "GPL"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);
