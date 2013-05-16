/******************************************************************************
 * dmieep.c - Linux kernel module for PEGATRON DMI EEPROM
 *
 * Copyright 2008-2011 Pegatron Corporation.
 *
 * DESCRIPTION:
 *  - This is the linux driver for PEGATRON DMI EEPROM 
 *
 * modification history
 * --------------------
 * v0.1   2010/11/02, Vincent Hao create this file
 * v0.2   2010/12/23, Tom Sun modify this file
 * v0.3   2011/02/23, Tom Sun modify this file
 * v0.4   2011/03/25, Add new ioctl functions for CPU revision and SKU id
 * v0.5   2011/03/29, Add new ioctl functions for calibrating
 * v0.6   2011/04/13, Export new function for WiFi country code
 * v0.7   2011/04/18, Add new ioctl functions for proximity
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h> /* for misc register, let user to use ioctl */
#include "dmieep.h"
#include "dmi_pri.h"
#include "dmi_api.h"
#include "dmi_pdf.h"

extern unsigned long long tegra_chip_uid(void);
extern int tegra_sku_id(void);
extern enum tegra_revision tegra_get_revision(void);

/* Size of EEPROM in bytes */
#define EEPROM_SIZE     256    

static unsigned long g_phys_addr;
static unsigned long g_phys_size;
/* data struct for dmieep device */
static struct dmieep_data_t    dmieep_data;


static int dmieep_update_data ( const unsigned char * tag_data, struct dmieep_data_t * data )
{
    int rv = 0;
    int data_size;
    u8 * pdata = data -> data; 
    Dmi_PassingData_Format      *ppdf;

    mutex_lock(&dmieep_data.mlock);
    // read data from tag given by bootloader
    ppdf = (Dmi_PassingData_Format*) tag_data;
    //printk(KERN_NOTICE DRIVER_NAME ": dmieep_update_data:byte count = [%d]\n", ppdf -> byte_count);
    data_size = ppdf -> byte_count - sizeof (int);
    memcpy ( pdata, ppdf -> data, data_size );
    //printk(KERN_NOTICE DRIVER_NAME ": dmieep_update_data: dump @ [%X][%X][%X][%X][%X][%X][%X][%X][%X][%X][%X][%X]\n", pdata[0],pdata[1],pdata[2],pdata[3],pdata[4],pdata[5],pdata[6],pdata[7],pdata[8],pdata[9],pdata[10],pdata[55]);

    data -> data_ready = 1;
    mutex_unlock(&dmieep_data.mlock);
    return rv;
}

static int __init dmieep_cld_arg(char *options)
{
    unsigned long start, size;
    char *p = options;

    start = -1;
    size = memparse(p, &p);
    if (*p == '@')
        start = memparse(p + 1, &p);
    g_phys_addr = (unsigned long) start;
    g_phys_size = (unsigned long) size;
    //printk(KERN_NOTICE DRIVER_NAME ": dmieep_cld_arg:start[%ld]size[%ld]\n", start, size);
    
    return 0;
}
__setup("nvmem=", dmieep_cld_arg);

static ssize_t bt_macid_show (struct device *d,
        struct device_attribute *attr, char *buf)
{
#if 1
    return sprintf(buf, "NA");
#else
    int k1, k2, len;
    char temp [ 256 ];
    char cbuf1 [ 16 ], cbuf2 [ 256 ];
 
    len = dmieep_read_data ( DMIEEP_INDEX_BTMAC, temp );
    if ( len<=0 )
        return sprintf(buf, "NA");
    *cbuf2=0;
    for ( k1=0; k1<len; k1++ )
    {
        k2 = temp[k1];
        sprintf(cbuf1, "%02X", k2 );
        strcat ( cbuf2, cbuf1 );
        if ( k1<len-1 )
            strcat ( cbuf2, ":" );
    }

    return sprintf(buf, "%s",cbuf2);
#endif
}

static ssize_t cap_sensor_data_show (struct device *d,
        struct device_attribute *attr, char *buf)
{
#if 1
    return sprintf(buf, "NA");
#else
    int k1, k2, len;
    char temp [ 256 ];
    char cbuf1 [ 16 ], cbuf2 [ 256 ];

    len = dmieep_read_data ( DMIEEP_INDEX_CAPSENSOR, temp );
    if ( len<=0 )
        return sprintf(buf, "NA");
    *cbuf2=0;
    for ( k1=0; k1<len; k1++ )
    {
        k2 = temp[k1];
        sprintf(cbuf1, "%02X", k2 );
        strcat ( cbuf2, cbuf1 );
        if ( k1<len-1 )
            strcat ( cbuf2, " " );
    }

    return sprintf(buf, "%s",cbuf2);
#endif
}

static ssize_t cap_sensor_data_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
    unsigned long lval;
    int ret, j, k, kp[32];
    char cbuf[32];

    if ( dmieep_data.writable != 1 )
        return -EINVAL;
    k = 4;
    ret = sscanf(buf, "%x %x %x %x", kp+0, kp+1, kp+2, kp+3);
    if (ret != k)
        return -EINVAL;    

    for ( j=0; j<k; j++ )
    {
        if ( kp[j] < 0 || kp[j] > 255 )
            return -EINVAL;
        cbuf[j]=(char) (kp[j]&0xFF);
    }
    dmieep_write_data ( DMIEEP_INDEX_CAPSENSOR, cbuf, k );
#endif
    return count;
}

static ssize_t writable_show (struct device *d,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", dmieep_data.writable );
}

static ssize_t writable_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    char *pc;

    pc = strstr ( buf, "PEGA" );
    if ( pc )
    {
        dmieep_data.writable = 1;
    }
    return count;
}

static ssize_t G_sensor_data_show (struct device *d,
        struct device_attribute *attr, char *buf)
{
    int k1, k2, len;
    char temp [ 256 ];
    char cbuf1 [ 16 ], cbuf2 [ 256 ];

    len = dmieep_read_data ( DMIEEP_INDEX_GSENSOR, temp );
    if ( len<=0 )
        return sprintf(buf, "NA");
    *cbuf2=0;
    for ( k1=0; k1<len; k1++ )
    {
        k2 = temp[k1];
        sprintf(cbuf1, "%02X", k2 );
        strcat ( cbuf2, cbuf1 );
        if ( k1<len-1 )
            strcat ( cbuf2, " " );
    }

    return sprintf(buf, "%s",cbuf2);
}

static ssize_t G_sensor_data_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret, j, k, kp[32];
    char cbuf[32];

    if ( dmieep_data.writable != 1 )
        return -EINVAL;    
    k = 12;
    ret = sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x", kp+0, kp+1, kp+2, kp+3, kp+4, kp+5, kp+6, kp+7, kp+8, kp+9, kp+10, kp+11);
    if (ret != k)
        return -EINVAL;

    for ( j=0; j<k; j++ )
    {
        if ( kp[j] < 0 || kp[j] > 255 )
            return -EINVAL;
        cbuf[j]=(char) (kp[j]&0xFF);
    }
    dmieep_write_data ( DMIEEP_INDEX_GSENSOR, cbuf, k );

    return count;
}

static ssize_t light_sensor_data_show (struct device *d,
        struct device_attribute *attr, char *buf)
{
    int k1, k2, len;
    char temp [ 256 ];
    char cbuf1 [ 16 ], cbuf2 [ 256 ];

    len = dmieep_read_data ( DMIEEP_INDEX_LIGHTSENSOR, temp );
    if ( len<=0 )
        return sprintf(buf, "NA");
    *cbuf2=0;
    for ( k1=0; k1<len; k1++ )
    {
        k2 = temp[k1];
        sprintf(cbuf1, "%02X", k2 );
        strcat ( cbuf2, cbuf1 );
        if ( k1<len-1 )
            strcat ( cbuf2, " " );
    }

    return sprintf(buf, "%s",cbuf2);
}

static ssize_t light_sensor_data_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret, j, k, kp[32];
    char cbuf[32];

    if ( dmieep_data.writable != 1 )
        return -EINVAL;
    
    k = 2;
    ret = sscanf(buf, "%x %x", kp+0, kp+1);
    if (ret != k)
        return -EINVAL;

    for ( j=0; j<k; j++ )
    {
        if ( kp[j] < 0 || kp[j] > 255 )
            return -EINVAL;
        cbuf[j]=(char) (kp[j]&0xFF);
    }
    dmieep_write_data ( DMIEEP_INDEX_LIGHTSENSOR, cbuf, k );

    return count;
}

static int dmieep_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int dmieep_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long dmieep_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int retval = 0, len;
    u64 chip_id = 0;
    char temp[256];
	
    switch (cmd)
    {
        case DMIEEP_G_DATA:
            chip_id = tegra_chip_uid();
            retval = copy_to_user(argp, &chip_id, sizeof(chip_id));
            break;
        case DMIEEP_G_REVISION:
            //printk(KERN_INFO "CPU Revision	: %s(%d)\n", tegra_revision_name[tegra_get_revision()], strlen(tegra_revision_name[tegra_get_revision()]));
            retval = copy_to_user(argp, tegra_revision_name[tegra_get_revision()], strlen(tegra_revision_name[tegra_get_revision()])+1);
            break;
        case DMIEEP_G_SKU:
            //printk(KERN_INFO "SKU ID	: %d\n", tegra_sku_id());
            retval = tegra_sku_id();
            break;
        case DMIEEP_G_GSENSOR:
            len = dmieep_read_data(DMIEEP_INDEX_GSENSOR, temp);
            if(len<=0)
                retval = -EIO;
            else
                retval = copy_to_user(argp, temp, len);
            break;
        case DMIEEP_S_GSENSOR:
            break;
        case DMIEEP_G_LIGHT:
            len = dmieep_read_data(DMIEEP_INDEX_LIGHTSENSOR, temp);
            if(len<=0)
                retval = -EIO;
            else
                retval = copy_to_user(argp, temp, len);
            break;
        case DMIEEP_S_LIGHT:
            break;
        case DMIEEP_G_PROXIMITY:
            len = dmieep_read_data(DMIEEP_INDEX_PROXIMITY, temp);
            if(len<=0)
                retval = -EIO;
            else
                retval = copy_to_user(argp, temp, len);
            break;
        case DMIEEP_S_PROXIMITY:
            break;

        default:
			retval = -EINVAL;
    }

    return retval;
}


static const struct file_operations dmieep_fops = {
	.owner	 = THIS_MODULE,
	.open	 = dmieep_open,
	.release = dmieep_close,
	.unlocked_ioctl	 = dmieep_ioctl,
};

static struct miscdevice dmieep_dev = {
	.name	= "dmieep",
	.fops	= &dmieep_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static DEVICE_ATTR(writable, S_IWUSR|S_IRUGO, writable_show, writable_store);
static DEVICE_ATTR(bt_mac_id, S_IRUGO, bt_macid_show, NULL);
static DEVICE_ATTR(cap_sensor_data, S_IWUSR|S_IRUGO, cap_sensor_data_show, cap_sensor_data_store);
static DEVICE_ATTR(G_sensor_data, S_IWUSR|S_IRUGO, G_sensor_data_show, G_sensor_data_store);
static DEVICE_ATTR(light_sensor_data, S_IWUSR|S_IRUGO, light_sensor_data_show, light_sensor_data_store);


static struct attribute *dmieep_sysfs_entries[] = {
    &dev_attr_writable.attr,
    &dev_attr_bt_mac_id.attr,
    &dev_attr_cap_sensor_data.attr,
    &dev_attr_G_sensor_data.attr,
    &dev_attr_light_sensor_data.attr,
    NULL
};

static struct attribute_group dmieep_attribute_group = {
    .name = NULL,       /* put in device directory */
    .attrs = dmieep_sysfs_entries,
};

struct tag_platform_data {
    int             num_data;
    unsigned char*  data;
};


static struct platform_device *dmieep_device;
static int __devinit dmieep_probe(struct platform_device *pdev)
{
	int rv = 0, ret;
    /* char temp [ 256 ]; */
    const struct tag_platform_data *antares_tag_platform_data;
    
    //printk(KERN_NOTICE DRIVER_NAME ": dmieep_probe() IN \n");
//    pr_err("dmieep - init device data struct... \n");
    
	/* initial device data struct */
	mutex_init(&dmieep_data.mlock);
    dmieep_data.data_ready = 0;
//    memset(dmieep_data.data, 0xFF, EEPROM_SIZE);  
    
    // read dmieep data from secondary memory device here
    antares_tag_platform_data = pdev->dev.platform_data;
    if(antares_tag_platform_data->num_data == 60)
        dmieep_update_data ( antares_tag_platform_data->data, & dmieep_data );

#if 0    
    for ( k1=DMIEEP_INDEX_MIN; k1<= DMIEEP_INDEX_MAX; k1++ )
    {
        printk(KERN_NOTICE DRIVER_NAME ": dmieep_probe() reading[%d]\n", k1-1);
        v = dmieep_read_data ( k1, temp );
        for ( k2=0; k2<v; k2++ )
            printk(KERN_NOTICE DRIVER_NAME ": data[%d]=[%d]\n", k2, temp[k2]);            
    }
#endif
#if 1
    dmieep_device = platform_device_register_simple("tegra_dmieep",
                -1, NULL, 0);
    if (IS_ERR(dmieep_device)) {
        printk(KERN_ERR "dmieep: platform_device_register failed.\n");
        ret = PTR_ERR(dmieep_device);
        goto out0;
    }

	ret = misc_register(&dmieep_dev);
	if (ret) {
		printk(KERN_ERR "dmiepp: failed to register misc, err=%d\n", rv);
		goto out1;
	}

    ret = sysfs_create_group(&dmieep_device->dev.kobj,
            &dmieep_attribute_group);
    if (ret) {
        printk(KERN_ERR "dmieep: failed to create sysfs device attributes.\n");
        goto out1;
    }
#endif
	//printk(KERN_NOTICE DRIVER_NAME ": dmieep_probe() OUT\n");
	return 0;
out1:
    platform_device_unregister(dmieep_device);
out0:
    return ret;
}

static int dmieep_remove(struct platform_device *pdev)
{
   	/* clean the dmieep data struct when dmieep device remove */
	misc_deregister(&dmieep_dev);
    sysfs_remove_group(&dmieep_device->dev.kobj, &dmieep_attribute_group);
    platform_device_unregister(dmieep_device);

    printk(KERN_INFO "dmieep is removed\n");

	return 0;
}


int dmieep_query_data ( int index )
// query data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0 : len of data buffer required
//          0 : data not ready
//         -1: no such data, index error
//         -2: internal error
{
    /* u8 * pdata = dmieep_data.data; */
    Dmi_Privdata_Ent    *pEnt;
    int idx, rv = 0;
    
    mutex_lock(&dmieep_data.mlock);
    if ( index < DMIEEP_INDEX_MIN || index > DMIEEP_INDEX_MAX )
    {
        rv = -1;
        goto err_exit;
    }
    idx = index - 1;
    if ( dmieep_data.data_ready != 1 )
    {
        rv = 0;
        goto err_exit;        
    }

    // data ready, query the data
    pEnt = &( Dmi_Privdata_Info[idx] );
    if ( ! pEnt )
    {
        rv = -2;
        goto err_exit;
    }    
    rv = pEnt -> len;
    
err_exit:
    mutex_unlock(&dmieep_data.mlock);
    return rv;
}

int dmieep_read_data ( int index, char * buf )
// actually read data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0: len of data been read, data has been filled in buf
//          0: data not ready
//         -1: parameter error
//         -2: no such data, index error
{
    u8 * pdata = dmieep_data.data;
    Dmi_Privdata_Ent    *pEnt;
    int offset, idx, rv = 0;

    if ( ! buf ) return -1;
    mutex_lock(&dmieep_data.mlock);
    if ( index < DMIEEP_INDEX_MIN || index > DMIEEP_INDEX_MAX )
    {
        rv = -1;
        goto err_exit;
    }
    idx = index - 1;
    if ( dmieep_data.data_ready != 1 )
    {
        rv = 0;
        goto err_exit;
    }

    // data ready, query the data
    pEnt = &( Dmi_Privdata_Info[idx] );
    if ( ! pEnt )
    {
        rv = -2;
        goto err_exit;
    }
    rv = pEnt -> len;
    offset = pEnt -> offset;
    memcpy ( buf, pdata+offset, rv );

err_exit:
    mutex_unlock(&dmieep_data.mlock);
    return rv;    
}

int dmieep_write_data ( int index, char * buf, int len )
// actually read data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0: len of data been written, should be equal to len
//          0: data not ready
//         -1: parameter error
//         -2: no such data, index error
{
    u8 * pdata = dmieep_data.data;
    Dmi_Privdata_Ent    *pEnt;
    int offset, idx, rv = 0;

    if ( ! buf ) return -1;
    mutex_lock(&dmieep_data.mlock);
    if ( index < DMIEEP_INDEX_MIN || index > DMIEEP_INDEX_MAX )
    {
        rv = -1;
        goto err_exit;
    }
    idx = index - 1;
    if ( dmieep_data.data_ready != 1 )
    {
        rv = 0;
        goto err_exit;
    }

    // data ready, query the data
    pEnt = &( Dmi_Privdata_Info[idx] );
    if ( ! pEnt )
    {
        rv = -2;
        goto err_exit;
    }
    if ( len < pEnt -> len )
    {
        rv = -1;
        goto err_exit;
    }    
    rv = pEnt -> len;
    offset = pEnt -> offset;
    memcpy ( pdata+offset, buf, rv );

err_exit:
    mutex_unlock(&dmieep_data.mlock);
    return rv;
}

void dmieep_get_wifi_countrycode(char *countrycode)
{
    int len;
    char temp[2];

    len = dmieep_read_data(DMIEEP_INDEX_WIFI_COUNTRYCODE, temp);
    if(len > 0)
        memcpy(countrycode, temp, 2);
}
EXPORT_SYMBOL(dmieep_get_wifi_countrycode);

static struct platform_driver dmieep_driver = {
    .probe      = dmieep_probe,
    .remove     = dmieep_remove,
    .driver     = {
        .name   = DRIVER_NAME,
    },
};

static int __init dmieep_init(void)
{
    int rv;
    printk(KERN_NOTICE DRIVER_NAME ": version %s loaded\n", DRIVER_VERSION);
    rv = platform_driver_register(&dmieep_driver);
    //printk(KERN_NOTICE DRIVER_NAME ": platform_driver_register rv=%d\n", rv);
    return rv;
}

static void __exit dmieep_exit(void)
{
    platform_driver_unregister(&dmieep_driver);
}

module_init(dmieep_init);
module_exit(dmieep_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("Vincent_Hao@pegatroncorp.com");
MODULE_LICENSE("GPL v2");

