#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include "gpio-names.h"

struct pdata {
        int bus;
        int addr;
        int device;
        int cmd;
};

static struct pdata pdata;
static struct kobject *i2c_ctl_kobj;

static void query_i2c(void) {
        int err;
        int ret;
        struct i2c_adapter *adapter;
        union i2c_smbus_data data;

        adapter = i2c_get_adapter(pdata.bus);
        if (adapter == NULL) {
                printk("i2c_get_adapter fail\n");
                return;
        }
        
        gpio_set_value(TEGRA_GPIO_PJ0, 1);
        ret = gpio_get_value(TEGRA_GPIO_PJ0);
        printk("before send gpio : %d\n", ret);
        
        err = i2c_smbus_xfer(adapter, pdata.addr, 0, I2C_SMBUS_READ, pdata.cmd, I2C_SMBUS_BYTE_DATA, &data);
        if (err) {
                printk("i2c_smbus_xfer fail\n");
                return;
        }
 
        gpio_set_value(TEGRA_GPIO_PJ0, 0);
        ret = gpio_get_value(TEGRA_GPIO_PJ0);
        printk("after send gpio : %d\n", ret);
        
        printk("i2c_smbus_data read %d\n", data.byte);
        pdata.device = data.byte;

#if 0
        struct i2c_client *client;
        static struct i2c_board_info board_info; 
        board_info.addr = pdata.addr;
        strlcpy(board_info.type, "i2c_controller", I2C_NAME_SIZE);
        client = i2c_new_device(adapter, &board_info);
        if (client == NULL) {
                printk("i2c_new_device fail: %d\n",);
                return;
        }
        pdata.device_id = i2c_smbus_read_byte_data(client, 0x00);
        i2c_unregister_device(client);
#endif
}

static ssize_t sysfs_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
        ssize_t ret = 0;
        
        if(!strcmp(attr->attr.name, "bus")) {
                ret = sprintf(buf, "0x%02x\n", pdata.bus);
        }
        else if(!strcmp(attr->attr.name, "addr")) {
                ret = sprintf(buf, "0x%02x\n", pdata.addr);
        }
        else if(!strcmp(attr->attr.name, "device")) {
                query_i2c();
                ret = sprintf(buf, "0x%02x\n", pdata.device);
                pdata.device = 0;
        }
        else if(!strcmp(attr->attr.name, "cmd")) {
                ret = sprintf(buf, "0x%02x\n", pdata.cmd);
        }
        
        return ret;
}

static ssize_t sysfs_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
        long ret = 0;
  
        if(!strcmp(attr->attr.name, "bus")) {
                ret = simple_strtol(buf, NULL, 16);
                printk("Bus store 0x%02x\n", (unsigned int)ret);
                pdata.bus = ret;
        }
        else if(!strcmp(attr->attr.name, "addr")) {
                ret = simple_strtol(buf, NULL, 16);
                printk("Addr store 0x%02x\n", (unsigned int)ret);
                pdata.addr = ret;
        }
        else if(!strcmp(attr->attr.name, "cmd")) {
                ret = simple_strtol(buf, NULL, 16);
                printk("Cmd store 0x%02x\n", (unsigned int)ret);
                pdata.cmd = ret;
        }
	
        return count;
}

static ssize_t gpio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
        int ret = 0;
        ret = gpio_get_value(TEGRA_GPIO_PJ0);
        printk("get gpio value is %d\n", ret);
        return sprintf(buf, "%d\n", ret);
}

static ssize_t gpio_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
        int rev = 0;
        rev = simple_strtol(buf, NULL, 10);
        printk("set gpio value %d\n", rev);
        gpio_set_value(TEGRA_GPIO_PJ0, rev);
        return count;
}

static struct kobj_attribute bus =
        __ATTR(bus, 0660, sysfs_show, sysfs_store);   
static struct kobj_attribute addr =
        __ATTR(addr, 0660, sysfs_show, sysfs_store);          
static struct kobj_attribute device =
        __ATTR(device, 0440, sysfs_show, sysfs_store);
static struct kobj_attribute cmd =
        __ATTR(cmd, 0660, sysfs_show, sysfs_store);
static struct kobj_attribute gpio =
        __ATTR(gpio, 0660, gpio_show, gpio_store);
                

static int __init i2c_ctl_init(void) {
        int ret = 0;
  
        printk("%s", __FUNCTION__);
        
        i2c_ctl_kobj = kobject_create_and_add("i2c_controller", firmware_kobj);
        ret = sysfs_create_file(i2c_ctl_kobj, &bus.attr);
        ret = sysfs_create_file(i2c_ctl_kobj, &addr.attr);
        ret = sysfs_create_file(i2c_ctl_kobj, &device.attr);
        ret = sysfs_create_file(i2c_ctl_kobj, &cmd.attr);
        ret = sysfs_create_file(i2c_ctl_kobj, &gpio.attr);
        
        ///tegra_gpio_enable(TEGRA_GPIO_PJ0);
        ret = gpio_request(TEGRA_GPIO_PJ0, "i2c_controller");
        tegra_gpio_enable(TEGRA_GPIO_PJ0);
        if (ret < 0) {
                printk("%s failed to request gpio:%d\n", __FUNCTION__, ret);
        }

        ret = gpio_direction_output(TEGRA_GPIO_PJ0, 0);
        if (ret < 0) {
                printk("%s fail set gpio output 0", __FUNCTION__);
        }
        
        pdata.bus = 0x00;
        pdata.addr = 0x00;
        pdata.device = 0x00;
        pdata.cmd = 0x00;
        
        return ret;
}

static void __exit  i2c_ctl_exit(void) {
        printk("%s\n", __FUNCTION__);
  
        sysfs_remove_file(i2c_ctl_kobj, &bus.attr);
        sysfs_remove_file(i2c_ctl_kobj, &addr.attr);
        sysfs_remove_file(i2c_ctl_kobj, &device.attr);
        sysfs_remove_file(i2c_ctl_kobj, &cmd.attr);
        sysfs_remove_file(i2c_ctl_kobj, &gpio.attr);
        
        gpio_free(TEGRA_GPIO_PJ0);
        
        kobject_del(i2c_ctl_kobj);
}

module_init(i2c_ctl_init);
module_exit(i2c_ctl_exit);
MODULE_LICENSE("GPL");
