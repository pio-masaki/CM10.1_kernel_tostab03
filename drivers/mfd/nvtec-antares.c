#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <linux/mfd/core.h>
#include <linux/mfd/nvtec-antares.h>

#define NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT    (0xAA55)
#define NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG     (0xEFBE)
#define NVTEC_MAGIC_EXIT_FLASH_MODE             (0xAA55)

#define NVTEC_MAX_FW_SIZE               65536 /* 64K */
#define NVTEC_I2C_MAX_PACKET_SIZE       34

struct nvtec {
    struct mutex    lock;
    struct device   *dev;
    struct i2c_client *client;

    int request_pin;
    int ap_wake_pin;
    int flash_mode;
    int state;

    struct blocking_notifier_head undock_notifier;
    struct blocking_notifier_head system_notifier;
    struct blocking_notifier_head battery_notifier;
    struct blocking_notifier_head cir_notifier;

    wait_queue_head_t wq_ack;
};

enum nvtec_state {
    NVTEC_STATE_S0 = 0,
    NVTEC_STATE_S1 = 1,
    NVTEC_STATE_S2 = 2,
    NVTEC_STATE_S3 = 3,
    NVTEC_STATE_S4 = 4,

    NVTEC_STATE_TEST = 0xf0,
    NVTEC_STATE_RESET = 0xff,
};

enum nvtec_hdmi_switch_mode {
    NVTEC_HDMI_SW_OFF = 0,
    NVTEC_HDMI_SW_MB = 1,
    NVTEC_HDMI_SW_DOCK = 2,
    NVTEC_HDMI_SW_AUTO = 3,
    NVTEC_HDMI_SW_MAX,
};

static struct nvtec *nvtec = NULL;
static int nvtec_set_state(struct i2c_client *client, uint16_t state);
static int nvtec_read_event(struct i2c_client *client, uint16_t *event);

static int __nvtec_read_byte(struct i2c_client *client, int reg, uint8_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint8_t) ret;

    return (ret < 0) ? ret : 0;
}

static int __nvtec_read_word(struct i2c_client *client, int reg, uint16_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_word_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint16_t) ret;

    return (ret < 0) ? ret : 0;
}

static int __nvtec_read_block(struct i2c_client *client, int reg, 
                                     int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_byte(struct i2c_client *client, int reg, uint8_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%02x ot 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_word(struct i2c_client *client, int reg, uint16_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_word_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%04x to 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_block(struct i2c_client *client, int reg,
                                      int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed write to 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

int nvtec_read_byte(struct device *dev, int reg, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_byte);

int nvtec_read_word(struct device *dev, int reg, uint16_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_word);

int nvtec_read_block(struct device *dev, int reg, int len, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_block);

int nvtec_write_byte(struct device *dev, int reg, uint8_t val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_byte(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_byte);

int nvtec_write_word(struct device *dev, int reg, uint16_t val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_word);

int nvtec_write_block(struct device *dev, int reg, int len, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_block(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_block);

int nvtec_prepare_to_power_off(void)
{
    return nvtec_set_state(nvtec->client, NVTEC_STATE_S4);
}
EXPORT_SYMBOL_GPL(nvtec_prepare_to_power_off);

int nvtec_register_event_notifier(struct device *dev, 
                                  enum nvtec_event_type event,
                                  struct notifier_block *nb)
{
    int ret;

    switch (event) {
    case NVTEC_EVENT_UNDOCK:
        ret = blocking_notifier_chain_register(&nvtec->undock_notifier, nb);
        break;
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_register(&nvtec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_register(&nvtec->battery_notifier, nb);
        break;
    case NVTEC_EVENT_CIR:
        ret = blocking_notifier_chain_register(&nvtec->cir_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_register_event_notifier);

int nvtec_unregister_event_notifier(struct device *dev,
                                    enum nvtec_event_type event,
                                    struct notifier_block *nb)
{
    int ret;

    switch (event) {
    case NVTEC_EVENT_UNDOCK:
        ret = blocking_notifier_chain_unregister(&nvtec->undock_notifier, nb);
        break;
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_unregister(&nvtec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_unregister(&nvtec->battery_notifier, nb);
        break;
    case NVTEC_EVENT_CIR:
        ret = blocking_notifier_chain_unregister(&nvtec->cir_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_unregister_event_notifier);

static int nvtec_get_firmware_version(struct i2c_client *client, uint8_t *ver_str)
{
    int ret;
    char version[32];
    int len;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(client, 0xC0, 32, version);
    mutex_unlock(&nvtec->lock);

    if (ret < 0)
        return ret;

    len = (version[0] >= 31) ? 31 : version[0];
    strncpy(ver_str, &version[1], len);
    ver_str[len] = '\0';

    return ret;
}

static int cmd_ack = 0;

static int nvtec_enter_flash_mode(struct i2c_client *client, uint16_t magic)
{
    uint16_t event;
    int ret;

    if (nvtec->flash_mode) {
        printk("nvtec: already in flash mode\n");
        return 0;
    }

    if (nvtec->state != NVTEC_STATE_S0) {
        printk("nvtec: EC not in S0 state\n");
        return -EINVAL;
    }

    if (magic != NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT &&
        magic != NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG) {
        printk("nvtec: Invalid magic number\n");
        return -EINVAL;
    }

    mutex_lock(&nvtec->lock);

    /* clean event */
    while (gpio_get_value(nvtec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    printk("nvtec: enter flash mode\n");
    nvtec->flash_mode = 1;
    
    cmd_ack = 0;
    ret= __nvtec_write_word(client, 0xe0, magic);
    if (ret < 0) {
        nvtec->flash_mode = 0;
        mutex_unlock(&nvtec->lock);
        return ret;
    }
    
    wait_event(nvtec->wq_ack, cmd_ack == 1);

    mutex_unlock(&nvtec->lock);
    
    return 0;
}

static int nvtec_exit_flash_mode(struct i2c_client *client, uint16_t magic)
{
    int ret;

    if (nvtec->flash_mode == 0)
        return -EINVAL;

    if (magic != NVTEC_MAGIC_EXIT_FLASH_MODE) {
        printk("nvtec: Invalid magic number\n");
        return -EINVAL;
    }

    mutex_lock(&nvtec->lock);
    cmd_ack = 0;
    ret = __nvtec_write_word(client, 0xe3, magic);
    if (ret < 0) {
        mutex_unlock(&nvtec->lock);
        return ret;
    }

    wait_event(nvtec->wq_ack, cmd_ack == 1);

    printk("nvtec: exit flash mode\n");
    
    nvtec->flash_mode = 0;
    
    /* EC will reset itself when exit flash mode */
    /* after reset, EC at S4 state */
    nvtec->state = NVTEC_STATE_S4; 
    mutex_unlock(&nvtec->lock);

    nvtec_set_state(client, NVTEC_STATE_S0);

    return 0;
}

static int nvtec_get_flash_id(struct i2c_client *client, uint8_t *id)
{
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(client, 0xc1, 32, id);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_get_state(struct i2c_client *client, uint16_t *state)
{
    int ret;
    
    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(client, 0xd1, state);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_state(struct i2c_client *client, uint16_t state)
{
    int ret = 0;

    if (state > NVTEC_STATE_S4 &&
        state != NVTEC_STATE_TEST &&
        state != NVTEC_STATE_RESET)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    if (nvtec->state == NVTEC_STATE_S3 && /* S3->S0 */
        state == NVTEC_STATE_S0) {
        gpio_set_value(nvtec->request_pin, 1);
        msleep(1);
        gpio_set_value(nvtec->request_pin, 0);
        msleep(5);
    } else if (nvtec->state == NVTEC_STATE_S4 && /* S4->S0 */
               state == NVTEC_STATE_S0) {
        gpio_set_value(nvtec->request_pin, 0);
        msleep(1);
        gpio_set_value(nvtec->request_pin, 1);
        msleep(5);
    } else if (nvtec->state == NVTEC_STATE_S4 || /* S3<->S4, should not happen */
               nvtec->state == NVTEC_STATE_S3) {
        gpio_set_value(nvtec->request_pin, 0);
        msleep(20);
        gpio_set_value(nvtec->request_pin, 1);
        msleep(20);
        gpio_set_value(nvtec->request_pin, 0);
        msleep(5);
        ret = __nvtec_write_word(client, 0xd1, state);
        if (ret < 0)
            goto exit;
    } else {                    /* S0->S3 or S0->S4 */
        ret = __nvtec_write_word(client, 0xd1, state);
        if (ret < 0)
            goto exit;
    } 

    printk("nvtec: state=S%d\n", state);
    nvtec->state = state;
 exit:
    mutex_unlock(&nvtec->lock);
    return ret;
}

static int nvtec_get_hdmi_sw_mode(struct i2c_client *client, uint16_t *mode)
{
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(client, 0xd2, mode);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_hdmi_sw_mode(struct i2c_client *client, uint16_t mode)
{
    int ret;

    if (mode > NVTEC_HDMI_SW_MAX)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd2, mode);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}

static int nvtec_set_hdmi_p1_strength(struct i2c_client *client, int strength)
{
    int ret;
    uint16_t value;

    if (strength > 3)
        return -EINVAL;
    
    value = (0x0d << 8) | (strength & 0xff);

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd3, value);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_hdmi_p2_strength(struct i2c_client *client, int strength)
{
    int ret;
    uint16_t value;

    if (strength > 3)
        return -EINVAL;

    value = (0x0d << 8) | (strength & 0xff);

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd3, value);
    mutex_unlock(&nvtec->lock);

    return ret;
}

/* static int nvtec_reset(struct i2c_client *client) */
/* { */
/*     return nvtec_set_state(client, NVTEC_STATE_RESET); */
/* } */

/* static int nvtec_test_mode(struct i2c_client *client) */
/* { */
/*     return nvtec_set_state(client, NVTEC_STATE_TEST); */
/* } */

static int nvtec_read_gpio_input(struct i2c_client *client, uint8_t port, uint8_t *val)
{
    int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(client, 0xa0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_read_gpio_output(struct i2c_client *client, uint8_t port, uint8_t *val)
{
    int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(client, 0xb0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_gpio_output(struct i2c_client *client, uint8_t port, uint8_t val)
{
    int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_byte(client, 0xb0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_transfer_firmware(struct i2c_client *client, uint8_t *buffer, int len)
{
    int ret;
    int byte_transferred = 0;
    int transfer_len = 0;
    
    if (NULL == buffer || len != NVTEC_MAX_FW_SIZE)
        return -EINVAL;
    
    if (nvtec->flash_mode == 0)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    while (byte_transferred != len) {
        if ((len - byte_transferred) >= I2C_SMBUS_BLOCK_MAX)
            transfer_len = I2C_SMBUS_BLOCK_MAX;
        else
            transfer_len = len - byte_transferred;

        cmd_ack = 0;
        ret = __nvtec_write_block(client, 0xe2,
                                  transfer_len,
                                  &buffer[byte_transferred]);

        if (ret != 0) {
            mutex_unlock(&nvtec->lock);
            return -EINVAL;
        }

        byte_transferred += transfer_len;
        
        printk("nvtec: byte_transferred=%d\n", byte_transferred);
        wait_event(nvtec->wq_ack, cmd_ack == 1);

    }
    mutex_unlock(&nvtec->lock);

    return len;
}

static int nvtec_flash_acknowledge(struct i2c_client *client)
{
    uint8_t value;
    int ret;

    /* no need protect */
    ret = __nvtec_read_byte(client, 0x0, &value);
    if (ret < 0)
        return -EFAULT;

    if (value == 0xfa) {
        cmd_ack = 1;
        wake_up(&nvtec->wq_ack);
        
        return 0;
    } else
        return -EFAULT;
}

static int nvtec_read_event(struct i2c_client *client, uint16_t *event)
{
    /* no need protect */
    return __nvtec_read_word(client, 0xd0, event);
}

static int __remove_subdev(struct device *dev, void *unused)
{
    platform_device_unregister(to_platform_device(dev));
    return 0;
}

static int nvtec_remove_subdevs(struct nvtec *ec)
{
    return device_for_each_child(ec->dev, NULL, __remove_subdev);
}

static int __devinit nvtec_add_subdevs(struct nvtec *ec, struct nvtec_platform_data *pdata)
{
    struct nvtec_subdev_info *subdev;
    struct platform_device *pdev;
    int i, ret = 0;

    for (i = 0; i < pdata->num_subdevs; i ++) {
        subdev = &pdata->subdevs[i];
        
        pdev = platform_device_alloc(subdev->name, subdev->id);

        pdev->dev.parent = ec->dev;
        pdev->dev.platform_data = subdev->platform_data;

        ret = platform_device_add(pdev);
        if (ret)
            goto failed;
    }
    return 0;

 failed:
    nvtec_remove_subdevs(ec);
    return ret;
}

static irqreturn_t nvtec_irq(int irq, void *data)
{
    struct nvtec *ec = data;
    uint16_t event = 0;
    uint8_t event_type = 0;
    int ret;

    if (ec->flash_mode) {
        ret = nvtec_flash_acknowledge(ec->client);
        return IRQ_HANDLED;
    }
 
    nvtec_read_event(ec->client, &event);
    event_type = (uint8_t)(event >> 8) & 0xff;

    switch (event_type) {
    case NVTEC_EVENT_UNDOCK:
        blocking_notifier_call_chain(&ec->undock_notifier, event, NULL);
        break;
    case NVTEC_EVENT_CIR:
    case NVTEC_EVENT_CIR_REPEAT:
        blocking_notifier_call_chain(&ec->cir_notifier, event, NULL);
        break;
    case NVTEC_EVENT_BATTERY:
        blocking_notifier_call_chain(&ec->battery_notifier, event, NULL);
        break;
    case NVTEC_EVENT_SYSTEM:
        blocking_notifier_call_chain(&ec->system_notifier, event, NULL);
        break;
    default:
        break;
    }

    return IRQ_HANDLED;
}

static int __devinit nvtec_irq_init(struct nvtec *ec, int irq)
{
    int ret;

    ret = request_threaded_irq(irq, NULL, nvtec_irq, IRQF_TRIGGER_FALLING, 
                               "nvtec", ec);
    if (!ret) {
        device_init_wakeup(ec->dev, 1);
        enable_irq_wake(irq);
    }

    return ret;
}

static ssize_t nvtec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return -EINVAL;
}

static ssize_t nvtec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    uint8_t *fw_buf;
    
    if (count != NVTEC_MAX_FW_SIZE)
        return -EINVAL;

    fw_buf = kzalloc(NVTEC_MAX_FW_SIZE, GFP_KERNEL);
    if (!fw_buf) {
        return -ENOMEM;
    };
    memset(fw_buf, 0, NVTEC_MAX_FW_SIZE);

    if (copy_from_user(fw_buf, buf, count))
        goto error;

    ret = nvtec_transfer_firmware(nvtec->client, fw_buf, count);
    if (ret != count)
        goto error;

    kfree(fw_buf);
    return count;
 error:
    printk("nvtec: nvtec_write fail!\n");
    kfree(fw_buf);
    return -EINVAL;
}

struct flash_id {
    char _id[8];
};

#define NVTEC_IOCTL_ENTER_FLASH    _IOW('N', 1, unsigned short)
#define NVTEC_IOCTL_EXIT_FLASH     _IOW('N', 2, unsigned short)
#define NVTEC_IOCTL_SET_CONFIG     _IOW('N', 3, int)
#define NVTEC_IOCTL_GET_FLASH_ID   _IOR('N', 3, struct flash_id)

static long nvtec_ioctl(struct file *filp, unsigned int cmd, 
                       unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = -EINVAL;
    uint16_t magic;

    switch (cmd) {
    case NVTEC_IOCTL_ENTER_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_enter_flash_mode(nvtec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_EXIT_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_exit_flash_mode(nvtec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_SET_CONFIG:
        break;

    case NVTEC_IOCTL_GET_FLASH_ID:
        break;
    default:
        break;
    }

    return ret;

}

const struct file_operations nvtec_fops = {
    .owner = THIS_MODULE,
    .read = nvtec_read,
    .write = nvtec_write,
    .unlocked_ioctl = nvtec_ioctl,
};

#define NVTEC_MINOR   111
static struct miscdevice nvtec_dev = {
    .minor = NVTEC_MINOR,
    .name = "nvtec",
    .fops = &nvtec_fops,
};

static char *get_state_string(uint16_t state)
{
    switch (state) {
    case NVTEC_STATE_S0:
        return "S0";
    case NVTEC_STATE_S1:
        return "S1";
    case NVTEC_STATE_S2:
        return "S2";
    case NVTEC_STATE_S3:
        return "S3";
    case NVTEC_STATE_S4:
        return "S4";
    case NVTEC_STATE_TEST:
        return "Test";
    case NVTEC_STATE_RESET:
        return "Reset";
    default:
        return NULL;
    }
}

static int str2state(const char *state_str)
{
    if (NULL == state_str)
        return -1;

    if(!strncmp("S0", state_str, 2))
        return NVTEC_STATE_S0;
    if(!strncmp("S1", state_str, 2))
        return NVTEC_STATE_S1;
    if(!strncmp("S2", state_str, 2))
        return NVTEC_STATE_S2;
    if(!strncmp("S3", state_str, 2))
        return NVTEC_STATE_S3;
    if(!strncmp("S4", state_str, 2))
        return NVTEC_STATE_S4;

    if(!strncmp("Test", state_str, 4))
        return NVTEC_STATE_TEST;
    if(!strncmp("Reset", state_str, 5))
        return NVTEC_STATE_RESET;

    return -1;
}

static ssize_t nvtec_state_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    int ret;
    uint16_t state;

    ret = nvtec_get_state(nvtec->client, &state);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", get_state_string(state));
}

static ssize_t nvtec_state_store(struct device *dev, 
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    /* NvBool ret; */
    int ret;
    int state;

    state = str2state(buf);
    printk("state = %d\n", state);
    if (state < 0)
        return 0;
        
    ret = nvtec_set_state(nvtec->client, (uint16_t) state);
    if (ret < 0)
        return 0;
    
    return count;
}

static ssize_t nvtec_version_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    int ret;
    char version[32];
    
    ret = nvtec_get_firmware_version(nvtec->client, version);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", version);
}

static ssize_t nvtec_flash_id_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    int ret;
    char flash_id[32];
    
    ret = nvtec_get_flash_id(nvtec->client, flash_id);
    if (ret < 0)
        return 0;

    return sprintf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n0x%02x 0x%02x 0x%02x 0x%02x\n", 
                   flash_id[0], flash_id[1], flash_id[2], flash_id[3],
                   flash_id[4], flash_id[5], flash_id[6], flash_id[7]);
}


static char *get_mode_string(uint16_t mode)
{
    switch (mode) {
    case NVTEC_HDMI_SW_OFF:
        return "off";
    case NVTEC_HDMI_SW_MB:
        return "mb";
    case NVTEC_HDMI_SW_DOCK:
        return "dock";
    case NVTEC_HDMI_SW_AUTO:
        return "auto";
    default:
        return NULL;
    }
}

static int str2mode(const char *mode_str)
{
    if (NULL == mode_str)
        return -1;

    if(!strncmp("off", mode_str, 3))
        return NVTEC_HDMI_SW_OFF;
    if(!strncmp("mb", mode_str, 2))
        return NVTEC_HDMI_SW_MB;
    if(!strncmp("dock", mode_str, 4))
        return NVTEC_HDMI_SW_DOCK;
    if(!strncmp("auto", mode_str, 4))
        return NVTEC_HDMI_SW_AUTO;

    return -1;
}

static ssize_t nvtec_hdmi_switch_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint16_t mode;

    ret = nvtec_get_hdmi_sw_mode(nvtec->client, &mode);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", get_mode_string(mode));
}

static ssize_t nvtec_hdmi_switch_store(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    int ret;
    int mode;

    mode = str2mode(buf);
    if (mode < 0)
        return 0;

    ret = nvtec_set_hdmi_sw_mode(nvtec->client, (uint16_t) mode);
    if (ret < 0)
        return 0;
    
    return count;
}

static ssize_t nvtec_gpio_input_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_input(nvtec->client, i , &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_output(nvtec->client, i, &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_store(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    int ret;
    char *cp, *next;
    int port;
    int value;
    
    next = (char *)buf;

    cp = strsep(&next, " ");
    port = simple_strtoul(cp, NULL, 10);

    cp = strsep(&next, " ");
    value = simple_strtoul(cp, NULL, 16);

    ret = nvtec_set_gpio_output(nvtec->client, (uint8_t) port, (uint8_t) value & 0xff);
    if (ret < 0) {
        printk("NvtEC: can't set GPIO\n");
        return 0;
    }

    return count;
}

static ssize_t nvtec_cmd_write_store(struct device *dev, 
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    char *cp, *endp, *next;
    int cmd;
    uint8_t value[32];
    uint16_t word_value;
    int len = 0, i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;

    while (len < 32 && cp != endp) {
           cp = strsep(&next, " ");
           if (NULL == cp) {
               break;
           }
           value[len++] = simple_strtoul(cp, &endp, 16);
    }

    printk("cmd:0x%02x\n", cmd);
    printk("value: len=%d\n", len);
    for (i = 0; i < len; i++)
        printk("0x%02x\n", value[i]);
    
    if (len == 0)
        return count;

    mutex_lock(&nvtec->lock);
    if (len == 1) {
        __nvtec_write_byte(nvtec->client, cmd, value[0]);
    }else if (len == 2) {
        word_value = value[1] << 8 | value[0];
        __nvtec_write_word(nvtec->client, cmd, word_value);
    } else {
        __nvtec_write_block(nvtec->client, cmd, len, value);
    }
    mutex_unlock(&nvtec->lock);
    
    return count;

}

static uint8_t g_byte;
static uint16_t g_word;
static uint8_t g_value[32];
static int read_type = 0;

static ssize_t nvtec_cmd_read_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    char temp_buf[32];
    int i;

    if (read_type == 0)
        return strlen(buf);
    else if (read_type == -1) {
        sprintf(buf, "read byte: 0x%02x\n", g_byte);
    } else if (read_type == -2) {
        sprintf(buf, "read word: 0x%04x\n", g_word);
    } else {
        sprintf(temp_buf, "read block:\n");
        strcat(buf, temp_buf);
        
        for (i = 0; i <= read_type; i ++) {
            sprintf(temp_buf, "value[%d]=0x%02x\n", i, g_value[i]);
            strcat(buf, temp_buf);
        }
    }
             
    return strlen(buf);
}

static ssize_t nvtec_cmd_read_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp, *next;
    int cmd;
    int i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;
    printk("cmd: 0x%02x\n", cmd);

    cp = strsep(&next, " ");

    mutex_lock(&nvtec->lock);
    if (!strncmp(cp, "b", 1)) {
        __nvtec_read_byte(nvtec->client, cmd, &g_byte);
        printk("byte: 0x%02x\n", g_byte);
        read_type = -1;
    } else if (!strncmp(cp, "w", 1)) {
        __nvtec_read_word(nvtec->client, cmd, &g_word);
        printk("word: 0x%02x\n", g_word);
        read_type = -2;
    } else if  (!strncmp(cp, "B", 1)) {
        __nvtec_read_block(nvtec->client, cmd, 32, g_value);
        for (i = 0; i <= g_value[0]; i ++)
            printk("value[%d]: 0x%02x\n", i, g_value[i]);
        read_type = g_value[0];
    }
    mutex_unlock(&nvtec->lock);

    return count;
}

static ssize_t nvtec_hdmi_p1_driver_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp;
    int strength;

    cp = (char *)buf;
    strength = simple_strtoul(cp, &endp, 10);
    if (cp == endp) {
        return count;
    }

    if (strength > 3)
        return count;

    nvtec_set_hdmi_p1_strength(nvtec->client, strength);
    
    return count;
}

static ssize_t nvtec_hdmi_p2_driver_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp;
    int strength;
    
    cp = (char *)buf;
    strength = simple_strtoul(cp, &endp, 10);
    if (cp == endp) {
        return count;
    }

    if (strength > 3)
        return count;

    nvtec_set_hdmi_p2_strength(nvtec->client, strength);
    
    return count;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, nvtec_state_show, nvtec_state_store);
static DEVICE_ATTR(version, S_IRUGO, nvtec_version_show, NULL);
static DEVICE_ATTR(flash_id, S_IRUGO, nvtec_flash_id_show, NULL);
static DEVICE_ATTR(hdmi_sw, S_IWUSR | S_IRUGO, nvtec_hdmi_switch_show, nvtec_hdmi_switch_store);
static DEVICE_ATTR(gpio_in, S_IRUGO, nvtec_gpio_input_show, NULL);
static DEVICE_ATTR(gpio_out, S_IWUSR | S_IRUGO, nvtec_gpio_output_show, nvtec_gpio_output_store);
static DEVICE_ATTR(cmd_write, S_IWUSR, NULL, nvtec_cmd_write_store);
static DEVICE_ATTR(cmd_read, S_IWUSR | S_IRUGO, nvtec_cmd_read_show, nvtec_cmd_read_store);
static DEVICE_ATTR(hdmi_p1, S_IWUSR, NULL, nvtec_hdmi_p1_driver_store);
static DEVICE_ATTR(hdmi_p2, S_IWUSR, NULL, nvtec_hdmi_p2_driver_store);


static int nvtec_add_sysfs_entry(void)
{
    int ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_state);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_version);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_flash_id);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_sw);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_gpio_in);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_gpio_out);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_cmd_write);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_cmd_read);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_p1);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_p2);
    if (ret < 0)
        return ret;

    return 0;

}

static void nvtec_remove_sysfs_entry(void)
{
    device_remove_file(nvtec_dev.this_device, &dev_attr_state);
    device_remove_file(nvtec_dev.this_device, &dev_attr_version);
    device_remove_file(nvtec_dev.this_device, &dev_attr_flash_id);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_sw);
    device_remove_file(nvtec_dev.this_device, &dev_attr_gpio_in);
    device_remove_file(nvtec_dev.this_device, &dev_attr_gpio_out);
    device_remove_file(nvtec_dev.this_device, &dev_attr_cmd_write);
    device_remove_file(nvtec_dev.this_device, &dev_attr_cmd_read);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_p1);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_p2);
}


static int __devinit nvtec_i2c_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct nvtec_platform_data *pdata = client->dev.platform_data;
    struct nvtec *ec;
    uint16_t event;
    uint16_t state;
    char version[32];
    int ret = -EINVAL;

    if (!pdata) {
        dev_err(&client->dev, "nvtec requires platform data\n");
        return -ENOTSUPP;
    }

    ec = kzalloc(sizeof(struct nvtec), GFP_KERNEL);
    if (NULL == ec)
        return -ENOMEM;

    ec->request_pin = pdata->request_pin;
    ec->ap_wake_pin = pdata->ap_wake_pin;
    ec->client = client;
    ec->dev = &client->dev;
    i2c_set_clientdata(client, ec);

    mutex_init(&ec->lock);
    init_waitqueue_head(&ec->wq_ack);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->undock_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->system_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->battery_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->cir_notifier);
    
    nvtec = ec;

    if (client->irq) {
        ret = nvtec_irq_init(ec, client->irq);
        if (ret) {
            dev_err(&client->dev, "IRQ init failed: %d\n", ret);
            goto err_irq_init;
        }
    }

    /* make sure the EC state is S0, otherwise wake it up */
    ret = nvtec_get_state(client, &state);
    if ((ret < 0) || (ret == 0 && state != NVTEC_STATE_S0)) {
        dev_err(&client->dev, "EC is not in S0 state\n");
        gpio_set_value(nvtec->request_pin, 0);
        msleep(20);
        gpio_set_value(nvtec->request_pin, 1);
        msleep(20);
        gpio_set_value(nvtec->request_pin, 0);
        msleep(5);
    }

    /* clean event */
    while (gpio_get_value(ec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(ec->client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    nvtec_set_hdmi_sw_mode(client, NVTEC_HDMI_SW_AUTO);

    ret = nvtec_add_subdevs(ec, pdata);
    if (ret) {
        dev_err(&client->dev, "add devices failed: %d\n", ret);
        goto err_add_devs;
    }

    ret = misc_register(&nvtec_dev);
    if (ret) {
        dev_err(&client->dev, "fail to register misc device: %d\n", ret);
        goto err_misc_reg;
    }

    ret = nvtec_add_sysfs_entry();
    if (ret) {
        dev_err(&client->dev, "fail to add sysfs entry: %d\n", ret);
        goto err_add_sysfs;
    }

    

    ret = nvtec_get_firmware_version(client, version);
    if (ret < 0) {
        dev_err(&client->dev, "Unknown EC firmware version: %d\n", ret);
    } else {
        dev_info(&client->dev, "EC version: %s\n", version);
    }

    

    return 0;
 err_add_sysfs:
    misc_deregister(&nvtec_dev);
 err_misc_reg:
    nvtec_remove_subdevs(ec);
 err_add_devs:
    if (client->irq)
        free_irq(client->irq, ec);
 err_irq_init:
    kfree(ec);
    return ret;
}

static int __devexit nvtec_i2c_remove(struct i2c_client *client)
{
    struct nvtec *ec = i2c_get_clientdata(client);

    gpio_free(ec->request_pin);
    gpio_free(ec->ap_wake_pin);

    if (client->irq)
        free_irq(client->irq, ec);

    kfree(ec);

    nvtec_remove_sysfs_entry();

    misc_deregister(&nvtec_dev);

    return 0;
}

static int nvtec_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S3);
    if (ret < 0)
        return -EACCES;

    return 0;
}

static int nvtec_i2c_resume(struct i2c_client *client)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S0);
    if (ret < 0)
        return -EACCES;

    nvtec_set_hdmi_sw_mode(client, NVTEC_HDMI_SW_AUTO);

    return 0;
}

static void nvtec_i2c_shutdown(struct i2c_client *client)
{
    if (client->irq)
        disable_irq(client->irq);
    
    nvtec_set_state(client, NVTEC_STATE_S4);
}

static const struct i2c_device_id nvtec_id_table[] = {
    { "nvtec", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, nvtec_id_table);

static struct i2c_driver nvtec_driver = {
    .driver = {
        .name = "nvtec",
        .owner = THIS_MODULE,
    },
    .probe  = nvtec_i2c_probe,
    .remove = nvtec_i2c_remove,
    .suspend = nvtec_i2c_suspend,
    .resume = nvtec_i2c_resume,
    .shutdown = nvtec_i2c_shutdown,
    .id_table = nvtec_id_table,
};

static int __init nvtec_init(void)
{
    return i2c_add_driver(&nvtec_driver);
}
subsys_initcall(nvtec_init);

static void __exit nvtec_exit(void)
{
    i2c_del_driver(&nvtec_driver);
}
module_exit(nvtec_exit);

MODULE_DESCRIPTION("Nuvoton EC core driver");
MODULE_AUTHOR("Ron Lee <ron1_lee@pegatroncorp.com>");
MODULE_LICENSE("GPL");
