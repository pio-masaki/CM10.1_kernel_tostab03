/*
 * drivers/input/touchscreen/tegra_odm.c
 *
 * Touchscreen class input driver for platforms using NVIDIA's Tegra ODM kit
 * driver interface
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#define DRIVER_NAME	"egalax_i2c"
#define TOUCH_NAME "eGalax Inc. I2C TouchController"

#define KERNEL_LEVEL_DEBUG 0

/* the kernel supports 5 fingers only as of now */
#define SUPPORTED_FINGERS 4
#define TOOL_PRESSURE	-1

#define SUPPORT_USER_LEVEL		1
#define SUPPORT_IDLE_MODE		1

#define MAX_I2C_LEN		10

#define EGALAX_FULLPOWER	0
#define EGALAX_IDLE			1
#define EGALAX_SLEEP		2

#define REPORTID_MOUSE		0x01
#define REPORTID_VENDOR		0x03
#define REPORTID_MTOUCH		0x04

struct egalax_i2c_driver_data {
	struct i2c_client	*client;
    struct input_dev    *input_dev;
    struct task_struct  *task;
    struct semaphore sem;
    bool          bPollingMode;
    u32           pollingIntervalMS;
    u32           MaxX;
    u32           MinX;
    u32           MaxY;
    u32           MinY;
    int         shutdown;
	u32			power_mode;
    struct early_suspend    early_suspend;
	#if SUPPORT_IDLE_MODE
	struct timer_list timer;
	struct work_struct work;
	int idle_interval;
	int press_alive;
	#endif
	int Pressure[SUPPORTED_FINGERS];
};

static int egalax_i2c_read(struct i2c_client *client, char *pbuf);
static int egalax_i2c_write(struct i2c_client *client, const char *pbuf, int count);
static bool egalax_i2c_power (struct i2c_client *client, u8 mode);

#if SUPPORT_USER_LEVEL
#define USER_LEVEL_DEBUG 0
#if USER_LEVEL_DEBUG
    #define TS_DEBUG(fmt,args...)  printk( KERN_DEBUG "[egalax_i2c]: " fmt, ## args)
#else
    #define TS_DEBUG(fmt,args...)
#endif

#define FIFO_SIZE		PAGE_SIZE
#define MAX_READ_BUF_LEN	50

#define EGALAX_IOC_MAGIC	0x72
#define	EGALAX_IOCWAKEUP	_IO(EGALAX_IOC_MAGIC, 1)
#define EGALAX_IOC_MAXNR	1

struct egalax_char_dev {
    int OpenCnts;
    struct cdev cdev;
    struct egalax_i2c_driver_data *touch;
    //struct kfifo *pDataFiFo;
    struct kfifo DataFiFo;
    unsigned char *pFiFoBuf;
    spinlock_t FiFoLock;
    struct mutex lock;
    wait_queue_head_t fifo_inq;
};

static int global_major = 0; // dynamic major by default
static int global_minor = 0;
static char fifo_read_buf[MAX_READ_BUF_LEN];

static struct egalax_char_dev *p_char_dev = NULL;   // allocated in init_module
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static struct class *egalax_class;

static int egalax_cdev_open(struct inode *inode, struct file *filp)
{
    struct egalax_char_dev *pCharDev;

    pCharDev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
    if ( pCharDev == NULL ) {
        TS_DEBUG(" No such char device node \n");
        return -ENODEV;
    }

    if ( !atomic_dec_and_test(&egalax_char_available) ) {
        atomic_inc(&egalax_char_available);
        return -EBUSY; /* already open */
    }

    pCharDev->OpenCnts++;
    filp->private_data = pCharDev;// Used by the read and write metheds

    TS_DEBUG("egalax_cdev_open done \n");
    try_module_get(THIS_MODULE);
    return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp)
{
    struct egalax_char_dev *pCharDev; // device information
	unsigned long flags;

    pCharDev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
    if ( pCharDev == NULL ) {
        TS_DEBUG(" No such char device node \n");
        return -ENODEV;
    }

    atomic_inc(&egalax_char_available); /* release the device */

    filp->private_data = NULL;
    pCharDev->OpenCnts--;

	spin_lock_irqsave(&pCharDev->FiFoLock, flags);
    kfifo_reset( &pCharDev->DataFiFo );
	spin_unlock_irqrestore(&pCharDev->FiFoLock, flags);

    TS_DEBUG("egalax_cdev_release done \n");
    module_put(THIS_MODULE);
    return 0;
}

static ssize_t egalax_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    int read_cnt, ret;
    struct egalax_char_dev *pCharDev = file->private_data;

    if ( mutex_lock_interruptible(&pCharDev->lock) )
        return -ERESTARTSYS;

    while ( kfifo_len(&pCharDev->DataFiFo)<1 ) { /* nothing to read */
        mutex_unlock(&pCharDev->lock); /* release the lock */
        if ( file->f_flags & O_NONBLOCK )
            return -EAGAIN;

        if ( wait_event_interruptible(pCharDev->fifo_inq, kfifo_len( &pCharDev->DataFiFo )>0) )
            return -ERESTARTSYS; /* signal: tell the fs layer to handle it */

        if ( mutex_lock_interruptible(&pCharDev->lock) )
            return -ERESTARTSYS;
    }

    if (count > MAX_READ_BUF_LEN)
        count = MAX_READ_BUF_LEN;

    //read_cnt = kfifo_out(pCharDev->pDataFiFo, fifo_read_buf, count);
    read_cnt = kfifo_out_spinlocked(&pCharDev->DataFiFo, fifo_read_buf, count, &pCharDev->FiFoLock);
    ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

	#if USER_LEVEL_DEBUG
	{
		char cBuf[128];
		TS_DEBUG("egalax_cdev_read %d bytes.\n", read_cnt);
		sprintf(cBuf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ",
				fifo_read_buf[0]&0xFF, fifo_read_buf[1]&0xFF, fifo_read_buf[2]&0xFF, fifo_read_buf[3]&0xFF, fifo_read_buf[4]&0xFF,
				fifo_read_buf[5]&0xFF, fifo_read_buf[6]&0xFF, fifo_read_buf[7]&0xFF, fifo_read_buf[8]&0xFF, fifo_read_buf[9]&0xFF);
		TS_DEBUG("egalax_cdev_read = %s\n", cBuf);
    }
	#endif

    mutex_unlock(&pCharDev->lock);

    return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    struct egalax_char_dev *pCharDev = file->private_data;
    struct egalax_i2c_driver_data *touch = NULL;
	unsigned long flags;
    char i2c_data[10];
	int data_len;
    int ret=0;

	touch = pCharDev->touch;

	if(touch->power_mode != EGALAX_FULLPOWER)
		return -EFAULT;

    if ( mutex_lock_interruptible(&pCharDev->lock) )
        return -ERESTARTSYS;

    if (count > MAX_I2C_LEN)
        count = MAX_I2C_LEN;

    data_len = count;
    if (copy_from_user(i2c_data, buf, data_len)) {
        mutex_unlock(&pCharDev->lock);
        return -EFAULT;
    }

	#if USER_LEVEL_DEBUG
    {
        char cBuf[128];
		TS_DEBUG("egalax_cdev_write %d bytes.\n", data_len);
		sprintf(cBuf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ",
				i2c_data[0]&0xFF, i2c_data[1]&0xFF, i2c_data[2]&0xFF, i2c_data[3]&0xFF, i2c_data[4]&0xFF,
				i2c_data[5]&0xFF, i2c_data[6]&0xFF, i2c_data[7]&0xFF, i2c_data[8]&0xFF, i2c_data[9]&0xFF);
        TS_DEBUG("egalax_cdev_write = %s", cBuf);
    }
	#endif

	spin_lock_irqsave(&pCharDev->FiFoLock, flags);
    kfifo_reset( &pCharDev->DataFiFo );
	spin_unlock_irqrestore(&pCharDev->FiFoLock, flags);

    ret = egalax_i2c_write(touch->client, (const char *)i2c_data, data_len);

    mutex_unlock(&pCharDev->lock);

    return ret;
}

static long egalax_cdev_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
    struct egalax_char_dev *pCharDev = file->private_data;
    int rval = -EINVAL;

    switch (cmd) {
		case EGALAX_IOCWAKEUP:
			TS_DEBUG("egalax_cdev_ioctl wakeup done \n");
			egalax_i2c_power(pCharDev->touch->client, EGALAX_FULLPOWER);
			rval = 0;
			break;
		default:
			rval = -ENOTTY;
			break;
    }

    return rval;
}

static unsigned int egalax_cdev_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct egalax_char_dev *pCharDev = filp->private_data;
    unsigned int mask = 0;

    mutex_lock(&pCharDev->lock);
    poll_wait(filp, &pCharDev->fifo_inq,  wait);

    if ( kfifo_len(&pCharDev->DataFiFo) > 0 )
        mask |= POLLIN | POLLRDNORM;    /* readable */
    if ( (FIFO_SIZE - kfifo_len(&pCharDev->DataFiFo)) > MAX_I2C_LEN )
        mask |= POLLOUT | POLLWRNORM;   /* writable */

    mutex_unlock(&pCharDev->lock);
    return mask;
}

static const struct file_operations egalax_cdev_fops = {
    .owner  = THIS_MODULE,
    .read   = egalax_cdev_read,
    .write  = egalax_cdev_write,
    .unlocked_ioctl  = egalax_cdev_ioctl,
    .poll   = egalax_cdev_poll,
    .open   = egalax_cdev_open,
    .release= egalax_cdev_release,
};

static struct egalax_char_dev* setup_chardev(dev_t dev)
{
    struct egalax_char_dev *pCharDev;
    int result;

    pCharDev = kmalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
    if (!pCharDev)
        goto fail_cdev;
    memset(pCharDev, 0, sizeof(struct egalax_char_dev));

    spin_lock_init( &pCharDev->FiFoLock );
    pCharDev->pFiFoBuf = kmalloc(sizeof(unsigned char)*FIFO_SIZE, GFP_KERNEL);
    if (!pCharDev->pFiFoBuf)
        goto fail_fifobuf;
    memset(pCharDev->pFiFoBuf, 0, sizeof(unsigned char)*FIFO_SIZE);

    //pCharDev->pDataFiFo = kfifo_init(pCharDev->pFiFoBuf, FIFO_SIZE, GFP_KERNEL, &pCharDev->FiFoLock);
    kfifo_init(&pCharDev->DataFiFo, pCharDev->pFiFoBuf, FIFO_SIZE);
    //if ( pCharDev->pDataFiFo==NULL )
    //    goto fail_kfifo;

    pCharDev->OpenCnts = 0;
    cdev_init(&pCharDev->cdev, &egalax_cdev_fops);
    pCharDev->cdev.owner = THIS_MODULE;
    mutex_init(&pCharDev->lock);
    init_waitqueue_head(&pCharDev->fifo_inq);

    result = cdev_add(&pCharDev->cdev, dev, 1);
    if (result) {
        TS_DEBUG(KERN_ERR "Error cdev ioctldev added\n");
        goto fail_kfifo;
    }

    return pCharDev;

    fail_kfifo:
    kfree(pCharDev->pFiFoBuf);
    fail_fifobuf:
    kfree(pCharDev);
    fail_cdev:
    return NULL;
}

static void egalax_chrdev_exit(void)
{
    dev_t devno = MKDEV(global_major, global_minor);

    if (p_char_dev) {
        // Get rid of our char dev entries
        if ( p_char_dev->pFiFoBuf )
            kfree(p_char_dev->pFiFoBuf);

        cdev_del(&p_char_dev->cdev);
        kfree(p_char_dev);
    }

    unregister_chrdev_region( devno, 1);

    if (!IS_ERR(egalax_class)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
        class_device_destroy(egalax_class, devno);
#else
        device_destroy(egalax_class, devno);
#endif
        class_destroy(egalax_class);
    }
}

static int egalax_chrdev_init(struct i2c_client *client)
{
    struct egalax_i2c_driver_data *touch = i2c_get_clientdata(client);
    int result = 0;
    dev_t devno = 0;

    // Asking for a dynamic major unless directed otherwise at load time.
    if (global_major) {
        devno = MKDEV(global_major, global_minor);
        result = register_chrdev_region(devno, 1, "egalax_i2c");
    } else {
        result = alloc_chrdev_region(&devno, global_minor, 1, "egalax_i2c");
        global_major = MAJOR(devno);
    }

    if (result < 0) {
        TS_DEBUG(" egalax_i2c cdev can't get major number\n");
        return 0;
    }

    // allocate the character device
    p_char_dev = setup_chardev(devno);

    if (!p_char_dev) {
        result = -ENOMEM;
        goto fail;
    }
    p_char_dev->touch = touch;

    egalax_class = class_create(THIS_MODULE, "egalax_i2c");
    if (IS_ERR(egalax_class)) {
        TS_DEBUG("Err: failed in creating class.\n");
        result = -EFAULT;
        goto fail;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
    class_device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#else
    device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#endif
    TS_DEBUG("register egalax_i2c cdev, major: %d minor: %d\n", MAJOR(devno), MINOR(devno));
    return result;
    fail:
    egalax_chrdev_exit();
    return result;
}
#endif

static int egalax_i2c_read(struct i2c_client *client, char *pbuf)
{
	return i2c_master_recv(client, pbuf, MAX_I2C_LEN);
}

static int egalax_i2c_write(struct i2c_client *client, const char *pbuf, int count)
{
	return i2c_master_send(client, pbuf, count);
}

static bool egalax_i2c_power (struct i2c_client *client, u8 mode)
{
	struct egalax_i2c_driver_data *touch = i2c_get_clientdata(client);
	static const char egalax_idle_cmd[] = {0x03, 0x06, 0x0A, 0x04, 0x36, 0x3F, 0x01, 0x00, 0x00, 0x00};
	static const char egalax_sleep_cmd[] = {0x03, 0x06, 0x0A, 0x03, 0x36, 0x3F, 0x02, 0x00, 0x00, 0x00};
	int gpio;

	#if KERNEL_LEVEL_DEBUG
	printk(KERN_INFO "\n############ %s:%d\n", __FUNCTION__, mode);
	#endif

    switch (mode) {
		case EGALAX_FULLPOWER:	// 45mA
			if(touch->power_mode == EGALAX_FULLPOWER)
				return false;
			gpio = irq_to_gpio(client->irq);
			disable_irq(client->irq);
			gpio_direction_output(gpio, 1);
			udelay(50);
			gpio_direction_output(gpio, 0);
			udelay(50);
			gpio_direction_output(gpio, 1);
			msleep(50);
			enable_irq(client->irq);
			gpio_direction_input(gpio);
			msleep(100);
			break;
		case EGALAX_IDLE:	// 12mA
			if(touch->power_mode != EGALAX_FULLPOWER)
				return false;
			egalax_i2c_write(client, egalax_idle_cmd, sizeof(egalax_idle_cmd));
			break;
		case EGALAX_SLEEP:	// 180uA
			if(touch->power_mode != EGALAX_FULLPOWER)
				return false;
			egalax_i2c_write(client, egalax_sleep_cmd, sizeof(egalax_sleep_cmd));
			break;
		default:
			return false;
    }
	touch->power_mode = mode;

    return true;
}

static irqreturn_t egalax_i2c_interrupt(int irq, void *dev_id)
{
	struct egalax_i2c_driver_data *touch = (struct egalax_i2c_driver_data*)dev_id;

	up(&touch->sem);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void egalax_i2c_early_suspend(struct early_suspend *es)
{
    int i;
    struct egalax_i2c_driver_data *touch;
    touch = container_of(es, struct egalax_i2c_driver_data, early_suspend);

    if (touch) {
		#if SUPPORT_IDLE_MODE
		del_timer_sync(&touch->timer);
		cancel_work_sync(&touch->work);
		#endif
		egalax_i2c_power(touch->client, EGALAX_FULLPOWER);
        egalax_i2c_power(touch->client, EGALAX_SLEEP);

		for(i=0; i<SUPPORTED_FINGERS; i++) {
			touch->Pressure[i] = TOOL_PRESSURE;
			input_mt_sync(touch->input_dev);
		}
		input_sync(touch->input_dev);
    } else {
        pr_err("egalax_i2c_early_suspend: NULL handles passed\n");
    }
}

static void egalax_i2c_late_resume(struct early_suspend *es)
{
    struct egalax_i2c_driver_data *touch;
    touch = container_of(es, struct egalax_i2c_driver_data, early_suspend);

    if (touch) {
        egalax_i2c_power(touch->client, EGALAX_FULLPOWER);
		#if SUPPORT_IDLE_MODE
		mod_timer(&touch->timer,
				jiffies + msecs_to_jiffies(touch->idle_interval));
		#endif
    } else {
        pr_err("egalax_i2c_late_resume: NULL handles passed\n");
    }
}
#else
static int egalax_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
    struct egalax_i2c_driver_data *touch = i2c_get_clientdata(client);

    if (touch) {
		#if SUPPORT_IDLE_MODE
		del_timer_sync(&touch->timer);
		cancel_work_sync(&touch->work);
		#endif
		egalax_i2c_power(touch->client, EGALAX_FULLPOWER);
        egalax_i2c_power(touch->client, EGALAX_SLEEP);
        return 0;
    }
    pr_err("egalax_i2c_suspend: NULL handles passed\n");
    return -1;
}

static int egalax_i2c_resume(struct i2c_client *client)
{
    struct egalax_i2c_driver_data *touch = i2c_get_clientdata(client);

    if (touch) {
        egalax_i2c_power(touch->client, EGALAX_FULLPOWER);
		#if SUPPORT_IDLE_MODE
		mod_timer(&touch->timer,
				jiffies + msecs_to_jiffies(touch->idle_interval));
		#endif
        return 0;
    }
    pr_err("egalax_i2c_resume: NULL handles passed\n");
    return -1;
}
#endif

static int egalax_i2c_thread(void *pdata)
{
    struct egalax_i2c_driver_data *touch = (struct egalax_i2c_driver_data*)pdata;
    unsigned int x[SUPPORTED_FINGERS] = {0}, y[SUPPORTED_FINGERS] = {0}, i = 0;
	char i2c_data[MAX_I2C_LEN];
    unsigned int fingers = 0, data_len;

	for (i = 0; i < SUPPORTED_FINGERS; i++)
		touch->Pressure[i] = TOOL_PRESSURE;

    for (;;) {
        if (touch->bPollingMode)
            msleep(touch->pollingIntervalMS);
        else
            down(&touch->sem);

		#if KERNEL_LEVEL_DEBUG
		printk(KERN_INFO "###### eGalax INT \n");
		#endif

        if (MAX_I2C_LEN != (data_len = egalax_i2c_read(touch->client, i2c_data))) {
            pr_err("Couldn't read touch sample\n");
            continue;
        }

		if(touch->power_mode == EGALAX_SLEEP)
			continue;

		switch (i2c_data[0]) {
			case REPORTID_MOUSE:
				continue;
			case REPORTID_VENDOR:
				break;
			case REPORTID_MTOUCH:
			    fingers = (i2c_data[1] & 0x7c) >> 2;
				if(fingers > SUPPORTED_FINGERS-1)
					continue;

			    touch->Pressure[fingers] = i2c_data[1] & 0x1;
			    x[fingers] = ((u16)i2c_data[3] << 8) | (u16)i2c_data[2];
			    y[fingers] = ((u16)i2c_data[5] << 8) | (u16)i2c_data[4];

				#if 0
			    printk(KERN_DEBUG "fingers = %d, x=%d, y=%d, p=%d\n",
			            fingers, x[fingers], y[fingers], touch->Pressure[fingers]);
				#endif
			    break;
			default:
			    continue;
		};

		#if KERNEL_LEVEL_DEBUG
		printk(KERN_INFO "RawData = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				i2c_data[0]&0xFF, i2c_data[1]&0xFF, i2c_data[2]&0xFF, i2c_data[3]&0xFF, i2c_data[4]&0xFF,
				i2c_data[5]&0xFF, i2c_data[6]&0xFF, i2c_data[7]&0xFF, i2c_data[8]&0xFF, i2c_data[9]&0xFF);
		#endif

		#if SUPPORT_IDLE_MODE
		touch->power_mode = EGALAX_FULLPOWER;
		touch->press_alive = 0;
		mod_timer(&touch->timer,
				jiffies + msecs_to_jiffies(touch->idle_interval));
		#endif

		#if SUPPORT_USER_LEVEL
		if (p_char_dev->OpenCnts) {
			#if USER_LEVEL_DEBUG
			char cBuf[128];
			sprintf(cBuf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ",
					i2c_data[0]&0xFF, i2c_data[1]&0xFF, i2c_data[2]&0xFF, i2c_data[3]&0xFF, i2c_data[4]&0xFF,
					i2c_data[5]&0xFF, i2c_data[6]&0xFF, i2c_data[7]&0xFF, i2c_data[8]&0xFF, i2c_data[9]&0xFF);
			TS_DEBUG("kfifo in = %s\n", cBuf);
			#endif
			/* push data into kfifo */
			//kfifo_put(p_char_dev->pDataFiFo, (u8 *)i2c_data, data_len);
			kfifo_in_spinlocked(&p_char_dev->DataFiFo, (u8 *)i2c_data, data_len, &p_char_dev->FiFoLock);
			wake_up_interruptible( &p_char_dev->fifo_inq );
			continue;
		}
		#endif

		if(i2c_data[0] != REPORTID_MTOUCH)
			continue;

		/* report co-ordinates to the multi-touch stack */
		for (i = 0; i < SUPPORTED_FINGERS; i++) {
			if(touch->Pressure[i] > 0) {
				#if SUPPORT_IDLE_MODE
				touch->press_alive = 1;
				#endif
				input_report_abs(touch->input_dev,
						ABS_MT_TRACKING_ID, i);
				input_report_abs(touch->input_dev,
						ABS_MT_PRESSURE, touch->Pressure[i]);
				input_report_abs(touch->input_dev,
						ABS_MT_POSITION_X, x[i]);
				input_report_abs(touch->input_dev,
						ABS_MT_POSITION_Y, y[i]);
			}
			if(touch->Pressure[i] != TOOL_PRESSURE) {
				input_mt_sync(touch->input_dev);
			}
		}
		input_sync(touch->input_dev);

		if(touch->Pressure[fingers] == 0)
			touch->Pressure[fingers] = TOOL_PRESSURE;
	}

    return 0;
}

#if SUPPORT_IDLE_MODE
static void egalax_i2c_timer(unsigned long _data)
{
	struct egalax_i2c_driver_data *touch = (struct egalax_i2c_driver_data *)_data;

	schedule_work(&touch->work);
}

static void egalax_i2c_work(struct work_struct *work)
{
	struct egalax_i2c_driver_data *touch =
		container_of(work, struct egalax_i2c_driver_data, work);

	if(!touch->press_alive && touch->power_mode == EGALAX_FULLPOWER)
		egalax_i2c_power(touch->client, EGALAX_IDLE);
}
#endif

static int egalax_input_open(struct input_dev *dev)
{
	struct egalax_i2c_driver_data *touch = input_get_drvdata(dev);
	//int gpio = irq_to_gpio(touch->client->irq);

	//enable_irq(touch->client->irq);
	//gpio_direction_input(gpio);
	//wake_up_process( touch->task );
	touch->power_mode = EGALAX_FULLPOWER;
	#if SUPPORT_IDLE_MODE
	mod_timer(&touch->timer,
			jiffies + msecs_to_jiffies(touch->idle_interval));
	#endif

	return 0;
}

static void egalax_input_close(struct input_dev *dev)
{
	//struct egalax_i2c_driver_data *touch = input_get_drvdata(dev);

	//disable_irq(touch->client->irq);
}

static int __devinit egalax_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct egalax_i2c_driver_data *touch = NULL;
    struct input_dev *input_dev = NULL;
	char arr[MAX_I2C_LEN];
    int err;

    touch = kzalloc(sizeof(struct egalax_i2c_driver_data), GFP_KERNEL);
	if(touch == NULL) {
		pr_err("%s: failed to allocate driver data\n", __FUNCTION__);
		return -ENOMEM;
	}
	touch->client = client;

	err = gpio_request(irq_to_gpio(client->irq), client->name);
	if (err < 0) {
		pr_err("%s: failed to request GPIO %d,"
			" error %d\n", __FUNCTION__, irq_to_gpio(client->irq), err);
		goto err_open_failed;
	}

	err = gpio_direction_input(irq_to_gpio(client->irq));
	if (err < 0) {
		pr_err("%s: failed to configure input"
			" direction for GPIO %d, error %d\n",
			__FUNCTION__, irq_to_gpio(client->irq), err);
		goto err_gpio_config;
	}

	arr[0] = 0x03;
    arr[1] = 0x01;
    arr[2] = 'A';
	if (egalax_i2c_write(touch->client, arr, 3) != 3) {
		pr_err("%s: Maybe touch controller is in Sleep or Idle mode, wake it up and try again\n", __FUNCTION__);
		gpio_direction_output(irq_to_gpio(client->irq), 1);
		udelay(50);
		gpio_direction_output(irq_to_gpio(client->irq), 0);
		udelay(50);
		gpio_direction_output(irq_to_gpio(client->irq), 1);
		msleep(100);
		gpio_direction_input(irq_to_gpio(client->irq));

		if (egalax_i2c_write(touch->client, arr, 3) != 3) {
			pr_err("%s: controller probe failed\n", __FUNCTION__);
			err = -ENODEV;
			goto err_gpio_config;
		}
	}
	i2c_set_clientdata(client, touch);

	#if SUPPORT_IDLE_MODE
	touch->idle_interval = 60000;
	setup_timer(&touch->timer, egalax_i2c_timer, (unsigned long)touch);
	INIT_WORK(&touch->work, egalax_i2c_work);
	#endif
	touch->power_mode = EGALAX_SLEEP;

	sema_init(&touch->sem, 0);
    touch->task = kthread_create(egalax_i2c_thread, touch, "egalax_i2c_thread");
    if (touch->task == NULL) {
        err = -1;
        goto err_kthread_create_failed;
    }
	wake_up_process( touch->task );

	input_dev = input_allocate_device();
    if (input_dev == NULL) {
        err = -ENOMEM;
        pr_err("%s: Failed to allocate input device\n", __FUNCTION__);
        goto err_input_allocate_device_failed;
    }
    touch->input_dev = input_dev;
    touch->input_dev->name = DRIVER_NAME;
	touch->input_dev->hint_events_per_packet = 64;
	touch->input_dev->dev.parent = &client->dev;
	touch->input_dev->id.bustype = BUS_I2C;
	touch->input_dev->id.vendor = 0x0EEF;
	touch->input_dev->id.product = 0x0020;
	touch->input_dev->open = egalax_input_open;
	touch->input_dev->close = egalax_input_close;
	input_set_drvdata(touch->input_dev, touch);

    /* Will generate sync at the end of all input */
    set_bit(EV_SYN, touch->input_dev->evbit);
    /* Input values are in absoulte values */
    set_bit(EV_ABS, touch->input_dev->evbit);

    touch->MaxX = 32767;
    touch->MinX = 0;
    touch->MaxY = 32767;
    touch->MinY = 0;

    input_set_abs_params(touch->input_dev, ABS_MT_POSITION_X,
                         touch->MinX, touch->MaxX, 0, 0);
    input_set_abs_params(touch->input_dev, ABS_MT_POSITION_Y,
                         touch->MinY, touch->MaxY, 0, 0);
    input_set_abs_params(touch->input_dev, ABS_MT_PRESSURE,
                         0, 1, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_MT_TRACKING_ID,
                         0, SUPPORTED_FINGERS, 0, 0);

	#if SUPPORT_USER_LEVEL
    egalax_chrdev_init(client);
	#endif

    err = input_register_device(touch->input_dev);
    if (err) {
        pr_err("%s: Unable to register input device\n", __FUNCTION__);
        goto err_input_register_device_failed;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    touch->early_suspend.suspend = egalax_i2c_early_suspend;
    touch->early_suspend.resume = egalax_i2c_late_resume;
    register_early_suspend(&touch->early_suspend);
#endif

	touch->bPollingMode = false;
	egalax_i2c_read(touch->client, arr);
	err = request_irq(client->irq, egalax_i2c_interrupt, IRQF_TRIGGER_FALLING,
			  client->name, touch);
	if (err) {
		pr_err("%s: Unable to request touchscreen IRQ.\n", __FUNCTION__);
		touch->bPollingMode = true;
        touch->pollingIntervalMS = 10;
	}
	//disable_irq(client->irq);

	//#if SUPPORT_IDLE_MODE
	//mod_timer(&touch->timer,
	//		jiffies + msecs_to_jiffies(touch->idle_interval));
	//#endif

    printk(KERN_INFO TOUCH_NAME
           ": Successfully registered the eGalax I2C touch driver\n");
    return 0;

	err_input_register_device_failed:
	input_free_device(touch->input_dev);
	err_input_allocate_device_failed:
	kthread_stop(touch->task);
	#if SUPPORT_IDLE_MODE
	cancel_work_sync(&touch->work);
	del_timer_sync(&touch->timer);
	#endif
	err_kthread_create_failed:
	err_gpio_config:
	gpio_free(irq_to_gpio(client->irq));
    err_open_failed:
    kfree(touch);
    return err;
}

static int __devexit egalax_i2c_remove(struct i2c_client *client) {
    struct egalax_i2c_driver_data *touch = i2c_get_clientdata(client);

	#if SUPPORT_USER_LEVEL
    egalax_chrdev_exit();
	#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&touch->early_suspend);
#endif
    touch->shutdown = 1;
    /* FIXME How to destroy the thread? Maybe we should use workqueues? */
	kthread_stop(touch->task);
    input_unregister_device(touch->input_dev);
    kfree(touch);
    return 0;
}

static const struct i2c_device_id egalax_i2c_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver egalax_i2c_driver = {
    .probe    = egalax_i2c_probe,
    .remove  = egalax_i2c_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = egalax_i2c_suspend,
    .resume  = egalax_i2c_resume,
#endif
	.id_table	= egalax_i2c_id,
    .driver  = {
        .name   = DRIVER_NAME,
    },
};

static int __init egalax_i2c_init(void) {
    int e;

	e = i2c_add_driver(&egalax_i2c_driver);
	if (e != 0) {
		pr_err("%s: failed to register with I2C bus with "
		       "error: 0x%x\n", __func__, e);
	}
	return e;
}

static void __exit egalax_i2c_exit(void) {

    i2c_del_driver(&egalax_i2c_driver);
}

module_init(egalax_i2c_init);
module_exit(egalax_i2c_exit);

MODULE_DESCRIPTION("eGalax I2C touch driver");


