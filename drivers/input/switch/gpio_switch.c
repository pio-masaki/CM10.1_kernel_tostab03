/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/switch.h>

struct gpio_switch_data {
	struct gpio_switch *switches;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	#ifdef CONFIG_SWITCH
	struct switch_dev sdev;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	#endif
};

struct gpio_sw_drvdata {
	struct input_dev *input;
	struct gpio_switch_data data[0];
};

static void gpio_sw_report_event(struct work_struct *work)
{
	struct gpio_switch_data *bdata =
		container_of(work, struct gpio_switch_data, work);
	struct gpio_switch *switches = bdata->switches;
	struct input_dev *input = bdata->input;
	unsigned int type = switches->type ?: EV_SW;
	int state = (gpio_get_value(switches->gpio) ? 1 : 0) ^ switches->active_low;
	
	input_event(input, type, switches->code, !!state);
	input_sync(input);
	#ifdef CONFIG_SWITCH
	switch_set_state(&bdata->sdev, state);
	#endif
}

static void gpio_sw_timer(unsigned long _data)
{
	struct gpio_switch_data *data = (struct gpio_switch_data *)_data;

	schedule_work(&data->work);
}

static irqreturn_t gpio_sw_isr(int irq, void *dev_id)
{
	struct gpio_switch_data *bdata = dev_id;
	struct gpio_switch *switches = bdata->switches;

	BUG_ON(irq != gpio_to_irq(switches->gpio));
	
	if (switches->debounce_interval)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(switches->debounce_interval));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static int __devinit gpio_sw_probe(struct platform_device *pdev)
{
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_sw_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	
	ddata = kzalloc(sizeof(struct gpio_sw_drvdata) +
			pdata->nswitches * sizeof(struct gpio_switch_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-switches";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	
	ddata->input = input;

	for (i = 0; i < pdata->nswitches; i++) {
		struct gpio_switch *switches = &pdata->switches[i];
		struct gpio_switch_data *bdata = &ddata->data[i];
		int irq;
		unsigned int type = switches->type ?: EV_SW;

		bdata->input = input;
		bdata->switches = switches;
		setup_timer(&bdata->timer,
			    gpio_sw_timer, (unsigned long)bdata);
		INIT_WORK(&bdata->work, gpio_sw_report_event);

		#ifdef CONFIG_SWITCH
		bdata->sdev.name = switches->desc;
	    error = switch_dev_register(&bdata->sdev);
		if (error < 0)
			goto fail1;
		#endif

		error = gpio_request(switches->gpio, switches->desc ?: "gpio-switches");
		if (error < 0) {
			pr_err("gpio-switches: failed to request GPIO %d,"
				" error %d\n", switches->gpio, error);
			goto fail2;
		}

		error = gpio_direction_input(switches->gpio);
		if (error < 0) {
			pr_err("gpio-switches: failed to configure input"
				" direction for GPIO %d, error %d\n",
				switches->gpio, error);
			gpio_free(switches->gpio);
			goto fail2;
		}

		irq = gpio_to_irq(switches->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("gpio-switches: Unable to get irq number"
				" for GPIO %d, error %d\n",
				switches->gpio, error);
			gpio_free(switches->gpio);
			goto fail2;
		}

		error = request_irq(irq, gpio_sw_isr,
				    IRQF_SHARED |
				    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    switches->desc ? switches->desc : "gpio-switches",
				    bdata);
		if (error) {
			pr_err("gpio-switches: Unable to claim irq %d; error %d\n",
				irq, error);
			gpio_free(switches->gpio);
			goto fail2;
		}

		input_set_capability(input, type, switches->code);

		/* Perform initial detection */
		gpio_sw_report_event(&bdata->work);
	}

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-switches: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->switches[i].gpio), &ddata->data[i]);
		if (pdata->switches[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->switches[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_sw_remove(struct platform_device *pdev)
{
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_sw_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nswitches; i++) {
		int irq = gpio_to_irq(pdata->switches[i].gpio);
		#ifdef CONFIG_SWITCH
		struct gpio_switch_data *bdata = &ddata->data[i];
		switch_dev_unregister(&bdata->sdev);
		#endif

		free_irq(irq, &ddata->data[i]);
		if (pdata->switches[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->switches[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_sw_suspend(struct device *dev)
{
	return 0;
}

static int gpio_sw_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_sw_drvdata *ddata = platform_get_drvdata(pdev);
	int i;

	/* Notify switch status after resume */
	for (i = 0; i < pdata->nswitches; i++) {
		schedule_work(&ddata->data[i].work);
	}

	return 0;
}

static const struct dev_pm_ops gpio_sw_pm_ops = {
	.suspend	= gpio_sw_suspend,
	.resume		= gpio_sw_resume,
};
#endif

static struct platform_driver gpio_sw_device_driver = {
	.probe		= gpio_sw_probe,
	.remove		= __devexit_p(gpio_sw_remove),
	.driver		= {
		.name	= "gpio-switches",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_sw_pm_ops,
#endif
	}
};

static int __init gpio_sw_init(void)
{
	return platform_driver_register(&gpio_sw_device_driver);
}

static void __exit gpio_sw_exit(void)
{
	platform_driver_unregister(&gpio_sw_device_driver);
}

module_init(gpio_sw_init);
module_exit(gpio_sw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Switch driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-switches");
