#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>

#ifdef CONFIG_CCIMX5X_PM_POWER_BUTTON
#include <linux/suspend.h>
#endif

#define DRIVER_NAME "da9052-onkey"

struct da9052_onkey_data {
	struct da9052 *da9052;
	struct da9052_eh_nb eh_data;
	struct input_dev *input;
	struct delayed_work polling_work;
};

#ifdef CONFIG_CCIMX5X_PM_POWER_BUTTON
enum power_on_state {
	PWRON_FROM_RESUME,
	PWRON_FALLING_EDGE,
	PWRON_FROM_SUSPEND
};

static int pwron_state = PWRON_FROM_RESUME;

static void da9052_power_button( struct da9052 *da9052 , int value )
{
	struct da9052_ssc_msg ssc_msg;
	unsigned int ret;

	if( system_state != SYSTEM_RUNNING)
		return;

	switch (pwron_state){
			case PWRON_FROM_RESUME:
				// Are we waking up from a non power button suspend
				// and this is the falling edge already?
				if( !value )
					pwron_state = PWRON_FROM_RESUME;
				else
					// Skipping falling edge
					pwron_state = PWRON_FALLING_EDGE;
				break;
			case PWRON_FALLING_EDGE:
				// Suspend on rising edge
				pm_suspend(PM_SUSPEND_MEM);
				// Did we wake by a power button press?
				da9052_lock(da9052);
				ssc_msg.addr = DA9052_EVENTB_REG;
				ret = da9052->read(da9052, &ssc_msg);
				if (ret) {
					da9052_unlock(da9052);
					pwron_state = PWRON_FROM_RESUME;
					return;
				}
				da9052_unlock(da9052);
				if( ssc_msg.data | DA9052_EVENTB_ENONKEY )
					pwron_state = PWRON_FROM_SUSPEND;
				else
					// Waken by other source
					pwron_state = PWRON_FROM_RESUME;
				break;
			case PWRON_FROM_SUSPEND:
				// Ignoring raising edge
				pwron_state = PWRON_FROM_RESUME;
				break;
			default:
				pr_err("power_on_evt_handler: Unitialized state\n");
	}
}
#endif

static void da9052_onkey_work_func(struct work_struct *work)
{
	struct da9052_onkey_data *da9052_onkey =
		container_of(work, struct da9052_onkey_data, polling_work.work);
	struct da9052_ssc_msg msg;
	unsigned int ret;
	int value;

	da9052_lock(da9052_onkey->da9052);
	msg.addr = DA9052_STATUSA_REG;
	ret = da9052_onkey->da9052->read(da9052_onkey->da9052, &msg);
	if (ret) {
		da9052_unlock(da9052_onkey->da9052);
		return;
	}
	da9052_unlock(da9052_onkey->da9052);
	value = (msg.data & DA9052_STATUSA_NONKEY) ? 0 : 1;

	input_report_key(da9052_onkey->input, KEY_POWER, value);
	input_sync(da9052_onkey->input);

#ifdef CONFIG_CCIMX5X_PM_POWER_BUTTON
	da9052_power_button( da9052_onkey->da9052 , value );
#endif

	/* if key down, polling for up */
	if (value)
		schedule_delayed_work(&da9052_onkey->polling_work, HZ/10);
}

static void da9052_onkey_report_event(struct da9052_eh_nb *eh_data,
				unsigned int event)
{
	struct da9052_onkey_data *da9052_onkey =
		container_of(eh_data, struct da9052_onkey_data, eh_data);
	cancel_delayed_work(&da9052_onkey->polling_work);
	schedule_delayed_work(&da9052_onkey->polling_work, 0);

}

static int __devinit da9052_onkey_probe(struct platform_device *pdev)
{
	struct da9052_onkey_data *da9052_onkey;
	int error,ret;
	struct da9052_ssc_msg msg;

	da9052_onkey = kzalloc(sizeof(*da9052_onkey), GFP_KERNEL);
	da9052_onkey->input = input_allocate_device();
	if (!da9052_onkey->input) {
		dev_err(&pdev->dev, "failed to allocate data device\n");
		error = -ENOMEM;
		goto fail1;
	}
	da9052_onkey->da9052 = dev_get_drvdata(pdev->dev.parent);

	if (!da9052_onkey->input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto fail2;
	}

	da9052_onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	da9052_onkey->input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	da9052_onkey->input->name = "da9052-onkey";
	da9052_onkey->input->phys = "da9052-onkey/input0";
	da9052_onkey->input->dev.parent = &pdev->dev;
	INIT_DELAYED_WORK(&da9052_onkey->polling_work, da9052_onkey_work_func);

	/* Set the EH structure */
	da9052_onkey->eh_data.eve_type = ONKEY_EVE;
	da9052_onkey->eh_data.call_back = &da9052_onkey_report_event;
	error = da9052_onkey->da9052->register_event_notifier(
				da9052_onkey->da9052,
				&da9052_onkey->eh_data);
	if (error)
		goto fail2;

	error = input_register_device(da9052_onkey->input);
	if (error) {
		dev_err(&pdev->dev, "Unable to register input\
				device,error: %d\n", error);
		goto fail3;
	}

	platform_set_drvdata(pdev, da9052_onkey);

	// Unmask IRQ
	da9052_lock(da9052_onkey->da9052);
	msg.addr = DA9052_IRQMASKB_REG;
	ret = da9052_onkey->da9052->read(da9052_onkey->da9052, &msg);
	if (ret) {
		da9052_unlock(da9052_onkey->da9052);
		return 0;
	}
	msg.data &= ~DA9052_IRQMASKB_MNONKEY;
	da9052_onkey->da9052->write(da9052_onkey->da9052, &msg);
	da9052_unlock(da9052_onkey->da9052);
	return 0;

fail3:
	da9052_onkey->da9052->unregister_event_notifier(da9052_onkey->da9052,
					&da9052_onkey->eh_data);
fail2:
	input_free_device(da9052_onkey->input);
fail1:
	kfree(da9052_onkey);
	return error;
}

static int __devexit da9052_onkey_remove(struct platform_device *pdev)
{
	struct da9052_onkey_data *da9052_onkey = pdev->dev.platform_data;
	da9052_onkey->da9052->unregister_event_notifier(da9052_onkey->da9052,
					&da9052_onkey->eh_data);
	input_unregister_device(da9052_onkey->input);
	kfree(da9052_onkey);

	return 0;
}

static struct platform_driver da9052_onkey_driver = {
	.probe		= da9052_onkey_probe,
	.remove		= __devexit_p(da9052_onkey_remove),
	.driver		= {
		.name	= "da9052-onkey",
		.owner	= THIS_MODULE,
	}
};

static int __init da9052_onkey_init(void)
{
	return platform_driver_register(&da9052_onkey_driver);
}

static void __exit da9052_onkey_exit(void)
{
	platform_driver_unregister(&da9052_onkey_driver);
}

module_init(da9052_onkey_init);
module_exit(da9052_onkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>");
MODULE_DESCRIPTION("Onkey driver for DA9052");
MODULE_ALIAS("platform:" DRIVER_NAME);
