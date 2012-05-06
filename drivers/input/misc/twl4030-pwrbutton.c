/**
 * twl4030-pwrbutton.c - TWL4030 Power Button Input Driver
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/workqueue.h>	// 100920 sookyoung.kim@lge.com For INIT_WORK()
#include <linux/delay.h>	// 100920 sookyoung.kim@lge.com

/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
#include <linux/dvs_suite.h>
/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */

#define PWR_PWRON_IRQ (1 << 0)

#define STS_HW_CONDITIONS 0xf

static struct workqueue_struct *pwrbutton_wq;	// 100920 sookyoung.kim@lge.com
static struct work_struct pwrbutton_wk;		// 100920 sookyoung.kim@lge.com
static struct input_dev *pwr;			// 100920 sookyoung.kim@lge.com

// 20100920 sookyoung.kim@lge.com Refine IRQ handlder with a work queue [START_LGE]
static void pwrbutton_wq_func(struct work_struct *work){

	int err;
	u8 value;

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value, STS_HW_CONDITIONS);
	if (!err)  {
#if 0  // this code is necessary for hdmi sleep sequence.
		extern int is_hdmi_enabled(void);
		if(is_hdmi_enabled())
		{
			if(!(value & PWR_PWRON_IRQ))
			{
				input_report_key(pwr, KEY_BACK, 1);
				input_sync(pwr);
				input_report_key(pwr, KEY_BACK, 0);
				input_sync(pwr);
				
				extern int WHAT_MODE_IS_IT(void);
				switch(WHAT_MODE_IS_IT()) // this code is necessary for hdmi sleep sequence from HDMI to LCD in Video mode.
				{					           // it takes at least specific time(ms) to change overlay completely.
					case 0:
						// Image gallery
						msleep(1000);
						break;
					case 1:
						// Video mode
						msleep(4000); 
						break;
					case 2:
						// Normal mode
						break;
				}
				input_report_key(pwr, KEY_POWER, 1);	
				input_sync(pwr);
				input_report_key(pwr, KEY_POWER, 0);
				input_sync(pwr);
				msleep(700);
			}
		}
		else
		{
			input_report_key(pwr, KEY_POWER, value & PWR_PWRON_IRQ);
			input_sync(pwr);
			printk( KERN_WARNING "[PWRBUTTON] pwrbutton irq has been processed! %d\n",
				(value & PWR_PWRON_IRQ));
		}

#else
		input_report_key(pwr, KEY_POWER, value & PWR_PWRON_IRQ);
		input_sync(pwr);
		printk( KERN_WARNING "[PWRBUTTON] pwrbutton irq has been processed! %d\n",
			(value & PWR_PWRON_IRQ));
#endif

		
	} else {
		dev_err(pwr->dev.parent, "twl4030: i2c error %d while reading"
			" TWL4030 PM_MASTER STS_HW_CONDITIONS register\n", err);
		printk(KERN_WARNING "[PWRBUTTON] pwrbutton irq caused an error!\n");
	}
}

static irqreturn_t powerbutton_irq(int irq, void *_pwr)
{
	pwr = _pwr;

	queue_work(pwrbutton_wq, &pwrbutton_wk);

/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
/* Move this code later to somewhere common, such as the irq entry point.
 */
#if 1
	if(ds_status.flag_run_dvs == 1){
        ds_status.flag_touch_timeout_count = DS_TOUCH_TIMEOUT_COUNT_MAX;    // = 6
        if(ds_status.touch_timeout_sec == 0){
            if(ds_counter.elapsed_usec + DS_TOUCH_TIMEOUT < 1000000){
                ds_status.touch_timeout_sec = ds_counter.elapsed_sec;
                ds_status.touch_timeout_usec = ds_counter.elapsed_usec + DS_TOUCH_TIMEOUT;
            }
            else{
                ds_status.touch_timeout_sec = ds_counter.elapsed_sec + 1;
                ds_status.touch_timeout_usec = (ds_counter.elapsed_usec + DS_TOUCH_TIMEOUT) - 1000000;
            }
        }
    }
#endif
/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */

	return IRQ_HANDLED;
}
// 20100920 sookyoung.kim@lge.com Refine IRQ handlder with a work queue [END_LGE]


// 20101012 jh.koo@lge.com for Test mode[START_LGE]
static ssize_t hub_pwrbutton_test_mode_store(struct device *dev,  struct device_attribute *attr,  char *buf)
{
	int ret, pwr_test;	
	
    ret = sscanf(buf, "%d", &pwr_test);

	if(pwr_test == 1) {
		
//		queue_work(pwrbutton_wq, &pwrbutton_wk);

		input_report_key(pwr, KEY_POWER, 1);	
		input_report_key(pwr, KEY_POWER, 0);
		input_sync(pwr);		
	}

	return ret;

}
static DEVICE_ATTR(pwrbutton_test_mode, 0664, NULL, hub_pwrbutton_test_mode_store);
// 20101012 jh.koo@lge.com for Test mode [END_LGE]

static int __devinit twl4030_pwrbutton_probe(struct platform_device *pdev)
{
	//struct input_dev *pwr;	// 100920 sookyoung.kim@lge.com
	int irq = platform_get_irq(pdev, 0);
	int err;

	pwrbutton_wq = create_workqueue("pwrbutton_workqueue");	// 100920 sookyoung.kim@lge.com

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
#if 1 // this code is necessary for hdmi sleep sequence.
    	set_bit(KEY_BACK, pwr->keybit);
#endif
	pwr->name = "twl4030_pwrbutton";
	pwr->phys = "twl4030_pwrbutton/input0";
	pwr->dev.parent = &pdev->dev;

	// 100920 sookyoung.kim@lge.com Register power button work queue function
	INIT_WORK(&pwrbutton_wk, pwrbutton_wq_func);

	err = request_irq(irq, powerbutton_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl4030_pwrbutton", pwr);
	if (err < 0) {
		dev_dbg(&pdev->dev, "Can't get IRQ for pwrbutton: %d\n", err);
		goto free_input_dev;
	}

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}
// 20101012 jh.koo@lge.com for TEST MODE [START_LGE]
	err = device_create_file(&pdev->dev, &dev_attr_pwrbutton_test_mode);
	if (err) {
		printk( "Hub-power button: Power Button_probe: Fail\n");
		device_remove_file(&pdev->dev, &dev_attr_pwrbutton_test_mode);
		return err;
	}
// 20101012 jh.koo@lge.com for TEST MODE [END_LGE]	

	platform_set_drvdata(pdev, pwr);

	return 0;

free_irq:
	free_irq(irq, NULL);
free_input_dev:
	input_free_device(pwr);
	return err;
}

static int __devexit twl4030_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *pwr = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, pwr);
	input_unregister_device(pwr);

	flush_workqueue(pwrbutton_wq);		// 100920 sookyoung.kim@lge.com
	destroy_workqueue(pwrbutton_wq);	// 100920 sookyoung.kim@lge.com

	return 0;
}

struct platform_driver twl4030_pwrbutton_driver = {
	.probe		= twl4030_pwrbutton_probe,
	.remove		= __devexit_p(twl4030_pwrbutton_remove),
	.driver		= {
		.name	= "twl4030_pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_pwrbutton_init(void)
{
	return platform_driver_register(&twl4030_pwrbutton_driver);
}
module_init(twl4030_pwrbutton_init);

static void __exit twl4030_pwrbutton_exit(void)
{
	platform_driver_unregister(&twl4030_pwrbutton_driver);
}
module_exit(twl4030_pwrbutton_exit);

MODULE_ALIAS("platform:twl4030_pwrbutton");
MODULE_DESCRIPTION("Triton2 Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver <peter.de-schrijver@nokia.com>");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");
