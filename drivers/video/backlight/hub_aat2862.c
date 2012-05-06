/* drivers/video/backlight/e920_aat2862.c 
 *
 * Copyright (C) 2010 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mux.h>	// 20100527 sookyoung.kim@lge.com
#include <linux/leds.h>
#include <plat/board.h>
#include <linux/earlysuspend.h> //20100709 kyungtae.oh@lge.com early_suspend
// 20100720 jh.koo@lge.com Hub touchscreen power sequence [START_LGE]
#include "../mux.h"
// 20100720 jh.koo@lge.com Hub touchscreen power sequence [END_LGE]
//20101126 taehwan.kim@lge.com check battery present [START_LGE]
int check_battery_present(void);
//20101126 taehwan.kim@lge.com check battery present [END_LGE]
static struct i2c_client *aat2862_i2c_client;

#define I2C_NO_REG	0xFF

#define MAX_BRIGHTNESS 		0x15	// 0000 1010 = 20.32mA
#define DEFAULT_BRIGHTNESS 	0x09	// 0000 1001 = mA

#define MODULE_NAME    "hub_aat2862"

#define BL_ON	1
#define BL_OFF	0

#define MAX_LDO_REG		8
static u8 mirror_reg[MAX_LDO_REG] = {0,};

struct aat2862_device {
	struct i2c_client *client;
	struct backlight_device *bl_dev;
    struct early_suspend early_suspend; //20100709 kyungtae.oh@lge.com early_suspend
};

struct bl_ctrl_data {
	unsigned short reg;
	unsigned short val;
};

//20100709 kyungtae.oh@lge.com early_suspend [START_LGE]
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aat2862_early_suspend(struct early_suspend *h);
static void aat2862_late_resume(struct early_suspend *h);
static int early_bl_timer = 1;
static int early_bl_value = 0;
#endif
//20100709 kyungtae.oh@lge.com early_suspend [END_LGE]

#define MY_LCD_CP_EN 149
#define HUB_I2C_BL_NAME "hub_i2c_bl"

// 20100720 jh.koo@lge.com Hub touchscreen power sequence [START_LGE]
#define MY_TOUCH_I2C2_SW	154
static int touch_count = 0;
// 20100720 jh.koo@lge.com Hub touchscreen power sequence [END_LGE]

// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [START_LGE]
#if 0
#define MY_HDMI_REG_EN		104
#define MY_CAM_VCM_EN		167
#define MY_DMB_EN		28
#define MY_CAM_SUBPM_EN		37
#define	MY_MOTION_INT		42
#define	MY_COM_INT		58
#endif
#if 0
static u16 i2c3_scl = 0; 	// sookyoung.kim
static u16 i2c3_sda = 0; 	// sookyoung.kim
#endif
#if 0
static u16 i2c4_scl = 0; 	// sookyoung.kim
static u16 i2c4_sda = 0; 	// sookyoung.kim
#endif
// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [END_LGE]

static const struct i2c_device_id hub_bl_id[] = {
	{ HUB_I2C_BL_NAME, 0 },
	//{ }
	{ },	// 20100526 sookyoung.kim@lge.com
};

static int aat2862_write_reg(struct i2c_client *client, unsigned char reg, unsigned char val);
static int aat2862_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *ret);

int cur_main_lcd_level = DEFAULT_BRIGHTNESS;
static int saved_main_lcd_level = DEFAULT_BRIGHTNESS;

static int backlight_status = BL_OFF;
static struct aat2862_device *main_aat2862_dev = NULL;
#if 0
static struct bl_ctrl_data pwron_seq[]=
{
	{0x00,0x4C},	/* LDOA 1.8V_LCD_IOVCC(0100), LDOB 3.0V_LCD_VCC_VCI (1100) */
	{0x01,0x4C},	/* LDOC 1.8V_TOUCH_VDD (0100), LDOD 3.0V_TOUCH_VCPIN (1100) */
	
	//{0x02, 0x01}, 	//Enable all LDOs
	// 20100525 sookyoung.kim@lge.com [START_LGE]
	{0x02, 0x0F},   //Enable all LDOs 
	// 20100525 sookyoung.kim@lge.com [END_LGE]

	//{0x03, 0xFF-DEFAULT_BRIGHTNESS},	// Main BL ctrl.
	//{0x04, 0x7F-DEFAULT_BRIGHTNESS},	// Sub BL ctrl.
	//{0x05, 0x7F-DEFAULT_BRIGHTNESS},	// Aux1 BL ctrl.
	//{0x06, 0x7F-DEFAULT_BRIGHTNESS},	// Aux2 BL ctrl.
	{I2C_NO_REG, 0x00}  /* End of array */
};
#endif

void aat2862_hreset(void)
{
	gpio_set_value(MY_LCD_CP_EN, 1);
	udelay(10);
	gpio_set_value(MY_LCD_CP_EN, 0);
	udelay(1000);
	gpio_set_value(MY_LCD_CP_EN, 1);
	udelay(10);
}

static int aat2862_write_reg(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg = {	
		client->addr, 0, 2, buf 
	};

	buf[0] = reg;
	buf[1] = val;
	
	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "i2c write error\n");
	}
	
	return 0;
}

static int aat2862_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *ret)
{
	int err;
	unsigned char buf = reg;

	struct i2c_msg msg[2] = { 
		{ client->addr, 0, 1, &buf },
		{ client->addr, I2C_M_RD, 1, ret}
	};

	if ((err = i2c_transfer(client->adapter, msg, 2)) < 0) {
		dev_err(&client->dev, "i2c read error\n");
	}
	
	return 0;
}

void aat2862_init(struct i2c_client *client) 
{
	//unsigned i;

	gpio_request(MY_LCD_CP_EN, "lcd bl");
	gpio_direction_output(MY_LCD_CP_EN, 1);

	aat2862_hreset();

#if 0	//20100709 kyungtae.oh@lge.com for resume
	for (i = 0; pwron_seq[i].reg != I2C_NO_REG; i++) {
		aat2862_write_reg(client, pwron_seq[i].reg, pwron_seq[i].val);
		mdelay(10);
	}
	backlight_status = BL_ON;
#endif
}

void aat2862_ldo_enable(int enable) 
{
	return;
}
EXPORT_SYMBOL(aat2862_ldo_enable);

static void aat2862_ldo_read(u8 reg , u8 *val)
{
	if (reg < 0 || reg > MAX_LDO_REG - 1)
	{
		printk("[LDO] Hub ldo invalid register access\n");
		return;
	}

	*val = i2c_smbus_read_byte_data(aat2862_i2c_client, reg);
	printk("[LDO] ldo Reg read 0x%X: Val=0x%X\n", (u8)reg, (u8)*val);
}

static int aat2862_ldo_write(u8 reg , u8 val)
{
	int ret =  -1  ;

	if (reg < 0 || reg > MAX_LDO_REG - 1)
	{
		printk("[LDO] Hub ldo invalid register access\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(aat2862_i2c_client, reg , val);
	if(ret >= 0)
	{
		mirror_reg[reg] = val;
		printk("[LDO] Hub ldo Write Reg. = 0x%X, Value 0x%X\n", reg, val);
	}

	return ret;
}

static inline int aat2862_clear_n_set(u8 clear, u8 set, u8 reg)
{
	int ret;
	u8 val = 0;

	/* Gets the initial register value */
	aat2862_ldo_read(reg, &val);

	/* Clearing all those bits to clear */
	val &= ~(clear);

	/* Setting all those bits to set */
	val |= set;

	/* Update the register */
	ret = aat2862_ldo_write(reg, val);
	if (ret)
		return ret;

	return 0;
}

/*
Bit3 Bit2 Bit1 Bit0 VLDO(V)
0 0 0 0 1.2
0 0 0 1 1.3
0 0 1 0 1.5
0 0 1 1 1.6
0 1 0 0 1.8
0 1 0 1 2.0
0 1 1 0 2.2
0 1 1 1 2.5
1 0 0 0 2.6
1 0 0 1 2.7
1 0 1 0 2.8
1 0 1 1 2.9
1 1 0 0 3.0
1 1 0 1 3.1
1 1 1 0 3.2
1 1 1 1 3.3
*/
typedef enum
{
	VLDO_1_2V,	/* 0 */
	VLDO_1_3V,
	VLDO_1_5V,
	VLDO_1_6V,
	VLDO_1_8V,
	VLDO_2_0V,	/* 5 */
	VLDO_2_2V,
	VLDO_2_5V,
	VLDO_2_6V,
	VLDO_2_7V,
	VLDO_2_8V,	/* 10 */
	VLDO_2_9V,
	VLDO_3_0V,
	VLDO_3_1V,
	VLDO_3_2V,
	VLDO_3_3V,	/* 15 */

	VLDO_MAX,	/* 16 */
} AAT2862_VLDO;

typedef enum
{
	ENLDO_A,	/* 0 */
	ENLDO_B,
	ENLDO_C,
	ENLDO_D,

	ENLDO_MAX,	/* 5 */
} AAT2862_ENLDO;

void aat2862_set_ldo(bool enable, AAT2862_ENLDO ldo_num, AAT2862_VLDO level)
{
//	if(!is_enabled)
		return;

	if(enable)
	{
		switch(ldo_num)
		{
			case ENLDO_A:
				aat2862_ldo_write(0x00, level<<4);
				break;
			case ENLDO_B:
				aat2862_ldo_write(0x00, level);
				break;
			case ENLDO_C:
				aat2862_ldo_write(0x01, level<<4);
				break;
			case ENLDO_D:
				aat2862_ldo_write(0x01, level);
				break;
			default:
				printk("[LDO] Hub ldo invalid register access\n");
				return;
		}

		aat2862_clear_n_set(0, 1<<ldo_num, 0x02);
	}
	else
	{
		aat2862_clear_n_set(1<<ldo_num, 0, 0x02);
	}
}
EXPORT_SYMBOL(aat2862_set_ldo);

// 20100630 jh.koo@lge.com Hub touchscreen power sequence [START_LGE]
static void touch_i2c2_enable(int on)
{
	if(on)
	{
		gpio_direction_output(MY_TOUCH_I2C2_SW, 1);
		gpio_set_value(MY_TOUCH_I2C2_SW, 1);
/*
		if(touch_count == 0) {
			msleep(400);
			touch_count = 1;
		}
		else if(touch_count == 1)
			msleep(250);	
*/
	}
	else
	{
		gpio_direction_output(MY_TOUCH_I2C2_SW, 0);
		gpio_set_value(MY_TOUCH_I2C2_SW, 0);
	}		

}

static void aat2862_touch_ldo_enable(struct i2c_client *client, int on)
{
#if 0
	aat2862_set_ldo(1, ENLDO_D, VLDO_3_0V);
	msleep(1);
	aat2862_set_ldo(1, ENLDO_C, VLDO_1_8V);	
#endif
	if(on)
	{
		aat2862_write_reg(client, 0x02, 0x0B);
		msleep(10);
		aat2862_write_reg(client, 0x02, 0x0F);
		msleep(10);		

		touch_i2c2_enable(on);
	}
	else
	{
		touch_i2c2_enable(on);

		msleep(10);
		aat2862_write_reg(client, 0x02, 0x0B);
		msleep(10);
		aat2862_write_reg(client, 0x02, 0x03);
	}

//	printk(KERN_WARNING"[!] %s()\n", __func__);
}
// 20100630 jh.koo@lge.com Hub touchscreen power sequence [END_LGE]
int get_muic_mode();	//20101124 kyungtae.oh@lge.com for CP Mode

static void aat2862_set_main_current_level(struct i2c_client *client, int level)
{
	struct aat2862_device *dev;
	unsigned char val;
	unsigned char aat2862_muic_mode;
	
	dev = (struct aat2862_device *)i2c_get_clientdata(client);
	cur_main_lcd_level = level; 
	dev->bl_dev->props.brightness = cur_main_lcd_level;
/* 20101124 kyungtae.oh@lge.com for CP_USB and CP_UART[LGE_START]*/	
	aat2862_muic_mode = get_muic_mode();

	if ((aat2862_muic_mode == 9) && (check_battery_present() == 0)) //20101126 taehwan.kim@lge.com to check battery present
		level = 0x00;
/* 20101124 kyungtae.oh@lge.com for CP_USB and CP_UART[LGE_END]*/
	val = 0xFF - level;
	//val = 0xC0 | (backlight_status<<5) | level;

	aat2862_write_reg(client, 0x03, val);

	if(level > 0x3F){
		val = 0;
	}else{
		val = 0x3F - level;
	}

	aat2862_write_reg(client, 0x04, val);
	aat2862_write_reg(client, 0x05, val);
	aat2862_write_reg(client, 0x06, val);

	mdelay(1);	// 20100526 sookyoung.kim@lge.com
}

static void leds_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	value = value/12;
	if (value > MAX_BRIGHTNESS)
		value = MAX_BRIGHTNESS;

	//20100709 kyungtae.oh@lge.com for resume [START_LGE]
	if(early_bl_timer == 0)
	{
		early_bl_value = value;
		return;
	}
	//20100709 kyungtae.oh@lge.com for resume [END_LGE]
	
	aat2862_set_main_current_level(aat2862_i2c_client, value);
	cur_main_lcd_level = value; 
	
	return;
}

static struct led_classdev lcd_backlight = {
	.name = "lcd-backlight",
	.brightness = MAX_BRIGHTNESS,
	.brightness_set = leds_brightness_set,
};

void aat2862_backlight_on(void)
{
	//printk("%s received (prev backlight_status: %s)\n", __func__, backlight_status?"ON":"OFF");
	if (backlight_status == BL_ON) return;
	//20100709 kyungtae.oh@lge.com for resume
	//aat2862_set_main_current_level(main_aat2862_dev->client, DEFAULT_BRIGHTNESS);
	aat2862_set_main_current_level(main_aat2862_dev->client, early_bl_value);
	backlight_status = BL_ON;

	return;
}
EXPORT_SYMBOL(aat2862_backlight_on);

void aat2862_backlight_off(void)
{
	if (backlight_status == BL_OFF) return;
	saved_main_lcd_level = cur_main_lcd_level;
	aat2862_set_main_current_level(main_aat2862_dev->client, 0);
	backlight_status = BL_OFF;	

	return;
}
EXPORT_SYMBOL(aat2862_backlight_off);

static int hub_bl_set_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	aat2862_set_main_current_level(client, bd->props.brightness);
	cur_main_lcd_level = bd->props.brightness; 
	
	return 0;
}

static int hub_bl_get_intensity(struct backlight_device *bd)
{
    struct i2c_client *client = to_i2c_client(bd->dev.parent);
    unsigned char val=0;

    aat2862_read_reg(client, 0x03, &val);
	val &= 0x1f;
    return (int)val;
}

ssize_t lcd_backlight_show_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n", cur_main_lcd_level);
	
	return r;
}

ssize_t lcd_backlight_store_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int level;
	struct i2c_client *client = to_i2c_client(dev); 

	if (!count)
		return -EINVAL;
	
	level = simple_strtoul(buf, NULL, 10);
	
	if (level > MAX_BRIGHTNESS)
		level = MAX_BRIGHTNESS;

	aat2862_set_main_current_level(client, level);
	cur_main_lcd_level = level; 
	
	return count;
}

//20100825 kyungtae.oh@lge.com for test_mode[START_LGE]
static int aat2862_bl_resume(struct i2c_client *client);
static int aat2862_bl_suspend(struct i2c_client *client, pm_message_t state);


ssize_t lcd_backlight_show_on_off(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r;
	r = snprintf(buf, PAGE_SIZE, "%d\n", backlight_status);
	//printk("%s received (prev backlight_status: %s)\n", __func__, backlight_status?"ON":"OFF");

	return r;
}

ssize_t lcd_backlight_store_on_off(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int on_off;
	int ret;
	struct i2c_client *client = to_i2c_client(dev); 

	if (!count)
		return -EINVAL;
	
	//printk("%s received (prev backlight_status: %s)\n", __func__, backlight_status?"ON":"OFF");
	
	on_off = simple_strtoul(buf, NULL, 10);
	sscanf(buf, "%d", &ret);

	
	printk(KERN_ERR "%d",on_off);
	
	if(ret){
		early_bl_value=DEFAULT_BRIGHTNESS;
		aat2862_bl_resume(client);
	}else
	    aat2862_bl_suspend(client, PMSG_SUSPEND);
	
	return count;

}
//20100825 kyungtae.oh@lge.com for test_mode[END_LGE]

DEVICE_ATTR(level, 0664, lcd_backlight_show_level, lcd_backlight_store_level);
DEVICE_ATTR(backlight_on_off, 0664, lcd_backlight_show_on_off, lcd_backlight_store_on_off);//20100825 kyungtae.oh@lge.com for test_mode

static struct backlight_ops hub_bl_ops = {
	.update_status = hub_bl_set_intensity,
	.get_brightness = hub_bl_get_intensity,
};

static int __init aat2862_probe(struct i2c_client *i2c_dev, const struct i2c_device_id *id)
{
	struct aat2862_device *dev;
	struct backlight_device *bl_dev;
	struct backlight_properties props;   // hjpark 20110427	
	int err;

	printk(KERN_INFO"%s: i2c probe start\n", __func__);

// 20100810 jh.koo@lge.com GPIO Initialization [START_LGE]
	omap_mux_init_gpio(MY_LCD_CP_EN, OMAP_PIN_OUTPUT);
	err = gpio_request(MY_LCD_CP_EN, "lcd_cp_en");
	if(err < 0) {	
		printk("can't get hub lcd cp enable GPIO\n");
		kzfree(dev);
		return -ENOSYS;
	}
	err = gpio_direction_output(MY_LCD_CP_EN, 1);	
	gpio_set_value(MY_LCD_CP_EN, 1);
	

	omap_mux_init_gpio(MY_TOUCH_I2C2_SW, OMAP_PIN_OUTPUT);
	err = gpio_request(MY_TOUCH_I2C2_SW, "touch_i2c2_sw");
	if(err < 0) {	
		printk("can't get hub touch i2c2 sw enable GPIO\n");
		kzfree(dev);
		return -ENOSYS;
	}
	err = gpio_direction_output(MY_TOUCH_I2C2_SW, 0);	
	gpio_set_value(MY_TOUCH_I2C2_SW, 0);

	aat2862_touch_ldo_enable(i2c_dev, 1);
// 20100810 jh.koo@lge.com GPIO Initialization [END_LGE]	

	aat2862_i2c_client = i2c_dev;

	dev = kzalloc(sizeof(struct aat2862_device), GFP_KERNEL);
	if(dev == NULL) {
		dev_err(&i2c_dev->dev,"fail alloc for aat2862_device\n");
		return 0;
	}

	main_aat2862_dev = dev;

#if 0 // froyo k32 origin
	bl_dev = backlight_device_register(HUB_I2C_BL_NAME, &i2c_dev->dev, NULL, &hub_bl_ops);
#else
	bl_dev = backlight_device_register(HUB_I2C_BL_NAME, &i2c_dev->dev, NULL, &hub_bl_ops, &props);
#endif 
	bl_dev->props.max_brightness = MAX_BRIGHTNESS;
	bl_dev->props.brightness = DEFAULT_BRIGHTNESS;
	bl_dev->props.power = FB_BLANK_UNBLANK;
	
	dev->bl_dev = bl_dev;
	dev->client = i2c_dev;
	i2c_set_clientdata(i2c_dev, dev);

/* LGE_CHANGE_S [kyungtae.oh@lge.com] 2010-03-16, for prevent display flicker*/
	//aat2862_init(i2c_dev);
/* LGE_CHANGE_S [kyungtae.oh@lge.com] 2010-03-16, for prevent display flicker*/

	led_classdev_register(&i2c_dev->dev, &lcd_backlight);

	aat2862_set_main_current_level(i2c_dev, DEFAULT_BRIGHTNESS); 
	err = device_create_file(&i2c_dev->dev, &dev_attr_level);
	err = device_create_file(&i2c_dev->dev, &dev_attr_backlight_on_off);//20100825 kyungtae.oh@lge.com for test_mode

//20100709 kyungtae.oh@lge.com early_suspend [START_LGE]
#ifdef CONFIG_HAS_EARLYSUSPEND
    dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    dev->early_suspend.suspend = aat2862_early_suspend;
    dev->early_suspend.resume = aat2862_late_resume;
    register_early_suspend(&dev->early_suspend);
#endif
//20100709 kyungtae.oh@lge.com early_suspend [END_LGE]

	return 0;
}

static int aat2862_remove(struct i2c_client *i2c_dev)
{
	struct aat2862_device *dev;

    unregister_early_suspend(&dev->early_suspend);
	gpio_free(MY_LCD_CP_EN);
 	device_remove_file(&i2c_dev->dev, &dev_attr_level);
 	device_remove_file(&i2c_dev->dev, &dev_attr_backlight_on_off);//20100825 kyungtae.oh@lge.com for test_mode
	dev = (struct aat2862_device *)i2c_get_clientdata(i2c_dev);
	backlight_device_unregister(dev->bl_dev);
	i2c_set_clientdata(i2c_dev, NULL);

	return 0;
}	

static int aat2862_suspend(struct i2c_client *client, pm_message_t state)
{
	//printk(KERN_INFO"%s: new state: %d\n",__func__, state.event);

	// 20100630 jh.koo@lge.com Hub touchscreen power sequence [START_LGE]
	aat2862_touch_ldo_enable(client, 0);
	// 20100630 jh.koo@lge.com Hub touchscreen power sequence [END_LGE]

	// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [START_LGE]
#if 0	// Followings reduces around 20uA.
	gpio_direction_output(MY_HDMI_REG_EN, 0);
	gpio_set_value(MY_HDMI_REG_EN, 0);
	gpio_direction_output(MY_CAM_SUBPM_EN, 0);
	gpio_set_value(MY_CAM_SUBPM_EN, 0);
	gpio_direction_output(MY_CAM_VCM_EN, 0);
	gpio_set_value(MY_CAM_VCM_EN, 0);
	gpio_direction_output(MY_DMB_EN, 0);
	gpio_set_value(MY_DMB_EN, 0);
	gpio_direction_output(MY_MOTION_INT, 0);
	gpio_direction_output(MY_COM_INT, 0);
#endif
#if 0	// Followings reduces around 60uA.
	i2c3_scl = omap_readw(0x480021C2);	// sookyoung.kim	
	i2c3_sda = omap_readw(0x480021C4);	// sookyoung.kim	
	omap_writew(0x0000, 0x480021C2);	// sookyoung.kim	
	omap_writew(0x0000, 0x480021C4);	// sookyoung.kim	
#endif
#if 0	// Followings, coupled with memory refresh rate control, reduces around 600uA.
	i2c4_scl = omap_readw(0x48002A00);	// sookyoung.kim	
	i2c4_sda = omap_readw(0x48002A02);	// sookyoung.kim	
	omap_writew(0x0000, 0x48002A00);	// sookyoung.kim	
	omap_writew(0x0000, 0x48002A02);	// sookyoung.kim	
#endif
	// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [END_LGE]


	client->dev.power.power_state = state;

	aat2862_write_reg(client, 0x02, 0x00);

	gpio_direction_output(MY_LCD_CP_EN, 0);
	gpio_set_value(MY_LCD_CP_EN, 0);

	return 0;
}

//20100709 kyungtae.oh@lge.com EARLYSUSPEND [START_LGE]
#ifdef CONFIG_HAS_EARLYSUSPEND
static int aat2862_bl_suspend(struct i2c_client *client, pm_message_t state)
{
    //printk(KERN_INFO"%s: new state: %d\n",__func__, state.event);

    //client->dev.power.power_state = state;
    aat2862_backlight_off();
    aat2862_write_reg(client, 0x03, 0x0F);
    aat2862_write_reg(client, 0x04, 0x0F);
    aat2862_write_reg(client, 0x05, 0x0F);
    aat2862_write_reg(client, 0x06, 0x0F);
    early_bl_timer = 0;
    return 0;
}
#endif
//20100709 kyungtae.oh@lge.com EARLYSUSPEND  [END_LGE]

static int aat2862_resume(struct i2c_client *client)
{
	aat2862_init(client);
	//printk(KERN_INFO"%s: old state: %d\n",__func__, client->dev.power.power_state.event);
	client->dev.power.power_state = PMSG_ON;
	// 20100525 sookyoung.kim@lge.com Patch for touch/panel resumption failure [START_LGE]
	aat2862_write_reg(client, 0x00, 0x4C);
	aat2862_write_reg(client, 0x01, 0x49);
	// 20100525 sookyoung.kim@lge.com Patch for touch/panel resumption failure [END_LGE]
	aat2862_write_reg(client, 0x02, 0x03);
	mdelay(10);

	// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [START_LGE]
#if 0	// Following reduces around 20uA.
	gpio_direction_output(MY_HDMI_REG_EN, 1);
	gpio_set_value(MY_HDMI_REG_EN, 1);
	gpio_direction_output(MY_CAM_SUBPM_EN, 1);
	gpio_set_value(MY_CAM_SUBPM_EN, 1);
	gpio_direction_output(MY_CAM_VCM_EN, 1);
	gpio_set_value(MY_CAM_VCM_EN, 1);
	gpio_direction_output(MY_DMB_EN, 1);
	gpio_set_value(MY_DMB_EN, 1);
	gpio_direction_input(MY_MOTION_INT);
	gpio_direction_input(MY_COM_INT);
#endif
#if 0	// Followings reduces around 60uA.
	omap_writew(i2c3_scl, 0x480021C2);	// sookyoung.kim	
	omap_writew(i2c3_sda, 0x480021C4);	// sookyoung.kim	
#endif
#if 0	// Followings, coupled with memory refresh rate control, reduces around 600uA.
	omap_writew(i2c4_scl, 0x48002A00);	// sookyoung.kim	
	omap_writew(i2c4_sda, 0x48002A02);	// sookyoung.kim	
#endif
	// 20101016 sookyoung.kim@lge.com Turn off unnecessary modules upon BL off [END_LGE]

	// 20100630 jh.koo@lge.com Hub touchscreen power sequence [START_LGE]
	aat2862_touch_ldo_enable(client, 1);
	// 20100630 jh.koo@lge.com Hub touchscreen power sequence [END_LGE]	

	//aat2862_backlight_on();

	return 0;
}

//20100709 kyungtae.oh@lge.com EARLYSUSPEND [START_LGE]
#ifdef CONFIG_HAS_EARLYSUSPEND
static int aat2862_bl_resume(struct i2c_client *client)
{
    early_bl_timer = 1;
    aat2862_backlight_on();
    
    return 0;
}

static void aat2862_early_suspend(struct early_suspend *h)
{
    struct aat2862_device *dev;
 
    dev = container_of(h, struct aat2862_device, early_suspend);
    aat2862_bl_suspend(dev->client, PMSG_SUSPEND);
}

static void aat2862_late_resume(struct early_suspend *h)
{
    struct aat2862_device *dev;
 
    dev = container_of(h, struct aat2862_device, early_suspend);
    aat2862_bl_resume(dev->client);
}
#endif
//20100709 kyungtae.oh@lge.com EARLYSUSPEND  [END_LGE]

static struct i2c_driver main_aat2862_driver = {
	.probe = aat2862_probe,
	.remove = aat2862_remove,
	.id_table = hub_bl_id, 
	.suspend = aat2862_suspend,
	.resume = aat2862_resume,
	.driver = {
		.name = HUB_I2C_BL_NAME,
	},
};


static int __init lcd_backlight_init(void)
{
	static int err=0;

	err = i2c_add_driver(&main_aat2862_driver);

	return err;
}
 
module_init(lcd_backlight_init);

MODULE_DESCRIPTION("AAT2862 Backlight Control");
MODULE_AUTHOR("kyungtae Oh <kyungtae.oh@lge.com>");
MODULE_LICENSE("GPL");
