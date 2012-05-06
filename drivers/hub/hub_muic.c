/*
 * Hub TI TS5USBA33402 MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

//#include <linux/config.h>	// FIXME: What is this for? No hub_rev_a.h for kernel but x-loader and u-boot.
#include <linux/module.h>
#include <linux/kernel.h>	// printk()
#include <linux/init.h>		// __init, __exit
#include <linux/uaccess.h>	// copy_from/to_user()
#include <linux/interrupt.h>	// request_irq()
#include <linux/irq.h>		// set_irq_type()
#include <linux/types.h>	// kernel data types
#include <asm/system.h>
// kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
// in turn, includes kernel/include/asm-generic/gpio.h.
// <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
// <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
// The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>	// usleep()
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	// INIT_WORK()
#include <linux/wakelock.h>

#include <linux/i2c/twl.h>		//for akm power
#include <linux/regulator/consumer.h>	//for akm power

#include "hub_muic.h"
#include "hub_charging_ic.h"
#include "../mux.h"

static const char name_muic_mode[MUIC_MODE_NO][30] = {
	"MUIC_UNKNOWN",		// 0
	"MUIC_NONE",   		// 1
	"MUIC_NA_TA",   	// 2
	"MUIC_LG_TA",   	// 3
	"MUIC_HCHH", 	  	// 4
	"MUIC_INVALID_CHG",  	// 5
	"MUIC_AP_UART",   	// 6
	"MUIC_CP_UART",		// 7
	"MUIC_AP_USB", 		// 8
	"MUIC_CP_USB",		// 9
	"MUIC_TV_OUT_NO_LOAD",	// 10
	"MUIC_EARMIC",		// 11
	"MUIC_TV_OUT_LOAD",	// 12
	"MUIC_OTG",   		// 13
	"MUIC_CP_DOWNLOAD",	// 14
	"MUIC_RESERVE1",	// 15
	"MUIC_RESERVE2",	// 16
};

static struct i2c_client *muic_client;
static struct work_struct muic_wq;
//struct wake_lock muic_wakelock; 	// FIXME: What is this for?
void charger_state_update_by_other(void);

TYPE_USIF_MODE usif_mode = USIF_AP;
TYPE_DP3T_MODE dp3t_mode = DP3T_NC;
TYPE_UPON_IRQ  upon_irq = NOT_UPON_IRQ;

TYPE_MUIC_MODE muic_mode = MUIC_UNKNOWN;

static u8 int_stat_val;
static u8 status_val;

static int muic_init_done=0;     // 20110113 ks.kwon@lge.com check muic driver init. state

// wake lock for usb connection
struct wlock {
	int wake_lock_on;
	struct wake_lock wake_lock;
};
static struct wlock the_wlock;
static int proc_write=0;

void dp3t_switch_ctrl(TYPE_DP3T_MODE mode);
void usif_switch_ctrl(TYPE_USIF_MODE mode);
s32 muic_AP_UART_set(void);
s32 muic_CP_UART_set(void);
s32 muic_AP_USB_set(void);
s32 muic_CP_USB_set(void);

int check_no_lcd();

void check_usb_reg(void); // by TI Prakash

static struct regulator *muic_vmmc1;
static struct regulator *muic_vmmc2;
static struct regulator *muic_vaux2;

extern 	int key_pressed;
extern  int key_row;
extern	int key_col;
static	int key_was_pressed=0;

extern	int reset_status;

/* S, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */
static int is_cp_retained;
/* E, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */

#ifdef CONFIG_PROC_FS
/*--------------------------------------------------------------------
 * BEGINS: Proc file system related functions for MUIC.
 *--------------------------------------------------------------------
 */
#define	HUB_MUIC_PROC_FILE "driver/hmuic"
static struct proc_dir_entry *hub_muic_proc_file;

/* S, 20110921, mschung@ubiquix.com, Support read current MUIC mode(/proc/dirver/hmuic). */
static ssize_t muic_proc_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
	int ret;
	char val[2]={0,};

	s32 i;
	u32 reg_val;

	static int read_done = 0;

	if( read_done )
	{
		read_done = 0;
		return 0;
	}

	for(i=0; i<=5; i++) {
		reg_val = i2c_smbus_read_byte_data(muic_client, i);
		printk(KERN_INFO "[MUIC] Reg 0x%X, Val=0x%X\n", i, reg_val);
	}

	printk(KERN_INFO "[MUIC] muic_proc_read(): muic_mode = %s (%d)\n", name_muic_mode[(s32)muic_mode], muic_mode);
	/* fill the buffer, return the buffer size */
	memset(val, 0, sizeof(val));
	val[0] = (char)muic_mode+'0';
	ret = sprintf( buf, "%s", val);

	read_done = 1;
	return ret;
}
/* E, 20110921, mschung@ubiquix.com, Support read current MUIC mode(/proc/dirver/hmuic). */

static ssize_t muic_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off){
	char messages[12];
	u32 val;
	u32 reg;
	s32 err;
	char cmd;

	if(len > 12)
		len = 12;

	if(copy_from_user(messages, buf, len))
		return -EFAULT;

	sscanf(buf, "%c 0x%x 0x%x", &cmd, &reg, &val);

	printk(KERN_INFO "[MUIC] LGE: MUIC_proc_write \n");

	proc_write = 1;

	switch(cmd){

		/* AP_UART mode*/
		case '6':
			muic_AP_UART_set();
			break;
		
		/* CP_UART mode*/
		case '7':
			muic_CP_UART_set();
			break;
			
		/* AP_USB mode*/
		case '8':
			is_cp_retained = 0; /* 20111105, mschung@ubiquix.com, Support AP_USB when CP USB retained. */
			if(!(muic_mode == MUIC_NONE))	{

				i2c_smbus_write_byte_data(muic_client, 0x03, 0x3F);
				msleep(30);

				printk(KERN_ERR "====^^==== 888 muic \n");
				muic_AP_USB_set(); //20101124 ks.kwon@lge.com 
				check_usb_reg(); // LGE TI Prakash
			}
			else {
				printk(KERN_WARNING "[MUIC] muic_proc, muic_mode = %d.\n", muic_mode);
				proc_write = 0;
				}
			break;

		/* CP_USB mode*/
		case '9':
			if(!(muic_mode == MUIC_NONE))	
				muic_CP_USB_set(); //20101124 ks.kwon@lge.com 
			else{
				printk(KERN_WARNING "[MUIC] muic_proc, muic_mode = %d.\n", muic_mode);
				proc_write = 0;
				}		
			break;
			
		case 'w':
			gpio_direction_output(101, 1);
			gpio_set_value(101, 1); 
			dp3t_switch_ctrl(DP3T_CP_USB);
			err = i2c_smbus_write_byte_data(muic_client, reg, val);
			printk(KERN_INFO "[MUIC] LGE: Hub MUIC Write Reg. = 0x%X, Value 0x%X\n", reg, val);
			break;
 
		default : 
				printk(KERN_INFO "[MUIC] LGE: Hub MUIC invalid command\n");
		        printk(KERN_INFO "[MUIC] 6: AP_UART, 7: CP_UART, 8: AP_USB, 9: CP_USB\n");
		        printk(KERN_INFO "[MUIC] or \"w address value\"\n");				
				proc_write = 0;
				break;
        }
	return len;
}

static struct file_operations hub_muic_proc_ops = {
	.read = muic_proc_read,
	.write = muic_proc_write,
};

static void create_hub_muic_proc_file(void){
	hub_muic_proc_file = create_proc_entry(HUB_MUIC_PROC_FILE, 0777, NULL);
	if(hub_muic_proc_file) {
		//hub_muic_proc_file->owner = THIS_MODULE; // 20100617 kernel API changed
		hub_muic_proc_file->proc_fops = &hub_muic_proc_ops;
	} else
		printk(KERN_INFO "[MUIC] LGE: Hub MUIC proc file create failed!\n");
}

static void remove_hub_muic_proc_file(void){
	remove_proc_entry(HUB_MUIC_PROC_FILE, NULL);
}

/*--------------------------------------------------------------------
 * ENDS: Proc file system related functions for MUIC.
 *--------------------------------------------------------------------
 */
#endif //CONFIG_PROC_FS

TYPE_MUIC_MODE get_muic_mode(void){
	return muic_mode;
}
EXPORT_SYMBOL(get_muic_mode);

/*
 * Function: Read the MUIC register whose internal address is addr
 *           and save the u8 content into value.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_read_byte(u8 addr, u8 *value){
	*value = i2c_smbus_read_byte_data(muic_client, (u8)addr);
	if(*value < 0){
		printk(KERN_INFO "[MUIC] muic_i2c_read_byte failed.\n");
		return *value;
	}
	else
		return 0;
}

/*
 * Function: Write u8 value to the MUIC register whose internal address is addr.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_write_byte(u8 addr, u8 value){
	s32 ret;
	ret = i2c_smbus_write_byte_data(muic_client, (u8)addr, (u8)value);
	if(ret < 0) printk(KERN_INFO "[MUIC] muic_i2c_write_byte failed.\n");
	return ret;
}

void usif_switch_ctrl(TYPE_USIF_MODE mode){

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}	
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */

	if(mode == USIF_AP){
		gpio_set_value(USIF_IN_1_GPIO, 0);
	}
	else if(mode == USIF_DP3T){
		gpio_set_value(USIF_IN_1_GPIO, 1);
	}
	else{
		/* Just keep the current path */
	}
	usif_mode = mode;
}
EXPORT_SYMBOL(usif_switch_ctrl);

void dp3t_switch_ctrl(TYPE_DP3T_MODE mode){
	if(mode == DP3T_AP_UART){
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
	}
	else if(mode == DP3T_CP_UART){
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
	}
	else if(mode == DP3T_CP_USB){
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
	}
	else if(mode == DP3T_NC){
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
	}
	else{
		/* Just keep the current path */
	}
	dp3t_mode = mode;
}

// 20100623 ks.kwon@lge.com  UART Switch/USB Switch Control for LGT REVA2 [START_LGE]
void uart_switch_ctrl(TYPE_UART_SW_MODE mode){
	if (mode == AP_UART)
		gpio_set_value(UART_SW, 0);
	else if (mode ==  USIF)
		gpio_set_value(UART_SW, 1);
	else{}
}

void usb_switch_ctrl(TYPE_USB_SW_MODE mode){
	if (mode == AP_USB)
		gpio_set_value(USB_SW, 0);
	else if (mode ==  CP_USB)
		gpio_set_value(USB_SW, 1);
	else{}
}
// 20100623 ks.kwon@lge.com  UART Switch/USB Switch Control for LGT REVA2 [END_LGE]

s32 muic_AP_UART_set(void){

	s32 ret;

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}	
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */	

	/* Turn on charger IC with FACTORY mode */
	charging_ic_set_factory_mode();
	/* Connect CP UART signals to AP */
//	usif_switch_ctrl(USIF_AP);
	/* Connect AP UART to MUIC UART */
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		uart_switch_ctrl(AP_UART);
	else
		dp3t_switch_ctrl(DP3T_AP_UART);
#else
	uart_switch_ctrl(AP_UART);		
#endif
	// ID_2P2, VLDO 2.6V, SEMREN on. ADC is auto.
	ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
	// INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP);
	// Connect DP, DM to UART_TX, UART_RX
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
	muic_mode = MUIC_AP_UART;
	printk(KERN_WARNING "[MUIC] muic_distinguish_vbus_accessory(): AP_UART\n");
	
	// wake lock for usb connection
	if(1==the_wlock.wake_lock_on){
		wake_unlock(&the_wlock.wake_lock);
		the_wlock.wake_lock_on=0;
		printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
	}
	// wake lock for usb connection

	return ret;
}

s32 muic_AP_USB_set(void){

	s32 ret;

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}	
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */

	/* Turn on charger IC with TA mode */
//	if(check_no_lcd())
//		charging_ic_set_factory_mode(); //taehwan.kim@lge.com 20101104 to fix trickle charge
	/* Connect CP UART signals to AP */
//	usif_switch_ctrl(USIF_AP);

#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		usb_switch_ctrl(AP_USB);
	else {	
	/* AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART. */
	dp3t_switch_ctrl(DP3T_AP_UART);
	}
#else
	usb_switch_ctrl(AP_USB);	
#endif

	// ID_200, VLDO 2.6V, SEMREN on. 
	// USB 2.0 also needs the charge pump (CP_EN) on.
	ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
	// INT_EN, CHG_TYP.
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP);
	// Connect DP, DM to USB_DP, USB_DM
	ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
	muic_mode = MUIC_AP_USB;
	printk(KERN_WARNING "[MUIC] muic_distinguish_vbus_accessory(): AP_USB\n");

	// wake lock for usb connection
	if(0==the_wlock.wake_lock_on){
		wake_lock(&the_wlock.wake_lock);
		the_wlock.wake_lock_on=1;
		printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=1 (locked)\n");

	}
	// wake lock for usb connection

	return ret;

}

s32 muic_CP_UART_set(void){

	s32 ret;

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}		
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */	

	/* Turn on charger IC with FACTORY mode */
	charging_ic_set_factory_mode();
	/* Connect CP UART signals to DP3T */
//	usif_switch_ctrl(USIF_DP3T);
	
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		uart_switch_ctrl(USIF);
	else { 
	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);
	}
#else
	uart_switch_ctrl(USIF);
#endif

	// ID_200, VLDO 2.6V, SEMREN on.
	ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
	// INT_EN, CHG_TYP.
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP);
	// Connect DP, DM to UART_TX, UART_RX
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
	muic_mode = MUIC_CP_UART;
	printk(KERN_WARNING "[MUIC] muic_distinguish_vbus_accessory(): CP_UART\n");

	//20101014 ks.kwon@lge.com, wake unlock only for HW rev. 1.0 [LGE_START]
	if (system_rev < 6){
	// wake lock for the factory mode
		if(0==the_wlock.wake_lock_on){
			wake_lock(&the_wlock.wake_lock);
			the_wlock.wake_lock_on=1;
			printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=1 (locked)\n");
		}
	}
	// wake lock for the factory mode	
	//20101014 ks.kwon@lge.com, wake unlock only for HW rev. 1.0 [LGE_END]

	return ret;	

}

s32 muic_CP_USB_set(void){

	s32 ret;

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}		
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */	

	/* IFX_USB_EN set to enable IFX_USB_VBUS */
	gpio_direction_output(101, 1);
	gpio_set_value(101, 1);

	/* Turn on charger IC with TA mode */
	charging_ic_set_factory_mode();
	/* Connect CP UART signals to AP */
//	usif_switch_ctrl(USIF_AP);
	
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		usb_switch_ctrl(CP_USB);
	else { 	
	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);
	}
#else
	usb_switch_ctrl(CP_USB);	
#endif

	// ID_200, VLDO 2.6V, SEMREN on. ADC is auto.
	// USB 2.0 also needs the charge pump (CP_EN) on.
	ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN | MCP_EN);
	// INT_EN, CHG_TYP, USB_DET_DIS on.
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP); //20100426 ks.kwon@lge.com Interrupt enable during CP_USB mode     

#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
	else {
	// Connect DP, DM to UART_TX, UART_RX
	ret = muic_i2c_write_byte(SW_CONTROL, DP_UART | DM_UART);
	}
#else
	ret = muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
#endif
	muic_mode = MUIC_CP_USB;
	printk(KERN_ERR "[MUIC] muic_distinguish_vbus_accessory(): CP_USB\n");

	//20101014 ks.kwon@lge.com, wake unlock only for HW rev. 1.0 [LGE_START]
	if ( (system_rev < 6) || (reset_status == 2) ){
	// wake lock for the factory mode
		if(0==the_wlock.wake_lock_on){
			wake_lock(&the_wlock.wake_lock);
			the_wlock.wake_lock_on=1;
			printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=1 (locked)\n");
		}
	}
	// wake lock for the factory mode	
	//20101014 ks.kwon@lge.com, wake unlock only for HW rev. 1.0 [LGE_END]
	return ret;
}


/* Initialize MUIC, i.e., the CONTROL_1,2 and SW_CONTROL registers.
 * 1) Prepare to sense INT_STAT and STATUS bits.
 * 2) Open MUIC paths. -> To keep the path from uboot setting, remove this stage.
 */ 
void muic_initialize(TYPE_RESET reset){

	s32 ret;

	printk(KERN_WARNING "[MUIC] muic_initialize()\n");

	/* Reset MUIC - Disable all */
	if(reset){
		ret = muic_i2c_write_byte(SW_CONTROL, DP_OPEN | DM_OPEN); // 20110915, mschung@ubiquix.com, Debug USB recognition at reboot.				
		ret = muic_i2c_write_byte(CONTROL_1, 0x00);
		ret = muic_i2c_write_byte(CONTROL_2, MUSB_DET_DIS);	// USB_DET_DIS is neg enabled
	}

	/* Initialize MUIC - Default setting.
	 *
	 * CONTROL_1:
	 * 
	 * 	ID_2P2 	= 0. Enable to distinguish MUIC_EARMIC from MUIC_TV_OUT_LOAD and MUIC_OTG.
	 * 		     Enable for MUIC_EARMIC operation.
	 *	ID_620 	= 0. Enable only to distinguish MUIC_TV_OUT_LOAD from MUIC_OTG.
	 *	ID_200 	= 1.
	 *	VLDO 	= 0. Enable to apply 2.3V for MUIC_EARMIC operation.
	 *	SEMREN 	= 1.
	 *	ADC_EN 	= 0. Because it is automatically enabled upon any change in ID resistor.
	 *	CP_EN 	= 0. Enalbe for USB 2.0 (MUIC_AP_USB, MUIC_CP_USB, and MUIC_OTG).
	 *		     Enable for Audio charge pump (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
	 * 
	 * CONTROL_2: 
	 *
	 * 	INTPOL 	= 0.
	 * 	INT_EN	= 1.
	 * 	MIC_LP	= 0.
	 * 	CP_AUD 	= 1. Disable for Audio operation (MUIC_EARMIC, MUIC_TV_OUT_LOAD).
	 * 	CHG_TYP	= 1.
	 * 	USB_DET_DIS = 0. Negative enabled.
	 *
	 * SW_CONTROL: 
	 *
	 * 	MIC_ON	= 0. Enable for MUIC_EARMIC and MUIC_TV_OUT_LOAD.
	 * 	DP	= 111 (open).
	 * 	DM	= 111 (open).
	 */
	ret = muic_i2c_write_byte(CONTROL_1, MID_200 | MSEMREN);
	ret = muic_i2c_write_byte(CONTROL_2, MCHG_TYP);


	/* The initialization time for the facility to set STATUS register's
	 * DCPORT and CHPORT = 250msec since CHG_TYP on.
	 * The initialization times for the facilities to set INT_STAT register bits
	 * since CONTROL_1, 2 settings are much shorter (< 70msec).
	 * The settle down time for INT_STAT and STATUS register bits
	 * since an interrupt occurs = 70msec.
	 * 
	 * Thus,
	 * we need to wait 250msec if we enable CHG_TYP from its inactive state.
	 * And, we need to wait just 70msec for all other cases including
	 * interrupt and CONTROL register settings.
	 */
	if(reset)
		msleep(250);
	else
		msleep(70);

	/* Enable interrupt (INT_EN = 1). Keep other bits the same */
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN |MCHG_TYP);
	
	/* Default setting : CP_UART to UART Switch. */
	usif_switch_ctrl(USIF_DP3T);

	return;
}

/* Distinguish charger type.
 * This function is called *ONLY IF* INT_STAT's CHGDET == 1, i.e., a charger is detected.
 *
 * Chargers becomes to use the default MUIC setting
 * because the detection always happens only after the MUIC_NONE mode
 * which leads the default MUIC setting.
 * Thus, we don't need to explictly set MUIC registers here.
 */ 
s32 muic_distinguish_charger(void){

	s32 ret = 0;

	printk(KERN_WARNING "[MUIC] muic_distinguish_charger()\n");

	/* Enable charger IC in TA mode */
	//charging_ic_set_ta_mode(); //20101104 taehwan.kim@lge.com to fix
    //trickle charge

#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2)
		uart_switch_ctrl(AP_UART);
	else { 	
	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);
	}
#endif
	/* Read STATUS */
	ret = muic_i2c_read_byte(STATUS, &status_val);
	if(ret < 0){
		printk(KERN_INFO "[MUIC] STATUS reading failed\n");
		muic_mode = MUIC_UNKNOWN;
		key_was_pressed = 0;
		return ret;
	}

	/* DCPORT == 1. NA_TA or LG_TA */
	if((status_val & MDCPORT) != 0){

		/* DCPORT == 1 && CHPORT == 1. Failed to detect charger type. Try again.
		 * CAUTION!!! Trying again can hang the system. Keep your eye on it closely.
		 */
		if((status_val & MCHPORT) != 0){
			//printk(KERN_WARNING "[MUIC]Failed to detect charger type. Try again!\n");
			muic_mode = MUIC_INVALID_CHG;
        	printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): MUIC_INVALID_CHG, MCHPORT = 1, DCPORT = 1\n");
		//	return -1;
		}
		/* DCPORT == 1 && CHPORT == 0. Definitely NA_TA or LG_TA */
		else{
			/* IDNO == 0x05, i.e., 180KOhm. NA_TA */
			if((int_stat_val & MIDNO) == 0x05){
				muic_mode = MUIC_NA_TA;
        	    printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): MUIC_NA_TA\n");

			}
			/* IDNO != 0x05. LG_TA by default */
			else{
//				printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): ");
				printk(KERN_WARNING "IDNO = 0x%x\n", (int_stat_val & MIDNO));
				muic_mode = MUIC_LG_TA;
            	printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): MUIC_LG_TA\n");
			}
		}
	}
	/* DCPORT == 0. HCHH or INVALID_CHG */
	else{
		/* DCPORT == 0 && CHPORT == 1. Definitely HCHH */
		if((status_val & MCHPORT) != 0){
			muic_mode = MUIC_HCHH;
        	printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): MUIC_HCHH\n");
		}
		/* DCPORT == 0 && CHPORT == 0. Definitely INVALID_CHG */
		else{
			muic_mode = MUIC_INVALID_CHG;
        	printk(KERN_WARNING "[MUIC] muic_distinguish_charger(): MUIC_INVALID_CHG, MCHPORT = 1, DCPORT = 0\n");
		}
	}
	return ret;
}

/* Distinguish accessory type which supplies VBUS.
 * These accessories includes AP_UART, CP_UART, AP_USB, and CP_USB.
 * It should be called *ONLY IF* VBUS == 1.
 */
s32 muic_distinguish_vbus_accessory(s32 upon_irq){

	s32 ret = 0;
	
	if((key_col == 6)&&(key_row == 1))
		key_was_pressed = 1;
	//else if ((key_col == 6)&&(key_row == 0))   
	//	key_was_pressed = 2;


	/* S, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */
	if (is_cp_retained)
	{
		printk(KERN_INFO "[MUIC] It is CP Retain mode. set usb path to CP_USB.\n");
		muic_CP_USB_set();
	}
	else
	/* E, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */

	{
		switch(int_stat_val & MIDNO){
#if 1	
			/* AP_UART */
			case 0x02: //20100918 ks.kwon@lge.com add key detection for AP_UART
				if(key_was_pressed == 1){
					muic_AP_UART_set();
				} else {
					muic_CP_USB_set();
				}
				break;
#endif
				/* CP_UART */
			case 0x04:
				muic_CP_UART_set();
				break;

				/* AP_USB */
			case 0x05:  //20100501 ks.kwon@lge.com Add 180K mode for VBUS Accessory
			case 0x0b:
				//	if(key_was_pressed == 2)
				//		muic_CP_USB_set();
				//	else        
				muic_AP_USB_set();
				check_usb_reg(); /* 20111105, mschung@ubiquix.com, for stable USB connection. */
				break;
			case 0x0a : //201021 ks.kwon@lge.com add 910K, CP_USB by factory demand
				muic_CP_USB_set();
				muic_mode = MUIC_CP_DOWNLOAD; //20101126 ks.kwon@lge.com notifying to the backlight driver.
				break;

				/* Default = Unknown accessory */
			default:
				//			printk(KERN_WARNING "[MUIC] muic_distinguish_vbus_accessory(): ");
				printk(KERN_WARNING "[MUIC] muic mode is illegal charger. (USB.org's spec out)\n");
				muic_mode = MUIC_ILLEGAL_CHG;
				key_was_pressed = 0;

				break;
		}
	}

	mdelay(1);	// MUIC DP/DM path settle down and CP_EN delay

	return ret;
}



/*
 * ID_200 Set		Connect UID LDO to ID pin via 200KOhm
 * VLDO Reset		Use 2.6V for all accessory detection (except for MIC bias)
 * SEMREN Set		Enable Send-End, Mic removal comparators and UID LDO -> Sets MR_COMP, SEND/END bits
 * ADC_EN Set		Enable ADC and UID LDO to generate IDNO. TI MUIC turns on this automatically.
 * CP_EN Set		Enable charge pump for USB2.0 and headset.
 * INT_EN Set		Enable MUIC interrupt generation to OMAP
 * CHG_TYP Set		Enable charger type recognition (TA or HCHH) -> Sets DCPORT, CHPORT bits
 * USB_DET_DIS Reset	Enable charger detection (Charger or USB) -> Sets CHGDET bit
 *
 * MUIC MODE
 *
 * V C D C IDNO IDNO IDNO
 * B H C H 200K 2.2K 620
 * U G P P
 * S D O O
 *   E R R
 *   T T T
 *Cases detected by muic_distinguish_charger():
 * 1 1 1 - 0101 ---- ---- NA_TA (ID resistor 180KOhm) - Not used actually.
 * 1 1 1 - ---- ---- ---- LG_TA
 * 1 1 0 1 ---- ---- ---- HCHH (High current host/hub charger) - Not used actually.
 * 1 1 0 0 ---- ---- ---- Invalid charger
 *
 *Cases detected by muic_distinguish_vbus_accessory():
 * 1 0 0 0 0010 ---- ---- AP_UART (ID resistor 56KOhm)
 * 1 0 0 0 0100 ---- ---- CP_UART (ID resistor 130KOhm)
 * 1 0 0 0 1011 ---- ---- AP_USB (ID resistor open)
 * 1 0 0 0 ???? ---- ---- CP_USB (ID resistor ????) - Not defined yet.
 *
 *Cases detected by muic_distinguish_non_vbus_accessory():
 * 0 0 - - 0001 ---- ---- TV_OUT_NO_LOAD (ID resistor 24KOhm.) - Not used.
 * 0 0 - - 0000 01XX ---- EARMIC (ID resistor ground)
 * 0 0 - - 0000 0000 0001 TV_OUT_LOAD (ID resistor ground) - Not used.
 * 0 0 - - 0000 0000 0000 OTG (ID resistor ground) - Not used.
 */
s32 TS5USBA33402_device_detection(s32 upon_irq){

	s32 ret = 0;

      /* 20110113 ks.kwon@lge.com check muic driver init. state [START] */
	if(!muic_init_done){

		printk(KERN_WARNING "[MUIC] MUIC has not been initialized! Nothing will be done!!!.\n");
		return 0;
	}	
      /* 20110113 ks.kwon@lge.com check muic driver init. state [END] */

	/* Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 *
	 * Do this only if TS5USBA33402_device_detection() was called upon IRQ. */
	if(upon_irq) msleep(70);

	/* Read INT_STAT */
	ret = muic_i2c_read_byte(INT_STAT, &int_stat_val);
	if(ret < 0){
		printk(KERN_INFO "[MUIC] INT_STAT reading failed\n");
		proc_write=0;
        // wake lock for usb connection
        if(1==the_wlock.wake_lock_on){
            wake_unlock(&the_wlock.wake_lock);
            the_wlock.wake_lock_on=0;
            printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
        }
        // wake lock for usb connection
		muic_mode = MUIC_UNKNOWN;
		key_was_pressed = 0;
		return ret;
	}
//    printk(KERN_WARNING "[MUIC]IDNO = %d\n", (int_stat_val & MIDNO));
    printk(KERN_WARNING "[MUIC]int_stat_val = 0x%x\n", int_stat_val);
    ret = muic_i2c_write_byte(CONTROL_2, 0x00); //interrupt masked by ks.kwon@lge.com for debugging

	printk(KERN_WARNING "[MUIC]key_pressed = %d, key_col = %d, key_row = %d, key_was_pressed = %d.\n", key_pressed, key_col, key_row, key_was_pressed);

	/* Branch according to the previous muic_mode */
	switch(muic_mode){

		/* MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
		 * First, at the initialization time where the muic_mode is not available yet.
		 * Second, whenever the current muic_mode detection is failed.
		 */
		case MUIC_UNKNOWN:

		/* If the previous muic_mode was MUIC_NONE,
		 * the only possible condition for a MUIC IRQ is plugging in an accessory.
		 */
		case MUIC_NONE:

            		/* VBUS == 1.
			 * MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, MUIC_INVALID_CHG,
			 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, or MUIC_CP_USB.
			 */
			if((int_stat_val & MVBUS) != 0 && proc_write == 0){
				/* CHGDET == 1, i.e., D+/D- are short. Charger is detected.
				 * MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, or MUIC_INVALID_CHG.
				 */
				if((int_stat_val & MCHGDET) != 0){
					/* Charger type disctinction */
					ret = muic_distinguish_charger();
				}
				/* CHGDET == 0, i.e., D+/D- are not short. No charger is detected.
				 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB.
				 */
				else{
					/* VBUS accessory type disctinction */
					ret = muic_distinguish_vbus_accessory(upon_irq);
				}
			}
			/* VBUS == 0.
			 * MUIC_TV_OUT_NO_LOAD, MUIC_EARMIC, MUIC_TV_OUT_LOAD, MUIC_OTG, or MUIC_NONE.
			 */
			else{
				/* IDNO == 0x0b, i.e., ID pin is open. No accessory is plugged in.
				 * MUIC_NONE.
				 */
//				if(((int_stat_val & MVBUS) == 0x00) && (((int_stat_val & MIDNO) == 0x0b) || (int_stat_val & MIDNO) == 0x05))){
					muic_mode = MUIC_NONE;
                // wake lock for usb connection
                if(1==the_wlock.wake_lock_on){
                    wake_unlock(&the_wlock.wake_lock);
                    the_wlock.wake_lock_on=0;
                    printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
                }
                // wake lock for usb connection
					key_was_pressed = 0;
					proc_write = 0;
					printk(KERN_WARNING "[MUIC] MUIC_NONE\n");

//				}
			}
			break;

		/* If the previous muic_mode was MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, MUIC_INVALID_CHG,
		 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
		 * the only possible condition for a MUIC IRQ is plugging out the accessory.
		 * 
		 * In this case, initialize MUIC and wait an IRQ.
		 * We don't need to wait 250msec because this is not an erronous case
		 * (we need to reset the facility to set STATUS for an erronous case and
		 * have to wait 250msec) and, if this is not an erronous case, the facility
		 * was already initialized at the system booting.
		 */
		case MUIC_NA_TA:
		case MUIC_LG_TA:
		case MUIC_HCHH:
		case MUIC_INVALID_CHG:
		case MUIC_AP_UART:
		case MUIC_CP_UART:
		case MUIC_AP_USB:
		case MUIC_CP_USB:
        case MUIC_ILLEGAL_CHG:
			/* Check if VBUS == 0 && IDNO == 0x1011 (open).
			 * If so, it's MUIC_NONE.
			 * Otherwise, it's an erronous situation. MUIC_UNKNOWN.
			 */
			if((int_stat_val & MVBUS) == 0){
				muic_mode = MUIC_NONE;
				key_was_pressed = 0;
				proc_write = 0;
				printk(KERN_WARNING "[MUIC] TS5USBA33402_device_detection(): MUIC_NONE\n");
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
			if (system_rev < 2) {
				/* DP3T switch has nothing to do with.
				 * Open DP3T */
				dp3t_switch_ctrl(DP3T_NC);	
			}
#endif
				//  Open DP, DM
				ret = muic_i2c_write_byte(SW_CONTROL, DP_OPEN | DM_OPEN);				

				// wake lock for usb connection
				if(1==the_wlock.wake_lock_on){
					wake_unlock(&the_wlock.wake_lock);
					the_wlock.wake_lock_on=0;
					printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
				}
				// wake lock for usb connection
				
			}
			// 20100603 ks.kwon@lge.com charger detect code when irq is occurred more than one time in a row [START_LGE]
			else if((int_stat_val & MVBUS) != 0 && proc_write == 0){
				/* CHGDET == 1, i.e., D+/D- are short. Charger is detected.
				 * MUIC_NA_TA, MUIC_LG_TA, MUIC_HCHH, or MUIC_INVALID_CHG.
				 */
				if((int_stat_val & MCHGDET) != 0){
					/* Charger type disctinction */
					ret = muic_distinguish_charger();
				}
				else{
					/* VBUS accessory type disctinction */
					ret = muic_distinguish_vbus_accessory(upon_irq);
				}
			
			}
			// 20100603 ks.kwon@lge.com charger detect code when irq is occurred more than one time in a row [END_LGE]
			
			else{
				if (proc_write == 0){
				muic_mode = MUIC_UNKNOWN;
                    // wake lock for usb connection
                    if(1==the_wlock.wake_lock_on){
                        wake_unlock(&the_wlock.wake_lock);
                        the_wlock.wake_lock_on=0;
                        printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
                    }
                    // wake lock for usb connection
				key_was_pressed = 0;
				ret = -1;
				printk(KERN_WARNING "[MUIC]Failed to distinguish muic_mode. Try again!\n");
				}
			}
			break;


		default:
			printk(KERN_WARNING "[MUIC]Failed to detect an accessory. Try again!");
			muic_mode = MUIC_UNKNOWN;
			key_was_pressed = 0;
			ret = -1;
			break;
	}	

	// INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
	ret = muic_i2c_write_byte(CONTROL_2, MINT_EN | MCHG_TYP); //interrupt enabled by ks.kwon@lge.com

    if(muic_mode == MUIC_UNKNOWN){

        muic_initialize(DEFAULT);
	proc_write = 0;
        // wake lock for usb connection
        if(1==the_wlock.wake_lock_on){
            wake_unlock(&the_wlock.wake_lock);
            the_wlock.wake_lock_on=0;
            printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
        }
        // wake lock for usb connection
    }
    if(muic_mode == MUIC_NONE) {        //20101002 ks.kwon@lge.com charging_ic_deactive only for MUIC_NONE        
        charging_ic_deactive();	        //20100506 ks.kwon@lge.com for charging animation, by demand from taehwan.kim
        proc_write = 0;
        // wake lock for usb connection
        if(1==the_wlock.wake_lock_on){
            wake_unlock(&the_wlock.wake_lock);
            the_wlock.wake_lock_on=0;
            printk(KERN_WARNING "[MUIC] ====^^==== wake_lock_on=0 (unlocked) \n");
        }
        // wake lock for usb connection
        printk(KERN_INFO "[MUIC]charging_ic_deactive()\n");
    }
    
	charger_state_update_by_other();
	return ret;
}
EXPORT_SYMBOL(TS5USBA33402_device_detection);

static void muic_wq_func(struct work_struct *muic_wq){
	s32 ret = 0;
	u32 retry_no;
	printk(KERN_INFO "[MUIC] muic_wq_func()\n");
	ret = TS5USBA33402_device_detection(UPON_IRQ);
	/* If an erronous situation occurs, try again */
	retry_no = 0;
	while(ret < 0 && retry_no < 3){
		printk(KERN_INFO "[MUIC] muic_wq_func(): TS5USBA33402_device_detection() failed %d times\n",
			retry_no);
		ret = TS5USBA33402_device_detection(NOT_UPON_IRQ);
		retry_no ++;
	}
}

static irqreturn_t muic_interrupt_handler(s32 irq, void *data){

	printk(KERN_WARNING "[MUIC] muic_interrupt_handler(): MUIC IRQ occurs! muic_mode = %s\n",
		name_muic_mode[(s32)muic_mode]);

	printk(KERN_INFO "[MUIC] MUIC IRQ occurs!\n");
	schedule_work(&muic_wq);
	return IRQ_HANDLED;
}

void vbus_irq_muic_handler(int state)
{
    if(muic_init_done){
        printk("[MUIC] %s vbus_state = %d \n",__func__,state);
        if (state == 2)
        {
            if (muic_mode == MUIC_NONE) schedule_work(&muic_wq);;
        }
        else
        {
            if (muic_mode != MUIC_NONE) schedule_work(&muic_wq);;
        }
    }
    printk("[MUIC] %s completed \n",__func__);
}
EXPORT_SYMBOL(vbus_irq_muic_handler);
/*
 * muic_probe() is called in the middle of booting sequence due to '__init'.
 * '__init' causes muic_probe() to be released after the booting.
 */
static s32 __init muic_probe(struct i2c_client *client, const struct i2c_device_id *id){

	struct device *akm_dev = &client->dev;
	s32 ret = 0;
	u32 retry_no;
	muic_client = client;

	// wake lock for usb connection
	wake_lock_init(&the_wlock.wake_lock, WAKE_LOCK_SUSPEND, "usb_connection");
	the_wlock.wake_lock_on=0;
	// wake lock for usb connection
	
	// GPIO initialization
	
	omap_mux_init_gpio(MUIC_INT_GPIO, OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(DP3T_IN_1_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(DP3T_IN_2_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(USIF_IN_1_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(101, OMAP_PIN_OUTPUT);			
	
   	 ret = gpio_request(101, "VBUS_EN");
	if(ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 101 VBUS_EN is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_request(USIF_IN_1_GPIO, "USIF switch control GPIO");
	if(ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 182 USIF1_SW is already occupied by other driver!\n");
		return -ENOSYS;
	}

#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2) {
		ret = gpio_request(UART_SW, "UART switch control GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 161 UART_SW GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
		ret = gpio_request(USB_SW, "USB switch control GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 162 USB_SW GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
	}
	else{	
		ret = gpio_request(DP3T_IN_1_GPIO, "DP3T switch control 1 GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 161 DP3T_IN_1_GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
		ret = gpio_request(DP3T_IN_2_GPIO, "DP3T switch control 2 GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 162 DP3T_IN_2_GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
	}
#else
		ret = gpio_request(UART_SW, "UART switch control GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 161 UART_SW GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
		ret = gpio_request(USB_SW, "USB switch control GPIO");
		if(ret < 0){
			printk(KERN_INFO "[MUIC] GPIO 162 USB_SW GPIO is already occupied by other driver!\n");
			return -ENOSYS;
		}
			
#endif


	ret = gpio_request(MUIC_INT_GPIO, "MUIC IRQ GPIO");
	if(ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 40 MUIC_INT_N is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_input(MUIC_INT_GPIO);
	if(ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 40 MUIC_INT_N direction initialization failed!\n");
		return -ENOSYS;
	}

	/* Register MUIC work queue function */
	INIT_WORK(&muic_wq, muic_wq_func);

	/* Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
	ret = request_irq(gpio_to_irq(MUIC_INT_GPIO),
			  muic_interrupt_handler,
			  IRQF_TRIGGER_FALLING,
			  "muic_irq",
			  &client->dev);
	if (ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 40 MUIC_INT_N IRQ line set up failed!\n");
		free_irq(gpio_to_irq(MUIC_INT_GPIO), &client->dev);
		return -ENOSYS;
	}

	/* Make the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	ret = enable_irq_wake(gpio_to_irq(MUIC_INT_GPIO));
	if(ret < 0){
		printk(KERN_INFO "[MUIC] GPIO 40 MUIC_INT_N wake up source setting failed!\n");
		disable_irq_wake(gpio_to_irq(MUIC_INT_GPIO));
		return -ENOSYS;
	}

	/* Prepare a human accessible method to control MUIC */
	create_hub_muic_proc_file();

 	//ks.kwon@lge.com, akm power on for not conflicting on i2c bus[START_LGE]
 	   muic_vmmc1 = regulator_get(akm_dev,"vmmc1");
 	   if (muic_vmmc1 == NULL) {
 	       printk(KERN_WARNING "[MUIC]Failed to get vmmc1 power regulator !! \n");
		}
	   muic_vmmc2 = regulator_get(akm_dev,"vmmc2");
 	   if (muic_vmmc2 == NULL) {
 	       printk(KERN_WARNING "[MUIC]Failed to get vmmc2 power regulator !! \n");
		}
	   muic_vaux2 = regulator_get(akm_dev,"vaux2");
 	   if (muic_vaux2 == NULL) {
 	       printk(KERN_WARNING "[MUIC]Failed to get vaux2 power regulator !! \n");
		}
	   

		printk(KERN_WARNING "[MUIC]turning on vmmc regulators. \n");	
		regulator_enable(muic_vmmc1);
		regulator_enable(muic_vmmc2);
		regulator_enable(muic_vaux2);
	
	    //ks.kwon@lge.com, akm power on for not conflicting on i2c bus [END_LGE]

        muic_init_done = 1; //20110113 ks.kwon@lge.com check muic driver init. state

	/* Initialize MUIC - Finally MUIC INT becomes enabled */
	muic_initialize(RESET);

	ret = TS5USBA33402_device_detection(NOT_UPON_IRQ);
	/* If an erronous situation occurs, try again */
	retry_no = 0;
	while(ret < 0 && retry_no < 3){
		printk(KERN_INFO "[MUIC] probe(): TS5USBA33402_device_detection() failed %d times\n", retry_no);
		ret = TS5USBA33402_device_detection(NOT_UPON_IRQ);
		retry_no ++;
	}

	printk(KERN_WARNING "[MUIC] muic_probe()done!\n");
	return ret;
}

static s32 muic_remove(struct i2c_client *client){
	free_irq(gpio_to_irq(MUIC_INT_GPIO), &client->dev);
	gpio_free(MUIC_INT_GPIO);
	gpio_free(USIF_IN_1_GPIO);
#if defined(CONFIG_MACH_LGE_HUB_REV_A)
	if (system_rev >= 2) {
		gpio_free(UART_SW);
		gpio_free(USB_SW);
	} else     {	
		gpio_free(DP3T_IN_1_GPIO);
		gpio_free(DP3T_IN_2_GPIO);
	}
#else
	gpio_free(UART_SW);
	gpio_free(USB_SW);
#endif	
	i2c_set_clientdata(client, NULL);	// For what?
	remove_hub_muic_proc_file();

	// wake lock for usb connection
	wake_lock_destroy(&the_wlock.wake_lock);
	// wake lock for usb connection
	
	return 0;
}

static s32 muic_suspend(struct i2c_client *client, pm_message_t state){
	client->dev.power.power_state = state;
	printk(KERN_WARNING "[MUIC]turning off twl regulators(vmmc1, vmmc2, vaux2). \n");	
	regulator_disable(muic_vmmc1);
	regulator_disable(muic_vmmc2);
	regulator_disable(muic_vaux2);
	return 0;
}
		
static s32 muic_resume(struct i2c_client *client){
	client->dev.power.power_state = PMSG_ON;
 	//ks.kwon@lge.com, akm power on for not conflicting on i2c bus[START_LGE]
 	  printk("[MUIC]turning on twl regulators(vmmc1, vmmc2, vaux2). \n");
	regulator_enable(muic_vmmc1);
	regulator_enable(muic_vmmc2);
	regulator_enable(muic_vaux2);
	//ks.kwon@lge.com, akm power on for not conflicting on i2c bus [END_LGE]
	TS5USBA33402_device_detection(NOT_UPON_IRQ);
	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{"hub_i2c_muic", 0},
	{/* end of list */},
};

/*
 * Allow user space tools to figure out which devices this driver can control.
 * The first parameter should be 'i2c' for i2c client chip drivers.
 */
//MODULE_MUIC_TABLE(i2c, muic_ids);

static struct i2c_driver muic_driver = {
	.probe	 	= muic_probe,
	.remove	 	= muic_remove,
	.suspend 	= muic_suspend,
	.resume  	= muic_resume,
	.id_table	= muic_ids,
	.driver	 	= {
	.name       = "hub_i2c_muic",
	.owner      = THIS_MODULE,
	},
};

extern int hub_cp_usb_mode;
static s32 __init muic_init(void){
	printk(KERN_WARNING "[MUIC] muic_init()\n");

	/* S, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */
	if (hub_cp_usb_mode == 1) {
		printk("[MUIC] CP_USB_MODE\n");
		muic_mode = MUIC_NONE;
		is_cp_retained = 1;
	}
	else {
		printk("[MUIC] AP_USB_MODE\n");
		is_cp_retained = 0;
	}
	/* E, 20110921, mschung@ubiquix.com, Support USB AP/CP retain mode. */

	return i2c_add_driver(&muic_driver);
}

static void __exit muic_exit(void){
	i2c_del_driver(&muic_driver);
}

module_init(muic_init);
module_exit(muic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Hub MUIC Driver");
MODULE_LICENSE("GPL");
