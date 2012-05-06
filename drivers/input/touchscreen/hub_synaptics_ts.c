/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/gpio.h>
//#include <linux/i2c/twl4030.h>
#include <linux/jiffies.h>
#include <linux/slab.h>


/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
#include <linux/dvs_suite.h>
/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */

// 20110131 jh.koo@lge.com, Hub Touch screen FW Upgrade [START_LGE]
#include "RMI4Funcs.h"
#include "UserI2C.h"
#include "PR792480-tm1534-001.h"		// FW Rev. 7

#define HUB_TS_INT	35
// 20110131 jh.koo@lge.com, Hub Touch screen FW Upgrade [END_LGE]


#define SYNAPTICS_TOUCH_DEBUG 0
 #if SYNAPTICS_TOUCH_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif

// #define HUB_MULTI_TOUCH
 
#define HUB_TS_EN_GPIO 		65
#define HUB_TS_POLLING_TIME 	15 /* polling time(msec) when touch was pressed */ 
#define SYNAPTICS_INT_REG		0x50
#define SYNAPTICS_INT_FLASH		1<<0
#define SYNAPTICS_INT_STATUS 	1<<1
#define SYNAPTICS_INT_ABS0 		1<<2
#define SYNAPTICS_INT_BUTTON	1<<3

#define SYNAPTICS_CONTROL_REG		0x4F
#define SYNAPTICS_CONTROL_SLEEP 	1<<0
#define SYNAPTICS_CONTROL_NOSLEEP	1<<2


static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int reported_finger_count;
	int8_t sensitivity_adjust;
	int (*power)(int on);
// 20100504 jh.koo@lge.com, correction of finger space [START_LGE]
	unsigned int count;
	int x_lastpt;
	int y_lastpt;
// 20100504 jh.koo@lge.com, correction of finger space [END_LGE]	
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

/*===========================================================================
                DEFINITIONS AND DECLARATIONS FOR MODULE

This section contains definitions for constants, macros, types, variables
and other items needed by this module.
===========================================================================*/

/*                               Macros                                    */
// 0x00 - not present, 0x01 - present & accurate, 0x10 - present but not accurate, 0x11 - Reserved
#define TS_SNTS_GET_FINGER_STATE_0(finger_status_reg) \
		(finger_status_reg&0x03)
#define TS_SNTS_GET_FINGER_STATE_1(finger_status_reg) \
		((finger_status_reg&0x0C)>>2)
#define TS_SNTS_GET_FINGER_STATE_2(finger_status_reg) \
		((finger_status_reg&0x30)>>4)
#define TS_SNTS_GET_FINGER_STATE_3(finger_status_reg) \
      ((finger_status_reg&0xC0)>>6)
#define TS_SNTS_GET_FINGER_STATE_4(finger_status_reg) \
      (finger_status_reg&0x03)

#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)(low_reg&0x0F))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)((low_reg&0xF0)/0x10))

#define TS_SNTS_HAS_PINCH(gesture_reg) \
		((gesture_reg&0x40)>>6)
#define TS_SNTS_HAS_FLICK(gesture_reg) \
		((gesture_reg&0x10)>>4)
#define TS_SNTS_HAS_DOUBLE_TAP(gesture_reg) \
		((gesture_reg&0x04)>>2)

#define TS_SNTS_GET_REPORT_RATE(device_control_reg) \
		((device_control_reg&0x40)>>6)
// 1st bit : '0' - Allow sleep mode, '1' - Full power without sleeping
// 2nd and 3rd bit : 0x00 - Normal Operation, 0x01 - Sensor Sleep
#define TS_SNTS_GET_SLEEP_MODE(device_control_reg) \
		(device_control_reg&0x07)

#define BLUE_LED_GPIO (OMAP_MAX_GPIO_LINES + TWL4030_GPIO_MAX)

/*                         CONSTANTS DATA DEFINITIONS                      */
#define FINGER_MAX 10 //최대 5개 손가락 사용  

#define TS_DEBUG
#define TS_DEBUG_BUFFER_SIZE		1024

/*
#ifndef TS_USES_IC_GESTURE
#define FLICK_TIME_THRESHOLD		22		// 8
#define FLICK_DISTANCE_THRESHOLD	30		// 15
#endif//TS_USES_IC_GESTURE 
//#define DEFAULT_POLLING_PERIOD		60
//#define HOLD_THRESHOLD				17		// 20		// 15
//#define LONG_THRESHOLD				1000//40   //25		// 10
*/
#define TS_SNTS_X_AXIS_MAX_TM1343_002 1441
#define TS_SNTS_Y_AXIS_MAX_TM1343_002 840
#define FLICK_RANGE 4
#define LONG_PRESS_RANGE 6
#define TAP_TH      10
#define DISTANCE_TH    15
#define SPREADING_TH   20
#define PINCHING_TH    20 

//1497

//------------------------------------------------------------------------------------------------
// Register setting value
//------------------------------------------------------------------------------------------------

  /* Register map header file for TM1343, Family 0x01, Revision 0x01 */
  /* Automatically generated on 2009-Aug-10  10:45:05, do not edit */

#ifndef SYNA_REGISTER_MAP_H
#define SYNA_REGISTER_MAP_H 1

/*      Register Name                                                                    Address     Register Description */
/*      -------------                                                                 -------     -------------------- */

#define SYNA_F01_RMI_QUERY03                               0x00B3   /* Firmware Revision Query */

//#define SYNA_F11_2D_QUERY00                                0x008E   /* Per-device Query */

/* Start of Page Description Table (PDT) */
#define SYNA_PDT_P00_F11_2D_QUERY_BASE                     0x00DD   /* Query Base */
#define SYNA_PDT_P00_F01_RMI_QUERY_BASE                    0x00E3   /* Query Base */
#define SYNA_PDT_P00_F34_FLASH_QUERY_BASE                  0x00E9   /* Query Base */
#define SYNA_P00_PAGESELECT                                0x00FF   /* Page Select register */

/* Offsets within the configuration block */

/*      Register Name                                                                     Offset      Register Description */
/*      -------------                                                                   ------      -------------------- */
#define SYNA_F01_RMI_DEVICE_00_CFGBLK_OFS                  0x0000   /* Device status & Control */
#define SYNA_F01_RMI_INTERRUPT_01_CFGBLK_OFS               0x0001   /* Interrupt status & Enable 0 */
#define SYNA_F11_RMI_CTRL00_CFGBLK_OFS                     0x0000   /* 2D Reprot Mode */

/* Masks for interrupt sources */
#endif  /* SYNA_REGISTER_MAP_H */

#define TS_SNTS_RMI_CTRL00_DEFAULT_VALUE	0x00	// No configured, 80 Hz, Sleep Enable, Normal operation
#define TS_SNTS_RMI_CTRL00_NO_SLEEP			0x48	// No configured, 40 Hz, Sleep Disable, Normal operation
#define TS_SNTS_RMI_CTRL00_RESET_VALUE		0x00	// No configured, 80 Hz, Sleep Enable, Normal operation

#define TS_SNTS_RMI_CTRL01_DEFAULT_VALUE	0x06	// Abs0 Enable, Status Enable, Flash Disable
#define TS_SNTS_RMI_CTRL01_ONLY_ABS0		0x04	// Abs0 Enable, Status Disable, Flash Disable
#define TS_SNTS_RMI_CTRL01_ONLY_FLASH		0x01	// Abs0 Disable, Status Disable, Flash Enable
#define TS_SNTS_RMI_CTRL01_RESET_VALUE		0x07	// Abs0 Enable, Status Enable, Flash Enable

#define TS_SNTS_2D_CTRL00_DEFAULT_VALUE		0x02	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Down/Up Reporting Mode
#define TS_SNTS_2D_CTRL00_REDUCED_REPORTING	0x01	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Reduced Reporting Mode
#define TS_SNTS_2D_CTRL00_FULL_REPORTING	0x00	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Continuous Reporting Mode

//#define TS_SNTS_2D_CTRL10_DEFAULT_VALUE		0x54	// Pinch Enable, Flick Enable, Double Tap Enable
//#define TS_SNTS_2D_CTRL10_RESET_VALUE		0x7F	// All Enable

//#define TS_SNTS_2D_CTRL18_VALUE				0x03	// Minimum Flick Distance (3 * (1 mm))
//#define TS_SNTS_2D_CTRL19_VALUE				0x04	// Minimum Flick Speed (4 * (1 mm / 100 ms))

#define TS_SNTS_2D_CTRL14_VALUE				0x00	// Sensitivity Adjust

///TODO: Must Check this.
//#ifdef TS_I2C_FAIL_MANAGEMENT
#define TS_SNTS_DELAY_BETWEEN_POWER_DOWN_UP 		100	// 100ms	// For T1021
//#endif
#define TS_SNTS_DELAY_TO_LOW_ATTN_FOR_CLEARPAD		500	// 500 ms	// For T1021

//#define TS_SNTS_DEVICE_COMMAND_RESET				1	// Reset
//#define TS_SNTS_FW_VER_RETRY_CNT					10

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                         DATA DEFINITIONS                                */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

typedef struct {
	unsigned char m_QueryBase;
	unsigned char m_CommandBase;
	unsigned char m_ControlBase;
	unsigned char m_DataBase;
	unsigned char m_IntSourceCount;
	unsigned char m_FunctionExists;
} T_RMI4FuncDescriptor;

/* Ku */
typedef int TS_FINGER_STATUS_TYPE;
#define TS_SNTS_NO_PRESENT  0x0
#define TS_SNTS_PRESENT_ACCURATE  0x1


#ifdef TS_VERSION_MANAGEMENT
typedef enum {
	E_TS_RMI4_PAGE_SELECT_00 = 0x00,
} E_TS_RMI_PAGE_SELECT;

typedef enum {
	TS_SNTS_VER_MIN,
	TS_SNTS_VER_MAX
} TS_VERSION_INFO;
#endif /* TS_VERSION_MANAGEMENT */

#ifdef TS_DEBUG
typedef struct {
	int X[FINGER_MAX];
	int Y[FINGER_MAX];
	unsigned char Z[FINGER_MAX];
	unsigned char W[FINGER_MAX];
	//TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	/* Ku */
	//TOUCH_EVENT_TYPE event[FINGER_MAX];
  	// For T1021
  	/*
	unsigned char gesture_flag[2];
	unsigned char pinch_motion_and_X_flick_dist;
	unsigned char Y_flick_dist;
	unsigned char flick_time_reg;
	*/
} touch_event_debugging_data;
#endif /* TS_DEBUG */

//added by jykim
#define START_ADDR      0x13
#define GESTURE_FLAGS	0x4A

// 20101022 jh.koo@lge.com define touch registers [START_LGE]
#define DELTA_X_THRESHOLD	0x53
#define DELTA_Y_THRESHOLD	0x54
#define GESTURE_ENABLES_1	0x5B
#define GESTURE_ENABLES_2	0x5C
// 20101022 jh.koo@lge.com define touch registers [START_LGE]

#define MELT_CONTROL	0xF0
#define NO_MELT			0x00
#define MELT			0x01
#define AUTO_MELT		0x10

static int melt_mode = 0;

typedef struct
{
	unsigned char device_status_reg;            //0x13
	unsigned char interrupt_status_reg;			//0x14
	unsigned char finger_state_reg[3];			//0x15~0x17
	
	// Finger 0
	unsigned char X_high_position_finger0_reg;  //0x18
	unsigned char Y_high_position_finger0_reg;	//0x19
	unsigned char XY_low_position_finger0_reg;	//0x1A
	unsigned char XY_width_finger0_reg;			//0x1B
	unsigned char Z_finger0_reg;				//0x1C
	
	// Finger 1
	unsigned char X_high_position_finger1_reg;  //0x1D
	unsigned char Y_high_position_finger1_reg;	//0x1E
	unsigned char XY_low_position_finger1_reg;	//0x1F
	unsigned char XY_width_finger1_reg;			//0x20
	unsigned char Z_finger1_reg;				//0x21

	// Finger 2
	unsigned char X_high_position_finger2_reg;  //0x22
	unsigned char Y_high_position_finger2_reg;	//0x23
	unsigned char XY_low_position_finger2_reg;	//0x24
	unsigned char XY_width_finger2_reg;			//0x25
	unsigned char Z_finger2_reg;				//0x26

	// Finger 3
	unsigned char X_high_position_finger3_reg;  //0x27
	unsigned char Y_high_position_finger3_reg;	//0x28
	unsigned char XY_low_position_finger3_reg;	//0x29
	unsigned char XY_width_finger3_reg;			//0x2A
	unsigned char Z_finger3_reg;				//0x2B

	// Finger 4
	unsigned char X_high_position_finger4_reg;  //0x2C
	unsigned char Y_high_position_finger4_reg;	//0x2D
	unsigned char XY_low_position_finger4_reg;	//0x2F
	unsigned char XY_width_finger4_reg;			//0x30
	unsigned char Z_finger4_reg;				//0x21
} ts_sensor_data;

typedef struct {
	unsigned char finger_count;
//  TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	unsigned int X_position[FINGER_MAX];
	unsigned int Y_position[FINGER_MAX];
//  TOUCH_EVENT_TYPE Event[FINGER_MAX];
	unsigned char pressure[FINGER_MAX];
} ts_finger_data;

static ts_sensor_data ts_reg_data={0};
static ts_finger_data curr_ts_data;

typedef struct {
	//TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	unsigned int action[FINGER_MAX];
  	unsigned int finger_data[FINGER_MAX];
} TOUCH_MULTIFINGER_DATA_TYPE;

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                            External Data                                */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/* Ku */
// extern rex_crit_sect_type ts_crit_sect;
// extern rex_timer_type ts_poll_timer;
//extern clk_cb_type ts_polling_clk_cb;

#ifdef TS_HW_VER_MANAGEMENT
#ifndef FEATURE_LGE_I2C_USING_GPIO
// extern unsigned int nTS_INT_OUT;
// extern unsigned int nTS_INT_IN;
// extern unsigned int nTS_INT;
#endif
#endif /* TS_HW_VER_MANAGEMENT */

/*                            Local Data                                   */
#ifdef TS_DEBUG
touch_event_debugging_data touch_event_log[TS_DEBUG_BUFFER_SIZE];
unsigned char touch_event_cnt=0;

touch_event_debugging_data touch_report_log[TS_DEBUG_BUFFER_SIZE];
unsigned char touch_report_cnt=0;

#endif /* TS_DEBUG */

#ifdef TS_I2C_FAIL_MANAGEMENT
static unsigned int nI2CFailCnt = 0;
#endif /* TS_I2C_FAIL_MANAGEMENT */

typedef struct {
	int X_position[LONG_PRESS_RANGE];
	int Y_position[LONG_PRESS_RANGE];
	int X2_position[LONG_PRESS_RANGE];
	int Y2_position[LONG_PRESS_RANGE];
    int distance[LONG_PRESS_RANGE];
} finger_history;

static finger_history synaptics_history;

#ifdef FEATURE_LGE_DSAT_TOUCHSCREEN_EMULATION
/* Event callback function */
typedef void (*hs_touch_event_cb_type) (void * cmd);
hs_touch_event_cb_type cb_ptr = NULL;
static int ts_cmer_tscrn = 0;
static int ts_cmec_tscrn = 0;
//static BOOL ts_touchable_reporting_to_ds = FALSE;
static BOOL ts_cind_inputstatus_in_sleep = FALSE;
#endif // FEATURE_LGE_DSAT_TOUCHSCREEN_EMULATION


//ended by jykim

static int ts_pre_state = 0; /* for checking the touch state */
static int ghost_finger_1 = 0; // remove for ghost finger
static int ghost_finger_2 = 0;
static int pressed = 0;
static int ghost_count = 0;
static unsigned long pressed_time;

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int int_mode;
	int ret;
	int width0, width1, width2, width3, width4;

	int i2 = 0;
	int touch2_prestate = 0;
	int touch1_prestate = 0;

	int_mode = i2c_smbus_read_byte_data(ts->client, 0x14);

       //printk("synaptics_ts_work_func : start  int_mode=%d\n", int_mode);

	i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data.device_status_reg);


	if(int_mode & SYNAPTICS_INT_ABS0) {

		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[0]) == 1 ) 
		{
			if(pressed == 0) {
				pressed_time = jiffies;
				ghost_finger_1 = 1;
				pressed++;
			}
			touch1_prestate = 1;
			curr_ts_data.X_position[0] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
  			curr_ts_data.Y_position[0] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
			synaptics_history.X_position[i2] = curr_ts_data.X_position[0];
			synaptics_history.Y_position[i2] = curr_ts_data.Y_position[0];
			
			if ((((ts_reg_data.XY_width_finger0_reg & 240) >> 4) - (ts_reg_data.XY_width_finger0_reg & 15)) > 0)
				width0 = (ts_reg_data.XY_width_finger0_reg & 240) >> 4;
			else
				width0 = ts_reg_data.XY_width_finger0_reg & 15;

			curr_ts_data.pressure[0] = ts_reg_data.Z_finger0_reg;	

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[0]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[0]);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[0]);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width0);

			//printk("Synaptics_X, Synaptics_Y = (%d, %d), z = %d, w = %d, %d\n", curr_ts_data.X_position[0], curr_ts_data.Y_position[0], ((ts_reg_data.XY_width_finger0_reg & 240) >> 4), (ts_reg_data.XY_width_finger0_reg & 15), width0);
		
			input_mt_sync(ts->input_dev);

		}else{
				touch1_prestate = 0;
				i2 = 0;
		}
			
		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[0]) == 0) 
		{ // && touch1_prestate == 1) {

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[0]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[0]);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width0);
			input_mt_sync(ts->input_dev);
		}
		
		if(TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[0]) == 1 /*&& touch1_prestate ==1*/)
		{
			ts_pre_state = 1;
			touch2_prestate = 1;
  			curr_ts_data.X_position[1] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger1_reg, ts_reg_data.XY_low_position_finger1_reg);
  			curr_ts_data.Y_position[1] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger1_reg, ts_reg_data.XY_low_position_finger1_reg);
 			synaptics_history.X2_position[i2] = curr_ts_data.X_position[1];
			synaptics_history.Y2_position[i2] = curr_ts_data.Y_position[1];
 			synaptics_history.distance[i2] = abs(abs(synaptics_history.X2_position[i2]-synaptics_history.X_position[i2])+abs(synaptics_history.Y2_position[i2]-synaptics_history.Y_position[i2]));
			
			if ((((ts_reg_data.XY_width_finger1_reg & 240) >> 4) - (ts_reg_data.XY_width_finger1_reg & 15)) > 0)
				width1 = (ts_reg_data.XY_width_finger1_reg & 240) >> 4;
			else
				width1 = ts_reg_data.XY_width_finger1_reg & 15;

			curr_ts_data.pressure[1] = ts_reg_data.Z_finger1_reg;	
					
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[1]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[1]);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[1]);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width1);

			input_mt_sync(ts->input_dev);
		}else{
			touch2_prestate = 0;
			i2 = 0;
		}

		if(TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[0]) == 0) 
		{ // && touch1_prestate == 1) {

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[1]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[1]);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width1);

			input_mt_sync(ts->input_dev);
		}

		if(TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[0]) == 1) // && touch1_prestate ==1)
		{
  			curr_ts_data.X_position[2] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger2_reg, ts_reg_data.XY_low_position_finger2_reg);
  			curr_ts_data.Y_position[2] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger2_reg, ts_reg_data.XY_low_position_finger2_reg);

		        	if ((((ts_reg_data.XY_width_finger2_reg & 240) >> 4) - (ts_reg_data.XY_width_finger2_reg & 15)) > 0)
				width2 = (ts_reg_data.XY_width_finger2_reg & 240) >> 4;
			else
				width2 = ts_reg_data.XY_width_finger2_reg & 15;

			curr_ts_data.pressure[2] = ts_reg_data.Z_finger2_reg;	
						
       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[2]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[2]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[2]);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width2);

			input_mt_sync(ts->input_dev);
		}

		if(TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[0]) == 0)
		{ // && touch1_prestate == 1) {

	       		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[2]);
        			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[2]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width2);

			input_mt_sync(ts->input_dev);
		}
		
		if(TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[0]) == 1) // && touch1_prestate ==1)
		{
  			curr_ts_data.X_position[3] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger3_reg, ts_reg_data.XY_low_position_finger3_reg);
  			curr_ts_data.Y_position[3] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger3_reg, ts_reg_data.XY_low_position_finger3_reg);
 	
        			if ((((ts_reg_data.XY_width_finger3_reg & 240) >> 4) - (ts_reg_data.XY_width_finger3_reg & 15)) > 0)
				width3 = (ts_reg_data.XY_width_finger3_reg & 240) >> 4;
			else
				width3 = ts_reg_data.XY_width_finger3_reg & 15;

			curr_ts_data.pressure[3] = ts_reg_data.Z_finger3_reg;	
						
       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[3]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[3]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[3]);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width3);

			input_mt_sync(ts->input_dev);
		}

		if(TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[0]) == 0) 
		{ // && touch1_prestate == 1) {

       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[3]);
        			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[3]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width3);

			input_mt_sync(ts->input_dev);
		}		

		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[1]) == 1) // && touch1_prestate ==1)
		{
  			curr_ts_data.X_position[4] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger4_reg, ts_reg_data.XY_low_position_finger4_reg);
  			curr_ts_data.Y_position[4] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger4_reg, ts_reg_data.XY_low_position_finger4_reg);

        			if ((((ts_reg_data.XY_width_finger4_reg & 240) >> 4) - (ts_reg_data.XY_width_finger4_reg & 15)) > 0)
				width4 = (ts_reg_data.XY_width_finger4_reg & 240) >> 4;
			else
				width4 = ts_reg_data.XY_width_finger4_reg & 15;

			curr_ts_data.pressure[4] = ts_reg_data.Z_finger4_reg;	
						
       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[4]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[4]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[4]);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width4);

			input_mt_sync(ts->input_dev);
		}

		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[1]) == 0) 
		{ // && touch1_prestate == 1) {

       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[4]);
        			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[4]);
        			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, width4);

			input_mt_sync(ts->input_dev);
		}
		
// 20101110 jh.koo@lge.com, check and change mode for remove ghost fingers [START_LGE]

//		printk("melt mode, f1 :0x%x, f2 : 0x%x, f3 :0x%x\n ghost_count : %d\n"	, ts_reg_data.finger_state_reg[0], ts_reg_data.finger_state_reg[1], ts_reg_data.finger_state_reg[2], ghost_count);
		if(melt_mode == 0)
		{		
			if(pressed)
			{
				if( TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[0]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[0]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[0]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[1]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[1]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[1]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[1]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[2]) == 1 ||
					TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[2]) == 1 )
				{
						ghost_finger_2 = 1;			
				}
			}

			if((TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[0]) == 0) && ghost_finger_1 == 1 && ghost_finger_2 == 0 && pressed == 1) 
			{
				if(jiffies - pressed_time < 2 * HZ)
				{
					ghost_count++;
					if(ghost_count > 3) {
							ret = i2c_smbus_write_byte_data(ts->client, MELT_CONTROL, NO_MELT);
							ghost_count = 0;
							melt_mode++;
					}
					//printk("%s() mode change, ghost count : %d\n", __func__, ghost_count);
				}
				ghost_finger_1 = 0;
				pressed = 0;
			}	
		
			if( TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[0]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[0]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[0]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[0]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[1]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[1]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg[1]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[1]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[2]) == 0 &&
				TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg[2]) == 0 ) 
			{
					ghost_finger_1 = 0;
					ghost_finger_2 = 0;
					pressed = 0;
			}		
		}			
// 20101110 jh.koo@lge.com, check and change mode for remove ghost fingers [END_LGE]

		input_mt_sync(ts->input_dev);

		if (touch1_prestate ==1 && touch2_prestate == 1)
			i2 = i2+1;
			               
		input_sync(ts->input_dev);

	}

       //printk("synaptics_ts_work_func : end \n");

SYNAPTICS_TS_IDLE:
	if (ts->use_irq) 
		enable_irq(ts->client->irq);

}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	//printk("synaptics_ts_timer_func \n");  

	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL); /* 12.5 msec */

	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	//printk("synaptics_ts_irq_handler  = %x \n", dev_id);
	disable_irq_nosync(ts->client->irq);

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

	queue_work(synaptics_wq, &ts->work);

	return IRQ_HANDLED;
}


// 20110131 jh.koo@lge.com Touch Screen FW Upgrade [START_LGE]
#define ESuccess                     0
#define I2C_SUCCESS 			0
#define EErrorTimeout               -1
#define EErrorFlashNotDisabled           -2
#define EErrorBootID                -3
#define EErrorFunctionNotSupported  -4
#define EErrorFlashImageCorrupted   -5
#define I2C_FAILURE -6

void RMI4CheckIfFatalError(EError errCode)
{
	if(errCode < ESuccess)
	{
		printk("\nFatal error: ");
		printk("%d\n", errCode);
		//	    exit();
		return;
	}
}

unsigned char I2C_HAL_BitRead_ATTN()
{
	return gpio_get_value(HUB_TS_INT);
}

bool SynaWaitForATTN(int dwMilliseconds)
{
	int trialMs=0;

	while( ( (I2C_HAL_BitRead_ATTN() ==1) & (trialMs<dwMilliseconds)) )
	{
		msleep(1);
		trialMs++;
	}
	
	if(I2C_HAL_BitRead_ATTN()== 0)						
		return ESuccess;
	
	return EErrorTimeout;			
}

void SpecialCopyEndianAgnostic(unsigned char *dest, unsigned short src) 
{
	dest[0] = src%0x100;  //Endian agnostic method
	dest[1] = src/0x100;  
}


void RMI4FuncsConstructor()
{
  // Initialize data members
  m_BaseAddresses.m_QueryBase = 0xff;
  m_BaseAddresses.m_CommandBase = 0xff; 
  m_BaseAddresses.m_ControlBase = 0xff;
  m_BaseAddresses.m_DataBase = 0xff;
  m_BaseAddresses.m_IntSourceCount = 0xff;
  m_BaseAddresses.m_ID = 0xff;

  m_uQuery_Base = 0xffff;
  m_bAttenAsserted = false;
  m_bFlashProgOnStartup = false;
  m_bUnconfigured = false;
  g_ConfigDataList[0x2d - 4 - 1].m_Address = 0x0000;
  g_ConfigDataList[0x2d - 4 - 1].m_Data = 0x00;
  g_ConfigDataCount = sizeof(g_ConfigDataList) / sizeof(struct ConfigDataBlock);

  m_firmwareImgData = (unsigned char*)NULL;
  m_configImgData = (unsigned char*)NULL;

}

void RMI4Init(struct i2c_client *client)
{
	DEBUG_MSG("%s() start.\n", __func__);

  // Set up blockSize and blockCount for UI and config
  RMI4ReadConfigInfo(client);
  RMI4ReadFirmwareInfo(client);

  // Allocate arrays
  m_firmwareImgData = &FirmwareImage[0];  // new unsigned char [GetFirmwareSize()];
  m_configImgData = &ConfigImage[0];      // new unsigned char [GetConfigSize()];

    DEBUG_MSG("%s() end.\n", __func__);
} 

// Read Bootloader ID from Block Data Registers as a 'key value'
EError RMI4ReadBootloadID(struct i2c_client *client)
{
  char uData[2];
//  m_ret = SynaReadRegister(m_uF34ReflashQuery_BootID, (unsigned char *)uData, 2);
  i2c_smbus_read_i2c_block_data(client, m_uF34ReflashQuery_BootID, sizeof(uData), &uData[0]);
 
  m_BootloadID = (unsigned int)uData[0] + (unsigned int)uData[1]*0x100;

  return m_ret;
}

EError RMI4WriteBootloadID(struct i2c_client *client)
{
  unsigned char uData[2];
  SpecialCopyEndianAgnostic(uData, m_BootloadID);

//  m_ret = SynaWriteRegister(m_uF34Reflash_BlockData, uData, 2);
  m_ret = i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_BlockData, sizeof(uData), &uData[0]);

  return m_ret;
}

EError RMI4IssueEnableFlashCommand(struct i2c_client *client)
{
//  m_ret = SynaWriteRegister(m_uF34Reflash_FlashControl, (unsigned char *)&s_uF34ReflashCmd_Enable, 1);
	unsigned char read_val;
	read_val = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);
	printk("read_val : 0x%x\n\n", read_val);
  	m_ret = i2c_smbus_write_byte_data(client, m_uF34Reflash_FlashControl, s_uF34ReflashCmd_Enable);

  	read_val = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);
	printk("read_val : 0x%x\n\n", read_val);
	printk("m_uF34Reflash_FlashControl : 0x%x\n", m_uF34Reflash_FlashControl);
	
  return m_ret;
}

void RMI4WaitATTN(struct i2c_client *client)
{
  int uErrorCount = 0;
  int ret;

  // To work around the physical address error from Control Bridge
  if (SynaWaitForATTN(1000))   
  {
    RMI4CheckIfFatalError(EErrorTimeout);
  }

  do {
//    m_ret = SynaReadRegister(m_uF34Reflash_FlashControl, &m_uPageData[0], 1);
//	m_uPageData[0] = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);
	ret = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);
	m_uPageData[0] = ret;

    // To work around the physical address error from control bridge
    // The default check count value is 3. But the value is larger for erase condition
//    if((m_ret != ESuccess) && uErrorCount < 300 ) //errorCount)
//	if(m_uPageData[0] < 0 && uErrorCount < 300)
	if(ret < 0 && uErrorCount < 300) 
    {
    
//    DEBUG_MSG("%s() do while if - uErrorCount : %d.\n", __func__, uErrorCount);
	mdelay(10);
	
      uErrorCount++;
      m_uPageData[0] = 0;
      continue;
    }
	
//	DEBUG_MSG("%s() do while - uErrorCount = %d, m_uPageData[0] = %d.\n", __func__, uErrorCount, m_uPageData[0]);
	mdelay(10);

//    RMI4CheckIfFatalError(m_ret);
	RMI4CheckIfFatalError(ret);
    
    // Clear the attention assertion by reading the interrupt status register
//    m_ret = SynaReadRegister(m_PdtF01Common.m_DataBase + 1, &m_uStatus, 1);
	ret = i2c_smbus_read_byte_data(client, m_PdtF01Common.m_DataBase + 1);

//    RMI4CheckIfFatalError(m_ret);
	RMI4CheckIfFatalError(ret);

  } while( m_uPageData[0] != 0x80 && uErrorCount < 300); //errorCount); 

  DEBUG_MSG("%s() end.\n\n", __func__);

}

EError RMI4IssueFlashControlCommand(struct i2c_client *client, unsigned char *command)
{
//  m_ret = SynaWriteRegister(m_uF34Reflash_FlashControl, command, 1);
  m_ret = i2c_smbus_write_byte_data(client, m_uF34Reflash_FlashControl, *command);
  return m_ret;
}

void RMI4ProgramConfiguration(struct i2c_client *client)
{
  unsigned char uData[2];
  unsigned char *puData = m_configImgData;
  unsigned short blockNum;

  int ret;

  for(blockNum = 0; blockNum < m_configBlockCount; blockNum++)
  {
    SpecialCopyEndianAgnostic(&uData[0], blockNum);

    // Write Configuration Block Number
//    m_ret = SynaWriteRegister(m_uF34Reflash_BlockNum, &uData[0], 2);
	ret = i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_BlockNum, sizeof(uData), &uData[0]);
//	printk(" %s() - m_uF34Reflash_BlockNum : %d", m_uF34Reflash_BlockNum);
//    RMI4CheckIfFatalError(m_ret);
	RMI4CheckIfFatalError(ret);

    
    // Write Data Block
//    m_ret = SynaWriteRegister(m_uF34Reflash_BlockData, puData, m_configBlockSize);
	i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_BlockData, m_configBlockSize, puData);
//    RMI4CheckIfFatalError(m_ret);
	RMI4CheckIfFatalError(ret);
    
    puData += m_configBlockSize;
    
    // Issue Write Configuration Block command to flash command register
    m_bAttenAsserted = false;
    uData[0] = s_uF34ReflashCmd_ConfigWrite;

    m_ret = RMI4IssueFlashControlCommand(client, &uData[0]);
//    RMI4CheckIfFatalError(m_ret);

    // Wait for ATTN
    mdelay(10);
    RMI4WaitATTN(client);

  }

}


// Enable Flashing programming
void RMI4EnableFlashing(struct i2c_client *client)  
{
  unsigned char uData[2];
  int ret;
  
  int uErrorCount = 0;

  // Read bootload ID
  m_ret = RMI4ReadBootloadID(client);
  RMI4CheckIfFatalError(m_ret);

  // Write bootID to block data registers  
  m_ret = RMI4WriteBootloadID(client);
  RMI4CheckIfFatalError(m_ret);

  do {
//    m_ret = SynaReadRegister(m_uF34Reflash_FlashControl, &m_uPageData[0], 1);
//	m_uPageData[0] = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);				// 20101001
	ret =  i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);
	m_uPageData[0] = ret;


    // To deal with ASIC physic address error from cdciapi lib when device is busy and not available for read
//    if((m_ret != ESuccess) && uErrorCount < 300)
	if(ret < 0 && uErrorCount < 300)
    {

		DEBUG_MSG("%s() do while if - uErrorCount : %d.\n", __func__, uErrorCount);
	
      uErrorCount++;
      m_uPageData[0] = 0;
      continue;
    }

	DEBUG_MSG("%s() do while - uErrorCount = %d, m_uPageData[0] = %d, ret = %d.\n", __func__, uErrorCount, m_uPageData[0], ret);

    RMI4CheckIfFatalError(m_ret);

    // Clear the attention assertion by reading the interrupt status register
//    m_ret = SynaReadRegister(m_PdtF01Common.m_DataBase + 1, &m_uStatus, 1);
	m_uStatus = i2c_smbus_read_byte_data(client, m_PdtF01Common.m_DataBase + 1);

    RMI4CheckIfFatalError(m_ret);
  } while(((m_uPageData[0] & 0x0f) != 0x00) && (uErrorCount <= 300)); 

  // Issue Enable flash command
  m_ret = RMI4IssueEnableFlashCommand(client);
  
  RMI4CheckIfFatalError(m_ret);
  
  RMI4ReadPageDescriptionTable(client);
 
  // Wait for ATTN and check if flash command state is idle
  RMI4WaitATTN(client);
 
  // Read the data block 0 to determine the correctness of the image
//  m_ret = SynaReadRegister(m_uF34Reflash_FlashControl, &uData[0], 1);
  uData[0] = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);

  RMI4CheckIfFatalError(m_ret);
  
  if ( uData[0] == 0x80 ) 
  {
    printk("flash enabled");
  }
  else if ( uData[0] == 0xff )
  {
    printk("flash failed");
    RMI4CheckIfFatalError( EErrorFlashNotDisabled );
  }
  else {
    printk("flash failed");
    RMI4CheckIfFatalError( EErrorFlashNotDisabled );
  }
}

// This function gets config block count and config block size
void RMI4ReadConfigInfo(struct i2c_client *client)
{
  unsigned char uData[2];

  int ret;
  
//  m_ret = SynaReadRegister(m_uF34ReflashQuery_ConfigBlockSize, &uData[0], 2);
  ret = i2c_smbus_read_i2c_block_data(client, m_uF34ReflashQuery_ConfigBlockSize, sizeof(uData), &uData[0]);
  RMI4CheckIfFatalError(ret);
//  RMI4CheckIfFatalError(m_ret);
  
  m_configBlockSize = uData[0] | (uData[1] << 8);

//  m_ret = SynaReadRegister(m_uF34ReflashQuery_ConfigBlockCount, &uData[0], 2);
  ret = i2c_smbus_read_i2c_block_data(client, m_uF34ReflashQuery_ConfigBlockCount, sizeof(uData), &uData[0]);
  RMI4CheckIfFatalError(ret);
//  RMI4CheckIfFatalError(m_ret);
  
  m_configBlockCount = uData[0] | (uData[1] << 8);
  m_configImgSize = m_configBlockSize*m_configBlockCount;
}

void RMI4CalculateChecksum(unsigned short * data, unsigned short len, unsigned long * dataBlock)
{
  unsigned long temp = *data++;
  unsigned long sum1;
  unsigned long sum2;

  *dataBlock = 0xffffffff;

  sum1 = *dataBlock & 0xFFFF;
  sum2 = *dataBlock >> 16;

  while (len--)
  {
    sum1 += temp;    
    sum2 += sum1;    
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  *dataBlock = sum2 << 16 | sum1;
}

bool RMI4ValidateBootloadID(struct i2c_client *client, unsigned short bootloadID)
{
  printk("In RMI4ValidateBootloadID\n");
  m_ret = RMI4ReadBootloadID(client);
  RMI4CheckIfFatalError(m_ret);
  printk("Bootload ID of device: %X, input bootID: %X\n", m_BootloadID, bootloadID);

  // check bootload ID against the value found in firmware--but only for image file format version 0
  return m_firmwareImgVersion != 0 || bootloadID == m_BootloadID;
}

void RMI4ResetDevice(struct i2c_client *client)
{
  unsigned short m_uF01DeviceControl_CommandReg = m_PdtF01Common.m_CommandBase;
  unsigned char uData[1];

  int ret;

  uData[0] = 1;
//  m_ret = SynaWriteRegister(m_uF01DeviceControl_CommandReg, &uData[0], 1);
  ret = i2c_smbus_write_byte_data(client, m_uF01DeviceControl_CommandReg, uData[0]);
  RMI4CheckIfFatalError(ret);  
//  RMI4CheckIfFatalError(m_ret);
}

EError RMI4DisableFlash(struct i2c_client *client)
{
  unsigned char uData[2];
  unsigned int uErrorCount = 0;

  // Issue a reset command
  RMI4ResetDevice(client);
//  SynaSleep(200);
	msleep(200);

  // Wait for ATTN to be asserted to see if device is in idle state
  if (SynaWaitForATTN(300))
  {
    RMI4CheckIfFatalError(EErrorTimeout);
  }

  do {
//    m_ret = SynaReadRegister(m_uF34Reflash_FlashControl, &m_uPageData[0], 1);
	m_uPageData[0] = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);

    // To work around the physical address error from control bridge
    if((m_ret != ESuccess) && uErrorCount < 300)
    {
      uErrorCount++;
      m_uPageData[0] = 0;
      continue;
    }
    printk("RMI4WaitATTN after errorCount loop, uErrorCount=%d\n", uErrorCount);

  } while(((m_uPageData[0] & 0x0f) != 0x00) && (uErrorCount <= 300));

  RMI4CheckIfFatalError(m_ret);
  
  // Clear the attention assertion by reading the interrupt status register
//  m_ret = SynaReadRegister(m_PdtF01Common.m_DataBase + 1, &m_uStatus, 1);
//	i2c_smbus_read_i2c_block_data(client, m_PdtF01Common.m_DataBase + 1, 1, &m_uStatus);
	m_uStatus = i2c_smbus_read_byte_data(client, m_PdtF01Common.m_DataBase + 1);

  RMI4CheckIfFatalError(m_ret);
  
  // Read F01 Status flash prog, ensure the 6th bit is '0'
  do
  {
//    m_ret = SynaReadRegister(m_uF01RMI_DataBase, &uData[0], 1);
//	i2c_smbus_read_i2c_block_data(client, m_uF01RMI_DataBase, 1, &uData[0]);
	uData[0] = i2c_smbus_read_byte_data(client, m_uF01RMI_DataBase);

    RMI4CheckIfFatalError(m_ret);

    printk("F01 data register bit 6: 0x%x\n", uData[0]);
  } while((uData[0] & 0x40)!= 0);

  // With a new flash image the page description table could change
  RMI4ReadPageDescriptionTable(client);

  return ESuccess;
}


EError RMI4IssueEraseCommand(struct i2c_client *client, unsigned char *command)
{
  // command = 3 - erase all; command = 7 - erase config
//  m_ret = SynaWriteRegister(m_uF34Reflash_FlashControl, command, 1);
  i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_FlashControl, sizeof(command), command);

  return m_ret;
}

EError RMI4FlashFirmwareWrite(struct i2c_client *client)
{
  unsigned char *puFirmwareData = m_firmwareImgData;
  unsigned char uData[2];
  
  unsigned short uBlockNum;

  int ret;

  printk("Flash Firmware starts\n");
  
  for(uBlockNum = 0; uBlockNum < m_firmwareBlockCount; ++uBlockNum)
  {
    uData[0] = uBlockNum & 0xff;
    uData[1] = (uBlockNum & 0xff00) >> 8;
    
    // Write Block Number
//    m_ret = SynaWriteRegister(m_uF34Reflash_BlockNum, &uData[0], 2);
	ret = i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_BlockNum, sizeof(uData), &uData[0]);
	RMI4CheckIfFatalError(ret);
//    RMI4CheckIfFatalError(m_ret);

    // Write Data Block
//    m_ret = SynaWriteRegister(m_uF34Reflash_BlockData, puFirmwareData, m_firmwareBlockSize);
	ret = i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_BlockData, m_firmwareBlockSize, puFirmwareData);
	RMI4CheckIfFatalError(ret);
//    RMI4CheckIfFatalError(m_ret);

    // Move to next data block
    puFirmwareData += m_firmwareBlockSize;

    // Issue Write Firmware Block command
    m_bAttenAsserted = false;
    uData[0] = 2;
//    m_ret = SynaWriteRegister(m_uF34Reflash_FlashControl, &uData[0], 1);
//	i2c_smbus_write_i2c_block_data(client, m_uF34Reflash_FlashControl, 1, &uData[0]);
	ret = i2c_smbus_write_byte_data(client, m_uF34Reflash_FlashControl, uData[0]);
	RMI4CheckIfFatalError(ret);
//    RMI4CheckIfFatalError(m_ret);

    // Wait ATTN. Read Flash Command register and check error
	mdelay(10);
    RMI4WaitATTN(client);
  }

  printk("Flash Firmware done\n");
  
  return ESuccess;
}


void RMI4ProgramFirmware(struct i2c_client *client)
{
  EError m_ret;
  unsigned char uData[1];

  if ( !RMI4ValidateBootloadID(client, m_bootloadImgID) )
  {
    RMI4CheckIfFatalError( EErrorBootID );
  }

  // Write bootID to data block register
  m_ret = RMI4WriteBootloadID(client);
  RMI4CheckIfFatalError(m_ret);

  // Issue the firmware and configuration erase command
  uData[0]=3;
  m_ret = RMI4IssueEraseCommand(client, &uData[0]);
  RMI4CheckIfFatalError(m_ret);

  DEBUG_MSG("%s() before RMI4WaitATTN!!\n", __func__);
  RMI4WaitATTN(client);

  // Write firmware image
  m_ret = RMI4FlashFirmwareWrite(client);
  RMI4CheckIfFatalError(m_ret);

}

// This function gets the firmware block size and block count
void RMI4ReadFirmwareInfo(struct i2c_client *client)
{
  unsigned char uData[2];

//  m_ret = SynaReadRegister(m_uF34ReflashQuery_FirmwareBlockSize, &uData[0], 2);
  i2c_smbus_read_i2c_block_data(client, m_uF34ReflashQuery_FirmwareBlockSize, sizeof(uData), &uData[0]);
  RMI4CheckIfFatalError(m_ret);

  m_firmwareBlockSize = uData[0] | (uData[1] << 8);

//  m_ret = SynaReadRegister(m_uF34ReflashQuery_FirmwareBlockCount, &uData[0], 2);
  i2c_smbus_read_i2c_block_data(client, m_uF34ReflashQuery_FirmwareBlockCount, sizeof(uData), &uData[0]);
  RMI4CheckIfFatalError(m_ret);

  m_firmwareBlockCount = uData[0] | (uData[1] << 8);
  m_firmwareImgSize = m_firmwareBlockCount*m_firmwareBlockSize;
  
  printk("m_firmwareBlockSize, m_firmwareBlockCount: %d, %d\n", m_firmwareBlockSize, m_firmwareBlockCount);
}


// static RMI4FunctionDescriptor reg_Buffer={0};


void RMI4ReadPageDescriptionTable(struct i2c_client *client) 
{
	struct RMI4FunctionDescriptor Buffer;
	unsigned short uAddress;

  // Read config data
//  SynaSleep(20);
	msleep(20);
//  struct RMI4FunctionDescriptor Buffer;

  m_PdtF01Common.m_ID = 0;
  m_PdtF34Flash.m_ID = 0;
  m_BaseAddresses.m_ID = 0xff;

//  unsigned short uAddress;

  for(uAddress = 0xe9; uAddress > 10; uAddress -= sizeof(struct RMI4FunctionDescriptor))
  {
//    m_ret = SynaReadRegister(uAddress, (unsigned char*)&Buffer, sizeof(Buffer));
	i2c_smbus_read_i2c_block_data(client, uAddress, sizeof(Buffer), &Buffer.m_QueryBase);
    RMI4CheckIfFatalError(m_ret);

    if(m_BaseAddresses.m_ID == 0xff)
    {
      m_BaseAddresses = Buffer;
    }

    switch(Buffer.m_ID)
    {
      case 0x34:
        m_PdtF34Flash = Buffer;
        break;
      case 0x01:
        m_PdtF01Common = Buffer;
        break;
    }

    if(Buffer.m_ID == 0)
    {
      break;
    }
    else
    {
      printk("Function $%02x found.\n", Buffer.m_ID); 
    }
  }

  // Initialize device related data members
  m_uF01RMI_DataBase = m_PdtF01Common.m_DataBase;
  m_uF01RMI_IntStatus = m_PdtF01Common.m_DataBase + 1;
  m_uF01RMI_CommandBase = m_PdtF01Common.m_CommandBase;
  m_uF01RMI_QueryBase = m_PdtF01Common.m_QueryBase;

  m_uF34Reflash_DataReg = m_PdtF34Flash.m_DataBase;
  m_uF34Reflash_BlockNum = m_PdtF34Flash.m_DataBase;
  m_uF34Reflash_BlockData = m_PdtF34Flash.m_DataBase + 2;
  m_uF34ReflashQuery_BootID = m_PdtF34Flash.m_QueryBase;

  m_uF34ReflashQuery_FlashPropertyQuery = m_PdtF34Flash.m_QueryBase + 2;
  m_uF34ReflashQuery_FirmwareBlockSize = m_PdtF34Flash.m_QueryBase + 3;
  m_uF34ReflashQuery_FirmwareBlockCount = m_PdtF34Flash.m_QueryBase + 5;
  m_uF34ReflashQuery_ConfigBlockSize = m_PdtF34Flash.m_QueryBase + 3;
  m_uF34ReflashQuery_ConfigBlockCount = m_PdtF34Flash.m_QueryBase + 7;
  
  RMI4setFlashAddrForDifFormat(client);
}

void RMI4WritePage(struct i2c_client *client)
{
	unsigned char m_uStatus;

  // Write page
  unsigned char uPage = 0x00;
  unsigned char uF01_RMI_Data[2];

//  m_ret = SynaWriteRegister(0xff, &uPage, 1);
  m_ret = i2c_smbus_write_byte_data(client, 0xff, uPage);
  RMI4CheckIfFatalError(m_ret);

//  unsigned char m_uStatus;
  do
  {
//    m_ret = SynaReadRegister(0, &m_uStatus, 1);
	m_uStatus = i2c_smbus_read_byte_data(client, 0);
    RMI4CheckIfFatalError(m_ret);

    if(m_uStatus & 0x40)
    {
      m_bFlashProgOnStartup = true;
    }

    if(m_uStatus & 0x80)
    {
      m_bUnconfigured = true;
      break;
    }

    printk("m_uStatus is 0x%x\n", m_uStatus);
  } while(m_uStatus & 0x40);

  if(m_bFlashProgOnStartup && ! m_bUnconfigured)
  {
    printk("Bootloader running\n");
  }
  else if(m_bUnconfigured)
  {
    printk("UI running\n");
  }

  RMI4ReadPageDescriptionTable(client);
  
  if(m_PdtF34Flash.m_ID == 0)
  {
    printk("Function $34 is not supported\n");
    RMI4CheckIfFatalError( EErrorFunctionNotSupported );   
  }

  printk("Function $34 addresses Control base:$%02x Query base: $%02x.\n", m_PdtF34Flash.m_ControlBase, m_PdtF34Flash.m_QueryBase);
  
  if(m_PdtF01Common.m_ID == 0)
  {
    printk("Function $01 is not supported\n");
    m_PdtF01Common.m_ID = 0x01;
    m_PdtF01Common.m_DataBase = 0;
    RMI4CheckIfFatalError( EErrorFunctionNotSupported );    
  }
  printk("Function $01 addresses Control base:$%02x Query base: $%02x.\n", m_PdtF01Common.m_ControlBase, m_PdtF01Common.m_QueryBase);

  // Get device status
//  m_ret = SynaReadRegister(m_PdtF01Common.m_DataBase, &uF01_RMI_Data[0], sizeof(uF01_RMI_Data));
  i2c_smbus_read_i2c_block_data(client, m_PdtF01Common.m_DataBase, sizeof(uF01_RMI_Data), &uF01_RMI_Data[0]);
  RMI4CheckIfFatalError(m_ret);

  // Check Device Status
  printk("Configured: %s\n", uF01_RMI_Data[0] & 0x80 ? "false" : "true");
  printk("FlashProg:  %s\n", uF01_RMI_Data[0] & 0x40 ? "true" : "false");
  printk("StatusCode: 0x%x \n", uF01_RMI_Data[0] & 0x0f );
}

void RMI4RMIInit(struct i2c_client *client, EProtocol pt, unsigned char i2cAddr, EAttention eAttn, unsigned int byteDelay,
                 unsigned int bitRate, unsigned long timeOut)
{
   switch (pt)
  {
    case Ei2c:       //i2c
      RMI4RMIInitI2C(client, i2cAddr, eAttn);	  
      break;
    case Espi:       //spi
      RMI4RMIInitSPI(client, byteDelay, bitRate, timeOut);	  
      break;
  }
}

void RMI4RMIInitI2C(struct i2c_client *client, unsigned char i2cAddr, EAttention i2cAttn)
{
  RMI4WritePage(client);
}

void RMI4RMIInitSPI(struct i2c_client *client, unsigned int byteDelay, unsigned int bitRate, unsigned long timeOut)
{ 
  RMI4WritePage(client);
}

bool RMI4isExpectedRegFormat(struct i2c_client *client)
{ 
  // Flash Properties query 1: registration map format version 1     
  //  0: registration map format version 0
//  m_ret = SynaReadRegister(m_uF34ReflashQuery_FlashPropertyQuery, &m_uPageData[0], 1);
  m_uPageData[0] = i2c_smbus_read_byte_data(client, m_uF34ReflashQuery_FlashPropertyQuery);
  RMI4CheckIfFatalError(m_ret);

  printk("FlashPropertyQuery = 0x%x\n", m_uPageData[0]);
  return ((m_uPageData[0] & 0x01) == 0x01);
}

unsigned long ExtractLongFromHeader(const unsigned char* SynaImage)  // Endian agnostic 
{
  return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}

unsigned short GetConfigSize()
{ 
  return m_configBlockSize*m_configBlockCount;
}

unsigned short GetFirmwareSize() 
{
  return m_firmwareBlockSize*m_firmwareBlockCount;
}

void RMI4ReadFirmwareHeader(struct i2c_client *client)
{
  unsigned long checkSumCode;
  
  m_fileSize = sizeof(SynaFirmware) -1;

  printk("\nScanning SynaFirmware[], the auto-generated C Header File - len = %d \n\n", m_fileSize);

  checkSumCode         = ExtractLongFromHeader(&(SynaFirmware[0]));
  m_bootloadImgID      = (unsigned int)SynaFirmware[4] + (unsigned int)SynaFirmware[5]*0x100;
  m_firmwareImgVersion = SynaFirmware[7]; 
  m_firmwareImgSize    = ExtractLongFromHeader(&(SynaFirmware[8]));
  m_configImgSize      = ExtractLongFromHeader(&(SynaFirmware[12]));    

  printk("Target = %s, ",&SynaFirmware[16]);
  printk("Cksum = 0x%X, Id = 0x%X, Ver = %d, FwSize = 0x%X, ConfigSize = 0x%X \n",
    checkSumCode, m_bootloadImgID, m_firmwareImgVersion, m_firmwareImgSize, m_configImgSize);
  
  RMI4ReadFirmwareInfo(client);   // Determine firmware organization - read firmware block size and firmware size

  RMI4CalculateChecksum((unsigned short*)&(SynaFirmware[4]), (unsigned short)(m_fileSize-4)>>1,
                        &m_FirmwareImgFile_checkSum);

  if (m_fileSize != (0x100+m_firmwareImgSize+m_configImgSize))
  {
    printk("Error: SynaFirmware[] size = 0x%X, expected 0x%X\n", m_fileSize, (0x100+m_firmwareImgSize+m_configImgSize));
    while(1);   
  }

  if (m_firmwareImgSize != GetFirmwareSize())
  {
    printk("Firmware image size verfication failed, size in image 0x%X did not match device size 0x%X\n", m_firmwareImgSize, GetFirmwareSize());
    while(1);
  }
    
  if (m_configImgSize != GetConfigSize())
  {
    printk("Configuration size verfication failed, size in image 0x%X did not match device size 0x%X\n", m_configImgSize, GetConfigSize());
    while(1); 
  }

  m_firmwareImgData=(unsigned char *)((&SynaFirmware[0])+0x100);

  memcpy(m_configImgData,   (&SynaFirmware[0])+0x100+m_firmwareImgSize, m_configImgSize);

//  SynaReadRegister(m_uF34Reflash_FlashControl, &m_uPageData[0], 1);
	m_uPageData[0] = i2c_smbus_read_byte_data(client, m_uF34Reflash_FlashControl);

  return;
}

void RMI4setFlashAddrForDifFormat(struct i2c_client *client)
{
  if (RMI4isExpectedRegFormat(client))
  {
    printk("Image format 1\n");
    m_uF34Reflash_FlashControl = m_PdtF34Flash.m_DataBase + m_firmwareBlockSize + 2;
    m_uF34Reflash_BlockNum = m_PdtF34Flash.m_DataBase;
    m_uF34Reflash_BlockData = m_PdtF34Flash.m_DataBase + 2;
  }
  else {
    m_uF34Reflash_FlashControl = m_PdtF34Flash.m_DataBase;
    m_uF34Reflash_BlockNum = m_PdtF34Flash.m_DataBase + 1;
    m_uF34Reflash_BlockData = m_PdtF34Flash.m_DataBase + 3;
  }
}

static void Synaptics_FW_upgrade(struct i2c_client *client)
{
	EProtocol protocolType = Ei2c; 
	EAttention attn = EAttnHighAndLow;
//	int i2c_address = 0x20; 		  // Default slave address	// for tm1228  
	unsigned int byte_delay = 100;	// Default byte delay
	unsigned int bit_rate = 2000; 	// Default bit rate
	unsigned int time_out = 3000; 	// Default spi mode


	RMI4FuncsConstructor();

	RMI4RMIInit(client, protocolType, (unsigned char)I2CAddr7Bit, attn, byte_delay, bit_rate, time_out);
	RMI4Init(client);

	RMI4setFlashAddrForDifFormat(client);
	RMI4EnableFlashing(client);

	RMI4ReadFirmwareHeader(client);
	RMI4ProgramFirmware(client);	//	issues the "eraseAll" so must call before any write
	RMI4ProgramConfiguration(client); 

	if (RMI4DisableFlash(client)!= ESuccess ) 	  
	{ 
//	  EndControlBridge();
		return;
	}		 
	printk("\nOutput: Reflash successful\n");

}
// 20110131 jh.koo@lge.com Touch Screen FW Upgrade [END_LGE]


int check_FW(struct i2c_client *client)
{

	int fw_rev;
	fw_rev = i2c_smbus_read_byte_data(client, 0xAB);

	printk("[!] %s() - FW rev. : %d\n", __func__, fw_rev);

	return fw_rev;
}


/*************************************************************************************************
 * 1. Set interrupt configuration
 * 2. Disable interrupt
 * 3. Power up
 * 4. Read RMI Version
 * 5. Read Firmware version & Upgrade firmware automatically
 * 6. Read Data To Initialization Touch IC
 * 7. Set some register
 * 8. Enable interrupt
*************************************************************************************************/
static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;

	int cur_fw_rev, up_fw_rev;
	
	struct synaptics_i2c_rmi_platform_data *pdata;
	unsigned long irqflags;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;
	uint32_t panel_version;

	//printk("%s() -- start\n\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	if (pdata) {
		while (pdata->version > panel_version)
			pdata++;
		ts->flags = pdata->flags;
		ts->sensitivity_adjust = pdata->sensitivity_adjust;
		irqflags = pdata->irqflags;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		irqflags = 0;
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}

  	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
  	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));

	/*************************************************************************************************
	 * 3. Power up
	 *************************************************************************************************/
//	Touch_reboot();		//added by jykim
//    gpio_set_value(BLUE_LED_GPIO, 0);
//	udelay(300);

	/*************************************************************************************************
	 * 4. Read RMI Version
	 * To distinguish T1021 and T1007. Select RMI Version
	 * TODO: Power를 이전에 하는 것으로 변경하면 위치 변경해야 한다.
	 *************************************************************************************************/
//	ret = i2c_smbus_write_byte_data(ts->client, 0x9B, 32);
//	ret = i2c_smbus_write_byte_data(ts->client, 0x9C, 32);

//	max_low = (unsigned char)i2c_smbus_read_byte_data(ts->client, 0x58);
//	max_high = (unsigned char)i2c_smbus_read_byte_data(ts->client, 0x59); 

//	max_x = max_high <<8
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "hub_synaptics_touch";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
   	set_bit(KEY_BACK, ts->input_dev->keybit);
//   	set_bit(EV_TG, ts->input_dev->evbit);
    	
   	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
	snap_left_on = snap_left_on * max_x / 0x10000;
	snap_left_off = snap_left_off * max_x / 0x10000;
	snap_right_on = snap_right_on * max_x / 0x10000;
	snap_right_off = snap_right_off * max_x / 0x10000;
	snap_top_on = snap_top_on * max_y / 0x10000;
	snap_top_off = snap_top_off * max_y / 0x10000;
	snap_bottom_on = snap_bottom_on * max_y / 0x10000;
	snap_bottom_off = snap_bottom_off * max_y / 0x10000;
	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;
	ts->snap_down[!!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_left;
	ts->snap_up[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x + inactive_area_right;
	ts->snap_down[!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_top;
	ts->snap_up[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y + inactive_area_bottom;
	ts->snap_down_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_on;
	ts->snap_down_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_off;
	ts->snap_up_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_on;
	ts->snap_up_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_off;
	ts->snap_down_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_on;
	ts->snap_down_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_off;
	ts->snap_up_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_on;
	ts->snap_up_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_off;

	printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	printk(KERN_INFO "synaptics_ts_probe: snap_x %d-%d %d-%d, snap_y %d-%d %d-%d\n",
	       snap_left_on, snap_left_off, snap_right_on, snap_right_off,
	       snap_top_on, snap_top_off, snap_bottom_on, snap_bottom_off);

	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 986, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1644, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
//	input_set_abs_params(ts->input_dev, SYN_TG_REPORT, 0, 3, 0, 0);
//	input_set_abs_params(ts->input_dev, TG_DIR, 0, 3, 0, 0);
//	input_set_abs_params(ts->input_dev, TG_SPEED, 0, 1681, 0, 0);


	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	//printk("########## irq [%d], irqflags[0x%x]\n", client->irq, irqflags);
	
	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, irqflags, client->name, ts);
//		if (ret == 0) {
			//ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */
			/* Enable ABS0 and Button Interrupt */
//			ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_INT_REG, SYNAPTICS_INT_ABS0|SYNAPTICS_INT_BUTTON);
//			if (ret)
//				free_irq(client->irq, ts);
//		}
		if (ret == 0) {
			ts->use_irq = 1;
			DEBUG_MSG("request_irq\n");
			}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	// Firmware upgrade check!!!
	cur_fw_rev = check_FW(ts->client);
	up_fw_rev = SynaFirmware[0x1f];

	// Firmware Upgrade if current fw version is lower version
	if((cur_fw_rev > 0) && (cur_fw_rev < up_fw_rev))
		Synaptics_FW_upgrade(ts->client);


	i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data.device_status_reg);

	if(ts_reg_data.interrupt_status_reg == 6)
	{
		printk("%s() - ts_reg_data.interrupt_status_reg : %d\n", __func__, ts_reg_data.interrupt_status_reg);
		queue_work(synaptics_wq, &ts->work);
	}
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP);

	ret = i2c_smbus_write_byte_data(ts->client, DELTA_X_THRESHOLD, 0x02);
	ret = i2c_smbus_write_byte_data(ts->client, DELTA_Y_THRESHOLD, 0x02);

	ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLES_1, 0x00);
	ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLES_2, 0x00);

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

/*20110310 seven.kim@lge.com to prevent IRQ during soft reset [START] */
void synaptics_ts_disable_irq()
{
// nus	if (lcd_off_boot == 0) 
// nus	      disable_irq(hub_ts_client->irq);
}
EXPORT_SYMBOL(synaptics_ts_disable_irq);
/*20110310 seven.kim@lge.com to prevent IRQ during soft reset [START] */

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
    
//  gpio_set_value(BLUE_LED_GPIO, 1);
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_SLEEP); /* sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power off failed\n");
	}

	melt_mode = 0;
	ghost_finger_1 = 0;
	ghost_finger_2 = 0;
	pressed = 0;	
//	ghost_count = 0;
	
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	
// 20100525 jh.koo@lge.com Turn off touch LDOs [START_LGE]
	//printk(KERN_WARNING"[!] %s() i2c start.\n\n", __func__);

	melt_mode = 0;
	ghost_finger_1 = 0;
	ghost_finger_2 = 0;
	pressed = 0;	
	ghost_count = 0;

	i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data.device_status_reg);
   	i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP); /* wake up */

	ret = i2c_smbus_write_byte_data(ts->client, DELTA_X_THRESHOLD, 0x02);
	ret = i2c_smbus_write_byte_data(ts->client, DELTA_Y_THRESHOLD, 0x02);
	
	ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLES_1, 0x00);
	ret = i2c_smbus_write_byte_data(ts->client, GESTURE_ENABLES_2, 0x00);

	ret = i2c_smbus_write_byte_data(ts->client, MELT_CONTROL, MELT);
// 20100525 jh.koo@lge.com Turn off touch LDOs [END_LGE]

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	check_FW(client);
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "hub_synaptics_ts", 0 },
// 20100525 sookyoung.kim@lge.com [START_LGE]
	{ },
	//{ }
// 20100525 sookyoung.kim@lge.com [END_LGE]
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
// 20100525 sookyoung.kim@lge.com [START_LGE]
		//.name	= "hub_i2c_ts",
		.name	= "hub_synaptics_ts",
// 20100525 sookyoung.kim@lge.com [END_LGE]
		.owner = THIS_MODULE,
	},
};

static int __devinit synaptics_ts_init(void)
{
//--[[ LGE_UBIQUIX_MODIFIED_START : usnoh@ubiquix.com [2011.07.03] - Improve the performance of touch driver
#if 0 
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
//	synaptics_wq = __create_workqueue("synaptics_wq", 1, 0, 1);
#else
	synaptics_wq = create_rt_workqueue("synaptics_wq");
#endif
//--]] LGE_UBIQUIX_MODIFIED_END : usnoh@ubiquix.com [2011.07.03]- Improve the performance of touch driver
	if (!synaptics_wq)
		return -ENOMEM;

	//printk ("LGE: Synaptics ts_init \n");
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
    
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");


