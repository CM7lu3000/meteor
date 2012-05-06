/*
 *  kxtf9_if.c
 *
 *  KXTF9 Accelerometer Interface   
 * 
 *  Copyright (C) 2010 LGE Inc.
 *
 *  2010/02/15 : created by <chanhee.park@lge.com>
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>

#include "kionix_accel_api.h"
#include "kxtf9_if.h"


/*--------------------------------------------------------------------------------------------------------------------
	LGE Interface 
   --------------------------------------------------------------------------------------------------------------------*/	

#define  MAX_KXTF9_SAMPLE_RATE_INDEX	 7
#define DEBUG   0
#define NO_COMPASS_SUPPORT  1
int flip_pre_state = 0;
int hub_proximity_check(void);
int screen_rotation_pre_state = 0; //20101108 taehwan.kim@lge.com For flip tunning

accel_send_tap_cb_fn_type	 accel_send_tap_cb_fn = NULL;
accel_send_shake_cb_fn_type	 accel_send_shake_cb_fn = NULL;
accel_send_flip_cb_fn_type	 accel_send_flip_cb_fn = NULL;
accel_send_snap_cb_fn_type	 accel_send_snap_cb_fn = NULL;

shake_data	    			 shakeData;
long			    		 shakeCount;

int 			  			 kxtf9_data_lpf_buf[MAX_KXTF9_SAMPLE_RATE_INDEX] = {6,12,25,50,100,200,400};


/** @brief  To set flip previous state as 0x02 (LCD to ground) 
 *  *  *   @author taehwan.kim@lge.com
 *   *   *     @date   2010.12.16
 *    *    *       */
void kxtf9_flip_status_initialized (void)
{
    screen_rotation_pre_state = 0x02; //20101108 taehwan.kim@lge.com For tunning flip event
}

/*---------------------------------------------------------------------------
	kxtf9_accel_service_flip_vibrator
   ---------------------------------------------------------------------------*/		
int  kxtf9_accel_service_flip(int vibrator_duration)
{
    printk("antispoon flip \n");
	KIONIX_ACCEL_service_flip(vibrator_duration);

	return 0;
}

/*---------------------------------------------------------------------------
	kxtf9_accel_service_isr
   ---------------------------------------------------------------------------*/		
int  kxtf9_accel_service_isr(int vib_time)
{
//--[[ LGE_UBIQUIX_MODIFIED_START : shyun@ubiquix.com [2011.07.07] - Unnecessary log removed
//    printk("antispoon isr \n");
//--]] LGE_UBIQUIX_MODIFIED_END : shyun@ubiquix.com [2011.07.07]- Unnecessary log removed
	KIONIX_ACCEL_service_interrupt(vib_time);

	return 0;
}
/*---------------------------------------------------------------------------
	kxtf9_accel_register_callback
 ---------------------------------------------------------------------------*/
void kxtf9_accel_register_i2c_func(void)
{
	KIONIX_ACCEL_i2c_register_cb();
}
void kxtf9_accel_tap_register_callback(void (*func_ptr)(int type,int direction))
{
	 accel_send_tap_cb_fn = func_ptr;
}
void kxtf9_accel_shake_register_callback(void (*func_ptr)(int value))
{
	 accel_send_shake_cb_fn = func_ptr;
}
void kxtf9_accel_flip_register_callback(void (*func_ptr)(int value))
{
	 accel_send_flip_cb_fn = func_ptr;
}
void kxtf9_accel_snap_register_callback(void (*func_ptr)(int value))
{
	 accel_send_snap_cb_fn = func_ptr;
}

/*---------------------------------------------------------------------------
	kxtf9_accel_send_event
   ---------------------------------------------------------------------------*/		
void kxtf9_accel_send_tap_event(int type,int direction)
{	
	if(accel_send_tap_cb_fn)
		accel_send_tap_cb_fn(type,direction);
}
void kxtf9_accel_send_shake_event(int value)
{
	if(accel_send_shake_cb_fn)
		accel_send_shake_cb_fn(value);
}
void kxtf9_accel_send_screen_rotation_event(int value, int pre_value)
{
	int status=0;
	int flip_data=0,snap_data=0;
    int proxi_state;
    unsigned char tilt_pos_cur;

	status = kxtf9_enable_gesture_status();
#if DEBUG
    printk("kxtf9_enable_gesture_value = %x, pre_value = %x ,flip_pre_state= %x\n"
            ,value, screen_rotation_pre_state, flip_pre_state);
#endif //DEBUG
#if defined(CONFIG_MACH_LGE_HUB_REV_A) //NO_COMPASS_SUPPORT
    if (system_rev == 1) {
        if (value == 0x04) accel_send_snap_cb_fn(99);
        if (value == 0x20) accel_send_snap_cb_fn(98);
    }
#endif
   if(status & KXTF9_FLIP_ENABLE)
   {
       proxi_state = hub_proximity_check();
       printk("[antispoon] proxi_state = %d pre_st = %d \n",proxi_state, screen_rotation_pre_state);
       if ((screen_rotation_pre_state != 0x02)// && (pre_value != 0x02) 
               && (value == 0x02) && (proxi_state/*hub_proximity_check()*/ == 0)) 
           flip_data= ACCEL_FLIP_UPSIDE_DOWN;
       else if ((value != 0x02) &&(flip_pre_state == ACCEL_FLIP_UPSIDE_DOWN)) 
           flip_data= ACCEL_FLIP_DOWNSIDE_UP;
       
       if((flip_data== ACCEL_FLIP_UPSIDE_DOWN)||(flip_data==ACCEL_FLIP_DOWNSIDE_UP))
       {
           printk("[antispoon] send FLIP event (%d) pre_st: %d, cur_st: %d\n",flip_data, screen_rotation_pre_state, value);
           if(accel_send_flip_cb_fn)
               accel_send_flip_cb_fn(flip_data);
           flip_pre_state = flip_data;
       }
   }
   if ((screen_rotation_pre_state != value) &&(value != 0x02) && (proxi_state != 0)) screen_rotation_pre_state = value; 
   else if ((screen_rotation_pre_state != value) && (value == 0x02) && (proxi_state == 0)) screen_rotation_pre_state = value;

   printk("[antispoon] screen_rotation_pre_state = %d \n",screen_rotation_pre_state); 
   if(status & KXTF9_SNAP_ENABLE)
   {
       if(value == 0x02) //CTRL_REG2_FDM
           snap_data = ACCEL_SNAP_PICTH_UP;
       else if(value == 0x08) //CTRL_REG2_DOM
       {
           if(!(status & KXTF9_FLIP_ENABLE))
               snap_data = ACCEL_SNAP_ROLL_RIGHT;	
       }
       
       if((snap_data== ACCEL_SNAP_PICTH_UP)||(snap_data == ACCEL_SNAP_ROLL_RIGHT))
       {
           if(accel_send_snap_cb_fn) 
               accel_send_snap_cb_fn(snap_data);
       }
   }
}

/****************  kxtf9_accel_init ********************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_init - Accel. raw, tilt,shake,flip,tapping
   ---------------------------------------------------------------------------*/		
int  kxtf9_accel_init(void)
{
	unsigned char value,dummy;
	int res=0;
	static init = 0;

	if (!init) {
		/* Need 50ms until the RAM load is finished after Power-up	*/
		msleep(200);

		KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_WHO_AM_I, &value, 1);
		//printk("[%s:%d] [MOTION] Device ID (0x%02x) \n", __FUNCTION__, __LINE__, value);
		KIONIX_ACCEL_reset();

		msleep(200);
		KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_WHO_AM_I, &value, 1);
		//printk("[%s:%d] [MOTION] Device ID 2nd try (0x%02x) \n", __FUNCTION__, __LINE__, value);
		init = 1;
	}

	//KIONIX_ACCEL_disable_outputs();
	//KIONIX_ACCEL_disable_interrupt();
	//KIONIX_ACCEL_disable_all();
	
	res |= KXTF9_set_G_range(2);
	res |= KXTF9_set_resolution(12);

    /*---------------- tilt & shake -------------------*/
	res |= KXTF9_set_hpf_odr(50);
	res |= KXTF9_set_lpf_odr(50); //20101026 taehwan.kim@lge.com Tunning for sampling rate to maximum.

    /*--------------flip - screen rotation --------------*/
	res |=KXTF9_set_odr_tilt(12);   //12.5 Hz
#if defined(CONFIG_MACH_LGE_HUB_REV_A) //NO_COMPASS_SUPPORT    //Disable Snap function (Hub don't use snap function"
	res |= KIONIX_ACCEL_position_mask_fu();
	res |= KIONIX_ACCEL_position_mask_fd();   // snap  --  snap_pitch_up
	res |= KIONIX_ACCEL_position_mask_ri();
	res |= KIONIX_ACCEL_position_mask_le();
#endif //defined(CONFIG_MACH_LGE_HUB_REV_A) //NO_COMPASS_SUPPORT
	res |= KIONIX_ACCEL_position_mask_up();   // flip
 	res |= KIONIX_ACCEL_position_mask_do();   // flip --- snap_roll_right
	//res |= KIONIX_ACCEL_tilt_timer(5); //20101026 taehwan.kim@lge.com Tunning for Flip event
	res |= KIONIX_ACCEL_enable_tilt_function(); //antispoon 0326
	/*---------------- Tapping  -----------------------*/
    res |= KXTF9_set_odr_tap(200); /*LGE_CHAGE_S_E [taehwan.kim@lge.com] 2010-04-01 Tunning data rate*/
    res |= KXTF9_tap_mask_TFU();
    res |= KXTF9_tap_mask_TFD();
    res |= KXTF9_tap_mask_TUP();
    res |= KXTF9_tap_mask_TDO();
    res |= KXTF9_tap_mask_TRI();
    res |= KXTF9_tap_mask_TLE();
     
	kxtf9_accel_enable_tap();
	  
    /*----------- shake init. value ---------------------*/
	shakeCount = 0; 
	KIONIX_SHAKE_Init(&shakeData); 

      /*------------ Interrupt ---------------------------*/
	res |= KIONIX_ACCEL_int_activeh();        
	res |= KIONIX_ACCEL_int_latch();

	KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_INT_REL, &dummy, 1);

	KIONIX_ACCEL_enable_interrupt();
	res |= KIONIX_ACCEL_enable_outputs();

    screen_rotation_pre_state = 0x02; //20101108 taehwan.kim@lge.com For tunning flip event
    printk("[antispoon] reset screen_rotation_pre_state \n");

    return res;
	
}

void kxtf9_accel_disable(void)
{
	KIONIX_ACCEL_disable_outputs();
	KIONIX_ACCEL_disable_interrupt();
	KIONIX_ACCEL_disable_all();
}

/**********************    TAP   ********************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_enable_tap
   ---------------------------------------------------------------------------*/		
static unsigned char old_osc;
int kxtf9_accel_enable_tap(void)
{
	int res = 0;
	unsigned char osc, old_vco_trim, new_vco_trim;
    unsigned char x_gain, y_gain, z_gain;
    unsigned char dummy;
		
    res |= KXTF9_set_odr_tap(400); // 400Hz

    KIONIX_ACCEL_read_bytes(0x59, &old_vco_trim, 1);
#if 1 // 600Hz
//    new_vco_trim = 128+((old_vco_trim-128)+((old_vco_trim-128)>>2)+11) ;    // OSCILATTOR_TRIM
    //printk("------------------------------> YJ : old_vco_trim = 0x%x  new_vco_trim = 0x%x\n", old_vco_trim, new_vco_trim);
//    KIONIX_ACCEL_write_byte(0x59, (unsigned char)new_vco_trim);

    KIONIX_ACCEL_write_byte(0x2B, 0x78);    // TDT_TIMER
    KIONIX_ACCEL_write_byte(0x2E, 0xF2);    // TDT_TAP_TIMER
    KIONIX_ACCEL_write_byte(0x2F, 0x36);    // TDT_TOTAL_TIMER
    KIONIX_ACCEL_write_byte(0x30, 0x3C);    // TDT_LATENCY_TIMER
    KIONIX_ACCEL_write_byte(0x31, 0xB4);    // TDT_WINDOW_TIMER
#else // 800Hz
    new_vco_trim = 128+(1.5*(old_vco_trim-128)+19) ;    // OSCILATTOR_TRIM
    //printk("------------------------------> YJ : old_vco_trim = 0x%x  new_vco_trim = 0x%x\n", old_vco_trim, new_vco_trim);
    KIONIX_ACCEL_write_byte(0x59, (unsigned char)new_vco_trim);

    KIONIX_ACCEL_write_byte(0x2B, 0xA0);    // TDT_TIMER
    KIONIX_ACCEL_write_byte(0x2E, 0xF2);    // TDT_TAP_TIMER
    KIONIX_ACCEL_write_byte(0x2F, 0x36);    // TDT_TOTAL_TIMER
    KIONIX_ACCEL_write_byte(0x30, 0x3C);    // TDT_LATENCY_TIMER
    KIONIX_ACCEL_write_byte(0x31, 0xF0);    // TDT_WINDOW_TIMER
#endif

    // X-gain control
    KIONIX_ACCEL_read_bytes(0x50, &x_gain, 1); // X_GAIN
    //KIONIX_ACCEL_write_byte(0x50, 200 );
    //printk("------------------------------> YJ : x_gain = 0x%x --> 200 \n", x_gain);

    // Y-gain control
    KIONIX_ACCEL_read_bytes(0x51, &y_gain, 1); // Y_GAIN
    //KIONIX_ACCEL_write_byte(0x51, 200 );
    //printk("------------------------------> YJ : y_gain = 0x%x --> 0x%x\n", y_gain, y_gain);

    // Z-gain control
    KIONIX_ACCEL_read_bytes(0x52, &z_gain, 1); // Z_GAIN
    //KIONIX_ACCEL_write_byte(0x52, (z_gain)>>1 );
    //printk("------------------------------> YJ : z_gain = 0x%x --> 0x%x\n", z_gain, (z_gain)>>1 );
    
    KXTF9_tdt_l_thresh(30); //201026 taehwan.kim@lge.com For higher tapping low threshold
    KXTF9_tap_mask_all_direction();  
    res |=KXTF9_enable_tap_detection();
	
    return res;

}
int kxtf9_accel_disable_tap(void)
{
	KXTF9_disable_tap_detection();

	// restore internal osc value.
 	//KIONIX_ACCEL_write_byte(0x59, old_osc);//antispoon
	
	return 0;
}
/*******************    Shake   ********************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_enable_shake
   ---------------------------------------------------------------------------*/		
int kxtf9_accel_enable_shake(void)
{
	shakeCount = 0;
	KIONIX_SHAKE_Init(&shakeData);

	return 0;
}
/*---------------------------------------------------------------------------
    kxtf9_accel_detect_shake(void)
   ---------------------------------------------------------------------------*/		
int kxtf9_accel_detect_shake(void)
{
	int     xAcc,yAcc,zAcc;
	long   sumSqr;
	long   tempCount;

	/* get current acceleration */
	KIONIX_ACCEL_read_LPF_g(&xAcc,&yAcc,&zAcc);
#if DEBUG 
    printk ("KIONIX_SHAKE_Update x=%x,y=%x,z=%x \n",xAcc,yAcc,zAcc);
#endif
	/* compute the sum of the squares */
	sumSqr = 0;
	sumSqr += (abs(xAcc)*abs(xAcc))/1000;
	sumSqr += (abs(yAcc)*abs(yAcc))/1000;
	sumSqr += (abs(zAcc)*abs(zAcc))/1000;

	/* update the shake detection engine */
	tempCount = KIONIX_SHAKE_Update(&shakeData,sumSqr);

	/* see if the count changed */
	if(( tempCount > 0 )&& (tempCount != shakeCount))
	{
		shakeCount = tempCount;

		if(shakeCount >= 2)     
		{
			/* Send EVENT To Application For Shake Detection */
			kxtf9_accel_send_shake_event((int)sumSqr);
		}
		else
		{
			shakeCount = tempCount;
		}
	}

	return 0;
	
}

/************************ FLIP **************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_detect_flip
   ---------------------------------------------------------------------------*/
int kxtf9_accel_detect_flip(int value)
{
   static int flip_count = 0;
   int data = 0;

   //CTRL_REG2_UPM |CTRL_REG2_DOM - screen rotation
   if((value == 0x04)||(value == 0x08))  
		flip_count+= 1;
   
   if(flip_count == 1){
       data = ACCEL_FLIP_UPSIDE_DOWN;
   }
   else if(flip_count == 2){
   	   flip_count = 0;
   	   data = ACCEL_FLIP_DOWNSIDE_UP;
   }

   return data;
   
}

/************************ SNAP **************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_enable_snap   ---> USING TAPPING Interrupt 
   ---------------------------------------------------------------------------*/		
int kxtf9_accel_enable_snap(void)
{
	unsigned char dummy;

	printk("kxtf9_accel_enable_snap..................\n");		
	KIONIX_ACCEL_disable_outputs();
	KIONIX_ACCEL_disable_interrupt();
	kxtf9_accel_disable_tap();
       	
	// Configure Directional-Tap
	KXTF9_set_odr_tap(50); // 50Hz

	// Check point : ODR for TAP is 50Hz,
	KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_CTRL_REG3, &dummy, 1);
	if(dummy != 0x41) 
	{
		printk("-I- CTRL_REG3 is set to 0x41 for Directional-shake (dummy=0x%x) \n", dummy);
		KIONIX_ACCEL_write_byte(KIONIX_ACCEL_I2C_CTRL_REG3, 0x41);
	}
	
	// parameter
	KIONIX_ACCEL_write_byte(0x2B, 0x09);
	KIONIX_ACCEL_write_byte(0x2C, 0xFF);
	KIONIX_ACCEL_write_byte(0x2D, 0x3C);
	KIONIX_ACCEL_write_byte(0x2E, 0x43);
	KIONIX_ACCEL_write_byte(0x2F, 0x18);
	KIONIX_ACCEL_write_byte(0x30, 0x08);
	KIONIX_ACCEL_write_byte(0x31, 0x19);
	
	KXTF9_tap_mask_all_direction();  // X+/-,Y+/-,Z+/- ?????
	KXTF9_enable_tap_detection();
		
	//KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_INT_REL, &dummy, 1);
		
	KIONIX_ACCEL_enable_interrupt();
	KIONIX_ACCEL_enable_outputs();
		
	return 0;

}

int kxtf9_accel_disable_snap(void)
{
	unsigned char dummy;
	
	printk("kxtf9_accel_disable_snap..................\n");	
	KIONIX_ACCEL_disable_interrupt();
	KIONIX_ACCEL_disable_outputs();
	KXTF9_disable_tap_detection();

	kxtf9_accel_enable_tap();

	KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_INT_REL, &dummy, 1);
		
	KIONIX_ACCEL_enable_interrupt();
	KIONIX_ACCEL_enable_outputs();

	return 0;	
	
}

/**********************  ACCEL. RAW DATA   ********************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_read_raw_data
   ---------------------------------------------------------------------------*/		
int kxtf9_accel_read_raw_data(unsigned char *buf)
{
	unsigned char data[6] ={0,};

/*LGE_CHANGE_S [taehwan.kim@lge.com] 2010-01-18 temporary enable function*/
  //  KIONIX_ACCEL_enable_outputs(); //antispoon0118
/*LGE_CHANGE_S [taehwan.kim@lge.com] 2010-01-18 temporary enable function*/
	kxtf9_i2c_read(KXTF9_I2C_XOUT_L,&data[0],1);
	kxtf9_i2c_read(KXTF9_I2C_XOUT_H,&data[1],1);
	kxtf9_i2c_read(KXTF9_I2C_YOUT_L,&data[2],1);
	kxtf9_i2c_read(KXTF9_I2C_YOUT_H,&data[3],1);
	kxtf9_i2c_read(KXTF9_I2C_ZOUT_L,&data[4],1);
	kxtf9_i2c_read(KXTF9_I2C_ZOUT_H,&data[5],1);
	
	if(buf)
	{
	    memcpy(buf,data,sizeof(unsigned char)*6);
	}
	
	return 0;
}

/**********************  ACCEL. DATA for JSR256 API   **************************/
/*---------------------------------------------------------------------------
	kxtf9_accel_api_read_data
   ---------------------------------------------------------------------------*/		
int kxtf9_accel_api_read_data(int *buf)
{
	if(buf)
	{
		KIONIX_ACCEL_read_LPF_g(&buf[0],&buf[1],&buf[2]);
	}

	return 0;
}

/*         output data rate     LPF Roll-Off
*    0:           12.5 Hz                   6.25 Hz            
*    1:              25 Hz                    12.5 Hz
*    2:              50 Hz                       25 Hz
*    3:            100 Hz                       50 Hz 
*    4 :           200 Hz                     100 Hz
*    5:            400 Hz                     200 Hz
*    6:            800 Hz                     400 Hz
*/

int kxtf9_accel_api_set_sample_rate(int index)
{
	int sample_rate =0;

//--[[ LGE_UBIQUIX_MODIFIED_START : shyun@ubiquix.com [2011.07.01] - Log removed
//    printk("kxtf9_accel_api_set_sample_rate index = %d \n",index);
//--]] LGE_UBIQUIX_MODIFIED_END : shyun@ubiquix.com [2011.07.01]- Log removed
	if((index < 0) || (index >= MAX_KXTF9_SAMPLE_RATE_INDEX))
	{
		return -1;
	}

	sample_rate = kxtf9_data_lpf_buf[index];
//--[[ LGE_UBIQUIX_MODIFIED_START : shyun@ubiquix.com [2011.07.01] - Log removed
//    printk("kxtf9_accel_api_set_sample_rate = %d \n",index);
//--]] LGE_UBIQUIX_MODIFIED_END : shyun@ubiquix.com [2011.07.01]- Log removed
       if(KXTF9_set_lpf_odr(sample_rate) != 0)
       {
       	return -1;
       }

	return 0;   
}

int kxtf9_accel_api_get_sample_rate(int *index)
{
	char data_ctrl_reg = 0;
	int    current_index = 0;

	if(index == NULL)
	{
		return -1;
	}
	
	if (KIONIX_ACCEL_read_bytes(KXTF9_I2C_DATA_CTRL_REG, &data_ctrl_reg, 1) != 0)   // fail
	{
		return -1;
	}

	current_index = (int)(data_ctrl_reg & 0x07);	

	printk("[kxtf9_accel_api_get_sample_rate] KXTF9_I2C_DATA_CTRL_REG[0x%x] \n",current_index);

	*index = current_index;

	return 0;
	
}

int kxtf9_accel_api_get_g_range(int *range)
{	
	char data_ctrl_reg = 0;
	int    current_range = 0;

	if(range == NULL)
	{
		return -1;
	}

	if(KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_CTRL_REG1, &data_ctrl_reg, 1) != 0)  // fail
	{
		return -1;
	}
	
	*range = 0;

	current_range = ((data_ctrl_reg & 0x18) >> 3);

	printk("kxtf9_accel_api_get_g_range: [0x%x]\n",current_range);

	switch(current_range)
	{
	case 0:
		*range = 2;
	case 1:
		*range = 4;
	case 2:
		*range = 8;
	default:
		break;
	}

	return 0;
	
}

int kxtf9_accel_api_get_resolution(int *resolution)
{	
	char data_ctrl_reg = 0;
	int    current_resolution = 0;

	if(resolution == NULL)
	{
		return -1;
	}

	if (KIONIX_ACCEL_read_bytes(KIONIX_ACCEL_I2C_CTRL_REG1, &data_ctrl_reg, 1) != 0){
		return 1;
	}
	
	*resolution = 0;

	current_resolution = ((data_ctrl_reg & 0x40) >> 6);

	printk("kxtf9_accel_api_get_resolution: [0x%x]\n",current_resolution);

	switch(current_resolution)
	{
	case 0:
		*resolution = 8;
	case 1:
		*resolution = 12;
	default:
		break;
	}

	return 0;
	
}


