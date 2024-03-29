/* 
 * Copyright (C) 2009 LGE Inc.
 * 
 * KIONIX Accelerometer API  header file.
 *
 * 2009/11/10 : 
 * 
 */


#ifndef __KIONIX_ACCEL_API_H__
#define __KIONIX_ACCEL_API_H__

#include <linux/i2c.h>

/*==================================================================================================
						MACROS
==================================================================================================*/
#define SET_REG_BIT(r,b) r |= b
#define UNSET_REG_BIT(r,b) r &= ~b

/*==================================================================================================
						KIONIX ACCEL REGISTER MAP
==================================================================================================*/
#define KIONIX_ACCEL_I2C_SLAVE_ADDR 		0x0F


/*==================================================================================================
						COMMON REGISTERS
==================================================================================================*/
#define KIONIX_ACCEL_I2C_ST_RESP			0x0C
#define KIONIX_ACCEL_I2C_WHO_AM_I			0x0F
#define KIONIX_ACCEL_I2C_TILT_POS_CUR		0x10
#define KIONIX_ACCEL_I2C_TILT_POS_PRE		0x11
#define KIONIX_ACCEL_I2C_STATUS_REG			0x18
#define KIONIX_ACCEL_I2C_INT_REL			0x1A
#define KIONIX_ACCEL_I2C_CTRL_REG1			0x1B
#define KIONIX_ACCEL_I2C_CTRL_REG2			0x1C
#define KIONIX_ACCEL_I2C_CTRL_REG3			0x1D
#define KIONIX_ACCEL_I2C_INT_CTRL_REG2		0x1F
#define KIONIX_ACCEL_I2C_TILT_TIMER			0x28
#define KIONIX_ACCEL_I2C_WUF_TIMER			0x29
#define KIONIX_ACCEL_I2C_WUF_THRESH			0x5A

/*==================================================================================================
						KXTE9-SPECIFIC REGISTERS
==================================================================================================*/
#define KXTE9_I2C_XOUT						0x12
#define KXTE9_I2C_YOUT						0x13
#define KXTE9_I2C_ZOUT						0x14
#define KXTE9_I2C_INT_SRC_REG1				0x16
#define KXTE9_I2C_INT_SRC_REG2				0x17
#define KXTE9_I2C_INT_CTRL_REG1				0x1E
#define KXTE9_I2C_B2S_TIMER					0x2A
#define KXTE9_I2C_B2S_THRESH				0x5B

/*==================================================================================================
						KXTF9-SPECIFIC REGISTERS
==================================================================================================*/
#define KXTF9_I2C_XOUT_HPF_L				0x00
#define KXTF9_I2C_XOUT_HPF_H				0x01
#define KXTF9_I2C_YOUT_HPF_L				0x02
#define KXTF9_I2C_YOUT_HPF_H				0x03
#define KXTF9_I2C_ZOUT_HPF_L				0x04
#define KXTF9_I2C_ZOUT_HPF_H				0x05
#define KXTF9_I2C_XOUT_L					0x06
#define KXTF9_I2C_XOUT_H					0x07
#define KXTF9_I2C_YOUT_L					0x08
#define KXTF9_I2C_YOUT_H					0x09
#define KXTF9_I2C_ZOUT_L					0x0A
#define KXTF9_I2C_ZOUT_H					0x0B
#define KXTF9_I2C_INT_SRC_REG1				0x15
#define KXTF9_I2C_INT_SRC_REG2				0x16
#define KXTF9_I2C_INT_CTRL_REG1				0x1E
#define KXTF9_I2C_INT_CTRL_REG3				0x20
#define KXTF9_I2C_DATA_CTRL_REG				0x21
#define KXTF9_I2C_TDT_TIMER					0x2B
#define KXTF9_I2C_TDT_H_THRESH				0x2C
#define KXTF9_I2C_TDT_L_THRESH				0x2D
#define KXTF9_I2C_TDT_TAP_TIMER				0x2E
#define KXTF9_I2C_TDT_TOTAL_TIMER			0x2F
#define KXTF9_I2C_TDT_LATENCY_TIMER			0x30
#define KXTF9_I2C_TDT_WINDOW_TIMER			0x31

/*==================================================================================================
						KIONIX ACCEL CONTROL BIT DEFINITION
==================================================================================================*/
/*==================================================================================================
						COMMON CONTROL BITS
==================================================================================================*/
#define CTRL_REG1_TPE						0x01	/* enables tilt position function */
#define CTRL_REG1_WUFE						0x02	/* enables wake up function */
#define CTRL_REG1_PC1						0x80	/* operating mode 1 = full power mode; 0 = stand by mode */
#define CTRL_REG2_FUM						0x01	/* face up state mask */
#define CTRL_REG2_FDM						0x02	/* face down state mask */
#define CTRL_REG2_UPM						0x04	/* up state mask */
#define CTRL_REG2_DOM						0x08	/* down state mask */
#define CTRL_REG2_RIM						0x10	/* right state mask */
#define CTRL_REG2_LEM						0x20	/* left state mask */
#define CTRL_REG3_OWUFB						0x01	/* active mode output data rate */
#define CTRL_REG3_OWUFA						0x02	/* active mode output data rate */
#define CTRL_REG3_STC						0x10	/* initiates self-test function */
#define CTRL_REG3_OTPB						0x20	/* sets output data rate for tilt position function */
#define CTRL_REG3_OTPA						0x40	/* sets output data rate for tilt position function */
#define CTRL_REG3_SRST						0x80	/* software reset */
#define INT_CTRL_REG2_XBW					0x80	/* X-axis motion mask */
#define INT_CTRL_REG2_YBW					0x40	/* Y-axis motion mask */
#define INT_CTRL_REG2_ZBW					0x20	/* Z-axis motion mask */

/*==================================================================================================
						KXTE9-SPECIFIC CONTROL BITS
==================================================================================================*/
#define CTRL_REG1_B2SE						0x04	/* enables back to sleep function on KXTE9 */
#define CTRL_REG1_ODRB						0x08	/* bit0 for selecting the output data rate on KXTE9 */
#define CTRL_REG1_ODRA						0x10  	/* bit1 for selecting the output data rate on KXTE9 */
#define CTRL_REG3_OB2SB						0x04	/* sets output data rate when in inactive mode (KXTE9) */
#define CTRL_REG3_OB2SA						0x08	/* sets output data rate when in incative mode (KXTE9) */
#define KXTE9_INT_CTRL_REG1_IEL				0x04	/* sets response of physical interrupt pin */
#define KXTE9_INT_CTRL_REG1_IEA				0x08	/* sets polarity of the physical interrupt pin*/
#define KXTE9_INT_CTRL_REG1_IEN				0x10	/* enables/disables the physical interrupt pin; 1=enable; 0=disable */
#define FULL_SCALE_RANGE_2_G    			2000    /* indicates full scale g-range of the KIONIX_ACCEL */
#define BIT_SENSITIVITY_2_G     			  16      /* indicates sensitivity of the KIONIX_ACCEL ((2^6)/4) */
#define ZERO_G_OFFSET 		    			  32      /* indicates 0g offset of the KIONIX_ACCEL ((2^6)/2) */

/*==================================================================================================
						KXTF9-SPECIFIC CONTROL BITS
==================================================================================================*/
#define CTRL_REG1_TDTE						0x04	/* enables tap/double tap function on KXTF9 */
#define CTRL_REG1_GSEL0						0x08	/* bit0 used for selecting the g range */
#define CTRL_REG1_GSEL1						0x10  	/* bit1 used for selecting the g range */
#define CTRL_REG1_DRDYE						0x20	/* enables physical interrupt when new accel data is available on KXTF9 */
#define CTRL_REG1_RES						0x40	/* performance mode on KXTF9 */
#define CTRL_REG3_OTDTB						0x04	/* sets output data rate for tap double tap function */
#define CTRL_REG3_OTDTA						0x08	/* sets output data rate for tap double tap function */
#define INT_CTRL_REG3_TFUM					0x01	/* Z positive tap detection mask */
#define INT_CTRL_REG3_TFDM					0x02	/* Z negative tap detection mask */
#define INT_CTRL_REG3_TUPM					0x04	/* Y positive tap detection mask */
#define INT_CTRL_REG3_TDOM					0x08	/* Y negative tap detection mask */
#define INT_CTRL_REG3_TRIM					0x10	/* X positive tap detection mask */
#define INT_CTRL_REG3_TLEM					0x20	/* X negative tap detection mask */
#define KXTF9_INT_CTRL_REG1_STPOL			0x02	/* ST (self test) polarity bit */
#define KXTF9_INT_CTRL_REG1_ULMB			0x04	/* unlatched mode for WUF 1=unlatche; 0=latched */
#define KXTF9_INT_CTRL_REG1_IEL				0x08	/* sets response of physical interrupt pin */
#define KXTF9_INT_CTRL_REG1_IEA				0x10	/* sets polarity of the physical interrupt pin*/
#define KXTF9_INT_CTRL_REG1_IEN				0x20	/* enables/disables the physical interrupt pin; 1=enable; 0=disable */
#define DATA_CTRL_REG_OSAC					0x01	/* sets LPF Acceleration output data rates */
#define DATA_CTRL_REG_OSAB					0x02	/* sets LPF Acceleration output data rates */
#define DATA_CTRL_REG_OSAA					0x04	/* sets LPF Acceleration output data rates */
#define DATA_CTRL_REG_HPFROB				0x10	/* sets HPF roll-off frequency */
#define DATA_CTRL_REG_HPFROA				0x20 	/* sets HPF roll-off frequency */


enum {
	PROCESS_NONE=0,
	PROCESS_INTERRUPT,
	PROCESS_SHAKEPOLL,
	PROCESS_OTHER,
};


//*************************************************************
//	Name		: shake.h
//	Author	  	: Kionix, Inc.
//	Version	 	: 1.0
//	Copyright   : 
//	Description : shake detection
//*************************************************************

#define CONFIG_RATE         	     			 50      // data rate (hz)
#define CONFIG_DURATION     	   				180     // max length of a shake (ms)
#define CONFIG_DELAY        	  				  0       // min seperation between shakes (ms)
#define CONFIG_TIMEOUT      	    			580 //280     // max seperation between shakes (ms)
#define CONFIG_THRESHOLD    			     	3000 //20000   // min shake threshold ((x^2 + y^2 + z^2) / 1000)

typedef struct
{
    long cntShake;                  // valid shake count
    long cntInvalid;                // invalid shake count

    long cntDuration;               // duration timer
    long cntDelay;                  // delay timer
    long cntTimeout;                // timeout timer

    long maxDuration;               // duration threshold
    long maxDelay;                  // delay threshold
    long maxTimeout;                // timeout threshold

} shake_data;


//*************************************************************
// I2C function
//*************************************************************

// [update] chanhee.park@lge.com

typedef int (*accel_i2c_read_fn)(unsigned char reg,unsigned char *data,int length);
typedef int (*accel_i2c_write_fn)(unsigned char reg,unsigned char data);


/*==================================================================================================
					COMMON FUNCTIONS
==================================================================================================*/
int KIONIX_ACCEL_read_bytes(int reg, unsigned char* data, int length);
int KIONIX_ACCEL_write_byte(int reg, int data);
int KIONIX_ACCEL_deinit(void);
int KIONIX_ACCEL_init(void);
void KIONIX_ACCEL_isr(void);
void KIONIX_ACCEL_enable_interrupt(void);
void KIONIX_ACCEL_disable_interrupt(void);
int KIONIX_ACCEL_enable_outputs(void);
int KIONIX_ACCEL_disable_outputs(void);
int KIONIX_ACCEL_enable_tilt_function(void);
int KIONIX_ACCEL_disable_tilt_function(void);
int KIONIX_ACCEL_enable_wake_up_function(void);
int KIONIX_ACCEL_disable_wake_up_function(void);
int KIONIX_ACCEL_enable_all(void);
int KIONIX_ACCEL_disable_all(void);
int KIONIX_ACCEL_sleep(void);
int KIONIX_ACCEL_read_interrupt_status(void);
int KIONIX_ACCEL_read_interrupt_source(char* interrupt_source);
int KIONIX_ACCEL_read_previous_position(char* previous_position);
int KIONIX_ACCEL_read_current_position(char* current_position);
int KIONIX_ACCEL_reset(void);
int KIONIX_ACCEL_tilt_timer(int tilt_timer);
int KIONIX_ACCEL_wuf_timer(int wuf_timer);
int KIONIX_ACCEL_wuf_thresh(int wuf_thresh);
int KIONIX_ACCEL_motion_mask_z(void);
int KIONIX_ACCEL_motion_unmask_z(void);
int KIONIX_ACCEL_motion_mask_y(void);
int KIONIX_ACCEL_motion_unmask_y(void);
int KIONIX_ACCEL_motion_mask_x(void);
int KIONIX_ACCEL_motion_unmask_x(void);
int KIONIX_ACCEL_position_mask_fu(void);
int KIONIX_ACCEL_position_mask_fd(void);
int KIONIX_ACCEL_position_mask_up(void);
int KIONIX_ACCEL_position_mask_do(void);
int KIONIX_ACCEL_position_mask_ri(void);
int KIONIX_ACCEL_position_mask_le(void);
int KIONIX_ACCEL_position_unmask_fu(void);
int KIONIX_ACCEL_position_unmask_fd(void);
int KIONIX_ACCEL_position_unmask_up(void);
int KIONIX_ACCEL_position_unmask_do(void);
int KIONIX_ACCEL_position_unmask_ri(void);
int KIONIX_ACCEL_position_unmask_le(void);
int KIONIX_ACCEL_service_flip(int vib_time); //antispoon
int KIONIX_ACCEL_set_odr_motion(int frequency);
int KIONIX_ACCEL_service_interrupt(int vibrator_value);
int KIONIX_ACCEL_read_LPF_cnt(int* x, int* y, int* z);
int KIONIX_ACCEL_read_LPF_g(int* gx, int* gy, int* gz);
int KIONIX_ACCEL_read_current_odr_motion(double* ODR_rate_motion);
int KIONIX_ACCEL_read_position_status(void);
int KIONIX_ACCEL_read_wuf_status(void);
int KIONIX_ACCEL_enable_int(void);
int KIONIX_ACCEL_disable_int(void);
int KIONIX_ACCEL_int_activeh(void);
int KIONIX_ACCEL_int_activel(void);
int KIONIX_ACCEL_int_latch(void);
int KIONIX_ACCEL_int_pulse(void);
void KIONIX_ACCEL_i2c_register_cb(void);
void KIONIX_SHAKE_Init(shake_data* data);
long KIONIX_SHAKE_Update(shake_data* data, long val);

/*==================================================================================================
					KXTE9-SPECIFIC FUNCTIONS
==================================================================================================*/
int KIONIX_ACCEL_enable_back_to_sleep(void);
int KIONIX_ACCEL_disable_back_to_sleep(void);
int KXTE9_read_b2s_status(void);
int KXTE9_b2s_timer(int b2s_timer);
/*==================================================================================================
					KXTF9-SPECIFIC FUNCTIONS
==================================================================================================*/
int KXTF9_enable_tap_detection(void);
int KXTF9_disable_tap_detection(void);
int KXTF9_read_single_tap_status(void);
int KXTF9_read_double_tap_status(void);
int KXTF9_set_odr_tilt(int frequency);
int KXTF9_set_G_range(int range);
int KXTF9_read_HPF_cnt(int* x, int* y, int* z);
int KXTF9_read_HPF_g(int* gx, int* gy, int* gz);
int KXTF9_read_current_odr_tilt(double* ODR_rate_tilt);
int KXTF9_read_current_odr_tap(double* ODR_rate_tap);
int KXTF9_read_tap_direction(char* int_src_reg1);
int KXTF9_int_alt_disable(void);
int KXTF9_int_alt_enable(void);
int KXTF9_tdt_timer(int tdt_timer);
int KXTF9_tdt_h_thresh(int tdt_h_thresh);
int KXTF9_tdt_l_thresh(int tdt_l_thresh);
int KXTF9_tdt_tap_timer(int tdt_tap_timer);
int KXTF9_tdt_total_timer(int tdt_total_timer);
int KXTF9_tdt_latency_timer(int tdt_latency_timer);
int KXTF9_tdt_window_timer(int tdt_window_timer);
int KXTF9_tap_mask_TFU(void);
int KXTF9_tap_unmask_TFU(void);
int KXTF9_tap_mask_TFD(void);
int KXTF9_tap_unmask_TFD(void);
int KXTF9_tap_mask_TUP(void);
int KXTF9_tap_unmask_TUP(void);
int KXTF9_tap_mask_TDO(void);
int KXTF9_tap_unmask_TDO(void);
int KXTF9_tap_mask_TRI(void);
int KXTF9_tap_unmask_TRI(void);
int KXTF9_tap_mask_TLE(void);
int KXTF9_tap_unmask_TLE(void);
int KXTF9_tap_mask_all_direction(void);
int KXTF9_tap_unmask_all_direction(void);
int KXTF9_set_odr_tap(int alpha);
int KXTF9_set_hpf_odr(int frequency);
int KXTF9_set_lpf_odr(int frequency);
int KXTF9_set_resolution(int resolution);




#endif //__KIONIX_ACCEL_API_H__


