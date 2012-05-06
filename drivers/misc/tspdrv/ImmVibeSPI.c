/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef 	IMMVIBESPIAPI
#undef 	IMMVIBESPIAPI
#endif
#define 	IMMVIBESPIAPI static

#include <linux/platform_device.h>
#include <mach/gpio.h>
//#include <linux/delay.h>
//#include "../staging/android/timed_output.h"
//#include <linux/hrtimer.h>
#include <plat/dmtimer.h>
#include <linux/regulator/consumer.h>
#if (CONFIG_MACH_LGE_HUB)
#include "../mux.h"
#endif

/* This SPI supports only one actuator. */
#define 	NUM_ACTUATORS   	1

#define 	VIB_DEBUG 		1
#if (defined(CONFIG_PRODUCT_LGE_HUB) || defined(CONFIG_PRODUCT_LGE_JUSTIN))
#undef		USE_SUBPM
#else // CONFIG_PRODUCT_LGE_BLACK
#define     USE_SUBPM 
#endif

static bool g_bAmpEnabled = false;

#ifdef USE_SUBPM
#if defined(CONFIG_REGULATOR_LP8720)
#include <linux/regulator/lp8720.h>
#endif
#endif


#if (defined(CONFIG_PRODUCT_LGE_HUB))
#define HUB_VIBE_GPIO_EN			136
#elif (defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN))
#define HUB_VIBE_GPIO_EN			57
#endif	// #if (CONFIG_MACH_LGE_HUB)
#define HUB_VIBE_PWM				56
#define HUB_VIBE_GPTIMER_NUM		10


/* LGE_CHANGE_S, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */
//--[[ LGE_UBIQUIX_MODIFIED_START : bsnoh@ubiquix.com : add from DCM_GB
#if (defined(CONFIG_PRODUCT_LGE_HUB) || defined(CONFIG_PRODUCT_LGE_BLACK) || defined(CONFIG_PRODUCT_LGE_JUSTIN))
//--[[ LGE_UBIQUIX_MODIFIED_END : bsnoh@ubiquix.com : add from DCM_GB
#define USE_SYS_CLK
#undef USE_32_CLK
#else
#define USE_32_CLK
#undef USE_SYS_CLK
#endif // if 0

//--[[ LGE_UBIQUIX_MODIFIED_START : bsnoh@ubiquix.com : add from DCM_GB
#if (defined(CONFIG_PRODUCT_LGE_HUB) || defined(CONFIG_PRODUCT_LGE_BLACK) || defined(CONFIG_PRODUCT_LGE_JUSTIN))
//--[[ LGE_UBIQUIX_MODIFIED_END : bsnoh@ubiquix.com : add from DCM_GB
#define PWM_DUTY_MAX	 1158 /*1158 /* 22.43 kHz */
#else
#ifdef USE_32_CLK
#define PWM_DUTY_MAX	 0xAEFFFFFF//use 32k clock source ok
#else
#define PWM_DUTY_MAX	 0x387638//use sys clock 26MHz source
#endif // USE_32_CLK
#endif // if 0
/* LGE_CHANGE_E, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */


#define PLTR_VALUE		(0xFFFFFFFF - PWM_DUTY_MAX)
#define PWM_DUTY_HALF	(0xFFFFFFFF - (PWM_DUTY_MAX >> 1))
static struct omap_dm_timer *omap_vibrator_timer = NULL;


#ifdef USE_SUBPM
#if defined(CONFIG_REGULATOR_LP8720)
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_output_enable(void);
#endif
#else
static struct regulator *vibe_regulator = NULL;
#endif

static void hub_vibrator_gpio_enable (int enable)
{
	if (enable)
		gpio_set_value(HUB_VIBE_GPIO_EN, 1);
	else 	
		gpio_set_value(HUB_VIBE_GPIO_EN, 0);
}

static void hub_vibrator_LDO_enable(int val)
{

#ifdef USE_SUBPM
	subpm_set_output(1, val);
	subpm_output_enable();
#else
	if (val == 1){
		regulator_enable(vibe_regulator);
	}else{
		regulator_disable(vibe_regulator);
	}
#endif
}

static void vib_enable(int on )
{
	hub_vibrator_gpio_enable(on);
	hub_vibrator_LDO_enable(on);

	return VIBE_S_SUCCESS;
}

static void vib_generatePWM(int on)
{
	if(on) {
		/* Select clock */
		omap_dm_timer_enable(omap_vibrator_timer);

/* LGE_CHANGE_S, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */
#ifdef USE_32_CLK
		omap_dm_timer_set_source(omap_vibrator_timer, OMAP_TIMER_SRC_32_KHZ);
#else
		omap_dm_timer_set_source(omap_vibrator_timer, OMAP_TIMER_SRC_SYS_CLK);
#endif
/* LGE_CHANGE_E, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */

		/* set a period */
		omap_dm_timer_set_load(omap_vibrator_timer, 1, PLTR_VALUE);

		/* set a duty */
		omap_dm_timer_set_match(omap_vibrator_timer, 1, PWM_DUTY_HALF);
/* LGE_CHANGE_S, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */
#if (defined(CONFIG_PRODUCT_LGE_HUB) || defined(CONFIG_PRODUCT_LGE_BLACK) || defined(CONFIG_PRODUCT_LGE_JUSTIN))
		omap_dm_timer_set_pwm(omap_vibrator_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
#else	// docomo
		omap_dm_timer_set_pwm(omap_vibrator_timer, 1, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
#endif
/* LGE_CHANGE_E, ryu.seeyeol@lge.com, 2011-03-21, Change the PWM Clock Source */
		omap_dm_timer_start(omap_vibrator_timer);		
	}
	else {
		omap_dm_timer_stop(omap_vibrator_timer);
		omap_dm_timer_disable(omap_vibrator_timer);
	}

	return VIBE_S_SUCCESS;
}


/* Called to disable amp (disable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable( VibeUInt8 nActuatorIndex )
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable[%d]\n", g_bAmpEnabled ));
	//printk("[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable[%d]\n", g_bAmpEnabled);
    	if ( g_bAmpEnabled ) {
        	g_bAmpEnabled = false;
		vib_enable(false);
		vib_generatePWM(false);
       }
	return VIBE_S_SUCCESS;
}


/* Called to enable amp (enable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable( VibeUInt8 nActuatorIndex)
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnabled[%d]\n", g_bAmpEnabled ));
	if ( ! g_bAmpEnabled ) {
		g_bAmpEnabled = true;
		vib_generatePWM(true);
		vib_enable(true);
	}
   	return VIBE_S_SUCCESS;
}


/* Called at initialization time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize( void)
{
	int status = 0;
	int ret = 0;
#if(defined(CONFIG_PRODUCT_LGE_HUB))
	// 20100810 jh.koo@lge.com GPIO Initialization [START_LGE]
	omap_mux_init_gpio(HUB_VIBE_GPIO_EN, OMAP_PIN_OUTPUT);
	//	omap_mux_init_gpio(HUB_VIBE_PWM, OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_OUTPUT);
	// 20100810 jh.koo@lge.com GPIO Initialization [END_LGE]	
#endif // CONFIG_PRODUCT_LGE_HUB

#ifdef USE_SUBPM
#else
	vibe_regulator = regulator_get(NULL, "vaux1");
	if (vibe_regulator == NULL) {
		printk("LGE: vaux1 regulator get fail\n");
		return VIBE_E_FAIL;
	}
#endif
   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));
	//g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
	ret = gpio_request(HUB_VIBE_GPIO_EN, "Hub Vibrator Enable");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for vibrator\n", __func__, HUB_VIBE_GPIO_EN);
	}
	gpio_direction_output(HUB_VIBE_GPIO_EN, 0);

	omap_vibrator_timer = omap_dm_timer_request_specific(HUB_VIBE_GPTIMER_NUM);
	if (omap_vibrator_timer == NULL) {
		printk(KERN_ERR "%s: failed to request omap pwm timer.\n", __func__);
		ret = -ENODEV;
	}
	omap_dm_timer_disable(omap_vibrator_timer); 
   	//ImmVibeSPI_ForceOut_AmpDisable( 0 );

    return VIBE_S_SUCCESS;
}


/* Called at termination time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate( void )
{
   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

    	ImmVibeSPI_ForceOut_AmpDisable(0);

    	return VIBE_S_SUCCESS;
}

bool bInTestMode = 0; /* 20110125 jiwon.seo@lge.com for ELT vibrator */
/*** Called by the real-time loop to set PWM duty cycle, and enable amp if required*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set( VibeUInt8 nActuatorIndex, VibeInt8 nForce )
{
//--[[ LGE_UBIQUIX_MODIFIED_START : bsnoh@ubiquix.com : add from DCM_GB
/* LGE_CHANGE_S, ryu.seeyeol@lge.com, 2011-05-13, This function executes twice.. duplicate vib_generatePWM() */
#if 1 
//--[[ LGE_UBIQUIX_MODIFIED_END : bsnoh@ubiquix.com : add from DCM_GB
	unsigned int nTmp;
	
	DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set nForce =  %d \n", nForce ));

#if 1
	/* Check the Force value with Max and Min force value */
	if (nForce > 127) nForce = 127;
	if (nForce < -127) nForce = -127;
	//20110305 seungdae.goh@lge.com [
#if defined(CONFIG_PRODUCT_LGE_JUSTIN) // bsnoh BLACK???
	if( nForce > 110 ) {
		nForce = 127; 
		//printk( "[ImmVibeSPI] Force MAX value [%d]  change !! \n", nForce );
	}
#elif (defined(CONFIG_PRODUCT_LGE_HUB))
	//20110305 seungdae.goh@lge.com ]
	
/* 20110125 jiwon.seo@lge.com for ELT vibrator [START] */
	if(bInTestMode)
		   {		   
			 if(nForce > 0 && nForce < 125) 
			 	nForce = 125; 
    	}
/* 20110125 jiwon.seo@lge.com for ELT vibrator [END] */
#endif
	
	if (nForce == 0) {
		hub_vibrator_gpio_enable(0);
		omap_dm_timer_stop(omap_vibrator_timer);		
	} else {
		hub_vibrator_gpio_enable(1);
		nTmp = 0xFFFFFFF7 - (((127 - nForce) * 9) >> 1);
// bsnoh@ubiquix.com Prevent register access for kernel panic if the timer already disabled[START]		
		omap_dm_timer_enable(omap_vibrator_timer);
// bsnoh@ubiquix.com Prevent register access for kernel panic if the timer already disabled[End]	
		omap_dm_timer_set_match(omap_vibrator_timer, 1, nTmp);
		omap_dm_timer_start(omap_vibrator_timer);		
	}
#endif
//--[[ LGE_UBIQUIX_MODIFIED_START : bsnoh@ubiquix.com : add from DCM_GB
#endif
/* LGE_CHANGE_E, ryu.seeyeol@lge.com, 2011-05-13, This function executes twice.. duplicate vib_generatePWM() */
//--[[ LGE_UBIQUIX_MODIFIED_END : bsnoh@ubiquix.com : add from DCM_GB
	return VIBE_S_SUCCESS;
}


/*** Called by the real-time loop to set force output, and enable amp if required*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples( VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer )
{
    VibeStatus status = VIBE_E_FAIL;

    /* nOutputSignalBitDepth should always be 8 */

    if (1 == nBufferSizeInBytes)
    {
        status = ImmVibeSPI_ForceOut_Set(nActuatorIndex, pForceOutputBuffer[0]);
    }
    else
    {
        /* Send 'nBufferSizeInBytes' bytes of data to HW */
        /* Will get here only if configured to handle Piezo actuators */
		printk( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes );
    }

    return status;
}


/*** Called to set force output frequency parameters*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency( VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue )
{
    /* This function is not called for ERM device */
	return VIBE_S_SUCCESS;
}


/*** Called to get the device name (device name must be returned as ANSI char)*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName( VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}

