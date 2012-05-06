/*
 * Copyright (C) 2010 LG Electronics Inc.
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom-panel.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// prime@sdcmicro.com Fix the compilation error in case of PANEL_HUB not being defined [START]
#if defined(CONFIG_PANEL_JUSTIN)

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>

#ifdef CONFIG_OMAP2_DSS_HDMI
#include <linux/sil9022.h>
// == 2011.05.11 === hycho@ubiquix.com START
//#include <linux/subpm_bh6172.h>
#if defined(CONFIG_REGULATOR_LP8720)
#include <linux/regulator/lp8720.h>
#define LP8720_ENABLE 37
#endif
// == 2011.05.11 === hycho@ubiquix.com END
#endif

#include <linux/omapfb.h>
#include <linux/delay.h>
#include <plat/omap-pm.h>
#include <plat/display.h>
#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include "mux.h"


#define ENABLE_VAUX2_DEDICATED          0x09
#define ENABLE_VAUX2_DEV_GRP            0x20
#define ENABLE_VAUX3_DEDICATED          0x03
#define ENABLE_VAUX3_DEV_GRP            0x20

#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36


#define LCD_PANEL_BACKLIGHT_GPIO 	(7 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_ENABLE_GPIO       (15 + OMAP_MAX_GPIO_LINES)

#define LCD_PANEL_RESET_GPIO		55
#define LCD_PANEL_QVGA_GPIO			56
#define TV_PANEL_ENABLE_GPIO		95

/* 20100623 kyungtae.oh@lge.com MIPI DSI setting [LGE_START]*/
#define DSI_CLOCK_POLARITY  0   /* +/- pin order */
#define DSI_DATA0_POLARITY  0   /* +/- pin order */
#define DSI_DATA1_POLARITY  0   /* +/- pin order */
#define DSI_CLOCK_LANE      3   /* Clock lane position: 1 */
#define DSI_DATA0_LANE      1   /* Data0 lane position: 2 */
#define DSI_DATA1_LANE      2   /* Data1 lane position: 3 */

#define LCD_RESET_N         34
#define LCD_TE              53 //V_SYNC
//--[[ LGE_UBIQUIX_MODIFIED_START : jslee@ubiquix.com [2011.06.27] - j_froyo merge
//#define LCD_CS              54
#define LCD_CP_EN						149
//--[[ LGE_UBIQUIX_MODIFIED_END : jslee@ubiquix.com [2011.06.27] - j_froyo merge
 
/* 20100623 kyungtae.oh@lge.com MIPI DSI setting [LGE_EMD]*/


#ifdef CONFIG_OMAP2_DSS_HDMI
#define HDMI_REG_EN_GPIO                55
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_output_enable(void);
#endif

extern void justin_lcd_initialize(void);

//--[[ LGE_UBIQUIX_MODIFIED_START : ymjun@mnbt.co.kr [2011.07.26] - CAM
//static 
int hdmi_power_initialize =0;          // == 2011.05.28 === hycho@ubiquix.com
//--]] LGE_UBIQUIX_MODIFIED_END : ymjun@mnbt.co.kr [2011.07.26] - CAM

static int justin_panel_enable_lcd(struct omap_dss_device *dssdev);
static void justin_panel_disable_lcd(struct omap_dss_device *dssdev);

/*--------------------------------------------------------------------------*/
static struct omap_dss_device justin_lcd_device = {
	.name = "lcd",
	.driver_name = "justin_panel",
/* 20100623 kyungtae.oh@lge.com justin use DSI MIPI [LGE_START]*/
	.type = OMAP_DISPLAY_TYPE_DSI,
// prime@sdcmicro.com Added missing elements to fix the problem with not enabled main clock for dss [START]
	.channel = OMAP_DSS_CHANNEL_LCD,
// prime@sdcmicro.com Added missing elements to fix the problem with not enabled main clock for dss [END]
// prime@sdcmicro.com Reconstructed the data structure to be more readable [START]
/* 20100623 kyungtae.oh@lge.com DSI setting [LGE_START]*/
	.phy.dsi		= {
		.clk_lane	= DSI_CLOCK_LANE,
		.clk_pol	= DSI_CLOCK_POLARITY,
		.data1_lane	= DSI_DATA0_LANE,
		.data1_pol	= DSI_DATA0_POLARITY,
		.data2_lane	= DSI_DATA1_LANE,
		.data2_pol	= DSI_DATA1_POLARITY,
		.ext_te		= 1,
		.ext_te_gpio	= 53, //LCD_TE : GPIO53
		.div		= {
			.lck_div	= 2,
			.pck_div	= 3,
			.regm		= 160,
			.regn		= 13,
			.regm_dispc	= 4,
			.regm_dsi	= 12,
			.lp_clk_div	= 3,
		},
	},
	.panel		= {
		.config	= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,
		.timings	= {
			.x_res	= 480,
			.y_res	= 800,
			.hsw		= 51,
			.hfp		= 14,
			.hbp		= 19,
			.vsw		= 8,
			.vfp		= 0,
			.vbp		= 22,
		},
	//LG_CHANGE_S lee.hyunji@lge.com 20110317	fixed LatinIME 
		.width_in_mm = 52,
		.height_in_mm = 86,
	//LG_CHANGE_E lee.hyunji@lge.com 20110317	fixed LatinIME 
	},
	.ctrl.pixel_size	= 24,
/* 20100623 kyungtae.oh@lge.com DSI setting [LGE_END]*/
// prime@sdcmicro.com Reconstructed the data structure to be more readable [END]
	.platform_enable = justin_panel_enable_lcd,
	.platform_disable = justin_panel_disable_lcd,
};


static int justin_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void justin_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}
static struct omap_dss_device justin_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = justin_panel_enable_tv,
	.platform_disable = justin_panel_disable_tv,
};

#ifdef CONFIG_OMAP2_DSS_HDMI	// changhoony.lee 2010-3-25
static void justin_hdmi_reset_enable(int level)
{
	/* Set GPIO_97 to high to pull SiI9022 HDMI transmitter out of reset
	* and low to disable it.
	*/
	gpio_request(HDMI_REG_EN_GPIO, "hdmi reset"); // == 2011.06.03 === hycho@ubiquix.com for warning delete
	gpio_direction_output(HDMI_REG_EN_GPIO, 1);
	gpio_set_value(HDMI_REG_EN_GPIO, level);
}

static int justin_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	int MaximumCount = 10;

	while ( hdmi_power_initialize==1 && MaximumCount-->0)
		msleep(100);

	hdmi_power_initialize =1;

	// CONTROL_PADCONF_DSS_PCLK
	omap_writel(omap_readl(0x480020D4) & ~0x7, 0x480020D4);
	
	extern void lp8720_reinit();
	lp8720_reinit();

//	gpio_direction_output(LP8720_ENABLE, 1);

	mdelay(20);

	//justin_panel_power_enable(1);
	justin_hdmi_reset_enable(1);
	mdelay(20);

//2010-5-8 changhoony.lee@lge.com sub-PMIC Power on

#if 0// buck?? for wt??
	subpm_set_output(SWREG,1);
	subpm_output_enable();
#endif
	
	subpm_set_output(LDO1,1);
	subpm_output_enable();

/* 20110531 comment by jslee@ubiquix.com
	subpm_set_output(LDO2,1);
	subpm_output_enable();
*/
	
	printk(KERN_INFO
		       "%s :: Enable_hdmi\n",__func__);
	msleep(20);

	return 0;
}

static void justin_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	int MaximumCount = 10;
	while ( hdmi_power_initialize==0 && MaximumCount-->0)
		msleep(100);

	MaximumCount = 10;

	extern int is_hdmi_nxp_driver_enabled(void);
	while(is_hdmi_nxp_driver_enabled() && MaximumCount-->0)
		msleep(100);
	/* prevent power off, before closing nxp driver.*/

	// CONTROL_PADCONF_DSS_PCLK
	omap_writel(omap_readl(0x480020D4) | 0x7, 0x480020D4);

	justin_hdmi_reset_enable(0);
//2010-5-8 changhoony.lee@lge.com sub-PMIC Power off
	subpm_set_output(LDO1,0);
	subpm_output_enable();

/* 20110531 comment by jslee@ubiquix.com
	subpm_set_output(LDO2,0);
	subpm_output_enable();
*/	
#if 0	
	subpm_set_output(SWREG,0);
	subpm_output_enable();
#endif
	msleep(20);

//	gpio_direction_output(LP8720_ENABLE, 0);

	msleep(20);
	printk(KERN_INFO
		       "%s :: Disable_hdmi\n",__func__);
	hdmi_power_initialize =0;

}

struct hdmi_platform_data justin_hdmi_data = {
#ifdef CONFIG_PM
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
#endif
};

static struct omap_dss_device justin_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.channel = OMAP_DSS_CHANNEL_LCD,
	.phy.dpi.data_lines = 24,
	.platform_enable = justin_panel_enable_hdmi,
	.platform_disable = justin_panel_disable_hdmi,
	.dev		= {
		.platform_data = &justin_hdmi_data,
	},
};
#endif	//#ifdef CONFIG_NXP_HDMI

static struct omap_dss_device *justin_dss_devices[] = {
	&justin_lcd_device,
	//&justin_tv_device, //20100623 kyungtae.oh@lge.com hub didn't use tv

#ifdef CONFIG_OMAP2_DSS_HDMI
	&justin_hdmi_device,
#endif

	
};

//20100928 kyungtae.oh@lge.com for id
//--[[ LGE_UBIQUIX_MODIFIED_START : hycho@ubiquix.com [2011.07.04] - Docomo merge
//extern unsigned get_last_off_on_transaction_id(struct device *dev);
static struct omap_dss_board_info justin_dss_data = {
//	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
//--[[ LGE_UBIQUIX_MODIFIED_END : hycho@ubiquix.com [2011.07.04] - Docomo merge
	.num_devices = ARRAY_SIZE(justin_dss_devices),
	.devices = justin_dss_devices,
	.default_device = &justin_lcd_device,
};

// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [START]
#if 0
static struct platform_device justin_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &justin_dss_data,
	},
};

static struct regulator_consumer_supply justin2_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &justin_dss_device.dev,
};

static struct regulator_consumer_supply justin_vdda_dac_supply = {
	.supply         = "vdda_dac",
	.dev            = &justin_dss_device.dev,
};

/*struct regulator_init_data justin_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &justin_vdda_dac_supply,
};*/

struct regulator_init_data justin2_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &justin2_vdds_dsi_supply,
};
#endif
// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [END]

static int justin_panel_power_enable(int enable)
{
#if 0
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEDICATED : 0,
				TWL4030_VPLL2_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEV_GRP : 0,
				TWL4030_VPLL2_DEV_GRP);
#endif
	return 0;
}

static int justin_panel_enable_lcd(struct omap_dss_device *dssdev)
{
#if 0		/* scchoi@ubiquix.com merge justin to black */
#if 1
	justin_lcd_initialize();
#else
	gpio_request(LCD_RESET_N, "lcd reset_n");
	gpio_direction_output(LCD_RESET_N, 1);
	mdelay(1);
	gpio_set_value(LCD_RESET_N, 0);
	mdelay(20);
	gpio_set_value(LCD_RESET_N, 1);
	mdelay(20);
#endif
#endif
	return 0;
}

static void justin_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_RESET_N, 0);
}

// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [START]
#if 0
struct platform_device *justin_devices[] __initdata = {
	&justin_dss_device,
	/*&justin_vout_device,*/
};

static struct omap2_mcspi_device_config justin_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */

};

struct spi_board_info justin_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "justin_disp_spi",
		.bus_num                = 1,
		.chip_select            = 2,
		.max_speed_hz           = 375000,
		.controller_data        = &justin_lcd_mcspi_config,
	},
};
#endif
// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [END]

void justin_lcd_tv_panel_init(void)
{
//	unsigned char lcd_panel_reset_gpio;
	omap_mux_init_signal("gpio_53", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpio_34", OMAP_PIN_OUTPUT);
//	omap_mux_init_signal("gpio_54", OMAP_PIN_OUTPUT);   //--[[ LGE_UBIQUIX_MODIFIED : jslee@ubiquix.com [2011.06.27] - j_froyo merge
#if 0
	gpio_request(LCD_RESET_N, "lcd reset_n");
	gpio_request(LCD_TE, "lcd te");
	gpio_request(LCD_CS, "lcd cs");
	gpio_direction_output(LCD_RESET_N, 1);
	gpio_direction_input(LCD_TE);
	gpio_direction_output(LCD_CS, 1);
#endif

#if 0
        omap_mux_init_signal("gpio_96", OMAP_PIN_OUTPUT);
        lcd_panel_reset_gpio = 96;

	omap_mux_init_signal("gpio_167", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_cs2", OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("gpio_94", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpio_96", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpio_56", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpio_95", OMAP_PIN_OUTPUT);

	gpio_request(lcd_panel_reset_gpio, "lcd reset");
	gpio_request(LCD_PANEL_QVGA_GPIO, "lcd qvga");
	gpio_request(TV_PANEL_ENABLE_GPIO, "tv enable");

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 0);
	gpio_direction_output(lcd_panel_reset_gpio, 0);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);
	gpio_direction_output(TV_PANEL_ENABLE_GPIO, 0);
	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 1);
	gpio_direction_output(lcd_panel_reset_gpio, 1);

	spi_register_board_info(justin_spi_board_info,
				ARRAY_SIZE(justin_spi_board_info));
#endif
//	printk("#####kyungtae########### justin_lcd_tv_panel");
// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [START]
#if 0
	platform_device_register(&justin_dss_device);
#else
	omap_display_init(&justin_dss_data);
#endif
// prime@sdcmicro.com Adapt display initialization code to 2.6.35 kernel [END]
}

#endif
// prime@sdcmicro.com Fix the compilation error in case of PANEL_HUB not being defined [END]

