/*
 * Hub DSI command mode panel
 *
 * Author: Kyungtae.oh <kyungtae.oh@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// prime@sdcmicro.com Reworked for 2.6.35 [START]

/*#define DEBUG*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <plat/display.h>

#define MODULE_NAME		"JUSTIN-LCD"

#ifndef DEBUG
//#define DEBUG
//#undef DEBUG
#endif

#ifdef DEBUG
#define DBG(fmt, args...) 				\
	printk(KERN_DEBUG "[%s] %s(%d): " 		\
		fmt, MODULE_NAME, __func__, __LINE__, ## args); 
#else	/* DEBUG */
#define DBG(...) 
#endif

#define GPIO_LCD_RESET_N		34

static int justin_dcs_read_1(enum omap_dsi_index ix, u8 dcs_cmd, u8 *data);
#ifdef DEBUG
static void read_status_reg(enum omap_dsi_index ix, u8* msg)
{
	u8 data[6];
	justin_dcs_read_1(ix, 0x0a, &data[0]);
	justin_dcs_read_1(ix, 0x0b, &data[1]);
	justin_dcs_read_1(ix, 0x0c, &data[2]);
	justin_dcs_read_1(ix, 0x0d, &data[3]);
	justin_dcs_read_1(ix, 0x0e, &data[4]);
	justin_dcs_read_1(ix, 0x0f, &data[5]);
	DBG("%s a:%02x b:%02x c:%02x d:%02x e:%02x f:%02x\n",msg, data[0], data[1], data[2], data[3], data[4], data[5]);
}
#else
#define	read_status_reg(ix, msg) while(0)
#endif

#define JUSTIN_LCD
/* DSI Virtual channel. Hardcoded for now. */
#define TCH 				0

#define DCS_READ_NUM_ERRORS		0x05
#define DCS_READ_POWER_MODE		0x0a
#define DCS_READ_MADCTL			0x0b
#define DCS_READ_PIXEL_FORMAT		0x0c
#define DCS_RDDSDR			0x0f
#define DCS_SLEEP_IN			0x10
#define DCS_SLEEP_OUT			0x11
#define DCS_DISPLAY_OFF			0x28
#define DCS_DISPLAY_ON			0x29
#define DCS_COLUMN_ADDR			0x2a
#define DCS_PAGE_ADDR			0x2b
#define DCS_MEMORY_WRITE		0x2c
#define DCS_TEAR_OFF			0x34
#define DCS_TEAR_ON			0x35
#define DCS_MEM_ACC_CTRL		0x36
#define DCS_PIXEL_FORMAT		0x3a
#define DCS_BRIGHTNESS			0x51
#define DCS_CTRL_DISPLAY		0x53
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
#define DCS_GET_ID1			0xf8	/*sunggyun.yu@lge.com for B-prj*/

/* #define HUB_USE_ESD_CHECK */
#define JUSTIN_ESD_CHECK_PERIOD	msecs_to_jiffies(5000)

extern int dsi_vc_write(enum omap_dsi_index ix, int channel, u8 cmd, u8 *data, int len);
static irqreturn_t justin_te_isr(int irq, void *data);
static void justin_te_timeout_work_callback(struct work_struct *work);
static int _justin_enable_te(struct omap_dss_device *dssdev, bool enable);
#ifdef HUB_USE_ESD_CHECK
static void justin_esd_work(struct work_struct *work);
#endif

static int no_lcd_flag = 0; // 20100901 taehwan.kim@lge.com Add detection for factory array test 

#define LONG_CMD_MIPI	0
#define SHORT_CMD_MIPI	1
#define END_OF_COMMAND	2
#define MIPI_DELAY_CMD  3

#define DSI_GEN_SHORTWRITE_NOPARAM 0x3
#define DSI_GEN_SHORTWRITE_1PARAM 0x13
#define DSI_GEN_SHORTWRITE_2PARAM 0x23
#define DSI_GEN_LONGWRITE 0x29
static int lcd_boot_status=1;
int lcd_off_boot=0;
extern u32 doing_wakeup; //LJH_TEST
#ifdef CONFIG_OMAP2_DSS_HDMI
extern bool HDMI_finalizing;
#endif
EXPORT_SYMBOL(lcd_off_boot);
EXPORT_SYMBOL(doing_wakeup);
static int __init nolcd_setup(char *unused)
{
	lcd_off_boot = 1;
	return 1;
}
__setup("nolcd", nolcd_setup);

static struct omap_dss_device *omapdssdev;

u8 lcd_command_for_mipi[][30] = {
#if 1 //KS_DEBUG
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM,0x02,0x03,0x00},												/* MIPI DSI config */
#endif

	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_1PARAM,0x01,0x20,},												/* Display Inversion */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_1PARAM,0x01,0x35,},		/* tearing effect  */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x03,0xB2, 0x00, 0xC8,},											/* Panel Characteristics Setting */
//	{MIPI_DELAY_CMD,5,}, //delay ms
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB3, 0x00,},									/* Panel Drive Setting */
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB4, 0x04,},									/* Display Mode Control */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xB5, 0x42, 0x10, 0x10, 0x00, 0x20 ,},						/* Display Control 1 */
#if 0	
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xB6, 0x0B, 0x0F, 0x03C, 0x18, 0x18, 0xE8,}, 				/* Display Control 2 */
#else	//HW Tuning TEST : natting 2010.1230
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xB6, 0x0B, 0x0F, 0x03C, 0x13, 0x13, 0xE8,}, 				/* Display Control 2 */
#endif	
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xB7, /*0x59*/0x49, 0x06, 0x1F, 0x00, 0x00 ,},	//jkr test, Send BTA
//	{MIPI_DELAY_CMD,5,}, //delay ms
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x03, 0xC0, 0x01, 0x11,}, 					/* oscilator setting */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x06, 0xC3, 0x07, 0x03, 0x04, 0x04, 0x04 ,},						/* Power Control 3 */
//	{MIPI_DELAY_CMD,100,}, //delay ms
#if 0
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xC4, 0x12, 0x24, 0x18, 0x18, 0x00, 0x49,}, 				/* Power Control 4 */
#else	//HW Tuning TEST : natting 2010.1230
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x07, 0xC4, 0x12, 0x24, 0x18, 0x18, 0x05 /* 0x04 -> 0x05 */, 0x49,}, 				/* Power Control 4 */
#endif
//	{MIPI_DELAY_CMD,100,}, //delay ms
	{SHORT_CMD_MIPI,DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xC5, 0x70 /* 0x63 -> 0x6B -> 0x70 */,},									/* Power Control 5 */
	{MIPI_DELAY_CMD,100,}, //delay ms
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x03,0xC6, 0x41, 0x63,},									/* Power Control 6 */
//	{MIPI_DELAY_CMD,100,}, //delay ms
#if 0
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD0, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Positive Gamma Curve for Red */
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD1, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Negative Gamma Curve for Red */
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD2, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Positive Gamma Curve for Green */
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD3, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Negative Gamma Curve for Green */
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD4, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Positive Gamma Curve for Blue */
  {LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD5, 0x03, 0x07, 0x73, 0x35, 0x00, 0x01, 0x20, 0x00, 0x03,}, /* Negative Gamma Curve for Blue */
#else	//HW Tuning TEST : natting 2010.1230
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD0, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Positive Gamma Curve for Red */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD1, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Negative Gamma Curve for Red */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD2, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Positive Gamma Curve for Green */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD3, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Negative Gamma Curve for Green */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD4, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Positive Gamma Curve for Blue */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE,0x0A,0xD5, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00, 0x30, 0x00, 0x03,}, /* Negative Gamma Curve for Blue */
#endif	
	{END_OF_COMMAND, },
};

struct justin_data {
	struct mutex lock;

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	bool enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;
	struct delayed_work te_timeout_work;
	bool use_ext_te;
	bool use_dsi_bl;

	bool cabc_broken;
	unsigned cabc_mode;

	bool intro_printed;

#ifdef HUB_USE_ESD_CHECK
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
#endif
	// LGE_UPDATE_S yoolje.cho@lge.com [[
	u8 gpio_lcd_reset_n;
	// LGE_UPDATE_E yoolje.cho@lge.com ]]
};

/** @brief  To check PIF connected for factory test mode
    @author taehwan.kim@lge.com
    @date   2010.09.03
    */
    
int dsi_vc_send_bta_sync(enum omap_dsi_index lcd_ix, int channel);
int dsi_vc_generic_write_short(int channel, u8 cmd, u8 *data, int len);
int dsi_vc_generic_write(int channel, u8 cmd, u8 *data, int len);
//jkr test start, Send BTA
int dsi_send_bta_only(struct omap_dss_device *dssdev);
void dsi_phy_config_4_bta(enum omap_dsi_index lcd_ix);
int dsi_vc_set_max_rx_packet_size_for_justin(enum omap_dsi_index lcd_ix, int channel, u16 len);
int dsi_DDR_CLK_ALWAYS_ON(enum omap_dsi_index lcd_ix,int enable);


/* B-Prj LCD update problem work around code [kyungyoon.kim@lge.com] 2010-12-20 */
static int lcd_status_check(enum omap_dsi_index ix)
{
	u8 data;
	justin_dcs_read_1(ix, 0x0a, &data);

	if (data==0x94)
	{
		DBG("LCD is Normal\n");
		return 0;
	}
	else
	{
		DBG("LCD is Abnormal\n");
		return 1;
	}
}
/* B-Prj LCD update problem work around code [kyungyoon.kim@lge.com] 2010-12-20 */

static void hw_guard_start(struct justin_data *td, int guard_msec)
{
	td->hw_guard_wait = msecs_to_jiffies(guard_msec);
	td->hw_guard_end = jiffies + td->hw_guard_wait;
}

static void hw_guard_wait(struct justin_data *td)
{
	unsigned long wait = td->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= td->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int justin_dcs_read_1(enum omap_dsi_index ix, u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];
	
	//jkr test start, nolcd
	if (lcd_off_boot == 1)
		return 0;
	//jkr test end, nolcd

	r = dsi_vc_dcs_read(ix, TCH, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int justin_dcs_write_0(enum omap_dsi_index ix, u8 dcs_cmd)
{
	//jkr test start, nolcd
	if (lcd_off_boot == 1)
		return 0;
	//jkr test end, nolcd
	return dsi_vc_dcs_write(ix, TCH, &dcs_cmd, 1);
}

static int justin_dcs_write_1(enum omap_dsi_index ix, u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	//jkr test start, nolcd
	if (lcd_off_boot == 1)
		return 0;
	//jkr test end, nolcd	
	return dsi_vc_dcs_write(ix, TCH, buf, 2);
}

static int justin_sleep_in(enum omap_dsi_index ix, struct justin_data *td)
{
	u8 cmd;
	int r;

#if 0//NATTING_TEST : i dun need u
	hw_guard_wait(td);
#endif

	cmd = DCS_SLEEP_IN;
	r = dsi_vc_dcs_write_nosync(ix, TCH, &cmd, 1);
	if (r)
		return r;

#if 0//NATTING_TEST : i dun need u
	hw_guard_start(td, 120);
#endif

	return 0;
}

static int justin_sleep_out(enum omap_dsi_index ix, struct justin_data *td)
{
	int r;

#if 0//NATTING_TEST : i dun need u
	hw_guard_wait(td);	
#endif

	r = justin_dcs_write_0(ix, DCS_SLEEP_OUT);
	if (r)
		return r;

#if 0//NATTING_TEST : we dun need u
	hw_guard_start(td, 120);	
#endif

	msleep(5);// for wt??

	return 0;
}

static int justin_get_id(enum omap_dsi_index ix, u8 *id1)
{
	int r;

	r = justin_dcs_read_1(ix, DCS_GET_ID1, id1);
	if (r)
		return r;

	return 0;
}

static int justin_set_addr_mode(enum omap_dsi_index ix, u8 rotate, bool mirror)
{
	int r;
	u8 mode;
	int b5, b6, b7;

	r = justin_dcs_read_1(ix, DCS_READ_MADCTL, &mode);
	if (r)
		return r;

	switch (rotate) {
	default:
	case 0:
		b7 = 0;
		b6 = 0;
		b5 = 0;
		break;
	case 1:
		b7 = 0;
		b6 = 1;
		b5 = 1;
		break;
	case 2:
		b7 = 1;
		b6 = 1;
		b5 = 0;
		break;
	case 3:
		b7 = 1;
		b6 = 0;
		b5 = 1;
		break;
	}

	if (mirror)
		b6 = !b6;

	mode &= ~((1<<7) | (1<<6) | (1<<5));
	mode |= (b7 << 7) | (b6 << 6) | (b5 << 5);

	return justin_dcs_write_1(ix, DCS_MEM_ACC_CTRL, mode);
}

static int justin_set_update_window(enum omap_dsi_index ix,
	u16 x, u16 y, u16 w, u16 h)
{
	int r = 0;
	u16 x1 = x;
	u16 x2 = x + w - 1;
	u16 y1 = y;
	u16 y2 = y + h - 1;

	u8 buf[5];
	buf[0] = DCS_COLUMN_ADDR;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(ix, TCH, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = DCS_PAGE_ADDR;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(ix, TCH, buf, sizeof(buf));
	if (r)
		return r;

	if(0) //KS_DEBUG
		dsi_vc_send_bta_sync(ix, TCH);

	return r;
}

static int justin_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	int level;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	//mutex_lock(&td->lock);

	if (td->use_dsi_bl) {
		if (td->enabled) {
			dsi_bus_lock(ix);
			r = justin_dcs_write_1(ix, DCS_BRIGHTNESS, level);
			dsi_bus_unlock(ix);
		} else {
			r = 0;
		}
	} else {
		if (!dssdev->set_backlight)
			r = -EINVAL;
		else
			r = dssdev->set_backlight(dssdev, level);
	}

	//mutex_unlock(&td->lock);

	return r;
}

static int justin_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static struct backlight_ops justin_bl_ops = {
	.get_brightness = justin_bl_get_intensity,
	.update_status  = justin_bl_update_status,
};

static void justin_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void justin_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int justin_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	if (timings->x_res != 480 || timings->y_res != 800)
		return -EINVAL;

	return 0;
}

static void justin_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->rotate == 0 || td->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static void justin_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

//	printk("[hycho] justin_framedone_cb\n");
	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(ix);
}

static irqreturn_t justin_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int old;
	int r;

	old = atomic_cmpxchg(&td->do_update, 1, 0);

	if (old) {
//		printk("[hycho] justin_te_isr = %d\n", old);		
		cancel_delayed_work(&td->te_timeout_work);

		r = omap_dsi_update(dssdev, TCH,
				td->update_region.x,
				td->update_region.y,
				td->update_region.w,
				td->update_region.h,
				justin_framedone_cb, dssdev);
		if (r)
			goto err;
	}

	return IRQ_HANDLED;
err:
	dev_err(&dssdev->dev, "start update failed\n");
	//dsi_bus_unlock(DSI1);
	return IRQ_HANDLED;
}

extern int lcd_backlight_status;
extern int lcd_status_backup;
extern  int lm3258_power_switch(int val);

static void justin_panel_init_lcd(struct omap_dss_device *dssdev)
{
	if(gpio_request(GPIO_LCD_RESET_N, "lcd gpio") < 0) 
  	{
		return;
	}
	gpio_direction_output(GPIO_LCD_RESET_N, 1);
	gpio_set_value(GPIO_LCD_RESET_N,0);
}

static void justin_hw_reset(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);

//	gpio_set_value(td->gpio_lcd_reset_n, 1);
//	mdelay(5);
	gpio_set_value(td->gpio_lcd_reset_n, 0);
	mdelay(1);
	gpio_set_value(td->gpio_lcd_reset_n, 1);
	mdelay(1);
}

extern void omap_pm_cpu_set_freq(unsigned long f);
int lcd_in_off_state = 0;
static int justin_power_on(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int i, r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if((doing_wakeup == 0)&&(lcd_boot_status==0))
	{
		omap_pm_cpu_set_freq(1000000000);	
		doing_wakeup = 1;
	}
	printk("DISPLAY ON start\n");

	lm3258_power_switch(1);
	lcd_status_backup = 1;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	if (lcd_boot_status== 0)
	{
		if (dssdev->platform_enable) 
		{
//			printk("[hycho] justin_power_on reset\n");
			gpio_request(34, "lcd reset_n");

			gpio_direction_output(34, 1);
			mdelay(1);
			gpio_set_value(34, 0);
			mdelay(10);
			gpio_set_value(34, 1);
			mdelay(10);
	//		r = dssdev->platform_enable(dssdev);
	//		if (r)	
	//		return r;
		}
	}

	if(lcd_boot_status== 1)
	{
//--[[ LGE_UBIQUIX_MODIFIED_START : hycho@ubiquix.com [2011.07.27] - HDMItoLCD dispaly problem
//		lcd_boot_status = 0;
//--]] LGE_UBIQUIX_MODIFIED_END : hycho@ubiquix.com [2011.07.27] - HDMItoLCD dispaly problem
	}

	/* it seems we have to wait a bit until cosmo_panel is ready */
	mdelay(5);	
	omapdss_dsi_vc_enable_hs(ix, 0, 0);	

	for (i = 0; lcd_command_for_mipi[i][0] != END_OF_COMMAND; i++) 
	{
		if(lcd_command_for_mipi[i][0]==MIPI_DELAY_CMD)
		{
				mdelay(lcd_command_for_mipi[i][1]);
		}
		else
		{
#if 1 //KS_DEBUG
			if(i==0)
			{	
				dsi_vc_write(ix,TCH,lcd_command_for_mipi[i][1],&lcd_command_for_mipi[i][3], lcd_command_for_mipi[i][2]);
			}
			else	
#endif
			{

				dsi_vc_dcs_write(ix, TCH, &lcd_command_for_mipi[i][3], lcd_command_for_mipi[i][2]);
			}
		}
	}

	mdelay(100);
//	printk("[hycho] justin_power_on 3\n");
#if 1 //temp, BTA rollback
	//jkr test start, Send BTA
	dsi_phy_config_4_bta(ix);
//	dsi_vc_set_max_rx_packet_size_for_justin(ix, dssdev->channel, 1);
	//jkr test end, Send BTA
//	printk("[hycho] justin_power_on 4\n");
#endif

	r = justin_sleep_out(ix, td);
	if (r)
		goto err;

//	printk("[hycho] justin_power_on 5\n");
	justin_dcs_write_1(ix, DCS_PIXEL_FORMAT, 0x7); /* 24bit/pixel */
	justin_dcs_write_0(ix, DCS_DISPLAY_ON);

	lcd_in_off_state = 1;

	td->enabled = 1;

	mdelay(100);
//	printk("[hycho] justin_power_on 6\n");
	omapdss_dsi_vc_enable_hs(ix, 0, 1);	

	printk("DISPLAY ON\n");

#ifndef NATTING_TEST
	printk("justin_panel_enable : DISPLAY ON DONE\n");
#endif
  
	return 0;
err:
//	if (lcd_off_boot) 
	printk("DISPLAY IS NOT SET"); //jkr test, nolcd
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	justin_hw_reset(dssdev);
	omapdss_dsi_display_disable(dssdev);
err0:
	return r;
}

static ssize_t justin_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	u8 errors;
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(ix);
		r = justin_dcs_read_1(ix, DCS_READ_NUM_ERRORS, &errors);
		dsi_bus_unlock(ix);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t justin_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	u8 id1;
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(ix);
//		r = justin_get_id(ix, &id1);
		dsi_bus_unlock(ix);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%02x\n", id1);
}

static const char *cabc_modes[] = {
	"off",		/* used also always when CABC is not supported */
	"ui",
	"still-image",
	"moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	int mode;
	int len;

	mode = td->cabc_mode;

	mode_str = "unknown";
	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];
	len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int i;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
		if (sysfs_streq(cabc_modes[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(cabc_modes))
		return -EINVAL;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(ix);
		if (!td->cabc_broken)
			justin_dcs_write_1(ix, DCS_WRITE_CABC, i);
		dsi_bus_unlock(ix);
	}

	td->cabc_mode = i;

	mutex_unlock(&td->lock);

	return count;
}

static ssize_t show_cabc_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	int i;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, justin_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, justin_hw_revision_show, NULL);
static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
		show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
		show_cabc_available_modes, NULL);

static struct attribute *justin_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	NULL,
};

static struct attribute_group justin_attr_group = {
	.attrs = justin_attrs,
};

static int justin_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct justin_data *td;
	struct backlight_device *bldev;
	int r;

	dev_dbg(&dssdev->dev, "probe\n");

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td) {
		r = -ENOMEM;
		goto err;
	}
	td->dssdev = dssdev;
	td->gpio_lcd_reset_n	= GPIO_LCD_RESET_N;

	mutex_init(&td->lock);

#ifdef HUB_USE_ESD_CHECK
	td->esd_wq = create_singlethread_workqueue("justin_esd");
	if (td->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&td->esd_work, justin_esd_work);
#endif

	dev_set_drvdata(&dssdev->dev, td);

  /*LG_CHANGE_S lee.hyunji@lge.com 20110223 LCD blinked in the middle of boot up a phone.*/
//	justin_panel_init_lcd(dssdev);
	omapdssdev = dssdev;
//	justin_hw_reset(dssdev);
  /*LG_CHANGE_E lee.hyunji@lge.com 20110223 LCD blinked in the middle of boot up a phone.*/

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	memset(&props, 0, sizeof(struct backlight_properties));
	if (!dssdev->set_backlight) {
		td->use_dsi_bl = true;
	}

	if (td->use_dsi_bl) {
		props.max_brightness = 255;
	}
	else {
		props.max_brightness = 127;
	}

	bldev = backlight_device_register(dssdev->name,
		       			  &dssdev->dev, 
					  dssdev, 
					  &justin_bl_ops, 
					  &props);

	if (IS_ERR(bldev)) {
		r = PTR_ERR(bldev);
		goto err_bl;
	}

	td->bldev = bldev;

	bldev->props.fb_blank	= FB_BLANK_UNBLANK;
	bldev->props.power	= FB_BLANK_UNBLANK;
	if (td->use_dsi_bl) {
		bldev->props.max_brightness = 255;		
		bldev->props.brightness = 255;
	}
	else {
		bldev->props.max_brightness = 127;
		bldev->props.brightness = 127;
	}

	justin_bl_update_status(bldev);

	if (dssdev->phy.dsi.ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;

		r = gpio_request(gpio, "justin irq");

		if (r) {
			dev_err(&dssdev->dev, "GPIO request failed\n");
			goto err_gpio;
		}

		gpio_direction_input(gpio);
		r = request_irq(OMAP_GPIO_IRQ(gpio), justin_te_isr, 
				IRQF_TRIGGER_RISING, "justin vsync", dssdev);

		if (r) {
			dev_err(&dssdev->dev, "IRQ request failed\n");
			gpio_free(gpio);
			goto err_irq;
		}

		INIT_DELAYED_WORK_DEFERRABLE(&td->te_timeout_work, justin_te_timeout_work_callback);
		
		dev_dbg(&dssdev->dev, "Using GPIO TE\n");
		td->use_ext_te = true;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &justin_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err_sysfs;
	}

/* LGE_CHANGE_S <sunggyun.yu@lge.com> 
 * the code location of enabled status must be at the end of function.*/
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD) {
		td->enabled = 1;
	}
#endif
/*LGE_CHANGE_E <sunggyun.yu@lge.com> */

	return 0;

err_sysfs:
	if (td->use_ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
	}
err_irq:
	if (td->use_ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		gpio_free(gpio);
	}
err_gpio:
	backlight_device_unregister(bldev);
err_bl:
#ifdef HUB_USE_ESD_CHECK
	destroy_workqueue(td->esd_wq);
err_wq:
#endif
	kfree(td);
err:
	return r;
}

//LG_CHANGE_S lee.hyunji@lge.com 20110317 	fixed LatinIME 
static void justin_get_dimension(struct omap_dss_device *dssdev,
		u32 *width, u32 *height)
{
	*width = dssdev->panel.width_in_mm;
	*height= dssdev->panel.height_in_mm;
}
//LG_CHANGE_E lee.hyunji@lge.com 20110317 	fixed LatinIME 

static void justin_remove(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &justin_attr_group);

	if (td->use_ext_te) {
		int gpio = dssdev->phy.dsi.ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	bldev = td->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	justin_bl_update_status(bldev);
	backlight_device_unregister(bldev);

#ifdef HUB_USE_ESD_CHECK
	cancel_delayed_work_sync(&td->esd_work);
	destroy_workqueue(td->esd_wq);
#endif

	/* reset, to be sure that the panel is in a valid state */
	justin_hw_reset(dssdev);

	kfree(td);
}

static void justin_power_off(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	read_status_reg(ix, "before disable");
	_justin_enable_te(dssdev, 0);

/*LGE_CHANGE_S <sunggyun.yu@lge.com> */
	if (!td->enabled)
		return;
/*LGE_CHANGE_E <sunggyun.yu@lge.com> */

	read_status_reg(ix, "after disable");

	r = justin_dcs_write_0(ix, DCS_DISPLAY_OFF);
	if (!r) {
		msleep(35);//B-prj
		r = justin_sleep_in(ix, td);
		lcd_in_off_state = 0; //LJH_TEST
		/* HACK: wait a bit so that the message goes through */
	msleep(20);
	}

	if (r) {
		dev_err(&dssdev->dev,
				"error disabling panel, issuing HW reset\n");
		justin_hw_reset(dssdev);
	}

	omapdss_dsi_display_disable(dssdev);

	td->enabled = 0;
}

static int justin_start(struct omap_dss_device *dssdev)
{
#ifdef HUB_USE_ESD_CHECK
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
#endif
	int r = 0;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dsi_bus_lock(ix);
	
	r = justin_power_on(dssdev);

	dsi_bus_unlock(ix);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
#ifdef HUB_USE_ESD_CHECK
		queue_delayed_work(td->esd_wq, &td->esd_work, JUSTIN_ESD_CHECK_PERIOD);
#endif
	}

	return r;
}

static void justin_stop(struct omap_dss_device *dssdev)
{
#ifdef HUB_USE_ESD_CHECK
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
#endif
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

#ifdef HUB_USE_ESD_CHECK
	cancel_delayed_work(&td->esd_work);
#endif

	dsi_bus_lock(ix);

	justin_power_off(dssdev);

	dsi_bus_unlock(ix);
}

static void justin_disable(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&td->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE ||
	    dssdev->state == OMAP_DSS_DISPLAY_TRANSITION)
		justin_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&td->lock);

	if(lcd_backlight_status == 0)
	{
		lm3258_power_switch(0);
	}
	lcd_status_backup = 0;
}

static int justin_suspend(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev = td->bldev;
	int r = 0;

	dev_dbg(&dssdev->dev, "suspend\n");

	mutex_lock(&td->lock);

	bldev->props.power = FB_BLANK_POWERDOWN;
	justin_bl_update_status(bldev);
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}
	justin_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
err:
	mutex_unlock(&td->lock);
	return r;
}

static int justin_enable(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

#ifdef CONFIG_OMAP2_DSS_HDMI//NATTING_TEMP
    // shouldn't reset LCD while LCD ON(BL ON)
    //if (cur_main_lcd_level && !lcd_boot_status) {
    if (HDMI_finalizing){
      //printk("%s::Panel is already enabled ::Should Return here\n", __func__);
      printk("%s::HDMI is finalizing ::dun have to init LCD again..MUST NOT actually \n", __func__);
      HDMI_finalizing = 0;
      return 0;
    }
#endif

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}
	r = justin_start(dssdev);
err:
	mutex_unlock(&td->lock);
	return r;
}

static int justin_resume(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev = td->bldev;
	int r = 0;

	dev_dbg(&dssdev->dev, "resume\n");

	lm3258_power_switch(1);
	
	mutex_lock(&td->lock);

	bldev->props.power = FB_BLANK_UNBLANK;
	justin_bl_update_status(bldev);
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}
	justin_hw_reset(dssdev); /* 20110813, mschung@ubiquix.com, LCD need to do hw_reset at resume. Some board's LCD does not wake-up without hw_reset. */	
	r = justin_start(dssdev);
err:
	mutex_unlock(&td->lock);
	return r;
}

static void justin_te_timeout_work_callback(struct work_struct *work)
{
	struct justin_data *td = container_of(work, struct justin_data,
					te_timeout_work.work);
	struct omap_dss_device *dssdev = td->dssdev;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dev_err(&dssdev->dev, "TE not received for 250ms!\n");

	atomic_set(&td->do_update, 0);

	//dsi_bus_unlock(ix);
}

static int justin_update_locked(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

//	printk("[hycho] justin_update_locked IN\n");

	WARN_ON(!dsi_bus_is_locked(ix));

	if (!td->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	r = justin_set_update_window(ix, x, y, w, h);
	if (r)
		goto err;

#if 0 // For test
	if (td->te_enabled && td->use_ext_te) {
		td->update_region.x = x;
		td->update_region.y = y;
		td->update_region.w = w;
		td->update_region.h = h;
		barrier();
		schedule_delayed_work(&td->te_timeout_work,
				msecs_to_jiffies(250));
		atomic_set(&td->do_update, 1);
	} else {
		/* We use VC(1) for VideoPort Data and VC(0) for L4 data */
		if (cpu_is_omap44xx())
			r = omap_dsi_update(dssdev, 1, x, y, w, h,
				justin_framedone_cb, dssdev);
		else
			r = omap_dsi_update(dssdev, TCH, x, y, w, h,
				justin_framedone_cb, dssdev);
		if (r)
			goto err;
	}
#else
	dsi_DDR_CLK_ALWAYS_ON(0,1); //jkr test, Send BTA
	
	r = omap_dsi_update(dssdev, TCH, x, y, w, h,
		justin_framedone_cb, dssdev);
	if (r)
		goto err;
#endif

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&td->lock);

//	printk("[hycho] justin_update_locked OUT\n");
	return 0;
err:
	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);
	return r;
}

static int justin_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	enum omap_dsi_index ix;
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);
	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

//	printk("[hycho] justin_update IN\n");
	mutex_lock(&td->lock);

	/* mark while waiting on bus so delayed update will not call update */
	dssdev->sched_update.waiting = true;
	dsi_bus_lock(ix);
	dssdev->sched_update.waiting = false;
//	printk("[hycho] justin_update OUT\n");
	return justin_update_locked(dssdev, x, y, w, h);
}

static int justin_sched_update(struct omap_dss_device *dssdev,
					u16 x, u16 y, u16 w, u16 h)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	if (mutex_trylock(&td->lock)) {
		r = omap_dsi_sched_update_lock(dssdev, x, y, w, h, false);

		if (!r) {
			/* start the update now */
			r = justin_update_locked(dssdev, x, y, w, h);

			return r;
		}

		if (r == -EBUSY)
			r = 0;

		mutex_unlock(&td->lock);

	} else {
		/* this locks dsi bus if it can and returns 0 */
		r = omap_dsi_sched_update_lock(dssdev, x, y, w, h, true);

		if (r == -EBUSY)
			r = 0;
	}

	return r;
}

static int justin_sync(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dev_dbg(&dssdev->dev, "sync\n");

  /*LG_CHANGE_S lee.hyunji@lge.com 20110316 when press power key in order to turn on/off the LCD, LCD can not be turned on.*/
#if 0
	mutex_lock(&td->lock);
	dsi_bus_lock(ix);
	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);
#endif
  /*LG_CHANGE_E lee.hyunji@lge.com 20110316 when press power key in order to turn on/off the LCD, LCD can not be turned on.*/

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int _justin_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r = 0;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (enable) {
		if (!td->te_enabled && td->enabled) {
//			r = justin_dcs_write_1(ix, DCS_TEAR_ON, 1);
			r = justin_dcs_write_1(ix, DCS_TEAR_ON, 0);
		}
	} else {
		if (td->te_enabled)
			r = justin_dcs_write_0(ix, DCS_TEAR_OFF);
	}

	if (!td->use_ext_te)
		omapdss_dsi_enable_te(dssdev, enable);

	msleep(50);

	return r;
}

static int justin_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&td->lock);

	printk("justin_enable_te IN\n");

	if (td->te_enabled == enable)
		goto end;

	dsi_bus_lock(ix);
	if (td->enabled) {
		r = _justin_enable_te(dssdev, enable);
		if (r)
			goto err;
	}

	td->te_enabled = enable;

	printk("justin_enable_te OUT\n");
	dsi_bus_unlock(ix);
end:
	mutex_unlock(&td->lock);

	return 0;
err:
	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);

	return r;
}

static int justin_get_te(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->te_enabled;
	mutex_unlock(&td->lock);

	return r;
}

static int justin_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dev_dbg(&dssdev->dev, "rotate %d\n", rotate);

	mutex_lock(&td->lock);

	if (td->rotate == rotate)
		goto end;

	dsi_bus_lock(ix);

	if (td->enabled) {
		r = justin_set_addr_mode(ix, rotate, td->mirror);

		if (r)
			goto err;
	}

	td->rotate = rotate;

	dsi_bus_unlock(ix);
end:
	mutex_unlock(&td->lock);
	return 0;
err:
	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);
	return r;
}

static u8 justin_get_rotate(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->rotate;
	mutex_unlock(&td->lock);

	return r;
}

static int justin_mirror(struct omap_dss_device *dssdev, bool enable)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	dev_dbg(&dssdev->dev, "mirror %d\n", enable);

	mutex_lock(&td->lock);

	if (td->mirror == enable)
		goto end;

	dsi_bus_lock(ix);
	if (td->enabled) {
		r = justin_set_addr_mode(ix, td->rotate, enable);
		if (r)
			goto err;
	}

	td->mirror = enable;

	dsi_bus_unlock(ix);
end:
	mutex_unlock(&td->lock);
	return 0;
err:
	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);
	return r;
}

static bool justin_get_mirror(struct omap_dss_device *dssdev)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->mirror;
	mutex_unlock(&td->lock);

	return r;
}

static int justin_run_test(struct omap_dss_device *dssdev, int test_num)
{
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	u8 id1;
	int r;
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&td->lock);

	if (!td->enabled) {
		r = -ENODEV;
		goto err1;
	}

	dsi_bus_lock(ix);

	r = justin_dcs_read_1(ix, DCS_GET_ID1, &id1);
	if (r)
		goto err2;

	dsi_bus_unlock(ix);
	mutex_unlock(&td->lock);
	return 0;
err2:
	dsi_bus_unlock(ix);
err1:
	mutex_unlock(&td->lock);
	return r;
}

static int justin_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;
	struct justin_data *td = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index ix;

	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	if (size < w * h * 3)
		return -ENOMEM;

	mutex_lock(&td->lock);

	if (!td->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * 3,
			dssdev->panel.timings.x_res *
			dssdev->panel.timings.y_res * 3);

	dsi_bus_lock(ix);

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	 * use short packets. plen 32 works, but bigger packets seem to cause
	 * an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	justin_set_update_window(ix, x, y, w, h);

	r = dsi_vc_set_max_rx_packet_size(ix, TCH, plen);
	if (r)
		goto err2;

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = dsi_vc_dcs_read(ix, TCH, dcs_cmd,
				buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(&dssdev->dev, "read error\n");
			goto err3;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&dssdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
					"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}

	r = buf_used;

err3:
	dsi_vc_set_max_rx_packet_size(ix, TCH, 1);
err2:
	dsi_bus_unlock(ix);
err1:
	mutex_unlock(&td->lock);
	return r;
}

#ifdef HUB_USE_ESD_CHECK
static void justin_esd_work(struct work_struct *work)
{
	struct justin_data *td = container_of(work, struct justin_data,
			esd_work.work);
	struct omap_dss_device *dssdev = td->dssdev;
	u8 state1, state2;
	int r;
	enum omap_dsi_index ix;
	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&td->lock);

	if (!td->enabled) {
		mutex_unlock(&td->lock);
		return;
	}

	dsi_bus_lock(ix);

	r = justin_dcs_read_1(ix, DCS_RDDSDR, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read Hub status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = justin_sleep_out(ix, td);
	if (r) {
		dev_err(&dssdev->dev, "failed to run Hub self-diagnostics\n");
		goto err;
	}

	r = justin_dcs_read_1(ix, DCS_RDDSDR, &state2);
	if (r) {
		dev_err(&dssdev->dev, "failed to read Hub status\n");
		goto err;
	}

	/* Each sleep out command will trigger a self diagnostic and flip
	 * Bit6 if the test passes.
	 */
	if (!((state1 ^ state2) & (1 << 6))) {
		dev_err(&dssdev->dev, "LCD self diagnostics failed\n");
		goto err;
	}
	/* Self-diagnostics result is also shown on TE GPIO line. We need
	 * to re-enable TE after self diagnostics */
	if (td->use_ext_te && td->te_enabled) {
		r = _justin_enable_te(dssdev, true);
		if (r)
			goto err;
	}

	dsi_bus_unlock(ix);

	queue_delayed_work(td->esd_wq, &td->esd_work, JUSTIN_ESD_CHECK_PERIOD);

	mutex_unlock(&td->lock);
	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	justin_power_off(dssdev);
	justin_hw_reset(dssdev);
	justin_power_on(dssdev);

	dsi_bus_unlock(ix);

	queue_delayed_work(td->esd_wq, &td->esd_work, JUSTIN_ESD_CHECK_PERIOD);

	mutex_unlock(&td->lock);
}
#endif

static int justin_set_update_mode(struct omap_dss_device *dssdev,
			       enum omap_dss_update_mode mode)
{
	if (mode != OMAP_DSS_UPDATE_MANUAL) {
		return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode 
justin_get_update_mode(struct omap_dss_device *dssdev)
{
	return OMAP_DSS_UPDATE_MANUAL;
}

static struct omap_dss_driver justin_driver = {
	.probe		= justin_probe,
	.remove		= justin_remove,

	.enable		= justin_enable,
	.disable	= justin_disable,
	.suspend	= justin_suspend,
	.resume		= justin_resume,

	.set_update_mode = justin_set_update_mode,
	.get_update_mode = justin_get_update_mode,

	.update		= justin_update,
	.sched_update	= justin_sched_update,
	.sync		= justin_sync,

	.get_resolution	= justin_get_resolution,

	.enable_te	= justin_enable_te,
	.get_te		= justin_get_te,

	.set_rotate	= justin_rotate,
	.get_rotate	= justin_get_rotate,
	.set_mirror	= justin_mirror,
	.get_mirror	= justin_get_mirror,
	.run_test	= justin_run_test,
	.memory_read	= justin_memory_read,

	.get_timings	= justin_get_timings,
	.set_timings	= justin_set_timings,
	.check_timings	= justin_check_timings,
	//LG_CHANGE_S lee.hyunji@lge.com 20110317	fixed LatinIME 
//	.get_dimension = justin_get_dimension,  
	//LG_CHANGE_E lee.hyunji@lge.com 20110317	fixed LatinIME 

	.driver         = {
		.name   = "justin_panel",	// 20110621 jslee@ubiquix.com hub_panel --> justin_panel
		.owner  = THIS_MODULE,
	},
};
int check_no_lcd(void)
{
    //printk("check_no_lcd for test mode = %d \n",no_lcd_flag);
    return lcd_off_boot; //ntyeongon.moon 20110404  changed => no_lcd_flag
}
EXPORT_SYMBOL(check_no_lcd);

static int __init justin_init(void)
{
	omap_dss_register_driver(&justin_driver);
	return 0;
}

static void __exit justin_exit(void)
{
	omap_dss_unregister_driver(&justin_driver);
}

module_init(justin_init);
module_exit(justin_exit);

MODULE_AUTHOR("kyungtae Oh <kyungtae.oh@lge.com>");
MODULE_DESCRIPTION("HUB Driver");
MODULE_LICENSE("GPL");
// prime@sdcmicro.com Reworked for 2.6.35 [END]
