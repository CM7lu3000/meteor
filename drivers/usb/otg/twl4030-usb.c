/*
 * twl4030_usb - TWL4030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 *	- HS USB ULPI mode works.
 *	- 3-pin mode support may be added in future.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#if defined(CONFIG_MACH_LGE_OMAP3)
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/wakelock.h> 
#endif // defined(CONFIG_MACH_LGE_OMAP3)



/* Register defines */

#define VENDOR_ID_LO			0x00
#define VENDOR_ID_HI			0x01
#define PRODUCT_ID_LO			0x02
#define PRODUCT_ID_HI			0x03

#define FUNC_CTRL			0x04
#define FUNC_CTRL_SET			0x05
#define FUNC_CTRL_CLR			0x06
#define FUNC_CTRL_SUSPENDM		(1 << 6)
#define FUNC_CTRL_RESET			(1 << 5)
#define FUNC_CTRL_OPMODE_MASK		(3 << 3) /* bits 3 and 4 */
#define FUNC_CTRL_OPMODE_NORMAL		(0 << 3)
#define FUNC_CTRL_OPMODE_NONDRIVING	(1 << 3)
#define FUNC_CTRL_OPMODE_DISABLE_BIT_NRZI	(2 << 3)
#define FUNC_CTRL_TERMSELECT		(1 << 2)
#define FUNC_CTRL_XCVRSELECT_MASK	(3 << 0) /* bits 0 and 1 */
#define FUNC_CTRL_XCVRSELECT_HS		(0 << 0)
#define FUNC_CTRL_XCVRSELECT_FS		(1 << 0)
#define FUNC_CTRL_XCVRSELECT_LS		(2 << 0)
#define FUNC_CTRL_XCVRSELECT_FS4LS	(3 << 0)

#define IFC_CTRL			0x07
#define IFC_CTRL_SET			0x08
#define IFC_CTRL_CLR			0x09
#define IFC_CTRL_INTERFACE_PROTECT_DISABLE	(1 << 7)
#define IFC_CTRL_AUTORESUME		(1 << 4)
#define IFC_CTRL_CLOCKSUSPENDM		(1 << 3)
#define IFC_CTRL_CARKITMODE		(1 << 2)
#define IFC_CTRL_FSLSSERIALMODE_3PIN	(1 << 1)

#define TWL4030_OTG_CTRL		0x0A
#define TWL4030_OTG_CTRL_SET		0x0B
#define TWL4030_OTG_CTRL_CLR		0x0C
#define TWL4030_OTG_CTRL_DRVVBUS	(1 << 5)
#define TWL4030_OTG_CTRL_CHRGVBUS	(1 << 4)
#define TWL4030_OTG_CTRL_DISCHRGVBUS	(1 << 3)
#define TWL4030_OTG_CTRL_DMPULLDOWN	(1 << 2)
#define TWL4030_OTG_CTRL_DPPULLDOWN	(1 << 1)
#define TWL4030_OTG_CTRL_IDPULLUP	(1 << 0)

#define USB_INT_EN_RISE			0x0D
#define USB_INT_EN_RISE_SET		0x0E
#define USB_INT_EN_RISE_CLR		0x0F
#define USB_INT_EN_FALL			0x10
#define USB_INT_EN_FALL_SET		0x11
#define USB_INT_EN_FALL_CLR		0x12
#define USB_INT_STS			0x13
#define USB_INT_LATCH			0x14
#define USB_INT_IDGND			(1 << 4)
#define USB_INT_SESSEND			(1 << 3)
#define USB_INT_SESSVALID		(1 << 2)
#define USB_INT_VBUSVALID		(1 << 1)
#define USB_INT_HOSTDISCONNECT		(1 << 0)

#define CARKIT_CTRL			0x19
#define CARKIT_CTRL_SET			0x1A
#define CARKIT_CTRL_CLR			0x1B
#define CARKIT_CTRL_MICEN		(1 << 6)
#define CARKIT_CTRL_SPKRIGHTEN		(1 << 5)
#define CARKIT_CTRL_SPKLEFTEN		(1 << 4)
#define CARKIT_CTRL_RXDEN		(1 << 3)
#define CARKIT_CTRL_TXDEN		(1 << 2)
#define CARKIT_CTRL_IDGNDDRV		(1 << 1)
#define CARKIT_CTRL_CARKITPWR		(1 << 0)
#define CARKIT_PLS_CTRL			0x22
#define CARKIT_PLS_CTRL_SET		0x23
#define CARKIT_PLS_CTRL_CLR		0x24
#define CARKIT_PLS_CTRL_SPKRRIGHT_BIASEN	(1 << 3)
#define CARKIT_PLS_CTRL_SPKRLEFT_BIASEN	(1 << 2)
#define CARKIT_PLS_CTRL_RXPLSEN		(1 << 1)
#define CARKIT_PLS_CTRL_TXPLSEN		(1 << 0)

#define MCPC_CTRL			0x30
#define MCPC_CTRL_SET			0x31
#define MCPC_CTRL_CLR			0x32
#define MCPC_CTRL_RTSOL			(1 << 7)
#define MCPC_CTRL_EXTSWR		(1 << 6)
#define MCPC_CTRL_EXTSWC		(1 << 5)
#define MCPC_CTRL_VOICESW		(1 << 4)
#define MCPC_CTRL_OUT64K		(1 << 3)
#define MCPC_CTRL_RTSCTSSW		(1 << 2)
#define MCPC_CTRL_HS_UART		(1 << 0)

#define MCPC_IO_CTRL			0x33
#define MCPC_IO_CTRL_SET		0x34
#define MCPC_IO_CTRL_CLR		0x35
#define MCPC_IO_CTRL_MICBIASEN		(1 << 5)
#define MCPC_IO_CTRL_CTS_NPU		(1 << 4)
#define MCPC_IO_CTRL_RXD_PU		(1 << 3)
#define MCPC_IO_CTRL_TXDTYP		(1 << 2)
#define MCPC_IO_CTRL_CTSTYP		(1 << 1)
#define MCPC_IO_CTRL_RTSTYP		(1 << 0)

#define MCPC_CTRL2			0x36
#define MCPC_CTRL2_SET			0x37
#define MCPC_CTRL2_CLR			0x38
#define MCPC_CTRL2_MCPC_CK_EN		(1 << 0)

#define OTHER_FUNC_CTRL			0x80
#define OTHER_FUNC_CTRL_SET		0x81
#define OTHER_FUNC_CTRL_CLR		0x82
#define OTHER_FUNC_CTRL_BDIS_ACON_EN	(1 << 4)
#define OTHER_FUNC_CTRL_FIVEWIRE_MODE	(1 << 2)

#define OTHER_IFC_CTRL			0x83
#define OTHER_IFC_CTRL_SET		0x84
#define OTHER_IFC_CTRL_CLR		0x85
#define OTHER_IFC_CTRL_OE_INT_EN	(1 << 6)
#define OTHER_IFC_CTRL_CEA2011_MODE	(1 << 5)
#define OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN	(1 << 4)
#define OTHER_IFC_CTRL_HIZ_ULPI_60MHZ_OUT	(1 << 3)
#define OTHER_IFC_CTRL_HIZ_ULPI		(1 << 2)
#define OTHER_IFC_CTRL_ALT_INT_REROUTE	(1 << 0)

#define OTHER_INT_EN_RISE		0x86
#define OTHER_INT_EN_RISE_SET		0x87
#define OTHER_INT_EN_RISE_CLR		0x88
#define OTHER_INT_EN_FALL		0x89
#define OTHER_INT_EN_FALL_SET		0x8A
#define OTHER_INT_EN_FALL_CLR		0x8B
#define OTHER_INT_STS			0x8C
#define OTHER_INT_LATCH			0x8D
#define OTHER_INT_VB_SESS_VLD		(1 << 7)
#define OTHER_INT_DM_HI			(1 << 6) /* not valid for "latch" reg */
#define OTHER_INT_DP_HI			(1 << 5) /* not valid for "latch" reg */
#define OTHER_INT_BDIS_ACON		(1 << 3) /* not valid for "fall" regs */
#define OTHER_INT_MANU			(1 << 1)
#define OTHER_INT_ABNORMAL_STRESS	(1 << 0)

#define ID_STATUS			0x96
#define ID_RES_FLOAT			(1 << 4)
#define ID_RES_440K			(1 << 3)
#define ID_RES_200K			(1 << 2)
#define ID_RES_102K			(1 << 1)
#define ID_RES_GND			(1 << 0)

#define POWER_CTRL			0xAC
#define POWER_CTRL_SET			0xAD
#define POWER_CTRL_CLR			0xAE
#define POWER_CTRL_OTG_ENAB		(1 << 5)

#define OTHER_IFC_CTRL2			0xAF
#define OTHER_IFC_CTRL2_SET		0xB0
#define OTHER_IFC_CTRL2_CLR		0xB1
#define OTHER_IFC_CTRL2_ULPI_STP_LOW	(1 << 4)
#define OTHER_IFC_CTRL2_ULPI_TXEN_POL	(1 << 3)
#define OTHER_IFC_CTRL2_ULPI_4PIN_2430	(1 << 2)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT1N	(0 << 0)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT2N	(1 << 0)

#define REG_CTRL_EN			0xB2
#define REG_CTRL_EN_SET			0xB3
#define REG_CTRL_EN_CLR			0xB4
#define REG_CTRL_ERROR			0xB5
#define ULPI_I2C_CONFLICT_INTEN		(1 << 0)

#define OTHER_FUNC_CTRL2		0xB8
#define OTHER_FUNC_CTRL2_SET		0xB9
#define OTHER_FUNC_CTRL2_CLR		0xBA
#define OTHER_FUNC_CTRL2_VBAT_TIMER_EN	(1 << 0)

/* following registers do not have separate _clr and _set registers */
#define VBUS_DEBOUNCE			0xC0
#define ID_DEBOUNCE			0xC1
#define VBAT_TIMER			0xD3
#define PHY_PWR_CTRL			0xFD
#define PHY_PWR_PHYPWD			(1 << 0)
#define PHY_CLK_CTRL			0xFE
#define PHY_CLK_CTRL_CLOCKGATING_EN	(1 << 2)
#define PHY_CLK_CTRL_CLK32K_EN		(1 << 1)
#define REQ_PHY_DPLL_CLK		(1 << 0)
#define PHY_CLK_CTRL_STS		0xFF
#define PHY_DPLL_CLK			(1 << 0)

/* In module TWL4030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E
#define STS_HW_CONDITIONS		0x0F

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1			0x7D
#define VUSB_DEDICATED2			0x7E
#define VUSB1V5_DEV_GRP			0x71
#define VUSB1V5_TYPE			0x72
#define VUSB1V5_REMAP			0x73
#define VUSB1V8_DEV_GRP			0x74
#define VUSB1V8_TYPE			0x75
#define VUSB1V8_REMAP			0x76
#define VUSB3V1_DEV_GRP			0x77
#define VUSB3V1_TYPE			0x78
#define VUSB3V1_REMAP			0x79

/* In module TWL4030_MODULE_INTBR */
#define PMBR1				0x0D
#define GPIO_USB_4PIN_ULPI_2430C	(3 << 0)

void check_usb_reg(void); // by TI Prakash
struct twl4030_usb *t2_transceiver;
//20101006 taehwan.kim@lge.com To  detect USB or charger on VBUS irq [START_LGE]
int get_muic_mode();
//20101006 taehwan.kim@lge.com To  detect USB or charger on VBUS irq [END_LGE]
/* LGE_CHANGE_START 2011-03-16 kenneth.kang@lge.com patch for Adb offline set and Mass Storage Driver detecting fail */    
void vbus_irq_muic_handler(int state);
/* LGE_CHANGE_END 2011-03-16 kenneth.kang@lge.com */

static irqreturn_t twl4030_usb_irq(int irq, void *_twl);

// LGE_CHANGE wake lock for usb connection
struct wlock {
	int wake_lock_on;
	int usb_connected;
	struct wake_lock wake_lock;
};
static struct wlock the_wlock;
static struct wlock the_wlock2;
// LGE_CHANGE wake lock for usb connection

// LGE_CHANGE work queue
static struct delayed_work twl4030_usb_wq;
static struct delayed_work twl4030_usb_wq2;
// LGE_CHANGE work queue

// LGE_CHANGE_S [daewung.kim@lge.com] 2010-12-29, USB interrupt failure at suspend state
extern int suspend_resume_statecheck;
// LGE_CHANGE_E [daewung.kim@lge.com] 2010-12-29, USB interrupt failure at suspend state
int muic_mode_final = 0; // LGE_Change

#if defined(CONFIG_MACH_LGE_OMAP3)
extern void musb_link_force_active(int enable);
#endif // defined(CONFIG_MACH_LGE_OMAP3)

struct twl4030_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* TWL4030 internal USB regulator supplies */
	struct regulator	*usb1v5;
	struct regulator	*usb1v8;
#if 0	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-11-27, for <USB interrupt BUG fix> */
	struct regulator	*usb3v1;
#endif /* CONFIG_MACH_LGE_HEAVEN */

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	/* pin configuration */
	enum twl4030_usb_mode	usb_mode;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool			irq_enabled;
};

// LGE_UPDATE jaejoong.kim 
//static struct wake_lock usb_lock;

#if 0 /* mbk_wake mbk_temp */ 
// LGE_CHANGE wake lock for usb connection
struct wlock {
	int wake_lock_on;
	struct wake_lock wake_lock;
};
static struct wlock the_wlock;
// LGE_CHANGE wake lock for usb connection

// LGE_CHANGE work queue
static struct delayed_work twl4030_usb_wq;
// LGE_CHANGE work queue
#endif 

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct twl4030_usb, otg);

/*-------------------------------------------------------------------------*/

static int twl4030_i2c_write_u8_verify(struct twl4030_usb *twl,
		u8 module, u8 data, u8 address)
{
	u8 check = 0;

	if ((twl_i2c_write_u8(module, data, address) >= 0) &&
	    (twl_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			1, module, address, check, data);

	/* Failed once: Try again */
	if ((twl_i2c_write_u8(module, data, address) >= 0) &&
	    (twl_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			2, module, address, check, data);

	/* Failed again: Return error */
	return -EBUSY;
}

#define twl4030_usb_write_verify(twl, address, data)	\
	twl4030_i2c_write_u8_verify(twl, TWL4030_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(struct twl4030_usb *twl,
		u8 address, u8 data)
{
	int ret = 0;

	ret = twl_i2c_write_u8(TWL4030_MODULE_USB, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl4030_readb(struct twl4030_usb *twl, u8 module, u8 address)
{
	u8 data = 0;
	int ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(twl->dev,
			"TWL4030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

static inline int twl4030_usb_read(struct twl4030_usb *twl, u8 address)
{
	return twl4030_readb(twl, TWL4030_MODULE_USB, address);
}

/*-------------------------------------------------------------------------*/

static inline int
twl4030_usb_set_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, ULPI_SET(reg), bits);
}

static inline int
twl4030_usb_clear_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, ULPI_CLR(reg), bits);
}

/*-------------------------------------------------------------------------*/

static enum usb_xceiv_events twl4030_usb_linkstat(struct twl4030_usb *twl)
{
	int	status;
	int	linkstat = USB_EVENT_NONE;

	/*
	 * For ID/VBUS sensing, see manual section 15.4.8 ...
	 * except when using only battery backup power, two
	 * comparators produce VBUS_PRES and ID_PRES signals,
	 * which don't match docs elsewhere.  But ... BIT(7)
	 * and BIT(2) of STS_HW_CONDITIONS, respectively, do
	 * seem to match up.  If either is true the USB_PRES
	 * signal is active, the OTG module is activated, and
	 * its interrupt may be raised (may wake the system).
	 */
	status = twl4030_readb(twl, TWL4030_MODULE_PM_MASTER,
			STS_HW_CONDITIONS);
	if (status < 0)
		dev_err(twl->dev, "USB link status err %d\n", status);
	else if (status & (BIT(7) | BIT(2))) {
		if (status & BIT(2))
			linkstat = USB_EVENT_ID;
		else
			linkstat = USB_EVENT_VBUS;
	} else
		linkstat = USB_EVENT_NONE;

	dev_dbg(twl->dev, "HW_CONDITIONS 0x%02x/%d; link %d\n",
			status, status, linkstat);

	/* REVISIT this assumes host and peripheral controllers
	 * are registered, and that both are active...
	 */

	spin_lock_irq(&twl->lock);
	twl->linkstat = linkstat;
	if (linkstat == USB_EVENT_ID) {
		twl->otg.default_a = true;
		twl->otg.state = OTG_STATE_A_IDLE;
	} else {
		twl->otg.default_a = false;
		twl->otg.state = OTG_STATE_B_IDLE;
	}
	spin_unlock_irq(&twl->lock);

	return linkstat;
}

static void twl4030_usb_set_mode(struct twl4030_usb *twl, int mode)
{
	twl->usb_mode = mode;

	switch (mode) {
	case T2_USB_MODE_ULPI:
		twl4030_usb_clear_bits(twl, ULPI_IFC_CTRL,
					ULPI_IFC_CTRL_CARKITMODE);
		twl4030_usb_set_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);
		twl4030_usb_clear_bits(twl, ULPI_FUNC_CTRL,
					ULPI_FUNC_CTRL_XCVRSEL_MASK |
					ULPI_FUNC_CTRL_OPMODE_MASK);
// 20110324 prime@sdcmicro.com Disabled IDGND interrupt [START]
		//twl4030_usb_clear_bits(twl, ULPI_USB_INT_EN_RISE, USB_INT_IDGND);
		//twl4030_usb_clear_bits(twl, ULPI_USB_INT_EN_FALL, USB_INT_IDGND);
// 20110324 prime@sdcmicro.com Disabled IDGND interrupt [END]
		break;
	case -1:
		/* FIXME: power on defaults */
		break;
	default:
		dev_err(twl->dev, "unsupported T2 transceiver mode %d\n",
				mode);
		break;
	};
}

void check_usb_reg(void)
{
	struct twl4030_usb *twl = t2_transceiver;
	int val = twl4030_usb_read(twl, FUNC_CTRL);

	printk (KERN_INFO "VVVV CHECK REG\n");
	printk (KERN_ERR "====^^==== check_usb_reg + : FUNC_CTRL=%x \n",val);

	/* S[, 20111128, mschung@ubiquix.com, for stable USB connection after hidden reset. */
	if ((val & 0xE7) != val)
	{
		val &= 0xE7;
		twl4030_usb_write(twl, FUNC_CTRL, val);
	}
	/* E], 20111128, mschung@ubiquix.com, for stable USB connection after hidden reset. */

	if(val!=0x40 && val!=0x45)
	{
		printk (KERN_ERR "====^^==== check_usb_reg 1 \n");
		printk(KERN_INFO "VVVV PANIC FUNC_CTRL val =%x \n",val);

		twl4030_usb_irq(twl->irq, twl); /* 20111105, mschung@ubiquix.com, for stable USB connection. */
	}        
}

EXPORT_SYMBOL(check_usb_reg);

static void twl4030_i2c_access(struct twl4030_usb *twl, int on)
{
	unsigned long timeout;
	int val = twl4030_usb_read(twl, PHY_CLK_CTRL);

	if (val >= 0) {
		if (on) {
			/* enable DPLL to access PHY registers over I2C */
			val |= REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);

			timeout = jiffies + HZ;
			while (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK)
				&& time_before(jiffies, timeout))
					udelay(10);
			if (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK))
				dev_err(twl->dev, "Timeout setting T2 HSUSB "
						"PHY DPLL clock\n");
		} else {
			/* let ULPI control the DPLL clock */
			val &= ~REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);
		}
	}
}

static void twl4030_phy_power(struct twl4030_usb *twl, int on)
{
	u8 pwr;

	pwr = twl4030_usb_read(twl, PHY_PWR_CTRL);
	printk(KERN_ERR "twl4030_phy_power + : PHY_PWR_CTRL=%x ====^^==== (twl4030-usb.c)\n", pwr);
	pwr = twl4030_usb_read(twl, PHY_CLK_CTRL);
	printk(KERN_ERR "twl4030_phy_power + : PHY_CLK_CTRL=%x ====^^==== (twl4030-usb.c)\n", pwr);
	pwr = twl4030_usb_read(twl, PHY_CLK_CTRL_STS);
	printk(KERN_ERR "twl4030_phy_power + : PHY_CLK_CTRL_STS=%x ====^^==== (twl4030-usb.c)\n", pwr);
	if (on) {
#if 0	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
		regulator_enable(twl->usb3v1);
		regulator_enable(twl->usb1v8);
#endif
		/*
		 * Disabling usb3v1 regulator (= writing 0 to VUSB3V1_DEV_GRP
		 * in twl4030) resets the VUSB_DEDICATED2 register. This reset
		 * enables VUSB3V1_SLEEP bit that remaps usb3v1 ACTIVE state to
		 * SLEEP. We work around this by clearing the bit after usv3v1
		 * is re-activated. This ensures that VUSB3V1 is really active.
		 */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
							VUSB_DEDICATED2);
#if 1	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
		regulator_enable(twl->usb1v8);
#endif
		regulator_enable(twl->usb1v5);
		pwr &= ~PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		twl4030_usb_write(twl, PHY_CLK_CTRL,
				  twl4030_usb_read(twl, PHY_CLK_CTRL) |
					(PHY_CLK_CTRL_CLOCKGATING_EN |
						PHY_CLK_CTRL_CLK32K_EN));
	} else  {
		msleep(250); // LGE_CHANGE [HUB] jjun.lee for USB unplug detect (TI Girish)
		pwr |= PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		regulator_disable(twl->usb1v5);
		regulator_disable(twl->usb1v8);
#if 1	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
		/* Put VUSB3V1 regulator in sleep mode */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x08,
				                        VUSB_DEDICATED2);
#else
		regulator_disable(twl->usb3v1);
#endif
	}
	pwr = twl4030_usb_read(twl, PHY_PWR_CTRL);
	printk(KERN_ERR "twl4030_phy_power - : PHY_PWR_CTRL=%x ====^^==== (twl4030-usb.c)\n", pwr);
	pwr = twl4030_usb_read(twl, PHY_CLK_CTRL);
	printk(KERN_ERR "twl4030_phy_power - : PHY_CLK_CTRL=%x ====^^==== (twl4030-usb.c)\n", pwr);
	pwr = twl4030_usb_read(twl, PHY_CLK_CTRL_STS);
	printk(KERN_ERR "twl4030_phy_power - : PHY_CLK_CTRL_STS=%x ====^^==== (twl4030-usb.c)\n", pwr);
}

static void twl4030_phy_suspend(struct twl4030_usb *twl, int controller_off)
{
	if (twl->asleep) {
		return;
	}

	twl4030_phy_power(twl, 0);
	twl->asleep = 1;
}

static void twl4030_phy_resume(struct twl4030_usb *twl)
{
	int status; // LGE CHANGE jjun.lee, Current Optimization by Prakash TI
	if (!twl->asleep) {
		return;
	}
	
	// LGE CHANGE jjun.lee, Current Optimization by Prakash TI
	/* To check the LINK status before resume..
	 * check to avoid enabling a LDO's 
	 * */
	status = twl4030_usb_linkstat(twl);
	if (status == USB_EVENT_NONE)
		return;
	// LGE CHANGE jjun.lee, Current Optimization by Prakash TI

	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(twl, 1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(twl, 0);
	twl->asleep = 0;

}

static int twl4030_usb_ldo_init(struct twl4030_usb *twl)
{
	/* Enable writing to power configuration registers */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xC0, PROTECT_KEY);
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0C, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

#if 1	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
	/* Turn on 3.1V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB3V1_DEV_GRP);
#else
	/* Initialize 3.1V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_DEV_GRP);

	twl->usb3v1 = regulator_get(twl->dev, "usb3v1");
	if (IS_ERR(twl->usb3v1))
		return -ENODEV;
#endif

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* Initialize 1.5V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_DEV_GRP);

	twl->usb1v5 = regulator_get(twl->dev, "usb1v5");
	if (IS_ERR(twl->usb1v5))
		goto fail1;

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* Initialize 1.8V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_DEV_GRP);

	twl->usb1v8 = regulator_get(twl->dev, "usb1v8");
	if (IS_ERR(twl->usb1v8))
		goto fail2;

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);

	return 0;

fail2:
	regulator_put(twl->usb1v5);
	twl->usb1v5 = NULL;
fail1:
#if 0	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
	regulator_put(twl->usb3v1);
	twl->usb3v1 = NULL;
#endif
	return -ENODEV;
}

static ssize_t twl4030_usb_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);
	ret = sprintf(buf, "%s\n",
			(twl->linkstat == USB_EVENT_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}
static DEVICE_ATTR(vbus, 0444, twl4030_usb_vbus_show, NULL);

#ifdef CONFIG_LGE_OMAP3_EXT_PWR
extern void set_external_power_detect(u8 ta_on);
extern void set_charging_current(void);

struct work_struct set_ext_pwr_twl_usb_work;
static void twl_usb_ext_pwr_work(struct work_struct *data)
{
	printk(KERN_INFO "[charging_msg] %s: ext_pwr \n", __FUNCTION__);
	set_charging_current();
	return ;
}

#if 0 /* mbk_temp */ 
#define REG_PWR_ISR1		0x00
#define REG_PWR_IMR1		0x01
#define REG_PWR_EDR1		0x05
#define REG_PWR_SIH_CTRL	0x007
int lge_twl4030charger_presence_evt(int vbus)
{
	int ret = 0;
	u8 pwr_isr1 = 0;
	u8 pwr_imr1 = 0;
	u8 pwr_edr1 = 0;
	u8 pwr_sih_ctrl = 0;

	ret = twl_i2c_read_u8(TWL4030_MODULE_INT, &pwr_isr1,
		REG_PWR_ISR1);
	printk("[charging_msg] %s: PWR_ISR1 0x%x\n", __FUNCTION__, pwr_isr1);

	ret = twl_i2c_read_u8(TWL4030_MODULE_INT, &pwr_imr1,
		REG_PWR_IMR1);
	printk("[charging_msg] %s: PWR_IMR1 0x%x\n", __FUNCTION__, pwr_imr1);

	ret = twl_i2c_read_u8(TWL4030_MODULE_INT, &pwr_edr1,
		REG_PWR_EDR1);
	printk("[charging_msg] %s: PWR_EDR1 0x%x\n", __FUNCTION__, pwr_edr1);

	ret = twl_i2c_read_u8(TWL4030_MODULE_INT, &pwr_edr1,
		REG_PWR_SIH_CTRL);
	printk("[charging_msg] %s: PWR_SIH_CTRL 0x%x\n", __FUNCTION__, pwr_sih_ctrl);

	return 0;
}
#endif 
#endif 

#if 0 /* mbk_wake mbk_temp */ 
// LGE_CHANGE work queue &  wake lock for usb connection
//static void twl4030_usb_wq_func(struct delayed_work *twl4030_usb_wq){
static void twl4030_usb_wq_func(struct work_struct *twl4030_usb_wq){
	wake_unlock(&the_wlock.wake_lock);
	the_wlock.wake_lock_on=0;
	printk(KERN_WARNING "[twl4030-usb] wake_lock_on=0 (unlocked) \n");
}
// LGE_CHANGE work queue &  wake lock for usb connection
#endif 

// LGE_CHANGE work queue &  wake lock for usb connection
static void twl4030_usb_wq_func(struct delayed_work *twl4030_usb_wq){
	if(0==the_wlock.usb_connected)
	{
		wake_unlock(&the_wlock.wake_lock);	
		the_wlock.wake_lock_on = 0;
		printk(KERN_WARNING "[twl4030-usb] wake_lock_on=0 (unlocked) \n");
	}
}

static void twl4030_usb_wq_func2(struct delayed_work *twl4030_usb_wq2)
{
	wake_unlock(&the_wlock2.wake_lock);
	printk(KERN_WARNING "[twl4030-usb] 22 wake_lock_on=0 (unlocked) \n");
}
// LGE_CHANGE work queue &  wake lock for usb connection


static irqreturn_t twl4030_usb_irq(int irq, void *_twl)
{
	struct twl4030_usb *twl = _twl;
	int status;
	//20101006 taehwan.kim@lge.com To  detect USB or charger on VBUS irq[START_LGE]
    	int i=0, muic_mode_temp = 0;

	// LGE_CHANGE_S [daewung.kim@lge.com] 2010-12-29, USB interrupt failure at suspend state
    	wake_lock(&the_wlock2.wake_lock);	
    	cancel_delayed_work(&twl4030_usb_wq2);    
    	schedule_delayed_work(&twl4030_usb_wq2, msecs_to_jiffies(10000));	 // to delay unlock wake_lock	
    	printk(KERN_WARNING "[twl4030-usb] 22 wake_lock_on=1 (locked)\n");
	printk(KERN_WARNING "twl4030_usb_irq: suspend_resume_statecheck: %d\n", suspend_resume_statecheck);

	if (1 != suspend_resume_statecheck) 
	{
    		msleep(1000); 
		//late_usbirq = 1;
		//return IRQ_HANDLED;
    	}

	if(1 != suspend_resume_statecheck)
	{ 
		for(i=1;i<6;i++)
		{
			msleep(200);
        		if(1 == suspend_resume_statecheck)
				break;
		}

	} 
	// LGE_CHANGE_E [daewung.kim@lge.com] 2010-12-29, USB interrupt failure at suspend state

    	muic_mode_final = get_muic_mode();
	//20101006 taehwan.kim@lge.com To  detect USB or charger on VBUS irq[END_LGE]
#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif
	msleep(200);
	for(i=0;i<9;i++)
	{
		muic_mode_temp = get_muic_mode();
		if (muic_mode_final != muic_mode_temp)
		{
			muic_mode_final = muic_mode_temp;
			break;
		}
		msleep(100);
	}
	if ( 0== muic_mode_final ) { //If unknown
		printk(KERN_WARNING "[twl4030-usb] muic mode unknown ----^^---- muic_mode: %d \n", muic_mode_final);
		return IRQ_HANDLED;
	}	
	if ( 2== muic_mode_final || 3== muic_mode_final  || 4== muic_mode_final  || 5== muic_mode_final ) { //If TA
		printk(KERN_WARNING "[twl4030-usb] TA detected ----^^---- muic_mode: %d \n", muic_mode_final);
		return IRQ_HANDLED;
	}
	status = twl4030_usb_linkstat(twl);
/* LGE_CHANGE_START 2011-03-16 kenneth.kang@lge.com patch for Adb offline set and Mass Storage Driver detecting fail */    
    	vbus_irq_muic_handler(status); //20110123 taehwan.kim@lge.com To support illegal charger (ex any charger)
/* LGE_CHANGE_END 2011-03-16 kenneth.kang@lge.com */
	if (status >= 0) {
		/* FIXME add a set_power() method so that B-devices can
		 * configure the charger appropriately.  It's not always
		 * correct to consume VBUS power, and how much current to
		 * consume is a function of the USB configuration chosen
		 * by the host.
		 *
		 * REVISIT usb_gadget_vbus_connect(...) as needed, ditto
		 * its disconnect() sibling, when changing to/from the
		 * USB_LINK_VBUS state.  musb_hdrc won't care until it
		 * starts to handle softconnect right.
		 */
// LGE_UPDATE_S 20110401 [jaejoong.kim@lge.com] check muic status 
		// if not AP_USB and not CP_USB
		if( (8 != muic_mode_final) && (9 != muic_mode_final) )
		{
			status = USB_EVENT_NONE;
			printk(KERN_WARNING "[twl4030-usb] USB_LINK_NONE (not AP_USB or CP_USB) ----^^---- muic_mode: %d \n", muic_mode_final);								
		}
// LGE_UPDATE_S 20110401 [jaejoong.kim@lge.com] check muic status 

		if (status == USB_EVENT_NONE)
		{

#if defined(CONFIG_MACH_LGE_OMAP3)
			musb_link_force_active(0);
#endif // defined(CONFIG_MACH_LGE_OMAP3)

			twl4030_phy_suspend(twl, 0);
			the_wlock.usb_connected = 0;
			if(1==the_wlock.wake_lock_on) 
			{
				schedule_delayed_work(&twl4030_usb_wq, msecs_to_jiffies(8000));	
				/* 500 msec */ // to delay unlock wake_lock
			} 
		}
		else // usb connnect
		{
			// LGE_CHANGE wake lock for usb connection
			if(8==muic_mode_final)
			{
				the_wlock.usb_connected = 1;		
				cancel_delayed_work(&twl4030_usb_wq);
			
				if(0==the_wlock.wake_lock_on)
				{
					wake_lock(&the_wlock.wake_lock);
					the_wlock.wake_lock_on=1;
					printk(KERN_WARNING "[twl4030-usb] wake_lock_on=1 (locked)\n");
				}

#if defined(CONFIG_MACH_LGE_OMAP3)
				musb_link_force_active(1);
#endif // defined(CONFIG_MACH_LGE_OMAP3)
				twl4030_phy_resume(twl);
			}
		blocking_notifier_call_chain(&twl->otg.notifier, status,
				twl->otg.gadget);
		}
#if 0 /* mbk_temp */ 
	printk(KERN_INFO "[charging_msg] %s: status %x\n", __FUNCTION__, status);
	//lge_twl4030charger_presence_evt(1);
	if( status == USB_EVENT_NONE) {
#ifdef CONFIG_LGE_OMAP3_EXT_PWR
		set_external_power_detect(0);
		schedule_work(&set_ext_pwr_twl_usb_work);
#endif 

#if 0 /* mbk_wake mbk_temp */ 
		// LGE_CHANGE work queue &  wake lock for usb connection
		if(1==the_wlock.wake_lock_on){
			schedule_delayed_work(&twl4030_usb_wq, msecs_to_jiffies(500));	/* 500 msec */ // to delay unlock wake_lock
		}
		// LGE_CHANGE work queue &  wake lock for usb connection
#endif
	}
	else if( status == USB_EVENT_VBUS) {
#ifdef CONFIG_LGE_OMAP3_EXT_PWR
		set_external_power_detect(1);
		schedule_work(&set_ext_pwr_twl_usb_work);
#endif 

#if 0 /* mbk_wake mbk_temp */ 
		// LGE_CHANGE wake lock for usb connection
		if(0==the_wlock.wake_lock_on){
			wake_lock(&the_wlock.wake_lock);
			the_wlock.wake_lock_on=1;
			printk(KERN_WARNING "[twl4030-usb] wake_lock_on=1 (locked)\n");
		}
		// LGE_CHANGE wake lock for usb connection
#endif
	}
#endif 
	}
	sysfs_notify(&twl->dev->kobj, NULL, "vbus");
	return IRQ_HANDLED;
}

static int twl4030_set_suspend(struct otg_transceiver *x, int suspend)
{
	struct twl4030_usb *twl = xceiv_to_twl(x);

	if (suspend)
		twl4030_phy_suspend(twl, 1);
	else
		twl4030_phy_resume(twl);

	return 0;
}

static int twl4030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int twl4030_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.host = host;
	if (!host)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int __devinit twl4030_usb_probe(struct platform_device *pdev)
{
	struct twl4030_usb_data *pdata = pdev->dev.platform_data;
	struct twl4030_usb	*twl;
	int			status, err;

#ifdef CONFIG_LGE_OMAP3_EXT_PWR
	INIT_WORK(&set_ext_pwr_twl_usb_work, twl_usb_ext_pwr_work);
#endif 

#if 1 /* mbk_wake mbk_temp */ 
	// LGE_CHANGE wake lock for usb connection
	wake_lock_init(&the_wlock.wake_lock, WAKE_LOCK_SUSPEND, "twl4030_usb_connection");
	the_wlock.wake_lock_on=0;
	the_wlock.usb_connected=0;
	wake_lock_init(&the_wlock2.wake_lock, WAKE_LOCK_SUSPEND, "twl4030_usb_connection2");
	the_wlock2.wake_lock_on=0;
	the_wlock2.usb_connected=0;
	// LGE_CHANGE wake lock for usb connection

	// LGE_CHANGE work queue
	INIT_DELAYED_WORK(&twl4030_usb_wq, twl4030_usb_wq_func);
	INIT_DELAYED_WORK(&twl4030_usb_wq2, twl4030_usb_wq_func2);	
	// LGE_CHANGE work queue
#endif

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;



	t2_transceiver = twl;

	twl->dev		= &pdev->dev;
	twl->irq		= platform_get_irq(pdev, 0);
	twl->otg.dev		= twl->dev;
	twl->otg.label		= "twl4030";
	twl->otg.set_host	= twl4030_set_host;
	twl->otg.set_peripheral	= twl4030_set_peripheral;
	twl->otg.set_suspend	= twl4030_set_suspend;
	twl->usb_mode		= pdata->usb_mode;
	twl->asleep		= 1;

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = twl4030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}

	otg_set_transceiver(&twl->otg);

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	BLOCKING_INIT_NOTIFIER_HEAD(&twl->otg.notifier);

	/* Our job is to use irqs and status from the power module
	 * to keep the transceiver disabled when nothing's connected.
	 *
	 * FIXME we actually shouldn't start enabling it until the
	 * USB controller drivers have said they're ready, by calling
	 * set_host() and/or set_peripheral() ... OTG_capable boards
	 * need both handles, otherwise just one suffices.
	 */
	twl->irq_enabled = true;
	status = request_threaded_irq(twl->irq, NULL, twl4030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"twl4030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return status;
	}
// LGE_UPDATE_S jaejoong.kim
//	wake_lock_init(&usb_lock, WAKE_LOCK_SUSPEND, "musb_wake_lock");
	//regulator_enable(twl->usb3v1);
	/* The IRQ handler just handles changes from the previous states
	 * of the ID and VBUS pins ... in probe() we must initialize that
	 * previous state.  The easy way:  fake an IRQ.
	 *
	 * REVISIT:  a real IRQ might have happened already, if PREEMPT is
	 * enabled.  Else the IRQ may not yet be configured or enabled,
	 * because of scheduling delays.
	 */
	twl4030_usb_irq(twl->irq, twl);
	//if (twl4030_usb_linkstat(twl) == USB_EVENT_NONE) {
	//	regulator_disable(twl->usb3v1);
	//}

	dev_info(&pdev->dev, "Initialized TWL4030 USB module\n");
	return 0;
}

static int __exit twl4030_usb_remove(struct platform_device *pdev)
{
	struct twl4030_usb *twl = platform_get_drvdata(pdev);
	int val;

	free_irq(twl->irq, twl);
	device_remove_file(twl->dev, &dev_attr_vbus);

	/* set transceiver mode to power on defaults */
	twl4030_usb_set_mode(twl, -1);

	/* autogate 60MHz ULPI clock,
	 * clear dpll clock request for i2c access,
	 * disable 32KHz
	 */
	val = twl4030_usb_read(twl, PHY_CLK_CTRL);
	if (val >= 0) {
		val |= PHY_CLK_CTRL_CLOCKGATING_EN;
		val &= ~(PHY_CLK_CTRL_CLK32K_EN | REQ_PHY_DPLL_CLK);
		twl4030_usb_write(twl, PHY_CLK_CTRL, (u8)val);
	}

	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);

	twl4030_phy_power(twl, 0);
	regulator_put(twl->usb1v5);
	regulator_put(twl->usb1v8);
#if 0	/* LGE_CHANGE [HEAVEN: newcomet@lge.com] on 2009-10-14, for <25.12 USB interrupt fix> */
	regulator_put(twl->usb3v1);
#endif
// LGE_UPDATE_S jaejoong.kim
//	wake_lock_destroy(&usb_lock);

	kfree(twl);

#if 1 /* mbk_wake mbk_temp */ 
	// LGE_CHANGE wake lock for usb connection
	wake_lock_destroy(&the_wlock.wake_lock);
	wake_lock_destroy(&the_wlock2.wake_lock);
	// LGE_CHANGE wake lock for usb connection
#endif

	return 0;
}

static struct platform_driver twl4030_usb_driver = {
	.probe		= twl4030_usb_probe,
	.remove		= __exit_p(twl4030_usb_remove),
	.driver		= {
		.name	= "twl4030_usb",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_usb_init(void)
{
	return platform_driver_register(&twl4030_usb_driver);
}
subsys_initcall(twl4030_usb_init);

static void __exit twl4030_usb_exit(void)
{
	platform_driver_unregister(&twl4030_usb_driver);
}
module_exit(twl4030_usb_exit);

MODULE_ALIAS("platform:twl4030_usb");
MODULE_AUTHOR("Texas Instruments, Inc, Nokia Corporation");
MODULE_DESCRIPTION("TWL4030 USB transceiver driver");
MODULE_LICENSE("GPL");
