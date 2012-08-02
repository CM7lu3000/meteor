/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>

// 20110425 prime@sdcmicro.com Patch for INTC autoidle management to make sure it is done in atomic operation with interrupt disabled [START]
#include <linux/notifier.h>
// 20110425 prime@sdcmicro.com Patch for INTC autoidle management to make sure it is done in atomic operation with interrupt disabled [END]

#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include <plat/opp.h>
#include <plat/voltage.h>

/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
#include <linux/dvs_suite.h>
/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */

#include "omap3-opp.h"
#include "opp44xx.h"

// LGE_UPDATE_S
#if defined(CONFIG_MACH_LGE_OMAP3)
#include "pm.h"

u32 sleep_while_idle;
u32 enable_off_mode;
#endif
// LGE_UPDATE_E

// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [START_LGE]
// TODO: make more pretty...
enum {
	RESET_NORMAL,
	RESET_CHARGER_DETECT,
	RESET_GLOBAL_SW_RESET,
	RESET_KERNEL_PANIC,
	RESET_HIDDEN_SW_RESET,
};

int reset_status = RESET_NORMAL;
int hidden_reset_enabled = 0;
/* LGE_CHANGE_S bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
static int hub_secure_mode = 0;
/* LGE_CHANGE_E bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
/* S, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */
#if defined(CONFIG_PRODUCT_LGE_HUB)
int hub_cp_usb_mode = 0;
#endif
/* E, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */

static ssize_t reset_status_show(struct kobject *, struct kobj_attribute *, char *);
static struct kobj_attribute reset_status_attr =
	__ATTR(reset_status, 0644, reset_status_show, NULL);

static ssize_t hidden_reset_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t hidden_reset_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);
static struct kobj_attribute hidden_reset_attr =
	__ATTR(hidden_reset, 0644, hidden_reset_show, hidden_reset_store);
/* LGE_CHANGE_S bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
static ssize_t secure_mode_show(struct kobject *, struct kobj_attribute *, char *);
static struct kobj_attribute secure_mode_attr =
	__ATTR(secure_mode, 0644, secure_mode_show, NULL);
/* LGE_CHANGE_E bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */

/* S, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */
#if defined(CONFIG_PRODUCT_LGE_HUB)
static ssize_t cp_usb_mode_show(struct kobject *, struct kobj_attribute *, char *);
static struct kobj_attribute cp_usb_mode_attr =
	__ATTR(cp_usb_mode, 0644, cp_usb_mode_show, NULL);
#endif
/* E, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */

static void reset_status_setup(char *str)
{
        if (str[0] == 'p')
            reset_status = RESET_KERNEL_PANIC;
        else if (str[0] == 'h')
            reset_status = RESET_HIDDEN_SW_RESET;
        else if (str[0] == 'c')
            reset_status = RESET_CHARGER_DETECT;
#if 0 /* 20110720, mschung@ubiquix.com, Block. Phone should boot-up, not showing chargerlogo, after SW Downloaded using LGDP. */
//--[[ LGE_UBIQUIX_MODIFIED_START : scchoi@ubiquix.com [2011-07-09] - Added RESET_GLOBAL_SW_RESET " s=rs"
#if defined(CONFIG_PRODUCT_LGE_HUB)
        else if (str[0] == 's')
           reset_status = RESET_GLOBAL_SW_RESET;
#endif		
//--]] LGE_UBIQUIX_MODIFIED_END : scchoi@ubiquix.com [2011-07-09] - Added RESET_GLOBAL_SW_RESET " s=rs"
#endif
        printk("reset_status: %c\n", str[0]);
}
__setup("rs=", reset_status_setup);

/* LGE_CHANGE_S bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
static void hub_secure_mode_setup(char *str)
{
	if (str[0] == '1')
		hub_secure_mode = 1;
	else 
		hub_secure_mode = 0;

	printk("hub_secure_mode: %d\n", hub_secure_mode);
}
__setup("secure=", hub_secure_mode_setup);
/* LGE_CHANGE_E bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */

/* S, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */
#if defined(CONFIG_PRODUCT_LGE_HUB)
static void hub_cp_usb_setup(char *str)
{
	if (str[0] == '1')
		hub_cp_usb_mode = 1;
	else 
		hub_cp_usb_mode = 0;

	printk("hub_cp_usb_mode: %d\n", hub_cp_usb_mode);
}
__setup("cpusb=", hub_cp_usb_setup);
#endif
/* E, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */

static ssize_t reset_status_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &reset_status_attr)
		return sprintf(buf, "%d\n", reset_status);
	else
		return -EINVAL;
}

/* LGE_CHANGE_S bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
static ssize_t secure_mode_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &secure_mode_attr)
		return sprintf(buf, "%d\n", hub_secure_mode);
	else
		return -EINVAL;
}
/* LGE_CHANGE_E bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */

/* S, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */
#if defined(CONFIG_PRODUCT_LGE_HUB)
static ssize_t cp_usb_mode_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &cp_usb_mode_attr)
		return sprintf(buf, "%d\n", hub_cp_usb_mode);
	else
		return -EINVAL;
}
#endif
/* E, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */

static ssize_t hidden_reset_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &hidden_reset_attr)
		return sprintf(buf, "%d\n", hidden_reset_enabled);
	else
		return -EINVAL;
}
static ssize_t hidden_reset_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1)
		return -EINVAL;

	if (attr == &hidden_reset_attr) {
                hidden_reset_enabled = value;
	} else {
		return -EINVAL;
	}
	return n;
}
// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [END_LGE]

static struct omap_device_pm_latency *pm_lats;

static struct device *mpu_dev;
static struct device *iva_dev;
static struct device *l3_dev;
static struct device *dsp_dev;

// 20110425 prime@sdcmicro.com Patch for INTC autoidle management to make sure it is done in atomic operation with interrupt disabled [START]
#if 1

/* idle notifications late in the idle path (atomic, interrupts disabled) */
static ATOMIC_NOTIFIER_HEAD(idle_notifier);

void omap_idle_notifier_register(struct notifier_block *n)
{
	atomic_notifier_chain_register(&idle_notifier, n);
}
EXPORT_SYMBOL_GPL(omap_idle_notifier_register);

void omap_idle_notifier_unregister(struct notifier_block *n)
{
	atomic_notifier_chain_unregister(&idle_notifier, n);
}
EXPORT_SYMBOL_GPL(omap_idle_notifier_unregister);

void omap_idle_notifier_start(void)
{
	atomic_notifier_call_chain(&idle_notifier, OMAP_IDLE_START, NULL);
}

void omap_idle_notifier_end(void)
{
	atomic_notifier_call_chain(&idle_notifier, OMAP_IDLE_END, NULL);
}

#endif
// 20110425 prime@sdcmicro.com Patch for INTC autoidle management to make sure it is done in atomic operation with interrupt disabled [END]

struct device *omap2_get_mpuss_device(void)
{
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
	if(ds_status.flag_correct_cpu_op_update_path == 0) 
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
	WARN_ON_ONCE(!mpu_dev);
	return mpu_dev;
}
EXPORT_SYMBOL(omap2_get_mpuss_device);

struct device *omap2_get_iva_device(void)
{
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
	if(ds_status.flag_correct_cpu_op_update_path == 0) 
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
	WARN_ON_ONCE(!iva_dev);
	return iva_dev;
}
EXPORT_SYMBOL(omap2_get_iva_device);

struct device *omap2_get_l3_device(void)
{
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
	if(ds_status.flag_correct_cpu_op_update_path == 0) 
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
	WARN_ON_ONCE(!l3_dev);
	return l3_dev;
}
EXPORT_SYMBOL(omap2_get_l3_device);

struct device *omap4_get_dsp_device(void)
{
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
	if(ds_status.flag_correct_cpu_op_update_path == 0) 
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
	WARN_ON_ONCE(!dsp_dev);
	return dsp_dev;
}
EXPORT_SYMBOL(omap4_get_dsp_device);

#ifdef CONFIG_OMAP_PM
/* Overclock vdd sysfs interface */
static ssize_t overclock_vdd_show(struct kobject *, struct kobj_attribute *,
              char *);
static ssize_t overclock_vdd_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);


static struct kobj_attribute overclock_vdd_opp1_attr =
    __ATTR(overclock_vdd_opp1, 0644, overclock_vdd_show, overclock_vdd_store);
static struct kobj_attribute overclock_vdd_opp2_attr =
    __ATTR(overclock_vdd_opp2, 0644, overclock_vdd_show, overclock_vdd_store);
static struct kobj_attribute overclock_vdd_opp3_attr =
    __ATTR(overclock_vdd_opp3, 0644, overclock_vdd_show, overclock_vdd_store);
static struct kobj_attribute overclock_vdd_opp4_attr =
    __ATTR(overclock_vdd_opp4, 0644, overclock_vdd_show, overclock_vdd_store);

/* PM stuff */
static ssize_t vdd_opp_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t vdd_opp_store(struct kobject *k, struct kobj_attribute *, const char *buf, size_t n);

static struct kobj_attribute vdd1_opp_attr =
	__ATTR(vdd1_opp, 0444, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd2_opp_attr =
	__ATTR(vdd2_opp, 0444, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd1_lock_attr =
	__ATTR(vdd1_lock, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd2_lock_attr =
	__ATTR(vdd2_lock, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute dsp_freq_attr =
	__ATTR(dsp_freq, 0644, vdd_opp_show, vdd_opp_store);

/* Overclock vdd sysfs interface */
static ssize_t overclock_vdd_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	unsigned int target_opp;
	unsigned long *vdd = -1;
	unsigned long *temp_vdd = -1;
	char *voltdm_name = "mpu";
	struct device *mpu_dev = omap2_get_mpuss_device();
	struct cpufreq_frequency_table *mpu_freq_table = *omap_pm_cpu_get_freq_table();
	struct omap_opp *temp_opp;
	struct voltagedomain *mpu_voltdm;
	struct omap_volt_data *mpu_voltdata;

	if(!mpu_dev || !mpu_freq_table)
		return -EINVAL;

	if ( attr == &overclock_vdd_opp1_attr) {
		target_opp = 0;
	}
	if ( attr == &overclock_vdd_opp2_attr) {
		target_opp = 1;
	}
	if ( attr == &overclock_vdd_opp3_attr) {
		target_opp = 2;
	}
	if ( attr == &overclock_vdd_opp4_attr) {
		target_opp = 3;
	}

	temp_opp = opp_find_freq_exact(mpu_dev, mpu_freq_table[target_opp].frequency*1000, true);
	if(IS_ERR(temp_opp))
		return -EINVAL;

	temp_vdd = opp_get_voltage(temp_opp);
	mpu_voltdm = omap_voltage_domain_get(voltdm_name);
	mpu_voltdata = omap_voltage_get_voltdata(mpu_voltdm, temp_vdd);
	vdd = mpu_voltdata->volt_nominal;

	return sprintf(buf, "%lu\n", vdd);
}

static ssize_t overclock_vdd_store(struct kobject *k,
        struct kobj_attribute *attr, const char *buf, size_t n)
{
/*	unsigned int target_opp_nr;
	u32 vdd = 0;
	u32 divider = 500;
	unsigned long temp_vdd = 0;
	unsigned int vdd_lower_limit = 0;
	unsigned int vdd_upper_limit = 0;
	char *voltdm_name = "mpu";
	unsigned int freq;
	struct device *mpu_dev = omap2_get_mpuss_device();
	struct cpufreq_frequency_table *mpu_freq_table = *omap_pm_cpu_get_freq_table();
	struct omap_opp *temp_opp;
	struct voltagedomain *mpu_voltdm;
	struct omap_volt_data *mpu_voltdata;

	if(!mpu_dev || !mpu_freq_table)
		return -EINVAL;

	if ( attr == &overclock_vdd_opp1_attr) {
		target_opp_nr = 0;
		vdd_lower_limit = 900000;
		vdd_upper_limit = 1200000;
	}
	if ( attr == &overclock_vdd_opp2_attr) {
		target_opp_nr = 1;
		vdd_lower_limit = 900000;
		vdd_upper_limit = 1300000;
	}
	if ( attr == &overclock_vdd_opp3_attr) {
		target_opp_nr = 2;
		vdd_lower_limit = 900000;
		vdd_upper_limit = 1400000;
	}
	if ( attr == &overclock_vdd_opp4_attr) {
		target_opp_nr = 3;
		vdd_lower_limit = 900000;
		vdd_upper_limit = 1500000;
	}

	temp_opp = opp_find_freq_exact(mpu_dev, mpu_freq_table[target_opp_nr].frequency*1000, true);
	if(IS_ERR(temp_opp))
		return -EINVAL;

	freq = temp_opp->rate;
	temp_vdd = opp_get_voltage(temp_opp);
	mpu_voltdm = omap_voltage_domain_get(voltdm_name);
	mpu_voltdata = omap_voltage_get_voltdata(mpu_voltdm, temp_vdd);

	if (sscanf(buf, "%u", &vdd) == 1) {
		// Enforce limits and check if voltage to be set is multiple of 500uV
		if(vdd <= vdd_upper_limit && vdd >= vdd_lower_limit && vdd % divider == 0) {
			//Handle opp
			opp_disable(temp_opp);
			struct omap_opp_def new_opp_def[] = {
				OMAP_OPP_DEF("mpu", true,  freq, vdd),
			};
			mpu_voltdata->volt_nominal = vdd;
			opp_add(new_opp_def);

			//Set
			omap_voltage_scale_vdd(mpu_voltdm, mpu_voltdata);
			return n;
		}
	}
*/
	return -EINVAL;
}

/* PM stuff */
static int vdd1_locked = 0;
static int vdd2_locked = 0;
static struct device sysfs_cpufreq_dev;

static ssize_t vdd_opp_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (attr == &vdd1_opp_attr)
		return sprintf(buf, "%hu\n", opp_find_freq_exact(mpu_dev, opp_get_rate(mpu_dev), true)->opp_id+1);
	else if (attr == &vdd2_opp_attr)
		return sprintf(buf, "%hu\n", opp_find_freq_exact(l3_dev, opp_get_rate(l3_dev), true)->opp_id+1);
	else if (attr == &vdd1_lock_attr)
		return sprintf(buf, "%hu\n", vdd1_locked);
	else if (attr == &vdd2_lock_attr)
		return sprintf(buf, "%hu\n", vdd2_locked);
	else if (attr == &dsp_freq_attr)
		return sprintf(buf, "%lu\n", opp_get_rate(iva_dev)/1000);
	else
		return -EINVAL;
}

static ssize_t vdd_opp_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned long value;
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
	unsigned long lc_freq = 0;
	/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */

	if (sscanf(buf, "%lu", &value) != 1)
		return -EINVAL;

	/* Check locks */
	if (attr == &vdd1_lock_attr) {
		if (vdd1_locked) {
			/* vdd1 currently locked */
			if (value == 0) {
				if (omap_pm_set_min_mpu_freq(&sysfs_cpufreq_dev, -1)) {
					printk(KERN_ERR "%s: Failed to remove vdd1_lock\n", __func__);
				} else {
					vdd1_locked = 0;
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
					ds_status.mpu_min_freq_to_lock = 0;
//printk(KERN_WARNING "MMM unlocked\n");
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					return n;
				}
			} else {
				printk(KERN_ERR "%s: vdd1 already locked to %d\n", __func__, vdd1_locked);
				return -EINVAL;
			}
		} else {
			/* vdd1 currently unlocked */
			if (value != 0) {
				u8 i = 0;
				unsigned long freq = 0;
				struct cpufreq_frequency_table *freq_table = *omap_pm_cpu_get_freq_table();
				if (freq_table == NULL) {
					printk(KERN_ERR "%s: Could not get freq_table\n", __func__);
					return -ENODEV;
				}
				for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
					if (freq_table[i].index == value - 1) {
						freq = freq_table[i].frequency;
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
						lc_freq = freq * 1000;
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
						break;
					}
				}
				if (freq_table[i].frequency == CPUFREQ_TABLE_END) {
					printk(KERN_ERR "%s: Invalid value [0..%d]\n", __func__, i-1);
					return -EINVAL;
				}
				if (omap_pm_set_min_mpu_freq(&sysfs_cpufreq_dev, freq * 1000)) {
					printk(KERN_ERR "%s: Failed to add vdd1_lock\n", __func__);
				} else {
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
					ds_status.mpu_min_freq_to_lock = lc_freq;
//printk(KERN_WARNING "MMM locked to %lu\n", lc_freq);
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					vdd1_locked = value;
				}
			} else {
				printk(KERN_ERR "%s: vdd1 already unlocked\n", __func__);
				return -EINVAL;
			}
		}
	} else if (attr == &vdd2_lock_attr) {
		if (vdd2_locked) {
			/* vdd2 currently locked */
			if (value == 0) {
				if (omap_pm_set_min_bus_tput(&sysfs_cpufreq_dev, OCP_INITIATOR_AGENT, 0)) {
					printk(KERN_ERR "%s: Failed to remove vdd2_lock\n", __func__);
				} else {
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
					ds_status.l3_min_freq_to_lock = 0;
//printk(KERN_WARNING "LLL unlocked\n");
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					vdd2_locked = 0;
					return n;
				}
			} else {
				printk(KERN_ERR "%s: vdd2 already locked to %d\n", __func__, vdd2_locked);
				return -EINVAL;
			}
		} else {
			/* vdd2 currently unlocked */
			if (value != 0) {
				unsigned long freq = 0;
				if (cpu_is_omap3630()) {
					if(value == 1) {
						freq = 100*1000*4;
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
						lc_freq = 100000000;
//printk(KERN_WARNING "LLL locked to %lu\n", lc_freq);
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					} else if (value == 2) {
						freq = 200*1000*4;
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
						lc_freq = 200000000;
//printk(KERN_WARNING "LLL locked to %lu\n", lc_freq);
						/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					} else {
						printk(KERN_ERR "%s: Invalid value [1,2]\n", __func__);
						return -EINVAL;
					}
				}
				else if (cpu_is_omap44xx()) {
					if (omap_rev() <= OMAP4430_REV_ES2_0) {
						if(value == 1) {
							freq = 100*1000*4;
						} else if (value == 2) {
							freq = 200*1000*4;
						} else {
							printk(KERN_ERR "%s: Invalid value [1,2]\n", __func__);
							return -EINVAL;
						}
					} else {
						if(value == 1) {
							freq = 98304*4;
						} else if (value == 2) {
							freq = 100*1000*4;
						} else if (value == 3) {
							freq = 200*1000*4;
						} else {
							printk(KERN_ERR "%s: Invalid value [1,2,3]\n", __func__);
							return -EINVAL;
						}
					}
				} else {
					printk(KERN_ERR "%s: Unsupported HW [OMAP3630, OMAP44XX]\n", __func__);
					return -ENODEV;
				}
				if (omap_pm_set_min_bus_tput(&sysfs_cpufreq_dev, OCP_INITIATOR_AGENT, freq)) {
					printk(KERN_ERR "%s: Failed to add vdd2_lock\n", __func__);
				} else {
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
					ds_status.l3_min_freq_to_lock = lc_freq;
					/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
					vdd2_locked = value;
				}
				return n;
			} else {
				printk(KERN_ERR "%s: vdd2 already unlocked\n", __func__);
				return -EINVAL;
			}
		}
	} else if (attr == &dsp_freq_attr) {
		u8 i, opp_id = 0;
		struct omap_opp *opp_table = omap_pm_dsp_get_opp_table();
		if (opp_table == NULL) {
			printk(KERN_ERR "%s: Could not get dsp opp_table\n", __func__);
			return -ENODEV;
		}
		for (i = 1; opp_table[i].rate; i++) {
			if (opp_table[i].rate >= value) {
				opp_id = i;
//printk(KERN_WARNING "III value %lu opp_id %d\n", value, opp_id);
				/* 20110331 sookyoung.kim@lge.com LG-DVFS [START_LGE] */
				switch(i){
					case 1:
//printk(KERN_WARNING "III case 1\n");
						//ds_status.mpu_min_freq_to_lock = 300000000;
						ds_status.mpu_min_freq_to_lock = 0;	// Unlocked.
						ds_status.iva_min_freq_to_lock = 260000000;
						break;
					case 2:
//printk(KERN_WARNING "III case 2\n");
						ds_status.mpu_min_freq_to_lock = 600000000;
						ds_status.iva_min_freq_to_lock = 520000000;
						break;
					case 3:
//printk(KERN_WARNING "III case 3\n");
						ds_status.mpu_min_freq_to_lock = 800000000;
						ds_status.iva_min_freq_to_lock = 660000000;
						break;
					case 4:
//printk(KERN_WARNING "III case 4\n");
						ds_status.mpu_min_freq_to_lock = 1000000000;
						ds_status.iva_min_freq_to_lock = 800000000;
						break;
					default:
//printk(KERN_WARNING "III default\n");
						ds_status.mpu_min_freq_to_lock = 1000000000;
						ds_status.iva_min_freq_to_lock = 800000000;
						break;
				}
				/* 20110331 sookyoung.kim@lge.com LG-DVFS [END_LGE] */
				break;
			}
		}

		if (opp_id == 0) {
			printk(KERN_ERR "%s: Invalid value\n", __func__);
			return -EINVAL;
		}
		omap_pm_dsp_set_min_opp(opp_id);

	} else if (attr == &vdd1_opp_attr) {
		printk(KERN_ERR "%s: changing vdd1_opp is not supported\n", __func__);
		return -EINVAL;
	} else if (attr == &vdd2_opp_attr) {
		printk(KERN_ERR "%s: changing vdd2_opp is not supported\n", __func__);
		return -EINVAL;
	} else {
		return -EINVAL;
	}
	return n;
}
#endif

/* static int _init_omap_device(struct omap_hwmod *oh, void *user) */
static int _init_omap_device(char *name, struct device **new_dev)
{
	struct omap_hwmod *oh;
	struct omap_device *od;

	oh = omap_hwmod_lookup(name);
	if (WARN(!oh, "%s: could not find omap_hwmod for %s\n",
		 __func__, name))
		return -ENODEV;
	od = omap_device_build(oh->name, 0, oh, NULL, 0, pm_lats, 0, false);
	if (WARN(IS_ERR(od), "%s: could not build omap_device for %s\n",
		 __func__, name))
		return -ENODEV;

	*new_dev = &od->pdev.dev;

	return 0;
}

/*
 * Build omap_devices for processors and bus.
 */
static void omap2_init_processor_devices(void)
{
	struct omap_hwmod *oh;

	_init_omap_device("mpu", &mpu_dev);

	if (cpu_is_omap34xx())
		_init_omap_device("iva", &iva_dev);
	oh = omap_hwmod_lookup("iva");
	if (oh && oh->od)
		iva_dev = &oh->od->pdev.dev;

	oh = omap_hwmod_lookup("dsp");
	if (oh && oh->od)
		dsp_dev = &oh->od->pdev.dev;

	if (cpu_is_omap44xx())
		_init_omap_device("l3_main_1", &l3_dev);
	else
		_init_omap_device("l3_main", &l3_dev);
}

static int __init omap2_common_pm_init(void)
{
// LGE_UPDATE_S : come from pm.c
#if defined(CONFIG_MACH_LGE_OMAP3)
	sleep_while_idle = 0;  // temp... should be checked..
	enable_off_mode = 1;
#endif
// LGE_UPDATE_E : come from pm.c

// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [START_LGE]
	int error = -EINVAL;
// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [END_LGE]

	omap2_init_processor_devices();
	if (cpu_is_omap34xx())
		omap3_pm_init_opp_table();
	else if (cpu_is_omap44xx())
		omap4_pm_init_opp_table();

	omap_pm_if_init();

// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [START_LGE]
	error = sysfs_create_file(power_kobj, &reset_status_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &hidden_reset_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
/* LGE_CHANGE_S bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
	if (hub_secure_mode) {
		error = sysfs_create_file(power_kobj, &secure_mode_attr.attr);
		if (error) {
			printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
			return error;
		}
	}
/* LGE_CHANGE_E bae.cheolhwan@lge.com, 2011-05-11. Root permission enable. */
// 20100520 jugwan.eom@lge.com For power on cause and hidden reset [END_LGE]

/* S, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */
#if defined(CONFIG_PRODUCT_LGE_HUB)
	error = sysfs_create_file(power_kobj, &cp_usb_mode_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
#endif
/* E, 20110809, mschung@ubiquix.com, Merge MUIC of HUB froyo (to do CTS test). */

#ifdef CONFIG_OMAP_PM
	{
		int error = -EINVAL;

		error = sysfs_create_file(power_kobj, &dsp_freq_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(dsp_freq) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd1_opp_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd1_opp) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd2_opp_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd2_opp) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd1_lock_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd1_lock) failed %d\n", __func__ ,error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd2_lock_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd2_lock) failed %d\n", __func__, error);
			return error;
		}
	}
#endif

	return 0;
}
device_initcall(omap2_common_pm_init);
