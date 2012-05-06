obj-$(CONFIG_LGE_MTC_ETA)	+= mtc_eta/
obj-$(CONFIG_HUB_CHARGING_IC)   += hub_charging_ic.o
obj-$(CONFIG_HUB_MODEM_CONTROL) += hub_modem_ctrl.o

# S, 20110809, mschung@ubiquix.com, Merge(Code Separate) MUIC of HUB froyo (to do CTS test).
ifeq ($(CONFIG_PRODUCT_LGE_HUB),y)
obj-$(CONFIG_HUB_MUIC)          += hub_muic.o
endif
ifeq ($(CONFIG_PRODUCT_LGE_BLACK),y)
obj-$(CONFIG_HUB_MUIC)          += black_muic.o
endif
ifeq ($(CONFIG_PRODUCT_LGE_JUSTIN),y)
obj-$(CONFIG_HUB_MUIC)          += justin_muic.o
endif
# E, 20110809, mschung@ubiquix.com, Merge(Code Separate) MUIC of HUB froyo (to do CTS test).

obj-$(CONFIG_LGE_USB_SWITCH)    += usb_switch.o
# LGE_CHANGE_S, hyun.seungjin@lge.com, 2011-02-23, Move directory from driver/hub to misc
#obj-$(CONFIG_HUB_PROXI_SENSOR)  += hub_proximity.o
#obj-$(CONFIG_HUB_COMPASS)       += akm8973.o
#obj-$(CONFIG_HUB_VIBRATOR)      += hub_vibrator.o
#obj-$(CONFIG_HUB_KXTF9)         += kxtf9.o kxtf9_if.o kionix_accel_api.o
#obj-$(CONFIG_HUB_MPU3050)   	+= mpu3050.o
#obj-$(CONFIG_HUB_MOTION)   	+= hub_motion.o
# LGE_CHANGE_E, hyun.seungjin@lge.com, 2011-02-23, Move directory from driver/hub to misc
obj-$(CONFIG_HUB_HEADSET_DET)  	+= headset_det.o
obj-$(CONFIG_HUB_AMP_WM9093)  += wm9093.o
obj-$(CONFIG_KERNEL_GSM0710MUX) += ts0710mux/
obj-$(CONFIG_RINETWORK_DRIVER)  += rindrv/
obj-$(CONFIG_HUB_GPS_GPIO)	+= hub_gps.o
obj-$(CONFIG_BD_ADDRESS)	+= bd_address.o
obj-$(CONFIG_BACKLIGHT_LM3528)	+=	lm3528.o		#20110627 added by jslee@ubiquix.com

# [[ LGE_UBIQUIX_MODIFIED_START : shyun@ubiquix.com [2011.08.26] - ATS Build source is added.
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += hub_ats.o
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += lge_ats_eta.o
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += lge_ats_event_log.o
# ]] LGE_UBIQUIX_MODIFIED_END : shyun@ubiquix.com [2011.08.26]- ATS Build source is added.


