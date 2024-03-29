#
# Makefile for the linux kernel.
#

# Common support
obj-y := id.o io.o control.o mux.o devices.o serial.o gpmc.o timer-gp.o pm.o\
	gpio.o dma.o dmtimer.o powerdomains.o

omap-2-3-common				= irq.o sdrc.o
hwmod-common				= omap_hwmod.o \
					  omap_hwmod_common_data.o
prcm-common				= prcm.o
clock-common				= clock.o clock_common_data.o \
					  clockdomain.o clkt_dpll.o \
					  clkt_clksel.o

obj-$(CONFIG_ARCH_OMAP2) += $(omap-2-3-common) $(prcm-common) $(hwmod-common)
obj-$(CONFIG_ARCH_OMAP3) += $(omap-2-3-common) $(prcm-common) $(hwmod-common)
obj-$(CONFIG_ARCH_OMAP4) += $(prcm-common) $(hwmod-common)

obj-$(CONFIG_OMAP_MCBSP)		+= mcbsp.o
obj-$(CONFIG_DEBUG_FS)			+= omap_hwmod_debug.o


# SMP support ONLY available for OMAP4
obj-$(CONFIG_SMP)			+= omap-smp.o omap-headsmp.o
obj-$(CONFIG_LOCAL_TIMERS)		+= timer-mpu.o
obj-$(CONFIG_HOTPLUG_CPU)		+= omap-hotplug.o
obj-$(CONFIG_ARCH_OMAP4)		+= omap44xx-smc.o omap4-common.o \
					   omap4-wakeupgen.o

AFLAGS_omap44xx-smc.o			:=-Wa,-march=armv7-a

# Functions loaded to SRAM
obj-$(CONFIG_ARCH_OMAP2420)		+= sram242x.o
obj-$(CONFIG_ARCH_OMAP2430)		+= sram243x.o
obj-$(CONFIG_ARCH_OMAP3)		+= sram34xx.o

AFLAGS_sram242x.o			:=-Wa,-march=armv6
AFLAGS_sram243x.o			:=-Wa,-march=armv6
AFLAGS_sram34xx.o			:=-Wa,-march=armv7-a

# Pin multiplexing
obj-$(CONFIG_ARCH_OMAP2420)		+= mux2420.o
obj-$(CONFIG_ARCH_OMAP2430)		+= mux2430.o
obj-$(CONFIG_ARCH_OMAP3)		+= mux34xx.o
obj-$(CONFIG_ARCH_OMAP4)		+= mux44xx.o

# SMS/SDRC
obj-$(CONFIG_ARCH_OMAP2)		+= sdrc2xxx.o
# obj-$(CONFIG_ARCH_OMAP3)		+= sdrc3xxx.o

# Power Management
ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_ARCH_OMAP2)		+= pm24xx.o
obj-$(CONFIG_ARCH_OMAP2)		+= sleep24xx.o
obj-$(CONFIG_ARCH_OMAP3)		+= pm34xx.o sleep34xx.o voltage.o \
					   cpuidle34xx.o pm_bus.o cpufreq34xx.o
obj-$(CONFIG_ARCH_OMAP4)		+= pm44xx.o pm_bus.o cpuidle44xx.o \
					   omap4-mpuss-lowpower.o sleep44xx.o \
					   voltage.o opp44xx_data.o \
					   dpll-44xx.o omap4-sar.o

obj-$(CONFIG_PM_DEBUG)			+= pm-debug.o
obj-$(CONFIG_OMAP_SMARTREFLEX)          += sr_device.o smartreflex.o
obj-$(CONFIG_OMAP_SMARTREFLEX_CLASS3)	+= smartreflex-class3.o
obj-$(CONFIG_OMAP_SMARTREFLEX_CLASS1P5)	+= smartreflex-class1p5.o

AFLAGS_sleep24xx.o			:=-Wa,-march=armv6
AFLAGS_sleep34xx.o			:=-Wa,-march=armv7-a

ifeq ($(CONFIG_PM_VERBOSE),y)
CFLAGS_pm_bus.o				+= -DDEBUG
endif

endif

# PRCM
obj-$(CONFIG_ARCH_OMAP2)		+= cm.o
obj-$(CONFIG_ARCH_OMAP3)		+= cm.o
obj-$(CONFIG_ARCH_OMAP4)		+= cm4xxx.o

# EMIF
obj-$(CONFIG_ARCH_OMAP4)		+= emif.o lpddr2_jedec_data.o lpddr2_elpida_data.o

# Clock framework
obj-$(CONFIG_ARCH_OMAP2)		+= $(clock-common) clock2xxx.o \
					   clkt2xxx_sys.o \
					   clkt2xxx_dpllcore.o \
					   clkt2xxx_virt_prcm_set.o \
					   clkt2xxx_apll.o clkt2xxx_osc.o
obj-$(CONFIG_ARCH_OMAP2420)		+= clock2420_data.o
obj-$(CONFIG_ARCH_OMAP2430)		+= clock2430.o clock2430_data.o
obj-$(CONFIG_ARCH_OMAP3)		+= $(clock-common) clock3xxx.o \
					   clock34xx.o clkt34xx_dpll3m2.o \
					   clock3517.o clock36xx.o \
					   dpll3xxx.o clock3xxx_data.o
obj-$(CONFIG_ARCH_OMAP4)		+= $(clock-common) clock44xx_data.o \
					   dpll3xxx.o

# OMAP2 clock rate set data (old "OPP" data)
obj-$(CONFIG_ARCH_OMAP2420)		+= opp2420_data.o
obj-$(CONFIG_ARCH_OMAP2430)		+= opp2430_data.o

# hwmod data
obj-$(CONFIG_ARCH_OMAP2420)		+= omap_hwmod_2420_data.o
obj-$(CONFIG_ARCH_OMAP2430)		+= omap_hwmod_2430_data.o
obj-$(CONFIG_ARCH_OMAP3)		+= omap_hwmod_3xxx_data.o
obj-$(CONFIG_ARCH_OMAP4)		+= omap_hwmod_44xx_data.o

#powerdomain framework
obj-$(CONFIG_ARCH_OMAP2)		+= powerdomains2xxx.o
obj-$(CONFIG_ARCH_OMAP3)		+= powerdomains2xxx.o
obj-$(CONFIG_ARCH_OMAP4)		+= powerdomains44xx.o

# EMU peripherals
obj-$(CONFIG_OMAP3_EMU)			+= emu.o

obj-$(CONFIG_OMAP_MBOX_FWK)		+= mailbox_mach.o
mailbox_mach-objs			:= mailbox.o

ifeq ($(CONFIG_OMAP_REMOTE_PROC),y)
obj-$(CONFIG_ARCH_OMAP4)		+= remoteproc44xx.o
ifeq ($(CONFIG_SYSLINK_IPU_PM_TRACES),y)
CFLAGS_ipu_drv.o				+= -DDEBUG
CFLAGS_ipu_dev.o				+= -DDEBUG
endif
obj-$(CONFIG_MPU_SYSLINK_IPC)		+= ipu_drv.o
obj-$(CONFIG_MPU_SYSLINK_IPC)		+= ipu_dev.o
obj-$(CONFIG_MPU_SYSLINK_IPC)		+= ipu_utility.o
endif
iommu-y					+= iommu2.o omap-iommu.o
obj-$(CONFIG_OMAP_IOMMU)		+= $(iommu-y)

i2c-omap-$(CONFIG_I2C_OMAP)		:= i2c.o
obj-y					+= $(i2c-omap-m) $(i2c-omap-y)

ifneq ($(CONFIG_MPU_BRIDGE),)
obj-y                                  += dspbridge.o
endif

# Specific board support
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= board-2430sdp.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_APOLLON)		+= board-apollon.o
obj-$(CONFIG_MACH_OMAP3_BEAGLE)		+= board-omap3beagle.o \
					   hsmmc.o
obj-$(CONFIG_MACH_DEVKIT8000)     	+= board-devkit8000.o \
                                           hsmmc.o
obj-$(CONFIG_MACH_OMAP_LDP)		+= board-ldp.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OVERO)		+= board-overo.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP3EVM)		+= board-omap3evm.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP3_PANDORA)	+= board-omap3pandora.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_3430SDP)		+= board-3430sdp.o \
					   hsmmc.o \
					   board-sdp-flash.o
obj-$(CONFIG_MACH_NOKIA_N8X0)		+= board-n8x0.o
obj-$(CONFIG_MACH_NOKIA_RX51)		+= board-rx51.o \
					   board-rx51-sdram.o \
					   board-rx51-peripherals.o \
					   board-rx51-video.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_ZOOM2)		+= board-zoom2.o \
					   board-zoom-peripherals.o \
					   board-zoom2-camera.o \
					   board-zoom-display.o \
					   hsmmc.o \
					   board-zoom-debugboard.o
obj-$(CONFIG_MACH_OMAP_ZOOM3)		+= board-zoom3.o \
					   board-zoom-flash.o \
					   board-zoom-peripherals.o \
					   board-zoom2-camera.o \
					   board-zoom-display.o \
					   hsmmc.o \
					   board-zoom-debugboard.o
obj-$(CONFIG_MACH_LGE_HUB)		+= board-hub.o \
					   board-hub-peripherals.o \
					   board-hub-camera.o \
					   board-hub-panel.o \
					   board-hub-wifi.o \
					   board-hub-debug.o \
					   board-hub-ers.o \
					   board-hub-mmc.o \
					   hsmmc.o
//--[[ LGE_UBIQUIX_MODIFIED_START : scchoi@ubiquix.com [2011.06.23] - Added  justin feature 
obj-$(CONFIG_PRODUCT_LGE_JUSTIN)	+= board-justin.o \
									   board-justin-peripherals.o \
									   board-justin-camera.o \
									   board-justin-panel.o \
									   board-justin-wifi.o \
									   board-justin-debug.o \
									   board-justin-ers.o \
									   board-justin-mmc.o \
									   hsmmc.o
//--]] LGE_UBIQUIX_MODIFIED_END : scchoi@ubiquix.com [2011.06.23]- Added justin feature. 
//--[[ LGE_UBIQUIX_MODIFIED_START : scchoi@ubiquix.com [2011.06.24] - Added black feature 
obj-$(CONFIG_PRODUCT_LGE_BLACK)	+= board-black.o \
									   board-black-peripherals.o \
									   board-black-camera.o \
									   board-black-panel.o \
									   board-black-wifi.o \
									   board-black-debug.o \
									   board-black-ers.o \
									   board-black-mmc.o \
									   hsmmc.o
//--]] LGE_UBIQUIX_MODIFIED_END : scchoi@ubiquix.com [2011.06.24]- Added black feature. 
# LGE_CHANGE_S, bae.cheolhwan@lge.com, 2011-03-26, Modify for AT%FBOOT & AT%SWVCHECK.
#110111 dongjin.sim@lge.com %SWV AP [START]
obj-$(CONFIG_PRODUCT_LGE_BLACK)		+= lge_emmc_direct_access.o					   
obj-$(CONFIG_PRODUCT_LGE_JUSTIN)		+= lge_emmc_direct_access.o					   
#110111 dongjin.sim@lge.com %SWV AP [END]
# LGE_CHANGE_E, bae.cheolhwan@lge.com, 2011-03-26, Modify for AT%FBOOT & AT%SWVCHECK.
obj-$(CONFIG_MACH_OMAP_3630SDP)		+= board-3630sdp.o \
					   board-zoom-peripherals.o \
					   board-zoom2-camera.o \
					   board-zoom-display.o \
					   hsmmc.o
obj-$(CONFIG_MACH_CM_T35)		+= board-cm-t35.o \
					   hsmmc.o
obj-$(CONFIG_MACH_IGEP0020)		+= board-igep0020.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP3_TOUCHBOOK)	+= board-omap3touchbook.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_4430SDP)		+= board-4430sdp.o \
					   hsmmc.o

obj-$(CONFIG_MACH_OMAP4_PANDA)		+= board-omap4panda.o \
					   hsmmc.o
# prime@sdcmicro : Fix duplicated symbols [START]
#obj-$(CONFIG_TIWLAN_SDIO)               += board-4430sdp-wifi.o
# prime@sdcmicro : Fix duplicated symbols [END]
obj-$(CONFIG_MACH_OMAP3517EVM)		+= board-am3517evm.o

obj-$(CONFIG_MACH_SBC3530)		+= board-omap3stalker.o \
					   hsmmc.o
# Platform specific device init code
obj-y					+= usb-musb.o
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o
obj-y					+= usb-ehci.o

onenand-$(CONFIG_MTD_ONENAND_OMAP2)	:= gpmc-onenand.o
obj-y					+= $(onenand-m) $(onenand-y)

nand-$(CONFIG_MTD_NAND_OMAP2)		:= gpmc-nand.o
obj-y					+= $(nand-m) $(nand-y)

smc91x-$(CONFIG_SMC91X)			:= gpmc-smc91x.o
obj-y					+= $(smc91x-m) $(smc91x-y)
obj-$(CONFIG_ARCH_OMAP4)		+= hwspinlocks.o
obj-$(CONFIG_OMAP_HSI_DEVICE)           += omap_hsi.o
