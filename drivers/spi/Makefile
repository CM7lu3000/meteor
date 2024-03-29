#
# Makefile for kernel SPI drivers.
#

ifeq ($(CONFIG_SPI_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

# small core, mostly translating board-specific
# config declarations into driver model code
obj-$(CONFIG_SPI_MASTER)		+= spi.o

# SPI master controller drivers (bus)
obj-$(CONFIG_SPI_ATMEL)			+= atmel_spi.o
obj-$(CONFIG_SPI_BFIN)			+= spi_bfin5xx.o
obj-$(CONFIG_SPI_BITBANG)		+= spi_bitbang.o
obj-$(CONFIG_SPI_AU1550)		+= au1550_spi.o
obj-$(CONFIG_SPI_BUTTERFLY)		+= spi_butterfly.o
obj-$(CONFIG_SPI_COLDFIRE_QSPI)		+= coldfire_qspi.o
obj-$(CONFIG_SPI_DAVINCI)		+= davinci_spi.o
obj-$(CONFIG_SPI_DESIGNWARE)		+= dw_spi.o
obj-$(CONFIG_SPI_DW_PCI)		+= dw_spi_pci.o
obj-$(CONFIG_SPI_DW_MMIO)		+= dw_spi_mmio.o
obj-$(CONFIG_SPI_EP93XX)		+= ep93xx_spi.o
obj-$(CONFIG_SPI_GPIO)			+= spi_gpio.o
obj-$(CONFIG_SPI_IMX)			+= spi_imx.o
obj-$(CONFIG_SPI_LM70_LLP)		+= spi_lm70llp.o
obj-$(CONFIG_SPI_PXA2XX)		+= pxa2xx_spi.o
obj-$(CONFIG_SPI_OMAP_UWIRE)		+= omap_uwire.o
ifeq ($(CONFIG_LGE_SPI), n)
obj-$(CONFIG_SPI_OMAP24XX)		+= omap2_mcspi.o
endif
obj-$(CONFIG_LGE_SPI_MASTER)	+= omap2_mcspi.o
##--[[ LGE_UBIQUIX_MODIFIED_START : scchoi@ubiquix.com [2011.07.14] - Replaced the justin-froyo's spi driver.
obj-$(CONFIG_LGE_SPI_SLAVE)		+= omap2_mcspi_justin.o
##-]][ LGE_UBIQUIX_MODIFIED_END : scchoi@ubiquix.com [2011.07.14] - Replaced the justin-froyo's spi driver.
obj-$(CONFIG_SPI_OMAP_100K)		+= omap_spi_100k.o
obj-$(CONFIG_SPI_ORION)			+= orion_spi.o
obj-$(CONFIG_SPI_PL022)			+= amba-pl022.o
obj-$(CONFIG_SPI_MPC512x_PSC)		+= mpc512x_psc_spi.o
obj-$(CONFIG_SPI_MPC52xx_PSC)		+= mpc52xx_psc_spi.o
obj-$(CONFIG_SPI_MPC52xx)		+= mpc52xx_spi.o
obj-$(CONFIG_SPI_MPC8xxx)		+= spi_mpc8xxx.o
obj-$(CONFIG_SPI_PPC4xx)		+= spi_ppc4xx.o
obj-$(CONFIG_SPI_S3C24XX_GPIO)		+= spi_s3c24xx_gpio.o
obj-$(CONFIG_SPI_S3C24XX)		+= spi_s3c24xx_hw.o
obj-$(CONFIG_SPI_S3C64XX)		+= spi_s3c64xx.o
obj-$(CONFIG_SPI_TXX9)			+= spi_txx9.o
obj-$(CONFIG_SPI_XILINX)		+= xilinx_spi.o
obj-$(CONFIG_SPI_XILINX_OF)		+= xilinx_spi_of.o
obj-$(CONFIG_SPI_XILINX_PLTFM)		+= xilinx_spi_pltfm.o
obj-$(CONFIG_SPI_SH_SCI)		+= spi_sh_sci.o
obj-$(CONFIG_SPI_SH_MSIOF)		+= spi_sh_msiof.o
obj-$(CONFIG_SPI_STMP3XXX)		+= spi_stmp.o
obj-$(CONFIG_SPI_NUC900)		+= spi_nuc900.o

# special build for s3c24xx spi driver with fiq support
spi_s3c24xx_hw-y			:= spi_s3c24xx.o
spi_s3c24xx_hw-$(CONFIG_SPI_S3C24XX_FIQ) += spi_s3c24xx_fiq.o

# 	... add above this line ...

# SPI protocol drivers (device/link on bus)
# //LGE_UPDATE_S [eungbo.shim@lge.com] 20100709
obj-$(CONFIG_LGE_SPI_MASTER)		+= ifx_n721_spi.o
#obj-$(CONFIG_LGE_SPI_SLAVE)		+= ifx_n721_spi_slave.o
# //LGE_UPDATS_E [eungbo.shim@lge.com] 20100709
#//--[[ LGE_UBIQUIX_MODIFIED_START : [scchoi@ubiquix.com] - Merged MDM6600 driver(based on justin), TODO: Add ifx feature.
obj-$(CONFIG_SPI_MDM6600)			+= mdm_6600_spi.o
#//--[[ LGE_UBIQUIX_MODIFIED_END : [scchoi@ubiquix.com] - Merged MDM6600 driver(based on justin), TODO: Add ifx feature.

obj-$(CONFIG_SPI_SPIDEV)	+= spidev.o
obj-$(CONFIG_SPI_TLE62X0)	+= tle62x0.o
# 	... add above this line ...

# SPI slave controller drivers (upstream link)
# 	... add above this line ...

# SPI slave drivers (protocol for that link)
# 	... add above this line ...
