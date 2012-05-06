/*
 * SDRC register values for the SAMSUNG KAT007012M
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_KAT007012M
#define ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_KAT007012M

#include <plat/sdrc.h>

/* SAMSUNG KAT007012M (100MHz optimized) = 12.05ns */
#define TDAL_83   3
#define TDPL_83   1
#define TRRD_83   1
#define TRCD_83   2
#define TRP_83    2
#define TRAS_83   4
#define TRC_83    5
#define TRFC_83   10 
#define V_ACTIMA_83 ((TRFC_83 << 27) | (TRC_83 << 22) | (TRAS_83 << 18) \
		|(TRP_83 << 15) | (TRCD_83 << 12) |(TRRD_83 << 9) |(TDPL_83 << 6) \
		| (TDAL_83))

#define TWTR_83   2
#define TCKE_83   2
#define TXP_83    2
#define XSR_83   10
#define V_ACTIMB_83 ((TCKE_83 << 12) | (XSR_83 << 0)) | \
				(TXP_83 << 8) | (TWTR_83 << 16)

/* (100MHz optimized) = 10ns */
#define TDAL_100   3
#define TDPL_100   2
#define TRRD_100   2
#define TRCD_100   2
#define TRP_100    2
#define TRAS_100   5
#define TRC_100    6
#define TRFC_100   13
#define V_ACTIMA_100 ((TRFC_100 << 27) | (TRC_100 << 22) | (TRAS_100 << 18) \
		|(TRP_100 << 15) | (TRCD_100 << 12) |(TRRD_100 << 9) |(TDPL_100 << 6) \
		| (TDAL_100))

#define TWTR_100   2
#define TCKE_100   2
#define TXP_100    2
#define XSR_100   13
#define V_ACTIMB_100 ((TCKE_100 << 12) | (XSR_100 << 0)) | \
				(TXP_100 << 8) | (TWTR_100 << 16)

/* (133MHz optimized) ~ 7.5ns */
#define TDAL_133   4
#define TDPL_133   2
#define TRRD_133   2
#define TRCD_133   2
#define TRP_133    2
#define TRAS_133   6
#define TRC_133    8
#define TRFC_133  16
#define V_ACTIMA_133 ((TRFC_133 << 27) | (TRC_133 << 22) | (TRAS_133 << 18) \
		|(TRP_133 << 15) | (TRCD_133 << 12) |(TRRD_133 << 9) |(TDPL_133 << 6) \
		| (TDAL_133))

#define TWTR_133   2
#define TCKE_133   2
#define TXP_133    2
#define XSR_133   16
#define V_ACTIMB_133 ((TCKE_133 << 12) | (XSR_133 << 0)) | \
				(TXP_133 << 8) | (TWTR_133 << 16)

/* (166MHz optimized) 6.02ns
 *   ACTIMA
 *	TDAL = Twr/Tck + Trp/tck = 12/6 + 18/6 = 2 + 3 = 5
 *	TDPL (Twr) = 12/6	= 2
 *	TRRD = 12/6	= 2
 *	TRCD = 18/6	= 3
 *	TRP = 18/6	= 3
 *	TRAS = 42/6	= 7
 *	TRC = 60/6	= 10
 *	TRFC = 72/6	= 12
 *   ACTIMB
 *	TCKE = 2
 *	XSR = 120/6 = 20
 */
#define TDAL_165   5
#define TDPL_165   2
#define TRRD_165   2
#define TRCD_165   3
#define TRP_165    3
#define TRAS_165   7
#define TRC_165   10
#define TRFC_165  20
#define V_ACTIMA_165 ((TRFC_165 << 27) | (TRC_165 << 22) | (TRAS_165 << 18) \
		| (TRP_165 << 15) | (TRCD_165 << 12) |(TRRD_165 << 9) | \
		(TDPL_165 << 6) | (TDAL_165))

#define TWTR_165   2
#define TCKE_165   2
#define TXP_165    2
#define XSR_165    20
#define V_ACTIMB_165 ((TCKE_165 << 12) | (XSR_165 << 0)) | \
				(TXP_165 << 8) | (TWTR_165 << 16)

#define TDAL_200   6
#define TDPL_200   3
#define TRRD_200   3
#define TRCD_200   4
#define TRP_200    4
#define TRAS_200   9
#define TRC_200   12
#define TRFC_200  25
#define V_ACTIMA_200 ((TRFC_200 << 27) | (TRC_200 << 22) | (TRAS_200 << 18) \
		| (TRP_200 << 15) | (TRCD_200 << 12) |(TRRD_200 << 9) | \
		(TDPL_200 << 6) | (TDAL_200))
#define TWTR_200   2
#define TCKE_200   2
#define TXP_200    2
#define XSR_200    25
#define V_ACTIMB_200 ((TCKE_200 << 12) | (XSR_200 << 0)) | \
				(TXP_200 << 8) | (TWTR_200 << 16)

#define SDP_3430_SDRC_RFR_CTRL_100MHz   0x0002da01
#define SDP_3430_SDRC_RFR_CTRL_133MHz   0x0003de01 /* 7.8us/7.5ns - 50=0x3de */
#define SDP_3430_SDRC_RFR_CTRL_165MHz   0x0004e201 /* 7.8us/6ns - 50=0x4e2 */

#define HUB_3630_SDRC_RFR_CTRL_83MHz    0x00025601
#define HUB_3630_SDRC_RFR_CTRL_100MHz   0x0002da01
#define HUB_3630_SDRC_RFR_CTRL_133MHz   0x0003dc01 /* 7.8us/7.5ns - 50=0x3dc */
#define HUB_3630_SDRC_RFR_CTRL_165MHz   0x0004dd01 /* 7.8us/6ns - 50=0x4dd */
#define HUB_3630_SDRC_RFR_CTRL_200MHz   0x0005e601

#define HUB_SDRC_MR_0_DDR		0x00000032
#define SDP_SDRC_MR_0_DDR		0x00000032

static struct omap_sdrc_params kat007012m_sdrc_params[] = {
	[0] = {
		.rate        = 200000000,
		.actim_ctrla = V_ACTIMA_200,
		.actim_ctrlb = V_ACTIMB_200,
		.rfr_ctrl    = HUB_3630_SDRC_RFR_CTRL_200MHz,
		.mr          = HUB_SDRC_MR_0_DDR,
	},
// 20100525 sookyoung.kim@lge.com TI patch for video stalling after resumption [START_LGE]
// Change was to reactivate rates 100000000 and 83000000 below.
/* skykrkrk@lge.com 2010-02-10, temporarily comment out for DVFS bug.
	[1] = {
		.rate        = 166000000,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = HUB_3630_SDRC_RFR_CTRL_165MHz,
		.mr	     = HUB_SDRC_MR_0_DDR,
	},
*/
// 20100525 sookyoung.kim@lge.com TI patch for video stalling after resumption [END_LGE]
// 20100912 sookyoung.kim@lge.com Correct index numbers to enable DVFS [START_LGE]
	[1] = {
		.rate        = 100000000,
		.actim_ctrla = V_ACTIMA_100,
		.actim_ctrlb = V_ACTIMB_100,
		.rfr_ctrl    = HUB_3630_SDRC_RFR_CTRL_100MHz,
		.mr	     = HUB_SDRC_MR_0_DDR,
	},
#if 0 // usnoh@ubiquix.com
	[2] = {
		.rate        = 83000000,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = HUB_3630_SDRC_RFR_CTRL_83MHz,
		.mr          = HUB_SDRC_MR_0_DDR,
	},
	[3] = {
		.rate        = 0
	},
#else
	[2] = {
		.rate        = 0
	},
#endif
// 20100912 sookyoung.kim@lge.com Correct index numbers to enable DVFS [END_LGE]
};

#endif

