/*
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Lesly A M <leslyam@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TWL4030_POWER

#include "twl4030.h"

#if 0
static struct prm_setup_vc twl4030_voltsetup_time = {
	/* VOLT SETUPTIME for RET */
	.ret = {
		.voltsetup1_vdd1 = 0x005B,
		.voltsetup1_vdd2 = 0x0055,
		.voltsetup2 = 0x0,
		.voltoffset = 0x0,
	},
	/* VOLT SETUPTIME for OFF */
	.off = {
		.voltsetup1_vdd1 = 0x00B3,
		.voltsetup1_vdd2 = 0x00A0,
		.voltsetup2 = 0x118,
		.voltoffset = 0x32,
	},
};
#endif

/*
 * Sequence to control the TRITON Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 */
static struct twl4030_ins __initdata sleep_on_seq[] = {
#if defined(CONFIG_PRODUCT_LGE_HUB)||defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN)
#if 0	//junyeop.kim@lge.com please confirm the power team, check
	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
#endif	       	
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},

	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to reduce current level */
	{MSG_SINGULAR(DEV_GRP_P1, 0xe, RES_STATE_ACTIVE), 0xe},
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [START_LGE]
	//{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_OFF), 0xe},
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [END_LGE]
	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to reduce current level */

	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [START_LGE]
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [START_LGE]
        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_SLEEP), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_SLEEP), 0x37},	
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [END_LGE]
	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [END_LGE]

#else /* CONFIG_PRODUCT_LGE_HUB, B, J */
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
#endif /* CONFIG_PRODUCT_LGE_HUB, B, J */
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1_P2 transition for wakeup.
 */
static struct twl4030_ins wakeup_p12_seq[] __initdata = {
#if defined(CONFIG_PRODUCT_LGE_HUB)||defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN)
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [START_LGE]
	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to wakeup from reduced current level */
	//{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_ACTIVE), 0xe},
	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to wakeup from reduced current level */
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [END_LGE]

	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},

	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [START_LGE]
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [START_LGE]
        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},	
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [END_LGE]
	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [END_LGE]

#else /* CONFIG_PRODUCT_LGE_HUB, B, J */
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
#endif /* CONFIG_PRODUCT_LGE_HUB, B, J */
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P3 transition for wakeup.
 */
static struct twl4030_ins wakeup_p3_seq[] __initdata = {
#if defined(CONFIG_PRODUCT_LGE_HUB)||defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN)
	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to wakeup from reduced current level */
	{MSG_SINGULAR(DEV_GRP_P1, 0x17, RES_STATE_ACTIVE), 0xe},
	/*LGE_CHANGE_S [kyw2029@lge.com] 2009-11-18, to wakeup from reduced current level */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},

	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [START_LGE]
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [START_LGE]
        //{MSG_BROADCAST(DEV_GRP_P3, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},
        {MSG_BROADCAST(DEV_GRP_P3, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_ACTIVE), 0x37},	
	// 20100823 sookyoung.kim@lge.com Fix LDOs to be turned on upon resumption [END_LGE]
	// 20100909 sookyoung.kim@lge.com Change to include CLKEN [END_LGE]

#else /* CONFIG_PRODUCT_LGE_HUB, B, J */
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 2},
#endif /* CONFIG_PRODUCT_LGE_HUB, B, J */
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

/*
 * Sequence to reset the TRITON Power resources,
 * when the system gets warm reset.
 * Executed upon warm reset signal.
 */
static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset Main_Ref.
 * Reset All type2_group2.
 * Reset VUSB_3v1.
 * Reset All type2_group1.
 * Reset RC.
 * Reenable twl4030.
 */
#if defined(CONFIG_PRODUCT_LGE_HUB)||defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN)
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
#else /* CONFIG_PRODUCT_LGE_HUB, B, J */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_Main_Ref, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
#endif /* CONFIG_PRODUCT_LGE_HUB, B, J */
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

/* TRITON script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts[] __initdata = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
#if defined(CONFIG_PRODUCT_LGE_HUB)||defined(CONFIG_PRODUCT_LGE_BLACK)||defined(CONFIG_PRODUCT_LGE_JUSTIN)
	{.resource = RES_HFCLKOUT,.devgroup = DEV_GRP_P3,.type = -1,
	 .type2 = -1},
	{.resource = RES_VDD1,.devgroup = DEV_GRP_P1,.type = -1,
	 .type2 = -1},
	{.resource = RES_VDD2,.devgroup = DEV_GRP_P1,.type = -1,
	 .type2 = -1},
	{.resource = RES_CLKEN,.devgroup = DEV_GRP_P3,.type = -1,
	 .type2 = 1},
#else /* CONFIG_PRODUCT_LGE_HUB, B, J */
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 3,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
		.type = 0, .type2 = 2, .remap_sleep = RES_STATE_SLEEP },
#endif /* CONFIG_PRODUCT_LGE_HUB, B, J */
	{ 0, 0},
};

static struct twl4030_power_data twl4030_generic_script __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

void twl4030_get_scripts(struct twl4030_power_data *t2scripts_data)
{
	t2scripts_data->scripts = twl4030_generic_script.scripts;
	t2scripts_data->num = twl4030_generic_script.num;
	t2scripts_data->resource_config =
			twl4030_generic_script.resource_config;
}

#if 0
void twl4030_get_vc_timings(struct prm_setup_vc *setup_vc)
{
	/* copies new voltsetup time for RERT */
	setup_vc->ret.voltsetup1_vdd1 =
				twl4030_voltsetup_time.ret.voltsetup1_vdd1;
	setup_vc->ret.voltsetup1_vdd2 =
				twl4030_voltsetup_time.ret.voltsetup1_vdd2;
	setup_vc->ret.voltsetup2 = twl4030_voltsetup_time.ret.voltsetup2;
	setup_vc->ret.voltoffset = twl4030_voltsetup_time.ret.voltoffset;

	/* copies new voltsetup time for OFF */
	setup_vc->off.voltsetup1_vdd1 =
				twl4030_voltsetup_time.off.voltsetup1_vdd1;
	setup_vc->off.voltsetup1_vdd2 =
				twl4030_voltsetup_time.off.voltsetup1_vdd2;
	setup_vc->off.voltsetup2 = twl4030_voltsetup_time.off.voltsetup2;
	setup_vc->off.voltoffset = twl4030_voltsetup_time.off.voltoffset;

}
#endif
#endif

