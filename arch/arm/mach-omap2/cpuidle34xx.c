/*
 * linux/arch/arm/mach-omap2/cpuidle34xx.c
 *
 * OMAP3 CPU IDLE Routines
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>

#include <plat/prcm.h>
#include <plat/irqs.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/control.h>
#include <plat/serial.h>

#include "pm.h"

#ifdef CONFIG_CPU_IDLE

#define OMAP3_MAX_STATES 7
#define OMAP3_STATE_C1 0 /* C1 - MPU WFI + Core active */
#define OMAP3_STATE_C2 1 /* C2 - MPU WFI + Core inactive */
#define OMAP3_STATE_C3 2 /* C3 - MPU CSWR + Core inactive */
#define OMAP3_STATE_C4 3 /* C4 - MPU OFF + Core iactive */
#define OMAP3_STATE_C5 4 /* C5 - MPU RET + Core RET */
#define OMAP3_STATE_C6 5 /* C6 - MPU OFF + Core RET */
#define OMAP3_STATE_C7 6 /* C7 - MPU OFF + Core OFF */

#define OMAP3_STATE_MAX OMAP3_STATE_C7

struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
	u32 threshold;
	u32 flags;
};

struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES];
struct omap3_processor_cx current_cx_state;
struct powerdomain *mpu_pd, *core_pd;

/*
 * The latencies/thresholds for various C states have
 * to be configured from the respective board files.
 * These are some default values (which might not provide
 * the best power savings) used on boards which do not
 * pass these details from the board file.
 */
static struct cpuidle_params cpuidle_params_table[] = {
	/* C1 */
	{1, 2, 12, 20},
	//{1, 2, 2, 5},
	/* C2 */
	{1, 2, 18, 25},
	//{1, 10, 10, 30},
	/* C3 */
	{1, 50, 50, 300},
	/* C4 */
	{1, 1500, 1800, 4000},
	/* C5 */
	{1, 2500, 7500, 12000},
	/* C6 */
	{1, 3000, 8500, 15000},
	/* C7 */
	{1, 10000, 30000, 300000},
};

static int omap3_idle_bm_check(void)
{
	if (!omap3_can_sleep())
		return 1;
	return 0;
}

static int _cpuidle_allow_idle(struct powerdomain *pwrdm,
				struct clockdomain *clkdm)
{
	omap2_clkdm_allow_idle(clkdm);
	return 0;
}

static int _cpuidle_deny_idle(struct powerdomain *pwrdm,
				struct clockdomain *clkdm)
{
	omap2_clkdm_deny_idle(clkdm);
	return 0;
}

/**
 * omap3_enter_idle - Programs OMAP3 to enter the specified state
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */

#if defined(CONFIG_PRODUCT_LGE_JUSTIN)
u32 doing_wakeup;
#endif
static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	u32 mpu_state = cx->mpu_state, core_state = cx->core_state;

	current_cx_state = *cx;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();

	pwrdm_set_next_pwrst(mpu_pd, mpu_state);
	pwrdm_set_next_pwrst(core_pd, core_state);

	if (omap_irq_pending() || need_resched())
		goto return_sleep_time;

	if (cx->type == OMAP3_STATE_C1 || cx->type == OMAP3_STATE_C2) {
		pwrdm_for_each_clkdm(mpu_pd, _cpuidle_deny_idle);
		pwrdm_for_each_clkdm(core_pd, _cpuidle_deny_idle);
	}

	/* Execute ARM wfi */
	omap_sram_idle();

	if (cx->type == OMAP3_STATE_C1 || cx->type == OMAP3_STATE_C2) {
		pwrdm_for_each_clkdm(mpu_pd, _cpuidle_allow_idle);
		pwrdm_for_each_clkdm(core_pd, _cpuidle_allow_idle);
	}

return_sleep_time:
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	local_irq_enable();
	local_fiq_enable();

	return ts_idle.tv_nsec / NSEC_PER_USEC + ts_idle.tv_sec * USEC_PER_SEC;
}

/**
 * next_valid_state - Find next valid c-state
 * @dev: cpuidle device
 * @state: Currently selected c-state
 *
 * If the current state is valid, it is returned back to the caller.
 * Else, this function searches for a lower c-state which is still
 * valid (as defined in omap3_power_states[]).
 */
static struct cpuidle_state *next_valid_state(struct cpuidle_device *dev,
						struct cpuidle_state *curr)
{
	struct cpuidle_state *next = NULL;
	struct omap3_processor_cx *cx;

	cx = (struct omap3_processor_cx *)cpuidle_get_statedata(curr);

	/* Check if current state is valid */
	if (cx->valid) {
		return curr;
	} else {
		u8 idx = OMAP3_STATE_MAX;

		/*
		 * Reach the current state starting at highest C-state
		 */
		for (; idx >= OMAP3_STATE_C1; idx--) {
			if (&dev->states[idx] == curr) {
				next = &dev->states[idx];
				break;
			}
		}

		/*
		 * Should never hit this condition.
		 */
		WARN_ON(next == NULL);

		/*
		 * Drop to next valid state.
		 * Start search from the next (lower) state.
		 */
		idx--;
		for (; idx >= OMAP3_STATE_C1; idx--) {
			struct omap3_processor_cx *cx;

			cx = cpuidle_get_statedata(&dev->states[idx]);
			if (cx->valid) {
				next = &dev->states[idx];
				break;
			}
		}
		/*
		 * C1 and C2 are always valid.
		 * So, no need to check for 'next==NULL' outside this loop.
		 */
	}

	return next;
}

/**
 * omap3_enter_idle_bm - Checks for any bus activity
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Used for C states with CPUIDLE_FLAG_CHECK_BM flag set. This
 * function checks for any pending activity and then programs the
 * device to the specified or a safer state.
 */
static int omap3_enter_idle_bm(struct cpuidle_device *dev,
			       struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = next_valid_state(dev, state);

	if ((state->flags & CPUIDLE_FLAG_CHECK_BM) && omap3_idle_bm_check()) {
		BUG_ON(!dev->safe_state);
		new_state = dev->safe_state;
	}

	dev->last_state = new_state;
	return omap3_enter_idle(dev, new_state);
}

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

/**
 * omap3_cpuidle_update_states - Update the cpuidle states.
 *
 * Currently, this function toggles the validity of idle states based upon
 * the flag 'enable_off_mode'. When the flag is set all states are valid.
 * Else, states leading to OFF state set to be invalid.
 */
void omap3_cpuidle_update_states(void)
{
	int i;

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		struct omap3_processor_cx *cx = &omap3_power_states[i];

		if (enable_off_mode) {
			cx->valid = 1;
		} else {
			if ((cx->mpu_state == PWRDM_POWER_OFF) ||
				(cx->core_state	== PWRDM_POWER_OFF))
				cx->valid = 0;
		}
	}
}

void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params)
{
	int i;

	if (!cpuidle_board_params)
		return;

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		cpuidle_params_table[i].valid =
			cpuidle_board_params[i].valid;
		cpuidle_params_table[i].sleep_latency =
			cpuidle_board_params[i].sleep_latency;
		cpuidle_params_table[i].wake_latency =
			cpuidle_board_params[i].wake_latency;
		cpuidle_params_table[i].threshold =
			cpuidle_board_params[i].threshold;
	}
	return;
}

/* omap3_init_power_states - Initialises the OMAP3 specific C states.
 *
 * Below is the desciption of each C state.
 * 	C1 . MPU WFI + Core active
 *	C2 . MPU WFI + Core inactive
 *	C3 . MPU CSWR + Core inactive
 *	C4 . MPU OFF + Core inactive
 *	C5 . MPU CSWR + Core CSWR
 *	C6 . MPU OFF + Core CSWR
 *	C7 . MPU OFF + Core OFF
 */
void omap_init_power_states(void)
{
	/* C1 . MPU WFI + Core active */
	omap3_power_states[OMAP3_STATE_C1].valid =
			cpuidle_params_table[OMAP3_STATE_C1].valid;
	omap3_power_states[OMAP3_STATE_C1].type = OMAP3_STATE_C1;
	omap3_power_states[OMAP3_STATE_C1].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C1].sleep_latency;
	omap3_power_states[OMAP3_STATE_C1].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C1].wake_latency;
	omap3_power_states[OMAP3_STATE_C1].threshold =
			cpuidle_params_table[OMAP3_STATE_C1].threshold;
	omap3_power_states[OMAP3_STATE_C1].mpu_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C1].core_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C1].flags = CPUIDLE_FLAG_TIME_VALID;

	/* C2 . MPU WFI + Core inactive */
	omap3_power_states[OMAP3_STATE_C2].valid =
			cpuidle_params_table[OMAP3_STATE_C2].valid;
	omap3_power_states[OMAP3_STATE_C2].type = OMAP3_STATE_C2;
	omap3_power_states[OMAP3_STATE_C2].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C2].sleep_latency;
	omap3_power_states[OMAP3_STATE_C2].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C2].wake_latency;
	omap3_power_states[OMAP3_STATE_C2].threshold =
			cpuidle_params_table[OMAP3_STATE_C2].threshold;
	omap3_power_states[OMAP3_STATE_C2].mpu_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C2].core_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C2].flags = CPUIDLE_FLAG_TIME_VALID;

	/* C3 . MPU CSWR + Core inactive */
	omap3_power_states[OMAP3_STATE_C3].valid =
			cpuidle_params_table[OMAP3_STATE_C3].valid;
	omap3_power_states[OMAP3_STATE_C3].type = OMAP3_STATE_C3;
	omap3_power_states[OMAP3_STATE_C3].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C3].sleep_latency;
	omap3_power_states[OMAP3_STATE_C3].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C3].wake_latency;
	omap3_power_states[OMAP3_STATE_C3].threshold =
			cpuidle_params_table[OMAP3_STATE_C3].threshold;
	omap3_power_states[OMAP3_STATE_C3].mpu_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C3].core_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C3].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C4 . MPU OFF + Core inactive */
	omap3_power_states[OMAP3_STATE_C4].valid =
			cpuidle_params_table[OMAP3_STATE_C4].valid;
	omap3_power_states[OMAP3_STATE_C4].type = OMAP3_STATE_C4;
	omap3_power_states[OMAP3_STATE_C4].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C4].sleep_latency;
	omap3_power_states[OMAP3_STATE_C4].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C4].wake_latency;
	omap3_power_states[OMAP3_STATE_C4].threshold =
			cpuidle_params_table[OMAP3_STATE_C4].threshold;
	omap3_power_states[OMAP3_STATE_C4].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C4].core_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C4].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C5 . MPU CSWR + Core CSWR*/
	omap3_power_states[OMAP3_STATE_C5].valid =
			cpuidle_params_table[OMAP3_STATE_C5].valid;
	omap3_power_states[OMAP3_STATE_C5].type = OMAP3_STATE_C5;
	omap3_power_states[OMAP3_STATE_C5].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C5].sleep_latency;
	omap3_power_states[OMAP3_STATE_C5].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C5].wake_latency;
	omap3_power_states[OMAP3_STATE_C5].threshold =
			cpuidle_params_table[OMAP3_STATE_C5].threshold;
	omap3_power_states[OMAP3_STATE_C5].mpu_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C6 . MPU OFF + Core CSWR */
	omap3_power_states[OMAP3_STATE_C6].valid =
			cpuidle_params_table[OMAP3_STATE_C6].valid;
	omap3_power_states[OMAP3_STATE_C6].type = OMAP3_STATE_C6;
	omap3_power_states[OMAP3_STATE_C6].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C6].sleep_latency;
	omap3_power_states[OMAP3_STATE_C6].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C6].wake_latency;
	omap3_power_states[OMAP3_STATE_C6].threshold =
			cpuidle_params_table[OMAP3_STATE_C6].threshold;
	omap3_power_states[OMAP3_STATE_C6].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C6].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C6].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C7 . MPU OFF + Core OFF */
	omap3_power_states[OMAP3_STATE_C7].valid =
			cpuidle_params_table[OMAP3_STATE_C7].valid;
	omap3_power_states[OMAP3_STATE_C7].type = OMAP3_STATE_C7;
	omap3_power_states[OMAP3_STATE_C7].sleep_latency =
			cpuidle_params_table[OMAP3_STATE_C7].sleep_latency;
	omap3_power_states[OMAP3_STATE_C7].wakeup_latency =
			cpuidle_params_table[OMAP3_STATE_C7].wake_latency;
	omap3_power_states[OMAP3_STATE_C7].threshold =
			cpuidle_params_table[OMAP3_STATE_C7].threshold;
	omap3_power_states[OMAP3_STATE_C7].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].core_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;
}

struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};

/**
 * omap3_idle_init - Init routine for OMAP3 idle
 *
 * Registers the OMAP3 specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	core_pd = pwrdm_lookup("core_pwrdm");

	omap_init_power_states();
	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
			omap3_enter_idle_bm : omap3_enter_idle;
		if (cx->type == OMAP3_STATE_C1)
			dev->safe_state = state;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;

	omap3_cpuidle_update_states();

	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
		       __func__);
		return -EIO;
	}

	return 0;
}
#else
int __init omap3_idle_init(void)
{
	return 0;
}
#endif /* CONFIG_CPU_IDLE */
