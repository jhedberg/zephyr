/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include "bs_types.h"
#include "irq_ctrl.h"
#include "nsi_cpu0_interrupts.h"
#include "nsi_cpun_if.h"
#include "nsi_hw_scheduler.h"
#include "nsi_hws_models_if.h"
#include "bstest_ticker.h"

/* Next time to wake the CPU, for a tick or for awake_cpu_asap */
static bs_time_t ticker_timer = TIME_NEVER;
static bs_time_t tick_time = TIME_NEVER;
static bs_time_t tick_period = TIME_NEVER;
static bool awake_cpu_asap;

static void ticker_timer_update(void)
{
	if (awake_cpu_asap) {
		ticker_timer = nsi_hws_get_time();
	} else {
		ticker_timer = tick_time;
	}

	nsi_hws_find_next_event();
}

void bst_ticker_amp_set_period(unsigned int inst, bs_time_t period)
{
	(void)inst;

	tick_period = period;
	tick_time = nsi_hws_get_time() + period;
	ticker_timer_update();
}

void bst_ticker_amp_set_next_tick_absolutelute(unsigned int inst, bs_time_t absolute_time)
{
	(void)inst;

	tick_time = absolute_time;
	ticker_timer_update();
}

void bst_ticker_amp_set_next_tick_delta(unsigned int inst, bs_time_t delta_time)
{
	(void)inst;

	tick_time = nsi_hws_get_time() + delta_time;
	ticker_timer_update();
}

void bst_ticker_amp_awake_cpu_asap(unsigned int inst)
{
	(void)inst;

	awake_cpu_asap = true;
	ticker_timer_update();
}

static void ticker_timer_triggered(void)
{
	if (awake_cpu_asap) {
		awake_cpu_asap = false;
		hw_irq_ctrl_raise_im(PHONY_HARD_IRQ);
	} else {
		if (tick_period != TIME_NEVER) {
			tick_time = nsi_hws_get_time() + tick_period;
		} else {
			tick_time = TIME_NEVER;
		}

		(void)nsif_cpun_test_hook(0, NULL);
	}

	ticker_timer_update();
}

/* Right after the timer of the native simulator, as in the nRF HW models */
NSI_HW_EVENT(ticker_timer, ticker_timer_triggered, 1);
