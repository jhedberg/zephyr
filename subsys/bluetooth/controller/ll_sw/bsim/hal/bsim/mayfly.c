/*
 * Copyright (c) 2016-2019 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include "hal/swi.h"

#include "util/memq.h"
#include "util/mayfly.h"

#include "ll_sw/lll.h"

#include "hal/debug.h"

#define MAYFLY_CALL_ID_LLL    TICKER_USER_ID_LLL
#define MAYFLY_CALL_ID_WORKER TICKER_USER_ID_ULL_HIGH
#define MAYFLY_CALL_ID_JOB    TICKER_USER_ID_ULL_LOW

void mayfly_enable_cb(uint8_t caller_id, uint8_t callee_id, uint8_t enable)
{
	ARG_UNUSED(caller_id);

	switch (callee_id) {
	case MAYFLY_CALL_ID_WORKER:
		if (enable != 0U) {
			irq_enable(HAL_SWI_WORKER_IRQ);
		} else {
			irq_disable(HAL_SWI_WORKER_IRQ);
		}
		break;

	case MAYFLY_CALL_ID_JOB:
		if (enable != 0U) {
			irq_enable(HAL_SWI_JOB_IRQ);
		} else {
			irq_disable(HAL_SWI_JOB_IRQ);
		}
		break;

	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

uint32_t mayfly_is_enabled(uint8_t caller_id, uint8_t callee_id)
{
	ARG_UNUSED(caller_id);

	switch (callee_id) {
	case MAYFLY_CALL_ID_LLL:
		return irq_is_enabled(HAL_SWI_RADIO_IRQ);

	case MAYFLY_CALL_ID_WORKER:
		return irq_is_enabled(HAL_SWI_WORKER_IRQ);

	case MAYFLY_CALL_ID_JOB:
		return irq_is_enabled(HAL_SWI_JOB_IRQ);

	default:
		LL_ASSERT_DBG(0);
		break;
	}

	return 0;
}

static bool is_pair(uint8_t caller_id, uint8_t callee_id, uint8_t a, uint8_t b)
{
	return ((caller_id == a) && (callee_id == b)) ||
	       ((caller_id == b) && (callee_id == a));
}

uint32_t mayfly_prio_is_equal(uint8_t caller_id, uint8_t callee_id)
{
	/* With ZLI the LLL runs above every other priority */
	return (caller_id == callee_id) ||
	       (!IS_ENABLED(CONFIG_BT_CTLR_ZLI) &&
		(CONFIG_BT_CTLR_LLL_PRIO == CONFIG_BT_CTLR_ULL_HIGH_PRIO) &&
		is_pair(caller_id, callee_id, MAYFLY_CALL_ID_LLL, MAYFLY_CALL_ID_WORKER)) ||
	       (!IS_ENABLED(CONFIG_BT_CTLR_ZLI) &&
		(CONFIG_BT_CTLR_LLL_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO) &&
		is_pair(caller_id, callee_id, MAYFLY_CALL_ID_LLL, MAYFLY_CALL_ID_JOB)) ||
	       ((CONFIG_BT_CTLR_ULL_HIGH_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO) &&
		is_pair(caller_id, callee_id, MAYFLY_CALL_ID_WORKER, MAYFLY_CALL_ID_JOB));
}

void mayfly_pend(uint8_t caller_id, uint8_t callee_id)
{
	ARG_UNUSED(caller_id);

	switch (callee_id) {
	case MAYFLY_CALL_ID_LLL:
		hal_swi_lll_pend();
		break;

	case MAYFLY_CALL_ID_WORKER:
		hal_swi_worker_pend();
		break;

	case MAYFLY_CALL_ID_JOB:
		hal_swi_job_pend();
		break;

	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

uint32_t mayfly_is_running(void)
{
	return k_is_in_isr();
}
