/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/hci_types.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_sync.h"

#include "hal/debug.h"

/**
 * @brief Common entry point for LLL event prepare invocations from ULL.
 *
 * This function will resolve the event priority and invoke the LLL
 * lll_prepare_resolve, which decides if event should be programmed in
 * the radio via the prepare callback function, or queued in the prepare
 * pipeline.
 *
 * @param is_abort_cb   Callback for checking if event is aborted
 * @param abort_cb      Callback for aborting event
 * @param prepare_cb    Callback for event prepare
 * @param event_prio    Priority of event [-128..127]
 * @param prepare_param Prepare data
 *
 * @return 0: Prepare was successfully completed
 *	   1: TICKER_STATUS_FAILURE: Preemption ticker stop error
 *	   2: TICKER_STATUS_BUSY: Preemption ticker stop error
 *	   -EINPROGRESS: Event already in progress and prepare was queued
 */
int lll_prepare(lll_is_abort_cb_t is_abort_cb, lll_abort_cb_t abort_cb,
		lll_prepare_cb_t prepare_cb, int8_t event_prio,
		struct lll_prepare_param *prepare_param)
{
	int err;

#if defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
	int prio = event_prio;
	struct lll_hdr *hdr = prepare_param->param;

	/* Establish priority based on:
	 * 1. Event priority passed to function
	 * 2. Force flag => priority = -127
	 * 3. Score (events terminated- and too late)
	 * 4. Latency (skipped- and programmed latency)
	 * 5. Critical priority is immutable (-128)
	 */
	if (prio > -128) {
		if (prepare_param->force) {
			prio = -127;
		} else {
			prio = MAX(-127, prio - hdr->score - hdr->latency);
		}
	}

	prepare_param->prio = prio;
#endif /* CONFIG_BT_CTLR_JIT_SCHEDULING */

	prepare_param->defer = 0U;

	err = lll_prepare_resolve(is_abort_cb, abort_cb, prepare_cb, prepare_param, 0U, 0U);

	return err;
}

void lll_resume(void *param)
{
	struct lll_event *next;
	int err;

	next = param;
	err = lll_prepare_resolve(next->is_abort_cb, next->abort_cb, next->prepare_cb,
				  &next->prepare_param, next->is_resume, 1U);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

#if defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
void lll_done_score(void *param, uint8_t result)
{
	struct lll_hdr *hdr = param;

	if (!hdr) {
		return;
	}

	if (result == DONE_COMPLETED) {
		hdr->score  = 0;
		hdr->latency = 0;
	} else {
		hdr->score++;
	}
}
#endif /* CONFIG_BT_CTLR_JIT_SCHEDULING */

#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC_CTE_TYPE_FILTERING)
enum sync_status lll_sync_cte_is_allowed(uint8_t cte_type_mask, uint8_t filter_policy,
					 uint8_t rx_cte_time, uint8_t rx_cte_type)
{
	bool cte_ok;

	if (cte_type_mask == BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_NO_FILTERING) {
		return SYNC_STAT_ALLOWED;
	}

	if (rx_cte_time > 0) {
		if ((cte_type_mask & BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_NO_CTE) != 0) {
			cte_ok = false;
		} else {
			switch (rx_cte_type) {
			case BT_HCI_LE_AOA_CTE:
				cte_ok = !(cte_type_mask &
					   BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_NO_AOA);
				break;
			case BT_HCI_LE_AOD_CTE_1US:
				cte_ok = !(cte_type_mask &
					   BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_NO_AOD_1US);
				break;
			case BT_HCI_LE_AOD_CTE_2US:
				cte_ok = !(cte_type_mask &
					   BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_NO_AOD_2US);
				break;
			default:
				/* Unknown or forbidden CTE type */
				cte_ok = false;
			}
		}
	} else {
		/* No CTEInfo in the PDU. The CTETime of a CTEInfo is in the
		 * range 2 to 20, so 0 tells the two apart.
		 */
		if ((cte_type_mask & BT_HCI_LE_PER_ADV_CREATE_SYNC_CTE_TYPE_ONLY_CTE) != 0) {
			cte_ok = false;
		} else {
			cte_ok = true;
		}
	}

	if (!cte_ok) {
		return filter_policy ? SYNC_STAT_CONT_SCAN : SYNC_STAT_TERM;
	}

	return SYNC_STAT_ALLOWED;
}
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC_CTE_TYPE_FILTERING */
