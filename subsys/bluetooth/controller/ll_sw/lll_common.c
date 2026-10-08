/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <limits.h>
#include <string.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/hci_types.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "hal/ccm.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_clock.h"
#include "lll/lll_df_types.h"
#include "lll_sync.h"
#include "lll_conn.h"
#include "lll_scan.h"
#include "lll_scan_aux.h"
#include "lll_filter.h"

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

#if defined(CONFIG_BT_OBSERVER)
bool lll_scan_isr_rx_check(const struct lll_scan *lll, uint8_t irkmatch_ok,
			   uint8_t devmatch_ok, uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	return (((lll->filter_policy & SCAN_FP_FILTER) == 0U) &&
		(!devmatch_ok || ull_filter_lll_rl_idx_allowed(irkmatch_ok,
							       rl_idx))) ||
	       (((lll->filter_policy & SCAN_FP_FILTER) != 0U) &&
		(devmatch_ok || ull_filter_lll_irk_in_fal(rl_idx)));
#else
	return ((lll->filter_policy & SCAN_FP_FILTER) == 0U) ||
		devmatch_ok;
#endif /* CONFIG_BT_CTLR_PRIVACY */
}

bool lll_scan_adva_check(const struct lll_scan *lll, uint8_t addr_type,
			 const uint8_t *addr, uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* Only applies to initiator with no filter accept list */
	if (rl_idx != FILTER_IDX_NONE) {
		return (rl_idx == lll->rl_idx);
	} else if (!ull_filter_lll_rl_addr_allowed(addr_type, addr, &rl_idx)) {
		return false;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	/* NOTE: This function to be used only to check AdvA when initiating,
	 *       hence, otherwise we should not use the return value.
	 *       This function is referenced in lll_scan_ext_tgta_check, but
	 *       is not used when not being an initiator, hence return false
	 *       is never reached.
	 */
#if defined(CONFIG_BT_CENTRAL)
	return ((lll->adv_addr_type == addr_type) &&
		!memcmp(lll->adv_addr, addr, BDADDR_SIZE));
#else /* CONFIG_BT_CENTRAL */
	return false;
#endif /* CONFIG_BT_CENTRAL */
}

static inline bool isr_scan_tgta_rpa_check(const struct lll_scan *lll,
					   uint8_t addr_type,
					   const uint8_t *addr,
					   bool *const dir_report)
{
	if (((lll->filter_policy & SCAN_FP_EXT) != 0U) && (addr_type != 0U) &&
	    ((addr[5] & 0xc0) == 0x40)) {

		if (dir_report) {
			*dir_report = true;
		}

		return true;
	}

	return false;
}

bool lll_scan_tgta_check(const struct lll_scan *lll, bool init,
			 uint8_t addr_type, const uint8_t *addr,
			 uint8_t rl_idx, bool *dir_report)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (ull_filter_lll_rl_addr_resolve(addr_type, addr, rl_idx)) {
		return true;
	} else if (init && lll->rpa_gen && ull_filter_lll_lrpa_get(rl_idx)) {
		/* Initiator generating RPAs, and could not resolve TargetA:
		 * discard
		 */
		return false;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	return (((lll->init_addr_type == addr_type) &&
		 !memcmp(lll->init_addr, addr, BDADDR_SIZE))) ||
	       /* allow directed adv packets where TargetA address
		* is resolvable private address (scanner only)
		*/
	       isr_scan_tgta_rpa_check(lll, addr_type, addr, dir_report);
}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
bool lll_scan_ext_tgta_check(const struct lll_scan *lll, bool pri, bool is_init,
			     const struct pdu_adv *pdu, uint8_t rl_idx,
			     bool *const dir_report)
{
	const struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	const struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	bool is_fal = ((lll->filter_policy & SCAN_FP_FILTER) != 0U);
	uint8_t adva_size = 0U;
	bool is_directed = false;
	bool has_aux = false;

	if (pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN + com_hdr->ext_hdr_len)) {
		return false;
	}

	/* Without an extended header, there are no flags either */
	if (com_hdr->ext_hdr_len != 0U) {
		adva_size = (hdr->adv_addr != 0U) ? ADVA_SIZE : 0U;
		is_directed = (hdr->tgt_addr != 0U);
		has_aux = (hdr->aux_ptr != 0U);

		if (com_hdr->ext_hdr_len < (sizeof(*hdr) + adva_size +
					    (is_directed ? TARGETA_SIZE : 0U))) {
			return false;
		}
	}

	if (adva_size == 0U) {
		/* Other than anonymous advertising has its AdvA in the
		 * AUX_ADV_IND that the AuxPtr of its ADV_EXT_IND points to
		 */
		if (pri && has_aux) {
			return true;
		}

		/* Connectable advertising has its AdvA in the AUX_ADV_IND */
		if (is_init) {
			return false;
		}
	} else if (is_init && !is_fal &&
		   !lll_scan_adva_check(lll, pdu->tx_addr,
					&hdr->data[ADVA_OFFSET], rl_idx)) {
		return false;
	}

	/* TargetA follows the AdvA, if any */
	return !is_directed ||
	       lll_scan_tgta_check(lll, is_init, pdu->rx_addr,
				   &hdr->data[adva_size], rl_idx, dir_report);
}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CENTRAL)
void lll_scan_prepare_connect_req(struct lll_scan *lll, struct pdu_adv *pdu_tx,
				  uint8_t phy, uint32_t pdu_end_us,
				  uint32_t conn_win_offset_us,
				  uint8_t adv_tx_addr, const uint8_t *adv_addr,
				  uint8_t init_tx_addr, const uint8_t *init_addr,
				  uint32_t *conn_space_us)
{
	struct lll_conn *lll_conn;
	uint32_t conn_interval_us;
	uint32_t conn_offset_us;

	lll_conn = lll->conn;

	/* Note: this code is also valid for AUX_CONNECT_REQ */
	pdu_tx->type = PDU_ADV_TYPE_CONNECT_IND;

	/* ChSel is RFU in AUX_CONNECT_REQ, as Channel Selection Algorithm #2
	 * is used anyway.
	 */
	if ((phy == PHY_LEGACY) && IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
		pdu_tx->chan_sel = 1;
	} else {
		pdu_tx->chan_sel = 0;
	}

	pdu_tx->tx_addr = init_tx_addr;
	pdu_tx->rx_addr = adv_tx_addr;
	pdu_tx->len = sizeof(struct pdu_adv_connect_ind);
	memcpy(&pdu_tx->connect_ind.init_addr[0], init_addr, BDADDR_SIZE);
	memcpy(&pdu_tx->connect_ind.adv_addr[0], adv_addr, BDADDR_SIZE);
	memcpy(&pdu_tx->connect_ind.access_addr[0],
	       &lll_conn->access_addr[0], 4);
	memcpy(&pdu_tx->connect_ind.crc_init[0], &lll_conn->crc_init[0], 3);
	pdu_tx->connect_ind.win_size = 1;

	conn_interval_us = (uint32_t)lll_conn->interval * CONN_INT_UNIT_US;
	conn_offset_us = pdu_end_us + EVENT_IFS_US +
			 PDU_AC_MAX_US(sizeof(struct pdu_adv_connect_ind),
				       (phy == PHY_LEGACY) ? PHY_1M : phy);

	/* Add transmitWindowDelay to default calculated connection offset:
	 * 1.25ms for a legacy PDU, 2.5ms for an LE Uncoded PHY and 3.75ms for
	 * an LE Coded PHY.
	 */
	if (0) {
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	} else if (phy) {
		if (phy & PHY_CODED) {
			conn_offset_us += WIN_DELAY_CODED;
		} else {
			conn_offset_us += WIN_DELAY_UNCODED;
		}
#endif
	} else {
		conn_offset_us += WIN_DELAY_LEGACY;
	}

	if (!IS_ENABLED(CONFIG_BT_CTLR_SCHED_ADVANCED) ||
	    lll->conn_win_offset_us == 0U) {
		*conn_space_us = conn_offset_us;
		pdu_tx->connect_ind.win_offset = sys_cpu_to_le16(0);
	} else {
		uint32_t win_offset_us = conn_win_offset_us;

		while ((win_offset_us & ((uint32_t)1 << 31)) ||
		       (win_offset_us < conn_offset_us)) {
			win_offset_us += conn_interval_us;
		}

		*conn_space_us = win_offset_us;
		pdu_tx->connect_ind.win_offset =
			sys_cpu_to_le16((win_offset_us - conn_offset_us) /
					CONN_INT_UNIT_US);
		pdu_tx->connect_ind.win_size++;
	}

	pdu_tx->connect_ind.interval = sys_cpu_to_le16(lll_conn->interval);
	pdu_tx->connect_ind.latency = sys_cpu_to_le16(lll_conn->latency);
	pdu_tx->connect_ind.timeout = sys_cpu_to_le16(lll->conn_timeout);
	memcpy(&pdu_tx->connect_ind.chan_map[0], &lll_conn->data_chan_map[0],
	       sizeof(pdu_tx->connect_ind.chan_map));
	pdu_tx->connect_ind.hop = lll_conn->data_chan_hop;
	pdu_tx->connect_ind.sca = lll_clock_sca_local_get();
}
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_ADV_EXT)
bool lll_scan_aux_connect_rsp_check(const struct lll_scan *lll,
				    const struct pdu_adv *pdu_tx,
				    const struct pdu_adv *pdu_rx, uint8_t rl_idx)
{
	const uint8_t *adva;
	bool is_adva;

	if (unlikely(pdu_rx->type != PDU_ADV_TYPE_AUX_CONNECT_RSP)) {
		return false;
	}

	if (unlikely(pdu_rx->len != (offsetof(struct pdu_adv_com_ext_adv, ext_hdr_adv_data) +
				     offsetof(struct pdu_adv_ext_hdr, data) + ADVA_SIZE +
				     TARGETA_SIZE))) {
		return false;
	}

	if (unlikely(pdu_rx->adv_ext_ind.adv_mode || !pdu_rx->adv_ext_ind.ext_hdr.adv_addr ||
		     !pdu_rx->adv_ext_ind.ext_hdr.tgt_addr)) {
		return false;
	}

	adva = &pdu_rx->adv_ext_ind.ext_hdr.data[ADVA_OFFSET];

	/* With the Filter Accept List, the Host gives no peer address, so the
	 * advertiser is the one the AUX_CONNECT_REQ was sent to.
	 */
	if ((lll->filter_policy & SCAN_FP_FILTER) != 0U) {
		is_adva = (pdu_rx->tx_addr == pdu_tx->rx_addr) &&
			  (memcmp(adva, pdu_tx->connect_ind.adv_addr,
				  BDADDR_SIZE) == 0);
	} else {
		is_adva = lll_scan_adva_check(lll, pdu_rx->tx_addr, adva,
					      rl_idx);
	}

	return is_adva &&
	       (pdu_rx->rx_addr == pdu_tx->tx_addr) &&
	       (memcmp(&pdu_rx->adv_ext_ind.ext_hdr.data[TGTA_OFFSET],
		       pdu_tx->connect_ind.init_addr, BDADDR_SIZE) == 0);
}
#endif /* CONFIG_BT_CENTRAL && CONFIG_BT_CTLR_ADV_EXT */
#endif /* CONFIG_BT_OBSERVER */
