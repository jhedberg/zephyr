/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/dbuf.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_df_types.h"
#include "lll_scan.h"
#include "lll_conn.h"
#include "lll_filter.h"
#include "lll_sched.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_addr.h"
#include "lll_scan_internal.h"

#include "hal/debug.h"

#define ADV_CHAN_MAX 3U

#define BACKOFF_UPPER_LIMIT_MAX 256U

static void isr_rx(const struct bsr_evt *e, void *param);

static struct {
	struct bsr_pkt_cfg cfg;
	const struct lll_filter *filter;
	bool resolve;

	/* The times reported to the ULL are relative to the current scan
	 * window.
	 */
	uint32_t ticks_ref;

	struct pdu_adv pdu_tx;
} evt;

/* Backoff of the scan requests, which spaces out the requests of scanners
 * that collide (Core Spec Vol 6, Part B, Section 4.4.3.2). It only starts over
 * on reset, as the LLL does not see the scanning being enabled.
 */
static struct {
	uint16_t upper_limit;
	uint16_t count;
	uint8_t successes;
	uint8_t failures;
} backoff;

static void backoff_reset(void)
{
	backoff.upper_limit = 1U;
	backoff.count = 1U;
	backoff.successes = 0U;
	backoff.failures = 0U;
}

int lll_scan_init(void)
{
	backoff_reset();

	return 0;
}

int lll_scan_reset(void)
{
	backoff_reset();

	return 0;
}

/* Returns true if a request is to be sent at this opportunity */
bool lll_scan_backoff_is_req(void)
{
	if (backoff.count > 1U) {
		backoff.count--;

		return false;
	}

	return true;
}

/* Two consecutive successes halve the upper limit, two consecutive failures
 * double it.
 */
void lll_scan_backoff_result(bool is_rsp)
{
	uint16_t rand_count = 0U;

	if (is_rsp) {
		backoff.failures = 0U;
		backoff.successes++;
	} else {
		backoff.successes = 0U;
		backoff.failures++;
	}

	if (backoff.successes == 2U) {
		backoff.successes = 0U;
		backoff.upper_limit = MAX(backoff.upper_limit / 2U, 1U);
	} else if (backoff.failures == 2U) {
		backoff.failures = 0U;
		backoff.upper_limit = MIN(backoff.upper_limit * 2U, BACKOFF_UPPER_LIMIT_MAX);
	}

	(void)lll_rand_isr_get(&rand_count, sizeof(rand_count));
	backoff.count = (rand_count % backoff.upper_limit) + 1U;
}

static void ticker_stop_cb(uint32_t ticks_at_expire, uint32_t ticks_drift, uint32_t remainder,
			   uint16_t lazy, uint8_t force, void *param)
{
	static memq_link_t link;
	static struct mayfly mfy = { 0, 0, &link, NULL, lll_disable };
	uint32_t ret;

	mfy.param = param;
	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL, 0U, &mfy);
	LL_ASSERT_ERR(ret == 0U);
}

static void ticker_op_start_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT_ERR(status == TICKER_STATUS_SUCCESS);
}

void lll_scan_filter_get(const struct lll_scan *lll, const struct lll_filter **filter,
			 bool *resolve)
{
	*filter = NULL;
	*resolve = false;

	if (IS_ENABLED(CONFIG_BT_CTLR_PRIVACY) && ull_filter_lll_rl_enabled()) {
		*filter = ull_filter_lll_get((lll->filter_policy & SCAN_FP_FILTER) != 0U);
		*resolve = true;
	} else if (IS_ENABLED(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && (lll->filter_policy != 0U)) {
		*filter = ull_filter_lll_get(true);
	}
}

uint8_t lll_scan_rl_idx_get(const struct lll_scan *lll, const struct lll_addr_match *match)
{
	if (!IS_ENABLED(CONFIG_BT_CTLR_PRIVACY)) {
		return FILTER_IDX_NONE;
	}

	if (match->devmatch_ok != 0U) {
		return ull_filter_lll_rl_idx((lll->filter_policy & SCAN_FP_FILTER) != 0U,
					     match->devmatch_id);
	}

	if (match->irkmatch_ok != 0U) {
		return ull_filter_lll_rl_irk_idx(match->irkmatch_id);
	}

	return FILTER_IDX_NONE;
}

bool lll_scan_isr_rx_filter(const struct lll_scan *lll, struct lll_addr_match *match,
			    uint8_t rl_idx)
{
	bool allow;

	allow = lll_scan_isr_rx_check(lll, match->irkmatch_ok, match->devmatch_ok, rl_idx);

#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC) && defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	match->devmatch_ok = allow;

	return allow || (lll->is_sync != 0U);
#else /* !CONFIG_BT_CTLR_SYNC_PERIODIC || !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
	return allow;
#endif /* !CONFIG_BT_CTLR_SYNC_PERIODIC || !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
}

void lll_scan_prepare_scan_req(const struct lll_scan *lll, struct pdu_adv *pdu_tx,
			       uint8_t adv_tx_addr, const uint8_t *adv_addr, uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

	/* AUX_SCAN_REQ is the same as SCAN_REQ */
	pdu_tx->type = PDU_ADV_TYPE_SCAN_REQ;
	pdu_tx->rfu = 0U;
	pdu_tx->chan_sel = 0U;
	pdu_tx->tx_addr = lll->init_addr_type;
	pdu_tx->rx_addr = adv_tx_addr;
	pdu_tx->len = sizeof(struct pdu_adv_scan_req);
	(void)memcpy(&pdu_tx->scan_req.scan_addr[0], &lll->init_addr[0], BDADDR_SIZE);
#if defined(CONFIG_BT_CTLR_PRIVACY)
	lrpa = ull_filter_lll_lrpa_get(rl_idx);
	if ((lll->rpa_gen != 0U) && (lrpa != NULL)) {
		pdu_tx->tx_addr = 1U;
		(void)memcpy(&pdu_tx->scan_req.scan_addr[0], lrpa->val, BDADDR_SIZE);
	}
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */
	(void)memcpy(&pdu_tx->scan_req.adv_addr[0], adv_addr, BDADDR_SIZE);
}

static bool is_duration_expired(const struct lll_scan *lll)
{
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	return (lll->duration_reload != 0U) && (lll->duration_expire == 0U);
#else /* !CONFIG_BT_CTLR_ADV_EXT */
	return false;
#endif /* !CONFIG_BT_CTLR_ADV_EXT */
}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
void lll_scan_isr_aux_release(struct lll_scan *lll)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc();
	LL_ASSERT_ERR(node_rx != NULL);

	node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_RELEASE;

	/* The ULL gets the auxiliary context from the scan context if it had
	 * not yet assigned it when the receptions were started.
	 */
	node_rx->rx_ftr.param = lll;
	node_rx->rx_ftr.lll_aux = lll->lll_aux;

	ull_rx_put_sched(node_rx->hdr.link, node_rx);
}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

static void isr_done_cleanup(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll;
	bool is_resume;

	ARG_UNUSED(e);

	/* Under race between duration expire, is_stop is set in this function,
	 * and event preemption, prevent generating duplicate scan done events.
	 */
	if (lll_is_done(param, &is_resume)) {
		return;
	}

	lll = param;
	lll->chan++;
	if (lll->chan == ADV_CHAN_MAX) {
		lll->chan = 0U;
	}

	/* Scanner stop can expire while here in this ISR.
	 * Deferred attempt to stop can fail as it would have
	 * expired, hence ignore failure.
	 */
	(void)ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_LLL, TICKER_ID_SCAN_STOP, NULL,
			  NULL);

#if defined(CONFIG_BT_CTLR_SCAN_INDICATION)
	struct node_rx_pdu *node_rx;

	/* Check if there are enough free node rx available:
	 * 1. For generating this scan indication
	 * 2. Keep one available free for reception of ACL connection Rx data
	 * 3. Keep one available free for reception on ACL connection to NACK
	 *    the PDU
	 */
	node_rx = ull_pdu_rx_alloc_peek(3);
	if (node_rx != NULL) {
		ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_SCAN_INDICATION;

		ull_rx_put_sched(node_rx->hdr.link, node_rx);
	}
#endif /* CONFIG_BT_CTLR_SCAN_INDICATION */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* No scan done event while the event is resumed after a preemption */
	if (!is_resume) {
		struct event_done_extra *extra;

		/* The ULL detects the end of the scan duration on scan done */
		extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN);
		LL_ASSERT_ERR(extra != NULL);
	}

	/* No further scan events once the scan duration has expired */
	if (unlikely(is_duration_expired(lll))) {
		lll->is_stop = 1U;
	}

	/* Auxiliary PDU receptions ended with the event */
	if (lll->is_aux_sched != 0U) {
		lll->is_aux_sched = 0U;

		lll_scan_isr_aux_release(lll);
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	/* Tail chain the LLL disable of any scan event in the pipeline if the
	 * scan role is to be stopped, on connection setup or when the scan
	 * duration has expired.
	 */
	if (lll->is_stop != 0U) {
		static memq_link_t link;
		static struct mayfly mfy = { 0, 0, &link, NULL, lll_disable };
		uint32_t ret;

		mfy.param = param;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
		LL_ASSERT_ERR(ret == 0U);
	}

	lll_isr_cleanup(param);
}

static int isr_rx_scan_report(struct lll_scan *lll, const struct bsr_evt *e,
			      const struct lll_addr_match *match, uint8_t rl_idx, bool dir_report)
{
	struct node_rx_pdu *node_rx;
	int err = 0;

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (node_rx == NULL) {
		return -ENOBUFS;
	}
	ull_pdu_rx_alloc();

	/* The advertising PDU or scan response is in the node rx */
	node_rx->hdr.handle = LLL_HANDLE_INVALID;
	node_rx->hdr.type = NODE_RX_TYPE_REPORT;

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	if (lll->phy != 0U) {
		struct pdu_adv *pdu = (void *)node_rx->pdu;

		/* The radio model has no LE Coded PHY */
		LL_ASSERT_DBG(lll->phy == PHY_1M);
		node_rx->hdr.type = NODE_RX_TYPE_EXT_1M_REPORT;

		if ((pdu->type == PDU_ADV_TYPE_SCAN_RSP) && (lll->is_adv_ind != 0U)) {
			pdu->type = PDU_ADV_TYPE_ADV_IND_SCAN_RSP;
		} else if (pdu->type == PDU_ADV_TYPE_EXT_IND) {
			struct node_rx_ftr *ftr = &node_rx->rx_ftr;

			/* A new auxiliary PDU chain is received */
			lll->lll_aux = NULL;

			ftr->param = lll;
			ftr->ticks_anchor = evt.ticks_ref;
			ftr->radio_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
			ftr->phy_flags = 0U;

			/* Receive the AUX_ADV_IND in this event if it is too
			 * soon for the ULL to schedule it.
			 */
			ftr->aux_lll_sched = lll_scan_aux_setup(lll, NULL, pdu, PHY_1M, e->ts_start,
								evt.ticks_ref);
			if (ftr->aux_lll_sched != 0U) {
				lll->is_aux_sched = 1U;
				err = -EBUSY;
			}
		}
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	node_rx->rx_ftr.rssi = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_PRIVACY)
	node_rx->rx_ftr.rl_idx = (match->irkmatch_ok != 0U) ? rl_idx : FILTER_IDX_NONE;
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	node_rx->rx_ftr.direct_resolved = (rl_idx != FILTER_IDX_NONE);
#endif /* CONFIG_BT_CTLR_ADV_EXT */
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if defined(CONFIG_BT_CTLR_EXT_SCAN_FP)
	node_rx->rx_ftr.direct = dir_report;
#else /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
	ARG_UNUSED(dir_report);
#endif /* !CONFIG_BT_CTLR_EXT_SCAN_FP */

#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC) && defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	/* Not reported if only received for the sync being created */
	node_rx->rx_ftr.devmatch = match->devmatch_ok;
#elif !defined(CONFIG_BT_CTLR_PRIVACY)
	ARG_UNUSED(match);
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC && CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return err;
}

/* The Rx goes on until a PDU is received or the scan window closes */
static void rx(struct lll_scan *lll, uint32_t start)
{
	struct node_rx_pdu *node_rx;

	lll->state = 0U;
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	lll->is_adv_ind = 0U;
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	evt.cfg.chan = 37U + lll->chan;

	lll_radio_rx(&evt.cfg, start, 0U, node_rx->pdu, isr_rx, lll);
}

static void rx_restart(struct lll_scan *lll)
{
	rx(lll, lll_radio_now() + 1U);
}

bool lll_scan_is_stopped(const struct lll_scan *lll)
{
#if defined(CONFIG_BT_CENTRAL)
	if ((lll->conn != NULL) &&
	    ((lll->conn->central.initiated != 0U) || (lll->conn->central.cancelled != 0U))) {
		return true;
	}
#endif /* CONFIG_BT_CENTRAL */

	return (lll->is_stop != 0U);
}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
void lll_scan_isr_resume(struct lll_scan *lll)
{
	/* Close the event if the scan is being stopped, e.g. on connection
	 * setup.
	 */
	if (lll_scan_is_stopped(lll)) {
		isr_done_cleanup(NULL, lll);

		return;
	}

	rx_restart(lll);
}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

/* Only the scan response of the advertiser the SCAN_REQ was sent to */
static bool scan_rsp_check(const struct lll_scan *lll, const struct pdu_adv *pdu)
{
	return (pdu->type == PDU_ADV_TYPE_SCAN_RSP) &&
	       (pdu->len >= offsetof(struct pdu_adv_scan_rsp, data)) &&
	       (pdu->len <= sizeof(struct pdu_adv_scan_rsp)) && (lll->state != 0U) &&
	       (evt.pdu_tx.rx_addr == pdu->tx_addr) &&
	       (memcmp(&evt.pdu_tx.scan_req.adv_addr[0], &pdu->scan_rsp.addr[0],
		       BDADDR_SIZE) == 0);
}

/* The PDU received after a SCAN_REQ, which gives the result of the backoff */
static void isr_rx_scan_rsp(const struct bsr_evt *e, void *param)
{
	const struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_scan_backoff_result((e->status == BSR_STATUS_OK) &&
				scan_rsp_check(param, (const struct pdu_adv *)node_rx->pdu));

	isr_rx(e, param);
}

static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = param;
	struct node_rx_pdu *node_rx;

	if (e->status != BSR_STATUS_OK) {
		rx_restart(lll);

		return;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(PHY_1M), node_rx->pdu, isr_rx_scan_rsp, lll);
}

static bool adv_ind_len_check(const struct pdu_adv *pdu)
{
	return (pdu->len >= offsetof(struct pdu_adv_adv_ind, data)) &&
	       (pdu->len <= sizeof(struct pdu_adv_adv_ind));
}

#if defined(CONFIG_BT_CENTRAL)
static bool init_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu, uint8_t rl_idx)
{
	if (((lll->filter_policy & SCAN_FP_FILTER) == 0U) &&
	    !lll_scan_adva_check(lll, pdu->tx_addr, pdu->adv_ind.addr, rl_idx)) {
		return false;
	}

	if (pdu->type == PDU_ADV_TYPE_ADV_IND) {
		return adv_ind_len_check(pdu);
	}

	return (pdu->type == PDU_ADV_TYPE_DIRECT_IND) &&
	       (pdu->len == sizeof(struct pdu_adv_direct_ind)) &&
	       lll_scan_tgta_check(lll, true, pdu->rx_addr, pdu->direct_ind.tgt_addr, rl_idx, NULL);
}

static int isr_rx_init(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx,
		       const struct lll_addr_match *match, uint8_t rl_idx)
{
	struct node_rx_ftr *ftr;
	struct node_rx_pdu *rx;
	struct pdu_adv *pdu_tx;
	uint32_t conn_space_us;
	struct ull_hdr *ull;
	uint32_t pdu_end_us;
	uint8_t init_tx_addr;
	uint8_t *init_addr;
	uint8_t chan_sel;
#if defined(CONFIG_BT_CTLR_PRIVACY)
	bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

	if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
		rx = ull_pdu_rx_alloc_peek(4);
	} else {
		rx = ull_pdu_rx_alloc_peek(3);
	}

	if (rx == NULL) {
		return -ENOBUFS;
	}

	/* The CONNECT_IND must be sent within the scan event */
	pdu_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
	if (lll->ticks_window == 0U) {
		uint32_t scan_interval_us;

		scan_interval_us = lll->interval * SCAN_INT_UNIT_US;
		pdu_end_us %= scan_interval_us;
	}
	ull = HDR_LLL2ULL(lll);
	if (pdu_end_us > (HAL_TICKER_TICKS_TO_US(ull->ticks_slot) - EVENT_IFS_US -
			  PDU_AC_MAX_US(sizeof(struct pdu_adv_connect_ind), PHY_1M) -
			  EVENT_OVERHEAD_START_US - EVENT_TICKER_RES_MARGIN_US)) {
		return -ETIME;
	}

	init_tx_addr = lll->init_addr_type;
	init_addr = lll->init_addr;
#if defined(CONFIG_BT_CTLR_PRIVACY)
	lrpa = ull_filter_lll_lrpa_get(rl_idx);
	if ((lll->rpa_gen != 0U) && (lrpa != NULL)) {
		init_tx_addr = 1U;
		init_addr = lrpa->val;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	pdu_tx = &evt.pdu_tx;
	lll_scan_prepare_connect_req(lll, pdu_tx, PHY_LEGACY,
				     e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref),
				     lll->conn_win_offset_us, pdu_adv_rx->tx_addr,
				     pdu_adv_rx->adv_ind.addr, init_tx_addr, init_addr,
				     &conn_space_us);

	/* The end of the CONNECT_IND closes the event */
	lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_tx, isr_done_cleanup, lll);

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	lll->conn->rssi_latest = lll_rssi_get(e->rssi);
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

	/* Stop further connection initiation */
	lll->conn->central.initiated = 1U;

	/* Stop further initiating events */
	lll->is_stop = 1U;

	rx = ull_pdu_rx_alloc();

	rx->hdr.type = NODE_RX_TYPE_CONNECTION;
	rx->hdr.handle = LLL_HANDLE_INVALID;

	/* Give the CONNECT_IND sent to the ULL in place of the received PDU,
	 * with the channel selection bit of the received PDU.
	 */
	chan_sel = pdu_adv_rx->chan_sel;
	(void)memcpy(rx->pdu, pdu_tx,
		     offsetof(struct pdu_adv, connect_ind) + sizeof(struct pdu_adv_connect_ind));
	pdu_adv_rx = (void *)rx->pdu;
	pdu_adv_rx->chan_sel = chan_sel;

	ftr = &rx->rx_ftr;
	ftr->param = lll;
	ftr->ticks_anchor = evt.ticks_ref;
	ftr->radio_end_us = conn_space_us;

#if defined(CONFIG_BT_CTLR_PRIVACY)
	ftr->rl_idx = (match->irkmatch_ok != 0U) ? rl_idx : FILTER_IDX_NONE;
	ftr->lrpa_used = (lll->rpa_gen != 0U) && (lrpa != NULL);
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(match);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

	if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
		ftr->extra = ull_pdu_rx_alloc();
	}

	ull_rx_put_sched(rx->hdr.link, rx);

	return 0;
}
#endif /* CONFIG_BT_CENTRAL */

static bool scan_req_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu)
{
	return ((pdu->type == PDU_ADV_TYPE_ADV_IND) || (pdu->type == PDU_ADV_TYPE_SCAN_IND)) &&
	       adv_ind_len_check(pdu) && (lll->type != 0U) && (lll->state == 0U);
}

static int isr_rx_scan_req(struct lll_scan *lll, const struct bsr_evt *e,
			   const struct pdu_adv *pdu_adv_rx, const struct lll_addr_match *match,
			   uint8_t rl_idx)
{
	int err;

	err = isr_rx_scan_report(lll, e, match, rl_idx, false);
	if (err != 0) {
		return err;
	}

	lll_scan_prepare_scan_req(lll, &evt.pdu_tx, pdu_adv_rx->tx_addr,
				  &pdu_adv_rx->adv_ind.addr[0], rl_idx);

	lll->state = 1U;

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* For the event type of the extended report of the scan response */
	if (pdu_adv_rx->type == PDU_ADV_TYPE_ADV_IND) {
		lll->is_adv_ind = 1U;
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, &evt.pdu_tx, isr_tx, lll);

	return 0;
}

static bool report_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu,
			     uint8_t rl_idx, bool *dir_report)
{
	if ((pdu->type == PDU_ADV_TYPE_ADV_IND) || (pdu->type == PDU_ADV_TYPE_NONCONN_IND) ||
	    (pdu->type == PDU_ADV_TYPE_SCAN_IND)) {
		return adv_ind_len_check(pdu);
	}

	if (pdu->type == PDU_ADV_TYPE_DIRECT_IND) {
		return (pdu->len == sizeof(struct pdu_adv_direct_ind)) &&
		       lll_scan_tgta_check(lll, false, pdu->rx_addr, pdu->direct_ind.tgt_addr,
					   rl_idx, dir_report);
	}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	if (pdu->type == PDU_ADV_TYPE_EXT_IND) {
		return (pdu->len != 0U) && (lll->phy != 0U) && (lll->state == 0U) &&
		       lll_scan_ext_tgta_check(lll, true, false, pdu, rl_idx, dir_report);
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	return scan_rsp_check(lll, pdu);
}

static int isr_rx_pdu(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx,
		      const struct lll_addr_match *match, uint8_t rl_idx)
{
	bool dir_report = false;
	int err;

#if defined(CONFIG_BT_CENTRAL)
	/* A connectable ADV_EXT_IND is reported as any other one, for its
	 * AUX_ADV_IND to be received.
	 */
	if ((lll->conn != NULL) && (pdu_adv_rx->type != PDU_ADV_TYPE_EXT_IND)) {
		if ((lll->conn->central.cancelled != 0U) ||
		    !init_pdu_check(lll, pdu_adv_rx, rl_idx)) {
			return -EINVAL;
		}

		return isr_rx_init(lll, e, pdu_adv_rx, match, rl_idx);
	}
#endif /* CONFIG_BT_CENTRAL */

	/* A PDU that the backoff holds the request for is only reported */
	if (scan_req_pdu_check(lll, pdu_adv_rx) && lll_scan_backoff_is_req()) {
		return isr_rx_scan_req(lll, e, pdu_adv_rx, match, rl_idx);
	}

	if (!report_pdu_check(lll, pdu_adv_rx, rl_idx, &dir_report)) {
		return -EINVAL;
	}

	err = isr_rx_scan_report(lll, e, match, rl_idx, dir_report);
	if (err == -EBUSY) {
		/* The auxiliary PDU is being received */
		return 0;
	} else if (err != 0) {
		return err;
	}

	return -ECANCELED;
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = param;
	struct node_rx_pdu *node_rx;
	struct lll_addr_match match;
	struct pdu_adv *pdu;
	bool has_adva;
	uint8_t rl_idx;
	int err;

	/* No PDU, or one with errors */
	if (e->status != BSR_STATUS_OK) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	pdu = (void *)node_rx->pdu;
	if (IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) && (pdu->type == PDU_ADV_TYPE_EXT_IND)) {
		/* An ADV_EXT_IND can have no AdvA */
		has_adva = lll_addr_match_ext(pdu, evt.filter, evt.resolve, &match);
	} else {
		lll_addr_match(pdu, evt.filter, evt.resolve, &match);
		has_adva = true;
	}

	rl_idx = lll_scan_rl_idx_get(lll, &match);

	if (has_adva && !lll_scan_isr_rx_filter(lll, &match, rl_idx)) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	err = isr_rx_pdu(lll, e, pdu, &match, rl_idx);
	if (err == 0) {
		return;
	}

isr_rx_do_close:
	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) && (err == -ECANCELED)) {
		isr_done_cleanup(e, lll);
	} else {
		rx_restart(lll);
	}
}

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	struct event_done_extra *extra;

	/* The ULL detects the end of the scan duration on scan done */
	extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN);
	LL_ASSERT_ERR(extra != NULL);
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	lll_isr_cleanup(param);
}

static int common_prepare_cb(struct lll_prepare_param *p, bool is_resume)
{
	struct lll_scan *lll = p->param;
	uint32_t overhead;
	uint32_t start_us;
	int err;

	DEBUG_RADIO_START_O(1);

	/* Not started if stopped on connection establishment race between
	 * LLL and ULL.
	 */
	if (IS_ENABLED(CONFIG_BT_CENTRAL) && unlikely(lll_scan_is_stopped(lll))) {
		lll_event_abort(lll);

		return 0;
	}

	overhead = lll_preempt_calc(p);
	if (overhead != 0U) {
		LL_ASSERT_OVERHEAD(overhead);

		lll_radio_stop(isr_abort, lll);

		return -ECANCELED;
	}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	lll->is_aux_sched = 0U;
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	/* The primary channel PDUs are on the LE 1M PHY */
	evt.cfg.aa = PDU_AC_ACCESS_ADDR;
	evt.cfg.crc_init = PDU_AC_CRC_IV;
	evt.cfg.phy = BSR_PHY_1M;
	evt.cfg.max_len = PDU_AC_LEG_PAYLOAD_SIZE_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	lll_scan_filter_get(lll, &evt.filter, &evt.resolve);

	start_us = lll_event_start_get(p, &evt.ticks_ref);
	rx(lll, start_us);

	if (!is_resume && (lll->ticks_window != 0U)) {
		uint32_t ret;

		ret = ticker_start(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_LLL, TICKER_ID_SCAN_STOP,
				   p->ticks_at_expire +
				   HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US),
				   lll->ticks_window, TICKER_NULL_PERIOD, TICKER_NULL_REMAINDER,
				   TICKER_NULL_LAZY, TICKER_NULL_SLOT, ticker_stop_cb, lll,
				   ticker_op_start_cb, (void *)__LINE__);
		LL_ASSERT_ERR((ret == TICKER_STATUS_SUCCESS) || (ret == TICKER_STATUS_BUSY));
	}

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_SCHED_ADVANCED)
	/* Get the offset, from this scan window, of the free time space after
	 * the other central connections where the first connection event is to
	 * be placed.
	 */
	if (lll->conn != NULL) {
		static memq_link_t link;
		static struct mayfly mfy = { 0U, 0U, &link, NULL,
					     ull_sched_mfy_after_cen_offset_get };
		struct lll_prepare_param *prepare_param;
		uint32_t ret;

		prepare_param = &lll->prepare_param;
		prepare_param->ticks_at_expire = p->ticks_at_expire;
		prepare_param->remainder = p->remainder;
		prepare_param->param = lll;

		mfy.param = prepare_param;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_LOW, 1U, &mfy);
		LL_ASSERT_ERR(ret == 0U);
	}
#endif /* CONFIG_BT_CENTRAL && CONFIG_BT_CTLR_SCHED_ADVANCED */

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(err == 0);

	DEBUG_RADIO_START_O(1);

	return 0;
}

static int prepare_cb(struct lll_prepare_param *p)
{
	return common_prepare_cb(p, false);
}

static int resume_prepare_cb(struct lll_prepare_param *p)
{
	lll_resume_param_set(p);

	return common_prepare_cb(p, true);
}

static void isr_window(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = param;
	uint32_t ticks_ref_prev;
	uint32_t start;

	ARG_UNUSED(e);

	lll->chan++;
	if (lll->chan == ADV_CHAN_MAX) {
		lll->chan = 0U;
	}

	/* The new window is the reference of the times reported to the ULL */
	ticks_ref_prev = evt.ticks_ref;
	start = lll_radio_now() + 1U;
	evt.ticks_ref = start;

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_SCHED_ADVANCED)
	if ((lll->conn != NULL) && (lll->conn_win_offset_us != 0U)) {
		/* Keep the offset of the free time space for the first
		 * connection event relative to the new reference. Underflow
		 * is accepted, the offset is moved to the future by
		 * connection intervals when establishing the connection.
		 */
		lll->conn_win_offset_us -= HAL_TICKER_TICKS_TO_US(
			ticker_ticks_diff_get(evt.ticks_ref, ticks_ref_prev));
	}
#else /* !CONFIG_BT_CENTRAL || !CONFIG_BT_CTLR_SCHED_ADVANCED */
	ARG_UNUSED(ticks_ref_prev);
#endif /* !CONFIG_BT_CENTRAL || !CONFIG_BT_CTLR_SCHED_ADVANCED */

	rx(lll, start);
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	struct lll_scan *lll = curr;

#if defined(CONFIG_BT_CENTRAL)
	/* Irrespective of same state/role (initiator radio event) or different
	 * state/role (example, advertising radio event) that overlaps the
	 * initiator, if a CONNECT_IND PDU has been enqueued for transmission
	 * then initiator shall not abort.
	 */
	if ((lll->conn != NULL) && (lll->conn->central.initiated != 0U)) {
		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	if (next != curr) {
		/* Not resumed once the scan duration has expired */
		if (unlikely(is_duration_expired(lll))) {
			return -ECANCELED;
		}

		/* Put back to resume state for continuous scanning */
		if (lll->ticks_window == 0U) {
			int err;

			*resume_cb = resume_prepare_cb;

			/* Keep the HF clock on until the resume */
			err = lll_hfclock_on();
			LL_ASSERT_ERR(err >= 0);

			return -EAGAIN;
		}

		return -ECANCELED;
	}

	if (unlikely(is_duration_expired(lll))) {
		lll_radio_stop(isr_done_cleanup, lll);

		return 0;
	}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* Do not abort the scan response or auxiliary PDU reception */
	if ((lll->state != 0U) || (lll->is_aux_sched != 0U)) {
		return 0;
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	/* Switch scan window to next radio channel */
	lll_radio_stop(isr_window, lll);

	return 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
#if defined(CONFIG_BT_CENTRAL)
		struct lll_scan *lll = param;

		/* The end of the CONNECT_IND being sent closes the event */
		if ((lll->conn != NULL) && (lll->conn->central.initiated != 0U)) {
			return;
		}
#endif /* CONFIG_BT_CENTRAL */

		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done_cleanup, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(param);
}

void lll_scan_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}
