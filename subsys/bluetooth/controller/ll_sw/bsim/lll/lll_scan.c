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
#include <zephyr/sys/byteorder.h>
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

#include "hal/debug.h"

#define ADV_CHAN_MAX 3U

#define BACKOFF_UPPER_LIMIT_MAX 256U

static void isr_rx(const struct bsr_evt *e, void *param);

static struct {
	struct bsr_pkt_cfg cfg;
	const struct lll_filter *filter;

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
static bool backoff_is_req(void)
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
static void backoff_result(bool is_rsp)
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

static bool rx_filter_check(const struct lll_scan *lll, uint8_t devmatch_ok)
{
	return ((lll->filter_policy & SCAN_FP_FILTER) == 0U) || (devmatch_ok != 0U);
}

#if defined(CONFIG_BT_CENTRAL)
static bool init_adva_check(const struct lll_scan *lll, uint8_t addr_type, const uint8_t *addr)
{
	return (lll->adv_addr_type == addr_type) && (memcmp(lll->adv_addr, addr, BDADDR_SIZE) == 0);
}

/* conn_space_us is relative to the scan window, as the other times reported to
 * the ULL.
 */
static void prepare_connect_ind(struct lll_scan *lll, const struct bsr_evt *e,
				struct pdu_adv *pdu_tx, uint8_t adv_tx_addr,
				const uint8_t *adv_addr, uint8_t init_tx_addr,
				const uint8_t *init_addr, uint32_t *conn_space_us)
{
	struct lll_conn *lll_conn;
	uint32_t conn_interval_us;
	uint32_t conn_offset_us;

	lll_conn = lll->conn;

	pdu_tx->type = PDU_ADV_TYPE_CONNECT_IND;

	if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
		pdu_tx->chan_sel = 1U;
	} else {
		pdu_tx->chan_sel = 0U;
	}

	pdu_tx->rfu = 0U;
	pdu_tx->tx_addr = init_tx_addr;
	pdu_tx->rx_addr = adv_tx_addr;
	pdu_tx->len = sizeof(struct pdu_adv_connect_ind);
	(void)memcpy(&pdu_tx->connect_ind.init_addr[0], init_addr, BDADDR_SIZE);
	(void)memcpy(&pdu_tx->connect_ind.adv_addr[0], adv_addr, BDADDR_SIZE);
	(void)memcpy(&pdu_tx->connect_ind.access_addr[0], &lll_conn->access_addr[0],
		     sizeof(pdu_tx->connect_ind.access_addr));
	(void)memcpy(&pdu_tx->connect_ind.crc_init[0], &lll_conn->crc_init[0],
		     sizeof(pdu_tx->connect_ind.crc_init));
	pdu_tx->connect_ind.win_size = 1U;

	/* The transmit window starts transmitWindowDelay after the end of the
	 * CONNECT_IND, sent tIFS after the received PDU.
	 */
	conn_interval_us = (uint32_t)lll_conn->interval * CONN_INT_UNIT_US;
	conn_offset_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref) + EVENT_IFS_US +
			 PDU_AC_MAX_US(sizeof(struct pdu_adv_connect_ind), PHY_1M) +
			 WIN_DELAY_LEGACY;

	if (!IS_ENABLED(CONFIG_BT_CTLR_SCHED_ADVANCED) || (lll->conn_win_offset_us == 0U)) {
		*conn_space_us = conn_offset_us;
		pdu_tx->connect_ind.win_offset = sys_cpu_to_le16(0);
	} else {
		uint32_t win_offset_us = lll->conn_win_offset_us;

		/* Place the first connection event after the other central
		 * connections, in the future.
		 */
		while (((win_offset_us & BIT(31)) != 0U) || (win_offset_us < conn_offset_us)) {
			win_offset_us += conn_interval_us;
		}

		*conn_space_us = win_offset_us;
		pdu_tx->connect_ind.win_offset =
			sys_cpu_to_le16((win_offset_us - conn_offset_us) / CONN_INT_UNIT_US);
		pdu_tx->connect_ind.win_size++;
	}

	pdu_tx->connect_ind.interval = sys_cpu_to_le16(lll_conn->interval);
	pdu_tx->connect_ind.latency = sys_cpu_to_le16(lll_conn->latency);
	pdu_tx->connect_ind.timeout = sys_cpu_to_le16(lll->conn_timeout);
	(void)memcpy(&pdu_tx->connect_ind.chan_map[0], &lll_conn->data_chan_map[0],
		     sizeof(pdu_tx->connect_ind.chan_map));
	pdu_tx->connect_ind.hop = lll_conn->data_chan_hop;
	pdu_tx->connect_ind.sca = lll_clock_sca_local_get();
}
#endif /* CONFIG_BT_CENTRAL */

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

	/* Tail chain the LLL disable of any scan event in the pipeline if the
	 * scan role is to be stopped, on connection setup.
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

static inline bool isr_scan_tgta_rpa_check(const struct lll_scan *lll, uint8_t addr_type,
					   const uint8_t *addr, bool *const dir_report)
{
	if (((lll->filter_policy & SCAN_FP_EXT) != 0U) && (addr_type != 0U) &&
	    BT_ADDR_IS_RPA((const bt_addr_t *)addr)) {
		if (dir_report != NULL) {
			*dir_report = true;
		}

		return true;
	}

	return false;
}

static bool isr_scan_tgta_check(const struct lll_scan *lll, uint8_t addr_type, const uint8_t *addr,
				bool *dir_report)
{
	if ((lll->init_addr_type == addr_type) &&
	    (memcmp(lll->init_addr, addr, BDADDR_SIZE) == 0)) {
		return true;
	}

	/* The extended scanner filter policies report directed advertising to
	 * an RPA that is not resolved (Core Spec Vol 6, Part B, Section 4.3.3).
	 */
	return isr_scan_tgta_rpa_check(lll, addr_type, addr, dir_report);
}

static int isr_rx_scan_report(struct lll_scan *lll, const struct bsr_evt *e, bool dir_report)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (node_rx == NULL) {
		return -ENOBUFS;
	}
	ull_pdu_rx_alloc();

	/* The advertising PDU or scan response is in the node rx */
	node_rx->hdr.handle = LLL_HANDLE_INVALID;
	node_rx->hdr.type = NODE_RX_TYPE_REPORT;

	node_rx->rx_ftr.rssi = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_EXT_SCAN_FP)
	node_rx->rx_ftr.direct = dir_report;
#else /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
	ARG_UNUSED(dir_report);
#endif /* !CONFIG_BT_CTLR_EXT_SCAN_FP */

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return 0;
}

/* The Rx goes on until a PDU is received or the scan window closes */
static void rx(struct lll_scan *lll, uint32_t start)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	evt.cfg.chan = 37U + lll->chan;

	lll_radio_rx(&evt.cfg, start, 0U, node_rx->pdu, isr_rx, lll);
}

static void rx_restart(struct lll_scan *lll)
{
	lll->state = 0U;

	rx(lll, lll_radio_now() + 1U);
}

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

	backoff_result((e->status == BSR_STATUS_OK) &&
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
static bool init_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu)
{
	if (((lll->filter_policy & SCAN_FP_FILTER) == 0U) &&
	    !init_adva_check(lll, pdu->tx_addr, pdu->adv_ind.addr)) {
		return false;
	}

	if (pdu->type == PDU_ADV_TYPE_ADV_IND) {
		return adv_ind_len_check(pdu);
	}

	return (pdu->type == PDU_ADV_TYPE_DIRECT_IND) &&
	       (pdu->len == sizeof(struct pdu_adv_direct_ind)) &&
	       isr_scan_tgta_check(lll, pdu->rx_addr, pdu->direct_ind.tgt_addr, NULL);
}

static int isr_rx_init(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx)
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

	pdu_tx = &evt.pdu_tx;
	prepare_connect_ind(lll, e, pdu_tx, pdu_adv_rx->tx_addr, pdu_adv_rx->adv_ind.addr,
			    init_tx_addr, init_addr, &conn_space_us);

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
			   const struct pdu_adv *pdu_adv_rx)
{
	struct pdu_adv *pdu_tx;
	int err;

	err = isr_rx_scan_report(lll, e, false);
	if (err != 0) {
		return err;
	}

	pdu_tx = &evt.pdu_tx;
	pdu_tx->type = PDU_ADV_TYPE_SCAN_REQ;
	pdu_tx->rfu = 0U;
	pdu_tx->chan_sel = 0U;
	pdu_tx->tx_addr = lll->init_addr_type;
	pdu_tx->rx_addr = pdu_adv_rx->tx_addr;
	pdu_tx->len = sizeof(struct pdu_adv_scan_req);
	(void)memcpy(&pdu_tx->scan_req.scan_addr[0], &lll->init_addr[0], BDADDR_SIZE);
	(void)memcpy(&pdu_tx->scan_req.adv_addr[0], &pdu_adv_rx->adv_ind.addr[0], BDADDR_SIZE);

	lll->state = 1U;

	lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_tx, isr_tx, lll);

	return 0;
}

static bool report_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu,
			     bool *dir_report)
{
	if ((pdu->type == PDU_ADV_TYPE_ADV_IND) || (pdu->type == PDU_ADV_TYPE_NONCONN_IND) ||
	    (pdu->type == PDU_ADV_TYPE_SCAN_IND)) {
		return adv_ind_len_check(pdu);
	}

	if (pdu->type == PDU_ADV_TYPE_DIRECT_IND) {
		return (pdu->len == sizeof(struct pdu_adv_direct_ind)) &&
		       isr_scan_tgta_check(lll, pdu->rx_addr, pdu->direct_ind.tgt_addr, dir_report);
	}

	return scan_rsp_check(lll, pdu);
}

static int isr_rx_pdu(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx)
{
	bool dir_report = false;
	int err;

#if defined(CONFIG_BT_CENTRAL)
	if (lll->conn != NULL) {
		if ((lll->conn->central.cancelled != 0U) || !init_pdu_check(lll, pdu_adv_rx)) {
			return -EINVAL;
		}

		return isr_rx_init(lll, e, pdu_adv_rx);
	}
#endif /* CONFIG_BT_CENTRAL */

	/* A PDU that the backoff holds the request for is only reported */
	if (scan_req_pdu_check(lll, pdu_adv_rx) && backoff_is_req()) {
		return isr_rx_scan_req(lll, e, pdu_adv_rx);
	}

	if (!report_pdu_check(lll, pdu_adv_rx, &dir_report)) {
		return -EINVAL;
	}

	err = isr_rx_scan_report(lll, e, dir_report);
	if (err != 0) {
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
	int err;

	/* No PDU, or one with errors */
	if (e->status != BSR_STATUS_OK) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	pdu = (void *)node_rx->pdu;
	lll_addr_match(pdu, evt.filter, &match);

	if (!rx_filter_check(lll, match.devmatch_ok)) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	err = isr_rx_pdu(lll, e, pdu);
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

static int common_prepare_cb(struct lll_prepare_param *p, bool is_resume)
{
	struct lll_scan *lll = p->param;
	uint32_t overhead;
	uint32_t start_us;
	int err;

	DEBUG_RADIO_START_O(1);

#if defined(CONFIG_BT_CENTRAL)
	/* Not started if stopped on connection establishment race between
	 * LLL and ULL.
	 */
	if (unlikely((lll->is_stop != 0U) ||
		     ((lll->conn != NULL) && ((lll->conn->central.initiated != 0U) ||
					      (lll->conn->central.cancelled != 0U))))) {
		lll_event_abort(lll);

		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	overhead = lll_preempt_calc(p);
	if (overhead != 0U) {
		LL_ASSERT_OVERHEAD(overhead);

		lll_event_abort(lll);

		return -ECANCELED;
	}

	lll->state = 0U;

	evt.cfg.aa = PDU_AC_ACCESS_ADDR;
	evt.cfg.crc_init = PDU_AC_CRC_IV;
	evt.cfg.phy = BSR_PHY_1M;
	evt.cfg.max_len = PDU_AC_LEG_PAYLOAD_SIZE_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	evt.filter = NULL;
	if (IS_ENABLED(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && (lll->filter_policy != 0U)) {
		evt.filter = ull_filter_lll_get(true);
	}

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

	lll->state = 0U;
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
