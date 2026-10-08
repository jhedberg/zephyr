/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Scanning and initiating events of the BabbleSim LLL.
 *
 * A scan event listens on one primary advertising channel from its start
 * until the end of the scan window, receiving one PDU after the other. An
 * active scanner sends a SCAN_REQ tIFS after a scannable PDU and listens for
 * the SCAN_RSP tIFS after it, and an initiator sends a CONNECT_IND tIFS after
 * a connectable PDU of the peer it connects to. Continuous scanning moves to
 * the next channel at each scan interval, without ending the event.
 *
 * With extended scanning, the auxiliary PDU of an ADV_EXT_IND that is too
 * soon for the ULL to schedule is received in the scan event, which goes
 * back to the primary channel at the end of the auxiliary PDUs (see
 * lll_scan_aux.c).
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
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

/* Maximum primary Advertising Radio Channels to scan */
#define ADV_CHAN_MAX 3U

static int prepare_cb(struct lll_prepare_param *p);
static int resume_prepare_cb(struct lll_prepare_param *p);
static int common_prepare_cb(struct lll_prepare_param *p, bool is_resume);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void ticker_stop_cb(uint32_t ticks_at_expire, uint32_t ticks_drift, uint32_t remainder,
			   uint16_t lazy, uint8_t force, void *param);
static void ticker_op_start_cb(uint32_t status, void *param);
static void isr_rx(const struct bsr_evt *e, void *param);
static void isr_tx(const struct bsr_evt *e, void *param);
static void isr_window(const struct bsr_evt *e, void *param);
static void isr_done_cleanup(const struct bsr_evt *e, void *param);
static void isr_abort(const struct bsr_evt *e, void *param);
static void rx(struct lll_scan *lll, uint32_t start);
static void rx_restart(struct lll_scan *lll);
static int isr_rx_pdu(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx,
		      const struct lll_addr_match *match, uint8_t rl_idx);
static bool isr_scan_tgta_check(const struct lll_scan *lll, bool init, uint8_t addr_type,
				const uint8_t *addr, uint8_t rl_idx, bool *dir_report);
static int isr_rx_scan_report(struct lll_scan *lll, const struct bsr_evt *e,
			      const struct lll_addr_match *match, uint8_t rl_idx, bool dir_report);

/* State of the current scan event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* Filter and resolution of the AdvA of received PDUs */
	const struct lll_filter *filter;
	bool resolve;

	/* Reference of the times reported to the ULL: the start of the
	 * current scan window
	 */
	uint32_t ticks_ref;

	/* The SCAN_REQ or CONNECT_IND sent */
	struct pdu_adv pdu_tx;
} evt;

int lll_scan_init(void)
{
	return 0;
}

int lll_scan_reset(void)
{
	return 0;
}

void lll_scan_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

void lll_scan_filter_get(const struct lll_scan *lll, const struct lll_filter **filter,
			 bool *resolve)
{
	*filter = NULL;
	*resolve = false;

	if (0) {
#if defined(CONFIG_BT_CTLR_PRIVACY)
	} else if (ull_filter_lll_rl_enabled()) {
		*filter = ull_filter_lll_get((lll->filter_policy & SCAN_FP_FILTER) != 0U);
		*resolve = true;
#endif /* CONFIG_BT_CTLR_PRIVACY */
	} else if (IS_ENABLED(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && lll->filter_policy) {
		*filter = ull_filter_lll_get(true);
	}
}

uint8_t lll_scan_rl_idx_get(const struct lll_scan *lll, const struct lll_addr_match *match)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	return match->devmatch_ok ?
	       ull_filter_lll_rl_idx(((lll->filter_policy & SCAN_FP_FILTER) != 0U),
				     match->devmatch_id) :
	       match->irkmatch_ok ? ull_filter_lll_rl_irk_idx(match->irkmatch_id) :
				    FILTER_IDX_NONE;
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(lll);
	ARG_UNUSED(match);

	return FILTER_IDX_NONE;
#endif /* !CONFIG_BT_CTLR_PRIVACY */
}

static bool isr_rx_check(const struct lll_scan *lll, uint8_t irkmatch_ok, uint8_t devmatch_ok,
			 uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	return (((lll->filter_policy & SCAN_FP_FILTER) == 0U) &&
		(!devmatch_ok || ull_filter_lll_rl_idx_allowed(irkmatch_ok, rl_idx))) ||
	       (((lll->filter_policy & SCAN_FP_FILTER) != 0U) &&
		(devmatch_ok || ull_filter_lll_irk_in_fal(rl_idx)));
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(irkmatch_ok);
	ARG_UNUSED(rl_idx);

	return ((lll->filter_policy & SCAN_FP_FILTER) == 0U) || devmatch_ok;
#endif /* !CONFIG_BT_CTLR_PRIVACY */
}

bool lll_scan_isr_rx_filter(const struct lll_scan *lll, struct lll_addr_match *match,
			    uint8_t rl_idx)
{
	bool allow;

	allow = isr_rx_check(lll, match->irkmatch_ok, match->devmatch_ok, rl_idx);

#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC) && defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	/* Reported only if the filter policy allows it */
	match->devmatch_ok = allow;

	/* Received anyway while a sync is being created, for the ULL to look
	 * for the periodic advertiser in it.
	 */
	return allow || lll->is_sync;
#else /* !CONFIG_BT_CTLR_SYNC_PERIODIC || !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
	return allow;
#endif /* !CONFIG_BT_CTLR_SYNC_PERIODIC || !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
}

#if defined(CONFIG_BT_CENTRAL) || defined(CONFIG_BT_CTLR_ADV_EXT)
bool lll_scan_adva_check(const struct lll_scan *lll, uint8_t addr_type, const uint8_t *addr,
			 uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* Only applies to initiator with no filter accept list */
	if (rl_idx != FILTER_IDX_NONE) {
		return (rl_idx == lll->rl_idx);
	} else if (!ull_filter_lll_rl_addr_allowed(addr_type, addr, &rl_idx)) {
		return false;
	}
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if defined(CONFIG_BT_CENTRAL)
	return (lll->adv_addr_type == addr_type) && !memcmp(lll->adv_addr, addr, BDADDR_SIZE);
#else /* !CONFIG_BT_CENTRAL */
	/* Only used when initiating */
	ARG_UNUSED(lll);
	ARG_UNUSED(addr_type);
	ARG_UNUSED(addr);

	return false;
#endif /* !CONFIG_BT_CENTRAL */
}
#endif /* CONFIG_BT_CENTRAL || CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
bool lll_scan_ext_tgta_check(const struct lll_scan *lll, bool pri, bool is_init,
			     const struct pdu_adv *pdu, uint8_t rl_idx, bool *dir_report)
{
	const uint8_t *adva;
	const uint8_t *tgta;
	uint8_t is_directed;

	/* Not a directed report unless the TargetA check below says so */
	if (dir_report) {
		*dir_report = false;
	}

	if (pri && !pdu->adv_ext_ind.ext_hdr.adv_addr) {
		return true;
	}

	if (pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN + sizeof(struct pdu_adv_ext_hdr) + ADVA_SIZE)) {
		return false;
	}

	is_directed = pdu->adv_ext_ind.ext_hdr.tgt_addr;
	if (is_directed && (pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN +
					sizeof(struct pdu_adv_ext_hdr) + ADVA_SIZE +
					TARGETA_SIZE))) {
		return false;
	}

	adva = &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET];
	tgta = &pdu->adv_ext_ind.ext_hdr.data[TGTA_OFFSET];

	return (!is_init || ((lll->filter_policy & SCAN_FP_FILTER) != 0U) ||
		lll_scan_adva_check(lll, pdu->tx_addr, adva, rl_idx)) &&
	       (!is_directed ||
		isr_scan_tgta_check(lll, is_init, pdu->rx_addr, tgta, rl_idx, dir_report));
}

void lll_scan_isr_resume(struct lll_scan *lll)
{
	/* Close the event if the scan is being stopped, e.g. on connection
	 * setup.
	 */
	if (lll->is_stop
#if defined(CONFIG_BT_CENTRAL)
	    || (lll->conn && (lll->conn->central.initiated || lll->conn->central.cancelled))
#endif /* CONFIG_BT_CENTRAL */
	   ) {
		isr_done_cleanup(NULL, lll);

		return;
	}

	rx_restart(lll);
}

void lll_scan_isr_aux_release(struct lll_scan *lll)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc();
	LL_ASSERT_ERR(node_rx);

	node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_RELEASE;

	/* The ULL gets the auxiliary context from the scan context if it had
	 * not yet assigned it when the receptions were started.
	 */
	node_rx->rx_ftr.param = lll;
	node_rx->rx_ftr.lll_aux = lll->lll_aux;

	ull_rx_put_sched(node_rx->hdr.link, node_rx);
}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CENTRAL)
void lll_scan_prepare_connect_req(struct lll_scan *lll, struct pdu_adv *pdu_tx, uint8_t phy,
				  uint32_t pdu_end_us, uint8_t adv_tx_addr, const uint8_t *adv_addr,
				  uint8_t init_tx_addr, const uint8_t *init_addr,
				  uint32_t *conn_space_us)
{
	struct lll_conn *lll_conn;
	uint32_t conn_interval_us;
	uint32_t conn_offset_us;
	uint8_t phy_pdu;

	lll_conn = lll->conn;

	/* AUX_CONNECT_REQ is the same as CONNECT_IND */
	pdu_tx->type = PDU_ADV_TYPE_CONNECT_IND;

	if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
		pdu_tx->chan_sel = 1;
	} else {
		pdu_tx->chan_sel = 0;
	}

	pdu_tx->rfu = 0U;
	pdu_tx->tx_addr = init_tx_addr;
	pdu_tx->rx_addr = adv_tx_addr;
	pdu_tx->len = sizeof(struct pdu_adv_connect_ind);
	(void)memcpy(&pdu_tx->connect_ind.init_addr[0], init_addr, BDADDR_SIZE);
	(void)memcpy(&pdu_tx->connect_ind.adv_addr[0], adv_addr, BDADDR_SIZE);
	(void)memcpy(&pdu_tx->connect_ind.access_addr[0], &lll_conn->access_addr[0], 4);
	(void)memcpy(&pdu_tx->connect_ind.crc_init[0], &lll_conn->crc_init[0], 3);
	pdu_tx->connect_ind.win_size = 1;

	/* The transmit window starts transmitWindowDelay after the end of the
	 * request sent tIFS after the received PDU: 1.25 ms after a legacy PDU,
	 * 2.5 ms after one on an LE Uncoded PHY.
	 */
	phy_pdu = (phy != PHY_LEGACY) ? phy : PHY_1M;
	conn_interval_us = (uint32_t)lll_conn->interval * CONN_INT_UNIT_US;
	conn_offset_us = pdu_end_us + EVENT_IFS_US +
			 PDU_AC_MAX_US(sizeof(struct pdu_adv_connect_ind), phy_pdu) +
			 ((phy != PHY_LEGACY) ? WIN_DELAY_UNCODED : WIN_DELAY_LEGACY);

	if (!IS_ENABLED(CONFIG_BT_CTLR_SCHED_ADVANCED) || (lll->conn_win_offset_us == 0U)) {
		*conn_space_us = conn_offset_us;
		pdu_tx->connect_ind.win_offset = sys_cpu_to_le16(0);
	} else {
		uint32_t win_offset_us = lll->conn_win_offset_us;

		/* Place the first connection event after the other central
		 * connections, in the future.
		 */
		while ((win_offset_us & BIT(31)) || (win_offset_us < conn_offset_us)) {
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

static int prepare_cb(struct lll_prepare_param *p)
{
	return common_prepare_cb(p, false);
}

static int resume_prepare_cb(struct lll_prepare_param *p)
{
	lll_resume_param_set(p);

	return common_prepare_cb(p, true);
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
	if (unlikely(lll->is_stop || (lll->conn && (lll->conn->central.initiated ||
						    lll->conn->central.cancelled)))) {
		lll_event_abort(lll);

		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	overhead = lll_preempt_calc(p);
	if (overhead) {
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

	if (!is_resume && lll->ticks_window) {
		uint32_t ret;

		/* Start the window close timeout */
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
	if (lll->conn) {
		static memq_link_t link;
		static struct mayfly mfy_after_cen_offset_get = {
			0U, 0U, &link, NULL, ull_sched_mfy_after_cen_offset_get};
		struct lll_prepare_param *prepare_param;
		uint32_t ret;

		/* Copy the required values to calculate the offsets */
		prepare_param = &lll->prepare_param;
		prepare_param->ticks_at_expire = p->ticks_at_expire;
		prepare_param->remainder = p->remainder;
		prepare_param->param = lll;

		mfy_after_cen_offset_get.param = prepare_param;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_LOW, 1U,
				     &mfy_after_cen_offset_get);
		LL_ASSERT_ERR(!ret);
	}
#endif /* CONFIG_BT_CENTRAL && CONFIG_BT_CTLR_SCHED_ADVANCED */

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_O(1);

	return 0;
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
	if (lll->conn && lll->conn->central.initiated) {
		/* Connection Establishment initiated, do not abort */
		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	/* Check if pre-emption by a different state/role radio event */
	if (next != curr) {
#if defined(CONFIG_BT_CTLR_ADV_EXT)
		/* Not resumed once the scan duration has expired */
		if (unlikely(lll->duration_reload && !lll->duration_expire)) {
			return -ECANCELED;
		}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

		/* Put back to resume state for continuous scanning */
		if (!lll->ticks_window) {
			int err;

			/* Set the resume prepare function to use for
			 * resumption after the pre-emptor is done.
			 */
			*resume_cb = resume_prepare_cb;

			/* Retain HF clock */
			err = lll_hfclock_on();
			LL_ASSERT_ERR(err >= 0);

			/* Yield to the pre-emptor, but be resumed thereafter */
			return -EAGAIN;
		}

		/* Yield to the pre-emptor */
		return -ECANCELED;
	}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	if (unlikely(lll->duration_reload && !lll->duration_expire)) {
		/* The scan duration has expired, close the event */
		lll_radio_stop(isr_done_cleanup, lll);

		return 0;
	} else if (lll->state || lll->is_aux_sched) {
		/* Do not abort the scan response or auxiliary PDU reception */
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

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
#if defined(CONFIG_BT_CENTRAL)
		struct lll_scan *lll = param;

		/* The end of the CONNECT_IND being sent closes the event */
		if (lll->conn && lll->conn->central.initiated) {
			return;
		}
#endif /* CONFIG_BT_CENTRAL */

		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done_cleanup, param);

		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(param);
}

static void ticker_stop_cb(uint32_t ticks_at_expire, uint32_t ticks_drift, uint32_t remainder,
			   uint16_t lazy, uint8_t force, void *param)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};
	uint32_t ret;

	mfy.param = param;
	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL, 0, &mfy);
	LL_ASSERT_ERR(!ret);
}

static void ticker_op_start_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT_ERR(status == TICKER_STATUS_SUCCESS);
}

/* Listen on the channel of the scan window from time start, until a PDU has
 * been received.
 */
static void rx(struct lll_scan *lll, uint32_t start)
{
	struct node_rx_pdu *node_rx;

	/* Initialize scanning state */
	lll->state = 0U;
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	lll->is_adv_ind = 0U;
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	evt.cfg.chan = 37 + lll->chan;

	lll_radio_rx(&evt.cfg, start, 0U, node_rx->pdu, isr_rx, lll);
}

/* Continue scanning after a received PDU, or a scan request and response */
static void rx_restart(struct lll_scan *lll)
{
	rx(lll, lll_radio_now() + 1U);
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
	LL_ASSERT_DBG(node_rx);

	pdu = (void *)node_rx->pdu;
	if (0) {
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	} else if (pdu->type == PDU_ADV_TYPE_EXT_IND) {
		/* An ADV_EXT_IND can have no AdvA */
		has_adva = lll_addr_match_ext(pdu, evt.filter, evt.resolve, &match);
#endif /* CONFIG_BT_CTLR_ADV_EXT */
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
	if (!err) {
		return;
	}

isr_rx_do_close:
	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) && (err == -ECANCELED)) {
		isr_done_cleanup(e, lll);
	} else {
		rx_restart(lll);
	}
}

/* End of a SCAN_REQ */
static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = param;
	struct node_rx_pdu *node_rx;

	if (e->status != BSR_STATUS_OK) {
		rx_restart(lll);

		return;
	}

	/* Listen for the SCAN_RSP */
	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(PHY_1M), node_rx->pdu, isr_rx, lll);
}

/* Next scan window of a continuous scan, on the next channel */
static void isr_window(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = param;
	uint32_t ticks_ref_prev;
	uint32_t start;

	ARG_UNUSED(e);

	/* Next radio channel to scan, round-robin 37, 38, and 39 */
	if (++lll->chan == ADV_CHAN_MAX) {
		lll->chan = 0U;
	}

	/* The new window is the reference of the times reported to the ULL */
	ticks_ref_prev = evt.ticks_ref;
	start = lll_radio_now() + 1U;
	evt.ticks_ref = start;

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_SCHED_ADVANCED)
	if (lll->conn && lll->conn_win_offset_us) {
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

	/* Next window to use next advertising radio channel */
	lll = param;
	if (++lll->chan == ADV_CHAN_MAX) {
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
	if (node_rx) {
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
		LL_ASSERT_ERR(extra);
	}

	/* No further scan events once the scan duration has expired */
	if (unlikely(lll->duration_reload && !lll->duration_expire)) {
		lll->is_stop = 1U;
	}

	/* Auxiliary PDU receptions ended with the event */
	if (lll->is_aux_sched) {
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
		static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};
		uint32_t ret;

		mfy.param = param;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
		LL_ASSERT_ERR(!ret);
	}

	lll_isr_cleanup(param);
}

/* End of an event that could not start on time */
static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	struct event_done_extra *extra;

	/* The ULL detects the end of the scan duration on scan done */
	extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN);
	LL_ASSERT_ERR(extra);
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	lll_isr_cleanup(param);
}

static int isr_rx_pdu(struct lll_scan *lll, const struct bsr_evt *e, struct pdu_adv *pdu_adv_rx,
		      const struct lll_addr_match *match, uint8_t rl_idx)
{
	bool dir_report = false;

	if (0) {
#if defined(CONFIG_BT_CENTRAL)
	/* Initiator */
	/* NOTE: A connectable ADV_EXT_IND is reported as any other one, for
	 *       its AUX_ADV_IND to be received.
	 */
	} else if (lll->conn && !lll->conn->central.cancelled &&
		   (pdu_adv_rx->type != PDU_ADV_TYPE_EXT_IND) &&
		   ((((lll->filter_policy & SCAN_FP_FILTER) != 0U) ||
		     lll_scan_adva_check(lll, pdu_adv_rx->tx_addr, pdu_adv_rx->adv_ind.addr,
					 rl_idx)) &&
		    (((pdu_adv_rx->type == PDU_ADV_TYPE_ADV_IND) &&
		      (pdu_adv_rx->len >= offsetof(struct pdu_adv_adv_ind, data)) &&
		      (pdu_adv_rx->len <= sizeof(struct pdu_adv_adv_ind))) ||
		     ((pdu_adv_rx->type == PDU_ADV_TYPE_DIRECT_IND) &&
		      (pdu_adv_rx->len == sizeof(struct pdu_adv_direct_ind)) &&
		      /* allow directed adv packets addressed to this device */
		      isr_scan_tgta_check(lll, true, pdu_adv_rx->rx_addr,
					  pdu_adv_rx->direct_ind.tgt_addr, rl_idx, NULL))))) {
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

		if (!rx) {
			return -ENOBUFS;
		}

		/* The CONNECT_IND must be sent within the scan event */
		pdu_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
		if (!lll->ticks_window) {
			uint32_t scan_interval_us;

			scan_interval_us = lll->interval * SCAN_INT_UNIT_US;
			pdu_end_us %= scan_interval_us;
		}
		ull = HDR_LLL2ULL(lll);
		if (pdu_end_us > (HAL_TICKER_TICKS_TO_US(ull->ticks_slot) - EVENT_IFS_US - 352 -
				  EVENT_OVERHEAD_START_US - EVENT_TICKER_RES_MARGIN_US)) {
			return -ETIME;
		}

#if defined(CONFIG_BT_CTLR_PRIVACY)
		lrpa = ull_filter_lll_lrpa_get(rl_idx);
		if (lll->rpa_gen && lrpa) {
			init_tx_addr = 1;
			init_addr = lrpa->val;
		} else {
#else /* !CONFIG_BT_CTLR_PRIVACY */
		if (1) {
#endif /* !CONFIG_BT_CTLR_PRIVACY */
			init_tx_addr = lll->init_addr_type;
			init_addr = lll->init_addr;
		}

		pdu_tx = &evt.pdu_tx;
		lll_scan_prepare_connect_req(lll, pdu_tx, PHY_LEGACY,
					     e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref),
					     pdu_adv_rx->tx_addr, pdu_adv_rx->adv_ind.addr,
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
		rx->hdr.handle = 0xffff;

		/* Give the CONNECT_IND sent to the ULL in place of the received
		 * PDU, with the channel selection bit of the received PDU.
		 */
		chan_sel = pdu_adv_rx->chan_sel;
		(void)memcpy(rx->pdu, pdu_tx,
			     (offsetof(struct pdu_adv, connect_ind) +
			      sizeof(struct pdu_adv_connect_ind)));
		pdu_adv_rx = (void *)rx->pdu;
		pdu_adv_rx->chan_sel = chan_sel;

		ftr = &(rx->rx_ftr);
		ftr->param = lll;
		ftr->ticks_anchor = evt.ticks_ref;
		ftr->radio_end_us = conn_space_us;

#if defined(CONFIG_BT_CTLR_PRIVACY)
		ftr->rl_idx = match->irkmatch_ok ? rl_idx : FILTER_IDX_NONE;
		ftr->lrpa_used = lll->rpa_gen && lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			ftr->extra = ull_pdu_rx_alloc();
		}

		ull_rx_put_sched(rx->hdr.link, rx);

		return 0;
#endif /* CONFIG_BT_CENTRAL */

	/* Active scanner */
	} else if (((pdu_adv_rx->type == PDU_ADV_TYPE_ADV_IND) ||
		    (pdu_adv_rx->type == PDU_ADV_TYPE_SCAN_IND)) &&
		   (pdu_adv_rx->len >= offsetof(struct pdu_adv_adv_ind, data)) &&
		   (pdu_adv_rx->len <= sizeof(struct pdu_adv_adv_ind)) &&
		   lll->type && !lll->state &&
#if defined(CONFIG_BT_CENTRAL)
		   !lll->conn) {
#else /* !CONFIG_BT_CENTRAL */
		   1) {
#endif /* !CONFIG_BT_CENTRAL */
		struct pdu_adv *pdu_tx;
#if defined(CONFIG_BT_CTLR_PRIVACY)
		bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */
		int err;

		/* save the adv packet */
		err = isr_rx_scan_report(lll, e, match, rl_idx, false);
		if (err) {
			return err;
		}

		/* prepare the scan request packet */
		pdu_tx = &evt.pdu_tx;
		pdu_tx->type = PDU_ADV_TYPE_SCAN_REQ;
		pdu_tx->rfu = 0U;
		pdu_tx->chan_sel = 0U;
		pdu_tx->rx_addr = pdu_adv_rx->tx_addr;
		pdu_tx->len = sizeof(struct pdu_adv_scan_req);
#if defined(CONFIG_BT_CTLR_PRIVACY)
		lrpa = ull_filter_lll_lrpa_get(rl_idx);
		if (lll->rpa_gen && lrpa) {
			pdu_tx->tx_addr = 1;
			(void)memcpy(&pdu_tx->scan_req.scan_addr[0], lrpa->val, BDADDR_SIZE);
		} else {
#else /* !CONFIG_BT_CTLR_PRIVACY */
		if (1) {
#endif /* !CONFIG_BT_CTLR_PRIVACY */
			pdu_tx->tx_addr = lll->init_addr_type;
			(void)memcpy(&pdu_tx->scan_req.scan_addr[0], &lll->init_addr[0],
				     BDADDR_SIZE);
		}
		(void)memcpy(&pdu_tx->scan_req.adv_addr[0], &pdu_adv_rx->adv_ind.addr[0],
			     BDADDR_SIZE);

		/* switch scanner state to active */
		lll->state = 1U;

#if defined(CONFIG_BT_CTLR_ADV_EXT)
		if (pdu_adv_rx->type == PDU_ADV_TYPE_ADV_IND) {
			lll->is_adv_ind = 1U;
		}
#endif /* CONFIG_BT_CTLR_ADV_EXT */

		lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_tx, isr_tx, lll);

		return 0;
	}
	/* Passive scanner or scan responses */
	else if (((((pdu_adv_rx->type == PDU_ADV_TYPE_ADV_IND) ||
		     (pdu_adv_rx->type == PDU_ADV_TYPE_NONCONN_IND) ||
		     (pdu_adv_rx->type == PDU_ADV_TYPE_SCAN_IND)) &&
		    (pdu_adv_rx->len >= offsetof(struct pdu_adv_adv_ind, data)) &&
		    (pdu_adv_rx->len <= sizeof(struct pdu_adv_adv_ind))) ||
		   ((pdu_adv_rx->type == PDU_ADV_TYPE_DIRECT_IND) &&
		    (pdu_adv_rx->len == sizeof(struct pdu_adv_direct_ind)) &&
		    /* allow directed adv packets addressed to this device */
		    isr_scan_tgta_check(lll, false, pdu_adv_rx->rx_addr,
					pdu_adv_rx->direct_ind.tgt_addr, rl_idx, &dir_report)) ||
#if defined(CONFIG_BT_CTLR_ADV_EXT)
		   ((pdu_adv_rx->type == PDU_ADV_TYPE_EXT_IND) && lll->phy && !lll->state &&
		    lll_scan_ext_tgta_check(lll, true, false, pdu_adv_rx, rl_idx, &dir_report)) ||
#endif /* CONFIG_BT_CTLR_ADV_EXT */
		   ((pdu_adv_rx->type == PDU_ADV_TYPE_SCAN_RSP) &&
		    (pdu_adv_rx->len >= offsetof(struct pdu_adv_scan_rsp, data)) &&
		    (pdu_adv_rx->len <= sizeof(struct pdu_adv_scan_rsp)) &&
		    (lll->state != 0U) &&
		    /* the scan response of the advertiser the SCAN_REQ was sent to */
		    (evt.pdu_tx.rx_addr == pdu_adv_rx->tx_addr) &&
		    !memcmp(&evt.pdu_tx.scan_req.adv_addr[0], &pdu_adv_rx->scan_rsp.addr[0],
			    BDADDR_SIZE))) &&
		  (pdu_adv_rx->len != 0) &&
#if defined(CONFIG_BT_CENTRAL)
		  /* An ADV_EXT_IND is also received when initiating, for its
		   * AUX_ADV_IND.
		   */
		  (!lll->conn || (pdu_adv_rx->type == PDU_ADV_TYPE_EXT_IND))) {
#else /* !CONFIG_BT_CENTRAL */
		  1) {
#endif /* !CONFIG_BT_CENTRAL */
		int err;

		/* save the scan response packet */
		err = isr_rx_scan_report(lll, e, match, rl_idx, dir_report);
		if (err) {
			/* The auxiliary PDU is being received */
			if (IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) && (err == -EBUSY)) {
				return 0;
			}

			return err;
		}
	}
	/* invalid PDU */
	else {
		/* ignore and close this rx/tx chain */
		return -EINVAL;
	}

	return -ECANCELED;
}

static inline bool isr_scan_tgta_rpa_check(const struct lll_scan *lll, uint8_t addr_type,
					   const uint8_t *addr, bool *const dir_report)
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

static bool isr_scan_tgta_check(const struct lll_scan *lll, bool init, uint8_t addr_type,
				const uint8_t *addr, uint8_t rl_idx, bool *dir_report)
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

static int isr_rx_scan_report(struct lll_scan *lll, const struct bsr_evt *e,
			      const struct lll_addr_match *match, uint8_t rl_idx, bool dir_report)
{
	struct node_rx_pdu *node_rx;
	int err = 0;

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (!node_rx) {
		return -ENOBUFS;
	}
	ull_pdu_rx_alloc();

	/* Prepare the report (adv or scan resp), the PDU is in the node rx */
	node_rx->hdr.handle = 0xffff;

	if (0) {
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	} else if (lll->phy) {
		struct pdu_adv *pdu = (void *)node_rx->pdu;

		/* Extended scanning, on the LE 1M PHY */
		LL_ASSERT_DBG(lll->phy == PHY_1M);
		node_rx->hdr.type = NODE_RX_TYPE_EXT_1M_REPORT;

		if ((pdu->type == PDU_ADV_TYPE_SCAN_RSP) && lll->is_adv_ind) {
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
			if (ftr->aux_lll_sched) {
				lll->is_aux_sched = 1U;
				err = -EBUSY;
			}
		}
#endif /* CONFIG_BT_CTLR_ADV_EXT */
	} else {
		node_rx->hdr.type = NODE_RX_TYPE_REPORT;
	}

	node_rx->rx_ftr.rssi = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* save the resolving list index. */
	node_rx->rx_ftr.rl_idx = match->irkmatch_ok ? rl_idx : FILTER_IDX_NONE;

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	node_rx->rx_ftr.direct_resolved = (rl_idx != FILTER_IDX_NONE);
#endif /* CONFIG_BT_CTLR_ADV_EXT */
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if defined(CONFIG_BT_CTLR_EXT_SCAN_FP)
	/* save the directed adv report flag */
	node_rx->rx_ftr.direct = dir_report;
#else /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
	ARG_UNUSED(dir_report);
#endif /* !CONFIG_BT_CTLR_EXT_SCAN_FP */

#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC) && defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	/* Reported if allowed by the filter policy */
	node_rx->rx_ftr.devmatch = match->devmatch_ok;
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC && CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */

#if !defined(CONFIG_BT_CTLR_PRIVACY) && \
	!(defined(CONFIG_BT_CTLR_SYNC_PERIODIC) && defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST))
	ARG_UNUSED(match);
#endif /* !CONFIG_BT_CTLR_PRIVACY && !(CONFIG_BT_CTLR_SYNC_PERIODIC && FILTER_ACCEPT_LIST) */

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return err;
}
