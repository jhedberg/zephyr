/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Legacy advertising events of the BabbleSim LLL.
 *
 * An advertising event transmits the advertising PDU on each channel of the
 * channel map, starting at the event start. After a connectable or scannable
 * PDU, the advertiser listens for a SCAN_REQ or CONNECT_IND tIFS later, and
 * answers a SCAN_REQ with the scan response tIFS after it. The next channel
 * is used as soon as the radio can switch to it. A high duty cycle directed
 * advertising event goes on cycling through the channels until it is aborted.
 *
 * With extended advertising, the ADV_EXT_IND sent on each channel has the
 * offset to the AUX_ADV_IND of the auxiliary advertising event that follows
 * (see lll_adv_aux.c), which then generates the done event of both.
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/dbuf.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_clock.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_aux.h"
#include "lll_adv_sync.h"
#include "lll_df_types.h"
#include "lll_conn.h"
#include "lll_filter.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_addr.h"
#include "lll_adv_internal.h"

#include "hal/debug.h"

/* Time from the end of the last radio operation on a channel to the start of
 * the advertising PDU on the next one, as the radio ramp up of a fast
 * ramping radio.
 */
#define ADV_CHAN_SWITCH_US 40U

static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_tx(const struct bsr_evt *e, void *param);
static void isr_rx(const struct bsr_evt *e, void *param);
static void isr_done(const struct bsr_evt *e, void *param);
static void isr_abort(const struct bsr_evt *e, void *param);
static struct pdu_adv *chan_tx(struct lll_adv *lll, uint32_t at);
static int isr_rx_pdu(struct lll_adv *lll, const struct bsr_evt *e, struct pdu_adv *pdu_rx,
		      const struct lll_addr_match *match);

/* State of the current advertising event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* Filter and resolution of the address of a received SCAN_REQ or
	 * CONNECT_IND
	 */
	const struct lll_filter *filter;
	bool resolve;

	/* Reference of the event times reported to the ULL */
	uint32_t ticks_ref;
} evt;

int lll_adv_init(void)
{
	int err;

#if defined(CONFIG_BT_CTLR_ADV_EXT) && (CONFIG_BT_CTLR_ADV_AUX_SET > 0)
	err = lll_adv_aux_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT && (CONFIG_BT_CTLR_ADV_AUX_SET > 0) */

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
	err = lll_adv_sync_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */

	err = lll_adv_pdu_init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_adv_reset(void)
{
	int err;

#if defined(CONFIG_BT_CTLR_ADV_EXT) && (CONFIG_BT_CTLR_ADV_AUX_SET > 0)
	err = lll_adv_aux_reset();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT && (CONFIG_BT_CTLR_ADV_AUX_SET > 0) */

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
	err = lll_adv_sync_reset();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */

	err = lll_adv_pdu_init_reset();
	if (err) {
		return err;
	}

	return 0;
}

void lll_adv_filter_get(const struct lll_adv *lll, const struct lll_filter **filter,
			bool *resolve)
{
	*filter = NULL;
	*resolve = false;

	if (0) {
#if defined(CONFIG_BT_CTLR_PRIVACY)
	} else if (ull_filter_lll_rl_enabled()) {
		*filter = ull_filter_lll_get(!!(lll->filter_policy));
		*resolve = true;
#endif /* CONFIG_BT_CTLR_PRIVACY */
	} else if (IS_ENABLED(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && lll->filter_policy) {
		*filter = ull_filter_lll_get(true);
	}
}

void lll_adv_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_adv *lll = p->param;
	struct pdu_adv *pdu;
	uint32_t overhead;
	uint32_t start_us;
	int err;

	DEBUG_RADIO_START_A(1);

#if defined(CONFIG_BT_PERIPHERAL)
	/* Not started if stopped on connection establishment, or when being
	 * disabled while connectable (cancelled flag set in thread context).
	 */
	if (unlikely(lll->conn && (lll->conn->periph.initiated || lll->conn->periph.cancelled))) {
		lll_event_abort(lll);

		return 0;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	overhead = lll_preempt_calc(p);
	if (overhead) {
		LL_ASSERT_OVERHEAD(overhead);

		lll_event_abort(lll);

		return -ECANCELED;
	}

	/* The primary channel PDUs are on the LE 1M PHY, the radio model has no
	 * LE Coded PHY.
	 */
	evt.cfg.aa = PDU_AC_ACCESS_ADDR;
	evt.cfg.crc_init = PDU_AC_CRC_IV;
	evt.cfg.phy = BSR_PHY_1M;
	evt.cfg.max_len = PDU_AC_LEG_PAYLOAD_SIZE_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	lll_adv_filter_get(lll, &evt.filter, &evt.resolve);

	start_us = lll_event_start_get(p, &evt.ticks_ref);

	lll->chan_map_curr = lll->chan_map;
	pdu = chan_tx(lll, start_us);

#if defined(CONFIG_BT_CTLR_ADV_EXT) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	if (lll->aux) {
		/* Fill the AuxPtr of the first PDU, read by the radio when its
		 * Tx starts.
		 */
		ull_adv_aux_lll_auxptr_fill(pdu, lll);

		/* The reference of the event, from which the aux offset of the
		 * later PDUs is calculated, is one tick before the first PDU.
		 */
		lll->aux->ticks_pri_pdu_offset += 1U;
	}
#else /* !CONFIG_BT_CTLR_ADV_EXT || !CONFIG_BT_TICKER_EXT_EXPIRE_INFO */
	/* The ULL has filled the AuxPtr of the first PDU */
	ARG_UNUSED(pdu);
#endif /* !CONFIG_BT_CTLR_ADV_EXT || !CONFIG_BT_TICKER_EXT_EXPIRE_INFO */

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_A(1);

	return 0;
}

#if defined(CONFIG_BT_PERIPHERAL)
static int resume_prepare_cb(struct lll_prepare_param *p)
{
	lll_resume_param_set(p);

	return prepare_cb(p);
}
#endif /* CONFIG_BT_PERIPHERAL */

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
#if defined(CONFIG_BT_PERIPHERAL)
	struct lll_adv *lll = curr;
	struct pdu_adv *pdu;
#endif /* CONFIG_BT_PERIPHERAL */

	if (next != curr) {
#if defined(CONFIG_BT_PERIPHERAL)
		if (lll->is_hdcd) {
			int err;

			/* Resume after the preempting event */
			*resume_cb = resume_prepare_cb;

			/* Retain HF clock */
			err = lll_hfclock_on();
			LL_ASSERT_ERR(err >= 0);

			return -EAGAIN;
		}
#endif /* CONFIG_BT_PERIPHERAL */

		return -ECANCELED;
	}

#if defined(CONFIG_BT_PERIPHERAL)
	/* A directed advertising event continues over its next prepare */
	pdu = lll_adv_data_curr_get(lll);
	if (pdu->type == PDU_ADV_TYPE_DIRECT_IND) {
		return 0;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_abort, param);

		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(param);
}

/* Transmit the advertising PDU on the next channel of the event, and return
 * it.
 */
static struct pdu_adv *chan_tx(struct lll_adv *lll, uint32_t at)
{
	struct pdu_adv *pdu;
	uint8_t chan;
	uint8_t upd;

	chan = find_lsb_set(lll->chan_map_curr);
	LL_ASSERT_DBG(chan);

	lll->chan_map_curr &= (lll->chan_map_curr - 1);

	evt.cfg.chan = 36 + chan;

	upd = 0U;
	pdu = lll_adv_data_latest_get(lll, &upd);
	LL_ASSERT_DBG(pdu);

	if ((pdu->type != PDU_ADV_TYPE_NONCONN_IND) &&
	    (!IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) || (pdu->type != PDU_ADV_TYPE_EXT_IND))) {
		struct pdu_adv *scan_pdu;

		scan_pdu = lll_adv_scan_rsp_latest_get(lll, &upd);
		LL_ASSERT_DBG(scan_pdu);

#if defined(CONFIG_BT_CTLR_PRIVACY)
		if (upd) {
			/* The scan response has the AdvA of the advertising PDU */
			(void)memcpy(&scan_pdu->scan_rsp.addr[0], &pdu->adv_ind.addr[0],
				     BDADDR_SIZE);
		}
#else /* !CONFIG_BT_CTLR_PRIVACY */
		ARG_UNUSED(scan_pdu);
#endif /* !CONFIG_BT_CTLR_PRIVACY */
	}

	lll_radio_tx(&evt.cfg, at, pdu, isr_tx, lll);

	return pdu;
}

/* End of the advertising PDU on a channel */
static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *node_rx;
	struct lll_adv *lll = param;
	struct pdu_adv *pdu;

	/* No SCAN_REQ or CONNECT_IND after a non-connectable and non-scannable
	 * PDU, nor after an ADV_EXT_IND.
	 */
	pdu = lll_adv_data_curr_get(lll);
	if ((e->status != BSR_STATUS_OK) || (pdu->type == PDU_ADV_TYPE_NONCONN_IND) ||
	    (IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) && (pdu->type == PDU_ADV_TYPE_EXT_IND))) {
		isr_done(e, lll);
		return;
	}

	/* Listen for a SCAN_REQ or CONNECT_IND */
	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(PHY_1M), node_rx->pdu, isr_rx, lll);
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_adv *lll = param;

	if (e->status == BSR_STATUS_OK) {
		struct node_rx_pdu *node_rx;
		struct lll_addr_match match;
		struct pdu_adv *pdu_rx;
		int err;

		node_rx = ull_pdu_rx_alloc_peek(1);
		LL_ASSERT_DBG(node_rx);

		pdu_rx = (void *)node_rx->pdu;
		lll_addr_match(pdu_rx, evt.filter, evt.resolve, &match);

		err = isr_rx_pdu(lll, e, pdu_rx, &match);
		if (!err) {
			return;
		}
	}

	isr_done(e, lll);
}

/* End of the radio operations on a channel: continue on the next one, or
 * close the event.
 */
static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_adv *lll = param;

	ARG_UNUSED(e);

#if defined(CONFIG_BT_PERIPHERAL)
	if (!IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) && lll->is_hdcd && !lll->chan_map_curr) {
		lll->chan_map_curr = lll->chan_map;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	/* Do not continue connectable advertising if advertising is being
	 * disabled, i.e. the cancelled flag is set.
	 */
	if (lll->chan_map_curr &&
#if defined(CONFIG_BT_PERIPHERAL)
	    (!lll->conn || !lll->conn->periph.cancelled) &&
#endif /* CONFIG_BT_PERIPHERAL */
	    1) {
		struct pdu_adv *pdu;
		uint32_t at;

		at = lll_radio_now() + ADV_CHAN_SWITCH_US;
		pdu = chan_tx(lll, at);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
		if (lll->aux) {
			uint32_t pdu_offset_us;

			/* Aux offset from this PDU, read by the radio when its
			 * Tx starts. The PDU is at pdu_offset_us from the
			 * reference of the event.
			 */
			pdu_offset_us = at - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
			(void)ull_adv_aux_lll_offset_fill(pdu, lll->aux->ticks_pri_pdu_offset,
							  lll->aux->us_pri_pdu_offset,
							  pdu_offset_us);
		}
#else /* !CONFIG_BT_CTLR_ADV_EXT */
		ARG_UNUSED(pdu);
#endif /* !CONFIG_BT_CTLR_ADV_EXT */

		return;
	}

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(3);

	if (node_rx) {
		ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_ADV_INDICATION;

		ull_rx_put_sched(node_rx->hdr.link, node_rx);
	}
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_CTLR_ADV_EXT) || defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
	struct event_done_extra *extra;

#if defined(CONFIG_BT_CTLR_ADV_EXT) && !defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
	/* The auxiliary advertising event generates the done event */
	if (lll->aux) {
		lll_isr_cleanup(lll);

		return;
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT && !CONFIG_BT_CTLR_JIT_SCHEDULING */

	extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_ADV);
	LL_ASSERT_ERR(extra);
#endif /* CONFIG_BT_CTLR_ADV_EXT || CONFIG_BT_CTLR_JIT_SCHEDULING */

	lll_isr_cleanup(lll);
}

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	lll_isr_cleanup(param);
}

static bool isr_rx_sr_adva_check(uint8_t tx_addr, const uint8_t *addr, const struct pdu_adv *sr)
{
	return (tx_addr == sr->rx_addr) && !memcmp(addr, sr->scan_req.adv_addr, BDADDR_SIZE);
}

bool lll_adv_scan_req_check(const struct lll_adv *lll, const struct pdu_adv *sr, uint8_t tx_addr,
			    const uint8_t *addr, uint8_t devmatch_ok, uint8_t *rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	return ((((lll->filter_policy & BT_LE_ADV_FP_FILTER_SCAN_REQ) == 0U) &&
		 ull_filter_lll_rl_addr_allowed(sr->tx_addr, sr->scan_req.scan_addr, rl_idx)) ||
		(((lll->filter_policy & BT_LE_ADV_FP_FILTER_SCAN_REQ) != 0U) &&
		 (devmatch_ok || ull_filter_lll_irk_in_fal(*rl_idx)))) &&
	       isr_rx_sr_adva_check(tx_addr, addr, sr);
#else /* !CONFIG_BT_CTLR_PRIVACY */
	return (((lll->filter_policy & BT_LE_ADV_FP_FILTER_SCAN_REQ) == 0U) || devmatch_ok) &&
	       isr_rx_sr_adva_check(tx_addr, addr, sr);
#endif /* !CONFIG_BT_CTLR_PRIVACY */
}

#if defined(CONFIG_BT_PERIPHERAL)
static bool isr_rx_ci_tgta_check(const struct lll_adv *lll, uint8_t rx_addr,
				 const uint8_t *tgt_addr, const struct pdu_adv *ci, uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	if ((rl_idx != FILTER_IDX_NONE) && (lll->rl_idx != FILTER_IDX_NONE)) {
		return rl_idx == lll->rl_idx;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	return (rx_addr == ci->tx_addr) &&
	       !memcmp(tgt_addr, ci->connect_ind.init_addr, BDADDR_SIZE);
}

static bool isr_rx_ci_adva_check(uint8_t tx_addr, const uint8_t *addr, const struct pdu_adv *ci)
{
	return (tx_addr == ci->rx_addr) && !memcmp(addr, ci->connect_ind.adv_addr, BDADDR_SIZE);
}

bool lll_adv_connect_ind_check(const struct lll_adv *lll, const struct pdu_adv *ci,
			       uint8_t tx_addr, const uint8_t *addr, uint8_t rx_addr,
			       const uint8_t *tgt_addr, uint8_t devmatch_ok, uint8_t *rl_idx)
{
	/* LL 4.3.2: filter policy shall be ignored for directed adv */
	if (tgt_addr) {
#if defined(CONFIG_BT_CTLR_PRIVACY)
		if (!ull_filter_lll_rl_addr_allowed(ci->tx_addr, ci->connect_ind.init_addr,
						    rl_idx)) {
			return false;
		}
#endif /* CONFIG_BT_CTLR_PRIVACY */

		return isr_rx_ci_adva_check(tx_addr, addr, ci) &&
		       isr_rx_ci_tgta_check(lll, rx_addr, tgt_addr, ci, *rl_idx);
	}

#if defined(CONFIG_BT_CTLR_PRIVACY)
	return ((((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) == 0U) &&
		 ull_filter_lll_rl_addr_allowed(ci->tx_addr, ci->connect_ind.init_addr, rl_idx)) ||
		(((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) != 0U) &&
		 (devmatch_ok || ull_filter_lll_irk_in_fal(*rl_idx)))) &&
	       isr_rx_ci_adva_check(tx_addr, addr, ci);
#else /* !CONFIG_BT_CTLR_PRIVACY */
	return (((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) == 0U) || devmatch_ok) &&
	       isr_rx_ci_adva_check(tx_addr, addr, ci);
#endif /* !CONFIG_BT_CTLR_PRIVACY */
}

/* Close the event after accepting a CONNECT_IND, and abort any of its
 * prepares or resumes in the pipeline.
 */
static void event_close_all(struct lll_adv *lll)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};
	uint32_t ret;

	lll_isr_cleanup(lll);

	mfy.param = lll;
	ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
	LL_ASSERT_ERR(!ret);
}
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
int lll_adv_scan_req_report(struct lll_adv *lll, const struct bsr_evt *e, uint8_t rl_idx)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (!node_rx) {
		return -ENOBUFS;
	}
	ull_pdu_rx_alloc();

	/* The SCAN_REQ is in the node rx PDU */
	node_rx->hdr.type = NODE_RX_TYPE_SCAN_REQ;
	node_rx->hdr.handle = ull_adv_lll_handle_get(lll);

	node_rx->rx_ftr.rssi = lll_rssi_get(e->rssi);
#if defined(CONFIG_BT_CTLR_PRIVACY)
	node_rx->rx_ftr.rl_idx = rl_idx;
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return 0;
}
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

static int isr_rx_pdu(struct lll_adv *lll, const struct bsr_evt *e, struct pdu_adv *pdu_rx,
		      const struct lll_addr_match *match)
{
	struct pdu_adv *pdu_adv;
	uint8_t *tgt_addr;
	uint8_t tx_addr;
	uint8_t rx_addr;
	uint8_t rl_idx;
	uint8_t *addr;

#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* An IRK match implies address resolution enabled */
	rl_idx = match->irkmatch_ok ? ull_filter_lll_rl_irk_idx(match->irkmatch_id) :
				      FILTER_IDX_NONE;
#else /* !CONFIG_BT_CTLR_PRIVACY */
	rl_idx = FILTER_IDX_NONE;
#endif /* !CONFIG_BT_CTLR_PRIVACY */

	pdu_adv = lll_adv_data_curr_get(lll);

	addr = pdu_adv->adv_ind.addr;
	tx_addr = pdu_adv->tx_addr;
	rx_addr = pdu_adv->rx_addr;

	if (pdu_adv->type == PDU_ADV_TYPE_DIRECT_IND) {
		tgt_addr = pdu_adv->direct_ind.tgt_addr;
	} else {
		tgt_addr = NULL;
	}

	if ((pdu_rx->type == PDU_ADV_TYPE_SCAN_REQ) &&
	    (pdu_rx->len == sizeof(struct pdu_adv_scan_req)) && (tgt_addr == NULL) &&
	    lll_adv_scan_req_check(lll, pdu_rx, tx_addr, addr, match->devmatch_ok, &rl_idx)) {
#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
		/* With extended advertising, only reported when enabled for the
		 * advertising set.
		 */
		if (!IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) || lll->scan_req_notify) {
			int err;

			/* Without a report, the scan response is not
			 * transmitted.
			 */
			err = lll_adv_scan_req_report(lll, e, rl_idx);
			if (err) {
				return err;
			}
		}
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

		lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, lll_adv_scan_rsp_curr_get(lll),
			     isr_done, lll);

		return 0;

#if defined(CONFIG_BT_PERIPHERAL)
	/* NOTE: Do not accept CONNECT_IND if cancelled flag is set in thread
	 *       context when disabling connectable advertising. This is to
	 *       avoid any race in checking the initiated flags in thread mode
	 *       which is set here if accepting a connection establishment.
	 *
	 *       Under this race, peer central would get failed to establish
	 *       connection as the disconnect reason. This is an acceptable
	 *       outcome to keep the thread mode implementation simple when
	 *       disabling connectable advertising.
	 */
	} else if ((pdu_rx->type == PDU_ADV_TYPE_CONNECT_IND) &&
		   (pdu_rx->len == sizeof(struct pdu_adv_connect_ind)) && lll->conn &&
		   !lll->conn->periph.cancelled &&
		   lll_adv_connect_ind_check(lll, pdu_rx, tx_addr, addr, rx_addr, tgt_addr,
					     match->devmatch_ok, &rl_idx)) {
		struct node_rx_ftr *ftr;
		struct node_rx_pdu *rx;

		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			rx = ull_pdu_rx_alloc_peek(4);
		} else {
			rx = ull_pdu_rx_alloc_peek(3);
		}

		if (!rx) {
			return -ENOBUFS;
		}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		lll->conn->rssi_latest = lll_rssi_get(e->rssi);
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

		/* Stop further LLL radio events */
		lll->conn->periph.initiated = 1U;

		/* The CONNECT_IND is in the node rx PDU */
		rx = ull_pdu_rx_alloc();

		rx->hdr.type = NODE_RX_TYPE_CONNECTION;
		rx->hdr.handle = 0xffff;

		ftr = &(rx->rx_ftr);
		ftr->param = lll;
		ftr->ticks_anchor = evt.ticks_ref;
		ftr->radio_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);

#if defined(CONFIG_BT_CTLR_PRIVACY)
		ftr->rl_idx = match->irkmatch_ok ? rl_idx : FILTER_IDX_NONE;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			ftr->extra = ull_pdu_rx_alloc();
		}

		ull_rx_put_sched(rx->hdr.link, rx);

		event_close_all(lll);

		return 0;
#endif /* CONFIG_BT_PERIPHERAL */
	}

	return -EINVAL;
}
