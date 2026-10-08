/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_chan.h"
#include "lll_df_types.h"
#include "lll_scan.h"
#include "lll_scan_aux.h"
#include "lll_sync.h"
#include "lll_filter.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_addr.h"
#include "lll_scan_internal.h"
#include "lll_sync_internal.h"

#include "ll_feat.h"

#include "hal/debug.h"

static void isr_rx_aux_chain(const struct bsr_evt *e, void *param);

static struct {
	struct bsr_pkt_cfg cfg;

	/* NULL in a periodic sync event */
	struct lll_scan_aux *lll_aux;

	/* The times reported to the ULL are relative to the current radio
	 * event.
	 */
	uint32_t ticks_ref;

	uint8_t phy;

	/* NODE_RX_TYPE_SYNC until the sync is established */
	uint8_t node_type;
	uint8_t sync_status;

	/* For the ULL to compensate the drift and supervise the sync */
	uint32_t rx_start;
	uint32_t aa_end;
	uint8_t trx_cnt;
	uint8_t crc_ok;
} evt;

int lll_sync_init(void)
{
	return 0;
}

static void cfg_set(const struct lll_sync *lll, uint8_t phy, uint8_t chan)
{
	evt.phy = phy;

	evt.cfg.aa = sys_get_le32(lll->access_addr);
	evt.cfg.crc_init = sys_get_le24(lll->crc_init);
	evt.cfg.phy = lll_radio_phy(phy);
	evt.cfg.chan = chan;
	evt.cfg.max_len = LL_EXT_OCTETS_RX_MAX;
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
}

static void rx(struct lll_sync *lll, uint32_t start, uint32_t window_us, lll_radio_cb_t cb)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_radio_rx(&evt.cfg, start, window_us, node_rx->pdu, cb, lll);
}

static uint8_t data_channel_calc(struct lll_sync *lll, uint16_t event_counter)
{
	if (lll->chm_first != lll->chm_last) {
		uint16_t instant_latency;

		instant_latency = (event_counter - lll->chm_instant) & EVENT_INSTANT_MAX;
		if (instant_latency <= EVENT_INSTANT_LATENCY_MAX) {
			lll->chm_first = lll->chm_last;
		}
	}

	return lll_chan_sel_2(event_counter, lll->data_chan_id,
			      lll->chm[lll->chm_first].data_chan_map,
			      lll->chm[lll->chm_first].data_chan_count);
}

static void isr_rx_done_cleanup(struct lll_sync *lll)
{
	struct event_done_extra *e;

	/* The ULL associates an auxiliary context with the sync again for the
	 * chain PDUs of the next event.
	 */
	lll->lll_aux = NULL;

	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_SYNC;
	e->trx_cnt = evt.trx_cnt;
	e->crc_valid = evt.crc_ok;

	if (evt.trx_cnt != 0U) {
		e->drift.start_to_address_actual_us = evt.aa_end - evt.rx_start;
		e->drift.window_widening_event_us = lll->window_widening_event_us;
		e->drift.preamble_to_addr_us = addr_us_get(lll->phy);

		/* Synchronized again on the anchor point received */
		lll->window_widening_event_us = 0U;
		lll->window_size_event_us = 0U;

#if defined(CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN)
		/* Having received its anchor point, the sync no longer has
		 * precedence over the overlapping ones.
		 */
		lll->abort_count = 0U;
#endif /* CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN */
	}

	lll_isr_cleanup(lll);
}

void lll_sync_isr_aux_release(struct lll_sync *lll, struct lll_scan_aux *lll_aux)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc();
	LL_ASSERT_ERR(node_rx != NULL);

	node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_RELEASE;

	node_rx->rx_ftr.param = lll;
	node_rx->rx_ftr.lll_aux = lll_aux;
	node_rx->rx_ftr.aux_failed = 1U;

	ull_rx_put_sched(node_rx->hdr.link, node_rx);
}

/* lll_aux is NULL for the PDU at the anchor point. Returns -EBUSY if the
 * chain PDU that the AuxPtr points to is received in this radio event, as it
 * starts too soon for the ULL to schedule its reception.
 */
static int isr_rx_report(struct lll_sync *lll, uint8_t node_type, const struct bsr_evt *e,
			 struct lll_scan_aux *lll_aux)
{
	struct lll_scan_aux_rx aux_rx;
	struct node_rx_pdu *node_rx;
	struct node_rx_ftr *ftr;
	bool aux_lll_sched;

	/* A node for the report, one for the report of incomplete data of a
	 * chain unless this is a chain PDU, and 2 kept for connections.
	 */
	if (node_type != NODE_RX_TYPE_EXT_AUX_REPORT) {
		node_rx = ull_pdu_rx_alloc_peek(4);
	} else {
		node_rx = ull_pdu_rx_alloc_peek(3);
	}
	if (node_rx == NULL) {
		/* A chain PDU not reported ends the chain */
		return (node_type == NODE_RX_TYPE_EXT_AUX_REPORT) ? -ENOMEM : 0;
	}

	/* The next reception uses the next free node rx */
	(void)ull_pdu_rx_alloc();

	node_rx->hdr.type = node_type;

	ftr = &node_rx->rx_ftr;
	ftr->param = lll;
	ftr->aux_failed = 0U;
	ftr->rssi = lll_rssi_get(e->rssi);
	ftr->ticks_anchor = evt.ticks_ref;
	ftr->radio_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
	ftr->phy_flags = 0U;
	ftr->sync_status = evt.sync_status;
	ftr->sync_rx_enabled = lll->is_rx_enabled;

	if (node_type != NODE_RX_TYPE_EXT_AUX_REPORT) {
		/* For the report of incomplete data of a chain */
		ftr->extra = ull_pdu_rx_alloc();
	} else {
		ftr->lll_aux = lll_aux;
	}

	aux_lll_sched = lll_scan_aux_rx_get((void *)node_rx->pdu, evt.phy, e->ts_start, &aux_rx);
	if (aux_lll_sched) {
		if (node_type != NODE_RX_TYPE_EXT_AUX_REPORT) {
			lll->is_aux_sched = 1U;
		}

		cfg_set(lll, aux_rx.phy, aux_rx.chan);
		rx(lll, aux_rx.start, aux_rx.window_us, isr_rx_aux_chain);
	}

	ftr->aux_lll_sched = aux_lll_sched;

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return aux_lll_sched ? -EBUSY : 0;
}

static void isr_rx_aux_chain(const struct bsr_evt *e, void *param)
{
	struct lll_sync *lll = param;
	struct lll_scan_aux *lll_aux;
	int err;

	/* In the periodic sync event, the ULL associates the auxiliary context
	 * with the sync once it has the report of the PDU with the AuxPtr.
	 */
	lll_aux = (evt.lll_aux != NULL) ? evt.lll_aux : lll->lll_aux;

	/* Without an auxiliary context, the chain is dropped */
	err = -EINVAL;
	if ((lll_aux != NULL) && (e->status == BSR_STATUS_OK)) {
		err = isr_rx_report(lll, NODE_RX_TYPE_EXT_AUX_REPORT, e, lll_aux);
		if (err == -EBUSY) {
			return;
		}
	}

	/* The chain ends before its last PDU */
	if (err != 0) {
		lll_sync_isr_aux_release(lll, lll_aux);
	}

	if (evt.lll_aux == NULL) {
		lll->is_aux_sched = 0U;

		isr_rx_done_cleanup(lll);
	} else {
		lll_isr_cleanup(evt.lll_aux);
	}
}

static void isr_rx_sync(const struct bsr_evt *e, void *param)
{
	struct lll_sync *lll = param;

	if ((e->status == BSR_STATUS_OK) || (e->status == BSR_STATUS_CRC_ERR)) {
		/* Anchor point received, for the drift compensation */
		evt.trx_cnt = 1U;
		evt.aa_end = e->ts_aa_end;
		evt.crc_ok = (e->status == BSR_STATUS_OK);

		/* The chain PDUs are received in this event */
		if ((evt.crc_ok != 0U) && (isr_rx_report(lll, evt.node_type, e, NULL) == -EBUSY)) {
			return;
		}
	}

	isr_rx_done_cleanup(lll);
}

static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_sync *lll = param;

	ARG_UNUSED(e);

	/* Not all the chain PDUs being received in the event were received */
	if (lll->is_aux_sched != 0U) {
		lll->is_aux_sched = 0U;

		lll_sync_isr_aux_release(lll, lll->lll_aux);
	}

	isr_rx_done_cleanup(lll);
}

static int prepare_cb_common(struct lll_prepare_param *p, uint8_t node_type, uint8_t sync_status)
{
	struct lll_sync *lll = p->param;
	uint16_t event_counter;
	uint32_t window_us;
	uint8_t chan_idx;
	int err;

	DEBUG_RADIO_START_O(1);

	lll->lazy_prepare = p->lazy;
	lll->skip_event = lll->skip_prepare + lll->lazy_prepare;
	lll->skip_prepare = 0U;

	event_counter = lll->event_counter + lll->skip_event;
	chan_idx = data_channel_calc(lll, event_counter);
	lll->event_counter = event_counter + 1U;

	lll->window_widening_prepare_us += lll->window_widening_periodic_us *
					   (lll->lazy_prepare + 1U);
	if (lll->window_widening_prepare_us > lll->window_widening_max_us) {
		lll->window_widening_prepare_us = lll->window_widening_max_us;
	}

	lll->window_widening_event_us += lll->window_widening_prepare_us;
	lll->window_widening_prepare_us = 0U;
	if (lll->window_widening_event_us > lll->window_widening_max_us) {
		lll->window_widening_event_us = lll->window_widening_max_us;
	}

	lll->is_aux_sched = 0U;

	evt.lll_aux = NULL;
	evt.node_type = node_type;
	evt.sync_status = sync_status;
	evt.trx_cnt = 0U;
	evt.crc_ok = 0U;

	if (lll_preempt_calc(p) != 0U) {
		lll_radio_stop(isr_done, lll);

		return -ECANCELED;
	}

	cfg_set(lll, lll->phy, chan_idx);

	/* The ULL starts the event the jitter, ticker resolution margin and
	 * window widening before the earliest expected anchor point. Listen
	 * until the latest one, plus the window of the offset unit of the
	 * SyncInfo in the first event.
	 */
	evt.rx_start = lll_event_start_get(p, &evt.ticks_ref);
	window_us = ((EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
		      lll->window_widening_event_us) << 1) +
		    lll->window_size_event_us + addr_us_get(lll->phy);
	rx(lll, evt.rx_start, window_us, isr_rx_sync);

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(err == 0);

	DEBUG_RADIO_START_O(1);

	return 0;
}

static int create_prepare_cb(struct lll_prepare_param *p)
{
	/* Without CTEInline support in the radio, the ULL checks the CTE type
	 * of the PDU that establishes the sync.
	 */
	return prepare_cb_common(p, NODE_RX_TYPE_SYNC, SYNC_STAT_ALLOWED);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	/* Once synchronized, a change of the CTE type does not affect the
	 * sync.
	 */
	return prepare_cb_common(p, NODE_RX_TYPE_SYNC_REPORT, SYNC_STAT_READY);
}

#if defined(CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN)
static int sync_is_abort(struct lll_sync *curr, struct lll_sync *next)
{
	if (next == NULL) {
		/* Not aborted when near the supervision timeout */
		return (curr->forced != 0U) ? 0 : -ECANCELED;
	}

	/* Overlapping sync events take turns at being aborted */
	if (curr->abort_count < next->abort_count) {
		if (curr->abort_count < UINT8_MAX) {
			curr->abort_count++;
		}

		return -ECANCELED;
	}

	if (next->abort_count < UINT8_MAX) {
		next->abort_count++;
	}

	return 0;
}
#endif /* CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN */

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	/* A periodic sync event is not resumed */
	ARG_UNUSED(resume_cb);

	/* The next event of the same sync is not started, as the current one
	 * continues.
	 */
	if (next == curr) {
		return 0;
	}

	/* The scan event that is next continues after this event */
	if (ull_scan_lll_is_valid_get(next) != NULL) {
		return 0;
	}

	/* As does the auxiliary scan event that is next */
	if (!IS_ENABLED(CONFIG_BT_CTLR_SYNC_PERIODIC_SKIP_ON_SCAN_AUX) &&
	    (ull_scan_aux_lll_is_valid_get(next) != NULL)) {
		return 0;
	}

#if defined(CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN)
	return sync_is_abort(curr, ull_sync_lll_is_valid_get(next));
#else /* !CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN */
	return -ECANCELED;
#endif /* !CONFIG_BT_CTLR_SCAN_AUX_SYNC_RESERVE_MIN */
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct event_done_extra *e;
	struct lll_sync *lll;
	int err;

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* The event counter and window widening of the next event account for
	 * the events aborted in the prepare pipeline.
	 */
	lll = prepare_param->param;
	lll->lazy_prepare = prepare_param->lazy;
	lll->skip_prepare += (lll->lazy_prepare + 1U);

	lll->window_widening_prepare_us += lll->window_widening_periodic_us *
					   (prepare_param->lazy + 1U);
	if (lll->window_widening_prepare_us > lll->window_widening_max_us) {
		lll->window_widening_prepare_us = lll->window_widening_max_us;
	}

	/* For the ULL to count the event as missed, for the sync timeout */
	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_SYNC;
	e->trx_cnt = 0U;
	e->crc_valid = 0U;

	lll_done(param);
}

void lll_sync_create_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, create_prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

void lll_sync_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

void lll_sync_aux_prepare_cb(struct lll_sync *lll, struct lll_scan_aux *lll_aux, uint32_t start_us,
			     uint32_t ticks_ref)
{
	evt.lll_aux = lll_aux;
	evt.ticks_ref = ticks_ref;
	cfg_set(lll, lll_aux->phy, lll_aux->chan);

	/* A PDU that starts at the end of the window is received too */
	rx(lll, start_us, lll_aux->window_size_us + addr_us_get(lll_aux->phy), isr_rx_aux_chain);
}
