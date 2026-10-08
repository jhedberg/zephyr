/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"
#include "hal/radio.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_clock.h"
#include "lll_df_types.h"
#include "lll_conn.h"
#include "lll_chan.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_conn_internal.h"

#include "hal/debug.h"

static void rx(struct lll_conn *lll, uint32_t start, uint32_t window_us);

static struct {
	struct bsr_pkt_cfg cfg;

	/* For the drift compensation of a peripheral event */
	uint32_t rx_start;
	uint32_t aa_end;

	uint16_t tx_cnt;
	uint16_t trx_cnt;
	uint8_t crc_valid;
	uint8_t crc_expire;
	uint8_t is_aborted;
	uint8_t trx_busy_iteration;
} evt;

static struct pdu_data pdu_empty;

#if defined(CONFIG_BT_CTLR_FORCE_MD_COUNT) && \
	(CONFIG_BT_CTLR_FORCE_MD_COUNT > 0)
#if defined(CONFIG_BT_CTLR_FORCE_MD_AUTO)
static uint8_t force_md_cnt_reload;
#define BT_CTLR_FORCE_MD_COUNT force_md_cnt_reload
#else /* !CONFIG_BT_CTLR_FORCE_MD_AUTO */
#define BT_CTLR_FORCE_MD_COUNT CONFIG_BT_CTLR_FORCE_MD_COUNT
#endif /* !CONFIG_BT_CTLR_FORCE_MD_AUTO */
static uint8_t force_md_cnt;

#define FORCE_MD_CNT_INIT() \
	do { \
		force_md_cnt = 0U; \
	} while (false)

#define FORCE_MD_CNT_DEC() \
	do { \
		if (force_md_cnt != 0U) { \
			force_md_cnt--; \
		} \
	} while (false)

#define FORCE_MD_CNT_GET() force_md_cnt

#define FORCE_MD_CNT_SET() \
	do { \
		if ((force_md_cnt != 0U) || \
		    (evt.trx_cnt >= (CONFIG_BT_BUF_ACL_TX_COUNT - 1))) { \
			force_md_cnt = BT_CTLR_FORCE_MD_COUNT; \
		} \
	} while (false)

#else /* !CONFIG_BT_CTLR_FORCE_MD_COUNT */
#define FORCE_MD_CNT_INIT()
#define FORCE_MD_CNT_DEC()
#define FORCE_MD_CNT_GET() 0
#define FORCE_MD_CNT_SET()
#endif /* !CONFIG_BT_CTLR_FORCE_MD_COUNT */

static inline uint8_t phy_tx_get(const struct lll_conn *lll)
{
#if defined(CONFIG_BT_CTLR_PHY)
	return lll->phy_tx;
#else /* !CONFIG_BT_CTLR_PHY */
	return PHY_1M;
#endif /* !CONFIG_BT_CTLR_PHY */
}

static inline uint8_t phy_rx_get(const struct lll_conn *lll)
{
#if defined(CONFIG_BT_CTLR_PHY)
	return lll->phy_rx;
#else /* !CONFIG_BT_CTLR_PHY */
	return PHY_1M;
#endif /* !CONFIG_BT_CTLR_PHY */
}

int lll_conn_init(void)
{
	pdu_empty.ll_id = PDU_DATA_LLID_DATA_CONTINUE;

	return 0;
}

int lll_conn_reset(void)
{
	FORCE_MD_CNT_INIT();

	return 0;
}

void lll_conn_flush(uint16_t handle, struct lll_conn *lll)
{
}

#if defined(CONFIG_BT_CTLR_FORCE_MD_AUTO)
uint8_t lll_conn_force_md_cnt_set(uint8_t reload_cnt)
{
	uint8_t previous;

	previous = force_md_cnt_reload;
	force_md_cnt_reload = reload_cnt;

	return previous;
}
#endif /* CONFIG_BT_CTLR_FORCE_MD_AUTO */

void lll_conn_prepare_reset(void)
{
	evt.tx_cnt = 0U;
	evt.trx_cnt = 0U;
	evt.crc_valid = 0U;
	evt.crc_expire = 0U;
	evt.is_aborted = 0U;
	evt.trx_busy_iteration = 0U;
	evt.aa_end = 0U;
}

uint8_t lll_conn_event_setup(struct lll_conn *lll, const struct lll_prepare_param *p)
{
	uint16_t event_counter;

	lll_conn_prepare_reset();

	lll->lazy_prepare = p->lazy;
	lll->latency_event = lll->latency_prepare + lll->lazy_prepare;

	event_counter = lll->event_counter + lll->latency_event;
	lll->event_counter = event_counter + 1U;

	lll->latency_prepare = 0U;

	if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2) && (lll->data_chan_sel != 0U)) {
		return lll_chan_sel_2(event_counter, lll->data_chan_id, &lll->data_chan_map[0],
				      lll->data_chan_count);
	}

	return lll_chan_sel_1(&lll->data_chan_use, lll->data_chan_hop, lll->latency_event,
			      &lll->data_chan_map[0], lll->data_chan_count);
}

static void cfg_set(const struct lll_conn *lll, uint8_t chan)
{
	evt.cfg.aa = sys_get_le32(lll->access_addr);
	evt.cfg.crc_init = sys_get_le24(lll->crc_init);
	evt.cfg.chan = chan;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
}

static void event_done(struct lll_conn *lll)
{
	struct event_done_extra *e;

	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = evt.trx_cnt;
	e->crc_valid = evt.crc_valid;
	e->is_aborted = evt.is_aborted;

#if defined(CONFIG_BT_PERIPHERAL)
	if ((evt.trx_cnt != 0U) && (lll->role == BT_HCI_ROLE_PERIPHERAL)) {
		uint8_t phy_rx;

#if defined(CONFIG_BT_CTLR_PHY)
		phy_rx = lll->periph.phy_rx_event;
#else /* !CONFIG_BT_CTLR_PHY */
		phy_rx = PHY_1M;
#endif /* !CONFIG_BT_CTLR_PHY */

		e->drift.start_to_address_actual_us = evt.aa_end - evt.rx_start;
		e->drift.window_widening_event_us = lll->periph.window_widening_event_us;
		e->drift.preamble_to_addr_us = addr_us_get(phy_rx);

		/* The anchor point is in sync again */
		lll->periph.window_widening_event_us = 0U;
		lll->periph.window_size_event_us = 0U;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	lll_isr_cleanup(lll);
}

static void isr_tx_last(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	event_done(param);
}

static void tx(struct lll_conn *lll, uint32_t at, struct pdu_data *pdu, lll_radio_cb_t cb)
{
	evt.cfg.phy = lll_radio_phy(phy_tx_get(lll));

	lll_radio_tx(&evt.cfg, at, pdu, cb, lll);
}

static void isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_rx, uint8_t *is_rx_enqueue,
		       struct node_tx **tx_release, uint8_t *is_done)
{
	if (pdu_rx->nesn != lll->sn) {
		struct pdu_data *pdu_tx;
		struct node_tx *tx;
		memq_link_t *link;

		lll->sn++;

#if defined(CONFIG_BT_PERIPHERAL)
		/* First ack (and redundantly any other ack) enable use of
		 * peripheral latency.
		 */
		if (lll->role == BT_HCI_ROLE_PERIPHERAL) {
			lll->periph.latency_enabled = 1U;
		}
#endif /* CONFIG_BT_PERIPHERAL */

		FORCE_MD_CNT_DEC();

		if (lll->empty == 0U) {
			link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, (void **)&tx);
		} else {
			lll->empty = 0U;

			pdu_tx = &pdu_empty;
			if (IS_ENABLED(CONFIG_BT_CENTRAL) && (lll->role == BT_HCI_ROLE_CENTRAL) &&
			    (pdu_rx->md == 0U)) {
				*is_done = (pdu_tx->md == 0U);
			}

			link = NULL;
		}

		if (link != NULL) {
			uint8_t pdu_tx_len;
			uint8_t offset;

			pdu_tx = (void *)(tx->pdu + lll->packet_tx_head_offset);

			pdu_tx_len = pdu_tx->len;

			offset = lll->packet_tx_head_offset + pdu_tx_len;
			if (offset < lll->packet_tx_head_len) {
				lll->packet_tx_head_offset = offset;
			} else if (offset == lll->packet_tx_head_len) {
				lll->packet_tx_head_len = 0U;
				lll->packet_tx_head_offset = 0U;

				memq_dequeue(lll->memq_tx.tail, &lll->memq_tx.head, NULL);

				/* Keep tx->next, which tells the ULL the pool of the node */
				link->next = tx->next;
				tx->next = link;

				*tx_release = tx;

				FORCE_MD_CNT_SET();
			} else {
				LL_ASSERT_DBG(0);
			}

			if (IS_ENABLED(CONFIG_BT_CENTRAL) && (lll->role == BT_HCI_ROLE_CENTRAL) &&
			    (pdu_rx->md == 0U)) {
				*is_done = (pdu_tx->md == 0U);
			}
		}
	}

	/* Process received data, never using the rx buffers reserved for an
	 * empty packet and internal control enqueue.
	 */
	if ((pdu_rx->sn == lll->nesn) && (ull_pdu_rx_alloc_peek(3) != NULL)) {
		lll->nesn++;

		if (pdu_rx->len != 0U) {
			*is_rx_enqueue = 1U;
		}
	}
}

static struct pdu_data *tx_prep(struct lll_conn *lll)
{
	struct node_tx *tx;
	struct pdu_data *p;
	memq_link_t *link;

	link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, (void **)&tx);
	if ((lll->empty != 0U) || (link == NULL)) {
		lll->empty = 1U;

		p = &pdu_empty;
		if ((link != NULL) || (FORCE_MD_CNT_GET() != 0U)) {
			p->md = 1U;
		} else {
			p->md = 0U;
		}
	} else {
		uint16_t max_tx_octets;

		p = (void *)(tx->pdu + lll->packet_tx_head_offset);

		if (lll->packet_tx_head_len == 0U) {
			lll->packet_tx_head_len = p->len;
		}

		if (lll->packet_tx_head_offset != 0U) {
			p->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
		}

		p->len = lll->packet_tx_head_len - lll->packet_tx_head_offset;

		max_tx_octets = ull_conn_lll_max_tx_octets_get(lll);

		if (((PDU_DC_CTRL_TX_SIZE_MAX <= PDU_DC_PAYLOAD_SIZE_MIN) ||
		     (p->ll_id != PDU_DATA_LLID_CTRL)) &&
		    (p->len > max_tx_octets)) {
			p->len = max_tx_octets;
			p->md = 1U;
		} else if ((link->next != lll->memq_tx.tail) || (FORCE_MD_CNT_GET() != 0U)) {
			p->md = 1U;
		} else {
			p->md = 0U;
		}

		p->rfu = 0U;
	}

	return p;
}

static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_conn *lll = param;
	uint32_t start;
	uint32_t end;

	if (e->status != BSR_STATUS_OK) {
		event_done(lll);
		return;
	}

	evt.tx_cnt++;

	/* The peer transmits tIFS after the end of our packet: listen from
	 * the earliest time its packet can start, and give up if no access
	 * address has been received by the latest time one can end.
	 */
	start = e->ts_end + lll->tifs_rx_us - EVENT_CLOCK_JITTER_US;
	end = e->ts_end + lll->tifs_hcto_us + EVENT_CLOCK_JITTER_US + RANGE_DELAY_US +
	      addr_us_get(phy_rx_get(lll));

	rx(lll, start, end - start);

#if defined(CONFIG_BT_CTLR_LOW_LAT)
	ull_conn_lll_tx_demux_sched(lll);
#endif /* CONFIG_BT_CTLR_LOW_LAT */
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	uint8_t is_empty_pdu_tx_retry;
	struct node_rx_pdu *node_rx;
	struct node_tx *tx_release;
	struct pdu_data *pdu_tx;
	struct pdu_data *pdu_rx;
	struct lll_conn *lll;
	uint8_t is_rx_enqueue;
	bool is_closed;
	uint8_t is_done;
	bool crc_ok;

	lll = param;

	/* No packet received, or the event has been stopped */
	if ((e->status != BSR_STATUS_OK) && (e->status != BSR_STATUS_CRC_ERR)) {
		event_done(lll);
		return;
	}

	evt.trx_cnt++;

	if (evt.trx_cnt == 1U) {
		/* First packet of the event, as anchor point */
		evt.aa_end = e->ts_aa_end;

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		/* One RSSI sample per event, from its first packet received */
		lll->rssi_latest = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_CONN_RSSI_EVENT)
		if (((lll->rssi_reported - lll->rssi_latest) & BIT_MASK(8)) >
		    LLL_CONN_RSSI_THRESHOLD) {
			if (lll->rssi_sample_count != 0U) {
				lll->rssi_sample_count--;
			}
		} else {
			lll->rssi_sample_count = LLL_CONN_RSSI_SAMPLE_COUNT;
		}
#endif /* CONFIG_BT_CTLR_CONN_RSSI_EVENT */
#endif /* CONFIG_BT_CTLR_CONN_RSSI */
	}

	is_done = 0U;
	is_closed = false;
	tx_release = NULL;
	is_rx_enqueue = 0U;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	pdu_rx = (void *)node_rx->pdu;

	crc_ok = (e->status == BSR_STATUS_OK);
	if (crc_ok) {
		isr_rx_pdu(lll, pdu_rx, &is_rx_enqueue, &tx_release, &is_done);

		evt.crc_expire = 0U;

		/* For the supervision timeout */
		evt.crc_valid = 1U;
	} else {
		/* Two consecutive CRC errors close the event (Core Spec Vol 6,
		 * Part B, Section 4.5.6).
		 */
		if (evt.crc_expire == 0U) {
			evt.crc_expire = 2U;
		}

		evt.crc_expire--;
		is_done = (evt.crc_expire == 0U);
	}

	is_empty_pdu_tx_retry = lll->empty;
	pdu_tx = tx_prep(lll);

#if defined(CONFIG_BT_PERIPHERAL)
	/* Close early so that the drift compensation is done before this event
	 * overlaps with the next interval.
	 */
	is_done = (is_done != 0U) || ((lll->role == BT_HCI_ROLE_PERIPHERAL) &&
				      (lll->periph.window_size_event_us != 0U));
#endif /* CONFIG_BT_PERIPHERAL */

	is_done = (is_done != 0U) || (crc_ok && (pdu_rx->md == 0U) && (pdu_tx->md == 0U) &&
				      (pdu_tx->len == 0U));

	/* Do not continue anymore if this event had continued despite an abort requested by same
	 * connection instance when overlapping due to connection event length being larger than
	 * the connection interval.
	 */
	is_done = (is_done != 0U) || (evt.trx_busy_iteration != 0U);

	if ((is_done != 0U) && (lll->role == BT_HCI_ROLE_CENTRAL)) {
		/* The central does not transmit anymore: restore the state
		 * if the last transmitted PDU was an empty PDU.
		 */
		lll->empty = is_empty_pdu_tx_retry;
		is_closed = true;

		goto isr_rx_exit;
	}

	pdu_tx->sn = lll->sn;
	pdu_tx->nesn = lll->nesn;

	/* A peripheral always responds, and closes its event after the last
	 * response.
	 */
	tx(lll, e->ts_end + lll->tifs_tx_us, pdu_tx, (is_done != 0U) ? isr_tx_last : isr_tx);

isr_rx_exit:
	if (tx_release != NULL) {
		LL_ASSERT_DBG(lll->handle != LLL_HANDLE_INVALID);

		ull_conn_lll_ack_enqueue(lll->handle, tx_release);
	}

	if (is_rx_enqueue != 0U) {
		ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_DC_PDU;
		node_rx->hdr.handle = lll->handle;

		ull_rx_put(node_rx->hdr.link, node_rx);
	}

	if ((tx_release != NULL) || (is_rx_enqueue != 0U)) {
		ull_rx_sched();
	}

	if (is_closed) {
		event_done(lll);
	}
}

static void rx(struct lll_conn *lll, uint32_t start, uint32_t window_us)
{
	struct node_rx_pdu *node_rx;
	uint16_t max_rx_octets;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	max_rx_octets = lll->dle.eff.max_rx_octets;
#else /* !CONFIG_BT_CTLR_DATA_LENGTH */
	max_rx_octets = PDU_DC_PAYLOAD_SIZE_MIN;
#endif /* !CONFIG_BT_CTLR_DATA_LENGTH */

	if ((PDU_DC_CTRL_RX_SIZE_MAX > PDU_DC_PAYLOAD_SIZE_MIN) &&
	    (max_rx_octets < PDU_DC_CTRL_RX_SIZE_MAX)) {
		max_rx_octets = PDU_DC_CTRL_RX_SIZE_MAX;
	}

	evt.cfg.phy = lll_radio_phy(phy_rx_get(lll));

	evt.cfg.max_len = max_rx_octets;

	lll_radio_rx(&evt.cfg, start, window_us, node_rx->pdu, isr_rx, lll);
}

void lll_conn_central_start(struct lll_conn *lll, uint8_t chan, uint32_t start_us)
{
	struct pdu_data *pdu_tx;

	cfg_set(lll, chan);

	pdu_tx = tx_prep(lll);
	pdu_tx->sn = lll->sn;
	pdu_tx->nesn = lll->nesn;

	tx(lll, start_us, pdu_tx, isr_tx);
}

void lll_conn_peripheral_start(struct lll_conn *lll, uint8_t chan, uint32_t start_us,
			       uint32_t window_us)
{
	cfg_set(lll, chan);

	evt.rx_start = start_us;

	rx(lll, start_us, window_us);
}

/* A packet can be longer than a short connection interval, so the next event of
 * the same connection is deferred a few times rather than abort the current one.
 */
#define TRX_BUSY_ITERATION_MAX MIN(4U, (EVENT_DEFER_MAX))

/* An event is not aborted by its own next event until it has completed its
 * first Rx as a central or Tx as a peripheral (cnt), as a single exchange can
 * last longer than the connection interval.
 */
static int is_abort(const struct lll_conn *lll, const void *next, uint16_t cnt)
{
	if (next != lll) {
		/* Do not be aborted by a different event if near supervision timeout */
		if ((lll->forced == 1U) && (cnt < 1U)) {
			return 0;
		}
	} else if ((cnt < 1U) && (evt.trx_busy_iteration < TRX_BUSY_ITERATION_MAX)) {
		evt.trx_busy_iteration++;

		return -EBUSY;
	}

	/* Without deferral support (EVENT_DEFER_MAX is 0) the event is always cancelled */
	LL_ASSERT_DBG((TRX_BUSY_ITERATION_MAX == 0U) ||
		      (evt.trx_busy_iteration < TRX_BUSY_ITERATION_MAX));

	return -ECANCELED;
}

#if defined(CONFIG_BT_CENTRAL)
int lll_conn_central_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	return is_abort(curr, next, evt.trx_cnt);
}
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
int lll_conn_peripheral_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	return is_abort(curr, next, evt.tx_cnt);
}
#endif /* CONFIG_BT_PERIPHERAL */

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	event_done(param);
}

void lll_conn_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct event_done_extra *e;
	struct lll_conn *lll;
	int err;

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		lll = param;

		/* For a peripheral role, ensure at least one PDU is tx-ed
		 * back to central, otherwise let the supervision timeout
		 * countdown be started.
		 */
		if ((lll->role == BT_HCI_ROLE_PERIPHERAL) && (evt.tx_cnt < 1U)) {
			evt.is_aborted = 1U;
		}

		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_abort, lll);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll = prepare_param->param;

	/* Accumulate the latency as event is aborted while being in pipeline */
	lll->lazy_prepare = prepare_param->lazy;
	lll->latency_prepare += (lll->lazy_prepare + 1U);

#if defined(CONFIG_BT_PERIPHERAL)
	if (lll->role == BT_HCI_ROLE_PERIPHERAL) {
		lll->periph.window_widening_prepare_us += lll->periph.window_widening_periodic_us *
							  (prepare_param->lazy + 1U);
		if (lll->periph.window_widening_prepare_us > lll->periph.window_widening_max_us) {
			lll->periph.window_widening_prepare_us = lll->periph.window_widening_max_us;
		}
	}
#endif /* CONFIG_BT_PERIPHERAL */

	/* Extra done event, to check supervision timeout */
	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = 0U;
	e->crc_valid = 0U;
	e->is_aborted = 1U;

	lll_done(param);
}
