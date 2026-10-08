/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
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
#include "util/mem.h"
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
#include "lll_conn.h"
#include "lll_conn_iso.h"
#include "lll_central_iso.h"
#include "lll_peripheral_iso.h"
#include "lll_iso_tx.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_ccm.h"

#include "ll_feat.h"

#include "hal/debug.h"

#define CIS_PDU_BUF_SIZE (offsetof(struct pdu_cis, payload) + \
			  MAX(LL_CIS_OCTETS_TX_MAX, LL_CIS_OCTETS_RX_MAX) + PDU_MIC_SIZE)

static bool se_next(struct lll_conn_iso_group *cig);

struct cis_evt {
	struct lll_conn_iso_stream *lll;

	uint16_t chan_id;
	uint16_t prn_s;
	uint16_t remap_idx;

	/* Subevents started in the event, the last one being the current
	 * subevent of the CIS.
	 */
	uint8_t se;

	/* Closed for the event, or torn down by the ULL during it */
	uint8_t is_closed:1;
};

static struct {
	struct bsr_pkt_cfg cfg;

	/* Start of the event: the CIG reference point for the central, and
	 * for the peripheral the start of its receive window, the jitter,
	 * ticker resolution margin and window widening before it.
	 */
	uint32_t start_us;

	/* Peripheral receive window of a subevent until a PDU has been
	 * received in the event, without the access address.
	 */
	uint32_t window_us;

	/* CIG reference point that the first PDU received by the peripheral
	 * in the event gives, and the access address duration of that PDU.
	 */
	uint32_t ref_us;
	uint32_t ref_addr_us;

	/* Start of the last PDU received by the peripheral, the offset of its
	 * subevent from the CIG reference point, and the position of the
	 * subevent among the subevents started in the event.
	 */
	uint32_t last_start_us;
	uint32_t last_offset_us;
	uint16_t last_idx;

	/* Subevents started in the event */
	uint16_t idx;

	/* CISes with a valid PDU received in the event, by CIS index */
	uint32_t trx_performed_bitmask;

#if defined(CONFIG_BT_CTLR_LE_ENC)
	uint8_t mic_state;

	/* The Rx in progress receives an encrypted PDU, from the state of the
	 * ACL at its start.
	 */
	uint8_t rx_enc:1;
#endif /* CONFIG_BT_CTLR_LE_ENC */

	/* Payloads that the peripheral did not receive and that are flushed at
	 * the end of the current subevent, after its response.
	 */
	uint8_t rx_flushed;

	/* Close Isochronous Event sent in the current subevent */
	uint8_t cie:1;

	/* The peripheral has received a PDU in the event, valid or not */
	uint8_t is_synced:1;

	/* CISes of the event, those active at its start, and the CIS of the
	 * current subevent.
	 */
	uint8_t cis_count;
	struct cis_evt *curr;
	struct cis_evt cis[CONFIG_BT_CTLR_CONN_ISO_STREAMS_PER_GROUP];

	/* PDU being sent, a copy of the payload as the ULL can release it
	 * while the Tx is pending, and PDU received.
	 */
	uint8_t pdu_tx[CIS_PDU_BUF_SIZE] __aligned(4);
	uint8_t pdu_rx[CIS_PDU_BUF_SIZE] __aligned(4);
} evt;

int lll_conn_iso_init(void)
{
	return 0;
}

void lll_conn_iso_flush(uint16_t handle, struct lll_conn_iso_stream *lll)
{
	/* The PDUs sent are copies, the ULL can release the payloads */
	ARG_UNUSED(handle);
	ARG_UNUSED(lll);
}

static bool is_peripheral(const struct lll_conn_iso_group *cig)
{
	return IS_ENABLED(CONFIG_BT_CTLR_PERIPHERAL_ISO) && (cig->role == BT_HCI_ROLE_PERIPHERAL);
}

/* Last event in which the current payload of a direction can be sent */
static uint64_t flush_event_get(const struct lll_conn_iso_stream_rxtx *d)
{
	return (d->payload_count / d->bn) + d->ft - 1U;
}

/* Subevent at the end of which the current payload of a direction is flushed,
 * in its last event.
 */
static uint8_t flush_se_get(const struct lll_conn_iso_stream *cis,
			    const struct lll_conn_iso_stream_rxtx *d)
{
	uint64_t payload_count = d->payload_count + d->bn_curr - 1U;

	return cis->nse - ((cis->nse / d->bn) * (d->bn - 1U - (payload_count % d->bn)));
}

/* Once the payloads of the burst of the current event are done, a direction
 * waits past the burst until the event is closed, the next burst being for
 * the next event.
 */
static void payload_next(struct lll_conn_iso_stream_rxtx *d, uint64_t event_count)
{
	d->bn_curr++;
	if ((d->bn_curr > d->bn) && ((d->payload_count / d->bn) < event_count)) {
		d->payload_count += d->bn;
		d->bn_curr = 1U;
	}
}

/* A payload is sent until it is acknowledged or its flush point has passed,
 * which is before the current event or at the end of subevent se of it, and
 * both sides then move on to the next payload as if it had been acknowledged.
 * Returns the number of payloads flushed, by which the sequence number moves.
 */
static uint8_t payload_flush(const struct lll_conn_iso_stream *cis,
			     struct lll_conn_iso_stream_rxtx *d, uint8_t se)
{
	uint8_t count = 0U;

	if (d->bn == 0U) {
		return 0U;
	}

	while (d->bn_curr <= d->bn) {
		uint64_t flush_event = flush_event_get(d);

		if ((flush_event > cis->event_count) ||
		    ((flush_event == cis->event_count) && (flush_se_get(cis, d) > se))) {
			break;
		}

		count++;
		payload_next(d, cis->event_count);
	}

	return count;
}

static void tx_release(struct lll_conn_iso_stream *cis)
{
	uint64_t payload_count;
	struct node_tx_iso *tx;
	memq_link_t *link;

	payload_count = cis->tx.payload_count + cis->tx.bn_curr - 1U;
	while (((link = memq_peek(cis->memq_tx.head, cis->memq_tx.tail, (void **)&tx)) != NULL) &&
	       (tx->payload_count < payload_count)) {
		(void)memq_dequeue(cis->memq_tx.tail, &cis->memq_tx.head, NULL);

		tx->next = link;
		ull_iso_lll_ack_enqueue(cis->handle, tx);
	}
}

static bool is_burst_done(const struct lll_conn_iso_stream_rxtx *d)
{
	return d->bn_curr > d->bn;
}

/* A CIS that the ULL has torn down since the start of the event */
static bool is_gone(const struct cis_evt *c)
{
	return c->lll->active == 0U;
}

/* The payloads that the event was the last one for are flushed, and a burst
 * that is done moves on to the next one.
 */
static uint8_t payload_close(const struct lll_conn_iso_stream *cis,
			     struct lll_conn_iso_stream_rxtx *d)
{
	uint8_t count;

	if (d->bn == 0U) {
		return 0U;
	}

	count = payload_flush(cis, d, cis->nse + 1U);
	if (d->bn_curr > d->bn) {
		d->payload_count += d->bn;
		d->bn_curr = 1U;
	}

	return count;
}

static void cis_close(struct cis_evt *c)
{
	struct lll_conn_iso_stream *cis = c->lll;

	c->is_closed = 1U;

	if (is_gone(c)) {
		return;
	}

	cis->sn += payload_close(cis, &cis->tx);
	cis->nesn += payload_close(cis, &cis->rx);

	tx_release(cis);
}

/* Offset of the current subevent of a CIS from the CIG reference point */
static uint32_t se_offset_get(const struct cis_evt *c)
{
	return c->lll->offset + ((c->se - 1U) * c->lll->sub_interval);
}

/* Payload of a CIS with the cisPayloadCounter payload_count, NULL if it has
 * not been provided.
 */
static struct node_tx_iso *payload_get(const struct lll_conn_iso_stream *cis,
				       uint64_t payload_count)
{
	struct node_tx_iso *tx;
	memq_link_t *link;

	link = cis->memq_tx.head;
	while (((link = memq_peek(link, cis->memq_tx.tail, (void **)&tx)) != NULL)) {
		if (tx->payload_count >= payload_count) {
			return (tx->payload_count == payload_count) ? tx : NULL;
		}

		link = link->next;
	}

	return NULL;
}

/* The current Tx payload, else a null PDU, which closes the event with cie
 * once there is no payload left to send.
 */
static void pdu_tx_prep(struct lll_conn_iso_stream *cis, const struct lll_conn *conn, bool cie)
{
	struct node_tx_iso *tx = NULL;
	uint64_t payload_count = 0U;
	struct pdu_cis *pdu;

	if (!is_burst_done(&cis->tx)) {
		payload_count = cis->tx.payload_count + cis->tx.bn_curr - 1U;
		tx = payload_get(cis, payload_count);
	}

	if (tx == NULL) {
		pdu = (void *)evt.pdu_tx;
		pdu->ll_id = PDU_CIS_LLID_START_CONTINUE;
		pdu->nesn = cis->nesn;
		pdu->sn = 0U; /* Reserved in a null PDU */
		pdu->cie = is_burst_done(&cis->tx) && cie;
		pdu->rfu0 = 0U;
		pdu->npi = 1U;
		pdu->rfu1 = 0U;
		pdu->len = 0U;

		cis->npi = 1U;

		return;
	}

	pdu = (void *)tx->pdu;
	pdu->nesn = cis->nesn;
	pdu->sn = cis->sn;
	pdu->cie = 0U;
	pdu->rfu0 = 0U;
	pdu->npi = 0U;
	pdu->rfu1 = 0U;

	cis->npi = 0U;

#if defined(CONFIG_BT_CTLR_LE_ENC)
	if ((pdu->len != 0U) && (conn->enc_tx != 0U)) {
		cis->tx.ccm.counter = payload_count;
		lll_ccm_encrypt(&cis->tx.ccm, LLL_CCM_HDR_MASK_CIS, pdu, evt.pdu_tx);

		return;
	}
#else /* !CONFIG_BT_CTLR_LE_ENC */
	ARG_UNUSED(conn);
#endif /* !CONFIG_BT_CTLR_LE_ENC */

	(void)memcpy(evt.pdu_tx, pdu, offsetof(struct pdu_cis, payload) + pdu->len);
}

static void se_tx(struct lll_conn_iso_group *cig, const struct lll_conn_iso_stream *cis,
		  uint32_t at, lll_radio_cb_t cb)
{
	evt.cfg.phy = lll_radio_phy(cis->tx.phy);

	lll_radio_tx(&evt.cfg, at, evt.pdu_tx, cb, cig);
}

static void se_rx(struct lll_conn_iso_group *cig, const struct lll_conn_iso_stream *cis,
		  const struct lll_conn *conn, uint32_t start_us, uint32_t window_us,
		  lll_radio_cb_t cb)
{
	/* A PDU longer than the local maximum, which the peer may have been
	 * allowed, is received with a CRC error rather than past the buffers.
	 */
	evt.cfg.phy = lll_radio_phy(cis->rx.phy);
	evt.cfg.max_len = MIN(cis->rx.max_pdu, LL_CIS_OCTETS_RX_MAX);

#if defined(CONFIG_BT_CTLR_LE_ENC)
	evt.rx_enc = conn->enc_rx;
	if (evt.rx_enc != 0U) {
		evt.cfg.max_len += PDU_MIC_SIZE;
	}
#else /* !CONFIG_BT_CTLR_LE_ENC */
	ARG_UNUSED(conn);
#endif /* !CONFIG_BT_CTLR_LE_ENC */

	lll_radio_rx(&evt.cfg, start_us, window_us, evt.pdu_rx, cb, cig);
}

#if defined(CONFIG_TEST_FT_SKIP_SUBEVENTS)
/* Test hook of the flush timeout tests, as in the Nordic LLL: what is received
 * in the first 2 subevents of skip_count events in every 3, and in the first
 * subevent of the event after them, is ignored.
 */
static bool test_ft_skip(const struct cis_evt *c, uint8_t skip_count)
{
	uint8_t n = c->lll->event_count % 3U;

	return ((n < skip_count) && (c->se <= 2U)) ||
	       ((n < (skip_count + 1U)) && (c->se <= 1U));
}
#endif /* CONFIG_TEST_FT_SKIP_SUBEVENTS */

/* Anchor point of a CIS in the event of the burst of its current Rx payload,
 * the timestamp of the payload.
 */
static uint32_t rx_timestamp_get(const struct lll_conn_iso_group *cig,
				 const struct lll_conn_iso_stream *cis, uint32_t ref_us)
{
	uint64_t burst_event = cis->rx.payload_count / cis->rx.bn;

	return ref_us + cis->offset -
	       (uint32_t)((cis->event_count - burst_event) * cig->iso_interval_us);
}

/* Returns false on a MIC failure */
static bool pdu_rx_copy(struct lll_conn_iso_stream *cis, const struct pdu_cis *pdu, void *out)
{
#if defined(CONFIG_BT_CTLR_LE_ENC)
	if ((evt.rx_enc != 0U) && (pdu->len != 0U)) {
		cis->rx.ccm.counter = cis->rx.payload_count + cis->rx.bn_curr - 1U;
		if (!lll_ccm_decrypt(&cis->rx.ccm, LLL_CCM_HDR_MASK_CIS, pdu, out)) {
			evt.mic_state = LLL_CONN_MIC_FAIL;

			return false;
		}

		evt.mic_state = LLL_CONN_MIC_PASS;

		return true;
	}
#else /* !CONFIG_BT_CTLR_LE_ENC */
	ARG_UNUSED(cis);
#endif /* !CONFIG_BT_CTLR_LE_ENC */

	(void)memcpy(out, pdu, offsetof(struct pdu_cis, payload) + pdu->len);

	return true;
}

/* ref_us is the CIG reference point of the event. Returns false on a MIC
 * failure, else sets is_new for a new payload and cie for a Close Isochronous
 * Event of the peer.
 */
static bool rx_pdu(struct lll_conn_iso_group *cig, struct lll_conn_iso_stream *cis,
		   uint32_t ref_us, bool *is_new, bool *cie)
{
	struct pdu_cis *pdu = (void *)evt.pdu_rx;
	struct node_rx_iso_meta *iso_meta;
	struct node_rx_pdu *node_rx;

	*is_new = false;
	*cie = (pdu->cie != 0U);

	/* Tx payload acknowledged */
	if ((pdu->nesn != cis->sn) && !is_burst_done(&cis->tx)) {
		cis->sn++;
		payload_next(&cis->tx, cis->event_count);
	}

	/* New Rx payload, kept with a node rx left free for the next PDU
	 * received by any role, else received again in a later subevent.
	 */
	if ((pdu->npi != 0U) || is_burst_done(&cis->rx) || (pdu->sn != cis->nesn) ||
	    (ull_iso_pdu_rx_alloc_peek(2U) == NULL)) {
		return true;
	}

	cis->nesn++;

	node_rx = ull_iso_pdu_rx_alloc_peek(1U);
	if (!pdu_rx_copy(cis, pdu, node_rx->pdu)) {
		return false;
	}

	(void)ull_iso_pdu_rx_alloc();

	node_rx->hdr.type = NODE_RX_TYPE_ISO_PDU;
	node_rx->hdr.handle = cis->handle;

	iso_meta = &node_rx->rx_iso_meta;
	iso_meta->payload_number = cis->rx.payload_count + cis->rx.bn_curr - 1U;
	iso_meta->timestamp = rx_timestamp_get(cig, cis, ref_us);
	iso_meta->status = 0U;

	iso_rx_put(node_rx->hdr.link, node_rx);
	iso_rx_sched();

	payload_next(&cis->rx, cis->event_count);

	*is_new = true;

	return true;
}

/* A valid PDU of the current subevent has been received, which establishes
 * the CIS and restarts its supervision timeout.
 */
static void trx_performed(void)
{
	struct cis_evt *c = evt.curr;

	evt.trx_performed_bitmask |= BIT(LL_CIS_IDX_FROM_HANDLE(c->lll->handle));

	ull_conn_iso_lll_cis_established(c->lll);
}

static void event_done(struct lll_conn_iso_group *cig)
{
	struct event_done_extra *e;

	for (uint8_t i = 0U; i < evt.cis_count; i++) {
		struct cis_evt *c = &evt.cis[i];

		if (c->is_closed == 0U) {
			cis_close(c);
		}
	}

	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_CIS;
	e->trx_performed_bitmask = evt.trx_performed_bitmask;

#if defined(CONFIG_BT_CTLR_LE_ENC)
	e->mic_state = evt.mic_state;
#endif /* CONFIG_BT_CTLR_LE_ENC */

#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
	if (is_peripheral(cig) && (evt.trx_performed_bitmask != 0U)) {
		e->drift.start_to_address_actual_us = evt.ref_us + evt.ref_addr_us - evt.start_us;
		e->drift.window_widening_event_us =
			EVENT_US_FRAC_TO_US(cig->window_widening_event_us_frac);
		e->drift.preamble_to_addr_us = evt.ref_addr_us;

		/* Synchronized again on the anchor point received */
		cig->window_widening_event_us_frac = 0U;
	}
#endif /* CONFIG_BT_CTLR_PERIPHERAL_ISO */

	lll_isr_cleanup(cig);
}

/* Returns false if the event has ended, as there is no next subevent */
static bool se_next_or_done(struct lll_conn_iso_group *cig)
{
	if (se_next(cig)) {
		return true;
	}

	event_done(cig);

	return false;
}

static void isr_rx_central(const struct bsr_evt *e, void *param)
{
	struct lll_conn_iso_group *cig = param;
	struct cis_evt *c = evt.curr;
	struct lll_conn_iso_stream *cis = c->lll;
	bool is_new = false;
	bool cie = false;
	bool is_aa;

	if (is_gone(c)) {
		c->is_closed = 1U;
		(void)se_next_or_done(cig);

		return;
	}

	is_aa = (e->status == BSR_STATUS_OK) || (e->status == BSR_STATUS_CRC_ERR);
#if defined(CONFIG_TEST_FT_CEN_SKIP_SUBEVENTS)
	is_aa = is_aa && !test_ft_skip(c, CONFIG_TEST_FT_CEN_SKIP_EVENTS_COUNT);
#endif /* CONFIG_TEST_FT_CEN_SKIP_SUBEVENTS */
	if (is_aa && (e->status == BSR_STATUS_OK)) {
		trx_performed();

		if (!rx_pdu(cig, cis, evt.start_us, &is_new, &cie)) {
			/* MIC failure, the CIS is terminated */
			event_done(cig);

			return;
		}
	}

	/* Payloads at their flush point at the end of the subevent */
	cis->sn += payload_flush(cis, &cis->tx, c->se);
	cis->nesn += payload_flush(cis, &cis->rx, c->se);

	tx_release(cis);

	/* Close the CIS once a Close Isochronous Event has been sent or
	 * received, or early once neither side has a payload left to send and
	 * a new payload received has been acknowledged.
	 */
	cie = cie || (evt.cie != 0U) ||
	      (is_aa && is_burst_done(&cis->rx) && is_burst_done(&cis->tx) && !is_new);
	if (cie || (c->se >= cis->nse)) {
		cis_close(c);
	}

	(void)se_next_or_done(cig);
}

static void isr_tx_central(const struct bsr_evt *e, void *param)
{
	struct lll_conn_iso_group *cig = param;
	struct cis_evt *c = evt.curr;
	struct lll_conn_iso_stream *cis = c->lll;

	if ((e->status != BSR_STATUS_OK) || is_gone(c)) {
		/* Not sent, too late to start it, or the CIS is torn down */
		evt.cie = 0U;
		isr_rx_central(e, param);

		return;
	}

	se_rx(cig, cis, ull_conn_lll_get(cis->acl_handle),
	      lll_radio_tifs_rx_start(e->ts_end, cis->tifs_us),
	      lll_radio_tifs_rx_window(cis->rx.phy), isr_rx_central);
}

static void se_start_central(struct lll_conn_iso_group *cig, struct cis_evt *c,
			     const struct lll_conn *conn)
{
	struct lll_conn_iso_stream *cis = c->lll;

	/* A null PDU closes the event once both directions are done */
	pdu_tx_prep(cis, conn, is_burst_done(&cis->rx));
	evt.cie = ((const struct pdu_cis *)evt.pdu_tx)->cie;

	se_tx(cig, cis, evt.start_us + se_offset_get(c), isr_tx_central);
}

static void isr_tx_peripheral(const struct bsr_evt *e, void *param)
{
	struct lll_conn_iso_group *cig = param;
	struct cis_evt *c = evt.curr;

	ARG_UNUSED(e);

	if (is_gone(c)) {
		c->is_closed = 1U;
	} else {
		c->lll->nesn += evt.rx_flushed;

		if ((evt.cie != 0U) || (c->se >= c->lll->nse)) {
			cis_close(c);
		}
	}

	evt.rx_flushed = 0U;

	(void)se_next_or_done(cig);
}

static void isr_rx_peripheral(const struct bsr_evt *e, void *param)
{
	struct lll_conn_iso_group *cig = param;
	struct cis_evt *c = evt.curr;
	struct lll_conn_iso_stream *cis = c->lll;
	bool is_new;
	bool is_aa;
	bool cie;

	if (is_gone(c)) {
		c->is_closed = 1U;
		(void)se_next_or_done(cig);

		return;
	}

	is_aa = (e->status == BSR_STATUS_OK) || (e->status == BSR_STATUS_CRC_ERR);
#if defined(CONFIG_TEST_FT_PER_SKIP_SUBEVENTS)
	is_aa = is_aa && !test_ft_skip(c, CONFIG_TEST_FT_PER_SKIP_EVENTS_COUNT);
#endif /* CONFIG_TEST_FT_PER_SKIP_SUBEVENTS */
	if (!is_aa) {
		/* No response without the PDU of the central: its payloads
		 * at their flush point at the end of the subevent, and the Tx
		 * payloads at theirs at the end of an earlier one, are done.
		 */
		cis->sn += payload_flush(cis, &cis->tx, c->se - 1U);
		cis->nesn += payload_flush(cis, &cis->rx, c->se);

		tx_release(cis);

		if (c->se >= cis->nse) {
			cis_close(c);
		}

		(void)se_next_or_done(cig);

		return;
	}

	if (evt.is_synced == 0U) {
		/* The first PDU received gives the anchor point */
		evt.ref_addr_us = addr_us_get(cis->rx.phy);
		evt.ref_us = e->ts_aa_end - evt.ref_addr_us - se_offset_get(c);
		evt.is_synced = 1U;
	}

	/* The last PDU received times the next subevents */
	evt.last_start_us = e->ts_aa_end - addr_us_get(cis->rx.phy);
	evt.last_offset_us = se_offset_get(c);
	evt.last_idx = evt.idx;

	cie = false;
	if (e->status == BSR_STATUS_OK) {
		trx_performed();

		if (!rx_pdu(cig, cis, evt.ref_us, &is_new, &cie)) {
			/* MIC failure, the CIS is terminated */
			event_done(cig);

			return;
		}
	}

	/* Tx payloads at their flush point at the end of an earlier subevent,
	 * the Tx payload of the subevent is still sent. The Rx payloads at
	 * theirs at the end of the subevent are flushed after the response,
	 * which acknowledges what has been received.
	 */
	cis->sn += payload_flush(cis, &cis->tx, c->se - 1U);
	evt.rx_flushed = payload_flush(cis, &cis->rx, c->se);

	tx_release(cis);

	/* Close the CIS after the response once neither side has a payload
	 * left to send in the event.
	 */
	evt.cie = cie || (is_burst_done(&cis->rx) && is_burst_done(&cis->tx) &&
			  (c->se < cis->nse));

	pdu_tx_prep(cis, ull_conn_lll_get(cis->acl_handle), evt.cie != 0U);

	se_tx(cig, cis, e->ts_end + cis->tifs_us, isr_tx_peripheral);
}

static void se_start_peripheral(struct lll_conn_iso_group *cig, struct cis_evt *c,
				const struct lll_conn *conn)
{
	struct lll_conn_iso_stream *cis = c->lll;
	uint32_t offset_us = se_offset_get(c);
	uint32_t addr_us = addr_us_get(cis->rx.phy);
	uint32_t window_us;
	uint32_t start_us;

	/* Until a PDU is received in the event, the window is widened by the
	 * drift of the sleep clocks since the last anchor point received.
	 */
	if (evt.is_synced != 0U) {
		uint32_t elapsed_us = offset_us - evt.last_offset_us;
		uint32_t jitter_us;

		jitter_us = lll_radio_se_jitter_get(evt.idx - evt.last_idx, elapsed_us);
		start_us = evt.last_start_us + elapsed_us - jitter_us;
		window_us = (jitter_us << 1) + RANGE_DELAY_US + addr_us;
	} else {
		start_us = evt.start_us + offset_us;
		window_us = evt.window_us + addr_us;
	}

	se_rx(cig, cis, conn, start_us, window_us, isr_rx_peripheral);
}

/* Start the next subevent in time of the CISes that are not closed. Returns
 * false if there is none.
 */
static bool se_next(struct lll_conn_iso_group *cig)
{
	const struct lll_conn *conn;
	struct lll_conn_iso_stream *cis;
	struct cis_evt *next = NULL;
	uint32_t next_us = 0U;

	/* The subevents run in the order of their offsets from the CIG
	 * reference point, which the same formula gives in the sequential and
	 * interleaved arrangements.
	 */
	for (uint8_t i = 0U; i < evt.cis_count; i++) {
		struct cis_evt *c = &evt.cis[i];
		uint32_t offset_us;

		if (c->is_closed != 0U) {
			continue;
		}

		if (is_gone(c)) {
			c->is_closed = 1U;

			continue;
		}

		offset_us = c->lll->offset + (c->se * c->lll->sub_interval);
		if ((next == NULL) || (offset_us < next_us)) {
			next = c;
			next_us = offset_us;
		}
	}

	if (next == NULL) {
		return false;
	}

	evt.curr = next;
	evt.idx++;
	next->se++;

	cis = next->lll;
	conn = ull_conn_lll_get(cis->acl_handle);
	LL_ASSERT_DBG(conn != NULL);

	evt.cfg.aa = sys_get_le32(cis->access_addr);
	evt.cfg.crc_init = sys_get_le24(conn->crc_init);
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = conn->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	/* The first subevent hops with the event counter, the next ones from
	 * it. A CIS uses the channel map of its ACL.
	 */
	if (next->se == 1U) {
		next->chan_id = lll_chan_id(cis->access_addr);
		evt.cfg.chan = lll_chan_iso_event((uint16_t)cis->event_count, next->chan_id,
						  conn->data_chan_map, conn->data_chan_count,
						  &next->prn_s, &next->remap_idx);
	} else {
		evt.cfg.chan = lll_chan_iso_subevent(next->chan_id, conn->data_chan_map,
						     conn->data_chan_count, &next->prn_s,
						     &next->remap_idx);
	}

	if (is_peripheral(cig)) {
		se_start_peripheral(cig, next, conn);
	} else {
		se_start_central(cig, next, conn);
	}

	return true;
}

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	/* The Rx payloads flushed after a response that was not sent */
	if ((evt.rx_flushed != 0U) && !is_gone(evt.curr)) {
		evt.curr->lll->nesn += evt.rx_flushed;
	}

	evt.rx_flushed = 0U;

	event_done(param);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_conn_iso_group *cig = p->param;
	struct lll_conn_iso_stream *cis;
	uint32_t ticks_ref;
	uint16_t handle;
	int err;

	cig->lazy_prepare = p->lazy;
	cig->latency_event = cig->latency_prepare + cig->lazy_prepare;
	cig->latency_prepare = 0U;

#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
	if (is_peripheral(cig)) {
		cig->window_widening_prepare_us_frac += cig->window_widening_periodic_us_frac *
							(cig->lazy_prepare + 1U);
		if (cig->window_widening_prepare_us_frac >
		    EVENT_US_TO_US_FRAC(cig->window_widening_max_us)) {
			cig->window_widening_prepare_us_frac =
				EVENT_US_TO_US_FRAC(cig->window_widening_max_us);
		}

		cig->window_widening_event_us_frac += cig->window_widening_prepare_us_frac;
		cig->window_widening_prepare_us_frac = 0U;
		if (cig->window_widening_event_us_frac >
		    EVENT_US_TO_US_FRAC(cig->window_widening_max_us)) {
			cig->window_widening_event_us_frac =
				EVENT_US_TO_US_FRAC(cig->window_widening_max_us);
		}
	}
#endif /* CONFIG_BT_CTLR_PERIPHERAL_ISO */

	evt.trx_performed_bitmask = 0U;
#if defined(CONFIG_BT_CTLR_LE_ENC)
	evt.mic_state = LLL_CONN_MIC_NONE;
#endif /* CONFIG_BT_CTLR_LE_ENC */
	evt.is_synced = 0U;
	evt.idx = 0U;
	evt.curr = NULL;

	/* The CISes active at the start of the event take part in it. One
	 * that becomes active during the event, at the instant of its ACL,
	 * starts in the next one.
	 */
	evt.cis_count = 0U;
	handle = UINT16_MAX;
	while (((cis = ull_conn_iso_lll_stream_get_by_group(cig, &handle)) != NULL)) {
		struct cis_evt *c;

		if (cis->active == 0U) {
			continue;
		}

		LL_ASSERT_ERR(evt.cis_count < ARRAY_SIZE(evt.cis));

		c = &evt.cis[evt.cis_count++];
		c->lll = cis;
		c->se = 0U;
		c->is_closed = 0U;

#if !defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
		cis->prepared = 1U;
#endif /* !CONFIG_BT_CTLR_JIT_SCHEDULING */

		cis->event_count = cis->event_count_prepare;

		/* The payloads of the events missed are past their flush point */
		cis->sn += payload_flush(cis, &cis->tx, 0U);
		cis->nesn += payload_flush(cis, &cis->rx, 0U);

		tx_release(cis);
	}

	if ((evt.cis_count == 0U) || (lll_preempt_calc(p) != 0U)) {
		/* The event is done, without any subevent */
		lll_radio_stop(isr_abort, cig);

		return -ECANCELED;
	}

	evt.start_us = lll_event_start_get(p, &ticks_ref);

#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
	if (is_peripheral(cig)) {
		/* The ULL starts the event the jitter, ticker resolution margin
		 * and window widening before the earliest expected CIG
		 * reference point. Listen until the latest one, plus a ticker
		 * resolution margin for a shift of the anchor point of the ACL
		 * at the instant that established a CIS.
		 */
		evt.window_us = ((EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
				  EVENT_US_FRAC_TO_US(cig->window_widening_event_us_frac)) << 1) +
				EVENT_TICKER_RES_MARGIN_US;
	}
#endif /* CONFIG_BT_CTLR_PERIPHERAL_ISO */

	(void)se_next(cig);

	err = lll_prepare_done(cig);
	LL_ASSERT_ERR(err == 0);

	return 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct lll_conn_iso_group *cig;
	int err;

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_abort, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* The event counters and window widening of the next event account
	 * for the events aborted in the prepare pipeline.
	 */
	cig = prepare_param->param;
	cig->lazy_prepare = prepare_param->lazy;
	cig->latency_prepare += (cig->lazy_prepare + 1U);

#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
	if (is_peripheral(cig)) {
		cig->window_widening_prepare_us_frac += cig->window_widening_periodic_us_frac *
							(cig->lazy_prepare + 1U);
		if (cig->window_widening_prepare_us_frac >
		    EVENT_US_TO_US_FRAC(cig->window_widening_max_us)) {
			cig->window_widening_prepare_us_frac =
				EVENT_US_TO_US_FRAC(cig->window_widening_max_us);
		}
	}
#endif /* CONFIG_BT_CTLR_PERIPHERAL_ISO */

	lll_done(param);
}

static void prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	/* A CIG event is not resumed, and ends for any event that overlaps */
	err = lll_prepare(lll_is_abort_cb, abort_cb, prepare_cb, 0U, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

void lll_central_iso_prepare(void *param)
{
	prepare(param);
}

void lll_peripheral_iso_prepare(void *param)
{
	prepare(param);
}
