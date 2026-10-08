/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Synchronized Receiver events of a Broadcast Isochronous Group (BIG) of the
 * BabbleSim LLL.
 *
 * A BIG event listens, in the order of the subevents in time (see
 * lll_adv_iso.c), for the subevents of the selected BISes that carry a
 * payload not received yet: the repetitions of a payload, and its
 * pre-transmissions in earlier events, are only listened for until one of
 * them has been received. The payloads received are kept in a sliding window
 * (lll_sync_iso.payload) and handed to the ULL in order once their event has
 * passed, as invalid payloads when they have not been received. The control
 * subevent is listened for when a BIS PDU has announced a new BIG control
 * PDU.
 *
 * Until a PDU has been received in the event, a subevent is listened for in
 * a window widened by the drift of the sleep clocks since the last anchor
 * point received, then around the time that the last PDU received gives,
 * for the active clock jitter of each subevent since that PDU.
 *
 * The event that establishes the synchronization only listens for the first
 * subevent of the first selected BIS.
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

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_chan.h"
#include "lll_sync_iso.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_ccm.h"

#include "ll_feat.h"

#include "hal/debug.h"

static int create_prepare_cb(struct lll_prepare_param *p);
static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_rx(const struct bsr_evt *e, void *param);
static void isr_abort(const struct bsr_evt *e, void *param);
static void isr_done(struct lll_sync_iso *lll);

/* Channel selection of a BIS, along the subevents of an event */
struct bis_chan {
	uint16_t id;
	uint16_t prn_s;
	uint16_t remap_idx;
};

/* Subevent window, around its expected access address, once a PDU has been
 * received in the event: the active clock jitter for each subevent since the
 * last PDU received, as transmitters that time each subevent from the end of
 * the previous one can drift a little at each, up to half the minimum gap
 * between subevents.
 */
#define SE_JITTER_US     (EVENT_CLOCK_JITTER_US << 1)
#define SE_JITTER_MAX_US (EVENT_IFS_US >> 1)

/* State of the current BIG event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* Start of the receive window of the first selected subevent, that is
	 * of the event, and window of a subevent listened for before any PDU
	 * has been received in the event.
	 */
	uint32_t rx_start;
	uint32_t window_us;

	/* End of the access address of the first selected subevent, given by
	 * the first PDU received in the event.
	 */
	uint32_t aa_end;

	/* End of the access address of the last PDU received in the event, the
	 * offset of its subevent, and the position of its subevent in time
	 * among all the subevents of the BIG.
	 */
	uint32_t last_aa_end;
	uint32_t last_offset_us;
	uint16_t last_idx;

	/* Offset from the BIG anchor point that the ULL gave the event start,
	 * the one of the first selected BIS.
	 */
	uint32_t first_us;

	/* bisPayloadCounter of the first payload of the event */
	uint64_t payload_count;

	uint16_t event_counter;

	/* Selected BISes, the first ones of lll_sync_iso.stream_handle, in
	 * increasing BIS number order, that the BIG has.
	 */
	uint8_t stream_count;

	/* Current subevent: selected BIS, its BIS number, 0 for the control
	 * subevent, and index of the subevent in the BIS.
	 */
	uint8_t stream;
	uint8_t bis;
	uint8_t se;

	/* Access address of the current subevent */
	uint8_t access_addr[4];

	/* PDUs received, and with a valid CRC */
	uint8_t trx_cnt;
	uint8_t crc_ok;

	/* Event establishing the synchronization */
	uint8_t is_create:1;

	struct bis_chan chan[BT_CTLR_SYNC_ISO_STREAM_MAX];

	/* BIG control PDU received */
	uint8_t pdu_ctrl[offsetof(struct pdu_bis, payload) + sizeof(struct pdu_big_ctrl)];

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	/* Encrypted PDU received */
	uint8_t pdu_enc[offsetof(struct pdu_bis, payload) +
			MAX(LL_BIS_OCTETS_RX_MAX, sizeof(struct pdu_big_ctrl)) + PDU_MIC_SIZE];
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */
} evt;

int lll_sync_iso_init(void)
{
	return 0;
}

int lll_sync_iso_reset(void)
{
	return 0;
}

void lll_sync_iso_create_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, create_prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

void lll_sync_iso_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

void lll_sync_iso_flush(uint8_t handle, struct lll_sync_iso *lll)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(lll);
}

static bool is_sequential(const struct lll_sync_iso *lll)
{
	return lll->bis_spacing >= (lll->sub_interval * lll->nse);
}

static struct lll_sync_iso_stream *stream_get(const struct lll_sync_iso *lll, uint8_t stream)
{
	return ull_sync_iso_lll_stream_get(lll->stream_handle[stream]);
}

/* Move to the next subevent in time of the selected BISes, returns false
 * after the last one.
 */
static bool se_next(const struct lll_sync_iso *lll)
{
	if (is_sequential(lll)) {
		if (++evt.se < lll->nse) {
			return true;
		}

		evt.se = 0U;
		if (++evt.stream >= evt.stream_count) {
			return false;
		}
	} else if (++evt.stream >= evt.stream_count) {
		evt.stream = 0U;
		if (++evt.se >= lll->nse) {
			return false;
		}
	}

	evt.bis = stream_get(lll, evt.stream)->bis_index;

	return true;
}

/* Offset from the first payload of the event of the payload sent in
 * subevent se of a BIS.
 */
static uint16_t payload_offset(const struct lll_sync_iso *lll, uint8_t se)
{
	uint8_t group = se / lll->bn;
	uint8_t n = se % lll->bn;

	if (group < lll->irc) {
		return n;
	}

	/* Pre-transmission of a payload of a later event */
	return ((group - lll->irc + 1U) * lll->pto * lll->bn) + n;
}

/* Slot of the payload of the current subevent in the sliding window, NULL if
 * it is beyond the window. The window starts at the payloads of the oldest
 * event not handed to the ULL yet, latency_event events ago.
 */
static struct node_rx_pdu **payload_slot_get(struct lll_sync_iso *lll)
{
	uint32_t offset;
	uint32_t idx;

	offset = (lll->latency_event * lll->bn) + payload_offset(lll, evt.se);
	if (offset >= lll->payload_count_max) {
		return NULL;
	}

	idx = lll->payload_tail + offset;
	if (idx >= lll->payload_count_max) {
		idx -= lll->payload_count_max;
	}

	return &lll->payload[evt.stream][idx];
}

/* Offset of the current subevent from the first selected subevent */
static uint32_t se_offset_get(const struct lll_sync_iso *lll)
{
	uint32_t offset;

	if (evt.bis) {
		offset = ((evt.bis - 1U) * lll->bis_spacing) + (evt.se * lll->sub_interval);
	} else {
		/* The control subevent follows the last subevent of the BIG */
		offset = ((lll->num_bis - 1U) * lll->bis_spacing) +
			 ((lll->nse - 1U) * lll->sub_interval) +
			 (is_sequential(lll) ? lll->sub_interval : lll->bis_spacing);
	}

	return offset - evt.first_us;
}

/* Position in time of the current subevent among all the subevents of the
 * BIG.
 */
static uint16_t se_idx_get(const struct lll_sync_iso *lll)
{
	if (!evt.bis) {
		return lll->num_bis * lll->nse;
	}

	if (is_sequential(lll)) {
		return ((evt.bis - 1U) * lll->nse) + evt.se;
	}

	return (evt.se * lll->num_bis) + (evt.bis - 1U);
}

/* Channel of the current BIS subevent. The channel selection of a BIS moves
 * along its subevents, whether they are listened for or not.
 */
static void bis_chan_calc(const struct lll_sync_iso *lll)
{
	struct bis_chan *chan = &evt.chan[evt.stream];

	if (!evt.se) {
		uint8_t access_addr[4];

		util_bis_aa_le32(evt.bis, (uint8_t *)lll->seed_access_addr, access_addr);
		chan->id = lll_chan_id(access_addr);
		evt.cfg.chan = lll_chan_iso_event(evt.event_counter, chan->id,
						  lll->data_chan_map, lll->data_chan_count,
						  &chan->prn_s, &chan->remap_idx);
	} else {
		evt.cfg.chan = lll_chan_iso_subevent(chan->id, lll->data_chan_map,
						     lll->data_chan_count, &chan->prn_s,
						     &chan->remap_idx);
	}
}

/* Listen for the current subevent */
static void se_rx(struct lll_sync_iso *lll)
{
	uint8_t crc_init[3];
	uint32_t offset_us;
	uint32_t window_us;
	uint32_t start_us;
	void *buf;

	/* Access address and CRC initial value of the BIS, the BIS number
	 * being the least significant octet of the latter.
	 */
	util_bis_aa_le32(evt.bis, lll->seed_access_addr, evt.access_addr);
	crc_init[0] = evt.bis;
	(void)memcpy(&crc_init[1], lll->base_crc_init, sizeof(lll->base_crc_init));
	evt.cfg.aa = sys_get_le32(evt.access_addr);
	evt.cfg.crc_init = sys_get_le24(crc_init);

	if (evt.bis) {
		struct node_rx_pdu *node_rx;

		/* By design, there is always a free node rx to receive in */
		node_rx = ull_iso_pdu_rx_alloc_peek(1U);
		LL_ASSERT_DBG(node_rx);

		buf = node_rx->pdu;
		evt.cfg.max_len = MIN(lll->max_pdu, LL_BIS_OCTETS_RX_MAX);
	} else {
		uint16_t prn_s;
		uint16_t remap_idx;

		/* The control subevent hops as the first subevent of a BIS */
		evt.cfg.chan = lll_chan_iso_event(evt.event_counter,
						  lll_chan_id(evt.access_addr),
						  lll->data_chan_map, lll->data_chan_count,
						  &prn_s, &remap_idx);

		buf = evt.pdu_ctrl;
		evt.cfg.max_len = sizeof(struct pdu_big_ctrl);
	}

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc) {
		buf = evt.pdu_enc;
		evt.cfg.max_len += PDU_MIC_SIZE;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	offset_us = se_offset_get(lll);
	if (evt.trx_cnt) {
		uint32_t jitter_us;

		/* Around the time that the last PDU received gives */
		jitter_us = MIN(SE_JITTER_US * (se_idx_get(lll) - evt.last_idx),
				SE_JITTER_MAX_US);
		start_us = evt.last_aa_end + (offset_us - evt.last_offset_us) -
			   addr_us_get(lll->phy) - jitter_us;
		window_us = (jitter_us << 1) + RANGE_DELAY_US + addr_us_get(lll->phy);
	} else {
		start_us = evt.rx_start + offset_us;
		window_us = evt.window_us;
	}

	lll_radio_rx(&evt.cfg, start_us, window_us, buf, isr_rx, lll);
}

/* Listen for the next subevent in time, from the current one if from_curr,
 * that carries a payload not received yet, or else for the control subevent
 * if a new BIG control PDU was announced. Returns false if there is none.
 */
static bool rx_next(struct lll_sync_iso *lll, bool from_curr)
{
	if (!evt.bis) {
		/* The control subevent is the last one */
		return false;
	}

	if (from_curr || se_next(lll)) {
		do {
			struct node_rx_pdu **slot;

			bis_chan_calc(lll);

			if (evt.is_create) {
				/* Only the first subevent */
				se_rx(lll);

				return true;
			}

			slot = payload_slot_get(lll);
			if (slot && !*slot) {
				se_rx(lll);

				return true;
			}
		} while (se_next(lll));
	}

	if (!evt.is_create && (lll->cssn_next != lll->cssn_curr)) {
		evt.bis = 0U;
		se_rx(lll);

		return true;
	}

	return false;
}

static int prepare_cb_common(struct lll_prepare_param *p, bool is_create)
{
	struct lll_sync_iso *lll = p->param;
	struct lll_sync_iso_stream *stream;
	uint32_t ticks_ref;
	int err;

	DEBUG_RADIO_START_O(1);

	/* Calculate the current event latency */
	lll->lazy_prepare = p->lazy;
	lll->latency_event = lll->latency_prepare + lll->lazy_prepare;

	/* Calculate the current event counter value */
	evt.event_counter = (lll->payload_count / lll->bn) + lll->latency_event;

	/* Update BIS payload counter to next value */
	lll->payload_count += (lll->latency_event + 1U) * lll->bn;
	evt.payload_count = lll->payload_count - lll->bn;

	/* Reset accumulated latencies */
	lll->latency_prepare = 0U;

	/* Accumulate window widening */
	lll->window_widening_prepare_us += lll->window_widening_periodic_us *
					   (lll->lazy_prepare + 1U);
	if (lll->window_widening_prepare_us > lll->window_widening_max_us) {
		lll->window_widening_prepare_us = lll->window_widening_max_us;
	}

	/* Current window widening */
	lll->window_widening_event_us += lll->window_widening_prepare_us;
	lll->window_widening_prepare_us = 0U;
	if (lll->window_widening_event_us > lll->window_widening_max_us) {
		lll->window_widening_event_us = lll->window_widening_max_us;
	}

	evt.is_create = is_create;
	evt.trx_cnt = 0U;
	evt.crc_ok = 0U;

	/* The selected BISes that the BIG has */
	evt.stream_count = 0U;
	while ((evt.stream_count < lll->stream_count) &&
	       (stream_get(lll, evt.stream_count)->bis_index <= lll->num_bis)) {
		evt.stream_count++;
	}

	if (!evt.stream_count || lll_preempt_calc(p)) {
		/* The event is done, as an event without PDUs received */
		lll_radio_stop(isr_abort, lll);

		return -ECANCELED;
	}

	evt.cfg.phy = lll_radio_phy(lll->phy);
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;

	/* The ULL starts the event the jitter, ticker resolution margin and
	 * window widening before the earliest expected first subevent of the
	 * first selected BIS, which it places as if each BIS took NSE
	 * subevents in the sequential arrangement. Listen until the latest
	 * one, plus the window of the offset unit of the BIGInfo in the first
	 * event.
	 */
	stream = stream_get(lll, 0U);
	if (is_sequential(lll)) {
		evt.first_us = (stream->bis_index - 1U) * lll->sub_interval *
			       ((lll->irc * lll->bn) + lll->ptc);
	} else {
		evt.first_us = (stream->bis_index - 1U) * lll->bis_spacing;
	}

	evt.rx_start = lll_event_start_get(p, &ticks_ref);
	evt.window_us = ((EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
			  lll->window_widening_event_us) << 1) +
			lll->window_size_event_us + addr_us_get(lll->phy);

	evt.stream = 0U;
	evt.bis = stream->bis_index;
	evt.se = 0U;
	if (!rx_next(lll, true)) {
		/* All the payloads of the event were pre-transmitted and
		 * received in earlier events.
		 */
		lll_radio_stop(isr_abort, lll);
	}

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_O(1);

	return 0;
}

static int create_prepare_cb(struct lll_prepare_param *p)
{
	return prepare_cb_common(p, true);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	return prepare_cb_common(p, false);
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	/* A BIG event is not resumed */
	ARG_UNUSED(resume_cb);

	/* The next event of the same BIG aborts the current one, the events
	 * of others wait for its end.
	 */
	return (next == curr) ? -ECANCELED : 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct event_done_extra *e;
	struct lll_sync_iso *lll;
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* The event is done with what it has received, once the radio
		 * has been stopped.
		 */
		lll_radio_stop(isr_abort, param);

		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* Accumulate the latency as event is aborted while being in pipeline */
	lll = prepare_param->param;
	lll->lazy_prepare = prepare_param->lazy;
	lll->latency_prepare += (lll->lazy_prepare + 1U);

	/* Accumulate window widening */
	lll->window_widening_prepare_us += lll->window_widening_periodic_us *
					   (prepare_param->lazy + 1U);
	if (lll->window_widening_prepare_us > lll->window_widening_max_us) {
		lll->window_widening_prepare_us = lll->window_widening_max_us;
	}

	/* Extra done event, to check sync lost */
	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e);

	e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO;
	e->estab_failed = 0U;
	e->trx_cnt = 0U;
	e->crc_valid = 0U;

	lll_done(param);
}

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
/* Decrypt the PDU received into out, returns false on a MIC failure */
static bool pdu_decrypt(struct lll_sync_iso *lll, uint64_t payload_count, void *out)
{
	lll->ccm_rx.counter = payload_count;
	(void)memcpy(lll->ccm_rx.iv, lll->giv, 4U);
	mem_xor_32(lll->ccm_rx.iv, lll->ccm_rx.iv, evt.access_addr);

	return lll_ccm_decrypt(&lll->ccm_rx, LLL_CCM_HDR_MASK_BIS, evt.pdu_enc, out);
}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

/* BIG control PDU received */
static void ctrl_rx(struct lll_sync_iso *lll)
{
	struct pdu_bis *pdu = (void *)evt.pdu_ctrl;

	lll->cssn_curr = lll->cssn_next;

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc && !pdu_decrypt(lll, evt.payload_count, pdu)) {
		lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;

		return;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	if (pdu->ll_id != PDU_BIS_LLID_CTRL) {
		return;
	}

	if (pdu->ctrl.opcode == PDU_BIG_CTRL_TYPE_TERM_IND) {
		if (!lll->term_reason) {
			struct pdu_big_ctrl_term_ind *term = &pdu->ctrl.term_ind;

			lll->term_reason = term->reason;
			lll->ctrl_instant = sys_le16_to_cpu(term->instant);
		}
	} else if (pdu->ctrl.opcode == PDU_BIG_CTRL_TYPE_CHAN_MAP_IND) {
		if (!lll->chm_chan_count) {
			struct pdu_big_ctrl_chan_map_ind *chm = &pdu->ctrl.chan_map_ind;
			uint8_t chan_count;

			chan_count = util_ones_count_get(chm->chm, sizeof(chm->chm));
			if (chan_count >= CHM_USED_COUNT_MIN) {
				lll->chm_chan_count = chan_count;
				(void)memcpy(lll->chm_chan_map, chm->chm,
					     sizeof(lll->chm_chan_map));
				lll->ctrl_instant = sys_le16_to_cpu(chm->instant);
			}
		}
	} else {
		/* Unknown control PDU, ignored */
	}
}

/* BIS PDU received with a valid CRC */
static void bis_rx(struct lll_sync_iso *lll)
{
	struct node_rx_iso_meta *iso_meta;
	struct node_rx_pdu **slot;
	struct node_rx_pdu *node_rx;
	uint64_t payload_count;
	uint16_t stream_handle;
	struct pdu_bis *pdu;
	uint8_t group;

	node_rx = ull_iso_pdu_rx_alloc_peek(1U);
	LL_ASSERT_DBG(node_rx);

	pdu = (void *)node_rx->pdu;
#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc) {
		pdu = (void *)evt.pdu_enc;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	/* A new BIG control PDU is sent in the control subevent */
	if (pdu->cstf && (pdu->cssn != lll->cssn_curr)) {
		lll->cssn_next = pdu->cssn;
	}

	payload_count = evt.payload_count + payload_offset(lll, evt.se);

	if (evt.is_create) {
#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
		/* A MIC failure fails the establishment */
		if (lll->enc && pdu->len && !pdu_decrypt(lll, payload_count, node_rx->pdu)) {
			lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;
		}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

		return;
	}

	/* Keep a payload received for the first time, with a free node rx
	 * left to receive the next one.
	 */
	slot = payload_slot_get(lll);
	if (!pdu->len || !slot || *slot || !ull_iso_pdu_rx_alloc_peek(2U)) {
		return;
	}

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc && !pdu_decrypt(lll, payload_count, node_rx->pdu)) {
		lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;

		return;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	(void)ull_iso_pdu_rx_alloc();

	stream_handle = lll->stream_handle[evt.stream];
	node_rx->hdr.type = NODE_RX_TYPE_ISO_PDU;
	node_rx->hdr.handle = LL_BIS_SYNC_HANDLE_FROM_IDX(stream_handle);

	/* The timestamp is the BIG anchor point of the event of the payload */
	iso_meta = &node_rx->rx_iso_meta;
	iso_meta->payload_number = payload_count;
	iso_meta->timestamp = evt.aa_end - addr_us_get(lll->phy) - evt.first_us;
	group = evt.se / lll->bn;
	if (group >= lll->irc) {
		iso_meta->timestamp += (group - lll->irc + 1U) * lll->pto *
				       lll->iso_interval * PERIODIC_INT_UNIT_US;
	}
	iso_meta->status = 0U;

	*slot = node_rx;
}

/* End of a subevent: the next one is listened for, or the event is done */
static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_sync_iso *lll = param;

	if ((e->status == BSR_STATUS_OK) || (e->status == BSR_STATUS_CRC_ERR)) {
		/* The first PDU received gives the drift of the anchor point,
		 * and the last one times the next subevents.
		 */
		if (!evt.trx_cnt) {
			evt.aa_end = e->ts_aa_end - se_offset_get(lll);
		}

		evt.last_aa_end = e->ts_aa_end;
		evt.last_offset_us = se_offset_get(lll);
		evt.last_idx = se_idx_get(lll);

		evt.trx_cnt++;

		if (e->status == BSR_STATUS_OK) {
			evt.crc_ok = 1U;

			if (evt.bis) {
				bis_rx(lll);
			} else {
				ctrl_rx(lll);
			}
		}
	}

	if (evt.is_create || lll->term_reason || !rx_next(lll, false)) {
		isr_done(lll);
	}
}

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	isr_done(param);
}

/* Invalid payload n of the event latency events before the current one, for
 * a payload not received. NULL if there is no node rx to spare, one being
 * kept free to receive the next PDU in.
 */
static struct node_rx_pdu *payload_invalid_get(const struct lll_sync_iso *lll,
					       uint16_t stream_handle, uint16_t latency,
					       uint8_t n)
{
	struct node_rx_iso_meta *iso_meta;
	struct node_rx_pdu *node_rx;
	struct pdu_bis *pdu;
	uint32_t anchor_us;

	if (!ull_iso_pdu_rx_alloc_peek(2U)) {
		return NULL;
	}

	node_rx = ull_iso_pdu_rx_alloc();

	pdu = (void *)node_rx->pdu;
	pdu->ll_id = PDU_BIS_LLID_COMPLETE_END;
	pdu->len = 0U;

	node_rx->hdr.type = NODE_RX_TYPE_ISO_PDU;
	node_rx->hdr.handle = LL_BIS_SYNC_HANDLE_FROM_IDX(stream_handle);

	/* The timestamp is the BIG anchor point of the event, expected if no
	 * PDU has been received in the current event.
	 */
	if (evt.trx_cnt) {
		anchor_us = evt.aa_end - addr_us_get(lll->phy);
	} else {
		anchor_us = evt.rx_start + EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
			    lll->window_widening_event_us;
	}

	iso_meta = &node_rx->rx_iso_meta;
	iso_meta->payload_number = lll->payload_count + n - ((latency + 1U) * lll->bn);
	iso_meta->timestamp = anchor_us - evt.first_us -
			      (latency * lll->iso_interval * PERIODIC_INT_UNIT_US);
	iso_meta->status = 1U;

	return node_rx;
}

/* Hand the payloads of the events elapsed to the ULL in order, the ones not
 * received as invalid payloads. The payloads of later events received as
 * pre-transmissions stay in the sliding window.
 */
static void payloads_put(struct lll_sync_iso *lll)
{
	bool is_put = false;
	uint16_t latency;

	latency = lll->latency_event;
	do {
		for (uint8_t stream = 0U; stream < evt.stream_count; stream++) {
			uint16_t idx = lll->payload_tail;

			for (uint8_t n = 0U; n < lll->bn; n++) {
				struct node_rx_pdu *node_rx = lll->payload[stream][idx];

				if (node_rx) {
					lll->payload[stream][idx] = NULL;
				} else {
					node_rx = payload_invalid_get(lll,
								      lll->stream_handle[stream],
								      latency, n);
				}

				if (node_rx) {
					iso_rx_put(node_rx->hdr.link, node_rx);
					is_put = true;
				}

				if (++idx >= lll->payload_count_max) {
					idx = 0U;
				}
			}
		}

		lll->payload_tail += lll->bn;
		if (lll->payload_tail >= lll->payload_count_max) {
			lll->payload_tail -= lll->payload_count_max;
		}
	} while (latency--);

	if (is_put) {
		iso_rx_sched();
	}
}

/* End of the BIG event */
static void isr_done(struct lll_sync_iso *lll)
{
	struct event_done_extra *e;

	if (!evt.is_create) {
		payloads_put(lll);
	}

	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e);

	if (evt.is_create) {
		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO_ESTAB;
		e->estab_failed = lll->term_reason ? 1U : 0U;
	} else if (lll->term_reason) {
		/* BIG terminated, or MIC failure */
		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO_TERMINATE;

		lll_isr_cleanup(lll);

		return;
	} else {
		/* Use the new channel map from its instant, the counter of the
		 * next event.
		 */
		if (lll->chm_chan_count &&
		    ((((lll->payload_count / lll->bn) - lll->ctrl_instant) & 0xFFFF) <= 0x7FFF)) {
			(void)memcpy(lll->data_chan_map, lll->chm_chan_map,
				     sizeof(lll->data_chan_map));
			lll->data_chan_count = lll->chm_chan_count;
			lll->chm_chan_count = 0U;
		}

		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO;
		e->estab_failed = 0U;
	}

	e->trx_cnt = evt.trx_cnt;
	e->crc_valid = evt.crc_ok;

	if (evt.trx_cnt) {
		e->drift.preamble_to_addr_us = addr_us_get(lll->phy);
		e->drift.start_to_address_actual_us = evt.aa_end - evt.rx_start;
		e->drift.window_widening_event_us = lll->window_widening_event_us;

		/* Reset window widening, as anchor point sync-ed */
		lll->window_widening_event_us = 0U;
		lll->window_size_event_us = 0U;
	}

	lll_isr_cleanup(lll);
}
