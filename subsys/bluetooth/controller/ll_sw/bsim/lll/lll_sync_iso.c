/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

static void isr_rx(const struct bsr_evt *e, void *param);

struct bis_chan {
	uint16_t id;
	uint16_t prn_s;
	uint16_t remap_idx;
};

static struct {
	struct bsr_pkt_cfg cfg;

	/* Start of the event, the window of the first selected subevent, and
	 * the window of the subevents listened for until a PDU is received.
	 */
	uint32_t rx_start;
	uint32_t window_us;

	/* End of the access address of the first selected subevent, given by
	 * the first PDU received in the event.
	 */
	uint32_t aa_end;

	/* The last PDU received in the event, at the subevent of that offset
	 * and position in time among all the subevents of the BIG.
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

	/* The first selected BISes in lll_sync_iso.stream_handle, in BIS number
	 * order, that the BIG has.
	 */
	uint8_t stream_count;

	/* Selected BIS, its BIS number, 0 for the control subevent, and
	 * subevent of the BIS.
	 */
	uint8_t stream;
	uint8_t bis;
	uint8_t se;

	uint8_t access_addr[4];

	uint8_t trx_cnt;
	uint8_t crc_ok;

	/* The control subevent is listened for once a BIS PDU has been
	 * received in the event and all of them announce a BIG control PDU,
	 * with the CSSN of the last one (Core Spec Vol 6, Part B, Section
	 * 4.4.5).
	 */
	uint8_t cssn;
	uint8_t is_bis_rx:1;
	uint8_t is_cstf:1;

	uint8_t is_create:1;

	struct bis_chan chan[BT_CTLR_SYNC_ISO_STREAM_MAX];

	uint8_t pdu_ctrl[offsetof(struct pdu_bis, payload) + sizeof(struct pdu_big_ctrl)];

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	uint8_t pdu_enc[offsetof(struct pdu_bis, payload) +
			MAX(LL_BIS_OCTETS_RX_MAX, sizeof(struct pdu_big_ctrl)) + PDU_MIC_SIZE];
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */
} evt;

int lll_sync_iso_init(void)
{
	return 0;
}

static bool is_sequential(const struct lll_sync_iso *lll)
{
	return lll->bis_spacing >= (lll->sub_interval * lll->nse);
}

static struct lll_sync_iso_stream *stream_get(const struct lll_sync_iso *lll, uint8_t stream)
{
	return ull_sync_iso_lll_stream_get(lll->stream_handle[stream]);
}

/* Moves to the next subevent in time of the selected BISes, returns false
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

/* Offset from the first payload of the event of the payload sent in subevent
 * se of a BIS. The first IRC x BN subevents send the BN payloads of the event
 * IRC times, and each next group of BN subevents the payloads of the event
 * PTO events further ahead (pre-transmissions).
 */
static uint16_t payload_offset(const struct lll_sync_iso *lll, uint8_t se)
{
	uint8_t group = se / lll->bn;
	uint8_t n = se % lll->bn;

	if (group < lll->irc) {
		return n;
	}

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

/* The broadcaster sends an empty PDU while it does not have the payload yet,
 * e.g. in a pre-transmission subevent, so the payload replaces an empty PDU
 * kept.
 */
static bool payload_is_empty(const struct node_rx_pdu *node_rx)
{
	const struct pdu_bis *pdu = (const void *)node_rx->pdu;

	return pdu->len == 0U;
}

/* Offset of the current subevent from the first selected subevent */
static uint32_t se_offset_get(const struct lll_sync_iso *lll)
{
	uint32_t offset;

	if (evt.bis != 0U) {
		offset = ((evt.bis - 1U) * lll->bis_spacing) + (evt.se * lll->sub_interval);
	} else {
		/* BIG_Control_Offset (Core Spec Vol 6, Part B, Section 4.4.6.7) */
		offset = is_sequential(lll) ? (lll->num_bis * lll->bis_spacing) :
					      (lll->nse * lll->sub_interval);
	}

	return offset - evt.first_us;
}

/* Position in time of the current subevent among all the subevents of the
 * BIG.
 */
static uint16_t se_idx_get(const struct lll_sync_iso *lll)
{
	if (evt.bis == 0U) {
		return lll->num_bis * lll->nse;
	}

	if (is_sequential(lll)) {
		return ((evt.bis - 1U) * lll->nse) + evt.se;
	}

	return (evt.se * lll->num_bis) + (evt.bis - 1U);
}

/* The channel selection of a BIS moves along its subevents, whether they are
 * listened for or not.
 */
static void bis_chan_calc(const struct lll_sync_iso *lll)
{
	struct bis_chan *chan = &evt.chan[evt.stream];

	if (evt.se == 0U) {
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

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
/* Returns false on a MIC failure */
static bool pdu_decrypt(struct lll_sync_iso *lll, uint64_t payload_count, void *out)
{
	lll->ccm_rx.counter = payload_count;
	(void)memcpy(lll->ccm_rx.iv, lll->giv, 4U);
	mem_xor_32(lll->ccm_rx.iv, lll->ccm_rx.iv, evt.access_addr);

	return lll_ccm_decrypt(&lll->ccm_rx, LLL_CCM_HDR_MASK_BIS, evt.pdu_enc, out);
}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

static void ctrl_rx(struct lll_sync_iso *lll)
{
	struct pdu_bis *pdu = (void *)evt.pdu_ctrl;

	lll->cssn_curr = evt.cssn;

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if ((lll->enc != 0U) && !pdu_decrypt(lll, evt.payload_count, pdu)) {
		lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;

		return;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	if (pdu->ll_id != PDU_BIS_LLID_CTRL) {
		return;
	}

	if (pdu->ctrl.opcode == PDU_BIG_CTRL_TYPE_TERM_IND) {
		if (lll->term_reason == 0U) {
			struct pdu_big_ctrl_term_ind *term = &pdu->ctrl.term_ind;

			lll->term_reason = term->reason;
			lll->ctrl_instant = sys_le16_to_cpu(term->instant);
		}
	} else if (pdu->ctrl.opcode == PDU_BIG_CTRL_TYPE_CHAN_MAP_IND) {
		if (lll->chm_chan_count == 0U) {
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
		/* Unknown control PDUs are ignored */
	}
}

static void bis_rx(struct lll_sync_iso *lll)
{
	struct node_rx_iso_meta *iso_meta;
	struct node_rx_pdu **slot;
	struct node_rx_pdu *node_rx;
	uint64_t payload_count;
	uint16_t stream_handle;
	struct pdu_bis *pdu;
	bool is_replace;
	uint8_t group;

	node_rx = ull_iso_pdu_rx_alloc_peek(1U);
	LL_ASSERT_DBG(node_rx != NULL);

	pdu = (void *)node_rx->pdu;
#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc != 0U) {
		pdu = (void *)evt.pdu_enc;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	evt.is_bis_rx = 1U;
	if (pdu->cstf != 0U) {
		evt.cssn = pdu->cssn;
	} else {
		evt.is_cstf = 0U;
	}

	payload_count = evt.payload_count + payload_offset(lll, evt.se);

	if (evt.is_create != 0U) {
#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
		/* A MIC failure fails the establishment */
		if ((lll->enc != 0U) && (pdu->len != 0U) &&
		    !pdu_decrypt(lll, payload_count, node_rx->pdu)) {
			lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;
		}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

		return;
	}

	/* Keep a payload received for the first time, an empty one too, with a
	 * free node rx left to receive the next one.
	 */
	slot = payload_slot_get(lll);
	if (slot == NULL) {
		return;
	}

	is_replace = (*slot != NULL) && (pdu->len != 0U) && payload_is_empty(*slot);
	if (((*slot != NULL) && !is_replace) ||
	    ((*slot == NULL) && (ull_iso_pdu_rx_alloc_peek(2U) == NULL))) {
		return;
	}

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc != 0U) {
		if (pdu->len == 0U) {
			/* An empty PDU is not encrypted */
			(void)memcpy(node_rx->pdu, pdu, offsetof(struct pdu_bis, payload));
		} else if (!pdu_decrypt(lll, payload_count, node_rx->pdu)) {
			lll->term_reason = BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL;

			return;
		}
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	if (is_replace) {
		pdu = (void *)node_rx->pdu;
		(void)memcpy((*slot)->pdu, pdu, offsetof(struct pdu_bis, payload) + pdu->len);

		return;
	}

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

	if (ull_iso_pdu_rx_alloc_peek(2U) == NULL) {
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
	if (evt.trx_cnt != 0U) {
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

/* The payloads of the events elapsed are handed to the ULL in order, the ones
 * not received as invalid payloads. The payloads of later events received as
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

				if (node_rx != NULL) {
					lll->payload[stream][idx] = NULL;
				} else {
					node_rx = payload_invalid_get(lll,
								      lll->stream_handle[stream],
								      latency, n);
				}

				if (node_rx != NULL) {
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
	} while (latency-- != 0U);

	if (is_put) {
		iso_rx_sched();
	}
}

static void isr_done(struct lll_sync_iso *lll)
{
	struct event_done_extra *e;

	if (evt.is_create == 0U) {
		payloads_put(lll);
	}

	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	if ((evt.is_create == 0U) && (lll->term_reason != 0U)) {
		/* The BIG was terminated, or a MIC failure terminates the sync */
		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO_TERMINATE;

		lll_isr_cleanup(lll);

		return;
	}

	if (evt.is_create != 0U) {
		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO_ESTAB;
		e->estab_failed = (lll->term_reason != 0U) ? 1U : 0U;
	} else {
		e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO;
		e->estab_failed = 0U;
	}

	e->trx_cnt = evt.trx_cnt;
	e->crc_valid = evt.crc_ok;

	if (evt.trx_cnt != 0U) {
		e->drift.preamble_to_addr_us = addr_us_get(lll->phy);
		e->drift.start_to_address_actual_us = evt.aa_end - evt.rx_start;
		e->drift.window_widening_event_us = lll->window_widening_event_us;

		/* Synchronized again on the anchor point received */
		lll->window_widening_event_us = 0U;
		lll->window_size_event_us = 0U;
	}

	lll_isr_cleanup(lll);
}

static void se_rx(struct lll_sync_iso *lll)
{
	uint8_t crc_init[3];
	uint32_t offset_us;
	uint32_t window_us;
	uint32_t start_us;
	void *buf;

	/* The BIS number is the least significant octet of the CRC init */
	util_bis_aa_le32(evt.bis, lll->seed_access_addr, evt.access_addr);
	crc_init[0] = evt.bis;
	(void)memcpy(&crc_init[1], lll->base_crc_init, sizeof(lll->base_crc_init));
	evt.cfg.aa = sys_get_le32(evt.access_addr);
	evt.cfg.crc_init = sys_get_le24(crc_init);

	if (evt.bis != 0U) {
		struct node_rx_pdu *node_rx;

		/* By design, there is always a free node rx to receive in */
		node_rx = ull_iso_pdu_rx_alloc_peek(1U);
		LL_ASSERT_DBG(node_rx != NULL);

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
	if (lll->enc != 0U) {
		buf = evt.pdu_enc;
		evt.cfg.max_len += PDU_MIC_SIZE;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	/* Until a PDU is received in the event, the window is widened by the
	 * drift of the sleep clocks since the last anchor point received.
	 */
	offset_us = se_offset_get(lll);
	if (evt.trx_cnt != 0U) {
		uint32_t jitter_us;

		jitter_us = lll_radio_se_jitter_get(se_idx_get(lll) - evt.last_idx,
						    offset_us - evt.last_offset_us);
		start_us = evt.last_aa_end + (offset_us - evt.last_offset_us) -
			   addr_us_get(lll->phy) - jitter_us;
		window_us = (jitter_us << 1) + RANGE_DELAY_US + addr_us_get(lll->phy);
	} else {
		start_us = evt.rx_start + offset_us;
		window_us = evt.window_us;
	}

	lll_radio_rx(&evt.cfg, start_us, window_us, buf, isr_rx, lll);
}

/* Listens for the next subevent in time, from the current one if from_curr,
 * that carries a payload not received yet, as the repetitions and
 * pre-transmissions of a payload are only listened for until one of them is
 * received. Else for the control subevent, if a new BIG control PDU was
 * announced. Returns false if there is none.
 */
static bool rx_next(struct lll_sync_iso *lll, bool from_curr)
{
	if (evt.bis == 0U) {
		/* The control subevent is the last one */
		return false;
	}

	if (from_curr || se_next(lll)) {
		do {
			struct node_rx_pdu **slot;

			bis_chan_calc(lll);

			/* The event that establishes the sync only listens for
			 * the first subevent.
			 */
			if (evt.is_create != 0U) {
				se_rx(lll);

				return true;
			}

			slot = payload_slot_get(lll);
			if ((slot != NULL) && ((*slot == NULL) || payload_is_empty(*slot))) {
				se_rx(lll);

				return true;
			}
		} while (se_next(lll));
	}

	if ((evt.is_create == 0U) && (evt.is_bis_rx != 0U) && (evt.is_cstf != 0U) &&
	    (evt.cssn != lll->cssn_curr)) {
		evt.bis = 0U;
		se_rx(lll);

		return true;
	}

	return false;
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_sync_iso *lll = param;

	if ((e->status == BSR_STATUS_OK) || (e->status == BSR_STATUS_CRC_ERR)) {
		/* The first PDU received gives the drift of the anchor point,
		 * and the last one times the next subevents.
		 */
		if (evt.trx_cnt == 0U) {
			evt.aa_end = e->ts_aa_end - se_offset_get(lll);
		}

		evt.last_aa_end = e->ts_aa_end;
		evt.last_offset_us = se_offset_get(lll);
		evt.last_idx = se_idx_get(lll);

		evt.trx_cnt++;

		if (e->status == BSR_STATUS_OK) {
			evt.crc_ok = 1U;

			if (evt.bis != 0U) {
				bis_rx(lll);
			} else {
				ctrl_rx(lll);
			}
		}
	}

	if ((evt.is_create != 0U) || (lll->term_reason != 0U) || !rx_next(lll, false)) {
		isr_done(lll);
	}
}

static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

	isr_done(param);
}

static int prepare_cb_common(struct lll_prepare_param *p, bool is_create)
{
	struct lll_sync_iso *lll = p->param;
	struct lll_sync_iso_stream *stream;
	uint32_t ticks_ref;
	int err;

	DEBUG_RADIO_START_O(1);

	lll->lazy_prepare = p->lazy;
	lll->latency_event = lll->latency_prepare + lll->lazy_prepare;
	evt.event_counter = (lll->payload_count / lll->bn) + lll->latency_event;

	lll->payload_count += (lll->latency_event + 1U) * lll->bn;
	evt.payload_count = lll->payload_count - lll->bn;

	lll->latency_prepare = 0U;

	/* The new channel map is used from its instant, also when the events
	 * before it were missed.
	 */
	if ((lll->chm_chan_count != 0U) &&
	    (((evt.event_counter - lll->ctrl_instant) & EVENT_INSTANT_MAX) <=
	     EVENT_INSTANT_LATENCY_MAX)) {
		(void)memcpy(lll->data_chan_map, lll->chm_chan_map, sizeof(lll->data_chan_map));
		lll->data_chan_count = lll->chm_chan_count;
		lll->chm_chan_count = 0U;
	}

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

	evt.is_create = is_create;
	evt.trx_cnt = 0U;
	evt.crc_ok = 0U;
	evt.is_bis_rx = 0U;
	evt.is_cstf = 1U;

	evt.stream_count = 0U;
	while ((evt.stream_count < lll->stream_count) &&
	       (stream_get(lll, evt.stream_count)->bis_index <= lll->num_bis)) {
		evt.stream_count++;
	}

	if ((evt.stream_count == 0U) || (lll_preempt_calc(p) != 0U)) {
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
	LL_ASSERT_ERR(err == 0);

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

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		/* The event is done with what it has received, once the radio
		 * has been stopped.
		 */
		lll_radio_stop(isr_abort, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* The event counter and window widening of the next event account for
	 * the events aborted in the prepare pipeline.
	 */
	lll = prepare_param->param;
	lll->lazy_prepare = prepare_param->lazy;
	lll->latency_prepare += (lll->lazy_prepare + 1U);

	lll->window_widening_prepare_us += lll->window_widening_periodic_us *
					   (prepare_param->lazy + 1U);
	if (lll->window_widening_prepare_us > lll->window_widening_max_us) {
		lll->window_widening_prepare_us = lll->window_widening_max_us;
	}

	/* For the ULL to count the event as missed, for the sync timeout */
	e = ull_event_done_extra_get();
	LL_ASSERT_ERR(e != NULL);

	e->type = EVENT_DONE_EXTRA_TYPE_SYNC_ISO;
	e->estab_failed = 0U;
	e->trx_cnt = 0U;
	e->crc_valid = 0U;

	lll_done(param);
}

void lll_sync_iso_create_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, create_prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

void lll_sync_iso_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

void lll_sync_iso_flush(uint8_t handle, struct lll_sync_iso *lll)
{
	bool is_put = false;

	ARG_UNUSED(handle);

	/* The payloads received ahead of their event, as pre-transmissions,
	 * are handed to the ULL rather than lost with the sync.
	 */
	for (uint8_t stream = 0U; stream < lll->stream_count; stream++) {
		uint16_t idx = lll->payload_tail;

		for (uint8_t n = 0U; n < lll->payload_count_max; n++) {
			struct node_rx_pdu *node_rx = lll->payload[stream][idx];

			if (node_rx != NULL) {
				lll->payload[stream][idx] = NULL;
				iso_rx_put(node_rx->hdr.link, node_rx);
				is_put = true;
			}

			if (++idx >= lll->payload_count_max) {
				idx = 0U;
			}
		}
	}

	if (is_put) {
		iso_rx_sched();
	}
}
