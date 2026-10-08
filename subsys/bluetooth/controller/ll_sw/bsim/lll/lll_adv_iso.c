/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Broadcast Isochronous Group (BIG) events of the BabbleSim LLL.
 *
 * A BIG event transmits the NSE subevents of each BIS, followed by the
 * control subevent while a BIG control PDU is being sent. The radio model
 * transmits at an absolute time, so each subevent is sent at its own start,
 * (BIS - 1) x BIS_Spacing + (subevent - 1) x Sub_Interval after the BIG
 * anchor point, which covers the sequential and the interleaved arrangements
 * alike: they only differ in the order of the subevents in time.
 *
 * Of the NSE subevents of a BIS, the first IRC x BN ones send the BN payloads
 * of the event IRC times, and the others the payloads of later events, PTO
 * events ahead for each group of BN subevents (pre-transmissions).
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
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_iso.h"
#include "lll_iso_tx.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_ccm.h"

#include "ll_feat.h"

#include "hal/debug.h"

static void prepare(void *param, lll_prepare_cb_t prepare_cb);
static int create_prepare_cb(struct lll_prepare_param *p);
static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void se_tx(struct lll_adv_iso *lll);
static void isr_tx(const struct bsr_evt *e, void *param);

/* Channel selection of a BIS, along the subevents of an event */
struct bis_chan {
	uint16_t id;
	uint16_t prn_s;
	uint16_t remap_idx;
};

/* Size of a BIS PDU: header, payload and MIC */
#define BIS_PDU_SIZE_MAX (offsetof(struct pdu_bis, payload) + \
			  MAX(LL_BIS_OCTETS_TX_MAX, sizeof(struct pdu_big_ctrl)) + \
			  PDU_MIC_SIZE)

/* State of the current BIG event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* BIG anchor point */
	uint32_t anchor_us;

	/* bisPayloadCounter of the first payload of the event */
	uint64_t payload_count;

	uint16_t event_counter;

	/* Subevent being sent: BIS number, 0 for the control subevent, and
	 * index of the subevent in the BIS.
	 */
	uint8_t bis;
	uint8_t se;

	/* Event completing the creation of the BIG */
	uint8_t is_create:1;

	/* A BIG control PDU is sent in the event */
	uint8_t is_ctrl:1;

	struct bis_chan chan[BT_CTLR_ADV_ISO_STREAM_MAX];

	/* BIG control PDU */
	uint8_t pdu_ctrl[offsetof(struct pdu_bis, payload) + sizeof(struct pdu_big_ctrl)];

	/* Empty or encrypted PDU being sent */
	uint8_t pdu_tx[BIS_PDU_SIZE_MAX];
} evt;

int lll_adv_iso_init(void)
{
	return 0;
}

int lll_adv_iso_reset(void)
{
	return 0;
}

void lll_adv_iso_create_prepare(void *param)
{
	prepare(param, create_prepare_cb);
}

void lll_adv_iso_prepare(void *param)
{
	prepare(param, prepare_cb);
}

static void prepare(void *param, lll_prepare_cb_t cb)
{
	struct lll_prepare_param *p = param;
	struct lll_adv_iso *lll = p->param;
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	/* Events elapsed. The ULL reads them to fill the BIGInfo, also for an
	 * event that is cancelled in the prepare pipeline.
	 */
	lll->latency_prepare += p->lazy + 1U;

	err = lll_prepare(is_abort_cb, abort_cb, cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

static bool is_sequential(const struct lll_adv_iso *lll)
{
	return lll->bis_spacing >= (lll->sub_interval * lll->nse);
}

/* Move to the next subevent in time, returns false after the last subevent
 * of the BISes.
 */
static bool se_next(const struct lll_adv_iso *lll)
{
	if (is_sequential(lll)) {
		if (++evt.se < lll->nse) {
			return true;
		}

		evt.se = 0U;

		return ++evt.bis <= lll->num_bis;
	}

	if (++evt.bis <= lll->num_bis) {
		return true;
	}

	evt.bis = 1U;

	return ++evt.se < lll->nse;
}

/* Offset from the first payload of the event of the payload sent in
 * subevent se of a BIS.
 */
static uint16_t payload_offset(const struct lll_adv_iso *lll, uint8_t se)
{
	uint8_t group = se / lll->bn;
	uint8_t n = se % lll->bn;

	if (group < lll->irc) {
		return n;
	}

	/* Pre-transmission of a payload of a later event */
	return ((group - lll->irc + 1U) * lll->pto * lll->bn) + n;
}

/* PDU of the payload of a BIS with the bisPayloadCounter payload_count, NULL
 * if it has not been provided.
 */
static struct pdu_bis *payload_get(const struct lll_adv_iso *lll, uint8_t bis,
				   uint64_t payload_count)
{
	struct lll_adv_iso_stream *stream;
	struct node_tx_iso *tx;
	memq_link_t *link;

	stream = ull_adv_iso_lll_stream_get(lll->stream_handle[bis - 1U]);
	LL_ASSERT_DBG(stream);

	link = stream->memq_tx.head;
	while ((link = memq_peek(link, stream->memq_tx.tail, (void **)&tx))) {
		if (tx->payload_count >= payload_count) {
			return (tx->payload_count == payload_count) ? (void *)tx->pdu : NULL;
		}

		link = link->next;
	}

	return NULL;
}

/* Release the payloads with a bisPayloadCounter before payload_count, they
 * are not sent anymore.
 */
static void payload_release(const struct lll_adv_iso *lll, uint64_t payload_count)
{
	for (uint8_t i = 0U; i < lll->num_bis; i++) {
		uint16_t stream_handle = lll->stream_handle[i];
		struct lll_adv_iso_stream *stream;
		struct node_tx_iso *tx;
		memq_link_t *link;

		stream = ull_adv_iso_lll_stream_get(stream_handle);
		LL_ASSERT_DBG(stream);

		while ((link = memq_peek(stream->memq_tx.head, stream->memq_tx.tail,
					 (void **)&tx)) &&
		       (tx->payload_count < payload_count)) {
			(void)memq_dequeue(stream->memq_tx.tail, &stream->memq_tx.head, NULL);

			tx->next = link;
			ull_iso_lll_ack_enqueue(LL_BIS_ADV_HANDLE_FROM_IDX(stream_handle), tx);
		}
	}
}

/* Start sending a BIG control PDU the ULL asked for, in this event and the
 * next ones until its instant.
 */
static void ctrl_start(struct lll_adv_iso *lll)
{
	if (lll->term_req) {
		if (!lll->term_ack) {
			lll->term_ack = 1U;
			lll->ctrl_expire = CONN_ESTAB_COUNTDOWN;
			lll->ctrl_instant = evt.event_counter + lll->ctrl_expire - 1U;
			lll->cssn++;
		}
	} else if (((lll->chm_req - lll->chm_ack) & CHM_STATE_MASK) == CHM_STATE_REQ) {
		lll->chm_ack--;
		lll->ctrl_expire = CONN_ESTAB_COUNTDOWN;
		lll->ctrl_instant = evt.event_counter + lll->ctrl_expire;
		lll->cssn++;
	}

	evt.is_ctrl = lll->term_ack ||
		      (((lll->chm_req - lll->chm_ack) & CHM_STATE_MASK) == CHM_STATE_SEND);
}

static int prepare_cb_common(struct lll_prepare_param *p, bool is_create)
{
	struct lll_adv_iso *lll = p->param;
	uint32_t ticks_ref;
	int err;

	DEBUG_RADIO_START_A(1);

	/* Deduce the latency */
	lll->latency_event = lll->latency_prepare - 1U;

	/* Calculate the current event counter value */
	evt.event_counter = (lll->payload_count / lll->bn) + lll->latency_event;

	/* Update BIS payload counter to next value */
	lll->payload_count += (lll->latency_prepare * lll->bn);
	evt.payload_count = lll->payload_count - lll->bn;

	/* Reset accumulated latencies */
	lll->latency_prepare = 0U;

	evt.is_create = is_create;

	/* Payloads of the events missed are not sent anymore */
	payload_release(lll, evt.payload_count);

	ctrl_start(lll);

	if (lll_preempt_calc(p)) {
		/* Not sent, the event is done */
		lll_event_abort(lll);

		return -ECANCELED;
	}

	evt.cfg.phy = lll_radio_phy(lll->phy);
	evt.cfg.max_len = 0U;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->adv->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	/* The first subevent of the first BIS starts at the anchor point */
	evt.anchor_us = lll_event_start_get(p, &ticks_ref);
	evt.bis = 1U;
	evt.se = 0U;
	se_tx(lll);

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_A(1);

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
	ARG_UNUSED(next);
	ARG_UNUSED(curr);
	ARG_UNUSED(resume_cb);

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* The payloads not sent are released in the next event */
		lll_event_abort(param);

		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline. Its latency was accumulated in
	 * its prepare.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(param);
}

/* PDU of the current BIS subevent, an empty PDU if its payload has not been
 * provided.
 */
static struct pdu_bis *bis_pdu_get(const struct lll_adv_iso *lll, uint64_t payload_count)
{
	struct pdu_bis *pdu;

	pdu = payload_get(lll, evt.bis, payload_count);
	if (!pdu) {
		pdu = (void *)evt.pdu_tx;
		pdu->ll_id = lll->framing ? PDU_BIS_LLID_FRAMED : PDU_BIS_LLID_START_CONTINUE;
		pdu->len = 0U;
	}

	return pdu;
}

/* PDU of the control subevent */
static struct pdu_bis *ctrl_pdu_get(const struct lll_adv_iso *lll)
{
	struct pdu_bis *pdu = (void *)evt.pdu_ctrl;

	pdu->ll_id = PDU_BIS_LLID_CTRL;

	if (lll->term_ack) {
		struct pdu_big_ctrl_term_ind *term = &pdu->ctrl.term_ind;

		pdu->len = offsetof(struct pdu_big_ctrl, ctrl_data) + sizeof(*term);
		pdu->ctrl.opcode = PDU_BIG_CTRL_TYPE_TERM_IND;
		term->reason = lll->term_reason;
		term->instant = sys_cpu_to_le16(lll->ctrl_instant);
	} else {
		struct pdu_big_ctrl_chan_map_ind *chm = &pdu->ctrl.chan_map_ind;

		pdu->len = offsetof(struct pdu_big_ctrl, ctrl_data) + sizeof(*chm);
		pdu->ctrl.opcode = PDU_BIG_CTRL_TYPE_CHAN_MAP_IND;
		(void)memcpy(chm->chm, lll->chm_chan_map, sizeof(chm->chm));
		chm->instant = sys_cpu_to_le16(lll->ctrl_instant);
	}

	return pdu;
}

/* Transmit the current subevent */
static void se_tx(struct lll_adv_iso *lll)
{
	uint64_t payload_count;
	uint8_t access_addr[4];
	struct bis_chan *chan;
	struct bis_chan ctrl;
	uint8_t crc_init[3];
	struct pdu_bis *pdu;
	uint32_t at;

	/* Access address and CRC initial value of the BIS, the BIS number
	 * being the least significant octet of the latter.
	 */
	util_bis_aa_le32(evt.bis, lll->seed_access_addr, access_addr);
	crc_init[0] = evt.bis;
	(void)memcpy(&crc_init[1], lll->base_crc_init, sizeof(lll->base_crc_init));
	evt.cfg.aa = sys_get_le32(access_addr);
	evt.cfg.crc_init = sys_get_le24(crc_init);

	if (evt.bis) {
		chan = &evt.chan[evt.bis - 1U];
		at = evt.anchor_us + ((evt.bis - 1U) * lll->bis_spacing) +
		     (evt.se * lll->sub_interval);
		payload_count = evt.payload_count + payload_offset(lll, evt.se);
		pdu = bis_pdu_get(lll, payload_count);
	} else {
		/* The control subevent follows the last subevent, as a next
		 * subevent of the last BIS in the sequential arrangement, or of
		 * a next BIS in the interleaved one.
		 */
		chan = &ctrl;
		at = evt.anchor_us + ((lll->num_bis - 1U) * lll->bis_spacing) +
		     ((lll->nse - 1U) * lll->sub_interval) +
		     (is_sequential(lll) ? lll->sub_interval : lll->bis_spacing);
		payload_count = evt.payload_count;
		pdu = ctrl_pdu_get(lll);
	}

	/* The first subevent of a BIS hops with the event counter, the next
	 * ones from it.
	 */
	if (!evt.se || !evt.bis) {
		chan->id = lll_chan_id(access_addr);
		evt.cfg.chan = lll_chan_iso_event(evt.event_counter, chan->id,
						  lll->data_chan_map, lll->data_chan_count,
						  &chan->prn_s, &chan->remap_idx);
	} else {
		evt.cfg.chan = lll_chan_iso_subevent(chan->id, lll->data_chan_map,
						     lll->data_chan_count, &chan->prn_s,
						     &chan->remap_idx);
	}

	pdu->cssn = lll->cssn;
	pdu->cstf = evt.bis ? evt.is_ctrl : 0U;
	pdu->rfu = 0U;

#if defined(CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
	if (lll->enc && pdu->len) {
		lll->ccm_tx.counter = payload_count;
		(void)memcpy(lll->ccm_tx.iv, lll->giv, 4U);
		mem_xor_32(lll->ccm_tx.iv, lll->ccm_tx.iv, access_addr);

		lll_ccm_encrypt(&lll->ccm_tx, LLL_CCM_HDR_MASK_BIS, pdu, evt.pdu_tx);
		pdu = (void *)evt.pdu_tx;
	}
#endif /* CONFIG_BT_CTLR_BROADCAST_ISO_ENC */

	lll_radio_tx(&evt.cfg, at, pdu, isr_tx, lll);
}

/* End of a BIG control PDU sent */
static void ctrl_done(struct lll_adv_iso *lll)
{
	uint16_t elapsed_event = lll->latency_event + 1U;
	struct lll_adv_sync *sync_lll;

	if (lll->ctrl_expire > elapsed_event) {
		lll->ctrl_expire -= elapsed_event;

		return;
	}

	/* At the instant */
	lll->ctrl_expire = 0U;

	if (lll->term_ack) {
		ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_ADV_ISO_TERMINATE);

		return;
	}

	/* Channel map update procedure done */
	lll->chm_ack = lll->chm_req;

	/* Have the ULL update the channel map in the BIGInfo of the periodic
	 * advertising, which uses the one of the BIG until then.
	 */
	sync_lll = lll->adv->sync;
	if (sync_lll->iso_chm_done_req == sync_lll->iso_chm_done_ack) {
		struct node_rx_pdu *rx;

		sync_lll->iso_chm_done_req++;

		rx = ull_pdu_rx_alloc();
		LL_ASSERT_ERR(rx);

		rx->hdr.type = NODE_RX_TYPE_BIG_CHM_COMPLETE;
		rx->rx_ftr.param = lll;

		ull_rx_put_sched(rx->hdr.link, rx);
	}

	/* Use the new channel map */
	lll->data_chan_count = lll->chm_chan_count;
	(void)memcpy(lll->data_chan_map, lll->chm_chan_map, sizeof(lll->data_chan_map));
}

/* End of a subevent: the next one is sent, or the event is done */
static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_adv_iso *lll = param;
	bool ctrl_sent;

	if (e->status == BSR_STATUS_OK) {
		if (evt.bis && se_next(lll)) {
			se_tx(lll);

			return;
		}

		if (evt.bis && evt.is_ctrl) {
			evt.bis = 0U;
			evt.se = 0U;
			se_tx(lll);

			return;
		}
	}

	ctrl_sent = (e->status == BSR_STATUS_OK) && !evt.bis;

	/* The payloads of the event have been sent */
	payload_release(lll, lll->payload_count);

	if (evt.is_create) {
		ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_ADV_ISO_COMPLETE);
	}

	if (ctrl_sent) {
		ctrl_done(lll);
	}

	lll_isr_cleanup(lll);
}
