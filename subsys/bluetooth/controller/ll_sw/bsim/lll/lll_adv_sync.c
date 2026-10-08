/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Periodic advertising events of the BabbleSim LLL.
 *
 * A periodic advertising event transmits the AUX_SYNC_IND on the channel of
 * the event counter, followed back to back by the AUX_CHAIN_IND PDUs of its
 * chain. The channel of a chain PDU is set in the AuxPtr of the PDU before
 * it, before that PDU is transmitted.
 *
 * There is no Constant Tone Extension transmission.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

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
#include "lll_clock.h"
#include "lll_chan.h"
#include "lll_df_types.h"
#include "lll_conn.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_sync.h"
#include "lll_adv_iso.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"

#include "hal/debug.h"

static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void pdu_tx(struct lll_adv_sync *lll, struct pdu_adv *pdu, uint32_t at);
static void isr_tx(const struct bsr_evt *e, void *param);
static void isr_done(const struct bsr_evt *e, void *param);

/* State of the current periodic advertising event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* The PDU being transmitted, when its AuxPtr points to a chain PDU
	 * that follows it.
	 */
	struct pdu_adv *pdu_chain;
} evt;

int lll_adv_sync_init(void)
{
	return 0;
}

int lll_adv_sync_reset(void)
{
	return 0;
}

void lll_adv_sync_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

static bool is_instant_or_past(uint16_t event_counter, uint16_t instant)
{
	uint16_t instant_latency;

	instant_latency = (event_counter - instant) & EVENT_INSTANT_MAX;

	return instant_latency <= EVENT_INSTANT_LATENCY_MAX;
}

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
/* AuxPtr of a periodic advertising PDU, NULL if it has none */
static struct pdu_adv_aux_ptr *aux_ptr_get(struct pdu_adv *pdu)
{
	struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	uint8_t *dptr;

	if (!com_hdr->ext_hdr_len || !hdr->aux_ptr) {
		return NULL;
	}

	/* No AdvA and TargetA, which are RFU in periodic advertising PDUs */
	dptr = hdr->data;
	if (hdr->cte_info) {
		dptr += sizeof(struct pdu_cte_info);
	}
	if (hdr->adi) {
		dptr += sizeof(struct pdu_adv_adi);
	}

	return (void *)dptr;
}
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_adv_sync *lll = p->param;
	uint16_t event_counter;
	uint32_t ticks_ref;
	struct pdu_adv *pdu;
	uint32_t start_us;
	uint8_t chan_idx;
	uint8_t upd;
	int err;

	DEBUG_RADIO_START_A(1);

	/* Calculate the current event latency */
	lll->latency_event = lll->latency_prepare + p->lazy;

	/* Calculate the current event counter value */
	event_counter = lll->event_counter + lll->latency_event;

	/* Update event counter to next value */
	lll->event_counter = event_counter + 1U;

	/* Reset accumulated latencies */
	lll->latency_prepare = 0U;

	/* Process channel map update, if any */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(event_counter, lll->chm_instant)) {
		/* At or past the instant, use channelMapNew */
		lll->chm_first = lll->chm_last;
	}

	chan_idx = lll_chan_sel_2(event_counter, lll->data_chan_id,
				  lll->chm[lll->chm_first].data_chan_map,
				  lll->chm[lll->chm_first].data_chan_count);

	upd = 0U;
	pdu = lll_adv_sync_data_latest_get(lll, NULL, &upd);
	LL_ASSERT_DBG(pdu);

#if defined(CONFIG_BT_CTLR_ADV_ISO) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	if (lll->iso) {
		ull_adv_iso_lll_biginfo_fill(pdu, lll);
	}
#endif /* CONFIG_BT_CTLR_ADV_ISO && CONFIG_BT_TICKER_EXT_EXPIRE_INFO */

	if (lll_preempt_calc(p)) {
		/* Not sent, the event is done */
		lll_event_abort(lll);

		return -ECANCELED;
	}

	evt.cfg.aa = sys_get_le32(lll->access_addr);
	evt.cfg.crc_init = sys_get_le24(lll->crc_init);
	evt.cfg.phy = lll_radio_phy(lll->adv->phy_s);
	evt.cfg.chan = chan_idx;
	evt.cfg.max_len = PDU_AC_PAYLOAD_SIZE_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->adv->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	start_us = lll_event_start_get(p, &ticks_ref);
	pdu_tx(lll, pdu, start_us);

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_A(1);

	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	/* A periodic advertising event is not resumed */
	ARG_UNUSED(next);
	ARG_UNUSED(curr);
	ARG_UNUSED(resume_cb);

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct lll_adv_sync *lll;
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done, param);

		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* Accumulate the latency as event is aborted while being in pipeline */
	lll = prepare_param->param;
	lll->latency_prepare += (prepare_param->lazy + 1U);

	lll_done(param);
}

/* Transmit pdu at time at, followed back to back by the chain PDU that its
 * AuxPtr points to, if any.
 */
static void pdu_tx(struct lll_adv_sync *lll, struct pdu_adv *pdu, uint32_t at)
{
#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	struct pdu_adv_aux_ptr *aux_ptr;

	aux_ptr = aux_ptr_get(pdu);
	if (aux_ptr) {
		/* Channel of the chain PDU that follows */
		aux_ptr->chan_idx = lll_chan_sel_2(lll->data_chan_counter, lll->data_chan_id,
						   lll->chm[lll->chm_first].data_chan_map,
						   lll->chm[lll->chm_first].data_chan_count);
		lll->data_chan_counter++;

		evt.pdu_chain = pdu;
	} else {
		evt.pdu_chain = NULL;
	}
#else /* !CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */
	evt.pdu_chain = NULL;
#endif /* !CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

	lll_radio_tx(&evt.cfg, at, pdu, isr_tx, lll);
}

/* End of a PDU: the chain PDU that its AuxPtr points to is sent back to back,
 * or the event is done.
 */
static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_adv_sync *lll = param;

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	if ((e->status == BSR_STATUS_OK) && evt.pdu_chain) {
		struct pdu_adv *pdu;

		pdu = lll_adv_pdu_linked_next_get(evt.pdu_chain);
		if (pdu) {
			/* On the channel in the AuxPtr of the PDU sent */
			evt.cfg.chan = aux_ptr_get(evt.pdu_chain)->chan_idx;

			pdu_tx(lll, pdu, e->ts_end + EVENT_SYNC_B2B_MAFS_US);

			return;
		}
	}
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

	isr_done(e, lll);
}

/* End of the periodic advertising event */
static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_adv_sync *lll = param;

	ARG_UNUSED(e);

	/* Have the ULL remove the Channel Map Update Indication from the ACAD */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(lll->event_counter, lll->chm_instant)) {
		struct node_rx_pdu *rx;

		rx = ull_pdu_rx_alloc();
		LL_ASSERT_ERR(rx);

		rx->hdr.type = NODE_RX_TYPE_SYNC_CHM_COMPLETE;
		rx->rx_ftr.param = lll;

		ull_rx_put_sched(rx->hdr.link, rx);
	}

	lll_isr_cleanup(lll);
}
