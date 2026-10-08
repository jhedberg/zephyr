/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
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

static void isr_tx(const struct bsr_evt *e, void *param);

static struct {
	struct bsr_pkt_cfg cfg;

	/* For the chain PDU linked to it, NULL if none follows back to back */
	struct pdu_adv *pdu_chain;
} evt;

static bool is_instant_or_past(uint16_t event_counter, uint16_t instant)
{
	uint16_t instant_latency;

	instant_latency = (event_counter - instant) & EVENT_INSTANT_MAX;

	return instant_latency <= EVENT_INSTANT_LATENCY_MAX;
}

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
static struct pdu_adv_aux_ptr *aux_ptr_get(struct pdu_adv *pdu)
{
	struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	uint8_t *dptr;

	if ((com_hdr->ext_hdr_len == 0U) || (hdr->aux_ptr == 0U)) {
		return NULL;
	}

	/* No AdvA and TargetA, which are RFU in periodic advertising PDUs */
	dptr = hdr->data;
	if (hdr->cte_info != 0U) {
		dptr += sizeof(struct pdu_cte_info);
	}
	if (hdr->adi != 0U) {
		dptr += sizeof(struct pdu_adv_adi);
	}

	return (void *)dptr;
}
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

/* The new channel map is used from the instant on, when the ULL also removes
 * the Channel Map Update Indication from the ACAD. Both are done together, so
 * that the ULL is signalled exactly once.
 */
static void chm_switch(struct lll_adv_sync *lll)
{
	struct node_rx_pdu *rx;

	lll->chm_first = lll->chm_last;

	rx = ull_pdu_rx_alloc();
	LL_ASSERT_ERR(rx != NULL);

	rx->hdr.type = NODE_RX_TYPE_SYNC_CHM_COMPLETE;
	rx->rx_ftr.param = lll;

	ull_rx_put_sched(rx->hdr.link, rx);
}

static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_adv_sync *lll = param;

	ARG_UNUSED(e);

	/* At the end of the event before the instant, so that the PDU of the
	 * event of the instant no longer has the indication.
	 */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(lll->event_counter, lll->chm_instant)) {
		chm_switch(lll);
	}

	lll_isr_cleanup(lll);
}

static void pdu_tx(struct lll_adv_sync *lll, struct pdu_adv *pdu, uint32_t at)
{
#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	struct pdu_adv_aux_ptr *aux_ptr;

	aux_ptr = aux_ptr_get(pdu);
	if (aux_ptr != NULL) {
		/* The radio model reads the PDU when its transmission starts,
		 * so the channel of the chain PDU can still be filled in.
		 */
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

static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_adv_sync *lll = param;

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
	if ((e->status == BSR_STATUS_OK) && (evt.pdu_chain != NULL)) {
		struct pdu_adv *pdu;

		pdu = lll_adv_pdu_linked_next_get(evt.pdu_chain);
		if (pdu != NULL) {
			/* On the channel in the AuxPtr of the PDU sent */
			evt.cfg.chan = aux_ptr_get(evt.pdu_chain)->chan_idx;

			pdu_tx(lll, pdu, e->ts_end + EVENT_SYNC_B2B_MAFS_US);

			return;
		}
	}
#endif /* CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK */

	isr_done(e, lll);
}

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

	lll->latency_event = lll->latency_prepare + p->lazy;
	lll->latency_prepare = 0U;

	event_counter = lll->event_counter + lll->latency_event;
	lll->event_counter = event_counter + 1U;

	/* The event before the instant does it, unless it was skipped or
	 * aborted.
	 */
	if ((lll->chm_first != lll->chm_last) &&
	    is_instant_or_past(event_counter, lll->chm_instant)) {
		chm_switch(lll);
	}

	chan_idx = lll_chan_sel_2(event_counter, lll->data_chan_id,
				  lll->chm[lll->chm_first].data_chan_map,
				  lll->chm[lll->chm_first].data_chan_count);

	upd = 0U;
	pdu = lll_adv_sync_data_latest_get(lll, NULL, &upd);
	LL_ASSERT_DBG(pdu != NULL);

#if defined(CONFIG_BT_CTLR_ADV_ISO) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	/* With the expiry information of the BIG ticker, the LLL fills in the
	 * BIGInfo rather than the ULL.
	 */
	if (lll->iso != NULL) {
		ull_adv_iso_lll_biginfo_fill(pdu, lll);
	}
#endif /* CONFIG_BT_CTLR_ADV_ISO && CONFIG_BT_TICKER_EXT_EXPIRE_INFO */

	if (lll_preempt_calc(p) != 0U) {
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
	LL_ASSERT_ERR(err == 0);

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

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	/* The event counter of the next event accounts for the events aborted
	 * in the prepare pipeline.
	 */
	lll = prepare_param->param;
	lll->latency_prepare += (prepare_param->lazy + 1U);

	lll_done(param);
}

void lll_adv_sync_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}
