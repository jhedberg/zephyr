/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Auxiliary channel scanning of the BabbleSim LLL.
 *
 * An auxiliary PDU that an AuxPtr points to is received in an auxiliary scan
 * event that the ULL schedules, or when it is too soon for that, in the
 * radio event of the received PDU: the scan event, or the auxiliary scan
 * event receiving a chain. The scan event then goes back to scanning the
 * primary channel when the chain ends. An active scanner sends an
 * AUX_SCAN_REQ tIFS after a scannable AUX_ADV_IND, and an initiator an
 * AUX_CONNECT_REQ tIFS after a connectable one, before listening for the
 * AUX_SCAN_RSP or AUX_CONNECT_RSP tIFS later.
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

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_df_types.h"
#include "lll_scan.h"
#include "lll_scan_aux.h"
#include "lll_conn.h"
#include "lll_filter.h"
#include "lll_sched.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_addr.h"
#include "lll_scan_internal.h"

#include "ll_feat.h"

#include "hal/debug.h"

static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_rx(const struct bsr_evt *e, void *param);
static void isr_tx_scan_req(const struct bsr_evt *e, void *param);
static void isr_done(const struct bsr_evt *e, void *param);
static void isr_rx_close(int err);
static int isr_rx_pdu(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
		      const struct bsr_evt *e, struct node_rx_pdu *node_rx, struct pdu_adv *pdu,
		      const struct lll_addr_match *match, uint8_t rl_idx);
#if defined(CONFIG_BT_CENTRAL)
static void isr_tx_connect_req(const struct bsr_evt *e, void *param);
static void isr_rx_connect_rsp(const struct bsr_evt *e, void *param);
static void conn_rx_release(struct lll_scan *lll);
#endif /* CONFIG_BT_CENTRAL */

/* State of the current auxiliary PDU reception */
static struct {
	struct bsr_pkt_cfg cfg;

	/* Filter and resolution of the AdvA of received PDUs */
	const struct lll_filter *filter;
	bool resolve;

	/* Scan context, and the auxiliary context of the current auxiliary
	 * scan event, NULL when the reception was started by the scan event
	 */
	struct lll_scan *lll;
	struct lll_scan_aux *lll_aux;

	/* PHY of the auxiliary PDUs, PHY_1M or PHY_2M */
	uint8_t phy;

	/* Reference of the times reported to the ULL */
	uint32_t ticks_ref;

	/* The AUX_SCAN_REQ or AUX_CONNECT_REQ sent */
	struct pdu_adv pdu_tx;

#if defined(CONFIG_BT_CENTRAL)
	/* Connection established on the AUX_CONNECT_REQ sent, held until the
	 * AUX_CONNECT_RSP is received.
	 */
	struct node_rx_pdu *node_conn_rx;
#endif /* CONFIG_BT_CENTRAL */
} evt;

/* PDUs reported to the ULL in the auxiliary scan event. Without any, the ULL
 * gets a done event to release the auxiliary context.
 */
static uint16_t trx_cnt;

int lll_scan_aux_init(void)
{
	return 0;
}

int lll_scan_aux_reset(void)
{
	return 0;
}

void lll_scan_aux_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

/* AuxPtr of a received PDU, NULL if it has none or it does not fit in the
 * extended header.
 */
static const struct pdu_adv_aux_ptr *aux_ptr_get(const struct pdu_adv *pdu)
{
	const struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	const struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	uint8_t offset;

	if ((pdu->len < PDU_AC_EXT_HEADER_SIZE_MIN) || !com_hdr->ext_hdr_len ||
	    (pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN + com_hdr->ext_hdr_len)) || !hdr->aux_ptr) {
		return NULL;
	}

	offset = sizeof(*hdr);
	if (hdr->adv_addr) {
		offset += BDADDR_SIZE;
	}
	if (hdr->tgt_addr) {
		offset += BDADDR_SIZE;
	}
	if (hdr->cte_info) {
		offset += sizeof(struct pdu_cte_info);
	}
	if (hdr->adi) {
		offset += sizeof(struct pdu_adv_adi);
	}

	if ((offset + sizeof(struct pdu_adv_aux_ptr)) > com_hdr->ext_hdr_len) {
		return NULL;
	}

	return (const void *)&com_hdr->ext_hdr_adv_data[offset];
}

/* Radio configuration of the auxiliary PDU receptions */
static void cfg_set(struct lll_scan *lll, uint8_t phy, uint8_t chan)
{
	evt.cfg.aa = PDU_AC_ACCESS_ADDR;
	evt.cfg.crc_init = PDU_AC_CRC_IV;
	evt.cfg.phy = lll_radio_phy(phy);
	evt.cfg.chan = chan;
	evt.cfg.max_len = LL_EXT_OCTETS_RX_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	lll_scan_filter_get(lll, &evt.filter, &evt.resolve);
}

/* Listen for an auxiliary PDU from time start */
static void rx(uint32_t start, uint32_t window_us)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll_radio_rx(&evt.cfg, start, window_us, node_rx->pdu, isr_rx, NULL);
}

bool lll_scan_aux_setup(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
			const struct pdu_adv *pdu, uint8_t phy, uint32_t pdu_start_us,
			uint32_t ticks_ref)
{
	const struct pdu_adv_aux_ptr *aux_ptr;
	uint32_t window_widening_us;
	uint32_t window_size_us;
	uint32_t aux_offset_us;
	uint32_t overhead_us;
	uint32_t pdu_us;
	uint8_t phy_aux;

	aux_ptr = aux_ptr_get(pdu);
	if (!aux_ptr || !PDU_ADV_AUX_PTR_OFFSET_GET(aux_ptr) ||
	    (aux_ptr->chan_idx >= CHM_USED_COUNT_MAX)) {
		return false;
	}

	/* The radio model has no Coded PHY */
	switch (PDU_ADV_AUX_PTR_PHY_GET(aux_ptr)) {
	case EXT_ADV_AUX_PHY_LE_1M:
		phy_aux = PHY_1M;
		break;
	case EXT_ADV_AUX_PHY_LE_2M:
		phy_aux = PHY_2M;
		break;
	default:
		return false;
	}

	window_size_us = aux_ptr->offs_units ? OFFS_UNIT_300_US : OFFS_UNIT_30_US;
	aux_offset_us = (uint32_t)PDU_ADV_AUX_PTR_OFFSET_GET(aux_ptr) * window_size_us;

	pdu_us = PDU_AC_US(pdu->len, phy, PHY_FLAGS_S8);
	if (!AUX_OFFSET_IS_VALID(aux_offset_us, window_size_us, pdu_us)) {
		return false;
	}

	if (aux_ptr->ca) {
		window_widening_us = SCA_DRIFT_50_PPM_US(aux_offset_us);
	} else {
		window_widening_us = SCA_DRIFT_500_PPM_US(aux_offset_us);
	}

	/* The ULL schedules the reception if it has the time to, with the same
	 * margins as when it decides to.
	 */
	overhead_us = pdu_us + window_widening_us + EVENT_TICKER_RES_MARGIN_US + EVENT_JITTER_US +
		      ((EVENT_OVERHEAD_END_US + EVENT_OVERHEAD_START_US +
			HAL_TICKER_TICKS_TO_US(HAL_TICKER_CNTR_CMP_OFFSET_MIN)) << 1);
	if (aux_offset_us > overhead_us) {
		return false;
	}

	trx_cnt = 0U;

	evt.lll = lll;
	evt.lll_aux = lll_aux;
	evt.phy = phy_aux;
	evt.ticks_ref = ticks_ref;
	cfg_set(lll, phy_aux, aux_ptr->chan_idx);

	/* The PDU starts within the offset unit after the aux offset, plus or
	 * minus the clock drift of the advertiser.
	 */
	rx(pdu_start_us + aux_offset_us - window_widening_us - EVENT_JITTER_US,
	   ((window_widening_us + EVENT_JITTER_US) << 1) + window_size_us + addr_us_get(phy_aux));

	return true;
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_scan_aux *lll_aux;
	struct lll_scan *lll;
	uint32_t start_us;
	int err;

	DEBUG_RADIO_START_O(1);

	lll_aux = p->param;
	lll = ull_scan_aux_lll_parent_get(lll_aux, NULL);

	trx_cnt = 0U;

	evt.lll = lll;
	evt.lll_aux = lll_aux;
	evt.phy = lll_aux->phy;
#if defined(CONFIG_BT_CENTRAL)
	evt.node_conn_rx = NULL;

	/* Not started if stopped on connection establishment race between
	 * LLL and ULL.
	 */
	if (unlikely(lll->is_stop || (lll->conn && (lll->conn->central.initiated ||
						    lll->conn->central.cancelled)))) {
		lll_radio_stop(isr_done, lll_aux);

		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	if (lll_preempt_calc(p)) {
		lll_radio_stop(isr_done, lll_aux);

		return -ECANCELED;
	}

	/* Initialize scanning state */
	lll_aux->state = 0U;

	cfg_set(lll, lll_aux->phy, lll_aux->chan);

	/* The event starts at the start of the window the ULL scheduled */
	start_us = lll_event_start_get(p, &evt.ticks_ref);
	rx(start_us, lll_aux->window_size_us + addr_us_get(lll_aux->phy));

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_SCHED_ADVANCED)
	/* Get the offset, from this event, of the free time space after the
	 * other central connections where the first connection event is to be
	 * placed.
	 */
	if (lll->conn) {
		static memq_link_t link;
		static struct mayfly mfy_after_cen_offset_get = {
			0U, 0U, &link, NULL, ull_sched_mfy_after_cen_offset_get};
		struct lll_prepare_param *prepare_param;
		uint32_t ret;

		/* As for a scan event, the scan context is given */
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

	err = lll_prepare_done(lll_aux);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_O(1);

	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	/* An auxiliary scan event is not resumed */
	ARG_UNUSED(resume_cb);
	ARG_UNUSED(curr);

#if defined(CONFIG_BT_CENTRAL)
	/* Complete the connection establishment once the AUX_CONNECT_REQ has
	 * been sent.
	 */
	if (evt.node_conn_rx) {
		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	/* The scan event that is next continues after this event */
	if (next && ull_scan_lll_is_valid_get(next)) {
		return 0;
	}

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct event_done_extra *e;
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

	e = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN_AUX);
	LL_ASSERT_ERR(e);

#if defined(CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS)
	e->lll = param;
#endif /* CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS */

	lll_done(param);
}

/* End of the auxiliary scan event */
static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_scan_aux *lll_aux = param;
	struct lll_scan *lll;

	ARG_UNUSED(e);

	lll = ull_scan_aux_lll_parent_get(lll_aux, NULL);

#if defined(CONFIG_BT_CENTRAL)
	/* No AUX_CONNECT_RSP to the AUX_CONNECT_REQ sent */
	if (evt.node_conn_rx) {
		conn_rx_release(lll);
	}
#endif /* CONFIG_BT_CENTRAL */

	if (!trx_cnt) {
		struct event_done_extra *extra;

		extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN_AUX);
		LL_ASSERT_ERR(extra);

#if defined(CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS)
		extra->lll = lll_aux;
#endif /* CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS */
	}

	/* Tail chain the LLL disable of any scan event in the pipeline if the
	 * scan role is to be stopped, on connection setup or when the scan
	 * duration has expired.
	 */
	if (lll->is_stop != 0U) {
		static memq_link_t link;
		static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};
		uint32_t ret;

		mfy.param = lll;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
		LL_ASSERT_ERR(!ret);
	}

	lll_isr_cleanup(lll_aux);
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = evt.lll;
	struct node_rx_pdu *node_rx;
	struct lll_addr_match match;
	struct pdu_adv *pdu;
	uint8_t rl_idx;
	int err;

	ARG_UNUSED(param);

	/* No PDU, or one with errors */
	if (e->status != BSR_STATUS_OK) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (!node_rx) {
		err = -ENOBUFS;

		goto isr_rx_do_close;
	}

	pdu = (void *)node_rx->pdu;
	if ((pdu->type != PDU_ADV_TYPE_EXT_IND) || !pdu->len) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	/* The filter policy applies to a PDU with an AdvA */
	if (lll_addr_match_ext(pdu, evt.filter, evt.resolve, &match)) {
		rl_idx = lll_scan_rl_idx_get(lll, &match);
		if (!lll_scan_isr_rx_check(lll, match.irkmatch_ok, match.devmatch_ok, rl_idx)) {
			err = -EINVAL;

			goto isr_rx_do_close;
		}
	} else {
		rl_idx = FILTER_IDX_NONE;
	}

	err = isr_rx_pdu(lll, evt.lll_aux, e, node_rx, pdu, &match, rl_idx);
	if (!err) {
		return;
	}

isr_rx_do_close:
	isr_rx_close(err);
}

/* End of the auxiliary PDU receptions: close the auxiliary scan event, or
 * go back to scanning the primary channel. -ECANCELED is the end of a chain
 * that has been reported to the ULL.
 */
static void isr_rx_close(int err)
{
	struct lll_scan *lll = evt.lll;

	if (evt.lll_aux) {
		isr_done(NULL, evt.lll_aux);

		return;
	}

	if (err != -ECANCELED) {
		lll_scan_isr_aux_release(lll);
	}

	lll->is_aux_sched = 0U;

	lll_scan_isr_resume(lll);
}

/* End of an AUX_SCAN_REQ, listen for the AUX_SCAN_RSP */
static void isr_tx_scan_req(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *node_rx;

	ARG_UNUSED(param);

	if (e->status != BSR_STATUS_OK) {
		isr_rx_close(-EINVAL);

		return;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(evt.phy), node_rx->pdu, isr_rx, NULL);
}

static void ftr_fill(struct node_rx_ftr *ftr, const struct bsr_evt *e, uint8_t irkmatch_ok,
		     uint8_t rl_idx, bool dir_report)
{
	ftr->ticks_anchor = evt.ticks_ref;
	ftr->radio_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
	ftr->phy_flags = 0U;
	ftr->rssi = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_PRIVACY)
	ftr->rl_idx = irkmatch_ok ? rl_idx : FILTER_IDX_NONE;
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(irkmatch_ok);
	ARG_UNUSED(rl_idx);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if defined(CONFIG_BT_CTLR_EXT_SCAN_FP)
	ftr->direct = dir_report;
#else /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
	ARG_UNUSED(dir_report);
#endif /* !CONFIG_BT_CTLR_EXT_SCAN_FP */
}

static int isr_rx_pdu(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
		      const struct bsr_evt *e, struct node_rx_pdu *node_rx, struct pdu_adv *pdu,
		      const struct lll_addr_match *match, uint8_t rl_idx)
{
	struct node_rx_ftr *ftr;
	bool dir_report = false;

	if (0) {
#if defined(CONFIG_BT_CENTRAL)
	/* Initiator */
	} else if (lll->conn && !lll->conn->central.cancelled &&
		   (pdu->adv_ext_ind.adv_mode & BT_HCI_LE_ADV_PROP_CONN) &&
		   lll_scan_ext_tgta_check(lll, false, true, pdu, rl_idx, NULL)) {
		/* AUX_CONNECT_REQ is a CONNECT_IND, and AUX_CONNECT_RSP has
		 * AdvA and TargetA in its extended header.
		 */
		const uint8_t aux_connect_req_len = sizeof(struct pdu_adv_connect_ind);
		const uint8_t aux_connect_rsp_len = PDU_AC_EXT_HEADER_SIZE_MIN +
						    sizeof(struct pdu_adv_ext_hdr) + ADVA_SIZE +
						    TARGETA_SIZE;
		uint32_t conn_space_us;
		struct node_rx_pdu *rx;
		struct pdu_adv *pdu_tx;
		struct ull_hdr *ull;
		uint32_t pdu_end_us;
		uint8_t init_tx_addr;
		uint8_t *init_addr;
#if defined(CONFIG_BT_CTLR_PRIVACY)
		bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		/* The ULL has not yet assigned an auxiliary context to the
		 * reception started by the scan event.
		 */
		if (!lll_aux && !lll->lll_aux) {
			return -ECHILD;
		}

		/* A connection on the secondary channel always uses CSA#2,
		 * and 2 nodes are kept for the connection.
		 */
		rx = ull_pdu_rx_alloc_peek(4);
		if (!rx) {
			return -ENOBUFS;
		}

		/* The connection request exchange must end within the scan
		 * event.
		 */
		pdu_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
		if (!lll->ticks_window) {
			uint32_t scan_interval_us;

			scan_interval_us = lll->interval * SCAN_INT_UNIT_US;
			pdu_end_us %= scan_interval_us;
		}
		ull = HDR_LLL2ULL(lll);
		if (pdu_end_us > (HAL_TICKER_TICKS_TO_US(ull->ticks_slot) - EVENT_IFS_US -
				  PDU_AC_MAX_US(aux_connect_req_len, evt.phy) - EVENT_IFS_US -
				  PDU_AC_MAX_US(aux_connect_rsp_len, evt.phy) -
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
		lll_scan_prepare_connect_req(lll, pdu_tx, evt.phy,
					     e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref),
					     pdu->tx_addr,
					     &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET],
					     init_tx_addr, init_addr, &conn_space_us);

		lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_tx, isr_tx_connect_req, NULL);

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

		/* Give the AUX_CONNECT_REQ to the ULL, with CSA#2 */
		(void)memcpy(rx->pdu, pdu_tx,
			     (offsetof(struct pdu_adv, connect_ind) +
			      sizeof(struct pdu_adv_connect_ind)));
		((struct pdu_adv *)rx->pdu)->chan_sel = 1U;

		ftr = &(rx->rx_ftr);
		ftr->param = lll;
		ftr->ticks_anchor = evt.ticks_ref;
		ftr->radio_end_us = conn_space_us;

#if defined(CONFIG_BT_CTLR_PRIVACY)
		ftr->rl_idx = match->irkmatch_ok ? rl_idx : FILTER_IDX_NONE;
		ftr->lrpa_used = lll->rpa_gen && lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		ftr->extra = ull_pdu_rx_alloc();

		/* Hold the connection until the AUX_CONNECT_RSP is received */
		evt.node_conn_rx = rx;

		return 0;
#endif /* CONFIG_BT_CENTRAL */

	/* Active scanner */
	} else if (
#if defined(CONFIG_BT_CENTRAL)
		   !lll->conn &&
#endif /* CONFIG_BT_CENTRAL */
		   lll->type &&
		   ((lll_aux && !lll_aux->state) || (lll->lll_aux && !lll->lll_aux->state)) &&
		   (pdu->adv_ext_ind.adv_mode & BT_HCI_LE_ADV_PROP_SCAN) &&
		   lll_scan_ext_tgta_check(lll, false, false, pdu, rl_idx, &dir_report)) {
		struct pdu_adv *pdu_tx;
#if defined(CONFIG_BT_CTLR_PRIVACY)
		bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		/* 2 nodes for the AUX_ADV_IND and AUX_SCAN_RSP reports, and 2
		 * kept for connections.
		 */
		if (!ull_pdu_rx_alloc_peek(4)) {
			return -ENOBUFS;
		}

		pdu_tx = &evt.pdu_tx;
		pdu_tx->type = PDU_ADV_TYPE_AUX_SCAN_REQ;
		pdu_tx->rfu = 0U;
		pdu_tx->chan_sel = 0U;
		pdu_tx->rx_addr = pdu->tx_addr;
		pdu_tx->len = sizeof(struct pdu_adv_scan_req);
#if defined(CONFIG_BT_CTLR_PRIVACY)
		lrpa = ull_filter_lll_lrpa_get(rl_idx);
		if (lll->rpa_gen && lrpa) {
			pdu_tx->tx_addr = 1;
			(void)memcpy(pdu_tx->scan_req.scan_addr, lrpa->val, BDADDR_SIZE);
		} else {
#else /* !CONFIG_BT_CTLR_PRIVACY */
		if (1) {
#endif /* !CONFIG_BT_CTLR_PRIVACY */
			pdu_tx->tx_addr = lll->init_addr_type;
			(void)memcpy(pdu_tx->scan_req.scan_addr, lll->init_addr, BDADDR_SIZE);
		}
		(void)memcpy(pdu_tx->scan_req.adv_addr,
			     &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], BDADDR_SIZE);

		lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_tx, isr_tx_scan_req, NULL);

		(void)ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_REPORT;

		ftr = &(node_rx->rx_ftr);
		if (lll_aux) {
			ftr->param = lll_aux;
			lll_aux->state = 1U;
		} else {
			ftr->param = lll;
			ftr->lll_aux = lll->lll_aux;
			lll->lll_aux->state = 1U;
		}
		ftr_fill(ftr, e, match->irkmatch_ok, rl_idx, dir_report);
		ftr->scan_req = 1U;
		ftr->scan_rsp = 0U;
		ftr->aux_lll_sched = 0U;

		ull_rx_put_sched(node_rx->hdr.link, node_rx);

		return 0;

	/* Passive scanner, scan responses or chain PDUs */
	} else if (
#if defined(CONFIG_BT_CENTRAL)
		   !lll->conn &&
#endif /* CONFIG_BT_CENTRAL */
		   ((lll_aux && lll_aux->is_chain_sched) ||
		    (lll->lll_aux && lll->lll_aux->is_chain_sched) ||
		    lll_scan_ext_tgta_check(lll, false, false, pdu, rl_idx, &dir_report))) {
		ftr = &(node_rx->rx_ftr);
		if (lll_aux) {
			/* Received in an auxiliary scan event */
			ftr->param = lll_aux;
			ftr->scan_rsp = lll_aux->state;

			/* Further auxiliary PDUs are chain PDUs */
			lll_aux->is_chain_sched = 1U;

			/* The ULL associates the auxiliary context with the
			 * scan context again if it is to be received in this
			 * event.
			 */
			lll->lll_aux = NULL;
		} else if (lll->lll_aux) {
			/* Received in the scan event */
			ftr->param = lll;
			ftr->lll_aux = lll->lll_aux;
			ftr->scan_rsp = lll->lll_aux->state;

			/* Further auxiliary PDUs are chain PDUs */
			lll->lll_aux->is_chain_sched = 1U;
		} else {
			/* The ULL has not yet assigned an auxiliary context
			 * to the reception started by the scan event.
			 */
			return -ECHILD;
		}

		/* Allocated before the next reception is started, which uses
		 * the next free node rx.
		 */
		(void)ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_REPORT;

		ftr_fill(ftr, e, match->irkmatch_ok, rl_idx, dir_report);
		ftr->scan_req = 0U;
		ftr->aux_lll_sched = lll_scan_aux_setup(lll, lll_aux, pdu, evt.phy, e->ts_start,
							evt.ticks_ref);

		ull_rx_put_sched(node_rx->hdr.link, node_rx);

		/* The next auxiliary PDU is received in this radio event */
		if (ftr->aux_lll_sched) {
			if (!lll_aux) {
				lll->is_aux_sched = 1U;
			}

			return 0;
		}

		/* A valid PDU has been reported to the ULL */
		trx_cnt++;

		return -ECANCELED;
	}

	return -EINVAL;
}

#if defined(CONFIG_BT_CENTRAL)
/* End of an AUX_CONNECT_REQ, listen for the AUX_CONNECT_RSP */
static void isr_tx_connect_req(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *node_rx;

	ARG_UNUSED(param);

	if (e->status != BSR_STATUS_OK) {
		isr_rx_connect_rsp(e, NULL);

		return;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(evt.phy), node_rx->pdu, isr_rx_connect_rsp, NULL);
}

static bool isr_rx_connect_rsp_check(const struct lll_scan *lll, const struct pdu_adv *pdu_tx,
				     const struct pdu_adv *pdu_rx, uint8_t rl_idx)
{
	if ((pdu_rx->type != PDU_ADV_TYPE_AUX_CONNECT_RSP) ||
	    (pdu_rx->len != (offsetof(struct pdu_adv_com_ext_adv, ext_hdr_adv_data) +
			     offsetof(struct pdu_adv_ext_hdr, data) + ADVA_SIZE + TARGETA_SIZE)) ||
	    pdu_rx->adv_ext_ind.adv_mode || !pdu_rx->adv_ext_ind.ext_hdr.adv_addr ||
	    !pdu_rx->adv_ext_ind.ext_hdr.tgt_addr) {
		return false;
	}

	return lll_scan_adva_check(lll, pdu_rx->tx_addr,
				   &pdu_rx->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], rl_idx) &&
	       (pdu_rx->rx_addr == pdu_tx->tx_addr) &&
	       !memcmp(&pdu_rx->adv_ext_ind.ext_hdr.data[TGTA_OFFSET],
		       pdu_tx->connect_ind.init_addr, BDADDR_SIZE);
}

/* Give up the connection established on the AUX_CONNECT_REQ sent, and go on
 * initiating.
 */
static void conn_rx_release(struct lll_scan *lll)
{
	struct node_rx_pdu *rx = evt.node_conn_rx;
	struct node_rx_pdu *rx_extra;

	evt.node_conn_rx = NULL;

	lll->conn->central.initiated = 0U;
	lll->is_stop = 0U;

	rx_extra = rx->rx_ftr.extra;

	rx->hdr.type = NODE_RX_TYPE_RELEASE;
	ull_rx_put(rx->hdr.link, rx);

	rx_extra->hdr.type = NODE_RX_TYPE_RELEASE;
	ull_rx_put_sched(rx_extra->hdr.link, rx_extra);
}

static void isr_rx_connect_rsp(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = evt.lll;
	struct lll_addr_match match;
	struct node_rx_pdu *rx;
	struct pdu_adv *pdu_rx;
	uint8_t rl_idx;

	ARG_UNUSED(param);

	rx = evt.node_conn_rx;
	LL_ASSERT_DBG(rx);

	pdu_rx = NULL;
	rl_idx = FILTER_IDX_NONE;
	(void)memset(&match, 0, sizeof(match));

	if (e->status == BSR_STATUS_OK) {
		struct node_rx_pdu *node_rx;

		node_rx = ull_pdu_rx_alloc_peek(1);
		LL_ASSERT_DBG(node_rx);

		pdu_rx = (void *)node_rx->pdu;

		/* Resolve the AdvA of the advertiser, which can differ from
		 * the one of the AUX_ADV_IND.
		 */
		(void)lll_addr_match_ext(pdu_rx, NULL, evt.resolve, &match);
		rl_idx = lll_scan_rl_idx_get(lll, &match);

		if (!isr_rx_connect_rsp_check(lll, &evt.pdu_tx, pdu_rx, rl_idx)) {
			pdu_rx = NULL;
		}
	}

	if (!pdu_rx) {
		/* Try again with connection initiation */
		conn_rx_release(lll);

		goto isr_rx_connect_rsp_do_close;
	}

	evt.node_conn_rx = NULL;

#if defined(CONFIG_BT_CTLR_PHY)
	/* The connection starts on the PHY of the AUX_CONNECT_REQ */
	struct lll_conn *conn_lll = lll->conn;

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	conn_lll->dle.eff.max_tx_time = MAX(conn_lll->dle.eff.max_tx_time,
					    PDU_DC_MAX_US(PDU_DC_PAYLOAD_SIZE_MIN, evt.phy));
	conn_lll->dle.eff.max_rx_time = MAX(conn_lll->dle.eff.max_rx_time,
					    PDU_DC_MAX_US(PDU_DC_PAYLOAD_SIZE_MIN, evt.phy));
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */

	conn_lll->phy_tx = evt.phy;
	conn_lll->phy_tx_time = evt.phy;
	conn_lll->phy_flags = 0U;
	conn_lll->phy_rx = evt.phy;
#endif /* CONFIG_BT_CTLR_PHY */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (match.irkmatch_ok) {
		struct pdu_adv *pdu;

		/* The peer address is the resolved AdvA */
		pdu = (void *)rx->pdu;
		pdu->rx_addr = pdu_rx->tx_addr;
		(void)memcpy(pdu->connect_ind.adv_addr,
			     &pdu_rx->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], BDADDR_SIZE);
		rx->rx_ftr.rl_idx = rl_idx;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	ull_rx_put_sched(rx->hdr.link, rx);

isr_rx_connect_rsp_do_close:
	if (evt.lll_aux) {
		isr_done(NULL, evt.lll_aux);

		return;
	}

	lll->is_aux_sched = 0U;
	lll_scan_isr_aux_release(lll);
	lll_scan_isr_resume(lll);
}
#endif /* CONFIG_BT_CENTRAL */
