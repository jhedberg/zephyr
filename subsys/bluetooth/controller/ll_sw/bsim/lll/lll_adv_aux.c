/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Auxiliary advertising events of the BabbleSim LLL.
 *
 * An auxiliary advertising event transmits, on the secondary PHY, the
 * AUX_ADV_IND that the AuxPtr of the ADV_EXT_IND PDUs of the primary
 * advertising event points to, followed back to back by the AUX_CHAIN_IND
 * PDUs of its chain. After a connectable or scannable AUX_ADV_IND, the
 * advertiser listens tIFS later for an AUX_CONNECT_REQ, answered with an
 * AUX_CONNECT_RSP, or for an AUX_SCAN_REQ, answered with the AUX_SCAN_RSP and
 * the chain PDUs that follow it.
 *
 * The channel of a chain PDU is set in the AuxPtr of the PDU before it, before
 * that PDU is transmitted.
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
#include "lll_adv_aux.h"
#include "lll_adv_sync.h"
#include "lll_filter.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_radio.h"
#include "lll_addr.h"
#include "lll_adv_internal.h"

#include "ull_adv_types.h"

#include "hal/debug.h"

static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void pdu_tx(struct lll_adv_aux *lll_aux, struct pdu_adv *pdu, uint32_t at,
		   lll_radio_cb_t cb);
static void isr_tx(const struct bsr_evt *e, void *param);
static void isr_tx_chain(const struct bsr_evt *e, void *param);
static void isr_rx(const struct bsr_evt *e, void *param);
static void isr_done(const struct bsr_evt *e, void *param);
static void isr_abort(const struct bsr_evt *e, void *param);
static int isr_rx_pdu(struct lll_adv_aux *lll_aux, const struct bsr_evt *e,
		      struct pdu_adv *pdu_rx, const struct lll_addr_match *match);
#if defined(CONFIG_BT_PERIPHERAL)
static void isr_tx_connect_rsp(const struct bsr_evt *e, void *param);
static void conn_rx_release(struct node_rx_pdu *rx);
#endif /* CONFIG_BT_PERIPHERAL */

/* State of the current auxiliary advertising event */
static struct {
	struct bsr_pkt_cfg cfg;

	/* Filter and resolution of the address of a received AUX_SCAN_REQ or
	 * AUX_CONNECT_REQ
	 */
	const struct lll_filter *filter;
	bool resolve;

	/* Reference of the event times reported to the ULL */
	uint32_t ticks_ref;

	/* The PDU being transmitted, when its AuxPtr points to a chain PDU
	 * that follows it.
	 */
	struct pdu_adv *pdu_chain;

#if defined(CONFIG_BT_PERIPHERAL)
	/* The AUX_CONNECT_RSP sent */
	struct pdu_adv pdu_tx;

	/* Connection established on the AUX_CONNECT_REQ received, held until
	 * the AUX_CONNECT_RSP has been sent.
	 */
	struct node_rx_pdu *node_conn_rx;
#endif /* CONFIG_BT_PERIPHERAL */
} evt;

int lll_adv_aux_init(void)
{
	return 0;
}

int lll_adv_aux_reset(void)
{
	return 0;
}

void lll_adv_aux_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

/* AuxPtr of an extended advertising PDU, NULL if it has none */
static struct pdu_adv_aux_ptr *aux_ptr_get(struct pdu_adv *pdu)
{
	struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	uint8_t *dptr;

	if (!com_hdr->ext_hdr_len || !hdr->aux_ptr) {
		return NULL;
	}

	dptr = hdr->data;
	if (hdr->adv_addr) {
		dptr += BDADDR_SIZE;
	}
	if (hdr->tgt_addr) {
		dptr += BDADDR_SIZE;
	}

	/* No CTEInfo in the primary and secondary channel PDUs */

	if (hdr->adi) {
		dptr += sizeof(struct pdu_adv_adi);
	}

	return (void *)dptr;
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct pdu_adv_com_ext_adv *com_hdr;
	struct lll_adv_aux *lll_aux;
	struct lll_adv *lll;
	struct pdu_adv *pdu;
	uint32_t start_us;
	uint8_t chan_idx;
	uint8_t upd;
	int err;

	DEBUG_RADIO_START_A(1);

	lll_aux = p->param;
	lll = lll_aux->adv;

#if defined(CONFIG_BT_PERIPHERAL)
	evt.node_conn_rx = NULL;

	/* Not started if stopped on connection establishment, or when being
	 * disabled while connectable.
	 */
	if (unlikely(lll->conn && (lll->conn->periph.initiated || lll->conn->periph.cancelled))) {
		lll_event_abort(lll_aux);

		return 0;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	upd = 0U;
	pdu = lll_adv_aux_data_latest_get(lll_aux, &upd);
	LL_ASSERT_DBG(pdu);

#if defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	struct ll_adv_aux_set *aux = HDR_LLL2ULL(lll_aux);

	/* The same channel as in the AuxPtr filled by the primary advertising
	 * event.
	 */
	chan_idx = lll_chan_sel_2(lll_aux->data_chan_counter, aux->data_chan_id,
				  aux->chm[aux->chm_first].data_chan_map,
				  aux->chm[aux->chm_first].data_chan_count);
#else /* !CONFIG_BT_TICKER_EXT_EXPIRE_INFO */
	struct pdu_adv_aux_ptr *aux_ptr;

	/* The channel in the AuxPtr of the primary channel PDUs */
	aux_ptr = aux_ptr_get(lll_adv_data_curr_get(lll));
	if (unlikely(!aux_ptr || !PDU_ADV_AUX_PTR_OFFSET_GET(aux_ptr))) {
		lll_radio_stop(isr_done, lll_aux);

		return 0;
	}

	chan_idx = aux_ptr->chan_idx;
#endif /* !CONFIG_BT_TICKER_EXT_EXPIRE_INFO */

	/* Next channel index calculation */
	lll_aux->data_chan_counter++;

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	/* Offset and event counter of the periodic advertising event that the
	 * SyncInfo points to.
	 */
	if (pdu->adv_ext_ind.ext_hdr_len && pdu->adv_ext_ind.ext_hdr.sync_info) {
		ull_adv_sync_lll_syncinfo_fill(pdu, lll_aux);
	}
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC && CONFIG_BT_TICKER_EXT_EXPIRE_INFO */

	if (lll_preempt_calc(p)) {
		lll_radio_stop(isr_done, lll_aux);

		return -ECANCELED;
	}

	evt.cfg.aa = PDU_AC_ACCESS_ADDR;
	evt.cfg.crc_init = PDU_AC_CRC_IV;
	evt.cfg.phy = lll_radio_phy(lll->phy_s);
	evt.cfg.chan = chan_idx;
	evt.cfg.max_len = PDU_AC_PAYLOAD_SIZE_MAX;
#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	evt.cfg.tx_power = lll->tx_pwr_lvl;
#else /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */
	evt.cfg.tx_power = RADIO_TXP_DEFAULT;
#endif /* !CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

	lll_adv_filter_get(lll, &evt.filter, &evt.resolve);

	start_us = lll_event_start_get(p, &evt.ticks_ref);

	com_hdr = &pdu->adv_ext_ind;
	if (com_hdr->adv_mode & (BT_HCI_LE_ADV_PROP_CONN | BT_HCI_LE_ADV_PROP_SCAN)) {
		struct pdu_adv *scan_pdu;

		scan_pdu = lll_adv_scan_rsp_latest_get(lll, &upd);
		LL_ASSERT_DBG(scan_pdu);

#if defined(CONFIG_BT_CTLR_PRIVACY)
		if (upd) {
			/* The AUX_SCAN_RSP has the AdvA of the AUX_ADV_IND */
			(void)memcpy(&scan_pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET],
				     &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], BDADDR_SIZE);
		}
#else /* !CONFIG_BT_CTLR_PRIVACY */
		ARG_UNUSED(scan_pdu);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

		/* An AUX_ADV_IND that can be answered has no chain */
		evt.pdu_chain = NULL;
		lll_radio_tx(&evt.cfg, start_us, pdu, isr_tx, lll_aux);
	} else {
		pdu_tx(lll_aux, pdu, start_us, isr_tx_chain);
	}

	err = lll_prepare_done(lll_aux);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_A(1);

	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	/* An auxiliary advertising event is not resumed */
	ARG_UNUSED(next);
	ARG_UNUSED(curr);
	ARG_UNUSED(resume_cb);

#if defined(CONFIG_BT_PERIPHERAL)
	/* Complete the connection establishment once the AUX_CONNECT_REQ has
	 * been accepted.
	 */
	if (evt.node_conn_rx) {
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

/* Transmit pdu at time at, and with cb as the end of its chain PDUs if its
 * AuxPtr points to any.
 */
static void pdu_tx(struct lll_adv_aux *lll_aux, struct pdu_adv *pdu, uint32_t at,
		   lll_radio_cb_t cb)
{
	struct pdu_adv_aux_ptr *aux_ptr;

	aux_ptr = aux_ptr_get(pdu);
	if (IS_ENABLED(CONFIG_BT_CTLR_ADV_AUX_PDU_BACK2BACK) && aux_ptr) {
		struct ll_adv_aux_set *aux = HDR_LLL2ULL(lll_aux);

		/* Channel of the chain PDU that follows */
		aux_ptr->chan_idx = lll_chan_sel_2(lll_aux->data_chan_counter, aux->data_chan_id,
						   aux->chm[aux->chm_first].data_chan_map,
						   aux->chm[aux->chm_first].data_chan_count);
		lll_aux->data_chan_counter++;

		evt.pdu_chain = pdu;
	} else {
		evt.pdu_chain = NULL;
	}

	lll_radio_tx(&evt.cfg, at, pdu, cb, lll_aux);
}

/* End of a connectable or scannable AUX_ADV_IND */
static void isr_tx(const struct bsr_evt *e, void *param)
{
	struct lll_adv_aux *lll_aux = param;
	struct node_rx_pdu *node_rx;
	struct lll_adv *lll;

	if (e->status != BSR_STATUS_OK) {
		isr_done(e, lll_aux);

		return;
	}

	/* Listen for an AUX_SCAN_REQ or AUX_CONNECT_REQ */
	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx);

	lll = lll_aux->adv;
	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(lll->phy_s), node_rx->pdu, isr_rx, lll_aux);
}

/* End of a PDU that can be followed by chain PDUs: the next chain PDU is sent
 * back to back, or the event is done.
 */
static void isr_tx_chain(const struct bsr_evt *e, void *param)
{
	struct lll_adv_aux *lll_aux = param;

#if defined(CONFIG_BT_CTLR_ADV_AUX_PDU_BACK2BACK)
	if ((e->status == BSR_STATUS_OK) && evt.pdu_chain) {
		struct pdu_adv_aux_ptr *aux_ptr;
		struct pdu_adv *pdu;

		pdu = lll_adv_pdu_linked_next_get(evt.pdu_chain);
		if (pdu) {
			/* On the channel in the AuxPtr of the PDU sent */
			aux_ptr = aux_ptr_get(evt.pdu_chain);
			evt.cfg.chan = aux_ptr->chan_idx;

			pdu_tx(lll_aux, pdu, e->ts_end + EVENT_B2B_MAFS_US, isr_tx_chain);

			return;
		}
	}
#endif /* CONFIG_BT_CTLR_ADV_AUX_PDU_BACK2BACK */

	isr_done(e, lll_aux);
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_adv_aux *lll_aux = param;

	if (e->status == BSR_STATUS_OK) {
		struct node_rx_pdu *node_rx;
		struct lll_addr_match match;
		struct pdu_adv *pdu_rx;
		int err;

		node_rx = ull_pdu_rx_alloc_peek(1);
		LL_ASSERT_DBG(node_rx);

		pdu_rx = (void *)node_rx->pdu;
		lll_addr_match(pdu_rx, evt.filter, evt.resolve, &match);

		err = isr_rx_pdu(lll_aux, e, pdu_rx, &match);
		if (!err) {
			return;
		}
	}

	isr_done(e, lll_aux);
}

/* End of the auxiliary advertising event, which is also the end of the
 * advertising event for the ULL.
 */
static void isr_done(const struct bsr_evt *e, void *param)
{
	struct event_done_extra *extra;

	ARG_UNUSED(e);

	extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_ADV_AUX);
	LL_ASSERT_ERR(extra);

	lll_isr_cleanup(param);
}

/* End of an aborted auxiliary advertising event */
static void isr_abort(const struct bsr_evt *e, void *param)
{
	ARG_UNUSED(e);

#if defined(CONFIG_BT_PERIPHERAL)
	/* The AUX_CONNECT_RSP has not been sent */
	if (evt.node_conn_rx) {
		struct node_rx_pdu *rx = evt.node_conn_rx;

		evt.node_conn_rx = NULL;
		conn_rx_release(rx);
	}
#endif /* CONFIG_BT_PERIPHERAL */

	lll_isr_cleanup(param);
}

static int isr_rx_pdu(struct lll_adv_aux *lll_aux, const struct bsr_evt *e,
		      struct pdu_adv *pdu_rx, const struct lll_addr_match *match)
{
	struct pdu_adv_ext_hdr *hdr;
	struct pdu_adv *pdu_aux;
	struct lll_adv *lll;
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

	lll = lll_aux->adv;

	/* AdvA, and TargetA if directed, of the AUX_ADV_IND sent */
	pdu_aux = lll_adv_aux_data_curr_get(lll_aux);
	hdr = &pdu_aux->adv_ext_ind.ext_hdr;
	addr = &hdr->data[ADVA_OFFSET];
	tx_addr = pdu_aux->tx_addr;
	if (hdr->tgt_addr) {
		tgt_addr = &hdr->data[TGTA_OFFSET];
	} else {
		tgt_addr = NULL;
	}
	rx_addr = pdu_aux->rx_addr;

	if ((pdu_rx->type == PDU_ADV_TYPE_AUX_SCAN_REQ) &&
	    (pdu_rx->len == sizeof(struct pdu_adv_scan_req)) &&
	    lll_adv_scan_req_check(lll, pdu_rx, tx_addr, addr, match->devmatch_ok, &rl_idx)) {
#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
		if (lll->scan_req_notify) {
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

		/* The AUX_SCAN_RSP, and its chain PDUs */
		pdu_tx(lll_aux, lll_adv_scan_rsp_curr_get(lll), e->ts_end + EVENT_IFS_US,
		       isr_tx_chain);

		return 0;

#if defined(CONFIG_BT_PERIPHERAL)
	/* NOTE: Do not accept an AUX_CONNECT_REQ if the cancelled flag is set
	 *       in thread context when disabling connectable advertising.
	 */
	} else if ((pdu_rx->type == PDU_ADV_TYPE_AUX_CONNECT_REQ) &&
		   (pdu_rx->len == sizeof(struct pdu_adv_connect_ind)) && lll->conn &&
		   !lll->conn->periph.cancelled &&
		   lll_adv_connect_ind_check(lll, pdu_rx, tx_addr, addr, rx_addr, tgt_addr,
					     match->devmatch_ok, &rl_idx)) {
		struct pdu_adv_com_ext_adv *cr_com_hdr;
		struct pdu_adv_ext_hdr *cr_hdr;
		struct node_rx_ftr *ftr;
		struct node_rx_pdu *rx;
		struct pdu_adv *pdu_cr;
		uint8_t *cr_dptr;

		/* A connection on the secondary channel always uses CSA#2 */
		rx = ull_pdu_rx_alloc_peek(4);
		if (!rx) {
			return -ENOBUFS;
		}

		/* The AUX_CONNECT_RSP has the AdvA and the InitA (TargetA) of
		 * the AUX_CONNECT_REQ.
		 */
		pdu_cr = &evt.pdu_tx;
		pdu_cr->type = PDU_ADV_TYPE_AUX_CONNECT_RSP;
		pdu_cr->rfu = 0U;
		pdu_cr->chan_sel = 0U;
		pdu_cr->tx_addr = pdu_rx->rx_addr;
		pdu_cr->rx_addr = pdu_rx->tx_addr;

		cr_com_hdr = &pdu_cr->adv_ext_ind;
		cr_com_hdr->adv_mode = 0U;

		cr_hdr = &cr_com_hdr->ext_hdr;
		cr_dptr = (void *)cr_hdr;
		*cr_dptr = 0U;
		cr_dptr = cr_hdr->data;

		cr_hdr->adv_addr = 1U;
		(void)memcpy(cr_dptr, pdu_rx->connect_ind.adv_addr, BDADDR_SIZE);
		cr_dptr += BDADDR_SIZE;

		cr_hdr->tgt_addr = 1U;
		(void)memcpy(cr_dptr, pdu_rx->connect_ind.init_addr, BDADDR_SIZE);
		cr_dptr += BDADDR_SIZE;

		cr_com_hdr->ext_hdr_len = cr_dptr - (uint8_t *)&cr_com_hdr->ext_hdr;
		pdu_cr->len = cr_dptr - &pdu_cr->payload[0];

		lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, pdu_cr, isr_tx_connect_rsp,
			     lll_aux);

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		lll->conn->rssi_latest = lll_rssi_get(e->rssi);
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

		/* The AUX_CONNECT_REQ is in the node rx PDU */
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

		ftr->extra = ull_pdu_rx_alloc();

		/* Given to the ULL once the AUX_CONNECT_RSP has been sent */
		evt.node_conn_rx = rx;

		return 0;
#endif /* CONFIG_BT_PERIPHERAL */
	}

	return -EINVAL;
}

#if defined(CONFIG_BT_PERIPHERAL)
/* End of the AUX_CONNECT_RSP, which closes the event */
static void isr_tx_connect_rsp(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *rx = evt.node_conn_rx;
	struct lll_adv_aux *lll_aux = param;
	struct lll_adv *lll = lll_aux->adv;

	evt.node_conn_rx = NULL;

	if (e->status == BSR_STATUS_OK) {
		/* Stop further LLL radio events */
		lll->conn->periph.initiated = 1U;

		ull_rx_put_sched(rx->hdr.link, rx);
	} else {
		/* Not connected, go on advertising */
		conn_rx_release(rx);
	}

	lll_isr_cleanup(lll_aux);
}

static void conn_rx_release(struct node_rx_pdu *rx)
{
	struct node_rx_pdu *rx_extra = rx->rx_ftr.extra;

	rx->hdr.type = NODE_RX_TYPE_RELEASE;
	ull_rx_put(rx->hdr.link, rx);

	rx_extra->hdr.type = NODE_RX_TYPE_RELEASE;
	ull_rx_put_sched(rx_extra->hdr.link, rx_extra);
}
#endif /* CONFIG_BT_PERIPHERAL */
