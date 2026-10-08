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

/* AdvA and TargetA are the only fields of an AUX_CONNECT_RSP */
#define AUX_CONNECT_RSP_LEN (PDU_AC_EXT_HEADER_SIZE_MIN + sizeof(struct pdu_adv_ext_hdr) + \
			     ADVA_SIZE + TARGETA_SIZE)

static void isr_rx(const struct bsr_evt *e, void *param);

static struct {
	struct bsr_pkt_cfg cfg;
	const struct lll_filter *filter;
	bool resolve;

	struct lll_scan *lll;
	/* NULL when the reception was started by the scan event */
	struct lll_scan_aux *lll_aux;

	uint8_t phy;

	/* The times reported to the ULL are relative to the current radio
	 * event.
	 */
	uint32_t ticks_ref;

#if defined(CONFIG_BT_CENTRAL)
	/* Held until the AUX_CONNECT_RSP is received */
	struct node_rx_pdu *node_conn_rx;
#endif /* CONFIG_BT_CENTRAL */

	struct pdu_adv pdu_tx;
} evt;

/* Without any PDU reported in the auxiliary scan event, the ULL gets a done
 * event to release the auxiliary context.
 */
static uint16_t trx_cnt;

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

static void ftr_fill(struct node_rx_ftr *ftr, const struct bsr_evt *e, uint8_t irkmatch_ok,
		     uint8_t rl_idx, bool dir_report)
{
	ftr->ticks_anchor = evt.ticks_ref;
	ftr->radio_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
	ftr->phy_flags = 0U;
	ftr->rssi = lll_rssi_get(e->rssi);

#if defined(CONFIG_BT_CTLR_PRIVACY)
	ftr->rl_idx = (irkmatch_ok != 0U) ? rl_idx : FILTER_IDX_NONE;
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

static void rx(uint32_t start, uint32_t window_us)
{
	struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_radio_rx(&evt.cfg, start, window_us, node_rx->pdu, isr_rx, NULL);
}

#if defined(CONFIG_BT_CENTRAL)
/* Without an AUX_CONNECT_RSP there is no connection, and the initiator goes
 * on.
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
#endif /* CONFIG_BT_CENTRAL */

static void isr_done(const struct bsr_evt *e, void *param)
{
	struct lll_scan_aux *lll_aux = param;
	struct lll_scan *lll;

	ARG_UNUSED(e);

	lll = ull_scan_aux_lll_parent_get(lll_aux, NULL);

#if defined(CONFIG_BT_CENTRAL)
	if (evt.node_conn_rx != NULL) {
		conn_rx_release(lll);
	}
#endif /* CONFIG_BT_CENTRAL */

	if (trx_cnt == 0U) {
		struct event_done_extra *extra;

		extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN_AUX);
		LL_ASSERT_ERR(extra != NULL);

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
		static struct mayfly mfy = { 0, 0, &link, NULL, lll_disable };
		uint32_t ret;

		mfy.param = lll;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
		LL_ASSERT_ERR(ret == 0U);
	}

	lll_isr_cleanup(lll_aux);
}

static void isr_rx_close(int err)
{
	struct lll_scan *lll = evt.lll;

	if (evt.lll_aux != NULL) {
		isr_done(NULL, evt.lll_aux);

		return;
	}

	/* Once a PDU has been reported (-ECANCELED) the ULL goes on from it,
	 * else it is to release the auxiliary context.
	 */
	if (err != -ECANCELED) {
		lll_scan_isr_aux_release(lll);
	}

	lll->is_aux_sched = 0U;

	lll_scan_isr_resume(lll);
}

/* Only the AUX_SCAN_RSP of the advertiser the AUX_SCAN_REQ was sent to */
static bool scan_rsp_check(const struct pdu_adv *pdu)
{
	const struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;

	return (pdu->type == PDU_ADV_TYPE_AUX_SCAN_RSP) &&
	       (pdu->len >= (PDU_AC_EXT_HEADER_SIZE_MIN + sizeof(struct pdu_adv_ext_hdr) +
			     ADVA_SIZE)) &&
	       (com_hdr->ext_hdr_len >= (sizeof(struct pdu_adv_ext_hdr) + ADVA_SIZE)) &&
	       (com_hdr->ext_hdr.adv_addr != 0U) && (pdu->tx_addr == evt.pdu_tx.rx_addr) &&
	       (memcmp(&com_hdr->ext_hdr.data[ADVA_OFFSET], &evt.pdu_tx.scan_req.adv_addr[0],
		       BDADDR_SIZE) == 0);
}

/* The PDU received after an AUX_SCAN_REQ, which gives the result of the
 * backoff.
 */
static void isr_rx_scan_rsp(const struct bsr_evt *e, void *param)
{
	const struct node_rx_pdu *node_rx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_scan_backoff_result((e->status == BSR_STATUS_OK) &&
				scan_rsp_check((const struct pdu_adv *)node_rx->pdu));

	isr_rx(e, param);
}

static void isr_tx_scan_req(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *node_rx;

	ARG_UNUSED(param);

	if (e->status != BSR_STATUS_OK) {
		isr_rx_close(-EINVAL);

		return;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(evt.phy), node_rx->pdu, isr_rx_scan_rsp, NULL);
}

#if defined(CONFIG_BT_CENTRAL)
static bool connect_rsp_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu_tx,
				  const struct pdu_adv *pdu_rx, uint8_t rl_idx)
{
	const uint8_t *adva = &pdu_rx->adv_ext_ind.ext_hdr.data[ADVA_OFFSET];
	bool is_adva;

	if ((pdu_rx->type != PDU_ADV_TYPE_AUX_CONNECT_RSP) ||
	    (pdu_rx->len != AUX_CONNECT_RSP_LEN) || (pdu_rx->adv_ext_ind.adv_mode != 0U) ||
	    (pdu_rx->adv_ext_ind.ext_hdr.adv_addr == 0U) ||
	    (pdu_rx->adv_ext_ind.ext_hdr.tgt_addr == 0U)) {
		return false;
	}

	/* With the Filter Accept List, the Host gives no peer address, so the
	 * advertiser is the one the AUX_CONNECT_REQ was sent to.
	 */
	if ((lll->filter_policy & SCAN_FP_FILTER) != 0U) {
		is_adva = (pdu_rx->tx_addr == pdu_tx->rx_addr) &&
			  (memcmp(adva, &pdu_tx->connect_ind.adv_addr[0], BDADDR_SIZE) == 0);
	} else {
		is_adva = lll_scan_adva_check(lll, pdu_rx->tx_addr, adva, rl_idx);
	}

	return is_adva && (pdu_rx->rx_addr == pdu_tx->tx_addr) &&
	       (memcmp(&pdu_rx->adv_ext_ind.ext_hdr.data[TGTA_OFFSET],
		       &pdu_tx->connect_ind.init_addr[0], BDADDR_SIZE) == 0);
}

#if defined(CONFIG_BT_CTLR_PHY)
/* The connection starts on the PHY of the AUX_CONNECT_REQ */
static void conn_phy_set(struct lll_conn *lll_conn, uint8_t phy)
{
#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	lll_conn->dle.eff.max_tx_time = MAX(lll_conn->dle.eff.max_tx_time,
					    PDU_DC_MAX_US(PDU_DC_PAYLOAD_SIZE_MIN, phy));
	lll_conn->dle.eff.max_rx_time = MAX(lll_conn->dle.eff.max_rx_time,
					    PDU_DC_MAX_US(PDU_DC_PAYLOAD_SIZE_MIN, phy));
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */

	lll_conn->phy_tx = phy;
	lll_conn->phy_tx_time = phy;
	lll_conn->phy_flags = 0U;
	lll_conn->phy_rx = phy;
}
#endif /* CONFIG_BT_CTLR_PHY */

static bool connect_rsp_rx(struct lll_scan *lll)
{
	struct node_rx_pdu *rx = evt.node_conn_rx;
	struct lll_addr_match match;
	struct node_rx_pdu *node_rx;
	struct pdu_adv *pdu_rx;
	uint8_t rl_idx;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	pdu_rx = (void *)node_rx->pdu;

	/* The AdvA is resolved again, as it can differ from the one of the
	 * AUX_ADV_IND.
	 */
	(void)lll_addr_match_ext(pdu_rx, NULL, evt.resolve, &match);
	rl_idx = lll_scan_rl_idx_get(lll, &match);

	if (!connect_rsp_pdu_check(lll, &evt.pdu_tx, pdu_rx, rl_idx)) {
		return false;
	}

	evt.node_conn_rx = NULL;

#if defined(CONFIG_BT_CTLR_PHY)
	conn_phy_set(lll->conn, evt.phy);
#endif /* CONFIG_BT_CTLR_PHY */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (match.irkmatch_ok != 0U) {
		struct pdu_adv *pdu = (void *)rx->pdu;

		/* The peer address is the AdvA of the AUX_CONNECT_RSP */
		pdu->rx_addr = pdu_rx->tx_addr;
		(void)memcpy(&pdu->connect_ind.adv_addr[0],
			     &pdu_rx->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], BDADDR_SIZE);
		rx->rx_ftr.rl_idx = rl_idx;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	ull_rx_put_sched(rx->hdr.link, rx);

	return true;
}

static void isr_rx_connect_rsp(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = evt.lll;
	bool is_rsp;

	ARG_UNUSED(param);

	LL_ASSERT_DBG(evt.node_conn_rx != NULL);

	is_rsp = (e->status == BSR_STATUS_OK) && connect_rsp_rx(lll);
	lll_scan_backoff_result(is_rsp);
	if (!is_rsp) {
		conn_rx_release(lll);
	}

	isr_rx_close(-EINVAL);
}

static void isr_tx_connect_req(const struct bsr_evt *e, void *param)
{
	struct node_rx_pdu *node_rx;

	ARG_UNUSED(param);

	if (e->status != BSR_STATUS_OK) {
		conn_rx_release(evt.lll);
		isr_rx_close(-EINVAL);

		return;
	}

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT_DBG(node_rx != NULL);

	lll_radio_rx(&evt.cfg, lll_radio_tifs_rx_start(e->ts_end, EVENT_IFS_US),
		     lll_radio_tifs_rx_window(evt.phy), node_rx->pdu, isr_rx_connect_rsp, NULL);
}

static bool init_pdu_check(const struct lll_scan *lll, const struct pdu_adv *pdu, uint8_t rl_idx)
{
	return ((pdu->adv_ext_ind.adv_mode & BT_HCI_LE_ADV_PROP_CONN) != 0U) &&
	       lll_scan_ext_tgta_check(lll, false, true, pdu, rl_idx, NULL);
}

static int isr_rx_init(struct lll_scan *lll, const struct lll_scan_aux *lll_aux,
		       const struct bsr_evt *e, const struct pdu_adv *pdu,
		       const struct lll_addr_match *match, uint8_t rl_idx)
{
	struct node_rx_ftr *ftr;
	struct node_rx_pdu *rx;
	struct pdu_adv *pdu_tx;
	uint32_t conn_space_us;
	struct ull_hdr *ull;
	uint32_t pdu_end_us;
	uint8_t init_tx_addr;
	uint8_t *init_addr;
#if defined(CONFIG_BT_CTLR_PRIVACY)
	bt_addr_t *lrpa;
#endif /* CONFIG_BT_CTLR_PRIVACY */

	/* The ULL has not yet assigned an auxiliary context to the reception
	 * started by the scan event.
	 */
	if ((lll_aux == NULL) && (lll->lll_aux == NULL)) {
		return -ECHILD;
	}

	/* A connection set up on the secondary channel uses CSA#2, which
	 * needs a node rx more.
	 */
	rx = ull_pdu_rx_alloc_peek(4);
	if (rx == NULL) {
		return -ENOBUFS;
	}

	/* The connection request exchange must end within the scan event */
	pdu_end_us = e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref);
	if (lll->ticks_window == 0U) {
		uint32_t scan_interval_us;

		scan_interval_us = lll->interval * SCAN_INT_UNIT_US;
		pdu_end_us %= scan_interval_us;
	}
	ull = HDR_LLL2ULL(lll);
	if (pdu_end_us > (HAL_TICKER_TICKS_TO_US(ull->ticks_slot) - EVENT_IFS_US -
			  PDU_AC_MAX_US(sizeof(struct pdu_adv_connect_ind), evt.phy) -
			  EVENT_IFS_US - PDU_AC_MAX_US(AUX_CONNECT_RSP_LEN, evt.phy) -
			  EVENT_OVERHEAD_START_US - EVENT_TICKER_RES_MARGIN_US)) {
		return -ETIME;
	}

	init_tx_addr = lll->init_addr_type;
	init_addr = lll->init_addr;
#if defined(CONFIG_BT_CTLR_PRIVACY)
	lrpa = ull_filter_lll_lrpa_get(rl_idx);
	if ((lll->rpa_gen != 0U) && (lrpa != NULL)) {
		init_tx_addr = 1U;
		init_addr = lrpa->val;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	pdu_tx = &evt.pdu_tx;
	lll_scan_prepare_connect_req(lll, pdu_tx, evt.phy,
				     e->ts_end - HAL_TICKER_TICKS_TO_US(evt.ticks_ref),
				     pdu->tx_addr, &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET],
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
	rx->hdr.handle = LLL_HANDLE_INVALID;

	/* Give the AUX_CONNECT_REQ sent to the ULL, with CSA#2 */
	(void)memcpy(rx->pdu, pdu_tx,
		     offsetof(struct pdu_adv, connect_ind) + sizeof(struct pdu_adv_connect_ind));
	((struct pdu_adv *)rx->pdu)->chan_sel = 1U;

	ftr = &rx->rx_ftr;
	ftr->param = lll;
	ftr->ticks_anchor = evt.ticks_ref;
	ftr->radio_end_us = conn_space_us;

#if defined(CONFIG_BT_CTLR_PRIVACY)
	ftr->rl_idx = (match->irkmatch_ok != 0U) ? rl_idx : FILTER_IDX_NONE;
	ftr->lrpa_used = (lll->rpa_gen != 0U) && (lrpa != NULL);
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(match);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

	ftr->extra = ull_pdu_rx_alloc();

	evt.node_conn_rx = rx;

	return 0;
}
#endif /* CONFIG_BT_CENTRAL */

static bool scan_req_pdu_check(const struct lll_scan *lll, const struct lll_scan_aux *lll_aux,
			       const struct pdu_adv *pdu, uint8_t rl_idx, bool *dir_report)
{
	return (lll->type != 0U) &&
	       (((lll_aux != NULL) && (lll_aux->state == 0U)) ||
		((lll->lll_aux != NULL) && (lll->lll_aux->state == 0U))) &&
	       ((pdu->adv_ext_ind.adv_mode & BT_HCI_LE_ADV_PROP_SCAN) != 0U) &&
	       lll_scan_ext_tgta_check(lll, false, false, pdu, rl_idx, dir_report);
}

static int isr_rx_scan_req(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
			   const struct bsr_evt *e, struct node_rx_pdu *node_rx,
			   const struct pdu_adv *pdu, uint8_t irkmatch_ok, uint8_t rl_idx,
			   bool dir_report)
{
	struct node_rx_ftr *ftr;

	/* Two node rx for the reports of the AUX_ADV_IND and AUX_SCAN_RSP, and
	 * two kept for the connections.
	 */
	if (ull_pdu_rx_alloc_peek(4) == NULL) {
		return -ENOBUFS;
	}

	lll_scan_prepare_scan_req(lll, &evt.pdu_tx, pdu->tx_addr,
				  &pdu->adv_ext_ind.ext_hdr.data[ADVA_OFFSET], rl_idx);
	lll_radio_tx(&evt.cfg, e->ts_end + EVENT_IFS_US, &evt.pdu_tx, isr_tx_scan_req, NULL);

	(void)ull_pdu_rx_alloc();

	node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_REPORT;

	ftr = &node_rx->rx_ftr;
	if (lll_aux != NULL) {
		ftr->param = lll_aux;
		lll_aux->state = 1U;
	} else {
		ftr->param = lll;
		ftr->lll_aux = lll->lll_aux;
		lll->lll_aux->state = 1U;
	}
	ftr_fill(ftr, e, irkmatch_ok, rl_idx, dir_report);
	ftr->scan_req = 1U;
	ftr->scan_rsp = 0U;
	ftr->aux_lll_sched = 0U;

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	return 0;
}

static bool report_pdu_check(const struct lll_scan *lll, const struct lll_scan_aux *lll_aux,
			     const struct pdu_adv *pdu, uint8_t rl_idx, bool *dir_report)
{
	/* A chain PDU has no AdvA or TargetA to check */
	return ((lll_aux != NULL) && (lll_aux->is_chain_sched != 0U)) ||
	       ((lll->lll_aux != NULL) && (lll->lll_aux->is_chain_sched != 0U)) ||
	       lll_scan_ext_tgta_check(lll, false, false, pdu, rl_idx, dir_report);
}

static int isr_rx_report(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
			 const struct bsr_evt *e, struct node_rx_pdu *node_rx,
			 const struct pdu_adv *pdu, uint8_t irkmatch_ok, uint8_t rl_idx,
			 bool dir_report)
{
	struct node_rx_ftr *ftr;

	ftr = &node_rx->rx_ftr;
	if (lll_aux != NULL) {
		ftr->param = lll_aux;
		ftr->scan_rsp = lll_aux->state;
		lll_aux->is_chain_sched = 1U;

		/* The ULL associates the auxiliary context with the scan
		 * context again if the next PDU is received in this event.
		 */
		lll->lll_aux = NULL;
	} else if (lll->lll_aux != NULL) {
		ftr->param = lll;
		ftr->lll_aux = lll->lll_aux;
		ftr->scan_rsp = lll->lll_aux->state;
		lll->lll_aux->is_chain_sched = 1U;
	} else {
		/* The ULL has not yet assigned an auxiliary context to the
		 * reception started by the scan event.
		 */
		return -ECHILD;
	}

	/* The next reception uses the next free node rx */
	(void)ull_pdu_rx_alloc();

	node_rx->hdr.type = NODE_RX_TYPE_EXT_AUX_REPORT;

	ftr_fill(ftr, e, irkmatch_ok, rl_idx, dir_report);
	ftr->scan_req = 0U;
	ftr->aux_lll_sched = lll_scan_aux_setup(lll, lll_aux, pdu, evt.phy, e->ts_start,
						evt.ticks_ref);

	ull_rx_put_sched(node_rx->hdr.link, node_rx);

	if (ftr->aux_lll_sched != 0U) {
		if (lll_aux == NULL) {
			lll->is_aux_sched = 1U;
		}

		return 0;
	}

	trx_cnt++;

	return -ECANCELED;
}

static int isr_rx_pdu(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
		      const struct bsr_evt *e, struct node_rx_pdu *node_rx,
		      const struct pdu_adv *pdu, const struct lll_addr_match *match,
		      uint8_t rl_idx)
{
	bool dir_report = false;

#if defined(CONFIG_BT_CENTRAL)
	if (lll->conn != NULL) {
		if ((lll->conn->central.cancelled != 0U) || !init_pdu_check(lll, pdu, rl_idx) ||
		    !lll_scan_backoff_is_req()) {
			return -EINVAL;
		}

		return isr_rx_init(lll, lll_aux, e, pdu, match, rl_idx);
	}
#endif /* CONFIG_BT_CENTRAL */

	/* A PDU that the backoff holds the request for is only reported */
	if (scan_req_pdu_check(lll, lll_aux, pdu, rl_idx, &dir_report) &&
	    lll_scan_backoff_is_req()) {
		return isr_rx_scan_req(lll, lll_aux, e, node_rx, pdu, match->irkmatch_ok, rl_idx,
				       dir_report);
	}

	if (!report_pdu_check(lll, lll_aux, pdu, rl_idx, &dir_report)) {
		return -EINVAL;
	}

	return isr_rx_report(lll, lll_aux, e, node_rx, pdu, match->irkmatch_ok, rl_idx,
			     dir_report);
}

static void isr_rx(const struct bsr_evt *e, void *param)
{
	struct lll_scan *lll = evt.lll;
	struct node_rx_pdu *node_rx;
	struct lll_addr_match match;
	struct pdu_adv *pdu;
	bool has_adva;
	uint8_t rl_idx;
	int err;

	ARG_UNUSED(param);

	/* No PDU, or one with errors */
	if (e->status != BSR_STATUS_OK) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	node_rx = ull_pdu_rx_alloc_peek(3);
	if (node_rx == NULL) {
		err = -ENOBUFS;

		goto isr_rx_do_close;
	}

	/* The auxiliary PDUs have the PDU type of ADV_EXT_IND */
	pdu = (void *)node_rx->pdu;
	if ((pdu->type != PDU_ADV_TYPE_EXT_IND) || (pdu->len == 0U)) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	/* A chain PDU can have no AdvA */
	has_adva = lll_addr_match_ext(pdu, evt.filter, evt.resolve, &match);
	rl_idx = lll_scan_rl_idx_get(lll, &match);

	if (has_adva && !lll_scan_isr_rx_check(lll, match.irkmatch_ok, match.devmatch_ok, rl_idx)) {
		err = -EINVAL;

		goto isr_rx_do_close;
	}

	err = isr_rx_pdu(lll, evt.lll_aux, e, node_rx, pdu, &match, rl_idx);
	if (err == 0) {
		return;
	}

isr_rx_do_close:
	isr_rx_close(err);
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
#endif /* CONFIG_BT_CENTRAL */

	/* Not started if stopped on connection establishment race between
	 * LLL and ULL.
	 */
	if (IS_ENABLED(CONFIG_BT_CENTRAL) && unlikely(lll_scan_is_stopped(lll))) {
		lll_radio_stop(isr_done, lll_aux);

		return 0;
	}

	if (lll_preempt_calc(p) != 0U) {
		lll_radio_stop(isr_done, lll_aux);

		return -ECANCELED;
	}

	lll_aux->state = 0U;

	cfg_set(lll, lll_aux->phy, lll_aux->chan);

	/* A PDU that starts at the end of the window is received too */
	start_us = lll_event_start_get(p, &evt.ticks_ref);
	rx(start_us, lll_aux->window_size_us + addr_us_get(lll_aux->phy));

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_SCHED_ADVANCED)
	/* Get the offset, from this event, of the free time space after the
	 * other central connections where the first connection event is to be
	 * placed.
	 */
	if (lll->conn != NULL) {
		static memq_link_t link;
		static struct mayfly mfy = { 0U, 0U, &link, NULL,
					     ull_sched_mfy_after_cen_offset_get };
		struct lll_prepare_param *prepare_param;
		uint32_t ret;

		/* The ULL gets the offset for the scan context */
		prepare_param = &lll->prepare_param;
		prepare_param->ticks_at_expire = p->ticks_at_expire;
		prepare_param->remainder = p->remainder;
		prepare_param->param = lll;

		mfy.param = prepare_param;

		ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_LOW, 1U, &mfy);
		LL_ASSERT_ERR(ret == 0U);
	}
#endif /* CONFIG_BT_CENTRAL && CONFIG_BT_CTLR_SCHED_ADVANCED */

	err = lll_prepare_done(lll_aux);
	LL_ASSERT_ERR(err == 0);

	DEBUG_RADIO_START_O(1);

	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	ARG_UNUSED(curr);

	/* An auxiliary scan event is not resumed */
	ARG_UNUSED(resume_cb);

#if defined(CONFIG_BT_CENTRAL)
	/* The connection is set up once the AUX_CONNECT_REQ has been sent */
	if (evt.node_conn_rx != NULL) {
		return 0;
	}
#endif /* CONFIG_BT_CENTRAL */

	/* The scan event that is next continues after this event */
	if ((next != NULL) && (ull_scan_lll_is_valid_get(next) != NULL)) {
		return 0;
	}

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	struct event_done_extra *extra;
	int err;

	/* An event in progress rather than one in the prepare pipeline */
	if (prepare_param == NULL) {
		/* The event is done once the radio has been stopped */
		lll_radio_stop(isr_done, param);

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_SCAN_AUX);
	LL_ASSERT_ERR(extra != NULL);

#if defined(CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS)
	extra->lll = param;
#endif /* CONFIG_BT_CTLR_SCAN_AUX_USE_CHAINS */

	lll_done(param);
}

void lll_scan_aux_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

/* NULL if the AuxPtr does not fit in the extended header */
static const struct pdu_adv_aux_ptr *aux_ptr_get(const struct pdu_adv *pdu)
{
	const struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;
	const struct pdu_adv_ext_hdr *hdr = &com_hdr->ext_hdr;
	uint8_t offset;

	if ((pdu->len < PDU_AC_EXT_HEADER_SIZE_MIN) || (com_hdr->ext_hdr_len == 0U) ||
	    (pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN + com_hdr->ext_hdr_len)) ||
	    (hdr->aux_ptr == 0U)) {
		return NULL;
	}

	offset = sizeof(*hdr);
	if (hdr->adv_addr != 0U) {
		offset += BDADDR_SIZE;
	}
	if (hdr->tgt_addr != 0U) {
		offset += BDADDR_SIZE;
	}
	if (hdr->cte_info != 0U) {
		offset += sizeof(struct pdu_cte_info);
	}
	if (hdr->adi != 0U) {
		offset += sizeof(struct pdu_adv_adi);
	}

	if ((offset + sizeof(struct pdu_adv_aux_ptr)) > com_hdr->ext_hdr_len) {
		return NULL;
	}

	return (const void *)&com_hdr->ext_hdr_adv_data[offset];
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
	if ((aux_ptr == NULL) || (PDU_ADV_AUX_PTR_OFFSET_GET(aux_ptr) == 0U) ||
	    (aux_ptr->chan_idx >= CHM_USED_COUNT_MAX)) {
		return false;
	}

	/* The radio model has no LE Coded PHY */
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

	window_size_us = (aux_ptr->offs_units != 0U) ? OFFS_UNIT_300_US : OFFS_UNIT_30_US;
	aux_offset_us = (uint32_t)PDU_ADV_AUX_PTR_OFFSET_GET(aux_ptr) * window_size_us;

	pdu_us = PDU_AC_US(pdu->len, phy, PHY_FLAGS_S8);
	if (!AUX_OFFSET_IS_VALID(aux_offset_us, window_size_us, pdu_us)) {
		return false;
	}

	if (aux_ptr->ca != 0U) {
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
