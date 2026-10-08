/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Shared by the primary channel advertising events (lll_adv.c) and the
 * auxiliary advertising events (lll_adv_aux.c) of the BabbleSim LLL.
 */

/* Filter and resolution of the address of a received SCAN_REQ or
 * CONNECT_IND, as per the filter policy of the advertiser.
 */
void lll_adv_filter_get(const struct lll_adv *lll, const struct lll_filter **filter,
			bool *resolve);

/* Check of a received SCAN_REQ or AUX_SCAN_REQ against the AdvA, tx_addr and
 * addr, of the advertising PDU sent.
 */
bool lll_adv_scan_req_check(const struct lll_adv *lll, const struct pdu_adv *sr, uint8_t tx_addr,
			    const uint8_t *addr, uint8_t devmatch_ok, uint8_t *rl_idx);

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
/* Report the received SCAN_REQ or AUX_SCAN_REQ, which is in the next free
 * node rx.
 */
int lll_adv_scan_req_report(struct lll_adv *lll, const struct bsr_evt *e, uint8_t rl_idx);
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

#if defined(CONFIG_BT_PERIPHERAL)
/* Check of a received CONNECT_IND or AUX_CONNECT_REQ against the AdvA, and
 * the TargetA of a directed advertising PDU, of the advertising PDU sent.
 */
bool lll_adv_connect_ind_check(const struct lll_adv *lll, const struct pdu_adv *ci,
			       uint8_t tx_addr, const uint8_t *addr, uint8_t rx_addr,
			       const uint8_t *tgt_addr, uint8_t devmatch_ok, uint8_t *rl_idx);
#endif /* CONFIG_BT_PERIPHERAL */
