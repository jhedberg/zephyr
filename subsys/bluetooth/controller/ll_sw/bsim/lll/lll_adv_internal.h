/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The auxiliary advertising events answer AUX_SCAN_REQ and AUX_CONNECT_REQ
 * as the advertising events answer SCAN_REQ and CONNECT_IND, so lll_adv_aux.c
 * shares these with them. tx_addr and addr are the AdvA of the PDU sent, and
 * rx_addr and tgt_addr its TargetA, tgt_addr being NULL if it has none.
 */
void lll_adv_filter_get(const struct lll_adv *lll, const struct lll_filter **filter,
			bool *resolve);
bool lll_adv_scan_req_check(const struct lll_adv *lll, const struct pdu_adv *sr, uint8_t tx_addr,
			    const uint8_t *addr, uint8_t rx_addr, const uint8_t *tgt_addr,
			    uint8_t devmatch_ok, uint8_t *rl_idx);
int lll_adv_scan_req_report(struct lll_adv *lll, const struct bsr_evt *e, uint8_t rl_idx);
bool lll_adv_connect_ind_check(const struct lll_adv *lll, const struct pdu_adv *ci,
			       uint8_t tx_addr, const uint8_t *addr, uint8_t rx_addr,
			       const uint8_t *tgt_addr, uint8_t devmatch_ok, uint8_t *rl_idx);
