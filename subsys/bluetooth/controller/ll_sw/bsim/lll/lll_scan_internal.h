/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The auxiliary PDUs are received as per the filter policy and initiator
 * parameters of the scan context, so lll_scan_aux.c shares these with the
 * scan events.
 */
void lll_scan_filter_get(const struct lll_scan *lll, const struct lll_filter **filter,
			 bool *resolve);
uint8_t lll_scan_rl_idx_get(const struct lll_scan *lll, const struct lll_addr_match *match);
bool lll_scan_is_stopped(const struct lll_scan *lll);

/* The backoff of the scan requests applies to AUX_SCAN_REQ and AUX_CONNECT_REQ
 * too. is_rsp is true if the response to the request was received.
 */
bool lll_scan_backoff_is_req(void);
void lll_scan_backoff_result(bool is_rsp);

void lll_scan_prepare_scan_req(const struct lll_scan *lll, struct pdu_adv *pdu_tx,
			       uint8_t adv_tx_addr, const uint8_t *adv_addr, uint8_t rl_idx);

/* The scan event goes on with the primary channel at the end of the auxiliary
 * PDUs it received.
 */
void lll_scan_isr_resume(struct lll_scan *lll);

/* The ULL releases the auxiliary context at the end of a chain it got the
 * reports of, else it needs this.
 */
void lll_scan_isr_aux_release(struct lll_scan *lll);

/* The ULL can not schedule an auxiliary PDU that starts too soon after the
 * PDU with the AuxPtr, so the current radio event receives it. lll_aux is
 * NULL in a scan event. Returns true if the reception has been started.
 */
bool lll_scan_aux_setup(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
			const struct pdu_adv *pdu, uint8_t phy, uint32_t pdu_start_us,
			uint32_t ticks_ref);
