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

/* While a periodic advertising sync is being created, the ULL looks for the
 * periodic advertiser in the PDUs that the filter policy rejects too. Those are
 * then received, with devmatch_ok set to whether the PDU is reported. Returns
 * false if the PDU is dropped.
 */
bool lll_scan_isr_rx_filter(const struct lll_scan *lll, struct lll_addr_match *match,
			    uint8_t rl_idx);

/* For an initiator that does not use the Filter Accept List */
bool lll_scan_adva_check(const struct lll_scan *lll, uint8_t addr_type, const uint8_t *addr,
			 uint8_t rl_idx);

/* The AdvA of an ADV_EXT_IND (pri) is optional, as the AUX_ADV_IND has it */
bool lll_scan_ext_tgta_check(const struct lll_scan *lll, bool pri, bool is_init,
			     const struct pdu_adv *pdu, uint8_t rl_idx, bool *dir_report);

void lll_scan_prepare_scan_req(const struct lll_scan *lll, struct pdu_adv *pdu_tx,
			       uint8_t adv_tx_addr, const uint8_t *adv_addr, uint8_t rl_idx);

/* phy is PHY_LEGACY for a CONNECT_IND, as transmitWindowDelay depends on it.
 * pdu_end_us and conn_space_us are relative to the reference of the event,
 * as the other times reported to the ULL.
 */
void lll_scan_prepare_connect_req(struct lll_scan *lll, struct pdu_adv *pdu_tx, uint8_t phy,
				  uint32_t pdu_end_us, uint8_t adv_tx_addr, const uint8_t *adv_addr,
				  uint8_t init_tx_addr, const uint8_t *init_addr,
				  uint32_t *conn_space_us);

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

struct lll_scan_aux_rx {
	uint32_t start;
	uint32_t window_us;
	uint8_t phy;
	uint8_t chan;
};

/* For the periodic sync, which receives such an auxiliary PDU itself. phy and
 * rx->phy are PHY_1M or PHY_2M. Returns false if the PDU has no valid AuxPtr,
 * or if the ULL can schedule the reception.
 */
bool lll_scan_aux_rx_get(const struct pdu_adv *pdu, uint8_t phy, uint32_t pdu_start_us,
			 struct lll_scan_aux_rx *rx);
