/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Shared by the primary channel scanning events (lll_scan.c) and the
 * auxiliary channel scanning (lll_scan_aux.c) of the BabbleSim LLL.
 */

/* Filter and resolution of the AdvA of received PDUs, as per the filter
 * policy of the scanner.
 */
void lll_scan_filter_get(const struct lll_scan *lll, const struct lll_filter **filter,
			 bool *resolve);

/* Resolving list index of a received AdvA */
uint8_t lll_scan_rl_idx_get(const struct lll_scan *lll, const struct lll_addr_match *match);

/* Filter policy check of a received AdvA. Returns false if the PDU is to be
 * dropped. While a periodic advertising sync is being created with the Filter
 * Accept List enabled, a PDU that the filter policy rejects is still received
 * for the ULL to look for the periodic advertiser in it, and
 * match->devmatch_ok is then set to whether it is reported.
 */
bool lll_scan_isr_rx_filter(const struct lll_scan *lll, struct lll_addr_match *match,
			    uint8_t rl_idx);

/* Initiator check of the AdvA of a received PDU, without a filter accept
 * list.
 */
bool lll_scan_adva_check(const struct lll_scan *lll, uint8_t addr_type, const uint8_t *addr,
			 uint8_t rl_idx);

/* AdvA and TargetA checks of a received extended advertising PDU. pri is
 * true for a primary channel PDU, which needs no AdvA.
 */
bool lll_scan_ext_tgta_check(const struct lll_scan *lll, bool pri, bool is_init,
			     const struct pdu_adv *pdu, uint8_t rl_idx, bool *dir_report);

/* Fill in the CONNECT_IND or AUX_CONNECT_REQ sent tIFS after a PDU received
 * on phy (0 for a legacy PDU) that ended pdu_end_us after the reference of
 * the event, and get the time of the first connection event relative to the
 * same reference.
 */
void lll_scan_prepare_connect_req(struct lll_scan *lll, struct pdu_adv *pdu_tx, uint8_t phy,
				  uint32_t pdu_end_us, uint8_t adv_tx_addr, const uint8_t *adv_addr,
				  uint8_t init_tx_addr, const uint8_t *init_addr,
				  uint32_t *conn_space_us);

/* Continue the primary channel scanning after the auxiliary PDU receptions
 * it started have ended, or close the scan event if it is being stopped.
 */
void lll_scan_isr_resume(struct lll_scan *lll);

/* Have the ULL release the auxiliary context of the auxiliary PDU receptions
 * started by the scan event, when they end without a last report.
 */
void lll_scan_isr_aux_release(struct lll_scan *lll);

/* Reception of an auxiliary PDU in the radio event of the PDU with the AuxPtr
 * that points to it.
 */
struct lll_scan_aux_rx {
	uint32_t start;     /* Start of the reception */
	uint32_t window_us; /* Window to receive the access address in */
	uint8_t phy;        /* PHY_1M or PHY_2M */
	uint8_t chan;
};

/* Get the reception of the auxiliary PDU that the AuxPtr of a PDU received on
 * phy from pdu_start_us points to. Returns false if the PDU has no valid
 * AuxPtr, or if the ULL has the time to schedule the reception.
 */
bool lll_scan_aux_rx_get(const struct pdu_adv *pdu, uint8_t phy, uint32_t pdu_start_us,
			 struct lll_scan_aux_rx *rx);

/* Listen for the auxiliary PDU that the AuxPtr of a PDU received on phy from
 * pdu_start_us points to, when it is too soon for the ULL to schedule it.
 * lll_aux is the auxiliary context of the current event, NULL in a scan
 * event, and ticks_ref the reference of the times reported to the ULL.
 * Returns true if the reception has been started.
 */
bool lll_scan_aux_setup(struct lll_scan *lll, struct lll_scan_aux *lll_aux,
			const struct pdu_adv *pdu, uint8_t phy, uint32_t pdu_start_us,
			uint32_t ticks_ref);
