/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void lll_scan_isr_resume(void *param);
uint8_t lll_scan_aux_setup(struct pdu_adv *pdu, uint8_t pdu_phy,
			   uint8_t pdu_phy_flags_rx, radio_isr_cb_t setup_cb,
			   void *param);
void lll_scan_aux_isr_aux_setup(void *param);
bool lll_scan_aux_addr_match_get(const struct lll_scan *lll,
				 const struct pdu_adv *pdu,
				 uint8_t *const devmatch_ok,
				 uint8_t *const devmatch_id,
				 uint8_t *const irkmatch_ok,
				 uint8_t *const irkmatch_id);
