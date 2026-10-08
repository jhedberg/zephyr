/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The advertising and scanning roles filter and resolve the address of a
 * received PDU the same way, so they share this. A NULL filter skips the
 * Filter Accept List match.
 */
struct lll_addr_match {
	uint8_t devmatch_ok;
	uint8_t devmatch_id;
	uint8_t irkmatch_ok;
	uint8_t irkmatch_id;
};

/* For the first address in the payload of a legacy PDU: AdvA, ScanA or InitA */
void lll_addr_match(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
		    struct lll_addr_match *match);

/* For the AdvA of an extended advertising PDU. Returns false, with no match,
 * if the PDU has none.
 */
bool lll_addr_match_ext(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
			struct lll_addr_match *match);
