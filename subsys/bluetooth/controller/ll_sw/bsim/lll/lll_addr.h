/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Filtering and resolution of the first address in the payload of a received
 * advertising channel PDU (AdvA, ScanA or InitA), as used by the filter
 * policies of the advertising and scanning roles.
 */
struct lll_addr_match {
	uint8_t devmatch_ok;
	uint8_t devmatch_id;
	uint8_t irkmatch_ok;
	uint8_t irkmatch_id;
};

/* Match the address against filter (an accept list or the resolving list),
 * if not NULL, and resolve it with the peer IRKs of the resolving list if
 * resolve is true.
 */
void lll_addr_match(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
		    struct lll_addr_match *match);
