/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Filtering and resolution of the address of a received advertising channel
 * PDU, as used by the filter policies of the advertising and scanning roles.
 */
struct lll_addr_match {
	uint8_t devmatch_ok;
	uint8_t devmatch_id;
	uint8_t irkmatch_ok;
	uint8_t irkmatch_id;
};

/* Match the first address in the payload of a legacy PDU (AdvA, ScanA or
 * InitA) against filter (an accept list or the resolving list), if not NULL,
 * and resolve it with the peer IRKs of the resolving list if resolve is true.
 */
void lll_addr_match(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
		    struct lll_addr_match *match);

/* As lll_addr_match(), for the AdvA in the extended header of an extended
 * advertising PDU. Returns false, with no match, if the PDU has no AdvA.
 */
bool lll_addr_match_ext(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
			struct lll_addr_match *match);
