/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>

#include "hal/ecb.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll_filter.h"
#include "lll_addr.h"

#if defined(CONFIG_BT_CTLR_PRIVACY)
/* A resolvable private address resolves with an IRK if its hash is
 * ah(IRK, prand) (Core Spec Vol 3, Part H, Section 2.2.2). The IRKs of the
 * resolving list are big endian.
 */
static bool rpa_irk_matches(const uint8_t *irk, const uint8_t *addr)
{
	uint8_t prand[16] = { 0 };
	uint8_t hash[16];

	prand[13] = addr[5];
	prand[14] = addr[4];
	prand[15] = addr[3];

	ecb_encrypt_be(irk, prand, hash);

	return (hash[15] == addr[0]) && (hash[14] == addr[1]) && (hash[13] == addr[2]);
}
#endif /* CONFIG_BT_CTLR_PRIVACY */

static void addr_match(uint8_t addr_type, const uint8_t *addr, const struct lll_filter *filter,
		       bool resolve, struct lll_addr_match *match)
{
#if defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	if (filter) {
		match->devmatch_ok = ull_filter_lll_fal_match(filter, addr_type, addr,
							      &match->devmatch_id);
	}
#else /* !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
	ARG_UNUSED(filter);
#endif /* !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (resolve && addr_type && ((addr[5] & 0xC0) == 0x40)) {
		const uint8_t (*irks)[IRK_SIZE];
		uint8_t count;

		irks = (const void *)ull_filter_lll_irks_get(&count);
		for (uint8_t i = 0U; i < count; i++) {
			if (rpa_irk_matches(irks[i], addr)) {
				match->irkmatch_ok = 1U;
				match->irkmatch_id = i;
				break;
			}
		}
	}
#else /* !CONFIG_BT_CTLR_PRIVACY */
	ARG_UNUSED(resolve);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

#if !defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && !defined(CONFIG_BT_CTLR_PRIVACY)
	ARG_UNUSED(addr_type);
	ARG_UNUSED(addr);
#endif /* !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST && !CONFIG_BT_CTLR_PRIVACY */
}

static void match_init(struct lll_addr_match *match)
{
	match->devmatch_ok = 0U;
	match->devmatch_id = FILTER_IDX_NONE;
	match->irkmatch_ok = 0U;
	match->irkmatch_id = FILTER_IDX_NONE;
}

void lll_addr_match(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
		    struct lll_addr_match *match)
{
	match_init(match);

	if (pdu->len < BDADDR_SIZE) {
		return;
	}

	addr_match(pdu->tx_addr, pdu->payload, filter, resolve, match);
}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
bool lll_addr_match_ext(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
			struct lll_addr_match *match)
{
	const struct pdu_adv_com_ext_adv *com_hdr = &pdu->adv_ext_ind;

	match_init(match);

	if ((pdu->len < (PDU_AC_EXT_HEADER_SIZE_MIN + sizeof(struct pdu_adv_ext_hdr) +
			 ADVA_SIZE)) ||
	    (com_hdr->ext_hdr_len < (sizeof(struct pdu_adv_ext_hdr) + ADVA_SIZE)) ||
	    !com_hdr->ext_hdr.adv_addr) {
		return false;
	}

	addr_match(pdu->tx_addr, &com_hdr->ext_hdr.data[ADVA_OFFSET], filter, resolve, match);

	return true;
}
#endif /* CONFIG_BT_CTLR_ADV_EXT */
