/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

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

void lll_addr_match(const struct pdu_adv *pdu, const struct lll_filter *filter, bool resolve,
		    struct lll_addr_match *match)
{
	const uint8_t *addr = pdu->payload;

	match->devmatch_ok = 0U;
	match->devmatch_id = FILTER_IDX_NONE;
	match->irkmatch_ok = 0U;
	match->irkmatch_id = FILTER_IDX_NONE;

	if (pdu->len < BDADDR_SIZE) {
		return;
	}

#if defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	if (filter) {
		match->devmatch_ok = ull_filter_lll_fal_match(filter, pdu->tx_addr, addr,
							      &match->devmatch_id);
	}
#else /* !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */
	ARG_UNUSED(filter);
	ARG_UNUSED(addr);
#endif /* !CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (resolve && pdu->tx_addr && ((addr[5] & 0xC0) == 0x40)) {
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
}
