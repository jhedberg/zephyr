/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Shared by the periodic sync events (lll_sync.c) and the auxiliary scan
 * events receiving the chain PDUs of a periodic advertising train
 * (lll_scan_aux.c) of the BabbleSim LLL.
 */

/* Listen for the chain PDU that the auxiliary scan event of lll_aux, which
 * the ULL scheduled for the periodic sync of lll, receives from time start_us.
 * ticks_ref is the reference of the times reported to the ULL.
 */
void lll_sync_aux_prepare_cb(struct lll_sync *lll, struct lll_scan_aux *lll_aux, uint32_t start_us,
			     uint32_t ticks_ref);

/* Have the ULL release the auxiliary context lll_aux of the chain PDU
 * receptions of the periodic sync of lll, and report the data as incomplete,
 * when they end before the last PDU of the chain.
 */
void lll_sync_isr_aux_release(struct lll_sync *lll, struct lll_scan_aux *lll_aux);
