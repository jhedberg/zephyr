/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The ULL schedules auxiliary scan events for the chain PDUs of a periodic
 * advertising train too, which lll_scan_aux.c hands to the periodic sync with
 * these. ticks_ref is the reference of the times reported to the ULL.
 */
void lll_sync_aux_prepare_cb(struct lll_sync *lll, struct lll_scan_aux *lll_aux, uint32_t start_us,
			     uint32_t ticks_ref);

/* The ULL releases the auxiliary context at the end of a chain it got the
 * reports of, else it needs this, which also has the data reported as
 * incomplete.
 */
void lll_sync_isr_aux_release(struct lll_sync *lll, struct lll_scan_aux *lll_aux);
