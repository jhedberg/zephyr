/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/* The auxiliary advertising events filter and report scan requests as the
 * advertising events do, so lll_adv_aux.c shares these with lll_adv.c.
 */
void lll_adv_filter_get(const struct lll_adv *lll, const struct lll_filter **filter,
			bool *resolve);
int lll_adv_scan_req_report(struct lll_adv *lll, const struct lll_radio_evt *e, uint8_t rl_idx);
