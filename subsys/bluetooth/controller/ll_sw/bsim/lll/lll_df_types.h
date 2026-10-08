/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* There is no Direction Finding support, the ULL only needs these declared */
#define BT_CTLR_DF_PER_ADV_CTE_NUM_MAX 0
#define CTE_LEN_US(n) ((n) * 8U)

struct lll_df_sync;
struct lll_df_conn_rx_cfg;
