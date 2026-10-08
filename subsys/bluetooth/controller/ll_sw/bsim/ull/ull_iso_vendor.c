/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>

#include <errno.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/hci_types.h>

#include "util/util.h"
#include "util/memq.h"

#include "hal/ccm.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"

#include "isoal.h"

#include "ull_iso_types.h"
#include "ull_iso_internal.h"

/* A simulation has nothing behind a vendor data path, so only the Controller
 * to Host direction is supported, for the tests of the vendor data path
 * interface, with the ISOAL sink of ll_data_path_sink_create() that the
 * application provides.
 */
static ATOMIC_DEFINE(rx_configured, BT_HCI_DATAPATH_ID_VS_END + 1U);

/* The other Data_Path_ID values are reserved (Core Spec Vol 4, Part E,
 * Section 7.3.101).
 */
static bool is_vs_path_id(uint8_t data_path_id)
{
	return IN_RANGE(data_path_id, BT_HCI_DATAPATH_ID_VS, BT_HCI_DATAPATH_ID_VS_END);
}

bool ll_data_path_configured(uint8_t data_path_dir, uint8_t data_path_id)
{
	return (data_path_dir == BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) &&
	       is_vs_path_id(data_path_id) && atomic_test_bit(rx_configured, data_path_id);
}

bool ll_data_path_source_create(uint16_t handle,
				struct ll_iso_datapath *datapath,
				isoal_source_pdu_alloc_cb *pdu_alloc,
				isoal_source_pdu_write_cb *pdu_write,
				isoal_source_pdu_emit_cb *pdu_emit,
				isoal_source_pdu_release_cb *pdu_release)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(datapath);
	ARG_UNUSED(pdu_alloc);
	ARG_UNUSED(pdu_write);
	ARG_UNUSED(pdu_emit);
	ARG_UNUSED(pdu_release);

	return false;
}

uint8_t ll_configure_data_path(uint8_t data_path_dir, uint8_t data_path_id,
			       uint8_t vs_config_len, uint8_t *vs_config)
{
	ARG_UNUSED(vs_config_len);
	ARG_UNUSED(vs_config);

	if (!is_vs_path_id(data_path_id)) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	if (data_path_dir != BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) {
		return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
	}

	atomic_set_bit(rx_configured, data_path_id);

	return 0;
}

int ll_data_path_tx_pdu_release(uint16_t handle, struct node_tx_iso *node_tx)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(node_tx);

	/* Tx acknowledgments take the default path to the ULL */
	return -ENOTSUP;
}
