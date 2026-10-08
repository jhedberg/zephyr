/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Vendor specific ISO data path of the bsim LLL. A simulation has no codec
 * or audio interface behind the controller, so this only serves the tests of
 * the vendor data path interface: once configured, the Controller to Host
 * vendor data path uses the ISOAL sink that ll_data_path_sink_create() of
 * the application provides. There is no vendor Host to Controller path.
 */

#include <stdint.h>
#include <stdbool.h>

#include <errno.h>

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

static bool is_rx_configured;

bool ll_data_path_configured(uint8_t data_path_dir, uint8_t data_path_id)
{
	ARG_UNUSED(data_path_id);

	return (data_path_dir == BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) && is_rx_configured;
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
	ARG_UNUSED(data_path_id);
	ARG_UNUSED(vs_config_len);
	ARG_UNUSED(vs_config);

	if (data_path_dir != BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) {
		return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
	}

	is_rx_configured = true;

	return 0;
}

int ll_data_path_tx_pdu_release(uint16_t handle, struct node_tx_iso *node_tx)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(node_tx);

	/* Tx acknowledgments take the default path to the ULL */
	return -ENOTSUP;
}
