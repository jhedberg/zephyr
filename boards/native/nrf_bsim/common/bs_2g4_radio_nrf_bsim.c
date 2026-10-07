/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * nrf_bsim platform functions for the generic packet level 2.4GHz radio model
 * (boards/native/common/bsim_2g4_radio).
 *
 * The model uses the Phy connection opened by the nRF HW models glue
 * (NRF_HWLowL). The nRF RADIO model shares this connection, so it must not be
 * used at the same time as this model.
 */

#include "bs_types.h"
#include "bs_tracing.h"
#include "nsi_hw_scheduler.h"
#include "NRF_HWLowL.h"
#include "NHW_config.h"
#include "NHW_common_types.h"
#include "irq_ctrl.h"
#include "phy_sync_ctrl.h"
#include "bs_2g4_radio_platform.h"

/* Use the same interrupt controller as the nRF RADIO */
static const struct nhw_irq_mapping radio_irq_map[] = NHW_RADIO_INT_MAP;

bs_time_t bsr_plat_phy_time_from_dev(bs_time_t dev_time)
{
	return hwll_phy_time_from_dev(dev_time);
}

bs_time_t bsr_plat_dev_time_from_phy(bs_time_t phy_time)
{
	return hwll_dev_time_from_phy(phy_time);
}

void bsr_plat_irq_raise(unsigned int irq)
{
	hw_irq_ctrl_raise_im(radio_irq_map[0].cntl_inst, irq);
}

void bsr_plat_phy_synced(bs_time_t dev_time)
{
	phy_sync_ctrl_set_last_phy_sync_time(dev_time);
}

void bsr_plat_phy_disconnected(void)
{
	bs_trace_raw_manual_time(3, nsi_hws_get_time(), "The phy disconnected us\n");
	hwll_disconnect_phy_and_exit();
}
