/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The packet level 2.4GHz radio model uses the Phy connection of the nRF HW
 * models (NRF_HWLowL), which the nRF RADIO model uses too, so only one of the
 * two radio models can be in use.
 */

#include <stdint.h>

#include "NHW_config.h"
#include "NHW_common_types.h"
#include "irq_ctrl.h"
#include "bs_2g4_radio_platform.h"
#include "bs_2g4_radio_if.h"

/* The interrupts go to the CPU the nRF RADIO would interrupt */
static const struct nhw_irq_mapping radio_irq_map[] = NHW_RADIO_INT_MAP;

void bsr_plat_irq_raise(unsigned int irq)
{
	hw_irq_ctrl_raise_im(radio_irq_map[0].cntl_inst, irq);
}

/*
 * Tests change the behavior of the radio with the test cheats of the nRF HW
 * models (hw_testcheat_if.h). As the nRF RADIO model is not used with this
 * model, the build wraps them (-Wl,--wrap) to apply them to this model too.
 */
void __real_hw_radio_testcheat_set_tx_power_gain(double power_offset);
void __real_hw_radio_testcheat_set_rx_power_gain(double power_offset);
void __real_hw_radio_testcheat_disable_tx(int64_t count);
void __real_hw_radio_testcheat_disable_rx(int64_t count_dont_sync, int64_t count_fail_crc);

void __wrap_hw_radio_testcheat_set_tx_power_gain(double power_offset)
{
	bsr_testcheat_set_tx_power_gain(power_offset);
	__real_hw_radio_testcheat_set_tx_power_gain(power_offset);
}

void __wrap_hw_radio_testcheat_set_rx_power_gain(double power_offset)
{
	bsr_testcheat_set_rx_power_gain(power_offset);
	__real_hw_radio_testcheat_set_rx_power_gain(power_offset);
}

void __wrap_hw_radio_testcheat_disable_tx(int64_t count)
{
	bsr_testcheat_disable_tx(count);
	__real_hw_radio_testcheat_disable_tx(count);
}

void __wrap_hw_radio_testcheat_disable_rx(int64_t count_dont_sync, int64_t count_fail_crc)
{
	bsr_testcheat_disable_rx(count_dont_sync, count_fail_crc);
	__real_hw_radio_testcheat_disable_rx(count_dont_sync, count_fail_crc);
}
