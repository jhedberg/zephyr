/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "irq_ctrl.h"
#include "bs_2g4_radio_platform.h"
#include "bs_2g4_radio_if.h"
#include "hw_testcheat_if.h"

void bsr_plat_irq_raise(unsigned int irq)
{
	hw_irq_ctrl_raise_im(irq);
}

/*
 * The bsim tests set the test cheats of the nRF RADIO model (see
 * hw_testcheat_if.h), which this board applies to its radio model instead.
 */
void hw_radio_testcheat_set_tx_power_gain(double power_offset)
{
	bsr_testcheat_set_tx_power_gain(power_offset);
}

void hw_radio_testcheat_set_rx_power_gain(double power_offset)
{
	bsr_testcheat_set_rx_power_gain(power_offset);
}

void hw_radio_testcheat_disable_tx(int64_t count)
{
	bsr_testcheat_disable_tx(count);
}

void hw_radio_testcheat_disable_rx(int64_t count_dont_sync, int64_t count_fail_crc)
{
	bsr_testcheat_disable_rx(count_dont_sync, count_fail_crc);
}
