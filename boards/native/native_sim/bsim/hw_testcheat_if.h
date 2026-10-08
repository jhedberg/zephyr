/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The radio test cheats of the nRF HW models, which the bsim tests written
 * for the nrf_bsim boards set, so that they also run on this board.
 */

#ifndef BOARDS_NATIVE_NATIVE_SIM_BSIM_HW_TESTCHEAT_IF_H
#define BOARDS_NATIVE_NATIVE_SIM_BSIM_HW_TESTCHEAT_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hw_radio_testcheat_set_tx_power_gain(double power_offset);
void hw_radio_testcheat_set_rx_power_gain(double power_offset);
void hw_radio_testcheat_disable_tx(int64_t count);
void hw_radio_testcheat_disable_rx(int64_t count_dont_sync, int64_t count_fail_crc);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_NATIVE_SIM_BSIM_HW_TESTCHEAT_IF_H */
