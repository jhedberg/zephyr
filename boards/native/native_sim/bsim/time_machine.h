/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The bsim tests written for the nrf_bsim boards include this header */

#ifndef BOARDS_NATIVE_NATIVE_SIM_BSIM_TIME_MACHINE_H
#define BOARDS_NATIVE_NATIVE_SIM_BSIM_TIME_MACHINE_H

#include "bs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void tm_set_phy_max_resync_offset(bs_time_t offset_in_us);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_NATIVE_SIM_BSIM_TIME_MACHINE_H */
