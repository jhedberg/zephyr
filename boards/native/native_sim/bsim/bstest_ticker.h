/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The timer of the bstests framework, with the interface of the nRF HW
 * models one, which bstests_entry.c uses. The board has a single CPU, so
 * <inst> is always 0.
 */

#ifndef BOARDS_NATIVE_NATIVE_SIM_BSIM_BSTEST_TICKER_H
#define BOARDS_NATIVE_NATIVE_SIM_BSIM_BSTEST_TICKER_H

#include "bs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void bst_ticker_amp_set_period(unsigned int inst, bs_time_t tick_period);
void bst_ticker_amp_set_next_tick_absolutelute(unsigned int inst, bs_time_t absolute_time);
void bst_ticker_amp_set_next_tick_delta(unsigned int inst, bs_time_t delta_time);
void bst_ticker_amp_awake_cpu_asap(unsigned int inst);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_NATIVE_SIM_BSIM_BSTEST_TICKER_H */
