/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The BabbleSim boards share the code in this folder, but each one reaches
 * the Phy, and models the clock of the device, with its own HW models. So
 * each board provides these functions, in the runner context.
 */

#ifndef BOARDS_NATIVE_COMMON_BSIM_BSIM_BOARD_IF_H
#define BOARDS_NATIVE_COMMON_BSIM_BSIM_BOARD_IF_H

#include <stdbool.h>
#include "bs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Connect to Phy <p> of simulation <s> as device <d> */
int hwll_connect_to_phy(unsigned int d, const char *s, const char *p);

/* Leave the simulation, and let it go on without this device */
void hwll_disconnect_phy(void);

/* Leave the simulation, and end it */
void hwll_terminate_simulation(void);

/* Run without connecting to a Phy */
void hwll_set_nosim(bool new_nosim);

/* Wait until the Phy reaches device time <d_t> */
void hwll_sync_time_with_phy(bs_time_t d_t);

/* Wait until the Phy reaches Phy time <phy_time> */
void hwll_wait_for_phy_simu_time(bs_time_t phy_time);

/* Convert a device time into a Phy time, and back */
bs_time_t hwll_phy_time_from_dev(bs_time_t d_t);
bs_time_t hwll_dev_time_from_phy(bs_time_t p_t);

/* Leave the simulation, and exit */
void hwll_disconnect_phy_and_exit(void);

/* Phy time at which the device time starts, in microseconds */
void bsim_board_start_offset_set(double offset_us);

/* Name of MCU <cpu> of the device */
const char *bsim_board_cpu_name_get(unsigned int cpu);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_COMMON_BSIM_BSIM_BOARD_IF_H */
