/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Functions each BabbleSim platform provides to the generic packet level
 * 2.4GHz radio model (bs_2g4_radio.c).
 *
 * The model talks to the 2G4 Phy through the libPhyCom "_nc" API, using the
 * connection the platform has opened for this device. The platform provides
 * the device <-> Phy time conversion, interrupt raising and the keeping track
 * of when the device last synchronized with the Phy.
 */

#ifndef BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H
#define BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H

#include "bs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Convert a device time into a Phy time, and back */
bs_time_t bsr_plat_phy_time_from_dev(bs_time_t dev_time);
bs_time_t bsr_plat_dev_time_from_phy(bs_time_t phy_time);

/* Raise an interrupt in the CPU running the embedded software */
void bsr_plat_irq_raise(unsigned int irq);

/* The device has synchronized with the Phy up to the given device time */
void bsr_plat_phy_synced(bs_time_t dev_time);

/* The Phy disconnected this device */
void bsr_plat_phy_disconnected(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H */
