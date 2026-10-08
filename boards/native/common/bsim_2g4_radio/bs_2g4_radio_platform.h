/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The model reaches the 2G4 Phy through the code all BabbleSim boards share,
 * but the interrupt controller is the board's own, so each board provides
 * this function to the model.
 */

#ifndef BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H
#define BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Raise an interrupt in the CPU running the embedded software */
void bsr_plat_irq_raise(unsigned int irq);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_PLATFORM_H */
