/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

/* Return true if the compare matched since the last call, and clear it */
bool cntr_cmp_evt_get_clear(void);
