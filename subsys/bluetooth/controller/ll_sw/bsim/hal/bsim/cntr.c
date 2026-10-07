/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/sys/util.h>

#include "hal/cntr.h"

#include "bs_2g4_radio_if.h"

/* The counter is the free running 1MHz counter of the BabbleSim radio model,
 * it cannot be stopped. Hence, start and stop only keep a reference count.
 */
static uint8_t refcount;

void cntr_init(void)
{
	bsr_cntr_cmp_disable();
}

uint32_t cntr_start(void)
{
	if (refcount++) {
		return 1;
	}

	return 0;
}

uint32_t cntr_stop(void)
{
	__ASSERT_NO_MSG(refcount);

	if (--refcount) {
		return 1;
	}

	return 0;
}

uint32_t cntr_cnt_get(void)
{
	return bsr_cntr_get();
}

void cntr_cmp_set(uint8_t cmp, uint32_t value)
{
	ARG_UNUSED(cmp);

	bsr_cntr_cmp_set(value);
}

bool cntr_cmp_evt_get_clear(void)
{
	return bsr_cntr_cmp_evt_get_clear();
}
