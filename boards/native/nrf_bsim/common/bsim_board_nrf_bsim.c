/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The Phy connection of the nRF HW models (NRF_HWLowL) already has the names
 * bsim_board_if.h uses, so only the rest of it is provided here.
 */

#include "NHW_misc.h"
#include "xo_if.h"
#include "bsim_board_if.h"

void bsim_board_start_offset_set(double offset_us)
{
	xo_model_set_toffset(offset_us);
}

const char *bsim_board_cpu_name_get(unsigned int cpu)
{
	return nhw_get_core_name(cpu);
}
