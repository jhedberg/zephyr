/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "bs_cmd_line.h"
#include "nsi_tasks.h"
#include "bsim_args_runner.h"
#include "xo_if.h"
#include "bsim_board_if.h"

/* As on the nrf_bsim boards, the Phy connection and the clock drift model of
 * the nRF HW models (NRF_HWLowL and trivial_xo), which are not specific to the
 * nRF SoCs, provide the rest of bsim_board_if.h.
 */
void bsim_board_start_offset_set(double offset_us)
{
	xo_model_set_toffset(offset_us);
}

const char *bsim_board_cpu_name_get(unsigned int cpu)
{
	(void)cpu;

	return "";
}

/* The bsim tests written for the nrf_bsim boards select the AES model of the
 * nRF HW models with this option, which this board has nothing to apply to.
 */
static bool real_aes;

static void register_args(void)
{
	static bs_args_struct_t args_struct_toadd[] = {
		{
		.option = "RealEncryption",
		.name = "realAES",
		.type = 'b',
		.dest = (void *)&real_aes,
		.descript = "Ignored, for compatibility with the nrf_bsim boards"
		},
		ARG_TABLE_ENDMARKER
	};

	bs_add_extra_dynargs(args_struct_toadd);
}

NSI_TASK(register_args, PRE_BOOT_1, 100);
