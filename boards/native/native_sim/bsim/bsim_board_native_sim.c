/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "bs_types.h"
#include "bs_tracing.h"
#include "bs_cmd_line.h"
#include "bs_pc_2G4.h"
#include "nsi_tasks.h"
#include "bsim_args_runner.h"
#include "bsim_board_if.h"

static bool nosim;
/* The board has no clock drift model, so the device time only lags the Phy
 * time by the start offset.
 */
static bs_time_t start_offset;

void hwll_set_nosim(bool new_nosim)
{
	nosim = new_nosim;
}

void bsim_board_start_offset_set(double offset_us)
{
	start_offset = (bs_time_t)(offset_us + 0.5);
}

const char *bsim_board_cpu_name_get(unsigned int cpu)
{
	(void)cpu;

	return "";
}

bs_time_t hwll_phy_time_from_dev(bs_time_t d_t)
{
	if (d_t == TIME_NEVER) {
		return TIME_NEVER;
	}

	return d_t + start_offset;
}

bs_time_t hwll_dev_time_from_phy(bs_time_t phy_t)
{
	if (phy_t == TIME_NEVER) {
		return TIME_NEVER;
	}

	return phy_t - start_offset;
}

int hwll_connect_to_phy(unsigned int d, const char *s, const char *p)
{
	if (nosim) {
		return 0;
	}

	return p2G4_dev_initcom_nc(d, s, p);
}

void hwll_disconnect_phy(void)
{
	if (!nosim) {
		p2G4_dev_disconnect_nc();
	}
}

void hwll_disconnect_phy_and_exit(void)
{
	hwll_disconnect_phy();
	bs_trace_exit_line("\n");
}

void hwll_terminate_simulation(void)
{
	if (!nosim) {
		p2G4_dev_terminate_nc();
	}
}

void hwll_wait_for_phy_simu_time(bs_time_t phy_time)
{
	pb_wait_t wait = { .end = phy_time };

	if (nosim) {
		return;
	}

	if (p2G4_dev_req_wait_nc_b(&wait) != 0) {
		bs_trace_raw_manual_time(3, phy_time, "The phy disconnected us\n");
		hwll_disconnect_phy_and_exit();
	}
}

void hwll_sync_time_with_phy(bs_time_t d_t)
{
	/* Stop short of d_t, as a Tx or Rx may start at d_t, which the Phy
	 * would then see as in the past.
	 */
	if (d_t == TIME_NEVER) {
		hwll_wait_for_phy_simu_time(TIME_NEVER);
	} else {
		hwll_wait_for_phy_simu_time(hwll_phy_time_from_dev(d_t - 2));
	}
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
