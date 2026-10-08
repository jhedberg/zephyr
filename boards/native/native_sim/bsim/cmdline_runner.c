/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nsi_cmdline.h"

/* From bs_cmd_line.h, which defines the same macros as nsi_cmdline.h */
void bs_add_extra_dynargs(struct args_struct_t *args_struct_toadd);

/* The native simulator models and the native_sim drivers register their
 * command line options with this, while the BabbleSim boards parse the
 * command line with the BabbleSim parser, whose option table has the same
 * layout.
 */
void nsi_add_command_line_opts(struct args_struct_t *args)
{
	bs_add_extra_dynargs(args);
}
