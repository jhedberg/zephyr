# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

# The BabbleSim boards add this directory, and implement bsim_board_if.h for
# it. trace_hook.c also provides the POSIX arch print functions, which a board
# may have its own of, so it is only built in the runner context here.

find_package(BabbleSim)

zephyr_library()

zephyr_library_sources(
  "argparse.c"
  "bstests_entry.c"
  "nsi_if.c"
)

zephyr_include_directories(
  "."
)

zephyr_library_include_directories(
  "${BSIM_COMPONENTS_PATH}/libUtilv1/src/"
  "${BSIM_COMPONENTS_PATH}/libPhyComv1/src/"
)

target_sources(native_simulator INTERFACE
  "bsim_args_runner.c"
  "bsim_extra_cpu_if_stubs.c"
  "phy_sync_ctrl.c"
  "runner_hooks.c"
  "trace_hook.c"
)

target_compile_options(native_simulator INTERFACE
  "-I${CMAKE_CURRENT_SOURCE_DIR}"
  "-I${BSIM_COMPONENTS_PATH}/libUtilv1/src/"
  "-I${BSIM_COMPONENTS_PATH}/libPhyComv1/src/"
  "-I${BSIM_COMPONENTS_PATH}/ext_2G4_libPhyComv1/src/"
  "-I${BSIM_COMPONENTS_PATH}/libRandv2/src/"
  "-DNSI_PRIMARY_MCU_N=${CONFIG_NATIVE_SIMULATOR_PRIMARY_MCU_INDEX}"
)

set(libpath "${BSIM_OUT_PATH}/lib")
set_property(TARGET native_simulator APPEND PROPERTY RUNNER_LINK_LIBRARIES
  "${libpath}/libUtilv1.32.a"
  "${libpath}/libPhyComv1.32.a"
  "${libpath}/lib2G4PhyComv1.32.a"
  "${libpath}/libRandv2.32.a"
)
