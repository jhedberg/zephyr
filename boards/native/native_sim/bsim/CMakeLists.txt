# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

add_subdirectory("${ZEPHYR_BASE}/boards/native/common/bsim"
  "${CMAKE_CURRENT_BINARY_DIR}/common_bsim"
)

# The nRF HW models have the connection to the Phy, a clock drift model, the
# Bluetooth LE CRC, and the interfaces of the test ticker and test cheats that
# the bsim tests use, none of which are specific to the nRF SoCs. Their
# interrupt controller has the same header name as the one of the native
# simulator, which is found first.
set(hw_models_dir "${ZEPHYR_NRF_HW_MODELS_MODULE_DIR}/src/HW_models")

zephyr_include_directories(
  "."
  "${hw_models_dir}"
)

# The BabbleSim boards parse the command line and trace with BabbleSim, so
# this takes the native simulator models without the command line parser and
# the tracing that NSI_NATIVE would add.
target_sources(native_simulator INTERFACE
  "${NSI_DIR}/native/src/hw_counter.c"
  "${NSI_DIR}/native/src/irq_ctrl.c"
  "${NSI_DIR}/native/src/native_rtc.c"
  "${NSI_DIR}/native/src/timer_model.c"
  "${hw_models_dir}/NRF_HWLowL.c"
  "${hw_models_dir}/trivial_xo.c"
  "bsim_board_native_sim.c"
  "bstest_ticker.c"
  "cmdline_runner.c"
)

target_compile_options(native_simulator INTERFACE
  "-I${NSI_DIR}/native/src/include"
  "-I${hw_models_dir}"
)

if(CONFIG_DT_HAS_ZEPHYR_BSIM_2G4_RADIO_ENABLED)
  add_subdirectory("${ZEPHYR_BASE}/boards/native/common/bsim_2g4_radio"
    "${CMAKE_CURRENT_BINARY_DIR}/bsim_2g4_radio"
  )
  target_sources(native_simulator INTERFACE
    "${hw_models_dir}/crc.c"
    "bs_2g4_radio_native_sim.c"
  )
endif()
