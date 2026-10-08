# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_link_libraries(subsys__bluetooth)

zephyr_library_sources(
  lll/lll.c
  lll/lll_clock.c
  lll/lll_radio.c
  lll/lll_addr.c
  ../lll_prepare_pipeline.c
  )
zephyr_library_sources_ifdef(
  CONFIG_BT_BROADCASTER
  lll/lll_adv.c
  ../lll_adv_pdu.c
  )
if(CONFIG_BT_OBSERVER)
  zephyr_library_sources(
    lll/lll_scan.c
    )
  zephyr_library_sources_ifdef(
    CONFIG_BT_CTLR_ADV_EXT
    lll/lll_scan_aux.c
    )
endif()
if(CONFIG_BT_CONN)
  zephyr_library_sources(
    lll/lll_conn.c
    )
  zephyr_library_sources_ifdef(
    CONFIG_BT_PERIPHERAL
    lll/lll_peripheral.c
    )
  zephyr_library_sources_ifdef(
    CONFIG_BT_CENTRAL
    lll/lll_central.c
    )
endif()
zephyr_library_sources_ifdef(
  CONFIG_BT_CTLR_LE_ENC
  lll/lll_ccm.c
  )

# The software AES in ecb.c is also used by the LLL for CCM and address
# resolution.
zephyr_library_sources(
  hal/bsim/cntr.c
  hal/bsim/ecb.c
  hal/bsim/mayfly.c
  hal/bsim/ticker.c
  )

zephyr_library_include_directories(
  .
  ../../../controller
  ../../../controller/ll_sw
  ../../../controller/include
  ${ZEPHYR_BASE}/boards/native/common/bsim_2g4_radio
  )

# nrfx_reserved_resources.h includes bt_ctlr_used_resources.h for any library
# when the split controller is enabled on an nRF based board.
zephyr_include_directories(
  hal/bsim/glue
  )

# The radio model runs in the native simulator runner context, next to the
# HW models of the board.
target_sources(native_simulator INTERFACE
  ${ZEPHYR_BASE}/boards/native/common/bsim_2g4_radio/bs_2g4_radio.c
  )
target_compile_options(native_simulator INTERFACE
  -I${ZEPHYR_BASE}/boards/native/common/bsim_2g4_radio
  )
if(CONFIG_SOC_SERIES_BSIM_NRFXX)
  target_sources(native_simulator INTERFACE
    ${ZEPHYR_BASE}/boards/native/nrf_bsim/common/bs_2g4_radio_nrf_bsim.c
    )
endif()

zephyr_library_compile_options_ifdef(
  CONFIG_BT_CTLR_OPTIMIZE_FOR_SPEED
  ${COMPILER_OPTIMIZE_FOR_SPEED_FLAG}
  )
