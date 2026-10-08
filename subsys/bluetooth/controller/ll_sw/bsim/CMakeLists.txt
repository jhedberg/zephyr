# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_link_libraries(subsys__bluetooth)

zephyr_library_sources(
  "lll/lll.c"
  "lll/lll_clock.c"
  "lll/lll_radio.c"
  "lll/lll_addr.c"
  "../lll_prepare_pipeline.c"
)

if(CONFIG_BT_BROADCASTER)
  zephyr_library_sources(
    "lll/lll_adv.c"
    "../lll_adv_pdu.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_ADV_EXT
    "lll/lll_adv_aux.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_ADV_PERIODIC
    "lll/lll_adv_sync.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_ADV_ISO
    "lll/lll_adv_iso.c"
  )
endif()

if(CONFIG_BT_OBSERVER)
  zephyr_library_sources(
    "lll/lll_scan.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_ADV_EXT
    "lll/lll_scan_aux.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_SYNC_PERIODIC
    "lll/lll_sync.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_SYNC_ISO
    "lll/lll_sync_iso.c"
  )
endif()

if(CONFIG_BT_CONN)
  zephyr_library_sources(
    "lll/lll_conn.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_PERIPHERAL
    "lll/lll_peripheral.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CENTRAL
    "lll/lll_central.c"
  )
  zephyr_library_sources_ifdef(CONFIG_BT_CTLR_CONN_ISO
    "lll/lll_conn_iso.c"
  )
endif()

if(CONFIG_BT_CTLR_LE_ENC OR CONFIG_BT_CTLR_BROADCAST_ISO_ENC)
  zephyr_library_sources(
    "lll/lll_ccm.c"
  )
endif()

# The LLL uses the software AES of ecb.c for CCM and address resolution too
zephyr_library_sources(
  "hal/bsim/cntr.c"
  "hal/bsim/ecb.c"
  "hal/bsim/ticker.c"
  "../../hal/mayfly.c"
  "../../hal/ticker.c"
)

zephyr_library_include_directories(
  "."
  "${ZEPHYR_BASE}/subsys/bluetooth/controller"
  "${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw"
  "${ZEPHYR_BASE}/subsys/bluetooth/controller/include"
)

# On nRF based boards, nrfx_reserved_resources.h includes
# bt_ctlr_used_resources.h in every library when the split controller is
# enabled.
zephyr_include_directories("hal/bsim/glue")

zephyr_library_compile_options_ifdef(CONFIG_BT_CTLR_OPTIMIZE_FOR_SPEED
  "${COMPILER_OPTIMIZE_FOR_SPEED_FLAG}"
)
