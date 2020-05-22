zephyr_include_directories(src)

zephyr_library_sources(
    src/nrf_802154.c
    src/nrf_802154_core.c
    src/nrf_802154_core_hooks.c
    src/nrf_802154_critical_section.c
    src/nrf_802154_debug.c
    src/nrf_802154_pib.c
    src/nrf_802154_queue.c
    src/nrf_802154_rssi.c
    src/nrf_802154_rx_buffer.c
    src/nrf_802154_stats.c
    src/nrf_802154_trx.c
    src/mac_features/nrf_802154_csma_ca.c
    src/mac_features/nrf_802154_delayed_trx.c
    src/mac_features/nrf_802154_filter.c
    src/mac_features/nrf_802154_frame_parser.c
    src/mac_features/nrf_802154_ifs.c
    src/mac_features/nrf_802154_precise_ack_timeout.c
    src/mac_features/ack_generator/nrf_802154_ack_data.c
    src/mac_features/ack_generator/nrf_802154_ack_generator.c
    src/mac_features/ack_generator/nrf_802154_enh_ack_generator.c
    src/mac_features/ack_generator/nrf_802154_imm_ack_generator.c
    src/platform/random/nrf_802154_random_zephyr.c
    src/platform/temperature/nrf_802154_temperature_none.c
    src/nrf_802154_notification_swi.c
    src/nrf_802154_request_swi.c
    src/nrf_802154_swi.c
)

if(     CONFIG_NRF_802154_CCA_MODE_ED)
  set(radio_cca_mode NRF_RADIO_CCA_MODE_ED)

elseif( CONFIG_NRF_802154_CCA_MODE_CARRIER)
  set(radio_cca_mode NRF_RADIO_CCA_MODE_CARRIER)

elseif( CONFIG_NRF_802154_CCA_MODE_CARRIER_AND_ED)
  set(radio_cca_mode NRF_RADIO_CCA_MODE_CARRIER_AND_ED)

elseif( CONFIG_NRF_802154_CCA_MODE_CARRIER_OR_ED)
  set(radio_cca_mode NRF_RADIO_CCA_MODE_CARRIER_OR_ED)

endif()

zephyr_compile_definitions(
  # Until IRQ configuration abstraction is added to the radio driver do not set
  # radio IRQ priority to 0, or the radio IRQ will ignore IRQ lock.
  NRF_802154_IRQ_PRIORITY=1
  NRF_802154_INTERNAL_RADIO_IRQ_HANDLING=0
  NRF_802154_INTERNAL_SWI_IRQ_HANDLING=0

  # Radio driver shim layer uses raw api
  NRF_802154_USE_RAW_API=1

  # Number of slots containing short addresses of nodes for which
  # pending data is stored.
  NRF_802154_PENDING_SHORT_ADDRESSES=16

  # Number of slots containing extended addresses of nodes for which
  # pending data is stored.
  NRF_802154_PENDING_EXTENDED_ADDRESSES=16

  # Number of buffers in receive queue.
  NRF_802154_RX_BUFFERS=8

  # CCA mode
  NRF_802154_CCA_MODE_DEFAULT=${radio_cca_mode}

  # CCA mode options
  NRF_802154_CCA_CORR_LIMIT_DEFAULT=${CONFIG_NRF_802154_CCA_CORR_LIMIT}
  NRF_802154_CCA_CORR_THRESHOLD_DEFAULT=${CONFIG_NRF_802154_CCA_CORR_THRESHOLD}
  NRF_802154_CCA_ED_THRESHOLD_DEFAULT=${CONFIG_NRF_802154_CCA_ED_THRESHOLD}

  # Enable CSMA/CA
  NRF_802154_CSMA_CA_ENABLED=1
  NRF_802154_TX_STARTED_NOTIFY_ENABLED=1

  # Enable ack timeout
  NRF_802154_ACK_TIMEOUT_ENABLED=1

  # Enable DTRX
  NRF_802154_DELAYED_TRX_ENABLED=1

  # Disable unused radio driver features
  NRF_802154_FRAME_TIMESTAMP_ENABLED=0
)
