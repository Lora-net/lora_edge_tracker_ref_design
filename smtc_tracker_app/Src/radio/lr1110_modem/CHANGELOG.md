# LR1110 modem driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v3.1.0] 2021-08-06

### Added

* `lr1110_modem_set_crystal_error()` and `lr1110_modem_set_crystal_error()` functions
* `lr1110_modem_helper_get_event_data()` helper function
* `lr1110_modem_event_t` structure definition

## [v3.0.1] 2021-05-11

### Fixed

* Documentation for `lr1110_modem_get_duty_cycle_status()` - definition of the parameter based on its sign

## [v3.0.0] 2021-05-07

### Added

* `LR1110_LORAWAN_REGION_AS923_GRP1/2/3`, `LR1110_LORAWAN_REGION_AU915`, `LR1110_LORAWAN_REGION_CN470`, `LR1110_LORAWAN_REGION_IN865`, `LR1110_LORAWAN_REGION_KR920` and `LR1110_LORAWAN_REGION_RU864` in `lr1110_modem_regions_t`
* `lr1110_modem_system_calibrate()` function
* `lr1110_modem_get_connection_timeout_status()` function
* `lr1110_modem_get_available_data_rate()` function
* `lr1110_modem_activate_lbt()` function, with `lr1110_modem_lbt_mode_t` type definition
* `lr1110_modem_set_output_power_config()` and `lr1110_modem_get_output_power_config()` functions, with associated `lr1110_modem_output_power_config_t` type
* `lr1110_modem_get_network_type()` and `lr1110_modem_set_network_type()` functions
* `lr1110_modem_set_nb_trans()` and `lr1110_modem_get_nb_trans()` functions
* `lr1110_modem_set_stream_redundancy_rate()` and `lr1110_modem_get_stream_redundancy_rate()` functions
* `LR1110_MODEM_DOWN_DATA_EVENT_RXC` entry in `lr1110_modem_down_data_flag_t`
* `lr1110_modem_helper_get_utc_time()`, `lr1110_modem_helper_gnss_get_gnss_week_number_rollover()` and `lr1110_modem_helper_gnss_get_almanac_date_by_index()` helper functions
* `lr1110_modem_helper_gnss_get_result_destination()` and `lr1110_modem_helper_gnss_get_event_type()` helper functions
* `lr1110_modem_hal_write_without_rc()` function - to be used when no return code is expected
* `LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON` entry in `lr1110_modem_wifi_mode_t`
* `lr1110_modem_wifi_fcs_info_byte_t` structure definition
* `lr1110_modem_wifi_extended_full_result_t` structure definition
* `lr1110_modem_wifi_read_extended_full_results()` function
* `lr1110_modem_ramp_time_t` and `lr1110_modem_pa_reg_supply_t` type definitions
* `lr1110_modem_output_power_config_t` structure definition
* `lr1110_modem_network_type_t` type definition
* `lr1110_modem_set_output_power_config()` and `lr1110_modem_get_output_power_config()` functions

### Changed

* `LR1110_MODEM_FILE_ENCRYPTION_*` is renamed `LR1110_MODEM_SERVICES_ENCRYPTION_*` in `lr1110_modem_encryption_mode_t`
* `LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_*HZ` is renamed `LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_*_HZ` in `lr1110_modem_gnss_frequency_search_space_t`
* `lr1110_modem_gnss_dmc_error_code_t` is renamed `lr1110_modem_gnss_context_status_error_code_t`
* `LR1110_MODEM_GNSS_DMC_*` is renamed `LR1110_MODEM_GNSS_CONTEXT_STATUS_NO_ERROR` in `lr1110_modem_gnss_context_status_error_code_t`
* `lr1110_modem_wifi_read_complete_results()` function is renamed `lr1110_modem_wifi_read_basic_complete_results()`
* `lr1110_modem_wifi_read_basic_results()` function is renamed `lr1110_modem_wifi_read_basic_mac_type_channel_results()`
* `lr1110_modem_system_reboot()` function now returns a response code
* `LR1110_MODEM_REGIONS_NUMBER` constant is now equal to 10
* `LR1110_MODEM_LORWAN_*` are renamed `LR1110_MODEM_LORAWAN_*` in `lr1110_modem_lorawan_state_t`
* `event_count` is renamed `missed_events_count` in `lr1110_modem_event_fields_t`
* `lr1110_modem_get_dm_info_interval()` function now takes `lr1110_modem_reporting_interval_format_t*` as additional parameter
* 'lr1110_modem_test_rssi()' function is renamed 'lr1110_modem_test_rssi_subghz()'
* `lr1110_modem_gnss_scan_*_md()` functions are renamed `lr1110_modem_gnss_scan_*()`
* `lr1110_modem_gnss_input_parameters_e` is renamed `lr1110_modem_gnss_result_mask_e`

### Fixed

* Typo in enumerated constants in `lr1110_modem_lorawan_state_t`
* Implementation of `lr1110_modem_get_dm_info_interval` function did not take `format` parameter into account
* Parameter `duty_cycle` in `lr1110_modem_get_duty_cycle_status()` function changed from `uint32_t` to `int32_t`
* Wrong parameter type in `lr1110_modem_send_dm_status` function
* GNSS bit mask parameters `LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK` and `LR1110_MODEM_GNSS_BIT_CHANGE_MASK` are inverted
* `LR1110_MODEM_TX_ERROR` constant value in `lr1110_modem_tx_done_event_t` is wrong
* `LR1110_MODEM_LORAWAN_BUSY` constant value in `lr1110_modem_lorawan_state_t` is wrong

### Removed

* `LR1110_MODEM_GROUP_ID_CRYPTO` entry in `lr1110_modem_api_group_id_t`
* `LR1110_MODEM_GNSS_DESTINATION_DMC` entry in `lr1110_modem_gnss_destination_t`
* `LR1110_MODEM_TEST_MODE_TST_BUSY_LOOP`, `LR1110_MODEM_TEST_MODE_TST_PANIC` and `LR1110_MODEM_TEST_MODE_TST_WATCHDOG` entries from `lr1110_modem_test_mode_t`
* `lr1110_modem_test_busy_loop()`, `lr1110_modem_test_panic()` and `lr1110_modem_test_watchdog()` functions
* `lr1110_modem_system_write_auxreg32()` and `lr1110_modem_system_read_auxreg32()` functions
* `LR1110_LORAWAN_BROWNOUT` from `lr1110_modem_status_t`
* `lr1110_modem_wifi_cfg_hardware_debarker()` function
* `LR1110_MODEM_RESPONSE_CODE_NOT_IMPLEMENTED`, `LR1110_MODEM_RESPONSE_CODE_BAD_FORMAT` and `LR1110_MODEM_RESPONSE_CODE_BAD_SIGNATURE` response codes

## [v2.0.1] 2020-10-28

### Fixed

* Removed `LR1110_MODEM_DOWN_DATA_EVENT_ACK` and `LR1110_MODEM_DOWN_DATA_EVENT_NACK` from `lr1110_modem_down_data_flag_t`

## [v2.0.0] 2020-10-23

### Changed

* `lr1110_modem_almanac_read_by_index` is renamed `lr1110_modem_gnss_almanac_read_by_index`
* `lr1110_modem_gnss_scan_autonomous_md` takes an effort mode as parameter

### Fixed

* Various compilation warnings
* Harmonization of the documentation

## [v1.0.0] 2020-10-19

### Added

* Initial release
