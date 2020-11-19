# LR1110 modem driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
