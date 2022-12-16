# LR1110 modem application example changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

### [v1.5.0] 2022-08-17 ###

### Changed
* TX power adapted for Japan / IN865 / AU915 / KR920 regions

### Fixed
* Disable almanac update by BLE when LR1110 firmware not present

### [v1.4.0] 2022-03-16 ###

### Added
* Add BLE/LoRaWAN GET_TRACKER_TYPE_CMD

### Changed
* Application startup improvement when LR1110 firmware is not present

### Fixed
* Change LBT bandwidth from 1.25MHz to 125KHz for Japan region
* Change LBT bandwidth from 1.25MHz to 125KHz for KR920 region
* Tracker was losting the BLE connection after a Modem-E firmware update
* HSE capacitors tuning to fix the frequency drift in BLE.
* LSCO jitter stability
* Clear accelerometer IRQ at startup

### [v1.3.0] 2021-09-06 ###

### Added

* Modem-E V1.1.7 support
* Support all LoRa Basics Modem-E region
* Define ADR custom list for regions using duty cycle (EU868 and RU864)
* Stream data only if Modem-E has enough time to send it
* Add new atomic examples
* Send all sensors values only if scan results are not good enough or when send keep alive frame otherwise send only the accelerometer move history
* Replace TLV Wi-Fi (0x08) by a new one (0x0E) with timestamp
* Add algorithm prioritizing the GNSS and Wi-Fi scan and send only one scan result 
* Add scan priority
* Check validity of NAV message before send it
* Add read/write tracker parameters over LoRaWAN
* Add Host and Modem reset counter
* Add auto diagnosis support
* Add CHANGELOG.md

### Changed

* Update LR1110 Modem-E drivers
* Improve stream usage
* Add more documentation
* Improve BLE connectivity stability
* Change the GNSS result mask to only pseudo range
* Improve LNA supply management during GNSS scan
* Power consumption improvement in airplane mode
* Refactor sensor TLV
* Refactor GNSS and Wi-Fi threads
* Replace temperature from accelerometer to MCU
* Internal log improvement, add tracker settings, Wi-Fi channels, type and timings and GNSS timings
* Battery low detection method improved
* Reduce the number of executed scan once the tracker static to one.
* Send only one NAV message even both antenna are selected

### Fixed

* minor bug fixes

### [v1.2.0] 2020-11-12  ###

### Added

* Add Get date BLE command
* Add Get/Flush Accumulated charge BLE command
* Add Internal Log memory space remaining BLE command
* Add empty GNSS scan in internal log

### Changed

* Merge LoRaWAN commissioning files for tracker and lorawan app
* Update LR1110 Modem-E driver

### Fixed

* Fix the SNR reading which was wrong
* Fix RNG module

### [v1.1.1] 2020-10-23 ###

### Added

* Add STM32WB SMPS (switched-mode power supply) configuration in application and bootloader

### [v1.1.1] 2020-10-02 ###

### Fixed

* Remove infinite loop in HardFault Handler / Error Handler / Hal mcu panic and replaced by hal_mcu_reset

### [v1.1.0] 2020-09-30 ###

### Added

* Test with LoRa Basics Modem 1.0.7 firmware version
* Add Get modem status BLE command
* Add Get Chip EUI BLE command
* Disable hall effect sensor when BLE is active preventing crash
* Add assistance position update by downlink

### Changed

* Pin code is regenerated each time that Dev or Join EUI is changed
* Code refactoring
* Code format
* Watchdog management improvement when BLE is connected

### Fixed

* Fix a flash read overflow in secure memory region during init
* Wi-Fi results were sent even if Wi-Fi feature was disable if the last scan was successful 
* Fix negative GNSS assistance position

### [v1.0.0] 2020-08-27 ###

### Added

* Add internal log capability
* Added enable/disable internal log over BLE
* Added read internal log over BLE

### Changed

* General code cleaning
* Update General documentation

### Fixed

* Watchdog bug fix when scan_internal was smaller than 15s 

### [v0.1.3] 2020-08-18 ###

### Added

* Operation on LR1110 modem 1.0.4 firmware version
* Added temperature reading from accelerometer
* Added LR1110 modem update over BLE
* Added accelerometer IRQ operation in static mode to retart quicker when tracker is static
* Added set ADR profile configurable over BLE
* Added get chip EUI over BLE
* Added get board voltage over BLE
* Added MCU Vref internal reading
* Added TX power offset to +2dB on EU868 band 
* Tracker switches in airplane mode is VRef is below the defined voltage threshold
* Added make file
* Added README.md 

### Changed

* GNSS assistance position managed more properly
* General code cleaning
* General documentation

### Fixed

* GNSS set constellation fix in BLE thread

### Known limitations:

* Behavior not perfect on US band without 64 channels gateway. If 8 channels gateway used, Frquency plan shall be received with the join accept
* Not working on TTN: TTN v2 US non functional (requires stack 1.0.2, modem uses 1.0.3). running on TTNv3, but TTNv3 not officialy released. 

### [v0.1.2] 2020-08-18 2020-08-01 ###

### Added

* Add airplane mode
* Add mobile and low range ADR in tracker app instead of network controlled ADR
* Set/Get Set Do/Don’t perform GNSS When Wi-Fi result is enough BLE command supported in the firmware and compliant with smartphone app V1.7

### Changed

* Power consumption optimization
* General code improvement
* Semtech licenses updated
* Update get modem version command
* Update set/get wi-fi channels ble command
* LR1110 modem driver update and upper layers according to it

### Fixed

* fix watchdog when : Join process interval increase / scan interval larger than 6min

### Known limitations:

* unstable behavior on V2B hardware
* not working on TTN: TTN v2 US non functional (requires stack 1.0.2, modem uses 1.0.3). running on TTNv3, but TTNv3 not released. Not corrected by R&D. Not confirmed: different

### [v0.1.1] 2020-07-18 ###

### Added

* Set/Get Region BLE command supported in the firmware and compliant with smartphone app V1.6
* Set/Get Use Semtech Join Sever usage (default = use Semtech Join Server) BLE command supported in the firmware but not implemented in the smartphone app V1.6
* Get Pin BLE command supported in the firmware but not implemented in the smartphone app V1.6

### Changed

* Low app duty cycle formatted in minutes instead of second for smartphone app V1.6

### Fixed

* Fix app duty cycle which was using the fixed value instead of the one set though the smartphone app

### [v0.1.1] 2020-06-17 ###

* Initiale release
