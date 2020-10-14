# LR1110 modem tracker sdk

## 1. Description

The LR1110 modem tracker SDK project contains the tracker application and several simple examples highlighting lr1110 modem tracker features :

Tracker application :
-	The app joins automatically the network then periodically :
	- Perform a Wi-Fi Scan
	- Perform a GNSS scan, the alcsync is needed
	- Collect data from sensor , MCU and Modem
	- Stream the scan results	
	- Can be configured through BLE
Simple LoRaWAN Class A application 868/915 MHz:
-	The app joins automatically the network then sends uplinks periodically
Simple GNSS example :
-	The unix date in ASCII is asked to the user through a terminal, once the date is received by the modem, it executes a GNSS scan periodically according to the gnss settings defined in the application
Simple Wi-Fi example :
-	The modem executes a Wi-Fi scan periodically according to the Wi-Fi settings defined in the application
Simple Tx continuous app :
-	The modem starts to send continuous wave
Simple BLE standalone app :
-	The tracker starts and stays in BLE
Update modem firmware :
-	Update trx to modem / modem to modem / modem to trx
Read internal log :
-	Read the internal log and send them by UART

## 2. Build & Install

To build the host softwares, proceed as follow:

-	use the Keil project, or
-	execute the makefile (in gcc folder)

## 3. Changelog

### V 1.1.1 ###

-	Remove infinite loop in HardFault Handler / Error Handler / Hal mcu panic and replaced by hal_mcu_reset

### V 1.1.0 ###

-	Test with LoRa Basics Modem 1.0.7 firmware version
-	Fix a flash read overflow in secure memory region during init
-	Wi-Fi results were sent even if Wi-Fi feature was disable if the last scan was successful 
- 	Code refactoring
- 	Code format
-	Pin code is regenerated each time that Dev or Join EUI is changed
-	Add Get modem status BLE command
-	Add Get Chip EUI BLE command
-	Fix negative GNSS assistance position
-	Watchdog management improvement when BLE is connected
-	Disable hall effect sensor when BLE is active preventing crash
-	Add assistance position update by downlink

### V 1.0.0 ###

- 	Add internal log capability
-	Added enable/disable internal log over BLE
-	Added read internal log over BLE
-	Watchdog bug fix when scan_internal was smaller than 15s 
-	General code cleaning
-	General documentation

### V 0.1.3 ###

- 	Operation on LR1110 modem 1.0.4 firmware version
-	Added temperature reading from accelerometer
-	GNSS set constellation fix in BLE thread
-	Added LR1110 modem update over BLE
-	Added accelerometer IRQ operation in static mode to retart quicker when tracker is static
-	GNSS assistance position managed more properly
-	Added set ADR profile configurable over BLE
-	Added get chip EUI over BLE
-	Added get board voltage over BLE
-	Added MCU Vref internal reading
-	Added TX power offset to +2dB on EU868 band 
-	Tracker switches in airplane mode is VRef is below the defined voltage threshold
-	General code cleaning
-	General documentation
-	Added make file
-	Added README.md 

Known limitations:

-	Behavior not perfect on US band without 64 channels gateway. If 8 channels gateway used, Frquency plan shall be received with the join accept
-	Not working on TTN: TTN v2 US non functional (requires stack 1.0.2, modem uses 1.0.3). running on TTNv3, but TTNv3 not officialy released. 

### V 0.1.2 ###

-	Add airplane mode
-	Power consumption optimization
-	Add mobile and low range ADR in tracker app instead of network controlled ADR
-	Set/Get Set Do/Don’t perform GNSS When Wi-Fi result is enough BLE command supported in the firmware and compliant with smartphone app V1.7
-	Bug fix concerning watchdog when : Join process interval increase / scan interval larger than 6min
-	Update get modem version command
-	Update set/get wi-fi channels ble command
-	General code improvement
-	Semtech licenses updated
-	LR1110 modem driver update and upper layers according to it

Known limitations:

-	unstable behavior on V2B hardware
-	not working on TTN: TTN v2 US non functional (requires stack 1.0.2, modem uses 1.0.3). running on TTNv3, but TTNv3 not released. Not corrected by R&D. Not confirmed: different

### V 0.1.1 ###

-	Set/Get Region BLE command supported in the firmware and compliant with smartphone app V1.6
-	Set/Get Use Semtech Join Sever usage (default = use Semtech Join Server) BLE command supported in the firmware but not implemented in the smartphone app V1.6
-	Get Pin BLE command supported in the firmware but not implemented in the smartphone app V1.6
-	Low app duty cycle formatted in minutes instead of second for smartphone app V1.6
-	Fix app duty cycle which was using the fixed value instead of the one set though the smartphone app
