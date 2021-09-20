# LoRa Edge Tracker Reference Design SDK

## 1. Description

The LoRa Edge Tracker Reference Design SDK project contains the tracker application and several simple examples highlighting the LoRa Edge Tracker Reference Design SDK features.

## 1.1. Tracker application

The app joins automatically the network then periodically:
- Perform a Wi-Fi Scan
- Perform a GNSS scan, the ALC Sync is needed
- Collect data from sensor, MCU and Modem
- Stream the scan results	
- Can be configured through BLE

Please read the [application documentation](doc/apps.Tracker.md) for more details.

## 1.2. Simple LoRaWAN Class A/C application 868/915 MHz

This application  joins automatically the LoRa Network server then sends uplinks periodically with the interval defined by APP_TX_DUTYCYCLE.

Please read the [application documentation](doc/apps.LoRaWAN.md) for more details.

## 1.3. Simple GNSS example

The unix date in ASCII is asked to the user through a terminal, once the date is received by the modem, it executes a GNSS scan periodically according to the gnss settings defined in the application.

Please read the [application documentation](doc/apps.gnss_test.md) for more details.


## 1.4. Simple Wi-Fi example

The modem executes a Wi-Fi scan periodically according to the Wi-Fi settings defined in the application.

Please read the [application documentation](doc/apps.wifi_test.md) for more details.

## 1.5. Simple Tx continuous app

The modem starts to send continuous wave.

Please read the [application documentation](doc/apps.tx_continuous.md) for more details.

## 1.6. Simple BLE standalone app

The tracker starts and stays in BLE.

## 1.7. Update modem firmware

Update trx to modem / modem to modem / modem to trx.

## 1.8. Read internal log

Read the internal log and send them by UART.

## 1.9. Simple LoRaWAN clock synchronization example

This application automatically joins the LoRaWAN Network Server. The LR1110 then gets its clock synchronized with the Application Layer Clock Synchronization service (ALC Sync).

Please read the [application documentation](doc/apps.clock_sync.md) for more details.

### 1.10. UART-based firmware update application example

This applications updates the LR1110 firmware. The new firmware is received from the UART.

Please read the [application documentation](doc/apps.uart_firmware_update.md) for more details.

### 1.11. Simple LoRaWAN Data Streaming application

This application joins automatically the LoRa Network Server then streams data periodically.

Please read the [application documentation](doc/apps.stream.md) for more details.

### 1.12. Large File Upload example

The app joins automatically and starts the upload data using the Large File Upload service.

Please read the [application documentation](doc/apps.large_file_upload.md) for more details.## 2. Requirements

## 2. Requirements

### 2.1. Hardware

The example applications are designed to run with the LR1110 Tracker device included in the LoRa Edge Tracker Reference Design Evaluation Kit.

### 2.2. LR1110 Firmware

The applications require that the LR1110 runs the Modem-E firmware version 1.1.7 or later. To update the Modem-E to the latest firmware version please use the Update modem firmware application included in this SDK. The latest firmware can be obtained from [another repository](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110).

Applications usually display the detected LR1110 Firmware version in the serial console when they start, here the LoRaWAN showing version 1.1.7 (0x10107) of the Modem-E firmware:

```
INFO : ###### ===== LoRa Basics Modem-E Tracker demo application ==== ######

APP VERSION : 1.3.0

INFO : ###### ===== LR1110 MODEM-E RESET 555 ==== ######

INFO : ###### ===== LR1110 MODEM-E VERSION ==== ######

LORAWAN     : 0X103
FIRMWARE    : 0X10107
BOOTLOADER  : 0X21DF6500
CLASS       : A
REGION      : EU868
```

## 3. Documentation

LoRa Edge Tracker Reference Design User Guide :
- https://lora-developers.semtech.com/library/tech-papers-and-guides/lora-edge-tracker-reference-design-user-guide/

LoRa Edge™ Tracker Evaluation Actility/Tago Kit User guide :
- https://lora-developers.semtech.com/library/tech-papers-and-guides/lr1110-tracking-evk-user-guide

Node red application server example compliant with LoRa Edge Tracker Reference Design :
- https://github.com/Lora-net/node-red-contrib-loracloud-utils
- https://lora-developers.semtech.com/resources/tools/lora-basics/lora-basics-for-end-nodes/developer-walk-through/

## 4. Build & Install

To build the example application for the STM32WB55 controller of the Tracker board, you will need:

* either a Keil MDK for Cortex-M commercial license, MDK-Lite will not work (https://www2.keil.com/mdk5)
* or the GNU Arm Embedded Toolchain (https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

### 4.1. Building with Keil MDK

Use the Keil project  `lr1110_modem_tracker_sdk.uvprojx` located in the `smtc_tracker_app/MDK-ARM` directory.

Keil creates an hex file containing directly the bootloader and can be flash to the address 0x08000000.

### 4.2. Building with the GNU Arm Embedded Toolchain

Run `make` from the `gcc` directory with the target application name as an argument:

```
$ make APP=tracker
```

Note: the supported application names are `tracker`, `lorawan`, `gnss`, `wifi`, `clock_sync`, `ble_standalone`, `update_modem_app`, `read_internal_log`, `tx_continuous`, `low_power` and `uart_firmware_update`.

The application binary file, `tracker.bin`, is created in the `gcc/build` directory.

When .bin is programmed into the MCU, it shall be flashed to the address 0x08007000, the address starting from 0x08000000 to 0x08007000 is reserved to the bootloader.
