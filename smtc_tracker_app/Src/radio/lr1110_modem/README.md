# lr1110_modem_sw_library

This project proposes an implementation in C of the drivers for LR1110 modem:

LoRaWAN communication
Wi-Fi Sniff
GNSS Sniff

It does not involve any state machine or high level API.

# LR1110_modem_driver project

This package proposes an implementation in C of the driver for **LR1110** modem radio component.

## Components

The driver is splitted in several components:

- Bootloader
- System configuration & Register / memory access
- LoRaWAN
- Wi-Fi Passive Scanning
- GNSS Scan Scanning

### Bootloader

This component is used to update the firmware.

### System configuration & Register / memory access

This component is used to interact with system-wide parameters like clock sources, integrated RF switches, etc and read / write data from registers or internal memory.

### LoRaWAN

This component is used to send / receive data through LoRaWAN.

### Wi-Fi Passive Scanning

This component is used to configure and initiate the passive scanning of the Wi-Fi signals that can be shared to request a geolocation.

### GNSS Scanning

This component is used to configure and initiate the acquisition of GNSS signals that can be shared to request a geolocation.

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- lr1110_hal_reset
- lr1110_hal_wakeup
- lr1110_hal_write
- lr1110_hal_read
- lr1110_hal_write_read

- lr1110_modem_hal_reset
- lr1110_modem_hal_wakeup
- lr1110_modem_hal_write
- lr1110_modem_hal_read
- lr1110_modem_hal_write_read
