# Simple LR1110 Wi-Fi example application

## 1. Description

This application performs a scan at regular intervals and display information about scanned access points.

## 2. Configuration 

The following constants are defined in the `main_simple_wifi_example.h` and can be changed to suit the planned test:

| Constant              | Comments |
| --------------------- | -------- |
| `WIFI_SCAN_PERIOD_MS` | Wi-Fi scanning interval between two scans, in milliseconds. |
| `WIFI_TYPE_SCAN` | Wi-Fi signal type for passive scanning (B or G/N). |
| `WIFI_SCAN_MODE` | Wi-Fi capture mode. |
| `WIFI_TIMEOUT_IN_MS` | The maximal duration of a single preamble search, in milliseconds. |
| `WIFI_NBR_RETRIALS` | The number of internal scan sequences per channel scanned. |
| `WIFI_MAX_RESULTS` | The maximal number of results to gather. |

## 3. Usage

The application requires no user intervention other than the static configuration above. It starts scanning as soon as it runs.

Information messages are displayed on the serial console, showing at regular intervals the number of access point scanned and then for each access point, its MAC address and RSSI value:

```
INFO : ###### ===== Wi-FI SCAN ==== ######

Nb access points scanned : 5 
Wi-Fi Acquisition mode : BEACON_AND_PKT
Wi-Fi result format : BASIC_MAC_TYPE_CHANNEL
MAC addr : 0XA0 0X39 0XEE 0X8D 0XDF 0XF6 - RSSI : -95 dBm - Channel : 1 - Signal Type : 1
MAC addr : 0XA0 0X1B 0X29 0XFE 0X61 0X80 - RSSI : -84 dBm - Channel : 1 - Signal Type : 1
MAC addr : 0X40 0X65 0XA3 0X1A 0X57 0XBE - RSSI : -84 dBm - Channel : 6 - Signal Type : 1
MAC addr : 0X8C 0X97 0XEA 0X96 0X89 0X40 - RSSI : -92 dBm - Channel : 6 - Signal Type : 1
MAC addr : 0X44 0XA6 0X1E 0X61 0X44 0X06 - RSSI : -79 dBm - Channel : 11 - Signal Type : 1

Scan Timing : 1450 ms
Scan Consumption : 17232 uas
```
