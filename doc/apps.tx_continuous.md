# Continuous transmission test application 

## 1. Description

Once started, this application will transmit continuously. Depending on the configuration option `TX_MODULATED`, it will either transmit a carrier wave or a rapid succession of modulated packets

## 2. Configuration

The `main_test_tx_continuous.h` header file define several constants which value can be set to define the LoRaWAN configuration of the application.

| Constant              | Comments |
| --------------------- | -------- |
| `TX_MODULATED` | If true, the application will transmit modulated packets, else it will emit a carrier wave. |
| `LORAWAN_REGION_USED` | Select the regulatory region. |
| `FREQUENCY` | Requested transmit frequency, in Hz. Must be valid in the regulatory region. |
| `TX_POWER_USED` | Output power, in dBm. |
