# Simple LoRaWAN clock synchronization example 

## 1. Description

The application will automatically join the LoRaWAN network after activating the Application Layer Clock synchronization mode. It will then display in the serial terminal console the internal LR1110 clock time at regular intervals.

The clock value is displayed as a GPS Epoch Time, i.e. a number of seconds since january 6, 1980. The clock value will be zero before synchronization with the ALC Sync service is achieved.

For more information on the Application Layer Clock Synchronization service, please refer to the [LoRa Developer portal](https://lora-developers.semtech.com/resources/tools/lora-edge-asset-management/?url=rst/Modem/alc_sync.html).

## 2. Configuration

### 2.1. LoRaWAN configuration

The `main_clock_sync.h` header file define several constants which value can be set to define the LoRaWAN configuration of the application.

| Constant              | Comments |
| --------------------- | -------- |
| `APP_GETTIME_PERIOD_MS` | Defines the delay in milliseconds between two time displays. |
| `LORAWAN_REGION_USED` | Select the regulatory region. |
| `LORAWAN_CLASS_USED`  | LoRaWAN device class. |
| `LORAWAN_DUTYCYCLE_ON` | Enable/disable the Duty Cycle |
| `LORAWAN_DEFAULT_DATARATE` | Selection of the Data Rate |

Supported values for `LORAWAN_REGION_USED`:

* `LR1110_LORAWAN_REGION_EU868 (default)`
* `LR1110_LORAWAN_REGION_US915`
* `LR1110_LORAWAN_REGION_AU915`
* `LR1110_LORAWAN_REGION_CN470`
* `LR1110_LORAWAN_REGION_AS923_GRP1`
* `LR1110_LORAWAN_REGION_AS923_GRP2`
* `LR1110_LORAWAN_REGION_AS923_GRP3`
* `LR1110_LORAWAN_REGION_IN865`
* `LR1110_LORAWAN_REGION_KR920`
* `LR1110_LORAWAN_REGION_RU864`

Supported values for `LORAWAN_CLASS_USED`:

* `LR1110_LORAWAN_CLASS_A (default)`
* `LR1110_LORAWAN_CLASS_C`

Supported values for `LORAWAN_DEFAULT_DATARATE`:

* `LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED (default)`
* `LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE`
* `LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER`
* `LR1110_MODEM_ADR_PROFILE_CUSTOM`

When `LORAWAN_DEFAULT_DATARATE` is set to `LR1110_MODEM_ADR_PROFILE_CUSTOM` then a list of Data Rates must be provided to `lr1110_modem_set_adr_profile()`. See the example of `adr_custom_list` in `main_clock_sync.h`.

### 2.2. Join configuration

The LR1110 is pre-provisioned with a ChipEUI/DevEUI and a JoinEUI. The application will use these identifiers if the `USE_PRODUCTION_KEYS` is equal to one. 

Alternatively, you can provide your own EUIs in `Inc/apps/lorawan_commissioning/lorawan_commissioning.h` by setting `USE_PRODUCTION_KEYS` to any other value and by changing the values of `LORAWAN_DEVICE_EUI` and `LORAWAN_JOIN_EUI` and `LORAWAN_APP_KEY`.
