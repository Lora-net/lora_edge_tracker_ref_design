# Simple LR1110 GNSS example application 

## 1. Description

This application executes a periodic GNSS scan according to the defined GNSS settings.

To get to position more accurate thanks to the NAV message returned by the scan, the application can work in assisted mode. It then requires approximate values for the device position (`GNSS_ASSIST_LATITUDE` and `GNSS_ASSIST_LONGITUDE` configuration options) and current time.
Note that the NAV message has to be solved by loracloud in order to have a position. The application doesn't return directly a position.

The LR1110 can work with the GPS constellation, the BeiDou constellation, or both constellations at the same time, as defined by the `GNSS_CONSTELLATION` configuration option.

## 2. Configuration

The following constants are defined in the `main_simple_gnss_example.h` file and can be changed to suit the planned test:

| Constant              | Comments |
| --------------------- | -------- |
| `GNSS_SCAN_PERIOD_MS` | Define the delay, in milliseconds, between two scans. |
| `CONSOLE_INPUT_ECHO` | Define and set to a non-zero value to echo input characters on the console output. |
| `GNSS_SCAN_TYPE` | GNSS Scan type. Either `ASSISTED_MODE` or `AUTONOMOUS_MODE` |
| `GNSS_ASSIST_LATITUDE` | Approximated device position latitude when using assist mode, in degrees. |
| `GNSS_ASSIST_LONGITUDE` | Approximated device position longitude when using assist mode, in degrees. |
| `GNSS_CONSTELLATION` | Constellation(s) to use. |

## 3. Usage

The application communicates with the user with the serial console.

In assisted mode, the user is requested to input the current date and time at startup. The application expects the time to be provided as an ASCII string representing the integer number of seconds elapsed since the UNIX epoch, i.e. the return value from the `date +%s` command:

```
$ date +%s
1623220248
```

If you don't have a Unix-compatible system to run this command you can use one of a number of websites that displays the current Unix epoch time. 

When running, the application displays on the serial console output a status of the detected satellites and associated navigation data:

```
SCAN...
Nb Detected satellites : 7
Satellites infos :
ID = 28 -- CN = 47
ID = 99 -- CN = 44
ID = 93 -- CN = 44
ID = 23 -- CN = 41
ID = 1 -- CN = 41
ID = 5 -- CN = 41
ID = 35 -- CN = 36
Scan timing radio_ms : 512
Scan timing computation_ms : 1392
NAV = 010154B2092808A238B0102188665C62E1B5C4390FC47A91265C5088CC1AE068D24B0922E3
D88BE320ECBBB1C5C2F1EE03
Is NAV message valid : 1
Average CN : 42
```
