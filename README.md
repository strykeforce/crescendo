# 2024 FIRST CRESCENDO

[![CI](https://github.com/strykeforce/crescendo/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/crescendo/actions/workflows/main.yml)

## Controls

### Driver Controller

### Operator Controller

## CAN Bus

| Subsystem  | Type     | Talon                 | ID | Comp PDP | Proto PDP | Motor      | Breaker |
| ---------- | -------- | --------------------- | -- | -------- | --------- | ---------- | ------- |
| Drive      | SRX      | azimuth               | 0  |          |           | 9015       |         |
| Drive      | SRX      | azimuth               | 1  |          |           | 9015       |         |
| Drive      | SRX      | azimuth               | 2  |          |           | 9015       |         |
| Drive      | SRX      | azimuth               | 3  |          |           | 9015       |         |
| Drive      | FX       | drive                 | 10 |          |           | kraken     |         |
| Drive      | FX       | drive                 | 11 |          |           | kraken     |         |
| Drive      | FX       | drive                 | 12 |          |           | kraken     |         |
| Drive      | FX       | drive                 | 13 |          |           | kraken     |         |

## Roborio
| Subsystem | Interface | Device | 
| --------- | --------- | ------ |
| Drive     | USB       | NAVX   |

## DIO
| Subsystem | name       | ID |
| --------- | ---------- | -- |
| Auto      | autoSwitch | 0  |
| Auto      | autoSwitch | 1  |
| Auto      | autoSwitch | 2  |
| Auto      | autoSwitch | 3  |
| Auto      | autoSwitch | 4  |
| Auto      | autoSwitch | 5  |
|           |            | 6  |
|           |            | 7  |
|           |            | 8  |
|           |            | 9  |

## MXP
| Subsystem | name   | ID |
| --------- | ------ | -- |
|           |        |    |

