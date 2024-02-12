# 2024 FIRST CRESCENDO

[![CI](https://github.com/strykeforce/crescendo/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/crescendo/actions/workflows/main.yml)

## Controls

### Driver Controller
![flysky](docs/driver-controls.png)

### Operator Controller
![operator](docs/operator-controls.png)

## CAN Bus

| Subsystem | Type     | Talon      | ID  | Comp PDP | Proto PDP | Motor  | Breaker |
| --------- | -------- | ---------- | --- | -------- | --------- | ------ | ------- |
| Drive     | SRX      | azimuth    | 0   |          | 16        | 9015   |         |
| Drive     | SRX      | azimuth    | 1   |          | 2         | 9015   |         |
| Drive     | SRX      | azimuth    | 2   |          | 17        | 9015   |         |
| Drive     | SRX      | azimuth    | 3   |          | 4         | 9015   |         |
| Drive     | FX       | drive      | 10  |          | 18        | kraken |         |
| Drive     | FX       | drive      | 11  |          | 1         | kraken |         |
| Drive     | FX       | drive      | 12  |          | 19        | kraken |         |
| Drive     | FX       | drive      | 13  |          | 0         | kraken |         |
| Intake    | FX       | intake     | 20  |          | 3         | falcon |         |
| Magazine  | FX       | magazine   | 25  |          | 13        | falcon |         |
| Elbow     | FX       | elbow      | 30  |          | 5         | falcon |         |
| Elbow     | CANifier | canifier   | 31  |          |           | -      |         |
| Wrist     | SRX      | wrist      | 35  |          | 11        | 9015   |         |
| Shooter   | FX       | leftShoot  | 40  |          | 12        | falcon |         |
| Shooter   | FX       | rightShoot | 41  |          | 10        | falcon |         |
| Climb     | FX       | climb      | 50  |          |           | falcon |         |
* intake beam break: to wheel 1 azimuth fwd lim
* magazine y-axis beam break: to wrist fwd lim
* magazine z-axis beam break: to wrist rev lim
*abs encoder for elbow to canifier

## CANifier
| Subsystem | Desc     | Name      | #   |
| --------- | -------- | --------- | --- |
|           |          | SDA       | 1   |
|           |          | SCL       | 2   |
| Elbow     | 5V (2)   | 5V        | 3   |
| Elbow     | A (7)    | QUAD A    | 4   |
| Elbow     | GND (10) | GND       | 5   |
| Elbow     | B (5)    | QUAD B    | 6   |
|           |          | INDEX     | 7   |
|           |          | LIMF      | 8   |
|           |          | GND       | 9   |
|           |          | LIMR      | 10  |
|           |          | GND       | 11  |
|           |          | CS/PWM3   | 12  |
| Elbow     | PWM (9)  | CLK/PWM0  | 13  |
|           |          | MOSI/PWM1 | 14  |
|           |          | MISO/PWM2 | 15  |
|           |          | 5V        | 16  |
|           |          | 5V        | 17  |
|           |          | 3.3V      | 18  |
|           |          | 3.3V      | 19  |
|           |          | GND       | 20  |
|           |          | GND       | 21  |


## Roborio
| Subsystem | Interface | Device | 
| --------- | --------- | ------ |
| Drive     | USB       | NAVX   |

## DIO
| Subsystem | name           | ID  |
| --------- | -------------- | --- |
| Auto      | autoSwitch     | 0   |
| Auto      | autoSwitch     | 1   |
| Auto      | autoSwitch     | 2   |
| Auto      | autoSwitch     | 3   |
| Auto      | autoSwitch     | 4   |
| Auto      | autoSwitch     | 5   |
| Robot     | eventInterlock | 6   |
|           |                | 7   |
|           |                | 8   |
|           |                | 9   |

## MXP
| Subsystem | name   | ID |
| --------- | ------ | -- |
|           |        |    |

