# Robot Mqtt Control
Robot C++/Arduino-Framework codes to handle mqtt comunication and events control

## Dependencies

| Library  | Version  | Usage  |
|---|---|---|
|[knolleary/PubSubClient](https://github.com/knolleary/pubsubclient.git)   | v2.1 | MQTT Client  |
|[jobitjoseph/SimpleOLED](https://github.com/jobitjoseph/SimpleOLED)   | v1.0.0 | SSD1306 driver  |
|[pololu/vl53l0x-arduino](https://github.com/pololu/vl53l0x-arduino)   | v1.3.1  | VL53L0X driver |
|[RobTillaart/SGP30](https://github.com/RobTillaart/SGP30)  | v0.3.3  | SGP30 driver  |
|[TartanLlama/expected](https://github.com/TartanLlama/expected.git)   | v1.3.1  | Stack exceptions  |

**Note:** see platformio.ini for specific project configuration and compile settings

## Command parser

| Command   | Description                          | Parameters (range)              |
|-----------|--------------------------------------|----------------------------------|
| `A <x> <y>`    | Arcade (joystick)                | x, y ∈ [-100, 100]            |
| `M <l> <r>`    | Direct move                      | l, r ∈ [-100, 100]            |
| `S <power> <fw_ms> <tn_ms>` | Autonomous square routine | power ∈ [1, 100], ms > 0      |
| `B <bkmd>`     | Brake mode                       | 0 or 1                        |
| `X`              | Stop                             | —                               |

## MQTT Topics

### Subscriber Topics
* robot/command

### Publisher Topics
* robot/response
* robot/status
* robot/telemetry
