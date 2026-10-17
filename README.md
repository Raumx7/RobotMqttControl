# Robot Mqtt Control
Robot C++/Arduino-Framework codes to handle mqtt comunication and events control

## Library dependencies

[knolleary/PubSubClient](https://github.com/knolleary/pubsubclient.git) v2.1  
[TartanLlama/expected](https://github.com/TartanLlama/expected.git)     v1.3.1

## Compile flags

```text
-fno-exceptions
-std=gnu++17
```
## Command parser

| Command   | Description                          | Parameters (range)              |
|-----------|--------------------------------------|----------------------------------|
| `A <x> <y>`    | Arcade (joystick)                | x, y ∈ [-100, 100]            |
| `M <l> <r>`    | Direct move                      | l, r ∈ [-100, 100]            |
| `S <power> <fw_ms> <tn_ms>` | Autonomous square routine | power ∈ [1, 100], ms > 0      |
| `B <bkmd>`     | Brake mode                       | 0 or 1                        |
| `X`              | Stop                             | —                               |