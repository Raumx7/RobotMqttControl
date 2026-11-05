#pragma once

#include <cstdint>

// Avialable robot commands
enum class CommandType : uint8_t {
    Arcade,
    Move,
    Square,
    SetBrake,
    Stop,
    RectangleScan
};

// Command errors depending on message recieved or robot state
enum class CommandError : uint8_t {
    Unknown,
    AlreadyExecuted,
    ExecutingAutonomous,
    InvalidArgument
};

// Autonomous machine state
enum class AutoState : uint8_t {
    Idle,
    Forward,
    Turn,
    RectangleScan
};

// Drivetrain brake mode
enum class BrakeMode : uint8_t {
    Coast,
    Brake
};

// Used to track robot status and validate commands
struct RobotParams {
    int l_power = 0;
    int r_power = 0;
    AutoState auto_state = AutoState::Idle;
    BrakeMode brake_mode = BrakeMode::Coast;
};

// Used to handle sensor data
struct SensorData {
    float temperature = 0.0f;
    float humidity    = 0.0f;
};
