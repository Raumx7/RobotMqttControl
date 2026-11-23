#pragma once

#include <cstdint>

// Available robot commands
enum class CommandType : uint8_t {
    Arcade,
    Move,
    Square,
    SetBrake,
    Stop,
    RectangleScan
};

// Command errors depending on message received or robot state
enum class CommandError : uint8_t {
    AutonomousRunning,
    CollisionDetected,
    InvalidArgument,
    Unknown
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

// Used to track robot parameters/status and validate commands
struct RobotParams {
    int l_power = 0;
    int r_power = 0;
    float ft_distance = 0.0f;
    float bk_distance = 0.0f;
    AutoState auto_state = AutoState::Idle;
    BrakeMode brake_mode = BrakeMode::Coast;
};

// Used to handle sensor data
struct SensorData {
    uint16_t tvoc     = 0;
    uint16_t eco2     = 0;
    float temperature = 0.0f;
    float humidity    = 0.0f;
};