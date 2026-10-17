#pragma once
#include <cstdint>

// Avialable robot commands
enum class CommandType : uint8_t {
    Arcade,
    Move,
    Square,
    SetBrake,
    Stop
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
    Turn
};

// Drivetrain brake mode
enum class BrakeMode : uint8_t {
    Coast,
    Brake
};

// Converts any previous type to a C-String
namespace fmt {
    constexpr const char* to_str(AutoState state) {
        return (state != AutoState::Idle) ? "autonomous" : "manual";
    }

    constexpr const char* to_str(BrakeMode mode) {
        return (mode == BrakeMode::Coast) ? "coast" : "brake";
    }

    constexpr const char* to_str(CommandType type) {
        switch (type) {
            case CommandType::Arcade:   return "arcade";
            case CommandType::Move:     return "move";
            case CommandType::Square:   return "square";
            case CommandType::SetBrake: return "set_brake";
            case CommandType::Stop:     return "stop";
        }
        return "unknown";
    }

    constexpr const char* to_str(CommandError error) {
        switch (error) {
            case CommandError::Unknown:             return "command_unknown";
            case CommandError::AlreadyExecuted:     return "already_executed";
            case CommandError::ExecutingAutonomous: return "autonomous_running";
            case CommandError::InvalidArgument:     return "invalid_argument";
        }
        return "unknown";
    }
}

// Used to track robot status and validate commands
struct RobotParams {
    int l_power = 0;
    int r_power = 0;
    AutoState auto_state = AutoState::Idle;
    BrakeMode brake_mode = BrakeMode::Coast;
};
