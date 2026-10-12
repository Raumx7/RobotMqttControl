#pragma once
#include <cstdint>

// Drivetrain brake mode
enum class BrakeMode : uint8_t {
    Coast,
    Brake
};

// Left and right motor groups speed values
struct MoveParams {
    int l_power = 0;
    int r_power = 0;
    BrakeMode brake_mode = BrakeMode::Coast;
};

// Command errors depending on message recieved or last robot state
enum class CommandError : uint8_t {
    Unknown                 = 0,
    AlreadyExcecuted        = 1,
    ExcecutingAutonomous    = 2,
    InvalidArgument         = 3
};

// String brake mode message
inline constexpr const char* BkStr[] = {
    "coast",
    "brake"
};

// String error messages
inline constexpr const char* ErrStr[] = {
    "command_unknown",      // idx[0] -> CommandError::Unknown
    "already_executed",     // idx[1] -> CommandError::AlreadyExcecuted
    "autonomous_running",   // idx[2] -> CommandError::ExcecutingAutonomous
    "invalid_argument"      // idx[3] -> CommandError::InvalidArgument
};
