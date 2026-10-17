#include "RobotController.h"
#include "cmd.h"

namespace {

    struct RobotPowers {
        int l_power;
        int r_power;
    };

    int clamp_power(int v) {
        if (v > 100) return 100;
        if (v < -100) return -100;
        return v;
    }

    bool same_powers(const RobotPowers& a, const RobotPowers& b) {
        return a.l_power == b.l_power && a.r_power == b.r_power;
    }

} // namespace

//==============================================================================

RobotController::RobotController(L298N& motors_left, L298N& motors_right)
    : motors_left_(motors_left), motors_right_(motors_right)
{
}

//==============================================================================
// Movement actions
//==============================================================================

void RobotController::move(int l_power, int r_power) {
    const int lp = clamp_power(l_power);
    const int rp = clamp_power(r_power);

    motors_left_.set_speed(lp);
    motors_right_.set_speed(rp);

    update_motor_powers(lp, rp);
}

void RobotController::arcade_control(int x_power, int y_power) {
    const int x = clamp_power(x_power);
    const int y = clamp_power(y_power);

    const int l = clamp_power(y + x);
    const int r = clamp_power(y - x);

    move(l, r);
}

void RobotController::stop() {
    if (params_.brake_mode == BrakeMode::Brake) {
        motors_left_.brake();
        motors_right_.brake();
    } else {
        motors_left_.stop();
        motors_right_.stop();
    }
    update_motor_powers(0, 0);
}

//==============================================================================

void RobotController::update_motor_powers(int l_power, int r_power) {
    params_.l_power = l_power;
    params_.r_power = r_power;
}

//==============================================================================
// Command Parser
//==============================================================================
//
// Grammar:
//   A <x> <y>                  -> arcade (joystick), x,y on [-100,100]
//   M <l> <r>                  -> direct move, l,r on [-100,100]
//   S <power> <fw_ms> <tn_ms>  -> autonomous square routine, power on [1,100], ms > 0
//   B <bkmd>                   -> brake mode 0 or 1
//   X                          -> stop
//
//==============================================================================

ControllerResponse RobotController::execute_command(std::string_view command) {

    if (command.empty()) {
        return tl::make_unexpected(CommandError::Unknown);
    }

    const char type = command[0];
    const std::string_view args = command.substr(1);

    // Autonomous state blocks all actions except stopping
    if (is_autonomous() && type != 'X') {
        return tl::make_unexpected(CommandError::ExecutingAutonomous);
    }

    switch (type) {
        case 'A': return handle_arcade(args);
        case 'M': return handle_move(args);
        case 'S': return handle_square(args);
        case 'B': return handle_brake_mode(args);
        case 'X': return handle_stop();
        default:  return tl::make_unexpected(CommandError::Unknown);
    }
}

//==============================================================================

ControllerResponse RobotController::handle_arcade(std::string_view args) {
    int x = 0, y = 0;
    if (!cmd::to_int(args, x, y)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (x < -100 || x > 100 || y < -100 || y > 100) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    const int l = clamp_power(y + x);
    const int r = clamp_power(y - x);

    if (same_powers(RobotPowers(l, r), RobotPowers(params_.l_power, params_.r_power))) {
        return tl::make_unexpected(CommandError::AlreadyExecuted);
    }

    arcade_control(x, y);
    return CommandType::Arcade;
}

//==============================================================================

ControllerResponse RobotController::handle_move(std::string_view args) {
    int l = 0, r = 0;
    if (!cmd::to_int(args, l, r)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (l < -100 || l > 100 || r < -100 || r > 100) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    if (same_powers(RobotPowers(l, r), RobotPowers(params_.l_power, params_.r_power))) {
        return tl::make_unexpected(CommandError::AlreadyExecuted);
    }

    move(l, r);
    return CommandType::Move;
}

//==============================================================================

ControllerResponse RobotController::handle_square(std::string_view args) {
    int power = 0, fw_ms = 0, tn_ms = 0;
    if (!cmd::to_int(args, power, fw_ms, tn_ms)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (power < 1 || power > 100 || fw_ms <= 0 || tn_ms <= 0) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    start_square(power, static_cast<unsigned>(fw_ms),
                        static_cast<unsigned>(tn_ms));

    return CommandType::Square;
}

//==============================================================================

ControllerResponse RobotController::handle_brake_mode(std::string_view args) {
    int brake_param = 0;

    if (!cmd::to_int(args, brake_param)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    if (brake_param != 0 && brake_param != 1) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    const auto brake_mode = static_cast<BrakeMode>(brake_param);

    if (brake_mode == params_.brake_mode) {
        return tl::make_unexpected(CommandError::AlreadyExecuted);
    }

    set_brake_mode(brake_mode);

    return CommandType::SetBrake;
}

//==============================================================================

ControllerResponse RobotController::handle_stop() {
    if (is_stopped()) {
        return tl::make_unexpected(CommandError::AlreadyExecuted);
    }

    stop();

    if (is_autonomous()) {
        params_.auto_state = AutoState::Idle;
    }

    return CommandType::Stop;
}

//==============================================================================
// Autonomous routine: square
//==============================================================================

void RobotController::start_square(int power,
                                   unsigned int forward_ms,
                                   unsigned int turn_ms)
{
    auto_power_             = clamp_power(power);
    auto_forward_ms_        = forward_ms;
    auto_turn_ms_           = turn_ms;
    auto_side_count_        = 0;
    params_.auto_state      = AutoState::Forward;
    auto_state_start_ms_    = millis();

    move(auto_power_, auto_power_); // first direction -> forward
}

//==============================================================================

void RobotController::update_square() {

    const unsigned long elapsed = millis() - auto_state_start_ms_;

    switch (params_.auto_state) {

        case AutoState::Forward:
            if (elapsed >= auto_forward_ms_) {
                move(auto_power_, -auto_power_);
                params_.auto_state = AutoState::Turn;
                auto_state_start_ms_ = millis();
            }
            break;

        case AutoState::Turn:
            if (elapsed >= auto_turn_ms_) {
                if (++auto_side_count_ >= 4) {
                    finish_autonomous();
                } else {
                    move(auto_power_, auto_power_);
                    params_.auto_state = AutoState::Forward;
                    auto_state_start_ms_ = millis();
                }
            }
            break;

        case AutoState::Idle:
        default:
            break;
    }
}

//==============================================================================

void RobotController::finish_autonomous() {
    stop();
    params_.auto_state = AutoState::Idle;
    auto_side_count_ = 0;
}

//==============================================================================

void RobotController::loop() {
    if (params_.auto_state == AutoState::Idle) return;
    update_square();
}

//==============================================================================

bool RobotController::has_same_params(const RobotParams& params) const {
    return (params_.l_power == params.l_power &&
            params_.r_power == params.r_power &&
            params_.auto_state == params.auto_state &&
            params_.brake_mode == params.brake_mode
            );
}
