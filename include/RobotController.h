#pragma once
#include <Arduino.h>
#include <tl/expected.hpp>
#include <string_view>

#include "L298N.h"
#include "types.h"

using ControllerResponse = tl::expected<CommandType, CommandError>;

class RobotController {
public:
    // Constructor
    RobotController(L298N& motors_left, L298N& motors_right);

    // Set brake mode
    void set_brake_mode(BrakeMode brake_mode) { params_.brake_mode = brake_mode; }

    // Direct motors control
    void move(int l_power, int r_power);

    // Joystick movement: y = forward/backward, x = yaw
    void arcade_control(int x_power, int y_power);

    // Stops the drivetrain with the current brake mode
    void stop();

    // Get brake mode
    [[nodiscard]] BrakeMode get_brake_mode() const { return params_.brake_mode; }

    // True when both motor groups are stopped
    [[nodiscard]] bool is_stopped() const { return motors_left_.is_stopped() && motors_right_.is_stopped(); }

    // True when autonomous routine is running
    [[nodiscard]] bool is_autonomous() const { return params_.auto_state != AutoState::Idle; }

    // True when current data is the same as data parameter
    [[nodiscard]] bool has_same_params(const RobotParams& params) const;

    // Returns the current move params
    [[nodiscard]] const RobotParams& get_params() const { return params_; }

    // Executes a command returning a command type or an error
    [[nodiscard]] ControllerResponse execute_command(std::string_view command);

    // Must be called in the main loop so it does not block other actions
    void loop();

private:
    // Intern Helper
    void update_motor_powers(int l_power, int r_power);

    // Parse commands
    ControllerResponse handle_arcade(std::string_view args);
    ControllerResponse handle_move(std::string_view args);
    ControllerResponse handle_square(std::string_view args);
    ControllerResponse handle_brake_mode(std::string_view args);
    ControllerResponse handle_stop();

    // Autonomous routine functions
    void start_square(int power, unsigned int forward_ms, unsigned int turn_ms);
    void update_square();
    void finish_autonomous();

    // Motor groups
    L298N& motors_left_;
    L298N& motors_right_;

    RobotParams params_{};

    // Autonomous
    unsigned long auto_state_start_ms_ = 0;
    unsigned int auto_forward_ms_ = 0;
    unsigned int auto_turn_ms_ = 0;
    int auto_power_ = 0;
    uint8_t auto_side_count_ = 0;
};
