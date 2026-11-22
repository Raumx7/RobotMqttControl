#pragma once

#include <string_view>

#include <Arduino.h>
#include <tl/expected.hpp>

#include "DistanceScanner.h"
#include "L298N.h"
#include "types.h"

using ControllerResponse = tl::expected<CommandType, CommandError>;

class RobotController {
public:
    // Constructor
    RobotController(L298N& motors_left, L298N& motors_right, DistanceScanner& distance_scanner);

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

    // True when autonomous routine is running
    [[nodiscard]] bool is_autonomous() const { return params_.auto_state != AutoState::Idle; }

    // True when both motor groups are stopped
    [[nodiscard]] bool is_stopped() const { return motors_left_.is_stopped() && motors_right_.is_stopped(); }

    // True when one or both motor groups are moving backward
    [[nodiscard]] bool is_moving_backward() const { return params_.l_power < 0 || params_.r_power < 0; }

    // True when one or both motor groups are moving forward
    [[nodiscard]] bool is_moving_forward() const { return params_.l_power > 0 || params_.r_power > 0; }

    // True when the robot detects collision based on current direction (false when stopped)
    [[nodiscard]] bool has_collision() const { return collision_ft || collision_bk; };

    // True when current data is the same as data parameter
    [[nodiscard]] bool has_same_params(const RobotParams& params) const;

    // Returns the current move params
    [[nodiscard]] const RobotParams& get_params() const { return params_; }

    // Executes a command returning a command type or an error
    [[nodiscard]] ControllerResponse execute_command(std::string_view command);

    // Must be called in the main loop to update autonomous routines and distance values
    void loop();

private:
    // Intern Helpers
    void update_motor_powers(int l_power, int r_power);
    void update_distance_values();

    // Evaluate the next robot movement based on motor powers
    [[nodiscard]] static bool will_move_backward(int l_power, int r_power) { return l_power < 0 || r_power < 0; }
    [[nodiscard]] static bool will_move_forward(int l_power, int r_power) { return l_power > 0 || r_power > 0; }

    // True if the proposed command encounters an obstacle in the direction in which it would imply movement
    [[nodiscard]] bool would_collide(int l_power, int r_power) const;

    // Parse commands
    ControllerResponse handle_arcade(std::string_view args);
    ControllerResponse handle_move(std::string_view args);
    ControllerResponse handle_square(std::string_view args);
    ControllerResponse handle_rectangle_scan(std::string_view args);
    ControllerResponse handle_brake_mode(std::string_view args);
    ControllerResponse handle_stop();

    // Autonomous routine functions
    void start_square(int power, unsigned int forward_ms, unsigned int turn_ms);
    void start_rectangle_scan(int power, unsigned int fw_long_ms, unsigned int fw_short_ms, unsigned int wait_ms, unsigned int turn_ms);
    void update_square();
    void update_rectangle_scan();
    void finish_autonomous();

    // Motor groups
    L298N& motors_left_;
    L298N& motors_right_;

    // Distance sensors handler
    DistanceScanner& distance_scanner_;

    // Robot parameters
    RobotParams params_{};

    // Collision status
    bool collision_ft = false;
    bool collision_bk = false;

    // Autonomous
    unsigned long auto_state_start_ms_ = 0; // time ms when the robot starts autonomous
    unsigned int auto_forward_ms_ = 0;      // time ms to move the robot forward
    unsigned int auto_turn_ms_ = 0;         // time ms to turn the robot
    unsigned int auto_fw_long_ms_  = 0;     // time ms to go forward (rectangle lenght)
    unsigned int auto_fw_short_ms_ = 0;     // time ms to go forward (rectangle hight)
    unsigned int auto_wait_ms_ = 5000;      // time ms to wait and take samples
    int auto_power_ = 0;                    // motor powers for the autonomous routines
    uint8_t auto_side_count_ = 0;           // number of turns (square or rectangle corners)
    int scan_step_ = 0;                     // prepares next robot movement
};
