#pragma once
#include <Arduino.h>
#include <tl/expected.hpp>
#include <string_view>
#include "L298N.h"
#include "types.h"

using ControllerResponse = tl::expected<MoveParams, CommandError>;

class RobotController {
public:
    RobotController(L298N& motors_left, L298N& motors_right);

    void set_brake_mode(BrakeMode brake_mode) { current_move_params_.brake_mode = brake_mode; }
    BrakeMode get_brake_mode() const { return current_move_params_.brake_mode; }

    // Control directo de motores (potencias independientes)
    void move(int l_power, int r_power);

    // Movimiento tipo joystick: y = adelante/atrás, x = giro
    void arcade_control(int x_power, int y_power);

    // Detiene el robot con el modo de frenado actual
    void stop();

    bool is_stopped() const { return motors_left_.is_stopped() && motors_right_.is_stopped(); }

    bool is_autonomous() const { return auto_state_ != AutoState::Idle; }

    MoveParams get_move_params() const { return current_move_params_; }

    // Ejecuta un comando. Devuelve el estado actual o un error.
    [[nodiscard]] ControllerResponse execute_command(std::string_view command);

    // Debe llamarse periódicamente desde el loop principal.
    // Avanza la rutina autónoma si está en curso.
    void update();

private:
    // --- Máquina de estados de la rutina autónoma ---
    enum class AutoState : uint8_t {
        Idle,
        Forward,
        Turn
    };

    // --- Helpers internos ---
    void update_move_values(int l_power, int r_power);

    // --- Parsers / handlers por comando ---
    ControllerResponse handle_arcade(std::string_view args);
    ControllerResponse handle_move(std::string_view args);
    ControllerResponse handle_square(std::string_view args);
    ControllerResponse handle_brake_mode(std::string_view args);
    ControllerResponse handle_stop();

    // --- Rutina autónoma ---
    void start_square(int power, unsigned int forward_ms, unsigned int turn_ms);
    void update_square();
    void finish_autonomous();

    // --- Miembros ---
    L298N& motors_left_;
    L298N& motors_right_;

    MoveParams current_move_params_{};

    // Estado de la rutina autónoma
    AutoState auto_state_ = AutoState::Idle;
    unsigned long auto_state_start_ms_ = 0;
    unsigned int auto_forward_ms_ = 0;
    unsigned int auto_turn_ms_ = 0;
    int auto_power_ = 0;
    uint8_t auto_side_count_ = 0;
};