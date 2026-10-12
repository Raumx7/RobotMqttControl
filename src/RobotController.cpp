#include "RobotController.h"
#include <cstdlib>
#include <cstdio>

namespace {

    // Clampa un valor al rango [-100, 100]
    int clamp_power(int v) {
        if (v > 100) return 100;
        if (v < -100) return -100;
        return v;
    }

    bool parse_int(std::string_view args, int& a) {
        char buf[16];

        const std::size_t n = args.size() < sizeof(buf) - 1
                                ? args.size() : sizeof(buf) - 1;
        for (std::size_t i = 0; i < n; ++i) buf[i] = args[i];
        buf[n] = '\0';

        char extra;
        // %n cuenta caracteres consumidos para verificar que no sobra basura
        int consumed = 0;
        if (std::sscanf(buf, " %d %n", &a, &consumed) != 1) return false;
        // Aseguramos que no haya más tokens después
        if (buf[consumed] != '\0') return false;
        return true;
    }

    // Parser minimalista: espera " <int> <int>" y devuelve true si
    // ambos enteros se parsearon correctamente.
    bool parse_two_ints(std::string_view args, int& a, int& b) {
        // Necesitamos una cadena null-terminated para sscanf.
        // Como los comandos son cortos, copiamos a un buffer pequeño.
        char buf[32];
        const std::size_t n = args.size() < sizeof(buf) - 1
                                ? args.size() : sizeof(buf) - 1;
        for (std::size_t i = 0; i < n; ++i) buf[i] = args[i];
        buf[n] = '\0';

        char extra;
        // %n cuenta caracteres consumidos para verificar que no sobra basura
        int consumed = 0;
        if (std::sscanf(buf, " %d %d %n", &a, &b, &consumed) != 2) return false;
        // Aseguramos que no haya más tokens después
        if (buf[consumed] != '\0') return false;
        return true;
    }

    bool parse_three_ints(std::string_view args, int& a, int& b, int& c) {
        char buf[48];
        const std::size_t n = args.size() < sizeof(buf) - 1
                                ? args.size() : sizeof(buf) - 1;
        for (std::size_t i = 0; i < n; ++i) buf[i] = args[i];
        buf[n] = '\0';

        int consumed = 0;
        if (std::sscanf(buf, " %d %d %d %n", &a, &b, &c, &consumed) != 3) return false;
        if (buf[consumed] != '\0') return false;
        return true;
    }

    // Comprueba si dos MoveParams son iguales
    bool same_params(const MoveParams& a, const MoveParams& b) {
        return a.l_power == b.l_power && a.r_power == b.r_power;
    }

} // namespace

//==============================================================================

RobotController::RobotController(L298N& motors_left, L298N& motors_right)
    : motors_left_(motors_left), motors_right_(motors_right)
{
}

//==============================================================================
// Acciones de movimiento
//==============================================================================

void RobotController::move(int l_power, int r_power) {
    const int lp = clamp_power(l_power);
    const int rp = clamp_power(r_power);

    motors_left_.set_speed(lp);
    motors_right_.set_speed(rp);

    update_move_values(lp, rp);
}

void RobotController::arcade_control(int x_power, int y_power) {
    const int x = clamp_power(x_power);
    const int y = clamp_power(y_power);

    const int l = clamp_power(y + x);
    const int r = clamp_power(y - x);

    move(l, r);
}

void RobotController::stop() {
    if (current_move_params_.brake_mode == BrakeMode::Brake) {
        motors_left_.brake();
        motors_right_.brake();
    } else {
        motors_left_.stop();
        motors_right_.stop();
    }
    update_move_values(0, 0);
}

//==============================================================================

void RobotController::update_move_values(int l_power, int r_power) {
    current_move_params_.l_power = l_power;
    current_move_params_.r_power = r_power;
}

//==============================================================================
// Parser de comandos
//==============================================================================
//
// Gramática:
//   A <x> <y>                -> arcade (joystick), x,y en [-100,100]
//   M <l> <r>                -> move directo, l,r en [-100,100]
//   S <power> <fw_ms> <tn_ms> -> rutina cuadrada, power en [1,100], ms > 0
//   X                        -> stop
//
//==============================================================================

ControllerResponse RobotController::execute_command(std::string_view command) {

    if (command.empty()) {
        return tl::make_unexpected(CommandError::Unknown);
    }

    const char type = command[0];
    const std::string_view args = command.substr(1);

    // Un comando autónomo en curso bloquea cualquier otra orden
    if (is_autonomous() && type != 'X') {
        return tl::make_unexpected(CommandError::ExcecutingAutonomous);
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
    if (!parse_two_ints(args, x, y)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (x < -100 || x > 100 || y < -100 || y > 100) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    // Calculamos los valores objetivo para comparar con el estado actual
    const int l = clamp_power(y + x);
    const int r = clamp_power(y - x);
    MoveParams target{l, r};

    if (same_params(target, current_move_params_)) {
        return tl::make_unexpected(CommandError::AlreadyExcecuted);
    }

    arcade_control(x, y);
    return current_move_params_;
}

//==============================================================================

ControllerResponse RobotController::handle_move(std::string_view args) {
    int l = 0, r = 0;
    if (!parse_two_ints(args, l, r)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (l < -100 || l > 100 || r < -100 || r > 100) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    MoveParams target{l, r};
    if (same_params(target, current_move_params_)) {
        return tl::make_unexpected(CommandError::AlreadyExcecuted);
    }

    move(l, r);
    return current_move_params_;
}

//==============================================================================

ControllerResponse RobotController::handle_square(std::string_view args) {
    int power = 0, fw_ms = 0, tn_ms = 0;
    if (!parse_three_ints(args, power, fw_ms, tn_ms)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }
    if (power < 1 || power > 100 || fw_ms <= 0 || tn_ms <= 0) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    start_square(power, static_cast<unsigned>(fw_ms),
                        static_cast<unsigned>(tn_ms));

    // Devolvemos el primer estado de la rutina
    return current_move_params_;
}

//==============================================================================

ControllerResponse RobotController::handle_brake_mode(std::string_view args) {
    int brake_param = 0;

    if (!parse_int(args, brake_param)) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    if (brake_param != 0 && brake_param != 1) {
        return tl::make_unexpected(CommandError::InvalidArgument);
    }

    const auto brake_mode = static_cast<BrakeMode>(brake_param);

    if (brake_mode == current_move_params_.brake_mode) {
        return tl::make_unexpected(CommandError::AlreadyExcecuted);
    }

    set_brake_mode(brake_mode);

    return current_move_params_;
}

//==============================================================================

ControllerResponse RobotController::handle_stop() {
    if (is_stopped()) {
        return tl::make_unexpected(CommandError::AlreadyExcecuted);
    }

    stop();

    // Si había una rutina autónoma en curso, la cancelamos
    if (is_autonomous()) {
        auto_state_ = AutoState::Idle;
    }

    return current_move_params_;
}

//==============================================================================
// Rutina autónoma: cuadrado
//==============================================================================

void RobotController::start_square(int power,
                                   unsigned int forward_ms,
                                   unsigned int turn_ms)
{
    auto_power_         = clamp_power(power);
    auto_forward_ms_    = forward_ms;
    auto_turn_ms_       = turn_ms;
    auto_side_count_    = 0;
    auto_state_         = AutoState::Forward;
    auto_state_start_ms_ = millis();

    // Primer tramo: avanzar recto
    move(auto_power_, auto_power_);
}

void RobotController::update_square() {

    const unsigned long elapsed = millis() - auto_state_start_ms_;

    switch (auto_state_) {

        case AutoState::Forward:
            if (elapsed >= auto_forward_ms_) {
                // Cambiar a giro a la derecha: izquierda adelante,
                // derecha atrás (giro sobre el eje)
                move(auto_power_, -auto_power_);
                auto_state_ = AutoState::Turn;
                auto_state_start_ms_ = millis();
            }
            break;

        case AutoState::Turn:
            if (elapsed >= auto_turn_ms_) {
                if (++auto_side_count_ >= 4) {
                    finish_autonomous();
                } else {
                    move(auto_power_, auto_power_);
                    auto_state_ = AutoState::Forward;
                    auto_state_start_ms_ = millis();
                }
            }
            break;

        case AutoState::Idle:
        default:
            break;
    }
}

void RobotController::finish_autonomous() {
    stop();
    auto_state_ = AutoState::Idle;
    auto_side_count_ = 0;
}

//==============================================================================

void RobotController::update() {
    if (auto_state_ == AutoState::Idle) return;
    update_square();
}