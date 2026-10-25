#pragma once

#include <cstdint>

//==============================================================================
// Driver for the L298N H-bridge (DC motor), adapted for ESP32.
//
// The L298N separates direction (IN1/IN2, digital) from magnitude (ENA, PWM).
// On ESP32 the LEDC peripheral is used to generate the PWM on ENA, since
// analogWrite() does not exist natively (Arduino emulates it poorly).
//
// NOTE: if ENA is physically tied to 5V (jumper), use the 2-pin constructor.
//       Magnitude will then be controlled only through IN1/IN2 (ON/OFF).
//==============================================================================
class L298N
{
public:

    /// Full constructor: direction control + magnitude (PWM on ENA).
    /// @param in1 Digital pin IN1.
    /// @param in2 Digital pin IN2.
    /// @param ena PWM pin connected to ENA.
    /// @param pwm_frec_hz Desired PWM frequency for ENA (default 3 kHz).
    L298N(
    uint8_t in1,
    uint8_t in2,
    uint8_t ena,
    uint32_t pwm_frec_hz = 3000);

    /// Reduced constructor: ENA physically tied to logic high.
    /// Speed can only be 0 (coast/brake) or ±100 (full).
    /// @param in1 Digital pin IN1.
    /// @param in2 Digital pin IN2.
    L298N(
        uint8_t in1,
        uint8_t in2);

    /// Configures the pins and leaves the motor in coast.
    void begin();

    /// Sets the speed in the range -100..100 (clamped).
    /// If ENA is tied high (2-pin constructor), any non-zero value
    /// is treated as ±100.
    void set_speed(int speed);

    /// Last speed requested with set_speed().
    [[nodiscard]] int get_speed() const;

    /// true if the motor is not being driven right now.
    [[nodiscard]] bool is_stopped() const { return speed_ == 0; }

    /// Coast: no torque, the motor spins freely by inertia.
    void stop();

    /// Active braking: IN1 and IN2 at the same level (controlled short circuit).
    void brake();

    /// Reverses the rotation direction.
    void reversed(bool reversed = true);

private:

    void update_outputs();

private:

    uint8_t in1_;
    uint8_t in2_;

    // If has_ena_ is false, the physical jumper keeps ENA enabled
    // and the motor can only run at full speed or zero.
    bool has_ena_;
    uint8_t ena_;
    uint32_t pwm_frec_hz_;

    int speed_ = 0;

    bool reversed_ = false;
    bool braking_  = false;

    static constexpr int MIN_SPEED = -100;
    static constexpr int MAX_SPEED = 100;
};