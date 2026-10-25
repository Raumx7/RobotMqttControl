#include <Arduino.h>
#include "L298N.h"

namespace {
    int clamp(int val, int min, int max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}

//==============================================================================
// Constructors
//==============================================================================

L298N::L298N(uint8_t in1, uint8_t in2, uint8_t ena, uint32_t pwm_frec_hz)
:
in1_(in1),
in2_(in2),
has_ena_(true),
ena_(ena),
pwm_frec_hz_(pwm_frec_hz)
{
}

//==============================================================================

L298N::L298N(uint8_t in1, uint8_t in2)
:
in1_(in1),
in2_(in2),
has_ena_(false),
ena_(0),
pwm_frec_hz_(0)
{
}

//==============================================================================

void L298N::begin()
{
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);

    if (has_ena_)
    {
        ledcAttach(ena_, pwm_frec_hz_, 8);
    }

    stop();
}

//==============================================================================

void L298N::set_speed(int speed)
{
    speed_ = clamp(speed, MIN_SPEED, MAX_SPEED);
    braking_ = false;
    update_outputs();
}

//==============================================================================

int L298N::get_speed() const
{
    return speed_;
}

//==============================================================================

void L298N::stop()
{
    speed_ = 0;
    braking_ = false;
    update_outputs();
}

//==============================================================================

void L298N::brake()
{
    speed_ = 0;
    braking_ = true;
    update_outputs();
}

//==============================================================================

void L298N::reversed(bool reversed)
{
    reversed_ = reversed;
    update_outputs();
}

//==============================================================================

void L298N::update_outputs()
{
    // ---- Case with ENA controlled by PWM ----
    if (has_ena_)
    {
        if (braking_)
        {
            digitalWrite(in1_, HIGH);
            digitalWrite(in2_, HIGH);
            ledcWrite(ena_, 255);
            return;
        }

        const int effective_speed = reversed_ ? -speed_ : speed_;

        if (effective_speed == 0)
        {
            // Coast: bridge fully disabled through ENA.
            ledcWrite(ena_, 0);
            return;
        }

        const auto level = static_cast<uint8_t>(
            (abs(effective_speed) * 255) / 100);

        if (effective_speed > 0)
        {
            digitalWrite(in1_, HIGH);
            digitalWrite(in2_, LOW);
        }
        else
        {
            digitalWrite(in1_, LOW);
            digitalWrite(in2_, HIGH);
        }

        ledcWrite(ena_, level);
        return;
    }

    // ---- Case with ENA tied to logic high (ON/OFF only) ----
    if (braking_)
    {
        digitalWrite(in1_, HIGH);
        digitalWrite(in2_, HIGH);
        return;
    }

    const int effective_speed = reversed_ ? -speed_ : speed_;

    if (effective_speed == 0)
    {
        // Coast: with ENA fixed high, coast is achieved by turning off
        // both inputs. The motor is left floating.
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, LOW);
        return;
    }

    // Any non-zero value => full speed.
    if (effective_speed > 0)
    {
        digitalWrite(in1_, HIGH);
        digitalWrite(in2_, LOW);
    }
    else
    {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, HIGH);
    }
}
