#pragma once
#include <stdint.h>

//==============================================================================
// Driver para el puente H L298N (motor DC) adaptado a ESP32.
//
// El L298N separa dirección (IN1/IN2, digitales) de magnitud (ENA, PWM).
// En ESP32 se usa el periférico LEDC para generar el PWM en ENA, ya que
// analogWrite() no existe de forma nativa (Arduino lo emula de forma pobre).
//
// NOTA: si ENA está puenteado físicamente a 5V (jumper), usa el constructor
//       de 2 pines. La magnitud se controlará solo por IN1/IN2 (ON/OFF).
//==============================================================================
class L298N
{
public:

    /// Constructor completo: control de dirección + magnitud (PWM en ENA).
    /// @param in1 Pin digital IN1.
    /// @param in2 Pin digital IN2.
    /// @param ena Pin PWM conectado a ENA.
    /// @param pwm_frec_hz Frecuencia deseada de PWM para ENA (por defecto 3 kHz).
    L298N(
    uint8_t in1,
    uint8_t in2,
    uint8_t ena,
    uint32_t pwm_frec_hz = 3000);

    /// Constructor reducido: ENA puenteado físicamente a nivel alto.
    /// La velocidad solo puede ser 0 (coast/brake) o ±100 (full).
    /// @param in1 Pin digital IN1.
    /// @param in2 Pin digital IN2.
    L298N(
        uint8_t in1,
        uint8_t in2);

    /// Configura los pines y deja el motor en coast.
    void init();

    /// Fija la velocidad, en el rango -100..100 (se clampa).
    /// Si ENA está puenteado (constructor de 2 pines), cualquier valor
    /// distinto de 0 se trata como ±100.
    void set_speed(int speed);

    /// Última velocidad pedida con set_speed().
    int get_speed() const;

    /// true si el motor no está siendo impulsado ahora mismo.
    bool is_stopped() const { return speed_ == 0; }

    /// Motor en coast: sin torque, gira libre por inercia.
    void stop();

    /// Frenado activo: IN1 e IN2 al mismo nivel (cortocircuito controlado).
    void brake();

    /// Invierte el sentido de giro.
    void set_reversed(bool reversed = true);

private:

    void update_outputs();

private:

    uint8_t in1_;
    uint8_t in2_;

    // Si has_ena_ es false, el jumper físico mantiene ENA activo
    // y el motor solo puede ir a full o a cero.
    bool has_ena_;
    uint8_t ena_;
    uint32_t pwm_frec_hz_;

    int speed_ = 0;

    bool reversed_ = false;
    bool braking_  = false;

    static constexpr int MIN_SPEED = -100;
    static constexpr int MAX_SPEED = 100;
};
