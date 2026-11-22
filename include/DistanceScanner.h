#pragma once

#include <VL53L0X.h>
#include <cstdint>

struct ScanDistances {
    float front = 0.0f;
    float back  = 0.0f;
    bool front_valid = false;
    bool back_valid  = false;
};

class DistanceScanner {
public:
    // Recibe sensores YA inicializados y en modo continuo por el main
    DistanceScanner(VL53L0X& sensor_ft, VL53L0X& sensor_bk);

    void set_collision_limit(float collision_limit);
    void set_offset_ft(float offset_ft);
    void set_offset_bk(float offset_bk);

    [[nodiscard]] ScanDistances get_distances() const { return data_; }
    [[nodiscard]] bool detects_front_collision() const;
    [[nodiscard]] bool detects_back_collision() const;
    [[nodiscard]] bool detects_any_collision() const;

    // Devuelve true si al menos un sensor trajo una lectura nueva en este ciclo
    bool update();

private:
    static constexpr uint16_t kMaxValidMm     = 2000;  // > 200 cm se considera fuera de rango
    static constexpr uint32_t kPollIntervalMs = 50;

    // Devuelve true si hubo dato nuevo (válido o no); out_valid refleja si la
    // medición es confiable. Si no hay dato listo, no toca out_dist/out_valid.
    bool update_single_sensor(VL53L0X& sensor, float offset_cm, float& out_dist, bool& out_valid);
    float to_centimeters(uint16_t raw_mm, float offset_cm) const;

    VL53L0X& sensor_ft_;
    VL53L0X& sensor_bk_;

    float collision_limit_ = 5.0f;
    float offset_ft_ = 0.0f;
    float offset_bk_ = 0.0f;

    unsigned long last_poll_ms_ = 0;
    ScanDistances data_{};
};
