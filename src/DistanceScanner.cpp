#include "DistanceScanner.h"

DistanceScanner::DistanceScanner(VL53L0X& sensor_ft, VL53L0X& sensor_bk)
    : sensor_ft_(sensor_ft), sensor_bk_(sensor_bk) {}

void DistanceScanner::set_collision_limit(float collision_limit) {
    collision_limit_ = collision_limit;
}

void DistanceScanner::set_offset_ft(float offset_ft) {
    offset_ft_ = offset_ft;
}

void DistanceScanner::set_offset_bk(float offset_bk) {
    offset_bk_ = offset_bk;
}

bool DistanceScanner::detects_front_collision() const {
    return data_.front_valid && (data_.front <= collision_limit_);
}

bool DistanceScanner::detects_back_collision() const {
    return data_.back_valid && (data_.back <= collision_limit_);
}

bool DistanceScanner::detects_any_collision() const {
    return detects_front_collision() || detects_back_collision();
}

float DistanceScanner::to_centimeters(uint16_t raw_mm, float offset_cm) const {
    const float cm = (static_cast<float>(raw_mm) / 10.0f) - offset_cm;
    return cm < 0.0f ? 0.0f : cm;
}

bool DistanceScanner::update_single_sensor(VL53L0X& sensor, float offset_cm, float& out_dist, bool& out_valid) {
    // Consulta no bloqueante: solo lee si hay dato nuevo listo
    const uint8_t status = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if ((status & 0x07) == 0) {
        return false;   // sin dato nuevo; conserva out_dist/out_valid previos
    }

    const uint16_t raw_mm = sensor.readRangeContinuousMillimeters();

    if (sensor.timeoutOccurred() || raw_mm >= kMaxValidMm) {
        out_valid = false;
        return true;    // hubo intento de lectura, pero no es confiable
    }

    out_dist  = to_centimeters(raw_mm, offset_cm);
    out_valid = true;
    return true;
}

bool DistanceScanner::update() {
    const unsigned long now = millis();
    if (now - last_poll_ms_ < kPollIntervalMs) {
        return false;
    }
    last_poll_ms_ = now;

    const bool ft_updated = update_single_sensor(sensor_ft_, offset_ft_, data_.front, data_.front_valid);
    const bool bk_updated = update_single_sensor(sensor_bk_, offset_bk_, data_.back,  data_.back_valid);

    return ft_updated || bk_updated;
}
