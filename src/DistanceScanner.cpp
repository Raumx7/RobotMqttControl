#include "DistanceScanner.h"

DistanceScanner::DistanceScanner(VL53L0X &sensor_ft, VL53L0X &sensor_bk)
    : sensor_ft_(sensor_ft), sensor_bk_(sensor_bk)
{
}

void DistanceScanner::set_collision_limit(float collision_limit) {
    collision_limit_ = collision_limit;
}

void DistanceScanner::set_offsets(float offset_ft, float offset_bk) {
    offset_ft_ = offset_ft;
    offset_bk_ = offset_bk;
}

bool DistanceScanner::detects_collision() const {
    return distances_[0] <= collision_limit_ || distances_[1] <= collision_limit_;
}

bool DistanceScanner::update() {
    bool success = false;

    const uint16_t distance_ft = sensor_ft_.readRangeContinuousMillimeters();

    if (!sensor_ft_.timeoutOccurred()) {
        distances_[0] = (static_cast<float>(distance_ft) / 10.0f) - offset_ft_;
        success = true;
    }

    const uint16_t distance_bk = sensor_bk_.readRangeContinuousMillimeters();

    if (!sensor_bk_.timeoutOccurred()) {
        distances_[1] = (static_cast<float>(distance_bk) / 10.0f) - offset_bk_;
        success = true;
    }

    return success;
}
