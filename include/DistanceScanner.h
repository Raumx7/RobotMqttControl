#pragma once

#include <VL53L0X.h>

class DistanceScanner {
public:
    // Constructor
    DistanceScanner(VL53L0X& sensor_ft, VL53L0X& sensor_bk);

    // Sets the distance collision limit
    void set_collision_limit(float collision_limit);

    // Sets distance offsets to sensor measurements
    void set_offsets(float offset_ft, float offset_bk);

    // Returns a pointer to the distance array
    [[nodiscard]] const float* get_distances() const { return distances_; }

    // True when detecting a dangerous collision distance
    [[nodiscard]] bool detects_collision() const;

    // Updates distance array with sensor readings (cm) and offsets
    bool update();

private:
    // Robot distance sensors
    VL53L0X& sensor_ft_;
    VL53L0X& sensor_bk_;

    // Distance collision limit
    float collision_limit_ = 5.0f;

    // Measurements offsets
    float offset_ft_ = 0.0f;
    float offset_bk_ = 0.0f;

    // Distances from sensors (0 -> ft, 1 -> bk) in cm
    float distances_[2] = {0.0f, 0.0f};
};
