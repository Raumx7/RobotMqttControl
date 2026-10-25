#include "AHT10_Light.h"
#include <Arduino.h>

bool AHT10_Light::begin(TwoWire &wire_bus) {
    i2c_bus_ = &wire_bus;

    // Verify the device response to the slave address
    i2c_bus_->beginTransmission(AHT10_ADDRESS);
    if (i2c_bus_->endTransmission() != 0) {
        return false;
    }

    // Initialize the sensor
    i2c_bus_->beginTransmission(AHT10_ADDRESS);
    i2c_bus_->write(CMD_INITIALIZE);
    i2c_bus_->write(CAL_DATA_1);
    i2c_bus_->write(CAL_DATA_2);
    if (i2c_bus_->endTransmission() != 0) {
        return false;
    }

    delay(20);
    return true;
}

bool AHT10_Light::read_data(float &temperature, float &humidity) const {
    if (i2c_bus_ == nullptr) return false; // When begin() is not called

    // Trigger meassures
    i2c_bus_->beginTransmission(AHT10_ADDRESS);
    i2c_bus_->write(CMD_MEASURE);
    i2c_bus_->write(MEASURE_DATA_1);
    i2c_bus_->write(MEASURE_DATA_2);
    if (i2c_bus_->endTransmission() != 0) {
        return false;
    }

    delay(80);

    // Ask the bus for data (6 bytes)
    if (i2c_bus_->requestFrom(AHT10_ADDRESS, static_cast<uint8_t>(6)) != 6) {
        return false;
    }

    uint8_t buffer[6]{};
    for (auto &byte : buffer) {
        byte = i2c_bus_->read();
    }

    if ((buffer[0] & STATUS_BUSY_MASK) != 0) {
        return false;
    }

    // Convert raw data
    const uint32_t raw_humidity = (((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3]) >> 4;
    const uint32_t raw_temperature = (((uint32_t)buffer[3] & 0x0F) << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];

    constexpr float conversion_factor = 1048576.0f;
    humidity = (static_cast<float>(raw_humidity) * 100.0f) / conversion_factor;
    temperature = ((static_cast<float>(raw_temperature) * 200.0f) / conversion_factor) - 50.0f;

    return true;
}
