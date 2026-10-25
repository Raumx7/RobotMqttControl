#pragma once

#include <cstdint>
#include <Wire.h>

class AHT10_Light {
public:
  AHT10_Light() = default;

  // Initialize communication with the sensor throught I2C bus (default Wire instance)
  bool begin(TwoWire &wire_bus = Wire);

  // Reads temperature and humidity, stroring the result in the passed reference variables
  bool read_data(float &temperature, float &humidity) const;

private:
  // Puntero interno para recordar qué bus I2C le fue asignado a esta instancia
  TwoWire* i2c_bus_{nullptr};

  static constexpr uint8_t AHT10_ADDRESS = 0x38;
  static constexpr uint8_t CMD_INITIALIZE = 0xE1;
  static constexpr uint8_t CMD_MEASURE    = 0xAC;

  static constexpr uint8_t CAL_DATA_1    = 0x08;
  static constexpr uint8_t CAL_DATA_2    = 0x00;
  static constexpr uint8_t MEASURE_DATA_1 = 0x33;
  static constexpr uint8_t MEASURE_DATA_2 = 0x00;

  static constexpr uint8_t STATUS_BUSY_MASK = 0x80;
};
