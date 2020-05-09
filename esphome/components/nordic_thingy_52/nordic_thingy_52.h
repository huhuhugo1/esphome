#pragma once

#include "esphome/core/component.h"

#ifdef ARDUINO_ARCH_ESP32

#include <vector>
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_client/esp32_ble_client.h"

namespace esphome {
namespace nordic_thingy_52 {

class NordicThingy52 : public Component, public esp32_ble_client::ESP32BLEClient {
 public:
  NordicThingy52(const std::string& address);
  void set_temperature(sensor::Sensor *temperature);
  void set_pressure(sensor::Sensor *pressure);
  void set_humidity(sensor::Sensor *humidity);
  void setup() override;
  void loop() override;

 protected:
  std::vector<uint8_t> temperature_data_;
  std::vector<uint8_t> pressure_data_;
  std::vector<uint8_t> humidity_data_;
 
  sensor::Sensor* temperature_ = nullptr;
  sensor::Sensor* pressure_ = nullptr;
  sensor::Sensor* humidity_ = nullptr;
};

}  // namespace nordic_thingy_52
}  // namespace esphome

#endif
