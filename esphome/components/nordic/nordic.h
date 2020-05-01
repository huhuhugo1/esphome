#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_scanner/esp32_ble_scanner.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace nordic {

class Nordic : public Component, public esp32_ble_scanner::ESP32BLENotificationSubscriber {
 public:
  Nordic(const std::string& server_name);
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_temperature(sensor::Sensor *temperature);
  void set_pressure(sensor::Sensor *pressure);
  void set_humidity(sensor::Sensor *humidity);
  void set_battery_level(sensor::Sensor *battery_level);
};

}  // namespace nordic
}  // namespace esphome

#endif
