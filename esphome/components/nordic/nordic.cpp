#include "nordic.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace nordic {

static const char *TAG = "nordic";
constexpr auto ENVIROMENT_SERVICE_UUID = "ef680200-9b35-4933-9b10-52ffa9740042";
constexpr auto TEMPERATURE_CHAR_UUID = "ef680201-9b35-4933-9b10-52ffa9740042";
constexpr auto PRESSURE_CHAR_UUID = "ef680202-9b35-4933-9b10-52ffa9740042";
constexpr auto HUMIDITY_CHAR_UUID = "ef680203-9b35-4933-9b10-52ffa9740042";

Nordic::Nordic(const std::string& server_name):
  esp32_ble_scanner::ESP32BLENotificationSubscriber(server_name) {}

void Nordic::dump_config() {
  ESP_LOGCONFIG(TAG, "Nordic");
}

void Nordic::set_temperature(sensor::Sensor *temperature) {
  add_notification_handler(ENVIROMENT_SERVICE_UUID, TEMPERATURE_CHAR_UUID, [temperature](uint8_t* d, size_t s) {
    if (s == 2) {
      float val = d[0];
      val += 0.01 * d[1];
      temperature->publish_state(val);
    } else {
      ESP_LOGW(TAG, "Unable to decode received temperature.");
    }
  });
}

void Nordic::set_pressure(sensor::Sensor *pressure) {
  add_notification_handler(ENVIROMENT_SERVICE_UUID, PRESSURE_CHAR_UUID, [pressure](uint8_t* d, size_t s) {
    if (s == 5) {
      float val = *reinterpret_cast<int32_t*>(d);
      val += 0.01 * d[4];
      pressure->publish_state(val);
    } else {
      ESP_LOGW(TAG, "Unable to decode received pressure.");
    }
  });
}

void Nordic::set_humidity(sensor::Sensor *humidity) {
  add_notification_handler(ENVIROMENT_SERVICE_UUID, HUMIDITY_CHAR_UUID, [humidity](uint8_t* d, size_t s) {
    if (s == 1) {
      humidity->publish_state(d[0]);
    } else {
      ESP_LOGW(TAG, "Unable to decode received humidity.");
    }
  });
}

void Nordic::set_battery_level(sensor::Sensor *battery_level) {
  add_notification_handler(ENVIROMENT_SERVICE_UUID, "ef680201-9b35-4933-9b10-52ffa9740042", [battery_level](uint8_t* d, size_t s) {
    if (s == 1) {
      battery_level->publish_state(d[0]);
    } else {
      ESP_LOGW(TAG, "Unable to decode received battery level.");
    }
  });
}
}  // namespace nordic
}  // namespace esphome

#endif
