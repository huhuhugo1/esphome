#include "nordic_thingy_52.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace nordic_thingy_52 {

static const char *TAG = "nordic_thingy_52";

constexpr auto ENVIROMENT_SERVICE_UUID = "ef680200-9b35-4933-9b10-52ffa9740042";
constexpr auto TEMPERATURE_CHAR_UUID = "ef680201-9b35-4933-9b10-52ffa9740042";
constexpr auto PRESSURE_CHAR_UUID = "ef680202-9b35-4933-9b10-52ffa9740042";
constexpr auto HUMIDITY_CHAR_UUID = "ef680203-9b35-4933-9b10-52ffa9740042";

NordicThingy52::NordicThingy52(const std::string& address):
  esp32_ble_client::ESP32BLEClient(address)
{
  temperature_data_.reserve(10);
  pressure_data_.reserve(10);
  humidity_data_.reserve(10);
}

void NordicThingy52::set_temperature(sensor::Sensor *temperature) {
  temperature_ = temperature;
}

void NordicThingy52::set_pressure(sensor::Sensor *pressure) {
  pressure_ = pressure;
}

void NordicThingy52::set_humidity(sensor::Sensor *humidity) {
  humidity_ = humidity;
}

void NordicThingy52::setup() {
  add_subscribed_characteristic(ENVIROMENT_SERVICE_UUID, TEMPERATURE_CHAR_UUID, &temperature_data_);
  add_subscribed_characteristic(ENVIROMENT_SERVICE_UUID, PRESSURE_CHAR_UUID, &pressure_data_);
  add_subscribed_characteristic(ENVIROMENT_SERVICE_UUID, HUMIDITY_CHAR_UUID, &humidity_data_);
  BLEDevice::init("");
}

void NordicThingy52::loop() {
  if (!is_connected())
    connect();
  
  size_t s;
  if (temperature_ && (s = temperature_data_.size())) {
    if (s == 2) {
      float val = temperature_data_[0];
      val += 0.01 * temperature_data_[1];
      temperature_->publish_state(val);
    } else {
      ESP_LOGW(TAG, "Unable to decode received temperature.");
    }
    temperature_data_.clear();
  }

  if (pressure_ && (s = pressure_data_.size())) {
    if (s == 5) {
      float val = *reinterpret_cast<int32_t*>(pressure_data_.data());
      val += 0.01 * pressure_data_[4];
      pressure_->publish_state(val);
    } else {
      ESP_LOGW(TAG, "Unable to decode received pressure.");
    }
    pressure_data_.clear();
  }

  if (humidity_ && (s = humidity_data_.size())) {
    if (s == 1) {
      humidity_->publish_state(humidity_data_[0]);
    } else {
      ESP_LOGW(TAG, "Unable to decode received humidity.");
    }
    humidity_data_.clear();
  }
}

}  // namespace nordic_thingy_52
}  // namespace esphome

#endif
