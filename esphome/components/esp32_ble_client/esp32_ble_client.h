#pragma once

#include "esphome/core/component.h"

#ifdef ARDUINO_ARCH_ESP32

#include "BLEDevice.h"
#include <unordered_map>
#include <string>

namespace esphome {
namespace esp32_ble_client {

class ESP32BLEClient {
 public:
  ESP32BLEClient(const std::string& address, esp_ble_addr_type_t type = BLE_ADDR_TYPE_RANDOM);
  void connect();
  bool is_connected() const;

 protected:
  void add_subscribed_characteristic(const std::string& s_uuid, const std::string& c_uuid, std::vector<uint8_t>* vec);
  void register_all_subscribed_characteristics();
  static void onNotify(BLERemoteCharacteristic*, uint8_t*, size_t, bool);

  std::string mac_address_;
  esp_ble_addr_type_t mac_address_type_;
  BLEClient* client_;
  
  std::unordered_map<std::string, std::unordered_map<std::string, std::vector<uint8_t>*>> uuid_to_vec_mapping_;
  static std::unordered_map<uint16_t, std::vector<uint8_t>*> handle_to_vec_mapping_;
};

}  // namespace esp32_ble_client
}  // namespace esphome

#endif
