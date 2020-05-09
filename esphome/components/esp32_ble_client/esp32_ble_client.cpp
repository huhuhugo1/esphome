#include "esp32_ble_client.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_client {

static const char *TAG = "esp32_ble_client";

std::unordered_map<uint16_t, std::vector<uint8_t>*> ESP32BLEClient::handle_to_vec_mapping_ =
  std::unordered_map<uint16_t, std::vector<uint8_t>*>();

ESP32BLEClient::ESP32BLEClient(const std::string& address, esp_ble_addr_type_t type) {
  mac_address_ = address;
  mac_address_type_ = type;
  client_ = BLEDevice::createClient();
}

void ESP32BLEClient::connect() {
  ESP_LOGI(TAG, "Connecting to server '%s'...", mac_address_.c_str());
  if (client_->connect(BLEAddress(mac_address_), mac_address_type_))
    register_all_subscribed_characteristics();
  else
    ESP_LOGW(TAG, "Connecting to server '%s' failed!", mac_address_.c_str());
}

bool ESP32BLEClient::is_connected() const { return client_->isConnected(); }

void ESP32BLEClient::add_subscribed_characteristic(const std::string& s_uuid, const std::string& c_uuid, std::vector<uint8_t>* vec) {
  auto it = uuid_to_vec_mapping_.emplace(s_uuid, std::unordered_map<std::string, std::vector<uint8_t>*>()).first;
  it->second.emplace(c_uuid, vec);
}

void ESP32BLEClient::register_all_subscribed_characteristics() {
  for (const auto& s: uuid_to_vec_mapping_) {
    BLERemoteService* remote_service = client_->getService(s.first);
    if (remote_service) {
      for (const auto& c: s.second) {
        BLERemoteCharacteristic* remote_characteristic = remote_service->getCharacteristic(c.first);
        if(remote_characteristic && remote_characteristic->canNotify()) {
          remote_characteristic->registerForNotify(ESP32BLEClient::onNotify);
          ESP32BLEClient::handle_to_vec_mapping_.emplace(remote_characteristic->getHandle(), c.second);
          if (auto d = remote_characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))) {
            uint16_t en = 1;
            d->writeValue((uint8_t*)&en, 2, true);
          }
        }
      }
    }
  }
}

void ESP32BLEClient::onNotify(BLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  ESP32BLEClient::handle_to_vec_mapping_[characteristic->getHandle()]->assign(data, data+length);
}

}  // namespace esp32_ble_client
}  // namespace esphome

#endif
