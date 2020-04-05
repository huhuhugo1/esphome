#include "esp32_ble_scanner.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_scanner {
  
std::unordered_map<uint16_t, ESP32BLEClient::ESP32BLEClientCallback> ESP32BLEClient::callbacks = 
  std::unordered_map<uint16_t, ESP32BLEClient::ESP32BLEClientCallback>();
  
void ESP32BLEClient::notifyCallback(BLERemoteCharacteristic* characteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.println(characteristic->getUUID().toString().c_str());
  ESP32BLEClient::callbacks[characteristic->getHandle()](pData, length);
}

bool ESP32BLEClient::connect() {
  // Connect to the remove BLE Server.
  if (pClient->connect(server)) {
    connected = true;
  } else {
    ESP_LOGE("TAG", "Unable to connect");
  }
  return connected;
}

void ESP32BLEClient::register_handlers() {
  for (const auto& s: subscriptions) {
    BLERemoteService* pRemoteService = pClient->getService(s.first);
    if (pRemoteService) {
      for (const auto& c: s.second) {
        BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(c.first);
        if(pRemoteCharacteristic && pRemoteCharacteristic->canNotify()) {
          pRemoteCharacteristic->registerForNotify(ESP32BLEClient::notifyCallback);
          ESP32BLEClient::callbacks.emplace(pRemoteCharacteristic->getHandle(), c.second);
          if (auto d = pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))) {
            uint16_t en = 1;
            d->writeValue((uint8_t*)&en, 2, true);
          }
        }
      }
    }
  }
}

void ESP32BLEClient::register_callback(const std::string& s_uuid, const std::string& c_uuid, const ESP32BLEClientCallback& cb) {
  auto it = subscriptions.emplace(s_uuid, std::unordered_map<std::string, ESP32BLEClientCallback>()).first;
  it->second.emplace(c_uuid, cb);
}


void ESP32BLEScanner::start_scan() {
  ESP_LOGI("TAG", "Start scanning ...");
  pBLEScan->start(10, false);
}

void ESP32BLEScanner::setup() {
  ::BLEDevice::init("");
  pBLEScan = ::BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(this);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  
  time_of_last_scan = millis();
  start_scan();
}

void ESP32BLEScanner::onResult(BLEAdvertisedDevice advertisedDevice) {
  ESP_LOGI("esp32_ble_scanner", "found:%s", advertisedDevice.getName().c_str());
  auto it = registered_clients.find(advertisedDevice.getName());
  if (it == registered_clients.end() || it->second->isConnected())
    return;
  
  it->second->set_server(advertisedDevice);
  clients_to_be_connected.push(it->second);
}

void ESP32BLEScanner::loop() {
  if (!clients_to_be_connected.empty()) {
    auto client = clients_to_be_connected.front();
    clients_to_be_connected.pop();
    if(client->connect())
      client->register_handlers();
  }

  unsigned long current_time = millis();
  if (current_time - time_of_last_scan > 10000) {
    time_of_last_scan = current_time;
    start_scan();
  }
}

void ESP32BLEScanner::dump_config() {}

void ESP32BLEScanner::register_client(const std::string& name, ESP32BLEClient* client) {
  registered_clients.emplace(name, client);
}

}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif
