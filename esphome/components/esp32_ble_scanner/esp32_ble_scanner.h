#pragma once

#include "esphome/core/component.h"
#include "BLEDevice.h"
#include <unordered_map>
#include <queue>
#include <string>

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_scanner {

class ESP32BLEClient : public BLEClientCallbacks {
  using ESP32BLEClientCallback = std::function<void(uint8_t* pData, size_t length)>;
  
  static std::unordered_map<uint16_t, ESP32BLEClientCallback> callbacks;
  static void notifyCallback(BLERemoteCharacteristic*, uint8_t*, size_t, bool);

  bool connected = false;
  BLEClient* pClient = nullptr;
  BLEAdvertisedDevice* server = nullptr;
  std::unordered_map<std::string, std::unordered_map<std::string, ESP32BLEClientCallback>> subscriptions;
 
 public:
  ESP32BLEClient() {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(this);
  }
  
  void onConnect(BLEClient* pclient) override {}

  void onDisconnect(BLEClient* pclient) override {
    ESP_LOGE("TAG", "Disconnected");
    connected = false;
  }

  void set_server(BLEAdvertisedDevice& s) {
    if (server) delete server;
    server = new BLEAdvertisedDevice(s);
  }

  bool connect();
  bool isConnected() { return connected; }
  bool isServerSet() { return server; }
  void register_handlers();
  void register_callback(const std::string& s_uuid, const std::string& c_uuid, const ESP32BLEClientCallback& cb);
};

class ESP32BLEScanner: public Component, public BLEAdvertisedDeviceCallbacks {
  std::unordered_map<std::string, ESP32BLEClient*> registered_clients;
  std::queue<ESP32BLEClient*> clients_to_be_connected;
  unsigned long time_of_last_scan;
  BLEScan* pBLEScan = nullptr;
  
  void start_scan();

 public:
  void setup() override;
  void onResult(BLEAdvertisedDevice advertisedDevice) override;
  void loop() override;
  void dump_config() override;
  void register_client(const std::string& name, ESP32BLEClient* client);
};

}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif