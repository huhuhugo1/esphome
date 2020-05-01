#pragma once

#include "esphome/core/component.h"
#include "BLEDevice.h"
#include <unordered_map>
#include <queue>
#include <string>

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_scanner {
using ESP32BLENotificationHandler = std::function<void(uint8_t* data, size_t length)>;

class ESP32BLEClient {
 public:
  virtual const std::string& get_server_name() const = 0;
  virtual void set_server_handle(BLEAdvertisedDevice& s) = 0;
  virtual bool connect_to_server() = 0;
  virtual bool is_connected() const = 0;
};

class ESP32BLENotificationSubscriber : public ESP32BLEClient, public BLEClientCallbacks {
 public:
  ESP32BLENotificationSubscriber(const std::string& server_name);
  const std::string& get_server_name() const override;
  void set_server_handle(BLEAdvertisedDevice& server_handle) override;
  bool connect_to_server() override;
  bool is_connected() const override;

  // BLEClientCallbacks
  void onConnect(BLEClient* client) override;
  void onDisconnect(BLEClient* client) override;

 protected:
  void add_notification_handler(const std::string& s_uuid, const std::string& c_uuid, const ESP32BLENotificationHandler& h);
  void register_all_handlers();

  std::unordered_map<std::string, std::unordered_map<std::string, ESP32BLENotificationHandler>> subscriptions_;
  std::string const server_name_;
  BLEClient* const client_;
  BLEAdvertisedDevice* server_handle_ = nullptr;
  bool connected_ = false;

  static std::unordered_map<uint16_t, ESP32BLENotificationHandler> handlers_;
  static void notifyCallback(BLERemoteCharacteristic*, uint8_t*, size_t, bool);
};

class ESP32BLEScanner: public Component, public BLEAdvertisedDeviceCallbacks {
 public:
  ESP32BLEScanner(int scan_window, int scan_interval, int scan_period, bool scan_active);
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_client(ESP32BLEClient* client);

 protected:
  void start_scan();
  void onResult(BLEAdvertisedDevice advertisedDevice) override;

  const int scan_window_;
  const int scan_interval_;
  const int scan_period_;
  const bool scan_active_;

  std::unordered_map<std::string, ESP32BLEClient*> registered_clients_;
  std::queue<ESP32BLEClient*> clients_to_be_connected_;
  unsigned long time_of_last_scan_;
  BLEScan* scanner_handle_ = nullptr;
};
}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif
