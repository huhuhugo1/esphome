#pragma once

#include "esphome/core/component.h"

#ifdef ARDUINO_ARCH_ESP32

#include "BLEDevice.h"
#include <unordered_map>
#include <string>
#include <memory>
#include <freertos/task.h>


namespace esphome {
namespace esp32_ble_scanner {
using ESP32BLENotificationHandler = std::function<void(uint8_t* data, size_t length)>;

template<typename T>
class RingBuffer {
  T* data = nullptr;
  size_t r_idx = 0;
  size_t w_idx = 0;
  size_t capacity = 0;
  size_t size = 0;
  SemaphoreHandle_t lock = xSemaphoreCreateMutex();

 public:
  RingBuffer(size_t cap) {
    data = new T[cap];
    capacity = cap;
  }

  ~RingBuffer() {
    delete[] data;
  }
  
  bool push(T&& val) {
    if (!xSemaphoreTake(lock, 5L / portTICK_PERIOD_MS)) return false;

    if (size == capacity) {
      xSemaphoreGive(lock);
      return false;
    }

    data[w_idx] = std::move(val);
    w_idx = (w_idx + 1) % capacity;
    size++;
    
    xSemaphoreGive(lock);
    return true;
  }

  bool pop(T& val) {
    if (!xSemaphoreTake(lock, 0)) return false;

    if (size == 0) {
      xSemaphoreGive(lock);
      return false;
    }

    val = std::move(data[r_idx]);
    r_idx = (r_idx + 1) % capacity;
    size--;

    xSemaphoreGive(lock);
    return true;
  }
};

struct ESP32BLENotification {
  uint16_t handle;
  uint8_t* data; 
  size_t length;
  bool isNotify;
};

class ESP32BLEClient {
 public:
  virtual const std::string& get_server_name() const = 0;
  virtual void set_server_handle(std::unique_ptr<BLEAdvertisedDevice>& s) = 0;
  virtual bool connect_to_server() = 0;
  virtual bool is_connected() const = 0;
};

class ESP32BLENotificationSubscriber : public ESP32BLEClient, public BLEClientCallbacks {
 public:
  ESP32BLENotificationSubscriber(const std::string& server_name);
  const std::string& get_server_name() const override;
  void set_server_handle(std::unique_ptr<BLEAdvertisedDevice>& server_handle) override;
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
  std::unique_ptr<BLEAdvertisedDevice> server_handle_;
  bool connected_ = false;
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
  ESP32BLEClient* client_to_be_connected_ = nullptr;
  RingBuffer<std::unique_ptr<BLEAdvertisedDevice>> discovered_devices_ = RingBuffer<std::unique_ptr<BLEAdvertisedDevice>>(5);
  unsigned long time_of_last_scan_;
  BLEScan* scanner_handle_ = nullptr;

 public:
  static std::unordered_map<uint16_t, ESP32BLENotificationHandler> handlers_;
  static RingBuffer<ESP32BLENotification> handlersToCall_;
  static void onNotify(BLERemoteCharacteristic*, uint8_t*, size_t, bool);
};
}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif
