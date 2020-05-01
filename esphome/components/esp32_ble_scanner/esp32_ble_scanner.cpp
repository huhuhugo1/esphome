#include "esp32_ble_scanner.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_scanner {

static const char *TAG = "esp32_ble_scanner";

std::unordered_map<uint16_t, ESP32BLENotificationHandler> ESP32BLEScanner::handlers_ =
  std::unordered_map<uint16_t, ESP32BLENotificationHandler>();

RingBuffer<ESP32BLENotification> ESP32BLEScanner::handlersToCall_ = RingBuffer<ESP32BLENotification>(5);

ESP32BLENotificationSubscriber::ESP32BLENotificationSubscriber(const std::string& server_name):
  server_name_(server_name),
  client_(BLEDevice::createClient())
{
  client_->setClientCallbacks(this);
}

const std::string& ESP32BLENotificationSubscriber::get_server_name() const {
  return server_name_;
}

void ESP32BLENotificationSubscriber::set_server_handle(std::unique_ptr<BLEAdvertisedDevice>& server_handle) {
  server_handle_ = std::move(server_handle);
}

bool ESP32BLENotificationSubscriber::connect_to_server() {
  ESP_LOGI(TAG, "Connecting to server '%s'...", server_name_.c_str());
  if (client_->connect(server_handle_.get())) {
    connected_ = true;
    register_all_handlers();
  } else {
    ESP_LOGW(TAG, "Connecting to server '%s' failed!", server_name_.c_str());
  }

  return connected_;
}

bool ESP32BLENotificationSubscriber::is_connected() const {
  return connected_;
}

void ESP32BLENotificationSubscriber::onConnect(BLEClient* client) {
  //ESP_LOGI(TAG, "Connected to server '%s'.", server_name_.c_str());
}

void ESP32BLENotificationSubscriber::onDisconnect(BLEClient* client) {
  //ESP_LOGI(TAG, "Disconnected from server '%s'.", server_name_.c_str());
  connected_ = false;
}

void ESP32BLENotificationSubscriber::add_notification_handler(const std::string& s_uuid, const std::string& c_uuid, const ESP32BLENotificationHandler& h) {
  auto it = subscriptions_.emplace(s_uuid, std::unordered_map<std::string, ESP32BLENotificationHandler>()).first;
  it->second.emplace(c_uuid, h);
}

void ESP32BLENotificationSubscriber::register_all_handlers() {
  for (const auto& s: subscriptions_) {
    BLERemoteService* remote_service = client_->getService(s.first);
    if (remote_service) {
      for (const auto& c: s.second) {
        BLERemoteCharacteristic* remote_characteristic = remote_service->getCharacteristic(c.first);
        if(remote_characteristic && remote_characteristic->canNotify()) {
          remote_characteristic->registerForNotify(ESP32BLEScanner::onNotify);
          ESP32BLEScanner::handlers_.emplace(remote_characteristic->getHandle(), c.second);
          if (auto d = remote_characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))) {
            uint16_t en = 1;
            d->writeValue((uint8_t*)&en, 2, true);
          }
        }
      }
    }
  }
}


ESP32BLEScanner::ESP32BLEScanner(int scan_window, int scan_interval, int scan_period, bool scan_active):
  scan_window_(scan_window), scan_interval_(scan_interval),
  scan_period_(scan_period), scan_active_(scan_active) {}

void ESP32BLEScanner::setup() {
  ::BLEDevice::init("");
  scanner_handle_ = ::BLEDevice::getScan();
  scanner_handle_->setAdvertisedDeviceCallbacks(this);
  scanner_handle_->setInterval(scan_interval_);
  scanner_handle_->setWindow(scan_window_);
  scanner_handle_->setActiveScan(scan_active_);

  time_of_last_scan_ = millis();
  start_scan();
}

void ESP32BLEScanner::loop() {
  if (client_to_be_connected_) {
    client_to_be_connected_->connect_to_server();
    client_to_be_connected_ = nullptr;
  }
  
  std::unique_ptr<BLEAdvertisedDevice> discovered_device;
  if (discovered_devices_.pop(discovered_device)) {
    ESP_LOGI(TAG, "Found server '%s'.", discovered_device->toString().c_str());
    auto it = registered_clients_.find(discovered_device->getName());
    if (it != registered_clients_.end() && !it->second->is_connected()) {
      it->second->set_server_handle(discovered_device);
      client_to_be_connected_ = it->second;
    }
    discovered_device.reset();
  }

  ESP32BLENotification n;
  if (handlersToCall_.pop(n)) {
    ESP32BLEScanner::handlers_[n.handle](n.data, n.length);
  }

  unsigned long current_time = millis();
  if (!discovered_device && (current_time - time_of_last_scan_) > scan_period_) {
    time_of_last_scan_ = current_time;
    start_scan();
  }
}

void ESP32BLEScanner::dump_config() {}

void ESP32BLEScanner::register_client(ESP32BLEClient* client) {
  registered_clients_.emplace(client->get_server_name(), client);
}

void ESP32BLEScanner::start_scan() {
  ESP_LOGI(TAG, "Start scanning ...");
  scanner_handle_->clearResults();
  scanner_handle_->start(20, nullptr, true);
}

void ESP32BLEScanner::onResult(BLEAdvertisedDevice advertisedDevice) {
  scanner_handle_->stop();
  if (!discovered_devices_.push(std::unique_ptr<BLEAdvertisedDevice>(new BLEAdvertisedDevice(advertisedDevice)))) {
    ESP_LOGW(TAG, "Scanned Value droped");
  }
}

void ESP32BLEScanner::onNotify(BLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  if (!ESP32BLEScanner::handlersToCall_.push({characteristic->getHandle(), data, length, isNotify})) {
    ESP_LOGW(TAG, "Value droped");
  }
}
}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif
