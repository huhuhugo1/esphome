#include "esp32_ble_scanner.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_ble_scanner {

static const char *TAG = "esp32_ble_scanner";

std::unordered_map<uint16_t, ESP32BLENotificationHandler> ESP32BLENotificationSubscriber::handlers_ =
  std::unordered_map<uint16_t, ESP32BLENotificationHandler>();

ESP32BLENotificationSubscriber::ESP32BLENotificationSubscriber(const std::string& server_name):
  server_name_(server_name),
  client_(BLEDevice::createClient())
{
  client_->setClientCallbacks(this);
}

const std::string& ESP32BLENotificationSubscriber::get_server_name() const {
  return server_name_;
}

void ESP32BLENotificationSubscriber::set_server_handle(BLEAdvertisedDevice& server_handle) {
  if (server_handle_) delete server_handle_;
  server_handle_ = new BLEAdvertisedDevice(server_handle);
}

bool ESP32BLENotificationSubscriber::connect_to_server() {
  ESP_LOGI(TAG, "Connecting to server '%s'...", server_name_.c_str());
  if (client_->connect(server_handle_)) {
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
  ESP_LOGI(TAG, "Connected to server '%s'.", server_name_.c_str());
}

void ESP32BLENotificationSubscriber::onDisconnect(BLEClient* client) {
  ESP_LOGI(TAG, "Disconnected from server '%s'.", server_name_.c_str());
  connected_ = false;
}

void ESP32BLENotificationSubscriber::notifyCallback(BLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  ESP32BLENotificationSubscriber::handlers_[characteristic->getHandle()](data, length);
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
          remote_characteristic->registerForNotify(ESP32BLENotificationSubscriber::notifyCallback);
          ESP32BLENotificationSubscriber::handlers_.emplace(remote_characteristic->getHandle(), c.second);
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
  if (!clients_to_be_connected_.empty()) {
    auto client = clients_to_be_connected_.front();
    clients_to_be_connected_.pop();
    client->connect_to_server();
  }

  unsigned long current_time = millis();
  if (current_time - time_of_last_scan_ > scan_period_) {
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
  scanner_handle_->start(10, false);
}

void ESP32BLEScanner::onResult(BLEAdvertisedDevice advertisedDevice) {
  ESP_LOGI(TAG, "Found server '%s'.", advertisedDevice.getName().c_str());
  auto it = registered_clients_.find(advertisedDevice.getName());
  if (it == registered_clients_.end() || it->second->is_connected())
    return;

  it->second->set_server_handle(advertisedDevice);
  clients_to_be_connected_.push(it->second);
}
}  // namespace esp32_ble_scanner
}  // namespace esphome

#endif
