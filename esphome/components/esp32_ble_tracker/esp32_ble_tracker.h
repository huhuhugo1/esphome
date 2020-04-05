#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#ifdef ARDUINO_ARCH_ESP32

#include <string>
#include <array>
#include <unordered_map>
#include <stdint.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_defs.h>

namespace esphome {
namespace esp32_ble_tracker {

class ESPBTUUID {
 public:
  ESPBTUUID();

  static ESPBTUUID from_uint16(uint16_t uuid);

  static ESPBTUUID from_uint32(uint32_t uuid);

  static ESPBTUUID from_raw(const uint8_t *data);

  bool contains(uint8_t data1, uint8_t data2) const;

  esp_bt_uuid_t get_uuid();

  std::string to_string();

 protected:
  esp_bt_uuid_t uuid_;
};

using adv_data_t = std::vector<uint8_t>;

struct ServiceData {
  ESPBTUUID uuid;
  adv_data_t data;
};

class ESPBLEiBeacon {
 public:
  ESPBLEiBeacon() { memset(&this->beacon_data_, 0, sizeof(this->beacon_data_)); }
  ESPBLEiBeacon(const uint8_t *data);
  static optional<ESPBLEiBeacon> from_manufacturer_data(const ServiceData &data);

  uint16_t get_major() { return reverse_bits_16(this->beacon_data_.major); }
  uint16_t get_minor() { return reverse_bits_16(this->beacon_data_.minor); }
  int8_t get_signal_power() { return this->beacon_data_.signal_power; }
  ESPBTUUID get_uuid() { return ESPBTUUID::from_raw(this->beacon_data_.proximity_uuid); }

 protected:
  struct {
    uint8_t sub_type;
    uint8_t proximity_uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t signal_power;
  } PACKED beacon_data_;
};

class ESPBTDevice {
 public:
  void parse_scan_rst(const esp_ble_gap_cb_param_t::ble_scan_result_evt_param &param);

  std::string address_str() const;

  uint64_t address_uint64() const;

  esp_ble_addr_type_t get_address_type() const { return this->address_type_; }
  int get_rssi() const { return rssi_; }
  const std::string &get_name() const { return this->name_; }

  ESPDEPRECATED("Use get_tx_powers() instead")
  optional<int8_t> get_tx_power() const {
    if (this->tx_powers_.empty())
      return {};
    return this->tx_powers_[0];
  }
  const std::vector<int8_t> &get_tx_powers() const { return tx_powers_; }

  const optional<uint16_t> &get_appearance() const { return appearance_; }
  const optional<uint8_t> &get_ad_flag() const { return ad_flag_; }
  const std::vector<ESPBTUUID> &get_service_uuids() const { return service_uuids_; }

  const std::vector<ServiceData> &get_manufacturer_datas() const { return manufacturer_datas_; }

  const std::vector<ServiceData> &get_service_datas() const { return service_datas_; }

  optional<ESPBLEiBeacon> get_ibeacon() const {
    for (auto &it : this->manufacturer_datas_) {
      auto res = ESPBLEiBeacon::from_manufacturer_data(it);
      if (res.has_value())
        return *res;
    }
    return {};
  }

 protected:
  void parse_adv_(const esp_ble_gap_cb_param_t::ble_scan_result_evt_param &param);

  esp_bd_addr_t address_{
      0,
  };
  esp_ble_addr_type_t address_type_{BLE_ADDR_TYPE_PUBLIC};
  int rssi_{0};
  std::string name_{};
  std::vector<int8_t> tx_powers_{};
  optional<uint16_t> appearance_{};
  optional<uint8_t> ad_flag_{};
  std::vector<ESPBTUUID> service_uuids_;
  std::vector<ServiceData> manufacturer_datas_{};
  std::vector<ServiceData> service_datas_{};
};

class ESP32BLETracker;

class ESPBTDeviceListener {
 public:
  virtual void on_scan_end() {}
  virtual bool parse_device(const ESPBTDevice &device) = 0;
  void set_parent(ESP32BLETracker *parent) { parent_ = parent; }

 protected:
  ESP32BLETracker *parent_{nullptr};
};

struct uuid_hash {
  std::size_t operator() (esp_bt_uuid_t const& uuid) const {
    return std::hash<uint32_t>{}(uuid.uuid.uuid32);
  }

  bool operator() (esp_bt_uuid_t const& uuid_a, esp_bt_uuid_t const& uuid_b) const {
    return uuid_a.uuid.uuid32 == uuid_b.uuid.uuid32;
  }
};

struct ESPBTClient {
  uint16_t gattc_if = ESP_GATT_IF_NONE;

  using notify_cb = std::function<void()>;
  using characteristic_map = std::unordered_map<esp_bt_uuid_t, notify_cb, uuid_hash, uuid_hash>;
  using service_map = std::unordered_map<esp_bt_uuid_t, characteristic_map, uuid_hash, uuid_hash>;
  
  service_map subscribed;
  std::unordered_map<uint16_t, notify_cb> callbacks;

  service_map::iterator processed_service_it;
  characteristic_map::iterator processed_characteristic_it;
  
  bool init() {
    esp_bt_uuid_t remote_filter_service_uuid = {
      .len = ESP_UUID_LEN_128,
      .uuid = {.uuid128 = {0x42, 0x00, 0x74, 0xa9, 0xff, 0x52, 0x10, 0x9b, 0x33, 0x49, 0x35, 0x9b, 0x00, 0x02, 0x68, 0xef},},
    };

    esp_bt_uuid_t remote_filter_char_uuid = {
      .len = ESP_UUID_LEN_128,
      .uuid = {.uuid128 = {0x42, 0x00, 0x74, 0xa9, 0xff, 0x52, 0x10, 0x9b, 0x33, 0x49, 0x35, 0x9b, 0x01, 0x02, 0x68, 0xef},},
    };

    esp_bt_uuid_t remote_filter_char_uuid_2 = {
      .len = ESP_UUID_LEN_128,
      .uuid = {.uuid128 = {0x42, 0x00, 0x74, 0xa9, 0xff, 0x52, 0x10, 0x9b, 0x33, 0x49, 0x35, 0x9b, 0x02, 0x02, 0x68, 0xef},},
    };
    
    subscribe(remote_filter_service_uuid, remote_filter_char_uuid, []() {ESP_LOGI("TEST", "tep");});
//    subscribe(remote_filter_service_uuid, remote_filter_char_uuid_2, []() {ESP_LOGI("TEST", "tlak");});
    
    if (subscribed.size() == 0) return false;
    processed_service_it = subscribed.begin();
    processed_characteristic_it = processed_service_it->second.begin();
    return true;
  }

  const esp_bt_uuid_t& current_service() { return processed_service_it->first; }
  const esp_bt_uuid_t& current_characteristic() { return processed_characteristic_it->first; } 
  const notify_cb& current_callback() { return processed_characteristic_it->second; }

  bool next_service() {
    ESP_LOGI("TEST", __FUNCTION__);
    processed_service_it++;
    return processed_service_it != subscribed.end();
  }
  
  bool next_characteristic() {
    ESP_LOGI("TEST", __FUNCTION__);
    processed_characteristic_it++;
    return processed_characteristic_it != processed_service_it->second.end();
  }

  void subscribe(const esp_bt_uuid_t& service_uuid, const esp_bt_uuid_t& characteristic_uuid, const notify_cb& cb) {
    if (subscribed.count(service_uuid) == 0)
      subscribed.emplace(service_uuid, characteristic_map()); //TODO
    
    auto& characteristics = subscribed[service_uuid];
    if (characteristics.count(characteristic_uuid) == 0)
      characteristics.emplace(characteristic_uuid, cb);
  }

  bool connected  = false;
  bool get_server = false;
  esp_gattc_char_elem_t *char_elem_result   = NULL;
  esp_gattc_descr_elem_t *descr_elem_result = NULL;

  uint16_t g_conn_id;
  uint16_t g_service_start_handle;
  uint16_t g_service_end_handle;
  uint16_t g_char_handle;
  esp_bd_addr_t g_remote_bda;

  /*TODOconst*/ esp_bd_addr_t server_address = {0xC4, 0xF3, 0xE8, 0xDA, 0xBF, 0x5A};
  ESP32BLETracker *parent_{nullptr};
  void set_parent(ESP32BLETracker *parent) { parent_ = parent; }

  const esp_bd_addr_t& get_server_address() { return server_address; }
  bool is_connected() { return connected; }
  void handle_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  bool open(esp_ble_gap_cb_param_t::ble_scan_result_evt_param &param);
  void reg_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void connect_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void open_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void cfg_mtu_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void search_res_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void reg(esp_bt_uuid_t char_uuid, esp_gatt_if_t gattc_if, uint16_t conn_id);
  void reg_for_notify_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void notify_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void write_descr_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void srvc_chg_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void write_char_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  void disconnect_evt_handle(esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
};

class ESP32BLETracker : public Component {
 public:
  void set_scan_duration(uint32_t scan_duration) { scan_duration_ = scan_duration; }
  void set_scan_interval(uint32_t scan_interval) { scan_interval_ = scan_interval; }
  void set_scan_window(uint32_t scan_window) { scan_window_ = scan_window; }
  void set_scan_active(bool scan_active) { scan_active_ = scan_active; }

  /// Setup the FreeRTOS task and the Bluetooth stack.
  void setup() override;
  void dump_config() override;

  void loop() override;

  void register_listener(ESPBTDeviceListener *listener) {
    listener->set_parent(this);
    this->listeners_.push_back(listener);
  }

  void register_client(ESPBTClient *client) {
    ESP_LOGI("esp32_ble_tracker", "register_client");
    client->set_parent(this);
    int id = this->clients_.size();
    esp_ble_gattc_app_register(id);
    this->clients_.push_back(client);
  }

  void print_bt_device_info(const ESPBTDevice &device);
  /// Start a single scan by setting up the parameters and doing some esp-idf calls.
  void pause_scan() { esp_ble_gap_stop_scanning(); };
  void resume_scan() { xSemaphoreGive(this->scan_end_lock_); };
 protected:
  /// The FreeRTOS task managing the bluetooth interface.
  static bool ble_setup();
  /// Start a single scan by setting up the parameters and doing some esp-idf calls.
  void start_scan(bool first);
  /// Callback that will handle all GAP events and redistribute them to other callbacks.
  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  /// Callback that will handle all GATTC events and redistribute them to other callbacks.
  static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
  /// Called when a `ESP_GAP_BLE_SCAN_RESULT_EVT` event is received.
  void gap_scan_result(const esp_ble_gap_cb_param_t::ble_scan_result_evt_param &param);
  /// Called when a `ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT` event is received.
  void gap_scan_set_param_complete(const esp_ble_gap_cb_param_t::ble_scan_param_cmpl_evt_param &param);
  /// Called when a `ESP_GAP_BLE_SCAN_START_COMPLETE_EVT` event is received.
  void gap_scan_start_complete(const esp_ble_gap_cb_param_t::ble_scan_start_cmpl_evt_param &param);

  /// Vector of addresses that have already been printed in print_bt_device_info
  std::vector<uint64_t> already_discovered_;
  std::vector<ESPBTDeviceListener *> listeners_;
  std::vector<ESPBTClient *> clients_;
  /// A structure holding the ESP BLE scan parameters.
  esp_ble_scan_params_t scan_params_;
  /// The interval in seconds to perform scans.
  uint32_t scan_duration_;
  uint32_t scan_interval_;
  uint32_t scan_window_;
  bool scan_active_;
  SemaphoreHandle_t scan_result_lock_;
  SemaphoreHandle_t scan_end_lock_;
  size_t scan_result_index_{0};
  esp_ble_gap_cb_param_t::ble_scan_result_evt_param scan_result_buffer_[16];
  esp_bt_status_t scan_start_failed_{ESP_BT_STATUS_SUCCESS};
  esp_bt_status_t scan_set_param_failed_{ESP_BT_STATUS_SUCCESS};
};

extern ESP32BLETracker *global_esp32_ble_tracker;

}  // namespace esp32_ble_tracker
}  // namespace esphome

#endif
