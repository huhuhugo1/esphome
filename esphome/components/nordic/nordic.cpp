#include "nordic.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace nordic {

static const char *TAG = "nordic";

void Nordic::dump_config() {
  ESP_LOGCONFIG(TAG, "Nordic");
}

}  // namespace nordic
}  // namespace esphome

#endif
