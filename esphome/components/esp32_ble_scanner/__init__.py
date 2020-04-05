import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import Condition
from esphome.const import CONF_ID
from esphome.core import coroutine


esp32_ble_scanner_ns = cg.esphome_ns.namespace('esp32_ble_scanner')
ESP32BLEScanner = esp32_ble_scanner_ns.class_('ESP32BLEScanner', cg.Component)
ESP32BLEClient = esp32_ble_scanner_ns.class_('ESP32BLEClient', cg.Component)

CONF_ESP32_BLE_ID = 'esp32_ble_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ESP32BLEScanner)
}).extend(cv.COMPONENT_SCHEMA)

ESP_BLE_DEVICE_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ESP32_BLE_ID): cv.use_id(ESP32BLEScanner),
})

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

@coroutine
def register_ble_client(var, config):
    paren = yield cg.get_variable(config[CONF_ESP32_BLE_ID])
    cg.add(paren.register_client("A", var))
