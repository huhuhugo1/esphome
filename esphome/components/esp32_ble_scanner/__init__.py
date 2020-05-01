import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import Condition
from esphome.const import CONF_ID, CONF_PERIOD, CONF_INTERVAL
from esphome.core import coroutine

CONFLICTS_WITH = ['esp32_ble_tracker', 'esp32_ble_beacon']

esp32_ble_scanner_ns = cg.esphome_ns.namespace('esp32_ble_scanner')
ESP32BLEScanner = esp32_ble_scanner_ns.class_('ESP32BLEScanner', cg.Component)
ESP32BLEClient = esp32_ble_scanner_ns.class_('ESP32BLEClient', cg.Component)

CONF_ESP32_BLE_ID = 'esp32_ble_id'
CONF_SCAN_PARAMETERS = 'scan_parameters'
CONF_WINDOW = 'window'
CONF_ACTIVE = 'active'

def validate_scan_parameters(config):
    window = config[CONF_WINDOW]
    interval = config[CONF_INTERVAL]
    period = config[CONF_PERIOD]

    if window > interval:
        raise cv.Invalid("Scan window ({}) needs to be smaller than scan interval ({})"
                         "".format(window, interval))
    if interval > period:
        raise cv.Invalid("Scan interval ({}) needs to be smaller than scan period ({})"
                         "".format(interval, period))
    return config

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ESP32BLEScanner),
    cv.Optional(CONF_SCAN_PARAMETERS, default={}): cv.All(cv.Schema({
        cv.Optional(CONF_WINDOW, default='30ms'): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_INTERVAL, default='320ms'): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PERIOD, default='5min'): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ACTIVE, default=True): cv.boolean
    }), validate_scan_parameters),
}).extend(cv.COMPONENT_SCHEMA)

ESP_BLE_DEVICE_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ESP32_BLE_ID): cv.use_id(ESP32BLEScanner),
})

def to_code(config):
    params = config[CONF_SCAN_PARAMETERS]
    var = cg.new_Pvariable(config[CONF_ID], params[CONF_WINDOW], params[CONF_INTERVAL],
                           params[CONF_PERIOD], params[CONF_ACTIVE])
    yield cg.register_component(var, config)

@coroutine
def register_ble_client(var, config):
    paren = yield cg.get_variable(config[CONF_ESP32_BLE_ID])
    cg.add(paren.register_client(var))
