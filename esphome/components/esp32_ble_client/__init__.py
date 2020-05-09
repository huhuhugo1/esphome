import esphome.codegen as cg
import esphome.config_validation as cv

esp32_ble_client_ns = cg.esphome_ns.namespace('esp32_ble_client')
ESP32BLEClient = esp32_ble_client_ns.class_('ESP32BLEClient')

CONFIG_SCHEMA = cv.Schema({})
