import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, esp32_ble_scanner
from esphome.const import CONF_BATTERY_LEVEL, CONF_HUMIDITY, CONF_TEMPERATURE, CONF_PRESSURE, \
    UNIT_CELSIUS, UNIT_HECTOPASCAL, ICON_THERMOMETER, UNIT_PERCENT, ICON_WATER_PERCENT, ICON_GAUGE, ICON_BATTERY, CONF_ID, \
    CONF_BLE_SERVER_NAME

DEPENDENCIES = ['esp32_ble_scanner']

nordic_ns = cg.esphome_ns.namespace('nordic')
Nordic = nordic_ns.class_('Nordic', esp32_ble_scanner.ESP32BLEClient, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Nordic),
    cv.Required(CONF_BLE_SERVER_NAME): cv.string,
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(UNIT_CELSIUS, ICON_THERMOMETER, 1),
    cv.Optional(CONF_PRESSURE): sensor.sensor_schema(UNIT_HECTOPASCAL, ICON_GAUGE, 1),
    cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(UNIT_PERCENT, ICON_WATER_PERCENT, 1),
    cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(UNIT_PERCENT, ICON_BATTERY, 0),
}).extend(cv.COMPONENT_SCHEMA).extend(esp32_ble_scanner.ESP_BLE_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_BLE_SERVER_NAME])

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature(sens))
    if CONF_PRESSURE in config:
        sens = yield sensor.new_sensor(config[CONF_PRESSURE])
        cg.add(var.set_pressure(sens))
    if CONF_HUMIDITY in config:
        sens = yield sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity(sens))
    if CONF_BATTERY_LEVEL in config:
        sens = yield sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery_level(sens))

    yield cg.register_component(var, config)
    yield esp32_ble_scanner.register_ble_client(var, config)
