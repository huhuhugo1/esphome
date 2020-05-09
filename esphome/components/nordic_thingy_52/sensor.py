import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, esp32_ble_client
from esphome.const import CONF_ID, CONF_MAC_ADDRESS, CONF_HUMIDITY, CONF_TEMPERATURE, CONF_PRESSURE, \
    UNIT_CELSIUS, ICON_THERMOMETER, UNIT_HECTOPASCAL, ICON_GAUGE, UNIT_PERCENT, ICON_WATER_PERCENT

DEPENDENCIES = ['esp32_ble_client']

nordic_thingy_52_ns = cg.esphome_ns.namespace('nordic_thingy_52')
NordicThingy52 = nordic_thingy_52_ns.class_('NordicThingy52', esp32_ble_client.ESP32BLEClient, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NordicThingy52),
    cv.Required(CONF_MAC_ADDRESS): cv.string,
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(UNIT_CELSIUS, ICON_THERMOMETER, 1),
    cv.Optional(CONF_PRESSURE): sensor.sensor_schema(UNIT_HECTOPASCAL, ICON_GAUGE, 1),
    cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(UNIT_PERCENT, ICON_WATER_PERCENT, 1),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_MAC_ADDRESS])

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature(sens))
    if CONF_PRESSURE in config:
        sens = yield sensor.new_sensor(config[CONF_PRESSURE])
        cg.add(var.set_pressure(sens))
    if CONF_HUMIDITY in config:
        sens = yield sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity(sens))

    yield cg.register_component(var, config)
