import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_OCCUPANCY,
)

from .. import  CONF_STHS34PF80_ID, Sths34pf80Component

DEPENDENCIES = ['sths34pf80']

# Configuration constants

CONF_PRESENCE_BINARY = 'presence'
CONF_MOTION = 'motion'
CONF_THERMAL_SHOCK = 'thermal_shock'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),
    
    cv.Optional(CONF_PRESENCE_BINARY): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_OCCUPANCY,
    ),
    
    cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_MOTION,
    ),
    
    cv.Optional(CONF_THERMAL_SHOCK): binary_sensor.binary_sensor_schema(),
})

async def to_code(config):
    parent_var = await cg.get_variable(config[CONF_STHS34PF80_ID])
    
    if binary_sensor_config := config.get(CONF_PRESENCE_BINARY):
        bin_sens = await binary_sensor.new_binary_sensor(binary_sensor_config)
        cg.add(parent_var.set_presence_detected_binary_sensor(bin_sens))

    if binary_sensor_config := config.get(CONF_MOTION):
        bin_sens = await binary_sensor.new_binary_sensor(binary_sensor_config)
        cg.add(parent_var.set_movement_detected_binary_sensor(bin_sens))

    if binary_sensor_config := config.get(CONF_THERMAL_SHOCK):
        bin_sens = await binary_sensor.new_binary_sensor(binary_sensor_config)
        cg.add(parent_var.set_thermal_shock_detected_binary_sensor(bin_sens))