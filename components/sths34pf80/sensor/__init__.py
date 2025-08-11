import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    UNIT_CELSIUS, 
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_THERMOMETER
)

from .. import CONF_STHS34PF80_ID, sths34pf80_ns, Sths34pf80Component

DEPENDENCIES = ['sths34pf80']

# Define sensor classes
# Absolute temperature sensors (always enabled)
AmbientTemperatureSensor = sths34pf80_ns.class_("AmbientTemperatureSensor", sensor.Sensor, cg.PollingComponent)
ObjectTemperatureSensor = sths34pf80_ns.class_("ObjectTemperatureSensor", sensor.Sensor, cg.PollingComponent)

# Temperature delta sensors (in degrees Celsius)
ObjectTemperatureDeltaSensor = sths34pf80_ns.class_("ObjectTemperatureDeltaSensor", sensor.Sensor, cg.PollingComponent)
PresenceTemperatureDeltaSensor = sths34pf80_ns.class_("PresenceTemperatureDeltaSensor", sensor.Sensor, cg.PollingComponent)
MotionTemperatureDeltaSensor = sths34pf80_ns.class_("MotionTemperatureDeltaSensor", sensor.Sensor, cg.PollingComponent)
ThermalShockTemperatureDeltaSensor = sths34pf80_ns.class_("ThermalShockTemperatureDeltaSensor", sensor.Sensor, cg.PollingComponent)

# Raw LSB sensors
PresenceRawLsbSensor = sths34pf80_ns.class_("PresenceRawLsbSensor", sensor.Sensor, cg.PollingComponent)
MotionRawLsbSensor = sths34pf80_ns.class_("MotionRawLsbSensor", sensor.Sensor, cg.PollingComponent)
ThermalShockRawLsbSensor = sths34pf80_ns.class_("ThermalShockRawLsbSensor", sensor.Sensor, cg.PollingComponent)

# Configuration constants
CONF_AMBIENT_TEMPERATURE = 'ambient_temperature'
CONF_OBJECT_TEMPERATURE = 'object_temperature'
CONF_OBJECT_TEMPERATURE_DELTA = 'object_temperature_delta'

# Temperature delta sensors (in degrees Celsius)
CONF_PRESENCE_TEMPERATURE_DELTA = 'presence_temperature_delta'
CONF_MOTION_TEMPERATURE_DELTA = 'motion_temperature_delta'
CONF_THERMAL_SHOCK_TEMPERATURE_DELTA = 'thermal_shock_temperature_delta'

# Raw LSB sensors
CONF_PRESENCE_RAW_LSB = 'presence_raw_lsb'
CONF_MOTION_RAW_LSB = 'motion_raw_lsb'
CONF_THERMAL_SHOCK_RAW_LSB = 'thermal_shock_raw_lsb'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),
    
    cv.Optional(CONF_AMBIENT_TEMPERATURE): sensor.sensor_schema(
        AmbientTemperatureSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=2,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_OBJECT_TEMPERATURE): sensor.sensor_schema(
        ObjectTemperatureSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=2,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_OBJECT_TEMPERATURE_DELTA): sensor.sensor_schema(
        ObjectTemperatureDeltaSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=2,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    # Temperature delta sensors (in degrees Celsius)
    cv.Optional(CONF_PRESENCE_TEMPERATURE_DELTA): sensor.sensor_schema(
        PresenceTemperatureDeltaSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=3,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_MOTION_TEMPERATURE_DELTA): sensor.sensor_schema(
        MotionTemperatureDeltaSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=3,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_THERMAL_SHOCK_TEMPERATURE_DELTA): sensor.sensor_schema(
        ThermalShockTemperatureDeltaSensor,
        unit_of_measurement=UNIT_CELSIUS,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=3,
        icon=ICON_THERMOMETER,
    ).extend(cv.polling_component_schema("1s")),
    
    # Raw LSB sensors
    cv.Optional(CONF_PRESENCE_RAW_LSB): sensor.sensor_schema(
        PresenceRawLsbSensor,
        unit_of_measurement="LSB",
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=0,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_MOTION_RAW_LSB): sensor.sensor_schema(
        MotionRawLsbSensor,
        unit_of_measurement="LSB",
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=0,
    ).extend(cv.polling_component_schema("1s")),
    
    cv.Optional(CONF_THERMAL_SHOCK_RAW_LSB): sensor.sensor_schema(
        ThermalShockRawLsbSensor,
        unit_of_measurement="LSB",
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=0,
    ).extend(cv.polling_component_schema("1s")),
})

async def to_code(config):
    parent_var = await cg.get_variable(config[CONF_STHS34PF80_ID])
    
    if sensor_config := config.get(CONF_AMBIENT_TEMPERATURE):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_OBJECT_TEMPERATURE):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    # Temperature delta sensors
    if sensor_config := config.get(CONF_OBJECT_TEMPERATURE_DELTA):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_PRESENCE_TEMPERATURE_DELTA):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_MOTION_TEMPERATURE_DELTA):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_THERMAL_SHOCK_TEMPERATURE_DELTA):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    # Raw LSB sensors
    if sensor_config := config.get(CONF_PRESENCE_RAW_LSB):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_MOTION_RAW_LSB):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)

    if sensor_config := config.get(CONF_THERMAL_SHOCK_RAW_LSB):
        sensor_var = await sensor.new_sensor(sensor_config)
        await cg.register_component(sensor_var, sensor_config)
        await cg.register_parented(sensor_var, parent_var)