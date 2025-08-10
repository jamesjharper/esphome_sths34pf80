import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    ICON_THERMOMETER,
    ICON_MOTION_SENSOR,
    ICON_THERMOMETER,
    UNIT_CELSIUS,
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_RESTORE_VALUE,
    CONF_STEP,
)

DEPENDENCIES = ["sths34pf80"]
from .. import sths34pf80_ns, Sths34pf80Component

# Configuration constants
CONF_STHS34PF80_ID = "sths34pf80_id"

# Threshold configuration keys
CONF_PRESENCE_THRESHOLD_LSB = "presence_threshold_lsb"
CONF_PRESENCE_THRESHOLD_DEGREE = "presence_threshold_degree"
CONF_MOTION_THRESHOLD_LSB = "motion_threshold_lsb"
CONF_MOTION_THRESHOLD_DEGREE = "motion_threshold_degree"
CONF_AMBIENT_SHOCK_THRESHOLD_LSB = "ambient_shock_threshold_lsb"
CONF_AMBIENT_SHOCK_THRESHOLD_DEGREE = "ambient_shock_threshold_degree"

# Hysteresis configuration keys
CONF_PRESENCE_HYSTERESIS_LSB = "presence_hysteresis_lsb"
CONF_PRESENCE_HYSTERESIS_DEGREE = "presence_hysteresis_degree"
CONF_MOTION_HYSTERESIS_LSB = "motion_hysteresis_lsb"
CONF_MOTION_HYSTERESIS_DEGREE = "motion_hysteresis_degree"
CONF_AMBIENT_SHOCK_HYSTERESIS_LSB = "ambient_shock_hysteresis_lsb"
CONF_AMBIENT_SHOCK_HYSTERESIS_DEGREE = "ambient_shock_hysteresis_degree"

# Sensitivity configuration key
CONF_SENSITIVITY = "sensitivity"

# Number classes
PresenceThresholdLsbNumber = sths34pf80_ns.class_("PresenceThresholdLsbNumber", number.Number, cg.Component)
PresenceThresholdDegreeNumber = sths34pf80_ns.class_("PresenceThresholdDegreeNumber", number.Number, cg.Component)
MotionThresholdLsbNumber = sths34pf80_ns.class_("MotionThresholdLsbNumber", number.Number, cg.Component)
MotionThresholdDegreeNumber = sths34pf80_ns.class_("MotionThresholdDegreeNumber", number.Number, cg.Component)
AmbientShockThresholdLsbNumber = sths34pf80_ns.class_("AmbientShockThresholdLsbNumber", number.Number, cg.Component)
AmbientShockThresholdDegreeNumber = sths34pf80_ns.class_("AmbientShockThresholdDegreeNumber", number.Number, cg.Component)

PresenceHysteresisLsbNumber = sths34pf80_ns.class_("PresenceHysteresisLsbNumber", number.Number, cg.Component)
PresenceHysteresisDegreeNumber = sths34pf80_ns.class_("PresenceHysteresisDegreeNumber", number.Number, cg.Component)
MotionHysteresisLsbNumber = sths34pf80_ns.class_("MotionHysteresisLsbNumber", number.Number, cg.Component)
MotionHysteresisDegreeNumber = sths34pf80_ns.class_("MotionHysteresisDegreeNumber", number.Number, cg.Component)
AmbientShockHysteresisLsbNumber = sths34pf80_ns.class_("AmbientShockHysteresisLsbNumber", number.Number, cg.Component)
AmbientShockHysteresisDegreeNumber = sths34pf80_ns.class_("AmbientShockHysteresisDegreeNumber", number.Number, cg.Component)

SensitivityNumber = sths34pf80_ns.class_("SensitivityNumber", number.Number, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),
    
    # Threshold Numbers - LSB values
    cv.Optional(CONF_PRESENCE_THRESHOLD_LSB): number.number_schema(
        PresenceThresholdLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
    ).extend({
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=65535): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=200): cv.float_,  # 200 lsb / 0.0992°C
    }),
    
    cv.Optional(CONF_MOTION_THRESHOLD_LSB): number.number_schema(
        MotionThresholdLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon=ICON_MOTION_SENSOR,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=255): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=200): cv.float_,  # 200 lsb / 0.0992°C
    }),
    
    cv.Optional(CONF_AMBIENT_SHOCK_THRESHOLD_LSB): number.number_schema(
        AmbientShockThresholdLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=65535): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=10): cv.float_,  # 10 lsb / 0.1000°C
    }),
    
    # Threshold Numbers - Degree values
    cv.Optional(CONF_PRESENCE_THRESHOLD_DEGREE): number.number_schema(
        PresenceThresholdDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-10.0): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=10.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.0992): cv.float_,  # 200 lsb / 0.0992°C
    }),
    
    cv.Optional(CONF_MOTION_THRESHOLD_DEGREE): number.number_schema(
        MotionThresholdDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_MOTION_SENSOR,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-5.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=5.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.0992): cv.float_,  # 200 lsb / 0.0992°C
    }),
    
    cv.Optional(CONF_AMBIENT_SHOCK_THRESHOLD_DEGREE): number.number_schema(
        AmbientShockThresholdDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-50.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=50.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.1000): cv.float_,  # 10 lsb / 0.1000°C
    }),
    
    # Hysteresis Numbers - LSB values
    cv.Optional(CONF_PRESENCE_HYSTERESIS_LSB): number.number_schema(
        PresenceHysteresisLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=255): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=50): cv.float_,  # 50 lsb / 0.0248°C
    }),
    
    cv.Optional(CONF_MOTION_HYSTERESIS_LSB): number.number_schema(
        MotionHysteresisLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_MOTION_SENSOR,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=255): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=50): cv.float_,  # 50 lsb / 0.0248°C
    }),
    
    cv.Optional(CONF_AMBIENT_SHOCK_HYSTERESIS_LSB): number.number_schema(
        AmbientShockHysteresisLsbNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=255): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=2): cv.float_,  # 2 lsb / 0.0200°C
    }),
    
    # Hysteresis Numbers - Degree values
    cv.Optional(CONF_PRESENCE_HYSTERESIS_DEGREE): number.number_schema(
        PresenceHysteresisDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-5.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=5.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.0001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.0248): cv.float_,  # 50 lsb / 0.0248°C
    }),
    
    cv.Optional(CONF_MOTION_HYSTERESIS_DEGREE): number.number_schema(
        MotionHysteresisDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_MOTION_SENSOR,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-5.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=5.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.0001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.0248): cv.float_,  # 50 lsb / 0.0248°C
    }),
    
    cv.Optional(CONF_AMBIENT_SHOCK_HYSTERESIS_DEGREE): number.number_schema(
        AmbientShockHysteresisDegreeNumber,
        device_class=DEVICE_CLASS_TEMPERATURE,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
        unit_of_measurement=UNIT_CELSIUS,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=-50.0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=50.0): cv.float_,
        cv.Optional(CONF_STEP, default=0.001): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=0.0200): cv.float_,  # 2 lsb / 0.0200°C
    }),
    
    # Sensitivity Number
    cv.Optional(CONF_SENSITIVITY): number.number_schema(
        SensitivityNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_THERMOMETER,
    ).extend({
        cv.Optional(CONF_MIN_VALUE, default=1): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=65535): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_VALUE, default=2016): cv.float_,  # Default sensitivity is 2016 LSB/°C
    }),
})

async def to_number_code(config, parented):
    var = await number.new_number(
        config, 
        min_value=config[CONF_MIN_VALUE], 
        max_value=config[CONF_MAX_VALUE], 
        step=config[CONF_STEP])

    if initial_value := config.get(CONF_INITIAL_VALUE):
        cg.add(var.set_initial_value(initial_value))

    if restore_value := config.get(CONF_RESTORE_VALUE):
        cg.add(var.set_restore_value(restore_value))
      
    await cg.register_component(var, config)
    await cg.register_parented(var, parented)

async def to_code(config):
    if number_config := config.get(CONF_PRESENCE_THRESHOLD_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_PRESENCE_THRESHOLD_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_MOTION_THRESHOLD_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_MOTION_THRESHOLD_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_AMBIENT_SHOCK_THRESHOLD_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_AMBIENT_SHOCK_THRESHOLD_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_PRESENCE_HYSTERESIS_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])

    if number_config := config.get(CONF_PRESENCE_HYSTERESIS_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])
   
    if number_config := config.get(CONF_MOTION_HYSTERESIS_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])  
   
    if number_config := config.get(CONF_MOTION_HYSTERESIS_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])  

    if number_config := config.get(CONF_AMBIENT_SHOCK_HYSTERESIS_LSB):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])  

    if number_config := config.get(CONF_AMBIENT_SHOCK_HYSTERESIS_DEGREE):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])  

    if number_config := config.get(CONF_SENSITIVITY):
        await to_number_code(number_config, config[CONF_STHS34PF80_ID])  