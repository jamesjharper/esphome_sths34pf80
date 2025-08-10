import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_RESTORE_VALUE,
    DEVICE_CLASS_EMPTY,
    ENTITY_CATEGORY_DIAGNOSTIC
)

from .. import sths34pf80_ns, CONF_STHS34PF80_ID, Sths34pf80Component

DEPENDENCIES = ["sths34pf80"]

EnableCalibrationSwitch = sths34pf80_ns.class_("EnableCalibrationSwitch", switch.Switch, cg.Component)

CONF_ENABLE_CALIBRATION = "enable_calibration"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),
    
    cv.Optional(CONF_ENABLE_CALIBRATION): switch.switch_schema(
        EnableCalibrationSwitch,
        device_class=DEVICE_CLASS_EMPTY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:tune",
    )
    .extend(
        {
            cv.Optional(CONF_RESTORE_VALUE, default=False): cv.boolean,
            cv.Optional(CONF_INITIAL_VALUE, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
})

async def to_sths34pf80_code(switch_config, parent_var):
    switch_var = await switch.new_switch(switch_config)

    if initial_value := switch_config.get(CONF_INITIAL_VALUE):
        cg.add(switch_var.set_initial_value(initial_value))

    if restore_value := switch_config.get(CONF_RESTORE_VALUE):
        cg.add(switch_var.set_restore_value(restore_value))

    await cg.register_component(switch_var, switch_config)
    await cg.register_parented(switch_var, parent_var)

async def to_code(config):
    parent_var = await cg.get_variable(config[CONF_STHS34PF80_ID])
    
    if switch_config := config.get(CONF_ENABLE_CALIBRATION):
        await to_sths34pf80_code(switch_config, parent_var)