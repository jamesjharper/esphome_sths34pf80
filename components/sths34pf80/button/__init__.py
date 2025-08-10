import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_RESTART,
    DEVICE_CLASS_UPDATE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_RESTART,
)

DEPENDENCIES = ["sths34pf80"]

from .. import sths34pf80_ns, CONF_STHS34PF80_ID, Sths34pf80Component

# Button classes
StartSamplingButton = sths34pf80_ns.class_("StartSamplingButton", button.Button)
StopSamplingButton = sths34pf80_ns.class_("StopSamplingButton", button.Button)

# Configuration keys
CONF_START_SAMPLING = "start_sampling"
CONF_STOP_SAMPLING = "stop_sampling"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),

    cv.Optional(CONF_START_SAMPLING): button.button_schema(
        StartSamplingButton,
        device_class=DEVICE_CLASS_UPDATE,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:play",
    ),

    cv.Optional(CONF_STOP_SAMPLING): button.button_schema(
        StopSamplingButton,
        device_class=DEVICE_CLASS_RESTART,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=ICON_RESTART,
    ),
})


async def to_sths34pf80_button_code(button_config, parent_var):
    button_var = await button.new_button(button_config)
    await cg.register_parented(button_var, parent_var)


async def to_code(config):
    parent_var = await cg.get_variable(config[CONF_STHS34PF80_ID])

    if button_config := config.get(CONF_START_SAMPLING):
        await to_sths34pf80_button_code(button_config, parent_var)

    if button_config := config.get(CONF_STOP_SAMPLING):
        await to_sths34pf80_button_code(button_config, parent_var)
