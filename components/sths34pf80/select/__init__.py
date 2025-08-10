import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_RESTORE_VALUE,
    ENTITY_CATEGORY_CONFIG
)

from .. import sths34pf80_ns, CONF_STHS34PF80_ID, Sths34pf80Component

DEPENDENCIES = ["sths34pf80"]

# Define select classes
AverageObjectTemperatureNumberSelect = sths34pf80_ns.class_("AverageObjectTemperatureNumberSelect", select.Select, cg.Component)
AverageAmbientTemperatureNumberSelect = sths34pf80_ns.class_("AverageAmbientTemperatureNumberSelect", select.Select, cg.Component)
GainModeSelect = sths34pf80_ns.class_("GainModeSelect", select.Select, cg.Component)
SampleRateSelect = sths34pf80_ns.class_("SampleRateSelect", select.Select, cg.Component)
LowPassFilterPresenceBandwidthSelect = sths34pf80_ns.class_("LowPassFilterPresenceBandwidthSelect", select.Select, cg.Component)
LowPassFilterMotionBandwidthSelect = sths34pf80_ns.class_("LowPassFilterMotionBandwidthSelect", select.Select, cg.Component)
LowPassFilterAmbientTemperatureBandwidthSelect = sths34pf80_ns.class_("LowPassFilterAmbientTemperatureBandwidthSelect", select.Select, cg.Component)

# Configuration constants
CONF_AVERAGE_OBJECT_TEMPERATURE_NUMBER = 'average_object_temperature_number'
CONF_AVERAGE_AMBIENT_TEMPERATURE_NUMBER = 'average_ambient_temperature_number'
CONF_GAIN_MODE = 'gain_mode'
CONF_SAMPLE_RATE = 'sample_rate'
CONF_LOW_PASS_FILTER_PRESENCE_BANDWIDTH = 'low_pass_filter_presence_bandwidth'
CONF_LOW_PASS_FILTER_MOTION_BANDWIDTH = 'low_pass_filter_motion_bandwidth'
CONF_LOW_PASS_FILTER_AMBIENT_TEMPERATURE_BANDWIDTH = 'low_pass_filter_ambient_temperature_bandwidth'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_STHS34PF80_ID): cv.use_id(Sths34pf80Component),
    
    cv.Optional(CONF_AVERAGE_OBJECT_TEMPERATURE_NUMBER): select.select_schema(
        AverageObjectTemperatureNumberSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:calculator",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="128"): cv.string,
    })),
    
    cv.Optional(CONF_AVERAGE_AMBIENT_TEMPERATURE_NUMBER): select.select_schema(
        AverageAmbientTemperatureNumberSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:calculator",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="8"): cv.string,
    })),
    
    cv.Optional(CONF_GAIN_MODE): select.select_schema(
        GainModeSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:amplifier",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="DEFAULT"): cv.string,
    })),
    
    cv.Optional(CONF_SAMPLE_RATE): select.select_schema(
        SampleRateSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:clock-fast",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="1HZ"): cv.string,
    })),
    
    cv.Optional(CONF_LOW_PASS_FILTER_PRESENCE_BANDWIDTH): select.select_schema(
        LowPassFilterPresenceBandwidthSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:filter",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="DIV_200"): cv.string,
    })),
    
    cv.Optional(CONF_LOW_PASS_FILTER_MOTION_BANDWIDTH): select.select_schema(
        LowPassFilterMotionBandwidthSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:filter",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="DIV_200"): cv.string,
    })),
    
    cv.Optional(CONF_LOW_PASS_FILTER_AMBIENT_TEMPERATURE_BANDWIDTH): select.select_schema(
        LowPassFilterAmbientTemperatureBandwidthSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:filter",
    ).extend(cv.Schema({
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_INITIAL_VALUE, default="DIV_50"): cv.string,
    })),
})

async def to_sths34pf80_code(select_config, parent_var, options, select_class):
    select_var = await select.new_select(select_config, options=options)

    if initial_value := select_config.get(CONF_INITIAL_VALUE):
        cg.add(select_var.set_initial_value(initial_value))

    if restore_value := select_config.get(CONF_RESTORE_VALUE):
        cg.add(select_var.set_restore_value(restore_value))

    await cg.register_component(select_var, select_config)
    await cg.register_parented(select_var, parent_var)

async def to_code(config):
    parent_var = await cg.get_variable(config[CONF_STHS34PF80_ID])
    
    if select_config := config.get(CONF_AVERAGE_OBJECT_TEMPERATURE_NUMBER):
        await to_sths34pf80_code(select_config, parent_var, 
            ["2", "8", "32", "128", "256", "512", "1024", "2048"], AverageObjectTemperatureNumberSelect)

    if select_config := config.get(CONF_AVERAGE_AMBIENT_TEMPERATURE_NUMBER):
        await to_sths34pf80_code(select_config, parent_var, 
            ["1", "2", "4", "8"], AverageAmbientTemperatureNumberSelect)

    if select_config := config.get(CONF_GAIN_MODE):
        await to_sths34pf80_code(select_config, parent_var, 
            ["DEFAULT", "WIDE"], GainModeSelect)

    if select_config := config.get(CONF_SAMPLE_RATE):
        await to_sths34pf80_code(select_config, parent_var, 
            ["OFF", "0.25HZ", "0.5HZ", "1HZ", "2HZ", "4HZ", "8HZ", "15HZ", "30HZ"], SampleRateSelect)

    if select_config := config.get(CONF_LOW_PASS_FILTER_PRESENCE_BANDWIDTH):
        await to_sths34pf80_code(select_config, parent_var, 
            ["DIV_9", "DIV_20", "DIV_50", "DIV_100", "DIV_200", "DIV_400", "DIV_800"], LowPassFilterPresenceBandwidthSelect)

    if select_config := config.get(CONF_LOW_PASS_FILTER_MOTION_BANDWIDTH):
        await to_sths34pf80_code(select_config, parent_var, 
            ["DIV_9", "DIV_20", "DIV_50", "DIV_100", "DIV_200", "DIV_400", "DIV_800"], LowPassFilterMotionBandwidthSelect)

    if select_config := config.get(CONF_LOW_PASS_FILTER_AMBIENT_TEMPERATURE_BANDWIDTH):
        await to_sths34pf80_code(select_config, parent_var, 
            ["DIV_9", "DIV_20", "DIV_50", "DIV_100", "DIV_200", "DIV_400", "DIV_800"], LowPassFilterAmbientTemperatureBandwidthSelect)