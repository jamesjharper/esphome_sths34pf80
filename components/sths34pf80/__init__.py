import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, core
from esphome.components import i2c
from esphome.const import CONF_ID

MULTI_CONF = True
DEPENDENCIES = ['i2c']

sths34pf80_ns = cg.esphome_ns.namespace('sths34pf80')
Sths34pf80Component = sths34pf80_ns.class_('Sths34pf80Component', cg.Component, i2c.I2CDevice)


CONF_STHS34PF80_ID = "sths34pf80_id"

# Configuration constants
CONF_INTERRUPT_PIN = 'interrupt_pin'
CONF_RECALIBRATION_INTERVAL = 'recalibration_interval'

# Device settings - Averaging
CONF_AVG_T_OBJECT_NUMBER = 'avg_t_object_number'
CONF_AVG_T_AMBIENT_NUMBER = 'avg_t_ambient_number'

# Device settings - Sensor config
CONF_GAIN_MODE = 'gain_mode'
CONF_TMOS_SENSITIVITY = 'tmos_sensitivity'
CONF_TMOS_ODR = 'tmos_odr'

# Device settings - Presence detection
CONF_PRESENCE_THRESHOLD_LSB = 'presence_threshold_lsb'
CONF_PRESENCE_THRESHOLD_DEGREES = 'presence_threshold_degrees'
CONF_PRESENCE_HYSTERESIS_LSB = 'presence_hysteresis_lsb'
CONF_PRESENCE_HYSTERESIS_DEGREES = 'presence_hysteresis_degrees'
CONF_LPF_PRESENCE_BANDWIDTH = 'lpf_presence_bandwidth'
CONF_PRESENCE_ABS_VALUE = 'presence_abs_value'

# Device settings - Motion detection
CONF_MOTION_THRESHOLD_LSB = 'motion_threshold_lsb'
CONF_MOTION_THRESHOLD_DEGREES = 'motion_threshold_degrees'
CONF_MOTION_HYSTERESIS_LSB = 'motion_hysteresis_lsb'
CONF_MOTION_HYSTERESIS_DEGREES = 'motion_hysteresis_degrees'
CONF_LPF_MOTION_BANDWIDTH = 'lpf_motion_bandwidth'
CONF_LPF_PRESENCE_MOTION_BANDWIDTH = 'lpf_presence_motion_bandwidth'

# Device settings - Ambient shock detection
CONF_T_AMBIENT_SHOCK_THRESHOLD_LSB = 't_ambient_shock_threshold_lsb'
CONF_T_AMBIENT_SHOCK_THRESHOLD_DEGREES = 't_ambient_shock_threshold_degrees'
CONF_T_AMBIENT_SHOCK_HYSTERESIS_LSB = 't_ambient_shock_hysteresis_lsb'
CONF_T_AMBIENT_SHOCK_HYSTERESIS_DEGREES = 't_ambient_shock_hysteresis_degrees'
CONF_LPF_AMBIENT_TEMP_BANDWIDTH = 'lpf_ambient_temp_bandwidth'

# Device settings - Interrupt configuration
CONF_TMOS_ROUTE_INTERRUPT = 'tmos_route_interrupt'
CONF_TMOS_INTERRUPT_OR = 'tmos_interrupt_or'
CONF_DATA_READY_MODE = 'data_ready_mode'
CONF_INTERRUPT_PULSED = 'interrupt_pulsed'
CONF_INTERRUPT_MODE = 'interrupt_mode'

# Device settings - Algorithm compensation
CONF_T_OBJECT_ALGO_COMPENSATION = 't_object_algo_compensation'

# Enum definitions
AvgTObjectNumber = cg.global_ns.enum('sths34pf80_avg_tobject_num_t')
AVG_T_OBJECT_NUMBERS = {
    '2': AvgTObjectNumber.STHS34PF80_AVG_TMOS_2,
    '8': AvgTObjectNumber.STHS34PF80_AVG_TMOS_8,
    '32': AvgTObjectNumber.STHS34PF80_AVG_TMOS_32,
    '128': AvgTObjectNumber.STHS34PF80_AVG_TMOS_128,
    '256': AvgTObjectNumber.STHS34PF80_AVG_TMOS_256,
    '512': AvgTObjectNumber.STHS34PF80_AVG_TMOS_512,
    '1024': AvgTObjectNumber.STHS34PF80_AVG_TMOS_1024,
    '2048': AvgTObjectNumber.STHS34PF80_AVG_TMOS_2048,
}

AvgTAmbientNumber = cg.global_ns.enum('sths34pf80_avg_tambient_num_t')
AVG_T_AMBIENT_NUMBERS = {
    '8': AvgTAmbientNumber.STHS34PF80_AVG_T_8,
    '4': AvgTAmbientNumber.STHS34PF80_AVG_T_4,
    '2': AvgTAmbientNumber.STHS34PF80_AVG_T_2,
    '1': AvgTAmbientNumber.STHS34PF80_AVG_T_1,
}

GainMode = cg.global_ns.enum('sths34pf80_gain_mode_t')
GAIN_MODES = {
    'WIDE': GainMode.STHS34PF80_GAIN_WIDE_MODE,
    'DEFAULT': GainMode.STHS34PF80_GAIN_DEFAULT_MODE,
}

TmosOdr = cg.global_ns.enum('sths34pf80_tmos_odr_t')
TMOS_ODRS = {
    'OFF': TmosOdr.STHS34PF80_TMOS_ODR_OFF,
    '0.25HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_0Hz25,
    '0.5HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_0Hz50,
    '1HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_1Hz,
    '2HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_2Hz,
    '4HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_4Hz,
    '8HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_8Hz,
    '15HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_15Hz,
    '30HZ': TmosOdr.STHS34PF80_TMOS_ODR_AT_30Hz,
}

LpfBandwidth = cg.global_ns.enum('sths34pf80_lpf_bandwidth_t')
LPF_BANDWIDTHS = {
    'DIV_9': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_9,
    'DIV_20': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_20,
    'DIV_50': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_50,
    'DIV_100': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_100,
    'DIV_200': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_200,
    'DIV_400': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_400,
    'DIV_800': LpfBandwidth.STHS34PF80_LPF_ODR_DIV_800,
}

TmosRouteInt = cg.global_ns.enum('sths34pf80_tmos_route_int_t')
TMOS_ROUTE_INTS = {
    'HIZ': TmosRouteInt.STHS34PF80_TMOS_INT_HIZ,
    'DRDY': TmosRouteInt.STHS34PF80_TMOS_INT_DRDY,
    'OR': TmosRouteInt.STHS34PF80_TMOS_INT_OR,
}

TmosIntOr = cg.global_ns.enum('sths34pf80_tmos_int_or_t')
TMOS_INT_ORS = {
    'NONE': TmosIntOr.STHS34PF80_TMOS_INT_NONE,
    'TSHOCK': TmosIntOr.STHS34PF80_TMOS_INT_TSHOCK,
    'MOTION': TmosIntOr.STHS34PF80_TMOS_INT_MOTION,
    'TSHOCK_MOTION': TmosIntOr.STHS34PF80_TMOS_INT_TSHOCK_MOTION,
    'PRESENCE': TmosIntOr.STHS34PF80_TMOS_INT_PRESENCE,
    'TSHOCK_PRESENCE': TmosIntOr.STHS34PF80_TMOS_INT_TSHOCK_PRESENCE,
    'MOTION_PRESENCE': TmosIntOr.STHS34PF80_TMOS_INT_MOTION_PRESENCE,
    'ALL': TmosIntOr.STHS34PF80_TMOS_INT_ALL,
}

DrdyMode = cg.global_ns.enum('sths34pf80_drdy_mode_t')
DRDY_MODES = {
    'PULSED': DrdyMode.STHS34PF80_DRDY_PULSED,
    'LATCHED': DrdyMode.STHS34PF80_DRDY_LATCHED,
}

PresenceAbsMode = sths34pf80_ns.enum('PresenceAbsMode')
PRESENCE_ABS_MODES = {
    'DISABLED': PresenceAbsMode.PRESENCE_ABS_DISABLED,
    'ENABLED': PresenceAbsMode.PRESENCE_ABS_ENABLED,
    'ENABLE_DELAYED': PresenceAbsMode.PRESENCE_ABS_ENABLE_DELAYED,
}

# Note: Interrupt mode is a struct, handled differently in configuration

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Sths34pf80Component),
        cv.Optional(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,

        # Device configuration - Averaging
        cv.Optional(CONF_AVG_T_OBJECT_NUMBER): cv.enum(AVG_T_OBJECT_NUMBERS, upper=True),
        cv.Optional(CONF_AVG_T_AMBIENT_NUMBER): cv.enum(AVG_T_AMBIENT_NUMBERS, upper=True),

        # Device configuration - Basic sensor settings
        cv.Optional(CONF_GAIN_MODE): cv.enum(GAIN_MODES, upper=True),
        cv.Optional(CONF_TMOS_SENSITIVITY): cv.All(cv.uint16_t, cv.Range(min=1, max=65535)),
        cv.Optional(CONF_TMOS_ODR, default="8HZ"): cv.enum(TMOS_ODRS, upper=True),

        # Automatic recalibration
        cv.Optional(CONF_RECALIBRATION_INTERVAL, default="10min"): cv.positive_time_period_milliseconds,

        # Device configuration - Presence detection (LSB and degrees versions)
        cv.Optional(CONF_PRESENCE_THRESHOLD_LSB): cv.All(cv.uint16_t, cv.Range(min=0, max=65535)),
        cv.Optional(CONF_PRESENCE_THRESHOLD_DEGREES): cv.temperature,
        cv.Optional(CONF_PRESENCE_HYSTERESIS_LSB): cv.All(cv.uint8_t, cv.Range(min=0, max=255)),
        cv.Optional(CONF_PRESENCE_HYSTERESIS_DEGREES): cv.temperature,
        cv.Optional(CONF_LPF_PRESENCE_BANDWIDTH): cv.enum(LPF_BANDWIDTHS, upper=True),
        cv.Optional(CONF_PRESENCE_ABS_VALUE, default="ENABLE_DELAYED"): cv.enum(PRESENCE_ABS_MODES, upper=True),

        # Device configuration - Motion detection (LSB and degrees versions)
        cv.Optional(CONF_MOTION_THRESHOLD_LSB): cv.All(cv.uint8_t, cv.Range(min=0, max=255)),
        cv.Optional(CONF_MOTION_THRESHOLD_DEGREES): cv.temperature,
        cv.Optional(CONF_MOTION_HYSTERESIS_LSB): cv.All(cv.uint8_t, cv.Range(min=0, max=255)),
        cv.Optional(CONF_MOTION_HYSTERESIS_DEGREES): cv.temperature,
        cv.Optional(CONF_LPF_MOTION_BANDWIDTH): cv.enum(LPF_BANDWIDTHS, upper=True),
        cv.Optional(CONF_LPF_PRESENCE_MOTION_BANDWIDTH): cv.enum(LPF_BANDWIDTHS, upper=True),

        # Device configuration - Ambient shock detection (LSB and degrees versions)
        cv.Optional(CONF_T_AMBIENT_SHOCK_THRESHOLD_LSB): cv.All(cv.uint16_t, cv.Range(min=0, max=65535)),
        cv.Optional(CONF_T_AMBIENT_SHOCK_THRESHOLD_DEGREES): cv.temperature,
        cv.Optional(CONF_T_AMBIENT_SHOCK_HYSTERESIS_LSB): cv.All(cv.uint8_t, cv.Range(min=0, max=255)),
        cv.Optional(CONF_T_AMBIENT_SHOCK_HYSTERESIS_DEGREES): cv.temperature,
        cv.Optional(CONF_LPF_AMBIENT_TEMP_BANDWIDTH): cv.enum(LPF_BANDWIDTHS, upper=True),

        # Device configuration - Interrupt settings
        cv.Optional(CONF_TMOS_ROUTE_INTERRUPT): cv.enum(TMOS_ROUTE_INTS, upper=True),
        cv.Optional(CONF_TMOS_INTERRUPT_OR): cv.enum(TMOS_INT_ORS, upper=True),
        cv.Optional(CONF_DATA_READY_MODE): cv.enum(DRDY_MODES, upper=True),
        cv.Optional(CONF_INTERRUPT_PULSED): cv.boolean,

        # Device configuration - Algorithm settings
        cv.Optional(CONF_T_OBJECT_ALGO_COMPENSATION): cv.boolean,
        
    }).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x5A)),
    
    # Validation to prevent conflicting LSB and degrees configurations
    cv.has_at_most_one_key(CONF_PRESENCE_THRESHOLD_LSB, CONF_PRESENCE_THRESHOLD_DEGREES),
    cv.has_at_most_one_key(CONF_PRESENCE_HYSTERESIS_LSB, CONF_PRESENCE_HYSTERESIS_DEGREES),
    cv.has_at_most_one_key(CONF_MOTION_THRESHOLD_LSB, CONF_MOTION_THRESHOLD_DEGREES),
    cv.has_at_most_one_key(CONF_MOTION_HYSTERESIS_LSB, CONF_MOTION_HYSTERESIS_DEGREES),
    cv.has_at_most_one_key(CONF_T_AMBIENT_SHOCK_THRESHOLD_LSB, CONF_T_AMBIENT_SHOCK_THRESHOLD_DEGREES),
    cv.has_at_most_one_key(CONF_T_AMBIENT_SHOCK_HYSTERESIS_LSB, CONF_T_AMBIENT_SHOCK_HYSTERESIS_DEGREES),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # GPIO configuration
    if CONF_INTERRUPT_PIN in config:
        interrupt_pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
        cg.add(var.set_interrupt_pin(interrupt_pin))

    # Device configuration - Averaging
    if CONF_AVG_T_OBJECT_NUMBER in config:
        cg.add(var.set_average_t_object_number(config[CONF_AVG_T_OBJECT_NUMBER]))

    if CONF_AVG_T_AMBIENT_NUMBER in config:
        cg.add(var.set_average_t_ambient_number(config[CONF_AVG_T_AMBIENT_NUMBER]))

    # Device configuration - Basic sensor settings
    if CONF_GAIN_MODE in config:
        cg.add(var.set_gain_mode(config[CONF_GAIN_MODE]))

    if CONF_TMOS_SENSITIVITY in config:
        val = config[CONF_TMOS_SENSITIVITY]
        cg.add(var.set_tmos_sensitivity(cg.RawExpression(f"&(uint16_t){{{val}}}")))

    if CONF_TMOS_ODR in config:
        cg.add(var.set_tmos_odr(config[CONF_TMOS_ODR]))

    # Automatic recalibration interval
    if CONF_RECALIBRATION_INTERVAL in config:
        interval_ms = config[CONF_RECALIBRATION_INTERVAL].total_milliseconds
        cg.add(var.set_recalibration_interval(interval_ms))

    # Device configuration - Presence detection
    if CONF_PRESENCE_THRESHOLD_LSB in config:
        cg.add(var.set_presence_threshold_lsb(config[CONF_PRESENCE_THRESHOLD_LSB]))
    elif CONF_PRESENCE_THRESHOLD_DEGREES in config:
        cg.add(var.set_presence_threshold_degrees(config[CONF_PRESENCE_THRESHOLD_DEGREES]))

    if CONF_PRESENCE_HYSTERESIS_LSB in config:
        cg.add(var.set_presence_hysteresis_lsb(config[CONF_PRESENCE_HYSTERESIS_LSB]))
    elif CONF_PRESENCE_HYSTERESIS_DEGREES in config:
        cg.add(var.set_presence_hysteresis_degree(config[CONF_PRESENCE_HYSTERESIS_DEGREES]))

    if CONF_LPF_PRESENCE_BANDWIDTH in config:
        cg.add(var.set_lpf_presence_bandwidth(config[CONF_LPF_PRESENCE_BANDWIDTH]))

    if CONF_PRESENCE_ABS_VALUE in config:
        cg.add(var.set_presence_abs_value(config[CONF_PRESENCE_ABS_VALUE]))

    # Device configuration - Motion detection
    if CONF_MOTION_THRESHOLD_LSB in config:
        cg.add(var.set_motion_threshold_lsb(config[CONF_MOTION_THRESHOLD_LSB]))
    elif CONF_MOTION_THRESHOLD_DEGREES in config:
        cg.add(var.set_motion_threshold_degree(config[CONF_MOTION_THRESHOLD_DEGREES]))

    if CONF_MOTION_HYSTERESIS_LSB in config:
        cg.add(var.set_motion_hysteresis_lsb(config[CONF_MOTION_HYSTERESIS_LSB]))
    elif CONF_MOTION_HYSTERESIS_DEGREES in config:
        cg.add(var.set_motion_hysteresis_degree(config[CONF_MOTION_HYSTERESIS_DEGREES]))

    if CONF_LPF_MOTION_BANDWIDTH in config:
        cg.add(var.set_lpf_motion_bandwidth(config[CONF_LPF_MOTION_BANDWIDTH]))

    if CONF_LPF_PRESENCE_MOTION_BANDWIDTH in config:
        cg.add(var.set_lpf_presence_motion_bandwidth(config[CONF_LPF_PRESENCE_MOTION_BANDWIDTH]))

    # Device configuration - Ambient shock detection
    if CONF_T_AMBIENT_SHOCK_THRESHOLD_LSB in config:
        cg.add(var.set_t_ambient_shock_threshold_lsb(config[CONF_T_AMBIENT_SHOCK_THRESHOLD_LSB]))
    elif CONF_T_AMBIENT_SHOCK_THRESHOLD_DEGREES in config:
        cg.add(var.set_t_ambient_shock_threshold_degree(config[CONF_T_AMBIENT_SHOCK_THRESHOLD_DEGREES]))

    if CONF_T_AMBIENT_SHOCK_HYSTERESIS_LSB in config:
        cg.add(var.set_t_ambient_shock_hysteresis_lsb(config[CONF_T_AMBIENT_SHOCK_HYSTERESIS_LSB]))
    elif CONF_T_AMBIENT_SHOCK_HYSTERESIS_DEGREES in config:
        cg.add(var.set_t_ambient_shock_hysteresis_degree(config[CONF_T_AMBIENT_SHOCK_HYSTERESIS_DEGREES]))

    if CONF_LPF_AMBIENT_TEMP_BANDWIDTH in config:
        cg.add(var.set_lpf_ambient_temp_bandwidth(config[CONF_LPF_AMBIENT_TEMP_BANDWIDTH]))

    # Device configuration - Interrupt settings
    if CONF_TMOS_ROUTE_INTERRUPT in config:
        cg.add(var.set_tmos_route_interrupt(config[CONF_TMOS_ROUTE_INTERRUPT]))

    if CONF_TMOS_INTERRUPT_OR in config:
        cg.add(var.set_tmos_interrupt_or(config[CONF_TMOS_INTERRUPT_OR]))

    if CONF_DATA_READY_MODE in config:
        cg.add(var.set_data_ready_mode(config[CONF_DATA_READY_MODE]))

    if CONF_INTERRUPT_PULSED in config:
        val = 1 if config[CONF_INTERRUPT_PULSED] else 0
        cg.add(var.set_interrupt_pulsed(val))

    # Device configuration - Algorithm settings
    if CONF_T_OBJECT_ALGO_COMPENSATION in config:
        cg.add(var.set_t_object_algo_compensation(config[CONF_T_OBJECT_ALGO_COMPENSATION]))
