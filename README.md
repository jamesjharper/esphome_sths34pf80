# STHS34PF80 Thermal Infrared Human Presence Sensor

The `sths34pf80` component allows you to use the STMicroelectronics STHS34PF80 thermal infrared sensor with ESPHome. This sensor has been designed to measure the amount of IR radiation emitted from an object within its field of view. The information is digitally processed by the ASIC, which can be programmed to monitor motion, presence, or an overtemperature condition.

## Features

- **Human Presence Detection**: Detects stationary people or objects with temperature differences from ambient
- **Motion Detection**: Detects moving objects with thermal signatures
- **Ambient Temperature Shock Detection**: Detects sudden changes in ambient temperature (e.g., Equipment failure, fire, etc)
- **Temperature Monitoring**: Provides ambient and object temperature readings. (Not suitable for reliable real world measurements)
- **Hardware Interrupt Support**: Optional interrupt pin for improved responsiveness and power efficiency

## Supported Devices

- STHS34PF80 thermal infrared sensor

## Connection

The STHS34PF80 communicates via I2C and optionally uses a GPIO pin for hardware interrupts.
This component only support I2C communication.

```yaml
# I2C bus configuration
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true

# Basic sensor configuration
sths34pf80:
  id: my_sths34pf80
  address: 0x5A              # Default I2C address
  interrupt_pin: GPIO4       # Optional: for hardware interrupts
```

## Configuration Variables

### Basic Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#id)): Manually specify the ID used for code generation.
- **address** (*Optional*, int): Manually specify the I2C address of the sensor. Can not be reconfigured, must be set to `0x5A`.
- **interrupt_pin** (*Optional*, [Pin Schema](https://esphome.io/guides/configuration-types.html#config-pin-schema)): The pin connected to the sensor's INT pin for hardware interrupts. Improves responsiveness and reduces power consumption.

### Configuration
The following settings have sensible defaults which should be adequate for most use cases for detecting human presence. Most users do not need to configure these settings unless you are having issues with reliable detections or have a specialized use case.

#### Sensor Settings
- **avg_t_object_number** (*Optional*, enum): Number of samples to average for object temperature. More averaging reduces noise but slows response. One of `2`, `8`, `32`, `128`, `256`, `512`, `1024`, `2048`. Defaults to `128`.
- **avg_t_ambient_number** (*Optional*, enum): Number of samples to average for ambient temperature. One of `1`, `2`, `4`, `8`. Defaults to `8`.
- **tmos_odr** (*Optional*, enum): Output data rate controlling measurement frequency. Higher rates consume more power but provide faster response. One of `0.25HZ`, `0.5HZ`, `1HZ`, `2HZ`, `4HZ`, `8HZ`, `15HZ`, `30HZ`. Defaults to `15HZ`.
- **gain_mode** (*Optional*, enum): Sensor gain mode. `DEFAULT` enables embedded algorithms with 2016 LSB/°C sensitivity. `WIDE` provides 8x measurement range (250 LSB/°C) but disables detection algorithms. Defaults to `DEFAULT`.

#### Detection Configuration
Most detection parameters can be specified in either LSB units or temperature degrees (°C). Choose one format per parameter, not both.

**Presence Detection:**
- **presence_threshold_lsb** / **presence_threshold_degrees** (*Optional*, int/float): Detection threshold. Lower values = more sensitive. Defaults to  `200` lsb / `0.0992` °C
- **presence_hysteresis_lsb** / **presence_hysteresis_degrees** (*Optional*, int/float): Temperature reduction required to clear detection. Prevents oscillation. Defaults to `50` LSB / `0.0248` °C
- **lpf_presence_bandwidth** (*Optional*, enum): Low-pass filter bandwidth as ratio to ODR. One of `DIV_9`, `DIV_20`, `DIV_50`, `DIV_100`, `DIV_200`, `DIV_400`, `DIV_800`. Defaults to `DIV_9`
- **presence_abs_value** (*Optional*, enum): Absolute value mode for detecting objects colder or hotter than the sensors ambient temperature. `DISABLED`, `ENABLED`, or `ENABLE_DELAYED` (safer, enables after first recalibration). Defaults to `ENABLE_DELAYED`.

**Motion Detection:**
- **motion_threshold_lsb** / **motion_threshold_degrees** (*Optional*, int/float): Motion detection threshold. Defaults to `200` lsb / `0.0992` °C
- **motion_hysteresis_lsb** / **motion_hysteresis_degrees** (*Optional*, int/float): Motion detection hysteresis. Defaults to `50` lsb / `0.0248` °C
- **lpf_motion_bandwidth** (*Optional*, enum): Motion algorithm low-pass filter bandwidth. One of `DIV_9`, `DIV_20`, `DIV_50`, `DIV_100`, `DIV_200`, `DIV_400`, `DIV_800`. Defaults to `DIV_200`
- **lpf_presence_motion_bandwidth** (*Optional*, enum): Combined presence/motion filter bandwidth. One of `DIV_9`, `DIV_20`, `DIV_50`, `DIV_100`, `DIV_200`, `DIV_400`, `DIV_800`. Defaults to `DIV_9`

**Ambient Shock Detection:**
- **t_ambient_shock_threshold_lsb** / **t_ambient_shock_threshold_degrees** (*Optional*, int/float): Ambient shock detection threshold. Defaults to `10` lsb / `0.1000 ` °C 
- **t_ambient_shock_hysteresis_lsb** / **t_ambient_shock_hysteresis_degrees** (*Optional*, int/float): Ambient shock hysteresis. Defaults to `2` lsb / `0.0200` °C
- **lpf_ambient_temp_bandwidth** (*Optional*, enum): Ambient temperature filter bandwidth. One of `DIV_9`, `DIV_20`, `DIV_50`, `DIV_100`, `DIV_200`, `DIV_400`, `DIV_800`. Defaults to `DIV_50`


#### Recalibration
- **recalibration_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time)): Automatic recalibration interval. Improves presence detection accuracy by recalibrating detection algorithms when no detection occurs. Required if presence_abs_value = `ENABLE_DELAYED`. Set to `0s` to disable. Defaults to `10min`.

#### Interrupt
Generally not recommended to change these settings and these settings are managed internal for data polling.
- **tmos_route_interrupt** (*Optional*, enum): Interrupt routing configuration. One of `HIZ`, `DRDY`, `OR`. 
- **tmos_interrupt_or** (*Optional*, enum): Interrupt OR configuration. One of `NONE`, `TSHOCK`, `MOTION`, `TSHOCK_MOTION`, `PRESENCE`, `TSHOCK_PRESENCE`, `MOTION_PRESENCE`, `ALL`. 
- **data_ready_mode** (*Optional*, enum): Data ready signal mode. One of `PULSED`, `LATCHED`. 
- **interrupt_pulsed** (*Optional*, boolean): Enable interrupt pulse mode.

## Binary Sensors

Binary sensors provide detection states based on the configured thresholds.

```yaml
binary_sensor:
  - platform: sths34pf80
    
    presence:
      name: "Presence Detected"
      device_class: occupancy
    
    motion:
      name: "Motion Detected"  
      device_class: motion
      
    thermal_shock:
      name: "Thermal Shock Detected"
```

### Configuration Variables
- **sths34pf80_id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#id)): The ID of the STHS34PF80 component.
- **presence** (*Optional*): Binary sensor for presence detection.
- **motion** (*Optional*): Binary sensor for motion detection. 
- **thermal_shock** (*Optional*): Binary sensor for ambient temperature shock detection. Useful for Equipment failure, fire, etc.

## Sensors

Temperature sensors provide readings in degrees Celsius and algorithm outputs for diagnostics.

```yaml
sensor:
  - platform: sths34pf80

    # Temperature readings
    ambient_temperature:
      name: "Ambient Temperature"
      update_interval: 1s
    
    object_temperature:
      name: "Object Temperature"
      update_interval: 1s
    
    # Algorithm outputs (for diagnostics)
    presence_temperature_delta:
      name: "Presence Algorithm Output"
      update_interval: 1s
    
    motion_temperature_delta:
      name: "Motion Algorithm Output"
      accuracy_decimals: 3
```

### Configuration Variables
- **sths34pf80_id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#id)): The ID of the STHS34PF80 component.
- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time)): The interval to check the sensor. Defaults to `60s`.

**Temperature Sensors:**
- **ambient_temperature** (*Optional*): Room/ambient temperature reading in °C.
- **object_temperature** (*Optional*): Temperature of objects in sensor's field of view in °C.
- **object_temperature_delta** (*Optional*): Difference between object and ambient temperature in °C.

**Algorithm Outputs:**
- **presence_temperature_delta** (*Optional*): Temperature change calculated by presence algorithm in °C.
- **motion_temperature_delta** (*Optional*): Temperature change calculated by motion algorithm in °C.
- **thermal_shock_temperature_delta** (*Optional*): Temperature change from ambient shock algorithm in °C.

**Raw Algorithm Outputs :**
- **presence_raw_lsb** (*Optional*): Raw LSB output from presence algorithm.
- **motion_raw_lsb** (*Optional*): Raw LSB output from motion algorithm.
- **thermal_shock_raw_lsb** (*Optional*): Raw LSB output from ambient shock algorithm.

## Runtime Configuration Components

These components allow runtime adjustment of sensor parameters through Home Assistant.

### Select Components

```yaml
select:
  - platform: sths34pf80
    
    gain_mode:
      name: "Gain Mode"
      initial_value: "DEFAULT"
    
    sample_rate:
      name: "Sample Rate"
      initial_value: "1HZ"
```

Available selects: `average_object_temperature_number`, `average_ambient_temperature_number`, `gain_mode`, `sample_rate`, `low_pass_filter_presence_bandwidth`, `low_pass_filter_motion_bandwidth`, `low_pass_filter_ambient_temperature_bandwidth`.

### Number Components

```yaml
number:
  - platform: sths34pf80
    
    presence_threshold_degree:
      name: "Presence Threshold"
    
    motion_threshold_degree:
      name: "Motion Threshold"
```

Available numbers include threshold and hysteresis controls for all detection algorithms, available in both LSB and degree formats.

### Switch Components

```yaml
switch:
  - platform: sths34pf80
    
    enable_calibration:
      name: "Enable Calibration"
```

## Button Components

```yaml
button:
  - platform: sths34pf80

    start_sampling:
      name: "Start Sampling"
    
    stop_sampling:
      name: "Stop Sampling"
```

## Usage Examples

### Basic Presence Detection

For most applications, the simple configuration is sufficient:

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22

sths34pf80:
  id: presence_sensor
  interrupt_pin: GPIO4

binary_sensor:
  - platform: sths34pf80
    sths34pf80_id: presence_sensor
    
    presence:
      name: "Room Occupied"
      device_class: occupancy
    
    motion:
      name: "Motion Detected"
      device_class: motion
```


## Performance Notes

- **Hardware Interrupts**: Using the interrupt pin improves responsiveness and reduces power consumption compared to polling.
- **Update Intervals**: For sensor components, match or exceed the configured ODR (output data rate) for optimal performance.
- **Defaults Are Sufficient**: The sensor works well with default settings for most applications. Advanced configuration is only needed for specific requirements.
- **Calibration**: The automatic recalibration feature improves accuracy, especially in environments where objects may be present during sensor initialization.


## Technical Details

Key features
- High-sensitivity infrared presence and motion detection sensor
- Reach up to 4 meters without lens for objects measuring 70 x 25 cm²
- Capable of distinguishing between stationary and moving objects
- 80° field of view
- Factory calibrated
- Embedded smart algorithm for presence / motion detection

Electrical specifications
 - Supply voltage: 1.7 V to 3.6 V
 - Supply current: 10 µA
 - 2-wire I²C / 3-wire SPI serial interface
 - Programmable ODRs from 0.25 Hz to 30 Hz

Sensing specifications
 - IR sensitivity: 2000 LSB/°C
 - RMS noise: 25 LSBrms
 - Operating wavelength: 5 µm to 20 µm
 - Local temperature sensor accuracy: ±0.3 °C

For detailed technical information, refer to the [STHS34PF80 datasheet](https://www.st.com/resource/en/datasheet/sths34pf80.pdf).

## See Also

- [Binary Sensor Component](https://esphome.io/components/binary_sensor/index.html)
- [Sensor Component](https://esphome.io/components/sensor/index.html)
- [I²C Bus](https://esphome.io/components/i2c.html)