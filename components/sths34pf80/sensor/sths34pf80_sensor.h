#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sths34pf80/sths34pf80_component.h"

namespace esphome {
namespace sths34pf80 {

class Sths34pf80Sensor : public sensor::Sensor, public PollingComponent {};

#define DEFINE_STHS34PF80_SENSOR(CLASS_NAME, GETTER, PARENT_TYPE, CALIBRATION_ONLY) \
  class CLASS_NAME : public sths34pf80::Sths34pf80Sensor, public Parented<PARENT_TYPE> { \
   public: \
    void update() override { \
      /* Only publish values if calibration is enabled */ \
      if (CALIBRATION_ONLY && !this->parent_->get_calibration_enabled()) { \
        if (!std::isnan(this->state)) { \
          this->publish_state(NAN); \
        } \
        return; \
      } \
      float val = 0.0f; \
      if (this->parent_->GETTER(&val) == 0) { \
        if (val != this->state) \
          this->publish_state(val); \
      } \
    } \
  };

// Absolute temperature sensors (always enabled)
DEFINE_STHS34PF80_SENSOR(AmbientTemperatureSensor, get_ambient_temperature, Sths34pf80Component, false)
DEFINE_STHS34PF80_SENSOR(ObjectTemperatureSensor, get_object_temperature, Sths34pf80Component, false)

// Temperature delta sensors (in degrees Celsius)
DEFINE_STHS34PF80_SENSOR(ObjectTemperatureDeltaSensor, get_object_temperature_delta, Sths34pf80Component, true)
DEFINE_STHS34PF80_SENSOR(PresenceTemperatureDeltaSensor, get_presence_temperature_delta, Sths34pf80Component, true)
DEFINE_STHS34PF80_SENSOR(MotionTemperatureDeltaSensor, get_motion_temperature_delta, Sths34pf80Component, true)
DEFINE_STHS34PF80_SENSOR(ThermalShockTemperatureDeltaSensor, get_ambient_shock_temperature_delta, Sths34pf80Component,
                         true)

// Raw LSB sensors
DEFINE_STHS34PF80_SENSOR(PresenceRawLsbSensor, get_presence_raw_lsb, Sths34pf80Component, true)
DEFINE_STHS34PF80_SENSOR(MotionRawLsbSensor, get_motion_raw_lsb, Sths34pf80Component, true)
DEFINE_STHS34PF80_SENSOR(ThermalShockRawLsbSensor, get_ambient_shock_raw_lsb, Sths34pf80Component, true)

}  // namespace sths34pf80
}  // namespace esphome