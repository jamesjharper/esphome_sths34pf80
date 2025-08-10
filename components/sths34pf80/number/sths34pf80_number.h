#pragma once

#include "esphome/core/preferences.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sths34pf80/sths34pf80_component.h"

namespace esphome {
namespace sths34pf80 {

class Sths34pf80Number : public number::Number, public Component {
 protected:
  optional<float> initial_value_{};
  bool restore_value_ = true;
  ESPPreferenceObject pref_;

 public:
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

  optional<float> get_initial_value();

  void set_initial_value(float initial_value) { this->initial_value_ = initial_value; }
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  void setup() override;

  void dump_config() override;
};

#define DEFINE_NUMBER_CONTROL(CLASS_NAME, SETTER, PARENT_TYPE) \
  class CLASS_NAME : public sths34pf80::Sths34pf80Number, public Parented<PARENT_TYPE> { \
   protected: \
    void control(float value) override { \
      this->publish_state(value); \
      this->parent_->SETTER; \
      if (this->restore_value_) \
        this->pref_.save(&value); \
    } \
  };

// Threshold (LSB)
DEFINE_NUMBER_CONTROL(PresenceThresholdLsbNumber, set_presence_threshold_lsb((uint16_t) value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(MotionThresholdLsbNumber, set_motion_threshold_lsb((uint8_t) value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(AmbientShockThresholdLsbNumber, set_t_ambient_shock_threshold_lsb((uint16_t) value),
                      Sths34pf80Component)

// Threshold (Degrees)
DEFINE_NUMBER_CONTROL(PresenceThresholdDegreeNumber, set_presence_threshold_degrees(value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(MotionThresholdDegreeNumber, set_motion_threshold_degree(value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(AmbientShockThresholdDegreeNumber, set_t_ambient_shock_threshold_degree(value),
                      Sths34pf80Component)

// Hysteresis (LSB)
DEFINE_NUMBER_CONTROL(PresenceHysteresisLsbNumber, set_presence_hysteresis_lsb((uint8_t) value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(MotionHysteresisLsbNumber, set_motion_hysteresis_lsb((uint8_t) value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(AmbientShockHysteresisLsbNumber, set_t_ambient_shock_hysteresis_lsb((uint8_t) value),
                      Sths34pf80Component)

// Hysteresis (Degrees)
DEFINE_NUMBER_CONTROL(PresenceHysteresisDegreeNumber, set_presence_hysteresis_degree(value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(MotionHysteresisDegreeNumber, set_motion_hysteresis_degree(value), Sths34pf80Component)
DEFINE_NUMBER_CONTROL(AmbientShockHysteresisDegreeNumber, set_t_ambient_shock_hysteresis_degree(value),
                      Sths34pf80Component)

// Sensitivity Number - special handling for pointer parameter
class SensitivityNumber : public sths34pf80::Sths34pf80Number, public Parented<Sths34pf80Component> {
 protected:
  void control(float value) override {
    this->publish_state(value);
    uint16_t val = (uint16_t) value;
    this->parent_->set_tmos_sensitivity(&val);
    if (this->restore_value_)
      this->pref_.save(&value);
  }
};

}  // namespace sths34pf80
}  // namespace esphome