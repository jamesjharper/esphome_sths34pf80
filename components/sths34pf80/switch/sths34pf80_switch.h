#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/components/sths34pf80/sths34pf80_component.h"

namespace esphome {
namespace sths34pf80 {

class Sths34pf80Switch : public switch_::Switch, public Component {
 protected:
  optional<bool> initial_value_{};
  bool restore_value_ = true;
  ESPPreferenceObject pref_;

 public:
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

  void set_initial_value(bool value) { this->initial_value_ = value; }
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  optional<bool> get_initial_value() {
    if (this->restore_value_) {
      this->pref_ = global_preferences->make_preference<bool>(this->get_object_id_hash());

      bool value;
      if (this->pref_.load(&value)) {
        return {value};
      }
    }

    return this->initial_value_;
  }

  void setup() {
    auto initial_value = this->get_initial_value();
    if (initial_value) {
      this->write_state(*initial_value);
    }
  }
};

#define DEFINE_STHS34PF80_SWITCH_CONTROL(CLASS_NAME, SETTER, PARENT_TYPE) \
  class CLASS_NAME : public sths34pf80::Sths34pf80Switch, public Parented<PARENT_TYPE> { \
   protected: \
    void write_state(bool state) override { \
      this->publish_state(state); \
      this->parent_->SETTER; \
      if (this->restore_value_) \
        this->pref_.save(&state); \
    } \
  };

DEFINE_STHS34PF80_SWITCH_CONTROL(EnableCalibrationSwitch, on_calibration_switch_state(state), Sths34pf80Component)

}  // namespace sths34pf80
}  // namespace esphome