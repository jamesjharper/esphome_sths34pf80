#include "sths34pf80_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sths34pf80 {

static const char *const TAG = "sths34pf80.number";

optional<float> Sths34pf80Number::get_initial_value() { return this->initial_value_; }

void Sths34pf80Number::setup() {
  float value;
  if (!this->restore_value_) {
    if (this->initial_value_.has_value()) {
      value = *this->initial_value_;
    } else {
      value = this->traits.get_min_value();
    }
  } else {
    this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
    if (!this->pref_.load(&value)) {
      if (this->initial_value_.has_value()) {
        value = *this->initial_value_;
      } else {
        value = this->traits.get_min_value();
      }
    }
  }
  this->publish_state(value);
}

void Sths34pf80Number::dump_config() {
  LOG_NUMBER("", "STHS34PF80 Number", this);
  ESP_LOGCONFIG(TAG, "  Restore value: %s", YESNO(this->restore_value_));
  if (this->initial_value_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Initial value: %.2f", *this->initial_value_);
  }
}

}  // namespace sths34pf80
}  // namespace esphome