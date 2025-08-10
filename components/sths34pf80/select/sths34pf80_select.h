#pragma once

#include "esphome/components/select/select.h"

namespace esphome {
namespace sths34pf80 {

class Sths34pf80Select : public select::Select, public Component {
 protected:
  optional<size_t> initial_value_{};
  bool restore_value_ = true;
  ESPPreferenceObject pref_;

 public:
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

  void set_initial_value(const std::string &option) { this->initial_value_ = this->index_of(option); }

  void set_initial_value(int initial_value) { this->initial_value_ = initial_value; }

  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  size_t get_initial_value() {
    if (this->restore_value_) {
      this->pref_ = global_preferences->make_preference<size_t>(this->get_object_id_hash());
      size_t value;
      if (this->pref_.load(&value)) {
        return value;
      }
    }

    if (this->initial_value_)
      return *this->initial_value_;
    return 0;
  }

  void setup() {
    auto initial_value = this->get_initial_value();
    auto select_value = this->at(initial_value);
    if (select_value) {
      this->control(*select_value);
    }
  }
};

#define DEFINE_STHS34PF80_SELECT_CONTROL(CLASS_NAME, SETTER, PARENT_TYPE) \
  class CLASS_NAME : public sths34pf80::Sths34pf80Select, public Parented<PARENT_TYPE> { \
   protected: \
    void control(const std::string &string_value) override { \
      auto opt_index_value = this->index_of(string_value); \
      if (!opt_index_value) \
        return; \
      auto index_value = *opt_index_value; \
      this->publish_state(string_value); \
      this->parent_->SETTER; \
      if (this->restore_value_) \
        this->pref_.save(&index_value); \
    } \
  };

}  // namespace sths34pf80
}  // namespace esphome