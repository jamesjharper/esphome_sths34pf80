#pragma once

#include "esphome/components/button/button.h"

namespace esphome {
namespace sths34pf80 {

// Base class for buttons related to the STHS34PF80 component.  Currently it
// does not add any functionality beyond the base Button class, but it
// provides a common type for future extension and macro convenience.
class Sths34pf80Button : public button::Button {};

// Convenience macro to generate simple stateless buttons that just invoke an
// action on their parent component when pressed.
#define DEFINE_STHS34PF80_BUTTON(CLASS_NAME, ACTION, PARENT_TYPE) \
  class CLASS_NAME : public sths34pf80::Sths34pf80Button, public Parented<PARENT_TYPE> { \
   public: \
    void press_action() override { this->parent_->ACTION; } \
  };

}  // namespace sths34pf80
}  // namespace esphome
