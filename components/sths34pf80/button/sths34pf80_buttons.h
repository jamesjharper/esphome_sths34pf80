#pragma once

#include "esphome/components/sths34pf80/button/sths34pf80_button.h"
#include "esphome/components/sths34pf80/sths34pf80_component.h"

namespace esphome {
namespace sths34pf80 {

DEFINE_STHS34PF80_BUTTON(StartSamplingButton, start_sampling(), Sths34pf80Component)
DEFINE_STHS34PF80_BUTTON(StopSamplingButton, stop_sampling(), Sths34pf80Component)

}  // namespace sths34pf80
}  // namespace esphome
