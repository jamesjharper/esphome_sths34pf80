#pragma once

#include <map>
#include <cstdint>
#include <cstddef>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

#include "esphome/components/sths34pf80/sths34pf80_reg.h"

namespace esphome {
namespace sths34pf80 {

static u_int16_t get_avg_t_ambient_numbers(sths34pf80_avg_tambient_num_t avg_t_amb_num) {
  switch (avg_t_amb_num) {
    case STHS34PF80_AVG_T_8:
      return 8;
    case STHS34PF80_AVG_T_4:
      return 4;
    case STHS34PF80_AVG_T_2:
      return 2;
    case STHS34PF80_AVG_T_1:
      return 1;
    default:
      return 0;
  }
}

static u_int16_t get_avg_t_object_numbers(sths34pf80_avg_tobject_num_t avg_t_obj_num) {
  switch (avg_t_obj_num) {
    case STHS34PF80_AVG_TMOS_2:
      return 2;
    case STHS34PF80_AVG_TMOS_8:
      return 8;
    case STHS34PF80_AVG_TMOS_32:
      return 32;
    case STHS34PF80_AVG_TMOS_128:
      return 128;
    case STHS34PF80_AVG_TMOS_256:
      return 256;
    case STHS34PF80_AVG_TMOS_512:
      return 512;
    case STHS34PF80_AVG_TMOS_1024:
      return 1024;
    case STHS34PF80_AVG_TMOS_2048:
      return 2048;
    default:
      return 0;
  }
}

static const char *get_gain_mode_str(sths34pf80_gain_mode_t gain_mode) {
  switch (gain_mode) {
    case STHS34PF80_GAIN_WIDE_MODE:
      return "Wide";
    case STHS34PF80_GAIN_DEFAULT_MODE:
      return "default";
    default:
      return "Unknown";
  }
}

static const char *get_one_shot_state_str(sths34pf80_tmos_one_shot_t state) {
  switch (state) {
    case STHS34PF80_TMOS_IDLE_MODE:
      return "idle";
    case STHS34PF80_TMOS_ONE_SHOT:
      return "one shot";
    default:
      return "Unknown";
  }
}

static float get_odr_interval(sths34pf80_tmos_odr_t odr) {
  switch (odr) {
    case STHS34PF80_TMOS_ODR_OFF:
      return NAN;
    case STHS34PF80_TMOS_ODR_AT_0Hz25:
      return 4000.0f;
    case STHS34PF80_TMOS_ODR_AT_0Hz50:
      return 2000.0f;
    case STHS34PF80_TMOS_ODR_AT_1Hz:
      return 1000.0f;
    case STHS34PF80_TMOS_ODR_AT_2Hz:
      return 500.0f;
    case STHS34PF80_TMOS_ODR_AT_4Hz:
      return 250.0f;
    case STHS34PF80_TMOS_ODR_AT_8Hz:
      return 126.0f;
    case STHS34PF80_TMOS_ODR_AT_15Hz:
      return 66.6f;
    case STHS34PF80_TMOS_ODR_AT_30Hz:
      return 33.3f;
    default:
      return NAN;
  }
}

static float get_odr_frequency(sths34pf80_tmos_odr_t odr) {
  switch (odr) {
    case STHS34PF80_TMOS_ODR_OFF:
      return NAN;
    case STHS34PF80_TMOS_ODR_AT_0Hz25:
      return 0.25f;
    case STHS34PF80_TMOS_ODR_AT_0Hz50:
      return 0.20f;
    case STHS34PF80_TMOS_ODR_AT_1Hz:
      return 1.0f;
    case STHS34PF80_TMOS_ODR_AT_2Hz:
      return 2.0f;
    case STHS34PF80_TMOS_ODR_AT_4Hz:
      return 4.0f;
    case STHS34PF80_TMOS_ODR_AT_8Hz:
      return 8.0f;
    case STHS34PF80_TMOS_ODR_AT_15Hz:
      return 15.0f;
    case STHS34PF80_TMOS_ODR_AT_30Hz:
      return 30.0f;
    default:
      return NAN;
  }
}

static const char *get_mem_bank_str(sths34pf80_mem_bank_t state) {
  switch (state) {
    case STHS34PF80_MAIN_MEM_BANK:
      return "main memory";
    case STHS34PF80_EMBED_FUNC_MEM_BANK:
      return "embedded function";
    default:
      return "Unknown";
  }
}

static u_int16_t get_lpr_bandwidth(sths34pf80_lpf_bandwidth_t state) {
  switch (state) {
    case STHS34PF80_LPF_ODR_DIV_9:
      return 9;
    case STHS34PF80_LPF_ODR_DIV_20:
      return 20;
    case STHS34PF80_LPF_ODR_DIV_50:
      return 50;
    case STHS34PF80_LPF_ODR_DIV_100:
      return 100;
    case STHS34PF80_LPF_ODR_DIV_200:
      return 200;
    case STHS34PF80_LPF_ODR_DIV_400:
      return 400;
    case STHS34PF80_LPF_ODR_DIV_800:
      return 000;
    default:
      return 0;
  }
}

static float get_lpr_frequency(sths34pf80_lpf_bandwidth_t state, sths34pf80_tmos_odr_t odr) {
  return (float) get_lpr_bandwidth(state) / (float) get_odr_frequency(odr);
}

static const char *get_route_int_str(sths34pf80_tmos_route_int_t state) {
  switch (state) {
    case STHS34PF80_TMOS_INT_HIZ:  // High-z, ie high impedance, ie disconnected
      return "Disabled";
    case STHS34PF80_TMOS_INT_DRDY:
      return "Data-Ready";
    case STHS34PF80_TMOS_INT_OR:
      return "Algorithm-Detect";
    default:
      return "Unknown";
  }
}

static const char *get_route_int_or_str(sths34pf80_tmos_int_or_t state) {
  switch (state) {
    case STHS34PF80_TMOS_INT_NONE:
      return "None";
    case STHS34PF80_TMOS_INT_TSHOCK:
      return "Shock";
    case STHS34PF80_TMOS_INT_MOTION:
      return "Motion";
    case STHS34PF80_TMOS_INT_TSHOCK_MOTION:
      return "Shock | Motion";
    case STHS34PF80_TMOS_INT_PRESENCE:
      return "Presence";

    case STHS34PF80_TMOS_INT_TSHOCK_PRESENCE:
      return "Shock | Presence";

    case STHS34PF80_TMOS_INT_MOTION_PRESENCE:
      return "Motion | Presence";

    case STHS34PF80_TMOS_INT_ALL:
      return "All";
    default:
      return "Unknown";
  }
}

static const char *get_int_mode_pin(sths34pf80_int_mode_t mode) {
  switch (mode.pin) {
    case sths34pf80_int_mode_t::STHS34PF80_PUSH_PULL:
      return "Push-Pull";
    case sths34pf80_int_mode_t::STHS34PF80_OPEN_DRAIN:
      return "Open-Drain";
    default:
      return "Unknown";
  }
}

static const char *get_int_mode_polarity(sths34pf80_int_mode_t mode) {
  switch (mode.polarity) {
    case sths34pf80_int_mode_t::STHS34PF80_ACTIVE_HIGH:
      return "Active-High";
    case sths34pf80_int_mode_t::STHS34PF80_ACTIVE_LOW:
      return "Active-Low";
    default:
      return "Unknown";
  }
}

static const char *get_data_ready_mode_str(sths34pf80_drdy_mode_t mode) {
  switch (mode) {
    case STHS34PF80_DRDY_PULSED:
      return "Pulsed";
    case STHS34PF80_DRDY_LATCHED:
      return "Latched";
    default:
      return "Unknown";
  }
}

static inline sths34pf80_tmos_odr_t get_max_odr_for_avg(sths34pf80_avg_tobject_num_t avg) {
  switch (avg) {
    case STHS34PF80_AVG_TMOS_2:
    case STHS34PF80_AVG_TMOS_8:
    case STHS34PF80_AVG_TMOS_32:
      return STHS34PF80_TMOS_ODR_AT_30Hz;
    case STHS34PF80_AVG_TMOS_128:
      return STHS34PF80_TMOS_ODR_AT_8Hz;
    case STHS34PF80_AVG_TMOS_256:
      return STHS34PF80_TMOS_ODR_AT_4Hz;
    case STHS34PF80_AVG_TMOS_512:
      return STHS34PF80_TMOS_ODR_AT_2Hz;
    case STHS34PF80_AVG_TMOS_1024:
      return STHS34PF80_TMOS_ODR_AT_1Hz;
    case STHS34PF80_AVG_TMOS_2048:
      return STHS34PF80_TMOS_ODR_AT_0Hz50;
    default:
      return STHS34PF80_TMOS_ODR_AT_30Hz;
  }
}

static inline sths34pf80_avg_tobject_num_t get_max_allowed_avg_for_odr(sths34pf80_tmos_odr_t odr) {
  switch (odr) {
    case STHS34PF80_TMOS_ODR_AT_30Hz:
    case STHS34PF80_TMOS_ODR_AT_15Hz:
      return STHS34PF80_AVG_TMOS_32;
    case STHS34PF80_TMOS_ODR_AT_8Hz:
      return STHS34PF80_AVG_TMOS_128;
    case STHS34PF80_TMOS_ODR_AT_4Hz:
      return STHS34PF80_AVG_TMOS_256;
    case STHS34PF80_TMOS_ODR_AT_2Hz:
      return STHS34PF80_AVG_TMOS_512;
    case STHS34PF80_TMOS_ODR_AT_1Hz:
      return STHS34PF80_AVG_TMOS_1024;
    case STHS34PF80_TMOS_ODR_AT_0Hz50:
    case STHS34PF80_TMOS_ODR_AT_0Hz25:
    default:
      return STHS34PF80_AVG_TMOS_2048;
  }
}

}  // namespace sths34pf80
}  // namespace esphome
