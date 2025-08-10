#pragma once

#include "esphome/components/sths34pf80/select/sths34pf80_select.h"
#include "esphome/components/sths34pf80/sths34pf80_component.h"

namespace esphome {
namespace sths34pf80 {

// Helper function to get enum value from options
template<typename T> T get_enum_from_index(size_t index, const std::vector<T> &enum_values) {
  if (index < enum_values.size()) {
    return enum_values[index];
  }
  return enum_values[0];  // Default to first option
}

// Averaging selects
DEFINE_STHS34PF80_SELECT_CONTROL(AverageObjectTemperatureNumberSelect,
                                 set_average_t_object_number(get_enum_from_index(
                                     index_value,
                                     std::vector<sths34pf80_avg_tobject_num_t>{
                                         STHS34PF80_AVG_TMOS_2, STHS34PF80_AVG_TMOS_8, STHS34PF80_AVG_TMOS_32,
                                         STHS34PF80_AVG_TMOS_128, STHS34PF80_AVG_TMOS_256, STHS34PF80_AVG_TMOS_512,
                                         STHS34PF80_AVG_TMOS_1024, STHS34PF80_AVG_TMOS_2048})),
                                 Sths34pf80Component)

DEFINE_STHS34PF80_SELECT_CONTROL(
    AverageAmbientTemperatureNumberSelect,
    set_average_t_ambient_number(get_enum_from_index(
        index_value, std::vector<sths34pf80_avg_tambient_num_t>{STHS34PF80_AVG_T_1, STHS34PF80_AVG_T_2,
                                                                STHS34PF80_AVG_T_4, STHS34PF80_AVG_T_8})),
    Sths34pf80Component)

// Gain mode select
DEFINE_STHS34PF80_SELECT_CONTROL(
    GainModeSelect,
    set_gain_mode(get_enum_from_index(index_value, std::vector<sths34pf80_gain_mode_t>{STHS34PF80_GAIN_DEFAULT_MODE,
                                                                                       STHS34PF80_GAIN_WIDE_MODE})),
    Sths34pf80Component)

// Sample rate select
DEFINE_STHS34PF80_SELECT_CONTROL(
    SampleRateSelect,
    set_tmos_odr(
        get_enum_from_index(index_value,
                            std::vector<sths34pf80_tmos_odr_t>{
                                STHS34PF80_TMOS_ODR_OFF, STHS34PF80_TMOS_ODR_AT_0Hz25, STHS34PF80_TMOS_ODR_AT_0Hz50,
                                STHS34PF80_TMOS_ODR_AT_1Hz, STHS34PF80_TMOS_ODR_AT_2Hz, STHS34PF80_TMOS_ODR_AT_4Hz,
                                STHS34PF80_TMOS_ODR_AT_8Hz, STHS34PF80_TMOS_ODR_AT_15Hz, STHS34PF80_TMOS_ODR_AT_30Hz})),
    Sths34pf80Component)

// Low-pass filter bandwidth selects
DEFINE_STHS34PF80_SELECT_CONTROL(LowPassFilterPresenceBandwidthSelect,
                                 set_lpf_presence_bandwidth(get_enum_from_index(
                                     index_value,
                                     std::vector<sths34pf80_lpf_bandwidth_t>{
                                         STHS34PF80_LPF_ODR_DIV_9, STHS34PF80_LPF_ODR_DIV_20, STHS34PF80_LPF_ODR_DIV_50,
                                         STHS34PF80_LPF_ODR_DIV_100, STHS34PF80_LPF_ODR_DIV_200,
                                         STHS34PF80_LPF_ODR_DIV_400, STHS34PF80_LPF_ODR_DIV_800})),
                                 Sths34pf80Component)

DEFINE_STHS34PF80_SELECT_CONTROL(LowPassFilterMotionBandwidthSelect,
                                 set_lpf_motion_bandwidth(get_enum_from_index(
                                     index_value,
                                     std::vector<sths34pf80_lpf_bandwidth_t>{
                                         STHS34PF80_LPF_ODR_DIV_9, STHS34PF80_LPF_ODR_DIV_20, STHS34PF80_LPF_ODR_DIV_50,
                                         STHS34PF80_LPF_ODR_DIV_100, STHS34PF80_LPF_ODR_DIV_200,
                                         STHS34PF80_LPF_ODR_DIV_400, STHS34PF80_LPF_ODR_DIV_800})),
                                 Sths34pf80Component)

DEFINE_STHS34PF80_SELECT_CONTROL(LowPassFilterAmbientTemperatureBandwidthSelect,
                                 set_lpf_ambient_temp_bandwidth(get_enum_from_index(
                                     index_value,
                                     std::vector<sths34pf80_lpf_bandwidth_t>{
                                         STHS34PF80_LPF_ODR_DIV_9, STHS34PF80_LPF_ODR_DIV_20, STHS34PF80_LPF_ODR_DIV_50,
                                         STHS34PF80_LPF_ODR_DIV_100, STHS34PF80_LPF_ODR_DIV_200,
                                         STHS34PF80_LPF_ODR_DIV_400, STHS34PF80_LPF_ODR_DIV_800})),
                                 Sths34pf80Component)

}  // namespace sths34pf80
}  // namespace esphome