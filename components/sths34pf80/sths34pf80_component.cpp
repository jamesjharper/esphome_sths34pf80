#include "esphome/components/sths34pf80/sths34pf80_component.h"
#include "esphome/components/sths34pf80/sths34pf80_enums.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sths34pf80 {

static const char *const TAG = "sths34pf80";

void IRAM_ATTR Sths34pf80InterruptState::interrupt_handler(Sths34pf80InterruptState *arg) { arg->interrupt_set = true; }

void Sths34pf80Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up sths34pf80...");

  sensor_.read_reg = Sths34pf80Component::read;
  sensor_.write_reg = Sths34pf80Component::write;
  sensor_.mdelay = Sths34pf80Component::delay_ms;
  sensor_.handle = this;

  // Configure hardware interrupt if available
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    this->state_.set_hardware_interrupt(this->interrupt_pin_);

    // Set default interrupt configuration
    if (!this->parameter_cache_.tmos_route_interrupt.has_value()) {
      this->parameter_cache_.tmos_route_interrupt = {STHS34PF80_TMOS_INT_DRDY};
    }
    if (!this->parameter_cache_.data_ready_mode.has_value()) {
      this->parameter_cache_.data_ready_mode = {STHS34PF80_DRDY_LATCHED};
    }
    if (!this->parameter_cache_.interrupt_pulsed.has_value()) {
      this->parameter_cache_.interrupt_pulsed = {0};
    }
    if (!this->parameter_cache_.tmos_interrupt_or.has_value()) {
      this->parameter_cache_.tmos_interrupt_or = {STHS34PF80_TMOS_INT_ALL};
    }
  } else {
    // turn off interrupt if not in use
    if (!this->parameter_cache_.tmos_route_interrupt.has_value()) {
      this->parameter_cache_.tmos_route_interrupt = {STHS34PF80_TMOS_INT_HIZ};
    }
     if (!this->parameter_cache_.data_ready_mode.has_value()) {
      this->parameter_cache_.data_ready_mode = {STHS34PF80_DRDY_LATCHED};
    }
    if (!this->parameter_cache_.interrupt_pulsed.has_value()) {
      this->parameter_cache_.interrupt_pulsed = {0};
    }
    if (!this->parameter_cache_.tmos_interrupt_or.has_value()) {
      this->parameter_cache_.tmos_interrupt_or = {STHS34PF80_TMOS_INT_NONE};
    }
  }

  this->init_device();

  ESP_LOGD(TAG, "sths34pf80 setup complete");
}

void Sths34pf80Component::dump_config() {

  ESP_LOGCONFIG(TAG, "Sths34pf80:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);

  if (this->state_.is_uninitialized()) {
    return;
  }

  uint8_t dev_id = 0;
  if (sths34pf80_device_id_get(&this->sensor_, &dev_id) == 0) {
    ESP_LOGCONFIG(TAG, "  Device Type Id: 0x%02X", dev_id);
  }

  sths34pf80_avg_tobject_num_t avg_t_obj_num;
  if (this->get_average_t_object_number(&avg_t_obj_num) == 0) {
    ESP_LOGCONFIG(TAG, "  Object Temperature Samples: %ix", get_avg_t_object_numbers(avg_t_obj_num));
  }

  sths34pf80_avg_tambient_num_t avg_t_amb_num;
  if (this->get_average_t_ambient_number(&avg_t_amb_num) == 0) {
    ESP_LOGCONFIG(TAG, "  Ambient Temperature Samples: %ix", get_avg_t_ambient_numbers(avg_t_amb_num));
  }

  sths34pf80_gain_mode_t gain;
  if (this->get_gain_mode(&gain) == 0) {
    ESP_LOGCONFIG(TAG, "  Gain Mode: %s", get_gain_mode_str(gain));
  }

  u_int16_t sensitivity;
  if (this->get_tmos_sensitivity(&sensitivity) == 0) {
    ESP_LOGCONFIG(TAG, "  Sensitivity: %i LSB / °C", sensitivity);
  }

  sths34pf80_tmos_odr_t odr;
  if (this->get_tmos_odr(&odr) == 0) {
    ESP_LOGCONFIG(TAG, "  ORD: %.2fHz, %.2fms", get_odr_frequency(odr), get_odr_interval(odr));
  }

  uint8_t compensation_enabled;
  if (this->get_t_object_algo_compensation(&compensation_enabled) == 0) {
    ESP_LOGCONFIG(TAG, "    Temperature Compensation: %s", compensation_enabled == 1 ? "yes" : "no");
  }

  ESP_LOGCONFIG(TAG, "  Presence detection:");
  uint16_t presence_threshold_lsb;
  if (this->get_presence_threshold_lsb(&presence_threshold_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Threshold: %u lsb / %.4f °C", presence_threshold_lsb,
                  this->convert_object_lsb_to_degrees(presence_threshold_lsb));
  }

  uint8_t presence_hysteresis_lsb;
  if (this->get_presence_hysteresis_lsb(&presence_hysteresis_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Hysteresis: %u  / %.4f °C", presence_hysteresis_lsb,
                  this->convert_object_lsb_to_degrees(presence_hysteresis_lsb));
  }

  sths34pf80_lpf_bandwidth_t lpf_presence_bandwidth;
  if (this->get_lpf_presence_bandwidth(&lpf_presence_bandwidth) == 0) {
    auto f = get_lpr_frequency(lpf_presence_bandwidth, odr);
    auto period = (1.0f / f) * 1000.0f;
    ESP_LOGCONFIG(TAG, "    Low Pass Filter: ORD / %i, %.2fHz, %.0fms ", get_lpr_bandwidth(lpf_presence_bandwidth), f,
                  period);
  }

  sths34pf80_lpf_bandwidth_t lpf_presence_motion;
  if (this->get_lpf_presence_motion_bandwidth(&lpf_presence_motion) == 0) {
    auto f = get_lpr_frequency(lpf_presence_motion, odr);
    auto period = (1.0f / f) * 1000.0f;
    ESP_LOGCONFIG(TAG, "    Presence Motion Low Pass Filter: ORD / %i, %.2fHz, %.0fms ",
                  get_lpr_bandwidth(lpf_presence_motion), f, period);
  }

  // Show presence absolute value mode
  if (this->parameter_cache_.presence_abs_value.has_value()) {
    auto mode = this->parameter_cache_.presence_abs_value.value();
    const char *mode_str = (mode == PRESENCE_ABS_DISABLED)  ? "disabled"
                           : (mode == PRESENCE_ABS_ENABLED) ? "enabled"
                                                            : "enable_delayed";
    ESP_LOGCONFIG(TAG, "    Absolute value: %s", mode_str);

  } else {
    uint8_t current_abs_value;
    if (sths34pf80_presence_abs_value_get(&this->sensor_, &current_abs_value) == 0) {
      const char *mode_str = current_abs_value == 1 ? "enabled" : "enabled";
      ESP_LOGCONFIG(TAG, "    Absolute value: %s", mode_str);
    }
  }

  // Motion detection
  ESP_LOGCONFIG(TAG, "  Motion detection:");

  uint16_t motion_threshold_lsb;
  if (this->get_motion_threshold_lsb(&motion_threshold_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Threshold: %u lsb / %.4f °C", motion_threshold_lsb,
                  this->convert_object_lsb_to_degrees(motion_threshold_lsb));
  }

  uint8_t motion_hysteresis_lsb;
  if (this->get_motion_hysteresis_lsb(&motion_hysteresis_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Hysteresis: %u lsb / %.4f °C", motion_hysteresis_lsb,
                  this->convert_object_lsb_to_degrees(motion_hysteresis_lsb));
  }

  sths34pf80_lpf_bandwidth_t lpf_motion;
  if (this->get_lpf_motion_bandwidth(&lpf_motion) == 0) {
    auto f = get_lpr_frequency(lpf_motion, odr);
    auto period = (1.0f / f) * 1000.0f;
    ESP_LOGCONFIG(TAG, "    Low Pass Filter: ORD / %i, %.2fHz, %.0fms ", get_lpr_bandwidth(lpf_motion), f, period);
  }

  if (this->get_lpf_presence_motion_bandwidth(&lpf_presence_motion) == 0) {
    auto f = get_lpr_frequency(lpf_presence_motion, odr);
    auto period = (1.0f / f) * 1000.0f;
    ESP_LOGCONFIG(TAG, "    Presence Motion Low Pass Filter: ORD / %i, %.2fHz, %.0fms ",
                  get_lpr_bandwidth(lpf_presence_motion), f, period);
  }

  // Ambient temperature shock detection

  ESP_LOGCONFIG(TAG, "  Ambient temperature shock detection:");

  sths34pf80_lpf_bandwidth_t lpf_ambient_temp;
  if (this->get_lpf_ambient_temp_bandwidth(&lpf_ambient_temp) == 0) {
    auto f = get_lpr_frequency(lpf_ambient_temp, odr);
    auto period = (1.0f / f) * 1000.0f;
    ESP_LOGCONFIG(TAG, "    Low Pass Filter: ORD / %i,  %.2fHz, %.0fms ", get_lpr_bandwidth(lpf_ambient_temp), f,
                  period);
  }

  uint16_t ambient_shock_threshold_lsb;
  if (this->get_t_ambient_shock_threshold_lsb(&ambient_shock_threshold_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Threshold: %u lsb / %.4f °C", ambient_shock_threshold_lsb,
                  this->convert_ambient_lsb_to_degrees(ambient_shock_threshold_lsb));
  }

  uint8_t ambient_shock_hysteresis_lsb;
  if (this->get_t_ambient_shock_hysteresis_lsb(&ambient_shock_hysteresis_lsb) == 0) {
    ESP_LOGCONFIG(TAG, "    Hysteresis: %u lsb / %.4f °C", ambient_shock_hysteresis_lsb,
                  this->convert_ambient_lsb_to_degrees(ambient_shock_hysteresis_lsb));
  }

  ESP_LOGCONFIG(TAG, "  Interrupt:");

  sths34pf80_tmos_route_int_t route_int;
  if (this->get_tmos_route_interrupt(&route_int) == 0) {
    ESP_LOGCONFIG(TAG, "    Routing: %s", get_route_int_str(route_int));
  }

  sths34pf80_tmos_int_or_t int_or;
  if (this->get_tmos_interrupt_or(&int_or) == 0) {
    ESP_LOGCONFIG(TAG, "    OR Events: %s", get_route_int_or_str(int_or));
  }

  sths34pf80_drdy_mode_t drdy_mode;
  if (this->get_data_ready_mode(&drdy_mode) == 0) {
    ESP_LOGCONFIG(TAG, "    Trigger Type: %s", get_data_ready_mode_str(drdy_mode));
  }

  uint8_t is_pulsed;
  if (this->get_interrupt_pulsed(&is_pulsed) == 0) {
    ESP_LOGCONFIG(TAG, "    Algo Pulsed: %s", is_pulsed ? "yes" : "no");
  }

  ESP_LOGCONFIG(TAG, "  GPIO:");

  sths34pf80_int_mode_t interrupt_mode;
  if (this->get_interrupt_mode(&interrupt_mode) == 0) {
    ESP_LOGCONFIG(TAG, "    pin: %s", get_int_mode_pin(interrupt_mode));
    ESP_LOGCONFIG(TAG, "    polarity: %s", get_int_mode_polarity(interrupt_mode));
  }
}

float Sths34pf80Component::get_setup_priority() const { return setup_priority::DATA; }


void Sths34pf80Component::init_device() {
  // Verify device ID
  uint8_t dev_id = 0;
  auto err = sths34pf80_device_id_get(&this->sensor_, &dev_id);
  if (err != 0) {
    ESP_LOGE(TAG, "Unable to resolve device type id. Please make sure the i2c address is correct.");
    this->mark_failed();
    return;
  }

  if (dev_id != STHS34PF80_ID) {
    ESP_LOGE(TAG, "Device Id %u is not supported by the component");
    this->mark_failed();
    return;
  }

  auto result = this->restart_device();
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to restart device. Attempting to continue");
  }

  // read factory calibrated sensitivity 
  if(this->get_tmos_sensitivity(&this->factory_calibrated_sensitivity_) != 0) {
    // If failed to get calibrated value assume approximate value
    this->factory_calibrated_sensitivity_ = 2000;
  }
  

  this->apply_cached_parameters();
  this->start_sampling();

  // Ensure binary sensors dont report "unknown" until a detection is made
  if (this->presence_detected_binary_sensor_) {
    this->presence_detected_binary_sensor_->publish_state(false);
  }

  if (this->movement_detected_binary_sensor_) {
    this->movement_detected_binary_sensor_->publish_state(false);
  }

  if (this->thermal_shock_detected_binary_sensor_) {
    this->thermal_shock_detected_binary_sensor_->publish_state(false);
  }
}

int32_t Sths34pf80Component::apply_cached_parameters() {
  // Updates should only be made during power down mode
 // this->stop_sampling();

  // Enable block data update, this is needed because esphome loop
  // is too slow to read reliably at higher ODR frequencies.
  int32_t result = sths34pf80_block_data_update_set(&this->sensor_, 1);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to enable block data rate update");
  }

  result = 0;
  // Apply cached temperature object average number

  auto avg_t_object_number = this->parameter_cache_.avg_t_object_number.value_or(get_max_allowed_avg_for_odr(this->parameter_cache_.tmos_odr));
  result = sths34pf80_avg_tobject_num_set(&this->sensor_, avg_t_object_number);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached temperature object average number");
    this->mark_failed();
  }

  // Apply cached ambient temperature average number
  result = sths34pf80_avg_tambient_num_set(&this->sensor_, this->parameter_cache_.avg_t_ambient_number.value_or(STHS34PF80_AVG_T_8));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached ambient temperature average number");
    this->mark_failed();
  }

  // Apply cached gain mode
  result = sths34pf80_gain_mode_set(&this->sensor_, this->parameter_cache_.gain_mode.value_or(STHS34PF80_GAIN_DEFAULT_MODE));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached gain mode");
    this->mark_failed();
  }

  // Apply cached TMOS sensitivity
  if (this->parameter_cache_.tmos_sensitivity.has_value()) {
    uint16_t sensitivity = this->parameter_cache_.tmos_sensitivity.value();
    result = sths34pf80_tmos_sensitivity_set(&this->sensor_, &sensitivity);
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached sensitivity");
      this->mark_failed();
    }
  }

  result = sths34pf80_lpf_m_bandwidth_set(&this->sensor_, this->parameter_cache_.lpf_motion_bandwidth.value_or(STHS34PF80_LPF_ODR_DIV_200));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached LPF motion bandwidth");
    this->mark_failed();
  }

  result = sths34pf80_lpf_p_m_bandwidth_set(&this->sensor_, this->parameter_cache_.lpf_presence_motion_bandwidth.value_or(STHS34PF80_LPF_ODR_DIV_9));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached LPF presence motion bandwidth");
    this->mark_failed();
  }

  result = sths34pf80_lpf_a_t_bandwidth_set(&this->sensor_, this->parameter_cache_.lpf_ambient_temp_bandwidth.value_or(STHS34PF80_LPF_ODR_DIV_50));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached LPF ambient temp bandwidth");
    this->mark_failed();
  }

  result = sths34pf80_lpf_p_bandwidth_set(&this->sensor_, this->parameter_cache_.lpf_presence_bandwidth.value_or(STHS34PF80_LPF_ODR_DIV_200));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached LPF presence bandwidth");
    this->mark_failed();
  }

  // Apply cached thresholds
  result = sths34pf80_presence_threshold_set(&this->sensor_, this->parameter_cache_.presence_threshold_lsb.value_or(200));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached presence threshold");
    this->mark_failed();
  }

  result = sths34pf80_motion_threshold_set(&this->sensor_, this->parameter_cache_.motion_threshold_lsb.value_or(200));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached motion threshold");
    this->mark_failed();
  }

  result = sths34pf80_tambient_shock_threshold_set(&this->sensor_,
                                                    this->parameter_cache_.t_ambient_shock_threshold_lsb.value_or(10));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached ambient shock threshold");
    this->mark_failed();
  }

  // Apply cached hysteresis values
  result = sths34pf80_motion_hysteresis_set(&this->sensor_, this->parameter_cache_.motion_hysteresis_lsb.value_or(50));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached motion hysteresis");
    this->mark_failed();
  }

  result = sths34pf80_presence_hysteresis_set(&this->sensor_, this->parameter_cache_.presence_hysteresis_lsb.value_or(50));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached presence hysteresis");
    this->mark_failed();
  }

  result = sths34pf80_tambient_shock_hysteresis_set(&this->sensor_,
                                                    this->parameter_cache_.t_ambient_shock_hysteresis_lsb.value_or(2));
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached ambient shock hysteresis");
    this->mark_failed();
  }

  uint8_t abs_value = this->parameter_cache_.presence_abs_value.value_or(PRESENCE_ABS_DISABLED) == PRESENCE_ABS_ENABLED ? 1 : 0;
  result = sths34pf80_presence_abs_value_set(&this->sensor_, abs_value);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to apply cached presence abs value");
    this->mark_failed();
  }


  //if (this->parameter_cache_.t_object_algo_compensation.has_value()) {
    result = sths34pf80_tobject_algo_compensation_set(&this->sensor_,
                                                      this->parameter_cache_.t_object_algo_compensation.value_or(0));
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached object algo compensation");
    }
  //}


  // Apply cached interrupt settings
  if (this->parameter_cache_.tmos_route_interrupt.has_value()) {
    result = sths34pf80_tmos_route_int_set(&this->sensor_, this->parameter_cache_.tmos_route_interrupt.value());
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached interrupt routing");
    }
  }

  if (this->parameter_cache_.tmos_interrupt_or.has_value()) {
    result = sths34pf80_tmos_int_or_set(&this->sensor_, this->parameter_cache_.tmos_interrupt_or.value());
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached interrupt OR");
    }
  }

  if (this->parameter_cache_.interrupt_mode.has_value()) {
    result = sths34pf80_int_mode_set(&this->sensor_, this->parameter_cache_.interrupt_mode.value());
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached interrupt mode");
    }
  }

  if (this->parameter_cache_.interrupt_pulsed.has_value()) {
    result = sths34pf80_int_or_pulsed_set(&this->sensor_, this->parameter_cache_.interrupt_pulsed.value());
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached interrupt pulsed enable/disable");
    }
  }

  if (this->parameter_cache_.data_ready_mode.has_value()) {
    result = sths34pf80_drdy_mode_set(&this->sensor_, this->parameter_cache_.data_ready_mode.value());
    if (result != 0) {
      ESP_LOGW(TAG, "Failed to apply cached data ready mode");
    }
  }

  return 0;
}

// Device Initialization and Status

int32_t Sths34pf80Component::is_ready() {
  uint8_t dev_id = 0;
  int32_t err = sths34pf80_device_id_get(&this->sensor_, &dev_id);
  return dev_id == STHS34PF80_ID;
}

int32_t Sths34pf80Component::restart_device() {
  int32_t result = sths34pf80_boot_set(&this->sensor_, true);
  if (result != 0) {
    return result;
  }
  this->sensor_.mdelay(3);  // Device needs 2.5ms to restart

  // Make sure the device is in power-down by disabling ODR
  result = sths34pf80_tmos_odr_set(&this->sensor_, STHS34PF80_TMOS_ODR_OFF);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to put device into power down mode", result);
  }

  // Reset embedded algorithms as required when leaving power-down
  result = sths34pf80_algo_reset(&this->sensor_);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to reset algorithms when starting sampling (error %d)", result);
  }

  // Mark as in an idle state
  this->state_.stop_measurements();
  return result;
}

// Sampling Control

int32_t Sths34pf80Component::stop_sampling() {
  if (this->state_.is_stopped()) {
    return 0;
  }

  // Clear interrupt
  sths34pf80_tmos_func_status_t status;
  this->get_status(&status);

  // Put device into power-down by disabling ODR
  sths34pf80_tmos_odr_set(&this->sensor_, STHS34PF80_TMOS_ODR_OFF);

  // Reset embedded algorithms as required when leaving power-down
  int32_t result = sths34pf80_algo_reset(&this->sensor_);
  if (result != 0) {
    ESP_LOGW(TAG, "Failed to reset algorithms when starting sampling (error %d)", result);
  }

  this->state_.stop_measurements();
  return 0;
}

int32_t Sths34pf80Component::start_sampling() {
  if (this->state_.is_measuring()) {
    return 0;
  }

  // Determine desired ODR – fall back to 1 Hz if not configured / was OFF
  sths34pf80_tmos_odr_t odr = this->parameter_cache_.tmos_odr;
  if (odr == STHS34PF80_TMOS_ODR_OFF) {
    odr = STHS34PF80_TMOS_ODR_AT_15Hz;
  }

  // Apply cached TMOS ODR and update sample poll rate
  // this takes the device out of power down mode, and
  // set it into continuous mode

  int32_t result = sths34pf80_tmos_odr_set(&this->sensor_, odr);

  this->odr_ms_ = (uint32_t) get_odr_interval(odr);
  this->measurement_start_ms_ = millis();
  this->schedule_next_poll(this->measurement_start_ms_);
  this->delay_recalibration();  // device will recalibrate as it starts
  this->state_.start_measurements();

  return 0;
}

// Device Measurement Loop

void Sths34pf80Component::loop() {
  if (this->state_.is_stopped() || this->state_.is_uninitialized()) {
    return;
  }
  auto now = millis();

  // Handle interrupt-based measurements
  if (this->state_.is_waiting_for_interrupted_result()) {
    if (!this->state_.has_measurement_interrupt()) {
      return;
    }

    if (!this->try_read_results(now)) {
      ESP_LOGD(TAG, "Poisoned interrupt detected.");
      this->schedule_next_poll(now);
      this->state_.return_to_waiting_for_result();
    }
    return;
  }

  // Handle polled measurements
  if (this->state_.is_waiting_for_polled_result()) {
    if (this->poll_is_due(now)) {
      if (this->try_read_results(now)) {
        // Only advance the schedule on successful read to avoid skipping data on I2C errors
        this->schedule_next_poll(now);
      }
    }
    return;
  }
}

bool Sths34pf80Component::try_read_results(uint32_t timestamp) {
  return this->check_if_results_are_ready() && this->process_results(timestamp);
}

bool Sths34pf80Component::check_if_results_are_ready() {
  sths34pf80_tmos_drdy_status_t dataReady;
  this->get_data_ready(&dataReady);
  return dataReady.drdy == 1;
}

bool Sths34pf80Component::process_results(uint32_t timestamp) {
  this->process_binary_sensor_results();

  if (this->recalibration_is_due(timestamp)) {
    // reset will calibrate the device
    ESP_LOGD(TAG, "Recalibrating device.");

    // Enable abs if delay abs was enabled.
    if (this->parameter_cache_.presence_abs_value.value_or(PRESENCE_ABS_DISABLED) == PRESENCE_ABS_ENABLE_DELAYED) {
      ESP_LOGI(TAG, "Presence absolute value mode enabled after first recalibration");
      this->parameter_cache_.presence_abs_value = {PRESENCE_ABS_ENABLED};
    }

    this->apply_cached_parameters();  // reset algorithm and reapply settings
    this->schedule_next_recalibration(timestamp);
  }
  this->state_.start_next_measurements();
  return true;
}

void Sths34pf80Component::process_binary_sensor_results() {
  sths34pf80_tmos_func_status_t status;
  if (this->get_status(&status) != 0) {
    return;
  }

  bool thermal_shock_detected = status.tamb_shock_flag == 1;
  bool motion_detected = status.mot_flag == 1;
  bool presence_detected = status.pres_flag == 1;
  bool is_clear = !(thermal_shock_detected || motion_detected || presence_detected);

  // reset recalibration timer while the scene isnt clear
  if (!is_clear) {
    this->delay_recalibration();
  }

  if (this->presence_detected_binary_sensor_ && this->presence_detected_binary_sensor_->state != presence_detected) {
    this->presence_detected_binary_sensor_->publish_state(presence_detected);
  }

  if (this->movement_detected_binary_sensor_ && this->movement_detected_binary_sensor_->state != motion_detected) {
    this->movement_detected_binary_sensor_->publish_state(motion_detected);
  }

  if (this->thermal_shock_detected_binary_sensor_ &&
      this->thermal_shock_detected_binary_sensor_->state != thermal_shock_detected) {
    this->thermal_shock_detected_binary_sensor_->publish_state(thermal_shock_detected);
  }
}

int32_t Sths34pf80Component::get_data_ready(sths34pf80_tmos_drdy_status_t *drdy) {
  return sths34pf80_tmos_drdy_status_get(&this->sensor_, drdy);
}

int32_t Sths34pf80Component::get_status(sths34pf80_tmos_func_status_t *val) {
  // Clears interrupt
  return sths34pf80_tmos_func_status_get(&this->sensor_, val);
}

// Temperature and Sensor Data Reading

int32_t Sths34pf80Component::get_presence_temperature_delta(float *val) {
  int16_t lsb = 0;
  int32_t result = sths34pf80_tpresence_raw_get(&this->sensor_, &lsb);
  *val = this->convert_object_lsb_to_degrees(lsb);
  return result;
}

int32_t Sths34pf80Component::get_motion_temperature_delta(float *val) {
  int16_t lsb = 0;
  int32_t result = sths34pf80_tmotion_raw_get(&this->sensor_, &lsb);
  *val = this->convert_object_lsb_to_degrees(lsb);
  return result;
}

int32_t Sths34pf80Component::get_ambient_shock_temperature_delta(float *val) {
  int16_t lsb = 0;
  int32_t result = sths34pf80_tamb_shock_raw_get(&this->sensor_, &lsb);
  *val = this->convert_ambient_lsb_to_degrees(lsb);
  return result;
}

int32_t Sths34pf80Component::get_presence_raw_lsb(float *lsb) {
  int16_t raw_lsb = 0;
  int32_t result = sths34pf80_tpresence_raw_get(&this->sensor_, &raw_lsb);
  *lsb = (float) raw_lsb;
  return result;
}

int32_t Sths34pf80Component::get_motion_raw_lsb(float *lsb) {
  int16_t raw_lsb = 0;
  int32_t result = sths34pf80_tmotion_raw_get(&this->sensor_, &raw_lsb);
  *lsb = (float) raw_lsb;
  return result;
}

int32_t Sths34pf80Component::get_ambient_shock_raw_lsb(float *lsb) {
  int16_t raw_lsb = 0;
  int32_t result = sths34pf80_tamb_shock_raw_get(&this->sensor_, &raw_lsb);
  *lsb = (float) raw_lsb;
  return result;
}

int32_t Sths34pf80Component::get_object_temperature_delta(float *temperature) {
  int16_t lsb = 0;
  int32_t result = this->parameter_cache_.t_object_algo_compensation.value_or(0) == 1
                       ? sths34pf80_tobj_comp_raw_get(&this->sensor_, &lsb)
                       : sths34pf80_tobject_raw_get(&this->sensor_, &lsb);

  *temperature = this->convert_object_lsb_to_degrees(lsb);
  return result;
}

int32_t Sths34pf80Component::get_ambient_temperature(float *temperature) {
  int16_t lsb100 = 0;
  int32_t result = sths34pf80_tambient_raw_get(&this->sensor_, &lsb100);
  *temperature = this->convert_ambient_lsb_to_degrees(lsb100);
  return result;
}

int32_t Sths34pf80Component::get_object_temperature(float *temperature) {
  float ambient = 0.0f;
  float object_delta = 0.0f;

  int32_t result1 = this->get_ambient_temperature(&ambient);
  if (result1 != 0) {
    return result1;
  }

  int32_t result2 = this->get_object_temperature_delta(&object_delta);
  if (result2 != 0) {
    return result2;
  }

  *temperature = ambient + object_delta;
  return 0;
}

// Configuration

int32_t Sths34pf80Component::get_average_t_object_number(sths34pf80_avg_tobject_num_t *val) {
  return sths34pf80_avg_tobject_num_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_average_t_object_number(sths34pf80_avg_tobject_num_t num) {
  this->parameter_cache_.avg_t_object_number = num;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_avg_tobject_num_set(&this->sensor_, num);
}

int32_t Sths34pf80Component::get_average_t_ambient_number(sths34pf80_avg_tambient_num_t *val) {
  return sths34pf80_avg_tambient_num_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_average_t_ambient_number(sths34pf80_avg_tambient_num_t num) {
  this->parameter_cache_.avg_t_ambient_number = num;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_avg_tambient_num_set(&this->sensor_, num);
}

int32_t Sths34pf80Component::get_gain_mode(sths34pf80_gain_mode_t *gain) {
  return sths34pf80_gain_mode_get(&this->sensor_, gain);
}

int32_t Sths34pf80Component::set_gain_mode(sths34pf80_gain_mode_t gain) {
  this->parameter_cache_.gain_mode = gain;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_gain_mode_set(&this->sensor_, gain);
}

int32_t Sths34pf80Component::get_tmos_sensitivity(uint16_t *val) {
  return sths34pf80_tmos_sensitivity_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_tmos_sensitivity(uint16_t *val) {
  this->parameter_cache_.tmos_sensitivity = *val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tmos_sensitivity_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_tmos_odr(sths34pf80_tmos_odr_t *val) {
  return sths34pf80_tmos_odr_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_tmos_odr(sths34pf80_tmos_odr_t val) {
  //  auto-configure optimal averaging for object temperature
  if (!this->parameter_cache_.avg_t_object_number.has_value()) {
    auto suggested_avg = get_max_allowed_avg_for_odr(val);
    this->set_average_t_object_number(suggested_avg);
  } else {
    auto configured_avg = this->parameter_cache_.avg_t_object_number.value();
    auto max_odr_for_avg = get_max_odr_for_avg(configured_avg);
    if (val > max_odr_for_avg) {
      ESP_LOGE(TAG,
               "Configured avg_t_object_number %u samples incompatible with requested ODR %.2fHz (max allowed %.2fHz).",
               get_avg_t_object_numbers(configured_avg), get_odr_frequency(val),
               get_odr_frequency(max_odr_for_avg));

      val = max_odr_for_avg;
    }
  }

  this->parameter_cache_.tmos_odr = val;
  this->odr_ms_ = (uint32_t) get_odr_interval(val);

  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply directly
  return sths34pf80_tmos_odr_set(&this->sensor_, val);
}

// Interrupt Configuration

int32_t Sths34pf80Component::get_tmos_route_interrupt(sths34pf80_tmos_route_int_t *val) {
  return sths34pf80_tmos_route_int_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_tmos_route_interrupt(sths34pf80_tmos_route_int_t val) {
  this->parameter_cache_.tmos_route_interrupt = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tmos_route_int_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_tmos_interrupt_or(sths34pf80_tmos_int_or_t *val) {
  return sths34pf80_tmos_int_or_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_tmos_interrupt_or(sths34pf80_tmos_int_or_t val) {
  this->parameter_cache_.tmos_interrupt_or = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tmos_int_or_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_interrupt_mode(sths34pf80_int_mode_t *val) {
  return sths34pf80_int_mode_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_interrupt_mode(sths34pf80_int_mode_t val) {
  this->parameter_cache_.interrupt_mode = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_int_mode_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_interrupt_pulsed(uint8_t *val) {
  return sths34pf80_int_or_pulsed_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_interrupt_pulsed(uint8_t pulse) {
  this->parameter_cache_.interrupt_pulsed = pulse;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_int_or_pulsed_set(&this->sensor_, pulse);
}

int32_t Sths34pf80Component::get_data_ready_mode(sths34pf80_drdy_mode_t *val) {
  return sths34pf80_drdy_mode_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_data_ready_mode(sths34pf80_drdy_mode_t val) {
  this->parameter_cache_.data_ready_mode = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_drdy_mode_set(&this->sensor_, val);
}

// Presence Detection Configuration

int32_t Sths34pf80Component::get_presence_threshold_lsb(uint16_t *val) {
  return sths34pf80_presence_threshold_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_presence_threshold_lsb(uint16_t threshold) {
  this->parameter_cache_.presence_threshold_lsb = threshold;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_presence_threshold_set(&this->sensor_, threshold);
}

int32_t Sths34pf80Component::get_presence_hysteresis_lsb(uint8_t *val) {
  return sths34pf80_presence_hysteresis_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_presence_hysteresis_lsb(uint8_t hysteresis) {
  this->parameter_cache_.presence_hysteresis_lsb = hysteresis;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_presence_hysteresis_set(&this->sensor_, hysteresis);
}

int32_t Sths34pf80Component::get_lpf_presence_bandwidth(sths34pf80_lpf_bandwidth_t *val) {
  return sths34pf80_lpf_p_bandwidth_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_lpf_presence_bandwidth(sths34pf80_lpf_bandwidth_t val) {
  this->parameter_cache_.lpf_presence_bandwidth = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_lpf_p_bandwidth_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_presence_abs_value(PresenceAbsMode *mode) {
  if (this->parameter_cache_.presence_abs_value.has_value()) {
    *mode = this->parameter_cache_.presence_abs_value.value();
    return 0;
  }

  uint8_t val = 0;
  int32_t result = sths34pf80_presence_abs_value_get(&this->sensor_, &val);
  *mode = val == 1 ? PRESENCE_ABS_ENABLED : PRESENCE_ABS_DISABLED;
  return result;
}

int32_t Sths34pf80Component::set_presence_abs_value(PresenceAbsMode mode) {
  this->parameter_cache_.presence_abs_value = mode;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }

  uint8_t val = mode == PRESENCE_ABS_ENABLED ? 1 : 0;
  return sths34pf80_presence_abs_value_set(&this->sensor_, val);
}

// Motion Detection Configuration

int32_t Sths34pf80Component::get_motion_threshold_lsb(uint16_t *val) {
  return sths34pf80_motion_threshold_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_motion_threshold_lsb(uint8_t threshold) {
  this->parameter_cache_.motion_threshold_lsb = threshold;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_motion_threshold_set(&this->sensor_, threshold);
}

int32_t Sths34pf80Component::get_motion_hysteresis_lsb(uint8_t *val) {
  return sths34pf80_motion_hysteresis_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_motion_hysteresis_lsb(uint8_t hysteresis) {
  this->parameter_cache_.motion_hysteresis_lsb = hysteresis;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_motion_hysteresis_set(&this->sensor_, hysteresis);
}

int32_t Sths34pf80Component::get_lpf_motion_bandwidth(sths34pf80_lpf_bandwidth_t *val) {
  return sths34pf80_lpf_m_bandwidth_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_lpf_motion_bandwidth(sths34pf80_lpf_bandwidth_t val) {
  this->parameter_cache_.lpf_motion_bandwidth = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_lpf_m_bandwidth_set(&this->sensor_, val);
}

int32_t Sths34pf80Component::get_lpf_presence_motion_bandwidth(sths34pf80_lpf_bandwidth_t *val) {
  return sths34pf80_lpf_p_m_bandwidth_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_lpf_presence_motion_bandwidth(sths34pf80_lpf_bandwidth_t val) {
  this->parameter_cache_.lpf_presence_motion_bandwidth = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_lpf_p_m_bandwidth_set(&this->sensor_, val);
}

// Ambient Shock Detection Configuration

int32_t Sths34pf80Component::get_t_ambient_shock_threshold_lsb(uint16_t *val) {
  return sths34pf80_tambient_shock_threshold_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_t_ambient_shock_threshold_lsb(uint16_t threshold) {
  this->parameter_cache_.t_ambient_shock_threshold_lsb = threshold;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tambient_shock_threshold_set(&this->sensor_, threshold);
}

int32_t Sths34pf80Component::get_t_ambient_shock_hysteresis_lsb(uint8_t *val) {
  return sths34pf80_tambient_shock_hysteresis_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_t_ambient_shock_hysteresis_lsb(uint8_t hysteresis) {
  this->parameter_cache_.t_ambient_shock_hysteresis_lsb = hysteresis;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tambient_shock_hysteresis_set(&this->sensor_, hysteresis);
}

int32_t Sths34pf80Component::get_lpf_ambient_temp_bandwidth(sths34pf80_lpf_bandwidth_t *val) {
  return sths34pf80_lpf_a_t_bandwidth_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_lpf_ambient_temp_bandwidth(sths34pf80_lpf_bandwidth_t val) {
  this->parameter_cache_.lpf_ambient_temp_bandwidth = val;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_lpf_a_t_bandwidth_set(&this->sensor_, val);
}

// Algorithm Configuration

int32_t Sths34pf80Component::get_t_object_algo_compensation(uint8_t *val) {
  return sths34pf80_tobject_algo_compensation_get(&this->sensor_, val);
}

int32_t Sths34pf80Component::set_t_object_algo_compensation(bool comp) {
  this->parameter_cache_.t_object_algo_compensation = comp ? 1 : 0;
  if (this->state_.is_uninitialized()) {
    return 0;  // Set during setup()
  } else if (!this->state_.is_stopped()) {
    return this->apply_cached_parameters();  // reset algorithm and reapply settings
  }
  // apply dirrectly
  return sths34pf80_tobject_algo_compensation_set(&this->sensor_, comp ? 1 : 0);
}

//  I2C Implementation

int32_t Sths34pf80Component::write_register_(uint8_t reg, const uint8_t *data, uint16_t length) {
  auto error = this->write_register(reg, data, length);
  if (error != esphome::i2c::ErrorCode::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write %i bytes to register 0x%04X. Error %i", length, reg, error);
    return -1;
  }
  return 0;
}

int32_t Sths34pf80Component::read_register_(uint8_t reg, uint8_t *data, uint16_t numBytes) {
  auto error = this->read_register(reg, data, numBytes, /*stop*/ true);
  if (error != esphome::i2c::ErrorCode::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read %u bytes from register 0x%04X", numBytes, reg);
    return -1;
  }
  return 0;
}

int32_t Sths34pf80Component::read(void *device, uint8_t addr, uint8_t *data, uint16_t numData) {
  return ((Sths34pf80Component *) device)->read_register_(addr, data, numData);
}

int32_t Sths34pf80Component::write(void *device, uint8_t addr, const uint8_t *data, uint16_t numData) {
  return ((Sths34pf80Component *) device)->write_register_(addr, data, numData);
}

};  // namespace sths34pf80
};  // namespace esphome