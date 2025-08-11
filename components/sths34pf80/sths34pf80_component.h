#pragma once

#include <map>
#include <cstdint>
#include <cstddef>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/optional.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/log.h"

#include "esphome/components/sths34pf80/sths34pf80_reg.h"

namespace esphome {
namespace sths34pf80 {

enum MeasurementState : uint8_t {
  MEASUREMENT_STATE__UNINITIALIZED = 0,
  MEASUREMENT_STATE__STOPPED = 1,

  MEASUREMENT_STATE__WAITING_FOR_POLLED_RESULT = 2,
  MEASUREMENT_STATE__WAITING_FOR_INTERRUPTED_RESULT = 3,
};

enum PresenceAbsMode : uint8_t {
  PRESENCE_ABS_DISABLED = 0,
  PRESENCE_ABS_ENABLED = 1,
  PRESENCE_ABS_ENABLE_DELAYED = 2,
};

struct Sths34pf80InterruptState {
  volatile MeasurementState state{MEASUREMENT_STATE__UNINITIALIZED};
  ISRInternalGPIOPin irq_pin;

  bool has_hardware_interrupt{false};
  bool interrupt_set{false};

  void set_hardware_interrupt(InternalGPIOPin *interrupt_pin) {
    this->has_hardware_interrupt = true;
    this->irq_pin = interrupt_pin->to_isr();
    interrupt_pin->attach_interrupt(Sths34pf80InterruptState::interrupt_handler, this, gpio::INTERRUPT_RISING_EDGE);
  }
  // Interrupt handler function
  static void interrupt_handler(Sths34pf80InterruptState *arg);

  inline bool is_uninitialized() const { return this->state == MEASUREMENT_STATE__UNINITIALIZED; }
  inline bool is_stopped() const { return this->state == MEASUREMENT_STATE__STOPPED; }

  inline bool is_measuring() const { return !(this->is_uninitialized() || this->is_stopped()); }

  inline bool is_waiting_for_polled_result() const {
    return this->state == MEASUREMENT_STATE__WAITING_FOR_POLLED_RESULT;
  }
  inline bool is_waiting_for_interrupted_result() const {
    return this->state == MEASUREMENT_STATE__WAITING_FOR_INTERRUPTED_RESULT;
  }

  inline bool has_measurement_interrupt() const {
    return this->interrupt_set && this->is_waiting_for_interrupted_result();
  }

  inline void uninitialized() { this->state = MEASUREMENT_STATE__UNINITIALIZED; }
  inline void initialized() { this->state = MEASUREMENT_STATE__STOPPED; }
  inline void stop_measurements() { this->state = MEASUREMENT_STATE__STOPPED; }
  inline void start_measurements() { this->wait_for_result(); }

  inline void wait_for_result() {
    this->state = this->has_hardware_interrupt ? MEASUREMENT_STATE__WAITING_FOR_INTERRUPTED_RESULT
                                               : MEASUREMENT_STATE__WAITING_FOR_POLLED_RESULT;
    this->interrupt_set = false;
  }

  inline void start_next_measurements() { this->wait_for_result(); }

  inline void return_to_waiting_for_result() {
    this->state = MEASUREMENT_STATE__WAITING_FOR_POLLED_RESULT;
    this->interrupt_set = false;
  }
};

class Sths34pf80Component : public Component, public i2c::I2CDevice {
 protected:
  // internal driver
  stmdev_ctx_t sensor_;
  Sths34pf80InterruptState state_;

  // Polling scheduling
  uint32_t measurement_start_ms_ = 0;
  uint32_t next_poll_due_ms_ = 0;
  uint32_t odr_ms_ = 1000;

  uint32_t recalibration_interval_ms_ = 0;  // 0 == disabled
  uint32_t last_recalibration_ms_ = 0;

  uint16_t factory_calibrated_sensitivity_ = 2000; // Approx value, actual value is read at start up 

  bool calibration_enabled_ = false;

  // Binary sensors for detection events
  binary_sensor::BinarySensor *presence_detected_binary_sensor_ = nullptr;
  binary_sensor::BinarySensor *movement_detected_binary_sensor_ = nullptr;
  binary_sensor::BinarySensor *thermal_shock_detected_binary_sensor_ = nullptr;

  InternalGPIOPin *interrupt_pin_ = nullptr;

  // Cache for storing parameter values
  struct ParameterCache {
    // Averaging configuration
    optional<sths34pf80_avg_tobject_num_t> avg_t_object_number;
    optional<sths34pf80_avg_tambient_num_t> avg_t_ambient_number;

    // Gain and sensitivity configuration
    optional<sths34pf80_gain_mode_t> gain_mode;
    optional<uint16_t> tmos_sensitivity;

    // Output data rate - determines measurement frequency and power consumption
    sths34pf80_tmos_odr_t tmos_odr = STHS34PF80_TMOS_ODR_AT_15Hz;

    // Presence detection algorithm settings
    optional<uint16_t> presence_threshold_lsb;
    optional<uint8_t> presence_hysteresis_lsb;
    optional<sths34pf80_lpf_bandwidth_t> lpf_presence_bandwidth;
    optional<sths34pf80_lpf_bandwidth_t> lpf_presence_motion_bandwidth;
    optional<PresenceAbsMode> presence_abs_value;

    // Motion detection algorithm settings
    optional<uint8_t> motion_threshold_lsb;
    optional<uint8_t> motion_hysteresis_lsb;
    optional<sths34pf80_lpf_bandwidth_t> lpf_motion_bandwidth;

    // Ambient temperature shock detection settings
    optional<sths34pf80_lpf_bandwidth_t> lpf_ambient_temp_bandwidth;
    optional<uint16_t> t_ambient_shock_threshold_lsb;
    optional<uint16_t> t_ambient_shock_hysteresis_lsb;

    // Interrupt configuration
    optional<sths34pf80_tmos_route_int_t> tmos_route_interrupt;
    optional<sths34pf80_drdy_mode_t> data_ready_mode;
    optional<sths34pf80_tmos_int_or_t> tmos_interrupt_or;

    optional<uint8_t> interrupt_pulsed;
    optional<sths34pf80_int_mode_t> interrupt_mode;

    // Algorithm compensation settings
    optional<uint8_t> t_object_algo_compensation;
  } parameter_cache_;

  inline bool poll_is_due(uint32_t now) const {
    // signed subtraction handles millis() rollover
    return (int32_t) (now - this->next_poll_due_ms_) >= 0;
  }

  inline void schedule_next_poll(uint32_t now) {
    // Align to the next exact ODR boundary using modulo arithmetic
    uint32_t elapsed = now - this->measurement_start_ms_;
    uint32_t remainder = elapsed % this->odr_ms_;
    this->next_poll_due_ms_ = now + (this->odr_ms_ - remainder);
  }

  inline bool recalibration_is_due(uint32_t now) {
    if (this->recalibration_interval_ms_ == 0) {
      return false;
    }
    if (this->last_recalibration_ms_ == 0) {
      this->last_recalibration_ms_ = now;
      return false;
    }
    return (now - this->last_recalibration_ms_) > this->recalibration_interval_ms_;
  }

  inline void schedule_next_recalibration(uint32_t timestamp) { this->last_recalibration_ms_ = timestamp; }

  inline void delay_recalibration() { this->last_recalibration_ms_ = 0; }

  void init_device();
  int32_t is_ready();

  int32_t apply_cached_parameters();
  bool try_read_results(uint32_t timestamp);

  bool check_if_results_are_ready();
  bool process_results(uint32_t timestamp);
  void process_binary_sensor_results();
  int32_t get_data_ready(sths34pf80_tmos_drdy_status_t *drdy);
  int32_t get_status(sths34pf80_tmos_func_status_t *statusVal);
  int32_t restart_device();

 public:
  // Esphome component impl
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void loop() override;

  // Esphome Component setters
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }

  void set_presence_detected_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    this->presence_detected_binary_sensor_ = binary_sensor;
  }

  void set_movement_detected_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    this->movement_detected_binary_sensor_ = binary_sensor;
  }

  void set_thermal_shock_detected_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    this->thermal_shock_detected_binary_sensor_ = binary_sensor;
  }

  // Device sample control

  int32_t start_sampling();
  int32_t stop_sampling();

  // Device outputs

  // Temperature measurements
  // Get the current ambient and object temperature readings in degrees Celsius
  int32_t get_ambient_temperature(float *t);
  int32_t get_object_temperature(float *t);
  int32_t get_object_temperature_delta(float *t);

  // Detection algorithm temperature deltas
  // Get temperature difference values (in °C) calculated by embedded algorithms
  int32_t get_presence_temperature_delta(float *t);
  int32_t get_motion_temperature_delta(float *t);
  int32_t get_ambient_shock_temperature_delta(float *t);

  // Raw algorithm outputs
  // Get unprocessed LSB values from embedded detection algorithms
  int32_t get_presence_raw_lsb(float *lsb);
  int32_t get_motion_raw_lsb(float *lsb);
  int32_t get_ambient_shock_raw_lsb(float *lsb);

  // Device Settings

  // Averaging configuration
  // Controls noise reduction vs response time trade-off
  // More averaging = less noise but slower response to changes
  int32_t get_average_t_object_number(sths34pf80_avg_tobject_num_t *val);
  int32_t set_average_t_object_number(sths34pf80_avg_tobject_num_t num);
  int32_t get_average_t_ambient_number(sths34pf80_avg_tambient_num_t *val);
  int32_t set_average_t_ambient_number(sths34pf80_avg_tambient_num_t num);

  // Gain and sensitivity configuration
  // Normal mode: 2000 LSB/°C, embedded algorithms enabled
  // Wide mode: 250 LSB/°C (8x measurement range)
  // Note: Wide mode disables embedded algorithms, requires external software detection
  int32_t get_gain_mode(sths34pf80_gain_mode_t *gain);
  int32_t set_gain_mode(sths34pf80_gain_mode_t mode);

  // TMOS sensitivity configuration
  // Used by compensation algorithm - update when using optical elements that reduce transmittance
  int32_t get_tmos_sensitivity(uint16_t *sense);
  int32_t set_tmos_sensitivity(uint16_t *val);

  // Optical transmittance (0.0-1.0)
  // Represents light transmission efficiency of optical elements (lenses, covers)
  int32_t get_transmittance(float *transmittance) {
    uint16_t sensitivity = 0;
    int32_t result = get_tmos_sensitivity(&sensitivity);
    *transmittance = sensitivity == 0 ? 1.0f : sensitivity / 2016.0f;
    return result;
  }

  int32_t set_transmittance(float transmittance) {
    uint16_t sensitivity = (uint16_t) (transmittance * 2016.0f);
    return this->set_tmos_sensitivity(&sensitivity);
  }

  // Automatic recalibration interval
  // Resets algorithms when no detection occurs for specified time
  // Improves presence detection accuracy, especially when using absolute mode
  // Required when objects may be present during device initialization
  inline void set_recalibration_interval(uint32_t interval_ms) { this->recalibration_interval_ms_ = interval_ms; }

  // Output data rate (ODR) configuration
  // Controls measurement frequency: 0.25-30 Hz
  // Higher rates = more power consumption but faster response
  // Also affects low-pass filter characteristics (calculated as fraction of ODR)
  int32_t get_tmos_odr(sths34pf80_tmos_odr_t *val);
  int32_t set_tmos_odr(sths34pf80_tmos_odr_t val);

  // Low pass filter applied to the temptrue data
  // as input for the embedded detection algorithms.
  //
  // Filters out high frequency noise. In practice these settings
  // are used to control how sensitive each detection algorithms is
  // to change in the sensor output.
  //
  // Values are given as ratio to ODR (Output Data Rate)
  // Changing ODR will affect filter characteristics
  int32_t get_lpf_motion_bandwidth(sths34pf80_lpf_bandwidth_t *val);
  int32_t set_lpf_motion_bandwidth(sths34pf80_lpf_bandwidth_t val);
  int32_t get_lpf_presence_motion_bandwidth(sths34pf80_lpf_bandwidth_t *val);
  int32_t set_lpf_presence_motion_bandwidth(sths34pf80_lpf_bandwidth_t val);
  int32_t get_lpf_ambient_temp_bandwidth(sths34pf80_lpf_bandwidth_t *val);
  int32_t set_lpf_ambient_temp_bandwidth(sths34pf80_lpf_bandwidth_t val);
  int32_t get_lpf_presence_bandwidth(sths34pf80_lpf_bandwidth_t *val);
  int32_t set_lpf_presence_bandwidth(sths34pf80_lpf_bandwidth_t val);

  // Presence detection absolute value mode
  // Enable when detecting objects that may be colder or hotter than ambient temperature
  //
  // - DISABLED: Absolute value is disabled
  // - ENABLED: Absolute value is enabled immediately
  // - ENABLE_DELAYED: Absolute value is disabled initially, but enabled after first recalibration
  //                   This ensures the scene is empty before enabling bidirectional sensitivity
  int32_t get_presence_abs_value(PresenceAbsMode *val);
  int32_t set_presence_abs_value(PresenceAbsMode mode);

  // Presence detection threshold
  // Determines sensitivity - lower values = more sensitive detection
  // Available in both LSB units and temperature degrees
  int32_t get_presence_threshold_lsb(uint16_t *val);
  int32_t get_presence_threshold_degree(float *val) {
    uint16_t lsb = 0;
    auto result = this->get_presence_threshold_lsb(&lsb);
    *val = this->convert_object_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_presence_threshold_lsb(uint16_t val);
  int32_t set_presence_threshold_degrees(float val) {
    uint16_t lsb = this->convert_degrees_to_ambient_lsb(val);
    return this->set_presence_threshold_lsb(lsb);
  }

  // Motion detection threshold
  // Determines motion sensitivity - lower values = more sensitive detection
  int32_t get_motion_threshold_lsb(uint16_t *val);
  int32_t get_motion_threshold_degree(float *val) {
    uint16_t lsb = 0;
    auto result = this->get_motion_threshold_lsb(&lsb);
    *val = this->convert_object_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_motion_threshold_lsb(uint8_t threshold);
  int32_t set_motion_threshold_degree(float val) {
    uint16_t lsb = this->convert_degrees_to_ambient_lsb(val);
    return this->set_motion_threshold_lsb(lsb);
  }

  // Ambient temperature shock detection threshold
  // Detects sudden ambient temperature changes
  int32_t get_t_ambient_shock_threshold_lsb(uint16_t *val);
  int32_t get_t_ambient_shock_threshold_degree(float *val) {
    uint16_t lsb = 0;
    auto result = this->get_t_ambient_shock_threshold_lsb(&lsb);
    *val = this->convert_ambient_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_t_ambient_shock_threshold_lsb(uint16_t threshold);
  int32_t set_t_ambient_shock_threshold_degree(float val) {
    uint16_t lsb = this->convert_degrees_to_ambient_lsb(val);
    return this->set_t_ambient_shock_threshold_lsb(lsb);
  }

  // Hysteresis configuration
  // Hysteresis is the reduction in temperature required to clear
  // the detection. In practices, this is to prevent flag oscillation
  // around threshold boundaries.
  //
  // Changing ether threshold or hysteresis value can be used reduce
  // oscillation if necessary
  int32_t get_motion_hysteresis_lsb(uint8_t *val);
  int32_t get_motion_hysteresis_degree(float *val) {
    uint8_t lsb = 0;
    auto result = this->get_motion_hysteresis_lsb(&lsb);
    *val = this->convert_object_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_motion_hysteresis_lsb(uint8_t hysteresis);
  int32_t set_motion_hysteresis_degree(float val) {
    uint8_t lsb = this->convert_degrees_to_object_lsb(val);
    return this->set_motion_hysteresis_lsb(lsb);
  }

  int32_t get_presence_hysteresis_lsb(uint8_t *val);
  int32_t get_presence_hysteresis_degree(float *val) {
    uint8_t lsb = 0;
    auto result = this->get_presence_hysteresis_lsb(&lsb);
    *val = this->convert_object_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_presence_hysteresis_lsb(uint8_t hysteresis);
  int32_t set_presence_hysteresis_degree(float val) {
    uint8_t lsb = this->convert_degrees_to_object_lsb(val);
    return this->set_presence_hysteresis_lsb(lsb);
  }

  int32_t get_t_ambient_shock_hysteresis_lsb(uint8_t *val);
  int32_t get_t_ambient_shock_hysteresis_degree(float *val) {
    uint8_t lsb = 0;
    auto result = this->get_t_ambient_shock_hysteresis_lsb(&lsb);
    *val = this->convert_ambient_lsb_to_degrees(lsb);
    return result;
  }
  int32_t set_t_ambient_shock_hysteresis_lsb(uint8_t hysteresis);
  int32_t set_t_ambient_shock_hysteresis_degree(float val) {
    uint8_t lsb = this->convert_degrees_to_ambient_lsb(val);
    return this->set_t_ambient_shock_hysteresis_lsb(lsb);
  }

  // Interrupt configuration
  // Configure what signals are routed to the INT pin and their electrical properties
  // Note: that these values are managed internally, changes may result in unexpect behavior
  int32_t get_tmos_route_interrupt(sths34pf80_tmos_route_int_t *val);
  int32_t set_tmos_route_interrupt(sths34pf80_tmos_route_int_t val);
  int32_t get_tmos_interrupt_or(sths34pf80_tmos_int_or_t *val);
  int32_t set_tmos_interrupt_or(sths34pf80_tmos_int_or_t val);
  int32_t get_data_ready_mode(sths34pf80_drdy_mode_t *val);
  int32_t set_data_ready_mode(sths34pf80_drdy_mode_t val);
  int32_t get_interrupt_pulsed(uint8_t *val);
  int32_t set_interrupt_pulsed(uint8_t pulse);

  // INT pin electrical: push-pull/open-drain, active high/low
  int32_t get_interrupt_mode(sths34pf80_int_mode_t *val);
  int32_t set_interrupt_mode(sths34pf80_int_mode_t val);

  // Embedded compensation algorithm
  // Linear ambient temperature compensation for object temperature readings
  // Currently not implemented
  int32_t get_t_object_algo_compensation(uint8_t *val);  // Get compensation algorithm state
  int32_t set_t_object_algo_compensation(bool comp);     // Enable/disable compensation

  // Calibration control
  // Controls whether calibration values are exposed to Home Assistant
  bool get_calibration_enabled() const { return this->calibration_enabled_; }

  void on_calibration_switch_state(bool state) { this->calibration_enabled_ = state; }

 protected:
  // Unit conversion utilities
  // Object temperature: Uses TMOS sensitivity default 2016 LSB/°C
  float convert_object_lsb_to_degrees(int16_t lsb_sen) { return (float) lsb_sen / this->factory_calibrated_sensitivity_; }

  int16_t convert_degrees_to_object_lsb(float degrees) { return (int16_t) (degrees * (float) this->factory_calibrated_sensitivity_); }

  // Ambient temperature: Fixed 100 LSB/°C sensitivity (datasheet Section 4.5.2)
  float convert_ambient_lsb_to_degrees(int16_t lsb100) { return (float) lsb100 / 100.0f; }
  int16_t convert_degrees_to_ambient_lsb(float degrees) { return (int16_t) (degrees * 100.0f); }

  // i2c impl

  int32_t write_register_(uint8_t reg, const uint8_t *data, uint16_t length);

  int32_t read_register_(uint8_t reg, uint8_t *data, uint16_t numBytes);

  static int32_t read(void *device, uint8_t addr, uint8_t *data, uint16_t numData);

  static int32_t write(void *device, uint8_t addr, const uint8_t *data, uint16_t numData);

  static void delay_ms(uint32_t millisec) { delay(millisec); }


};

}  // namespace sths34pf80
}  // namespace esphome
