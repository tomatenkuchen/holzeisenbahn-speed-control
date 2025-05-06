/// @file speed_ctrl.hpp
/// @brief control speed of train
/// @author tomatenkuchen
/// @copyright GPLv2.0

#include "driver/gptimer.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "pid.hpp"
#include <cstdtint>
#include <limits>
#include <stdexcept>

namespace drive {

class MeasureSpeed {

public:
  struct Config {
    /// circumpherance of propulsion wheel in meter
    float wheel_circumpherance_m;
    /// timer handle
    gptimer_config_t timer_cfg;
  };

  MeasureSpeed(Config const &_cfg) : cfg{_cfg} {
    if (gptimer_new_timer(&cfg.timer_cfg, &timer_handle) != ESP_OK) {
      throw std::runtime_error("measure speed: timer init failed");
    }
    gptimer_enable(timer_handle);
    gptimer_start(timer_handle);
    gptimer_get_raw_count(timer_handle, &timestamp_latest_tacho_event);
  }

  ~MeasureSpeed() {
    gptimer_stop(timer_handle);
    gptimer_disable(timer_handle);
    gptimer_del_timer(timer_handle);
  }

  /// execute this function on a tacho event
  void on_tacho_event() {
    uint64_t new_count;
    gptimer_get_raw_count(timer_handle, &new_count);
    uint64_t const delta_count = new_count - timestamp_latest_tacho_event;
    speed_m_per_s =
        cfg.wheel_circumpherance_m * cfg.timer_cfg.resolution_hz / delta_count;
    timestamp_latest_tacho_event = new_count;
  }

  float get_speed_m_per_s() const { return speed_m_per_s; }

private:
  Config cfg;
  gptimer_handle_t timer_handle;
  float speed_m_per_s = 0;
  uint64_t timestamp_latest_tacho_event = 0;
};

class MotorControl {
public:
  struct Config {
    mcpwm_timer_config_t timer_cfg;
    mcpwm_operator_config_t operator_cfg;
    mcpwm_comparator_config_t comparator_cfg;
  };

  MotorControl(Config const &_cfg) : cfg{_cfg} {
    mcpwm_new_timer(&cfg.timer_cfg, &timer_handle);
    mcpwm_new_operator(&cfg.operator_cfg, &operator_handle);
    mcpwm_operator_connect_timer(operator_handle, timer_handle);
    mcpwm_new_comparator(operator_handle, &cfg.comparator_cfg,
                         &comparator_handle);
    mcpwm_comparator_set_compare_value(comparator_handle, 0);
    mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP);
  }

  ~MotorControl() {
    mcpwm_del_comparator(comparator_handle);
    mcpwm_del_operator(operator_handle);
    mcpwm_del_timer(timer_handle);
  }

  /// @brief acts like the gas pedal of a car but in both directions
  /// @param duty factor of how much power to be sent to motor. negative values
  /// indicate opposite direction
  void set_duty(int32_t duty) {
    uint32_t const period_ticks = cfg.timer_cfg.period_ticks;
    uint64_t const period_offset = period_ticks / 2;
    int32_t const period_duty =
        period_offset * duty / std::numeric_limits<int32_t>::max();
    uint32_t const new_compare = cfg.timer_cfg.period_ticks / 2 + period_duty;
    mcpwm_comparator_set_compare_value(comparator_handle, new_compare);
  }

private:
  Config cfg;
  mcpwm_timer_handle_t timer_handle;
  mcpwm_oper_handle_t operator_handle;
  mcpwm_cmpr_handle_t comparator_handle;
};

class SpeedControl {
public:
  SpeedControl() {}
  ~SpeedControl() {}

  void set_ref_speed_m_per_s(float speed_m_per_s) {
    speed_ref_m_per_s = speed_m_per_s;
  }

  void on_tacho_event() {
    measure.on_tacho_event();
    float const current_speed = measure.get_speed();
    float const error = speed_ref_m_per_s - current_speed;
    int32_t const duty = pid.update(error);
    control.set_duty(duty);
  }

private:
  MeasureSpeed measure;
  MotorControl control;
  PID<int32_t> pid;
  float speed_ref_m_per_s;
};
} // namespace drive
