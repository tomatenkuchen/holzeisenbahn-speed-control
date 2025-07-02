/// @file speed_ctrl.hpp
/// @brief control speed of train
/// @author tomatenkuchen
/// @copyright GPLv2.0

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "hal/gpio_types.h"
#include "lok/pid.hpp"

using namespace std::chrono_literals;

namespace lok {

class MeasureSpeed {
 public:
  struct Config {
    /// circumpherance of propulsion wheel in meter
    float wheel_circumference_m;
    /// timer handle
    gptimer_config_t timer_cfg;
    /// tacho input pin
    gpio_config_t tacho_pin;
    /// tacho pin callback function
    void (*tacho_pin_callback)(void *);
  };

  MeasureSpeed(Config const &_cfg);

  ~MeasureSpeed();

  /// execute this function on a tacho event
  void on_tacho_event();

  /// get current speed
  float get_speed_m_per_s() const;

  /// get time delta between the latest tacho events
  std::chrono::microseconds get_latest_delta_t() const;

 private:
  constexpr static inline int tacho_input_pin = 15;
  constexpr static inline int tacho_input_pin_mask = 1 << tacho_input_pin;
  constexpr static Config measure_cfg = {
      .wheel_circumference_m = 0.1,
      .timer_cfg =
          {
              .clk_src = GPTIMER_CLK_SRC_DEFAULT,
              .direction = GPTIMER_COUNT_UP,
              .resolution_hz = 1'000'000,  // 1MHz, 1 tick=1us
          },
      .tacho_pin =
          {
              .pin_bit_mask = tacho_input_pin_mask,
              .mode = GPIO_MODE_INPUT,
              .pull_up_en = GPIO_PULLUP_DISABLE,
              .pull_down_en = GPIO_PULLDOWN_ENABLE,
              .intr_type = GPIO_INTR_POSEDGE,
          },
  };

  Config cfg;
  gptimer_handle_t timer_handle;
  float speed_m_per_s = 0;
  std::chrono::microseconds delta_t_measurements = 0s;
  std::chrono::microseconds timestamp_latest_tacho_event = 0s;

  /// get the most recent timestamp from counter
  std::chrono::microseconds get_current_time();
};

}  // namespace lok
