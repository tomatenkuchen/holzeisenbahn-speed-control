/// @file speed_ctrl.hpp
/// @brief control speed of train
/// @author tomatenkuchen
/// @copyright GPLv2.0

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "lok/pid.hpp"

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
  };

  MeasureSpeed(Config const &_cfg);

  ~MeasureSpeed();

  /// execute this function on a tacho event
  void on_tacho_event();

  float get_speed_m_per_s() const;

 private:
  Config cfg;
  gptimer_handle_t timer_handle;
  float speed_m_per_s = 0;
  uint64_t timestamp_latest_tacho_event = 0;
};

}  // namespace lok
