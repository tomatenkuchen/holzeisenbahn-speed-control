/// @file speed_ctrl.hpp
/// @brief control speed of train
/// @author tomatenkuchen
/// @copyright GPLv2.0

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "lok/inverter.hpp"
#include "lok/led.hpp"
#include "lok/pid.hpp"
#include "lok/tacho.hpp"

namespace lok {

class SpeedControl {
 public:
  struct Config {
    void (*tacho_pin_callback)(void*);
  };

  SpeedControl(Config const& cfg);

  void set_ref_speed_m_per_s(float speed_m_per_s);

  void on_tacho_event();

 private:
  constexpr static sig::PIDController<float>::Config pid_cfg = {
      .amp_i = 1.,
      .amp_p = 2.,
      .amp_d = 0.,
      .limit_max = 4.3,
      .limit_min = -4.3,
  };

  MeasureSpeed measure;
  Inverter inverter;
  sig::PIDController<float> pid;
  float speed_ref_m_per_s = 0.;
};

}  // namespace lok
