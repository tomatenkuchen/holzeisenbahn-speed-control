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
#include "lok/pid.hpp"
#include "lok/tacho.hpp"

namespace lok {

class SpeedControl {
  constexpr static inline int EXAMPLE_FOC_PWM_UH_GPIO = 47;
  constexpr static inline int EXAMPLE_FOC_PWM_UL_GPIO = 21;
  constexpr static inline int EXAMPLE_FOC_PWM_VH_GPIO = 14;
  constexpr static inline int EXAMPLE_FOC_PWM_VL_GPIO = 13;
  constexpr static inline int EXAMPLE_FOC_PWM_WH_GPIO = 12;
  constexpr static inline int EXAMPLE_FOC_PWM_WL_GPIO = 11;
  constexpr static inline uint32_t inverter_timer_resolution = 10'000'000;
  constexpr static inline uint32_t inverter_pwm_period = 1000;

  Inverter::Config inverter_cfg = {
      .timer_config =
          {
              .group_id = 0,
              .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
              .resolution_hz = inverter_timer_resolution,
              // UP_DOWN mode will generate center align pwm wave, which can
              // reduce MOSFET switch times on same effect, extend life
              .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
              .period_ticks = inverter_pwm_period,
          },
      .operator_config =
          {
              .group_id = 0,
          },
      .compare_config =
          {
              .flags =
                  {
                      .update_cmp_on_tez = true,
                  },
          },
      .gen_gpios =
          {
              {EXAMPLE_FOC_PWM_UH_GPIO, EXAMPLE_FOC_PWM_UL_GPIO},
              {EXAMPLE_FOC_PWM_VH_GPIO, EXAMPLE_FOC_PWM_VL_GPIO},
              {EXAMPLE_FOC_PWM_WH_GPIO, EXAMPLE_FOC_PWM_WL_GPIO},
          },
      .dt_config =
          {
              .posedge_delay_ticks = 5,
          },
      .inv_dt_config =
          {
              .negedge_delay_ticks = 5,
              .flags =
                  {
                      .invert_output = true,
                  },
          },
  };

  constexpr static MeasureSpeed::Config measure_cfg = {
      .wheel_circumference_m = 0.1,
      .timer_cfg =
          {
              .clk_src = GPTIMER_CLK_SRC_DEFAULT,
              .direction = GPTIMER_COUNT_UP,
              .resolution_hz = 1'000'000,  // 1MHz, 1 tick=1us
          },
      .tacho_pin =
          {
              .intr_type = GPIO_INTR_POSEDGE,
              .mode = GPIO_MODE_OUTPUT,
              .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
              .pull_down_en = 0,
              .pull_up_en = 0,
              .pin_bit_mask = GPIO_INPUT_PIN_SEL,
              .mode = GPIO_MODE_INPUT,
              .pull_up_en = 1,
          },
  };

  constexpr static sig::PIDController<float>::Config pid_cfg = {
      .amp_i = 1.,
      .amp_p = 2.,
      .amp_d = 0.,
      .limit_max = 4.3,
      .limit_min = -4.3,
  };

 public:
  SpeedControl();
  ~SpeedControl();

  void set_ref_speed_m_per_s(float speed_m_per_s);

  void on_tacho_event();

 private:
  MeasureSpeed measure;
  Inverter inverter;
  sig::PIDController<float> pid;
  float speed_ref_m_per_s;
};

}  // namespace lok
