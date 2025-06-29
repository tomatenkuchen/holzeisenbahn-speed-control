/// @file lok.hpp
/// @brief combination class for all functions
/// @license GNU GPL V2

#pragma once

#include "inverter.hpp"
#include "led.hpp"

namespace lok {

class Lok {
 public:
  Lok();
  ~Lok();

  Led<2> lights;
  Inverter inverter;

 private:
  constexpr static inline int EXAMPLE_FOC_PWM_UH_GPIO = 47;
  constexpr static inline int EXAMPLE_FOC_PWM_UL_GPIO = 21;
  constexpr static inline int EXAMPLE_FOC_PWM_VH_GPIO = 14;
  constexpr static inline int EXAMPLE_FOC_PWM_VL_GPIO = 13;
  constexpr static inline int EXAMPLE_FOC_PWM_WH_GPIO = 12;
  constexpr static inline int EXAMPLE_FOC_PWM_WL_GPIO = 11;
  constexpr static inline uint32_t inverter_timer_resolution = 10'000'000;
  constexpr static inline uint32_t inverter_pwm_period = 1000;

  constexpr static inline Led<2>::Config led_config = {
      .pins =
          {
              {
                  {
                      .red_pin = gpio_num_t(0),
                      .red_channel = LEDC_CHANNEL_0,
                      .green_pin = gpio_num_t(1),
                      .green_channel = LEDC_CHANNEL_1,
                      .blue_pin = gpio_num_t(2),
                      .blue_channel = LEDC_CHANNEL_2,
                  },
                  {
                      .red_pin = gpio_num_t(3),
                      .red_channel = LEDC_CHANNEL_3,
                      .green_pin = gpio_num_t(4),
                      .green_channel = LEDC_CHANNEL_4,
                      .blue_pin = gpio_num_t(5),
                      .blue_channel = LEDC_CHANNEL_5,
                  },
              },
          },
      .timer = LEDC_TIMER_0,
      .timer_mode = LEDC_LOW_SPEED_MODE,
      .timer_resolution = LEDC_TIMER_13_BIT,
      .timer_frequency = 4000,
  };

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
};  // namespace lok

Lok::Lok() : lights(led_config), inverter(inverter_cfg) {}
Lok::~Lok() {}

}  // namespace lok
