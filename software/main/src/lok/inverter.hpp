/// @file inverter.hpp
/// @brief hardware interface for voltage output
/// @license GNU GPL V2.0

#pragma once

#include <array>

#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/mcpwm_types.h"

namespace lok {
using inverter_handle_t = struct mcpwm_svpwm_ctx *;
using MilliVolts = uint32_t;

class Inverter {
 public:
  constexpr static inline int EXAMPLE_FOC_PWM_UH_GPIO = 47;
  constexpr static inline int EXAMPLE_FOC_PWM_UL_GPIO = 21;
  constexpr static inline int EXAMPLE_FOC_PWM_VH_GPIO = 14;
  constexpr static inline int EXAMPLE_FOC_PWM_VL_GPIO = 13;
  constexpr static inline int EXAMPLE_FOC_PWM_WH_GPIO = 12;
  constexpr static inline int EXAMPLE_FOC_PWM_WL_GPIO = 11;
  constexpr static inline uint32_t inverter_timer_resolution = 10'000'000;
  constexpr static inline uint32_t inverter_pwm_period = 1000;

  struct AdcConfig {
    adc_oneshot_unit_init_cfg_t unit;
    adc_channel_t channel;
    adc_oneshot_chan_cfg_t channel_cfg;
  };

  struct PwmConfig {
    mcpwm_timer_config_t timer_config;         // pwm timer and timing config
    mcpwm_operator_config_t operator_config;   // mcpwm operator config
    mcpwm_comparator_config_t compare_config;  // mcpwm comparator config
    int gen_gpios[3][2];                       // 6 GPIO pins for generator config
    mcpwm_dead_time_config_t dt_config;        // dead time config for positive pwm output
    mcpwm_dead_time_config_t inv_dt_config;    // dead time config for negative pwm output
    mcpwm_timer_event_callbacks_t *event;
  };

  struct Config {
    AdcConfig adc;
    PwmConfig pwm;
  };

  constexpr static inline Inverter::Config inverter_cfg = {
      .adc =
          {
              .unit =
                  {
                      .unit_id = ADC_UNIT_1,
                  },
              .channel =
                  {
                      .atten = ADC_ATTEN_DB_12,
                      .bitwidth = ADC_BITWIDTH_DEFAULT,
                  },
          },
      .pwm =
          {

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
          },
  };

  Inverter(Config const &Config = inverter_cfg);

  ~Inverter();

  /**
   * @brief set 3 channels pwm comparator value for invertor
   *
   * @param handle  svpwm invertor handler
   * @param u comparator value for channel UH and UL
   * @param v comparator value for channel VH and VL
   * @param w comparator value for channel WH and WL
   *
   */
  void set_voltages(std::array<MilliVolts, 3> voltages_mV);

 private:
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t operators[3];
  mcpwm_cmpr_handle_t comparators[3];
  mcpwm_gen_handle_t generators[3][2];

  adc_oneshot_unit_handle_t adc_handle;

  void init_pwm(PwmConfig const &config);
  void init_adc(AdcConfig const &config);
  MilliVolts get_battery_voltage();
};

}  // namespace lok
