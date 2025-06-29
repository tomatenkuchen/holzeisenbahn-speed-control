/// @file inverter.hpp
/// @brief hardware interface for voltage output
/// @license GNU GPL V2.0

#pragma once

#include <array>

#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_timer.h"
#include "hal/mcpwm_types.h"

namespace lok {
using inverter_handle_t = struct mcpwm_svpwm_ctx *;

using Duty = uint16_t;

class Inverter {
 public:
  struct Config {
    mcpwm_timer_config_t timer_config;         // pwm timer and timing config
    mcpwm_operator_config_t operator_config;   // mcpwm operator config
    mcpwm_comparator_config_t compare_config;  // mcpwm comparator config
    int gen_gpios[3][2];                       // 6 GPIO pins for generator config
    mcpwm_dead_time_config_t dt_config;        // dead time config for positive pwm output
    mcpwm_dead_time_config_t inv_dt_config;    // dead time config for negative pwm output
    mcpwm_timer_event_callbacks_t *event;
  };

  Inverter(Config const &Config);
  ~Inverter();

  /**
   * @brief set 3 channels pwm comparator value for invertor
   *
   * @param handle  svpwm invertor handler
   * @param u comparator value for channel UH and UL
   * @param v comparator value for channel VH and VL
   * @param w comparator value for channel WH and WL
   *
   * @return  - ESP_OK: set compare value successfully
   *          - ESP_ERR_INVALID_ARG: NULL arguments
   */
  esp_err_t set_duty(std::array<Duty, 3> duties);

  /**
   * @brief start/stop a svpwm invertor
   *
   * @param command   see "mcpwm_timer_start_stop_cmd_t"
   *
   */
  void start(mcpwm_timer_start_stop_cmd_t command);

 private:
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t operators[3];
  mcpwm_cmpr_handle_t comparators[3];
  mcpwm_gen_handle_t generators[3][2];
};

}  // namespace lok
