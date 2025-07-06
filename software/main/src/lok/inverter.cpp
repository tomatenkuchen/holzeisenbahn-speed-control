#include <stdexcept>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/mcpwm_types.h"
#include "inverter.hpp"

namespace lok {

Inverter::Inverter(Config const &config) {
  esp_err_t ret;

  if (mcpwm_new_timer(&config.timer_config, &timer) != ESP_OK) {
    throw std::runtime_error("inverter: failed to init pwm module");
  }

  for (int i = 0; i < 3; i++) {
    if (mcpwm_new_operator(&config.operator_config, &operators[i]) != ESP_OK) {
      throw std::runtime_error("inverter: faild to establish operator");
    }
    if (mcpwm_operator_connect_timer(operators[i], timer)) {
      throw std::runtime_error("inverter: Connect operators to the same timer failed");
    }
  }

  for (int i = 0; i < 3; i++) {
    if (mcpwm_new_comparator(operators[i], &config.compare_config, &comparators[i]) != ESP_OK) {
      throw std::runtime_error("inverter: Create comparators failed");
    }
    if (mcpwm_comparator_set_compare_value(comparators[i], 0) != ESP_OK) {
      throw std::runtime_error("inverter: Set comparators failed");
    }
  }

  mcpwm_generator_config_t gen_config = {};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      gen_config.gen_gpio_num = config.gen_gpios[i][j];
      if (mcpwm_new_generator(operators[i], &gen_config, &generators[i][j]) != ESP_OK) {
        throw std::runtime_error("inverter: Create PWM generator pin failed");
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    if (mcpwm_generator_set_actions_on_compare_event(
            generators[i][0],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i],
                                           MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[i],
                                           MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END()) != ESP_OK) {
      throw std::runtime_error("inverter: Set generator actions failed");
    }
  }

  for (int i = 0; i < 3; i++) {
    if (mcpwm_generator_set_dead_time(generators[i][0], generators[i][0], config.dt_config) !=
        ESP_OK) {
      throw std::runtime_error("inverter: failed setting dead time");
    };
    if (mcpwm_generator_set_dead_time(generators[i][0], generators[i][1], config.inv_dt_config) !=
        ESP_OK) {
      throw std::runtime_error("inverter: Setup inv deadtime failed");
    }
  }

  if (!(handle && event)) {
    throw std::runtime_error("inverter: invalid argument");
  }

  if (mcpwm_timer_register_event_callbacks(timer, event, user_ctx) != ESP_OK) {
    throw std::runtime_error("inverter: register callbacks failed");
  }

  if (mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
    throw std::runtime_error("inverter: mcpwm timer start failed");
  }

  // reset voltage output to 0
  if (mcpwm_comparator_set_compare_value(comparators[i], 0 != ESP_OK) {
    throw std::runtime_error("inverter: set duty failed");
  }

  *ret_inverter = svpwm_dev;
  return ESP_OK;
}

Inverter::~Inverter() {
  mcpwm_timer_disable(timer);
  for (int i = 0; i < 3; i++) {
    mcpwm_del_generator(generators[i][0]);
    mcpwm_del_generator(generators[i][1]);
    mcpwm_del_comparator(comparators[i]);
    mcpwm_del_operator(operators[i]);
  }
  mcpwm_del_timer(timer);
}

void Inverter::set_voltage(std::array<Voltage, 3> voltages) {
  auto const v_vat_mV = get_battery_voltage();
  for (int i = 0; i < voltages.size(); i++) {
    uint16_t duty = voltages[i] * inverter_pwm_period / v_bat_mV;
    if (mcpwm_comparator_set_compare_value(comparators[i], voltages[i]) != ESP_OK) {
      throw std::runtime_error("inverter: set duty failed");
    }
  }
}

}  // namespace lok
