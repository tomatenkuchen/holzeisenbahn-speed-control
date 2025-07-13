#include <stdexcept>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "hal/mcpwm_hal.h"
#include "hal/mcpwm_ll.h"
#include "hal/mcpwm_types.h"
#include "inverter.hpp"

namespace lok {

Inverter::Inverter(Config const& config) {
  init_pwm(config.pwm);
  init_adc(config.adc);
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

void Inverter::set_voltages(std::array<MilliVolts, 3> voltages_mV) {
  auto const v_bat_mV = get_battery_voltage();
  for (int i = 0; i < voltages_mV.size(); i++) {
    uint16_t duty = voltages_mV[i] * inverter_pwm_period / v_bat_mV;
    if (mcpwm_comparator_set_compare_value(comparators[i], duty) != ESP_OK) {
      throw std::runtime_error("inverter: set duty failed");
    }
  }
}

void Inverter::init_pwm(PwmConfig const& config) {
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
    if (mcpwm_generator_set_dead_time(generators[i][0], generators[i][0], &config.dt_config) !=
        ESP_OK) {
      throw std::runtime_error("inverter: failed setting dead time");
    };
    if (mcpwm_generator_set_dead_time(generators[i][0], generators[i][1], &config.inv_dt_config) !=
        ESP_OK) {
      throw std::runtime_error("inverter: Setup inv deadtime failed");
    }
    // reset voltage output to 0
    if (mcpwm_comparator_set_compare_value(comparators[i], 0 != ESP_OK)) {
      throw std::runtime_error("inverter: set duty failed");
    }
  }

  if (mcpwm_timer_register_event_callbacks(timer, config.event, nullptr) != ESP_OK) {
    throw std::runtime_error("inverter: register callbacks failed");
  }

  if (mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
    throw std::runtime_error("inverter: mcpwm timer start failed");
  }
}

void Inverter::init_adc(AdcConfig const& config) {
  adc_oneshot_new_unit(&config.unit, &adc_handle);
  adc_oneshot_config_channel(adc_handle, &config.channel, &config.channel_cfg);
}

MilliVolts Inverter::get_battery_voltage() {
  uint16_t adc_raw;
  adc_oneshot_read(adc_handle, &config.channel, &adc_raw);
  // some calculation
  MilliVolts const res = adc_raw * 1000;
  return res;
}

}  // namespace lok
