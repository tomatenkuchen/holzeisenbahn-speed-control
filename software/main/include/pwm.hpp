/**
 * @file pwm.hpp
 * @brief pwm controller for motor speed controll
 * @author tomatenkuchen
 * @date 2024-12-29
 * @license GPLv2 @see license.md
 */

#pragma once

#include "driver/ledc.h"

class Pwm {
public:
  struct Config {
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
  };

  Pwm(Config const &_cfg);

  ~Pwm();

  void set_duty(uint16_t duty);

private:
  Config cfg;
};
