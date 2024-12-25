#pragma once

#include "driver/ledc.h"
#include "hal/ledc_types.h"

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
