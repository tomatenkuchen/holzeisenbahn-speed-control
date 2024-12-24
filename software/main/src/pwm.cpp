#include "pwm.hpp"

extern "C" {
#include "driver/ledc.h"
}

Pwm::Pwm(Config const &_cfg) : cfg{_cfg} {
  ledc_timer_config(&cfg.timer);
  ledc_channel_config(&cfg.channel);
}

Pwm::~Pwm() { set_duty(0); }

void Pwm::set_duty(uint16_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                duty >> (16 - cfg.timer.duty_resolution));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
