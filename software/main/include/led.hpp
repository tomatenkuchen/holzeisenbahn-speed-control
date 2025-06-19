#pragma once

#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"

namespace led {

class Led {
 public:
  Led(gpio_num_t pin_no);
  ~Led();
  void on();
  void off();

 private:
  gpio_num_t pin;
}

}  // namespace led
