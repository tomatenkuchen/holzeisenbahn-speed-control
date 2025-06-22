#include "driver/gpio.h"
#include "led.hpp"

namespace led {

Led::Led(gpio_num_t pin_no) : pin{pin_no} {
  gpio_reset_pin(pin_no);
  gpio_set_direction(pin_no, GPIO_MODE_OUTPUT);
}

void Led::on() { gpio_set_level(pin, false); }

void Led::off() { gpio_set_level(pin, true); }

}  // namespace led
