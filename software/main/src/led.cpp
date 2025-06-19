#include "driver/gpio.h"
#include "led.hpp"

namespace led {

void Led::on() { gpio_set_level(pin, false); }

void Led::off() { gpio_set_level(pin, true); }

Led::Led(gpio_num_t pin_no) : pin{pin_no} {
  gpio_reset_pin(led_gpio);
  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
}
}  // namespace led
