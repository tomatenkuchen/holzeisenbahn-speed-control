#include "led.hpp"

extern "C" {
#include "driver/gpio.h"
}

namespace {
bool led_state;
gpio_num_t constexpr led_blink_gpio = static_cast<gpio_num_t>(8);
} // namespace

bool get_led_state() { return led_state; }

void led_on() {
  led_state = true;
  gpio_set_level(led_blink_gpio, true);
}

void led_off() {
  led_state = false;
  gpio_set_level(led_blink_gpio, false);
}

void led_init() {
  gpio_reset_pin(led_blink_gpio);
  gpio_set_direction(led_blink_gpio, GPIO_MODE_OUTPUT);
}
