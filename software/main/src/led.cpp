#include "common.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led.hpp"
#include <cstdint>

namespace led {

namespace {
uint8_t led_state;
constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
} // namespace

uint8_t get_state() { return led_state; }

void on() { gpio_set_level(led_gpio, true); }

void off() { gpio_set_level(led_gpio, false); }

void init() {
  ESP_LOGI(TAG, "example configured to blink gpio led!");
  gpio_reset_pin(led_gpio);
  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
}
} // namespace led
