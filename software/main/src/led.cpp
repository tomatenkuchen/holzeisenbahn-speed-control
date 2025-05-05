#include "driver/gpio.h"
#include "esp_log.h"
#include "led.hpp"
#include "sdkconfig.h"
#include <cstdint>
#include <string>

namespace led {

namespace {
constexpr std::string TAG = "led";
uint8_t led_state;
constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
} // namespace

uint8_t get_state() { return led_state; }

void on() { gpio_set_level(led_gpio, false); }

void off() { gpio_set_level(led_gpio, true); }

void init() {
  ESP_LOGI(TAG.c_str(), "example configured to blink gpio led!");
  gpio_reset_pin(led_gpio);
  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
}
} // namespace led
