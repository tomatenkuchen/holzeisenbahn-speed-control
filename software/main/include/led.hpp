#pragma once

#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <cstdint>

#define BLINK_GPIO CONFIG_BLINK_GPIO

namespace led {
uint8_t get_state();
void on();
void off();
void init();
} // namespace led
