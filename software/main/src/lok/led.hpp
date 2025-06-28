/// @file led.hpp
/// @brief led driver using led controller hw accelerator

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "led.hpp"
#include "sdkconfig.h"
#include "soc/gpio_num.h"

using namespace std::chrono_literals;

namespace led {

using Milliseconds = std::chrono::duration<uint32_t, std::milli>;

namespace color {

struct Color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

constexpr Color off = {.red = 0, .green = 0, .blue = 0};
constexpr Color red = {.red = 255, .green = 0, .blue = 0};
constexpr Color yellow = {.red = 255, .green = 255, .blue = 0};
constexpr Color green = {.red = 0, .green = 255, .blue = 0};
constexpr Color cyan = {.red = 0, .green = 255, .blue = 255};
constexpr Color blue = {.red = 0, .green = 0, .blue = 255};
constexpr Color magenta = {.red = 255, .green = 0, .blue = 255};
constexpr Color orange = {.red = 255, .green = 128, .blue = 0};
constexpr Color rose = {.red = 255, .green = 0, .blue = 128};
constexpr Color purple = {.red = 178, .green = 102, .blue = 255};
constexpr Color white = {.red = 255, .green = 255, .blue = 255};

}  // namespace color

template <uint8_t num_of_rgb_leds>
class Led {
 public:
  struct LedPins {
    gpio_num_t red_pin;
    ledc_channel_t red_channel;
    gpio_num_t green_pin;
    ledc_channel_t green_channel;
    gpio_num_t blue_pin;
    ledc_channel_t blue_channel;
  };

  struct Config {
    std::array<LedPins, num_of_rgb_leds> pins;
    ledc_timer_t timer;
    ledc_mode_t timer_mode;
    ledc_timer_bit_t timer_resolution;
    uint32_t timer_frequency;
  };

  Led(Config const& _cfg);

  void set_color(color::Color color, uint8_t led_index, Milliseconds fade_time = 1s);

 private:
  struct Duty {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
  };

  Duty convert_color_to_duty(color::Color const& color);

  Config cfg;

  void led_fade(uint8_t led_index, color::Color to, Milliseconds duration);

  void config_fade(uint8_t index, Duty to, Milliseconds duration);

  void fade_start(uint8_t index);
};

// Define some colors R, G, B channel PWM duty cycles
template <uint8_t num_of_rgb_leds>

Led<num_of_rgb_leds>::Duty Led<num_of_rgb_leds>::convert_color_to_duty(color::Color const& color) {
  uint32_t const resolution = 1 << cfg.timer_resolution;
  Duty const d = {
      .red = color.red * resolution / 256,
      .green = color.green * resolution / 256,
      .blue = color.blue * resolution / 256,
  };
  return d;
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::fade_start(uint8_t index) {
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).red_channel, LEDC_FADE_NO_WAIT);
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).green_channel, LEDC_FADE_NO_WAIT);
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).blue_channel, LEDC_FADE_NO_WAIT);
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::config_fade(uint8_t index, Duty to, Milliseconds duration) {
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(index).red_channel, to.red, duration.count());
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(index).green_channel, to.green,
                          duration.count());
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(index).blue_channel, to.blue,
                          duration.count());
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::led_fade(uint8_t led_index, color::Color to, Milliseconds duration) {
  auto const duty_to = convert_color_to_duty(to);

  config_fade(led_index, duty_to, duration);
  fade_start(led_index);
  vTaskDelay(pdMS_TO_TICKS(duration.count()));
}

template <uint8_t num_of_rgb_leds>
Led<num_of_rgb_leds>::Led(Config const& _cfg) : cfg{_cfg} {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode = cfg.timer_mode,
      .duty_resolution = cfg.timer_resolution,
      .timer_num = cfg.timer,
      .freq_hz = cfg.timer_frequency,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ledc_timer_config(&ledc_timer);

  // Prepare and then apply the LEDC PWM configuration to the six channels
  ledc_channel_config_t ledc_channel = {
      .speed_mode = cfg.timer_mode,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = cfg.timer,
      .duty = 0,
      .hpoint = 0,
  };

  for (auto const& pin : cfg.pins) {
    ledc_channel.channel = pin.red_channel;
    ledc_channel.gpio_num = pin.red_pin;
    ledc_channel_config(&ledc_channel);

    ledc_channel.channel = pin.green_channel;
    ledc_channel.gpio_num = pin.green_pin;
    ledc_channel_config(&ledc_channel);

    ledc_channel.channel = pin.blue_channel;
    ledc_channel.gpio_num = pin.blue_pin;
    ledc_channel_config(&ledc_channel);
  }

  ledc_fade_func_install(0);
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::set_color(color::Color color, uint8_t led_index,
                                     Milliseconds fade_time) {
  led_fade(led_index, color, fade_time);
}

}  // namespace led
