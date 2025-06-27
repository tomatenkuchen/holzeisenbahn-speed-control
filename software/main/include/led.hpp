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
    float gamma_factor;
    ledc_timer_t timer;
    ledc_mode_t timer_mode;
    ledc_timer_bit_t timer_resolution;
    uint32_t timer_frequency;
  };

  Led(Config const& _cfg);

  void set_color(color::Color color, uint8_t led_index, Milliseconds fade_time = 1s);

 private:
  uint32_t RGB_TO_DUTY(uint8_t rgb);

  struct Duty {
    uint32_t red;
    uint32_t green;
    uint32_t blue;

    Duty() = default;

    Duty(color::Color c)
        : red{RGB_TO_DUTY(c.red)}, green{RGB_TO_DUTY(c.green)}, blue{RGB_TO_DUTY(c.blue)} {}
  };

  Config cfg;

  color::Color old_color = color::off;

  void led_fade(uint8_t led_index, color::Color from, color::Color to, Milliseconds duration);

  void rgb_set_gamma_curve_fade(uint8_t index, Duty from, Duty to, Milliseconds duration);

  void rgb_set_linear_fade(uint8_t led_index, Duty target, Milliseconds duration);

  void rgb_fade_start(uint8_t index);

  uint32_t gamma_correction_calculator(uint32_t duty);
};

// Define some colors R, G, B channel PWM duty cycles
template <uint8_t num_of_rgb_leds>
uint32_t Led<num_of_rgb_leds>::RGB_TO_DUTY(uint8_t rgb) {
  return rgb * (1 << cfg.timer_resolution) / 255;
}

template <uint8_t num_of_rgb_leds>
uint32_t Led<num_of_rgb_leds>::gamma_correction_calculator(uint32_t duty) {
  uint32_t const resolution = 1 << cfg.timer_resolution;
  float const scaled_duty = static_cast<float>(duty) / resolution;
  uint32_t const gamma = std::pow(scaled_duty, cfg.gamma_factor);
  return gamma * resolution;
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::rgb_fade_start(uint8_t index) {
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).red_channel, LEDC_FADE_NO_WAIT);
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).green_channel, LEDC_FADE_NO_WAIT);
  ledc_fade_start(cfg.timer_mode, cfg.pins.at(index).blue_channel, LEDC_FADE_NO_WAIT);
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::rgb_set_linear_fade(uint8_t led_index, Duty target,
                                               Milliseconds duration) {
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(led_index).red_channel, target.red,
                          duration.count());
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(led_index).green_channel, target.green,
                          duration.count());
  ledc_set_fade_with_time(cfg.timer_mode, cfg.pins.at(led_index).blue_channel, target.blue,
                          duration.count());
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::rgb_set_gamma_curve_fade(uint8_t index, Duty from, Duty to,
                                                    Milliseconds duration) {
  const uint32_t linear_fade_segments = 12;
  uint32_t actual_fade_ranges;
  ledc_fade_param_config_t fade_params_list[SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX] = {};

  ledc_fill_multi_fade_param_list(cfg.timer_mode, cfg.pins.at(index).red_channel, from.red, to.red,
                                  linear_fade_segments, duration.count(),
                                  gamma_correction_calculator, SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX,
                                  fade_params_list, &actual_fade_ranges);
  ledc_set_multi_fade(cfg.timer_mode, cfg.pins.at(index).red_channel,
                      gamma_correction_calculator(from.red), fade_params_list, actual_fade_ranges);

  ledc_fill_multi_fade_param_list(cfg.timer_mode, cfg.pins.at(index).green_channel, from.green,
                                  to.green, linear_fade_segments, duration,
                                  gamma_correction_calculator, SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX,
                                  fade_params_list, &actual_fade_ranges);
  ledc_set_multi_fade(cfg.timer_mode, cfg.pins.at(index).green_channel,
                      gamma_correction_calculator(from.green), fade_params_list,
                      actual_fade_ranges);

  ledc_fill_multi_fade_param_list(cfg.timer_mode, cfg.pins.at(index).blue_channel, from.blue,
                                  to.blue, linear_fade_segments, duration,
                                  gamma_correction_calculator, SOC_LEDC_GAMMA_CURVE_FADE_RANGE_MAX,
                                  fade_params_list, &actual_fade_ranges);
  ledc_set_multi_fade(cfg.timer_mode, cfg.pins.at(index).blue_channel,
                      gamma_correction_calculator(from.blue), fade_params_list, actual_fade_ranges);
}

template <uint8_t num_of_rgb_leds>
void Led<num_of_rgb_leds>::led_fade(uint8_t led_index, color::Color from, color::Color to,
                                    Milliseconds duration) {
  const Duty duty_from(from);
  const Duty duty_to(to);

  rgb_set_gamma_curve_fade(led_index, duty_from, duty_to, duration);
  rgb_fade_start(led_index);
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
  led_fade(led_index, old_color, color, fade_time);
  old_color = color;
}

}  // namespace led
