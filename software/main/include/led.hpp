#pragma once
#include <array>
#include <chrono>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"
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

}  // namespace led
