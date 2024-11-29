#include "driver/ledc.h"
#include "hal/ledc_types.h"

class Led {
public:
  struct Config {
    ledc_timer_config_t timer;
    ledc_channel_config_t channel;
  };

  Led(Config const &_cfg) : cfg{_cfg} {
    ledc_timer_config(&cfg.timer);
    ledc_channel_config(&cfg.channel);
  }

  ~Led() {}

  void set_duty(uint16_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                  duty >> (16 - cfg.timer.duty_resolution));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  }

private:
  Config cfg;
};

extern "C" void app_main() {
  Led::Config cfg = {
      .timer =
          {
              .speed_mode = LEDC_LOW_SPEED_MODE,
              .duty_resolution = LEDC_TIMER_13_BIT,
              .timer_num = LEDC_TIMER_0,
              .freq_hz = 1000,
              .clk_cfg = LEDC_AUTO_CLK,
              .deconfigure = false,
          },
      .channel =
          {
              .gpio_num = 5,
              .speed_mode = LEDC_LOW_SPEED_MODE,
              .channel = LEDC_CHANNEL_0,
              .intr_type = LEDC_INTR_DISABLE,
              .timer_sel = LEDC_TIMER_0,
              .duty = 0,
              .hpoint = 0,
              .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
              .flags = {false},
          },
  };
  Led led(cfg);
  led.set_duty(32768);
}
