#include "driver/ledc.h"
#include "esp_err.h"
#include "hal/ledc_types.h"

static void example_ledc_init(void) {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                    .duty_resolution = LEDC_TIMER_13_BIT,
                                    .timer_num = LEDC_TIMER_0,
                                    .freq_hz = 1000,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .channel = LEDC_CHANNEL_0,
                                        .timer_sel = LEDC_TIMER_0,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = 5,
                                        .duty = 0,
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void) {
  // Set the LEDC peripheral configuration
  example_ledc_init();
  // Set duty to 50%
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 2048));
  // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
