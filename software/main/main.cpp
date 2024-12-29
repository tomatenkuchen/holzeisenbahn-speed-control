#include "ble.hpp"
#include "heart_rate.hpp"
#include "led.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include <stdexcept>
#include <string>

extern "C" {
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}

int32_t speed_ref = 0;
ble::BLE *ble_ptr;

namespace {

ble::GATT::Service<ble_uuid16_t, 2> heart_rate{
    .uuid =
        {
            .u =
                {
                    .type = BLE_UUID_TYPE_16,
                },
            .value = 0x180D,
        },
    .is_indicated = false,
    .characteristic =
        {
            .uuid =
                {
                    .u =
                        {
                            .type = BLE_UUID_TYPE_16,
                        },
                    .value = 0x2A37,
                },
            .value = {0},
            .value_handle = 0,
            .connection_handle_id = 0,
            .is_connection_handle_initialized = false,
        },
};

ble::GATT::Service<ble_uuid16_t, 2> led = {
    .uuid =
        {
            .u =
                {
                    .type = BLE_UUID_TYPE_16,
                },
            .value = 0x1815,
        },
    .is_indicated = false,
    .characteristic =
        {
            .uuid =
                {
                    .u =
                        {
                            .type = BLE_UUID_TYPE_16,
                        },
                    .value = 0x1234,
                },
            .value = {0},
            .value_handle = 0,
            .connection_handle_id = 0,
            .is_connection_handle_initialized = false,
        },
};

int32_t get_motor_speed() { return 0; }

void nimble_host_task(void *param) {
  ESP_LOGI("main", "nimble host task started");
  ble_ptr->nimble_host_task();
  vTaskDelete(nullptr);
}

void heart_rate_task(void *param) {
  ESP_LOGI("main", "heart rate task started");

  while (true) {
    update_heart_rate();
    ESP_LOGI("GATT-Server", "heart rate updated to %d", get_heart_rate());
    ble_ptr->send_indication(heart_rate);
    vTaskDelay(100);
  }

  vTaskDelete(nullptr);
}

void motor_task(void *param) {
  Pwm::Config cfg = {
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

  Pwm pwm(cfg);
  Pid<int32_t> speed_control(1000);

  while (true) {
    auto speed_is = get_motor_speed();
    auto speed_error = speed_ref - speed_is;
    auto duty_new = speed_control.update(speed_error);
    pwm.set_duty(duty_new);
    vTaskDelay(100);
  }
  vTaskDelete(nullptr);
}

} // namespace

extern "C" void app_main() {
  try {
    ble::BLE ble("tomato-ble");
    ble_ptr = &ble;

    led_init();

    xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, nullptr, 5, nullptr);
    xTaskCreate(heart_rate_task, "Heart Rate", 4 * 1024, nullptr, 5, nullptr);
    xTaskCreate(motor_task, "Motor Control", 4 * 1024, nullptr, 5, nullptr);

  } catch (std::runtime_error &e) {
    ESP_LOGE("main", "error occured: %s", e.what());
  }
  return;
}
