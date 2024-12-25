#include "gap.hpp"
#include "gatt_svc.hpp"
#include "heart_rate.hpp"
#include "led.hpp"
#include "pid.hpp"
#include "pwm.hpp"

#include <stdexcept>
#include <string>

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void ble_store_config_init(void);
}

int32_t speed_ref = 0;

namespace {

int32_t get_motor_speed() { return 0; }

/**
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller */
void on_stack_reset(int reason) {
  ESP_LOGI("GATT-Server", "nimble stack reset, reset reason: %d", reason);
}

void on_stack_sync(void) { adv_init(); }

void nimble_host_config_init(void) {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

void nimble_host_task(void *param) {
  ESP_LOGI("GATT-Server", "nimble host task has been started!");

  nimble_port_run();

  vTaskDelete(nullptr);
}

void heart_rate_task(void *param) {
  ESP_LOGI("GATT-Server", "heart rate task has been started!");

  while (true) {
    update_heart_rate();
    ESP_LOGI("GATT-Server", "heart rate updated to %d", get_heart_rate());
    send_heart_rate_indication();
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

void prepare_nvm() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    throw std::runtime_error(std::string("failed to initialize nvs flash"));
  }
}

void init_nimble_host() {
  if (nimble_port_init()) {
    throw std::runtime_error(std::string("failed to initialize nimble stack"));
  }
}

} // namespace

extern "C" void app_main() {
  try {
    led_init();
    prepare_nvm();
    init_nimble_host();
    gap_init();
    gatt_svc_init();
    nimble_host_config_init();
    xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, nullptr, 5, nullptr);
    xTaskCreate(heart_rate_task, "Heart Rate", 4 * 1024, nullptr, 5, nullptr);
    xTaskCreate(motor_task, "Motor Control", 4 * 1024, nullptr, 5, nullptr);
  } catch (std::runtime_error &e) {
    ESP_LOGE("main", "error occured: %s", e.what());
  }
  return;
}
