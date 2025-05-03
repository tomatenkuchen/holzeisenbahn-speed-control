#include "ble.hpp"
#include "common.hpp"
#include "driver/gpio.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gap.hpp"
#include "gatt_svc.hpp"
#include "hal/gpio_types.h"
#include "heart_rate.hpp"
#include "led.hpp"
#include <stdexcept>
#include <string>

Ble *ble_ptr;

void heart_rate_task(void *param) {
  ESP_LOGI(TAG, "heart rate task has been started!");

  while (true) {
    update_heart_rate();
    send_heart_rate_indication();
    ESP_LOGI(TAG, "heart rate updated to %d", get_heart_rate());
    vTaskDelay(100);
  }

  vTaskDelete(NULL);
}

void ble_nimble_task(void *param) { ble_ptr->nimble_host_task(); }

extern "C" void app_main() {
  try {

    led::init();

    Ble ble("esp-gatt-c++", Ble::Antenna::external);
    ble_ptr = &ble;

    xTaskCreate(ble_nimble_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(heart_rate_task, "Heart Rate", 4 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE("main", "error: %s", err_msg.c_str());
  }
}
