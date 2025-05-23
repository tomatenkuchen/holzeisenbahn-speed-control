#include "ble.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gap.hpp"
#include "gatt_svc.hpp"
#include "hal/gpio_types.h"
#include "heart_rate.hpp"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "led.hpp"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "speed_ctrl.hpp"
#include <stdexcept>
#include <string>

namespace {

constexpr std::string TAG = "main";

ble::Ble *ble_ptr;

void speed_control_task(void *param) {
  SpeedControl speed_control;
  while (true) {

    vTaskDelay(100);
  }

  vTaskDelete(NULL);
}

void ble_nimble_task(void *param) {
  ble::Ble ble("henri-lok", ble::Ble::Antenna::external);
  ble.nimble_host_task();
  vTaskDelete(NULL);
}

} // namespace

extern "C" void app_main() {
  try {

    led::init();

    xTaskCreate(ble_nimble_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(heart_rate_task, "Heart Rate", 4 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE(TAG.c_str(), "error: %s", err_msg.c_str());
  }
}
