#include <stdexcept>
#include <string>

#include "ble.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_store.h"
#include "led.hpp"
#include "sdkconfig.h"
#include "speed_ctrl.hpp"

namespace {

extern "C" void ble_store_config_init();

void speed_control_task(void *param) {
  drive::SpeedControl speed_control;
  while (true) {
    vTaskDelay(100);
  }
  vTaskDelete(NULL);
}

ble::Ble *ble_ptr;

void ble_nimble_task(void *param) {
  ble::Ble ble("henri-lok", ble::Ble::Antenna::external);
  ble_ptr = &ble;

  ble.nimble_host_task();
  vTaskDelete(NULL);
}

void on_stack_reset(int reason) { ESP_LOGI("main", "ble stack reset"); }

void on_stack_sync() { ble_ptr->advertize(); }

void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg) {
  ble_ptr->service_register_callback(ctxt, arg);
}

void add_callbacks() {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = service_register_callback;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

}  // namespace

extern "C" void app_main() {
  try {
    constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
    led::Led Led(led_gpio);

    add_callbacks();

    xTaskCreate(ble_nimble_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(speed_control_task, "Heart Rate", 4 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE("main", "error: %s", err_msg.c_str());
  }
}
