#include "ble.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gap.hpp"
#include "gatt_svc.hpp"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "led.hpp"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdexcept>
#include <string>

extern "C" void ble_store_config_init();

namespace {

constexpr std::string TAG = "ble";

void on_stack_reset(int reason);
void on_stack_sync();
void on_stack_reset(int reason) {
  /* On reset, print reset reason to console */
  ESP_LOGI(TAG.c_str(), "nimble stack reset, reset reason: %d", reason);
}

void on_stack_sync() {
  /* On stack sync, do advertising initialization */
  adv_init();
}

} // namespace

Ble::Ble(std::string device_name, Antenna antenna) {
  choose_antenna(antenna);
  init_nvs();
  init_nimble_port();
  gap_init(device_name);
  gatt_svc_init();
  nimble_host_config_init();
}

void Ble::nimble_host_task() {
  nimble_port_run();
  vTaskDelete(NULL);
}

/// @brief control advertizing
/// @param enable true -> start, false -> stop
void Ble::advertize(bool enable) {
  if (enable) {
    ESP_LOGI(TAG.c_str(), "start advertizing");
  } else {
    ESP_LOGI(TAG.c_str(), "stop advertizing");
  }
}

void Ble::init_nvs() {

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    throw std::runtime_error("nvs init failed");
  }
}

void Ble::init_nimble_port() {
  if (nimble_port_init() != ESP_OK) {
    throw std::runtime_error("nimble port init failed");
  }
}

void Ble::nimble_host_config_init() {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

void Ble::choose_antenna(Antenna antenna) {
  // enable rf switch
  gpio_reset_pin(rf_switch_gpio);
  gpio_set_direction(rf_switch_gpio, GPIO_MODE_OUTPUT);
  gpio_set_level(rf_switch_gpio, false);

  // enable switch pin
  gpio_reset_pin(antenna_switch_gpio);
  gpio_set_direction(antenna_switch_gpio, GPIO_MODE_OUTPUT);

  gpio_set_level(antenna_switch_gpio, antenna == Antenna::external);
}
