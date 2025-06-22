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

constexpr std::string TAG = "main";

extern "C" void ble_store_config_init();

ble::Ble *ble_ptr;
led::Led *led_ptr;

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                   void *arg);

/* Automation IO service */
const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);

uint16_t led_chr_val_handle;

const ble_uuid128_t led_chr_uuid = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
                                                    0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

ble_gatt_chr_def const led_characteristic = {
    .uuid = &led_chr_uuid.u,
    .access_cb = led_chr_access,
    .flags = BLE_GATT_CHR_F_WRITE,
    .val_handle = &led_chr_val_handle,
};

std::array<ble_gatt_chr_def, 2> led_characteristics = {
    led_characteristic,
    {0},
};

ble_gatt_svc_def const led_service = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &auto_io_svc_uuid.u,
    .characteristics = led_characteristics.data(),
};

std::array<ble_gatt_svc_def, 2> ble_services = {
    led_service,
    {0},
};

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                   void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ERR_UNSPECIFIED;
  }

  // Verify attribute handle
  if (attr_handle != led_chr_val_handle) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  // Verify access buffer length
  if (ctxt->om->om_len != 1) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  /* Turn the LED on or off according to the operation bit */
  if (ctxt->om->om_data[0]) {
    led_ptr->on();
  } else {
    led_ptr->off();
  }

  return 0;
}

void speed_control_task(void *param) {
  drive::SpeedControl speed_control;
  while (true) {
    vTaskDelay(100);
  }
  vTaskDelete(NULL);
}

/// callback routine for gap event servicing
/// @param event type of event that occured
/// @param args additional info besides event data. not used by any callback but
/// required by callback type
int event_handler(ble_gap_event *event, void *args) {
  ble_ptr->event_handler(event);

  return 0;
}

void on_stack_reset(int reason) { ESP_LOGI("main", "ble stack reset"); }

void on_stack_sync() { ble_ptr->start_advertising(); }

void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg) {
  ESP_LOGI(TAG.c_str(), "gatt service register callback called");
}

void add_callbacks() {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = service_register_callback;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

void ble_nimble_task(void *param) {
  ble::Ble ble("henri-lok", event_handler, ble_services.data(), ble::Ble::Antenna::external);
  ble_ptr = &ble;

  add_callbacks();

  ble.nimble_host_task();
  vTaskDelete(NULL);
}

}  // namespace

extern "C" void app_main() {
  try {
    constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
    led::Led Led(led_gpio);
    led_ptr = &Led;

    xTaskCreate(ble_nimble_task, "NimBLE Host", 8 * 1024, NULL, 5, NULL);
    //    xTaskCreate(speed_control_task, "Heart Rate", 4 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE("main", "error: %s", err_msg.c_str());
  }
}
