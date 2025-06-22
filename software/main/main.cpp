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

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                          void *arg);

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                   void *arg);

const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x180D);

uint8_t heart_rate_chr_val[2] = {0};
uint16_t heart_rate_chr_val_handle;
const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x2A37);

uint16_t heart_rate_chr_conn_handle = 0;
bool heart_rate_chr_conn_handle_inited = false;
bool heart_rate_ind_status = false;

/* Automation IO service */
const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);

uint16_t led_chr_val_handle;

const ble_uuid128_t led_chr_uuid = BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
                                                    0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

ble_gatt_chr_def const heart_rate_characteristic = {
    .uuid = &heart_rate_chr_uuid.u,
    .access_cb = heart_rate_chr_access,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    .val_handle = &heart_rate_chr_val_handle,
};

std::array<ble_gatt_chr_def, 2> heart_rate_characteristics = {
    heart_rate_characteristic,
    {0},
};

ble_gatt_svc_def heart_rate_service = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &heart_rate_svc_uuid.u,
    .characteristics = heart_rate_characteristics.data(),
};

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

std::array<ble_gatt_svc_def, 3> ble_services = {
    heart_rate_service,
    led_service,
    {0},
};

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                          void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    throw std::runtime_error("gatt: bad heart rate access");
  }

  // Verify connection handle
  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "characteristic read; conn_handle=%d attr_handle=%d", conn_handle,
             attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(), "characteristic read by nimble stack; attr_handle=%d", attr_handle);
  }

  // Verify attribute handle
  if (attr_handle == heart_rate_chr_val_handle) {
    // Update access buffer value
    // heart_rate_chr_val[1] = get_heart_rate();
    int rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val, sizeof(heart_rate_chr_val));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

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

void update_heartrate_subscription_state(ble_gap_event const *const event) {
  heart_rate_chr_conn_handle = event->subscribe.conn_handle;
  heart_rate_chr_conn_handle_inited = true;
  heart_rate_ind_status = event->subscribe.cur_indicate;
}

void service_subscribe_callback(ble_gap_event *event) {
  // Check attribute handle
  if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
    update_heartrate_subscription_state(event);
  }
}

/// callback routine for gap event servicing
/// @param event type of event that occured
/// @param args additional info besides event data. not used by any callback but
/// required by callback type
int event_handler(ble_gap_event *event, void *args) {
  ble_ptr->event_handler(event);

  switch (event->type) {
    case BLE_GAP_EVENT_SUBSCRIBE:
      service_subscribe_callback(event);
  }
  return 0;
}

void ble_nimble_task(void *param) {
  ble::Ble ble("henri-lok", event_handler, ble_services.data(), ble::Ble::Antenna::external);
  ble_ptr = &ble;

  ble.nimble_host_task();
  vTaskDelete(NULL);
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

}  // namespace

extern "C" void app_main() {
  try {
    constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
    led::Led Led(led_gpio);
    led_ptr = &Led;

    add_callbacks();

    xTaskCreate(ble_nimble_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(speed_control_task, "Heart Rate", 4 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE("main", "error: %s", err_msg.c_str());
  }
}
