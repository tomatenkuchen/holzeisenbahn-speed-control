#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gatt.hpp"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "led.hpp"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <array>
#include <stdexcept>
#include <string>

namespace gatt {
namespace {

constexpr std::string TAG = "gatt";

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

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
const ble_uuid128_t led_chr_uuid =
    BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,
                     0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

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

std::array<ble_gatt_svc_def, 3> const ble_services = {
    heart_rate_service,
    led_service,
    {0},
};

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    throw std::runtime_error("gatt: bad heart rate access");
  }

  // Verify connection handle
  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "characteristic read; conn_handle=%d attr_handle=%d",
             conn_handle, attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(), "characteristic read by nimble stack; attr_handle=%d",
             attr_handle);
  }

  // Verify attribute handle
  if (attr_handle == heart_rate_chr_val_handle) {
    // Update access buffer value
    // heart_rate_chr_val[1] = get_heart_rate();
    int rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val,
                            sizeof(heart_rate_chr_val));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ERR_UNSPECIFIED;
  }

  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "characteristic write; conn_handle=%d attr_handle=%d",
             conn_handle, attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(),
             "characteristic write by nimble stack; attr_handle=%d",
             attr_handle);
  }

  // Verify attribute handle
  if (attr_handle != led_chr_val_handle) {
    throw std::runtime_error(
        "unexpected access operation on led characteristic");
  }

  // Verify access buffer length
  if (ctxt->om->om_len != 1) {
    throw std::runtime_error(
        "unexpected access operation on led characteristic");
  }

  /* Turn the LED on or off according to the operation bit */
  if (ctxt->om->om_data[0]) {
    led::on();
    ESP_LOGI(TAG.c_str(), "led turned on!");
  } else {
    led::off();
    ESP_LOGI(TAG.c_str(), "led turned off!");
  }

  return 0;
}

} // namespace

void send_heart_rate_indication() {
  if (heart_rate_ind_status && heart_rate_chr_conn_handle_inited) {
    ble_gatts_indicate(heart_rate_chr_conn_handle, heart_rate_chr_val_handle);
    ESP_LOGI(TAG.c_str(), "heart rate indication sent!");
  }
}

void Gatt::service_register_event(ble_gatt_register_ctxt *ctxt) {
  char buf[BLE_UUID_STR_LEN];
  ESP_LOGD(TAG.c_str(), "registered service %s with handle=%d",
           ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
}

void Gatt::characteristic_register_event(ble_gatt_register_ctxt *ctxt) {
  char buf[BLE_UUID_STR_LEN];
  ESP_LOGD(TAG.c_str(),
           "registering characteristic %s with "
           "def_handle=%d val_handle=%d",
           ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle,
           ctxt->chr.val_handle);
}

void Gatt::descriptor_register_event(ble_gatt_register_ctxt *ctxt) {
  ESP_LOGD(TAG.c_str(), "registering descriptor %s with handle=%d",
           ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
}

void Gatt::service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg) {
  char buf[BLE_UUID_STR_LEN];

  /* Handle GATT attributes register events */
  switch (ctxt->op) {

  case BLE_GATT_REGISTER_OP_SVC:
    service_register_event(ctxt);
    return;

  case BLE_GATT_REGISTER_OP_CHR:
    characteristic_register_event(ctxt);
    return;

  case BLE_GATT_REGISTER_OP_DSC:
    descriptor_register_event(ctxt);
    return;

  default:
    return;
  }
}

void gatt_svr_subscribe_cb(ble_gap_event *event) {
  /* Check connection handle */
  if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "subscribe event; conn_handle=%d attr_handle=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(), "subscribe by nimble stack; attr_handle=%d",
             event->subscribe.attr_handle);
  }

  /* Check attribute handle */
  if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
    /* Update heart rate subscription status */
    heart_rate_chr_conn_handle = event->subscribe.conn_handle;
    heart_rate_chr_conn_handle_inited = true;
    heart_rate_ind_status = event->subscribe.cur_indicate;
  }
}

Gatt::Gatt() {
  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(ble_services.data()) != 0) {
    throw std::runtime_error("gatt service counter update failed");
  }

  if (ble_gatts_add_svcs(ble_services.data()) != 0) {
    throw std::runtime_error("gatt service inclusion failed");
  }
}

} // namespace gatt
