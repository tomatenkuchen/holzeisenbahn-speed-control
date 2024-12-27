#include "gatt.hpp"
#include "heart_rate.hpp"
#include "host/ble_uuid.h"
#include "led.hpp"
#include <array>
#include <cstdint>
#include <stdexcept>

extern "C" {
#include "esp_log.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
}

namespace ble::gatt {
namespace {

int heart_rate_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt,
                                     void *arg);

int led_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

template <typename uuid_length, uint8_t N> struct Characteristic {
  uuid_length uuid;
  std::array<uint8_t, N> value = {0};
  uint16_t value_handle;
  uint16_t connection_handle_id;
  bool is_connection_handle_initialized;
};
template <typename uuid_length, uint8_t N> struct Service {
  uuid_length uuid;
  bool is_indicated;
  Characteristic<uuid_length, N> characteristic;
};

Service<ble_uuid16_t, 2> heart_rate{
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

Service<ble_uuid16_t, 2> led = {
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

const ble_gatt_chr_def heart_rate_characteristic = {
    .uuid = &heart_rate.characteristic.uuid.u,
    .access_cb = heart_rate_characteristic_access,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    .val_handle = &heart_rate.characteristic.value_handle,
};

const ble_gatt_chr_def led_characteristic = {
    .uuid = &led.characteristic.uuid.u,
    .access_cb = led_characteristic_access,
    .flags = BLE_GATT_CHR_F_WRITE,
    .val_handle = &led.characteristic.value_handle,
};

std::array<ble_gatt_chr_def, 2> led_characteristic_array = {
    led_characteristic, ble_gatt_chr_def{0}};
std::array<ble_gatt_chr_def, 2> heart_rate_characteristic_array = {
    heart_rate_characteristic, ble_gatt_chr_def{0}};

const struct ble_gatt_svc_def heart_rate_service_cfg = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &heart_rate.uuid.u,
    .characteristics = heart_rate_characteristic_array.data(),
};

const struct ble_gatt_svc_def led_service_cfg = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &led.uuid.u,
    .characteristics = led_characteristic_array.data(),
};

const std::array<ble_gatt_svc_def, 3> gatt_services = {{
    heart_rate_service_cfg,
    led_service_cfg,
    {0},
}};

int heart_rate_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                                     ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ESP_LOGI("GATT-Server",
               "characteristic read; conn_handle=%d attr_handle=%d",
               conn_handle, attr_handle);
    } else {
      ESP_LOGI("GATT-Server",
               "characteristic read by nimble stack; attr_handle=%d",
               attr_handle);
    }

    // Verify attribute handle
    if (attr_handle == heart_rate.characteristic.value_handle) {
      // Update access buffer value
      heart_rate.characteristic.value[1] = get_heart_rate();
      return os_mbuf_append(ctxt->om, &heart_rate.characteristic.value,
                            sizeof(heart_rate.characteristic.value));
    }
  } else {
    throw std::runtime_error(
        "unexpected access operation to heart rate characteristic");
  }
  return -1;
}

int led_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ESP_LOGI("GATT-Server",
               "characteristic write; conn_handle=%d attr_handle=%d",
               conn_handle, attr_handle);
    } else {
      ESP_LOGI("GATT-Server",
               "characteristic write by nimble stack; attr_handle=%d",
               attr_handle);
    }

    /* Verify attribute handle */
    if (attr_handle == led.characteristic.value_handle) {
      /* Verify access buffer length */
      if (ctxt->om->om_len == 1) {
        /* Turn the LED on or off according to the operation bit */
        if (ctxt->om->om_data[0]) {
          led_on();
          ESP_LOGI("GATT-Server", "led turned on!");
        } else {
          led_off();
          ESP_LOGI("GATT-Server", "led turned off!");
        }
      } else {
        throw std::runtime_error("unexpected access to led characteristic");
      }
    }

    return 0;
  } else {
    throw std::runtime_error("unexpected access to led characteristic");
  }
}
} // namespace

void send_heart_rate_indication() {
  if (heart_rate.is_indicated &&
      heart_rate.characteristic.is_connection_handle_initialized) {
    ble_gatts_indicate(heart_rate.characteristic.connection_handle_id,
                       heart_rate.characteristic.value_handle);
    ESP_LOGI("GATT-Server", "heart rate indication sent!");
  }
}

void service_subscribe_cb(ble_gap_event *event) {
  if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI("GATT-Server", "subscribe event; conn_handle=%d attr_handle=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle);
  } else {
    ESP_LOGI("GATT-Server", "subscribe by nimble stack; attr_handle=%d",
             event->subscribe.attr_handle);
  }

  if (event->subscribe.attr_handle == heart_rate.characteristic.value_handle) {
    heart_rate.characteristic.connection_handle_id =
        event->subscribe.conn_handle;
    heart_rate.characteristic.is_connection_handle_initialized = true;
    heart_rate.is_indicated = event->subscribe.cur_indicate;
  }
}

/**
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event */
void service_register_cb(ble_gatt_register_ctxt *ctxt, void *arg) {
  char buf[BLE_UUID_STR_LEN];

  switch (ctxt->op) {
  case BLE_GATT_REGISTER_OP_SVC:
    /* Service register event */
    ESP_LOGD("GATT-Server", "registered service %s with handle=%d",
             ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
    break;

  case BLE_GATT_REGISTER_OP_CHR:
    /* Characteristic register event */
    ESP_LOGD("GATT-Server",
             "registering characteristic %s with def_handle=%d val_handle=%d",
             ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
             ctxt->chr.def_handle, ctxt->chr.val_handle);
    break;

  case BLE_GATT_REGISTER_OP_DSC:
    /* Descriptor register event */
    ESP_LOGD("GATT-Server", "registering descriptor %s with handle=%d",
             ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
    break;

  default:
    throw std::runtime_error("unknown attribute registered");
  }
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server */
void service_init() {

  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(gatt_services.data())) {
    std::runtime_error("gatt counter error");
  }

  if (ble_gatts_add_svcs(gatt_services.data())) {
    std::runtime_error("gatt add service failed");
  }
}
} // namespace ble::gatt
